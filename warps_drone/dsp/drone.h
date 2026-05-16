// Top-level drone synthesizer with page-based parameter set.
//
// Signal flow per audio block:
//
//   audio IN L ──► KS voices (4, chord) ─┐
//                          ▲             │
//        excite LFO ───────┤             │
//        pink/white noise ─┘             │
//                                        ├── + ──► SVF ──► reverb ──► × gain ──► out
//   audio IN R ─► env follower ─► trig ──┘            │           ▲
//                  (re-plucks the voices)             │           │
//                                                     │   modal bank ──► mixed via harmonics
//                                                     │
//                                  ┌── reverb tail FB ┘ (smear knob)
//                                  ▼
//                           noise floor of voices

#ifndef WARPS_DRONE_DSP_DRONE_H_
#define WARPS_DRONE_DSP_DRONE_H_

#include <math.h>

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"

#include "warps_drone/dsp/parameters.h"
#include "warps_drone/dsp/voice_bank.h"
#include "warps_drone/dsp/modal_bank.h"
#include "warps_drone/dsp/tape_delay.h"
#include "warps_drone/dsp/reverb.h"

namespace warps_drone {

class Drone {
 public:
  Drone() { }

  void Init(uint16_t* reverb_buffer, float sample_rate) {
    sample_rate_      = sample_rate;
    inv_sample_rate_  = 1.0f / sample_rate;
    block_in_seconds_ = 32.0f / sample_rate;

    voices_.Init(sample_rate);
    modal_.Init(sample_rate);
    delay_.Init();
    reverb_.Init(reverb_buffer, sample_rate);
    svf_   .Init();
    svf_hp_.Init();
    for (int s = 0; s < 3; ++s) {
      saw_phase_[s] = static_cast<float>(s) * 0.333f;  // stagger
      saw_inc_[s]   = 0.0f;
    }

    lfo_phase_   = 0.0f;
    env_r_       = 0.0f;
    reverb_send_ = 0.0f;
    ks_lpf_state_ = 0.0f;
    ks_lpf_a_     = 1.0f;
    ks_lpf_bypass_ = true;

    // Sensible first-block defaults (cv_scaler overwrites immediately
    // anyway, but this avoids NaN-y math if the first block races).
    // Matches the cv_scaler default tables - first audio block won't
    // sound wildly different from steady state. Captured live from
    // the module.
    parameters_                    = {};
    parameters_.chord              = 0.49f;
    parameters_.lpf_cutoff         = 0.73f;
    parameters_.pitch              = 0.66f;
    parameters_.reverb_amount      = 0.44f;
    parameters_.chord_mode         = 0.91f;  // SUS4
    parameters_.hpf_cutoff         = 1.00f;  // inverted: 1.0 = open, 0.0 = kill
    parameters_.pitch_octave       = 0.50f;  // octave 3 -> C4 base
    parameters_.filter_resonance   = 0.00f;
    parameters_.burst_shape        = 0.19f;
    parameters_.pluck_shape        = 0.68f;
    parameters_.pluck_amplitude    = 0.03f;
    parameters_.smear              = 0.73f;
    parameters_.exciter_rate       = 0.50f;
    parameters_.damping            = 0.99f;
    parameters_.noise_floor_base   = 0.00f;
    parameters_.reverb_size        = 0.87f;
    parameters_.reverb_diffusion   = 0.76f;
    parameters_.reverb_lp          = 0.40f;
    parameters_.reverb_shimmer     = 0.50f;
    parameters_.reverb_shim_rate   = 0.50f;
    parameters_.reverb_drive       = 0.35f;
    parameters_.harmonics          = 0.40f;
    parameters_.modal_brightness   = 0.60f;
    parameters_.modal_stiffness    = 0.20f;
    parameters_.modal_count        = 0.50f;
    parameters_.modal_pickup       = 0.50f;
    parameters_.karplus_lpf        = 0.85f;  // ~8 kHz - tames burst hiss
  }

  // Re-pluck all KS voices with a shaped noise burst.
  //
  // burst_shape (ALGO unshifted on KARPLUS page) morphs the burst's
  // amplitude envelope between four shapes, clockwise:
  //   0.00  gate         (short fixed 10 ms rect - discrete "click")
  //   0.33  negative saw (decaying ramp, full pluck length)
  //   0.67  triangle     (rise then fall, full pluck length)
  //   1.00  saw          (rising ramp / swell, full pluck length)
  //
  // pluck_shape (PARAM unshifted) sets the burst's full length:
  // 5 ms (sharp tick) up to 600 ms (slow swell). Length is honoured
  // from bs=0.333 upward; below that the length is interpolated down
  // toward a fixed ~10 ms gate so the rect end of the morph reads as a
  // discrete pulse, not as a sustained noise floor.
  // pluck_amplitude (ALGO shifted) sets the burst's peak amplitude.
  inline void Pluck() {
    // Pluck amplitude scaled to be on the same order as the steady-state
    // noise floor (which sits around 0.18 in the loop at high settings),
    // so plucks accent rather than overpower the continuous noise drone.
    const float amp  = parameters_.pluck_amplitude * 0.10f;
    const float bs   = parameters_.burst_shape;
    const float full = 0.005f + 0.595f * parameters_.pluck_shape;
    constexpr float kGateSeconds = 0.010f;
    const float u    = bs < 0.333333f ? bs * 3.0f : 1.0f;
    const float length = kGateSeconds + (full - kGateSeconds) * u;
    const int   n      = static_cast<int>(length * sample_rate_);
    voices_.Excite(amp, n, bs);
    // Saw stack is a continuous drone underneath the plucked bank -
    // it doesn't get re-excited per pluck. Its job is to anchor the
    // bass register, not to add pluck transients.
  }

  DroneParameters* mutable_parameters() { return &parameters_; }

  void Process(FloatFrame* in_out, size_t n) {
    // -----------------------------------------------------------------
    // Pitch: pot continuous, CV quantised chromatic V/oct.
    // -----------------------------------------------------------------
    // Pitch:
    //   pot           = 12 continuous semitones (1 octave) of fine pitch
    //   octave knob   = which octave the pot lives in (0..5 -> C1..C6)
    //   V/oct CV      = chromatic transposition on top (quantised w/ hysteresis)
    // Total semitones from C1 = octave*12 + pot_semitones + cv_quantised
    const float pot_semitones = parameters_.pitch * 12.0f;        // 0..12
    int oct = static_cast<int>(parameters_.pitch_octave * 6.0f);  // 0..5
    if (oct > 5) oct = 5;

    // V/oct CV with hysteresis. parameters_.reserved_a now arrives from
    // CvScaler already converted to semitones (calibrated). Just apply
    // the hysteresis quantiser.
    const float cv_semitones = parameters_.reserved_a;
    const float diff = cv_semitones - static_cast<float>(last_cv_q_);
    const float adiff = diff < 0.0f ? -diff : diff;
    if (adiff > 0.65f) {
      last_cv_q_ = static_cast<int>(roundf(cv_semitones));
    }

    const float total = static_cast<float>(oct) * 12.0f
                      + pot_semitones
                      + static_cast<float>(last_cv_q_);
    // C1 = 32.703 Hz is the lowest octave's base.
    const float root_hz = 32.703f * powf(2.0f, total * (1.0f / 12.0f));

    // -----------------------------------------------------------------
    // Voice bank, KS damping, noise feed.
    // -----------------------------------------------------------------
    voices_.set_chord(root_hz, parameters_.chord, parameters_.chord_mode);
    const float d = parameters_.damping;
    voices_.set_decay(0.990f + 0.010f * d);
    voices_.set_brightness(d);
    voices_.set_noise_color(parameters_.white_pink_mix);

    // Sub-osc - three detuned naive saw oscillators that mirror the
    // chord stack. The voice bank computed ratios for 4 voices; the
    // saws use the first 3 (root + two chord notes / detuned siblings)
    // shifted DOWN one octave so they sit as a bass extension under
    // the main bank. Behaviour by zone:
    //   UNISON  - three saws at root-1oct (very fat unison)
    //   DETUNE  - same three saws with the bank's spread for flutter
    //   +Nth    - root, 2nd chord note, 3rd chord note, all -1 oct
    // A small fixed extra detune (+2¢ / -3¢) is layered on the bottom
    // two saws so the UNISON zone still gets a slow flutter rather
    // than locking to a single phase-coherent tone.
    const float* r = voices_.ratios();
    saw_inc_[0] = root_hz * r[0] * 0.50f                 * inv_sample_rate_;
    saw_inc_[1] = root_hz * r[1] * 0.50f * 1.00116f      * inv_sample_rate_;
    saw_inc_[2] = root_hz * r[2] * 0.50f * 0.99827f      * inv_sample_rate_;

    // Post-saw LPF - one-pole, ~600 Hz. Smooths the saw discontinuities
    // (a touch of warmth) and keeps the sub from leaking HF into the
    // chord band. Coefficient unused-by-decay because cutoff is fixed.
    constexpr float kSubLpfHz = 600.0f;
    const float a_sub = 1.0f - expf(-2.0f * static_cast<float>(M_PI)
                                     * kSubLpfHz * inv_sample_rate_);
    sub_lpf_a_ = a_sub;

    // Exciter LFO: a *trigger generator*. exciter_rate=0 -> off;
    // otherwise fires a Pluck() each cycle at rate_hz. Range 0..30 Hz.
    // Suppressed while an external trigger is active on IN R - patch
    // a clock there and the internal LFO yields automatically.
    const float rate_norm = parameters_.exciter_rate;
    const float rate_hz   = rate_norm * rate_norm * 30.0f;
    // ~4 s timeout @ 1.5 kHz block rate. If no IN R trigger for that
    // long, internal LFO resumes.
    const bool external_active = blocks_since_ext_trig_ < 6000;
    if (rate_hz > 0.01f && !external_active) {
      lfo_phase_ += rate_hz * block_in_seconds_;
      if (lfo_phase_ >= 1.0f) {
        lfo_phase_ -= 1.0f;
        Pluck();
      }
    } else {
      lfo_phase_ = 0.0f;
    }
    if (blocks_since_ext_trig_ < 1000000) ++blocks_since_ext_trig_;

    // Continuous noise floor is driven *only* by noise_floor_base now
    // - the old `strength` slot has been repurposed as the burst-shape
    // envelope morph (see Pluck() above), since strength was musically
    // indistinguishable from noise_floor_base for sustained drone use.
    const float floor_base  = parameters_.noise_floor_base;
    // the reverb-tail smear contribution is *gated by reverb_amount*.
    // gate, the reverb keeps running internally even at
    // wet=0 (only the dry/wet mix is bypassed), and the smear loop
    // would keep pumping noise into the strings - making it impossible
    // to A/B a clean dry chord against the wet drone. With the gate,
    // setting rev wet to 0 fully severs the loop.
    const float rev_a = parameters_.reverb_amount;
    // Noise-floor ceilings opened up so the strings can saturate hard
    // when you wants the "constantly excited" sound. Smear also
    // bumped 4× so it's clearly audible alongside the damping noise.
    const float total_noise_floor =
        floor_base * floor_base * 0.050f                        // primary fuel
      + d * d * 0.012f                                          // dark-side noise
      + reverb_send_ * parameters_.smear * rev_a * 0.100f;      // gated smear (4×)
    voices_.set_noise_floor(total_noise_floor);

    // -----------------------------------------------------------------
    // Modal resonator - locked to stable_v1 defaults until the red
    // SHIMMER page is wired back up. The knobs on page 3 still write
    // to their parameters_ fields, but we ignore them here.
    // -----------------------------------------------------------------
    modal_.set_fundamental(root_hz);
    modal_.set_brightness(0.60f);
    modal_.set_stiffness (0.12f);
    modal_.set_pickup    (0.50f);
    modal_.set_active_modes(16);

    // -----------------------------------------------------------------
    // Filters - independent LPF and HPF in series, applied per-sample
    // BEFORE the reverb so the wash also responds to the filter sweeps.
    //   LPF cutoff: log 60 Hz..12 kHz, PARAM unshifted (+CV)
    //   HPF cutoff: log 16 kHz..20 Hz, PARAM shifted   (inverted!)
    //   Shared resonance (LVL2 shifted) -> 0.7..10 Q applied to both.
    //
    // The HP cutoff knob is inverted so its deadzone sits at full CCW,
    // matching the LPF: both knobs at full CCW = silence, both at full
    // CW = wide open.
    // -----------------------------------------------------------------
    const float fc_lp = 60.0f * powf(200.0f, parameters_.lpf_cutoff);
    const float fc_hp = 20.0f * powf(800.0f, 1.0f - parameters_.hpf_cutoff);

    // KS post-voice LPF - one-pole, log 200 Hz..16 kHz on parameters_
    // .karplus_lpf. Sits between voice/modal sum and the performance
    // filters so it tames the raw K-S hash *before* anything else in
    // the chain (including the reverb input) sees it. Coefficient
    // saturates at 1 as fc -> Nyquist, i.e. fully open at full CW.
    // KS LPF coefficient - bypass entirely when the knob is fully CW so
    // even the one-pole's natural HF rolloff (-3 dB ~5 kHz at fc=Nyquist)
    // is removed. Otherwise compute the standard one-pole a-coefficient.
    if (parameters_.karplus_lpf >= 0.999f) {
      ks_lpf_bypass_ = true;
    } else {
      ks_lpf_bypass_ = false;
      const float fc_ks = 200.0f * powf(80.0f, parameters_.karplus_lpf);
      const float aks   = 1.0f - expf(-2.0f * static_cast<float>(M_PI)
                                       * fc_ks * inv_sample_rate_);
      ks_lpf_a_ = aks > 1.0f ? 1.0f : (aks < 0.0f ? 0.0f : aks);
    }
    const float q_lin = 0.7f + parameters_.filter_resonance * 9.3f;
    svf_   .set_f_q<stmlib::FREQUENCY_FAST>(fc_lp * inv_sample_rate_, q_lin);
    svf_hp_.set_f_q<stmlib::FREQUENCY_FAST>(fc_hp * inv_sample_rate_, q_lin);

    // -----------------------------------------------------------------
    // Reverb - clouds-style sane defaults, knob ranges narrowed to the
    // musical zone. Shimmer depth/rate are intentionally *not* set
    // here - the reverb runs with its clouds-original LFO modulation.
    // -----------------------------------------------------------------
    reverb_.set_amount    (parameters_.reverb_amount);
    reverb_.set_input_gain(0.30f + 0.50f * parameters_.reverb_drive); // 0.30..0.80
    // 0.30..1.00 - at full CW the all-pass loop is at unity gain and
    // the reverb freezes (effectively infinite tail / drone hold).
    reverb_.set_time      (0.30f + 0.70f * parameters_.reverb_size);
    reverb_.set_diffusion (0.30f + 0.65f * parameters_.reverb_diffusion); // 0.30..0.95 - wider audible spread
    reverb_.set_lp        (0.20f + 0.70f * parameters_.reverb_lp);    // 0.20..0.90

    // -----------------------------------------------------------------
    // Sample loop.
    // -----------------------------------------------------------------
    constexpr float kAudioExcite = 0.10f;
    for (size_t i = 0; i < n; ++i) {
      const float audio_l = in_out[i].l;
      const float audio_r = in_out[i].r;

      // Envelope follower on audio in R for trigger detection.
      const float rect = audio_r < 0.0f ? -audio_r : audio_r;
      const float c = rect > env_r_ ? 0.20f : 0.002f;
      env_r_ += c * (rect - env_r_);

      // 1. KS voice bank, with audio L mixed into each loop input.
      float voice = voices_.Process(audio_l * kAudioExcite);
      // 1a. Saw-stack sub - three naive saws following the chord
      //     ratios, summed and averaged, then post-LPF'd at ~600 Hz so
      //     it stays in the bass register. Mix gain is low (0.15)
      //     because the saws are continuous full-scale oscillators -
      //     anything higher slams the reverb input on top of the K-S
      //     bank's own peaks.
      float saw_sum = 0.0f;
      for (int s = 0; s < 3; ++s) {
        saw_phase_[s] += saw_inc_[s];
        if (saw_phase_[s] >= 1.0f) saw_phase_[s] -= 1.0f;
        saw_sum += 2.0f * saw_phase_[s] - 1.0f;
      }
      saw_sum *= (1.0f / 3.0f);
      sub_lpf_state_ += sub_lpf_a_ * (saw_sum - sub_lpf_state_);
      voice += sub_lpf_state_ * 0.15f;
      // 2. Modal resonator excited by the same source.
      const float modal_out = modal_.Process(voice);
      // 3. Sum. Modal mix locked to the stable_v1 value (red SHIMMER
      //    page is disabled - the knob still moves, just no effect).
      constexpr float kHarmonicsLocked = 0.40f;
      float mixed = voice + modal_out * kHarmonicsLocked;
      // 3a. KS-side LPF - kills broadband hiss generated by burst noise
      //     and string excitation. Bypassed entirely when the knob is
      //     full CW so the K-S/modal output passes through untouched.
      if (!ks_lpf_bypass_) {
        ks_lpf_state_ += ks_lpf_a_ * (mixed - ks_lpf_state_);
        mixed = ks_lpf_state_;
      }
      // 4. Performance LPF -> HPF in series. Two SVFs feeding the
      //    reverb, so the reverb tail also responds to filter sweeps.
      const float lp_out   = svf_   .Process<stmlib::FILTER_MODE_LOW_PASS >(mixed);
      const float filtered = svf_hp_.Process<stmlib::FILTER_MODE_HIGH_PASS>(lp_out);
      // 5. Tape delay (mix coefficient 0 = defeated for now).
      float delay_wet = delay_.Process(filtered);
      float pre_reverb = filtered + delay_wet * 0.0f;
      // 6. Write stereo input for the reverb (mono pre-reverb).
      in_out[i].l = pre_reverb;
      in_out[i].r = pre_reverb;
    }

    // Schmitt trigger on audio-in-R envelope. ~10 ms refractory.
    if (trig_refractory_ > 0) {
      --trig_refractory_;
    } else if (!trig_armed_ && env_r_ > 0.12f) {
      Pluck();
      trig_flag_              = true;
      trig_armed_             = true;
      trig_refractory_        = 15;
      blocks_since_ext_trig_  = 0;        // mark external clock active
    } else if (trig_armed_ && env_r_ < 0.04f) {
      trig_armed_ = false;
    }

    // 7. Plate reverb in place.
    reverb_.Process(in_out, n);

    // 8. Update the reverb-tail feedback EMA.
    const float new_send = sqrtf(reverb_.tail_energy() / static_cast<float>(n));
    reverb_send_ += 0.1f * (new_send - reverb_send_);
  }

  // Output gain - the LVL1 shifted slot used to drive this, but now
  // owns the pitch-octave selector. The entry point uses a fixed
  // unity gain (with SoftLimit catching transient peaks).
  static constexpr float kOutputGain = 1.0f;
  inline float output_gain() const { return kOutputGain; }

  inline VoicingZone voicing_zone() const { return voices_.zone(); }
  inline ChordMode   chord_mode()   const { return voices_.chord_mode(); }
  inline float       reverb_send()  const { return reverb_send_; }

  inline bool TakeTriggerFlag() {
    const bool f = trig_flag_;
    trig_flag_ = false;
    return f;
  }

 private:
  float            sample_rate_      = 48000.0f;
  float            inv_sample_rate_  = 1.0f / 48000.0f;
  float            block_in_seconds_ = 32.0f / 48000.0f;
  DroneParameters  parameters_;
  VoiceBank        voices_;
  ModalBank        modal_;
  TapeDelay        delay_;
  PlateReverb      reverb_;
  stmlib::Svf      svf_;        // Perf LPF, mono pre-reverb
  stmlib::Svf      svf_hp_;     // Perf HPF, mono pre-reverb (in series)
  // Sub-osc: three detuned naive saw oscillators (root, -1 oct, -2 oct).
  // Replaces an earlier K-S sub attempt that never converged to clean
  // pitch at -2/-3 oct.
  float            saw_phase_[3]   = {0.0f, 0.0f, 0.0f};
  float            saw_inc_[3]     = {0.0f, 0.0f, 0.0f};
  // Post-saw one-pole LPF (~600 Hz) - softens the saw discontinuities
  // and keeps sub HF out of the chord band.
  float            sub_lpf_state_  = 0.0f;
  float            sub_lpf_a_      = 1.0f;
  float            reverb_send_ = 0.0f;
  float            lfo_phase_   = 0.0f;
  // KS-side one-pole LPF - runs on the mono voice+modal sum, before any
  // performance filters. Cutoff lives on karplus_lpf (Page 1 LVL1 shift).
  // bypass=true at full CW means the signal passes through completely
  // unfiltered, so a fully-open knob is mathematically transparent.
  float            ks_lpf_state_  = 0.0f;
  float            ks_lpf_a_      = 1.0f;
  bool             ks_lpf_bypass_ = true;

  float            env_r_                  = 0.0f;
  bool             trig_armed_             = false;
  bool             trig_flag_              = false;
  uint16_t         trig_refractory_        = 0;
  int              last_cv_q_              = 0;   // pitch CV hysteresis state
  // Blocks elapsed since the last external (IN R) trigger fired. While
  // this stays small the internal exciter LFO is suppressed - patching
  // a clock to IN R automatically takes over from the LFO knob.
  uint32_t         blocks_since_ext_trig_  = 1000000;

  DISALLOW_COPY_AND_ASSIGN(Drone);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_DRONE_H_
