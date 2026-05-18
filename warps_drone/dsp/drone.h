// signal: IN L + pulse -> ks bank -> +modal*harmonics -> ks lpf -> svf ->
//         predelay/haas -> plate reverb -> overdrive -> out.
// smear feeds reverb tail energy back into the ks noise floor

#ifndef WARPS_DRONE_DSP_DRONE_H_
#define WARPS_DRONE_DSP_DRONE_H_

#include <math.h>
#include <stdint.h>

// debug: bypass ks+modal entirely (pulse/saw -> filter -> reverb only).
// #define WARPS_DRONE_BYPASS_KS 1

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"

#include "warps_drone/dsp/parameters.h"
#include "warps_drone/dsp/pink_noise.h"
#include "warps_drone/dsp/voice_bank.h"
#include "warps_drone/dsp/modal_bank.h"
#include "warps_drone/dsp/karplus_voice.h"
#include "warps_drone/dsp/reverb.h"

namespace warps_drone {

class Drone {
 public:
  Drone() { }

  void Init(float* reverb_buffer, float sample_rate) {
    sample_rate_      = sample_rate;
    inv_sample_rate_  = 1.0f / sample_rate;

    voices_.Init(sample_rate);
    // Bass string — separate K-S voice. High decay so a single pluck
    // rings long enough to behave as a drone bed between plucks.
    // Brightness moderate (it's bass, doesn't need top sparkle).
    bass_voice_.Init();
    bass_voice_.set_decay(0.998f);
    bass_voice_.set_brightness(0.55f);
    // Continuous noise excitation — bass rings evenly at any tuned pitch
    // (no pluck event, no harmonic-alignment dependency on a shared pulse).
    bass_voice_.set_noise_floor(0.015f);
    bass_voice_.set_noise_color(0.0f);   // 0 = pure pink (warm low-tilt)
    // ~1.8 kHz one-pole LPF on the bass output so the K-S top-end harmonics
    // stay out of the chord band.
    bass_lpf_a_ = 1.0f - expf(-2.0f * static_cast<float>(M_PI)
                              * 1800.0f * inv_sample_rate_);
    bass_zone_prev_ = -1;
    chord_duck_env_ = 1.0f;
    chord_duck_target_ = 1.0f;
    bass_sub_phase_ = 0.0f;
    bass_sub_lpf_state_ = 0.0f;
    bass_log_freq_ = 0.0f;
    bass_log_freq_init_ = false;
    // ~500 Hz one-pole LPF on the sub-saw — keeps it weighty without
    // bleeding mid/HF into the bass band.
    bass_sub_lpf_a_ = 1.0f - expf(-2.0f * static_cast<float>(M_PI)
                                  * 500.0f * inv_sample_rate_);

    modal_.Init(sample_rate);
    // 8 modes at boot; switched to 4 in Process() while the saturator
    // is engaged (see notes there). Pickup/stiffness fixed.
    modal_.set_active_modes(8);
    modal_.set_pickup(0.5f);
    modal_.set_stiffness(0.15f);
    modal_.set_brightness(0.60f);
    reverb_.Init(reverb_buffer, sample_rate);
    svf_.Init();
    bass_svf_.Init();
    vinyl_noise_.Init();
    vinyl_lp_state_ = 0.0f;
    // vinyl LPF coefficient is computed per-block from vinyl_color knob;
    // seed at a warm cutoff so the first block doesn't pop.
    vinyl_lp_a_ = 1.0f - expf(-2.0f * static_cast<float>(M_PI)
                              * 1500.0f * inv_sample_rate_);
    // ~1.5 kHz LP on the crackle output — softens the click attack so
    // it reads as warm "vinyl tick" instead of bright metallic "snap".
    crackle_lp_a_ = 1.0f - expf(-2.0f * static_cast<float>(M_PI)
                                * 1500.0f * inv_sample_rate_);
    crackle_lp_state_ = 0.0f;
    for (int s = 0; s < 3; ++s) {
      saw_phase_[s] = static_cast<float>(s) * 0.333f;
      saw_inc_[s]   = 0.0f;
    }
    pulse_phase_           = 0.0f;
    pulse_inc_             = 0.0f;
    pulse_width_           = 0.5f;
    pulse_width_target_    = 0.5f;
    pulse_jitter_counter_  = 0;

    reverb_send_ = 0.0f;
    pre_reverb_dc_state_ = 0.0f;
    ks_lpf_state_ = 0.0f;
    ks_lpf_a_     = 1.0f;
    ks_lpf_bypass_ = true;

    // First-block defaults — overwritten by cv_scaler on the first
    // Read(). Numbers below mirror kAlgo/kParam/kLvl1/kLvl2 in
    // cv_scaler.cc (developer's settings recovered from the journal),
    // so first-audio-block sounds consistent with the post-journal
    // state instead of a random silence/full mix.
    parameters_                    = {};
    parameters_.chord              = 0.41f;
    parameters_.bass_note          = 0.55f;
    parameters_.pitch_octave       = 1.00f;
    parameters_.lpf_cutoff         = 0.86f;
    parameters_.chord_mode         = 0.19f;  // MINOR bank
    parameters_.bass_glide         = 0.00f;
    parameters_.chord_volume       = 1.00f;
    parameters_.bass_volume        = 0.37f;
    parameters_.damping            = 0.75f;
    parameters_.bass_weight        = 0.28f;
    parameters_.karplus_lpf        = 1.00f;
    parameters_.noise_floor_base   = 0.65f;
    parameters_.harmonics          = 0.86f;
    parameters_.bass_drive         = 0.06f;
    parameters_.filter_resonance   = 0.00f;
    parameters_.white_pink_mix     = 0.22f;
    parameters_.reverb_size        = 0.89f;
    parameters_.reverb_diffusion   = 0.81f;
    parameters_.reverb_lp          = 1.00f;
    parameters_.reverb_amount      = 1.00f;
    parameters_.reverb_predelay    = 0.84f;
    parameters_.smear              = 0.75f;
    parameters_.reverb_drive       = 0.00f;
    parameters_.reverb_shim_rate   = 0.29f;
    parameters_.distortion         = 0.00f;
    parameters_.distortion_warmth  = 1.00f;
    parameters_.distortion_tone    = 1.00f;
    parameters_.vinyl_noise        = 0.27f;
    parameters_.distortion_bias    = 0.51f;
    parameters_.vinyl_color        = 0.34f;
    parameters_.vinyl_flutter      = 0.00f;   // flutter off by default
    parameters_.modal_brightness   = 0.60f;
  }

  DroneParameters* mutable_parameters() { return &parameters_; }

  void Process(FloatFrame* in_out, size_t n) {
    // pitch: octave selector (0..5 = C1..C6) + V/oct CV chromatic.
    // Fine-pitch knob removed (its slot is chord_volume now).
    int oct = static_cast<int>(parameters_.pitch_octave * 6.0f);
    if (oct > 5) oct = 5;

    // CHORD V/OCT (LVL1 CV, calibrated) — continuous addend, no quantize.
    // The cal Transform output is LP-smoothed upstream; using it raw
    // means a sequencer step glides over the LP time-constant (~10 ms),
    // but it kills the click-on-quantize-step that the saturator was
    // bursting on.
    const float total = static_cast<float>(oct) * 12.0f
                      + parameters_.chord_voct_semitones;
    // C1 = 32.703 Hz
    const float root_hz = 32.703f * powf(2.0f, total * (1.0f / 12.0f));

    voices_.set_chord(root_hz, parameters_.chord, parameters_.chord_mode);

    // Bass K-S character: fixed warm-bright / pink. Bass knob is the
    // chord-tone zone selector; the V/OCT cable (if patched) just adds
    // chromatic semitones on top.
    bass_voice_.set_brightness(0.55f);
    bass_voice_.set_noise_color(0.0f);

    // damping knob - 2-stage:
    //   [0, 0.5]  bright 0..0.97, decay 0.990..0.995  (short, dark->bright)
    //   [0.5, 1]  bright 0.97,    decay 0.995..0.999  (bright, long sustain)
    // bright caps at 0.97 - at 1.0 the 2-tap damp identity lets pulse-osc
    // PWM transitions integrate without bound near nyquist
    const float d = parameters_.damping;
    float ks_bright, ks_decay;
    if (d < 0.5f) {
      const float u = d * 2.0f;
      ks_bright = u * 0.97f;
      ks_decay  = 0.990f + 0.005f * u;
    } else {
      const float u = (d - 0.5f) * 2.0f;
      ks_bright = 0.97f;
      ks_decay  = 0.995f + 0.004f * u;
    }
    voices_.set_decay(ks_decay);
    voices_.set_brightness(ks_bright);
    voices_.set_noise_color(parameters_.white_pink_mix);

    // sub-osc: 3 naive saws at -1 oct, taking the bank's first 3 ratios.
    // small fixed detune (+2¢/-3¢) on bottom two for unison flutter
    const float* r = voices_.ratios();
    saw_inc_[0] = root_hz * r[0] * 0.50f                 * inv_sample_rate_;
    saw_inc_[1] = root_hz * r[1] * 0.50f * 1.00116f      * inv_sample_rate_;
    saw_inc_[2] = root_hz * r[2] * 0.50f * 0.99827f      * inv_sample_rate_;

    // post-saw lpf, ~600Hz. keeps sub HF out of the chord band
    constexpr float kSubLpfHz = 600.0f;
    const float a_sub = 1.0f - expf(-2.0f * static_cast<float>(M_PI)
                                     * kSubLpfHz * inv_sample_rate_);
    sub_lpf_a_ = a_sub;

    // Pulse osc — fixed-frequency, internal-only K-S exciter for the
    // chord. Subsonic (~25 Hz) with PWM + ±0.5 % freq jitter so its
    // harmonic series scatters into the chord band without locking.
    // The old "pulse pitch" knob is now the BASS NOTE selector below.
    constexpr float kPulseHz = 25.0f;
    pulse_freq_jitter_state_ +=
        0.02f * (pulse_freq_jitter_target_ - pulse_freq_jitter_state_);
    pulse_inc_ = kPulseHz * inv_sample_rate_ *
                 (1.0f + pulse_freq_jitter_state_);
    ++pulse_jitter_counter_;
    if (pulse_jitter_counter_ >= 75) {
      pulse_jitter_counter_ = 0;
      pulse_width_target_ =
          0.40f + stmlib::Random::GetFloat() * 0.20f;
      pulse_freq_jitter_target_ =
          (stmlib::Random::GetFloat() - 0.5f) * 0.010f;
    }
    pulse_enabled_ = true;

    // Bass pitch — chord-tone zone snap (knob) + V/OCT chromatic
    // transpose (added on top, continuous, no quantize).
    static constexpr int8_t kBassTones[BANK_COUNT][4] = {
      { 3, 10,  7,  0},   // DETUNE
      { 3, 10,  7,  0},   // MINOR
      { 2,  7,  5,  0},   // MAJOR
      { 2, 10,  5,  0},   // DOM7
      { 5,  7, 10,  0},   // DRONE
      { 1,  6,  3,  0},   // WEIRD
    };
    int bass_zone = static_cast<int>(parameters_.bass_note * 7.0f);
    if (bass_zone < 0) bass_zone = 0;
    if (bass_zone > 6) bass_zone = 6;
    const ChordBank cb = voices_.bank();
    const int tone_idx = bass_zone < 3 ? bass_zone : (bass_zone - 3);
    const int oct_shift = bass_zone < 3 ? 0 : 12;
    const int chord_tone_semis = kBassTones[cb][tone_idx] + oct_shift;
    const float bass_semis_above =
        static_cast<float>(chord_tone_semis) + parameters_.bass_voct_semitones;
    constexpr float kBassBaseHz = 32.703f;   // C1
    const float bass_hz = kBassBaseHz *
        powf(2.0f, bass_semis_above * (1.0f / 12.0f));
    // Glide: smooth log2(bass_hz) toward the target. Per-block one-pole.
    //   block_dt = n / sample_rate (~0.67 ms at 32-sample blocks).
    //   glide_seconds = parameters_.bass_glide × 2.0 (CCW=0, CW=2s).
    //   τ = glide_seconds → 1-exp(-dt/τ) per block.
    const float target_log = log2f(bass_hz);
    if (!bass_log_freq_init_) {
      bass_log_freq_ = target_log;
      bass_log_freq_init_ = true;
    } else {
      // τ = knob × 0.14 s → full glide (5τ ≈ 99 %) reaches in 0.7 s at CW.
      const float glide_s = parameters_.bass_glide * 0.14f;
      if (glide_s < 0.005f) {
        bass_log_freq_ = target_log;  // CCW dead-zone → instant
      } else {
        const float block_dt = static_cast<float>(n) * inv_sample_rate_;
        const float a = 1.0f - expf(-block_dt / glide_s);
        bass_log_freq_ += a * (target_log - bass_log_freq_);
      }
    }
    const float bass_hz_glided = exp2f(bass_log_freq_);
    bass_voice_.set_delay(sample_rate_ / bass_hz_glided);
    // Saw layer at the same pitch as the K-S bass — follows the glide.
    bass_sub_inc_ = bass_hz_glided * inv_sample_rate_;
    // bass_drive lengthens the K-S decay (drone tail grows) and feeds
    // a downstream saturator. Range 0.997 → 0.9995 — drone-friendly,
    // capped just under self-osc.
    bass_voice_.set_decay(0.997f + parameters_.bass_drive * 0.0025f);

    // Side-chain duck on the chord — fires only on bass-knob zone
    // changes (discrete, click-free). V/OCT changes don't fire the
    // duck (continuous, would chatter from any CV noise).
    //
    // Duck depth is FIXED, not scaled with bass_volume. The previous
    // scaling made the duck disappear at moderate bass settings —
    // exactly the levels where the duck does the most work
    // (subtle bass benefits from chord ducking to feel like a note).
    // Only gate by whether the bass is audible at all.
    if (bass_zone != bass_zone_prev_ &&
        bass_zone_prev_ != INT32_MIN &&
        parameters_.bass_volume > 0.05f) {
      chord_duck_target_ = 0.55f;        // chord dips to 55 % (= 45 % drop)
    }
    bass_zone_prev_ = bass_zone;

    // K-S noise floor — continuous excitation source for the chord
    // drone. Scaled by karplus_lpf so the noise reduces along with the
    // LPF cutoff: bright = full noise (drone character), dark = quiet
    // noise (modal-bank BPFs don't get exposed and rung through the
    // saturator).
    //   smear gated by reverb_amount — otherwise wet=0 still pumps noise.
    const float floor_base    = parameters_.noise_floor_base;
    const float rev_a         = parameters_.reverb_amount;
    const float lpf_noise_gate = 0.20f + 0.80f * parameters_.karplus_lpf;
    const float total_noise_floor =
        (floor_base * floor_base * 0.150f
         + d * d * 0.012f) * lpf_noise_gate
      + reverb_send_ * parameters_.smear * rev_a * 0.100f;
    voices_.set_noise_floor(total_noise_floor);

    // Only update modal-bank fundamental on a meaningful pitch move.
    // RecomputeFrequencies retunes 8 resonant BPFs every call; doing
    // it every block with continuous V/OCT (sub-cent wobble) makes
    // the high-Q filter coefficients chatter against their state and
    // ring — masked by the K-S HF normally, but laid bare and amplified
    // by the saturator the moment the karplus_lpf is engaged.
    // 0.5 % threshold ≈ 9 cents — well below sequencer-step amounts.
    if (modal_root_hz_ <= 0.0f ||
        fabsf(root_hz - modal_root_hz_) > modal_root_hz_ * 0.005f) {
      modal_.set_fundamental(root_hz);
      modal_root_hz_ = root_hz;
    }

    // CPU budget guard. When overdrive is engaged, the saturator adds
    // ~4 sqrtf per sample and vinyl_buf is summed; the inner loop can
    // overrun the codec block deadline at 32 samples/48k. Audible as
    // wideband "krr krr" glitches whose rate depends on chord-density
    // (different K-S delay patterns → different cache/cycle profiles).
    // Halve the modal bank to 4 modes during overdrive — saves ~80
    // cycles/sample — and restore 8 modes when overdrive is bypassed.
    modal_.set_active_modes(parameters_.distortion > 0.005f ? 4 : 8);

    // perf lpf: log 60Hz..12kHz, +cv. ks lpf: log 200Hz..16kHz, bypass at CW
    const float fc_lp = 60.0f * powf(200.0f, parameters_.lpf_cutoff);
    if (parameters_.karplus_lpf >= 0.999f) {
      ks_lpf_bypass_ = true;
    } else {
      ks_lpf_bypass_ = false;
      const float fc_ks = 200.0f * powf(80.0f, parameters_.karplus_lpf);
      const float aks   = 1.0f - expf(-2.0f * static_cast<float>(M_PI)
                                       * fc_ks * inv_sample_rate_);
      ks_lpf_a_ = aks > 1.0f ? 1.0f : (aks < 0.0f ? 0.0f : aks);
    }
    // Q: 0.7 butterworth -> 10 whistle
    // Perf LPF Q — KARPLUS LVL1 shifted (filter_resonance knob).
    // 0.7 (Butterworth) at CCW → 10 (whistle) at CW.
    const float q_lin = 0.7f + parameters_.filter_resonance * 9.3f;
    svf_     .set_f_q<stmlib::FREQUENCY_FAST>(fc_lp * inv_sample_rate_, q_lin);
    bass_svf_.set_f_q<stmlib::FREQUENCY_FAST>(fc_lp * inv_sample_rate_, q_lin);

    // predelay: 0..3000 samp (~62ms), slewed
    {
      const float pd_target = parameters_.reverb_predelay * 3000.0f;
      pre_delay_samples_ += 0.05f * (pd_target - pre_delay_samples_);
    }
    // bypass crossfade <0.5% -> mono-dry, ~130ms TC
    {
      const float target = parameters_.reverb_amount > 0.005f ? 1.0f : 0.0f;
      reverb_bypass_fade_ += 0.005f * (target - reverb_bypass_fade_);
    }
    reverb_.set_amount    (parameters_.reverb_amount);
    reverb_.set_input_gain(0.15f + 0.25f * parameters_.reverb_drive);
    // time clamped <1 - true freeze + distortion flat-lines AC
    reverb_.set_time      (0.30f + 0.69f * parameters_.reverb_size);
    reverb_.set_diffusion (0.95f * parameters_.reverb_diffusion);
    reverb_.set_lp        (0.20f + 0.70f * parameters_.reverb_lp);
    reverb_.set_shimmer_rate(0.25f + 1.75f * parameters_.reverb_shim_rate);

    // Vinyl-flutter LFO — modulates vinyl noise volume.
    //   knob 0          → off (lfo bypassed)
    //   knob 0..0.5     → triangle LFO, 0.2 → 2 Hz as knob moves CW
    //   knob 0.5..1     → slewed random LFO, 2 Hz → 0.3 Hz as knob CW
    // Max rate capped at 2 Hz so the AM doesn't reach into the audible
    // band where the saturator turns each LFO cycle into a wideband
    // transient ("krr krr"). Gated by vinyl_noise > 2 % too.
    {
      const float fk = parameters_.vinyl_flutter;
      if (fk <= 0.01f || parameters_.vinyl_noise < 0.02f) {
        flutter_active_     = false;
        flutter_rate_       = 0.0f;
        flutter_random_mode_= false;
      } else if (fk < 0.5f) {
        flutter_active_      = true;
        flutter_random_mode_ = false;
        const float t = fk * 2.0f;
        const float hz = 0.2f + t * 1.8f;
        flutter_rate_ = hz * inv_sample_rate_;
      } else {
        flutter_active_      = true;
        flutter_random_mode_ = true;
        const float t = (fk - 0.5f) * 2.0f;
        const float hz = 2.0f - t * 1.7f;
        flutter_rate_ = hz * inv_sample_rate_;
      }
    }

    // Vinyl color → noise LPF cutoff, log 400 Hz..6 kHz.
    //   CCW (0)   → 400 Hz  (very warm, "rumbly tape")
    //   default 0.3 → ~1.2 kHz (vintage vinyl)
    //   CW (1)    → 6 kHz   (bright surface hiss)
    const float vinyl_fc = 400.0f * powf(15.0f, parameters_.vinyl_color);
    vinyl_lp_a_ = 1.0f - expf(-2.0f * static_cast<float>(M_PI)
                               * vinyl_fc * inv_sample_rate_);

    for (size_t i = 0; i < n; ++i) {
      // pulse-osc — sole exciter; internal drone.
      pulse_phase_ += pulse_inc_;
      if (pulse_phase_ >= 1.0f) pulse_phase_ -= 1.0f;
      pulse_width_ += 0.001f * (pulse_width_target_ - pulse_width_);
      const float pulse_out = pulse_enabled_
          ? (pulse_phase_ < pulse_width_ ? 1.0f : -1.0f)
          : 0.0f;
      const float pulse_inject = pulse_out * 0.020f;
      const float bass_excite  = pulse_inject;
      const float ks_input     = pulse_inject;
#ifdef WARPS_DRONE_BYPASS_KS
      (void)ks_input;
      float voice = pulse_out * 0.08f;
#else
      float voice = voices_.Process(ks_input);
#endif
      // saw sub
      float saw_sum = 0.0f;
      for (int s = 0; s < 3; ++s) {
        saw_phase_[s] += saw_inc_[s];
        if (saw_phase_[s] >= 1.0f) saw_phase_[s] -= 1.0f;
        saw_sum += 2.0f * saw_phase_[s] - 1.0f;
      }
      saw_sum *= (1.0f / 3.0f);
      sub_lpf_state_ += sub_lpf_a_ * (saw_sum - sub_lpf_state_);
      voice += sub_lpf_state_ * 0.04f;

      // (Chord duck is applied post-reverb so the wet tail also dips,
      //  and so the dry-only chord doesn't dip while the wet stays loud.)

      // Bass string — continuous noise-excited drone, independent path.
      // Pulse-osc burst is also injected so the bass tracks the chord's
      // growl character; the noise floor guarantees an even ring at any
      // tuned pitch regardless of harmonic alignment. Routed through a
      // dedicated SVF that mirrors the perf LPF (same cutoff + Q) so
      // the LPF knob shapes the bass too — bass then bypasses the
      // reverb (summed in after reverb_.Process below).
      // K-S bass string. External exciter on IN R (when patched), else
      // shared internal pulse — same wave as the chord network.
      const float bass_ks = bass_voice_.Process(bass_excite);

      // Sub-octave saw — naive triangle/saw at bass_hz/2, low-pass'd at
      // ~500 Hz to keep it weighty. Mix amount tracks bass_weight.
      bass_sub_phase_ += bass_sub_inc_;
      if (bass_sub_phase_ >= 1.0f) bass_sub_phase_ -= 1.0f;
      const float sub_raw = 2.0f * bass_sub_phase_ - 1.0f;
      bass_sub_lpf_state_ += bass_sub_lpf_a_ * (sub_raw - bass_sub_lpf_state_);

      float bass_mix = bass_ks + bass_sub_lpf_state_ * parameters_.bass_weight * 0.35f;

      // Bass drive — pre-saturator gain × soft-clip. The saturator
      // preserves the harmonic character; the K-S decay extension
      // (set above) keeps the drone-tail growth. The make-DOWN gain
      // below compensates so the long-decay + saturated bass doesn't
      // dwarf the chord at high drive settings.
      const float drv = 1.0f + parameters_.bass_drive * 4.0f;
      const float x = bass_mix * drv;
      bass_mix = x / sqrtf(1.0f + x * x);
      bass_mix *= 1.0f - parameters_.bass_drive * 0.55f;

      // Through the perf LPF (same cutoff + Q as the chord).
      bass_buf_[i] = parameters_.lpf_cutoff >= 0.999f
          ? bass_mix
          : bass_svf_.Process<stmlib::FILTER_MODE_LOW_PASS>(bass_mix);

#ifdef WARPS_DRONE_BYPASS_KS
      float mixed = voice;
#else
      const float modal_out = modal_.Process(voice);
      float mixed = voice + modal_out * parameters_.harmonics;
#endif
      if (!ks_lpf_bypass_) {
        ks_lpf_state_ += ks_lpf_a_ * (mixed - ks_lpf_state_);
        mixed = ks_lpf_state_;
      }
      const float filtered = parameters_.lpf_cutoff >= 0.999f
          ? mixed
          : svf_.Process<stmlib::FILTER_MODE_LOW_PASS>(mixed);
      // vinyl bed - pink -> 5kHz lpf -> pre-reverb. gated by overdrive,
      // ramps in over first half of drive (0.005..0.5 -> 0..1)
      const float raw_n = vinyl_noise_.Next();
      vinyl_lp_state_ += vinyl_lp_a_ * (raw_n - vinyl_lp_state_);
      // Page-active gate: noise plays iff OVERDRIVE is engaged.
      // Color-compensation: low-cutoff (warm) positions lose audible
      // energy (1/f noise × narrow LPF), so multiply by 1+(1-color)·1.5
      // so the perceived loudness stays even across the color knob.
      const float vinyl_gate = parameters_.distortion > 0.005f ? 1.0f : 0.0f;
      const float vinyl_comp = 1.0f + (1.0f - parameters_.vinyl_color) * 2.5f;
      // Flutter LFO multiplier — 0..1 modulates the vinyl volume.
      float flutter_mul = 1.0f;
      if (flutter_active_) {
        flutter_phase_ += flutter_rate_;
        if (flutter_phase_ >= 1.0f) {
          flutter_phase_ -= 1.0f;
          if (flutter_random_mode_) {
            flutter_random_target_ = stmlib::Random::GetFloat();
          }
        }
        if (flutter_random_mode_) {
          flutter_value_ += 0.0008f *
              (flutter_random_target_ - flutter_value_);
        } else {
          const float tri = flutter_phase_ < 0.5f
              ? flutter_phase_ * 2.0f
              : (1.0f - flutter_phase_) * 2.0f;
          flutter_value_ = tri * tri * (3.0f - 2.0f * tri);
        }
        flutter_mul = 0.85f + 0.15f * flutter_value_;
      }
      const float vinyl_amt  = parameters_.vinyl_noise * 0.060f
                              * vinyl_comp * vinyl_gate * flutter_mul;
      // Crackles — random one-shot exp-decay impulses. Both density
      // (Poisson rate) and amplitude scale with (1 - vinyl_color):
      //   color = 1.0 (CW)   → no crackles, clean carpet
      //   color = 0.0 (CCW)  → busy crackle bed (~50 events/s)
      // Riding on top of the same vinyl_noise level so they mute
      // together when the user pulls vinyl_noise down.
      const float crackle_amount = 1.0f - parameters_.vinyl_color;
      const float crackle_p = crackle_amount * 0.0010f;
      if (vinyl_gate > 0.0f && stmlib::Random::GetFloat() < crackle_p) {
        // Cubic shaping (sign-preserving) makes loud clicks rare —
        // most events are small "ticks" and the occasional one really
        // pops through. Matches how vinyl wear actually sounds.
        const float r = 2.0f * stmlib::Random::GetFloat() - 1.0f;
        crackle_env_ = r * r * r * crackle_amount;
      }
      crackle_env_ *= 0.991f;   // ~1.5 ms tail — duller than 0.985

      // LP the crackle output — kills the metallic edge on the attack
      // transient so each click reads as a warm tape pop.
      crackle_lp_state_ += crackle_lp_a_ * (crackle_env_ - crackle_lp_state_);

      // Noise + crackles bypass both the reverb and the overdrive saturator;
      // summed post-chain in warps_drone.cc. Both scale with flutter_mul so
      // the crackle/noise ratio stays constant throughout the LFO cycle —
      // without this, crackles stay at full amplitude while the noise bed
      // dips, creating harsh isolated pops at the LFO trough.
      // Noise + crackles bypass both the reverb and the overdrive saturator;
      // summed post-chain in warps_drone.cc. Both scale with flutter_mul so
      // the crackle/noise ratio stays constant throughout the LFO cycle —
      // without this, crackles stay at full amplitude while the noise bed
      // dips, creating harsh isolated pops at the LFO trough.
      vinyl_buf_[i] =
          vinyl_lp_state_ * vinyl_amt
        + crackle_lp_state_ * parameters_.vinyl_noise * vinyl_gate * 0.35f
                            * flutter_mul;
      float pre_reverb = filtered;

      // dc blocker - pulse PWM × ks loop DC × reverb feedback otherwise
      // saturates asymmetric and dims AC. ~5Hz corner
      pre_reverb_dc_state_ += 0.0007f * (pre_reverb - pre_reverb_dc_state_);
      pre_reverb -= pre_reverb_dc_state_;

      pre_reverb = stmlib::SoftLimit(pre_reverb);

      // predelay + haas: 1 ring buf, 2 taps (R = L + kStereoDelayR samples).
      // fractional read avoids pitch-shift on knob sweeps
      stereo_delay_buf_[stereo_delay_pos_] = pre_reverb;
      const float pd        = pre_delay_samples_;
      const int   pd_int    = static_cast<int>(pd);
      const float pd_frac   = pd - static_cast<float>(pd_int);
      const uint32_t mask   = kStereoDelayBufSize - 1;
      const uint32_t l_a    = (stereo_delay_pos_ + kStereoDelayBufSize - pd_int    ) & mask;
      const uint32_t l_b    = (stereo_delay_pos_ + kStereoDelayBufSize - pd_int - 1) & mask;
      const uint32_t r_offs = pd_int + kStereoDelayR;
      const uint32_t r_a    = (stereo_delay_pos_ + kStereoDelayBufSize - r_offs    ) & mask;
      const uint32_t r_b    = (stereo_delay_pos_ + kStereoDelayBufSize - r_offs - 1) & mask;
      const float pre_l = stereo_delay_buf_[l_a] +
                          pd_frac * (stereo_delay_buf_[l_b] - stereo_delay_buf_[l_a]);
      const float pre_r = stereo_delay_buf_[r_a] +
                          pd_frac * (stereo_delay_buf_[r_b] - stereo_delay_buf_[r_a]);
      // smooth bypass crossfade - clickless threshold cross
      const float fade = reverb_bypass_fade_;
      in_out[i].l = pre_reverb + (pre_l - pre_reverb) * fade;
      in_out[i].r = pre_reverb + (pre_r - pre_reverb) * fade;
      stereo_delay_pos_ = (stereo_delay_pos_ + 1) & mask;
    }

    // plate reverb always runs - wet=0 just nulls the mix, no un-bypass glitch
    reverb_.Process(in_out, n);

    // Post-reverb pass: side-chain duck on the chord (wet+dry both),
    // then add the bass (unducked, bypasses reverb).
    //   chord_duck_target_ steps to 0.55 on each bass-note change and
    //   drifts back to 1.0 with τ ≈ 600 ms (α = 3.5e-5).
    //   chord_duck_env_ tracks target with τ ≈ 10 ms (α = 2.0e-3) —
    //   fast enough to follow the step but smooth enough to avoid the
    //   discontinuity click an instantaneous drop produced.
    // Post-reverb: chord_volume scales the chord (wet+dry) after the
    // duck. bass_volume scales the bass sum (bypasses reverb).
    // Half-travel on bass — the K-S + sub-saw + drive stack runs hot
    // and at full knob the bass swamps the chord. Halving the gain
    // shortens the useful range to the CCW half of the knob.
    const float chord_gain = parameters_.chord_volume;
    const float bass_gain  = parameters_.bass_volume * 0.5f;
    for (size_t i = 0; i < n; ++i) {
      const float chord_g = chord_duck_env_ * chord_gain;
      in_out[i].l *= chord_g;
      in_out[i].r *= chord_g;
      const float bs = bass_buf_[i] * bass_gain;
      in_out[i].l += bs;
      in_out[i].r += bs;
      // vinyl_buf is exposed via vinyl_buf() — summed AFTER the
      // saturator in warps_drone.cc so flutter modulation doesn't
      // interact with the non-linearity (which would create AM
      // sidebands audible as rhythmic clicks).
      chord_duck_target_ += 3.5e-5f * (1.0f - chord_duck_target_);
      chord_duck_env_    += 2.0e-3f * (chord_duck_target_ - chord_duck_env_);
    }

    // tail-energy ema for smear feedback
    const float new_send = sqrtf(reverb_.tail_energy() / static_cast<float>(n));
    reverb_send_ += 0.1f * (new_send - reverb_send_);
  }

  static constexpr float kOutputGain = 1.0f;
  inline float output_gain() const { return kOutputGain; }

  inline ChordBank   bank()         const { return voices_.bank(); }
  inline DensityZone density_zone() const { return voices_.density_zone(); }
  // Per-block vinyl-noise buffer for post-chain summing in warps_drone.cc.
  inline const float* vinyl_buf()   const { return vinyl_buf_; }
  inline float       reverb_send()  const { return reverb_send_; }
  inline float       distortion_amount() const { return parameters_.distortion; }
  inline float       distortion_warmth() const { return parameters_.distortion_warmth; }
  inline float       distortion_tone()   const { return parameters_.distortion_tone; }
  inline float       distortion_bias()   const { return parameters_.distortion_bias; }

 private:
  float            sample_rate_      = 48000.0f;
  float            inv_sample_rate_  = 1.0f / 48000.0f;
  DroneParameters  parameters_;
  VoiceBank        voices_;
  ModalBank        modal_;
  PlateReverb      reverb_;
  stmlib::Svf      svf_;        // perf lpf (chord path)
  stmlib::Svf      bass_svf_;   // perf lpf (bass path — same cutoff/Q)
  // sub-osc: 3 naive saws (root, -1 oct, -2 oct)
  float            saw_phase_[3]   = {0.0f, 0.0f, 0.0f};
  float            saw_inc_[3]     = {0.0f, 0.0f, 0.0f};
  // pulse-osc - subsonic bipolar w/ PWM jitter
  float            pulse_phase_           = 0.0f;
  float            pulse_inc_             = 0.0f;
  float            pulse_width_           = 0.5f;
  float            pulse_width_target_    = 0.5f;
  uint16_t         pulse_jitter_counter_  = 0;
  // ±0.5% freq jitter
  float            pulse_freq_jitter_state_  = 0.0f;
  float            pulse_freq_jitter_target_ = 0.0f;
  bool             pulse_enabled_            = true;
  // pre-reverb ring: L = predelay, R = +kStereoDelayR (haas)
  static constexpr uint32_t kStereoDelayBufSize = 4096;   // ~85ms
  static constexpr uint32_t kStereoDelayR       = 960;    // 20ms @ 48k
  float            stereo_delay_buf_[kStereoDelayBufSize] = {0.0f};
  uint32_t         stereo_delay_pos_         = 0;
  float            pre_delay_samples_        = 0.0f;
  float            pre_reverb_dc_state_      = 0.0f;   // ~5Hz hpf
  float            sub_lpf_state_  = 0.0f;
  float            sub_lpf_a_      = 1.0f;
  PinkNoise        vinyl_noise_;
  float            vinyl_lp_state_ = 0.0f;
  float            vinyl_lp_a_     = 1.0f;
  // Vinyl crackle — sparse exp-decay clicks, density+amplitude scale
  // with (1 - vinyl_color) so CCW = max crackle, CW = clean carpet.
  // LP'd at ~1.5 kHz so the clicks read as "tape" rather than "snap".
  float            crackle_env_      = 0.0f;
  float            crackle_lp_state_ = 0.0f;
  float            crackle_lp_a_     = 1.0f;
  float            reverb_send_ = 0.0f;
  // ks lpf, bypass at full CW
  float            ks_lpf_state_  = 0.0f;
  float            ks_lpf_a_      = 1.0f;
  bool             ks_lpf_bypass_ = true;
  float            reverb_bypass_fade_ = 0.0f;

  int              last_cv_q_              = 0;
  int              last_bass_cv_q_         = 0;
  // Last fundamental we pushed into the modal bank — used to gate
  // RecomputeFrequencies so small V/OCT wobble doesn't ring the BPFs.
  float            modal_root_hz_          = 0.0f;
  // Vinyl-flutter LFO state.
  bool             flutter_active_        = false;
  bool             flutter_random_mode_   = false;
  float            flutter_phase_         = 0.0f;
  float            flutter_rate_          = 0.0f;
  float            flutter_value_         = 1.0f;
  float            flutter_random_target_ = 1.0f;


  // Bass string — 4th K-S voice repurposed. Tuned to chord-tone × bass
  // octave (decoupled from chord pitch), plucked on note change,
  // bypasses the reverb entirely. Output is buffered per-block so it
  // can be summed in after reverb_.Process runs on the chord.
  KarplusVoice     bass_voice_;
  int              bass_zone_prev_         = INT32_MIN;     // skip duck @ boot
  float            chord_duck_env_         = 1.0f;   // sidechain duck (1=open)
  float            chord_duck_target_      = 1.0f;   // target env tracks (smooth attack)
  float            bass_lpf_state_         = 0.0f;
  float            bass_lpf_a_             = 1.0f;
  static constexpr size_t kMaxBlock        = 32;
  float            bass_buf_[kMaxBlock]    = {0.0f};
  float            vinyl_buf_[kMaxBlock]   = {0.0f};
  // Bass portamento — smoothed in log2(freq) so a glide between two
  // octaves takes the same audible time regardless of register.
  float            bass_log_freq_          = 0.0f;
  bool             bass_log_freq_init_     = false;
  // Saw layer (at the bass K-S pitch — same octave, mixed via bass_weight).
  float            bass_sub_phase_         = 0.0f;
  float            bass_sub_inc_           = 0.0f;
  float            bass_sub_lpf_state_     = 0.0f;
  float            bass_sub_lpf_a_         = 1.0f;

  DISALLOW_COPY_AND_ASSIGN(Drone);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_DRONE_H_
