// signal: IN L + pulse -> ks bank -> +modal*harmonics -> ks lpf -> svf ->
//         predelay/haas -> plate reverb -> overdrive -> out.
// smear feeds reverb tail energy back into the ks noise floor

#ifndef WARPS_DRONE_DSP_DRONE_H_
#define WARPS_DRONE_DSP_DRONE_H_

#include <math.h>

// debug: bypass ks+modal entirely (pulse/saw -> filter -> reverb only).
// #define WARPS_DRONE_BYPASS_KS 1

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"

#include "warps_drone/dsp/parameters.h"
#include "warps_drone/dsp/pink_noise.h"
#include "warps_drone/dsp/voice_bank.h"
#include "warps_drone/dsp/modal_bank.h"
#include "warps_drone/dsp/reverb.h"

namespace warps_drone {

class Drone {
 public:
  Drone() { }

  void Init(float* reverb_buffer, float sample_rate) {
    sample_rate_      = sample_rate;
    inv_sample_rate_  = 1.0f / sample_rate;

    voices_.Init(sample_rate);
    modal_.Init(sample_rate);
    // modal shape fixed at Init - count/pickup/stiffness inaudible under
    // the dense mix. also saves the per-block recompute
    modal_.set_active_modes(8);
    modal_.set_pickup(0.5f);
    modal_.set_stiffness(0.15f);
    modal_.set_brightness(0.60f);
    reverb_.Init(reverb_buffer, sample_rate);
    svf_.Init();
    vinyl_noise_.Init();
    vinyl_lp_state_ = 0.0f;
    // vinyl: pink -> 5kHz lpf, warm tape-hiss bed
    vinyl_lp_a_ = 1.0f - expf(-2.0f * static_cast<float>(M_PI)
                              * 5000.0f * inv_sample_rate_);
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

    // first-block defaults - overwritten by cv_scaler immediately
    parameters_                    = {};
    parameters_.chord              = 0.30f;
    parameters_.lpf_cutoff         = 0.73f;
    parameters_.pulse_gain         = 1.00f;
    parameters_.reverb_amount      = 0.44f;
    parameters_.chord_mode         = 0.25f;
    parameters_.pitch              = 0.50f;
    parameters_.pitch_octave       = 0.50f;
    parameters_.smear              = 0.00f;
    parameters_.filter_resonance   = 0.00f;
    parameters_.damping            = 0.99f;
    parameters_.noise_floor_base   = 0.00f;
    parameters_.reverb_size        = 0.87f;
    parameters_.reverb_diffusion   = 0.60f;
    parameters_.reverb_lp          = 0.40f;
    parameters_.distortion         = 0.00f;
    parameters_.distortion_warmth  = 1.00f;
    parameters_.distortion_tone    = 1.00f;
    parameters_.distortion_bias    = 0.00f;
    parameters_.vinyl_noise        = 0.00f;
    parameters_.reverb_shim_rate   = 0.50f;
    parameters_.reverb_drive       = 0.00f;
    parameters_.harmonics          = 0.40f;
    parameters_.modal_brightness   = 0.60f;
    parameters_.modal_stiffness    = 0.20f;
    parameters_.modal_count        = 0.50f;
    parameters_.modal_pickup       = 0.50f;
    parameters_.karplus_lpf        = 0.85f;
    parameters_.pulse_freq         = 0.50f;
  }

  DroneParameters* mutable_parameters() { return &parameters_; }

  void Process(FloatFrame* in_out, size_t n) {
    // pitch: pot = 12 st (1 oct), octave = 0..5 (C1..C6), cv = chromatic
    const float pot_semitones = parameters_.pitch * 12.0f;
    int oct = static_cast<int>(parameters_.pitch_octave * 6.0f);
    if (oct > 5) oct = 5;

    // cv arrives in semitones (calibrated). hysteresis quantise
    const float cv_semitones = parameters_.reserved_a;
    const float diff = cv_semitones - static_cast<float>(last_cv_q_);
    const float adiff = diff < 0.0f ? -diff : diff;
    if (adiff > 0.65f) {
      last_cv_q_ = static_cast<int>(roundf(cv_semitones));
    }

    const float total = static_cast<float>(oct) * 12.0f
                      + pot_semitones
                      + static_cast<float>(last_cv_q_);
    // C1 = 32.703 Hz
    const float root_hz = 32.703f * powf(2.0f, total * (1.0f / 12.0f));

    voices_.set_chord(root_hz, parameters_.chord, parameters_.chord_mode);
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

    // pulse-osc as primary ks excitation. bipolar, PWM walks 0.40..0.60.
    // freq: DETUNE = log 50..250Hz; chord banks = snap to 2-oct scale
    // one octave below chord root -> harmonics align with chord pitches.
    // CCW dead zone disables
    pulse_enabled_ = parameters_.pulse_freq >= 0.005f;
    float pulse_hz;
    const ChordBank b = voices_.bank();
    if (b == BANK_DETUNE) {
      pulse_hz = 50.0f * powf(5.0f, parameters_.pulse_freq);
    } else {
      // Jump within the chord - 4 zones, one per active voice-bank ratio,
      // one octave below root. Pulse harmonics land on chord pitches so
      // the K-S strings ring them strongly (audible by construction)
      (void)b;
      const float* rr = voices_.ratios();
      int idx = static_cast<int>(parameters_.pulse_freq * 4.0f);
      if (idx < 0) idx = 0;
      if (idx >= 4) idx = 3;
      pulse_hz = root_hz * rr[idx] * 0.25f;  // 2 octaves below chord root
    }
    // ±0.5% freq jitter - kills 3-5Hz tail beats from harmonic alignment
    // with ks resonances. sub-cent, inaudible as pitch
    pulse_freq_jitter_state_ +=
        0.02f * (pulse_freq_jitter_target_ - pulse_freq_jitter_state_);
    pulse_inc_ = pulse_hz * inv_sample_rate_ *
                 (1.0f + pulse_freq_jitter_state_);

    // re-roll PWM + freq-jitter targets every ~50ms (75 blocks)
    ++pulse_jitter_counter_;
    if (pulse_jitter_counter_ >= 75) {
      pulse_jitter_counter_ = 0;
      pulse_width_target_ =
          0.40f + stmlib::Random::GetFloat() * 0.20f;
      pulse_freq_jitter_target_ =
          (stmlib::Random::GetFloat() - 0.5f) * 0.010f;
    }

    // smear gated by reverb_amount - otherwise wet=0 still pumps noise
    const float floor_base  = parameters_.noise_floor_base;
    const float rev_a       = parameters_.reverb_amount;
    const float total_noise_floor =
        floor_base * floor_base * 0.150f
      + d * d * 0.012f
      + reverb_send_ * parameters_.smear * rev_a * 0.100f;
    voices_.set_noise_floor(total_noise_floor);

    modal_.set_fundamental(root_hz);

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
    const float q_lin = 0.7f + parameters_.filter_resonance * 9.3f;
    svf_.set_f_q<stmlib::FREQUENCY_FAST>(fc_lp * inv_sample_rate_, q_lin);

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

    constexpr float kAudioExcite = 0.10f;
    for (size_t i = 0; i < n; ++i) {
      const float audio_l = in_out[i].l;

      // pulse-osc
      pulse_phase_ += pulse_inc_;
      if (pulse_phase_ >= 1.0f) pulse_phase_ -= 1.0f;
      pulse_width_ += 0.001f * (pulse_width_target_ - pulse_width_);
      const float pulse_out = pulse_enabled_
          ? (pulse_phase_ < pulse_width_ ? 1.0f : -1.0f)
          : 0.0f;

      // ks injection capped 0.025 - higher slammed reverb above ~C5
      const float pulse_inject = pulse_out * (0.050f * parameters_.pulse_gain);
      const float ks_input = audio_l * kAudioExcite + pulse_inject;
#ifdef WARPS_DRONE_BYPASS_KS
      (void)ks_input;
      float voice = pulse_out * 0.08f + audio_l * 0.25f;
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
      float vinyl_gate = parameters_.distortion > 0.005f
          ? parameters_.distortion * 2.0f
          : 0.0f;
      if (vinyl_gate > 1.0f) vinyl_gate = 1.0f;
      const float vinyl_amt = parameters_.vinyl_noise * 0.05f * vinyl_gate;
      float pre_reverb = filtered + vinyl_lp_state_ * vinyl_amt;

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

    // tail-energy ema for smear feedback
    const float new_send = sqrtf(reverb_.tail_energy() / static_cast<float>(n));
    reverb_send_ += 0.1f * (new_send - reverb_send_);
  }

  static constexpr float kOutputGain = 1.0f;
  inline float output_gain() const { return kOutputGain; }

  inline ChordBank   bank()         const { return voices_.bank(); }
  inline DensityZone density_zone() const { return voices_.density_zone(); }
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
  stmlib::Svf      svf_;        // perf lpf
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
  float            reverb_send_ = 0.0f;
  // ks lpf, bypass at full CW
  float            ks_lpf_state_  = 0.0f;
  float            ks_lpf_a_      = 1.0f;
  bool             ks_lpf_bypass_ = true;
  float            reverb_bypass_fade_ = 0.0f;

  int              last_cv_q_              = 0;

  DISALLOW_COPY_AND_ASSIGN(Drone);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_DRONE_H_
