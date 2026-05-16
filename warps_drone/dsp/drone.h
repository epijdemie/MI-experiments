// Top-level drone synthesizer.
//
// Topology per audio block:
//
//     ┌── reverb tail FB ──────────────────────────────────────┐
//     │                                                        │
//   audio in L (excite) ─► KS voice bank (4 strings, chord) ──┐│
//                                        │                    ││
//                                        ├── + ──► SVF ──► tape delay ──► plate reverb ──► out
//   Modal resonator (16 modes) ──────────┘                                  │
//      excited by same source                                                │
//   audio in R ─► envelope follower ─► schmitt ─► Pluck()                    │
//                                                                            │
// The reverb is wet-only here - the dry voices are blended in via
// `reverb_amount`. The reverb tail energy is fed back into the KS
// exciter (continuous noise floor) to produce the slow swelling
// "smear" you wants.
//
// Audio I/O usage:
//   IN L  -> mixed per-sample into each KS voice's loop input (continuous
//           audio excitement - anything from a sine wave to a sample plays
//           the chord through the strings).
//   IN R  -> magnitude-rectified, peak-tracking envelope follower with a
//           schmitt + refractory window fires Pluck() on each rising edge.
//           Works for triggers, gates, and slow LFOs.

#ifndef WARPS_DRONE_DSP_DRONE_H_
#define WARPS_DRONE_DSP_DRONE_H_

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
    sample_rate_ = sample_rate;
    voices_.Init(sample_rate);
    modal_.Init(sample_rate);
    delay_.Init();
    reverb_.Init(reverb_buffer, sample_rate);
    svf_.Init();

    // Defaults - pretty calm cathedral on boot.
    parameters_.pitch         = 0.30f;    // ~80 Hz root
    parameters_.damping       = 0.30f;
    parameters_.excite_amount = 0.55f;    // moderate continuous excitation
    parameters_.voicing       = 0.80f;    // zone 6 = MAJOR
    parameters_.harmonics     = 0.40f;
    parameters_.brightness    = 0.60f;
    parameters_.filter_cutoff = 0.60f;
    parameters_.filter_morph  = 0.00f;   // LP
    parameters_.delay_time    = 0.35f;
    parameters_.delay_fb      = 0.50f;
    parameters_.reverb_size   = 0.85f;
    parameters_.reverb_amount = 0.60f;
    parameters_.reverb_to_ks  = 0.30f;

    // Bootstrap pluck on power-up so we hear something immediately
    // even before the continuous noise floor fills the strings.
    Pluck();
  }

  // Re-pluck all KS voices. Soft 10-ms noise burst - gentle attack
  // that adds character without dwarfing the steady-state drone.
  inline void Pluck() {
    voices_.Excite(0.18f, static_cast<int>(0.010f * sample_rate_));
  }

  DroneParameters* mutable_parameters() { return &parameters_; }

  void Process(FloatFrame* in_out, size_t n) {
    // Map normalised params into DSP units once per block.
    const float root_hz = 40.0f * std::pow(11.0f, parameters_.pitch);
    voices_.set_chord(root_hz, parameters_.voicing);
    // Damping curve, rebuilt for "always pad-like" feel:
    //   CCW (damping=0) -> decay 0.99, bright, no extra noise   (clean long)
    //   CW  (damping=1) -> decay 1.0,  dark,   extra noise feed  (longer + noisy)
    // The old "short" character is gone - both ends sustain; the knob
    // morphs between clean-bright pad and noise-saturated drone.
    const float d = parameters_.damping;
    voices_.set_decay(0.990f + 0.010f * d);
    voices_.set_brightness(d);

    // Noise injection per sample. Three contributions:
    //   - excite_amount²·0.015  - primary continuous "drone fuel"
    //   - damping²·0.012        - additional noise mixed in at high damping
    //   - reverb-tail × smear   - slow swelling re-injection from the tail.
    //     Scaled down (was 0.08) to avoid self-oscillation locking into a
    //     fixed-rate pulse. The reverb_size knob now sets the loop period
    //     (bigger reverb -> slower swells), the smear knob the intensity.
    const float e = parameters_.excite_amount;
    voices_.set_noise_floor(e * e * 0.015f +
                            d * d * 0.012f +
                            reverb_send_ * parameters_.reverb_to_ks * 0.025f);

    modal_.set_fundamental(root_hz);
    modal_.set_brightness(parameters_.brightness);
    modal_.set_stiffness(0.2f * parameters_.brightness);

    // SVF: cutoff log 60 Hz..12 kHz, q fixed mid.
    const float fc = 60.0f * std::pow(200.0f, parameters_.filter_cutoff);
    svf_.set_f_q<stmlib::FREQUENCY_FAST>(fc / sample_rate_, 1.2f);

    delay_.set_delay(sample_rate_ *
                     (0.02f + 0.48f * parameters_.delay_time));
    delay_.set_feedback(0.05f + 0.85f * parameters_.delay_fb);
    delay_.set_tone(0.7f);

    reverb_.set_amount(parameters_.reverb_amount);
    reverb_.set_input_gain(0.5f);
    reverb_.set_time(0.5f + 0.49f * parameters_.reverb_size);
    reverb_.set_diffusion(0.7f);
    reverb_.set_lp(0.6f);

    // Snapshot audio input R (envelope follower for trigger detection).
    // We have to grab it BEFORE the loop overwrites in_out with synth
    // output. We also use in_out[i].l as audio excitement per-sample,
    // read just-in-time before each voice tick.
    constexpr float kAudioExcite = 0.10f;
    float env_in_acc = 0.0f;
    const float morph = parameters_.filter_morph;
    for (size_t i = 0; i < n; ++i) {
      const float audio_l = in_out[i].l;
      const float audio_r = in_out[i].r;

      // Envelope follower on audio in R - fast attack (1 ms-ish), slow
      // release. Tracks magnitude so triggers, gates and LFOs all look
      // the same (rising-edge of magnitude).
      const float rect = audio_r < 0.0f ? -audio_r : audio_r;
      const float c = rect > env_r_ ? 0.20f : 0.002f;
      env_r_ += c * (rect - env_r_);
      env_in_acc += rect;

      // 1. Voice bank, with audio_l mixed into each string's loop input.
      const float voice = voices_.Process(audio_l * kAudioExcite);
      // 2. Modal resonator excited by the voice.
      const float modal_out = modal_.Process(voice);
      // 3. Mix.
      float mixed = voice + modal_out * parameters_.harmonics;
      // 4. SVF morph (LP at morph=0 -> HP at morph=1).
      float lp, hp;
      svf_.Process<stmlib::FILTER_MODE_LOW_PASS,
                   stmlib::FILTER_MODE_HIGH_PASS>(mixed, &lp, &hp);
      float filtered = lp + (hp - lp) * morph;
      // 5. Tape delay (currently defeated - see prior comment).
      float delay_wet = delay_.Process(filtered);
      float pre_reverb = filtered + delay_wet * 0.0f;
      // 6. Write stereo output (mono for now).
      in_out[i].l = pre_reverb;
      in_out[i].r = pre_reverb;
    }

    // Schmitt on the audio-in-R envelope. ~10 ms refractory keeps a
    // single edge from firing twice; thresholds are well above codec
    // self-noise so an unpatched jack stays quiet.
    if (trig_refractory_ > 0) {
      --trig_refractory_;
    } else if (!trig_armed_ && env_r_ > 0.12f) {
      Pluck();
      trig_flag_       = true;
      trig_armed_      = true;
      trig_refractory_ = 15;            // ~10 ms at 1.5 kHz block rate
    } else if (trig_armed_ && env_r_ < 0.04f) {
      trig_armed_ = false;
    }

    // 7. Plate reverb in place (handles wet/dry blend internally).
    reverb_.Process(in_out, n);

    // 8. Update the reverb-tail feedback send for next block.
    //    EMA-smoothed RMS-ish quantity from the reverb output energy.
    const float new_send = std::sqrt(reverb_.tail_energy() / static_cast<float>(n));
    reverb_send_ += 0.1f * (new_send - reverb_send_);
  }

  inline float       reverb_send()  const { return reverb_send_; }
  inline VoicingZone voicing_zone() const { return voices_.zone(); }

  // True (once) on each rising edge detected on audio in R. Caller
  // uses this to flash the trigger LED.
  inline bool TakeTriggerFlag() {
    const bool f = trig_flag_;
    trig_flag_ = false;
    return f;
  }

 private:
  float            sample_rate_ = 48000.0f;
  DroneParameters  parameters_;
  VoiceBank        voices_;
  ModalBank        modal_;
  TapeDelay        delay_;
  PlateReverb      reverb_;
  stmlib::Svf      svf_;
  float            reverb_send_     = 0.0f;

  // Audio-in R envelope follower + schmitt state.
  float            env_r_           = 0.0f;
  bool             trig_armed_      = false;
  bool             trig_flag_       = false;
  uint16_t         trig_refractory_ = 0;

  DISALLOW_COPY_AND_ASSIGN(Drone);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_DRONE_H_
