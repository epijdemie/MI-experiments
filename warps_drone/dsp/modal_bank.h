// Parallel modal resonator bank - 16 bandpass filters tuned to harmonics
// of a fundamental. Excited by the same signal that drives the KS voices,
// adds harmonically-clean shimmer with no IM-fog. The 1/n amplitude
// weighting follows natural overtone series.
//
// Stiffness (set_stiffness) detunes higher modes upward (bell-like) or
// downward (slack string). Brightness scales Q across the bank.

#ifndef WARPS_DRONE_DSP_MODAL_BANK_H_
#define WARPS_DRONE_DSP_MODAL_BANK_H_

#include <math.h>

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"

namespace warps_drone {

class ModalBank {
 public:
  static constexpr int kNumModes = 16;

  ModalBank() { }

  void Init(float sample_rate) {
    inv_sr_ = 1.0f / sample_rate;
    for (int i = 0; i < kNumModes; ++i) {
      modes_[i].Init();
      amp_[i] = 1.0f / (1.0f + i);  // 1/n natural weighting
    }
    fundamental_hz_ = 110.0f;
    stiffness_     = 0.0f;
    active_modes_  = kNumModes;
    pickup_pos_    = 0.5f;
    set_q(20.0f);
    RecomputeFrequencies();
    RecomputeAmps();
  }

  // Active mode count, 1..kNumModes. Higher = brighter / fuller spectrum.
  inline void set_active_modes(int count) {
    if (count < 1) count = 1;
    if (count > kNumModes) count = kNumModes;
    active_modes_ = count;
  }

  // Pickup position 0..1 - cosine-weights modes to simulate where you
  // "pluck" / pickup a string. 0 = node (suppresses even modes),
  // 0.5 = balanced, 1 = other end.
  inline void set_pickup(float p) {
    pickup_pos_ = p;
    RecomputeAmps();
  }

  // Fundamental in Hz. Modes are tuned to integer multiples (plus stiffness).
  inline void set_fundamental(float hz) {
    if (hz < 20.0f) hz = 20.0f;
    fundamental_hz_ = hz;
    RecomputeFrequencies();
  }

  // Stiffness in 0..1. 0 = ideal harmonic ratios. 1 = strongly stretched
  // (bell-like; ~10 % spread by mode 16).
  inline void set_stiffness(float s) {
    stiffness_ = s;
    RecomputeFrequencies();
  }

  // Q for the lowest mode. Higher modes' Q is loss-tapered (ramp down to
  // 0.6× by the top of the bank) so the spectrum doesn't get spiky.
  void set_q(float q_base) {
    q_base_ = q_base;
    RecomputeQs();
  }

  // 0 = dark (only low modes ring), 1 = bright (full Q across bank).
  inline void set_brightness(float b) {
    brightness_ = b;
    RecomputeQs();
  }

  // Tick one sample. Sum of BPF outputs, scaled to roughly unit gain.
  inline float Process(float in) {
    float acc = 0.0f;
    for (int i = 0; i < active_modes_; ++i) {
      acc += modes_[i].Process<stmlib::FILTER_MODE_BAND_PASS_NORMALIZED>(in)
             * amp_[i];
    }
    return acc * 0.5f;
  }

 private:
  void RecomputeFrequencies() {
    for (int i = 0; i < kNumModes; ++i) {
      float ratio = static_cast<float>(i + 1);
      // Stiffness pushes higher modes up: ratio *= (1 + s*i^1.5 * 0.01)
      const float stretch = 1.0f + stiffness_ * 0.012f *
                            ratio * (ratio - 1.0f);
      float f = fundamental_hz_ * ratio * stretch * inv_sr_;
      if (f > 0.45f) f = 0.45f;
      // Re-derive g and h. Q is set separately via RecomputeQs().
      modes_[i].set_f_q<stmlib::FREQUENCY_FAST>(f, q_table_[i]);
    }
  }

  void RecomputeQs() {
    for (int i = 0; i < kNumModes; ++i) {
      // Loss taper: full Q at the bottom, 0.55× Q at the top.
      const float taper = 1.0f - 0.45f * (static_cast<float>(i) /
                          static_cast<float>(kNumModes - 1));
      // Brightness scales Q on top.
      const float q = q_base_ * taper * (0.3f + 0.7f * brightness_);
      q_table_[i] = q > 1.0f ? q : 1.0f;
    }
    RecomputeFrequencies();
  }

  void RecomputeAmps() {
    // Base envelope = 1/(1+i) (natural overtone falloff) modulated by
    // a pickup-position raised cosine weighting.
    for (int i = 0; i < kNumModes; ++i) {
      const float natural = 1.0f / (1.0f + static_cast<float>(i));
      const float pos_angle = pickup_pos_ * 3.14159265f *
                              static_cast<float>(i + 1);
      const float pickup_w = 0.4f + 0.6f * fabsf(sinf(pos_angle));
      amp_[i] = natural * pickup_w;
    }
  }

  stmlib::Svf modes_[kNumModes];
  float       q_table_[kNumModes];
  float       amp_[kNumModes];
  float       inv_sr_;
  float       fundamental_hz_;
  float       stiffness_;
  float       q_base_     = 20.0f;
  float       brightness_ = 0.8f;
  int         active_modes_ = kNumModes;
  float       pickup_pos_   = 0.5f;

  DISALLOW_COPY_AND_ASSIGN(ModalBank);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_MODAL_BANK_H_
