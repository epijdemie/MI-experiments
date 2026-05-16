// Mono tape-style delay with feedback and 1-pole tone control on the
// loop. Fractional read for smooth time-modulation.
//
// Buffer size 24576 samples ≈ 512 ms at 48 kHz, stored as int16 to save
// space (48 KB of buffer). Plenty for ~20 ms…500 ms range.

#ifndef WARPS_DRONE_DSP_TAPE_DELAY_H_
#define WARPS_DRONE_DSP_TAPE_DELAY_H_

#include <algorithm>

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"

namespace warps_drone {

class TapeDelay {
 public:
  static constexpr size_t kSize = 24576;  // power of 2 -> fast mask

  TapeDelay() { }

  void Init() {
    std::fill(&buffer_[0], &buffer_[kSize], 0);
    write_ptr_ = 0;
    delay_     = 6000.0f;
    feedback_  = 0.5f;
    tone_      = 0.5f;
    lp_state_  = 0.0f;
  }

  inline void set_delay(float samples) {
    if (samples < 4.0f)         samples = 4.0f;
    if (samples > kSize - 2)    samples = kSize - 2;
    delay_ = samples;
  }

  inline void set_feedback(float f) { feedback_ = f; }

  // 0 = dark loop, 1 = bright loop.
  inline void set_tone(float t) { tone_ = t; }

  inline float Process(float in) {
    float d = delay_;
    MAKE_INTEGRAL_FRACTIONAL(d);
    int r0 = (write_ptr_ - d_integral - 1 + kSize) & kMask;
    int r1 = (r0 - 1 + kSize) & kMask;
    float a = static_cast<float>(buffer_[r0]) * (1.0f / 32768.0f);
    float b = static_cast<float>(buffer_[r1]) * (1.0f / 32768.0f);
    float read = a + (b - a) * d_fractional;

    // 1-pole LPF on the feedback path; coefficient = tone (0 dark, 1 bright).
    lp_state_ += (0.05f + 0.85f * tone_) * (read - lp_state_);
    float wet = lp_state_;

    float to_write = in + wet * feedback_;
    // Soft-clip pre-store to keep int16 range safe even under high FB.
    to_write = stmlib::SoftLimit(to_write);
    int32_t s = static_cast<int32_t>(to_write * 32767.0f);
    if (s >  32767) s =  32767;
    if (s < -32767) s = -32767;
    buffer_[write_ptr_] = static_cast<int16_t>(s);
    write_ptr_ = (write_ptr_ + 1) & kMask;

    return wet;
  }

 private:
  static constexpr int kMask = kSize - 1;
  int16_t buffer_[kSize];
  int     write_ptr_;
  float   delay_;
  float   feedback_;
  float   tone_;
  float   lp_state_;

  DISALLOW_COPY_AND_ASSIGN(TapeDelay);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_TAPE_DELAY_H_
