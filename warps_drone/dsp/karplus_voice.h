// Single Karplus-Strong string voice.
//
// Topology:
//   read y = buf[read_ptr]
//   filtered = damp_a * y + damp_b * prev_y         (1-pole average, LPF)
//   prev_y = y
//   write buf[write_ptr] = filtered * decay + input
//
// Pitch = sample_rate / delay_length. delay_length is recomputed from the
// floating-point pitch with a fractional read for smooth tuning.
//
// Excitation: an attack burst of bandlimited noise injected via
// Excite(amount). The voice is otherwise driven by the `input` argument
// to Process(), which is where reverb-tail feedback and ambient noise
// can be added every sample.

#ifndef WARPS_DRONE_DSP_KARPLUS_VOICE_H_
#define WARPS_DRONE_DSP_KARPLUS_VOICE_H_

#include <algorithm>

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/utils/random.h"

#include "warps_drone/dsp/pink_noise.h"

namespace warps_drone {

class KarplusVoice {
 public:
  // 2048 float = ~23 Hz floor at 48 kHz. Plenty of headroom for the
  // 40–500 Hz range we actually use.
  static constexpr size_t kMaxLen = 2048;

  KarplusVoice() { }

  void Init() {
    std::fill(&buffer_[0], &buffer_[kMaxLen], 0.0f);
    write_ptr_  = 0;
    prev_y_     = 0.0f;
    delay_len_  = 200.0f;
    decay_      = 0.995f;
    damp_a_     = 0.5f;
    damp_b_     = 0.5f;
    excite_     = 0.0f;
    excite_phase_ = 0;
    pink_.Init();
  }

  // Set pitch by delay length in samples (fractional). Caller derives
  // this from frequency: delay_len = sample_rate / freq_hz.
  inline void set_delay(float delay) {
    if (delay < 4.0f)         delay = 4.0f;
    if (delay > kMaxLen - 2)  delay = kMaxLen - 2;
    delay_len_ = delay;
  }

  // Decay = in-loop gain. 0.95 ≈ short, 0.999 ≈ sustained, 1.0 = self-osc.
  inline void set_decay(float d) { decay_ = d; }

  // Brightness: redistributes the 2-tap lowpass between same-sample and
  // delayed-sample. b=0 -> bright (a=1, b=0, no LPF), b=1 -> dark (a=0.5,
  // b=0.5, max smoothing).
  inline void set_brightness(float bright) {
    damp_b_ = 0.5f * (1.0f - bright);
    damp_a_ = 1.0f - damp_b_;
  }

  // Inject a noise burst of duration ~delay_len_ * fraction. Called on
  // every "pluck" gesture (when you wants a re-attack) or
  // continuously at low amplitude to keep the string fed.
  inline void Excite(float amount, int duration) {
    excite_       = amount;
    excite_phase_ = duration;
  }

  // Continuous noise injection at the input. amount=0..1 maps to
  // ambient noise level mixed into the loop every sample.
  inline void set_noise_floor(float amount) { noise_floor_ = amount; }

  // One audio-rate tick. `feedback_in` is the external signal mixed
  // into the string input (reverb tail, neighbour-voice cross-talk, etc.).
  inline float Process(float feedback_in) {
    // Fractional read from the delay line.
    float d = delay_len_;
    MAKE_INTEGRAL_FRACTIONAL(d);
    int r0 = (write_ptr_ - d_integral - 1 + kMaxLen) & kMask;
    int r1 = (r0 - 1 + kMaxLen) & kMask;
    float a = buffer_[r0];
    float b = buffer_[r1];
    float y = a + (b - a) * d_fractional;

    // 2-tap lowpass (damping).
    float filt = damp_a_ * y + damp_b_ * prev_y_;
    prev_y_ = y;

    // Excitation: pluck transient uses white noise (sharp attack), the
    // continuous noise floor uses pink for a thick, low-end-weighted
    // texture that excites the string's lower harmonics more strongly.
    float exc = 0.0f;
    if (excite_phase_ > 0) {
      exc = excite_ * (2.0f * stmlib::Random::GetFloat() - 1.0f);
      --excite_phase_;
    }
    if (noise_floor_ > 0.0f) {
      exc += noise_floor_ * pink_.Next();
    }

    float input = filt * decay_ + exc + feedback_in;
    buffer_[write_ptr_] = input;
    write_ptr_ = (write_ptr_ + 1) & kMask;

    return filt;
  }

 private:
  static constexpr int kMask = kMaxLen - 1;  // requires kMaxLen power of 2

  float buffer_[kMaxLen];
  int   write_ptr_;
  float prev_y_;
  float delay_len_;
  float decay_;
  float damp_a_;
  float damp_b_;
  float excite_;
  int   excite_phase_;
  float noise_floor_ = 0.0f;
  PinkNoise pink_;

  DISALLOW_COPY_AND_ASSIGN(KarplusVoice);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_KARPLUS_VOICE_H_
