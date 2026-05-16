// single ks string voice.
//   read -> 2-tap damp (avg lpf) -> ×decay + input -> write.
// pitch = sr / delay; fractional read for smooth tuning.
// Excite() injects bandlimited noise burst

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
  // 40–500 Hz range we actually use
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
    excite_amount_  = 0.0f;
    excite_phase_   = 0;
    excite_length_  = 1;
    excite_shape_   = 0.0f;
    excite_active_  = false;
    pink_.Init();
  }

  // Set pitch by delay length in samples (fractional). Caller derives
  // this from frequency: delay_len = sample_rate / freq_hz
  inline void set_delay(float delay) {
    if (delay < 4.0f)         delay = 4.0f;
    if (delay > kMaxLen - 2)  delay = kMaxLen - 2;
    delay_len_ = delay;
  }

  // Decay = in-loop gain. 0.95 ≈ short, 0.999 ≈ sustained, 1.0 = self-osc
  inline void set_decay(float d) { decay_ = d; }

  // Brightness: redistributes the 2-tap lowpass between same-sample and
  // delayed-sample. b=0 -> bright (a=1, b=0, no LPF), b=1 -> dark (a=0.5,
  // b=0.5, max smoothing)
  inline void set_brightness(float bright) {
    damp_b_ = 0.5f * (1.0f - bright);
    damp_a_ = 1.0f - damp_b_;
  }

  // Inject a noise burst whose amplitude envelope morphs through four
  // shapes across `length` samples. `shape` (0..1) selects the morph:
  //   0.00  rectangle    (constant - classic Karplus excitation)
  //   0.33  negative saw (peak -> 0 - decaying pluck)
  //   0.67  triangle     (0 -> peak -> 0)
  //   1.00  saw          (0 -> peak - rising swell)
  // The burst always lasts the full `length` regardless of shape.
  // (Order chosen so CCW = instant hit, CW = slow swell - each step is
  // sonically distinct rather than two near-equivalent decaying shapes.)
  inline void Excite(float amount, int length, float shape) {
    excite_amount_  = amount;
    excite_length_  = length > 0 ? length : 1;
    excite_shape_   = shape < 0.0f ? 0.0f : (shape > 1.0f ? 1.0f : shape);
    excite_phase_   = 0;
    excite_active_  = true;
  }

  // Continuous noise injection at the input. amount=0..1 maps to
  // ambient noise level mixed into the loop every sample
  inline void set_noise_floor(float amount) { noise_floor_ = amount; }

  // 0 = pure pink (thick, low-end weighted), 1 = pure white (bright)
  inline void set_noise_color(float mix) { noise_color_mix_ = mix; }

  // One audio-rate tick. `feedback_in` is the external signal mixed
  // into the string input (reverb tail, neighbour-voice cross-talk, etc.)
  inline float Process(float feedback_in) {
    // Fractional read from the delay line
    float d = delay_len_;
    MAKE_INTEGRAL_FRACTIONAL(d);
    int r0 = (write_ptr_ - d_integral - 1 + kMaxLen) & kMask;
    int r1 = (r0 - 1 + kMaxLen) & kMask;
    float a = buffer_[r0];
    float b = buffer_[r1];
    float y = a + (b - a) * d_fractional;

    // 2-tap lowpass (damping)
    float filt = damp_a_ * y + damp_b_ * prev_y_;
    prev_y_ = y;

    // Excitation: white-noise burst whose amplitude envelope morphs
    // between rectangle / saw / triangle / negative-saw across the
    // full burst length. Continuous noise floor is independent and
    // always pink/white-mixed
    float exc = 0.0f;
    if (excite_active_) {
      if (excite_phase_ >= excite_length_) {
        excite_active_ = false;
      } else {
        const float t = static_cast<float>(excite_phase_) /
                        static_cast<float>(excite_length_);
        const float rect = 1.0f;
        const float saw  = t;
        const float tri  = t < 0.5f ? 2.0f * t : 2.0f * (1.0f - t);
        const float nsaw = 1.0f - t;
        float env;
        const float bs = excite_shape_;
        if (bs < 0.333333f) {
          const float u = bs * 3.0f;
          env = rect + (nsaw - rect) * u;
        } else if (bs < 0.666667f) {
          const float u = (bs - 0.333333f) * 3.0f;
          env = nsaw + (tri - nsaw) * u;
        } else {
          const float u = (bs - 0.666667f) * 3.0f;
          env = tri + (saw - tri) * u;
        }
        exc = excite_amount_ * env *
              (2.0f * stmlib::Random::GetFloat() - 1.0f);
        ++excite_phase_;
      }
    }
    if (noise_floor_ > 0.0f) {
      const float pink = pink_.Next();
      const float white = 2.0f * stmlib::Random::GetFloat() - 1.0f;
      exc += noise_floor_ *
             (pink * (1.0f - noise_color_mix_) + white * noise_color_mix_);
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
  float excite_amount_  = 0.0f;
  int   excite_phase_   = 0;
  int   excite_length_  = 1;
  float excite_shape_   = 0.0f;
  bool  excite_active_  = false;
  float noise_floor_    = 0.0f;
  float noise_color_mix_ = 0.0f;   // 0=pink, 1=white
  PinkNoise pink_;

  DISALLOW_COPY_AND_ASSIGN(KarplusVoice);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_KARPLUS_VOICE_H_
