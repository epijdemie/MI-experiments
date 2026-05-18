// 4-line fdn reverb. hybrid memory layout for tail quality:
//   fxengine buffer (float32, 8192 samples = 32 KB) in CCM: pre-delay,
//   input diffuser, in-loop branch APs, output diffuser. dma-isolated +
//   single-cycle.
//   fdn tank (4 × float[≈3200..6500] ≈ 76 KB) in main SRAM, linear-interp
//   reads. float32 keeps long tails clean to the math noise floor.
//
// spectral dynamics: 4 cheap recurrent cos oscillators at incommensurate
// slow rates modulate each branch's air-absorption LP cutoff. tail spectrum
// breathes/evolves over many seconds with no pitch movement, no sinf cost.

#ifndef WARPS_REVERB_DSP_REVERB_H_
#define WARPS_REVERB_DSP_REVERB_H_

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"

#include "clouds/dsp/fx/fx_engine.h"

#include "warps_reverb/dsp/mixing_matrix.h"
#include "warps_reverb/dsp/parameters.h"

namespace warps_reverb {

struct ShortFrame {
  int16_t l;
  int16_t r;
};

struct FloatFrame {
  float l;
  float r;
};

class Reverb {
 public:
  Reverb() { }
  ~Reverb() { }

  // buffer = kBufferSize × float in CCM (32 KB)
  void Init(float* buffer, float sample_rate);

  void Tick();
  void Process(FloatFrame* in_out, size_t size);

  ReverbParameters* mutable_parameters() { return &parameters_; }

  // decayed wet peak. >0.95 = soft-limit clipping warning
  inline float peak() const { return peak_; }

  static constexpr size_t kBufferSize = 8192;

  // per-line tank delay buffers. sizes = base * 1.4 + 240 mod headroom
  static constexpr size_t kTankSize0 = 3200;
  static constexpr size_t kTankSize1 = 4256;
  static constexpr size_t kTankSize2 = 5344;
  static constexpr size_t kTankSize3 = 6464;

 private:
  typedef clouds::FxEngine<kBufferSize, clouds::FORMAT_32_BIT> Engine;
  Engine engine_;

  MixingMatrix matrix_;

  // per-branch filter state
  float lpf_state_[4];     // air-absorption lp (modulated by spectral lfos)
  float hpf_state_[4];     // low-cut hp

  // 4 recurrent cosine oscillators (Goertzel-style, no sinf at runtime).
  // y[n] = c * y[n-1] - y[n-2] where c = 2*cos(2π*rate/fs)
  // ONE mul + ONE sub per sample per oscillator. fixed slow rates,
  // incommensurate so the lines never re-sync
  float osc_y1_[4];
  float osc_y2_[4];
  float osc_c_[4];

  // tank buffers - main sram
  float tank_0_[kTankSize0];
  float tank_1_[kTankSize1];
  float tank_2_[kTankSize2];
  float tank_3_[kTankSize3];
  int   tank_w_[4];

  // tick -> process cache
  float coef_input_gain_;
  float coef_pre_delay_samples_;
  float coef_input_diff_a_;
  float coef_input_diff_b_;
  float coef_branch_diff_;
  float coef_feedback_;
  float coef_low_cut_hp_;
  float coef_damping_;       // center lp coefficient
  float coef_spectral_;      // lfo modulation depth around damping center
  float coef_dry_wet_;

  float sample_rate_;
  float peak_;
  float peak_block_;

  ReverbParameters parameters_;

  DISALLOW_COPY_AND_ASSIGN(Reverb);
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_REVERB_H_
