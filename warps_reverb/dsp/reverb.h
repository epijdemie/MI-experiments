// 4-line fdn reverb. hybrid memory layout for tail quality:
//   fxengine buffer (float32, 8192 samples = 32 KB) lives in CCM: pre-delay,
//   input diffuser, in-loop branch APs, output diffuser. dma-isolated +
//   single-cycle.
//   fdn tank (4 × float[≈3200..6500] ≈ 76 KB) lives in main SRAM, read with
//   linear interpolation + own LFOs. float32 storage keeps long tails clean
//   to the noise floor of the math, not the storage format.

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

  // fxengine buffer = 8192 × float32 = 32 KB in CCM. holds pre-delay (2400)
  // + input diffuser (1464) + branch APs (1024) + output diffuser (1590)
  // = 6478 used of 8192
  static constexpr size_t kBufferSize = 8192;

  // per-line tank delay buffers. sizes = base * 1.4 + 240 mod headroom,
  // rounded up. these live in main sram (default placement)
  static constexpr size_t kTankSize0 = 3200;   // line 0 (base 2089)
  static constexpr size_t kTankSize1 = 4256;   // line 1 (base 2843)
  static constexpr size_t kTankSize2 = 5344;   // line 2 (base 3617)
  static constexpr size_t kTankSize3 = 6464;   // line 3 (base 4421)

 private:
  typedef clouds::FxEngine<kBufferSize, clouds::FORMAT_32_BIT> Engine;
  Engine engine_;

  MixingMatrix matrix_;

  // per-branch filter state (small, lives wherever class does)
  float lpf_state_[4];     // air-absorption lp (fixed corner)
  float hpf_state_[4];     // low-cut hp

  // tape-echo line before the input diffuser. fixed 200 ms delay,
  // feedback amount = echo_feedback knob (TIMBRE)
  static constexpr size_t kEchoSize = 9600;   // 200 ms @ 48k
  float echo_buffer_[kEchoSize];
  int   echo_w_;

  // own lfos for tank reads (fxengine lfos aren't publicly accessible)
  float lfo_phase_[2];
  float lfo_phase_inc_[2];

  // tank buffers - main sram. each modulated, linear-interp read
  float tank_0_[kTankSize0];
  float tank_1_[kTankSize1];
  float tank_2_[kTankSize2];
  float tank_3_[kTankSize3];
  int   tank_w_[4];

  // tick -> process cache
  float coef_input_gain_;
  float coef_pre_delay_samples_;
  float coef_input_diff_a_;       // dattorro-style input ap gains
  float coef_input_diff_b_;
  float coef_branch_diff_;        // in-loop ap gain
  float coef_feedback_;
  float coef_low_cut_hp_;
  float coef_echo_feedback_;
  float coef_dry_wet_;
  float coef_mod_amplitude_;

  float sample_rate_;
  float peak_;
  float peak_block_;

  ReverbParameters parameters_;

  DISALLOW_COPY_AND_ASSIGN(Reverb);
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_REVERB_H_
