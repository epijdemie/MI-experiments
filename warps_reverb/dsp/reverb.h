// 4-line fdn reverb on clouds' fxengine. per branch: ap diffuser -> modulated
// delay -> 1-pole lpf. M(α) cross-couples (α=0 identity -> α=1 hadamard).
// smooth-saturated feedback path bounds gain at decay=1.0 (no runaway).

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

  // buffer = kBufferSize × uint16_t in main sram
  void Init(uint16_t* buffer, float sample_rate);

  void Tick();
  void Process(FloatFrame* in_out, size_t size);

  ReverbParameters* mutable_parameters() { return &parameters_; }

  // decayed wet peak. >0.95 = soft-limit clipping warning
  inline float peak() const { return peak_; }

  // fxengine buffer = 64k (32768 × u16). all delay lines must fit
  static constexpr size_t kBufferSize = 32768;

 private:
  typedef clouds::FxEngine<kBufferSize, clouds::FORMAT_16_BIT> Engine;
  Engine engine_;

  MixingMatrix matrix_;

  float lpf_state_[4];

  // tick -> process cache
  float coef_input_gain_;
  float coef_pre_delay_samples_;
  float coef_diffusion_;
  float coef_feedback_;
  float coef_tilt_lpf_;
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
