// live params + mode-specific dsp overrides

#ifndef WARPS_REVERB_DSP_PARAMETERS_H_
#define WARPS_REVERB_DSP_PARAMETERS_H_

#include "stmlib/stmlib.h"

namespace warps_reverb {

// all ranges 0..1 unless noted. modes live in the ui layer
struct ReverbParameters {
  float size;        // room -> cathedral
  float decay;       // fb gain (scaled per-mode)
  float diffusion;   // echo -> smear
  float motion;      // lfo depth (also tilt-lpf in mode 0)
  float speed;       // 0.1..30 Hz lfo
  float pre_delay;   // 0..100 ms (drone mode: osc pitch)
  float dry_wet;
  float shimmer;     // stub
  float tone;        // HF damping (mode 0)
  float filter;      // svf morph LP↔HP (drone)
  float noise;       // 0..mode-max
  float drive;       // pre-sat gain in fb path

  bool freeze;
};

// per-mode coefficients - presets in modes.cc
struct DspOverrides {
  // decay maps linearly: feedback_min..feedback_max
  float feedback_min;
  float feedback_max;

  // per-channel fb scaling vs matrix peak gain. 0.7 ≈ 1/sqrt(2)
  float matrix_comp;

  // true: motion drives lfo depth only, tone drives the loop lpf
  bool decouple_tilt;

  // ≥0: fixed alpha. <0: alpha tracks size knob
  float matrix_alpha;

  bool show_clip_warning;

  // drone-bootstrap osc amplitude. 0 = off. freq = parameters_.pre_delay
  float osc_amplitude;
  float noise_amplitude;

  // motion also wobbles per-branch tilt lpf at the speed-lfo rate
  bool motion_wobbles_tilt;
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_PARAMETERS_H_
