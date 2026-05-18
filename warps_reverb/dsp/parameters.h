// live params for the single-mode high-end reverb. all 0..1 unless noted

#ifndef WARPS_REVERB_DSP_PARAMETERS_H_
#define WARPS_REVERB_DSP_PARAMETERS_H_

#include "stmlib/stmlib.h"

namespace warps_reverb {

struct ReverbParameters {
  // performance layer
  float decay;        // fb gain. 1.0 = freeze. saturator catches overshoot
  float tone;         // tilt eq in loop. 0 = dark, 1 = bright
  float size;         // delay scale + matrix identity↔hadamard morph
  float dry_wet;      // mix

  // sculpt layer (shifted)
  float pre_delay;    // 0..200ms front-of-chain delay
  float diffusion;    // input/output ap gain
  float modulation;   // single knob: lfo depth + rate
  float low_cut;      // hp corner in fb path (kills sub buildup)
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_PARAMETERS_H_
