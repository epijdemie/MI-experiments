// live params for the single-mode high-end reverb. all 0..1 unless noted

#ifndef WARPS_REVERB_DSP_PARAMETERS_H_
#define WARPS_REVERB_DSP_PARAMETERS_H_

#include "stmlib/stmlib.h"

namespace warps_reverb {

struct ReverbParameters {
  // performance layer
  float decay;        // fb gain. 1.0 = freeze. saturator catches overshoot
  float diffusion;    // input/output ap gain - echo→smear
  float size;         // tank delay scaling (matrix locked at full hadamard)
  float dry_wet;      // mix

  // sculpt layer (shifted)
  float pre_delay;    // 0..50ms front-of-chain delay
  float damping;      // tail brightness - per-branch lp corner
  float spectral;     // spectral dynamics depth. 4 slow cos oscillators at
                      // incommensurate rates modulate per-branch lp cutoff
                      // → tail spectrum breathes/evolves over many seconds
                      // without pitch movement
  float low_cut;      // hp corner in fb path (kills sub buildup)
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_PARAMETERS_H_
