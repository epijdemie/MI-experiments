// live params for the single-mode high-end reverb. all 0..1 unless noted

#ifndef WARPS_REVERB_DSP_PARAMETERS_H_
#define WARPS_REVERB_DSP_PARAMETERS_H_

#include "stmlib/stmlib.h"

namespace warps_reverb {

struct ReverbParameters {
  // performance layer
  float decay;          // fb gain. 1.0 = freeze. saturator catches overshoot
  float echo_feedback;  // tape-echo feedback BEFORE the reverb (0..0.95).
                        // each repeat gets washed by the reverb → evolving
                        // wave patterns. 0 = no echo, 1 = long sustaining loop
  float size;           // delay scale (matrix locked at full hadamard)
  float dry_wet;        // mix

  // sculpt layer (shifted)
  float pre_delay;    // 0..50ms front-of-chain delay
  float echo_time;    // tape echo delay length, ~30..200ms
  float spread;       // chord harmonics intensity. 4 granular pitch shifters
                      // (3rd / 5th / 7th / 9th up) read from echo buffer,
                      // sum into wet_in. 0 = off, 1 = full chord stack
  float low_cut;      // hp corner in fb path (kills sub buildup)
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_PARAMETERS_H_
