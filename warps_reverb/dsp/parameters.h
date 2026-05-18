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
  float diffusion;    // input/output ap gain
  float spread;       // chord-spread depth. 4 slow lfos at incommensurate
                      // rates (0.07-0.18 Hz) detune the lines apart. at 0,
                      // no movement; at 1, lines drift across ~5 ms range
                      // independently → harmonic relationships open up
  float low_cut;      // hp corner in fb path (kills sub buildup)
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_PARAMETERS_H_
