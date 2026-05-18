// live params for the single-mode high-end reverb. all 0..1 unless noted

#ifndef WARPS_REVERB_DSP_PARAMETERS_H_
#define WARPS_REVERB_DSP_PARAMETERS_H_

#include "stmlib/stmlib.h"

namespace warps_reverb {

struct ReverbParameters {
  // performance layer
  float decay;          // fb gain. 1.0 = freeze. saturator catches overshoot
  float output_cutoff;  // post-reverb biquad lp cutoff (100 Hz..18 kHz log)
  float size;           // tank delay scaling
  float dry_wet;        // mix

  // sculpt layer (shifted)
  float pre_delay;      // 0..50ms front-of-chain delay
  float resonance;      // post-reverb biquad Q (0 = flat, 1 = ringing)
  float spectral;       // in-loop lp modulation depth - subtle spectral
                        // breathing (4 incommensurate slow cos oscs)
  float low_cut;        // hp corner in fb path (kills sub buildup)
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_PARAMETERS_H_
