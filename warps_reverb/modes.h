// single-mode panel config. fixed knob mapping, two-layer (shift = button held)
//
//                unshifted        shifted
//   ALGORITHM    decay            pre-delay
//   TIMBRE      echo feedback    echo time
//   LEVEL 1     size             chord harmonics
//   LEVEL 2     mix              low-cut

#ifndef WARPS_REVERB_MODES_H_
#define WARPS_REVERB_MODES_H_

#include "stmlib/stmlib.h"

#include "warps_reverb/dsp/parameters.h"

namespace warps_reverb {

enum ParameterId {
  PARAM_DECAY,
  PARAM_ECHO_FEEDBACK,
  PARAM_SIZE,
  PARAM_DRY_WET,
  PARAM_PRE_DELAY,
  PARAM_ECHO_TIME,
  PARAM_SPREAD,
  PARAM_LOW_CUT,
  PARAM_LAST
};

struct PotMapping {
  ParameterId unshifted;
  ParameterId shifted;
};

struct PanelConfig {
  ReverbParameters defaults;
  PotMapping algorithm;
  PotMapping timbre;
  PotMapping level1;
  PotMapping level2;
};

extern const PanelConfig kConfig;

float* WriteSlot(ReverbParameters* p, ParameterId id);
float  ReadSlot(const ReverbParameters& p, ParameterId id);

}  // namespace warps_reverb

#endif  // WARPS_REVERB_MODES_H_
