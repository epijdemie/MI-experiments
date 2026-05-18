#include "warps_reverb/modes.h"

namespace warps_reverb {

float* WriteSlot(ReverbParameters* p, ParameterId id) {
  switch (id) {
    case PARAM_DECAY:         return &p->decay;
    case PARAM_ECHO_FEEDBACK: return &p->echo_feedback;
    case PARAM_SIZE:          return &p->size;
    case PARAM_DRY_WET:    return &p->dry_wet;
    case PARAM_PRE_DELAY:  return &p->pre_delay;
    case PARAM_ECHO_TIME:  return &p->echo_time;
    case PARAM_SPREAD: return &p->spread;
    case PARAM_LOW_CUT:    return &p->low_cut;
    default:               return &p->decay;
  }
}

float ReadSlot(const ReverbParameters& p, ParameterId id) {
  switch (id) {
    case PARAM_DECAY:         return p.decay;
    case PARAM_ECHO_FEEDBACK: return p.echo_feedback;
    case PARAM_SIZE:          return p.size;
    case PARAM_DRY_WET:    return p.dry_wet;
    case PARAM_PRE_DELAY:  return p.pre_delay;
    case PARAM_ECHO_TIME:  return p.echo_time;
    case PARAM_SPREAD: return p.spread;
    case PARAM_LOW_CUT:    return p.low_cut;
    default:               return 0.0f;
  }
}

const PanelConfig kConfig = {
  .defaults = {
    .decay         = 0.6f,    // curve maps this to loop gain ≈ 0.84 → ~4 s tail
    .echo_feedback = 0.0f,    // no tape echo by default
    .size          = 0.85f,   // long-ish delays
    .dry_wet       = 0.5f,
    .pre_delay     = 0.0f,
    .echo_time     = 0.7f,    // ~145 ms - classic tape echo territory
    .spread        = 0.3f,    // gentle harmonic spread by default
    .low_cut       = 0.3f,    // ~40 Hz hp - kills sub buildup at high decay
  },
  .algorithm = { PARAM_DECAY,         PARAM_PRE_DELAY  },
  .timbre    = { PARAM_ECHO_FEEDBACK, PARAM_ECHO_TIME  },
  .level1    = { PARAM_SIZE,    PARAM_SPREAD },
  .level2    = { PARAM_DRY_WET, PARAM_LOW_CUT    },
};

}  // namespace warps_reverb
