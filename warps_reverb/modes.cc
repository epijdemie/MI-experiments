#include "warps_reverb/modes.h"

namespace warps_reverb {

float* WriteSlot(ReverbParameters* p, ParameterId id) {
  switch (id) {
    case PARAM_DECAY:      return &p->decay;
    case PARAM_TONE:       return &p->tone;
    case PARAM_SIZE:       return &p->size;
    case PARAM_DRY_WET:    return &p->dry_wet;
    case PARAM_PRE_DELAY:  return &p->pre_delay;
    case PARAM_DIFFUSION:  return &p->diffusion;
    case PARAM_MODULATION: return &p->modulation;
    case PARAM_LOW_CUT:    return &p->low_cut;
    default:               return &p->decay;
  }
}

float ReadSlot(const ReverbParameters& p, ParameterId id) {
  switch (id) {
    case PARAM_DECAY:      return p.decay;
    case PARAM_TONE:       return p.tone;
    case PARAM_SIZE:       return p.size;
    case PARAM_DRY_WET:    return p.dry_wet;
    case PARAM_PRE_DELAY:  return p.pre_delay;
    case PARAM_DIFFUSION:  return p.diffusion;
    case PARAM_MODULATION: return p.modulation;
    case PARAM_LOW_CUT:    return p.low_cut;
    default:               return 0.0f;
  }
}

const PanelConfig kConfig = {
  .defaults = {
    .decay      = 0.6f,     // curve maps this to loop gain ≈ 0.84 → ~4 s tail
    .tone       = 0.5f,     // neutral tilt
    .size       = 0.85f,    // mostly-hadamard matrix - smooth wash, not combs
    .dry_wet    = 0.5f,
    .pre_delay  = 0.0f,
    .diffusion  = 0.85f,    // strong in-loop diffusion (kills modal ringing)
    .modulation = 0.4f,     // ~2 ms swing - meaningful modal smear
    .low_cut    = 0.3f,     // ~40 Hz hp - kills sub buildup at high decay
  },
  .algorithm = { PARAM_DECAY,   PARAM_PRE_DELAY  },
  .timbre    = { PARAM_TONE,    PARAM_DIFFUSION  },
  .level1    = { PARAM_SIZE,    PARAM_MODULATION },
  .level2    = { PARAM_DRY_WET, PARAM_LOW_CUT    },
};

}  // namespace warps_reverb
