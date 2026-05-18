#include "warps_reverb/modes.h"

namespace warps_reverb {

float* WriteSlot(ReverbParameters* p, ParameterId id) {
  switch (id) {
    case PARAM_DECAY:     return &p->decay;
    case PARAM_DIFFUSION: return &p->diffusion;
    case PARAM_SIZE:      return &p->size;
    case PARAM_DRY_WET:   return &p->dry_wet;
    case PARAM_PRE_DELAY: return &p->pre_delay;
    case PARAM_DAMPING:   return &p->damping;
    case PARAM_SPECTRAL:  return &p->spectral;
    case PARAM_LOW_CUT:   return &p->low_cut;
    default:              return &p->decay;
  }
}

float ReadSlot(const ReverbParameters& p, ParameterId id) {
  switch (id) {
    case PARAM_DECAY:     return p.decay;
    case PARAM_DIFFUSION: return p.diffusion;
    case PARAM_SIZE:      return p.size;
    case PARAM_DRY_WET:   return p.dry_wet;
    case PARAM_PRE_DELAY: return p.pre_delay;
    case PARAM_DAMPING:   return p.damping;
    case PARAM_SPECTRAL:  return p.spectral;
    case PARAM_LOW_CUT:   return p.low_cut;
    default:              return 0.0f;
  }
}

const PanelConfig kConfig = {
  .defaults = {
    .decay     = 0.6f,     // ~4 s tail at default
    .diffusion = 0.85f,    // strong smear
    .size      = 0.85f,    // long-ish delays
    .dry_wet   = 0.5f,
    .pre_delay = 0.0f,
    .damping   = 0.5f,     // ~5 kHz lp center (current default)
    .spectral  = 0.3f,     // gentle spectral breathing
    .low_cut   = 0.3f,
  },
  .algorithm = { PARAM_DECAY,     PARAM_PRE_DELAY },
  .timbre    = { PARAM_DIFFUSION, PARAM_DAMPING   },
  .level1    = { PARAM_SIZE,      PARAM_SPECTRAL  },
  .level2    = { PARAM_DRY_WET,   PARAM_LOW_CUT   },
};

}  // namespace warps_reverb
