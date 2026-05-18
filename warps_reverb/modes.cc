#include "warps_reverb/modes.h"

namespace warps_reverb {

float* WriteSlot(ReverbParameters* p, ParameterId id) {
  switch (id) {
    case PARAM_DECAY:         return &p->decay;
    case PARAM_OUTPUT_CUTOFF: return &p->output_cutoff;
    case PARAM_SIZE:          return &p->size;
    case PARAM_DRY_WET:       return &p->dry_wet;
    case PARAM_PRE_DELAY:     return &p->pre_delay;
    case PARAM_RESONANCE:     return &p->resonance;
    case PARAM_SPECTRAL:      return &p->spectral;
    case PARAM_LOW_CUT:       return &p->low_cut;
    default:                  return &p->decay;
  }
}

float ReadSlot(const ReverbParameters& p, ParameterId id) {
  switch (id) {
    case PARAM_DECAY:         return p.decay;
    case PARAM_OUTPUT_CUTOFF: return p.output_cutoff;
    case PARAM_SIZE:          return p.size;
    case PARAM_DRY_WET:       return p.dry_wet;
    case PARAM_PRE_DELAY:     return p.pre_delay;
    case PARAM_RESONANCE:     return p.resonance;
    case PARAM_SPECTRAL:      return p.spectral;
    case PARAM_LOW_CUT:       return p.low_cut;
    default:                  return 0.0f;
  }
}

const PanelConfig kConfig = {
  .defaults = {
    .decay         = 0.6f,    // ~4 s tail at default
    .output_cutoff = 0.85f,   // ~12 kHz - mostly open
    .size          = 0.85f,
    .dry_wet       = 0.5f,
    .pre_delay     = 0.0f,
    .resonance     = 0.0f,    // flat by default
    .spectral      = 0.3f,    // subtle spectral breathing
    .low_cut       = 0.3f,
  },
  .algorithm = { PARAM_DECAY,         PARAM_PRE_DELAY },
  .timbre    = { PARAM_OUTPUT_CUTOFF, PARAM_RESONANCE },
  .level1    = { PARAM_SIZE,          PARAM_SPECTRAL  },
  .level2    = { PARAM_DRY_WET,       PARAM_LOW_CUT   },
};

}  // namespace warps_reverb
