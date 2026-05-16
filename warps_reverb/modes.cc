#include "warps_reverb/modes.h"

namespace warps_reverb {

// PARAM_NONE sink - unbound mapping slots dispatch here
static float g_param_sink = 0.0f;

float* WriteSlot(ReverbParameters* p, ParameterId id) {
  switch (id) {
    case PARAM_DECAY:     return &p->decay;
    case PARAM_SIZE:      return &p->size;
    case PARAM_DIFFUSION: return &p->diffusion;
    case PARAM_MOTION:    return &p->motion;
    case PARAM_SPEED:     return &p->speed;
    case PARAM_PRE_DELAY: return &p->pre_delay;
    case PARAM_DRY_WET:   return &p->dry_wet;
    case PARAM_SHIMMER:   return &p->shimmer;
    case PARAM_TONE:      return &p->tone;
    case PARAM_FILTER:    return &p->filter;
    case PARAM_NOISE:     return &p->noise;
    case PARAM_DRIVE:     return &p->drive;
    case PARAM_NONE:
    default:              return &g_param_sink;
  }
}

float ReadSlot(const ReverbParameters& p, ParameterId id) {
  switch (id) {
    case PARAM_DECAY:     return p.decay;
    case PARAM_SIZE:      return p.size;
    case PARAM_DIFFUSION: return p.diffusion;
    case PARAM_MOTION:    return p.motion;
    case PARAM_SPEED:     return p.speed;
    case PARAM_PRE_DELAY: return p.pre_delay;
    case PARAM_DRY_WET:   return p.dry_wet;
    case PARAM_SHIMMER:   return p.shimmer;
    case PARAM_TONE:      return p.tone;
    case PARAM_FILTER:    return p.filter;
    case PARAM_NOISE:     return p.noise;
    case PARAM_DRIVE:     return p.drive;
    case PARAM_NONE:
    default:              return 0.0f;
  }
}

const ModeConfig kModeTable[MODE_COUNT] = {

  // mode 0: clean reverb (LED off).
  // hall delay lines, unity fb ceiling, matrix-gain comp in fb path
  {
    .name = "reverb",
    .defaults = {
      .size      = 0.7f,
      .decay     = 0.6f,
      .diffusion = 0.5f,
      .motion    = 0.15f,
      .speed     = 0.2f,
      .pre_delay = 0.0f,
      .dry_wet   = 0.5f,
      .shimmer   = 0.0f,
      .tone      = 0.7f,
      .filter    = 0.0f,
      .noise     = 0.0f,
      .drive     = 0.0f,
      .freeze    = false,
    },
    .algorithm = { PARAM_DECAY,   PARAM_SHIMMER },
    .timbre    = { PARAM_SIZE,    PARAM_DIFFUSION },
    .level1    = { PARAM_MOTION,  PARAM_SPEED },
    .level2    = { PARAM_DRY_WET, PARAM_PRE_DELAY },
    .dsp = {
      .feedback_min        = 0.0f,
      .feedback_max        = 1.0f,
      .matrix_comp         = 0.7f,
      .decouple_tilt       = false,
      .matrix_alpha        = -1.0f,
      .show_clip_warning   = true,
      .osc_amplitude       = 0.0f,
      .noise_amplitude     = 0.0f,
      .motion_wobbles_tilt = false,
    },
    .osc_red = 0, .osc_green = 0,
  },

  // mode 1: drone (LED green) - ks string + outer fb loop per channel.
  // decay = fb gain (activates ~0.6+). decouple_tilt selects ks path
  {
    .name = "drone",
    .defaults = {
      .size      = 0.5f,
      .decay     = 0.75f,
      .diffusion = 0.5f,
      .motion    = 0.3f,          // fb_delay (~30ms)
      .speed     = 0.0f,
      .pre_delay = 0.35f,         // pitch (~85Hz)
      .dry_wet   = 0.5f,
      .shimmer   = 0.0f,
      .tone      = 0.6f,
      .filter    = 0.55f,         // outer-loop lp
      .noise     = 0.5f,
      .drive     = 0.0f,
      .freeze    = false,
    },
    .algorithm = { PARAM_PRE_DELAY, PARAM_FILTER },
    .timbre    = { PARAM_TONE,      PARAM_MOTION },
    .level1    = { PARAM_DECAY,     PARAM_DRIVE },
    .level2    = { PARAM_DRY_WET,   PARAM_NOISE },
    .dsp = {
      .feedback_min        = 0.0f,
      .feedback_max        = 1.0f,
      .matrix_comp         = 0.7f,
      .decouple_tilt       = true,
      .matrix_alpha        = 1.0f,
      .show_clip_warning   = false,
      .osc_amplitude       = 0.0f,
      .noise_amplitude     = 0.0f,
      .motion_wobbles_tilt = false,
    },
    .osc_red = 0, .osc_green = 255,
  },

  // mode 2: harsh / erbe-overshoot (LED orange).
  // same dsp path as mode 0 but fb_max=1.25, no matrix comp - saturator
  // engages hot. timbre primary = diffusion (vs size)
  {
    .name = "first-audible",
    .defaults = {
      .size      = 0.5f,
      .decay     = 0.7f,
      .diffusion = 0.0f,
      .motion    = 0.05f,
      .speed     = 0.2f,
      .pre_delay = 0.0f,
      .dry_wet   = 0.5f,
      .shimmer   = 0.0f,
      .tone      = 0.5f,
      .filter    = 0.0f,
      .noise     = 0.0f,
      .drive     = 0.0f,
      .freeze    = false,
    },
    .algorithm = { PARAM_DECAY,     PARAM_TONE },
    .timbre    = { PARAM_DIFFUSION, PARAM_SIZE },
    .level1    = { PARAM_MOTION,    PARAM_SPEED },
    .level2    = { PARAM_DRY_WET,   PARAM_PRE_DELAY },
    .dsp = {
      .feedback_min        = 0.0f,
      .feedback_max        = 1.25f,
      .matrix_comp         = 1.0f,
      .decouple_tilt       = false,
      .matrix_alpha        = -1.0f,
      .show_clip_warning   = true,
      .osc_amplitude       = 0.0f,
      .noise_amplitude     = 0.0f,
      .motion_wobbles_tilt = false,
    },
    .osc_red = 255, .osc_green = 255,  // orange
  },
};

}  // namespace warps_reverb
