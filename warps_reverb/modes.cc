#include "warps_reverb/modes.h"

namespace warps_reverb {

// Sink for PARAM_NONE writes. CvScaler dispatches into this when a mapping
// slot is unbound; the slot still has a pot+CV reading us, we just discard.
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

  // ---- Mode 0: clean reverb (LED off) ----
  // Hall-class delay lines, feedback ceiling at unity, matrix-gain
  // compensation in the feedback path so the saturator only engages on
  // transients. Defaults tuned for an immediate hall character on power-up.
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
      .filter    = 0.0f,           // unused in mode 0
      .noise     = 0.0f,           // unused in mode 0
      .drive     = 0.0f,           // unused in mode 0
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

  // ---- Mode 1: drone (LED green) ----
  // The first-iteration character: feedback ceiling 1.25, no matrix-gain
  // compensation, no diffusion. The FDN saturator engages continuously and
  // the loop self-oscillates musically. The big knob is bound to MOTION
  // (the parameter formerly known as Depth) because that's what produced
  // sci-fi sweeps when you was playing the self-oscillating loop -
  // sweeping LFO amplitude modulates the loop's pitch character. Speed
  // (LFO rate) sits on LEVEL 1 primary. Decay defaults high to boot us
  // straight into the self-osc territory.
  {
    .name = "drone",
    .defaults = {
      // Hecker-style starting point: 4-voice unison with mild drift, lots
      // of space, dark low-pass character, audible grain.
      .size      = 0.6f,
      .decay     = 0.7f,          // long sustaining tail by default
      .diffusion = 0.5f,
      .motion    = 0.4f,
      .speed     = 0.1f,
      .pre_delay = 0.0f,
      .dry_wet   = 1.0f,
      .shimmer   = 0.0f,
      .tone      = 0.3f,          // dark default - per-branch LPF held low
      .filter    = 0.0f,
      .noise     = 0.12f,         // gentle bed - too much seeds squeal
      .drive     = 0.0f,          // clean at default - sweep up for grit
      .freeze    = false,
    },
    .algorithm = { PARAM_DRIVE,     PARAM_DECAY },
    .timbre    = { PARAM_FILTER,    PARAM_SIZE },
    .level1    = { PARAM_SPEED,     PARAM_MOTION },
    .level2    = { PARAM_PRE_DELAY, PARAM_NOISE },
    .dsp = {
      // Long-tail drone: high feedback floor + low ceiling overshoot.
      // The per-branch LPF (always on in drone now) damps high frequencies
      // each circulation, so we can push feedback near unity without
      // the loop turning into a tea kettle.
      .feedback_min        = 0.88f,
      .feedback_max        = 1.05f,
      .matrix_comp         = 1.0f,
      .decouple_tilt       = true,
      .matrix_alpha        = 1.0f,
      .show_clip_warning   = false,
      .osc_amplitude       = 0.10f,   // slightly less drive into the loop
      .noise_amplitude     = 0.04f,
      .motion_wobbles_tilt = true,
    },
    .osc_red = 0, .osc_green = 255,
  },

  // ---- Mode 2: noise pad (LED orange) ----
  // Noise-excited stationary pad. No oscillators - filtered brown noise drives
  // the FDN directly. Orthogonal Hadamard matrix + near-unity feedback + SVF
  // recirculating -> dense smeared cloud that sustains without audio input.
  // External signal folds in via wet_in and gets absorbed into the pad.
  //
  // ALGO = Noise density, TIMBRE = Filter morph (LP->HP). Secondary mapping
  // mirrors drone so the two modes share muscle memory on shifted controls.
  {
    .name = "noise-pad",
    .defaults = {
      .size      = 0.6f,
      .decay     = 0.75f,
      .diffusion = 0.5f,
      .motion    = 0.2f,
      .speed     = 0.1f,
      .pre_delay = 0.0f,
      .dry_wet   = 1.0f,
      .shimmer   = 0.0f,
      .tone      = 0.5f,
      .filter    = 0.0f,
      .noise     = 0.5f,
      .drive     = 0.0f,
      .freeze    = false,
    },
    .algorithm = { PARAM_NOISE,   PARAM_DECAY },
    .timbre    = { PARAM_FILTER,  PARAM_SIZE },
    .level1    = { PARAM_SPEED,   PARAM_MOTION },
    .level2    = { PARAM_DRY_WET, PARAM_PRE_DELAY },
    .dsp = {
      .feedback_min        = 0.5f,
      .feedback_max        = 1.05f,
      .matrix_comp         = 1.0f,
      .decouple_tilt       = true,
      .matrix_alpha        = 1.0f,   // Hadamard locked - energy preserved
      .show_clip_warning   = false,
      .osc_amplitude       = 0.0f,
      .noise_amplitude     = 0.06f,
      .motion_wobbles_tilt = true,
    },
    .osc_red = 255, .osc_green = 255,  // orange (red + green on bicolor LED)
  },
};

}  // namespace warps_reverb
