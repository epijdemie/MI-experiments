// Live parameters and mode-specific DSP overrides for the Erbe-Verb-style
// firmware.

#ifndef WARPS_REVERB_DSP_PARAMETERS_H_
#define WARPS_REVERB_DSP_PARAMETERS_H_

#include "stmlib/stmlib.h"

namespace warps_reverb {

// No mode selector inside the params - modes are managed by the UI layer.
// All ranges normalised [0, 1] unless noted otherwise.
struct ReverbParameters {
  float size;        // 0 = tight room, 1 = cathedral
  float decay;       // feedback gain (scaled per-mode by feedback_min/max)
  float diffusion;   // 0 = discrete echoes, 1 = dense smear
  float motion;      // LFO depth (and tilt-LPF in mode 0). 0 = static.
  float speed;       // 0.1..30 Hz LFO rate
  float pre_delay;   // 0..100 ms input tap; in drone mode repurposed to
                     // control the injected oscillator pitch.
  float dry_wet;     // 0 = dry, 1 = 100 % wet
  float shimmer;     // 0 = clean, 1 = bright/saturated. Stub for now.
  float tone;        // HF damping in the feedback loop (used in mode 0 only;
                     // drone uses the SVF filter morph instead).
  float filter;      // SVF morph: 0 = LP, 1 = HP (drone mode).
  float noise;       // Noise injection amount, 0..1 -> 0..mode-specific max.
  float drive;       // Pre-saturator gain in the feedback path. 0 = clean,
                     // 1 = heavy harmonic distortion / fuzz character.

  bool freeze;
};

// Coefficients that swap per mode. See modes.cc for the actual presets.
struct DspOverrides {
  // Decay knob maps linearly between feedback_min (CCW) and feedback_max (CW).
  // Mode "reverb": (0, 1.0) - clean fade-out range.
  // Mode "drone":  (0.7, 1.75) - always-on baseline + deep self-osc territory.
  float feedback_min;
  float feedback_max;

  // Per-channel scaling on feedback writes to counter the mixing matrix's
  // peak gain. 0.7 ≈ 1/sqrt(2). 1.0 = no compensation (saturator continuous).
  float matrix_comp;

  // When true, Motion only drives LFO depth; the feedback-loop LPF is driven
  // by the separate `tone` parameter instead. Required for drone mode so
  // self-oscillation tones aren't damped out by low-Motion settings.
  bool decouple_tilt;

  // Mixing matrix interpolation alpha. If ≥ 0, fixed at that value. If < 0,
  // alpha tracks Size knob. Drone mode locks at 1.0 (orthogonal Hadamard)
  // because intermediate alpha is rank-deficient and destroys energy.
  float matrix_alpha;

  // Skip the red clip-warning LED when true. Drone mode runs continuously
  // near saturation by design.
  bool show_clip_warning;

  // Amplitude of the internal drone-bootstrap oscillator (mixed into the FDN
  // input). 0 = no oscillator (mode 0). > 0 = always-on excitation. The
  // oscillator frequency is derived from parameters_.pre_delay.
  float osc_amplitude;

  // Amplitude of filtered noise injection (also into the FDN input).
  // Excites all FDN modes evenly -> fuller spectrum, less "single-tone"
  // character. 0 = no noise.
  float noise_amplitude;

  // When true, in addition to driving FDN delay LFO depth, Motion also
  // makes the per-branch tilt LPF cutoff wobble at the Speed-LFO rate.
  // Gives Motion a "whole arrangement moves" feel - pitch shifts on the
  // delay reads AND tone shifts on the loop filter, in step.
  bool motion_wobbles_tilt;
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_PARAMETERS_H_
