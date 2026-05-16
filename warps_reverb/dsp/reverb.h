// Erbe-Verb-style 4-line FDN reverb built on Clouds' FxEngine.
//
// Topology (Erbe ICMC 2015, "Building the Erbe-Verb"):
//
//   in ─► pre-delay ──► splitter ─►┐
//                                  ├── branch 0: AP(g) ► delay(τ_0, mod) ► LPF ┐
//                                  ├── branch 1: AP(g) ► delay(τ_1, mod) ► LPF │
//                                  ├── branch 2: AP(g) ► delay(τ_2, mod) ► LPF ├─► M(α)
//                                  └── branch 3: AP(g) ► delay(τ_3, mod) ► LPF ┘
//                                                                              │
//                                       saturate(feedback_gain · M(α)·branch) ◄┘
//
// M(α) interpolates between identity (α=0, four independent comb filters) and
// the orthogonal 4×4 Hadamard matrix (α=1, full cross-coupling FDN). The
// saturator on the feedback path allows feedback_gain up to 1.25 with bounded
// loop energy - Erbe's "overshoot" regime.
//
// FxEngine handles: the contiguous delay buffer, Schroeder allpass via
// WriteAllPass, one-pole LPFs via Lp, and LFO-modulated reads via Interpolate.
// The interpolated mixing matrix and the saturator live outside FxEngine.
//
// Target sound: clean (Erbe-Verb / Strymon Star Lab territory), NOT smeary
// (Clouds / Symbiote territory). The clean character comes from:
//   - FORMAT_16_BIT storage (12 dB cleaner per tap than Clouds' FORMAT_12_BIT)
//   - Diffusion decoupled from Size so it can sweep to ZERO (discrete echoes)
//   - LPF cutoff defaulted high (open, not warm)
//   - No pre-processing - codec -> reverb -> codec, nothing in between
// A FORMAT_32_BIT upgrade is a one-line change once we hear how 16-bit lands.

#ifndef WARPS_REVERB_DSP_REVERB_H_
#define WARPS_REVERB_DSP_REVERB_H_

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"

#include "clouds/dsp/fx/fx_engine.h"

#include "warps_reverb/dsp/mixing_matrix.h"
#include "warps_reverb/dsp/parameters.h"

namespace warps_reverb {

struct ShortFrame {
  int16_t l;
  int16_t r;
};

struct FloatFrame {
  float l;
  float r;
};

class Reverb {
 public:
  Reverb() { }
  ~Reverb() { }

  // buffer must point to FxEngine::kBufferSize uint16_t samples in main SRAM.
  void Init(uint16_t* buffer, float sample_rate);

  // Apply mode-specific DSP coefficient overrides. Called from Ui::EnterMode.
  void set_dsp_overrides(const DspOverrides& d) {
    feedback_min_         = d.feedback_min;
    feedback_max_         = d.feedback_max;
    matrix_comp_          = d.matrix_comp;
    decouple_tilt_        = d.decouple_tilt;
    matrix_alpha_         = d.matrix_alpha;
    osc_amplitude_        = d.osc_amplitude;
    noise_max_            = d.noise_amplitude;
    motion_wobbles_tilt_  = d.motion_wobbles_tilt;
  }

  // Block-rate parameter update - call once before each Process() call.
  void Tick();

  // Per-block audio processing. Reads in_out, writes back wet/dry mix.
  void Process(FloatFrame* in_out, size_t size);

  ReverbParameters* mutable_parameters() { return &parameters_; }

  // Decayed peak of the wet output, in [0, ~1+ε]. Anything > 0.95 means the
  // final SoftLimit at the codec is actively saturating - i.e., clipping.
  // Used by the UI to flash the main LED red.
  inline float peak() const { return peak_; }

  // Size of the FxEngine buffer, in samples (uint16_t each = 2 bytes).
  // 32768 = 64 KB. Sum of all reserved delay lines must be < this.
  static constexpr size_t kBufferSize = 32768;

 private:
  typedef clouds::FxEngine<kBufferSize, clouds::FORMAT_16_BIT> Engine;
  Engine engine_;

  MixingMatrix matrix_;

  // Per-branch state held between samples.
  float lpf_state_[4];

  // Cached coefficients written by Tick(), consumed by Process().
  float coef_input_gain_;
  float coef_pre_delay_samples_;
  float coef_diffusion_;
  float coef_feedback_;
  float coef_tilt_lpf_;
  float coef_dry_wet_;
  float coef_mod_amplitude_;

  float sample_rate_;
  float peak_;          // decayed peak of wet output
  float peak_block_;    // running peak within the current block
  float feedback_min_;  // from current mode
  float feedback_max_;  // from current mode
  float matrix_comp_;   // from current mode
  bool  decouple_tilt_; // from current mode
  float matrix_alpha_;  // from current mode (-1 -> use size; ≥0 -> fixed)
  float osc_amplitude_; // from current mode (0 = no oscillator)

  // Drone voice bank - 4 unison oscillators with independent slow drift
  // for the Tim Hecker-style "wandering pitch" character. All four
  // voices sit at the same base pitch (parameters_.pre_delay); each
  // gets its own drift LFO at incommensurate rates so the detune pattern
  // never repeats. parameters_.motion controls drift depth.
  float osc_phase_[4];
  float osc_phase_inc_;          // baseline (un-drifted) phase increment

  // Drift LFOs - one per voice, decorrelated rates.
  float drift_phase_[4];
  float drift_phase_inc_[4];

  // Noise injection (Strega-style). Filtered white noise mixed into the FDN
  // input alongside the oscillator. noise_max_ is the mode's ceiling;
  // parameters_.noise scales 0..1 within that.
  float noise_max_;
  float noise_lp_state_;

  // Per-branch state-variable filter (drone mode only). Provides LP/BP/HP
  // outputs simultaneously; PARAM_FILTER morphs between them.
  float svf_lp_[4];
  float svf_bp_[4];
  float svf_f_base_;     // SVF frequency coefficient (~ 2 sin(π fc / SR))
  float svf_q_;          // damping coefficient
  float filter_morph_;   // 0 = LP, 1 = HP - from parameters_.filter

  // Pre-saturator drive in the feedback path. drive_factor_ = 1 + drive*4
  // so the parameter sweep covers 1× (clean) to 5× (heavy harmonic content).
  float drive_factor_;

  // Motion LFO - sample-rate sine that wobbles the per-branch tilt LPF
  // cutoff (in drone mode). Runs at the Speed parameter's rate; depth
  // comes from Motion. Independent of the FxEngine's per-block LFOs.
  float motion_lfo_phase_;
  float motion_lfo_phase_inc_;
  float motion_lfo_depth_;        // 0..1, scaled from parameters_.motion
  bool  motion_wobbles_tilt_;     // from current mode
  ReverbParameters parameters_;

  DISALLOW_COPY_AND_ASSIGN(Reverb);
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_REVERB_H_
