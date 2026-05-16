#include "warps_reverb/dsp/reverb.h"

#include <cmath>
#include <cstring>

#include "stmlib/utils/random.h"

namespace warps_reverb {

using clouds::LFO_1;
using clouds::LFO_2;

constexpr size_t Reverb::kBufferSize;

// Co-prime base delays at 48 kHz (samples). ~44/59/75/92 ms - hall-class
// spread that spaces the comb-filter modes wider than the original Erbe
// ballpark, avoiding the Karplus-Strong/comb character of shorter lines.
// All four values are primes so they're trivially co-prime.
namespace {
constexpr float kBaseDelay0 = 2089.0f;
constexpr float kBaseDelay1 = 2843.0f;
constexpr float kBaseDelay2 = 3617.0f;
constexpr float kBaseDelay3 = 4421.0f;

constexpr float kPreDelayMaxSamples = 4800.0f;  // 100 ms @ 48 kHz

// Smooth saturator: y = x / √√(1 + x⁴). Strictly bounded ±1, smooth knee,
// near-linear in the |x| < 0.5 range so moderate signals pass cleanly while
// hot peaks compress gradually. Much less "harsh" than the previous tanh-
// style hard clip - no kink at the threshold, no abrupt hard clip.
inline float SmoothSat(float x) {
  const float x2 = x * x;
  const float x4 = x2 * x2;
  return x / stmlib::Sqrt(stmlib::Sqrt(1.0f + x4));
}

// Bipolar white noise in [-1, 1] from stmlib's PRNG.
inline float WhiteNoise() {
  const uint32_t w = stmlib::Random::GetWord();
  return static_cast<int32_t>(w) * (1.0f / 2147483648.0f);
}
}  // namespace

void Reverb::Init(uint16_t* buffer, float sample_rate) {
  sample_rate_ = sample_rate;
  peak_ = 0.0f;
  peak_block_ = 0.0f;
  feedback_min_ = 0.0f;
  feedback_max_ = 1.0f;
  matrix_comp_  = 0.7f;
  decouple_tilt_ = false;
  matrix_alpha_ = -1.0f;
  osc_amplitude_ = 0.0f;
  osc_phase_inc_ = 0.0f;
  for (int i = 0; i < 4; ++i) {
    osc_phase_[i] = static_cast<float>(i) * 1.57f;     // stagger initial phases
    drift_phase_[i] = static_cast<float>(i) * 0.78f;
    drift_phase_inc_[i] = 0.0f;
  }
  noise_max_ = 0.0f;
  noise_lp_state_ = 0.0f;
  for (int i = 0; i < 4; ++i) { svf_lp_[i] = 0.0f; svf_bp_[i] = 0.0f; }
  svf_f_base_ = 0.06f;    // ~460 Hz cutoff at 48 kHz - dark drone floor
  svf_q_ = 1.6f;          // heavier damping - kills any residual ringing
  filter_morph_ = 0.0f;
  drive_factor_ = 1.0f;
  motion_wobbles_tilt_ = false;
  motion_lfo_phase_ = 0.0f;
  motion_lfo_phase_inc_ = 0.0f;
  motion_lfo_depth_ = 0.0f;
  engine_.Init(buffer);
  std::memset(lpf_state_, 0, sizeof(lpf_state_));

  // Two LFOs at fixed multiplied frequencies - Erbe's multiphase scheme uses
  // four phases of one sine; FxEngine gives us two LFOs, which we'll mix via
  // sign-flip to synthesize four effective phases across the branches.
  // Frequencies set in Tick() from parameters_.speed.

  // Placeholder defaults - Ui::Init/EnterMode overwrites these from the
  // active mode's ModeConfig.defaults before the first audio block runs.
  parameters_.size      = 0.5f;
  parameters_.decay     = 0.5f;
  parameters_.diffusion = 0.0f;
  parameters_.motion    = 0.0f;
  parameters_.speed     = 0.0f;
  parameters_.pre_delay = 0.0f;
  parameters_.dry_wet   = 0.5f;
  parameters_.shimmer   = 0.0f;
  parameters_.freeze    = false;

  Tick();
}

void Reverb::Tick() {
  coef_input_gain_ = 0.5f;
  coef_pre_delay_samples_ = parameters_.pre_delay * kPreDelayMaxSamples;
  // Diffusion is independent of Size. Max stays at Erbe's 0.8 ceiling.
  coef_diffusion_ = 0.8f * parameters_.diffusion;
  // Decay knob mapping: mode-specific linear interpolation between
  // feedback_min_ (knob CCW) and feedback_max_ (knob CW). Mode 0 uses
  // (0, 1.0) - knob CCW fully kills the tail. Mode 1 uses (0.7, 1.75) -
  // knob CCW still gives a long tail, knob CW pushes deep into self-osc.
  coef_feedback_ = feedback_min_ +
                   parameters_.decay * (feedback_max_ - feedback_min_);

  // Tilt LPF baseline. Mode 0 = motion-coupled. Drone (decouple_tilt_) reads
  // from the dedicated Tone parameter instead. In drone mode the actual
  // per-sample LPF coefficient is modulated around this baseline by the
  // motion LFO (see Process).
  if (decouple_tilt_) {
    // Drone-mode tilt baseline: darker range. k=0.05 ≈ 390 Hz, k=0.4 ≈ 4 kHz.
    // The per-branch one-pole LPF is the main thing taming high-frequency
    // squeal in the long feedback loop, so its baseline lives in dark
    // territory by default and Tone opens it up.
    coef_tilt_lpf_ = 0.05f + 0.35f * parameters_.tone;
  } else {
    coef_tilt_lpf_ = 0.55f + 0.44f * parameters_.motion;
  }

  // FDN delay LFO depth - always driven by Motion. This is what gives the
  // pitch-territory shimmer to the reverb tail; combined with the tilt LPF
  // wobble in drone mode, the whole spectrum breathes in step.
  coef_mod_amplitude_ = parameters_.motion * 0.6f;

  // Drone voice bank base pitch + drift LFO rates. Logarithmic 60 -> 500 Hz
  // for the carrier; drift LFOs run at base_rate × (7,11,13,17) / 7 so they
  // never align (Hecker's "wandering" character). Base drift rate scaled by
  // parameters_.speed (very slow at default - Hecker drones drift over many
  // seconds).
  if (osc_amplitude_ > 0.0f) {
    const float osc_hz = 60.0f * exp2f(parameters_.pre_delay * 3.06f);
    osc_phase_inc_ = 2.0f * static_cast<float>(M_PI) * osc_hz / sample_rate_;

    // Drift rates: base 0.04 Hz (period ~25 s) × prime ratios. Speed knob
    // multiplies the base up to ~2 Hz at max.
    const float base_drift_hz = 0.04f + parameters_.speed * 1.96f;
    constexpr float kPrimes[4] = { 7.0f, 11.0f, 13.0f, 17.0f };
    for (int i = 0; i < 4; ++i) {
      const float hz = base_drift_hz * kPrimes[i] / 7.0f;
      drift_phase_inc_[i] =
          2.0f * static_cast<float>(M_PI) * hz / sample_rate_;
    }
  }

  // Motion LFO (sample-rate sine, used to modulate the SVF cutoff in drone
  // mode and the tilt LPF coefficient in mode 0). Speed maps to 0.1–50 Hz.
  if (motion_wobbles_tilt_) {
    const float mod_hz = 0.1f + parameters_.speed * 49.9f;
    motion_lfo_phase_inc_ =
        2.0f * static_cast<float>(M_PI) * mod_hz / sample_rate_;
    motion_lfo_depth_ = parameters_.motion;
  } else {
    motion_lfo_phase_inc_ = 0.0f;
    motion_lfo_depth_ = 0.0f;
  }

  // SVF morph parameter snapshot.
  filter_morph_ = parameters_.filter;

  // Drive: pre-saturator gain in feedback path. 1× = clean limiting, 5× =
  // heavy harmonic distortion. The smooth saturator clamps anyway so this
  // controls *character*, not loudness - adds harmonics rather than volume.
  drive_factor_ = 1.0f + parameters_.drive * 4.0f;

  coef_dry_wet_ = parameters_.dry_wet;

  // Two LFOs at f and f·1.41 - incommensurate ratio for richer modulation
  // when the four branches reference them with alternating signs.
  const float lfo_hz = 0.1f + parameters_.speed * 49.9f;  // 0.1..50 Hz
  engine_.SetLFOFrequency(LFO_1, lfo_hz / sample_rate_);
  engine_.SetLFOFrequency(LFO_2, lfo_hz * 1.41f / sample_rate_);

  // Matrix alpha: mode 0 tracks Size (mixing morph is part of the reverb
  // character). Mode 1 locks alpha at 1.0 because intermediate α produces
  // a rank-deficient mixing matrix that destroys energy in half the FDN's
  // state space - killing self-oscillation. With α=1.0 the matrix is
  // orthogonal and the drone can sustain.
  const float alpha = (matrix_alpha_ < 0.0f)
                          ? parameters_.size
                          : matrix_alpha_;
  matrix_.set_alpha(alpha);

  // Shimmer placeholder: until we build a proper granular pitch shifter, this
  // just adds extra feedback gain (push into self-resonance) and brightness.
  // Real implementation is a TODO. At least the parameter is wired up.
  if (parameters_.shimmer > 0.0f) {
    coef_feedback_ += parameters_.shimmer * 0.15f;   // up to +15% feedback
    coef_tilt_lpf_  = stmlib::Crossfade(
        coef_tilt_lpf_, 0.995f, parameters_.shimmer);  // opens HF further
  }

  // Freeze: lock feedback to unity and mute input. SoftLimit catches drift.
  if (parameters_.freeze) {
    coef_feedback_ = 1.0f;
    coef_input_gain_ = 0.0f;
  }
  // Reverse: TODO - Erbe's Reverse flips the delay-read direction so the
  // envelope plays back tail-first. With FxEngine's read primitives this
  // needs a custom Read-from-(write_ptr + length - offset) path; deferring
  // until basic operation is verified on hardware.
}

void Reverb::Process(FloatFrame* in_out, size_t size) {
  // Memory reservations for the FxEngine buffer. Sum must be < 32768.
  //
  //   pre_delay   : 4800    (100 ms @ 48 kHz)
  //   ap0..ap3    : 256 ×4 = 1024
  //   del0..del3  : 6200 ×4 = 24800 (sized for base × ~1.4× Size sweep)
  //   total       : 30624  ✓ (under 32768, with headroom)
  typedef Engine::Reserve<4800,                    // pre_delay
          Engine::Reserve<256,                     // ap0
          Engine::Reserve<256,                     // ap1
          Engine::Reserve<256,                     // ap2
          Engine::Reserve<256,                     // ap3
          Engine::Reserve<6200,                    // del0
          Engine::Reserve<6200,                    // del1
          Engine::Reserve<6200,                    // del2
          Engine::Reserve<6200> > > > > > > > > Memory;
  Engine::DelayLine<Memory, 0> pre_delay;
  Engine::DelayLine<Memory, 1> ap0;
  Engine::DelayLine<Memory, 2> ap1;
  Engine::DelayLine<Memory, 3> ap2;
  Engine::DelayLine<Memory, 4> ap3;
  Engine::DelayLine<Memory, 5> del0;
  Engine::DelayLine<Memory, 6> del1;
  Engine::DelayLine<Memory, 7> del2;
  Engine::DelayLine<Memory, 8> del3;
  Engine::Context c;

  // Snapshot coefficients for the inner loop.
  const float gain      = coef_input_gain_;
  const float pre       = coef_pre_delay_samples_;
  const float kap       = coef_diffusion_;
  const float kfb       = coef_feedback_;
  const float klp_base  = coef_tilt_lpf_;       // wobbled per-sample if drone
  const float wet       = coef_dry_wet_;
  const float ampl      = coef_mod_amplitude_;
  // Size sweep tighter than before (0.4..1.4x) so even at max we stay within
  // the per-delay reservation of 6200 samples (longest base 4421 × 1.4 ≈ 6190).
  const float size_k = 0.4f + 1.0f * parameters_.size;

  const float tau0 = kBaseDelay0 * size_k;
  const float tau1 = kBaseDelay1 * size_k;
  const float tau2 = kBaseDelay2 * size_k;
  const float tau3 = kBaseDelay3 * size_k;

  float lp0 = lpf_state_[0];
  float lp1 = lpf_state_[1];
  float lp2 = lpf_state_[2];
  float lp3 = lpf_state_[3];

  while (size--) {
    engine_.Start(&c);

    // ---- Pre-delay ----
    c.Read(in_out->l + in_out->r, gain);
    c.Write(pre_delay, 0.0f);
    c.Interpolate(pre_delay, pre, LFO_1, 0.0f, 1.0f);
    float wet_in;
    c.Write(wet_in, 0.0f);

    // ---- Voice bank injection ----
    // 4 sines at the same base pitch, each independently pitch-modulated
    // by its own slow drift LFO. parameters_.motion scales the drift range
    // up to ±~50 cents (factor 0.03). At motion=0, all four are perfectly
    // unison (= effectively a single louder voice). At motion=1, the four
    // wander apart and create chorus / beating / Hecker-style smear.
    if (osc_amplitude_ > 0.0f) {
      const float drift_range = parameters_.motion * 0.03f;
      float voice_sum = 0.0f;
      for (int i = 0; i < 4; ++i) {
        drift_phase_[i] += drift_phase_inc_[i];
        if (drift_phase_[i] > 2.0f * static_cast<float>(M_PI)) {
          drift_phase_[i] -= 2.0f * static_cast<float>(M_PI);
        }
        const float pitch_mul = 1.0f + sinf(drift_phase_[i]) * drift_range;
        osc_phase_[i] += osc_phase_inc_ * pitch_mul;
        if (osc_phase_[i] > 2.0f * static_cast<float>(M_PI)) {
          osc_phase_[i] -= 2.0f * static_cast<float>(M_PI);
        }
        voice_sum += sinf(osc_phase_[i]);
      }
      // 4 voices summed, normalize amplitude (×0.25). Inject scaled by mode
      // osc_amplitude - same overall level as a single voice would be.
      wet_in += voice_sum * (osc_amplitude_ * 0.25f);
    }

    // ---- Noise injection (Strega-style) ----
    // Filtered to brown-ish spectrum. noise_max_ is the mode's ceiling;
    // user's noise parameter scales within that range.
    if (noise_max_ > 0.0f && parameters_.noise > 0.0f) {
      const float white = WhiteNoise();
      noise_lp_state_ += 0.06f * (white - noise_lp_state_);
      wet_in += noise_lp_state_ * parameters_.noise * noise_max_;
    }

    // ---- Motion LFO advance ----
    // One LFO drives both the mode-0 tilt LPF wobble and the drone-mode
    // SVF cutoff wobble. Computed once per sample.
    float lfo_value = 0.0f;
    if (motion_lfo_depth_ > 0.0f) {
      motion_lfo_phase_ += motion_lfo_phase_inc_;
      if (motion_lfo_phase_ > 2.0f * static_cast<float>(M_PI)) {
        motion_lfo_phase_ -= 2.0f * static_cast<float>(M_PI);
      }
      lfo_value = sinf(motion_lfo_phase_);
    }

    // Mode-0 tilt-LPF coefficient (used by c.Lp branches when not in drone).
    float klp_sample = klp_base + lfo_value * motion_lfo_depth_ * 0.25f;
    if (klp_sample < 0.05f) klp_sample = 0.05f;
    if (klp_sample > 0.99f) klp_sample = 0.99f;

    // Drone-mode SVF coefficients. Tight modulation around a dark base so the
    // recirculating signal never gets bright enough to seed self-squeal.
    // Range ~0.02..0.10 -> ~150..760 Hz.
    float svf_f = svf_f_base_ + lfo_value * motion_lfo_depth_ * 0.04f;
    if (svf_f < 0.02f) svf_f = 0.02f;
    if (svf_f > 0.15f) svf_f = 0.15f;

    // ---- Per-branch processing ----
    // Branch i = pre + saturate(prior loop feedback)
    //         -> Schroeder allpass (diffuser, gain = kap)
    //         -> modulated delay read at tau_i ± depth · sin(lfo)
    //         -> one-pole LPF
    //
    // Feedback into each branch comes from the *previous block's* mixed
    // output, stored at the delay's tail. The FxEngine's circular buffer
    // gives us this for free: writing at the head and reading at base+length
    // walks the entire RT60.
    float b0, b1, b2, b3;

    // Branch 0. Per-branch one-pole LPF tames the output AND the feedback
    // loop. In drone mode the SVF runs additionally on the recirculating
    // signal (post-matrix) for the Filter morph; the per-branch LPF guards
    // against high-frequency squeal in the long-feedback regime.
    c.Load(wet_in);
    c.Read(ap0 TAIL, -kap);
    c.WriteAllPass(ap0, kap);
    c.Interpolate(del0, tau0, LFO_1, ampl * tau0, 1.0f);
    c.Lp(lp0, klp_sample);
    c.Write(b0, 0.0f);

    // Branch 1
    c.Load(wet_in);
    c.Read(ap1 TAIL, kap);
    c.WriteAllPass(ap1, -kap);
    c.Interpolate(del1, tau1, LFO_2, ampl * tau1, 1.0f);
    c.Lp(lp1, klp_sample);
    c.Write(b1, 0.0f);

    // Branch 2
    c.Load(wet_in);
    c.Read(ap2 TAIL, -kap);
    c.WriteAllPass(ap2, kap);
    c.Interpolate(del2, tau2, LFO_1, -ampl * tau2, 1.0f);
    c.Lp(lp2, klp_sample);
    c.Write(b2, 0.0f);

    // Branch 3
    c.Load(wet_in);
    c.Read(ap3 TAIL, kap);
    c.WriteAllPass(ap3, -kap);
    c.Interpolate(del3, tau3, LFO_2, -ampl * tau3, 1.0f);
    c.Lp(lp3, klp_sample);
    c.Write(b3, 0.0f);

    // ---- Mixing matrix M(α) ----
    float branch_in[4]  = { b0, b1, b2, b3 };
    float mixed[4];
    matrix_.Apply(branch_in, mixed);

    // ---- Drone-mode SVF on the recirculating signal ----
    // Filter sits BETWEEN matrix and feedback writes - affects only what
    // recirculates through the FDN. The direct output tap (b_i above)
    // stays unfiltered, so Filter morph shapes the long-term character of
    // the tail rather than directly colouring the audible mix.
    if (decouple_tilt_) {
      for (int i = 0; i < 4; ++i) {
        const float hp = mixed[i] - svf_lp_[i] - svf_q_ * svf_bp_[i];
        svf_bp_[i] += svf_f * hp;
        svf_lp_[i] += svf_f * svf_bp_[i];
        const float lp = svf_lp_[i];
        mixed[i] = lp * (1.0f - filter_morph_) + hp * filter_morph_;
      }
    }

    // ---- Feedback writes (with saturation) ----
    // matrix_comp_ comes from the active mode (see DspOverrides). 0.7
    // ≈ 1/sqrt(2) for clean reverb (saturator on transients only). 1.0 for
    // drone mode (no compensation -> continuous saturation, self-osc).
    const float kMatrixComp = matrix_comp_;
    const float drive = drive_factor_;
    // SmoothSat is bounded ±1, so drive controls harmonic content rather
    // than loudness. Higher drive pushes deeper into the nonlinear region
    // and adds saturation overtones to the feedback signal.
    c.Load(SmoothSat(mixed[0] * kfb * kMatrixComp * drive));
    c.Write(del0, 0.0f);
    c.Load(SmoothSat(mixed[1] * kfb * kMatrixComp * drive));
    c.Write(del1, 0.0f);
    c.Load(SmoothSat(mixed[2] * kfb * kMatrixComp * drive));
    c.Write(del2, 0.0f);
    c.Load(SmoothSat(mixed[3] * kfb * kMatrixComp * drive));
    c.Write(del3, 0.0f);

    // ---- Stereo output tap ----
    // Branches 0+2 -> L, 1+3 -> R. Crossfade against the dry input.
    const float wet_l = b0 + b2;
    const float wet_r = b1 + b3;
    // Peak-track the wet signal pre-crossfade. Anything > 1 will saturate
    // at the final codec output's SoftLimit -> clip indicator.
    const float magl = wet_l < 0 ? -wet_l : wet_l;
    const float magr = wet_r < 0 ? -wet_r : wet_r;
    const float magm = magl > magr ? magl : magr;
    if (magm > peak_block_) peak_block_ = magm;

    in_out->l = stmlib::Crossfade(in_out->l, wet_l, wet);
    in_out->r = stmlib::Crossfade(in_out->r, wet_r, wet);
    ++in_out;
  }

  lpf_state_[0] = lp0;
  lpf_state_[1] = lp1;
  lpf_state_[2] = lp2;
  lpf_state_[3] = lp3;

  // Decay the per-block peak into the persistent peak. ~5 ms attack via
  // peak_block_ rising directly; ~150 ms release via 0.9× per block.
  peak_ = peak_block_ > peak_ ? peak_block_ : peak_ * 0.9f;
  peak_block_ = 0.0f;
}

}  // namespace warps_reverb
