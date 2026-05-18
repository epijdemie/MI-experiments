#include "warps_reverb/dsp/reverb.h"

#include <cmath>
#include <cstring>

namespace warps_reverb {

constexpr size_t Reverb::kBufferSize;
constexpr size_t Reverb::kTankSize0;
constexpr size_t Reverb::kTankSize1;
constexpr size_t Reverb::kTankSize2;
constexpr size_t Reverb::kTankSize3;
constexpr size_t Reverb::kEchoSize;

// co-prime base delays @ 48k. ~44/59/75/92ms (hall spread)
namespace {
constexpr float kBaseDelay0 = 2089.0f;
constexpr float kBaseDelay1 = 2843.0f;
constexpr float kBaseDelay2 = 3617.0f;
constexpr float kBaseDelay3 = 4421.0f;

constexpr float kPreDelayMaxSamples = 2400.0f;  // 50 ms @ 48k

// max tank delay-line modulation swing - very small static offset to keep
// the lines from being identical-length combs at extreme size settings.
// budget 40 samples → reservation needs ≥ 40 extra per line
constexpr float kStaticOffset[4] = { 0.0f, 13.0f, 27.0f, 39.0f };

// chord harmonics: 4 pitch shifters at major 3rd / perfect 5th / major 7th /
// major 9th. for pitch UP by ratio R, the read position must advance FASTER
// than write, so delay shrinks → phase decreases by (R - 1) per sample
constexpr float kChordRates[4] = {
  -0.25992f,   // 3rd  (5/4)
  -0.49831f,   // 5th  (3/2)
  -0.88775f,   // 7th  (15/8)
  -1.24492f,   // 9th  (9/4)
};

// chord shifter input scale. 4 shifters × Hann pair (sums to 1) = unit RMS
// per shifter; sum of 4 is ~2× RMS. scale down so wet_in isn't blown out
constexpr float kChordMix = 0.25f;

// echo delay range. 30 ms .. 200 ms (full buffer)
constexpr float kEchoTimeMin = 1440.0f;     // 30 ms @ 48k
constexpr float kEchoTimeMax = 9500.0f;     // ~198 ms (just under buffer)

// matrix gain compensation. 1.0 lets the loop reach unity gain at decay=1.0
// (cathedral / freeze territory). smoothsat bounds the loop — it asymptotes
// to ±1 so overshoot is absorbed musically. lower values cap rt60 short
constexpr float kMatrixComp = 1.0f;

// air-absorption lp coefficient. ~5 kHz at 48k. always-on hf damping
constexpr float kAbsorbCoef = 0.50f;

// fxengine output ap gain (fixed). dattorro standard
constexpr float kOutAp = 0.5f;

// max tape-echo feedback. capped < 1.0 so even at full knob it eventually
// decays. 0.95 = ~20 audible repeats before -40 dB
constexpr float kMaxEchoFeedback = 0.95f;

// smooth sat: y = x / ⁴√(1 + x⁴). bounded ±1, smooth knee
inline float SmoothSat(float x) {
  const float x2 = x * x;
  const float x4 = x2 * x2;
  return x / stmlib::Sqrt(stmlib::Sqrt(1.0f + x4));
}

// tank read with linear interpolation. delay in [0, size-1].
// w = write position (just-written sample is at w-1)
inline float ReadTankLinear(const float* buf, int size, int w, float delay) {
  const int delay_i = static_cast<int>(delay);
  const float delay_f = delay - static_cast<float>(delay_i);
  int idx0 = w - delay_i - 1;
  idx0 %= size;
  if (idx0 < 0) idx0 += size;
  int idx1 = idx0 - 1;
  if (idx1 < 0) idx1 += size;
  return buf[idx0] + (buf[idx1] - buf[idx0]) * delay_f;
}
}  // namespace

void Reverb::Init(float* buffer, float sample_rate) {
  sample_rate_ = sample_rate;
  peak_ = 0.0f;
  peak_block_ = 0.0f;

  engine_.Init(buffer);
  std::memset(lpf_state_, 0, sizeof(lpf_state_));
  std::memset(hpf_state_, 0, sizeof(hpf_state_));

  std::memset(tank_0_, 0, sizeof(tank_0_));
  std::memset(tank_1_, 0, sizeof(tank_1_));
  std::memset(tank_2_, 0, sizeof(tank_2_));
  std::memset(tank_3_, 0, sizeof(tank_3_));
  for (int i = 0; i < 4; ++i) tank_w_[i] = 0;

  std::memset(echo_buffer_, 0, sizeof(echo_buffer_));
  echo_w_ = 0;

  // stagger starting phases - keeps grains from re-aligning
  for (int i = 0; i < 4; ++i) {
    chord_phase_[i] = static_cast<float>(i) * 0.25f * kGrainSize;
  }

  parameters_.decay         = 0.5f;
  parameters_.echo_feedback = 0.0f;
  parameters_.size          = 0.5f;
  parameters_.dry_wet       = 0.5f;
  parameters_.pre_delay     = 0.0f;
  parameters_.echo_time     = 0.5f;
  parameters_.spread        = 0.1f;
  parameters_.low_cut       = 0.0f;

  Tick();
}

void Reverb::Tick() {
  coef_input_gain_         = 0.5f;
  coef_pre_delay_samples_  = parameters_.pre_delay * kPreDelayMaxSamples;
  // diffusion is no longer a knob - fixed at 0.85 (full-ish dattorro).
  // strong in-loop ap diffusion is required to kill modal ringing
  constexpr float kDiffusionFixed = 0.85f;
  coef_input_diff_a_       = 0.75f  * kDiffusionFixed;
  coef_input_diff_b_       = 0.625f * kDiffusionFixed;
  coef_branch_diff_        = 0.7f   * kDiffusionFixed;
  // decay knob: x*(2-x) curve so middle position already gives long tail.
  // 0.0→0, 0.5→0.75, 0.75→0.94, 1.0→1.0. saturator catches overshoot at top
  {
    const float d = parameters_.decay;
    coef_feedback_ = d * (2.0f - d);
  }
  coef_chord_gain_         = parameters_.spread * kChordMix;
  coef_dry_wet_            = parameters_.dry_wet;

  // echo time: linear knob → 30..200 ms in samples
  coef_echo_delay_samples_ = kEchoTimeMin +
      parameters_.echo_time * (kEchoTimeMax - kEchoTimeMin);

  // low_cut: 1-pole hp in each branch. 5..200 Hz log corner.
  // always slightly active so the loop can't accumulate DC
  const float hp_hz = 5.0f * exp2f(parameters_.low_cut * 5.32f);
  coef_low_cut_hp_  = 2.0f * static_cast<float>(M_PI) * hp_hz / sample_rate_;

  // tape echo feedback. 0 = single pass (no echo), 0.95 = long sustaining loop
  coef_echo_feedback_ = parameters_.echo_feedback * kMaxEchoFeedback;

  // lfo rates fixed at construction (kSpreadRateHz). only depth is knob-driven

  // matrix locked at full hadamard. partial-α leaves substantial self-coupling
  // on each line which lets it ring at its own modes (karplus-like).
  // size knob purely scales delay lengths now
  matrix_.set_alpha(1.0f);
}

void Reverb::Process(FloatFrame* in_out, size_t size) {
  // fxengine reservations (samples @ 48k):
  //   pre_delay         2400           50 ms front-of-chain delay
  //   input_ap0..3      230,173,613,448  dattorro input diffuser (~28 ms)
  //   branch_ap0..3     4 × 256        in-loop diffusion
  //   out_ap_l_a/b      230, 560       output diffuser L
  //   out_ap_r_a/b      320, 480       output diffuser R
  // total: 2400 + 1464 + 1024 + 1590 = 6478 of 8192
  typedef Engine::Reserve<2400,
          Engine::Reserve<230,
          Engine::Reserve<173,
          Engine::Reserve<613,
          Engine::Reserve<448,
          Engine::Reserve<256,
          Engine::Reserve<256,
          Engine::Reserve<256,
          Engine::Reserve<256,
          Engine::Reserve<230,
          Engine::Reserve<560,
          Engine::Reserve<320,
          Engine::Reserve<480>
          > > > > > > > > > > > > Memory;
  Engine::DelayLine<Memory,  0> pre_delay;
  Engine::DelayLine<Memory,  1> input_ap0;
  Engine::DelayLine<Memory,  2> input_ap1;
  Engine::DelayLine<Memory,  3> input_ap2;
  Engine::DelayLine<Memory,  4> input_ap3;
  Engine::DelayLine<Memory,  5> ap0;
  Engine::DelayLine<Memory,  6> ap1;
  Engine::DelayLine<Memory,  7> ap2;
  Engine::DelayLine<Memory,  8> ap3;
  Engine::DelayLine<Memory,  9> out_ap_l_a;
  Engine::DelayLine<Memory, 10> out_ap_l_b;
  Engine::DelayLine<Memory, 11> out_ap_r_a;
  Engine::DelayLine<Memory, 12> out_ap_r_b;
  Engine::Context c;

  const float gain      = coef_input_gain_;
  const float pre       = coef_pre_delay_samples_;
  const float kda       = coef_input_diff_a_;
  const float kdb       = coef_input_diff_b_;
  const float kap       = coef_branch_diff_;
  const float kfb       = coef_feedback_;
  const float khp       = coef_low_cut_hp_;
  const float kecho     = coef_echo_feedback_;
  const float echo_dly  = coef_echo_delay_samples_;
  const float chord_g   = coef_chord_gain_;
  const float wet       = coef_dry_wet_;

  // tape-echo state pulled into locals
  const int echo_n      = static_cast<int>(kEchoSize);
  int       echo_w      = echo_w_;

  // chord-shifter phases
  float cp0 = chord_phase_[0], cp1 = chord_phase_[1];
  float cp2 = chord_phase_[2], cp3 = chord_phase_[3];
  constexpr float kGrain = static_cast<float>(kGrainSize);

  const float size_k = 0.4f + 1.0f * parameters_.size;
  const float tau0 = kBaseDelay0 * size_k;
  const float tau1 = kBaseDelay1 * size_k;
  const float tau2 = kBaseDelay2 * size_k;
  const float tau3 = kBaseDelay3 * size_k;

  float lp0 = lpf_state_[0], lp1 = lpf_state_[1];
  float lp2 = lpf_state_[2], lp3 = lpf_state_[3];
  float hp0 = hpf_state_[0], hp1 = hpf_state_[1];
  float hp2 = hpf_state_[2], hp3 = hpf_state_[3];

  float* const echo = echo_buffer_;

  // tank pointers + sizes for the loop
  float* const t0 = tank_0_;  const int n0 = static_cast<int>(kTankSize0);
  float* const t1 = tank_1_;  const int n1 = static_cast<int>(kTankSize1);
  float* const t2 = tank_2_;  const int n2 = static_cast<int>(kTankSize2);
  float* const t3 = tank_3_;  const int n3 = static_cast<int>(kTankSize3);
  int w0 = tank_w_[0], w1 = tank_w_[1], w2 = tank_w_[2], w3 = tank_w_[3];

  while (size--) {
    engine_.Start(&c);

    // ---- pre-delay ----
    c.Read(in_out->l + in_out->r, gain);
    c.Write(pre_delay, 0.0f);
    c.Interpolate(pre_delay, pre, 1.0f);

    // ---- tape echo (before input diffuser) ----
    // read echo buffer at variable delay (echo_time knob), mix with pre-delayed
    // input, write back. each repeat goes through input diffuser + reverb
    // → wash patterns evolve over time.
    float pre_input;
    c.Write(pre_input, 0.0f);
    // variable echo delay - linear interp read
    const int   ed_i = static_cast<int>(echo_dly);
    const float ed_f = echo_dly - static_cast<float>(ed_i);
    int er0 = echo_w - ed_i - 1;
    er0 %= echo_n;
    if (er0 < 0) er0 += echo_n;
    int er1 = er0 - 1;
    if (er1 < 0) er1 += echo_n;
    const float echo_tail = echo[er0] + (echo[er1] - echo[er0]) * ed_f;
    // saturate the write back into the echo buffer — without this the loop
    // can run away at high feedback (no bound) and feed Inf into the reverb
    const float echo_mix  = SmoothSat(pre_input + kecho * echo_tail);
    echo[echo_w] = echo_mix;
    if (++echo_w >= echo_n) echo_w = 0;

    // ---- chord harmonics (4 pitch shifters reading from echo buffer) ----
    // each shifter: two grain heads offset by kGrain/2, hann-windowed.
    // grain reads are at delay = phase samples back from echo_w
    float chord_out = 0.0f;
    if (chord_g > 0.0f) {
      // process all 4 shifters
      float* phases[4] = { &cp0, &cp1, &cp2, &cp3 };
      for (int i = 0; i < 4; ++i) {
        const float p1 = *phases[i];
        float p2 = p1 + kGrain * 0.5f;
        if (p2 >= kGrain) p2 -= kGrain;

        // linear-interp reads at delay = p1 / p2 samples back
        auto grain_read = [&](float p) -> float {
          const int p_i = static_cast<int>(p);
          const float p_f = p - static_cast<float>(p_i);
          int gr0 = echo_w - p_i - 1;
          gr0 %= echo_n;
          if (gr0 < 0) gr0 += echo_n;
          int gr1 = gr0 - 1;
          if (gr1 < 0) gr1 += echo_n;
          return echo[gr0] + (echo[gr1] - echo[gr0]) * p_f;
        };

        const float g1 = grain_read(p1);
        const float g2 = grain_read(p2);

        // hann window: sin²(π * p / kGrain). pair sums to 1
        const float a1 = static_cast<float>(M_PI) * p1 / kGrain;
        const float a2 = static_cast<float>(M_PI) * p2 / kGrain;
        const float s1 = sinf(a1); const float w1 = s1 * s1;
        const float s2 = sinf(a2); const float w2 = s2 * s2;

        chord_out += g1 * w1 + g2 * w2;

        // advance phase by -(ratio - 1) per sample → delay shrinks → pitch up.
        // wrap when phase goes below 0
        *phases[i] += kChordRates[i];
        while (*phases[i] < 0.0f)    *phases[i] += kGrain;
        while (*phases[i] >= kGrain) *phases[i] -= kGrain;
      }
    }

    // saturate chord contribution too - shifters can sum loud in rare cases
    c.Load(echo_mix + SmoothSat(chord_g * chord_out));

    // ---- input diffuser ----
    c.Read(input_ap0 TAIL, -kda);
    c.WriteAllPass(input_ap0, kda);
    c.Read(input_ap1 TAIL, kda);
    c.WriteAllPass(input_ap1, -kda);
    c.Read(input_ap2 TAIL, -kdb);
    c.WriteAllPass(input_ap2, kdb);
    c.Read(input_ap3 TAIL, kdb);
    c.WriteAllPass(input_ap3, -kdb);

    float wet_in;
    c.Write(wet_in, 0.0f);

    // ---- read tank lines (modulated, linear interp) ----
    // tank lines run at fixed delays. ap-in-loop + full hadamard handle
    // modal diffusion; no lfo modulation needed (and the user hated the
    // glide character)
    const float r0 = ReadTankLinear(t0, n0, w0, tau0);
    const float r1 = ReadTankLinear(t1, n1, w1, tau1);
    const float r2 = ReadTankLinear(t2, n2, w2, tau2);
    const float r3 = ReadTankLinear(t3, n3, w3, tau3);

    // ---- per-branch chain: in-loop ap on (wet_in + tank read),
    // then air-absorb lp, then low-cut hp ----
    float b0, b1, b2, b3;

    c.Load(wet_in + r0);
    c.Read(ap0 TAIL, -kap);
    c.WriteAllPass(ap0, kap);
    c.Lp(lp0, kAbsorbCoef);
    c.Hp(hp0, khp);
    c.Write(b0, 0.0f);

    c.Load(wet_in + r1);
    c.Read(ap1 TAIL, kap);
    c.WriteAllPass(ap1, -kap);
    c.Lp(lp1, kAbsorbCoef);
    c.Hp(hp1, khp);
    c.Write(b1, 0.0f);

    c.Load(wet_in + r2);
    c.Read(ap2 TAIL, -kap);
    c.WriteAllPass(ap2, kap);
    c.Lp(lp2, kAbsorbCoef);
    c.Hp(hp2, khp);
    c.Write(b2, 0.0f);

    c.Load(wet_in + r3);
    c.Read(ap3 TAIL, kap);
    c.WriteAllPass(ap3, -kap);
    c.Lp(lp3, kAbsorbCoef);
    c.Hp(hp3, khp);
    c.Write(b3, 0.0f);

    // ---- matrix mix + smoothsat -> write back to tank ----
    float branch_in[4] = { b0, b1, b2, b3 };
    float mixed[4];
    matrix_.Apply(branch_in, mixed);

    t0[w0] = SmoothSat(mixed[0] * kfb * kMatrixComp);
    t1[w1] = SmoothSat(mixed[1] * kfb * kMatrixComp);
    t2[w2] = SmoothSat(mixed[2] * kfb * kMatrixComp);
    t3[w3] = SmoothSat(mixed[3] * kfb * kMatrixComp);
    if (++w0 >= n0) w0 = 0;
    if (++w1 >= n1) w1 = 0;
    if (++w2 >= n2) w2 = 0;
    if (++w3 >= n3) w3 = 0;

    // ---- stereo tap + output diffuser ----
    const float tap_l = b0 + b2;
    const float tap_r = b1 + b3;

    float wet_l, wet_r;

    c.Load(tap_l);
    c.Read(out_ap_l_a TAIL, -kOutAp);
    c.WriteAllPass(out_ap_l_a, kOutAp);
    c.Read(out_ap_l_b TAIL, kOutAp);
    c.WriteAllPass(out_ap_l_b, -kOutAp);
    c.Write(wet_l, 0.0f);

    c.Load(tap_r);
    c.Read(out_ap_r_a TAIL, -kOutAp);
    c.WriteAllPass(out_ap_r_a, kOutAp);
    c.Read(out_ap_r_b TAIL, kOutAp);
    c.WriteAllPass(out_ap_r_b, -kOutAp);
    c.Write(wet_r, 0.0f);

    const float magl = wet_l < 0 ? -wet_l : wet_l;
    const float magr = wet_r < 0 ? -wet_r : wet_r;
    const float magm = magl > magr ? magl : magr;
    if (magm > peak_block_) peak_block_ = magm;

    in_out->l = stmlib::Crossfade(in_out->l, wet_l, wet);
    in_out->r = stmlib::Crossfade(in_out->r, wet_r, wet);
    ++in_out;
  }

  lpf_state_[0] = lp0; lpf_state_[1] = lp1;
  lpf_state_[2] = lp2; lpf_state_[3] = lp3;
  hpf_state_[0] = hp0; hpf_state_[1] = hp1;
  hpf_state_[2] = hp2; hpf_state_[3] = hp3;
  tank_w_[0] = w0; tank_w_[1] = w1;
  tank_w_[2] = w2; tank_w_[3] = w3;
  chord_phase_[0] = cp0; chord_phase_[1] = cp1;
  chord_phase_[2] = cp2; chord_phase_[3] = cp3;
  echo_w_ = echo_w;

  // peak: instant attack, ~150ms release
  peak_ = peak_block_ > peak_ ? peak_block_ : peak_ * 0.9f;
  peak_block_ = 0.0f;
}

}  // namespace warps_reverb
