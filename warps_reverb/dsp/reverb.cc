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

// max chord-spread depth in samples per tank line. ±200 ≈ ±4 ms at 48k.
// reservation headroom is 240 so this fits with margin
constexpr float kMaxSpreadDepth = 200.0f;

// 4 incommensurate slow lfo rates (Hz). periods ~5-14 s, mutually prime-ish
// so lines never align - keeps the spread evolving without re-syncing
constexpr float kSpreadRateHz[4] = { 0.073f, 0.097f, 0.131f, 0.181f };

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

  // stagger starting phases so lines don't all begin from zero
  for (int i = 0; i < 4; ++i) {
    lfo_phase_[i] = static_cast<float>(i) * 1.5707963f;
    lfo_phase_inc_[i] =
        2.0f * static_cast<float>(M_PI) * kSpreadRateHz[i] / sample_rate;
  }

  parameters_.decay         = 0.5f;
  parameters_.echo_feedback = 0.0f;
  parameters_.size          = 0.5f;
  parameters_.dry_wet       = 0.5f;
  parameters_.pre_delay     = 0.0f;
  parameters_.diffusion     = 0.5f;
  parameters_.spread        = 0.1f;
  parameters_.low_cut       = 0.0f;

  Tick();
}

void Reverb::Tick() {
  coef_input_gain_         = 0.5f;
  coef_pre_delay_samples_  = parameters_.pre_delay * kPreDelayMaxSamples;
  coef_input_diff_a_       = 0.75f  * parameters_.diffusion;
  coef_input_diff_b_       = 0.625f * parameters_.diffusion;
  // in-loop ap is the ONLY diffuser inside the feedback loop - has to break
  // up each tank line's modal resonances. higher gain (Schroeder-classic 0.7)
  coef_branch_diff_        = 0.7f * parameters_.diffusion;
  // decay knob: x*(2-x) curve so middle position already gives long tail.
  // 0.0→0, 0.5→0.75, 0.75→0.94, 1.0→1.0. saturator catches overshoot at top
  {
    const float d = parameters_.decay;
    coef_feedback_ = d * (2.0f - d);
  }
  coef_spread_depth_       = parameters_.spread * kMaxSpreadDepth;
  coef_dry_wet_            = parameters_.dry_wet;

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
  const float wet       = coef_dry_wet_;
  const float ampl      = coef_spread_depth_;

  // tape-echo state pulled into locals
  const int echo_n      = static_cast<int>(kEchoSize);
  int       echo_w      = echo_w_;

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

  float lfo_ph0 = lfo_phase_[0], lfo_ph1 = lfo_phase_[1];
  float lfo_ph2 = lfo_phase_[2], lfo_ph3 = lfo_phase_[3];
  const float lfo_inc0 = lfo_phase_inc_[0];
  const float lfo_inc1 = lfo_phase_inc_[1];
  const float lfo_inc2 = lfo_phase_inc_[2];
  const float lfo_inc3 = lfo_phase_inc_[3];
  constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);

  while (size--) {
    // advance 4 slow lfos. 4 × sinf at 48k = 192k/s ≈ 6% CPU — affordable
    lfo_ph0 += lfo_inc0; if (lfo_ph0 > kTwoPi) lfo_ph0 -= kTwoPi;
    lfo_ph1 += lfo_inc1; if (lfo_ph1 > kTwoPi) lfo_ph1 -= kTwoPi;
    lfo_ph2 += lfo_inc2; if (lfo_ph2 > kTwoPi) lfo_ph2 -= kTwoPi;
    lfo_ph3 += lfo_inc3; if (lfo_ph3 > kTwoPi) lfo_ph3 -= kTwoPi;
    const float lfo_v0 = sinf(lfo_ph0);
    const float lfo_v1 = sinf(lfo_ph1);
    const float lfo_v2 = sinf(lfo_ph2);
    const float lfo_v3 = sinf(lfo_ph3);

    engine_.Start(&c);

    // ---- pre-delay ----
    c.Read(in_out->l + in_out->r, gain);
    c.Write(pre_delay, 0.0f);
    c.Interpolate(pre_delay, pre, 1.0f);

    // ---- tape echo (before input diffuser) ----
    // read tail of echo buffer at full delay, mix with pre-delayed input,
    // write back. each repeat then goes through the input diffuser + reverb
    // → wash patterns evolve over time.
    float pre_input;
    c.Write(pre_input, 0.0f);
    int echo_r = echo_w + 1;
    if (echo_r >= echo_n) echo_r -= echo_n;
    const float echo_tail = echo[echo_r];
    const float echo_mix  = pre_input + kecho * echo_tail;
    echo[echo_w] = echo_mix;
    if (++echo_w >= echo_n) echo_w = 0;
    c.Load(echo_mix);

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
    const float r0 = ReadTankLinear(t0, n0, w0, tau0 + ampl * lfo_v0);
    const float r1 = ReadTankLinear(t1, n1, w1, tau1 + ampl * lfo_v1);
    const float r2 = ReadTankLinear(t2, n2, w2, tau2 + ampl * lfo_v2);
    const float r3 = ReadTankLinear(t3, n3, w3, tau3 + ampl * lfo_v3);

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
  lfo_phase_[0] = lfo_ph0;
  lfo_phase_[1] = lfo_ph1;
  lfo_phase_[2] = lfo_ph2;
  lfo_phase_[3] = lfo_ph3;
  echo_w_ = echo_w;

  // peak: instant attack, ~150ms release
  peak_ = peak_block_ > peak_ ? peak_block_ : peak_ * 0.9f;
  peak_block_ = 0.0f;
}

}  // namespace warps_reverb
