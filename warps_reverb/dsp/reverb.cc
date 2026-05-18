#include "warps_reverb/dsp/reverb.h"

#include <cmath>
#include <cstring>

namespace warps_reverb {

constexpr size_t Reverb::kBufferSize;
constexpr size_t Reverb::kTankSize0;
constexpr size_t Reverb::kTankSize1;
constexpr size_t Reverb::kTankSize2;
constexpr size_t Reverb::kTankSize3;

// co-prime base delays @ 48k. ~44/59/75/92ms (hall spread)
namespace {
constexpr float kBaseDelay0 = 2089.0f;
constexpr float kBaseDelay1 = 2843.0f;
constexpr float kBaseDelay2 = 3617.0f;
constexpr float kBaseDelay3 = 4421.0f;

constexpr float kPreDelayMaxSamples = 2400.0f;  // 50 ms @ 48k

// max delay-line modulation swing in samples. ±240 ≈ ±5 ms at 48k
constexpr float kMaxModSamples = 240.0f;

// matrix gain compensation. 1.0 lets the loop reach unity gain at decay=1.0
// (cathedral / freeze territory). smoothsat bounds the loop — it asymptotes
// to ±1 so overshoot is absorbed musically. lower values cap rt60 short
constexpr float kMatrixComp = 1.0f;

// air-absorption lp coefficient. ~5 kHz at 48k. always-on hf damping
constexpr float kAbsorbCoef = 0.50f;

// tilt eq split coefficient. ~800 Hz crossover between warm and airy bands
constexpr float kTiltCoef = 0.105f;

// fxengine output ap gain (fixed). dattorro standard
constexpr float kOutAp = 0.5f;

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
  std::memset(tilt_state_, 0, sizeof(tilt_state_));
  std::memset(hpf_state_, 0, sizeof(hpf_state_));

  std::memset(tank_0_, 0, sizeof(tank_0_));
  std::memset(tank_1_, 0, sizeof(tank_1_));
  std::memset(tank_2_, 0, sizeof(tank_2_));
  std::memset(tank_3_, 0, sizeof(tank_3_));
  for (int i = 0; i < 4; ++i) tank_w_[i] = 0;

  lfo_phase_[0] = 0.0f;
  lfo_phase_[1] = 1.5707963f;       // quadrature start
  lfo_phase_inc_[0] = 0.0f;
  lfo_phase_inc_[1] = 0.0f;

  parameters_.decay      = 0.5f;
  parameters_.tone       = 0.5f;
  parameters_.size       = 0.5f;
  parameters_.dry_wet    = 0.5f;
  parameters_.pre_delay  = 0.0f;
  parameters_.diffusion  = 0.5f;
  parameters_.modulation = 0.1f;
  parameters_.low_cut    = 0.0f;

  Tick();
}

void Reverb::Tick() {
  coef_input_gain_         = 0.5f;
  coef_pre_delay_samples_  = parameters_.pre_delay * kPreDelayMaxSamples;
  coef_input_diff_a_       = 0.75f  * parameters_.diffusion;
  coef_input_diff_b_       = 0.625f * parameters_.diffusion;
  coef_branch_diff_        = 0.5f * parameters_.diffusion;
  // decay knob: x*(2-x) curve so middle position already gives long tail.
  // 0.0→0, 0.5→0.75, 0.75→0.94, 1.0→1.0. saturator catches overshoot at top
  {
    const float d = parameters_.decay;
    coef_feedback_ = d * (2.0f - d);
  }
  coef_mod_amplitude_      = parameters_.modulation * kMaxModSamples;
  coef_dry_wet_            = parameters_.dry_wet;

  // low_cut: 1-pole hp in each branch. 5..200 Hz log corner.
  // always slightly active so the loop can't accumulate DC
  const float hp_hz = 5.0f * exp2f(parameters_.low_cut * 5.32f);
  coef_low_cut_hp_  = 2.0f * static_cast<float>(M_PI) * hp_hz / sample_rate_;

  // tilt eq: complementary lp/hp gains around the 800 Hz split.
  // both ≤ 1 → loop stays bounded
  const float tilt = (parameters_.tone - 0.5f) * 2.0f;
  if (tilt >= 0.0f) {
    coef_tilt_lp_gain_ = 1.0f - tilt;
    coef_tilt_hp_gain_ = 1.0f;
  } else {
    coef_tilt_lp_gain_ = 1.0f;
    coef_tilt_hp_gain_ = 1.0f + tilt;
  }

  // tank-mod lfos. rate climbs with modulation knob: 0.1..1.6 Hz log
  const float lfo_hz = 0.1f * exp2f(parameters_.modulation * 4.0f);
  lfo_phase_inc_[0] =
      2.0f * static_cast<float>(M_PI) * lfo_hz / sample_rate_;
  lfo_phase_inc_[1] =
      2.0f * static_cast<float>(M_PI) * lfo_hz * 1.41f / sample_rate_;

  // matrix alpha tracks size knob: identity (small) -> hadamard (cathedral)
  matrix_.set_alpha(parameters_.size);
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
  const float lp_gain   = coef_tilt_lp_gain_;
  const float hp_gain   = coef_tilt_hp_gain_;
  const float wet       = coef_dry_wet_;
  const float ampl      = coef_mod_amplitude_;

  const float size_k = 0.4f + 1.0f * parameters_.size;
  const float tau0 = kBaseDelay0 * size_k;
  const float tau1 = kBaseDelay1 * size_k;
  const float tau2 = kBaseDelay2 * size_k;
  const float tau3 = kBaseDelay3 * size_k;

  float lp0 = lpf_state_[0], lp1 = lpf_state_[1];
  float lp2 = lpf_state_[2], lp3 = lpf_state_[3];
  float hp0 = hpf_state_[0], hp1 = hpf_state_[1];
  float hp2 = hpf_state_[2], hp3 = hpf_state_[3];

  // tank pointers + sizes for the loop
  float* const t0 = tank_0_;  const int n0 = static_cast<int>(kTankSize0);
  float* const t1 = tank_1_;  const int n1 = static_cast<int>(kTankSize1);
  float* const t2 = tank_2_;  const int n2 = static_cast<int>(kTankSize2);
  float* const t3 = tank_3_;  const int n3 = static_cast<int>(kTankSize3);
  int w0 = tank_w_[0], w1 = tank_w_[1], w2 = tank_w_[2], w3 = tank_w_[3];

  float lfo_ph0 = lfo_phase_[0], lfo_ph1 = lfo_phase_[1];
  const float lfo_inc0 = lfo_phase_inc_[0];
  const float lfo_inc1 = lfo_phase_inc_[1];
  constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);

  while (size--) {
    // advance lfos. sinf at 48k×2 ≈ 96k/s — affordable
    lfo_ph0 += lfo_inc0; if (lfo_ph0 > kTwoPi) lfo_ph0 -= kTwoPi;
    lfo_ph1 += lfo_inc1; if (lfo_ph1 > kTwoPi) lfo_ph1 -= kTwoPi;
    const float lfo_v0 = sinf(lfo_ph0);
    const float lfo_v1 = sinf(lfo_ph1);

    engine_.Start(&c);

    // ---- pre-delay ----
    c.Read(in_out->l + in_out->r, gain);
    c.Write(pre_delay, 0.0f);
    c.Interpolate(pre_delay, pre, 1.0f);

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
    const float r2 = ReadTankLinear(t2, n2, w2, tau2 - ampl * lfo_v0);
    const float r3 = ReadTankLinear(t3, n3, w3, tau3 - ampl * lfo_v1);

    // ---- per-branch chain: in-loop ap -> + tank read -> absorb lp -> tilt -> hp
    float branch_pre;
    float b0, b1, b2, b3;

    // branch 0
    c.Load(wet_in);
    c.Read(ap0 TAIL, -kap);
    c.WriteAllPass(ap0, kap);
    float ap_out;
    c.Write(ap_out, 0.0f);
    c.Load(ap_out + r0);
    c.Lp(lp0, kAbsorbCoef);
    c.Write(branch_pre, 0.0f);
    tilt_state_[0] += kTiltCoef * (branch_pre - tilt_state_[0]);
    {
      const float lp_p = tilt_state_[0];
      const float hp_p = branch_pre - lp_p;
      c.Load(lp_p * lp_gain + hp_p * hp_gain);
    }
    c.Hp(hp0, khp);
    c.Write(b0, 0.0f);

    // branch 1
    c.Load(wet_in);
    c.Read(ap1 TAIL, kap);
    c.WriteAllPass(ap1, -kap);
    c.Write(ap_out, 0.0f);
    c.Load(ap_out + r1);
    c.Lp(lp1, kAbsorbCoef);
    c.Write(branch_pre, 0.0f);
    tilt_state_[1] += kTiltCoef * (branch_pre - tilt_state_[1]);
    {
      const float lp_p = tilt_state_[1];
      const float hp_p = branch_pre - lp_p;
      c.Load(lp_p * lp_gain + hp_p * hp_gain);
    }
    c.Hp(hp1, khp);
    c.Write(b1, 0.0f);

    // branch 2
    c.Load(wet_in);
    c.Read(ap2 TAIL, -kap);
    c.WriteAllPass(ap2, kap);
    c.Write(ap_out, 0.0f);
    c.Load(ap_out + r2);
    c.Lp(lp2, kAbsorbCoef);
    c.Write(branch_pre, 0.0f);
    tilt_state_[2] += kTiltCoef * (branch_pre - tilt_state_[2]);
    {
      const float lp_p = tilt_state_[2];
      const float hp_p = branch_pre - lp_p;
      c.Load(lp_p * lp_gain + hp_p * hp_gain);
    }
    c.Hp(hp2, khp);
    c.Write(b2, 0.0f);

    // branch 3
    c.Load(wet_in);
    c.Read(ap3 TAIL, kap);
    c.WriteAllPass(ap3, -kap);
    c.Write(ap_out, 0.0f);
    c.Load(ap_out + r3);
    c.Lp(lp3, kAbsorbCoef);
    c.Write(branch_pre, 0.0f);
    tilt_state_[3] += kTiltCoef * (branch_pre - tilt_state_[3]);
    {
      const float lp_p = tilt_state_[3];
      const float hp_p = branch_pre - lp_p;
      c.Load(lp_p * lp_gain + hp_p * hp_gain);
    }
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

  // peak: instant attack, ~150ms release
  peak_ = peak_block_ > peak_ ? peak_block_ : peak_ * 0.9f;
  peak_block_ = 0.0f;
}

}  // namespace warps_reverb
