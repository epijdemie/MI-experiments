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

// matrix gain compensation
constexpr float kMatrixComp = 1.0f;

// fxengine output ap gain (fixed). dattorro standard
constexpr float kOutAp = 0.5f;

// hardcoded diffusion (knob was barely audible). full dattorro values now,
// no scaling — more smearing of early reflections so they sound less like
// the dry input punching through the tail
constexpr float kInputDiffA = 0.75f;
constexpr float kInputDiffB = 0.625f;
constexpr float kBranchDiff = 0.70f;

// in-loop absorption LP at fixed bright corner. ~7 kHz @ 48k
constexpr float kAbsorbFixed = 0.60f;

// shimmer grain length as float (matches Reverb::kShimmerGrain = 2048)
constexpr float kShimmerGrainF     = 2048.0f;
constexpr float kShimmerGrainHalfF = 1024.0f;

// smooth sat: y = x / ⁴√(1 + x⁴)
inline float SmoothSat(float x) {
  const float x2 = x * x;
  const float x4 = x2 * x2;
  return x / stmlib::Sqrt(stmlib::Sqrt(1.0f + x4));
}

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

  std::memset(out_pd_l_, 0, sizeof(out_pd_l_));
  std::memset(out_pd_r_, 0, sizeof(out_pd_r_));
  out_pd_w_ = 0;

  shimmer_phase_ = kShimmerGrainF;

  // biquad state cleared
  bq_b0_ = bq_b1_ = bq_b2_ = bq_a1_ = bq_a2_ = 0.0f;
  bq_l_x1_ = bq_l_x2_ = bq_l_y1_ = bq_l_y2_ = 0.0f;
  bq_r_x1_ = bq_r_x2_ = bq_r_y1_ = bq_r_y2_ = 0.0f;

  parameters_.decay         = 0.5f;
  parameters_.output_cutoff = 0.85f;
  parameters_.size          = 0.5f;
  parameters_.dry_wet       = 0.5f;
  parameters_.pre_delay     = 0.0f;
  parameters_.resonance     = 0.0f;
  parameters_.spectral      = 0.0f;
  parameters_.low_cut       = 0.0f;

  Tick();
}

void Reverb::Tick() {
  coef_input_gain_         = 0.5f;
  coef_pre_delay_samples_  = parameters_.pre_delay * kPreDelayMaxSamples;

  // decay knob: x*(2-x) curve - middle position already gives long tail
  {
    const float d = parameters_.decay;
    coef_feedback_ = d * (2.0f - d);
  }

  // dry/wet with equal-power crossfade (sqrt curves) for responsive
  // mid-knob feel. plus narrow dead-band on the endpoints (top/bottom 5%)
  // for guaranteed pure wet/dry zones
  {
    float w = parameters_.dry_wet;
    if (w < 0.05f) w = 0.0f;
    else if (w > 0.95f) w = 1.0f;
    coef_dry_wet_ = w;
    coef_wet_gain_ = stmlib::Sqrt(w);
    coef_dry_gain_ = stmlib::Sqrt(1.0f - w);
  }

  // low_cut: 1-pole hp in each branch. 5..200 Hz log corner
  const float hp_hz = 5.0f * exp2f(parameters_.low_cut * 5.32f);
  coef_low_cut_hp_  = 2.0f * static_cast<float>(M_PI) * hp_hz / sample_rate_;

  // shimmer scale: squared curve × 0.1 max. recirculation amplifies the
  // injection 6-7× in steady state, so ss level ~0.6 input. audible halo.
  // squared curve keeps lower half of knob subtle
  {
    const float s = parameters_.spectral;
    coef_shimmer_ = s * s * 0.1f;
  }

  // post-reverb biquad lp coefficients (RBJ cookbook, direct form I).
  // cutoff: 250 Hz .. 16 kHz log. Q: 0.5 .. 6 (resonance from flat to ringing)
  // fc_min lifted from 100 → 250 Hz because (1-cos(ω)) at very low ω loses
  // single-precision float resolution → biquad doesn't actually attenuate
  const float fc = 250.0f * exp2f(parameters_.output_cutoff * 6.0f);
  const float q  = 0.5f + parameters_.resonance * 5.5f;
  // safe clamp: stop fc from approaching Nyquist
  float fc_clamped = fc;
  if (fc_clamped > 16000.0f) fc_clamped = 16000.0f;
  const float omega = 2.0f * static_cast<float>(M_PI) * fc_clamped / sample_rate_;
  const float sw = sinf(omega);
  const float cw = cosf(omega);
  const float alpha = sw / (2.0f * q);
  const float a0 = 1.0f + alpha;
  const float inv_a0 = 1.0f / a0;
  const float one_minus_cw = 1.0f - cw;
  bq_b0_ = (one_minus_cw * 0.5f) * inv_a0;
  bq_b1_ = one_minus_cw * inv_a0;
  bq_b2_ = bq_b0_;
  bq_a1_ = (-2.0f * cw) * inv_a0;
  bq_a2_ = (1.0f - alpha) * inv_a0;

  matrix_.set_alpha(1.0f);
}

void Reverb::Process(FloatFrame* in_out, size_t size) {
  // fxengine reservations: pre_delay + input diffuser + branch APs + output APs
  // total 6478 of 8192
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
  const float kfb       = coef_feedback_;
  const float khp       = coef_low_cut_hp_;
  const float shimmer_amt = coef_shimmer_;
  const float dry_gain    = coef_dry_gain_;
  const float wet_gain    = coef_wet_gain_;

  const float size_k = 0.4f + 1.0f * parameters_.size;
  const float tau0 = kBaseDelay0 * size_k;
  const float tau1 = kBaseDelay1 * size_k;
  const float tau2 = kBaseDelay2 * size_k;
  const float tau3 = kBaseDelay3 * size_k;

  float lp0 = lpf_state_[0], lp1 = lpf_state_[1];
  float lp2 = lpf_state_[2], lp3 = lpf_state_[3];
  float hp0 = hpf_state_[0], hp1 = hpf_state_[1];
  float hp2 = hpf_state_[2], hp3 = hpf_state_[3];

  float* const t0 = tank_0_;  const int n0 = static_cast<int>(kTankSize0);
  float* const t1 = tank_1_;  const int n1 = static_cast<int>(kTankSize1);
  float* const t2 = tank_2_;  const int n2 = static_cast<int>(kTankSize2);
  float* const t3 = tank_3_;  const int n3 = static_cast<int>(kTankSize3);
  int w0 = tank_w_[0], w1 = tank_w_[1], w2 = tank_w_[2], w3 = tank_w_[3];

  float shimmer_phase = shimmer_phase_;

  // biquad coefficients (constant for the block, computed in Tick)
  const float bb0 = bq_b0_, bb1 = bq_b1_, bb2 = bq_b2_;
  const float ba1 = bq_a1_, ba2 = bq_a2_;
  // biquad state (per channel)
  float lx1 = bq_l_x1_, lx2 = bq_l_x2_, ly1 = bq_l_y1_, ly2 = bq_l_y2_;
  float rx1 = bq_r_x1_, rx2 = bq_r_x2_, ry1 = bq_r_y1_, ry2 = bq_r_y2_;

  // output pre-delay write pointer
  int out_pd_w = out_pd_w_;

  while (size--) {
    engine_.Start(&c);

    // pre-delay
    c.Read(in_out->l + in_out->r, gain);
    c.Write(pre_delay, 0.0f);
    c.Interpolate(pre_delay, pre, 1.0f);

    // input diffuser (hardcoded gains)
    c.Read(input_ap0 TAIL, -kInputDiffA);
    c.WriteAllPass(input_ap0, kInputDiffA);
    c.Read(input_ap1 TAIL, kInputDiffA);
    c.WriteAllPass(input_ap1, -kInputDiffA);
    c.Read(input_ap2 TAIL, -kInputDiffB);
    c.WriteAllPass(input_ap2, kInputDiffB);
    c.Read(input_ap3 TAIL, kInputDiffB);
    c.WriteAllPass(input_ap3, -kInputDiffB);

    float wet_in;
    c.Write(wet_in, 0.0f);

    // tank reads
    const float r0 = ReadTankLinear(t0, n0, w0, tau0);
    const float r1 = ReadTankLinear(t1, n1, w1, tau1);
    const float r2 = ReadTankLinear(t2, n2, w2, tau2);
    const float r3 = ReadTankLinear(t3, n3, w3, tau3);

    // per-branch chain
    float b0, b1, b2, b3;

    c.Load(wet_in + r0);
    c.Read(ap0 TAIL, -kBranchDiff);
    c.WriteAllPass(ap0, kBranchDiff);
    c.Lp(lp0, kAbsorbFixed);
    c.Hp(hp0, khp);
    c.Write(b0, 0.0f);

    c.Load(wet_in + r1);
    c.Read(ap1 TAIL, kBranchDiff);
    c.WriteAllPass(ap1, -kBranchDiff);
    c.Lp(lp1, kAbsorbFixed);
    c.Hp(hp1, khp);
    c.Write(b1, 0.0f);

    c.Load(wet_in + r2);
    c.Read(ap2 TAIL, -kBranchDiff);
    c.WriteAllPass(ap2, kBranchDiff);
    c.Lp(lp2, kAbsorbFixed);
    c.Hp(hp2, khp);
    c.Write(b2, 0.0f);

    c.Load(wet_in + r3);
    c.Read(ap3 TAIL, kBranchDiff);
    c.WriteAllPass(ap3, -kBranchDiff);
    c.Lp(lp3, kAbsorbFixed);
    c.Hp(hp3, khp);
    c.Write(b3, 0.0f);

    // matrix mix + smoothsat → tank write
    float branch_in[4] = { b0, b1, b2, b3 };
    float mixed[4];
    matrix_.Apply(branch_in, mixed);

    // ---- shimmer (octave-up grain reader from tank_3) ----
    // two grain heads offset by kShimmerGrain/2, triangle-windowed crossfade.
    // pair sums to 1 at any offset, so output magnitude ≈ grain magnitude.
    // shifted content injected into all 4 tank writes → builds rising halo
    // through the loop's LP + saturator (bounded, doesn't run away ultrasonic)
    const float sp1 = shimmer_phase;
    float sp2 = sp1 + kShimmerGrainHalfF;
    if (sp2 >= kShimmerGrainF) sp2 -= kShimmerGrainF;

    const float sg1 = ReadTankLinear(t3, n3, w3, sp1);
    const float sg2 = ReadTankLinear(t3, n3, w3, sp2);

    // triangle window (no sinf): w(x) = 1 - |2x - 1|, x ∈ [0, 1]
    const float sn1 = sp1 * (1.0f / kShimmerGrainF);
    const float sn2 = sp2 * (1.0f / kShimmerGrainF);
    float sw1 = 2.0f * sn1 - 1.0f; if (sw1 < 0.0f) sw1 = -sw1; sw1 = 1.0f - sw1;
    float sw2 = 2.0f * sn2 - 1.0f; if (sw2 < 0.0f) sw2 = -sw2; sw2 = 1.0f - sw2;

    const float shimmer = (sg1 * sw1 + sg2 * sw2) * shimmer_amt;

    // delay shrinks by 1 per sample → octave up (read advances at 2× write)
    shimmer_phase -= 1.0f;
    if (shimmer_phase < 0.0f) shimmer_phase += kShimmerGrainF;

    t0[w0] = SmoothSat(mixed[0] * kfb * kMatrixComp + shimmer);
    t1[w1] = SmoothSat(mixed[1] * kfb * kMatrixComp + shimmer);
    t2[w2] = SmoothSat(mixed[2] * kfb * kMatrixComp + shimmer);
    t3[w3] = SmoothSat(mixed[3] * kfb * kMatrixComp + shimmer);
    if (++w0 >= n0) w0 = 0;
    if (++w1 >= n1) w1 = 0;
    if (++w2 >= n2) w2 = 0;
    if (++w3 >= n3) w3 = 0;

    // ---- wet output tap from TANK reads (no direct input bleed) ----
    // r_n contains saturated, decayed, matrix-mixed history. pure tail.
    const float raw_tap_l = r0 + r2;
    const float raw_tap_r = r1 + r3;

    // ---- output pre-delay: shifts ALL wet activity ~40 ms later relative
    // to the dry, so even the first reflection is clearly past the dry
    // transient. ring buffer per channel.
    const float tap_l = out_pd_l_[out_pd_w];
    const float tap_r = out_pd_r_[out_pd_w];
    out_pd_l_[out_pd_w] = raw_tap_l;
    out_pd_r_[out_pd_w] = raw_tap_r;
    if (++out_pd_w >= kOutputPreDelay) out_pd_w = 0;

    // output diffuser
    float diff_l, diff_r;

    c.Load(tap_l);
    c.Read(out_ap_l_a TAIL, -kOutAp);
    c.WriteAllPass(out_ap_l_a, kOutAp);
    c.Read(out_ap_l_b TAIL, kOutAp);
    c.WriteAllPass(out_ap_l_b, -kOutAp);
    c.Write(diff_l, 0.0f);

    c.Load(tap_r);
    c.Read(out_ap_r_a TAIL, -kOutAp);
    c.WriteAllPass(out_ap_r_a, kOutAp);
    c.Read(out_ap_r_b TAIL, kOutAp);
    c.WriteAllPass(out_ap_r_b, -kOutAp);
    c.Write(diff_r, 0.0f);

    // ---- biquad LP per channel (user cutoff + resonance) ----
    const float wet_l = bb0 * diff_l + bb1 * lx1 + bb2 * lx2
                      - ba1 * ly1 - ba2 * ly2;
    lx2 = lx1; lx1 = diff_l;
    ly2 = ly1; ly1 = wet_l;

    const float wet_r = bb0 * diff_r + bb1 * rx1 + bb2 * rx2
                      - ba1 * ry1 - ba2 * ry2;
    rx2 = rx1; rx1 = diff_r;
    ry2 = ry1; ry1 = wet_r;

    float final_l = wet_l;
    float final_r = wet_r;
    if (!std::isfinite(final_l)) final_l = 0.0f;
    if (!std::isfinite(final_r)) final_r = 0.0f;

    const float magl = final_l < 0 ? -final_l : final_l;
    const float magr = final_r < 0 ? -final_r : final_r;
    const float magm = magl > magr ? magl : magr;
    if (magm > peak_block_) peak_block_ = magm;

    // equal-power mix: both gains pre-computed in Tick via sqrt curves.
    // 50/50 sits at -3 dB instead of linear's -6 dB → wet feels present
    // throughout the knob travel, not 'dead until 12 o'clock'
    in_out->l = in_out->l * dry_gain + final_l * wet_gain;
    in_out->r = in_out->r * dry_gain + final_r * wet_gain;
    ++in_out;
  }

  lpf_state_[0] = lp0; lpf_state_[1] = lp1;
  lpf_state_[2] = lp2; lpf_state_[3] = lp3;
  hpf_state_[0] = hp0; hpf_state_[1] = hp1;
  hpf_state_[2] = hp2; hpf_state_[3] = hp3;
  tank_w_[0] = w0; tank_w_[1] = w1;
  tank_w_[2] = w2; tank_w_[3] = w3;
  shimmer_phase_ = shimmer_phase;
  bq_l_x1_ = lx1; bq_l_x2_ = lx2; bq_l_y1_ = ly1; bq_l_y2_ = ly2;
  bq_r_x1_ = rx1; bq_r_x2_ = rx2; bq_r_y1_ = ry1; bq_r_y2_ = ry2;
  out_pd_w_ = out_pd_w;

  peak_ = peak_block_ > peak_ ? peak_block_ : peak_ * 0.9f;
  peak_block_ = 0.0f;
}

}  // namespace warps_reverb
