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

// spectral oscillator rates (Hz). slow + mutually incommensurate so the 4
// lines never re-sync → tail keeps evolving
constexpr float kSpectralRateHz[4] = { 0.073f, 0.097f, 0.131f, 0.181f };

// smooth sat: y = x / ⁴√(1 + x⁴). bounded ±1, smooth knee
inline float SmoothSat(float x) {
  const float x2 = x * x;
  const float x4 = x2 * x2;
  return x / stmlib::Sqrt(stmlib::Sqrt(1.0f + x4));
}

// tank read with linear interpolation. delay in [0, size-1].
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

  // initialize 4 recurrent cos oscillators. one-time sinf/cosf at boot,
  // then ONE mul + ONE sub per sample per osc forever after.
  // y[n] = c*y[n-1] - y[n-2] generates cos(ω*n + phase).
  // seed: y2 = cos(phase - ω), y1 = cos(phase)
  for (int i = 0; i < 4; ++i) {
    const float omega = 2.0f * static_cast<float>(M_PI)
                      * kSpectralRateHz[i] / sample_rate_;
    osc_c_[i] = 2.0f * cosf(omega);
    const float phase = static_cast<float>(i) * 0.78539816f;  // π/4 stagger
    osc_y1_[i] = cosf(phase);
    osc_y2_[i] = cosf(phase - omega);
  }

  parameters_.decay     = 0.5f;
  parameters_.diffusion = 0.5f;
  parameters_.size      = 0.5f;
  parameters_.dry_wet   = 0.5f;
  parameters_.pre_delay = 0.0f;
  parameters_.damping   = 0.5f;
  parameters_.spectral  = 0.0f;
  parameters_.low_cut   = 0.0f;

  Tick();
}

void Reverb::Tick() {
  coef_input_gain_         = 0.5f;
  coef_pre_delay_samples_  = parameters_.pre_delay * kPreDelayMaxSamples;

  // input diffuser: dattorro pair scaled by knob
  coef_input_diff_a_       = 0.75f  * parameters_.diffusion;
  coef_input_diff_b_       = 0.625f * parameters_.diffusion;
  // in-loop ap is the only diffuser inside the feedback loop - higher gain
  coef_branch_diff_        = 0.7f   * parameters_.diffusion;

  // decay knob: x*(2-x) curve so middle position already gives long tail.
  // 0.0→0, 0.5→0.75, 0.75→0.94, 1.0→1.0
  {
    const float d = parameters_.decay;
    coef_feedback_ = d * (2.0f - d);
  }

  coef_dry_wet_ = parameters_.dry_wet;

  // low_cut: 1-pole hp in each branch. 5..200 Hz log corner
  const float hp_hz = 5.0f * exp2f(parameters_.low_cut * 5.32f);
  coef_low_cut_hp_  = 2.0f * static_cast<float>(M_PI) * hp_hz / sample_rate_;

  // damping = LP coefficient center. maps knob to musical brightness range.
  // k = 0.1 → ~750 Hz (very dark), 0.5 → ~5 kHz, 0.9 → ~14 kHz (open)
  coef_damping_ = 0.1f + 0.8f * parameters_.damping;

  // spectral depth: lfo swing around damping center. capped so k stays in
  // safe range. max swing ±0.3 means k ∈ [damp±0.3] clamped to [0.05, 0.95]
  coef_spectral_ = 0.30f * parameters_.spectral;

  // matrix locked at full hadamard
  matrix_.set_alpha(1.0f);
}

void Reverb::Process(FloatFrame* in_out, size_t size) {
  // fxengine reservations (samples @ 48k):
  //   pre_delay         2400           50 ms
  //   input_ap0..3      230,173,613,448  dattorro input diffuser (~28 ms)
  //   branch_ap0..3     4 × 256        in-loop diffusion
  //   out_ap_l_a/b      230, 560       output diffuser L
  //   out_ap_r_a/b      320, 480       output diffuser R
  // total: 6478 of 8192
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
  const float damp_k    = coef_damping_;
  const float spectral  = coef_spectral_;
  const float wet       = coef_dry_wet_;

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

  // oscillator state to locals for tight inner loop
  float y1_0 = osc_y1_[0], y2_0 = osc_y2_[0]; const float oc0 = osc_c_[0];
  float y1_1 = osc_y1_[1], y2_1 = osc_y2_[1]; const float oc1 = osc_c_[1];
  float y1_2 = osc_y1_[2], y2_2 = osc_y2_[2]; const float oc2 = osc_c_[2];
  float y1_3 = osc_y1_[3], y2_3 = osc_y2_[3]; const float oc3 = osc_c_[3];

  while (size--) {
    // advance 4 recurrent cos oscillators. ONE mul + ONE sub per osc
    const float v0 = oc0 * y1_0 - y2_0; y2_0 = y1_0; y1_0 = v0;
    const float v1 = oc1 * y1_1 - y2_1; y2_1 = y1_1; y1_1 = v1;
    const float v2 = oc2 * y1_2 - y2_2; y2_2 = y1_2; y1_2 = v2;
    const float v3 = oc3 * y1_3 - y2_3; y2_3 = y1_3; y1_3 = v3;

    // per-branch lp coefficient = damp ± spectral * lfo, clamped to safe range
    float k_b0 = damp_k + spectral * v0;
    float k_b1 = damp_k + spectral * v1;
    float k_b2 = damp_k + spectral * v2;
    float k_b3 = damp_k + spectral * v3;
    if (k_b0 < 0.05f) k_b0 = 0.05f; else if (k_b0 > 0.95f) k_b0 = 0.95f;
    if (k_b1 < 0.05f) k_b1 = 0.05f; else if (k_b1 > 0.95f) k_b1 = 0.95f;
    if (k_b2 < 0.05f) k_b2 = 0.05f; else if (k_b2 > 0.95f) k_b2 = 0.95f;
    if (k_b3 < 0.05f) k_b3 = 0.05f; else if (k_b3 > 0.95f) k_b3 = 0.95f;

    engine_.Start(&c);

    // ---- pre-delay ----
    c.Read(in_out->l + in_out->r, gain);
    c.Write(pre_delay, 0.0f);
    c.Interpolate(pre_delay, pre, 1.0f);

    // ---- input diffuser: 4 series APs, alternating signs ----
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

    // ---- tank reads (static delays) ----
    const float r0 = ReadTankLinear(t0, n0, w0, tau0);
    const float r1 = ReadTankLinear(t1, n1, w1, tau1);
    const float r2 = ReadTankLinear(t2, n2, w2, tau2);
    const float r3 = ReadTankLinear(t3, n3, w3, tau3);

    // ---- per-branch: ap-in-loop, modulated absorb-lp, low-cut hp ----
    float b0, b1, b2, b3;

    c.Load(wet_in + r0);
    c.Read(ap0 TAIL, -kap);
    c.WriteAllPass(ap0, kap);
    c.Lp(lp0, k_b0);
    c.Hp(hp0, khp);
    c.Write(b0, 0.0f);

    c.Load(wet_in + r1);
    c.Read(ap1 TAIL, kap);
    c.WriteAllPass(ap1, -kap);
    c.Lp(lp1, k_b1);
    c.Hp(hp1, khp);
    c.Write(b1, 0.0f);

    c.Load(wet_in + r2);
    c.Read(ap2 TAIL, -kap);
    c.WriteAllPass(ap2, kap);
    c.Lp(lp2, k_b2);
    c.Hp(hp2, khp);
    c.Write(b2, 0.0f);

    c.Load(wet_in + r3);
    c.Read(ap3 TAIL, kap);
    c.WriteAllPass(ap3, -kap);
    c.Lp(lp3, k_b3);
    c.Hp(hp3, khp);
    c.Write(b3, 0.0f);

    // ---- matrix mix + smoothsat → tank write ----
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
  osc_y1_[0] = y1_0; osc_y2_[0] = y2_0;
  osc_y1_[1] = y1_1; osc_y2_[1] = y2_1;
  osc_y1_[2] = y1_2; osc_y2_[2] = y2_2;
  osc_y1_[3] = y1_3; osc_y2_[3] = y2_3;

  peak_ = peak_block_ > peak_ ? peak_block_ : peak_ * 0.9f;
  peak_block_ = 0.0f;
}

}  // namespace warps_reverb
