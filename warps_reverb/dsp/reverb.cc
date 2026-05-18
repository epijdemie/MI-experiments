#include "warps_reverb/dsp/reverb.h"

#include <cmath>
#include <cstring>

namespace warps_reverb {

using clouds::LFO_1;
using clouds::LFO_2;

constexpr size_t Reverb::kBufferSize;

// co-prime base delays @ 48k. ~44/59/75/92ms (hall spread)
namespace {
constexpr float kBaseDelay0 = 2089.0f;
constexpr float kBaseDelay1 = 2843.0f;
constexpr float kBaseDelay2 = 3617.0f;
constexpr float kBaseDelay3 = 4421.0f;

constexpr float kPreDelayMaxSamples = 4800.0f;  // 100ms @ 48k

// matrix gain compensation. 0.7 ≈ 1/√2 (peak of hadamard at α=1)
constexpr float kMatrixComp = 0.7f;

// smooth sat: y = x / ⁴√(1 + x⁴). bounded ±1, smooth knee
inline float SmoothSat(float x) {
  const float x2 = x * x;
  const float x4 = x2 * x2;
  return x / stmlib::Sqrt(stmlib::Sqrt(1.0f + x4));
}
}  // namespace

void Reverb::Init(uint16_t* buffer, float sample_rate) {
  sample_rate_ = sample_rate;
  peak_ = 0.0f;
  peak_block_ = 0.0f;

  engine_.Init(buffer);
  std::memset(lpf_state_, 0, sizeof(lpf_state_));
  std::memset(hpf_state_, 0, sizeof(hpf_state_));

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
  coef_diffusion_          = 0.8f * parameters_.diffusion;
  coef_feedback_           = parameters_.decay;        // 0..1
  coef_tilt_lpf_           = 0.3f + 0.68f * parameters_.tone;
  coef_mod_amplitude_      = parameters_.modulation * 0.6f;
  coef_dry_wet_            = parameters_.dry_wet;

  // low_cut: 1-pole hp in each branch. 5..200 Hz log corner.
  // k = 2π * f / fs. always slightly active so the loop can't accumulate DC
  const float hp_hz = 5.0f * exp2f(parameters_.low_cut * 5.32f);
  coef_low_cut_hp_  = 2.0f * static_cast<float>(M_PI) * hp_hz / sample_rate_;

  // modulation knob drives lfo rate too. 0.1..1.6 Hz log
  const float lfo_hz = 0.1f * exp2f(parameters_.modulation * 4.0f);
  engine_.SetLFOFrequency(LFO_1, lfo_hz / sample_rate_);
  engine_.SetLFOFrequency(LFO_2, lfo_hz * 1.41f / sample_rate_);

  // matrix alpha tracks size knob: identity (small) -> hadamard (cathedral)
  matrix_.set_alpha(parameters_.size);
}

void Reverb::Process(FloatFrame* in_out, size_t size) {
  // fxengine reservations: pre_delay 4800 + 4×ap 1024 + 4×del 24800 = 30624
  typedef Engine::Reserve<4800,
          Engine::Reserve<256,
          Engine::Reserve<256,
          Engine::Reserve<256,
          Engine::Reserve<256,
          Engine::Reserve<6200,
          Engine::Reserve<6200,
          Engine::Reserve<6200,
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

  const float gain      = coef_input_gain_;
  const float pre       = coef_pre_delay_samples_;
  const float kap       = coef_diffusion_;
  const float kfb       = coef_feedback_;
  const float klp       = coef_tilt_lpf_;
  const float khp       = coef_low_cut_hp_;
  const float wet       = coef_dry_wet_;
  const float ampl      = coef_mod_amplitude_;
  // size 0.4..1.4× - longest base 4421 × 1.4 = 6190 fits in 6200 reservation
  const float size_k = 0.4f + 1.0f * parameters_.size;

  const float tau0 = kBaseDelay0 * size_k;
  const float tau1 = kBaseDelay1 * size_k;
  const float tau2 = kBaseDelay2 * size_k;
  const float tau3 = kBaseDelay3 * size_k;

  float lp0 = lpf_state_[0];
  float lp1 = lpf_state_[1];
  float lp2 = lpf_state_[2];
  float lp3 = lpf_state_[3];

  float hp0 = hpf_state_[0];
  float hp1 = hpf_state_[1];
  float hp2 = hpf_state_[2];
  float hp3 = hpf_state_[3];

  while (size--) {
    engine_.Start(&c);

    // ---- pre-delay ----
    c.Read(in_out->l + in_out->r, gain);
    c.Write(pre_delay, 0.0f);
    c.Interpolate(pre_delay, pre, LFO_1, 0.0f, 1.0f);
    float wet_in;
    c.Write(wet_in, 0.0f);

    // per-branch: ap diffuser -> modulated delay -> 1-pole lpf
    float b0, b1, b2, b3;

    c.Load(wet_in);
    c.Read(ap0 TAIL, -kap);
    c.WriteAllPass(ap0, kap);
    c.Interpolate(del0, tau0, LFO_1, ampl * tau0, 1.0f);
    c.Lp(lp0, klp);
    c.Hp(hp0, khp);
    c.Write(b0, 0.0f);

    c.Load(wet_in);
    c.Read(ap1 TAIL, kap);
    c.WriteAllPass(ap1, -kap);
    c.Interpolate(del1, tau1, LFO_2, ampl * tau1, 1.0f);
    c.Lp(lp1, klp);
    c.Hp(hp1, khp);
    c.Write(b1, 0.0f);

    c.Load(wet_in);
    c.Read(ap2 TAIL, -kap);
    c.WriteAllPass(ap2, kap);
    c.Interpolate(del2, tau2, LFO_1, -ampl * tau2, 1.0f);
    c.Lp(lp2, klp);
    c.Hp(hp2, khp);
    c.Write(b2, 0.0f);

    c.Load(wet_in);
    c.Read(ap3 TAIL, kap);
    c.WriteAllPass(ap3, -kap);
    c.Interpolate(del3, tau3, LFO_2, -ampl * tau3, 1.0f);
    c.Lp(lp3, klp);
    c.Hp(hp3, khp);
    c.Write(b3, 0.0f);

    float branch_in[4]  = { b0, b1, b2, b3 };
    float mixed[4];
    matrix_.Apply(branch_in, mixed);

    // fb writes - smoothsat bounds the loop, matrix_comp tames hadamard peak
    c.Load(SmoothSat(mixed[0] * kfb * kMatrixComp));
    c.Write(del0, 0.0f);
    c.Load(SmoothSat(mixed[1] * kfb * kMatrixComp));
    c.Write(del1, 0.0f);
    c.Load(SmoothSat(mixed[2] * kfb * kMatrixComp));
    c.Write(del2, 0.0f);
    c.Load(SmoothSat(mixed[3] * kfb * kMatrixComp));
    c.Write(del3, 0.0f);

    // stereo tap: 0+2 -> L, 1+3 -> R
    const float wet_l = b0 + b2;
    const float wet_r = b1 + b3;
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

  hpf_state_[0] = hp0;
  hpf_state_[1] = hp1;
  hpf_state_[2] = hp2;
  hpf_state_[3] = hp3;

  // peak: instant attack, ~150ms release
  peak_ = peak_block_ > peak_ ? peak_block_ : peak_ * 0.9f;
  peak_block_ = 0.0f;
}

}  // namespace warps_reverb
