#include "warps_reverb/dsp/reverb.h"

#include <cmath>
#include <cstring>

#include "stmlib/utils/random.h"

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

// smooth sat: y = x / ⁴√(1 + x⁴). bounded ±1, smooth knee
inline float SmoothSat(float x) {
  const float x2 = x * x;
  const float x4 = x2 * x2;
  return x / stmlib::Sqrt(stmlib::Sqrt(1.0f + x4));
}

// bipolar white noise, ±1
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

  std::memset(ks_buffer_, 0, sizeof(ks_buffer_));
  std::memset(fb_buffer_, 0, sizeof(fb_buffer_));
  for (int i = 0; i < 2; ++i) {
    ks_write_[i] = 0;
    ks_dc_x1_[i] = 0.0f;
    ks_dc_y1_[i] = 0.0f;
    ks_damping_state_[i] = 0.0f;
    ks_delay_samples_[i] = 200.0f;
    ks_drift_phase_[i] = static_cast<float>(i) * 1.13f;
    ks_drift_phase_inc_[i] = 0.0f;
    fb_write_[i] = 0;
    svf_lp_drone_[i] = 0.0f;
    svf_bp_drone_[i] = 0.0f;
    in_loop_hp_state_[i] = 0.0f;
  }
  ks_drift_depth_ = 0.0f;
  svf_f_base_drone_ = 0.1f;
  svf_q_base_drone_ = 1.0f;

  std::memset(rvb_comb_a_, 0, sizeof(rvb_comb_a_));
  std::memset(rvb_comb_b_, 0, sizeof(rvb_comb_b_));
  std::memset(rvb_comb_c_, 0, sizeof(rvb_comb_c_));
  std::memset(rvb_comb_d_, 0, sizeof(rvb_comb_d_));
  std::memset(rvb_ap_la_, 0, sizeof(rvb_ap_la_));
  std::memset(rvb_ap_lb_, 0, sizeof(rvb_ap_lb_));
  std::memset(rvb_ap_ra_, 0, sizeof(rvb_ap_ra_));
  std::memset(rvb_ap_rb_, 0, sizeof(rvb_ap_rb_));
  for (int i = 0; i < 4; ++i) {
    rvb_comb_w_[i] = 0;
    rvb_comb_lp_[i] = 0.0f;
    rvb_ap_w_[i] = 0;
  }
  rvb_mix_ = 0.0f;
  rvb_feedback_ = 0.92f;
  rvb_damping_ = 0.3f;
  drone_base_noise_ = 0.0f;
  ks_internal_damp_ = 0.95f;
  ks_damping_coef_ = 0.5f;
  drone_fb_gain_ = 0.0f;
  drone_fb_delay_samples_ = 1000.0f;
  drone_drive_ = 1.0f;
  drone_lp_coef_ = 1.0f;
  drone_hp_coef_ = 0.0f;
  drone_noise_amp_ = 0.0f;
  svf_f_base_ = 0.115f;   // ~880 Hz cutoff at 48 kHz - slightly darker
  svf_q_ = 1.4f;          // Butterworth-ish damping - no ringing peak
  filter_morph_ = 0.0f;
  drive_factor_ = 1.0f;
  motion_wobbles_tilt_ = false;
  motion_lfo_phase_ = 0.0f;
  motion_lfo_phase_inc_ = 0.0f;
  motion_lfo_depth_ = 0.0f;
  engine_.Init(buffer);
  std::memset(lpf_state_, 0, sizeof(lpf_state_));

  // placeholder defaults - Ui::EnterMode overwrites before first block
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
  coef_diffusion_ = 0.8f * parameters_.diffusion;
  coef_feedback_ = feedback_min_ +
                   parameters_.decay * (feedback_max_ - feedback_min_);

  // tilt lpf baseline. drone reads from tone; motion lfo modulates per-sample
  if (decouple_tilt_) {
    coef_tilt_lpf_ = 0.3f + 0.68f * parameters_.tone;
  } else {
    coef_tilt_lpf_ = 0.55f + 0.44f * parameters_.motion;
  }

  // fdn delay lfo depth from motion
  coef_mod_amplitude_ = parameters_.motion * 0.6f;

  // drone osc + drift LFOs. carrier log 60..500Hz. drift rates × prime
  // ratios so they never align
  if (osc_amplitude_ > 0.0f) {
    const float osc_hz = 60.0f * exp2f(parameters_.pre_delay * 3.06f);
    osc_phase_inc_ = 2.0f * static_cast<float>(M_PI) * osc_hz / sample_rate_;

    const float base_drift_hz = 0.04f + parameters_.speed * 1.96f;
    constexpr float kPrimes[4] = { 7.0f, 11.0f, 13.0f, 17.0f };
    for (int i = 0; i < 4; ++i) {
      const float hz = base_drift_hz * kPrimes[i] / 7.0f;
      drift_phase_inc_[i] =
          2.0f * static_cast<float>(M_PI) * hz / sample_rate_;
    }
  }

  // motion lfo modulates svf cutoff (drone) or tilt lpf (mode 0). 0.1..50Hz
  if (motion_wobbles_tilt_) {
    const float mod_hz = 0.1f + parameters_.speed * 49.9f;
    motion_lfo_phase_inc_ =
        2.0f * static_cast<float>(M_PI) * mod_hz / sample_rate_;
    motion_lfo_depth_ = parameters_.motion;
  } else {
    motion_lfo_phase_inc_ = 0.0f;
    motion_lfo_depth_ = 0.0f;
  }

  filter_morph_ = parameters_.filter;

  // 1× clean -> 5× harmonic - saturator clamps so this is character not gain
  drive_factor_ = 1.0f + parameters_.drive * 4.0f;

  coef_dry_wet_ = parameters_.dry_wet;

  // 2 lfos at f and f·1.41 - incommensurate
  const float lfo_hz = 0.1f + parameters_.speed * 49.9f;
  engine_.SetLFOFrequency(LFO_1, lfo_hz / sample_rate_);
  engine_.SetLFOFrequency(LFO_2, lfo_hz * 1.41f / sample_rate_);

  // Matrix alpha: mode 0 tracks Size (mixing morph is part of the reverb
  // character). Mode 1 locks alpha at 1.0 because intermediate α produces
  // a rank-deficient mixing matrix that destroys energy in half the FDN's
  // state space - killing self-oscillation. With α=1.0 the matrix is
  // orthogonal and the drone can sustain
  const float alpha = (matrix_alpha_ < 0.0f)
                          ? parameters_.size
                          : matrix_alpha_;
  matrix_.set_alpha(alpha);

  // shimmer placeholder - just adds fb gain + brightness for now
  if (parameters_.shimmer > 0.0f) {
    coef_feedback_ += parameters_.shimmer * 0.15f;
    coef_tilt_lpf_  = stmlib::Crossfade(
        coef_tilt_lpf_, 0.995f, parameters_.shimmer);
  }

  if (parameters_.freeze) {
    coef_feedback_ = 1.0f;
    coef_input_gain_ = 0.0f;
  }

  // ks drone coefficients
  if (decouple_tilt_) {
    // pitch log ~30..500Hz. R channel +3¢ for stereo width
    const float pitch_hz = 30.0f * exp2f(parameters_.pre_delay * 4.06f);
    const float ks_max = static_cast<float>(kKsSize - 4);
    const float delay_l = sample_rate_ / pitch_hz;
    const float delay_r = sample_rate_ / (pitch_hz * 1.0017f);
    ks_delay_samples_[0] = delay_l > ks_max ? ks_max : delay_l;
    ks_delay_samples_[1] = delay_r > ks_max ? ks_max : delay_r;

    // tone -> damping lp cutoff. 0.02 ≈ 150Hz, 0.5 ≈ 6kHz
    ks_damping_coef_ = 0.02f + 0.48f * parameters_.tone;

    // decay walks BOTH string damping AND outer fb gain
    const float decay = parameters_.decay;
    ks_internal_damp_ = 0.85f + decay * 0.14f;
    drone_fb_gain_ = 0.05f + decay * 0.20f;
    drone_base_noise_ = 0.010f;

    drone_fb_delay_samples_ = sample_rate_ * 0.02f;
    if (drone_fb_delay_samples_ > kFbDelSize - 1)
      drone_fb_delay_samples_ = kFbDelSize - 1;

    // 2 drift LFOs (10s, 13s periods) modulate svf cutoff + Q (pitch stable)
    ks_drift_phase_inc_[0] =
        2.0f * static_cast<float>(M_PI) * 0.10f / sample_rate_;
    ks_drift_phase_inc_[1] =
        2.0f * static_cast<float>(M_PI) * 0.076f / sample_rate_;
    ks_drift_depth_ = parameters_.motion;

    // filter -> svf cutoff 150Hz..1.9kHz. Q base ≈ 1.4
    svf_f_base_drone_ = 0.02f + 0.23f * parameters_.filter;
    svf_q_base_drone_ = 0.7f;

    rvb_mix_ = parameters_.dry_wet;
    rvb_feedback_ = 0.99f;        // ~14s RT60
    rvb_damping_ = 0.20f;

    drone_drive_ = 1.0f + parameters_.drive * 2.0f;

    drone_lp_coef_ = 1.0f;
    drone_hp_coef_ = 0.005f;

    drone_noise_amp_ = parameters_.noise * 0.2f;
  }
  // TODO: reverse playback (custom FxEngine read path)
}

// ks drone - string/channel inside outer fb loop. saturator stays in
// series so harmonics are musical (not IM fog)
void Reverb::ProcessKarplusDrone(FloatFrame* in_out, size_t size) {
  const float fb_gain = drone_fb_gain_;
  const float fb_delay = drone_fb_delay_samples_;
  const float drive = drone_drive_;
  const float lp_k = drone_lp_coef_;
  const float hp_k = drone_hp_coef_;
  const float noise_amp = drone_noise_amp_;
  const float damp_k = ks_damping_coef_;

  while (size--) {
    float ext_l = in_out->l;
    float ext_r = in_out->r;
    float out[2];

    for (int ch = 0; ch < 2; ++ch) {
      const float ext = ch == 0 ? ext_l : ext_r;

      // outer fb delay (linear interp)
      const float fbd = fb_delay;
      const int fbd_i = static_cast<int>(fbd);
      const float fbd_f = fbd - static_cast<float>(fbd_i);
      const int fb_rd0 =
          (fb_write_[ch] - fbd_i - 1 + kFbDelSize * 2) % kFbDelSize;
      const int fb_rd1 = (fb_rd0 - 1 + kFbDelSize) % kFbDelSize;
      const float fb_read =
          fb_buffer_[ch][fb_rd0] + (fb_buffer_[ch][fb_rd1] -
                                    fb_buffer_[ch][fb_rd0]) * fbd_f;

      // ks excitation: fb + noise + ext
      const float noise =
          WhiteNoise() * (noise_amp + drone_base_noise_ + 0.0001f);
      const float ks_in = fb_read + noise + ext * 0.25f;

      // drift lfo for svf modulation (not pitch)
      ks_drift_phase_[ch] += ks_drift_phase_inc_[ch];
      if (ks_drift_phase_[ch] > 2.0f * static_cast<float>(M_PI)) {
        ks_drift_phase_[ch] -= 2.0f * static_cast<float>(M_PI);
      }
      const float drift_sin = sinf(ks_drift_phase_[ch]);
      // quadrature cos - cutoff/Q mods 90° out of phase
      const float drift_cos =
          sinf(ks_drift_phase_[ch] + 1.5707963f);

      // ks string: read -> +in -> dc block -> damping lp -> write back
      float kd = ks_delay_samples_[ch];
      if (kd < 4.0f) kd = 4.0f;
      if (kd > kKsSize - 4) kd = kKsSize - 4;
      const int kd_i = static_cast<int>(kd);
      const float kd_f = kd - static_cast<float>(kd_i);
      const int ks_rd0 = (ks_write_[ch] - kd_i - 1 + kKsSize * 2) % kKsSize;
      const int ks_rd1 = (ks_rd0 - 1 + kKsSize) % kKsSize;
      const float ks_read =
          ks_buffer_[ch][ks_rd0] + (ks_buffer_[ch][ks_rd1] -
                                    ks_buffer_[ch][ks_rd0]) * kd_f;

      float s = ks_read + ks_in;
      if (s > 8.0f) s = 8.0f;
      if (s < -8.0f) s = -8.0f;

      const float dc_out = s - ks_dc_x1_[ch] + 0.995f * ks_dc_y1_[ch];
      ks_dc_x1_[ch] = s;
      ks_dc_y1_[ch] = dc_out;
      s = dc_out * ks_internal_damp_;

      // write FULL-SPECTRUM back to the delay - brightness lp is downstream
      ks_buffer_[ch][ks_write_[ch]] = s;
      ks_write_[ch] = (ks_write_[ch] + 1) % kKsSize;

      // outer loop: drift-modulated svf -> hp dc-blocker -> smoothsat
      float outer = s * drive;

      // drift mods cutoff (sin) and Q (cos, ±1 octave / Q 1..3.3)
      float svf_f = svf_f_base_drone_ + drift_sin * ks_drift_depth_ * 0.10f;
      if (svf_f < 0.01f) svf_f = 0.01f;
      if (svf_f > 0.30f) svf_f = 0.30f;
      float svf_q = svf_q_base_drone_ - drift_cos * ks_drift_depth_ * 0.4f;
      if (svf_q < 0.3f) svf_q = 0.3f;
      if (svf_q > 1.4f) svf_q = 1.4f;

      // chamberlin svf
      const float hp = outer - svf_lp_drone_[ch] - svf_q * svf_bp_drone_[ch];
      svf_bp_drone_[ch] += svf_f * hp;
      svf_lp_drone_[ch] += svf_f * svf_bp_drone_[ch];
      outer = svf_lp_drone_[ch];

      // hp dc-blocker
      in_loop_hp_state_[ch] += hp_k * (outer - in_loop_hp_state_[ch]);
      outer = outer - in_loop_hp_state_[ch];
      (void)lp_k; (void)drive;

      // smoothsat bounds loop energy - svf can peak above 1 at high Q
      outer = SmoothSat(outer);

      out[ch] = outer;
    }

    // schroeder reverb (mono in, stereo via separate AP chains)
    const float rvb_in = (out[0] + out[1]) * 0.20f;

    // 4 parallel combs, lp-damped feedback

    const float ca = rvb_comb_a_[rvb_comb_w_[0]];
    const float cb = rvb_comb_b_[rvb_comb_w_[1]];
    const float cc = rvb_comb_c_[rvb_comb_w_[2]];
    const float cd = rvb_comb_d_[rvb_comb_w_[3]];

    rvb_comb_lp_[0] += rvb_damping_ * (ca - rvb_comb_lp_[0]);
    rvb_comb_lp_[1] += rvb_damping_ * (cb - rvb_comb_lp_[1]);
    rvb_comb_lp_[2] += rvb_damping_ * (cc - rvb_comb_lp_[2]);
    rvb_comb_lp_[3] += rvb_damping_ * (cd - rvb_comb_lp_[3]);

    rvb_comb_a_[rvb_comb_w_[0]] = rvb_in + rvb_comb_lp_[0] * rvb_feedback_;
    rvb_comb_b_[rvb_comb_w_[1]] = rvb_in + rvb_comb_lp_[1] * rvb_feedback_;
    rvb_comb_c_[rvb_comb_w_[2]] = rvb_in + rvb_comb_lp_[2] * rvb_feedback_;
    rvb_comb_d_[rvb_comb_w_[3]] = rvb_in + rvb_comb_lp_[3] * rvb_feedback_;

    rvb_comb_w_[0] = (rvb_comb_w_[0] + 1) % kRvbCombA;
    rvb_comb_w_[1] = (rvb_comb_w_[1] + 1) % kRvbCombB;
    rvb_comb_w_[2] = (rvb_comb_w_[2] + 1) % kRvbCombC;
    rvb_comb_w_[3] = (rvb_comb_w_[3] + 1) % kRvbCombD;

    const float comb_sum = (ca + cb + cc + cd) * 0.25f;

    // diffusion 2-AP / channel
    const float kApGain = 0.5f;
    float sL = comb_sum;
    {
      const float buf = rvb_ap_la_[rvb_ap_w_[0]];
      const float v = buf - kApGain * sL;
      const float y = kApGain * v + sL;
      rvb_ap_la_[rvb_ap_w_[0]] = y;
      rvb_ap_w_[0] = (rvb_ap_w_[0] + 1) % kRvbApLa;
      sL = v;
    }
    {
      const float buf = rvb_ap_lb_[rvb_ap_w_[1]];
      const float v = buf - kApGain * sL;
      const float y = kApGain * v + sL;
      rvb_ap_lb_[rvb_ap_w_[1]] = y;
      rvb_ap_w_[1] = (rvb_ap_w_[1] + 1) % kRvbApLb;
      sL = v;
    }
    float sR = comb_sum;
    {
      const float buf = rvb_ap_ra_[rvb_ap_w_[2]];
      const float v = buf - kApGain * sR;
      const float y = kApGain * v + sR;
      rvb_ap_ra_[rvb_ap_w_[2]] = y;
      rvb_ap_w_[2] = (rvb_ap_w_[2] + 1) % kRvbApRa;
      sR = v;
    }
    {
      const float buf = rvb_ap_rb_[rvb_ap_w_[3]];
      const float v = buf - kApGain * sR;
      const float y = kApGain * v + sR;
      rvb_ap_rb_[rvb_ap_w_[3]] = y;
      rvb_ap_w_[3] = (rvb_ap_w_[3] + 1) % kRvbApRb;
      sR = v;
    }

    // outer fb delay write - dry + small reverb send (long-tail integration)
    const float mix = rvb_mix_;
    fb_buffer_[0][fb_write_[0]] = (out[0] + sL * mix * 0.05f) * fb_gain;
    fb_write_[0] = (fb_write_[0] + 1) % kFbDelSize;
    fb_buffer_[1][fb_write_[1]] = (out[1] + sR * mix * 0.05f) * fb_gain;
    fb_write_[1] = (fb_write_[1] + 1) % kFbDelSize;

    // scale combs (fb=0.99 -> DC × ~100, resonances higher) so dry mix is sane
    sL *= 0.20f;
    sR *= 0.20f;

    // brightness lp out-of-loop, audible only
    ks_damping_state_[0] += damp_k * (out[0] - ks_damping_state_[0]);
    ks_damping_state_[1] += damp_k * (out[1] - ks_damping_state_[1]);
    const float bright_l = ks_damping_state_[0];
    const float bright_r = ks_damping_state_[1];

    // mix×4 so wet dominates at high mix
    const float out_l = (bright_l + sL * mix * 4.0f) * 0.4f;
    const float out_r = (bright_r + sR * mix * 4.0f) * 0.4f;


    const float ml = out_l < 0 ? -out_l : out_l;
    const float mr = out_r < 0 ? -out_r : out_r;
    const float m  = ml > mr ? ml : mr;
    if (m > peak_block_) peak_block_ = m;

    in_out->l = out_l;
    in_out->r = out_r;
    ++in_out;
  }

  peak_ = peak_block_ > peak_ ? peak_block_ : peak_ * 0.9f;
  peak_block_ = 0.0f;
}

void Reverb::Process(FloatFrame* in_out, size_t size) {
  if (decouple_tilt_) {
    ProcessKarplusDrone(in_out, size);
    return;
  }

  // FxEngine reservations: pre_delay 4800 + 4×ap 1024 + 4×del 24800 = 30624
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
  const float klp_base  = coef_tilt_lpf_;
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

  while (size--) {
    engine_.Start(&c);

    // ---- Pre-delay ----
    c.Read(in_out->l + in_out->r, gain);
    c.Write(pre_delay, 0.0f);
    c.Interpolate(pre_delay, pre, LFO_1, 0.0f, 1.0f);
    float wet_in;
    c.Write(wet_in, 0.0f);

    // 4-voice unison injection. motion scales drift ±~50¢ (factor 0.03)
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
      wet_in += voice_sum * (osc_amplitude_ * 0.25f);
    }

    // strega-style brown-ish noise inject
    if (noise_max_ > 0.0f && parameters_.noise > 0.0f) {
      const float white = WhiteNoise();
      noise_lp_state_ += 0.06f * (white - noise_lp_state_);
      wet_in += noise_lp_state_ * parameters_.noise * noise_max_;
    }

    // motion lfo - drives both mode 0 tilt lpf and drone svf cutoff
    float lfo_value = 0.0f;
    if (motion_lfo_depth_ > 0.0f) {
      motion_lfo_phase_ += motion_lfo_phase_inc_;
      if (motion_lfo_phase_ > 2.0f * static_cast<float>(M_PI)) {
        motion_lfo_phase_ -= 2.0f * static_cast<float>(M_PI);
      }
      lfo_value = sinf(motion_lfo_phase_);
    }

    // mode-0 tilt lpf coef
    float klp_sample = klp_base + lfo_value * motion_lfo_depth_ * 0.25f;
    if (klp_sample < 0.05f) klp_sample = 0.05f;
    if (klp_sample > 0.99f) klp_sample = 0.99f;

    // drone svf cutoff: f ~0.06..0.20 (~460..1530Hz)
    float svf_f = svf_f_base_ + lfo_value * motion_lfo_depth_ * 0.07f;
    if (svf_f < 0.02f) svf_f = 0.02f;
    if (svf_f > 0.25f) svf_f = 0.25f;

    // per-branch: ap diffuser -> modulated delay -> one-pole lpf.
    // drone mode skips per-branch lpf (svf runs post-matrix instead)
    float b0, b1, b2, b3;

    c.Load(wet_in);
    c.Read(ap0 TAIL, -kap);
    c.WriteAllPass(ap0, kap);
    c.Interpolate(del0, tau0, LFO_1, ampl * tau0, 1.0f);
    if (!decouple_tilt_) c.Lp(lp0, klp_sample);
    c.Write(b0, 0.0f);

    c.Load(wet_in);
    c.Read(ap1 TAIL, kap);
    c.WriteAllPass(ap1, -kap);
    c.Interpolate(del1, tau1, LFO_2, ampl * tau1, 1.0f);
    if (!decouple_tilt_) c.Lp(lp1, klp_sample);
    c.Write(b1, 0.0f);

    c.Load(wet_in);
    c.Read(ap2 TAIL, -kap);
    c.WriteAllPass(ap2, kap);
    c.Interpolate(del2, tau2, LFO_1, -ampl * tau2, 1.0f);
    if (!decouple_tilt_) c.Lp(lp2, klp_sample);
    c.Write(b2, 0.0f);

    c.Load(wet_in);
    c.Read(ap3 TAIL, kap);
    c.WriteAllPass(ap3, -kap);
    c.Interpolate(del3, tau3, LFO_2, -ampl * tau3, 1.0f);
    if (!decouple_tilt_) c.Lp(lp3, klp_sample);
    c.Write(b3, 0.0f);

    float branch_in[4]  = { b0, b1, b2, b3 };
    float mixed[4];
    matrix_.Apply(branch_in, mixed);

    // drone: svf on recirculation only - output tap stays unfiltered
    if (decouple_tilt_) {
      for (int i = 0; i < 4; ++i) {
        const float hp = mixed[i] - svf_lp_[i] - svf_q_ * svf_bp_[i];
        svf_bp_[i] += svf_f * hp;
        svf_lp_[i] += svf_f * svf_bp_[i];
        const float lp = svf_lp_[i];
        mixed[i] = lp * (1.0f - filter_morph_) + hp * filter_morph_;
      }
    }

    // fb writes - matrix_comp: 0.7 ≈ 1/√2 (transient sat), 1.0 = continuous
    const float kMatrixComp = matrix_comp_;
    const float drive = drive_factor_;
    c.Load(SmoothSat(mixed[0] * kfb * kMatrixComp * drive));
    c.Write(del0, 0.0f);
    c.Load(SmoothSat(mixed[1] * kfb * kMatrixComp * drive));
    c.Write(del1, 0.0f);
    c.Load(SmoothSat(mixed[2] * kfb * kMatrixComp * drive));
    c.Write(del2, 0.0f);
    c.Load(SmoothSat(mixed[3] * kfb * kMatrixComp * drive));
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

  // peak: instant attack, ~150ms release
  peak_ = peak_block_ > peak_ ? peak_block_ : peak_ * 0.9f;
  peak_block_ = 0.0f;
}

}  // namespace warps_reverb
