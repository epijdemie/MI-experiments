#include "warps_reverb/cv_scaler.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"

namespace warps_reverb {

using namespace warps;
using namespace stmlib;

void CvScaler::Init() {
  adc_.Init();
  std::fill(&lp_state_[0], &lp_state_[ADC_LAST], 0.0f);

  // Generic neutral initial values; HandleModeChange() will repaint these
  // from the active mode's defaults once Ui::Init runs.
  algorithm_.Init(0.5f, 0.5f);
  timbre_   .Init(0.5f, 0.5f);
  level1_   .Init(0.5f, 0.5f);
  level2_   .Init(0.5f, 0.5f);

  for (size_t i = 0; i < 4; ++i) prev_pot_value_[i] = 0.0f;
  movement_flag_ = false;
}

void CvScaler::HandleModeChange(const ModeConfig& cfg) {
  // Reset each pot's committed values to the new mode's defaults and
  // re-arm movement detection.
  algorithm_.Reset(
      ReadSlot(cfg.defaults, cfg.algorithm.unshifted),
      ReadSlot(cfg.defaults, cfg.algorithm.shifted));
  timbre_.Reset(
      ReadSlot(cfg.defaults, cfg.timbre.unshifted),
      ReadSlot(cfg.defaults, cfg.timbre.shifted));
  level1_.Reset(
      ReadSlot(cfg.defaults, cfg.level1.unshifted),
      ReadSlot(cfg.defaults, cfg.level1.shifted));
  level2_.Reset(
      ReadSlot(cfg.defaults, cfg.level2.unshifted),
      ReadSlot(cfg.defaults, cfg.level2.shifted));
}

void CvScaler::UpdateLpf() {
  constexpr float kCvLp = 0.08f;
  constexpr float kPotLp = 0.33f * kCvLp;

  lp_state_[ADC_ALGORITHM_POT] += kPotLp *
      (adc_.float_value(ADC_ALGORITHM_POT) - lp_state_[ADC_ALGORITHM_POT]);
  lp_state_[ADC_PARAMETER_POT] += kPotLp *
      (adc_.float_value(ADC_PARAMETER_POT) - lp_state_[ADC_PARAMETER_POT]);
  lp_state_[ADC_LEVEL_1_POT]   += kPotLp *
      (adc_.float_value(ADC_LEVEL_1_POT)   - lp_state_[ADC_LEVEL_1_POT]);
  lp_state_[ADC_LEVEL_2_POT]   += kPotLp *
      (adc_.float_value(ADC_LEVEL_2_POT)   - lp_state_[ADC_LEVEL_2_POT]);

  lp_state_[ADC_ALGORITHM_CV]  += kCvLp *
      (adc_.float_value(ADC_ALGORITHM_CV)  - lp_state_[ADC_ALGORITHM_CV]);
  lp_state_[ADC_PARAMETER_CV]  += kCvLp *
      (adc_.float_value(ADC_PARAMETER_CV)  - lp_state_[ADC_PARAMETER_CV]);
  lp_state_[ADC_LEVEL_1_CV]    += kCvLp *
      (adc_.float_value(ADC_LEVEL_1_CV)    - lp_state_[ADC_LEVEL_1_CV]);
  lp_state_[ADC_LEVEL_2_CV]    += kCvLp *
      (adc_.float_value(ADC_LEVEL_2_CV)    - lp_state_[ADC_LEVEL_2_CV]);
}

namespace {

// Apply pot + CV onto a soft-takeover'd value and write the result into the
// parameter slot identified by id. CV is centred (0.5 at idle); a ±5V swing
// roughly maps to ±0.5 of parameter range.
inline void Dispatch(SoftTakeover* st,
                     float pot, float cv, int layer,
                     ParameterId unshifted_id, ParameterId shifted_id,
                     ReverbParameters* p) {
  // Tick soft-takeover with this layer's reading.
  (void)st->Process(pot, layer);

  // Always write the UNSHIFTED parameter from committed[0] + CV; this stays
  // live regardless of which layer is currently active (so external CV
  // keeps modulating even while you is editing shifted params).
  float v = st->committed(0) + cv;
  CONSTRAIN(v, 0.0f, 1.0f);
  *WriteSlot(p, unshifted_id) = v;

  // SHIFTED parameter from committed[1] (no CV).
  *WriteSlot(p, shifted_id) = st->committed(1);
}

}  // namespace

void CvScaler::Read(ReverbParameters* p, bool shifted,
                    const ModeConfig& mode_cfg) {
  UpdateLpf();

  const int layer = shifted ? 1 : 0;

  // Pot readings (lowpassed).
  const float algo_pot   = lp_state_[ADC_ALGORITHM_POT];
  const float timbre_pot = lp_state_[ADC_PARAMETER_POT];
  const float l1_pot     = lp_state_[ADC_LEVEL_1_POT];
  const float l2_pot     = lp_state_[ADC_LEVEL_2_POT];

  // CV inputs are bipolar around 0.5 idle. Invert sign so a positive CV
  // raises the parameter (the ADC is wired such that idle reads ~0.5 and
  // increasing voltage decreases the float reading - same convention as
  // stock warps' cv_scaler.cc).
  const float algo_cv   = 0.5f - lp_state_[ADC_ALGORITHM_CV];
  const float timbre_cv = 0.5f - lp_state_[ADC_PARAMETER_CV];
  const float l1_cv     = 0.5f - lp_state_[ADC_LEVEL_1_CV];
  const float l2_cv     = 0.5f - lp_state_[ADC_LEVEL_2_CV];

  Dispatch(&algorithm_, algo_pot,   algo_cv,   layer,
           mode_cfg.algorithm.unshifted, mode_cfg.algorithm.shifted, p);
  Dispatch(&timbre_,    timbre_pot, timbre_cv, layer,
           mode_cfg.timbre.unshifted,    mode_cfg.timbre.shifted,    p);
  Dispatch(&level1_,    l1_pot,     l1_cv,     layer,
           mode_cfg.level1.unshifted,    mode_cfg.level1.shifted,    p);
  Dispatch(&level2_,    l2_pot,     l2_cv,     layer,
           mode_cfg.level2.unshifted,    mode_cfg.level2.shifted,    p);

  // Movement flag - distinguish shift gesture from short tap.
  constexpr float kMoveThreshold = 0.01f;
  const float now[4] = { algo_pot, timbre_pot, l1_pot, l2_pot };
  for (int i = 0; i < 4; ++i) {
    if (now[i] - prev_pot_value_[i] > kMoveThreshold ||
        prev_pot_value_[i] - now[i] > kMoveThreshold) {
      movement_flag_ = true;
      prev_pot_value_[i] = now[i];
    }
  }

  adc_.Convert();
}

bool CvScaler::TakeMovementFlag() {
  const bool f = movement_flag_;
  movement_flag_ = false;
  return f;
}

void CvScaler::DetectAudioNormalization(warps::Codec::Frame* in, size_t size) {
  for (int32_t channel = 0; channel < 2; ++channel) {
    int32_t count = 0;
    int16_t* input_samples = &in->l + channel;
    for (size_t i = 0; i < size; i += 16) {
      int16_t s = input_samples[i * 2];
      if (s > 50 && s < 1500) ++count;
      else if (s > -1500 && s < -50) --count;
    }
    (void)count;  // TODO: wire to NormalizationDetector
  }
}

}  // namespace warps_reverb
