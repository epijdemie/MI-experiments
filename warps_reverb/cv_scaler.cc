#include "warps_reverb/cv_scaler.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"

namespace warps_reverb {

using namespace warps;
using namespace stmlib;

void CvScaler::Init() {
  adc_.Init();
  std::fill(&lp_state_[0], &lp_state_[ADC_LAST], 0.0f);

  // neutral seeds - HandleReset repaints from kConfig.defaults
  algorithm_.Init(0.5f, 0.5f);
  timbre_   .Init(0.5f, 0.5f);
  level1_   .Init(0.5f, 0.5f);
  level2_   .Init(0.5f, 0.5f);

  for (size_t i = 0; i < 4; ++i) prev_pot_value_[i] = 0.0f;
  movement_flag_ = false;
}

void CvScaler::HandleReset() {
  algorithm_.Reset(
      ReadSlot(kConfig.defaults, kConfig.algorithm.unshifted),
      ReadSlot(kConfig.defaults, kConfig.algorithm.shifted));
  timbre_.Reset(
      ReadSlot(kConfig.defaults, kConfig.timbre.unshifted),
      ReadSlot(kConfig.defaults, kConfig.timbre.shifted));
  level1_.Reset(
      ReadSlot(kConfig.defaults, kConfig.level1.unshifted),
      ReadSlot(kConfig.defaults, kConfig.level1.shifted));
  level2_.Reset(
      ReadSlot(kConfig.defaults, kConfig.level2.unshifted),
      ReadSlot(kConfig.defaults, kConfig.level2.shifted));
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

// pot+cv -> soft-takeover'd param. cv is bipolar around 0.5 (±5V -> ±0.5)
inline void Dispatch(SoftTakeover* st,
                     float pot, float cv, int layer,
                     const PotMapping& map,
                     ReverbParameters* p) {
  (void)st->Process(pot, layer);

  // unshifted always live (cv keeps modulating during shift edits)
  float v = st->committed(0) + cv;
  CONSTRAIN(v, 0.0f, 1.0f);
  *WriteSlot(p, map.unshifted) = v;

  *WriteSlot(p, map.shifted) = st->committed(1);
}

}  // namespace

void CvScaler::Read(ReverbParameters* p, bool shifted) {
  UpdateLpf();

  const int layer = shifted ? 1 : 0;

  const float algo_pot   = lp_state_[ADC_ALGORITHM_POT];
  const float timbre_pot = lp_state_[ADC_PARAMETER_POT];
  const float l1_pot     = lp_state_[ADC_LEVEL_1_POT];
  const float l2_pot     = lp_state_[ADC_LEVEL_2_POT];

  // cv bipolar around 0.5; +V drops the reading (stock warps convention)
  const float algo_cv   = 0.5f - lp_state_[ADC_ALGORITHM_CV];
  const float timbre_cv = 0.5f - lp_state_[ADC_PARAMETER_CV];
  const float l1_cv     = 0.5f - lp_state_[ADC_LEVEL_1_CV];
  const float l2_cv     = 0.5f - lp_state_[ADC_LEVEL_2_CV];

  Dispatch(&algorithm_, algo_pot,   algo_cv,   layer, kConfig.algorithm, p);
  Dispatch(&timbre_,    timbre_pot, timbre_cv, layer, kConfig.timbre,    p);
  Dispatch(&level1_,    l1_pot,     l1_cv,     layer, kConfig.level1,    p);
  // LEVEL 2 CV intentionally zeroed: the level 2 jack on this hardware
  // appears to normalize to a non-mid voltage, causing cv to subtract ~0.5
  // from the wet param. result was full-CW pot only reaching ~50% wet.
  // dry/wet is now pot-only on LEVEL 2 (other params still receive CV)
  (void)l2_cv;
  Dispatch(&level2_,    l2_pot,     0.0f,      layer, kConfig.level2,    p);

  // movement flag - separates shift gesture from a tap
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
