#include "warps_drone/cv_scaler.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"

namespace warps_drone {

using namespace warps;
using namespace stmlib;

void CvScaler::Init() {
  adc_.Init();
  std::fill(&lp_state_[0], &lp_state_[ADC_LAST], 0.0f);

  // Initial committed values per layer - match Drone::Init defaults so
  // the first reading doesn't snap params to mid-knob.
  // (Layer 0 = unshifted, Layer 1 = shifted.)
  algorithm_.Init(0.80f, 0.30f);  // voicing (zone 6 = major) | pitch (~80 Hz)
  timbre_   .Init(0.60f, 0.55f);  // filter                   | excite
  level1_   .Init(0.30f, 0.30f);  // damping                  | smear
  level2_   .Init(0.60f, 0.85f);  // amount                   | size
}

namespace {

// 1-pole LPF coefficients.
constexpr float kCvLp  = 0.08f;
constexpr float kPotLp = 0.33f * kCvLp;

inline float ClampUnit(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

}  // namespace

void CvScaler::Read(DroneParameters* p, bool shifted) {
  // Smooth ADC inputs.
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

  const int layer = shifted ? 1 : 0;

  // CV inputs are bipolar around 0.5 idle (sign mirrored - see warps).
  const float algo_cv   = 0.5f - lp_state_[ADC_ALGORITHM_CV];
  const float timbre_cv = 0.5f - lp_state_[ADC_PARAMETER_CV];
  const float l1_cv     = 0.5f - lp_state_[ADC_LEVEL_1_CV];
  const float l2_cv     = 0.5f - lp_state_[ADC_LEVEL_2_CV];

  // Tick soft-takeover for every pot with this layer's reading. The
  // committed values for the *other* layer keep their last values so the
  // shifted params don't jump when you releases the button.
  algorithm_.Process(lp_state_[ADC_ALGORITHM_POT], layer);
  timbre_   .Process(lp_state_[ADC_PARAMETER_POT], layer);
  level1_   .Process(lp_state_[ADC_LEVEL_1_POT],   layer);
  level2_   .Process(lp_state_[ADC_LEVEL_2_POT],   layer);

  // Unshifted params get pot (committed[0]) + CV. Always live.
  p->voicing       = ClampUnit(algorithm_.committed(0) + algo_cv);
  p->filter_cutoff = ClampUnit(timbre_   .committed(0) + timbre_cv);
  p->damping       = ClampUnit(level1_   .committed(0) + l1_cv);
  p->reverb_amount = ClampUnit(level2_   .committed(0) + l2_cv);

  // Shifted params get pot only (no CV - Warps has no shifted CV jacks).
  p->pitch         = algorithm_.committed(1);
  p->excite_amount = timbre_   .committed(1);
  p->reverb_to_ks  = level1_   .committed(1);
  p->reverb_size   = level2_   .committed(1);

  adc_.Convert();
}

void CvScaler::DetectAudioNormalization(warps::Codec::Frame*, size_t) {
  // Stub. Drone is self-excited; audio input currently unused.
}

}  // namespace warps_drone
