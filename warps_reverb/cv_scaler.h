// cv scaler - adc, lpf, soft-takeover across shift, dispatch via fixed panel
// mapping. 4 pots × 2 layers (unshifted / button-held). cv jacks feed unshifted

#ifndef WARPS_REVERB_CV_SCALER_H_
#define WARPS_REVERB_CV_SCALER_H_

#include "stmlib/stmlib.h"

#include "warps/drivers/adc.h"
#include "warps/drivers/codec.h"

#include "warps_reverb/dsp/parameters.h"
#include "warps_reverb/modes.h"

namespace warps_reverb {

// 2-layer soft-takeover. on layer entry, snapshot pot; once it moves
// >kMoveThreshold, switch to live absolute tracking
class SoftTakeover {
 public:
  void Init(float initial_unshifted, float initial_shifted) {
    committed_[0] = initial_unshifted;
    committed_[1] = initial_shifted;
    pot_at_entry_[0] = 0.0f;
    pot_at_entry_[1] = 0.0f;
    moved_[0] = false;
    moved_[1] = false;
    prev_layer_ = -1;
  }

  // re-arm: knob must wiggle to engage
  inline void Reset(float unshifted, float shifted) {
    committed_[0] = unshifted;
    committed_[1] = shifted;
    moved_[0] = false;
    moved_[1] = false;
    prev_layer_ = -1;
  }

  inline float Process(float pot, int layer) {
    if (layer != prev_layer_) {
      pot_at_entry_[layer] = pot;
      moved_[layer] = false;
      prev_layer_ = layer;
    }
    if (!moved_[layer]) {
      const float diff = pot - pot_at_entry_[layer];
      if (diff > kMoveThreshold || diff < -kMoveThreshold) {
        moved_[layer] = true;
      }
    }
    if (moved_[layer]) {
      committed_[layer] = pot;
    }
    return committed_[layer];
  }

  inline float committed(int layer) const { return committed_[layer]; }

 private:
  // 0.5% - well above adc+lpf noise floor
  static constexpr float kMoveThreshold = 0.005f;
  float committed_[2];
  float pot_at_entry_[2];
  bool  moved_[2];
  int   prev_layer_;
};

class CvScaler {
 public:
  CvScaler() { }
  ~CvScaler() { }

  void Init();
  void Read(ReverbParameters* p, bool shifted);

  // re-sync committed values to panel defaults, re-arm takeover
  void HandleReset();

  bool TakeMovementFlag();

  void DetectAudioNormalization(warps::Codec::Frame* in, size_t size);

 private:
  void UpdateLpf();

  warps::Adc adc_;
  float lp_state_[warps::ADC_LAST];

  SoftTakeover algorithm_;
  SoftTakeover timbre_;
  SoftTakeover level1_;
  SoftTakeover level2_;

  float prev_pot_value_[4];
  bool  movement_flag_;

  DISALLOW_COPY_AND_ASSIGN(CvScaler);
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_CV_SCALER_H_
