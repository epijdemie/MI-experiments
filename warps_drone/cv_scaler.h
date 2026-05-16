// CV scaler - read 4 pots + 4 CV inputs, smooth them, and map onto the
// DroneParameters struct. Two layers via a shift gesture (button hold):
//
//                  Unshifted (default)        Shifted (button held)
//   ALGORITHM pot  voicing (chord type)       pitch (chord root)
//   PARAMETER pot  filter cutoff              excite_amount (drone fuel)
//   LEVEL 1   pot  damping (KS tone+texture)  reverb_to_ks (smear)
//   LEVEL 2   pot  reverb amount (wet/dry)    reverb size
//
//   All CVs apply to the corresponding unshifted parameter (Warps has
//   no shifted CV jacks). SoftTakeover keeps a per-layer committed
//   value so swapping layers doesn't snap params.
//
//   Audio inputs are owned by Drone, not by this class:
//     IN L (jack 1)  -> continuous audio excitement of the KS voices
//     IN R (jack 2)  -> envelope-followed trigger / gate / LFO input
//                       (rising edge -> re-pluck, same as button tap)

#ifndef WARPS_DRONE_CV_SCALER_H_
#define WARPS_DRONE_CV_SCALER_H_

#include "stmlib/stmlib.h"

#include "warps/drivers/adc.h"
#include "warps/drivers/codec.h"

#include "warps_drone/dsp/parameters.h"

namespace warps_drone {

// Movement-engaged absolute pot tracker, two-layer. Identical pattern to
// warps_reverb - each pot has a committed value per layer, and the
// committed value only follows the pot once you has moved it more
// than kMoveThreshold from where it was when the layer was entered.
class SoftTakeover {
 public:
  void Init(float initial_unshifted, float initial_shifted) {
    committed_[0]     = initial_unshifted;
    committed_[1]     = initial_shifted;
    pot_at_entry_[0]  = 0.0f;
    pot_at_entry_[1]  = 0.0f;
    moved_[0]         = false;
    moved_[1]         = false;
    prev_layer_       = -1;
  }

  inline float Process(float pot, int layer) {
    if (layer != prev_layer_) {
      pot_at_entry_[layer] = pot;
      moved_[layer]        = false;
      prev_layer_          = layer;
    }
    if (!moved_[layer]) {
      const float diff = pot - pot_at_entry_[layer];
      if (diff > kMoveThreshold || diff < -kMoveThreshold) {
        moved_[layer] = true;
      }
    }
    if (moved_[layer]) committed_[layer] = pot;
    return committed_[layer];
  }

  inline float committed(int layer) const { return committed_[layer]; }

 private:
  static constexpr float kMoveThreshold = 0.005f;
  float committed_[2];
  float pot_at_entry_[2];
  bool  moved_[2];
  int   prev_layer_;
};

class CvScaler {
 public:
  CvScaler() { }

  void Init();
  void Read(DroneParameters* p, bool shifted);

  void DetectAudioNormalization(warps::Codec::Frame* in, size_t size);

 private:
  warps::Adc adc_;
  float      lp_state_[warps::ADC_LAST];

  SoftTakeover algorithm_;
  SoftTakeover timbre_;
  SoftTakeover level1_;
  SoftTakeover level2_;

  DISALLOW_COPY_AND_ASSIGN(CvScaler);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_CV_SCALER_H_
