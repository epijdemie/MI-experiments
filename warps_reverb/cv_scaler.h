// CV scaler. ADC reads, LPF smoothing, soft-takeover across the shift
// layer, and dispatch into the active mode's parameter mapping table.
//
// Each of the four pot+CV controls (ALGORITHM, TIMBRE, LEVEL 1, LEVEL 2)
// has two layers (unshifted = default, shifted = button-held). The active
// mode's ModeConfig provides a PotMapping per control telling us which
// ReverbParameters field to write to in each layer. CV jacks always feed
// the unshifted parameter (no shifted-layer CV inputs).

#ifndef WARPS_REVERB_CV_SCALER_H_
#define WARPS_REVERB_CV_SCALER_H_

#include "stmlib/stmlib.h"

#include "warps/drivers/adc.h"
#include "warps/drivers/codec.h"

#include "warps_reverb/dsp/parameters.h"
#include "warps_reverb/modes.h"

namespace warps_reverb {

// Movement-engaged absolute pot tracker, two-layer.
//
// Each pot stores one committed value per layer. When you enters a
// layer, the pot's current physical position is snapshotted. Until the
// pot has moved more than kMoveThreshold away from that snapshot, the
// layer's committed value is frozen - the parameter doesn't change just
// because we switched layers. Once movement is detected, the parameter
// jumps to (and then continuously tracks) the pot's current position
// absolutely.
//
// This is the "best of both worlds" between soft-takeover (which makes
// you dial back to a stored value) and pure absolute mode (which silently
// overwrites parameters on layer switch).
class SoftTakeover {
 public:
  void Init(float initial_unshifted, float initial_shifted) {
    committed_[0] = initial_unshifted;
    committed_[1] = initial_shifted;
    pot_at_entry_[0] = 0.0f;
    pot_at_entry_[1] = 0.0f;
    moved_[0] = false;
    moved_[1] = false;
    prev_layer_ = -1;        // forces re-snapshot on first Process
  }

  // Reset all engagement state. Used on mode change so you has to
  // physically move a knob before that knob takes over the mode's defaults.
  inline void Reset(float unshifted, float shifted) {
    committed_[0] = unshifted;
    committed_[1] = shifted;
    moved_[0] = false;
    moved_[1] = false;
    prev_layer_ = -1;
  }

  inline float Process(float pot, int layer) {
    if (layer != prev_layer_) {
      // Layer just entered - snapshot pot, require movement to engage.
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
  // 0.5 % engagement threshold - any deliberate pot nudge engages the layer
  // and starts tracking absolutely. ADC + LPF noise is well below this, so
  // we don't get false engagements at rest. The previous 2 % threshold made
  // the bottom of the knob travel feel like a dead zone before engagement.
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
  void Read(ReverbParameters* p, bool shifted, const ModeConfig& mode_cfg);

  // Call when the mode changes. Re-syncs each pot's committed values to
  // the new mode's defaults and re-arms movement detection - knobs need
  // a physical wiggle before they start writing into the new mode's params.
  void HandleModeChange(const ModeConfig& mode_cfg);

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
