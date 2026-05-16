// CV scaler - read 4 pots + 4 CV inputs, smooth them, and map onto the
// DroneParameters struct.
//
// Two-axis control system:
//   page    (0..3)  - selected by single-tap of the button
//   shifted (0..1)  - engaged while button is held
//
// Each pot has 4 pages × 2 layers = 8 committed values. Switching
// page or layer re-arms the takeover detector: a knob has to move
// >0.5 % before it starts writing into the new slot.
//
// CV inputs are *always* mapped to Page 0 unshifted params, regardless
// of the page or shift state on the panel. This keeps live modulation
// patched to a sequencer or LFO from breaking when you flip pages.
//
//   ALGO_CV  -> chord (voicing)
//   PARAM_CV -> filter cutoff
//   LVL1_CV  -> pitch (V/oct, summed with quantised pot in semitones)
//   LVL2_CV  -> reverb amount

#ifndef WARPS_DRONE_CV_SCALER_H_
#define WARPS_DRONE_CV_SCALER_H_

#include "stmlib/stmlib.h"
#include "stmlib/system/sector_storage_f4xx.h"

#include "warps/drivers/adc.h"
#include "warps/drivers/codec.h"

#include "warps/drivers/adc.h"

#include "warps_drone/dsp/parameters.h"
#include "warps_drone/settings.h"

namespace warps_drone {

// 8-slot soft-takeover. Same engagement rule as the 2-layer version:
// on a slot switch, snapshot the pot; only start writing the new slot
// once the pot has physically moved >kMoveThreshold from the snapshot.
class SoftTakeover8 {
 public:
  void Init(const float* initial_values_8) {
    for (int i = 0; i < 8; ++i) {
      committed_[i]    = initial_values_8[i];
      pot_at_entry_[i] = 0.0f;
      moved_[i]        = false;
    }
    prev_slot_ = -1;
  }

  inline float Process(float pot, int slot) {
    if (slot != prev_slot_) {
      pot_at_entry_[slot] = pot;
      moved_[slot]        = false;
      prev_slot_          = slot;
    }
    if (!moved_[slot]) {
      const float d = pot - pot_at_entry_[slot];
      if (d > kMoveThreshold || d < -kMoveThreshold) moved_[slot] = true;
    }
    if (moved_[slot]) committed_[slot] = pot;
    return committed_[slot];
  }

  inline float committed(int slot) const { return committed_[slot]; }

  // Force-set committed value for a slot - used when two physical pots
  // alias the same logical parameter and we want to keep both slots in
  // sync (e.g. reverb wet exposed on Page 0 LVL2 AND Page 2 ALGO shift).
  inline void SetCommitted(int slot, float value) {
    committed_[slot] = value;
  }

 private:
  static constexpr float kMoveThreshold = 0.005f;
  float committed_[8];
  float pot_at_entry_[8];
  bool  moved_[8];
  int   prev_slot_;
};

class CvScaler {
 public:
  CvScaler() { }

  void Init(const Settings* settings);
  void Read(DroneParameters* p, ControlPage page, bool shifted);

  // Smoothed raw V/oct ADC value (0..1). Exposed so the UI can capture
  // the two reference points during the calibration ritual without
  // duplicating the LPF chain.
  inline float voct_raw() const { return lp_state_[warps::ADC_LEVEL_1_CV]; }

  // Called from the main loop (not the audio ISR). Persists the current
  // committed values to flash when you has been idle for kSaveIdleBlocks
  // audio blocks (~10 s) since the last knob movement. A dirty-sequence
  // counter ensures no edit is missed even if it happens while a save is
  // in progress - the next idle period will save again.
  void MaybeSave();

  void DetectAudioNormalization(warps::Codec::Frame* in, size_t size);

 private:
  // page=0..3, shift=0..1 -> slot=0..7
  static inline int Slot(ControlPage page, bool shifted) {
    return (static_cast<int>(page) << 1) | (shifted ? 1 : 0);
  }

  warps::Adc adc_;
  float      lp_state_[warps::ADC_LAST];

  // Borrowed pointer (lifetime managed by warps_drone.cc). Holds the
  // V/oct calibration coefficients applied to the raw ADC value when
  // computing parameters_.reserved_a (now semitones, not raw ADC).
  const Settings* settings_ = nullptr;

  SoftTakeover8 algorithm_;
  SoftTakeover8 timbre_;
  SoftTakeover8 level1_;
  SoftTakeover8 level2_;

  // Persistence - log-structured journal in flash sector 11.
  //
  // Each committed-value change is appended as an 8-byte record:
  //   bytes 0..1   = magic tag (0xDA75) - distinguishes a written
  //                  record from erased flash (0xFFFF)
  //   bytes 2..3   = slot_id (0..31) = pot_idx*8 + page*2 + shifted
  //   bytes 4..7   = float value
  //
  // Boot scans the sector top-to-bottom and replays each record into
  // its slot. Later records shadow earlier ones for the same slot.
  // When the sector fills (~16 380 records) we erase and rewrite the
  // current state - the only audible event in the whole save flow.
  // Erase rate: roughly one per ~hour of active editing.
  static constexpr uint16_t kRecordMagic       = 0xDA75;
  static constexpr uint32_t kSectorBase        = 0x080E0000;
  static constexpr uint32_t kSectorEnd         = 0x08100000;
  static constexpr int      kNumSlots          = 32;
  // Per-knob settle time before its latest value gets persisted.
  // ~500 ms at the 1.5 kHz audio block rate.
  static constexpr uint32_t kKnobSettleBlocks  = 750;

  // Next unwritten address in the journal sector.
  uint32_t flash_cursor_ = kSectorBase;

  // Per-slot dirty + idle tracking. A slot becomes dirty when the
  // committed value last seen in Read() differs from this block's.
  // The idle counter for a dirty slot ticks every block until it
  // crosses the settle threshold, at which point MaybeSave persists
  // that one slot.
  bool              slot_dirty_[kNumSlots];
  volatile uint32_t slot_idle_blocks_[kNumSlots];

  // Block tick paces flash writes (one word per audio block).
  volatile uint32_t block_tick_ = 0;

  // Save state machine. Each record is written across two audio blocks:
  // one block writes the id-word (magic | slot_id), the next writes
  // the value word.
  enum SaveState : uint8_t {
    SAVE_IDLE,
    SAVE_WRITE_ID,
    SAVE_WRITE_VALUE,
  };
  SaveState save_state_        = SAVE_IDLE;
  int       save_slot_id_      = -1;
  uint32_t  save_record_addr_  = 0;
  uint32_t  save_id_word_      = 0;
  float     save_value_        = 0.0f;
  uint32_t  save_last_tick_    = 0;

  DISALLOW_COPY_AND_ASSIGN(CvScaler);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_CV_SCALER_H_
