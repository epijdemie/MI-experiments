// cv scaler - 4 pots + 4 cv -> DroneParameters.
// 4 pages × 2 shift layers = 8 slots per pot. >0.5% move re-arms takeover.
// cv always feeds page-0 unshifted: algo->chord, param->lpf, lvl1->pitch (v/oct),
// lvl2->reverb amount

#ifndef WARPS_DRONE_CV_SCALER_H_
#define WARPS_DRONE_CV_SCALER_H_

#include "stmlib/stmlib.h"
#include "stmlib/system/sector_storage_f4xx.h"

#include "warps/drivers/adc.h"
#include "warps/drivers/codec.h"
#include "warps/drivers/normalization_probe.h"

#include "warps_drone/dsp/parameters.h"
#include "warps_drone/settings.h"

namespace warps_drone {


// 8-slot soft-takeover. on slot switch, snapshot pot; only write the new
// slot once pot moves >kMoveThreshold from snapshot
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

  // force-set - used when two pots alias the same param
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

  // smoothed v/oct adc (0..1), shared with the cal ritual.
  //   voct_chord_raw = LVL1 CV (chord pitch)
  //   voct_bass_raw  = PARAM CV (bass pitch)
  inline float voct_chord_raw() const { return lp_state_[warps::ADC_LEVEL_1_CV]; }
  inline float voct_bass_raw()  const { return lp_state_[warps::ADC_PARAMETER_CV]; }


  // smoothed pot positions (0..1). UI uses these to detect "no knob
  // movement" while watching for the cal-entry gesture.
  inline float pot_big()   const { return lp_state_[warps::ADC_ALGORITHM_POT]; }
  inline float pot_small() const { return lp_state_[warps::ADC_PARAMETER_POT]; }
  inline float pot_lvl1()  const { return lp_state_[warps::ADC_LEVEL_1_POT]; }
  inline float pot_lvl2()  const { return lp_state_[warps::ADC_LEVEL_2_POT]; }

  // main loop. persists dirty slots after kKnobSettleBlocks idle
  void MaybeSave();

  void DetectAudioNormalization(warps::Codec::Frame* in, size_t size);

 private:
  // page=0..3, shift=0..1 -> slot=0..7
  static inline int Slot(ControlPage page, bool shifted) {
    return (static_cast<int>(page) << 1) | (shifted ? 1 : 0);
  }

  warps::Adc adc_;
  float      lp_state_[warps::ADC_LAST];


  const Settings* settings_ = nullptr;   // owned by warps_drone.cc

  // Idle V/OCT semitone offset captured at boot (no cable assumed
  // patched). Subtracted from runtime Transform so idle reads as
  // exactly 0 semis — kills the constant detune that would otherwise
  // shift chord/bass pitches when no cable is in.
  float boot_offset_chord_semis_ = 0.0f;
  float boot_offset_bass_semis_  = 0.0f;
  // Slow second-stage LP on the V/OCT semitones — kills sub-semitone
  // ADC jitter that otherwise modulates the K-S delays + modal-bank
  // fundamentals every block and gets amplified into a "stretching"
  // artifact under distortion.
  float voct_chord_lp_ = 0.0f;
  float voct_bass_lp_  = 0.0f;

  SoftTakeover8 algorithm_;
  SoftTakeover8 timbre_;
  SoftTakeover8 level1_;
  SoftTakeover8 level2_;

  // log-structured journal, flash sector 11.
  //   [magic:u16][slot_id:u16][value:f32]  - 8 bytes/record.
  //   slot_id = pot*8 + page*2 + shift, 0..31.
  // boot replays top-to-bottom; later records shadow earlier ones.
  // sector full -> erase + rewrite (only audible save event)
  static constexpr uint16_t kRecordMagic       = 0xDA75;
  static constexpr uint32_t kSectorBase        = 0x080E0000;
  static constexpr uint32_t kSectorEnd         = 0x08100000;
  static constexpr int      kNumSlots          = 32;
  static constexpr uint32_t kKnobSettleBlocks  = 750;   // ~500ms

  uint32_t flash_cursor_ = kSectorBase;

  bool              slot_dirty_[kNumSlots];
  volatile uint32_t slot_idle_blocks_[kNumSlots];

  volatile uint32_t block_tick_ = 0;

  // save fsm: 2 audio blocks per record (id word, then value word)
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
