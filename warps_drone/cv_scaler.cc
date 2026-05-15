#include "warps_drone/cv_scaler.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"

namespace warps_drone {

using namespace warps;
using namespace stmlib;

namespace {

inline float ClampUnit(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

// Initial committed values for a pot across all 8 slots. Index order:
//   [page0_unshifted, page0_shifted, page1_u, page1_s, page2_u, page2_s,
//    page3_u, page3_s]
// These should match Drone::Init() defaults - first reading shouldn't
// snap any parameter.

// Default committed values for every (page, shift) slot.
// Indices: 0=PAGE_PERFORMANCE u, 1=p0 s, 2=PAGE_KARPLUS u, 3=p1 s,
//          4=PAGE_REVERB u,     5=p2 s, 6=PAGE_SHIMMER u, 7=p3 s.

// Captured live from the module - these were the values dialled in at
// the time of the readout (sus4 / +3rd stack, pulsing soft plucks at
// ~2.5 Hz, long reverb with heavy smear). They sit close to each
// physical pot position so the first-engagement jump is small.

// ALGO   :  chord stack      | chord mode      |
//           damping          | modal mix       |
//           reverb size      | predelay (NYI)  |
//           reserved         | modal count
// KARPLUS shifted ALGO (idx 3) is the modal-bank mix; default 0.40
// reproduces the previous locked value so first boot sounds identical
// to before the knob was exposed.
constexpr float kAlgoDefaults[8]   = {
    0.49f, 0.91f, 0.99f, 0.40f, 0.87f, 0.00f, 0.40f, 0.50f
};

// PARAM  :  pulse freq       | reserved        |
//           white/pink mix   | reserved        |
//           diffusion        | smear           |
//           modal bright     | pickup
// PERFORMANCE PARAM unshifted is the K-S exciter pulse-osc frequency
// (log 50 Hz..250 Hz). KARPLUS PARAM unshifted is noise spectrum mix
// (0 = pink, 1 = white). REVERB diffusion (idx 4) and smear (idx 5)
// default to 0 so the reverb starts clean while dialling in sources.
constexpr float kParamDefaults[8]  = {
    0.50f, 1.00f, 0.50f, 0.00f, 0.00f, 0.00f, 0.60f, 0.50f
};

// LVL1   :  pitch octave     | pitch (semitones)|
//           ks LPF cutoff    | reserved        |
//           reverb lp        | reverb drive    |
//           modal stiffness  | shimmer depth
// ks LPF default 0.85 -> ~8 kHz, bright but with the worst hiss tamed.
// REVERB-page reverb_drive (idx 5) defaulted to 0 so reverb input gain
// sits at its minimum (0.30) - clean baseline for dial-in.
constexpr float kLvl1Defaults[8]   = {
    0.60f, 0.66f, 0.85f, 0.00f, 0.40f, 0.00f, 0.20f, 0.50f
};

// LVL2   :  LPF cutoff       | HPF cutoff      |
//           noise floor base | reserved        |
//           reverb amount    | reserved        |
//           dynamics         | shimmer rate
// PERFORMANCE LVL2 hosts both filters (unshifted = LPF, shifted = HPF,
// inverted). KARPLUS noise floor default 0.20 - some baseline noise
// fuel so the K-S has something to chew on out of the box. Reverb wet
// lives on REVERB-page LVL2 unshifted now - no more PERF/REVERB dup.
constexpr float kLvl2Defaults[8]   = {
    0.73f, 1.00f, 0.20f, 0.00f, 0.44f, 0.50f, 0.00f, 0.50f
};

}  // namespace

void CvScaler::Init(const Settings* settings) {
  settings_ = settings;
  adc_.Init();
  std::fill(&lp_state_[0], &lp_state_[ADC_LAST], 0.0f);

  // CV inputs idle at ~0.5 ADC (mid-rail bias). Initialise the LPF
  // state there so the first dozen blocks don't see a giant fake
  // CV offset while the LPF catches up to the real reading.
  lp_state_[ADC_ALGORITHM_CV] = 0.5f;
  lp_state_[ADC_PARAMETER_CV] = 0.5f;
  lp_state_[ADC_LEVEL_1_CV]   = 0.5f;
  lp_state_[ADC_LEVEL_2_CV]   = 0.5f;

  // Start each pot's slots at the hardcoded defaults; the journal
  // replay below will overwrite any slots that have been saved.
  float defaults[4][8];
  for (int s = 0; s < 8; ++s) {
    defaults[0][s] = kAlgoDefaults[s];
    defaults[1][s] = kParamDefaults[s];
    defaults[2][s] = kLvl1Defaults[s];
    defaults[3][s] = kLvl2Defaults[s];
  }

  // Scan the journal sector top-to-bottom. The 4 PERFORMANCE-page
  // unshifted slots (slot index 0 for each pot - chord / cutoff /
  // pitch / reverb wet) are intentionally skipped during replay
  // because they're not persisted at all. Those follow the live
  // physical pot position, set further down.
  struct FlashRecord {
    uint16_t magic;
    uint16_t slot_id;
    float    value;
  };
  const FlashRecord* rec   = reinterpret_cast<const FlashRecord*>(kSectorBase);
  const FlashRecord* end   = reinterpret_cast<const FlashRecord*>(kSectorEnd);
  const FlashRecord* first = rec;

  if (first->magic == 0xFFFF && first->slot_id == 0xFFFF) {
    flash_cursor_ = kSectorBase;
  } else if (first->magic == kRecordMagic) {
    while (rec < end &&
           rec->magic    == kRecordMagic &&
           rec->slot_id  <  kNumSlots) {
      const int pot  = rec->slot_id / 8;
      const int slot = rec->slot_id % 8;
      if (slot != 0) {                       // skip performance slots
        defaults[pot][slot] = rec->value;
      }
      ++rec;
    }
    flash_cursor_ = reinterpret_cast<uint32_t>(rec);
  } else {
    // Corrupted or a previous firmware's data - erase to recover.
    // Codec hasn't started yet, so the 1.1 s erase is silent.
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    FLASH_EraseSector(11 * 8, VoltageRange_3);
    flash_cursor_ = kSectorBase;
  }

  // Read the live physical pot positions for the 4 PERFORMANCE-page
  // unshifted slots. Trigger an ADC conversion and busy-wait for the
  // 8-channel scan to complete (~400 µs at ADC clock; we wait a bit
  // longer to be safe). This runs before the codec is started, so
  // there's no audio in flight to worry about.
  adc_.Convert();
  for (volatile uint32_t i = 0; i < 100000; ++i) { __NOP(); }
  defaults[0][0] = adc_.float_value(ADC_ALGORITHM_POT);
  defaults[1][0] = adc_.float_value(ADC_PARAMETER_POT);
  defaults[2][0] = adc_.float_value(ADC_LEVEL_1_POT);
  defaults[3][0] = adc_.float_value(ADC_LEVEL_2_POT);

  // Seed the LPF with the live pot reading too, so the first audio
  // block doesn't see a transient as the LPF settles.
  lp_state_[ADC_ALGORITHM_POT] = defaults[0][0];
  lp_state_[ADC_PARAMETER_POT] = defaults[1][0];
  lp_state_[ADC_LEVEL_1_POT]   = defaults[2][0];
  lp_state_[ADC_LEVEL_2_POT]   = defaults[3][0];

  algorithm_.Init(defaults[0]);
  timbre_   .Init(defaults[1]);
  level1_   .Init(defaults[2]);
  level2_   .Init(defaults[3]);

  for (int i = 0; i < kNumSlots; ++i) {
    slot_dirty_[i]       = false;
    slot_idle_blocks_[i] = kKnobSettleBlocks;
  }
  block_tick_ = 0;
  save_state_ = SAVE_IDLE;
}

void CvScaler::Read(DroneParameters* p, ControlPage page, bool shifted) {
  constexpr float kCvLp  = 0.08f;
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

  const int slot = Slot(page, shifted);

  // Snapshot committed values before Process() so we can detect any
  // movement that actually moved a slot (not just CV/ADC jitter).
  const float prev_algo = algorithm_.committed(slot);
  const float prev_prm  = timbre_   .committed(slot);
  const float prev_l1   = level1_   .committed(slot);
  const float prev_l2   = level2_   .committed(slot);

  // Tick all four pots for the active slot.
  algorithm_.Process(lp_state_[ADC_ALGORITHM_POT], slot);
  timbre_   .Process(lp_state_[ADC_PARAMETER_POT], slot);
  level1_   .Process(lp_state_[ADC_LEVEL_1_POT],   slot);
  level2_   .Process(lp_state_[ADC_LEVEL_2_POT],   slot);

  // Per-slot dirty tracking. The 4 PERFORMANCE-page unshifted slots
  // (slot == 0) are *not* persisted - they follow the live pot and
  // would otherwise junk-fill the flash sector during normal play.
  if (slot != 0) {
    const float now_algo = algorithm_.committed(slot);
    const float now_prm  = timbre_   .committed(slot);
    const float now_l1   = level1_   .committed(slot);
    const float now_l2   = level2_   .committed(slot);

    if (prev_algo != now_algo) {
      const int sid = 0 * 8 + slot;
      slot_dirty_[sid]       = true;
      slot_idle_blocks_[sid] = 0;
    }
    if (prev_prm != now_prm) {
      const int sid = 1 * 8 + slot;
      slot_dirty_[sid]       = true;
      slot_idle_blocks_[sid] = 0;
    }
    if (prev_l1 != now_l1) {
      const int sid = 2 * 8 + slot;
      slot_dirty_[sid]       = true;
      slot_idle_blocks_[sid] = 0;
    }
    if (prev_l2 != now_l2) {
      const int sid = 3 * 8 + slot;
      slot_dirty_[sid]       = true;
      slot_idle_blocks_[sid] = 0;
    }
  }

  // Tick the idle counter of every still-dirty slot.
  for (int i = 0; i < kNumSlots; ++i) {
    if (slot_dirty_[i] && slot_idle_blocks_[i] < kKnobSettleBlocks) {
      ++slot_idle_blocks_[i];
    }
  }
  // Block tick paces the save state machine.
  ++block_tick_;

  // CVs are bipolar around 0.5 idle; positive voltage decreases reading.
  const float algo_cv   = 0.5f - lp_state_[ADC_ALGORITHM_CV];
  const float timbre_cv = 0.5f - lp_state_[ADC_PARAMETER_CV];
  const float l2_cv     = 0.5f - lp_state_[ADC_LEVEL_2_CV];
  // LVL1 CV is V/oct - keep it raw (Drone does the semitone math).
  const float l1_cv_raw = lp_state_[ADC_LEVEL_1_CV];

  // ----------------------------------------------------------------
  // Page 0 unshifted slots, plus CV summed in for the four CV-able ones.
  // ----------------------------------------------------------------
  p->chord         = ClampUnit(algorithm_.committed(Slot(PAGE_PERFORMANCE, false)) + algo_cv);
  // PARAM unshifted on PERFORMANCE = K-S exciter pulse-osc frequency.
  // The pulse-osc replaces the internal noise burst as the primary
  // K-S excitation; its harmonic content gets filtered into pitch by
  // each string in the chord bank (same trick as Strega-via-IN-L).
  p->pulse_freq    = ClampUnit(timbre_   .committed(Slot(PAGE_PERFORMANCE, false)) + timbre_cv);
  // LVL1 unshifted = octave selector; PERF-unshifted slots are journal-
  // excluded so this always follows the live pot at boot. Fine semitone
  // pitch lives on the shifted layer and DOES get persisted.
  p->pitch_octave  = ClampUnit(level1_   .committed(Slot(PAGE_PERFORMANCE, false)));
  // LVL2 unshifted = LPF cutoff (moved here from PARAM unshifted to make
  // room for the pulse-freq knob). LVL2 CV follows the parameter.
  p->lpf_cutoff    = ClampUnit(level2_   .committed(Slot(PAGE_PERFORMANCE, false)) + l2_cv);

  // Stash the *calibrated* V/oct value (in semitones from C-something)
  // where Drone expects it. We piggyback on reserved_a (an internal
  // scratch field that nothing else writes). The settings pointer is
  // nullable in tests; production always provides it via Init().
  p->reserved_a = settings_
      ? settings_->calibration().Transform(l1_cv_raw)
      : (0.5f - l1_cv_raw) * 120.0f;

  // ----------------------------------------------------------------
  // Page 0 shifted
  // ----------------------------------------------------------------
  p->chord_mode       = algorithm_.committed(Slot(PAGE_PERFORMANCE, true));
  // PARAM shifted: reserved (previously HPF, now on LVL2 shifted).
  p->pitch            = level1_   .committed(Slot(PAGE_PERFORMANCE, true));
  // LVL2 shifted = HPF cutoff (inverted on knob, CW = open).
  p->hpf_cutoff       = level2_   .committed(Slot(PAGE_PERFORMANCE, true));
  // filter_resonance unplugged - left at the Init default (Q = 0.7,
  // Butterworth-flat). Re-expose on a future slot if it becomes useful.

  // ----------------------------------------------------------------
  // Page 1 KARPLUS
  // ----------------------------------------------------------------
  // KARPLUS page - plucking machinery removed. The four unshifted slots
  // are tone-shaping the K-S strings; all shifted slots are reserved.
  //   BIG    unshifted = damping             |  shifted = reserved
  //   SMALL  unshifted = white/pink mix      |  shifted = reserved
  //   LVL1   unshifted = ks LPF cutoff       |  shifted = reserved
  //   LVL2   unshifted = noise floor base    |  shifted = reserved
  p->damping          = algorithm_.committed(Slot(PAGE_KARPLUS,     false));
  p->white_pink_mix   = timbre_   .committed(Slot(PAGE_KARPLUS,     false));
  p->karplus_lpf      = level1_   .committed(Slot(PAGE_KARPLUS,     false));
  p->noise_floor_base = level2_   .committed(Slot(PAGE_KARPLUS,     false));
  // BIG + shift on KARPLUS = modal-bank mix. Lives here so you
  // can dial the harmonic-forest density (cleaner ↔ washier reverb)
  // from the same page that shapes the K-S tone.
  p->harmonics        = algorithm_.committed(Slot(PAGE_KARPLUS,     true));
  // Other shifted-layer slots on KARPLUS are still reserved.

  // ----------------------------------------------------------------
  // Page 2 REVERB
  //   BIG   unshifted = reverb_size      |  shifted = reverb_predelay (NYI)
  //   SMALL unshifted = reverb_diffusion |  shifted = smear
  //   LVL1  unshifted = reverb_lp        |  shifted = reverb_drive
  //   LVL2  unshifted = reverb_amount    |  shifted = reverb_res_b (reserved)
  //
  // Reverb wet lives ONLY here now (no longer mirrored on PERFORMANCE
  // LVL2). LVL2 CV no longer routes to wet - the PERFORMANCE LVL2 CV
  // jack now controls LPF cutoff. Wet is knob-only.
  // ----------------------------------------------------------------
  p->reverb_size      = algorithm_.committed(Slot(PAGE_REVERB,      false));
  p->reverb_diffusion = timbre_   .committed(Slot(PAGE_REVERB,      false));
  p->reverb_lp        = level1_   .committed(Slot(PAGE_REVERB,      false));
  p->reverb_amount    = level2_   .committed(Slot(PAGE_REVERB,      false));
  p->reverb_predelay  = algorithm_.committed(Slot(PAGE_REVERB,      true));
  p->smear            = timbre_   .committed(Slot(PAGE_REVERB,      true));
  p->reverb_drive     = level1_   .committed(Slot(PAGE_REVERB,      true));
  p->reverb_res_b     = level2_   .committed(Slot(PAGE_REVERB,      true));

  // ----------------------------------------------------------------
  // Page 3 SHIMMER & GLIMMER (DSP disabled; values still tracked)
  // ----------------------------------------------------------------
  // SHIMMER ALGO unshifted: was harmonics, now moved to KARPLUS shift.
  // Slot is reserved.
  p->modal_brightness = timbre_   .committed(Slot(PAGE_SHIMMER,     false));
  p->modal_stiffness  = level1_   .committed(Slot(PAGE_SHIMMER,     false));
  p->dynamics         = level2_   .committed(Slot(PAGE_SHIMMER,     false));
  p->modal_count      = algorithm_.committed(Slot(PAGE_SHIMMER,     true));
  p->modal_pickup     = timbre_   .committed(Slot(PAGE_SHIMMER,     true));
  p->reverb_shimmer   = level1_   .committed(Slot(PAGE_SHIMMER,     true));
  p->reverb_shim_rate = level2_   .committed(Slot(PAGE_SHIMMER,     true));

  adc_.Convert();
}

void CvScaler::MaybeSave() {
  switch (save_state_) {
    case SAVE_IDLE: {
      // Find the first dirty slot whose idle counter has reached the
      // settle threshold. One slot per pass so each MaybeSave call
      // returns quickly.
      int found = -1;
      for (int i = 0; i < kNumSlots; ++i) {
        if (slot_dirty_[i] && slot_idle_blocks_[i] >= kKnobSettleBlocks) {
          found = i;
          break;
        }
      }
      if (found < 0) return;

      // Sector wrap? Erase + re-dirty everything so the post-erase
      // sector ends up with the current state of all 32 slots.
      // The 1.1 s erase blocks the flash interface; this is the only
      // audible event in the save flow. Happens once per ~16 380
      // records, which is many hours of editing.
      if (flash_cursor_ + 8 > kSectorEnd) {
        FLASH_Unlock();
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                        FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
        FLASH_EraseSector(11 * 8, VoltageRange_3);
        flash_cursor_ = kSectorBase;
        for (int i = 0; i < kNumSlots; ++i) {
          slot_dirty_[i]       = true;
          slot_idle_blocks_[i] = kKnobSettleBlocks;
        }
        return;  // pick up the first slot on the next call
      }

      // Read the current value for the slot we found.
      const int pot  = found / 8;
      const int slot = found % 8;
      float value = 0.0f;
      switch (pot) {
        case 0: value = algorithm_.committed(slot); break;
        case 1: value = timbre_   .committed(slot); break;
        case 2: value = level1_   .committed(slot); break;
        case 3: value = level2_   .committed(slot); break;
      }

      save_slot_id_     = found;
      save_record_addr_ = flash_cursor_;
      save_id_word_     = static_cast<uint32_t>(kRecordMagic) |
                          (static_cast<uint32_t>(found) << 16);
      save_value_       = value;
      flash_cursor_    += 8;

      FLASH_Unlock();
      FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                      FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
      save_state_     = SAVE_WRITE_ID;
      save_last_tick_ = block_tick_;
      break;
    }

    case SAVE_WRITE_ID: {
      if (block_tick_ == save_last_tick_) return;
      save_last_tick_ = block_tick_;
      FLASH_ProgramWord(save_record_addr_, save_id_word_);
      save_state_ = SAVE_WRITE_VALUE;
      break;
    }

    case SAVE_WRITE_VALUE: {
      if (block_tick_ == save_last_tick_) return;
      save_last_tick_ = block_tick_;
      uint32_t value_word;
      __builtin_memcpy(&value_word, &save_value_, sizeof(float));
      FLASH_ProgramWord(save_record_addr_ + 4, value_word);
      slot_dirty_[save_slot_id_] = false;
      save_state_ = SAVE_IDLE;
      break;
    }
  }
}

void CvScaler::DetectAudioNormalization(warps::Codec::Frame*, size_t) {
  // Stub. Audio inputs are used by Drone (excitement + trigger).
}

}  // namespace warps_drone
