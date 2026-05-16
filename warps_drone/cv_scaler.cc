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

// slot defaults - must match Drone::Init() so first read doesn't snap.
// slot = page*2 + shift:
//   0 PERF u | 1 PERF s | 2 KARPLUS u | 3 KARPLUS s
//   4 REVERB u | 5 REVERB s | 6 OVERDRIVE u | 7 OVERDRIVE s

// algo:  chord | mode | damping | modal mix | rev size | predelay | _ | _
constexpr float kAlgoDefaults[8]   = {
    0.30f, 0.25f, 0.99f, 0.40f, 0.87f, 0.00f, 0.40f, 0.50f
};

// param: pulse freq | _ | white/pink | _ | diffusion | smear | warmth | pickup
constexpr float kParamDefaults[8]  = {
    0.50f, 1.00f, 0.50f, 0.50f, 0.00f, 0.00f, 1.00f, 0.50f
};

// lvl1:  octave | pitch (st) | ks lpf | _ | rev lp | rev drive | tone | _
constexpr float kLvl1Defaults[8]   = {
    0.60f, 0.50f, 0.85f, 0.50f, 0.40f, 0.00f, 1.00f, 0.00f
};

// lvl2:  lpf | Q | noise floor | modal stiff | rev amt | shim rate | bias | vinyl
constexpr float kLvl2Defaults[8]   = {
    0.73f, 0.00f, 0.20f, 0.20f, 0.44f, 0.50f, 0.00f, 0.00f
};

}  // namespace

void CvScaler::Init(const Settings* settings) {
  settings_ = settings;
  adc_.Init();
  std::fill(&lp_state_[0], &lp_state_[ADC_LAST], 0.0f);

  // cv idles at ~0.5 adc - seed lpf so first blocks don't see a fake spike
  lp_state_[ADC_ALGORITHM_CV] = 0.5f;
  lp_state_[ADC_PARAMETER_CV] = 0.5f;
  lp_state_[ADC_LEVEL_1_CV]   = 0.5f;
  lp_state_[ADC_LEVEL_2_CV]   = 0.5f;

  // start at hardcoded defaults; journal replay below overrides saved slots
  float defaults[4][8];
  for (int s = 0; s < 8; ++s) {
    defaults[0][s] = kAlgoDefaults[s];
    defaults[1][s] = kParamDefaults[s];
    defaults[2][s] = kLvl1Defaults[s];
    defaults[3][s] = kLvl2Defaults[s];
  }

  // replay journal. PERF unshifted (slot 0) is not persisted - follows
  // the live pot position, captured below
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
      if (slot != 0) {
        defaults[pot][slot] = rec->value;
      }
      ++rec;
    }
    flash_cursor_ = reinterpret_cast<uint32_t>(rec);
  } else {
    // corrupted / old firmware data - erase. silent (codec not started)
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    FLASH_EraseSector(11 * 8, VoltageRange_3);
    flash_cursor_ = kSectorBase;
  }

  // capture live pot positions for PERF unshifted slots. adc scan ~400µs;
  // safe margin. codec not started yet
  adc_.Convert();
  for (volatile uint32_t i = 0; i < 100000; ++i) { __NOP(); }
  defaults[0][0] = adc_.float_value(ADC_ALGORITHM_POT);
  defaults[1][0] = adc_.float_value(ADC_PARAMETER_POT);
  defaults[2][0] = adc_.float_value(ADC_LEVEL_1_POT);
  defaults[3][0] = adc_.float_value(ADC_LEVEL_2_POT);

  // seed lpf with live pot reading - no first-block transient
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

  // snapshot for movement detection
  const float prev_algo = algorithm_.committed(slot);
  const float prev_prm  = timbre_   .committed(slot);
  const float prev_l1   = level1_   .committed(slot);
  const float prev_l2   = level2_   .committed(slot);

  algorithm_.Process(lp_state_[ADC_ALGORITHM_POT], slot);
  timbre_   .Process(lp_state_[ADC_PARAMETER_POT], slot);
  level1_   .Process(lp_state_[ADC_LEVEL_1_POT],   slot);
  level2_   .Process(lp_state_[ADC_LEVEL_2_POT],   slot);

  // PERF unshifted (slot 0) not persisted - would junk-fill the journal
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

  for (int i = 0; i < kNumSlots; ++i) {
    if (slot_dirty_[i] && slot_idle_blocks_[i] < kKnobSettleBlocks) {
      ++slot_idle_blocks_[i];
    }
  }
  ++block_tick_;

  // cv bipolar around 0.5; +V drops the reading
  const float algo_cv   = 0.5f - lp_state_[ADC_ALGORITHM_CV];
  const float timbre_cv = 0.5f - lp_state_[ADC_PARAMETER_CV];
  const float l2_cv     = 0.5f - lp_state_[ADC_LEVEL_2_CV];
  // lvl1 cv = v/oct, raw (Drone does the semitone math)
  const float l1_cv_raw = lp_state_[ADC_LEVEL_1_CV];

  // PERF unshifted, +cv
  p->chord         = ClampUnit(algorithm_.committed(Slot(PAGE_PERFORMANCE, false)) + algo_cv);
  p->pulse_freq    = ClampUnit(timbre_   .committed(Slot(PAGE_PERFORMANCE, false)) + timbre_cv);
  p->pitch_octave  = ClampUnit(level1_   .committed(Slot(PAGE_PERFORMANCE, false)));
  p->lpf_cutoff    = ClampUnit(level2_   .committed(Slot(PAGE_PERFORMANCE, false)) + l2_cv);

  // calibrated v/oct -> semitones, piggybacked on reserved_a
  p->reserved_a = settings_
      ? settings_->calibration().Transform(l1_cv_raw)
      : (0.5f - l1_cv_raw) * 120.0f;

  // PERF shifted
  p->chord_mode       = algorithm_.committed(Slot(PAGE_PERFORMANCE, true));
  p->pulse_gain       = timbre_   .committed(Slot(PAGE_PERFORMANCE, true));
  p->pitch            = level1_   .committed(Slot(PAGE_PERFORMANCE, true));
  p->filter_resonance = level2_   .committed(Slot(PAGE_PERFORMANCE, true));

  // KARPLUS unshifted: damping | white/pink | ks lpf | noise floor
  // shifted ALGO = modal mix; other shifted slots inert (modal params
  // hardcoded in Drone::Init)
  p->damping          = algorithm_.committed(Slot(PAGE_KARPLUS,     false));
  p->white_pink_mix   = timbre_   .committed(Slot(PAGE_KARPLUS,     false));
  p->karplus_lpf      = level1_   .committed(Slot(PAGE_KARPLUS,     false));
  p->noise_floor_base = level2_   .committed(Slot(PAGE_KARPLUS,     false));
  p->harmonics        = algorithm_.committed(Slot(PAGE_KARPLUS,     true));

  // REVERB:  size | diffusion | lp | amount  (unshifted)
  //          predelay | smear | drive | shim_rate  (shifted)
  p->reverb_size      = algorithm_.committed(Slot(PAGE_REVERB,      false));
  p->reverb_diffusion = timbre_   .committed(Slot(PAGE_REVERB,      false));
  p->reverb_lp        = level1_   .committed(Slot(PAGE_REVERB,      false));
  p->reverb_amount    = level2_   .committed(Slot(PAGE_REVERB,      false));
  p->reverb_predelay  = algorithm_.committed(Slot(PAGE_REVERB,      true));
  p->smear            = timbre_   .committed(Slot(PAGE_REVERB,      true));
  p->reverb_drive     = level1_   .committed(Slot(PAGE_REVERB,      true));
  p->reverb_shim_rate = level2_   .committed(Slot(PAGE_REVERB,      true));

  // OVERDRIVE unshifted: drive | warmth | tone | vinyl
  // shifted: only lvl2 used (bias). BIG/SMALL/LVL1 shifted reserved
  p->distortion        = algorithm_.committed(Slot(PAGE_OVERDRIVE, false));
  p->distortion_warmth = timbre_   .committed(Slot(PAGE_OVERDRIVE, false));
  p->distortion_tone   = level1_   .committed(Slot(PAGE_OVERDRIVE, false));
  p->vinyl_noise       = level2_   .committed(Slot(PAGE_OVERDRIVE, false));
  p->distortion_bias   = level2_   .committed(Slot(PAGE_OVERDRIVE, true));

  adc_.Convert();
}

void CvScaler::MaybeSave() {
  switch (save_state_) {
    case SAVE_IDLE: {
      // first settled-dirty slot, one per pass
      int found = -1;
      for (int i = 0; i < kNumSlots; ++i) {
        if (slot_dirty_[i] && slot_idle_blocks_[i] >= kKnobSettleBlocks) {
          found = i;
          break;
        }
      }
      if (found < 0) return;

      // sector wrap -> erase + re-dirty all. 1.1s blocking erase
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
        return;
      }

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
  // stub - audio inputs handled in Drone
}

}  // namespace warps_drone
