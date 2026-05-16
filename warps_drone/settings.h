// V/oct calibration persisted in flash, in the Émilie style.
//
// Two-point calibration ritual: user patches +1 V (C1) and +3 V (C3) and
// presses the button at each step. We solve for scale and offset of the
// linear map  semitones = scale * adc_raw + offset.
//
// Defaults match the original hardcoded mapping  semitones = (0.5 - adc) * 120
// so an uncalibrated module behaves identically to before. After a successful
// calibration, the new values are written to flash sector 2 (16 KB, free) and
// reloaded on every boot.

#ifndef WARPS_DRONE_SETTINGS_H_
#define WARPS_DRONE_SETTINGS_H_

#include "stmlib/stmlib.h"
#include "stmlib/system/storage.h"

namespace warps_drone {

struct CalibrationData {
  // V/oct CV transform: semitones = voct_scale * adc + voct_offset.
  // Defaults reproduce the pre-calibration formula (0.5 - adc) * 120.
  float voct_scale;
  float voct_offset;

  inline float Transform(float adc_raw) const {
    return voct_scale * adc_raw + voct_offset;
  }
};

class Settings {
 public:
  Settings() { }
  ~Settings() { }

  void Init() {
    if (!Storage::Load(&calibration_)) {
      // No valid record yet (fresh flash or checksum mismatch). Use the
      // pre-calibration defaults so the module still produces sane V/oct.
      calibration_.voct_scale  = -120.0f;
      calibration_.voct_offset =  60.0f;
    }
  }

  void Save() {
    Storage::Save(calibration_);
  }

  inline const CalibrationData& calibration() const { return calibration_; }
  inline CalibrationData* mutable_calibration()     { return &calibration_; }

 private:
  // Sector 2 (0x08008000, 16 KB) - not touched by code (text sits in 0..1)
  // and not touched by the slot journal (which lives in sector 11).
  typedef stmlib::Storage<2> Storage;

  CalibrationData calibration_;

  DISALLOW_COPY_AND_ASSIGN(Settings);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_SETTINGS_H_
