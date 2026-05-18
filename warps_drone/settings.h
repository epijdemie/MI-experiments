// v/oct cal - 2-point (C2, C4). semitones = scale * adc + offset.
// flash sector 5 (0x08020000, 128 KB).
//   - moved out of sector 2 to make room for the app under
//     APPLICATION_LARGE = TRUE (app start at 0x08008000 = sector 2).
//   - sector 11 (0x080E0000) hosts the cv_scaler journal.

#ifndef WARPS_DRONE_SETTINGS_H_
#define WARPS_DRONE_SETTINGS_H_

#include "stmlib/stmlib.h"
#include "stmlib/system/storage.h"

namespace warps_drone {

struct CalibrationData {
  float voct_scale;
  float voct_offset;

  inline float Transform(float adc_raw) const {
    return voct_scale * adc_raw + voct_offset;
  }
};

struct CalibrationSet {
  CalibrationData chord;   // LVL1 CV
  CalibrationData bass;    // PARAM CV
};

class Settings {
 public:
  Settings() { }
  ~Settings() { }

  void Init() {
    if (!Storage::Load(&calibration_)) {
      // Pre-cal defaults — lifted from the developer's calibrated unit.
      // Both inputs default to the same scale/offset; the input circuits
      // are nominally identical, so this gets the user close enough to
      // play before they walk the cal ritual.
      calibration_.chord.voct_scale  = -109.44f;
      calibration_.chord.voct_offset =  100.32f;
      calibration_.bass .voct_scale  = -109.44f;
      calibration_.bass .voct_offset =  100.32f;
    }
  }

  void Save() {
    Storage::Save(calibration_);
  }

  inline const CalibrationData& calibration_chord() const { return calibration_.chord; }
  inline const CalibrationData& calibration_bass()  const { return calibration_.bass;  }
  inline CalibrationData* mutable_calibration_chord() { return &calibration_.chord; }
  inline CalibrationData* mutable_calibration_bass()  { return &calibration_.bass;  }

 private:
  typedef stmlib::Storage<5> Storage;

  CalibrationSet calibration_;

  DISALLOW_COPY_AND_ASSIGN(Settings);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_SETTINGS_H_
