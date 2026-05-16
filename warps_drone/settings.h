// v/oct cal - 2-point (C1, C3). semitones = scale * adc + offset.
// flash sector 2

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

class Settings {
 public:
  Settings() { }
  ~Settings() { }

  void Init() {
    if (!Storage::Load(&calibration_)) {
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
  typedef stmlib::Storage<2> Storage;

  CalibrationData calibration_;

  DISALLOW_COPY_AND_ASSIGN(Settings);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_SETTINGS_H_
