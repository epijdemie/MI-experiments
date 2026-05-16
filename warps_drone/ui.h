// button: tap -> next page, hold -> shift, boot-hold -> v/oct cal.
// osc led = page indicator. main rgb = chord / shift / clip / cal status

#ifndef WARPS_DRONE_UI_H_
#define WARPS_DRONE_UI_H_

#include "stmlib/stmlib.h"

#include "warps/drivers/leds.h"
#include "warps/drivers/switches.h"

#include "warps_drone/dsp/parameters.h"

namespace warps_drone {

class Drone;
class CvScaler;
class Settings;

enum UiMode {
  UI_MODE_NORMAL = 0,
  UI_MODE_CALIBRATION_C1,    // +1V capture
  UI_MODE_CALIBRATION_C3,    // +3V capture
  UI_MODE_CALIBRATION_OK,
  UI_MODE_CALIBRATION_ERROR,
};

class Ui {
 public:
  Ui() { }

  void Init(Drone* drone, DroneParameters* parameters,
            CvScaler* cv_scaler, Settings* settings);
  void Poll();

  inline ControlPage page()    const { return page_; }
  inline bool        shifted() const { return shifted_; }

  inline void set_peak       (float p) { peak_       = p; }
  inline void notify_trigger ()        { trigger_flash_ = kFlashTicks; }

 private:
  void UpdateLeds();
  void HandleCalibrationButton();
  void FinishCalibration();

  // poll @ ~1.5kHz (1 / audio block, 32 samp @ 48k)
  static constexpr uint16_t kTapWindow              = 300;   // ~200ms
  static constexpr uint16_t kFlashTicks             = 220;   // ~150ms
  static constexpr uint16_t kCalibrationStatusTicks = 2200;  // ~1.5s
  static constexpr float    kClipThreshold = 0.95f;

  warps::Switches  switches_;
  warps::Leds      leds_;
  Drone*           drone_;
  DroneParameters* parameters_;
  CvScaler*        cv_scaler_;
  Settings*        settings_;

  bool         prev_pressed_;
  uint16_t     press_ticks_;
  bool         shifted_;
  ControlPage  page_;
  uint32_t     led_tick_      = 0;
  uint16_t     trigger_flash_ = 0;
  float        peak_          = 0.0f;

  UiMode       mode_                = UI_MODE_NORMAL;
  uint16_t     status_ticks_        = 0;
  float        voct_bias_capture_   = 0.0f;
  float        voct_c1_capture_     = 0.0f;
  // boot-hold cal: ignore presses until physical release; that release
  // also captures the unpatched v/oct bias as 0V anchor (PITCH must be
  // unpatched at boot)
  bool         wait_for_release_    = false;

  DISALLOW_COPY_AND_ASSIGN(Ui);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_UI_H_
