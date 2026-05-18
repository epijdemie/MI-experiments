// button: tap -> next page, hold -> shift. v/oct cal entry: hold the
// button for 10 s while on the OVERDRIVE page WITHOUT moving any pot.
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
  UI_MODE_CALIBRATION_C1,         // +1V capture, chord input (LVL1)
  UI_MODE_CALIBRATION_C3,         // +3V capture, chord input
  UI_MODE_CALIBRATION_HANDOVER,   // brief LED handover, alt blink ~1s
  UI_MODE_CALIBRATION_C1_BASS,    // +1V capture, bass input (PARAM)
  UI_MODE_CALIBRATION_C3_BASS,    // +3V capture, bass input
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
  static constexpr uint16_t kTapWindow              = 300;    // ~200ms
  static constexpr uint16_t kFlashTicks             = 220;    // ~150ms
  static constexpr uint16_t kCalibrationStatusTicks = 2200;   // ~1.5s
  // Cal-entry gesture: OVERDRIVE page, button hold for this long, while
  // no pot has moved by more than kCalPotIdleThreshold from press-time.
  static constexpr uint16_t kCalWarnTicks           = 12000;  // ~8s pulse warning
  static constexpr uint16_t kCalEnterTicks          = 15000;  // ~10s -> enter cal
  static constexpr float    kCalPotIdleThreshold    = 0.01f;  // 1 % of pot range
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
  float        voct_c1_capture_      = 0.0f;   // chord
  float        voct_c1_capture_bass_ = 0.0f;   // bass
  float        chord_scale_pending_  = 0.0f;
  float        chord_offset_pending_ = 0.0f;
  // ~1 s handover blink between chord cal and bass cal.
  static constexpr uint16_t kCalibrationHandoverTicks = 1500;
  // When transitioning into cal mode mid-press, swallow the rest of the
  // current hold so it doesn't immediately get read as the +1 V press.
  bool         wait_for_release_    = false;
  // Cal-entry watcher: pot positions captured at press_edge, and a
  // sticky abort flag set the moment any pot drifts more than threshold.
  float        pot_at_press_[4]     = {0.0f, 0.0f, 0.0f, 0.0f};
  bool         cal_arm_blocked_     = false;

  DISALLOW_COPY_AND_ASSIGN(Ui);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_UI_H_
