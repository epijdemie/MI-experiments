// UI for the page-based drone firmware.
//
// Button gestures:
//   single tap (short press+release)  -> cycle to next page
//   hold (press held > kTapWindow)    -> shift layer (per page)
//   held during boot                  -> enter V/oct calibration
//   pluck via IN R audio (separate)   -> re-pluck (handled in Drone)
//
// LEDs:
//   OSC bicolor       - page indicator (off / green / orange / red).
//                       Briefly punches orange on each detected trigger.
//   Main RGB knob LED - chord color (current voicing zone). Solid
//                       white while shift held. Red flash on output clip.
//                       Blinking green/yellow during calibration steps.

#ifndef WARPS_DRONE_UI_H_
#define WARPS_DRONE_UI_H_

#include "stmlib/stmlib.h"

#include "warps/drivers/leds.h"
#include "warps/drivers/switches.h"

#include "warps_drone/dsp/parameters.h"

namespace warps_drone {

class Drone;      // forward - dsp/drone.h
class CvScaler;   // forward - cv_scaler.h
class Settings;   // forward - settings.h

enum UiMode {
  UI_MODE_NORMAL = 0,
  UI_MODE_CALIBRATION_C1,    // capture +1 V reference
  UI_MODE_CALIBRATION_C3,    // capture +3 V reference
  UI_MODE_CALIBRATION_OK,    // brief solid-green success indication
  UI_MODE_CALIBRATION_ERROR, // brief solid-red error indication
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
  void HandleCalibrationButton();   // process a press_edge while calibrating
  void FinishCalibration();         // compute scale/offset, validate, save

  // Poll runs ~1.5 kHz (one call per audio block, 32 samples @ 48 kHz).
  // Tap window ~200 ms - anything shorter and held release reads as a
  // tap, anything longer reads as a hold (shift gesture).
  static constexpr uint16_t kTapWindow             = 300;
  static constexpr uint16_t kFlashTicks            = 220;    // ~150 ms
  // ~1.5 s post-calibration LED splash before returning to normal.
  static constexpr uint16_t kCalibrationStatusTicks = 2200;
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
  // When entering calibration via boot-hold, you is still holding
  // the button when the first Poll() runs. We have to wait for them to
  // physically release before treating the next press as a capture
  // event - otherwise the boot-hold itself counts as the C1 press.
  // The release_edge that ends the boot-hold also auto-captures the
  // unpatched V/oct bias used as the "0 V" anchor for the offset, so
  // you must have NO cable patched to PITCH at this moment.
  bool         wait_for_release_    = false;

  DISALLOW_COPY_AND_ASSIGN(Ui);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_UI_H_
