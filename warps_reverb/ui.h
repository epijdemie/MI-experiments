// Button-gesture and LED state machine.
//
// Gestures (mode-independent):
//   - Short tap -> cycle to next mode (with wraparound)
//   - Hold      -> shift layer for knobs
//
// LEDs:
//   - Main RGB (under big knob): solid white while shift held, red flash on
//     output clipping, off otherwise.
//   - OSC bicolor (next to button): steady color = current mode.
//       mode 0: off            mode 2: red
//       mode 1: green          mode 3: orange (red + green)
//
// Pressing the button cancels any "tap pending" decision - only on release
// after the multi-tap window do we commit. A press that held longer than
// kHoldTicks or moved knobs is treated as a shift gesture, not a tap.

#ifndef WARPS_REVERB_UI_H_
#define WARPS_REVERB_UI_H_

#include "stmlib/stmlib.h"

#include "warps/drivers/leds.h"
#include "warps/drivers/switches.h"

#include "warps_reverb/cv_scaler.h"
#include "warps_reverb/dsp/parameters.h"
#include "warps_reverb/modes.h"

namespace warps_reverb {

class Reverb;  // forward - defined in dsp/reverb.h

class Ui {
 public:
  Ui() { }
  ~Ui() { }

  void Init(CvScaler* cv_scaler, Reverb* reverb,
            ReverbParameters* parameters);

  void Poll();

  inline bool shifted() const { return shifted_; }
  inline FirmwareMode mode() const { return mode_; }
  inline const ModeConfig& mode_config() const { return ModeFor(mode_); }

 private:
  void UpdateLeds();
  void EnterMode(FirmwareMode m);

  // Poll runs at ~1.5 kHz (one call per audio block).
  // ~1 s tolerance - naturally slow taps (200–400 ms is normal) still register
  // as taps. Only deliberate long presses or knob-moving gestures count as
  // shift edits. The previous 130 ms threshold was way too aggressive.
  static constexpr uint16_t kHoldTicks      = 1500;

  // Output peak above which we flash the main LED red.
  static constexpr float    kClipThreshold  = 0.95f;

  warps::Switches   switches_;
  warps::Leds       leds_;
  CvScaler*         cv_scaler_;
  Reverb*           reverb_;
  ReverbParameters* parameters_;

  bool      prev_pressed_;
  uint16_t  press_ticks_;
  bool      moved_during_press_;

  bool          shifted_;
  FirmwareMode  mode_;

  DISALLOW_COPY_AND_ASSIGN(Ui);
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_UI_H_
