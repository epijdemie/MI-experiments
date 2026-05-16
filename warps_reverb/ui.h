// button + led state machine.
// tap -> next mode (wrap). hold or knob-move -> shift.
// main rgb: white shifted, red on clip.
// osc bicolor: mode (0 off, 1 green, 2 red, 3 orange)

#ifndef WARPS_REVERB_UI_H_
#define WARPS_REVERB_UI_H_

#include "stmlib/stmlib.h"

#include "warps/drivers/leds.h"
#include "warps/drivers/switches.h"

#include "warps_reverb/cv_scaler.h"
#include "warps_reverb/dsp/parameters.h"
#include "warps_reverb/modes.h"

namespace warps_reverb {

class Reverb;

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

  // poll @ ~1.5kHz. ~1s tap tolerance
  static constexpr uint16_t kHoldTicks      = 1500;
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
