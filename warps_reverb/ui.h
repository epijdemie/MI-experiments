// button + led state machine.
// hold or knob-move while pressed -> shift layer.
// short tap reserved for future utility (freeze toggle).
// main rgb: dim white when shifted, red on clip.

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

 private:
  void UpdateLeds();

  // poll @ ~1.5kHz. ~1s hold tolerance
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

  bool      shifted_;

  DISALLOW_COPY_AND_ASSIGN(Ui);
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_UI_H_
