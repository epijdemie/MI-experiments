#include "warps_reverb/ui.h"

#include "warps_reverb/dsp/reverb.h"

namespace warps_reverb {

void Ui::Init(CvScaler* cv_scaler, Reverb* reverb,
              ReverbParameters* parameters) {
  switches_.Init();
  leds_.Init();
  cv_scaler_ = cv_scaler;
  reverb_ = reverb;
  parameters_ = parameters;

  prev_pressed_ = false;
  press_ticks_ = 0;
  moved_during_press_ = false;
  shifted_ = false;

  // seed from panel defaults, arm soft-takeover
  *parameters_ = kConfig.defaults;
  cv_scaler_->HandleReset();
}

void Ui::Poll() {
  switches_.Debounce();

  const bool pressed = switches_.pressed(0);
  const bool press_edge = pressed && !prev_pressed_;

  if (press_edge) {
    press_ticks_ = 0;
    moved_during_press_ = false;
    shifted_ = true;
    cv_scaler_->TakeMovementFlag();
  } else if (pressed && cv_scaler_->TakeMovementFlag()) {
    moved_during_press_ = true;
  }

  if (pressed) {
    if (press_ticks_ < UINT16_MAX) ++press_ticks_;
  } else {
    shifted_ = false;
  }

  prev_pressed_ = pressed;
  UpdateLeds();
}

void Ui::UpdateLeds() {
  uint8_t r = 0, g = 0, b = 0;
  if (shifted_) {
    // dim white - shift active
    r = g = b = 80;
  } else if (reverb_->peak() > kClipThreshold) {
    r = 255;
  }
  leds_.set_main(r, g, b);
  leds_.set_osc(0, 0);
  leds_.Write();
}

}  // namespace warps_reverb
