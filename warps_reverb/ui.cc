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

  // Boot into mode 0. EnterMode pushes the mode's defaults into the live
  // parameters and updates the reverb's DSP overrides.
  EnterMode(MODE_REVERB);
}

void Ui::EnterMode(FirmwareMode m) {
  mode_ = m;
  const ModeConfig& cfg = ModeFor(m);
  *parameters_ = cfg.defaults;
  reverb_->set_dsp_overrides(cfg.dsp);
  cv_scaler_->HandleModeChange(cfg);
}

void Ui::Poll() {
  switches_.Debounce();

  const bool pressed = switches_.pressed(0);
  const bool press_edge   = pressed && !prev_pressed_;
  const bool release_edge = !pressed && prev_pressed_;

  if (press_edge) {
    press_ticks_ = 0;
    moved_during_press_ = false;
    shifted_ = true;
    // Discard any movement flag accumulated BEFORE this press - knob
    // fiddling between mode-switch taps used to leak into moved_during_press_
    // and make taps register as shift gestures (mode change failed).
    cv_scaler_->TakeMovementFlag();
  } else if (pressed && cv_scaler_->TakeMovementFlag()) {
    moved_during_press_ = true;
  }

  if (pressed) {
    if (press_ticks_ < UINT16_MAX) ++press_ticks_;
  } else {
    shifted_ = false;
  }

  if (release_edge) {
    const bool was_hold =
        press_ticks_ >= kHoldTicks || moved_during_press_;
    if (!was_hold) {
      // Short tap -> cycle to next mode.
      const FirmwareMode next =
          static_cast<FirmwareMode>((mode_ + 1) % MODE_COUNT);
      EnterMode(next);
    }
  }

  prev_pressed_ = pressed;
  UpdateLeds();
}

void Ui::UpdateLeds() {
  const ModeConfig& cfg = ModeFor(mode_);

  // ---- Main RGB LED ----
  uint8_t r = 0, g = 0, b = 0;
  if (shifted_) {
    r = g = b = 255;                                     // white while shifted
  } else if (cfg.dsp.show_clip_warning &&
             reverb_->peak() > kClipThreshold) {
    r = 255;                                             // red on clip
  }
  leds_.set_main(r, g, b);

  // ---- OSC bicolor LED - current mode ----
  leds_.set_osc(cfg.osc_red, cfg.osc_green);

  leds_.Write();
}

}  // namespace warps_reverb
