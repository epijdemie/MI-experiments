#include "warps_drone/ui.h"

#include "warps_drone/cv_scaler.h"
#include "warps_drone/dsp/drone.h"
#include "warps_drone/dsp/voice_bank.h"
#include "warps_drone/settings.h"

namespace warps_drone {

void Ui::Init(Drone* drone, DroneParameters* parameters,
              CvScaler* cv_scaler, Settings* settings) {
  switches_.Init();
  leds_.Init();
  drone_      = drone;
  parameters_ = parameters;
  cv_scaler_  = cv_scaler;
  settings_   = settings;

  press_ticks_  = 0;
  shifted_      = false;
  page_         = PAGE_PERFORMANCE;
  status_ticks_ = 0;

  // Boot-hold detection - if the button is physically pressed right
  // after the GPIO peripheral is up, enter calibration. Settle the
  // debounce shift register first (8 reads of "pressed" fills it to
  // 0x00 = pressed()=true) so the very first Poll() doesn't see a
  // bogus release+press from the still-settling debouncer.
  // wait_for_release_ then suppresses press_edge events until the
  // user physically lets go of the boot-hold - otherwise the held
  // button at boot would itself count as the C1 capture press.
  if (switches_.pressed_immediate(0)) {
    for (int i = 0; i < 16; ++i) switches_.Debounce();
    mode_              = UI_MODE_CALIBRATION_C1;
    prev_pressed_      = switches_.pressed(0);
    wait_for_release_  = true;
  } else {
    mode_              = UI_MODE_NORMAL;
    prev_pressed_      = false;
    wait_for_release_  = false;
  }
}

void Ui::Poll() {
  switches_.Debounce();

  const bool pressed      = switches_.pressed(0);
  const bool press_edge   = pressed && !prev_pressed_;
  const bool release_edge = !pressed && prev_pressed_;

  if (mode_ == UI_MODE_NORMAL) {
    if (press_edge) {
      press_ticks_ = 0;
      shifted_     = true;   // engage shift immediately for snappy edits
    }
    if (pressed && press_ticks_ < UINT16_MAX) ++press_ticks_;

    if (release_edge) {
      shifted_ = false;
      // Short release = tap = page cycle. Long release = end of hold.
      if (press_ticks_ < kTapWindow) {
        page_ = static_cast<ControlPage>((page_ + 1) % PAGE_COUNT);
      }
      press_ticks_ = 0;
    }
  } else if (mode_ == UI_MODE_CALIBRATION_C1 ||
             mode_ == UI_MODE_CALIBRATION_C3) {
    if (wait_for_release_) {
      // Boot-hold suppression: don't accept any captures until the
      // user has physically released the button at least once.
      // On release_edge ALSO snapshot the unpatched V/oct bias - the
      // user is expected to boot with nothing patched to PITCH, so
      // whatever the smoothed ADC reads at that instant is the "0 V"
      // anchor used to align the offset.
      if (release_edge) {
        wait_for_release_  = false;
        voct_bias_capture_ = cv_scaler_->voct_raw();
      }
    } else if (press_edge) {
      // Each press_edge captures the current V/oct ADC, advances state.
      HandleCalibrationButton();
    }
  } else {
    // OK / ERROR splash: count down then return to normal.
    if (status_ticks_ > 0) {
      --status_ticks_;
    } else {
      mode_ = UI_MODE_NORMAL;
    }
  }

  prev_pressed_ = pressed;
  UpdateLeds();
}

void Ui::HandleCalibrationButton() {
  const float voct = cv_scaler_->voct_raw();
  if (mode_ == UI_MODE_CALIBRATION_C1) {
    voct_c1_capture_ = voct;
    mode_ = UI_MODE_CALIBRATION_C3;
  } else {
    voct_c1_capture_ = voct_c1_capture_;  // (kept for clarity)
    const float c3 = voct;
    const float delta = c3 - voct_c1_capture_;
    // The Warps V/oct ADC reading falls as pitch rises (current default
    // is semitones = (0.5 - adc) * 120, i.e. scale = -120 sem / unit).
    // Two octaves between the 1 V and 3 V points should produce roughly
    // delta ≈ -0.2. Accept anything in [-0.5, -0.05]; outside that the
    // user almost certainly didn't patch the right voltages.
    if (delta > -0.5f && delta < -0.05f) {
      // Scale from the two captured voltages; offset from the bias
      // capture (taken when you released the boot-hold, before
      // patching anything) so semitones = 0 at the unpatched ADC
      // reading. Without this anchor the per-module bias point (which
      // is NOT necessarily 0.5) leaks straight into the pitch.
      CalibrationData* c = settings_->mutable_calibration();
      c->voct_scale  = 24.0f / delta;             // semitones per ADC unit
      c->voct_offset = -c->voct_scale * voct_bias_capture_;
      settings_->Save();
      mode_ = UI_MODE_CALIBRATION_OK;
    } else {
      mode_ = UI_MODE_CALIBRATION_ERROR;
    }
    status_ticks_ = kCalibrationStatusTicks;
  }
}

void Ui::UpdateLeds() {
  ++led_tick_;
  if (trigger_flash_ > 0) --trigger_flash_;

  // ----- Main RGB knob LED: chord color (with shift override). -----
  uint8_t r = 0, g = 0, b = 0;
  if (peak_ > kClipThreshold) {
    r = 255;
  } else if (shifted_ && page_ == PAGE_PERFORMANCE) {
    // Shift on PERFORMANCE -> BIG knob picks chord *mode*. Display
    // the mode colour so you can see what they're choosing.
    switch (drone_->chord_mode()) {
      case MODE_MAJOR:                            g = 255;           break;
      case MODE_MINOR:                            b = 255;           break;
      case MODE_DOM7:                             r = 255; g = 160;  break; // amber
      case MODE_DIM:                              r = 255;           break;
      case MODE_SUS2:                             g = 255; b = 255;  break; // cyan
      case MODE_SUS4:                             r = 220; b = 255;  break; // violet
      default:                                    r = g = b = 255;   break;
    }
  } else if (shifted_) {
    r = g = b = 255;     // generic shift indicator on other pages
  } else {
    // BIG knob unshifted = chord stack. Each stack zone gets its own
    // hue so the chord identity is visible without holding shift.
    // Brightness ramps with the chord knob position (140 -> 255) so
    // detune sub-position inside a zone is still visible as a swell.
    const float c = parameters_->chord;
    const uint8_t val = 140 + static_cast<uint8_t>(c * 115.0f);
    switch (drone_->voicing_zone()) {
      case STACK_UNISON: r = g = b = val;                       break; // white
      case STACK_DETUNE: g = val;          b = val;             break; // cyan
      case STACK_3RD:    g = val;                               break; // green
      case STACK_5TH:    r = val;          g = (val * 7) / 10;  break; // amber
      case STACK_7TH:    r = val;          g = (val * 4) / 10;  break; // orange
      case STACK_9TH:    r = val;                               break; // red
      default:           r = g = b = val;                       break;
    }
  }
  leds_.set_main(r, g, b);

  // ----- OSC bicolor LED -----
  // Default = page indicator. Calibration takes it over for feedback -
  // blinks green at C1, yellow at C3, solid green on success, solid red
  // on validation error.
  uint8_t osc_r = 0, osc_g = 0;
  switch (mode_) {
    case UI_MODE_CALIBRATION_C1: {
      const bool blink = (led_tick_ & 0x100) != 0;   // ~3 Hz at 1.5 kHz
      if (blink) osc_g = 255;
      break;
    }
    case UI_MODE_CALIBRATION_C3: {
      const bool blink = (led_tick_ & 0x100) != 0;
      if (blink) { osc_r = 255; osc_g = 220; }       // yellow
      break;
    }
    case UI_MODE_CALIBRATION_OK:
      osc_g = 255;
      break;
    case UI_MODE_CALIBRATION_ERROR:
      osc_r = 255;
      break;
    default:
      switch (page_) {
        case PAGE_PERFORMANCE:                          break;   // off
        case PAGE_KARPLUS:    osc_g = 255;              break;   // green
        case PAGE_REVERB:     osc_r = 255; osc_g = 220; break;   // orange
        case PAGE_SHIMMER:    osc_r = 255;              break;   // red
        default:                                        break;
      }
      break;
  }
  leds_.set_osc(osc_r, osc_g);
  leds_.Write();
}

}  // namespace warps_drone
