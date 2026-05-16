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

  // boot-hold -> cal. settle debounce first so first Poll() doesn't see
  // a bogus release+press
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
      shifted_     = true;
    }
    if (pressed && press_ticks_ < UINT16_MAX) ++press_ticks_;

    if (release_edge) {
      shifted_ = false;
      // short release = tap -> page cycle
      if (press_ticks_ < kTapWindow) {
        page_ = static_cast<ControlPage>((page_ + 1) % PAGE_COUNT);
      }
      press_ticks_ = 0;
    }
  } else if (mode_ == UI_MODE_CALIBRATION_C1 ||
             mode_ == UI_MODE_CALIBRATION_C3) {
    if (wait_for_release_) {
      // release of boot-hold also snapshots the unpatched v/oct bias
      if (release_edge) {
        wait_for_release_  = false;
        voct_bias_capture_ = cv_scaler_->voct_raw();
      }
    } else if (press_edge) {
      HandleCalibrationButton();
    }
  } else {
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
    const float c3 = voct;
    const float delta = c3 - voct_c1_capture_;
    // adc falls as pitch rises; 2 oct between C1/C3 -> delta ≈ -0.2
    if (delta > -0.5f && delta < -0.05f) {
      CalibrationData* c = settings_->mutable_calibration();
      c->voct_scale  = 24.0f / delta;
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

  // main rgb: chord colour (shift override)
  uint8_t r = 0, g = 0, b = 0;
  if (peak_ > kClipThreshold) {
    r = 255;
  } else if (shifted_ && page_ == PAGE_PERFORMANCE) {
    // shift on PERF -> BIG = chord bank, show bank colour
    switch (drone_->bank()) {
      case BANK_DETUNE:                           r = g = b = 255;   break; // white
      case BANK_MINOR:                            b = 255;           break; // blue
      case BANK_MAJOR:                            g = 255;           break; // green
      case BANK_DOM7:                             r = 255; g = 160;  break; // amber
      case BANK_DRONE:                            g = 255; b = 255;  break; // cyan
      case BANK_WEIRD:                            r = 220; b = 255;  break; // violet
      default:                                    r = g = b = 255;   break;
    }
  } else if (shifted_) {
    r = g = b = 255;
  } else {
    // unshifted BIG. In chord banks: density zone hue, brightness ramp.
    // In DETUNE bank: bank colour (white) with brightness ramp
    const float c = parameters_->chord;
    const uint8_t val = 140 + static_cast<uint8_t>(c * 115.0f);
    if (drone_->bank() == BANK_DETUNE) {
      r = g = b = val;                                                       // white
    } else {
      switch (drone_->density_zone()) {
        case DENSITY_POWER: r = g = b = val;                       break;    // white
        case DENSITY_TRIAD: g = val;                               break;    // green
        case DENSITY_7TH:   r = val;          g = (val * 7) / 10;  break;    // amber
        case DENSITY_9TH:   r = val;          g = (val * 4) / 10;  break;    // orange
        case DENSITY_EXT:   r = val;                               break;    // red
        default:            r = g = b = val;                       break;
      }
    }
  }
  leds_.set_main(r, g, b);

  // osc bicolor: page indicator. cal overrides - green blink C1, yellow C3,
  // solid green ok, solid red err
  uint8_t osc_r = 0, osc_g = 0;
  switch (mode_) {
    case UI_MODE_CALIBRATION_C1: {
      const bool blink = (led_tick_ & 0x100) != 0;
      if (blink) osc_g = 255;
      break;
    }
    case UI_MODE_CALIBRATION_C3: {
      const bool blink = (led_tick_ & 0x100) != 0;
      if (blink) { osc_r = 255; osc_g = 220; }
      break;
    }
    case UI_MODE_CALIBRATION_OK:
      osc_g = 255;
      break;
    case UI_MODE_CALIBRATION_ERROR:
      osc_r = 255;
      break;
    default: {
      // breathing for dimmed (bypassed) state - triangle, ~2.7s period,
      // 5..20%
      const uint32_t bph    = led_tick_ & 0xFFF;
      const uint32_t bhalf  = bph < 0x800 ? bph : 0xFFF - bph;
      const float    breath = static_cast<float>(bhalf) * (1.0f / 2048.0f);
      const uint8_t  br_r   = 13 + static_cast<uint8_t>(38.0f * breath);
      const uint8_t  br_g   = 11 + static_cast<uint8_t>(33.0f * breath);
      switch (page_) {
        case PAGE_PERFORMANCE:                          break;
        case PAGE_KARPLUS:    osc_g = 255;              break;
        case PAGE_REVERB: {
          const bool active = parameters_->reverb_amount > 0.005f;
          osc_r = active ? 255 : br_r;
          osc_g = active ? 220 : br_g;
          break;
        }
        case PAGE_OVERDRIVE:
          osc_r = parameters_->distortion > 0.005f ? 255 : br_r;
          break;
        default:                                        break;
      }
      break;
    }
  }
  leds_.set_osc(osc_r, osc_g);
  leds_.Write();
}

}  // namespace warps_drone
