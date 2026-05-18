#include "warps_drone/ui.h"

#include "warps_drone/cv_scaler.h"
#include "warps_drone/dsp/drone.h"
#include "warps_drone/dsp/voice_bank.h"
#include "warps_drone/settings.h"

namespace warps_drone {

namespace {

inline float Absf(float a) { return a < 0.0f ? -a : a; }

}  // namespace

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

  // Boot-hold belongs to the bootloader now (Émilie's QPSK WAV loader
  // at 0x08000000). By the time this app runs, the user has either
  // released or is mid-update inside the bootloader. Always boot into
  // normal play.
  for (int i = 0; i < 16; ++i) switches_.Debounce();
  mode_              = UI_MODE_NORMAL;
  prev_pressed_      = switches_.pressed(0);
  wait_for_release_  = false;
  cal_arm_blocked_   = false;
}

void Ui::Poll() {
  switches_.Debounce();

  const bool pressed      = switches_.pressed(0);
  const bool press_edge   = pressed && !prev_pressed_;
  const bool release_edge = !pressed && prev_pressed_;

  if (mode_ == UI_MODE_NORMAL) {
    if (press_edge) {
      press_ticks_     = 0;
      shifted_         = true;
      cal_arm_blocked_ = false;
      // Snapshot pot positions so the cal-entry watcher can detect any
      // movement during the hold and abort cleanly.
      pot_at_press_[0] = cv_scaler_->pot_big();
      pot_at_press_[1] = cv_scaler_->pot_small();
      pot_at_press_[2] = cv_scaler_->pot_lvl1();
      pot_at_press_[3] = cv_scaler_->pot_lvl2();
    }
    if (pressed && press_ticks_ < UINT16_MAX) ++press_ticks_;

    // Cal-entry watcher. Active only on OVERDRIVE page; aborts on
    // any pot drift > threshold; fires at kCalEnterTicks.
    if (pressed && page_ == PAGE_OVERDRIVE && !cal_arm_blocked_) {
      const float d0 = Absf(cv_scaler_->pot_big()   - pot_at_press_[0]);
      const float d1 = Absf(cv_scaler_->pot_small() - pot_at_press_[1]);
      const float d2 = Absf(cv_scaler_->pot_lvl1()  - pot_at_press_[2]);
      const float d3 = Absf(cv_scaler_->pot_lvl2()  - pot_at_press_[3]);
      if (d0 > kCalPotIdleThreshold || d1 > kCalPotIdleThreshold ||
          d2 > kCalPotIdleThreshold || d3 > kCalPotIdleThreshold) {
        cal_arm_blocked_ = true;
      } else if (press_ticks_ >= kCalEnterTicks) {
        mode_             = UI_MODE_CALIBRATION_C1;
        wait_for_release_ = true;     // swallow the rest of this hold
        shifted_          = false;
        press_ticks_      = 0;
      }
    }

    if (release_edge) {
      shifted_ = false;
      // short release = tap -> page cycle
      if (press_ticks_ < kTapWindow) {
        page_ = static_cast<ControlPage>((page_ + 1) % PAGE_COUNT);
      }
      press_ticks_ = 0;
    }
  } else if (mode_ == UI_MODE_CALIBRATION_C1      ||
             mode_ == UI_MODE_CALIBRATION_C3      ||
             mode_ == UI_MODE_CALIBRATION_C1_BASS ||
             mode_ == UI_MODE_CALIBRATION_C3_BASS) {
    if (wait_for_release_) {
      if (release_edge) wait_for_release_ = false;
    } else if (press_edge) {
      HandleCalibrationButton();
    }
  } else if (mode_ == UI_MODE_CALIBRATION_HANDOVER) {
    if (status_ticks_ > 0) {
      --status_ticks_;
    } else {
      mode_              = UI_MODE_CALIBRATION_C1_BASS;
      wait_for_release_  = false;
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
  // 4-point cal: chord (LVL1) then bass (PARAM). Plaits/Rings-style
  // 2-point per input — capture +1 V (green blink), capture +3 V
  // (yellow blink), advance. Between chord and bass legs, the
  // HANDOVER mode pulses both indicators ~1 s as the visual cue that
  // the second input is up next.
  //   chord leg uses OSC LED green/yellow.
  //   bass  leg uses main RGB green/yellow (the "other" indicator).
  // semitones = scale * adc + offset, anchored so adc_at_C1 -> +12.
  switch (mode_) {
    case UI_MODE_CALIBRATION_C1: {
      voct_c1_capture_ = cv_scaler_->voct_chord_raw();
      mode_ = UI_MODE_CALIBRATION_C3;
      break;
    }
    case UI_MODE_CALIBRATION_C3: {
      const float c3    = cv_scaler_->voct_chord_raw();
      const float delta = c3 - voct_c1_capture_;
      if (delta > -0.5f && delta < -0.05f) {
        // Stash the chord cal — committed only after bass succeeds too,
        // so a fumbled bass capture doesn't clobber a working chord cal.
        chord_scale_pending_  = 24.0f / delta;
        chord_offset_pending_ = 12.0f -
            chord_scale_pending_ * voct_c1_capture_;
        mode_              = UI_MODE_CALIBRATION_HANDOVER;
        status_ticks_      = kCalibrationHandoverTicks;
        wait_for_release_  = true;   // swallow the rest of the C3 press
      } else {
        mode_         = UI_MODE_CALIBRATION_ERROR;
        status_ticks_ = kCalibrationStatusTicks;
      }
      break;
    }
    case UI_MODE_CALIBRATION_C1_BASS: {
      voct_c1_capture_bass_ = cv_scaler_->voct_bass_raw();
      mode_ = UI_MODE_CALIBRATION_C3_BASS;
      break;
    }
    case UI_MODE_CALIBRATION_C3_BASS: {
      const float c3    = cv_scaler_->voct_bass_raw();
      const float delta = c3 - voct_c1_capture_bass_;
      if (delta > -0.5f && delta < -0.05f) {
        // Both legs good — commit both cal pairs and save.
        CalibrationData* c_chord = settings_->mutable_calibration_chord();
        CalibrationData* c_bass  = settings_->mutable_calibration_bass();
        c_chord->voct_scale  = chord_scale_pending_;
        c_chord->voct_offset = chord_offset_pending_;
        c_bass ->voct_scale  = 24.0f / delta;
        c_bass ->voct_offset = 12.0f -
            c_bass->voct_scale * voct_c1_capture_bass_;
        settings_->Save();
        mode_ = UI_MODE_CALIBRATION_OK;
      } else {
        mode_ = UI_MODE_CALIBRATION_ERROR;
      }
      status_ticks_ = kCalibrationStatusTicks;
      break;
    }
    default:
      break;
  }
}

void Ui::UpdateLeds() {
  ++led_tick_;
  if (trigger_flash_ > 0) --trigger_flash_;

  // Cal modes hijack both LEDs — chord leg lives on the OSC LED, bass
  // leg on the main RGB. Early-return so the normal page/chord/clip
  // colour logic doesn't fight the cal indicator.
  if (mode_ != UI_MODE_NORMAL) {
    uint8_t main_r = 0, main_g = 0, main_b = 0;
    uint8_t osc_r = 0, osc_g = 0;
    const bool slow_blink = (led_tick_ & 0x100) != 0;   // ~3 Hz
    switch (mode_) {
      case UI_MODE_CALIBRATION_C1:
        // chord leg, +1 V — OSC slow green blink
        if (slow_blink) osc_g = 255;
        break;
      case UI_MODE_CALIBRATION_C3:
        // chord leg, +3 V — OSC slow yellow blink
        if (slow_blink) { osc_r = 255; osc_g = 220; }
        break;
      case UI_MODE_CALIBRATION_HANDOVER: {
        // alternating blink — OSC yellow ↔ main green, ~3 Hz
        if (slow_blink) {
          osc_r = 255; osc_g = 220;
        } else {
          main_g = 255;
        }
        break;
      }
      case UI_MODE_CALIBRATION_C1_BASS:
        // bass leg, +1 V — main RGB slow green blink
        if (slow_blink) main_g = 255;
        break;
      case UI_MODE_CALIBRATION_C3_BASS:
        // bass leg, +3 V — main RGB slow yellow blink
        if (slow_blink) { main_r = 255; main_g = 220; }
        break;
      case UI_MODE_CALIBRATION_OK:
        osc_g = 255;
        main_g = 255;
        break;
      case UI_MODE_CALIBRATION_ERROR:
        osc_r = 255;
        main_r = 255;
        break;
      default:
        break;
    }
    leds_.set_main(main_r, main_g, main_b);
    leds_.set_osc(osc_r, osc_g);
    leds_.Write();
    return;
  }

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

  // OSC bicolor — page indicator + cal-entry warning. (Cal-mode LED
  // hijack is handled in the early-return above; we only reach this
  // point in UI_MODE_NORMAL.)
  uint8_t osc_r = 0, osc_g = 0;
  {
      // Cal-entry warning pulse: while on OVERDRIVE + holding + no pot
      // moved, between 8 s and 10 s, blink yellow fast so the user has
      // a chance to abort by releasing before cal kicks in.
      const bool cal_warning = page_ == PAGE_OVERDRIVE
                            && switches_.pressed(0)
                            && !cal_arm_blocked_
                            && press_ticks_ >= kCalWarnTicks;
      if (cal_warning) {
        const bool blink = (led_tick_ & 0x40) != 0;   // ~5 Hz
        if (blink) { osc_r = 200; osc_g = 200; }
      } else {
        // breathing for dimmed (bypassed) state - triangle, ~2.7s,
        // 1..5%
        const uint32_t bph    = led_tick_ & 0xFFF;
        const uint32_t bhalf  = bph < 0x800 ? bph : 0xFFF - bph;
        const float    breath = static_cast<float>(bhalf) * (1.0f / 2048.0f);
        const uint8_t  br_r   = 3 + static_cast<uint8_t>(10.0f * breath);
        const uint8_t  br_g   = 2 + static_cast<uint8_t>(9.0f * breath);
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
      }
  }
  leds_.set_osc(osc_r, osc_g);
  leds_.Write();
}

}  // namespace warps_drone
