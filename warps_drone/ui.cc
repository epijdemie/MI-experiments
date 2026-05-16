#include "warps_drone/ui.h"

#include "warps_drone/dsp/drone.h"
#include "warps_drone/dsp/voice_bank.h"

namespace warps_drone {

void Ui::Init(Drone* drone, DroneParameters* parameters) {
  switches_.Init();
  leds_.Init();
  drone_      = drone;
  parameters_ = parameters;

  prev_pressed_ = false;
  press_ticks_  = 0;
  shifted_      = false;
}

void Ui::Poll() {
  switches_.Debounce();

  const bool pressed      = switches_.pressed(0);
  const bool press_edge   = pressed && !prev_pressed_;
  const bool release_edge = !pressed && prev_pressed_;

  if (press_edge) {
    press_ticks_ = 0;
    shifted_     = true;  // engage shift immediately on press
  }
  if (pressed && press_ticks_ < UINT16_MAX) ++press_ticks_;

  if (release_edge) {
    shifted_ = false;
    if (press_ticks_ < kHoldTicks) {
      // Short tap -> re-pluck. Excite the KS voices for ~20 ms.
      drone_->Pluck();
    }
    press_ticks_ = 0;
  }

  prev_pressed_ = pressed;
  UpdateLeds();
}

void Ui::UpdateLeds() {
  ++led_tick_;
  if (trigger_flash_ > 0) --trigger_flash_;

  uint8_t r = 0, g = 0, b = 0;
  if (peak_ > kClipThreshold) {
    r = 255;          // red overrides everything on clip
  } else if (shifted_) {
    // Shift held - knobs are on the secondary layer. Solid white so the
    // user has a clear "shift active" signal.
    r = g = b = 255;
  } else {
    // Default view: show the current chord zone in colour, since
    // voicing is now on the unshifted ALGORITHM knob.
    const VoicingZone z = drone_->voicing_zone();
    const bool blink_on = (led_tick_ / 450) & 1;   // ~300 ms blink cadence
    switch (z) {
      case VOICING_DETUNE_UNISON: if (blink_on) { r = g = b = 255; } break;
      case VOICING_OCTAVES:                       r = g = b = 255;   break;
      case VOICING_OPEN_FIFTH:                    r = 255; g = 180;  break; // yellow
      case VOICING_MINOR:                         b = 255;           break; // blue
      case VOICING_MIN7:                          r = 120; b = 255;  break; // violet (cool variant)
      case VOICING_SUS4:                          g = 200; b = 255;  break; // cyan
      case VOICING_MAJOR:                         g = 255;           break; // green
      case VOICING_MAJ7:                          r = 255; g = 80;   break; // orange (warm variant)
      default:                                    r = g = b = 255;   break;
    }
  }
  leds_.set_main(r, g, b);

  // OSC LED: steady green, flashes bright orange (red+green) for ~50 ms
  // each time a trigger arrives on LEVEL_1_CV - so you can verify
  // the trigger wiring without listening.
  if (trigger_flash_ > 0) {
    leds_.set_osc(255, 255);
  } else {
    leds_.set_osc(0, 255);
  }
  leds_.Write();
}

}  // namespace warps_drone
