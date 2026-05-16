// Minimal UI for the drone firmware.
//
// Button gestures:
//   short tap   -> re-pluck all KS voices (manual transient attack)
//   hold        -> shift layer (knobs gain second function)
//
// LEDs:
//   OSC bicolor : steady green ("drone mode active")
//   main RGB    : solid white while shift held;
//                 otherwise blue pulse with the reverb tail energy;
//                 red flash on output clipping.

#ifndef WARPS_DRONE_UI_H_
#define WARPS_DRONE_UI_H_

#include "stmlib/stmlib.h"

#include "warps/drivers/leds.h"
#include "warps/drivers/switches.h"

#include "warps_drone/dsp/parameters.h"

namespace warps_drone {

class Drone;  // forward - defined in dsp/drone.h

class Ui {
 public:
  Ui() { }

  void Init(Drone* drone, DroneParameters* parameters);
  void Poll();

  inline bool shifted() const { return shifted_; }

  inline void set_peak       (float p) { peak_       = p; }
  inline void set_send_meter (float s) { send_meter_ = s; }
  inline void notify_trigger ()        { trigger_flash_ = kFlashTicks; }

 private:
  void UpdateLeds();

  // Poll runs at ~1.5 kHz (one call per audio block, 32 samples @ 48 kHz).
  // ~1 s tolerance - natural taps still register as taps; only deliberate
  // long presses count as a shift gesture.
  static constexpr uint16_t kHoldTicks     = 1500;
  static constexpr uint16_t kFlashTicks    = 220;    // ~150 ms @ 1.5 kHz Poll
  static constexpr float    kClipThreshold = 0.95f;

  warps::Switches  switches_;
  warps::Leds      leds_;
  Drone*           drone_;
  DroneParameters* parameters_;

  bool      prev_pressed_;
  uint16_t  press_ticks_;
  bool      shifted_;
  uint32_t  led_tick_ = 0;
  uint16_t  trigger_flash_ = 0;

  float peak_       = 0.0f;
  float send_meter_ = 0.0f;

  DISALLOW_COPY_AND_ASSIGN(Ui);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_UI_H_
