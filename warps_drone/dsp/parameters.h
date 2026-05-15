// Live parameter struct for the drone firmware. 32 user-facing params,
// organised by the 4 control pages × 2 layers (shift). All values
// normalised [0, 1] unless noted.

#ifndef WARPS_DRONE_DSP_PARAMETERS_H_
#define WARPS_DRONE_DSP_PARAMETERS_H_

#include "stmlib/stmlib.h"

namespace warps_drone {

enum ControlPage {
  PAGE_PERFORMANCE = 0,   // OSC LED off
  PAGE_KARPLUS,           // OSC LED green
  PAGE_REVERB,            // OSC LED orange
  PAGE_SHIMMER,           // OSC LED red
  PAGE_COUNT
};

struct DroneParameters {
  // -----------------------------------------------------------------
  // Page 0 - PERFORMANCE
  // -----------------------------------------------------------------
  float chord;            // ALGO  unshifted, +CV  - voicing 0..1 -> 8 zones
  float pulse_freq;       // PARAM unshifted, +CV  - K-S exciter pulse osc, log 4 Hz..200 Hz
  float pitch_octave;     // LVL1  unshifted, +V/oct CV - octave selector (6 zones)
  float lpf_cutoff;       // LVL2  unshifted, +CV  - LPF cutoff, log 60 Hz…12 kHz

  float chord_mode;       // ALGO  shifted        - chord quality (6 zones)
  // PARAM shifted: reserved (was HPF, moved to LVL2 shifted)
  float pitch;            // LVL1  shifted        - semitone within octave
  float hpf_cutoff;       // LVL2  shifted        - HPF cutoff, log 16 kHz..20 Hz (inverted)

  // filter_resonance is unplugged from any knob for now - stays at the
  // Init default (Butterworth, Q=0.7). Re-expose on a future slot if
  // resonance becomes useful again.
  float filter_resonance;

  // -----------------------------------------------------------------
  // Page 1 - KARPLUS  (pluck machinery removed - pulse-osc on PERF SMALL
  // is the K-S exciter now; this page just shapes the strings' tone)
  // -----------------------------------------------------------------
  float damping;          // ALGO  unshifted      - KS tone (bright -> dark)
  float white_pink_mix;   // PARAM unshifted      - noise floor spectrum (0=pink, 1=white)
  float karplus_lpf;      // LVL1  unshifted      - post-KS LPF cutoff (200 Hz..16 kHz, log)
  float noise_floor_base; // LVL2  unshifted      - constant baseline noise into K-S
  // All four shifted slots on KARPLUS are reserved.

  // Legacy pluck-related fields, kept in the struct so other code that
  // still references them compiles, but no slot writes here anymore.
  float burst_shape;
  float pluck_shape;
  float pluck_amplitude;
  float exciter_rate;

  // -----------------------------------------------------------------
  // Page 2 - REVERB
  // -----------------------------------------------------------------
  float reverb_size;      // ALGO  unshifted      - decay time
  float reverb_diffusion; // PARAM unshifted      - discrete echoes ↔ smooth
  float reverb_lp;        // LVL1  unshifted      - HF damping in tail
  float reverb_amount;    // LVL2  unshifted      - dry/wet (knob-only)

  float reverb_predelay;  // ALGO  shifted        - pre-delay before tank (NYI)
  float smear;            // PARAM shifted        - reverb-tail -> KS feedback
  float reverb_drive;     // LVL1  shifted        - input drive
  float reverb_res_b;     // LVL2  shifted        - reserved

  // -----------------------------------------------------------------
  // Page 3 - SHIMMER & GLIMMER (DSP disabled for now)
  // -----------------------------------------------------------------
  float harmonics;        // ALGO  unshifted      - modal resonator mix
  float modal_brightness; // PARAM unshifted      - modal Q taper
  float modal_stiffness;  // LVL1  unshifted      - harmonic ↔ bell-like
  float dynamics;         // LVL2  unshifted      - reserved for compressor

  float modal_count;      // ALGO  shifted        - active mode count 4..16
  float modal_pickup;     // PARAM shifted        - pickup position
  float reverb_shimmer;   // LVL1  shifted        - reverb LFO depth (moved here)
  float reverb_shim_rate; // LVL2  shifted        - reverb LFO rate scaling (moved)

  // Internal scratch - cv_scaler stashes the raw V/oct ADC reading here
  // so Drone can do the semitone math. Not a user-facing slot.
  float reserved_a;
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_PARAMETERS_H_
