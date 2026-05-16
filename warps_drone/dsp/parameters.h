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
  float lpf_cutoff;       // PARAM unshifted, +CV  - LPF cutoff, log 60 Hz…12 kHz
  float pitch;            // LVL1  unshifted, +V/oct CV - semitone within octave
  float reverb_amount;    // LVL2  unshifted, +CV  - wet/dry

  float chord_mode;       // ALGO  shifted        - chord quality (6 zones)
  float hpf_cutoff;       // PARAM shifted        - HPF cutoff, log 20 Hz…800 Hz
  float pitch_octave;     // LVL1  shifted        - 0..1 -> octave selector (6 zones)
  float filter_resonance; // LVL2  shifted        - 0..1 -> Q 0.7..10

  // -----------------------------------------------------------------
  // Page 1 - KARPLUS (exciter character)
  // -----------------------------------------------------------------
  float burst_shape;      // ALGO  unshifted      - pluck envelope morph (A/R)
  float pluck_shape;      // PARAM unshifted      - pluck total duration
  float exciter_rate;     // LVL1  unshifted      - auto-trigger LFO Hz, 0 = off
  float noise_floor_base; // LVL2  unshifted      - constant baseline noise

  float pluck_amplitude;  // ALGO  shifted        - transient burst size
  float damping;          // PARAM shifted        - KS tone (bright->dark+noisy)
  float karplus_lpf;      // LVL1  shifted        - post-KS LPF cutoff (200 Hz..16 kHz, log)
  float white_pink_mix;   // LVL2  shifted        - noise spectrum (0=pink, 1=white)

  // -----------------------------------------------------------------
  // Page 2 - REVERB
  // -----------------------------------------------------------------
  float reverb_size;      // ALGO  unshifted      - decay time
  float reverb_diffusion; // PARAM unshifted      - discrete echoes ↔ smooth
  float reverb_lp;        // LVL1  unshifted      - HF damping in tail
  float smear;            // LVL2  unshifted      - reverb-tail -> KS feedback (moved)

  // ALGO shifted = reverb_amount duplicate (same field as page 0 LVL2);
  // see cv_scaler.cc for the override logic. No separate field needed.
  float reverb_drive;     // PARAM shifted        - input drive
  float reverb_predelay;  // LVL1  shifted        - pre-delay before tank (NYI)
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
