// live params - 4 pages × 2 layers. all values 0..1 unless noted

#ifndef WARPS_DRONE_DSP_PARAMETERS_H_
#define WARPS_DRONE_DSP_PARAMETERS_H_

#include "stmlib/stmlib.h"

namespace warps_drone {

enum ControlPage {
  PAGE_PERFORMANCE = 0,   // OSC LED off
  PAGE_KARPLUS,           // OSC LED green
  PAGE_REVERB,            // OSC LED orange
  PAGE_OVERDRIVE,         // OSC LED red - overdrive section
  PAGE_COUNT
};
constexpr ControlPage PAGE_DIST    = PAGE_OVERDRIVE;
constexpr ControlPage PAGE_SHIMMER = PAGE_OVERDRIVE;

struct DroneParameters {
  // PERFORMANCE
  float chord;            // ALGO  unshifted, +CV  - voicing 0..1 -> 8 zones
  float pulse_freq;       // PARAM unshifted, +CV  - K-S exciter pulse osc, log 4 Hz..200 Hz
  float pitch_octave;     // LVL1  unshifted, +V/oct CV - octave selector (6 zones)
  float lpf_cutoff;       // LVL2  unshifted, +CV  - LPF cutoff, log 60 Hz…12 kHz

  float chord_mode;       // ALGO  shifted        - chord quality (6 zones)
  float pulse_gain;       // PARAM shifted        - pulse-osc level into K-S
  float pitch;            // LVL1  shifted        - fine pitch, semitone within octave
  float filter_resonance; // LVL2  shifted        - perf LPF Q (0.7..10)

  // KARPLUS - string tone shaping (pulse-osc exciter lives on PERF)
  float damping;          // ALGO  unshifted      - KS tone (bright -> dark)
  float white_pink_mix;   // PARAM unshifted      - noise floor spectrum (0=pink, 1=white)
  float karplus_lpf;      // LVL1  unshifted      - post-KS LPF cutoff (200 Hz..16 kHz, log)
  float noise_floor_base; // LVL2  unshifted      - constant baseline noise into K-S
  // ALGO   shifted = harmonics (modal-bank mix)
  float modal_count;      // PARAM shifted        - modal-bank active modes (4..16)
  float modal_pickup;     // LVL1  shifted        - modal pickup position (cosine weighting)
  float modal_stiffness;  // LVL2  shifted        - modal-bank inharmonicity (harmonic ↔ bell)

  // REVERB
  float reverb_size;      // ALGO  unshifted      - decay time
  float reverb_diffusion; // PARAM unshifted      - discrete echoes ↔ smooth
  float reverb_lp;        // LVL1  unshifted      - HF damping in tail
  float reverb_amount;    // LVL2  unshifted      - dry/wet (knob-only)

  float reverb_predelay;  // ALGO  shifted        - pre-delay before tank (NYI)
  float smear;            // PARAM shifted        - reverb-tail -> KS feedback
  float reverb_drive;     // LVL1  shifted        - input drive
  float reverb_shim_rate; // LVL2  shifted        - plate-LFO rate scaling (chorus speed)

  float harmonics;        // KARPLUS ALGO shifted  - modal-bank mix
  float modal_brightness; // (unwired)
  float dynamics;         // (unwired)

  // OVERDRIVE
  float distortion;       // ALGO  unshifted      - drive amount
  float distortion_warmth;// PARAM unshifted      - curve+drive: 1 = smooth low-drive tube, 0 = hot hard-clip fuzz
  float distortion_tone;  // LVL1  unshifted      - post-distortion brightness (LPF cutoff)
  float vinyl_noise;      // LVL2  unshifted      - vinyl/tape hiss amount (pre-reverb)
  float distortion_bias;  // LVL2  shifted        - asymmetric clipping bias (even harmonics)

  // scratch - cv_scaler stashes calibrated v/oct (semitones) here
  float reserved_a;
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_PARAMETERS_H_
