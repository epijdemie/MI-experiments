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
  float chord;            // ALGO  unshifted, +CV  - density 0..1 -> 5 zones
  // PARAM unshifted knob = bass_note (chord-tone zone snap, 7 zones,
  // chord-bank-aware). PARAM_CV adds chromatic semitones on top via
  // bass_voct_semitones below.
  float bass_note;
  float pitch_octave;     // LVL1  unshifted, +V/oct CV - chord octave selector (6 zones)
  float lpf_cutoff;       // LVL2  unshifted, +CV  - LPF cutoff, log 60 Hz…12 kHz

  float chord_mode;       // ALGO  shifted        - chord bank (6 banks)
  float bass_glide;       // PARAM shifted        - bass portamento (CCW=off, CW=0.5 s)
  float chord_volume;     // LVL1  shifted        - chord output level (post-reverb)
  float bass_volume;      // LVL2  shifted        - bass output level (post-reverb sum)

  // KARPLUS - string tone shaping + bass voice character
  float damping;          // ALGO  unshifted      - chord K-S tone (bright -> dark)
  float bass_weight;      // PARAM unshifted      - bass sub-octave saw mix (phat)
  float karplus_lpf;      // LVL1  unshifted      - post-KS LPF cutoff (200 Hz..16 kHz, log)
  float noise_floor_base; // LVL2  unshifted      - constant baseline noise into chord K-S
  // ALGO  shifted = harmonics (modal-bank mix)
  float bass_drive;       // PARAM shifted        - bass decay + saturator (drony / cinematic)
  float filter_resonance; // LVL1  shifted        - perf LPF Q (0.7..10)
  float white_pink_mix;   // LVL2  shifted        - chord K-S noise color (0=pink, 1=white)

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
  float distortion_warmth;// PARAM unshifted      - curve: 1 = smooth tube, 0 = hard fuzz
  float distortion_tone;  // LVL1  unshifted      - post-distortion brightness (LPF cutoff)
  float vinyl_noise;      // LVL2  unshifted      - vinyl/tape noise amount (post-chain sum)
  float distortion_bias;  // PARAM shifted        - asymmetric clipping bias (even harmonics)
  float vinyl_color;      // LVL2  shifted        - noise LPF cutoff (CCW=warm, CW=bright)
  float vinyl_flutter;    // LVL1  shifted        - vinyl noise volume flutter
                          //   0      = off
                          //   0..0.5 = triangle LFO, slow → fast
                          //   0.5..1 = slewed random, fast → slow

  // cv_scaler stashes calibrated v/oct here (semitones), one per input
  float chord_voct_semitones;   // LVL1 CV, calibrated by chord cal pair
  float bass_voct_semitones;    // PARAM CV, calibrated by bass cal pair
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_PARAMETERS_H_
