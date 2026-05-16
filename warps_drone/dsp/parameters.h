// Live parameter struct for the drone firmware. Single layer (no shift),
// all fields normalised [0, 1] unless noted.

#ifndef WARPS_DRONE_DSP_PARAMETERS_H_
#define WARPS_DRONE_DSP_PARAMETERS_H_

#include "stmlib/stmlib.h"

namespace warps_drone {

struct DroneParameters {
  // Karplus voice bank
  float pitch;         // chord root: 0 -> ~50 Hz, 1 -> ~440 Hz (log)
  float damping;       // KS in-loop damping: 0 = bright/long, 1 = dark/short
  float excite_amount; // noise vs. plectrum mix into KS input. 0 = pure pluck,
                       // 1 = pure noise (sustained excitation)
  float voicing;       // 0 = unison-detuned, 0.5 = octaves, 1 = major triad

  // Modal resonator (parallel shimmer source)
  float harmonics;     // mix of modal resonator into the signal path
  float brightness;    // resonator Q / how many modes ring strongly

  // Filter
  float filter_cutoff; // SVF cutoff
  float filter_morph;  // 0 = LP, 1 = HP

  // Tape delay
  float delay_time;    // 0 -> ~20 ms, 1 -> ~500 ms
  float delay_fb;      // tape feedback 0..0.95

  // Reverb
  float reverb_size;   // reverb time
  float reverb_amount; // wet mix
  float reverb_to_ks;  // reverb tail -> KS exciter feedback (the "smear" knob)
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_PARAMETERS_H_
