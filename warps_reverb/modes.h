// Firmware modes - each mode bundles a parameter mapping (which knob drives
// which parameter), default parameter values, DSP coefficient overrides,
// and an OSC-LED color used to indicate the active mode.
//
// Button gestures (mode-independent):
//   - Hold        -> shift layer (knobs gain second function)
//   - Short tap   -> cycle to next mode (wraps)
//
// Adding a new mode:
//   1. Append an entry to the FirmwareMode enum
//   2. Add a const ModeConfig in modes.cc kModeTable
//   3. Update kModeCount via MODE_COUNT
//   4. Add an entry to the panel cheatsheet SVG

#ifndef WARPS_REVERB_MODES_H_
#define WARPS_REVERB_MODES_H_

#include "stmlib/stmlib.h"

#include "warps_reverb/dsp/parameters.h"

namespace warps_reverb {

// IDs for every user-facing reverb parameter. PARAM_NONE = unmapped slot,
// useful when a mode doesn't want to bind one of the pots.
enum ParameterId {
  PARAM_NONE = 0,
  PARAM_DECAY,
  PARAM_SIZE,
  PARAM_DIFFUSION,
  PARAM_MOTION,
  PARAM_SPEED,
  PARAM_PRE_DELAY,
  PARAM_DRY_WET,
  PARAM_SHIMMER,
  PARAM_TONE,
  PARAM_FILTER,
  PARAM_NOISE,
  PARAM_DRIVE,
  PARAM_LAST
};

enum FirmwareMode {
  MODE_REVERB,        // mode 0 - OSC LED off    - clean hall
  MODE_DRONE,         // mode 1 - OSC LED green  - sci-fi self-osc drone
  MODE_HARSH,         // mode 2 - OSC LED orange - the first audible build
                      //                          (kept for reference / fun)
  // Mode 3 (red) : reserved
  MODE_COUNT
};

// Pot dual-function binding. unshifted layer is also the CV target.
struct PotMapping {
  ParameterId unshifted;
  ParameterId shifted;
};

// DspOverrides moved to dsp/parameters.h so Reverb can consume the struct
// directly without pulling in modes.h.

struct ModeConfig {
  const char* name;
  ReverbParameters defaults;
  PotMapping algorithm;       // big ALGORITHM knob
  PotMapping timbre;          // big TIMBRE knob
  PotMapping level1;          // LEVEL 1 trimmer
  PotMapping level2;          // LEVEL 2 trimmer
  DspOverrides dsp;
  uint8_t osc_red;            // OSC LED steady-state color while this mode active
  uint8_t osc_green;
};

extern const ModeConfig kModeTable[MODE_COUNT];

inline const ModeConfig& ModeFor(FirmwareMode m) {
  return kModeTable[m];
}

// Returns a pointer to the named field in *p for the given ParameterId.
// PARAM_NONE returns a pointer to a process-wide sink (writes are dropped).
float* WriteSlot(ReverbParameters* p, ParameterId id);

// Read-only counterpart for const access (mode defaults).
float ReadSlot(const ReverbParameters& p, ParameterId id);

}  // namespace warps_reverb

#endif  // WARPS_REVERB_MODES_H_
