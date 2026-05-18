// 3-voice ks chord bank. (Voice 4 is reused in Drone as a standalone
// bass string with its own pluck/duck behaviour — not managed here.)
//
// ChordBank (BIG shifted): DETUNE / MINOR / MAJOR / DOM7 / DRONE / WEIRD.
// DensityZone (BIG unshifted, chord banks only):
//   POWER → TRIAD → 7TH → 9TH → EXT.
//
// DETUNE ignores density zones — density runs as a continuous spread shape
// across the 3 voices.
//
// `ratios()` is published so the saw-stack sub and the bass string follow
// the same chord math.

#ifndef WARPS_DRONE_DSP_VOICE_BANK_H_
#define WARPS_DRONE_DSP_VOICE_BANK_H_

#include <math.h>

#include "warps_drone/dsp/karplus_voice.h"

namespace warps_drone {

enum ChordBank {
  BANK_DETUNE = 0,
  BANK_MINOR,
  BANK_MAJOR,
  BANK_DOM7,
  BANK_DRONE,
  BANK_WEIRD,
  BANK_COUNT
};

enum DensityZone {
  DENSITY_POWER = 0,
  DENSITY_TRIAD,
  DENSITY_7TH,
  DENSITY_9TH,
  DENSITY_EXT,
  DENSITY_COUNT
};

class VoiceBank {
 public:
  static constexpr int kNumVoices = 3;

  VoiceBank() { }

  void Init(float sample_rate) {
    sample_rate_ = sample_rate;
    for (int i = 0; i < kNumVoices; ++i) voices_[i].Init();
  }

  // density_norm (0..1) — for chord banks, quantises to one of 5 density
  // zones. For DETUNE bank, runs continuously and scales the spread.
  // bank_norm    (0..1) — quantises to one of 6 ChordBank values.
  void set_chord(float root_hz, float density_norm, float bank_norm) {
    int bi = static_cast<int>(bank_norm * BANK_COUNT);
    if (bi < 0) bi = 0;
    if (bi >= BANK_COUNT) bi = BANK_COUNT - 1;
    bank_ = static_cast<ChordBank>(bi);

    float ratios[kNumVoices];

    if (bank_ == BANK_DETUNE) {
      density_zone_ = DENSITY_POWER;
      const float dn = density_norm < 0.0f ? 0.0f
                       : (density_norm > 1.0f ? 1.0f : density_norm);
      DetuneRatios(dn, ratios);
    } else {
      int di = static_cast<int>(density_norm * DENSITY_COUNT);
      if (di < 0) di = 0;
      if (di >= DENSITY_COUNT) di = DENSITY_COUNT - 1;
      density_zone_ = static_cast<DensityZone>(di);

      // 3-voice semitone offsets from root.
      // Row order (skipping DETUNE): MINOR / MAJOR / DOM7 / DRONE / WEIRD.
      // Column order:                POWER / TRIAD / 7TH / 9TH / EXT.
      static constexpr int kSemis[BANK_COUNT - 1][DENSITY_COUNT][kNumVoices] = {
        // MINOR
        {{ 0,  7, 12},   // open power (root + 5 + 8)
         { 0,  3,  7},   // minor triad
         { 0,  3, 10},   // m7 (drop 5)
         { 0, 10, 14},   // m9 over ♭7
         { 0,  5, 10}},  // root + 4 + ♭7
        // MAJOR
        {{ 0,  7, 12},   // open power
         { 0,  4,  7},   // major triad
         { 0,  4, 11},   // maj7 (drop 5)
         { 0, 11, 14},   // maj7 + 9 (drop 5)
         { 0,  4, 21}},  // root + 3 + maj13
        // DOM7
        {{ 0,  7, 12},   // open power
         { 0,  4, 10},   // dom7 (drop 5)
         { 0, 10, 14},   // dom9 (drop 5)
         { 0, 10, 13},   // ♭9 (drop 5)
         { 0,  6, 10}},  // lydian dom (♯4 + ♭7)
        // DRONE-PAD
        {{ 0,  0,  0},   // pure unison
         { 0,  0, 12},   // octave double
         { 0,  7, 12},   // root + 5 + 8
         { 0,  5,  7},   // quartal triad
         { 0,  5, 10}},  // stacked 4ths
        // WEIRD
        {{ 0,  1,  7},   // ♭9 stab
         { 0,  6, 10},   // altered dom (♯4-♭7)
         { 0,  4,  8},   // whole-tone-ish
         { 0,  3,  9},   // polychord m + VI
         { 0,  1,  2}},  // chromatic cluster
      };
      const int row = static_cast<int>(bank_) - 1;
      for (int v = 0; v < kNumVoices; ++v) {
        const int s = kSemis[row][di][v];
        ratios[v] = powf(2.0f, static_cast<float>(s) * (1.0f / 12.0f));
      }
    }

    for (int v = 0; v < kNumVoices; ++v) {
      float f = root_hz * ratios[v];
      if (f < 23.0f) f = 23.0f;
      voices_[v].set_delay(sample_rate_ / f);
      ratios_[v] = ratios[v];
    }
  }

  // Last chord ratios computed by set_chord(). Drone uses these for the
  // saw-stack sub-osc and the bass-string chord-tone snap.
  inline const float* ratios() const { return ratios_; }

  inline ChordBank   bank()         const { return bank_; }
  inline DensityZone density_zone() const { return density_zone_; }

  inline void set_decay(float d) {
    for (int v = 0; v < kNumVoices; ++v) voices_[v].set_decay(d);
  }
  inline void set_brightness(float b) {
    for (int v = 0; v < kNumVoices; ++v) voices_[v].set_brightness(b);
  }
  inline void set_noise_floor(float n) {
    for (int v = 0; v < kNumVoices; ++v) voices_[v].set_noise_floor(n);
  }
  inline void set_noise_color(float mix) {
    for (int v = 0; v < kNumVoices; ++v) voices_[v].set_noise_color(mix);
  }
  inline void Excite(float amount, int length, float shape) {
    for (int v = 0; v < kNumVoices; ++v) {
      voices_[v].Excite(amount, length, shape);
    }
  }

  inline float Process(float feedback_in) {
    float sum = 0.0f;
    for (int v = 0; v < kNumVoices; ++v) {
      sum += voices_[v].Process(feedback_in);
    }
    return sum * (1.0f / 3.0f);
  }

 private:
  // DETUNE bank — 5 sub-shapes crossfaded by density across 3 voices.
  //   0.00..0.20  tight       (0 → ±3 c symmetric)
  //   0.20..0.40  chorus      (±3 → ±15 c)
  //   0.40..0.60  phat        (±15 → ±30 c)
  //   0.60..0.80  piano-stretch (ascending stack, ±0..+50 c per step)
  //   0.80..1.00  wide JI-ish (top voice biases toward just-major-3rd 5/4)
  void DetuneRatios(float dn, float* r) const {
    const float kCent = 1.0f / 1200.0f;
    if (dn < 0.20f) {
      const float t = dn * 5.0f;
      const float s = (3.0f * t) * kCent;
      r[0] = 1.0f - s;  r[1] = 1.0f;  r[2] = 1.0f + s;
    } else if (dn < 0.40f) {
      const float t = (dn - 0.20f) * 5.0f;
      const float s = (3.0f + 12.0f * t) * kCent;
      r[0] = 1.0f - s;  r[1] = 1.0f;  r[2] = 1.0f + s;
    } else if (dn < 0.60f) {
      const float t = (dn - 0.40f) * 5.0f;
      const float s = (15.0f + 15.0f * t) * kCent;
      r[0] = 1.0f - s;  r[1] = 1.0f;  r[2] = 1.0f + s;
    } else if (dn < 0.80f) {
      const float t = (dn - 0.60f) * 5.0f;
      const float step = (30.0f + 20.0f * t) * kCent;
      r[0] = 1.0f;
      r[1] = 1.0f + step;
      r[2] = 1.0f + step * 2.0f;
    } else {
      const float t = (dn - 0.80f) * 5.0f;
      const float wide = (50.0f + 50.0f * t) * kCent;
      const float ji_pull = 0.10f * t;
      r[0] = 1.0f - wide;
      r[1] = 1.0f + wide * 0.30f;
      r[2] = (1.0f + wide) * (1.0f - ji_pull) + 1.25f * ji_pull;
    }
  }

  KarplusVoice voices_[kNumVoices];
  float        sample_rate_   = 48000.0f;
  ChordBank    bank_          = BANK_DETUNE;
  DensityZone  density_zone_  = DENSITY_POWER;
  float        ratios_[kNumVoices] = {1.0f, 1.0f, 1.0f};

  DISALLOW_COPY_AND_ASSIGN(VoiceBank);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_VOICE_BANK_H_
