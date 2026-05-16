// 4-voice ks chord bank.
//
// ChordBank (BIG shifted): DETUNE / MINOR / MAJOR / DOM7 / DRONE / WEIRD.
// DensityZone (BIG unshifted, chord banks only):
//   POWER -> TRIAD -> 7TH -> 9TH -> EXT.
//
// DETUNE ignores density zones - density runs as a continuous spread shape
// across the 4 voices.
//
// `ratios()` is published so the saw-stack sub follows the same math

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
  static constexpr int kNumVoices = 4;

  VoiceBank() { }

  void Init(float sample_rate) {
    sample_rate_ = sample_rate;
    for (int i = 0; i < kNumVoices; ++i) voices_[i].Init();
  }

  // density_norm (0..1) - for chord banks, quantises to one of 5 density
  // zones. For DETUNE bank, runs continuously and scales the spread.
  // bank_norm    (0..1) - quantises to one of 6 ChordBank values
  void set_chord(float root_hz, float density_norm, float bank_norm) {
    // Quantise bank
    int bi = static_cast<int>(bank_norm * BANK_COUNT);
    if (bi < 0) bi = 0;
    if (bi >= BANK_COUNT) bi = BANK_COUNT - 1;
    bank_ = static_cast<ChordBank>(bi);

    float ratios[kNumVoices];

    if (bank_ == BANK_DETUNE) {
      // No discrete density zones - density runs continuously and
      // drives an asymmetric spread shape. Five sub-shapes are
      // crossfaded by the knob position so a slow sweep traverses
      // unison -> chorus -> phat -> piano-stretch -> wide-cluster
      density_zone_ = DENSITY_POWER;  // legacy LED hint, unused here
      const float dn = density_norm < 0.0f ? 0.0f
                       : (density_norm > 1.0f ? 1.0f : density_norm);
      DetuneRatios(dn, ratios);
    } else {
      // Quantise density into 5 zones
      int di = static_cast<int>(density_norm * DENSITY_COUNT);
      if (di < 0) di = 0;
      if (di >= DENSITY_COUNT) di = DENSITY_COUNT - 1;
      density_zone_ = static_cast<DensityZone>(di);

      // Lookup the 4-voice semitone offsets from root for (bank, density).
      // Banks here are indexed 1..5 (DETUNE is index 0, handled above)
      // so the table has BANK_COUNT-1 = 5 rows.
      //
      // Row order matches enum order minus DETUNE:
      //   0 MINOR  1 MAJOR  2 DOM7  3 DRONE  4 WEIRD
      // Column order:
      //   0 POWER  1 TRIAD  2 7TH  3 9TH  4 EXT
      static constexpr int kSemis[BANK_COUNT - 1][DENSITY_COUNT][4] = {
        // MINOR
        {{ 0,  0,  7,  7},   // power 5
         { 0,  3,  7, 12},   // minor triad
         { 0,  3,  7, 10},   // m7
         { 0,  3, 10, 14},   // m9 (drop 5)
         { 0,  3,  5, 10}},  // m + 4 + ♭7 (Andalusian flavour)
        // MAJOR
        {{ 0,  0,  7,  7},   // power 5
         { 0,  4,  7, 12},   // major triad
         { 0,  4,  7, 11},   // maj7
         { 0,  4, 11, 14},   // maj9 (drop 5)
         { 0,  4, 11, 21}},  // maj13 (open voicing)
        // DOM7
        {{ 0,  0,  7,  7},   // power 5
         { 0,  4,  7, 10},   // 7
         { 0,  4, 10, 14},   // 9
         { 0,  4, 10, 13},   // ♭9
         { 0,  4,  6, 10}},  // lydian dom (♯4)
        // DRONE-PAD
        {{ 0,  0,  0,  0},   // pure unison
         { 0, 12,  0, 12},   // octave double
         { 0,  7, 12, 19},   // root + 5th + octaves
         { 0,  5,  7, 12},   // quartal triad
         { 0,  5, 10, 15}},  // stacked 4ths
        // WEIRD
        {{ 0,  1,  7, 13},   // ♭9 stab
         { 0,  6, 10, 14},   // altered dom (♯4-♭7-9)
         { 0,  4,  8, 10},   // whole-tone-ish
         { 0,  3,  9, 12},   // polychord m + VI
         { 0,  1,  2,  3}},  // chromatic cluster
      };
      const int row = static_cast<int>(bank_) - 1;  // DETUNE skipped
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

  // Last chord ratios computed by set_chord(). Drone uses these so the
  // sub-osc saw stack mirrors the chord knob (detune spread + chord
  // intervals) without duplicating interval tables
  inline const float* ratios() const { return ratios_; }

  inline ChordBank    bank()          const { return bank_; }
  inline DensityZone  density_zone()  const { return density_zone_; }

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
    return sum * 0.25f;
  }

 private:
  // DETUNE bank shape: 5 sub-shapes crossfaded by density.
  //   0.00..0.20  tight       (0 -> ±3 c symmetric outer pair)
  //   0.20..0.40  chorus      (±3 -> ±15 c)
  //   0.40..0.60  phat        (±15 -> ±30 c)
  //   0.60..0.80  piano-stretch (asymmetric ascending stack, ±0..+50 c)
  //   0.80..1.00  wide JI-ish (3:4:5:6-flavoured spread, > ±50 c)
  void DetuneRatios(float dn, float* r) const {
    // Helper: 4-voice symmetric spread around 1.0 by total cents `c`
    // (outer pair separation in cents)
    const float kCent = 1.0f / 1200.0f;
    if (dn < 0.20f) {
      const float t = dn * 5.0f;
      const float cents = 3.0f * t;          // 0..3 c outer pair
      const float s = cents * kCent;
      r[0] = 1.0f - s;
      r[1] = 1.0f - s * 0.33f;
      r[2] = 1.0f + s * 0.33f;
      r[3] = 1.0f + s;
    } else if (dn < 0.40f) {
      const float t = (dn - 0.20f) * 5.0f;
      const float cents = 3.0f + 12.0f * t;  // 3..15 c
      const float s = cents * kCent;
      r[0] = 1.0f - s;
      r[1] = 1.0f - s * 0.33f;
      r[2] = 1.0f + s * 0.33f;
      r[3] = 1.0f + s;
    } else if (dn < 0.60f) {
      const float t = (dn - 0.40f) * 5.0f;
      const float cents = 15.0f + 15.0f * t; // 15..30 c
      const float s = cents * kCent;
      r[0] = 1.0f - s;
      r[1] = 1.0f - s * 0.33f;
      r[2] = 1.0f + s * 0.33f;
      r[3] = 1.0f + s;
    } else if (dn < 0.80f) {
      // Piano-stretch: ascending stack, each voice sharper than the last
      const float t = (dn - 0.60f) * 5.0f;
      const float step = (30.0f + 20.0f * t) * kCent;  // 30..50 c per step
      r[0] = 1.0f;
      r[1] = 1.0f + step;
      r[2] = 1.0f + step * 2.0f;
      r[3] = 1.0f + step * 3.0f;
    } else {
      // Wide JI-ish: symmetric outward burst with one voice biased
      // toward a just-major-third ratio so beating becomes complex
      const float t = (dn - 0.80f) * 5.0f;
      const float wide = (50.0f + 50.0f * t) * kCent;  // 50..100 c
      r[0] = 1.0f - wide;
      r[1] = 1.0f - wide * 0.30f;
      r[2] = 1.0f + wide * 0.30f;
      // ratio drifts toward 5/4 (just major 3rd, +386 c) at full density
      const float ji_pull = 0.10f * t;       // 0..0.10
      r[3] = (1.0f + wide) * (1.0f - ji_pull) + 1.25f * ji_pull;
    }
  }

  KarplusVoice voices_[kNumVoices];
  float        sample_rate_   = 48000.0f;
  ChordBank    bank_          = BANK_DETUNE;
  DensityZone  density_zone_  = DENSITY_POWER;
  float        ratios_[kNumVoices] = {1.0f, 1.0f, 1.0f, 1.0f};

  DISALLOW_COPY_AND_ASSIGN(VoiceBank);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_VOICE_BANK_H_
