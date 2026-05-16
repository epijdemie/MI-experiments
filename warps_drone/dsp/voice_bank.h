// 4-voice Karplus-Strong chord bank with 8 discrete chord zones.
//
// Voicing zones (relative to chord root frequency), ordered from
// open/light through cool/minor -> neutral -> warm/major:
//   0  DETUNE_UNISON   root × {1.000, 0.997, 1.003, 1.006}   - slow beats
//   1  OCTAVES         root × {1, 2,    2,    4    }         - pure stack
//   2  OPEN_FIFTH      root × {1, 3/2,  2,    3    }         - power chord
//   3  MINOR           root × {1, 6/5,  3/2,  2    }         - minor triad
//   4  MIN7            root × {1, 6/5,  3/2,  9/5  }         - minor 7
//   5  SUS4            root × {1, 4/3,  3/2,  2    }         - sus4 triad
//   6  MAJOR           root × {1, 5/4,  3/2,  2    }         - major triad
//   7  MAJ7            root × {1, 5/4,  3/2,  15/8 }         - major 7
//
// A continuous `voicing` parameter (0..1) is quantised into one of the
// eight zones; the zone index is exposed for the UI LED.

#ifndef WARPS_DRONE_DSP_VOICE_BANK_H_
#define WARPS_DRONE_DSP_VOICE_BANK_H_

#include "warps_drone/dsp/karplus_voice.h"

namespace warps_drone {

enum VoicingZone {
  VOICING_DETUNE_UNISON = 0,
  VOICING_OCTAVES,
  VOICING_OPEN_FIFTH,
  VOICING_MINOR,
  VOICING_MIN7,
  VOICING_SUS4,
  VOICING_MAJOR,
  VOICING_MAJ7,
  VOICING_ZONE_COUNT
};

class VoiceBank {
 public:
  static constexpr int kNumVoices = 4;

  VoiceBank() { }

  void Init(float sample_rate) {
    sample_rate_ = sample_rate;
    for (int i = 0; i < kNumVoices; ++i) voices_[i].Init();
  }

  void set_chord(float root_hz, float voicing) {
    // Fixed ratios for zones 1..7 (octaves -> min7). Zone 0 (detune
    // unison) is computed dynamically from the knob's sub-position
    // so you can dial spread from perfect-unison (no beats) to
    // the boundary with zone 1 (≈±1.2 % spread).
    static constexpr float kRatios[VOICING_ZONE_COUNT][kNumVoices] = {
      {0},                                  // DETUNE_UNISON - unused
      {1.000f,  2.000f,  2.000f,  4.000f},  // OCTAVES
      {1.000f,  1.500f,  2.000f,  3.000f},  // OPEN_FIFTH (P5+oct)
      {1.000f,  1.200f,  1.500f,  2.000f},  // MINOR
      {1.000f,  1.200f,  1.500f,  1.800f},  // MIN7
      {1.000f,  1.333f,  1.500f,  2.000f},  // SUS4
      {1.000f,  1.250f,  1.500f,  2.000f},  // MAJOR
      {1.000f,  1.250f,  1.500f,  1.875f},  // MAJ7
    };

    int z = static_cast<int>(voicing * VOICING_ZONE_COUNT);
    if (z < 0) z = 0;
    if (z >= VOICING_ZONE_COUNT) z = VOICING_ZONE_COUNT - 1;
    zone_ = static_cast<VoicingZone>(z);

    float ratios[kNumVoices];
    if (z == VOICING_DETUNE_UNISON) {
      // Sub-position within zone 0 (zone width = 1/8 = 0.125).
      // t=0 -> all unison; t=1 (boundary with octaves) -> max spread.
      float t = voicing * static_cast<float>(VOICING_ZONE_COUNT);
      if (t > 1.0f) t = 1.0f;
      const float spread = 0.012f * t;       // up to ±1.2 % at the edge
      ratios[0] = 1.000f;
      ratios[1] = 1.000f - spread * 0.5f;
      ratios[2] = 1.000f + spread * 0.5f;
      ratios[3] = 1.000f + spread;
    } else {
      for (int v = 0; v < kNumVoices; ++v) ratios[v] = kRatios[z][v];
    }

    for (int v = 0; v < kNumVoices; ++v) {
      float f = root_hz * ratios[v];
      if (f < 23.0f) f = 23.0f;
      voices_[v].set_delay(sample_rate_ / f);
    }
  }

  inline VoicingZone zone() const { return zone_; }

  inline void set_decay(float d) {
    for (int v = 0; v < kNumVoices; ++v) voices_[v].set_decay(d);
  }
  inline void set_brightness(float b) {
    for (int v = 0; v < kNumVoices; ++v) voices_[v].set_brightness(b);
  }
  inline void set_noise_floor(float n) {
    for (int v = 0; v < kNumVoices; ++v) voices_[v].set_noise_floor(n);
  }
  inline void Excite(float amount, int duration) {
    for (int v = 0; v < kNumVoices; ++v) voices_[v].Excite(amount, duration);
  }

  inline float Process(float feedback_in) {
    float sum = 0.0f;
    for (int v = 0; v < kNumVoices; ++v) {
      sum += voices_[v].Process(feedback_in);
    }
    return sum * 0.25f;
  }

 private:
  KarplusVoice voices_[kNumVoices];
  float        sample_rate_ = 48000.0f;
  VoicingZone  zone_        = VOICING_DETUNE_UNISON;

  DISALLOW_COPY_AND_ASSIGN(VoiceBank);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_VOICE_BANK_H_
