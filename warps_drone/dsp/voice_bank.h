// 4-voice Karplus-Strong chord bank.
//
// Two-axis chord selection:
//   ChordStack (BIG unshifted) - *what intervals are stacked*
//     0  UNISON      all 4 voices at root
//     1  DETUNE      slight ratio spread -> slow beating
//                    (sub-position within the zone scales the spread,
//                     so the knob feels phat across the whole zone)
//     2  +3rd        root, 3rd, root+oct, 3rd+oct
//     3  +5th        root, 3rd, 5th, root+oct
//     4  +7th        root, 3rd, 5th, 7th
//     5  +9th        root, 3rd, 7th, 9th  (drop 5th, jazzy)
//
//   ChordMode  (BIG shifted) - *what quality the 3rd/7th/9th are*
//     0  MAJOR       3=+4, 5=+7, 7=+11, 9=+14
//     1  MINOR       3=+3, 5=+7, 7=+10, 9=+14
//     2  DOM7        3=+4, 5=+7, 7=+10, 9=+14
//     3  DIM         3=+3, 5=+6, 7=+9,  9=+13
//     4  SUS2        3=+2, 5=+7, 7=+10, 9=+14
//     5  SUS4        3=+5, 5=+7, 7=+10, 9=+14
//
// Mode only affects stacks 2..5 (intervals containing 3rd/5th/7th/9th);
// stacks 0..1 (unison, detune) are mode-agnostic.

#ifndef WARPS_DRONE_DSP_VOICE_BANK_H_
#define WARPS_DRONE_DSP_VOICE_BANK_H_

#include <math.h>

#include "warps_drone/dsp/karplus_voice.h"

namespace warps_drone {

enum ChordStack {
  STACK_UNISON = 0,
  STACK_DETUNE,
  STACK_3RD,
  STACK_5TH,
  STACK_7TH,
  STACK_9TH,
  STACK_COUNT
};

enum ChordMode {
  MODE_MAJOR = 0,
  MODE_MINOR,
  MODE_DOM7,
  MODE_DIM,
  MODE_SUS2,
  MODE_SUS4,
  MODE_COUNT
};

// Backward-compat alias for the UI's old enum name; the LED switch in
// ui.cc still uses `VoicingZone` so we keep it pointing at the new
// stack enum without breaking anything.
typedef ChordStack VoicingZone;

class VoiceBank {
 public:
  static constexpr int kNumVoices = 4;

  VoiceBank() { }

  void Init(float sample_rate) {
    sample_rate_ = sample_rate;
    for (int i = 0; i < kNumVoices; ++i) voices_[i].Init();
  }

  // stack_norm (0..1) -> one of 6 ChordStack zones (sub-position within
  // a zone is meaningful inside STACK_DETUNE: it scales the spread).
  // mode_norm  (0..1) -> one of 6 ChordMode zones (mode affects only
  // stacks that contain 3rd/5th/7th/9th).
  void set_chord(float root_hz, float stack_norm, float mode_norm) {
    // Quantise mode.
    int mi = static_cast<int>(mode_norm * MODE_COUNT);
    if (mi < 0) mi = 0;
    if (mi >= MODE_COUNT) mi = MODE_COUNT - 1;
    chord_mode_ = static_cast<ChordMode>(mi);

    // Custom-width zones - unison gets a sliver, detune sweeps up to
    // 11 o'clock (~0.40) so the phat beating area still dominates the
    // bottom of the knob, then the four chord stacks share the rest
    // in equal-width steps of 0.15.
    //   0.00 .. 0.03  UNISON
    //   0.03 .. 0.40  DETUNE  (sub-pos scales spread)
    //   0.40 .. 0.55  +3rd
    //   0.55 .. 0.70  +5th
    //   0.70 .. 0.85  +7th
    //   0.85 .. 1.00  +9th
    float detune_t = 0.0f;
    if (stack_norm < 0.03f) {
      stack_ = STACK_UNISON;
    } else if (stack_norm < 0.40f) {
      stack_ = STACK_DETUNE;
      detune_t = (stack_norm - 0.03f) * (1.0f / 0.37f);
    } else if (stack_norm < 0.55f) {
      stack_ = STACK_3RD;
    } else if (stack_norm < 0.70f) {
      stack_ = STACK_5TH;
    } else if (stack_norm < 0.85f) {
      stack_ = STACK_7TH;
    } else {
      stack_ = STACK_9TH;
    }

    float ratios[kNumVoices];
    if (stack_ == STACK_UNISON) {
      ratios[0] = ratios[1] = ratios[2] = ratios[3] = 1.0f;
    } else if (stack_ == STACK_DETUNE) {
      if (detune_t > 1.0f) detune_t = 1.0f;
      // Spread max bumped (was 0.012 ≈ 21 cents) so the widened detune
      // zone reaches genuinely "beyond chord" territory at 12 o'clock -
      // ~40 cents between outer voices, into chorus/cluster range.
      const float spread = 0.024f * detune_t;
      ratios[0] = 1.000f;
      ratios[1] = 1.000f - spread * 0.5f;
      ratios[2] = 1.000f + spread * 0.5f;
      ratios[3] = 1.000f + spread;
    } else {
      // Stacks 2..5 use mode intervals (in semitones from root).
      // table[mode] = {third, fifth, seventh, ninth}
      static constexpr int kIntervals[MODE_COUNT][4] = {
        { 4, 7, 11, 14 },  // MAJOR
        { 3, 7, 10, 14 },  // MINOR
        { 4, 7, 10, 14 },  // DOM7
        { 3, 6,  9, 13 },  // DIM
        { 2, 7, 10, 14 },  // SUS2 (3rd replaced by 2nd)
        { 5, 7, 10, 14 },  // SUS4 (3rd replaced by 4th)
      };
      const int third   = kIntervals[mi][0];
      const int fifth   = kIntervals[mi][1];
      const int seventh = kIntervals[mi][2];
      const int ninth   = kIntervals[mi][3];

      int s[4] = {0, 0, 0, 0};
      switch (stack_) {
        case STACK_3RD:
          s[0] = 0;     s[1] = third;
          s[2] = 12;    s[3] = third + 12;
          break;
        case STACK_5TH:
          s[0] = 0;     s[1] = third;
          s[2] = fifth; s[3] = 12;
          break;
        case STACK_7TH:
          s[0] = 0;     s[1] = third;
          s[2] = fifth; s[3] = seventh;
          break;
        case STACK_9TH:
          // Drop the 5th in favour of the 9th - jazzier, more open.
          s[0] = 0;     s[1] = third;
          s[2] = seventh; s[3] = ninth;
          break;
        default:
          break;
      }
      for (int v = 0; v < kNumVoices; ++v) {
        ratios[v] = powf(2.0f, static_cast<float>(s[v]) * (1.0f / 12.0f));
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
  // sub-osc saw stack can mirror the chord knob (detune spread + chord
  // intervals) using the SAME math as the main bank - no duplicated
  // interval tables, no risk of drift between the two layers.
  inline const float* ratios() const { return ratios_; }

  inline ChordStack stack()      const { return stack_; }
  inline ChordMode  chord_mode() const { return chord_mode_; }
  // Old API name - returns the stack so existing UI LED code keeps
  // compiling without modification.
  inline VoicingZone zone()      const { return stack_; }

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
  KarplusVoice voices_[kNumVoices];
  float        sample_rate_ = 48000.0f;
  ChordStack   stack_       = STACK_UNISON;
  ChordMode    chord_mode_  = MODE_MAJOR;
  float        ratios_[kNumVoices] = {1.0f, 1.0f, 1.0f, 1.0f};

  DISALLOW_COPY_AND_ASSIGN(VoiceBank);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_VOICE_BANK_H_
