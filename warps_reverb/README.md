# WARPS REVERB

Alternative high-end reverb firmware for the Mutable Instruments **Warps**
module. Long sustained tails in the spirit of Strymon Starlab / Make Noise
Erbe-Verb, with octave-up shimmer and stereo ping-pong tremolo. Pure
reverb — no synthesis sources, no delay-line effects.

---

## Get it

Latest stable: **v1.0** ([release notes below](#release-notes))

[`warps_reverb_v1.0.wav`](builds/warps_reverb_v1.0.wav) | Hold the button while powering on, play the WAV into IN L from your phone or laptop

[`warps_reverb_v1.0.combo.bin`](builds/warps_reverb_v1.0.combo.bin) | JTAG-flash at `0x08000000`. Includes Émilie's QPSK WAV bootloader + this firmware

### How the WAV update works

1. Disconnect everything from IN L.
2. Power off the module.
3. **Hold the button** and power on. The bootloader is listening for an audio update.
4. Patch your laptop / phone headphone output into IN L.
5. Set output volume near max, disable any system EQ or "loudness" enhancement.
6. Play `warps_reverb_v1.0.wav` from start to finish.
7. The module re-flashes itself and reboots. New firmware running.

Common mistakes: cable not fully seated, headphone out too quiet, system
DSP compressing the signal. If the LED doesn't change within ~5 s of
playing audio, stop the WAV, check the cable and level, retry.

---

## Cheatsheet

The full knob reference lives in [`panel/cheatsheet.svg`](panel/cheatsheet.svg)
— print as a sticker overlay, or just keep open on your screen.

| Pot       | Unshifted   | Shifted (hold button)  |
|-----------|-------------|------------------------|
| ALGORITHM | **Decay**   | Pre-delay              |
| TIMBRE    | **Cutoff**  | Resonance              |
| LEVEL 1   | **Size**    | Spectral (shimmer)     |
| LEVEL 2   | **Mix**     | Low-cut                |

### What the knobs do

- **Decay**: CCW half (0–12 o'clock) holds tail length at "medium long"
  and adds stereo ping-pong tremolo on the wet — depth grows as you go
  further CCW. CW half (12–full) ramps loop gain up to unity for a
  smooth long-to-freeze tail. So 12 o'clock = clean medium tail,
  CCW = rhythmic chopping that bounces L↔R, CW = cathedral / freeze.
- **Cutoff / Resonance**: post-reverb biquad LP for shaping the wet
  tone, ~250 Hz at full CCW to ~16 kHz at full CW. Resonance dials in
  filter Q for synthy sweeps.
- **Size**: scales tank delay lengths. Small room → cathedral.
- **Mix**: dry/wet with equal-power crossfade. Top 5% is guaranteed pure
  wet, bottom 5% pure dry.
- **Pre-delay**: 0–50 ms front-of-chain delay AND tremolo speed.
  Short pre-delay = fast tremolo (~10 Hz), long pre-delay = slow swell
  (~0.5 Hz). The two move together — long space, slow rhythm.
- **Spectral**: octave-up granular shimmer. Reads from the longest tank
  line at 2× rate with triangle-windowed grain crossfade. A slow 0.13 Hz
  LFO modulates amplitude so the halo glimmers in and out over ~8 s.
- **Low-cut**: HP corner in the feedback loop, 5–200 Hz log. Kills sub
  buildup at high decay so the loop doesn't choke / breathe weirdly.

LED: dim white = shift active. Red = wet peak clipping warning.

### Audio I/O

- **IN L + IN R**: stereo audio input.
- **OUT L + OUT R**: stereo audio output.
- **DECAY CV**, **CUTOFF CV**, **SIZE CV**: bipolar ±5 V CV for each
  unshifted parameter.
- **MIX CV**: jack is present but firmware ignores it on this hardware
  (the jack normalization on the prototype unit was throwing the dry/wet
  balance off — disabled until properly calibrated).

---

## Topology

```
in_L,R ─► gain ─► pre-delay (0..50 ms) ─► input diffuser (6 series APs)
                                                       │
                                                       ▼
                                          FDN tank — 4 modulated lines
                                          per line:  AP-in-loop
                                                   → fixed-corner LP (~7 kHz)
                                                   → low-cut HP (knob)
                                                   → SmoothSat saturator
                                                   → Hadamard matrix mix
                                                   → write back
                                          ┌───── shimmer (octave-up grain) ──┘
                                                       │
                                          output pre-delay (40 ms)
                                          → output diffuser (2 APs / ch)
                                          → biquad LP (cutoff + Q knobs)
                                          → stereo ping-pong tremolo
                                          → equal-power dry/wet crossfade
                                                       │
                                                       ▼
                                                    out_L,R
```

Bounded-tail invariants:
- SmoothSat in every feedback line — loop gain → 1.0 asymptotes, never runs away.
- Tilt/LP/HP gains all ≤ 1 — no amplification in the loop.
- Matrix energy-preserving (Hadamard) — no inter-line gain buildup.
- Low-cut HP kills DC walk that would otherwise choke the saturator at high decay.

Hybrid memory layout: 32 KB float32 FxEngine buffer in CCM (pre-delay,
diffusers, in-loop APs, output diffuser), 76 KB FDN tank arrays in main
SRAM (4 lines, ~46–130 ms each at default Size), 15 KB output pre-delay.

---

## Build from source

```sh
cd /path/to/repo

TOOLCHAIN_PATH=$HOME/opt/arm-gnu-toolchain-13.3.rel1-darwin-arm64-arm-none-eabi/ \
  make -f warps/bootloader/makefile

TOOLCHAIN_PATH=$HOME/opt/arm-gnu-toolchain-13.3.rel1-darwin-arm64-arm-none-eabi/ \
  make -f warps_reverb/makefile build/warps_reverb/warps_reverb_bootloader_combo.bin

TOOLCHAIN_PATH=$HOME/opt/arm-gnu-toolchain-13.3.rel1-darwin-arm64-arm-none-eabi/ \
  make -f warps_reverb/makefile wav
```

Then either `./warps_reverb/flash_combo.sh` (JTAG, first-time install)
or use the resulting WAV (audio update, no JTAG needed).

See [`HANDOVER.md`](HANDOVER.md) for the engineering notes and
tweakpoints (where to poke if you want to push the character around).

---

## Release notes

### v1.0 — stable

- 4-line FDN reverb on Émilie's FxEngine + custom tank arrays for tail quality
- Octave-up granular shimmer with amplitude-modulated halo
- Stereo ping-pong tremolo on CCW decay, paired with pre-delay (rate)
- Post-reverb biquad LP with resonance for tail shaping
- 40 ms output pre-delay so the wet sits clearly behind the dry transient
- 6-stage input diffuser smears input transients before the tank
- Smoothstep × equal-power dry/wet crossfade
