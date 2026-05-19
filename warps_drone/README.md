# WARPS DRONES

Alternative drone-machine firmware for the Mutable Instruments **Warps**
module.

Two voices: a 3-string Karplus-Strong **chord** with modal harmonics and
a long plate reverb tail, plus an independent K-S **bass** that bypasses the reverb. Distortion and noise end of chain to increase thickness.


---

## Get it

Latest stable: **v1.2** ([release notes below](#release-notes))


[`warps_drones_v1.2.wav`](builds/warps_drones_v1.2.wav) | Hold the button while powering on, play the WAV into IN L from your phone or laptop

[`warps_drones_v1.2.combo.bin`](builds/warps_drones_v1.2.combo.bin) | JTAG-flash at `0x08000000`. Includes Émilie's QPSK WAV bootloader + this firmware


### How the WAV update works

1. Disconnect everything from IN L.
2. Power off the module.
3. **Hold the button** and power on. The bootloader is listening for an audio update.
4. Patch your laptop / phone headphone output into IN L.
5. Set output volume near max, disable any system EQ or "loudness" enhancement.
6. Play `warps_drones_v1.2.wav` from start to finish.
7. The module re-flashes itself and reboots. New firmware running.

Common mistakes: cable not fully seated, headphone out too quiet,
system DSP compressing the signal. If the LED doesn't change within
~5 s of playing audio, stop the WAV, check the cable and level, retry.

---

## Cheatsheet

The full knob reference lives in [`panel/drone_cheatsheet.svg`](panel/drone_cheatsheet.svg)
a one-page summary of all four pages, chord banks, bass note tables, and the calibration ritual.

Quick orientation:

- **Tap the button** to cycle through 4 pages: **PERFORMANCE → KARPLUS → REVERB → OVERDRIVE**.
- **Hold the button** while turning a knob to access its second function (the "shifted" layer).
- OSC LED tells you which page you're on (off / green / orange / red).
- Main RGB LED shows what colour your chord is.

### The four pages, in one paragraph each

**PERFORMANCE** (LED off) is where you play. The BIG knob is your chord
— twist for density (root → triad → 7th → 9th → extension), hold for
the chord bank (Detune / Minor / Major / Dom7 / Drone-Pad / Weird). The
SMALL knob is the bass note (7 zones across two octaves, picking out
chord-tones of the active bank). LVL1 is the chord's octave. LVL2 is
the master LPF cutoff.

**KARPLUS** (LED green) is where you sculpt the source. Damping (bright
↔ sustained), bass weight (adds a saw layer to the bass for fatness),
KS LPF, noise floor. Shift for harmonics (modal-bank mix), bass drive
(adds growl and lengthens the drone tail), resonance Q, noise color
(pink ↔ white).

**REVERB** (LED orange) is the wash. Plate-style Dattorro running on
the chord only — bass and vinyl noise sit on top, dry. Size / diffusion
/ HF damp / wet. Shift for predelay, smear (tail feeds back into the
strings), input drive, chorus rate.

**OVERDRIVE** (LED red) is end-of-chain. Drive (tube ↔ fuzz), tone,
vinyl noise level. Shift for bias (asymmetric clip, adds even
harmonics) and vinyl colour (warm rumble at CCW, surface hiss at CW,
crackle clicks layered in below mid-knob).

### Breathing LEDs

REVERB and OVERDRIVE LEDs **breathe** gently when the page is bypassed
— REVERB wet below 0.5 %, or drive below 0.5 %. Solid full-bright =
engaged. The whole stage hard-bypasses below the threshold so the dry
signal passes through uncoloured.

Pull the wet (REVERB LVL2 unshifted) below ~1 % to mute the reverb.
Pull drive (OVERDRIVE BIG unshifted) below ~1 % to mute the overdrive
section. Vinyl noise is gated by the overdrive page being engaged, so
muting drive also mutes the noise carpet.

---

## V/oct calibration

My default calibration is roughly right for most modules but not
perfect. If you care about playing the chord from a keyboard or a
sequencer, you'll want to calibrate.

1. **Tap** until you're on the OVERDRIVE page (LED red).
2. Make sure your hands are off all the knobs.
3. **Hold the button.** Wait.
4. At about 8 seconds the OSC LED will start a fast yellow pulse — that's the warning. Keep holding.
5. At 10 seconds it switches to a **slow green blink**: "patch +1 V into PITCH and press the button".
6. Patch +1 V into the PITCH jack and press → it goes **slow yellow blink**: "patch +3 V and press".
7. Patch +3 V (2 octaves up from +1 V) and press → **solid green** for a second = saved. **Solid red** = something was off; retry.

If you touch a knob during the hold, the timer aborts (release and re-press to retry). The whole gesture is designed to be impossible to do by accident.
Calibration data lives in flash sector 5 and **survives firmware updates** — once you've calibrated, future WAV updates won't wipe it.

---

## Release notes

### v1.2

- Émilie's QPSK WAV bootloader integrated, update via audio, no JTAG required.
- V/oct calibration moved from sector 2 to sector 5 so it survives firmware updates.
- New in-app calibration ritual (OVERDRIVE page + 10 s hold). Boot-hold now belongs to the bootloader.
- Pre-cal V/oct defaults baked in from a calibrated module, usable tracking out of the box.
- First-boot defaults baked in from a working playing setup, MINOR chord, warm reverb, vinyl bed — instead of zero/silence.

### v1.1

- Bass voice (K-S string) added as a 4th independent voice with its own pitch, octave, glide, weight, drive, and volume.
- Bass note quantises to chord tones of the active bank across 7 zones / 2 octaves.
- Chord side-chain duck on every bass-note change, scaled by bass volume.
- Vinyl noise + crackle bed, post-reverb, with warm-to-bright color sweep.
- KARPLUS SMALL column dedicated to bass tone (weight + drive).

### Earlier

See `git log` for the full history.

---

*by epijdemie*
