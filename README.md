# Mutable Mutations

Personal alt-firmware fork of [Mutable Instruments](https://pichenettes.github.io/mutable-instruments-documentation/) Eurorack modules. Focused on full firmware replacements rather than mods.

## Active projects

| Module    | Replacement firmware                            | Status             |
|-----------|-------------------------------------------------|--------------------|
| **Warps** | [`warps_drone/`](warps_drone/) — drone machine: 3-string Karplus chord + modal harmonics + plate reverb + overdrive + vinyl noise | v1.2 stable        |
| **Warps** | [`warps_reverb/`](warps_reverb/) — clouds-engine FDN reverb on Warps hardware | in progress |

Both target stock Warps hardware (STM32F405, WM8731 codec, no mods).

## Repo layout

Only what's actively touched or required to compile is tracked:

- `warps_drone/`, `warps_reverb/` — the projects
- `warps/drivers/`, `warps/bootloader/`, `warps/meter.h` — hardware drivers + Émilie's QPSK WAV bootloader
- `stmlib/` (submodule), `stm_audio_bootloader/` (submodule) — shared STM32 support
- `clouds/dsp/fx/fx_engine.h` — single header used by `warps_reverb`

Everything else from the upstream fork (other modules, AVR toolchain) is git-ignored locally. To start working on one: `git add -f <dir>` and remove the entry from `.gitignore`.

## License

Code (STM32F projects): MIT.

Hardware (when relevant): cc-by-sa-3.0.

Upstream code and design by Émilie Gillet (emilie.o.gillet@gmail.com).

## Derivative-works notice

Per the upstream guideline: **"Mutable Instruments" is a registered trademark and must not be used in derivative works.** This fork is named *Mutable Mutations* (the alt-firmware project), and each module's replacement firmware uses a distinct name (`warps_drone`, `warps_reverb`) to avoid confusion with the stock Warps firmware.
