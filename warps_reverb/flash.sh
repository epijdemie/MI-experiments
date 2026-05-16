#!/usr/bin/env bash
# Flash warps_reverb to a Warps module via Olimex ARM-USB-OCD-H over JTAG.
# Run from repo root after `TOOLCHAIN_PATH=... make -f warps_reverb/makefile`.
set -euo pipefail

OCD="$HOME/opt/xpack-openocd-0.12.0-3/bin/openocd"
SCRIPTS="$HOME/opt/xpack-openocd-0.12.0-3/openocd/scripts"
BIN="build/warps_reverb/warps_reverb.bin"

ELF="build/warps_reverb/warps_reverb.elf"

if [[ ! -f "$ELF" ]]; then
  echo "no build artifact at $ELF - run make first" >&2
  exit 1
fi

# Stmlib's `make all` only builds the .hex, leaving the .bin stale from an
# older build. Regenerate the .bin from the current .elf before flashing.
"$HOME/opt/arm-gnu-toolchain-13.3.rel1-darwin-arm64-arm-none-eabi/bin/arm-none-eabi-objcopy" \
  -O binary "$ELF" "$BIN"

"$OCD" \
  -s "$SCRIPTS" \
  -f interface/ftdi/olimex-arm-usb-ocd-h.cfg \
  -c "transport select jtag; adapter speed 1000" \
  -f target/stm32f4x.cfg \
  -c "program $BIN verify reset exit 0x08000000"
