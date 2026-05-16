#!/usr/bin/env bash
# flash warps_reverb via olimex arm-usb-ocd-h, jtag.
# run from repo root after `TOOLCHAIN_PATH=... make -f warps_reverb/makefile`
set -euo pipefail

OCD="$HOME/opt/xpack-openocd-0.12.0-3/bin/openocd"
SCRIPTS="$HOME/opt/xpack-openocd-0.12.0-3/openocd/scripts"
BIN="build/warps_reverb/warps_reverb.bin"

ELF="build/warps_reverb/warps_reverb.elf"

if [[ ! -f "$ELF" ]]; then
  echo "no build artifact at $ELF - run make first" >&2
  exit 1
fi

# stmlib's `make all` leaves .bin stale - regenerate from fresh .elf
"$HOME/opt/arm-gnu-toolchain-13.3.rel1-darwin-arm64-arm-none-eabi/bin/arm-none-eabi-objcopy" \
  -O binary "$ELF" "$BIN"

"$OCD" \
  -s "$SCRIPTS" \
  -f interface/ftdi/olimex-arm-usb-ocd-h.cfg \
  -c "transport select jtag; adapter speed 1000" \
  -f target/stm32f4x.cfg \
  -c "program $BIN verify reset exit 0x08000000"
