#!/usr/bin/env bash
# Flash the combo (bootloader + app) for a first-time install via
# Olimex ARM-USB-OCD-H. Writes to 0x08000000 — bootloader claims
# sectors 0+1 (32 KB), app continues from 0x08008000.
#
# Run from repo root after BOTH:
#   TOOLCHAIN_PATH=... make -f warps/bootloader/makefile
#   TOOLCHAIN_PATH=... make -f warps_drone/makefile build/warps_drone/warps_drone_bootloader_combo.bin
set -euo pipefail

OCD="$HOME/opt/xpack-openocd-0.12.0-3/bin/openocd"
SCRIPTS="$HOME/opt/xpack-openocd-0.12.0-3/openocd/scripts"
COMBO="build/warps_drone/warps_drone_bootloader_combo.bin"

if [[ ! -f "$COMBO" ]]; then
  echo "no combo binary at $COMBO" >&2
  echo "run: make -f warps_drone/makefile $COMBO" >&2
  exit 1
fi

"$OCD" \
  -s "$SCRIPTS" \
  -f interface/ftdi/olimex-arm-usb-ocd-h.cfg \
  -c "transport select jtag; adapter speed 1000" \
  -f target/stm32f4x.cfg \
  -c "program $COMBO verify reset exit 0x08000000"
