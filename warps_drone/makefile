# Drone-machine firmware for the Mutable Instruments Warps module.
#
# Topology: 4-voice Karplus-Strong chord -> modal resonator (parallel) ->
# SVF filter -> tape delay -> Clouds plate reverb, with reverb tail feeding
# back into the KS exciter to drive long evolving swells.
#
# Built on the same hardware as stock Warps (STM32F405RGT6 @ 168 MHz,
# WM8731). Audio path runs at 48 kHz, int16 -> float internally.
#
# Links at 0x08000000 (no bootloader). Upload via JTAG / ST-Link.

F_CRYSTAL      = 8000000L
F_CPU          = 168000000L
SYSCLOCK       = SYSCLK_FREQ_168MHz
FAMILY         = f4xx

UPLOAD_COMMAND  = upload_combo_jtag_erase_first

TARGET         = warps_drone
PACKAGES       = warps_drone \
		warps_drone/dsp \
		warps/drivers \
		stmlib/dsp \
		stmlib/utils \
		stmlib/system

TOOLCHAIN_PATH ?= /usr/local/arm-4.8.3/

include stmlib/makefile.inc

# Quiet GCC about stmlib's older C++ style (register kw, deprecated casts).
CPPFLAGS += -Wno-register -Wno-deprecated
