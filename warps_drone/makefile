# warps drone firmware. stm32f405 @ 168MHz, WM8731, 48k/float.
# links at 0x08000000 (no bootloader). upload via JTAG / ST-Link

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

# Quiet GCC about stmlib's older C++ style (register kw, deprecated casts)
CPPFLAGS += -Wno-register -Wno-deprecated
