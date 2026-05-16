# Erbe-Verb-style firmware variant for the Mutable Instruments Warps module.
#
# Same MCU and codec as stock Warps (STM32F405RGT6 @ 168 MHz, WM8731), but the
# internal audio path runs at 48 kHz instead of 96 kHz to free up SRAM for the
# reverb delay buffers. The board is unmodified - this is a firmware-only swap.
#
# Upload via JTAG / ST-Link (the stmlib upload_combo_jtag_erase_first rule).
# The WAV-bootloader path is unused; image still links at 0x08004000 so the
# factory warps_bootloader stays usable for recovery if you keep it in flash.

# System specifications (mirrors warps/makefile)
F_CRYSTAL      = 8000000L
F_CPU          = 168000000L
SYSCLOCK       = SYSCLK_FREQ_168MHz
FAMILY         = f4xx

# Link the firmware at 0x08000000 (no bootloader). JTAG-flash directly to that
# address. To return to factory layout (firmware at 0x08008000 + factory
# bootloader at 0x08000000), uncomment both APPLICATION_LARGE and BOOTLOADER
# below.
# APPLICATION_LARGE = TRUE
# BOOTLOADER        = warps_bootloader

UPLOAD_COMMAND  = upload_combo_jtag_erase_first

TARGET         = warps_reverb
PACKAGES       = warps_reverb \
		warps_reverb/dsp \
		warps/drivers \
		stmlib/dsp \
		stmlib/utils \
		stmlib/system
# No RESOURCES line - we don't ship generated lookup tables yet.

TOOLCHAIN_PATH ?= /usr/local/arm-4.8.3/

include stmlib/makefile.inc

# Quiet modern GCC's complaints about old-style code in MI's bundled CMSIS:
#   - 'register' keyword (banned in C++17, used by core_cmFunc.h)
#   - implicit fallthrough, deprecated casts, etc. - add as encountered.
CPPFLAGS += -Wno-register -Wno-deprecated

# JTAG/ST-Link upload is provided by stmlib via UPLOAD_COMMAND above:
#   make upload   ->   upload_combo_jtag_erase_first rule (erase + write + verify)
