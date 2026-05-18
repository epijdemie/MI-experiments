# warps drone firmware. stm32f405 @ 168MHz, WM8731, 48k/float.
# links at 0x08008000 (app start), with Émilie's QPSK WAV bootloader at
# 0x08000000 (sectors 0+1, 32 KB). Build the bootloader once via
#   make -f warps/bootloader/makefile
# then this makefile produces the app + combo (bootloader + app) for
# first-time JTAG install, and a wav: target encodes the app for
# end-user updates through the audio input.

F_CRYSTAL      = 8000000L
F_CPU          = 168000000L
SYSCLOCK       = SYSCLK_FREQ_168MHz
FAMILY         = f4xx

APPLICATION_LARGE = TRUE
BOOTLOADER        = warps_bootloader

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

# Encode the app .bin as a QPSK WAV so end users can update via IN L
# without needing JTAG. Matches Émilie's warps WAV bootloader format
# (48 kHz / 12 kbps / 6 kHz carrier / 256-byte packets).
wav: $(TARGET_BIN)
	PYTHONPATH=. python3 stm_audio_bootloader/qpsk/encoder.py \
		-t stm32f4 -s 48000 -b 12000 -c 6000 -p 256 \
		$(TARGET_BIN)
