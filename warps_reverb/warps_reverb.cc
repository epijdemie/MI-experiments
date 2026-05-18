// reverb firmware variant - entry point. 48k codec, routes through Reverb

#include "warps/drivers/codec.h"
#include "warps/drivers/system.h"
#include "warps/drivers/version.h"

#include "warps_reverb/cv_scaler.h"
#include "warps_reverb/dsp/reverb.h"
#include "warps_reverb/ui.h"

using namespace warps_reverb;
using namespace stmlib;

// 32k FxEngine buffer in CCM (single-cycle, no dma contention). float32
// for clean recirculation through the diffusers and saturator.
// tank delay lines live inside the Reverb class (main sram, also float32).
float reverb_buffer[Reverb::kBufferSize]
    __attribute__((section(".ccmdata")));

warps::Codec   codec;
warps::System  sys;
warps::Version version;

Reverb   reverb;
CvScaler cv_scaler;
Ui       ui;

int __errno;

extern "C" {
void NMI_Handler() { }
void HardFault_Handler() { while (1); }
void MemManage_Handler() { while (1); }
void BusFault_Handler() { while (1); }
void UsageFault_Handler() { while (1); }
void SVC_Handler() { }
void DebugMon_Handler() { }
void PendSV_Handler() { }
}

constexpr float kSampleRate = 48000.0f;
constexpr size_t kBlockSize = 32;
constexpr float kIntToFloat = 1.0f / 32768.0f;
constexpr float kFloatToInt = 32767.0f;

FloatFrame scratch[kBlockSize];

void FillBuffer(warps::Codec::Frame* input,
                warps::Codec::Frame* output,
                size_t n) {
  cv_scaler.DetectAudioNormalization(input, n);
  cv_scaler.Read(reverb.mutable_parameters(), ui.shifted());
  ui.Poll();

  for (size_t i = 0; i < n; ++i) {
    scratch[i].l = input[i].l * kIntToFloat;
    scratch[i].r = input[i].r * kIntToFloat;
  }

  reverb.Tick();
  reverb.Process(scratch, n);

  for (size_t i = 0; i < n; ++i) {
    output[i].l = static_cast<int16_t>(
        stmlib::SoftLimit(scratch[i].l) * kFloatToInt);
    output[i].r = static_cast<int16_t>(
        stmlib::SoftLimit(scratch[i].r) * kFloatToInt);
  }
}

void Init() {
  // sys.Init(true) — VTOR offset to APPLICATION_LARGE (0x08008000).
  // Émilie's QPSK WAV bootloader lives at 0x08000000 (sectors 0+1).
  sys.Init(true);
  version.Init();

  reverb.Init(reverb_buffer, kSampleRate);
  cv_scaler.Init();
  ui.Init(&cv_scaler, &reverb, reverb.mutable_parameters());

  if (!codec.Init(!version.revised(), static_cast<int>(kSampleRate))) {
    while (1);
  }
  if (!codec.Start(kBlockSize, &FillBuffer)) {
    while (1);
  }
  codec.set_line_input_gain(24);
}

int main(void) {
  Init();
  while (1) {
    // all work runs in the codec isr
  }
}
