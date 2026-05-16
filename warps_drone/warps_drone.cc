// Drone-machine firmware for Warps. Entry point.

#include "warps/drivers/codec.h"
#include "warps/drivers/system.h"
#include "warps/drivers/version.h"

#include "warps_drone/cv_scaler.h"
#include "warps_drone/ui.h"
#include "warps_drone/dsp/drone.h"

using namespace warps_drone;
using namespace stmlib;

// 32 KB plate reverb buffer in main SRAM (.bss).
uint16_t reverb_buffer[PlateReverb::kBufferSize];

warps::Codec   codec;
warps::System  sys;
warps::Version version;

Drone     drone;
CvScaler  cv_scaler;
Ui        ui;

int __errno;

extern "C" {
void NMI_Handler()       { }
void HardFault_Handler() { while (1); }
void MemManage_Handler() { while (1); }
void BusFault_Handler()  { while (1); }
void UsageFault_Handler(){ while (1); }
void SVC_Handler()       { }
void DebugMon_Handler()  { }
void PendSV_Handler()    { }
}

constexpr float  kSampleRate = 48000.0f;
constexpr size_t kBlockSize  = 32;
constexpr float  kIntToFloat = 1.0f / 32768.0f;
constexpr float  kFloatToInt = 32767.0f;

FloatFrame scratch[kBlockSize];

void FillBuffer(warps::Codec::Frame* input,
                warps::Codec::Frame* output,
                size_t n) {
  cv_scaler.DetectAudioNormalization(input, n);
  cv_scaler.Read(drone.mutable_parameters(), ui.shifted());
  ui.Poll();

  // Pass the codec input straight to Drone:
  //   scratch[i].l = audio excitement (mixed into the KS loop)
  //   scratch[i].r = envelope follower input (trigger/gate/LFO)
  for (size_t i = 0; i < n; ++i) {
    scratch[i].l = input[i].l * kIntToFloat;
    scratch[i].r = input[i].r * kIntToFloat;
  }

  drone.Process(scratch, n);

  if (drone.TakeTriggerFlag()) ui.notify_trigger();

  // Output-stage gain. The drone's natural RMS sits very low (~-28 dBFS
  // steady-state) - this brings it up so the steady drone is audibly
  // dominant, with the SoftLimit catching transient peaks (plucks).
  constexpr float kOutputGain = 3.0f;

  float peak = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    float l = scratch[i].l * kOutputGain;
    float r = scratch[i].r * kOutputGain;
    float a = l > 0 ? l : -l;
    float b = r > 0 ? r : -r;
    if (a > peak) peak = a;
    if (b > peak) peak = b;
    output[i].l = static_cast<int16_t>(SoftLimit(l) * kFloatToInt);
    output[i].r = static_cast<int16_t>(SoftLimit(r) * kFloatToInt);
  }
  ui.set_peak(peak);
  ui.set_send_meter(drone.reverb_send());
}

void Init() {
  // sys.Init(false) -> don't offset VTOR. We link at 0x08000000, no bootloader.
  sys.Init(false);
  version.Init();

  drone.Init(reverb_buffer, kSampleRate);
  cv_scaler.Init();
  ui.Init(&drone, drone.mutable_parameters());

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
    // All work runs in the codec ISR.
  }
}
