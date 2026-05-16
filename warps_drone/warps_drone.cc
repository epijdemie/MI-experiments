// Drone-machine firmware for Warps. Entry point.

#include "warps/drivers/codec.h"
#include "warps/drivers/system.h"
#include "warps/drivers/version.h"

#include "warps_drone/cv_scaler.h"
#include "warps_drone/settings.h"
#include "warps_drone/ui.h"
#include "warps_drone/dsp/drone.h"

using namespace warps_drone;
using namespace stmlib;

// 32 KB plate reverb buffer in main SRAM (.bss).
uint16_t reverb_buffer[PlateReverb::kBufferSize];

warps::Codec   codec;
warps::System  sys;
warps::Version version;

Settings  settings;
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
  cv_scaler.Read(drone.mutable_parameters(), ui.page(), ui.shifted());
  ui.Poll();

  // Pass the codec input straight to Drone:
  //   scratch[i].l = audio excitement (mixed into the KS loop)
  //   scratch[i].r = envelope follower input (trigger/gate/LFO)
  for (size_t i = 0; i < n; ++i) {
    scratch[i].l = input[i].l * kIntToFloat;
    scratch[i].r = input[i].r * kIntToFloat;
  }

  drone.Process(scratch, n);

  // External trigger still fires Pluck() inside Drone (audio in R).
  // We used to flash the OSC LED on each rising edge - that's been
  // dropped per user request; the steady page color stays untouched.
  (void)drone.TakeTriggerFlag();

  // Output-stage gain, now driven by the PERFORMANCE-page LVL1 shift
  // knob (drone.output_gain() maps 0..1 -> 0.5×..6×). SoftLimit catches
  // peaks; very high gain settings will harmonically clip the output.
  const float output_gain = drone.output_gain();

  float peak = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    float l = scratch[i].l * output_gain;
    float r = scratch[i].r * output_gain;
    float a = l > 0 ? l : -l;
    float b = r > 0 ? r : -r;
    if (a > peak) peak = a;
    if (b > peak) peak = b;
    output[i].l = static_cast<int16_t>(SoftLimit(l) * kFloatToInt);
    output[i].r = static_cast<int16_t>(SoftLimit(r) * kFloatToInt);
  }
  ui.set_peak(peak);
}

void Init() {
  // sys.Init(false) -> don't offset VTOR. We link at 0x08000000, no bootloader.
  sys.Init(false);
  version.Init();

  settings.Init();
  drone.Init(reverb_buffer, kSampleRate);
  cv_scaler.Init(&settings);
  ui.Init(&drone, drone.mutable_parameters(), &cv_scaler, &settings);

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
    // Audio work happens entirely in the codec ISR. The only thing
    // the main loop owns is the autosave window: when CvScaler sees
    // ~10 s of knob idleness with dirty state, it stages a flash
    // write here. Writes block the audio briefly (~one block) but
    // happen seldom (typically once per editing session).
    cv_scaler.MaybeSave();
  }
}
