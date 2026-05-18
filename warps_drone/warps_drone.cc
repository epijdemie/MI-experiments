// drone firmware for warps - entry point

#include <math.h>

#include "warps/drivers/codec.h"
#include "warps/drivers/system.h"
#include "warps/drivers/version.h"

#include "warps_drone/cv_scaler.h"
#include "warps_drone/settings.h"
#include "warps_drone/ui.h"
#include "warps_drone/dsp/drone.h"

using namespace warps_drone;
using namespace stmlib;

// 64k plate reverb buffer in ccm. NOLOAD, cleared in PlateReverb::Init
float reverb_buffer[PlateReverb::kBufferSize]
    __attribute__((section(".ccmdata")));

// overdrive: dc blocker + tone lpf state
float dist_dc_xl_ = 0.0f, dist_dc_yl_ = 0.0f;
float dist_dc_xr_ = 0.0f, dist_dc_yr_ = 0.0f;
float dist_tone_l_ = 0.0f, dist_tone_r_ = 0.0f;

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

  // IN L -> chord K-S exciter (when patched). IN R -> bass K-S exciter
  // (when patched). Normalization probe lives in cv_scaler — unpatched
  // means the internal pulse-osc continues to drive the network.
  for (size_t i = 0; i < n; ++i) {
    scratch[i].l = input[i].l * kIntToFloat;
    scratch[i].r = input[i].r * kIntToFloat;
  }

  drone.Process(scratch, n);

  // overdrive: drive | warmth (tube↔fuzz) | tone | bias.
  // dc-blocker before chain - reverb tail accumulates DC, would flat-
  // line saturator. hard bypass <0.5% to match soft-takeover dead zone
  const float dist_raw = drone.distortion_amount();
  const float dist     = dist_raw > 0.005f ? dist_raw : 0.0f;
  if (dist > 0.0f) {
    const float warmth   = drone.distortion_warmth();
    const float hardness = 1.0f - warmth;
    // drive ceiling: tube=1 -> 4×, fuzz=0 -> 12×
    const float drive    = 1.0f + dist * (11.0f - warmth * 8.0f);
    // asym bias for even harmonics; bias_offset re-centres post-sat
    const float bias        = drone.distortion_bias() * 0.4f;
    const float bias_offset = bias / sqrtf(1.0f + bias * bias);
    // tone: log 200Hz..18kHz, knob 1 = wide open
    const float tone_knob = drone.distortion_tone();
    const float fc = 200.0f * powf(90.0f, tone_knob);
    const float two_pi_inv_sr = 6.2831853f / kSampleRate;
    const float tone_a = 1.0f - expf(-two_pi_inv_sr * fc);
    for (size_t i = 0; i < n; ++i) {
      // dc block
      const float dl = scratch[i].l - dist_dc_xl_ + 0.9995f * dist_dc_yl_;
      const float dr = scratch[i].r - dist_dc_xr_ + 0.9995f * dist_dc_yr_;
      dist_dc_xl_ = scratch[i].l;  dist_dc_yl_ = dl;
      dist_dc_xr_ = scratch[i].r;  dist_dc_yr_ = dr;
      // pre-sat
      const float xl = dl * drive + bias;
      const float xr = dr * drive + bias;
      // saturator - smooth ↔ hard by warmth
      const float smooth_l = xl / sqrtf(1.0f + xl * xl);
      const float smooth_r = xr / sqrtf(1.0f + xr * xr);
      const float hard_l   = xl >  1.0f ?  1.0f : (xl < -1.0f ? -1.0f : xl);
      const float hard_r   = xr >  1.0f ?  1.0f : (xr < -1.0f ? -1.0f : xr);
      float satl = smooth_l + (hard_l - smooth_l) * hardness - bias_offset;
      float satr = smooth_r + (hard_r - smooth_r) * hardness - bias_offset;
      // tone lpf
      dist_tone_l_ += tone_a * (satl - dist_tone_l_);
      dist_tone_r_ += tone_a * (satr - dist_tone_r_);
      satl = dist_tone_l_;
      satr = dist_tone_r_;
      // dist-weighted crossfade into dry
      scratch[i].l += (satl - scratch[i].l) * dist;
      scratch[i].r += (satr - scratch[i].r) * dist;
    }
  }

  const float output_gain = drone.output_gain();
  const float* vinyl = drone.vinyl_buf();

  // crossfade SoftLimit out as dist rises - saturator already bounds.
  // Vinyl noise is summed AFTER all non-linear stages (SoftLimit + the
  // dist crossfade) — purely additive at the output.
  float peak = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    float l = scratch[i].l * output_gain;
    float r = scratch[i].r * output_gain;
    const float sl_l = SoftLimit(l);
    const float sl_r = SoftLimit(r);
    l = sl_l + (l - sl_l) * dist;
    r = sl_r + (r - sl_r) * dist;
    l += vinyl[i] * output_gain;
    r += vinyl[i] * output_gain;
    float a = l > 0 ? l : -l;
    float b = r > 0 ? r : -r;
    if (a > peak) peak = a;
    if (b > peak) peak = b;
    output[i].l = static_cast<int16_t>(
        Clip16(static_cast<int32_t>(l * kFloatToInt)));
    output[i].r = static_cast<int16_t>(
        Clip16(static_cast<int32_t>(r * kFloatToInt)));
  }
  ui.set_peak(peak);
}

void Init() {
  // sys.Init(true) — VTOR offset to APPLICATION_LARGE (0x08008000).
  // Émilie's QPSK WAV bootloader lives at 0x08000000 (sectors 0+1).
  sys.Init(true);
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
    // audio runs in codec isr. main loop owns the autosave window only
    cv_scaler.MaybeSave();
  }
}
