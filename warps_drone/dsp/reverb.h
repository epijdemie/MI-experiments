// Plate reverb - Griesinger topology (Dattorro paper): 4 input allpass
// diffusers, then a loop of two parallel (2 allpass + 1 delay) sections,
// cross-coupled for stereo. Modulated allpass and modulated long delay
// give built-in chorus/shimmer.
//
// Adapted from clouds/dsp/fx/reverb.h (Émilie Gillet, MIT). Differences:
//   - namespace warps_drone
//   - LFO rates re-derived at Init() from the actual sample rate
//     (clouds hardcoded 32 kHz; we run at 48 kHz)
//   - Slightly different default diffusion / lp values
//
// Memory: a single 32 KB uint16_t buffer (16384 samples, 12-bit compressed).
// Pass the buffer into Init(); the engine doesn't allocate.

#ifndef WARPS_DRONE_DSP_REVERB_H_
#define WARPS_DRONE_DSP_REVERB_H_

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"

#include "warps_drone/dsp/fx_engine.h"

namespace warps_drone {

class PlateReverb {
 public:
  static constexpr size_t kBufferSize = 16384;

  PlateReverb() { }
  ~PlateReverb() { }

  void Init(uint16_t* buffer, float sample_rate) {
    engine_.Init(buffer);
    engine_.SetLFOFrequency(LFO_1, 0.5f / sample_rate);
    engine_.SetLFOFrequency(LFO_2, 0.3f / sample_rate);
    lp_           = 0.7f;
    diffusion_    = 0.7f;
    amount_       = 0.5f;
    input_gain_   = 0.5f;
    reverb_time_  = 0.7f;
    lp_decay_1_   = 0.0f;
    lp_decay_2_   = 0.0f;
  }

  // Processes a stereo block in place. amount_ is wet/dry mix.
  // Returns the post-mix block via in_out, plus accumulates the running
  // mean-square of the "wet" signal so the caller can derive a feedback
  // send energy.
  void Process(FloatFrame* in_out, size_t size) {
    typedef E::Reserve<113,
      E::Reserve<162,
      E::Reserve<241,
      E::Reserve<399,
      E::Reserve<1653,
      E::Reserve<2038,
      E::Reserve<3411,
      E::Reserve<1913,
      E::Reserve<1663,
      E::Reserve<4782> > > > > > > > > > Memory;
    E::DelayLine<Memory, 0> ap1;
    E::DelayLine<Memory, 1> ap2;
    E::DelayLine<Memory, 2> ap3;
    E::DelayLine<Memory, 3> ap4;
    E::DelayLine<Memory, 4> dap1a;
    E::DelayLine<Memory, 5> dap1b;
    E::DelayLine<Memory, 6> del1;
    E::DelayLine<Memory, 7> dap2a;
    E::DelayLine<Memory, 8> dap2b;
    E::DelayLine<Memory, 9> del2;
    E::Context c;

    const float kap = diffusion_;
    const float klp = lp_;
    const float krt = reverb_time_;
    const float amount = amount_;
    const float gain = input_gain_;

    float lp_1 = lp_decay_1_;
    float lp_2 = lp_decay_2_;
    float wet_l_acc = 0.0f;
    float wet_r_acc = 0.0f;

    while (size--) {
      float wet;
      float apout = 0.0f;
      engine_.Start(&c);

      c.Interpolate(ap1, 10.0f, LFO_1, 60.0f, 1.0f);
      c.Write(ap1, 100, 0.0f);

      c.Read(in_out->l + in_out->r, gain);

      c.Read(ap1 TAIL, kap);
      c.WriteAllPass(ap1, -kap);
      c.Read(ap2 TAIL, kap);
      c.WriteAllPass(ap2, -kap);
      c.Read(ap3 TAIL, kap);
      c.WriteAllPass(ap3, -kap);
      c.Read(ap4 TAIL, kap);
      c.WriteAllPass(ap4, -kap);
      c.Write(apout);

      c.Load(apout);
      c.Interpolate(del2, 4680.0f, LFO_2, 100.0f, krt);
      c.Lp(lp_1, klp);
      c.Read(dap1a TAIL, -kap);
      c.WriteAllPass(dap1a, kap);
      c.Read(dap1b TAIL, kap);
      c.WriteAllPass(dap1b, -kap);
      c.Write(del1, 2.0f);
      c.Write(wet, 0.0f);

      wet_l_acc += wet * wet;
      in_out->l += (wet - in_out->l) * amount;

      c.Load(apout);
      c.Read(del1 TAIL, krt);
      c.Lp(lp_2, klp);
      c.Read(dap2a TAIL, kap);
      c.WriteAllPass(dap2a, -kap);
      c.Read(dap2b TAIL, -kap);
      c.WriteAllPass(dap2b, kap);
      c.Write(del2, 2.0f);
      c.Write(wet, 0.0f);

      wet_r_acc += wet * wet;
      in_out->r += (wet - in_out->r) * amount;

      ++in_out;
    }

    lp_decay_1_ = lp_1;
    lp_decay_2_ = lp_2;
    tail_energy_ = 0.5f * (wet_l_acc + wet_r_acc);
  }

  inline void set_amount    (float a)  { amount_      = a; }
  inline void set_input_gain(float g)  { input_gain_  = g; }
  inline void set_time      (float t)  { reverb_time_ = t; }
  inline void set_diffusion (float d)  { diffusion_   = d; }
  inline void set_lp        (float l)  { lp_          = l; }

  // Running mean-square of the wet signal across the last Process() call.
  // Caller uses this to drive a "reverb tail -> exciter" feedback path.
  inline float tail_energy() const { return tail_energy_; }

 private:
  typedef FxEngine<kBufferSize, FORMAT_12_BIT> E;
  E engine_;

  float amount_;
  float input_gain_;
  float reverb_time_;
  float diffusion_;
  float lp_;
  float lp_decay_1_;
  float lp_decay_2_;
  float tail_energy_ = 0.0f;

  DISALLOW_COPY_AND_ASSIGN(PlateReverb);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_REVERB_H_
