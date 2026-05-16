// plate reverb - griesinger / dattorro topology.
// adapted from clouds/dsp/fx/reverb.h (émilie gillet, MIT). LFO rates
// re-derived from sample rate (clouds was 32k, here 48k).
// memory: 32k uint16 buffer (16384 × 12-bit compressed)

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

  void Init(float* buffer, float sample_rate) {
    engine_.Init(buffer);
    sample_rate_  = sample_rate;
    lp_           = 0.7f;
    diffusion_    = 0.7f;
    amount_       = 0.5f;
    input_gain_   = 0.5f;
    reverb_time_  = 0.7f;
    shimmer_depth_ = 1.0f;
    shimmer_rate_  = 1.0f;
    lp_decay_1_   = 0.0f;
    lp_decay_2_   = 0.0f;
    lp_decay_1b_  = 0.0f;
    lp_decay_2b_  = 0.0f;
    UpdateLfos();
  }

  // stereo block in place. accumulates wet RMS for fb send energy
  void Process(FloatFrame* in_out, size_t size) {
    // 16384 samp budget. 4 input AP + 2×6 loop AP + 2 combs (= 16 AP).
    // all prime lengths
    typedef E::Reserve<113,
      E::Reserve<162,
      E::Reserve<241,
      E::Reserve<397,
      E::Reserve<311,
      E::Reserve<547,
      E::Reserve<829,
      E::Reserve<1607,
      E::Reserve<2017,
      E::Reserve<1499,
      E::Reserve<379,
      E::Reserve<631,
      E::Reserve<919,
      E::Reserve<1663,
      E::Reserve<1907,
      E::Reserve<2069> > > > > > > > > > > > > > > > Memory;
    E::DelayLine<Memory, 0>  ap1;
    E::DelayLine<Memory, 1>  ap2;
    E::DelayLine<Memory, 2>  ap3;
    E::DelayLine<Memory, 3>  ap4;
    E::DelayLine<Memory, 4>  dap1a;
    E::DelayLine<Memory, 5>  dap1b;
    E::DelayLine<Memory, 6>  dap1c;
    E::DelayLine<Memory, 7>  dap1d;
    E::DelayLine<Memory, 8>  dap1e;
    E::DelayLine<Memory, 9>  del1;
    E::DelayLine<Memory, 10> dap2a;
    E::DelayLine<Memory, 11> dap2b;
    E::DelayLine<Memory, 12> dap2c;
    E::DelayLine<Memory, 13> dap2d;
    E::DelayLine<Memory, 14> dap2e;
    E::DelayLine<Memory, 15> del2;
    E::Context c;

    // slew diffusion (~33ms TC) to keep AP coefs stable across knob jumps
    diffusion_smooth_ += 0.02f * (diffusion_ - diffusion_smooth_);
    const float kap  = diffusion_smooth_;
    const float klp  = lp_;
    // 2nd cascaded 1-pole, gentler - pair = 12dB/oct rolloff
    const float klp2 = klp * 0.6f;
    const float krt  = reverb_time_;
    const float amount = amount_;
    const float gain = input_gain_;

    float lp_1  = lp_decay_1_;
    float lp_2  = lp_decay_2_;
    float lp_1b = lp_decay_1b_;
    float lp_2b = lp_decay_2b_;
    float wet_l_acc = 0.0f;
    float wet_r_acc = 0.0f;

    while (size--) {
      float wet;
      float apout = 0.0f;
      engine_.Start(&c);

      c.Interpolate(ap1, 10.0f, LFO_1, 60.0f * shimmer_depth_, 1.0f);
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
      // del2 length is now 2069; mod read max = 2000 + 40 = 2040 ≤ 2069.
      // del2 = 2069; max mod read 2000+40 ≤ 2069
      c.Interpolate(del2, 2000.0f, LFO_2, 40.0f * shimmer_depth_, krt);
      c.Lp(lp_1, klp);
      c.Lp(lp_1b, klp2);
      c.Read(dap1a TAIL, -kap);
      c.WriteAllPass(dap1a, kap);
      c.Read(dap1b TAIL, kap);
      c.WriteAllPass(dap1b, -kap);
      c.Read(dap1c TAIL, -kap);
      c.WriteAllPass(dap1c, kap);
      c.Read(dap1d TAIL, kap);
      c.WriteAllPass(dap1d, -kap);
      c.Read(dap1e TAIL, -kap);
      c.WriteAllPass(dap1e, kap);
      c.Write(del1, 2.0f);
      c.Write(wet, 0.0f);

      wet_l_acc += wet * wet;
      in_out->l += (wet - in_out->l) * amount;

      c.Load(apout);
      c.Read(del1 TAIL, krt);
      c.Lp(lp_2, klp);
      c.Lp(lp_2b, klp2);
      c.Read(dap2a TAIL, kap);
      c.WriteAllPass(dap2a, -kap);
      c.Read(dap2b TAIL, -kap);
      c.WriteAllPass(dap2b, kap);
      c.Read(dap2c TAIL, kap);
      c.WriteAllPass(dap2c, -kap);
      c.Read(dap2d TAIL, -kap);
      c.WriteAllPass(dap2d, kap);
      c.Read(dap2e TAIL, kap);
      c.WriteAllPass(dap2e, -kap);
      c.Write(del2, 2.0f);
      c.Write(wet, 0.0f);

      wet_r_acc += wet * wet;
      in_out->r += (wet - in_out->r) * amount;

      ++in_out;
    }

    lp_decay_1_  = lp_1;
    lp_decay_2_  = lp_2;
    lp_decay_1b_ = lp_1b;
    lp_decay_2b_ = lp_2b;
    tail_energy_ = 0.5f * (wet_l_acc + wet_r_acc);
  }

  inline void set_amount    (float a)  { amount_      = a; }
  inline void set_input_gain(float g)  { input_gain_  = g; }
  inline void set_time      (float t)  { reverb_time_ = t; }
  inline void set_diffusion (float d)  { diffusion_   = d; }
  inline void set_lp        (float l)  { lp_          = l; }

  inline void set_shimmer_depth(float d) { shimmer_depth_ = d; }
  inline void set_shimmer_rate (float r) {
    if (r < 0.05f) r = 0.05f;
    if (r != shimmer_rate_) {
      shimmer_rate_ = r;
      UpdateLfos();
    }
  }

  // last-block wet RMS - drives reverb->exciter feedback
  inline float tail_energy() const { return tail_energy_; }

 private:
  void UpdateLfos() {
    engine_.SetLFOFrequency(LFO_1, (0.5f * shimmer_rate_) / sample_rate_);
    engine_.SetLFOFrequency(LFO_2, (0.3f * shimmer_rate_) / sample_rate_);
  }

  typedef FxEngine<kBufferSize, FORMAT_32_BIT> E;
  E engine_;

  float sample_rate_;
  float amount_;
  float input_gain_;
  float reverb_time_;
  float diffusion_;
  float lp_;
  float shimmer_depth_;
  float shimmer_rate_;
  float lp_decay_1_;
  float lp_decay_2_;
  float lp_decay_1b_;
  float lp_decay_2b_;
  float diffusion_smooth_ = 0.0f;
  float tail_energy_ = 0.0f;

  DISALLOW_COPY_AND_ASSIGN(PlateReverb);
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_REVERB_H_
