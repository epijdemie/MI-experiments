// 4×4 mixing matrix, M(α) = (1-α)·I + α·(½)·H.
// α=0: 4 independent combs. α=1: fully cross-coupled (orthogonal hadamard)

#ifndef WARPS_REVERB_DSP_MIXING_MATRIX_H_
#define WARPS_REVERB_DSP_MIXING_MATRIX_H_

#include "stmlib/stmlib.h"

namespace warps_reverb {

class MixingMatrix {
 public:
  MixingMatrix() : alpha_(0.0f) { }
  ~MixingMatrix() { }

  inline void set_alpha(float a) { alpha_ = a; }

  // out[i] = Σⱼ M_ij(α) · in[j]
  inline void Apply(const float in[4], float out[4]) const {
    const float a = alpha_;
    const float h = 0.5f * a;
    const float id = 1.0f - a;
    const float s0 = in[0] + in[1] + in[2] + in[3];
    const float s1 = in[0] - in[1] + in[2] - in[3];
    const float s2 = in[0] + in[1] - in[2] - in[3];
    const float s3 = in[0] - in[1] - in[2] + in[3];
    out[0] = id * in[0] + h * s0;
    out[1] = id * in[1] + h * s1;
    out[2] = id * in[2] + h * s2;
    out[3] = id * in[3] + h * s3;
  }

 private:
  float alpha_;

  DISALLOW_COPY_AND_ASSIGN(MixingMatrix);
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_MIXING_MATRIX_H_
