// 4×4 mixing matrix interpolated between identity and Hadamard.
//
// Erbe's central trick: at α = 0 the four FDN branches are independent (four
// parallel comb filters); at α = 1 they're fully cross-coupled (classic FDN
// reverb). A continuously-swept α gives the "opens up" feeling of his
// Size/Decay interaction.
//
//   M(α) = (1 - α) · I  +  α · (1/2) · H
//
// where H is the normalized 4×4 Hadamard matrix:
//
//   1/2 · [ +1 +1 +1 +1 ;
//           +1 -1 +1 -1 ;
//           +1 +1 -1 -1 ;
//           +1 -1 -1 +1 ]
//
// Pre-multiplied by (1/2) it is orthogonal, so feedback gain ≤ 1 keeps the
// network unconditionally stable in the α = 1 limit. Stability at α < 1 is
// guaranteed by the per-branch LPF + saturator on the loop.

#ifndef WARPS_REVERB_DSP_MIXING_MATRIX_H_
#define WARPS_REVERB_DSP_MIXING_MATRIX_H_

#include "stmlib/stmlib.h"

namespace warps_reverb {

class MixingMatrix {
 public:
  MixingMatrix() : alpha_(0.0f) { }
  ~MixingMatrix() { }

  inline void set_alpha(float a) { alpha_ = a; }

  // Apply M(alpha) to a 4-vector in-place.
  // out[i] = sum_j M_ij(α) * in[j]
  inline void Apply(const float in[4], float out[4]) const {
    const float a = alpha_;
    const float h = 0.5f * a;
    const float id = 1.0f - a;
    // Identity term contributes id * in[i] to out[i].
    // Hadamard term contributes h * (signed sum) to each row.
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
