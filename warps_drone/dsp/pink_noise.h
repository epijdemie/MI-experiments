// pink (1/f) noise - paul kellet IIR. 6 cascaded 1-poles, -3dB/oct.
// out ~±1 (peaks ~±1.1)

#ifndef WARPS_DRONE_DSP_PINK_NOISE_H_
#define WARPS_DRONE_DSP_PINK_NOISE_H_

#include "stmlib/utils/random.h"

namespace warps_drone {

class PinkNoise {
 public:
  void Init() {
    b0_ = b1_ = b2_ = b3_ = b4_ = b5_ = b6_ = 0.0f;
  }

  inline float Next() {
    const float white = 2.0f * stmlib::Random::GetFloat() - 1.0f;
    b0_ = 0.99886f * b0_ + white * 0.0555179f;
    b1_ = 0.99332f * b1_ + white * 0.0750759f;
    b2_ = 0.96900f * b2_ + white * 0.1538520f;
    b3_ = 0.86650f * b3_ + white * 0.3104856f;
    b4_ = 0.55000f * b4_ + white * 0.5329522f;
    b5_ = -0.7616f * b5_ - white * 0.0168980f;
    const float pink = b0_ + b1_ + b2_ + b3_ + b4_ + b5_ + b6_
                     + white * 0.5362f;
    b6_ = white * 0.115926f;
    return pink * 0.11f;   // normalise to ~±1
  }

 private:
  float b0_, b1_, b2_, b3_, b4_, b5_, b6_;
};

}  // namespace warps_drone

#endif  // WARPS_DRONE_DSP_PINK_NOISE_H_
