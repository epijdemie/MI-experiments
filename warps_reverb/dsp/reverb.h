// erbe-verb 4-line FDN reverb on clouds' FxEngine.
// per branch: AP(g) -> delay(τ, mod) -> LPF. M(α) cross-couples (α=0 identity ->
// α=1 hadamard). saturated feedback supports gain up to 1.25 (erbe overshoot).
// storage = FORMAT_16_BIT (12dB cleaner than clouds default)

#ifndef WARPS_REVERB_DSP_REVERB_H_
#define WARPS_REVERB_DSP_REVERB_H_

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"

#include "clouds/dsp/fx/fx_engine.h"

#include "warps_reverb/dsp/mixing_matrix.h"
#include "warps_reverb/dsp/parameters.h"

namespace warps_reverb {

struct ShortFrame {
  int16_t l;
  int16_t r;
};

struct FloatFrame {
  float l;
  float r;
};

class Reverb {
 public:
  Reverb() { }
  ~Reverb() { }

  // buffer = FxEngine::kBufferSize × uint16_t in main sram
  void Init(uint16_t* buffer, float sample_rate);

  // mode coefficient overrides - from Ui::EnterMode
  void set_dsp_overrides(const DspOverrides& d) {
    feedback_min_         = d.feedback_min;
    feedback_max_         = d.feedback_max;
    matrix_comp_          = d.matrix_comp;
    decouple_tilt_        = d.decouple_tilt;
    matrix_alpha_         = d.matrix_alpha;
    osc_amplitude_        = d.osc_amplitude;
    noise_max_            = d.noise_amplitude;
    motion_wobbles_tilt_  = d.motion_wobbles_tilt;
  }

  void Tick();
  void Process(FloatFrame* in_out, size_t size);

  ReverbParameters* mutable_parameters() { return &parameters_; }

  // decayed wet peak. >0.95 = soft-limit clipping
  inline float peak() const { return peak_; }

  // FxEngine buffer = 64k (32768 × u16). all delay lines must fit
  static constexpr size_t kBufferSize = 32768;

 private:
  typedef clouds::FxEngine<kBufferSize, clouds::FORMAT_16_BIT> Engine;
  Engine engine_;

  MixingMatrix matrix_;

  float lpf_state_[4];

  // tick -> process cache
  float coef_input_gain_;
  float coef_pre_delay_samples_;
  float coef_diffusion_;
  float coef_feedback_;
  float coef_tilt_lpf_;
  float coef_dry_wet_;
  float coef_mod_amplitude_;

  float sample_rate_;
  float peak_;
  float peak_block_;
  float feedback_min_;
  float feedback_max_;
  float matrix_comp_;
  bool  decouple_tilt_;
  float matrix_alpha_;       // <0 -> use size, ≥0 -> fixed
  float osc_amplitude_;

  // drone voice bank - 4 unison oscs w/ independent drift LFOs
  float osc_phase_[4];
  float osc_phase_inc_;

  float drift_phase_[4];
  float drift_phase_inc_[4];

  // strega-style noise inject into FDN input
  float noise_max_;
  float noise_lp_state_;

  // per-branch svf (drone mode only). filter morphs LP↔HP
  float svf_lp_[4];
  float svf_bp_[4];
  float svf_f_base_;
  float svf_q_;
  float filter_morph_;

  // drive_factor = 1 + drive*4  (1× clean -> 5× hot)
  float drive_factor_;

  // motion lfo - wobbles per-branch tilt lpf in drone mode
  float motion_lfo_phase_;
  float motion_lfo_phase_inc_;
  float motion_lfo_depth_;
  bool  motion_wobbles_tilt_;

  // ks drone state (decouple_tilt_ = true path).
  // string per channel, outer fb loop w/ sat + filters. ks lines live in
  // float arrays (not the FxEngine buffer) to keep sustain clean
  static constexpr int kKsSize = 1024;     // ~47Hz min pitch @ 48k
  static constexpr int kFbDelSize = 1600;  // ~33ms max outer fb delay

  float ks_buffer_[2][kKsSize];
  int   ks_write_[2];
  float ks_dc_x1_[2];
  float ks_dc_y1_[2];
  float ks_damping_state_[2];
  float ks_delay_samples_[2];                    // base delay per channel
  float ks_drift_phase_[2];
  float ks_drift_phase_inc_[2];

  float fb_buffer_[2][kFbDelSize];
  int   fb_write_[2];

  // outer-loop 2-pole resonant LP svf - drift LFOs modulate cutoff/res
  float svf_lp_drone_[2];
  float svf_bp_drone_[2];
  float in_loop_hp_state_[2];

  float svf_f_base_drone_;
  float svf_q_base_drone_;
  float ks_drift_depth_;
  float ks_damping_coef_;

  // schroeder reverb inside outer fb loop. 4 damped combs -> 2-AP diffuser
  // per channel. comb fb ~0.92 -> ~2.5s RT60. prime lengths
  static constexpr int kRvbCombA = 727;
  static constexpr int kRvbCombB = 977;
  static constexpr int kRvbCombC = 1259;
  static constexpr int kRvbCombD = 1487;
  static constexpr int kRvbApLa  = 223;
  static constexpr int kRvbApLb  = 379;
  static constexpr int kRvbApRa  = 251;
  static constexpr int kRvbApRb  = 401;

  float rvb_comb_a_[kRvbCombA];
  float rvb_comb_b_[kRvbCombB];
  float rvb_comb_c_[kRvbCombC];
  float rvb_comb_d_[kRvbCombD];
  int   rvb_comb_w_[4];
  float rvb_comb_lp_[4];

  float rvb_ap_la_[kRvbApLa];
  float rvb_ap_lb_[kRvbApLb];
  float rvb_ap_ra_[kRvbApRa];
  float rvb_ap_rb_[kRvbApRb];
  int   rvb_ap_w_[4];

  float rvb_mix_;
  float rvb_feedback_;
  float rvb_damping_;

  float drone_base_noise_;       // always-on seed

  // ks per-cycle damping ties drone loudness to decay knob
  float ks_internal_damp_;
  float drone_fb_gain_;
  float drone_fb_delay_samples_;
  float drone_drive_;
  float drone_lp_coef_;
  float drone_hp_coef_;
  float drone_noise_amp_;

  void ProcessKarplusDrone(FloatFrame* in_out, size_t size);

  ReverbParameters parameters_;

  DISALLOW_COPY_AND_ASSIGN(Reverb);
};

}  // namespace warps_reverb

#endif  // WARPS_REVERB_DSP_REVERB_H_
