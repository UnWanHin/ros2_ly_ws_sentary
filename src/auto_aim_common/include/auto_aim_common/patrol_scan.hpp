#pragma once

#include <algorithm>
#include <cmath>

namespace ly_auto_aim::patrol {

struct Angles {
  double yaw_deg{0.0};
  double pitch_deg{0.0};
};

struct Config {
  double mode1_yaw_step_deg{9.0};
  double mode1_yaw_boost_step_deg{10.0};
  double mode1_pitch_center_deg{5.0};
  double mode1_pitch_half_range_deg{15.0};
  double mode1_pitch_period_ms{2000.0};

  double mode2_yaw_step_deg{1.0};
  double mode2_yaw_boost_step_deg{1.1};
  double mode2_yaw_half_range_deg{30.0};
  double mode2_center_drift_per_cycle_deg{-70.0};
  double mode2_pitch_center_deg{0.0};
  double mode2_pitch_half_range_deg{13.0};
  double mode2_pitch_period_ms{500.0};

  double mode3_yaw_step_deg{6.0};
  double mode3_pitch_offset_deg{0.0};
  double mode3_pitch_half_range_deg{12.0};
  double mode3_pitch_period_ms{2000.0};
};

class Scanner {
 public:
  void Reset() noexcept {
    active_mode_ = 0;
    center_yaw_deg_ = 0.0;
    phase_rad_ = 0.0;
    initialized_ = false;
  }

  Angles Step(Angles feedback, double elapsed_ms, int mode, const Config& config) {
    constexpr double kTwoPi = 6.2831853071795864769;

    double yaw_step_deg = config.mode1_yaw_step_deg;
    double pitch_center_deg = config.mode1_pitch_center_deg;
    double pitch_half_range_deg = config.mode1_pitch_half_range_deg;
    double pitch_period_ms = config.mode1_pitch_period_ms;
    double next_yaw_deg = feedback.yaw_deg + yaw_step_deg;

    if (mode == 2) {
      yaw_step_deg = config.mode2_yaw_step_deg;
      pitch_center_deg = config.mode2_pitch_center_deg;
      pitch_half_range_deg = config.mode2_pitch_half_range_deg;
      pitch_period_ms = config.mode2_pitch_period_ms;

      if (!initialized_ || active_mode_ != mode) {
        active_mode_ = mode;
        center_yaw_deg_ = feedback.yaw_deg;
        phase_rad_ = 0.0;
        initialized_ = true;
      }

      const double half_range_deg = std::max(config.mode2_yaw_half_range_deg, 1.0);
      const double phase_step = yaw_step_deg / half_range_deg;
      const double center_drift_step =
          config.mode2_center_drift_per_cycle_deg * phase_step / kTwoPi;
      center_yaw_deg_ = NormalizeAngleNear(center_yaw_deg_ + center_drift_step, feedback.yaw_deg);
      phase_rad_ = std::fmod(phase_rad_ + phase_step, kTwoPi);
      if (phase_rad_ < 0.0) {
        phase_rad_ += kTwoPi;
      }
      next_yaw_deg = NormalizeAngleNear(
          center_yaw_deg_ + config.mode2_yaw_half_range_deg * std::sin(phase_rad_),
          feedback.yaw_deg);
    } else {
      if (active_mode_ != mode || initialized_) {
        Reset();
        active_mode_ = mode;
      }

      if (mode == 3) {
        yaw_step_deg = config.mode3_yaw_step_deg;
        pitch_center_deg = config.mode3_pitch_offset_deg;
        pitch_half_range_deg = config.mode3_pitch_half_range_deg;
        pitch_period_ms = config.mode3_pitch_period_ms;
      }
      next_yaw_deg = feedback.yaw_deg + yaw_step_deg;
    }

    const double next_pitch_deg = pitch_center_deg +
        pitch_half_range_deg * std::sin(elapsed_ms * kTwoPi / std::max(pitch_period_ms, 1.0));
    return {next_yaw_deg, next_pitch_deg};
  }

 private:
  static double NormalizeAngleNear(double angle_deg, double reference_deg) {
    return reference_deg + std::remainder(angle_deg - reference_deg, 360.0);
  }

  int active_mode_{0};
  double center_yaw_deg_{0.0};
  double phase_rad_{0.0};
  bool initialized_{false};
};

}  // namespace ly_auto_aim::patrol
