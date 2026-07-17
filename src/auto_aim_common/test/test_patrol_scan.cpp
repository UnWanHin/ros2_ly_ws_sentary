#include <auto_aim_common/patrol_scan.hpp>
#include <gtest/gtest.h>

TEST(PatrolScanner, Mode1AdvancesYawAndUsesPitchCurve) {
  ly_auto_aim::patrol::Config config{};
  config.mode1_yaw_step_deg = 9.0;
  config.mode1_pitch_center_deg = 5.0;
  config.mode1_pitch_half_range_deg = 15.0;
  config.mode1_pitch_period_ms = 2000.0;

  ly_auto_aim::patrol::Scanner scanner;
  const auto command = scanner.Step({10.0, 4.0}, 0.0, 1, config);

  EXPECT_DOUBLE_EQ(command.yaw_deg, 19.0);
  EXPECT_DOUBLE_EQ(command.pitch_deg, 5.0);
}

TEST(PatrolScanner, Mode2CompletesSineCycleWithCenterDriftAndReset) {
  ly_auto_aim::patrol::Config config{};
  constexpr double kTwoPi = 6.2831853071795864769;
  config.mode2_yaw_step_deg = kTwoPi / 4.0;
  config.mode2_yaw_half_range_deg = 1.0;
  config.mode2_center_drift_per_cycle_deg = -8.0;

  ly_auto_aim::patrol::Scanner scanner;
  const auto first_quarter = scanner.Step({100.0, 0.0}, 0.0, 2, config);
  const auto half_cycle = scanner.Step({100.0, 0.0}, 0.0, 2, config);
  const auto third_quarter = scanner.Step({100.0, 0.0}, 0.0, 2, config);
  const auto full_cycle = scanner.Step({100.0, 0.0}, 0.0, 2, config);

  // Each quarter-cycle drifts the center by -2 degrees before applying +/-1 amplitude.
  EXPECT_NEAR(first_quarter.yaw_deg, 99.0, 1e-9);
  EXPECT_NEAR(half_cycle.yaw_deg, 96.0, 1e-9);
  EXPECT_NEAR(third_quarter.yaw_deg, 93.0, 1e-9);
  EXPECT_NEAR(full_cycle.yaw_deg, 92.0, 1e-9);

  scanner.Reset();
  const auto reset_first_quarter = scanner.Step({7.0, 0.0}, 0.0, 2, config);
  EXPECT_NEAR(reset_first_quarter.yaw_deg, 6.0, 1e-9);
}

TEST(PatrolScanner, Mode3UsesConfiguredHighPitchCurve) {
  ly_auto_aim::patrol::Config config{};
  config.mode3_yaw_step_deg = 6.0;
  config.mode3_pitch_offset_deg = 20.0;
  config.mode3_pitch_half_range_deg = 0.0;

  ly_auto_aim::patrol::Scanner scanner;
  const auto command = scanner.Step({-5.0, 0.0}, 100.0, 3, config);

  EXPECT_DOUBLE_EQ(command.yaw_deg, 1.0);
  EXPECT_DOUBLE_EQ(command.pitch_deg, 20.0);
}
