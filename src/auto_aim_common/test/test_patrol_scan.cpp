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

TEST(PatrolScanner, Mode2AnchorsAtFeedbackAndOscillates) {
  ly_auto_aim::patrol::Config config{};
  config.mode2_yaw_step_deg = 1.0;
  config.mode2_yaw_half_range_deg = 30.0;

  ly_auto_aim::patrol::Scanner scanner;
  const auto first = scanner.Step({42.0, 0.0}, 0.0, 2, config);
  EXPECT_GT(first.yaw_deg, 42.0);

  scanner.Reset();
  const auto reseeded = scanner.Step({7.0, 0.0}, 0.0, 2, config);
  EXPECT_GT(reseeded.yaw_deg, 7.0);
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
