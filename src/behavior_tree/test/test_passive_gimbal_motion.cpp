#include "../module/PassiveGimbalMotion.hpp"

#include <gtest/gtest.h>

namespace {

LangYa::GimbalAnglesType Angles(const float yaw, const float pitch) {
    return LangYa::GimbalAnglesType{yaw, pitch};
}

}  // namespace

TEST(PassiveGimbalMotionTest, LimitsFaceModeReacquisitionByElapsedTime) {
    BehaviorTree::PassiveGimbalMotion motion;
    const auto start = std::chrono::steady_clock::now();
    const auto initial = motion.Step(
        Angles(0.0f, 0.0f), Angles(90.0f, 45.0f), start, 120.0, 60.0);
    const auto after_ten_ms = motion.Step(
        Angles(0.0f, 0.0f), Angles(90.0f, 45.0f), start + std::chrono::milliseconds(10), 120.0, 60.0);

    EXPECT_FLOAT_EQ(initial.Yaw, 0.0f);
    EXPECT_FLOAT_EQ(initial.Pitch, 0.0f);
    EXPECT_NEAR(after_ten_ms.Yaw, 1.2f, 0.01f);
    EXPECT_NEAR(after_ten_ms.Pitch, 0.6f, 0.01f);
}

TEST(PassiveGimbalMotionTest, CapsStalledControlIntervalsToAvoidACommandJump) {
    BehaviorTree::PassiveGimbalMotion motion;
    const auto start = std::chrono::steady_clock::now();
    motion.Step(Angles(0.0f, 0.0f), Angles(180.0f, 0.0f), start, 120.0, 60.0);
    const auto after_stall = motion.Step(
        Angles(0.0f, 0.0f), Angles(180.0f, 0.0f), start + std::chrono::milliseconds(250), 120.0, 60.0);

    EXPECT_NEAR(after_stall.Yaw, 3.0f, 0.01f);
}
