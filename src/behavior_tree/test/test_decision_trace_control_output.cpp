#include "Application.hpp"

#include <gtest/gtest.h>

namespace {

gimbal_driver::msg::FireCode MakeFireCode(
    const std::uint8_t fire_status,
    const bool follow_mode,
    const std::uint8_t rotate) {
    gimbal_driver::msg::FireCode message;
    message.field_mask = gimbal_driver::msg::FireCode::FIELD_ALL;
    message.raw = 0x5A;
    message.fire_status = fire_status;
    message.cap_state = 2;
    message.follow_mode = follow_mode;
    message.aim_mode = true;
    message.rotate = rotate;
    return message;
}

gimbal_driver::msg::GimbalAngles MakeAngles() {
    gimbal_driver::msg::GimbalAngles message;
    message.yaw = 12.5F;
    message.pitch = -3.25F;
    return message;
}

gimbal_driver::msg::GimbalTrajectory MakeTrajectory() {
    gimbal_driver::msg::GimbalTrajectory message;
    message.yaw = 12.5F;
    message.pitch = -3.25F;
    message.yaw_omega = 1.0F;
    message.pitch_omega = -2.0F;
    message.yaw_alpha = 3.0F;
    message.pitch_alpha = -4.0F;
    return message;
}

}  // namespace

TEST(DecisionTraceControlOutput, KeepsCallbackFeedbackDistinctFromFinalFireCommand) {
    const auto feedback = MakeFireCode(0, false, 0);
    const auto feedback_snapshot = BehaviorTree::MakeTraceFireCodeSnapshot(feedback);
    auto final_command = MakeFireCode(3, true, 3);

    EXPECT_EQ(feedback_snapshot.Rotate, 0);
    EXPECT_EQ(final_command.rotate, 3);

    final_command.rotate = 1;
    EXPECT_EQ(feedback_snapshot.Rotate, 0);
}

TEST(DecisionTraceControlOutput, CopiesTheExactPublishedMessagesIntoAnImmutableSnapshot) {
    auto angles = MakeAngles();
    auto fire_code = MakeFireCode(3, true, 3);
    const auto trajectory = std::optional<gimbal_driver::msg::GimbalTrajectory>{MakeTrajectory()};

    const auto snapshot = BehaviorTree::MakeControlOutputTraceSnapshot(
        19,
        std::chrono::steady_clock::now(),
        BehaviorTree::ControlOutputSnapshotSource::Normal,
        angles,
        true,
        fire_code,
        true,
        trajectory,
        true,
        BehaviorTree::ControlTrajectoryUnavailableReason::None);

    EXPECT_TRUE(snapshot.Available);
    EXPECT_EQ(snapshot.Sequence, 19U);
    EXPECT_TRUE(snapshot.Angles.Published);
    EXPECT_FLOAT_EQ(snapshot.Angles.Yaw, 12.5F);
    EXPECT_TRUE(snapshot.FireCode.Published);
    EXPECT_EQ(snapshot.FireCode.FireCode.Rotate, 3);
    EXPECT_TRUE(snapshot.Trajectory.Published);
    EXPECT_TRUE(snapshot.Trajectory.Available);
    EXPECT_EQ(snapshot.Trajectory.UnavailableReason,
              BehaviorTree::ControlTrajectoryUnavailableReason::None);
    EXPECT_FLOAT_EQ(snapshot.Trajectory.YawAlpha, 3.0F);

    angles.yaw = 99.0F;
    fire_code.rotate = 0;
    EXPECT_FLOAT_EQ(snapshot.Angles.Yaw, 12.5F);
    EXPECT_EQ(snapshot.FireCode.FireCode.Rotate, 3);
}

TEST(DecisionTraceControlOutput, SafeControlCapturesNoTrajectoryWithAnExplicitReason) {
    const auto snapshot = BehaviorTree::MakeControlOutputTraceSnapshot(
        20,
        std::chrono::steady_clock::now(),
        BehaviorTree::ControlOutputSnapshotSource::SafeControl,
        MakeAngles(),
        true,
        MakeFireCode(0, false, 0),
        true,
        std::nullopt,
        false,
        BehaviorTree::ControlTrajectoryUnavailableReason::SafeControl);

    EXPECT_TRUE(snapshot.Available);
    EXPECT_EQ(snapshot.Source, BehaviorTree::ControlOutputSnapshotSource::SafeControl);
    EXPECT_TRUE(snapshot.Angles.Published);
    EXPECT_TRUE(snapshot.FireCode.Published);
    EXPECT_FALSE(snapshot.Trajectory.Published);
    EXPECT_FALSE(snapshot.Trajectory.Available);
    EXPECT_EQ(snapshot.Trajectory.UnavailableReason,
              BehaviorTree::ControlTrajectoryUnavailableReason::SafeControl);
}
