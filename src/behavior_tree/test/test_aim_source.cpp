#include "../include/AimSource.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

LangYa::AimData FreshAim() {
    LangYa::AimData data;
    data.Fresh = true;
    data.Valid = true;
    data.FireStatus = true;
    data.HasLatchedAngles = true;
    data.LastValidTime = std::chrono::steady_clock::now();
    return data;
}

}  // namespace

TEST(AimSourceTest, FormalViewsMapToExternalAim) {
    using namespace BehaviorTree;
    using namespace LangYa;

    AimData external_aim;

    external_aim = FreshAim();
    const auto external_view = MakeAimSourceView(external_aim);
    EXPECT_EQ(external_view.Active, &external_aim);
    EXPECT_EQ(external_view.AutoAim, &external_aim);
    EXPECT_EQ(external_view.Buff, &external_aim);
    EXPECT_EQ(external_view.Outpost, &external_aim);
    EXPECT_TRUE(AimFreshAndValid(*external_view.Active));
}

TEST(AimSourceTest, BuffLockUsesUnifiedExternalAim) {
    using namespace BehaviorTree;
    using namespace LangYa;

    AimData external_aim = FreshAim();
    external_aim.BuffFollow = false;
    EXPECT_TRUE(AimBuffTargetLocked(external_aim, true));
    EXPECT_TRUE(AimBuffFireReady(external_aim, true));
}

TEST(AimSourceTest, LatchedTargetHonorsHoldWindowAndFreshFlags) {
    using namespace BehaviorTree;
    using namespace LangYa;

    AimData latched_only;
    const auto now = std::chrono::steady_clock::now();
    latched_only.HasLatchedAngles = true;
    latched_only.LastValidTime = now - std::chrono::milliseconds(50);
    EXPECT_TRUE(AimLatchedRecently(latched_only, now, std::chrono::milliseconds(100)));
    EXPECT_FALSE(
        AimLatchedRecently(latched_only, now, std::chrono::milliseconds(10)));

    bool fresh_target = true;
    bool latched_target = true;
    EXPECT_TRUE(
        AimTargetForAngles(
            latched_only,
            false,
            true,
            now,
            std::chrono::milliseconds(100),
            &fresh_target,
            &latched_target));
    EXPECT_FALSE(fresh_target);
    EXPECT_TRUE(latched_target);
}

TEST(AimSourceTest, FreshValidAimMapsAllTrajectoryFields) {
    using namespace BehaviorTree;
    using namespace LangYa;

    AimData aim = FreshAim();
    aim.Angles = GimbalAnglesType{10.0F, -5.0F};
    aim.YawOmega = 20.0F;
    aim.PitchOmega = -10.0F;
    aim.YawAlpha = 30.0F;
    aim.PitchAlpha = -15.0F;

    const auto trajectory = MakeGimbalTrajectory(aim);
    ASSERT_TRUE(trajectory.has_value());
    EXPECT_FLOAT_EQ(trajectory->yaw, 10.0F);
    EXPECT_FLOAT_EQ(trajectory->pitch, -5.0F);
    EXPECT_FLOAT_EQ(trajectory->yaw_omega, 20.0F);
    EXPECT_FLOAT_EQ(trajectory->pitch_omega, -10.0F);
    EXPECT_FLOAT_EQ(trajectory->yaw_alpha, 30.0F);
    EXPECT_FLOAT_EQ(trajectory->pitch_alpha, -15.0F);
}

TEST(AimSourceTest, InvalidOrNonFiniteAimDoesNotMapTrajectory) {
    using namespace BehaviorTree;
    using namespace LangYa;

    AimData aim = FreshAim();
    aim.Angles = GimbalAnglesType{10.0F, -5.0F};
    aim.YawOmega = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(MakeGimbalTrajectory(aim).has_value());

    aim = FreshAim();
    aim.Valid = false;
    EXPECT_FALSE(MakeGimbalTrajectory(aim).has_value());
}
