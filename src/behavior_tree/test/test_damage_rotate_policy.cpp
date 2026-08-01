#include <gtest/gtest.h>

#include "DamageRotatePolicy.hpp"

namespace {

TEST(DamageRotatePolicy, RampsFromZeroToThreeAtConfiguredBoundaries) {
    LangYa::DamageRotateSetting setting;
    setting.Gear0HoldMs = 220;
    setting.Gear1HoldMs = 220;
    setting.Gear2HoldMs = 220;

    EXPECT_EQ(BehaviorTree::ResolveDamageRotateGear(setting, 0U, 0), 0U);
    EXPECT_EQ(BehaviorTree::ResolveDamageRotateGear(setting, 0U, 219), 0U);
    EXPECT_EQ(BehaviorTree::ResolveDamageRotateGear(setting, 0U, 220), 1U);
    EXPECT_EQ(BehaviorTree::ResolveDamageRotateGear(setting, 0U, 439), 1U);
    EXPECT_EQ(BehaviorTree::ResolveDamageRotateGear(setting, 0U, 440), 2U);
    EXPECT_EQ(BehaviorTree::ResolveDamageRotateGear(setting, 0U, 659), 2U);
    EXPECT_EQ(BehaviorTree::ResolveDamageRotateGear(setting, 0U, 660), 3U);
}

TEST(DamageRotatePolicy, PreservesConfiguredDefaultGear) {
    LangYa::DamageRotateSetting setting;
    setting.Gear0HoldMs = 220;
    setting.Gear1HoldMs = 220;
    setting.Gear2HoldMs = 220;

    EXPECT_EQ(BehaviorTree::ResolveDamageRotateGear(setting, 2U, 0), 2U);
    EXPECT_EQ(BehaviorTree::ResolveDamageRotateGear(setting, 2U, 220), 2U);
    EXPECT_EQ(BehaviorTree::ResolveDamageRotateGear(setting, 2U, 660), 3U);
}

TEST(DamageRotatePolicy, FollowModeOverridesDamageRotate) {
    EXPECT_EQ(BehaviorTree::ResolveRotateGearWithFollowPriority(3U, true), 0U);
    EXPECT_EQ(BehaviorTree::ResolveRotateGearWithFollowPriority(3U, false), 3U);
}

TEST(DamageRotatePolicy, CastleDoesNotImplicitlySuppressDamageRotation) {
    const auto resolution = BehaviorTree::ResolveFinalRotateGear(3U, false, false, false, false);

    EXPECT_EQ(resolution.Gear, 3U);
    EXPECT_EQ(resolution.SuppressedBy, BehaviorTree::RotateSuppressionSource::None);
}

TEST(DamageRotatePolicy, ExplicitNavigationSuppressionIsReported) {
    const auto resolution = BehaviorTree::ResolveFinalRotateGear(3U, false, false, true, false);

    EXPECT_EQ(resolution.Gear, 0U);
    EXPECT_EQ(
        resolution.SuppressedBy,
        BehaviorTree::RotateSuppressionSource::NavigationShouldRotateFalse);
    EXPECT_STREQ(
        BehaviorTree::RotateSuppressionSourceToString(resolution.SuppressedBy),
        "navi_should_rotate_false");
}

TEST(DamageRotatePolicy, DebugStopHasPriorityOverAllOtherSources) {
    const auto resolution = BehaviorTree::ResolveFinalRotateGear(3U, true, false, false, true);

    EXPECT_EQ(resolution.Gear, 0U);
    EXPECT_EQ(resolution.SuppressedBy, BehaviorTree::RotateSuppressionSource::StopRotate);
}

}  // namespace
