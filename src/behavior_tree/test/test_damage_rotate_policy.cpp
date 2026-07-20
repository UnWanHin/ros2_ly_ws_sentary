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

}  // namespace
