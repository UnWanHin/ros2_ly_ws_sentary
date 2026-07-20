#include <gtest/gtest.h>

#include "TacticalProtectionPolicy.hpp"

namespace {

TEST(TacticalProtectionPolicy, PreservesProfileBaselineWhenYamlDoesNotSetTheSwitch) {
    EXPECT_FALSE(BehaviorTree::ResolveTacticalFeatureEnable(false, false, true));
    EXPECT_TRUE(BehaviorTree::ResolveTacticalFeatureEnable(true, false, false));
}

TEST(TacticalProtectionPolicy, ExplicitYamlValueOverridesTheProfileBaseline) {
    EXPECT_TRUE(BehaviorTree::ResolveTacticalFeatureEnable(false, true, true));
    EXPECT_FALSE(BehaviorTree::ResolveTacticalFeatureEnable(true, true, false));
}

TEST(TacticalProtectionPolicy, ProtectCastleRfidRequiresMasterSourceAndEvent) {
    EXPECT_TRUE(BehaviorTree::IsProtectCastleRfidEventEnabled(true, true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidEventEnabled(false, true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidEventEnabled(true, false, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleRfidEventEnabled(true, true, false));
}

TEST(TacticalProtectionPolicy, ProtectCastleEnemyPositionRequiresMasterAndSource) {
    EXPECT_TRUE(BehaviorTree::IsProtectCastleEnemyPositionEnabled(true, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleEnemyPositionEnabled(false, true));
    EXPECT_FALSE(BehaviorTree::IsProtectCastleEnemyPositionEnabled(true, false));
}

}  // namespace
