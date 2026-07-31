#include "EnemyPositionSourcePolicy.hpp"

#include <gtest/gtest.h>

TEST(EnemyPositionSourcePolicyTest, RejectsCameraFallbackWhenDisabled) {
    LangYa::ChaseSetting setting;
    setting.EnableNaviTargetOfficialFallback = false;

    EXPECT_FALSE(BehaviorTree::ShouldAcceptNaviTargetOfficialFallback(setting, false));
}

TEST(EnemyPositionSourcePolicyTest, PreservesFreshLowerMachinePrecedence) {
    LangYa::ChaseSetting setting;
    setting.EnableNaviTargetOfficialFallback = true;

    EXPECT_FALSE(BehaviorTree::ShouldAcceptNaviTargetOfficialFallback(setting, true));
    EXPECT_TRUE(BehaviorTree::ShouldAcceptNaviTargetOfficialFallback(setting, false));
}
