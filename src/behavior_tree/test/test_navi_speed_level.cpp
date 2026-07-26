#include "../include/NaviSpeedLevel.hpp"

#include <gtest/gtest.h>

TEST(NaviSpeedLevelTest, PreservesTheThreeDefinedLevels) {
    EXPECT_EQ(BehaviorTree::NormalizeNaviSpeedLevel(BehaviorTree::kNaviSpeedStop), 0U);
    EXPECT_EQ(BehaviorTree::NormalizeNaviSpeedLevel(BehaviorTree::kNaviSpeedNormal), 1U);
    EXPECT_EQ(BehaviorTree::NormalizeNaviSpeedLevel(BehaviorTree::kNaviSpeedFast), 2U);
}

TEST(NaviSpeedLevelTest, InvalidLevelFallsBackToNormal) {
    EXPECT_EQ(BehaviorTree::NormalizeNaviSpeedLevel(3U), 1U);
    EXPECT_EQ(BehaviorTree::NormalizeNaviSpeedLevel(255U), 1U);
}
