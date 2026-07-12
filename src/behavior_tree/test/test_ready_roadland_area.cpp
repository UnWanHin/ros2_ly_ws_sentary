#include "../module/Area.hpp"

#include <gtest/gtest.h>

TEST(ReadyRoadlandAreaTest, ContainsItsRedAndBlueInteriorPoints) {
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsideReadyRoadlandArea(LangYa::UnitTeam::Red, 800, 120));
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsideReadyRoadlandArea(LangYa::UnitTeam::Blue, 2000, 1380));
}

TEST(ReadyRoadlandAreaTest, RejectsPointsOutsideEachTeamBoundary) {
    EXPECT_FALSE(BehaviorTree::Area::IsPointInsideReadyRoadlandArea(LangYa::UnitTeam::Red, 350, 200));
    EXPECT_FALSE(BehaviorTree::Area::IsPointInsideReadyRoadlandArea(LangYa::UnitTeam::Blue, 2450, 1350));
}

TEST(ReadyRoadlandAreaTest, UsesRedBlueMirroredBoundaries) {
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsideReadyRoadlandArea(LangYa::UnitTeam::Red, 510, 19));
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsideReadyRoadlandArea(LangYa::UnitTeam::Blue, 2290, 1481));
}

TEST(PreRoadlandAreaTest, ContainsItsRedAndBlueInteriorPoints) {
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsidePreRoadlandArea(LangYa::UnitTeam::Red, 450, 100));
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsidePreRoadlandArea(LangYa::UnitTeam::Blue, 2200, 1200));
}

TEST(PreRoadlandAreaTest, RejectsPointsOutsideEachTeamBoundary) {
    EXPECT_FALSE(BehaviorTree::Area::IsPointInsidePreRoadlandArea(LangYa::UnitTeam::Red, 800, 120));
    EXPECT_FALSE(BehaviorTree::Area::IsPointInsidePreRoadlandArea(LangYa::UnitTeam::Blue, 2000, 1380));
}

TEST(PreRoadlandAreaTest, UsesRedBlueMirroredBoundaries) {
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsidePreRoadlandArea(LangYa::UnitTeam::Red, 510, 19));
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsidePreRoadlandArea(LangYa::UnitTeam::Blue, 2290, 1481));
}
