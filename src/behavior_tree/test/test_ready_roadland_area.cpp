#include "../module/Area.hpp"

#include <gtest/gtest.h>

TEST(RoadlandAreaTest, ContainsItsRedAndBlueInteriorPoints) {
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsideMainArea(
        LangYa::UnitTeam::Red, BehaviorTree::Area::MainAreaKind::Roadland, 800, 120));
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsideMainArea(
        LangYa::UnitTeam::Blue, BehaviorTree::Area::MainAreaKind::Roadland, 2000, 1380));
}

TEST(RoadlandAreaTest, RejectsPointsOutsideEachTeamBoundary) {
    EXPECT_FALSE(BehaviorTree::Area::IsPointInsideMainArea(
        LangYa::UnitTeam::Red, BehaviorTree::Area::MainAreaKind::Roadland, 350, 200));
    EXPECT_FALSE(BehaviorTree::Area::IsPointInsideMainArea(
        LangYa::UnitTeam::Blue, BehaviorTree::Area::MainAreaKind::Roadland, 2450, 1350));
}

TEST(RoadlandAreaTest, UsesRedBlueMirroredBoundaries) {
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsideMainArea(
        LangYa::UnitTeam::Red, BehaviorTree::Area::MainAreaKind::Roadland, 510, 19));
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsideMainArea(
        LangYa::UnitTeam::Blue, BehaviorTree::Area::MainAreaKind::Roadland, 2290, 1481));
}

TEST(PreRoadlandAreaTest, ContainsItsRedAndBlueInteriorPoints) {
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsideMainArea(
        LangYa::UnitTeam::Red, BehaviorTree::Area::MainAreaKind::PreRoadland, 450, 100));
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsideMainArea(
        LangYa::UnitTeam::Blue, BehaviorTree::Area::MainAreaKind::PreRoadland, 2200, 1200));
}

TEST(PreRoadlandAreaTest, RejectsPointsOutsideEachTeamBoundary) {
    EXPECT_FALSE(BehaviorTree::Area::IsPointInsidePreRoadlandArea(LangYa::UnitTeam::Red, 800, 120));
    EXPECT_FALSE(BehaviorTree::Area::IsPointInsidePreRoadlandArea(LangYa::UnitTeam::Blue, 2000, 1380));
}

TEST(PreRoadlandAreaTest, UsesRedBlueMirroredBoundaries) {
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsidePreRoadlandArea(LangYa::UnitTeam::Red, 510, 19));
    EXPECT_TRUE(BehaviorTree::Area::IsPointInsidePreRoadlandArea(LangYa::UnitTeam::Blue, 2290, 1481));
}

TEST(RoadlandSplitPointOwnershipTest, NavigationPointsHaveOneFormalArea) {
    using BehaviorTree::Area::IsPointInsideMainArea;
    using BehaviorTree::Area::MainAreaKind;
    using LangYa::UnitTeam;

    const auto red_pre = BehaviorTree::Area::PreRoadland(UnitTeam::Red);
    const auto blue_pre = BehaviorTree::Area::PreRoadland(UnitTeam::Blue);
    EXPECT_EQ(LangYa::PreRoadland.ID, 25U);
    EXPECT_TRUE(IsPointInsideMainArea(UnitTeam::Red, MainAreaKind::PreRoadland, red_pre.x, red_pre.y));
    EXPECT_FALSE(IsPointInsideMainArea(UnitTeam::Red, MainAreaKind::Roadland, red_pre.x, red_pre.y));
    EXPECT_TRUE(IsPointInsideMainArea(UnitTeam::Blue, MainAreaKind::PreRoadland, blue_pre.x, blue_pre.y));
    EXPECT_FALSE(IsPointInsideMainArea(UnitTeam::Blue, MainAreaKind::Roadland, blue_pre.x, blue_pre.y));

    const auto red_to_base = BehaviorTree::Area::CentralToBase(UnitTeam::Red);
    const auto blue_to_base = BehaviorTree::Area::CentralToBase(UnitTeam::Blue);
    EXPECT_EQ(red_to_base.x, 515U);
    EXPECT_EQ(red_to_base.y, 100U);
    EXPECT_EQ(blue_to_base.x, 2285U);
    EXPECT_EQ(blue_to_base.y, 1400U);
    EXPECT_TRUE(IsPointInsideMainArea(UnitTeam::Red, MainAreaKind::Roadland, red_to_base.x, red_to_base.y));
    EXPECT_FALSE(IsPointInsideMainArea(UnitTeam::Red, MainAreaKind::PreRoadland, red_to_base.x, red_to_base.y));
    EXPECT_TRUE(IsPointInsideMainArea(UnitTeam::Blue, MainAreaKind::Roadland, blue_to_base.x, blue_to_base.y));
    EXPECT_FALSE(IsPointInsideMainArea(UnitTeam::Blue, MainAreaKind::PreRoadland, blue_to_base.x, blue_to_base.y));

    const auto red_to_central = BehaviorTree::Area::BaseToCentral(UnitTeam::Red);
    const auto blue_to_central = BehaviorTree::Area::BaseToCentral(UnitTeam::Blue);
    EXPECT_TRUE(IsPointInsideMainArea(UnitTeam::Red, MainAreaKind::Roadland, red_to_central.x, red_to_central.y));
    EXPECT_TRUE(IsPointInsideMainArea(UnitTeam::Blue, MainAreaKind::Roadland, blue_to_central.x, blue_to_central.y));
}
