#include "../include/AreaManager.hpp"
#include "../include/DefaultStrategyManager.hpp"
#include "../module/json.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

namespace {

LangYa::Config SplitConfig() {
    LangYa::Config config;
    config.RegionalAreaTaskSettings.Enable = true;
    config.RegionalAreaTaskSettings.MyPreRoadland.Enable = true;
    config.RegionalAreaTaskSettings.MyReadyRoadland.Enable = true;
    config.RegionalAreaTaskSettings.DefaultPolicy.Enable = true;
    config.DecisionAutonomySettings.NaviGoal.UseAreaScope = true;
    config.DecisionAutonomySettings.NaviGoal.MyArea = {"pre_roadland", "ready_roadland"};
    return config;
}

}  // namespace

TEST(PreReadyRoadlandTaskTest, GoalIdsResolveToTheirFormalMainAreas) {
    using BehaviorTree::Area::MainAreaKind;
    using BehaviorTree::AreaManager;
    using LangYa::UnitTeam;

    const auto pre = AreaManager::ResolveGoalMainArea(LangYa::PreRoadland.ID, UnitTeam::Red);
    ASSERT_TRUE(pre.has_value());
    EXPECT_EQ(pre->Kind, MainAreaKind::PreRoadland);
    EXPECT_FALSE(pre->UsedNearestFallback);

    for (const auto goal : {LangYa::CentralToBase.ID, LangYa::BaseToCentral.ID}) {
        const auto road = AreaManager::ResolveGoalMainArea(goal, UnitTeam::Red);
        ASSERT_TRUE(road.has_value());
        EXPECT_EQ(road->Kind, MainAreaKind::ReadyRoadland);
        EXPECT_FALSE(road->UsedNearestFallback);
    }
}

TEST(PreReadyRoadlandTaskTest, AreaManagerStartsIndependentPreAndReadyRoadlandTasks) {
    using BehaviorTree::AreaManager;
    using BehaviorTree::RegionalAreaTaskType;
    using LangYa::UnitTeam;

    AreaManager manager;
    const auto now = std::chrono::steady_clock::now();
    const LangYa::MyBaseAreaTaskSetting base_setting;
    const LangYa::PatrolGoalSelectionSetting selection;

    const auto pre_plan = manager.PlanRegionalAreaTaskForGoal(
        LangYa::PreRoadland.ID, UnitTeam::Red, UnitTeam::Red, true,
        false, 0, 0, base_setting, selection, now, false);
    ASSERT_TRUE(pre_plan.has_value());
    EXPECT_EQ(pre_plan->Type, RegionalAreaTaskType::MyPreRoadland);
    EXPECT_EQ(pre_plan->InitialBaseGoal, LangYa::PreRoadland.ID);

    const auto road_plan = manager.PlanRegionalAreaTaskForGoal(
        LangYa::CentralToBase.ID, UnitTeam::Red, UnitTeam::Red, true,
        false, 0, 0, base_setting, selection, now, false);
    ASSERT_TRUE(road_plan.has_value());
    EXPECT_EQ(road_plan->Type, RegionalAreaTaskType::MyReadyRoadland);
    EXPECT_EQ(road_plan->InitialBaseGoal, LangYa::CentralToBase.ID);
}

TEST(PreReadyRoadlandTaskTest, DefaultPolicyOffersSeparatePreAndReadyRoadlandCandidates) {
    using BehaviorTree::DefaultRegionalPolicyInput;
    using BehaviorTree::DefaultStrategyManager;
    using BehaviorTree::RegionalAreaTaskType;
    using LangYa::UnitTeam;

    const auto config = SplitConfig();
    DefaultStrategyManager manager;
    const auto candidates = manager.BuildRegionalAreaCandidates(DefaultRegionalPolicyInput{
        .Config = &config,
        .MyTeam = UnitTeam::Red,
        .HealthFresh = true,
        .AmmoFresh = true,
        .Health = 400,
        .Ammo = 100,
        .Now = std::chrono::steady_clock::now()
    });

    bool found_pre = false;
    bool found_ready_roadland = false;
    for (const auto& candidate : candidates) {
        found_pre |= candidate.TaskType == RegionalAreaTaskType::MyPreRoadland &&
            candidate.BaseGoalId == LangYa::PreRoadland.ID;
        found_ready_roadland |= candidate.TaskType == RegionalAreaTaskType::MyReadyRoadland &&
            candidate.BaseGoalId == LangYa::CentralToBase.ID;
    }
    EXPECT_TRUE(found_pre);
    EXPECT_TRUE(found_ready_roadland);
}

TEST(PreReadyRoadlandTaskTest, FormalRegionalProfileEnablesPreAndReadyRoadlandAreas) {
    const auto source_root = std::filesystem::path(__FILE__).parent_path().parent_path();
    for (const auto& relative_path : {
             std::filesystem::path{"Scripts/config.json"},
             std::filesystem::path{"Scripts/ConfigJson/regional_competition.json"}}) {
        const auto profile_path = source_root / relative_path;
        std::ifstream profile_file(profile_path);
        ASSERT_TRUE(profile_file.is_open()) << profile_path;

        nlohmann::json profile;
        profile_file >> profile;
        const auto& my_area = profile.at("DecisionAutonomy").at("NaviGoal").at("MyArea");

        EXPECT_TRUE(my_area.at("PreRoadland").get<bool>()) << profile_path;
        EXPECT_TRUE(my_area.at("ReadyRoadland").get<bool>()) << profile_path;
        EXPECT_FALSE(my_area.contains("Roadland")) << profile_path;
    }
}

TEST(PreReadyRoadlandTaskTest, AreaScopeUsesReadyRoadlandWithoutLegacyRoadlandToken) {
    const auto ready = BehaviorTree::AreaManager::MainAreaKindFromToken("ready_roadland");
    EXPECT_TRUE(ready.has_value());
    const auto compact_ready = BehaviorTree::AreaManager::MainAreaKindFromToken("readyroadland");
    EXPECT_TRUE(compact_ready.has_value());
    EXPECT_FALSE(BehaviorTree::AreaManager::MainAreaKindFromToken("roadland").has_value());
    EXPECT_FALSE(BehaviorTree::AreaManager::MainAreaKindFromToken("road_land").has_value());
    EXPECT_FALSE(BehaviorTree::AreaManager::MainAreaKindFromToken("road").has_value());
}

TEST(PreReadyRoadlandTaskTest, RegionalDefenseKeepsThePreRoadlandThreatCoverage) {
    BehaviorTree::AreaManager manager;
    const auto threat = manager.AnalyzeRegionalDefenseThreat(
        LangYa::UnitTeam::Red,
        LangYa::UnitTeam::Blue,
        false,
        {{457, 72}});

    EXPECT_TRUE(threat.HardThreat);
}
