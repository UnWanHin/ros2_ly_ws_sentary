#include "../include/AreaManager.hpp"
#include "../include/DefaultStrategyManager.hpp"
#include "../include/DecisionIntent.hpp"
#include "../include/EventManager.hpp"
#include "../include/MapCommandTask.hpp"
#include "../module/json.hpp"

#include <algorithm>
#include <array>
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

TEST(PreReadyRoadlandTaskTest, MapCommandAcceptsOnlyNonZeroCoordinateModeAndConvertsToRawCentimeters) {
    const auto now = std::chrono::steady_clock::now();
    LangYa::MapCommandSetting setting;
    BehaviorTree::MapCommandTask task;

    EXPECT_FALSE(task.Observe(
        {.HasTargetPosition = true, .XMeter = 0.0F, .YMeter = 0.0F}, setting, now));
    EXPECT_FALSE(task.Observe(
        {.HasTargetPosition = false, .XMeter = 2.5F, .YMeter = 3.25F}, setting, now));
    EXPECT_FALSE(task.Active(now));

    EXPECT_TRUE(task.Observe(
        {.HasTargetPosition = true, .XMeter = 2.5F, .YMeter = 3.25F}, setting, now));
    const auto raw_goal = task.ActiveGoal(now);
    ASSERT_TRUE(raw_goal.has_value());
    EXPECT_EQ(raw_goal->XCentimeter, 250U);
    EXPECT_EQ(raw_goal->YCentimeter, 325U);
}

TEST(PreReadyRoadlandTaskTest, MapCommandRejectsOutOfFieldAndRoundedDefaultCoordinates) {
    const auto now = std::chrono::steady_clock::now();
    LangYa::MapCommandSetting setting;
    BehaviorTree::MapCommandTask task;

    EXPECT_FALSE(task.Observe(
        {.HasTargetPosition = true, .XMeter = 0.004F, .YMeter = 0.0F}, setting, now));
    EXPECT_FALSE(task.Observe(
        {.HasTargetPosition = true, .XMeter = 28.01F, .YMeter = 10.0F}, setting, now));
    EXPECT_FALSE(task.Observe(
        {.HasTargetPosition = true, .XMeter = 10.0F, .YMeter = 15.01F}, setting, now));
    EXPECT_TRUE(task.Observe(
        {.HasTargetPosition = true, .XMeter = 0.0F, .YMeter = 15.0F}, setting, now));
}

TEST(PreReadyRoadlandTaskTest, MapCommandDeduplicatesRepeatsAndDropsOwnershipOnExpiryOrCancel) {
    const auto now = std::chrono::steady_clock::now();
    LangYa::MapCommandSetting setting;
    setting.HoldSec = 45;
    setting.DedupDistanceCm = 20;
    BehaviorTree::MapCommandTask task;
    const BehaviorTree::MapCommandInput point{
        .HasTargetPosition = true, .XMeter = 2.5F, .YMeter = 3.25F};

    EXPECT_TRUE(task.Observe(point, setting, now));
    EXPECT_FALSE(task.Observe(point, setting, now + std::chrono::milliseconds(100)));
    EXPECT_TRUE(task.Active(now + std::chrono::seconds(44)));
    EXPECT_FALSE(task.Active(now + std::chrono::seconds(45)));

    const BehaviorTree::MapCommandInput replacement{
        .HasTargetPosition = true, .XMeter = 2.8F, .YMeter = 3.25F};
    EXPECT_TRUE(task.Observe(replacement, setting, now + std::chrono::seconds(46)));
    EXPECT_TRUE(task.Active(now + std::chrono::seconds(46)));
    task.Cancel();
    EXPECT_FALSE(task.Active(now + std::chrono::seconds(46)));
    EXPECT_FALSE(task.Observe(replacement, setting, now + std::chrono::seconds(47)));
    EXPECT_TRUE(task.Observe(
        {.HasTargetPosition = true, .XMeter = 3.1F, .YMeter = 3.25F},
        setting,
        now + std::chrono::seconds(47)));
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

TEST(PreReadyRoadlandTaskTest, DefaultPolicyDoesNotImmediatelyRepeatLastEligibleArea) {
    using BehaviorTree::DefaultRegionalPolicyInput;
    using BehaviorTree::DefaultStrategyManager;
    using BehaviorTree::RegionalAreaTaskType;
    using LangYa::UnitTeam;

    auto config = SplitConfig();
    config.RegionalAreaTaskSettings.MyBase.Enable = true;
    config.DecisionAutonomySettings.NaviGoal.MyArea = {
        "base", "pre_roadland", "ready_roadland"};

    DefaultStrategyManager manager;
    const DefaultRegionalPolicyInput input{
        .Config = &config,
        .MyTeam = UnitTeam::Red,
        .HealthFresh = true,
        .AmmoFresh = true,
        .Health = 400,
        .Ammo = 100,
        .Now = std::chrono::steady_clock::now()
    };

    const auto first = manager.BuildRegionalAreaCandidates(input);
    ASSERT_FALSE(first.empty());
    ASSERT_EQ(first.front().TaskType, RegionalAreaTaskType::MyBase);

    manager.CommitRegionalAreaSelection(first.front(), input.Now);
    const auto next = manager.BuildRegionalAreaCandidates(input);
    ASSERT_GT(next.size(), 1U);
    EXPECT_NE(next.front().TaskType, RegionalAreaTaskType::MyBase);
}

TEST(PreReadyRoadlandTaskTest, DefaultPolicyResumesPreemptedEligibleAreaBeforeRescoring) {
    using BehaviorTree::DefaultRegionalPolicyInput;
    using BehaviorTree::DefaultStrategyManager;
    using BehaviorTree::RegionalAreaTaskType;
    using LangYa::UnitTeam;

    auto config = SplitConfig();
    config.RegionalAreaTaskSettings.MyBase.Enable = true;
    config.DecisionAutonomySettings.NaviGoal.MyArea = {
        "base", "pre_roadland", "ready_roadland"};

    DefaultStrategyManager manager;
    const DefaultRegionalPolicyInput input{
        .Config = &config,
        .MyTeam = UnitTeam::Red,
        .HealthFresh = true,
        .AmmoFresh = true,
        .Health = 400,
        .Ammo = 100,
        .Now = std::chrono::steady_clock::now()
    };

    manager.RecordRegionalAreaResult(
        RegionalAreaTaskType::MyPreRoadland,
        "preempted",
        input.Now,
        config.RegionalAreaTaskSettings.DefaultPolicy);

    const auto candidates = manager.BuildRegionalAreaCandidates(input);
    ASSERT_FALSE(candidates.empty());
    EXPECT_EQ(candidates.front().TaskType, RegionalAreaTaskType::MyPreRoadland);
}

TEST(PreReadyRoadlandTaskTest, DefaultPolicyDoesNotResumePreemptedAreaOutsideScope) {
    using BehaviorTree::DefaultRegionalPolicyInput;
    using BehaviorTree::DefaultStrategyManager;
    using BehaviorTree::RegionalAreaTaskType;
    using LangYa::UnitTeam;

    auto config = SplitConfig();
    config.RegionalAreaTaskSettings.MyBase.Enable = true;
    config.DecisionAutonomySettings.NaviGoal.MyArea = {"base", "ready_roadland"};

    DefaultStrategyManager manager;
    const DefaultRegionalPolicyInput input{
        .Config = &config,
        .MyTeam = UnitTeam::Red,
        .HealthFresh = true,
        .AmmoFresh = true,
        .Health = 400,
        .Ammo = 100,
        .Now = std::chrono::steady_clock::now()
    };

    manager.RecordRegionalAreaResult(
        RegionalAreaTaskType::MyPreRoadland,
        "preempted",
        input.Now,
        config.RegionalAreaTaskSettings.DefaultPolicy);

    const auto candidates = manager.BuildRegionalAreaCandidates(input);
    ASSERT_FALSE(candidates.empty());
    EXPECT_EQ(candidates.front().TaskType, RegionalAreaTaskType::MyBase);

    config.DecisionAutonomySettings.NaviGoal.MyArea = {
        "base", "pre_roadland", "ready_roadland"};
    const auto later_candidates = manager.BuildRegionalAreaCandidates(input);
    ASSERT_FALSE(later_candidates.empty());
    EXPECT_EQ(later_candidates.front().TaskType, RegionalAreaTaskType::MyBase);
}

TEST(PreReadyRoadlandTaskTest, OpeningOutpostTravelHasAimModeDecisionIntent) {
    EXPECT_EQ(
        BehaviorTree::DecisionReasonFromString(
            "regional_tactical_opening_outpost_scout_travel"),
        BehaviorTree::DecisionReason::AimModeOutpost);
}

TEST(PreReadyRoadlandTaskTest, DefaultBasePatrolUsesOnlyCastlePoints) {
    const LangYa::MyBaseAreaTaskSetting setting;
    const std::array<std::uint8_t, 4> expected{
        LangYa::CastleLeft1.ID,
        LangYa::CastleLeft2.ID,
        LangYa::CastleRight2.ID,
        LangYa::CastleRight1.ID,
    };

    ASSERT_EQ(setting.PatrolGoals.size(), expected.size());
    EXPECT_EQ(setting.GoalHoldSec, 15);
    for (const auto& goal : setting.PatrolGoals) {
        EXPECT_NE(
            std::find(expected.begin(), expected.end(), goal.BaseGoalId),
            expected.end());
    }
}

TEST(PreReadyRoadlandTaskTest, DefaultAreaHoldDefaultsAreFifteenSeconds) {
    const LangYa::MyBaseAreaTaskSetting base;
    const LangYa::MyHighlandAreaTaskSetting highland;
    const LangYa::MyPreRoadlandAreaTaskSetting pre_roadland;
    const LangYa::MyReadyRoadlandAreaTaskSetting ready_roadland;
    const LangYa::CommonCentralAreaTaskSetting central;

    EXPECT_EQ(base.GoalHoldSec, 15);
    EXPECT_EQ(highland.HighlandPatrolHoldSec, 15);
    EXPECT_EQ(highland.BuffShootHoldSec, 15);
    EXPECT_EQ(pre_roadland.GoalHoldSec, 15);
    EXPECT_EQ(ready_roadland.GuardHoldSec, 15);
    EXPECT_EQ(central.GoalHoldSec, 15);
}

TEST(PreReadyRoadlandTaskTest, CommonCentralHoldsAnArrivedPointBeforeAdvancing) {
    using BehaviorTree::AreaManager;
    using BehaviorTree::RegionalAreaTaskTickInput;
    using LangYa::UnitTeam;

    AreaManager manager;
    const auto now = std::chrono::steady_clock::now();
    LangYa::RegionalAreaTaskSetting setting;
    setting.Enable = true;
    setting.CommonCentral.Enable = true;
    manager.StartRegionalAreaTask(BehaviorTree::RegionalAreaTaskPlan{
        .Type = BehaviorTree::RegionalAreaTaskType::CommonCentral,
        .GoalTeam = UnitTeam::Red,
        .ApplyTeamOffset = true,
        .TriggerBaseGoal = LangYa::OutpostArea.ID,
        .InitialBaseGoal = LangYa::OutpostArea.ID,
        .InitialGoalTeam = UnitTeam::Red,
        .InitialPatrolIndex = 0,
    }, now);
    ASSERT_TRUE(manager.RegionalAreaTaskActive());
    ASSERT_EQ(manager.RegionalAreaTask().Type, BehaviorTree::RegionalAreaTaskType::CommonCentral);

    const auto before_arrival = manager.TickRegionalAreaTask(
        RegionalAreaTaskTickInput{.Setting = setting, .Now = now});
    ASSERT_TRUE(before_arrival.Active);

    const auto arrived = manager.TickRegionalAreaTask(
        RegionalAreaTaskTickInput{
            .Setting = setting,
            .Now = now + std::chrono::seconds(1),
            .IsCurrentGoalArrived = true,
        });
    EXPECT_EQ(arrived.BaseGoalId, before_arrival.BaseGoalId);

    const auto after_hold = manager.TickRegionalAreaTask(
        RegionalAreaTaskTickInput{
            .Setting = setting,
            .Now = now + std::chrono::seconds(16),
        });
    EXPECT_NE(after_hold.BaseGoalId, before_arrival.BaseGoalId);
}

TEST(PreReadyRoadlandTaskTest, EventManagerDoesNotTreatRawReachedAsFinalArrival) {
    BehaviorTree::EventManager manager;
    const auto now = std::chrono::steady_clock::now();

    const auto snapshot = manager.Evaluate(BehaviorTree::EventEvaluateInput{
        .Now = now,
        .HasNaviReach = true,
        .NaviReach = true,
        .LastNaviReachRxTime = now,
    });

    EXPECT_FALSE(snapshot.GoalReached);

    const auto composite_snapshot = manager.Evaluate(BehaviorTree::EventEvaluateInput{
        .Now = now,
        .CompositeGoalReached = true,
    });
    EXPECT_TRUE(composite_snapshot.GoalReached);
}

TEST(PreReadyRoadlandTaskTest, ProgressWatchdogUsesCompositeArrivalOnly) {
    using BehaviorTree::AreaManager;
    using BehaviorTree::NaviProgressWatchdogInput;
    using LangYa::UnitTeam;

    AreaManager manager;
    const auto now = std::chrono::steady_clock::now();
    manager.UpdateProgressWatchdogGoal(
        LangYa::BuffShoot.ID,
        LangYa::BuffShoot.ID,
        UnitTeam::Red,
        true,
        AreaManager::GoalPointByBaseId(LangYa::BuffShoot.ID, UnitTeam::Red),
        1,
        1,
        now);

    LangYa::NaviProgressWatchdogSetting setting;
    setting.Enable = true;
    setting.NoMoveTimeoutSec = 1;
    setting.MoveProgressCm = 10;

    const auto decision = manager.TickProgressWatchdog(NaviProgressWatchdogInput{
        .Enabled = true,
        .HasSelfPosition = true,
        .SelfX = 1,
        .SelfY = 1,
        .IsCurrentGoalArrived = false,
        .Setting = setting,
        .Now = now + std::chrono::seconds(2),
    });

    EXPECT_TRUE(decision.NeedFallback);

    const auto arrived = manager.TickProgressWatchdog(NaviProgressWatchdogInput{
        .Enabled = true,
        .HasSelfPosition = true,
        .SelfX = 1,
        .SelfY = 1,
        .IsCurrentGoalArrived = true,
        .Setting = setting,
        .Now = now + std::chrono::seconds(3),
    });
    EXPECT_FALSE(arrived.NeedFallback);
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
