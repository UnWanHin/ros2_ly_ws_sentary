#include "../include/ChasePolicy.hpp"

#include <gtest/gtest.h>

namespace {

using BehaviorTree::Area::MainAreaKind;
using BehaviorTree::AreaKey;
using BehaviorTree::AreaSide;
using BehaviorTree::ChasePolicyContext;
using BehaviorTree::EvaluateRegionalChasePolicy;
using BehaviorTree::ResolvedAreaKey;
using BehaviorTree::RegionalAreaTaskRuntime;
using BehaviorTree::RegionalAreaTaskType;
using LangYa::ChasePolicySetting;
using LangYa::UnitTeam;

ChasePolicySetting EnabledPolicy() {
    ChasePolicySetting setting;
    setting.Enable = true;
    setting.MyHighland = true;
    setting.MyPreRoadland = true;
    setting.CommonCentral = true;
    return setting;
}

RegionalAreaTaskRuntime HighlandPlan() {
    RegionalAreaTaskRuntime task;
    task.Active = true;
    task.Type = RegionalAreaTaskType::MyHighland;
    task.OwnerTeam = UnitTeam::Red;
    return task;
}

ChasePolicyContext SameHighlandContext() {
    ChasePolicyContext context;
    context.RegionalProfile = true;
    context.YieldablePlan = true;
    context.Plan = HighlandPlan();
    context.TargetPositionFresh = true;
    context.TargetArea = ResolvedAreaKey{
        .Key = AreaKey{.Side = AreaSide::My, .Kind = MainAreaKind::Highland, .Team = UnitTeam::Red},
        .UsedNearestFallback = false};
    return context;
}

ChasePolicyContext CastlePerimeterContext() {
    ChasePolicyContext context;
    context.RegionalProfile = true;
    context.TargetPositionFresh = true;
    context.ExplicitAllowedArea = AreaKey{
        .Side = AreaSide::My,
        .Kind = MainAreaKind::Base,
        .Team = UnitTeam::Red};
    context.TargetArea = ResolvedAreaKey{
        .Key = *context.ExplicitAllowedArea,
        .UsedNearestFallback = false};
    return context;
}

}  // namespace

TEST(ChasePolicyTest, AllowsFreshExactTargetInEnabledPlannedArea) {
    const auto result = EvaluateRegionalChasePolicy(EnabledPolicy(), SameHighlandContext());
    EXPECT_TRUE(result.Allowed);
}

TEST(ChasePolicyTest, MapsEveryDefaultRegionalTaskToItsPlannedArea) {
    const auto expect_area = [](const RegionalAreaTaskType type,
                                const AreaSide side,
                                const MainAreaKind kind,
                                const UnitTeam team) {
        RegionalAreaTaskRuntime task;
        task.Active = true;
        task.Type = type;
        task.OwnerTeam = UnitTeam::Red;
        const auto area = BehaviorTree::PlannedAreaKeyForChase(task);
        ASSERT_TRUE(area.has_value());
        EXPECT_EQ(area->Side, side);
        EXPECT_EQ(area->Kind, kind);
        EXPECT_EQ(area->Team, team);
    };

    expect_area(RegionalAreaTaskType::MyBase, AreaSide::My, MainAreaKind::Base, UnitTeam::Red);
    expect_area(RegionalAreaTaskType::MyHighland, AreaSide::My, MainAreaKind::Highland, UnitTeam::Red);
    expect_area(RegionalAreaTaskType::MyPreRoadland, AreaSide::My, MainAreaKind::PreRoadland, UnitTeam::Red);
    expect_area(RegionalAreaTaskType::MyReadyRoadland, AreaSide::My, MainAreaKind::ReadyRoadland, UnitTeam::Red);
    expect_area(RegionalAreaTaskType::CommonCentral, AreaSide::Common, MainAreaKind::Central, UnitTeam::Unknown);
}

TEST(ChasePolicyTest, RejectsTargetInAnotherEnabledArea) {
    auto context = SameHighlandContext();
    context.TargetArea = ResolvedAreaKey{
        .Key = AreaKey{.Side = AreaSide::Common, .Kind = MainAreaKind::Central, .Team = UnitTeam::Unknown},
        .UsedNearestFallback = false};

    EXPECT_FALSE(EvaluateRegionalChasePolicy(EnabledPolicy(), context).Allowed);
}

TEST(ChasePolicyTest, RejectsNearestAreaFallback) {
    auto context = SameHighlandContext();
    context.TargetArea->UsedNearestFallback = true;

    EXPECT_FALSE(EvaluateRegionalChasePolicy(EnabledPolicy(), context).Allowed);
}

TEST(ChasePolicyTest, RejectsDisabledPlannedArea) {
    auto setting = EnabledPolicy();
    setting.MyHighland = false;

    EXPECT_FALSE(EvaluateRegionalChasePolicy(setting, SameHighlandContext()).Allowed);
}

TEST(ChasePolicyTest, RejectsStaleOrMissingTargetPosition) {
    auto context = SameHighlandContext();
    context.TargetPositionFresh = false;
    EXPECT_FALSE(EvaluateRegionalChasePolicy(EnabledPolicy(), context).Allowed);

    context.TargetPositionFresh = true;
    context.TargetArea.reset();
    EXPECT_FALSE(EvaluateRegionalChasePolicy(EnabledPolicy(), context).Allowed);
}

TEST(ChasePolicyTest, RejectsMissingOrUnyieldablePlan) {
    auto context = SameHighlandContext();
    context.Plan.reset();
    EXPECT_FALSE(EvaluateRegionalChasePolicy(EnabledPolicy(), context).Allowed);

    context = SameHighlandContext();
    context.YieldablePlan = false;
    EXPECT_FALSE(EvaluateRegionalChasePolicy(EnabledPolicy(), context).Allowed);
}

TEST(ChasePolicyTest, CastlePerimeterDefenseAllowsFreshExactMyBaseTargetWithoutDefaultPlan) {
    auto setting = EnabledPolicy();
    setting.MyBase = true;

    const auto result = EvaluateRegionalChasePolicy(setting, CastlePerimeterContext());

    EXPECT_TRUE(result.Allowed);
}

TEST(ChasePolicyTest, CastlePerimeterDefenseRejectsStaleCrossAreaNearestOrDisabledTarget) {
    auto setting = EnabledPolicy();
    setting.MyBase = true;
    auto context = CastlePerimeterContext();

    context.TargetPositionFresh = false;
    EXPECT_FALSE(EvaluateRegionalChasePolicy(setting, context).Allowed);

    context = CastlePerimeterContext();
    context.TargetArea = ResolvedAreaKey{
        .Key = AreaKey{.Side = AreaSide::My, .Kind = MainAreaKind::Highland, .Team = UnitTeam::Red},
        .UsedNearestFallback = false};
    EXPECT_FALSE(EvaluateRegionalChasePolicy(setting, context).Allowed);

    context = CastlePerimeterContext();
    context.TargetArea->UsedNearestFallback = true;
    EXPECT_FALSE(EvaluateRegionalChasePolicy(setting, context).Allowed);

    setting.MyBase = false;
    EXPECT_FALSE(EvaluateRegionalChasePolicy(setting, CastlePerimeterContext()).Allowed);
}

TEST(ChasePolicyTest, RejectsSideMismatchAndPolicyDisable) {
    auto context = SameHighlandContext();
    context.TargetArea = ResolvedAreaKey{
        .Key = AreaKey{.Side = AreaSide::Enemy, .Kind = MainAreaKind::Highland, .Team = UnitTeam::Blue},
        .UsedNearestFallback = false};
    EXPECT_FALSE(EvaluateRegionalChasePolicy(EnabledPolicy(), context).Allowed);

    auto setting = EnabledPolicy();
    setting.Enable = false;
    EXPECT_FALSE(EvaluateRegionalChasePolicy(setting, SameHighlandContext()).Allowed);
}

TEST(ChasePolicyTest, NonRegionalProfilesKeepExistingChaseBehavior) {
    auto context = SameHighlandContext();
    context.RegionalProfile = false;
    context.Plan.reset();
    context.TargetArea.reset();
    context.TargetPositionFresh = false;

    EXPECT_TRUE(EvaluateRegionalChasePolicy(ChasePolicySetting{}, context).Allowed);
}
