#include "../include/FaceModeManager.hpp"

#include <gtest/gtest.h>

namespace {

LangYa::AimData FreshFaceAngles() {
    LangYa::AimData data;
    data.Valid = true;
    data.Fresh = true;
    data.HasLatchedAngles = true;
    data.LastValidTime = std::chrono::steady_clock::now();
    data.Angles = LangYa::GimbalAnglesType{12.0f, -3.0f};
    return data;
}

LangYa::FaceModeSetting EnabledFaceMode() {
    LangYa::FaceModeSetting setting;
    setting.Enable = true;
    setting.SuppressFire = true;
    setting.LostTargetHoldMs = 300;
    return setting;
}

BehaviorTree::RegionalAreaTaskTickResult RegionalFaceRequest() {
    BehaviorTree::RegionalAreaTaskTickResult result;
    result.Active = true;
    result.UseFaceMode = true;
    result.Phase = BehaviorTree::RegionalAreaTaskPhase::ApproachHighland;
    return result;
}

}  // namespace

TEST(FaceModeManagerTest, RegionalRequestResolvesToSingleActiveDecision) {
    BehaviorTree::FaceModeManager manager;
    manager.BeginCycle();
    manager.RequestRegionalTask(RegionalFaceRequest(), nullptr);

    const auto decision = manager.Resolve(
        FreshFaceAngles(), EnabledFaceMode(), LangYa::PatrolScanSetting{}, false, false, false,
        std::chrono::steady_clock::now());

    EXPECT_TRUE(decision.Requested);
    EXPECT_TRUE(decision.Active);
    EXPECT_TRUE(decision.SuppressFire);
    EXPECT_EQ(decision.RequestSource, BehaviorTree::FaceModeManager::Source::Regional);
    ASSERT_TRUE(decision.Angles.has_value());
    EXPECT_FLOAT_EQ(decision.Angles->Yaw, 12.0f);
}

TEST(FaceModeManagerTest, VisualAimAndNavigationOnlySuppressFaceOutputNotRequestState) {
    BehaviorTree::FaceModeManager manager;
    manager.BeginCycle();
    manager.RequestRegionalTask(RegionalFaceRequest(), nullptr);

    const auto visual_priority = manager.Resolve(
        FreshFaceAngles(), EnabledFaceMode(), LangYa::PatrolScanSetting{}, true, false, false,
        std::chrono::steady_clock::now());
    EXPECT_TRUE(visual_priority.Requested);
    EXPECT_FALSE(visual_priority.Active);

    const auto navigation_suppressed = manager.Resolve(
        FreshFaceAngles(), EnabledFaceMode(), LangYa::PatrolScanSetting{}, false, true, true,
        std::chrono::steady_clock::now());
    EXPECT_TRUE(navigation_suppressed.Requested);
    EXPECT_FALSE(navigation_suppressed.Active);
}

TEST(FaceModeManagerTest, AimRequestOverridesRegionalRequestWithinSameCycle) {
    BehaviorTree::FaceModeManager manager;
    manager.BeginCycle();
    manager.RequestRegionalTask(RegionalFaceRequest(), nullptr);
    manager.RequestAimTarget(LangYa::AimMode::Outpost, LangYa::UnitTeam::Red, nullptr);

    const auto decision = manager.Resolve(
        FreshFaceAngles(), EnabledFaceMode(), LangYa::PatrolScanSetting{}, false, true, true,
        std::chrono::steady_clock::now());

    EXPECT_TRUE(decision.Active);
    EXPECT_EQ(decision.RequestSource, BehaviorTree::FaceModeManager::Source::Outpost);
}

TEST(FaceModeManagerTest, MissingAnglesUsesConfiguredPatrolFallback) {
    BehaviorTree::FaceModeManager manager;
    manager.BeginCycle();
    manager.RequestRegionalTask(RegionalFaceRequest(), nullptr);
    LangYa::AimData missing_angles;
    LangYa::PatrolScanSetting patrol;
    patrol.FaceModeFallbackEnable = true;

    const auto decision = manager.Resolve(
        missing_angles, EnabledFaceMode(), patrol, false, false, false,
        std::chrono::steady_clock::now());

    EXPECT_TRUE(decision.Requested);
    EXPECT_FALSE(decision.Active);
    EXPECT_TRUE(decision.UsePatrolFallback);
}

TEST(FaceModeManagerTest, StartGateOutpostRequestFallsBackToOutpostPatrolWithoutAngles) {
    BehaviorTree::FaceModeManager manager;
    manager.BeginCycle();
    manager.RequestStartGateOutpost(LangYa::UnitTeam::Red, nullptr);

    LangYa::AimData missing_angles;
    LangYa::PatrolScanSetting patrol;
    patrol.FaceModeFallbackEnable = true;
    patrol.OutpostFaceModeFallbackMode = 3;

    const auto decision = manager.Resolve(
        missing_angles, EnabledFaceMode(), patrol, false, false, false,
        std::chrono::steady_clock::now());

    EXPECT_TRUE(decision.Requested);
    EXPECT_FALSE(decision.Active);
    EXPECT_TRUE(decision.UsePatrolFallback);
    EXPECT_EQ(decision.RequestSource, BehaviorTree::FaceModeManager::Source::StartGate);
}

TEST(FaceModeManagerTest, StartGateAcceptsOnlyFreshPostRequestNonManualSolution) {
    EXPECT_TRUE(BehaviorTree::FaceModeManager::StartGateOutpostSolutionReady(
        true, true, false, 8, 7, true));
    EXPECT_FALSE(BehaviorTree::FaceModeManager::StartGateOutpostSolutionReady(
        false, true, false, 8, 7, true));
    EXPECT_FALSE(BehaviorTree::FaceModeManager::StartGateOutpostSolutionReady(
        true, true, true, 8, 7, true));
    EXPECT_FALSE(BehaviorTree::FaceModeManager::StartGateOutpostSolutionReady(
        true, true, false, 7, 7, true));
    EXPECT_FALSE(BehaviorTree::FaceModeManager::StartGateOutpostSolutionReady(
        true, true, false, 8, 7, false));
}

TEST(FaceModeManagerTest, StartGateKeepsLatchedAnglesBetweenSolverCallbacks) {
    const auto now = std::chrono::steady_clock::now();
    auto held_angles = FreshFaceAngles();
    held_angles.Fresh = false;
    held_angles.LastValidTime = now - std::chrono::milliseconds(120);

    EXPECT_TRUE(BehaviorTree::FaceModeManager::AnglesUsable(
        held_angles, EnabledFaceMode(), now));
    EXPECT_FALSE(BehaviorTree::FaceModeManager::AnglesUsable(
        held_angles, EnabledFaceMode(), now + std::chrono::milliseconds(301)));
}

TEST(FaceModeManagerTest, StartGateReadmitsFaceModeAfterSolverRecovers) {
    using Readiness = BehaviorTree::FaceModeManager::StartGateOutpostReadiness;

    EXPECT_EQ(
        BehaviorTree::FaceModeManager::DiagnoseStartGateOutpostSolution(
            true, true, false, false, 8, 7, true),
        Readiness::SolverFunctionDisabled);
    EXPECT_EQ(
        BehaviorTree::FaceModeManager::DiagnoseStartGateOutpostSolution(
            true, true, true, false, 9, 7, true),
        Readiness::Ready);
}

TEST(FaceModeManagerTest, StartGateDiagnosticNamesTheFirstRejectedGate) {
    using Readiness = BehaviorTree::FaceModeManager::StartGateOutpostReadiness;

    EXPECT_EQ(
        BehaviorTree::FaceModeManager::DiagnoseStartGateOutpostSolution(
            false, false, false, true, 0, 0, false),
        Readiness::StatusMissing);
    EXPECT_EQ(
        BehaviorTree::FaceModeManager::DiagnoseStartGateOutpostSolution(
            true, false, false, true, 0, 0, false),
        Readiness::StatusStale);
    EXPECT_EQ(
        BehaviorTree::FaceModeManager::DiagnoseStartGateOutpostSolution(
            true, true, false, true, 0, 0, false),
        Readiness::SolverFunctionDisabled);
    EXPECT_EQ(
        BehaviorTree::FaceModeManager::DiagnoseStartGateOutpostSolution(
            true, true, true, true, 0, 0, false),
        Readiness::ManualTarget);
    EXPECT_EQ(
        BehaviorTree::FaceModeManager::DiagnoseStartGateOutpostSolution(
            true, true, true, false, 7, 7, true),
        Readiness::TargetNotUpdated);
    EXPECT_EQ(
        BehaviorTree::FaceModeManager::DiagnoseStartGateOutpostSolution(
            true, true, true, false, 8, 7, false),
        Readiness::AnglesUnavailable);
    EXPECT_EQ(
        BehaviorTree::FaceModeManager::DiagnoseStartGateOutpostSolution(
            true, true, true, false, 8, 7, true),
        Readiness::Ready);
    EXPECT_STREQ(
        BehaviorTree::FaceModeManager::StartGateOutpostReadinessName(Readiness::TargetNotUpdated),
        "target_generation_not_advanced");
}
