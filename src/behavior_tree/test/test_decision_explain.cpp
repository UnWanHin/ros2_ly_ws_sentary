#include <gtest/gtest.h>

#include "DecisionExplain.hpp"

namespace {

BehaviorTree::DecisionExplain::NavigationObservation MakeObservation(
    const BehaviorTree::DecisionReason reason,
    const std::uint8_t base_goal,
    const std::uint8_t resolved_goal,
    const std::uint16_t x_centimeter,
    const std::uint16_t y_centimeter,
    std::string detail) {
    return BehaviorTree::DecisionExplain::NavigationObservation{
        .Intent = BehaviorTree::DecisionIntent{
            .Layer = BehaviorTree::DecisionLayerForReason(reason),
            .Reason = reason,
            .BaseGoalId = base_goal,
            .ResolvedGoalId = resolved_goal,
            .GoalTeam = LangYa::UnitTeam::Red,
            .ApplyTeamOffset = true,
            .Priority = BehaviorTree::DecisionPriorityForReason(reason),
            .Detail = std::move(detail),
        },
        .PublishedGoalId = resolved_goal,
        .XCentimeter = x_centimeter,
        .YCentimeter = y_centimeter,
        .PublishNaviGoal = true,
        .NaviGoalPublishAllowed = true,
    };
}

BehaviorTree::DecisionExplain::NavigationObservation MakeMapCommandObservation(
    const std::uint16_t x_centimeter,
    const std::uint16_t y_centimeter) {
    auto observation = MakeObservation(
        BehaviorTree::DecisionReason::MapCommand, 0U, 0U, x_centimeter, y_centimeter, "map_command");
    observation.OutputKind = BehaviorTree::DecisionExplain::NavigationOutputKind::RawMapCommand;
    return observation;
}

}  // namespace

TEST(DecisionExplain, FormatsEffectiveAreaAndTacticalSettings) {
    const BehaviorTree::DecisionExplain::ConfigSnapshot snapshot{
        .RegionalAreaTaskEnable = true,
        .MyBaseEnable = true,
        .MyHighlandEnable = false,
        .MyPreRoadlandEnable = true,
        .MyReadyRoadlandEnable = true,
        .CommonCentralEnable = false,
        .ProtectCastleEnable = true,
        .ProtectCastleRfidEnable = true,
        .ProtectCastleEnemyPosEnable = false,
        .ProtectCastleStayWhenRfid = true,
        .ProtectHeroEnable = true,
        .DamageRotateDefaultGear = 0,
        .DamageRotateNoHitTimeoutMs = 1800,
        .DamageRotateGear0HoldMs = 220,
        .DamageRotateGear1HoldMs = 220,
        .DamageRotateGear2HoldMs = 220,
        .DamageRotateScanBoostWindowMs = 1300,
        .DamageRotateScanYawPhaseMs = 160,
    };

    const auto lines = BehaviorTree::DecisionExplain::FormatConfigLines(snapshot);

    EXPECT_EQ(lines.size(), 3U);
    EXPECT_NE(lines[0].find("MyHighland=0"), std::string::npos);
    EXPECT_NE(lines[1].find("enemy_pos=0"), std::string::npos);
}

TEST(DecisionExplain, SuppressesIdenticalPublishedNavigation) {
    const auto observation = MakeObservation(
        BehaviorTree::DecisionReason::ProtectHero, 8U, 108U, 1600U, 720U, "protect_hero");

    const auto first = BehaviorTree::DecisionExplain::MakeFingerprint(observation);

    EXPECT_TRUE(first.has_value());
    EXPECT_EQ(first, BehaviorTree::DecisionExplain::MakeFingerprint(observation));
}

TEST(DecisionExplain, IncludesReasonChangeAndRawMapCommandCoordinate) {
    const auto first = MakeObservation(
        BehaviorTree::DecisionReason::DefaultAreaPolicy, 8U, 108U, 1600U, 720U, "MyHighland");
    const auto second = MakeObservation(
        BehaviorTree::DecisionReason::ProtectHero, 8U, 108U, 1600U, 720U, "protect_hero");
    const auto map = MakeMapCommandObservation(927U, 563U);

    EXPECT_NE(BehaviorTree::DecisionExplain::MakeFingerprint(first),
              BehaviorTree::DecisionExplain::MakeFingerprint(second));
    EXPECT_NE(BehaviorTree::DecisionExplain::FormatNavigationLine(map).find("raw_map_command"),
              std::string::npos);
}

TEST(DecisionExplain, FormatsFinalIntentAndCoordinateFields) {
    const auto observation = MakeObservation(
        BehaviorTree::DecisionReason::ProtectHero, 8U, 108U, 1600U, 720U, "protect_hero");
    const auto line = BehaviorTree::DecisionExplain::FormatNavigationLine(observation);

    EXPECT_NE(line.find("layer=tactical"), std::string::npos);
    EXPECT_NE(line.find("reason=protect_hero"), std::string::npos);
    EXPECT_NE(line.find("base_goal=8"), std::string::npos);
    EXPECT_NE(line.find("resolved_goal=108"), std::string::npos);
    EXPECT_NE(line.find("pos_cm=(1600,720)"), std::string::npos);
    EXPECT_NE(line.find("detail=protect_hero"), std::string::npos);
}

TEST(DecisionExplain, DoesNotProduceFingerprintForDisabledNavigation) {
    auto observation = MakeObservation(
        BehaviorTree::DecisionReason::ProtectHero, 8U, 108U, 1600U, 720U, "protect_hero");
    observation.NaviGoalPublishAllowed = false;

    EXPECT_FALSE(BehaviorTree::DecisionExplain::MakeFingerprint(observation).has_value());

    observation.NaviGoalPublishAllowed = true;
    observation.PublishNaviGoal = false;
    EXPECT_FALSE(BehaviorTree::DecisionExplain::MakeFingerprint(observation).has_value());
}

TEST(DecisionExplain, FingerprintChangesWithEveryPublishedExplanationField) {
    const auto baseline = MakeObservation(
        BehaviorTree::DecisionReason::ProtectHero, 8U, 108U, 1600U, 720U, "protect_hero");
    const auto baseline_fingerprint = BehaviorTree::DecisionExplain::MakeFingerprint(baseline);
    ASSERT_TRUE(baseline_fingerprint.has_value());

    auto changed = baseline;
    changed.Intent.Layer = BehaviorTree::DecisionLayer::Task;
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
    changed = baseline;
    changed.Intent.BaseGoalId = 9U;
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
    changed = baseline;
    changed.Intent.ResolvedGoalId = 109U;
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
    changed = baseline;
    changed.PublishedGoalId = 109U;
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
    changed = baseline;
    changed.XCentimeter = 1601U;
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
    changed = baseline;
    changed.YCentimeter = 721U;
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
    changed = baseline;
    changed.Intent.GoalTeam = LangYa::UnitTeam::Blue;
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
    changed = baseline;
    changed.Intent.ApplyTeamOffset = false;
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
    changed = baseline;
    ++changed.Intent.Priority;
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
    changed = baseline;
    changed.Intent.Detail = "different_detail";
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
    changed = baseline;
    changed.OutputKind = BehaviorTree::DecisionExplain::NavigationOutputKind::RawMapCommand;
    EXPECT_NE(baseline_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(changed));
}

TEST(DecisionExplain, FormatsManualMapPoseAndRelativeTargetWithTheirActualUnits) {
    auto manual = MakeObservation(
        BehaviorTree::DecisionReason::AimModeOutpost, 7U, 107U, 0U, 0U, "outpost_task");
    manual.OutputKind = BehaviorTree::DecisionExplain::NavigationOutputKind::ManualMapPose;
    manual.XMeter = 8.5F;
    manual.YMeter = 3.2F;
    manual.ZMeter = 0.0F;
    manual.FrameId = "map";
    const auto manual_line = BehaviorTree::DecisionExplain::FormatNavigationLine(manual);
    EXPECT_NE(manual_line.find("manual_outpost_goal_pose"), std::string::npos);
    EXPECT_NE(manual_line.find("pos_map_m=(8.5,3.2,0)"), std::string::npos);

    auto relative = MakeObservation(
        BehaviorTree::DecisionReason::Chase, 8U, 108U, 0U, 0U, "chase");
    relative.OutputKind = BehaviorTree::DecisionExplain::NavigationOutputKind::RelativeTarget;
    relative.XMeter = 2.5F;
    relative.YMeter = -0.5F;
    relative.ZMeter = 0.1F;
    relative.FrameId = "gimbal_world";
    relative.RelativeTargetValid = true;
    relative.RelativeTargetArmorType = 3U;
    relative.RelativeTargetAimMode = 1U;
    const auto relative_line = BehaviorTree::DecisionExplain::FormatNavigationLine(relative);
    EXPECT_NE(relative_line.find("relative_target"), std::string::npos);
    EXPECT_NE(relative_line.find("rel_m=(2.5,-0.5,0.1)"), std::string::npos);
    EXPECT_NE(relative_line.find("frame=gimbal_world"), std::string::npos);
    EXPECT_NE(relative_line.find("reason=chase"), std::string::npos);

    const auto relative_fingerprint = BehaviorTree::DecisionExplain::MakeFingerprint(relative);
    relative.XMeter = 3.0F;
    EXPECT_EQ(relative_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(relative));
    relative.RelativeTargetArmorType = 4U;
    EXPECT_NE(relative_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(relative));

    const auto manual_fingerprint = BehaviorTree::DecisionExplain::MakeFingerprint(manual);
    manual.XMeter = 8.6F;
    EXPECT_NE(manual_fingerprint, BehaviorTree::DecisionExplain::MakeFingerprint(manual));
}

TEST(DecisionExplain, FormatsHardRecoveryAndStablePatrolDetails) {
    const auto recovery = MakeObservation(
        BehaviorTree::DecisionReason::Recovery,
        2U,
        102U,
        250U,
        200U,
        "enter_default hp=120 ammo=45");
    const auto recovery_line = BehaviorTree::DecisionExplain::FormatNavigationLine(recovery);
    EXPECT_NE(recovery_line.find("layer=hard"), std::string::npos);
    EXPECT_NE(recovery_line.find("reason=recovery"), std::string::npos);
    EXPECT_NE(recovery_line.find("detail=enter_default hp=120 ammo=45"), std::string::npos);

    const auto area_task = MakeObservation(
        BehaviorTree::DecisionReason::DefaultAreaPolicy,
        8U,
        108U,
        1600U,
        720U,
        "area_task=MyHighland phase=patrol");
    EXPECT_NE(
        BehaviorTree::DecisionExplain::FormatNavigationLine(area_task)
            .find("detail=area_task=MyHighland phase=patrol"),
        std::string::npos);

    const auto idle = MakeObservation(
        BehaviorTree::DecisionReason::RegionalIdlePatrol,
        9U,
        109U,
        1200U,
        800U,
        "index=2 hold_sec=8");
    EXPECT_NE(
        BehaviorTree::DecisionExplain::FormatNavigationLine(idle)
            .find("detail=index=2 hold_sec=8"),
        std::string::npos);
}
