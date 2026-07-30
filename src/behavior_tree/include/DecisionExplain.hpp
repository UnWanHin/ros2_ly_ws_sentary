// AUTO-COMMENT: Pure formatting and de-duplication support for final navigation decisions.

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>

#include "DecisionIntent.hpp"

namespace BehaviorTree::DecisionExplain {

enum class NavigationOutputKind : std::uint8_t {
    GoalPoint = 0,
    RawMapCommand = 1,
    RelativeTarget = 2,
    ManualMapPose = 3,
};

struct ConfigSnapshot {
    bool RegionalAreaTaskEnable{false};
    bool MyBaseEnable{false};
    bool MyHighlandEnable{false};
    bool MyPreRoadlandEnable{false};
    bool MyReadyRoadlandEnable{false};
    bool CommonCentralEnable{false};
    bool ProtectCastleEnable{false};
    bool ProtectCastleBaseEnable{false};
    bool ProtectCastleRfidEnable{false};
    bool ProtectCastleEnemyPosEnable{false};
    bool ProtectCastleStayWhenRfid{false};
    bool ProtectOutpostEnable{false};
    int ProtectOutpostHealthFreshMs{0};
    int ProtectOutpostDamageWindowMs{0};
    int ProtectOutpostDamageThresholdHp{0};
    int ProtectOutpostSearchHoldSec{0};
    int ProtectOutpostUnreachableCooldownSec{0};
    bool ProtectHeroEnable{false};
    bool ProtectHeroProactiveHoldWhenHeroInHighland{false};
    int ProtectHeroStartElapsedSec{0};
    int ProtectHeroHoldSec{0};
    int ProtectHeroNoEnemyReleaseSec{0};
    int ProtectHeroFriendPositionFreshMs{0};
    int ProtectHeroFriendHealthFreshMs{0};
    std::uint8_t ProtectHeroGoalBaseId{0};
    int ProtectCastlePriority{0};
    int ProtectOutpostPriority{0};
    int ProtectHeroPriority{0};
    int ChasePriority{0};
    std::uint8_t DamageRotateDefaultGear{0};
    int DamageRotateNoHitTimeoutMs{0};
    int DamageRotateGear0HoldMs{0};
    int DamageRotateGear1HoldMs{0};
    int DamageRotateGear2HoldMs{0};
    int DamageRotateScanBoostWindowMs{0};
    int DamageRotateScanYawPhaseMs{0};
};

struct NavigationObservation {
    DecisionIntent Intent{};
    std::uint8_t PublishedGoalId{0};
    std::uint16_t XCentimeter{0};
    std::uint16_t YCentimeter{0};
    bool PublishNaviGoal{false};
    bool NaviGoalPublishAllowed{false};
    NavigationOutputKind OutputKind{NavigationOutputKind::GoalPoint};
    float XMeter{0.0F};
    float YMeter{0.0F};
    float ZMeter{0.0F};
    std::string FrameId{};
    bool RelativeTargetValid{false};
    std::uint8_t RelativeTargetArmorType{0};
    std::uint8_t RelativeTargetAimMode{0};
};

struct Fingerprint {
    DecisionLayer Layer{DecisionLayer::Unknown};
    DecisionReason Reason{DecisionReason::Unknown};
    std::uint8_t BaseGoalId{0};
    std::uint8_t ResolvedGoalId{0};
    LangYa::UnitTeam GoalTeam{LangYa::UnitTeam::Unknown};
    bool ApplyTeamOffset{false};
    int Priority{0};
    std::string Detail{};
    std::uint8_t PublishedGoalId{0};
    std::uint16_t XCentimeter{0};
    std::uint16_t YCentimeter{0};
    NavigationOutputKind OutputKind{NavigationOutputKind::GoalPoint};
    float XMeter{0.0F};
    float YMeter{0.0F};
    float ZMeter{0.0F};
    std::string FrameId{};
    bool RelativeTargetValid{false};
    std::uint8_t RelativeTargetArmorType{0};
    std::uint8_t RelativeTargetAimMode{0};

    bool operator==(const Fingerprint&) const = default;
};

inline std::array<std::string, 3> FormatConfigLines(const ConfigSnapshot& snapshot) {
    const auto enabled = [](const bool value) { return value ? 1 : 0; };

    std::ostringstream area;
    area << "[DecisionExplain][config] area regional_task="
         << enabled(snapshot.RegionalAreaTaskEnable)
         << " MyBase=" << enabled(snapshot.MyBaseEnable)
         << " MyHighland=" << enabled(snapshot.MyHighlandEnable)
         << " MyPreRoadland=" << enabled(snapshot.MyPreRoadlandEnable)
         << " MyReadyRoadland=" << enabled(snapshot.MyReadyRoadlandEnable)
         << " CommonCentral=" << enabled(snapshot.CommonCentralEnable);

    std::ostringstream tactical;
    tactical << "[DecisionExplain][config] tactical protect_castle="
             << enabled(snapshot.ProtectCastleEnable)
             << " base=" << enabled(snapshot.ProtectCastleBaseEnable)
             << " rfid=" << enabled(snapshot.ProtectCastleRfidEnable)
             << " enemy_pos=" << enabled(snapshot.ProtectCastleEnemyPosEnable)
             << " stay_when_rfid=" << enabled(snapshot.ProtectCastleStayWhenRfid)
             << " protect_outpost=" << (snapshot.ProtectOutpostEnable ? "enabled" : "disabled")
             << " health_fresh_ms=" << snapshot.ProtectOutpostHealthFreshMs
             << " damage_window_ms=" << snapshot.ProtectOutpostDamageWindowMs
             << " damage_threshold_hp=" << snapshot.ProtectOutpostDamageThresholdHp
             << " search_hold_sec=" << snapshot.ProtectOutpostSearchHoldSec
             << " unreachable_cooldown_sec=" << snapshot.ProtectOutpostUnreachableCooldownSec
             << " protect_hero=" << enabled(snapshot.ProtectHeroEnable)
             << " hero_proactive_hold="
             << enabled(snapshot.ProtectHeroProactiveHoldWhenHeroInHighland)
             << " hero_start_elapsed_sec=" << snapshot.ProtectHeroStartElapsedSec
             << " hero_hold_sec=" << snapshot.ProtectHeroHoldSec
             << " hero_no_enemy_release_sec=" << snapshot.ProtectHeroNoEnemyReleaseSec
             << " hero_position_fresh_ms=" << snapshot.ProtectHeroFriendPositionFreshMs
             << " hero_health_fresh_ms=" << snapshot.ProtectHeroFriendHealthFreshMs
             << " hero_goal_base_id=" << static_cast<unsigned int>(snapshot.ProtectHeroGoalBaseId)
             << " priority.protect_castle=" << snapshot.ProtectCastlePriority
             << " priority.protect_outpost=" << snapshot.ProtectOutpostPriority
             << " priority.protect_hero=" << snapshot.ProtectHeroPriority
             << " priority.chase=" << snapshot.ChasePriority;

    std::ostringstream damage_rotate;
    damage_rotate << "[DecisionExplain][config] damage_rotate default_gear="
                  << static_cast<unsigned int>(snapshot.DamageRotateDefaultGear)
                  << " no_hit_timeout_ms=" << snapshot.DamageRotateNoHitTimeoutMs
                  << " gear0_hold_ms=" << snapshot.DamageRotateGear0HoldMs
                  << " gear1_hold_ms=" << snapshot.DamageRotateGear1HoldMs
                  << " gear2_hold_ms=" << snapshot.DamageRotateGear2HoldMs
                  << " scan_boost_window_ms=" << snapshot.DamageRotateScanBoostWindowMs
                  << " scan_yaw_phase_ms=" << snapshot.DamageRotateScanYawPhaseMs;

    return {area.str(), tactical.str(), damage_rotate.str()};
}

inline std::optional<Fingerprint> MakeFingerprint(const NavigationObservation& observation) {
    if (!observation.PublishNaviGoal || !observation.NaviGoalPublishAllowed) {
        return std::nullopt;
    }

    Fingerprint fingerprint{
        .Layer = observation.Intent.Layer,
        .Reason = observation.Intent.Reason,
        .BaseGoalId = observation.Intent.BaseGoalId,
        .ResolvedGoalId = observation.Intent.ResolvedGoalId,
        .GoalTeam = observation.Intent.GoalTeam,
        .ApplyTeamOffset = observation.Intent.ApplyTeamOffset,
        .Priority = observation.Intent.Priority,
        .Detail = observation.Intent.Detail,
        .PublishedGoalId = observation.PublishedGoalId,
        .XCentimeter = observation.XCentimeter,
        .YCentimeter = observation.YCentimeter,
        .OutputKind = observation.OutputKind,
        .FrameId = observation.FrameId,
        .RelativeTargetValid = observation.RelativeTargetValid,
        .RelativeTargetArmorType = observation.RelativeTargetArmorType,
        .RelativeTargetAimMode = observation.RelativeTargetAimMode,
    };
    if (observation.OutputKind == NavigationOutputKind::ManualMapPose) {
        fingerprint.XMeter = observation.XMeter;
        fingerprint.YMeter = observation.YMeter;
        fingerprint.ZMeter = observation.ZMeter;
    }
    return fingerprint;
}

inline const char* UnitTeamToString(const LangYa::UnitTeam team) noexcept {
    switch (team) {
        case LangYa::UnitTeam::Red: return "red";
        case LangYa::UnitTeam::Blue: return "blue";
        default: return "unknown";
    }
}

inline std::string FormatNavigationLine(const NavigationObservation& observation) {
    std::ostringstream line;
    line << "[DecisionExplain][navi] destination=";
    switch (observation.OutputKind) {
        case NavigationOutputKind::RawMapCommand:
            line << "raw_map_command"
                 << " pos_cm=(" << observation.XCentimeter << ',' << observation.YCentimeter << ')';
            break;
        case NavigationOutputKind::RelativeTarget:
            line << "relative_target"
                 << " target_valid=" << (observation.RelativeTargetValid ? 1 : 0)
                 << " rel_m=(" << observation.XMeter << ',' << observation.YMeter << ','
                 << observation.ZMeter << ')'
                 << " frame=" << (observation.FrameId.empty() ? "<default>" : observation.FrameId)
                 << " armor_type=" << static_cast<unsigned int>(observation.RelativeTargetArmorType)
                 << " aim_mode=" << static_cast<unsigned int>(observation.RelativeTargetAimMode);
            break;
        case NavigationOutputKind::ManualMapPose:
            line << "manual_outpost_goal_pose"
                 << " pos_map_m=(" << observation.XMeter << ',' << observation.YMeter << ','
                 << observation.ZMeter << ')'
                 << " frame=" << (observation.FrameId.empty() ? "map" : observation.FrameId);
            break;
        case NavigationOutputKind::GoalPoint:
        default:
            line << "goal_id=" << static_cast<unsigned int>(observation.PublishedGoalId)
                 << " pos_cm=(" << observation.XCentimeter << ',' << observation.YCentimeter << ')';
            break;
    }
    line << " layer=" << DecisionLayerToString(observation.Intent.Layer)
         << " reason=" << DecisionReasonToString(observation.Intent.Reason)
         << " base_goal=" << static_cast<unsigned int>(observation.Intent.BaseGoalId)
         << " resolved_goal=" << static_cast<unsigned int>(observation.Intent.ResolvedGoalId)
         << " team=" << UnitTeamToString(observation.Intent.GoalTeam)
         << " apply_team_offset=" << (observation.Intent.ApplyTeamOffset ? 1 : 0)
         << " priority=" << observation.Intent.Priority
         << " detail=" << observation.Intent.Detail;
    return line.str();
}

}  // namespace BehaviorTree::DecisionExplain
