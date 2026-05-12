// AUTO-COMMENT: file overview
// Typed decision intent metadata for behavior_tree navigation decisions.

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

enum class DecisionLayer : std::uint8_t {
    Unknown = 0,
    Hard = 1,
    Default = 2,
    Task = 3,
    Tactical = 4,
    RegionalDefense = 5,
    AimMode = 6,
    Watchdog = 7,
    IdlePatrol = 8,
    Special = 9
};

enum class DecisionReason : std::uint8_t {
    Unknown = 0,
    AreaScopeBlocked = 1,
    DefaultAreaPolicy = 2,
    RegionalDefense = 3,
    OwnBaseMulti = 4,
    OwnBaseChase = 5,
    OwnBaseGuard = 6,
    OwnHighlandRoadland = 7,
    OwnRoadland = 8,
    OwnHighland = 9,
    CommonCentral = 10,
    EnemyRoadlandSoft = 11,
    EnemyHighlandSoft = 12,
    AimModeBuff = 13,
    AimModeOutpost = 14,
    NaviProgressWatchdog = 15,
    RegionalIdlePatrol = 16,
    AreaTask = 17,
    NaviTransition = 18,
    Start = 19,
    OwnFortressGainPointEnemy = 20,
    ProtectHero = 21,
    SpecialMiniRoadland = 22,
    SpecialPatrol = 23
};

struct DecisionIntent {
    DecisionLayer Layer{DecisionLayer::Unknown};
    DecisionReason Reason{DecisionReason::Unknown};
    std::uint8_t BaseGoalId{LangYa::Home.ID};
    std::uint8_t ResolvedGoalId{0};
    LangYa::UnitTeam GoalTeam{LangYa::UnitTeam::Unknown};
    bool ApplyTeamOffset{true};
    int Priority{0};
    std::string Detail{};
};

inline const char* DecisionLayerToString(const DecisionLayer layer) noexcept {
    switch (layer) {
        case DecisionLayer::Hard: return "hard";
        case DecisionLayer::Default: return "default";
        case DecisionLayer::Task: return "task";
        case DecisionLayer::Tactical: return "tactical";
        case DecisionLayer::RegionalDefense: return "regional_defense";
        case DecisionLayer::AimMode: return "aim_mode";
        case DecisionLayer::Watchdog: return "watchdog";
        case DecisionLayer::IdlePatrol: return "idle_patrol";
        case DecisionLayer::Special: return "special";
        default: return "unknown";
    }
}

inline const char* DecisionReasonToString(const DecisionReason reason) noexcept {
    switch (reason) {
        case DecisionReason::AreaScopeBlocked: return "area_scope_blocked";
        case DecisionReason::DefaultAreaPolicy: return "default_area_policy";
        case DecisionReason::RegionalDefense: return "regional_defense";
        case DecisionReason::OwnBaseMulti: return "own_base_multi";
        case DecisionReason::OwnBaseChase: return "own_base_chase";
        case DecisionReason::OwnBaseGuard: return "own_base_guard";
        case DecisionReason::OwnHighlandRoadland: return "own_highland_roadland";
        case DecisionReason::OwnRoadland: return "own_roadland";
        case DecisionReason::OwnHighland: return "own_highland";
        case DecisionReason::CommonCentral: return "common_central";
        case DecisionReason::EnemyRoadlandSoft: return "enemy_roadland_soft";
        case DecisionReason::EnemyHighlandSoft: return "enemy_highland_soft";
        case DecisionReason::AimModeBuff: return "regional_tactical_buff_mode";
        case DecisionReason::AimModeOutpost: return "regional_tactical_aim_mode";
        case DecisionReason::NaviProgressWatchdog: return "navi_progress_watchdog";
        case DecisionReason::RegionalIdlePatrol: return "regional_idle_patrol";
        case DecisionReason::AreaTask: return "area_task";
        case DecisionReason::NaviTransition: return "navi_transition";
        case DecisionReason::Start: return "start";
        case DecisionReason::OwnFortressGainPointEnemy: return "own_fortress_gain_point_enemy";
        case DecisionReason::ProtectHero: return "protect_hero";
        case DecisionReason::SpecialMiniRoadland: return "special_mini_roadland";
        case DecisionReason::SpecialPatrol: return "special_patrol";
        default: return "unknown";
    }
}

inline DecisionReason DecisionReasonFromString(const std::string_view reason) noexcept {
    if (reason == "area_scope" || reason == "area_scope_blocked") return DecisionReason::AreaScopeBlocked;
    if (reason == "default_area_policy") return DecisionReason::DefaultAreaPolicy;
    if (reason == "regional_defense") return DecisionReason::RegionalDefense;
    if (reason == "own_base_multi") return DecisionReason::OwnBaseMulti;
    if (reason == "own_base_chase") return DecisionReason::OwnBaseChase;
    if (reason == "own_base_guard") return DecisionReason::OwnBaseGuard;
    if (reason == "own_highland_roadland") return DecisionReason::OwnHighlandRoadland;
    if (reason == "own_roadland") return DecisionReason::OwnRoadland;
    if (reason == "own_highland") return DecisionReason::OwnHighland;
    if (reason == "common_central") return DecisionReason::CommonCentral;
    if (reason == "enemy_roadland_soft") return DecisionReason::EnemyRoadlandSoft;
    if (reason == "enemy_highland_soft") return DecisionReason::EnemyHighlandSoft;
    if (reason == "regional_tactical_buff_mode") return DecisionReason::AimModeBuff;
    if (reason == "regional_tactical_aim_mode") return DecisionReason::AimModeOutpost;
    if (reason == "navi_progress_watchdog") return DecisionReason::NaviProgressWatchdog;
    if (reason == "regional_idle_patrol") return DecisionReason::RegionalIdlePatrol;
    if (reason == "area_task") return DecisionReason::AreaTask;
    if (reason == "navi_transition") return DecisionReason::NaviTransition;
    if (reason == "start") return DecisionReason::Start;
    if (reason == "own_fortress_gain_point_enemy") return DecisionReason::OwnFortressGainPointEnemy;
    if (reason == "protect_hero") return DecisionReason::ProtectHero;
    if (reason == "special_mini_roadland") return DecisionReason::SpecialMiniRoadland;
    if (reason == "special_patrol") return DecisionReason::SpecialPatrol;
    return DecisionReason::Unknown;
}

inline DecisionLayer DecisionLayerForReason(const DecisionReason reason) noexcept {
    switch (reason) {
        case DecisionReason::DefaultAreaPolicy:
            return DecisionLayer::Default;
        case DecisionReason::OwnBaseMulti:
        case DecisionReason::OwnBaseChase:
        case DecisionReason::OwnBaseGuard:
        case DecisionReason::OwnHighlandRoadland:
        case DecisionReason::OwnRoadland:
        case DecisionReason::OwnHighland:
        case DecisionReason::CommonCentral:
        case DecisionReason::EnemyRoadlandSoft:
        case DecisionReason::EnemyHighlandSoft:
        case DecisionReason::RegionalDefense:
        case DecisionReason::OwnFortressGainPointEnemy:
            return DecisionLayer::RegionalDefense;
        case DecisionReason::AimModeBuff:
        case DecisionReason::AimModeOutpost:
            return DecisionLayer::AimMode;
        case DecisionReason::NaviProgressWatchdog:
            return DecisionLayer::Watchdog;
        case DecisionReason::RegionalIdlePatrol:
            return DecisionLayer::IdlePatrol;
        case DecisionReason::AreaTask:
        case DecisionReason::NaviTransition:
            return DecisionLayer::Task;
        case DecisionReason::AreaScopeBlocked:
        case DecisionReason::ProtectHero:
            return DecisionLayer::Tactical;
        case DecisionReason::SpecialMiniRoadland:
        case DecisionReason::SpecialPatrol:
            return DecisionLayer::Special;
        default:
            return DecisionLayer::Unknown;
    }
}

inline int DecisionPriorityForReason(const DecisionReason reason) noexcept {
    switch (DecisionLayerForReason(reason)) {
        case DecisionLayer::Hard: return 400;
        case DecisionLayer::RegionalDefense: return 300;
        case DecisionLayer::AimMode: return 260;
        case DecisionLayer::Watchdog: return 240;
        case DecisionLayer::Task: return 200;
        case DecisionLayer::Special: return 140;
        case DecisionLayer::Default: return 120;
        case DecisionLayer::Tactical: return 100;
        case DecisionLayer::IdlePatrol: return 40;
        default: return 0;
    }
}

}  // namespace BehaviorTree
