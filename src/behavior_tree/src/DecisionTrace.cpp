// AUTO-COMMENT: file overview
// Optional JSONL decision trace writer for offline visualization tools.

#include "../include/Application.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>

using namespace LangYa;
using namespace BehaviorTree;

namespace BehaviorTree {

namespace {

constexpr int kTraceFieldWidthCm = 2800;
constexpr int kTraceFieldHeightCm = 1500;

const char* UnitTeamToString(const UnitTeam team) noexcept {
    switch (team) {
        case UnitTeam::Red: return "red";
        case UnitTeam::Blue: return "blue";
        case UnitTeam::Other: return "other";
        default: return "unknown";
    }
}

const char* AimModeToString(const AimMode mode) noexcept {
    switch (mode) {
        case AimMode::None: return "None";
        case AimMode::AutoAim: return "AutoAim";
        case AimMode::RotateScan: return "RotateScan";
        case AimMode::Buff: return "Buff";
        case AimMode::Outpost: return "Outpost";
        default: return "Unknown";
    }
}

const char* ArmorTypeToString(const ArmorType type) noexcept {
    switch (type) {
        case ArmorType::Base: return "Base";
        case ArmorType::Hero: return "Hero";
        case ArmorType::Engineer: return "Engineer";
        case ArmorType::Infantry1: return "Infantry1";
        case ArmorType::Infantry2: return "Infantry2";
        case ArmorType::Infantry3: return "Infantry3";
        case ArmorType::Sentry: return "Sentry";
        case ArmorType::Outpost: return "Outpost";
        default: return "Unknown";
    }
}

const char* UnitTypeToString(const UnitType type) noexcept {
    switch (type) {
        case UnitType::Hero: return "Hero";
        case UnitType::Engineer: return "Engineer";
        case UnitType::Infantry1: return "Infantry1";
        case UnitType::Infantry2: return "Infantry2";
        case UnitType::Infantry3: return "Infantry3";
        case UnitType::Drone: return "Drone";
        case UnitType::Sentry: return "Sentry";
        case UnitType::Dart: return "Dart";
        case UnitType::Radar: return "Radar";
        default: return "Unknown";
    }
}

const char* GoalName(const int base_goal_id) noexcept {
    switch (base_goal_id) {
        case LangYa::Home.ID: return "Home";
        case LangYa::Base.ID: return "Base";
        case LangYa::Recovery.ID: return "Recovery";
        case LangYa::BuffShoot.ID: return "BuffShoot";
        case LangYa::LeftHighLand.ID: return "LeftHighLand";
        case LangYa::CastleLeft1.ID: return "CastleLeft1";
        case LangYa::Castle.ID: return "Castle";
        case LangYa::CastleRight1.ID: return "CastleRight1";
        case LangYa::CastleRight2.ID: return "CastleRight2";
        case LangYa::FlyRoad.ID: return "FlyRoad";
        case LangYa::OutpostArea.ID: return "OutpostArea";
        case LangYa::MidShoot.ID: return "MidShoot";
        case LangYa::LeftShoot.ID: return "LeftShoot";
        case LangYa::OutpostShoot.ID: return "OutpostShoot";
        case LangYa::BuffAround1.ID: return "BuffAround1";
        case LangYa::BuffAround2.ID: return "BuffAround2";
        case LangYa::RightShoot.ID: return "RightShoot";
        case LangYa::HoleRoad.ID: return "HoleRoad";
        case LangYa::OccupyArea.ID: return "OccupyArea";
        case LangYa::Highland.ID: return "Highland";
        case LangYa::CastleLeft2.ID: return "CastleLeft2";
        case LangYa::BaseToCentral.ID: return "BaseToCentral";
        case LangYa::CentralToBase.ID: return "CentralToBase";
        case LangYa::BuffOutpost.ID: return "BuffOutpost";
        case LangYa::OutpostGuard.ID: return "OutpostGuard";
        default: return "Unknown";
    }
}

int GoalBaseId(const std::uint8_t goal_id) noexcept {
    return goal_id >= LangYa::TeamedLocation::LocationCount
        ? static_cast<int>(goal_id - LangYa::TeamedLocation::LocationCount)
        : static_cast<int>(goal_id);
}

const char* GoalSide(const std::uint8_t goal_id) noexcept {
    return goal_id >= LangYa::TeamedLocation::LocationCount ? "blue" : "red";
}

json FiniteFloat(const float value) {
    if (std::isfinite(value)) {
        return value;
    }
    return nullptr;
}

json UnitListToJson(const std::vector<UnitType>& units) {
    json out = json::array();
    for (const auto unit : units) {
        out.push_back({
            {"id", static_cast<int>(unit)},
            {"name", UnitTypeToString(unit)},
        });
    }
    return out;
}

json RobotToJson(const Robot& robot, const UnitType type, const char* side) {
    return {
        {"type_id", static_cast<int>(type)},
        {"type", UnitTypeToString(type)},
        {"side", side},
        {"team_id", static_cast<int>(robot.team_)},
        {"hp", static_cast<int>(robot.currentHealth_)},
        {"max_hp", static_cast<int>(robot.maxHealth_)},
        {"distance_m", FiniteFloat(robot.distance_)},
        {"position_cm", {
            {"x", static_cast<int>(robot.position_.X)},
            {"y", static_cast<int>(robot.position_.Y)},
        }},
    };
}

json RobotsToJson(const Robots& robots, const char* side) {
    static constexpr std::array<UnitType, 6> kTraceUnits{
        UnitType::Hero,
        UnitType::Engineer,
        UnitType::Infantry1,
        UnitType::Infantry2,
        UnitType::Infantry3,
        UnitType::Sentry,
    };

    json out = json::array();
    for (const auto type : kTraceUnits) {
        out.push_back(RobotToJson(robots[type], type, side));
    }
    return out;
}

json PostureValueJson(const std::uint8_t value) {
    const auto posture = ToPosture(value);
    return {
        {"id", static_cast<int>(value)},
        {"name", PostureToString(posture)},
    };
}

json PostureEnumJson(const SentryPosture posture) {
    return {
        {"id", static_cast<int>(ToPostureValue(posture))},
        {"name", PostureToString(posture)},
    };
}

}  // namespace

bool Application::InitDecisionTrace() {
    if (!decisionTraceRequested_) {
        return false;
    }
    if (decisionTraceFile_.empty()) {
        if (LoggerPtr) {
            LoggerPtr->Warning("Decision trace requested but decision_trace_file is empty; trace stays disabled.");
        }
        return false;
    }

    try {
        std::filesystem::path trace_path(decisionTraceFile_);
        if (!trace_path.is_absolute()) {
            trace_path = std::filesystem::current_path() / trace_path;
        }
        trace_path = trace_path.lexically_normal();

        const auto parent = trace_path.parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }

        decisionTraceStream_.open(trace_path, std::ios::out | std::ios::trunc);
        if (!decisionTraceStream_.is_open()) {
            decisionTraceEnabled_ = false;
            if (LoggerPtr) {
                LoggerPtr->Warning("Decision trace disabled: cannot open {}", trace_path.string());
            }
            return false;
        }

        decisionTraceFile_ = trace_path.string();
        decisionTraceEnabled_ = true;
        decisionTraceTickCount_ = 0;
        decisionTraceWriteCount_ = 0;
        if (LoggerPtr) {
            LoggerPtr->Info("Decision trace enabled: {}", decisionTraceFile_);
        }
        return true;
    } catch (const std::exception& ex) {
        decisionTraceEnabled_ = false;
        if (LoggerPtr) {
            LoggerPtr->Warning("Decision trace disabled: {}", ex.what());
        }
        return false;
    }
}

void Application::CloseDecisionTrace() {
    if (decisionTraceStream_.is_open()) {
        decisionTraceStream_.flush();
        decisionTraceStream_.close();
    }
    decisionTraceEnabled_ = false;
}

void Application::WriteDecisionTrace(const std::string_view event) {
    if (!decisionTraceEnabled_ || !decisionTraceStream_.is_open()) {
        return;
    }

    const bool is_tick_event = event == "tick";
    std::uint64_t trace_tick = decisionTraceTickCount_;
    if (is_tick_event) {
        trace_tick = decisionTraceTickCount_++;
        if (decisionTraceEveryTicks_ > 1 &&
            (trace_tick % static_cast<std::uint64_t>(decisionTraceEveryTicks_)) != 0) {
            return;
        }
    }

    const auto now = std::chrono::steady_clock::now();
    const auto wall_now = std::chrono::system_clock::now();
    const double elapsed_sec = std::chrono::duration<double>(now - gameStartTime).count();
    const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        wall_now.time_since_epoch()).count();

    const int goal_base_id = GoalBaseId(naviCommandGoal);
    const auto posture_runtime = postureManager_.Runtime();
    const bool enable_chase_to_navi =
        config.ChaseSettings.Enable && config.ChaseSettings.ToNavi;
    const bool uses_chase_to_navi =
        config.NaviSettings.UseXY && config.NaviSettings.ToNavi && enable_chase_to_navi;
    const bool uses_goal_pos = config.NaviSettings.UseXY && !uses_chase_to_navi;
    const bool uses_goal_pos_bridge = uses_goal_pos && config.NaviSettings.ToNavi;
    const bool uses_any_to_navi = uses_chase_to_navi || uses_goal_pos_bridge;
    const char* output_kind = "goal_id";
    const char* output_topic = ly_navi_goal::Name;
    const char* final_goal_pos_topic = "";
    if (uses_chase_to_navi) {
        output_kind = "relative_target_bridge";
        output_topic = ly_navi_target_rel::Name;
        final_goal_pos_topic = "/goal_pose";
    } else if (uses_goal_pos) {
        output_kind = uses_goal_pos_bridge ? "goal_pos_raw_bridge" : "goal_pos";
        output_topic = uses_goal_pos_bridge ? ly_navi_goal_pos_raw::Name : ly_navi_goal_pos::Name;
        final_goal_pos_topic = uses_goal_pos_bridge ? "/goal_pose" : ly_navi_goal_pos::Name;
    }

    json record;
    record["schema"] = "ly_decision_trace_v1";
    record["schema_version"] = 2;
    record["event"] = std::string(event);
    record["tick"] = trace_tick;
    record["t"] = elapsed_sec;
    record["elapsed_sec"] = ElapsedSeconds();
    record["wall_time_ms"] = wall_ms;

    record["field_cm"] = {
        {"width", kTraceFieldWidthCm},
        {"height", kTraceFieldHeightCm},
        {"frame", "left_bottom_origin_cm"},
    };
    record["competition_profile"] = CompetitionProfileToString(competitionProfile_);
    record["strategy_mode"] = StrategyModeToString(strategyMode_);
    record["strategy_mode_id"] = static_cast<int>(strategyMode_);
    record["team"] = UnitTeamToString(team);
    record["team_id"] = static_cast<int>(team);

    record["aim_mode"] = AimModeToString(aimMode);
    record["aim_mode_id"] = static_cast<int>(aimMode);
    record["aim_enable"] = {
        {"auto_aim", aimMode == AimMode::AutoAim || aimMode == AimMode::RotateScan},
        {"buff", aimMode == AimMode::Buff},
        {"outpost", aimMode == AimMode::Outpost},
    };
    record["target_armor"] = {
        {"id", static_cast<int>(targetArmor.Type)},
        {"name", ArmorTypeToString(targetArmor.Type)},
        {"distance_m", FiniteFloat(targetArmor.Distance)},
    };
    record["target_state"] = {
        {"has_recent_target", HasRecentTarget()},
        {"fresh_auto_aim", autoAimData.Fresh && autoAimData.Valid},
        {"fresh_buff", buffAimData.Fresh && buffAimData.Valid && buffAimData.BuffFollow},
        {"fresh_outpost", outpostAimData.Fresh && outpostAimData.Valid},
        {"hitable_targets", UnitListToJson(hitableTargets)},
        {"reliable_enemy_positions", UnitListToJson(reliableEnemyPosuition)},
    };
    record["units"] = {
        {"friend", RobotsToJson(friendRobots, "friend")},
        {"enemy", RobotsToJson(enemyRobots, "enemy")},
    };

    record["decision_output"] = {
        {"kind", output_kind},
        {"source", "behavior_tree"},
        {"publish_allowed", naviGoalPublishAllowed_},
        {"publish_enabled", publishNaviGoal_},
        {"uses_goal_pos", uses_goal_pos},
        {"uses_to_navi", uses_any_to_navi},
        {"relative_target_valid", naviRelativeTargetValid},
        {"output_topic", output_topic},
        {"output_frame", "map"},
        {"final_goal_pos_topic", final_goal_pos_topic},
        {"goal_id", static_cast<int>(naviCommandGoal)},
        {"goal_base_id", goal_base_id},
        {"goal_name", GoalName(goal_base_id)},
        {"goal_side", GoalSide(naviCommandGoal)},
        {"speed_level", static_cast<int>(speedLevel)},
        {"goal_pos_cm", {
            {"x", static_cast<int>(naviGoalPosition.x)},
            {"y", static_cast<int>(naviGoalPosition.y)},
        }},
    };

    record["navi_goal"] = {
        {"id", static_cast<int>(naviCommandGoal)},
        {"base_id", goal_base_id},
        {"name", GoalName(goal_base_id)},
        {"side", GoalSide(naviCommandGoal)},
        {"publish_allowed", naviGoalPublishAllowed_},
        {"publish_enabled", publishNaviGoal_},
        {"speed_level", static_cast<int>(speedLevel)},
        {"position_cm", {
            {"x", static_cast<int>(naviGoalPosition.x)},
            {"y", static_cast<int>(naviGoalPosition.y)},
        }},
    };
    record["navi_velocity"] = {
        {"input_x", static_cast<int>(naviVelocityInput.X)},
        {"input_y", static_cast<int>(naviVelocityInput.Y)},
        {"output_x", static_cast<int>(naviVelocity.X)},
        {"output_y", static_cast<int>(naviVelocity.Y)},
    };
    record["navi_relative_target"] = {
        {"valid", naviRelativeTargetValid},
        {"x", FiniteFloat(naviRelativeTargetX)},
        {"y", FiniteFloat(naviRelativeTargetY)},
        {"z", FiniteFloat(naviRelativeTargetZ)},
        {"distance", FiniteFloat(naviRelativeTargetDistance)},
        {"yaw_error_deg", FiniteFloat(naviRelativeTargetYawErrorDeg)},
        {"pitch_error_deg", FiniteFloat(naviRelativeTargetPitchErrorDeg)},
        {"armor_type", static_cast<int>(naviRelativeTargetArmorType)},
        {"aim_mode", static_cast<int>(naviRelativeTargetAimMode)},
    };

    record["posture"] = {
        {"command", PostureValueJson(postureCommand)},
        {"state", PostureValueJson(postureState)},
        {"last_desired", PostureEnumJson(postureLastDesired_)},
        {"last_reason", postureLastReason_},
        {"under_fire_recent", IsUnderFireRecent()},
        {"under_fire_burst", IsUnderFireBurst()},
        {"runtime", {
            {"current", PostureEnumJson(posture_runtime.Current)},
            {"desired", PostureEnumJson(posture_runtime.Desired)},
            {"pending", PostureEnumJson(posture_runtime.Pending)},
            {"has_pending", posture_runtime.HasPending},
            {"feedback_stale", posture_runtime.FeedbackStale},
            {"retry_count", posture_runtime.RetryCount},
            {"accum_sec", {
                {"attack", posture_runtime.AccumSec[static_cast<int>(SentryPosture::Attack)]},
                {"defense", posture_runtime.AccumSec[static_cast<int>(SentryPosture::Defense)]},
                {"move", posture_runtime.AccumSec[static_cast<int>(SentryPosture::Move)]},
            }},
            {"degraded", {
                {"attack", posture_runtime.Degraded[static_cast<int>(SentryPosture::Attack)]},
                {"defense", posture_runtime.Degraded[static_cast<int>(SentryPosture::Defense)]},
                {"move", posture_runtime.Degraded[static_cast<int>(SentryPosture::Move)]},
            }},
        }},
    };

    record["referee"] = {
        {"self_hp", static_cast<int>(myselfHealth)},
        {"self_outpost_hp", static_cast<int>(selfOutpostHealth)},
        {"enemy_outpost_hp", static_cast<int>(enemyOutpostHealth)},
        {"self_base_hp", static_cast<int>(selfBaseHealth)},
        {"enemy_base_hp", static_cast<int>(enemyBaseHealth)},
        {"ammo", static_cast<int>(ammoLeft)},
        {"time_left", static_cast<int>(timeLeft)},
        {"is_game_begin", is_game_begin},
        {"has_self_hp", hasReceivedMyselfHealth_},
        {"has_ammo", hasReceivedAmmoLeft_},
        {"has_game_start", hasReceivedGameStartFlag_},
        {"rfid_status", rfidStatus},
        {"has_rfid_status_2", hasRfidStatus2},
        {"rfid_status_2", static_cast<int>(rfidStatus2)},
        {"ext_event_data", extEventData},
        {"has_event_data", hasReceivedEventData_},
        {"event_self_small_energy_status", static_cast<int>(eventSelfSmallEnergyStatus_)},
        {"event_self_large_energy_status", static_cast<int>(eventSelfLargeEnergyStatus_)},
        {"team_buff", {
            {"recovery", static_cast<int>(teamBuff.RecoveryBuff)},
            {"cooling", static_cast<int>(teamBuff.CoolingBuff)},
            {"defence", static_cast<int>(teamBuff.DefenceBuff)},
            {"vulnerability", static_cast<int>(teamBuff.VulnerabilityBuff)},
            {"attack", static_cast<int>(teamBuff.AttackBuff)},
            {"remaining_energy", static_cast<int>(teamBuff.RemainingEnergy)},
        }},
    };

    record["gimbal"] = {
        {"yaw_deg", FiniteFloat(gimbalAngles.Yaw)},
        {"pitch_deg", FiniteFloat(gimbalAngles.Pitch)},
        {"yaw_vel_raw", static_cast<int>(gimbalYawVelRaw)},
        {"yaw_vel_deg_per_sec", FiniteFloat(gimbalYawVelDegPerSec)},
        {"yaw_angle_raw", static_cast<int>(gimbalYawAngleRaw)},
        {"yaw_angle_deg", FiniteFloat(gimbalYawAngleDeg)},
        {"cap_v", static_cast<int>(capV)},
        {"navi_lower_head", static_cast<int>(naviLowerHead)},
        {"fire_code", {
            {"fire_status", static_cast<int>(RecFireCode.FireStatus)},
            {"cap_state", static_cast<int>(RecFireCode.CapState)},
            {"follow_mode", static_cast<int>(RecFireCode.FollowMode)},
            {"aim_mode", static_cast<int>(RecFireCode.AimMode)},
            {"rotate", static_cast<int>(RecFireCode.Rotate)},
        }},
    };

    record["runtime_guard"] = {
        {"fault", RuntimeFaultCodeToString(runtimeFaultCode_.load(std::memory_order_relaxed))},
        {"recovery_requested", runtimeRecoveryRequested_.load(std::memory_order_relaxed)},
        {"recovering", runtimeRecovering_},
    };

    decisionTraceStream_ << record.dump() << '\n';
    ++decisionTraceWriteCount_;
    if (!is_tick_event || (decisionTraceWriteCount_ % 20) == 0) {
        decisionTraceStream_.flush();
    }
}

}  // namespace BehaviorTree
