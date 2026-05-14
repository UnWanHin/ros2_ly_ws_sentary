// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <initializer_list>
#include <utility>

namespace {

std::string NormalizeProfile(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::string NormalizeAutonomyToken(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        if (c == '-' || c == ' ') {
            return '_';
        }
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string NormalizePointToken(std::string value) {
    value = NormalizeAutonomyToken(std::move(value));
    value.erase(
        std::remove_if(
            value.begin(),
            value.end(),
            [](const char c) {
                return c == '_' || c == '.' || c == '/';
            }),
        value.end());
    return value;
}

std::uint8_t ClampRotateGear(const int rotate) noexcept {
    return static_cast<std::uint8_t>(std::clamp(rotate, 0, 3));
}

std::string NormalizeMainAreaToken(std::string value) {
    value = NormalizeAutonomyToken(std::move(value));
    if (value == "base") {
        return "base";
    }
    if (value == "highland" || value == "high_land" || value == "high") {
        return "highland";
    }
    if (value == "roadland" || value == "road_land" || value == "road") {
        return "roadland";
    }
    if (value == "central" || value == "center" || value == "centre" || value == "middle") {
        return "central";
    }
    return {};
}

bool AppendRegionalPatrolGoalByName(
    const std::string& goal_name,
    std::vector<std::uint8_t>& goals) {
    const auto token = NormalizeAutonomyToken(goal_name);
    std::uint8_t goal_id = LangYa::Home.ID;
    if (token == "highland" || token == "high_land") {
        goal_id = LangYa::Highland.ID;
    } else if (token == "buffshoot" || token == "buff_shoot") {
        goal_id = LangYa::BuffShoot.ID;
    } else if (token == "holeroad" || token == "hole_road") {
        goal_id = LangYa::HoleRoad.ID;
    } else if (token == "castleleft1" || token == "castle_left1" || token == "castle_left_1" ||
               token == "castleleft" || token == "castle_left") {
        goal_id = LangYa::CastleLeft1.ID;
    } else if (token == "castleleft2" || token == "castle_left2" || token == "castle_left_2") {
        goal_id = LangYa::CastleLeft2.ID;
    } else if (token == "castleright2" || token == "castle_right2" || token == "castle_right_2") {
        goal_id = LangYa::CastleRight2.ID;
    } else if (token == "castleright1" || token == "castle_right1" || token == "castle_right_1") {
        goal_id = LangYa::CastleRight1.ID;
    } else if (token == "castle") {
        goal_id = LangYa::Castle.ID;
    } else if (token == "buffoutpost" || token == "buff_outpost") {
        goal_id = LangYa::BuffOutpost.ID;
    } else if (token == "outpostguard" || token == "outpost_guard") {
        goal_id = LangYa::OutpostGuard.ID;
    } else {
        return false;
    }
    goals.push_back(goal_id);
    return true;
}

const std::vector<std::pair<const char*, std::uint8_t>>& MyBasePatrolGoalNameMap() {
    static const std::vector<std::pair<const char*, std::uint8_t>> goals{
        {"CastleLeft1", LangYa::CastleLeft1.ID},
        {"CastleLeft2", LangYa::CastleLeft2.ID},
        {"CastleRight2", LangYa::CastleRight2.ID},
        {"CastleRight1", LangYa::CastleRight1.ID},
        {"HoleRoad", LangYa::HoleRoad.ID},
        {"OutpostGuard", LangYa::OutpostGuard.ID},
        {"BuffOutpost", LangYa::BuffOutpost.ID}
    };
    return goals;
}

const std::vector<std::pair<const char*, std::uint8_t>>& PointManagerGoalNameMap() {
    static const std::vector<std::pair<const char*, std::uint8_t>> goals{
        {"Home", LangYa::Home.ID},
        {"Base", LangYa::Base.ID},
        {"Recovery", LangYa::Recovery.ID},
        {"BuffShoot", LangYa::BuffShoot.ID},
        {"LeftHighLand", LangYa::LeftHighLand.ID},
        {"CastleLeft1", LangYa::CastleLeft1.ID},
        {"CastleLeft2", LangYa::CastleLeft2.ID},
        {"Castle", LangYa::Castle.ID},
        {"CastleRight1", LangYa::CastleRight1.ID},
        {"CastleRight2", LangYa::CastleRight2.ID},
        {"FlyRoad", LangYa::FlyRoad.ID},
        {"OutpostArea", LangYa::OutpostArea.ID},
        {"MidShoot", LangYa::MidShoot.ID},
        {"LeftShoot", LangYa::LeftShoot.ID},
        {"OutpostShoot", LangYa::OutpostShoot.ID},
        {"BuffAround1", LangYa::BuffAround1.ID},
        {"BuffAround2", LangYa::BuffAround2.ID},
        {"RightShoot", LangYa::RightShoot.ID},
        {"HoleRoad", LangYa::HoleRoad.ID},
        {"OccupyArea", LangYa::OccupyArea.ID},
        {"Highland", LangYa::Highland.ID},
        {"BaseToCentral", LangYa::BaseToCentral.ID},
        {"CentralToBase", LangYa::CentralToBase.ID},
        {"BuffOutpost", LangYa::BuffOutpost.ID},
        {"OutpostGuard", LangYa::OutpostGuard.ID},
        {"MiniRoadland", LangYa::MiniRoadland.ID},
        {"CentralLeftA", LangYa::CentralLeftA.ID},
        {"CentralLeftB", LangYa::CentralLeftB.ID}
    };
    return goals;
}

bool MyBasePatrolGoalIdByName(const std::string& goal_name, std::uint8_t& goal_id) {
    const auto token = NormalizeAutonomyToken(goal_name);
    for (const auto& [name, id] : MyBasePatrolGoalNameMap()) {
        if (NormalizeAutonomyToken(name) == token) {
            goal_id = id;
            return true;
        }
    }
    return false;
}

bool PointManagerGoalIdByName(const std::string& goal_name, std::uint8_t& goal_id) {
    const auto token = NormalizePointToken(goal_name);
    for (const auto& [name, id] : PointManagerGoalNameMap()) {
        if (NormalizePointToken(name) == token) {
            goal_id = id;
            return true;
        }
    }
    return false;
}

void UpsertMyBasePatrolGoalWeight(
    std::vector<LangYa::MyBasePatrolGoalSetting>& goals,
    const std::uint8_t goal_id,
    const double weight) {
    const auto it = std::find_if(
        goals.begin(),
        goals.end(),
        [goal_id](const LangYa::MyBasePatrolGoalSetting& goal) {
            return goal.BaseGoalId == goal_id;
        });
    if (it != goals.end()) {
        it->Weight = weight;
        return;
    }
    goals.push_back(LangYa::MyBasePatrolGoalSetting{
        .BaseGoalId = goal_id,
        .Weight = weight
    });
}

BehaviorTree::CompetitionProfile ParseCompetitionProfile(const std::string& value) {
    const auto normalized = NormalizeProfile(value);
    if (normalized == "league") {
        return BehaviorTree::CompetitionProfile::League;
    }
    return BehaviorTree::CompetitionProfile::Regional;
}

bool IsValidBaseGoal(const std::uint8_t goal_id) {
    return BehaviorTree::AreaManager::IsValidBaseGoalId(goal_id);
}

bool ReadOptionalBoolParam(
    const std::shared_ptr<rclcpp::Node>& node,
    std::initializer_list<const char*> names,
    bool& value) {
    if (!node) {
        return false;
    }
    for (const auto* name : names) {
        if (!node->has_parameter(name)) {
            continue;
        }
        rclcpp::Parameter param;
        if (!node->get_parameter(name, param)) {
            continue;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            value = param.as_bool();
            return true;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            value = param.as_int() != 0;
            return true;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            auto normalized = NormalizeProfile(param.as_string());
            if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
                value = true;
                return true;
            }
            if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
                value = false;
                return true;
            }
        }
    }
    return false;
}

bool ReadOptionalIntParam(
    const std::shared_ptr<rclcpp::Node>& node,
    std::initializer_list<const char*> names,
    int& value) {
    if (!node) {
        return false;
    }
    for (const auto* name : names) {
        if (!node->has_parameter(name)) {
            continue;
        }
        rclcpp::Parameter param;
        if (!node->get_parameter(name, param)) {
            continue;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            value = static_cast<int>(param.as_int());
            return true;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            value = static_cast<int>(param.as_double());
            return true;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            try {
                value = std::stoi(param.as_string());
                return true;
            } catch (...) {
                continue;
            }
        }
    }
    return false;
}

bool ReadOptionalIntParam(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<std::string>& names,
    int& value) {
    if (!node) {
        return false;
    }
    for (const auto& name : names) {
        if (!node->has_parameter(name)) {
            continue;
        }
        rclcpp::Parameter param;
        if (!node->get_parameter(name, param)) {
            continue;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            value = static_cast<int>(param.as_int());
            return true;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            value = static_cast<int>(param.as_double());
            return true;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            try {
                value = std::stoi(param.as_string());
                return true;
            } catch (...) {
                continue;
            }
        }
    }
    return false;
}

bool ReadOptionalDoubleParam(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<std::string>& names,
    double& value) {
    if (!node) {
        return false;
    }
    for (const auto& name : names) {
        if (!node->has_parameter(name)) {
            continue;
        }
        rclcpp::Parameter param;
        if (!node->get_parameter(name, param)) {
            continue;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            value = param.as_double();
            return true;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            value = static_cast<double>(param.as_int());
            return true;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            try {
                value = std::stod(param.as_string());
                return true;
            } catch (...) {
                continue;
            }
        }
    }
    return false;
}

bool ReadOptionalStringParam(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<std::string>& names,
    std::string& value) {
    if (!node) {
        return false;
    }
    for (const auto& name : names) {
        if (!node->has_parameter(name)) {
            continue;
        }
        rclcpp::Parameter param;
        if (!node->get_parameter(name, param)) {
            continue;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            value = param.as_string();
            return true;
        }
    }
    return false;
}

bool ReadOptionalBoolParam(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<std::string>& names,
    bool& value) {
    if (!node) {
        return false;
    }
    for (const auto& name : names) {
        if (!node->has_parameter(name)) {
            continue;
        }
        rclcpp::Parameter param;
        if (!node->get_parameter(name, param)) {
            continue;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            value = param.as_bool();
            return true;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            value = param.as_int() != 0;
            return true;
        }
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            auto normalized = NormalizeProfile(param.as_string());
            if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
                value = true;
                return true;
            }
            if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
                value = false;
                return true;
            }
        }
    }
    return false;
}

std::string ResolveBehaviorTreeConfigPath(const std::string& configured_path) {
    if (configured_path.empty()) {
        return {};
    }

    const std::filesystem::path path(configured_path);
    if (path.is_absolute()) {
        return path.lexically_normal().string();
    }

    try {
        const auto pkg_path = ament_index_cpp::get_package_share_directory("behavior_tree");
        return (std::filesystem::path(pkg_path) / path).lexically_normal().string();
    } catch (...) {
        return path.lexically_normal().string();
    }
}

bool LoadNaviDebugPlanFile(LangYa::NaviDebugSetting& nd, const std::shared_ptr<Logger>& logger) {
    const std::string default_plan_file = "Scripts/ConfigJson/regional/debug/navi_debug_points.json";
    nd.PlanFile = ResolveBehaviorTreeConfigPath(nd.PlanFile.empty() ? default_plan_file : nd.PlanFile);

    std::ifstream ifs(nd.PlanFile);
    if (!ifs.is_open()) {
        if (logger) {
            logger->Warning("Failed to open NaviDebug plan file: {}", nd.PlanFile);
        }
        return false;
    }

    nlohmann::json root;
    ifs >> root;

    const nlohmann::json* selected_plan = nullptr;
    std::string selected_plan_name = nd.ActivePlan.empty()
        ? root.value("ActivePlan", std::string{})
        : nd.ActivePlan;

    const auto plans_it = root.find("Plans");
    if (plans_it != root.end() && plans_it->is_object()) {
        if (selected_plan_name.empty()) {
            if (plans_it->size() == 1U) {
                selected_plan_name = plans_it->begin().key();
            } else if (plans_it->contains("default")) {
                selected_plan_name = "default";
            }
        }
        if (!selected_plan_name.empty()) {
            const auto selected_it = plans_it->find(selected_plan_name);
            if (selected_it != plans_it->end() && selected_it->is_object()) {
                selected_plan = &(*selected_it);
            }
        }
        if (selected_plan == nullptr && !plans_it->empty()) {
            selected_plan_name = plans_it->begin().key();
            selected_plan = &plans_it->begin().value();
            if (logger) {
                logger->Warning("NaviDebug plan '{}' not found, fallback to first plan '{}'.",
                    nd.ActivePlan, selected_plan_name);
            }
        }
    } else {
        selected_plan = &root;
        if (selected_plan_name.empty()) {
            selected_plan_name = "<root>";
        }
    }

    if (selected_plan == nullptr || !selected_plan->is_object()) {
        if (logger) {
            logger->Warning("NaviDebug plan file '{}' has no valid plan object.", nd.PlanFile);
        }
        return false;
    }

    nd.ActivePlan = selected_plan_name;
    nd.GoalHoldSec = selected_plan->value("GoalHoldSec", nd.GoalHoldSec);
    nd.DisableTeamOffset = selected_plan->value("DisableTeamOffset", nd.DisableTeamOffset);
    nd.IgnoreRecovery = selected_plan->value("IgnoreRecovery", nd.IgnoreRecovery);
    nd.SpeedLevel = static_cast<std::uint8_t>(
        std::clamp(selected_plan->value("SpeedLevel", static_cast<int>(nd.SpeedLevel)), 0, 2));

    const std::string plan_mode = NormalizeProfile(selected_plan->value(
        "Mode", nd.Random ? std::string("random") : std::string("sequence")));
    if (plan_mode == "random") {
        nd.Random = true;
    } else if (plan_mode == "sequence" || plan_mode == "sequential" || plan_mode == "ordered") {
        nd.Random = false;
    } else {
        nd.Random = selected_plan->value("Random", nd.Random);
    }

    if (selected_plan->contains("Goals")) {
        selected_plan->at("Goals").get_to(nd.Goals);
    } else {
        nd.Goals.clear();
    }

    if (logger) {
        logger->Info(
            "Loaded NaviDebug plan file='{}' active_plan='{}' mode={} goals={} hold_sec={} speed={} disable_team_offset={} ignore_recovery={}",
            nd.PlanFile,
            nd.ActivePlan,
            nd.Random ? "random" : "sequence",
            nd.Goals.size(),
            nd.GoalHoldSec,
            static_cast<int>(nd.SpeedLevel),
            nd.DisableTeamOffset ? 1 : 0,
            nd.IgnoreRecovery ? 1 : 0);
    }
    return true;
}

std::vector<std::string> ParseAreaScopeList(const nlohmann::json& value) {
    std::vector<std::string> areas;
    if (value.is_array()) {
        value.get_to(areas);
        return areas;
    }
    if (!value.is_object()) {
        return areas;
    }

    areas.reserve(value.size());
    for (const auto& [area_name, enabled_value] : value.items()) {
        bool enabled = false;
        if (enabled_value.is_boolean()) {
            enabled = enabled_value.get<bool>();
        } else if (enabled_value.is_number_integer()) {
            enabled = enabled_value.get<int>() != 0;
        }
        if (enabled) {
            areas.push_back(area_name);
        }
    }
    return areas;
}

}  // namespace


namespace LangYa {

    // 自定义 from_json 函数，用于自动转换 JSON 到结构体
    void from_json(const json& j, AimDebug& ad) {
        ad.StopFire = j.value("StopFire", ad.StopFire);
        ad.StopRotate = j.value("StopRotate", ad.StopRotate);
        ad.StopScan = j.value("StopScan", ad.StopScan);
        ad.ForceOutpost = j.value("ForceOutpost", ad.ForceOutpost);
        ad.ForceBuff = j.value("ForceBuff", ad.ForceBuff);
        ad.HitCar = j.value("HitCar", ad.HitCar);
        ad.FireRequireTargetStatus = j.value("FireRequireTargetStatus", ad.FireRequireTargetStatus);
        ad.ReuseLatchedAnglesOnNoTarget = j.value("ReuseLatchedAnglesOnNoTarget", ad.ReuseLatchedAnglesOnNoTarget);
        ad.LatchedTargetHoldMs = j.value("LatchedTargetHoldMs", ad.LatchedTargetHoldMs);
    }

    void from_json(const json& j, PatrolScanSetting& ps) {
        ps.Mode = j.value("Mode", ps.Mode);
        if (j.contains("Mode1") && j.at("Mode1").is_object()) {
            const auto& mode = j.at("Mode1");
            ps.Mode1YawStepDegPerTick =
                mode.value("YawStepDegPerTick", ps.Mode1YawStepDegPerTick);
            ps.Mode1YawBoostStepDegPerTick =
                mode.value("YawBoostStepDegPerTick", ps.Mode1YawBoostStepDegPerTick);
            ps.Mode1PitchCenterDeg =
                mode.value("PitchCenterDeg", ps.Mode1PitchCenterDeg);
            ps.Mode1PitchHalfRangeDeg =
                mode.value("PitchHalfRangeDeg", ps.Mode1PitchHalfRangeDeg);
            ps.Mode1PitchPeriodMs =
                mode.value("PitchPeriodMs", ps.Mode1PitchPeriodMs);
        }
        if (j.contains("Mode2") && j.at("Mode2").is_object()) {
            const auto& mode = j.at("Mode2");
            ps.Mode2YawStepDegPerTick =
                mode.value("YawStepDegPerTick", ps.Mode2YawStepDegPerTick);
            ps.Mode2YawBoostStepDegPerTick =
                mode.value("YawBoostStepDegPerTick", ps.Mode2YawBoostStepDegPerTick);
            ps.Mode2YawHalfRangeDeg =
                mode.value("YawHalfRangeDeg", ps.Mode2YawHalfRangeDeg);
            ps.Mode2CenterDriftPerCycleDeg =
                mode.value("CenterDriftPerCycleDeg", ps.Mode2CenterDriftPerCycleDeg);
            ps.Mode2PitchCenterDeg =
                mode.value("PitchCenterDeg", ps.Mode2PitchCenterDeg);
            ps.Mode2PitchHalfRangeDeg =
                mode.value("PitchHalfRangeDeg", ps.Mode2PitchHalfRangeDeg);
            ps.Mode2PitchPeriodMs =
                mode.value("PitchPeriodMs", ps.Mode2PitchPeriodMs);
        }
        if (j.contains("Mode3") && j.at("Mode3").is_object()) {
            const auto& mode = j.at("Mode3");
            ps.Mode3YawStepDegPerTick =
                mode.value("YawStepDegPerTick", ps.Mode3YawStepDegPerTick);
            ps.Mode3PitchOffsetDeg =
                mode.value("PitchOffsetDeg", ps.Mode3PitchOffsetDeg);
            ps.Mode3PitchHalfRangeDeg =
                mode.value("PitchHalfRangeDeg", ps.Mode3PitchHalfRangeDeg);
            ps.Mode3PitchPeriodMs =
                mode.value("PitchPeriodMs", ps.Mode3PitchPeriodMs);
        }
    }

    void from_json(const json& j, Rate& r) {
        r.FireRate = j.value("FireRate", r.FireRate);
        r.TreeTickRate = j.value("TreeTickRate", r.TreeTickRate);
        r.NaviCommandRate = j.value("NaviCommandRate", r.NaviCommandRate);
    }
    void from_json(const json& j, BuffTimerSetting& bs) {
        bs.Enable = j.value("Enable", bs.Enable);
        bs.StartSec = j.value("StartSec", bs.StartSec);
        bs.EndSec = j.value("EndSec", bs.EndSec);
        bs.MaxShootCount = j.value("MaxShootCount", bs.MaxShootCount);
    }
    void from_json(const json& j, BuffConfirmSetting& bs) {
        bs.RefereeFreshTimeoutMs = j.value("RefereeFreshTimeoutMs", bs.RefereeFreshTimeoutMs);
        bs.PulseMs = j.value("PulseMs", bs.PulseMs);
        bs.RetryIntervalMs = j.value("RetryIntervalMs", bs.RetryIntervalMs);
        bs.PostConfirmGraceMs = j.value("PostConfirmGraceMs", bs.PostConfirmGraceMs);
        bs.TaskHoldTimeoutMs = j.value("TaskHoldTimeoutMs", bs.TaskHoldTimeoutMs);
        bs.DamageAbortThreshold = j.value("DamageAbortThreshold", bs.DamageAbortThreshold);
        bs.DamageAbortWindowMs = j.value("DamageAbortWindowMs", bs.DamageAbortWindowMs);
        bs.DamageAbortHoldMs = j.value("DamageAbortHoldMs", bs.DamageAbortHoldMs);
    }
    void from_json(const json& j, OutpostConfirmSetting& os) {
        os.RefereeFreshTimeoutMs = j.value("RefereeFreshTimeoutMs", os.RefereeFreshTimeoutMs);
        os.TrustEnemyOutpostHp = j.value("TrustEnemyOutpostHp", os.TrustEnemyOutpostHp);
        os.MaxGameTimeSec = j.value("MaxGameTimeSec", os.MaxGameTimeSec);
        os.MinSelfHp = j.value("MinSelfHp", os.MinSelfHp);
        os.MinAmmo = j.value("MinAmmo", os.MinAmmo);
        os.VisualScoutWithoutHp = j.value("VisualScoutWithoutHp", os.VisualScoutWithoutHp);
        os.VisualScoutHoldMs = j.value("VisualScoutHoldMs", os.VisualScoutHoldMs);
        os.VisualScoutCooldownMs = j.value("VisualScoutCooldownMs", os.VisualScoutCooldownMs);
        os.VisualScoutFaceDistanceCm = j.value("VisualScoutFaceDistanceCm", os.VisualScoutFaceDistanceCm);
        os.PostWindowScoutEnable = j.value("PostWindowScoutEnable", os.PostWindowScoutEnable);
        os.PostWindowScoutIntervalSec = j.value("PostWindowScoutIntervalSec", os.PostWindowScoutIntervalSec);
        os.PostWindowScoutHoldMs = j.value("PostWindowScoutHoldMs", os.PostWindowScoutHoldMs);
        os.ArmorWarningDistanceCm = j.value("ArmorInterruptMaxDistanceCm", os.ArmorWarningDistanceCm);
        os.ArmorWarningDistanceCm = j.value("ArmorWarningDistanceCm", os.ArmorWarningDistanceCm);
        os.ArmorInterruptMaxDistanceCm = os.ArmorWarningDistanceCm;
        os.PostArmorFaceSearchMs = j.value("PostArmorFaceSearchMs", os.PostArmorFaceSearchMs);
        os.DamageAbortThreshold = j.value("DamageAbortThreshold", os.DamageAbortThreshold);
        os.DamageAbortWindowMs = j.value("DamageAbortWindowMs", os.DamageAbortWindowMs);
        os.DamageAbortHoldMs = j.value("DamageAbortHoldMs", os.DamageAbortHoldMs);
        os.OpeningHighPriority = j.value("OpeningHighPriority", os.OpeningHighPriority);
        os.OpeningHoldUntilWindowEnd =
            j.value("OpeningHoldUntilWindowEnd", os.OpeningHoldUntilWindowEnd);
        os.SuppressChaseWhileActive = j.value("SuppressChaseWhileActive", os.SuppressChaseWhileActive);
        if (j.contains("ManualGoal") && j.at("ManualGoal").is_object()) {
            const auto& manual = j.at("ManualGoal");
            os.ManualGoalEnable = manual.value("Enable", os.ManualGoalEnable);
            os.ManualGoalMapXM = manual.value("MapXM", os.ManualGoalMapXM);
            os.ManualGoalMapYM = manual.value("MapYM", os.ManualGoalMapYM);
            os.ManualGoalMapZM = manual.value("MapZM", os.ManualGoalMapZM);
        }
        os.ManualGoalEnable = j.value("ManualGoalEnable", os.ManualGoalEnable);
        os.ManualGoalMapXM = j.value("ManualGoalMapXM", os.ManualGoalMapXM);
        os.ManualGoalMapYM = j.value("ManualGoalMapYM", os.ManualGoalMapYM);
        os.ManualGoalMapZM = j.value("ManualGoalMapZM", os.ManualGoalMapZM);
    }
    void from_json(const json& j, TaskSetting& ts) {
        ts.Buff = j.value("Buff", ts.Buff);
        ts.Outpost = j.value("Outpost", ts.Outpost);
        if (j.contains("BuffTimer")) {
            j.at("BuffTimer").get_to(ts.BuffTimer);
        }
        if (j.contains("BuffConfirm")) {
            j.at("BuffConfirm").get_to(ts.BuffConfirm);
        }
        if (j.contains("OutpostConfirm")) {
            j.at("OutpostConfirm").get_to(ts.OutpostConfirm);
        }
    }

    void from_json(const json& j, DamageOpenGateSetting& dog) {
        dog.Enable = j.value("Enable", dog.Enable);
        dog.HealthDropThreshold = j.value("HealthDropThreshold", dog.HealthDropThreshold);
    }

    void from_json(const json& j, StartGateSetting& sg) {
        sg.AllowGimbalPatrolBeforeStart =
            j.value("AllowGimbalPatrolBeforeStart", sg.AllowGimbalPatrolBeforeStart);
    }

    void from_json(const json& j, NaviSetting& ns) {
        ns.UseXY = j.value("UseXY", ns.UseXY);
        if (j.contains("ToNavi")) {
            ns.ToNavi = j.value("ToNavi", ns.ToNavi);
        } else {
            ns.ToNavi = j.value("UseTfGoalBridge", ns.ToNavi);
        }
    }

    void from_json(const json& j, FaceModeSetting& fs) {
        fs.Enable = j.value("Enable", fs.Enable);
        fs.LostTargetHoldMs = j.value("LostTargetHoldMs", fs.LostTargetHoldMs);
        fs.SuppressFire = j.value("SuppressFire", fs.SuppressFire);
        fs.FallbackToPatrolScanMode2 =
            j.value("FallbackToPatrolScanMode2", fs.FallbackToPatrolScanMode2);
        fs.FallbackPatrolScanMode =
            j.value("FallbackPatrolScanMode", fs.FallbackPatrolScanMode);
        fs.OutpostFallbackPatrolScanMode =
            j.value("OutpostFallbackPatrolScanMode", fs.OutpostFallbackPatrolScanMode);
    }

    void from_json(const json& j, ExternalAimSetting& ea) {
        // Behavion branch: external aim is the only official aim chain.
        // Keep the Enable field in old JSON files as documentation only.
        ea.Enable = true;
        ea.ResultFreshTimeoutMs = j.value("ResultFreshTimeoutMs", ea.ResultFreshTimeoutMs);
        ea.TargetFreshTimeoutMs = j.value("TargetFreshTimeoutMs", ea.TargetFreshTimeoutMs);
        ea.UseTargetArrayAsArmorList =
            j.value("UseTargetArrayAsArmorList", ea.UseTargetArrayAsArmorList);
        ea.PublishSelectTarget = j.value("PublishSelectTarget", ea.PublishSelectTarget);
        ea.TargetDefaultFrame = j.value("TargetDefaultFrame", ea.TargetDefaultFrame);
    }

    void from_json(const json& j, NaviRotateControlSetting& nr) {
        nr.Enable = j.value("Enable", nr.Enable);
        nr.FreshTimeoutMs = j.value("FreshTimeoutMs", nr.FreshTimeoutMs);
        nr.DefaultIsRotate = j.value("DefaultIsRotate", nr.DefaultIsRotate);
        nr.ForceFollowModeWhenFalse = j.value("ForceFollowModeWhenFalse", nr.ForceFollowModeWhenFalse);
        nr.ClearFollowModeWhenTrue = j.value("ClearFollowModeWhenTrue", nr.ClearFollowModeWhenTrue);
        nr.ClearRegionalFaceModeWhenTrue =
            j.value("ClearRegionalFaceModeWhenTrue", nr.ClearRegionalFaceModeWhenTrue);
        nr.StopRotateWhenFalse = j.value("StopRotateWhenFalse", nr.StopRotateWhenFalse);
    }

    void from_json(const json& j, PointRotateSetting& pr) {
        pr.Enable = j.value("Enable", pr.Enable);
        pr.Rotate = j.value("Rotate", pr.Rotate);
    }

    void from_json(const json& j, PointManagerSetting& pm) {
        if (j.contains("Global") && j.at("Global").is_object()) {
            j.at("Global").get_to(pm.Global);
        }
        if (!j.contains("Points") || !j.at("Points").is_object()) {
            return;
        }
        for (const auto& [point_name, point_value] : j.at("Points").items()) {
            std::uint8_t goal_id = LangYa::Home.ID;
            if (!PointManagerGoalIdByName(point_name, goal_id)) {
                continue;
            }
            PointRotateSetting setting{};
            if (point_value.is_object()) {
                point_value.get_to(setting);
            } else if (point_value.is_number_integer()) {
                setting.Enable = true;
                setting.Rotate = point_value.get<int>();
            }
            pm.Points[goal_id] = setting;
        }
    }

    void from_json(const json& j, SentryPositionFusionSourceSetting& source) {
        source.Enable = j.value("Enable", source.Enable);
        source.Priority = j.value("Priority", source.Priority);
        source.Weight = j.value("Weight", source.Weight);
        source.FreshTimeoutMs = j.value("FreshTimeoutMs", source.FreshTimeoutMs);
    }

    void from_json(const json& j, SentryPositionFusionSetting& fusion) {
        fusion.Enable = j.value("Enable", fusion.Enable);
        fusion.Mode = j.value("Mode", fusion.Mode);
        fusion.FreshTimeoutMs = j.value("FreshTimeoutMs", fusion.FreshTimeoutMs);
        const auto read_source = [&](const char* name, SentryPositionFusionSourceSetting& source) {
            if (j.contains(name) && j.at(name).is_object()) {
                j.at(name).get_to(source);
            }
            if (j.contains("Sources") && j.at("Sources").is_object() &&
                j.at("Sources").contains(name) &&
                j.at("Sources").at(name).is_object()) {
                j.at("Sources").at(name).get_to(source);
            }
        };
        read_source("Uwb", fusion.Uwb);
        read_source("UWB", fusion.Uwb);
        read_source("PositionData", fusion.PositionData);
        read_source("Navi", fusion.Navi);
    }

    void from_json(const json& j, LeagueStrategySetting& ls) {
        ls.EnableRouteCompat = j.value("EnableRouteCompat", ls.EnableRouteCompat);
        ls.UseHealthRecovery = j.value("UseHealthRecovery", ls.UseHealthRecovery);
        ls.HealthRecoveryThreshold = j.value("HealthRecoveryThreshold", ls.HealthRecoveryThreshold);
        ls.UseAmmoRecovery = j.value("UseAmmoRecovery", ls.UseAmmoRecovery);
        ls.AmmoRecoveryThreshold = j.value("AmmoRecoveryThreshold", ls.AmmoRecoveryThreshold);
        ls.DamageScanBoostEnable = j.value("DamageScanBoostEnable", ls.DamageScanBoostEnable);
        ls.HealthRecoveryExitMin = j.value("HealthRecoveryExitMin", ls.HealthRecoveryExitMin);
        ls.HealthRecoveryExitPreferred = j.value("HealthRecoveryExitPreferred", ls.HealthRecoveryExitPreferred);
        ls.HealthRecoveryPlateauSec = j.value("HealthRecoveryPlateauSec", ls.HealthRecoveryPlateauSec);
        ls.HealthRecoveryExitStableSec = j.value("HealthRecoveryExitStableSec", ls.HealthRecoveryExitStableSec);
        ls.HealthRecoveryMaxHoldSec = j.value("HealthRecoveryMaxHoldSec", ls.HealthRecoveryMaxHoldSec);
        ls.HealthRecoveryCooldownSec = j.value("HealthRecoveryCooldownSec", ls.HealthRecoveryCooldownSec);
        ls.MainGoal = j.value("MainGoal", ls.MainGoal);
        ls.GoalHoldSec = j.value("GoalHoldSec", ls.GoalHoldSec);
        if (j.contains("PatrolGoals")) {
            j.at("PatrolGoals").get_to(ls.PatrolGoals);
        }
    }

    void from_json(const json& j, ShowcasePatrolSetting& sp) {
        sp.Enable = j.value("Enable", sp.Enable);
        sp.GoalHoldSec = j.value("GoalHoldSec", sp.GoalHoldSec);
        sp.Random = j.value("Random", sp.Random);
        sp.DisableTeamOffset = j.value("DisableTeamOffset", sp.DisableTeamOffset);
        sp.IgnoreRecovery = j.value("IgnoreRecovery", sp.IgnoreRecovery);
        if (j.contains("Goals")) {
            j.at("Goals").get_to(sp.Goals);
        }
    }

    void from_json(const json& j, NaviDebugSetting& nd) {
        nd.Enable = j.value("Enable", nd.Enable);
        nd.PlanFile = j.value("PlanFile", nd.PlanFile);
        nd.ActivePlan = j.value("ActivePlan", nd.ActivePlan);
        nd.GoalHoldSec = j.value("GoalHoldSec", nd.GoalHoldSec);
        nd.Random = j.value("Random", nd.Random);
        nd.DisableTeamOffset = j.value("DisableTeamOffset", nd.DisableTeamOffset);
        nd.IgnoreRecovery = j.value("IgnoreRecovery", nd.IgnoreRecovery);
        nd.SpeedLevel = static_cast<std::uint8_t>(
            std::clamp(j.value("SpeedLevel", static_cast<int>(nd.SpeedLevel)), 0, 2));
        if (j.contains("Goals")) {
            j.at("Goals").get_to(nd.Goals);
        }
    }

    void from_json(const json& j, ChaseAreaLimitSetting& ca) {
        ca.Enable = j.value("Enable", ca.Enable);
        ca.BoundaryMarginCm = j.value("BoundaryMarginCm", ca.BoundaryMarginCm);
        ca.ChaseEnableCrossArea = j.value("ChaseEnableCrossArea", ca.ChaseEnableCrossArea);
        ca.HoldWhenNoIntersection = j.value("HoldWhenNoIntersection", ca.HoldWhenNoIntersection);
    }

    void from_json(const json& j, ChaseSetting& cs) {
        cs.Enable = j.value("Enable", cs.Enable);
        cs.FollowAimTarget = j.value("FollowAimTarget", cs.FollowAimTarget);
        if (j.contains("ToNavi")) {
            cs.ToNavi = j.value("ToNavi", cs.ToNavi);
        } else {
            cs.ToNavi = j.value("UseRelativeTargetTopic", cs.ToNavi);
        }
        cs.UseOfficialPositionSource = j.value("UseOfficialPositionSource", cs.UseOfficialPositionSource);
        cs.PreferOfficialPositionSource = j.value("PreferOfficialPositionSource", cs.PreferOfficialPositionSource);
        cs.OfficialPositionFreshMs = j.value("OfficialPositionFreshMs", cs.OfficialPositionFreshMs);
        cs.EnableInAutoAim = j.value("EnableInAutoAim", cs.EnableInAutoAim);
        cs.EnableInRotateScan = j.value("EnableInRotateScan", cs.EnableInRotateScan);
        cs.EnableInOutpostMode = j.value("EnableInOutpostMode", cs.EnableInOutpostMode);
        cs.EnableInBuffMode = j.value("EnableInBuffMode", cs.EnableInBuffMode);
        cs.StopWhenNoTarget = j.value("StopWhenNoTarget", cs.StopWhenNoTarget);
        cs.LostTargetHoldMs = j.value("LostTargetHoldMs", cs.LostTargetHoldMs);
        cs.PreferredDistanceCm = j.value("PreferredDistanceCm", cs.PreferredDistanceCm);
        cs.DistanceDeadbandCm = j.value("DistanceDeadbandCm", cs.DistanceDeadbandCm);
        if (j.contains("AreaLimit") && j.at("AreaLimit").is_object()) {
            j.at("AreaLimit").get_to(cs.AreaLimit);
        }
        cs.MinValidDistanceCm = j.value("MinValidDistanceCm", cs.MinValidDistanceCm);
        cs.MaxValidDistanceCm = j.value("MaxValidDistanceCm", cs.MaxValidDistanceCm);
        cs.DistanceKp = j.value("DistanceKp", cs.DistanceKp);
        cs.MaxForwardSpeed = j.value("MaxForwardSpeed", cs.MaxForwardSpeed);
        cs.MaxBackwardSpeed = j.value("MaxBackwardSpeed", cs.MaxBackwardSpeed);
        cs.UseYawStrafe = j.value("UseYawStrafe", cs.UseYawStrafe);
        cs.YawKp = j.value("YawKp", cs.YawKp);
        cs.YawDeadbandDeg = j.value("YawDeadbandDeg", cs.YawDeadbandDeg);
        cs.MaxStrafeSpeed = j.value("MaxStrafeSpeed", cs.MaxStrafeSpeed);
        cs.InvertStrafeDirection = j.value("InvertStrafeDirection", cs.InvertStrafeDirection);
    }

    void from_json(const json& j, PostureSetting& ps) {
        ps.Enable = j.value("Enable", ps.Enable);
        ps.SwitchCooldownSec = j.value("SwitchCooldownSec", ps.SwitchCooldownSec);
        ps.MaxSinglePostureSec = j.value("MaxSinglePostureSec", ps.MaxSinglePostureSec);
        ps.EarlyRotateSec = j.value("EarlyRotateSec", ps.EarlyRotateSec);
        ps.MinHoldSec = j.value("MinHoldSec", ps.MinHoldSec);
        ps.PendingAckTimeoutMs = j.value("PendingAckTimeoutMs", ps.PendingAckTimeoutMs);
        ps.RetryIntervalMs = j.value("RetryIntervalMs", ps.RetryIntervalMs);
        ps.MaxRetryCount = j.value("MaxRetryCount", ps.MaxRetryCount);
        ps.OptimisticAck = j.value("OptimisticAck", ps.OptimisticAck);
        ps.TargetKeepMs = j.value("TargetKeepMs", ps.TargetKeepMs);
        ps.DamageKeepSec = j.value("DamageKeepSec", ps.DamageKeepSec);
        ps.DamageBurstWindowMs = j.value("DamageBurstWindowMs", ps.DamageBurstWindowMs);
        ps.DamageBurstThreshold = j.value("DamageBurstThreshold", ps.DamageBurstThreshold);
        ps.DamageBurstDefenseHoldSec = j.value("DamageBurstDefenseHoldSec", ps.DamageBurstDefenseHoldSec);
        ps.LowHealthThreshold = j.value("LowHealthThreshold", ps.LowHealthThreshold);
        ps.VeryLowHealthThreshold = j.value("VeryLowHealthThreshold", ps.VeryLowHealthThreshold);
        ps.LowAmmoThreshold = j.value("LowAmmoThreshold", ps.LowAmmoThreshold);
        ps.ScoreHysteresis = j.value("ScoreHysteresis", ps.ScoreHysteresis);
    }

    void from_json(const json& j, NaviGoalAutonomySetting& na) {
        na.UseAreaScope = j.value("UseAreaScope", na.UseAreaScope);
        if (j.contains("MyArea")) {
            na.MyArea = ParseAreaScopeList(j.at("MyArea"));
        }
        if (j.contains("EnemyArea")) {
            na.EnemyArea = ParseAreaScopeList(j.at("EnemyArea"));
        }
        if (j.contains("CommonArea")) {
            na.CommonArea = ParseAreaScopeList(j.at("CommonArea"));
        }
        if (j.contains("HighlandCompat") && j.at("HighlandCompat").is_object()) {
            const auto& compat = j.at("HighlandCompat");
            na.HighlandCompatEnable = compat.value("Enable", na.HighlandCompatEnable);
            na.HighlandCompatDisableRotate = compat.value("DisableRotate", na.HighlandCompatDisableRotate);
            na.HighlandCompatArriveDistanceCm = compat.value("ArriveDistanceCm", na.HighlandCompatArriveDistanceCm);
            na.HighlandCompatTimeoutSec = compat.value("TimeoutSec", na.HighlandCompatTimeoutSec);
            na.DistanceFallbackGraceMs = compat.value("DistanceFallbackGraceMs", na.DistanceFallbackGraceMs);
        }
        if (j.contains("BuffOutpostCompat") && j.at("BuffOutpostCompat").is_object()) {
            const auto& compat = j.at("BuffOutpostCompat");
            na.BuffOutpostCompatEnable = compat.value("Enable", na.BuffOutpostCompatEnable);
            na.BuffOutpostCompatTimeoutSec = compat.value("TimeoutSec", na.BuffOutpostCompatTimeoutSec);
        }
        na.HighlandCompatEnable = j.value("HighlandCompatEnable", na.HighlandCompatEnable);
        na.HighlandCompatDisableRotate = j.value("HighlandCompatDisableRotate", na.HighlandCompatDisableRotate);
        na.HighlandCompatArriveDistanceCm =
            j.value("HighlandCompatArriveDistanceCm", na.HighlandCompatArriveDistanceCm);
        na.HighlandCompatTimeoutSec = j.value("HighlandCompatTimeoutSec", na.HighlandCompatTimeoutSec);
        na.BuffOutpostCompatEnable = j.value("BuffOutpostCompatEnable", na.BuffOutpostCompatEnable);
        na.BuffOutpostCompatTimeoutSec =
            j.value("BuffOutpostCompatTimeoutSec", na.BuffOutpostCompatTimeoutSec);
        na.DistanceFallbackGraceMs = j.value("DistanceFallbackGraceMs", na.DistanceFallbackGraceMs);
    }

    void from_json(const json& j, AimTargetAutonomySetting& aa) {
        aa.Enable = j.value("Enable", aa.Enable);
        aa.PriorityWeight = j.value("PriorityWeight", aa.PriorityWeight);
        aa.DistanceWeight = j.value("DistanceWeight", aa.DistanceWeight);
        aa.LowHealthWeight = j.value("LowHealthWeight", aa.LowHealthWeight);
        aa.CurrentTargetBonus = j.value("CurrentTargetBonus", aa.CurrentTargetBonus);
        aa.HeroBonus = j.value("HeroBonus", aa.HeroBonus);
        aa.SentryBonus = j.value("SentryBonus", aa.SentryBonus);
        aa.HealthFreshTimeoutMs = j.value("HealthFreshTimeoutMs", aa.HealthFreshTimeoutMs);
        aa.DeadHealthConfirmMs = j.value("DeadHealthConfirmMs", aa.DeadHealthConfirmMs);
        aa.DeadHealthHoldMs = j.value("DeadHealthHoldMs", aa.DeadHealthHoldMs);
        aa.RespawnTransitionTimeoutMs =
            j.value("RespawnTransitionTimeoutMs", aa.RespawnTransitionTimeoutMs);
        aa.LostTargetHoldMs = j.value("LostTargetHoldMs", aa.LostTargetHoldMs);
        aa.MinSwitchIntervalMs = j.value("MinSwitchIntervalMs", aa.MinSwitchIntervalMs);
        aa.SwitchScoreMargin = j.value("SwitchScoreMargin", aa.SwitchScoreMargin);
        aa.RespawnInvulnerableSec = j.value("RespawnInvulnerableSec", aa.RespawnInvulnerableSec);
        aa.SentryRespawnInvulnerableSec =
            j.value("SentryRespawnInvulnerableSec", aa.SentryRespawnInvulnerableSec);
    }

    void from_json(const json& j, RegionalDefenseSetting& rd) {
        rd.Enable = j.value("Enable", rd.Enable);
        rd.EnableSoftEnemySideThreat = j.value("EnableSoftEnemySideThreat", rd.EnableSoftEnemySideThreat);
        rd.EnemyPositionFreshMs = j.value("EnemyPositionFreshMs", rd.EnemyPositionFreshMs);
        rd.HardHoldSec = j.value("HardHoldSec", rd.HardHoldSec);
        rd.SoftHoldSec = j.value("SoftHoldSec", rd.SoftHoldSec);
        rd.SearchHoldSec = j.value("SearchHoldSec", rd.SearchHoldSec);
        rd.SearchNoTargetSec = j.value("SearchNoTargetSec", rd.SearchNoTargetSec);
        rd.FortressStandEnemyCountMin = j.value("FortressStandEnemyCountMin", rd.FortressStandEnemyCountMin);
        rd.FortressNoContactDegradeSec = j.value("FortressNoContactDegradeSec", rd.FortressNoContactDegradeSec);
        rd.FortressDegradeCooldownSec = j.value("FortressDegradeCooldownSec", rd.FortressDegradeCooldownSec);
        rd.StrongHealthMin = j.value("StrongHealthMin", rd.StrongHealthMin);
        rd.StrongAmmoMin = j.value("StrongAmmoMin", rd.StrongAmmoMin);
        rd.MultiEnemyBaseCount = j.value("MultiEnemyBaseCount", rd.MultiEnemyBaseCount);
    }

    void from_json(const json& j, HeroProtectionSetting& hp) {
        hp.Enable = j.value("Enable", hp.Enable);
        hp.StartElapsedSec = j.value("StartElapsedSec", hp.StartElapsedSec);
        hp.HoldSec = j.value("HoldSec", hp.HoldSec);
        hp.NoEnemyReleaseSec = j.value("NoEnemyReleaseSec", hp.NoEnemyReleaseSec);
        hp.FriendPositionFreshMs = j.value("FriendPositionFreshMs", hp.FriendPositionFreshMs);
        hp.FriendHealthFreshMs = j.value("FriendHealthFreshMs", hp.FriendHealthFreshMs);
        const int goal_base_id = j.value("GoalBaseId", static_cast<int>(hp.GoalBaseId));
        hp.GoalBaseId = static_cast<std::uint8_t>(std::clamp(goal_base_id, 0, 255));
    }

    void from_json(const json& j, NaviProgressWatchdogSetting& np) {
        np.Enable = j.value("Enable", np.Enable);
        np.ArriveDistanceCm = j.value("ArriveDistanceCm", np.ArriveDistanceCm);
        np.MoveProgressCm = j.value("MoveProgressCm", np.MoveProgressCm);
        np.NoMoveTimeoutSec = j.value("NoMoveTimeoutSec", np.NoMoveTimeoutSec);
        np.FallbackHoldSec = j.value("FallbackHoldSec", np.FallbackHoldSec);
        np.FallbackCooldownSec = j.value("FallbackCooldownSec", np.FallbackCooldownSec);
    }

    void from_json(const json& j, RegionalIdlePatrolSetting& rp) {
        rp.Enable = j.value("Enable", rp.Enable);
        rp.GoalHoldSec = j.value("GoalHoldSec", rp.GoalHoldSec);
        if (j.contains("Goals")) {
            j.at("Goals").get_to(rp.Goals);
        }
        if (j.contains("GoalEnable") && j.at("GoalEnable").is_object()) {
            rp.GoalEnableProvided = true;
            rp.Goals.clear();
            const auto& goal_enable = j.at("GoalEnable");
            const std::vector<std::string> ordered_goal_names{
                "Highland",
                "BuffShoot",
                "HoleRoad",
                "CastleLeft1",
                "CastleLeft2",
                "CastleRight2",
                "CastleRight1",
                "Castle"
            };
            for (const auto& goal_name : ordered_goal_names) {
                if (!goal_enable.contains(goal_name) ||
                    !goal_enable.at(goal_name).is_boolean() ||
                    !goal_enable.at(goal_name).get<bool>()) {
                    continue;
                }
                AppendRegionalPatrolGoalByName(goal_name, rp.Goals);
            }
            for (auto it = goal_enable.begin(); it != goal_enable.end(); ++it) {
                const auto goal_name = it.key();
                const bool known_goal = std::find(
                    ordered_goal_names.begin(),
                    ordered_goal_names.end(),
                    goal_name) != ordered_goal_names.end();
                const bool enabled = it.value().is_boolean() ? it.value().get<bool>() : false;
                if (known_goal || !enabled) {
                    continue;
                }
                AppendRegionalPatrolGoalByName(goal_name, rp.Goals);
            }
        }
    }

    void from_json(const json& j, SpecialMiniRoadlandSetting& mr) {
        mr.Enable = j.value("Enable", mr.Enable);
        mr.GoalHoldSec = j.value("GoalHoldSec", mr.GoalHoldSec);
        mr.SpeedLevel = j.value("SpeedLevel", mr.SpeedLevel);
        const int goal_base_id = j.value("GoalBaseId", static_cast<int>(mr.GoalBaseId));
        mr.GoalBaseId = static_cast<std::uint8_t>(std::clamp(goal_base_id, 0, 255));
    }

    void from_json(const json& j, SpecialPatrolSetting& sp) {
        sp.Enable = j.value("Enable", sp.Enable);
        sp.GoalHoldSec = j.value("GoalHoldSec", sp.GoalHoldSec);
        sp.SpeedLevel = j.value("SpeedLevel", sp.SpeedLevel);
        sp.SuppressChase = j.value("SuppressChase", sp.SuppressChase);
        sp.StopOnTarget = j.value("StopOnTarget", sp.StopOnTarget);
    }

    void from_json(const json& j, SpecialSetting& ss) {
        if (j.contains("MiniRoadland")) {
            if (j.at("MiniRoadland").is_object()) {
                j.at("MiniRoadland").get_to(ss.MiniRoadland);
            } else if (j.at("MiniRoadland").is_boolean()) {
                ss.MiniRoadland.Enable = j.at("MiniRoadland").get<bool>();
            }
        }
        if (j.contains("Patrol")) {
            if (j.at("Patrol").is_object()) {
                j.at("Patrol").get_to(ss.Patrol);
            } else if (j.at("Patrol").is_boolean()) {
                ss.Patrol.Enable = j.at("Patrol").get<bool>();
            }
        }
    }

    void from_json(const json& j, MyHighlandAreaTaskSetting& hs) {
        hs.Enable = j.value("Enable", hs.Enable);
        hs.UseFaceMode = j.value("UseFaceMode", hs.UseFaceMode);
        hs.ApproachTimeoutSec = j.value("ApproachTimeoutSec", hs.ApproachTimeoutSec);
        hs.HighlandPatrolHoldSec = j.value("HighlandPatrolHoldSec", hs.HighlandPatrolHoldSec);
        hs.BuffShootTravelTimeoutSec = j.value("BuffShootTravelTimeoutSec", hs.BuffShootTravelTimeoutSec);
        hs.BuffShootHoldSec = j.value("BuffShootHoldSec", hs.BuffShootHoldSec);
        hs.LeaveTimeoutSec = j.value("LeaveTimeoutSec", hs.LeaveTimeoutSec);
    }

    void from_json(const json& j, MyBaseAreaTaskSetting& bs) {
        bs.Enable = j.value("Enable", bs.Enable);
        bs.TravelTimeoutSec = j.value("TravelTimeoutSec", bs.TravelTimeoutSec);
        bs.CommandHoldSec = j.value("CommandHoldSec", bs.CommandHoldSec);
        bs.GoalHoldSec = j.value("GoalHoldSec", bs.GoalHoldSec);
        bs.MaxPatrolSteps = j.value("MaxPatrolSteps", bs.MaxPatrolSteps);
        if (j.contains("Patrol") && j.at("Patrol").is_object()) {
            const auto& patrol = j.at("Patrol");
            bs.PatrolDistancePenaltyPerMeter =
                patrol.value("DistancePenaltyPerMeter", bs.PatrolDistancePenaltyPerMeter);
            bs.PatrolCurrentGoalPenalty =
                patrol.value("CurrentGoalPenalty", bs.PatrolCurrentGoalPenalty);
            if (patrol.contains("GoalWeights") && patrol.at("GoalWeights").is_object()) {
                bs.PatrolGoals.clear();
                for (auto it = patrol.at("GoalWeights").begin(); it != patrol.at("GoalWeights").end(); ++it) {
                    std::uint8_t goal_id = LangYa::Home.ID;
                    if (!MyBasePatrolGoalIdByName(it.key(), goal_id) ||
                        !it.value().is_number()) {
                        continue;
                    }
                    bs.PatrolGoals.push_back(LangYa::MyBasePatrolGoalSetting{
                        .BaseGoalId = goal_id,
                        .Weight = it.value().get<double>()
                    });
                }
            }
        }
    }

    void from_json(const json& j, MyRoadlandAreaTaskSetting& rs) {
        rs.Enable = j.value("Enable", rs.Enable);
        rs.UseFaceMode = j.value("UseFaceMode", rs.UseFaceMode);
        rs.TravelTimeoutSec = j.value("TravelTimeoutSec", rs.TravelTimeoutSec);
        rs.CrossTimeoutSec = j.value("CrossTimeoutSec", rs.CrossTimeoutSec);
        rs.CommandHoldSec = j.value("CommandHoldSec", rs.CommandHoldSec);
        rs.GuardHoldSec = j.value("GuardHoldSec", rs.GuardHoldSec);
        rs.FaceTargetZCm = j.value("FaceTargetZCm", rs.FaceTargetZCm);
        rs.HealthyHpMin = j.value("HealthyHpMin", rs.HealthyHpMin);
        rs.HealthyAmmoMin = j.value("HealthyAmmoMin", rs.HealthyAmmoMin);
    }

    void from_json(const json& j, CommonCentralAreaTaskSetting& cs) {
        cs.Enable = j.value("Enable", cs.Enable);
        cs.TravelTimeoutSec = j.value("TravelTimeoutSec", cs.TravelTimeoutSec);
        cs.CommandHoldSec = j.value("CommandHoldSec", cs.CommandHoldSec);
        cs.MaxPatrolSteps = j.value("MaxPatrolSteps", cs.MaxPatrolSteps);
        cs.HealthyHpMin = j.value("HealthyHpMin", cs.HealthyHpMin);
        cs.HealthyAmmoMin = j.value("HealthyAmmoMin", cs.HealthyAmmoMin);
    }

    void from_json(const json& j, DefaultPolicyHealthSetting& hs) {
        hs.MyAreaHpMin = j.value("MyAreaHpMin", hs.MyAreaHpMin);
        hs.CommonCentralHpMin = j.value("CommonCentralHpMin", hs.CommonCentralHpMin);
        hs.EnemyAreaHpMin = j.value("EnemyAreaHpMin", hs.EnemyAreaHpMin);
        hs.LowResourceFallbackHp = j.value("LowResourceFallbackHp", hs.LowResourceFallbackHp);
    }

    void from_json(const json& j, DefaultPolicyAmmoSetting& as) {
        as.MyAreaAmmoMin = j.value("MyAreaAmmoMin", as.MyAreaAmmoMin);
        as.CommonCentralAmmoMin = j.value("CommonCentralAmmoMin", as.CommonCentralAmmoMin);
        as.EnemyAreaAmmoMin = j.value("EnemyAreaAmmoMin", as.EnemyAreaAmmoMin);
        as.LowResourceFallbackAmmo = j.value("LowResourceFallbackAmmo", as.LowResourceFallbackAmmo);
    }

    void from_json(const json& j, DefaultPolicyScoreSetting& ss) {
        ss.WeightMyBase = j.value("WeightMyBase", ss.WeightMyBase);
        ss.WeightMyHighland = j.value("WeightMyHighland", ss.WeightMyHighland);
        ss.WeightMyRoadland = j.value("WeightMyRoadland", ss.WeightMyRoadland);
        ss.WeightCommonCentral = j.value("WeightCommonCentral", ss.WeightCommonCentral);
        ss.WeightEnemyBase = j.value("WeightEnemyBase", ss.WeightEnemyBase);
        ss.WeightEnemyHighland = j.value("WeightEnemyHighland", ss.WeightEnemyHighland);
        ss.WeightEnemyRoadland = j.value("WeightEnemyRoadland", ss.WeightEnemyRoadland);
        ss.DistancePenaltyPerMeter = j.value("DistancePenaltyPerMeter", ss.DistancePenaltyPerMeter);
        ss.CurrentAreaPenalty = j.value("CurrentAreaPenalty", ss.CurrentAreaPenalty);
        ss.LastAreaPenalty = j.value("LastAreaPenalty", ss.LastAreaPenalty);
        ss.AfterHighlandMyBaseBonus = j.value("AfterHighlandMyBaseBonus", ss.AfterHighlandMyBaseBonus);
        ss.AfterHighlandMyRoadlandBonus = j.value("AfterHighlandMyRoadlandBonus", ss.AfterHighlandMyRoadlandBonus);
        ss.LowResourceMyBaseBonus = j.value("LowResourceMyBaseBonus", ss.LowResourceMyBaseBonus);
    }

    void from_json(const json& j, DefaultPolicyRetrySetting& rs) {
        rs.CompleteCooldownSec = j.value("CompleteCooldownSec", rs.CompleteCooldownSec);
        rs.FailureCooldownSec = j.value("FailureCooldownSec", rs.FailureCooldownSec);
        rs.UnreachableCooldownSec = j.value("UnreachableCooldownSec", rs.UnreachableCooldownSec);
        rs.MaxRetry = j.value("MaxRetry", rs.MaxRetry);
    }

    void from_json(const json& j, DefaultPolicySetting& ds) {
        ds.Enable = j.value("Enable", ds.Enable);
        if (j.contains("Health") && j.at("Health").is_object()) {
            j.at("Health").get_to(ds.Health);
        }
        if (j.contains("Ammo") && j.at("Ammo").is_object()) {
            j.at("Ammo").get_to(ds.Ammo);
        }
        if (j.contains("Score") && j.at("Score").is_object()) {
            j.at("Score").get_to(ds.Score);
        }
        if (j.contains("Retry") && j.at("Retry").is_object()) {
            j.at("Retry").get_to(ds.Retry);
        }
    }

    void from_json(const json& j, RegionalAreaTaskSetting& rt) {
        rt.Enable = j.value("Enable", rt.Enable);
        rt.IgnoreRecovery = j.value("IgnoreRecovery", rt.IgnoreRecovery);
        if (j.contains("MyHighland") && j.at("MyHighland").is_object()) {
            j.at("MyHighland").get_to(rt.MyHighland);
        }
        if (j.contains("MyBase") && j.at("MyBase").is_object()) {
            j.at("MyBase").get_to(rt.MyBase);
        }
        if (j.contains("MyRoadland") && j.at("MyRoadland").is_object()) {
            j.at("MyRoadland").get_to(rt.MyRoadland);
        }
        if (j.contains("CommonCentral") && j.at("CommonCentral").is_object()) {
            j.at("CommonCentral").get_to(rt.CommonCentral);
        }
        if (j.contains("DefaultPolicy") && j.at("DefaultPolicy").is_object()) {
            j.at("DefaultPolicy").get_to(rt.DefaultPolicy);
        }
    }

    void from_json(const json& j, DecisionAutonomySetting& da) {
        da.Enable = j.value("Enable", da.Enable);
        if (j.contains("EnabledModules")) {
            j.at("EnabledModules").get_to(da.EnabledModules);
        }
        if (j.contains("HardRuleModules")) {
            j.at("HardRuleModules").get_to(da.HardRuleModules);
        }
        if (j.contains("NaviGoal")) {
            j.at("NaviGoal").get_to(da.NaviGoal);
        }
        if (j.contains("AimTarget")) {
            j.at("AimTarget").get_to(da.AimTarget);
        }
    }

    void from_json(const json& j, Config& c) {
        if (j.contains("AimDebug")) {
            j.at("AimDebug").get_to(c.AimDebugSettings);
        }
        if (j.contains("PatrolScan")) {
            j.at("PatrolScan").get_to(c.PatrolScanSettings);
        }
        if (j.contains("Rate")) {
            j.at("Rate").get_to(c.RateSettings);
        }
        c.SwitchPoint = j.value("Switch_Point", c.SwitchPoint);
        c.SwitchPoint = j.value("SwitchPoint", c.SwitchPoint);
        if (j.contains("Task")) {
            j.at("Task").get_to(c.TaskSettings);
        }
        if (j.contains("DamageOpenGate")) {
            j.at("DamageOpenGate").get_to(c.DamageOpenGateSettings);
        }
        if (j.contains("StartGate")) {
            j.at("StartGate").get_to(c.StartGateSettings);
        }
        c.ScanCounter = j.value("ScanCounter", c.ScanCounter);
        if (j.contains("NaviSetting")) {
            j.at("NaviSetting").get_to(c.NaviSettings);
        }
        if (j.contains("FaceMode")) {
            j.at("FaceMode").get_to(c.FaceModeSettings);
        }
        if (j.contains("ExternalAim")) {
            j.at("ExternalAim").get_to(c.ExternalAimSettings);
        }
        if (j.contains("NaviRotateControl")) {
            j.at("NaviRotateControl").get_to(c.NaviRotateControlSettings);
        }
        if (j.contains("PointManager")) {
            j.at("PointManager").get_to(c.PointManagerSettings);
        }
        if (j.contains("SentryPositionFusion")) {
            j.at("SentryPositionFusion").get_to(c.SentryPositionFusionSettings);
        }
        if (j.contains("LeagueStrategy")) {
            j.at("LeagueStrategy").get_to(c.LeagueStrategySettings);
        }
        if (j.contains("ShowcasePatrol")) {
            j.at("ShowcasePatrol").get_to(c.ShowcasePatrolSettings);
        }
        if (j.contains("NaviDebug")) {
            j.at("NaviDebug").get_to(c.NaviDebugSettings);
        }
        if (j.contains("RegionalDefense")) {
            j.at("RegionalDefense").get_to(c.RegionalDefenseSettings);
        }
        if (j.contains("HeroProtection")) {
            j.at("HeroProtection").get_to(c.HeroProtectionSettings);
        }
        if (j.contains("NaviProgressWatchdog")) {
            j.at("NaviProgressWatchdog").get_to(c.NaviProgressWatchdogSettings);
        }
        if (j.contains("RegionalIdlePatrol")) {
            j.at("RegionalIdlePatrol").get_to(c.RegionalIdlePatrolSettings);
        }
        if (j.contains("Special")) {
            j.at("Special").get_to(c.SpecialSettings);
        }
        if (j.contains("RegionalAreaTask")) {
            j.at("RegionalAreaTask").get_to(c.RegionalAreaTaskSettings);
        }
        if (j.contains("AimTargetPriority")) {
            j.at("AimTargetPriority").get_to(c.AimTargetPriority);
        }
        if (j.contains("AimTargetIgnore")) {
            j.at("AimTargetIgnore").get_to(c.AimTargetIgnore);
        }
        if (j.contains("DecisionAutonomy")) {
            j.at("DecisionAutonomy").get_to(c.DecisionAutonomySettings);
        }
        if (j.contains("Chase")) {
            j.at("Chase").get_to(c.ChaseSettings);
        }
        if (j.contains("Posture")) {
            j.at("Posture").get_to(c.PostureSettings);
        }
        c.CompetitionProfile = j.value("CompetitionProfile", c.CompetitionProfile);
    }
}

namespace BehaviorTree {
    using namespace LangYa;
    using json = nlohmann::json;

    std::uint8_t Application::ResolvePointDefaultRotate(
        const std::uint8_t base_goal_id,
        const std::uint8_t fallback) const noexcept {
        if (config.PointManagerSettings.Global.Enable) {
            return ClampRotateGear(config.PointManagerSettings.Global.Rotate);
        }
        const std::uint8_t effective_base_goal_id =
            base_goal_id >= LangYa::TeamedLocation::LocationCount
                ? static_cast<std::uint8_t>(base_goal_id - LangYa::TeamedLocation::LocationCount)
                : base_goal_id;
        const auto it = config.PointManagerSettings.Points.find(effective_base_goal_id);
        if (it != config.PointManagerSettings.Points.end()) {
            return it->second.Enable ? ClampRotateGear(it->second.Rotate) : 0;
        }
        return ClampRotateGear(fallback);
    }

    void Application::ApplyTaskParameterOverrides() {
        ReadOptionalBoolParam(
            node_,
            {
                "Task.Buff",
                "Task/Buff"
            },
            config.TaskSettings.Buff);
        ReadOptionalBoolParam(
            node_,
            {
                "Task.Outpost",
                "Task/Outpost"
            },
            config.TaskSettings.Outpost);
        ReadOptionalBoolParam(
            node_,
            {
                "Task.BuffTimer.Enable",
                "Task/BuffTimer/Enable"
            },
            config.TaskSettings.BuffTimer.Enable);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffTimer.StartSec",
                "Task/BuffTimer/StartSec"
            },
            config.TaskSettings.BuffTimer.StartSec);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffTimer.EndSec",
                "Task/BuffTimer/EndSec"
            },
            config.TaskSettings.BuffTimer.EndSec);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffTimer.MaxShootCount",
                "Task/BuffTimer/MaxShootCount"
            },
            config.TaskSettings.BuffTimer.MaxShootCount);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffConfirm.RefereeFreshTimeoutMs",
                "Task/BuffConfirm/RefereeFreshTimeoutMs"
            },
            config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffConfirm.PulseMs",
                "Task/BuffConfirm/PulseMs"
            },
            config.TaskSettings.BuffConfirm.PulseMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffConfirm.RetryIntervalMs",
                "Task/BuffConfirm/RetryIntervalMs"
            },
            config.TaskSettings.BuffConfirm.RetryIntervalMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffConfirm.PostConfirmGraceMs",
                "Task/BuffConfirm/PostConfirmGraceMs"
            },
            config.TaskSettings.BuffConfirm.PostConfirmGraceMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffConfirm.TaskHoldTimeoutMs",
                "Task/BuffConfirm/TaskHoldTimeoutMs"
            },
            config.TaskSettings.BuffConfirm.TaskHoldTimeoutMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffConfirm.DamageAbortThreshold",
                "Task/BuffConfirm/DamageAbortThreshold"
            },
            config.TaskSettings.BuffConfirm.DamageAbortThreshold);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffConfirm.DamageAbortWindowMs",
                "Task/BuffConfirm/DamageAbortWindowMs"
            },
            config.TaskSettings.BuffConfirm.DamageAbortWindowMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.BuffConfirm.DamageAbortHoldMs",
                "Task/BuffConfirm/DamageAbortHoldMs"
            },
            config.TaskSettings.BuffConfirm.DamageAbortHoldMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.RefereeFreshTimeoutMs",
                "Task/OutpostConfirm/RefereeFreshTimeoutMs"
            },
            config.TaskSettings.OutpostConfirm.RefereeFreshTimeoutMs);
        ReadOptionalBoolParam(
            node_,
            {
                "Task.OutpostConfirm.TrustEnemyOutpostHp",
                "Task/OutpostConfirm/TrustEnemyOutpostHp"
            },
            config.TaskSettings.OutpostConfirm.TrustEnemyOutpostHp);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.MaxGameTimeSec",
                "Task/OutpostConfirm/MaxGameTimeSec"
            },
            config.TaskSettings.OutpostConfirm.MaxGameTimeSec);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.MinSelfHp",
                "Task/OutpostConfirm/MinSelfHp"
            },
            config.TaskSettings.OutpostConfirm.MinSelfHp);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.MinAmmo",
                "Task/OutpostConfirm/MinAmmo"
            },
            config.TaskSettings.OutpostConfirm.MinAmmo);
        ReadOptionalBoolParam(
            node_,
            {
                "Task.OutpostConfirm.VisualScoutWithoutHp",
                "Task/OutpostConfirm/VisualScoutWithoutHp"
            },
            config.TaskSettings.OutpostConfirm.VisualScoutWithoutHp);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.VisualScoutHoldMs",
                "Task/OutpostConfirm/VisualScoutHoldMs"
            },
            config.TaskSettings.OutpostConfirm.VisualScoutHoldMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.VisualScoutCooldownMs",
                "Task/OutpostConfirm/VisualScoutCooldownMs"
            },
            config.TaskSettings.OutpostConfirm.VisualScoutCooldownMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.VisualScoutFaceDistanceCm",
                "Task/OutpostConfirm/VisualScoutFaceDistanceCm"
            },
            config.TaskSettings.OutpostConfirm.VisualScoutFaceDistanceCm);
        ReadOptionalBoolParam(
            node_,
            {
                "Task.OutpostConfirm.PostWindowScoutEnable",
                "Task/OutpostConfirm/PostWindowScoutEnable"
            },
            config.TaskSettings.OutpostConfirm.PostWindowScoutEnable);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.PostWindowScoutIntervalSec",
                "Task/OutpostConfirm/PostWindowScoutIntervalSec"
            },
            config.TaskSettings.OutpostConfirm.PostWindowScoutIntervalSec);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.PostWindowScoutHoldMs",
                "Task/OutpostConfirm/PostWindowScoutHoldMs"
            },
            config.TaskSettings.OutpostConfirm.PostWindowScoutHoldMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.ArmorInterruptMaxDistanceCm",
                "Task/OutpostConfirm/ArmorInterruptMaxDistanceCm"
            },
            config.TaskSettings.OutpostConfirm.ArmorWarningDistanceCm);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.ArmorWarningDistanceCm",
                "Task/OutpostConfirm/ArmorWarningDistanceCm"
            },
            config.TaskSettings.OutpostConfirm.ArmorWarningDistanceCm);
        config.TaskSettings.OutpostConfirm.ArmorInterruptMaxDistanceCm =
            config.TaskSettings.OutpostConfirm.ArmorWarningDistanceCm;
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.PostArmorFaceSearchMs",
                "Task/OutpostConfirm/PostArmorFaceSearchMs"
            },
            config.TaskSettings.OutpostConfirm.PostArmorFaceSearchMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.DamageAbortThreshold",
                "Task/OutpostConfirm/DamageAbortThreshold"
            },
            config.TaskSettings.OutpostConfirm.DamageAbortThreshold);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.DamageAbortWindowMs",
                "Task/OutpostConfirm/DamageAbortWindowMs"
            },
            config.TaskSettings.OutpostConfirm.DamageAbortWindowMs);
        ReadOptionalIntParam(
            node_,
            {
                "Task.OutpostConfirm.DamageAbortHoldMs",
                "Task/OutpostConfirm/DamageAbortHoldMs"
            },
            config.TaskSettings.OutpostConfirm.DamageAbortHoldMs);
        ReadOptionalBoolParam(
            node_,
            {
                "Task.OutpostConfirm.OpeningHighPriority",
                "Task/OutpostConfirm/OpeningHighPriority"
            },
            config.TaskSettings.OutpostConfirm.OpeningHighPriority);
        ReadOptionalBoolParam(
            node_,
            {
                "Task.OutpostConfirm.OpeningHoldUntilWindowEnd",
                "Task/OutpostConfirm/OpeningHoldUntilWindowEnd"
            },
            config.TaskSettings.OutpostConfirm.OpeningHoldUntilWindowEnd);
        ReadOptionalBoolParam(
            node_,
            {
                "Task.OutpostConfirm.SuppressChaseWhileActive",
                "Task/OutpostConfirm/SuppressChaseWhileActive"
            },
            config.TaskSettings.OutpostConfirm.SuppressChaseWhileActive);
        ReadOptionalBoolParam(
            node_,
            {
                "Task.OutpostConfirm.ManualGoal.Enable",
                "Task/OutpostConfirm/ManualGoal/Enable",
                "Task.OutpostConfirm.ManualGoalEnable",
                "Task/OutpostConfirm/ManualGoalEnable"
            },
            config.TaskSettings.OutpostConfirm.ManualGoalEnable);
        ReadOptionalDoubleParam(
            node_,
            {
                "Task.OutpostConfirm.ManualGoal.MapXM",
                "Task/OutpostConfirm/ManualGoal/MapXM",
                "Task.OutpostConfirm.ManualGoalMapXM",
                "Task/OutpostConfirm/ManualGoalMapXM"
            },
            config.TaskSettings.OutpostConfirm.ManualGoalMapXM);
        ReadOptionalDoubleParam(
            node_,
            {
                "Task.OutpostConfirm.ManualGoal.MapYM",
                "Task/OutpostConfirm/ManualGoal/MapYM",
                "Task.OutpostConfirm.ManualGoalMapYM",
                "Task/OutpostConfirm/ManualGoalMapYM"
            },
            config.TaskSettings.OutpostConfirm.ManualGoalMapYM);
        ReadOptionalDoubleParam(
            node_,
            {
                "Task.OutpostConfirm.ManualGoal.MapZM",
                "Task/OutpostConfirm/ManualGoal/MapZM",
                "Task.OutpostConfirm.ManualGoalMapZM",
                "Task/OutpostConfirm/ManualGoalMapZM"
            },
            config.TaskSettings.OutpostConfirm.ManualGoalMapZM);
    }

    void Application::ApplySpecialParameterOverrides() {
        auto& mini = config.SpecialSettings.MiniRoadland;
        auto& patrol = config.SpecialSettings.Patrol;
        ReadOptionalBoolParam(
            node_,
            {
                "Special.MiniRoadland.Enable",
                "Special/MiniRoadland/Enable",
                "Special.MiniRoadland",
                "Special/MiniRoadland"
            },
            mini.Enable);
        ReadOptionalIntParam(
            node_,
            {
                "Special.MiniRoadland.GoalHoldSec",
                "Special/MiniRoadland/GoalHoldSec"
            },
            mini.GoalHoldSec);
        ReadOptionalIntParam(
            node_,
            {
                "Special.MiniRoadland.SpeedLevel",
                "Special/MiniRoadland/SpeedLevel"
            },
            mini.SpeedLevel);
        int goal_base_id = static_cast<int>(mini.GoalBaseId);
        if (ReadOptionalIntParam(
                node_,
                {
                    "Special.MiniRoadland.GoalBaseId",
                    "Special/MiniRoadland/GoalBaseId"
                },
                goal_base_id)) {
            mini.GoalBaseId = static_cast<std::uint8_t>(std::clamp(goal_base_id, 0, 255));
        }
        ReadOptionalBoolParam(
            node_,
            {
                "Special.Patrol.Enable",
                "Special/Patrol/Enable",
                "Special.Patrol",
                "Special/Patrol"
            },
            patrol.Enable);
        ReadOptionalIntParam(
            node_,
            {
                "Special.Patrol.GoalHoldSec",
                "Special/Patrol/GoalHoldSec"
            },
            patrol.GoalHoldSec);
        ReadOptionalIntParam(
            node_,
            {
                "Special.Patrol.SpeedLevel",
                "Special/Patrol/SpeedLevel"
            },
            patrol.SpeedLevel);
        ReadOptionalBoolParam(
            node_,
            {
                "Special.Patrol.SuppressChase",
                "Special/Patrol/SuppressChase"
            },
            patrol.SuppressChase);
        ReadOptionalBoolParam(
            node_,
            {
                "Special.Patrol.StopOnTarget",
                "Special/Patrol/StopOnTarget"
            },
            patrol.StopOnTarget);
    }

    void Application::ApplyFaceModeParameterOverrides() {
        ReadOptionalBoolParam(
            node_,
            {
                "FaceMode.Enable",
                "FaceMode/Enable"
            },
            config.FaceModeSettings.Enable);
        ReadOptionalIntParam(
            node_,
            {
                "FaceMode.LostTargetHoldMs",
                "FaceMode/LostTargetHoldMs"
            },
            config.FaceModeSettings.LostTargetHoldMs);
        ReadOptionalBoolParam(
            node_,
            {
                "FaceMode.SuppressFire",
                "FaceMode/SuppressFire"
            },
            config.FaceModeSettings.SuppressFire);
        ReadOptionalBoolParam(
            node_,
            {
                "FaceMode.FallbackToPatrolScanMode2",
                "FaceMode/FallbackToPatrolScanMode2"
            },
            config.FaceModeSettings.FallbackToPatrolScanMode2);
        ReadOptionalIntParam(
            node_,
            {
                "FaceMode.FallbackPatrolScanMode",
                "FaceMode/FallbackPatrolScanMode"
            },
            config.FaceModeSettings.FallbackPatrolScanMode);
        ReadOptionalIntParam(
            node_,
            {
                "FaceMode.OutpostFallbackPatrolScanMode",
                "FaceMode/OutpostFallbackPatrolScanMode"
            },
            config.FaceModeSettings.OutpostFallbackPatrolScanMode);
    }

    void Application::ApplyNaviRotateControlParameterOverrides() {
        auto& setting = config.NaviRotateControlSettings;
        ReadOptionalBoolParam(
            node_,
            {
                "NaviRotateControl.Enable",
                "NaviRotateControl/Enable"
            },
            setting.Enable);
        ReadOptionalIntParam(
            node_,
            {
                "NaviRotateControl.FreshTimeoutMs",
                "NaviRotateControl/FreshTimeoutMs"
            },
            setting.FreshTimeoutMs);
        ReadOptionalBoolParam(
            node_,
            {
                "NaviRotateControl.DefaultIsRotate",
                "NaviRotateControl/DefaultIsRotate"
            },
            setting.DefaultIsRotate);
        ReadOptionalBoolParam(
            node_,
            {
                "NaviRotateControl.ForceFollowModeWhenFalse",
                "NaviRotateControl/ForceFollowModeWhenFalse"
            },
            setting.ForceFollowModeWhenFalse);
        ReadOptionalBoolParam(
            node_,
            {
                "NaviRotateControl.ClearFollowModeWhenTrue",
                "NaviRotateControl/ClearFollowModeWhenTrue"
            },
            setting.ClearFollowModeWhenTrue);
        ReadOptionalBoolParam(
            node_,
            {
                "NaviRotateControl.ClearRegionalFaceModeWhenTrue",
                "NaviRotateControl/ClearRegionalFaceModeWhenTrue"
            },
            setting.ClearRegionalFaceModeWhenTrue);
        ReadOptionalBoolParam(
            node_,
            {
                "NaviRotateControl.StopRotateWhenFalse",
                "NaviRotateControl/StopRotateWhenFalse"
            },
            setting.StopRotateWhenFalse);
    }

    void Application::ApplyPointManagerParameterOverrides() {
        auto& setting = config.PointManagerSettings;
        ReadOptionalBoolParam(
            node_,
            {
                "PointManager.Global.Enable",
                "PointManager/Global/Enable"
            },
            setting.Global.Enable);
        int global_rotate = setting.Global.Rotate;
        if (ReadOptionalIntParam(
                node_,
                {
                    "PointManager.Global.Rotate",
                    "PointManager/Global/Rotate"
                },
                global_rotate)) {
            setting.Global.Rotate = ClampRotateGear(global_rotate);
        }

        for (const auto& [point_name, base_goal_id] : PointManagerGoalNameMap()) {
            auto point_setting = setting.Points.count(base_goal_id) != 0
                ? setting.Points.at(base_goal_id)
                : LangYa::PointRotateSetting{};
            bool touched = setting.Points.count(base_goal_id) != 0;
            const std::string name{point_name};
            if (ReadOptionalBoolParam(
                    node_,
                    {
                        "PointManager.Points." + name + ".Enable",
                        "PointManager/Points/" + name + "/Enable",
                        "PointManager." + name + ".Enable",
                        "PointManager/" + name + "/Enable"
                    },
                    point_setting.Enable)) {
                touched = true;
            }
            int rotate = point_setting.Rotate;
            if (ReadOptionalIntParam(
                    node_,
                    {
                        "PointManager.Points." + name + ".Rotate",
                        "PointManager/Points/" + name + "/Rotate",
                        "PointManager." + name + ".Rotate",
                        "PointManager/" + name + "/Rotate"
                    },
                    rotate)) {
                point_setting.Rotate = ClampRotateGear(rotate);
                touched = true;
            }
            if (touched) {
                point_setting.Rotate = ClampRotateGear(point_setting.Rotate);
                setting.Points[base_goal_id] = point_setting;
            }
        }
    }

    void Application::ApplyPatrolScanParameterOverrides() {
        auto& setting = config.PatrolScanSettings;
        ReadOptionalIntParam(
            node_,
            {
                "PatrolScan.Mode",
                "PatrolScan/Mode"
            },
            setting.Mode);

        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode1.YawStepDegPerTick", "PatrolScan/Mode1/YawStepDegPerTick"},
            setting.Mode1YawStepDegPerTick);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode1.YawBoostStepDegPerTick", "PatrolScan/Mode1/YawBoostStepDegPerTick"},
            setting.Mode1YawBoostStepDegPerTick);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode1.PitchCenterDeg", "PatrolScan/Mode1/PitchCenterDeg"},
            setting.Mode1PitchCenterDeg);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode1.PitchHalfRangeDeg", "PatrolScan/Mode1/PitchHalfRangeDeg"},
            setting.Mode1PitchHalfRangeDeg);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode1.PitchPeriodMs", "PatrolScan/Mode1/PitchPeriodMs"},
            setting.Mode1PitchPeriodMs);

        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode2.YawStepDegPerTick", "PatrolScan/Mode2/YawStepDegPerTick"},
            setting.Mode2YawStepDegPerTick);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode2.YawBoostStepDegPerTick", "PatrolScan/Mode2/YawBoostStepDegPerTick"},
            setting.Mode2YawBoostStepDegPerTick);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode2.YawHalfRangeDeg", "PatrolScan/Mode2/YawHalfRangeDeg"},
            setting.Mode2YawHalfRangeDeg);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode2.CenterDriftPerCycleDeg", "PatrolScan/Mode2/CenterDriftPerCycleDeg"},
            setting.Mode2CenterDriftPerCycleDeg);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode2.PitchCenterDeg", "PatrolScan/Mode2/PitchCenterDeg"},
            setting.Mode2PitchCenterDeg);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode2.PitchHalfRangeDeg", "PatrolScan/Mode2/PitchHalfRangeDeg"},
            setting.Mode2PitchHalfRangeDeg);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode2.PitchPeriodMs", "PatrolScan/Mode2/PitchPeriodMs"},
            setting.Mode2PitchPeriodMs);

        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode3.YawStepDegPerTick", "PatrolScan/Mode3/YawStepDegPerTick"},
            setting.Mode3YawStepDegPerTick);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode3.PitchOffsetDeg", "PatrolScan/Mode3/PitchOffsetDeg"},
            setting.Mode3PitchOffsetDeg);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode3.PitchHalfRangeDeg", "PatrolScan/Mode3/PitchHalfRangeDeg"},
            setting.Mode3PitchHalfRangeDeg);
        ReadOptionalDoubleParam(
            node_,
            {"PatrolScan.Mode3.PitchPeriodMs", "PatrolScan/Mode3/PitchPeriodMs"},
            setting.Mode3PitchPeriodMs);
    }

    void Application::ApplyStartGateParameterOverrides() {
        auto& setting = config.StartGateSettings;
        ReadOptionalBoolParam(
            node_,
            {
                "StartGate.AllowGimbalPatrolBeforeStart",
                "StartGate/AllowGimbalPatrolBeforeStart"
            },
            setting.AllowGimbalPatrolBeforeStart);
    }

    void Application::ApplyExternalAimParameterOverrides() {
        auto& setting = config.ExternalAimSettings;
        setting.Enable = true;
        ReadOptionalIntParam(
            node_,
            {
                "ExternalAim.ResultFreshTimeoutMs",
                "ExternalAim/ResultFreshTimeoutMs"
            },
            setting.ResultFreshTimeoutMs);
        ReadOptionalIntParam(
            node_,
            {
                "ExternalAim.TargetFreshTimeoutMs",
                "ExternalAim/TargetFreshTimeoutMs"
            },
            setting.TargetFreshTimeoutMs);
        ReadOptionalBoolParam(
            node_,
            {
                "ExternalAim.UseTargetArrayAsArmorList",
                "ExternalAim/UseTargetArrayAsArmorList"
            },
            setting.UseTargetArrayAsArmorList);
        ReadOptionalBoolParam(
            node_,
            {
                "ExternalAim.PublishSelectTarget",
                "ExternalAim/PublishSelectTarget"
            },
            setting.PublishSelectTarget);
    }

    void Application::ApplyAreaManagerParameterOverrides() {
        auto& navi_goal = config.DecisionAutonomySettings.NaviGoal;
        auto& task = config.RegionalAreaTaskSettings;
        auto& highland = task.MyHighland;
        auto& base = task.MyBase;
        auto& roadland = task.MyRoadland;
        auto& central = task.CommonCentral;
        auto& policy = task.DefaultPolicy;

        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Switch_Point",
                "AreaManager/Switch_Point",
                "AreaManager.SwitchPoint",
                "AreaManager/SwitchPoint",
                "Switch_Point",
                "SwitchPoint"
            },
            config.SwitchPoint);
        Area::SetSwitchPoint(config.SwitchPoint);

        auto& sentry_position_fusion = config.SentryPositionFusionSettings;
        auto fusion_names = [](const std::string& key) {
            return std::vector<std::string>{
                "AreaManager.SentryPositionFusion." + key,
                "AreaManager/SentryPositionFusion/" + key
            };
        };
        auto fusion_source_names = [](const std::vector<std::string>& source_names,
                                      const std::string& key) {
            std::vector<std::string> names;
            names.reserve(source_names.size() * 4U);
            for (const auto& source_name : source_names) {
                names.push_back("AreaManager.SentryPositionFusion.Sources." + source_name + "." + key);
                names.push_back("AreaManager/SentryPositionFusion/Sources/" + source_name + "/" + key);
                names.push_back("AreaManager.SentryPositionFusion." + source_name + "." + key);
                names.push_back("AreaManager/SentryPositionFusion/" + source_name + "/" + key);
            }
            return names;
        };
        auto read_fusion_source = [&](const std::vector<std::string>& source_names,
                                      SentryPositionFusionSourceSetting& source) {
            ReadOptionalBoolParam(node_, fusion_source_names(source_names, "Enable"), source.Enable);
            ReadOptionalIntParam(node_, fusion_source_names(source_names, "Priority"), source.Priority);
            ReadOptionalDoubleParam(node_, fusion_source_names(source_names, "Weight"), source.Weight);
            ReadOptionalIntParam(node_, fusion_source_names(source_names, "FreshTimeoutMs"), source.FreshTimeoutMs);
        };
        ReadOptionalBoolParam(node_, fusion_names("Enable"), sentry_position_fusion.Enable);
        ReadOptionalStringParam(node_, fusion_names("Mode"), sentry_position_fusion.Mode);
        ReadOptionalIntParam(node_, fusion_names("FreshTimeoutMs"), sentry_position_fusion.FreshTimeoutMs);
        read_fusion_source({"Uwb", "UWB"}, sentry_position_fusion.Uwb);
        read_fusion_source({"PositionData"}, sentry_position_fusion.PositionData);
        read_fusion_source({"Navi", "NaviPosition"}, sentry_position_fusion.Navi);

        auto read_area_group = [&](const std::string& group_name,
                                   std::vector<std::string>& area_list,
                                   const std::vector<std::pair<std::string, std::string>>& area_tokens) {
            bool any_configured = false;
            std::vector<std::string> next_area_list;
            next_area_list.reserve(area_tokens.size());
            for (const auto& [display_name, token] : area_tokens) {
                bool enabled =
                    std::find(area_list.begin(), area_list.end(), token) != area_list.end();
                const bool configured = ReadOptionalBoolParam(
                    node_,
                    std::vector<std::string>{
                        "AreaManager.Area." + group_name + "." + display_name + ".Enable",
                        "AreaManager.Area." + group_name + "." + display_name,
                        "AreaManager/Area/" + group_name + "/" + display_name + "/Enable",
                        "AreaManager/Area/" + group_name + "/" + display_name
                    },
                    enabled);
                any_configured = any_configured || configured;
                if (enabled) {
                    next_area_list.push_back(token);
                }
            }
            if (any_configured) {
                area_list = std::move(next_area_list);
            }
        };

        ReadOptionalBoolParam(
            node_,
            {"AreaManager.Area.UseAreaScope", "AreaManager/Area/UseAreaScope"},
            navi_goal.UseAreaScope);
        read_area_group(
            "MyArea",
            navi_goal.MyArea,
            {{"Base", "base"}, {"Highland", "highland"}, {"Roadland", "roadland"}});
        read_area_group(
            "EnemyArea",
            navi_goal.EnemyArea,
            {{"Base", "base"}, {"Highland", "highland"}, {"Roadland", "roadland"}});
        read_area_group(
            "CommonArea",
            navi_goal.CommonArea,
            {{"Central", "central"}});

        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.BuffOutpostCompat.Enable",
                "AreaManager/Area/BuffOutpostCompat/Enable",
                "DecisionAutonomy.NaviGoal.BuffOutpostCompat.Enable",
                "DecisionAutonomy/NaviGoal/BuffOutpostCompat/Enable"
            },
            navi_goal.BuffOutpostCompatEnable);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.BuffOutpostCompat.TimeoutSec",
                "AreaManager/Area/BuffOutpostCompat/TimeoutSec",
                "DecisionAutonomy.NaviGoal.BuffOutpostCompat.TimeoutSec",
                "DecisionAutonomy/NaviGoal/BuffOutpostCompat/TimeoutSec"
            },
            navi_goal.BuffOutpostCompatTimeoutSec);

        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.RegionalAreaTask.IgnoreRecovery",
                "AreaManager/RegionalAreaTask/IgnoreRecovery",
                "AreaManager.Task.IgnoreRecovery",
                "AreaManager/Task/IgnoreRecovery"
            },
            task.IgnoreRecovery);

        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.MyArea.Highland.Task.Enable",
                "AreaManager/Area/MyArea/Highland/Task/Enable",
                "AreaManager.Task.Enable",
                "AreaManager/Task/Enable",
                "AreaManager.RegionalAreaTask.Enable",
                "AreaManager/RegionalAreaTask/Enable"
            },
            task.Enable);
        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.MyArea.Base.Task.Enable",
                "AreaManager/Area/MyArea/Base/Task/Enable"
            },
            base.Enable);
        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.MyArea.Roadland.Task.Enable",
                "AreaManager/Area/MyArea/Roadland/Task/Enable"
            },
            roadland.Enable);
        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.CommonArea.Central.Task.Enable",
                "AreaManager/Area/CommonArea/Central/Task/Enable"
            },
            central.Enable);
        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.MyArea.Highland.Task.MyHighland.Enable",
                "AreaManager/Area/MyArea/Highland/Task/MyHighland/Enable",
                "AreaManager.Task.MyHighland.Enable",
                "AreaManager/Task/MyHighland/Enable",
                "AreaManager.RegionalAreaTask.MyHighland.Enable",
                "AreaManager/RegionalAreaTask/MyHighland/Enable"
            },
            highland.Enable);
        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.MyArea.Highland.Task.MyHighland.UseFaceMode",
                "AreaManager/Area/MyArea/Highland/Task/MyHighland/UseFaceMode",
                "AreaManager.Task.MyHighland.UseFaceMode",
                "AreaManager/Task/MyHighland/UseFaceMode",
                "AreaManager.RegionalAreaTask.MyHighland.UseFaceMode",
                "AreaManager/RegionalAreaTask/MyHighland/UseFaceMode"
            },
            highland.UseFaceMode);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Highland.Task.MyHighland.ApproachTimeoutSec",
                "AreaManager/Area/MyArea/Highland/Task/MyHighland/ApproachTimeoutSec",
                "AreaManager.Task.MyHighland.ApproachTimeoutSec",
                "AreaManager/Task/MyHighland/ApproachTimeoutSec",
                "AreaManager.RegionalAreaTask.MyHighland.ApproachTimeoutSec",
                "AreaManager/RegionalAreaTask/MyHighland/ApproachTimeoutSec"
            },
            highland.ApproachTimeoutSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Highland.Task.MyHighland.HighlandPatrolHoldSec",
                "AreaManager/Area/MyArea/Highland/Task/MyHighland/HighlandPatrolHoldSec",
                "AreaManager.Task.MyHighland.HighlandPatrolHoldSec",
                "AreaManager/Task/MyHighland/HighlandPatrolHoldSec",
                "AreaManager.RegionalAreaTask.MyHighland.HighlandPatrolHoldSec",
                "AreaManager/RegionalAreaTask/MyHighland/HighlandPatrolHoldSec"
            },
            highland.HighlandPatrolHoldSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Highland.Task.MyHighland.BuffShootTravelTimeoutSec",
                "AreaManager/Area/MyArea/Highland/Task/MyHighland/BuffShootTravelTimeoutSec",
                "AreaManager.Task.MyHighland.BuffShootTravelTimeoutSec",
                "AreaManager/Task/MyHighland/BuffShootTravelTimeoutSec",
                "AreaManager.RegionalAreaTask.MyHighland.BuffShootTravelTimeoutSec",
                "AreaManager/RegionalAreaTask/MyHighland/BuffShootTravelTimeoutSec"
            },
            highland.BuffShootTravelTimeoutSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Highland.Task.MyHighland.BuffShootHoldSec",
                "AreaManager/Area/MyArea/Highland/Task/MyHighland/BuffShootHoldSec",
                "AreaManager.Task.MyHighland.BuffShootHoldSec",
                "AreaManager/Task/MyHighland/BuffShootHoldSec",
                "AreaManager.RegionalAreaTask.MyHighland.BuffShootHoldSec",
                "AreaManager/RegionalAreaTask/MyHighland/BuffShootHoldSec"
            },
            highland.BuffShootHoldSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Highland.Task.MyHighland.LeaveTimeoutSec",
                "AreaManager/Area/MyArea/Highland/Task/MyHighland/LeaveTimeoutSec",
                "AreaManager.Task.MyHighland.LeaveTimeoutSec",
                "AreaManager/Task/MyHighland/LeaveTimeoutSec",
                "AreaManager.RegionalAreaTask.MyHighland.LeaveTimeoutSec",
                "AreaManager/RegionalAreaTask/MyHighland/LeaveTimeoutSec"
            },
            highland.LeaveTimeoutSec);
        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.MyArea.Base.Task.MyBase.Enable",
                "AreaManager/Area/MyArea/Base/Task/MyBase/Enable",
                "AreaManager.Task.MyBase.Enable",
                "AreaManager/Task/MyBase/Enable",
                "AreaManager.RegionalAreaTask.MyBase.Enable",
                "AreaManager/RegionalAreaTask/MyBase/Enable"
            },
            base.Enable);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Base.Task.MyBase.TravelTimeoutSec",
                "AreaManager/Area/MyArea/Base/Task/MyBase/TravelTimeoutSec",
                "AreaManager.Task.MyBase.TravelTimeoutSec",
                "AreaManager/Task/MyBase/TravelTimeoutSec",
                "AreaManager.RegionalAreaTask.MyBase.TravelTimeoutSec",
                "AreaManager/RegionalAreaTask/MyBase/TravelTimeoutSec"
            },
            base.TravelTimeoutSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Base.Task.MyBase.CommandHoldSec",
                "AreaManager/Area/MyArea/Base/Task/MyBase/CommandHoldSec",
                "AreaManager.Task.MyBase.CommandHoldSec",
                "AreaManager/Task/MyBase/CommandHoldSec",
                "AreaManager.RegionalAreaTask.MyBase.CommandHoldSec",
                "AreaManager/RegionalAreaTask/MyBase/CommandHoldSec"
            },
            base.CommandHoldSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Base.Task.MyBase.GoalHoldSec",
                "AreaManager/Area/MyArea/Base/Task/MyBase/GoalHoldSec",
                "AreaManager.Task.MyBase.GoalHoldSec",
                "AreaManager/Task/MyBase/GoalHoldSec",
                "AreaManager.RegionalAreaTask.MyBase.GoalHoldSec",
                "AreaManager/RegionalAreaTask/MyBase/GoalHoldSec"
            },
            base.GoalHoldSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Base.Task.MyBase.MaxPatrolSteps",
                "AreaManager/Area/MyArea/Base/Task/MyBase/MaxPatrolSteps",
                "AreaManager.Task.MyBase.MaxPatrolSteps",
                "AreaManager/Task/MyBase/MaxPatrolSteps",
                "AreaManager.RegionalAreaTask.MyBase.MaxPatrolSteps",
                "AreaManager/RegionalAreaTask/MyBase/MaxPatrolSteps"
            },
            base.MaxPatrolSteps);
        ReadOptionalDoubleParam(
            node_,
            {
                "AreaManager.Area.MyArea.Base.Task.MyBase.Patrol.DistancePenaltyPerMeter",
                "AreaManager/Area/MyArea/Base/Task/MyBase/Patrol/DistancePenaltyPerMeter",
                "AreaManager.Task.MyBase.Patrol.DistancePenaltyPerMeter",
                "AreaManager/Task/MyBase/Patrol/DistancePenaltyPerMeter",
                "AreaManager.RegionalAreaTask.MyBase.Patrol.DistancePenaltyPerMeter",
                "AreaManager/RegionalAreaTask/MyBase/Patrol/DistancePenaltyPerMeter"
            },
            base.PatrolDistancePenaltyPerMeter);
        ReadOptionalDoubleParam(
            node_,
            {
                "AreaManager.Area.MyArea.Base.Task.MyBase.Patrol.CurrentGoalPenalty",
                "AreaManager/Area/MyArea/Base/Task/MyBase/Patrol/CurrentGoalPenalty",
                "AreaManager.Task.MyBase.Patrol.CurrentGoalPenalty",
                "AreaManager/Task/MyBase/Patrol/CurrentGoalPenalty",
                "AreaManager.RegionalAreaTask.MyBase.Patrol.CurrentGoalPenalty",
                "AreaManager/RegionalAreaTask/MyBase/Patrol/CurrentGoalPenalty"
            },
            base.PatrolCurrentGoalPenalty);
        for (const auto& [goal_name, goal_id] : MyBasePatrolGoalNameMap()) {
            std::vector<std::string> weight_names;
            const auto append_weight_names = [&weight_names, goal_name](const std::string& prefix) {
                weight_names.push_back(prefix + ".GoalWeights." + goal_name);
                weight_names.push_back(prefix + "/GoalWeights/" + goal_name);
            };
            append_weight_names("AreaManager.Area.MyArea.Base.Task.MyBase.Patrol");
            append_weight_names("AreaManager/Area/MyArea/Base/Task/MyBase/Patrol");
            append_weight_names("AreaManager.Task.MyBase.Patrol");
            append_weight_names("AreaManager/Task/MyBase/Patrol");
            append_weight_names("AreaManager.RegionalAreaTask.MyBase.Patrol");
            append_weight_names("AreaManager/RegionalAreaTask/MyBase/Patrol");
            double weight = 0.0;
            if (ReadOptionalDoubleParam(node_, weight_names, weight)) {
                UpsertMyBasePatrolGoalWeight(base.PatrolGoals, goal_id, weight);
            }
        }
        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.MyArea.Roadland.Task.MyRoadland.Enable",
                "AreaManager/Area/MyArea/Roadland/Task/MyRoadland/Enable",
                "AreaManager.Task.MyRoadland.Enable",
                "AreaManager/Task/MyRoadland/Enable",
                "AreaManager.RegionalAreaTask.MyRoadland.Enable",
                "AreaManager/RegionalAreaTask/MyRoadland/Enable"
            },
            roadland.Enable);
        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.MyArea.Roadland.Task.MyRoadland.UseFaceMode",
                "AreaManager/Area/MyArea/Roadland/Task/MyRoadland/UseFaceMode",
                "AreaManager.Task.MyRoadland.UseFaceMode",
                "AreaManager/Task/MyRoadland/UseFaceMode",
                "AreaManager.RegionalAreaTask.MyRoadland.UseFaceMode",
                "AreaManager/RegionalAreaTask/MyRoadland/UseFaceMode"
            },
            roadland.UseFaceMode);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Roadland.Task.MyRoadland.TravelTimeoutSec",
                "AreaManager/Area/MyArea/Roadland/Task/MyRoadland/TravelTimeoutSec",
                "AreaManager.Task.MyRoadland.TravelTimeoutSec",
                "AreaManager/Task/MyRoadland/TravelTimeoutSec",
                "AreaManager.RegionalAreaTask.MyRoadland.TravelTimeoutSec",
                "AreaManager/RegionalAreaTask/MyRoadland/TravelTimeoutSec"
            },
            roadland.TravelTimeoutSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Roadland.Task.MyRoadland.CrossTimeoutSec",
                "AreaManager/Area/MyArea/Roadland/Task/MyRoadland/CrossTimeoutSec",
                "AreaManager.Task.MyRoadland.CrossTimeoutSec",
                "AreaManager/Task/MyRoadland/CrossTimeoutSec",
                "AreaManager.RegionalAreaTask.MyRoadland.CrossTimeoutSec",
                "AreaManager/RegionalAreaTask/MyRoadland/CrossTimeoutSec"
            },
            roadland.CrossTimeoutSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Roadland.Task.MyRoadland.CommandHoldSec",
                "AreaManager/Area/MyArea/Roadland/Task/MyRoadland/CommandHoldSec",
                "AreaManager.Task.MyRoadland.CommandHoldSec",
                "AreaManager/Task/MyRoadland/CommandHoldSec",
                "AreaManager.RegionalAreaTask.MyRoadland.CommandHoldSec",
                "AreaManager/RegionalAreaTask/MyRoadland/CommandHoldSec"
            },
            roadland.CommandHoldSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Roadland.Task.MyRoadland.GuardHoldSec",
                "AreaManager/Area/MyArea/Roadland/Task/MyRoadland/GuardHoldSec",
                "AreaManager.Task.MyRoadland.GuardHoldSec",
                "AreaManager/Task/MyRoadland/GuardHoldSec",
                "AreaManager.RegionalAreaTask.MyRoadland.GuardHoldSec",
                "AreaManager/RegionalAreaTask/MyRoadland/GuardHoldSec"
            },
            roadland.GuardHoldSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Roadland.Task.MyRoadland.FaceTargetZCm",
                "AreaManager/Area/MyArea/Roadland/Task/MyRoadland/FaceTargetZCm",
                "AreaManager.Task.MyRoadland.FaceTargetZCm",
                "AreaManager/Task/MyRoadland/FaceTargetZCm",
                "AreaManager.RegionalAreaTask.MyRoadland.FaceTargetZCm",
                "AreaManager/RegionalAreaTask/MyRoadland/FaceTargetZCm"
            },
            roadland.FaceTargetZCm);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Roadland.Task.MyRoadland.HealthyHpMin",
                "AreaManager/Area/MyArea/Roadland/Task/MyRoadland/HealthyHpMin",
                "AreaManager.Task.MyRoadland.HealthyHpMin",
                "AreaManager/Task/MyRoadland/HealthyHpMin",
                "AreaManager.RegionalAreaTask.MyRoadland.HealthyHpMin",
                "AreaManager/RegionalAreaTask/MyRoadland/HealthyHpMin"
            },
            roadland.HealthyHpMin);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.MyArea.Roadland.Task.MyRoadland.HealthyAmmoMin",
                "AreaManager/Area/MyArea/Roadland/Task/MyRoadland/HealthyAmmoMin",
                "AreaManager.Task.MyRoadland.HealthyAmmoMin",
                "AreaManager/Task/MyRoadland/HealthyAmmoMin",
                "AreaManager.RegionalAreaTask.MyRoadland.HealthyAmmoMin",
                "AreaManager/RegionalAreaTask/MyRoadland/HealthyAmmoMin"
            },
            roadland.HealthyAmmoMin);
        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.Area.CommonArea.Central.Task.CommonCentral.Enable",
                "AreaManager/Area/CommonArea/Central/Task/CommonCentral/Enable",
                "AreaManager.Task.CommonCentral.Enable",
                "AreaManager/Task/CommonCentral/Enable",
                "AreaManager.RegionalAreaTask.CommonCentral.Enable",
                "AreaManager/RegionalAreaTask/CommonCentral/Enable"
            },
            central.Enable);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.CommonArea.Central.Task.CommonCentral.TravelTimeoutSec",
                "AreaManager/Area/CommonArea/Central/Task/CommonCentral/TravelTimeoutSec",
                "AreaManager.Task.CommonCentral.TravelTimeoutSec",
                "AreaManager/Task/CommonCentral/TravelTimeoutSec",
                "AreaManager.RegionalAreaTask.CommonCentral.TravelTimeoutSec",
                "AreaManager/RegionalAreaTask/CommonCentral/TravelTimeoutSec"
            },
            central.TravelTimeoutSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.CommonArea.Central.Task.CommonCentral.CommandHoldSec",
                "AreaManager/Area/CommonArea/Central/Task/CommonCentral/CommandHoldSec",
                "AreaManager.Task.CommonCentral.CommandHoldSec",
                "AreaManager/Task/CommonCentral/CommandHoldSec",
                "AreaManager.RegionalAreaTask.CommonCentral.CommandHoldSec",
                "AreaManager/RegionalAreaTask/CommonCentral/CommandHoldSec"
            },
            central.CommandHoldSec);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.CommonArea.Central.Task.CommonCentral.MaxPatrolSteps",
                "AreaManager/Area/CommonArea/Central/Task/CommonCentral/MaxPatrolSteps",
                "AreaManager.Task.CommonCentral.MaxPatrolSteps",
                "AreaManager/Task/CommonCentral/MaxPatrolSteps",
                "AreaManager.RegionalAreaTask.CommonCentral.MaxPatrolSteps",
                "AreaManager/RegionalAreaTask/CommonCentral/MaxPatrolSteps"
            },
            central.MaxPatrolSteps);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.CommonArea.Central.Task.CommonCentral.HealthyHpMin",
                "AreaManager/Area/CommonArea/Central/Task/CommonCentral/HealthyHpMin",
                "AreaManager.Task.CommonCentral.HealthyHpMin",
                "AreaManager/Task/CommonCentral/HealthyHpMin",
                "AreaManager.RegionalAreaTask.CommonCentral.HealthyHpMin",
                "AreaManager/RegionalAreaTask/CommonCentral/HealthyHpMin"
            },
            central.HealthyHpMin);
        ReadOptionalIntParam(
            node_,
            {
                "AreaManager.Area.CommonArea.Central.Task.CommonCentral.HealthyAmmoMin",
                "AreaManager/Area/CommonArea/Central/Task/CommonCentral/HealthyAmmoMin",
                "AreaManager.Task.CommonCentral.HealthyAmmoMin",
                "AreaManager/Task/CommonCentral/HealthyAmmoMin",
                "AreaManager.RegionalAreaTask.CommonCentral.HealthyAmmoMin",
                "AreaManager/RegionalAreaTask/CommonCentral/HealthyAmmoMin"
            },
            central.HealthyAmmoMin);

        auto policy_names = [](const std::string& section, const std::string& key) {
            return std::vector<std::string>{
                "AreaManager.DefaultPolicy." + section + "." + key,
                "AreaManager/DefaultPolicy/" + section + "/" + key,
                "AreaManager.RegionalAreaTask.DefaultPolicy." + section + "." + key,
                "AreaManager/RegionalAreaTask/DefaultPolicy/" + section + "/" + key,
                "AreaManager.Task.DefaultPolicy." + section + "." + key,
                "AreaManager/Task/DefaultPolicy/" + section + "/" + key
            };
        };
        ReadOptionalBoolParam(
            node_,
            {
                "AreaManager.DefaultPolicy.Enable",
                "AreaManager/DefaultPolicy/Enable",
                "AreaManager.RegionalAreaTask.DefaultPolicy.Enable",
                "AreaManager/RegionalAreaTask/DefaultPolicy/Enable",
                "AreaManager.Task.DefaultPolicy.Enable",
                "AreaManager/Task/DefaultPolicy/Enable"
            },
            policy.Enable);
        ReadOptionalIntParam(node_, policy_names("Health", "MyAreaHpMin"), policy.Health.MyAreaHpMin);
        ReadOptionalIntParam(node_, policy_names("Health", "CommonCentralHpMin"), policy.Health.CommonCentralHpMin);
        ReadOptionalIntParam(node_, policy_names("Health", "EnemyAreaHpMin"), policy.Health.EnemyAreaHpMin);
        ReadOptionalIntParam(node_, policy_names("Health", "LowResourceFallbackHp"), policy.Health.LowResourceFallbackHp);
        ReadOptionalIntParam(node_, policy_names("Ammo", "MyAreaAmmoMin"), policy.Ammo.MyAreaAmmoMin);
        ReadOptionalIntParam(node_, policy_names("Ammo", "CommonCentralAmmoMin"), policy.Ammo.CommonCentralAmmoMin);
        ReadOptionalIntParam(node_, policy_names("Ammo", "EnemyAreaAmmoMin"), policy.Ammo.EnemyAreaAmmoMin);
        ReadOptionalIntParam(node_, policy_names("Ammo", "LowResourceFallbackAmmo"), policy.Ammo.LowResourceFallbackAmmo);
        ReadOptionalDoubleParam(node_, policy_names("Score", "WeightMyBase"), policy.Score.WeightMyBase);
        ReadOptionalDoubleParam(node_, policy_names("Score", "WeightMyHighland"), policy.Score.WeightMyHighland);
        ReadOptionalDoubleParam(node_, policy_names("Score", "WeightMyRoadland"), policy.Score.WeightMyRoadland);
        ReadOptionalDoubleParam(node_, policy_names("Score", "WeightCommonCentral"), policy.Score.WeightCommonCentral);
        ReadOptionalDoubleParam(node_, policy_names("Score", "WeightEnemyBase"), policy.Score.WeightEnemyBase);
        ReadOptionalDoubleParam(node_, policy_names("Score", "WeightEnemyHighland"), policy.Score.WeightEnemyHighland);
        ReadOptionalDoubleParam(node_, policy_names("Score", "WeightEnemyRoadland"), policy.Score.WeightEnemyRoadland);
        ReadOptionalDoubleParam(node_, policy_names("Score", "DistancePenaltyPerMeter"), policy.Score.DistancePenaltyPerMeter);
        ReadOptionalDoubleParam(node_, policy_names("Score", "CurrentAreaPenalty"), policy.Score.CurrentAreaPenalty);
        ReadOptionalDoubleParam(node_, policy_names("Score", "LastAreaPenalty"), policy.Score.LastAreaPenalty);
        ReadOptionalDoubleParam(node_, policy_names("Score", "AfterHighlandMyBaseBonus"), policy.Score.AfterHighlandMyBaseBonus);
        ReadOptionalDoubleParam(node_, policy_names("Score", "AfterHighlandMyRoadlandBonus"), policy.Score.AfterHighlandMyRoadlandBonus);
        ReadOptionalDoubleParam(node_, policy_names("Score", "LowResourceMyBaseBonus"), policy.Score.LowResourceMyBaseBonus);
        ReadOptionalIntParam(node_, policy_names("Retry", "CompleteCooldownSec"), policy.Retry.CompleteCooldownSec);
        ReadOptionalIntParam(node_, policy_names("Retry", "FailureCooldownSec"), policy.Retry.FailureCooldownSec);
        ReadOptionalIntParam(node_, policy_names("Retry", "UnreachableCooldownSec"), policy.Retry.UnreachableCooldownSec);
        ReadOptionalIntParam(node_, policy_names("Retry", "MaxRetry"), policy.Retry.MaxRetry);
        if (highland.Enable || base.Enable || roadland.Enable || central.Enable) {
            task.Enable = true;
        }
    }

    bool Application::ConfigurationInit() {
        std::ifstream ifs(config_file_);
        if (!ifs.is_open()) {
            LoggerPtr->Error("Failed to open config.json, file location: {}", config_file_);
            return false;
        }
        LoggerPtr->Info("Open config.json, file location: {}", config_file_);

        // 解析 JSON 文件
        json j;
        ifs >> j;
        // 一次性反序列化到 Config，后续再做范围校验与默认回退。
        config = j.get<Config>();
        ApplyTaskParameterOverrides();
        ApplyAreaManagerParameterOverrides();
        ApplySpecialParameterOverrides();
        ApplyStartGateParameterOverrides();
        ApplyNaviRotateControlParameterOverrides();
        ApplyPointManagerParameterOverrides();
        ApplyPatrolScanParameterOverrides();
        ApplyFaceModeParameterOverrides();
        ApplyExternalAimParameterOverrides();
        LoggerPtr->Debug("Switch_Point: {}", config.SwitchPoint);
        LoggerPtr->Debug("------ AimDebug ------");
        LoggerPtr->Debug("StopFire: {}", config.AimDebugSettings.StopFire);
        LoggerPtr->Debug("StopRotate: {}", config.AimDebugSettings.StopRotate);
        LoggerPtr->Debug("StopScan: {}", config.AimDebugSettings.StopScan);
        LoggerPtr->Debug("ForceOutpost: {}", config.AimDebugSettings.ForceOutpost);
        LoggerPtr->Debug("ForceBuff: {}", config.AimDebugSettings.ForceBuff);
        LoggerPtr->Debug("HitCar: {}", config.AimDebugSettings.HitCar);
        LoggerPtr->Debug("FireRequireTargetStatus: {}", config.AimDebugSettings.FireRequireTargetStatus);
        LoggerPtr->Debug("ReuseLatchedAnglesOnNoTarget: {}", config.AimDebugSettings.ReuseLatchedAnglesOnNoTarget);
        LoggerPtr->Debug("LatchedTargetHoldMs: {}", config.AimDebugSettings.LatchedTargetHoldMs);
        LoggerPtr->Debug("------ PatrolScan ------");
        LoggerPtr->Debug("Mode: {}", config.PatrolScanSettings.Mode);
        LoggerPtr->Debug(
            "Mode1: yaw_step={} yaw_boost_step={} pitch_center={} pitch_half_range={} pitch_period_ms={}",
            config.PatrolScanSettings.Mode1YawStepDegPerTick,
            config.PatrolScanSettings.Mode1YawBoostStepDegPerTick,
            config.PatrolScanSettings.Mode1PitchCenterDeg,
            config.PatrolScanSettings.Mode1PitchHalfRangeDeg,
            config.PatrolScanSettings.Mode1PitchPeriodMs);
        LoggerPtr->Debug(
            "Mode2: yaw_step={} yaw_boost_step={} yaw_half_range={} center_drift_per_cycle={} pitch_center={} pitch_half_range={} pitch_period_ms={}",
            config.PatrolScanSettings.Mode2YawStepDegPerTick,
            config.PatrolScanSettings.Mode2YawBoostStepDegPerTick,
            config.PatrolScanSettings.Mode2YawHalfRangeDeg,
            config.PatrolScanSettings.Mode2CenterDriftPerCycleDeg,
            config.PatrolScanSettings.Mode2PitchCenterDeg,
            config.PatrolScanSettings.Mode2PitchHalfRangeDeg,
            config.PatrolScanSettings.Mode2PitchPeriodMs);
        LoggerPtr->Debug(
            "Mode3: yaw_step={} pitch_offset={} pitch_half_range={} pitch_period_ms={}",
            config.PatrolScanSettings.Mode3YawStepDegPerTick,
            config.PatrolScanSettings.Mode3PitchOffsetDeg,
            config.PatrolScanSettings.Mode3PitchHalfRangeDeg,
            config.PatrolScanSettings.Mode3PitchPeriodMs);
        LoggerPtr->Debug("------ Rate ------");
        LoggerPtr->Debug("FireRate: {}", config.RateSettings.FireRate);
        LoggerPtr->Debug("TickRate: {}", config.RateSettings.TreeTickRate);
        LoggerPtr->Debug("NaviCommandRate: {}", config.RateSettings.NaviCommandRate);
        LoggerPtr->Debug("ScanCounter: {}", config.ScanCounter);
        LoggerPtr->Debug("------ StartGate ------");
        LoggerPtr->Debug(
            "AllowGimbalPatrolBeforeStart: {}",
            config.StartGateSettings.AllowGimbalPatrolBeforeStart);
        LoggerPtr->Debug("------ Task ------");
        LoggerPtr->Debug("Buff: {}", config.TaskSettings.Buff);
        LoggerPtr->Debug("Outpost: {}", config.TaskSettings.Outpost);
        LoggerPtr->Debug(
            "BuffTimer: enable={} start={} end={} max_shoot={}",
            config.TaskSettings.BuffTimer.Enable,
            config.TaskSettings.BuffTimer.StartSec,
            config.TaskSettings.BuffTimer.EndSec,
            config.TaskSettings.BuffTimer.MaxShootCount);
        LoggerPtr->Debug(
            "BuffConfirm: referee_fresh_ms={} pulse_ms={} retry_ms={} grace_ms={} hold_timeout_ms={} damage_abort_threshold={} damage_abort_window_ms={} damage_abort_hold_ms={}",
            config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs,
            config.TaskSettings.BuffConfirm.PulseMs,
            config.TaskSettings.BuffConfirm.RetryIntervalMs,
            config.TaskSettings.BuffConfirm.PostConfirmGraceMs,
            config.TaskSettings.BuffConfirm.TaskHoldTimeoutMs,
            config.TaskSettings.BuffConfirm.DamageAbortThreshold,
            config.TaskSettings.BuffConfirm.DamageAbortWindowMs,
            config.TaskSettings.BuffConfirm.DamageAbortHoldMs);
        LoggerPtr->Debug(
            "OutpostConfirm: referee_fresh_ms={} trust_enemy_outpost_hp={} max_game_time_sec={} min_self_hp={} min_ammo={} visual_scout_without_hp={} visual_scout_hold_ms={} visual_scout_cooldown_ms={} visual_scout_face_distance_cm={} post_window_scout_enable={} post_window_scout_interval_sec={} post_window_scout_hold_ms={} armor_warning_distance_cm={} post_armor_face_search_ms={} damage_abort_threshold={} damage_abort_window_ms={} damage_abort_hold_ms={} opening_high_priority={} opening_hold_until_window_end={} suppress_chase_while_active={}",
            config.TaskSettings.OutpostConfirm.RefereeFreshTimeoutMs,
            config.TaskSettings.OutpostConfirm.TrustEnemyOutpostHp ? 1 : 0,
            config.TaskSettings.OutpostConfirm.MaxGameTimeSec,
            config.TaskSettings.OutpostConfirm.MinSelfHp,
            config.TaskSettings.OutpostConfirm.MinAmmo,
            config.TaskSettings.OutpostConfirm.VisualScoutWithoutHp ? 1 : 0,
            config.TaskSettings.OutpostConfirm.VisualScoutHoldMs,
            config.TaskSettings.OutpostConfirm.VisualScoutCooldownMs,
            config.TaskSettings.OutpostConfirm.VisualScoutFaceDistanceCm,
            config.TaskSettings.OutpostConfirm.PostWindowScoutEnable ? 1 : 0,
            config.TaskSettings.OutpostConfirm.PostWindowScoutIntervalSec,
            config.TaskSettings.OutpostConfirm.PostWindowScoutHoldMs,
            config.TaskSettings.OutpostConfirm.ArmorWarningDistanceCm,
            config.TaskSettings.OutpostConfirm.PostArmorFaceSearchMs,
            config.TaskSettings.OutpostConfirm.DamageAbortThreshold,
            config.TaskSettings.OutpostConfirm.DamageAbortWindowMs,
            config.TaskSettings.OutpostConfirm.DamageAbortHoldMs,
            config.TaskSettings.OutpostConfirm.OpeningHighPriority ? 1 : 0,
            config.TaskSettings.OutpostConfirm.OpeningHoldUntilWindowEnd ? 1 : 0,
            config.TaskSettings.OutpostConfirm.SuppressChaseWhileActive ? 1 : 0);
        LoggerPtr->Debug(
            "OutpostManualGoal: enable={} map=({:.3f}, {:.3f}, {:.3f}) m",
            config.TaskSettings.OutpostConfirm.ManualGoalEnable ? 1 : 0,
            config.TaskSettings.OutpostConfirm.ManualGoalMapXM,
            config.TaskSettings.OutpostConfirm.ManualGoalMapYM,
            config.TaskSettings.OutpostConfirm.ManualGoalMapZM);
        LoggerPtr->Debug("------ DamageOpenGate ------");
        LoggerPtr->Debug("Enable: {}", config.DamageOpenGateSettings.Enable);
        LoggerPtr->Debug("HealthDropThreshold: {}", config.DamageOpenGateSettings.HealthDropThreshold);
        LoggerPtr->Debug("------ NaviSetting ------");
        LoggerPtr->Debug("UseXY: {}", config.NaviSettings.UseXY);
        LoggerPtr->Debug("Navi.ToNavi: {}", config.NaviSettings.ToNavi);
        LoggerPtr->Debug("------ FaceMode ------");
        LoggerPtr->Debug("Enable: {}", config.FaceModeSettings.Enable);
        LoggerPtr->Debug("LostTargetHoldMs: {}", config.FaceModeSettings.LostTargetHoldMs);
        LoggerPtr->Debug("SuppressFire: {}", config.FaceModeSettings.SuppressFire);
        LoggerPtr->Debug("FallbackToPatrolScanMode2: {}", config.FaceModeSettings.FallbackToPatrolScanMode2);
        LoggerPtr->Debug("FallbackPatrolScanMode: {}", config.FaceModeSettings.FallbackPatrolScanMode);
        LoggerPtr->Debug("OutpostFallbackPatrolScanMode: {}", config.FaceModeSettings.OutpostFallbackPatrolScanMode);
        LoggerPtr->Debug("------ ExternalAim ------");
        LoggerPtr->Debug("Enable: {}", config.ExternalAimSettings.Enable);
        LoggerPtr->Debug("ResultFreshTimeoutMs: {}", config.ExternalAimSettings.ResultFreshTimeoutMs);
        LoggerPtr->Debug("TargetFreshTimeoutMs: {}", config.ExternalAimSettings.TargetFreshTimeoutMs);
        LoggerPtr->Debug("UseTargetArrayAsArmorList: {}", config.ExternalAimSettings.UseTargetArrayAsArmorList);
        LoggerPtr->Debug("PublishSelectTarget: {}", config.ExternalAimSettings.PublishSelectTarget);
        LoggerPtr->Debug("TargetDefaultFrame: {}", config.ExternalAimSettings.TargetDefaultFrame);
        LoggerPtr->Debug("------ NaviRotateControl ------");
        LoggerPtr->Debug("Enable: {}", config.NaviRotateControlSettings.Enable);
        LoggerPtr->Debug("FreshTimeoutMs: {}", config.NaviRotateControlSettings.FreshTimeoutMs);
        LoggerPtr->Debug("DefaultIsRotate: {}", config.NaviRotateControlSettings.DefaultIsRotate);
        LoggerPtr->Debug("ForceFollowModeWhenFalse: {}", config.NaviRotateControlSettings.ForceFollowModeWhenFalse);
        LoggerPtr->Debug("ClearFollowModeWhenTrue: {}", config.NaviRotateControlSettings.ClearFollowModeWhenTrue);
        LoggerPtr->Debug("ClearRegionalFaceModeWhenTrue: {}", config.NaviRotateControlSettings.ClearRegionalFaceModeWhenTrue);
        LoggerPtr->Debug("StopRotateWhenFalse: {}", config.NaviRotateControlSettings.StopRotateWhenFalse);
        LoggerPtr->Debug("------ PointManager ------");
        LoggerPtr->Debug(
            "Global.Enable: {}",
            config.PointManagerSettings.Global.Enable);
        LoggerPtr->Debug(
            "Global.Rotate: {}",
            config.PointManagerSettings.Global.Rotate);
        for (const auto& [point_name, base_goal_id] : PointManagerGoalNameMap()) {
            const auto it = config.PointManagerSettings.Points.find(base_goal_id);
            if (it == config.PointManagerSettings.Points.end()) {
                continue;
            }
            LoggerPtr->Debug(
                "{}: enable={} rotate={}",
                point_name,
                it->second.Enable,
                it->second.Rotate);
        }
        LoggerPtr->Debug("------ SentryPositionFusion ------");
        LoggerPtr->Debug("Enable: {}", config.SentryPositionFusionSettings.Enable);
        LoggerPtr->Debug("Mode: {}", config.SentryPositionFusionSettings.Mode);
        LoggerPtr->Debug("FreshTimeoutMs: {}", config.SentryPositionFusionSettings.FreshTimeoutMs);
        LoggerPtr->Debug(
            "Uwb: Enable={} Priority={} Weight={} FreshTimeoutMs={}",
            config.SentryPositionFusionSettings.Uwb.Enable,
            config.SentryPositionFusionSettings.Uwb.Priority,
            config.SentryPositionFusionSettings.Uwb.Weight,
            config.SentryPositionFusionSettings.Uwb.FreshTimeoutMs);
        LoggerPtr->Debug(
            "PositionData: Enable={} Priority={} Weight={} FreshTimeoutMs={}",
            config.SentryPositionFusionSettings.PositionData.Enable,
            config.SentryPositionFusionSettings.PositionData.Priority,
            config.SentryPositionFusionSettings.PositionData.Weight,
            config.SentryPositionFusionSettings.PositionData.FreshTimeoutMs);
        LoggerPtr->Debug(
            "Navi: Enable={} Priority={} Weight={} FreshTimeoutMs={}",
            config.SentryPositionFusionSettings.Navi.Enable,
            config.SentryPositionFusionSettings.Navi.Priority,
            config.SentryPositionFusionSettings.Navi.Weight,
            config.SentryPositionFusionSettings.Navi.FreshTimeoutMs);
        LoggerPtr->Debug("------ LeagueStrategy ------");
        LoggerPtr->Debug("EnableRouteCompat: {}", config.LeagueStrategySettings.EnableRouteCompat);
        LoggerPtr->Debug("UseHealthRecovery: {}", config.LeagueStrategySettings.UseHealthRecovery);
        LoggerPtr->Debug("HealthRecoveryThreshold: {}", config.LeagueStrategySettings.HealthRecoveryThreshold);
        LoggerPtr->Debug("UseAmmoRecovery: {}", config.LeagueStrategySettings.UseAmmoRecovery);
        LoggerPtr->Debug("AmmoRecoveryThreshold: {}", config.LeagueStrategySettings.AmmoRecoveryThreshold);
        LoggerPtr->Debug("DamageScanBoostEnable: {}", config.LeagueStrategySettings.DamageScanBoostEnable);
        LoggerPtr->Debug("HealthRecoveryExitMin: {}", config.LeagueStrategySettings.HealthRecoveryExitMin);
        LoggerPtr->Debug("HealthRecoveryExitPreferred: {}", config.LeagueStrategySettings.HealthRecoveryExitPreferred);
        LoggerPtr->Debug("HealthRecoveryPlateauSec: {}", config.LeagueStrategySettings.HealthRecoveryPlateauSec);
        LoggerPtr->Debug("HealthRecoveryExitStableSec: {}", config.LeagueStrategySettings.HealthRecoveryExitStableSec);
        LoggerPtr->Debug("HealthRecoveryMaxHoldSec: {}", config.LeagueStrategySettings.HealthRecoveryMaxHoldSec);
        LoggerPtr->Debug("HealthRecoveryCooldownSec: {}", config.LeagueStrategySettings.HealthRecoveryCooldownSec);
        LoggerPtr->Debug("MainGoal: {}", static_cast<int>(config.LeagueStrategySettings.MainGoal));
        LoggerPtr->Debug("GoalHoldSec: {}", config.LeagueStrategySettings.GoalHoldSec);
        LoggerPtr->Debug("------ ShowcasePatrol ------");
        LoggerPtr->Debug("Enable: {}", config.ShowcasePatrolSettings.Enable);
        LoggerPtr->Debug("GoalHoldSec: {}", config.ShowcasePatrolSettings.GoalHoldSec);
        LoggerPtr->Debug("Random: {}", config.ShowcasePatrolSettings.Random);
        LoggerPtr->Debug("DisableTeamOffset: {}", config.ShowcasePatrolSettings.DisableTeamOffset);
        LoggerPtr->Debug("IgnoreRecovery: {}", config.ShowcasePatrolSettings.IgnoreRecovery);
        LoggerPtr->Debug("------ NaviDebug ------");
        LoggerPtr->Debug("Enable: {}", config.NaviDebugSettings.Enable);
        LoggerPtr->Debug("PlanFile: {}", config.NaviDebugSettings.PlanFile);
        LoggerPtr->Debug("ActivePlan: {}", config.NaviDebugSettings.ActivePlan);
        LoggerPtr->Debug("------ RegionalDefense ------");
        LoggerPtr->Debug("Enable: {}", config.RegionalDefenseSettings.Enable);
        LoggerPtr->Debug("EnableSoftEnemySideThreat: {}", config.RegionalDefenseSettings.EnableSoftEnemySideThreat);
        LoggerPtr->Debug("EnemyPositionFreshMs: {}", config.RegionalDefenseSettings.EnemyPositionFreshMs);
        LoggerPtr->Debug("HardHoldSec: {}", config.RegionalDefenseSettings.HardHoldSec);
        LoggerPtr->Debug("SoftHoldSec: {}", config.RegionalDefenseSettings.SoftHoldSec);
        LoggerPtr->Debug("SearchHoldSec: {}", config.RegionalDefenseSettings.SearchHoldSec);
        LoggerPtr->Debug("SearchNoTargetSec: {}", config.RegionalDefenseSettings.SearchNoTargetSec);
        LoggerPtr->Debug("FortressStandEnemyCountMin: {}", config.RegionalDefenseSettings.FortressStandEnemyCountMin);
        LoggerPtr->Debug("FortressNoContactDegradeSec: {}", config.RegionalDefenseSettings.FortressNoContactDegradeSec);
        LoggerPtr->Debug("FortressDegradeCooldownSec: {}", config.RegionalDefenseSettings.FortressDegradeCooldownSec);
        LoggerPtr->Debug("StrongHealthMin: {}", config.RegionalDefenseSettings.StrongHealthMin);
        LoggerPtr->Debug("StrongAmmoMin: {}", config.RegionalDefenseSettings.StrongAmmoMin);
        LoggerPtr->Debug("MultiEnemyBaseCount: {}", config.RegionalDefenseSettings.MultiEnemyBaseCount);
        LoggerPtr->Debug("------ HeroProtection ------");
        LoggerPtr->Debug("Enable: {}", config.HeroProtectionSettings.Enable);
        LoggerPtr->Debug("StartElapsedSec: {}", config.HeroProtectionSettings.StartElapsedSec);
        LoggerPtr->Debug("HoldSec: {}", config.HeroProtectionSettings.HoldSec);
        LoggerPtr->Debug("NoEnemyReleaseSec: {}", config.HeroProtectionSettings.NoEnemyReleaseSec);
        LoggerPtr->Debug("FriendPositionFreshMs: {}", config.HeroProtectionSettings.FriendPositionFreshMs);
        LoggerPtr->Debug("FriendHealthFreshMs: {}", config.HeroProtectionSettings.FriendHealthFreshMs);
        LoggerPtr->Debug("GoalBaseId: {}", static_cast<int>(config.HeroProtectionSettings.GoalBaseId));
        LoggerPtr->Debug("------ NaviProgressWatchdog ------");
        LoggerPtr->Debug("Enable: {}", config.NaviProgressWatchdogSettings.Enable);
        LoggerPtr->Debug("ArriveDistanceCm: {}", config.NaviProgressWatchdogSettings.ArriveDistanceCm);
        LoggerPtr->Debug("MoveProgressCm: {}", config.NaviProgressWatchdogSettings.MoveProgressCm);
        LoggerPtr->Debug("NoMoveTimeoutSec: {}", config.NaviProgressWatchdogSettings.NoMoveTimeoutSec);
        LoggerPtr->Debug("FallbackHoldSec: {}", config.NaviProgressWatchdogSettings.FallbackHoldSec);
        LoggerPtr->Debug("FallbackCooldownSec: {}", config.NaviProgressWatchdogSettings.FallbackCooldownSec);
        LoggerPtr->Debug("------ RegionalIdlePatrol ------");
        LoggerPtr->Debug("Enable: {}", config.RegionalIdlePatrolSettings.Enable);
        LoggerPtr->Debug("GoalHoldSec: {}", config.RegionalIdlePatrolSettings.GoalHoldSec);
        for (const auto goal_id : config.RegionalIdlePatrolSettings.Goals) {
            LoggerPtr->Debug("Goal: {}", static_cast<int>(goal_id));
        }
        LoggerPtr->Debug("------ Special ------");
        LoggerPtr->Debug("MiniRoadland.Enable: {}", config.SpecialSettings.MiniRoadland.Enable);
        LoggerPtr->Debug("MiniRoadland.GoalHoldSec: {}", config.SpecialSettings.MiniRoadland.GoalHoldSec);
        LoggerPtr->Debug("MiniRoadland.GoalBaseId: {}", static_cast<int>(config.SpecialSettings.MiniRoadland.GoalBaseId));
        LoggerPtr->Debug("MiniRoadland.SpeedLevel: {}", config.SpecialSettings.MiniRoadland.SpeedLevel);
        LoggerPtr->Debug("Patrol.Enable: {}", config.SpecialSettings.Patrol.Enable);
        LoggerPtr->Debug("Patrol.GoalHoldSec: {}", config.SpecialSettings.Patrol.GoalHoldSec);
        LoggerPtr->Debug("Patrol.SpeedLevel: {}", config.SpecialSettings.Patrol.SpeedLevel);
        LoggerPtr->Debug("Patrol.SuppressChase: {}", config.SpecialSettings.Patrol.SuppressChase);
        LoggerPtr->Debug("Patrol.StopOnTarget: {}", config.SpecialSettings.Patrol.StopOnTarget);
        LoggerPtr->Debug("------ RegionalAreaTask ------");
        LoggerPtr->Debug("Enable: {}", config.RegionalAreaTaskSettings.Enable);
        LoggerPtr->Debug("IgnoreRecovery: {}", config.RegionalAreaTaskSettings.IgnoreRecovery);
        LoggerPtr->Debug("MyHighland.Enable: {}", config.RegionalAreaTaskSettings.MyHighland.Enable);
        LoggerPtr->Debug("MyHighland.UseFaceMode: {}", config.RegionalAreaTaskSettings.MyHighland.UseFaceMode);
        LoggerPtr->Debug("MyHighland.ApproachTimeoutSec: {}", config.RegionalAreaTaskSettings.MyHighland.ApproachTimeoutSec);
        LoggerPtr->Debug("MyHighland.HighlandPatrolHoldSec: {}", config.RegionalAreaTaskSettings.MyHighland.HighlandPatrolHoldSec);
        LoggerPtr->Debug("MyHighland.BuffShootTravelTimeoutSec: {}", config.RegionalAreaTaskSettings.MyHighland.BuffShootTravelTimeoutSec);
        LoggerPtr->Debug("MyHighland.BuffShootHoldSec: {}", config.RegionalAreaTaskSettings.MyHighland.BuffShootHoldSec);
        LoggerPtr->Debug("MyHighland.LeaveTimeoutSec: {}", config.RegionalAreaTaskSettings.MyHighland.LeaveTimeoutSec);
        LoggerPtr->Debug("MyBase.Enable: {}", config.RegionalAreaTaskSettings.MyBase.Enable);
        LoggerPtr->Debug("MyBase.TravelTimeoutSec: {}", config.RegionalAreaTaskSettings.MyBase.TravelTimeoutSec);
        LoggerPtr->Debug("MyBase.CommandHoldSec: {}", config.RegionalAreaTaskSettings.MyBase.CommandHoldSec);
        LoggerPtr->Debug("MyBase.GoalHoldSec: {}", config.RegionalAreaTaskSettings.MyBase.GoalHoldSec);
        LoggerPtr->Debug("MyBase.MaxPatrolSteps: {}", config.RegionalAreaTaskSettings.MyBase.MaxPatrolSteps);
        LoggerPtr->Debug("MyBase.PatrolDistancePenaltyPerMeter: {}", config.RegionalAreaTaskSettings.MyBase.PatrolDistancePenaltyPerMeter);
        LoggerPtr->Debug("MyBase.PatrolCurrentGoalPenalty: {}", config.RegionalAreaTaskSettings.MyBase.PatrolCurrentGoalPenalty);
        for (const auto& goal : config.RegionalAreaTaskSettings.MyBase.PatrolGoals) {
            LoggerPtr->Debug(
                "MyBase.PatrolGoal: id={} weight={}",
                static_cast<int>(goal.BaseGoalId),
                goal.Weight);
        }
        LoggerPtr->Debug("MyRoadland.Enable: {}", config.RegionalAreaTaskSettings.MyRoadland.Enable);
        LoggerPtr->Debug("MyRoadland.UseFaceMode: {}", config.RegionalAreaTaskSettings.MyRoadland.UseFaceMode);
        LoggerPtr->Debug("MyRoadland.TravelTimeoutSec: {}", config.RegionalAreaTaskSettings.MyRoadland.TravelTimeoutSec);
        LoggerPtr->Debug("MyRoadland.CrossTimeoutSec: {}", config.RegionalAreaTaskSettings.MyRoadland.CrossTimeoutSec);
        LoggerPtr->Debug("MyRoadland.CommandHoldSec: {}", config.RegionalAreaTaskSettings.MyRoadland.CommandHoldSec);
        LoggerPtr->Debug("MyRoadland.GuardHoldSec: {}", config.RegionalAreaTaskSettings.MyRoadland.GuardHoldSec);
        LoggerPtr->Debug("MyRoadland.FaceTargetZCm: {}", config.RegionalAreaTaskSettings.MyRoadland.FaceTargetZCm);
        LoggerPtr->Debug("MyRoadland.HealthyHpMin: {}", config.RegionalAreaTaskSettings.MyRoadland.HealthyHpMin);
        LoggerPtr->Debug("MyRoadland.HealthyAmmoMin: {}", config.RegionalAreaTaskSettings.MyRoadland.HealthyAmmoMin);
        LoggerPtr->Debug("CommonCentral.Enable: {}", config.RegionalAreaTaskSettings.CommonCentral.Enable);
        LoggerPtr->Debug("CommonCentral.TravelTimeoutSec: {}", config.RegionalAreaTaskSettings.CommonCentral.TravelTimeoutSec);
        LoggerPtr->Debug("CommonCentral.CommandHoldSec: {}", config.RegionalAreaTaskSettings.CommonCentral.CommandHoldSec);
        LoggerPtr->Debug("CommonCentral.MaxPatrolSteps: {}", config.RegionalAreaTaskSettings.CommonCentral.MaxPatrolSteps);
        LoggerPtr->Debug("CommonCentral.HealthyHpMin: {}", config.RegionalAreaTaskSettings.CommonCentral.HealthyHpMin);
        LoggerPtr->Debug("CommonCentral.HealthyAmmoMin: {}", config.RegionalAreaTaskSettings.CommonCentral.HealthyAmmoMin);
        LoggerPtr->Debug("DefaultPolicy.Enable: {}", config.RegionalAreaTaskSettings.DefaultPolicy.Enable);
        LoggerPtr->Debug("DefaultPolicy.Health: my={} common={} enemy={} fallback={}",
            config.RegionalAreaTaskSettings.DefaultPolicy.Health.MyAreaHpMin,
            config.RegionalAreaTaskSettings.DefaultPolicy.Health.CommonCentralHpMin,
            config.RegionalAreaTaskSettings.DefaultPolicy.Health.EnemyAreaHpMin,
            config.RegionalAreaTaskSettings.DefaultPolicy.Health.LowResourceFallbackHp);
        LoggerPtr->Debug("DefaultPolicy.Ammo: my={} common={} enemy={} fallback={}",
            config.RegionalAreaTaskSettings.DefaultPolicy.Ammo.MyAreaAmmoMin,
            config.RegionalAreaTaskSettings.DefaultPolicy.Ammo.CommonCentralAmmoMin,
            config.RegionalAreaTaskSettings.DefaultPolicy.Ammo.EnemyAreaAmmoMin,
            config.RegionalAreaTaskSettings.DefaultPolicy.Ammo.LowResourceFallbackAmmo);
        LoggerPtr->Debug("------ AimTargetPriority ------");
        for (const auto armor_id : config.AimTargetPriority) {
            LoggerPtr->Debug("ArmorTypeId: {}", armor_id);
        }
        LoggerPtr->Debug("------ AimTargetIgnore ------");
        for (const auto armor_id : config.AimTargetIgnore) {
            LoggerPtr->Debug("ArmorTypeId: {}", armor_id);
        }
        LoggerPtr->Debug("------ DecisionAutonomy ------");
        LoggerPtr->Debug("Enable: {}", config.DecisionAutonomySettings.Enable);
        LoggerPtr->Debug("EnabledModules:");
        for (const auto& module : config.DecisionAutonomySettings.EnabledModules) {
            LoggerPtr->Debug("  {}", module);
        }
        LoggerPtr->Debug("HardRuleModules:");
        for (const auto& module : config.DecisionAutonomySettings.HardRuleModules) {
            LoggerPtr->Debug("  {}", module);
        }
        LoggerPtr->Debug("NaviGoal.UseAreaScope: {}", config.DecisionAutonomySettings.NaviGoal.UseAreaScope);
        LoggerPtr->Debug("NaviGoal.MyArea:");
        for (const auto& area : config.DecisionAutonomySettings.NaviGoal.MyArea) {
            LoggerPtr->Debug("  {}", area);
        }
        LoggerPtr->Debug("NaviGoal.EnemyArea:");
        for (const auto& area : config.DecisionAutonomySettings.NaviGoal.EnemyArea) {
            LoggerPtr->Debug("  {}", area);
        }
        LoggerPtr->Debug("NaviGoal.CommonArea:");
        for (const auto& area : config.DecisionAutonomySettings.NaviGoal.CommonArea) {
            LoggerPtr->Debug("  {}", area);
        }
        LoggerPtr->Debug(
            "NaviGoal.HighlandCompat(enable/disable_rotate/arrive_cm/timeout_s/distance_fallback_grace_ms): {}/{}/{}/{}/{}",
            config.DecisionAutonomySettings.NaviGoal.HighlandCompatEnable,
            config.DecisionAutonomySettings.NaviGoal.HighlandCompatDisableRotate,
            config.DecisionAutonomySettings.NaviGoal.HighlandCompatArriveDistanceCm,
            config.DecisionAutonomySettings.NaviGoal.HighlandCompatTimeoutSec,
            config.DecisionAutonomySettings.NaviGoal.DistanceFallbackGraceMs);
        LoggerPtr->Debug(
            "NaviGoal.BuffOutpostCompat(enable/timeout_s): {}/{}",
            config.DecisionAutonomySettings.NaviGoal.BuffOutpostCompatEnable,
            config.DecisionAutonomySettings.NaviGoal.BuffOutpostCompatTimeoutSec);
        LoggerPtr->Debug(
            "AimTarget(enable, weights priority/distance/low_health/current_target, hold_ms/switch_ms/health_fresh_ms/dead_confirm_ms/dead_hold_ms/respawn_transition_ms/invuln_sec/sentry_invuln_sec): {}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{}",
            config.DecisionAutonomySettings.AimTarget.Enable,
            config.DecisionAutonomySettings.AimTarget.PriorityWeight,
            config.DecisionAutonomySettings.AimTarget.DistanceWeight,
            config.DecisionAutonomySettings.AimTarget.LowHealthWeight,
            config.DecisionAutonomySettings.AimTarget.CurrentTargetBonus,
            config.DecisionAutonomySettings.AimTarget.LostTargetHoldMs,
            config.DecisionAutonomySettings.AimTarget.MinSwitchIntervalMs,
            config.DecisionAutonomySettings.AimTarget.HealthFreshTimeoutMs,
            config.DecisionAutonomySettings.AimTarget.DeadHealthConfirmMs,
            config.DecisionAutonomySettings.AimTarget.DeadHealthHoldMs,
            config.DecisionAutonomySettings.AimTarget.RespawnTransitionTimeoutMs,
            config.DecisionAutonomySettings.AimTarget.RespawnInvulnerableSec,
            config.DecisionAutonomySettings.AimTarget.SentryRespawnInvulnerableSec);
        LoggerPtr->Debug("------ Chase ------");
        LoggerPtr->Debug("Enable: {}", config.ChaseSettings.Enable);
        LoggerPtr->Debug("FollowAimTarget: {}", config.ChaseSettings.FollowAimTarget);
        LoggerPtr->Debug("Chase.ToNavi: {}", config.ChaseSettings.ToNavi);
        LoggerPtr->Debug("UseOfficialPositionSource: {}", config.ChaseSettings.UseOfficialPositionSource);
        LoggerPtr->Debug("PreferOfficialPositionSource: {}", config.ChaseSettings.PreferOfficialPositionSource);
        LoggerPtr->Debug("OfficialPositionFreshMs: {}", config.ChaseSettings.OfficialPositionFreshMs);
        LoggerPtr->Debug("EnableInAutoAim: {}", config.ChaseSettings.EnableInAutoAim);
        LoggerPtr->Debug("EnableInRotateScan: {}", config.ChaseSettings.EnableInRotateScan);
        LoggerPtr->Debug("EnableInOutpostMode: {}", config.ChaseSettings.EnableInOutpostMode);
        LoggerPtr->Debug("EnableInBuffMode: {}", config.ChaseSettings.EnableInBuffMode);
        LoggerPtr->Debug("StopWhenNoTarget: {}", config.ChaseSettings.StopWhenNoTarget);
        LoggerPtr->Debug("LostTargetHoldMs: {}", config.ChaseSettings.LostTargetHoldMs);
        LoggerPtr->Debug("PreferredDistanceCm: {}", config.ChaseSettings.PreferredDistanceCm);
        LoggerPtr->Debug("DistanceDeadbandCm: {}", config.ChaseSettings.DistanceDeadbandCm);
        LoggerPtr->Debug(
            "AreaLimit: Enable={} BoundaryMarginCm={} ChaseEnableCrossArea={} HoldWhenNoIntersection={}",
            config.ChaseSettings.AreaLimit.Enable,
            config.ChaseSettings.AreaLimit.BoundaryMarginCm,
            config.ChaseSettings.AreaLimit.ChaseEnableCrossArea,
            config.ChaseSettings.AreaLimit.HoldWhenNoIntersection);
        LoggerPtr->Debug("MinValidDistanceCm: {}", config.ChaseSettings.MinValidDistanceCm);
        LoggerPtr->Debug("MaxValidDistanceCm: {}", config.ChaseSettings.MaxValidDistanceCm);
        LoggerPtr->Debug("DistanceKp: {}", config.ChaseSettings.DistanceKp);
        LoggerPtr->Debug("MaxForwardSpeed: {}", config.ChaseSettings.MaxForwardSpeed);
        LoggerPtr->Debug("MaxBackwardSpeed: {}", config.ChaseSettings.MaxBackwardSpeed);
        LoggerPtr->Debug("UseYawStrafe: {}", config.ChaseSettings.UseYawStrafe);
        LoggerPtr->Debug("YawKp: {}", config.ChaseSettings.YawKp);
        LoggerPtr->Debug("YawDeadbandDeg: {}", config.ChaseSettings.YawDeadbandDeg);
        LoggerPtr->Debug("MaxStrafeSpeed: {}", config.ChaseSettings.MaxStrafeSpeed);
        LoggerPtr->Debug("InvertStrafeDirection: {}", config.ChaseSettings.InvertStrafeDirection);
        LoggerPtr->Debug("------ PostureSetting ------");
        LoggerPtr->Debug("Enable: {}", config.PostureSettings.Enable);
        LoggerPtr->Debug("SwitchCooldownSec: {}", config.PostureSettings.SwitchCooldownSec);
        LoggerPtr->Debug("MaxSinglePostureSec: {}", config.PostureSettings.MaxSinglePostureSec);
        LoggerPtr->Debug("EarlyRotateSec: {}", config.PostureSettings.EarlyRotateSec);
        LoggerPtr->Debug("MinHoldSec: {}", config.PostureSettings.MinHoldSec);
        LoggerPtr->Debug("PendingAckTimeoutMs: {}", config.PostureSettings.PendingAckTimeoutMs);
        LoggerPtr->Debug("RetryIntervalMs: {}", config.PostureSettings.RetryIntervalMs);
        LoggerPtr->Debug("MaxRetryCount: {}", config.PostureSettings.MaxRetryCount);
        LoggerPtr->Debug("OptimisticAck: {}", config.PostureSettings.OptimisticAck);
        LoggerPtr->Debug("TargetKeepMs: {}", config.PostureSettings.TargetKeepMs);
        LoggerPtr->Debug("DamageKeepSec: {}", config.PostureSettings.DamageKeepSec);
        LoggerPtr->Debug("DamageBurstWindowMs: {}", config.PostureSettings.DamageBurstWindowMs);
        LoggerPtr->Debug("DamageBurstThreshold: {}", config.PostureSettings.DamageBurstThreshold);
        LoggerPtr->Debug("DamageBurstDefenseHoldSec: {}", config.PostureSettings.DamageBurstDefenseHoldSec);
        LoggerPtr->Debug("LowHealthThreshold: {}", config.PostureSettings.LowHealthThreshold);
        LoggerPtr->Debug("VeryLowHealthThreshold: {}", config.PostureSettings.VeryLowHealthThreshold);
        LoggerPtr->Debug("LowAmmoThreshold: {}", config.PostureSettings.LowAmmoThreshold);
        LoggerPtr->Debug("ScoreHysteresis: {}", config.PostureSettings.ScoreHysteresis);
        LoggerPtr->Debug("------ End ------");
        LoggerPtr->Debug("Configuration completed.");
        fireRateClock.reset(config.RateSettings.FireRate);
        treeTickRateClock.reset(config.RateSettings.TreeTickRate);
        naviCommandRateClock.reset(config.RateSettings.NaviCommandRate);

        // 关键参数防御式校验：避免配置错误把系统带入不可控状态。
        if (config.LeagueStrategySettings.GoalHoldSec <= 0) {
            LoggerPtr->Warning("Invalid LeagueStrategy.GoalHoldSec={}, fallback to 15.", config.LeagueStrategySettings.GoalHoldSec);
            config.LeagueStrategySettings.GoalHoldSec = 15;
        }
        if (config.LeagueStrategySettings.HealthRecoveryExitMin > 400) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryExitMin={}, clamp to 400.",
                config.LeagueStrategySettings.HealthRecoveryExitMin);
            config.LeagueStrategySettings.HealthRecoveryExitMin = 400;
        }
        if (config.LeagueStrategySettings.HealthRecoveryExitPreferred > 400) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryExitPreferred={}, clamp to 400.",
                config.LeagueStrategySettings.HealthRecoveryExitPreferred);
            config.LeagueStrategySettings.HealthRecoveryExitPreferred = 400;
        }
        if (config.LeagueStrategySettings.HealthRecoveryExitPreferred <
            config.LeagueStrategySettings.HealthRecoveryExitMin) {
            LoggerPtr->Warning(
                "LeagueStrategy.HealthRecoveryExitPreferred({}) < ExitMin({}), align preferred to min.",
                config.LeagueStrategySettings.HealthRecoveryExitPreferred,
                config.LeagueStrategySettings.HealthRecoveryExitMin);
            config.LeagueStrategySettings.HealthRecoveryExitPreferred =
                config.LeagueStrategySettings.HealthRecoveryExitMin;
        }
        if (config.LeagueStrategySettings.HealthRecoveryPlateauSec <= 0) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryPlateauSec={}, fallback to 2.",
                config.LeagueStrategySettings.HealthRecoveryPlateauSec);
            config.LeagueStrategySettings.HealthRecoveryPlateauSec = 2;
        }
        if (config.LeagueStrategySettings.HealthRecoveryExitStableSec <= 0) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryExitStableSec={}, fallback to 1.",
                config.LeagueStrategySettings.HealthRecoveryExitStableSec);
            config.LeagueStrategySettings.HealthRecoveryExitStableSec = 1;
        }
        if (config.LeagueStrategySettings.HealthRecoveryMaxHoldSec <= 0) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryMaxHoldSec={}, fallback to 12.",
                config.LeagueStrategySettings.HealthRecoveryMaxHoldSec);
            config.LeagueStrategySettings.HealthRecoveryMaxHoldSec = 12;
        }
        if (config.LeagueStrategySettings.HealthRecoveryCooldownSec < 0) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryCooldownSec={}, fallback to 0.",
                config.LeagueStrategySettings.HealthRecoveryCooldownSec);
            config.LeagueStrategySettings.HealthRecoveryCooldownSec = 0;
        }
        if (config.DamageOpenGateSettings.HealthDropThreshold == 0) {
            LoggerPtr->Warning(
                "Invalid DamageOpenGate.HealthDropThreshold=0, fallback to 30.");
            config.DamageOpenGateSettings.HealthDropThreshold = 30;
        }
        if (config.DamageOpenGateSettings.HealthDropThreshold > 400) {
            LoggerPtr->Warning(
                "Invalid DamageOpenGate.HealthDropThreshold={}, clamp to 400.",
                config.DamageOpenGateSettings.HealthDropThreshold);
            config.DamageOpenGateSettings.HealthDropThreshold = 400;
        }
        auto& outpost_confirm = config.TaskSettings.OutpostConfirm;
        if (outpost_confirm.RefereeFreshTimeoutMs <= 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.RefereeFreshTimeoutMs={}, fallback to 2000.",
                outpost_confirm.RefereeFreshTimeoutMs);
            outpost_confirm.RefereeFreshTimeoutMs = 2000;
        }
        if (outpost_confirm.MaxGameTimeSec < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.MaxGameTimeSec={}, fallback to 120.",
                outpost_confirm.MaxGameTimeSec);
            outpost_confirm.MaxGameTimeSec = 120;
        }
        if (outpost_confirm.MinSelfHp < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.MinSelfHp={}, fallback to 150.",
                outpost_confirm.MinSelfHp);
            outpost_confirm.MinSelfHp = 150;
        }
        if (outpost_confirm.MinAmmo < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.MinAmmo={}, fallback to 30.",
                outpost_confirm.MinAmmo);
            outpost_confirm.MinAmmo = 30;
        }
        if (outpost_confirm.VisualScoutHoldMs < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.VisualScoutHoldMs={}, fallback to 10000.",
                outpost_confirm.VisualScoutHoldMs);
            outpost_confirm.VisualScoutHoldMs = 10000;
        }
        if (outpost_confirm.VisualScoutCooldownMs < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.VisualScoutCooldownMs={}, fallback to 15000.",
                outpost_confirm.VisualScoutCooldownMs);
            outpost_confirm.VisualScoutCooldownMs = 15000;
        }
        if (outpost_confirm.VisualScoutFaceDistanceCm < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.VisualScoutFaceDistanceCm={}, fallback to 300.",
                outpost_confirm.VisualScoutFaceDistanceCm);
            outpost_confirm.VisualScoutFaceDistanceCm = 300;
        }
        if (outpost_confirm.PostWindowScoutIntervalSec < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.PostWindowScoutIntervalSec={}, fallback to 60.",
                outpost_confirm.PostWindowScoutIntervalSec);
            outpost_confirm.PostWindowScoutIntervalSec = 60;
        }
        if (outpost_confirm.PostWindowScoutHoldMs < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.PostWindowScoutHoldMs={}, fallback to 5000.",
                outpost_confirm.PostWindowScoutHoldMs);
            outpost_confirm.PostWindowScoutHoldMs = 5000;
        }
        if (outpost_confirm.ArmorWarningDistanceCm < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.ArmorWarningDistanceCm={}, fallback to 1000.",
                outpost_confirm.ArmorWarningDistanceCm);
            outpost_confirm.ArmorWarningDistanceCm = 1000;
        }
        outpost_confirm.ArmorInterruptMaxDistanceCm = outpost_confirm.ArmorWarningDistanceCm;
        if (outpost_confirm.PostArmorFaceSearchMs < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.PostArmorFaceSearchMs={}, fallback to 5000.",
                outpost_confirm.PostArmorFaceSearchMs);
            outpost_confirm.PostArmorFaceSearchMs = 5000;
        }
        if (outpost_confirm.DamageAbortThreshold < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.DamageAbortThreshold={}, fallback to 30.",
                outpost_confirm.DamageAbortThreshold);
            outpost_confirm.DamageAbortThreshold = 30;
        }
        if (outpost_confirm.DamageAbortWindowMs < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.DamageAbortWindowMs={}, fallback to 1000.",
                outpost_confirm.DamageAbortWindowMs);
            outpost_confirm.DamageAbortWindowMs = 1000;
        }
        if (outpost_confirm.DamageAbortHoldMs < 0) {
            LoggerPtr->Warning(
                "Invalid Task.OutpostConfirm.DamageAbortHoldMs={}, fallback to 3000.",
                outpost_confirm.DamageAbortHoldMs);
            outpost_confirm.DamageAbortHoldMs = 3000;
        }
        auto sanitize_manual_goal_m = [this](const char* key, double& value) {
            if (!std::isfinite(value)) {
                LoggerPtr->Warning("Invalid {}={}, fallback to 0.", key, value);
                value = 0.0;
            }
        };
        sanitize_manual_goal_m(
            "Task.OutpostConfirm.ManualGoal.MapXM",
            outpost_confirm.ManualGoalMapXM);
        sanitize_manual_goal_m(
            "Task.OutpostConfirm.ManualGoal.MapYM",
            outpost_confirm.ManualGoalMapYM);
        sanitize_manual_goal_m(
            "Task.OutpostConfirm.ManualGoal.MapZM",
            outpost_confirm.ManualGoalMapZM);
        if (config.FaceModeSettings.LostTargetHoldMs < 0) {
            LoggerPtr->Warning(
                "Invalid FaceMode.LostTargetHoldMs={}, fallback to 300.",
                config.FaceModeSettings.LostTargetHoldMs);
            config.FaceModeSettings.LostTargetHoldMs = 300;
        }
        if (config.ExternalAimSettings.ResultFreshTimeoutMs <= 0) {
            LoggerPtr->Warning(
                "Invalid ExternalAim.ResultFreshTimeoutMs={}, fallback to 300.",
                config.ExternalAimSettings.ResultFreshTimeoutMs);
            config.ExternalAimSettings.ResultFreshTimeoutMs = 300;
        }
        if (config.ExternalAimSettings.TargetFreshTimeoutMs <= 0) {
            LoggerPtr->Warning(
                "Invalid ExternalAim.TargetFreshTimeoutMs={}, fallback to 500.",
                config.ExternalAimSettings.TargetFreshTimeoutMs);
            config.ExternalAimSettings.TargetFreshTimeoutMs = 500;
        }
        if (config.ExternalAimSettings.TargetDefaultFrame.empty()) {
            LoggerPtr->Warning("ExternalAim.TargetDefaultFrame is empty, fallback to gimbal_world.");
            config.ExternalAimSettings.TargetDefaultFrame = "gimbal_world";
        }
        if (config.NaviRotateControlSettings.FreshTimeoutMs <= 0) {
            LoggerPtr->Warning(
                "Invalid NaviRotateControl.FreshTimeoutMs={}, fallback to 500.",
                config.NaviRotateControlSettings.FreshTimeoutMs);
            config.NaviRotateControlSettings.FreshTimeoutMs = 500;
        }
        config.PointManagerSettings.Global.Rotate =
            ClampRotateGear(config.PointManagerSettings.Global.Rotate);
        for (auto& [base_goal_id, point_setting] : config.PointManagerSettings.Points) {
            point_setting.Rotate = ClampRotateGear(point_setting.Rotate);
        }
        auto& sentry_position_fusion = config.SentryPositionFusionSettings;
        if (sentry_position_fusion.FreshTimeoutMs <= 0) {
            LoggerPtr->Warning(
                "Invalid SentryPositionFusion.FreshTimeoutMs={}, fallback to 2000.",
                sentry_position_fusion.FreshTimeoutMs);
            sentry_position_fusion.FreshTimeoutMs = 2000;
        }
        const auto fusion_mode = NormalizeAutonomyToken(sentry_position_fusion.Mode);
        if (fusion_mode != "priority" &&
            fusion_mode != "weighted" &&
            fusion_mode != "weight" &&
            fusion_mode != "weighted_fit") {
            LoggerPtr->Warning(
                "Invalid SentryPositionFusion.Mode='{}', fallback to priority.",
                sentry_position_fusion.Mode);
            sentry_position_fusion.Mode = "priority";
        }
        auto sanitize_fusion_source = [&](const char* name,
                                          SentryPositionFusionSourceSetting& source) {
            if (source.Weight < 0.0) {
                LoggerPtr->Warning(
                    "Invalid SentryPositionFusion.{}.Weight={}, fallback to 0.",
                    name,
                    source.Weight);
                source.Weight = 0.0;
            }
            if (source.FreshTimeoutMs < 0) {
                LoggerPtr->Warning(
                    "Invalid SentryPositionFusion.{}.FreshTimeoutMs={}, use global FreshTimeoutMs.",
                    name,
                    source.FreshTimeoutMs);
                source.FreshTimeoutMs = 0;
            }
        };
        sanitize_fusion_source("Uwb", sentry_position_fusion.Uwb);
        sanitize_fusion_source("PositionData", sentry_position_fusion.PositionData);
        sanitize_fusion_source("Navi", sentry_position_fusion.Navi);
        if (!IsValidBaseGoal(config.LeagueStrategySettings.MainGoal)) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.MainGoal={}, fallback to OccupyArea.",
                static_cast<int>(config.LeagueStrategySettings.MainGoal));
            config.LeagueStrategySettings.MainGoal = LangYa::OccupyArea.ID;
        }
        std::vector<std::uint8_t> sanitized_patrol_goals;
        sanitized_patrol_goals.reserve(config.LeagueStrategySettings.PatrolGoals.size());
        for (const auto goal_id : config.LeagueStrategySettings.PatrolGoals) {
            if (!IsValidBaseGoal(goal_id)) {
                LoggerPtr->Warning("Ignore invalid LeagueStrategy.PatrolGoals item={}.", static_cast<int>(goal_id));
                continue;
            }
            if (std::find(sanitized_patrol_goals.begin(), sanitized_patrol_goals.end(), goal_id) == sanitized_patrol_goals.end()) {
                sanitized_patrol_goals.push_back(goal_id);
            }
        }
        config.LeagueStrategySettings.PatrolGoals = std::move(sanitized_patrol_goals);

        if (config.ShowcasePatrolSettings.GoalHoldSec <= 0) {
            LoggerPtr->Warning(
                "Invalid ShowcasePatrol.GoalHoldSec={}, fallback to 5.",
                config.ShowcasePatrolSettings.GoalHoldSec);
            config.ShowcasePatrolSettings.GoalHoldSec = 5;
        }
        std::vector<std::uint8_t> sanitized_showcase_goals;
        sanitized_showcase_goals.reserve(config.ShowcasePatrolSettings.Goals.size());
        for (const auto goal_id : config.ShowcasePatrolSettings.Goals) {
            if (!IsValidBaseGoal(goal_id)) {
                LoggerPtr->Warning("Ignore invalid ShowcasePatrol.Goals item={}.", static_cast<int>(goal_id));
                continue;
            }
            if (std::find(sanitized_showcase_goals.begin(), sanitized_showcase_goals.end(), goal_id) ==
                sanitized_showcase_goals.end()) {
                sanitized_showcase_goals.push_back(goal_id);
            }
        }
        config.ShowcasePatrolSettings.Goals = std::move(sanitized_showcase_goals);
        if (config.ShowcasePatrolSettings.Enable && config.ShowcasePatrolSettings.Goals.empty()) {
            LoggerPtr->Warning("ShowcasePatrol enabled but no valid goals found, fallback to OccupyArea.");
            config.ShowcasePatrolSettings.Goals.push_back(LangYa::OccupyArea.ID);
        }

        if (config.NaviDebugSettings.Enable) {
            if (!LoadNaviDebugPlanFile(config.NaviDebugSettings, LoggerPtr)) {
                LoggerPtr->Warning("Disable NaviDebug and fallback to legacy TestNavi route.");
                config.NaviDebugSettings.Enable = false;
            }
        }
        if (config.NaviDebugSettings.GoalHoldSec <= 0) {
            LoggerPtr->Warning(
                "Invalid NaviDebug.GoalHoldSec={}, fallback to 5.",
                config.NaviDebugSettings.GoalHoldSec);
            config.NaviDebugSettings.GoalHoldSec = 5;
        }
        std::vector<std::uint8_t> sanitized_navi_debug_goals;
        sanitized_navi_debug_goals.reserve(config.NaviDebugSettings.Goals.size());
        for (const auto goal_id : config.NaviDebugSettings.Goals) {
            if (!IsValidBaseGoal(goal_id)) {
                LoggerPtr->Warning("Ignore invalid NaviDebug.Goals item={}.", static_cast<int>(goal_id));
                continue;
            }
            if (std::find(sanitized_navi_debug_goals.begin(), sanitized_navi_debug_goals.end(), goal_id) ==
                sanitized_navi_debug_goals.end()) {
                sanitized_navi_debug_goals.push_back(goal_id);
            }
        }
        config.NaviDebugSettings.Goals = std::move(sanitized_navi_debug_goals);
        if (config.NaviDebugSettings.Enable && config.NaviDebugSettings.Goals.empty()) {
            LoggerPtr->Warning("NaviDebug enabled but no valid goals found, fallback to OccupyArea.");
            config.NaviDebugSettings.Goals.push_back(LangYa::OccupyArea.ID);
        }

        if (config.RegionalDefenseSettings.EnemyPositionFreshMs <= 0) {
            LoggerPtr->Warning("Invalid RegionalDefense.EnemyPositionFreshMs={}, fallback to 2500.",
                               config.RegionalDefenseSettings.EnemyPositionFreshMs);
            config.RegionalDefenseSettings.EnemyPositionFreshMs = 2500;
        }
        if (config.RegionalDefenseSettings.HardHoldSec <= 0) {
            LoggerPtr->Warning("Invalid RegionalDefense.HardHoldSec={}, fallback to 5.",
                               config.RegionalDefenseSettings.HardHoldSec);
            config.RegionalDefenseSettings.HardHoldSec = 5;
        }
        if (config.RegionalDefenseSettings.SoftHoldSec <= 0) {
            LoggerPtr->Warning("Invalid RegionalDefense.SoftHoldSec={}, fallback to 8.",
                               config.RegionalDefenseSettings.SoftHoldSec);
            config.RegionalDefenseSettings.SoftHoldSec = 8;
        }
        if (config.RegionalDefenseSettings.SearchHoldSec <= 0) {
            LoggerPtr->Warning("Invalid RegionalDefense.SearchHoldSec={}, fallback to 4.",
                               config.RegionalDefenseSettings.SearchHoldSec);
            config.RegionalDefenseSettings.SearchHoldSec = 4;
        }
        if (config.RegionalDefenseSettings.SearchNoTargetSec <= 0) {
            LoggerPtr->Warning("Invalid RegionalDefense.SearchNoTargetSec={}, fallback to 4.",
                               config.RegionalDefenseSettings.SearchNoTargetSec);
            config.RegionalDefenseSettings.SearchNoTargetSec = 4;
        }
        if (config.RegionalDefenseSettings.FortressStandEnemyCountMin <= 0) {
            LoggerPtr->Warning("Invalid RegionalDefense.FortressStandEnemyCountMin={}, fallback to 2.",
                               config.RegionalDefenseSettings.FortressStandEnemyCountMin);
            config.RegionalDefenseSettings.FortressStandEnemyCountMin = 2;
        }
        if (config.RegionalDefenseSettings.FortressNoContactDegradeSec <= 0) {
            LoggerPtr->Warning("Invalid RegionalDefense.FortressNoContactDegradeSec={}, fallback to 8.",
                               config.RegionalDefenseSettings.FortressNoContactDegradeSec);
            config.RegionalDefenseSettings.FortressNoContactDegradeSec = 8;
        }
        if (config.RegionalDefenseSettings.FortressDegradeCooldownSec <= 0) {
            LoggerPtr->Warning("Invalid RegionalDefense.FortressDegradeCooldownSec={}, fallback to 6.",
                               config.RegionalDefenseSettings.FortressDegradeCooldownSec);
            config.RegionalDefenseSettings.FortressDegradeCooldownSec = 6;
        }
        if (config.RegionalDefenseSettings.MultiEnemyBaseCount <= 0) {
            LoggerPtr->Warning("Invalid RegionalDefense.MultiEnemyBaseCount={}, fallback to 2.",
                               config.RegionalDefenseSettings.MultiEnemyBaseCount);
            config.RegionalDefenseSettings.MultiEnemyBaseCount = 2;
        }
        if (config.HeroProtectionSettings.StartElapsedSec < 0) {
            LoggerPtr->Warning("Invalid HeroProtection.StartElapsedSec={}, fallback to 120.",
                               config.HeroProtectionSettings.StartElapsedSec);
            config.HeroProtectionSettings.StartElapsedSec = 120;
        }
        if (config.HeroProtectionSettings.HoldSec <= 0) {
            LoggerPtr->Warning("Invalid HeroProtection.HoldSec={}, fallback to 30.",
                               config.HeroProtectionSettings.HoldSec);
            config.HeroProtectionSettings.HoldSec = 30;
        }
        if (config.HeroProtectionSettings.NoEnemyReleaseSec <= 0) {
            LoggerPtr->Warning("Invalid HeroProtection.NoEnemyReleaseSec={}, fallback to 8.",
                               config.HeroProtectionSettings.NoEnemyReleaseSec);
            config.HeroProtectionSettings.NoEnemyReleaseSec = 8;
        }
        if (config.HeroProtectionSettings.FriendPositionFreshMs <= 0) {
            LoggerPtr->Warning("Invalid HeroProtection.FriendPositionFreshMs={}, fallback to 2500.",
                               config.HeroProtectionSettings.FriendPositionFreshMs);
            config.HeroProtectionSettings.FriendPositionFreshMs = 2500;
        }
        if (config.HeroProtectionSettings.FriendHealthFreshMs <= 0) {
            LoggerPtr->Warning("Invalid HeroProtection.FriendHealthFreshMs={}, fallback to 2500.",
                               config.HeroProtectionSettings.FriendHealthFreshMs);
            config.HeroProtectionSettings.FriendHealthFreshMs = 2500;
        }
        if (!AreaManager::IsValidBaseGoalId(config.HeroProtectionSettings.GoalBaseId)) {
            LoggerPtr->Warning("Invalid HeroProtection.GoalBaseId={}, fallback to Highland.",
                               static_cast<int>(config.HeroProtectionSettings.GoalBaseId));
            config.HeroProtectionSettings.GoalBaseId = LangYa::Highland.ID;
        }
        if (config.NaviProgressWatchdogSettings.ArriveDistanceCm <= 0) {
            LoggerPtr->Warning("Invalid NaviProgressWatchdog.ArriveDistanceCm={}, fallback to 140.",
                               config.NaviProgressWatchdogSettings.ArriveDistanceCm);
            config.NaviProgressWatchdogSettings.ArriveDistanceCm = 140;
        }
        if (config.NaviProgressWatchdogSettings.MoveProgressCm <= 0) {
            LoggerPtr->Warning("Invalid NaviProgressWatchdog.MoveProgressCm={}, fallback to 80.",
                               config.NaviProgressWatchdogSettings.MoveProgressCm);
            config.NaviProgressWatchdogSettings.MoveProgressCm = 80;
        }
        if (config.NaviProgressWatchdogSettings.NoMoveTimeoutSec <= 0) {
            LoggerPtr->Warning("Invalid NaviProgressWatchdog.NoMoveTimeoutSec={}, fallback to 14.",
                               config.NaviProgressWatchdogSettings.NoMoveTimeoutSec);
            config.NaviProgressWatchdogSettings.NoMoveTimeoutSec = 14;
        }
        if (config.NaviProgressWatchdogSettings.FallbackHoldSec <= 0) {
            LoggerPtr->Warning("Invalid NaviProgressWatchdog.FallbackHoldSec={}, fallback to 5.",
                               config.NaviProgressWatchdogSettings.FallbackHoldSec);
            config.NaviProgressWatchdogSettings.FallbackHoldSec = 5;
        }
        if (config.NaviProgressWatchdogSettings.FallbackCooldownSec < 0) {
            LoggerPtr->Warning("Invalid NaviProgressWatchdog.FallbackCooldownSec={}, fallback to 0.",
                               config.NaviProgressWatchdogSettings.FallbackCooldownSec);
            config.NaviProgressWatchdogSettings.FallbackCooldownSec = 0;
        }
        if (config.RegionalIdlePatrolSettings.GoalHoldSec <= 0) {
            LoggerPtr->Warning("Invalid RegionalIdlePatrol.GoalHoldSec={}, fallback to 8.",
                               config.RegionalIdlePatrolSettings.GoalHoldSec);
            config.RegionalIdlePatrolSettings.GoalHoldSec = 8;
        }
        std::vector<std::uint8_t> sanitized_regional_idle_goals;
        sanitized_regional_idle_goals.reserve(config.RegionalIdlePatrolSettings.Goals.size());
        for (const auto goal_id : config.RegionalIdlePatrolSettings.Goals) {
            if (!IsValidBaseGoal(goal_id)) {
                LoggerPtr->Warning("Ignore invalid RegionalIdlePatrol.Goals item={}.", static_cast<int>(goal_id));
                continue;
            }
            if (std::find(sanitized_regional_idle_goals.begin(), sanitized_regional_idle_goals.end(), goal_id) ==
                sanitized_regional_idle_goals.end()) {
                sanitized_regional_idle_goals.push_back(goal_id);
            }
        }
        config.RegionalIdlePatrolSettings.Goals = std::move(sanitized_regional_idle_goals);
        if (config.RegionalIdlePatrolSettings.Enable &&
            config.RegionalIdlePatrolSettings.Goals.empty() &&
            !config.RegionalIdlePatrolSettings.GoalEnableProvided) {
            LoggerPtr->Warning("RegionalIdlePatrol enabled but no valid goals found, fallback to default patrol route.");
            config.RegionalIdlePatrolSettings.Goals = {
                LangYa::HoleRoad.ID,
                LangYa::Castle.ID,
                LangYa::CastleRight2.ID,
                LangYa::CastleRight1.ID,
                LangYa::CastleLeft1.ID,
                LangYa::CastleLeft2.ID
            };
        } else if (config.RegionalIdlePatrolSettings.Enable &&
                   config.RegionalIdlePatrolSettings.Goals.empty()) {
            LoggerPtr->Warning("RegionalIdlePatrol enabled but all GoalEnable entries are false or invalid.");
        }

        auto& mini_roadland = config.SpecialSettings.MiniRoadland;
        if (mini_roadland.GoalHoldSec <= 0) {
            LoggerPtr->Warning("Invalid Special.MiniRoadland.GoalHoldSec={}, fallback to 6.",
                               mini_roadland.GoalHoldSec);
            mini_roadland.GoalHoldSec = 6;
        }
        if (mini_roadland.GoalBaseId != LangYa::MiniRoadland.ID) {
            LoggerPtr->Warning("Invalid Special.MiniRoadland.GoalBaseId={}, fallback to MiniRoadland.",
                               static_cast<int>(mini_roadland.GoalBaseId));
            mini_roadland.GoalBaseId = LangYa::MiniRoadland.ID;
        }
        if (mini_roadland.SpeedLevel < 0) {
            LoggerPtr->Warning("Invalid Special.MiniRoadland.SpeedLevel={}, fallback to 1.",
                               mini_roadland.SpeedLevel);
            mini_roadland.SpeedLevel = 1;
        }
        if (mini_roadland.SpeedLevel > 255) {
            LoggerPtr->Warning("Invalid Special.MiniRoadland.SpeedLevel={}, clamp to 255.",
                               mini_roadland.SpeedLevel);
            mini_roadland.SpeedLevel = 255;
        }
        auto& special_patrol = config.SpecialSettings.Patrol;
        if (special_patrol.GoalHoldSec < 0) {
            LoggerPtr->Warning("Invalid Special.Patrol.GoalHoldSec={}, fallback to 0.",
                               special_patrol.GoalHoldSec);
            special_patrol.GoalHoldSec = 0;
        }
        if (special_patrol.SpeedLevel < 0) {
            LoggerPtr->Warning("Invalid Special.Patrol.SpeedLevel={}, fallback to 1.",
                               special_patrol.SpeedLevel);
            special_patrol.SpeedLevel = 1;
        }
        if (special_patrol.SpeedLevel > 255) {
            LoggerPtr->Warning("Invalid Special.Patrol.SpeedLevel={}, clamp to 255.",
                               special_patrol.SpeedLevel);
            special_patrol.SpeedLevel = 255;
        }

        auto& highland_task = config.RegionalAreaTaskSettings.MyHighland;
        if (highland_task.ApproachTimeoutSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyHighland.ApproachTimeoutSec={}, fallback to 8.",
                highland_task.ApproachTimeoutSec);
            highland_task.ApproachTimeoutSec = 8;
        }
        if (highland_task.HighlandPatrolHoldSec < 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyHighland.HighlandPatrolHoldSec={}, fallback to 2.",
                highland_task.HighlandPatrolHoldSec);
            highland_task.HighlandPatrolHoldSec = 2;
        }
        if (highland_task.BuffShootTravelTimeoutSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyHighland.BuffShootTravelTimeoutSec={}, fallback to 8.",
                highland_task.BuffShootTravelTimeoutSec);
            highland_task.BuffShootTravelTimeoutSec = 8;
        }
        if (highland_task.BuffShootHoldSec < 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyHighland.BuffShootHoldSec={}, fallback to 10.",
                highland_task.BuffShootHoldSec);
            highland_task.BuffShootHoldSec = 10;
        }
        if (highland_task.LeaveTimeoutSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyHighland.LeaveTimeoutSec={}, fallback to 8.",
                highland_task.LeaveTimeoutSec);
            highland_task.LeaveTimeoutSec = 8;
        }
        auto& base_task = config.RegionalAreaTaskSettings.MyBase;
        if (base_task.TravelTimeoutSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyBase.TravelTimeoutSec={}, fallback to 12.",
                base_task.TravelTimeoutSec);
            base_task.TravelTimeoutSec = 12;
        }
        if (base_task.CommandHoldSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyBase.CommandHoldSec={}, fallback to 1.",
                base_task.CommandHoldSec);
            base_task.CommandHoldSec = 1;
        }
        if (base_task.GoalHoldSec < 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyBase.GoalHoldSec={}, fallback to 15.",
                base_task.GoalHoldSec);
            base_task.GoalHoldSec = 15;
        }
        if (base_task.MaxPatrolSteps <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyBase.MaxPatrolSteps={}, fallback to 4.",
                base_task.MaxPatrolSteps);
            base_task.MaxPatrolSteps = 4;
        }
        if (base_task.PatrolDistancePenaltyPerMeter < 0.0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyBase.Patrol.DistancePenaltyPerMeter={}, fallback to 0.4.",
                base_task.PatrolDistancePenaltyPerMeter);
            base_task.PatrolDistancePenaltyPerMeter = 0.4;
        }
        if (base_task.PatrolCurrentGoalPenalty < 0.0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyBase.Patrol.CurrentGoalPenalty={}, fallback to 5.",
                base_task.PatrolCurrentGoalPenalty);
            base_task.PatrolCurrentGoalPenalty = 5.0;
        }
        std::vector<LangYa::MyBasePatrolGoalSetting> sanitized_base_patrol_goals;
        sanitized_base_patrol_goals.reserve(base_task.PatrolGoals.size());
        for (const auto& goal : base_task.PatrolGoals) {
            if (!IsValidBaseGoal(goal.BaseGoalId) ||
                BehaviorTree::AreaManager::IsReservedNonCombatGoalId(goal.BaseGoalId)) {
                LoggerPtr->Warning(
                    "Ignore invalid RegionalAreaTask.MyBase.Patrol.GoalWeights goal={}.",
                    static_cast<int>(goal.BaseGoalId));
                continue;
            }
            const auto it = std::find_if(
                sanitized_base_patrol_goals.begin(),
                sanitized_base_patrol_goals.end(),
                [&goal](const LangYa::MyBasePatrolGoalSetting& item) {
                    return item.BaseGoalId == goal.BaseGoalId;
                });
            if (it != sanitized_base_patrol_goals.end()) {
                it->Weight = goal.Weight;
                continue;
            }
            sanitized_base_patrol_goals.push_back(goal);
        }
        const bool has_enabled_base_patrol_goal = std::any_of(
            sanitized_base_patrol_goals.begin(),
            sanitized_base_patrol_goals.end(),
            [](const LangYa::MyBasePatrolGoalSetting& goal) {
                return goal.Weight > 0.0;
            });
        if (!has_enabled_base_patrol_goal) {
            LoggerPtr->Warning(
                "RegionalAreaTask.MyBase.Patrol has no enabled goal, fallback to default Base patrol weights.");
            sanitized_base_patrol_goals = {
                {LangYa::CastleLeft1.ID, 10.0},
                {LangYa::CastleLeft2.ID, 10.0},
                {LangYa::CastleRight2.ID, 10.0},
                {LangYa::CastleRight1.ID, 10.0},
                {LangYa::HoleRoad.ID, 7.0},
                {LangYa::OutpostGuard.ID, 7.0},
                {LangYa::BuffOutpost.ID, 6.0}
            };
        }
        base_task.PatrolGoals = std::move(sanitized_base_patrol_goals);
        auto& roadland_task = config.RegionalAreaTaskSettings.MyRoadland;
        if (roadland_task.TravelTimeoutSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyRoadland.TravelTimeoutSec={}, fallback to 12.",
                roadland_task.TravelTimeoutSec);
            roadland_task.TravelTimeoutSec = 12;
        }
        if (roadland_task.CrossTimeoutSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyRoadland.CrossTimeoutSec={}, fallback to 8.",
                roadland_task.CrossTimeoutSec);
            roadland_task.CrossTimeoutSec = 8;
        }
        if (roadland_task.CommandHoldSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyRoadland.CommandHoldSec={}, fallback to 1.",
                roadland_task.CommandHoldSec);
            roadland_task.CommandHoldSec = 1;
        }
        if (roadland_task.GuardHoldSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyRoadland.GuardHoldSec={}, fallback to 2.",
                roadland_task.GuardHoldSec);
            roadland_task.GuardHoldSec = 2;
        }
        if (roadland_task.FaceTargetZCm < 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyRoadland.FaceTargetZCm={}, fallback to 100.",
                roadland_task.FaceTargetZCm);
            roadland_task.FaceTargetZCm = 100;
        }
        if (roadland_task.HealthyHpMin < 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyRoadland.HealthyHpMin={}, fallback to 300.",
                roadland_task.HealthyHpMin);
            roadland_task.HealthyHpMin = 300;
        }
        if (roadland_task.HealthyAmmoMin < 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.MyRoadland.HealthyAmmoMin={}, fallback to 50.",
                roadland_task.HealthyAmmoMin);
            roadland_task.HealthyAmmoMin = 50;
        }
        auto& central_task = config.RegionalAreaTaskSettings.CommonCentral;
        if (central_task.TravelTimeoutSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.CommonCentral.TravelTimeoutSec={}, fallback to 12.",
                central_task.TravelTimeoutSec);
            central_task.TravelTimeoutSec = 12;
        }
        if (central_task.CommandHoldSec <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.CommonCentral.CommandHoldSec={}, fallback to 1.",
                central_task.CommandHoldSec);
            central_task.CommandHoldSec = 1;
        }
        if (central_task.MaxPatrolSteps <= 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.CommonCentral.MaxPatrolSteps={}, fallback to 8.",
                central_task.MaxPatrolSteps);
            central_task.MaxPatrolSteps = 8;
        }
        if (central_task.HealthyHpMin < 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.CommonCentral.HealthyHpMin={}, fallback to 300.",
                central_task.HealthyHpMin);
            central_task.HealthyHpMin = 300;
        }
        if (central_task.HealthyAmmoMin < 0) {
            LoggerPtr->Warning(
                "Invalid RegionalAreaTask.CommonCentral.HealthyAmmoMin={}, fallback to 50.",
                central_task.HealthyAmmoMin);
            central_task.HealthyAmmoMin = 50;
        }
        auto& default_policy = config.RegionalAreaTaskSettings.DefaultPolicy;
        if (default_policy.Health.MyAreaHpMin < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Health.MyAreaHpMin={}, fallback to 250.",
                default_policy.Health.MyAreaHpMin);
            default_policy.Health.MyAreaHpMin = 250;
        }
        if (default_policy.Health.CommonCentralHpMin < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Health.CommonCentralHpMin={}, fallback to 300.",
                default_policy.Health.CommonCentralHpMin);
            default_policy.Health.CommonCentralHpMin = 300;
        }
        if (default_policy.Health.EnemyAreaHpMin < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Health.EnemyAreaHpMin={}, fallback to 350.",
                default_policy.Health.EnemyAreaHpMin);
            default_policy.Health.EnemyAreaHpMin = 350;
        }
        if (default_policy.Health.LowResourceFallbackHp < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Health.LowResourceFallbackHp={}, fallback to 250.",
                default_policy.Health.LowResourceFallbackHp);
            default_policy.Health.LowResourceFallbackHp = 250;
        }
        if (default_policy.Ammo.MyAreaAmmoMin < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Ammo.MyAreaAmmoMin={}, fallback to 50.",
                default_policy.Ammo.MyAreaAmmoMin);
            default_policy.Ammo.MyAreaAmmoMin = 50;
        }
        if (default_policy.Ammo.CommonCentralAmmoMin < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Ammo.CommonCentralAmmoMin={}, fallback to 50.",
                default_policy.Ammo.CommonCentralAmmoMin);
            default_policy.Ammo.CommonCentralAmmoMin = 50;
        }
        if (default_policy.Ammo.EnemyAreaAmmoMin < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Ammo.EnemyAreaAmmoMin={}, fallback to 80.",
                default_policy.Ammo.EnemyAreaAmmoMin);
            default_policy.Ammo.EnemyAreaAmmoMin = 80;
        }
        if (default_policy.Ammo.LowResourceFallbackAmmo < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Ammo.LowResourceFallbackAmmo={}, fallback to 30.",
                default_policy.Ammo.LowResourceFallbackAmmo);
            default_policy.Ammo.LowResourceFallbackAmmo = 30;
        }
        if (default_policy.Score.DistancePenaltyPerMeter < 0.0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Score.DistancePenaltyPerMeter={}, fallback to 0.4.",
                default_policy.Score.DistancePenaltyPerMeter);
            default_policy.Score.DistancePenaltyPerMeter = 0.4;
        }
        if (default_policy.Retry.CompleteCooldownSec < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Retry.CompleteCooldownSec={}, fallback to 2.",
                default_policy.Retry.CompleteCooldownSec);
            default_policy.Retry.CompleteCooldownSec = 2;
        }
        if (default_policy.Retry.FailureCooldownSec < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Retry.FailureCooldownSec={}, fallback to 8.",
                default_policy.Retry.FailureCooldownSec);
            default_policy.Retry.FailureCooldownSec = 8;
        }
        if (default_policy.Retry.UnreachableCooldownSec < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Retry.UnreachableCooldownSec={}, fallback to 12.",
                default_policy.Retry.UnreachableCooldownSec);
            default_policy.Retry.UnreachableCooldownSec = 12;
        }
        if (default_policy.Retry.MaxRetry < 0) {
            LoggerPtr->Warning("Invalid DefaultPolicy.Retry.MaxRetry={}, fallback to 2.",
                default_policy.Retry.MaxRetry);
            default_policy.Retry.MaxRetry = 2;
        }

        auto valid_patrol_scan_mode = [](const int mode) {
            return mode == 1 || mode == 2 || mode == 3;
        };
        if (!valid_patrol_scan_mode(config.PatrolScanSettings.Mode)) {
            LoggerPtr->Warning("Invalid PatrolScan.Mode={}, fallback to 1.", config.PatrolScanSettings.Mode);
            config.PatrolScanSettings.Mode = 1;
        }
        if (!valid_patrol_scan_mode(config.FaceModeSettings.FallbackPatrolScanMode)) {
            LoggerPtr->Warning(
                "Invalid FaceMode.FallbackPatrolScanMode={}, fallback to 2.",
                config.FaceModeSettings.FallbackPatrolScanMode);
            config.FaceModeSettings.FallbackPatrolScanMode = 2;
        }
        if (!valid_patrol_scan_mode(config.FaceModeSettings.OutpostFallbackPatrolScanMode)) {
            LoggerPtr->Warning(
                "Invalid FaceMode.OutpostFallbackPatrolScanMode={}, fallback to 3.",
                config.FaceModeSettings.OutpostFallbackPatrolScanMode);
            config.FaceModeSettings.OutpostFallbackPatrolScanMode = 3;
        }
        auto& patrol = config.PatrolScanSettings;
        auto sanitize_positive_double = [this](double& value, const double fallback, const char* key) {
            if (!std::isfinite(value) || value <= 0.0) {
                LoggerPtr->Warning("Invalid {}={}, fallback to {}.", key, value, fallback);
                value = fallback;
            }
        };
        auto sanitize_non_negative_double = [this](double& value, const double fallback, const char* key) {
            if (!std::isfinite(value) || value < 0.0) {
                LoggerPtr->Warning("Invalid {}={}, fallback to {}.", key, value, fallback);
                value = fallback;
            }
        };
        auto sanitize_finite_double = [this](double& value, const double fallback, const char* key) {
            if (!std::isfinite(value)) {
                LoggerPtr->Warning("Invalid {}={}, fallback to {}.", key, value, fallback);
                value = fallback;
            }
        };
        sanitize_positive_double(
            patrol.Mode1YawStepDegPerTick, 9.0, "PatrolScan.Mode1.YawStepDegPerTick");
        sanitize_positive_double(
            patrol.Mode1YawBoostStepDegPerTick, 10.0, "PatrolScan.Mode1.YawBoostStepDegPerTick");
        sanitize_finite_double(
            patrol.Mode1PitchCenterDeg, 0.0, "PatrolScan.Mode1.PitchCenterDeg");
        sanitize_non_negative_double(
            patrol.Mode1PitchHalfRangeDeg, 13.0, "PatrolScan.Mode1.PitchHalfRangeDeg");
        sanitize_positive_double(
            patrol.Mode1PitchPeriodMs, 500.0, "PatrolScan.Mode1.PitchPeriodMs");
        sanitize_positive_double(
            patrol.Mode2YawStepDegPerTick, 1.0, "PatrolScan.Mode2.YawStepDegPerTick");
        sanitize_positive_double(
            patrol.Mode2YawBoostStepDegPerTick, 1.1, "PatrolScan.Mode2.YawBoostStepDegPerTick");
        sanitize_positive_double(
            patrol.Mode2YawHalfRangeDeg, 30.0, "PatrolScan.Mode2.YawHalfRangeDeg");
        sanitize_finite_double(
            patrol.Mode2CenterDriftPerCycleDeg, -70.0, "PatrolScan.Mode2.CenterDriftPerCycleDeg");
        sanitize_finite_double(
            patrol.Mode2PitchCenterDeg, 0.0, "PatrolScan.Mode2.PitchCenterDeg");
        sanitize_non_negative_double(
            patrol.Mode2PitchHalfRangeDeg, 13.0, "PatrolScan.Mode2.PitchHalfRangeDeg");
        sanitize_positive_double(
            patrol.Mode2PitchPeriodMs, 500.0, "PatrolScan.Mode2.PitchPeriodMs");
        sanitize_positive_double(
            patrol.Mode3YawStepDegPerTick, 1.0, "PatrolScan.Mode3.YawStepDegPerTick");
        sanitize_finite_double(
            patrol.Mode3PitchOffsetDeg, 15.0, "PatrolScan.Mode3.PitchOffsetDeg");
        sanitize_non_negative_double(
            patrol.Mode3PitchHalfRangeDeg, 3.0, "PatrolScan.Mode3.PitchHalfRangeDeg");
        sanitize_positive_double(
            patrol.Mode3PitchPeriodMs, 500.0, "PatrolScan.Mode3.PitchPeriodMs");

        const std::vector<int> default_aim_target_priority{
            static_cast<int>(ArmorType::Hero),
            static_cast<int>(ArmorType::Infantry1),
            static_cast<int>(ArmorType::Infantry2),
            static_cast<int>(ArmorType::Sentry),
            static_cast<int>(ArmorType::Engineer)
        };
        const auto is_valid_armor_priority = [](const int armor_id) -> bool {
            switch (static_cast<ArmorType>(armor_id)) {
                case ArmorType::Hero:
                case ArmorType::Engineer:
                case ArmorType::Infantry1:
                case ArmorType::Infantry2:
                case ArmorType::Sentry:
                    return true;
                default:
                    return false;
            }
        };
        std::vector<int> sanitized_aim_target_priority;
        sanitized_aim_target_priority.reserve(config.AimTargetPriority.size());
        for (const auto armor_id : config.AimTargetPriority) {
            if (!is_valid_armor_priority(armor_id)) {
                LoggerPtr->Warning("Ignore invalid AimTargetPriority item={}.", armor_id);
                continue;
            }
            if (std::find(sanitized_aim_target_priority.begin(),
                          sanitized_aim_target_priority.end(),
                          armor_id) != sanitized_aim_target_priority.end()) {
                continue;
            }
            sanitized_aim_target_priority.push_back(armor_id);
        }
        if (sanitized_aim_target_priority.empty()) {
            LoggerPtr->Warning("AimTargetPriority is empty after sanitize, fallback to default.");
            sanitized_aim_target_priority = default_aim_target_priority;
        }
        config.AimTargetPriority = std::move(sanitized_aim_target_priority);

        const auto is_valid_armor_ignore = [](const int armor_id) -> bool {
            switch (static_cast<ArmorType>(armor_id)) {
                case ArmorType::Hero:
                case ArmorType::Engineer:
                case ArmorType::Infantry1:
                case ArmorType::Infantry2:
                case ArmorType::Sentry:
                case ArmorType::Outpost:
                    return true;
                default:
                    return false;
            }
        };
        std::vector<int> sanitized_aim_target_ignore;
        sanitized_aim_target_ignore.reserve(config.AimTargetIgnore.size());
        for (const auto armor_id : config.AimTargetIgnore) {
            if (!is_valid_armor_ignore(armor_id)) {
                LoggerPtr->Warning("Ignore invalid AimTargetIgnore item={}.", armor_id);
                continue;
            }
            if (std::find(sanitized_aim_target_ignore.begin(),
                          sanitized_aim_target_ignore.end(),
                          armor_id) != sanitized_aim_target_ignore.end()) {
                continue;
            }
            sanitized_aim_target_ignore.push_back(armor_id);
        }
        config.AimTargetIgnore = std::move(sanitized_aim_target_ignore);

        auto sanitize_module_list = [](const std::vector<std::string>& input_modules) {
            std::vector<std::string> output_modules;
            output_modules.reserve(input_modules.size());
            for (auto module : input_modules) {
                module = NormalizeAutonomyToken(std::move(module));
                if (module.empty()) {
                    continue;
                }
                if (std::find(output_modules.begin(), output_modules.end(), module) != output_modules.end()) {
                    continue;
                }
                output_modules.push_back(std::move(module));
            }
            return output_modules;
        };
        config.DecisionAutonomySettings.EnabledModules =
            sanitize_module_list(config.DecisionAutonomySettings.EnabledModules);
        config.DecisionAutonomySettings.HardRuleModules =
            sanitize_module_list(config.DecisionAutonomySettings.HardRuleModules);
        if (config.DecisionAutonomySettings.EnabledModules.empty()) {
            config.DecisionAutonomySettings.EnabledModules = {"aim_target"};
        }

        auto& autonomy = config.DecisionAutonomySettings;
        auto sanitize_area_list = [this](const std::vector<std::string>& areas,
                                         const char* option_name,
                                         const bool common_area) {
            std::vector<std::string> sanitized_areas;
            sanitized_areas.reserve(areas.size());
            for (const auto& area : areas) {
                const auto normalized = NormalizeMainAreaToken(area);
                if (normalized.empty()) {
                    LoggerPtr->Warning("Ignore invalid DecisionAutonomy.{} area '{}'.",
                                       option_name, area);
                    continue;
                }
                if (common_area && normalized != "central") {
                    LoggerPtr->Warning("Ignore DecisionAutonomy.{} area '{}': CommonArea only supports Central.",
                                       option_name, area);
                    continue;
                }
                if (!common_area && normalized == "central") {
                    LoggerPtr->Warning("Ignore DecisionAutonomy.{} area '{}': Central belongs to CommonArea.",
                                       option_name, area);
                    continue;
                }
                if (std::find(sanitized_areas.begin(),
                              sanitized_areas.end(),
                              normalized) != sanitized_areas.end()) {
                    continue;
                }
                sanitized_areas.push_back(normalized);
            }
            return sanitized_areas;
        };
        autonomy.NaviGoal.MyArea =
            sanitize_area_list(autonomy.NaviGoal.MyArea, "NaviGoal.MyArea", false);
        autonomy.NaviGoal.EnemyArea =
            sanitize_area_list(autonomy.NaviGoal.EnemyArea, "NaviGoal.EnemyArea", false);
        autonomy.NaviGoal.CommonArea =
            sanitize_area_list(autonomy.NaviGoal.CommonArea, "NaviGoal.CommonArea", true);
        if (autonomy.NaviGoal.UseAreaScope) {
            if (autonomy.NaviGoal.MyArea.empty()) {
                LoggerPtr->Warning("DecisionAutonomy.NaviGoal.UseAreaScope=true but MyArea is empty; my-side navigation goals will be blocked.");
            }
            if (autonomy.NaviGoal.EnemyArea.empty()) {
                LoggerPtr->Warning("DecisionAutonomy.NaviGoal.UseAreaScope=true but EnemyArea is empty; enemy-side navigation goals will be blocked.");
            }
        }

        auto clamp_non_negative = [this](double& value, const char* key_name) {
            if (value < 0.0) {
                LoggerPtr->Warning("Invalid {}={}, clamp to 0.", key_name, value);
                value = 0.0;
            }
        };
        auto clamp_non_negative_int = [this](int& value, const char* key_name) {
            if (value < 0) {
                LoggerPtr->Warning("Invalid {}={}, clamp to 0.", key_name, value);
                value = 0;
            }
        };
        if (autonomy.NaviGoal.HighlandCompatArriveDistanceCm <= 0) {
            LoggerPtr->Warning("Invalid DecisionAutonomy.NaviGoal.HighlandCompat.ArriveDistanceCm={}, fallback to 20.",
                               autonomy.NaviGoal.HighlandCompatArriveDistanceCm);
            autonomy.NaviGoal.HighlandCompatArriveDistanceCm = 20;
        }
        if (autonomy.NaviGoal.HighlandCompatTimeoutSec <= 0) {
            LoggerPtr->Warning("Invalid DecisionAutonomy.NaviGoal.HighlandCompat.TimeoutSec={}, fallback to 6.",
                               autonomy.NaviGoal.HighlandCompatTimeoutSec);
            autonomy.NaviGoal.HighlandCompatTimeoutSec = 6;
        }
        if (autonomy.NaviGoal.BuffOutpostCompatTimeoutSec <= 0) {
            LoggerPtr->Warning("Invalid DecisionAutonomy.NaviGoal.BuffOutpostCompat.TimeoutSec={}, fallback to 6.",
                               autonomy.NaviGoal.BuffOutpostCompatTimeoutSec);
            autonomy.NaviGoal.BuffOutpostCompatTimeoutSec = 6;
        }
        if (autonomy.NaviGoal.DistanceFallbackGraceMs < 0) {
            LoggerPtr->Warning("Invalid DecisionAutonomy.NaviGoal.DistanceFallbackGraceMs={}, fallback to 3000.",
                               autonomy.NaviGoal.DistanceFallbackGraceMs);
            autonomy.NaviGoal.DistanceFallbackGraceMs = 3000;
        }
        clamp_non_negative(autonomy.AimTarget.PriorityWeight, "DecisionAutonomy.AimTarget.PriorityWeight");
        clamp_non_negative(autonomy.AimTarget.DistanceWeight, "DecisionAutonomy.AimTarget.DistanceWeight");
        clamp_non_negative(autonomy.AimTarget.LowHealthWeight, "DecisionAutonomy.AimTarget.LowHealthWeight");
        clamp_non_negative(autonomy.AimTarget.CurrentTargetBonus, "DecisionAutonomy.AimTarget.CurrentTargetBonus");
        clamp_non_negative(autonomy.AimTarget.HeroBonus, "DecisionAutonomy.AimTarget.HeroBonus");
        clamp_non_negative(autonomy.AimTarget.SentryBonus, "DecisionAutonomy.AimTarget.SentryBonus");
        clamp_non_negative(autonomy.AimTarget.SwitchScoreMargin, "DecisionAutonomy.AimTarget.SwitchScoreMargin");
        clamp_non_negative_int(autonomy.AimTarget.HealthFreshTimeoutMs, "DecisionAutonomy.AimTarget.HealthFreshTimeoutMs");
        clamp_non_negative_int(autonomy.AimTarget.DeadHealthConfirmMs, "DecisionAutonomy.AimTarget.DeadHealthConfirmMs");
        clamp_non_negative_int(autonomy.AimTarget.DeadHealthHoldMs, "DecisionAutonomy.AimTarget.DeadHealthHoldMs");
        clamp_non_negative_int(
            autonomy.AimTarget.RespawnTransitionTimeoutMs,
            "DecisionAutonomy.AimTarget.RespawnTransitionTimeoutMs");
        clamp_non_negative_int(autonomy.AimTarget.LostTargetHoldMs, "DecisionAutonomy.AimTarget.LostTargetHoldMs");
        clamp_non_negative_int(autonomy.AimTarget.MinSwitchIntervalMs, "DecisionAutonomy.AimTarget.MinSwitchIntervalMs");
        clamp_non_negative_int(autonomy.AimTarget.RespawnInvulnerableSec, "DecisionAutonomy.AimTarget.RespawnInvulnerableSec");
        clamp_non_negative_int(
            autonomy.AimTarget.SentryRespawnInvulnerableSec,
            "DecisionAutonomy.AimTarget.SentryRespawnInvulnerableSec");

        if (config.ChaseSettings.LostTargetHoldMs < 0) {
            LoggerPtr->Warning("Invalid Chase.LostTargetHoldMs={}, fallback to 0.",
                               config.ChaseSettings.LostTargetHoldMs);
            config.ChaseSettings.LostTargetHoldMs = 0;
        }
        if (config.ChaseSettings.OfficialPositionFreshMs < 0) {
            LoggerPtr->Warning("Invalid Chase.OfficialPositionFreshMs={}, fallback to 0.",
                               config.ChaseSettings.OfficialPositionFreshMs);
            config.ChaseSettings.OfficialPositionFreshMs = 0;
        }
        if (config.ChaseSettings.PreferredDistanceCm <= 0) {
            LoggerPtr->Warning("Invalid Chase.PreferredDistanceCm={}, fallback to 100.",
                               config.ChaseSettings.PreferredDistanceCm);
            config.ChaseSettings.PreferredDistanceCm = 100;
        }
        if (config.ChaseSettings.DistanceDeadbandCm < 0) {
            LoggerPtr->Warning("Invalid Chase.DistanceDeadbandCm={}, fallback to 0.",
                               config.ChaseSettings.DistanceDeadbandCm);
            config.ChaseSettings.DistanceDeadbandCm = 0;
        }
        if (config.ChaseSettings.AreaLimit.BoundaryMarginCm < 0) {
            LoggerPtr->Warning("Invalid Chase.AreaLimit.BoundaryMarginCm={}, fallback to 0.",
                               config.ChaseSettings.AreaLimit.BoundaryMarginCm);
            config.ChaseSettings.AreaLimit.BoundaryMarginCm = 0;
        }
        if (config.ChaseSettings.MinValidDistanceCm < 0) {
            LoggerPtr->Warning("Invalid Chase.MinValidDistanceCm={}, fallback to 0.",
                               config.ChaseSettings.MinValidDistanceCm);
            config.ChaseSettings.MinValidDistanceCm = 0;
        }
        if (config.ChaseSettings.MaxValidDistanceCm <= config.ChaseSettings.MinValidDistanceCm) {
            LoggerPtr->Warning(
                "Invalid Chase distance range [{}, {}], fallback to [80, 1200].",
                config.ChaseSettings.MinValidDistanceCm,
                config.ChaseSettings.MaxValidDistanceCm);
            config.ChaseSettings.MinValidDistanceCm = 80;
            config.ChaseSettings.MaxValidDistanceCm = 1200;
        }
        if (config.ChaseSettings.DistanceKp < 0.0) {
            LoggerPtr->Warning("Invalid Chase.DistanceKp={}, fallback to 0.06.",
                               config.ChaseSettings.DistanceKp);
            config.ChaseSettings.DistanceKp = 0.06;
        }
        config.ChaseSettings.MaxForwardSpeed = std::clamp(config.ChaseSettings.MaxForwardSpeed, 0, 127);
        config.ChaseSettings.MaxBackwardSpeed = std::clamp(config.ChaseSettings.MaxBackwardSpeed, 0, 127);
        config.ChaseSettings.MaxStrafeSpeed = std::clamp(config.ChaseSettings.MaxStrafeSpeed, 0, 127);
        if (config.ChaseSettings.YawKp < 0.0) {
            LoggerPtr->Warning("Invalid Chase.YawKp={}, fallback to 0.4.",
                               config.ChaseSettings.YawKp);
            config.ChaseSettings.YawKp = 0.4;
        }
        if (config.ChaseSettings.YawDeadbandDeg < 0) {
            LoggerPtr->Warning("Invalid Chase.YawDeadbandDeg={}, fallback to 0.",
                               config.ChaseSettings.YawDeadbandDeg);
            config.ChaseSettings.YawDeadbandDeg = 0;
        }

        const std::string config_profile = NormalizeProfile(config.CompetitionProfile);
        const std::string effective_profile = competitionProfileOverride_.empty()
            ? config_profile
            : NormalizeProfile(competitionProfileOverride_);
        if (!competitionProfileOverride_.empty() && !config_profile.empty() &&
            effective_profile != config_profile) {
            LoggerPtr->Warning(
                "competition_profile override '{}' replaces config profile '{}'.",
                competitionProfileOverride_, config.CompetitionProfile);
        }
        // 最终生效 profile = launch override 优先，其次 JSON，最后 regional 默认值。
        const auto selected_profile = effective_profile.empty() ? std::string("regional") : effective_profile;
        config.CompetitionProfile = selected_profile;
        competitionProfile_ = ParseCompetitionProfile(config.CompetitionProfile);
        LoggerPtr->Info("Effective CompetitionProfile: {}", CompetitionProfileToString(competitionProfile_));
        if (competitionProfile_ == CompetitionProfile::League && config.NaviSettings.UseXY) {
            LoggerPtr->Warning("League profile is using UseXY=true. Goal-ID mode is recommended for league.");
        }
        if (config.ShowcasePatrolSettings.Enable && config.NaviSettings.UseXY) {
            LoggerPtr->Warning("ShowcasePatrol is enabled with UseXY=true. DisableTeamOffset only affects /ly/navi/goal.");
        }
        if (config.NaviDebugSettings.Enable && config.NaviSettings.UseXY) {
            LoggerPtr->Warning("NaviDebug is enabled with UseXY=true. Dedicated point-plan mode recommends /ly/navi/goal.");
        }

        if (competitionProfile_ == CompetitionProfile::League) {
            strategyMode_ = StrategyMode::LeagueSimple;
        } else {
            strategyMode_ = StrategyMode::Regional;
        }
        LoggerPtr->Info("Initial StrategyMode: {}", StrategyModeToString(strategyMode_));

        const auto now = std::chrono::steady_clock::now();
        areaManager_.Configure(config.DecisionAutonomySettings.NaviGoal);
        areaManager_.Reset(now);
        postureManager_.Configure(config.PostureSettings);
        postureManager_.Reset(now, SentryPosture::Move);
        leaguePatrolGoalIndex_ = 0;
        leaguePatrolGoalInitialized_ = false;
        showcasePatrolGoalIndex_ = 0;
        showcasePatrolGoalInitialized_ = false;
        naviDebugGoalIndex_ = 0;
        naviDebugGoalInitialized_ = false;
        leagueRecoveryActive_ = false;
        leagueRecoveryStartTime_ = std::chrono::steady_clock::time_point{};
        leagueRecoveryReach350Time_ = std::chrono::steady_clock::time_point{};
        leagueRecoveryCooldownUntil_ = std::chrono::steady_clock::time_point{};
        leagueRecoveryEntryHealth_ = 0;
        leagueRecoveryPeakHealth_ = 0;

        return true;
    }
}
