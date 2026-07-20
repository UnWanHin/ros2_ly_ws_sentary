#include "../include/AreaManager.hpp"

#include <gimbal_driver/msg/unit_info.hpp>

#include <charconv>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {

using BehaviorTree::AreaKey;
using BehaviorTree::AreaSide;
using BehaviorTree::ResolvedAreaKey;
using LangYa::UnitTeam;

struct Options {
    int x{0};
    int y{0};
    bool has_x{false};
    bool has_y{false};
    bool print_all_teams{true};
    UnitTeam my_team{UnitTeam::Red};
    bool switch_point{false};
    bool help_requested{false};
};

std::string ToLower(std::string_view text) {
    std::string out{text};
    for (auto& ch : out) {
        if (ch >= 'A' && ch <= 'Z') {
            ch = static_cast<char>(ch - 'A' + 'a');
        }
    }
    return out;
}

std::optional<int> ParseInt(std::string_view text) {
    int value = 0;
    const auto* begin = text.data();
    const auto* end = text.data() + text.size();
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc{} || result.ptr != end) {
        return std::nullopt;
    }
    return value;
}

std::optional<bool> ParseBool(std::string_view text) {
    const auto value = ToLower(text);
    if (value == "1" || value == "true" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off") {
        return false;
    }
    return std::nullopt;
}

std::optional<UnitTeam> ParseTeam(std::string_view text) {
    const auto team = ToLower(text);
    if (team == "red" || team == "r") {
        return UnitTeam::Red;
    }
    if (team == "blue" || team == "b") {
        return UnitTeam::Blue;
    }
    return std::nullopt;
}

const char* TeamName(UnitTeam team) {
    switch (team) {
        case UnitTeam::Red:
            return "red";
        case UnitTeam::Blue:
            return "blue";
        default:
            return "unknown";
    }
}

UnitTeam OppositeTeam(UnitTeam team) {
    if (team == UnitTeam::Red) {
        return UnitTeam::Blue;
    }
    if (team == UnitTeam::Blue) {
        return UnitTeam::Red;
    }
    return UnitTeam::Unknown;
}

const char* AreaSideName(AreaSide side) {
    switch (side) {
        case AreaSide::My:
            return "my";
        case AreaSide::Enemy:
            return "enemy";
        case AreaSide::Common:
            return "common";
        default:
            return "unknown";
    }
}

std::string AreaKeyName(const AreaKey& key) {
    if (key.Side == AreaSide::Common) {
        return "central";
    }
    if (key.Side == AreaSide::My) {
        return std::string{"my_"} + BehaviorTree::Area::MainAreaKindName(key.Kind);
    }
    if (key.Side == AreaSide::Enemy) {
        return std::string{"enemy_"} + BehaviorTree::Area::MainAreaKindName(key.Kind);
    }
    return "unknown";
}

std::uint8_t AreaId(const AreaKey& key) {
    using Info = gimbal_driver::msg::UnitInfo;
    if (key.Side == AreaSide::Common &&
        key.Kind == BehaviorTree::Area::MainAreaKind::Central) {
        return Info::AREA_CENTRAL;
    }
    if (key.Side == AreaSide::My) {
        switch (key.Kind) {
            case BehaviorTree::Area::MainAreaKind::Base:
                return Info::AREA_MY_BASE;
            case BehaviorTree::Area::MainAreaKind::Highland:
                return Info::AREA_MY_HIGHLAND;
            case BehaviorTree::Area::MainAreaKind::PreRoadland:
                return Info::AREA_MY_PRE_ROADLAND;
            case BehaviorTree::Area::MainAreaKind::ReadyRoadland:
                return Info::AREA_MY_ROADLAND;
            default:
                return Info::AREA_UNKNOWN;
        }
    }
    if (key.Side == AreaSide::Enemy) {
        switch (key.Kind) {
            case BehaviorTree::Area::MainAreaKind::Base:
                return Info::AREA_ENEMY_BASE;
            case BehaviorTree::Area::MainAreaKind::Highland:
                return Info::AREA_ENEMY_HIGHLAND;
            case BehaviorTree::Area::MainAreaKind::PreRoadland:
                return Info::AREA_ENEMY_PRE_ROADLAND;
            case BehaviorTree::Area::MainAreaKind::ReadyRoadland:
                return Info::AREA_ENEMY_ROADLAND;
            default:
                return Info::AREA_UNKNOWN;
        }
    }
    return Info::AREA_UNKNOWN;
}

void PrintUsage(const char* argv0) {
    std::cerr
        << "Usage:\n"
        << "  " << argv0 << " <official_map_x_cm> <official_map_y_cm> [--team red|blue]\n"
        << "  " << argv0 << " --team red --switch-point false -- 315 1219\n\n"
        << "Options:\n"
        << "  --team <red|blue>       Print one formal my/enemy perspective.\n"
        << "                           If omitted, both red and blue perspectives are printed.\n"
        << "  --switch-point <bool>   Match AreaManager.SwitchPoint lookup behavior.\n"
        << "  --help                  Show this message.\n";
}

bool ParseArgs(int argc, char** argv, Options& options) {
    std::vector<std::string_view> positional;
    bool end_of_options = false;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg{argv[i]};
        if (!end_of_options && arg == "--") {
            end_of_options = true;
            continue;
        }
        if (!end_of_options && (arg == "--help" || arg == "-h")) {
            PrintUsage(argv[0]);
            options.help_requested = true;
            return true;
        }
        if (!end_of_options && (arg == "--team" || arg == "-t")) {
            if (++i >= argc) {
                std::cerr << "Missing value for " << arg << "\n";
                return false;
            }
            const auto team = ParseTeam(argv[i]);
            if (!team.has_value()) {
                std::cerr << "Invalid team: " << argv[i] << "\n";
                return false;
            }
            options.my_team = *team;
            options.print_all_teams = false;
            continue;
        }
        if (!end_of_options && arg.rfind("--team=", 0) == 0) {
            const auto team = ParseTeam(arg.substr(7));
            if (!team.has_value()) {
                std::cerr << "Invalid team: " << arg.substr(7) << "\n";
                return false;
            }
            options.my_team = *team;
            options.print_all_teams = false;
            continue;
        }
        if (!end_of_options && arg == "--switch-point") {
            if (++i >= argc) {
                std::cerr << "Missing value for " << arg << "\n";
                return false;
            }
            const auto enabled = ParseBool(argv[i]);
            if (!enabled.has_value()) {
                std::cerr << "Invalid switch-point value: " << argv[i] << "\n";
                return false;
            }
            options.switch_point = *enabled;
            continue;
        }
        if (!end_of_options && arg.rfind("--switch-point=", 0) == 0) {
            const auto enabled = ParseBool(arg.substr(15));
            if (!enabled.has_value()) {
                std::cerr << "Invalid switch-point value: " << arg.substr(15) << "\n";
                return false;
            }
            options.switch_point = *enabled;
            continue;
        }
        positional.push_back(arg);
    }

    if (positional.size() != 2U) {
        PrintUsage(argv[0]);
        return false;
    }
    const auto x = ParseInt(positional[0]);
    const auto y = ParseInt(positional[1]);
    if (!x.has_value() || !y.has_value()) {
        std::cerr << "Coordinates must be integer official-map centimeters.\n";
        return false;
    }
    options.x = *x;
    options.y = *y;
    options.has_x = true;
    options.has_y = true;
    return true;
}

void PrintExactMembership(UnitTeam area_team, int x, int y) {
    std::cout << "  exact_" << TeamName(area_team) << "_main_area: ";
    const auto exact = BehaviorTree::AreaManager::ResolvePointMainAreaExact(area_team, x, y);
    if (exact.has_value()) {
        std::cout << BehaviorTree::Area::MainAreaKindName(*exact) << "\n";
    } else {
        std::cout << "none\n";
    }
    std::cout << "  " << TeamName(area_team) << "_protect_hero: "
              << (BehaviorTree::Area::IsPointInsideProtectHeroArea(area_team, x, y) ? "true" : "false") << "\n";
    std::cout << "  " << TeamName(area_team) << "_pre_roadland: "
              << (BehaviorTree::Area::IsPointInsidePreRoadlandArea(area_team, x, y) ? "true" : "false") << "\n";
    std::cout << "  " << TeamName(area_team) << "_recovery_area: "
              << (BehaviorTree::Area::IsPointInsideRecoveryArea(area_team, x, y) ? "true" : "false") << "\n";
    std::cout << "  " << TeamName(area_team) << "_central_left_line: "
              << (BehaviorTree::Area::IsPointInsideCentralLeftLineArea(area_team, x, y) ? "true" : "false") << "\n";
    std::cout << "  " << TeamName(area_team) << "_settle_area: "
              << (BehaviorTree::Area::IsPointInsideSettleArea(area_team, x, y) ? "true" : "false") << "\n";
}

void PrintResolution(UnitTeam my_team, int x, int y) {
    const auto enemy_team = OppositeTeam(my_team);
    const auto resolved = BehaviorTree::AreaManager::ResolveAreaKeyForPointWithNearest(
        my_team,
        enemy_team,
        x,
        y);

    std::cout << "perspective:\n";
    std::cout << "  my_team: " << TeamName(my_team) << "\n";
    std::cout << "  enemy_team: " << TeamName(enemy_team) << "\n";
    if (!resolved.has_value()) {
        std::cout << "  resolved_area: unknown\n";
        std::cout << "  area_id: 0\n";
        std::cout << "  used_nearest_fallback: false\n";
    } else {
        const auto& key = resolved->Key;
        std::cout << "  resolved_area: " << AreaKeyName(key) << "\n";
        std::cout << "  area_id: " << static_cast<int>(AreaId(key)) << "\n";
        std::cout << "  area_side: " << AreaSideName(key.Side) << "\n";
        std::cout << "  area_kind: " << BehaviorTree::Area::MainAreaKindName(key.Kind) << "\n";
        std::cout << "  area_team: " << TeamName(key.Team) << "\n";
        std::cout << "  used_nearest_fallback: "
                  << (resolved->UsedNearestFallback ? "true" : "false") << "\n";
    }
    PrintExactMembership(UnitTeam::Red, x, y);
    PrintExactMembership(UnitTeam::Blue, x, y);
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseArgs(argc, argv, options)) {
        return 1;
    }
    if (options.help_requested) {
        return 0;
    }

    BehaviorTree::Area::SetSwitchPoint(options.switch_point);

    std::cout << "input:\n";
    std::cout << "  x_cm: " << options.x << "\n";
    std::cout << "  y_cm: " << options.y << "\n";
    std::cout << "  frame: official_map\n";
    std::cout << "  switch_point: " << (options.switch_point ? "true" : "false") << "\n";

    if (options.print_all_teams) {
        PrintResolution(UnitTeam::Red, options.x, options.y);
        PrintResolution(UnitTeam::Blue, options.x, options.y);
        return 0;
    }

    PrintResolution(options.my_team, options.x, options.y);
    return 0;
}
