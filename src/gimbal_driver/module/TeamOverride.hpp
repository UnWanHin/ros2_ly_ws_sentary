#pragma once

namespace LangYa {

struct TeamOverrideConfig {
    bool enabled{false};
    bool red{false};
    bool blue{false};
};

enum class TeamOverrideStatus {
    Disabled,
    ForceRed,
    ForceBlue,
    Invalid,
};

struct TeamOverrideResolution {
    bool is_team_red{false};
    TeamOverrideStatus status{TeamOverrideStatus::Disabled};
};

// The override is resolved in the gimbal driver's semantic boundary.  This
// preserves one formal publisher while retaining lower-machine data as input.
inline constexpr TeamOverrideResolution ResolveTeamOverride(
    const TeamOverrideConfig& config,
    const bool lower_machine_is_red) noexcept {
    if (!config.enabled) {
        return {lower_machine_is_red, TeamOverrideStatus::Disabled};
    }
    if (config.red && !config.blue) {
        return {true, TeamOverrideStatus::ForceRed};
    }
    if (!config.red && config.blue) {
        return {false, TeamOverrideStatus::ForceBlue};
    }
    return {lower_machine_is_red, TeamOverrideStatus::Invalid};
}

}  // namespace LangYa
