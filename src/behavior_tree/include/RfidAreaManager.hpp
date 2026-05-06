#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "../module/Area.hpp"
#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

enum class RfidAreaKind : std::uint8_t {
    Unknown,
    SelfBaseGainPoint,
    SelfSupply,
    SelfNonResourceSupply,
    SelfResourceSupply,
    SelfHighlandGainPoint,
    EnemyHighlandGainPoint,
    SelfRoadCrossing,
    EnemyRoadCrossing,
    SelfCentralHighlandCrossing,
    EnemyCentralHighlandCrossing,
    SelfTunnel,
    EnemyTunnel,
    Tunnel,
    CenterGainPoint,
    SelfFortressGainPoint,
    EnemyFortressGainPoint,
    SelfOutpostGainPoint,
    EnemyOutpostGainPoint,
    SelfAssemblyGainPoint,
    EnemyAssemblyGainPoint,
    SelfFlyRamp,
    EnemyFlyRamp,
    OnSelfSideRfid,
    OnEnemySideRfid,
    Any,
};

struct RfidAreaSpec {
    std::string Name{};
    RfidAreaKind Kind{RfidAreaKind::Unknown};
    std::vector<Area::Point<int>> Boundary{};
    int Priority{0};
    bool Enabled{true};
};

struct RfidAreaEvaluation {
    std::string Name{};
    RfidAreaKind Kind{RfidAreaKind::Unknown};
    int Priority{0};
    bool Enabled{false};
    bool RfidFresh{false};
    bool RfidTriggered{false};
    bool HasCenter{false};
    Area::Point<double> Center{};
};

const char* RfidAreaKindName(RfidAreaKind kind) noexcept;
std::optional<RfidAreaKind> RfidAreaKindFromToken(std::string_view token);

bool IsRfidAreaKindTriggered(const RfidMatchState& state, RfidAreaKind kind) noexcept;
bool IsPointInsideRfidArea(const RfidAreaSpec& spec, int x, int y) noexcept;

std::optional<Area::Point<double>> ComputeRfidAreaCenter(
    const std::vector<Area::Point<int>>& boundary) noexcept;
std::optional<Area::Point<std::uint16_t>> ComputeRfidAreaCenterGoal(
    const std::vector<Area::Point<int>>& boundary) noexcept;

RfidAreaEvaluation EvaluateRfidArea(
    const RfidMatchState& state,
    const RfidAreaSpec& spec);
std::vector<RfidAreaEvaluation> EvaluateRfidAreas(
    const RfidMatchState& state,
    const std::vector<RfidAreaSpec>& specs);
std::optional<RfidAreaEvaluation> SelectHighestPriorityTriggeredRfidArea(
    const RfidMatchState& state,
    const std::vector<RfidAreaSpec>& specs);

} // namespace BehaviorTree
