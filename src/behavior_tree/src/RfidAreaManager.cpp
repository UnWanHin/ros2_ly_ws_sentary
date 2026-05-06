#include "../include/RfidAreaManager.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace BehaviorTree {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::string NormalizeToken(std::string_view token) {
    std::string normalized;
    normalized.reserve(token.size());
    for (const char c : token) {
        if (c == '_' || c == '-' || c == '/' || c == ' ') {
            continue;
        }
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return normalized;
}

Area::Point<double> AverageCenter(const std::vector<Area::Point<int>>& boundary) noexcept {
    long double sum_x = 0.0L;
    long double sum_y = 0.0L;
    for (const auto& point : boundary) {
        sum_x += static_cast<long double>(point.x);
        sum_y += static_cast<long double>(point.y);
    }
    const auto count = static_cast<long double>(boundary.size());
    return {
        static_cast<double>(sum_x / count),
        static_cast<double>(sum_y / count),
    };
}

std::uint16_t ClampRoundToU16(const double value) noexcept {
    if (!std::isfinite(value) || value <= 0.0) {
        return 0;
    }
    constexpr auto max_u16 = static_cast<double>(std::numeric_limits<std::uint16_t>::max());
    if (value >= max_u16) {
        return std::numeric_limits<std::uint16_t>::max();
    }
    return static_cast<std::uint16_t>(std::lround(value));
}

bool IsCircleRingValid(const Area::CircleRing<double>& ring) noexcept {
    return std::isfinite(ring.center.x) &&
           std::isfinite(ring.center.y) &&
           std::isfinite(ring.innerRadiusCm) &&
           std::isfinite(ring.outerRadiusCm) &&
           ring.innerRadiusCm >= 0.0 &&
           ring.outerRadiusCm > ring.innerRadiusCm;
}

std::vector<Area::Point<double>> ComputeCircleRingRepresentativePoints(
    const Area::CircleRing<double>& ring) noexcept {
    std::vector<Area::Point<double>> points;
    if (!IsCircleRingValid(ring)) {
        return points;
    }

    const int segment_count = std::max(1, ring.segmentCount);
    points.reserve(static_cast<std::size_t>(segment_count));

    const double radius = (ring.innerRadiusCm + ring.outerRadiusCm) * 0.5;
    const double start_angle_rad = ring.startAngleDeg * kPi / 180.0;
    const double step = 2.0 * kPi / static_cast<double>(segment_count);
    for (int i = 0; i < segment_count; ++i) {
        const double angle = start_angle_rad + step * static_cast<double>(i);
        points.push_back({
            ring.center.x + radius * std::cos(angle),
            ring.center.y + radius * std::sin(angle),
        });
    }
    return points;
}

} // namespace

const char* RfidAreaKindName(const RfidAreaKind kind) noexcept {
    switch (kind) {
        case RfidAreaKind::SelfBaseGainPoint: return "SelfBaseGainPoint";
        case RfidAreaKind::SelfSupply: return "SelfSupply";
        case RfidAreaKind::SelfNonResourceSupply: return "SelfNonResourceSupply";
        case RfidAreaKind::SelfResourceSupply: return "SelfResourceSupply";
        case RfidAreaKind::SelfHighlandGainPoint: return "SelfHighlandGainPoint";
        case RfidAreaKind::EnemyHighlandGainPoint: return "EnemyHighlandGainPoint";
        case RfidAreaKind::SelfRoadCrossing: return "SelfRoadCrossing";
        case RfidAreaKind::EnemyRoadCrossing: return "EnemyRoadCrossing";
        case RfidAreaKind::SelfCentralHighlandCrossing: return "SelfCentralHighlandCrossing";
        case RfidAreaKind::EnemyCentralHighlandCrossing: return "EnemyCentralHighlandCrossing";
        case RfidAreaKind::SelfTunnel: return "SelfTunnel";
        case RfidAreaKind::EnemyTunnel: return "EnemyTunnel";
        case RfidAreaKind::Tunnel: return "Tunnel";
        case RfidAreaKind::CenterGainPoint: return "CenterGainPoint";
        case RfidAreaKind::SelfFortressGainPoint: return "SelfFortressGainPoint";
        case RfidAreaKind::EnemyFortressGainPoint: return "EnemyFortressGainPoint";
        case RfidAreaKind::SelfOutpostGainPoint: return "SelfOutpostGainPoint";
        case RfidAreaKind::EnemyOutpostGainPoint: return "EnemyOutpostGainPoint";
        case RfidAreaKind::SelfAssemblyGainPoint: return "SelfAssemblyGainPoint";
        case RfidAreaKind::EnemyAssemblyGainPoint: return "EnemyAssemblyGainPoint";
        case RfidAreaKind::SelfFlyRamp: return "SelfFlyRamp";
        case RfidAreaKind::EnemyFlyRamp: return "EnemyFlyRamp";
        case RfidAreaKind::OnSelfSideRfid: return "OnSelfSideRfid";
        case RfidAreaKind::OnEnemySideRfid: return "OnEnemySideRfid";
        case RfidAreaKind::Any: return "Any";
        case RfidAreaKind::Unknown:
        default:
            return "Unknown";
    }
}

std::optional<RfidAreaKind> RfidAreaKindFromToken(const std::string_view token) {
    const auto normalized = NormalizeToken(token);
    if (normalized == "selfbasegainpoint") return RfidAreaKind::SelfBaseGainPoint;
    if (normalized == "selfsupply") return RfidAreaKind::SelfSupply;
    if (normalized == "selfnonresourcesupply") return RfidAreaKind::SelfNonResourceSupply;
    if (normalized == "selfresourcesupply") return RfidAreaKind::SelfResourceSupply;
    if (normalized == "selfhighlandgainpoint") return RfidAreaKind::SelfHighlandGainPoint;
    if (normalized == "enemyhighlandgainpoint") return RfidAreaKind::EnemyHighlandGainPoint;
    if (normalized == "selfroadcrossing") return RfidAreaKind::SelfRoadCrossing;
    if (normalized == "enemyroadcrossing") return RfidAreaKind::EnemyRoadCrossing;
    if (normalized == "selfcentralhighlandcrossing") return RfidAreaKind::SelfCentralHighlandCrossing;
    if (normalized == "enemycentralhighlandcrossing") return RfidAreaKind::EnemyCentralHighlandCrossing;
    if (normalized == "selftunnel") return RfidAreaKind::SelfTunnel;
    if (normalized == "enemytunnel") return RfidAreaKind::EnemyTunnel;
    if (normalized == "tunnel") return RfidAreaKind::Tunnel;
    if (normalized == "centergainpoint") return RfidAreaKind::CenterGainPoint;
    if (normalized == "selffortressgainpoint") return RfidAreaKind::SelfFortressGainPoint;
    if (normalized == "enemyfortressgainpoint") return RfidAreaKind::EnemyFortressGainPoint;
    if (normalized == "selfoutpostgainpoint") return RfidAreaKind::SelfOutpostGainPoint;
    if (normalized == "enemyoutpostgainpoint") return RfidAreaKind::EnemyOutpostGainPoint;
    if (normalized == "selfassemblygainpoint") return RfidAreaKind::SelfAssemblyGainPoint;
    if (normalized == "enemyassemblygainpoint") return RfidAreaKind::EnemyAssemblyGainPoint;
    if (normalized == "selfflyramp") return RfidAreaKind::SelfFlyRamp;
    if (normalized == "enemyflyramp") return RfidAreaKind::EnemyFlyRamp;
    if (normalized == "onselfsiderfid") return RfidAreaKind::OnSelfSideRfid;
    if (normalized == "onenemysiderfid") return RfidAreaKind::OnEnemySideRfid;
    if (normalized == "any") return RfidAreaKind::Any;
    return std::nullopt;
}

bool IsRfidAreaKindTriggered(const RfidMatchState& state, const RfidAreaKind kind) noexcept {
    if (!state.Fresh) {
        return false;
    }

    switch (kind) {
        case RfidAreaKind::SelfBaseGainPoint: return state.SelfBaseGainPoint;
        case RfidAreaKind::SelfSupply: return state.SelfSupply;
        case RfidAreaKind::SelfNonResourceSupply: return state.SelfNonResourceSupply;
        case RfidAreaKind::SelfResourceSupply: return state.SelfResourceSupply;
        case RfidAreaKind::SelfHighlandGainPoint: return state.SelfHighlandGainPoint;
        case RfidAreaKind::EnemyHighlandGainPoint: return state.EnemyHighlandGainPoint;
        case RfidAreaKind::SelfRoadCrossing: return state.SelfRoadCrossing;
        case RfidAreaKind::EnemyRoadCrossing: return state.EnemyRoadCrossing;
        case RfidAreaKind::SelfCentralHighlandCrossing: return state.SelfCentralHighlandCrossing;
        case RfidAreaKind::EnemyCentralHighlandCrossing: return state.EnemyCentralHighlandCrossing;
        case RfidAreaKind::SelfTunnel: return state.SelfTunnel;
        case RfidAreaKind::EnemyTunnel: return state.EnemyTunnel;
        case RfidAreaKind::Tunnel: return state.Tunnel;
        case RfidAreaKind::CenterGainPoint: return state.CenterGainPoint;
        case RfidAreaKind::SelfFortressGainPoint: return state.SelfFortressGainPoint;
        case RfidAreaKind::EnemyFortressGainPoint: return state.EnemyFortressGainPoint;
        case RfidAreaKind::SelfOutpostGainPoint: return state.SelfOutpostGainPoint;
        case RfidAreaKind::EnemyOutpostGainPoint: return state.EnemyOutpostGainPoint;
        case RfidAreaKind::SelfAssemblyGainPoint: return state.SelfAssemblyGainPoint;
        case RfidAreaKind::EnemyAssemblyGainPoint: return state.EnemyAssemblyGainPoint;
        case RfidAreaKind::SelfFlyRamp: return state.SelfFlyRamp;
        case RfidAreaKind::EnemyFlyRamp: return state.EnemyFlyRamp;
        case RfidAreaKind::OnSelfSideRfid: return state.OnSelfSideRfid;
        case RfidAreaKind::OnEnemySideRfid: return state.OnEnemySideRfid;
        case RfidAreaKind::Any: return state.Any;
        case RfidAreaKind::Unknown:
        default:
            return false;
    }
}

bool IsPointInsideRfidArea(const RfidAreaSpec& spec, const int x, const int y) noexcept {
    if (!spec.Enabled) {
        return false;
    }
    switch (spec.Shape) {
        case Area::ShapeType::Polygon:
            if (spec.Boundary.size() < 3) {
                return false;
            }
            return Area::IsPointInsideMainAreaBoundary(spec.Boundary, x, y);
        case Area::ShapeType::CircleRing: {
            if (!IsCircleRingValid(spec.CircleRing)) {
                return false;
            }
            const double dx = static_cast<double>(x) - spec.CircleRing.center.x;
            const double dy = static_cast<double>(y) - spec.CircleRing.center.y;
            const double distance_sq = dx * dx + dy * dy;
            const double inner_sq = spec.CircleRing.innerRadiusCm * spec.CircleRing.innerRadiusCm;
            const double outer_sq = spec.CircleRing.outerRadiusCm * spec.CircleRing.outerRadiusCm;
            return distance_sq >= inner_sq && distance_sq <= outer_sq;
        }
        default:
            return false;
    }
}

std::optional<Area::Point<double>> ComputeRfidAreaCenter(
    const std::vector<Area::Point<int>>& boundary) noexcept {
    if (boundary.empty()) {
        return std::nullopt;
    }
    if (boundary.size() < 3) {
        return AverageCenter(boundary);
    }

    long double twice_area = 0.0L;
    long double weighted_x = 0.0L;
    long double weighted_y = 0.0L;
    for (std::size_t i = 0, j = boundary.size() - 1; i < boundary.size(); j = i++) {
        const auto& p0 = boundary[j];
        const auto& p1 = boundary[i];
        const long double cross =
            static_cast<long double>(p0.x) * static_cast<long double>(p1.y) -
            static_cast<long double>(p1.x) * static_cast<long double>(p0.y);
        twice_area += cross;
        weighted_x += (static_cast<long double>(p0.x) + static_cast<long double>(p1.x)) * cross;
        weighted_y += (static_cast<long double>(p0.y) + static_cast<long double>(p1.y)) * cross;
    }

    if (std::abs(twice_area) <= 1.0e-9L) {
        return AverageCenter(boundary);
    }

    return Area::Point<double>{
        static_cast<double>(weighted_x / (3.0L * twice_area)),
        static_cast<double>(weighted_y / (3.0L * twice_area)),
    };
}

std::optional<Area::Point<std::uint16_t>> ComputeRfidAreaCenterGoal(
    const std::vector<Area::Point<int>>& boundary) noexcept {
    const auto center = ComputeRfidAreaCenter(boundary);
    if (!center.has_value()) {
        return std::nullopt;
    }
    return Area::Point<std::uint16_t>{
        ClampRoundToU16(center->x),
        ClampRoundToU16(center->y),
    };
}

std::vector<Area::Point<double>> ComputeRfidAreaRepresentativePoints(
    const RfidAreaSpec& spec) noexcept {
    switch (spec.Shape) {
        case Area::ShapeType::Polygon: {
            std::vector<Area::Point<double>> points;
            const auto center = ComputeRfidAreaCenter(spec.Boundary);
            if (center.has_value()) {
                points.push_back(*center);
            }
            return points;
        }
        case Area::ShapeType::CircleRing:
            return ComputeCircleRingRepresentativePoints(spec.CircleRing);
        default:
            return {};
    }
}

std::vector<Area::Point<std::uint16_t>> ComputeRfidAreaRepresentativeGoals(
    const RfidAreaSpec& spec) noexcept {
    const auto points = ComputeRfidAreaRepresentativePoints(spec);
    std::vector<Area::Point<std::uint16_t>> goals;
    goals.reserve(points.size());
    for (const auto& point : points) {
        goals.push_back({
            ClampRoundToU16(point.x),
            ClampRoundToU16(point.y),
        });
    }
    return goals;
}

std::optional<Area::Point<double>> ComputeRfidAreaCenter(const RfidAreaSpec& spec) noexcept {
    const auto points = ComputeRfidAreaRepresentativePoints(spec);
    if (points.empty()) {
        return std::nullopt;
    }
    return points.front();
}

std::optional<Area::Point<std::uint16_t>> ComputeRfidAreaCenterGoal(
    const RfidAreaSpec& spec) noexcept {
    const auto center = ComputeRfidAreaCenter(spec);
    if (!center.has_value()) {
        return std::nullopt;
    }
    return Area::Point<std::uint16_t>{
        ClampRoundToU16(center->x),
        ClampRoundToU16(center->y),
    };
}

RfidAreaEvaluation EvaluateRfidArea(
    const RfidMatchState& state,
    const RfidAreaSpec& spec) {
    RfidAreaEvaluation evaluation;
    evaluation.Name = spec.Name;
    evaluation.Kind = spec.Kind;
    evaluation.Priority = spec.Priority;
    evaluation.Enabled = spec.Enabled;
    evaluation.RfidFresh = state.Fresh;
    evaluation.RfidTriggered = spec.Enabled && IsRfidAreaKindTriggered(state, spec.Kind);

    evaluation.RepresentativePoints = ComputeRfidAreaRepresentativePoints(spec);
    if (!evaluation.RepresentativePoints.empty()) {
        evaluation.HasCenter = true;
        evaluation.Center = evaluation.RepresentativePoints.front();
    }
    return evaluation;
}

std::vector<RfidAreaEvaluation> EvaluateRfidAreas(
    const RfidMatchState& state,
    const std::vector<RfidAreaSpec>& specs) {
    std::vector<RfidAreaEvaluation> evaluations;
    evaluations.reserve(specs.size());
    for (const auto& spec : specs) {
        evaluations.push_back(EvaluateRfidArea(state, spec));
    }
    return evaluations;
}

std::optional<RfidAreaEvaluation> SelectHighestPriorityTriggeredRfidArea(
    const RfidMatchState& state,
    const std::vector<RfidAreaSpec>& specs) {
    std::optional<RfidAreaEvaluation> best;
    for (const auto& spec : specs) {
        auto evaluation = EvaluateRfidArea(state, spec);
        if (!evaluation.RfidTriggered || !evaluation.HasCenter) {
            continue;
        }
        if (!best.has_value() || evaluation.Priority > best->Priority) {
            best = std::move(evaluation);
        }
    }
    return best;
}

} // namespace BehaviorTree
