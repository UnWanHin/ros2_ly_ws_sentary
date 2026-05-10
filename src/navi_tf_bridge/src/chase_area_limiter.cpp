#include "navi_tf_bridge/chase_area_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace navi_tf_bridge
{
namespace
{

constexpr double kEpsilon = 1e-9;

double cross(
  const ChaseAreaLimiter::PointCm a,
  const ChaseAreaLimiter::PointCm b)
{
  return a.x * b.y - a.y * b.x;
}

ChaseAreaLimiter::PointCm subtract(
  const ChaseAreaLimiter::PointCm a,
  const ChaseAreaLimiter::PointCm b)
{
  return ChaseAreaLimiter::PointCm{a.x - b.x, a.y - b.y};
}

}  // namespace

ChaseAreaLimiter::ChaseAreaLimiter(Config config)
: config_(std::move(config))
{
}

void ChaseAreaLimiter::setConfig(Config config)
{
  config.boundary_margin_cm = std::max(0.0, config.boundary_margin_cm);
  config_ = std::move(config);
}

const ChaseAreaLimiter::Config & ChaseAreaLimiter::config() const
{
  return config_;
}

bool ChaseAreaLimiter::isFinite(const PointCm point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool ChaseAreaLimiter::isPointOnSegment(
  const PointCm point,
  const PointCm start,
  const PointCm end)
{
  const double area = cross(subtract(point, start), subtract(end, start));
  if (std::abs(area) > kEpsilon) {
    return false;
  }
  return point.x >= std::min(start.x, end.x) - kEpsilon &&
         point.x <= std::max(start.x, end.x) + kEpsilon &&
         point.y >= std::min(start.y, end.y) - kEpsilon &&
         point.y <= std::max(start.y, end.y) + kEpsilon;
}

bool ChaseAreaLimiter::isPointInsidePolygon(
  const std::vector<PointCm> & polygon,
  const PointCm point)
{
  if (polygon.size() < 3 || !isFinite(point)) {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & pi = polygon[i];
    const auto & pj = polygon[j];
    if (isPointOnSegment(point, pj, pi)) {
      return true;
    }
    if ((pi.y > point.y) != (pj.y > point.y)) {
      const double intersect_x =
        (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x;
      if (point.x < intersect_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

const ChaseAreaLimiter::Area * ChaseAreaLimiter::findContainingArea(
  const std::vector<Area> & areas,
  const PointCm point)
{
  for (const auto & area : areas) {
    if (isPointInsidePolygon(area.boundary, point)) {
      return &area;
    }
  }
  return nullptr;
}

bool ChaseAreaLimiter::segmentIntersectionT(
  const PointCm start,
  const PointCm end,
  const PointCm edge_start,
  const PointCm edge_end,
  double & t_out)
{
  const PointCm r = subtract(end, start);
  const PointCm s = subtract(edge_end, edge_start);
  const double denom = cross(r, s);
  if (std::abs(denom) <= kEpsilon) {
    return false;
  }

  const PointCm qp = subtract(edge_start, start);
  const double t = cross(qp, s) / denom;
  const double u = cross(qp, r) / denom;
  if (t < -kEpsilon || t > 1.0 + kEpsilon || u < -kEpsilon || u > 1.0 + kEpsilon) {
    return false;
  }

  t_out = std::clamp(t, 0.0, 1.0);
  return true;
}

bool ChaseAreaLimiter::firstBoundaryIntersectionT(
  const std::vector<PointCm> & polygon,
  const PointCm start,
  const PointCm end,
  double & t_out)
{
  if (polygon.size() < 3) {
    return false;
  }

  double best_t = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    double t = 0.0;
    if (segmentIntersectionT(start, end, polygon[j], polygon[i], t) && t > kEpsilon) {
      best_t = std::min(best_t, t);
    }
  }

  if (!std::isfinite(best_t)) {
    return false;
  }
  t_out = best_t;
  return true;
}

ChaseAreaLimiter::Result ChaseAreaLimiter::limit(
  const PointCm self,
  const PointCm goal) const
{
  Result result{};
  result.point = goal;

  if (!config_.enabled) {
    result.status = Status::Disabled;
    return result;
  }
  if (config_.areas.empty()) {
    result.status = Status::EmptyConfig;
    return result;
  }
  if (!isFinite(self) || !isFinite(goal)) {
    result.status = Status::InvalidInput;
    result.publish = false;
    return result;
  }

  const auto * area = findContainingArea(config_.areas, self);
  if (area == nullptr) {
    result.status = Status::UnknownArea;
    if (config_.hold_when_unknown_area) {
      result.point = self;
      result.hold_current = true;
    }
    return result;
  }

  result.area_name = area->name;
  if (isPointInsidePolygon(area->boundary, goal)) {
    result.status = Status::Inside;
    return result;
  }

  double t_hit = 0.0;
  if (!firstBoundaryIntersectionT(area->boundary, self, goal, t_hit)) {
    result.status = Status::NoIntersection;
    if (config_.hold_when_no_intersection) {
      result.point = self;
      result.hold_current = true;
    }
    return result;
  }

  const double dx = goal.x - self.x;
  const double dy = goal.y - self.y;
  const double length_cm = std::hypot(dx, dy);
  double t_limit = t_hit;
  if (length_cm > kEpsilon && config_.boundary_margin_cm > 0.0) {
    t_limit = std::max(0.0, t_hit - config_.boundary_margin_cm / length_cm);
  }

  result.status = Status::Clamped;
  result.clamped = true;
  result.point = PointCm{self.x + dx * t_limit, self.y + dy * t_limit};
  return result;
}

const char * ChaseAreaLimiter::statusName(const Status status)
{
  switch (status) {
    case Status::Disabled:
      return "disabled";
    case Status::EmptyConfig:
      return "empty_config";
    case Status::InvalidInput:
      return "invalid_input";
    case Status::Inside:
      return "inside";
    case Status::Clamped:
      return "clamped";
    case Status::UnknownArea:
      return "unknown_area";
    case Status::NoIntersection:
      return "no_intersection";
    default:
      return "unknown";
  }
}

}  // namespace navi_tf_bridge
