#pragma once

#include <string>
#include <vector>

namespace navi_tf_bridge
{

class ChaseAreaLimiter
{
public:
  struct PointCm
  {
    double x{0.0};
    double y{0.0};
  };

  struct Area
  {
    std::string name{};
    std::vector<PointCm> boundary{};
  };

  struct Config
  {
    bool enabled{false};
    std::string area_header_file{};
    double boundary_margin_cm{30.0};
    bool hold_when_unknown_area{false};
    bool hold_when_no_intersection{true};
    std::vector<Area> areas{};
  };

  enum class Status
  {
    Disabled,
    EmptyConfig,
    InvalidInput,
    Inside,
    Clamped,
    UnknownArea,
    NoIntersection,
  };

  struct Result
  {
    bool publish{true};
    bool clamped{false};
    bool hold_current{false};
    Status status{Status::Disabled};
    std::string area_name{};
    PointCm point{};
  };

  ChaseAreaLimiter() = default;
  explicit ChaseAreaLimiter(Config config);

  void setConfig(Config config);
  const Config & config() const;

  Result limit(PointCm self, PointCm goal) const;

  static const char * statusName(Status status);

private:
  static bool isFinite(PointCm point);
  static bool isPointOnSegment(PointCm point, PointCm start, PointCm end);
  static bool isPointInsidePolygon(const std::vector<PointCm> & polygon, PointCm point);
  static const Area * findContainingArea(const std::vector<Area> & areas, PointCm point);
  static bool segmentIntersectionT(
    PointCm start,
    PointCm end,
    PointCm edge_start,
    PointCm edge_end,
    double & t_out);
  static bool firstBoundaryIntersectionT(
    const std::vector<PointCm> & polygon,
    PointCm start,
    PointCm end,
    double & t_out);

  Config config_{};
};

}  // namespace navi_tf_bridge
