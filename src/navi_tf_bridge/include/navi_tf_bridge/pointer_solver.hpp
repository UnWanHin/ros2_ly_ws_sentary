#pragma once

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

namespace navi_tf_bridge
{

class PointerSolver
{
public:
  struct Config
  {
    bool enabled{false};
    std::string model{"rigid"};
    std::string unit{"cm"};
    std::string source_frame{"official_map"};
    std::string target_frame{"map"};
    std::string map_frame{"map"};
    std::vector<double> source_points{};
    std::vector<double> target_points{};
    std::vector<double> transform_matrix{};
  };

  struct Transform2D
  {
    double m00{1.0};
    double m01{0.0};
    double m10{0.0};
    double m11{1.0};
    double tx_m{0.0};
    double ty_m{0.0};
  };

  struct RawGoalPair
  {
    double sx_m{0.0};
    double sy_m{0.0};
    double tx_m{0.0};
    double ty_m{0.0};
  };

  PointerSolver() = default;
  explicit PointerSolver(Config config);

  void setConfig(Config config);
  const Config & config() const;

  bool initialize(rclcpp::Node & node);
  bool ready() const;
  const Transform2D & transform() const;
  geometry_msgs::msg::Point applyMeters(double x_m, double y_m, double z_m = 0.0) const;

private:
  bool initializeMatrix(rclcpp::Node & node);
  bool parsePairs(rclcpp::Node & node, std::vector<RawGoalPair> & pairs_out) const;

  Config config_{};
  Transform2D transform_{};
  bool ready_{false};
};

}  // namespace navi_tf_bridge
