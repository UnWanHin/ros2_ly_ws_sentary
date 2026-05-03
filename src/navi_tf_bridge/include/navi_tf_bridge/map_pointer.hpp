#pragma once

#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "navi_tf_bridge/pointer_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "tf2_ros/buffer.h"

namespace navi_tf_bridge
{

class MapPointer
{
public:
  struct Config
  {
    bool enabled{true};
    std::string raw_frame{"map"};
    std::string map_frame{"map"};
    bool use_static_calibration{false};
    PointerSolver::Config solver{};
  };

  MapPointer() = default;
  explicit MapPointer(Config config);

  void setConfig(Config config);
  const Config & config() const;

  void initialize(rclcpp::Node & node);
  bool toMap(
    const std_msgs::msg::UInt16MultiArray & msg,
    tf2_ros::Buffer & tf_buffer,
    rclcpp::Node & node,
    geometry_msgs::msg::PointStamped & point_map,
    std::string & source_name);

private:
  Config config_{};
  PointerSolver solver_{};
  bool raw_tf_ready_logged_once_{false};
  bool raw_tf_missing_logged_once_{false};
};

}  // namespace navi_tf_bridge
