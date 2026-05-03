#pragma once

#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace navi_tf_bridge
{

class PointerDebug
{
public:
  struct Config
  {
    bool enabled{true};
    std::string map_frame{"map"};
    std::string reference_frame{"map"};
    std::string area_header_file{};
    std::string output_file{};
  };

  PointerDebug() = default;
  explicit PointerDebug(Config config);

  void setConfig(Config config);
  const Config & config() const;

  bool onTimer(
    const std::vector<std::string> & source_candidates,
    tf2_ros::Buffer & tf_buffer,
    rclcpp::Node & node);

private:
  struct NamedPointCm
  {
    std::string name;
    int x_cm{0};
    int y_cm{0};
  };

  bool parseAreaHeaderPoints(rclcpp::Node & node, std::vector<NamedPointCm> & points_out) const;
  bool lookupTransformWithCandidates(
    const std::vector<std::string> & source_candidates,
    const rclcpp::Time & transform_time,
    tf2_ros::Buffer & tf_buffer,
    geometry_msgs::msg::TransformStamped & tf_out,
    std::string & resolved_source_frame,
    std::string & last_tf_error) const;
  bool exportPointPairs(
    const std::string & tf_ready_source_frame,
    const geometry_msgs::msg::TransformStamped & tf_map_ref,
    rclcpp::Node & node);

  Config config_{};
  bool tf_ready_logged_once_{false};
  bool tf_missing_logged_once_{false};
  bool exported_{false};
};

}  // namespace navi_tf_bridge
