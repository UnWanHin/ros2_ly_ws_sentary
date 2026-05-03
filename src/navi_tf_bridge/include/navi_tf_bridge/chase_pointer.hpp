#pragma once

#include <string>
#include <vector>

#include "auto_aim_common/msg/relative_target.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace navi_tf_bridge
{

class ChasePointer
{
public:
  struct Config
  {
    std::string map_frame{"map"};
    std::string base_frame{"base_link"};
    std::string fallback_base_frame{"baselink"};
    bool use_msg_frame_id{true};
    std::string default_frame{"gimbal_world"};
    int preferred_distance_cm{100};
    int distance_deadband_cm{50};
    bool stop_when_no_target{true};
    bool allow_reverse_goal{false};
  };

  ChasePointer() = default;
  explicit ChasePointer(Config config);

  void setConfig(Config config);
  const Config & config() const;

  std::vector<std::string> buildSourceCandidates(const std::string & msg_frame_id) const;
  bool buildRelativeGoalPoint(
    const auto_aim_common::msg::RelativeTarget & msg,
    rclcpp::Node & node,
    geometry_msgs::msg::Point & point_out) const;
  bool transformToMap(
    const geometry_msgs::msg::Point & point_rel,
    const std::vector<std::string> & source_candidates,
    const rclcpp::Time & transform_time,
    tf2_ros::Buffer & tf_buffer,
    geometry_msgs::msg::PointStamped & point_map,
    std::string & resolved_source_frame,
    std::string & last_tf_error) const;

private:
  static void appendUniqueFrame(std::vector<std::string> & frames, const std::string & frame);

  Config config_{};
};

}  // namespace navi_tf_bridge
