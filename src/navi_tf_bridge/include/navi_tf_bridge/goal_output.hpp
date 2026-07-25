#pragma once

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

namespace navi_tf_bridge
{

geometry_msgs::msg::Point ScaleGoalPosePoint(
  const geometry_msgs::msg::Point & point_map,
  double uniform_scale);

class GoalOutput
{
public:
  struct Config
  {
    std::string map_frame{"map"};
    bool publish_target_map{true};
    bool publish_goal_pos{false};
    bool publish_goal_pose{true};
    double goal_pose_uniform_scale{1.0};
    bool invert_y_axis{false};
    int y_axis_max_cm{1500};
    bool uint16_encode_enabled{false};
    double uint16_encode_x_scale{1.0};
    double uint16_encode_y_scale{1.0};
    double uint16_encode_x_offset_cm{0.0};
    double uint16_encode_y_offset_cm{0.0};
  };

  struct Publishers
  {
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr goal_pos;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose;
  };

  GoalOutput() = default;
  explicit GoalOutput(Config config);

  void setConfig(Config config);
  const Config & config() const;
  void sanitize(rclcpp::Node & node);

  void publishMapPointAsGoal(
    const geometry_msgs::msg::Point & point_map,
    const std::string & resolved_source_frame,
    const rclcpp::Time & stamp,
    rclcpp::Node & node,
    const Publishers & publishers) const;

private:
  void publishMapPointAsGoalPose(
    const geometry_msgs::msg::Point & point_map,
    const rclcpp::Time & stamp,
    rclcpp::Node & node,
    const Publishers & publishers) const;

  Config config_{};
};

}  // namespace navi_tf_bridge
