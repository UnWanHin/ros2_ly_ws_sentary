#include "navi_tf_bridge/goal_output.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace navi_tf_bridge
{

GoalOutput::GoalOutput(Config config)
: config_(std::move(config))
{
}

void GoalOutput::setConfig(Config config)
{
  config_ = std::move(config);
}

const GoalOutput::Config & GoalOutput::config() const
{
  return config_;
}

void GoalOutput::sanitize(rclcpp::Node & node)
{
  if (std::abs(config_.uint16_encode_x_scale) <= 1e-9) {
    RCLCPP_WARN(
      node.get_logger(),
      "goal_pos_uint16_encode_x_scale is near zero, fallback to 1.0");
    config_.uint16_encode_x_scale = 1.0;
  }
  if (std::abs(config_.uint16_encode_y_scale) <= 1e-9) {
    RCLCPP_WARN(
      node.get_logger(),
      "goal_pos_uint16_encode_y_scale is near zero, fallback to 1.0");
    config_.uint16_encode_y_scale = 1.0;
  }
}

void GoalOutput::publishMapPointAsGoal(
  const geometry_msgs::msg::Point & point_map,
  const std::string & resolved_source_frame,
  const rclcpp::Time & stamp,
  rclcpp::Node & node,
  const Publishers & publishers) const
{
  publishMapPointAsGoalPose(point_map, stamp, node, publishers);
  if (!config_.publish_goal_pos || !publishers.goal_pos) {
    return;
  }

  long x_cm = std::lround(point_map.x * 100.0);
  long y_cm = std::lround(point_map.y * 100.0);

  if (config_.invert_y_axis) {
    y_cm = static_cast<long>(config_.y_axis_max_cm) - y_cm;
  }

  long encoded_x_cm = x_cm;
  long encoded_y_cm = y_cm;
  if (config_.uint16_encode_enabled) {
    encoded_x_cm = std::lround(
      config_.uint16_encode_x_scale * static_cast<double>(x_cm) +
      config_.uint16_encode_x_offset_cm);
    encoded_y_cm = std::lround(
      config_.uint16_encode_y_scale * static_cast<double>(y_cm) +
      config_.uint16_encode_y_offset_cm);
  }

  const long kU16Min = 0L;
  const long kU16Max = static_cast<long>(std::numeric_limits<std::uint16_t>::max());
  if (
    encoded_x_cm < kU16Min || encoded_x_cm > kU16Max || encoded_y_cm < kU16Min ||
    encoded_y_cm > kU16Max)
  {
    RCLCPP_WARN_THROTTLE(
      node.get_logger(),
      *node.get_clock(),
      2000,
      "Drop transformed point (frame %s): map_xy=(%.3f, %.3f)m raw_cm=(%ld, %ld) "
      "encoded_cm=(%ld, %ld) out of UInt16 range",
      resolved_source_frame.c_str(),
      point_map.x,
      point_map.y,
      x_cm,
      y_cm,
      encoded_x_cm,
      encoded_y_cm);
    return;
  }

  std_msgs::msg::UInt16MultiArray out;
  out.data = {
    static_cast<std::uint16_t>(encoded_x_cm),
    static_cast<std::uint16_t>(encoded_y_cm)};
  publishers.goal_pos->publish(out);
}

void GoalOutput::publishMapPointAsGoalPose(
  const geometry_msgs::msg::Point & point_map,
  const rclcpp::Time & stamp,
  rclcpp::Node & node,
  const Publishers & publishers) const
{
  if (!config_.publish_goal_pose || !publishers.goal_pose) {
    return;
  }

  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = config_.map_frame;
  out.header.stamp = (stamp.nanoseconds() == 0) ? node.now() : stamp;
  out.pose.position = point_map;
  out.pose.orientation.w = 1.0;
  publishers.goal_pose->publish(out);
}

}  // namespace navi_tf_bridge
