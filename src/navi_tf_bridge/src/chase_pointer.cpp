#include "navi_tf_bridge/chase_pointer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace navi_tf_bridge
{

ChasePointer::ChasePointer(Config config)
: config_(std::move(config))
{
}

void ChasePointer::setConfig(Config config)
{
  config_ = std::move(config);
}

const ChasePointer::Config & ChasePointer::config() const
{
  return config_;
}

void ChasePointer::appendUniqueFrame(
  std::vector<std::string> & frames,
  const std::string & frame)
{
  if (frame.empty()) {
    return;
  }
  if (std::find(frames.begin(), frames.end(), frame) == frames.end()) {
    frames.push_back(frame);
  }
}

std::vector<std::string> ChasePointer::buildSourceCandidates(
  const std::string & msg_frame_id) const
{
  const bool has_explicit_source_frame = config_.use_msg_frame_id && !msg_frame_id.empty();
  std::vector<std::string> source_candidates;
  source_candidates.reserve(2);

  if (has_explicit_source_frame) {
    appendUniqueFrame(source_candidates, msg_frame_id);
    return source_candidates;
  }

  appendUniqueFrame(source_candidates, config_.default_frame);
  // Compatibility fallback: if no explicit frame and default frame is missing, treat target_rel
  // as already in base frame.
  appendUniqueFrame(source_candidates, config_.base_frame);
  return source_candidates;
}

bool ChasePointer::buildRelativeGoalPoint(
  const auto_aim_common::msg::RelativeTarget & msg,
  rclcpp::Node & node,
  geometry_msgs::msg::Point & point_out) const
{
  point_out.x = 0.0;
  point_out.y = 0.0;
  point_out.z = 0.0;

  if (!msg.valid) {
    return config_.stop_when_no_target;
  }

  const double target_x = static_cast<double>(msg.x);
  const double target_y = static_cast<double>(msg.y);
  if (!std::isfinite(target_x) || !std::isfinite(target_y)) {
    RCLCPP_WARN_THROTTLE(
      node.get_logger(),
      *node.get_clock(),
      2000,
      "Drop invalid target_rel: x=%.3f y=%.3f",
      target_x,
      target_y);
    return false;
  }

  const double planar_distance_m = std::hypot(target_x, target_y);
  constexpr double kMinPlanarDistanceM = 1e-6;
  if (planar_distance_m <= kMinPlanarDistanceM) {
    return true;
  }

  const double preferred_distance_m = static_cast<double>(config_.preferred_distance_cm) * 0.01;
  const double deadband_m = static_cast<double>(config_.distance_deadband_cm) * 0.01;
  const double move_distance_m = planar_distance_m - preferred_distance_m;

  if (std::abs(move_distance_m) <= deadband_m) {
    return true;
  }

  if (move_distance_m < 0.0 && !config_.allow_reverse_goal) {
    return true;
  }

  const double scale = move_distance_m / planar_distance_m;
  point_out.x = target_x * scale;
  point_out.y = target_y * scale;
  return true;
}

bool ChasePointer::transformToMap(
  const geometry_msgs::msg::Point & point_rel,
  const std::vector<std::string> & source_candidates,
  const rclcpp::Time & transform_time,
  tf2_ros::Buffer & tf_buffer,
  geometry_msgs::msg::PointStamped & point_map,
  std::string & resolved_source_frame,
  std::string & last_tf_error) const
{
  geometry_msgs::msg::PointStamped point_source;
  point_source.header.stamp = transform_time;
  point_source.point = point_rel;

  for (const auto & source_frame : source_candidates) {
    point_source.header.frame_id = source_frame;
    try {
      const geometry_msgs::msg::TransformStamped tf_map_source =
        tf_buffer.lookupTransform(
        config_.map_frame,
        source_frame,
        transform_time,
        rclcpp::Duration::from_seconds(0.05));
      tf2::doTransform(point_source, point_map, tf_map_source);
      resolved_source_frame = source_frame;
      return true;
    } catch (const tf2::TransformException & ex) {
      last_tf_error =
        "lookup " + config_.map_frame + " <- " + source_frame + " failed: " + ex.what();
    }
  }

  return false;
}

}  // namespace navi_tf_bridge
