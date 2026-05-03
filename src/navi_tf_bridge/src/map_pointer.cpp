#include "navi_tf_bridge/map_pointer.hpp"

#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace navi_tf_bridge
{

MapPointer::MapPointer(Config config)
: config_(std::move(config)),
  solver_(config_.solver)
{
}

void MapPointer::setConfig(Config config)
{
  config_ = std::move(config);
  solver_.setConfig(config_.solver);
  raw_tf_ready_logged_once_ = false;
  raw_tf_missing_logged_once_ = false;
}

const MapPointer::Config & MapPointer::config() const
{
  return config_;
}

void MapPointer::initialize(rclcpp::Node & node)
{
  solver_.setConfig(config_.solver);
  if (config_.use_static_calibration) {
    solver_.initialize(node);
  }
}

bool MapPointer::toMap(
  const std_msgs::msg::UInt16MultiArray & msg,
  tf2_ros::Buffer & tf_buffer,
  rclcpp::Node & node,
  geometry_msgs::msg::PointStamped & point_map,
  std::string & source_name)
{
  if (!config_.enabled) {
    return false;
  }
  if (msg.data.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      node.get_logger(),
      *node.get_clock(),
      2000,
      "Drop invalid goal_pos_raw: data size=%zu (need >=2)",
      msg.data.size());
    return false;
  }

  if (config_.use_static_calibration) {
    if (!solver_.ready()) {
      RCLCPP_WARN_THROTTLE(
        node.get_logger(),
        *node.get_clock(),
        2000,
        "Drop goal_pos_raw because static calibration is enabled but not ready.");
      return false;
    }

    const double raw_x_m = static_cast<double>(msg.data[0]) * 0.01;
    const double raw_y_m = static_cast<double>(msg.data[1]) * 0.01;
    point_map.header.frame_id = config_.map_frame;
    point_map.header.stamp = node.now();
    point_map.point = solver_.applyMeters(raw_x_m, raw_y_m, 0.0);
    source_name = "raw_goal_static_calibration";
    return true;
  }

  geometry_msgs::msg::PointStamped point_in;
  point_in.header.frame_id = config_.raw_frame;
  point_in.header.stamp = node.now();
  point_in.point.x = static_cast<double>(msg.data[0]) * 0.01;
  point_in.point.y = static_cast<double>(msg.data[1]) * 0.01;
  point_in.point.z = 0.0;

  if (config_.raw_frame == config_.map_frame) {
    point_map = point_in;
    source_name = config_.raw_frame;
    return true;
  }

  try {
    const geometry_msgs::msg::TransformStamped tf_map_raw =
      tf_buffer.lookupTransform(
      config_.map_frame,
      config_.raw_frame,
      rclcpp::Time(0, 0, node.get_clock()->get_clock_type()),
      rclcpp::Duration::from_seconds(0.05));
    tf2::doTransform(point_in, point_map, tf_map_raw);
    source_name = config_.raw_frame;
    if (!raw_tf_ready_logged_once_) {
      RCLCPP_INFO(
        node.get_logger(),
        "Raw goal TF chain ready: %s <- %s",
        config_.map_frame.c_str(),
        config_.raw_frame.c_str());
      raw_tf_ready_logged_once_ = true;
    }
    return true;
  } catch (const tf2::TransformException & ex) {
    if (!raw_tf_missing_logged_once_) {
      RCLCPP_WARN(
        node.get_logger(),
        "Raw goal TF chain not ready (%s <- %s): %s",
        config_.map_frame.c_str(),
        config_.raw_frame.c_str(),
        ex.what());
      raw_tf_missing_logged_once_ = true;
    }
    return false;
  }
}

}  // namespace navi_tf_bridge
