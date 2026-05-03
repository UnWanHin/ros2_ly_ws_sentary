#include "navi_tf_bridge/pointer_debug.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <system_error>
#include <utility>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace navi_tf_bridge
{

PointerDebug::PointerDebug(Config config)
: config_(std::move(config))
{
}

void PointerDebug::setConfig(Config config)
{
  config_ = std::move(config);
  tf_ready_logged_once_ = false;
  tf_missing_logged_once_ = false;
  exported_ = false;
}

const PointerDebug::Config & PointerDebug::config() const
{
  return config_;
}

bool PointerDebug::parseAreaHeaderPoints(
  rclcpp::Node & node,
  std::vector<NamedPointCm> & points_out) const
{
  points_out.clear();
  if (config_.area_header_file.empty()) {
    return false;
  }

  std::ifstream ifs(config_.area_header_file);
  if (!ifs.is_open()) {
    RCLCPP_WARN(
      node.get_logger(),
      "TF debug export: cannot open area header file: %s",
      config_.area_header_file.c_str());
    return false;
  }

  std::stringstream buffer;
  buffer << ifs.rdbuf();
  const std::string content = buffer.str();

  // Match lines like:
  // Location<std::uint16_t> Home{ {393, 810}, {2408, 683} };
  const std::regex pattern(
    R"(Location<\s*std::uint16_t\s*>\s*([A-Za-z0-9_]+)\s*\{\s*\{\s*([0-9]+)\s*,\s*([0-9]+)\s*\}\s*,\s*\{\s*([0-9]+)\s*,\s*([0-9]+)\s*\}\s*\}\s*;)");

  std::sregex_iterator it(content.begin(), content.end(), pattern);
  std::sregex_iterator end;
  for (; it != end; ++it) {
    const auto & m = *it;
    const std::string name = m[1].str();
    const int red_x = std::stoi(m[2].str());
    const int red_y = std::stoi(m[3].str());
    const int blue_x = std::stoi(m[4].str());
    const int blue_y = std::stoi(m[5].str());
    points_out.push_back(NamedPointCm{name + ".red", red_x, red_y});
    points_out.push_back(NamedPointCm{name + ".blue", blue_x, blue_y});
  }

  if (points_out.empty()) {
    RCLCPP_WARN(
      node.get_logger(),
      "TF debug export: no Location<std::uint16_t> points found in %s",
      config_.area_header_file.c_str());
    return false;
  }

  return true;
}

bool PointerDebug::lookupTransformWithCandidates(
  const std::vector<std::string> & source_candidates,
  const rclcpp::Time & transform_time,
  tf2_ros::Buffer & tf_buffer,
  geometry_msgs::msg::TransformStamped & tf_out,
  std::string & resolved_source_frame,
  std::string & last_tf_error) const
{
  for (const auto & source_frame : source_candidates) {
    try {
      tf_out = tf_buffer.lookupTransform(
        config_.map_frame,
        source_frame,
        transform_time,
        rclcpp::Duration::from_seconds(0.05));
      resolved_source_frame = source_frame;
      return true;
    } catch (const tf2::TransformException & ex) {
      last_tf_error = ex.what();
    }
  }
  return false;
}

bool PointerDebug::exportPointPairs(
  const std::string & tf_ready_source_frame,
  const geometry_msgs::msg::TransformStamped & tf_map_ref,
  rclcpp::Node & node)
{
  std::vector<NamedPointCm> points_cm;
  if (!parseAreaHeaderPoints(node, points_cm)) {
    return false;
  }
  if (config_.output_file.empty()) {
    RCLCPP_WARN(node.get_logger(), "TF debug export: debug_point_pairs_output_file is empty.");
    return false;
  }

  std::filesystem::path out_path(config_.output_file);
  if (out_path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(out_path.parent_path(), ec);
    if (ec) {
      RCLCPP_WARN(
        node.get_logger(),
        "TF debug export: failed to create parent dir for %s: %s",
        config_.output_file.c_str(),
        ec.message().c_str());
      return false;
    }
  }

  std::ofstream ofs(config_.output_file, std::ios::out | std::ios::trunc);
  if (!ofs.is_open()) {
    RCLCPP_WARN(
      node.get_logger(),
      "TF debug export: failed to open output file: %s",
      config_.output_file.c_str());
    return false;
  }

  ofs << "meta:\n";
  ofs << "  generated_ns: " << node.now().nanoseconds() << "\n";
  ofs << "  map_frame: " << config_.map_frame << "\n";
  ofs << "  tf_ready_source_frame: " << tf_ready_source_frame << "\n";
  ofs << "  reference_frame: " << config_.reference_frame << "\n";
  ofs << "  area_header_file: " << config_.area_header_file << "\n";
  ofs << "points:\n";

  for (const auto & point : points_cm) {
    geometry_msgs::msg::PointStamped in;
    in.header.frame_id = config_.reference_frame;
    in.header.stamp = tf_map_ref.header.stamp;
    in.point.x = static_cast<double>(point.x_cm) * 0.01;
    in.point.y = static_cast<double>(point.y_cm) * 0.01;
    in.point.z = 0.0;

    geometry_msgs::msg::PointStamped out;
    tf2::doTransform(in, out, tf_map_ref);
    const long tx_cm = std::lround(out.point.x * 100.0);
    const long ty_cm = std::lround(out.point.y * 100.0);
    const long dx_cm = tx_cm - static_cast<long>(point.x_cm);
    const long dy_cm = ty_cm - static_cast<long>(point.y_cm);

    ofs << "  - name: " << point.name << "\n";
    ofs << "    original_cm: {x: " << point.x_cm << ", y: " << point.y_cm << "}\n";
    ofs << "    transformed_cm: {x: " << tx_cm << ", y: " << ty_cm << "}\n";
    ofs << "    delta_cm: {dx: " << dx_cm << ", dy: " << dy_cm << "}\n";
  }

  return true;
}

bool PointerDebug::onTimer(
  const std::vector<std::string> & source_candidates,
  tf2_ros::Buffer & tf_buffer,
  rclcpp::Node & node)
{
  if (!config_.enabled || exported_) {
    return true;
  }

  geometry_msgs::msg::TransformStamped tf_map_source;
  std::string last_tf_error;
  std::string resolved_source_frame;
  const rclcpp::Time latest_time(0, 0, node.get_clock()->get_clock_type());
  if (!lookupTransformWithCandidates(
      source_candidates, latest_time, tf_buffer, tf_map_source, resolved_source_frame,
      last_tf_error))
  {
    if (!tf_missing_logged_once_) {
      std::ostringstream oss;
      for (const auto & frame : source_candidates) {
        oss << frame << " ";
      }
      RCLCPP_WARN(
        node.get_logger(),
        "TF frame chain not ready yet (%s <- [%s]). Last error: %s",
        config_.map_frame.c_str(),
        oss.str().c_str(),
        last_tf_error.c_str());
      tf_missing_logged_once_ = true;
    }
    return false;
  }

  if (!tf_ready_logged_once_) {
    RCLCPP_INFO(
      node.get_logger(),
      "TF frame chain ready: %s <- %s",
      config_.map_frame.c_str(),
      resolved_source_frame.c_str());
    tf_ready_logged_once_ = true;
  }

  geometry_msgs::msg::TransformStamped tf_map_ref;
  std::string tf_ref_error;
  std::string tf_ref_resolved;
  if (!lookupTransformWithCandidates(
      {config_.reference_frame}, latest_time, tf_buffer, tf_map_ref, tf_ref_resolved,
      tf_ref_error))
  {
    RCLCPP_WARN(
      node.get_logger(),
      "TF debug export: failed to resolve reference frame (%s <- %s): %s",
      config_.map_frame.c_str(),
      config_.reference_frame.c_str(),
      tf_ref_error.c_str());
    return true;
  }

  if (exportPointPairs(resolved_source_frame, tf_map_ref, node)) {
    exported_ = true;
    RCLCPP_INFO(
      node.get_logger(),
      "TF debug point pairs exported: %s",
      config_.output_file.c_str());
  }
  return true;
}

}  // namespace navi_tf_bridge
