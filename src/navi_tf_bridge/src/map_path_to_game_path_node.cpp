#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gimbal_driver/msg/map_path.hpp"
#include "gimbal_driver/msg/sentry_info.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navi_tf_bridge/map_pointer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace navi_tf_bridge
{

class MapPathToGamePathNode final : public rclcpp::Node
{
public:
  MapPathToGamePathNode()
  : Node("map_path_to_game_path_node")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/ly/navi/path");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/ly/game/path");
    const std::string map_frame = this->declare_parameter<std::string>("map_frame", "map");
    intention_ = static_cast<std::uint8_t>(std::clamp<std::int64_t>(
      this->declare_parameter<std::int64_t>("intention", 3), 1, 3));
    sentry_info_topic_ = this->declare_parameter<std::string>(
      "sentry_info_topic", "/ly/game/sentry/info");

    const bool use_static_calibration =
      this->declare_parameter<bool>("use_raw_goal_static_calibration", true);
    const std::string calibration_model =
      this->declare_parameter<std::string>("raw_goal_calibration_model", "matrix");
    const std::string calibration_unit =
      this->declare_parameter<std::string>("raw_goal_calibration_unit", "m");
    const std::string source_frame =
      this->declare_parameter<std::string>("raw_goal_source_frame", "official_map");
    const std::string target_frame =
      this->declare_parameter<std::string>("raw_goal_target_frame", map_frame);
    const std::vector<double> source_points = this->declare_parameter<std::vector<double>>(
      "raw_goal_source_points", std::vector<double>{});
    const std::vector<double> target_points = this->declare_parameter<std::vector<double>>(
      "raw_goal_target_points", std::vector<double>{});
    const std::vector<double> transform_matrix = this->declare_parameter<std::vector<double>>(
      "raw_goal_transform_matrix", std::vector<double>{});

    map_pointer_.setConfig(MapPointer::Config{
      .enabled = true,
      .raw_frame = source_frame,
      .map_frame = target_frame,
      .use_static_calibration = use_static_calibration,
      .solver = PointerSolver::Config{
        .enabled = use_static_calibration,
        .model = calibration_model,
        .unit = calibration_unit,
        .source_frame = source_frame,
        .target_frame = target_frame,
        .source_points = source_points,
        .target_points = target_points,
        .transform_matrix = transform_matrix}});
    map_pointer_.initialize(*this);

    if (!use_static_calibration) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Map path bridge requires use_raw_goal_static_calibration=true: map -> official inverse matrix is required.");
    }
    if (target_frame != map_frame) {
      RCLCPP_WARN(
        this->get_logger(),
        "map_frame=%s differs from raw_goal_target_frame=%s; input Path header must use %s.",
        map_frame.c_str(), target_frame.c_str(), target_frame.c_str());
    }
    expected_map_frame_ = target_frame;

    publisher_ = this->create_publisher<gimbal_driver::msg::MapPath>(output_topic_, rclcpp::QoS(10));
    subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&MapPathToGamePathNode::onPath, this, std::placeholders::_1));
    sentry_info_subscription_ = this->create_subscription<gimbal_driver::msg::SentryInfo>(
      sentry_info_topic_, rclcpp::QoS(10),
      [this](const gimbal_driver::msg::SentryInfo::SharedPtr msg) {
        if (!msg) {
          return;
        }
        self_robot_id_ = msg->self_robot_id;
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Map path bridge ready: %s (nav_msgs/Path, %s/m) -> %s (official dm, intention=%u, sender from %s.self_robot_id).",
      input_topic_.c_str(), expected_map_frame_.c_str(), output_topic_.c_str(), intention_, sentry_info_topic_.c_str());
  }

private:
  static constexpr std::size_t kMaxPoints = 50;

  bool toOfficialDecimeters(
    const geometry_msgs::msg::Point & point_map,
    std::int32_t & x_dm,
    std::int32_t & y_dm) const
  {
    if (!std::isfinite(point_map.x) || !std::isfinite(point_map.y)) {
      return false;
    }
    double raw_x_cm = 0.0;
    double raw_y_cm = 0.0;
    if (!map_pointer_.mapToRawCentimeters(point_map, raw_x_cm, raw_y_cm)) {
      return false;
    }
    x_dm = static_cast<std::int32_t>(std::lround(raw_x_cm / 10.0));
    y_dm = static_cast<std::int32_t>(std::lround(raw_y_cm / 10.0));
    return x_dm >= 0 && x_dm <= 65535 && y_dm >= 0 && y_dm <= 65535;
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg || msg->poses.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Drop empty /ly/navi/path.");
      return;
    }
    if (self_robot_id_ == 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Drop /ly/navi/path: waiting for %s.self_robot_id.",
        sentry_info_topic_.c_str());
      return;
    }
    if (!msg->header.frame_id.empty() && msg->header.frame_id != expected_map_frame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Drop /ly/navi/path frame '%s': expected '%s'.",
        msg->header.frame_id.c_str(), expected_map_frame_.c_str());
      return;
    }

    const std::size_t point_count = std::min(msg->poses.size(), kMaxPoints);
    std::array<std::int32_t, kMaxPoints> x_dm{};
    std::array<std::int32_t, kMaxPoints> y_dm{};
    for (std::size_t index = 0; index < point_count; ++index) {
      if (!toOfficialDecimeters(msg->poses[index].pose.position, x_dm[index], y_dm[index])) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Drop /ly/navi/path: point %zu cannot convert to representable official-map dm.", index);
        return;
      }
    }

    gimbal_driver::msg::MapPath output;
    output.header = msg->header;
    output.intention = intention_;
    output.start_position_x_dm = static_cast<std::uint16_t>(x_dm[0]);
    output.start_position_y_dm = static_cast<std::uint16_t>(y_dm[0]);
    output.sender_id = self_robot_id_;

    for (std::size_t index = 1; index < point_count; ++index) {
      const std::int32_t delta_x = x_dm[index] - x_dm[index - 1];
      const std::int32_t delta_y = y_dm[index] - y_dm[index - 1];
      if (delta_x < -128 || delta_x > 127 || delta_y < -128 || delta_y > 127) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Drop /ly/navi/path: delta %zu is (%d, %d) dm outside int8 range.",
          index, delta_x, delta_y);
        return;
      }
      output.delta_x_dm[index - 1] = static_cast<std::int8_t>(delta_x);
      output.delta_y_dm[index - 1] = static_cast<std::int8_t>(delta_y);
    }

    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string sentry_info_topic_;
  std::string expected_map_frame_;
  std::uint8_t intention_{3};
  std::uint16_t self_robot_id_{0};
  MapPointer map_pointer_;
  rclcpp::Publisher<gimbal_driver::msg::MapPath>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
  rclcpp::Subscription<gimbal_driver::msg::SentryInfo>::SharedPtr sentry_info_subscription_;
};

}  // namespace navi_tf_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navi_tf_bridge::MapPathToGamePathNode>());
  rclcpp::shutdown();
  return 0;
}
