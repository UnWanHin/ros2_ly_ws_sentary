#include <cmath>
#include <algorithm>
#include <chrono>
#include <functional>
#include <fstream>
#include <limits>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "gimbal_driver/msg/fire_code.hpp"
#include "gimbal_driver/msg/gimbal_angles.hpp"
#include "navi_tf_bridge/pointer_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navi_tf_bridge
{
namespace
{

std::string trim(std::string text)
{
  const auto begin = text.find_first_not_of(" \t\r\n\"'");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t\r\n\"'");
  return text.substr(begin, end - begin + 1);
}

std::string readScalar(const std::string & content, const std::string & key, const std::string & fallback)
{
  const std::regex pattern("(^|\\n)\\s*" + key + R"(\s*:\s*([^#\r\n]+))");
  std::smatch match;
  if (std::regex_search(content, match, pattern) && match.size() >= 3) {
    return trim(match[2].str());
  }
  return fallback;
}

std::vector<double> readNumberList(const std::string & content, const std::string & key)
{
  const auto key_pos = content.find(key);
  if (key_pos == std::string::npos) {
    return {};
  }
  const auto begin = content.find('[', key_pos);
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = content.find(']', begin);
  if (end == std::string::npos || end <= begin) {
    return {};
  }

  const std::string list_text = content.substr(begin + 1, end - begin - 1);
  const std::regex number_pattern(R"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)");
  std::vector<double> values;
  for (
    std::sregex_iterator it(list_text.begin(), list_text.end(), number_pattern), last;
    it != last;
    ++it)
  {
    values.push_back(std::stod(it->str()));
  }
  return values;
}

}  // namespace

class FaceModeNode : public rclcpp::Node
{
public:
  FaceModeNode()
  : Node("map_aim_point_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    official_map_x_m_ = declareRequiredCentimeterParameter("official_map_x");
    official_map_y_m_ = declareRequiredCentimeterParameter("official_map_y");
    map_z_m_ = declareRequiredCentimeterParameter("map_z");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "official_map");
    aim_frame_ = this->declare_parameter<std::string>("aim_frame", "gimbal_world");
    camera_frame_ = this->declare_parameter<std::string>("camera_frame", "gx_camera");
    solve_mode_ = this->declare_parameter<std::string>("solve_mode", "camera_projection");
    solve_frame_ = this->declare_parameter<std::string>("solve_frame", "base_link");

    const std::string gimbal_topic =
      this->declare_parameter<std::string>("gimbal_angles_topic", "/ly/gimbal/angles");
    const std::string control_topic =
      this->declare_parameter<std::string>("control_angles_topic", "/ly/control/angles");
    const std::string firecode_topic =
      this->declare_parameter<std::string>("control_firecode_topic", "/ly/control/firecode");
    const std::string face_target_topic =
      this->declare_parameter<std::string>("face_target_topic", "/ly/face_mode/target_raw");
    publish_firecode_ = this->declare_parameter<bool>("publish_firecode", true);
    aim_mode_ = this->declare_parameter<bool>("aim_mode", true);

    const std::string bridge_config_file =
      this->declare_parameter<std::string>("bridge_config_file", "");
    const bool use_static_calibration =
      this->declare_parameter<bool>("use_raw_goal_static_calibration", false);
    raw_goal_target_frame_override_ =
      trim(this->declare_parameter<std::string>("raw_goal_target_frame", ""));

    const double publish_hz = std::max(1.0, this->declare_parameter<double>("publish_hz", 30.0));
    tf_timeout_ = rclcpp::Duration::from_seconds(
      std::max(0.01, this->declare_parameter<double>("tf_timeout_sec", 0.05)));
    use_gimbal_stamp_for_tf_ =
      this->declare_parameter<bool>("use_gimbal_stamp_for_tf", false);
    max_gimbal_stamp_age_sec_ =
      std::max(0.0, this->declare_parameter<double>("max_gimbal_stamp_age_sec", 0.50));
    min_distance_m_ = std::max(0.01, this->declare_parameter<double>("min_distance_m", 0.10));
    max_target_distance_m_ =
      std::max(0.0, this->declare_parameter<double>("max_target_distance_m", 100.0));
    command_filter_alpha_ =
      std::clamp(this->declare_parameter<double>("command_filter_alpha", 1.0), 0.0, 1.0);
    yaw_sign_ = this->declare_parameter<double>("yaw_sign", -1.0);
    pitch_sign_ = this->declare_parameter<double>("pitch_sign", 1.0);
    yaw_bias_deg_ = this->declare_parameter<double>("yaw_bias_deg", 0.0);
    pitch_bias_deg_ = this->declare_parameter<double>("pitch_bias_deg", 0.0);
    max_yaw_step_deg_ =
      std::max(0.0, this->declare_parameter<double>("max_yaw_step_deg", 0.0));
    max_pitch_step_deg_ =
      std::max(0.0, this->declare_parameter<double>("max_pitch_step_deg", 0.0));

    refreshActiveTargetFromOfficial("initial");
    if (use_static_calibration) {
      loadRawGoalStaticCalibration(bridge_config_file);
    }

    pub_angles_ = this->create_publisher<gimbal_driver::msg::GimbalAngles>(control_topic, 10);
    if (publish_firecode_) {
      pub_firecode_ = this->create_publisher<gimbal_driver::msg::FireCode>(firecode_topic, 10);
    }
    sub_angles_ = this->create_subscription<gimbal_driver::msg::GimbalAngles>(
      gimbal_topic,
      20,
      std::bind(&FaceModeNode::onGimbalAngles, this, std::placeholders::_1));
    sub_face_target_raw_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      face_target_topic,
      10,
      std::bind(&FaceModeNode::onFaceTargetRaw, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz),
      std::bind(&FaceModeNode::onTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "FaceMode started: raw_target=(%.3f, %.3f, %.3f)m@%s active_target=(%.3f, %.3f, %.3f)m@%s "
      "solve_mode=%s solve_frame=%s aim_frame=%s camera_frame=%s -> %s, gimbal=%s, firecode=%s, "
      "use_gimbal_stamp_for_tf=%s, command_filter_alpha=%.2f",
      official_map_x_m_,
      official_map_y_m_,
      map_z_m_,
      target_frame_.c_str(),
      active_target_point_.x,
      active_target_point_.y,
      active_target_point_.z,
      active_target_frame_.c_str(),
      solve_mode_.c_str(),
      solve_frame_.c_str(),
      aim_frame_.c_str(),
      camera_frame_.c_str(),
      control_topic.c_str(),
      gimbal_topic.c_str(),
      publish_firecode_ ? "on" : "off",
      use_gimbal_stamp_for_tf_ ? "true" : "false",
      command_filter_alpha_);
  }

private:
  double declareRequiredCentimeterParameter(const std::string & name)
  {
    try {
      return this->declare_parameter<double>(name) * 0.01;
    } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
      throw std::runtime_error(
        "FaceMode requires parameter '" + name +
        "' in cm. Pass official_map_x:=... official_map_y:=... map_z:=...");
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & ex) {
      throw std::runtime_error(
        "FaceMode parameter '" + name + "' must be a number in cm: " + ex.what());
    }
  }

  void loadRawGoalStaticCalibration(const std::string & bridge_config_file)
  {
    if (bridge_config_file.empty()) {
      RCLCPP_WARN(this->get_logger(), "bridge_config_file is empty; raw-goal static calibration disabled");
      return;
    }

    std::ifstream ifs(bridge_config_file);
    if (!ifs.is_open()) {
      RCLCPP_WARN(
        this->get_logger(),
        "failed to load bridge_config_file '%s'; raw-goal static calibration disabled",
        bridge_config_file.c_str());
      return;
    }
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string content = buffer.str();

    const std::string model = readScalar(content, "raw_goal_calibration_model", "matrix");
    if (!(model == "matrix" || model == "MATRIX" || model == "matrix_4x4")) {
      RCLCPP_WARN(
        this->get_logger(),
        "map aim point only supports raw_goal_calibration_model=matrix, got '%s'; raw-goal static calibration disabled",
        model.c_str());
      return;
    }

    const std::vector<double> matrix = readNumberList(content, "raw_goal_transform_matrix");
    if (matrix.size() != 16) {
      RCLCPP_WARN(
        this->get_logger(),
        "raw_goal_transform_matrix in '%s' must contain 16 values; raw-goal static calibration disabled",
        bridge_config_file.c_str());
      return;
    }

    const std::string configured_target_frame =
      readScalar(content, "raw_goal_target_frame", "map");
    raw_goal_solver_.setConfig(PointerSolver::Config{
      .enabled = true,
      .model = model,
      .unit = readScalar(content, "raw_goal_calibration_unit", "m"),
      .source_frame = readScalar(content, "raw_goal_source_frame", "official_map"),
      .target_frame = configured_target_frame,
      .map_frame = configured_target_frame,
      .transform_matrix = matrix});
    if (!raw_goal_solver_.initialize(*this)) {
      return;
    }

    raw_goal_static_calibration_ready_ = true;
    raw_goal_configured_target_frame_ = configured_target_frame;
    refreshActiveTargetFromOfficial("raw-goal static calibration loaded");
  }

  void refreshActiveTargetFromOfficial(const std::string & reason)
  {
    if (raw_goal_static_calibration_ready_ && raw_goal_solver_.ready()) {
      active_target_point_ =
        raw_goal_solver_.applyMeters(official_map_x_m_, official_map_y_m_, map_z_m_);
      active_target_frame_ =
        raw_goal_target_frame_override_.empty() ?
        raw_goal_configured_target_frame_ : raw_goal_target_frame_override_;
    } else {
      active_target_frame_ = target_frame_;
      active_target_point_.x = official_map_x_m_;
      active_target_point_.y = official_map_y_m_;
      active_target_point_.z = map_z_m_;
    }
    last_yaw_cmd_deg_.reset();
    last_pitch_cmd_deg_.reset();
    RCLCPP_INFO(
      this->get_logger(),
      "FaceMode target updated (%s): official=(%.3f, %.3f, %.3f)m -> %s target=(%.3f, %.3f, %.3f)m",
      reason.c_str(),
      official_map_x_m_,
      official_map_y_m_,
      map_z_m_,
      active_target_frame_.c_str(),
      active_target_point_.x,
      active_target_point_.y,
      active_target_point_.z);
  }

  void onGimbalAngles(const gimbal_driver::msg::GimbalAngles::SharedPtr msg)
  {
    current_angles_ = *msg;
  }

  void onFaceTargetRaw(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() < 3) {
      warnThrottled("drop invalid FaceMode raw target: need [official_map_x, official_map_y, map_z] cm");
      return;
    }
    const double next_x_m = static_cast<double>(msg->data[0]) * 0.01;
    const double next_y_m = static_cast<double>(msg->data[1]) * 0.01;
    const double next_z_m = static_cast<double>(msg->data[2]) * 0.01;
    if (std::abs(next_x_m - official_map_x_m_) < 1e-9 &&
        std::abs(next_y_m - official_map_y_m_) < 1e-9 &&
        std::abs(next_z_m - map_z_m_) < 1e-9) {
      return;
    }
    official_map_x_m_ = next_x_m;
    official_map_y_m_ = next_y_m;
    map_z_m_ = next_z_m;
    refreshActiveTargetFromOfficial("topic /ly/face_mode/target_raw");
  }

  bool stampIsZero(const gimbal_driver::msg::GimbalAngles & msg) const
  {
    return msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0;
  }

  rclcpp::Time selectTfLookupTime(const gimbal_driver::msg::GimbalAngles & msg)
  {
    if (!use_gimbal_stamp_for_tf_ || stampIsZero(msg)) {
      return rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    }

    const rclcpp::Time stamp(msg.header.stamp, this->get_clock()->get_clock_type());
    if (max_gimbal_stamp_age_sec_ > 0.0) {
      const double age_sec = (this->now().nanoseconds() - stamp.nanoseconds()) * 1e-9;
      if (age_sec > max_gimbal_stamp_age_sec_) {
        warnThrottled(
          "gimbal angle stamp is stale (" + std::to_string(age_sec) +
          "s old); skip map aim command");
        throw std::runtime_error("stale gimbal stamp");
      }
    }
    return stamp;
  }

  void warnThrottled(const std::string & text)
  {
    const int64_t now_ns = this->now().nanoseconds();
    if (now_ns - last_warn_ns_ > static_cast<int64_t>(2e9)) {
      RCLCPP_WARN(this->get_logger(), "%s", text.c_str());
      last_warn_ns_ = now_ns;
    }
  }

  static double normalizeNear(const double target_deg, const double reference_deg)
  {
    return reference_deg + std::remainder(target_deg - reference_deg, 360.0);
  }

  static double limitStep(const double target_deg, const double current_deg, const double max_step_deg)
  {
    if (max_step_deg <= 0.0) {
      return target_deg;
    }
    double delta = std::remainder(target_deg - current_deg, 360.0);
    delta = std::max(-max_step_deg, std::min(max_step_deg, delta));
    return current_deg + delta;
  }

  static double lowpassAngle(const double target_deg, const double previous_deg, const double alpha)
  {
    const double target_near = previous_deg + std::remainder(target_deg - previous_deg, 360.0);
    if (alpha <= 0.0 || alpha >= 1.0) {
      return target_near;
    }
    return previous_deg + alpha * (target_near - previous_deg);
  }

  std::optional<geometry_msgs::msg::Point> lookupTargetInFrame(
    const std::string & frame,
    const rclcpp::Time & lookup_time,
    const std::string & missing_hint)
  {
    geometry_msgs::msg::PointStamped point_in;
    point_in.header.frame_id = active_target_frame_;
    point_in.header.stamp = lookup_time;
    point_in.point = active_target_point_;

    try {
      const auto tf_msg = tf_buffer_.lookupTransform(
        frame,
        active_target_frame_,
        lookup_time,
        tf_timeout_);
      geometry_msgs::msg::PointStamped point_out;
      tf2::doTransform(point_in, point_out, tf_msg);
      return point_out.point;
    } catch (const tf2::TransformException & ex) {
      std::string hint;
      const std::string exc_text = ex.what();
      if (exc_text.find(active_target_frame_) != std::string::npos) {
        hint =
          "; frame '" + active_target_frame_ +
          "' is absent. Start navigation/localization that publishes map -> base_link, "
          "or use use_mock_map_to_base:=true for bench tests";
      } else if (exc_text.find(frame) != std::string::npos) {
        hint = missing_hint;
      }
      warnThrottled("TF not ready: " + frame + " <- " + active_target_frame_ + ": " + ex.what() + hint);
      return std::nullopt;
    }
  }

  struct SolvedCommand
  {
    double yaw_deg{0.0};
    double pitch_deg{0.0};
    geometry_msgs::msg::Point target{};
    std::string detail{};
  };

  std::optional<SolvedCommand> solveBaseLinkAngles(
    const rclcpp::Time & lookup_time,
    const double current_yaw_deg,
    const double current_pitch_deg)
  {
    const auto target = lookupTargetInFrame(
      solve_frame_,
      lookup_time,
      "; frame '" + solve_frame_ + "' is absent. Start localization/TF and make sure base_link is connected to map");
    if (!target) {
      return std::nullopt;
    }

    const double horizontal = std::hypot(target->x, target->y);
    const double distance = std::hypot(horizontal, target->z);
    if (distance < min_distance_m_) {
      warnThrottled(
        "target too close in " + solve_frame_ + ": (" + std::to_string(target->x) + ", " +
        std::to_string(target->y) + ", " + std::to_string(target->z) + ")m");
      return std::nullopt;
    }
    if (max_target_distance_m_ > 0.0 && distance > max_target_distance_m_) {
      warnThrottled(
        "target distance in " + solve_frame_ + " is unreasonable (" +
        std::to_string(distance) + "m); skip map aim command. Check odom/localization TF.");
      return std::nullopt;
    }

    const double target_yaw_deg =
      yaw_sign_ * std::atan2(target->y, target->x) * 180.0 / M_PI + yaw_bias_deg_;
    const double pitch_cmd_deg =
      pitch_sign_ * std::atan2(target->z, horizontal) * 180.0 / M_PI + pitch_bias_deg_;
    const double yaw_cmd_deg = normalizeNear(target_yaw_deg, current_yaw_deg);
    std::ostringstream detail;
    detail << "target_in_" << solve_frame_ << "=(" << target->x << "," << target->y << ","
           << target->z << ")m target_yaw=" << target_yaw_deg;
    (void)current_pitch_deg;
    return SolvedCommand{yaw_cmd_deg, pitch_cmd_deg, *target, detail.str()};
  }

  std::optional<SolvedCommand> solveRelativeGeometryAngles(
    const rclcpp::Time & lookup_time,
    const double current_yaw_deg,
    const double current_pitch_deg)
  {
    const auto target = lookupTargetInFrame(
      solve_frame_,
      lookup_time,
      "; frame '" + solve_frame_ +
      "' is absent. Start localization/TF and make sure the gimbal TF chain is connected to map");
    if (!target) {
      return std::nullopt;
    }

    const double horizontal = std::hypot(target->x, target->y);
    const double distance = std::hypot(horizontal, target->z);
    if (distance < min_distance_m_) {
      warnThrottled(
        "target too close in " + solve_frame_ + ": (" + std::to_string(target->x) + ", " +
        std::to_string(target->y) + ", " + std::to_string(target->z) + ")m");
      return std::nullopt;
    }
    if (max_target_distance_m_ > 0.0 && distance > max_target_distance_m_) {
      warnThrottled(
        "target distance in " + solve_frame_ + " is unreasonable (" +
        std::to_string(distance) + "m); skip map aim command. Check odom/localization TF.");
      return std::nullopt;
    }

    const double sign = yaw_sign_ < 0.0 ? -1.0 : 1.0;
    const double current_forward_rad = sign * current_yaw_deg * M_PI / 180.0;
    const double forward_x = std::cos(current_forward_rad);
    const double forward_y = std::sin(current_forward_rad);
    const double dot = forward_x * target->x + forward_y * target->y;
    const double cross = forward_x * target->y - forward_y * target->x;
    double yaw_error_rad = std::atan2(cross, dot);
    if (dot < 0.0 && std::abs(cross) < 1e-6) {
      double turn_sign = 1.0;
      if (last_yaw_cmd_deg_) {
        const double last_delta = std::remainder(*last_yaw_cmd_deg_ - current_yaw_deg, 360.0);
        if (std::abs(last_delta) > 1e-3) {
          turn_sign = last_delta < 0.0 ? -1.0 : 1.0;
        }
      }
      yaw_error_rad = turn_sign * M_PI;
    }
    const double yaw_error_deg = sign * yaw_error_rad * 180.0 / M_PI + yaw_bias_deg_;
    const double yaw_cmd_deg = normalizeNear(current_yaw_deg + yaw_error_deg, current_yaw_deg);
    const double pitch_cmd_deg =
      pitch_sign_ * std::atan2(target->z, horizontal) * 180.0 / M_PI + pitch_bias_deg_;

    std::ostringstream detail;
    detail << "target_in_" << solve_frame_ << "=(" << target->x << "," << target->y << ","
           << target->z << ")m err_yaw=" << yaw_error_deg
           << " dot=" << dot << " cross=" << cross << " target_pitch=" << pitch_cmd_deg;
    (void)current_pitch_deg;
    return SolvedCommand{yaw_cmd_deg, pitch_cmd_deg, *target, detail.str()};
  }

  std::optional<SolvedCommand> solveCameraProjectionAngles(
    const rclcpp::Time & lookup_time,
    const double current_yaw_deg,
    const double current_pitch_deg)
  {
    const auto target_camera = lookupTargetInFrame(
      camera_frame_,
      lookup_time,
      "; frame '" + camera_frame_ + "' is absent. Start sentry_tf/tf_tree and make sure gimbal_barrel -> gx_camera is published");
    if (!target_camera) {
      return std::nullopt;
    }

    const double cx = target_camera->x;
    const double cy = target_camera->y;
    const double cz = target_camera->z;
    const double distance = std::sqrt(cx * cx + cy * cy + cz * cz);
    if (distance < min_distance_m_) {
      warnThrottled(
        "target too close in " + camera_frame_ + ": (" + std::to_string(cx) + ", " +
        std::to_string(cy) + ", " + std::to_string(cz) + ")m");
      return std::nullopt;
    }
    if (max_target_distance_m_ > 0.0 && distance > max_target_distance_m_) {
      warnThrottled(
        "target distance in " + camera_frame_ + " is unreasonable (" +
        std::to_string(distance) + "m); skip map aim command. Check odom/localization TF.");
      return std::nullopt;
    }
    const bool target_behind_camera = cz <= 0.0;
    if (target_behind_camera) {
      warnThrottled(
        "target is behind " + camera_frame_ + ": (" + std::to_string(cx) + ", " +
        std::to_string(cy) + ", " + std::to_string(cz) +
        ")m; use geometric yaw/pitch fallback to turn it into camera front");
    }

    const double yaw_error_deg = yaw_sign_ * std::atan2(cx, cz) * 180.0 / M_PI + yaw_bias_deg_;
    const double pitch_error_deg =
      pitch_sign_ * std::atan2(-cy, std::hypot(cx, cz)) * 180.0 / M_PI + pitch_bias_deg_;
    const double yaw_cmd_deg = normalizeNear(current_yaw_deg + yaw_error_deg, current_yaw_deg);
    const double pitch_cmd_deg = current_pitch_deg + pitch_error_deg;

    auto target_solve = lookupTargetInFrame(solve_frame_, lookup_time, "");
    if (!target_solve) {
      geometry_msgs::msg::Point nan_point;
      nan_point.x = std::numeric_limits<double>::quiet_NaN();
      nan_point.y = std::numeric_limits<double>::quiet_NaN();
      nan_point.z = std::numeric_limits<double>::quiet_NaN();
      target_solve = nan_point;
    }

    std::ostringstream detail;
    detail << "target_in_" << solve_frame_ << "=(" << target_solve->x << "," << target_solve->y
           << "," << target_solve->z << ")m target_in_" << camera_frame_ << "=(" << cx << ","
           << cy << "," << cz << ")m err_yaw=" << yaw_error_deg
           << " err_pitch=" << pitch_error_deg;
    if (target_behind_camera) {
      detail << " behind_fallback=geometric";
    }
    return SolvedCommand{yaw_cmd_deg, pitch_cmd_deg, *target_camera, detail.str()};
  }

  void onTimer()
  {
    if (!current_angles_) {
      warnThrottled("waiting for /ly/gimbal/angles before publishing map aim command");
      return;
    }

    rclcpp::Time lookup_time(0, 0, this->get_clock()->get_clock_type());
    try {
      lookup_time = selectTfLookupTime(*current_angles_);
    } catch (const std::runtime_error &) {
      return;
    }

    const double current_yaw_deg = static_cast<double>(current_angles_->yaw);
    const double current_pitch_deg = static_cast<double>(current_angles_->pitch);
    std::optional<SolvedCommand> solved;
    if (solve_mode_ == "camera" || solve_mode_ == "camera_projection" || solve_mode_ == "gx_camera") {
      solved = solveCameraProjectionAngles(lookup_time, current_yaw_deg, current_pitch_deg);
    } else if (solve_mode_ == "relative" || solve_mode_ == "relative_geometry" ||
      solve_mode_ == "gimbal_relative")
    {
      solved = solveRelativeGeometryAngles(lookup_time, current_yaw_deg, current_pitch_deg);
    } else if (solve_mode_ == "base" || solve_mode_ == "base_link" || solve_mode_ == "absolute") {
      solved = solveBaseLinkAngles(lookup_time, current_yaw_deg, current_pitch_deg);
    } else {
      warnThrottled(
        "unknown solve_mode '" + solve_mode_ +
        "'; use camera_projection, relative_geometry, or base_link");
      return;
    }
    if (!solved) {
      return;
    }

    double yaw_cmd_deg = solved->yaw_deg;
    double pitch_cmd_deg = solved->pitch_deg;
    if (last_yaw_cmd_deg_ && 0.0 < command_filter_alpha_ && command_filter_alpha_ < 1.0) {
      yaw_cmd_deg = lowpassAngle(yaw_cmd_deg, *last_yaw_cmd_deg_, command_filter_alpha_);
    }
    if (last_pitch_cmd_deg_ && 0.0 < command_filter_alpha_ && command_filter_alpha_ < 1.0) {
      pitch_cmd_deg =
        *last_pitch_cmd_deg_ + command_filter_alpha_ * (pitch_cmd_deg - *last_pitch_cmd_deg_);
    }

    const double yaw_step_ref = last_yaw_cmd_deg_.value_or(current_yaw_deg);
    const double pitch_step_ref = last_pitch_cmd_deg_.value_or(current_pitch_deg);
    yaw_cmd_deg = limitStep(yaw_cmd_deg, yaw_step_ref, max_yaw_step_deg_);
    pitch_cmd_deg = limitStep(pitch_cmd_deg, pitch_step_ref, max_pitch_step_deg_);

    const auto stamp = this->now();
    gimbal_driver::msg::GimbalAngles angle_msg;
    angle_msg.header.stamp = stamp;
    angle_msg.yaw = static_cast<float>(yaw_cmd_deg);
    angle_msg.pitch = static_cast<float>(pitch_cmd_deg);
    pub_angles_->publish(angle_msg);
    last_yaw_cmd_deg_ = yaw_cmd_deg;
    last_pitch_cmd_deg_ = pitch_cmd_deg;

    if (pub_firecode_) {
      gimbal_driver::msg::FireCode fire_msg;
      fire_msg.header.stamp = stamp;
      fire_msg.field_mask =
        gimbal_driver::msg::FireCode::FIELD_FIRE_STATUS |
        gimbal_driver::msg::FireCode::FIELD_AIM_MODE;
      fire_msg.fire_status = 0;
      fire_msg.aim_mode = aim_mode_;
      pub_firecode_->publish(fire_msg);
    }

    const int64_t now_ns = this->now().nanoseconds();
    if (now_ns - last_info_ns_ > static_cast<int64_t>(2e9)) {
      RCLCPP_INFO(
        this->get_logger(),
        "map aim command: target_%s=(%.2f,%.2f,%.2f)m %s yaw=%.2f pitch=%.2f current_yaw=%.2f current_pitch=%.2f",
        active_target_frame_.c_str(),
        active_target_point_.x,
        active_target_point_.y,
        active_target_point_.z,
        solved->detail.c_str(),
        angle_msg.yaw,
        angle_msg.pitch,
        current_yaw_deg,
        current_pitch_deg);
      last_info_ns_ = now_ns;
    }
  }

  double official_map_x_m_{0.0};
  double official_map_y_m_{0.0};
  double map_z_m_{0.0};
  std::string target_frame_;
  std::string active_target_frame_;
  geometry_msgs::msg::Point active_target_point_;
  std::string raw_goal_target_frame_override_;
  std::string raw_goal_configured_target_frame_{"map"};
  PointerSolver raw_goal_solver_{};
  bool raw_goal_static_calibration_ready_{false};
  std::string aim_frame_;
  std::string camera_frame_;
  std::string solve_mode_;
  std::string solve_frame_;

  bool publish_firecode_{true};
  bool aim_mode_{true};
  rclcpp::Duration tf_timeout_{0, 0};
  bool use_gimbal_stamp_for_tf_{false};
  double max_gimbal_stamp_age_sec_{0.50};
  double min_distance_m_{0.10};
  double max_target_distance_m_{100.0};
  double command_filter_alpha_{1.0};
  double yaw_sign_{-1.0};
  double pitch_sign_{1.0};
  double yaw_bias_deg_{0.0};
  double pitch_bias_deg_{0.0};
  double max_yaw_step_deg_{0.0};
  double max_pitch_step_deg_{0.0};

  std::optional<gimbal_driver::msg::GimbalAngles> current_angles_;
  std::optional<double> last_yaw_cmd_deg_;
  std::optional<double> last_pitch_cmd_deg_;
  int64_t last_warn_ns_{0};
  int64_t last_info_ns_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<gimbal_driver::msg::GimbalAngles>::SharedPtr pub_angles_;
  rclcpp::Publisher<gimbal_driver::msg::FireCode>::SharedPtr pub_firecode_;
  rclcpp::Subscription<gimbal_driver::msg::GimbalAngles>::SharedPtr sub_angles_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_face_target_raw_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navi_tf_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<navi_tf_bridge::FaceModeNode>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("map_aim_point_node"), "FaceMode failed: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
}
