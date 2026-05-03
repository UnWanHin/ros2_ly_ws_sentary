#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "auto_aim_common/msg/relative_target.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "navi_tf_bridge/chase_pointer.hpp"
#include "navi_tf_bridge/goal_output.hpp"
#include "navi_tf_bridge/map_pointer.hpp"
#include "navi_tf_bridge/pointer_debug.hpp"
#include "navi_tf_bridge/pointer_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navi_tf_bridge
{

class TargetRelToGoalPosNode : public rclcpp::Node
{
public:
  TargetRelToGoalPosNode()
  : Node("target_rel_to_goal_pos_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/ly/navi/target_rel");
    input_goal_pos_raw_topic_ =
      this->declare_parameter<std::string>("input_goal_pos_raw_topic", "/ly/navi/goal_pos_raw");
    output_goal_pos_topic_ =
      this->declare_parameter<std::string>("output_goal_pos_topic", "/ly/navi/goal_pos");
    output_goal_pose_topic_ =
      this->declare_parameter<std::string>("output_goal_pose_topic", "/goal_pose");
    output_target_map_topic_ =
      this->declare_parameter<std::string>("output_target_map_topic", "/ly/navi/target_map");

    const std::string map_frame = this->declare_parameter<std::string>("map_frame", "map");
    const std::string base_frame =
      this->declare_parameter<std::string>("base_frame", "base_link");
    const std::string fallback_base_frame =
      this->declare_parameter<std::string>("fallback_base_frame", "baselink");
    const bool use_msg_frame_id = this->declare_parameter<bool>("use_msg_frame_id", true);
    const std::string target_rel_default_frame =
      this->declare_parameter<std::string>("target_rel_default_frame", "gx_camera");

    const bool publish_target_map = this->declare_parameter<bool>("publish_target_map", true);
    const bool publish_goal_pos = this->declare_parameter<bool>("publish_goal_pos", false);
    const bool publish_goal_pose = this->declare_parameter<bool>("publish_goal_pose", true);
    const bool invert_y_axis = this->declare_parameter<bool>("invert_y_axis", false);
    const int y_axis_max_cm = this->declare_parameter<int>("y_axis_max_cm", 1500);
    const bool goal_pos_uint16_encode_enabled =
      this->declare_parameter<bool>("goal_pos_uint16_encode_enabled", false);
    const double goal_pos_uint16_encode_x_scale =
      this->declare_parameter<double>("goal_pos_uint16_encode_x_scale", 1.0);
    const double goal_pos_uint16_encode_y_scale =
      this->declare_parameter<double>("goal_pos_uint16_encode_y_scale", 1.0);
    const double goal_pos_uint16_encode_x_offset_cm =
      this->declare_parameter<double>("goal_pos_uint16_encode_x_offset_cm", 0.0);
    const double goal_pos_uint16_encode_y_offset_cm =
      this->declare_parameter<double>("goal_pos_uint16_encode_y_offset_cm", 0.0);

    const int preferred_distance_cm =
      this->declare_parameter<int>("preferred_distance_cm", 100);
    const int distance_deadband_cm =
      this->declare_parameter<int>("distance_deadband_cm", 50);
    const bool stop_when_no_target =
      this->declare_parameter<bool>("stop_when_no_target", true);
    const bool allow_reverse_goal =
      this->declare_parameter<bool>("allow_reverse_goal", false);

    const bool enable_goal_pos_raw_bridge =
      this->declare_parameter<bool>("enable_goal_pos_raw_bridge", true);
    const std::string goal_pos_raw_frame =
      this->declare_parameter<std::string>("goal_pos_raw_frame", "map");

    const bool debug_export_point_pairs =
      this->declare_parameter<bool>("debug_export_point_pairs", true);
    const std::string debug_points_reference_frame =
      this->declare_parameter<std::string>("debug_points_reference_frame", "map");
    const std::string debug_area_header_file =
      this->declare_parameter<std::string>("debug_area_header_file", "");
    const std::string debug_point_pairs_output_file =
      this->declare_parameter<std::string>("debug_point_pairs_output_file", "");

    const bool use_raw_goal_static_calibration =
      this->declare_parameter<bool>("use_raw_goal_static_calibration", false);
    const std::string raw_goal_calibration_model =
      this->declare_parameter<std::string>("raw_goal_calibration_model", "rigid");
    const std::string raw_goal_calibration_unit =
      this->declare_parameter<std::string>("raw_goal_calibration_unit", "cm");
    const std::string raw_goal_source_frame =
      this->declare_parameter<std::string>("raw_goal_source_frame", "official_map");
    const std::string raw_goal_target_frame =
      this->declare_parameter<std::string>("raw_goal_target_frame", "map");
    const std::vector<double> raw_goal_source_points =
      this->declare_parameter<std::vector<double>>(
      "raw_goal_source_points", std::vector<double>{});
    const std::vector<double> raw_goal_target_points =
      this->declare_parameter<std::vector<double>>(
      "raw_goal_target_points", std::vector<double>{});
    const std::vector<double> raw_goal_transform_matrix =
      this->declare_parameter<std::vector<double>>(
      "raw_goal_transform_matrix", std::vector<double>{});

    chase_pointer_.setConfig(ChasePointer::Config{
      .map_frame = map_frame,
      .base_frame = base_frame,
      .fallback_base_frame = fallback_base_frame,
      .use_msg_frame_id = use_msg_frame_id,
      .default_frame = target_rel_default_frame,
      .preferred_distance_cm = preferred_distance_cm,
      .distance_deadband_cm = distance_deadband_cm,
      .stop_when_no_target = stop_when_no_target,
      .allow_reverse_goal = allow_reverse_goal});

    goal_output_.setConfig(GoalOutput::Config{
      .map_frame = map_frame,
      .publish_target_map = publish_target_map,
      .publish_goal_pos = publish_goal_pos,
      .publish_goal_pose = publish_goal_pose,
      .invert_y_axis = invert_y_axis,
      .y_axis_max_cm = y_axis_max_cm,
      .uint16_encode_enabled = goal_pos_uint16_encode_enabled,
      .uint16_encode_x_scale = goal_pos_uint16_encode_x_scale,
      .uint16_encode_y_scale = goal_pos_uint16_encode_y_scale,
      .uint16_encode_x_offset_cm = goal_pos_uint16_encode_x_offset_cm,
      .uint16_encode_y_offset_cm = goal_pos_uint16_encode_y_offset_cm});
    goal_output_.sanitize(*this);

    PointerSolver::Config solver_config{
      .enabled = use_raw_goal_static_calibration,
      .model = raw_goal_calibration_model,
      .unit = raw_goal_calibration_unit,
      .source_frame = raw_goal_source_frame,
      .target_frame = raw_goal_target_frame,
      .map_frame = map_frame,
      .source_points = raw_goal_source_points,
      .target_points = raw_goal_target_points,
      .transform_matrix = raw_goal_transform_matrix};
    map_pointer_.setConfig(MapPointer::Config{
      .enabled = enable_goal_pos_raw_bridge,
      .raw_frame = goal_pos_raw_frame,
      .map_frame = map_frame,
      .use_static_calibration = use_raw_goal_static_calibration,
      .solver = solver_config});
    map_pointer_.initialize(*this);

    pointer_debug_.setConfig(PointerDebug::Config{
      .enabled = debug_export_point_pairs,
      .map_frame = map_frame,
      .reference_frame = debug_points_reference_frame,
      .area_header_file = debug_area_header_file,
      .output_file = debug_point_pairs_output_file});

    sub_target_rel_ = this->create_subscription<auto_aim_common::msg::RelativeTarget>(
      input_topic_,
      rclcpp::QoS(10),
      std::bind(&TargetRelToGoalPosNode::targetRelCallback, this, std::placeholders::_1));
    sub_goal_pos_raw_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      input_goal_pos_raw_topic_,
      rclcpp::QoS(10),
      std::bind(&TargetRelToGoalPosNode::goalPosRawCallback, this, std::placeholders::_1));

    const auto & output_config = goal_output_.config();
    if (output_config.publish_goal_pos) {
      goal_publishers_.goal_pos =
        this->create_publisher<std_msgs::msg::UInt16MultiArray>(output_goal_pos_topic_, 10);
    }
    if (output_config.publish_goal_pose) {
      goal_publishers_.goal_pose =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(output_goal_pose_topic_, 10);
    }
    if (output_config.publish_target_map) {
      pub_target_map_ =
        this->create_publisher<geometry_msgs::msg::PointStamped>(output_target_map_topic_, 10);
    }

    logStartup();

    if (pointer_debug_.config().enabled) {
      debug_export_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TargetRelToGoalPosNode::onDebugExportTimer, this));
      RCLCPP_INFO(
        this->get_logger(),
        "TF debug export enabled. ref_frame=%s area_header=%s out=%s",
        pointer_debug_.config().reference_frame.c_str(),
        pointer_debug_.config().area_header_file.empty() ?
        "<empty>" : pointer_debug_.config().area_header_file.c_str(),
        pointer_debug_.config().output_file.empty() ?
        "<empty>" : pointer_debug_.config().output_file.c_str());
    }
  }

private:
  void logStartup() const
  {
    const auto & chase = chase_pointer_.config();
    const auto & output = goal_output_.config();
    const auto & map = map_pointer_.config();
    const auto & solver = map.solver;
    const double inv_x_scale = 1.0 / output.uint16_encode_x_scale;
    const double inv_y_scale = 1.0 / output.uint16_encode_y_scale;
    const double inv_x_offset = -output.uint16_encode_x_offset_cm / output.uint16_encode_x_scale;
    const double inv_y_offset = -output.uint16_encode_y_offset_cm / output.uint16_encode_y_scale;

    RCLCPP_INFO(
      this->get_logger(),
      "Started target_rel -> goal bridge. in=%s goal_pos_out=%s publish_goal_pos=%s goal_pose_out=%s publish_goal_pose=%s map_frame=%s base_frame=%s "
      "fallback_base_frame=%s target_rel_default_frame=%s raw_goal_in=%s raw_goal_frame=%s invert_y_axis=%s y_axis_max_cm=%d preferred_distance_cm=%d "
      "goal_u16_encode=%s enc=[[%.6f,0,%.3f],[0,%.6f,%.3f]] dec=[[%.6f,0,%.3f],[0,%.6f,%.3f]] "
      "distance_deadband_cm=%d stop_when_no_target=%s allow_reverse_goal=%s "
      "use_raw_goal_static_calibration=%s model=%s source_frame=%s target_frame=%s",
      input_topic_.c_str(),
      output_goal_pos_topic_.c_str(),
      output.publish_goal_pos ? "true" : "false",
      output_goal_pose_topic_.c_str(),
      output.publish_goal_pose ? "true" : "false",
      output.map_frame.c_str(),
      chase.base_frame.c_str(),
      chase.fallback_base_frame.c_str(),
      chase.default_frame.c_str(),
      input_goal_pos_raw_topic_.c_str(),
      map.raw_frame.c_str(),
      output.invert_y_axis ? "true" : "false",
      output.y_axis_max_cm,
      chase.preferred_distance_cm,
      output.uint16_encode_enabled ? "true" : "false",
      output.uint16_encode_x_scale,
      output.uint16_encode_x_offset_cm,
      output.uint16_encode_y_scale,
      output.uint16_encode_y_offset_cm,
      inv_x_scale,
      inv_x_offset,
      inv_y_scale,
      inv_y_offset,
      chase.distance_deadband_cm,
      chase.stop_when_no_target ? "true" : "false",
      chase.allow_reverse_goal ? "true" : "false",
      map.use_static_calibration ? "true" : "false",
      solver.model.c_str(),
      solver.source_frame.c_str(),
      solver.target_frame.c_str());
  }

  void onDebugExportTimer()
  {
    const bool should_cancel = pointer_debug_.onTimer(
      chase_pointer_.buildSourceCandidates(""),
      tf_buffer_,
      *this);
    if (should_cancel && debug_export_timer_) {
      debug_export_timer_->cancel();
    }
  }

  void goalPosRawCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (!msg) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Drop invalid goal_pos_raw: data size=0 (need >=2)");
      return;
    }

    geometry_msgs::msg::PointStamped point_map;
    std::string source_name;
    if (!map_pointer_.toMap(*msg, tf_buffer_, *this, point_map, source_name)) {
      return;
    }
    goal_output_.publishMapPointAsGoal(
      point_map.point,
      source_name,
      point_map.header.stamp,
      *this,
      goal_publishers_);
  }

  void targetRelCallback(const auto_aim_common::msg::RelativeTarget::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    geometry_msgs::msg::Point relative_goal_point;
    if (!chase_pointer_.buildRelativeGoalPoint(*msg, *this, relative_goal_point)) {
      return;
    }

    const auto source_candidates = chase_pointer_.buildSourceCandidates(msg->header.frame_id);
    const rclcpp::Time transform_time(msg->header.stamp);

    geometry_msgs::msg::PointStamped point_map;
    std::string last_tf_error;
    std::string resolved_source_frame;
    if (!chase_pointer_.transformToMap(
        relative_goal_point,
        source_candidates,
        transform_time,
        tf_buffer_,
        point_map,
        resolved_source_frame,
        last_tf_error))
    {
      std::ostringstream source_oss;
      for (const auto & frame : source_candidates) {
        source_oss << frame << " ";
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "TF transform failed (source=[%s], map=%s). Last error: %s",
        source_oss.str().c_str(),
        chase_pointer_.config().map_frame.c_str(),
        last_tf_error.c_str());
      return;
    }

    point_map.header.frame_id = chase_pointer_.config().map_frame;
    point_map.header.stamp =
      (transform_time.nanoseconds() == 0) ? this->now() : transform_time;

    if (goal_output_.config().publish_target_map && pub_target_map_) {
      pub_target_map_->publish(point_map);
    }
    goal_output_.publishMapPointAsGoal(
      point_map.point,
      resolved_source_frame,
      point_map.header.stamp,
      *this,
      goal_publishers_);
  }

  std::string input_topic_;
  std::string input_goal_pos_raw_topic_;
  std::string output_goal_pos_topic_;
  std::string output_goal_pose_topic_;
  std::string output_target_map_topic_;

  ChasePointer chase_pointer_;
  MapPointer map_pointer_;
  GoalOutput goal_output_;
  PointerDebug pointer_debug_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<auto_aim_common::msg::RelativeTarget>::SharedPtr sub_target_rel_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_goal_pos_raw_;
  GoalOutput::Publishers goal_publishers_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_map_;
  rclcpp::TimerBase::SharedPtr debug_export_timer_;
};

}  // namespace navi_tf_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navi_tf_bridge::TargetRelToGoalPosNode>());
  rclcpp::shutdown();
  return 0;
}
