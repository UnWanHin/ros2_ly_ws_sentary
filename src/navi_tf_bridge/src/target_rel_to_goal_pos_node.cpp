#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "auto_aim_common/msg/relative_target.hpp"
#include "gimbal_driver/msg/stamped_u_int16_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "navi_tf_bridge/chase_area_limiter.hpp"
#include "navi_tf_bridge/chase_pointer.hpp"
#include "navi_tf_bridge/goal_output.hpp"
#include "navi_tf_bridge/map_pointer.hpp"
#include "navi_tf_bridge/pointer_debug.hpp"
#include "navi_tf_bridge/pointer_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
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
    output_target_official_topic_ =
      this->declare_parameter<std::string>(
      "output_target_official_topic", "/ly/navi/target_official");
    output_navi_position_topic_ =
      this->declare_parameter<std::string>("output_navi_position_topic", "/ly/navi/position");

    const std::string map_frame = this->declare_parameter<std::string>("map_frame", "map");
    const std::string base_frame =
      this->declare_parameter<std::string>("base_frame", "base_link");
    const std::string fallback_base_frame =
      this->declare_parameter<std::string>("fallback_base_frame", "baselink");
    const bool use_msg_frame_id = this->declare_parameter<bool>("use_msg_frame_id", true);
    const std::string target_rel_default_frame =
      this->declare_parameter<std::string>("target_rel_default_frame", "gimbal_world");

    const bool publish_target_map = this->declare_parameter<bool>("publish_target_map", true);
    const bool publish_target_official =
      this->declare_parameter<bool>("publish_target_official", true);
    const bool publish_navi_position =
      this->declare_parameter<bool>("publish_navi_position", true);
    const bool publish_goal_pos = this->declare_parameter<bool>("publish_goal_pos", false);
    const bool publish_goal_pose = this->declare_parameter<bool>("publish_goal_pose", true);
    const double navi_position_publish_hz =
      std::max(1.0, this->declare_parameter<double>("navi_position_publish_hz", 10.0));
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
    const bool chase_area_limit_enable =
      this->declare_parameter<bool>("chase_area_limit.enable", false);
    const std::string chase_area_limit_area_header_file =
      this->declare_parameter<std::string>("chase_area_limit.area_header_file", "");
    const double chase_area_limit_boundary_margin_cm =
      this->declare_parameter<double>("chase_area_limit.boundary_margin_cm", 30.0);
    const bool chase_area_limit_chase_enable_cross_area =
      this->declare_parameter<bool>("chase_area_limit.chase_enable_cross_area", false);
    const bool chase_area_limit_hold_when_no_intersection =
      this->declare_parameter<bool>("chase_area_limit.hold_when_no_intersection", true);
    chase_area_limit_use_area_scope_ =
      this->declare_parameter<bool>("chase_area_limit.use_area_scope", false);
    chase_area_limit_my_area_ =
      splitAreaTokens(this->declare_parameter<std::string>("chase_area_limit.my_area", ""));
    chase_area_limit_enemy_area_ =
      splitAreaTokens(this->declare_parameter<std::string>("chase_area_limit.enemy_area", ""));
    chase_area_limit_common_area_ =
      splitAreaTokens(this->declare_parameter<std::string>("chase_area_limit.common_area", ""));
    const std::string friend_is_team_red_topic =
      this->declare_parameter<std::string>(
      "chase_area_limit.friend_is_team_red_topic", "/ly/friend/is_team_red");
    const std::vector<std::string> chase_area_limit_area_names =
      this->declare_parameter<std::vector<std::string>>(
      "chase_area_limit.area_names", std::vector<std::string>{});

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
    navi_position_frame_ = raw_goal_source_frame;
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

    ChaseAreaLimiter::Config chase_area_limit_config{
      .enabled = chase_area_limit_enable,
      .area_header_file = chase_area_limit_area_header_file,
      .boundary_margin_cm = chase_area_limit_boundary_margin_cm,
      .chase_enable_cross_area = chase_area_limit_chase_enable_cross_area,
      .hold_when_no_intersection = chase_area_limit_hold_when_no_intersection,
      .require_allowed_area_match = false,
      .allowed_area_names = {},
      .areas = {}};
    if (!chase_area_limit_area_header_file.empty()) {
      chase_area_limit_config.areas =
        loadChaseAreasFromAreaHeader(chase_area_limit_area_header_file);
    }
    if (chase_area_limit_config.areas.empty()) {
      for (const auto & area_name : chase_area_limit_area_names) {
        const auto coords = this->declare_parameter<std::vector<std::int64_t>>(
          "chase_area_limit.areas." + area_name, std::vector<std::int64_t>{});
        if (coords.size() < 6 || coords.size() % 2 != 0) {
          RCLCPP_WARN(
            this->get_logger(),
            "Ignore invalid chase area '%s': need even [x1,y1,...] with at least 3 points, got %zu values.",
            area_name.c_str(),
            coords.size());
          continue;
        }
        ChaseAreaLimiter::Area area;
        area.name = area_name;
        area.boundary.reserve(coords.size() / 2);
        for (std::size_t i = 0; i + 1 < coords.size(); i += 2) {
          area.boundary.push_back(
            ChaseAreaLimiter::PointCm{
              static_cast<double>(coords[i]),
              static_cast<double>(coords[i + 1])});
        }
        chase_area_limit_config.areas.push_back(std::move(area));
      }
    }
    chase_area_limiter_.setConfig(std::move(chase_area_limit_config));
    updateChaseAreaLimitAllowedAreas();

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
    sub_friend_is_team_red_ = this->create_subscription<std_msgs::msg::Bool>(
      friend_is_team_red_topic,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg) {
          return;
        }
        friend_is_team_red_ = msg->data;
        updateChaseAreaLimitAllowedAreas();
      });

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
    if (publish_target_official) {
      pub_target_official_ =
        this->create_publisher<gimbal_driver::msg::StampedUInt16MultiArray>(
        output_target_official_topic_, 10);
    }
    if (publish_navi_position) {
      pub_navi_position_ =
        this->create_publisher<gimbal_driver::msg::StampedUInt16MultiArray>(
          output_navi_position_topic_, 10);
      navi_position_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / navi_position_publish_hz),
        std::bind(&TargetRelToGoalPosNode::onNaviPositionTimer, this));
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
  static std::string normalizeAreaToken(std::string token)
  {
    std::transform(
      token.begin(), token.end(), token.begin(),
      [](const unsigned char c) {
        if (c == '-' || c == ' ') {
          return '_';
        }
        return static_cast<char>(std::tolower(c));
      });
    return token;
  }

  static std::vector<std::string> splitAreaTokens(const std::string & raw)
  {
    std::vector<std::string> tokens;
    std::stringstream ss(raw);
    std::string token;
    while (std::getline(ss, token, ',')) {
      token = normalizeAreaToken(token);
      token.erase(
        std::remove_if(
          token.begin(), token.end(),
          [](const unsigned char c) { return std::isspace(c) != 0; }),
        token.end());
      if (!token.empty()) {
        tokens.push_back(token);
      }
    }
    return tokens;
  }

  static void addSideAreaNames(
    std::vector<std::string> & names,
    const std::vector<std::string> & scope,
    const char * side)
  {
    for (const auto & token : scope) {
      if (token == "base") {
        names.push_back(std::string(side) + "_base");
      } else if (token == "highland" || token == "high_land" || token == "high") {
        names.push_back(std::string(side) + "_highland");
      } else if (token == "roadland" || token == "road_land" || token == "road") {
        names.push_back(std::string(side) + "_roadland");
      }
    }
  }

  static void addCommonAreaNames(
    std::vector<std::string> & names,
    const std::vector<std::string> & scope)
  {
    for (const auto & token : scope) {
      if (token == "central" || token == "center" || token == "centre" || token == "middle") {
        names.push_back("common_central");
      }
    }
  }

  void updateChaseAreaLimitAllowedAreas()
  {
    auto config = chase_area_limiter_.config();
    config.allowed_area_names.clear();
    config.require_allowed_area_match =
      config.chase_enable_cross_area && chase_area_limit_use_area_scope_;

    if (config.require_allowed_area_match) {
      addCommonAreaNames(config.allowed_area_names, chase_area_limit_common_area_);
      if (friend_is_team_red_.has_value()) {
        const char * my_side = *friend_is_team_red_ ? "red" : "blue";
        const char * enemy_side = *friend_is_team_red_ ? "blue" : "red";
        addSideAreaNames(config.allowed_area_names, chase_area_limit_my_area_, my_side);
        addSideAreaNames(config.allowed_area_names, chase_area_limit_enemy_area_, enemy_side);
      }
      std::sort(config.allowed_area_names.begin(), config.allowed_area_names.end());
      config.allowed_area_names.erase(
        std::unique(config.allowed_area_names.begin(), config.allowed_area_names.end()),
        config.allowed_area_names.end());
    }

    chase_area_limiter_.setConfig(std::move(config));
  }

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
      "fallback_base_frame=%s target_rel_default_frame=%s target_official_out=%s publish_target_official=%s raw_goal_in=%s raw_goal_frame=%s invert_y_axis=%s y_axis_max_cm=%d preferred_distance_cm=%d "
      "goal_u16_encode=%s enc=[[%.6f,0,%.3f],[0,%.6f,%.3f]] dec=[[%.6f,0,%.3f],[0,%.6f,%.3f]] "
      "distance_deadband_cm=%d stop_when_no_target=%s allow_reverse_goal=%s "
      "chase_area_limit=%s area_header=%s area_count=%zu boundary_margin_cm=%.1f chase_enable_cross_area=%s "
      "scope_enable=%s allowed_area_count=%zu hold_no_intersection=%s "
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
      output_target_official_topic_.c_str(),
      pub_target_official_ ? "true" : "false",
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
      chase_area_limiter_.config().enabled ? "true" : "false",
      chase_area_limiter_.config().area_header_file.empty() ?
      "<empty>" : chase_area_limiter_.config().area_header_file.c_str(),
      chase_area_limiter_.config().areas.size(),
      chase_area_limiter_.config().boundary_margin_cm,
      chase_area_limiter_.config().chase_enable_cross_area ? "true" : "false",
      chase_area_limiter_.config().require_allowed_area_match ? "true" : "false",
      chase_area_limiter_.config().allowed_area_names.size(),
      chase_area_limiter_.config().hold_when_no_intersection ? "true" : "false",
      map.use_static_calibration ? "true" : "false",
      solver.model.c_str(),
      solver.source_frame.c_str(),
      solver.target_frame.c_str());
  }

  std::vector<ChaseAreaLimiter::Area> loadChaseAreasFromAreaHeader(
    const std::string & area_header_file) const
  {
    std::vector<ChaseAreaLimiter::Area> areas;
    if (area_header_file.empty()) {
      return areas;
    }

    std::ifstream ifs(area_header_file);
    if (!ifs.is_open()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Chase area limit: cannot open Area.hpp: %s",
        area_header_file.c_str());
      return areas;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string content = buffer.str();

    const std::vector<std::pair<std::string, std::string>> expected_areas{
      {"RedMainAreaBasePoints", "red_base"},
      {"RedMainAreaHighlandPoints", "red_highland"},
      {"RedMainAreaRoadlandPoints", "red_roadland"},
      {"CommonMainAreaCentralPoints", "common_central"},
      {"BlueMainAreaBasePoints", "blue_base"},
      {"BlueMainAreaHighlandPoints", "blue_highland"},
      {"BlueMainAreaRoadlandPoints", "blue_roadland"},
    };

    const std::regex vector_pattern(
      R"(static\s+const\s+std::vector<\s*Point<\s*int\s*>\s*>\s*([A-Za-z0-9_]+)\s*=\s*\{([\s\S]*?)\};)");
    const std::regex point_pattern(R"(\{\s*(-?[0-9]+)\s*,\s*(-?[0-9]+)\s*\})");

    for (const auto & [cpp_name, area_name] : expected_areas) {
      std::vector<ChaseAreaLimiter::PointCm> boundary;
      std::sregex_iterator it(content.begin(), content.end(), vector_pattern);
      std::sregex_iterator end;
      for (; it != end; ++it) {
        const auto & vector_match = *it;
        if (vector_match[1].str() != cpp_name) {
          continue;
        }
        const std::string body = vector_match[2].str();
        std::sregex_iterator point_it(body.begin(), body.end(), point_pattern);
        for (; point_it != end; ++point_it) {
          boundary.push_back(
            ChaseAreaLimiter::PointCm{
              static_cast<double>(std::stoi((*point_it)[1].str())),
              static_cast<double>(std::stoi((*point_it)[2].str()))});
        }
        break;
      }

      if (boundary.size() < 3) {
        RCLCPP_WARN(
          this->get_logger(),
          "Chase area limit: Area.hpp missing or invalid %s in %s.",
          cpp_name.c_str(),
          area_header_file.c_str());
        continue;
      }
      areas.push_back(
        ChaseAreaLimiter::Area{
          .name = area_name,
          .boundary = std::move(boundary)});
    }

    if (areas.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Chase area limit: no main-area boundaries parsed from %s.",
        area_header_file.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Chase area limit: parsed %zu main-area boundaries from %s.",
        areas.size(),
        area_header_file.c_str());
    }
    return areas;
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

  bool lookupBaseMapPoint(
    geometry_msgs::msg::Point & point_map,
    std::string & resolved_source_frame,
    std::string & last_tf_error,
    rclcpp::Time * source_stamp = nullptr)
  {
    resolved_source_frame.clear();
    last_tf_error.clear();
    if (source_stamp) {
      *source_stamp = this->now();
    }
    for (const auto & source_frame :
      {chase_pointer_.config().base_frame, chase_pointer_.config().fallback_base_frame})
    {
      if (source_frame.empty()) {
        continue;
      }
      try {
        const geometry_msgs::msg::TransformStamped tf_map_base =
          tf_buffer_.lookupTransform(
          chase_pointer_.config().map_frame,
          source_frame,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(0.05));
        point_map.x = tf_map_base.transform.translation.x;
        point_map.y = tf_map_base.transform.translation.y;
        point_map.z = tf_map_base.transform.translation.z;
        resolved_source_frame = source_frame;
        if (source_stamp) {
          const rclcpp::Time tf_stamp(
            tf_map_base.header.stamp,
            this->get_clock()->get_clock_type());
          *source_stamp = (tf_stamp.nanoseconds() == 0) ? this->now() : tf_stamp;
        }
        return true;
      } catch (const tf2::TransformException & ex) {
        last_tf_error =
          "lookup " + chase_pointer_.config().map_frame + " <- " + source_frame +
          " failed: " + ex.what();
      }
    }
    return false;
  }

  void onNaviPositionTimer()
  {
    if (!pub_navi_position_) {
      return;
    }

    geometry_msgs::msg::Point point_map;
    std::string resolved_source_frame;
    std::string last_tf_error;
    rclcpp::Time source_stamp = this->now();
    if (!lookupBaseMapPoint(point_map, resolved_source_frame, last_tf_error, &source_stamp)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Cannot publish /ly/navi/position: %s",
        last_tf_error.c_str());
      return;
    }

    double raw_x_cm = 0.0;
    double raw_y_cm = 0.0;
    if (!map_pointer_.mapToRawCentimeters(point_map, raw_x_cm, raw_y_cm)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Cannot publish /ly/navi/position: raw-goal static calibration is not ready.");
      return;
    }

    gimbal_driver::msg::StampedUInt16MultiArray msg;
    msg.header.stamp = source_stamp;
    msg.header.frame_id = navi_position_frame_;
    msg.data = {
      static_cast<std::uint16_t>(std::clamp(std::lround(raw_x_cm), 0L, 65535L)),
      static_cast<std::uint16_t>(std::clamp(std::lround(raw_y_cm), 0L, 65535L))
    };
    msg.map_point = point_map;
    msg.map_frame = chase_pointer_.config().map_frame;
    msg.source_frame = resolved_source_frame;
    pub_navi_position_->publish(msg);
  }

  bool rawCentimetersToMapPoint(
    const double raw_x_cm,
    const double raw_y_cm,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::PointStamped & point_map,
    std::string & source_name)
  {
    if (!std::isfinite(raw_x_cm) || !std::isfinite(raw_y_cm)) {
      return false;
    }

    std_msgs::msg::UInt16MultiArray raw_msg;
    raw_msg.data = {
      static_cast<std::uint16_t>(std::clamp(std::lround(raw_x_cm), 0L, 65535L)),
      static_cast<std::uint16_t>(std::clamp(std::lround(raw_y_cm), 0L, 65535L))
    };
    if (!map_pointer_.toMap(raw_msg, tf_buffer_, *this, point_map, source_name)) {
      return false;
    }
    point_map.header.frame_id = chase_pointer_.config().map_frame;
    point_map.header.stamp = stamp;
    return true;
  }

  bool applyChaseAreaLimit(
    geometry_msgs::msg::PointStamped & point_map,
    const rclcpp::Time & stamp)
  {
    if (!chase_area_limiter_.config().enabled) {
      return true;
    }

    geometry_msgs::msg::Point self_map;
    std::string resolved_source_frame;
    std::string last_tf_error;
    if (!lookupBaseMapPoint(self_map, resolved_source_frame, last_tf_error)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Chase area limit skipped: cannot resolve current base map point: %s",
        last_tf_error.c_str());
      return true;
    }

    double self_raw_x_cm = 0.0;
    double self_raw_y_cm = 0.0;
    double goal_raw_x_cm = 0.0;
    double goal_raw_y_cm = 0.0;
    if (!map_pointer_.mapToRawCentimeters(self_map, self_raw_x_cm, self_raw_y_cm) ||
        !map_pointer_.mapToRawCentimeters(point_map.point, goal_raw_x_cm, goal_raw_y_cm))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Chase area limit skipped: raw-goal static calibration is not ready.");
      return true;
    }

    const auto result = chase_area_limiter_.limit(
      ChaseAreaLimiter::PointCm{self_raw_x_cm, self_raw_y_cm},
      ChaseAreaLimiter::PointCm{goal_raw_x_cm, goal_raw_y_cm});
    if (!result.publish) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Chase area limit dropped goal: status=%s.",
        ChaseAreaLimiter::statusName(result.status));
      return false;
    }

    if (!result.clamped && !result.hold_current) {
      if (result.status == ChaseAreaLimiter::Status::UnknownArea ||
          result.status == ChaseAreaLimiter::Status::EmptyConfig)
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Chase area limit pass-through: status=%s self=(%.1f, %.1f) goal=(%.1f, %.1f).",
          ChaseAreaLimiter::statusName(result.status),
          self_raw_x_cm,
          self_raw_y_cm,
          goal_raw_x_cm,
          goal_raw_y_cm);
      }
      return true;
    }

    geometry_msgs::msg::PointStamped limited_map;
    std::string source_name;
    if (!rawCentimetersToMapPoint(result.point.x, result.point.y, stamp, limited_map, source_name)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Chase area limit failed to convert limited official point to map: status=%s point=(%.1f, %.1f).",
        ChaseAreaLimiter::statusName(result.status),
        result.point.x,
        result.point.y);
      return false;
    }

    point_map.point = limited_map.point;
    point_map.header.frame_id = limited_map.header.frame_id;
    point_map.header.stamp = limited_map.header.stamp;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Chase area limit %s area=%s self=(%.1f, %.1f) raw_goal=(%.1f, %.1f) limited=(%.1f, %.1f).",
      ChaseAreaLimiter::statusName(result.status),
      result.area_name.empty() ? "<none>" : result.area_name.c_str(),
      self_raw_x_cm,
      self_raw_y_cm,
      goal_raw_x_cm,
      goal_raw_y_cm,
      result.point.x,
      result.point.y);
    return true;
  }

  void publishTargetOfficial(
    const auto_aim_common::msg::RelativeTarget & msg,
    const geometry_msgs::msg::PointStamped & target_map,
    const std::string & resolved_source_frame)
  {
    if (!pub_target_official_) {
      return;
    }
    if (!msg.valid) {
      return;
    }

    double raw_x_cm = 0.0;
    double raw_y_cm = 0.0;
    if (!map_pointer_.mapToRawCentimeters(target_map.point, raw_x_cm, raw_y_cm)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Cannot publish /ly/navi/target_official: raw-goal static calibration is not ready.");
      return;
    }

    gimbal_driver::msg::StampedUInt16MultiArray official_msg;
    official_msg.header.stamp = target_map.header.stamp;
    official_msg.header.frame_id = navi_position_frame_;
    official_msg.data = {
      static_cast<std::uint16_t>(std::clamp(std::lround(raw_x_cm), 0L, 65535L)),
      static_cast<std::uint16_t>(std::clamp(std::lround(raw_y_cm), 0L, 65535L)),
      static_cast<std::uint16_t>(msg.armor_type)
    };
    official_msg.map_point = target_map.point;
    official_msg.map_frame = chase_pointer_.config().map_frame;
    official_msg.source_frame = resolved_source_frame;
    pub_target_official_->publish(official_msg);
  }

  bool transformExactTargetToMap(
    const auto_aim_common::msg::RelativeTarget & msg,
    const std::vector<std::string> & source_candidates,
    const rclcpp::Time & transform_time,
    geometry_msgs::msg::PointStamped & target_map,
    std::string & resolved_source_frame,
    std::string & last_tf_error)
  {
    if (!msg.valid) {
      return false;
    }

    const double x = static_cast<double>(msg.x);
    const double y = static_cast<double>(msg.y);
    const double z = static_cast<double>(msg.z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }
    if (std::hypot(x, y) <= 1e-6) {
      return false;
    }

    geometry_msgs::msg::Point target_rel;
    target_rel.x = x;
    target_rel.y = y;
    target_rel.z = z;
    if (!chase_pointer_.transformToMap(
        target_rel,
        source_candidates,
        transform_time,
        tf_buffer_,
        target_map,
        resolved_source_frame,
        last_tf_error))
    {
      return false;
    }

    target_map.header.frame_id = chase_pointer_.config().map_frame;
    target_map.header.stamp =
      (transform_time.nanoseconds() == 0) ? this->now() : transform_time;
    return true;
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

    geometry_msgs::msg::PointStamped target_map;
    std::string target_last_tf_error;
    std::string target_resolved_source_frame;
    if (transformExactTargetToMap(
        *msg,
        source_candidates,
        transform_time,
        target_map,
        target_resolved_source_frame,
        target_last_tf_error))
    {
      publishTargetOfficial(*msg, target_map, target_resolved_source_frame);
    }

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

    if (!applyChaseAreaLimit(point_map, point_map.header.stamp)) {
      return;
    }

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
  std::string output_target_official_topic_;
  std::string output_navi_position_topic_;
  std::string navi_position_frame_{"official_map"};

  ChasePointer chase_pointer_;
  ChaseAreaLimiter chase_area_limiter_;
  bool chase_area_limit_use_area_scope_{false};
  std::vector<std::string> chase_area_limit_my_area_{};
  std::vector<std::string> chase_area_limit_enemy_area_{};
  std::vector<std::string> chase_area_limit_common_area_{};
  std::optional<bool> friend_is_team_red_{};
  MapPointer map_pointer_;
  GoalOutput goal_output_;
  PointerDebug pointer_debug_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<auto_aim_common::msg::RelativeTarget>::SharedPtr sub_target_rel_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_goal_pos_raw_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_friend_is_team_red_;
  GoalOutput::Publishers goal_publishers_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_map_;
  rclcpp::Publisher<gimbal_driver::msg::StampedUInt16MultiArray>::SharedPtr pub_target_official_;
  rclcpp::Publisher<gimbal_driver::msg::StampedUInt16MultiArray>::SharedPtr pub_navi_position_;
  rclcpp::TimerBase::SharedPtr debug_export_timer_;
  rclcpp::TimerBase::SharedPtr navi_position_timer_;
};

}  // namespace navi_tf_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navi_tf_bridge::TargetRelToGoalPosNode>());
  rclcpp::shutdown();
  return 0;
}
