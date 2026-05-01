#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "auto_aim_common/msg/relative_target.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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

    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    fallback_base_frame_ =
      this->declare_parameter<std::string>("fallback_base_frame", "baselink");
    use_msg_frame_id_ = this->declare_parameter<bool>("use_msg_frame_id", true);
    target_rel_default_frame_ =
      this->declare_parameter<std::string>("target_rel_default_frame", "gimbal_world");

    publish_target_map_ = this->declare_parameter<bool>("publish_target_map", true);
    publish_goal_pos_ = this->declare_parameter<bool>("publish_goal_pos", false);
    publish_goal_pose_ = this->declare_parameter<bool>("publish_goal_pose", true);
    invert_y_axis_ = this->declare_parameter<bool>("invert_y_axis", false);
    y_axis_max_cm_ = this->declare_parameter<int>("y_axis_max_cm", 1500);
    goal_pos_uint16_encode_enabled_ =
      this->declare_parameter<bool>("goal_pos_uint16_encode_enabled", false);
    goal_pos_uint16_encode_x_scale_ =
      this->declare_parameter<double>("goal_pos_uint16_encode_x_scale", 1.0);
    goal_pos_uint16_encode_y_scale_ =
      this->declare_parameter<double>("goal_pos_uint16_encode_y_scale", 1.0);
    goal_pos_uint16_encode_x_offset_cm_ =
      this->declare_parameter<double>("goal_pos_uint16_encode_x_offset_cm", 0.0);
    goal_pos_uint16_encode_y_offset_cm_ =
      this->declare_parameter<double>("goal_pos_uint16_encode_y_offset_cm", 0.0);
    preferred_distance_cm_ = this->declare_parameter<int>("preferred_distance_cm", 100);
    distance_deadband_cm_ = this->declare_parameter<int>("distance_deadband_cm", 50);
    stop_when_no_target_ = this->declare_parameter<bool>("stop_when_no_target", true);
    allow_reverse_goal_ = this->declare_parameter<bool>("allow_reverse_goal", false);
    enable_goal_pos_raw_bridge_ =
      this->declare_parameter<bool>("enable_goal_pos_raw_bridge", true);
    goal_pos_raw_frame_ = this->declare_parameter<std::string>("goal_pos_raw_frame", "map");
    debug_export_point_pairs_ = this->declare_parameter<bool>("debug_export_point_pairs", true);
    debug_points_reference_frame_ =
      this->declare_parameter<std::string>("debug_points_reference_frame", "map");
    debug_area_header_file_ = this->declare_parameter<std::string>("debug_area_header_file", "");
    debug_point_pairs_output_file_ =
      this->declare_parameter<std::string>("debug_point_pairs_output_file", "");
    use_raw_goal_static_calibration_ =
      this->declare_parameter<bool>("use_raw_goal_static_calibration", false);
    raw_goal_calibration_model_ =
      this->declare_parameter<std::string>("raw_goal_calibration_model", "rigid");
    raw_goal_calibration_unit_ =
      this->declare_parameter<std::string>("raw_goal_calibration_unit", "cm");
    raw_goal_source_frame_ =
      this->declare_parameter<std::string>("raw_goal_source_frame", "official_map");
    raw_goal_target_frame_ =
      this->declare_parameter<std::string>("raw_goal_target_frame", "map");
    raw_goal_source_points_ =
      this->declare_parameter<std::vector<double>>(
      "raw_goal_source_points", std::vector<double>{});
    raw_goal_target_points_ =
      this->declare_parameter<std::vector<double>>(
      "raw_goal_target_points", std::vector<double>{});
    raw_goal_transform_matrix_ =
      this->declare_parameter<std::vector<double>>(
      "raw_goal_transform_matrix", std::vector<double>{});

    initializeRawGoalStaticCalibration();

    sub_target_rel_ = this->create_subscription<auto_aim_common::msg::RelativeTarget>(
      input_topic_,
      rclcpp::QoS(10),
      std::bind(&TargetRelToGoalPosNode::targetRelCallback, this, std::placeholders::_1));
    sub_goal_pos_raw_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      input_goal_pos_raw_topic_,
      rclcpp::QoS(10),
      std::bind(&TargetRelToGoalPosNode::goalPosRawCallback, this, std::placeholders::_1));

    if (publish_goal_pos_) {
      pub_goal_pos_ =
        this->create_publisher<std_msgs::msg::UInt16MultiArray>(output_goal_pos_topic_, 10);
    }
    if (publish_goal_pose_) {
      pub_goal_pose_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(output_goal_pose_topic_, 10);
    }

    if (publish_target_map_) {
      pub_target_map_ =
        this->create_publisher<geometry_msgs::msg::PointStamped>(output_target_map_topic_, 10);
    }

    if (std::abs(goal_pos_uint16_encode_x_scale_) <= 1e-9) {
      RCLCPP_WARN(
        this->get_logger(),
        "goal_pos_uint16_encode_x_scale is near zero, fallback to 1.0");
      goal_pos_uint16_encode_x_scale_ = 1.0;
    }
    if (std::abs(goal_pos_uint16_encode_y_scale_) <= 1e-9) {
      RCLCPP_WARN(
        this->get_logger(),
        "goal_pos_uint16_encode_y_scale is near zero, fallback to 1.0");
      goal_pos_uint16_encode_y_scale_ = 1.0;
    }

    const double inv_x_scale = 1.0 / goal_pos_uint16_encode_x_scale_;
    const double inv_y_scale = 1.0 / goal_pos_uint16_encode_y_scale_;
    const double inv_x_offset = -goal_pos_uint16_encode_x_offset_cm_ / goal_pos_uint16_encode_x_scale_;
    const double inv_y_offset = -goal_pos_uint16_encode_y_offset_cm_ / goal_pos_uint16_encode_y_scale_;

    RCLCPP_INFO(
      this->get_logger(),
      "Started target_rel -> goal bridge. in=%s goal_pos_out=%s publish_goal_pos=%s goal_pose_out=%s publish_goal_pose=%s map_frame=%s base_frame=%s "
      "fallback_base_frame=%s target_rel_default_frame=%s raw_goal_in=%s raw_goal_frame=%s invert_y_axis=%s y_axis_max_cm=%d preferred_distance_cm=%d "
      "goal_u16_encode=%s enc=[[%.6f,0,%.3f],[0,%.6f,%.3f]] dec=[[%.6f,0,%.3f],[0,%.6f,%.3f]] "
      "distance_deadband_cm=%d stop_when_no_target=%s allow_reverse_goal=%s "
      "use_raw_goal_static_calibration=%s model=%s source_frame=%s target_frame=%s",
      input_topic_.c_str(),
      output_goal_pos_topic_.c_str(),
      publish_goal_pos_ ? "true" : "false",
      output_goal_pose_topic_.c_str(),
      publish_goal_pose_ ? "true" : "false",
      map_frame_.c_str(),
      base_frame_.c_str(),
      fallback_base_frame_.c_str(),
      target_rel_default_frame_.c_str(),
      input_goal_pos_raw_topic_.c_str(),
      goal_pos_raw_frame_.c_str(),
      invert_y_axis_ ? "true" : "false",
      y_axis_max_cm_,
      preferred_distance_cm_,
      goal_pos_uint16_encode_enabled_ ? "true" : "false",
      goal_pos_uint16_encode_x_scale_,
      goal_pos_uint16_encode_x_offset_cm_,
      goal_pos_uint16_encode_y_scale_,
      goal_pos_uint16_encode_y_offset_cm_,
      inv_x_scale,
      inv_x_offset,
      inv_y_scale,
      inv_y_offset,
      distance_deadband_cm_,
      stop_when_no_target_ ? "true" : "false",
      allow_reverse_goal_ ? "true" : "false",
      use_raw_goal_static_calibration_ ? "true" : "false",
      raw_goal_calibration_model_.c_str(),
      raw_goal_source_frame_.c_str(),
      raw_goal_target_frame_.c_str());

    if (debug_export_point_pairs_) {
      debug_export_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TargetRelToGoalPosNode::onDebugExportTimer, this));
      RCLCPP_INFO(
        this->get_logger(),
        "TF debug export enabled. ref_frame=%s area_header=%s out=%s",
        debug_points_reference_frame_.c_str(),
        debug_area_header_file_.empty() ? "<empty>" : debug_area_header_file_.c_str(),
        debug_point_pairs_output_file_.empty() ? "<empty>" : debug_point_pairs_output_file_.c_str());
    }
  }

private:
  static void appendUniqueFrame(std::vector<std::string> & frames, const std::string & frame)
  {
    if (frame.empty()) {
      return;
    }
    if (std::find(frames.begin(), frames.end(), frame) == frames.end()) {
      frames.push_back(frame);
    }
  }

  std::vector<std::string> buildSourceCandidates(const std::string & msg_frame_id) const
  {
    const bool has_explicit_source_frame = use_msg_frame_id_ && !msg_frame_id.empty();
    std::vector<std::string> source_candidates;
    source_candidates.reserve(2);

    if (has_explicit_source_frame) {
      appendUniqueFrame(source_candidates, msg_frame_id);
      return source_candidates;
    }

    appendUniqueFrame(source_candidates, target_rel_default_frame_);
    // Compatibility fallback: if no explicit frame and default frame is missing, treat target_rel
    // as already in base frame.
    appendUniqueFrame(source_candidates, base_frame_);
    return source_candidates;
  }

  struct NamedPointCm
  {
    std::string name;
    int x_cm{0};
    int y_cm{0};
  };

  struct RawGoalPair
  {
    double sx_m{0.0};
    double sy_m{0.0};
    double tx_m{0.0};
    double ty_m{0.0};
  };

  static double triangleArea2(
    const std::pair<double, double> & p1,
    const std::pair<double, double> & p2,
    const std::pair<double, double> & p3)
  {
    return (p2.first - p1.first) * (p3.second - p1.second) -
           (p2.second - p1.second) * (p3.first - p1.first);
  }

  static double maxAbsTriangleArea2(const std::vector<std::pair<double, double>> & points)
  {
    double best = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      for (std::size_t j = i + 1; j < points.size(); ++j) {
        for (std::size_t k = j + 1; k < points.size(); ++k) {
          best = std::max(best, std::abs(triangleArea2(points[i], points[j], points[k])));
        }
      }
    }
    return best;
  }

  bool parseRawGoalPairs(std::vector<RawGoalPair> & pairs_out, double & unit_scale) const
  {
    pairs_out.clear();
    unit_scale = 0.01;
    const std::string unit = raw_goal_calibration_unit_;
    if (unit == "cm" || unit == "CM") {
      unit_scale = 0.01;
    } else if (unit == "m" || unit == "M") {
      unit_scale = 1.0;
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Raw-goal static calibration unit '%s' is invalid. Use 'cm' or 'm'.",
        raw_goal_calibration_unit_.c_str());
      return false;
    }

    if (raw_goal_source_points_.empty() || raw_goal_target_points_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Raw-goal static calibration enabled but point lists are empty.");
      return false;
    }
    if (raw_goal_source_points_.size() != raw_goal_target_points_.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Raw-goal static calibration point size mismatch: source=%zu target=%zu",
        raw_goal_source_points_.size(),
        raw_goal_target_points_.size());
      return false;
    }
    if ((raw_goal_source_points_.size() % 2) != 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Raw-goal static calibration points must be [x1,y1,x2,y2,...], got odd length=%zu",
        raw_goal_source_points_.size());
      return false;
    }

    const std::size_t pair_count = raw_goal_source_points_.size() / 2;
    if (pair_count < 3) {
      RCLCPP_WARN(
        this->get_logger(),
        "Raw-goal static calibration needs at least 3 point pairs, got %zu.",
        pair_count);
      return false;
    }

    pairs_out.reserve(pair_count);
    for (std::size_t i = 0; i < pair_count; ++i) {
      const std::size_t idx = i * 2;
      pairs_out.push_back(RawGoalPair{
        .sx_m = raw_goal_source_points_[idx] * unit_scale,
        .sy_m = raw_goal_source_points_[idx + 1] * unit_scale,
        .tx_m = raw_goal_target_points_[idx] * unit_scale,
        .ty_m = raw_goal_target_points_[idx + 1] * unit_scale});
    }

    std::vector<std::pair<double, double>> src_points;
    std::vector<std::pair<double, double>> dst_points;
    src_points.reserve(pairs_out.size());
    dst_points.reserve(pairs_out.size());
    for (const auto & p : pairs_out) {
      src_points.emplace_back(p.sx_m, p.sy_m);
      dst_points.emplace_back(p.tx_m, p.ty_m);
    }
    if (maxAbsTriangleArea2(src_points) <= 1e-12 || maxAbsTriangleArea2(dst_points) <= 1e-12) {
      RCLCPP_WARN(
        this->get_logger(),
        "Raw-goal static calibration points are degenerate (nearly collinear).");
      return false;
    }

    return true;
  }

  static bool solveLinear6x6(
    std::array<std::array<double, 6>, 6> a,
    std::array<double, 6> b,
    std::array<double, 6> & x)
  {
    constexpr int N = 6;
    for (int i = 0; i < N; ++i) {
      int pivot = i;
      double best = std::abs(a[i][i]);
      for (int r = i + 1; r < N; ++r) {
        const double v = std::abs(a[r][i]);
        if (v > best) {
          best = v;
          pivot = r;
        }
      }
      if (best <= 1e-12) {
        return false;
      }
      if (pivot != i) {
        std::swap(a[pivot], a[i]);
        std::swap(b[pivot], b[i]);
      }

      const double diag = a[i][i];
      for (int c = i; c < N; ++c) {
        a[i][c] /= diag;
      }
      b[i] /= diag;

      for (int r = 0; r < N; ++r) {
        if (r == i) {
          continue;
        }
        const double factor = a[r][i];
        if (std::abs(factor) <= 1e-15) {
          continue;
        }
        for (int c = i; c < N; ++c) {
          a[r][c] -= factor * a[i][c];
        }
        b[r] -= factor * b[i];
      }
    }

    x = b;
    return true;
  }

  static bool solveAffine2D(
    const std::vector<RawGoalPair> & pairs,
    double & m00,
    double & m01,
    double & m10,
    double & m11,
    double & tx,
    double & ty)
  {
    std::array<std::array<double, 6>, 6> ata{};
    std::array<double, 6> atb{};

    for (const auto & p : pairs) {
      const std::array<double, 6> row_x{p.sx_m, p.sy_m, 1.0, 0.0, 0.0, 0.0};
      const std::array<double, 6> row_y{0.0, 0.0, 0.0, p.sx_m, p.sy_m, 1.0};
      const double bx = p.tx_m;
      const double by = p.ty_m;

      for (int i = 0; i < 6; ++i) {
        atb[i] += row_x[i] * bx + row_y[i] * by;
        for (int j = 0; j < 6; ++j) {
          ata[i][j] += row_x[i] * row_x[j] + row_y[i] * row_y[j];
        }
      }
    }

    std::array<double, 6> x{};
    if (!solveLinear6x6(ata, atb, x)) {
      return false;
    }
    m00 = x[0];
    m01 = x[1];
    tx = x[2];
    m10 = x[3];
    m11 = x[4];
    ty = x[5];
    return true;
  }

  static bool solveRigid2D(
    const std::vector<RawGoalPair> & pairs,
    double & m00,
    double & m01,
    double & m10,
    double & m11,
    double & tx,
    double & ty)
  {
    const double n = static_cast<double>(pairs.size());
    double src_cx = 0.0;
    double src_cy = 0.0;
    double dst_cx = 0.0;
    double dst_cy = 0.0;
    for (const auto & p : pairs) {
      src_cx += p.sx_m;
      src_cy += p.sy_m;
      dst_cx += p.tx_m;
      dst_cy += p.ty_m;
    }
    src_cx /= n;
    src_cy /= n;
    dst_cx /= n;
    dst_cy /= n;

    double cross_term = 0.0;
    double dot_term = 0.0;
    for (const auto & p : pairs) {
      const double xs = p.sx_m - src_cx;
      const double ys = p.sy_m - src_cy;
      const double xt = p.tx_m - dst_cx;
      const double yt = p.ty_m - dst_cy;
      cross_term += xs * yt - ys * xt;
      dot_term += xs * xt + ys * yt;
    }
    if (std::abs(cross_term) + std::abs(dot_term) <= 1e-12) {
      return false;
    }

    const double yaw = std::atan2(cross_term, dot_term);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    m00 = c;
    m01 = -s;
    m10 = s;
    m11 = c;
    tx = dst_cx - (c * src_cx - s * src_cy);
    ty = dst_cy - (s * src_cx + c * src_cy);
    return true;
  }

  static void computeResidual(
    const std::vector<RawGoalPair> & pairs,
    const double m00,
    const double m01,
    const double m10,
    const double m11,
    const double tx,
    const double ty,
    double & rmse_m,
    double & max_err_m)
  {
    rmse_m = 0.0;
    max_err_m = 0.0;
    if (pairs.empty()) {
      return;
    }
    double sum_sq = 0.0;
    for (const auto & p : pairs) {
      const double px = m00 * p.sx_m + m01 * p.sy_m + tx;
      const double py = m10 * p.sx_m + m11 * p.sy_m + ty;
      const double err = std::hypot(px - p.tx_m, py - p.ty_m);
      sum_sq += err * err;
      max_err_m = std::max(max_err_m, err);
    }
    rmse_m = std::sqrt(sum_sq / static_cast<double>(pairs.size()));
  }


  bool initializeRawGoalMatrixCalibration()
  {
    double unit_scale = 0.01;
    const std::string unit = raw_goal_calibration_unit_;
    if (unit == "cm" || unit == "CM") {
      unit_scale = 0.01;
    } else if (unit == "m" || unit == "M") {
      unit_scale = 1.0;
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Raw-goal matrix calibration unit '%s' is invalid. Use 'cm' or 'm'.",
        raw_goal_calibration_unit_.c_str());
      return false;
    }

    if (raw_goal_transform_matrix_.size() != 16) {
      RCLCPP_WARN(
        this->get_logger(),
        "Raw-goal matrix calibration needs 16 row-major values, got %zu.",
        raw_goal_transform_matrix_.size());
      return false;
    }

    // Matrix is configured in raw_goal_calibration_unit, while runtime points are meters.
    raw_goal_calib_m00_ = raw_goal_transform_matrix_[0];
    raw_goal_calib_m01_ = raw_goal_transform_matrix_[1];
    raw_goal_calib_m10_ = raw_goal_transform_matrix_[4];
    raw_goal_calib_m11_ = raw_goal_transform_matrix_[5];
    raw_goal_calib_tx_m_ = raw_goal_transform_matrix_[3] * unit_scale;
    raw_goal_calib_ty_m_ = raw_goal_transform_matrix_[7] * unit_scale;
    raw_goal_static_calibration_ready_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Raw-goal static calibration ready. model=matrix unit=%s source_frame=%s target_frame=%s "
      "matrix4x4=[[%.9f, %.9f, %.9f, %.9f],[%.9f, %.9f, %.9f, %.9f],"
      "[%.9f, %.9f, %.9f, %.9f],[%.9f, %.9f, %.9f, %.9f]] "
      "effective_m=[[%.9f, %.9f, %.9f],[%.9f, %.9f, %.9f]]",
      raw_goal_calibration_unit_.c_str(),
      raw_goal_source_frame_.c_str(),
      raw_goal_target_frame_.c_str(),
      raw_goal_transform_matrix_[0],
      raw_goal_transform_matrix_[1],
      raw_goal_transform_matrix_[2],
      raw_goal_transform_matrix_[3],
      raw_goal_transform_matrix_[4],
      raw_goal_transform_matrix_[5],
      raw_goal_transform_matrix_[6],
      raw_goal_transform_matrix_[7],
      raw_goal_transform_matrix_[8],
      raw_goal_transform_matrix_[9],
      raw_goal_transform_matrix_[10],
      raw_goal_transform_matrix_[11],
      raw_goal_transform_matrix_[12],
      raw_goal_transform_matrix_[13],
      raw_goal_transform_matrix_[14],
      raw_goal_transform_matrix_[15],
      raw_goal_calib_m00_,
      raw_goal_calib_m01_,
      raw_goal_calib_tx_m_,
      raw_goal_calib_m10_,
      raw_goal_calib_m11_,
      raw_goal_calib_ty_m_);
    return true;
  }

  void initializeRawGoalStaticCalibration()
  {
    raw_goal_static_calibration_ready_ = false;
    if (!use_raw_goal_static_calibration_) {
      return;
    }

    if (raw_goal_target_frame_ != map_frame_) {
      RCLCPP_WARN(
        this->get_logger(),
        "raw_goal_target_frame='%s' differs from map_frame='%s'. Converted output is still "
        "published as map_frame.",
        raw_goal_target_frame_.c_str(),
        map_frame_.c_str());
    }

    const std::string model = raw_goal_calibration_model_;
    const bool wants_matrix =
      (model == "matrix" || model == "MATRIX" || model == "matrix_4x4");
    if (wants_matrix) {
      if (!initializeRawGoalMatrixCalibration()) {
        RCLCPP_WARN(
          this->get_logger(),
          "Raw-goal static calibration matrix setup failed.");
      }
      return;
    }

    std::vector<RawGoalPair> pairs;
    double unit_scale = 0.01;
    if (!parseRawGoalPairs(pairs, unit_scale)) {
      return;
    }

    bool ok = false;
    std::string solved_model = "rigid";
    const bool wants_affine = (model == "affine" || model == "AFFINE" || model == "affine_2d");
    const bool wants_rigid = (model == "rigid" || model == "RIGID" || model == "rigid_2d");
    const bool wants_auto = (model == "auto" || model == "AUTO");
    if (wants_affine) {
      solved_model = "affine";
      ok = solveAffine2D(
        pairs,
        raw_goal_calib_m00_,
        raw_goal_calib_m01_,
        raw_goal_calib_m10_,
        raw_goal_calib_m11_,
        raw_goal_calib_tx_m_,
        raw_goal_calib_ty_m_);
    } else if (wants_rigid) {
      solved_model = "rigid";
      ok = solveRigid2D(
        pairs,
        raw_goal_calib_m00_,
        raw_goal_calib_m01_,
        raw_goal_calib_m10_,
        raw_goal_calib_m11_,
        raw_goal_calib_tx_m_,
        raw_goal_calib_ty_m_);
    } else if (wants_auto) {
      double aff_m00 = 0.0;
      double aff_m01 = 0.0;
      double aff_m10 = 0.0;
      double aff_m11 = 0.0;
      double aff_tx = 0.0;
      double aff_ty = 0.0;
      const bool ok_affine = solveAffine2D(
        pairs, aff_m00, aff_m01, aff_m10, aff_m11, aff_tx, aff_ty);

      double rig_m00 = 0.0;
      double rig_m01 = 0.0;
      double rig_m10 = 0.0;
      double rig_m11 = 0.0;
      double rig_tx = 0.0;
      double rig_ty = 0.0;
      const bool ok_rigid = solveRigid2D(
        pairs, rig_m00, rig_m01, rig_m10, rig_m11, rig_tx, rig_ty);

      if (ok_affine && !ok_rigid) {
        solved_model = "affine";
        ok = true;
        raw_goal_calib_m00_ = aff_m00;
        raw_goal_calib_m01_ = aff_m01;
        raw_goal_calib_m10_ = aff_m10;
        raw_goal_calib_m11_ = aff_m11;
        raw_goal_calib_tx_m_ = aff_tx;
        raw_goal_calib_ty_m_ = aff_ty;
      } else if (!ok_affine && ok_rigid) {
        solved_model = "rigid";
        ok = true;
        raw_goal_calib_m00_ = rig_m00;
        raw_goal_calib_m01_ = rig_m01;
        raw_goal_calib_m10_ = rig_m10;
        raw_goal_calib_m11_ = rig_m11;
        raw_goal_calib_tx_m_ = rig_tx;
        raw_goal_calib_ty_m_ = rig_ty;
      } else if (ok_affine && ok_rigid) {
        double aff_rmse_m = 0.0;
        double aff_max_m = 0.0;
        computeResidual(
          pairs, aff_m00, aff_m01, aff_m10, aff_m11, aff_tx, aff_ty, aff_rmse_m, aff_max_m);
        double rig_rmse_m = 0.0;
        double rig_max_m = 0.0;
        computeResidual(
          pairs, rig_m00, rig_m01, rig_m10, rig_m11, rig_tx, rig_ty, rig_rmse_m, rig_max_m);

        const bool choose_affine = (aff_rmse_m + 1e-9) < rig_rmse_m;
        solved_model = choose_affine ? "affine" : "rigid";
        ok = true;
        raw_goal_calib_m00_ = choose_affine ? aff_m00 : rig_m00;
        raw_goal_calib_m01_ = choose_affine ? aff_m01 : rig_m01;
        raw_goal_calib_m10_ = choose_affine ? aff_m10 : rig_m10;
        raw_goal_calib_m11_ = choose_affine ? aff_m11 : rig_m11;
        raw_goal_calib_tx_m_ = choose_affine ? aff_tx : rig_tx;
        raw_goal_calib_ty_m_ = choose_affine ? aff_ty : rig_ty;
        RCLCPP_INFO(
          this->get_logger(),
          "Raw-goal auto model pick: affine_rmse=%.4fm rigid_rmse=%.4fm choose=%s",
          aff_rmse_m,
          rig_rmse_m,
          solved_model.c_str());
      }
    } else {
      solved_model = "rigid";
      ok = solveRigid2D(
        pairs,
        raw_goal_calib_m00_,
        raw_goal_calib_m01_,
        raw_goal_calib_m10_,
        raw_goal_calib_m11_,
        raw_goal_calib_tx_m_,
        raw_goal_calib_ty_m_);
      if (!ok) {
        RCLCPP_WARN(
          this->get_logger(),
          "Unknown raw_goal_calibration_model='%s', fallback to rigid failed.",
          raw_goal_calibration_model_.c_str());
      }
    }

    if (!ok) {
      RCLCPP_WARN(
        this->get_logger(),
        "Raw-goal static calibration solve failed. model=%s",
        raw_goal_calibration_model_.c_str());
      return;
    }

    double rmse_m = 0.0;
    double max_err_m = 0.0;
    computeResidual(
      pairs,
      raw_goal_calib_m00_,
      raw_goal_calib_m01_,
      raw_goal_calib_m10_,
      raw_goal_calib_m11_,
      raw_goal_calib_tx_m_,
      raw_goal_calib_ty_m_,
      rmse_m,
      max_err_m);

    raw_goal_static_calibration_ready_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Raw-goal static calibration ready. model=%s (request=%s) points=%zu unit=%s source_frame=%s "
      "target_frame=%s matrix=[[%.6f, %.6f, %.6f],[%.6f, %.6f, %.6f]] rmse=%.4fm max=%.4fm",
      solved_model.c_str(),
      raw_goal_calibration_model_.c_str(),
      pairs.size(),
      raw_goal_calibration_unit_.c_str(),
      raw_goal_source_frame_.c_str(),
      raw_goal_target_frame_.c_str(),
      raw_goal_calib_m00_,
      raw_goal_calib_m01_,
      raw_goal_calib_tx_m_,
      raw_goal_calib_m10_,
      raw_goal_calib_m11_,
      raw_goal_calib_ty_m_,
      rmse_m,
      max_err_m);
  }

  bool parseAreaHeaderPoints(std::vector<NamedPointCm> & points_out) const
  {
    points_out.clear();
    if (debug_area_header_file_.empty()) {
      return false;
    }

    std::ifstream ifs(debug_area_header_file_);
    if (!ifs.is_open()) {
      RCLCPP_WARN(
        this->get_logger(),
        "TF debug export: cannot open area header file: %s",
        debug_area_header_file_.c_str());
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
        this->get_logger(),
        "TF debug export: no Location<std::uint16_t> points found in %s",
        debug_area_header_file_.c_str());
      return false;
    }

    return true;
  }

  bool lookupTransformWithCandidates(
    const std::vector<std::string> & source_candidates,
    const rclcpp::Time & transform_time,
    geometry_msgs::msg::TransformStamped & tf_out,
    std::string & resolved_source_frame,
    std::string & last_tf_error) const
  {
    for (const auto & source_frame : source_candidates) {
      try {
        tf_out = tf_buffer_.lookupTransform(
          map_frame_,
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

  bool exportDebugPointPairs(
    const std::string & tf_ready_source_frame,
    const geometry_msgs::msg::TransformStamped & tf_map_ref)
  {
    std::vector<NamedPointCm> points_cm;
    if (!parseAreaHeaderPoints(points_cm)) {
      return false;
    }
    if (debug_point_pairs_output_file_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "TF debug export: debug_point_pairs_output_file is empty.");
      return false;
    }

    std::filesystem::path out_path(debug_point_pairs_output_file_);
    if (out_path.has_parent_path()) {
      std::error_code ec;
      std::filesystem::create_directories(out_path.parent_path(), ec);
      if (ec) {
        RCLCPP_WARN(
          this->get_logger(),
          "TF debug export: failed to create parent dir for %s: %s",
          debug_point_pairs_output_file_.c_str(),
          ec.message().c_str());
        return false;
      }
    }

    std::ofstream ofs(debug_point_pairs_output_file_, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
      RCLCPP_WARN(
        this->get_logger(),
        "TF debug export: failed to open output file: %s",
        debug_point_pairs_output_file_.c_str());
      return false;
    }

    ofs << "meta:\n";
    ofs << "  generated_ns: " << this->now().nanoseconds() << "\n";
    ofs << "  map_frame: " << map_frame_ << "\n";
    ofs << "  tf_ready_source_frame: " << tf_ready_source_frame << "\n";
    ofs << "  reference_frame: " << debug_points_reference_frame_ << "\n";
    ofs << "  area_header_file: " << debug_area_header_file_ << "\n";
    ofs << "points:\n";

    for (const auto & point : points_cm) {
      geometry_msgs::msg::PointStamped in;
      in.header.frame_id = debug_points_reference_frame_;
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

  void onDebugExportTimer()
  {
    if (!debug_export_point_pairs_ || debug_point_pairs_exported_) {
      if (debug_export_timer_) {
        debug_export_timer_->cancel();
      }
      return;
    }

    const auto source_candidates = buildSourceCandidates("");
    geometry_msgs::msg::TransformStamped tf_map_source;
    std::string last_tf_error;
    std::string resolved_source_frame;
    const rclcpp::Time latest_time(0, 0, this->get_clock()->get_clock_type());
    if (!lookupTransformWithCandidates(
        source_candidates, latest_time, tf_map_source, resolved_source_frame, last_tf_error))
    {
      if (!debug_tf_missing_logged_once_) {
        std::ostringstream oss;
        for (const auto & frame : source_candidates) {
          oss << frame << " ";
        }
        RCLCPP_WARN(
          this->get_logger(),
          "TF frame chain not ready yet (%s <- [%s]). Last error: %s",
          map_frame_.c_str(),
          oss.str().c_str(),
          last_tf_error.c_str());
        debug_tf_missing_logged_once_ = true;
      }
      return;
    }

    if (!debug_tf_ready_logged_once_) {
      RCLCPP_INFO(
        this->get_logger(),
        "TF frame chain ready: %s <- %s",
        map_frame_.c_str(),
        resolved_source_frame.c_str());
      debug_tf_ready_logged_once_ = true;
    }

    geometry_msgs::msg::TransformStamped tf_map_ref;
    std::string tf_ref_error;
    std::string tf_ref_resolved;
    if (!lookupTransformWithCandidates(
        {debug_points_reference_frame_},
        latest_time,
        tf_map_ref,
        tf_ref_resolved,
        tf_ref_error))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "TF debug export: failed to resolve reference frame (%s <- %s): %s",
        map_frame_.c_str(),
        debug_points_reference_frame_.c_str(),
        tf_ref_error.c_str());
      if (debug_export_timer_) {
        debug_export_timer_->cancel();
      }
      return;
    }

    if (exportDebugPointPairs(resolved_source_frame, tf_map_ref)) {
      debug_point_pairs_exported_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "TF debug point pairs exported: %s",
        debug_point_pairs_output_file_.c_str());
    }
    if (debug_export_timer_) {
      debug_export_timer_->cancel();
    }
  }

  bool buildRelativeGoalPoint(
    const auto_aim_common::msg::RelativeTarget & msg,
    geometry_msgs::msg::Point & point_out)
  {
    point_out.x = 0.0;
    point_out.y = 0.0;
    point_out.z = 0.0;

    if (!msg.valid) {
      return stop_when_no_target_;
    }

    const double target_x = static_cast<double>(msg.x);
    const double target_y = static_cast<double>(msg.y);
    if (!std::isfinite(target_x) || !std::isfinite(target_y)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
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

    const double preferred_distance_m = static_cast<double>(preferred_distance_cm_) * 0.01;
    const double deadband_m = static_cast<double>(distance_deadband_cm_) * 0.01;
    const double move_distance_m = planar_distance_m - preferred_distance_m;

    if (std::abs(move_distance_m) <= deadband_m) {
      return true;
    }

    if (move_distance_m < 0.0 && !allow_reverse_goal_) {
      return true;
    }

    const double scale = move_distance_m / planar_distance_m;
    point_out.x = target_x * scale;
    point_out.y = target_y * scale;
    return true;
  }

  bool transformRelativePointToMap(
    const geometry_msgs::msg::Point & point_rel,
    const std::vector<std::string> & source_candidates,
    const rclcpp::Time & transform_time,
    geometry_msgs::msg::PointStamped & point_map,
    std::string & resolved_source_frame,
    std::string & last_tf_error)
  {
    geometry_msgs::msg::PointStamped point_source;
    point_source.header.stamp = transform_time;
    point_source.point = point_rel;

    for (const auto & source_frame : source_candidates) {
      point_source.header.frame_id = source_frame;
      try {
        const geometry_msgs::msg::TransformStamped tf_map_source =
          tf_buffer_.lookupTransform(
          map_frame_,
          source_frame,
          transform_time,
          rclcpp::Duration::from_seconds(0.05));
        tf2::doTransform(point_source, point_map, tf_map_source);
        resolved_source_frame = source_frame;
        return true;
      } catch (const tf2::TransformException & ex) {
        last_tf_error = "lookup " + map_frame_ + " <- " + source_frame + " failed: " + ex.what();
      }
    }

    return false;
  }

  void publishMapPointAsGoal(
    const geometry_msgs::msg::Point & point_map,
    const std::string & resolved_source_frame,
    const rclcpp::Time & stamp = rclcpp::Time(0, 0, RCL_ROS_TIME))
  {
    publishMapPointAsGoalPose(point_map, stamp);
    if (!publish_goal_pos_ || !pub_goal_pos_) {
      return;
    }

    long x_cm = std::lround(point_map.x * 100.0);
    long y_cm = std::lround(point_map.y * 100.0);

    if (invert_y_axis_) {
      y_cm = static_cast<long>(y_axis_max_cm_) - y_cm;
    }

    long encoded_x_cm = x_cm;
    long encoded_y_cm = y_cm;
    if (goal_pos_uint16_encode_enabled_) {
      encoded_x_cm = std::lround(
        goal_pos_uint16_encode_x_scale_ * static_cast<double>(x_cm) +
        goal_pos_uint16_encode_x_offset_cm_);
      encoded_y_cm = std::lround(
        goal_pos_uint16_encode_y_scale_ * static_cast<double>(y_cm) +
        goal_pos_uint16_encode_y_offset_cm_);
    }

    const long kU16Min = 0L;
    const long kU16Max = static_cast<long>(std::numeric_limits<std::uint16_t>::max());
    if (
      encoded_x_cm < kU16Min || encoded_x_cm > kU16Max || encoded_y_cm < kU16Min ||
      encoded_y_cm > kU16Max)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
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
    pub_goal_pos_->publish(out);
  }

  void publishMapPointAsGoalPose(
    const geometry_msgs::msg::Point & point_map,
    const rclcpp::Time & stamp)
  {
    if (!publish_goal_pose_ || !pub_goal_pose_) {
      return;
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = map_frame_;
    out.header.stamp = (stamp.nanoseconds() == 0) ? this->now() : stamp;
    out.pose.position = point_map;
    out.pose.orientation.w = 1.0;
    pub_goal_pose_->publish(out);
  }

  void goalPosRawCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (!enable_goal_pos_raw_bridge_) {
      return;
    }
    if (!msg || msg->data.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Drop invalid goal_pos_raw: data size=%zu (need >=2)",
        msg ? msg->data.size() : 0);
      return;
    }

    if (use_raw_goal_static_calibration_) {
      if (!raw_goal_static_calibration_ready_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Drop goal_pos_raw because static calibration is enabled but not ready.");
        return;
      }

      const double raw_x_m = static_cast<double>(msg->data[0]) * 0.01;
      const double raw_y_m = static_cast<double>(msg->data[1]) * 0.01;
      geometry_msgs::msg::Point point_map;
      point_map.x =
        raw_goal_calib_m00_ * raw_x_m + raw_goal_calib_m01_ * raw_y_m + raw_goal_calib_tx_m_;
      point_map.y =
        raw_goal_calib_m10_ * raw_x_m + raw_goal_calib_m11_ * raw_y_m + raw_goal_calib_ty_m_;
      point_map.z = 0.0;
      publishMapPointAsGoal(point_map, "raw_goal_static_calibration", this->now());
      return;
    }

    geometry_msgs::msg::PointStamped point_in;
    point_in.header.frame_id = goal_pos_raw_frame_;
    point_in.header.stamp = this->now();
    point_in.point.x = static_cast<double>(msg->data[0]) * 0.01;
    point_in.point.y = static_cast<double>(msg->data[1]) * 0.01;
    point_in.point.z = 0.0;

    geometry_msgs::msg::PointStamped point_map;
    if (goal_pos_raw_frame_ == map_frame_) {
      point_map = point_in;
    } else {
      try {
        const geometry_msgs::msg::TransformStamped tf_map_raw =
          tf_buffer_.lookupTransform(
          map_frame_,
          goal_pos_raw_frame_,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(0.05));
        tf2::doTransform(point_in, point_map, tf_map_raw);
        if (!goal_raw_tf_ready_logged_once_) {
          RCLCPP_INFO(
            this->get_logger(),
            "Raw goal TF chain ready: %s <- %s",
            map_frame_.c_str(),
            goal_pos_raw_frame_.c_str());
          goal_raw_tf_ready_logged_once_ = true;
        }
      } catch (const tf2::TransformException & ex) {
        if (!goal_raw_tf_missing_logged_once_) {
          RCLCPP_WARN(
            this->get_logger(),
            "Raw goal TF chain not ready (%s <- %s): %s",
            map_frame_.c_str(),
            goal_pos_raw_frame_.c_str(),
            ex.what());
          goal_raw_tf_missing_logged_once_ = true;
        }
        return;
      }
    }

    publishMapPointAsGoal(point_map.point, goal_pos_raw_frame_, point_map.header.stamp);
  }

  void targetRelCallback(const auto_aim_common::msg::RelativeTarget::SharedPtr msg)
  {
    geometry_msgs::msg::Point relative_goal_point;
    if (!buildRelativeGoalPoint(*msg, relative_goal_point)) {
      return;
    }

    const auto source_candidates = buildSourceCandidates(msg->header.frame_id);
    const rclcpp::Time transform_time(msg->header.stamp);

    geometry_msgs::msg::PointStamped point_map;
    std::string last_tf_error;
    std::string resolved_source_frame;
    if (!transformRelativePointToMap(
        relative_goal_point,
        source_candidates,
        transform_time,
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
        map_frame_.c_str(),
        last_tf_error.c_str());
      return;
    }

    point_map.header.frame_id = map_frame_;
    point_map.header.stamp =
      (transform_time.nanoseconds() == 0) ? this->now() : transform_time;

    if (publish_target_map_ && pub_target_map_) {
      pub_target_map_->publish(point_map);
    }
    publishMapPointAsGoal(point_map.point, resolved_source_frame, point_map.header.stamp);
  }

  std::string input_topic_;
  std::string input_goal_pos_raw_topic_;
  std::string output_goal_pos_topic_;
  std::string output_goal_pose_topic_;
  std::string output_target_map_topic_;

  std::string map_frame_;
  std::string base_frame_;
  std::string fallback_base_frame_;
  bool use_msg_frame_id_{true};
  std::string target_rel_default_frame_{"gimbal_world"};

  bool publish_target_map_{true};
  bool publish_goal_pos_{false};
  bool publish_goal_pose_{true};
  bool invert_y_axis_{false};
  int y_axis_max_cm_{1500};
  bool goal_pos_uint16_encode_enabled_{false};
  double goal_pos_uint16_encode_x_scale_{1.0};
  double goal_pos_uint16_encode_y_scale_{1.0};
  double goal_pos_uint16_encode_x_offset_cm_{0.0};
  double goal_pos_uint16_encode_y_offset_cm_{0.0};
  int preferred_distance_cm_{100};
  int distance_deadband_cm_{50};
  bool stop_when_no_target_{true};
  bool allow_reverse_goal_{false};
  bool enable_goal_pos_raw_bridge_{true};
  std::string goal_pos_raw_frame_{"map"};
  bool use_raw_goal_static_calibration_{false};
  std::string raw_goal_calibration_model_{"rigid"};
  std::string raw_goal_calibration_unit_{"cm"};
  std::string raw_goal_source_frame_{"official_map"};
  std::string raw_goal_target_frame_{"map"};
  std::vector<double> raw_goal_source_points_{};
  std::vector<double> raw_goal_target_points_{};
  std::vector<double> raw_goal_transform_matrix_{};
  bool raw_goal_static_calibration_ready_{false};
  double raw_goal_calib_m00_{1.0};
  double raw_goal_calib_m01_{0.0};
  double raw_goal_calib_m10_{0.0};
  double raw_goal_calib_m11_{1.0};
  double raw_goal_calib_tx_m_{0.0};
  double raw_goal_calib_ty_m_{0.0};
  bool debug_export_point_pairs_{true};
  std::string debug_points_reference_frame_{"map"};
  std::string debug_area_header_file_;
  std::string debug_point_pairs_output_file_;
  bool debug_tf_ready_logged_once_{false};
  bool debug_tf_missing_logged_once_{false};
  bool goal_raw_tf_ready_logged_once_{false};
  bool goal_raw_tf_missing_logged_once_{false};
  bool debug_point_pairs_exported_{false};
  rclcpp::TimerBase::SharedPtr debug_export_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<auto_aim_common::msg::RelativeTarget>::SharedPtr sub_target_rel_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_goal_pos_raw_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_goal_pos_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_map_;
};
}  // namespace navi_tf_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navi_tf_bridge::TargetRelToGoalPosNode>());
  rclcpp::shutdown();
  return 0;
}
