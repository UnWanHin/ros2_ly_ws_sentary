#include "navi_tf_bridge/pointer_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace navi_tf_bridge
{
namespace
{

double triangleArea2(
  const std::pair<double, double> & p1,
  const std::pair<double, double> & p2,
  const std::pair<double, double> & p3)
{
  return (p2.first - p1.first) * (p3.second - p1.second) -
         (p2.second - p1.second) * (p3.first - p1.first);
}

double maxAbsTriangleArea2(const std::vector<std::pair<double, double>> & points)
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

bool solveLinear6x6(
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

bool solveAffine2D(
  const std::vector<PointerSolver::RawGoalPair> & pairs,
  PointerSolver::Transform2D & transform)
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
  transform.m00 = x[0];
  transform.m01 = x[1];
  transform.tx_m = x[2];
  transform.m10 = x[3];
  transform.m11 = x[4];
  transform.ty_m = x[5];
  return true;
}

bool solveRigid2D(
  const std::vector<PointerSolver::RawGoalPair> & pairs,
  PointerSolver::Transform2D & transform)
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
  transform.m00 = c;
  transform.m01 = -s;
  transform.m10 = s;
  transform.m11 = c;
  transform.tx_m = dst_cx - (c * src_cx - s * src_cy);
  transform.ty_m = dst_cy - (s * src_cx + c * src_cy);
  return true;
}

void computeResidual(
  const std::vector<PointerSolver::RawGoalPair> & pairs,
  const PointerSolver::Transform2D & transform,
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
    const double px = transform.m00 * p.sx_m + transform.m01 * p.sy_m + transform.tx_m;
    const double py = transform.m10 * p.sx_m + transform.m11 * p.sy_m + transform.ty_m;
    const double err = std::hypot(px - p.tx_m, py - p.ty_m);
    sum_sq += err * err;
    max_err_m = std::max(max_err_m, err);
  }
  rmse_m = std::sqrt(sum_sq / static_cast<double>(pairs.size()));
}

}  // namespace

PointerSolver::PointerSolver(Config config)
: config_(std::move(config))
{
}

void PointerSolver::setConfig(Config config)
{
  config_ = std::move(config);
  ready_ = false;
  transform_ = Transform2D{};
}

const PointerSolver::Config & PointerSolver::config() const
{
  return config_;
}

bool PointerSolver::ready() const
{
  return ready_;
}

const PointerSolver::Transform2D & PointerSolver::transform() const
{
  return transform_;
}

geometry_msgs::msg::Point PointerSolver::applyMeters(
  const double x_m,
  const double y_m,
  const double z_m) const
{
  geometry_msgs::msg::Point point;
  point.x = transform_.m00 * x_m + transform_.m01 * y_m + transform_.tx_m;
  point.y = transform_.m10 * x_m + transform_.m11 * y_m + transform_.ty_m;
  point.z = z_m;
  return point;
}

bool PointerSolver::initializeMatrix(rclcpp::Node & node)
{
  double unit_scale = 0.01;
  const std::string unit = config_.unit;
  if (unit == "cm" || unit == "CM") {
    unit_scale = 0.01;
  } else if (unit == "m" || unit == "M") {
    unit_scale = 1.0;
  } else {
    RCLCPP_WARN(
      node.get_logger(),
      "Raw-goal matrix calibration unit '%s' is invalid. Use 'cm' or 'm'.",
      config_.unit.c_str());
    return false;
  }

  if (config_.transform_matrix.size() != 16) {
    RCLCPP_WARN(
      node.get_logger(),
      "Raw-goal matrix calibration needs 16 row-major values, got %zu.",
      config_.transform_matrix.size());
    return false;
  }

  transform_.m00 = config_.transform_matrix[0];
  transform_.m01 = config_.transform_matrix[1];
  transform_.m10 = config_.transform_matrix[4];
  transform_.m11 = config_.transform_matrix[5];
  transform_.tx_m = config_.transform_matrix[3] * unit_scale;
  transform_.ty_m = config_.transform_matrix[7] * unit_scale;
  ready_ = true;

  RCLCPP_INFO(
    node.get_logger(),
    "Raw-goal static calibration ready. model=matrix unit=%s source_frame=%s target_frame=%s "
    "matrix4x4=[[%.9f, %.9f, %.9f, %.9f],[%.9f, %.9f, %.9f, %.9f],"
    "[%.9f, %.9f, %.9f, %.9f],[%.9f, %.9f, %.9f, %.9f]] "
    "effective_m=[[%.9f, %.9f, %.9f],[%.9f, %.9f, %.9f]]",
    config_.unit.c_str(),
    config_.source_frame.c_str(),
    config_.target_frame.c_str(),
    config_.transform_matrix[0],
    config_.transform_matrix[1],
    config_.transform_matrix[2],
    config_.transform_matrix[3],
    config_.transform_matrix[4],
    config_.transform_matrix[5],
    config_.transform_matrix[6],
    config_.transform_matrix[7],
    config_.transform_matrix[8],
    config_.transform_matrix[9],
    config_.transform_matrix[10],
    config_.transform_matrix[11],
    config_.transform_matrix[12],
    config_.transform_matrix[13],
    config_.transform_matrix[14],
    config_.transform_matrix[15],
    transform_.m00,
    transform_.m01,
    transform_.tx_m,
    transform_.m10,
    transform_.m11,
    transform_.ty_m);
  return true;
}

bool PointerSolver::parsePairs(
  rclcpp::Node & node,
  std::vector<RawGoalPair> & pairs_out) const
{
  pairs_out.clear();
  double unit_scale = 0.01;
  const std::string unit = config_.unit;
  if (unit == "cm" || unit == "CM") {
    unit_scale = 0.01;
  } else if (unit == "m" || unit == "M") {
    unit_scale = 1.0;
  } else {
    RCLCPP_WARN(
      node.get_logger(),
      "Raw-goal static calibration unit '%s' is invalid. Use 'cm' or 'm'.",
      config_.unit.c_str());
    return false;
  }

  if (config_.source_points.empty() || config_.target_points.empty()) {
    RCLCPP_WARN(
      node.get_logger(),
      "Raw-goal static calibration enabled but point lists are empty.");
    return false;
  }
  if (config_.source_points.size() != config_.target_points.size()) {
    RCLCPP_WARN(
      node.get_logger(),
      "Raw-goal static calibration point size mismatch: source=%zu target=%zu",
      config_.source_points.size(),
      config_.target_points.size());
    return false;
  }
  if ((config_.source_points.size() % 2) != 0) {
    RCLCPP_WARN(
      node.get_logger(),
      "Raw-goal static calibration points must be [x1,y1,x2,y2,...], got odd length=%zu",
      config_.source_points.size());
    return false;
  }

  const std::size_t pair_count = config_.source_points.size() / 2;
  if (pair_count < 3) {
    RCLCPP_WARN(
      node.get_logger(),
      "Raw-goal static calibration needs at least 3 point pairs, got %zu.",
      pair_count);
    return false;
  }

  pairs_out.reserve(pair_count);
  for (std::size_t i = 0; i < pair_count; ++i) {
    const std::size_t idx = i * 2;
    pairs_out.push_back(RawGoalPair{
      .sx_m = config_.source_points[idx] * unit_scale,
      .sy_m = config_.source_points[idx + 1] * unit_scale,
      .tx_m = config_.target_points[idx] * unit_scale,
      .ty_m = config_.target_points[idx + 1] * unit_scale});
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
      node.get_logger(),
      "Raw-goal static calibration points are degenerate (nearly collinear).");
    return false;
  }

  return true;
}

bool PointerSolver::initialize(rclcpp::Node & node)
{
  ready_ = false;
  transform_ = Transform2D{};
  if (!config_.enabled) {
    return false;
  }

  if (config_.target_frame != config_.map_frame) {
    RCLCPP_WARN(
      node.get_logger(),
      "raw_goal_target_frame='%s' differs from map_frame='%s'. Converted output is still "
      "published as map_frame.",
      config_.target_frame.c_str(),
      config_.map_frame.c_str());
  }

  const std::string model = config_.model;
  const bool wants_matrix =
    (model == "matrix" || model == "MATRIX" || model == "matrix_4x4");
  if (wants_matrix) {
    if (!initializeMatrix(node)) {
      RCLCPP_WARN(node.get_logger(), "Raw-goal static calibration matrix setup failed.");
    }
    return ready_;
  }

  std::vector<RawGoalPair> pairs;
  if (!parsePairs(node, pairs)) {
    return false;
  }

  bool ok = false;
  std::string solved_model = "rigid";
  const bool wants_affine = (model == "affine" || model == "AFFINE" || model == "affine_2d");
  const bool wants_rigid = (model == "rigid" || model == "RIGID" || model == "rigid_2d");
  const bool wants_auto = (model == "auto" || model == "AUTO");
  if (wants_affine) {
    solved_model = "affine";
    ok = solveAffine2D(pairs, transform_);
  } else if (wants_rigid) {
    solved_model = "rigid";
    ok = solveRigid2D(pairs, transform_);
  } else if (wants_auto) {
    Transform2D affine;
    const bool ok_affine = solveAffine2D(pairs, affine);

    Transform2D rigid;
    const bool ok_rigid = solveRigid2D(pairs, rigid);

    if (ok_affine && !ok_rigid) {
      solved_model = "affine";
      ok = true;
      transform_ = affine;
    } else if (!ok_affine && ok_rigid) {
      solved_model = "rigid";
      ok = true;
      transform_ = rigid;
    } else if (ok_affine && ok_rigid) {
      double aff_rmse_m = 0.0;
      double aff_max_m = 0.0;
      computeResidual(pairs, affine, aff_rmse_m, aff_max_m);
      double rig_rmse_m = 0.0;
      double rig_max_m = 0.0;
      computeResidual(pairs, rigid, rig_rmse_m, rig_max_m);

      const bool choose_affine = (aff_rmse_m + 1e-9) < rig_rmse_m;
      solved_model = choose_affine ? "affine" : "rigid";
      ok = true;
      transform_ = choose_affine ? affine : rigid;
      RCLCPP_INFO(
        node.get_logger(),
        "Raw-goal auto model pick: affine_rmse=%.4fm rigid_rmse=%.4fm choose=%s",
        aff_rmse_m,
        rig_rmse_m,
        solved_model.c_str());
    }
  } else {
    solved_model = "rigid";
    ok = solveRigid2D(pairs, transform_);
    if (!ok) {
      RCLCPP_WARN(
        node.get_logger(),
        "Unknown raw_goal_calibration_model='%s', fallback to rigid failed.",
        config_.model.c_str());
    }
  }

  if (!ok) {
    RCLCPP_WARN(
      node.get_logger(),
      "Raw-goal static calibration solve failed. model=%s",
      config_.model.c_str());
    return false;
  }

  double rmse_m = 0.0;
  double max_err_m = 0.0;
  computeResidual(pairs, transform_, rmse_m, max_err_m);

  ready_ = true;
  RCLCPP_INFO(
    node.get_logger(),
    "Raw-goal static calibration ready. model=%s (request=%s) points=%zu unit=%s source_frame=%s "
    "target_frame=%s matrix=[[%.6f, %.6f, %.6f],[%.6f, %.6f, %.6f]] rmse=%.4fm max=%.4fm",
    solved_model.c_str(),
    config_.model.c_str(),
    pairs.size(),
    config_.unit.c_str(),
    config_.source_frame.c_str(),
    config_.target_frame.c_str(),
    transform_.m00,
    transform_.m01,
    transform_.tx_m,
    transform_.m10,
    transform_.m11,
    transform_.ty_m,
    rmse_m,
    max_err_m);
  return true;
}

}  // namespace navi_tf_bridge
