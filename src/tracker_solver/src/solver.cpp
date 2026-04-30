// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "solver/solver.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <Logger/Logger.hpp>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/rclcpp.hpp> // [ROS 2]
#include <fmt/format.h>      // [ROS 2]

using namespace LangYa;
using namespace ly_auto_aim;

// [ROS 2] 1. 定义全局节点变量
namespace ly_auto_aim::inline solver {
    rclcpp::Node::SharedPtr global_tracker_solver_node = nullptr;
}

// [ROS 2] 2. 本地 roslog 适配
namespace roslog_adapter {
    static rclcpp::Logger get_logger() {
        if (ly_auto_aim::solver::global_tracker_solver_node) {
            return ly_auto_aim::solver::global_tracker_solver_node->get_logger();
        }
        return rclcpp::get_logger("tracker_solver");
    }
    template <typename... Args>
    void debug(const char* format_str, const Args&... args) {
        RCLCPP_DEBUG(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void warn(const char* format_str, const Args&... args) {
        RCLCPP_WARN(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void info(const char* format_str, const Args&... args) {
        RCLCPP_INFO(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void error(const char* format_str, const Args&... args) {
        RCLCPP_ERROR(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
}
// 宏替换
#define roslog roslog_adapter

auto solver::createSolver() -> std::shared_ptr<Solver> {
    return std::make_shared<solver::Solver>();
}

namespace ly_auto_aim::inline solver {

void Solver::setPnPDebugEnabled(bool enabled)
{
    pnpDebugEnabled_ = enabled;
    if (!pnpDebugEnabled_) {
        lastPnPDebugRecords_.clear();
    }
}

// [ROS 2] 3. 构造函数实现
Solver::Solver() {
    if (!global_tracker_solver_node) {
        std::cerr << "[Solver] Error: global_tracker_solver_node is NULL" << std::endl;
        return;
    }
    auto node = global_tracker_solver_node;
    std::string ns = "solver_config.";
    cameraIntrinsicMatrix.setIdentity();
    distorationCoefficients.setZero();
    cameraOffset.setZero();
    cameraRotationMatrix.setIdentity();

    // 1. 加载相机内参矩阵
    std::vector<double> intrinsic_flat;
    std::string intrinsic_name = ns + "camera_intrinsic_matrix";
    if (!node->has_parameter(intrinsic_name)) node->declare_parameter(intrinsic_name, std::vector<double>({}));
    
    if (!node->get_parameter(intrinsic_name, intrinsic_flat) || intrinsic_flat.size() != 9) {
        roslog::error("Invalid camera_intrinsic_matrix format");
    } else {
        cameraIntrinsicMatrix << 
            intrinsic_flat[0], intrinsic_flat[1], intrinsic_flat[2],
            intrinsic_flat[3], intrinsic_flat[4], intrinsic_flat[5],
            intrinsic_flat[6], intrinsic_flat[7], intrinsic_flat[8];
    }
    
    // 2. 获取畸变系数
    std::vector<double> distortion_vec;
    std::string distortion_name = ns + "camera_distortion_coefficients";
    if (!node->has_parameter(distortion_name)) node->declare_parameter(distortion_name, std::vector<double>({}));

    if (!node->get_parameter(distortion_name, distortion_vec) || distortion_vec.size() != 5) {
        roslog::error("Invalid distortion coefficients");
    } else {
        distorationCoefficients = Eigen::Vector5d(distortion_vec.data());
    }

    // 3. 相机偏移量
    std::vector<double> offset_vec;
    std::string offset_name = ns + "camera_offset";
    if (!node->has_parameter(offset_name)) node->declare_parameter(offset_name, std::vector<double>({}));

    if (node->get_parameter(offset_name, offset_vec) && offset_vec.size() == 3) {
        cameraOffset = Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]) * 0.001;
    } else {
        roslog::warn("Using default camera offset [0,0,0]");
        cameraOffset.setZero();
    }

    // 4. 相机旋转参数
    std::vector<double> rotation_vec;
    std::string rotation_name = ns + "camera_rotation";
    if (!node->has_parameter(rotation_name)) node->declare_parameter(rotation_name, std::vector<double>({}));

    if (node->get_parameter(rotation_name, rotation_vec) && rotation_vec.size() == 3) {
        cv::Mat rotationVector(3, 1, CV_64F);
        for(int i = 0; i < 3; ++i)
            rotationVector.at<double>(i) = rotation_vec[i] * M_PI / 180.0; 

        cv::Mat rotationMatrix;
        cv::Rodrigues(rotationVector, rotationMatrix);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                cameraRotationMatrix(i, j) = rotationMatrix.at<double>(i, j);
    } else {
        roslog::warn("Using identity rotation matrix");
        cameraRotationMatrix.setIdentity();
    }

    // 初始化焦距和主点坐标
    if (cameraIntrinsicMatrix.rows() == 3 && cameraIntrinsicMatrix.cols() == 3) {
        f_x = cameraIntrinsicMatrix(0, 0);
        f_y = cameraIntrinsicMatrix(1, 1);
        c_x = cameraIntrinsicMatrix(0, 2);
        c_y = cameraIntrinsicMatrix(1, 2);
        if (f_x <= 0.0 || f_y <= 0.0 || std::abs(cameraIntrinsicMatrix(2, 2) - 1.0) > 1e-6) {
            roslog::warn(
                "Non-standard solver camera matrix: fx={}, fy={}, cx={}, cy={}, k22={}",
                f_x,
                f_y,
                c_x,
                c_y,
                cameraIntrinsicMatrix(2, 2));
        }
    } else {
        f_x = f_y = c_x = c_y = 0;
    }
}

// 距离修正系数
const double distCoef = 5.0 / 3.7;
inline constexpr auto SmallArmorHalfWidth = 0.0675f;
inline constexpr auto SmallArmorHalfHeight = 0.0275f;
inline constexpr auto SmallArmorWidthRatio = 1.0f;
inline constexpr auto SmallArmorHeightRatio = 1.0f;
// SAH : Small Armor Half
inline constexpr auto SAHW = SmallArmorHalfWidth * SmallArmorWidthRatio;
inline constexpr auto SAHH = SmallArmorHalfHeight * SmallArmorHeightRatio;
const std::vector SmallArmorPoints = // 装甲板放在地上
    {
        cv::Point3f(-SAHW, SAHH, 0.0f), cv::Point3f(SAHW, SAHH, 0.0f), cv::Point3f(SAHW, -SAHH, 0.0f),
        cv::Point3f(-SAHW, -SAHH, 0.0f)};

inline constexpr auto LargeArmorHalfWidth = 0.116f;
inline constexpr auto LargeArmorHalfHeight = 0.0275f;
inline constexpr auto LargeArmorWidthRatio = 0.87f;
inline constexpr auto LargeArmorHeightRatio = 1.0f;
inline constexpr auto LAHW = LargeArmorHalfWidth * LargeArmorWidthRatio;
inline constexpr auto LAHH = LargeArmorHalfHeight * LargeArmorHeightRatio;
inline constexpr auto LargeArmorXRatio = 0.87f;
const std::vector LargeArmorPoints = {cv::Point3f(-LAHW, LAHH, 0.0f), cv::Point3f(LAHW, LAHH, 0.0f),
    cv::Point3f(LAHW, -LAHH, 0.0f), cv::Point3f(-LAHW, -LAHH, 0.0f)};

double A = 2.419558578250169;
double B = 130.9631449517108;
double C = 0.4683121472580308;
double D = -0.4554121653810246;

Eigen::Matrix3d CameraIntrinsicsParameterPack::GetCameraMatrix() const {
    Eigen::Matrix3d matrix;
    matrix << FocalLength[0], 0, PrincipalPoint[0], 0, FocalLength[1], PrincipalPoint[1], 0, 0, 1;
    return matrix;
}

Eigen::Vector5d CameraIntrinsicsParameterPack::GetDistortionCoefficients() const {
    Eigen::Vector5d matrix;
    matrix << RadialDistortion[0], RadialDistortion[1], TangentialDistortion[0], TangentialDistortion[1],
        RadialDistortion[2];
    return matrix;
}

[[deprecated("Use solvePnP method instead")]]
double calcDistance(const std::vector<cv::Point2f> &points, double yaw, double pitch) {
    auto dist = [](const cv::Point2f &p1, const cv::Point2f &p2) { return std::hypot(p1.x - p2.x, p1.y - p2.y); };
    double s1 = (dist(points[0], points[1]) + dist(points[2], points[3])) / 2.0;
    double s2 = (dist(points[1], points[2]) + dist(points[3], points[0])) / 2.0;
    double costheta_1 = (s1 * s1 + s2 * s2 - std::pow(dist(points[0], points[2]), 2)) / (2 * s1 * s2);
    double costheta_2 = (s1 * s1 + s2 * s2 - std::pow(dist(points[1], points[3]), 2)) / (2 * s1 * s2);
    double costheta = (costheta_1 + costheta_2) / 2.0;
    double delta = 4 * std::pow(A * s1 * s2 * costheta, 2) + std::pow(s2 * s2 - std::pow(A * s1, 2), 2);
    double distance = B / std::sqrt(std::pow(A * s1, 2) + s2 * s2 + std::sqrt(delta));
    double a1 = std::sqrt(distance * distance + C * C - 2 * distance * C * std::cos(yaw));
    return std::sqrt(a1 * a1 + D * D - 2 * a1 * D * std::cos(pitch));
}

cv::Point2f exactCenter(const std::vector<cv::Point2f> &points) {
    cv::Point2f center(0, 0);
    if (points.size() != 4) {
        return center;
    }
    for (const auto &point: points) {
        center.x += point.x;
        center.y += point.y;
    }
    center.x /= 4.0;
    center.y /= 4.0;
    return center;
}

inline double normalizeAngle(double angle) { return std::remainder(angle, 2 * M_PI); }

struct PnPSelection {
    bool valid = false;
    int index = -1;
    double armor_yaw = 0.0;
    double reprojection_error = std::numeric_limits<double>::max();
    double score = std::numeric_limits<double>::max();
    cv::Mat tvec;
};

double armorYawFromRvec(const cv::Mat &rvec)
{
    cv::Mat rotMat;
    cv::Rodrigues(rvec, rotMat);
    return normalizeAngle(std::atan2(rotMat.at<double>(2, 0), rotMat.at<double>(0, 0)));
}

double reprojectionError(
    const std::vector<cv::Point3f> &objectPoints,
    const std::vector<cv::Point2f> &imagePoints,
    const cv::Mat &cameraMatrix,
    const cv::Mat &distCoeffs,
    const cv::Mat &rvec,
    const cv::Mat &tvec)
{
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projected);
    if (projected.size() != imagePoints.size() || projected.empty()) {
        return std::numeric_limits<double>::max();
    }

    double error = 0.0;
    for (std::size_t i = 0; i < projected.size(); ++i) {
        error += cv::norm(projected[i] - imagePoints[i]);
    }
    return error / static_cast<double>(projected.size());
}

PnPSelection selectPnPSolution(
    const std::vector<cv::Point3f> &objectPoints,
    const std::vector<cv::Point2f> &imagePoints,
    const cv::Mat &cameraMatrix,
    const cv::Mat &distCoeffs,
    const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    bool hasYawHistory,
    double previousYaw,
    bool hasWholeCarYawHint,
    double wholeCarYawHint)
{
    constexpr double kYawHistoryPenaltyPxPerRad = 2.0;
    constexpr double kWholeCarHintPenaltyPxPerRad = 4.0;

    PnPSelection best;
    const auto count = std::min(rvecs.size(), tvecs.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (rvecs[i].empty() || tvecs[i].empty()) {
            continue;
        }
        const double z = tvecs[i].at<double>(2);
        if (!std::isfinite(z) || z <= 0.0) {
            continue;
        }

        const double armor_yaw = armorYawFromRvec(rvecs[i]);
        const double reproj_error = reprojectionError(
            objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs[i], tvecs[i]);
        if (!std::isfinite(armor_yaw) || !std::isfinite(reproj_error)) {
            continue;
        }

        double score = reproj_error;
        if (hasYawHistory) {
            score += kYawHistoryPenaltyPxPerRad *
                     std::abs(normalizeAngle(armor_yaw - previousYaw));
        }
        if (hasWholeCarYawHint) {
            score += kWholeCarHintPenaltyPxPerRad *
                     std::abs(normalizeAngle(armor_yaw - wholeCarYawHint));
        }

        roslog::debug(
            "PnP solution {}: yaw={}, reproj_error={}, score={}, has_history={}, has_whole_car_hint={}",
            i,
            armor_yaw,
            reproj_error,
            score,
            hasYawHistory ? 1 : 0,
            hasWholeCarYawHint ? 1 : 0);

        if (score < best.score) {
            best.valid = true;
            best.index = static_cast<int>(i);
            best.armor_yaw = armor_yaw;
            best.reprojection_error = reproj_error;
            best.score = score;
            best.tvec = tvecs[i].clone();
        }
    }
    return best;
}

// return {pyd, armor_yaw}
std::pair<XYZ, double> Solver::camera2world(
    const ArmorXYV &trackResult, const GimbalAngleType &gimbalAngle_deg, bool isLarge,
    int car_id, int armor_id) {
    cv::Mat cameraMatrix, distCoeffs;
    cv::eigen2cv(cameraIntrinsicMatrix, cameraMatrix);
    cv::eigen2cv(distorationCoefficients, distCoeffs);
    const auto gimbal_yaw = gimbalAngle_deg.yaw * M_PI / 180;
    PnPDebugRecord pnp_debug;
    pnp_debug.car_id = car_id;
    pnp_debug.armor_id = armor_id;
    pnp_debug.used_whole_car = false;

    if (trackResult.size() != 4) {
        if (pnpDebugEnabled_) {
            lastPnPDebugRecords_.push_back(pnp_debug);
        }
        return std::make_pair(XYZ(), 0.0);
    }

    std::vector<cv::Point3f> objectPoints = isLarge ? LargeArmorPoints : SmallArmorPoints;

    std::vector<cv::Point2f> imagePoints;
    for (const auto &xyv: trackResult) { 
        imagePoints.emplace_back(xyv.x, xyv.y);
    }
    
    std::vector<cv::Mat> rvecs, tvecs;
    int solutions = cv::solvePnPGeneric(
        objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    pnp_debug.solution_count = solutions;
    if (solutions <= 0 || rvecs.empty() || tvecs.empty()) {
        roslog::warn("solvePnPGeneric returned no valid armor-only solution for car_id={}, armor_id={}", car_id, armor_id);
        if (pnpDebugEnabled_) {
            lastPnPDebugRecords_.push_back(pnp_debug);
        }
        return std::make_pair(XYZ(), 0.0);
    }

    const auto key = std::make_pair(car_id, armor_id);
    const auto history_it = armorYawHistory_.find(key);
    const bool has_history = history_it != armorYawHistory_.end();
    const double previous_yaw = has_history ? history_it->second : 0.0;
    pnp_debug.has_yaw_history = has_history;
    const auto selected = selectPnPSolution(
        objectPoints,
        imagePoints,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs,
        has_history,
        previous_yaw,
        false,
        0.0);
    if (!selected.valid) {
        roslog::warn("No usable armor-only PnP solution for car_id={}, armor_id={}", car_id, armor_id);
        if (pnpDebugEnabled_) {
            lastPnPDebugRecords_.push_back(pnp_debug);
        }
        return std::make_pair(XYZ(), 0.0);
    }
    armorYawHistory_[key] = selected.armor_yaw;

    roslog::debug(
        "Selected armor-only PnP solution {}: yaw={}, reproj_error={}, score={}, car_id={}, armor_id={}",
        selected.index,
        selected.armor_yaw,
        selected.reprojection_error,
        selected.score,
        car_id,
        armor_id);

    const auto &tvec = selected.tvec;
    XYZ camera(tvec.at<double>(2), -tvec.at<double>(0), -tvec.at<double>(1));
    roslog::debug("before transform: x:{},y:{},z:{}", camera.x, camera.y, camera.z);
    XYZ result = camera2world(camera, gimbalAngle_deg);
    const double world_yaw = selected.armor_yaw + gimbal_yaw;
    pnp_debug.valid = true;
    pnp_debug.selected_index = selected.index;
    pnp_debug.armor_yaw_rad = selected.armor_yaw;
    pnp_debug.world_yaw_rad = world_yaw;
    pnp_debug.reprojection_error_px = selected.reprojection_error;
    pnp_debug.score = selected.score;
    pnp_debug.world_x = result.x;
    pnp_debug.world_y = result.y;
    pnp_debug.world_z = result.z;
    if (pnpDebugEnabled_) {
        lastPnPDebugRecords_.push_back(pnp_debug);
    }
    
    roslog::debug("x:{},y:{},z:{},dist:{}",result.x,result.y,result.z, sqrt(result.x*result.x + result.y*result.y + result.z*result.z));
    return std::make_pair(result, world_yaw);
}

std::pair<XYZ, double> Solver::camera2worldWithWholeCar(
    const ArmorXYV &trackResult, const GimbalAngleType &gimbalAngle_deg, const cv::Rect &bounding_rect, bool isLarge,
    int car_id, int armor_id) {
    cv::Mat cameraMatrix, distCoeffs;
    cv::eigen2cv(cameraIntrinsicMatrix, cameraMatrix);
    cv::eigen2cv(distorationCoefficients, distCoeffs);
    const auto gimbal_yaw = gimbalAngle_deg.yaw * M_PI / 180;
    PnPDebugRecord pnp_debug;
    pnp_debug.car_id = car_id;
    pnp_debug.armor_id = armor_id;
    pnp_debug.used_whole_car = true;

    if (trackResult.size() != 4) {
        if (pnpDebugEnabled_) {
            lastPnPDebugRecords_.push_back(pnp_debug);
        }
        return std::make_pair(XYZ(), 0.0);
    }
    if (bounding_rect.width <= 1) {
        roslog::warn("Invalid bounding_rect.width={}, fallback to armor-only PnP.", bounding_rect.width);
        if (pnpDebugEnabled_) {
            lastPnPDebugRecords_.push_back(pnp_debug);
        }
        return camera2world(trackResult, gimbalAngle_deg, isLarge, car_id, armor_id);
    }

    std::vector<cv::Point3f> objectPoints = isLarge ? LargeArmorPoints : SmallArmorPoints;

    std::vector<cv::Point2f> imagePoints;
    for (const auto &xyv: trackResult) { 
        imagePoints.emplace_back(xyv.x, xyv.y);
    }
    
    std::vector<cv::Mat> rvecs, tvecs;
    int solutions = cv::solvePnPGeneric(
        objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    pnp_debug.solution_count = solutions;
    if (solutions <= 0 || rvecs.empty() || tvecs.empty()) {
        roslog::warn("solvePnPGeneric returned no valid solution, fallback to armor-only PnP.");
        if (pnpDebugEnabled_) {
            lastPnPDebugRecords_.push_back(pnp_debug);
        }
        return camera2world(trackResult, gimbalAngle_deg, isLarge, car_id, armor_id);
    }

    double rect_center_x = (bounding_rect.x + bounding_rect.width / 2);
    double armor_center_x = (imagePoints[0].x + imagePoints[1].x + imagePoints[2].x + imagePoints[3].x) / 4;
    double half_width = bounding_rect.width / 2;
    const double ratio = (armor_center_x - rect_center_x) / half_width;
    const double estimate_armor_yaw = std::asin(std::clamp(ratio, -1.0, 1.0));
    pnp_debug.has_whole_car_yaw_hint = true;
    pnp_debug.whole_car_yaw_hint_rad = estimate_armor_yaw;
    const auto key = std::make_pair(car_id, armor_id);
    const auto history_it = armorYawHistory_.find(key);
    const bool has_history = history_it != armorYawHistory_.end();
    const double previous_yaw = has_history ? history_it->second : 0.0;
    pnp_debug.has_yaw_history = has_history;
    const auto selected = selectPnPSolution(
        objectPoints,
        imagePoints,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs,
        has_history,
        previous_yaw,
        true,
        estimate_armor_yaw);

    if (!selected.valid) {
        roslog::warn("Empty tvec after solution selection, fallback to armor-only PnP.");
        if (pnpDebugEnabled_) {
            lastPnPDebugRecords_.push_back(pnp_debug);
        }
        return camera2world(trackResult, gimbalAngle_deg, isLarge, car_id, armor_id);
    }
    armorYawHistory_[key] = selected.armor_yaw;

    roslog::debug(
        "Selected whole-car PnP solution {}: yaw={}, whole_car_hint={}, reproj_error={}, score={}, car_id={}, armor_id={}",
        selected.index,
        selected.armor_yaw,
        estimate_armor_yaw,
        selected.reprojection_error,
        selected.score,
        car_id,
        armor_id);

    const auto &tvec = selected.tvec;
    XYZ camera(tvec.at<double>(2), -tvec.at<double>(0), -tvec.at<double>(1));
    roslog::debug("before transform: x:{},y:{},z:{}", camera.x, camera.y, camera.z);
    XYZ result = camera2world(camera, gimbalAngle_deg);
    const double world_yaw = selected.armor_yaw + gimbal_yaw;
    pnp_debug.valid = true;
    pnp_debug.selected_index = selected.index;
    pnp_debug.armor_yaw_rad = selected.armor_yaw;
    pnp_debug.world_yaw_rad = world_yaw;
    pnp_debug.reprojection_error_px = selected.reprojection_error;
    pnp_debug.score = selected.score;
    pnp_debug.world_x = result.x;
    pnp_debug.world_y = result.y;
    pnp_debug.world_z = result.z;
    if (pnpDebugEnabled_) {
        lastPnPDebugRecords_.push_back(pnp_debug);
    }
    
    roslog::debug("x:{},y:{},z:{}, dist: {}", result.x, result.y, result.z, sqrt(result.x * result.x + result.y * result.y + result.z * result.z));
    return std::make_pair(result, world_yaw);
}

void Solver::solve_all(
    std::pair<std::vector<TrackResult>, std::vector<CarTrackResult>> &trackResults, GimbalAngleType &gimbalAngle_deg) {
    if (pnpDebugEnabled_) {
        lastPnPDebugRecords_.clear();
    }
    for (auto &trackResult: trackResults.first) {
        auto it = std::find_if(trackResults.second.begin(), trackResults.second.end(),
            [&trackResult](
                const CarTrackResult &carTrackResult) { return carTrackResult.car_id == trackResult.car_id; });
        XYZ xyz_imu; 
        double yaw = 0.0;
        if (it == trackResults.second.end()) {
            roslog::debug("No car track result found for car_id: {}", trackResult.car_id);
            std::tie(xyz_imu, yaw) = camera2world(
                trackResult.armor,
                gimbalAngle_deg,
                trackResult.car_id == 1,
                trackResult.car_id,
                trackResult.armor_id);
        }
        else {
            roslog::debug("Car track result found for car_id: {}", trackResult.car_id);
            std::tie(xyz_imu, yaw) = camera2worldWithWholeCar(
                trackResult.armor,
                gimbalAngle_deg,
                it->bounding_rect,
                trackResult.car_id == 1,
                trackResult.car_id,
                trackResult.armor_id);
        }
        roslog::debug("solver finished for car_id: {}", trackResult.car_id);
        trackResult.location.imu = gimbalAngle_deg;
        trackResult.location.xyz_imu = xyz_imu;
        trackResult.yaw = yaw;

        CXYD coord = trackResult.location.cxy; 
    }
    for (auto &trackResult: trackResults.second) {
        int car_id = trackResult.car_id;
        int car_type = trackResult.car_type;
        auto bounding_rect = trackResult.bounding_rect;
    }
}

void Solver::setCameraIntrinsicMatrix(const Eigen::Matrix3d &cameraIntrinsicMatrix) {
    this->cameraIntrinsicMatrix = cameraIntrinsicMatrix;
}

void Solver::setCameraOffset(const Eigen::Vector3d &cameraOffset) { this->cameraOffset = cameraOffset; }

void Solver::setDistorationCoefficients(const Eigen::Vector5d &distorationCoefficients) {
    this->distorationCoefficients = distorationCoefficients;
}

} // namespace ly_auto_aim::inline solver
