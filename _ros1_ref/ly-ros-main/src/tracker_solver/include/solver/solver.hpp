#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <RosTools/RosTools.hpp>
#include <Logger/Logger.hpp>
#include "auto_aim_common/Location.hpp"
#include "auto_aim_common/SolverType.hpp"
#include "auto_aim_common/TrackerType.hpp"


namespace ly_auto_aim::inline solver {
using tracker::ArmorXYV;
/**
 * 留着后面用，现在先不用，solver现在还是太丑了
 */
struct CameraIntrinsicsParameterPack {
    float FocalLength[2]{1331.1f, 1330.1f};
    float PrincipalPoint[2]{624.5817f, 526.3662f};
    float RadialDistortion[3]{-0.1059f, -0.3427f, 1.4125f};
    float TangentialDistortion[2]{0.0072f, 0};

    Eigen::Matrix3d GetCameraMatrix() const;

    Eigen::Vector5d GetDistortionCoefficients() const;
};

/**
 * @brief 完整名称其实是pnp_solver，使用了pnp算法来计算距离，但是使用线性拟合的办法计算全局的yaw和pitch
 * @note 后面可以把pitch和yaw的的计算优化一下
 */
class Solver : public BaseSolver {
private:
    Eigen::Matrix3d cameraIntrinsicMatrix;
    Eigen::Vector3d cameraOffset;
    Eigen::Vector5d distorationCoefficients; // k1,k2,p1,p2,k3
    Eigen::Vector3d cameraRotation;
    Eigen::Matrix3d cameraRotationMatrix;
    double f_x, f_y, c_x, c_y;
    // static double X = 637.5, Y = 496.5;  // center of the image
    // static double X1 = -3.78e-6, Y1 = 7.394e-4, X2 = 7.266e-4, Y2 = 3e-6;
public:
    explicit Solver() {
        ros::NodeHandle nh("solver_config");

        // 1. 加载相机内参矩阵
        std::vector<double> intrinsic_flat;
        if (!nh.getParam("camera_intrinsic_matrix", intrinsic_flat) || intrinsic_flat.size() != 9) {
            roslog::error("Invalid camera_intrinsic_matrix format");
            throw std::runtime_error("Parameter loading failed");
        }
        cameraIntrinsicMatrix << 
            intrinsic_flat[0], intrinsic_flat[1], intrinsic_flat[2],
            intrinsic_flat[3], intrinsic_flat[4], intrinsic_flat[5],
            intrinsic_flat[6], intrinsic_flat[7], intrinsic_flat[8];
        
        // 2. 获取畸变系数
        std::vector<double> distortion_vec;
        if (!nh.getParam("camera_distortion_coefficients", distortion_vec) || distortion_vec.size() != 5) {
            roslog::error("Invalid distortion coefficients");
            throw std::runtime_error("Parameter loading failed");
        }
        distorationCoefficients = Eigen::Vector5d(distortion_vec.data());

        // 3. 相机偏移量（含单位转换，转化成m）
        std::vector<double> offset_vec;
        if (nh.getParam("camera_offset", offset_vec) && offset_vec.size() == 3) {
            cameraOffset = Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]) * 0.001;
        } else {
            roslog::warn("Using default camera offset [0,0,0]");
            cameraOffset.setZero();
        }

        // 4. 相机旋转参数
        std::vector<double> rotation_vec;
        if (nh.getParam("camera_rotation", rotation_vec) && rotation_vec.size() == 3) {
            cv::Mat rotationVector(3, 1, CV_64F);
            for(int i = 0; i < 3; ++i)
                rotationVector.at<double>(i) = rotation_vec[i] * M_PI / 180.0; // 转换为弧度

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
        f_x = cameraIntrinsicMatrix(0, 0);
        f_y = cameraIntrinsicMatrix(1, 1);
        c_x = cameraIntrinsicMatrix(0, 2);
        c_y = cameraIntrinsicMatrix(1, 2);
    };

    inline PYD XYZ2PYD(const XYZ &in) const override {
        double distance = sqrt(in.x * in.x + in.y * in.y + in.z * in.z);
        double pitch = asin(in.z / distance);
        double yaw = atan2(in.y, in.x);
        return PYD(pitch, yaw, distance);
    }
    inline XYZ PYD2XYZ(const PYD &in) const override {
        XYZ out;
        out.x = in.distance * cos(in.pitch) * cos(in.yaw);
        out.y = in.distance * cos(in.pitch) * sin(in.yaw);
        out.z = in.distance * sin(in.pitch);
        return out;
    }
	inline XYZ CXYD2XYZ(const CXYD& in) const override
	{
		//eliminate distortion
        XYZ out;
        out.x = in.k;
        out.y = (c_x - in.cx) * in.k / f_x;
        out.z = (c_y - in.cy) * in.k / f_y;
		return out;
	}
	inline CXYD XYZ2CXYD(const XYZ& in) const override
	{
        double cx = c_x - in.y / in.x * f_x;
        double cy = c_y - in.z / in.x * f_y;
        //distortion
//        double r2 = cx * cx + cy * cy;
//        double r4 = r2 * r2;
//        double r6 = r4 * r2;
//        double r = 1 + distorationCoefficients(0) * r2 + distorationCoefficients(1) * r4 + distorationCoefficients(4) * r6;
//        CXYD out;
//        out.cx = cx * r + 2 * distorationCoefficients(2) * cx * cy + distorationCoefficients(3) * (r2 + 2 * cx * cx);
//        out.cy = cy * r + 2 * distorationCoefficients(3) * cx * cy + distorationCoefficients(2) * (r2 + 2 * cy * cy);
//        out.k = in.x;
        CXYD out;
        out.cx = cx;
        out.cy = cy;
        out.k = in.x;
		return out;
	}
    // inline PYD fuseGimbalAngle(const PYD &in, const PYD &gimbalAngle) const override {
    //     PYD out;
    //     out.pitch = in.pitch + gimbalAngle.pitch;
    //     out.yaw = in.yaw + gimbalAngle.yaw;
    //     out.distance = in.distance;
    //     return out;
    // }
    // inline PYD separateGimbalAngle(const PYD &in, const PYD &gimbalAngle) const override {
    //     PYD out;
    //     out.pitch = in.pitch - gimbalAngle.pitch;
    //     out.yaw = in.yaw - gimbalAngle.yaw;
    //     out.distance = in.distance;
    //     return out;
    // }

    inline XYZ camera2world(const XYZ& in, const PYD& imuData) const override
	{
        double imu_yaw = imuData.yaw;
        double imu_pitch = imuData.pitch;
        //double imu_roll = 0;
        Eigen::Vector3d in_eigen = Eigen::Vector3d(in.x, in.y, in.z);
        Eigen::Vector3d gimbal = cameraRotationMatrix * in_eigen + cameraOffset;
        Eigen::Vector3d world = Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                                 * Eigen::AngleAxisd(-imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                                            * gimbal;
        return XYZ(world(0), world(1), world(2));
	}
	inline XYZ world2camera(const XYZ& in, const PYD& imuData) const override
	{
        double imu_yaw = imuData.yaw;
        double imu_pitch = imuData.pitch;
        Eigen::Vector3d in_eigen = Eigen::Vector3d(in.x, in.y, in.z);
        Eigen::Vector3d gimbal = Eigen::AngleAxisd(imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                                 * Eigen::AngleAxisd(-imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                                            * in_eigen;
        Eigen::Vector3d camera = cameraRotationMatrix.inverse() * (gimbal - cameraOffset);

        return XYZ(camera(0), camera(1), camera(2));
	}

    std::pair<XYZ, double> camera2world(
        const ArmorXYV &trackResult, const GimbalAngleType &gimbalAngle_deg, bool isLarge);
    std::pair<XYZ, double> camera2worldWithWholeCar(const ArmorXYV &trackResult, const GimbalAngleType &gimbalAngle_deg,
        const cv::Rect &bounding_rect, bool isLarge);

    void solve_all( std::pair<std::vector<TrackResult>, std::vector<CarTrackResult>>& trackResults, 
                    GimbalAngleType& gimbalAngle_deg);

    void setCameraIntrinsicMatrix(const Eigen::Matrix3d &cameraIntrinsicMatrix);
    void setCameraOffset(const Eigen::Vector3d &cameraOffset);
    void setDistorationCoefficients(const Eigen::Vector5d &distorationCoefficients);
};
std::shared_ptr<Solver> createSolver();
} // namespace ly_auto_aim::inline solver
