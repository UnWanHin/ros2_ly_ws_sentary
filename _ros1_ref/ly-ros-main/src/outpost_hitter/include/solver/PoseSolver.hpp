#pragma once
#include <auto_aim_common/DetectionType.hpp>
#include <vector>
#include <ros/ros.h>
#include <Logger/Logger.hpp>
#include <detector/BBoxes.h>
#include "solver.hpp"
#include "detector/detector.hpp"

using namespace cv;
using namespace std;
using namespace DETECTOR;
using namespace ly_auto_aim;

namespace SOLVER
{
    class PoseSolver
    {
    public:
        // PoseSolver() = default;
        explicit PoseSolver(){
            ros::NodeHandle nh("solver_config");

                    
            std::vector<double> intrinsic_flat;
                if (!nh.getParam("camera_intrinsic_matrix", intrinsic_flat) || intrinsic_flat.size() != 9) {
                roslog::error("Invalid camera_intrinsic_matrix format");
                throw std::runtime_error("Parameter loading failed");
            }
            intrinsicMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    intrinsicMatrix.at<double>(i, j) = intrinsic_flat[i * 3 + j];
                }
            }

            fx_ = intrinsicMatrix.at<double>(0, 0);
            fy_ = intrinsicMatrix.at<double>(1, 1);
            u0_ = intrinsicMatrix.at<double>(0, 2);
            v0_ = intrinsicMatrix.at<double>(1, 2);
            std::vector<double> distortion_flat;
            if (!nh.getParam("camera_distortion_coefficients", distortion_flat) || distortion_flat.size() != 5) {
                roslog::error("Invalid camera_distortion_coefficients format");
                throw std::runtime_error("Parameter loading failed");
            }
            distortionCoefficients = cv::Mat(1, 5, CV_64F, distortion_flat.data()).clone();
            
        }

        ~PoseSolver() = default;

        ArmorPoses solveArmorPoses(const ly_auto_aim::detector::Detections &, const float &, const float &);

        void setCameraMatrix(double fx, double fy, double u0, double v0, double k1, double k2, double p1, double p2, double k3)
        {
            // 相机内参
            intrinsicMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
            intrinsicMatrix.ptr<double>(0)[0] = fx;
            intrinsicMatrix.ptr<double>(0)[2] = u0;
            intrinsicMatrix.ptr<double>(1)[1] = fy;
            intrinsicMatrix.ptr<double>(1)[2] = v0;
            intrinsicMatrix.ptr<double>(2)[2] = 1.0f;

            // 畸变系数
            distortionCoefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
            distortionCoefficients.ptr<double>(0)[0] = k1;
            distortionCoefficients.ptr<double>(1)[0] = k2;
            distortionCoefficients.ptr<double>(2)[0] = p1;
            distortionCoefficients.ptr<double>(3)[0] = p2;
            distortionCoefficients.ptr<double>(4)[0] = k3;

            this->fx_ = fx;
            this->fy_ = fy;
            this->u0_ = u0;
            this->v0_ = v0;
        }

    private:
        //////////////////////////////////////////////////
        // 相机参数：通过标定获取
        cv::Mat intrinsicMatrix;
        cv::Mat distortionCoefficients;
        double fx_, fy_, u0_, v0_;

        cv::Mat rvec; // 旋转向量 OpenCV
        cv::Mat tvec; // 平移向量 OpenCV

        //////////////////////////////////////////////////
        // 装甲板的物理信息，用于PNP解算
        float length_of_small = 0.0675f;
        float height_of_small = 0.0275f;
        float length_of_big = 0.1125f;
        float height_of_big = 0.0275f;

        // 小装甲板3d坐标
        vector<Point3f> points_small_3d = {Point3f(-length_of_small, -height_of_small, 0.f),
                                           Point3f(length_of_small, -height_of_small, 0.f),
                                           Point3f(length_of_small, height_of_small, 0.f),
                                           Point3f(-length_of_small, height_of_small, 0.f)};

        //  大装甲板3d坐标
        vector<Point3f> points_large_3d = {Point3f(-length_of_big, -height_of_big, 0.f),
                                           Point3f(length_of_big, -height_of_big, 0.f),
                                           Point3f(length_of_big, height_of_big, 0.f),

                                           Point3f(-length_of_big, height_of_big, 0.f)};
        //////////////////////////////////////////////////

        void point2Angle(const cv::Point2f &, float &, float &);
    };
} // namespace SOLVER