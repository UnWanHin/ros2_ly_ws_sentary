#include "solver/PoseSolver.hpp"

using namespace DETECTOR;
namespace SOLVER
{
    ArmorPoses PoseSolver::solveArmorPoses(const Detections &detections, const float &imu_yaw, const float &imu_pitch)
    {
        // opencv坐标系：x轴向右，y轴向下，z轴向前
        std::cout<<"recv yaw"<<imu_yaw<<"recv pitch"<<imu_pitch<<std::endl;
        ArmorPoses armor_poses;
        for (auto bbox : detections)
        {
            ArmorPose armor_pose_temp;
            // 从bbox中获取装甲板的3d坐标
            vector<Point2f> corners;
            for (int i = 0; i < 4; i++)
            {
                corners.push_back(bbox.corners[i]);
            }

            // small armor
            solvePnP(points_small_3d, corners, intrinsicMatrix, distortionCoefficients,
                     rvec, tvec, false, cv::SOLVEPNP_IPPE);

            cv::Mat m_R;
            Eigen::Matrix3d e_R;
            Eigen::Vector3d e_T;
            cv::Rodrigues(rvec, m_R);    
            cv2eigen(tvec, e_T);
            cv2eigen(m_R, e_R);

            std::cout<<"et_x"<<e_T[0]<<"et_y"<<e_T[1]<<"et_z"<<e_T[2]<<std::endl;
            
            armor_pose_temp.pyd.distance = e_T.norm();

            // 计算装甲板法向量在相机坐标系下的方向
            Eigen::Vector3d armor_normal = e_R.col(2);  // 取旋转矩阵的第三列，即z轴方向
            
            // 计算法向量在水平面上的投影
            double theta_cam = atan2(armor_normal(0), armor_normal(2));  // 在相机坐标系下的朝向角
            
            // 转换到世界坐标系
            armor_pose_temp.theta_world = theta_cam + imu_yaw * CV_PI / 180.0f;
            
            // 将theta_world限制在[-π, π]范围内
            while(armor_pose_temp.theta_world > M_PI) armor_pose_temp.theta_world -= 2*M_PI;
            while(armor_pose_temp.theta_world < -M_PI) armor_pose_temp.theta_world += 2*M_PI;

            armor_pose_temp.pyd.pitch = (imu_pitch) * CV_PI / 180.0f + atan2(-e_T(1), sqrt(e_T(2)*e_T(2) + e_T(0)*e_T(0)));
            armor_pose_temp.pyd.yaw = (imu_yaw) * CV_PI / 180.0f - atan2(e_T(0), e_T(2));


            // 将yaw限制在[-π, π]范围内
            while(armor_pose_temp.pyd.yaw > M_PI) armor_pose_temp.pyd.yaw -= 2*M_PI;
            while(armor_pose_temp.pyd.yaw < -M_PI) armor_pose_temp.pyd.yaw += 2*M_PI;

            std::cout<<"pitch"<<armor_pose_temp.pyd.pitch<<std::endl;
            std::cout<<"yaw"<<armor_pose_temp.pyd.yaw<<std::endl;
            


            armor_poses.push_back(armor_pose_temp);
        }
        return armor_poses;
    }

    // 将点p转换为角度delta_yaw和delta_pitch
    void PoseSolver::point2Angle(const cv::Point2f &p, float &delta_yaw, float &delta_pitch)
    {
        double u = (p.x - this->u0_) / this->fx_;
        double v = (p.y - this->v0_) / this->fy_;

        delta_yaw = atan(u);
        delta_pitch = atan(v);
    }

} // namespace SOLVER