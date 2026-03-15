#include <RosTools/RosTools.hpp>
#include <ros/ros.h>
#include <TimeStamp/TimeStamp.hpp>

#include <auto_aim_common/DetectionType.hpp>
#include <auto_aim_common/Target.h>
#include <auto_aim_common/Armor.h>
#include <auto_aim_common/Armors.h>
#include <std_msgs/Bool.h>

#include <map>
#include <mutex>
#include <memory>

#include "solver/solver.hpp"
#include "detector/detector.hpp"
#include "solver/PoseSolver.hpp"
#include "predictor/predictor.hpp"
#include "predictor/OutpostPredictor.hpp"
#include "predictor/DerectionJudger.hpp"
#include "predictor/TopFilter.hpp"
#include "controller/MuzzleSolver.hpp"
#include "controller/BoardSelector.hpp"
#include "utils/utils.h"

using namespace LangYa;
using namespace LY_UTILS;
using namespace ly_auto_aim;

#define MAX_LOSS_COUNT 10
#define USE_MEASURE_UPDATE true
#define USE_ITERATION_UPDATE false

namespace{
    LY_DEF_ROS_TOPIC(ly_outpost_armors, "/ly/outpost/armors", auto_aim_common::Armors);
    LY_DEF_ROS_TOPIC(ly_outpost_enable, "/ly/outpost/enable", std_msgs::Bool);
    LY_DEF_ROS_TOPIC(ly_outpost_target, "/ly/outpost/target", auto_aim_common::Target);

    constexpr const char AppName[] = "outpost_hitter";

    class OutpostHitterNode
    {
        public:
            OutpostHitterNode(int argc, char** argv){
                node.Initialize(argc, argv);
                node.GenSubscriber<ly_outpost_armors>([this](auto msg) { outpost_detection_callback(msg); });
                // node.GenSubscriber<ly_outpost_enable>([this](auto msg) { outpost_enable_callback(msg); });
                solver = std::make_unique<SOLVER::PoseSolver>();
                outpost_predictor = std::make_unique<PREDICTOR::OutpostPredictor>();
                derection_judger = std::make_unique<PREDICTOR::DirectionJudger>();
                top_filter  = std::make_unique<PREDICTOR::TopFilter>();
                muzzle_solver = std::make_unique<CONTROLLER::MuzzleSolver>(Eigen::Vector3d(0.0, 0.0, 0.0));
                board_selector = std::make_unique<CONTROLLER::BoardSelector>();

                roslog::warn("OutpostHitterNode> Initialized.");
            }
            ~OutpostHitterNode() = default;

            void outpost_detection_callback(const auto_aim_common::Armors::ConstPtr& msg) {
                // if(!outpost_enable.load()) return;

                roslog::warn("OutpostHitterNode> Received outpost detection message.");

                // ​****** 步骤一：detection的获取 ​*********
                ros::Time stamp = msg->header.stamp;
                float yaw_now = msg->Yaw;
                float pitch_now = msg->Pitch; 
                /// debug
                yaw_now = 1000.0f;
                pitch_now = 10.0f;

                Detections detections;
                convertToDetections(msg, detections);
                roslog::warn("Conrners converted");
                
                if(detections.empty()) {
                    roslog::warn("OutpostHitterNode> outpost not found.");
                    return;
                }
                // ​*********************************
                roslog::info("#####1");

                // ​****** 步骤二: 解算 ​*************
                SOLVER::ArmorPoses armor_poses = solver->solveArmorPoses(detections, yaw_now, pitch_now);
                roslog::info("Armor poses solved");
                // ​**********************************

                // ​******** 步骤三： 预测 ​************
                /// 取出最近的装甲板
                auto min_pitch_armor = std::min_element(armor_poses.begin(), armor_poses.end(),[](const SOLVER::ArmorPose &a, const SOLVER::ArmorPose &b)
                                                           { return a.pyd.distance < b.pyd.distance; });

                if (min_pitch_armor == armor_poses.end()) {
                    roslog::warn("OutpostHitterNode> No valid armor poses found.");
                    return;
                }
                PREDICTOR::OutpostInformation outpost_info;
                /// 判断方向
                if(!derection_judger->isDerectionJudged()){

                    roslog::warn("Direction Judging ...");
                    /// 更新世界坐标系下的yaw角和距离
                    derection_judger->updateWorldPYD(min_pitch_armor->pyd.pitch, min_pitch_armor->pyd.yaw, min_pitch_armor->pyd.distance);
                }
                if(derection_judger->isDerectionJudged()){
                    roslog::warn("Direction Judged");
                    if(!outpost_predictor->is_initialized) { // 初始化
                        roslog::warn("OutpostHitterNode> Initializing outpost predictor.");
                        outpost_predictor->initPredictor(*(min_pitch_armor), derection_judger->getDirection());
                        last_time_stamp = stamp;
                    } else { // 量测更新
                        roslog::warn("OutpostHitterNode> Running outpost predictor.");
                        int delta_time = static_cast<int>((stamp - last_time_stamp).toSec());
                        last_time_stamp = stamp;
                        outpost_info = outpost_predictor->runPredictor(delta_time, *(min_pitch_armor), USE_MEASURE_UPDATE);

                        
                        roslog::warn("Predicted Outpost outpost_theta: {} ",outpost_info.outpost_theta);
                        roslog::warn("Predicted Outpost outpost_omega: {} ",outpost_info.outpost_omega);
                        roslog::warn("Predicted Outpost outpost_vx:{} ",outpost_info.center_velocity[0]);
                        roslog::warn("Predicted Outpost outpost_vy:{} ",outpost_info.center_velocity[1]);
                        roslog::warn("Predicted Outpost outpost_vz:{} ",outpost_info.center_velocity[2]);
                        roslog::warn("Predicted Outpost outpost_x:{} ",outpost_info.center_position[0]);
                        roslog::warn("Predicted Outpost outpost_y:{} ",outpost_info.center_position[1]);
                        roslog::warn("Predicted Outpost outpost_z:{} ",outpost_info.center_position[2]);
                        
                        
                        
                        
                        if (!outpost_info.is_valid) {
                            roslog::warn("OutpostHitterNode> Outpost info is not valid.");
                            return;
                        }

                        // publishOutpostInfo(outpost_info, stamp);
                    }
                } else {
                    roslog::warn("Dierection Judging ...");
                    return;
                }
                // ​**********************************
                roslog::info("######2");

                // ​********* 步骤四：计算角度 ​************
                /// 坐标系x：右大左小，y:前大后小，z:上大下小
                muzzle_solver->setBulletSpeed(23.0);

                CONTROLLER::BoardInformations board_info = muzzle_solver->solveMuzzle(outpost_info);

                if(board_info.size()==0) return ;

                roslog::info("########3");
                auto best_board = board_selector->selectBestBoard(board_info);
                roslog::info("########4");
                double pitch_setpoint = best_board.aim_pitch;
                pitch_setpoint *= 180 / M_PI;
                pitch_setpoint -= 2.0; // 需要减去2度的偏差
                double yaw_setpoint = best_board.aim_yaw;
                yaw_setpoint *= 180 / M_PI;
                yaw_setpoint = (float)(yaw_setpoint + std::round((yaw_now - yaw_setpoint) / 360.0) * 360.0);
                roslog::info("#######5");
                if(yaw_setpoint - yaw_now > 80 || yaw_setpoint - yaw_now < -80){
                    // roslog::warn("OutpostHitterNode> Yaw setpoint is too far from current yaw, skipping.");
                    return;
                }

                // ​***************************************


                // ​************ 步骤五: 发布消息********
                auto_aim_common::Target target_msg;
                target_msg.header.stamp = stamp;
                target_msg.yaw = yaw_setpoint;
                target_msg.pitch = pitch_setpoint;
                node.Publisher<ly_outpost_target>().publish(target_msg);
            }

        private:
            void convertToDetections(const auto_aim_common::Armors::ConstPtr& msg, Detections& detections) {
                detections.clear();
                detections.reserve(msg->armors.size());
                for (const auto& armor : msg->armors) {
                    if(armor.type != 7) continue; /// 需要判断这个是否已经被过滤了
                    /// 需要进一步判断是否对应的上，感觉对不上的样子
                    double angle1 = atan2(armor.corners_y[3] - armor.corners_y[1], armor.corners_x[0] - armor.corners_x[3]);
                    double angle2 = atan2(armor.corners_y[2] - armor.corners_y[0], armor.corners_x[1] - armor.corners_x[2]);
                    if(abs(angle1 - M_PI/2) >= M_PI/6 || abs(angle2 - M_PI/2) >= M_PI/6) continue;
                    detections.emplace_back(Detection{
                        .tag_id = armor.type,  // C++20 
                        .corners = {
                            {armor.corners_x[0], armor.corners_y[0]},
                            {armor.corners_x[1], armor.corners_y[1]},
                            {armor.corners_x[2], armor.corners_y[2]},
                            {armor.corners_x[3], armor.corners_y[3]}
                        }
                    });
                }
            }

            ROSNode<AppName> node;
            std::unique_ptr<SOLVER::PoseSolver> solver;
            std::unique_ptr<PREDICTOR::OutpostPredictor> outpost_predictor;
            std::unique_ptr<PREDICTOR::DirectionJudger> derection_judger;
            std::unique_ptr<PREDICTOR::TopFilter> top_filter;
            std::unique_ptr<CONTROLLER::MuzzleSolver> muzzle_solver;
            std::unique_ptr<CONTROLLER::BoardSelector> board_selector;
            ros::Time last_time_stamp;
    };
}


int main(int argc, char** argv){
    OutpostHitterNode outpost_hitter_node(argc, argv);
    ros::spin();
    return 0;
}