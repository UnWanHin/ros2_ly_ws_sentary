#include <RosTools/RosTools.hpp>
#include "Logger/Logger.hpp"
#include <TimeStamp/TimeStamp.hpp>
#include <auto_aim_common/Location.hpp>
#include <auto_aim_common/TrackerType.hpp>
#include <auto_aim_common/PredictionType.hpp>
#include <auto_aim_common/DetectionType.hpp>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

#include "auto_aim_common/CarTracker.h"
#include "auto_aim_common/ArmorTracker.h"
#include "auto_aim_common/Trackers.h"
#include "auto_aim_common/Target.h"
#include "auto_aim_common/DebugFilter.h"

#include "predictor/predictor.hpp"
#include "controller/controller.hpp"
#include "solver/solver.hpp"

// std
#include <atomic>
//atomic不需要加🔒

using namespace LangYa;
using namespace ly_auto_aim;

namespace {
    LY_DEF_ROS_TOPIC(ly_tracker_results, "/ly/tracker/results", auto_aim_common::Trackers);
    LY_DEF_ROS_TOPIC(ly_predictor_target, "/ly/predictor/target", auto_aim_common::Target);
    LY_DEF_ROS_TOPIC(ly_predictor_debug, "/ly/predictor/debug", auto_aim_common::DebugFilter);
    LY_DEF_ROS_TOPIC(ly_bt_target, "/ly/bt/target", std_msgs::UInt8);
    LY_DEF_ROS_TOPIC(ly_bullet_speed, "/ly/bullet/speed", std_msgs::Float32);
    

    constexpr const char AppName[] = "predictor_node";
    std::atomic<ArmorType> automic_target;  /// 接收决策的数据包
    std::atomic<float> atomic_bullet_speed{23.0f};  /// 接收子弹速度的数据包

    class PredictorNode{
        public:
            PredictorNode(int argc, char** argv){
                node.Initialize(argc, argv);
                node.GenSubscriber<ly_tracker_results>([this](const auto msg) { predictor_callback(msg); } );
                node.GenSubscriber<ly_bt_target>([this](const auto msg) { get_target_callback(msg); });
                node.GenSubscriber<ly_bullet_speed>([this](const auto msg) { get_bullet_speed_callback(msg); });
                solver = createSolver();
                predictor = createPredictor();
                controller = createController();
                location::Location::registerSolver(solver);
                controller->registPredictFunc([this](Time::TimeStamp timestamp) {
                    return predictor->predict(timestamp);
                });
            }
            ~PredictorNode() = default;

            void get_target_callback(const std_msgs::UInt8::ConstPtr& msg) {
                automic_target = static_cast<ArmorType>(msg->data);
            }

            void get_bullet_speed_callback(const std_msgs::Float32::ConstPtr& msg) {
                atomic_bullet_speed = static_cast<float>(msg->data);
            }

            void convertMsgToTrackResults(auto& msg, TrackResultPairs& track_results, GimbalAngleType gimbal_angle){
                track_results.first.clear();
                track_results.second.clear();
                for (const auto& armor_tra_msg : msg->armorTrackers) {
                    TrackResult armor_track_result;
                    armor_track_result.car_id = armor_tra_msg.car_id;
                    armor_track_result.armor_id = armor_tra_msg.armor_id;
                    armor_track_result.yaw = armor_tra_msg.yaw;
                    armor_track_result.location.imu = gimbal_angle;
                    XYZ armor_xyz(armor_tra_msg.x, armor_tra_msg.y, armor_tra_msg.z); /// 这地方会有潜在的问题，就是没有这个构造函数
                    armor_track_result.location.xyz_imu = armor_xyz;
                    track_results.first.emplace_back(std::move(armor_track_result));
                }
                for (const auto& car_tra_msg : msg->carTrackers) {
                    CarTrackResult car_track_result;
                    car_track_result.car_id = car_tra_msg.car_id;
                    car_track_result.bounding_rect.x = car_tra_msg.bounding_rect.x;
                    car_track_result.bounding_rect.y = car_tra_msg.bounding_rect.y;
                    car_track_result.bounding_rect.width = car_tra_msg.bounding_rect.width;
                    car_track_result.bounding_rect.height = car_tra_msg.bounding_rect.height;
                    track_results.second.emplace_back(std::move(car_track_result));
                }
            }

            void predictor_callback(const auto_aim_common::Trackers::ConstPtr& msg){
                // std::lock_guard<std::mutex> lock(data_mutex);
                GimbalAngleType gimbal_angle{msg->Pitch, msg->Yaw};

                if(location::Location::isSolverRegistered() == false){
                    // location::registerSolver(std::make_shared<ly_auto_aim::solver::BaseSolver>());
                    roslog::error("11111111111111111, no solver in other package");
                }

                TrackResultPairs track_results;
                convertMsgToTrackResults(msg, track_results, gimbal_angle);
                int target = static_cast<int>(automic_target.load());
                float bullet_speed = static_cast<float>(atomic_bullet_speed.load());
                ControlResult control_result = controller->control(gimbal_angle, target, bullet_speed);  /// 然后这里调用的是predictor的方法，注意里面用的是now_time + fly_time

                predictor->update(track_results, msg->header.stamp); /// 看看怎么回事

                auto_aim_common::Target target_msg;
                target_msg.status = control_result.valid;  /// 后面还要注意一下这个isFindTarget是不是真的能用
                target_msg.yaw = control_result.yaw_actual_want;
                target_msg.pitch = control_result.pitch_actual_want;
                target_msg.header = msg->header;

                auto_aim_common::DebugFilter debug_filter_msg;
                auto predictions = predictor->predict(msg->header.stamp);
                for(auto& prediction : predictions){
                    XYZ car_XYZ = prediction.center;
                    debug_filter_msg.position.x = car_XYZ.x;
                    debug_filter_msg.position.y = car_XYZ.y;
                    debug_filter_msg.position.z = car_XYZ.z;
                    debug_filter_msg.yaw = prediction.theta;
                    debug_filter_msg.v_yaw = prediction.omega;
                    debug_filter_msg.velocity.x = prediction.vx;
                    debug_filter_msg.velocity.y = prediction.vy;
                    debug_filter_msg.radius_1 = prediction.r1;
                    debug_filter_msg.radius_2 = prediction.r2;
                }

                if(control_result.valid){
                    node.Publisher<ly_predictor_target>().publish(target_msg);
                    node.Publisher<ly_predictor_debug>().publish(debug_filter_msg);
                }

            }

        private:
            ROSNode<AppName> node;
            std::shared_ptr<solver::Solver> solver;
            std::unique_ptr<Predictor> predictor;
            std::shared_ptr<Controller> controller;
            std::mutex data_mutex;

    };
}

int main(int argc, char** argv) {
    PredictorNode predictor_node(argc, argv);
    ros::spin();
    return 0;
}