#include <ros/ros.h>
#include "car_tracker/tracker.hpp"
#include "car_tracker/tracker_matcher.hpp"
#include "solver/solver.hpp"
#include <RosTools/RosTools.hpp>

#include <auto_aim_common/Armors.h>
#include <auto_aim_common/DetectionType.hpp>
#include <auto_aim_common/TrackerType.hpp>
#include <auto_aim_common/Trackers.h>
#include <auto_aim_common/CarTracker.h>
#include <auto_aim_common/ArmorTracker.h>
#include <TimeStamp/TimeStamp.hpp>
#include <Logger/Logger.hpp>

using namespace LangYa;
using namespace ly_auto_aim;

namespace{

    LY_DEF_ROS_TOPIC(ly_detector_armors, "/ly/detector/armors", auto_aim_common::Armors);
    LY_DEF_ROS_TOPIC(ly_tracker_results, "/ly/tracker/results", auto_aim_common::Trackers);

    constexpr const char AppName[] = "car_tracker_solver";

    /**
     * @brief 作用是接收消息生成detections和Cardetections
     * @brief 然后merge->getTrackerResult->solver->trackResult
     */
    class CarTrackerSolverNode{

    public:
        CarTrackerSolverNode(int argc, char** argv) {
            node.Initialize(argc, argv);
            tracker = createTracker();
            solver = createSolver();
            location::Location::registerSolver(solver);
            node.GenSubscriber<ly_detector_armors>([this](auto msg) { detection_callback(msg); } );
        }
        ~CarTrackerSolverNode() = default;

        void convertToDetections(const auto_aim_common::Armors::ConstPtr& msg, Detections& detections) {
            detections.clear();
            detections.reserve(msg->armors.size());
            for (const auto& armor : msg->armors) {
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

        void convertToCarDetections(const auto_aim_common::Armors::ConstPtr& msg, CarDetections& car_detections) {
            car_detections.clear();
            car_detections.reserve(msg->cars.size());
            for (const auto& car : msg->cars) {
                car_detections.emplace_back(CarDetection{
                    .bounding_rect = {
                        car.bounding_rect.x,
                        car.bounding_rect.y,
                        car.bounding_rect.width,
                        car.bounding_rect.height
                    },
                    .tag_id = car.car_id
                });
                // car_detection.tag_id = car.car_id;
                // car_detection.bounding_rect.x = car.bounding_rect.x;
                // car_detection.bounding_rect.y = car.bounding_rect.y;
                // car_detection.bounding_rect.width = car.bounding_rect.width;
                // car_detection.bounding_rect.height = car.bounding_rect.height;
                // car_detections.emplace_back(std::move(car_detection));
            }
        }

        void publish_all(const auto& track_results, auto& tracks_msg){
            for(const auto& armor_track_result : track_results.first){
                auto_aim_common::ArmorTracker armor_tracker_msg;
                XYZ armor_xyz = armor_track_result.location.xyz_imu;  //// 预防潜在的问题，没有转化完成就被拿出来了
                armor_tracker_msg.x = armor_xyz.x;
                armor_tracker_msg.y = armor_xyz.y;
                armor_tracker_msg.z = armor_xyz.z;
                armor_tracker_msg.yaw = armor_track_result.yaw;
                armor_tracker_msg.armor_id = armor_track_result.armor_id;
                armor_tracker_msg.car_id = armor_track_result.car_id;
                tracks_msg.armorTrackers.emplace_back(std::move(armor_tracker_msg));
            }
            for(const auto& car_track_result : track_results.second){
                auto_aim_common::CarTracker car_tracker_msg;
                car_tracker_msg.car_id = car_track_result.car_id;
                car_tracker_msg.bounding_rect.x = car_track_result.bounding_rect.x;
                car_tracker_msg.bounding_rect.y = car_track_result.bounding_rect.y;
                car_tracker_msg.bounding_rect.width = car_track_result.bounding_rect.width;
                car_tracker_msg.bounding_rect.height = car_track_result.bounding_rect.height;
                tracks_msg.carTrackers.emplace_back(std::move(car_tracker_msg));
            }
        }

        void detection_callback(const auto_aim_common::Armors::ConstPtr& msg){
            // std::lock_guard<std::mutex> lock(data_mutex);
            auto_aim_common::Trackers trackers_msg;

            trackers_msg.header.stamp = msg->header.stamp;
            trackers_msg.header.frame_id = msg->header.frame_id;
            trackers_msg.Pitch = msg->Pitch;
            trackers_msg.Yaw = msg->Yaw;
            
            
            if (msg->armors.empty()) {
                roslog::warn("detection_callback> empty armors");
                return;
            }
            std::vector<Detection> detections;
            convertToDetections(msg, detections);
            std::vector<CarDetection> car_detections;
            convertToCarDetections(msg, car_detections);

            GimbalAngleType gimbal_angle{msg->Pitch, msg->Yaw};
            
            roslog::info("begin to merge detections");
            tracker->merge(detections);
            roslog::info("begin to merge car_detections");
            tracker->merge(car_detections);
            roslog::info("get track result");
            auto track_results = tracker->getTrackResult(msg->header.stamp, gimbal_angle);
            roslog::info("tracker> solver all");
            solver->solve_all(track_results, gimbal_angle);
            roslog::info("publish all function");
            publish_all(track_results, trackers_msg);
            roslog::info("begin to publish");
            node.Publisher<ly_tracker_results>().publish(trackers_msg);
        }
    private:
        ROSNode<AppName> node;
        std::mutex data_mutex;
        std::unique_ptr<tracker::Tracker> tracker;
        std::shared_ptr<solver::Solver> solver;
        CameraIntrinsicsParameterPack cameraIntrinsics{};
    };
}

int main(int argc, char** argv){
    CarTrackerSolverNode car_tracker_solver(argc, argv);
    ros::spin();
    return 0;
}