// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

/*
 * car_tracker_solver_node
 *
 * 链路位置：
 * /ly/detector/armors -> tracker+solver -> /ly/tracker/results
 *
 * 主要功能：
 * - 检测结果融合、目标关联与跟踪
 * - 在统一时间源下执行坐标解算，输出给 predictor
 */
// [ROS 2] 引入頭文件
#include <rclcpp/rclcpp.hpp>
#include "car_tracker/tracker.hpp"
#include "car_tracker/tracker_matcher.hpp"
#include "solver/solver.hpp"
#include <RosTools/RosTools.hpp>

// [ROS 2] 消息頭文件
#include <auto_aim_common/msg/armors.hpp>
#include <auto_aim_common/DetectionType.hpp>
#include <auto_aim_common/TrackerType.hpp>
#include <auto_aim_common/msg/trackers.hpp>
#include <auto_aim_common/msg/car_tracker.hpp>
#include <auto_aim_common/msg/armor_tracker.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <Logger/Logger.hpp>
#include <fmt/format.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

using namespace LangYa;
using namespace ly_auto_aim;

namespace {
    rclcpp::Node::SharedPtr global_node_ptr = nullptr;
}

namespace roslog_adapter {
    static rclcpp::Logger get_logger() {
        if (global_node_ptr) {
            return global_node_ptr->get_logger();
        }
        return rclcpp::get_logger("car_tracker_solver");
    }
    template <typename... Args>
    void warn(const char* format_str, const Args&... args) {
        RCLCPP_WARN(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void info(const char* format_str, const Args&... args) {
        RCLCPP_INFO(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
}
#define roslog roslog_adapter

namespace{

    LY_DEF_ROS_TOPIC(ly_detector_armors, "/ly/detector/armors", auto_aim_common::msg::Armors);
    LY_DEF_ROS_TOPIC(ly_tracker_results, "/ly/tracker/results", auto_aim_common::msg::Trackers);

    constexpr const char AppName[] = "car_tracker_solver";
    constexpr double kRadToDeg = 57.29577951308232;

    class AimTimerFileLogger {
    public:
        bool Open(bool enabled, const std::string& dir, const std::string& component) {
            enabled_ = enabled;
            if (!enabled_) {
                return false;
            }

            try {
                const auto log_dir = std::filesystem::path(ExpandHome(dir.empty() ? "~/Log/AimTimer" : dir));
                std::filesystem::create_directories(log_dir);
                path_ = (log_dir / BuildFileName(component)).string();
                stream_.open(path_, std::ios::out | std::ios::app);
                if (!stream_.is_open()) {
                    enabled_ = false;
                    return false;
                }
                Write("event=logger_start component=" + component + " file=" + path_);
                return true;
            } catch (...) {
                enabled_ = false;
                return false;
            }
        }

        bool Enabled() const {
            return enabled_ && stream_.is_open();
        }

        const std::string& Path() const {
            return path_;
        }

        void Write(const std::string& line) {
            if (!Enabled()) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            stream_ << "wall_time=" << NowString() << " " << line << '\n';
            stream_.flush();
        }

    private:
        static std::string ExpandHome(const std::string& path) {
            const char* home = std::getenv("HOME");
            if (!home || !*home) {
                return path;
            }
            if (path == "~") {
                return std::string(home);
            }
            if (path.rfind("~/", 0) == 0) {
                return std::string(home) + "/" + path.substr(2);
            }
            return path;
        }

        static std::string TimestampString(const char* format) {
            const auto now = std::chrono::system_clock::now();
            const auto now_time = std::chrono::system_clock::to_time_t(now);
            std::tm time_info{};
            localtime_r(&now_time, &time_info);
            std::ostringstream oss;
            oss << std::put_time(&time_info, format);
            return oss.str();
        }

        static std::string NowString() {
            const auto now = std::chrono::system_clock::now();
            const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    now.time_since_epoch()) %
                                1000;
            std::ostringstream oss;
            oss << TimestampString("%Y-%m-%dT%H:%M:%S") << '.'
                << std::setw(3) << std::setfill('0') << now_ms.count();
            return oss.str();
        }

        static std::string BuildFileName(const std::string& component) {
            std::ostringstream oss;
            oss << "AT_" << TimestampString("%Y%m%d_%H%M%S");
            if (!component.empty()) {
                oss << "_" << component;
            }
            oss << ".log";
            return oss.str();
        }

        bool enabled_ = false;
        std::string path_;
        std::ofstream stream_;
        mutable std::mutex mutex_;
    };

    class CarTrackerSolverNode{

    public:
        // 【修改 1】構造函數清空，只初始化 node
        CarTrackerSolverNode() : node() {
        }

        // 【修改 2】新增 Init 函數，延後初始化
        void Init() {
            // 現在這裡執行時，全域指針已經有值了，Solver 才能讀到參數！
            tracker = ly_auto_aim::tracker::createTracker(); 
            solver = ly_auto_aim::solver::createSolver();

            bool use_matcher_tracking = true;
            node.GetParam("tracker_config.use_matcher_tracking", use_matcher_tracking, true);
            tracker->setUseMatcherTracking(use_matcher_tracking);
            bool use_whole_car_matcher = true;
            node.GetParam("tracker_config.use_whole_car_matcher", use_whole_car_matcher, true);
            tracker->setUseWholeCarMatcher(use_whole_car_matcher);
            InitAimTimerLogger();
            RCLCPP_INFO(
                node.get_logger(),
                "tracker_config.use_matcher_tracking=%s, tracker_config.use_whole_car_matcher=%s",
                use_matcher_tracking ? "true" : "false",
                use_whole_car_matcher ? "true" : "false");
            
            location::Location::registerSolver(solver);
            
            // 訂閱也放在這裡，確保 solver 準備好之後才開始收數據
            node.GenSubscriber<ly_detector_armors>([this](const auto_aim_common::msg::Armors::ConstSharedPtr msg) { 
                detection_callback(msg); 
            });

            RCLCPP_INFO(node.get_logger(), "Tracker Solver Initialized Successfully!");
        }

        ~CarTrackerSolverNode() = default;

        void convertToDetections(const auto_aim_common::msg::Armors::ConstSharedPtr& msg, Detections& detections) {
            detections.clear();
            detections.reserve(msg->armors.size());
            for (const auto& armor : msg->armors) {
                detections.emplace_back(Detection{
                    .tag_id = armor.type,
                    .corners = {
                        {armor.corners_x[0], armor.corners_y[0]},
                        {armor.corners_x[1], armor.corners_y[1]},
                        {armor.corners_x[2], armor.corners_y[2]},
                        {armor.corners_x[3], armor.corners_y[3]}
                    },
                    // 這裡可以傳遞更多信息如果需要
                });
            }
        }

        void convertToCarDetections(const auto_aim_common::msg::Armors::ConstSharedPtr& msg, CarDetections& car_detections) {
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
            }
        }

        void publish_all(const auto& track_results, auto& tracks_msg){
            for(const auto& armor_track_result : track_results.first){
                auto_aim_common::msg::ArmorTracker armor_tracker_msg;
                XYZ armor_xyz = armor_track_result.location.xyz_imu;
                armor_tracker_msg.x = armor_xyz.x;
                armor_tracker_msg.y = armor_xyz.y;
                armor_tracker_msg.z = armor_xyz.z;
                armor_tracker_msg.yaw = armor_track_result.yaw;
                armor_tracker_msg.armor_id = armor_track_result.armor_id;
                armor_tracker_msg.car_id = armor_track_result.car_id;
                tracks_msg.armor_trackers.emplace_back(std::move(armor_tracker_msg));
            }
            for(const auto& car_track_result : track_results.second){
                auto_aim_common::msg::CarTracker car_tracker_msg;
                car_tracker_msg.car_id = car_track_result.car_id;
                car_tracker_msg.bounding_rect.x = car_track_result.bounding_rect.x;
                car_tracker_msg.bounding_rect.y = car_track_result.bounding_rect.y;
                car_tracker_msg.bounding_rect.width = car_track_result.bounding_rect.width;
                car_tracker_msg.bounding_rect.height = car_track_result.bounding_rect.height;
                tracks_msg.car_trackers.emplace_back(std::move(car_tracker_msg));
            }
        }

        void detection_callback(const auto_aim_common::msg::Armors::ConstSharedPtr msg){
            // 雙重保險：如果 solver 還沒初始化，直接返回
            if (!solver) return;

            auto_aim_common::msg::Trackers trackers_msg;
            trackers_msg.header.stamp = msg->header.stamp;
            trackers_msg.header.frame_id = msg->header.frame_id;
            trackers_msg.pitch = msg->pitch; 
            trackers_msg.yaw = msg->yaw;

            std::vector<Detection> detections;
            convertToDetections(msg, detections);
            std::vector<CarDetection> car_detections;
            convertToCarDetections(msg, car_detections);

            GimbalAngleType gimbal_angle{msg->pitch, msg->yaw};
            
            tracker->merge(detections);
            tracker->merge(car_detections);
            
            // =========================================================================
            // 【安全修復】 
            // 這裡為了防止 "Different Time Sources" 報錯，我們也統一轉成 double
            // 這跟 Predictor 的修復邏輯是一樣的，確保萬無一失
            // =========================================================================
            double msg_time_sec = rclcpp::Time(msg->header.stamp).seconds();
            Time::TimeStamp timestamp(msg_time_sec);
            // =========================================================================

            auto track_results = tracker->getTrackResult(timestamp, gimbal_angle);
            
            // solve_all also clears per-frame PnP debug records when no armor is tracked.
            solver->solve_all(track_results, gimbal_angle);
            LogAimTimerFrame(msg, detections.size(), car_detections.size(), track_results);
            
            publish_all(track_results, trackers_msg);
            
            node.Publisher<ly_tracker_results>()->publish(trackers_msg);
        }

        template <typename T>
        void ReadParamWithAlias(
            const std::string& dot_name,
            const std::string& slash_name,
            T& value,
            const T& default_value) {
            if (node.has_parameter(dot_name)) {
                (void)node.get_parameter(dot_name, value);
                return;
            }
            if (node.has_parameter(slash_name)) {
                (void)node.get_parameter(slash_name, value);
                return;
            }
            node.declare_parameter(dot_name, default_value);
            (void)node.get_parameter(dot_name, value);
        }

        void InitAimTimerLogger() {
            bool enabled = false;
            std::string dir = "~/Log/AimTimer";
            ReadParamWithAlias("aim_timer_log.enable", "aim_timer_log/enable", enabled, enabled);
            ReadParamWithAlias("aim_timer_log.dir", "aim_timer_log/dir", dir, dir);

            if (!enabled) {
                return;
            }

            if (aim_timer_logger_.Open(enabled, dir, "tracker_solver")) {
                RCLCPP_INFO(node.get_logger(), "AimTimer tracker_solver log enabled: %s", aim_timer_logger_.Path().c_str());
            } else {
                RCLCPP_WARN(node.get_logger(), "AimTimer tracker_solver log requested but file open failed.");
            }
            if (solver) {
                solver->setPnPDebugEnabled(aim_timer_logger_.Enabled());
            }
        }

        void LogAimTimerFrame(
            const auto_aim_common::msg::Armors::ConstSharedPtr& msg,
            std::size_t detection_count,
            std::size_t car_detection_count,
            const TrackResultPairs& track_results) {
            if (!aim_timer_logger_.Enabled()) {
                return;
            }

            const auto now = node.now();
            const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
            const double input_age_ms = (now.seconds() - stamp_sec) * 1000.0;
            double max_xyz_jump = 0.0;
            double max_yaw_jump_deg = 0.0;
            std::size_t jump_samples = 0;

            for (const auto& track_result : track_results.first) {
                const auto key = std::make_pair(track_result.car_id, track_result.armor_id);
                const auto previous = last_solved_observations_.find(key);
                if (previous != last_solved_observations_.end()) {
                    const auto& prev = previous->second;
                    const XYZ xyz = track_result.location.xyz_imu;
                    const double dx = xyz.x - prev.xyz.x;
                    const double dy = xyz.y - prev.xyz.y;
                    const double dz = xyz.z - prev.xyz.z;
                    max_xyz_jump = std::max(max_xyz_jump, std::sqrt(dx * dx + dy * dy + dz * dz));
                    max_yaw_jump_deg = std::max(
                        max_yaw_jump_deg,
                        std::abs(std::remainder(track_result.yaw - prev.yaw, 2.0 * M_PI)) * kRadToDeg);
                    ++jump_samples;
                }
                last_solved_observations_[key] = TrackerObservationSnapshot{
                    track_result.location.xyz_imu,
                    track_result.yaw};
            }

            {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3)
                    << "node=tracker_solver event=tracker_frame"
                    << " stamp_sec=" << stamp_sec
                    << " now_sec=" << now.seconds()
                    << " input_age_ms=" << input_age_ms
                    << " det_armors=" << detection_count
                    << " det_cars=" << car_detection_count
                    << " track_armors=" << track_results.first.size()
                    << " track_cars=" << track_results.second.size()
                    << " pnp_records=" << (solver ? solver->getLastPnPDebugRecords().size() : 0)
                    << " max_xyz_jump=" << max_xyz_jump
                    << " max_yaw_jump_deg=" << max_yaw_jump_deg
                    << " jump_samples=" << jump_samples
                    << " gimbal_pitch_deg=" << msg->pitch
                    << " gimbal_yaw_deg=" << msg->yaw;
                aim_timer_logger_.Write(oss.str());
            }

            for (const auto& track_result : track_results.first) {
                const XYZ xyz = track_result.location.xyz_imu;
                const double distance = std::sqrt(xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z);
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(4)
                    << "node=tracker_solver event=armor_world"
                    << " car_id=" << track_result.car_id
                    << " armor_id=" << track_result.armor_id
                    << " x=" << xyz.x
                    << " y=" << xyz.y
                    << " z=" << xyz.z
                    << " distance=" << distance
                    << " yaw_rad=" << track_result.yaw
                    << " yaw_deg=" << track_result.yaw * kRadToDeg;
                aim_timer_logger_.Write(oss.str());
            }

            if (!solver) {
                return;
            }
            for (const auto& record : solver->getLastPnPDebugRecords()) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(4)
                    << "node=tracker_solver event=pnp"
                    << " mode=" << (record.used_whole_car ? "whole_car" : "armor_only")
                    << " car_id=" << record.car_id
                    << " armor_id=" << record.armor_id
                    << " valid=" << (record.valid ? 1 : 0)
                    << " solutions=" << record.solution_count
                    << " selected=" << record.selected_index
                    << " has_history=" << (record.has_yaw_history ? 1 : 0)
                    << " has_whole_car_hint=" << (record.has_whole_car_yaw_hint ? 1 : 0)
                    << " reproj_px=" << record.reprojection_error_px
                    << " score=" << record.score
                    << " armor_yaw_deg=" << record.armor_yaw_rad * kRadToDeg
                    << " world_yaw_deg=" << record.world_yaw_rad * kRadToDeg
                    << " whole_car_hint_deg=" << record.whole_car_yaw_hint_rad * kRadToDeg
                    << " world_x=" << record.world_x
                    << " world_y=" << record.world_y
                    << " world_z=" << record.world_z;
                aim_timer_logger_.Write(oss.str());
            }
        }

    public:
        ROSNode<AppName> node;
    private:
        struct TrackerObservationSnapshot {
            XYZ xyz;
            double yaw = 0.0;
        };
        std::mutex data_mutex;
        std::unique_ptr<ly_auto_aim::tracker::Tracker> tracker;
        std::shared_ptr<ly_auto_aim::solver::Solver> solver;
        ly_auto_aim::solver::CameraIntrinsicsParameterPack cameraIntrinsics{};
        AimTimerFileLogger aim_timer_logger_;
        std::map<std::pair<int, int>, TrackerObservationSnapshot> last_solved_observations_;
    };
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    auto app = std::make_shared<CarTrackerSolverNode>();
    std::shared_ptr<rclcpp::Node> node_ptr(&app->node, [](auto*){});
    
    // 【修改 3】先賦值，後初始化
    ly_auto_aim::solver::global_tracker_solver_node = node_ptr;
    global_node_ptr = node_ptr; 

    // 執行初始化邏輯 (此時 global_tracker_solver_node 已就緒)
    app->Init();

    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
