// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

/*
 * predictor_node
 *
 * 链路位置：
 * /ly/tracker/results -> predictor/controller -> /ly/predictor/target
 *
 * 主要功能：
 * - 根据 tracker 结果做状态预测
 * - 结合当前目标类型(/ly/bt/target)与弹速(/ly/bullet/speed)输出控制目标
 * - 发布 debug_filter 供可视化和调参
 */
#include <rclcpp/rclcpp.hpp>
#include <RosTools/RosTools.hpp>
#include "Logger/Logger.hpp"
#include <TimeStamp/TimeStamp.hpp>
#include <auto_aim_common/Location.hpp>
#include <auto_aim_common/TrackerType.hpp>
#include <auto_aim_common/PredictionType.hpp>
#include <auto_aim_common/DetectionType.hpp>

// [ROS 2] 消息頭文件
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <auto_aim_common/msg/car_tracker.hpp>
#include <auto_aim_common/msg/armor_tracker.hpp>
#include <auto_aim_common/msg/trackers.hpp>
#include <auto_aim_common/msg/target.hpp>
#include <auto_aim_common/msg/debug_filter.hpp>
#include <auto_aim_common/msg/predictor_vis.hpp>

#include "predictor/predictor.hpp"
#include "controller/controller.hpp"
#include "solver/solver.hpp"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

using namespace LangYa;
using namespace ly_auto_aim;

namespace {
    LY_DEF_ROS_TOPIC(ly_tracker_results, "/ly/tracker/results", auto_aim_common::msg::Trackers);
    LY_DEF_ROS_TOPIC(ly_predictor_target, "/ly/predictor/target", auto_aim_common::msg::Target);
    LY_DEF_ROS_TOPIC(ly_predictor_debug, "/ly/predictor/debug", auto_aim_common::msg::DebugFilter);
    LY_DEF_ROS_TOPIC(ly_predictor_vis, "/ly/predictor/vis", auto_aim_common::msg::PredictorVis);
    LY_DEF_ROS_TOPIC(ly_bt_target, "/ly/bt/target", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_bullet_speed, "/ly/bullet/speed", std_msgs::msg::Float32);
    
    constexpr const char AppName[] = "predictor_node";
    std::atomic<ArmorType> automic_target;
    std::atomic<float> atomic_bullet_speed{23.0f};

    const char* InvalidReasonToString(const ly_auto_aim::controller::ControlInvalidReason reason) {
        using ly_auto_aim::controller::ControlInvalidReason;
        switch (reason) {
            case ControlInvalidReason::None: return "none";
            case ControlInvalidReason::NoPrediction: return "no_prediction";
            case ControlInvalidReason::InvalidCar: return "invalid_car";
            case ControlInvalidReason::InvalidCarAfterFlyTime: return "invalid_car_after_flytime";
            case ControlInvalidReason::NoArmorFallbackBallisticFail: return "no_armor_fallback_ballistic_fail";
            case ControlInvalidReason::InvalidArmor: return "invalid_armor";
            case ControlInvalidReason::ArmorBallisticFail: return "armor_ballistic_fail";
            case ControlInvalidReason::UnstableTrack: return "unstable_track";
            default: return "unknown";
        }
    }

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

    class PredictorNode {
        public:
            // 【修改 1】構造函數清空！只做最基本的 node 初始化
            PredictorNode() : node() { 
            }

            // 【修改 2】Init 函數，延後初始化
            void Init() {
                solver = ly_auto_aim::solver::createSolver(); 
                predictor = ly_auto_aim::predictor::createPredictor();
                controller = ly_auto_aim::controller::createController();
                node.GetParam(
                    "predictor_config.publish_only_on_new_tracker_frame",
                    publish_only_on_new_tracker_frame_,
                    publish_only_on_new_tracker_frame_);
                node.GetParam(
                    "predictor_config.publish_on_tracker_callback",
                    publish_on_tracker_callback_,
                    publish_on_tracker_callback_);
                node.GetParam(
                    "predictor_config.require_observation_fresh_for_target",
                    require_observation_fresh_for_target_,
                    require_observation_fresh_for_target_);
                double coast_timeout_sec = coast_timeout_.seconds();
                node.GetParam(
                    "predictor_config.coast_timeout_sec",
                    coast_timeout_sec,
                    coast_timeout_sec);
                coast_timeout_ = rclcpp::Duration::from_seconds(coast_timeout_sec);
                double max_tracker_age_sec = max_tracker_age_.seconds();
                node.GetParam(
                    "predictor_config.max_tracker_age_sec",
                    max_tracker_age_sec,
                    max_tracker_age_sec);
                max_tracker_age_ = rclcpp::Duration::from_seconds(max_tracker_age_sec);
                InitAimTimerLogger();
                RCLCPP_INFO(
                    node.get_logger(),
                    "predictor_config.publish_on_tracker_callback=%s, predictor_config.publish_only_on_new_tracker_frame=%s, predictor_config.require_observation_fresh_for_target=%s, predictor_config.coast_timeout_sec=%.3f, predictor_config.max_tracker_age_sec=%.3f",
                    publish_on_tracker_callback_ ? "true" : "false",
                    publish_only_on_new_tracker_frame_ ? "true" : "false",
                    require_observation_fresh_for_target_ ? "true" : "false",
                    coast_timeout_.seconds(),
                    max_tracker_age_.seconds());
                
                location::Location::registerSolver(solver);
                
                controller->registPredictFunc([this](Time::TimeStamp timestamp) {
                    const double t_sec = rclcpp::Time(timestamp).seconds();
                    return predictor->predict(Time::TimeStamp(t_sec));
                });

                node.GenSubscriber<ly_tracker_results>([this](const auto_aim_common::msg::Trackers::ConstSharedPtr msg) { 
                    predictor_callback(msg); 
                });
                node.GenSubscriber<ly_bt_target>([this](const std_msgs::msg::UInt8::ConstSharedPtr msg) { 
                    get_target_callback(msg); 
                });
                node.GenSubscriber<ly_bullet_speed>([this](const std_msgs::msg::Float32::ConstSharedPtr msg) { 
                    get_bullet_speed_callback(msg); 
                });

                if (!publish_on_tracker_callback_) {
                    publish_timer_ = node.create_wall_timer(
                        std::chrono::milliseconds(10),
                        [this]() { publish_timer_callback(); });
                }
                
                RCLCPP_INFO(node.get_logger(), "Predictor Modules Initialized Successfully!");
            }

            ~PredictorNode() = default;

            void get_target_callback(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
                automic_target = static_cast<ArmorType>(msg->data);
            }

            void get_bullet_speed_callback(const std_msgs::msg::Float32::ConstSharedPtr msg) {
                atomic_bullet_speed = static_cast<float>(msg->data);
            }

            void convertMsgToTrackResults(const auto_aim_common::msg::Trackers::ConstSharedPtr& msg, TrackResultPairs& track_results, GimbalAngleType gimbal_angle){
                track_results.first.clear();
                track_results.second.clear();
                for (const auto& armor_tra_msg : msg->armor_trackers) {
                    TrackResult armor_track_result;
                    armor_track_result.car_id = armor_tra_msg.car_id;
                    armor_track_result.armor_id = armor_tra_msg.armor_id;
                    armor_track_result.yaw = armor_tra_msg.yaw;
                    armor_track_result.location.imu = gimbal_angle;
                    XYZ armor_xyz(armor_tra_msg.x, armor_tra_msg.y, armor_tra_msg.z); 
                    armor_track_result.location.xyz_imu = armor_xyz;
                    track_results.first.emplace_back(std::move(armor_track_result));
                }
                for (const auto& car_tra_msg : msg->car_trackers) {
                    CarTrackResult car_track_result;
                    car_track_result.car_id = car_tra_msg.car_id;
                    car_track_result.bounding_rect.x = car_tra_msg.bounding_rect.x;
                    car_track_result.bounding_rect.y = car_tra_msg.bounding_rect.y;
                    car_track_result.bounding_rect.width = car_tra_msg.bounding_rect.width;
                    car_track_result.bounding_rect.height = car_tra_msg.bounding_rect.height;
                    track_results.second.emplace_back(std::move(car_track_result));
                }
            }

            void predictor_callback(const auto_aim_common::msg::Trackers::ConstSharedPtr msg){
                GimbalAngleType gimbal_angle{msg->pitch, msg->yaw};

                // 增加一個保護，避免還沒初始化就調用
                if(!location::Location::isSolverRegistered()){
                    return; 
                }

                TrackResultPairs track_results;
                convertMsgToTrackResults(msg, track_results, gimbal_angle);
                const auto tracker_stamp = rclcpp::Time(msg->header.stamp);
                const double msg_time_sec = tracker_stamp.seconds();
                Time::TimeStamp timestamp(msg_time_sec);

                auto_aim_common::msg::Target target_msg;
                auto_aim_common::msg::DebugFilter debug_filter_msg;
                auto_aim_common::msg::PredictorVis predictor_vis_msg;
                bool publish_target = false;
                bool publish_debug = false;
                bool publish_vis = false;

                std::lock_guard<std::mutex> lock(data_mutex);
                const auto target = static_cast<int>(automic_target.load());
                const auto bullet_speed = static_cast<float>(atomic_bullet_speed.load());
                ly_auto_aim::controller::ControlResult control_result{};
                bool finite_target = true;
                if (publish_on_tracker_callback_) {
                    control_result = controller->control(gimbal_angle, target, bullet_speed);
                    target_msg.status = control_result.valid;
                    target_msg.yaw = control_result.yaw_actual_want;
                    target_msg.pitch = control_result.pitch_actual_want;
                    target_msg.header = msg->header;
                    target_msg.buff_follow = false;
                    finite_target = std::isfinite(target_msg.yaw) && std::isfinite(target_msg.pitch);
                    publish_target = target_msg.status && finite_target;
                }

                double max_xyz_jump = 0.0;
                double max_yaw_jump_deg = 0.0;
                std::size_t jump_sample_count = 0;
                for (const auto& track_result : track_results.first) {
                    const auto key = std::make_pair(track_result.car_id, track_result.armor_id);
                    const auto previous = last_tracker_observations_.find(key);
                    if (previous != last_tracker_observations_.end()) {
                        const auto& prev = previous->second;
                        const XYZ now_xyz = track_result.location.xyz_imu;
                        const double dx = now_xyz.x - prev.xyz.x;
                        const double dy = now_xyz.y - prev.xyz.y;
                        const double dz = now_xyz.z - prev.xyz.z;
                        max_xyz_jump = std::max(max_xyz_jump, std::sqrt(dx * dx + dy * dy + dz * dz));
                        max_yaw_jump_deg = std::max(
                            max_yaw_jump_deg,
                            std::abs(std::remainder(track_result.yaw - prev.yaw, 2.0 * M_PI)) * 180.0 / M_PI);
                        ++jump_sample_count;
                    }
                    last_tracker_observations_[key] = TrackerObservationSnapshot{
                        track_result.location.xyz_imu,
                        track_result.yaw};
                }
                const auto update_stats = predictor->update(track_results, timestamp);
                const auto callback_time = node.now();
                last_gimbal_angle_ = gimbal_angle;
                last_tracker_header_ = msg->header;
                last_update_time_ = callback_time;
                has_tracker_input_ = true;
                has_new_tracker_frame_.store(true, std::memory_order_release);
                if (update_stats.model_update_count > 0) {
                    last_observation_time_ = last_update_time_;
                    last_observation_stamp_ = tracker_stamp;
                }
                LogAimTimerTrackerUpdate(
                    update_stats,
                    track_results,
                    callback_time,
                    tracker_stamp,
                    max_xyz_jump,
                    max_yaw_jump_deg,
                    jump_sample_count);
                if (publish_on_tracker_callback_) {
                    const auto predictions = predictor->predict(timestamp);
                    FillPredictorOutputs(
                        predictions,
                        predictor_vis_msg,
                        debug_filter_msg,
                        msg->header);
                    publish_vis = true;
                    publish_debug = publish_target && !predictions.empty();
                    if (!publish_target && finite_target &&
                        (last_invalid_reason_log_time_.nanoseconds() == 0 ||
                         (callback_time - last_invalid_reason_log_time_) > invalid_reason_log_interval_)) {
                        RCLCPP_INFO(
                            node.get_logger(),
                            "predictor callback target suppressed reason=%s finite_target=%s yaw=%.2f pitch=%.2f",
                            InvalidReasonToString(control_result.invalid_reason),
                            finite_target ? "true" : "false",
                            target_msg.yaw,
                            target_msg.pitch);
                        last_invalid_reason_log_time_ = callback_time;
                    }
                    LogAimTimerTarget(
                        callback_time,
                        target,
                        bullet_speed,
                        predictions.size(),
                        !predictions.empty(),
                        true,
                        true,
                        true,
                        0.0,
                        (callback_time - tracker_stamp).seconds() * 1000.0,
                        finite_target,
                        publish_target,
                        target_msg,
                        std::string("tracker_callback:") + InvalidReasonToString(control_result.invalid_reason));
                }
                if (last_update_stats_log_time_.nanoseconds() == 0 ||
                    (callback_time - last_update_stats_log_time_) > update_stats_log_interval_) {
                    RCLCPP_INFO(
                        node.get_logger(),
                        "predictor update stats: armors=%zu cars=%zu model_updates=%zu tracker_age_ms=%.1f max_xyz_jump=%.3f max_yaw_jump_deg=%.2f jump_samples=%zu",
                        update_stats.armor_count,
                        update_stats.car_count,
                        update_stats.model_update_count,
                        (callback_time - tracker_stamp).seconds() * 1000.0,
                        max_xyz_jump,
                        max_yaw_jump_deg,
                        jump_sample_count);
                    last_update_stats_log_time_ = callback_time;
                }

                if (publish_target) {
                    node.Publisher<ly_predictor_target>()->publish(target_msg);
                }
                if (publish_debug) {
                    node.Publisher<ly_predictor_debug>()->publish(debug_filter_msg);
                }
                if (publish_vis) {
                    node.Publisher<ly_predictor_vis>()->publish(predictor_vis_msg);
                }
            }

            void publish_timer_callback() {
                if (!has_tracker_input_ || !location::Location::isSolverRegistered()) {
                    return;
                }
                if (publish_only_on_new_tracker_frame_ &&
                    !has_new_tracker_frame_.exchange(false, std::memory_order_acq_rel)) {
                    return;
                }

                auto_aim_common::msg::Target target_msg;
                auto_aim_common::msg::DebugFilter debug_filter_msg;
                auto_aim_common::msg::PredictorVis predictor_vis_msg;
                bool publish_target = false;
                bool publish_debug = false;
                bool publish_vis = false;

                {
                    std::lock_guard<std::mutex> lock(data_mutex);

                    const auto now = node.now();
                    const double now_sec = now.seconds();
                    const auto target = static_cast<int>(automic_target.load());
                    const auto bullet_speed = static_cast<float>(atomic_bullet_speed.load());
                    const Time::TimeStamp timestamp(now_sec);
                    const auto predictions = predictor->predict(timestamp);
                    const bool has_predictions = !predictions.empty();
                    const bool observation_receive_fresh =
                        last_observation_time_.nanoseconds() != 0 &&
                        (now - last_observation_time_) <= coast_timeout_;
                    const bool tracker_stamp_fresh =
                        last_observation_stamp_.nanoseconds() != 0 &&
                        (now - last_observation_stamp_) <= max_tracker_age_;
                    const bool observation_fresh = observation_receive_fresh && tracker_stamp_fresh;
                    const auto observation_age_ms =
                        last_observation_time_.nanoseconds() == 0
                            ? -1.0
                            : (now - last_observation_time_).seconds() * 1000.0;
                    const auto tracker_stamp_age_ms =
                        last_observation_stamp_.nanoseconds() == 0
                            ? -1.0
                            : (now - last_observation_stamp_).seconds() * 1000.0;
                    bool finite_target = true;
                    std::string aim_timer_reason = "not_evaluated";
                    const bool can_log_invalid_reason =
                        last_invalid_reason_log_time_.nanoseconds() == 0 ||
                        (now - last_invalid_reason_log_time_) > invalid_reason_log_interval_;

                    target_msg.header = last_tracker_header_;
                    target_msg.header.stamp = now;
                    target_msg.buff_follow = false;
                    FillPredictorOutputs(
                        predictions,
                        predictor_vis_msg,
                        debug_filter_msg,
                        target_msg.header);
                    publish_vis = true;

                    if ((require_observation_fresh_for_target_ && !observation_fresh) ||
                        (!has_predictions && !observation_fresh)) {
                        // Keep the predictor's robust stale/no-prediction handling,
                        // but do not publish invalid targets to behavior_tree.
                        target_msg.status = false;
                        finite_target = std::isfinite(target_msg.yaw) && std::isfinite(target_msg.pitch);
                        aim_timer_reason = has_predictions ? "observation_stale" : "no_predictions_and_stale";
                    } else {
                        const auto control_result =
                            controller->control(last_gimbal_angle_, target, bullet_speed);
                        target_msg.status = control_result.valid;
                        target_msg.yaw = control_result.yaw_actual_want;
                        target_msg.pitch = control_result.pitch_actual_want;
                        aim_timer_reason = InvalidReasonToString(control_result.invalid_reason);

                        finite_target =
                            std::isfinite(target_msg.yaw) && std::isfinite(target_msg.pitch);
                        if (target_msg.status && finite_target) {
                            publish_target = true;
                            publish_debug = has_predictions;
                        }

                        if ((!target_msg.status || !finite_target) && can_log_invalid_reason) {
                            RCLCPP_INFO(
                                node.get_logger(),
                                "predictor target suppressed reason=%s has_predictions=%s observation_fresh=%s receive_age_ms=%.1f tracker_stamp_age_ms=%.1f finite_target=%s yaw=%.2f pitch=%.2f",
                                InvalidReasonToString(control_result.invalid_reason),
                                has_predictions ? "true" : "false",
                                observation_fresh ? "true" : "false",
                                observation_age_ms,
                                tracker_stamp_age_ms,
                                finite_target ? "true" : "false",
                                target_msg.yaw,
                                target_msg.pitch);
                            last_invalid_reason_log_time_ = now;
                        }
                    }

                    if (!target_msg.status && !has_predictions && !observation_fresh &&
                        can_log_invalid_reason) {
                        RCLCPP_INFO(
                            node.get_logger(),
                            "predictor status=false reason=no_predictions_and_stale has_predictions=false observation_fresh=false receive_age_ms=%.1f tracker_stamp_age_ms=%.1f",
                            observation_age_ms,
                            tracker_stamp_age_ms);
                        last_invalid_reason_log_time_ = now;
                    }
                    LogAimTimerTarget(
                        now,
                        target,
                        bullet_speed,
                        predictions.size(),
                        has_predictions,
                        observation_receive_fresh,
                        tracker_stamp_fresh,
                        observation_fresh,
                        observation_age_ms,
                        tracker_stamp_age_ms,
                        finite_target,
                        publish_target,
                        target_msg,
                        aim_timer_reason);
                }

                if (publish_target) {
                    node.Publisher<ly_predictor_target>()->publish(target_msg);
                }
                if (publish_debug) {
                    node.Publisher<ly_predictor_debug>()->publish(debug_filter_msg);
                }
                if (publish_vis) {
                    node.Publisher<ly_predictor_vis>()->publish(predictor_vis_msg);
                }
            }

            void FillPredictorOutputs(
                const ly_auto_aim::predictor::Predictions& predictions,
                auto_aim_common::msg::PredictorVis& predictor_vis_msg,
                auto_aim_common::msg::DebugFilter& debug_filter_msg,
                const std_msgs::msg::Header& header) {
                predictor_vis_msg.header = header;
                predictor_vis_msg.has_predictions = !predictions.empty();
                predictor_vis_msg.aimed_car_id = -1;
                predictor_vis_msg.aimed_armor_id = -1;
                predictor_vis_msg.cars.clear();

                if (predictions.empty()) {
                    return;
                }

                predictor_vis_msg.cars.reserve(predictions.size());
                for (const auto& prediction : predictions) {
                    auto_aim_common::msg::PredictorCarVis car_vis_msg;
                    car_vis_msg.car_id = prediction.id;
                    car_vis_msg.stable = prediction.stable;
                    car_vis_msg.center.x = prediction.center.x;
                    car_vis_msg.center.y = prediction.center.y;
                    car_vis_msg.center.z = prediction.center.z;
                    car_vis_msg.armors.reserve(prediction.armors.size());
                    for (const auto& armor : prediction.armors) {
                        auto_aim_common::msg::PredictorArmorVis armor_vis_msg;
                        armor_vis_msg.id = armor.id;
                        armor_vis_msg.status = static_cast<std::int32_t>(armor.status);
                        armor_vis_msg.center.x = armor.center.x;
                        armor_vis_msg.center.y = armor.center.y;
                        armor_vis_msg.center.z = armor.center.z;
                        armor_vis_msg.yaw = static_cast<float>(armor.yaw);
                        armor_vis_msg.theta = static_cast<float>(armor.theta);
                        car_vis_msg.armors.push_back(std::move(armor_vis_msg));
                    }
                    predictor_vis_msg.cars.push_back(std::move(car_vis_msg));

                    debug_filter_msg.tracking = true;
                    XYZ car_XYZ = prediction.center;
                    debug_filter_msg.position.x = car_XYZ.x;
                    debug_filter_msg.position.y = car_XYZ.y;
                    debug_filter_msg.position.z = car_XYZ.z;
                    debug_filter_msg.yaw = prediction.theta;
                    debug_filter_msg.v_yaw = prediction.omega;
                    debug_filter_msg.velocity.x = prediction.vx;
                    debug_filter_msg.velocity.y = prediction.vy;
                    debug_filter_msg.velocity.z = 0.0;
                    debug_filter_msg.radius_1 = prediction.r1;
                    debug_filter_msg.radius_2 = prediction.r2;
                    debug_filter_msg.z_2 = prediction.z2;
                }
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

                if (aim_timer_logger_.Open(enabled, dir, "predictor")) {
                    RCLCPP_INFO(node.get_logger(), "AimTimer predictor log enabled: %s", aim_timer_logger_.Path().c_str());
                } else {
                    RCLCPP_WARN(node.get_logger(), "AimTimer predictor log requested but file open failed.");
                }
            }

            void LogAimTimerTrackerUpdate(
                const PredictorUpdateStats& stats,
                const TrackResultPairs& track_results,
                const rclcpp::Time& callback_time,
                const rclcpp::Time& tracker_stamp,
                double max_xyz_jump,
                double max_yaw_jump_deg,
                std::size_t jump_sample_count) {
                if (!aim_timer_logger_.Enabled()) {
                    return;
                }

                {
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(3)
                        << "node=predictor event=tracker_update"
                        << " stamp_sec=" << tracker_stamp.seconds()
                        << " callback_sec=" << callback_time.seconds()
                        << " tracker_age_ms=" << (callback_time.seconds() - tracker_stamp.seconds()) * 1000.0
                        << " armors=" << stats.armor_count
                        << " cars=" << stats.car_count
                        << " model_updates=" << stats.model_update_count
                        << " max_xyz_jump=" << max_xyz_jump
                        << " max_yaw_jump_deg=" << max_yaw_jump_deg
                        << " jump_samples=" << jump_sample_count
                        << " gimbal_pitch_deg=" << last_gimbal_angle_.pitch
                        << " gimbal_yaw_deg=" << last_gimbal_angle_.yaw;
                    aim_timer_logger_.Write(oss.str());
                }

                for (const auto& track_result : track_results.first) {
                    const XYZ xyz = track_result.location.xyz_imu;
                    const double distance = std::sqrt(xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z);
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(4)
                        << "node=predictor event=tracker_armor_input"
                        << " car_id=" << track_result.car_id
                        << " armor_id=" << track_result.armor_id
                        << " x=" << xyz.x
                        << " y=" << xyz.y
                        << " z=" << xyz.z
                        << " distance=" << distance
                        << " yaw_rad=" << track_result.yaw
                        << " yaw_deg=" << track_result.yaw * 180.0 / M_PI;
                    aim_timer_logger_.Write(oss.str());
                }
            }

            void LogAimTimerTarget(
                const rclcpp::Time& now,
                int target,
                float bullet_speed,
                std::size_t prediction_count,
                bool has_predictions,
                bool observation_receive_fresh,
                bool tracker_stamp_fresh,
                bool observation_fresh,
                double observation_age_ms,
                double tracker_stamp_age_ms,
                bool finite_target,
                bool publish_target,
                const auto_aim_common::msg::Target& target_msg,
                const std::string& reason) {
                if (!aim_timer_logger_.Enabled()) {
                    return;
                }

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3)
                    << "node=predictor event=target_timer"
                    << " now_sec=" << now.seconds()
                    << " target_type=" << target
                    << " bullet_speed=" << bullet_speed
                    << " prediction_count=" << prediction_count
                    << " has_predictions=" << (has_predictions ? 1 : 0)
                    << " receive_fresh=" << (observation_receive_fresh ? 1 : 0)
                    << " tracker_stamp_fresh=" << (tracker_stamp_fresh ? 1 : 0)
                    << " observation_fresh=" << (observation_fresh ? 1 : 0)
                    << " receive_age_ms=" << observation_age_ms
                    << " tracker_stamp_age_ms=" << tracker_stamp_age_ms
                    << " status=" << (target_msg.status ? 1 : 0)
                    << " publish=" << (publish_target ? 1 : 0)
                    << " finite_target=" << (finite_target ? 1 : 0)
                    << " yaw_cmd_deg=" << target_msg.yaw
                    << " pitch_cmd_deg=" << target_msg.pitch
                    << " gimbal_yaw_deg=" << last_gimbal_angle_.yaw
                    << " gimbal_pitch_deg=" << last_gimbal_angle_.pitch
                    << " reason=" << reason;
                aim_timer_logger_.Write(oss.str());
            }

        public: 
            ROSNode<AppName> node;
        private:
            std::shared_ptr<ly_auto_aim::solver::Solver> solver;
            std::unique_ptr<ly_auto_aim::predictor::Predictor> predictor;
            std::shared_ptr<ly_auto_aim::controller::Controller> controller;
            std::mutex data_mutex;
            struct TrackerObservationSnapshot {
                XYZ xyz;
                double yaw = 0.0;
            };
            std::map<std::pair<int, int>, TrackerObservationSnapshot> last_tracker_observations_;
            rclcpp::TimerBase::SharedPtr publish_timer_{};
            GimbalAngleType last_gimbal_angle_{0.0, 0.0};
            std_msgs::msg::Header last_tracker_header_{};
            rclcpp::Time last_update_time_{};
            rclcpp::Time last_observation_time_{};
            rclcpp::Time last_observation_stamp_{};
            rclcpp::Time last_update_stats_log_time_{};
            std::atomic_bool has_tracker_input_{false};
            std::atomic_bool has_new_tracker_frame_{false};
            bool publish_on_tracker_callback_{true};
            bool publish_only_on_new_tracker_frame_{false};
            bool require_observation_fresh_for_target_{false};
            rclcpp::Duration coast_timeout_{rclcpp::Duration::from_seconds(0.10)};
            rclcpp::Duration max_tracker_age_{rclcpp::Duration::from_seconds(0.15)};
            rclcpp::Time last_invalid_reason_log_time_{};
            const rclcpp::Duration invalid_reason_log_interval_{rclcpp::Duration::from_seconds(0.5)};
            const rclcpp::Duration update_stats_log_interval_{rclcpp::Duration::from_seconds(1.0)};
            AimTimerFileLogger aim_timer_logger_;
    };
}

int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    rclcpp::init(argc, argv);
    auto app = std::make_shared<PredictorNode>();
    std::shared_ptr<rclcpp::Node> node_ptr(&app->node, [](auto*){});

    ly_auto_aim::controller::global_controller_node = node_ptr;
    ly_auto_aim::solver::global_predictor_solver_node = node_ptr;
    ly_auto_aim::predictor::global_predictor_node = node_ptr;

    app->Init();

    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
