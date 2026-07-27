// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

/*
 * gimbal_driver 主节点
 *
 * 核心职责：
 * 1) 订阅 /ly/control/*（上位机控制指令）
 * 2) 与下位机串口设备交互（真实设备或虚拟设备）
 * 3) 发布 /ly/gimbal/* 与 /ly/game/*（回传状态）
 *
 * 备注：
 * - /ly/control/sentry_cmd 为完整裁判命令输入，/ly/control/posture 为姿态命令输入。
 * - /ly/gimbal/posture 发布下位机/裁判姿态回读状态，不镜像上位机命令。
 */
#include <chrono>
#include <thread>
#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <mutex>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/executors.hpp>
#include <sstream>
#include <type_traits>

#include "gimbal_driver/msg/gimbal_angles.hpp"
#include "gimbal_driver/msg/chassis.hpp"
#include "gimbal_driver/msg/control_velocity.hpp"
#include "gimbal_driver/msg/custom_info.hpp"
#include "gimbal_driver/msg/event_data.hpp"
#include "gimbal_driver/msg/fire_code.hpp"
#include "gimbal_driver/msg/rfid_status.hpp"
#include "gimbal_driver/msg/gimbal_raw_frame.hpp"
#include "gimbal_driver/msg/uwb_pos.hpp"
#include "gimbal_driver/msg/vel.hpp"
#include "gimbal_driver/msg/health.hpp"
#include "gimbal_driver/msg/game_data.hpp"
#include "gimbal_driver/msg/map_command.hpp"
#include "gimbal_driver/msg/map_path.hpp"
#include "gimbal_driver/msg/buff_data.hpp"
#include "gimbal_driver/msg/bullet_info.hpp"
#include "gimbal_driver/msg/position_data.hpp"
#include "gimbal_driver/msg/sentry_cmd.hpp"
#include "gimbal_driver/msg/sentry_info.hpp"
#include "gimbal_driver/msg/stamped_int16.hpp"
#include "gimbal_driver/msg/stamped_u_int16.hpp"
#include "gimbal_driver/msg/stamped_u_int16_multi_array.hpp"
#include "gimbal_driver/msg/gimbal_state.hpp"
#include "gimbal_driver/msg/gimbal_trajectory.hpp"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "module/BasicTypes.hpp"
#include "module/EnhancedPostureGuard.hpp"
#include "module/MapPathRateLimiter.hpp"
#include "module/crc_checker.hpp"
#include "module/IODevice.hpp"
#include "module/ROSTools.hpp"
#include "RawDownlinkTest.hpp"

using namespace LangYa;

namespace
{
    LY_DEF_ROS_TOPIC(ly_control_angles, "/ly/control/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_control_trajectory, "/ly/control/trajectory", gimbal_driver::msg::GimbalTrajectory);
    LY_DEF_ROS_TOPIC(ly_control_firecode, "/ly/control/firecode", gimbal_driver::msg::FireCode);
    LY_DEF_ROS_TOPIC(ly_control_vel, "/ly/control/vel", gimbal_driver::msg::ControlVelocity);
    LY_DEF_ROS_TOPIC(ly_control_posture, "/ly/control/posture", gimbal_driver::msg::SentryCmd);
    LY_DEF_ROS_TOPIC(ly_control_sentry_cmd, "/ly/control/sentry_cmd", gimbal_driver::msg::SentryCmd);
    LY_DEF_ROS_TOPIC(ly_control_map_path, "/ly/control/map_path", gimbal_driver::msg::MapPath);
    LY_DEF_ROS_TOPIC(ly_game_path, "/ly/game/path", gimbal_driver::msg::MapPath);
    LY_DEF_ROS_TOPIC(ly_control_custom_info, "/ly/control/custom_info", gimbal_driver::msg::CustomInfo);
    LY_DEF_ROS_TOPIC(ly_navi_vel, "/ly/navi/vel", gimbal_driver::msg::Vel);
    LY_DEF_ROS_TOPIC(ly_navi_should_rotate, "/ly/navi/should_rotate", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_bt_sentry_position, "/ly/bt/sentry_position", geometry_msgs::msg::PointStamped);

    LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_gimbal_state, "/ly/gimbal/state", gimbal_driver::msg::GimbalState);
    LY_DEF_ROS_TOPIC(ly_gimbal_firecode, "/ly/gimbal/firecode", gimbal_driver::msg::FireCode);
    LY_DEF_ROS_TOPIC(ly_gimbal_vel, "/ly/gimbal/vel", gimbal_driver::msg::Vel);
    LY_DEF_ROS_TOPIC(ly_gimbal_chassis, "/ly/gimbal/chassis", gimbal_driver::msg::Chassis);
    LY_DEF_ROS_TOPIC(ly_gimbal_big_yaw_angles, "/ly/gimbal/big_yaw_angles", std_msgs::msg::Float32);
    LY_DEF_ROS_TOPIC(ly_gimbal_posture, "/ly/gimbal/posture", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_gimbal_capV, "/ly/gimbal/capV", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_game_eventdata, "ly/gimbal/eventdata", std_msgs::msg::UInt32);
    LY_DEF_ROS_TOPIC(ly_game_event_data, "/ly/game/event_data", gimbal_driver::msg::EventData);

    LY_DEF_ROS_TOPIC(ly_friend_is_precaution, "/ly/friend/is_precaution", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_friend_is_at_home, "/ly/friend/is_at_home", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_friend_is_team_red, "/ly/friend/is_team_red", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_friend_hp, "/ly/friend/hp", gimbal_driver::msg::Health);
    LY_DEF_ROS_TOPIC(ly_friend_op_hp, "/ly/friend/op_hp", gimbal_driver::msg::StampedUInt16);
    LY_DEF_ROS_TOPIC(ly_friend_base_hp, "/ly/friend/base_hp", gimbal_driver::msg::StampedUInt16);

    LY_DEF_ROS_TOPIC(ly_friend_ammo_left, "/ly/friend/ammo_left", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_friend_uwb_pos, "/ly/friend/uwb_pos", gimbal_driver::msg::StampedUInt16MultiArray);
    LY_DEF_ROS_TOPIC(ly_friend_uwb_yaw, "/ly/friend/uwb_yaw", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_game_is_start, "/ly/game/is_start", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_game_time_left, "/ly/game/time_left", gimbal_driver::msg::StampedUInt16);

    LY_DEF_ROS_TOPIC(ly_enemy_hp, "/ly/enemy/hp", gimbal_driver::msg::Health);
    LY_DEF_ROS_TOPIC(ly_enemy_op_hp, "/ly/enemy/op_hp", gimbal_driver::msg::StampedUInt16);
    LY_DEF_ROS_TOPIC(ly_enemy_base_hp, "/ly/enemy/base_hp", gimbal_driver::msg::StampedUInt16);

    LY_DEF_ROS_TOPIC(ly_game_all, "/ly/game/all", gimbal_driver::msg::GameData);
    LY_DEF_ROS_TOPIC(ly_bullet_speed, "/ly/bullet/speed", std_msgs::msg::Float32);

    LY_DEF_ROS_TOPIC(ly_team_buff, "/ly/team/buff", gimbal_driver::msg::BuffData);
    LY_DEF_ROS_TOPIC(ly_game_rfid, "/ly/game/rfid", gimbal_driver::msg::RfidStatus);
    LY_DEF_ROS_TOPIC(ly_position_data, "/ly/position/data", gimbal_driver::msg::PositionData);
    LY_DEF_ROS_TOPIC(ly_game_sentry_info, "/ly/game/sentry/info", gimbal_driver::msg::SentryInfo);
    LY_DEF_ROS_TOPIC(ly_game_bullet, "/ly/game/bullet", gimbal_driver::msg::BulletInfo);
    LY_DEF_ROS_TOPIC(ly_game_map_command, "/ly/game/map_command", gimbal_driver::msg::MapCommand);
    LY_DEF_ROS_TOPIC(ly_game_damage_difference, "/ly/game/damage_difference", gimbal_driver::msg::StampedInt16);
        

    using namespace std::chrono_literals;
    class Application
    {
    public:
        inline static constexpr const char Name[] = "gimbal_driver";

    private:
        ROSNode<Name> Node;
        std::atomic_bool DeviceError{ false };
        IODevice<TypedMessage<sizeof(GimbalData)>, GimbalControlFrame> Device{};
        MultiCallback<GimbalControlFrame> CallbackGenerator;
        GimbalControlFrame controlShadow_{};
        SentryCmdType sentryCmdShadow_{};
        std::uint8_t postureCommand_{0}; // 0=不控制, 1=进攻, 2=防御, 3=移动, 4~6=强化姿态
        std::uint8_t postureState_{0};   // 0=未知, 1=进攻, 2=防御, 3=移动
        int postureTxRepeatCount_{3};
        std::chrono::milliseconds postureTxInterval_{20};
        bool enhancedPostureGuardEnable_{true};
        std::chrono::milliseconds enhancedPostureInfo3FreshTimeout_{1500};
        std::chrono::milliseconds firecodePartialHold_{100};
        std::chrono::milliseconds navigationTestStaleTimeout_{500};
        std::chrono::milliseconds gamePathFreshTimeout_{5000};
        MapPathRateLimiter mapPathRateLimiter_{std::chrono::seconds{1}};
        std::uint8_t nextMapPathSequence_{0};
        std::chrono::milliseconds gimbalDynamicsTimeout_{200};
        std::chrono::milliseconds gimbalStatePublishPeriod_{20};
        std::chrono::steady_clock::time_point nextGimbalStatePublishTime_{
            std::chrono::steady_clock::time_point::min()
        };
        float sentryCoordX_{0.0f};
        float sentryCoordY_{0.0f};
        bool sentryCoordPending_{false};
        bool sentryCoordValid_{false};
        std::chrono::steady_clock::time_point sentryCoordNextSendTime_{
            std::chrono::steady_clock::time_point::min()
        };
        std::chrono::milliseconds sentryCoordSendInterval_{100};
        std::chrono::steady_clock::time_point sentryCoordLastRxTime_{};
        int sentryCoordFieldWidthX_{2800};
        int sentryCoordFieldWidthY_{1500};
        int sentryCoordFreshTimeoutMs_{2000};
        float velocityRawToMps_{0.025f};
        bool navigationTestEnable_{false};
        bool navigationVelocityForwardEnable_{false};
        bool navigationModeEnable_{false};
        bool navigationModeVelChainEnable_{false};
        std::uint8_t navigationModeRotateLevel_{0};
        bool navigationModeShouldRotateEnable_{false};
        bool navigationModeFollowModeWhenFalse_{false};
        bool navigationModeShouldRotate_{true};
        bool navigationTestVelocityActive_{false};
        std::uint8_t posturePendingToSend_{0};
        std::uint8_t postureLastSent_{0};
        int posturePendingRepeat_{0};
        std::array<std::chrono::steady_clock::time_point, 5> firecodeLastUpdate_{};
        std::chrono::steady_clock::time_point navigationTestLastVelRxTime_{};
        std::chrono::steady_clock::time_point postureNextSendTime_{
            std::chrono::steady_clock::time_point::min()
        };
        std::uint32_t latestRfidStatusRaw_{0};
        bool hasRfidStatusRaw_{false};
        std::uint8_t latestRfidStatus2_{0};
        bool hasRfidStatus2_{false};
        std::uint64_t latestSentryInfo3_{0};
        bool hasSentryInfo3_{false};
        std::uint16_t selfSentryRobotId_{0};
        std::chrono::steady_clock::time_point lastSentryInfo3RxTime_{};
        float latestBulletInitialSpeed_{0.0f};
        bool hasBulletInitialSpeed_{false};
        BulletDataAndRfid2 latestBulletDataAndRfid2_{};
        bool hasBulletDataAndRfid2_{false};
        std::chrono::steady_clock::time_point preciseOutpostHpLastRxTime_{};
        std::chrono::milliseconds preciseOutpostHpFreshTimeout_{1500};
        std::mutex gimbalStateMutex_{};
        float latestGimbalYaw_{0.0F};
        float latestGimbalPitch_{0.0F};
        float latestGimbalYawOmega_{0.0F};
        float latestGimbalPitchOmega_{0.0F};
        float latestGimbalYawAlpha_{0.0F};
        float latestGimbalPitchAlpha_{0.0F};
        bool hasGimbalDynamics_{false};
        std::uint32_t latestGimbalSampleTickMs_{0};
        std::chrono::steady_clock::time_point latestGimbalDynamicsRxTime_{};
        GimbalTrajectoryFrame trajectoryShadow_{};
        bool rawSerialLogEnable_{false};
        bool rawSerialLogUplink_{true};
        bool rawSerialLogDownlink_{true};
        bool rawSerialLogScreen_{false};
        bool rawSerialLogFlush_{true};
        std::string rawSerialLogDir_{"~/Log/GimbalRaw"};
        std::string rawSerialLogTypeIds_{"all"};
        std::array<bool, 256> rawSerialLogTypeIdEnabled_{};
        std::ofstream rawSerialLogFile_{};
        std::mutex rawSerialLogMutex_{};
        bool rawSerialTopicEnable_{false};
        bool rawSerialTopicUplink_{true};
        bool rawSerialTopicDownlink_{true};
        std::string rawSerialTopicTypeIds_{"all"};
        std::array<bool, 256> rawSerialTopicTypeIdEnabled_{};
        bool rawDownlinkTestMode_{false};
        rclcpp::Publisher<gimbal_driver::msg::GimbalRawFrame>::SharedPtr rawSerialRxPublisher_{};
        rclcpp::Publisher<gimbal_driver::msg::GimbalRawFrame>::SharedPtr rawSerialTxPublisher_{};
        static constexpr std::size_t kUploadTypeIdCount = 12;
        static constexpr std::size_t kDownloadTypeIdCount = 6;
        bool serialModeEnable_{false};
        bool serialModeUploadEnable_{true};
        bool serialModeDownloadEnable_{true};
        std::array<bool, kUploadTypeIdCount> serialModeUploadTypeIdEnabled_{};
        std::array<bool, kDownloadTypeIdCount> serialModeDownloadTypeIdEnabled_{};
        std::array<rclcpp::Publisher<gimbal_driver::msg::GimbalRawFrame>::SharedPtr,
            kUploadTypeIdCount> serialModeUploadPublishers_{};
        std::array<rclcpp::Publisher<gimbal_driver::msg::GimbalRawFrame>::SharedPtr,
            kDownloadTypeIdCount> serialModeDownloadPublishers_{};
        std::array<rclcpp::Subscription<gimbal_driver::msg::GimbalRawFrame>::SharedPtr,
            kDownloadTypeIdCount> rawDownlinkTestSubscriptions_{};
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subSentryPosition_{};
        rclcpp::Subscription<gimbal_driver::msg::GimbalTrajectory>::SharedPtr trajectorySubscription_{};
        rclcpp::Publisher<gimbal_driver::msg::GimbalState>::SharedPtr gimbalStatePublisher_{};

        enum FireCodeFieldIndex : std::size_t {
            kFireStatusField = 0,
            kCapStateField,
            kFollowModeField,
            kAimModeField,
            kRotateField,
        };

        static bool IsValidPosture(std::uint8_t posture) noexcept {
            return posture >= 1 && posture <= 3;
        }

        static bool IsValidPostureCommand(std::uint8_t posture) noexcept {
            return posture >= 1 && posture <= 6;
        }

        bool HasFreshSentryInfo3() const noexcept {
            return hasSentryInfo3_ &&
                lastSentryInfo3RxTime_.time_since_epoch().count() != 0 &&
                std::chrono::steady_clock::now() - lastSentryInfo3RxTime_ <=
                    enhancedPostureInfo3FreshTimeout_;
        }

        bool IsPostureCommandAllowed(const std::uint8_t posture) const noexcept {
            return IsEnhancedPostureCommandAllowed(
                posture,
                enhancedPostureGuardEnable_,
                HasFreshSentryInfo3(),
                latestSentryInfo3_);
        }

        void WarnRejectedEnhancedPosture(const char* source, const std::uint8_t posture) const {
            roslog::warn(
                "Reject %s enhanced posture=%u: TypeID10 fresh=%d remaining_s=%u guard=%d",
                source,
                posture,
                HasFreshSentryInfo3() ? 1 : 0,
                static_cast<unsigned int>(EnhancedPostureRemainingSec(posture, latestSentryInfo3_)),
                enhancedPostureGuardEnable_ ? 1 : 0);
        }

        std::uint8_t PostureForTransmission(
            const std::uint8_t posture,
            const char* source) {
            const auto transmitted = PostureCommandForTransmission(
                posture,
                enhancedPostureGuardEnable_,
                HasFreshSentryInfo3(),
                latestSentryInfo3_);
            if (transmitted == posture) {
                return transmitted;
            }

            WarnRejectedEnhancedPosture(source, posture);
            if (posturePendingToSend_ == posture) {
                posturePendingToSend_ = 0;
                posturePendingRepeat_ = 0;
            }
            if (postureCommand_ == posture) {
                postureCommand_ = 0;
            }
            if (sentryCmdShadow_.Posture == posture) {
                sentryCmdShadow_.Posture = 0;
            }
            return transmitted;
        }

        static std::uint8_t ClampU2(std::uint8_t value) noexcept {
            return static_cast<std::uint8_t>(value & 0x03u);
        }

        static std::uint8_t ClampU3(std::uint8_t value) noexcept {
            return static_cast<std::uint8_t>(value & 0x07u);
        }

        static std::uint8_t ClampU8Bits(std::uint8_t value, unsigned width) noexcept {
            const std::uint8_t mask = static_cast<std::uint8_t>((1u << width) - 1u);
            return static_cast<std::uint8_t>(value & mask);
        }

        static std::uint16_t ClampU16Bits(std::uint16_t value, unsigned width) noexcept {
            const std::uint16_t mask = static_cast<std::uint16_t>((1u << width) - 1u);
            return static_cast<std::uint16_t>(value & mask);
        }

        static std::int8_t ClampInt8(int value) noexcept {
            return static_cast<std::int8_t>(std::clamp(value, -128, 127));
        }

        static std::int8_t EncodeNavigationTestVelocityRaw(float value) noexcept {
            if (!std::isfinite(value)) {
                return 0;
            }
            if (value > 127.0f) {
                return 127;
            }
            if (value < -128.0f) {
                return -128;
            }
            return ClampInt8(static_cast<int>(std::lround(value)));
        }

        static bool Bit(const std::uint32_t raw, const unsigned shift) noexcept {
            return ((raw >> shift) & 0x1u) != 0u;
        }

        static std::uint8_t BitsU8(const std::uint32_t raw, const unsigned shift, const unsigned width) noexcept {
            const std::uint32_t mask = (1u << width) - 1u;
            return static_cast<std::uint8_t>((raw >> shift) & mask);
        }

        static std::uint16_t BitsU16(const std::uint32_t raw, const unsigned shift, const unsigned width) noexcept {
            const std::uint32_t mask = (1u << width) - 1u;
            return static_cast<std::uint16_t>((raw >> shift) & mask);
        }

        static std::uint8_t BitsU8FromU64(const std::uint64_t raw, const unsigned shift, const unsigned width) noexcept {
            const std::uint64_t mask = (1ull << width) - 1ull;
            return static_cast<std::uint8_t>((raw >> shift) & mask);
        }

        static std::uint8_t FireCodeRaw(const FireCodeType& firecode) noexcept {
            return *reinterpret_cast<const std::uint8_t*>(&firecode);
        }

        static FireCodeType FireCodeFromRaw(std::uint8_t raw) noexcept {
            FireCodeType firecode{};
            *reinterpret_cast<std::uint8_t*>(&firecode) = raw;
            return firecode;
        }

        static gimbal_driver::msg::EventData ToEventDataMsg(std::uint32_t raw) {
            gimbal_driver::msg::EventData msg;
            msg.raw = raw;
            msg.self_supply_status = BitsU8(raw, 0, 3);
            msg.self_supply_occupied = Bit(raw, 0);
            msg.self_supply_reserved = Bit(raw, 1);
            msg.self_rmul_supply_occupied = Bit(raw, 2);
            msg.self_small_energy_status = BitsU8(raw, 3, 2);
            msg.self_large_energy_status = BitsU8(raw, 5, 2);
            msg.self_central_highland_status = BitsU8(raw, 7, 2);
            msg.self_trapezoid_highland_status = BitsU8(raw, 9, 2);
            msg.enemy_last_dart_hit_time = BitsU16(raw, 11, 9);
            msg.enemy_last_dart_hit_target = BitsU8(raw, 20, 3);
            msg.center_gain_point_status = BitsU8(raw, 23, 2);
            msg.self_fortress_gain_point_status = BitsU8(raw, 25, 2);
            msg.self_outpost_gain_point_status = BitsU8(raw, 27, 2);
            msg.self_base_gain_point_status = Bit(raw, 29);
            msg.reserved = BitsU8(raw, 30, 2);
            return msg;
        }

        static const char* TypeIdName(std::uint8_t type_id) noexcept {
            switch (type_id) {
                case GimbalData::TypeID: return "GimbalData";
                case GameData::TypeID: return "GameData";
                case HealthMyselfData::TypeID: return "HealthMyselfData";
                case HealthEnemyData::TypeID: return "HealthEnemyData";
                case RFIDAndBuffData::TypeID: return "RFIDAndBuffData";
                case PositionData::TypeID: return "PositionData";
                case ChassisData::TypeID: return "ChassisData";
                case SentryData::TypeID: return "SentryData";
                case BulletDataAndRfid2::TypeID: return "BulletDataAndRfid2";
                case MapCommandData::TypeID: return "MapCommandData";
                case SentryInfo3AndOutpostHpData::TypeID: return "SentryInfo3AndOutpostHpData";
                case GimbalDynamicsData::TypeID: return "GimbalDynamicsData";
                default: return "Unknown";
            }
        }

        static std::string ExpandUserPath(std::string path) {
            if (path == "~") {
                if (const char* home = std::getenv("HOME")) {
                    return std::string{home};
                }
            } else if (path.rfind("~/", 0) == 0) {
                if (const char* home = std::getenv("HOME")) {
                    return std::string{home} + path.substr(1);
                }
            }
            return path;
        }

        static std::string TimestampForFileName() {
            const auto now = std::chrono::system_clock::now();
            const std::time_t time = std::chrono::system_clock::to_time_t(now);
            std::tm local_time{};
            if (const auto* tm_ptr = std::localtime(&time)) {
                local_time = *tm_ptr;
            }
            std::ostringstream oss;
            oss << std::put_time(&local_time, "%Y%m%d_%H%M%S");
            return oss.str();
        }

        static std::uint64_t WallTimeNs() {
            return static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
        }

        template<typename T>
        static std::string BytesToHex(const T& item) {
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(&item);
            std::ostringstream oss;
            oss << std::hex << std::setfill('0');
            for (std::size_t i = 0; i < sizeof(T); ++i) {
                if (i != 0) {
                    oss << ' ';
                }
                oss << std::setw(2) << static_cast<unsigned>(bytes[i]);
            }
            return oss.str();
        }

        static void ParseRawTypeIdFilter(
            const std::string& type_ids,
            std::array<bool, 256>& enabled,
            const char* label) {
            enabled.fill(false);
            auto normalized = type_ids.empty() ? std::string{"all"} : type_ids;
            std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
                return static_cast<char>(std::tolower(ch));
            });
            if (normalized == "all" || normalized == "*") {
                enabled.fill(true);
                return;
            }

            std::stringstream ss{normalized};
            std::string token;
            while (std::getline(ss, token, ',')) {
                token.erase(
                    std::remove_if(token.begin(), token.end(), [](unsigned char ch) {
                        return std::isspace(ch) != 0;
                    }),
                    token.end());
                if (token.empty()) {
                    continue;
                }
                try {
                    const auto value = std::stoi(token);
                    if (value >= 0 && value <= 255) {
                        enabled[static_cast<std::size_t>(value)] = true;
                    }
                } catch (const std::exception& ex) {
                    roslog::warn("Invalid %s type id token '%s': %s",
                                 label,
                                 token.c_str(),
                                 ex.what());
                }
            }
        }

        template<typename T>
        static void AssignRawBytes(gimbal_driver::msg::GimbalRawFrame& msg, const T& item) {
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(&item);
            msg.data.assign(bytes, bytes + sizeof(T));
        }

        void ConfigureRawSerialLog(
            bool enable,
            bool uplink,
            bool downlink,
            bool screen,
            bool flush,
            const std::string& dir,
            const std::string& type_ids) {
            std::lock_guard lock{rawSerialLogMutex_};
            rawSerialLogEnable_ = enable;
            rawSerialLogUplink_ = uplink;
            rawSerialLogDownlink_ = downlink;
            rawSerialLogScreen_ = screen;
            rawSerialLogFlush_ = flush;
            rawSerialLogDir_ = dir.empty() ? std::string{"~/Log/GimbalRaw"} : dir;
            rawSerialLogTypeIds_ = type_ids.empty() ? std::string{"all"} : type_ids;
            ParseRawTypeIdFilter(rawSerialLogTypeIds_, rawSerialLogTypeIdEnabled_, "gimbal raw file log");

            if (!rawSerialLogEnable_) {
                rawSerialLogFile_.close();
                return;
            }

            const auto expanded_dir = ExpandUserPath(rawSerialLogDir_);
            std::error_code ec;
            std::filesystem::create_directories(expanded_dir, ec);
            if (ec) {
                roslog::error("Cannot create gimbal raw log dir '%s': %s",
                              expanded_dir.c_str(),
                              ec.message().c_str());
                rawSerialLogEnable_ = false;
                return;
            }

            const auto log_path = std::filesystem::path(expanded_dir) /
                ("gimbal_raw_" + TimestampForFileName() + ".log");
            rawSerialLogFile_.open(log_path, std::ios::out | std::ios::app);
            if (!rawSerialLogFile_) {
                roslog::error("Cannot open gimbal raw log file '%s'", log_path.string().c_str());
                rawSerialLogEnable_ = false;
                return;
            }
            rawSerialLogFile_
                << "# gimbal_driver raw serial log\n"
                << "# time_ns direction type size hex extra\n";
            if (rawSerialLogFlush_) {
                rawSerialLogFile_.flush();
            }
            roslog::warn("gimbal raw serial log enabled: file=%s uplink=%s downlink=%s type_ids=%s",
                         log_path.string().c_str(),
                         rawSerialLogUplink_ ? "true" : "false",
                         rawSerialLogDownlink_ ? "true" : "false",
                         rawSerialLogTypeIds_.c_str());
        }

        void ConfigureRawSerialTopic(bool enable, bool uplink, bool downlink, const std::string& type_ids) {
            rawSerialTopicEnable_ = enable;
            rawSerialTopicUplink_ = uplink;
            rawSerialTopicDownlink_ = downlink;
            rawSerialTopicTypeIds_ = type_ids.empty() ? std::string{"all"} : type_ids;
            ParseRawTypeIdFilter(rawSerialTopicTypeIds_, rawSerialTopicTypeIdEnabled_, "gimbal raw topic log");

            if (!rawSerialTopicEnable_) {
                rawSerialRxPublisher_.reset();
                rawSerialTxPublisher_.reset();
                return;
            }

            auto node = Node.GetNode();
            if (rawSerialTopicUplink_) {
                rawSerialRxPublisher_ =
                    node->create_publisher<gimbal_driver::msg::GimbalRawFrame>(
                        "/ly/log/gimbal_raw_rx",
                        rclcpp::SensorDataQoS());
            }
            if (rawSerialTopicDownlink_) {
                rawSerialTxPublisher_ =
                    node->create_publisher<gimbal_driver::msg::GimbalRawFrame>(
                        "/ly/log/gimbal_raw_tx",
                        rclcpp::SensorDataQoS());
            }
        }

        void ConfigureSerialModeRawTopics(
            const bool enable,
            const bool upload_enable,
            const std::array<bool, kUploadTypeIdCount>& upload_type_ids,
            const bool download_enable,
            const std::array<bool, kDownloadTypeIdCount>& download_type_ids) {
            serialModeEnable_ = enable;
            serialModeUploadEnable_ = upload_enable;
            serialModeDownloadEnable_ = download_enable;
            serialModeUploadTypeIdEnabled_ = upload_type_ids;
            serialModeDownloadTypeIdEnabled_ = download_type_ids;
            serialModeUploadPublishers_.fill(nullptr);
            serialModeDownloadPublishers_.fill(nullptr);

            if (!serialModeEnable_) {
                return;
            }

            auto node = Node.GetNode();
            if (serialModeUploadEnable_) {
                for (std::size_t type_id = 0; type_id < kUploadTypeIdCount; ++type_id) {
                    if (!serialModeUploadTypeIdEnabled_[type_id]) {
                        continue;
                    }
                    serialModeUploadPublishers_[type_id] =
                        node->create_publisher<gimbal_driver::msg::GimbalRawFrame>(
                            "/ly/upload/typeid" + std::to_string(type_id),
                            rclcpp::SensorDataQoS());
                }
            }
            if (serialModeDownloadEnable_) {
                for (std::size_t type_id = 0; type_id < kDownloadTypeIdCount; ++type_id) {
                    if (!serialModeDownloadTypeIdEnabled_[type_id]) {
                        continue;
                    }
                    std::ostringstream topic;
                    topic << "/ly/download/typeid0x" << std::hex << std::setw(2)
                          << std::setfill('0') << type_id;
                    serialModeDownloadPublishers_[type_id] =
                        node->create_publisher<gimbal_driver::msg::GimbalRawFrame>(
                            topic.str(), rclcpp::SensorDataQoS());
                }
            }
        }

        template<typename T>
        void WriteRawSerialLogLine(
            const char* direction,
            const char* type,
            const T& item,
            const std::string& extra = {}) {
            try {
                std::lock_guard lock{rawSerialLogMutex_};
                if (!rawSerialLogEnable_) {
                    return;
                }

                std::ostringstream line;
                line << WallTimeNs()
                     << " " << direction
                     << " " << type
                     << " size=" << sizeof(T)
                     << " hex=\"" << BytesToHex(item) << "\"";
                if (!extra.empty()) {
                    line << " " << extra;
                }

                if (rawSerialLogFile_) {
                    rawSerialLogFile_ << line.str() << '\n';
                    if (rawSerialLogFlush_) {
                        rawSerialLogFile_.flush();
                    }
                    if (!rawSerialLogFile_) {
                        rawSerialLogEnable_ = false;
                        rawSerialLogFile_.close();
                        roslog::error("gimbal raw serial log disabled after a write failure");
                    }
                }
                if (rawSerialLogScreen_) {
                    roslog::info("%s", line.str().c_str());
                }
            } catch (const std::exception& ex) {
                std::lock_guard lock{rawSerialLogMutex_};
                rawSerialLogEnable_ = false;
                rawSerialLogFile_.close();
                roslog::error("gimbal raw serial log disabled after exception: %s", ex.what());
            } catch (...) {
                std::lock_guard lock{rawSerialLogMutex_};
                rawSerialLogEnable_ = false;
                rawSerialLogFile_.close();
                roslog::error("gimbal raw serial log disabled after an unknown exception");
            }
        }

        void PublishRawRxTopic(const TypedMessage<sizeof(GimbalData)>& message) {
            if (!rawSerialRxPublisher_ || rawSerialRxPublisher_->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_RX;
            msg.type_id = message.TypeID;
            AssignRawBytes(msg, message);
            rawSerialRxPublisher_->publish(msg);
        }

        void PublishSerialModeUploadTopic(const TypedMessage<sizeof(GimbalData)>& message) {
            const auto type_id = static_cast<std::size_t>(message.TypeID);
            if (!serialModeEnable_ || !serialModeUploadEnable_ || type_id >= kUploadTypeIdCount) {
                return;
            }
            const auto& publisher = serialModeUploadPublishers_[type_id];
            if (!publisher || publisher->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_RX;
            msg.type_id = message.TypeID;
            AssignRawBytes(msg, message);
            publisher->publish(msg);
        }

        template<typename T>
        void PublishSerialModeDownloadTopic(const T& data, const std::uint8_t downlink_type_id) {
            const auto type_id = static_cast<std::size_t>(downlink_type_id);
            if (!serialModeEnable_ || !serialModeDownloadEnable_ || type_id >= kDownloadTypeIdCount) {
                return;
            }
            const auto& publisher = serialModeDownloadPublishers_[type_id];
            if (!publisher || publisher->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_TX;
            msg.type_id = downlink_type_id;
            AssignRawBytes(msg, data);
            if constexpr (std::is_same_v<T, GimbalControlFrame>) {
                msg.firecode_raw = FireCodeRaw(data.FireCode);
            } else if constexpr (std::is_same_v<T, SentryCommandFrame>) {
                msg.sentry_cmd_raw = std::bit_cast<std::uint32_t>(data.SentryCmd);
            }
            publisher->publish(msg);
        }

        void PublishRawTxTopic(const GimbalControlFrame& data) {
            if (!rawSerialTxPublisher_ || rawSerialTxPublisher_->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_TX;
            msg.type_id = gimbal_driver::msg::GimbalRawFrame::TYPE_ID_TX_CONTROL;
            AssignRawBytes(msg, data);
            msg.firecode_raw = FireCodeRaw(data.FireCode);
            rawSerialTxPublisher_->publish(msg);
        }

        void PublishRawTxTopic(const SentryCommandFrame& data) {
            if (!rawSerialTxPublisher_ || rawSerialTxPublisher_->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_TX;
            msg.type_id = gimbal_driver::msg::GimbalRawFrame::TYPE_ID_TX_SENTRY_COMMAND;
            AssignRawBytes(msg, data);
            msg.sentry_cmd_raw = std::bit_cast<std::uint32_t>(data.SentryCmd);
            rawSerialTxPublisher_->publish(msg);
        }

        void PublishRawTxTopic(const MapPathFragmentFrame& data) {
            if (!rawSerialTxPublisher_ || rawSerialTxPublisher_->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_TX;
            msg.type_id = gimbal_driver::msg::GimbalRawFrame::TYPE_ID_TX_MAP_PATH;
            AssignRawBytes(msg, data);
            rawSerialTxPublisher_->publish(msg);
        }

        void PublishRawTxTopic(const CustomInfoFrame& data) {
            if (!rawSerialTxPublisher_ || rawSerialTxPublisher_->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_TX;
            msg.type_id = gimbal_driver::msg::GimbalRawFrame::TYPE_ID_TX_CUSTOM_INFO;
            AssignRawBytes(msg, data);
            rawSerialTxPublisher_->publish(msg);
        }

        void PublishRawTxTopic(const SentryCoordinateFrame& data) {
            if (!rawSerialTxPublisher_ || rawSerialTxPublisher_->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_TX;
            msg.type_id = gimbal_driver::msg::GimbalRawFrame::TYPE_ID_TX_SENTRY_COORDINATE;
            AssignRawBytes(msg, data);
            rawSerialTxPublisher_->publish(msg);
        }

        void PublishRawTxTopic(const GimbalTrajectoryFrame& data) {
            if (!rawSerialTxPublisher_ || rawSerialTxPublisher_->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_TX;
            msg.type_id = gimbal_driver::msg::GimbalRawFrame::TYPE_ID_TX_TRAJECTORY;
            AssignRawBytes(msg, data);
            rawSerialTxPublisher_->publish(msg);
        }

        void LogUplinkRaw(const TypedMessage<sizeof(GimbalData)>& message) {
            if (rawSerialLogEnable_ &&
                rawSerialLogUplink_ &&
                rawSerialLogTypeIdEnabled_[static_cast<std::size_t>(message.TypeID)]) {
                std::ostringstream extra;
                extra << "name=" << TypeIdName(message.TypeID);
                const auto type_id = std::to_string(message.TypeID);
                WriteRawSerialLogLine("rx", type_id.c_str(), message, extra.str());
            }

            if (rawSerialTopicEnable_ &&
                rawSerialTopicUplink_ &&
                rawSerialTopicTypeIdEnabled_[static_cast<std::size_t>(message.TypeID)]) {
                PublishRawRxTopic(message);
            }
            PublishSerialModeUploadTopic(message);
        }

        void LogDownlinkRaw(const GimbalControlFrame& data, const char* reason) {
            if (rawSerialLogEnable_ && rawSerialLogDownlink_) {
                std::ostringstream extra;
                extra << "reason=" << (reason ? reason : "control")
                      << " firecode_raw=" << static_cast<unsigned>(FireCodeRaw(data.FireCode));
                WriteRawSerialLogLine("tx", "control", data, extra.str());
            }

            if (rawSerialTopicEnable_ && rawSerialTopicDownlink_) {
                PublishRawTxTopic(data);
            }
            PublishSerialModeDownloadTopic(data, GimbalControlFrame::DownlinkTypeIDValue);
        }

        void LogDownlinkRaw(const SentryCommandFrame& data, const char* reason) {
            if (rawSerialLogEnable_ && rawSerialLogDownlink_) {
                std::ostringstream extra;
                extra << "reason=" << (reason ? reason : "sentry_command")
                      << " sentry_cmd_raw=" << std::bit_cast<std::uint32_t>(data.SentryCmd);
                WriteRawSerialLogLine("tx", "sentry_command", data, extra.str());
            }

            if (rawSerialTopicEnable_ && rawSerialTopicDownlink_) {
                PublishRawTxTopic(data);
            }
            PublishSerialModeDownloadTopic(data, SentryCommandFrame::DownlinkTypeIDValue);
        }

        void LogDownlinkRaw(const MapPathFragmentFrame& data, const char* reason) {
            if (rawSerialLogEnable_ && rawSerialLogDownlink_) {
                std::ostringstream extra;
                extra << "reason=" << (reason ? reason : "map_path_fragment")
                      << " sequence=" << static_cast<unsigned>(data.Sequence)
                      << " fragment=" << static_cast<unsigned>(data.FragmentIndex)
                      << "/" << static_cast<unsigned>(data.FragmentCount)
                      << " payload_length=" << static_cast<unsigned>(data.PayloadLength);
                WriteRawSerialLogLine("tx", "map_path_fragment", data, extra.str());
            }

            if (rawSerialTopicEnable_ && rawSerialTopicDownlink_) {
                PublishRawTxTopic(data);
            }
            PublishSerialModeDownloadTopic(data, MapPathFragmentFrame::DownlinkTypeIDValue);
        }

        void LogDownlinkRaw(const CustomInfoFrame& data, const char* reason) {
            if (rawSerialLogEnable_ && rawSerialLogDownlink_) {
                std::ostringstream extra;
                extra << "reason=" << (reason ? reason : "custom_info")
                      << " sender_id=" << data.SenderId
                      << " receiver_id=" << data.ReceiverId;
                WriteRawSerialLogLine("tx", "custom_info", data, extra.str());
            }

            if (rawSerialTopicEnable_ && rawSerialTopicDownlink_) {
                PublishRawTxTopic(data);
            }
            PublishSerialModeDownloadTopic(data, CustomInfoFrame::DownlinkTypeIDValue);
        }

        void LogDownlinkRaw(const SentryCoordinateFrame& data, const char* reason) {
            if (rawSerialLogEnable_ && rawSerialLogDownlink_) {
                std::ostringstream extra;
                extra << "reason=" << (reason ? reason : "sentry_coordinate")
                      << " downlink_type_id="
                      << static_cast<unsigned>(data.DownlinkTypeID)
                      << " x_cm=" << static_cast<int>(data.X_cm)
                      << " y_cm=" << static_cast<int>(data.Y_cm)
                      << " crc8=" << static_cast<unsigned>(data.CRC8);
                WriteRawSerialLogLine("tx", "sentry_coordinate", data, extra.str());
            }

            if (rawSerialTopicEnable_ && rawSerialTopicDownlink_) {
                PublishRawTxTopic(data);
            }
            PublishSerialModeDownloadTopic(data, SentryCoordinateFrame::DownlinkTypeIDValue);
        }

        void LogDownlinkRaw(const GimbalTrajectoryFrame& data, const char* reason) {
            if (rawSerialLogEnable_ && rawSerialLogDownlink_) {
                std::ostringstream extra;
                extra << "reason=" << (reason ? reason : "trajectory")
                      << " yaw=" << data.Yaw
                      << " pitch=" << data.Pitch
                      << " yaw_omega=" << data.YawOmega
                      << " pitch_omega=" << data.PitchOmega
                      << " yaw_alpha=" << data.YawAlpha
                      << " pitch_alpha=" << data.PitchAlpha;
                WriteRawSerialLogLine("tx", "trajectory", data, extra.str());
            }

            if (rawSerialTopicEnable_ && rawSerialTopicDownlink_) {
                PublishRawTxTopic(data);
            }
            PublishSerialModeDownloadTopic(data, GimbalTrajectoryFrame::DownlinkTypeIDValue);
        }

        template<typename TFrame>
        void SendRawDownlinkTestFrame(
            const std::uint8_t expected_type_id,
            const gimbal_driver::msg::GimbalRawFrame::ConstSharedPtr& msg) {
            if (!rawDownlinkTestMode_ || !msg) {
                return;
            }
            if (msg->direction != gimbal_driver::msg::GimbalRawFrame::DIRECTION_TX ||
                msg->type_id != expected_type_id ||
                !IsValidRawDownlinkTestFrame(
                    expected_type_id,
                    std::span<const std::uint8_t>{msg->data.data(), msg->data.size()})) {
                roslog::warn(
                    "Drop raw downlink test frame: expected=0x%02x direction=%u type_id=0x%02x size=%zu",
                    static_cast<unsigned>(expected_type_id),
                    static_cast<unsigned>(msg->direction),
                    static_cast<unsigned>(msg->type_id),
                    msg->data.size());
                return;
            }
            if (DeviceError) {
                return;
            }

            TFrame frame{};
            std::memcpy(&frame, msg->data.data(), sizeof(frame));
            if (!Device.WriteRaw(frame)) {
                DeviceError = true;
                return;
            }
            LogDownlinkRaw(frame, "raw_downlink_test");
        }

        void GenRawDownlinkTestSubscriptions() {
            auto node = Node.GetNode();
            for (std::size_t type_id = 0; type_id < kDownloadTypeIdCount; ++type_id) {
                std::ostringstream topic;
                topic << "/ly/download/typeid0x" << std::hex << std::setw(2)
                      << std::setfill('0') << type_id;
                rawDownlinkTestSubscriptions_[type_id] =
                    node->create_subscription<gimbal_driver::msg::GimbalRawFrame>(
                        topic.str(),
                        rclcpp::SensorDataQoS(),
                        [this, type_id](const gimbal_driver::msg::GimbalRawFrame::ConstSharedPtr msg) {
                            switch (type_id) {
                                case GimbalControlFrame::DownlinkTypeIDValue:
                                    SendRawDownlinkTestFrame<GimbalControlFrame>(
                                        GimbalControlFrame::DownlinkTypeIDValue, msg);
                                    break;
                                case SentryCommandFrame::DownlinkTypeIDValue:
                                    SendRawDownlinkTestFrame<SentryCommandFrame>(
                                        SentryCommandFrame::DownlinkTypeIDValue, msg);
                                    break;
                                case MapPathFragmentFrame::DownlinkTypeIDValue:
                                    SendRawDownlinkTestFrame<MapPathFragmentFrame>(
                                        MapPathFragmentFrame::DownlinkTypeIDValue, msg);
                                    break;
                                case CustomInfoFrame::DownlinkTypeIDValue:
                                    SendRawDownlinkTestFrame<CustomInfoFrame>(
                                        CustomInfoFrame::DownlinkTypeIDValue, msg);
                                    break;
                                case SentryCoordinateFrame::DownlinkTypeIDValue:
                                    SendRawDownlinkTestFrame<SentryCoordinateFrame>(
                                        SentryCoordinateFrame::DownlinkTypeIDValue, msg);
                                    break;
                                case GimbalTrajectoryFrame::DownlinkTypeIDValue:
                                    SendRawDownlinkTestFrame<GimbalTrajectoryFrame>(
                                        GimbalTrajectoryFrame::DownlinkTypeIDValue, msg);
                                    break;
                                default:
                                    break;
                            }
                        });
            }
        }

        static gimbal_driver::msg::RfidStatus ToRfidStatusMsg(
            std::uint32_t raw,
            bool has_status2 = false,
            std::uint8_t raw_status2 = 0
        ) {
            gimbal_driver::msg::RfidStatus msg;
            msg.raw = raw;
            msg.friend_base = Bit(raw, 0);
            msg.friend_central = Bit(raw, 1);
            msg.enemy_central = Bit(raw, 2);
            msg.friend_highland = Bit(raw, 3);
            msg.enemy_highland = Bit(raw, 4);
            msg.friend_flyroad_front = Bit(raw, 5);
            msg.friend_flyroad_back = Bit(raw, 6);
            msg.enemy_flyroad_front = Bit(raw, 7);
            msg.enemy_flyroad_back = Bit(raw, 8);
            msg.friend_central_under = Bit(raw, 9);
            msg.friend_central_high = Bit(raw, 10);
            msg.enemy_central_under = Bit(raw, 11);
            msg.enemy_central_high = Bit(raw, 12);
            msg.friend_roadland_under = Bit(raw, 13);
            msg.friend_roadland_high = Bit(raw, 14);
            msg.enemy_roadland_under = Bit(raw, 15);
            msg.enemy_roadland_high = Bit(raw, 16);
            msg.friend_bastion = Bit(raw, 17);
            msg.friend_outpost = Bit(raw, 18);
            msg.friend_supply_noremix = Bit(raw, 19);
            msg.friend_supply_remix = Bit(raw, 20);
            msg.friend_armor = Bit(raw, 21);
            msg.enemy_armor = Bit(raw, 22);
            msg.central_rmul = Bit(raw, 23);
            msg.enemy_bastion = Bit(raw, 24);
            msg.enemy_outpost = Bit(raw, 25);
            msg.friend_tunnel_roadland_down = Bit(raw, 26);
            msg.friend_tunnel_roadland_mid = Bit(raw, 27);
            msg.friend_tunnel_roadland_up = Bit(raw, 28);
            msg.friend_tunnel_highland_low = Bit(raw, 29);
            msg.friend_tunnel_highland_mid = Bit(raw, 30);
            msg.friend_tunnel_highland_high = Bit(raw, 31);
            msg.has_rfid_status_2 = has_status2;
            msg.rfid_status_2_raw = has_status2 ? raw_status2 : 0;
            msg.enemy_tunnel_roadland_down = has_status2 && Bit(raw_status2, 0);
            msg.enemy_tunnel_roadland_mid = has_status2 && Bit(raw_status2, 1);
            msg.enemy_tunnel_roadland_up = has_status2 && Bit(raw_status2, 2);
            msg.enemy_tunnel_highland_low = has_status2 && Bit(raw_status2, 3);
            msg.enemy_tunnel_highland_mid = has_status2 && Bit(raw_status2, 4);
            msg.enemy_tunnel_highland_high = has_status2 && Bit(raw_status2, 5);
            msg.rfid_status_2_reserved = has_status2 ? BitsU8(raw_status2, 6, 2) : 0;
            return msg;
        }

        static gimbal_driver::msg::SentryInfo ToSentryInfoMsg(
            const SentryData& data,
            const bool has_sentry_info_3,
            const std::uint64_t sentry_info_3,
            const std::uint32_t sentry_info_3_age_ms) {
            gimbal_driver::msg::SentryInfo msg;
            msg.sentry_info_raw = data.SentryInfo;
            msg.sentry_info_2_raw = data.SentryInfo2;
            msg.sentry_info_3_raw = has_sentry_info_3 ? sentry_info_3 : 0;
            msg.reserved = data.Reserved;

            msg.exchanged_projectile_allowance = BitsU16(data.SentryInfo, 0, 11);
            msg.remote_projectile_exchange_count = BitsU8(data.SentryInfo, 11, 4);
            msg.remote_hp_exchange_count = BitsU8(data.SentryInfo, 15, 4);
            msg.can_confirm_free_revive = Bit(data.SentryInfo, 19);
            msg.can_exchange_immediate_revive = Bit(data.SentryInfo, 20);
            msg.immediate_revive_cost = BitsU16(data.SentryInfo, 21, 10);
            msg.sentry_info_reserved = Bit(data.SentryInfo, 31);

            msg.out_of_combat = Bit(data.SentryInfo2, 0);
            msg.remaining_exchangeable_17mm = BitsU16(data.SentryInfo2, 1, 11);
            msg.posture = BitsU8(data.SentryInfo2, 12, 2);
            msg.can_activate_energy_mechanism = Bit(data.SentryInfo2, 14);
            msg.enhanced_posture = Bit(data.SentryInfo2, 15);
            msg.sentry_info_2_reserved = msg.enhanced_posture;

            msg.has_sentry_info_3 = has_sentry_info_3;
            msg.sentry_info_3_age_ms = sentry_info_3_age_ms;
            if (has_sentry_info_3) {
                msg.attack_posture_remaining_s = BitsU8FromU64(sentry_info_3, 0, 8);
                msg.defense_posture_remaining_s = BitsU8FromU64(sentry_info_3, 8, 8);
                msg.move_posture_remaining_s = BitsU8FromU64(sentry_info_3, 16, 8);
                msg.sentry_info_3_reserved_low = BitsU8FromU64(sentry_info_3, 24, 8);
                msg.enhanced_attack_posture_remaining_s = BitsU8FromU64(sentry_info_3, 32, 8);
                msg.enhanced_defense_posture_remaining_s = BitsU8FromU64(sentry_info_3, 40, 8);
                msg.enhanced_move_posture_remaining_s = BitsU8FromU64(sentry_info_3, 48, 8);
                msg.sentry_info_3_reserved_high = BitsU8FromU64(sentry_info_3, 56, 8);
            }
            return msg;
        }

        static gimbal_driver::msg::FireCode ToFireCodeMsg(const FireCodeType& firecode) {
            gimbal_driver::msg::FireCode msg;
            msg.field_mask = gimbal_driver::msg::FireCode::FIELD_ALL;
            msg.fire_status = firecode.FireStatus;
            msg.cap_state = firecode.CapState;
            msg.follow_mode = firecode.FollowMode != 0;
            msg.aim_mode = firecode.AimMode != 0;
            msg.rotate = firecode.Rotate;
            msg.raw = FireCodeRaw(firecode);
            return msg;
        }

        std::int8_t EncodeVelocityRaw(float meters_per_second) const noexcept {
            if (velocityRawToMps_ <= 0.0f) {
                return 0;
            }
            return ClampInt8(static_cast<int>(std::lround(meters_per_second / velocityRawToMps_)));
        }

        float DecodeVelocityRaw(std::int8_t raw) const noexcept {
            return static_cast<float>(raw) * velocityRawToMps_;
        }

        static std::int16_t DecodeI16(std::uint16_t raw) noexcept {
            return std::bit_cast<std::int16_t>(raw);
        }

        static float DecodeI16WithScale(std::uint16_t raw, float divisor) noexcept {
            return static_cast<float>(DecodeI16(raw)) / divisor;
        }

        static std::int16_t EncodeCoordinateCm(float value, int max_cm) noexcept {
            if (!std::isfinite(value)) {
                return 0;
            }
            const auto upper = static_cast<float>(std::max(0, max_cm));
            const auto clamped = std::clamp(value, 0.0f, upper);
            return static_cast<std::int16_t>(std::lround(clamped));
        }

        void ArmPostureTx(std::uint8_t posture) {
            if (!IsValidPostureCommand(posture)) {
                return;
            }
            if (posture == postureLastSent_ && posturePendingRepeat_ == 0) {
                return;
            }
            posturePendingToSend_ = posture;
            posturePendingRepeat_ = std::max(1, postureTxRepeatCount_);
            postureNextSendTime_ = std::chrono::steady_clock::now();
            roslog::info("Posture TX armed: cmd=%u, repeat=%d",
                         posturePendingToSend_, posturePendingRepeat_);
        }

        void MaybeSendPostureTx() {
            if (rawDownlinkTestMode_) {
                return;
            }
            if (posturePendingRepeat_ <= 0) {
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now < postureNextSendTime_) {
                return;
            }

            const auto pending_posture = posturePendingToSend_;
            SentryCommandFrame frame;
            frame.SentryCmd = sentryCmdShadow_;
            frame.SentryCmd.Posture = PostureForTransmission(
                pending_posture, "posture_repeat");
            if (!Device.WriteRaw(frame)) {
                DeviceError = true;
                return;
            }
            LogDownlinkRaw(frame, "posture_repeat");
            sentryCmdShadow_ = frame.SentryCmd;

            if (frame.SentryCmd.Posture != pending_posture) {
                posturePendingRepeat_ = 0;
                postureLastSent_ = 0;
                return;
            }

            posturePendingRepeat_--;
            postureNextSendTime_ = now + postureTxInterval_;
            if (posturePendingRepeat_ == 0) {
                postureLastSent_ = posturePendingToSend_;
            }
        }

        void PublishPosture(std::uint8_t posture) {
            using topic = ly_gimbal_posture;
            topic::Msg msg;
            msg.data = posture;
            Node.Publisher<topic>()->publish(msg);
        }

        bool HasFreshPreciseOutpostHp() const noexcept {
            if (preciseOutpostHpLastRxTime_.time_since_epoch().count() == 0) {
                return false;
            }
            return std::chrono::steady_clock::now() - preciseOutpostHpLastRxTime_ <=
                   preciseOutpostHpFreshTimeout_;
        }

        void PublishOutpostHp(
            std::uint16_t self_hp,
            std::uint16_t enemy_hp,
            const rclcpp::Time& stamp) {
            {
                using topic = ly_enemy_op_hp;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.data = enemy_hp;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_op_hp;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.data = self_hp;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void SendGimbalTrajectory(const gimbal_driver::msg::GimbalTrajectory& msg) {
            if (rawDownlinkTestMode_) {
                return;
            }
            if (!IsFiniteGimbalTrajectory(
                    msg.yaw, msg.pitch, msg.yaw_omega, msg.pitch_omega,
                    msg.yaw_alpha, msg.pitch_alpha)) {
                roslog::warn("Drop /ly/control/trajectory with non-finite value");
                return;
            }
            if (DeviceError) {
                return;
            }
            trajectoryShadow_ = ToGimbalTrajectoryFrame(
                msg.yaw, msg.pitch, msg.yaw_omega, msg.pitch_omega,
                msg.yaw_alpha, msg.pitch_alpha);
            if (!Device.WriteRaw(trajectoryShadow_)) {
                DeviceError = true;
                return;
            }
            LogDownlinkRaw(trajectoryShadow_, "trajectory_callback");
        }

        void DegradeStaleFireCode(FireCodeType& firecode, const std::chrono::steady_clock::time_point now) const {
            auto stale = [&](FireCodeFieldIndex field) {
                const auto stamp = firecodeLastUpdate_[static_cast<std::size_t>(field)];
                return stamp.time_since_epoch().count() == 0 ||
                       (now - stamp) > firecodePartialHold_;
            };

            if (stale(kFireStatusField)) firecode.FireStatus = 0;
            if (stale(kCapStateField)) firecode.CapState = 0;
            if (stale(kFollowModeField)) firecode.FollowMode = 0;
            if (stale(kAimModeField)) firecode.AimMode = 0;
            if (stale(kRotateField)) firecode.Rotate = 0;
        }

        void ApplyFireCodeCommand(GimbalControlFrame& g, const gimbal_driver::msg::FireCode& m) {
            const auto now = std::chrono::steady_clock::now();
            const bool full_snapshot = (m.field_mask == 0) ||
                ((m.field_mask & gimbal_driver::msg::FireCode::FIELD_ALL) == gimbal_driver::msg::FireCode::FIELD_ALL);

            auto mark = [&](FireCodeFieldIndex field) {
                firecodeLastUpdate_[static_cast<std::size_t>(field)] = now;
            };

            if (full_snapshot || (m.field_mask & gimbal_driver::msg::FireCode::FIELD_FIRE_STATUS)) {
                g.FireCode.FireStatus = ClampU2(m.fire_status);
                mark(kFireStatusField);
            }
            if (full_snapshot || (m.field_mask & gimbal_driver::msg::FireCode::FIELD_CAP_STATE)) {
                g.FireCode.CapState = ClampU2(m.cap_state);
                mark(kCapStateField);
            }
            if (full_snapshot || (m.field_mask & gimbal_driver::msg::FireCode::FIELD_FOLLOW_MODE)) {
                g.FireCode.FollowMode = m.follow_mode ? 1 : 0;
                mark(kFollowModeField);
            }
            if (full_snapshot || (m.field_mask & gimbal_driver::msg::FireCode::FIELD_AIM_MODE)) {
                g.FireCode.AimMode = m.aim_mode ? 1 : 0;
                mark(kAimModeField);
            }
            if (full_snapshot || (m.field_mask & gimbal_driver::msg::FireCode::FIELD_ROTATE)) {
                g.FireCode.Rotate = ClampU2(m.rotate);
                mark(kRotateField);
            }

            if (!full_snapshot) {
                DegradeStaleFireCode(g.FireCode, now);
            }
        }

        void ApplySentryCmdCommand(SentryCmdType& command, const gimbal_driver::msg::SentryCmd& m) {
            const bool full_snapshot = (m.field_mask == 0) ||
                ((m.field_mask & gimbal_driver::msg::SentryCmd::FIELD_ALL) == gimbal_driver::msg::SentryCmd::FIELD_ALL);

            auto has_field = [&](const std::uint8_t field) {
                return full_snapshot || ((m.field_mask & field) != 0);
            };

            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_CONFIRM_FREE_REVIVE)) {
                command.ConfirmFreeRevive = m.confirm_free_revive ? 1 : 0;
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_CONFIRM_IMMEDIATE_REVIVE)) {
                command.ConfirmImmediateRevive = m.confirm_immediate_revive ? 1 : 0;
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_EXCHANGE_PROJECTILE_ALLOWANCE)) {
                command.ExchangeProjectileAllowance = ClampU16Bits(m.exchange_projectile_allowance, 11);
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_REMOTE_PROJECTILE_EXCHANGE_COUNT)) {
                command.RemoteProjectileExchangeCount =
                    ClampU8Bits(m.remote_projectile_exchange_count, 4);
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_REMOTE_HP_EXCHANGE_COUNT)) {
                command.RemoteHpExchangeCount = ClampU8Bits(m.remote_hp_exchange_count, 4);
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_POSTURE)) {
                if (m.posture > 6) {
                    roslog::warn("Invalid /ly/control/sentry_cmd posture: %u (expect 0/1/2/3/4/5/6)",
                                 m.posture);
                } else if (!IsPostureCommandAllowed(m.posture)) {
                    WarnRejectedEnhancedPosture("/ly/control/sentry_cmd", m.posture);
                } else {
                    const auto posture = ClampU3(m.posture);
                    command.Posture = posture;
                    postureCommand_ = posture;
                    if (posture == 0) {
                        posturePendingRepeat_ = 0;
                        posturePendingToSend_ = 0;
                    } else {
                        ArmPostureTx(posture);
                    }
                }
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_CONFIRM_ENERGY_ACTIVATE)) {
                command.ConfirmEnergyActivate = m.confirm_energy_activate ? 1 : 0;
            }
        }

        bool SendSentryCommand(const char* reason) {
            if (rawDownlinkTestMode_) {
                return false;
            }
            if (DeviceError) {
                return false;
            }
            SentryCommandFrame frame;
            frame.SentryCmd = sentryCmdShadow_;
            frame.SentryCmd.Posture = PostureForTransmission(
                frame.SentryCmd.Posture, reason);
            if (!Device.WriteRaw(frame)) {
                DeviceError = true;
                return false;
            }
            LogDownlinkRaw(frame, reason);
            return true;
        }

        bool IsFreshGamePath(const gimbal_driver::msg::MapPath& msg) const {
            if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
                roslog::warn("Drop /ly/game/path without header.stamp");
                return false;
            }
            const auto now = Node.GetNode()->now();
            const rclcpp::Time stamp(msg.header.stamp, now.get_clock_type());
            const auto age = now - stamp;
            if (age.nanoseconds() > gamePathFreshTimeout_.count() * 1000000LL) {
                roslog::warn(
                    "Drop stale /ly/game/path: age=%ld ms exceeds %ld ms",
                    age.nanoseconds() / 1000000LL,
                    gamePathFreshTimeout_.count());
                return false;
            }
            return true;
        }

        void SendMapPath(const gimbal_driver::msg::MapPath& msg, const bool require_fresh_stamp) {
            if (rawDownlinkTestMode_) {
                return;
            }
            if (DeviceError) {
                return;
            }
            if (require_fresh_stamp && !IsFreshGamePath(msg)) {
                return;
            }
            if (msg.intention < 1 || msg.intention > 3) {
                roslog::warn("Invalid map path intention: %u (expect 1/2/3)", msg.intention);
                return;
            }
            if (!mapPathRateLimiter_.TryAcquire(std::chrono::steady_clock::now())) {
                roslog::warn("Drop map path: referee 0x0307 rate limit is 1 Hz");
                return;
            }
            MapPathFrame frame;
            frame.Intention = msg.intention;
            frame.StartPositionX_dm = msg.start_position_x_dm;
            frame.StartPositionY_dm = msg.start_position_y_dm;
            std::copy(msg.delta_x_dm.begin(), msg.delta_x_dm.end(), std::begin(frame.DeltaX_dm));
            std::copy(msg.delta_y_dm.begin(), msg.delta_y_dm.end(), std::begin(frame.DeltaY_dm));
            frame.SenderId = msg.sender_id;
            const auto fragments = MakeMapPathFragments(frame, nextMapPathSequence_++);
            for (const auto& fragment : fragments) {
                if (!Device.WriteRaw(fragment)) {
                    DeviceError = true;
                    return;
                }
                LogDownlinkRaw(fragment, "map_path_fragment");
            }
        }

        void SendCustomInfo(const gimbal_driver::msg::CustomInfo& msg) {
            if (rawDownlinkTestMode_) {
                return;
            }
            if (DeviceError) {
                return;
            }
            CustomInfoFrame frame;
            frame.SenderId = msg.sender_id;
            frame.ReceiverId = msg.receiver_id;
            std::copy(msg.user_data_utf16.begin(), msg.user_data_utf16.end(), std::begin(frame.UserDataUtf16));
            if (!Device.WriteRaw(frame)) {
                DeviceError = true;
                return;
            }
            LogDownlinkRaw(frame, "custom_info");
        }

        void MaybeSendSentryCoordinate() {
            if (rawDownlinkTestMode_) {
                return;
            }
            if (!sentryCoordPending_ || !sentryCoordValid_) {
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now < sentryCoordNextSendTime_) {
                return;
            }
            if (sentryCoordLastRxTime_.time_since_epoch().count() != 0 &&
                now - sentryCoordLastRxTime_ > std::chrono::milliseconds(sentryCoordFreshTimeoutMs_)) {
                sentryCoordPending_ = false;
                sentryCoordValid_ = false;
                return;
            }

            SentryCoordinateFrame frame;
            frame.X_cm = EncodeCoordinateCm(sentryCoordX_, sentryCoordFieldWidthX_);
            frame.Y_cm = EncodeCoordinateCm(sentryCoordY_, sentryCoordFieldWidthY_);
            frame.CRC8 = CRCChecker::CRC8::calculate_downlink(
                reinterpret_cast<const std::uint8_t*>(&frame),
                sizeof(frame) - sizeof(frame.CRC8));
            if (!Device.WriteRaw(frame)) {
                DeviceError = true;
                return;
            }
            LogDownlinkRaw(frame, "sentry_coordinate");
            sentryCoordPending_ = false;
            sentryCoordNextSendTime_ = now + sentryCoordSendInterval_;
        }

        void HandleSentryPosition(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            if (!msg) {
                return;
            }
            if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y)) {
                return;
            }

            sentryCoordX_ = static_cast<float>(msg->point.x * 100.0);
            sentryCoordY_ = static_cast<float>(msg->point.y * 100.0);
            sentryCoordPending_ = true;
            sentryCoordValid_ = true;
            sentryCoordLastRxTime_ = std::chrono::steady_clock::now();
        }

        void MaybeApplyFireCodeStaleFallback() {
            const auto now = std::chrono::steady_clock::now();
            auto next = controlShadow_.FireCode;
            const auto before_raw = FireCodeRaw(next);
            DegradeStaleFireCode(next, now);
            if (FireCodeRaw(next) == before_raw) {
                return;
            }
            CallbackGenerator.Modify([&](GimbalControlFrame& g) {
                g.FireCode = next;
            });
        }

        void MaybeApplyNavigationTestStaleFallback() {
            if (!navigationVelocityForwardEnable_ || !navigationTestVelocityActive_) {
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            if (navigationTestLastVelRxTime_.time_since_epoch().count() == 0 ||
                now - navigationTestLastVelRxTime_ <= navigationTestStaleTimeout_) {
                return;
            }

            CallbackGenerator.Modify([&](GimbalControlFrame& g) {
                g.Velocity.X = 0;
                g.Velocity.Y = 0;
            });
            navigationTestVelocityActive_ = false;
            roslog::warn("navigation velocity forwarding: /ly/navi/vel stale for >%d ms, publish zero velocity",
                         static_cast<int>(navigationTestStaleTimeout_.count()));
        }

        void ApplyNavigationModeRotate(const bool should_rotate) {
            if (!navigationModeEnable_) {
                return;
            }

            CallbackGenerator.Modify([this, should_rotate](GimbalControlFrame& g) {
                g.FireCode.Rotate = should_rotate ? navigationModeRotateLevel_ : 0;
                if (navigationModeFollowModeWhenFalse_) {
                    g.FireCode.FollowMode = should_rotate ? 0 : 1;
                }
            });
        }

        template<typename TTopic>
        void GenSub(auto modifier)
        {
            Node.GenSubscriber<TTopic>(CallbackGenerator.Generate<TTopic>(modifier));
        }

        //TODO disable cooldown timer
        using clock_t = std::chrono::steady_clock;
           
        struct Timer {
            clock_t::duration interval;
            clock_t::time_point head;

            bool check(const clock_t::time_point now) {
                const auto delta_t = now - head;
                if (delta_t > interval) {
                    head = now;
                    return true;
                }
                return false;
            }
        };

        class StateTimer {
            Timer shoot_timer{};
            Timer cooldown_timer{};
            bool is_shoot{false};

            Timer& get_timer() {
                return is_shoot ? shoot_timer : cooldown_timer;
            }

        public:
            bool can_shoot() const noexcept {
                return is_shoot;
            }

            StateTimer(const clock_t::duration shoot, const clock_t::duration cooldown) {
                const auto now = clock_t::now();
                shoot_timer.head = now;
                shoot_timer.interval = shoot;
                cooldown_timer.head = now;
                cooldown_timer.interval = cooldown;
            }

            bool check() {
                const auto now = clock_t::now();
                if (get_timer().check(now)) {
                    is_shoot = !is_shoot;
                    get_timer().head = now;
                }
                
                return can_shoot();
            }

        };

        StateTimer state_timer{std::chrono::milliseconds(100), std::chrono::milliseconds(2000)}; //簡學長寫的timer，使用的話則關掉下面的false &&
        //TODO 

        void GenSubs()
        {
            if (rawDownlinkTestMode_) {
                GenRawDownlinkTestSubscriptions();
                return;
            }

            GenSub<ly_control_angles>([](GimbalControlFrame& g, const gimbal_driver::msg::GimbalAngles& m)
                                        {
                                            g.GimbalAngles.Yaw = static_cast<float>(m.yaw);
                                            g.GimbalAngles.Pitch = static_cast<float>(m.pitch);
                                        });

            trajectorySubscription_ = Node.GetNode()->create_subscription<gimbal_driver::msg::GimbalTrajectory>(
                ly_control_trajectory::Name,
                rclcpp::SensorDataQoS().keep_last(1),
                [this](const gimbal_driver::msg::GimbalTrajectory::ConstSharedPtr msg) {
                    if (!msg) {
                        return;
                    }
                    SendGimbalTrajectory(*msg);
                });

            GenSub<ly_control_firecode>([this](GimbalControlFrame& g, const gimbal_driver::msg::FireCode& m)
                                        {
                                            ApplyFireCodeCommand(g, m);
                                            if (false && !state_timer.check()) {
                                                g.FireCode.FireStatus = 0;
                                            }
                                        });

            GenSub<ly_control_vel>([this](GimbalControlFrame& g, const gimbal_driver::msg::ControlVelocity& m)
                                   {
                                       if (m.use_raw) {
                                           g.Velocity.X = m.raw_x;
                                           g.Velocity.Y = m.raw_y;
                                           return;
                                       }
                                       g.Velocity.X = EncodeVelocityRaw(m.x_mps);
                                       g.Velocity.Y = EncodeVelocityRaw(m.y_mps);
                                   });

            if (navigationVelocityForwardEnable_) {
                GenSub<ly_navi_vel>([this](GimbalControlFrame& g, const gimbal_driver::msg::Vel& m)
                                    {
                                        g.Velocity.X = EncodeNavigationTestVelocityRaw(m.x);
                                        g.Velocity.Y = EncodeNavigationTestVelocityRaw(m.y);
                                        navigationTestLastVelRxTime_ = std::chrono::steady_clock::now();
                                        navigationTestVelocityActive_ = true;
                });
            }

            if (navigationModeEnable_ && navigationModeShouldRotateEnable_) {
                Node.GenSubscriber<ly_navi_should_rotate>([this](const ly_navi_should_rotate::CallbackArg msg) {
                    navigationModeShouldRotate_ = msg->data;
                    ApplyNavigationModeRotate(navigationModeShouldRotate_);
                });
            }

            Node.GenSubscriber<ly_control_posture>([this](const ly_control_posture::CallbackArg msg) {
                const bool has_posture =
                    msg->field_mask == 0 ||
                    ((msg->field_mask & gimbal_driver::msg::SentryCmd::FIELD_POSTURE) != 0);
                if (!has_posture) {
                    roslog::warn("/ly/control/posture missing FIELD_POSTURE; ignore");
                    return;
                }
                const auto command = msg->posture;
                if (command != 0 && !IsValidPostureCommand(command)) {
                    roslog::warn("Invalid /ly/control/posture: %u (expect 0/1/2/3/4/5/6)", command);
                    return;
                }
                if (!IsPostureCommandAllowed(command)) {
                    WarnRejectedEnhancedPosture("/ly/control/posture", command);
                    return;
                }
                postureCommand_ = command;
                sentryCmdShadow_.Posture = command;
                if (command == 0) {
                    posturePendingRepeat_ = 0;
                    posturePendingToSend_ = 0;
                } else {
                    ArmPostureTx(command);
                }
                SendSentryCommand("posture");
            });

            Node.GenSubscriber<ly_control_sentry_cmd>([this](const ly_control_sentry_cmd::CallbackArg msg) {
                ApplySentryCmdCommand(sentryCmdShadow_, *msg);
                SendSentryCommand("sentry_command");
            });

            Node.GenSubscriber<ly_control_map_path>([this](const ly_control_map_path::CallbackArg msg) {
                SendMapPath(*msg, false);
            });

            Node.GenSubscriber<ly_game_path>([this](const ly_game_path::CallbackArg msg) {
                SendMapPath(*msg, true);
            });

            Node.GenSubscriber<ly_control_custom_info>([this](const ly_control_custom_info::CallbackArg msg) {
                SendCustomInfo(*msg);
            });
        }

        void PublishRfidStatus(const rclcpp::Time& stamp) {
            if (!hasRfidStatusRaw_ && !hasRfidStatus2_) {
                return;
            }
            using topic = ly_game_rfid;
            auto msg = ToRfidStatusMsg(latestRfidStatusRaw_, hasRfidStatus2_, latestRfidStatus2_);
            msg.header.stamp = stamp;
            Node.Publisher<topic>()->publish(msg);
        }

        void PublishBulletInfo(const rclcpp::Time& stamp) {
            using topic = ly_game_bullet;
            topic::Msg msg;
            msg.header.stamp = stamp;

            msg.has_initial_speed = hasBulletInitialSpeed_;
            msg.initial_speed = hasBulletInitialSpeed_ ? latestBulletInitialSpeed_ : 0.0f;

            msg.has_shoot_data = hasBulletDataAndRfid2_;
            msg.bullet_type = hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.BulletType : 0;
            msg.shooter_number = hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.ShooterNumber : 0;
            msg.launching_frequency =
                hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.LaunchingFrequency : 0;

            msg.has_projectile_allowance = hasBulletDataAndRfid2_;
            msg.projectile_allowance_17mm =
                hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.ProjectileAllowance17mm : 0;
            msg.projectile_allowance_42mm =
                hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.ProjectileAllowance42mm : 0;
            msg.remaining_gold_coin =
                hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.RemainingGoldCoin : 0;
            msg.projectile_allowance_fortress_17mm =
                hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.ProjectileAllowanceFortress : 0;
            Node.Publisher<topic>()->publish(msg);
        }

        void PublishGimbalState(const rclcpp::Time& stamp) {
            gimbal_driver::msg::GimbalState msg;
            msg.header.stamp = stamp;

            {
                std::lock_guard lock{gimbalStateMutex_};
                msg.yaw = latestGimbalYaw_;
                msg.pitch = latestGimbalPitch_;
                const auto dynamics_age = std::chrono::steady_clock::now() - latestGimbalDynamicsRxTime_;
                if (hasGimbalDynamics_ &&
                    dynamics_age <= gimbalDynamicsTimeout_) {
                    msg.yaw_omega = latestGimbalYawOmega_;
                    msg.pitch_omega = latestGimbalPitchOmega_;
                    msg.yaw_alpha = latestGimbalYawAlpha_;
                    msg.pitch_alpha = latestGimbalPitchAlpha_;
                }
            }

            if (gimbalStatePublisher_) {
                gimbalStatePublisher_->publish(msg);
            }
        }

        void MaybePublishGimbalState() {
            const auto now = std::chrono::steady_clock::now();
            if (now < nextGimbalStatePublishTime_) {
                return;
            }
            nextGimbalStatePublishTime_ = now + gimbalStatePublishPeriod_;
            PublishGimbalState(Node.GetNode()->now());
        }

        void ResetGimbalInterfaceState() {
            std::lock_guard lock{gimbalStateMutex_};
            latestGimbalYaw_ = 0.0F;
            latestGimbalPitch_ = 0.0F;
            latestGimbalYawOmega_ = 0.0F;
            latestGimbalPitchOmega_ = 0.0F;
            latestGimbalYawAlpha_ = 0.0F;
            latestGimbalPitchAlpha_ = 0.0F;
            hasGimbalDynamics_ = false;
            latestGimbalSampleTickMs_ = 0;
            latestGimbalDynamicsRxTime_ = {};
            trajectoryShadow_ = {};
            nextGimbalStatePublishTime_ = std::chrono::steady_clock::now();
        }

        void PubGimbalDynamics(
            const TypedMessage<sizeof(GimbalData)>& frame,
            const rclcpp::Time& stamp) {
            if (!CRCChecker::CRC8::verify(
                    reinterpret_cast<const std::uint8_t*>(&frame), sizeof(frame))) {
                roslog::warn("Drop TypeID 11 gimbal dynamics frame with invalid CRC8");
                return;
            }

            const auto& data = frame.GetDataAs<GimbalDynamicsData>();
            {
                std::lock_guard lock{gimbalStateMutex_};
                if (hasGimbalDynamics_ &&
                    !IsNewerSampleTick(
                        data.SampleTickMs, latestGimbalSampleTickMs_)) {
                    roslog::warn(
                        "Drop out-of-order TypeID 11 sample tick: current=%u previous=%u",
                        data.SampleTickMs,
                        latestGimbalSampleTickMs_);
                    return;
                }
                if (hasGimbalDynamics_) {
                    const auto sample_delta = data.SampleTickMs - latestGimbalSampleTickMs_;
                    if (sample_delta > static_cast<std::uint32_t>(gimbalDynamicsTimeout_.count())) {
                        roslog::warn(
                            "TypeID 11 sample gap is %u ms (timeout=%ld ms)",
                            sample_delta,
                            static_cast<long>(gimbalDynamicsTimeout_.count()));
                    }
                }
                latestGimbalYawOmega_ = static_cast<float>(data.YawOmegaDpsX10) / 10.0F;
                latestGimbalPitchOmega_ = static_cast<float>(data.PitchOmegaDpsX10) / 10.0F;
                latestGimbalYawAlpha_ = static_cast<float>(data.YawAlphaDps2);
                latestGimbalPitchAlpha_ = static_cast<float>(data.PitchAlphaDps2);
                latestGimbalSampleTickMs_ = data.SampleTickMs;
                latestGimbalDynamicsRxTime_ = std::chrono::steady_clock::now();
                hasGimbalDynamics_ = true;
            }
            PublishGimbalState(stamp);
        }

        void  PubGimbalData(const GimbalData& data)
	        {
            const auto stamp = Node.GetNode()->now();
            {
                std::lock_guard lock{gimbalStateMutex_};
                latestGimbalYaw_ = static_cast<float>(data.GimbalAngles.Yaw);
                latestGimbalPitch_ = static_cast<float>(data.GimbalAngles.Pitch);
            }
            {
                using topic = ly_gimbal_angles;
                topic::Msg msg;
                msg.yaw = static_cast<float>(data.GimbalAngles.Yaw);
                msg.pitch = static_cast<float>(data.GimbalAngles.Pitch);
                msg.header.stamp = stamp;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_firecode;
                auto msg = ToFireCodeMsg(data.FireCode);
                msg.header.stamp = Node.GetNode()->now();
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_capV;
                topic::Msg msg;
                msg.data = static_cast<std::uint8_t>(data.CapV);
                Node.Publisher<topic>()->publish(msg);
            }
            PublishGimbalState(stamp);
        }

        void PubGameData(const GameData& data, const rclcpp::Time& stamp)
        {
            selfSentryRobotId_ = data.GameCode.IsMyTeamRed ? 7u : 107u;
            const bool use_legacy_outpost_hp = !HasFreshPreciseOutpostHp();
            {
                using topic = ly_game_all;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.gamecode = *reinterpret_cast<const std::uint16_t*>(&data.GameCode);
                msg.ammoleft = data.AmmoLeft;
                msg.timeleft = data.TimeLeft;
                msg.selfhealth = data.SelfHealth;
                msg.exteventdata = *static_cast<const std::uint32_t*>(&data.ExtEventData);
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_ammo_left;
                topic::Msg msg;
                msg.data = data.AmmoLeft;
                Node.Publisher<topic>()->publish(msg);
            }
            if (use_legacy_outpost_hp) {
                PublishOutpostHp(
                    static_cast<std::uint16_t>(data.GameCode.SelfOutpostHealth * 25u),
                    static_cast<std::uint16_t>(data.GameCode.EnemyOutpostHealth * 25u),
                    stamp);
            }
            {
                using topic = ly_friend_is_precaution;
                topic::Msg msg;
                msg.data = data.GameCode.HeroPrecaution;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_is_start;
                topic::Msg msg;
                msg.data = data.GameCode.IsGameBegin;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_is_team_red;
                topic::Msg msg;
                msg.data = data.GameCode.IsMyTeamRed;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_is_at_home;
                topic::Msg msg;
                msg.data = data.GameCode.IsReturnedHome;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_time_left;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.data = data.TimeLeft;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_eventdata;
                topic::Msg msg;
                msg.data = static_cast<std::uint32_t>(data.ExtEventData);
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_event_data;
                auto msg = ToEventDataMsg(data.ExtEventData);
                msg.header.stamp = stamp;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubRFIDAndBuffData(const RFIDAndBuffData& data){
            latestRfidStatusRaw_ = data.RFIDStatus;
            hasRfidStatusRaw_ = true;
            PublishRfidStatus(Node.GetNode()->now());
            {
                using topic = ly_team_buff;
                topic::Msg msg;
                msg.recoverybuff = data.BuffStatus.RecoveryBuff;
                msg.coolingbuff = data.BuffStatus.CoolingBuff;
                msg.defencebuff = data.BuffStatus.DefenceBuff;
                msg.vulnerabilitybuff = data.BuffStatus.VulnerabilityBuff;
                msg.attackbuff = data.BuffStatus.AttackBuff;
                msg.remainingenergy = data.BuffStatus.RemainingEnergy;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubPositionData(const PositionData& data){
            {
                {
                    using topic = ly_position_data;
                    topic::Msg msg;
                    msg.header.stamp = Node.GetNode()->now();
                    msg.friendcarid = data.Friend.CarId;
                    msg.friendx = data.Friend.X;
                    msg.friendy = data.Friend.Y;
                    msg.enemycarid = data.Enemy.CarId;
                    msg.enemyx = data.Enemy.X;
                    msg.enemyy = data.Enemy.Y;
                    Node.Publisher<topic>()->publish(msg);
                }
                if(data.Friend.CarId == 7) {
                    using topic = ly_friend_uwb_pos;
                    std::vector<std::uint16_t> pos = {
                        static_cast<std::uint16_t>(data.Friend.X),
                        static_cast<std::uint16_t>(data.Friend.Y)
                    };
                    topic::Msg msg;
                    msg.header.stamp = Node.GetNode()->now();
                    msg.header.frame_id = "official_map";
                    msg.data = pos;
                    Node.Publisher<topic>()->publish(msg);
                }
            }
            {
                using topic = ly_bullet_speed;
                topic::Msg msg;
                msg.data = static_cast<float>(data.BulletSpeed) / 100.0f;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubHealthMyselfData(const HealthMyselfData& data, const rclcpp::Time& stamp){
            {
                using topic = ly_friend_hp;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.hero = data.HeroMyself;
                msg.engineer = data.EngineerMyself;
                msg.infantry1 = data.Infantry1Myself;
                msg.infantry2 = data.Infantry2Myself;
                msg.reserve = data.BaseMyself;
                msg.sentry = data.SentryMyself;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_base_hp;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.data = data.BaseMyself;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubHealthEnemyData(const HealthEnemyData& data, const rclcpp::Time& stamp){
            {
                using topic = ly_enemy_hp;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.hero = data.HeroEnemy;
                msg.engineer = data.EngineerEnemy;
                msg.infantry1 = data.Infantry1Enemy;
                msg.infantry2 = data.Infantry2Enemy;
                msg.reserve = data.BaseEnemy;
                msg.sentry = data.SentryEnemy;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_enemy_base_hp;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.data = data.BaseEnemy;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubChassisData(const ChassisData& data, const rclcpp::Time& stamp) {
            const auto packed1_u32 = static_cast<std::uint32_t>(data.ChassisPacked1);
            const auto packed2_u32 = static_cast<std::uint32_t>(data.ChassisPacked2);

            const auto steer_angle_u16 = static_cast<std::uint16_t>(packed1_u32 & 0xFFFFu);
            const auto angular_vel_u16 = static_cast<std::uint16_t>((packed1_u32 >> 16) & 0xFFFFu);
            const auto vel_x_u16 = static_cast<std::uint16_t>(packed2_u32 & 0xFFFFu);
            const auto vel_y_u16 = static_cast<std::uint16_t>((packed2_u32 >> 16) & 0xFFFFu);

            const auto steer_angle = DecodeI16WithScale(steer_angle_u16, 10.0f);
            const auto angular_velocity = DecodeI16WithScale(angular_vel_u16, 100.0f);
            const auto velocity_x = DecodeI16WithScale(vel_x_u16, 100.0f);
            const auto velocity_y = DecodeI16WithScale(vel_y_u16, 100.0f);

            {
                using topic = ly_friend_uwb_yaw;
                topic::Msg msg;
                msg.data = data.UWBAngleYaw;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_chassis;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.steer_angle = steer_angle;
                msg.angular_velocity = angular_velocity;
                msg.velocity_x = velocity_x;
                msg.velocity_y = velocity_y;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_big_yaw_angles;
                topic::Msg msg;
                msg.data = steer_angle;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_vel;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.x = velocity_x;
                msg.y = velocity_y;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_damage_difference;
                topic::Msg msg;
                msg.header.stamp = stamp;
                msg.data = data.DamageDifference;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubSentryData(const SentryData& data) {
            const auto now = Node.GetNode()->now();
            const auto sentry_info_3_age_ms = [&]() -> std::uint32_t {
                if (!hasSentryInfo3_ || lastSentryInfo3RxTime_.time_since_epoch().count() == 0) {
                    return std::numeric_limits<std::uint32_t>::max();
                }
                const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - lastSentryInfo3RxTime_).count();
                return age >= static_cast<decltype(age)>(std::numeric_limits<std::uint32_t>::max())
                    ? std::numeric_limits<std::uint32_t>::max()
                    : static_cast<std::uint32_t>(std::max<decltype(age)>(age, 0));
            }();
            auto sentry_info_msg = ToSentryInfoMsg(
                data, hasSentryInfo3_, latestSentryInfo3_, sentry_info_3_age_ms);
            sentry_info_msg.header.stamp = now;
            sentry_info_msg.self_robot_id = selfSentryRobotId_;
            {
                using topic = ly_game_sentry_info;
                Node.Publisher<topic>()->publish(sentry_info_msg);
            }
            if (IsValidPosture(sentry_info_msg.posture)) {
                postureState_ = sentry_info_msg.posture;
                PublishPosture(postureState_);
            }

            latestBulletInitialSpeed_ = data.BulletInitialSpeed;
            hasBulletInitialSpeed_ = true;
            PublishBulletInfo(now);
        }

        void PubSentryInfo3AndOutpostHpData(
            const SentryInfo3AndOutpostHpData& data,
            const rclcpp::Time& stamp) {
            latestSentryInfo3_ = data.SentryInfo3;
            hasSentryInfo3_ = true;
            lastSentryInfo3RxTime_ = std::chrono::steady_clock::now();
            preciseOutpostHpLastRxTime_ = std::chrono::steady_clock::now();
            PublishOutpostHp(data.SelfOutpostHealth, data.EnemyOutpostHealth, stamp);
        }

        void PubBulletDataAndRfid2(const BulletDataAndRfid2& data) {
            const auto now = Node.GetNode()->now();
            latestBulletDataAndRfid2_ = data;
            hasBulletDataAndRfid2_ = true;
            latestRfidStatus2_ = data.RfidStatus2;
            hasRfidStatus2_ = true;

            PublishBulletInfo(now);
            PublishRfidStatus(now);
        }

        void PubMapCommandData(const MapCommandData& data) {
            using topic = ly_game_map_command;
            topic::Msg msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.has_target_position = data.TargetRobotId == 0;
            msg.target_position_x_m = data.TargetPositionXMeter();
            msg.target_position_y_m = data.TargetPositionYMeter();
            msg.has_target_robot = data.TargetRobotId != 0;
            msg.target_robot_id = data.TargetRobotId;
            msg.cmd_keyboard = data.CmdKeyboard;
            msg.cmd_source = data.CmdSource;
            Node.Publisher<topic>()->publish(msg);
        }

        void LoopRead()
        {
            Device.LoopRead(DeviceError, [this](const TypedMessage<sizeof(GimbalData)>& m)
            {
                LogUplinkRaw(m);
                const auto stamp = Node.GetNode()->now();
                switch (m.TypeID)
                {
                    case GimbalData::TypeID:
                        PubGimbalData(m.GetDataAs<GimbalData>());
                        break;

                    case GameData::TypeID:
                    {
                        const auto& game_data = m.GetDataAs<GameData>();
                        PubGameData(game_data, stamp);
                        break;
                    }

                    case HealthMyselfData::TypeID:
                        PubHealthMyselfData(m.GetDataAs<HealthMyselfData>(), stamp);
                        break;

                    case HealthEnemyData::TypeID:
                        PubHealthEnemyData(m.GetDataAs<HealthEnemyData>(), stamp);
                        break;

                    case RFIDAndBuffData::TypeID:
                        PubRFIDAndBuffData(m.GetDataAs<RFIDAndBuffData>());
                        break;

                    case PositionData::TypeID:
                        PubPositionData(m.GetDataAs<PositionData>());
                        break;
                    case ChassisData::TypeID:
                    {
                        const auto& chassis_data = m.GetDataAs<ChassisData>();
                        PubChassisData(chassis_data, stamp);
                        break;
                    }
                    case SentryData::TypeID:
                    {
                        PubSentryData(m.GetDataAs<SentryData>());
                        break;
                    }
                    case BulletDataAndRfid2::TypeID:
                    {
                        PubBulletDataAndRfid2(m.GetDataAs<BulletDataAndRfid2>());
                        break;
                    }
                    case MapCommandData::TypeID:
                    {
                        PubMapCommandData(m.GetDataAs<MapCommandData>());
                        break;
                    }
                    case SentryInfo3AndOutpostHpData::TypeID:
                    {
                        PubSentryInfo3AndOutpostHpData(
                            m.GetDataAs<SentryInfo3AndOutpostHpData>(), stamp);
                        break;
                    }
                    case GimbalDynamicsData::TypeID:
                    {
                        PubGimbalDynamics(m, Node.GetNode()->now());
                        break;
                    }

                    default:
                        roslog::warn("Application::LoopRead: ignore unknown type id(%u)",
                                      static_cast<unsigned int>(m.TypeID));
                        break;
                }
            });
        }

        void TestVirtualLoopback(){
            if (rawDownlinkTestMode_) {
                Device.LoopRead(DeviceError, [](const TypedMessage<sizeof(GimbalData)>&) {});
                return;
            }
            TypedMessage<sizeof(GimbalData)> test_msg{};
            GimbalControlFrame test_msg2{};
            test_msg.TypeID = GimbalData::TypeID;
            test_msg.GetDataAs<GimbalData>().GimbalAngles.Yaw = 45.0f;
            test_msg2.GimbalAngles.Yaw = 30.0f;

            Device.Write(test_msg2);
            
            Device.LoopRead(DeviceError, [&](const TypedMessage<sizeof(GimbalData)>& received){
                if (memcmp(&test_msg, &received, sizeof(test_msg)) == 0) {
                    RCLCPP_INFO(rclcpp::get_logger("test"), "Virtual loopback test passed");
                }
            });
        }

    public:
        Application() noexcept : CallbackGenerator{
                [this](const auto& data)
                {
                    if (rawDownlinkTestMode_) {
                        return;
                    }
                    controlShadow_ = data;
                    if (DeviceError) return;
                    if (!Device.Write(data)) {
                        DeviceError = true;
                        return;
                    }
                    LogDownlinkRaw(data, "control_callback");
                }
        }
        {
        }

        void Run(int argc, char** argv)
        {
            Node.Initialize(argc, argv);
            Node.Publisher<ly_gimbal_big_yaw_angles>();
            auto node = Node.GetNode();
            gimbalStatePublisher_ = node->create_publisher<gimbal_driver::msg::GimbalState>(
                ly_gimbal_state::Name,
                rclcpp::SensorDataQoS().keep_last(1));
            rclcpp::Rate rate(250);
            bool useVirtualDevice = false;
            std::string serialDeviceName{"/dev/ttyACM0"};
            int serialBaudRate = 115200;
            auto getParamCompat = [this](const char* slash_key, const char* dot_key, auto& value, const auto& default_value) {
                using T = std::decay_t<decltype(value)>;
                Node.GetParam<T>(slash_key, value, static_cast<T>(default_value));
                T dot_value = value;
                Node.GetParam<T>(dot_key, dot_value, value);
                value = dot_value;
            };
            getParamCompat("io_config/use_virtual_device", "io_config.use_virtual_device", useVirtualDevice, false);
            getParamCompat("io_config/device_name", "io_config.device_name", serialDeviceName, std::string{"/dev/ttyACM0"});
            getParamCompat("io_config/baud_rate", "io_config.baud_rate", serialBaudRate, 115200);
            int postureRepeatCount = postureTxRepeatCount_;
            int postureRepeatIntervalMs = static_cast<int>(postureTxInterval_.count());
            bool enhancedPostureGuardEnable = enhancedPostureGuardEnable_;
            int enhancedPostureInfo3FreshMs =
                static_cast<int>(enhancedPostureInfo3FreshTimeout_.count());
            int firecodePartialHoldMs = static_cast<int>(firecodePartialHold_.count());
            int navigationTestStaleTimeoutMs = static_cast<int>(navigationTestStaleTimeout_.count());
            int gamePathFreshTimeoutMs = static_cast<int>(gamePathFreshTimeout_.count());
            int gimbalDynamicsTimeoutMs = static_cast<int>(gimbalDynamicsTimeout_.count());
            int gimbalStatePublishPeriodMs = static_cast<int>(gimbalStatePublishPeriod_.count());
            int sentryCoordSendIntervalMs = static_cast<int>(sentryCoordSendInterval_.count());
            int sentryCoordFieldWidthX = sentryCoordFieldWidthX_;
            int sentryCoordFieldWidthY = sentryCoordFieldWidthY_;
            int sentryCoordFreshTimeoutMs = sentryCoordFreshTimeoutMs_;
            double velocityRawToMps = velocityRawToMps_;
            bool navigationTestEnable = navigationTestEnable_;
            bool navigationModeEnable = navigationModeEnable_;
            bool navigationModeVelChainEnable = navigationModeVelChainEnable_;
            int navigationModeRotateLevel = navigationModeRotateLevel_;
            bool navigationModeShouldRotateEnable = navigationModeShouldRotateEnable_;
            bool navigationModeFollowModeWhenFalse = navigationModeFollowModeWhenFalse_;
            bool rawSerialLogEnable = rawSerialLogEnable_;
            bool rawSerialLogUplink = rawSerialLogUplink_;
            bool rawSerialLogDownlink = rawSerialLogDownlink_;
            bool rawSerialLogScreen = rawSerialLogScreen_;
            bool rawSerialLogFlush = rawSerialLogFlush_;
            std::string rawSerialLogDir = rawSerialLogDir_;
            std::string rawSerialLogTypeIds = rawSerialLogTypeIds_;
            bool rawSerialTopicEnable = rawSerialTopicEnable_;
            bool rawSerialTopicUplink = rawSerialTopicUplink_;
            bool rawSerialTopicDownlink = rawSerialTopicDownlink_;
            std::string rawSerialTopicTypeIds = rawSerialTopicTypeIds_;
            bool rawDownlinkTestMode = rawDownlinkTestMode_;
            bool serialModeEnable = serialModeEnable_;
            bool serialModeUploadEnable = serialModeUploadEnable_;
            bool serialModeDownloadEnable = serialModeDownloadEnable_;
            auto serialModeUploadTypeIdEnabled = serialModeUploadTypeIdEnabled_;
            auto serialModeDownloadTypeIdEnabled = serialModeDownloadTypeIdEnabled_;
            getParamCompat(
                "io_config/posture_repeat_count",
                "io_config.posture_repeat_count",
                postureRepeatCount,
                postureRepeatCount);
            getParamCompat(
                "io_config/posture_repeat_interval_ms",
                "io_config.posture_repeat_interval_ms",
                postureRepeatIntervalMs,
                postureRepeatIntervalMs);
            getParamCompat(
                "io_config/enhanced_posture_guard/enable",
                "io_config.enhanced_posture_guard.enable",
                enhancedPostureGuardEnable,
                enhancedPostureGuardEnable);
            getParamCompat(
                "io_config/enhanced_posture_guard/sentry_info3_fresh_ms",
                "io_config.enhanced_posture_guard.sentry_info3_fresh_ms",
                enhancedPostureInfo3FreshMs,
                enhancedPostureInfo3FreshMs);
            getParamCompat(
                "io_config/firecode_partial_hold_ms",
                "io_config.firecode_partial_hold_ms",
                firecodePartialHoldMs,
                firecodePartialHoldMs);
            getParamCompat(
                "io_config/velocity_raw_to_mps",
                "io_config.velocity_raw_to_mps",
                velocityRawToMps,
                velocityRawToMps);
            getParamCompat(
                "io_config/navigation_test",
                "io_config.navigation_test",
                navigationTestEnable,
                navigationTestEnable);
            getParamCompat(
                "io_config/navigation_test_stale_timeout_ms",
                "io_config.navigation_test_stale_timeout_ms",
                navigationTestStaleTimeoutMs,
                navigationTestStaleTimeoutMs);
            getParamCompat(
                "io_config/navigation_mode/enabled",
                "io_config.navigation_mode.enabled",
                navigationModeEnable,
                navigationModeEnable);
            getParamCompat(
                "io_config/navigation_mode/vel_chain",
                "io_config.navigation_mode.vel_chain",
                navigationModeVelChainEnable,
                navigationModeVelChainEnable);
            getParamCompat(
                "io_config/navigation_mode/rotate_level",
                "io_config.navigation_mode.rotate_level",
                navigationModeRotateLevel,
                navigationModeRotateLevel);
            getParamCompat(
                "io_config/navigation_mode/should_rotate/enabled",
                "io_config.navigation_mode.should_rotate.enabled",
                navigationModeShouldRotateEnable,
                navigationModeShouldRotateEnable);
            getParamCompat(
                "io_config/navigation_mode/should_rotate/follow_mode_when_false",
                "io_config/navigation_mode.should_rotate.follow_mode_when_false",
                navigationModeFollowModeWhenFalse,
                navigationModeFollowModeWhenFalse);
            getParamCompat(
                "io_config/game_path_fresh_timeout_ms",
                "io_config.game_path_fresh_timeout_ms",
                gamePathFreshTimeoutMs,
                gamePathFreshTimeoutMs);
            getParamCompat(
                "io_config/gimbal_dynamics_timeout_ms",
                "io_config.gimbal_dynamics_timeout_ms",
                gimbalDynamicsTimeoutMs,
                gimbalDynamicsTimeoutMs);
            getParamCompat(
                "io_config/gimbal_state_publish_period_ms",
                "io_config.gimbal_state_publish_period_ms",
                gimbalStatePublishPeriodMs,
                gimbalStatePublishPeriodMs);
            getParamCompat(
                "io_config/sentry_coord_send_interval_ms",
                "io_config.sentry_coord_send_interval_ms",
                sentryCoordSendIntervalMs,
                sentryCoordSendIntervalMs);
            getParamCompat(
                "io_config/sentry_coord_field_width_x",
                "io_config.sentry_coord_field_width_x",
                sentryCoordFieldWidthX,
                sentryCoordFieldWidthX);
            getParamCompat(
                "io_config/sentry_coord_field_width_y",
                "io_config.sentry_coord_field_width_y",
                sentryCoordFieldWidthY,
                sentryCoordFieldWidthY);
            getParamCompat(
                "io_config/sentry_coord_fresh_timeout_ms",
                "io_config.sentry_coord_fresh_timeout_ms",
                sentryCoordFreshTimeoutMs,
                sentryCoordFreshTimeoutMs);
            getParamCompat(
                "io_config/raw_serial_log_enable",
                "io_config.raw_serial_log_enable",
                rawSerialLogEnable,
                rawSerialLogEnable);
            getParamCompat(
                "io_config/raw_serial_log_uplink",
                "io_config.raw_serial_log_uplink",
                rawSerialLogUplink,
                rawSerialLogUplink);
            getParamCompat(
                "io_config/raw_serial_log_downlink",
                "io_config.raw_serial_log_downlink",
                rawSerialLogDownlink,
                rawSerialLogDownlink);
            getParamCompat(
                "io_config/raw_serial_log_screen",
                "io_config.raw_serial_log_screen",
                rawSerialLogScreen,
                rawSerialLogScreen);
            getParamCompat(
                "io_config/raw_serial_log_flush",
                "io_config.raw_serial_log_flush",
                rawSerialLogFlush,
                rawSerialLogFlush);
            getParamCompat(
                "io_config/raw_serial_log_dir",
                "io_config.raw_serial_log_dir",
                rawSerialLogDir,
                rawSerialLogDir);
            getParamCompat(
                "io_config/raw_serial_log_type_ids",
                "io_config.raw_serial_log_type_ids",
                rawSerialLogTypeIds,
                rawSerialLogTypeIds);
            getParamCompat(
                "io_config/raw_serial_topic_enable",
                "io_config.raw_serial_topic_enable",
                rawSerialTopicEnable,
                rawSerialTopicEnable);
            getParamCompat(
                "io_config/raw_serial_topic_uplink",
                "io_config.raw_serial_topic_uplink",
                rawSerialTopicUplink,
                rawSerialTopicUplink);
            getParamCompat(
                "io_config/raw_serial_topic_downlink",
                "io_config.raw_serial_topic_downlink",
                rawSerialTopicDownlink,
                rawSerialTopicDownlink);
            getParamCompat(
                "io_config/raw_serial_topic_type_ids",
                "io_config.raw_serial_topic_type_ids",
                rawSerialTopicTypeIds,
                rawSerialTopicTypeIds);
            getParamCompat(
                "io_config/raw_downlink_test_mode",
                "io_config.raw_downlink_test_mode",
                rawDownlinkTestMode,
                rawDownlinkTestMode);
            getParamCompat(
                "io_config/serial_mode",
                "io_config.serial_mode",
                serialModeEnable,
                serialModeEnable);
            getParamCompat(
                "io_config/upload/enable",
                "io_config.upload.enable",
                serialModeUploadEnable,
                serialModeUploadEnable);
            getParamCompat(
                "io_config/download/enable",
                "io_config.download.enable",
                serialModeDownloadEnable,
                serialModeDownloadEnable);
            for (std::size_t type_id = 0; type_id < kUploadTypeIdCount; ++type_id) {
                const auto key = "io_config/upload/typeid" + std::to_string(type_id);
                auto dot_key = key;
                std::replace(dot_key.begin(), dot_key.end(), '/', '.');
                getParamCompat(
                    key.c_str(),
                    dot_key.c_str(),
                    serialModeUploadTypeIdEnabled[type_id],
                    serialModeUploadTypeIdEnabled[type_id]);
            }
            for (std::size_t type_id = 0; type_id < kDownloadTypeIdCount; ++type_id) {
                std::ostringstream key;
                key << "io_config/download/typeid0x" << std::hex << std::setw(2)
                    << std::setfill('0') << type_id;
                const auto key_string = key.str();
                auto dot_key = key_string;
                std::replace(dot_key.begin(), dot_key.end(), '/', '.');
                getParamCompat(
                    key_string.c_str(),
                    dot_key.c_str(),
                    serialModeDownloadTypeIdEnabled[type_id],
                    serialModeDownloadTypeIdEnabled[type_id]);
            }

            if (postureRepeatCount <= 0) {
                roslog::warn("Invalid posture_repeat_count=%d, fallback to 3", postureRepeatCount);
                postureRepeatCount = 3;
            }
            if (postureRepeatIntervalMs <= 0) {
                roslog::warn("Invalid posture_repeat_interval_ms=%d, fallback to 20",
                             postureRepeatIntervalMs);
                postureRepeatIntervalMs = 20;
            }
            if (enhancedPostureInfo3FreshMs <= 0) {
                roslog::warn(
                    "Invalid enhanced_posture_guard.sentry_info3_fresh_ms=%d, fallback to 1500",
                    enhancedPostureInfo3FreshMs);
                enhancedPostureInfo3FreshMs = 1500;
            }
            if (firecodePartialHoldMs <= 0) {
                roslog::warn("Invalid firecode_partial_hold_ms=%d, fallback to 100", firecodePartialHoldMs);
                firecodePartialHoldMs = 100;
            }
            if (velocityRawToMps <= 0.0) {
                roslog::warn("Invalid velocity_raw_to_mps=%f, fallback to 0.025", velocityRawToMps);
                velocityRawToMps = 0.025;
            }
            if (navigationTestStaleTimeoutMs <= 0) {
                roslog::warn(
                    "Invalid navigation_test_stale_timeout_ms=%d, fallback to 500",
                    navigationTestStaleTimeoutMs);
                navigationTestStaleTimeoutMs = 500;
            }
            if (navigationModeRotateLevel < 0 || navigationModeRotateLevel > 3) {
                roslog::warn(
                    "Invalid navigation_mode.rotate_level=%d, clamp to 0..3",
                    navigationModeRotateLevel);
                navigationModeRotateLevel = std::clamp(navigationModeRotateLevel, 0, 3);
            }
            if (gamePathFreshTimeoutMs <= 0) {
                roslog::warn(
                    "Invalid game_path_fresh_timeout_ms=%d, fallback to 5000",
                    gamePathFreshTimeoutMs);
                gamePathFreshTimeoutMs = 5000;
            }
            if (gimbalDynamicsTimeoutMs <= 0) {
                roslog::warn(
                    "Invalid gimbal_dynamics_timeout_ms=%d, fallback to 200",
                    gimbalDynamicsTimeoutMs);
                gimbalDynamicsTimeoutMs = 200;
            }
            if (gimbalStatePublishPeriodMs <= 0) {
                roslog::warn(
                    "Invalid gimbal_state_publish_period_ms=%d, fallback to 20",
                    gimbalStatePublishPeriodMs);
                gimbalStatePublishPeriodMs = 20;
            }
            if (sentryCoordSendIntervalMs < 20) {
                roslog::warn(
                    "Invalid sentry_coord_send_interval_ms=%d, fallback to 20",
                    sentryCoordSendIntervalMs);
                sentryCoordSendIntervalMs = 20;
            }
            if (sentryCoordFieldWidthX <= 0) {
                roslog::warn(
                    "Invalid sentry_coord_field_width_x=%d, fallback to 2800",
                    sentryCoordFieldWidthX);
                sentryCoordFieldWidthX = 2800;
            }
            if (sentryCoordFieldWidthY <= 0) {
                roslog::warn(
                    "Invalid sentry_coord_field_width_y=%d, fallback to 1500",
                    sentryCoordFieldWidthY);
                sentryCoordFieldWidthY = 1500;
            }
            if (sentryCoordFreshTimeoutMs <= 0) {
                roslog::warn(
                    "Invalid sentry_coord_fresh_timeout_ms=%d, fallback to 2000",
                    sentryCoordFreshTimeoutMs);
                sentryCoordFreshTimeoutMs = 2000;
            }

            postureTxRepeatCount_ = postureRepeatCount;
            postureTxInterval_ = std::chrono::milliseconds(postureRepeatIntervalMs);
            enhancedPostureGuardEnable_ = enhancedPostureGuardEnable;
            enhancedPostureInfo3FreshTimeout_ = std::chrono::milliseconds(enhancedPostureInfo3FreshMs);
            firecodePartialHold_ = std::chrono::milliseconds(firecodePartialHoldMs);
            navigationTestStaleTimeout_ = std::chrono::milliseconds(navigationTestStaleTimeoutMs);
            gamePathFreshTimeout_ = std::chrono::milliseconds(gamePathFreshTimeoutMs);
            gimbalDynamicsTimeout_ = std::chrono::milliseconds(gimbalDynamicsTimeoutMs);
            gimbalStatePublishPeriod_ = std::chrono::milliseconds(gimbalStatePublishPeriodMs);
            sentryCoordSendInterval_ = std::chrono::milliseconds(sentryCoordSendIntervalMs);
            sentryCoordFieldWidthX_ = sentryCoordFieldWidthX;
            sentryCoordFieldWidthY_ = sentryCoordFieldWidthY;
            sentryCoordFreshTimeoutMs_ = sentryCoordFreshTimeoutMs;
            velocityRawToMps_ = static_cast<float>(velocityRawToMps);
            navigationTestEnable_ = navigationTestEnable;
            navigationModeEnable_ = navigationModeEnable;
            navigationModeVelChainEnable_ = navigationModeVelChainEnable;
            navigationModeRotateLevel_ = static_cast<std::uint8_t>(navigationModeRotateLevel);
            navigationModeShouldRotateEnable_ = navigationModeShouldRotateEnable;
            navigationModeFollowModeWhenFalse_ = navigationModeFollowModeWhenFalse;
            navigationVelocityForwardEnable_ =
                navigationTestEnable_ || (navigationModeEnable_ && navigationModeVelChainEnable_);
            rawDownlinkTestMode_ = rawDownlinkTestMode;
            if (rawDownlinkTestMode_) {
                rawSerialLogEnable = true;
                rawSerialLogUplink = true;
                rawSerialLogDownlink = true;
                rawSerialLogScreen = true;
                rawSerialTopicEnable = true;
                rawSerialTopicUplink = true;
                rawSerialTopicDownlink = true;
                serialModeDownloadEnable = false;
            }
            ConfigureRawSerialLog(
                rawSerialLogEnable,
                rawSerialLogUplink,
                rawSerialLogDownlink,
                rawSerialLogScreen,
                rawSerialLogFlush,
                rawSerialLogDir,
                rawSerialLogTypeIds);
            ConfigureRawSerialTopic(
                rawSerialTopicEnable,
                rawSerialTopicUplink,
                rawSerialTopicDownlink,
                rawSerialTopicTypeIds);
            ConfigureSerialModeRawTopics(
                serialModeEnable,
                serialModeUploadEnable,
                serialModeUploadTypeIdEnabled,
                serialModeDownloadEnable,
                serialModeDownloadTypeIdEnabled);
            GenSubs();
            if (rawDownlinkTestMode_) {
                roslog::warn(
                    "RAW DOWNLINK TEST MODE active: only /ly/download/typeid0x00..05 may write serial; normal control writers are disabled.");
            }
            roslog::warn("posture_tx merged mode: repeat_count=%d repeat_interval_ms=%d",
                         postureTxRepeatCount_,
                         static_cast<int>(postureTxInterval_.count()));
            roslog::warn(
                "enhanced posture guard: enable=%s sentry_info3_fresh_ms=%d",
                enhancedPostureGuardEnable_ ? "true" : "false",
                static_cast<int>(enhancedPostureInfo3FreshTimeout_.count()));
            roslog::warn("semantic control: firecode_partial_hold_ms=%d velocity_raw_to_mps=%.4f",
                         static_cast<int>(firecodePartialHold_.count()),
                         static_cast<double>(velocityRawToMps_));
            if (navigationTestEnable_) {
                roslog::warn(
                    "navigation_test enabled: /ly/navi/vel writes lower velocity directly; stale_timeout_ms=%d",
                    static_cast<int>(navigationTestStaleTimeout_.count()));
            }
            if (navigationModeEnable_) {
                roslog::warn(
                    "navigation_mode standalone direct-debug enabled: vel_chain=%s rotate_level=%u should_rotate=%s follow_mode_when_false=%s",
                    navigationModeVelChainEnable_ ? "true" : "false",
                    static_cast<unsigned int>(navigationModeRotateLevel_),
                    navigationModeShouldRotateEnable_ ? "true" : "false",
                    navigationModeFollowModeWhenFalse_ ? "true" : "false");
                if (navigationModeVelChainEnable_) {
                    roslog::warn(
                        "navigation_mode: direct velocity subscriber active: %s -> lower-machine Velocity",
                        ly_navi_vel::Name);
                }
                if (navigationModeShouldRotateEnable_) {
                    roslog::warn(
                        "navigation_mode: Rotate subscriber active: %s (false => Rotate=0%s)",
                        ly_navi_should_rotate::Name,
                        navigationModeFollowModeWhenFalse_ ? ", FollowMode=1" : "");
                }
            }
            roslog::warn(
                "sentry coordinate downlink: topic=%s DownlinkTypeID=0x%02x interval_ms=%d field_cm=(%d,%d) fresh_timeout_ms=%d",
                ly_bt_sentry_position::Name,
                SentryCoordinateFrame::DownlinkTypeIDValue,
                static_cast<int>(sentryCoordSendInterval_.count()),
                sentryCoordFieldWidthX_,
                sentryCoordFieldWidthY_,
                sentryCoordFreshTimeoutMs_);
            roslog::warn("gimbal raw serial log: enable=%s uplink=%s downlink=%s screen=%s dir=%s type_ids=%s",
                         rawSerialLogEnable_ ? "true" : "false",
                         rawSerialLogUplink_ ? "true" : "false",
                         rawSerialLogDownlink_ ? "true" : "false",
                         rawSerialLogScreen_ ? "true" : "false",
                         rawSerialLogDir_.c_str(),
                         rawSerialLogTypeIds_.c_str());
            roslog::warn("gimbal raw serial topic: enable=%s uplink=%s downlink=%s type_ids=%s",
                         rawSerialTopicEnable_ ? "true" : "false",
                         rawSerialTopicUplink_ ? "true" : "false",
                         rawSerialTopicDownlink_ ? "true" : "false",
                         rawSerialTopicTypeIds_.c_str());
            roslog::warn("serial_mode raw topics: enable=%s upload=%s download=%s",
                         serialModeEnable_ ? "true" : "false",
                         serialModeUploadEnable_ ? "true" : "false",
                         serialModeDownloadEnable_ ? "true" : "false");
            roslog::warn(
                "gimbal state publish: period_ms=%d dynamics_timeout_ms=%d",
                static_cast<int>(gimbalStatePublishPeriod_.count()),
                static_cast<int>(gimbalDynamicsTimeout_.count()));

            if (!rawDownlinkTestMode_) {
                subSentryPosition_ = node->create_subscription<ly_bt_sentry_position::Msg>(
                    ly_bt_sentry_position::Name,
                    10,
                    [this](const ly_bt_sentry_position::Msg::SharedPtr msg) {
                        HandleSentryPosition(msg);
                    });
            }

            while (rclcpp::ok())
            {
                if (!DeviceError) DeviceError = true;
                std::this_thread::sleep_for(1s);
                if (!Device.Initialize(useVirtualDevice, serialDeviceName, serialBaudRate)) continue;
                DeviceError = false;
                ResetGimbalInterfaceState();
                posturePendingRepeat_ = 0;
                postureLastSent_ = 0;
                navigationTestVelocityActive_ = false;
                navigationTestLastVelRxTime_ = {};
                if (navigationModeEnable_) {
                    ApplyNavigationModeRotate(navigationModeShouldRotate_);
                }
                const auto reconnect_posture = IsValidPostureCommand(postureCommand_)
                    ? PostureForTransmission(postureCommand_, "reconnect")
                    : 0;
                sentryCmdShadow_.Posture = reconnect_posture;
                if (reconnect_posture != 0) {
                    ArmPostureTx(reconnect_posture);
                }
                std::jthread reading{ [this, useVirtualDevice] { useVirtualDevice ? TestVirtualLoopback() : LoopRead(); } };
                while (!DeviceError) {
                    rclcpp::spin_some(node);
                    MaybeApplyFireCodeStaleFallback();
                    MaybeApplyNavigationTestStaleFallback();
                    MaybeSendPostureTx();
                    MaybeSendSentryCoordinate();
                    MaybePublishGimbalState();
                    rate.sleep();
                }
            }
        }
    };
}

int main(int argc, char** argv) try
{
    RCLCPP_INFO(rclcpp::get_logger("main"), "main: running %s", Application::Name);
    Application app{};
    app.Run(argc, argv);
    return 0;
}
catch (const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("main"), "main: %s", e.what());
    return 1;
}
