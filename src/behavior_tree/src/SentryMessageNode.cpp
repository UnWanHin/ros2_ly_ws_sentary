// Read-only tactical notification node. It never alters BT, navigation, or control state.
#include "SentryMessagePolicy.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "gimbal_driver/msg/custom_info.hpp"
#include "gimbal_driver/msg/event_data.hpp"
#include "gimbal_driver/msg/position_data.hpp"
#include "gimbal_driver/msg/sentry_info.hpp"
#include "gimbal_driver/msg/stamped_u_int16.hpp"
#include "sentry_msgs/msg/aim_result.hpp"
#include "sentry_msgs/msg/aim_target.hpp"

namespace BehaviorTree {
namespace {

using Clock = std::chrono::steady_clock;
constexpr int kSentryUnitId = 7;

struct Position {
    int x{0};
    int y{0};
    Clock::time_point received{};
};

struct EventConfig {
    bool enabled{true};
    std::chrono::milliseconds cooldown{0};
};

struct PendingMessage {
    std::string key;
    std::string text;
    std::vector<std::uint16_t> receivers;
    int priority{0};
};

class SentryMessageNode final : public rclcpp::Node {
public:
    SentryMessageNode()
        : Node("sentry_message", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
        if (!ReadBool("SentryMessage.Enable", false)) {
            RCLCPP_INFO(get_logger(), "sentry message disabled by SentryMessage.Enable=false");
            return;
        }

        global_min_interval_ = std::chrono::milliseconds(ReadInt("SentryMessage.GlobalMinIntervalMs", 500));
        state_fresh_timeout_ = std::chrono::milliseconds(ReadInt("SentryMessage.StateFreshMs", 2500));
        max_pending_ = std::max(1, ReadInt("SentryMessage.MaxPendingMessages", 16));
        LoadRecipients();
        LoadEventConfig("AttackTarget", attack_);
        LoadEventConfig("BaseDamage", base_damage_);
        LoadEventConfig("OutpostDamage", outpost_damage_);
        LoadEventConfig("CastleCapture", castle_capture_);
        LoadEventConfig("NearbyAlly", nearby_);
        LoadEventConfig("NearestEnemy", nearest_enemy_);
        nearby_distance_cm_ = ReadInt("SentryMessage.NearbyAlly.DistanceCm", 250);
        nearest_enemy_distance_cm_ = ReadInt("SentryMessage.NearestEnemy.ThreatDistanceCm", 400);

        publisher_ = create_publisher<gimbal_driver::msg::CustomInfo>("/ly/control/custom_info", 10);
        subscriptions_.push_back(create_subscription<gimbal_driver::msg::SentryInfo>(
            "/ly/game/sentry/info", 10,
            [this](const gimbal_driver::msg::SentryInfo::SharedPtr message) { OnSentryInfo(*message); }));
        subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
            "/ly/friend/is_team_red", 10,
            [this](const std_msgs::msg::Bool::SharedPtr message) { is_team_red_ = message->data; }));
        subscriptions_.push_back(create_subscription<gimbal_driver::msg::StampedUInt16>(
            "/ly/friend/base_hp", 10,
            [this](const gimbal_driver::msg::StampedUInt16::SharedPtr message) { OnHealth(*message, true); }));
        subscriptions_.push_back(create_subscription<gimbal_driver::msg::StampedUInt16>(
            "/ly/friend/op_hp", 10,
            [this](const gimbal_driver::msg::StampedUInt16::SharedPtr message) { OnHealth(*message, false); }));
        subscriptions_.push_back(create_subscription<gimbal_driver::msg::EventData>(
            "/ly/game/event_data", 10,
            [this](const gimbal_driver::msg::EventData::SharedPtr message) { OnEventData(*message); }));
        subscriptions_.push_back(create_subscription<gimbal_driver::msg::PositionData>(
            "/ly/position/data", 20,
            [this](const gimbal_driver::msg::PositionData::SharedPtr message) { OnPosition(*message); }));
        subscriptions_.push_back(create_subscription<sentry_msgs::msg::AimTarget>(
            "/ly/aim/select_target", 10,
            [this](const sentry_msgs::msg::AimTarget::SharedPtr message) { selected_target_ = message->id; }));
        subscriptions_.push_back(create_subscription<sentry_msgs::msg::AimResult>(
            "/ly/aim/result", 10,
            [this](const sentry_msgs::msg::AimResult::SharedPtr message) { OnAimResult(*message); }));
        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { FlushOne(); });
        RCLCPP_INFO(get_logger(), "sentry message enabled; only configured referee client IDs can receive messages");
    }

private:
    bool ReadBool(const std::string& name, const bool fallback) {
        if (!has_parameter(name)) {
            declare_parameter(name, fallback);
        }
        return get_parameter(name).as_bool();
    }

    int ReadInt(const std::string& name, const int fallback) {
        if (!has_parameter(name)) {
            declare_parameter(name, fallback);
        }
        return static_cast<int>(get_parameter(name).as_int());
    }

    void LoadEventConfig(const std::string& name, EventConfig& config) {
        config.enabled = ReadBool("SentryMessage." + name + ".Enable", true);
        config.cooldown = std::chrono::milliseconds(ReadInt("SentryMessage." + name + ".CooldownMs", 1000));
    }

    void LoadRecipients() {
        static constexpr std::array<std::pair<int, const char*>, 6> units{{
            {1, "Hero"}, {2, "Engineer"}, {3, "Infantry1"}, {4, "Infantry2"},
            {5, "Infantry3"}, {7, "Sentry"},
        }};
        for (const auto [unit, label] : units) {
            red_receivers_[unit] = ReadReceiver("Red", label);
            blue_receivers_[unit] = ReadReceiver("Blue", label);
        }
    }

    std::uint16_t ReadReceiver(const char* team, const char* unit) {
        const auto value = std::max(0, ReadInt(
            std::string("SentryMessage.Recipients.") + team + "." + unit, 0));
        return static_cast<std::uint16_t>(std::min(value, static_cast<int>(std::numeric_limits<std::uint16_t>::max())));
    }

    bool IsFreshHeader(const builtin_interfaces::msg::Time& stamp) const {
        if (stamp.sec == 0 && stamp.nanosec == 0) {
            return false;
        }
        const auto age = now() - rclcpp::Time(stamp);
        return age.nanoseconds() >= 0 && age.nanoseconds() <= state_fresh_timeout_.count() * 1000000LL;
    }

    bool IsFresh(const Position& position) const {
        return position.received.time_since_epoch().count() != 0 &&
            Clock::now() - position.received <= state_fresh_timeout_;
    }

    std::uint16_t ReceiverFor(const int unit) const {
        if (!is_team_red_.has_value()) {
            return 0U;
        }
        const auto& mapping = *is_team_red_ ? red_receivers_ : blue_receivers_;
        const auto found = mapping.find(unit);
        return found == mapping.end() ? 0U : found->second;
    }

    std::vector<std::uint16_t> AllReceivers() const {
        if (!is_team_red_.has_value()) {
            return {};
        }
        const auto& mapping = *is_team_red_ ? red_receivers_ : blue_receivers_;
        std::vector<std::uint16_t> receivers;
        for (const auto& [_, receiver] : mapping) {
            if (receiver != 0U && std::find(receivers.begin(), receivers.end(), receiver) == receivers.end()) {
                receivers.push_back(receiver);
            }
        }
        return receivers;
    }

    void Enqueue(const EventConfig& config, const std::string& key, const std::string& text,
                 std::vector<std::uint16_t> receivers, const int priority) {
        if (!config.enabled || receivers.empty()) {
            return;
        }
        const auto now_time = Clock::now();
        const auto last = last_event_.find(key);
        if (last != last_event_.end() && now_time - last->second < config.cooldown) {
            return;
        }
        last_event_[key] = now_time;
        for (auto& pending : pending_) {
            if (pending.key == key) {
                pending.text = text;
                pending.receivers = std::move(receivers);
                pending.priority = priority;
                return;
            }
        }
        if (static_cast<int>(pending_.size()) >= max_pending_) {
            const auto lowest = std::min_element(pending_.begin(), pending_.end(),
                [](const auto& lhs, const auto& rhs) { return lhs.priority < rhs.priority; });
            if (lowest == pending_.end() || lowest->priority >= priority) {
                return;
            }
            pending_.erase(lowest);
        }
        pending_.push_back({key, text, std::move(receivers), priority});
    }

    void FlushOne() {
        if (pending_.empty() || sender_id_ == 0U || Clock::now() - last_sent_ < global_min_interval_) {
            return;
        }
        const auto selected = std::max_element(pending_.begin(), pending_.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.priority < rhs.priority; });
        PendingMessage pending = std::move(*selected);
        pending_.erase(selected);
        for (const auto receiver : pending.receivers) {
            gimbal_driver::msg::CustomInfo message;
            message.header.stamp = now();
            message.sender_id = sender_id_;
            message.receiver_id = receiver;
            message.user_data_utf16 = SentryMessage::EncodeUtf8ToUtf16Le(pending.text);
            publisher_->publish(message);
        }
        last_sent_ = Clock::now();
        RCLCPP_INFO(get_logger(), "notify key=%s text=%s receivers=%zu", pending.key.c_str(),
                    pending.text.c_str(), pending.receivers.size());
    }

    void OnSentryInfo(const gimbal_driver::msg::SentryInfo& message) {
        if (message.self_robot_id == 7U || message.self_robot_id == 107U) {
            sender_id_ = message.self_robot_id;
        }
    }

    void OnHealth(const gimbal_driver::msg::StampedUInt16& message, const bool base) {
        if (!IsFreshHeader(message.header.stamp)) {
            return;
        }
        auto& baseline = base ? base_hp_ : outpost_hp_;
        const auto& config = base ? base_damage_ : outpost_damage_;
        const char* key = base ? "base_damage" : "outpost_damage";
        const char* label = base ? "BASE" : "OUTPOST";
        if (baseline.has_value() && message.data < *baseline) {
            Enqueue(config, key, std::string(label) + " HIT -" +
                std::to_string(static_cast<unsigned>(*baseline - message.data)), AllReceivers(), 100);
        }
        baseline = message.data;
    }

    void OnEventData(const gimbal_driver::msg::EventData& message) {
        if (!IsFreshHeader(message.header.stamp)) {
            return;
        }
        const bool contested = message.self_fortress_gain_point_status == 2U ||
            message.self_fortress_gain_point_status == 3U;
        if (castle_contested_.has_value() && !*castle_contested_ && contested) {
            Enqueue(castle_capture_, "castle_capture", "CASTLE ALERT", AllReceivers(), 120);
        }
        castle_contested_ = contested;
    }

    void OnAimResult(const sentry_msgs::msg::AimResult& message) {
        if (!last_fire_ && message.fire && selected_target_ != 0U) {
            Enqueue(attack_, "attack_" + std::to_string(selected_target_),
                    "ATTACK " + std::to_string(selected_target_), AllReceivers(), 70);
        }
        last_fire_ = message.fire;
    }

    static bool ValidPosition(const int x, const int y) { return x != 0 || y != 0; }

    void OnPosition(const gimbal_driver::msg::PositionData& message) {
        if (!IsFreshHeader(message.header.stamp)) {
            return;
        }
        const auto now_time = Clock::now();
        const auto friend_id = static_cast<std::size_t>(SentryMessage::CanonicalUnitId(message.friendcarid));
        if (friend_id < friends_.size() && ValidPosition(message.friendx, message.friendy)) {
            friends_[friend_id] = {message.friendx, message.friendy, now_time};
        }
        const auto enemy_id = static_cast<std::size_t>(SentryMessage::CanonicalUnitId(message.enemycarid));
        if (enemy_id < enemies_.size() && ValidPosition(message.enemyx, message.enemyy)) {
            enemies_[enemy_id] = {message.enemyx, message.enemyy, now_time};
        }
        CheckProximity();
    }

    static int DistanceCm(const Position& lhs, const Position& rhs) {
        const auto dx = lhs.x - rhs.x;
        const auto dy = lhs.y - rhs.y;
        return static_cast<int>(std::lround(std::sqrt(static_cast<double>(dx * dx + dy * dy))));
    }

    void CheckProximity() {
        const auto& sentry = friends_[kSentryUnitId];
        if (!IsFresh(sentry)) {
            return;
        }
        for (std::size_t unit = 1; unit < friends_.size(); ++unit) {
            if (unit == kSentryUnitId || !IsFresh(friends_[unit])) {
                continue;
            }
            const bool nearby = DistanceCm(sentry, friends_[unit]) <= nearby_distance_cm_;
            if (nearby && !nearby_last_[unit]) {
                Enqueue(nearby_, "nearby_" + std::to_string(unit), "SENTRY NEAR",
                        {ReceiverFor(static_cast<int>(unit))}, 40);
            }
            nearby_last_[unit] = nearby;
            CheckNearestEnemy(static_cast<int>(unit));
        }
    }

    void CheckNearestEnemy(const int friend_unit) {
        int nearest_id = 0;
        int nearest_distance = std::numeric_limits<int>::max();
        for (std::size_t enemy = 1; enemy < enemies_.size(); ++enemy) {
            if (!IsFresh(enemies_[enemy])) {
                continue;
            }
            const auto distance = DistanceCm(friends_[friend_unit], enemies_[enemy]);
            if (distance < nearest_distance) {
                nearest_id = static_cast<int>(enemy);
                nearest_distance = distance;
            }
        }
        const bool threat = nearest_id != 0 && nearest_distance <= nearest_enemy_distance_cm_;
        if (threat && nearest_enemy_last_[friend_unit] != nearest_id) {
            Enqueue(nearest_enemy_, "enemy_" + std::to_string(friend_unit),
                    "ENEMY " + std::to_string(nearest_id) + " " +
                        std::to_string(nearest_distance / 100) + "M",
                    {ReceiverFor(friend_unit)}, 50);
        }
        nearest_enemy_last_[friend_unit] = threat ? nearest_id : 0;
    }

    int max_pending_{16};
    int nearby_distance_cm_{250};
    int nearest_enemy_distance_cm_{400};
    std::chrono::milliseconds global_min_interval_{500};
    std::chrono::milliseconds state_fresh_timeout_{2500};
    EventConfig attack_{};
    EventConfig base_damage_{};
    EventConfig outpost_damage_{};
    EventConfig castle_capture_{};
    EventConfig nearby_{};
    EventConfig nearest_enemy_{};
    std::optional<bool> is_team_red_{};
    std::uint16_t sender_id_{0};
    std::uint8_t selected_target_{0};
    bool last_fire_{false};
    std::optional<std::uint16_t> base_hp_{};
    std::optional<std::uint16_t> outpost_hp_{};
    std::optional<bool> castle_contested_{};
    std::unordered_map<int, std::uint16_t> red_receivers_{};
    std::unordered_map<int, std::uint16_t> blue_receivers_{};
    std::unordered_map<std::string, Clock::time_point> last_event_{};
    std::deque<PendingMessage> pending_{};
    Clock::time_point last_sent_{};
    std::array<Position, 10> friends_{};
    std::array<Position, 10> enemies_{};
    std::array<bool, 10> nearby_last_{};
    std::array<int, 10> nearest_enemy_last_{};
    rclcpp::Publisher<gimbal_driver::msg::CustomInfo>::SharedPtr publisher_{};
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_{};
    rclcpp::TimerBase::SharedPtr timer_{};
};

}  // namespace

std::shared_ptr<rclcpp::Node> MakeSentryMessageNode() {
    return std::make_shared<SentryMessageNode>();
}

}  // namespace BehaviorTree
