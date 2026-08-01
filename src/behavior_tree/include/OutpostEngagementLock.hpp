#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "PostureTypes.hpp"

namespace BehaviorTree {

struct OutpostEngagementSetting {
    bool Enable{true};
    bool EnhancedAttackOnEnemyHpDrop{true};
    std::uint16_t NormalAttackLockExitHp{200};
    std::uint16_t EnhancedAttackLockExitHp{250};
};

struct OutpostEngagementInput {
    bool Target7Fresh{false};
    bool SelectedTarget7{false};
    bool EnemyHpFresh{false};
    std::uint16_t EnemyHp{0};
    bool SelfHpFresh{false};
    std::uint16_t SelfHp{0};
    bool NavigationReachable{true};
    PostureRuntime Posture{};
    bool PostureCooldownReady{false};
    bool EnhancedAttackRemainingFresh{false};
    std::uint8_t EnhancedAttackRemainingSec{0};
};

enum class OutpostEngagementExitReason : std::uint8_t {
    None = 0,
    TargetLost,
    EnemyHpStale,
    EnemyHpZero,
    NavigationUnreachable,
    NormalHealthThreshold,
    EnhancedHealthThreshold,
};

struct OutpostEngagementDecision {
    bool Active{false};
    bool HoldTarget{false};
    bool CancelPending{false};
    bool EnhancedArmed{false};
    bool EnhancedUnavailable{false};
    bool EnhancedPending{false};
    bool EnhancedActive{false};
    std::optional<PostureMode> Intent{};
    OutpostEngagementExitReason ExitReason{OutpostEngagementExitReason::None};
};

class OutpostEngagementLock {
public:
    using TimePoint = std::chrono::steady_clock::time_point;

    void Configure(const OutpostEngagementSetting& setting) noexcept;
    void Reset() noexcept;
    void MarkEnhancedUnavailable() noexcept;
    OutpostEngagementDecision Tick(TimePoint now, const OutpostEngagementInput& input);

private:
    OutpostEngagementSetting setting_{};
    bool active_{false};
    bool have_enemy_hp_{false};
    std::uint16_t last_enemy_hp_{0};
    bool enhanced_armed_{false};
    // Set only after the posture manager has accepted an enhanced request
    // (the request is visible as pending/active on the next tick).
    bool enhanced_request_accepted_{false};
    bool enhanced_unavailable_{false};

    OutpostEngagementDecision Exit(OutpostEngagementExitReason reason) noexcept;
};

}  // namespace BehaviorTree
