// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include "../include/ExternalAimTargetPolicy.hpp"
#include "../include/NaviRotatePosture.hpp"

#include <algorithm>
#include <cmath>

namespace BehaviorTree {

namespace {
struct PostureScore {
    int Attack{0};
    int Defense{0};
    int Move{0};
};

int GetScore(const PostureScore& score, const SentryPosture posture) {
    switch (posture) {
        case SentryPosture::Attack: return score.Attack;
        case SentryPosture::Defense: return score.Defense;
        case SentryPosture::Move: return score.Move;
        default: return -1000000;
    }
}

void AddScore(PostureScore& score, const SentryPosture posture, const int delta) {
    switch (posture) {
        case SentryPosture::Attack: score.Attack += delta; break;
        case SentryPosture::Defense: score.Defense += delta; break;
        case SentryPosture::Move: score.Move += delta; break;
        default: break;
    }
}

bool IsRecoveryGoal(const std::uint8_t goal_id) noexcept {
    const std::uint8_t base_goal_id =
        goal_id >= LangYa::TeamedLocation::LocationCount
            ? static_cast<std::uint8_t>(goal_id - LangYa::TeamedLocation::LocationCount)
            : goal_id;
    return base_goal_id == LangYa::Recovery.ID;
}
}  // namespace

bool Application::HasRecentTarget() const {
    if (lastTargetSeenTime.time_since_epoch().count() == 0) return false;
    const int keep_ms = std::max(0, config.PostureSettings.TargetKeepMs);
    return (std::chrono::steady_clock::now() - lastTargetSeenTime) <= std::chrono::milliseconds(keep_ms);
}

AimSourceView Application::CurrentAimSource() const noexcept {
    return MakeAimSourceView(externalAimData);
}

const AimData& Application::CurrentAimData() const noexcept {
    return *CurrentAimSource().Active;
}

bool Application::AutoAimFreshAndValid() const noexcept {
    return AimFreshAndValid(*CurrentAimSource().AutoAim);
}

bool Application::CurrentAimFreshAndValid() const noexcept {
    return AimFreshAndValid(CurrentAimData());
}

bool Application::CurrentAimFreshOrLatched(
    const std::chrono::steady_clock::time_point now,
    const int hold_ms) const noexcept {
    return AimFreshOrLatchedRecently(
        CurrentAimData(),
        now,
        std::chrono::milliseconds(std::max(0, hold_ms)));
}

bool Application::CurrentAimTargetForAngles(
    const bool callback_seen,
    const std::chrono::steady_clock::time_point now,
    const int hold_ms,
    bool* fresh_target,
    bool* latched_target) const noexcept {
    return AimTargetForAngles(
        CurrentAimData(),
        callback_seen,
        config.AimDebugSettings.ReuseLatchedAnglesOnNoTarget,
        now,
        std::chrono::milliseconds(std::max(0, hold_ms)),
        fresh_target,
        latched_target);
}

bool Application::BuffAimTargetLocked() const noexcept {
    const auto source = CurrentAimSource();
    return AimBuffTargetLocked(*source.Buff, source.ExternalAimActive);
}

bool Application::BuffAimFreshAndFireReady() const noexcept {
    const auto source = CurrentAimSource();
    return AimBuffFireReady(*source.Buff, source.ExternalAimActive);
}

bool Application::OutpostAimFreshAndValid() const noexcept {
    return AimFreshAndValid(*CurrentAimSource().Outpost);
}

bool Application::OutpostAimFreshOrLatched(
    const std::chrono::steady_clock::time_point now,
    const int hold_ms) const noexcept {
    return AimFreshOrLatchedRecently(
        *CurrentAimSource().Outpost,
        now,
        std::chrono::milliseconds(std::max(0, hold_ms)));
}

bool Application::IsUnderFireRecent() const {
    if (lastDamageTime.time_since_epoch().count() == 0) return false;
    const int keep_sec = std::max(0, config.PostureSettings.DamageKeepSec);
    return (std::chrono::steady_clock::now() - lastDamageTime) <= std::chrono::seconds(keep_sec);
}

bool Application::IsUnderFireBurst() const {
    const auto now = std::chrono::steady_clock::now();
    const int hold_sec = std::max(0, config.PostureSettings.DamageBurstDefenseHoldSec);
    if (hold_sec > 0 && lastDamageBurstTime_.time_since_epoch().count() != 0 &&
        (now - lastDamageBurstTime_) <= std::chrono::seconds(hold_sec)) {
        return true;
    }

    return IsDamageBurst(
        config.PostureSettings.DamageBurstWindowMs,
        config.PostureSettings.DamageBurstThreshold);
}

bool Application::IsDamageBurst(const int window_ms, const int threshold) const {
    if (window_ms <= 0 || threshold <= 0) {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    std::uint32_t total_damage = 0;
    for (auto it = postureRecentDamageSamples_.rbegin(); it != postureRecentDamageSamples_.rend(); ++it) {
        if ((now - it->Time) > std::chrono::milliseconds(window_ms)) {
            break;
        }
        total_damage += it->Delta;
        if (total_damage >= static_cast<std::uint32_t>(threshold)) {
            return true;
        }
    }
    return false;
}

void Application::RecordDamageSample(const std::chrono::steady_clock::time_point now, const std::uint16_t damage) {
    if (damage == 0U) {
        return;
    }

    postureRecentDamageSamples_.push_back({now, damage});

    const int window_ms = std::max(0, config.PostureSettings.DamageBurstWindowMs);
    if (window_ms > 0) {
        const auto cutoff = now - std::chrono::milliseconds(window_ms);
        while (!postureRecentDamageSamples_.empty() && postureRecentDamageSamples_.front().Time < cutoff) {
            postureRecentDamageSamples_.pop_front();
        }
    } else if (postureRecentDamageSamples_.size() > 32U) {
        postureRecentDamageSamples_.pop_front();
    }

    const int threshold = std::max(0, config.PostureSettings.DamageBurstThreshold);
    if (window_ms <= 0 || threshold <= 0) {
        return;
    }

    std::uint32_t total_damage = 0;
    for (auto it = postureRecentDamageSamples_.rbegin(); it != postureRecentDamageSamples_.rend(); ++it) {
        if ((now - it->Time) > std::chrono::milliseconds(window_ms)) {
            break;
        }
        total_damage += it->Delta;
        if (total_damage >= static_cast<std::uint32_t>(threshold)) {
            lastDamageBurstTime_ = now;
            return;
        }
    }
}

TaskPostureIntentState Application::ResolveTaskPostureIntent(
    const std::chrono::steady_clock::time_point now) const {
    const auto owns_base_goal = [&](const std::uint8_t base_goal_id,
                                    const UnitTeam goal_team,
                                    const bool apply_team_offset) {
        if (!AreaManager::IsValidBaseGoalId(base_goal_id)) {
            return false;
        }
        const auto expected_id = ResolveGoalId(base_goal_id, goal_team, apply_team_offset);
        const auto expected_position = AreaManager::GoalPointByBaseId(base_goal_id, goal_team);
        return naviCommandGoal == expected_id &&
               naviGoalPosition.x == expected_position.x &&
               naviGoalPosition.y == expected_position.y;
    };
    const auto make = [](const TaskPostureIntent intent, const char* source, const bool owns_goal) {
        return TaskPostureIntentState{.Intent = intent, .Source = source, .OwnsCurrentGoal = owns_goal};
    };

    if (IsRecoveryGoal(naviCommandGoal)) {
        const auto& enhanced_recovery_move =
            config.TacticalSettings.EnhancedPosture.RecoveryMove;
        const bool respawn_suppressed =
            enhancedMoveRespawnSuppressUntil_.time_since_epoch().count() != 0 &&
            now < enhancedMoveRespawnSuppressUntil_;
        const bool recovery_traveling =
            !IsBaseGoalArrived(LangYa::Recovery.ID, team, true);
        const bool self_health_fresh =
            hasReceivedMyselfHealth_ &&
            lastMyselfHealthRxTime.time_since_epoch().count() != 0 &&
            now - lastMyselfHealthRxTime <= std::chrono::milliseconds(
                std::max(1, config.TaskSettings.OutpostConfirm.RefereeFreshTimeoutMs));
        auto referee_timer = postureRefereeTimer_;
        if (referee_timer.HasInfo3 &&
            referee_timer.AgeMeasuredAt.time_since_epoch().count() != 0) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - referee_timer.AgeMeasuredAt).count();
            const auto current_age_ms = static_cast<std::uint64_t>(referee_timer.AgeMs) +
                static_cast<std::uint64_t>(std::max<decltype(elapsed_ms)>(elapsed_ms, 0));
            referee_timer.Fresh = current_age_ms <= static_cast<std::uint64_t>(
                std::max(0, config.PostureSettings.RefereeInfo3FreshMs));
        }
        const bool can_request_enhanced_move = ShouldRequestRecoveryEnhancedMove(
            strategyMode_ == StrategyMode::Regional,
            enhanced_recovery_move.Enable,
            respawn_suppressed,
            enhancedRecoveryMoveUnavailable_,
            recovery_traveling,
            self_health_fresh,
            myselfHealth,
            enhanced_recovery_move.HealthThresholdHp,
            postureManager_.Runtime(),
            referee_timer);
        if (can_request_enhanced_move) {
            return make(TaskPostureIntent::RecoveryEnhancedMove, "recovery_enhanced_move", true);
        }
        return make(TaskPostureIntent::HardMove, "recovery", true);
    }
    if (aimMode == AimMode::Buff) {
        return make(TaskPostureIntent::HardMove, "buff", false);
    }
    const auto outpost_goal_id = ResolveGoalId(LangYa::BuffOutpost.ID, team, true);
    const auto outpost_goal_position = AreaManager::GoalPointByBaseId(LangYa::BuffOutpost.ID, team);
    const bool owns_outpost_goal = naviCommandGoal == outpost_goal_id &&
        naviGoalPosition.x == outpost_goal_position.x &&
        naviGoalPosition.y == outpost_goal_position.y;
    const bool outpost_task_active = aimMode == AimMode::Outpost ||
        outpostVisualScoutNavigationActive_ ||
        outpostArmorInterruptActive_ ||
        (outpostPostArmorFaceSearchUntil_.time_since_epoch().count() != 0 &&
         now < outpostPostArmorFaceSearchUntil_);
    if (outpost_task_active && owns_outpost_goal) {
        return make(
            IsBaseGoalArrived(LangYa::BuffOutpost.ID, team, true)
                ? TaskPostureIntent::SoftArrived
                : TaskPostureIntent::SoftTransit,
            "outpost",
            true);
    }

    const auto protect_outpost_goal = Area::ProtectOutpost(team);
    const auto protect_outpost_goal_id = ResolveGoalId(LangYa::ProtectOutpost.ID, team, true);
    const bool owns_protect_outpost_goal = naviCommandGoal == protect_outpost_goal_id &&
        naviGoalPosition.x == protect_outpost_goal.x &&
        naviGoalPosition.y == protect_outpost_goal.y;
    if (owns_protect_outpost_goal) {
        if (protectOutpostState_.Phase == ProtectOutpostPhase::SearchHold) {
            return make(TaskPostureIntent::SoftArrived, "protect_outpost_search_hold", true);
        }
        if (protectOutpostState_.Phase == ProtectOutpostPhase::Travel) {
            return make(TaskPostureIntent::SoftTransit, "protect_outpost_travel", true);
        }
    }

    if (protectHeroActive_ && lastDecisionIntent_.Reason == DecisionReason::ProtectHero) {
        const auto base_goal_id = AreaManager::IsValidBaseGoalId(config.TacticalSettings.ProtectHero.GoalBaseId)
            ? config.TacticalSettings.ProtectHero.GoalBaseId
            : LangYa::Highland.ID;
        if (owns_base_goal(base_goal_id, team, true)) {
            const bool arrived = IsBaseGoalArrived(base_goal_id, team, true);
            if (!arrived) {
                return make(TaskPostureIntent::SoftTransit, "protect_hero", true);
            }
            const auto& enhanced_defense = config.TacticalSettings.ProtectHero.EnhancedDefense;
            return make(
                ResolveProtectHeroHoldIntent(
                    enhanced_defense.Enable,
                    IsDamageBurst(
                        enhanced_defense.DamageWindowMs,
                        enhanced_defense.DamageThresholdHp),
                    protectHeroEnhancedDefenseUnavailable_),
                "protect_hero",
                true);
        }
    }

    if (IsUnderFireBurst()) {
        return make(TaskPostureIntent::HardDefense, "damage_burst", false);
    }

    const bool regional_defense_active =
        regionalDefenseSearchKind_ != RegionalDefenseSearchKind::None &&
        DecisionLayerForReason(lastDecisionIntent_.Reason) == DecisionLayer::RegionalDefense;
    if (regional_defense_active && owns_base_goal(regionalDefenseSearchBaseGoal_, team, true)) {
        return make(
            IsBaseGoalArrived(regionalDefenseSearchBaseGoal_, team, true)
                ? TaskPostureIntent::SoftArrived
                : TaskPostureIntent::SoftTransit,
            "regional_defense",
            true);
    }

    if (lastDecisionIntent_.Reason == DecisionReason::SpecialPatrol &&
        owns_base_goal(lastDecisionIntent_.BaseGoalId, team, true)) {
        const bool special_hold = specialPatrolHoldActive_ &&
            specialPatrolHoldBaseGoal_ == lastDecisionIntent_.BaseGoalId;
        return make(
            special_hold ? TaskPostureIntent::SoftArrived : TaskPostureIntent::SoftTransit,
            special_hold ? "special_patrol_hold" : "special_patrol_travel",
            true);
    }

    const auto& area_task = areaManager_.RegionalAreaTask();
    if (area_task.Active && area_task.Origin == RegionalAreaTaskOrigin::DefaultPolicy &&
        owns_base_goal(area_task.CurrentBaseGoal, area_task.GoalTeam, area_task.ApplyTeamOffset)) {
        const auto hint = ResolveRegionalAreaTaskPostureHint(area_task);
        return make(
            hint == RegionalAreaTaskPostureHint::ArrivedHold
                ? TaskPostureIntent::SoftArrived
                : TaskPostureIntent::SoftTransit,
            hint == RegionalAreaTaskPostureHint::ArrivedHold ? "default_arrived_hold" : "default_transit",
            true);
    }

    return {};
}

SentryPosture Application::SelectDesiredPosture(const bool has_target) const {
    if (!config.PostureSettings.Enable) {
        return SentryPosture::Unknown;
    }

    if (aimMode == AimMode::Buff) {
        return SentryPosture::Move;
    }
    if (IsRecoveryGoal(naviCommandGoal)) {
        return SentryPosture::Move;
    }

    const auto& runtime = postureManager_.Runtime();
    const auto now = std::chrono::steady_clock::now();
    const int target_keep_ms = std::max(0, config.PostureSettings.TargetKeepMs);
    const bool outpost_target_recent = OutpostAimFreshOrLatched(now, target_keep_ms);
    const bool outpost_at_buff_outpost =
        IsBaseGoalArrived(LangYa::BuffOutpost.ID, team, true);
    const bool outpost_attack_ready =
        aimMode == AimMode::Outpost &&
        targetArmor.Type == ArmorType::Outpost &&
        outpost_at_buff_outpost &&
        outpost_target_recent;
    const bool outpost_post_armor_face_search_active =
        outpostPostArmorFaceSearchUntil_.time_since_epoch().count() != 0 &&
        now < outpostPostArmorFaceSearchUntil_;
    const bool outpost_task_active =
        aimMode == AimMode::Outpost ||
        outpostVisualScoutNavigationActive_ ||
        outpostArmorInterruptActive_ ||
        outpost_post_armor_face_search_active;
    PostureScore score{};

    // 1) 基础策略加权
    switch (strategyMode_) {
        case StrategyMode::Regional:
            score.Attack += 2;
            score.Move += 1;
            break;
        case StrategyMode::LeagueSimple:
            score.Attack += 1;
            score.Move += 3;
            break;
    }

    // 2) 瞄准模式加权
    if (aimMode == AimMode::RotateScan) {
        score.Move += 4;
    } else if (aimMode == AimMode::Buff || aimMode == AimMode::Outpost) {
        score.Attack += 3;
        score.Move += 1;
    } else {
        score.Attack += 2;
    }

    // 3) 目标可见性（带时间窗）
    if (has_target) {
        score.Attack += 6;
        score.Move -= 1;
    } else {
        score.Move += 1;
    }

    // 4) 资源与风险态势
    const bool low_energy =
        teamBuff.RemainingEnergy == 0b10000 ||
        teamBuff.RemainingEnergy == 0b00000;
    const bool very_low_health = myselfHealth <= static_cast<std::uint16_t>(config.PostureSettings.VeryLowHealthThreshold);
    const bool low_health = myselfHealth <= static_cast<std::uint16_t>(config.PostureSettings.LowHealthThreshold);
    const bool low_ammo = ammoLeft <= static_cast<std::uint16_t>(config.PostureSettings.LowAmmoThreshold);
    const bool under_fire = IsUnderFireRecent();
    const bool under_fire_burst = IsUnderFireBurst();

    if (under_fire_burst) {
        return SentryPosture::Defense;
    }
    if (outpost_attack_ready) {
        if (very_low_health || low_health) {
            return SentryPosture::Defense;
        }
        return SentryPosture::Attack;
    }
    if (outpost_task_active && (very_low_health || low_health)) {
        return SentryPosture::Defense;
    }
    if (outpost_task_active && outpost_at_buff_outpost) {
        return SentryPosture::Attack;
    }
    if (outpost_task_active) {
        return SentryPosture::Move;
    }

    if (low_energy) {
        score.Defense += 6;
        score.Move += 3;
        score.Attack -= 4;
    }
    if (very_low_health) {
        score.Defense += 7;
        score.Attack -= 4;
    } else if (low_health) {
        score.Defense += 4;
        score.Attack -= 2;
    }
    if (low_ammo) {
        score.Defense += 2;
        score.Move += 2;
        score.Attack -= 2;
    }
    if (under_fire) {
        score.Defense += 5;
        score.Move += 1;
        score.Attack -= 2;
    }

    // 5) 回读过期时降低激进性，优先保守/机动
    if (runtime.FeedbackStale) {
        score.Defense += 3;
        score.Move += 3;
        score.Attack -= 3;
    }

    // 6) 单姿态累计过久时，惩罚当前姿态，降低 3 分钟惩罚风险
    if (IsValidPosture(runtime.Current.Base)) {
        const auto idx = ToPostureValue(runtime.Current.Base);
        if (idx > 0U) {
            if (runtime.Degraded[idx]) {
                AddScore(score, runtime.Current.Base, -3);
            } else if (runtime.AccumSec[idx] >= static_cast<double>(config.PostureSettings.EarlyRotateSec)) {
                AddScore(score, runtime.Current.Base, -2);
            }
        }
    }

    // 7) pending 期间轻微偏向 pending，减少频繁改口
    if (runtime.HasPending && IsValidPosture(runtime.Pending.Base)) {
        AddScore(score, runtime.Pending.Base, 2);
    }

    // 8) 裁判 sentry_info_3 新鲜时，按剩余时长降低接近弱化姿态的候选分数。
    // 强化状态只偏向保持当前攻/防/移类别，不在这里自动下发 4/5/6 命令。
    if (runtime.UsingRefereeTimer) {
        const auto& remaining = runtime.RefereeEnhancedPosture
            ? runtime.RefereeEnhancedRemainingSec
            : runtime.RefereeRemainingSec;
        const int warn_sec = std::max(0, config.PostureSettings.RefereeRemainWarnSec);
        const int max_penalty = std::max(0, config.PostureSettings.RefereeRemainPenalty);
        const int zero_penalty = std::max(max_penalty, config.PostureSettings.RefereeZeroRemainPenalty);

        for (const auto posture : {SentryPosture::Attack, SentryPosture::Defense, SentryPosture::Move}) {
            const auto idx = ToPostureValue(posture);
            const int seconds = remaining[idx];
            if (seconds == 0) {
                AddScore(score, posture, -zero_penalty);
            } else if (warn_sec > 0 && seconds <= warn_sec && max_penalty > 0) {
                const int penalty = (max_penalty * (warn_sec - seconds + 1) + warn_sec - 1) / warn_sec;
                AddScore(score, posture, -penalty);
            }
        }

        if (runtime.RefereeEnhancedPosture && IsValidPosture(runtime.Current.Base)) {
            AddScore(
                score,
                runtime.Current.Base,
                std::max(0, config.PostureSettings.EnhancedCurrentPostureBonus));
        }
    }

    SentryPosture best = SentryPosture::Attack;
    int best_score = score.Attack;
    if (score.Defense > best_score) {
        best = SentryPosture::Defense;
        best_score = score.Defense;
    }
    if (score.Move > best_score) {
        best = SentryPosture::Move;
        best_score = score.Move;
    }

    // 9) 分差迟滞：分差不够大时保持当前姿态，避免抖动
    const int hysteresis = std::max(0, config.PostureSettings.ScoreHysteresis);
    if (IsValidPosture(runtime.Current.Base)) {
        const int current_score = GetScore(score, runtime.Current.Base);
        if (best != runtime.Current.Base && (best_score - current_score) <= hysteresis) {
            best = runtime.Current.Base;
        }
    }

    return best;
}

void Application::UpdatePostureCommand(const bool has_target) {
    postureCommand = 0;
    if (!config.PostureSettings.Enable) return;

    const auto now = std::chrono::steady_clock::now();
    if (!postureHealthInitialized_) {
        postureLastHealth_ = myselfHealth;
        postureHealthInitialized_ = true;
    } else if (myselfHealth < postureLastHealth_) {
        RecordDamageSample(now, static_cast<std::uint16_t>(postureLastHealth_ - myselfHealth));
        lastDamageTime = now;
        postureLastHealth_ = myselfHealth;
    } else {
        postureLastHealth_ = myselfHealth;
    }

    const bool has_visible_selected_target =
        ShouldUseVisibleExternalAimForAttackPosture(HasFreshSelectedExternalAimTarget());
    const bool has_target_recent = has_target || has_visible_selected_target || HasRecentTarget();
    const auto scored_desired = SelectDesiredPosture(has_target_recent);
    auto intent = ResolveTaskPostureIntent(now);
    const bool navi_move_override = ShouldRequestMovePostureWhenNaviFalse(
        config.NaviControlSettings,
        hasReceivedNaviIsRotate_,
        lastNaviIsRotateRxTime_,
        now,
        naviIsRotate);
    if (navi_move_override) {
        intent = {TaskPostureIntent::HardMove, "navi_should_rotate_false", false};
    }
    auto referee_timer = postureRefereeTimer_;
    const auto fresh_limit_ms = std::max(0, config.PostureSettings.RefereeInfo3FreshMs);
    if (referee_timer.HasInfo3 && referee_timer.AgeMeasuredAt.time_since_epoch().count() != 0) {
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - referee_timer.AgeMeasuredAt).count();
        const auto current_age_ms = static_cast<std::uint64_t>(referee_timer.AgeMs) +
            static_cast<std::uint64_t>(std::max<decltype(elapsed_ms)>(elapsed_ms, 0));
        referee_timer.Fresh = current_age_ms <= static_cast<std::uint64_t>(fresh_limit_ms);
    }
    referee_timer.EnhancedContradictionGraceMs = std::max(
        0, config.TacticalSettings.EnhancedPosture.ContradictionGraceMs);

    TransitPostureContext transit_context;
    transit_context.Enabled = config.PostureSettings.DynamicTransitReserveEnable;
    transit_context.AllowAttackDuringTransit = has_target_recent;
    transit_context.NominalSpeedMps = config.PostureSettings.TransitNominalSpeedMps;
    transit_context.SafetyFactor = config.PostureSettings.TransitSafetyFactor;
    transit_context.ArrivalBufferSec = config.PostureSettings.TransitArrivalBufferSec;
    transit_context.MinReserveSec = config.PostureSettings.TransitMinReserveSec;
    transit_context.MaxReserveSec = config.PostureSettings.TransitMaxReserveSec;
    transit_context.FallbackReserveSec = config.PostureSettings.TransitFallbackReserveSec;

    const auto self_position = GetSentryPositionState(now);
    const bool has_active_goal = naviCommandGoal != 0U &&
        naviGoalPosition.x > 0U && naviGoalPosition.y > 0U;
    if (has_active_goal && self_position.HasPosition && self_position.Fresh) {
        transit_context.HasFreshDistance = true;
        transit_context.DistanceCm = std::hypot(
            static_cast<double>(naviGoalPosition.x) - static_cast<double>(self_position.X),
            static_cast<double>(naviGoalPosition.y) - static_cast<double>(self_position.Y));
    }
    const bool velocity_timestamp_valid =
        lastNaviVelocityRxTime_.time_since_epoch().count() != 0 &&
        now >= lastNaviVelocityRxTime_ &&
        now - lastNaviVelocityRxTime_ <= std::chrono::milliseconds(std::max(
            1, config.PostureSettings.TransitVelocityFreshMs));
    if (velocity_timestamp_valid) {
        transit_context.HasFreshVelocity = true;
        transit_context.VelocityMps = std::hypot(
            static_cast<double>(naviVelocityInput.X),
            static_cast<double>(naviVelocityInput.Y)) * 0.025;
    }
    const auto task_request = ResolveTaskPostureRequest(
        intent.Intent,
        scored_desired,
        postureManager_.Runtime(),
        transit_context,
        referee_timer);
    auto desired = task_request.Mode.Base;
    PostureMode requested = task_request.Mode;
    auto request_policy = task_request.Policy;
    if (outpostEngagementDecision_.Intent.has_value()) {
        requested = *outpostEngagementDecision_.Intent;
        request_policy = PostureRequestPolicy::OutpostLock();
    }
    if (ShouldForceDamageBurstDefense(
            IsRecoveryGoal(naviCommandGoal),
            IsUnderFireBurst(),
            intent.Intent,
            requested.Enhanced)) {
        requested = {SentryPosture::Defense, false};
        desired = SentryPosture::Defense;
        request_policy = PostureRequestPolicy::RequiredPosture();
        intent = {TaskPostureIntent::HardDefense, "damage_burst", false};
    }
    const bool posture_feedback_fresh = IsPostureFeedbackFresh(
        hasReceivedPostureState_,
        lastPostureStateRxTime_,
        std::max(1, config.PostureSettings.FeedbackFreshMs),
        now);
    const auto decision = postureManager_.Tick(
        now,
        requested,
        {postureState, referee_timer.Enhanced, posture_feedback_fresh, referee_timer.Fresh, lastPostureStateRxTime_},
        referee_timer,
        request_policy);
    if (outpostEngagementDecision_.EnhancedPending && std::string_view(decision.Reason) == "pending_preserved") {
        outpostEngagementLock_.MarkEnhancedUnavailable();
    }
    const bool protect_hero_defense_intent =
        intent.Intent == TaskPostureIntent::ProtectHeroDefenseHold ||
        intent.Intent == TaskPostureIntent::ProtectHeroEnhancedDefense;
    if (!protect_hero_defense_intent) {
        protectHeroEnhancedDefenseUnavailable_ = false;
    } else if (intent.Intent == TaskPostureIntent::ProtectHeroEnhancedDefense &&
               std::string_view(decision.Reason) == "pending_preserved") {
        protectHeroEnhancedDefenseUnavailable_ = true;
    }
    if (!IsRecoveryGoal(naviCommandGoal)) {
        enhancedRecoveryMoveUnavailable_ = false;
    } else if (intent.Intent == TaskPostureIntent::RecoveryEnhancedMove &&
               std::string_view(decision.Reason) == "pending_preserved") {
        enhancedRecoveryMoveUnavailable_ = true;
    }
    postureCommand = decision.Command;
    const auto& runtime = postureManager_.Runtime();
    const auto respawn_suppress_remaining_ms =
        enhancedMoveRespawnSuppressUntil_.time_since_epoch().count() != 0 &&
        now < enhancedMoveRespawnSuppressUntil_
            ? std::chrono::duration_cast<std::chrono::milliseconds>(
                  enhancedMoveRespawnSuppressUntil_ - now).count()
            : 0;

    const bool desired_changed = desired != postureLastDesired_;
    const bool reason_changed = postureLastReason_ != decision.Reason;
    postureLastDesired_ = desired;
    postureLastReason_ = decision.Reason;
    postureTaskIntent_ = intent;

    if (LoggerPtr && (decision.Sent || desired_changed || reason_changed)) {
        LoggerPtr->Info(
            "[Posture] cmd={} scored={} desired={} requested_enhanced={} current={} current_enhanced={} pending={} pending_enhanced={} pending_priority={} pending_source={} has_target_recent={} task_intent={} task_source={} task_owns_goal={} transit_reserve_sec={} transit_distance_cm={} transit_velocity_mps={} transit_attack_allowed={} navi_move_override={} under_fire={} under_fire_burst={} feedback_stale={} referee_timer={} enhanced={} enhanced_quarantined={} recovery_enhanced_unavailable={} respawn_suppress_remaining_ms={} reason={}",
            static_cast<int>(postureCommand),
            PostureToString(scored_desired),
            PostureToString(desired),
            requested.Enhanced ? 1 : 0,
            PostureToString(runtime.Current.Base),
            runtime.Current.Enhanced ? 1 : 0,
            PostureToString(runtime.Pending.Base),
            runtime.Pending.Enhanced ? 1 : 0,
            PostureRequestPriorityToString(runtime.PendingPriority),
            runtime.PendingSource,
            has_target_recent ? 1 : 0,
            TaskPostureIntentToString(intent.Intent),
            intent.Source,
            intent.OwnsCurrentGoal ? 1 : 0,
            ComputeTransitMoveReserveSec(transit_context),
            transit_context.HasFreshDistance ? transit_context.DistanceCm : -1.0,
            transit_context.HasFreshVelocity ? transit_context.VelocityMps : -1.0,
            transit_context.AllowAttackDuringTransit ? 1 : 0,
            navi_move_override ? 1 : 0,
            IsUnderFireRecent() ? 1 : 0,
            IsUnderFireBurst() ? 1 : 0,
            runtime.FeedbackStale ? 1 : 0,
            runtime.UsingRefereeTimer ? 1 : 0,
            runtime.RefereeEnhancedPosture ? 1 : 0,
            runtime.EnhancedFeedbackQuarantined ? 1 : 0,
            enhancedRecoveryMoveUnavailable_ ? 1 : 0,
            respawn_suppress_remaining_ms,
            decision.Reason);
    }
}

}  // namespace BehaviorTree
