#pragma once

#include "Application.hpp"

namespace BehaviorTree {

class AppSyncActionNode : public BT::SyncActionNode {
public:
    AppSyncActionNode(const std::string& name,
                      const BT::NodeConfig& config,
                      Application* app)
        : BT::SyncActionNode(name, config), app_(app) {}

protected:
    Application* app_;
};

class AppConditionNode : public BT::ConditionNode {
public:
    AppConditionNode(const std::string& name,
                     const BT::NodeConfig& config,
                     Application* app)
        : BT::ConditionNode(name, config), app_(app) {}

protected:
    Application* app_;
};

class UpdateGlobalData : public AppSyncActionNode {
public:
    UpdateGlobalData(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppSyncActionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        app_->ResetTickBlackboard();
        app_->UpdateBlackBoard();

        const auto global_board = app_->GetGlobalBlackboard();
        const auto tick_board = app_->GetTickBlackboard();
        global_board->set("NowTime", app_->ElapsedSeconds());
        global_board->set("StrategyMode", static_cast<std::uint8_t>(app_->GetStrategyMode()));
        if (tick_board) {
            tick_board->set("NowTime", app_->ElapsedSeconds());
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class SelectAimModeNode : public AppSyncActionNode {
public:
    SelectAimModeNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppSyncActionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        app_->SetAimMode();
        app_->CheckDebug();
        app_->GetGlobalBlackboard()->set("AimMode", static_cast<std::uint8_t>(app_->GetAimMode()));
        return BT::NodeStatus::SUCCESS;
    }
};

class SelectStrategyModeNode : public AppSyncActionNode {
public:
    SelectStrategyModeNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppSyncActionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        const auto global_board = app_->GetGlobalBlackboard();
        if (!global_board) {
            return BT::NodeStatus::FAILURE;
        }
        const int now_time = app_->ElapsedSeconds();

        std::uint16_t self_outpost_health = 0;
        std::uint16_t enemy_outpost_health = 0;
        std::uint16_t self_health = 0;
        std::uint16_t time_left = 0;
        BuffType team_buff{};

        (void)global_board->get("SelfOutpostHealth", self_outpost_health);
        (void)global_board->get("EnemyOutpostHealth", enemy_outpost_health);
        (void)global_board->get("SelfHealth", self_health);
        (void)global_board->get("TimeLeft", time_left);
        (void)global_board->get("TeamBuff", team_buff);

        const StrategyMode current_strategy = app_->GetStrategyMode();
        StrategyMode next_strategy = current_strategy;
        const bool low_resource = (self_health < 100 || time_left <= 120 ||
                                   team_buff.RemainingEnergy == 0b10000 ||
                                   team_buff.RemainingEnergy == 0b00000);
        const bool sentry_window = (enemy_outpost_health > 0 && self_outpost_health > 100 && now_time < 55);

        // 开局保留 config 给定的初始策略，随后再进入动态切换
        if (now_time < 10) {
            next_strategy = current_strategy;
        } else if (low_resource) {
            next_strategy = StrategyMode::Protected;
        } else if (current_strategy == StrategyMode::NaviTest && now_time < 340) {
            next_strategy = StrategyMode::NaviTest;
        } else if (sentry_window) {
            next_strategy = StrategyMode::HitSentry;
        } else {
            next_strategy = StrategyMode::HitHero;
        }

        app_->SetStrategyMode(next_strategy);
        global_board->set("StrategyMode", static_cast<std::uint8_t>(next_strategy));
        return BT::NodeStatus::SUCCESS;
    }
};

class CheckNeedRecoveryNode : public AppConditionNode {
public:
    CheckNeedRecoveryNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppConditionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        return app_->CheckPositionRecovery() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IsAimModeBuffNode : public AppConditionNode {
public:
    IsAimModeBuffNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppConditionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        return app_->GetAimMode() == AimMode::Buff ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IsAimModeOutpostNode : public AppConditionNode {
public:
    IsAimModeOutpostNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppConditionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        return app_->GetAimMode() == AimMode::Outpost ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IsStrategyHitSentryNode : public AppConditionNode {
public:
    IsStrategyHitSentryNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppConditionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        return app_->GetStrategyMode() == StrategyMode::HitSentry ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IsStrategyHitHeroNode : public AppConditionNode {
public:
    IsStrategyHitHeroNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppConditionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        return app_->GetStrategyMode() == StrategyMode::HitHero ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IsStrategyProtectedNode : public AppConditionNode {
public:
    IsStrategyProtectedNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppConditionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        return app_->GetStrategyMode() == StrategyMode::Protected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IsStrategyNaviTestNode : public AppConditionNode {
public:
    IsStrategyNaviTestNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppConditionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        return app_->GetStrategyMode() == StrategyMode::NaviTest ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class ExecuteHitSentryStrategyNode : public AppSyncActionNode {
public:
    ExecuteHitSentryStrategyNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppSyncActionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        app_->SetPositionHitSentry();
        return BT::NodeStatus::SUCCESS;
    }
};

class ExecuteHitHeroStrategyNode : public AppSyncActionNode {
public:
    ExecuteHitHeroStrategyNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppSyncActionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        app_->SetPositionHitHero();
        return BT::NodeStatus::SUCCESS;
    }
};

class ExecuteProtectedStrategyNode : public AppSyncActionNode {
public:
    ExecuteProtectedStrategyNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppSyncActionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        app_->SetPositionProtect();
        return BT::NodeStatus::SUCCESS;
    }
};

class ExecuteNaviTestStrategyNode : public AppSyncActionNode {
public:
    ExecuteNaviTestStrategyNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppSyncActionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        app_->SetPositionNaviTest();
        return BT::NodeStatus::SUCCESS;
    }
};

class PreprocessDataNode : public AppSyncActionNode {
public:
    PreprocessDataNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppSyncActionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        app_->ProcessData();
        const auto tick_board = app_->GetTickBlackboard();
        if (tick_board) {
            tick_board->set("NowTime", app_->ElapsedSeconds());
            tick_board->set("HitableTargets", app_->GetHitableTargetsCopy());
            tick_board->set("ReliableEnemyPositions", app_->GetReliableEnemyPositionsCopy());
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class SelectAimTargetNode : public AppSyncActionNode {
public:
    SelectAimTargetNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppSyncActionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        app_->SetAimTarget();
        const auto tick_board = app_->GetTickBlackboard();
        if (tick_board) {
            tick_board->set("TargetArmor", app_->GetTargetArmorCopy());
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class PublishAllNode : public AppSyncActionNode {
public:
    PublishAllNode(const std::string& name, const BT::NodeConfig& config, Application* app)
        : AppSyncActionNode(name, config, app) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        app_->PublishTogether();
        app_->PrintMessageAll();
        return BT::NodeStatus::SUCCESS;
    }
};

} // namespace BehaviorTree
