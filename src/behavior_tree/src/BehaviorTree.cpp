#include "../include/Application.hpp"
#include "../include/BTNodes.hpp"

using namespace LangYa;

namespace BehaviorTree {

bool Application::LoadBehaviorTree() noexcept {
    try {
        if (!GlobalBlackboard_) {
            GlobalBlackboard_ = BT::Blackboard::create();
        }
        if (!TickBlackboard_) {
            TickBlackboard_ = BT::Blackboard::create();
        }
        GlobalBlackboard_->set("TickBlackboard", TickBlackboard_);

        BTree = Factory.createTreeFromFile(behavior_tree_file_, GlobalBlackboard_);

        btFileLogger_ = std::make_unique<BT::FileLogger2>(BTree, "behavior_tree_trace.fbl");
        btGrootPublisher_ = std::make_unique<BT::Groot2Publisher>(BTree, 1667);

        return true;
    }
    catch (const std::exception& ex) {
        LoggerPtr->Error("BehaviorTree Script Load Failed, location: {}", behavior_tree_file_);
        LoggerPtr->Error("Error Message: {}", ex.what());
        return false;
    }
}

/**
 * @brief 注册行为树节点 \n
 */
bool Application::RegisterTreeNodes() {
    try {
#define REGISTER_APP_NODE(NodeType, NodeID)                                                 \
    Factory.registerBuilder<NodeType>(                                                      \
        NodeID,                                                                             \
        [this](const std::string& name, const BT::NodeConfig& config) {                    \
            return std::make_unique<NodeType>(name, config, this);                         \
        })

        REGISTER_APP_NODE(UpdateGlobalData, "UpdateGlobalData");
        REGISTER_APP_NODE(SelectAimModeNode, "SelectAimMode");
        REGISTER_APP_NODE(SelectStrategyModeNode, "SelectStrategyMode");
        REGISTER_APP_NODE(CheckNeedRecoveryNode, "CheckNeedRecovery");
        REGISTER_APP_NODE(IsAimModeBuffNode, "IsAimModeBuff");
        REGISTER_APP_NODE(IsAimModeOutpostNode, "IsAimModeOutpost");
        REGISTER_APP_NODE(IsStrategyHitSentryNode, "IsStrategyHitSentry");
        REGISTER_APP_NODE(IsStrategyHitHeroNode, "IsStrategyHitHero");
        REGISTER_APP_NODE(IsStrategyProtectedNode, "IsStrategyProtected");
        REGISTER_APP_NODE(IsStrategyNaviTestNode, "IsStrategyNaviTest");
        REGISTER_APP_NODE(ExecuteHitSentryStrategyNode, "ExecuteHitSentryStrategy");
        REGISTER_APP_NODE(ExecuteHitHeroStrategyNode, "ExecuteHitHeroStrategy");
        REGISTER_APP_NODE(ExecuteProtectedStrategyNode, "ExecuteProtectedStrategy");
        REGISTER_APP_NODE(ExecuteNaviTestStrategyNode, "ExecuteNaviTestStrategy");
        REGISTER_APP_NODE(PreprocessDataNode, "PreprocessData");
        REGISTER_APP_NODE(SelectAimTargetNode, "SelectAimTarget");
        REGISTER_APP_NODE(PublishAllNode, "PublishAll");

#undef REGISTER_APP_NODE

        // 保留旧节点注册，保证旧 XML 不会因为重构直接失效
        Factory.registerNodeType<SetAimTargetFromAim>("SetAimTargetFromAim");
        Factory.registerNodeType<SetNaviPosition>("SetNaviPosition");

        return true;
    }
    catch (const std::exception& ex) {
        LoggerPtr->Error("RegisterTreeNodes Failed: {}", ex.what());
        return false;
    }
}

} // namespace BehaviorTree
