// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include "../include/BTNodes.hpp"
#include <cstdlib>

using namespace LangYa;

namespace BehaviorTree {

namespace {
bool EnvEnabled(const char* key, bool default_value) {
    const char* v = std::getenv(key);
    if (!v || !*v) return default_value;
    const std::string s(v);
    return s == "1" || s == "true" || s == "TRUE" || s == "on" || s == "ON";
}

std::string EnvString(const char* key, const char* default_value) {
    const char* v = std::getenv(key);
    if (!v || !*v) {
        return std::string(default_value);
    }
    return std::string(v);
}

int EnvInt(const char* key, int default_value) {
    const char* v = std::getenv(key);
    if (!v || !*v) {
        return default_value;
    }
    try {
        return std::stoi(v);
    } catch (...) {
        return default_value;
    }
}
} // namespace

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

        // 稳定性优先：默认关闭 BT 调试附加器，避免在实机环境引入额外崩溃面。
        // 如需开启，导出环境变量：
        //   BT_ENABLE_COUT_LOGGER=1
        //   BT_ENABLE_FILE_LOGGER=1
        //   BT_ENABLE_MINITRACE=1
        //   BT_ENABLE_GROOT=1
        // 可选参数：
        //   BT_FILE_LOG_PATH=/abs/path/trace.btlog
        //   BT_MINITRACE_FILE=/abs/path/trace.json
        //   BT_GROOT_PORT=1667
        if (EnvEnabled("BT_ENABLE_COUT_LOGGER", false)) {
            try {
                btCoutLogger_ = std::make_unique<BT::StdCoutLogger>(BTree);
                LoggerPtr->Info("BT cout logger enabled.");
            } catch (const std::exception& ex) {
                LoggerPtr->Warning("BT cout logger init failed: {}", ex.what());
            }
        }
        if (EnvEnabled("BT_ENABLE_FILE_LOGGER", false)) {
            try {
                const auto file_log_path = EnvString("BT_FILE_LOG_PATH", "behavior_tree_trace.btlog");
                btFileLogger_ = std::make_unique<BT::FileLogger2>(BTree, file_log_path);
                LoggerPtr->Info("BT file logger enabled: {}", file_log_path);
            } catch (const std::exception& ex) {
                LoggerPtr->Warning("BT file logger init failed: {}", ex.what());
            }
        }
        if (EnvEnabled("BT_ENABLE_MINITRACE", false)) {
            try {
                const auto minitrace_file = EnvString("BT_MINITRACE_FILE", "behavior_tree_trace.json");
                btMinitraceLogger_ = std::make_unique<BT::MinitraceLogger>(BTree, minitrace_file.c_str());
                LoggerPtr->Info("BT minitrace logger enabled: {}", minitrace_file);
            } catch (const std::exception& ex) {
                LoggerPtr->Warning("BT minitrace logger init failed: {}", ex.what());
            }
        }
        if (EnvEnabled("BT_ENABLE_GROOT", false)) {
            try {
                const int groot_port = EnvInt("BT_GROOT_PORT", 1667);
                btGrootPublisher_ = std::make_unique<BT::Groot2Publisher>(BTree, static_cast<uint16_t>(groot_port));
                LoggerPtr->Info("BT Groot publisher enabled on port {}.", groot_port);
            } catch (const std::exception& ex) {
                LoggerPtr->Warning("BT Groot publisher init failed: {}", ex.what());
            }
        }

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
        REGISTER_APP_NODE(HardLayerNode, "Hard");
        REGISTER_APP_NODE(DefaultLayerNode, "Default");
        REGISTER_APP_NODE(TaskLayerNode, "Task");
        REGISTER_APP_NODE(TacticalLayerNode, "Tactical");
        REGISTER_APP_NODE(FinalizerLayerNode, "Finalizer");
        REGISTER_APP_NODE(IsAimModeBuffNode, "IsAimModeBuff");
        REGISTER_APP_NODE(IsAimModeOutpostNode, "IsAimModeOutpost");
        REGISTER_APP_NODE(PreprocessDataNode, "PreprocessData");
        REGISTER_APP_NODE(SelectAimTargetNode, "SelectAimTarget");
        REGISTER_APP_NODE(SelectPostureNode, "SelectPosture");
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
