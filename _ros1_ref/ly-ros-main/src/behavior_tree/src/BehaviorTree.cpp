#include "../include/Application.hpp"

using namespace LangYa;

namespace BehaviorTree {
    bool Application::LoadBehaviorTree() noexcept {
        try {
            BlackBoard = BT::Blackboard::create();
            BTree = Factory.createTreeFromFile(behavior_tree_file, BlackBoard);
            return true;
        }
        catch (const std::exception &ex) {
            LoggerPtr->Error("BehaviorTree Script Load Failed, location: {}", behavior_tree_file);
            LoggerPtr->Error("Error Message: {}", ex.what());
            return false;
        }
    }

    /**
     * @brief 注册行为树节点 \n
     */
    bool Application::RegisterTreeNodes() {

#define REGISTER_NODES(NodeName) Factory.registerNodeType<NodeName>(#NodeName)

        // REGISTER_NODES(GoToPosition);
        // REGISTER_NODES(SetAimTarget);
        // REGISTER_NODES(SetCommandInterval);
        // REGISTER_NODES(CheckCommandInterval);
        // REGISTER_NODES(ConditionIsSafe);
        // REGISTER_NODES(IsLowHealth);
        // REGISTER_NODES(IsLowAmmo);
        // REGISTER_NODES(IsLowTime);
        // REGISTER_NODES(RecoverState);
        // REGISTER_NODES(SetPositionRepeatFired);

        // REGISTER_NODES(SetPositionRepeat);
        REGISTER_NODES(SetAimTargetFromAim);
        REGISTER_NODES(SetNaviPosition);
        
        // REGISTER_NODES(CountPlus);
        // REGISTER_NODES(CountDiv);
#undef REGISTER_NODES

        return true;
    }

}