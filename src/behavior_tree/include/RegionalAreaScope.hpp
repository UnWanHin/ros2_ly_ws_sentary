#pragma once

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

// JSON keeps the complete policy baseline. For Regional Default navigation,
// AreaManager.yaml is the live owner of the five local/common area switches.
inline void ApplyRegionalAreaTaskScopeOverride(
    const LangYa::RegionalAreaTaskSetting& regional_task,
    LangYa::NaviGoalAutonomySetting& navi_goal) {
    if (!navi_goal.UseAreaScope) {
        return;
    }

    navi_goal.MyArea.clear();
    navi_goal.CommonArea.clear();
    if (!regional_task.Enable) {
        return;
    }

    if (regional_task.MyBase.Enable) {
        navi_goal.MyArea.emplace_back("base");
    }
    if (regional_task.MyHighland.Enable) {
        navi_goal.MyArea.emplace_back("highland");
    }
    if (regional_task.MyPreRoadland.Enable) {
        navi_goal.MyArea.emplace_back("pre_roadland");
    }
    if (regional_task.MyReadyRoadland.Enable) {
        navi_goal.MyArea.emplace_back("ready_roadland");
    }
    if (regional_task.CommonCentral.Enable) {
        navi_goal.CommonArea.emplace_back("central");
    }
}

}  // namespace BehaviorTree
