#include "../include/Application.hpp"


namespace BehaviorTree {

    void Application::PublishMessageAll() {
        
        PubAimModeEnableData();
        PubGimbalControlData();
        PubAimTargetData();
        PubNaviControlData();
        if(naviCommandRateClock.trigger()) {
            naviCommandRateClock.tick();
            // PubNaviGoal();
            if(config.NaviSettings.UseXY) PubNaviGoalPos();
            else PubNaviGoal();
        }
    }

    void Application::PubAimModeEnableData() {
        {
            using topic = ly_aa_enable;
            topic::Msg msg;
            if (aimMode != AimMode::Buff) {
                msg.data = true;
            }else msg.data = false;
            node.Publisher<topic>().publish(msg);
        }
        {
            using topic = ly_ra_enable;
            topic::Msg msg;
            if (aimMode == AimMode::Buff) {
                msg.data = true;
            }else msg.data = false;
            node.Publisher<topic>().publish(msg);
        }
        {
            using topic = ly_outpost_enable;
            topic::Msg msg;
            if (aimMode == AimMode::Outpost) {
                msg.data = true;
            }else msg.data = false;
            node.Publisher<topic>().publish(msg);
        }
    }

    /**
     * @brief 发布云台角度控制数据和火控数据 \n
     * @brief 云台数据可以随便修改，修改完之后发布即可，火控数据只有在接收到辐瞄的目标数据之后才会自动翻转
     * @param gimbalControlData 云台控制数据
     */
    void Application::PubGimbalControlData() {
        {
            using topic = ly_control_angles;
            topic::Msg msg;
            msg.Yaw = gimbalControlData.GimbalAngles.Yaw;
            msg.Pitch = gimbalControlData.GimbalAngles.Pitch;
            msg.header.stamp = ros::Time::now();
            node.Publisher<topic>().publish(msg);
        }
        {
            using topic = ly_control_firecode;
            topic::Msg msg;
            msg.data = *reinterpret_cast<std::uint8_t *>(&gimbalControlData.FireCode);
            node.Publisher<topic>().publish(msg);
        }
    }
    /**
     * @brief 发布自瞄应该击打的目标
     */
    void Application::PubAimTargetData() {
        {
            using topic = ly_bt_target;
            topic::Msg msg;
            msg.data = static_cast<uint8_t>(targetArmor.Type);
            node.Publisher<topic>().publish(msg);
        }
    }
    /**
     * @brief 发布导航的底盘速度控制数据
     * @param naviVelocity 导航速度X, Y
     */
    void Application::PubNaviControlData() {
        {
            using topic = ly_control_vel;
            topic::Msg msg;
            msg.X = naviVelocity.X;
            msg.Y = naviVelocity.Y;
            node.Publisher<topic>().publish(msg);
        }
    }
    /**
     * @brief 发布给导航的目标点
     */
    void Application::PubNaviGoal() {
        {
            using topic = ly_navi_goal;
            topic::Msg msg;
            msg.data = naviCommandGoal;
            node.Publisher<topic>().publish(msg);
        }
        {
            using topic = ly_navi_speed_level;
            topic::Msg msg;
            msg.data = speedLevel;
            node.Publisher<topic>().publish(msg);
        }
    }

    void Application::PubNaviGoalPos() {
        using topic = ly_navi_goal_pos;
        topic::Msg msg;
        std::vector<uint16_t> data = {naviGoalPosition.x, naviGoalPosition.y};
        msg.data = data;
        node.Publisher<topic>().publish(msg);
    }
}