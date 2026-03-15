#include "../include/Application.hpp"

using namespace LangYa;

namespace BehaviorTree {
    /**
     * @brief 等待比赛开始前的预操作 \n
     * @brief 1. 等待云台数据 \n
     * @brief 2. 等待比赛开始 \n
     * @brief 3. 设置云台角度 \n
     * @brief 4. 设置导航目标 \n
     * @brief 5. 设置底盘速度 \n
     */
    void Application::WaitForGameStart() {
        /// 设置云台角度
        gimbalControlData.GimbalAngles.Yaw = gimbalAngles.Yaw;
        gimbalControlData.GimbalAngles.Pitch = AngleType{0};

        boost::filesystem::path waiting_for_begin("/home/hustlyrm/workspace/devel/lib/behavior_tree/.waiting_for_begin");
        LoggerPtr->Info("Waiting For Game Start!");

        ros::Time last_navi_command_time = ros::Time::now();
        boost::system::error_code ec{};
        while (exists(waiting_for_begin, ec) && ros::ok()) {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
            ros::Time now = ros::Time::now();

            SET_POSITION(Home, team);
            // PubNaviGoal();
            if(naviCommandRateClock.trigger()) {
                naviCommandRateClock.tick();
                if(config.NaviSettings.UseXY) PubNaviGoalPos();
                else PubNaviGoal();
            }

            if (naviVelocity.X || naviVelocity.Y) {
                PubNaviControlData();
                last_navi_command_time = now;
            }
            else {
                if (now - last_navi_command_time > ros::Duration(3)) {
                    naviVelocity.X = 0;
                    naviVelocity.Y = 0;
                    PubNaviControlData();
                }
            }
            PubGimbalControlData();

            if (is_game_begin) {
                LoggerPtr->Info("!!!!Game !! start!!!!");
                break;
            }
        }
        LoggerPtr->Info("Stop Waiting For Game");
    }

    void Application::WaitBeforeGame() {
        LoggerPtr->Info("Waiting Before Game");
        /// 取得第一个云台数据包
        while (ros::ok()) {
            ros::spinOnce();
            if (gimbalAngles.Yaw != 0 || gimbalAngles.Pitch != 0) {
                LoggerPtr->Info("Waiting For First Gimbal Data");
                break;
            }
            /// 休眠
            std::this_thread::sleep_for(1s);
        }
        LoggerPtr->Info("Stop Waiting For First Gimbal Data");

        WaitForGameStart();

        LoggerPtr->Info("Stop Waiting Before Game");
    }
}