#include "auto_aim_common/ControllerType.hpp"
#include "auto_aim_common/PredictionType.hpp"
#include "auto_aim_common/SolverType.hpp"
#include "TimeStamp/TimeStamp.hpp"
#include "Logger/Logger.hpp"
#include "RosTools/RosTools.hpp"


namespace ly_auto_aim::inline controller
{
    using ly_auto_aim::predictor::Armor;
    using ly_auto_aim::predictor::Prediction;
    using ly_auto_aim::predictor::Predictions;
    using ly_auto_aim::solver::GimbalAngleType;
    const double PI = 3.1415926;
    const double GRAVITY = 9.794;
    const double C_D = 0.42;
    const double RHO = 1.169;
    class Controller 
    {
    public:
        void registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc);
        explicit Controller(){
            ros::NodeHandle nh("controller_config");

            if(!nh.getParam("bullet_speed", bullet_speed)) {
                roslog::warn("Bullet speed not set, using default value");
                bullet_speed = 23.0;
            }

            double shoot_delay;
            if(!nh.getParam("shoot_delay", shoot_delay)) {
                roslog::warn("Shoot delay not set, using default value");
                shoot_delay = 0.1;
            }
            shootDelay = Time::TimeDuration(shoot_delay);
            
            // 读取射击表相关参数
            loadShootTableParams(nh);
        }
        ControlResult control(const GimbalAngleType& gimbal_angle, int target, float bullet_speed = 23.0f);
    private:
        std::function<Predictions(Time::TimeStamp)> predictFunc;
        bool aim_new = false;
        bool aiming = false;
        const int waitFrame = 5;
        int accumulate_aim_request = 0;
        std::pair<int, int> aim_armor_id = {-1, -1};//(car, armor)
        int aim_want = - waitFrame;
        int old_aim_want = - waitFrame;
        int max_iter = 100;
        double tol = 1e-6;
        double bullet_speed = 23.0;
        const double min_bullet_speed = 20.0;
        const double bullet_speed_alpha = 0.5;
        double bullet_mass = 3.2e-3;
        double bullet_diameter = 16.8e-3;
        bool judgeAimNew(bool request = true);  /// request在步兵中被使用，哨兵一直为1
        bool calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z);
        Time::TimeDuration shootDelay{0.1}; /// 单位是秒
        Time::TimeDuration flyTime{0.0}; /// 单位是秒

        double tol_pitch = 0.0;
        double tol_yaw = 0.0;
        double pitch_offset = 0.0;
        double yaw_offset = 0.0;
        double x_offset = 0.0;
        double y_offset = 0.0;
        double z_offset = 0.0;
        double armor_yaw_allow = 45.0;
        
        // 射击表相关参数
        bool shoot_table_adjust = false;
        std::vector<double> pitch_param;
        std::vector<double> yaw_param;
        
        // 射击表相关方法
        void loadShootTableParams(ros::NodeHandle& nh);
        double fitPitch(double z_height, double horizontal_distance) const;
        double fitYaw(double z_height, double horizontal_distance) const;
        bool calcPitchYawWithShootTable(double& pitch, double& yaw, double& time, 
                                       double target_x, double target_y, double target_z);
    };

    std::shared_ptr<Controller> createController();    
} // namespace controller