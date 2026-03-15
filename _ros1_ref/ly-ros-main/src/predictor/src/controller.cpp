#include "controller/controller.hpp"
#include <cmath>
#include <Logger/Logger.hpp>

auto ly_auto_aim::controller::createController() -> std::shared_ptr<ly_auto_aim::controller::Controller>
{
    return std::make_unique<ly_auto_aim::controller::Controller>();
}

using namespace ly_auto_aim::controller;

void Controller::registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc)
{
    this->predictFunc = predictFunc;
}

void Controller::loadShootTableParams(ros::NodeHandle& nh)
{
    // 读取射击表调整开关
    if(!nh.getParam("shoot_table_adjust/enable", shoot_table_adjust)) {
        roslog::warn("Shoot table adjust not set, using default value false");
        shoot_table_adjust = false;
    }
    
    if(shoot_table_adjust) {
        // 初始化参数向量
        pitch_param.resize(6, 0.0);
        yaw_param.resize(6, 0.0);
        
        // 读取pitch参数
        nh.getParam("shoot_table_adjust/pitch/intercept", pitch_param[0]);
        nh.getParam("shoot_table_adjust/pitch/coef_z", pitch_param[1]);
        nh.getParam("shoot_table_adjust/pitch/coef_d", pitch_param[2]);
        nh.getParam("shoot_table_adjust/pitch/coef_z2", pitch_param[3]);
        nh.getParam("shoot_table_adjust/pitch/coef_zd", pitch_param[4]);
        nh.getParam("shoot_table_adjust/pitch/coef_d2", pitch_param[5]);
        
        // 读取yaw参数
        nh.getParam("shoot_table_adjust/yaw/intercept", yaw_param[0]);
        nh.getParam("shoot_table_adjust/yaw/coef_z", yaw_param[1]);
        nh.getParam("shoot_table_adjust/yaw/coef_d", yaw_param[2]);
        nh.getParam("shoot_table_adjust/yaw/coef_z2", yaw_param[3]);
        nh.getParam("shoot_table_adjust/yaw/coef_zd", yaw_param[4]);
        nh.getParam("shoot_table_adjust/yaw/coef_d2", yaw_param[5]);
        
        roslog::info("Shoot table adjustment enabled");
        roslog::info("Pitch params: intercept={}, coef_z={}, coef_d={}", 
                    pitch_param[0], pitch_param[1], pitch_param[2]);
        roslog::info("Yaw params: intercept={}, coef_z={}, coef_d={}", 
                    yaw_param[0], yaw_param[1], yaw_param[2]);
    } else {
        roslog::info("Shoot table adjustment disabled");
    }
}

double Controller::fitPitch(double z_height, double horizontal_distance) const
{
    if (!shoot_table_adjust || pitch_param.size() < 6) {
        return 0.0;
    }
    
    // Model 3: Full 2nd Order 参数 (PITCH)
    double intercept = pitch_param[0];
    double coef_z = pitch_param[1];
    double coef_d = pitch_param[2];
    double coef_z2 = pitch_param[3];
    double coef_zd = pitch_param[4];
    double coef_d2 = pitch_param[5];

    // 计算特征值
    double z2 = z_height * z_height;
    double d2 = horizontal_distance * horizontal_distance;
    double zd = z_height * horizontal_distance;
    
    // 完整二阶多项式: intercept + z + d + z² + z*d + d²
    return intercept + 
           coef_z * z_height + 
           coef_d * horizontal_distance + 
           coef_z2 * z2 + 
           coef_zd * zd + 
           coef_d2 * d2;
}

double Controller::fitYaw(double z_height, double horizontal_distance) const
{
    if (!shoot_table_adjust || yaw_param.size() < 6) {
        return 0.0;
    }
    
    // Model 3: Full 2nd Order 参数 (YAW)
    double intercept = yaw_param[0];
    double coef_z = yaw_param[1];
    double coef_d = yaw_param[2];
    double coef_z2 = yaw_param[3];
    double coef_zd = yaw_param[4];
    double coef_d2 = yaw_param[5];

    // 计算特征值
    double z2 = z_height * z_height;
    double d2 = horizontal_distance * horizontal_distance;
    double zd = z_height * horizontal_distance;

    // 完整二阶多项式: intercept + z + d + z² + z*d + d²
    return intercept + 
           coef_z * z_height + 
           coef_d * horizontal_distance + 
           coef_z2 * z2 + 
           coef_zd * zd + 
           coef_d2 * d2;
}

bool Controller::calcPitchYawWithShootTable(double& pitch, double& yaw, double& time, 
                                           double target_x, double target_y, double target_z)
{
    // 先进行基础弹道计算
    bool success = calcPitchYaw(pitch, yaw, time, target_x, target_y, target_z);
    
    if (success && shoot_table_adjust) {
        // 计算水平距离和高度
        double horizontal_distance = std::sqrt(target_x * target_x + target_y * target_y);
        
        // 应用射击表补偿（以弧度为单位）
        double pitch_compensation = fitPitch(target_z, horizontal_distance) * PI / 180.0;
        double yaw_compensation = fitYaw(target_z, horizontal_distance) * PI / 180.0;
        
        pitch += pitch_compensation;
        yaw += yaw_compensation;
        
        roslog::info("Applied shoot table compensation - pitch: {} rad, yaw: {} rad", 
                    pitch_compensation, yaw_compensation);
    }
    
    return success;
}

const int camera_halfwidth=640;
const int camera_halfheight=512;

static bool thetaInRange(double theta_deg, double range_deg)
{
    theta_deg = std::remainder(theta_deg, 360.0);
    if(std::abs(theta_deg) < range_deg)
        return true;
    else
        return false;
} 

ControlResult Controller::control(const GimbalAngleType& gimbal_angle, int target, float bullet_speed)
{
    ControlResult result;
    result.yaw_setpoint = gimbal_angle.yaw;
    result.pitch_setpoint = gimbal_angle.pitch;
    result.pitch_actual_want = gimbal_angle.pitch;
    result.yaw_actual_want = gimbal_angle.yaw;
    result.valid = false;
    result.shoot_flag = false;

    Predictions predictions_for_time = predictFunc(Time::TimeStamp::now() + flyTime + shootDelay);
    if (predictions_for_time.empty())
    {
        roslog::warn("No prediction");
        return result;
    }
    bool is_valid_car_id = std::any_of(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (!is_valid_car_id)
    {
        roslog::warn("Invalid car id");

        bool found = false;
        double min_distance = std::numeric_limits<double>::max();
        for(const auto& prediction : predictions_for_time){
            if(prediction.id == target){
                aim_armor_id.first = target;
                aim_armor_id.second = -1;
                found = true;
            } 
        }
        if(!found)
        {
            for (const auto& prediction : predictions_for_time)
            {
                double distance = sqrt(prediction.center.x * prediction.center.x + prediction.center.y * prediction.center.y);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    aim_armor_id.first = prediction.id;
                    aim_armor_id.second = -1;
                    found = true;
                }
            }
        }
    }
    auto it = std::find_if(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (it == predictions_for_time.end())
    {
        roslog::warn("New car id invalid(shouldn't happen)");
        return result;
    }
    roslog::warn("aim car: {}", aim_armor_id.first);
    //calc new flytime
    double distance = sqrt(it->center.x * it->center.x + it->center.y * it->center.y);
    if (distance > 0.0)
    {
        constexpr float stable_bullet_speed = 23.0f;
        if(bullet_speed - stable_bullet_speed < 1.0f && bullet_speed - stable_bullet_speed > -1.0f){
            this->bullet_speed = stable_bullet_speed;
        }
        this->bullet_speed = stable_bullet_speed;
        roslog::info("info: my_bullet_speed to cal time: {}", this->bullet_speed);
        double calculated_time_sec = distance / this->bullet_speed;
        roslog::info("info: calculated_time_sec = {}", calculated_time_sec);
        flyTime = Time::TimeDuration(calculated_time_sec);
        roslog::info("info: Successfully created flyTime with {} seconds.", flyTime.toSec());
        predictions_for_time = predictFunc(Time::TimeStamp::now() + flyTime + shootDelay);
        roslog::info("info: end second predict.");
    }
    it = std::find_if(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (it == predictions_for_time.end())
    {
        roslog::warn("New car id invalid after flytime update");
        return result;
    }
    bool is_valid_armor_id = std::any_of(it->armors.begin(), it->armors.end(),
        [&](const auto& armor) { return (armor.id == aim_armor_id.second) && (armor.status == Armor::AVAILABLE); });
    if (!is_valid_armor_id)
    {
        roslog::warn("Invalid armor id");
        //重新选择一个距离最近的
        double min_distance = std::numeric_limits<double>::max();
        bool found = false;
        for (const auto& armor : it->armors)
        {
            if (armor.status == Armor::AVAILABLE)
            {
                double distance = sqrt(armor.center.x * armor.center.x + armor.center.y * armor.center.y);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    aim_armor_id.second = armor.id;
                    found = true;
                }
            }
        }
        if (!found)
        {
            roslog::warn("No available armor");
            roslog::warn("Now choose to aim at the car");
            double pitch = 0.0;
            double yaw = 0.0;
            double time = 0.0;
            // 使用带射击表补偿的弹道计算
            bool success = calcPitchYawWithShootTable(pitch, yaw, time, 
                                                     it->center.x + x_offset, 
                                                     it->center.y + y_offset, 
                                                     it->center.z + z_offset);
            if (!success)
            {
                roslog::warn("calcPitchYawWithShootTable failed");
                return result;
            }
            result.pitch_setpoint = pitch * 180 / PI;
            result.yaw_setpoint = yaw * 180 / PI;
            result.yaw_setpoint = result.yaw_setpoint + std::round((gimbal_angle.yaw - result.yaw_setpoint) / 360.0f) * 360.0f;
            result.pitch_actual_want = result.pitch_setpoint;
            result.yaw_actual_want = result.yaw_setpoint;
            result.valid = true;
            result.shoot_flag = false;
            flyTime = Time::TimeDuration(time);
            roslog::info("aim_pitch:{},aim_yaw:{}", result.pitch_setpoint, result.yaw_setpoint);
            return result;
        }
    }
    auto armor_it = std::find_if(it->armors.begin(), it->armors.end(),
        [&](const auto& armor) { return armor.id == aim_armor_id.second; });
    if (armor_it == it->armors.end())
    {
        roslog::warn("Invalid armor id (shoudln't happen)");
        return result;
    }
    roslog::warn("aim armor id: {}", aim_armor_id.second);
    //should calc new time
    //but now we just use the old time
    double pitch = 0.0;
    double yaw = 0.0;
    double time = 0.0;
    // 使用带射击表补偿的弹道计算
    if (!calcPitchYawWithShootTable(pitch, yaw, time, armor_it->center.x, armor_it->center.y, armor_it->center.z))
    {
        roslog::warn("calcPitchYawWithShootTable failed");
        return result;
    }
    result.pitch_setpoint = pitch * 180 / PI;
    result.yaw_setpoint = yaw * 180 / PI;
    result.yaw_setpoint = result.yaw_setpoint + std::round((gimbal_angle.yaw - result.yaw_setpoint) / 360.0f) * 360.0f;
    result.pitch_actual_want = result.pitch_setpoint;
    result.yaw_actual_want = result.yaw_setpoint;
    result.valid = true;
    float yaw_diff = result.yaw_actual_want - gimbal_angle.yaw;
    if(!it->stable) result.valid = false;
    if(yaw_diff > 80.0f || yaw_diff < -80.0f) result.valid = false;
    if(result.pitch_setpoint < -2.0f ) result.pitch_actual_want -= 1.0f; /// 高打低偏置补丁

    roslog::warn("debug: finish control"); 

    return result;
}

bool Controller::judgeAimNew(bool request)
{
    aim_new = false;
    if(((!aiming) && request) || (aiming && (!request)))
    {
        accumulate_aim_request++;
    }
    else
    {
        accumulate_aim_request = 0;
    }
    if(accumulate_aim_request > waitFrame)
    {
        accumulate_aim_request = 0;
        aiming = request;
        if(aiming)
            aim_new = true;
    }
    return aim_new;
}

bool Controller::calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z)
{
    roslog::info("targetx,y,z:{},{},{}",target_x,target_y,target_z);
    double distance = sqrt(target_x * target_x + target_y * target_y);
    double theta = pitch;
    double delta_z = 0.0;
    // 首先计算空气阻力系数 K
    double k1 = C_D * RHO * (PI * bullet_diameter * bullet_diameter) / 8 / bullet_mass;
    for (int i = 0; i < max_iter; i++)
    {
        // 计算炮弹的飞行时间
        double t = (exp(k1 * distance) - 1) / (k1 * bullet_speed * cos(theta));

        delta_z = target_z - bullet_speed * sin(theta) * t / cos(theta) + 0.5 * GRAVITY * t * t / cos(theta) / cos(theta);

        // 不断更新theta，直到小于某一个阈值
        if (fabs(delta_z) < tol)
        {
            time = t;
            break;
        }

        // 更新角度
        theta -= delta_z / (-(bullet_speed * t) / pow(cos(theta), 2) + GRAVITY * t * t / (bullet_speed * bullet_speed) * sin(theta) / pow(cos(theta), 3));
    }
    if(fabs(delta_z) > tol)
    {
        //不更新pitch和yaw
        roslog::warn("calcPitchYaw failed");
        return false;//计算失败
    }
    else
    {
        pitch = theta;
        yaw = atan2(target_y, target_x);
        return true;
    }
}
