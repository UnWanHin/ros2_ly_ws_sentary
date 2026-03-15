#pragma once

#include "../module/BasicTypes.hpp"

#include <iostream>
#include <chrono>

using namespace LangYa;



namespace BehaviorTree {

    

    class Robot {
    public:
        Robot() {
            maxHealth_ = 100;
            currentHealth_ = 100;
            distance_ = 30;
            invulnerableStartTime_ = std::chrono::steady_clock::now();
        }
        Robot(int maxHealth, double initialDistance)
            : maxHealth_(maxHealth), currentHealth_(maxHealth), distance_(initialDistance), invulnerableStartTime_() {}

        // 检查是否处于无敌状态
        bool isInvulnerable() const {
            if (currentHealth_ == 0) return false;
            // 判断是否在无敌时间范围内（10秒）
            auto now = std::chrono::steady_clock::now();
            return (currentHealth_ > 0 && !invulnerableStartTime_.time_since_epoch().count() == 0 &&
                    std::chrono::duration_cast<std::chrono::seconds>(now - invulnerableStartTime_).count() < 10);
        }

        // 判断是否位于特殊区域
        bool inCastleMyself() {
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::CastleRed.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::CastleBlue.isPointInside(position_.X, position_.Y);
        } // 己方堡垒
        bool inCastleEnemy() {
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::CastleBlue.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::CastleRed.isPointInside(position_.X, position_.Y);
        } // 敌方堡垒
        bool inCentralHighLandRedMysself() {
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::CentralHighLandRed.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::CentralHighLandBlue.isPointInside(position_.X, position_.Y);
        } // 己方中央高地
        bool inCentralHighLandEnemy() {
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::CentralHighLandBlue.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::CentralHighLandRed.isPointInside(position_.X, position_.Y);
        } // 敌方中央高地
        bool inRoadLandMyself() { 
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::RoadLandRed.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::RoadLandBlue.isPointInside(position_.X, position_.Y);
        } // 己方公路区
        bool inRoadLandEnemy() { 
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::RoadLandBlue.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::RoadLandRed.isPointInside(position_.X, position_.Y);
        } // 敌方公路区
        bool inFlyLandMyself() { 
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::FlyLandRed.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::FlyLandBlue.isPointInside(position_.X, position_.Y);
        } // 己方飞坡区
        bool inFlyLandEnemy() { 
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::FlyLandBlue.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::FlyLandRed.isPointInside(position_.X, position_.Y);
        } // 敌方飞坡区


        void setCurrentHealth(int health) {
            if (currentHealth_ == 0 && health > 0 && health != maxHealth_) { // 判断是否无敌
                invulnerableStartTime_ = std::chrono::steady_clock::now();
            }
            currentHealth_ = health;
        }

        void SetDistance(double distance) {
            distance_ = distance;
        }
        void SetPosition(UWBPositionType position) {
            position_ = position;
        }
        void setTeamColor(UnitTeam team) {
            team_ = team;
        }
        
    public:

        UnitTeam team_;
        std::uint16_t maxHealth_;                     // 最大血量
        std::uint16_t currentHealth_;                 // 当前血量
        float distance_;                              // 距离
        UWBPositionType position_;                    // 位置
        std::chrono::steady_clock::time_point invulnerableStartTime_; // 无敌状态开始时间
    };

    class Robots {
    public:
        Robots() {
            for (auto& robot : robots_) {
                robot = Robot(100, 30);
            }
            robots_[static_cast<size_t>(UnitType::Hero)] = Robot(200, 30);
            robots_[static_cast<size_t>(UnitType::Engineer)] = Robot(250, 30);
            robots_[static_cast<size_t>(UnitType::Sentry)] = Robot(400, 30);
        }
        Robot& operator[](UnitType type) {
            return robots_[static_cast<size_t>(type)];
        }

        const Robot& operator[](UnitType type) const {
            return robots_[static_cast<size_t>(type)];
        }

        Robot& operator[](int type) {
            return robots_[type];
        }
        const Robot& operator[](int type) const {
            return robots_[type];
        }

    private:
        std::array<Robot, 10> robots_; // 根据 UnitType 枚举数量调整大小
    };

}

