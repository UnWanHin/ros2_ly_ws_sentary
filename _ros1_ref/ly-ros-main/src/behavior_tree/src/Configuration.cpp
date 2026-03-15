#include "../include/Application.hpp"


namespace LangYa {

    // 自定义 from_json 函数，用于自动转换 JSON 到结构体
    void from_json(const json& j, AimDebug& ad) {
        j.at("StopFire").get_to(ad.StopFire);
        j.at("StopRotate").get_to(ad.StopRotate);
        j.at("StopScan").get_to(ad.StopScan);
        j.at("HitOutpost").get_to(ad.HitOutpost);
        j.at("HitBuff").get_to(ad.HitBuff);
        j.at("HitCar").get_to(ad.HitCar);
    }

    void from_json(const json& j, Rate& r) {
        j.at("FireRate").get_to(r.FireRate);
        j.at("TreeTickRate").get_to(r.TreeTickRate);
        j.at("NaviCommandRate").get_to(r.NaviCommandRate);
    }
    void from_json(const json& j, GameStrategy& gs) {
        j.at("HitBuff").get_to(gs.HitBuff);
        j.at("HitOutpost").get_to(gs.HitOutpost);
        j.at("TestNavi").get_to(gs.TestNavi);
        j.at("HitSentry").get_to(gs.HitSentry);
        j.at("Protected").get_to(gs.Protected);
    }
    void from_json(const json& j, NaviSetting& ns) {
        j.at("UseXY").get_to(ns.UseXY);
    }

    void from_json(const json& j, Config& c) {
        j.at("AimDebug").get_to(c.AimDebugSettings);
        j.at("Rate").get_to(c.RateSettings);
        j.at("GameStrategy").get_to(c.GameStrategySettings);
        j.at("ScanCounter").get_to(c.ScanCounter);
        j.at("NaviSetting").get_to(c.NaviSettings);
    }
}

namespace BehaviorTree {
    using namespace LangYa;
    using json = nlohmann::json;

    bool Application::ConfigurationInit() {
        std::ifstream ifs(config_file);
        if (!ifs.is_open()) {
            LoggerPtr->Error("Failed to open config.json, file location: {}", config_file);
            return false;
        }
        LoggerPtr->Info("Open config.json, file location: {}", config_file);

        // 解析 JSON 文件
        json j;
        ifs >> j;
        config = j.get<Config>();
        LoggerPtr->Debug("------ AimDebug ------");
        LoggerPtr->Debug("StopFire: {}", config.AimDebugSettings.StopFire);
        LoggerPtr->Debug("StopRotate: {}", config.AimDebugSettings.StopRotate);
        LoggerPtr->Debug("StopScan: {}", config.AimDebugSettings.StopScan);
        LoggerPtr->Debug("HitOutpost: {}", config.AimDebugSettings.HitOutpost);
        LoggerPtr->Debug("HitBuff: {}", config.AimDebugSettings.HitBuff);
        LoggerPtr->Debug("HitCar: {}", config.AimDebugSettings.HitCar);
        LoggerPtr->Debug("------ Rate ------");
        LoggerPtr->Debug("FireRate: {}", config.RateSettings.FireRate);
        LoggerPtr->Debug("TickRate: {}", config.RateSettings.TreeTickRate);
        LoggerPtr->Debug("NaviCommandRate: {}", config.RateSettings.NaviCommandRate);
        LoggerPtr->Debug("------ GameStrategy ------");
        LoggerPtr->Debug("ScanCounter: {}", config.ScanCounter);
        LoggerPtr->Debug("HitOutpost: {}", config.GameStrategySettings.HitOutpost);
        LoggerPtr->Debug("HitBuff: {}", config.GameStrategySettings.HitBuff);
        LoggerPtr->Debug("HitSentry: {}", config.GameStrategySettings.HitSentry);
        LoggerPtr->Debug("------ NaviSetting ------");
        LoggerPtr->Debug("UseXY: {}", config.NaviSettings.UseXY);
        LoggerPtr->Debug("------ End ------");
        LoggerPtr->Debug("Configuration completed.");
        fireRateClock.reset(config.RateSettings.FireRate);
        treeTickRateClock.reset(config.RateSettings.TreeTickRate);
        naviCommandRateClock.reset(config.RateSettings.NaviCommandRate);
        return true;
    }
}