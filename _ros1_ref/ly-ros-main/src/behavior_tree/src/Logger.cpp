#include "../include/Application.hpp"

using namespace Utils::Logger;

namespace BehaviorTree {

    std::string GenerateLogFilename() {
        // 获取当前时间点
        auto now = std::chrono::system_clock::now();
        // 转换为时间戳
        auto timestamp = std::chrono::system_clock::to_time_t(now);
        // 转换为本地时间
        std::tm time_info = *std::localtime(&timestamp);
    
        // 使用 stringstream 构建文件名
        std::ostringstream oss;
        oss << "/home/hustlyrm/Log/BT_"
            << std::setw(4) << std::setfill('0') << (time_info.tm_year + 1900)
            << std::setw(2) << std::setfill('0') << (time_info.tm_mon + 1)
            << std::setw(2) << std::setfill('0') << time_info.tm_mday << "_"
            << std::setw(2) << std::setfill('0') << time_info.tm_hour
            << std::setw(2) << std::setfill('0') << time_info.tm_min
            << std::setw(2) << std::setfill('0') << time_info.tm_sec
            << ".log";
    
        return oss.str();
    } 
    bool Application::InitLogger() {
        LoggerPtr = std::make_shared<Logger>();
        auto filename = GenerateLogFilename();  // 动态生成文件名
        auto filePolicy = std::make_shared<FileLogPolicy>(filename);
        auto consolePolicy = std::make_shared<ConsoleLogPolicy>();
        LoggerPtr->AddPolicy(filePolicy);
        LoggerPtr->AddPolicy(consolePolicy);
        return true;
    }
}