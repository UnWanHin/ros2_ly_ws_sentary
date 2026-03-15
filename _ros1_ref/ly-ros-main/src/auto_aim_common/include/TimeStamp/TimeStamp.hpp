#pragma once
#include <ros/ros.h>
#include <iomanip>
#include <sstream>
#include <ctime>

class DurationExt : public ros::Duration {
public:
    using ros::Duration::Duration; // 继承构造函数
    
    // 返回毫秒值
    double count() const { return this->toSec() * 1000.0; }

    // 返回秒
    double toSeconds() const { return this->toSec(); }
    
    // 运算符重载
    DurationExt operator-(const DurationExt& other) const {
        return DurationExt(this->toSec() - other.toSec());
    }

    // 新增：将 Duration 转换为字符串（例如 "3.500s"）
    std::string toString() const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << this->toSec() << "s";
        return ss.str();
    }
};


///TODO 性能优化：频繁调用 toString() 时，可预分配缓冲区或使用线程安全的 strftime 替代方案。
// 实现 Time 的 toString()
namespace Time {
    using TimeStamp = ros::Time;
    using TimeDuration = ros::Duration;

    // 新增：将 ros::Time 转换为格式化字符串（例如 "2025-05-14 15:30:45"）
    inline std::string toString(const TimeStamp& timestamp) {
        if (timestamp.isZero()) return "0"; // 处理未初始化时间
        
        time_t raw_time = static_cast<time_t>(timestamp.sec);
        struct tm* time_info = localtime(&raw_time);
        
        char buffer[80];
        strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", time_info);
        
        // 添加纳秒部分（例如 "2025-05-14 15:30:45.123456"）
        std::stringstream ss;
        ss << buffer << "." << std::setw(9) << std::setfill('0') << timestamp.nsec;
        return ss.str();
    }
}

// 重载运算符确保 Time 相减返回 DurationExt
inline DurationExt operator-(const ros::Time& t1, const ros::Time& t2) {
    return DurationExt(t1.toSec() - t2.toSec());
}