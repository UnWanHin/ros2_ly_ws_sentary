#pragma once
#include <fmt/format.h>
#include <fmt/chrono.h>

#include <ros/ros.h>

namespace roslog
{
#define LY_DEF_ROS_LOG(name, rosName)\
    template<typename ...Args> \
    void name(const std::string& format_str, Args &&... args) \
    { \
        ROS_##rosName##_STREAM(fmt::format(format_str, std::forward<Args>(args)...)); \
    } \
    // generate ros log function

    LY_DEF_ROS_LOG(error, ERROR);
    LY_DEF_ROS_LOG(warn, WARN);
    LY_DEF_ROS_LOG(info, INFO);
    LY_DEF_ROS_LOG(debug, DEBUG);
#undef LY_DEF_ROS_LOG

}