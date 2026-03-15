#pragma once

#include <map>
#include <memory>
#include <functional>
#include <mutex>

#include <fmt/format.h>
#include <fmt/chrono.h>

#include <ros/ros.h>

namespace LangYa::roslog
{
#define LY_DEF_ROS_LOG(name, rosName)\
    template<typename ...Args> \
    void name(const auto fmt, Args &&... args) \
    { \
        ROS_##rosName##_STREAM(fmt::format(fmt, std::forward<Args>(args)...)); \
    } \

    LY_DEF_ROS_LOG(error, ERROR);
    LY_DEF_ROS_LOG(warn, WARN);
    LY_DEF_ROS_LOG(info, INFO);
    LY_DEF_ROS_LOG(debug, DEBUG);
#undef LY_DEF_ROS_LOG

}

namespace LangYa
{
    template<typename TGlobal>
    class MultiCallback
    {
        std::function<void(const TGlobal&)> PostEvent;
        std::mutex Mutex{};
        TGlobal Global{};

    public:
        MultiCallback(std::function<void(const TGlobal&)> postEvent) noexcept : PostEvent(std::move(postEvent)) {}

        template<typename TTopic>
        std::function<void(typename TTopic::CallbackArg)> Generate(
                std::function<void(TGlobal&, typename TTopic::Msg const&)> modifier
        ) noexcept
        {
            return [this, modifier](typename TTopic::CallbackArg arg)
            {
                std::lock_guard lock{ Mutex };
                modifier(Global, *arg);
                PostEvent(Global);
            };
        }
    };

    template<const char* TName, typename TMessage>
    struct ROSTopic
    {
        static constexpr auto Name = TName;
        using Msg = TMessage;
        using CallbackArg = typename Msg::ConstPtr;
    };

#define LY_DEF_ROS_TOPIC(varname, topic, type)\
    inline static constexpr const char varname##_topic_name[] = topic;\
    using varname = LangYa::ROSTopic<varname##_topic_name, type>;

    template<const char* TName>
    class ROSNode
    {
        std::unique_ptr<ros::NodeHandle> NodePtr{};
        std::map<std::string, ros::Publisher> Publishers{};
        std::map<std::string, ros::Subscriber> Subscribers{};

    public:
        ROSNode() noexcept = default;

        bool Initialize(int argc, char** argv) noexcept try
        {
            if (!ros::isInitialized()) ros::init(argc, argv, TName);
            NodePtr = std::make_unique<ros::NodeHandle>();
            return true;
        }
        catch (const std::exception& ex)
        {
            ROS_ERROR("ROSNode::Initialize: %s", ex.what());
            return false;
        }

        template<typename TTopic>
        ros::Publisher& Publisher()
        {
            std::string name{ TTopic::Name };
            if (!Publishers.contains(name))
                Publishers.insert({ name, NodePtr.get()->advertise<typename TTopic::Msg>(name, 3) });
            return Publishers[name];
        }

        template<typename TTopic>
        void GenSubscriber(auto callback)
        {
            std::string name{ TTopic::Name };
            if (Subscribers.contains(name)) return;
            Subscribers.insert({ name, NodePtr.get()->subscribe<typename TTopic::Msg>(name, 3, callback) });
        }

        template<typename T>
        bool GetParam(const std::string& name, T& value, const T& defaultValue) noexcept
        {
            if(NodePtr.get()->param(name, value, defaultValue)) return true;
            roslog::error("ROSNode::GetParam: cannot get param({})", name);
            return false;
        }
    };


}

