#pragma once
#include <memory>
#include <functional>
#include <mutex>
#include "../Logger/Logger.hpp"

namespace LangYa
{
    template<typename TGlobal>
    class MultiCallback
    {
        std::mutex Mutex{};
        TGlobal Global;

    public:
        MultiCallback(TGlobal g) noexcept : Global(g) {}

        template<typename TTopic>
        std::function<void(typename TTopic::CallbackArg)> Generate(auto modifier) noexcept
        {
            return [this, modifier](typename TTopic::CallbackArg const& arg)
            {
                std::lock_guard lock{ Mutex };
                modifier(Global, arg);
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

        template<typename TTopic, typename TFunc>
        void GenSubscriber(TFunc&& callback)
        {
            std::string name{ TTopic::Name };
            if (Subscribers.contains(name)) return;
            Subscribers.insert({ name, NodePtr.get()->subscribe<typename TTopic::Msg>(name, 2, std::forward<TFunc>(callback)) });
        }

        template<typename T>
        bool GetParam(const std::string& param_name, T& value, const T& default_val)
        {
            if (NodePtr.get()->param(param_name, value, default_val)) return true;
            else roslog::error("Failed to get param '{}'", param_name);
            return false;
            // return params_.Load(param_name, value, default_val);
        }
    };

}

