#pragma once

#include <cstdint>

namespace BehaviorTree {

class Application;

enum class StrategyLayer : std::uint8_t {
    Hard = 0,
    Default = 1,
    Task = 2,
    Tactical = 3,
    Finalizer = 4
};

const char* StrategyLayerName(StrategyLayer layer) noexcept;

class StrategyManager {
public:
    void Reset(Application& app) noexcept;
    void MarkHandled(Application& app, StrategyLayer layer, bool hard_lock = false) noexcept;

    bool Handled() const noexcept { return handled_; }
    bool HardLock() const noexcept { return hard_lock_; }
    bool DefaultRequested() const noexcept { return default_requested_; }
    const char* HandledBy() const noexcept { return handled_by_; }

    bool RunHard(Application& app);
    bool RunDefault(Application& app);
    bool RunTask(Application& app);
    bool RunTactical(Application& app);
    bool RunFinalizer(Application& app);

private:
    void PublishRuntimeToBlackboards(Application& app) const noexcept;

    bool handled_{false};
    bool hard_lock_{false};
    bool default_requested_{false};
    bool default_goal_commanded_{false};
    StrategyLayer handled_layer_{StrategyLayer::Finalizer};
    const char* handled_by_{"none"};
};

}  // namespace BehaviorTree
