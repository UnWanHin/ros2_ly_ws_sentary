#include "../include/NaviRotatePosture.hpp"

#include <gtest/gtest.h>

namespace {

LangYa::NaviControlSetting EnabledSetting() {
    LangYa::NaviControlSetting setting;
    setting.Enable = true;
    setting.SetPostureToMoveWhenFalse = true;
    setting.FreshTimeoutMs = 500;
    return setting;
}

}  // namespace

TEST(NaviRotatePostureTest, FreshFalseRequestsMovePosture) {
    const auto now = std::chrono::steady_clock::now();

    EXPECT_TRUE(BehaviorTree::ShouldRequestMovePostureWhenNaviFalse(
        EnabledSetting(), true, now - std::chrono::milliseconds(499), now, false));
}

TEST(NaviRotatePostureTest, FreshTrueDoesNotRequestMovePosture) {
    const auto now = std::chrono::steady_clock::now();

    EXPECT_FALSE(BehaviorTree::ShouldRequestMovePostureWhenNaviFalse(
        EnabledSetting(), true, now, now, true));
}

TEST(NaviRotatePostureTest, StaleFalseDoesNotRequestMovePosture) {
    const auto now = std::chrono::steady_clock::now();

    EXPECT_FALSE(BehaviorTree::ShouldRequestMovePostureWhenNaviFalse(
        EnabledSetting(), true, now - std::chrono::milliseconds(501), now, false));
}

TEST(NaviRotatePostureTest, DisabledFeatureDoesNotRequestMovePosture) {
    const auto now = std::chrono::steady_clock::now();
    auto setting = EnabledSetting();
    setting.SetPostureToMoveWhenFalse = false;

    EXPECT_FALSE(BehaviorTree::ShouldRequestMovePostureWhenNaviFalse(
        setting, true, now, now, false));
}
