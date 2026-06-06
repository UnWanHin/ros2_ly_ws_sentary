#include "../include/AimSource.hpp"

#include <gtest/gtest.h>

namespace {

LangYa::AimData FreshAim() {
    LangYa::AimData data;
    data.Fresh = true;
    data.Valid = true;
    data.FireStatus = true;
    data.HasLatchedAngles = true;
    data.LastValidTime = std::chrono::steady_clock::now();
    return data;
}

}  // namespace

TEST(AimSourceTest, ExternalModeMapsAllFormalViewsToExternalAim) {
    using namespace BehaviorTree;
    using namespace LangYa;

    AimData auto_aim;
    AimData external_aim;
    AimData buff_aim;
    AimData outpost_aim;

    external_aim = FreshAim();
    const auto external_view = MakeAimSourceView(
        true,
        AimMode::Outpost,
        auto_aim,
        external_aim,
        buff_aim,
        outpost_aim);
    EXPECT_EQ(external_view.Active, &external_aim);
    EXPECT_EQ(external_view.AutoAim, &external_aim);
    EXPECT_EQ(external_view.Buff, &external_aim);
    EXPECT_EQ(external_view.Outpost, &external_aim);
    EXPECT_TRUE(AimFreshAndValid(*external_view.Active));
}

TEST(AimSourceTest, LegacyModeKeepsAimModeSpecificSources) {
    using namespace BehaviorTree;
    using namespace LangYa;

    AimData auto_aim;
    AimData external_aim;
    AimData buff_aim;
    AimData outpost_aim;

    buff_aim = FreshAim();
    const auto legacy_buff_view = MakeAimSourceView(
        false,
        AimMode::Buff,
        auto_aim,
        external_aim,
        buff_aim,
        outpost_aim);
    EXPECT_EQ(legacy_buff_view.Active, &buff_aim);
    EXPECT_EQ(legacy_buff_view.Buff, &buff_aim);

    outpost_aim = FreshAim();
    const auto legacy_outpost_view = MakeAimSourceView(
        false,
        AimMode::Outpost,
        auto_aim,
        external_aim,
        buff_aim,
        outpost_aim);
    EXPECT_EQ(legacy_outpost_view.Active, &outpost_aim);
    EXPECT_EQ(legacy_outpost_view.Outpost, &outpost_aim);

    auto_aim = FreshAim();
    const auto legacy_auto_view = MakeAimSourceView(
        false,
        AimMode::RotateScan,
        auto_aim,
        external_aim,
        buff_aim,
        outpost_aim);
    EXPECT_EQ(legacy_auto_view.Active, &auto_aim);
}

TEST(AimSourceTest, BuffLockKeepsLegacyBuffFollowButExternalUsesUnifiedAim) {
    using namespace BehaviorTree;
    using namespace LangYa;

    AimData buff_aim = FreshAim();
    buff_aim.BuffFollow = false;
    EXPECT_FALSE(AimBuffTargetLocked(buff_aim, false));
    EXPECT_FALSE(AimBuffFireReady(buff_aim, false));

    buff_aim.BuffFollow = true;
    EXPECT_TRUE(AimBuffTargetLocked(buff_aim, false));
    EXPECT_TRUE(AimBuffFireReady(buff_aim, false));

    AimData external_aim = FreshAim();
    external_aim.BuffFollow = false;
    EXPECT_TRUE(AimBuffTargetLocked(external_aim, true));
    EXPECT_TRUE(AimBuffFireReady(external_aim, true));
}

TEST(AimSourceTest, LatchedTargetHonorsHoldWindowAndFreshFlags) {
    using namespace BehaviorTree;
    using namespace LangYa;

    AimData latched_only;
    const auto now = std::chrono::steady_clock::now();
    latched_only.HasLatchedAngles = true;
    latched_only.LastValidTime = now - std::chrono::milliseconds(50);
    EXPECT_TRUE(AimLatchedRecently(latched_only, now, std::chrono::milliseconds(100)));
    EXPECT_FALSE(
        AimLatchedRecently(latched_only, now, std::chrono::milliseconds(10)));

    bool fresh_target = true;
    bool latched_target = true;
    EXPECT_TRUE(
        AimTargetForAngles(
            latched_only,
            false,
            true,
            now,
            std::chrono::milliseconds(100),
            &fresh_target,
            &latched_target));
    EXPECT_FALSE(fresh_target);
    EXPECT_TRUE(latched_target);
}
