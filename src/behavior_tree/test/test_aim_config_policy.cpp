#include "../include/AimConfigPolicy.hpp"

#include <gtest/gtest.h>

namespace {

TEST(AimConfigPolicy, DisabledOverridePreservesProfileValues) {
    std::vector<int> priority{1, 3, 4};
    std::vector<int> ignore{7};

    BehaviorTree::ApplyAimConfigOverride(
        priority,
        ignore,
        BehaviorTree::AimConfigOverride{
            .Enabled = false,
            .TargetPriority = std::vector<int>{6},
            .TargetIgnore = std::vector<int>{2}});

    EXPECT_EQ(priority, (std::vector<int>{1, 3, 4}));
    EXPECT_EQ(ignore, (std::vector<int>{7}));
}

TEST(AimConfigPolicy, EnabledOverrideReplacesOnlyProvidedLists) {
    std::vector<int> priority{1, 3, 4};
    std::vector<int> ignore{7};

    BehaviorTree::ApplyAimConfigOverride(
        priority,
        ignore,
        BehaviorTree::AimConfigOverride{
            .Enabled = true,
            .TargetPriority = std::vector<int>{6, 2},
            .TargetIgnore = std::nullopt});

    EXPECT_EQ(priority, (std::vector<int>{6, 2}));
    EXPECT_EQ(ignore, (std::vector<int>{7}));
}

TEST(AimConfigPolicy, EnabledEmptyIgnoreExplicitlyClearsProfileIgnore) {
    std::vector<int> priority{1, 3, 4};
    std::vector<int> ignore{7, 2};

    BehaviorTree::ApplyAimConfigOverride(
        priority,
        ignore,
        BehaviorTree::AimConfigOverride{
            .Enabled = true,
            .TargetPriority = std::nullopt,
            .TargetIgnore = std::vector<int>{}});

    EXPECT_EQ(priority, (std::vector<int>{1, 3, 4}));
    EXPECT_TRUE(ignore.empty());
}

}  // namespace
