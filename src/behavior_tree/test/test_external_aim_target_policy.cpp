#include "ExternalAimTargetPolicy.hpp"

#include <gtest/gtest.h>

TEST(ExternalAimTargetPolicyTest, SelectsFreshVisibleOutpostOutsideOutpostTask) {
    EXPECT_TRUE(BehaviorTree::ShouldSelectFreshOutpostAimTarget(true, false));
}

TEST(ExternalAimTargetPolicyTest, DoesNotSelectMissingOrIgnoredOutpost) {
    EXPECT_FALSE(BehaviorTree::ShouldSelectFreshOutpostAimTarget(false, false));
    EXPECT_FALSE(BehaviorTree::ShouldSelectFreshOutpostAimTarget(true, true));
}
