#include "ExternalAimTargetPolicy.hpp"

#include <gtest/gtest.h>

TEST(ExternalAimTargetPolicyTest, SelectsFreshVisibleOutpostOutsideOutpostTask) {
    EXPECT_TRUE(BehaviorTree::ShouldSelectFreshOutpostAimTarget(true, false));
}

TEST(ExternalAimTargetPolicyTest, DoesNotSelectMissingOrIgnoredOutpost) {
    EXPECT_FALSE(BehaviorTree::ShouldSelectFreshOutpostAimTarget(false, false));
    EXPECT_FALSE(BehaviorTree::ShouldSelectFreshOutpostAimTarget(true, true));
}

TEST(ExternalAimTargetPolicyTest, SelectsEveryFormalArmorTargetType) {
    for (const auto armor_type : {
             LangYa::ArmorType::Base,
             LangYa::ArmorType::Hero,
             LangYa::ArmorType::Engineer,
             LangYa::ArmorType::Infantry1,
             LangYa::ArmorType::Infantry2,
             LangYa::ArmorType::Infantry3,
             LangYa::ArmorType::Sentry,
             LangYa::ArmorType::Outpost}) {
        EXPECT_TRUE(BehaviorTree::ShouldSelectFreshExternalAimTarget(
            armor_type, true, false));
    }
    EXPECT_FALSE(BehaviorTree::ShouldSelectFreshExternalAimTarget(
        LangYa::ArmorType::UnKnown, true, false));
}

TEST(ExternalAimTargetPolicyTest, VisibleSelectedTargetRequestsAttackPosture) {
    EXPECT_TRUE(BehaviorTree::ShouldUseVisibleExternalAimForAttackPosture(true));
    EXPECT_FALSE(BehaviorTree::ShouldUseVisibleExternalAimForAttackPosture(false));
}
