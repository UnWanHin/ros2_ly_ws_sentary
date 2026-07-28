#include <gtest/gtest.h>

#include "SentryMessagePolicy.hpp"

TEST(SentryMessagePolicy, EncodesAsciiAsUtf16LittleEndian) {
    const auto bytes = BehaviorTree::SentryMessage::EncodeUtf8ToUtf16Le("BASE HIT");
    EXPECT_EQ(bytes[0], static_cast<std::uint8_t>('B'));
    EXPECT_EQ(bytes[1], 0U);
    EXPECT_EQ(bytes[14], static_cast<std::uint8_t>('T'));
    EXPECT_EQ(bytes[15], 0U);
}

TEST(SentryMessagePolicy, EncodesFifteenChineseCharactersExactly) {
    const auto bytes = BehaviorTree::SentryMessage::EncodeUtf8ToUtf16Le(
        "一二三四五六七八九十一二三四五");
    EXPECT_NE(bytes[28], 0U);
    EXPECT_NE(bytes[29], 0U);
}

TEST(SentryMessagePolicy, DoesNotSplitUtf16SurrogatePairAtCapacity) {
    const auto bytes = BehaviorTree::SentryMessage::EncodeUtf8ToUtf16Le("12345678901234😀");
    EXPECT_EQ(bytes[28], 0U);
    EXPECT_EQ(bytes[29], 0U);
}

TEST(SentryMessagePolicy, NormalizesBlueRobotIdsToLocalUnitIds) {
    EXPECT_EQ(BehaviorTree::SentryMessage::CanonicalUnitId(101U), 1U);
    EXPECT_EQ(BehaviorTree::SentryMessage::CanonicalUnitId(107U), 7U);
    EXPECT_EQ(BehaviorTree::SentryMessage::CanonicalUnitId(7U), 7U);
}
