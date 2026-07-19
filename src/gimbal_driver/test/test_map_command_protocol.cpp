#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "BasicTypes.hpp"

namespace {

TEST(MapCommandProtocol, DecodesLowerMachineFixedPointPayload)
{
    LangYa::TypedMessage<sizeof(LangYa::GimbalData)> frame{};
    frame.TypeID = LangYa::MapCommandData::TypeID;
    frame.Data = {
        0x87U, 0x03U, 0x36U, 0x02U, 0x00U, 0x00U,
        0x06U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U};
    frame.Tail = 0x82U;

    const auto& data = frame.GetDataAs<LangYa::MapCommandData>();
    EXPECT_EQ(data.TargetPositionXCentimeter, 903);
    EXPECT_EQ(data.TargetPositionYCentimeter, 566);
    EXPECT_EQ(data.CmdKeyboard, 0U);
    EXPECT_EQ(data.TargetRobotId, 0U);
    EXPECT_EQ(data.CmdSource, 262U);
    EXPECT_EQ(data.Reserved, 0U);
    EXPECT_FLOAT_EQ(data.TargetPositionXMeter(), 9.03F);
    EXPECT_FLOAT_EQ(data.TargetPositionYMeter(), 5.66F);
}

}  // namespace
