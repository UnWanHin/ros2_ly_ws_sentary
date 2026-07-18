#include <array>
#include <cstdint>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>

#include "MpcGimbalProtocol.hpp"
#include "crc_checker.hpp"

namespace {

TEST(MpcGimbalProtocol, RejectsNonFiniteTrajectory)
{
    aim_msgs::msg::ControlAngles msg;
    msg.yaw = 1.0F;
    msg.pitch = 2.0F;
    msg.yaw_omega = 3.0F;
    msg.pitch_omega = 4.0F;
    msg.yaw_alpha = 5.0F;
    msg.pitch_alpha = 6.0F;
    EXPECT_TRUE(LangYa::mpc_gimbal_protocol::IsFiniteTrajectory(msg));

    msg.pitch_alpha = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(LangYa::mpc_gimbal_protocol::IsFiniteTrajectory(msg));
}

TEST(MpcGimbalProtocol, MapsSixFieldsTo26ByteFrame)
{
    aim_msgs::msg::ControlAngles msg;
    msg.yaw = 10.0F;
    msg.pitch = -5.0F;
    msg.yaw_omega = 20.0F;
    msg.pitch_omega = -10.0F;
    msg.yaw_alpha = 30.0F;
    msg.pitch_alpha = -15.0F;

    const auto frame = LangYa::mpc_gimbal_protocol::ToTrajectoryFrame(msg);
    EXPECT_EQ(sizeof(frame), 26U);
    EXPECT_EQ(frame.HeadFlag, 0x21U);
    EXPECT_EQ(frame.DownlinkTypeID, 0x05U);
    EXPECT_FLOAT_EQ(frame.Yaw, msg.yaw);
    EXPECT_FLOAT_EQ(frame.Pitch, msg.pitch);
    EXPECT_FLOAT_EQ(frame.YawOmega, msg.yaw_omega);
    EXPECT_FLOAT_EQ(frame.PitchOmega, msg.pitch_omega);
    EXPECT_FLOAT_EQ(frame.YawAlpha, msg.yaw_alpha);
    EXPECT_FLOAT_EQ(frame.PitchAlpha, msg.pitch_alpha);

    std::array<std::uint8_t, sizeof(frame)> bytes{};
    std::memcpy(bytes.data(), &frame, bytes.size());
    EXPECT_EQ(bytes[0], 0x21U);
    EXPECT_EQ(bytes[1], 0x05U);
    EXPECT_EQ(bytes[2], 0x00U);
    EXPECT_EQ(bytes[3], 0x00U);
    EXPECT_EQ(bytes[4], 0x20U);
    EXPECT_EQ(bytes[5], 0x41U);
}

TEST(MpcGimbalProtocol, ValidatesType11CrcAndRejectsCorruption)
{
    LangYa::TypedMessage<sizeof(LangYa::GimbalData)> frame{};
    frame.TypeID = LangYa::GimbalDynamicsData::TypeID;
    auto & data = frame.GetDataAs<LangYa::GimbalDynamicsData>();
    data.YawOmegaDpsX10 = 200;
    data.PitchOmegaDpsX10 = -100;
    data.YawAlphaDps2 = 30;
    data.PitchAlphaDps2 = -15;
    data.SampleTickMs = 1234U;

    CRCChecker::CRC8::append(
        reinterpret_cast<std::uint8_t *>(&frame), sizeof(frame));
    EXPECT_TRUE(CRCChecker::CRC8::verify(
        reinterpret_cast<const std::uint8_t *>(&frame), sizeof(frame)));

    frame.Data[0] ^= 0x01U;
    EXPECT_FALSE(CRCChecker::CRC8::verify(
        reinterpret_cast<const std::uint8_t *>(&frame), sizeof(frame)));
}

TEST(MpcGimbalProtocol, HandlesSampleTickOrderingAndWraparound)
{
    using LangYa::mpc_gimbal_protocol::IsNewerSampleTick;
    EXPECT_TRUE(IsNewerSampleTick(101U, 100U));
    EXPECT_FALSE(IsNewerSampleTick(100U, 100U));
    EXPECT_FALSE(IsNewerSampleTick(99U, 100U));
    EXPECT_TRUE(IsNewerSampleTick(1U, 0xFFFFFFFFU));
}

}  // namespace
