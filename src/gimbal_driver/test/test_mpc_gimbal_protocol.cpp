#include <array>
#include <cstdint>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>

#include "BasicTypes.hpp"
#include "RawDownlinkTest.hpp"
#include "crc_checker.hpp"
#include "gimbal_driver/msg/gimbal_trajectory.hpp"

namespace {

TEST(MpcGimbalProtocol, RejectsNonFiniteTrajectory)
{
    gimbal_driver::msg::GimbalTrajectory msg;
    msg.yaw = 1.0F;
    msg.pitch = 2.0F;
    msg.yaw_omega = 3.0F;
    msg.pitch_omega = 4.0F;
    msg.yaw_alpha = 5.0F;
    msg.pitch_alpha = 6.0F;
    EXPECT_TRUE(LangYa::IsFiniteGimbalTrajectory(
        msg.yaw, msg.pitch, msg.yaw_omega, msg.pitch_omega,
        msg.yaw_alpha, msg.pitch_alpha));

    msg.pitch_alpha = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(LangYa::IsFiniteGimbalTrajectory(
        msg.yaw, msg.pitch, msg.yaw_omega, msg.pitch_omega,
        msg.yaw_alpha, msg.pitch_alpha));
}

TEST(MpcGimbalProtocol, MapsSixFieldsTo26ByteFrame)
{
    gimbal_driver::msg::GimbalTrajectory msg;
    msg.yaw = 10.0F;
    msg.pitch = -5.0F;
    msg.yaw_omega = 20.0F;
    msg.pitch_omega = -10.0F;
    msg.yaw_alpha = 30.0F;
    msg.pitch_alpha = -15.0F;

    const auto frame = LangYa::ToGimbalTrajectoryFrame(
        msg.yaw, msg.pitch, msg.yaw_omega, msg.pitch_omega,
        msg.yaw_alpha, msg.pitch_alpha);
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
    using LangYa::IsNewerSampleTick;
    EXPECT_TRUE(IsNewerSampleTick(101U, 100U));
    EXPECT_FALSE(IsNewerSampleTick(100U, 100U));
    EXPECT_FALSE(IsNewerSampleTick(99U, 100U));
    EXPECT_TRUE(IsNewerSampleTick(1U, 0xFFFFFFFFU));
}

TEST(MapPathFragmentProtocol, PreservesAllFiftyPointsWithinTwo64ByteFrames)
{
    LangYa::MapPathFrame path;
    path.Intention = 3U;
    path.StartPositionX_dm = 1234U;
    path.StartPositionY_dm = 567U;
    path.SenderId = 107U;
    for (std::size_t index = 0; index < std::size(path.DeltaX_dm); ++index) {
        path.DeltaX_dm[index] = static_cast<std::int8_t>(index - 24);
        path.DeltaY_dm[index] = static_cast<std::int8_t>(24 - index);
    }

    const auto fragments = LangYa::MakeMapPathFragments(path, 0x5AU);
    ASSERT_EQ(fragments.size(), 2U);

    std::array<std::uint8_t, LangYa::kMapPathPayloadBytes> recovered{};
    std::size_t recovered_size = 0;
    for (std::size_t index = 0; index < fragments.size(); ++index) {
        const auto& fragment = fragments[index];
        EXPECT_LE(sizeof(fragment), 64U);
        EXPECT_EQ(fragment.HeadFlag, 0x21U);
        EXPECT_EQ(fragment.DownlinkTypeID, 0x02U);
        EXPECT_EQ(fragment.Sequence, 0x5AU);
        EXPECT_EQ(fragment.FragmentIndex, index);
        EXPECT_EQ(fragment.FragmentCount, 2U);
        EXPECT_EQ(fragment.PayloadLength, index == 0 ? 56U : 49U);
        EXPECT_TRUE(LangYa::IsValidMapPathFragment(fragment));
        std::memcpy(
            recovered.data() + recovered_size,
            fragment.Payload.data(),
            fragment.PayloadLength);
        recovered_size += fragment.PayloadLength;
    }
    EXPECT_EQ(recovered_size, LangYa::kMapPathPayloadBytes);

    LangYa::MapPathFrame reconstructed;
    std::memcpy(
        reinterpret_cast<std::uint8_t*>(&reconstructed) + 2,
        recovered.data(),
        recovered.size());
    EXPECT_EQ(reconstructed.Intention, path.Intention);
    EXPECT_EQ(reconstructed.StartPositionX_dm, path.StartPositionX_dm);
    EXPECT_EQ(reconstructed.StartPositionY_dm, path.StartPositionY_dm);
    EXPECT_EQ(reconstructed.SenderId, path.SenderId);
    EXPECT_EQ(
        0,
        std::memcmp(path.DeltaX_dm, reconstructed.DeltaX_dm, sizeof(path.DeltaX_dm)));
    EXPECT_EQ(
        0,
        std::memcmp(path.DeltaY_dm, reconstructed.DeltaY_dm, sizeof(path.DeltaY_dm)));

    auto corrupted = fragments[1];
    corrupted.Payload[0] ^= 0x01U;
    EXPECT_FALSE(LangYa::IsValidMapPathFragment(corrupted));
}

TEST(MapPathFragmentProtocol, UsesDocumentedCrc16Vector)
{
    constexpr std::array<std::uint8_t, 9> payload{
        '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(
        LangYa::CalculateMapPathFragmentCrc16(payload.data(), payload.size()),
        0x6F91U);
}

template <typename T>
std::array<std::uint8_t, sizeof(T)> RawBytes(const T& frame)
{
    std::array<std::uint8_t, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &frame, bytes.size());
    return bytes;
}

TEST(RawDownlinkTest, AcceptsEachValidPhysicalFrame)
{
    LangYa::GimbalControlFrame control;
    LangYa::SentryCommandFrame sentry_command;
    LangYa::MapPathFrame map_path;
    const auto map_fragment = LangYa::MakeMapPathFragments(map_path, 7U).front();
    LangYa::CustomInfoFrame custom_info;
    LangYa::SentryCoordinateFrame coordinate;
    coordinate.CRC8 = CRCChecker::CRC8::calculate_downlink(
        reinterpret_cast<const std::uint8_t*>(&coordinate), sizeof(coordinate) - 1U);
    const auto trajectory = LangYa::ToGimbalTrajectoryFrame(
        1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);

    EXPECT_TRUE(LangYa::IsValidRawDownlinkTestFrame(0x00U, RawBytes(control)));
    EXPECT_TRUE(LangYa::IsValidRawDownlinkTestFrame(0x01U, RawBytes(sentry_command)));
    EXPECT_TRUE(LangYa::IsValidRawDownlinkTestFrame(0x02U, RawBytes(map_fragment)));
    EXPECT_TRUE(LangYa::IsValidRawDownlinkTestFrame(0x03U, RawBytes(custom_info)));
    EXPECT_TRUE(LangYa::IsValidRawDownlinkTestFrame(0x04U, RawBytes(coordinate)));
    EXPECT_TRUE(LangYa::IsValidRawDownlinkTestFrame(0x05U, RawBytes(trajectory)));
}

TEST(RawDownlinkTest, RejectsMismatchedOrCorruptedPhysicalFrames)
{
    LangYa::GimbalControlFrame control;
    auto control_bytes = RawBytes(control);
    EXPECT_FALSE(LangYa::IsValidRawDownlinkTestFrame(0x01U, control_bytes));
    control_bytes[0] = 0U;
    EXPECT_FALSE(LangYa::IsValidRawDownlinkTestFrame(0x00U, control_bytes));

    control_bytes = RawBytes(control);
    control_bytes[1] = 0x01U;
    EXPECT_FALSE(LangYa::IsValidRawDownlinkTestFrame(0x00U, control_bytes));
    EXPECT_FALSE(LangYa::IsValidRawDownlinkTestFrame(
        0x00U, std::span<const std::uint8_t>{control_bytes.data(), control_bytes.size() - 1U}));

    LangYa::MapPathFrame map_path;
    auto map_fragment = LangYa::MakeMapPathFragments(map_path, 7U).front();
    map_fragment.Payload[0] ^= 0x01U;
    EXPECT_FALSE(LangYa::IsValidRawDownlinkTestFrame(0x02U, RawBytes(map_fragment)));

    LangYa::SentryCoordinateFrame coordinate;
    coordinate.CRC8 = CRCChecker::CRC8::calculate_downlink(
        reinterpret_cast<const std::uint8_t*>(&coordinate), sizeof(coordinate) - 1U);
    coordinate.CRC8 ^= 0x01U;
    EXPECT_FALSE(LangYa::IsValidRawDownlinkTestFrame(0x04U, RawBytes(coordinate)));
}

}  // namespace
