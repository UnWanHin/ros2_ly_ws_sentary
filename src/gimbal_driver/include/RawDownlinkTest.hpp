#pragma once

#include <cstdint>
#include <cstring>
#include <span>

#include "BasicTypes.hpp"
#include "crc_checker.hpp"

namespace LangYa {

inline bool IsValidRawDownlinkTestFrame(
    const std::uint8_t expected_type_id,
    const std::span<const std::uint8_t> bytes) noexcept {
    if (bytes.size() < 2U || bytes[0] != '!' || bytes[1] != expected_type_id) {
        return false;
    }

    const auto has_size = [&bytes](const std::size_t expected_size) {
        return bytes.size() == expected_size;
    };
    switch (expected_type_id) {
        case GimbalControlFrame::DownlinkTypeIDValue:
            return has_size(sizeof(GimbalControlFrame));
        case SentryCommandFrame::DownlinkTypeIDValue:
            return has_size(sizeof(SentryCommandFrame));
        case MapPathFragmentFrame::DownlinkTypeIDValue: {
            if (!has_size(sizeof(MapPathFragmentFrame))) {
                return false;
            }
            MapPathFragmentFrame frame{};
            std::memcpy(&frame, bytes.data(), sizeof(frame));
            return IsValidMapPathFragment(frame);
        }
        case CustomInfoFrame::DownlinkTypeIDValue:
            return has_size(sizeof(CustomInfoFrame));
        case SentryCoordinateFrame::DownlinkTypeIDValue: {
            if (!has_size(sizeof(SentryCoordinateFrame))) {
                return false;
            }
            SentryCoordinateFrame frame{};
            std::memcpy(&frame, bytes.data(), sizeof(frame));
            return frame.CRC8 == CRCChecker::CRC8::calculate_downlink(
                reinterpret_cast<const std::uint8_t*>(&frame), sizeof(frame) - 1U);
        }
        case GimbalTrajectoryFrame::DownlinkTypeIDValue:
            return has_size(sizeof(GimbalTrajectoryFrame));
        default:
            return false;
    }
}

}  // namespace LangYa
