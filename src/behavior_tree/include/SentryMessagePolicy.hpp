// UTF-8 text conversion for the referee custom_info_t payload.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace BehaviorTree::SentryMessage {

constexpr std::size_t kCustomInfoUtf16Bytes = 30;

constexpr std::uint8_t CanonicalUnitId(const std::uint8_t robot_id) noexcept {
    return static_cast<std::uint8_t>(robot_id % 100U);
}

inline std::array<std::uint8_t, kCustomInfoUtf16Bytes> EncodeUtf8ToUtf16Le(
    const std::string_view text) noexcept {
    std::array<std::uint8_t, kCustomInfoUtf16Bytes> bytes{};
    std::size_t input = 0;
    std::size_t output = 0;

    auto append_unit = [&](const std::uint16_t unit) {
        if (output + 2U > bytes.size()) {
            return false;
        }
        bytes[output++] = static_cast<std::uint8_t>(unit & 0xFFU);
        bytes[output++] = static_cast<std::uint8_t>((unit >> 8U) & 0xFFU);
        return true;
    };

    while (input < text.size()) {
        const auto first = static_cast<std::uint8_t>(text[input++]);
        std::uint32_t code_point = 0xFFFDU;
        std::size_t continuation_count = 0;
        if (first < 0x80U) {
            code_point = first;
        } else if ((first & 0xE0U) == 0xC0U) {
            code_point = first & 0x1FU;
            continuation_count = 1;
        } else if ((first & 0xF0U) == 0xE0U) {
            code_point = first & 0x0FU;
            continuation_count = 2;
        } else if ((first & 0xF8U) == 0xF0U) {
            code_point = first & 0x07U;
            continuation_count = 3;
        }

        bool valid = continuation_count == 0 || input + continuation_count <= text.size();
        for (std::size_t index = 0; valid && index < continuation_count; ++index) {
            const auto next = static_cast<std::uint8_t>(text[input + index]);
            if ((next & 0xC0U) != 0x80U) {
                valid = false;
                break;
            }
            code_point = (code_point << 6U) | (next & 0x3FU);
        }
        if (continuation_count > 0 && valid) {
            input += continuation_count;
        }
        if (!valid || code_point > 0x10FFFFU || (code_point >= 0xD800U && code_point <= 0xDFFFU)) {
            code_point = 0xFFFDU;
        }

        if (code_point <= 0xFFFFU) {
            if (!append_unit(static_cast<std::uint16_t>(code_point))) {
                break;
            }
        } else {
            if (output + 4U > bytes.size()) {
                break;
            }
            const auto value = code_point - 0x10000U;
            (void)append_unit(static_cast<std::uint16_t>(0xD800U + (value >> 10U)));
            (void)append_unit(static_cast<std::uint16_t>(0xDC00U + (value & 0x3FFU)));
        }
    }
    return bytes;
}

}  // namespace BehaviorTree::SentryMessage
