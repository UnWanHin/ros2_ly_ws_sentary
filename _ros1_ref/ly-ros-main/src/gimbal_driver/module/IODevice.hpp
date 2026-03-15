#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include <boost/asio.hpp>

#include "ROSTools.hpp"
#include "pp_span.hpp"

namespace LangYa {
template<typename TRead, typename TWrite>
class IODevice {
    boost::asio::io_context IOContext{};
    std::unique_ptr<boost::asio::serial_port> DevicePtr{};

    bool MakeDevice(const std::string_view name) noexcept try {
#pragma region serial_port constants
        using baud_rate_type = boost::asio::serial_port::baud_rate;
        using flow_control_type = boost::asio::serial_port::flow_control;
        using parity_type = boost::asio::serial_port::parity;
        using stop_bits_type = boost::asio::serial_port::stop_bits;
        using character_size_type = boost::asio::serial_port::character_size;
        // 这些赋值语句总不会报错吧……
        static const baud_rate_type BaudRate{115200};
        static const flow_control_type FlowControl{flow_control_type::none};
        static const parity_type Parity{parity_type::none};
        static const stop_bits_type StopBits{stop_bits_type::one};
        static const character_size_type CharacterSize{};
#pragma endregion

        auto dev_ptr = std::make_unique<boost::asio::serial_port>(IOContext);
        std::string dev_name{name};
        dev_ptr->open(dev_name);

        dev_ptr->set_option(BaudRate);
        dev_ptr->set_option(FlowControl);
        dev_ptr->set_option(Parity);
        dev_ptr->set_option(StopBits);
        dev_ptr->set_option(CharacterSize);

        DevicePtr = std::move(dev_ptr);
        return true;
    }
    catch (const std::exception &ex) {
        roslog::error("IODevice::MakeDevice: on device({}), reason: {}", name, ex.what());
        return false;
    }

public:
    IODevice() noexcept = default;

    bool Initialize(bool useVirtualDevice = false) noexcept try {
        roslog::warn("use virtual? {}", useVirtualDevice);

        static constexpr auto namelist = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"};
        if(!useVirtualDevice){
            for (const auto &name: namelist)
                if (MakeDevice(name)) return true;
        }

        // 切换到虚拟设备模式
        static constexpr auto virtual_ports = {"/dev/ttyVirt0", "/dev/ttyVirt1"};
        if (useVirtualDevice) {
            // 创建虚拟串口对（需预装socat）
            int ret = system("sudo socat pty,link=/dev/ttyVirt0,mode=666 pty,link=/dev/ttyVirt1,mode=666 &");
            if (WEXITSTATUS(ret) != 0) {
                roslog::error("create virtual port fail ,error code {}", WEXITSTATUS(ret));
                return false;
            }
            roslog::info("Virtual ports created: /dev/ttyVirt0 <-> /dev/ttyVirt1");

            for (const auto& name : virtual_ports){
                if (MakeDevice(name)) {
                    std::string cmd = "sudo chmod 666 " + std::string(name);
                    if(system(cmd.c_str()) != 0) {
                        roslog::error("Failed to set permissions for virtual port {}, error code: {}", name, WEXITSTATUS(ret));
                        return false;
                    }
                    return true;
                }
            }
        }
        return false;
    }
    catch (const std::exception &ex) {
        roslog::error("IODevice::Initialize: {}", ex.what());
        return false;
    }

    void LoopRead(std::atomic_bool &deviceError, std::function<void(const TRead &)> callback) noexcept try {
        PPBuffer<TRead> BufferRead{};
        auto &pps = BufferRead.Span;
        auto buffer = boost::asio::buffer(pps.pong.data(), pps.pong.size());
        std::array<std::uint8_t, sizeof(TRead)> message_buffer;
        while (!deviceError) {
            const auto bytes = boost::asio::read(*DevicePtr, buffer);
            if (bytes < sizeof(TRead)) throw std::runtime_error("Invalid message size");
            if (!pps.examine(message_buffer)) continue;
            callback(*reinterpret_cast<const TRead *>(message_buffer.data()));
        }
    }
    catch (const std::exception &ex) {
        deviceError = true;
        roslog::error("IODevice::LoopRead: {}", ex.what());
    }

    bool Write(const TWrite &item) noexcept try {
        if (!DevicePtr) return false;
        boost::asio::write(
            *DevicePtr, boost::asio::buffer(reinterpret_cast<const std::uint8_t *>(&item), sizeof(TWrite)));
        return true;
    }
    catch (const std::exception &ex) {
        roslog::error("IODevice::Write: {}", ex.what());
        return false;
    }
};

} // namespace LangYa
