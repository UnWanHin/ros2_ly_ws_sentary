#pragma once

#include "LogPolicy.hpp"

#include <iostream>
#include <fstream>
#include <ctime>
#include <mutex>
#include <vector>
#include <string>
#include <atomic>
#include <iomanip>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <thread>

namespace Utils::Logger {

class FileLogPolicy : public LogPolicy {
public:
    explicit FileLogPolicy(
        const std::string& file_path,
        std::size_t max_pending_messages = 4096);
    ~FileLogPolicy() override;

    void Write(LogLevel level, const std::string& message) override;
    void Flush() override;
    bool IsHealthy() const;
    std::uint64_t DroppedMessageCount() const;

private:
    std::ofstream file_;
    std::string file_path_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<std::string> pending_messages_;
    std::thread thread_;
    bool stop_thread_{false};
    bool healthy_{true};
    const std::size_t max_pending_messages_;
    std::uint64_t dropped_message_count_{0};
    void WriteToFile();
    static std::string FormatMessage(LogLevel level, const std::string& message);
};

}
