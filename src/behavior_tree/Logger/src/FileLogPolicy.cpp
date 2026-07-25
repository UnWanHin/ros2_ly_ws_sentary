#include "Utils/Logger/FileLogPolicy.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace Utils::Logger {
    FileLogPolicy::FileLogPolicy(
        const std::string& file_path,
        const std::size_t max_pending_messages)
        : file_path_(file_path),
          max_pending_messages_(std::max<std::size_t>(1, max_pending_messages)) {
        file_.open(file_path, std::ios::out | std::ios::app);
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open log file: " + file_path);
        }
        thread_ = std::thread(&FileLogPolicy::WriteToFile, this);
    }

    FileLogPolicy::~FileLogPolicy() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_thread_ = true;
        }
        condition_.notify_all();
        if(thread_.joinable()) {
            thread_.join();
        }
        if (file_.is_open()) {
            file_.close();
        }
    }

    void FileLogPolicy::Write(Utils::Logger::LogLevel level, const std::string &message) {
        try {
            const auto formatted = FormatMessage(level, message);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!healthy_) {
                    return;
                }
                if (pending_messages_.size() >= max_pending_messages_) {
                    ++dropped_message_count_;
                    return;
                }
                pending_messages_.push_back(formatted);
            }
            condition_.notify_one();
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            healthy_ = false;
            pending_messages_.clear();
        }
    }

    void FileLogPolicy::Flush() {
        condition_.notify_one();
    }

    bool FileLogPolicy::IsHealthy() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return healthy_;
    }

    std::uint64_t FileLogPolicy::DroppedMessageCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_message_count_;
    }

    void FileLogPolicy::WriteToFile() {
        while (true) {
            std::deque<std::string> batch;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] {
                    return stop_thread_ || !pending_messages_.empty() || !healthy_;
                });
                if (!healthy_) {
                    return;
                }
                if (pending_messages_.empty() && stop_thread_) {
                    return;
                }
                batch.swap(pending_messages_);
            }

            try {
                for (const auto& message : batch) {
                    file_.write(message.c_str(), static_cast<std::streamsize>(message.size()));
                }
                file_.flush();
                if (!file_) {
                    throw std::runtime_error("file write failed");
                }
            } catch (...) {
                std::lock_guard<std::mutex> lock(mutex_);
                healthy_ = false;
                dropped_message_count_ += pending_messages_.size();
                pending_messages_.clear();
                std::cerr << "File logger disabled after write failure: " << file_path_ << std::endl;
                return;
            }
        }
    }

    std::string FileLogPolicy::FormatMessage(LogLevel level, const std::string& message) {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        std::tm tm{};
        localtime_r(&now_c, &tm);
        std::ostringstream oss;
        oss << "[" << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setw(3) << std::setfill('0') << now_ms.count() << "] ";
        switch (level) {
            case LogLevel::Info:
                oss << "[INFO] ";
                break;
            case LogLevel::Debug:
                oss << "[DEBUG] ";
                break;
            case LogLevel::Error:
                oss << "[ERROR] ";
                break;
            case LogLevel::Warning:
                oss << "[WARNING] ";
                break;
            case LogLevel::Trace:
                oss << "[TRACE] ";
                break;
        }
        oss << message << std::endl;
        return oss.str();
    }

}
