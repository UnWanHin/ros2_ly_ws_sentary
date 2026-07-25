#include "LogPolicy.hpp"
#include <vector>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <future>
#include <sstream>
#include <chrono>

namespace Utils::Logger {

    class Logger {
    public:
        explicit Logger(std::size_t max_queue_depth = 4096);
        ~Logger();

        void AddPolicy(std::shared_ptr<LogPolicy> policy);

        void Flush();
        std::uint64_t DroppedMessageCount() const;
        template <typename... Args>
        void Log(LogLevel level, const std::string& format, Args... args) {
            std::ostringstream oss;
            FormatMessage(oss, format, args...);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (queue_.size() >= max_queue_depth_) {
                    ++dropped_message_count_;
                    ++pending_drop_notice_count_;
                    return;
                }
                if (pending_drop_notice_count_ > 0 && queue_.size() + 1 < max_queue_depth_) {
                    queue_.push({
                        LogLevel::Warning,
                        "Logger queue recovered; dropped " +
                            std::to_string(pending_drop_notice_count_) + " records."
                    });
                    pending_drop_notice_count_ = 0;
                }
                queue_.push({level, oss.str()});
            }
            condition_.notify_one();
        }


        template <typename... Args>
        void Debug(const std::string& format, Args... args) {
            Log(LogLevel::Debug, format, args...);
        }
        template <typename... Args>
        void Info(const std::string& format, Args... args) {
            Log(LogLevel::Info, format, args...);
        }
        template <typename... Args>
        void Error(const std::string& format, Args... args) {
            Log(LogLevel::Error, format, args...);
        }
        template <typename... Args>
        void Warning(const std::string& format, Args... args) {
            Log(LogLevel::Warning, format, args...);
        }
        template <typename... Args>
        void Trace(const std::string& format, Args... args) {
            Log(LogLevel::Trace, format, args...);
        }

    private:
        struct LogMessage {
            LogLevel level;
            std::string message;
        };

        std::vector<std::shared_ptr<LogPolicy>> policies_;
        mutable std::mutex mutex_;
        std::queue<LogMessage> queue_;
        std::condition_variable condition_;
        std::thread thread_;
        std::thread timer_thread_;
        std::atomic<bool> stop_;
        const std::size_t max_queue_depth_;
        std::uint64_t dropped_message_count_{0};
        std::uint64_t pending_drop_notice_count_{0};

        static void FormatMessage(std::ostringstream& oss, const std::string& format);
        template <typename T, typename... Args>
        void FormatMessage(std::ostringstream& oss, const std::string& format, T value, Args... args) {
            size_t pos = format.find("{}");
            if (pos != std::string::npos) {
                oss << format.substr(0, pos) << value;
                FormatMessage(oss, format.substr(pos + 2), args...);
            } else {
                oss << format;
            }
        }

        void ProcessQueue();
        void TimerTask();
    };

}
