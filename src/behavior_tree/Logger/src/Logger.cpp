#include "Utils/Logger/Logger.hpp"

#include <algorithm>

namespace Utils :: Logger {
    Logger::Logger(const std::size_t max_queue_depth)
        : stop_(false), max_queue_depth_(std::max<std::size_t>(1, max_queue_depth)) {
        thread_ = std::thread(&Logger::ProcessQueue, this);
        timer_thread_ = std::thread(&Logger::TimerTask, this);
    }
    Logger::~Logger() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        condition_.notify_all();
        thread_.join();
        timer_thread_.join();
    }

    void Logger::AddPolicy(std::shared_ptr<LogPolicy> policy) {
        std::lock_guard<std::mutex> lock(mutex_);
        policies_.push_back(policy);
    }
    void Logger::Flush() {
        std::vector<std::shared_ptr<LogPolicy>> policies;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            policies = policies_;
        }
        for (auto& policy : policies) {
            policy->Flush();
        }
    }

    std::uint64_t Logger::DroppedMessageCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_message_count_;
    }

    void Logger::FormatMessage(std::ostringstream& oss, const std::string& format) {
        oss << format;
    }

    void Logger::ProcessQueue()  {
        while (true) {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stop_ || !queue_.empty(); });
            if (stop_ && queue_.empty()) {
                return;
            }
            LogMessage message = std::move(queue_.front());
            queue_.pop();
            const auto policies = policies_;
            lock.unlock();

            for (const auto& policy : policies) {
                try {
                    policy->Write(message.level, message.message);
                } catch (...) {
                    // A diagnostic policy must never terminate the BT runtime.
                }
            }
        }
    }
    void Logger::TimerTask() {
        while (!stop_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            Flush();
        }
    }
}
