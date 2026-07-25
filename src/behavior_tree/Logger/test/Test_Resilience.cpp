#include "Utils/Logger.hpp"

#include <chrono>
#include <memory>
#include <thread>

namespace {

class BlockingPolicy final : public Utils::Logger::LogPolicy {
public:
    void Write(Utils::Logger::LogLevel, const std::string&) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void Flush() override {}
};

}  // namespace

int main() {
    using namespace std::chrono_literals;
    using Utils::Logger::FileLogPolicy;
    using Utils::Logger::Logger;

    // /dev/full accepts open() then reports a write failure. The file policy
    // must disable only itself and return control to the caller.
    FileLogPolicy failing_file{"/dev/full"};
    failing_file.Write(Utils::Logger::LogLevel::Info, "raw logging must not own runtime");
    failing_file.Flush();
    for (int i = 0; i < 100 && failing_file.IsHealthy(); ++i) {
        std::this_thread::sleep_for(5ms);
    }
    if (failing_file.IsHealthy()) {
        return 1;
    }

    Logger logger{4};
    logger.AddPolicy(std::make_shared<BlockingPolicy>());
    for (int i = 0; i < 128; ++i) {
        logger.Info("record {}", i);
    }
    if (logger.DroppedMessageCount() == 0) {
        return 2;
    }
    return 0;
}
