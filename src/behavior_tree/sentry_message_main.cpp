#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace BehaviorTree {
std::shared_ptr<rclcpp::Node> MakeSentryMessageNode();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(BehaviorTree::MakeSentryMessageNode());
    rclcpp::shutdown();
    return 0;
}
