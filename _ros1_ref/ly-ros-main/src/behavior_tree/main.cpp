#include "include/Application.hpp"


int main(int argc, char **argv) try {
    ROS_INFO("main: running %s", BehaviorTree::Application::nodeName);
    BehaviorTree::Application app(argc, argv);
    app.Run();
    return 0;
}
catch (const std::exception &ex) {
    std::cerr << ex.what() << std::endl;
    return 1;
}