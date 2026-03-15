#include "../include/Application.hpp"

using namespace LangYa;

namespace BehaviorTree {
    
    Application::Application(int argc, char **argv) {
        if(InitLogger()) {
            LoggerPtr->Info("Logger Init Success!");
        }else {
            throw std::runtime_error("Logger Init Failed!");
        }
        if (!node.Initialize(argc, argv)) {
            throw std::runtime_error("Failed to initialize ROS node");
        }
        SubscribeMessageAll();
        ConfigurationInit();
        if(!RegisterTreeNodes()) {
            throw std::runtime_error("Behavior Tree Node Register Failed!");
        }
        // if (!LoadBehaviorTree()) {
        //     throw std::runtime_error("Behavior Tree Load Failed!");
        // }
        LoggerPtr->Info("Application Start!");
    }

    Application::~Application() {
        LoggerPtr->Info("Application Stop!");
        LoggerPtr->Flush();
    }

    void Application::Run() {
        WaitBeforeGame();
        gameStartTime = std::chrono::steady_clock::now();
        GameLoop();
    }
}