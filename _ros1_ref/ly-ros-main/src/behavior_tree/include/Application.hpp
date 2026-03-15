#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "Utils/Logger.hpp"

#include "../module/BasicTypes.hpp"
#include "../module/SineWave.hpp"
#include "../module/json.hpp"
#include "../module/Rate.hpp"
#include "../module/Counter.hpp"
#include "../module/Random.hpp"
#include "../module/Area.hpp"

#include "Node.hpp"
#include "Topic.hpp"
#include "Robot.hpp"



using namespace BT;
using namespace LangYa;
using namespace Utils::Logger;
using json = nlohmann::json;

namespace BehaviorTree {

    #define SET_POSITION(area, team) \
    do { \
        naviCommandGoal = LangYa::area(team); \
        naviGoalPosition = BehaviorTree::Area::area(team); \
    } while(0)


class Application {
public:
    inline static constexpr const char nodeName[] = "behavior_tree";
    // inline static constexpr auto behavior_tree_file = "/home/hustlyrm/Desktop/Test/main.xml";
    inline static constexpr auto behavior_tree_file = "/home/hustlyrm/workspace/src/behavior_tree/Scripts/main.xml";
    inline static constexpr auto config_file = "/home/hustlyrm/workspace/src/behavior_tree/Scripts/config.json";

private:
    ROSNode<nodeName> node{};

    SineWave pitch_wave{5.0f, 0.0f, 500ms, std::chrono::steady_clock::now()};

    int buff_shoot_count = 0; // 打符的击打次数

    /// 决策需要的数据
    UnitTeam team{UnitTeam::Red}; // 当前队伍颜色
    std::uint16_t enemyOutpostHealth{0}; // 敌方前哨站血量
    std::uint16_t selfOutpostHealth{0}; // 我方前哨站血量
    std::uint16_t enemyBaseHealth{0};  // 基地血量，分度值100，0-50，2000以下护甲展开
    std::uint16_t selfBaseHealth{0};
    std::uint16_t ammoLeft{0}; // 剩余子弹数，分区赛初始弹丸数量为300
    std::uint16_t timeLeft{0}; // 比赛剩余时间
    std::uint16_t myselfHealth{0}; // 自己的血量
    // HealthMyselfData ourHealth{0}; // 队友的血量
    // HealthEnemyData enemyHealth{0}; // 敌方血量
    // std::array<UWBPositionType, 10> friendPosition{}; // 队友的坐标
    // std::array<int, 10> enemyPositonReliable{};
    // std::array<UWBPositionType, 10> enemyPosition{}; // 敌方目标坐标
    Robots friendRobots; // 己方机器人的信息，包括血量，位置等状态
    Robots enemyRobots; // 敌方机器人的信息，包括血量，位置等状态
    BuffType teamBuff{0}; // 当前的增益情况
    std::uint32_t rfidStatus{0}; // 当前的RFID识别状态
    std::uint32_t extEventData{};
    std::array<ArmorData, 10> armorList; // 辅瞄返回的装甲板序列
    bool is_game_begin{false}; // 比赛开始的标志
    FireCodeType RecFireCode{}; // 云台的火控数据
    std::uint8_t capV;
    std::uint8_t naviLowerHead{false};


    /// 决策修改控制数据需要的前置数据
    std::chrono::steady_clock::time_point gameStartTime{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point lastFoundEnemyTime{std::chrono::steady_clock::now()}; // 根据接收到的目标消息的时间戳更新
    GimbalAnglesType gimbalAngles{0, 0}; // 定义回调，接收当前云台角度
    std::vector<UnitType> reliableEnemyPosuition; // 每次预处理，坐标可靠的敌方坐标
    std::vector<UnitType> hitableTargets; // 每次预处理，不是无敌状态的视野范围内的敌方目标

    /// 决策的控制数据: 云台，火控, 导航, 击打目标
    AimMode aimMode{AimMode::RotateScan};
    // ArmorType targetArmor{ArmorType::Hero}; // 目标装甲板
    ArmorData targetArmor{}; // 目标装甲板，包括距离
    AimData autoAimData{}; // 定义回调，接收的辅瞄云台角度数据
    AimData buffAimData{}; // 定义回调，接收的打符云台角度数据
    AimData outpostAimData{}; // 定义回调，接收的打哨站云台角度数据
    GimbalControlData gimbalControlData{}; /// 发送给云台的角度控制数据，火控数据等
    std::atomic<bool> isFindTargetAtomic{false}; // 在回调函数中，每接收一次消息就会被置为true，然后在发送完控制数据之后置为false

    std::uint8_t naviCommandGoal{0}; // 导航目标
    Area::Point<std::uint16_t> naviGoalPosition{}; // 导航定位目标
    std::uint8_t lastNaviComnamdGoal{0}; // 上一次导航目标
    VelocityType naviVelocity{0, 0}; /// 定义回调，接收导航的速度控制数据
    TimerClock naviCommandIntervalClock{Seconds{10}}, recoveryClock{Seconds{90}}; // 控制间隔，回家时间
    std::uint8_t speedLevel{1}; // 0 没电, 1 正常, 2 快速

    BT::Blackboard::Ptr BlackBoard = BT::Blackboard::create(); // 行为树数据黑板
    BT::BehaviorTreeFactory Factory{}; // 行为树工厂
    BT::Tree BTree{}; // 行为树

    std::shared_ptr<Logger> LoggerPtr; // 日志

    Config config{}; // 配置文件

    RateClock fireRateClock{20}, treeTickRateClock{100}, naviCommandRateClock{2}; // 频率控制
    TimerClock rotateTimerClock{Seconds{2}}; // 旋转时间
    DescentDetector<std::uint16_t>  healthDecreaseDetector{400}; // 血量丢失检测器



    // 订阅消息
    MultiCallback<Application&> callbacks{*this};
    template<typename TTopic>
    void GenSub(auto callback) {
        node.GenSubscriber<TTopic>(callbacks.Generate<TTopic>(callback));
    }
    void SubscribeMessageAll();
    void PrintMessageAll();



    // 发布消息
    void PublishMessageAll();
    void PubAimModeEnableData();
    void PubGimbalControlData();
    void PubAimTargetData();
    void PubNaviControlData();
    void PubNaviGoal();
    void PubNaviGoalPos();


    // 等待比赛开始
    void WaitForGameStart();
    void WaitBeforeGame();

    //  比赛循环
    void GameLoop();

    /**
     * @brief 从黑板获取数据 \n
     */
    template<typename T>
    T GetInfoFromBlackBoard(const std::string &key) {
        T value{};
        bool ValueExist = BlackBoard->get<T>(key, value);
        if (!ValueExist) {
            LoggerPtr->Error("Blackboard key {} not found", key.c_str());
        }
        return value;
    }
    void UpdateBlackBoard();
    void TransportData();
    void PublishTogether();
    void TreeTick();
    void ProcessData();
    bool CheckPositionRecovery();
    void SetPositionRepeat();
    void SetPositionProtect();
    void SetPositionNaviTest();
    void SetPositionHitSentry();
    void SetPositionHitHero();
    void SetAimTarget();
    void SetAimTargetNormal();
    void SetAimMode();
    void CheckDebug();

    // 行为树初始化
    bool LoadBehaviorTree() noexcept;
    bool RegisterTreeNodes();

    // 初始化地图
    void InitMap();

    // 日志初始化
    bool InitLogger();

    // 获取配置文件
    bool ConfigurationInit();


public:
    Application(int argc, char **argv);
    ~Application();
    void Run();
};
}