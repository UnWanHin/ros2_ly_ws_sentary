# behavior_tree — 決策框架節點

## 概述

`behavior_tree` 是整個系統的**大腦/決策中心**，負責：
1. 匯總所有感知數據（来自 `gimbal_driver`：比賽狀態、血量；来自 `predictor/outpost_hitter/buff_hitter`：瞄準指令）
2. 運行決策邏輯（BehaviorTree.CPP v4 行為樹 + 姿態決策節點）
3. 向 `gimbal_driver` 發送最終的雲台控制指令、開火碼、導航指令

> 当前状态：主循環使用 `rclcpp::ok()`，並在每輪執行 `BTree.tickWhileRunning(...)`。

---

## 目錄結構

```
behavior_tree/
├── CMakeLists.txt
├── package.xml
├── main.cpp                    # 入口，創建 Application 並 Run()
├── Scripts/
│   ├── main.xml                # BT v4 行為樹主文件
│   ├── config.json             # 默認/兼容策略配置文件
│   └── ConfigJson/             # 可切換比賽 profile 配置
│       ├── league_competition.json
│       ├── regional_competition.json
│       ├── regional_simple_competition.json
│       ├── league/             # league 相關測試 preset
│       └── regional/           # regional 調試/展示/單區域 preset
├── include/
│   ├── Application.hpp         # 核心類：所有狀態變量 + 所有函數聲明
│   ├── AreaManager.hpp         # regional 大區域任務與導航區域狀態機
│   ├── StrategyManager.hpp     # Hard/Task/Tactical/Special/Default/Finalizer 分層策略調度
│   ├── Node.hpp                # BT節點定義（BT v4 動作節點/條件節點等）
│   ├── Topic.hpp               # ROS Topic 定義（LY_DEF_ROS_TOPIC宏）
│   └── Robot.hpp               # Robot類/UnitType/UnitTeam 等遊戲數據類型
├── src/
│   ├── Application.cpp         # 構造函數、Run() 流程
│   ├── DecisionTrace.cpp       # 可選 JSONL 決策 trace（離線 decision_viz 使用）
│   ├── GameLoop.cpp            # 主循環邏輯（UpdateBlackBoard/TreeTick/PublishTogether）
│   ├── BehaviorTree.cpp        # BT初始化（RegisterTreeNodes, LoadBehaviorTree）
│   ├── Configuration.cpp       # 讀取 config.json
│   ├── AreaManager.cpp         # regional 大區域任務狀態機實作
│   ├── StrategyManager.cpp     # 分層策略調度實作
│   ├── Logger.cpp              # 日誌初始化
│   ├── PublishMessage.cpp      # 所有發布函數（PubGimbalControlData等）
│   ├── SetPosition.cpp         # 導航位置決策
│   ├── SubscribeMessage.cpp    # 所有訂閱回調（接收感知和遊戲數據）
│   ├── FaceModeManager.cpp     # FaceMode 角度鎖存、啟停判斷與固定朝向目標發布
│   └── WaitBeforeGame.cpp      # 等待比賽開始邏輯
├── module/                     # 工具庫
│   ├── BasicTypes.hpp          # 遊戲數據類型（AimMode、ArmorType、FireCodeType等）
│   ├── ROSTools.hpp            # ROSNode工具
│   ├── Area.hpp                # 地圖區域定義（各個點位的座標）
│   ├── Rate.hpp                # 頻率控制（RateClock、TimerClock）
│   ├── SineWave.hpp            # 正弦波生成（掃描時俯仰角波動）
│   ├── Counter.hpp             # 計數器
│   ├── Random.hpp              # 隨機數生成器
│   └── json.hpp                # nlohmann/json（第三方，953KB）
└── Logger/                     # 日誌系統（第三方或自實現）
    └── （多個頭文件和src）
```

---

## BT v4 依赖策略（2026-03）

- 当前默认 **强制优先使用工作区 pinned BT v4**：
  - `third_party/behaviortree_cpp_v4/install`
  - `src/behavior_tree/CMakeLists.txt:18`
  - `src/behavior_tree/CMakeLists.txt:21`
- 原因：
  - 已确认本机 `/usr/local/lib/libbehaviortree_cpp.so` 会导致 `behavior_tree_node` 在 `rclcpp::Node` 构造阶段离线段错误
  - 工作区 pinned 副本经最小化诊断可正常和 ROS 2 `rclcpp::Node` 共存
- 如需显式切回“系统优先，再 fallback 到工作区”，可在构建时传：
  - `-DBTCPP_FORCE_PINNED=OFF`
- 当前 pinned config 不导出版本字符串，因此 `force pinned` 分支只要求 config 存在，不再强依赖版本变量。
- `package.xml` 仍保留 `behaviortree_cpp` 依赖，便于系统包安装和跨环境兼容：
  - `src/behavior_tree/package.xml`

---

## 核心文件詳解

### `main.cpp` — 入口

```cpp
rclcpp::init(argc, argv);
BehaviorTree::Application app(argc, argv);
app.Run();
rclcpp::shutdown();
```

### `include/Application.hpp` + `src/Application.cpp` — 核心應用類

#### 成員變量（狀態存儲）

**感知數據（從回調函數更新）**：
| 變量 | 類型 | 數據來源 Topic |
|------|------|----------------|
| `team` | `UnitTeam` | `/ly/friend/is_team_red` |
| `enemyOutpostHealth` | `uint16_t` | `/ly/enemy/op_hp` |
| `selfOutpostHealth` | `uint16_t` | `/ly/friend/op_hp` |
| `selfBaseHealth` | `uint16_t` | `/ly/friend/base_hp` |
| `ammoLeft` | `uint16_t` | `/ly/friend/ammo_left` |
| `timeLeft` | `uint16_t` | `/ly/game/time_left` |
| `myselfHealth` | `uint16_t` | `/ly/friend/hp` → sentry 血量 |
| `friendRobots` | `Robots` | `/ly/position/data`, `/ly/friend/hp` |
| `enemyRobots` | `Robots` | `/ly/enemy/hp`, `/ly/position/data` |
| `teamBuff` | `BuffType` | `/ly/team/buff` |
| `rfidMatchState` | `RfidMatchState` | `/ly/game/rfid` 聚合出的 RFID 區域匹配狀態 |
| `armorList` | `array<ArmorData,10>` | `/ly/predictor/target` 等（含距離和類型） |
| `gimbalAngles` | `GimbalAnglesType` | `/ly/gimbal/angles` |
| `naviVelocity` | `VelocityType` | `/ly/gimbal/vel`（底盤速度反饋） |
| `isFindTargetAtomic` | `atomic<bool>` | 当前已恢复为老链路语义：`predictor/outpost/buff` 回调到达即置 true，本轮直接触发锁敌与开火判定 |

**決策輸出數據**：
| 變量 | 說明 |
|------|------|
| `aimMode` | 當前瞄準模式（AutoAim/Buff/Outpost/RotateScan/FaceMode） |
| `targetArmor` | 要打的裝甲板類型 + 距離 |
| `autoAimData` | 普通瞄準的角度數據（來自 predictor） |
| `buffAimData` | 打符的角度數據（來自 buff_hitter） |
| `outpostAimData` | 前哨站角度數據（來自 outpost_hitter） |
| `faceModeData` | FaceMode 固定點朝向角輸入；区域任务需要固定朝向时会读取这一路 |
| `gimbalControlData` | 最終發出的雲台控制數據（角度+火控） |
| `naviCommandGoal` | 導航目標點位（uint8，對應 Area 枚舉） |
| `speedLevel` | 底盤速度等級（0=停、1=正常、2=快） |

#### 構造流程（`Application.cpp`）

```
Application::Application()
├── rclcpp::init（如果還沒init）
├── create_node("behavior_tree")，允許從 launch 注入參數覆蓋
├── InitLogger()              → 初始化日誌（Logger子系統）
├── 從 ament_index 獲取包路徑 → 定位 Scripts/main.xml 和 config.json
├── 可選讀取 `competition_profile` / `bt_config_file` / `bt_tree_file`
├── 可選讀取 `debug_bypass_is_start` / `wait_for_game_start_timeout_sec` / `league_referee_stale_timeout_ms`
├── 可選讀取 `decision_trace_enabled` / `decision_trace_file` / `decision_trace_every_n_ticks`（默認不寫 trace）
├── SubscribeMessageAll()     → 訂閱全部上游 Topics
├── PublishMessageAll()       → 創建全部發布者
├── ConfigurationInit()       → 讀取 config.json
├── RegisterTreeNodes()       → 向 BT Factory 注冊自定義節點
└── LoadBehaviorTree()        → 從 main.xml 創建行為樹
```

#### 運行流程（`Application.cpp` → `Run()`）

```
Run()
├── WaitBeforeGame()  ← 等待比賽開始（默認阻塞，監聽 /ly/game/is_start）
├── gameStartTime = now()
└── GameLoop()        ← 主循環
```

### `src/DecisionTrace.cpp` — 離線決策可視化輸出

`decision_trace_enabled:=true` 且 `decision_trace_file` 非空時，`behavior_tree` 每隔 `decision_trace_every_n_ticks` 個 tick 寫一行 JSONL。該文件由 `src/decision_viz` 離線播放，不改變任何 ROS topic 或決策控制鏈。默認 `decision_trace_enabled:=false`，正常比賽不開檔、不寫 trace。

Trace 會保留 `navi_goal` 原始資料，同時輸出穩定的 `decision_output` 模型。後續決策內部改成新的橋接或策略流程時，viewer 優先看 `decision_output.goal_pos_cm`、output topic、publish flags，而不是直接耦合到某個舊策略欄位。

典型啟動：

```bash
./scripts/start.sh nogate --mode league \
  decision_trace_enabled:=true \
  decision_trace_file:=log/decision_trace.jsonl \
  decision_trace_every_n_ticks:=5
```

維護文檔見：
[docs/sentry/internal/decision_visualization.md](../sentry/internal/decision_visualization.md)

---

### `src/GameLoop.cpp` — 主循環邏輯（最重要的文件）

#### `GameLoop()` — 主循環

```cpp
while (rclcpp::ok()) {
    rclcpp::spin_some(node_);                 // 處理回調
    const auto status = BTree.tickWhileRunning(std::chrono::milliseconds(1));
    treeTickRateClock.sleep();                // 頻率控制
}
```

#### `UpdateBlackBoard()` — 把感知數據寫黑板

把所有感知變量寫入 `BT::Blackboard`：
```cpp
BlackBoard->set<UnitTeam>("MyTeam", team);
BlackBoard->set<uint16_t>("TimeLeft", timeLeft);
BlackBoard->set<uint16_t>("SelfHealth", SelfHealth);
BlackBoard->set<uint16_t>("AmmoLeft", ammoLeft);
BlackBoard->set<Robots>("FriendRobots", friendRobots);
BlackBoard->set<Robots>("EnemyRobots", enemyRobots);
BlackBoard->set<uint16_t>("EnemyOutpostHealth", enemyOutpostHealth);
BlackBoard->set<uint16_t>("SelfOutpostHealth", selfOutpostHealth);
BlackBoard->set("ArmorList", armorList);
BlackBoard->set("TeamBuff", teamBuff);
```

#### `TreeTick()` — BT 根節點調度

```cpp
void TreeTick() {
    if (BTree.subtrees.empty()) return;
    const auto status = BTree.tickWhileRunning(std::chrono::milliseconds(1));
    if (status == BT::NodeStatus::FAILURE) { ... }
}
```

#### `SetAimMode()` — 模式切換邏輯

| 條件 | 設置 |
|------|------|
| `Task.Buff=true` 且 now_time<25 且 buff次數≤15 | `AimMode::Buff` |
| 能量機關已激活（event_data 狀態或 legacy buff 狀態） | `AimMode::RotateScan`（Buff進了就切回掃描） |
| `Task.Outpost=true` 且敵方前哨血量新鮮、大於 0、now_time<90 | 去 `BuffOutpost`；距點小於 `VisualScoutFaceDistanceCm` 後才切 `AimMode::Outpost` |
| `Task.Outpost=true` 且 `VisualScoutWithoutHp=true`、資源/時間/不可達 gate 通過 | 不依賴 `op_hp`，先以普通裝甲模式去 `BuffOutpost`；進入 `VisualScoutFaceDistanceCm` 後開前哨視覺/FaceMode，到點後仍沒有 `/ly/outpost/target` 才退出並 cooldown |
| `AimDebug.ForceBuff=true` / `AimDebug.ForceOutpost=true` | 調試覆蓋到 `AimMode::Buff` / `AimMode::Outpost` |
| 其他 | `AimMode::RotateScan` |

> Regional strategy mode 现在不再自动切到旧单策略点表。`CompetitionProfile=league` 才固定走 `LeagueSimple`；`CompetitionProfile=regional` 固定为 `Regional`，Default 只通过 AreaManager 区域任务输出。

#### `SetAimTarget()` — 目標選擇優先級

在普通模式（RotateScan）下，優先級：**英雄 > 步兵1/2（距離近的）> 哨兵 > 工程**

工程師在比賽開始60秒內不打（有無敵保護）。

#### `PublishTogether()` — 匯總並發布（最複雜的函數）

這個函數決定最終發出什麼角度和火控碼：

1. **小陀螺控制**：根據血量下降速度（`healthDecreaseDetector`）和底盤速度（`naviVelocity`），動態設置 `FireCode.Rotate`（0=停止、1-3=不同速度）；启用 `NaviRotateControl.yaml` 后，新鲜 `/ly/navi/should_rotate=false` 会临时强制 `FollowMode+Rotate=0`，新鲜 `true` 会恢复 BT 正常小陀螺/巡逻，并关闭 regional 区域兼容 FaceMode。
2. **FaceMode 優先級**：`FaceModeManager` 管理角度鎖存、啟停判斷與 `/ly/face_mode/target_raw` 目標發布；`sentry_all.launch.py` 默认拉起 `map_aim_point_node`，用 TF 相对几何把该目标解成 `/ly/face_mode/angles`；区域任务启用 FaceMode 时接管云台角，停止云台巡逻扫描，并按 `FaceMode.SuppressFire` 停止新的开火翻转；FaceMode 本身不清零 `FireCode.Rotate`，底盘小陀螺继续由原策略输出。
3. **FollowMode 優先級**：`FireCode.FollowMode=1` 時停止 rotate、停止巡邏掃描、保持當前雲台角，並停止新的 `FireStatus` 翻轉。
4. **本輪收到目標回調時**：
   - 按 `aimMode` 從對應的 `Aim*Data` 取角度
   - `autoaim/outpost` 不再依賴 `Target.status` 來決定是否翻火控
   - 非 `buff` 模式按 `fireRateClock` 控制翻轉开火
4. **本輪未收到目標回調但距離上次鎖敵未超過 2 秒時**：
   - 沿用最近一次 latched 目標角
5. **超過2秒仍未收到目標時**：
   - 切換到掃描模式，Yaw+3° 偏移，Pitch 用 `SineWave` 做俯仰波動
6. 調用 `PublishMessageAll()` 發出最終指令

---

### `src/SubscribeMessage.cpp` — 所有訂閱回調

| 訂閱 Topic | 更新的變量 | 說明 |
|-----------|-----------|------|
| `/ly/friend/is_team_red` | `team` | 我方顏色 |
| `/ly/game/all` | `myselfHealth` | 下位機彙總裁判數據（自血） |
| `/ly/game/is_start` | `is_game_begin` | 比賽開始標誌 |
| `/ly/game/time_left` | `timeLeft` | 剩餘時間 |
| `/ly/friend/hp` | `friendRobots` | 我方各車血量明細 |
| `/ly/enemy/hp` | `enemyRobots` | 敵方血量 |
| `/ly/friend/ammo_left` | `ammoLeft` | 子彈數 |
| `/ly/enemy/op_hp` | `enemyOutpostHealth` | 敵方前哨血量 |
| `/ly/friend/op_hp` | `selfOutpostHealth` | 我方前哨血量 |
| `/ly/friend/base_hp` | `selfBaseHealth` | 我方基地血量 |
| `/ly/team/buff` | `teamBuff` | 增益狀態 |
| `/ly/game/rfid` | `rfidStatus`, `rfidStatus2`, `rfidMatchState` | 裁判 RFID bit 語義和 BT 內部區域匹配狀態；1s 內未更新則 `RfidFresh=false` |
| `/ly/game/bullet` | `bulletInfo` | TypeID 7/8 合并出的弹速、发射事件、允许发弹量、金币；当前只缓存，不参与正式决策 |
| `/ly/friend/uwb_pos` | `SentryPositionFusion` source | 雷達/UWB 自身坐標，`StampedUInt16MultiArray data=[x,y]` 帶 `header.stamp` |
| `/ly/position/data` | `friendRobots`, `enemyRobots`（更新position） | 通用位置；`friendcarid == Sentry` 會進 `SentryPositionFusion`，其他 friend/enemy 仍直接更新 `friendRobots/enemyRobots`；raw `(0,0)` 視為 unknown，不刷新 BT 狀態 |
| `/ly/gimbal/angles` | `gimbalAngles` | 當前雲台角 |
| `/ly/gimbal/posture` | `postureState` | 姿態回讀（0未知/1進攻/2防禦/3移動） |
| `/ly/gimbal/vel` | `naviVelocity` | 底盤速度反饋 |
| `/ly/predictor/target` | `autoAimData`, `isFindTargetAtomic` | 普通瞄準角度；当前已恢复为老链路语义：消息一到就锁，`autoaim` 侧直接视为可跟随且可开火 |
| `/ly/outpost/target` | `outpostAimData`, `isFindTargetAtomic` | 前哨瞄準角度；当前同样按老链路语义视为可开火 |
| `/ly/buff/target` | `buffAimData`, `isFindTargetAtomic` | 打符瞄準角度 |
| `/ly/face_mode/angles` | `faceModeData` | FaceMode 角度输入；正式 `sentry_all` 中由 `map_aim_point_node` 输出到这个 topic |
| `/ly/navi/position` | `SentryPositionFusion` source | 导航 TF 反解出的自身位置；BT 只消费 `StampedUInt16MultiArray data=[official_x_cm,official_y_cm]`，消息另带 `header.stamp` 和 map 系 `map_point` |
| `/ly/navi/target_official` | `enemyRobots` fallback position | `navi_tf_bridge` 把 `/ly/aim/armor_targets` 中每个 target point 以及当前追击目标 TF 到 map 后反算回 official-map cm；BT 只在没有新鲜非零 `/ly/position/data` 时写入敌方对应 `armor_type` 的位置，并在 `/ly/enemy/info.position_source` 标记 `navi_target_official` |
| `/ly/navi/reached` | `naviReach` | 導航當前目標是否已到達；外部狀態新鮮且匹配當前目標時優先使用 |
| `/ly/navi/reachable` | `naviReachable` | 導航當前目標是否有有效路徑；超時/未收到/不匹配當前目標時退回內部距離判斷 |
| `/ly/navi/should_rotate` | `naviIsRotate` | 外部导航区域兼容旋转控制；true 恢复正常巡逻，false 关闭小陀螺并请求 FollowMode |
| `/ly/gimbal/capV` | `capV` | 電容電壓 |

安全降級（兼容默認行為）：

- `/ly/friend/hp`、`/ly/enemy/hp` 只把 `hp > 0` 的單位寫入 BT 狀態；`0` 視為 unknown，不刷新血量和 freshness。
- `/ly/position/data` 會做 ID 邊界檢查，非法 `carid` 直接忽略並節流告警；`x=0,y=0` 視為 unknown，不刷新位置和 freshness。
- 自身哨兵坐標由 `AreaManager.SentryPositionFusion` 統一輸出到 `friendRobots[Sentry].position_`。`Mode=priority` 時按 `Priority` 選最新鮮的最高優先級源；`Mode=weighted` 時對新鮮源按 `Weight` 加權平均。
- 聯盟賽回補判斷只在 `myselfHealth/ammoLeft` 已收到（且可選地未過期）時生效，避免默認值 `0` 誤觸發回補。
- `wait_for_game_start_timeout_sec`、`debug_bypass_is_start` 默認關閉，不改變原始開賽門控行為。

### `src/PublishMessage.cpp` — 所有發布函數

| 發布 Topic | 說明 |
|-----------|------|
| `/ly/control/angles` | 目標雲台角（`gimbalControlData.GimbalAngles`） |
| `/ly/control/firecode` | 火控碼（開火狀態、電容、FollowMode、瞄準模式、旋轉速度） |
| `/ly/control/vel` | 底盤速度指令（導航） |
| `/ly/control/posture` | 姿態指令，`SentryCmd` 只帶 `FIELD_POSTURE`（0不下發/1進攻/2防禦/3移動） |
| `/ly/vision/mode` | 視覺鏈路模式，`UInt8`：0=DISABLED, 1=ARMOR, 2=BUFF, 3=OUTPOST |
| `/ly/bt/target` | 當前打擊目標類型（→ `detector` 和 `predictor`） |
| `/ly/face_mode/target_raw` | FaceMode 动态目标，`[official_map_x, official_map_y, map_z]` cm |
| `/ly/navi/target_rel` | 追擊相對目標點（x/y/z，供導航側閉環） |
| `/ly/navi/goal` | 導航目標點位 |
| `/ly/navi/goal_pos_raw` | TF bridge 靜態點位輸入 |
| `/goal_pose` | TF bridge 最終導航目標座標（geometry_msgs/PoseStamped） |
| `/ly/navi/goal_pos` | legacy/direct-XY 兼容座標 |
| `/ly/navi/speed_level` | 底盤速度等級 |

### 比賽 Profile 切換

- `competition_profile:=league`
  - 使用聯盟賽簡化決策 profile
  - 若未顯式指定 `bt_config_file`，默認讀 `Scripts/ConfigJson/league_competition.json`
  - 建議搭配 `NaviSetting.UseXY=false`，只發 `/ly/navi/goal`
  - 聯盟賽會進入顯式策略 `LeagueSimple`
- `competition_profile:=regional`
  - 保持分區賽/原有複雜策略
  - 若未顯式指定 `bt_config_file`，默認讀 `Scripts/ConfigJson/regional_competition.json`
  - 進入 `Regional`，不再借用舊單策略點表
- `bt_config_file:=Scripts/ConfigJson/regional/debug/showcase_competition.json`
  - 展示模式配置：仍走 regional 主流程，但縮短姿態切換等待，並支持短時受擊切防守
  - 展示巡邏點位在 `ShowcasePatrol.Goals` 修改；`DisableTeamOffset=true` 時直接下發基礎點位 ID `0..18`
- `bt_config_file:=Scripts/ConfigJson/regional/debug/navi_debug_competition.json`
  - 舊導航調試配置不再接入正式 regional 主鏈路；需要點位調試時應走明確的 navi/areatest 腳本或單獨恢復調試入口
  - 原本的命名 plan、隨機/順序巡邏、獨立速度等級只作 legacy 參考
- `bt_config_file:=Scripts/ConfigJson/regional_competition.json`
  - 顯式指定某份 BT JSON 配置
- `wait_for_game_start_timeout_sec:=0`
  - 0=禁用超時（默認）；>0 時超時後跳過 `is_start` 門控（調試用）
- `debug_bypass_is_start:=false`
  - true=直接跳過 `is_start` 門控（調試用，默認 false）
- `StartGate.AllowGimbalPatrolBeforeStart`
  - YAML 開關在 `src/behavior_tree/config/Task.yaml`；`true` 時 gated 啟動等待 `/ly/game/is_start=true` 期間只壓零底盤速度/小陀螺，雲台仍按 `PatrolScan.Mode` 掃描
- `league_referee_stale_timeout_ms:=0`
  - 0=禁用新鮮度檢查（默認）；>0 時聯盟賽回補會檢查 hp/ammo 回傳是否過期

推薦啟動方式（腳本入口）：

- `./scripts/start.sh gated`
  - 啟動時會交互選擇 `league/regional`（或直接傳 `--mode`）
  - 腳本會自動對齊 `competition_profile` 與 `bt_config_file`
  - 默認分層注入：`scripts/config/base_config.yaml` + `src/*/config/*_config.yaml` + `scripts/config/override_config.yaml`
  - `config_file` 只作為最後覆蓋層，顯式傳入時覆蓋 `override_config.yaml`
- `./scripts/start.sh gated --mode league`
  - 非交互固定聯盟賽
- `./scripts/start.sh gated --mode regional`
  - 非交互固定分區賽
- `./scripts/start.sh showcase`
  - 非交互固定展示模式（regional profile + showcase 配置）
- `./scripts/start.sh nogate --mode regional`
  - 調試入口：固定注入 `debug_bypass_is_start:=true`，可在無裁判 `is_start` 下聯調其他模塊

### 聯盟賽決策（當前實作）

聯盟賽現在是單獨的一條簡化策略分支：

- 策略名：`LeagueSimple`
- 導航輸出：優先用 `/ly/navi/goal`
- 默認目標 ID：`OccupyArea = 3`
- 決策規則只有兩段：
  - 血量低 / 彈藥低：切 `Recovery`
  - 否則：切 `OccupyArea`

補充：

- `OccupyArea` 在 BT 只是目標序號，具體區域隨機點位應由導航側解釋。
- `Area::OccupyArea` 只保留兼容坐標，占位給舊的 `goal_pos` 鏈路，不建議聯盟賽依賴它。
- `bt_tree_file:=Scripts/main.xml`
  - 顯式指定 BT XML 文件

### 追擊配置（新增）

比賽配置文件（`regional_competition.json` / `league_competition.json`）新增：

- `AimTargetPriority`：目標優先級（按 `ArmorType` 整數 ID 排序）
  - 默認：`[1, 3, 4, 6, 2]`（Hero > Infantry1 > Infantry2 > Sentry > Engineer）
- `Chase`：底盤追擊配置
  - `Enable`：總開關
  - `ToNavi`：改由 BT 發布導航追擊輸入，導航側負責速度閉環
  - `UseOfficialPositionSource`：允許用 `/ly/position/data` 的敵方官方坐標作追擊源
  - `PreferOfficialPositionSource`：默認 `false`；`/ly/aim/armor_targets` 追擊點優先走 `/ly/navi/target_rel -> /goal_pose`，官方坐標只作退化來源
  - `OfficialPositionFreshMs`：敵方/自身官方坐標最大有效時間；超時不使用官方源
  - `PreferredDistanceCm`：與目標保持的最適距離（cm）
  - `DistanceDeadbandCm`：距離死區（cm）
  - `AreaLimit`：`ToNavi=true` 的追擊區域限制；`Enable=true` 時啟用追擊限區，`BoundaryMarginCm` 控制離邊界保留距離，自身不在任何已知大區域時保持當前點。`ChaseEnableCrossArea=false` 表示單次追擊只在當前所在大區域內；`true` 表示可追到 `DecisionAutonomy.NaviGoal` 已開啟的大區域內，未開啟的大區域仍不允許。
  - `DistanceKp` / `MaxForwardSpeed` / `MaxBackwardSpeed`：前後追擊控制
  - `UseYawStrafe` / `YawKp` / `YawDeadbandDeg` / `MaxStrafeSpeed`：側向跟隨控制
  - `LostTargetHoldMs` / `StopWhenNoTarget`：丟目標回退策略

`Chase.ToNavi=true` 時現在是多源輸出：

- `/ly/aim/armor_targets` 追擊點有效：使用 `/ly/aim/armor_targets` 中當前選中目標的 point，帶來源 frame（默認 `gimbal_world`）發布 `/ly/navi/target_rel`，由 `navi_tf_bridge` TF 轉 `/goal_pose`；bridge 會按 `Chase.AreaLimit` 和 `ChaseEnableCrossArea` 做大區域限制。
- 同時，`navi_tf_bridge` 會直接訂閱 `/ly/aim/armor_targets`，把 array 裡每個有效 target point 反算成 `/ly/navi/target_official`；BT 對每個 `armor_type` 做敵方位置 fallback 更新，但新鮮 `/ly/position/data` 仍優先。
- `/ly/aim/armor_targets` 追擊點不可用且官方坐標源有效：`targetArmor -> enemyRobots[unit].position_`，結合自身官方坐標按 `PreferredDistanceCm` 留距後，BT 先按同一個 `Chase.AreaLimit` / `ChaseEnableCrossArea` 限制，再發布 `/ly/navi/goal_pos_raw`，由 `navi_tf_bridge` 的 4x4 靜態矩陣轉 `/goal_pose`。
- `Chase.ToNavi=false` 仍是 BT 內部速度追擊，只使用視覺角度/距離計算 `/ly/gimbal/vel`。

### 黑板結構（兩種賽制一致）

- 分區賽與聯盟賽都共用同一套 BT 主循環與雙黑板結構。
- `GlobalBlackboard_`：跨 tick 保留比賽狀態、策略模式、導航目標等。
- `TickBlackboard_`：每 tick 暫存中間決策資料。
- 差異只在策略分派與導航目標選擇，不在黑板架構。

---

### `module/BasicTypes.hpp` — 遊戲數據類型

| 類型 | 說明 |
|------|------|
| `AimMode` | 枚舉：AutoAim/Buff/Outpost/RotateScan/FaceMode；固定点朝向可由 `navi_tf_bridge/map_aim_point_node` 直接发角，也可输出到 `/ly/face_mode/angles` 给 BT 区域任务使用 |
| `ArmorType` | 枚舉：Hero=1, Engineer=2, Infantry1=3, Infantry2=4, Sentry=5, Outpost=7 |
| `UnitType` | 同上，但用於 Robot 對象索引 |
| `UnitTeam` | Red/Blue |
| `GimbalAnglesType` | {Yaw, Pitch}（float） |
| `GimbalControlData` | {GimbalAngles, FireCode, Velocity} |
| `FireCodeType` | 位域：FireStatus, CapState, FollowMode, AimMode, Rotate(2bit) |
| `AimData` | {Angles(YawPitch), FireStatus, BuffFollow, Valid, Fresh, HasLatchedAngles, LastValidTime} |
| `RateClock` | 固定頻率時鐘（用毫秒計時） |
| `TimerClock` | 計時器（用於判斷是否到達某時刻） |
| `DescentDetector<T>` | 下降檢測器，用於血量減少判斷 |

### `module/Area.hpp` — 地圖區域定義

定義了比賽地圖上所有點位（`BuffShoot`、`OutpostShoot`、`MidShoot`、`CastleLeft1`/`CastleLeft2` 等），用 `(x, y)` 座標表示，支持紅/藍兩隊鏡像：

```cpp
SET_POSITION(BuffShoot, MyTeam);  // 設置導航目標為打符點位
```

`Area::BuffShoot.near(x, y, 100, MyTeam)` 判斷當前位置是否在某點位附近100cm內。

当前区域过渡状态机会用 `Highland` 兼容点处理进入/离开我方高地：进入、经由、离开时开启 `FollowMode`；从我方高地回我方基地侧目标会优先经 `CastleLeft1`，到达后关闭 `FollowMode` 再继续原目标。高地兼容到达半径当前为 `DecisionAutonomy.NaviGoal.HighlandCompat.ArriveDistanceCm=20` cm。

`Area.MyArea.Highland.Task.MyHighland` 是区域任务框架的第一条任务：上游选中我方 Highland 大区点时，任务会按 `Highland(FollowMode+FaceMode)` -> `Highland` 短暂停留并恢复巡逻/小陀螺 -> `BuffShoot` 到达后驻守 10 秒 -> `HoleRoad(FollowMode+FaceMode)` 离开。到达判断复用 `/ly/navi/reached`、`/ly/navi/reachable`，外部状态不可用时才回退到自身位置距离。

`Area.MyArea.Base.Task.MyBase` 管我方 Base 的无特别事件巡游：上游选中我方 Base 大区点时，会在 `Base.yaml` 的候选点里按权重和自身距离评分，候选包含 `CastleLeft1`、`CastleLeft2`、`CastleRight2`、`CastleRight1`、`HoleRoad`、`OutpostGuard`、`BuffOutpost`；拿不到自身坐标时只按权重评估。到达判断同样优先使用 `/ly/navi/reached`、`/ly/navi/reachable`。

`Area.MyArea.Roadland.Task.MyRoadland` 管我方 Roadland 的无特别事件区域任务：上游选中我方 Roadland 大区点时，先正常去 `CentralToBase`；到点、不可达或超时后进入强绑定穿越段，打开 `FollowMode + FaceMode`，FaceMode 目标为 `BaseToCentral`，直到到达 `BaseToCentral`、不可达或超时才恢复巡逻/开火/小陀螺。驻守 `BaseToCentral` 时保持普通巡逻和小陀螺；如果明确需要离开或健康/弹量数据低于阈值，则用 `FaceMode(CentralToBase)+FollowMode` 穿回 `CentralToBase`，完成或超时后结束。穿越段不会被受击、识别目标、Buff/Outpost 模式等高优先级逻辑直接取消。

`Area.CommonArea.Central.Task.CommonCentral` 管 Central 公共区域的健康巡逻任务：上游选中 Central 大区点且自身血量/弹量数据新鲜并达到阈值时，从当前坐标最近的巡逻点插入循环。循环顺序为 `my OutpostArea -> my RightShoot -> my BuffAround2 -> my LeftShoot -> my OutpostShoot -> enemy RightShoot -> enemy OccupyArea -> enemy OutpostShoot -> my OutpostArea`。拿不到自身坐标时从 `my OutpostArea` 开始；到达/不可达仍复用 `/ly/navi/reached`、`/ly/navi/reachable`。

区域状态机参数集中在 `src/behavior_tree/config/AreaManager.yaml`：`AreaManager.Switch_Point` 默认 `false`，设为 `true` 时只交换 `Area.hpp` 中红/蓝官方点位和区域边界查找结果，不交换 `team` 语义和导航 goal ID。`AreaManager.SentryPositionFusion` 管自身哨兵三源坐标融合，来源是 `/ly/friend/uwb_pos`、`/ly/position/data` 的 `friendcarid == Sentry` 和 `/ly/navi/position`；`Mode=priority` 按 `Priority` 选择新鲜源，`Mode=weighted` 按 `Weight` 加权平均，`FreshTimeoutMs` 控制坐标新鲜度。`Base.yaml` 管 MyBase patrol 候选点权重。`Task.Buff/Outpost` 开关集中在 `src/behavior_tree/config/Task.yaml`，会覆盖 JSON 同名字段。`Special.MiniRoadland` 開關集中在 `src/behavior_tree/config/Special.yaml`，啟用後在 Tactical 和 Default 之間去己方 `MiniRoadland` 偵察點；`Special.Patrol.GoalHoldSec=0` 時巡邏到點即切下一點。`NaviRotateControl.yaml` 讓外部導航通過 `/ly/navi/should_rotate` 接管 Castle/Roadland/Highland 這類區域兼容的小陀螺/FollowMode/regional FaceMode 交替。`Task.OutpostConfirm.VisualScoutWithoutHp` 讓前哨任務不依賴 `op_hp`，而是去 `BuffOutpost`，路上保持普通裝甲視覺和 Move 姿態，到點後才開前哨視覺、FaceMode 和 Attack 姿態；`VisualScoutHoldMs/CooldownMs` 控制開局窗口內到點後的偵查窗口和冷卻，`PostWindowScoutIntervalSec/PostWindowScoutHoldMs` 控制 120 秒後低優先級週期回看前哨，`ArmorWarningDistanceCm` 控制普通裝甲目標可打斷前哨任務的最遠距離，`PostArmorFaceSearchMs` 控制打車後回前哨 FaceMode 搜索時間；舊 `ArmorInterruptMaxDistanceCm` 仍可讀取作兼容。默认 YAML 不写 `Area.MyArea/EnemyArea/CommonArea` 区域开关，避免覆盖不同 `bt_config_file` 的区域选择；正式 regional 和单区域 areatest 的可选区域仍由 `ConfigJson` 里的 `DecisionAutonomy.NaviGoal.MyArea/EnemyArea/CommonArea` 控制。`AreaManager.RegionalAreaTask.MyHighland/MyBase/MyRoadland/CommonCentral` 管各自区域任务时序，`AreaManager.DefaultPolicy` 管 Default 层的血量/弹量门槛、区域权重、距离惩罚、冷却和重试。RegionalDefense 使用 `/ly/position/data` 的官方场地坐标判定敌方区域，高优先级搜索 Base/Highland/Roadland/Central 威胁，不用 map/odom 坐标混判。区域任务需要动态切换 FaceMode 目标时，BT 发布 `/ly/face_mode/target_raw`，格式为 `[official_map_x, official_map_y, map_z]` cm。正式 regional 不再使用旧单策略点表兜底。

---

## 訂閱/發布 Topic 完整匯總

### 訂閱（共15+個）
來自 `gimbal_driver`（遊戲狀態、血量、雲台角、子彈速度等）
來自 `predictor`（`/ly/predictor/target`）
來自 `outpost_hitter`（`/ly/outpost/target`）
來自 `buff_hitter`（`/ly/buff/target`）
來自外部導航狀態（`/ly/navi/reached`、`/ly/navi/reachable`）

### 發布（共10+個）
控制類：`/ly/control/angles`, `/ly/control/firecode`, `/ly/control/vel`, `/ly/control/posture`
模式切換：`/ly/vision/mode`
目標廣播：`/ly/bt/target`（→ `detector`, `predictor`）
導航：`/ly/navi/*`

---

## 黑板与后续方向

当前实现已使用 BT v4 主树执行，且在运行期维护全局黑板与 tick 黑板。
后续若继续拆分，可沿「感知黑板 / 决策黑板」分层，以降低 `Application` 聚合状态复杂度。
