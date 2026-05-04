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
│   ├── StrategyManager.hpp     # Hard/Default/Task/Tactical/Finalizer 分層策略調度
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
| `team` | `UnitTeam` | `/ly/me/is_team_red` |
| `enemyOutpostHealth` | `uint16_t` | `/ly/enemy/op_hp` |
| `selfOutpostHealth` | `uint16_t` | `/ly/me/op_hp` |
| `selfBaseHealth` | `uint16_t` | `/ly/me/base_hp` |
| `ammoLeft` | `uint16_t` | `/ly/me/ammo_left` |
| `timeLeft` | `uint16_t` | `/ly/game/time_left` |
| `myselfHealth` | `uint16_t` | `/ly/me/hp` → sentry 血量 |
| `friendRobots` | `Robots` | `/ly/position/data`, `/ly/me/hp` |
| `enemyRobots` | `Robots` | `/ly/enemy/hp`, `/ly/position/data` |
| `teamBuff` | `BuffType` | `/ly/team/buff` |
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
[docs/sentry/decision_visualization_2026-04-27.md](../sentry/decision_visualization_2026-04-27.md)

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
| config.HitBuff=true 且 now_time<25 且 buff次數≤15 | `AimMode::Buff` |
| Buff激活（DefenceBuff或VulnerabilityBuff>20） | `AimMode::RotateScan`（Buff進了就切回掃描） |
| config.HitOutpost=true 且 enemyOutpostHealth>0 且 now_time<90 | `AimMode::Outpost` |
| 其他 | `AimMode::RotateScan` |

#### `SetAimTarget()` — 目標選擇優先級

在普通模式（RotateScan）下，優先級：**英雄 > 步兵1/2（距離近的）> 哨兵 > 工程**

工程師在比賽開始60秒內不打（有無敵保護）。

#### `PublishTogether()` — 匯總並發布（最複雜的函數）

這個函數決定最終發出什麼角度和火控碼：

1. **小陀螺控制**：根據血量下降速度（`healthDecreaseDetector`）和底盤速度（`naviVelocity`），動態設置 `FireCode.Rotate`（0=停止、1-3=不同速度）
2. **FaceMode 優先級**：区域任务启用 FaceMode 时接管云台角，停止云台巡逻扫描，并按 `FaceMode.SuppressFire` 停止新的开火翻转；FaceMode 本身不清零 `FireCode.Rotate`，底盘小陀螺继续由原策略输出。
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
| `/ly/me/is_team_red` | `team` | 我方顏色 |
| `/ly/game/all` | `myselfHealth` | 下位機彙總裁判數據（自血） |
| `/ly/game/is_start` | `is_game_begin` | 比賽開始標誌 |
| `/ly/game/time_left` | `timeLeft` | 剩餘時間 |
| `/ly/me/hp` | `friendRobots` | 我方各車血量明細 |
| `/ly/enemy/hp` | `enemyRobots` | 敵方血量 |
| `/ly/me/ammo_left` | `ammoLeft` | 子彈數 |
| `/ly/enemy/op_hp` | `enemyOutpostHealth` | 敵方前哨血量 |
| `/ly/me/op_hp` | `selfOutpostHealth` | 我方前哨血量 |
| `/ly/me/base_hp` | `selfBaseHealth` | 我方基地血量 |
| `/ly/team/buff` | `teamBuff` | 增益狀態 |
| `/ly/position/data` | `friendRobots`, `enemyRobots`（更新position） | UWB位置 |
| `/ly/gimbal/angles` | `gimbalAngles` | 當前雲台角 |
| `/ly/gimbal/posture` | `postureState` | 姿態回讀（0未知/1進攻/2防禦/3移動） |
| `/ly/gimbal/vel` | `naviVelocity` | 底盤速度反饋 |
| `/ly/predictor/target` | `autoAimData`, `isFindTargetAtomic` | 普通瞄準角度；当前已恢复为老链路语义：消息一到就锁，`autoaim` 侧直接视为可跟随且可开火 |
| `/ly/outpost/target` | `outpostAimData`, `isFindTargetAtomic` | 前哨瞄準角度；当前同样按老链路语义视为可开火 |
| `/ly/buff/target` | `buffAimData`, `isFindTargetAtomic` | 打符瞄準角度 |
| `/ly/face_mode/angles` | `faceModeData` | FaceMode 角度输入；给 BT 内部接管时让 `map_aim_point_node` 输出到这个 topic |
| `/ly/navi/position` | `friendRobots[Sentry].position_` | 导航 TF 反解出的自身官方地图厘米坐标，作为 `/ly/position/data` 之外的补充位置来源 |
| `/ly/navi/reached` | `naviReach` | 導航當前目標是否已到達；外部狀態新鮮且匹配當前目標時優先使用 |
| `/ly/navi/reachable` | `naviReachable` | 導航當前目標是否有有效路徑；超時/未收到/不匹配當前目標時退回內部距離判斷 |
| `/ly/gimbal/capV` | `capV` | 電容電壓 |

安全降級（兼容默認行為）：

- `/ly/position/data` 會做 ID 邊界檢查，非法 `carid` 直接忽略並節流告警。
- 聯盟賽回補判斷只在 `myselfHealth/ammoLeft` 已收到（且可選地未過期）時生效，避免默認值 `0` 誤觸發回補。
- `wait_for_game_start_timeout_sec`、`debug_bypass_is_start` 默認關閉，不改變原始開賽門控行為。

### `src/PublishMessage.cpp` — 所有發布函數

| 發布 Topic | 說明 |
|-----------|------|
| `/ly/control/angles` | 目標雲台角（`gimbalControlData.GimbalAngles`） |
| `/ly/control/firecode` | 火控碼（開火狀態、電容、FollowMode、瞄準模式、旋轉速度） |
| `/ly/control/vel` | 底盤速度指令（導航） |
| `/ly/control/posture` | 姿態指令（0不下發/1進攻/2防禦/3移動） |
| `/ly/aa/enable` | 普通瞄準開關 |
| `/ly/ra/enable` | 打符模式開關 |
| `/ly/outpost/enable` | 前哨站瞄準開關 |
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
  - 聯盟賽會進入顯式策略 `LeagueSimple`，不再借用 `HitHero`
- `competition_profile:=regional`
  - 保持分區賽/原有複雜策略
  - 若未顯式指定 `bt_config_file`，默認讀 `Scripts/ConfigJson/regional_competition.json`
- `bt_config_file:=Scripts/ConfigJson/regional/debug/showcase_competition.json`
  - 展示模式配置：仍走 regional 主流程，但縮短姿態切換等待，並支持短時受擊切防守
  - 展示巡邏點位在 `ShowcasePatrol.Goals` 修改；`DisableTeamOffset=true` 時直接下發基礎點位 ID `0..18`
- `bt_config_file:=Scripts/ConfigJson/regional/debug/navi_debug_competition.json`
  - 導航調試配置：固定走 `NaviTest`，並從 `Scripts/ConfigJson/regional/debug/navi_debug_points.json` 讀臨時點位計劃
  - 支持命名 plan、隨機/順序巡邏、獨立速度等級、可選忽略回血回補
- `bt_config_file:=Scripts/ConfigJson/regional_competition.json`
  - 顯式指定某份 BT JSON 配置
- `wait_for_game_start_timeout_sec:=0`
  - 0=禁用超時（默認）；>0 時超時後跳過 `is_start` 門控（調試用）
- `debug_bypass_is_start:=false`
  - true=直接跳過 `is_start` 門控（調試用，默認 false）
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
  - `ToNavi`：改由 BT 發布 `/ly/navi/target_rel`（x/y/z 相對目標點），導航側負責速度閉環
  - `PreferredDistanceCm`：與目標保持的最適距離（cm）
  - `DistanceDeadbandCm`：距離死區（cm）
  - `DistanceKp` / `MaxForwardSpeed` / `MaxBackwardSpeed`：前後追擊控制
  - `UseYawStrafe` / `YawKp` / `YawDeadbandDeg` / `MaxStrafeSpeed`：側向跟隨控制
  - `LostTargetHoldMs` / `StopWhenNoTarget`：丟目標回退策略

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

`Area.MyArea.Base.Task.MyBase` 管我方 Base 的无特别事件巡游：上游选中我方 Base 大区点时，会从当前坐标最近的 `CastleLeft1`、`CastleLeft2`、`CastleRight2`、`CastleRight1` 中进入循环；拿不到自身坐标时从 `CastleLeft2` 开始。循环顺序固定为 `CastleLeft1 -> CastleLeft2 -> CastleRight2 -> CastleRight1 -> CastleLeft1`，到达判断同样优先使用 `/ly/navi/reached`、`/ly/navi/reachable`。

`Area.MyArea.Roadland.Task.MyRoadland` 管我方 Roadland 的无特别事件区域任务：上游选中我方 Roadland 大区点时，先正常去 `CentralToBase`；到点、不可达或超时后进入强绑定穿越段，打开 `FollowMode + FaceMode`，FaceMode 目标为 `BaseToCentral`，直到到达 `BaseToCentral`、不可达或超时才恢复巡逻/开火/小陀螺。驻守 `BaseToCentral` 时保持普通巡逻和小陀螺；如果明确需要离开或健康/弹量数据低于阈值，则用 `FaceMode(CentralToBase)+FollowMode` 穿回 `CentralToBase`，完成或超时后结束。穿越段不会被受击、识别目标、Buff/Outpost 模式等高优先级逻辑直接取消。

`Area.CommonArea.Central.Task.CommonCentral` 管 Central 公共区域的健康巡逻任务：上游选中 Central 大区点且自身血量/弹量数据新鲜并达到阈值时，从当前坐标最近的巡逻点插入循环。循环顺序为 `my OutpostArea -> my RightShoot -> my BuffAround2 -> my LeftShoot -> my OutpostShoot -> enemy RightShoot -> enemy OccupyArea -> enemy OutpostShoot -> my OutpostArea`。拿不到自身坐标时从 `my OutpostArea` 开始；到达/不可达仍复用 `/ly/navi/reached`、`/ly/navi/reachable`。

区域状态机参数集中在 `config/AreaManager.yaml`：`AreaManager.Switch_Point` 默认 `false`，设为 `true` 时只交换 `Area.hpp` 中红/蓝官方点位和区域边界查找结果，不交换 `team` 语义和导航 goal ID。默认 YAML 不写 `Area.MyArea/EnemyArea/CommonArea` 区域开关，避免覆盖不同 `bt_config_file` 的区域选择；正式 regional 和单区域 areatest 的可选区域仍由 `ConfigJson` 里的 `DecisionAutonomy.NaviGoal.MyArea/EnemyArea/CommonArea` 控制。`AreaManager.RegionalAreaTask.MyHighland/MyBase/MyRoadland/CommonCentral` 管各自区域任务时序，`AreaManager.DefaultPolicy` 管 Default 层的血量/弹量门槛、区域权重、距离惩罚、冷却和重试。区域任务需要动态切换 FaceMode 目标时，BT 发布 `/ly/face_mode/target_raw`，格式为 `[official_map_x, official_map_y, map_z]` cm。

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
模式切換：`/ly/aa/enable`, `/ly/ra/enable`, `/ly/outpost/enable`
目標廣播：`/ly/bt/target`（→ `detector`, `predictor`）
導航：`/ly/navi/*`

---

## 黑板与后续方向

当前实现已使用 BT v4 主树执行，且在运行期维护全局黑板与 tick 黑板。
后续若继续拆分，可沿「感知黑板 / 决策黑板」分层，以降低 `Application` 聚合状态复杂度。
