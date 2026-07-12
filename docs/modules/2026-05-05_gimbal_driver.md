# gimbal_driver — 雲台驅動節點

Updated: 2026-07-12

## 概述

`gimbal_driver` 是整個系統的**硬件接口層**，負責與底層電控板（stm32/串口）雙向通信：
- **讀取方向**：從串口讀取電控數據（雲台角度、比賽狀態、血量、子彈速度等），解析後發布成多個 ROS2 Topic
- **寫入方向**：訂閱決策節點（`behavior_tree`）發來的控制指令，轉發給電控板

**整個系統的唯一硬件接入點**，所有感知和決策靠這個節點提供狀態。

---

## 目錄結構

```
gimbal_driver/
├── CMakeLists.txt
├── package.xml
├── main.cpp                    # 主節點，Application 類
├── launch/
│   ├── gimbal_driver.launch.py  # ROS 2 主入口（推薦）
│   └── gimbal_driver.launch     # ROS 2 XML 兼容入口
├── config/
│   ├── gimbal_driver_config.yaml # 串口/下位機正式基線
│   └── navigation_test.yaml      # 導航速度直連測試 overlay
├── msg/                        # 自定義消息類型
│   ├── GimbalAngles.msg        # 雲台角度
│   ├── GameData.msg            # 比賽數據（彙總）
│   ├── Health.msg              # 各機器人血量
│   ├── BuffData.msg            # 能量機關增益狀態
│   ├── PositionData.msg        # 機器人位置（UWB）
│   ├── UWBPos.msg              # UWB位置
│   ├── Chassis.msg             # TypeID=6 底盘状态四元
│   └── Vel.msg                 # 速度指令
└── module/
    ├── BasicTypes.hpp          # 底層數據結構定義
    ├── IODevice.hpp            # 串口讀寫封裝
    ├── ROSTools.hpp            # ROS工具（ROSNode、LY_DEF_ROS_TOPIC等）
    ├── crc_checker.hpp         # CRC校驗
    ├── no_ranges_error.hpp     # 範圍錯誤處理
    └── pp_span.hpp             # 數據包解析輔助
```

---

## 單獨啟動（建議）

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py
```

虛擬串口測試：

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py use_virtual_device:=true
```

### 配置歸屬

`src/gimbal_driver/config/gimbal_driver_config.yaml` 是串口、下位機、裁判下行、路徑/自身座標
時效，以及 raw serial 診斷的唯一正式基線。它保留原本的 `io_config` nested key 與既有
`"io_config/..."` flat key，因為 `main.cpp` 需要相容兩種歷史讀法。

正式 `sentry_all.launch.py` 的 gimbal 參數載入順序為：跨模組 `base_config.yaml` →
`gimbal_driver_config.yaml` → 全域 `override_config.yaml` → launch/CLI 顯式覆蓋。單獨啟動
`gimbal_driver.launch.py` 也預設讀同一份 module baseline。`navigation_test.yaml` 只可作
離車調試 overlay，正式鏈路保持 `navigation_test: false`。

兼容 XML 入口：

```bash
ros2 launch gimbal_driver gimbal_driver.launch
```

---

## 核心文件詳解

### `main.cpp` — 主節點（Application 類）

整個節點是一個 `Application` 類，包含一個 `ROSNode` 和一個 `IODevice`（串口設備）。

**啟動流程**：
```
main()
  └── app.Run()
        ├── Node.Initialize(argc, argv)   // 初始化 ROS 節點
        ├── GenSubs()                     // 生成訂閱者（接收控制指令）
        └── 主循環:
              ├── Device.Initialize()     // 嘗試打開串口
              ├── std::jthread: LoopRead()  // 後台持續讀串口
              └── while(!DeviceError): rclcpp::spin_some()  // 處理回調，發送控制數據
```

#### 「讀取」路徑：`LoopRead()` → `Pub*()`

從串口讀到 `TypedMessage`，根據 `TypeID` 分發到不同的 `Pub*` 函數。
當前上行是分型幀模式，實際使用 `TypeID=0..10`：

| TypeID 對應數據結構 | 調用函數 | 發布的 Topic |
|---|---|---|
| `GimbalData` | `PubGimbalData()` | `/ly/gimbal/angles`, `/ly/gimbal/firecode`, `/ly/gimbal/vel`, `/ly/gimbal/capV` |
| `GameData` | `PubGameData()` | `/ly/game/all`, `/ly/game/event_data`, `/ly/friend/ammo_left`, `/ly/friend/is_team_red`, `/ly/game/is_start`, `/ly/game/time_left`, 等；`op_hp` 只做 TypeID 10 不新鲜时的 `*25` fallback |
| `HealthMyselfData` | `PubHealthMyselfData()` | `/ly/friend/hp`, `/ly/friend/base_hp` |
| `HealthEnemyData` | `PubHealthEnemyData()` | `/ly/enemy/hp`, `/ly/enemy/base_hp` |
| `RFIDAndBuffData` | `PubRFIDAndBuffData()` | `/ly/game/rfid`, `/ly/team/buff` |
| `PositionData` | `PubPositionData()` | `/ly/position/data`, `/ly/friend/uwb_pos`, `/ly/bullet/speed` |
| `ChassisData` (`TypeID=6`) | `PubChassisData()` | `/ly/friend/uwb_yaw`, `/ly/gimbal/chassis`（四元浮點）, `/ly/game/damage_difference` |
| `SentryData` (`TypeID=7`) | `PubSentryData()` | `/ly/game/sentry/info`, `/ly/game/bullet`, `/ly/gimbal/posture`（有效 `/ly/game/sentry/info.posture` 覆盖） |
| `BulletDataAndRfid2` (`TypeID=8`) | `PubBulletDataAndRfid2()` | `/ly/game/bullet`, `/ly/game/rfid` |
| `MapCommandData` (`TypeID=9`) | `PubMapCommandData()` | `/ly/game/map_command` |
| `SentryInfo3AndOutpostHpData` (`TypeID=10`) | `PubSentryInfo3AndOutpostHpData()` | `/ly/friend/op_hp`, `/ly/enemy/op_hp`；更新 `/ly/game/sentry/info.sentry_info_3_raw` shadow |

#### 「寫入」路徑：`GenSubs()` / 直接命令發送 → `Device.WriteRaw()`

下行不是上行那套 `TypedMessage<TypeID=...>`。`0x00` 控制 topic 仍寫入共享的
`GimbalControlFrame`，由 `CallbackGenerator` 立即發送；裁判命令、路徑、文字與座標使用各自的
`DownlinkTypeID` 和固定長度 frame。

| DownlinkTypeID | Frame | 長度 | 用途 |
|---|---|---:|---|
| `0x00` | `GimbalControlFrame` | 13B | 雲台角、底盤速度、FireCode |
| `0x01` | `SentryCommandFrame` | 6B | `0x0120 sentry_cmd` |
| `0x02` | `MapPathFrame` | 107B | `0x0307 map_data_t` |
| `0x03` | `CustomInfoFrame` | 36B | `0x0308 custom_info_t` |
| `0x04` | `SentryCoordinateFrame` | 17B | BT 融合後自身座標 |

| 訂閱 Topic | 對應字段 | 說明 |
|---|---|---|
| `/ly/control/angles` (`GimbalAngles`) | `GimbalControlFrame.GimbalAngles.Yaw/Pitch` | 期望雲台角 |
| `/ly/control/firecode` (`FireCode`) | `GimbalControlFrame.FireCode` | 分字段開火/電容/模式/旋轉指令 |
| `/ly/control/vel` (`ControlVelocity`) | `GimbalControlFrame.Velocity.X/Y` | 語義速度；`use_raw=true` 時保留原 int8 下發 |
| `/ly/control/posture` (`SentryCmd`) | `SentryCommandFrame.SentryCmd.Posture` | `0x01` 姿態指令，只使用 `FIELD_POSTURE`；`1~3` 普通、`4~6` 強化姿態 |
| `/ly/control/sentry_cmd` (`SentryCmd`) | `SentryCommandFrame.SentryCmd` | `0x01` 完整哨兵裁判命令入口 |
| `/ly/control/map_path` (`MapPath`) | `MapPathFrame` | `0x02` 裁判 `0x0307` 小地圖路徑 |
| `/ly/control/custom_info` (`CustomInfo`) | `CustomInfoFrame` | `0x03` 裁判 `0x0308` UTF-16 文字 |
| `/ly/bt/sentry_position` (`PointStamped`) | `SentryCoordinateFrame.X_cm/Y_cm` | BT 融合後自身坐標，m 轉 cm 後下發 |

姿態下發採用獨立 `0x01` frame：
- 收到有效姿態命令（1/2/3/4/5/6）後更新 `SentryCmd` shadow 並立即發送
- 姿態切換仍按配置重發
- 失聯重連後會按當前姿態重發

---

### `module/BasicTypes.hpp` — 底層數據結構

定義了所有與電控板通信的二進制數據結構（用 `#pragma pack`/位域）：

| 結構體 | 說明 |
|--------|------|
| `GimbalData` | 電控→上位機：雲台角、速度、開火狀態等 |
| `GimbalControlFrame` | 上位機→電控：`DownlinkTypeID=0x00`，期望雲台角、開火指令、速度 |
| `SentryCommandFrame` | 上位機→電控：`DownlinkTypeID=0x01`，裁判 `0x0120 sentry_cmd` |
| `MapPathFrame` / `CustomInfoFrame` | 上位機→電控：`0x02/0x03`，裁判 `0x0307/0x0308` payload |
| `SentryCoordinateFrame` | 上位機→電控：`DownlinkTypeID=0x04`，哨兵自身 official-map 坐標 |
| `GameData` | 裁判系統數據：比賽狀態、血量、子彈數、時間 |
| `HealthMyselfData` / `HealthEnemyData` | 我方/敵方各機器人血量 |
| `RFIDAndBuffData` | `0x0209 rfid_status` 低 32 位 + `0x0204` 增益數據（防禦、攻擊、回血等） |
| `PositionData` | UWB定位數據（友/敵機器人X、Y座標）+ 子彈速度 |
| `ChassisData` | UWB yaw + 裁判 `0x0003 damage_difference` + 底盘四元（舵角/角速度/x速/y速，8位整数+8位小数） |
| `SentryInfo3AndOutpostHpData` | 裁判 `0x020D sentry_info_3` + 裁判 `0x0003 ally/enemy_outpost_HP` 精确前哨血量 |
| `ExtendData` | `TypeID=7` 预留 12B 扩展帧 |
| `FireCodeType` | 位域：`FireStatus`（開火狀態）、`Rotate`（旋轉速度0-3）、`AimMode`（瞄準模式） |

---

### `module/IODevice.hpp` — 串口設備封裝

```cpp
template<typename ReadType, typename WriteType>
class IODevice {
    bool Initialize(bool useVirtualDevice = false);  // 打開串口（或虛擬迴環）
    bool Write(const WriteType& data);               // 寫入電控
    bool WriteRaw(const OtherType& data);            // 寫入任意二進制幀（通用接口）
    void LoopRead(std::atomic_bool& error, std::function<void(const ReadType&)> callback);  // 持續讀取
};
```

當前 `gimbal_driver` 實例化為：

```cpp
IODevice<TypedMessage<sizeof(GimbalData)>, GimbalControlFrame>
```

含義：
- 上行：讀 `TypedMessage`，依 `TypeID` 分發 `0..9`
- 下行：按 `DownlinkTypeID=0x00~0x04` 發送不同長度 frame；`0x00=13B` 控制、`0x01=6B sentry_cmd`、`0x02=107B` 路徑、`0x03=36B` 自訂訊息、`0x04=17B` 座標

**虛擬設備模式**（`useVirtualDevice=true`）：用於在沒有硬件時做本地迴環測試，通過 `TestVirtualLoopback()` 驗證數據收發。

---

### `module/ROSTools.hpp` — ROS工具宏

`gimbal_driver/module/ROSTools.hpp` 和 `auto_aim_common/include/RosTools/RosTools.hpp` 功能類似，但 `gimbal_driver` 自帶了一份本地版本用於自身的 `MultiCallback` 機制。

**`MultiCallback<GimbalControlFrame>`**：線程安全的多訂閱者回調彙總器。多個訂閱者回調各自更新 `GimbalControlFrame` 的不同字段，最後由回調函數觸發一次 `Device.Write()`。

---

## msg/ 消息格式詳解

### `GimbalAngles.msg`

**Topic**：`/ly/gimbal/angles`（讀取）、`/ly/control/angles`（寫入）

| 字段 | 類型 | 說明 |
|------|------|------|
| `yaw` | `float32` | 當前 yaw 角（度，絕對角） |
| `pitch` | `float32` | 當前 pitch 角（度，向下為正） |

### `GameData.msg`

**Topic**：`/ly/game/all`

| 字段 | 類型 | 說明 |
|------|------|------|
| `gamecode` | `uint16` | 裁判系統遊戲狀態碼（包含比賽開始、哨兵血量、隊伍顏色、英雄警告等位域） |
| `ammoleft` | `uint16` | 剩餘子彈數 |
| `timeleft` | `uint16` | 比賽剩餘時間（秒） |
| `selfhealth` | `uint16` | 自身血量 |
| `exteventdata` | `uint32` | 擴展事件數據（RFID、補給站等） |

### `Health.msg`

**Topic**：`/ly/friend/hp`, `/ly/enemy/hp`

| 字段 | 說明 |
|------|------|
| `hero`, `engineer`, `infantry1`, `infantry2`, `reserve`, `sentry` | 各機器人血量（uint16） |

### `BuffData.msg`

**Topic**：`/ly/team/buff`

| 字段 | 說明 |
|------|------|
| `recoverybuff`, `coolingbuff`, `defencebuff`, `vulnerabilitybuff`, `attackbuff` | 各增益激活標誌（uint8） |
| `remainingenergy` | 剩餘底盤能量（5位，0b10000=5%） |

### `PositionData.msg`

**Topic**：`/ly/position/data`

| 字段 | 說明 |
|------|------|
| `friendcarid`, `friendx`, `friendy` | 友方車輛ID和UWB座標（cm） |
| `enemycarid`, `enemyx`, `enemyy` | 敵方車輛ID和UWB座標（cm） |

---

## Topic 匯總

### 發布的 Topics

| Topic | 消息類型 | 說明 |
|-------|----------|------|
| `/ly/gimbal/angles` | `GimbalAngles` | **最重要**：雲台當前角度，`detector` 和 `behavior_tree` 都需要 |
| `/ly/friend/is_team_red` | `Bool` | 我方是否紅隊 |
| `/ly/game/is_start` | `Bool` | 比賽是否開始 |
| `/ly/game/time_left` | `UInt16` | 剩餘時間 |
| `/ly/friend/hp` | `Health` | 我方各機器人血量 |
| `/ly/enemy/hp` | `Health` | 敵方各機器人血量 |
| `/ly/friend/op_hp` | `UInt16` | 我方前哨血量；优先 TypeID 10 `0x0003 ally_outpost_HP` 精确值，TypeID 1 `SelfOutpostHealth * 25` 只做 fallback |
| `/ly/enemy/op_hp` | `UInt16` | 敵方前哨血量；优先 TypeID 10 `0x0003 enemy_outpost_HP` 精确值，TypeID 1 `EnemyOutpostHealth * 25` 只做 fallback |
| `/ly/friend/ammo_left` | `UInt16` | 剩餘子彈 |
| `/ly/bullet/speed` | `Float32` | 子彈速度（m/s，当前代码发布 `PositionData.BulletSpeed / 100.0f`） |
| `/ly/team/buff` | `BuffData` | 能量機關增益狀態 |
| `/ly/game/rfid` | `RfidStatus` | 0x0209 `rfid_status` 低 32 位拆字段；TypeID 8 的 `rfid_status_2` 也合并在这里 |
| `/ly/position/data` | `PositionData` | UWB位置數據 |
| `/ly/friend/uwb_pos` | `StampedUInt16MultiArray` | 自身UWB位置 `data=[x, y]`，带 `header.stamp` |
| `/ly/gimbal/chassis` | `Chassis` | 底盘四元反馈（`steer_angle`, `angular_velocity`, `velocity_x`, `velocity_y`） |
| `/ly/gimbal/posture` | `UInt8` | 姿態回讀（只由 TypeID 7 `/ly/game/sentry/info.posture` 的有效值發布；僅 1/2/3 視為有效） |
| `/ly/game/sentry/info` | `SentryInfo` | 裁判 `0x020D sentry_info/sentry_info_2/sentry_info_3` 拆字段；`sentry_info_3` shadow 由 TypeID 10 更新，随 TypeID 7 发布 |
| `/ly/game/damage_difference` | `Int16` | 裁判 `0x0003 game_robot_HP_t` offset 8，己方全隊總傷害與對方全隊總傷害之差 |
| `ly/gimbal/eventdata` | `UInt32` | 場地事件原始值（當前 topic 字符串無前導 `/`） |
| `/ly/game/event_data` | `EventData` | 0x0101 `event_data` 按 RM2026 V1.3.0 拆字段 |

### 訂閱的 Topics

| Topic | 消息類型 | 說明 |
|-------|----------|------|
| `/ly/control/angles` | `GimbalAngles` | 接收決策節點的目標角度 |
| `/ly/control/firecode` | `FireCode` | 接收分字段火控指令 |
| `/ly/control/vel` | `ControlVelocity` | 接收語義速度/原始速度指令 |
| `/ly/control/posture` | `SentryCmd` | 接收姿態指令（上位決策輸入，只使用 `FIELD_POSTURE`） |
| `/ly/control/sentry_cmd` | `SentryCmd` | 接收完整哨兵裁判命令 |
| `/ly/control/map_path` | `MapPath` | 接收裁判 `0x0307` 小地圖路徑 |
| `/ly/control/custom_info` | `CustomInfo` | 接收裁判 `0x0308` UTF-16 自訂訊息 |
| `/ly/bt/sentry_position` | `PointStamped` | 接收 BT 融合後自身坐標，用於座標 downlink frame |

### 姿態下發参数

| 字段 | 固定值 | 说明 |
|---|---:|---|
| `repeat_count` | `3` | 每次切换默认重发 3 次 |
| `repeat_interval_ms` | `20` | 重发间隔 20ms |
| `field` | `SentryCommandFrame.SentryCmd.Posture` | 姿态走独立 `DownlinkTypeID=0x01` frame |

---

## 修改注意事項

- **串口協議調整**：修改 `module/BasicTypes.hpp` 的結構體時一定要注意字節對齊和電控端的協議版本一致
- **姿態指令協議策略**：姿態走独立 `SentryCommandFrame`，下位机必须按 ID/长度解析五种下行 frame
- **新增 Topic**：在 `main.cpp` 增加 `LY_DEF_ROS_TOPIC` 定義和對應的 `Pub*()` 函數，並在 `LoopRead()` 的 switch-case 中處理
- **`/ly/bullet/speed`**：当前实现直接发布 `data.BulletSpeed / 100.0f`；若下游表现为固定弹速，优先检查下游是否又做默认值或平滑策略
- **虛擬設備**：調試時可設置 YAML 參數 `io_config/use_virtual_device: true` 來使用迴環模式而無需電控硬件
