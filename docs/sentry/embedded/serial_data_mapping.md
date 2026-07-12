# 串口上下行数据映射总表

Updated: 2026-07-12

> 配置归属：`gimbal_driver` 的串口、下位机与 raw 上行诊断基线集中在
> `src/gimbal_driver/config/gimbal_driver_config.yaml`；根目录 `config/base_config.yaml`
> 不再承载 `io_config`。

## 1. 说明

这份文档只描述**当前上位机代码实际实现的串口对接逻辑**，范围以 `gimbal_driver` 为准：

- 上位机 -> 下位机：`src/gimbal_driver/main.cpp` 里的订阅与下发
- 下位机 -> 上位机：`src/gimbal_driver/main.cpp` 里的串口解析与 topic 发布
- 底层结构定义：`src/gimbal_driver/module/BasicTypes.hpp`

不讨论下位机固件内部怎么采集这些值，只说明：

1. 当前串口上到底有哪些结构
2. 每个字段对应什么数据
3. 上位机收到/发出后怎么解析
4. 最后映射到哪些 ROS topic

---

## 2. 总体链路

当前 `gimbal_driver` 的串口模板实例是：

```cpp
IODevice<TypedMessage<sizeof(GimbalData)>, GimbalControlFrame>
```

含义：

- 上行：下位机 -> 上位机，使用 `TypedMessage`
- 下行：上位机 -> 下位机，`GimbalControlFrame` 为默认写类型，其他 `DownlinkTypeID` frame 通过 `WriteRaw()` 写入

也就是说：

1. 上行是**带 `TypeID` 的分型幀**
2. 下行是**带 `DownlinkTypeID` 的可变长度 frame**：`0x00=13B`、`0x01=6B`、`0x02=107B`、`0x03=36B`、`0x04=17B`

上行 `TypeID` 和下行 `DownlinkTypeID` 是独立编号空间，不共用语义。

### 2.1 维护约定

以后所有上下位机通信相关改动，优先维护这份文档，并在同一次改动里同步更新对应模块文档。需要更新的范围包括：

- 串口结构体、字节布局、`TypeID` 分配、字段单位或编码方式
- 新增/删除/重命名 ROS topic、msg 字段、参数开关
- 裁判协议到本仓库串口字段的映射，例如 `0x0208`、`0x020D`、`0x0301/0x0120`、`0x0303`
- 下位机固件需要遵守的主控制幀、回读幀、状态位、命令位约定
- 行为树或其它上位机模块对这些 topic/字段的消费方式

如果只是记录历史调试结论，可以放到 `docs/record/`；如果会影响当前联调接口，以本文档为准。

### 2.2 Raw 串口日志

`gimbal_driver` 可以把当前上下位机串口原始幀写到文件，方便对接下位机时核对 byte：

- 配置入口：`config/common.yaml`
- 默认状态：关闭
- 默认目录：`~/Log/GimbalRaw`
- 文件名：`gimbal_raw_YYYYMMDD_HHMMSS.log`

`common.yaml` 可配置。文件内使用分层 YAML；`sentry_all.launch.py` 的 launch argument 仍保留旧的 flat 名称，方便命令行覆盖。

| key | 默认 | 含义 |
|---|---|---|
| `gimbal_raw.file.enable` | `false` | 文件日志总开关 |
| `gimbal_raw.file.uplink` | `true` | 记录下位机 -> 上位机 TypeID 幀 |
| `gimbal_raw.file.downlink` | `true` | 记录上位机 -> 下位机 downlink frame |
| `gimbal_raw.file.screen` | `false` | 同时输出到 ROS screen 日志 |
| `gimbal_raw.file.flush` | `true` | 每行写入后 flush，便于掉电/崩溃前保留日志 |
| `gimbal_raw.file.dir` | `~/Log/GimbalRaw` | 日志目录 |
| `gimbal_raw.file.type_ids` | `all` | 上行 TypeID 过滤，支持 `all` 或逗号列表如 `1,7,8` |

日志行格式：

```text
time_ns rx 7 size=15 hex="21 07 ..." name=SentryData
time_ns tx control size=13 hex="21 00 ..." reason=control_callback firecode_raw=0
time_ns tx sentry_command size=6 hex="21 01 ..." reason=posture sentry_cmd_raw=0
time_ns tx sentry_coordinate size=17 hex="21 04 ..." reason=sentry_coordinate downlink_type_id=4 x_cm=1230 y_cm=450 crc8=42
```

注意：`gimbal_raw.file.type_ids` 只过滤上行 `TypeID`。下行是否记录只由 `gimbal_raw.file.downlink` 控制。

同一套 raw 数据也可以选择发布成 ROS2 topic，避免每帧转 hex 和写磁盘：

| key | 默认 | 含义 |
|---|---|---|
| `gimbal_raw.topic.enable` | `false` | raw topic 总开关 |
| `gimbal_raw.topic.uplink` | `true` | 发布下位机 -> 上位机 TypeID 幀到 `/ly/log/gimbal_raw_rx` |
| `gimbal_raw.topic.downlink` | `true` | 发布上位机 -> 下位机 downlink frame 到 `/ly/log/gimbal_raw_tx` |
| `gimbal_raw.topic.type_ids` | `all` | 上行 TypeID 过滤，支持 `all` 或逗号列表如 `1,7,8` |

raw topic 使用 `gimbal_driver/msg/GimbalRawFrame`，`data` 是原始二进制 bytes，不是 hex 字符串。`gimbal_driver` 只有在对应 topic 存在 subscriber 时才组包发布；若用 rosbag 录这些 topic，负载会转移到 DDS/rosbag 写盘。

---

## 3. 下行：上位机 -> 下位机

### 3.0 当前下发结构（RM2026 V2.0）

固件先读 byte1 的 `DownlinkTypeID`，再按长度和字段解析：

| ID | frame | 总长度 | ROS 输入/用途 |
|---|---|---:|---|
| `0x00` | `GimbalControlFrame` | 13B | `/ly/control/angles`、`/ly/control/vel`、`/ly/control/firecode` |
| `0x01` | `SentryCommandFrame` | 6B | `/ly/control/posture`、`/ly/control/sentry_cmd`，映射裁判 `0x0120` |
| `0x02` | `MapPathFrame` | 107B | `/ly/control/map_path`，映射裁判 `0x0307` |
| `0x03` | `CustomInfoFrame` | 36B | `/ly/control/custom_info`，映射裁判 `0x0308` |
| `0x04` | `SentryCoordinateFrame` | 17B | `/ly/bt/sentry_position`，坐标 frame 保留 CRC8 |

完整字节布局、CRC8、V2.0 `SentryCmd` 位定义和旧协议迁移规则以
[`docs/sentry/embedded/downlink_control_frame.md`](downlink_control_frame.md) 为准。

### 3.1 历史 V1.3 17B 布局（已废弃；下述 ID/字段均非当前实现）

以下两包描述的是已废弃的 V1.3 布局，仅保留作联调历史对照；不得用于当前固件。

- `0x00`：`GimbalControlFrame`
- `0x01`：`SentryCoordinateFrame`

## 3.1.1 `GimbalControlFrame`（DownlinkTypeID=0x00）

```cpp
struct GimbalControlFrame
{
    std::uint8_t HeadFlag{ '!' };
    std::uint8_t DownlinkTypeID{ 0x00 };
    VelocityType Velocity;
    GimbalAnglesType GimbalAngles;
    FireCodeType FireCode;
    SentryCmdType SentryCmd;
};
```

按当前定义，主控制 frame 长度是 **17B**。

### 3.1.2 主控制 frame 字节布局

| byte offset | 字段 | 类型 | 来源 | 当前上位机写法 |
|---|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定值 | `'!'` / `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定值 | `0x00` |
| 2 | `Velocity.X` | `int8` | `/ly/control/vel.raw_x` 或 `x_mps` 编码 | `use_raw=true` 直接写；否则按 `velocity_raw_to_mps` 编码 |
| 3 | `Velocity.Y` | `int8` | `/ly/control/vel.raw_y` 或 `y_mps` 编码 | `use_raw=true` 直接写；否则按 `velocity_raw_to_mps` 编码 |
| 4~7 | `GimbalAngles.Yaw` | `float` | `/ly/control/angles.yaw` | 直接写 `float` |
| 8~11 | `GimbalAngles.Pitch` | `float` | `/ly/control/angles.pitch` | 直接写 `float` |
| 12 | `FireCode` | `uint8` | `/ly/control/firecode` 分字段 | `FireCode` msg 组包；partial 字段 100ms 超时退回 0 |
| 13~16 | `SentryCmd` | `uint32` | `/ly/control/posture` 或 `/ly/control/sentry_cmd` | little-endian，映射裁判 `0x0120 sentry_cmd` |

### 3.1.3 主控制 ROS 输入与字段映射

| ROS topic | ROS 消息字段 | 串口字段 | 备注 |
|---|---|---|---|
| `/ly/control/angles` | `yaw` | `GimbalAngles.Yaw` | 云台目标 yaw |
| `/ly/control/angles` | `pitch` | `GimbalAngles.Pitch` | 云台目标 pitch |
| `/ly/control/vel` | `x_mps/raw_x/use_raw` | `Velocity.X` | 语义速度或原始 int8 |
| `/ly/control/vel` | `y_mps/raw_y/use_raw` | `Velocity.Y` | 语义速度或原始 int8 |
| `/ly/control/firecode` | `fire_status/cap_state/follow_mode/aim_mode/rotate/field_mask` | `FireCode` | 分字段组包 |
| `/ly/control/posture` | `field_mask/posture` | `SentryCmd.Posture` | BT 姿态切换主入口，消息类型为 `SentryCmd`，只使用 `FIELD_POSTURE` |
| `/ly/control/sentry_cmd` | `confirm/revive/exchange/posture/energy` | `SentryCmd` | 分字段组包，直接对应裁判 `0x0120` |

### 3.1.4 `SentryCoordinateFrame`（DownlinkTypeID=0x01）

```cpp
struct SentryCoordinateFrame
{
    std::uint8_t HeadFlag{ '!' };
    std::uint8_t DownlinkTypeID{ 0x01 };
    std::int16_t X_cm{ 0 };
    std::int16_t Y_cm{ 0 };
    std::uint8_t Reserved[10]{ 0 };
    std::uint8_t CRC8{ 0 };
};
```

| byte offset | 字段 | 类型 | 来源 | 当前上位机写法 |
|---|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定值 | `'!'` / `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定值 | `0x01` |
| 2~3 | `X_cm` | `int16` | `/ly/bt/sentry_position.point.x` | m 转 cm，clamp 到 `sentry_coord_field_width_x` |
| 4~5 | `Y_cm` | `int16` | `/ly/bt/sentry_position.point.y` | m 转 cm，clamp 到 `sentry_coord_field_width_y` |
| 6~15 | `Reserved` | `uint8[10]` | 固定值 | `0` |
| 16 | `CRC8` | `uint8` | byte0~15 | poly `0x31`，init `0xFF`，非反射 |

相关参数：

- `io_config/sentry_coord_send_interval_ms`：默认 `100`
- `io_config/sentry_coord_field_width_x`：默认 `2800`
- `io_config/sentry_coord_field_width_y`：默认 `1500`
- `io_config/sentry_coord_fresh_timeout_ms`：默认 `2000`

### 3.1.5 `FireCode` 位定义

`FireCode` 在代码里是位域结构，但下发时上位机按**整字节**写入：

| bit | 名称 | 含义 |
|---|---|---|
| 0~1 | `FireStatus` | 开火位，代码注释约定为翻转触发 |
| 2~3 | `CapState` | 电容状态 |
| 4 | `FollowMode` | 跟随模式 |
| 5 | `AimMode` | 瞄准模式 |
| 6~7 | `Rotate` | 小陀螺档位 |

`behavior_tree` 发布 `FollowMode=1` 时，现在只改 `FireCode.FollowMode` 这个 bit；不会因为该 bit 自动把 `Rotate` 压到 `0`、关闭 `AimMode`、停止新的 `FireStatus` 翻转或保持当前云台角度。小陀螺、FaceMode 和停火分别由 `Rotate`、FaceMode、`SuppressFire`/开火逻辑独立控制。

### 3.1.6 `SentryCmd` 位定义

`SentryCmd` 是 4B 命令字，按 little-endian 写入 `GimbalControlFrame` byte `13~16`：

| bit | 名称 | 含义 |
|---|---|---|
| 0 | `ConfirmFreeRevive` | 确认免费复活 |
| 1 | `ConfirmImmediateRevive` | 确认兑换立即复活 |
| 2~12 | `ExchangeProjectileAllowance` | 非远程兑换允许发弹量累计值 |
| 13~16 | `RemoteProjectileExchangeCount` | 远程兑换发弹量请求次数 |
| 17~20 | `RemoteHpExchangeCount` | 远程兑换血量请求次数 |
| 21~22 | `Posture` | `0=保留, 1=进攻, 2=防御, 3=移动` |
| 23 | `ConfirmEnergyActivate` | 确认己方能量机关进入正在激活状态 |
| 24~31 | `Reserved` | 保留 |

### 3.1.7 `Posture` 当前规则

当前代码行为：

1. BT 姿态主链路发布 `/ly/control/posture`，消息类型为 `gimbal_driver/msg/SentryCmd`
2. `/ly/control/sentry_cmd` 仍可直接写 `SentryCmd`，包含姿态、复活、兑弹、远程回血、能量机关确认等字段
3. `0` 允许写入，表示“不请求姿态切换 / 保留值”
4. `1/2/3` 为有效姿态
5. 有效姿态会按参数做重发

当前重发参数默认值：

- `io_config/posture_repeat_count = 3`
- `io_config/posture_repeat_interval_ms = 20`

---

## 4. 上行：下位机 -> 上位机

## 4.1 当前上行总封装

上行读取类型是：

```cpp
TypedMessage<sizeof(GimbalData)>
```

按当前代码，`GimbalData` payload 是 12B，因此当前上行总封装可理解为：

| 字段 | 长度 |
|---|---:|
| `HeadFlag` | 1B |
| `TypeID` | 1B |
| `Data` | 12B |
| `Tail` | 1B |

总长度：**15B**

### 4.1.1 `TypeID` 分发表

| TypeID | 结构体 | 解析函数 |
|---|---|---|
| `0` | `GimbalData` | `PubGimbalData()` |
| `1` | `GameData` | `PubGameData()` |
| `2` | `HealthMyselfData` | `PubHealthMyselfData()` |
| `3` | `HealthEnemyData` | `PubHealthEnemyData()` |
| `4` | `RFIDAndBuffData` | `PubRFIDAndBuffData()` |
| `5` | `PositionData` | `PubPositionData()` |
| `6` | `ChassisData` | `PubChassisData()` |
| `7` | `SentryData` | `PubSentryData()` |
| `8` | `BulletDataAndRfid2` | `PubBulletDataAndRfid2()` |
| `9` | `MapCommandData` | `PubMapCommandData()` |
| `10` | `SentryInfo3AndOutpostHpData` | `PubSentryInfo3AndOutpostHpData()` |

---

## 5. 各 `TypeID` 的字段、解析与 topic

## 5.1 `TypeID=0` - `GimbalData`

结构：

```cpp
struct GimbalData
{
    GimbalAnglesType GimbalAngles;
    VelocityType Velocity;
    FireCodeType FireCode;
    std::uint8_t CapV;
};
```

### 5.1.1 字段映射

| 串口字段 | 解析方式 | 发布 topic | ROS 字段 |
|---|---|---|---|
| `GimbalAngles.Yaw` | 直接读 `float` | `/ly/gimbal/angles` | `yaw` |
| `GimbalAngles.Pitch` | 直接读 `float` | `/ly/gimbal/angles` | `pitch` |
| `Velocity.X` | 直接读 `int8` | `/ly/gimbal/vel` | `x` |
| `Velocity.Y` | 直接读 `int8` | `/ly/gimbal/vel` | `y` |
| `FireCode` | 位字段拆解并保留 `raw` | `/ly/gimbal/firecode` | `fire_status/cap_state/follow_mode/aim_mode/rotate/raw` |
| `CapV` | 直接读 `uint8` | `/ly/gimbal/capV` | `data` |

---

## 5.2 `TypeID=1` - `GameData`

结构：

```cpp
struct GameData
{
    GameCodeType GameCode;
    std::uint16_t AmmoLeft;
    std::uint16_t TimeLeft;
    std::uint16_t SelfHealth;
    std::uint32_t ExtEventData;
};
```

### 5.2.1 `GameCode` 位定义

| bit | 字段 | 当前含义 |
|---|---|---|
| 0 | `IsGameBegin` | 比赛开始标志 |
| 1 | `HeroPrecaution` | 英雄预警 |
| 2 | `IsMyTeamRed` | 我方是否红方 |
| 3~8 | `EnemyOutpostHealth` | 敌方前哨站血量分度值 |
| 9~14 | `SelfOutpostHealth` | 我方前哨站血量分度值 |
| 15 | `IsReturnedHome` | 是否回家 |

当前代码对这里的前哨站血量只做兼容 fallback：

- 如果 `TypeID=10` 精确前哨站血量未收到或超过 1500ms 未更新，`EnemyOutpostHealth * 25` -> `/ly/enemy/op_hp`
- 如果 `TypeID=10` 精确前哨站血量未收到或超过 1500ms 未更新，`SelfOutpostHealth * 25` -> `/ly/friend/op_hp`
- `TypeID=10` 新鲜时，不允许 `TypeID=1` 的 `*25` 旧值覆盖 `/ly/friend/op_hp` 和 `/ly/enemy/op_hp`

### 5.2.2 `ExtEventData` 位定义

`BasicTypes.hpp` 中定义了位域。当前 `gimbal_driver` 会同时发布原始值和 `/ly/game/event_data` 语义拆字段；但并不是每个字段都已经被 `behavior_tree` 用作决策条件：

| bit | 字段 |
|---|---|
| 0~2 | `SelfSupplyStatus` |
| 3~4 | `SelfSmallEnergyStatus` |
| 5~6 | `SelfLargeEnergyStatus` |
| 7~8 | `SelfCentralHighlandStatus` |
| 9~10 | `SelfTrapezoidHighlandStatus` |
| 11~19 | `EnemyLastDartHitTime` |
| 20~22 | `EnemyLastDartHitTarget` |
| 23~24 | `CenterGainPointStatus` |
| 25~26 | `SelfFortressGainPointStatus` |
| 27~28 | `SelfOutpostGainPointStatus` |
| 29 | `SelfBaseGainPointStatus` |
| 30~31 | `Reserved` |

### 5.2.3 字段映射

| 串口字段 | 解析方式 | 发布 topic | ROS 字段 / 备注 |
|---|---|---|---|
| `GameCode` | 整体重解释成 `uint16` | `/ly/game/all` | `gamecode` |
| `AmmoLeft` | 直接读 `uint16` | `/ly/game/all` | `ammoleft` |
| `AmmoLeft` | 直接读 `uint16` | `/ly/friend/ammo_left` | `data` |
| `TimeLeft` | 直接读 `uint16` | `/ly/game/all` | `timeleft` |
| `TimeLeft` | 直接读 `uint16` | `/ly/game/time_left` | `data` |
| `SelfHealth` | 直接读 `uint16` | `/ly/game/all` | `selfhealth` |
| `ExtEventData` | 整体转 `uint32` | `/ly/game/all` | `exteventdata` |
| `ExtEventData` | 整体转 `uint32` | `ly/gimbal/eventdata` | `data`，注意当前 topic 字符串无前导 `/` |
| `ExtEventData` | 按 V1.3.0 bit 拆字段 | `/ly/game/event_data` | `EventData` |
| `GameCode.EnemyOutpostHealth` | `* 25` fallback | `/ly/enemy/op_hp` | 仅在 `TypeID=10` 精确前哨血量未收到或超过 1500ms 未更新时发布 |
| `GameCode.HeroPrecaution` | 直接读 bit | `/ly/friend/is_precaution` | `data` |
| `GameCode.IsGameBegin` | 直接读 bit | `/ly/game/is_start` | `data` |
| `GameCode.IsMyTeamRed` | 直接读 bit | `/ly/friend/is_team_red` | `data` |
| `GameCode.IsReturnedHome` | 直接读 bit | `/ly/friend/is_at_home` | `data` |
| `GameCode.SelfOutpostHealth` | `* 25` fallback | `/ly/friend/op_hp` | 仅在 `TypeID=10` 精确前哨血量未收到或超过 1500ms 未更新时发布 |

---

## 5.3 `TypeID=2` - `HealthMyselfData`

结构：

```cpp
struct HealthMyselfData {
    std::uint16_t HeroMyself;
    std::uint16_t EngineerMyself;
    std::uint16_t Infantry1Myself;
    std::uint16_t Infantry2Myself;
    std::uint16_t BaseMyself;
    std::uint16_t SentryMyself;
};
```

### 5.3.1 字段映射

| 串口字段 | 发布 topic | ROS 字段 |
|---|---|---|
| `HeroMyself` | `/ly/friend/hp` | `hero` |
| `EngineerMyself` | `/ly/friend/hp` | `engineer` |
| `Infantry1Myself` | `/ly/friend/hp` | `infantry1` |
| `Infantry2Myself` | `/ly/friend/hp` | `infantry2` |
| `BaseMyself` | `/ly/friend/hp` | `reserve` |
| `SentryMyself` | `/ly/friend/hp` | `sentry` |
| `BaseMyself` | `/ly/friend/base_hp` | `data` |

注意：当前 `Health.msg` 的 `reserve` 字段，实际上被上位机填的是 `BaseMyself`。

---

## 5.4 `TypeID=3` - `HealthEnemyData`

结构：

```cpp
struct HealthEnemyData {
    std::uint16_t HeroEnemy;
    std::uint16_t EngineerEnemy;
    std::uint16_t Infantry1Enemy;
    std::uint16_t Infantry2Enemy;
    std::uint16_t BaseEnemy;
    std::uint16_t SentryEnemy;
};
```

### 5.4.1 字段映射

| 串口字段 | 发布 topic | ROS 字段 |
|---|---|---|
| `HeroEnemy` | `/ly/enemy/hp` | `hero` |
| `EngineerEnemy` | `/ly/enemy/hp` | `engineer` |
| `Infantry1Enemy` | `/ly/enemy/hp` | `infantry1` |
| `Infantry2Enemy` | `/ly/enemy/hp` | `infantry2` |
| `BaseEnemy` | `/ly/enemy/hp` | `reserve` |
| `SentryEnemy` | `/ly/enemy/hp` | `sentry` |
| `BaseEnemy` | `/ly/enemy/base_hp` | `data` |

注意：当前 `Health.msg` 的 `reserve` 字段，实际上被上位机填的是 `BaseEnemy`。

---

## 5.5 `TypeID=4` - `RFIDAndBuffData`

结构：

```cpp
struct BuffType{
    std::uint8_t reserve;
    std::uint8_t RecoveryBuff;
    std::uint8_t CoolingBuff;
    std::uint8_t DefenceBuff;
    std::uint8_t VulnerabilityBuff;
    std::uint16_t AttackBuff;
    std::uint8_t RemainingEnergy;
};

struct RFIDAndBuffData{
    BuffType BuffStatus;
    std::uint32_t RFIDStatus; // 0x0209 rfid_status（低 32 位，bit0-31）
};
```

### 5.5.1 字段映射

| 串口字段 | 发布 topic | ROS 字段 / 备注 |
|---|---|---|
| `RFIDStatus` | `/ly/game/rfid` | `RfidStatus` 拆字段，TypeID=4 覆盖 bit0-31；TypeID=8 补 `rfid_status_2` |
| `BuffStatus.RecoveryBuff` | `/ly/team/buff` | `recoverybuff` |
| `BuffStatus.CoolingBuff` | `/ly/team/buff` | `coolingbuff` |
| `BuffStatus.DefenceBuff` | `/ly/team/buff` | `defencebuff` |
| `BuffStatus.VulnerabilityBuff` | `/ly/team/buff` | `vulnerabilitybuff` |
| `BuffStatus.AttackBuff` | `/ly/team/buff` | `attackbuff` |
| `BuffStatus.RemainingEnergy` | `/ly/team/buff` | `remainingenergy` |

### 5.5.2 `RFIDStatus` 位语义（RM2026 V1.3.0，0x0209 bit0-31）

| bit | ROS 字段 | 含义 |
|---|---|---|
| `0` | `friend_base` | 己方基地增益点 |
| `1` | `friend_central` | 己方中央高地增益点 |
| `2` | `enemy_central` | 对方中央高地增益点 |
| `3` | `friend_highland` | 己方梯形高地增益点 |
| `4` | `enemy_highland` | 对方梯形高地增益点 |
| `5` | `friend_flyroad_front` | 己方飞坡前（靠近己方一侧） |
| `6` | `friend_flyroad_back` | 己方飞坡后（靠近己方一侧） |
| `7` | `enemy_flyroad_front` | 对方飞坡前（靠近对方一侧） |
| `8` | `enemy_flyroad_back` | 对方飞坡后（靠近对方一侧） |
| `9` | `friend_central_under` | 己方中央高地下方 |
| `10` | `friend_central_high` | 己方中央高地上方 |
| `11` | `enemy_central_under` | 对方中央高地下方 |
| `12` | `enemy_central_high` | 对方中央高地上方 |
| `13` | `friend_roadland_under` | 己方公路下方 |
| `14` | `friend_roadland_high` | 己方公路上方 |
| `15` | `enemy_roadland_under` | 对方公路下方 |
| `16` | `enemy_roadland_high` | 对方公路上方 |
| `17` | `friend_bastion` | 己方堡垒增益点 |
| `18` | `friend_outpost` | 己方前哨站增益点 |
| `19` | `friend_supply_noremix` | 己方与资源区不重叠的补给区 / RMUL 补给区 |
| `20` | `friend_supply_remix` | 己方与资源区重叠的补给区 |
| `21` | `friend_armor` | 己方装配增益点 |
| `22` | `enemy_armor` | 对方装配增益点 |
| `23` | `central_rmul` | 中心增益点（仅 RMUL 适用；ROS 字段名使用小写） |
| `24` | `enemy_bastion` | 对方堡垒增益点 |
| `25` | `enemy_outpost` | 对方前哨站增益点 |
| `26` | `friend_tunnel_roadland_down` | 己方隧道下方（靠近己方公路区） |
| `27` | `friend_tunnel_roadland_mid` | 己方隧道中间（靠近己方公路区） |
| `28` | `friend_tunnel_roadland_up` | 己方隧道上方（靠近己方公路区） |
| `29` | `friend_tunnel_highland_low` | 己方隧道较低处（靠近己方梯形高地） |
| `30` | `friend_tunnel_highland_mid` | 己方隧道较中间（靠近己方梯形高地） |
| `31` | `friend_tunnel_highland_high` | 己方隧道较高处（靠近己方梯形高地） |

注意：

- `BuffStatus.reserve` 当前没有被发布
- `0x0209` 的 `rfid_status_2`（额外 8 bit）不并入 `TypeID=4`，由 `TypeID=8` 承载
- `/ly/game/rfid` 的 `RfidStatus` 包含 `has_rfid_status_2`、`rfid_status_2_raw` 和 bit0-5 的语义字段；收到 TypeID=8 后会把 `rfid_status_2` 合并发布

### 5.5.3 `rfid_status_2` 扩展语义（RM2026 V1.3.0，0x0209 offset 4）

| bit | ROS 字段 | 含义 |
|---|---|---|
| `0` | `enemy_tunnel_roadland_down` | 对方隧道靠近对方公路一侧下方 |
| `1` | `enemy_tunnel_roadland_mid` | 对方隧道靠近对方公路一侧中间 |
| `2` | `enemy_tunnel_roadland_up` | 对方隧道靠近对方公路一侧上方 |
| `3` | `enemy_tunnel_highland_low` | 对方隧道靠近对方梯形高地较低处 |
| `4` | `enemy_tunnel_highland_mid` | 对方隧道靠近对方梯形高地较中间 |
| `5` | `enemy_tunnel_highland_high` | 对方隧道靠近对方梯形高地较高处 |
| `6-7` | `rfid_status_2_reserved` | 保留位 |

---

## 5.6 `TypeID=5` - `PositionData`

结构：

```cpp
struct PositionType{
    uint8_t CarId;
    int16_t X;
    int16_t Y;
};

struct PositionData{
    PositionType Friend;
    PositionType Enemy;
    uint16_t BulletSpeed;
};
```

### 5.6.1 字段映射

| 串口字段 | 解析方式 | 发布 topic | ROS 字段 / 备注 |
|---|---|---|---|
| `Friend.CarId` | 直接读 | `/ly/position/data` | `friendcarid` |
| `Friend.X` | 直接读 | `/ly/position/data` | `friendx` |
| `Friend.Y` | 直接读 | `/ly/position/data` | `friendy` |
| `Enemy.CarId` | 直接读 | `/ly/position/data` | `enemycarid` |
| `Enemy.X` | 直接读 | `/ly/position/data` | `enemyx` |
| `Enemy.Y` | 直接读 | `/ly/position/data` | `enemyy` |
| `BulletSpeed` | `/100.0f` | `/ly/bullet/speed` | `data` |

### 5.6.2 特殊逻辑

如果：

```cpp
data.Friend.CarId == 7
```

则上位机还会额外发布：

- topic：`/ly/friend/uwb_pos`
- 类型：`gimbal_driver/msg/StampedUInt16MultiArray`
- 内容：`data=[Friend.X, Friend.Y]`，`header.stamp` 为 `gimbal_driver` 发布时间

注意当前代码行为：

1. `Friend.X/Y` 原始类型是 `int16_t`
2. 发布 `/ly/friend/uwb_pos` 时，代码做了 `static_cast<std::uint16_t>`
3. 所以这里是“按当前实现直接转换后发布”，不是重新定义过坐标系

---

## 5.7 `TypeID=6` - `ChassisData`

结构：

```cpp
struct ChassisData {
    std::uint16_t UWBAngleYaw;
    std::int16_t DamageDifference;
    std::uint32_t ChassisPacked1;
    std::uint32_t ChassisPacked2;
};
```

### 5.7.1 字段映射

| 串口字段 | 解析方式 | 发布 topic | 备注 |
|---|---|---|---|
| `UWBAngleYaw` | 直接读 `uint16` | `/ly/friend/uwb_yaw` | 自身朝向角 |
| `DamageDifference` | 直接读 `int16` | `/ly/game/damage_difference` | 裁判 `0x0003 game_robot_HP_t` offset 8，己方全队总伤害与对方全队总伤害之差 |
| `ChassisPacked1` low16（byte0~1） | 8位整数+8位小数（2位小数） | `/ly/gimbal/chassis` | `steer_angle` |
| `ChassisPacked1` high16（byte2~3） | 8位整数+8位小数（2位小数） | `/ly/gimbal/chassis` | `angular_velocity` |
| `ChassisPacked2` low16（byte0~1） | 8位整数+8位小数（2位小数） | `/ly/gimbal/chassis` | `velocity_x` |
| `ChassisPacked2` high16（byte2~3） | 8位整数+8位小数（2位小数） | `/ly/gimbal/chassis` | `velocity_y` |

### 5.7.2 姿态回读规则

TypeID 6 不再承载姿态兼容回读。姿态回读只从 TypeID 7 的 `SentryData.SentryInfo2`
拆出；`0x020D sentry_info_2 bit12-13` 为有效 `1/2/3` 时，`gimbal_driver`
发布 `/ly/gimbal/posture`。

下位机对接要求：

- TypeID 6 byte `2~3` 填 `int16_t DamageDifference`。
- 来源是 RM2026 通信协议 V2.0.0 `0x0003 game_robot_HP_t` byte offset `8`。
- 语义是 `己方全队总伤害 - 对方全队总伤害`，允许为负数。
- TypeID 6 不再用于姿态；姿态请透传 TypeID 7 的 `0x020D sentry_info_2.posture`。

---

## 5.8 `TypeID=7` - `SentryData`

结构：

```cpp
struct SentryData {
    uint32_t SentryInfo;
    uint16_t SentryInfo2;
    float BulletInitialSpeed;
    uint16_t Reserved;
};
```

来源：

| 串口字段 | 裁判系统字段 | 发布 topic / ROS 字段 |
|---|---|---|
| `SentryInfo` | `0x020D sentry_info` offset 0 | `/ly/game/sentry/info.sentry_info_raw`，并拆语义字段 |
| `SentryInfo2` | `0x020D sentry_info_2` offset 4 | `/ly/game/sentry/info.sentry_info_2_raw`，并拆语义字段 |
| `BulletInitialSpeed` | `0x0207 shoot_data.initial_speed` offset 3 | `/ly/game/bullet.initial_speed` |
| `Reserved` | 上下位机保留 | `/ly/game/sentry/info.reserved` |

`0x020D sentry_info_2 bit12-13` 会发布到 `SentryInfo.posture`。当该字段为有效
`1/2/3` 时，`gimbal_driver` 也会同步覆盖发布到 `/ly/gimbal/posture`，作为当前优先姿态回读来源。

`0x020D sentry_info_3` 因为 `TypeID=7` payload 已满，改由 `TypeID=10` 更新 shadow；
`/ly/game/sentry/info` 仍由 `TypeID=7` 的节奏发布，并在已收到 `TypeID=10` 后附带最新
`sentry_info_3_raw` 与拆出的剩余时间字段。这样不会用高频前哨血量包刷新
`sentry_info_2` 的新鲜度。

### 5.8.1 `SentryInfo` 拆字段

| 字段 | 位 | ROS 字段 |
|---|---|---|
| 除远程兑换外成功兑换的允许发弹量 | `sentry_info bit0-10` | `exchanged_projectile_allowance` |
| 成功远程兑换允许发弹量次数 | `bit11-14` | `remote_projectile_exchange_count` |
| 成功远程兑换血量次数 | `bit15-18` | `remote_hp_exchange_count` |
| 当前可确认免费复活 | `bit19` | `can_confirm_free_revive` |
| 当前可兑换立即复活 | `bit20` | `can_exchange_immediate_revive` |
| 立即复活所需金币 | `bit21-30` | `immediate_revive_cost` |
| 保留 | `bit31` | `sentry_info_reserved` |
| 当前是否脱战 | `sentry_info_2 bit0` | `out_of_combat` |
| 队伍 17mm 允许发弹量剩余可兑换数 | `bit1-11` | `remaining_exchangeable_17mm` |
| 当前姿态 | `bit12-13` | `posture` |
| 己方能量机关当前可进入正在激活状态 | `bit14` | `can_activate_energy_mechanism` |
| 当前姿态是否为强化姿态 | `bit15` | `enhanced_posture`，同时保留 `sentry_info_2_reserved` 兼容字段 |

### 5.8.2 `SentryInfo3` 拆字段

| 字段 | 位 | ROS 字段 |
|---|---|---|
| 是否已经收到 `sentry_info_3` | - | `has_sentry_info_3` |
| 哨兵进攻姿态弱化前剩余可持续时长，单位秒 | `sentry_info_3 bit0-7` | `attack_posture_remaining_s` |
| 哨兵防御姿态弱化前剩余可持续时长，单位秒 | `bit8-15` | `defense_posture_remaining_s` |
| 哨兵移动姿态弱化前剩余可持续时长，单位秒 | `bit16-23` | `move_posture_remaining_s` |
| 保留 | `bit24-31` | `sentry_info_3_reserved_low` |
| 哨兵强化进攻姿态剩余可持续时长，单位秒 | `bit32-39` | `enhanced_attack_posture_remaining_s` |
| 哨兵强化防御姿态剩余可持续时长，单位秒 | `bit40-47` | `enhanced_defense_posture_remaining_s` |
| 哨兵强化移动姿态剩余可持续时长，单位秒 | `bit48-55` | `enhanced_move_posture_remaining_s` |
| 保留 | `bit56-63` | `sentry_info_3_reserved_high` |

## 5.9 `TypeID=8` - `BulletDataAndRfid2`

结构：

```cpp
struct BulletDataAndRfid2 {
    uint8_t BulletType;
    uint8_t ShooterNumber;
    uint8_t LaunchingFrequency;
    uint16_t ProjectileAllowance17mm;
    uint16_t ProjectileAllowance42mm;
    uint16_t RemainingGoldCoin;
    uint16_t ProjectileAllowanceFortress;
    uint8_t RfidStatus2;
};
```

来源：

| 串口字段 | 裁判系统字段 | 发布 topic / ROS 字段 |
|---|---|---|
| `BulletType` | `0x0207 shoot_data.bullet_type` offset 0 | `/ly/game/bullet.bullet_type` |
| `ShooterNumber` | `0x0207 shoot_data.shooter_number` offset 1 | `/ly/game/bullet.shooter_number` |
| `LaunchingFrequency` | `0x0207 shoot_data.launching_frequency` offset 2 | `/ly/game/bullet.launching_frequency` |
| `ProjectileAllowance17mm` | `0x0208 projectile_allowance_17mm` offset 0 | `/ly/game/bullet.projectile_allowance_17mm` |
| `ProjectileAllowance42mm` | `0x0208 projectile_allowance_42mm` offset 2 | `/ly/game/bullet.projectile_allowance_42mm` |
| `RemainingGoldCoin` | `0x0208 remaining_gold_coin` offset 4 | `/ly/game/bullet.remaining_gold_coin` |
| `ProjectileAllowanceFortress` | `0x0208 projectile_allowance_fortress` offset 6 | `/ly/game/bullet.projectile_allowance_fortress_17mm` |
| `RfidStatus2` | `0x0209 rfid_status_2` offset 4 | `/ly/game/rfid.rfid_status_2_raw` |

注意：`0x0207` 被拆在 TypeID=7 和 TypeID=8 两帧中。下位机应在收到一次 `0x0207`
时同步更新内部 `shoot_data` shadow，再分别填入 TypeID=7/8。上位机以最近一次值合成
`/ly/game/bullet`，并通过 `has_initial_speed`、`has_shoot_data`、
`has_projectile_allowance` 标记当前消息中哪些部分已经收到。

`TypeID=8` 的 `rfid_status_2` 会合并到现有 `/ly/game/rfid`。`gimbal_driver` 对
TypeID=4 的低 32 位 `rfid_status` 和 TypeID=8 的 `rfid_status_2` 分别维护 shadow；
任一侧到达都会用“新的这一半 + 旧的另一半”发布完整 `/ly/game/rfid`。如果 TypeID=8
先于 TypeID=4 到达，低 32 位暂按默认 0 发布，并标记 `has_rfid_status_2=true`。

## 5.10 `TypeID=9` - `MapCommandData`

结构：

```cpp
struct MapCommandData {
    float TargetPositionX;
    float TargetPositionY;
    uint8_t CmdKeyboard;
    uint8_t TargetRobotId;
    uint16_t CmdSource;
};
```

来源：

| 串口字段 | 裁判系统字段 | 发布 topic / ROS 字段 |
|---|---|---|
| `TargetPositionX` | `0x0303 map_command_t.target_position_x` offset 0，单位 m | `/ly/game/map_command.target_position_x_m` |
| `TargetPositionY` | `0x0303 map_command_t.target_position_y` offset 4，单位 m | `/ly/game/map_command.target_position_y_m` |
| `CmdKeyboard` | `0x0303 map_command_t.cmd_keyboard` offset 8 | `/ly/game/map_command.cmd_keyboard` |
| `TargetRobotId` | `0x0303 map_command_t.target_robot_id` offset 9 | `/ly/game/map_command.target_robot_id` |
| `CmdSource` | `0x0303 map_command_t.cmd_source` offset 10 | `/ly/game/map_command.cmd_source` |

ROS 语义：

- `/ly/game/map_command.header.stamp` 是 `gimbal_driver` 收到 TypeID=9 后发布 ROS 消息的时间。
- `TargetRobotId == 0` 表示坐标模式，`MapCommand.has_target_position=true`。
- `TargetRobotId != 0` 表示目标机器人模式，`MapCommand.has_target_robot=true`；按裁判协议，此时 `TargetPositionX/Y` 应为 `0`。
- `CmdSource` 是信息来源 ID，ID 对应关系见通信协议附录。
- `0x0303` 触发发送后会以 `100ms` 间隔额外重发到共 5 包，并在下一次触发前以 `1Hz` 持续发送最近一次内容。任何会触发导航/行为的消费端必须自行去重。

注意：RM2026 通信协议 V1.3.0 的 `0x0303` 总表写数据段长度为 `15`，但详细 `map_command_t`
字段合计为 `12B`。本仓库 `TypeID=9` 使用详细结构的 `12B` payload；外层
`TypedMessage<sizeof(GimbalData)>` 总长度仍为 `15B`。

## 5.11 `TypeID=10` - `SentryInfo3AndOutpostHpData`

结构：

```cpp
struct SentryInfo3AndOutpostHpData {
    uint64_t SentryInfo3;
    uint16_t SelfOutpostHealth;
    uint16_t EnemyOutpostHealth;
};
```

### 5.11.1 字节布局

| byte offset | 字段 | 裁判系统字段 | 发布 topic / ROS 字段 |
|---|---|---|---|
| 0~7 | `SentryInfo3` | `0x020D sentry_info_t.sentry_info_3` offset 6 | 更新 `/ly/game/sentry/info` 的 `sentry_info_3_raw` shadow；实际发布仍由 TypeID=7 触发 |
| 8~9 | `SelfOutpostHealth` | `0x0003 game_robot_HP_t.ally_outpost_HP` offset 12 | `/ly/friend/op_hp.data` |
| 10~11 | `EnemyOutpostHealth` | `0x0003 game_robot_HP_t.enemy_outpost_HP` offset 16 | `/ly/enemy/op_hp.data` |

下位机要注意：这里按裁判 `0x0003` 官方顺序放前哨血量，**己方/ally 在前，敌方/enemy 在后**。
不要沿用旧 `GameCodeType` 的 `EnemyOutpostHealth` 在前的 bit-field 顺序。

### 5.11.2 前哨血量优先级

`/ly/friend/op_hp` 和 `/ly/enemy/op_hp` 的优先级是：

1. `TypeID=10` 的 `SelfOutpostHealth` / `EnemyOutpostHealth`，直接按 `uint16_t` 原始血量发布。
2. 如果 `TypeID=10` 从未收到，或最近一次 `TypeID=10` 超过 1500ms 未更新，则回退到
   `TypeID=1 GameCode` 的 6-bit 分度值 `* 25`。

`0` 是合法血量，表示前哨站已被摧毁；不能把 `0` 当成“没收到”。
所以下位机如果暂时拿不到裁判 `0x0003` 的前哨血量，不要用 `0` 或默认值继续发送
`TypeID=10`；应暂停发送 `TypeID=10`，让上位机在超过 1500ms 后自动回退到
`TypeID=1 GameCode * 25`。

---

## 6. 当前已对接数据汇总

## 6.1 上位机 -> 下位机已对接

| 数据项 | 串口字段 | 来源 topic |
|---|---|---|
| 云台目标角 yaw | `GimbalControlFrame.GimbalAngles.Yaw` | `/ly/control/angles` |
| 云台目标角 pitch | `GimbalControlFrame.GimbalAngles.Pitch` | `/ly/control/angles` |
| 底盘速度 x | `GimbalControlFrame.Velocity.X` | `/ly/control/vel` (`ControlVelocity`) |
| 底盘速度 y | `GimbalControlFrame.Velocity.Y` | `/ly/control/vel` (`ControlVelocity`) |
| 火控字段 | `GimbalControlFrame.FireCode` | `/ly/control/firecode` (`FireCode`) |
| 姿态命令 | `SentryCommandFrame.SentryCmd.Posture`（`0x01`） | `/ly/control/posture` |
| 哨兵裁判命令 | `SentryCommandFrame.SentryCmd`（`0x01`） | `/ly/control/sentry_cmd` (`SentryCmd`) |
| 裁判路径 | `MapPathFrame`（`0x02`） | `/ly/control/map_path` (`MapPath`) |
| 裁判自定义信息 | `CustomInfoFrame`（`0x03`） | `/ly/control/custom_info` (`CustomInfo`) |
| 哨兵自身坐标 x/y | `SentryCoordinateFrame.X_cm/Y_cm`（`0x04`） | `/ly/bt/sentry_position` (`PointStamped`) |

## 6.2 下位机 -> 上位机已对接

| TypeID | 数据项 | 串口字段 | 输出 topic |
|---|---|---|---|
| `0` | 云台角 | `GimbalData.GimbalAngles` | `/ly/gimbal/angles` |
| `0` | 底盘速度 | `GimbalData.Velocity` | `/ly/gimbal/vel` |
| `0` | 火控状态 | `GimbalData.FireCode` | `/ly/gimbal/firecode` (`FireCode`) |
| `0` | 电容值 | `GimbalData.CapV` | `/ly/gimbal/capV` |
| `1` | 比赛摘要 | `GameData` | `/ly/game/all` |
| `1` | 子弹余量 | `GameData.AmmoLeft` | `/ly/friend/ammo_left` |
| `1` | 比赛剩余时间 | `GameData.TimeLeft` | `/ly/game/time_left` |
| `1` | 敌方前哨站血量 fallback | `GameCode.EnemyOutpostHealth * 25` | `/ly/enemy/op_hp`，仅 TypeID=10 不新鲜时发布 |
| `1` | 我方前哨站血量 fallback | `GameCode.SelfOutpostHealth * 25` | `/ly/friend/op_hp`，仅 TypeID=10 不新鲜时发布 |
| `1` | 英雄预警 | `GameCode.HeroPrecaution` | `/ly/friend/is_precaution` |
| `1` | 比赛开始标志 | `GameCode.IsGameBegin` | `/ly/game/is_start` |
| `1` | 我方颜色 | `GameCode.IsMyTeamRed` | `/ly/friend/is_team_red` |
| `1` | 回家标志 | `GameCode.IsReturnedHome` | `/ly/friend/is_at_home` |
| `1` | 场地事件原始值/拆字段 | `GameData.ExtEventData` | `/ly/game/all`, `ly/gimbal/eventdata`, `/ly/game/event_data` |
| `2` | 我方各兵种血量 | `HealthMyselfData` | `/ly/friend/hp` |
| `2` | 我方基地血量 | `HealthMyselfData.BaseMyself` | `/ly/friend/base_hp` |
| `3` | 敌方各兵种血量 | `HealthEnemyData` | `/ly/enemy/hp` |
| `3` | 敌方基地血量 | `HealthEnemyData.BaseEnemy` | `/ly/enemy/base_hp` |
| `4` | RFID | `RFIDAndBuffData.RFIDStatus` | `/ly/game/rfid` |
| `4` | 增益状态 | `RFIDAndBuffData.BuffStatus.*` | `/ly/team/buff` |
| `5` | 位置数据 | `PositionData.Friend/Enemy` | `/ly/position/data` |
| `5` | 自身 UWB 坐标 | `PositionData.Friend.X/Y` | `/ly/friend/uwb_pos` |
| `5` | 弹速 | `PositionData.BulletSpeed` | `/ly/bullet/speed` |
| `6` | 自身朝向 | `ChassisData.UWBAngleYaw` | `/ly/friend/uwb_yaw` |
| `6` | 底盘回读（四元） | `ChassisPacked1/2`（8位整数+8位小数） | `/ly/gimbal/chassis` |
| `6` | 全队总伤害差 | `ChassisData.DamageDifference`（裁判 `0x0003` offset 8） | `/ly/game/damage_difference` |
| `7` | 哨兵自主决策状态 | `SentryData.SentryInfo/SentryInfo2` + 最新 TypeID=10 `SentryInfo3` shadow | `/ly/game/sentry/info`；有效 `posture` 同步覆盖 `/ly/gimbal/posture` |
| `7` | 发射初速度 | `SentryData.BulletInitialSpeed` | `/ly/game/bullet` |
| `8` | 发射事件字段 | `BulletDataAndRfid2.BulletType/ShooterNumber/LaunchingFrequency` | `/ly/game/bullet` |
| `8` | 允许发弹量/金币 | `BulletDataAndRfid2.ProjectileAllowance* / RemainingGoldCoin` | `/ly/game/bullet` |
| `8` | RFID 扩展字节 | `BulletDataAndRfid2.RfidStatus2` | `/ly/game/rfid` |
| `9` | 选手端小地图交互数据 | `MapCommandData` | `/ly/game/map_command` |
| `10` | 哨兵 `sentry_info_3` | `SentryInfo3AndOutpostHpData.SentryInfo3` | 更新 `/ly/game/sentry/info` 的 shadow，随 TypeID=7 发布 |
| `10` | 我方前哨站精确血量 | `SentryInfo3AndOutpostHpData.SelfOutpostHealth` | `/ly/friend/op_hp` |
| `10` | 敌方前哨站精确血量 | `SentryInfo3AndOutpostHpData.EnemyOutpostHealth` | `/ly/enemy/op_hp` |

---

## 7. 当前未真正拆出的字段

下面这些字段在结构定义里有，但在本仓库当前 `gimbal_driver` 中没有单独解析/发布：

| 结构 | 字段 | 当前状态 |
|---|---|---|
| `RFIDAndBuffData.BuffStatus` | `reserve` | 未发布 |

---

## 8. 裁判协议状态/命令与当前串口 `TypeID` 的关系

这里区分两层协议：

1. 裁判系统协议：例如 `0x0208`、`0x020D`、`0x0301/0x0120`。
2. 本仓库上下位机串口协议：例如上行 `TypeID=1 GameData`、`TypeID=4 RFIDAndBuffData`、下行 `DownlinkTypeID=0x00 GimbalControlFrame`。

当前上位机不是直接收发完整裁判协议幀，而是依赖下位机把裁判/底盘/云台状态整理成上面的 `TypeID` 或主控制幀字段。

| 裁判协议项 | 方向 | 当前本仓库状态 |
|---|---|---|
| `0x0207 shoot_data` | 裁判系统 -> 机器人状态 | 已通过 TypeID=7/8 进入 `/ly/game/bullet`；旧 `/ly/bullet/speed` 仍保留 TypeID=5 来源 |
| `0x0003 game_robot_HP` | 裁判系统 -> 全体机器人状态 | TypeID=6 的 `DamageDifference` 承载 offset 8 的 `int16_t damage_difference`，发布到 `/ly/game/damage_difference`；TypeID=10 承载 offset 12 `ally_outpost_HP` 和 offset 16 `enemy_outpost_HP`，优先发布到 `/ly/friend/op_hp`、`/ly/enemy/op_hp` |
| `0x0208 projectile_allowance` | 裁判系统 -> 机器人状态 | 已通过 TypeID=8 进入 `/ly/game/bullet`；旧 `/ly/friend/ammo_left` 仍保留 TypeID=1 来源 |
| `0x020D sentry_info/sentry_info_2/sentry_info_3` | 裁判系统 -> 哨兵状态 | `sentry_info/sentry_info_2` 通过 TypeID=7 进入 `/ly/game/sentry/info`；`sentry_info_3` 通过 TypeID=10 更新 shadow，随 TypeID=7 发布；有效 `posture` 会覆盖 `/ly/gimbal/posture` |
| `0x0301 + data_cmd_id=0x0120 sentry_cmd` | 机器人 -> 裁判系统命令 | 姿态/完整命令经 `/ly/control/posture`、`/ly/control/sentry_cmd` 进入独立 `DownlinkTypeID=0x01 SentryCommandFrame`；V2.0 姿态为 bit21-23，能量确认在 bit24；下位机负责封装裁判 `0x0301/0x0120` |
| `0x0307 map_data_t` | 机器人 -> 己方选手端路径显示 | `/ly/control/map_path` 进入 `DownlinkTypeID=0x02 MapPathFrame`；下位机负责封装裁判 `0x0307` |
| `0x0308 custom_info_t` | 机器人 -> 己方选手端自定义文字 | `/ly/control/custom_info` 进入 `DownlinkTypeID=0x03 CustomInfoFrame`；30B UTF-16 原始字节由上游提供，下位机负责封装裁判 `0x0308` |
| `0x0303 map_command_t` | 选手端 -> 机器人状态/指令输入 | 通过 TypeID=9 进入 `/ly/game/map_command`；BT 当前只缓存消息，不触发导航 |

### 8.1 当前看弹量和兑弹怎么走

当前“看弹量”的链路是：

```text
下位机 TypeID=1 GameData.AmmoLeft
  -> gimbal_driver
  -> /ly/friend/ammo_left
  -> behavior_tree ammoLeft
```

当前“兑弹”的下发接口已经接到 `/ly/control/sentry_cmd` 和独立 `SentryCommandFrame` (`DownlinkTypeID=0x01`)。BT 姿态切换主链路走 `/ly/control/posture`；能量机关确认会在打符链路满足到点、识别锁定、`can_activate_energy_mechanism=true` 后，通过 `/ly/control/sentry_cmd` 主动下发 `confirm_energy_activate` 脉冲。兑弹/远程回血/复活确认仍未由自动策略主动下发；BT 目前仍只会根据低弹量进入 Recovery/回补策略。

如果后续要实现自动兑弹，建议按两个方向补齐：

1. 状态回读：下位机把 `0x0208` 和 `0x020D` 拆成语义状态。当前已用 TypeID=7/8 上发 `0x0207`、`0x0208`、`0x0209 rfid_status_2` 和 `0x020D`；若后续还要更多裁判字段，应继续新增明确语义字段或新 TypeID。
2. 命令下发：当前已新增上位机 -> 下位机语义命令 `/ly/control/sentry_cmd`，不再挤进 `FireCode`。该命令最终由下位机写入裁判 `0x0301/0x0120 sentry_cmd`，包括兑换弹量、远程兑换请求次数、远程回血请求次数、确认复活、确认能量机关激活等字段。

注意：`0x020D` 是状态回读，不是上位机发出去的命令。真正的兑弹/激活确认命令在 `0x0301/0x0120 sentry_cmd`。

### 8.2 超级对抗赛哨兵兑弹规则和 2025/2026 协议对比

按本地规则文件 `RoboMaster 2026 机甲大师超级对抗赛比赛规则手册V1.4.2（20260430）.pdf`，哨兵初始允许发弹量为 300。局内额外弹量有三类来源：

| 类型 | 条件 | 规则侧语义 | 串口/裁判命令侧语义 |
|---|---|---|---|
| 非远程兑换 | 占领己方补给区、基地增益点、前哨站增益点 | 17mm 最小 10 发；规则表为 10 金币/10 发 | `0x0120 sentry_cmd bit2-12`，表示哨兵将要兑换的发弹量累计值，必须单调递增 |
| 远程兑换发弹量 | 哨兵处于脱战状态 | 17mm 最小 100 发；规则表为 150 金币/100 发；成功后 6 秒生效 | `0x0120 sentry_cmd bit13-16`，表示远程兑换发弹量请求次数，必须单调递增且每次只加 1 |
| 补给区获取 | 比赛开始后每隔 1 分钟，哨兵占领己方补给区增益点 | 每次可获取 100 发，未获取部分可累积 | 这是规则自动获取，不是 `0x0120` 兑换命令；上位机应从 `0x0208`/裁判状态观察弹量变化 |

状态回读对应 `0x020D sentry_info`：

| 字段 | 2026 V1.3.0 | 2025 V1.9.0 | 结论 |
|---|---|---|---|
| `bit0-10` | 除远程兑换外，哨兵成功兑换的允许发弹量 | 同左 | 两版都把非远程兑换成功量单独统计 |
| `bit11-14` | 哨兵成功远程兑换允许发弹量的次数 | 同左 | 两版都把远程兑换按成功次数统计 |
| `bit15-18` | 哨兵成功远程兑换血量的次数 | 同左 | 远程回血和远程兑弹分开 |
| `sentry_info_2 bit0` | 当前是否脱战 | 同左 | 远程兑换前应参考这个状态 |
| `sentry_info_2 bit1-11` | 队伍 17mm 允许发弹量剩余可兑换数 | 同左 | 可以用作还可兑换多少的状态回读 |
| `sentry_info_2 bit12-13` | 当前姿态 | 2025 保留 | 2026 新增姿态回读 |
| `sentry_info_2 bit14` | 能量机关当前可激活 | 2025 保留 | 2026 新增能量机关激活状态 |

命令下发对应 `0x0301 + data_cmd_id=0x0120 sentry_cmd`：

| 字段 | 2026 V1.3.0 | 2025 V1.9.0 | 结论 |
|---|---|---|---|
| `bit2-12` | 非远程兑换发弹量累计值，单调递增 | 同左 | 2025 已经区分非远程兑换 |
| `bit13-16` | 远程兑换发弹量请求次数，每次只加 1 | 同左 | 2025 已经区分远程兑换 |
| `bit17-20` | 远程兑换血量请求次数，每次只加 1 | 同左 | 远程回血独立 |
| `bit21-23` | 姿态切换命令，`1~6` | 2025 保留 | RM2026 V2.0 扩展为普通/强化姿态 |
| `bit24` | 确认能量机关进入正在激活状态 | 2025 保留 | RM2026 V2.0 定义 |

因此，2025 通信协议里哨兵已经分了远程和非远程兑弹；2026 主要是在同一个 `0x020D/0x0120` 框架上补了姿态和能量机关相关位。后续上位机如果要接管兑弹，不能只发一个 bool，应至少区分：

1. 非远程兑换累计目标值。
2. 远程兑换发弹量请求计数。
3. 远程兑换血量请求计数。
4. 当前命令 shadow，保证这些累计/计数字段单调递增，不覆盖姿态和能量机关位。

### 8.3 能回血、能兑弹、脱战怎么判断

规则来源：

- `RoboMaster 2026 机甲大师超级对抗赛比赛规则手册V1.4.2（20260430）.pdf`
  - PDF page 18：脱战定义
  - PDF page 75：补给区回血、远程兑换血量
  - PDF page 80：允许发弹量机制
  - PDF page 116：哨兵特殊机制
- `RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0（20260327）.pdf`
  - PDF page 23：`0x020D sentry_info_2`
  - PDF page 30：`0x0120 sentry_cmd`
  - PDF page 68、71-72：选手端/语义控制命令枚举

#### 8.3.1 能回血

规则上，除异常离线或被罚下外，地面机器人可以回血和复活。哨兵相关回血分两类：

| 类型 | 条件 | 效果 | 协议/状态建议 |
|---|---|---|---|
| 补给区回血 | 存活机器人占领己方补给区增益点 | 每秒恢复上限血量 `10%` | 用 RFID/位置/规则区判断是否在己方补给区；回血结果看当前血量变化 |
| 脱战补给区回血 | 比赛开始 4 分钟后，处于脱战状态且占领己方补给区增益点 | 每秒恢复上限血量 `25%`；一旦不脱战，25% 立即失效 | 脱战状态优先看 `0x020D sentry_info_2 bit0` |
| 远程兑换血量 | 英雄、步兵、哨兵处于脱战状态 | 确认后 6 秒，增加此时上限血量 `60%`，不超过上限 | `0x0120 bit17-20` 远程回血请求次数；成功次数看 `0x020D bit15-18` |

注意：

- 远程兑换血量确认后的 6 秒内，如果机器人战亡，则远程兑换血量无效，金币不返还。
- 当前本仓库没有完整 `0x020D`，所以上位机暂时不能可靠知道「官方判定的脱战」和远程回血成功次数。

#### 8.3.2 能兑弹

哨兵初始允许发弹量为 `300`。规则上，哨兵可以通过三种方式增加允许发弹量：

| 类型 | 条件 | 最小单位 | 生效方式 | 协议/状态建议 |
|---|---|---|---|---|
| 非远程兑换 | 占领己方补给区、基地增益点、前哨站增益点 | 17mm 为 `10` 发；42mm 为 `1` 发 | 兑换成功后增加允许发弹量 | `0x0120 bit2-12` 写累计兑换发弹量；成功量看 `0x020D bit0-10` |
| 远程兑换发弹量 | 机器人处于脱战状态 | 17mm 为 `100` 发；42mm 为 `10` 发 | 成功后 6 秒生效 | `0x0120 bit13-16` 写远程兑弹请求次数；成功次数看 `0x020D bit11-14` |
| 补给区获取 | 比赛开始后每隔 1 分钟，哨兵占领己方补给区增益点 | 每次 `100` 发 | 未获取部分可以累积 | 这是规则自动获取，不是 `0x0120` 兑换命令；弹量结果看 `0x0208` 或当前允许发弹量 |

协议侧还给了选手端/语义控制枚举：

| 命令 | 含义 |
|---|---|
| `CommonCommand cmd_type=1` | 兑换 17mm 发弹量，参数必须为 10 的倍数 |
| `CommonCommand cmd_type=2` | 兑换 42mm 发弹量 |
| `CommonCommand cmd_type=5` | 远程兑换允许发弹量 |
| `CommonCommand cmd_type=6` | 远程兑换血量 |
| `SentryCtrlCommand command_id=1` | 补血点补弹 |
| `SentryCtrlCommand command_id=2` | 补给站实体补弹 |
| `SentryCtrlCommand command_id=3` | 远程补弹 |
| `SentryCtrlCommand command_id=4` | 远程回血 |

本仓库后续如果做上位机自动兑弹，建议以 `0x0120 sentry_cmd` 为下位机最终执行目标，同时在 ROS 侧保留上述语义区分，不要压成一个 `bool exchange_ammo`。

#### 8.3.3 脱战怎么算

规则定义：

```text
机器人在存活状态下连续 6 秒未发射弹丸且未被扣血，即为脱战。
比赛开始时机器人默认为脱战状态。
```

因此官方判定逻辑可以理解为：

```text
alive = current_hp > 0 且裁判系统主控在线
last_combat_time = max(last_projectile_fired_time, last_hp_decrease_time)
out_of_combat = alive && (now - last_combat_time >= 6s)
```

实现优先级建议：

1. 最可靠：下位机解析裁判 `0x020D sentry_info_2 bit0`，上发给上位机。该 bit 为 1 时，哨兵当前处于脱战状态。
2. 退化估算：如果暂时没有 `0x020D`，上位机可用自身血量下降时间和发弹时间估算：
   - `last_hp_decrease_time`：来自 `/ly/game/all.selfhealth` 或 `/ly/friend/hp.sentry` 的下降沿。
   - `last_projectile_fired_time`：优先来自裁判发射事件/允许发弹量减少；没有时只能用上位机 firecode 翻转作为近似。
   - 估算值只能作为策略保护，不能作为和裁判完全一致的判定。

当前代码现状：

- 已有 `/ly/friend/ammo_left`，可看简化弹量。
- 已有 `/ly/friend/hp`、`/ly/game/all.selfhealth`，可看血量变化。
- 没有完整 `0x020D sentry_info_2 bit0`，所以没有官方脱战状态 topic。
- 没有完整 `0x0120 sentry_cmd` 下行，所以还不能由上位机真正下发自动兑弹/远程回血命令。

---

## 9. 代码定位

- 串口结构：`src/gimbal_driver/module/BasicTypes.hpp`
- 下发订阅入口：`src/gimbal_driver/main.cpp` 的 `GenSubs()`
- 上行解析分发入口：`src/gimbal_driver/main.cpp` 的 `LoopRead()`
- `TypeID=0`：`PubGimbalData()`
- `TypeID=1`：`PubGameData()`
- `TypeID=2`：`PubHealthMyselfData()`
- `TypeID=3`：`PubHealthEnemyData()`
- `TypeID=4`：`PubRFIDAndBuffData()`
- `TypeID=5`：`PubPositionData()`
- `TypeID=6`：`PubChassisData()`
- `TypeID=7`：`PubSentryData()`
- `TypeID=8`：`PubBulletDataAndRfid2()`
- `TypeID=9`：`PubMapCommandData()`
- `TypeID=10`：`PubSentryInfo3AndOutpostHpData()`
