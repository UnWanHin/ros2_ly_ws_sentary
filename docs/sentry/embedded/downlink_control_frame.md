# 上位机下发协议总览（给下位机）

Updated: 2026-07-08

## 1. 目的与范围

本文档描述「上位机 -> 下位机」串口下发协议，供电控固件直接对接。
不包含完整上行回传细节（上行姿态回读可参考 `TypeID=6 ChassisData` 约定）。

当前下行是同一串口上的 **DownlinkTypeID 分型 17B frame**：

- `DownlinkTypeID=0x00`：`GimbalControlFrame`，角度、底盘速度、火控、哨兵裁判命令。
- `DownlinkTypeID=0x01`：`SentryCoordinateFrame`，BT 融合后的哨兵自身坐标。

注意：上行 `TypeID` 和下行 `DownlinkTypeID` 是两个独立编号空间。上行仍使用
`TypedMessage + TypeID=0..9`；下行不再把 byte1 叫 `TypeID`，避免读代码时和上行编号混淆。

## 2. 通道说明

- 物理链路：同一串口全双工
- 下发网关节点：`gimbal_driver`
- 下行 frame 定义：`src/gimbal_driver/module/BasicTypes.hpp`
- 下行写入：`src/gimbal_driver/main.cpp`

输入 ROS Topic：

| Topic | 消息 | 下行用途 |
|---|---|---|
| `/ly/control/angles` | `gimbal_driver/msg/GimbalAngles` | 写 `GimbalControlFrame.GimbalAngles` |
| `/ly/control/vel` | `gimbal_driver/msg/ControlVelocity` | 写 `GimbalControlFrame.Velocity` |
| `/ly/control/firecode` | `gimbal_driver/msg/FireCode` | 写 `GimbalControlFrame.FireCode` |
| `/ly/control/posture` | `gimbal_driver/msg/SentryCmd` | 姿态切换主入口，只写 `SentryCmd.Posture` |
| `/ly/control/sentry_cmd` | `gimbal_driver/msg/SentryCmd` | 完整哨兵裁判命令入口 |
| `/ly/navi/vel` | `gimbal_driver/msg/Vel` | 仅 `io_config/navigation_test=true` 时直通速度 |
| `/ly/bt/sentry_position` | `geometry_msgs/msg/PointStamped` | 写 `SentryCoordinateFrame.X_cm/Y_cm` |

## 3. `GimbalControlFrame`（DownlinkTypeID=0x00）

对应结构体：`GimbalControlFrame`

按 `#pragma pack(1)` 编排，长度 **17 bytes**：

| byte offset | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `'!'` / `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定 `0x00`，表示主控制 frame |
| 2 | `Velocity.X` | `int8` | 底盘 x 速度原始值 |
| 3 | `Velocity.Y` | `int8` | 底盘 y 速度原始值 |
| 4-7 | `GimbalAngles.Yaw` | `float32` | 云台 yaw |
| 8-11 | `GimbalAngles.Pitch` | `float32` | 云台 pitch |
| 12 | `FireCode` | `uint8` | 开火、电容、FollowMode、AimMode、小陀螺 |
| 13-16 | `SentryCmd` | `uint32` | 裁判 `0x0120 sentry_cmd`，little-endian |

旧 17B `GimbalControlData` 的 byte16 是 `Tail=0`。新结构移除 `Tail`，byte1 改为
`DownlinkTypeID`，后续字段整体后移 1B。当前 `SentryCmd.Reserved` 默认清零，所以正常情况下
byte16 仍为 `0x00`，但固件不应再把它当 Tail 判断 frame 类型。

### 3.1 ROS -> 字段映射

| ROS topic | ROS 消息 | 串口字段 | 说明 |
|---|---|---|---|
| `/ly/control/angles` | `GimbalAngles` | `GimbalAngles.Yaw/Pitch` | 直接写 float |
| `/ly/control/vel` | `ControlVelocity` | `Velocity.X/Y` | `use_raw=true` 时直接写 `raw_x/raw_y` |
| `/ly/navi/vel` | `Vel` | `Velocity.X/Y` | 仅 `navigation_test=true` 单测直通；`x/y` round+clamp 到 `int8` |
| `/ly/control/firecode` | `FireCode` | `FireCode` | 支持 `field_mask` 局部更新 |
| `/ly/control/posture` | `SentryCmd` | `SentryCmd.Posture` | 只使用 `FIELD_POSTURE`，只改 `bit21-22` |
| `/ly/control/sentry_cmd` | `SentryCmd` | `SentryCmd` | 支持 `field_mask` 局部更新，给非姿态裁判命令使用 |

## 4. `SentryCoordinateFrame`（DownlinkTypeID=0x01）

对应结构体：`SentryCoordinateFrame`

按 `#pragma pack(1)` 编排，长度 **17 bytes**：

| byte offset | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `'!'` / `0x21` |
| 1 | `DownlinkTypeID` | `uint8` | 固定 `0x01`，表示哨兵自身坐标 frame |
| 2-3 | `X_cm` | `int16` | official-map x，单位 cm，little-endian |
| 4-5 | `Y_cm` | `int16` | official-map y，单位 cm，little-endian |
| 6-15 | `Reserved` | `uint8[10]` | 保留，当前填 0 |
| 16 | `CRC8` | `uint8` | 对 byte0-15 计算 CRC8 |

CRC8 参数：

- poly：`0x31`
- init：`0xFF`
- 非反射
- 校验范围：frame byte0 到 byte15

坐标来源：

```text
AreaManager.SentryPositionFusion
  -> behavior_tree publishes /ly/bt/sentry_position (PointStamped, frame_id=map, unit=m)
  -> gimbal_driver converts x/y from m to cm
  -> SentryCoordinateFrame.X_cm/Y_cm
```

`gimbal_driver` 会 clamp 到场地范围，默认 `x=[0,2800]cm`、`y=[0,1500]cm`。

相关参数（`config/base_config.yaml` 的 `io_config`）：

| 参数 | 默认 | 说明 |
|---|---:|---|
| `sentry_coord_send_interval_ms` | `100` | 坐标 frame 最小发送间隔 |
| `sentry_coord_field_width_x` | `2800` | x clamp 上限，cm |
| `sentry_coord_field_width_y` | `1500` | y clamp 上限，cm |
| `sentry_coord_fresh_timeout_ms` | `2000` | 坐标输入超过该时间未刷新则停止发送 |

## 5. `FireCode` 位语义（1B）

| bit | 字段 | 含义 |
|---|---|---|
| 0-1 | `FireStatus` | 开火位，翻转触发，`0b00 <-> 0b11` |
| 2-3 | `CapState` | 电容状态 |
| 4 | `FollowMode` | 跟随模式 |
| 5 | `AimMode` | 辅瞄/巡逻模式 |
| 6-7 | `Rotate` | 小陀螺档位 |

上位机 `behavior_tree` 发布 `FollowMode=1` 时，只改 bit4 本身；不会因为该 bit 自动强制
`Rotate=0`、`AimMode=0`、停止 `FireStatus` 翻转或停用云台巡逻扫描。`/ly/navi/should_rotate=false`
仍可按配置额外把 `Rotate=0`，FaceMode/停火也由各自独立控制。

下位机建议：不要把 `FireStatus==1` 当作“持续开火”，按翻转沿触发。

## 6. `SentryCmd` 位语义（4B）

`SentryCmd` 对应裁判协议 `0x0301 + data_cmd_id=0x0120 sentry_cmd` 的 32-bit 命令字。
上位机当前直接把它放进 `GimbalControlFrame` byte `13-16`。

| bit | 字段 | ROS 字段 | 说明 |
|---|---|---|---|
| 0 | `ConfirmFreeRevive` | `confirm_free_revive` | 确认免费复活 |
| 1 | `ConfirmImmediateRevive` | `confirm_immediate_revive` | 确认兑换立即复活 |
| 2-12 | `ExchangeProjectileAllowance` | `exchange_projectile_allowance` | 非远程兑换允许发弹量累计值，必须单调递增 |
| 13-16 | `RemoteProjectileExchangeCount` | `remote_projectile_exchange_count` | 远程兑换发弹量请求次数，每次请求只加 1 |
| 17-20 | `RemoteHpExchangeCount` | `remote_hp_exchange_count` | 远程兑换血量请求次数，每次请求只加 1 |
| 21-22 | `Posture` | `posture` | `0` 保留，`1` 进攻，`2` 防御，`3` 移动 |
| 23 | `ConfirmEnergyActivate` | `confirm_energy_activate` | 确认己方能量机关进入正在激活状态 |
| 24-31 | `Reserved` | - | 保留，填 0 |

### 6.1 ROS `SentryCmd.msg`

`/ly/control/sentry_cmd` 使用 `gimbal_driver/msg/SentryCmd`：

- `field_mask=0` 或 `FIELD_ALL`：完整快照，所有字段都应用。
- `field_mask!=0`：只更新 mask 指定字段，未指定字段保留 `gimbal_driver` 当前 shadow。
- `raw` 只用于调试和记录；当前 `gimbal_driver` 按语义字段组包，不按 `raw` 反解。

当前 BT 姿态链路发布 `/ly/control/posture`，消息类型为 `gimbal_driver/msg/SentryCmd`。
`gimbal_driver` 只取其中 `FIELD_POSTURE/posture` 写入 `SentryCmd.Posture`。
`/ly/control/sentry_cmd` 保留为完整 `SentryCmd` 命令入口。

姿态链路应只发布：

```text
field_mask = FIELD_POSTURE
posture = 1/2/3
```

### 6.2 姿态重发策略

- 收到 `/ly/control/posture` 且 mask 包含 `FIELD_POSTURE` 时，转写到 `SentryCmd.Posture`。
- 收到 `/ly/control/sentry_cmd` 且 mask 包含 `FIELD_POSTURE` 时，也会写入 `SentryCmd.Posture`。
- 每次姿态切换按参数重发（默认 `3` 次，间隔 `20ms`）。
- 串口重连后会按当前姿态再次触发重发。

### 6.3 姿态回读语义

- `/ly/gimbal/posture` 表示**下位机/裁判姿态回读状态**。
- 当前来源有两条：`TypeID=6 ChassisData.Posture` 和 `TypeID=7 SentryData.SentryInfo2 posture`。
- `TypeID=7` 的有效 `posture=1/2/3` 会覆盖发布到 `/ly/gimbal/posture`；`0` 不主动清掉当前姿态。
- 不建议将 `/ly/control/sentry_cmd.posture` 或 `/ly/control/posture` 直接镜像回
  `/ly/gimbal/posture`，否则会掩盖“已下发但未执行”的链路问题。
- 若下位机暂未实现 TypeID 6 回读，但已经透传裁判 `0x020D`，`/ly/gimbal/posture`
  仍可由 `/ly/game/sentry/info.posture` 更新。

## 7. 下位机实现要求

1. 下行统一读取 17B frame，先检查 byte0 是否为 `0x21`。
2. 读取 byte1 为 `DownlinkTypeID`，再按 `0x00/0x01` 分支解析；不要和上行 `TypeID` 共用 enum。
3. `DownlinkTypeID=0x00` 时按 `GimbalControlFrame` 解析 byte2-16。
4. `DownlinkTypeID=0x01` 时按 `SentryCoordinateFrame` 解析，并校验 byte16 CRC8。
5. 对未知 `DownlinkTypeID` 丢弃该 frame，避免误把坐标 frame 当控制 frame。
6. 对 `GimbalControlFrame.SentryCmd` 使用 `sentry_cmd_shadow` 维护裁判 `0x0120` 命令字。
7. 将 `sentry_cmd_shadow` 封装到裁判系统串口 `0x0301 + data_cmd_id=0x0120`。
8. 上行把下位机实际姿态状态写入 `TypeID=6 ChassisData.Posture`，作为兼容回读。
9. 上行继续把裁判 `0x020D` 拆到 TypeID=7；其中 `sentry_info_2 bit12-13 posture` 会作为 `/ly/gimbal/posture` 的优先回读来源。
10. 上行把 `0x0208/0x0209` 拆到 TypeID=8。
11. 上行把 `0x0303 map_command_t` 拆到 TypeID=9。

## 8. 固件实现速查

### 8.1 C 结构体

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  head_flag;        // 0x21
    uint8_t  downlink_type_id; // 0x00
    int8_t   vel_x;
    int8_t   vel_y;
    float    yaw;
    float    pitch;
    uint8_t  fire_code;
    uint32_t sentry_cmd;       // little-endian, referee 0x0120 sentry_cmd
} gimbal_control_frame_t;      // sizeof == 17

typedef struct {
    uint8_t head_flag;         // 0x21
    uint8_t downlink_type_id;  // 0x01
    int16_t x_cm;
    int16_t y_cm;
    uint8_t reserved[10];
    uint8_t crc8;
} sentry_coordinate_frame_t;   // sizeof == 17
#pragma pack(pop)
```

### 8.2 `sentry_cmd` 更新建议

如果下位机决定只信任上位机主控制 frame 完整命令字，可以直接：

```c
sentry_cmd_shadow = frame.sentry_cmd;
```

如果下位机还有本地保护或人工命令，需要按 mask/策略局部合并，至少保证姿态位不覆盖其它命令位：

```c
static inline void set_bits_u32(uint32_t *value, uint32_t mask, uint32_t shifted_value)
{
    *value = (*value & ~mask) | (shifted_value & mask);
}

set_bits_u32(&sentry_cmd_shadow, 0x3u << 21, frame.sentry_cmd & (0x3u << 21));
```

确认类命令 `bit0`、`bit1`、`bit23` 需要和上位机约定保持时间和清零条件，避免一直重复确认。
累计/计数字段 `bit2-20` 必须单调递增，成功次数以裁判上行 `0x020D` 为准。

## 9. 下发前推荐参考的上行状态

| 下发动作 | 推荐参考状态 |
|---|---|
| 免费复活 | `/ly/game/sentry/info.can_confirm_free_revive` |
| 立即复活 | `/ly/game/sentry/info.can_exchange_immediate_revive` 和 `immediate_revive_cost` |
| 非远程兑弹 | `/ly/game/bullet.remaining_gold_coin`、`projectile_allowance_17mm`、`/ly/game/rfid` |
| 远程兑弹 | `/ly/game/sentry/info.out_of_combat`、`remaining_exchangeable_17mm`、`remaining_gold_coin` |
| 远程回血 | `/ly/game/sentry/info.out_of_combat`、`remote_hp_exchange_count`、`remaining_gold_coin` |
| 能量机关确认 | `/ly/game/sentry/info.can_activate_energy_mechanism` 和 `/ly/game/event_data` 能量机关状态 |

## 10. 版本切换建议

- 旧 13B 主幀：无姿态字段。
- 旧 14B 主幀：byte `12` 是单独 `Posture`。
- 旧 17B `GimbalControlData`：byte `1` 是 `Velocity.X`，byte `16` 是 `Tail=0`。
- 新 17B `GimbalControlFrame`：byte `1` 是 `DownlinkTypeID=0x00`，byte `13-16` 是完整 `SentryCmd`。
- 新 17B `SentryCoordinateFrame`：byte `1` 是 `DownlinkTypeID=0x01`，byte `2-5` 是 `X/Y cm`。

下位机联调时应明确当前解析版本，避免把新主控制 frame 的 `DownlinkTypeID` 当速度，或把坐标 frame 误解析成控制 frame。

## 11. 联调最小命令

发角度：

```bash
ros2 topic pub /ly/control/angles gimbal_driver/msg/GimbalAngles "{yaw: 10.0, pitch: 2.0}" -1
```

发火控：

```bash
ros2 topic pub /ly/control/firecode gimbal_driver/msg/FireCode "{field_mask: 1, fire_status: 3}" -1
```

发姿态（BT 主链路，防御）：

```bash
ros2 topic pub /ly/control/posture gimbal_driver/msg/SentryCmd "{field_mask: 32, posture: 2}" -1
```

直接测试完整 `SentryCmd` 入口：

```bash
ros2 topic pub /ly/control/sentry_cmd gimbal_driver/msg/SentryCmd "{field_mask: 32, posture: 2}" -1
```

直接测试坐标下发：

```bash
ros2 topic pub /ly/bt/sentry_position geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: 12.3, y: 4.5, z: 0.0}}" -1
```
