# 上位机下发协议总览（给下位机）

Updated: 2026-05-11

## 1. 目的与范围

本文档描述「上位机 -> 下位机」串口下发协议，供电控固件直接对接。
不包含完整上行回传细节（上行姿态回读可参考 `TypeID=6 ChassisData` 约定）。

当前下发采用**单通道单主幀**：角度、底盘速度、火控、哨兵裁判命令全部并入
`GimbalControlData`。

当前实现中：

- 上行：`TypedMessage` + `TypeID=0..8`
- 下行：直接写 `GimbalControlData` 原始主幀，不再发送独立下发 `TypeID`

## 2. 通道说明

- 物理链路：同一串口全双工
- 下发网关节点：`gimbal_driver`
- 输入 ROS Topic：
  - `/ly/control/angles`
  - `/ly/control/vel`
  - `/ly/control/firecode`
  - `/ly/control/posture`，BT 姿态切换主入口，消息类型也是 `SentryCmd`，只使用 `FIELD_POSTURE`
  - `/ly/control/sentry_cmd`，完整哨兵裁判命令入口，供复活、兑弹、能量机关确认等字段使用

## 3. 主控制幀

### 3.1 幀结构

对应结构体：`GimbalControlData`
代码：`src/gimbal_driver/module/BasicTypes.hpp`

按 `#pragma pack(1)` 编排，长度 **17 bytes**：

| byte offset | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | `HeadFlag` | `uint8` | 固定 `'!'` / `0x21` |
| 1 | `Velocity.X` | `int8` | 底盘 x 速度原始值 |
| 2 | `Velocity.Y` | `int8` | 底盘 y 速度原始值 |
| 3-6 | `GimbalAngles.Yaw` | `float32` | 云台 yaw |
| 7-10 | `GimbalAngles.Pitch` | `float32` | 云台 pitch |
| 11 | `FireCode` | `uint8` | 开火、电容、FollowMode、AimMode、小陀螺 |
| 12-15 | `SentryCmd` | `uint32` | 裁判 `0x0120 sentry_cmd` 低 24 bit，little-endian |
| 16 | `Tail` | `uint8` | 固定 `0x00` |

与上一版 14B 主幀相比：移除单独 `Posture` byte，新增 4B `SentryCmd`，`Tail` 从 byte `13`
后移到 byte `16`。姿态现在在 `SentryCmd bit21-22`。

### 3.2 ROS -> 字段映射

| ROS topic | ROS 消息 | 串口字段 | 说明 |
|---|---|---|---|
| `/ly/control/angles` | `GimbalAngles` | `GimbalAngles.Yaw/Pitch` | 直接写 float |
| `/ly/control/vel` | `ControlVelocity` | `Velocity.X/Y` | `use_raw=true` 时直接写 `raw_x/raw_y` |
| `/ly/control/firecode` | `FireCode` | `FireCode` | 支持 `field_mask` 局部更新 |
| `/ly/control/posture` | `SentryCmd` | `SentryCmd.Posture` | 姿态切换主入口，只使用 `FIELD_POSTURE`，只改 `bit21-22` |
| `/ly/control/sentry_cmd` | `SentryCmd` | `SentryCmd` | 支持 `field_mask` 局部更新，给非姿态裁判命令使用 |

## 4. `FireCode` 位语义（1B）

| bit | 字段 | 含义 |
|---|---|---|
| 0-1 | `FireStatus` | 开火位，翻转触发，`0b00 <-> 0b11` |
| 2-3 | `CapState` | 电容状态 |
| 4 | `FollowMode` | 跟随模式 |
| 5 | `AimMode` | 辅瞄/巡逻模式 |
| 6-7 | `Rotate` | 小陀螺档位 |

上位机 `behavior_tree` 发布 `FollowMode=1` 时，只改 bit4 本身；不会因为该 bit 自动强制 `Rotate=0`、`AimMode=0`、停止 `FireStatus` 翻转或停用云台巡逻扫描。`/ly/navi/is_rotate=false` 仍可按配置额外把 `Rotate=0`，FaceMode/停火也由各自独立控制。

下位机建议：不要把 `FireStatus==1` 当作“持续开火”，按翻转沿触发。

## 5. `SentryCmd` 位语义（4B）

`SentryCmd` 对应裁判协议 `0x0301 + data_cmd_id=0x0120 sentry_cmd` 的 32-bit 命令字。
上位机当前直接把它放进主控制幀 byte `12-15`。

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

### 5.1 ROS `SentryCmd.msg`

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

### 5.2 姿态重发策略

- 收到 `/ly/control/posture` 且 mask 包含 `FIELD_POSTURE` 时，转写到 `SentryCmd.Posture`。
- 收到 `/ly/control/sentry_cmd` 且 mask 包含 `FIELD_POSTURE` 时，也会写入 `SentryCmd.Posture`。
- 每次姿态切换按参数重发（默认 `3` 次，间隔 `20ms`）。
- 串口重连后会按当前姿态再次触发重发。

### 5.3 姿态回读语义

- `/ly/gimbal/posture` 表示**下位机/裁判姿态回读状态**。
- 当前来源有两条：`TypeID=6 ChassisData.Posture` 和 `TypeID=7 SentryData.SentryInfo2 posture`。
- `TypeID=7` 的有效 `posture=1/2/3` 会覆盖发布到 `/ly/gimbal/posture`；`0` 不主动清掉当前姿态。
- 不建议将 `/ly/control/sentry_cmd.posture` 或 `/ly/control/posture` 直接镜像回
  `/ly/gimbal/posture`，否则会掩盖“已下发但未执行”的链路问题。
- 若下位机暂未实现 TypeID 6 回读，但已经透传裁判 `0x020D`，`/ly/gimbal/posture` 仍可由 `/ly/game/sentry/info.posture` 更新。

## 6. 下位机实现要求

1. 主控制幀解析长度改为 **17B**，并更新 `Tail` 校验偏移到 byte `16`。
2. 按 little-endian 读取 byte `12-15` 为 `uint32_t sentry_cmd`.
3. 用 `sentry_cmd_shadow` 维护裁判 `0x0120` 命令字，不能用姿态或其它命令重置整字。
4. 将 `sentry_cmd_shadow` 封装到裁判系统串口 `0x0301 + data_cmd_id=0x0120`。
5. 上行把下位机实际姿态状态写入 `TypeID=6 ChassisData.Posture`，作为兼容回读。
6. 上行继续把裁判 `0x020D` 拆到 TypeID=7；其中 `sentry_info_2 bit12-13 posture` 会作为 `/ly/gimbal/posture` 的优先回读来源。
7. 上行把 `0x0208/0x0209` 拆到 TypeID=8。

## 7. 固件实现速查

### 7.1 C 结构体

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  head_flag;  // 0x21
    int8_t   vel_x;
    int8_t   vel_y;
    float    yaw;
    float    pitch;
    uint8_t  fire_code;
    uint32_t sentry_cmd; // little-endian, referee 0x0120 sentry_cmd
    uint8_t  tail;       // 0x00
} gimbal_control_frame_t; // sizeof == 17
#pragma pack(pop)
```

### 7.2 `sentry_cmd` 更新建议

如果下位机决定只信任上位机主幀完整命令字，可以直接：

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

## 8. 下发前推荐参考的上行状态

| 下发动作 | 推荐参考状态 |
|---|---|
| 免费复活 | `/ly/game/sentry/info.can_confirm_free_revive` |
| 立即复活 | `/ly/game/sentry/info.can_exchange_immediate_revive` 和 `immediate_revive_cost` |
| 非远程兑弹 | `/ly/game/bullet.remaining_gold_coin`、`projectile_allowance_17mm`、`/ly/game/rfid` |
| 远程兑弹 | `/ly/game/sentry/info.out_of_combat`、`remaining_exchangeable_17mm`、`remaining_gold_coin` |
| 远程回血 | `/ly/game/sentry/info.out_of_combat`、`remote_hp_exchange_count`、`remaining_gold_coin` |
| 能量机关确认 | `/ly/game/sentry/info.can_activate_energy_mechanism` 和 `/ly/game/event_data` 能量机关状态 |

## 9. 版本切换建议

- 旧 13B 主幀：无姿态字段。
- 旧 14B 主幀：byte `12` 是单独 `Posture`。
- 新 17B 主幀：byte `12-15` 是完整 `SentryCmd`，`Tail` 在 byte `16`。

下位机联调时应明确当前解析版本，避免把新主幀的 `sentry_cmd` 低字节误当旧 `posture`。

## 10. 联调最小命令

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
