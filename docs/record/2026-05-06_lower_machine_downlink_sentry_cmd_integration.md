# Lower Machine Downlink SentryCmd Integration

Updated: 2026-05-06

本文是给下位机对接「上位机下行」的变更记录。当前有效协议细节仍以
`docs/sentry/embedded/downlink_control_frame.md` 和
`docs/sentry/embedded/referee_serial_integration.md` 为准。

## 1. 这次下行改了什么

上位机下发主控制幀从旧的 **14B** 改成 **17B**。

旧版：

```text
HeadFlag + Velocity + GimbalAngles + FireCode + Posture + Tail
```

新版：

```text
HeadFlag + Velocity + GimbalAngles + FireCode + SentryCmd + Tail
```

核心变化：

- 移除单独 1B `Posture` 字段。
- 新增 4B `SentryCmd` 字段。
- 姿态从独立 byte 改到 `SentryCmd bit21-22`。
- `Tail` 偏移从 byte `13` 后移到 byte `16`。
- `SentryCmd` 直接对应裁判系统 `0x0301 + data_cmd_id=0x0120 sentry_cmd` 的 32-bit 命令字。

## 2. 下位机要解析的 17B 主控制幀

按 `#pragma pack(1)` 对齐：

| byte offset | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | `head_flag` | `uint8_t` | 固定 `0x21` |
| 1 | `vel_x` | `int8_t` | 底盘 x 速度原始值 |
| 2 | `vel_y` | `int8_t` | 底盘 y 速度原始值 |
| 3-6 | `yaw` | `float` | 云台 yaw |
| 7-10 | `pitch` | `float` | 云台 pitch |
| 11 | `fire_code` | `uint8_t` | 开火、电容、FollowMode、AimMode、小陀螺 |
| 12-15 | `sentry_cmd` | `uint32_t` | little-endian，裁判 `0x0120 sentry_cmd` |
| 16 | `tail` | `uint8_t` | 固定 `0x00` |

建议固件结构：

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  head_flag;
    int8_t   vel_x;
    int8_t   vel_y;
    float    yaw;
    float    pitch;
    uint8_t  fire_code;
    uint32_t sentry_cmd;
    uint8_t  tail;
} gimbal_control_frame_t; // sizeof == 17
#pragma pack(pop)
```

下位机不要再按旧 14B 协议从 byte `12` 读取单独 `posture`。

## 3. `FireCode` 仍然是车内控制

`fire_code` 还是 1B，不属于裁判 `0x0120`。

| bit | 字段 | 意义 |
|---|---|---|
| 0-1 | `FireStatus` | 开火翻转位 |
| 2-3 | `CapState` | 电容状态 |
| 4 | `FollowMode` | 跟随模式 |
| 5 | `AimMode` | 辅瞄/巡逻模式 |
| 6-7 | `Rotate` | 小陀螺档位 |

## 4. `SentryCmd` 对应裁判 `0x0120`

`sentry_cmd` 是 32-bit 命令字，下位机应封装进裁判系统
`0x0301 + data_cmd_id=0x0120`。

| bit | 字段 | 意义 |
|---|---|---|
| 0 | `confirm_free_revive` | 确认免费复活 |
| 1 | `confirm_immediate_revive` | 确认兑换立即复活 |
| 2-12 | `exchange_projectile_allowance` | 非远程兑换允许发弹量累计值 |
| 13-16 | `remote_projectile_exchange_count` | 远程兑换发弹量请求次数 |
| 17-20 | `remote_hp_exchange_count` | 远程兑换血量请求次数 |
| 21-22 | `posture` | `0` 保留，`1` 进攻，`2` 防御，`3` 移动 |
| 23 | `confirm_energy_activate` | 确认能量机关进入正在激活状态 |
| 24-31 | reserved | 保留，填 0 |

注意：

- `exchange_projectile_allowance` 是累计值，不是单次值。
- `remote_projectile_exchange_count` 和 `remote_hp_exchange_count` 是请求次数，每次请求只加 1。
- `bit0`、`bit1`、`bit23` 是确认类 bit，后续联调要约定保持几帧和何时清零。

## 5. 上位机 ROS 链路

当前 BT 主链路已经改成：

```text
behavior_tree
  -> /ly/referee/sentry_cmd (gimbal_driver/msg/SentryCmd)
  -> gimbal_driver
  -> GimbalControlData.SentryCmd
  -> 下位机
```

旧的 `/ly/control/posture` 仍保留为兼容入口。它不会再占用主幀 byte `12`，而是由
`gimbal_driver` 转写到 `SentryCmd.Posture`。

当前 BT 实际只主动下发姿态：

```text
field_mask = FIELD_POSTURE
posture = 1/2/3
```

复活、兑弹、远程回血、能量机关确认的接口已经在 `SentryCmd` 里预留，但 BT 自动策略还没有开始主动下发这些字段。

## 6. 下位机推荐处理方式

最小实现：

1. 串口接收主控制幀长度改为 `17`。
2. 校验 `head_flag == 0x21`，`tail == 0x00`。
3. 读取 byte `12-15` 为 `uint32_t upper_sentry_cmd_shadow`。
4. 将 `upper_sentry_cmd_shadow` 封装为裁判系统 `0x0301 / 0x0120 sentry_cmd`。
5. 继续通过 TypeID 6 回传实际姿态到上位机 `/ly/gimbal/posture`。

如果下位机没有本地人工裁判命令，最简单可以直接：

```c
referee_sentry_cmd_shadow = frame.sentry_cmd;
```

如果下位机还有本地保护或人工命令，需要自己合并 shadow，不能因为姿态更新清掉其它位。

## 7. 联调先测什么

第一阶段只测姿态：

1. 上位机发 `/ly/referee/sentry_cmd`，`posture=2`。
2. 下位机确认收到 17B 主幀。
3. 下位机确认 `sentry_cmd bit21-22 == 2`。
4. 下位机封装裁判 `0x0301 / 0x0120`，写入 `bit21-22`。
5. 下位机从实际状态回传 TypeID 6 `ChassisData.Posture`。
6. 上位机 `/ly/gimbal/posture` 能看到 `2`。

测试命令：

```bash
ros2 topic pub /ly/referee/sentry_cmd gimbal_driver/msg/SentryCmd "{field_mask: 32, posture: 2}" -1
```

旧兼容入口也可测：

```bash
ros2 topic pub /ly/control/posture std_msgs/msg/UInt8 "{data: 2}" -1
```

## 8. 对接文件

- `src/gimbal_driver/module/BasicTypes.hpp`
  - `SentryCmdType`
  - `GimbalControlData`
- `src/gimbal_driver/msg/SentryCmd.msg`
- `src/gimbal_driver/main.cpp`
  - `/ly/referee/sentry_cmd`
  - `/ly/control/posture` 兼容转写
- `docs/sentry/embedded/downlink_control_frame.md`
- `docs/sentry/embedded/referee_serial_integration.md`
