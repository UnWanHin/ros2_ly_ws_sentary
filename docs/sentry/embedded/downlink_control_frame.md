# 上位机下发协议总览（给下位机）

Updated: 2026-05-06

## 1. 目的与范围

本文档描述「上位机 -> 下位机」串口下发协议，供电控固件直接对接。  
不包含完整上行回传细节（上行姿态回读可参考 `TypeID=6 ChassisData` 约定）。

当前下发采用**单通道单主幀**：角度/底盘速度/火控/姿态全部并入 `GimbalControlData`。
当前实现中：
- 上行：`TypedMessage` + `TypeID=0..6`
- 下行：直接写 `GimbalControlData` 原始主幀，不再发送独立 `TypeID=7`

---

## 2. 通道说明

- 物理链路：同一串口全双工
- 下发网关节点：`gimbal_driver`
- 输入 ROS Topic：
  - `/ly/control/angles`
  - `/ly/control/vel`
  - `/ly/control/firecode`
  - `/ly/control/posture`

---

## 3. 主控制幀（含姿态）

### 3.1 幀结构

对应结构体：`GimbalControlData`  
代码：`src/gimbal_driver/module/BasicTypes.hpp`

按 `#pragma pack(1)` 编排，长度 **14 bytes**：

1. `HeadFlag`（1B）=`'!'` (`0x21`)
2. `Velocity.X`（1B, int8）
3. `Velocity.Y`（1B, int8）
4. `GimbalAngles.Yaw`（4B, float32）
5. `GimbalAngles.Pitch`（4B, float32）
6. `FireCode`（1B, bitfield）
7. `Posture`（1B, uint8）
8. `Tail`（1B）=`0x00`

> 与旧版相比：新增 `Posture` 字段，`Tail` 偏移后移 1 字节。

### 3.2 ROS -> 字段映射

- `/ly/control/angles` (`gimbal_driver/msg/GimbalAngles`)
  - `yaw` -> `GimbalAngles.Yaw`
  - `pitch` -> `GimbalAngles.Pitch`
- `/ly/control/vel` (`gimbal_driver/msg/ControlVelocity`)
  - `x_mps/y_mps` 按 `velocity_raw_to_mps` 编码到 `Velocity.X/Y`
  - `use_raw=true` 时 `raw_x/raw_y` 直接写入 `Velocity.X/Y`
- `/ly/control/firecode` (`gimbal_driver/msg/FireCode`)
  - `fire_status/cap_state/follow_mode/aim_mode/rotate` -> `FireCode` 各 bit
  - `field_mask` 非 0 时只更新指定字段；未更新字段 100ms 后退回 0
- `/ly/control/posture` (`std_msgs/msg/UInt8`)
  - `1` 进攻
  - `2` 防御
  - `3` 移动
  - `0` 保留（不请求姿态切换）

### 3.3 FireCode 位语义（1字节）

从低位到高位：
1. bit0-1: `FireStatus`（开火位，翻转触发，`0b00 <-> 0b11`）
2. bit2-3: `CapState`
3. bit4: `FollowMode`
4. bit5: `AimMode`
5. bit6-7: `Rotate`

上位机 `behavior_tree` 当前约定：`FollowMode=1` 时仍下发 bit4，同时强制 `Rotate=0`、`AimMode=0`，不再翻转 `FireStatus`，并保持当前云台角度以停用巡逻扫描。

下位机建议：
- 不要把 `FireStatus==1` 当作“持续开火”，按翻转沿触发。

### 3.4 姿态重发策略（上位机行为）

- 收到 `/ly/control/posture` 的 `1/2/3` 时，立即写入主幀 `Posture` 字段并下发
- 每次切换按参数重发（默认 `3` 次，间隔 `20ms`）
- 串口重连后会按当前姿态再次触发重发

### 3.5 姿态回读语义（联调约定）

- `/ly/gimbal/posture` 表示**下位机回读状态**，当前来源 `TypeID=6 ChassisData.Posture`（1/2/3）。
- 不建议将 `/ly/control/posture` 直接镜像回 `/ly/gimbal/posture`，否则会掩盖“已下发但未执行”的链路问题。
- 若下位机暂未实现回读，`/ly/gimbal/posture` 可保持未更新，上位机会按无回读路径处理（含重试/超时策略）。

---

## 4. 下位机实现要求

1. 主控制幀解析长度改为 **14B**，并更新 `Tail` 校验偏移。  
2. 解析 `Posture` 字段，仅接受 `1/2/3`，`0` 视为保留值。  
3. 将姿态映射到裁判链路 `0x0120 bit21-22`。  
4. 建议维护 `sentry_cmd_shadow`，只改 bit21-22，不重置其他控制位。  
5. 上行把姿态状态写入 `TypeID=6 ChassisData.Posture`。

---

## 4.1 固件实现速查

### C 结构体

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t head_flag;
    int8_t  vel_x;
    int8_t  vel_y;
    float   yaw;
    float   pitch;
    uint8_t fire_code;
    uint8_t posture;
    uint8_t tail;
} gimbal_control_frame_t; // sizeof == 14
#pragma pack(pop)
```

### 关键偏移

- `posture` 在 byte `12`
- `tail` 在 byte `13`

### bit21-22 更新

```c
uint32_t mask = (0x3u << 21);
sentry_cmd_shadow = (sentry_cmd_shadow & ~mask) | (((uint32_t)posture & 0x3u) << 21);
```

---

## 5. 后续裁判命令下发扩展

当前代码还没有完整 `0x0301 + data_cmd_id=0x0120 sentry_cmd` 下发。现在实际只通过主控制幀的
`Posture` 字段让下位机更新 `sentry_cmd bit21-22`。如果后续要让上位机接管复活、远程回血、
兑换发弹量或确认能量机关激活，建议新增一条明确的「裁判命令」下发接口，不要继续挤进 `FireCode`。

不建议复用现有字段：

- `FireCode` 1B 已经被开火、电容、FollowMode、AimMode、小陀螺占满。
- `Posture` 只表达姿态，不应混入复活、回血、兑弹或能量机关确认。
- TypeID 7/8 是下位机 -> 上位机上行状态，不是下发命令通道。

### 5.1 上位机需要表达的语义

建议 ROS 侧后续新增一个独立 topic，例如 `/ly/referee/sentry_cmd`，消息字段至少包含：

| 语义字段 | 裁判 `0x0120 sentry_cmd` 位 | 类型建议 | 说明 |
|---|---|---|---|
| `confirm_free_revive` | `bit0` | `bool` | 确认免费复活 |
| `confirm_immediate_revive` | `bit1` | `bool` | 确认兑换立即复活 |
| `exchange_projectile_allowance` | `bit2-12` | `uint16` | 非远程兑换允许发弹量累计值，必须单调递增 |
| `remote_projectile_exchange_count` | `bit13-16` | `uint8` | 远程兑换允许发弹量请求次数，每次请求只加 1 |
| `remote_hp_exchange_count` | `bit17-20` | `uint8` | 远程兑换血量请求次数，每次请求只加 1 |
| `posture` | `bit21-22` | `uint8` | 1 进攻，2 防御，3 移动；当前已由 `/ly/control/posture` 覆盖 |
| `confirm_energy_activate` | `bit23` | `bool` | 确认己方能量机关进入正在激活状态 |

### 5.2 下位机实现要求

下位机应维护一个 `sentry_cmd_shadow`，每次只更新被上位机明确请求的位，不要重置整个 `uint32_t`：

```c
static uint32_t sentry_cmd_shadow = 0;

static inline void set_bits_u32(uint32_t *value, uint32_t mask, uint32_t shifted_value)
{
    *value = (*value & ~mask) | (shifted_value & mask);
}
```

关键约束：

- `bit2-12` 是累计兑换量，不是单次兑换量。
- `bit13-16` 和 `bit17-20` 是请求次数，成功次数要看上行 `0x020D sentry_info`。
- `bit21-22` 姿态位不能被其它命令覆盖。
- `bit23` 能量机关确认应配合上行 `0x020D sentry_info_2 bit14` 的 `can_activate_energy_mechanism` 使用。
- `bit0`、`bit1`、`bit23` 属于确认类命令，后续实现时需要约定发送保持时间和清零条件，避免一直重复确认。

### 5.3 需要配合的上行状态

下发 `0x0120` 前，上位机应该优先参考这些上行状态：

| 下发动作 | 推荐参考状态 |
|---|---|
| 免费复活 | `/ly/referee/sentry_info.can_confirm_free_revive` |
| 立即复活 | `/ly/referee/sentry_info.can_exchange_immediate_revive` 和 `immediate_revive_cost` |
| 非远程兑弹 | `/ly/referee/bullet_info.remaining_gold_coin`、`projectile_allowance_17mm`、`/ly/me/rfid` |
| 远程兑弹 | `/ly/referee/sentry_info.out_of_combat`、`remaining_exchangeable_17mm`、`remaining_gold_coin` |
| 远程回血 | `/ly/referee/sentry_info.out_of_combat`、`remote_hp_exchange_count`、`remaining_gold_coin` |
| 能量机关确认 | `/ly/referee/sentry_info.can_activate_energy_mechanism` 和 `/ly/game/event_data` 能量机关状态 |

---

## 6. 版本切换建议

- 若下位机仍在旧版 13B 主幀解析逻辑，必须先升级解析器再联调。  
- 若需要灰度切换，可短期保留双解析分支（13B/14B），确认稳定后再移除旧分支。  

---

## 7. 联调最小命令

1. 发角度：
```bash
ros2 topic pub /ly/control/angles gimbal_driver/msg/GimbalAngles "{yaw: 10.0, pitch: 2.0}" -1
```

2. 发速度：
```bash
ros2 topic pub /ly/control/vel gimbal_driver/msg/ControlVelocity "{x_mps: 0.25, y_mps: -0.25}" -1
```

3. 发火控（示例 `fire_status=3`）：
```bash
ros2 topic pub /ly/control/firecode gimbal_driver/msg/FireCode "{field_mask: 1, fire_status: 3}" -1
```

4. 发姿态（防御）：
```bash
ros2 topic pub /ly/control/posture std_msgs/msg/UInt8 "{data: 2}" -1
```
