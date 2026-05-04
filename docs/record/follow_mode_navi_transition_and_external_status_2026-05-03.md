# FollowMode / 区域过渡 / 导航状态对接记录

日期：2026-05-03

## 背景

原先代码里的 `HoleMode` 语义已经不再表示“钻洞模式”，现改为 `FollowMode`。  
这个位当前用于区域过渡时让下位机/云台进入稳定跟随状态：停小陀螺、停巡逻扫描、停止新的开火翻转。

同时，区域过渡到达判断接入导航侧外部状态，避免只靠上位机自身坐标距离判断。

## FireCode 语义

`gimbal_driver/msg/FireCode.msg` 当前字段：

- `field_mask`
- `fire_status`
- `cap_state`
- `follow_mode`
- `aim_mode`
- `rotate`
- `raw`

位定义：

| bit | 名称 | 含义 |
|---|---|---|
| 0~1 | `FireStatus` | 开火翻转位 |
| 2~3 | `CapState` | 电容状态 |
| 4 | `FollowMode` | 跟随模式 |
| 5 | `AimMode` | 辅瞄模式 |
| 6~7 | `Rotate` | 小陀螺档位 |

`FollowMode=1` 时，`behavior_tree` 当前约定：

- `Rotate=0`
- `AimMode=0`
- 不再翻转新的 `FireStatus`
- 停止巡逻扫描
- 云台角保持当前回读角

## 区域过渡状态机

代码中将原 Highland 兼容逻辑封装为 `NaviAreaTransitionRuntime`，类型包括：

- `EnterMyHighland`
- `ViaHighland`
- `LeaveMyHighland`
- `LeaveMyHighlandViaCastleLeft`

当前行为：

1. 进入我方高地时，先开 `FollowMode`，目标为我方 `Highland`。
2. 需要经高地兼容点时，先走 `Highland`，到达/超时/不可达后再继续原目标。
3. 从我方高地离开时，开 `FollowMode`。
4. 从我方高地去我方基地侧目标时，优先经 `CastleLeft1`，到达后关闭 `FollowMode` 再继续原目标。

配置位置：

- `src/behavior_tree/Scripts/ConfigJson/regional_competition.json`
- `src/behavior_tree/Scripts/ConfigJson/regional_simple_competition.json`

当前区域过渡到达半径：

```json
"HighlandCompat": {
  "Enable": true,
  "DisableRotate": true,
  "ArriveDistanceCm": 20,
  "TimeoutSec": 6
}
```

## 导航外部状态

`behavior_tree` 当前订阅：

- `/ly/navi/reached` (`std_msgs/msg/Bool`)
  - `true`：当前导航目标已到达
  - `false`：仍在路上
- `/ly/navi/reachable` (`std_msgs/msg/Bool`)
  - `true`：当前目标有有效路径
  - `false`：当前目标不可达

注意：代码里的 topic 名是 `/ly/navi/reached`，不是 `/ly/navi/reach`。

外部状态只在满足以下条件时参与判断：

1. 上位机已经发布过当前 `naviCommandGoal`。
2. 外部状态是在当前目标发布之后收到的。
3. 状态对应的 goal id 和 goal position 与当前目标一致。
4. 状态在 2 秒内更新。

如果没收到第一帧、状态超时、状态属于旧目标，BT 会回退到上位机自身距离判断。

## 导航进度 watchdog

`NaviProgressWatchdog` 现在也会参考外部状态：

- `reached=true`：视为已到达，不触发 fallback。
- `reachable=false`：视为不可达，可触发 fallback。
- 没有可用外部状态时：继续使用上位机自身位置、到达半径和无移动超时判断。

默认参数：

```json
"NaviProgressWatchdog": {
  "Enable": true,
  "ArriveDistanceCm": 140,
  "MoveProgressCm": 60,
  "NoMoveTimeoutSec": 14,
  "FallbackHoldSec": 5,
  "FallbackCooldownSec": 12
}
```

## 对 `regional` / `regional_simple` 的影响

这套区域过渡和外部导航状态在 `regional_competition.json` 与 `regional_simple_competition.json` 都生效。

差异仍由各自策略 profile 决定；到达/不可达判断、`FollowMode` 区域过渡控制是一致的。

## 验证方法

观察订阅：

```bash
ros2 topic info /ly/navi/reached -v
ros2 topic info /ly/navi/reachable -v
```

模拟导航反馈：

```bash
ros2 topic pub /ly/navi/reachable std_msgs/msg/Bool "{data: true}" -1
ros2 topic pub /ly/navi/reached std_msgs/msg/Bool "{data: true}" -1
```

观察下发：

```bash
ros2 topic echo /ly/control/firecode gimbal_driver/msg/FireCode
```

预期区域过渡期间：

- `follow_mode: true`
- `rotate: 0`
- `aim_mode: false`
- `fire_status` 不再被 BT 主动翻转
