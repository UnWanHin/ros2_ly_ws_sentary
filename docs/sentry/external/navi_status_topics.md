# Navi External Status Topics

Updated: 2026-05-27

`behavior_tree` 只订阅外部导航状态，不发布这些状态 topic。

| Topic | Type | Semantics |
|---|---|---|
| `/ly/navi/reached` | `std_msgs/msg/Bool` | 外部导航源：`true` 表示导航端确认当前目标已到达，`false` 表示导航端尚未确认到达。它不是 BT 内部最终 reached。 |
| `/ly/navi/reachable` | `std_msgs/msg/Bool` | `true` 表示当前导航目标有有效路径，`false` 表示当前目标不可达。 |
| `/ly/navi/should_rotate` | `std_msgs/msg/Bool` | `true` 表示恢复 BT 正常小陀螺/巡逻；`false` 表示导航请求 BT 关闭小陀螺并打开 `FollowMode`。 |

注意 topic 名是 `/ly/navi/reached`，不是 `/ly/navi/reach`。

当前 BT 判断到达时会先校验外部状态是否属于当前 goal：

1. 外部状态必须在当前导航目标发布之后收到。
2. 状态必须匹配当前 `naviCommandGoal` 和目标坐标。
3. 状态必须在 2 秒内更新。
4. `/ly/navi/reachable=false` 时认为当前目标不可达。
5. `/ly/navi/reached=true` 新鲜有效时作为高优先级到达来源。

`/ly/navi/reached=false` 不能永久否决 BT 的自身坐标距离判断。goal-start grace 之后，如果融合自身坐标已经进入到达半径，BT 可以判定内部 composite reached。后续应把这个结果封装成 `GoalReachState`，避免任务层直接消费 raw `/ly/navi/reached`。

`/ly/navi/should_rotate` 由 `src/behavior_tree/config/NaviRotateControl.yaml` 控制是否启用。默认配置中 500 ms 内没有新消息时按 `DefaultIsRotate=true` 处理；新鲜 `true` 会按 `ClearFollowModeWhenTrue=true` 释放 `FollowMode`，并按 `ClearRegionalFaceModeWhenTrue=true` 关闭 regional 区域兼容用的 FaceMode；新鲜 `false` 只在本轮输出强制 `FollowMode`，不会因为旧消息长时间卡住。
