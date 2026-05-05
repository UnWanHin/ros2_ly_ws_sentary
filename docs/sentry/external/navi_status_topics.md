# Navi External Status Topics

Updated: 2026-05-06

`behavior_tree` 只订阅外部导航状态，不发布这两个状态 topic。

| Topic | Type | Semantics |
|---|---|---|
| `/ly/navi/reached` | `std_msgs/msg/Bool` | `true` 表示当前导航目标已到达，`false` 表示仍在路上。 |
| `/ly/navi/reachable` | `std_msgs/msg/Bool` | `true` 表示当前导航目标有有效路径，`false` 表示当前目标不可达。 |

注意 topic 名是 `/ly/navi/reached`，不是 `/ly/navi/reach`。

当前 BT 判断到达时会先看外部状态：

1. 外部状态必须在当前导航目标发布之后收到。
2. 状态必须匹配当前 `naviCommandGoal` 和目标坐标。
3. 状态必须在 2 秒内更新。
4. `/ly/navi/reachable=false` 时认为当前目标不可达。
5. `/ly/navi/reached` 新鲜有效时直接作为到达结果。

如果还没收到第一帧、状态超时、状态属于旧目标，BT 才回退到自身位置和目标点距离判断。
