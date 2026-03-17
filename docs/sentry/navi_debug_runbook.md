# 导航调试 Runbook

## 1. 目标

用于在不改老比赛点位定义的前提下，单独调试 `/ly/navi/goal` 发点链路。

这套链路分成两种入口：

1. `behavior_tree` 内的 `NaviDebug` 模式
2. BT 外的 standalone 导航巡逻发点脚本

两边共用同一份临时点位计划文件：

- `src/behavior_tree/Scripts/ConfigJson/navi_debug_points.json`

---

## 2. 改哪里

主配置：

- `src/behavior_tree/Scripts/ConfigJson/navi_debug_competition.json`

临时点位计划：

- `src/behavior_tree/Scripts/ConfigJson/navi_debug_points.json`

点位计划文件格式：

```json
{
  "ActivePlan": "test_site_random",
  "Plans": {
    "test_site_random": {
      "Mode": "random",
      "GoalHoldSec": 5,
      "SpeedLevel": 1,
      "DisableTeamOffset": true,
      "IgnoreRecovery": true,
      "Goals": [6, 11, 14, 15]
    }
  }
}
```

字段说明：

- `ActivePlan`：当前默认计划名
- `Plans.<name>.Mode`：`random` 或 `sequence`
- `Plans.<name>.GoalHoldSec`：每个点停留秒数
- `Plans.<name>.SpeedLevel`：同步发布到 `/ly/navi/speed_level`
- `Plans.<name>.DisableTeamOffset`：`true` 时直接发基础 ID `0..18`
- `Plans.<name>.IgnoreRecovery`：`true` 时忽略 BT 的回血/补弹回补逻辑
- `Plans.<name>.Goals`：基础点位 ID 列表

---

## 3. 单独测试

### 3.1 最轻量：不走 BT，直接自动发点

```bash
./scripts/debug.sh navi-patrol
```

指定计划名：

```bash
./scripts/debug.sh navi-patrol --plan test_site_sequence
```

蓝方映射：

```bash
./scripts/debug.sh navi-patrol --team blue
```

### 3.2 走 BT 的导航调试模式

默认只拉 `behavior_tree`：

```bash
./scripts/start_sentry_navi_debug.sh
```

如果现场接了裁判系统，想保留开赛门控：

```bash
./scripts/start_sentry_navi_debug.sh --with-gate
```

如果还想顺手带上 `gimbal_driver`：

```bash
./scripts/start_sentry_navi_debug.sh -- use_gimbal:=true
```

---

## 4. 现在的行为

- `NaviDebug` 启用后，`behavior_tree` 会固定保持 `NaviTest`
- 发点顺序由 `navi_debug_points.json` 决定
- `Mode=random` 时随机切点
- `Mode=sequence` 时按数组顺序轮巡
- 默认推荐 `UseXY=false`，只发 `/ly/navi/goal`
- 这条调试模式默认没有普通辅瞄
- 这条调试模式默认没有独立小陀螺测试

---

## 5. 注意

- 这套方案只改“发哪些点位 ID”，不改导航侧坐标定义。
- 如果你现场要的是全新 XY 临时坐标，那还得让导航侧支持新的坐标表，或改回 `/ly/navi/goal_pos`。
- `./scripts/debug.sh navi-patrol` 直接读取 `src/behavior_tree/Scripts/ConfigJson/navi_debug_points.json`。
- `./scripts/start_sentry_navi_debug.sh` 里的 `behavior_tree` 运行时读取的是 `install/behavior_tree/share/behavior_tree/Scripts/ConfigJson/*.json`，所以改完源文件后要先 `colcon build --packages-select behavior_tree`。
