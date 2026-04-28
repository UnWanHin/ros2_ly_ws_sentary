# DamageOpenGate 通用配置记录

日期：2026-04-28

## 背景

`gate` 与 `nogate` 启动链路共用 `behavior_tree/sentry_all.launch.py` 和同一套决策代码；`nogate` 只通过 `debug_bypass_is_start:=true` 跳过开赛等待。

原“受击扣血达到阈值后打开开赛门”逻辑挂在 `LeagueStrategy` 下，并且 `WaitBeforeGame` 额外检查 `IsLeagueProfile()`，所以 regional 等配置即使补同名键也不会生效。

## 改动

新增顶层通用配置：

```json
"DamageOpenGate": {
    "Enable": false,
    "HealthDropThreshold": 30
}
```

含义：

- `Enable`：是否启用“等待开赛期间，己方血量下降达到阈值则放行”的额外开赛门。
- `HealthDropThreshold`：从等待期间记录到的最高血量到当前血量的下降阈值，默认 `30`。

旧配置项已移除：

- `LeagueStrategy.EnableDamageOpenGate`
- `LeagueStrategy.DamageOpenGateThreshold`

## 行为

`WaitForGameStart()` 不再按 `CompetitionProfile` 限制此逻辑。只要 `DamageOpenGate.Enable=true`，league、regional、debug/test 配置都能使用。

regional 与其它 JSON 均已加入该配置，但默认 `Enable=false`，因此默认行为不变。

`navi_debug_points.json` 是点位计划文件，不属于 behavior tree 主 `Config` schema，因此没有加入 `DamageOpenGate`。
