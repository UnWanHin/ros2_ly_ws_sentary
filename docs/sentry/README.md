# Sentry Current Docs

Updated: 2026-05-06

`docs/sentry` 只放当前有效的哨兵专项说明，不再按日期堆放历史记录。历史改动、旧 runbook、旧接口迁移记录放到 `docs/record/`。

## 当前入口

| 目录 | 作用 |
|---|---|
| `regional/` | Regional 决策行为、大区域状态机、区域任务和视觉/任务模式语义 |
| `league/` | League 模式专项说明；当前仅保留入口，后续联赛模式差异写在这里 |
| `internal/` | 本仓内部 ROS 链路、BT trace、离线工具和维护接口 |
| `external/` | 外部导航等非本仓节点提供的 ROS topic/interface |
| `embedded/` | 电控/下位机串口协议、上下行数据映射和下发控制幀 |
| `info/` | 当前策略会用到的赛规血量、弹量、回血、兑弹、脱战信息 |

## 维护约定

- 改 Regional BT 决策行为、区域任务、模式语义时，更新 `regional/`。
- 改 League 专项行为时，更新 `league/`。
- 改本仓内部 ROS topic、trace schema、调试工具接口时，更新 `internal/`。
- 改外部导航或其它外部包提供的接口时，更新 `external/`。
- 改串口结构、上下位机字段、电控下发/回读约定时，更新 `embedded/`。
- 改赛规相关阈值、资源模型、回血/兑弹/复活理解时，更新 `info/`。
- 只记录一次性调试、历史迁移、旧方案对照时，写到 `docs/record/`，不要放回 `docs/sentry/` 根目录。
