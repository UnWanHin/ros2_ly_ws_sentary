# Scripts Guide

这次把 `scripts/` 重新收敛了，根目录只保留 3 个入口：

- `./scripts/start.sh`
- `./scripts/debug.sh`
- `./scripts/selfcheck.sh`

它们都支持两种用法：

- 不带参数：进入交互菜单
- 带子命令：直接转发到对应分类脚本

例如：

```bash
./scripts/start.sh
./scripts/start.sh gated --mode league
./scripts/debug.sh autoaim-debug --online
./scripts/selfcheck.sh sentry --runtime-only --launch --wait 12 --skip-hz
```

## 目录结构

```text
scripts/
├── start.sh                 # 启动类总入口（交互菜单）
├── debug.sh                 # 调试/联调类总入口（交互菜单）
├── selfcheck.sh             # 自检类总入口（交互菜单）
├── start/                   # 启动类分类脚本
├── debug/                   # 调试类分类脚本
├── selfcheck/               # 自检类分类脚本
├── launch/                  # 真实启动实现
├── feature_test/            # 单项功能测试框架
├── tools/                   # 工具脚本
└── config/                  # 比赛/测试配置
```

原则：

- `scripts/start/`、`scripts/debug/`、`scripts/selfcheck/` 是你平时真正需要打开的分类入口。
- `scripts/launch/` 是实现层，不是给人记命令用的。
- 旧的根目录壳脚本已经删掉，避免同一件事出现两三个名字。

## 一眼看懂

### 1. 正式比赛主入口

用这个：

```bash
./scripts/start.sh gated --mode league      # 联盟赛，有门控
./scripts/start.sh gated --mode regional    # 分区赛，有门控
./scripts/start.sh nogate --mode league     # 联盟赛，无门控
./scripts/start.sh nogate --mode regional   # 分区赛，无门控
```

对应脚本：

- `scripts/start/sentry_all.sh`
- `scripts/start/sentry_all_nogate.sh`
- 实际实现：`scripts/launch/start_sentry_all.sh`

### 2. `competition_autoaim` 是什么

现在用这个名字：

```bash
./scripts/debug.sh competition-autoaim
```

对应脚本：

- `scripts/debug/competition_autoaim.sh`
- 实际实现：`scripts/launch/start_competition_autoaim_test.sh`

它不是“正式比赛完全等价入口”，而是“比赛风格辅瞄预设”。

默认行为：

- 使用 `scripts/config/auto_aim_config_competition.yaml`
- 默认 `--mode regional`
- 默认 `--nogate`
- 默认启用：`gimbal_driver / detector / tracker_solver / predictor / behavior_tree`
- 默认关闭：`buff_hitter / outpost_hitter`

所以它适合：

- 快速验证比赛风格的 autoaim 主链
- 快速看 `behavior_tree` + `predictor` 的联调效果

但它不等于正式比赛整套默认路径，因为正式比赛主入口仍然是：

```bash
./scripts/start.sh gated --mode league
./scripts/start.sh gated --mode regional
```

### 3. `showcase` 是什么

`showcase` 不是联赛，也不是分区赛。

它只是“姿态展示 / 展示巡逻”模式。

现在用这个：

```bash
./scripts/start.sh showcase
```

对应脚本：

- `scripts/start/showcase.sh`
- 实际实现：`scripts/launch/start_sentry_showcase.sh`

核心特点：

- 固定走 `mode 3`
- 底层还是 `regional` 主流程
- 自动换成 `showcase_competition.json`
- 用于姿态展示、展示巡逻、演示链路

### 4. `start_sentry_all_competition.sh` 还要不要

不要了，已经删掉。

原因很简单：

- 它原来只是兼容壳
- 最终还是转发到 `start_sentry_all.sh`
- 保留它只会让“正式入口到底是谁”更乱

## 分类脚本对照

### Start

| 分类脚本 | 用途 | 实际实现 |
| --- | --- | --- |
| `scripts/start/sentry_all.sh` | 正式整链路入口 | `scripts/launch/start_sentry_all.sh` |
| `scripts/start/sentry_all_nogate.sh` | 绕过开赛门控的整链路入口 | `scripts/launch/start_sentry_all_nogate.sh` |
| `scripts/start/showcase.sh` | 姿态展示 / 展示巡逻 | `scripts/launch/start_sentry_showcase.sh` |

### Debug

| 分类脚本 | 用途 | 实际实现 |
| --- | --- | --- |
| `scripts/debug/competition_autoaim.sh` | 比赛风格辅瞄预设 | `scripts/launch/start_competition_autoaim_test.sh` |
| `scripts/debug/autoaim_debug.sh` | 感知链 / mapper / 火控联调 | `scripts/launch/start_autoaim_debug.sh` |
| `scripts/debug/autoaim_test.sh` | 一键装甲板辅瞄测试 | `scripts/feature_test/standalone/modes/armor_mode.sh` |
| `scripts/debug/navi_debug.sh` | behavior_tree-only 导航调试 | `scripts/launch/start_sentry_navi_debug.sh` |
| `scripts/debug/standalone.sh` | 单项功能测试菜单 | `scripts/feature_test/standalone/run_standalone_menu.sh` |
| `scripts/debug/navi_patrol.sh` | JSON 巡逻点发 `/ly/navi/goal` | `scripts/feature_test/standalone/modes/navi_patrol_mode.sh` |
| `scripts/debug/navi_goal_cli.sh` | 手动发导航目标 | `scripts/feature_test/standalone/tools/navi_goal_cli_pub.py` |
| `scripts/debug/ballistic_error_log.sh` | 过滤弹道/锁敌日志 | `scripts/tools/monitor_ballistic_errors.sh` |
| `scripts/debug/shooting_table_calib.sh` | 射表标定 | `scripts/launch/start_shooting_table_calib.sh` |
| `scripts/debug/shooting_table_autoaim.sh` | 射表直连 autoaim 验证 | `scripts/launch/start_shooting_table_autoaim.sh` |
| `scripts/debug/chassis_gyro.sh` | 小陀螺/Rotate 链路测试 | `scripts/feature_test/standalone/modes/chassis_spin_mode.sh` |

### Selfcheck

| 分类脚本 | 用途 |
| --- | --- |
| `scripts/selfcheck/pc.sh` | 开发机 build + 静态自检 |
| `scripts/selfcheck/robot.sh` | 上车硬件/网络预检查 + 运行态自检包装 |
| `scripts/selfcheck/sentry.sh` | 底层核心套件，可做静态或运行态检查 |

## 三个顶层菜单怎么用

### `./scripts/start.sh`

菜单项：

1. `gated`
2. `nogate`

说明：

- 进入后会继续二次询问你要跑 `league` 还是 `regional`
- `showcase` 仍保留为直达入口，但不出现在交互菜单里：

```bash
./scripts/start.sh showcase
```

### `./scripts/debug.sh`

菜单项：

1. `competition-autoaim`
2. `autoaim-debug`
3. `autoaim-test`
4. `navi-debug`
5. `standalone`
6. `navi-patrol`
7. `navi-goal-cli`
8. `ballistic-log`
9. `shooting-table-calib`
10. `shooting-table-autoaim`
11. `chassis-gyro`

### `./scripts/selfcheck.sh`

菜单项：

1. `pc`
2. `robot`
3. `sentry`

区别：

- `pc`：开发机用。可选 build，然后跑静态检查。
- `robot`：车上用。先看串口/权限/网络，再调用 `sentry` 做运行态检查。
- `sentry`：底层总套件。既能 `--static-only`，也能 `--runtime-only --launch`。`pc` 和 `robot` 都是在包装它。

## 常用命令

```bash
# 联盟赛，有门控
./scripts/start.sh gated --mode league

# 分区赛，无门控
./scripts/start.sh nogate --mode regional

# 展示姿态
./scripts/start.sh showcase

# 比赛风格 autoaim 预设
./scripts/debug.sh competition-autoaim --mode league

# 射表标定
./scripts/debug.sh shooting-table-calib --team red --output screen

# 一键装甲板联调
./scripts/debug.sh autoaim-test --online

# 导航调试
./scripts/debug.sh navi-debug

# 离车自检
./scripts/selfcheck.sh pc

# 上车运行态自检
./scripts/selfcheck.sh robot --with-hz
```

## 备注

- 这次重构只收敛了 `scripts/` 的入口层，`scripts/launch/`、`feature_test/`、`tools/` 仍保留实现。
- 仓库里其他文档如果还出现旧命令，按上面的新入口映射替换理解即可。
