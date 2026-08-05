# ros2_ly_ws_sentry 接手阅读指南

Updated: 2026-08-05

这是一个 ROS2 Humble / `colcon` 哨兵上位机工作区。当前 `Behavion` 主链路已经切到 **decision-only**：本仓负责下位机串口、导航/FaceMode bridge 和行为树决策；相机、检测、追踪、弹道、gimbal TF 和最终 aim/firing 判定由外部 `sentry.aim` / `sentry_tf` 提供。

## 当前正式主链

```text
sentry.aim
  -> /ly/aim/armor_targets
  -> behavior_tree
  -> /ly/aim/select_target
  -> sentry.aim
  -> /ly/aim/result
  -> behavior_tree
  -> /ly/control/angles + /ly/control/firecode + /ly/control/vel/posture/sentry_cmd
  -> gimbal_driver
  -> 下位机
```

内部相机、检测、追踪、预测、打符、前哨和射表标定包已经移除。迁移范围与验证边界见
[docs/record/2026-07-12_remove_internal_vision_calibration_packages.md](docs/record/2026-07-12_remove_internal_vision_calibration_packages.md)。

## 推荐阅读顺序

1. 项目总索引
   [docs/README.md](docs/README.md)
2. 当前系统链路与行为
   [docs/architecture/2026-07-12_project_link_graph.md](docs/architecture/2026-07-12_project_link_graph.md)
   [docs/record/2026-07-12_remove_internal_vision_calibration_packages.md](docs/record/2026-07-12_remove_internal_vision_calibration_packages.md)
   [docs/sentry/internal/ros2_topic_structure.md](docs/sentry/internal/ros2_topic_structure.md)
3. 当前主链模块
   [docs/modules/2026-05-05_gimbal_driver.md](docs/modules/2026-05-05_gimbal_driver.md)
   [docs/modules/2026-05-05_behavior_tree.md](docs/modules/2026-05-05_behavior_tree.md)
   [docs/modules/2026-05-04_navi_tf_bridge.md](docs/modules/2026-05-04_navi_tf_bridge.md)
4. 哨兵决策与模拟器
   [docs/sentry/README.md](docs/sentry/README.md)
   [docs/sentry/regional/current_behavior.md](docs/sentry/regional/current_behavior.md)
   [docs/sentry/regional/decision_framework.md](docs/sentry/regional/decision_framework.md)
   [docs/sentry/regional/patrol_scan_modes.md](docs/sentry/regional/patrol_scan_modes.md)
   [docs/sentry/internal/simulator.md](docs/sentry/internal/simulator.md)
5. 串口、裁判系统和实机检查
   [docs/sentry/embedded/serial_data_mapping.md](docs/sentry/embedded/serial_data_mapping.md)
   [docs/sentry/embedded/downlink_control_frame.md](docs/sentry/embedded/downlink_control_frame.md)
   [docs/guides/2026-05-04_external_topic_boundary.md](docs/guides/2026-05-04_external_topic_boundary.md)

历史姿态和旧自瞄说明已归档到 `docs/record/`，例如：

- [docs/record/2026-03-12_sentry_decision_autoaim_manual.md](docs/record/2026-03-12_sentry_decision_autoaim_manual.md)
- [docs/record/2026-03-04_sentry_posture_system.md](docs/record/2026-03-04_sentry_posture_system.md)
- [docs/record/2026-03-03_sentry_posture_interface_change.md](docs/record/2026-03-03_sentry_posture_interface_change.md)

## 常用命令

```bash
cd ~/ros2_ly_ws_sentry
source /opt/ros/humble/setup.bash
source /home/hustlyrm/sentry.aim/install/setup.bash
colcon build --allow-overriding gimbal_driver
source install/setup.bash
./scripts/selfcheck.sh sentry --static-only
```

`gimbal_driver` owns the MPC gimbal interfaces: `/ly/gimbal/state` uses
`gimbal_driver/msg/GimbalState` and `/ly/control/trajectory` uses
`gimbal_driver/msg/GimbalTrajectory`. External `sentry.aim` consumes/publishes
these topics, but is not a build dependency of this workspace.

正式/调试入口：

```bash
./scripts/start.sh gated --mode league
./scripts/start.sh gated --mode regional
./scripts/start.sh nogate --mode regional
./scripts/debug.sh armor_test --mode regional
./scripts/selfcheck.sh sentry --skip-hz
```

离线 simulator：

```bash
python3 scripts/python/start.py
```

本地文档与关系图：

```bash
./tools/Library.sh
```

打开 `http://127.0.0.1:1037/`。Documentation、Graph 和 Split 是同一份
`docs/**/*.md` 的不同视图；Markdown 链接和 Obsidian wikilink 会自动显示为关系，
不需要生成或维护独立图数据。

## 交接：本地图谱、端口与工具

下一位开发者先从这三个入口开始；它们分别服务于**理解工程**、**离线验证决策**和
**运行／自检主链**，不要把它们与正式机器人控制界面混为一谈。

| 端口 | 归属与用途 | 启动方式 | 备注 |
| --- | --- | --- | --- |
| `1037` | 本仓文件网站与工程关系图 | `./tools/Library.sh` | Documentation、Graph、Split 都直接读取 `docs/**/*.md`。每份 Markdown 是一个图节点，Markdown／Obsidian 链接是关系边；**不要维护独立 graph JSON**。 |
| `9011` | 浏览器优先的离线 Tactical Simulator | `./tools/Simulator.sh` | 默认生产页面为 `/`，`/tactical` 只是别名。用于 mock／trace 的离线决策验证；pygame 仅能以 `--debug-pygame` 作为本机诊断 renderer，不是第二套生产 UI。 |
| `9000` | 直接调用 `scripts/python/start.py` 的 simulator 默认端口 | `python3 scripts/python/start.py` | `Simulator.sh` 会明确覆盖为 `9011`。需要并行运行时用 `--web-port <port>` 或 `SIMULATOR_WEB_PORT=<port>`，避免占用同一端口。 |
| `6011` | **不属于本仓固定接口** | 无 | 当前仓库没有 6011 的启动器、配置或 API 契约。若现场已有该端口服务，先用 `ss -ltnp | rg ':6011\\b'` 确认进程与所属工程；不得假定它是 simulator、图谱站或 ROS 主链的一部分。 |

常用工具与边界：

| 工具 | 用途 | 交接约束 |
| --- | --- | --- |
| `./tools/Library.sh` | 启动 1037 文件／图谱网站 | 文件是唯一真相；修改 `docs/` 后图谱会自动更新，不需要生成、提交或同步图数据库。 |
| `./tools/Simulator.sh` | 启动 9011 离线 Tactical Simulator | 浏览器是唯一生产 UI；它通过既有 command bus 驱动 mock／offline 状态，不重写正式 BT、ROS 或下位机逻辑。 |
| `./scripts/start.sh` | 正式／展示启动入口 | 正式比赛优先使用 `gated --mode league` 或 `gated --mode regional`；外部 `sentry.aim`／`sentry_tf`／导航属于仓外依赖。 |
| `./scripts/debug.sh` | 有边界的联调入口 | 仅在对应 debug profile 使用；不要与正式 BT 对同一 `/ly/control/*` topic 并行发布。 |
| `./scripts/selfcheck.sh sentry --static-only` | 交接后的首轮静态检查 | 改 topic、message、参数或主链时，再执行相应的 runtime／launch 检查。 |
| `tools/Behaviortree/`、`tools/maps/`、`tools/PnPcamera/` | 分别为离线 BT XML 查看、地图／点位维护、相机内参标定 | 它们是辅助工具，不是额外的正式仿真或控制主链。 |

图谱和架构的阅读顺序：先打开 1037 的 `docs/README.md`，再阅读
[`docs/architecture/2026-07-12_project_link_graph.md`](docs/architecture/2026-07-12_project_link_graph.md)、
[`docs/sentry/regional/current_behavior.md`](docs/sentry/regional/current_behavior.md) 与各模块文档。
若文档与当前 source、launch、package manifest 或 message 定义冲突，以 source 为准，并在同一次修改中更新最近的文档。

## 维护约定

- 根目录只保留这份接手指南；新增文档放到 `docs/`。
- 改 topic/msg/参数时，同步更新对应模块文档、系统链路文档和 `docs/README.md`。
- 改行为树 trace schema、导航点位、姿态字段、target 字段或 unit state 字段时，同步更新 `src/simulator` 相关文档。
- `build/`、`install/`、`log/` 是生成目录，不提交。
