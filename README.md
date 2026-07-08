# ros2_ly_ws_sentry 接手阅读指南

Updated: 2026-07-08

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

正式入口不会启动本仓 legacy internal auto-aim 的 `detector / tracker_solver / predictor / outpost_hitter / buff_hitter`。这些包仍保留给单测、调试和历史链路对照。

## 推荐阅读顺序

1. 项目总索引
   [docs/README.md](docs/README.md)
2. 当前系统链路与行为
   [docs/architecture/2026-05-03_message_and_link_flow.md](docs/architecture/2026-05-03_message_and_link_flow.md)
   [docs/architecture/2026-05-05_system_behavior.md](docs/architecture/2026-05-05_system_behavior.md)
   [docs/sentry/internal/ros2_topic_structure.md](docs/sentry/internal/ros2_topic_structure.md)
3. 当前主链模块
   [docs/modules/2026-05-05_gimbal_driver.md](docs/modules/2026-05-05_gimbal_driver.md)
   [docs/modules/2026-05-05_behavior_tree.md](docs/modules/2026-05-05_behavior_tree.md)
   [docs/modules/2026-05-04_navi_tf_bridge.md](docs/modules/2026-05-04_navi_tf_bridge.md)
4. Legacy / 调试视觉链路
   [docs/modules/2026-05-05_detector.md](docs/modules/2026-05-05_detector.md)
   [docs/modules/2026-03-04_tracker_solver.md](docs/modules/2026-03-04_tracker_solver.md)
   [docs/modules/2026-04-23_predictor.md](docs/modules/2026-04-23_predictor.md)
   [docs/modules/2026-05-05_outpost_hitter.md](docs/modules/2026-05-05_outpost_hitter.md)
   [docs/modules/2026-05-05_buff_hitter.md](docs/modules/2026-05-05_buff_hitter.md)
5. 哨兵决策与模拟器
   [docs/sentry/README.md](docs/sentry/README.md)
   [docs/sentry/regional/current_behavior.md](docs/sentry/regional/current_behavior.md)
   [docs/sentry/regional/decision_framework.md](docs/sentry/regional/decision_framework.md)
   [docs/sentry/regional/patrol_scan_modes.md](docs/sentry/regional/patrol_scan_modes.md)
   [docs/sentry/internal/simulator.md](docs/sentry/internal/simulator.md)
6. 串口、裁判系统和实机检查
   [docs/sentry/embedded/serial_data_mapping.md](docs/sentry/embedded/serial_data_mapping.md)
   [docs/sentry/embedded/downlink_control_frame.md](docs/sentry/embedded/downlink_control_frame.md)
   [docs/guides/2026-03-04_config_setup_guide.md](docs/guides/2026-03-04_config_setup_guide.md)
   [docs/guides/2026-03-05_preflight_checklist.md](docs/guides/2026-03-05_preflight_checklist.md)

历史姿态和旧自瞄说明已归档到 `docs/record/`，例如：

- [docs/record/2026-03-12_sentry_decision_autoaim_manual.md](docs/record/2026-03-12_sentry_decision_autoaim_manual.md)
- [docs/record/2026-03-04_sentry_posture_system.md](docs/record/2026-03-04_sentry_posture_system.md)
- [docs/record/2026-03-03_sentry_posture_interface_change.md](docs/record/2026-03-03_sentry_posture_interface_change.md)

## 常用命令

```bash
cd ~/ros2_ly_ws_sentry
colcon build
source install/setup.bash
./scripts/selfcheck.sh sentry --static-only
```

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

## 维护约定

- 根目录只保留这份接手指南；新增文档放到 `docs/`。
- 改 topic/msg/参数时，同步更新对应模块文档、系统链路文档和 `docs/README.md`。
- 改行为树 trace schema、导航点位、姿态字段、target 字段或 unit state 字段时，同步更新 `src/simulator` 相关文档。
- `build/`、`install/`、`log/` 是生成目录，不提交。
