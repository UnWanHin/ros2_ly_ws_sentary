# ros2_ly_ws_sentary 新接手阅读指南

这个仓库的文档已经统一收敛到 [`docs/`](docs/2026-05-03_README.md)。

如果你是刚接手项目，按下面顺序读，能最快建立全局认知：

1. 项目总览与目录入口
[`docs/2026-05-03_README.md`](docs/2026-05-03_README.md)
2. 系统链路与运行行为
[`docs/architecture/2026-05-03_message_and_link_flow.md`](docs/architecture/2026-05-03_message_and_link_flow.md)
[`docs/architecture/2026-05-05_system_behavior.md`](docs/architecture/2026-05-05_system_behavior.md)
3. 核心模块（建议按链路顺序）
[`docs/modules/2026-05-05_gimbal_driver.md`](docs/modules/2026-05-05_gimbal_driver.md)
[`docs/modules/2026-05-05_detector.md`](docs/modules/2026-05-05_detector.md)
[`docs/modules/2026-03-04_tracker_solver.md`](docs/modules/2026-03-04_tracker_solver.md)
[`docs/modules/2026-04-23_predictor.md`](docs/modules/2026-04-23_predictor.md)
[`docs/modules/2026-05-05_behavior_tree.md`](docs/modules/2026-05-05_behavior_tree.md)
补充模块（按需查阅）
[`docs/modules/2026-05-05_outpost_hitter.md`](docs/modules/2026-05-05_outpost_hitter.md)
[`docs/modules/2026-05-05_buff_hitter.md`](docs/modules/2026-05-05_buff_hitter.md)
[`docs/modules/2026-03-05_shooting_table_calib.md`](docs/modules/2026-03-05_shooting_table_calib.md)
[`docs/modules/2026-04-23_auto_aim_common.md`](docs/modules/2026-04-23_auto_aim_common.md)
4. 哨兵决策与姿态（2026 重点）
[`docs/sentry/2026-03-12_sentry_decision_autoaim_manual.md`](docs/sentry/2026-03-12_sentry_decision_autoaim_manual.md)
[`docs/sentry/2026-03-04_sentry_posture_system.md`](docs/sentry/2026-03-04_sentry_posture_system.md)
[`docs/sentry/2026-03-03_sentry_posture_interface_change.md`](docs/sentry/2026-03-03_sentry_posture_interface_change.md)
5. 上车前和实机检查
[`docs/guides/2026-03-04_config_setup_guide.md`](docs/guides/2026-03-04_config_setup_guide.md)
[`docs/guides/2026-05-03_module_standalone_test.md`](docs/guides/2026-05-03_module_standalone_test.md)
[`docs/guides/2026-03-05_preflight_checklist.md`](docs/guides/2026-03-05_preflight_checklist.md)
[`docs/guides/2026-03-04_test_guide.md`](docs/guides/2026-03-04_test_guide.md)

常用命令（本工作区）：

```bash
cd ~/ros2_ly_ws_sentary
colcon build
source install/setup.bash
./scripts/selfcheck.sh sentry --skip-hz
```

也可以直接用三个顶层菜单入口：

```bash
./scripts/start.sh
./scripts/debug.sh
./scripts/selfcheck.sh
```


说明：
- 根目录仅保留这份接手指南，其他文档已迁入 `docs/`。
- 比赛规则与通信协议 PDF 在 `docs/rules/`。
