# 文档总索引（docs）

本目录是 `ros2_ly_ws_sentry` 的唯一文档入口，按“上手 -> 架构 -> 模块 -> 实机”组织。

## 目录结构

```text
docs/
├── 2026-05-03_README.md         # 当前索引
├── architecture/                # 系统行为、消息链路
├── guides/                      # 配置、测试、上车前清单
├── modules/                     # 各 ROS 包说明
├── record/                      # 已落地的重要改动记录与调参记录
├── sentry/                      # 哨兵决策/姿态专项
├── reports/                     # 阶段性检查报告
├── references/                  # 参考材料（如 logger 文档）
├── rules/                       # 官方规则/通信协议 PDF
└── plans/                       # 历史方案与改造计划
```

## 推荐阅读顺序

1. 全局链路
[architecture/2026-05-03_message_and_link_flow.md](architecture/2026-05-03_message_and_link_flow.md)
2. `/ly/control/angles` 专项链路追踪
[architecture/2026-05-04_control_angles_data_flow.md](architecture/2026-05-04_control_angles_data_flow.md)
3. `/ly/control/firecode` 专项链路追踪
[architecture/2026-05-04_fire_control_flow.md](architecture/2026-05-04_fire_control_flow.md)
4. 当前系统运行行为
[architecture/2026-05-05_system_behavior.md](architecture/2026-05-05_system_behavior.md)
5. 模块文档（建议按数据流）
[modules/2026-05-05_gimbal_driver.md](modules/2026-05-05_gimbal_driver.md)
[modules/2026-05-05_detector.md](modules/2026-05-05_detector.md)
[modules/2026-03-04_tracker_solver.md](modules/2026-03-04_tracker_solver.md)
[modules/2026-04-23_predictor.md](modules/2026-04-23_predictor.md)
[modules/2026-05-05_behavior_tree.md](modules/2026-05-05_behavior_tree.md)
补充模块文档
[modules/2026-05-05_outpost_hitter.md](modules/2026-05-05_outpost_hitter.md)
[modules/2026-05-05_buff_hitter.md](modules/2026-05-05_buff_hitter.md)
[modules/2026-03-05_shooting_table_calib.md](modules/2026-03-05_shooting_table_calib.md)
[modules/2026-04-23_auto_aim_common.md](modules/2026-04-23_auto_aim_common.md)
[modules/2026-05-04_navi_tf_bridge.md](modules/2026-05-04_navi_tf_bridge.md)
6. 哨兵专项（2026 姿态机制）
[sentry/2026-05-04_decision_runtime_behavior.md](sentry/2026-05-04_decision_runtime_behavior.md)
[sentry/2026-05-03_sentry_decision_zone_blocks.md](sentry/2026-05-03_sentry_decision_zone_blocks.md)
[sentry/2026-03-12_sentry_decision_autoaim_manual.md](sentry/2026-03-12_sentry_decision_autoaim_manual.md)
[sentry/2026-04-27_decision_visualization.md](sentry/2026-04-27_decision_visualization.md)
[sentry/2026-05-04_optional_items_demo_runbook.md](sentry/2026-05-04_optional_items_demo_runbook.md)
[sentry/2026-05-04_showcase_demo_runbook.md](sentry/2026-05-04_showcase_demo_runbook.md)
[sentry/2026-05-04_navi_debug_runbook.md](sentry/2026-05-04_navi_debug_runbook.md)
[sentry/2026-03-04_sentry_posture_system.md](sentry/2026-03-04_sentry_posture_system.md)
[sentry/2026-03-03_sentry_posture_interface_change.md](sentry/2026-03-03_sentry_posture_interface_change.md)
[sentry/2026-03-13_posture_lower_firmware_integration.md](sentry/2026-03-13_posture_lower_firmware_integration.md)
[sentry/2026-03-13_posture_firmware_integration_checklist.md](sentry/2026-03-13_posture_firmware_integration_checklist.md)
[sentry/2026-05-02_lower_downlink_message_contract.md](sentry/2026-05-02_lower_downlink_message_contract.md)
7. 落地执行与上车前检查
[guides/2026-03-04_config_setup_guide.md](guides/2026-03-04_config_setup_guide.md)
[guides/2026-04-11_auto_aim_tuning_handover.md](guides/2026-04-11_auto_aim_tuning_handover.md)
[guides/2026-03-17_auto_aim_yamlization_checklist.md](guides/2026-03-17_auto_aim_yamlization_checklist.md)
[guides/2026-05-04_external_topic_boundary.md](guides/2026-05-04_external_topic_boundary.md)
[guides/2026-03-17_self_check_dual_suite.md](guides/2026-03-17_self_check_dual_suite.md)
[guides/2026-05-03_module_standalone_test.md](guides/2026-05-03_module_standalone_test.md)
[guides/2026-03-04_test_guide.md](guides/2026-03-04_test_guide.md)
[guides/2026-03-05_preflight_checklist.md](guides/2026-03-05_preflight_checklist.md)
8. 近期稳定性修复记录（接口不变）
[reports/2026-03-05_stability_fix_no_interface_change.md](reports/2026-03-05_stability_fix_no_interface_change.md)
[reports/2026-03-05_self_check_status.md](reports/2026-03-05_self_check_status.md)
[reports/2026-03-05_repository_completeness_audit.md](reports/2026-03-05_repository_completeness_audit.md)
[reports/2026-03-05_full_link_audit.md](reports/2026-03-05_full_link_audit.md)
9. 重要行为变更与调参记录
[record/2026-03-18_behavior_tree_bt_pinned_and_old_fire_logic.md](record/2026-03-18_behavior_tree_bt_pinned_and_old_fire_logic.md)
[record/2026-03-18_autoaim_follow_fire_change.md](record/2026-03-18_autoaim_follow_fire_change.md)
[record/2026-05-01_tf_tree_integration_for_navi.md](record/2026-05-01_tf_tree_integration_for_navi.md)
[record/2026-05-03_navi_tf_bridge_facemode_and_script_layout.md](record/2026-05-03_navi_tf_bridge_facemode_and_script_layout.md)
[record/2026-05-03_follow_mode_navi_transition_and_external_status.md](record/2026-05-03_follow_mode_navi_transition_and_external_status.md)
[record/2026-05-03_README.md](record/2026-05-03_README.md)

## 分类说明

- `architecture/`
  - 系统级说明，先读它再看单包。
- `modules/`
  - 每个包的职责、topic、关键实现点。
- `sentry/`
  - 决策与姿态设计文档，和比赛规则最相关。
- `guides/`
  - 面向实机调试和赛前执行。
- `reports/`
  - 历史检查结果，可能与当前代码存在时间差。
- `record/`
  - 已落地的重要变更、现场调参与风险结论，优先写事实与当前建议。
- `plans/`
  - 方案设计记录，不代表最终已合入实现。

## 脚本说明入口

- `scripts/README.md`：脚本用途与命令总览（离车自检、上车自检、基础套件）。

## 历史与兼容文档

- 旧行为说明（保留对照）
[architecture/2026-04-22_system_behavior_v1.md](architecture/2026-04-22_system_behavior_v1.md)
- 旧版射表标定说明（保留对照）
[modules/2026-03-04_shooting_table_calib_usage_legacy.md](modules/2026-03-04_shooting_table_calib_usage_legacy.md)

## 文档维护约定

1. 新增文档统一放 `docs/`，禁止再落到仓库根目录。
2. 涉及接口变更（topic/msg/参数）时，至少同步更新：
   - 对应模块文档
   - `architecture/2026-05-03_message_and_link_flow.md`
   - 本索引文件
3. 实机流程改动后，优先更新：
   - `guides/2026-03-04_config_setup_guide.md`
   - `guides/2026-03-05_preflight_checklist.md`
