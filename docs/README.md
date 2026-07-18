# 文档总索引（docs）

本目录是 `ros2_ly_ws_sentry` 的唯一文档入口，按“上手 -> 架构 -> 模块 -> 实机”组织。

当前 `Behavion` 正式主链是 decision-only：本仓 `sentry_all` 启动 `gimbal_driver`、`navi_tf_bridge` / FaceMode 和 `behavior_tree`，外部 `sentry.aim` / `sentry_tf` 提供相机、检测、追踪、弹道、gimbal TF 和最终 aim/fire 门控。内部视觉、预测、打符、前哨和射表标定包已移除。

## 目录结构

```text
docs/
├── README.md                    # 当前索引
├── architecture/                # 系统行为、消息链路
├── guides/                      # 配置、测试、上车前清单
├── modules/                     # 各 ROS 包说明
├── record/                      # 已落地的重要改动记录与调参记录
├── sentry/                      # 当前有效哨兵专项文档
├── reports/                     # 阶段性检查报告
├── references/                  # 参考材料（如 logger 文档）
├── rules/                       # 官方规则/通信协议 PDF
└── plans/                       # 历史方案与改造计划
```

## 推荐阅读顺序

1. 全局链路
[architecture/2026-07-12_project_link_graph.md](architecture/2026-07-12_project_link_graph.md)
2. 内部视觉移除与外部 aim 迁移
[record/2026-07-12_remove_internal_vision_calibration_packages.md](record/2026-07-12_remove_internal_vision_calibration_packages.md)
3. 当前系统运行行为
[sentry/internal/ros2_topic_structure.md](sentry/internal/ros2_topic_structure.md)
[sentry/internal/ros2_topic_tree.md](sentry/internal/ros2_topic_tree.md)
4. 当前主链模块文档
[modules/2026-05-05_gimbal_driver.md](modules/2026-05-05_gimbal_driver.md)
[modules/2026-05-05_behavior_tree.md](modules/2026-05-05_behavior_tree.md)
[modules/2026-05-04_navi_tf_bridge.md](modules/2026-05-04_navi_tf_bridge.md)
5. 共用接口与移除记录
[modules/2026-04-23_auto_aim_common.md](modules/2026-04-23_auto_aim_common.md)
[record/2026-07-12_remove_internal_vision_calibration_packages.md](record/2026-07-12_remove_internal_vision_calibration_packages.md)
7. 哨兵专项（当前有效）
[sentry/README.md](sentry/README.md)
[sentry/regional/current_behavior.md](sentry/regional/current_behavior.md)
[sentry/regional/decision_framework.md](sentry/regional/decision_framework.md)
[sentry/regional/patrol_scan_modes.md](sentry/regional/patrol_scan_modes.md)
[sentry/internal/simulator.md](sentry/internal/simulator.md)
[sentry/embedded/serial_data_mapping.md](sentry/embedded/serial_data_mapping.md)
[sentry/embedded/downlink_control_frame.md](sentry/embedded/downlink_control_frame.md)
[sentry/info/rule_resource_profile.md](sentry/info/rule_resource_profile.md)
8. 落地执行与上车前检查
[guides/2026-05-04_external_topic_boundary.md](guides/2026-05-04_external_topic_boundary.md)
[guides/2026-03-17_self_check_dual_suite.md](guides/2026-03-17_self_check_dual_suite.md)
[record/2026-07-12_remove_internal_vision_calibration_packages.md](record/2026-07-12_remove_internal_vision_calibration_packages.md)
9. 近期稳定性修复记录（接口不变）
[reports/2026-03-05_stability_fix_no_interface_change.md](reports/2026-03-05_stability_fix_no_interface_change.md)
[reports/2026-03-05_self_check_status.md](reports/2026-03-05_self_check_status.md)
[reports/2026-03-05_repository_completeness_audit.md](reports/2026-03-05_repository_completeness_audit.md)
[reports/2026-03-05_full_link_audit.md](reports/2026-03-05_full_link_audit.md)
[reports/2026-07-16_config_script_inventory.md](reports/2026-07-16_config_script_inventory.md)
10. 重要行为变更与调参记录
[record/2026-07-18_complete_regional_roadland_split_activation.md](record/2026-07-18_complete_regional_roadland_split_activation.md)
[record/2026-07-16_remove_local_tf_tree.md](record/2026-07-16_remove_local_tf_tree.md)
[record/2026-03-18_behavior_tree_bt_pinned_and_old_fire_logic.md](record/2026-03-18_behavior_tree_bt_pinned_and_old_fire_logic.md)
[record/2026-03-18_autoaim_follow_fire_change.md](record/2026-03-18_autoaim_follow_fire_change.md)
[record/2026-05-01_tf_tree_integration_for_navi.md](record/2026-05-01_tf_tree_integration_for_navi.md)（历史；本仓 fallback 已移除）
[record/2026-05-03_navi_tf_bridge_facemode_and_script_layout.md](record/2026-05-03_navi_tf_bridge_facemode_and_script_layout.md)（历史；本仓 fallback 已移除）
[record/2026-05-03_follow_mode_navi_transition_and_external_status.md](record/2026-05-03_follow_mode_navi_transition_and_external_status.md)
[record/2026-05-03_README.md](record/2026-05-03_README.md)
[record/2026-07-11_typeid10_sentry_info3_outpost_hp.md](record/2026-07-11_typeid10_sentry_info3_outpost_hp.md)

## 分类说明

- `architecture/`
  - 系统级说明，先读它再看单包。
- `modules/`
  - 每个包的职责、topic、关键实现点。
- `sentry/`
  - 当前有效哨兵专项文档，按 `regional/league/internal/external/embedded/info` 维护。
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

## 文档维护约定

1. 新增文档统一放 `docs/`，禁止再落到仓库根目录。
2. 涉及接口变更（topic/msg/参数）时，至少同步更新：
   - 对应模块文档
   - `architecture/2026-07-12_project_link_graph.md`
   - 本索引文件
3. 实机流程改动后，优先更新 `guides/2026-05-04_external_topic_boundary.md`、对应模块文档和本索引。
