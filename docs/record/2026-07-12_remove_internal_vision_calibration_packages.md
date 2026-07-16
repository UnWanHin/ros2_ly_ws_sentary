# 移除内部视觉与标定包

Updated: 2026-07-12

## 决策

本仓正式辅瞄来源已经收敛为外部 `/ly/aim/*`。因此删除不再由 `sentry_all` 启动、且没有正式下游依赖的内部相机、检测、追踪、预测、打符、前哨与射表标定包。

删除包：

- `detector`
- `tracker_solver`
- `predictor`
- `outpost_hitter`
- `buff_hitter`
- `shooting_table_calib`
- `buff_shooting_table_calib`

同时删除这些包专用的调试、标定、拟合、standalone 入口和 simulator legacy mock。`behavior_tree` 也不再订阅旧的 `/ly/detector/armors`、`/ly/predictor/target`、`/ly/buff/target`、`/ly/outpost/target`，避免删除 producer 后留下看似可用的死接口。历史报告可以保留旧名作为历史事实，但不得把它们当作当前可运行链路。

## 保留范围

- `behavior_tree`：策略、Regional、姿态、导航、外部 aim 消费、控制发布。
- `gimbal_driver`：下位机和裁判系统串口、上行语义、下行控制。
- `navi_tf_bridge`：导航目标、路径到裁判 `0x0307`、FaceMode 支撑。
- `auto_aim_common`：仍由 `behavior_tree` 使用 `GoalReach`、`navi_tf_bridge` 使用 `RelativeTarget`；保留其旧消息定义不等于恢复旧视觉包。
- `simulator`：离线决策与 trace 工具；只保留正式 `/ly/aim/*` mock，不再发布旧视觉 topic。

## 当前正式接口

```text
external /ly/aim/armor_targets + /ly/aim/result
  -> behavior_tree
  -> /ly/control/angles + /ly/control/firecode
  -> gimbal_driver
  -> lower controller
```

- `behavior_tree` 向外部 aim 发布 `/ly/aim/select_target`。
- `ExternalAim.UseTargetArrayAsArmorList=true` 时，`AimResult` 必须与新鲜的匹配 target-array context 一起使用。
- `/ly/gimbal/angles` 是 RuntimeGuard 的必要反馈；缺失时 BT 会进入 `gimbal_stale` 安全控制，不能把没有该反馈的本地注入结果当作端到端通过。

## 验证边界

- 本次清理的原始验证已完成；2026-07-16 起本仓 `tf_tree` fallback 已移除，后续验证包清单为 `auto_aim_common gimbal_driver navi_tf_bridge behavior_tree simulator`，gimbal TF 由外部 `sentry_tf` 唯一提供。
- WSL 环境没有真实云台反馈和外部 aim 实例，因此不能声称硬件端到端控制链已经通过；实机仍需验证 `/ly/gimbal/angles`、外部 `/ly/aim/*` 与实际下行控制。
