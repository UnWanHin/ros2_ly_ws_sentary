# 系统行为说明（历史版本）

> 本文包含已移除的内部视觉链路。当前运行行为以
> [`2026-07-12_project_link_graph.md`](2026-07-12_project_link_graph.md) 和
> [`../record/2026-07-12_remove_internal_vision_calibration_packages.md`](../record/2026-07-12_remove_internal_vision_calibration_packages.md) 为准。

Updated: 2026-05-27

本文描述当前 `Behavion` 分支的正式运行行为。旧的本仓内部自瞄链路仍可用于调试，但不再是正式 `sentry_all` 主链。

## 当前结论

### launch 后会不会自动打？

正式比赛入口会不会下发开火，取决于三层条件同时成立：

1. `behavior_tree` 正在运行，并允许当前策略开火。
2. 外部 `sentry.aim` 发布 `/ly/aim/result`，且 `follow=true`。
3. 同一帧 `/ly/aim/result.fire=true`，BT 才翻转 `/ly/control/firecode`。

`follow=false` 时，BT 不接管该帧 yaw/pitch，也不会使用该帧 fire。`fire=false` 但 `follow=true` 时，BT 可以接管角度，但不会开火。

## 正式主链路

正式入口：

```bash
./scripts/start.sh gated --mode league
./scripts/start.sh gated --mode regional
./scripts/start.sh nogate --mode league
./scripts/start.sh nogate --mode regional
```

底层最终进入：

```bash
ros2 launch behavior_tree sentry_all.launch.py
```

当前正式 `sentry_all` 默认启动：

- `gimbal_driver`
- `navi_tf_bridge` 的导航目标 bridge
- `map_aim_point_node` 作为 BT FaceMode solver
- `behavior_tree`

当前正式 `sentry_all` 默认不启动：

- `detector`
- `tracker_solver`
- `predictor`
- `outpost_hitter`
- `buff_hitter`
- 本仓不再发布 gimbal TF；正式与调试都要求外部 `sentry_tf`。

主数据流：

```text
[sentry.aim] -> /ly/aim/armor_targets
      |
      v
[behavior_tree] 根据区域任务、姿态、血量、目标优先级选择目标
      |
      v
/ly/aim/select_target -> [sentry.aim]
      |
      v
/ly/aim/result (follow, fire, yaw, pitch)
      |
      v
[behavior_tree]
      |
      +-> /ly/control/angles
      +-> /ly/control/firecode
      +-> /ly/control/vel
      +-> /ly/control/posture
      +-> /ly/control/sentry_cmd
      |
      v
[gimbal_driver] -> 串口主控制帧 -> 下位机
```

TF 关系：

- 外部 `sentry_tf` 是 gimbal TF 的唯一发布者。
- 本仓 `navi_tf_bridge` 只查询和使用该 TF，不发布 TF。

## `/ly/aim/*` 语义

| Topic | Direction | Type | 当前语义 |
|---|---|---|---|
| `/ly/aim/armor_targets` | external aim -> BT | `sentry_msgs/msg/AimTargetArray` | 外部 aim 给出的可打目标候选。BT 用于目标优先级、Chase 相对点和 `/ly/aim/select_target` 回填。 |
| `/ly/aim/select_target` | BT -> external aim | `sentry_msgs/msg/AimTarget` | BT 当前选择的装甲板目标。 |
| `/ly/aim/result` | external aim -> BT | `sentry_msgs/msg/AimResult` | 外部 aim 的最终角度接管和开火门控：`follow`、`fire`、`yaw`、`pitch`。 |

`sentry_msgs/msg/AimResult` 必须包含 `bool follow`。启动脚本和 selfcheck 都会检查这个字段，避免旧版 `sentry_msgs` 混入正式链路。

## 控制输出

`behavior_tree` 是正式链路唯一控制端。它统一发布：

| Topic | Type | Consumer | 说明 |
|---|---|---|---|
| `/ly/control/angles` | `gimbal_driver/msg/GimbalAngles` | `gimbal_driver` | 云台目标角。 |
| `/ly/control/firecode` | `gimbal_driver/msg/FireCode` | `gimbal_driver` | 开火、电容、follow、aim、rotate 语义字段。 |
| `/ly/control/vel` | `gimbal_driver/msg/ControlVelocity` | `gimbal_driver` | 底盘速度。 |
| `/ly/control/posture` | `gimbal_driver/msg/SentryCmd` | `gimbal_driver` | 姿态专用入口。 |
| `/ly/control/sentry_cmd` | `gimbal_driver/msg/SentryCmd` | `gimbal_driver` | 完整哨兵裁判命令入口。 |

调试脚本如 `mapper_node.py`、`fire_flip_test.py`、FaceMode 直接控制模式也可能发布 `/ly/control/*`。这些属于调试控制源，不应和正式 `behavior_tree` 控制链并行抢控制。

## 导航与 FaceMode

BT 不直接面向外部导航发布最终 `/goal_pose`，通常由 `navi_tf_bridge` 完成转换：

```text
/ly/navi/goal_pos_raw  -> navi_tf_bridge -> /goal_pose
/ly/navi/target_rel    -> navi_tf_bridge -> /goal_pose
/ly/face_mode/target_raw -> map_aim_point_node -> /ly/face_mode/angles -> behavior_tree -> /ly/control/angles
```

FaceMode 在正式链路中默认由 `map_aim_point_node` 输出 `/ly/face_mode/angles`，再由 BT 统一决定是否接管到 `/ly/control/angles`。FaceMode 本身不清零底盘小陀螺 `Rotate`，需要停小陀螺时由 BT 的 FollowMode/导航兼容逻辑处理。

## Legacy Internal Auto-Aim

以下链路仍保留作调试、标定和历史对照，但不是当前正式主链：

```text
gimbal_driver/camera
  -> detector
  -> /ly/detector/armors
  -> tracker_solver
  -> /ly/tracker/results
  -> predictor
  -> /ly/predictor/target
  -> behavior_tree 或 mapper_node.py
  -> /ly/control/angles + /ly/control/firecode
  -> gimbal_driver
```

相关入口：

```bash
ros2 launch detector auto_aim.launch.py
python3 src/detector/script/mapper_node.py --target-id 6 --enable-fire true --auto-fire true
python3 src/detector/script/fire_flip_test.py --fire-hz 8.0
```

这些入口适合验证 legacy detector/tracker/predictor、射表、火控翻转等局部链路。它们不代表正式比赛启动方式。

## 自检

离车静态检查：

```bash
./scripts/selfcheck.sh sentry --static-only
```

检查当前已运行的 ROS 图：

```bash
./scripts/selfcheck.sh sentry --skip-hz
```

自动启动正式链再检查：

```bash
./scripts/selfcheck.sh sentry --launch --wait 10 --skip-hz
```

如果未启动任何 ROS2 stack，`--skip-hz` 仍会在 runtime graph 阶段报 `No ROS2 nodes found`。这表示没有运行中的节点，不代表静态文件或接口契约失败。

## 文档优先级

当前链路以这些文档为准：

- [2026-05-03_message_and_link_flow.md](2026-05-03_message_and_link_flow.md)
- [../sentry/internal/ros2_topic_structure.md](../sentry/internal/ros2_topic_structure.md)
- [../modules/2026-05-05_behavior_tree.md](../modules/2026-05-05_behavior_tree.md)
- [../modules/2026-05-05_gimbal_driver.md](../modules/2026-05-05_gimbal_driver.md)
- [../modules/2026-05-04_navi_tf_bridge.md](../modules/2026-05-04_navi_tf_bridge.md)

历史方案和旧行为说明放在 `docs/record/` 和 `docs/plans/`，阅读时只作为背景，不作为当前正式链路依据。
