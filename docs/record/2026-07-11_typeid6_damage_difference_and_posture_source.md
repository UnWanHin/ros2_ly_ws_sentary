# TypeID=6 damage_difference / posture source adjustment

Updated: 2026-07-11

## 1. 變更結果

`gimbal_driver` 的 `TypeID=6 ChassisData` 已從姿態兼容回讀，改為承載裁判 `0x0003 game_robot_HP_t` 的 `damage_difference`。

当前有效语义：

- `TypeID=6 byte 0~1`：`UWBAngleYaw`
- `TypeID=6 byte 2~3`：`DamageDifference`
- `TypeID=6 byte 4~11`：`ChassisPacked1/2`

## 2. 裁判來源

`DamageDifference` 对应 RoboMaster 2026 通信协议 V2.0.0 的 `0x0003 game_robot_HP_t`：

- offset `8`
- size `2`
- type `int16_t`
- 语义：`己方全队总伤害 - 对方全队总伤害`

## 3. ROS topic

新增：

- `/ly/game/damage_difference`
- message type: `std_msgs/msg/Int16`

## 4. 姿态回读

`/ly/gimbal/posture` 现在只由 `TypeID=7 SentryData.SentryInfo2.posture` 发布。

- 有效值：`1/2/3`
- `0` 不主动清掉当前姿态
- `TypeID=6` 不再参与姿态回读

## 5. 受影响文件

- `src/gimbal_driver/module/BasicTypes.hpp`
- `src/gimbal_driver/main.cpp`
- `src/gimbal_driver/msg/Chassis.msg`
- `docs/sentry/embedded/serial_data_mapping.md`
- `docs/sentry/embedded/downlink_control_frame.md`
- `docs/sentry/embedded/referee_serial_integration.md`
- `docs/modules/2026-05-05_gimbal_driver.md`
- `docs/sentry/internal/ros2_topic_structure.md`
- `docs/sentry/internal/ros2_topic_tree.md`
- `.understand-anything/knowledge-graph.json`

