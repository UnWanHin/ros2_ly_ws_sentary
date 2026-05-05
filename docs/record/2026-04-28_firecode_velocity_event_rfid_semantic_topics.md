# FireCode / Velocity / EventData / RFID 语义化记录

日期：2026-04-28

## 1. 协议基准

本次以 `docs/rules/RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0（20260327）.pdf` 为准。

特别注意：`0x0101 event_data` 不使用 `RoboMaster 裁判系统串口协议附录 V1.9.0（20250703）` 的布局。当前仓库保留 RM2026 V1.3.0 的 bit 定义：

- bit0-2：己方补给区状态
- bit3-4：己方小能量机关状态
- bit5-6：己方大能量机关状态
- bit7-8：己方中央高地状态
- bit9-10：己方梯形高地状态
- bit11-19：对方飞镖最后一次命中时间
- bit20-22：对方飞镖最后一次命中目标
- bit23-24：中心增益点状态
- bit25-26：己方堡垒增益点状态
- bit27-28：己方前哨站增益点状态
- bit29：己方基地增益点状态
- bit30-31：保留

## 2. ROS Topic 改动

新增消息：

- `gimbal_driver/msg/FireCode`
- `gimbal_driver/msg/ControlVelocity`
- `gimbal_driver/msg/EventData`
- `gimbal_driver/msg/RfidStatus`

变更 topic 类型：

- `/ly/control/firecode`：`std_msgs/msg/UInt8` -> `gimbal_driver/msg/FireCode`
- `/ly/gimbal/firecode`：`std_msgs/msg/UInt8` -> `gimbal_driver/msg/FireCode`
- `/ly/control/vel`：`gimbal_driver/msg/Vel` -> `gimbal_driver/msg/ControlVelocity`

语义 topic：

- `/ly/game/event_data`：按 V1.3.0 拆解 `0x0101 event_data`
- `/ly/me/rfid`：原 topic 改为 `RfidStatus`，按 V1.3.0 拆解 `0x0209 rfid_status` 低 32 位

保留兼容 topic：

- `/ly/game/all.exteventdata` 仍是原始 `uint32`
- `ly/gimbal/eventdata` 仍是原始 `UInt32`
- `/ly/gimbal/vel` 仍是下位机反馈的物理速度 `Vel`

## 3. FireCode 行为

`FireCode` msg 按下位机 1 字节 bitfield 拆成：

- `fire_status`：bit0-1，开火翻转位，现有链路仍按 `0b00 <-> 0b11`
- `cap_state`：bit2-3
- `follow_mode`：bit4
- `aim_mode`：bit5
- `rotate`：bit6-7

当前 `behavior_tree` 约定：`follow_mode=true` 时仍按 bit4 下发，同时强制 `rotate=0`、`aim_mode=false`，停止新的 `fire_status` 翻转，并保持当前云台角度以停用巡逻扫描。

`field_mask` 用于 partial update：

- `field_mask=0` 或 `FIELD_ALL`：视为完整快照，所有字段都写入
- 非 0：只写 mask 指定字段
- 未更新字段在 `io_config/firecode_partial_hold_ms` 内保留旧值，默认 `100ms`
- 超过保留窗口仍未更新的字段退回 `0`

示例：

```bash
ros2 topic pub /ly/control/firecode gimbal_driver/msg/FireCode "{field_mask: 1, fire_status: 3}" -1
ros2 topic pub /ly/control/firecode gimbal_driver/msg/FireCode "{field_mask: 16, rotate: 1}" -1
```

## 4. Velocity 行为

`/ly/control/vel` 改为 `ControlVelocity`：

- `use_raw=false`：`gimbal_driver` 将 `x_mps/y_mps` 按 `io_config/velocity_raw_to_mps` 编码成下位机 `int8`
- `use_raw=true`：直接写 `raw_x/raw_y`，用于保持旧决策和测试脚本的精确 int8 行为

默认换算参数：

- `io_config/velocity_raw_to_mps = 0.025`
- 即旧 raw `100` 约等于 `2.5m/s`

下位机串口主控制帧不变，仍写入原来的 `Velocity.X/Y` 两个 `int8` 字段。

## 5. EventData / RFID

`/ly/game/event_data` 从 `GameData.ExtEventData` 原值拆字段发布，布局对应 RM2026 V1.3.0 的 `0x0101`。

`/ly/me/rfid` 从 `RFIDAndBuffData.RFIDStatus` 拆字段发布，只覆盖 `0x0209 rfid_status` 的 bit0-31。协议里的 `rfid_status_2` 额外 8 位暂不进入当前 `TypeID=4`，因为当前自定义上行 payload 固定 12B，不能追加第 13 字节。

2026-05-05 追加调整：`RfidStatus` 消息已预留 `rfid_status_2` 字段和 bit0-5 的语义名；发布端在下位机未提供该字节时固定 `has_rfid_status_2=false`，不改变串口 payload。

2026-04-28 追加调整：不再保留 `/ly/me/rfid_status` 这个额外 topic，避免与 `/ly/me/rfid` 混淆；`/ly/me/rfid` 本身就是语义化 `RfidStatus`。

## 6. 链路同步

已同步：

- `gimbal_driver` topic 类型、解析、发布、参数
- `behavior_tree` topic 类型、发布、订阅、runtime safe-control
- `shooting_table_calib` 火控发布
- `buff_shooting_table_calib` 火控回读采样
- `buff_hitter` 火控 topic 类型声明
- detector/feature-test/debug 脚本中的 firecode 和 control velocity 发布
- `scripts/selfcheck/sentry.sh` topic type 契约
- `docs/sentry/2026-05-05_current_upper_lower_data_mapping.md`
- `docs/sentry/2026-05-02_lower_downlink_message_contract.md`
- `docs/modules/2026-05-05_gimbal_driver.md`

## 7. 验证

2026-04-28 追加调整：

- `config/common.yaml` 新增 `firecode_partial_hold_ms` 与 `velocity_raw_to_mps`
- `scripts/launch/start_sentry_all.sh` 会读取这两个值，并传给 `behavior_tree/sentry_all.launch.py`
- `sentry_all.launch.py` 将其作为 typed ROS 参数覆盖给 `gimbal_driver`
- `config/base_config.yaml` 同步保留同名默认值，供直接 launch 或不走 wrapper 的场景使用

已执行：

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select gimbal_driver behavior_tree shooting_table_calib buff_hitter buff_shooting_table_calib
```

结果：

- `gimbal_driver`、`shooting_table_calib`、`buff_hitter`、`buff_shooting_table_calib` 编译通过
- 第一轮 `behavior_tree` 并行编译时 `cc1plus` 被系统 kill，随后单 worker 复跑通过：

```bash
source /opt/ros/humble/setup.bash && MAKEFLAGS=-j1 colcon build --packages-select behavior_tree --parallel-workers 1
```

Python 语法检查通过：

```bash
python3 -m py_compile scripts/feature_test/chassis_spin_test.py scripts/feature_test/chassis_spin_translate_test.py scripts/feature_test/chassis_spin_sine_translate_test.py scripts/feature_test/chassis_vel_test.py scripts/feature_test/scan_gimbal_test.py scripts/feature_test/standalone/tools/target_to_control_bridge.py src/detector/script/buff_test_bridge.py src/detector/script/fire_flip_test.py src/buff_shooting_table_calib/script/buff_shooting_table_calib_node.py
```

尝试执行：

```bash
./scripts/selfcheck.sh sentry --skip-hz
```

结果：未通过。失败点在既有静态缺口 `scripts/launch/start_autoaim_debug.sh` 缺失/不可执行，以及当前未启动 ROS2 graph；本次修改涉及的脚本语法检查项通过。

common.yaml 参数链路补充验证：

```bash
bash -n scripts/launch/start_sentry_all.sh
python3 -m py_compile src/behavior_tree/launch/sentry_all.launch.py
source /opt/ros/humble/setup.bash && MAKEFLAGS=-j1 colcon build --packages-select behavior_tree --parallel-workers 1
export ROS_LOG_DIR=/tmp/ros2_logs
./scripts/launch/start_sentry_all.sh --no-cleanup-existing -- --show-args
```

结果：`--show-args` 中可见 `firecode_partial_hold_ms` 与 `velocity_raw_to_mps`，wrapper 日志显示默认值来自 `config/common.yaml`。
