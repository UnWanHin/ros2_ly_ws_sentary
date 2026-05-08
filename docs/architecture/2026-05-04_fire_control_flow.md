# `/ly/control/firecode` 火控链路

本文档只追一条链路：

- `/ly/predictor/target`
- 如何进入 `behavior_tree`
- 如何变成 `/ly/control/firecode`
- 最终如何写进下位机主控制帧

适用代码状态：当前仓库主线实现（2026-05-03）。

## 1. 当前主链路

```text
/ly/tracker/results
  -> predictor
  -> /ly/predictor/target
  -> behavior_tree
  -> /ly/control/firecode
  -> gimbal_driver
  -> 串口下发下位机
```

结论先说：

- `behavior_tree` 不自己做目标识别，它只消费上游给出的目标角
- `gimbal_driver` 不自己判“该不该打”，它只转发火控字节
- 当前 `autoaim/outpost` 这条链已经恢复为老逻辑：一旦 `predictor/outpost` 回调到达，`behavior_tree` 本轮就按节拍翻 firecode
- `predictor/controller` 仍决定“要不要向 BT 发有效 target”
- `FollowMode=1` 时，`behavior_tree` 会停止 rotate、停止巡逻扫描、停止新的 `FireStatus` 翻转，并保持当前云台角。
- 区域任务启用 FaceMode 时，`behavior_tree` 会停止云台巡逻扫描、使用 `/ly/face_mode/angles` 或保持当前云台角，并按 `FaceMode.SuppressFire` 停止新的 `FireStatus` 翻转；FaceMode 本身不清零 `Rotate`，底盘小陀螺是否停止由 `FollowMode` 决定。

## 2. predictor：谁决定 `status`

### 2.1 当前结构

`predictor_node` 当前默认走回调即发布，保留 timer 模式作为回退：

1. `predictor_callback()`
   - 收 `/ly/tracker/results`
   - 立即调用 `controller->control(...)`
   - 发布 `/ly/predictor/target`
   - 再更新内部模型和 debug/vis 输出，保持接近 ROS1 参考版的响应路径
2. `publish_timer_callback()`（`config/common.yaml` 中 `predictor.publish_on_tracker_callback=false` 时使用）
   - 固定 `10ms` 定时
   - 调 `controller->control(...)`
   - 发布 `/ly/predictor/target`

关键文件：

- [predictor_node.cpp](/home/unwanhin/ros2_ly_ws_sentry/src/predictor/predictor_node.cpp)

### 2.2 `Target` 的生成

当前代码里：

- 有预测且可计算控制角时：
  - `target_msg.status = control_result.valid`
- 没预测且观测也过期时：
  - 不再向 `behavior_tree` 发布无效 target

所以：

- `behavior_tree` 只会收到“当前可用于锁敌的 target”
- `status=false + NaN` 不再进入 BT 主链

### 2.3 当前 `status=false` 的主要原因

控制器当前会把 `valid=false` 的主要原因打成诊断日志：

- `no_prediction`
- `invalid_car`
- `invalid_car_after_flytime`
- `no_armor_fallback_ballistic_fail`
- `invalid_armor`
- `armor_ballistic_fail`

注意：

- 当前版本已经去掉了 `yaw jump reject`
- 当前版本也不再因为 `unstable_track` 直接硬关掉 `valid`

## 3. behavior_tree：谁真正翻转火控

### 3.1 订阅 `/ly/predictor/target`

在 [SubscribeMessage.cpp](/home/unwanhin/ros2_ly_ws_sentary/src/behavior_tree/src/SubscribeMessage.cpp)：

- 消息一来就把目标置为“可锁”
- `autoaim` 路径当前直接视为可开火
- `outpost` 路径当前同样直接视为可开火
- `buff` 仍保留 `msg->status` 的开火语义

所以这里当前的语义是：

- `Angles`：跟随角
- `autoaim/outpost.FireStatus`：老链路兼容值，回调即置真
- `buff.FireStatus`：仍由上游 status 决定

### 3.2 `PublishTogether()` 中的火控逻辑

在 [GameLoop.cpp](/home/unwanhin/ros2_ly_ws_sentary/src/behavior_tree/src/GameLoop.cpp)：

- `allow_fire = activeAimData->FireStatus`
- 若 `FireRequireTargetStatus=false`，则允许忽略 `FireStatus`
- 若 `StopFire=true`，则无论如何不翻火控

普通自瞄/前哨模式：

- 当前已恢复为老代码语义
- 只要这一轮收到了目标回调
- 且 `fireRateClock.trigger()`
- 就会执行：
  - `RecFireCode.FlipFireStatus()`
  - `gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus`

打符模式：

- 只在 `buffAimData.FireStatus=true` 时翻转

### 3.3 当前开火门控现状

- `AimDebug.FireRequireTargetStatus` 仍保留在配置里
- 但对当前 `autoaim/outpost` 主链已经不是决定性门控
- `buff` 模式仍然看 `FireStatus`

## 4. `/ly/control/firecode` 如何下发

### 4.1 behavior_tree 发布

在 [PublishMessage.cpp](/home/unwanhin/ros2_ly_ws_sentary/src/behavior_tree/src/PublishMessage.cpp)：

- `PubGimbalControlData()` 会同时发布：
  - `/ly/control/angles`
  - `/ly/control/firecode`

其中 `/ly/control/firecode` 使用 `gimbal_driver/msg/FireCode` 的语义字段发布完整快照：

```cpp
msg.field_mask = gimbal_driver::msg::FireCode::FIELD_ALL;
msg.fire_status = firecode.FireStatus;
msg.cap_state = firecode.CapState;
msg.follow_mode = firecode.FollowMode != 0;
msg.aim_mode = firecode.AimMode != 0;
msg.rotate = firecode.Rotate;
msg.raw = *reinterpret_cast<const std::uint8_t*>(&firecode);
```

### 4.2 gimbal_driver 订阅并写入主控制帧

在 [gimbal_driver/main.cpp](/home/unwanhin/ros2_ly_ws_sentary/src/gimbal_driver/main.cpp)：

- 订阅 `/ly/control/firecode`
- 按 `field_mask` 更新 `FireStatus/CapState/FollowMode/AimMode/Rotate`
- `field_mask=FIELD_ALL` 或 `0` 表示完整快照；部分字段更新会受 `firecode_partial_hold_ms` 过期保护
- 然后通过统一的 `CallbackGenerator -> Device.Write(...)` 写串口

所以：

- `gimbal_driver` 不做二次 fire 判定
- 它只是把上位机的 firecode 语义字段同步到下位机主控制帧

## 5. 当前代码上的判断

如果现在“没打出来”，从代码链路上更可能是：

1. predictor 没生成有效 target，因此根本没给 BT 回调
2. `behavior_tree` 进入了 `StopFire` 或模式不对
3. 下位机对 `FireCode` 的位语义和上位机预期不一致

当前代码本身并没有删除或断开火控链。

## 6. 最推荐的排查顺序

建议按这个顺序看：

```bash
ros2 topic echo /ly/predictor/target
ros2 topic echo /ly/control/firecode
ros2 topic echo /ly/gimbal/firecode
```

判读方法：

1. `/ly/predictor/target`
   - 看 predictor 当前是否有有效 target 在发
2. `/ly/control/firecode`
   - 看 `behavior_tree` 是否真的翻了火控字节
3. `/ly/gimbal/firecode`
   - 看 `gimbal_driver` / 下位机回传是否一致

如果：

- `/ly/control/firecode` 已经在翻
- 但下位机没反应

问题就更偏：

- 下位机 firecode 语义
- 或下位机实际发射条件

而不是上位机 `behavior_tree`。
