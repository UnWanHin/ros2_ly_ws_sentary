# `/ly/control/firecode` 火控链路

本文档只追一条链路：

- `/ly/predictor/target.status`
- 如何进入 `behavior_tree`
- 如何变成 `/ly/control/firecode`
- 最终如何写进下位机主控制帧

适用代码状态：当前仓库主线实现（2026-03-18）。

## 1. 当前主链路

```text
/ly/tracker/results
  -> predictor
  -> /ly/predictor/target.status
  -> behavior_tree
  -> /ly/control/firecode
  -> gimbal_driver
  -> 串口下发下位机
```

结论先说：

- `behavior_tree` 不自己做目标识别，它只消费上游给出的 `status`
- `gimbal_driver` 不自己判“该不该打”，它只转发火控字节
- 当前“能不能打”主要由 `predictor/controller` 的 `ControlResult.valid` 决定

## 2. predictor：谁决定 `status`

### 2.1 当前结构

`predictor_node` 现在分成两段：

1. `predictor_callback()`
   - 收 `/ly/tracker/results`
   - 只更新内部模型
2. `publish_timer_callback()`
   - 固定 `10ms` 定时
   - 调 `controller->control(...)`
   - 发布 `/ly/predictor/target`

关键文件：

- [predictor_node.cpp](/home/unwanhin/ros2_ly_ws_sentary/src/predictor/predictor_node.cpp)

### 2.2 `Target.status` 的生成

当前代码里：

- 有预测且可计算控制角时：
  - `target_msg.status = control_result.valid`
- 没预测且观测也过期时：
  - `target_msg.status = false`
  - `target_msg.yaw = NaN`
  - `target_msg.pitch = NaN`

所以：

- `status=true`：当前控制器允许开火
- `status=false`：当前控制器不允许开火，或当前根本没有可用预测

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

- `autoAimData.FireStatus = msg->status`
- 消息一来就把目标置为“可锁”
- 若 `yaw/pitch` 有限值，则刷新 `autoAimData.Angles`

所以这里的语义是：

- `Angles`：跟随角
- `FireStatus`：开火许可

### 3.2 `PublishTogether()` 中的火控逻辑

在 [GameLoop.cpp](/home/unwanhin/ros2_ly_ws_sentary/src/behavior_tree/src/GameLoop.cpp)：

- `allow_fire = activeAimData->FireStatus`
- 若 `FireRequireTargetStatus=false`，则允许忽略 `FireStatus`
- 若 `StopFire=true`，则无论如何不翻火控

普通自瞄/前哨模式：

- 只有 `allow_fire=true`
- 且 `fireRateClock.trigger()`
- 才会执行：
  - `RecFireCode.FlipFireStatus()`
  - `gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus`

打符模式：

- 只在 `buffAimData.FireStatus=true` 时翻转

### 3.3 当前 competition profile 的门控状态

当前仓库里：

- `regional_competition.json`
- `league_competition.json`

都把：

- `AimDebug.FireRequireTargetStatus = false`

也就是说：

- 当前这两个 competition profile 下
- `behavior_tree` 不再强依赖 `Target.status=true` 才允许翻 fire code

## 4. `/ly/control/firecode` 如何下发

### 4.1 behavior_tree 发布

在 [PublishMessage.cpp](/home/unwanhin/ros2_ly_ws_sentary/src/behavior_tree/src/PublishMessage.cpp)：

- `PubGimbalControlData()` 会同时发布：
  - `/ly/control/angles`
  - `/ly/control/firecode`

其中 `/ly/control/firecode` 是：

```cpp
msg.data = *reinterpret_cast<std::uint8_t *>(&gimbalControlData.FireCode);
```

### 4.2 gimbal_driver 订阅并写入主控制帧

在 [gimbal_driver/main.cpp](/home/unwanhin/ros2_ly_ws_sentary/src/gimbal_driver/main.cpp)：

- 订阅 `/ly/control/firecode`
- 直接把 `UInt8.data` 写进 `GimbalControlData.FireCode`
- 然后通过统一的 `CallbackGenerator -> Device.Write(...)` 写串口

所以：

- `gimbal_driver` 不做二次 fire 判定
- 它只是把上位机的 firecode 原样塞进下位机主控制帧

## 5. 当前代码上的判断

如果现在“没打出来”，从代码链路上更可能是：

1. predictor 没生成有效 target
2. predictor 生成了 target，但 `status=false`
3. `behavior_tree` 进入了 `StopFire` 或模式不对
4. 下位机对 `FireCode` 的位语义和上位机预期不一致

当前代码本身并没有删除或断开火控链。

## 6. 最推荐的排查顺序

建议按这个顺序看：

```bash
ros2 topic echo /ly/predictor/target
ros2 topic echo /ly/control/firecode
ros2 topic echo /ly/gimbal/firecode
```

判读方法：

1. `/ly/predictor/target.status`
   - 看 predictor 当前是否放行
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
