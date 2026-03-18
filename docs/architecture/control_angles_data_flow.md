# `/ly/control/angles` 数据流追踪

本文档只追一条链路：`/ly/control/angles` 是怎么从“辅瞄锁敌”一路形成并最终下发到下位机的。

适用代码状态：当前仓库主线实现（2026-03-17）。

## 1. 当前在线主链路

当前实际在线的主链路是：

```text
下位机回传云台角
  -> /ly/gimbal/angles
  -> detector
  -> /ly/detector/armors
  -> tracker_solver
  -> /ly/tracker/results
  -> predictor
  -> /ly/predictor/target
  -> behavior_tree
  -> /ly/control/angles
  -> gimbal_driver
  -> 串口下发下位机
```

结论：

- `predictor` 不直接发 `/ly/control/angles`
- 当前比赛链路由 `behavior_tree` 统一收口后再发 `/ly/control/angles`
- `mapper_node.py` 属于独立测试桥，不是默认比赛控制链路

## 2. 一图看懂

```text
serial(lower) -> gimbal_driver -> /ly/gimbal/angles
                                 |
                                 v
camera -> detector -> /ly/detector/armors -> tracker_solver -> /ly/tracker/results
                                                                       |
                                                                       v
behavior_tree -> /ly/bt/target -------------------------------> predictor
                                                                       |
                                                                       v
                                                        /ly/predictor/target
                                                                       |
                                                                       v
                                                           behavior_tree
                                                                       |
                                                                       v
                                                           /ly/control/angles
                                                                       |
                                                                       v
                                                            gimbal_driver -> serial(lower)
```

## 3. 每一跳的职责

### 3.1 `gimbal_driver -> /ly/gimbal/angles`

下位机回传当前云台角，`gimbal_driver` 发布到 `/ly/gimbal/angles`。

关键代码：

- [`src/gimbal_driver/main.cpp`](../../src/gimbal_driver/main.cpp)
  - `PubGimbalData()` 发布 `/ly/gimbal/angles`

关键位置：

```cpp
msg.yaw = static_cast<float>(data.GimbalAngles.Yaw);
msg.pitch = static_cast<float>(data.GimbalAngles.Pitch);
Node.Publisher<ly_gimbal_angles>()->publish(msg);
```

作用：

- 给 `detector` 绑定“这帧图像对应的云台实际角度”
- 给 `behavior_tree` 提供当前云台反馈

## 3.2 `detector`

`detector` 同时吃三类输入：

- `/ly/gimbal/angles`
- `/ly/bt/target`
- 图像输入

其中 `/ly/gimbal/angles` 会先写入原子变量：

- [`src/detector/detector_node.cpp`](../../src/detector/detector_node.cpp)
  - `gimbal_callback()`

随后每帧检测时，把当前云台角写进 `Armors.msg`：

- [`src/auto_aim_common/msg/Armors.msg`](../../src/auto_aim_common/msg/Armors.msg)
  - `yaw`
  - `pitch`

关键流程：

1. 从图像中检出全部装甲板/车辆
2. 根据 `/ly/bt/target` 做类型过滤
3. 对过滤后的装甲板做 `ReFindAndSolveAll(...)`
4. 用 PnP 解算装甲板位姿
5. 发布 `/ly/detector/armors`

关键代码：

- [`src/detector/detector_node.cpp`](../../src/detector/detector_node.cpp)
  - `gimbal_callback()`
  - `get_target_callback()`
  - `ImageLoop()`

这里最重要的不是只“识别到了谁”，而是把“这帧图像对应的当前云台角”一起带下去，减少链路延迟错位。

## 3.3 `/ly/detector/armors -> tracker_solver`

`tracker_solver` 订阅 `/ly/detector/armors`，做两件事：

1. 目标关联与跟踪
2. 统一时间源下的坐标解算

输出消息是 [`src/auto_aim_common/msg/Trackers.msg`](../../src/auto_aim_common/msg/Trackers.msg)：

- `armor_trackers[]`
- `car_trackers[]`
- `yaw`
- `pitch`

注意：`Trackers.msg` 里的 `yaw/pitch` 直接沿用了 `Armors.msg` 里的当前云台角。

关键代码：

- [`src/tracker_solver/car_tracker_solver_node.cpp`](../../src/tracker_solver/car_tracker_solver_node.cpp)
  - `detection_callback()`
  - `solver->solve_all(track_results, gimbal_angle)`

## 3.4 `/ly/tracker/results -> predictor`

`predictor` 订阅：

- `/ly/tracker/results`
- `/ly/bt/target`
- `/ly/bullet/speed`

输出：

- `/ly/predictor/target`

消息类型：

- [`src/auto_aim_common/msg/Target.msg`](../../src/auto_aim_common/msg/Target.msg)
  - `status`
  - `yaw`
  - `pitch`

关键流程：

1. `/ly/tracker/results` 回调先用当前帧观测更新状态
2. `predictor_node` 以固定 `10ms` timer 调 `controller->control(...)`
3. `controller` 内部调用 `predictFunc(now + flyTime + shootDelay)` 取未来时刻目标
4. `calcPitchYawWithShootTable(...)` 计算最终期望 yaw/pitch
5. `Target.msg` 持续发布：
   - 有预测时发布角度
   - 完全失锁时发布 `status=false` 且 `yaw/pitch=NaN`

关键点：

- `Target.msg.yaw/pitch` 不是像素误差，也不是相对增量
- 这里发出的已经是“期望云台角”

关键代码：

- [`src/predictor/predictor_node.cpp`](../../src/predictor/predictor_node.cpp)
- [`src/predictor/src/controller.cpp`](../../src/predictor/src/controller.cpp)

## 3.5 `/ly/predictor/target -> behavior_tree`

当前主链路里，`behavior_tree` 是 `/ly/control/angles` 的唯一默认发布者。

它会订阅 `/ly/predictor/target`，当前语义是：

- 消息一来就把目标置为“可锁”
- 若 `yaw/pitch` 有限值，则刷新 `autoAimData.Angles`
- `status` 只继续作为 fire 许可写入 `FireStatus`

然后在 `PublishTogether()` 里：

1. 判断当前 `aimMode`
2. 若当前 `AimData.Valid=true`，则直接 `nextAngles = activeAimData->Angles`
3. 若暂时没目标，但已有 latched angle，则沿用最近一次目标角
4. 若超过扫描等待时间仍无目标，则进入扫描逻辑
5. 仅在从未锁到过角时，才回退到当前云台角
5. 最终写入 `gimbalControlData.GimbalAngles`

最后由 `PubGimbalControlData()` 发布 `/ly/control/angles`。

关键代码：

- [`src/behavior_tree/src/SubscribeMessage.cpp`](../../src/behavior_tree/src/SubscribeMessage.cpp)
- [`src/behavior_tree/src/GameLoop.cpp`](../../src/behavior_tree/src/GameLoop.cpp)
- [`src/behavior_tree/src/PublishMessage.cpp`](../../src/behavior_tree/src/PublishMessage.cpp)
- [`src/behavior_tree/src/Application.cpp`](../../src/behavior_tree/src/Application.cpp)

## 3.6 `/ly/control/angles -> gimbal_driver`

`gimbal_driver` 订阅 `/ly/control/angles` 后，直接把：

- `msg.yaw`
- `msg.pitch`

写入 `GimbalControlData.GimbalAngles`，随后立刻 `Device.Write(...)` 下发。

关键代码：

- [`src/gimbal_driver/main.cpp`](../../src/gimbal_driver/main.cpp)
  - `GenSub<ly_control_angles>(...)`
  - `CallbackGenerator -> Device.Write(...)`

这里没有额外再做角度解算；`gimbal_driver` 在这条链上是“控制口网关”，不是“二次求解器”。

## 4. 关键消息的语义

### 4.1 `Armors.msg`

来源：

- `detector`

关键字段：

- `armors[]`: 检测结果
- `cars[]`: 车辆框
- `yaw/pitch`: 当前图像绑定的云台角

用途：

- 给 `tracker_solver` 提供检测结果和时刻对应的云台姿态

### 4.2 `Trackers.msg`

来源：

- `tracker_solver`

关键字段：

- `armor_trackers[]`: 跟踪后的装甲板三维信息
- `car_trackers[]`: 跟踪后的车辆框
- `yaw/pitch`: 本帧对应云台角

用途：

- 给 `predictor` 做目标预测和弹道解算

### 4.3 `Target.msg`

来源：

- `predictor`

关键字段：

- `status`: 当前控制是否有效
- `yaw/pitch`: 期望云台角

用途：

- 给 `behavior_tree` 做最终锁角/火控整合

## 5. 现在与历史链路的区别

仓库里同时存在两种说法：

### 5.1 当前比赛链路

```text
predictor -> behavior_tree -> /ly/control/angles -> gimbal_driver
```

这条是当前主线代码实际跑的链路。

### 5.2 历史/测试链路

```text
predictor -> mapper_node.py -> /ly/control/angles -> gimbal_driver
```

这条只用于快速联调或独立测试，不是默认比赛控制链。

如果你看到旧文档写“`predictor` 直接到 `gimbal_driver`”或“`mapper_node` 转发为默认链路”，要以当前代码为准。

## 6. 出问题时该看哪一跳

### 6.1 `detector` 没锁到目标

先看：

```bash
ros2 topic echo /ly/gimbal/angles
ros2 topic echo /ly/detector/armors
ros2 topic echo /ly/bt/target
```

重点检查：

- `/ly/gimbal/angles` 是否正常更新
- `/ly/bt/target` 是否把目标类型限制错了
- `/ly/detector/armors` 是否已经空了

### 6.2 `predictor` 有结果，但 `/ly/control/angles` 没数据

先看：

```bash
ros2 topic echo /ly/predictor/target
ros2 topic echo /ly/control/angles
ros2 topic info /ly/control/angles -v
```

重点检查：

- `behavior_tree` 是否启动
- 比赛开始门控是否卡住
- `Target.msg.status` 是否一直为 `false`

### 6.3 `/ly/control/angles` 有数据，但下位机不动

先看：

```bash
ros2 topic echo /ly/control/angles
ros2 topic info /ly/control/angles -v
ros2 topic echo /ly/gimbal/angles
```

重点检查：

- `gimbal_driver` 是否在线并订阅 `/ly/control/angles`
- 串口是否正常
- 下位机是否回传新的 `/ly/gimbal/angles`

## 7. 最小追踪命令

建议按这个顺序开终端看：

```bash
ros2 topic echo /ly/gimbal/angles
ros2 topic echo /ly/detector/armors
ros2 topic echo /ly/tracker/results
ros2 topic echo /ly/predictor/target
ros2 topic echo /ly/control/angles
```

再用下面命令确认发布/订阅方向：

```bash
ros2 topic info /ly/bt/target -v
ros2 topic info /ly/predictor/target -v
ros2 topic info /ly/control/angles -v
```

## 8. 对应代码入口速查

- `gimbal_driver` 上行角度发布  
  [`src/gimbal_driver/main.cpp`](../../src/gimbal_driver/main.cpp)
- `detector` 读取云台角并发布 `Armors.msg`  
  [`src/detector/detector_node.cpp`](../../src/detector/detector_node.cpp)
- `tracker_solver` 生成 `Trackers.msg`  
  [`src/tracker_solver/car_tracker_solver_node.cpp`](../../src/tracker_solver/car_tracker_solver_node.cpp)
- `predictor` 生成 `Target.msg`  
  [`src/predictor/predictor_node.cpp`](../../src/predictor/predictor_node.cpp)
- `controller` 计算期望 yaw/pitch  
  [`src/predictor/src/controller.cpp`](../../src/predictor/src/controller.cpp)
- `behavior_tree` 接收 `Target.msg` 并发布 `/ly/control/angles`  
  [`src/behavior_tree/src/SubscribeMessage.cpp`](../../src/behavior_tree/src/SubscribeMessage.cpp)  
  [`src/behavior_tree/src/GameLoop.cpp`](../../src/behavior_tree/src/GameLoop.cpp)  
  [`src/behavior_tree/src/PublishMessage.cpp`](../../src/behavior_tree/src/PublishMessage.cpp)
