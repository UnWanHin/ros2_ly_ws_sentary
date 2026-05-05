# behavior_tree pinned BT 与老火控逻辑记录（2026-03-18）

## 背景

本次有两条直接影响联调的问题：

1. `behavior_tree_node` 离线启动在 `rclcpp::Node` 构造阶段段错误。
2. `behavior_tree` 已恢复老链路“回调即锁 / 回调即按节拍开火”，但新版 `predictor` 仍会向 BT 发布无效 target（`status=false` 或非有限角），导致两边语义不一致。

## 根因/判断

### 1. BT 库问题，不是 config 问题

已确认：

- 不是 `config.json` / `regional_competition.json` 的内容问题。
- 崩溃发生在 `behavior_tree_node` 创建 `rclcpp::Node` 时，甚至早于 `ConfigurationInit()`。
- 最小化诊断表明：
  - 纯 `rclcpp::Node` 程序正常
  - 只要链接系统 `/usr/local/lib/libbehaviortree_cpp.so`，再创建 `rclcpp::Node` 就会崩
  - 工作区 `third_party/behaviortree_cpp_v4/install/lib/libbehaviortree_cpp.so` 则可正常与 ROS 2 `Node` 共存

因此当前结论是：

- 系统 BT 4.8.0 库有问题
- 工作区 pinned BT 可用

### 2. predictor / BT 对接语义不一致

当前目标是：

- `behavior_tree` 按 `ly-ros-main` 老链路工作
- 一旦 `predictor` 回调到达，BT 侧就只管锁敌和翻 firecode

那么 `predictor` 就不应再把无效 target 发给 BT。

## 改动内容

### A. behavior_tree 默认改为 pinned BT

修改文件：

- `src/behavior_tree/CMakeLists.txt`

改动：

- 新增 `BTCPP_FORCE_PINNED`
- 默认值改为 `ON`
- 默认使用：
  - `third_party/behaviortree_cpp_v4/install`
- pinned config 没有版本字符串时，允许继续使用已验证可运行的本地库

结果：

- 普通 `colcon build --packages-select behavior_tree`
  现在就会默认走工作区 pinned BT
- 不再优先吃坏掉的 `/usr/local/lib/libbehaviortree_cpp.so`

### B. predictor 只向 BT 发布有效 target

修改文件：

- `src/predictor/predictor_node.cpp`

改动：

- 保留新版 timer / coast / invalid-reason 诊断设计
- 但 `/ly/predictor/target` 改为：
  - 仅在 `status=true`
  - 且 `yaw/pitch` 为有限值
  时才发布给 BT

结果：

- BT 不再收到 `status=false + NaN`
- 与当前“老链路 BT”兼容

### C. behavior_tree 恢复老链路 fire / lock 语义

修改文件：

- `src/behavior_tree/src/SubscribeMessage.cpp`
- `src/behavior_tree/src/GameLoop.cpp`

改动：

- `autoaim/outpost` 回调一到就视为可锁、可参与开火节拍
- 非 `buff` 模式按老逻辑翻 `FireCode`
- 丢目标后保角 `2000ms`，超时再扫描

## 当前关键参数/约束

### 1. BT 依赖选择

文件：

- `src/behavior_tree/CMakeLists.txt`

参数：

- `BTCPP_FORCE_PINNED`

默认：

- `ON`

含义：

- `ON`：默认使用工作区可运行的 pinned BT
- `OFF`：恢复“系统优先，找不到再 fallback”的策略

建议：

- 当前环境保持 `ON`
- 只有确认系统 BT 已修好时，再切回 `OFF`

### 2. 自瞄/前哨链 fire 语义

当前：

- `behavior_tree` 不再依赖 `predictor.status` 决定 `autoaim/outpost` 是否翻 firecode
- `predictor` 的责任改为“只在有效时给 BT 发 target”

### 3. buff 语义

当前：

- `buff` 模式仍保留 `msg.status -> FireStatus` 的旧含义

## 微调建议

1. 先不要回到系统 BT。
2. 先用现在这套 pinned BT 完成离线/在线联调。
3. 以后如果要回系统优先，先重新验证最小程序：
   - 纯 `rclcpp::Node`
   - 链接 `behaviortree_cpp` 的 `rclcpp::Node`

## 验证方法

### 1. 验证 BT 选择

```bash
cd ~/ros2_ly_ws_sentary
source /opt/ros/humble/setup.bash
colcon build --packages-select behavior_tree
```

看构建输出是否出现：

- `BehaviorTree.CPP: force pinned workspace copy`
- `Pinned BehaviorTree.CPP root: .../third_party/behaviortree_cpp_v4/install`

### 2. 验证 behavior_tree 离线启动

```bash
source install/setup.bash
timeout 2s install/behavior_tree/lib/behavior_tree/behavior_tree_node --ros-args -p debug_bypass_is_start:=true
```

当前期望：

- 不再段错误
- 能打印：
  - `Logger Init Success!`
  - `Loading BT from ...`
  - `Open config.json ...`
  - `Waiting Before Game`

### 3. 验证 predictor -> BT 对接

观察：

```bash
ros2 topic echo /ly/predictor/target
ros2 topic echo /ly/control/angles
ros2 topic echo /ly/control/firecode
```

当前期望：

- `predictor` 只在有效 target 时才对 BT 发消息
- BT 收到消息后按老逻辑锁敌与翻 firecode
