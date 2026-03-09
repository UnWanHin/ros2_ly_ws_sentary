# 子模块单独测试指南（ROS 2）

这份指南用于**不启动整套哨兵系统**时，单独拉起某个模块做联调。

## 通用准备

```bash
cd ~/ros2_ly_ws_sentary
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
```

可先做 launch 语法检查：

```bash
export ROS_LOG_DIR=/tmp/ros2_logs
mkdir -p ${ROS_LOG_DIR}
ros2 launch <pkg> <launch_file> --show-args
```

## 1) gimbal_driver 单测

推荐入口（Python launch）：

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py
```

虚拟串口测试：

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py use_virtual_device:=true
```

兼容入口（XML）：

```bash
ros2 launch gimbal_driver gimbal_driver.launch
```

## 2) detector 链路单测

自瞄链路：

```bash
ros2 launch detector auto_aim.launch.py
```

查看可选参数（含 mapper 与隔离话题）：

```bash
ros2 launch detector auto_aim.launch.py --show-args
```

可选：同一条 launch 内启动 mapper（默认隔离到 debug 控制话题，不直接驱动云台）：

```bash
ros2 launch detector auto_aim.launch.py use_mapper:=true
```

若要让 mapper 直接驱动云台（仅限单控制源联调）：

```bash
ros2 launch detector auto_aim.launch.py \
  use_mapper:=true \
  mapper_angles_topic:=/ly/control/angles \
  mapper_firecode_topic:=/ly/control/firecode
```

打符链路：

```bash
ros2 launch detector buff.launch.py
```

前哨链路：

```bash
ros2 launch detector outpost.launch.py
```

保留旧入口（仅调试对照）：

```bash
ros2 launch detector old_auto_aim.launch
```

## 3) behavior_tree 单测

```bash
ros2 launch behavior_tree behavior_tree.launch.py
```

## 4) shooting_table_calib 单测

推荐入口（Python launch）：

```bash
ros2 launch shooting_table_calib shooting_table_calib.launch.py
```

兼容入口（XML）：

```bash
ros2 launch shooting_table_calib shooting_table_calib.launch
```

## 5) detector 脚本单测（火控/映射）

火控翻转压测（持续翻转 `/ly/control/firecode`）：

```bash
python3 src/detector/script/fire_flip_test.py --fire-hz 8.0
```

可选：脚本内同时拉起 detector 节点：

```bash
python3 src/detector/script/fire_flip_test.py --start-detector true --params-file src/detector/config/auto_aim_config.yaml
```

目标映射工具（`/ly/predictor/target` -> `/ly/control/angles`）：

```bash
python3 src/detector/script/mapper_node.py --target-id 6 --enable-fire true --auto-fire true
```

建议联调时同时观察控制话题发布者，避免多写冲突导致云台抽动：

```bash
ros2 topic info /ly/control/angles -v
ros2 topic info /ly/control/firecode -v
```

若发布者超过 1 个（例如 `behavior_tree` 与 `mapper_node` 同时在写），先停掉其中一个控制源。

## 备注

- 日常整链路仍建议使用：
  `ros2 launch behavior_tree sentry_all.launch.py`
- 单测时若出现参数不生效，优先检查是否正确加载 `detector/config/auto_aim_config.yaml`。
- 若出现“第一次正常、第二次卡住”，先检查是否有残留控制源：
  - `ros2 topic info /ly/control/angles -v`
  - `ros2 topic info /ly/control/firecode -v`
  - `pgrep -af "gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|behavior_tree_node|mapper_node|fire_flip_test"`
- 统一自检入口（推荐）：
  - 离车：`./scripts/self_check_pc.sh`
  - 上车：`./scripts/self_check_robot.sh`
