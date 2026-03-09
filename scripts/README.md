# Scripts Usage Guide

本目录放运行与自检脚本。下面按“用途 -> 命令 -> 结果判定”给出最常用入口。

## 1. `self_check_pc.sh`（离车自检）

用途：
- 开发机执行，验证构建、静态配置契约、launch 语法。
- 不依赖车端硬件链路。

常用命令：

```bash
# 标准离车自检（会先构建）
./scripts/self_check_pc.sh

# 仅静态检查，不重建
./scripts/self_check_pc.sh --no-build

# 指定包并执行 colcon test
./scripts/self_check_pc.sh --packages "behavior_tree outpost_hitter predictor" --test
```

## 2. `self_check_robot.sh`（上车自检）

用途：
- 车载机执行，检查串口候选设备、权限、网络可达，以及运行时 ROS2 图契约。
- 会调用 `self_check_sentry.sh --runtime-only --launch` 自动拉起整链路。
- 默认只跑运行态（`--runtime-only`），避免重复静态检查。

常用命令：

```bash
# 快速上车自检（默认跳过 hz）
./scripts/self_check_robot.sh

# 严格上车自检（含 hz 采样）
./scripts/self_check_robot.sh --with-hz

# 传递 launch 参数
./scripts/self_check_robot.sh -- --config_file:=/abs/path/auto_aim_config.yaml
```

## 3. `self_check_sentry.sh`（基础自检套件）

用途：
- 核心检查脚本，支持静态/运行时分模式，PC 与 Robot 脚本均复用它。

常用命令：

```bash
# 仅静态（文件、配置、BT XML）
./scripts/self_check_sentry.sh --static-only

# 离线模式运行时链路（自动传 offline:=true）
./scripts/self_check_sentry.sh --runtime-only --launch --offline --wait 12 --skip-hz

# 仅运行时链路（你提到的命令）
./scripts/self_check_sentry.sh --runtime-only --launch --wait 12 --skip-hz

# 运行时链路 + 频率
./scripts/self_check_sentry.sh --runtime-only --launch --wait 12
```

## 4. `start_sentry_all.sh`（整链路启动）

用途：
- 直接启动 `sentry_all.launch.py`，常用于手工调试或配合其他脚本。

```bash
./scripts/start_sentry_all.sh

# 离线模式（强制虚拟串口 + 视频回放参数覆盖）
./scripts/start_sentry_all.sh --offline
```

二次启动卡住时，优先用清理模式（默认已开启）：

```bash
./scripts/start_sentry_all.sh --cleanup-existing
```

脚本会把 `ROS_LOG_DIR` 默认指向 `/tmp/ros2_logs`，避免 `~/.ros/log` 权限异常导致 launch 直接失败。

若你明确要保留当前正在跑的进程，可禁用清理并在冲突时失败退出：

```bash
./scripts/start_sentry_all.sh --no-cleanup-existing
```

## 5. `start_autoaim_debug.sh`（auto_aim + mapper 联调）

用途：
- 面向“感知链/火控链”拆分联调，不拉起 behavior_tree。
- 支持三种模式：`perception`（仅感知）、`mapper`（仅 mapper）、`fire`（感知+mapper，默认）。
- 默认 `--offline`，并带残留进程清理、单控制源保护（防止与 behavior_tree 同时写 control 话题）。

常用命令：

```bash
# 默认：感知链 + mapper（火控联调）
./scripts/start_autoaim_debug.sh

# 仅感知链（不控火）
./scripts/start_autoaim_debug.sh --mode perception

# 仅 mapper（要求感知链已启动）
./scripts/start_autoaim_debug.sh --mode mapper

# 指定 mapper 目标优先级与回退目标
./scripts/start_autoaim_debug.sh --target-priority "6,3,4,5" --target-id 6

# 在线模式（不传 offline:=true）
./scripts/start_autoaim_debug.sh --online
```

## 6. `feature_test/run_feature_test.sh`（BT 外功能测试框架，单控制源）

用途：
- 使用单个配置文件启动“云台/底盘”功能测试，运行在 `behavior_tree` 外。
- 默认启用“单控制源保护”：检测到 `/behavior_tree` 或控制话题多发布者时拒绝接管。

默认配置文件：
- `scripts/config/sentry_feature_test.yaml`

常用命令：

```bash
# 按配置运行（默认是 gimbal armor）
./scripts/feature_test/run_feature_test.sh

# 只预览命令，不执行
./scripts/feature_test/run_feature_test.sh --dry-run

# 指定配置文件
./scripts/feature_test/run_feature_test.sh --config ./scripts/config/sentry_feature_test.yaml
```

当前 Phase 1 支持：
- 云台：`armor`、`scan`
- 底盘：`velocity`

## 结果判定

- `FAIL: 0`：通过。
- `WARN > 0`：可继续，但需确认是否为当前场景预期（如未接网线）。
- 若使用 `--launch` 且失败，脚本会自动输出 `Launch Diagnosis`，包含崩溃签名与最近日志尾部。
- 运行前会提示当前配置是否要求“真实相机/真实串口”，避免离车时误用比赛参数。
- 使用 `--offline` 时，会自动传 `offline:=true` 给 launch，避免每次手改 YAML。
- 运行时链路检查已覆盖 posture 合约：`/ly/control/posture` 与 `/ly/gimbal/posture`。
- 若 `ros2 run` 直接报 `Segmentation fault` 且日志里有 `Failed opening file ~/.ros/log/... Permission denied`，先修复日志目录权限或临时设置 `export ROS_LOG_DIR=/tmp/ros2_logs` 再复测。

## 比赛/调试开关位置

统一在：

- `src/detector/config/auto_aim_config.yaml`

重点键位（已在 YAML 内写明“比赛建议/调试建议”注释）：

- `detector_config/use_video`
- `detector_config/debug_mode`
- `detector_config/web_show`
- `detector_config/show`
- `detector_config/draw`
- `detector_config/use_ros_bag`
- `detector_config/save_video`
- `io_config/use_virtual_device`
