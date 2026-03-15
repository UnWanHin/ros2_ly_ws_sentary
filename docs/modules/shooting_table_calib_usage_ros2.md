# shooting_table_calib ROS2 使用说明（当前版本）

本文档对应当前工作区脚本与节点实现：
- 启动脚本：`scripts/start_shooting_table_calib.sh`
- 实际入口：`scripts/launch/start_shooting_table_calib.sh`
- 拟合工具：`scripts/tools/fit_shooting_table.py`
- 节点源码：`src/shooting_table_calib/shooting_table_nodev1.cpp`

## 1. 使用前准备

先在工作区根目录执行：

```bash
cd /home/hustrm/ros2_ly_ws_sentary
colcon build --packages-select shooting_table_calib tracker_solver
```

如果之前有旧流程占用相机，先停掉：

```bash
pkill -f "ros2 launch shooting_table_calib"
pkill -f "shooting_table_calib_node"
pkill -f "gimbal_driver_node"
```

## 2. 启动命令（红队/蓝队）

红队（默认推荐）：

```bash
./scripts/start_shooting_table_calib.sh --team red --output screen
```

蓝队：

```bash
./scripts/start_shooting_table_calib.sh --team blue --output screen
```

说明：
- `--output screen` 必须使用，键盘交互才正常。
- 不带 `--team` 时默认等同 `--team red`。
- 直接用默认命令启动时，退出标定后会自动拟合当前这轮 CSV，并同步到比赛配置 `scripts/config/auto_aim_config_competition.yaml`。

## 3. 启动成功判据

终端建议至少看到以下信息：
- `team_red=true debug_team_blue=false`（红队）或相反（蓝队）
- `Terminal input mode initialized (stdin)` 或 `(/dev/tty)`
- `ROS 2 Shooting Table Calib Node Started`
- `Server started at http://0.0.0.0:8081/stream`

如果有画面但没框，确认：
- `draw_image=true`
- `web_show=true`

## 4. 键盘操作（当前实现）

- `a`：解算并锁定一次（会立即发送瞄准命令）
- `g`：进入仅瞄准模式（不发火）
- `f`：开火（受双轴锁定保护）
- `x`：停止
- `w/s`：pitch 微调（+/-）
- `d/j`：yaw 微调（+/-）
- `h`：保存一条标定记录到 CSV，并在终端提示当前 CSV 路径
- `r`：重置
- `l`：重载射表参数
- `q`：退出
- `t`：测试发布（调试用途）

## 5. 标定流程（实操）

建议每个距离做 5~10 条记录，距离可按 `2m/3m/4m/5m`：

1. 把靶放到目标距离。
2. 按 `a` 让系统锁定并瞄准。
3. 用 `w/s/d/j` 微调到中心命中。
4. 按 `h` 保存记录。
5. 换距离重复。

记录文件位置：

```text
~/workspace/record/shooting_table_*.csv
```

注意：
- 当前实现里，`h` 只是追加保存一条 CSV 记录，不会在按下当下立刻做拟合。
- `r` 只是重置当前控制状态，不会删除或清空 CSV。
- 节点不会在运行中热更新拟合参数；默认启动脚本会在你退出标定后自动拟合当前这轮 CSV。

## 6. 拟合与回写流程

默认推荐流程：

```bash
./scripts/start_shooting_table_calib.sh --team red --output screen
```

做完这一轮标定后按 `q` 退出。脚本会自动：
- 找到当前这轮新生成的 `shooting_table_*.csv`
- 做拟合
- 把结果写回 `scripts/config/auto_aim_config_competition.yaml`

随后直接运行：

```bash
./scripts/start_competition_autoaim_test.sh
```

就会使用这份最新拟合，因为该脚本默认加载的就是 `scripts/config/auto_aim_config_competition.yaml`。

只拟合最新一份 CSV，生成一个参数覆盖 YAML：

```bash
./scripts/start_shooting_table_calib.sh --fit-latest
```

拟合最新一份 CSV，并直接回写比赛配置：

```bash
./scripts/start_shooting_table_calib.sh --fit-latest \
  --write-config scripts/config/auto_aim_config_competition.yaml
```

如果你只想采集，不想退出后自动拟合：

```bash
./scripts/start_shooting_table_calib.sh --team red --output screen --no-auto-fit
```

把 `~/workspace/record` 下全部 CSV 一起拟合：

```bash
./scripts/start_shooting_table_calib.sh --fit-all
```

说明：
- 默认 CSV 采集目录是 `~/workspace/record`。
- `--record-dir` 只影响“拟合时去哪里找 CSV”，不会改变节点实际保存 CSV 的路径。
- 默认标定启动会在退出后自动同步比赛配置；如果不想覆盖比赛配置，使用 `--no-write-config`。
- `--write-config` 会原地改 YAML，并生成一个 `.bak.<timestamp>` 备份文件。
- 如果只想先看拟合结果，不改正式配置，就只用 `--fit-latest` 或加 `--output-fit-yaml <path>`。

## 7. 开火安全保护（已启用）

当前版本加入了“必须 yaw/pitch 同时收敛才允许开火”的保护：

- 开火前若未双轴收敛：阻止开火，打印 `Fire blocked`
- 开火中若失去双轴收敛：立即停火，打印 `Fire stopped`

默认阈值：
- `shooting_table_calib.fire_max_yaw_error_deg = 8.0`
- `shooting_table_calib.fire_max_pitch_error_deg = 5.0`
- `shooting_table_calib.fire_require_dual_axis_lock = true`

## 8. 常见问题排查

### 8.1 按键没反应

检查：
- 是否使用 `--output screen`
- 是否看到 `Terminal input mode initialized (...)`

若显示 `Terminal input disabled`，说明当前终端不是交互 TTY，请在本机交互终端直接启动。

### 8.2 启动报 `COLCON_TRACE: unbound variable`

当前脚本已兼容此问题。若你手上是旧脚本，先拉最新或重开终端后再试。

### 8.3 相机打不开（device has been open）

通常是旧进程占用相机。先执行第 1 节的 `pkill` 清理，再启动。

### 8.4 有检测框但按 `a` 对不上

优先检查：
- 队伍参数是否正确（红队/蓝队）
- 当前目标是否为 `Infantry2`（当前标定节点默认过滤该目标）
- 目标是否在画面中心附近（锁定策略会优先选最近准心目标）

## 9. 可选离线（视频）模式

无相机/离线回放可使用：

```bash
./scripts/start_shooting_table_calib.sh --team red --use-gimbal false --output screen -- \
  detector_config.use_video:=true \
  detector_config/use_video:=true \
  detector_config.video_path:=src/record/record.mkv \
  detector_config/video_path:=src/record/record.mkv
```

该模式用于验证算法链路和流程，不代表实车弹道最终效果。
