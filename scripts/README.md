# Scripts Guide

这次把 `scripts/` 重新收敛了，根目录只保留 3 个入口：

- `./scripts/start.sh`
- `./scripts/debug.sh`
- `./scripts/selfcheck.sh`

它们都支持两种用法：

- 不带参数：进入交互菜单
- 带子命令：直接转发到对应分类脚本

例如：

```bash
./scripts/start.sh
./scripts/start.sh gated --mode league
./scripts/debug.sh armor_test --mode regional
./scripts/selfcheck.sh sentry --runtime-only --launch --wait 12 --skip-hz
```

如果你想快速知道“姿态怎么测、`/ly/navi/goal` 怎么发、比赛主链怎么起”，先读：

- `scripts/FUNCTION_GUIDE.md`

## 目录结构

```text
scripts/
├── start.sh                 # 启动类总入口（交互菜单）
├── debug.sh                 # 调试/联调类总入口（交互菜单）
├── selfcheck.sh             # 自检类总入口（交互菜单）
├── python/start.py          # 离线决策一键入口（固定 regional）
├── start/                   # 启动类分类脚本
├── debug/                   # 调试类分类脚本
├── selfcheck/               # 自检类分类脚本
├── launch/                  # 完整/决策 stack 启动实现；少量旧入口兼容 wrapper
├── aim/                     # 辅瞄/识别测试 wrapper
├── navi/                    # 导航/TF/固定朝向工具 wrapper
├── areatest/                # regional 单区域 AreaManager 实链路测试 wrapper
├── feature_test/            # 单项功能测试框架
├── tools/                   # 工具脚本
└── config/                  # 根层配置（base + override）
```

原则：

- `scripts/start/`、`scripts/debug/`、`scripts/selfcheck/` 是你平时真正需要打开的分类入口。
- `scripts/debug/` 是有意暴露出来的稳定调试接口；即使里面有 wrapper，也保留给人直接找命令用。
- `scripts/launch/` 只保留完整/决策 stack；少量历史命令只做兼容转发。辅瞄、导航、标定分别在 `scripts/aim/`、`scripts/navi/`、`scripts/tools/`。
- 根层 `config/` 只放全局共享配置，例如 `base_config.yaml`、`override_config.yaml`、`common.yaml`。
- `behavior_tree` 自己的状态机/任务开关配置放在 `src/behavior_tree/config/`，例如 `AreaManager.yaml`、`Task.yaml`。
- 功能测试配置放在 `scripts/feature_test/config/`。
- 旧的根目录壳脚本已经删掉，避免同一件事出现两三个名字。

## 一眼看懂

### 1. 正式比赛主入口

用这个：

```bash
./scripts/start.sh gated --mode league      # 联盟赛，有门控
./scripts/start.sh gated --mode regional    # 分区赛，有门控
./scripts/start.sh nogate --mode league     # 联盟赛，无门控
./scripts/start.sh nogate --mode regional   # 分区赛，无门控
```

对应脚本：

- `scripts/start/sentry_all.sh`
- `scripts/start/sentry_all_nogate.sh`
- 实际实现：`scripts/launch/start_sentry_all.sh`

`scripts/launch/start_sentry_all.sh` 会读取 `config/common.yaml` 的现场共享开关。`rosbag_play_enable: true` 时会启动 `ros2 bag play`，默认路径是 `rosbag_path: ~/Log/rosbag`；同时 gimbal 走虚拟 IO，detector 切到 `use_ros_bag=true`。

### 2. `armor_test` 是什么

现在用这个名字：

```bash
./scripts/debug.sh armor_test
```

对应脚本：

- `scripts/debug/armor_test.sh`
- 实际实现：`scripts/aim/armor_test.sh`

它不是“正式比赛完全等价入口”，而是“比赛风格辅瞄预设”。

默认行为：

- 使用分层配置（`config/base_config.yaml` + `src/detector/config/detector_config.yaml` + `src/predictor/config/predictor_config.yaml`，并叠加 `config/override_config.yaml`）
- 默认 `--mode regional`
- 默认 `--nogate`
- 默认启用：`gimbal_driver / detector / tracker_solver / predictor / behavior_tree`
- 默认关闭：`buff_hitter / outpost_hitter`

所以它适合：

- 快速验证比赛风格的 autoaim 主链
- 快速看 `behavior_tree` + `predictor` 的联调效果

但它不等于正式比赛整套默认路径，因为正式比赛主入口仍然是：

```bash
./scripts/start.sh gated --mode league
./scripts/start.sh gated --mode regional
```

### 2.1 离线决策一键入口（固定 regional）

```bash
python3 ./scripts/python/start.py
```

默认会同时打开：

- pygame 窗口
- 端口画面流：`http://127.0.0.1:<port>/`（默认读 `src/decision_viz/config/default.yaml` 的 `web_stream.port`，默认值 9000）
- 分区赛超对抗 7 分钟离线比赛时钟（默认 420 秒）
- 右侧控制按钮：`Start` / `Pause` / `+10s` / `-10s` / `Reset`
- 默认保留 `/ly/game/is_start` 门控；点击 `Start` 才会进入开赛状态
- 默认离线仅发官方地图坐标（关闭 tf goal bridge 转换）

可选：

```bash
python3 ./scripts/python/start.py --target predictor
python3 ./scripts/python/start.py --trace
python3 ./scripts/python/start.py --no-view
python3 ./scripts/python/start.py --web-port 9010
python3 ./scripts/python/start.py --match-duration-sec 420
python3 ./scripts/python/start.py --control-file /tmp/decision_viz_match_control.jsonl
python3 ./scripts/python/start.py --bypass-is-start
python3 ./scripts/python/start.py --keep-to-navi
```

### 3. `showcase` 是什么

`showcase` 不是联赛，也不是分区赛。

它只是“姿态展示 / 展示巡逻”模式。

现在用这个：

```bash
./scripts/start.sh showcase
```

对应脚本：

- `scripts/start/showcase.sh`
- 实际实现：`scripts/launch/start_sentry_showcase.sh`

核心特点：

- 固定走 `mode 3`
- 底层还是 `regional` 主流程
- 自动换成 `regional/debug/showcase_competition.json`
- 用于姿态展示、展示巡逻、演示链路

### 4. `start_sentry_all_competition.sh` 还要不要

不要了，已经删掉。

原因很简单：

- 它原来只是兼容壳
- 最终还是转发到 `start_sentry_all.sh`
- 保留它只会让“正式入口到底是谁”更乱

## 分类脚本对照

### Start

| 分类脚本 | 用途 | 实际实现 |
| --- | --- | --- |
| `scripts/start/sentry_all.sh` | 正式整链路入口 | `scripts/launch/start_sentry_all.sh` |
| `scripts/start/sentry_all_nogate.sh` | 绕过开赛门控的整链路入口 | `scripts/launch/start_sentry_all_nogate.sh` |
| `scripts/start/showcase.sh` | 姿态展示 / 展示巡逻 | `scripts/launch/start_sentry_showcase.sh` |

### Debug

| 分类脚本 | 用途 | 实际实现 |
| --- | --- | --- |
| `scripts/debug/armor_test.sh` | 比赛风格辅瞄预设 | `scripts/aim/armor_test.sh` |
| `scripts/debug/navi_debug.sh` | behavior_tree-only 导航调试 | `scripts/launch/start_sentry_navi_debug.sh` |
| `scripts/debug/standalone.sh` | 单项功能测试菜单 | `scripts/feature_test/standalone/run_standalone_menu.sh` |
| `scripts/debug/navi_goal.sh` | JSON 巡逻点发 `/ly/navi/goal` | `scripts/feature_test/standalone/modes/navi_patrol_mode.sh` |
| `scripts/debug/navi_goal_cli.sh` | 手动发导航目标 | `scripts/feature_test/standalone/tools/navi_goal_cli_pub.py` |
| `scripts/debug/ballistic_error_log.sh` | 过滤弹道/锁敌日志 | `scripts/tools/monitor_ballistic_errors.sh` |
| `scripts/debug/shooting_table_calib.sh` | 射表标定 | `scripts/tools/shooting_table_calib.sh` |
| `scripts/debug/buff_shooting_table_calib.sh` | 打符射表标定 | `scripts/tools/buff_shooting_table_calib.sh` |
| `scripts/debug/control_angles_test.sh` | 直接发 `/ly/control/angles` 角度命令 | 脚本内置发布逻辑 |
| `scripts/debug/rotate_level.sh` | Rotate 档位循环与回读测试 | 脚本内置发布/回读逻辑 |
| `scripts/debug/move_rotate.sh` | 小陀螺 + 正弦平移联动测试 | `scripts/feature_test/standalone/modes/chassis_spin_sine_translate_mode.sh` |
| `scripts/debug/posture_test.sh` | 姿态切换循环与回读测试 | 脚本内置发布/回读逻辑 |
| `scripts/debug/sentry_cmd_downlink_test.sh` | 启动 `gimbal_driver`，默认每 5 秒轮发姿态 1/2/3，并验证 `SentryCmd` raw 下行帧/上行回读 | 脚本内置 `/ly/control/posture -> /ly/control/sentry_cmd` relay、raw TX/RX 和 RFID/姿态观察逻辑 |
| `scripts/debug/chase_only.sh` | 纯追击联调（无门控，默认连下位机） | `scripts/launch/start_sentry_chase_only.sh` |
| `scripts/debug/outpost_target_test.sh` | 发 `/ly/outpost/target` yaw 序列，验证前哨桥接 | 脚本内置发布逻辑 |
| `scripts/debug/goal_pos_test.sh` | 仅用于静态点位 `/ly/navi/goal_pos_raw` 转换测试，预览后确认才发 `geometry_msgs/PoseStamped /goal_pose`；不是追击测试 | `scripts/navi/navitomap.sh` |

### Navi / Aim-Point

| 脚本 | 用途 |
| --- | --- |
| `scripts/navi/navitomap.sh` | 手动官方地图点转 `/goal_pose` |
| `scripts/navi/OfficialToNavi.sh` | 纯静态换算：official map 点 -> navi/map 点，默认 official 输入 cm、navi 输出 m |
| `scripts/navi/NaviToOfficial.sh` | 纯静态反向换算：navi/map 点 -> official map 点，默认 navi 输入 m、official 输出 cm |
| `scripts/navi/facemode.sh` | 兼容旧入口，等同 `facemode_cross_matrix.sh` |
| `scripts/navi/facemode_cross_matrix.sh` | FaceMode 官方 X/Y 经 `tf_config.yaml` 矩阵转 map，默认输入 cm |
| `scripts/navi/facemode_map.sh` | FaceMode 直接使用导航 `map` 系 X/Y/Z，不过矩阵；默认用 `map -> gimbal_small_yaw` 几何解，不走相机，输入 m |
| `scripts/navi/facemode_official.sh` | FaceMode 直接使用 `official_map` frame，不过矩阵，默认输入 cm，需要 TF 中有 official_map 链路 |
| `scripts/navi/position.sh` | 一键查看 `/ly/navi/position`，默认只输出 `data: [official_map_x_cm, official_map_y_cm]` |
| `scripts/navi/map_aim_point_test.sh` | FaceMode 完整测试入口，可拉 `gimbal_driver` / `tf_tree`，支持 `--unit m\|cm`，默认 cm |
| `scripts/navi/map_aim_point_attach.sh` | 已有 stack 上只附加 FaceMode 节点 |

`kabsch_calib` / `affine_calib` / `navi_calib_simple` 標定工具的默认单位链路是 `official_map(m) -> map(m) -> raw_goal_transform_matrix(m)`；如果官方地图点按 cm 存，要在 YAML 或命令行显式写 `source_unit: cm` / `--source-unit cm`。矩阵输出给 `tf_config.yaml` 时保持 m；工具都会在 `--help` 和互动输入时显示单位链路。

`kabsch_calib` 和 `affine_calib` 的默认可编辑点表都是 `src/navi_tf_bridge/config/navi_calib.yaml`。直接运行 `python3 src/navi_tf_bridge/script/kabsch_calib.py` 或 `python3 src/navi_tf_bridge/script/affine_calib.py` 后，如果在 `pair[1]` 直接回车，会使用这份 YAML；如果输入任意点，则使用新输入的点重新标定。

### Area Test

| 脚本 | 用途 |
| --- | --- |
| `scripts/areatest/regional_base.sh` | 单测我方 Base 区域任务，走正式 `sentry_all --mode regional` 和 AreaManager |
| `scripts/areatest/regional_highland.sh` | 单测我方 Highland 区域任务，走正式 regional 链路 |
| `scripts/areatest/regional_roadland.sh` | 单测我方 Roadland 区域任务，走正式 regional 链路 |
| `scripts/areatest/regional_central.sh` | 单测 Common Central 区域任务，走正式 regional 链路 |
| `scripts/areatest/regional_area_test.sh` | 上面四个脚本的共用 runner，可手动传 `base/highland/roadland/central` |

单区域 regional 脚本不是 `navi_debug`，也不是直接发 `/ly/navi/goal`。它们通过专用 `bt_config_file` 只啟用一個大區域，再启动 `scripts/launch/start_sentry_all.sh --mode regional`，因此會從 Default 的大區域輪換入口進入 `TrySetScopedPositionByBaseGoal()`、AreaManager 和實際 `/ly/navi/goal_pos_raw -> /goal_pose` 鏈路。

默认模式仍保留正式链路里可用的 autoaim/fire/posture 行为，便于测接近实战的单区域效果。只想测区域内无事件时的游走/驻守，用 `--pure`：

```bash
./scripts/areatest/regional_roadland.sh --pure
```

`--pure` 会切到 pure preset：停火、关闭 Chase、关闭 Posture、忽略 Recovery 回补，并默认不启动 detector/tracker/predictor/outpost/buff 节点；AreaManager 区域任务和导航桥仍走正式链路。

如果在桌面/台架上没有真实裁判数据，低血量/低弹量默认值会触发 recovery，Central 也不会进入健康巡逻。可以临时加：

```bash
./scripts/areatest/regional_central.sh --pure --fake-referee
```

`--fake-referee` 只用于测试，会发布健康的 `/ly/game/all`、`/ly/friend/ammo_left` 和开赛标志；上车连真实下位机时不要开这个选项。

### Selfcheck

| 分类脚本 | 用途 |
| --- | --- |
| `scripts/selfcheck/pc.sh` | 开发机 build + 静态自检 |
| `scripts/selfcheck/robot.sh` | 上车硬件/网络预检查 + 运行态自检包装 |
| `scripts/selfcheck/sentry.sh` | 底层核心套件，可做静态或运行态检查 |

## 三个顶层菜单怎么用

### `./scripts/start.sh`

菜单项：

1. `gated`
2. `nogate`

说明：

- 进入后会继续二次询问你要跑 `league` 还是 `regional`
- `showcase` 仍保留为直达入口，但不出现在交互菜单里：

```bash
./scripts/start.sh showcase
```

### `./scripts/debug.sh`

菜单项：

1. `armor_test`
2. `navi-debug`
3. `standalone`
4. `navi_goal`
5. `navi-goal-cli`
6. `ballistic-log`
7. `shooting-table-calib`
8. `buff-shooting-table-calib`
9. `control-angles-test`
10. `rotate_level`
11. `move_rotate`
12. `posture-test`
13. `sentry-cmd-downlink`
14. `chase-only`
15. `outpost-target-test`
16. `goal-pos-test`

### `./scripts/selfcheck.sh`

菜单项：

1. `pc`
2. `robot`
3. `sentry`

区别：

- `pc`：开发机用。可选 build，然后跑静态检查。
- `robot`：车上用。先看串口/权限/网络，再调用 `sentry` 做运行态检查。
- `sentry`：底层总套件。既能 `--static-only`，也能 `--runtime-only --launch`。`pc` 和 `robot` 都是在包装它。

## 常用命令

```bash
# 联盟赛，有门控
./scripts/start.sh gated --mode league

# 分区赛，无门控
./scripts/start.sh nogate --mode regional

# 展示姿态
./scripts/start.sh showcase

# 比赛风格 autoaim 预设
./scripts/debug.sh armor_test --mode league

# 射表标定
./scripts/debug.sh shooting-table-calib --team red --output screen

# 打符射表标定（采样）
./scripts/debug.sh buff-shooting-table-calib --calib-mode periodic --csv-strategy latest

# 弹道/锁敌异常日志过滤
./scripts/debug.sh ballistic-log --with-valid

# 导航调试
./scripts/debug.sh navi-debug

# 纯追击联调（默认无门控，连下位机）
./scripts/debug.sh chase-only

# 静态点位转换测试：输入官方/原始 x y，确认后才发布 /goal_pose；不是追击
./scripts/debug.sh goal-pos-test

# 离车自检
./scripts/selfcheck.sh pc

# 上车运行态自检
./scripts/selfcheck.sh robot --with-hz
```

## 备注

- 这次重构只收敛了 `scripts/` 的入口层，`scripts/launch/`、`scripts/aim/`、`scripts/navi/`、`feature_test/`、`tools/` 仍保留实现。
- 仓库里其他文档如果还出现旧命令，按上面的新入口映射替换理解即可。
