# 哨兵功能测试框架（BehaviorTree 外）设计方案

## 1. 目标

你要的目标可以总结为：
- 一个配置文件控制多种功能测试（`true/false` 开关）
- 功能按两大类拆分：`云台`、`底盘`
- 支持“云台 + 底盘”同时运行
- 测试框架运行在 `behavior_tree` 外，不依赖 BT 主循环

本方案只做设计，不改接口格式，不改现有 topic/type。

---

## 2. 现状核对（基于现有代码）

### 2.1 云台相关（现有能力）
- **打装甲板**：已具备（`auto_aim.launch.py + mapper_node`）。
  - 参考：`src/detector/launch/auto_aim.launch.py`
  - 参考：`src/detector/script/mapper_node.py`
- **打前哨站**：`outpost_hitter` 有目标输出（`/ly/outpost/target`），但目前 `mapper_node` 默认消费的是 `/ly/predictor/target`，需外部桥接脚本。
- **打符**：`buff_hitter` 有目标输出（`/ly/buff/target`），同样需要桥接脚本。
- **扫描模式**：当前主要逻辑在 BT 内（无目标扫描分支），BT 外需要单独扫描发布脚本。
  - 参考：`src/behavior_tree/src/GameLoop.cpp`

### 2.2 底盘相关（现有能力）
- **回家/策略移动/巡逻路线**：在 BT 内已有（`SetPosition*` 系列），BT 外暂无统一入口。
  - 参考：`src/behavior_tree/src/GameLoop.cpp`
- **发坐标导航**：可在 BT 外直接发 `/ly/navi/goal` 或 `/ly/navi/goal_pos`，但是否执行取决于外部导航消费者是否在线。
- **小陀螺**：当前由 `FireCode.Rotate` 位控制；BT 外可做独立发布器，但要严格对齐 `FireCode` 编码。

结论：你的方向是可行的，但需要“外部测试桥接层”把现有节点拼起来。

---

## 3. 设计原则

- **P0：不侵入 BT**：测试模式在 `scripts/` 和少量独立测试节点里实现。
- **P0：单控制源保护**：默认禁止 `behavior_tree` 与测试节点同时写 `/ly/control/*`。
- **P0：配置驱动**：一个 profile 配置全流程。
- **P1：可组合运行**：`gimbal` 与 `chassis` 可并行，内部子模式可互斥或串行。
- **P1：可审计日志**：每个测试节点输出固定诊断字段（rx/tx、话题发布者、当前模式）。

---

## 4. 目录与组件建议（在 BT 外）

建议新增（后续实现阶段）：

```text
scripts/
  feature_test/
    run_feature_test.sh           # 主入口（读配置+调度）
    lib/
      common.sh                   # source/cleanup/log/timeout
      guard.sh                    # 单控制源检查
      ros_graph.sh                # 节点与话题可用性检查
    run_stack.sh                  # 拉 auto_aim 基础链
    run_gimbal.sh                 # 云台模式调度
    run_chassis.sh                # 底盘模式调度
  config/
    sentry_feature_test.yaml      # 单配置文件
```

必要时新增 ROS2 Python 测试节点（不放 BT 包）：
- `src/detector/script/target_bridge_node.py`（统一桥接 predictor/outpost/buff 的 Target 到 control）
- `src/detector/script/scan_gimbal_test.py`（扫描角度发布）
- `src/detector/script/chassis_vel_test.py`（速度测试发布）
- `src/detector/script/nav_goal_patrol_test.py`（坐标/巡逻点位发布）
- `src/detector/script/firecode_rotate_test.py`（小陀螺位测试发布）

---

## 5. 单配置文件（建议结构）

文件：`scripts/config/sentry_feature_test.yaml`

```yaml
global:
  offline: true
  cleanup_existing: true
  wait_stack_sec: 10
  cmd_timeout_sec: 5
  config_file: "src/detector/config/auto_aim_config.yaml"
  allow_multi_control: false

run:
  gimbal: true
  chassis: true

gimbal:
  mode:
    armor: true
    outpost: false
    buff: false
    scan: false
  fire:
    enable: true
    auto_fire: true
  target:
    priority: [6, 3, 4, 5]
    fallback_id: 6
    timeout_sec: 1.0

chassis:
  mode:
    spin: false
    velocity: true
    nav_goal: false
    patrol: false
  spin:
    rotate_level: 1
    publish_hz: 10
  velocity:
    profile: "square"   # square/circle/custom
    speed_x: 20
    speed_y: 0
    period_sec: 8
  nav_goal:
    use_xy: true
    goals: [[600, 600], [1200, 600], [1200, 1000], [600, 1000]]
  patrol:
    route: ["CastleLeft", "CastleRight1", "BuffShoot", "MidShoot"]
    dwell_sec: 5
```

约束建议：
- `gimbal.mode` 同时只允许一个 `true`（防冲突）。
- `chassis.mode` 可同时多项 `true`，但需在脚本层标注冲突规则。

---

## 6. 功能到实现映射（你提的项目）

### 6.1 云台类

1) 打装甲板  
- 直接复用：`auto_aim + mapper_node`

2) 打前哨站  
- 用 `outpost_hitter` 作为 target source，走 `target_bridge_node.py`

3) 打符  
- 用 `buff_hitter` 作为 target source，走 `target_bridge_node.py`

4) 扫描模式  
- `scan_gimbal_test.py` 直接发 `/ly/control/angles`（可选上下摆 + 左右扫）

### 6.2 底盘类

1) 小陀螺  
- `firecode_rotate_test.py` 发布 `Rotate` 位（不触发 FireStatus）
- 风险点：`FireCode` 位域编码必须与 `gimbal_driver` 一致

2) 发坐标导航  
- `nav_goal_patrol_test.py` 发布 `/ly/navi/goal_pos`（或 `/ly/navi/goal`）
- 需外部导航消费者在线才能看到实车动作

3) 自动巡逻  
- 用固定 route 定时轮询发布目标点/坐标
- 本质是“外部巡逻脚本”，不走 BT 策略树

---

## 7. 运行流程（主脚本）

`run_feature_test.sh` 建议流程：

1. 读取配置，打印生效配置摘要  
2. 清理残留进程（可选）  
3. 拉起基础感知链（`auto_aim.launch.py`，按 `offline` 覆盖）  
4. 运行 `guard`：
   - 检查 `/behavior_tree` 是否在线
   - 检查 `/ly/control/angles`、`/ly/control/firecode` 多发布者
5. 启动 gimbal 模式进程  
6. 启动 chassis 模式进程  
7. 周期诊断输出（topic publisher/订阅、关键 rx/tx 计数）  
8. Ctrl+C 统一回收：先停测试节点，再停 launch，最后发送安全归零命令（firecode=0, vel=0）

---

## 8. 验收标准（建议）

- `P0-1`：单配置拉起 `armor` 模式，10s 内可见控制话题输出。
- `P0-2`：`allow_multi_control=false` 时，检测到 `/behavior_tree` 在线会拒绝接管。
- `P0-3`：`gimbal + chassis` 同时跑时，不出现脚本级崩溃，退出可清理干净。
- `P1-1`：`outpost`/`buff` 模式可独立切换并驱动控制输出。
- `P1-2`：`scan`/`velocity`/`patrol` 均有统一诊断日志字段。

---

## 9. 风险与规避

- **FireCode 位域风险**：小陀螺测试若编码错，会误触发开火。  
  规避：单独写编码/解码自检，默认先在 debug 话题验证。
- **导航执行依赖外部栈**：仓库内不保证有完整消费者。  
  规避：把“话题发布成功”与“车辆执行成功”分开判定。
- **多发布者冲突**：云台抽动/指令互抢。  
  规避：主脚本默认启用单控制源保护。

---

## 10. 分阶段落地建议

- **Phase 1（最小可用）**
  - `armor + scan + velocity`
  - 主脚本 + 配置 + 单控制源保护 + 统一清理

- **Phase 2（扩展）**
  - `outpost + buff + nav_goal/patrol`
  - 通用 `target_bridge_node.py`

- **Phase 3（完善）**
  - 小陀螺位测试与编码校验
  - 与 `self_check_sentry.sh` 集成测试 profile

