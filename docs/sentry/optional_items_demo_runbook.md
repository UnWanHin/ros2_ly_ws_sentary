# 哨兵选做项目展示流程

> 适用范围：按你当前提供的赛项文字，整理成现场可执行流程。  
> 目标：把现有仓库能力和现场展示动作对齐，不额外改业务代码。

---

## 1. 这份文档覆盖什么

按当前规则描述，现场展示分成三块：

1. 小装甲模块辅瞄识别 + 50 发击打展示
2. 哨兵“姿态”切换系统展示
3. 自动导航 / 地形跨越展示

这三块当前都能用现有脚本启动，但不是一套脚本全包。

当前建议：

- 项目一：优先用 standalone 自瞄脚本
- 项目二：优先用 `showcase` 展示模式
- 项目三：优先用导航调试 / standalone 导航巡逻脚本

注意：

- `standalone 自瞄` 和 `showcase` 不要同时运行，它们都会抢 `/ly/control/angles`、`/ly/control/firecode`
- 导航链当前推荐走 `/ly/navi/goal` 点位序号，不走 `/ly/navi/goal_pos`
- 本仓库只负责“发哪些点位 ID”，不负责修改导航侧的坐标定义

---

## 2. 现场总流程建议

推荐按这个顺序做，切换成本最低：

1. 项目一：先做小装甲识别可视化，再做 50 发击打
2. 项目二：停掉 standalone 自瞄，切到 `showcase` 做姿态切换
3. 项目三：停掉 `showcase`，切到导航调试 / standalone 巡逻，做自动导航和跨越展示

推荐分工：

- 1 人负责脚本启动 / 停止
- 1 人负责录像、计时、记录弹量 / 血量材料
- 1 人负责目标模块运动或场地安全

---

## 3. 项目一：小装甲模块辅瞄识别 + 50 发展示

### 3.1 场地与物理条件

按规则现场布置：

- 划两条相距 5 米的平行线 `A`、`B`
- 机器人在 `A` 外自由平移
- 小装甲模块目标沿 `B` 线 5 米范围往复平移
- 目标往复周期平均平移速度大于 `1 m/s`
- 目标自转速度大于 `0.4 r/s`
- 全程 1 分钟内发射 50 发弹丸

说明：

- 目标模块的平移速度、自转速度、血量统计，不是本仓库控制的内容，要由目标装置 / 现场人员保证
- 本仓库负责识别、跟踪、控云台、火控输出和可视化

### 3.2 推荐启动方式

先只展示识别与可视化：

```bash
./scripts/debug.sh autoaim-debug --mode perception --online
```

浏览器打开：

```text
http://localhost:8001/stream
```

确认识别框、目标切换、预测输出正常后，停掉上面的感知-only 链路，再启动完整自瞄：

```bash
./scripts/debug.sh autoaim-test --online
```

如果你要求“必须挂在裁判开赛门控后再接管控制”，不要用 standalone，自瞄和姿态一起走：

```bash
./scripts/start.sh showcase --with-gate -- use_buff:=false use_outpost:=false
```

### 3.3 50 发怎么执行

当前仓库没有“打到 50 发自动停火”的现成闭环。

现场最稳妥的做法：

1. 开始前只装 `50` 发
2. 记录开始时剩余弹量
3. 启动 1 分钟计时
4. 运行完整自瞄链路
5. 结束后记录剩余弹量

辅助记录建议：

```bash
ros2 topic echo /ly/me/ammo_left
```

说明：

- `/ly/me/ammo_left` 来自下位机回传，可作为“发弹前后差值”的辅助证明
- `mapper_node` 的 `tx_fire_count` 只是 firecode 发布次数，不是真实出弹数，不能拿来当 50 发证明

### 3.4 现场展示步骤

推荐按这个节奏做：

1. 启动识别-only：
   `./scripts/debug.sh autoaim-debug --mode perception --online`
2. 打开浏览器展示可视化流：
   `http://localhost:8001/stream`
3. 让目标模块沿 `B` 线运动、自转，展示识别框和目标切换
4. 停掉 perception-only
5. 启动完整自瞄：
   `./scripts/debug.sh autoaim-test --online`
6. 装弹 `50` 发，记录开始弹量
7. 开始 1 分钟计时并击打目标
8. 结束后记录剩余弹量、目标模块前后血量差、现场录像

### 3.5 需要留哪些证明材料

至少保留这些材料：

- 小装甲识别可视化画面录像
- 机器人击打全过程录像
- 目标模块开始前血量 / 结束后血量画面
- 开始前剩余弹量 / 结束后剩余弹量
- 若目标模块或裁判端自带命中数统计，保留该画面

建议现场统计口径：

- 命中率证明优先用目标模块 / 裁判系统实际显示结果
- 若只给血量差，则按现场裁判认可口径换算

### 3.6 当前系统边界

- 当前仓库能完成：识别、可视化、跟踪、火控输出
- 当前仓库不能自动完成：50 发自动收口、命中率自动统计、目标模块血量差自动归档
- 所以项目一当前是“功能展示已具备，取证闭环主要靠现场流程”

---

## 4. 项目二：姿态切换系统展示

### 4.1 推荐启动方式

内部调试、不等门控：

```bash
./scripts/start.sh showcase
```

如果现场接裁判系统，建议正式展示时保留门控：

```bash
./scripts/start.sh showcase --with-gate -- use_buff:=false use_outpost:=false
```

### 4.2 当前展示逻辑

当前 `showcase` 已经把姿态展示收进单独配置：

- 配置文件：`src/behavior_tree/Scripts/ConfigJson/regional/debug/showcase_competition.json`
- 默认巡逻：`ShowcasePatrol.Enable = true`
- 默认姿态切换：
  - 无目标时优先 `Move`
  - 有目标或短时保目标时切 `Attack`
  - 短时间受击累计超过阈值时切 `Defense`
  - 脱战后回主循环

### 4.3 推荐展示步骤

正式展示建议这样讲：

1. 启动 `showcase`
2. 机器人进入巡逻 / 移动态，展示 `Move`
3. 人为喂目标，让系统切到 `Attack`
4. 若已接裁判血量输入，短时受击触发 `Defense`
5. 脱战后回到主循环或继续巡逻

建议同时观察：

```bash
ros2 topic echo /ly/control/posture
ros2 topic echo /ly/gimbal/posture
```

### 4.4 如果要改展示巡逻点

改这里：

- `src/behavior_tree/Scripts/ConfigJson/regional/debug/showcase_competition.json`

重点字段：

- `ShowcasePatrol.Goals`
- `ShowcasePatrol.Random`
- `ShowcasePatrol.GoalHoldSec`

改完后需要：

```bash
colcon build --packages-select behavior_tree
```

### 4.5 当前系统边界

- `Move -> Attack` 不依赖裁判系统，只要有目标就能展示
- `Attack -> Defense` 当前依赖血量下降输入
- 如果没有裁判血量输入，当前仓库没有单独的 HP 注入脚本用于正式演示 `Defense`
- 所以项目二正式展示，推荐接裁判系统或等价血量回传链路

---

## 5. 项目三：自动导航 / 地形跨越展示

### 5.1 先讲当前能力边界

当前仓库能稳定做的是：

- 按预设点位 ID 自动巡逻
- 支持顺序巡逻 / 随机巡逻
- 适合做“自动导航通过若干预设点”或“自动重复跨越若干地形入口”的展示

当前仓库没有实现的是：

- 基于地形完成情况的高级任务择优决策
- 现场临时 XY 坐标由本仓库直接下发

也就是说，项目三当前更适合做：

- “自动导航 + 按路线完成若干地形跨越项目”

不适合直接宣称：

- “全自主在线选择所有项目最优解”

### 5.2 推荐启动方式

如果导航侧已经在线，只想发临时巡逻点：

```bash
./scripts/debug.sh navi_goal --plan test_site_sequence
```

如果你想用 `behavior_tree` 这条链保持发点：

```bash
./scripts/debug.sh navi-debug
```

如果现场接裁判系统，保留门控：

```bash
./scripts/debug.sh navi-debug --with-gate
```

### 5.3 临时路线改哪里

改这里：

- `src/behavior_tree/Scripts/ConfigJson/regional/debug/navi_debug_points.json`

当前格式示例：

```json
{
  "ActivePlan": "test_site_random",
  "Plans": {
    "test_site_random": {
      "Mode": "random",
      "GoalHoldSec": 5,
      "SpeedLevel": 1,
      "DisableTeamOffset": true,
      "IgnoreRecovery": true,
      "Goals": [6, 11, 14, 15]
    }
  }
}
```

你现场主要改：

- `ActivePlan`
- `Plans.<name>.Mode`
- `Plans.<name>.Goals`
- `Plans.<name>.GoalHoldSec`

说明：

- `Mode=sequence`：按数组顺序跑
- `Mode=random`：在这组点里随机切换

如果你只跑 standalone 导航巡逻：

- 改完 JSON 不用 build

如果你跑 `./scripts/debug.sh navi-debug`：

- 改完后要先：

```bash
colcon build --packages-select behavior_tree
```

### 5.4 现场展示步骤

推荐做法：

1. 先选一组你能稳定通过的地形项目
2. 给每个项目准备入口 / 出口点位 ID
3. 把这些点位写进 `navi_debug_points.json`
4. 先用 standalone 巡逻验证顺序
5. 正式展示时按顺序重复通过，每个项目累计完成 3 次
6. 全程录像，禁止人工触碰机器人和场地

### 5.5 非比赛场地的关键风险

当前导航是 `goal ID` 模式，不是直接下发 XY。

所以如果你当前展示场地不是比赛场地，需要提前确认：

- 导航侧已经把这些点位 ID 映射到当前展示场地的真实坐标

如果导航侧还没准备临时坐标表，这边只改 JSON 的 `Goals` 序号是跑不起来的。

---

## 6. 快速命令表

项目一，只看识别可视化：

```bash
./scripts/debug.sh autoaim-debug --mode perception --online
```

项目一，完整自瞄击打：

```bash
./scripts/debug.sh autoaim-test --online
```

项目二，姿态展示：

```bash
./scripts/start.sh showcase
```

项目二，带裁判门控的姿态展示：

```bash
./scripts/start.sh showcase --with-gate -- use_buff:=false use_outpost:=false
```

项目三，单独导航巡逻：

```bash
./scripts/debug.sh navi_goal --plan test_site_sequence
```

项目三，BT 导航调试模式：

```bash
./scripts/debug.sh navi-debug
```

---

## 7. 结论

按当前代码状态：

- 项目一：功能展示可以做，统计 / 取证闭环要靠现场流程补齐
- 项目二：`Move`、`Attack` 已可展示，`Defense` 正式展示建议接裁判血量输入
- 项目三：适合做预设路线式自动导航 / 跨越展示，不宜夸大为复杂在线自主择优

如果只求现场稳定落地，建议把三项拆开跑，不要追求“一套脚本连续演完全部”。
