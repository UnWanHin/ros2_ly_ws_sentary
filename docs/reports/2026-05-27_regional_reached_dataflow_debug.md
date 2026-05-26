# Regional Reached Dataflow Debug

Date: 2026-05-27

Scope: `scripts/start.sh gate --mode regional` regional 主链路、`behavior_tree` 到达判定、Outpost / FaceMode 入口。原排查描述里的 `regioanl` 是拼写错误；`scripts/start.sh` 只识别 `regional`。

Evidence bags:

- `/home/hustlyrm/record/20260517_083127/main`
- `/home/hustlyrm/record/20260517_084309/main`

## Conclusion

用户判断方向是对的：当前项目里“到达”不是一个统一事实，而是多条源在不同位置被分别消费。

`/ly/navi/reached` 只能定义为外部导航源之一，不能定义为 BT 内部最终 reached。内部最终判断至少要聚合：

- `/ly/navi/reached`：外部导航的到达源。
- `/ly/navi/reachable`：外部导航的不可达源。
- 自身融合坐标：来自 `/ly/friend/uwb_pos`、`/ly/position/data`、`/ly/navi/position`，用于距离 goal 判断。
- goal-start grace / travel timeout / progress watchdog：用于防止长期卡住，但应区分“物理到达”和“任务终止/超时放行”。

当前 `IsBaseGoalArrived()` 已经有一部分聚合：先看当前 goal 的 reachable/reached，再在 grace 后用融合坐标距离兜底。但这个聚合没有被封装成明确的 `GoalReachState` 合约，`EventManager`、blackboard、trace 和部分任务语义仍然把 raw `/ly/navi/reached` 当作 `GoalReached`。这就是核心数据流问题。

## Fresh Means

`fresh` 不是消息里的字段，它是本地根据接收时间和当前 goal 做出的有效性判断。

`/ly/navi/reached` callback 只保存三件事：

- `naviReach = msg->data`
- `hasReceivedNaviReach_ = true`
- `lastNaviReachRxTime_ = now`

`GetExternalNaviReachForGoal()` 返回 `std::optional<bool>`。它为 `has_value()` 时，才表示 external reached 对当前 goal 是 fresh。条件在 `IsNaviExternalStatusFreshForGoal()` 里：

- 已初始化外部导航状态 goal。
- 外部状态记录的 goal id 等于被查询 goal id。
- 外部状态记录的 goal 坐标等于被查询 goal 坐标。
- 当前 `naviCommandGoal` 仍等于这个 goal id。
- `last_rx` 非零。
- `last_rx >= naviExternalStatusGoalStartTime_`，即消息必须在本次 goal 下发之后到达。
- `now - last_rx <= kNaviExternalStatusTimeoutMs`，当前为 2000 ms。

注意：`EventManager::Fresh()` 是另一套更弱的 freshness，只检查收到过、时间戳非零、未超时；它没有 goal id / goal position 约束。因此 `EventSnapshot.GoalReached` 现在不能代表“当前 goal 的内部 reached”。

## Bag Evidence

两份 bag 都显示外部 reached 长期不能作为唯一依据：

| Bag | `/ly/navi/reached` | true | false | `/ly/vision/mode` | `/ly/face_mode/target_raw` |
|---|---:|---:|---:|---|---:|
| `20260517_083127/main` | 44,241 | 0 | 44,241 | 全程 `1` armor | 0 |
| `20260517_084309/main` | 48,522 | 0 | 48,522 | 全程 `1` armor | 0 |

第一份 bag 的关键窗口：

- 97.157s 下发 `/ly/navi/goal_pos_raw = (1580, 150)`。
- 107.156s `/ly/navi/position = (1586, 159)`，距离 goal 约 10.8 cm。
- 该 15s 窗口内 `/ly/navi/reached` 收到约 1497 条，但没有任何 true。

这说明外部导航在物理位置已经足够接近时仍持续发布 false。如果上层消费者只看 `/ly/navi/reached`，就会把已经接近 goal 的状态误判为未到达。

第二份 bag 还有记录侧问题：`/ly/aim/armor_targets` 因 QoS reliability 不兼容没有录到消息，无法从 bag 完整复盘外部 aim 候选目标链路。但这不影响 reached 结论，因为 `/ly/navi/reached`、`/ly/navi/position`、`/ly/vision/mode` 和 `/ly/face_mode/target_raw` 都已经足够证明 reached 源不统一。

## Current Code Findings

### Finding 1: Raw external reached leaks into event state

`EventManager::Evaluate()` 当前逻辑：

```cpp
snapshot.NaviReachFresh = Fresh(...);
snapshot.GoalReached = snapshot.NaviReachFresh && input.NaviReach;
```

这只等于“最近收到 raw `/ly/navi/reached=true`”，不是“当前 goal 已经由内部聚合判定到达”。而且这条 event freshness 没有绑定 goal id / goal position。

Impact:

- `EventGoalReached` blackboard 字段和 decision trace 的 `events.goal_reached` 名字会误导调试。
- 未来任何节点如果消费 `EventGoalReached`，都会绕过 position 兜底和 timeout 保护。

### Finding 2: Composite reached exists as function behavior, not as interface

`IsBaseGoalArrived()` 当前已经做了部分聚合：

1. 当前 goal 的 `/ly/navi/reachable=false`，直接未到达。
2. 当前 goal 的 `/ly/navi/reached=true`，直接到达。
3. goal-start grace 内，不使用坐标兜底。
4. grace 后，融合自身坐标进入到达半径则到达。

这比只看 `/ly/navi/reached` 合理，但它只是一个 `bool` 函数，没有暴露来源、距离、freshness、grace 状态、timeout 状态。消费者只能再写别的判断，例如 `IsBaseGoalWithinDistance()`、watchdog timeout、regional task travel timeout，于是 reached 概念继续分裂。

### Finding 3: Timeout should not be collapsed into reached bool

用户说“超时也是一个源”这个方向需要拆细：

- 如果下游要的是“可以推进状态机”，应定义 `goal_done` 或 `terminal_status`，它可以包含 reached / unreachable / timeout。
- 如果下游要的是“已经物理到点，可以开 FaceMode 或贴点射击”，不应把 timeout 当成 reached。

否则 timeout 会让系统在没有到点时误进入 FaceMode / Outpost 射击姿态。正确接口应同时给出 `status` 和 `reason/source`，而不是只有一个 Bool。

### Finding 4: Outpost / FaceMode 当前需要统一读 composite state

当前代码里 Outpost visual scout 已经有一些避免 strict reached 的逻辑，例如 `outpost_visual_scout_face_ready = point_reached || IsBaseGoalWithinDistance(...)`。这说明方向已经开始从 raw reached 转向距离条件。

但这仍然不是统一接口：

- `outpost_visual_scout_point_reached` 用 `IsBaseGoalArrived()`。
- `outpost_visual_scout_face_ready` 额外用 `IsBaseGoalWithinDistance()`。
- progress watchdog 用 `AreaManager::TickProgressWatchdog()` 自己判断接近/移动/timeout。
- event snapshot 的 `GoalReached` 仍只看 raw external reached。

因此排查时会出现“一个地方认为接近了，另一个地方还认为没 reached”的状态。

## Interface Recommendation

不要把 composite topic 命名为 `/ly/gimbal/reached`。这个语义属于 BT / navigation decision，不属于云台硬件或 gimbal driver。更合适的命名：

- 内部 C++ 合约：`GoalReachState` 或 `NaviGoalReachState`。
- 对外 debug topic：`/ly/bt/goal_reach_state`。
- 如果必须给导航域发布 Bool：`/ly/navi/reached_composite`，但 Bool 会丢失来源，不建议作为唯一 debug 面。

建议先做内部接口，不急着把 topic 作为控制面：

```cpp
enum class GoalReachStatus {
    Unknown,
    Traveling,
    Reached,
    Unreachable,
    Timeout
};

struct GoalReachState {
    std::uint8_t goal_id;
    Area::Point<std::uint16_t> goal_position;
    GoalReachStatus status;
    bool external_reach_fresh;
    bool external_reach;
    bool external_reachable_fresh;
    bool external_reachable;
    bool position_fresh;
    double distance_cm;
    bool within_arrive_distance;
    bool distance_fallback_allowed;
    bool travel_timeout;
};
```

Then:

- `IsBaseGoalArrived()` 只包装 `EvaluateGoalReach(...).status == Reached`。
- `IsBaseGoalExternallyUnreachable()` 包装 `status == Unreachable` 或保留明确 external-only 名字。
- `EventManager` 不再自己计算 raw `GoalReached`，而是接收 `GoalReachState` 的结果。
- Decision trace 记录 `goal_reach.status`、`goal_reach.reason/source`、`distance_cm`，不再只记录 ambiguous bool。
- Outpost / FaceMode 消费 `status == Reached` 或 `distance_cm <= VisualScoutFaceDistanceCm`，不要直接读 `/ly/navi/reached`。

## Issues To Track

### RCH-001: `EventSnapshot.GoalReached` 名字错误

Current: 表示 raw `/ly/navi/reached` fresh true。

Expected: 表示当前 goal 的内部 composite reached，或改名为 `ExternalNaviReached`。

Acceptance:

- blackboard 不再暴露误导性的 `EventGoalReached`，或其值来自 composite state。
- decision trace 能区分 external reached、position reached、timeout、unreachable。

### RCH-002: Goal freshness 逻辑有两套

Current:

- `GetExternalNaviReachForGoal()` 有 goal id / position / start time 约束。
- `EventManager::Fresh()` 只有时间约束。

Expected: 所有“当前 goal reached/reachable”判断必须走同一个 goal-scoped freshness。

Acceptance:

- `EventManager` 不直接输入 raw `HasNaviReach/NaviReach/LastNaviReachRxTime` 来生成 reached。
- 单元或 selfcheck 能覆盖“旧 goal 的 reached 不能污染新 goal”。

### RCH-003: Reached、done、timeout 三种语义需要拆开

Current: 多处只拿 Bool 判断，容易把“可推进”和“物理到点”混在一起。

Expected:

- `Reached` 表示物理/可信到达。
- `Unreachable` 表示外部导航判定不可达。
- `Timeout` 表示保护性推进，不等于物理到点。

Acceptance:

- Outpost FaceMode 只由 physical reached 或 face-distance readiness 打开。
- Regional patrol phase 可以根据 done/timeout 推进，但 trace 必须说明原因。

### RCH-004: Outpost / FaceMode gate 必须消费统一接口

Current: Outpost 相关逻辑同时使用 `IsBaseGoalArrived()`、`IsBaseGoalWithinDistance()` 和独立 timeout/cooldown 状态。

Expected: Outpost gate 的输入来自同一个 `GoalReachState`，并明确使用哪个字段：

- travel 是否继续：`status`。
- FaceMode 是否打开：`distance_cm <= VisualScoutFaceDistanceCm` 或 `status == Reached`。
- no-target timeout 是否开始：FaceMode 已打开后开始，不依赖 raw `/ly/navi/reached`。

Acceptance:

- replay 中 `/ly/navi/reached` 全 false 但 position 已接近时，能进入 `/ly/vision/mode=3` 和 `/ly/face_mode/target_raw`。
- trace 可看到进入原因是 `distance_within_face_range` 或 `position_reached`。

### RCH-005: 缺少 composite reached debug topic

Current: 外部只能看到 raw `/ly/navi/reached`，看不到 BT 内部为什么认为到达、未到达或 timeout。

Expected: 增加 debug topic 或 trace 字段，优先 trace，topic 可选。

Acceptance:

- 每个 goal 至少记录 goal id、goal position、self position、distance、external reached/reachable fresh/value、status、reason。
- 如果发布 ROS topic，优先 `/ly/bt/goal_reach_state`，不要使用 `/ly/gimbal/reached`。

### RCH-006: Bag 记录 QoS 会遮蔽 aim 侧证据

Current: `20260517_084309/main` 的 `/ly/aim/armor_targets` 因 recorder RELIABLE vs publisher BEST_EFFORT 没录到消息。

Expected: recorder 对 `/ly/aim/armor_targets` 使用 best_effort QoS override。

Acceptance:

- 新 bag 中 `/ly/aim/armor_targets` count 不为 0。
- debug report 能同时复盘 reached、vision mode、face mode、aim target array。
