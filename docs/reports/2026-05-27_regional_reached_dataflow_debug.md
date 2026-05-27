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

## Position Dataflow Follow-up

Updated: 2026-05-28

本轮额外核查 `/ly/position/data`、相机目标、`/ly/enemy/info`、`/ly/friend/info` 和 `/goal_pose` 的关系。结论：

- `/ly/position/data` 的 enemy 字段在两份 bag 中都没有有效坐标。
- BT 中可见的敌方位置不是 `/ly/position/data` enemy，而是 `navi_tf_bridge` 输出的 `/ly/navi/target_official` 被 BT 写回 `enemyRobots` 后发布到 `/ly/enemy/info`，`position_source=navi_target_official`。
- 相机源 `/ly/aim/armor_targets` 不直接覆盖 `enemyRobots`。它先进入 BT 的 `externalAimTargets_` / `armorList`，用于目标选择、`hitableTargets` 和 Chase 相对点；同时 `navi_tf_bridge` 会订阅该 topic，把有效 target point 反算成 `/ly/navi/target_official`，作为 BT 敌方 official-map fallback。
- 自身位置进入 BT 的源是 `friendRobots[Sentry].position_`，由 `AreaManager.SentryPositionFusion` 融合 `/ly/friend/uwb_pos`、`/ly/navi/position`、`/ly/position/data` 的 sentry friend slot。默认 priority 顺序是 UWB、Navi、PositionData。两份 bag 的 `/ly/friend/info` 里 Sentry position source 全是 `uwb`，说明当时实际被 BT 采用的自身位置主源是 `/ly/friend/uwb_pos`。

Bag 证据：

| Bag | `/ly/position/data` count | enemy non-zero | friend Sentry non-zero | `/ly/enemy/info` position source | `/ly/aim/armor_targets` | `/ly/navi/target_rel` | `/ly/navi/target_map` |
|---|---:|---:|---:|---|---:|---:|---:|
| `20260517_083127/main` | 3,374 | 0 | 674 | `navi_target_official` for Sentry/Infantry1/Infantry2/Engineer/Hero | 17,046 | 144 | 58 |
| `20260517_084309/main` | 4,853 | 0 | 971 | `navi_target_official` for Sentry/Infantry2 | 0 recorded | 0 | 0 |

第二份 bag 的 `/ly/aim/armor_targets` 为 0 是记录 QoS 问题，不等于运行时没有相机目标。证据是 `/ly/enemy/info` 仍出现 `position_source=navi_target_official`，说明运行时 BT 收到了 bridge 反算后的 official fallback；只是 recorder 没把上游 `/ly/aim/armor_targets` 录下来。

`/goal_pose` 来源需要分两类看：

- 大多数 `/goal_pose` 由 BT 发布 `/ly/navi/goal_pos_raw`，再由 `navi_tf_bridge` 转成 `/goal_pose`。例如 `Blue BuffOutpost (1580,150)`、`Blue OutpostGuard (1831,1132)`、`Blue Castle (2132,749)`、`Blue HoleRoad (1677,204)`。
- 追击相对点链路是 `/ly/navi/target_rel -> /goal_pose`。第一份 bag 只有 144 条，全部是 `armor=Infantry1`、`frame=gimbal_world`；第二份 bag 为 0。因此这两份 bag 里大部分 `/goal_pose` 不是 Chase relative target 直接生成，而是 BT 固定 official goal 经 bridge 生成。

对“回防 goal_pose 是雷达/裁判数据还是自身数据触发”的判断：

- 不是 `/ly/position/data` enemy 触发；两份 bag enemy 坐标为 0。
- 有一部分 `Castle` / `HoleRoad` / `CastleRight2` 这类回防/防守点是在有新鲜 `navi_target_official` 敌方位置时出现的。例如第一份 bag 136s 以后 `/ly/enemy/info` 出现 `Sentry:...:navi_target_official`，随后出现 `Blue Castle`、`Blue HoleRoad` 等点；第二份 bag 316s-353s 也有同类现象。
- 也有一部分 `Castle` / `OutpostGuard` / `BuffOutpost` 切换发生时没有近期敌方 official fallback，例如第一份 bag 124.281s `Blue Castle`、第二份 bag 308.840s `Blue Castle`。这些不能只凭 `/goal_pose` 判定为 regional defense，可能来自 Outpost visual scout、默认区域任务、progress watchdog fallback 或其他固定点策略。
- 自身位置不会单独生成“敌人入侵回防”事实，但会影响区域任务、到点判断、candidate 排序、progress watchdog 和 Chase official goal 的 self/target 几何。两份 bag 中自身位置实际采用 UWB 源，所以“自身数据”主要解释的是“我在什么区/离哪个点近/是否到点”，不是敌方威胁来源。

当前 bag 没有 decision trace topic，也没有能直接把每一条 `/goal_pose` 绑定到 `DecisionReason` 的记录。因此只能从 topic 侧做强推断：敌方威胁来源如果存在，就是 camera/bridge 的 `navi_target_official`；如果当时没有 fresh enemy fallback，则该 `/goal_pose` 不能归因为敌方数据。

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
- 对外 debug topic：`/ly/navi/reach_state`，使用结构化 `auto_aim_common/msg/GoalReach`，不新增 composite Bool。
- 如果后续必须给导航域发布 Bool：`/ly/navi/reached_composite`，但 Bool 会丢失来源，不建议作为唯一 debug 面。

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

## Implementation Plan

### Phase 1: Add internal reach contract

Files likely touched:

- `src/behavior_tree/include/Application.hpp`
- `src/behavior_tree/src/GameLoop.cpp`

Change:

- Add `GoalReachStatus`, `GoalReachReason`, and `GoalReachState`.
- Add `EvaluateNaviGoalReach(goal_id, goal_position, arrive_distance_cm)` as the single goal-scoped evaluator.
- Keep existing external ROS topics unchanged.

Acceptance:

- `EvaluateNaviGoalReach()` reports external reached, external unreachable, position reached, grace-blocked traveling, stale/unknown separately.
- `IsNaviGoalPositionArrived()` and `IsBaseGoalArrived()` become thin wrappers around `status == Reached`.
- No caller loses current behavior, because the old bool functions still exist.

### Phase 2: Move event snapshot away from raw reached

Files likely touched:

- `src/behavior_tree/include/EventManager.hpp`
- `src/behavior_tree/src/EventManager.cpp`
- `src/behavior_tree/src/GameLoop.cpp`
- `src/behavior_tree/src/DecisionTrace.cpp`

Change:

- Stop deriving `EventSnapshot.GoalReached` directly from `NaviReachFresh && NaviReach`.
- Either rename the raw fields to `ExternalNaviReachFresh/ExternalNaviReached`, or feed the composite state into `EventManager`.
- Extend decision trace with composite reach fields.

Acceptance:

- Trace can show: raw external reached false, position reached true, final status reached.
- Old-goal reached cannot contaminate current-goal reached.
- `EventGoalReached` is either removed/renamed or guaranteed to mean composite reached.

### Phase 3: Refactor Outpost / FaceMode gates to consume state

Files likely touched:

- `src/behavior_tree/src/GameLoop.cpp`
- Possibly `src/behavior_tree/include/FaceModeManager.hpp` only if FaceMode needs explicit reason fields.

Change:

- Compute one `GoalReachState` for `BuffOutpost` in the outpost block.
- Use `state.status == Reached` for point reached.
- Use `state.distance_cm <= VisualScoutFaceDistanceCm` for face-distance readiness.
- Do not let `Timeout` mean physical reached; timeout can release or advance travel, but not directly open physical arrived-only behavior.

Acceptance:

- When `/ly/navi/reached` is always false but `/ly/navi/position` enters the face-distance radius, outpost can enter FaceMode / `/ly/vision/mode=3`.
- When position is stale, the system does not false-open FaceMode only because `/ly/navi/reached=false` stayed fresh.
- Damage abort, armor interrupt, cooldown, and post-window scout semantics remain unchanged.

### Phase 4: Add debug surface

Files likely touched:

- `src/behavior_tree/src/DecisionTrace.cpp`
- Optional: `src/behavior_tree/include/Topic.hpp`, message package, publisher wiring.

Change:

- First add trace fields because it is lower risk than adding a new ROS contract.
- Add ROS debug topic only after trace proves the state is useful.

Implemented topic:

- `/ly/navi/reach_state`

Avoid:

- `/ly/gimbal/reached`, because reached is a BT/navigation decision, not gimbal hardware state.

Acceptance:

- A replay/debug session can identify why a goal is traveling/reached/unreachable/timeout from one state record.
- The record includes goal id, goal position, self position, distance, source freshness, status, and reason.

### Phase 5: Tests and runtime verification

Recommended tests:

- Small C++ unit-style test for pure reach evaluation. If `Application` is too heavy, extract the pure evaluator into a small helper struct/function first.
- Test cases:
  - current-goal external reached true -> `Reached`.
  - current-goal external reachable false -> `Unreachable`.
  - external reached false + grace active + position near -> `Traveling`.
  - external reached false + grace expired + position near -> `Reached`.
  - stale position + external false -> `Traveling` or `Unknown`, not `Reached`.
  - timeout -> `Timeout`, not `Reached`.

Runtime verification:

- `ccb --packages-select auto_aim_common behavior_tree --allow-overriding auto_aim_common`
- `./scripts/selfcheck.sh sentry --static-only`
- replay or inspect the two 2026 bags and confirm trace shows position-based reached where `/ly/navi/reached` remains false.

## Implementation Update

Updated: 2026-05-28

本次改动把 reached 链路收敛成一个内部 `GoalReachState`，并通过 `/ly/navi/reach_state` 发布 `auto_aim_common/msg/GoalReach`。原始 topic 保持不变：

- `/ly/navi/reached`：外部导航 reached 源。
- `/ly/navi/reachable`：外部导航 reachable 源。
- `/ly/navi/reach_state`：BT 内部 composite reach state，用于调试和统一消费。

命名约定：

- `ReachStatus` / `GoalReachStatus` 是状态枚举值：`UNKNOWN`、`TRAVELING`、`REACHED`、`UNREACHABLE`、`TIMEOUT`。
- `GoalReachState` / `GoalReach` 是完整状态对象，包含 status、reason、goal id/坐标、goal age、external reached/reachable freshness、融合坐标、distance、arrive/face distance、grace、timeout。

当前消费链路：

- `IsNaviGoalPositionArrived()` 和 `IsBaseGoalArrived()` 只包装 `EvaluateNaviGoalReach(...).Status == Reached`。
- `EventSnapshot.GoalReached` 改为 composite reached，不再由 raw `NaviReachFresh && NaviReach` 直接生成。
- BT blackboard 增加 `NaviGoalReachStatus`、`NaviGoalReachReason`、`NaviGoalReachDistanceCm`、`NaviGoalReachWithinArriveDistance`、`NaviGoalReachWithinFaceDistance`、`NaviGoalReachTimeout`。
- decision trace 新增 `goal_reach_state` 对象。
- Outpost / FaceMode gate 对 `BuffOutpost` 只计算一次 `GoalReachState`：
  - point reached 使用 `status == reached`。
  - face ready 使用 `status == reached || within_face_distance`。
  - timeout 不等于 physical reached，不直接打开 FaceMode。

已验证：

- `git diff --check`
- `ccb --packages-select auto_aim_common behavior_tree --allow-overriding auto_aim_common`

## Issues To Track

### RCH-001: `EventSnapshot.GoalReached` 名字错误

Status: fixed by composite `GoalReachState`.

Current: 表示 raw `/ly/navi/reached` fresh true。

Expected: 表示当前 goal 的内部 composite reached，或改名为 `ExternalNaviReached`。

Acceptance:

- blackboard 不再暴露误导性的 `EventGoalReached`，或其值来自 composite state。
- decision trace 能区分 external reached、position reached、timeout、unreachable。

### RCH-002: Goal freshness 逻辑有两套

Status: fixed for current goal reached/unreachable consumers that now use `EvaluateNaviGoalReach()`. Raw freshness fields remain in trace/event only as source evidence.

Current:

- `GetExternalNaviReachForGoal()` 有 goal id / position / start time 约束。
- `EventManager::Fresh()` 只有时间约束。

Expected: 所有“当前 goal reached/reachable”判断必须走同一个 goal-scoped freshness。

Acceptance:

- `EventManager` 不直接输入 raw `HasNaviReach/NaviReach/LastNaviReachRxTime` 来生成 reached。
- 单元或 selfcheck 能覆盖“旧 goal 的 reached 不能污染新 goal”。

### RCH-003: Reached、done、timeout 三种语义需要拆开

Status: partially fixed. `GoalReachStatus::Timeout` is represented separately and is not treated as physical reached. Some regional task phase timeouts still live in `AreaManager` and should remain explicit phase logic.

Current: 多处只拿 Bool 判断，容易把“可推进”和“物理到点”混在一起。

Expected:

- `Reached` 表示物理/可信到达。
- `Unreachable` 表示外部导航判定不可达。
- `Timeout` 表示保护性推进，不等于物理到点。

Acceptance:

- Outpost FaceMode 只由 physical reached 或 face-distance readiness 打开。
- Regional patrol phase 可以根据 done/timeout 推进，但 trace 必须说明原因。

### RCH-004: Outpost / FaceMode gate 必须消费统一接口

Status: fixed for the `BuffOutpost` visual scout gate.

Current: Outpost 相关逻辑同时使用 `IsBaseGoalArrived()`、`IsBaseGoalWithinDistance()` 和独立 timeout/cooldown 状态。

Expected: Outpost gate 的输入来自同一个 `GoalReachState`，并明确使用哪个字段：

- travel 是否继续：`status`。
- FaceMode 是否打开：`distance_cm <= VisualScoutFaceDistanceCm` 或 `status == Reached`。
- no-target timeout 是否开始：FaceMode 已打开后开始，不依赖 raw `/ly/navi/reached`。

Acceptance:

- replay 中 `/ly/navi/reached` 全 false 但 position 已接近时，能进入 `/ly/vision/mode=3` 和 `/ly/face_mode/target_raw`。
- trace 可看到进入原因是 `distance_within_face_range` 或 `position_reached`。

### RCH-005: 缺少 composite reached debug topic

Status: fixed by `/ly/navi/reach_state`.

Current: 外部只能看到 raw `/ly/navi/reached`，看不到 BT 内部为什么认为到达、未到达或 timeout。

Expected: 增加 debug topic 和 trace 字段。

Acceptance:

- 每个 goal 至少记录 goal id、goal position、self position、distance、external reached/reachable fresh/value、status、reason。
- ROS topic 使用 `/ly/navi/reach_state`，不要使用 `/ly/gimbal/reached`。

### RCH-006: Bag 记录 QoS 会遮蔽 aim 侧证据

Current: `20260517_084309/main` 的 `/ly/aim/armor_targets` 因 recorder RELIABLE vs publisher BEST_EFFORT 没录到消息。

Expected: recorder 对 `/ly/aim/armor_targets` 使用 best_effort QoS override。

Acceptance:

- 新 bag 中 `/ly/aim/armor_targets` count 不为 0。
- debug report 能同时复盘 reached、vision mode、face mode、aim target array。

### POS-001: `/ly/position/data` enemy 字段没有有效坐标

Status: confirmed in both 2026-05-17 bags.

Current: `/ly/position/data` 持续发布 friend/enemy car id，但 enemy x/y 全是 0；BT 不能从这个 topic 得到敌方官方坐标。

Expected: 如果下位机/雷达链路负责官方敌方位置，应在 `/ly/position/data.enemyx/enemyy` 提供非零坐标；否则文档和调试界面必须明确敌方 official fallback 实际来自 `/ly/navi/target_official`。

Acceptance:

- 新 bag 中统计 `/ly/position/data` enemy non-zero count。
- `/ly/enemy/info.position_source` 能区分 `position_data` 与 `navi_target_official`。

### POS-002: `/ly/navi/target_official` 缺少直接 bag 证据

Status: open.

Current: `topics_at_start.txt` 能看到 `/ly/navi/target_official`，但两份 bag 的 sqlite topic 表和 `ros2 bag info` 没有该 topic 消息。只能通过 `/ly/enemy/info.position_source=navi_target_official` 间接证明 BT 收到了 bridge fallback。

Expected: record 脚本必须直接记录 `/ly/navi/target_official` 消息，方便复盘 camera target point 如何变成 official-map enemy position。

Acceptance:

- 新 bag 中 `/ly/navi/target_official` count > 0 when camera target point valid。
- 能把 `/ly/aim/armor_targets`、`/ly/navi/target_official`、`/ly/enemy/info` 按时间串起来。

### POS-003: `/goal_pose` 缺少 decision reason 绑定

Status: open.

Current: `/goal_pose` 只能看到最终 map pose；`/ly/navi/goal_pos_raw` 只能看到 official goal 坐标。没有 decision trace 时，无法确定每条 `Castle` / `HoleRoad` 是 RegionalDefense、Outpost visual scout、Default regional task 还是 progress watchdog fallback。

Expected: runtime bag 应记录 decision trace，至少包含 `decision_intent.reason`、`output_kind`、`output_topic`、`goal_reach_state`、`friend/enemy position source summary`。

Acceptance:

- 新 bag 能按 timestamp 把 `/ly/navi/goal_pos_raw` 或 `/ly/navi/target_rel` 对齐到 `DecisionReason`。
- 对“回防是敌方威胁触发还是默认任务切点”不需要靠推断。
