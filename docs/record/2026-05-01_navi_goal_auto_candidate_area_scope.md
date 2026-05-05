# Navi Goal 自动候选与区域归类封装记录

日期：2026-05-01

## 目标

在不改 topic/消息链路的前提下，补齐两点：

1. `UseCustomCandidates=false` 时，内建候选支持自动扩展（新点可自动进入评估）。
2. `UseAreaScope=true` 时，若点不落入任何主区，自动归入最近主区再参与范围判定。

## 改动文件

- `src/behavior_tree/src/GameLoop.cpp`

## 具体改动

1. 新增主区解析封装 `ResolveGoalMainArea(...)`
   - 先按 `Base/Highland/Roadland/Central` 判断是否落区。
   - `Central` 是公共区，不再按 `MyArea` / `EnemyArea` 分敌我，而是由 `CommonArea.Central` 统一控制。
   - 若不落区，按点到各主区边界质心的距离，归入最近主区（nearest fallback）。

2. 更新 `IsNaviGoalAllowedByAreaScope(...)`
   - 由“点在 allowed 区域内”改为“点所属主区在 allowed 区域内”。
   - `Base/Highland/Roadland` 仍按 `MyArea` / `EnemyArea` 过滤；`Central` 按 `CommonArea` 过滤。
   - 当触发最近主区回退时，输出 debug 日志，便于联调确认。

3. 内建候选生成改为封装函数 `BuildBuiltinNaviGoalOptions(...)`
   - 保留原有内建候选作为兼容基础。
   - 自动补入当前合法 goal id 范围内的缺失点位（去重）。
   - 保留 `Home/Base/Recovery` 为保留非战斗点，不自动补入。
   - `Protected` 只自动补我方点；`HitHero/HitSentry` 自动补我方和敌方点，并默认排除主区为 `Base` 的自动补点，避免后场偏置。

## 链路影响

- 不改已有发布/订阅 topic。
- 不改消息类型。
- `UseCustomCandidates=true` 行为不变（仍完全按 JSON 自定义候选）。
- `UseCustomCandidates=false` 时，候选覆盖面变大，能自动纳入新增点位（前提是该点位有合法 `goal_id` 和 `GoalPointByBaseId` 映射）。

## 验证

- 增量构建通过：
  - `cmake --build build/behavior_tree --target behavior_tree_node -j1`
