# 到達判定與 Default 駐留收斂

Updated: 2026-07-19

## 決定

`GoalReachState` 是本倉所有正式「已到達／不可達」決策的唯一入口。`/ly/navi/reached`、
`/ly/navi/reachable` 和自身融合坐標只作為它的輸入證據；它們不再能由 EventManager、
regional area task、recovery 或 navigation watchdog 直接改變任務狀態。

區域任務的目前目標輸入統一命名為 `IsCurrentGoalArrived` /
`IsCurrentGoalUnreachable`，取代會誤導成只對 Base 有效的
`CurrentBaseGoalArrived` / `CurrentBaseGoalUnreachable`。

navigation watchdog 移除獨立的 `ArriveDistanceCm=140` 判定。它和其他消費者使用
`DecisionAutonomy.NaviGoal.HighlandCompat.ArriveDistanceCm` 的同一個 composite 結果；
watchdog 僅根據持續位移、不可達與 timeout 進行保護性 fallback。

## Default 駐留

以下實際到點或 guard phase 預設均為 15 秒：

- MyBase 的每個 Castle 點；
- MyHighland 的 HighlandPatrol 與 BuffShootHold；
- MyPreRoadland 的 GoalHold；
- MyReadyRoadland 的 GuardHold；
- CommonCentral 的每一個巡邏點。

Highland approach/leave、ReadyRoadland approach/cross/return 等行進 phase 維持原本不強加駐留的
語義。不可達或 travel timeout 可照既有任務規則跳過駐留，並不被 15 秒 timer 阻塞。

## 驗證

- RED：raw `reached=true` 會錯誤抑制 watchdog fallback 的回歸測試失敗。
- GREEN：`GoalReachState` 驅動的 watchdog 測試、EventManager raw-observation 測試，以及
  CommonCentral 15 秒停留測試通過。
- `colcon build --packages-select behavior_tree --symlink-install --executor sequential`
- `ctest --test-dir build/behavior_tree --output-on-failure`：7/7 通過。
- `./scripts/selfcheck.sh sentry --static-only`：109 PASS、0 WARN、0 FAIL。
