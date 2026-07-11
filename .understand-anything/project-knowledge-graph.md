# ros2_ly_ws_sentry Knowledge Graph

Generated: 2026-07-12T00:00:00+08:00

Checked against HEAD: `cd8cfa54702c30e8931c1593fe50e01afd5969ec`

Current graph shape: 198 nodes, 227 edges, 6 layers.

Current ROS packages covered by graph:

- `auto_aim_common`
- `behavior_tree`
- `buff_hitter`
- `buff_shooting_table_calib`
- `detector`
- `gimbal_driver`
- `navi_tf_bridge`
- `outpost_hitter`
- `predictor`
- `shooting_table_calib`
- `simulator`
- `tf_tree`
- `tracker_solver`

```mermaid
flowchart LR
  AIM[external sentry.aim] -->|/ly/aim/armor_targets| BT[behavior_tree]
  BT -->|/ly/aim/select_target| AIM
  AIM -->|/ly/aim/result follow/fire/yaw/pitch| BT
  BT -->|/ly/control/angles/firecode/vel/posture/sentry_cmd| GD[gimbal_driver]
  PATROLYAML[Patrol.yaml PatrolScan] -->|Mode1/2/3 + TaskOverrides| PATROL[BT PatrolScanSettings]
  PATROL -->|no-target / FaceMode fallback angles| BT
  PATROLTEST[patrolmode_pub.py] -.manual test reads Patrol.yaml.-> PATROLYAML
  PATROLTEST -.test /ly/control/angles.-> GD
  BT -->|/ly/bt/sentry_position PointStamped map m| GD
  NAVI -->|/ly/navi/path nav_msgs/Path map/m + stamp| PATH[map_path_to_game_path_node]
  PATH -->|/ly/game/path MapPath official dm + original stamp| GD
  GD -->|serial downlink 0x00 control / 0x01 sentry_cmd / 0x02 map path / 0x03 custom info / 0x04 coordinate| LOWER[lower machine]
  LOWER -->|referee/gimbal state| GD
  GD -->|/ly/gimbal/* /ly/game/* /ly/friend/*| BT
  GD -->|/ly/game/sentry/info: sentry_info_3 remaining seconds + age| BT
  GD -->|/ly/friend/uwb_pos + /ly/position/data| FUSION[SentryPositionFusion]
  NAVI -->|/ly/navi/position| FUSION
  FUSION -->|fused sentry self position cm| BT

  BT -->|/ly/navi/target_rel /ly/navi/goal_pos_raw| NAVI[navi_tf_bridge]
  NAVI -->|/goal_pose /ly/navi/goal_pos| NAVSTACK[external navigation]
  NAVSTACK -->|/ly/navi/reached /ly/navi/reachable| REACH[Composite GoalReachState]
  FUSION -->|distance fallback| REACH
  REACH -->|EventGoalReached/EventGoalUnreachable + /ly/navi/reach_state| BT
  BT -->|/ly/face_mode/target_raw| FACE[map_aim_point_node]
  FACE -->|/ly/face_mode/angles| BT
  TF[external sentry_tf] -.preferred TF.-> BT
  TFTREE[tf_tree fallback] -.only when use_tf_tree:=true.-> BT

  subgraph Legacy_Debug[Legacy / debug internal auto-aim]
    GD2[gimbal_driver camera/gimbal state] --> DET[detector]
    DET -->|/ly/detector/armors| TRK[tracker_solver]
    TRK -->|/ly/tracker/results| PRED[predictor]
    PRED -->|/ly/predictor/target| BT
    DET -->|/ly/outpost/armors| OUT[outpost_hitter]
    OUT -->|/ly/outpost/target| BT
    BUFF[buff_hitter] -->|/ly/buff/target| BT
  end

  BT -->|DecisionTrace| SIM[src/simulator]
```

## Outputs

- JSON graph: `.understand-anything/knowledge-graph.json`
- Scan inventory: `.understand-anything/intermediate/scan-result.json`
- Metadata: `.understand-anything/meta.json`
- Read-only local dashboard: `python3 scripts/understand_graph_dashboard.py` -> `http://127.0.0.1:8765/`
- Detailed project graph: `docs/architecture/2026-07-12_project_link_graph.md`
- Detailed Regional graph: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`

## Notes

- 正式主鏈是 decision-only。
- `detector/tracker_solver/predictor/outpost_hitter/buff_hitter` 是 legacy/debug，不是 `sentry_all` 正式主鏈。
- 這份圖譜是 package/topic/file 級，不是完整 AST function call graph。
- 本次核對時工作區有未提交改動；圖譜以 source-checked fallback 模式更新，package/module 清單仍與 `src/*/package.xml` 的 13 個 ROS 包一致。
- 2026-07-08 fallback update：新增 composite `GoalReachState` contract、`/ly/navi/reach_state`、三源 `SentryPositionFusion`、`/ly/bt/sentry_position`（`behavior_tree` -> `gimbal_driver`）和初版下行 frame 摘要。
- `AGENTS.md` 現在要求 graph-relevant work 同步檢查 docs 和 `.understand-anything/` freshness，並驗證 JSON、diff 和 selfcheck。
- 2026-07-08 fallback update：補上 `Patrol.yaml` / `PatrolScan.TaskOverrides` 作為雲台巡邏 mode、FaceMode/Outpost fallback 和 pitch offset 的主配置鏈路，並新增 `docs/sentry/regional/patrol_scan_modes.md`。
- 2026-07-08 fallback update：手動 `scripts/gimbal/patrolmode_pub.py` 也對齊 `Patrol.yaml`，`--outpost` 從 `PatrolScan.TaskOverrides.OutpostPitchOffsetDeg` 取值。
- 2026-07-08 fallback update：`PatrolScan.Mode2` 已回到較早 500ms 參數組：`YawStep=1.0`、`YawBoost=1.1`、`YawHalfRange=30.0`、`CenterDrift=-70.0`、`PitchCenter=0.0`、`PitchHalfRange=13.0`、`PitchPeriodMs=500.0`。
- 2026-07-11 fallback update：`TypeID=6 ChassisData` 不再承載姿態兼容回讀，改為承載裁判 `0x0003 game_robot_HP_t` offset 8 的 `damage_difference`，並新增 `/ly/game/damage_difference` topic。
- 2026-07-11 fallback update：新增 `TypeID=10 SentryInfo3AndOutpostHpData`，承載 `0x020D sentry_info_3` 和 `0x0003 ally/enemy_outpost_HP`；`/ly/friend/op_hp`、`/ly/enemy/op_hp` 優先使用 TypeID 10 精確血量，TypeID 1 `GameCode * 25` 只作 fallback。
- 2026-07-11 fallback update：下行改為 `DownlinkTypeID=0x00~0x04` 五種 frame：13B 控制、6B `sentry_cmd`、107B `0x0307` 路徑、36B `0x0308` 自訂訊息、17B 自身座標；新增 `/ly/control/map_path`、`/ly/control/custom_info`。
- 2026-07-11 fallback update：`behavior_tree` 的姿態輪換/弱化判定在 TypeID 10 `sentry_info_3` age 不超過 `Posture.RefereeInfo3FreshMs`（預設 1500ms）時優先使用裁判普通/強化剩餘秒數；本地 `AccumSec` 持續累積，資料缺失或過期立即 fallback。
- 2026-07-12 fallback update：新增全工程與 Regional 細節 Mermaid 圖，並提供零依賴、唯讀的本地 Dashboard。`/ly/navi/speed_level` 只由 BT 作為策略檔位送往外部導航；`/ly/control/vel` 仍固定由 raw 值 * 0.025 換算，不以 speed_level 二次縮放。
- 2026-07-12 fallback update：新增導航 path -> 裁判 `0x0307` bridge。外部 `/ly/navi/path` 是 `nav_msgs/Path`（map/m）；`map_path_to_game_path_node` 使用現有 raw-goal 矩陣反算 official-map dm，保留 `header.stamp` 發到 `/ly/game/path`，`gimbal_driver` 直接下發 `0x02`。`/ly/control/map_path` 僅留手動相容。
