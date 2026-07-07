# ros2_ly_ws_sentry Knowledge Graph

Generated: 2026-07-07T16:58:59+00:00

Checked against HEAD: `84af1f3e758edad06802479b2357102a47c29778`

Current graph shape: 182 nodes, 186 edges, 6 layers.

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
  BT -->|/ly/bt/sentry_position PointStamped map m| GD
  GD -->|serial downlink DownlinkTypeID 0x00 control / 0x01 coordinate| LOWER[lower machine]
  LOWER -->|referee/gimbal state| GD
  GD -->|/ly/gimbal/* /ly/game/* /ly/friend/*| BT

  BT -->|/ly/navi/target_rel /ly/navi/goal_pos_raw| NAVI[navi_tf_bridge]
  NAVI -->|/goal_pose /ly/navi/goal_pos| NAVSTACK[external navigation]
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

## Notes

- 正式主鏈是 decision-only。
- `detector/tracker_solver/predictor/outpost_hitter/buff_hitter` 是 legacy/debug，不是 `sentry_all` 正式主鏈。
- 這份圖譜是 package/topic/file 級，不是完整 AST function call graph。
- 本次核對時工作區有未提交改動；已確認當前 graph module 清單與 `src/*/package.xml` 的 13 個 ROS 包一致。
- 2026-07-08 fallback update：新增 `/ly/bt/sentry_position`（`behavior_tree` -> `gimbal_driver`）和 `DownlinkTypeID=0x00/0x01` 下行 frame 摘要；package/module 清單未重掃，仍以現有 `src/*/package.xml` 覆蓋為準。
