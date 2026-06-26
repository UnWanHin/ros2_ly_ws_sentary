# ros2_ly_ws_sentry Knowledge Graph

Generated: 2026-06-26T07:26:03+00:00

```mermaid
flowchart LR
  AIM[external sentry.aim] -->|/ly/aim/armor_targets| BT[behavior_tree]
  BT -->|/ly/aim/select_target| AIM
  AIM -->|/ly/aim/result follow/fire/yaw/pitch| BT
  BT -->|/ly/control/angles/firecode/vel/posture/sentry_cmd| GD[gimbal_driver]
  GD -->|serial downlink| LOWER[lower machine]
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
