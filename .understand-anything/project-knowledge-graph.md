# ros2_ly_ws_sentry Knowledge Graph

Generated: 2026-07-19T06:20:00+00:00

Checked against committed runtime source HEAD: `10aa6a7` (graph artifact commit follows; the user-owned docs/rules lock file remains untracked)

Current graph shape: 92 nodes, 119 edges, 6 layers.

Current ROS packages covered by graph:

- `auto_aim_common`
- `behavior_tree`
- `gimbal_driver`
- `navi_tf_bridge`
- `simulator`

前哨強化交戰鎖：`/ly/enemy/op_hp` 的新鮮正值與 target 7 建立鎖；HP 下降只可 arm 一次強化進攻 `4`。`PostureManager` 仍是唯一 `/ly/control/posture` owner，並以 `posture=1 && enhanced_posture=true` 確認強攻。

```mermaid
flowchart LR
  AIM[external sentry.aim] -->|/ly/aim/armor_targets + result| BT[behavior_tree]
  AIM -->|/ly/control/trajectory GimbalTrajectory| GD[gimbal_driver]
  BT -->|/ly/aim/select_target| AIM
  BT -->|/ly/control/angles firecode vel posture sentry_cmd| GD[gimbal_driver]
  GD -->|/ly/gimbal/state GimbalState| AIM
  GD -->|serial 0x00~0x05| LOWER[lower controller / referee]
  LOWER -->|gimbal + referee state| GD
  GD -->|/ly/gimbal/* /ly/game/* /ly/friend/*| BT
  LOWER -->|TypeID 9 /ly/game/map_command| BT

  BT -->|/ly/navi/goal + speed_level| NAV
  BT -->|goal_pos_raw / target_rel / MapCommand raw cm| BRIDGE[navi_tf_bridge]
  BT -->|FaceMode request| FACE[FaceModeManager]
  FACE -->|/ly/face_mode/target_raw| FACE_SOLVER[map_aim_point_node]
  FACE_SOLVER -->|/ly/face_mode/angles| FACE
  FACE -->|single decision| BT
  BRIDGE -->|/goal_pose| NAV[external navigation]
  NAV -->|reached / reachable / path| BT
  NAV -.debug_node only:\n/ly/navi/vel + should_rotate.-> DEBUG_VEL[navi_vel_to_control_vel]
  DEBUG_VEL -.100 Hz /ly/control/vel + firecode.-> GD
  NAV -->|/ly/navi/path map/m + stamp| PATH[map_path_to_game_path_node]
  PATH -->|/ly/game/path official dm + stamp| GD

  TF[external sentry_tf] -.sole gimbal TF provider.-> BRIDGE
  BT -->|DecisionTrace JSONL| SIM[simulator]
```

## Outputs

- JSON graph: `.understand-anything/knowledge-graph.json`
- Scan inventory: `.understand-anything/intermediate/scan-result.json`
- Metadata: `.understand-anything/meta.json`
- Read-only dashboard: `python3 scripts/understand_graph_dashboard.py` -> `http://127.0.0.1:1037/`
- Detailed project graph: `docs/architecture/2026-07-12_project_link_graph.md`
- Detailed Regional graph: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`

## Notes

- 正式目標來源只有外部 `/ly/aim/armor_targets` 與 `/ly/aim/result`；BT 不再訂閱舊內部輔瞄 topic。
- FaceMode 的 Regional、Buff、Outpost 請求統一由 `FaceModeManager` 收集並仲裁；最終角度/FireCode 仍只由 BT 的單一控制出口發布。
- `PreRoadland`、`ReadyRoadland` 是正式同級 MainArea：前者走 ID 25 的 MyPreRoadland 到點保持；後者使用原 ReadyRoadLand 邊界，保留 ID 21/22 的 MyReadyRoadland 強綁定穿越。舊 `Roadland` alias 與未使用的 `RoadlandFollow` helper 已移除。正式 `regional_competition.json` 把兩者都列為 Default 可選區域；兩者設定分別由 `AreaManager.RegionalAreaTask.MyPreRoadland/MyReadyRoadland` 所有。RegionalDefense 將前後段聚合為 RoadCorridor 防守覆蓋。Simulator 會分別繪製兩個正式主區。`UnitInfo.area_id` 新增 8/9 表示敵我 PreRoadland，既有 0-7 不變。
- Default 只會從各 BT JSON 的 `DecisionAutonomy.NaviGoal.MyArea/EnemyArea/CommonArea` 已啟用區域中評分；仍有替代候選時，剛選過的區域延後到本輪最後。若 MyBase/MyPreRoadland/CommonCentral 等 yieldable Default task 被 Buff/Outpost、RegionalDefense 或 Special 暫時搶占，會記為 `preempted`，下一次只要仍 eligible 就先續走原區域；首次恢復選點若不 eligible 則立即丟棄該恢復權，Recovery、timeout、unreachable、unhealthy 也維持終止。MyBase route 是程式固定的四個 Castle 點、每點保持 15 秒；`Base.yaml`／`base_strategy_config_file` 與 MyBase `Patrol.GoalWeights` 注入已移除。`BuffOutpost`、`HoleRoad`、`OutpostGuard` 不再由 Default 發布，`BuffOutpost` 只由 Buff/Outpost Tactical 擁有。
- `ChasePolicy` 是 Regional Tactical 的導航授權：只在可讓出的 Default `RegionalAreaTask` 已承諾區域時，接受新鮮官方敵方坐標精確落在同一 `AreaKey`，並由 `Chase.yaml` 開啟該 planned area。缺失/過期座標、邊界外或 nearest fallback、異區、未開啟區域一律不追；拒絕只清本拍 chase 輸出，原 Default goal 持續。Chase 在 Tactical，位於目前預設關閉的 Special 之前；League/Showcase 保留既有 area-scope。bridge 的 `Chase.AreaLimit` 解析 Base、Highland、PreRoadland、ReadyRoadland 與 Central 共 9 個現行主區，舊 `roadland` token 只兼容 ReadyRoadland。
- `Task.OutpostConfirm.OpeningHoldUntilWindowEnd=true` 時，`OutpostOpeningHold.hpp` 以 `[0, OpeningHoldSec)` 作為唯一 hard hold 邊界；預設 `OpeningHoldSec=120` 在前哨 safety gate 合格時固定 BuffOutpost navigation ownership 到第 120 秒前，Default、普通巡邏與 soft tactical 不可換點。hard hold 的 priority 不依賴 `OpeningHighPriority`，後者只控制非 hold 的一般開局時間窗。前哨已毀、不可達、受擊/資源安全 gate、Hard Recovery 與己方 Base 的 RegionalDefense 硬威脅保留接管權。
- `GoalReachState` 是到達/不可達的唯一 final contract：raw `/ly/navi/reached`、`/ly/navi/reachable` 和融合坐標只提供證據。EventManager、regional area task、recovery 和 navigation watchdog 都只消費 composite status；watchdog 已移除 raw fallback 與獨立 140 cm 到達半徑。Default 的 MyHighland、MyPreRoadland、MyReadyRoadland guard 和每一個 CommonCentral 巡邏點均和 MyBase 一樣保持 15 秒；純行進 phase 不加駐留。
- `TypeID=9` 的裁判 `0x0303 map_command_t` 以 `/ly/game/map_command` 進 BT。`Task.MapCommand` 只接受官方場地內的坐標模式點，預設持有 45 秒並在 20cm 內去重；它直接發布 `/ly/navi/goal_pos_raw` 官方厘米坐標給 bridge，不直接發布 `/goal_pose`。同點的 5x/100ms 與 1Hz 重送不續期；目標機器人模式沒有坐標，不導航。MapCommand 高於 Default/Tactical/Special，整個 Hard 層（Recovery 與 ReadyRoadland 不可中斷穿越）可取消它。其持有期間清除舊 BaseGoal 的外部 status binding 並不發布 `GoalReachState`，避免任意坐標污染區域到點契約。
- `auto_aim_common` 是正式共用介面包：`GoalReach` 用於 reached 狀態，`RelativeTarget` 用於導航追擊。
- `TypeID=10` 提供 `sentry_info_3` 和精確敵我前哨血量；TypeID=1 的 `GameCode * 25` 只作 fallback。
- `DownlinkTypeID=0x00~0x05` 為控制、SentryCmd、0x0307 路徑、0x0308 自訂訊息、自身座標與 MPC trajectory；0x02 每次以相同 sequence 的兩段 64B CRC16 fragment 傳送，重組後仍是完整 107B / 50 點路徑。TypeID 11 动态反馈与 TypeID 0 角度组合为 `/ly/gimbal/state`，driver 使用 SensorData QoS、拒绝非有限 trajectory，并默认每 20ms 周期发布状态。
- `gimbal_driver` 的串口/下位機基線集中在 `src/gimbal_driver/config/gimbal_driver_config.yaml`；正式 `sentry_all` 透過 `gimbal_driver.launch.py` 載入它，並只把 root `base_config_file`／`config_file` 的安全 `io_config` 相容鍵路由給 driver，絕不把全域 YAML 注入 BT、導航或 FaceMode。
- `io_config.serial_mode=true` 時，逐 ID raw 觀測 topic 為 `/ly/upload/typeid0..11` 與 `/ly/download/typeid0x00..05`；語義 topic 保持不變。
- `src/gimbal_driver/config/debug_mode.yaml` 是 `debug_node.launch.py` 載入的 bridge profile；內建 `navi_vel_to_control_vel.py` 以 100 Hz 將 `/ly/navi/vel` 轉為正式 `/ly/control/vel`、將 `/ly/navi/should_rotate` 轉為 partial `/ly/control/firecode`，500 ms stale 時發布零速度。driver 保持正式 control subscriber，兩類控制最終以同一 `GimbalControlFrame` 下行；此入口不啟動 BT，故不消費也不宣告 `SetPostureToMoveWhenFalse`。正式 root 相容路由明確略過 `navigation_test`／`navigation_mode`。
- 正式 BT 的 `src/behavior_tree/config/NaviRotateControl.yaml` 可透過 `SetPostureToMoveWhenFalse` 讓新鮮 `/ly/navi/should_rotate=false` 僅覆蓋當拍目標姿態為 Move；現有 `PostureManager` 繼續獨佔 cooldown、hold、feedback 與 pending/retry。若冷卻等待中訊號轉 true 或過期，下一拍恢復原策略，因此不補發 Move。這是正式 BT key，不是 driver debug profile key。
- 2026-07-16 source audit 移除本倉 `tf_tree` fallback；外部 `sentry_tf` 是唯一 gimbal TF provider。aim feedback、`/ly/control/sentry_cmd`、FaceMode solver、BT 0x04 position downlink、導航 feedback、`debug_node -> debug_mode.yaml` profile 與 root gimbal compatibility routing 保持不變；source ROS Humble、`sentry.common` 後，本次 `behavior_tree` 與 `simulator` 184 tests 及 static selfcheck（108 PASS／0 WARN／0 FAIL）均通過，完整 external aim runtime graph 驗證仍未執行。
- 本圖譜為 source-checked fallback；因本機沒有可用 Understand Anything plugin core，未執行 plugin regeneration。MPC 的 ROS schema 由本倉 `gimbal_driver/msg/GimbalState.msg` 與 `GimbalTrajectory.msg` 定義；外部 `sentry.aim` 只經既有 topic 消費狀態、發布軌跡，不是本倉建置依賴。
