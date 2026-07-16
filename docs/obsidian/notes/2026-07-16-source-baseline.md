---
tags:
  - baseline
  - source-audit
  - ros2
status: source-checked-runtime-pending
updated: 2026-07-16
---

# 2026-07-16 工程 source baseline

> 範圍：開始新工程前的可追溯現況，不改 runtime。此 note 以 source、launch、package manifest、既有測試與 fallback graph 交叉確認；ROS runtime graph 尚未驗證。

## 已確認的正式邊界

- 現行 workspace 只有六個 ROS package：[[docs/obsidian/_generated/Packages/auto_aim_common|auto_aim_common]]、[[docs/obsidian/_generated/Packages/behavior_tree|behavior_tree]]、[[docs/obsidian/_generated/Packages/gimbal_driver|gimbal_driver]]、[[docs/obsidian/_generated/Packages/navi_tf_bridge|navi_tf_bridge]]、[[docs/obsidian/_generated/Packages/tf_tree|tf_tree]]、[[docs/obsidian/_generated/Packages/simulator|simulator]]。
- 正式 decision-only 主鏈是外部 aim 的 [[docs/obsidian/_generated/Topics/ly__aim__armor_targets|/ly/aim/armor_targets]]、[[docs/obsidian/_generated/Topics/ly__aim__result|/ly/aim/result]] 進 BT；BT 回授 [[docs/obsidian/_generated/Topics/ly__aim__select_target|/ly/aim/select_target]]，並以單一控制出口發布 [[docs/obsidian/_generated/Topics/ly__control__angles|angles]]、[[docs/obsidian/_generated/Topics/ly__control__firecode|firecode]]、[[docs/obsidian/_generated/Topics/ly__control__vel|vel]]、[[docs/obsidian/_generated/Topics/ly__control__posture|posture]]、[[docs/obsidian/_generated/Topics/ly__control__sentry_cmd|sentry_cmd]] 給 `gimbal_driver`。
- `sentry_all.launch.py` 正式組合 `gimbal_driver`、`behavior_tree`、可選 `navi_tf_bridge`／FaceMode solver／`tf_tree`。外部 `sentry_tf` 是首選；本倉 `tf_tree` 只在 `use_tf_tree=true` 時由雲台角度建立 TF，給導航／FaceMode bridge 使用。
- `navi_tf_bridge` 接收 BT 的 `goal_pos_raw`、`target_rel` 和外部 armor targets；它輸出導航 `/goal_pose`，並把 position／official target 回饋 BT。`/ly/navi/goal` 是導航相容目標，不經此 bridge。

## Regional 與離線契約

- `PreRoadland` 與 `Roadland` 是同級正式 MainArea；`MyPreRoadland` 使用 BaseGoalId 25 的到點保持，`MyRoadland` 保留 ID 21/22 的 guarded crossing。[[docs/sentry/regional/2026-07-12_regional_decision_graph|Regional 決策圖]]與 `test_ready_roadland_area.cpp`、`test_roadland_split_task.cpp` 是目前靜態證據。
- `DecisionTrace` 是 BT 到 [[docs/obsidian/_generated/Packages/simulator|simulator]] 的離線契約；修改決策輸出、導航點位、姿態、target 或 unit state 時，必須同步 simulator model／trace／validation／config 與文件。

## 串口與裁判

- [[docs/obsidian/_generated/Packages/gimbal_driver|gimbal_driver]] 擁有 TypeID uplink 與 DownlinkTypeID 0x00–0x04。BT 自身座標走 `/ly/bt/sentry_position` 到 0x04；完整 `sentry_cmd` 和 posture 走 0x01。
- `serial_mode` 是按 ID raw observability 開關：upload `/ly/upload/typeid0..10`，download `/ly/download/typeid0x00..04`，皆為 `GimbalRawFrame`。這些是動態 family，不由一般字串掃描生成單一靜態 topic note；權威說明是 [[docs/sentry/embedded/serial_data_mapping|serial data mapping]]。

## 圖譜與驗收狀態

- `.understand-anything/` 在本次 source audit 已更新到 HEAD `d4777589`，並補齊 aim feedback、`sentry_cmd`、FaceMode solver、BT 0x04、導航 feedback 和正確 `tf_tree -> navi_tf_bridge` fallback 關係。
- Obsidian generated index 已涵蓋 package、message 與可靜態辨識的 `/ly/...` topic；手寫理解與調查保留在 `docs/obsidian/notes/`。
- 本環境已以 ROS Humble Bash 完成完整 `colcon build`（六個 package 全部成功），並在依序 source ROS Humble、`../sentry.common/install/setup.bash`、本 workspace 後通過 static selfcheck（105 PASS、0 WARN、0 FAIL）。formal aim 的 launch／runtime graph 驗證仍為 **pending**；可用外部 navigation、TF 與下位機／offline substitute 時，再執行 `./scripts/selfcheck.sh sentry --launch --wait 10`。
- GitHub issue workflow 也為 **pending**：repo 規定用 `gh`，但本環境未安裝 `gh`。目前工作樹另有大量 `100644 -> 100755` mode-only noise，開始工程前應先確定它是否為 mount／檔案系統副作用，避免污染 review。

## 本輪再核對（不改 runtime）

- `navi_publish_goal_pose` 是 `sentry_all.launch.py` 的已宣告 launch argument，但現行 bridge include 固定傳入 `publish_goal_pose: "true"`；因此傳 `navi_publish_goal_pose:=false` **目前不會關閉** bridge 的 `/goal_pose` 輸出。後續做離線／導航調參時，應以此為已知行為，待有明確需求再決定是否修正介面。
- 重新完成 `obsidian_sync.py --check`（0 write／0 delete／0 conflict）、其 11 個 pytest 回歸測試、JSON／Python 語法檢查、`git diff --check`，以及 `./scripts/selfcheck.sh sentry --static-only`（105 PASS、0 WARN、0 FAIL）。
- Distrobox 環境已補齊 `python3-pip` 與符合 `src/simulator/requirements.txt` 的 user-site `pygame 2.6.1`（`/usr/bin/python3` 解析到此版本）；Ubuntu 的 `python3-pygame` 亦已安裝作系統基線。`src/tf_tree/src/tf_node.cpp` 已用 ROS Humble 既有 uncrustify 規則格式化，未改行為。
- 在此環境重新跑完整 `colcon build && colcon test && colcon test-result --verbose`：六個 package 全部 build 完成，181 tests、0 errors、0 failures、1 skipped。這取代本 note 先前的 simulator 缺件與 tf_tree formatting 失敗紀錄。
- 工作樹的 mode-only noise 已量化為 473 個 `100644 -> 100755` 變更；本輪只新增本人工 note，未調整任何 ROS runtime／介面／測試原始碼。


## 來源

- [[README|專案入口]]、[[docs/architecture/2026-07-12_project_link_graph|正式架構]]、[[docs/architecture/2026-07-12_project_link_graph|topic／serial 邊界]]
- [[src/behavior_tree/launch/sentry_all.launch.py|正式 launch]]、[[src/behavior_tree/include/Topic.hpp|BT topic contract]]、[[src/behavior_tree/src/PublishMessage.cpp|BT outputs]]、[[src/behavior_tree/src/SubscribeMessage.cpp|BT inputs]]
- [[src/gimbal_driver/main.cpp|driver runtime]]、[[src/gimbal_driver/module/BasicTypes.hpp|downlink frames]]、[[src/gimbal_driver/config/gimbal_driver_config.yaml|serial config]]
- [[src/navi_tf_bridge/src/target_rel_to_goal_pos_node.cpp|navigation bridge]]、[[src/navi_tf_bridge/src/pointer_solver_node.cpp|FaceMode solver]]
