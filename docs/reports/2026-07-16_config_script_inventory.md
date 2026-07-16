# YAML 與腳本現況盤點

Updated: 2026-07-16

## 結論與邊界

本輪以目前 source、launch、package install 規則、測試與維護文件交叉盤點了 22 份 YAML
和全部現存 shell 入口。沒有發現可僅憑「檔案未被另一支 .sh 呼叫」就安全刪除的 YAML 或
腳本。許多檔案是人直接執行的 ROS/實機工具、debug profile 或 calibration input，不一定
會被程式碼反向引用。

目前結論：

- 保留：有正式 launch、node、測試、校準工具或離線 workflow consumer。
- 保留並明確記錄：config/base_config.yaml、config/override_config.yaml、
  config/common.yaml 與薄 wrapper；它們仍是公開 CLI／有效 precedence 的一部分。
- 尚無可刪候選：唯一有完整替代與零 consumer 證據的 scripts/navi/navi_vel_chain.{sh,py}
  已在提交 5a4c1c8 移除，由 gimbal_driver/debug_node.launch.py + debug_mode.yaml 取代。

本文件只記錄現況，不改動 runtime、YAML、launch argument 或命令入口。後續刪除必須滿足
「第二批候選與刪除 gate」。

## YAML owner 與 consumer

| YAML | owner／分類 | 已確認 consumer | 結論 |
| --- | --- | --- | --- |
| config/base_config.yaml | workspace compatibility baseline | sentry_all.launch.py、decision_chase.launch.py、navi_control_chain.sh、selfcheck | 保留；目前空值不代表沒有 CLI 合約。 |
| config/common.yaml | wrapper 的現場操作 profile | scripts/launch/start_sentry_all.sh、map_aim_point_test.sh | 保留；集中日誌、rosbag、raw serial 觀測與明確的 stack 級覆蓋。 |
| config/override_config.yaml | workspace compatibility override | sentry_all.launch.py、decision_chase.launch.py、navi_control_chain.sh、selfcheck | 保留；空檔仍提供 config_file 覆蓋入口。 |
| scripts/feature_test/config/sentry_feature_test.yaml | feature-test scenario | scripts/feature_test/run_feature_test.sh | 保留；是該框架的預設輸入。 |
| src/behavior_tree/config/AreaManager.yaml | BT 區域、位置融合正式 baseline | sentry_all.launch.py、behavior_tree.launch.py | 保留。 |
| src/behavior_tree/config/Base.yaml | BT Base／Regional 正式補充 | sentry_all.launch.py | 保留；standalone BT launch 故意不載入它。 |
| src/behavior_tree/config/NaviRotateControl.yaml | BT 正式 /ly/navi/should_rotate 仲裁 | sentry_all.launch.py、behavior_tree.launch.py、chase.sh | 保留；不得與 driver direct-debug 合併。 |
| src/behavior_tree/config/OutpostRegionalTest.yaml | 前哨聯調 overlay | outpost_regional_test.launch.py | 保留；不是正式 baseline。 |
| src/behavior_tree/config/Patrol.yaml | BT 巡邏掃描與 task override | sentry_all.launch.py、behavior_tree.launch.py、patrol/debug scripts | 保留。 |
| src/behavior_tree/config/PointManager.yaml | BT 導航點位 | sentry_all.launch.py、behavior_tree.launch.py | 保留。 |
| src/behavior_tree/config/Special.yaml | BT 特殊區域策略 | sentry_all.launch.py、behavior_tree.launch.py | 保留。 |
| src/behavior_tree/config/Task.yaml | BT 任務、StartGate、FaceMode baseline | sentry_all.launch.py、behavior_tree.launch.py | 保留。 |
| src/gimbal_driver/config/debug_mode.yaml | 單節點 driver direct-debug overlay | gimbal_driver/debug_node.launch.py | 保留；只允許 debug node 載入。 |
| src/gimbal_driver/config/gimbal_driver_config.yaml | driver 串口、下位機、raw serial 正式 baseline | gimbal launch、sentry_all.launch.py、FaceMode launch、driver test scripts | 保留。 |
| src/navi_tf_bridge/config/navi_calib.yaml | calibration point input | kabsch_calib.py、affine_calib.py | 保留。 |
| src/navi_tf_bridge/config/tf_calib_points_example.yaml | calibration 範例 | calibration 文件／人工工具輸入 | 保留；範例不等於死檔。 |
| src/navi_tf_bridge/config/tf_config.yaml | navi bridge runtime 與 official/map calibration | bridge launch、FaceMode／navitomap／座標轉換 scripts | 保留。 |
| src/navi_tf_bridge/config/tf_kabsch_points_example.yaml | Kabsch calibration 範例 | calibration 文件／人工工具輸入 | 保留。 |
| src/simulator/assets/manifest.yaml | simulator asset manifest | simulator package、quality workflow | 保留。 |
| src/simulator/config/default.yaml | simulator 預設場地／視覺設定 | simulator CLI、offline workflow | 保留。 |
| src/simulator/config/visual_asset_qa.yaml | simulator sprite QA profile | quality tests、offline workflow | 保留。 |
| src/tf_tree/config/tf_tree.yaml | tf_tree runtime／FaceMode fallback geometry | tf_tree.launch.py、sentry_all.launch.py、map_aim_point.launch.py | 保留。 |

### 必須保持分離的 YAML

1. gimbal_driver_config.yaml 是正式 serial/lower-machine owner；debug_mode.yaml 只提供
   單節點 direct-debug，不能以載入順序猜測誰生效。
2. NaviRotateControl.yaml 是正式 BT 鏈的導航 Rotate 仲裁；debug_mode.yaml 是 driver
   直連 /ly/navi/*。名稱接近，但 owner 與控制鏈不同。
3. BT 的 AreaManager、Task、Patrol、PointManager、Special 是按決策責任分區，目前不應
   為了減少檔數合併。
4. calibration examples 與 simulator QA profile 有明確人機／測試用途；它們不是 production
   runtime baseline，但仍是受維護資產。

## 有效 precedence（按入口）

沒有一條可套用所有節點的全專案 precedence。下表只記錄 source 已證實的入口，日後變更必須
更新相應列。

| 入口 | 有效載入／覆蓋順序 | 說明 |
| --- | --- | --- |
| ros2 launch behavior_tree sentry_all.launch.py | package config 預設／明確 config file → launch parameter list | 正式 stack；gimbal node 只接收 driver baseline 與正式 launch override。 |
| scripts/launch/start_sentry_all.sh | wrapper 解析 common.yaml → 呼叫 sentry_all.launch.py → wrapper 轉出的 inline ROS parameters 為最後層 | common.yaml 是現場操作 profile；其 firecode、velocity、StartGate、FaceMode key 可明確覆蓋 package YAML。 |
| ros2 launch behavior_tree behavior_tree.launch.py | AreaManager.yaml → Task.yaml → NaviRotateControl.yaml → PointManager.yaml → Patrol.yaml → Special.yaml → CLI | standalone path 不載入 Base.yaml，也沒有 sentry_all 的 inline ExternalAim／StartGate/manual-goal 層。 |
| ros2 launch gimbal_driver gimbal_driver.launch.py | gimbal_driver_config.yaml → config_file（若明確指定）→ CLI | 安全的單節點正式 driver 預設；不帶 navigation direct-debug。 |
| ros2 launch gimbal_driver debug_node.launch.py | gimbal_driver_config.yaml → debug_config_file（預設 debug_mode.yaml）→ explicit CLI | 唯一允許 driver 直連 /ly/navi/vel 與 /ly/navi/should_rotate 的入口。 |
| ros2 launch navi_tf_bridge target_rel_to_goal_pos.launch.py | tf_config.yaml → launch arguments → sentry_all 從選定 BT JSON 解析出的 bridge inline values | BT JSON 的 NaviSetting/Chase 可覆蓋 bridge target-rel 設定。 |
| calibration CLI | navi_calib.yaml 或明確輸入／範例 → CLI | kabsch_calib.py、affine_calib.py 的 point input 與 runtime tf_config.yaml 分開。 |
| simulator CLI | default.yaml 或指定 QA config → CLI | simulator config 不注入 ROS node。 |

## Shell 入口分類

薄 wrapper 是既有命令相容性入口，不等同於重複實作。以下列出目前全部 shell 檔；除第二欄
明確標示的 shared library 外，其餘均可作為人直接執行或 dispatcher 目標。

| 路徑／群組 | 類型 | 現況與刪除判定 |
| --- | --- | --- |
| scripts/start.sh、scripts/debug.sh、scripts/selfcheck.sh | 頂層 dispatcher | 保留；提供官方子命令入口。 |
| scripts/start/sentry_all.sh、scripts/start/sentry_all_nogate.sh、scripts/start/showcase.sh | 薄 wrapper | 保留；需明確 CLI migration 才能淘汰。 |
| scripts/launch/start_sentry_all.sh、scripts/launch/start_sentry_all_nogate.sh、scripts/launch/start_sentry_showcase.sh、scripts/launch/start_sentry_navi_debug.sh、scripts/launch/start_sentry_chase_only.sh、scripts/launch/start_sentry_decision_chase.sh | 正式／debug launcher | 保留；含環境、互斥清理或參數組裝。 |
| scripts/launch/map_aim_point_test.sh | 薄 wrapper | 保留；對舊入口相容，轉發至 scripts/navi/map_aim_point_test.sh。 |
| scripts/aim/Outpost_Simlator.sh、scripts/aim/armor_only_test.sh、scripts/aim/armor_patrol_test.sh、scripts/aim/armor_test.sh、scripts/aim/outpost_regional.sh | aim／前哨聯調 launcher | 保留；各自 launch mode 與安全選項不同。 |
| scripts/areatest/regional_area_test.sh | 區域測試實作 | 保留；四個區域快捷命令的共同 owner。 |
| scripts/areatest/regional_base.sh、scripts/areatest/regional_central.sh、scripts/areatest/regional_highland.sh、scripts/areatest/regional_roadland.sh | 薄 wrapper | 保留；待確認是否需要區域快捷 CLI 時才可評估。 |
| scripts/debug/armor_test.sh、scripts/debug/chase_only.sh、scripts/debug/goal_pos_test.sh、scripts/debug/move_rotate.sh、scripts/debug/navi_debug.sh、scripts/debug/navi_goal.sh、scripts/debug/standalone.sh | 薄 wrapper | 保留；由 debug.sh dispatcher 暴露。 |
| scripts/debug/navi_goal_cli.sh | CLI helper | 保留；包含具體 goal input 行為。 |
| scripts/debug/control_angles_test.sh、scripts/debug/control_sink.sh、scripts/debug/patrolmode3_test.sh、scripts/debug/posture_test.sh、scripts/debug/rotate_level.sh、scripts/debug/sentry_cmd_downlink_test.sh | 下位機／控制測試 | 保留；手動硬體工具未被 script 引用不是刪除證據。 |
| scripts/feature_test/lib/common.sh、scripts/feature_test/lib/guard.sh | shared library | 保留；由 feature-test runner source。 |
| scripts/feature_test/run_feature_test.sh | feature-test runner | 保留；讀取 sentry_feature_test.yaml。 |
| scripts/feature_test/standalone/lib/common.sh | standalone shared library | 保留；由 standalone menu／modes source。 |
| scripts/feature_test/standalone/run_standalone_menu.sh | standalone dispatcher | 保留；載入 mode scripts。 |
| scripts/feature_test/standalone/modes/chassis_spin_mode.sh、scripts/feature_test/standalone/modes/chassis_spin_sine_translate_mode.sh、scripts/feature_test/standalone/modes/chassis_spin_translate_mode.sh、scripts/feature_test/standalone/modes/navi_patrol_mode.sh | standalone modes | 保留；有不同 control stimulus。 |
| scripts/gimbal/patrolmode_common.sh | gimbal patrol shared implementation | 保留；由 mode wrappers source／執行。 |
| scripts/gimbal/patrolmode1.sh、scripts/gimbal/patrolmode2.sh、scripts/gimbal/patrolmode3.sh | 薄 wrapper | 保留；可在 wrapper migration 時再評估。 |
| scripts/lib/ros_launch_common.sh | workspace shared library | 保留；多個 launcher source。 |
| scripts/navi/NaviToOfficial.sh、scripts/navi/OfficialToNavi.sh | calibration conversion CLI | 保留；直接使用 tf_config.yaml。 |
| scripts/navi/chase.sh、scripts/navi/navitomap.sh、scripts/navi/navi_control_chain.sh、scripts/navi/position.sh | navigation／正式鏈調試工具 | 保留；各自覆蓋 chase、目標、正式 BT control、位置觀測。 |
| scripts/navi/facemode.sh | 薄 compatibility wrapper | 保留；維持舊 FaceMode 預設轉交 cross-matrix mode。 |
| scripts/navi/facemode_cross_matrix.sh、scripts/navi/facemode_map.sh、scripts/navi/facemode_official.sh | FaceMode mode wrapper | 保留；共同轉交 map-aim test，但 frame／單位預設不同。 |
| scripts/navi/map_aim_point_attach.sh、scripts/navi/map_aim_point_test.sh | FaceMode 實作／測試 | 保留；前者可 attach，後者組裝 launch。 |
| scripts/selfcheck/pc.sh、scripts/selfcheck/robot.sh、scripts/selfcheck/sentry.sh | selfcheck suite | 保留；由頂層 selfcheck dispatcher 使用。 |

## 第二批候選與刪除 gate

目前第二批候選清單為空。下列項目看似冗餘，但不符合刪除條件：

| 項目 | 為何現在不能刪 | 若日後要處理 |
| --- | --- | --- |
| config/base_config.yaml、config/override_config.yaml | 雖是空 ROS parameter map，仍被 launch、wrapper、selfcheck 接受。 | 先設計 deprecated argument 或移除 migration，列 old/new CLI 對照。 |
| common.yaml 的 driver／BT runtime key | wrapper 會把它轉為最後的 inline override，且這是現場操作 profile 的既有用途。 | 保留；在 common.yaml、啟動文件與本表維持清楚 precedence 說明。 |
| 區域、巡邏、start/debug 薄 wrapper | 是可直接使用的既有 CLI，且有 dispatcher／文件入口。 | 明確決定官方命令集合後，先發 migration note，再刪 alias。 |
| tf_config.yaml 兩段 raw_goal_transform_matrix | 是 node-scoped ROS params，目前兩個 node 都需要。 | 另立 calibration-source 設計，不能以單純文字去重。 |
| OutpostRegionalTest.yaml、examples、simulator QA YAML | 有 launch、校準或測試 consumer。 | 不列候選。 |

每個新候選必須同時通過：

1. rg 對 source、launch、wrapper、test、package install 與現行文件沒有 consumer；
2. git log --follow 沒有仍有效的相容性承諾；
3. 替代入口已存在，且在適當層級完成 build／launch／runtime 驗證；
4. script index、module docs、Obsidian 與 Understand Anything graph 已移除或更新關聯。

## 已知架構風險（不在本批修改）

- common.yaml 同時保存日誌／rosbag／raw serial 觀測與少數 stack 級 runtime override。這是
  合理的現場操作 profile；維護重點是讓其覆蓋關係可見，而非強制把每個 key 移出。
- Base.yaml 的 AreaManager.RegionalAreaTask.PatrolSelection 會在正式 launch 中覆蓋
  AreaManager.yaml 的同一路徑。現有 8 個 leaf（距離、當前點、未訪問、freshness、近期訪問
  的 score knobs）逐值相同，形成雙重真相；可在一個行為保持的獨立切片中從 Base.yaml 移除
  這段重複，只保留 MyBase 專屬 GoalHoldSec 與 GoalWeights。
- scripts/debug/control_angles_test.sh、rotate_level.sh、posture_test.sh、
  patrolmode3_test.sh、sentry_cmd_downlink_test.sh 與 scripts/gimbal/patrolmode_common.sh
  各自重複 gimbal_driver 的啟動、virtual device、等待、PID cleanup 與 trap 樣板。這是
  可抽出 scripts/lib 的共享 lifecycle helper 的候選；保留每支腳本的 topic stimulus 與 CLI，
  不會減少現場可見入口。
- behavior_tree 的 21 份 ConfigJson 約 1930 行，其中 regional test/debug JSON 有刻意的
  完整 profile 副本。現行 loader 沒有 JSON inheritance/overlay 合約；若為減少檔案而加入
  合併機制，會改變 profile 解析與驗收範圍，暫不列為清理候選。
- tf_config.yaml 在 target_rel_to_goal_pos_node 與 map_path_to_game_path_node 各保留一份
  static calibration matrix；兩份資料可漂移。pointer_solver_node.cpp 另有 raw-text 讀 bridge
  config 的行為，應與 calibration source 一起設計，不宜在這次盤點中順手改。
- direct sentry_all launch 使用 installed package share 的 root config，而 wrapper 會傳 source
  workspace 絕對路徑；source YAML 改後未重建時，兩者可觀察到不同版本。
- .understand-anything/project-knowledge-graph.md 的 Checked against HEAD 是舊 audit snapshot；
  下次圖譜 source regeneration 應一併更新 metadata，不能把舊文字當成當前 runtime 證據。

## 驗證來源

- README.md、docs/README.md、docs/agents/domain.md；
- src/behavior_tree/launch/sentry_all.launch.py、
  src/behavior_tree/launch/behavior_tree.launch.py、
  src/gimbal_driver/launch/debug_node.launch.py、
  src/navi_tf_bridge/launch/target_rel_to_goal_pos.launch.py；
- scripts/launch/start_sentry_all.sh、scripts/navi/navi_control_chain.sh、
  scripts/selfcheck/sentry.sh 與各 shell entry point；
- package config、calibration 工具、simulator quality/offline workflow tests；
- rg --files -g '*.yaml' -g '*.yml'、rg --files scripts -g '*.sh' 與 source reference scan。
