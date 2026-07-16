---
tags:
  - config
  - scripts
  - governance
  - source-audit
status: source-checked
updated: 2026-07-16
---

# YAML／腳本治理盤點

本 note 是目前 YAML owner、launch precedence 與 shell 入口治理的人工導覽。權威盤點是
[[docs/reports/2026-07-16_config_script_inventory|2026-07-16 config/script inventory]]；
它以 source、launch、工具與現行測試為準，不把「沒有被另一支 script 呼叫」誤判成死檔。

## 關聯

- [[docs/obsidian/Home|知識庫首頁]]
- [[docs/obsidian/_generated/Packages/gimbal_driver|gimbal_driver]]
- [[docs/obsidian/_generated/Packages/behavior_tree|behavior_tree]]
- [[docs/obsidian/_generated/Packages/navi_tf_bridge|navi_tf_bridge]]
- [[src/gimbal_driver/config/gimbal_driver_config.yaml|driver 正式 config]]
- [[src/gimbal_driver/config/debug_mode.yaml|driver direct-debug profile]]

## 三批治理

1. **盤點與記錄**：記錄每個 YAML 的 owner、consumer 與按入口區分的 precedence；分類每支 shell
   為 launcher、工具、shared library 或薄 wrapper。
2. **有證據的刪除**：只有 source、launch、test、package install、現行文件和 git history 都證明
   沒有 consumer，且替代入口已驗證時，才建立獨立刪除提交。
3. **介面 migration**：才處理 empty compatibility config、薄 wrapper alias 與 calibration
   source 去漂移；每項先給 old/new CLI 對照和回退方式。

## 目前不可刪的項目

- config/base_config.yaml 與 config/override_config.yaml 雖是空 parameter map，仍是多個 launch、
  wrapper 與 selfcheck 接受的 CLI 介面。
- config/common.yaml 是現場操作 profile：日誌、rosbag、raw serial 觀測與少數 stack 級
  override 由 wrapper 轉成最後一層 inline ROS parameter。這個覆蓋關係應被記錄與維持，
  不應為了形式上的單一 owner 而移除。
- 薄 wrapper 仍是既有的直接命令／dispatcher 入口；它們不是重複 runtime 實作。
- [[src/behavior_tree/config/NaviRotateControl.yaml|NaviRotateControl]] 屬正式 BT 仲裁；
  [[src/gimbal_driver/config/debug_mode.yaml|debug_mode]] 屬單節點 driver direct-debug，不能合併。
  `SetPostureToMoveWhenFalse` 只由 BT 的 `PostureManager` 消費；`debug_node` 不啟動 BT，故 driver profile 不應也不能提供此 key。

## 後續追蹤

- 下一個實際 cleanup 候選應以無 consumer 的檔案或已確認廢棄的 alias 為準；現有 common.yaml
  不是刪除候選。
- Base.yaml 與 AreaManager.yaml 的同值 PatrolSelection overlay 已去重；共用評分由
  AreaManager.yaml 擁有，Base.yaml 保留 MyBase 專屬巡邏權重與保持時間。
- 六支 gimbal 測試／巡邏腳本已共用
  [[scripts/lib/gimbal_test_lifecycle.sh|gimbal_test_lifecycle]]；它只統一 driver lifecycle，保留現有
  debug 入口、topic stimulus、raw serial override 與各腳本額外 PID 回收。
- regional area test 的四組 normal/pure JSON 已收斂為
  [[src/behavior_tree/Scripts/ConfigJson/regional/test/regional_area_template.json|regional_area_template]]
  加 [[scripts/areatest/regional_area_profile.py|regional_area_profile]]。wrapper 只在啟動前產生 temporary
  JSON，沒有向 production loader 加入 inheritance/overlay；PreRoadland 及其餘 debug profile 仍是獨立檢視。
- tf_config.yaml 的兩份 raw goal calibration matrix 與 pointer solver 的 raw config reader
  需要獨立設計單一 calibration source，不在本盤點順手修改。
