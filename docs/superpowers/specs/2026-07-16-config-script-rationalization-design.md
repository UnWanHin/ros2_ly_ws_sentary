# YAML 與腳本收斂設計

## Status

Partially superseded on 2026-07-16 — 第一批盤點已完成；`tf_tree` 已依外部 `sentry_tf` 固定可用的確認移除。其餘後續只對有刪除證據的候選或明確確認的介面 migration 改動。

## Context

本 workspace 有 22 份 YAML 與多個 shell 入口。問題不是設定檔數量本身，而是難以從檔名判斷：

- 哪個 package 擁有某個 runtime key；
- 某個 launch/wrapper 實際套用了哪些 YAML，以及最後誰覆蓋誰；
- 某個短 `.sh` 是必要的使用者入口、測試 profile，還是可移除的重複實作。

近期已把 driver 的 navigation direct-debug 從正式 baseline 分離為
`src/gimbal_driver/config/debug_mode.yaml`。後續收斂必須保留這個正式／debug 硬邊界，且不得改變
正式 `/ly/navi/vel -> behavior_tree -> /ly/control/vel -> gimbal_driver` 鏈路。

## Goals

1. 建立來源驗證的 YAML owner、消費者與 precedence 記錄。
2. 移除已由 source、launch、測試與文件交叉證明不再使用的檔案。
3. 將空的 compatibility config 與純轉發 wrapper 視為獨立 migration，而不是未經驗證的順手刪除。
4. 每一批都有可回退的單一提交、文件與驗證證據。
5. 保留 `common.yaml` 作為現場操作 profile；其日誌、rosbag、raw serial 觀測與明確 stack 級
   override 不因形式上的單一 owner 原則而遷出。

## Non-goals

- 不合併語義不同的正式、debug、測試或 calibration profile。
- 不在本工作中重寫 behavior-tree 戰術設定、導航座標或下位機協議。
- 不刪除僅因未被其他 shell script 呼叫、但仍是人可直接使用的命令。
- 不以歷史文件的未更新記錄作為 runtime 檔案可刪除的證據。

## Source-verified inventory

| 類別 | 唯一 owner／角色 | 目前結論 |
| --- | --- | --- |
| `src/gimbal_driver/config/gimbal_driver_config.yaml` | driver 正式串口、下位機、raw serial baseline | 保留；正式 `sentry_all` 只載入此基線。 |
| `src/gimbal_driver/config/debug_mode.yaml` | 單節點 driver direct-debug overlay | 保留；只由 `debug_node.launch.py` 疊加到 driver baseline。 |
| `src/behavior_tree/config/*.yaml` | BT 的區域、任務、巡邏、點位、導航 Rotate 等分區設定 | 保留分區；`OutpostRegionalTest.yaml` 是被 `outpost_regional_test.launch.py` 載入的 debug profile，非死檔。 |
| `src/navi_tf_bridge/config/tf_config.yaml` | bridge runtime 與 official/map calibration | 保留；兩個 node scope 重複 matrix 是漂移風險，需另案設計單一 calibration source。 |
| `navi_calib.yaml`、`tf_*_points_example.yaml` | calibration input 與範例 | 保留；工具與文件仍使用。 |
| `src/tf_tree/config/tf_tree.yaml` | 已移除的本倉 TF fallback | 已於 2026-07-16 刪除；外部 `sentry_tf` 為唯一 gimbal TF provider。 |
| simulator YAML | simulator runtime、asset manifest、visual QA | 保留；由 automated tests 與 offline workflow 使用。 |
| `config/common.yaml` | wrapper 的現場操作 profile，且目前也會變成 BT/driver inline ROS parameter override | 保留；不能直接刪，因其覆蓋優先序目前有效。 |
| `config/base_config.yaml`、`config/override_config.yaml` | 現為空的 launch compatibility layers | 需在第三批遷移；仍被 launch、wrapper、selfcheck 接受。 |

## Script inventory policy

腳本先分為四類；只有第一類在有完整證據時可直接刪除：

1. **已替代且無 consumer 的實作**：例如已刪除的 `navi_vel_chain`，其行為已由 driver direct-debug 接替。
2. **正式／debug launcher**：執行 ROS launch 或提供安全互斥、環境載入、參數轉換；保留直到有等價官方入口與實跑驗證。
3. **薄 wrapper／別名**：例如 `scripts/start/`、部分 `scripts/debug/`、區域快捷 wrapper。它們沒有重複實作，但是否保留取決於是否承諾既有 CLI。
4. **測試與校準工具**：有特定輸入、topic 或安全限制；不得以檔案短小或目前未被其他 script 呼叫判定為冗餘。

## Three phases

### Phase 1 — inventory and record

建立一份 `docs/reports/` 的現況表，列出每份 YAML 的 owner、正式/debug/test/calibration 分類、source consumer、有效 precedence，以及每支 shell 的類別、入口與 evidence。此階段只加文件與圖譜註記；不改 runtime。

### Phase 2 — evidence-backed deletion

對每一個候選項採四項 gate：

1. `rg` 找不到 source、launch、wrapper、test、package install 或現行文件 consumer；
2. `git log --follow` 沒有仍需保留的相容性承諾；
3. 沒有替代缺口，或替代入口已做 runtime 驗證；
4. 更新 script index、module docs、Obsidian／Understand Anything graph 後，static selfcheck 與 targeted checks 通過。

每個語義獨立的刪除群組單獨提交。

### Phase 3 — interface migration

只在明確確認後處理：

- 空 `base_config.yaml`／`override_config.yaml` 的 launch argument 被 deprecated 或移除；
- 薄 wrapper 收斂為一份官方 CLI，並保留或移除 alias。

這一批改動 launch argument 與運行 precedence，必須先寫 migration note、列出 old/new command 對照，並跑 launch/selfcheck 驗證。

## Precedence rules to record, not assume

沒有單一全專案 YAML precedence。第一批必須逐入口記錄：

- `sentry_all.launch.py`：package config file → launch parameter list → wrapper 從 `common.yaml` 解析出的 inline parameter；後者作為現場操作 profile 可覆蓋前者，且此順序必須保留在文件與 launch log。
- `debug_node.launch.py`：driver baseline → `debug_config_file` → explicit CLI parameter。
- navi bridge：`tf_config.yaml` → launch argument / selected BT JSON 解析出的 inline override。
- 直接 launch 與 wrapper 的預設路徑可能不同：前者可讀 installed package share，後者可傳 source workspace 的絕對路徑；盤點必須把這點列出。

## Verification

- Phase 1：YAML/JSON syntax、靜態 consumer table、`git diff --check`、`./scripts/selfcheck.sh sentry --static-only`。
- Phase 2：上述檢查加上受影響 shell 的 `bash -n`、`ros2 launch --show-args`、targeted build；若是 runtime chain 則加啟動 selfcheck 或說明硬體限制。
- Phase 3：上述所有檢查，並比較 old/new precedence 的 launch logs 或 parameter dump。

## Records

- 現況盤點放 `docs/reports/`，帶 `Updated: YYYY-MM-DD`。
- 每次決策更新最近的 module/script docs 與 `.understand-anything/` graph；圖譜只記錄已 source-verified 的 runtime relation。
- 不把歷史 `docs/record/` 改寫成當前真相；若被新設計取代，新增明確的 current-status note。
