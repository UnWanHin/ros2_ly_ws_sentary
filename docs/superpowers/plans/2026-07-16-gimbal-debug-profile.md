# Gimbal 單節點 Debug Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 將 navigation 直連調試完全隔離出正式 gimbal 基線與 `sentry_all`，並提供可延伸的單節點 `debug_node` 入口。

**Architecture:** 正式 gimbal 設定由 package-owned baseline 與明確 launch 覆蓋構成；全域 BT override 不再進入 gimbal。`debug_node.launch.py` 只組合 baseline 和一份明確 debug profile，因此新增 aim 等單節點 profile 時不需改正式入口。

**Tech Stack:** ROS 2 Humble launch Python、ROS 2 parameters YAML、colcon、Bash static selfcheck。

## Global Constraints

- 保持 `/ly/*` topic、訊息與既有正式控制語義不變。
- 保留 `main.cpp` 對舊 `io_config.navigation_test` 的讀取相容性，不新增 runtime fallback。
- `debug_node` 預設只啟動 `gimbal_driver`，且 profile 只由 package config 擁有。
- 修改 launch／設定後更新 module docs 與 Understand Anything graph，並驗證 JSON。

---

### Task 1: 建立設定邊界與單節點 debug 入口

**Files:**

- Modify: `src/gimbal_driver/config/gimbal_driver_config.yaml`
- Modify: `src/gimbal_driver/config/navigation_test.yaml`
- Create: `src/gimbal_driver/launch/debug_node.launch.py`
- Test: `scripts/selfcheck/sentry.sh`

**Interfaces:** `debug_node.launch.py [debug_config_file:=PATH] [use_virtual_device:=BOOL]` 依序載入 baseline、profile、CLI 覆蓋。

- [x] 寫出會失敗的 static contract：baseline 不含 navigation debug key，profile 含完整 `navigation_mode` 設定。
- [x] 執行 `./scripts/selfcheck.sh sentry --static-only`，確認現狀因 baseline 含 debug key 而失敗。
- [x] 從 baseline 移除 debug key；新增 `debug_node.launch.py`，透過既有 `gimbal_driver.launch.py` 載入 baseline、`debug_config_file`、raw 診斷與虛擬串口參數。
- [x] 再次執行 static selfcheck，確認通過。

### Task 2: 封閉正式入口的 gimbal 參數來源

**Files:**

- Modify: `src/behavior_tree/launch/sentry_all.launch.py`
- Test: `scripts/selfcheck/sentry.sh`

**Interfaces:** formal gimbal `parameters=[gimbal_driver_config_file, explicit formal overrides]`；不含 `base_config_file` 和 `config_file`。

- [x] 擴充 static contract，拒絕 formal gimbal node lists 傳入 `base_config_file` 或 `config_file`。
- [x] 執行 static selfcheck，確認現狀失敗。
- [x] 從兩個 gimbal node parameter lists 移除這兩個非 gimbal-owned YAML，保留 module baseline 與既有 runtime overrides。
- [x] 執行 static selfcheck，確認通過。

### Task 3: 更新說明、圖譜與驗證

**Files:**

- Modify: `docs/modules/2026-05-05_gimbal_driver.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`

- [x] 更正 profile Rotate 預設為 `1`；說明 `debug_node.launch.py` 的三層載入順序與不得和 BT 並行。
- [x] 更新 graph 的 gimbal／sentry_all 摘要與 navigation debug edge，維持 source-audit metadata 一致。
- [x] 執行 `source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/source_sentry_env.sh && colcon build --packages-select gimbal_driver behavior_tree && ./scripts/selfcheck.sh sentry --static-only && python3 -m json.tool .understand-anything/knowledge-graph.json >/dev/null && python3 -m json.tool .understand-anything/meta.json >/dev/null && git diff --check`。
- [x] 檢視 staged diff，提交 `gimbal_driver: isolate standalone debug profile`。
