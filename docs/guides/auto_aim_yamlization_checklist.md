# 輔瞄參數 YAML 化改造清單（工程落地版）

目標：把「改一處就生效」落地，減少比賽前臨時改碼風險。  
約束：不改 topic/msg 格式，不改下位機協議字段。

## 1. 現狀總結

- 已主要 YAML 化：
  - `detector`：`src/detector/config/auto_aim_config.yaml`
  - `tracker_solver / predictor / outpost_hitter(PoseSolver)` 透過 `solver_config.*` 讀同一份 YAML
- 未完全 YAML 化：
  - `buff_hitter` 主要讀 `src/buff_hitter/config/config.json`
  - `outpost_hitter` 仍有關鍵硬編碼（彈速/偏置/閾值）
  - `buff_hitter` 有彈速常量硬編碼（22.9）

## 2. 優先級清單

## P0（必做，穩定性直接收益）

1. `outpost_hitter` 發射關鍵參數 YAML 化  
現狀：`muzzle_solver->setBulletSpeed(23.0)`、`pitch_setpoint -= 2.0`、`yaw ±80` 寫死。  
建議鍵：
- `outpost_config.fire.bullet_speed`
- `outpost_config.fire.pitch_bias_deg`
- `outpost_config.fire.max_yaw_delta_deg`
- `outpost_config.fire.time_delay_sec`（目前 `MuzzleSolver` 內部為 `0.10`）

2. `buff_hitter` 彈速來源去硬編碼  
現狀：`main.cpp` 用 `22.9` 常量，未使用配置內 `controller.bullet_speed`。  
建議：
- 先改為讀 `config.json` 的 `controller.bullet_speed`
- 下一步再加 `--params-file` YAML 覆蓋（保留 JSON 回退）

3. `outpost_predictor` 的 `dt` 來源可配置  
現狀：`delta_t` 先算後被強制覆蓋為 `0.015`。  
建議鍵：
- `outpost_config.predictor.dt_override_sec`（可選，預設關閉）
- `outpost_config.predictor.process_noise.*`
- `outpost_config.predictor.measure_noise.*`
- `outpost_config.predictor.chi_square_threshold`

## P1（建議做，調參效率收益）

1. `DirectionJudger` 閾值 YAML 化  
現狀：`ARMOR_DISTANCE_THRESHOLD=0.2`、`MAX_HISTORY_SIZE=10`。  
建議鍵：
- `outpost_config.direction.armor_distance_threshold`
- `outpost_config.direction.history_size`
- `outpost_config.direction.yaw_diff_threshold`

2. `buff_hitter` 全量配置逐步遷移到 YAML  
現狀：核心參數仍在 JSON（補償、符半徑、模型路徑等）。  
建議策略：
- 第一階段：YAML 可選覆蓋（JSON 為默認）
- 第二階段：YAML 為主，JSON 僅兼容

## P2（可選，長期維護）

1. `detector` 模型閾值外置  
現狀：`armor_detector.cpp` 內有 NMS/置信度常量。  
建議鍵：
- `detector_config.model.nms_thresh`
- `detector_config.model.bbox_conf_thresh`
- `detector_config.model.merge_conf_thresh`

## 3. 遷移原則（避免接口風險）

1. 僅新增參數，不刪舊行為；讀不到參數時維持原默認值。  
2. 保持 `dot/slash` 兩套鍵兼容（沿用 detector 現有做法）。  
3. 不改 topic 名、不改 msg 定義、不改串口包結構。  
4. 每完成一項 YAML 化，必須同步更新：
- `docs/guides/auto_aim_tuning_handover.md`
- 對應模塊文檔
- 本清單狀態

## 4. 每項改造驗收模板

1. 代碼驗證  
- `colcon build --packages-select <changed_pkgs>`
- `./scripts/selfcheck.sh pc --no-build`

2. 參數生效驗證  
- 啟動節點時打印「最終生效值」
- 改 YAML 後重啟節點，確認日志中的值變更

3. 鏈路回歸  
- `./scripts/selfcheck.sh sentry --runtime-only --launch --wait 12 --skip-hz`

## 5. 建議執行順序

1. `outpost_hitter` P0 三項  
2. `buff_hitter` 彈速去硬編碼  
3. `DirectionJudger` + `OutpostPredictor` 閾值化  
4. `buff_hitter` JSON -> YAML 覆蓋層  
5. `detector` 模型閾值外置
