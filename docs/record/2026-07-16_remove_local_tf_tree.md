# 移除本倉 `tf_tree` fallback

Updated: 2026-07-16

## 決策

本倉不再保留 `tf_tree` ROS package，也不提供 `use_tf_tree`、`tf_tree_params_file` 等 launch 或腳本入口。外部 `sentry_tf` 是正式與調試環境唯一的 gimbal TF provider。

## 原因

`tf_tree` 僅由 `/ly/gimbal/angles` 建立一段本地 gimbal TF，和外部 `sentry_tf` 提供的 TF 範圍重疊。主鏈已固定依賴外部 TF，保留 fallback 會增加重複 TF owner 與錯誤啟動選項的維護成本。

## 影響與不變項

- `navi_tf_bridge` 和 FaceMode 仍只查詢 TF；它們的 topic、message、frame 名稱不變。
- `sentry_all`、FaceMode test、Outpost test 不再能在未啟動外部 `sentry_tf` 的環境獨立補 TF。
- 正式控制鏈不變：外部導航 -> `behavior_tree` -> `/ly/control/*` -> `gimbal_driver`。

## 操作要求

啟動本倉正式或 FaceMode／Outpost 調試入口前，必須先由外部 stack 啟動 `sentry_tf` 並確認 gimbal TF chain 可用。若缺少 `gx_camera` 等 frame，`navi_tf_bridge` 會明確報出外部 TF 缺失。

## 驗收

本記錄隨同 package、launch、腳本、Obsidian 索引和 Understand Anything fallback graph 更新。source ROS Humble、`sentry.common` 與本 workspace 後，五個保留 package build 成功，169 tests 為 0 errors／0 failures／0 skipped，`./scripts/selfcheck.sh sentry --static-only` 為 108 PASS／0 WARN／0 FAIL。
