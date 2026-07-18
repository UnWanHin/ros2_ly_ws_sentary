---
tags:
  - sentry
  - ros2
  - 知識庫入口
---

# 哨兵工程知識庫首頁

Updated: 2026-07-19

這是 `ros2_ly_ws_sentry` 的 Obsidian 導覽入口。先從來源掃描得到的節點進入，再回到現有架構文件確認目前行為；人工判斷與現場經驗則保存在獨立筆記區。

## 從這裡開始

- [[docs/obsidian/_generated/Index|自動索引]]：依 package manifest、訊息定義、source、launch 與 config 生成的 package／topic／message 目錄。
- [[docs/obsidian/notes/2026-07-16-source-baseline|2026-07-16 source baseline]]：開始工程前的 source-backed 架構、驗收與追蹤基線。
- [[docs/obsidian/notes/2026-07-16-config-script-governance|YAML／腳本治理盤點]]：owner、precedence 與分批清理 gate。
- [[docs/record/2026-07-19_composite_arrival_default_holds|到達判定與 Default 駐留收斂]]：GoalReachState 唯一入口與 Default 15 秒駐留範圍。
- [[docs/record/2026-07-18_default_area_patrol|Default 區域巡邏收斂]]：Default area eligibility、四 Castle Base route 與 BuffOutpost tactical ownership。
- [[docs/obsidian/notes/2026-07-16-rmuc-v2-enhanced-postures|RMUC V2.0 強化姿態規則]]：強化姿態的官方時限、效果、串口映射與尚未啟用的策略邊界。
- [[docs/architecture/2026-07-12_project_link_graph|現行主鏈]]：目前 decision-only 架構、內外部責任與重要資料鏈路。
- [[docs/obsidian/notes/README|人工筆記]]：不受同步器管理的調查、決策、實機觀察與待辦脈絡。

## 依問題導覽

### 我想理解目前系統怎麼跑

- [[docs/architecture/2026-07-12_project_link_graph|專案主鏈與外部邊界]]
- [[docs/sentry/internal/ros2_topic_structure|ROS2 topic 結構]]
- [[docs/sentry/internal/ros2_topic_tree|ROS2 topic 樹]]
- [[docs/sentry/README|哨兵專項文件入口]]

### 我想查一個 ROS package 或介面

- 從 [[docs/obsidian/_generated/Index|自動索引]] 進入對應 package、topic 或 message；每個生成節點都保留來源位置與可反向跳轉的關係。
- [[docs/modules/2026-05-05_behavior_tree|behavior_tree]]：決策與輸出。
- [[docs/modules/2026-05-05_gimbal_driver|gimbal_driver]]：串口、下位機資料與控制輸出。
- [[docs/modules/2026-05-04_navi_tf_bridge|navi_tf_bridge]]：導航／FaceMode bridge。
- [[docs/modules/2026-04-23_auto_aim_common|auto_aim_common]]：共用 message 與介面。

### 我想了解目前哨兵決策、實機或協議

- [[docs/sentry/regional/current_behavior|Regional 當前行為]]
- [[docs/sentry/regional/decision_framework|決策框架]]
- [[docs/sentry/internal/simulator|離線 simulator]]
- [[docs/sentry/embedded/serial_data_mapping|串口資料映射]]
- [[docs/sentry/embedded/downlink_control_frame|下行控制幀]]
- [[docs/guides/2026-05-04_external_topic_boundary|外部 topic 邊界與實機指引]]

## 使用方式

1. 以 repository root 開啟 vault，並先讀 [[docs/obsidian/README|使用與同步說明]]。
2. 從自動索引選擇要追蹤的 package、topic 或 message，使用 Local Graph 查看一跳關係。
3. 遇到需要人工判斷的情況，在 [[docs/obsidian/notes/README|人工筆記]] 新增 note，以 wikilink 連回相關生成節點與既有文件。
4. 當來源改變時執行同步器；若 `--check` 顯示漂移，先確認 source 與正式文件是否已一致，再更新生成內容。

## 事實來源與限制

生成視圖用於導航，不取代 source review。package manifest、message 定義、source、launch 與 config 是本倉事實來源；現有 architecture／module／sentry 文件補充設計脈絡。外部 `sentry.aim`、導航、TF 和下位機僅以與本倉相接的 topic 或協議邊界描述。
