# Regional Roadland 同級拆分 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (recommended) to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 將舊 `Roadland + MiniRoadland Special` 遷移為正式同級 `PreRoadland + Roadland` 區域任務，同時保持導航 goal ID 與 Roadland 穿越鏈路。

**Architecture:** `MainAreaKind::PreRoadland` 新增為正式 area；既有 `Roadland` 名稱保留但使用 ReadyRoadLand 四邊形。ID 25 改名 `PreRoadland` 並成為 `MyPreRoadland` 任務唯一目標；ID 21/22 維持 Roadland 雙向穿越。DefaultStrategyManager 用兩個同級候選與獨立 retry state 評分；不改 ROS、串口或導航 goal ID 合約。

**Tech Stack:** ROS2 Humble、C++20、BehaviorTree.CPP、GoogleTest、YAML/JSON、colcon。

---

### Task 1: 建立正式區域模型與點位歸屬

**Files:**
- Modify: `src/behavior_tree/module/Area.hpp`
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/include/AreaManager.hpp`
- Modify: `src/behavior_tree/src/AreaManager.cpp`
- Test: `src/behavior_tree/test/test_ready_roadland_area.cpp`

- [x] 將 `MainAreaKind` 擴展為 `PreRoadland`，使新的 `Roadland` boundary 直接使用原 ReadyRoadLand polygon。
- [x] 移除 MiniRoadland polygon/API，將 `LangYa::MiniRoadland{25}` 改名為 `LangYa::PreRoadland{25}`；保留 ID 25。
- [x] 將 `CentralToBase` 改為紅 `(515,100)`、藍 `(2285,1400)`；保留 ID 22。
- [x] 在正式 main-area 檢查中確認 ID 25 只歸 PreRoadland、ID 21/22 只歸 Roadland，並讓共用邊界優先解析 Roadland。

### Task 2: 將 Mini Special 遷移為正式 PreRoadland AreaTask

**Files:**
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/include/AreaManager.hpp`
- Modify: `src/behavior_tree/src/AreaManager.cpp`
- Modify: `src/behavior_tree/include/Application.hpp`
- Modify: `src/behavior_tree/src/GameLoop.cpp`
- Modify: `src/behavior_tree/src/DefaultStrategyManager.cpp`
- Modify: `src/behavior_tree/include/DefaultStrategyManager.hpp`
- Modify: `src/behavior_tree/config/AreaManager.yaml`
- Modify: `src/behavior_tree/config/Special.yaml`
- Modify: `src/behavior_tree/config/PointManager.yaml`

- [x] 新增 `MyPreRoadlandAreaTaskSetting`、`RegionalAreaTaskType::MyPreRoadland` 與簡單 approach/hold phase。
- [x] 將原 Mini 的 Enable、GoalHoldSec、SpeedLevel 放到 `AreaManager.RegionalAreaTask.MyPreRoadland`；刪除 `SpecialMiniRoadlandSetting`、解析器和 GameLoop 入口。
- [x] DefaultStrategyManager 加入 MyPreRoadland 的 area scope、權重、retry/cooldown、current/last area 評分；其目標固定 ID 25。
- [x] 保留 MyRoadland 強綁定 phase、FaceMode、FollowMode、安全返回與 ID 21/22。

### Task 3: 更新所有配置 scope、工具和測試

**Files:**
- Modify: `src/behavior_tree/Scripts/ConfigJson/**/*.json`
- Modify: `src/behavior_tree/tools/AreaCalculator.cpp`
- Modify: `src/behavior_tree/CMakeLists.txt`
- Modify/Create: `src/behavior_tree/test/test_*.cpp`

- [x] 在所有 `DecisionAutonomy.NaviGoal.MyArea/EnemyArea` 增加 `PreRoadland` 開關，不改既有 Roadland key。
- [x] AreaCalculator 改為顯示正式 pre_roadland/roadland，移除 mini_roadland candidate 文案。
- [x] 測試幾何、ID 25/21/22 歸屬、共用邊界唯一歸屬與 Default candidate 分離。

### Task 4: 更新 trace、文件與圖譜

**Files:**
- Modify: `src/behavior_tree/src/DecisionTrace.cpp`（若 task string/trace 有列舉映射）
- Modify: `src/simulator/simulator/{model.py,trace.py,validation.py}`（若 task enum/schema 有假設）
- Modify: `docs/sentry/regional/decision_framework.md`
- Modify: `docs/modules/2026-05-05_behavior_tree.md`
- Modify: `docs/sentry/regional/2026-07-12_regional_decision_graph.md`
- Modify: `.understand-anything/{knowledge-graph.json,project-knowledge-graph.md,meta.json}`

- [x] 記錄新 Regional task、點位歸屬、刪除 Mini special 與 Roadland boundary replacement。
- [x] 更新互動圖譜的區域/BT 關係和 metadata，保持 source-checked fallback 狀態。

### Task 5: 分層驗證

**Files:** 無。

- [x] `colcon build --packages-select behavior_tree simulator --allow-overriding behavior_tree gimbal_driver --cmake-args -DBUILD_TESTING=ON`
- [x] `ctest --test-dir build/behavior_tree --output-on-failure`
- [x] 以 `area_calculator` 查紅藍 PreRoadland/Roadland 內點與 ID 25/21/22。
- [x] `./scripts/selfcheck.sh sentry --static-only`、`git diff --check`、JSON syntax check。
- [ ] 不啟動實機/串口；報告靜態與單元測試證據及未覆蓋的真機導航風險。
