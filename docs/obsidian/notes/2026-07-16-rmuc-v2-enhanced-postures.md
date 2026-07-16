---
title: RMUC V2.0 強化姿態規則
date: 2026-07-16
tags:
  - rmuc
  - sentry
  - posture
  - referee
---

# RMUC V2.0 強化姿態規則

官方規則比對已確認：RMUC V2.0.1 在普通進攻／防禦／移動姿態之外，新增三種每類獨立、每局最多 15 秒的強化姿態。它們是有限戰術資源，而不是普通姿態參數。

完整可追溯規則與工程接口記錄：[[docs/sentry/info/2026-07-16_rmuc_v2_enhanced_postures|RMUC V2.0 強化姿態規則與工程現況]]。

## 目前工程狀態

- `gimbal_driver` 可以下行 `4=強化進攻`、`5=強化防禦`、`6=強化移動`。
- `/ly/game/sentry/info` 已提供 `enhanced_posture` 與六個普通／強化姿態剩餘時間字段。
- `behavior_tree` 會讀取這些回報與寫入 DecisionTrace，但目前只自動選擇普通 `1..3`；尚未自動消耗強化姿態額度。

後續前哨策略需以此筆記作為強化姿態設計的規則基線。
