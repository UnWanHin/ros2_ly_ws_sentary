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
- `behavior_tree` 使用前哨交戰鎖：新鮮選擇 7 且官方敵前哨 HP 有效時先鎖普通進攻 `1`；鎖內 HP 新鮮下降後，確認普通進攻和姿態冷卻完成才申請一次 `4=強化進攻`。
- `4` 的確認必須同時滿足新鮮 `posture=1` 和 `enhanced_posture=true`；重試耗盡保留普通進攻與 7，不自動切防禦/移動。
- 普通/強化鎖的自身血量退出門檻分別為 `200/250`；敵前哨 HP 為零或 stale、導航不可達會立即取消 pending 並釋放鎖。

後續前哨策略需以此筆記作為強化姿態設計的規則基線。
