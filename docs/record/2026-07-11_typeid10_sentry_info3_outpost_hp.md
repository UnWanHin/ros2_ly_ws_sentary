# TypeID=10 sentry_info_3 / precise outpost HP

Updated: 2026-07-11

## 1. 變更結果

`gimbal_driver` 新增上行 `TypeID=10 SentryInfo3AndOutpostHpData`，payload 仍是 12B。

| byte offset | 字段 | 裁判來源 |
|---|---|---|
| 0~7 | `SentryInfo3` | `0x020D sentry_info_t.sentry_info_3` offset 6 |
| 8~9 | `SelfOutpostHealth` | `0x0003 game_robot_HP_t.ally_outpost_HP` offset 12 |
| 10~11 | `EnemyOutpostHealth` | `0x0003 game_robot_HP_t.enemy_outpost_HP` offset 16 |

前哨血量按官方 `0x0003` 順序打包：己方在前，敵方在後。

## 2. ROS topic

- `/ly/friend/op_hp`：優先發布 TypeID 10 的 `SelfOutpostHealth`
- `/ly/enemy/op_hp`：優先發布 TypeID 10 的 `EnemyOutpostHealth`
- `/ly/game/sentry/info`：新增 `sentry_info_3_raw`、`has_sentry_info_3` 和普通/強化姿態剩餘秒數字段

`TypeID=1 GameCode` 里的 6-bit 前哨血量仍保留，但只作 fallback：TypeID 10 未收到，或最近一次 TypeID 10 超過 1500ms 未更新時，才發布 `GameCode * 25`。

`0` 是合法前哨血量，表示前哨站已毀，不表示沒有收到。
因此如果下位機暫時拿不到裁判 `0x0003` 的前哨血量，不要用 `0` 或預設值繼續發 TypeID 10；應暫停發 TypeID 10，讓上位機在 1500ms 後自動 fallback 到舊 `GameCode * 25`。

## 3. 下位機要求

1. 收到裁判 `0x020D sentry_info_t` 時，保存 `sentry_info_3`，並填入 TypeID 10 byte `0~7`。
2. 收到裁判 `0x0003 game_robot_HP_t` 時，保存 `ally_outpost_HP` 和 `enemy_outpost_HP`，並填入 TypeID 10 byte `8~11`。
3. 不要把 TypeID 10 的前哨血量按舊 `GameCodeType` 的 enemy-first 順序打包；本次使用 official `ally` first。
4. 若 `0x0003` 未收到或已判定失效，暫停發 TypeID 10，而不是發未初始化血量；上位機以 TypeID 10 新鮮度決定是否 fallback。

## 4. 上位機行為

`TypeID=10` 到達後立即發布兩個 `op_hp` topic。`sentry_info_3` 只更新 shadow；`/ly/game/sentry/info` 仍由 TypeID 7 發布，避免 TypeID 10 的前哨血量更新頻率錯誤刷新 `sentry_info_2` 的新鮮度。
