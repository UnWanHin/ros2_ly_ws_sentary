# 前哨強化交戰鎖設計

Status: approved for specification review

## Goal

在敵方前哨可見且被選為目標 `7` 時，讓哨兵以普通進攻姿態建立穩定交戰；當新鮮敵方前哨血量下降時，有限度地使用 RMUC V2.0 強化進攻姿態，並在姿態切換或強化期間避免非安全必要的轉火與退出。

## Scope and Non-Goals

- 保留既有 `/ly/aim/*`、`/ly/control/posture`、`/ly/control/sentry_cmd`、`/ly/game/sentry/info`、`/ly/enemy/op_hp` topic 與 message 契約。
- 保留既有導航、FaceMode、FireCode 與一般裝甲目標選擇的接口。
- 只改變前哨 id `7` 的交戰鎖與姿態意圖優先級；不把強化鎖套用到 Buff、Regional 或一般巡邏。
- 不在本功能中改動裁判協議、下位機 frame 布局或自動兌彈／復活策略。
- 不在沒有新鮮官方姿態與剩餘秒數資料時猜測強化已成功或可使用。

## Rule Basis

官方依據是 `docs/rules/RoboMaster 2026 机甲大师超级对抗赛比赛规则手册V2.0.1（20260629）.pdf`，5.6.4（PDF 第 115 頁、印刷頁 114）：

- 姿態切換冷卻為 5 秒。
- 強化進攻、強化防禦、強化移動各自每局最多累計 15 秒，額度彼此獨立。
- 強化進攻時熱量固定為 0、發彈不增加熱量、底盤功率為 1/2，且有 25% 易傷。
- 基礎姿態累計超過 3 分鐘後弱化；官方例子表明強化姿態時間會影響對應基礎姿態的累計風險。

完整規則與 V1.4.2 差異見 `docs/sentry/info/2026-07-16_rmuc_v2_enhanced_postures.md`。

## Architecture

### Ownership

`PostureManager` 保持唯一的姿態命令 owner。新增的 `OutpostEngagementLock` 不直接發布姿態 topic；它只根據前哨交戰狀態產生高優先級姿態 intent，交由 `PostureManager` 統一執行 ACK、5 秒冷卻與重發。

`PostureManager` 必須從只追蹤普通 `Attack/Defense/Move` 擴展為追蹤複合姿態模式：

| 下行值 | 基礎類別 | 強化 |
|---:|---|---|
| `1` | Attack | false |
| `2` | Defense | false |
| `3` | Move | false |
| `4` | Attack | true |
| `5` | Defense | true |
| `6` | Move | true |

普通類別繼續由 `/ly/gimbal/posture` 回讀；強化位由 `/ly/game/sentry/info.enhanced_posture` 回讀。只有兩者同時匹配才確認 `4/5/6` request，例如 `4` 的 ACK 是 `posture=1 && enhanced_posture=true`。

### Engagement State

`OutpostEngagementLock` 以以下條件建立：

1. 外部 aim 對 outpost id `7` 有新鮮有效目標與角度。
2. BT 已選擇 target id `7`。
3. 敵方前哨 HP 新鮮且大於零。

建立後，鎖先請求普通進攻 `1`。鎖定期間若敵方前哨 HP 出現新鮮的正向下降，且官方強化進攻剩餘秒數大於零，則設定 `EnhancedAttackArmed`。前哨 HP 沒有傷害來源欄位，因此此條件代表可觀測的「前哨正在承受傷害」，不假稱由本哨兵單獨造成。

鎖建立時記錄第一筆新鮮正值敵方前哨 HP，之後只在同一連續鎖中比較相鄰的新鮮 HP 樣本；新值小於上一筆樣本才 arm 強化。HP 上升（例如前哨重建）會更新基線但不 arm 強化。

當普通進攻已確認且姿態切換冷卻完成，`EnhancedAttackArmed` 請求強化進攻 `4`。強化進攻只在本次連續的 7 交戰鎖中嘗試一次；重試耗盡後標記本鎖的強化不可用，仍保持已確認的普通進攻與 7 鎖定。

### Target and Exit Policy

| 狀態 | 一般裝甲出現 | 允許前哨退出／轉火 |
|---|---|---|
| 普通進攻 pending 或確認後的 5 秒冷卻 | 保持 7 | 新鮮自身 HP `<=200`，或敵方前哨 HP 為 0／stale，或導航明確 unreachable |
| 強化進攻 pending 或 confirmed active | 保持 7 | 新鮮自身 HP `<=250`，或敵方前哨 HP 為 0／stale，或導航明確 unreachable |
| 沒有前哨交戰鎖 | 既有一般裝甲規則 | 既有前哨／一般裝甲規則 |

強化結束、目標 7 失去新鮮性、敵方前哨 HP 為零或 stale 時，解除對應鎖並回到既有前哨搜尋與一般裝甲優先級。鎖的出口不會抑制硬故障、死亡、導航不可達或官方前哨不存在資料。

任一鎖退出條件成立時，先取消該鎖所擁有的 pending `1` 或 `4` request，再把姿態選擇權交回既有普通策略。這可防止低血、前哨毀壞或路徑不可達後仍重發進攻／強化進攻；全域姿態冷卻仍由 `PostureManager` 維持。

### ACK and Failure Policy

每個姿態 request 在發出後進入 pending。pending 時：

1. 僅重發原 request，不能因目標分數或其他姿態建議改發另一個值。
2. 收到複合回讀 ACK 後才更新 `Current`、開始 5 秒冷卻並允許下一個 intent。
3. 沒有 ACK 時沿用既有 timeout、retry interval 與 retry count。
4. retry 耗盡時不執行既有的「選擇另一個普通姿態」fallback；保留最後一個已確認姿態與前哨鎖。
5. 若真實回讀新鮮，禁止 optimistic ACK；回讀缺失時的現有 optimistic fallback 只能用於普通模式，不能把強化 `4/5/6` 視為成功。

## Configuration

新增的前哨策略鍵放在 `Task.OutpostConfirm`，而既有 global `Posture` 鍵保持 owner：

```yaml
Task:
  OutpostConfirm:
    TrustEnemyOutpostHp: true
    EnhancedAttackOnEnemyHpDrop: true
    NormalAttackLockExitHp: 200
    EnhancedAttackLockExitHp: 250
```

`Posture.SwitchCooldownSec=5`、`PendingAckTimeoutMs`、`RetryIntervalMs`、`MaxRetryCount`、`RefereeInfo3FreshMs` 保持既有 `Posture` 配置歸屬。新的前哨鍵只決定何時建立或解除戰術鎖，不複製姿態 transport 與 ACK 設定。

## Observability and Documentation

DecisionTrace 增加前哨交戰鎖狀態、armed／pending／active 強化進攻狀態、鎖退出原因與複合姿態 request/ACK 狀態。因 trace schema 變更，`src/simulator/simulator/model.py`、`trace.py`、`validation.py` 與 `docs/sentry/internal/simulator.md` 必須在同一變更同步。

更新 Regional decision graph、`current_behavior.md`、`decision_framework.md`、`project-knowledge-graph.md`、`knowledge-graph.json`、`meta.json`，使前哨交戰鎖與姿態資料邊界可在 dashboard 中追蹤。

## Acceptance Criteria

1. select id `7` 的新鮮目標會建立前哨鎖並請求普通進攻。
2. `posture=1` 但 `enhanced_posture=false` 不可確認強化進攻 `4`。
3. 敵前哨 HP 新鮮下降只能在已鎖 7 時 arm 一次強化進攻；無強化剩餘秒數時不得發 `4`。
4. 普通姿態鎖在新鮮自身 HP 大於 200 時不轉火；強化鎖在新鮮自身 HP 大於 250 時不轉火。
5. 兩個鎖都在敵前哨 HP 為零或 stale、導航 unreachable 時立即解除。
6. pending 強化 ACK 失敗只重發 `4`；耗盡後保留普通進攻與 7 鎖，不改發 `2` 或 `3`。
7. 外部 topics、message layout、serial frame layout 與非前哨姿態行為保持相容。
8. 目標 build、PostureManager unit tests、behavior_tree tests、simulator validation、static selfcheck、Obsidian check、圖譜 JSON 驗證與 `git diff --check` 均通過。
