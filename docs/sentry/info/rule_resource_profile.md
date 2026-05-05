# 賽規血量/彈量與 BT 資源模型盤點

Updated: 2026-05-06

本文記錄 `docs/rules` 裡和哨兵血量、彈量相關的賽規數值，並對照目前 `gimbal_driver -> behavior_tree` 的數據流。這次只做分析文檔，不改運行代碼。

## 規則來源

本次核對的本地文件：

- `docs/rules/RoboMaster 2026 机甲大师超级对抗赛比赛规则手册V1.4.1（20260417）.pdf`
- `docs/rules/RoboMaster 2026 机甲大师高校联盟赛比赛规则手册 V1.2.0（20260109）.pdf`
- `docs/rules/RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0（20260327）.pdf`
- `docs/rules/RoboMaster 裁判系统串口协议附录 V1.9.0（20250703）.pdf`

環境裡沒有 `pdftotext/pdfplumber`，所以用 `pypdf` 做文字抽取核對。下面的頁碼按 PDF 頁面序號記錄。

## 賽規數值

### 超級對抗賽 V1.4.1

哨兵關鍵數值：

| 項目 | 數值 |
|---|---:|
| 自動哨兵初始/上限血量 | 400 |
| 半自動哨兵初始/上限血量 | 200 |
| 自動哨兵熱量上限 | 260 |
| 自動哨兵熱量冷卻 | 30/s |
| 半自動哨兵熱量上限 | 100 |
| 半自動哨兵熱量冷卻 | 10/s |
| 自動哨兵底盤功率上限 | 100 W |
| 半自動哨兵底盤功率上限 | 60 W |
| 初始允許發彈量 | 300 |

來源：超級對抗賽 V1.4.1，第 29-30 頁。

彈量與回血規則：

| 項目 | 數值/語義 |
|---|---|
| 七分鐘比賽階段允許發彈量 | 可通過補給區、基地增益點、前哨站增益點兌換或直接獲取，也可遠程兌換 |
| 補給區/基地/前哨兌換最小單位 | 17mm 為 10 發，42mm 為 1 發 |
| 遠程兌換最小單位 | 17mm 為 100 發，42mm 為 10 發 |
| 補給區獲取 | 比賽開始後每隔 1 分鐘，哨兵占領己方補給區可獲取 100 發，未獲取部分可累積 |
| 補給區回血 | 一般為每秒上限血量 10%；比賽開始 4 分鐘後，脫戰且占領補給區時為 25% |
| 遠程回血 | 6 秒後增加此時上限血量 60%，不超過上限 |

來源：超級對抗賽 V1.4.1，第 75、80-81 頁。

基地/前哨數值：

| 項目 | 數值 |
|---|---:|
| 基地血量 | 5000 |
| 前哨站血量 | 1500 |
| 重建後前哨站血量 | 750 |
| 基地護甲展開門檻 | 基地血量降至 2000 及以下 |

來源：超級對抗賽 V1.4.1，第 98-99 頁。

### 聯盟賽 V1.2.0

哨兵關鍵數值：

| 項目 | 數值 |
|---|---:|
| 哨兵初始/上限血量 | 400 |
| 哨兵底盤功率上限 | 100 W |
| 哨兵熱量上限 | 260 |
| 哨兵熱量冷卻 | 30/s |
| 初始允許發彈量 | 750 |
| 局內是否可增加允許發彈量 | 不可增加 |

來源：聯盟賽 V1.2.0，第 19-20 頁。

回血/復活：

| 項目 | 數值/語義 |
|---|---|
| 補給區回血 | 英雄、步兵、哨兵占領己方補給區時，每秒恢復上限血量 25% |
| 復活後血量 | 上限血量 20% |
| 初次復活讀條 | 5，每次戰亡後增加 5 |

來源：聯盟賽 V1.2.0，第 34 頁。

聯盟賽是 3V3 對抗，核心勝負是勝利點，不是超級對抗賽的基地/前哨血量鏈路。BT 在 `CompetitionProfile=league` 時會直接選 `LeagueSimple`，不應依賴超級對抗賽的前哨/基地判斷。

## 通信協議對應

高校系列賽通信協議裡和這些值相關的接口：

| 命令碼 | 語義 |
|---|---|
| `0x0003` | 機器人血量數據，包含己方英雄、工程、步兵、哨兵、前哨站、基地血量 |
| `0x0201` | 機器人性能體系數據，包含當前血量、血量上限、熱量冷卻、熱量上限、底盤功率上限 |
| `0x0208` | 允許發彈量，包含 17mm、42mm、剩餘金幣、堡壘儲備 17mm 允許發彈量 |
| `0x020D` | 哨兵自主決策信息，包含哨兵兌彈、遠程兌彈、遠程回血、復活、姿態等狀態 |

來源：高校系列賽通信協議 V1.3.0，第 7-8、10、14、18、23-24 頁；裁判系統串口協議附錄 V1.9.0，第 10-11、14、19、23、31-32 頁。

重點是：規則上有「當前血量」和「血量上限」兩類數據，但目前本倉庫的 `gimbal_driver` 沒有把血量上限單獨發給 BT。

## 現有 gimbal_driver 鏈路

`gimbal_driver` 目前像一個裁判/下位機數據轉發層，不保存比賽規則 profile。

主要數據結構：

- `src/gimbal_driver/module/BasicTypes.hpp`
  - `GameData::AmmoLeft`
  - `GameData::SelfHealth`
  - `GameCodeType::EnemyOutpostHealth`
  - `GameCodeType::SelfOutpostHealth`
  - `HealthMyselfData`
  - `HealthEnemyData`

主要發布：

- `src/gimbal_driver/main.cpp`
  - `/ly/game/all`
    - `ammoleft`
    - `timeleft`
    - `selfhealth`
    - `gamecode`
    - `exteventdata`
  - `/ly/me/ammo_left`
    - `GameData::AmmoLeft`
  - `/ly/me/op_hp`
    - `GameCode.SelfOutpostHealth * 25`
  - `/ly/enemy/op_hp`
    - `GameCode.EnemyOutpostHealth * 25`
  - `/ly/me/base_hp`
    - `HealthMyselfData::BaseMyself`
  - `/ly/enemy/base_hp`
    - `HealthEnemyData::BaseEnemy`
  - `/ly/me/hp`
    - 己方各機器人血量
  - `/ly/enemy/hp`
    - 敵方各機器人血量

因此，`gimbal_driver` 現在沒有「聯盟賽/超級對抗賽」的分支，也不應該把規則策略塞進去。它只要穩定發當前裁判/下位機數值即可。

有一個值得注意的實現細節：前哨站血量在 `GameCodeType` 裡是 6 bit，發布時乘以 25。這剛好可以覆蓋超級對抗賽前哨站 1500 血量，因為 60 * 25 = 1500。

## 現有 BT 鏈路

BT 目前只存一份 runtime 當前資源值：

- `src/behavior_tree/include/Application.hpp`
  - `myselfHealth`
  - `ammoLeft`
  - `selfOutpostHealth`
  - `enemyOutpostHealth`
  - `selfBaseHealth`
  - `enemyBaseHealth`

訂閱來源：

- `src/behavior_tree/src/SubscribeMessage.cpp`
  - `/ly/game/all.selfhealth` -> `myselfHealth`
  - `/ly/me/ammo_left` -> `ammoLeft`
  - `/ly/me/op_hp` -> `selfOutpostHealth`
  - `/ly/enemy/op_hp` -> `enemyOutpostHealth`
  - `/ly/me/base_hp` -> `selfBaseHealth`
  - `/ly/enemy/base_hp` -> `enemyBaseHealth`
  - `/ly/me/hp` / `/ly/enemy/hp` -> 己方/敵方機器人血量表

也就是說，BT 現在的 runtime 當前血量/彈量只有一份，不會同時保存「聯盟賽版本」和「超級對抗賽版本」。目前 profile 差異主要靠配置文件的門檻值來體現。

## 目前用到血量/彈量的決策點

### Regional 相關

`config/AreaManager.yaml`：

| 模塊 | 當前門檻 |
|---|---:|
| `MyRoadland.HealthyHpMin` | 300 |
| `MyRoadland.HealthyAmmoMin` | 50 |
| `CommonCentral.HealthyHpMin` | 300 |
| `CommonCentral.HealthyAmmoMin` | 50 |

`src/behavior_tree/Scripts/ConfigJson/regional_competition.json`：

| 模塊 | 當前門檻 |
|---|---:|
| `RegionalDefense.StrongHealthMin` | 250 |
| `RegionalDefense.StrongAmmoMin` | 40 |
| `Posture.LowHealthThreshold` | 120 |
| `Posture.VeryLowHealthThreshold` | 80 |
| `Posture.LowAmmoThreshold` | 30 |

代碼位置：

- `src/behavior_tree/src/GameLoop.cpp`
  - Roadland/Central 健康判斷使用 `myselfHealth` 和 `ammoLeft`
  - RegionalDefense 強資源判斷使用 `StrongHealthMin/StrongAmmoMin`
- `src/behavior_tree/src/PostureLogic.cpp`
  - 姿態根據低血、極低血、低彈加權

這些值明顯按「400 血哨兵」設計：300 約等於 75% 血量，250 約等於 62.5% 血量，120 約等於 30% 血量，80 約等於 20% 血量。

### League 相關

`src/behavior_tree/Scripts/ConfigJson/league_competition.json`：

| 模塊 | 當前門檻 |
|---|---:|
| `LeagueStrategy.HealthRecoveryThreshold` | 210 |
| `LeagueStrategy.AmmoRecoveryThreshold` | 30 |
| `LeagueStrategy.HealthRecoveryExitMin` | 350 |
| `LeagueStrategy.HealthRecoveryExitPreferred` | 400 |

代碼位置：

- `src/behavior_tree/src/GameLoop.cpp::CheckPositionRecovery()`
- `src/behavior_tree/src/Configuration.cpp`
  - 目前會把 `HealthRecoveryExitMin/Preferred` clamp 到 400

聯盟賽哨兵上限血量就是 400，所以這個 clamp 對現在聯盟賽哨兵是對的。但如果以後 regional 用半自動哨兵 200 血，這套硬門檻就會不合適。

### 舊共用/硬編碼點

`src/behavior_tree/src/GameLoop.cpp::CheckPositionRecovery()` 非 league 分支有硬編碼：

- 已在 Recovery 且 `myselfHealth < 380` 時保持 Recovery；
- `myselfHealth < 150` 時回 Recovery；
- `ammoLeft <= 30` 且 recoveryClock 到期時回 Recovery。

舊 `HitHero/HitSentry` 邏輯還有：

- `selfOutpostHealth > 100`
- `selfOutpostHealth > 200`
- `selfBaseHealth > 2000`

這些是老策略門檻，不是正式的規則 profile 模型。

## 現狀判斷

結論：

- `gimbal_driver` 不存賽規，不應在那裡分 profile。
- BT runtime 當前值現在只有一份：`myselfHealth/ammoLeft/...`。
- BT 配置已經分了 `regional_competition.json` 和 `league_competition.json`，但沒有一個正式的「RuleResourceProfile」去表達兩套賽規的血量上限、初始彈量、補給方式。
- 對自動哨兵來說，超級對抗賽和聯盟賽的上限血量都是 400；主要差異是初始/局內彈量：
  - 超級對抗賽：初始 300，局內可兌換/獲取；
  - 聯盟賽：初始 750，局內不可增加。
- 如果 regional 走半自動哨兵，超級對抗賽上限血量會變成 200，現在很多 BT 門檻會出錯。

所以，「聯盟賽和超級對抗賽是否要分兩份」的回答是：

- 當前自動哨兵血量上限相同，暫時不一定因血量必須分；
- 但彈量規則已經不同，而且半自動 regional 會讓血量上限不同；
- 因此 BT 裡最好補一個正式資源規則 profile，不要只靠散落的硬編碼和 JSON 門檻。

## 建議設計

建議後續在 BT 內部增加一層只描述賽規資源的模型，例如：

```cpp
enum class RuleResourceProfile {
    RegionalSuperAuto,
    RegionalSuperSemiAuto,
    League
};

struct SentryResourceRule {
    std::uint16_t MaxHealth;
    std::uint16_t InitialAmmo17mm;
    bool AmmoCanIncreaseInMatch;
    int SupplyHealPercent;
    int SupplyHealOutOfCombatPercent;
    std::uint16_t BaseHealth;
    std::uint16_t OutpostHealth;
};
```

初始表可以先是：

| Profile | MaxHealth | InitialAmmo17mm | AmmoCanIncreaseInMatch | BaseHealth | OutpostHealth |
|---|---:|---:|---|---:|---:|
| `RegionalSuperAuto` | 400 | 300 | true | 5000 | 1500 |
| `RegionalSuperSemiAuto` | 200 | 300 | true | 5000 | 1500 |
| `League` | 400 | 750 | false | 0/unused | 0/unused |

選擇規則：

- `CompetitionProfile=regional` 時優先使用 `RegionalSuperAuto`。
- 如果後面真的要半自動哨兵，再加配置選 `RegionalSuperSemiAuto`。
- `CompetitionProfile=league` 時使用 `League`。
- profile 缺失或未知時，按你的要求，默認 regional 優先。

使用方式：

- runtime 當前值仍然只存一份，因為裁判系統上報的是當前真值；
- rule profile 只提供「上限、初始、能不能補彈、百分比門檻」；
- 需要健康判斷時，用 profile 生成門檻，而不是到處寫 300/250/150/380；
- AreaManager 的 regional 任務仍然優先使用 regional profile，不被 league 門檻覆蓋。

## 建議優先改動順序

1. 先在 BT 加 `SentryResourceRule` 和 `GetActiveResourceRule()`，不改 topic。
2. 把 regional 的健康判斷保留現值，但由 profile 推導出默認值：
   - Healthy 約 75% max HP；
   - Strong 約 60%-65% max HP；
   - Low 約 30% max HP；
   - VeryLow 約 20% max HP。
3. 保留 JSON 顯式配置覆蓋，避免現場調參失效。
4. 把 `CheckPositionRecovery()` 裡非 league 的 `380/150/30` 改成 profile-aware 門檻。
5. 如果下位機/裁判鏈路能提供 `0x0201 maximum_HP` 和 `0x0208 projectile_allowance` 的完整字段，再新增 topic；但不要讓 `gimbal_driver` 自己決策 profile。

## 風險點

- `gimbal_driver` 目前只把 `GameData::AmmoLeft` 當一個 17mm 剩餘/允許發彈量使用，沒有分 17mm、42mm、堡壘儲備、剩餘金幣。
- BT 目前沒有 `maximum_HP`，所以無法自動判斷自動/半自動哨兵的 400/200 上限。
- `HealthRecoveryExitPreferred=400` 和 `CheckPositionRecovery()` 的 380 對 400 血哨兵合理，但對 200 血半自動哨兵不合理。
- Regional 的 `HealthyHpMin=300` 對 400 血自動哨兵合理，對 200 血半自動哨兵會導致任務永遠不健康。
- 聯盟賽和超級對抗賽的彈量規則不同，後續如果要做自動兌彈/彈量保守策略，不能只看 `ammoLeft <= 30`。
