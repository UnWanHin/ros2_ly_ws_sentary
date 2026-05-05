# 哨兵底層決策區塊記錄

Updated: 2026-05-06

> 目的：先記錄新的底層決策區塊點位，後續再整理成可調參、可用包含判定的哨兵活動範圍配置。

## 座標約定

- 座標順序：先 `x`，再 `y`。
- 單位：`cm`。
- 本文件記錄的是新的底層決策區塊點位，不是 `src/behavior_tree/module/Area.hpp` 裡的 `Area` 點位。
- `roadland` 是一個底層決策區塊；`Red-roadland` 和 `Blue-roadland` 分別是這個區塊的紅方版本與藍方版本。
- 後續新增區塊時，沿用 `<Team>-<block_name>` 的分組方式記錄。
- 同一區塊先按紅藍分開，再按 `Custom ID` 編號遞增連線；若用作多邊形包含判定，第一點和最後一點相連形成閉合區域。

## 決策配置語義

程式內已把這批區塊作為 `MainArea` 決策區域使用，和舊的 `CastleRed` / `RoadLandRed` / `BaseRed` 等歷史區域保持分離。

JSON 配置入口：

```json
"DecisionAutonomy": {
  "Enable": true,
  "NaviGoal": {
    "UseAreaScope": true,
    "MyArea": {
      "Base": true,
      "Highland": true,
      "Roadland": false
    },
    "EnemyArea": {
      "Base": true,
      "Highland": true,
      "Roadland": false
    },
    "CommonArea": {
      "Central": false
    }
  }
}
```

- `MyArea` 表示我方活動區塊；`EnemyArea` 表示敵方活動區塊。
- `CommonArea` 表示不分敵我的公共區塊；目前只支持 `Central`。
- 程式會根據已識別隊伍自動映射：若本車是紅方，`MyArea` 使用 Red MainArea，`EnemyArea` 使用 Blue MainArea；若本車是藍方則相反。
- `Central` 不再放進 `MyArea` / `EnemyArea`，而是由 `CommonArea.Central` 統一控制。
- `UseAreaScope=false` 時保持舊導航決策行為。
- `UseAreaScope=true` 時，Utility 自評估候選點和正式策略 fallback 都會先檢查是否落在允許區塊內；不允許的點不參與評分，也不會發布對應 `goal_pos`。
- Recovery 保命回補、NaviTest 調試路線、League/Showcase 專用路線仍按原硬規則執行，不受這個作戰區域調參限制。

## Red-roadland

| Custom ID | Label | Team | X(cm) | Y(cm) |
| --- | --- | --- | ---: | ---: |
| C9 | Custom | 紅 | 685 | 368 |
| C11 | Custom | 紅 | 391 | 373 |
| C13 | Custom | 紅 | 389 | 13 |
| C15 | Custom | 紅 | 753 | 235 |
| C19 | Custom | 紅 | 1217 | 24 |
| C21 | Custom | 紅 | 1228 | 263 |
| C23 | Custom | 紅 | 1026 | 265 |

Point tuple form:

```text
Red-roadland = [
  (685, 368),   # C9
  (391, 373),   # C11
  (389, 13),    # C13
  (753, 235),   # C15
  (1217, 24),   # C19
  (1228, 263),  # C21
  (1026, 265),  # C23
]
```

## Blue-roadland

| Custom ID | Label | Team | X(cm) | Y(cm) |
| --- | --- | --- | ---: | ---: |
| C10 | Custom | 藍 | 2115 | 1132 |
| C12 | Custom | 藍 | 2409 | 1127 |
| C14 | Custom | 藍 | 2411 | 1487 |
| C16 | Custom | 藍 | 2047 | 1265 |
| C20 | Custom | 藍 | 1583 | 1476 |
| C22 | Custom | 藍 | 1572 | 1237 |
| C24 | Custom | 藍 | 1774 | 1235 |

Point tuple form:

```text
Blue-roadland = [
  (2115, 1132),  # C10
  (2409, 1127),  # C12
  (2411, 1487),  # C14
  (2047, 1265),  # C16
  (1583, 1476),  # C20
  (1572, 1237),  # C22
  (1774, 1235),  # C24
]
```

## Red-Highland

| Custom ID | Label | Team | X(cm) | Y(cm) |
| --- | --- | --- | ---: | ---: |
| C41 | Custom | 紅 | 313 | 1065 |
| C43 | Custom | 紅 | 315 | 1497 |
| C45 | Custom | 紅 | 1221 | 1494 |
| C47 | Custom | 紅 | 1226 | 1400 |
| C49 | Custom | 紅 | 976 | 1403 |
| C51 | Custom | 紅 | 744 | 1077 |

Point tuple form:

```text
Red-Highland = [
  (313, 1065),   # C41
  (315, 1497),   # C43
  (1221, 1494),  # C45
  (1226, 1400),  # C47
  (976, 1403),   # C49
  (744, 1077),   # C51
]
```

## Blue-Highland

| Custom ID | Label | Team | X(cm) | Y(cm) |
| --- | --- | --- | ---: | ---: |
| C42 | Custom | 藍 | 2487 | 435 |
| C44 | Custom | 藍 | 2485 | 3 |
| C46 | Custom | 藍 | 1579 | 6 |
| C48 | Custom | 藍 | 1574 | 100 |
| C50 | Custom | 藍 | 1824 | 97 |
| C52 | Custom | 藍 | 2056 | 423 |

Point tuple form:

```text
Blue-Highland = [
  (2487, 435),  # C42
  (2485, 3),    # C44
  (1579, 6),    # C46
  (1574, 100),  # C48
  (1824, 97),   # C50
  (2056, 423),  # C52
]
```

## Red-base

| Custom ID | Label | Team | X(cm) | Y(cm) |
| --- | --- | --- | ---: | ---: |
| C53 | Custom | 紅 | 38 | 997 |
| C55 | Custom | 紅 | 765 | 1010 |
| C57 | Custom | 紅 | 1040 | 1368 |
| C59 | Custom | 紅 | 1253 | 1384 |
| C61 | Custom | 紅 | 1256 | 1318 |
| C67 | Custom | 紅 | 999 | 958 |
| C69 | Custom | 紅 | 992 | 508 |
| C71 | Custom | 紅 | 1040 | 400 |
| C73 | Custom | 紅 | 1042 | 325 |
| C75 | Custom | 紅 | 735 | 327 |
| C77 | Custom | 紅 | 687 | 423 |
| C79 | Custom | 紅 | 334 | 423 |
| C81 | Custom | 紅 | 338 | 22 |
| C83 | Custom | 紅 | 143 | 22 |
| C85 | Custom | 紅 | 157 | 196 |
| C87 | Custom | 紅 | 100 | 201 |
| C89 | Custom | 紅 | 109 | 299 |
| C91 | Custom | 紅 | 13 | 315 |

Point tuple form:

```text
Red-base = [
  (38, 997),    # C53
  (765, 1010),  # C55
  (1040, 1368), # C57
  (1253, 1384), # C59
  (1256, 1318), # C61
  (999, 958),   # C67
  (992, 508),   # C69
  (1040, 400),  # C71
  (1042, 325),  # C73
  (735, 327),   # C75
  (687, 423),   # C77
  (334, 423),   # C79
  (338, 22),    # C81
  (143, 22),    # C83
  (157, 196),   # C85
  (100, 201),   # C87
  (109, 299),   # C89
  (13, 315),    # C91
]
```

## Blue-base

| Custom ID | Label | Team | X(cm) | Y(cm) |
| --- | --- | --- | ---: | ---: |
| C54 | Custom | 藍 | 2762 | 503 |
| C56 | Custom | 藍 | 2035 | 490 |
| C58 | Custom | 藍 | 1760 | 132 |
| C60 | Custom | 藍 | 1547 | 116 |
| C62 | Custom | 藍 | 1544 | 182 |
| C68 | Custom | 藍 | 1801 | 542 |
| C70 | Custom | 藍 | 1808 | 992 |
| C72 | Custom | 藍 | 1760 | 1100 |
| C74 | Custom | 藍 | 1758 | 1175 |
| C76 | Custom | 藍 | 2065 | 1173 |
| C78 | Custom | 藍 | 2113 | 1077 |
| C80 | Custom | 藍 | 2466 | 1077 |
| C82 | Custom | 藍 | 2462 | 1478 |
| C84 | Custom | 藍 | 2657 | 1478 |
| C86 | Custom | 藍 | 2643 | 1304 |
| C88 | Custom | 藍 | 2700 | 1299 |
| C90 | Custom | 藍 | 2691 | 1201 |
| C92 | Custom | 藍 | 2787 | 1185 |

Point tuple form:

```text
Blue-base = [
  (2762, 503),  # C54
  (2035, 490),  # C56
  (1760, 132),  # C58
  (1547, 116),  # C60
  (1544, 182),  # C62
  (1801, 542),  # C68
  (1808, 992),  # C70
  (1760, 1100), # C72
  (1758, 1175), # C74
  (2065, 1173), # C76
  (2113, 1077), # C78
  (2466, 1077), # C80
  (2462, 1478), # C82
  (2657, 1478), # C84
  (2643, 1304), # C86
  (2700, 1299), # C88
  (2691, 1201), # C90
  (2787, 1185), # C92
]
```

## Red-Central

| Custom ID | Label | Team | X(cm) | Y(cm) |
| --- | --- | --- | ---: | ---: |
| C99 | Custom | 紅 | 1187 | 269 |
| C101 | Custom | 紅 | 1182 | 471 |
| C103 | Custom | 紅 | 1047 | 469 |
| C105 | Custom | 紅 | 1029 | 533 |
| C107 | Custom | 紅 | 1022 | 953 |
| C109 | Custom | 紅 | 1235 | 1247 |
| C111 | Custom | 紅 | 1618 | 1247 |
| C113 | Custom | 紅 | 1606 | 1031 |
| C115 | Custom | 紅 | 1753 | 1033 |
| C117 | Custom | 紅 | 1778 | 990 |
| C119 | Custom | 紅 | 1755 | 558 |
| C121 | Custom | 紅 | 1565 | 260 |

Point tuple form:

```text
Red-Central = [
  (1187, 269),  # C99
  (1182, 471),  # C101
  (1047, 469),  # C103
  (1029, 533),  # C105
  (1022, 953),  # C107
  (1235, 1247), # C109
  (1618, 1247), # C111
  (1606, 1031), # C113
  (1753, 1033), # C115
  (1778, 990),  # C117
  (1755, 558),  # C119
  (1565, 260),  # C121
]
```

## Blue-Central

| Custom ID | Label | Team | X(cm) | Y(cm) |
| --- | --- | --- | ---: | ---: |
| C100 | Custom | 藍 | 1613 | 1231 |
| C102 | Custom | 藍 | 1618 | 1029 |
| C104 | Custom | 藍 | 1753 | 1031 |
| C106 | Custom | 藍 | 1771 | 967 |
| C108 | Custom | 藍 | 1778 | 547 |
| C110 | Custom | 藍 | 1565 | 253 |
| C112 | Custom | 藍 | 1182 | 253 |
| C114 | Custom | 藍 | 1194 | 469 |
| C116 | Custom | 藍 | 1047 | 467 |
| C118 | Custom | 藍 | 1022 | 510 |
| C120 | Custom | 藍 | 1045 | 942 |
| C122 | Custom | 藍 | 1235 | 1240 |

Point tuple form:

```text
Blue-Central = [
  (1613, 1231), # C100
  (1618, 1029), # C102
  (1753, 1031), # C104
  (1771, 967),  # C106
  (1778, 547),  # C108
  (1565, 253),  # C110
  (1182, 253),  # C112
  (1194, 469),  # C114
  (1047, 467),  # C116
  (1022, 510),  # C118
  (1045, 942),  # C120
  (1235, 1240), # C122
]
```
