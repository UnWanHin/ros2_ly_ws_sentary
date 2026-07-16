# Obsidian 工程知識庫設計

**日期：** 2026-07-15
**狀態：** 已授權實作

## 目標

把 `ros2_ly_ws_sentry` 的現況做成可直接以 Obsidian 開啟、可從原始碼重建的工程知識庫。它應讓接手者從 package、ROS topic、message、launch／決策鏈路與現有文件之間跳轉，而不是再維護一份與程式脫節的靜態圖。

## 已考慮方案

1. 只保留 Understand Anything Dashboard：資料已有，但瀏覽入口是單一網頁，無法利用 Obsidian 的反向連結、Local Graph 與手寫知識筆記。
2. 新增獨立 vault 並複製所有文件：隔離性好，但會造成兩份文件與來源漂移。
3. **採用：以 repository root 作為 vault，新增 `docs/obsidian/` 知識入口與生成區。** 既有 `docs/` 保持原位，新 notes 直接 wikilink 到程式碼與現有 Markdown，Obsidian 的 Graph／Local Graph 立即可用。

## 架構

```text
原始碼 / package.xml / msg / launch / YAML
                 │
                 ▼
       scripts/obsidian_sync.py
                 │
                 ├── docs/obsidian/_generated/  # 僅工具擁有
                 │   ├── Index.md
                 │   ├── Packages/*.md
                 │   ├── Topics/*.md
                 │   ├── Messages/*.md
                 │   └── Reports/sync-report.md
                 │
                 └── docs/obsidian/              # 人工維護入口與 notes/
```

使用者以 repository root 開啟 Obsidian vault；不提交或覆蓋 `.obsidian/` 個人 UI 設定。Generated notes 使用 YAML properties、可追溯的 wikilink 與固定 `generated` 標記。人工內容只放 `docs/obsidian/notes/` 或入口 note，永不被同步器寫入。

## 來源與邊界

- `package.xml` 是 ROS package 身分與依賴的來源。
- `msg/*.msg` 是訊息欄位的來源。
- `src/<package>/` 和其 `launch/`、`config/` 中的 C++、Python、XML、YAML 是 topic 出現位置的來源。
- 既有 `docs/`、`.understand-anything/knowledge-graph.json` 只作可跳轉的補充證據；同步器不把既有圖譜當唯一真相。
- 第一輪不改 ROS runtime、topic／message 契約、launch 組成或 `.understand-anything/` 內容。

## 同步契約

`python3 scripts/obsidian_sync.py` 會以標準函式庫掃描，確定性地逐檔重建工具擁有的 generated note，並更新 manifest 以清理已不存在的 generated note。它只允許刪除 manifest 中列出、仍帶有生成標記、且位於 `_generated/` 內的檔案；遇到人工改寫、symlink 或越界 manifest 路徑時保留檔案並以非零狀態回報。

`--check` 不寫檔，若原始碼與生成輸出不一致則回傳非零；適合在修改 package、topic、message、launch 或決策鏈路後執行。`--dry-run` 顯示預計異動數量。

## 品質與驗證

- 為 topic 擷取、message 欄位、確定性輸出、`--check` 漂移與不碰人工筆記寫 pytest（以 `python3 -m pytest` 執行，不依賴 PATH 中的 `pytest` wrapper）。
- 首次同步後以 `--check` 驗證無漂移。
- 驗證所有 markdown link 目標、JSON manifest 與 `git diff --check`。
- 依本倉規範再跑 `./scripts/selfcheck.sh sentry --static-only`；同步器不介入 ROS runtime。

## 非目標

- 不自動安裝 Obsidian 外掛或依賴 Dataview。
- 不生成或提交使用者的 `.obsidian/` UI／workspace 設定。
- 不覆寫現有 docs、現有 Understand Anything 圖譜、或本工作樹原有未提交變更。
- 不把外部 `sentry.aim`、外部導航、下位機韌體的內部實作誤記為本倉事實。
