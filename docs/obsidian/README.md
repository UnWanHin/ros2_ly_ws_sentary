# Obsidian 工程知識庫

Updated: 2026-07-15

此目錄把現有 ROS2 哨兵工作區整理成可瀏覽、可持續維護的工程知識庫。請以 **repository root**（本檔案所在倉庫的根目錄）開啟 Obsidian vault，而不是單獨開啟 `docs/obsidian/`；這樣既有文件、原始碼與生成索引都能在同一個 Graph 與 Local Graph 中互相跳轉。

從 [[docs/obsidian/Home|知識庫首頁]] 開始。它連到目前主鏈、模組／哨兵文件、人工筆記與由原始碼生成的索引。

## 內容邊界

- `docs/obsidian/_generated/`：只由同步工具產生。這裡的 package、topic 與 message 資料以 `package.xml`、`.msg`、source、launch 與 config 為依據；不要直接手改。
- `docs/obsidian/notes/`：人工筆記區。不會被同步工具修改或刪除，適合記錄現場觀察、待查問題、決策理由與連結脈絡。
- 既有 `docs/`：仍是目前文件的原位置。知識庫只建立可導航的入口，不複製或取代這些文件。

第一次使用可先開啟 [[docs/obsidian/Home|知識庫首頁]]，再用 Obsidian 的 Local Graph 查看單一 package、topic 或 message 的直接關係。

## 同步與檢查

修改 package、topic、message、launch 或決策鏈路後，執行下列命令重建或驗證自動索引：

```bash
python3 scripts/obsidian_sync.py
python3 scripts/obsidian_sync.py --check
python3 scripts/obsidian_sync.py --dry-run
```

- 第一個命令將來源掃描結果寫入 `docs/obsidian/_generated/`。
- `--check` 只檢查生成內容是否已隨來源更新；有漂移時會以非零狀態結束。
- `--dry-run` 只列出預計異動，不寫入檔案。

同步器只管理帶有生成標記、且列在 manifest 的檔案；它不會寫入 `notes/`。不要提交 `.obsidian/` 個人 UI 設定或工作區狀態。

## 維護原則

1. 原始碼、launch、package manifest 和訊息定義是執行行為與介面事實來源；生成筆記是其可導航視圖。
2. 手動補充的判斷、現場脈絡與未解問題放入 [[docs/obsidian/notes/README|人工筆記]]，並使用 wikilink 指向既有文件或生成節點。
3. 改變 ROS topic、message、參數、launch 組成、行為樹輸出或 simulator trace 時，先更新距離事實最近的既有文件，再重跑同步與檢查。
4. 外部 `sentry.aim`、導航、TF 與下位機只記錄本倉可觀測的邊界與契約；不要把其內部實作誤寫成此倉事實。
