# 人工筆記區

Updated: 2026-07-15

`docs/obsidian/notes/` 用於保存不適合從原始碼自動推導的工程知識：現場觀察、問題調查、決策理由、驗證結果、待辦脈絡與交接提示。此目錄 **不由同步工具管理**；同步、`--check` 與生成檔清理都不會修改或刪除其中內容。

回到 [[docs/obsidian/Home|知識庫首頁]]，或從 [[docs/obsidian/_generated/Index|自動索引]] 找到要關聯的 package、topic 與 message。

## 建議寫法

1. 一個議題一個 Markdown 檔，採用容易搜尋的名稱，例如 `2026-07-15-regional-reach-investigation.md`。
2. 開頭寫日期、狀態與可重現的事實；把推測、已驗證結論與待確認事項分開。
3. 用 wikilink 指向相關的生成 package／topic／message note，以及最近的 architecture、module、sentry 或 record 文件。
4. 若筆記結論改變了正式行為或介面描述，更新距離該事實最近的 `docs/` 文件與 source；筆記不應成為唯一的 runtime 契約。

## 範例骨架

```markdown
# Regional 到點事件調查

日期：2026-07-15
狀態：調查中

## 觀察

- 現場／重播中可重現的事實。

## 關聯

- [[docs/obsidian/_generated/Topics/ly__control__angles|/ly/control/angles]]
- [[docs/sentry/regional/current_behavior|Regional 當前行為]]

## 待確認

- 仍需要以 source、launch 或實機驗證的問題。
```

不要把個人設定放入本倉；尤其不提交 `.obsidian/` 內的 UI、workspace 或外掛狀態。
