# Obsidian Knowledge Vault Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立一個可由本倉原始碼重建的 Obsidian 工程知識庫，不改 ROS runtime。

**Architecture:** Repository root 是 Obsidian vault；`docs/obsidian/` 提供人工入口與筆記區，而 `docs/obsidian/_generated/` 僅由同步腳本管理。同步器直接掃描 ROS package manifests、message 定義和 source／launch／config 檔中的 topic 字串，生成可反向導航的 Markdown。

**Tech Stack:** Python 3 standard library、pytest（`python3 -m pytest`）、Obsidian core Graph／Local Graph、Markdown/YAML front matter。

## Global Constraints

- 不修改 ROS runtime、topic/message/public config 契約或 `.understand-anything/` 圖譜。
- 不依賴或提交 Obsidian 外掛／`.obsidian/` 使用者設定。
- 只讓同步器寫入 `docs/obsidian/_generated/`；手寫內容放 `docs/obsidian/notes/`。
- 保留外部系統邊界：外部 `sentry.aim`、導航、TF、下位機不是本倉的內部實作。
- 現有工作樹不乾淨；只 stage／review 本次新增檔案，未獲授權不提交其他變更。

---

### Task 1: 建立 vault 入口與生成範圍

**Files:**
- Create: `docs/obsidian/README.md`
- Create: `docs/obsidian/Home.md`
- Create: `docs/obsidian/notes/README.md`

**Interfaces:**
- Consumes: repository root 作為 Obsidian vault、既有 `docs/README.md` 與架構文件。
- Produces: 可人工維護的入口；`Home.md` 連到生成索引、現有架構／模組／sentry 文件與 notes 區。

- [ ] **Step 1: 建立入口測試預期**

在 `tests/test_obsidian_sync.py` 定義：同步後 `Index.md`、`Packages/behavior_tree.md`、`Topics/ly__control__angles.md`、`Messages/gimbal_driver/GimbalAngles.md` 均存在，且 `docs/obsidian/notes/manual.md` 不受同步影響。

- [ ] **Step 2: 建立入口內容**

`README.md` 明確要求「以 repository root 開啟 vault」，列出同步、check、dry-run 命令；`Home.md` 使用 `[[obsidian/_generated/Index|自動索引]]`、`[[architecture/2026-07-12_project_link_graph|現行主鏈]]` 和 `[[obsidian/notes/README|人工筆記]]`。`notes/README.md` 說明此目錄不由工具管理。

- [ ] **Step 3: 驗證入口不需 Obsidian 外掛**

Run: `rg -n 'Dataview|\.obsidian' docs/obsidian`

Expected: 僅允許「不提交 `.obsidian/`」的說明，沒有外掛需求。

### Task 2: 撰寫可測試的 source-led 同步器

**Files:**
- Create: `scripts/obsidian_sync.py`
- Create: `tests/test_obsidian_sync.py`

**Interfaces:**
- Consumes: `--repo-root PATH`（預設 script 的 parent）、固定輸出 `docs/obsidian`、`--check`、`--dry-run`。
- Produces: `collect_workspace(root: Path) -> WorkspaceIndex`、`render_workspace(index: WorkspaceIndex) -> dict[PurePosixPath, str]`、`sync(root: Path, output: Path, check: bool, dry_run: bool) -> SyncResult`；generated tree 和 `.manifest.json`。

- [ ] **Step 1: 寫 failing tests**

```python
def test_collect_workspace_reads_package_message_and_topic(tmp_path: Path) -> None:
    make_package(tmp_path, "demo", message="State.msg", source='pub("/ly/demo/state")')
    index = module.collect_workspace(tmp_path)
    assert index.packages["demo"].messages["State"] == ["uint8 level"]
    assert index.topics["/ly/demo/state"].packages == {"demo"}

def test_check_reports_drift_without_writing(tmp_path: Path) -> None:
    make_workspace(tmp_path)
    module.sync(tmp_path, output(tmp_path), check=False, dry_run=False)
    (tmp_path / "src/demo/src/node.cpp").write_text('"/ly/demo/changed"')
    assert module.sync(tmp_path, output(tmp_path), check=True, dry_run=False).is_current is False
```

- [ ] **Step 2: 確認 RED**

Run: `python3 -m pytest -q tests/test_obsidian_sync.py`

Expected: FAIL，因 `scripts/obsidian_sync.py` 尚不存在。

- [ ] **Step 3: 實作最小 scanner 與 render contract**

```python
TOPIC_RE = re.compile(r"(?<![A-Za-z0-9_])(/ly(?:/[A-Za-z0-9_]+)+)")
SOURCE_SUFFIXES = {".cpp", ".cc", ".cxx", ".hpp", ".h", ".py", ".xml", ".yaml", ".yml"}

def collect_workspace(root: Path) -> WorkspaceIndex:
    # Parse src/*/package.xml; scan only each package's source, launch and config files.
    # Parse msg/*.msg after stripping comments; retain deterministic sorted paths.
    ...

def render_workspace(index: WorkspaceIndex) -> dict[PurePosixPath, str]:
    # Render Index, one package note, one topic note, one message note and sync report.
    # Every generated file starts with GENERATED_MARKER and uses vault-root wikilinks.
    ...
```

`sync` 必須只刪除先前 `.manifest.json` 列出、內容仍以 `GENERATED_MARKER` 開頭、且路徑仍在 `_generated/` 內的檔案；未標記檔案、symlink 與越界 manifest path 一律保留並在報告列出衝突。

- [ ] **Step 4: 確認 GREEN**

Run: `python3 -m pytest -q tests/test_obsidian_sync.py`

Expected: PASS，覆蓋 package、message、topic、determinism、`--check` 與手寫 note 保護。

### Task 3: 產生正式初始 vault 並提供操作文件

**Files:**
- Create: `docs/obsidian/_generated/.manifest.json`（由工具產生）
- Create: `docs/obsidian/_generated/Index.md`（由工具產生）
- Create: `docs/obsidian/_generated/Packages/*.md`（由工具產生）
- Create: `docs/obsidian/_generated/Topics/*.md`（由工具產生）
- Create: `docs/obsidian/_generated/Messages/*/*.md`（由工具產生）
- Create: `docs/obsidian/_generated/Reports/sync-report.md`（由工具產生）
- Modify: `scripts/README.md`

**Interfaces:**
- Consumes: `python3 scripts/obsidian_sync.py [--check|--dry-run]`。
- Produces: 可由 Obsidian graph 瀏覽的 source-backed notes；腳本命令入口。

- [ ] **Step 1: 生成正式內容**

Run: `python3 scripts/obsidian_sync.py`

Expected: 列出 scanned package/topic/message count 和 written note count，退出碼 0。

- [ ] **Step 2: 加入 script README 指引**

在 `scripts/README.md` 增加「Obsidian knowledge vault」小節，列出下列精確命令與用途：

```bash
python3 scripts/obsidian_sync.py
python3 scripts/obsidian_sync.py --check
python3 scripts/obsidian_sync.py --dry-run
```

- [ ] **Step 3: 驗證生成輸出與連結**

Run: `python3 scripts/obsidian_sync.py --check && python3 -m json.tool docs/obsidian/_generated/.manifest.json >/dev/null`

Expected: exit 0；manifest 是合法 JSON；無生成漂移。

### Task 4: Repository 驗證與交接

**Files:**
- Modify: 僅 Task 1–3 列出的檔案。

- [ ] **Step 1: 執行 focused tests**

Run: `python3 -m pytest -q tests/test_obsidian_sync.py`

Expected: 0 failures。

- [ ] **Step 2: 執行靜態工程驗證**

Run: `./scripts/selfcheck.sh sentry --static-only`

Expected: exit 0；若環境缺少 ROS dependency，記錄精確失敗原因而不宣稱通過。

- [ ] **Step 3: 檢查差異與格式**

Run: `git diff --check -- docs/obsidian scripts/obsidian_sync.py scripts/README.md tests/test_obsidian_sync.py && git status --short`

Expected: diff check exit 0；只報告本次新增／修改路徑，並清楚與 pre-existing dirty files 分開。

- [ ] **Step 4: 建議 commit（不自動提交）**

```bash
git add docs/obsidian scripts/obsidian_sync.py scripts/README.md tests/test_obsidian_sync.py
git commit -m "docs: add source-backed Obsidian knowledge vault"
```

僅在使用者確認要提交、且 staging area 只有上述路徑時執行。
