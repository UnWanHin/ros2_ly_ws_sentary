# Config and Script Inventory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a source-verified, maintainable record of every YAML and shell entry point before any runtime configuration or command compatibility removal.

**Architecture:** A report is the canonical human audit for ownership, consumers, and precedence. An Obsidian note links the audit into the maintained vault. This slice changes documentation only; it does not change ROS source, launch arguments, YAML, or shell behaviour. If the vault check finds generated-index drift from already-committed source, regenerate and include only those deterministic index updates.

**Tech Stack:** Markdown, Obsidian wikilinks, `rg`, `git log`, ROS2 static checks.

## Global Constraints

- Preserve the formal/debug boundary: `debug_mode.yaml` only loads through `gimbal_driver/debug_node.launch.py`.
- Do not delete, move, rename, or change semantics of any YAML or shell script in this slice.
- Treat source files, launch files, package manifests, current tests, and current docs as evidence; historical documents are not sufficient deletion proof.
- Add records in Chinese and use `Updated: 2026-07-16` for current documents.
- Keep the worktree on `Behavion`; create one documentation-only commit after checks pass.

---

### Task 1: Publish the source-verified YAML and script inventory

**Files:**

- Create: `docs/reports/2026-07-16_config_script_inventory.md`

**Interfaces:**

- Consumes: YAML files under `config/` and `src/*/config/`; shell entry points under `scripts/`; current launch and wrapper source.
- Produces: a deletion-gate-ready table that later migration work can reference without inferring ownership from filenames.

- [x] **Step 1: Collect active consumers and file roles**

Run:

```bash
rg --files -g '*.yaml' -g '*.yml' | sort
rg --files scripts -g '*.sh' | sort
rg -n --glob '!build/**' --glob '!install/**' --glob '!log/**' \
  'base_config\\.yaml|common\\.yaml|override_config\\.yaml|debug_mode\\.yaml|tf_config\\.yaml|OutpostRegionalTest\\.yaml' .
```

Expected: every retained YAML has a source, launch, test, calibration-tool, or document-supported role; a short wrapper is not marked dead solely for being short.

- [x] **Step 2: Write the inventory report**

Create `docs/reports/2026-07-16_config_script_inventory.md` with these exact sections:

```markdown
# YAML 與腳本現況盤點

Updated: 2026-07-16

## 結論與邊界
## YAML owner 與 consumer
## 有效 precedence（按入口）
## Shell 入口分類
## 第二批候選與刪除 gate
## 已知架構風險（不在本批修改）
## 驗證來源
```

The YAML table must cover all 22 current YAML files. The script table must classify every current shell file by directory and purpose, explicitly list thin wrappers, and distinguish “not a deletion candidate” from “needs consumer audit”.

- [x] **Step 3: Check report completeness**

Run:

```bash
for yaml_file in $(rg --files -g '*.yaml' -g '*.yml' | sort); do
  rg -F "$yaml_file" docs/reports/2026-07-16_config_script_inventory.md >/dev/null || exit 1
done
```

Expected: exit 0; every inventory YAML path is explicitly recorded.

### Task 2: Link the audit into maintained documentation and Obsidian

**Files:**

- Modify: `docs/README.md`
- Modify: `docs/obsidian/Home.md`
- Create: `docs/obsidian/notes/2026-07-16-config-script-governance.md`

**Interfaces:**

- Consumes: the Task 1 report and existing Obsidian generated package/config notes.
- Produces: a discoverable documentation index and a human-maintained vault note; neither changes generated files.

- [x] **Step 1: Add the audit to the documentation index**

Add this link under `docs/README.md`’s current reports section:

```markdown
[reports/2026-07-16_config_script_inventory.md](reports/2026-07-16_config_script_inventory.md)
```

- [x] **Step 2: Add a vault governance note**

Create `docs/obsidian/notes/2026-07-16-config-script-governance.md` with this front matter:

```markdown
---
tags:
  - config
  - scripts
  - governance
  - source-audit
status: source-checked
updated: 2026-07-16
---
```

The note must link to the report, the gimbal driver configuration note, the behavior tree package note, the navi bridge package note, and `docs/obsidian/Home`. It must state the three phases and warn that empty compatibility config and thin wrappers are not deletion-approved.

- [x] **Step 3: Link the governance note from the vault home**

Add one bullet under `docs/obsidian/Home.md`’s “從這裡開始” section:

```markdown
- [[docs/obsidian/notes/2026-07-16-config-script-governance|YAML／腳本治理盤點]]：owner、precedence 與分批清理 gate。
```

- [x] **Step 4: Verify vault source-generated separation**

Run:

```bash
python3 scripts/obsidian_sync.py --check
```

Expected: exit 0; the manual note is not rewritten and generated index remains source-aligned. If the first check reports drift from already-committed source, run the deterministic sync once, inspect its generated-only diff, then rerun this check.

### Task 3: Validate and commit the documentation-only slice

**Files:**

- Modify: `docs/reports/2026-07-16_config_script_inventory.md`
- Modify: `docs/README.md`
- Modify: `docs/obsidian/Home.md`
- Create: `docs/obsidian/notes/2026-07-16-config-script-governance.md`

**Interfaces:**

- Consumes: Tasks 1–2 records.
- Produces: a verified, reviewable documentation commit; no runtime file changes.

- [x] **Step 1: Validate format and links**

Run:

```bash
git diff --check
rg -n 'TBD|TODO|implement later|fill in details' \
  docs/reports/2026-07-16_config_script_inventory.md \
  docs/obsidian/notes/2026-07-16-config-script-governance.md
```

Expected: `git diff --check` exits 0 and the placeholder scan produces no matches.

- [x] **Step 2: Run the project documentation/static gate**

Run:

```bash
source /home/hiraeth/Documents/DirtroBox/Ubuntu-22.04/source_sentry_env.sh
./scripts/selfcheck.sh sentry --static-only
```

Expected: exit 0 with no FAIL entries; this documents that the inventory slice did not change runtime contracts.

- [x] **Step 3: Review the staged scope and commit**

Run:

```bash
git add docs/README.md docs/obsidian/Home.md \
  docs/obsidian/_generated/Topics/ly__control.md \
  docs/obsidian/_generated/Topics/ly__control__vel.md \
  docs/obsidian/_generated/Topics/ly__navi__should_rotate.md \
  docs/obsidian/_generated/Topics/ly__navi__vel.md \
  docs/obsidian/notes/2026-07-16-config-script-governance.md \
  docs/reports/2026-07-16_config_script_inventory.md \
  docs/superpowers/plans/2026-07-16-config-script-inventory.md
git diff --cached --check
git diff --cached --stat
git commit -m "docs: inventory config and script ownership"
```

Expected: one documentation-only commit. Do not push unless the user asks.
