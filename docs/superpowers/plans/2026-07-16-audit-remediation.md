# Configuration Audit Remediation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove misleading configuration behavior, repair the affected debug/test paths, and bring source-backed documentation and graph metadata to the current workspace state.

**Architecture:** Keep package YAML ownership unchanged while preserving the legacy root `base_config_file` and `config_file` interface. First trace and retain every existing consumer's effective behavior; introduce node-scoped compatibility translation only where source evidence proves a legacy key must still reach a node. The navigation bridge receives the resolved goal-pose argument, direct gimbal tests pass their full YAML as the driver baseline, and regional test profiles remain alive until their child launch exits.

**Tech Stack:** ROS 2 Humble launch Python, Bash, Python standard library, JSON, colcon.

## Global Constraints

- Do not inject root global YAML into `gimbal_driver` or weaken the formal/debug boundary.
- Preserve ROS topics, message types, and existing debug CLI names.
- Add a failing regression check before each behavior fix.
- Keep graph artifacts source-backed and mutually consistent.

---

### Task 1: Preserve and make legacy root config arguments effective

**Files:**
- Modify: `src/behavior_tree/launch/sentry_all.launch.py`
- Modify: `scripts/launch/start_sentry_all.sh`
- Modify: `scripts/selfcheck/sentry.sh`
- Modify: current architecture/config documentation

- [x] Add a static contract test that fails while `config_file`/`base_config_file` are described as active overlays.
- [x] Trace the actual keys in legacy files and route only supported keys to their owning node without injecting global YAML into unrelated nodes.
- [x] Keep wrapper defaults and direct `base_config_file:=...` / `config_file:=...` invocation working; log the resolved owner mapping.
- [x] Verify `--show-args`, compatibility invocation, parameter effect, and static selfcheck.

### Task 2: Repair effective navigation and gimbal debug parameters

**Files:**
- Modify: `src/behavior_tree/launch/sentry_all.launch.py`
- Modify: `scripts/lib/gimbal_test_lifecycle.sh`
- Modify: `scripts/selfcheck/sentry.sh`

- [x] Add source contracts that fail when `navi_publish_goal_pose` is not forwarded to the bridge and when lifecycle scripts put a full baseline YAML in `config_file`.
- [x] Forward `navi_publish_goal_pose` to the bridge and pass test YAML as `base_config_file`.
- [x] Run virtual-driver smoke checks with and without the direct test wrapper, proving `use_virtual=true` reaches `IODevice`.

### Task 3: Make generated regional profile cleanup exception-safe

**Files:**
- Modify: `scripts/areatest/regional_area_test.sh`
- Modify: `scripts/areatest/test_regional_area_profile.py`

- [x] Add failing tests for a generator error and for cleanup ordering in the fake/mock launch branch.
- [x] Install cleanup immediately after mktemp; wait for `LAUNCH_PID` after signal delivery before unlinking the profile.
- [x] Verify all normal/pure profile combinations, cleanup behavior, and shell syntax.

### Task 4: Refresh graph and operator records

**Files:**
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/meta.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/intermediate/scan-result.json` when inventory is regenerated
- Modify: closest architecture/module/script/Obsidian records

- [x] Regenerate or source-update current HEAD, working-tree state, generated timestamp, and graph counts consistently.
- [x] Correct source-inconsistent precedence and path statements, including the scope of `debug_node`.
- [x] Validate every JSON artifact, Obsidian sync, targeted build, static selfcheck, and `git diff --check`.
