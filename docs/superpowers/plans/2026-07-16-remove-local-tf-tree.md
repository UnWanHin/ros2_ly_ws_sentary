# Remove Local tf_tree Fallback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the unneeded in-repository `tf_tree` fallback while retaining external `sentry_tf` as the sole TF provider.

**Architecture:** `navi_tf_bridge` continues to consume the external TF tree; no in-workspace node publishes gimbal TF. Launch composition and operator scripts stop accepting a local TF fallback mode, avoiding duplicate ownership of the same frames.

**Tech Stack:** ROS 2 Humble launch Python, Bash, CMake/ament, Markdown, JSON graph artifacts.

## Global Constraints

- External `/tf` and `/tf_static` frame/topic contracts remain unchanged.
- Do not change behavior-tree, navigation, FaceMode, or gimbal topic/message interfaces.
- Preserve only current behavior: external `sentry_tf` is required for normal and debug operation.
- Use `apply_patch` for all repository modifications.

---

### Task 1: Remove the local runtime package and launch fallback contract

**Files:**
- Delete: `src/tf_tree/`
- Modify: `src/behavior_tree/launch/sentry_all.launch.py`
- Modify: `src/navi_tf_bridge/launch/map_aim_point.launch.py`
- Modify: `src/behavior_tree/launch/competition_autoaim.launch.py`
- Modify: `src/behavior_tree/launch/showcase.launch.py`
- Modify: `src/behavior_tree/launch/chase_only.launch.py`
- Modify: `src/behavior_tree/launch/outpost_regional_test.launch.py`

**Interfaces:**
- Consumes: externally published gimbal TF from `sentry_tf`.
- Produces: launch APIs with no `use_tf_tree`, `tf_tree_params_file`, or `resolved_tf_tree_params_file` argument.

- [x] Remove package discovery, fallback defaults, launch arguments, logs, resolver function and conditional include.
- [x] Remove wrapper forwarding of the deleted arguments.
- [x] Verify every affected launch Python file compiles:

```bash
python3 -m py_compile src/behavior_tree/launch/*.launch.py src/navi_tf_bridge/launch/*.launch.py
```

- [x] Commit the runtime slice after verification:

```bash
git add -u src
git commit -m "launch: remove local tf_tree fallback"
```

### Task 2: Remove the retired operator modes and update static self-check

**Files:**
- Modify: `scripts/navi/map_aim_point_test.sh`
- Modify: `scripts/navi/chase.sh`
- Modify: `scripts/aim/Outpost_Simlator.sh`
- Modify: `scripts/aim/outpost_regional.sh`
- Modify: `scripts/selfcheck/pc.sh`
- Modify: `scripts/selfcheck/sentry.sh`

**Interfaces:**
- Consumes: external `sentry_tf` started by the external stack.
- Produces: scripts that never inject removed launch arguments and self-check package lists without `tf_tree`.

- [x] Delete `--with-tf-tree` / `--without-tf-tree` parsing and launch forwarding; update help text to state external TF is required.
- [x] Remove the obsolete dual-owner runtime check that names the deleted local node.
- [x] Run shell syntax checks:

```bash
bash -n scripts/navi/map_aim_point_test.sh scripts/navi/chase.sh scripts/aim/Outpost_Simlator.sh scripts/aim/outpost_regional.sh scripts/selfcheck/pc.sh scripts/selfcheck/sentry.sh
```

- [x] Commit the script slice after verification:

```bash
git add scripts
git commit -m "scripts: require external sentry tf"
```

### Task 3: Refresh documentation and knowledge artifacts

**Files:**
- Modify: `README.md`, `AGENTS.md`, `docs/README.md`, current architecture/module/guide docs, and scripts docs that describe `tf_tree` as current behavior.
- Modify: `.understand-anything/knowledge-graph.json`, `.understand-anything/project-knowledge-graph.md`, `.understand-anything/meta.json`, `.understand-anything/intermediate/scan-result.json`.
- Delete: generated Obsidian package/topic pages only owned by `tf_tree`.
- Create: `docs/record/2026-07-16_remove_local_tf_tree.md`.

**Interfaces:**
- Produces: one source-checked map with no `tf_tree` package/fallback edge and a durable record explaining external TF ownership.

- [x] Update current-state documents and graph metadata to the actual HEAD/working-tree state.
- [x] Leave historical records intact, labelling them historical only where a current reader could otherwise mistake them for active behavior.
- [x] Validate all changed JSON and graph references:

```bash
python3 -m json.tool .understand-anything/knowledge-graph.json >/dev/null
python3 -m json.tool .understand-anything/meta.json >/dev/null
python3 -m json.tool .understand-anything/intermediate/scan-result.json >/dev/null
rg -n --glob '!docs/record/**' --glob '!docs/plans/**' 'tf_tree|use_tf_tree|tf_tree_params_file' README.md AGENTS.md docs scripts src .understand-anything
```

### Task 4: Build, static contract verification, review, and delivery

**Files:**
- Verify the full staged diff only; no new source files.

- [x] Build maintained packages:

```bash
source /opt/ros/humble/setup.bash
source /home/hiraeth/sentry.common/install/setup.bash
colcon build --packages-select auto_aim_common gimbal_driver navi_tf_bridge behavior_tree simulator
```

- [x] Load the result and run static contract checks:

```bash
source install/setup.bash
./scripts/selfcheck.sh sentry --static-only
git diff --check
```

- [x] Review changed files for launch dependency, external TF contract, documentation/graph agreement, shell behavior, and unwanted interface changes.
- [ ] Commit the documentation/graph slice and push only after all checks pass.
