# Navi False Move Posture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a fresh `/ly/navi/should_rotate=false` request Move posture while preserving existing posture cooldown, pending, and FireCode behavior.

**Architecture:** Add one additive `NaviRotateControl` boolean and a small pure freshness predicate. `Application::UpdatePostureCommand()` uses that predicate to override only the current tick's desired posture before passing it to the existing `PostureManager`; the manager remains the sole owner of cooldown, hold, pending, retry, and feedback behavior.

**Tech Stack:** ROS2 Humble, C++20, rclcpp parameters, GoogleTest, YAML, Understand Anything fallback graph.

## Global Constraints

- Preserve all existing ROS topic, message, launch, and CLI contracts.
- Do not alter the existing `should_rotate` FireCode/FollowMode handling.
- A stale or true message cancels only an unsent Move request; already pending commands retain existing manager semantics.
- Use the existing five-second posture cooldown configured by `Posture.SwitchCooldownSec`.

---

### Task 1: Prove the fresh-false predicate

**Files:**
- Create: `src/behavior_tree/include/NaviRotatePosture.hpp`
- Create: `src/behavior_tree/test/test_navi_rotate_posture.cpp`
- Modify: `src/behavior_tree/CMakeLists.txt`

**Interfaces:**
- Consumes: `LangYa::NaviRotateControlSetting`, receipt state, receipt time, current time, and `should_rotate` value.
- Produces: `BehaviorTree::ShouldRequestMovePostureWhenNaviFalse(...) -> bool`.

- [x] **Step 1: Write failing tests** for enabled fresh false, true, stale false, and disabled feature.
- [x] **Step 2: Run** `colcon test --packages-select behavior_tree --ctest-args -R test_navi_rotate_posture --output-on-failure`; expected compilation failure before the predicate exists.
- [x] **Step 3: Implement** the pure predicate with the same freshness boundary as `NaviRotateControl.FreshTimeoutMs`.
- [x] **Step 4: Re-run** the targeted test; expected all four cases pass.

### Task 2: Connect the additive configuration to posture selection

**Files:**
- Modify: `src/behavior_tree/module/BasicTypes.hpp`
- Modify: `src/behavior_tree/src/Configuration.cpp`
- Modify: `src/behavior_tree/src/PostureLogic.cpp`
- Modify: `src/behavior_tree/config/NaviRotateControl.yaml`

**Interfaces:**
- Consumes: `NaviRotateControl.SetPostureToMoveWhenFalse` from YAML or either supported rclcpp override spelling.
- Produces: a per-tick Move desired posture only for a fresh false input.

- [x] **Step 1: Add** the false-by-default configuration member and load/override/log it alongside its sibling navigation settings.
- [x] **Step 2: Call** the tested predicate immediately after normal desired-posture selection and before `PostureManager::Tick()`.
- [x] **Step 3: Enable** the option explicitly in the formal YAML and document the cancellation semantics in its comment.
- [x] **Step 4: Build and test** `behavior_tree`.

### Task 3: Synchronize project knowledge and validate

**Files:**
- Modify: `docs/modules/2026-05-05_behavior_tree.md`
- Modify: `docs/sentry/internal/ros2_topic_structure.md`
- Modify: `.understand-anything/knowledge-graph.json`
- Modify: `.understand-anything/project-knowledge-graph.md`
- Modify: `.understand-anything/meta.json`

- [x] **Step 1: Document** the formal posture request and its cancellation boundary without claiming it cancels pending downlink commands.
- [x] **Step 2: Update** the source-checked fallback graph/config-topic summaries and freshness metadata.
- [x] **Step 3: Run** targeted build/test, `./scripts/selfcheck.sh sentry --static-only`, JSON validation, Obsidian sync check, and `git diff --check`.
- [x] **Step 4: Commit and push** the focused verified change without staging unrelated user changes.
