# Repository Guidelines

## Project Structure & Module Organization
This repository is a ROS2 workspace built with `colcon`.
- `src/`: retained runtime packages (`auto_aim_common`, `behavior_tree`, `gimbal_driver`, `navi_tf_bridge`, `simulator`).
- `scripts/`: operational scripts (for example, `selfcheck.sh`, `start.sh`).
- `docs/`: contributor-facing documentation, organized by `architecture/`, `guides/`, `modules/`, `sentry/`, `reports/`, `rules/`.
- Generated artifacts: `build/`, `install/`, `log/` (do not commit).

## Build, Test, and Development Commands
- `colcon build`  
  Build all ROS2 packages in this workspace.
- `colcon build --packages-select auto_aim_common gimbal_driver navi_tf_bridge behavior_tree simulator`
  Faster iterative build for selected modules.
- `source install/setup.bash`  
  Load built packages into the current shell.
- `./scripts/selfcheck.sh sentry --skip-hz`  
  Static + graph contract checks (without frequency sampling).
- `./scripts/selfcheck.sh sentry --launch --wait 10`  
  Launch stack and run end-to-end runtime checks.
- `colcon test && colcon test-result --verbose`  
  Run package tests where available.

## Coding Style & Naming Conventions
- Primary languages: C++ (ROS2 nodes), Python (launch/scripts), YAML/JSON (config).
- Follow existing file-level style; do not reformat unrelated code.
- Use descriptive names aligned with existing patterns:
  - Topics: `/ly/<domain>/<name>` (snake_case).
  - Config keys: preserve existing public topic/message/config names when touching runtime interfaces.
- Avoid hardcoded hardware values (device name, baud rate); keep them in YAML.

## Testing Guidelines
- Minimum before PR: targeted build for changed packages + `selfcheck.sh sentry --skip-hz`.
- For runtime/link changes, include one launched self-check run or explain why unavailable.
- If changing topics/messages/params, validate publisher-subscriber contracts and update docs.

## Commit & Pull Request Guidelines
- Recent history favors short, module-focused messages (often Chinese/English mixed), e.g. `posture 寫入bt樹`, `相機硬參數yaml化`.
- Recommended commit format: `<module>: <what changed>` (concise, imperative).
- PRs should include:
  - Scope and motivation
  - Affected packages/files
  - Verification commands and results
  - Config/doc updates (especially under `docs/`) when interfaces change.

## Security & Configuration Tips
- Never commit secrets, device-specific credentials, or local absolute paths.
- Keep shared runtime configuration in `config/base_config.yaml`, `config/common.yaml`, and the owning package configuration files.

## Simulator Maintenance
- `src/simulator` is the maintained offline pygame viewer for behavior-tree decision traces; keep it in `src/`, not `tools/`.
- When changing behavior-tree decision outputs, trace schema, navigation goal IDs/coordinates, posture fields, target fields, or unit state fields, update in the same change:
  - `src/behavior_tree/src/DecisionTrace.cpp`
  - `src/simulator/simulator/model.py`
  - `src/simulator/simulator/trace.py`
  - `src/simulator/simulator/validation.py` when the new output affects offline checks
  - `src/simulator/config/default.yaml` when map, field, point, unit, layer, or style assumptions change
  - the current simulator docs under `docs/sentry/`, currently `docs/sentry/internal/simulator.md`
- Dated simulator docs must include an `Updated: YYYY-MM-DD` line.

## Default Engineer Mode (Project)
- Default to the `$cautious-super-engineer` working style for all tasks unless the user explicitly overrides it.
- Understand architecture, data/call chain, and current behavior before making edits.
- Apply minimal local diffs only; do not refactor structure, move modules, or rename interfaces unless explicitly requested.
- Do not add broad defensive guards, fallback layers, retries, or compatibility shims unless a concrete failure mode is proven and approved.
- Do not perform chain/link rerouting or coverage-style rewiring unless explicitly required and validated.
- Preserve external contracts (topics, messages, APIs, config keys) unless interface change is the explicit task.

## Agent skills

### Issue tracker

Issues and PRDs are tracked in GitHub Issues for `HUSTLYRM/2026_sentry`; use `gh` from inside this clone. See `docs/agents/issue-tracker.md`.

### Triage labels

Use the default Matt Pocock triage roles: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, and `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

This is a single-context ROS2 sentry workspace; read the repo-level docs first, and treat `CONTEXT.md` / `docs/adr/` as optional lazy-created docs. See `docs/agents/domain.md`.

### Required project-understanding workflow

For non-trivial work, combine the repo's default `$cautious-super-engineer` style with the installed Matt Pocock skills and the current documentation:

- Read `README.md`, `docs/README.md`, and `docs/agents/domain.md` before changing runtime links, launch files, behavior-tree logic, message semantics, or simulator contracts.
- Use `$zoom-out` when you need a module/caller map or when the task touches unfamiliar code paths.
- Use `./tools/Library.sh` when a document/relationship view helps; it derives its Graph and Split views directly from `docs/**/*.md`.
- Treat the graph as a visualization of current documentation, not an independent authority. Source files, launch files, package manifests, and current docs remain authoritative.
- If a document conflicts with source evidence, trust source evidence and update the affected document in the same change.
- Before declaring non-trivial runtime/link/interface/architecture work complete, re-check the relevant docs against current source evidence and update stale pages in the same change.

### Documentation-derived graph

`docs/**/*.md` is the only persisted source for the documentation graph.

- Every Markdown page under `docs/` is one node. Markdown links and Obsidian wikilinks are the only graph edges.
- Do not create, keep, or manually edit a graph JSON/database/node list/edge list. The server creates a transient in-memory projection for `/api/documents` on demand.
- Adding, editing, or deleting a documentation page must be enough to add, update, or remove its graph node. Do not add a second synchronization step.
- For package/topic/message/decision-flow changes, update the nearest current Markdown documentation. `localhost:1037` will reflect it automatically.

### Interactive graph browser maintenance

- `scripts/understand_graph_dashboard.py` is the single local entry point for the documentation website. It serves `scripts/understand_graph_dashboard.html` with Documentation, Graph, and Split views.
- Keep the browser as a layered document navigator, not a separate graph application. Category navigation, full-page reading, and graph selection must share the same selected document state.
- Keep `docs/sentry/regional/2026-07-12_regional_decision_graph.md` current with the actual BT tick order, task-layer priority, event/reached handling, navigation outputs, and posture handoff. Its Markdown links become graph relationships automatically.
- When changing the browser, verify `/`, `/api/documents`, and `/api/document?id=<known-doc-id>`; `/graph.json` must not exist. Validate inline JavaScript through the browser tests, use `git diff --check`, and keep it dependency-free unless the user explicitly approves a new frontend dependency.

### Documentation and graph freshness

- Runtime behavior, ROS topic/msg/param semantics, launch composition, behavior-tree decisions, simulator trace contracts, navigation/FaceMode flows, and embedded serial mappings must not leave stale documentation behind.
- When source evidence changes a documented behavior, update the closest current docs under `docs/` and any dated `Updated: YYYY-MM-DD` line in the same change. Historical reports may keep old analysis only if a clear current-status note explains what has been superseded.
- Documentation changes need no graph regeneration: the document website calculates the live projection on every API request and the browser polls it while open.
- Before final response for graph-relevant work, run the dashboard unit/browser tests, verify the live API routes, run `git diff --check`, and run at least `./scripts/selfcheck.sh sentry --static-only` for runtime-adjacent changes.

### Serial Protocol Observability

- `io_config.serial_mode` in `src/gimbal_driver/config/gimbal_driver_config.yaml` controls per-ID raw serial ROS topics.
  - Upload raw: `/ly/upload/typeid0` through `/ly/upload/typeid10`.
  - Download raw: `/ly/download/typeid0x00` through `/ly/download/typeid0x04`.
  - All carry `gimbal_driver/msg/GimbalRawFrame` with `header.stamp`; existing `/ly/game/*` and
    `/ly/gimbal/*` semantic topics remain the runtime interface and must not be renamed for raw tracing.
- When adding or changing any serial `TypeID` / `DownlinkTypeID`, update in the same change:
  - `src/gimbal_driver/module/BasicTypes.hpp` and `src/gimbal_driver/main.cpp`
  - `src/gimbal_driver/config/gimbal_driver_config.yaml` SerialMode `upload.typeidN` / `download.typeid0xNN` switches
  - `docs/sentry/embedded/serial_data_mapping.md` and/or `docs/sentry/embedded/downlink_control_frame.md`
  - the closest current protocol brief under `docs/`
- Preserve the no-subscriber fast path for raw publishers. Raw observation must not add message allocation or
  publication work when no topic consumer is connected.

## Skill Auto-Match & Auto-Install
- Automatically match and use the minimal relevant skill set when user intent clearly maps to available skills.
- Prefer the documentation website at `localhost:1037` for codebase orientation, architecture graphing, onboarding maps, graph-backed explanations, and diff impact analysis. It is derived directly from the current Markdown pages.
- Prefer the installed Matt Pocock skills when they match: `$diagnose` for bugs/failures, `$tdd` for test-first work, `$triage` for issue workflow, `$to-issues` for breaking plans into issues, `$to-prd` for PRDs, `$improve-codebase-architecture` for architecture work, `$zoom-out` for broader context, `$grill-me` / `$grill-with-docs` for stress-testing plans, `$handoff` for handoff summaries, and `$caveman` only when the user asks for terse mode.
- Prefer the installed Addy Osmani skills as secondary engineering review tools when they match:
  - `$code-review-and-quality` for risk reviews, pre-merge review, and multi-axis checks of correctness/readability/architecture/security/performance.
  - `$doubt-driven-development` for non-trivial claims about runtime chains, fallback behavior, safety-critical decisions, or cross-module invariants; use it to actively look for wrong assumptions before declaring a conclusion.
  - `$code-simplification` when a fix would otherwise add broad duplicated logic; preserve behavior and use the smallest local helper that improves readability.
  - `$incremental-implementation` for staged changes touching ROS2 launch/runtime paths, behavior-tree decisions, or large config/doc migrations.
  - `$deprecation-and-migration` when deciding whether a legacy subsystem should remain, be marked legacy, or be removed.
  - `$api-and-interface-design` when changing ROS topics, message semantics, launch arguments, config keys, or public package interfaces.
  - `$source-driven-development` when correctness depends on current official documentation for an external framework/library.
  - `$documentation-and-adrs` when decisions need durable docs or ADRs.
- If a required skill is missing locally, automatically try to install it with `$skill-installer` (curated first, then explicit GitHub path if needed).
- Prefer trusted/local sources in this order when possible: workspace/local skills, curated skills, pinned GitHub repo path.
- If auto-install fails (network, permission, missing repo, incompatible skill), continue with the best fallback workflow instead of blocking the task.
- Do not run destructive or privileged actions for skill installation/configuration unless explicitly requested by the user.
- Briefly report which skill was auto-matched/auto-installed and whether fallback mode was used.
