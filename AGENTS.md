# Repository Guidelines

## Project Structure & Module Organization
This repository is a ROS2 workspace built with `colcon`.
- `src/`: retained runtime packages (`auto_aim_common`, `behavior_tree`, `gimbal_driver`, `navi_tf_bridge`, `tf_tree`, `simulator`).
- `scripts/`: operational scripts (for example, `selfcheck.sh`, `start.sh`).
- `docs/`: contributor-facing documentation, organized by `architecture/`, `guides/`, `modules/`, `sentry/`, `reports/`, `rules/`.
- Generated artifacts: `build/`, `install/`, `log/` (do not commit).

## Build, Test, and Development Commands
- `colcon build`  
  Build all ROS2 packages in this workspace.
- `colcon build --packages-select auto_aim_common gimbal_driver navi_tf_bridge behavior_tree tf_tree simulator`
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

For non-trivial work, combine the repo's default `$cautious-super-engineer` style with the installed Matt Pocock skills and the Understand Anything graph:

- Read `README.md`, `docs/README.md`, and `docs/agents/domain.md` before changing runtime links, launch files, behavior-tree logic, message semantics, or simulator contracts.
- Use `$zoom-out` when you need a module/caller map or when the task touches unfamiliar code paths.
- Consult `.understand-anything/project-knowledge-graph.md` for the quick architecture map and `.understand-anything/knowledge-graph.json` for structured package/topic/file relationships.
- Treat the graph as guidance, not authority. Source files, launch files, package manifests, and current docs remain authoritative.
- If graph evidence conflicts with source evidence, trust source evidence, update the graph, and report the mismatch.
- Before declaring non-trivial runtime/link/interface/architecture work complete, re-check the relevant docs and graph against current source evidence. If they are stale, update them in the same change and report the verification used.

### Understand Anything graph

Use Understand Anything as the long-lived project map for this workspace.

- Preferred skills when available in the current runtime: `$understand`, `$understand-chat`, `$understand-dashboard`, `$understand-diff`, `$understand-domain`, `$understand-explain`, and `$understand-onboard`.
- Canonical graph outputs live under `.understand-anything/`:
  - `knowledge-graph.json` for structured graph consumers
  - `project-knowledge-graph.md` for the human-readable Mermaid overview
  - `intermediate/scan-result.json` for scan inventory
  - `meta.json` for analyzed commit metadata
- Update or regenerate the graph when changing ROS package boundaries, launch composition, topic publishers/subscribers, message schemas, behavior-tree decision outputs, navigation/FaceMode flows, simulator trace contracts, or architecture docs.
- Treat graph freshness as part of the contract: keep `project-knowledge-graph.md`, `knowledge-graph.json`, and `meta.json` aligned with the current source-verified architecture, current HEAD, working tree status, and graph shape. Do not leave stale `Generated`, `Checked against HEAD`, `lastCheckedNote`, node counts, or edge counts after graph-relevant work.
- Prefer Chinese output for generated summaries in this repo (`--language zh`) unless the user asks otherwise.
- If the installed Understand Anything skill cannot run because its plugin root/core package is unavailable, create or update an Understand Anything-compatible fallback graph from repo docs, `package.xml`, launch files, topic definitions, and key source files. State clearly that fallback mode was used.
- Do not let graph generation alter ROS runtime behavior. Keep graph updates as analysis artifacts unless the user explicitly asks for runtime changes.

### Interactive graph browser maintenance

- `scripts/understand_graph_dashboard.py` is the single local entry point for the read-only interactive graph browser; it serves `scripts/understand_graph_dashboard.html` plus the canonical `.understand-anything/` graph files.
- Maintain the browser as a layered navigator, not a single all-node canvas: it must keep engineering-domain entry points, ROS topic catalog, data-processing catalog, message-schema catalog, Regional/referee catalogs, node detail pages, and one-hop local relation views.
- Keep the dedicated `Regional 流程圖` current with the actual BT tick order, task-layer priority, event/reached handling, navigation outputs, and posture handoff. Each flow block must link to a current source file, ROS topic, config, or Regional document; update this view in the same change whenever those decision semantics change.
- A node detail page must let users jump to every directly connected source/target. ROS topic nodes must expose their publisher/subscriber relations from graph edges; data-processing nodes must expose their input/output relations. Do not replace these with unlinked prose.
- Whenever graph-relevant work changes a package/topic/message schema/processing edge/decision flow, update the corresponding nodes, edges, summaries, tags, and layer membership in `.understand-anything/knowledge-graph.json` so the browser categories and node-to-node navigation remain correct. Update `project-knowledge-graph.md` and `meta.json` in the same change.
- When changing the browser itself, verify the served root page, `/graph.json`, `/project.md`, and `/regional.md`; validate inline JavaScript syntax, use `git diff --check`, and keep it dependency-free unless the user explicitly approves a new frontend dependency.

### Documentation and graph freshness

- Runtime behavior, ROS topic/msg/param semantics, launch composition, behavior-tree decisions, simulator trace contracts, navigation/FaceMode flows, and embedded serial mappings must not leave stale docs or stale graph entries behind.
- When source evidence changes a documented behavior, update the closest current docs under `docs/` and any dated `Updated: YYYY-MM-DD` line in the same change. Historical reports may keep old analysis only if a clear current-status note explains what has been superseded.
- When changing docs that describe architecture or interfaces, also check whether `.understand-anything/` must be updated. Architecture docs and graph files should agree on current package/topic/message/decision-flow facts.
- Keep `.understand-anything/knowledge-graph.json`, `.understand-anything/project-knowledge-graph.md`, and `.understand-anything/meta.json` mutually consistent; also update `.understand-anything/intermediate/scan-result.json` when package/topic inventory is regenerated.
- Before final response for graph-relevant work, validate JSON graph files with `python3 -m json.tool` or `jq`, run `git diff --check`, and run at least `./scripts/selfcheck.sh sentry --static-only`; use `./scripts/selfcheck.sh sentry --skip-hz` or launched self-check when runtime graph evidence is required.
- If a full graph regeneration cannot run, keep the fallback graph update source-driven: cite source files, launch files, package manifests, and current docs used; record fallback mode in `meta.json` / graph notes.

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
  - `.understand-anything/` graph artifacts and the applicable protocol brief under `docs/plans/`
- Preserve the no-subscriber fast path for raw publishers. Raw observation must not add message allocation or
  publication work when no topic consumer is connected.

## Skill Auto-Match & Auto-Install
- Automatically match and use the minimal relevant skill set when user intent clearly maps to available skills.
- Prefer Understand Anything for codebase orientation, architecture graphing, onboarding maps, graph-backed explanations, and diff impact analysis. Use the existing graph first; regenerate only when it is missing, stale, or the task changes graph-relevant interfaces.
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
