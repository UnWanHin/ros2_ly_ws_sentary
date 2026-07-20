# Document-Derived Graph Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the standalone graph database with a live documentation-derived Graph view at `localhost:1037`.

**Architecture:** `scripts/understand_graph_dashboard.py` scans `docs/**/*.md` at request time and returns an ephemeral document projection. The existing single-page dashboard renders documentation, graph, or split mode from that projection; no graph data is persisted.

**Tech Stack:** Python standard library HTTP server, dependency-free HTML/CSS/Canvas/JavaScript, pytest and Playwright.

## Global Constraints

- `docs/**/*.md` is the only source of truth for graph nodes and edges.
- Do not create, serve, or maintain `graph.json` or another graph database.
- Existing documentation pages must be directly readable and graph-selectable at the same URL.
- Source changes must appear without a page reload.

### Task 1: Document Projection Contract

**Files:**
- Modify: `scripts/understand_graph_dashboard.py`
- Create: `tests/test_understand_graph_dashboard.py`

- [ ] Add failing tests for Markdown discovery, title/category derivation,
  Markdown and Obsidian link resolution, and an edit/add/remove version change.
- [ ] Implement an in-memory-only `build_document_projection()` result used by
  `/api/documents`; do not read `.understand-anything/knowledge-graph.json`.
- [ ] Verify the tests and the HTTP endpoint, then commit the contract slice.

### Task 2: Unified Documentation Shell

**Files:**
- Modify: `scripts/understand_graph_dashboard.py`
- Modify: `scripts/understand_graph_dashboard.html`
- Modify: `tests/test_understand_graph_dashboard.py`

- [ ] Replace old graph-specific fetches with the document projection.
- [ ] Render the derived category sidebar and Documentation/Graph/Split modes.
- [ ] Make document selection and graph selection one shared state; double-click
  opens the same document reader.
- [ ] Implement canvas force layout, zoom/pan, drag, focus fade, category
  clustering, and live projection polling.
- [ ] Verify desktop and narrow browser interaction with Playwright.

### Task 3: Retire the Duplicate Graph Source

**Files:**
- Delete: `.understand-anything/knowledge-graph.json`
- Delete: `.understand-anything/project-knowledge-graph.md`
- Delete: `.understand-anything/meta.json`
- Delete: `.understand-anything/intermediate/scan-result.json`
- Modify: `AGENTS.md`, `README.md`, `docs/README.md`, `docs/obsidian/*`, and
  dashboard/tool documentation that describes the old artifacts.

- [ ] Remove runtime and current-document references to the retired graph
  artifacts; historical records remain historical and need not be rewritten.
- [ ] Update the Library entrypoint description to call it a documentation
  website with derived graph view.
- [ ] Run dashboard HTTP, unit, browser, Obsidian-sync, diff, and static checks;
  commit the migration.
