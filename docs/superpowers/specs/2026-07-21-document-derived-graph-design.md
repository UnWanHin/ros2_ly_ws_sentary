# Document-Derived Graph Design

Updated: 2026-07-21

## Decision

`localhost:1037` is a documentation site with three views of one source:
Documentation, Graph, and Split. The source of truth is every Markdown page
under `docs/`; graph nodes and edges are ephemeral projections, never files or
hand-maintained records.

## Projection Contract

- Each `docs/**/*.md` file is one node with its repository-relative path,
  title, category, and modification version read directly from that file.
- Markdown links and Obsidian wikilinks create edges. Directory membership is
  a derived category for grouping and filtering, not a stored graph edge.
- The server exposes a transient document projection at `/api/documents`.
  It recalculates its source version from the files and returns a new projection
  whenever a document is added, removed, or edited.
- The browser polls that version while open. It updates its documentation,
  graph, and selection state without a page reload.
- `.understand-anything/knowledge-graph.json` and all dashboard code paths
  that depend on it are retired. There is no graph database or compatibility
  endpoint.

## Presentation

The site uses one shell: a category sidebar plus Documentation, Graph, and
Split view modes. The graph is a canvas-backed, Obsidian-style force layout
with calm monochrome blue-gray categories, soft gray low-opacity edges,
hover-neighbor focus, smooth zoom/pan, draggable nodes, and a selected-document
camera focus. Double-clicking a node opens the same document in the reader.

## Scope

This changes only the local documentation website and its generated graph
artifacts. It does not alter ROS runtime behavior, documentation content, or
the simulator.
