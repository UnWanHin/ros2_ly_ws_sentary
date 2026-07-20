#!/usr/bin/env python3
"""Serve the repository's source-checked knowledge graph and interactive relationship view."""

from __future__ import annotations

import argparse
import hashlib
import json
import mimetypes
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


ROOT = Path(__file__).resolve().parent.parent
DOCS_ROOT = ROOT / "docs"
MARKDOWN_LINK = re.compile(r"\[[^\]]*\]\(([^)]+)\)")
WIKILINK = re.compile(r"\[\[([^\]|#]+)(?:#[^\]|]+)?(?:\|[^\]]+)?\]\]")
TITLE = re.compile(r"^#\s+(.+?)\s*$", re.MULTILINE)


def read_bytes(path: Path) -> bytes:
    return path.read_bytes()


def document_id(path: Path, docs_root: Path) -> str:
    return path.relative_to(docs_root).with_suffix("").as_posix()


def title_for_document(path: Path, content: str) -> str:
    match = TITLE.search(content)
    if match is not None:
        return match.group(1).strip()
    return path.stem.replace("_", " ")


def category_for_document(path: Path, docs_root: Path) -> str:
    parts = path.relative_to(docs_root).parent.parts
    if not parts:
        return "general"
    if parts[:3] == ("obsidian", "_generated", "Topics"):
        return "ros-topics"
    if parts[:2] == ("sentry", "regional"):
        return "decision-tree"
    if parts[:2] == ("sentry", "internal"):
        return "flow"
    if parts[0] == "architecture":
        return "architecture"
    if parts[:2] == ("obsidian", "_generated"):
        return "generated-index"
    return parts[0]


def resolve_link(raw_link: str, source: Path, root: Path, docs_root: Path) -> str | None:
    link = raw_link.strip().split("#", maxsplit=1)[0].split("?", maxsplit=1)[0]
    if not link or "://" in link or link.startswith(("mailto:", "#")):
        return None
    docs_relative = link.lstrip("/")
    candidates = [source.parent / link]
    if docs_relative.startswith("docs/"):
        candidates.insert(0, root / docs_relative)
    else:
        candidates.append(docs_root / docs_relative)
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate.suffix != ".md":
            candidate = candidate.with_suffix(".md")
        try:
            if candidate.is_file():
                return document_id(candidate, docs_root)
        except ValueError:
            continue
    return None


def document_links(content: str, source: Path, root: Path, docs_root: Path) -> set[str]:
    links: set[str] = set()
    for match in MARKDOWN_LINK.finditer(content):
        target = resolve_link(match.group(1), source, root, docs_root)
        if target is not None:
            links.add(target)
    for match in WIKILINK.finditer(content):
        target = resolve_link(match.group(1), source, root, docs_root)
        if target is not None:
            links.add(target)
    return links


def build_document_projection(root: Path = ROOT) -> dict[str, object]:
    """Build an ephemeral graph projection directly from Markdown documents."""

    root = root.resolve()
    docs_root = root / "docs"
    documents = sorted(path for path in docs_root.rglob("*.md") if path.is_file())
    digest = hashlib.sha256()
    nodes: list[dict[str, object]] = []
    links_by_source: dict[str, set[str]] = {}
    for path in documents:
        content = path.read_text(encoding="utf-8", errors="replace")
        node_id = document_id(path, docs_root)
        digest.update(node_id.encode("utf-8"))
        digest.update(b"\0")
        digest.update(content.encode("utf-8"))
        links_by_source[node_id] = document_links(content, path, root, docs_root)
        nodes.append(
            {
                "id": node_id,
                "path": path.relative_to(root).as_posix(),
                "title": title_for_document(path, content),
                "category": category_for_document(path, docs_root),
                "updated_ns": path.stat().st_mtime_ns,
            }
        )

    node_ids = {str(node["id"]) for node in nodes}
    edges = [
        {"source": source, "target": target}
        for source, targets in links_by_source.items()
        for target in sorted(targets)
        if target in node_ids and target != source
    ]
    degree: dict[str, int] = {node_id: 0 for node_id in node_ids}
    for edge in edges:
        degree[str(edge["source"])] += 1
        degree[str(edge["target"])] += 1
    for node in nodes:
        node["degree"] = degree[str(node["id"])]

    categories: dict[str, int] = {}
    for node in nodes:
        category = str(node["category"])
        categories[category] = categories.get(category, 0) + 1
    return {
        "revision": digest.hexdigest(),
        "nodes": nodes,
        "edges": edges,
        "categories": [{"id": key, "count": value} for key, value in sorted(categories.items())],
    }


def document_content(root: Path, node_id: str) -> dict[str, str] | None:
    projection = build_document_projection(root)
    node = next((item for item in projection["nodes"] if item["id"] == node_id), None)
    if not isinstance(node, dict):
        return None
    path = (root / str(node["path"])).resolve()
    try:
        path.relative_to((root / "docs").resolve())
    except ValueError:
        return None
    return {"id": node_id, "title": str(node["title"]), "content": path.read_text(encoding="utf-8", errors="replace")}


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "LYUnderstandGraph/1.0"

    def log_message(self, format: str, *args: object) -> None:
        print("[understand-graph] " + (format % args))

    def send_file(self, path: Path, content_type: str) -> None:
        try:
            content = read_bytes(path)
        except FileNotFoundError:
            self.send_error(404, f"Missing source: {path.relative_to(ROOT)}")
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def send_json(self, payload: object) -> None:
        content = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    @property
    def root(self) -> Path:
        return Path(getattr(self.server, "root", ROOT)).resolve()

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API.
        request = urlparse(self.path)
        path = request.path
        if path == "/api/documents":
            self.send_json(build_document_projection(self.root))
            return
        if path == "/api/document":
            node_id = parse_qs(request.query).get("id", [""])[0]
            document = document_content(self.root, node_id)
            if document is None:
                self.send_error(404, "Unknown documentation page")
            else:
                self.send_json(document)
            return
        routes = {
            "/": (ROOT / "scripts" / "understand_graph_dashboard.html", "text/html; charset=utf-8"),
            "/index.html": (ROOT / "scripts" / "understand_graph_dashboard.html", "text/html; charset=utf-8"),
        }
        target = routes.get(path)
        if target is None:
            self.send_error(404, "Unknown dashboard route")
            return
        self.send_file(*target)


def create_server(host: str, port: int, root: Path = ROOT) -> ThreadingHTTPServer:
    """Create a dashboard server with a request-time documentation source."""

    server = ThreadingHTTPServer((host, port), DashboardHandler)
    server.root = root.resolve()  # type: ignore[attr-defined]
    return server


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Serve the local read-only dashboard with project, Regional, and node-edge graph views."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind address (default: 127.0.0.1).")
    parser.add_argument("--port", type=int, default=1037, help="TCP port (default: 1037).")
    args = parser.parse_args()

    if not DOCS_ROOT.is_dir():
        raise SystemExit(f"Documentation root not found: {DOCS_ROOT}")

    server = create_server(args.host, args.port)
    dashboard_url = f"http://{args.host}:{args.port}/"
    print(f"Understand graph dashboard started: {dashboard_url}")
    print(f"Open this URL in your browser: {dashboard_url}")
    print("Views: Documentation, Graph, and Split; every view derives from docs/**/*.md.")
    print("Read-only source: current Markdown documents. No graph database is used.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nDashboard stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
