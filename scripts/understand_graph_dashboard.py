#!/usr/bin/env python3
"""Serve the repository's source-checked knowledge graph as a local dashboard."""

from __future__ import annotations

import argparse
import json
import mimetypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parent.parent
GRAPH_FILE = ROOT / ".understand-anything" / "knowledge-graph.json"
PROJECT_DOC = ROOT / "docs" / "architecture" / "2026-07-12_project_link_graph.md"
REGIONAL_DOC = ROOT / "docs" / "sentry" / "regional" / "2026-07-12_regional_decision_graph.md"


def read_bytes(path: Path) -> bytes:
    return path.read_bytes()


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

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API.
        path = urlparse(self.path).path
        routes = {
            "/": (ROOT / "scripts" / "understand_graph_dashboard.html", "text/html; charset=utf-8"),
            "/index.html": (ROOT / "scripts" / "understand_graph_dashboard.html", "text/html; charset=utf-8"),
            "/graph.json": (GRAPH_FILE, "application/json; charset=utf-8"),
            "/project.md": (PROJECT_DOC, "text/markdown; charset=utf-8"),
            "/regional.md": (REGIONAL_DOC, "text/markdown; charset=utf-8"),
        }
        target = routes.get(path)
        if target is None:
            self.send_error(404, "Unknown dashboard route")
            return
        self.send_file(*target)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Serve .understand-anything/knowledge-graph.json as a local read-only dashboard."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind address (default: 127.0.0.1).")
    parser.add_argument("--port", type=int, default=8765, help="TCP port (default: 8765).")
    args = parser.parse_args()

    if not GRAPH_FILE.is_file():
        raise SystemExit(f"Graph file not found: {GRAPH_FILE}")
    for doc in (PROJECT_DOC, REGIONAL_DOC):
        if not doc.is_file():
            raise SystemExit(f"Graph documentation not found: {doc}")

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    print(f"Understand graph dashboard: http://{args.host}:{args.port}/")
    print("Read-only sources: .understand-anything/knowledge-graph.json and docs graph pages.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nDashboard stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
