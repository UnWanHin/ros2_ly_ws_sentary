from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from threading import Thread
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest


def load_dashboard_module():
    source = Path(__file__).resolve().parents[1] / "scripts" / "understand_graph_dashboard.py"
    spec = importlib.util.spec_from_file_location("understand_graph_dashboard", source)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_document(root: Path, relative: str, text: str) -> Path:
    path = root / "docs" / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def test_document_projection_derives_nodes_and_links_from_markdown(tmp_path: Path) -> None:
    dashboard = load_dashboard_module()
    write_document(
        tmp_path,
        "architecture/overview.md",
        "# Architecture Overview\n\nSee [decision](../sentry/regional/decision.md).\n",
    )
    write_document(tmp_path, "sentry/regional/decision.md", "# Decision\n\n[[docs/architecture/overview|Overview]]\n")

    projection = dashboard.build_document_projection(tmp_path)

    nodes = {node["id"]: node for node in projection["nodes"]}
    assert set(nodes) == {"architecture/overview", "sentry/regional/decision"}
    assert nodes["architecture/overview"]["title"] == "Architecture Overview"
    assert nodes["sentry/regional/decision"]["category"] == "decision-tree"
    assert {(edge["source"], edge["target"]) for edge in projection["edges"]} == {
        ("architecture/overview", "sentry/regional/decision"),
        ("sentry/regional/decision", "architecture/overview"),
    }


def test_document_projection_resolves_docs_root_links_from_any_document(tmp_path: Path) -> None:
    dashboard = load_dashboard_module()
    write_document(tmp_path, "architecture/overview.md", "# Architecture\n")
    write_document(
        tmp_path,
        "sentry/current.md",
        "# Current\n\n[[architecture/overview]]\n\n[Overview](/docs/architecture/overview.md)\n",
    )

    projection = dashboard.build_document_projection(tmp_path)

    assert {(edge["source"], edge["target"]) for edge in projection["edges"]} == {
        ("sentry/current", "architecture/overview")
    }


def test_document_projection_changes_when_a_page_is_edited_added_or_removed(tmp_path: Path) -> None:
    dashboard = load_dashboard_module()
    alpha = write_document(tmp_path, "architecture/alpha.md", "# Alpha\n")

    initial = dashboard.build_document_projection(tmp_path)
    alpha.write_text("# Alpha Revised\n", encoding="utf-8")
    edited = dashboard.build_document_projection(tmp_path)
    write_document(tmp_path, "guides/guide.md", "# Guide\n")
    added = dashboard.build_document_projection(tmp_path)
    alpha.unlink()
    removed = dashboard.build_document_projection(tmp_path)

    assert initial["revision"] != edited["revision"]
    assert edited["revision"] != added["revision"]
    assert added["revision"] != removed["revision"]
    assert {node["id"] for node in removed["nodes"]} == {"guides/guide"}


def test_http_api_reads_only_the_live_document_projection(tmp_path: Path) -> None:
    dashboard = load_dashboard_module()
    page = write_document(tmp_path, "architecture/overview.md", "# First title\n")
    server = dashboard.create_server("127.0.0.1", 0, tmp_path)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urlopen(base_url + "/api/documents") as response:
            projection = json.load(response)
        assert {node["id"] for node in projection["nodes"]} == {"architecture/overview"}

        page.write_text("# Updated title\n", encoding="utf-8")
        with urlopen(base_url + "/api/document?id=architecture%2Foverview") as response:
            document = json.load(response)
        assert document == {
            "id": "architecture/overview",
            "title": "Updated title",
            "content": "# Updated title\n",
        }

        with pytest.raises(HTTPError) as error:
            urlopen(base_url + "/graph.json")
        assert error.value.code == 404
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_documentation_graph_and_split_share_live_document_state(tmp_path: Path) -> None:
    sync_api = pytest.importorskip("playwright.sync_api")
    dashboard = load_dashboard_module()
    write_document(
        tmp_path,
        "architecture/overview.md",
        "# Architecture\n\nSee [decision](../sentry/regional/decision.md).\n",
    )
    write_document(tmp_path, "sentry/regional/decision.md", "# Decision\n\n[[docs/architecture/overview]]\n")
    server = dashboard.create_server("127.0.0.1", 0, tmp_path)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with sync_api.sync_playwright() as playwright:
            browser = playwright.chromium.launch()
            page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=1)
            errors: list[str] = []
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.on("console", lambda message: errors.append(message.text) if message.type == "error" else None)
            page.goto(base_url, wait_until="networkidle")
            page.locator(".reader h1").wait_for()
            assert page.locator(".reader h1").text_content() == "Architecture"

            page.locator('[data-view="graph"]').click()
            canvas = page.locator("#graph")
            canvas.wait_for(state="visible")
            assert canvas.bounding_box()["width"] > 900
            decision_position = page.evaluate(
                """() => {
                  const node = state.byId.get('sentry/regional/decision');
                  return {
                    x: node.x * state.camera.zoom + state.camera.x,
                    y: node.y * state.camera.zoom + state.camera.y,
                  };
                }"""
            )
            box = canvas.bounding_box()
            assert page.evaluate(
                """position => hitNode(position.x, position.y)?.id || null""",
                decision_position,
            ) == "sentry/regional/decision"
            page.mouse.click(box["x"] + decision_position["x"], box["y"] + decision_position["y"])
            assert page.evaluate("state.selectedId") == "sentry/regional/decision"
            assert "graph" in page.locator("#workspace").get_attribute("class")

            page.mouse.dblclick(box["x"] + decision_position["x"], box["y"] + decision_position["y"])
            page.locator(".reader h1").wait_for()
            assert page.locator(".reader h1").text_content() == "Decision"
            assert "docs" in page.locator("#workspace").get_attribute("class")

            page.locator('[data-view="split"]').click()
            assert page.locator("#readerPane").is_visible()
            assert canvas.is_visible()

            write_document(tmp_path, "guides/live.md", "# Live update\n")
            page.locator("#nodeCount").get_by_text("3 pages").wait_for()
            assert not errors
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
