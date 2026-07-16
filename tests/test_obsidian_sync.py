"""Regression tests for the source-led Obsidian knowledge-vault synchronizer."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPOSITORY_ROOT / "scripts" / "obsidian_sync.py"


def load_module() -> ModuleType:
    """Import the script as a module without requiring scripts to be a package."""
    spec = importlib.util.spec_from_file_location("obsidian_sync", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load synchronizer from {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


module = load_module()


def make_package(root: Path, name: str, *, message: str | None = None, source: str = "") -> Path:
    package_root = root / "src" / name
    (package_root / "src").mkdir(parents=True)
    (package_root / "package.xml").write_text(
        "<package format=\"3\">"
        f"<name>{name}</name>"
        "<version>0.0.0</version>"
        "<description>test package</description>"
        "<maintainer email=\"test@example.com\">Test</maintainer>"
        "<license>MIT</license>"
        "<depend>rclcpp</depend>"
        "</package>",
        encoding="utf-8",
    )
    (package_root / "src" / "node.cpp").write_text(source, encoding="utf-8")
    if message is not None:
        message_path = package_root / "msg" / message
        message_path.parent.mkdir()
        message_path.write_text("uint8 level # a state level\n", encoding="utf-8")
    return package_root


def make_workspace(root: Path) -> Path:
    return make_package(
        root,
        "demo",
        message="State.msg",
        source='publisher->publish("/ly/demo/state");',
    )


def output(root: Path) -> Path:
    return root / "docs" / "obsidian"


def test_collect_workspace_reads_package_message_and_topic(tmp_path: Path) -> None:
    make_package(tmp_path, "demo", message="State.msg", source='pub("/ly/demo/state")')

    index = module.collect_workspace(tmp_path)

    assert index.packages["demo"].messages["State"] == ["uint8 level"]
    assert index.topics["/ly/demo/state"].packages == {"demo"}


def test_render_workspace_is_deterministic_and_links_from_vault_root(tmp_path: Path) -> None:
    make_workspace(tmp_path)
    index = module.collect_workspace(tmp_path)

    first = module.render_workspace(index)
    second = module.render_workspace(index)

    assert first == second
    assert first[module.PurePosixPath("_generated/Index.md")].startswith(module.GENERATED_MARKER)
    assert "[[docs/obsidian/_generated/Packages/demo|demo]]" in first[
        module.PurePosixPath("_generated/Index.md")
    ]
    assert "[[src/demo/src/node.cpp|src/demo/src/node.cpp]]" in first[
        module.PurePosixPath("_generated/Topics/ly__demo__state.md")
    ]


def test_check_reports_drift_without_writing(tmp_path: Path) -> None:
    package_root = make_workspace(tmp_path)
    vault_output = output(tmp_path)
    module.sync(tmp_path, vault_output, check=False, dry_run=False)
    topic_note = vault_output / "_generated" / "Topics" / "ly__demo__state.md"
    before_check = topic_note.read_text(encoding="utf-8")
    (package_root / "src" / "node.cpp").write_text('"/ly/demo/changed"', encoding="utf-8")

    result = module.sync(tmp_path, vault_output, check=True, dry_run=False)

    assert result.is_current is False
    assert topic_note.read_text(encoding="utf-8") == before_check


def test_sync_never_touches_human_notes_outside_generated_tree(tmp_path: Path) -> None:
    make_workspace(tmp_path)
    vault_output = output(tmp_path)
    manual_note = vault_output / "notes" / "manual.md"
    manual_note.parent.mkdir(parents=True)
    manual_note.write_text("My engineering judgement.\n", encoding="utf-8")

    result = module.sync(tmp_path, vault_output, check=False, dry_run=False)

    assert result.is_current is True
    assert manual_note.read_text(encoding="utf-8") == "My engineering judgement.\n"
    assert (vault_output / "_generated" / "Index.md").is_file()


def test_sync_preserves_manifest_listed_file_without_generated_marker(tmp_path: Path) -> None:
    package_root = make_workspace(tmp_path)
    vault_output = output(tmp_path)
    module.sync(tmp_path, vault_output, check=False, dry_run=False)
    message_note = vault_output / "_generated" / "Messages" / "demo" / "State.md"
    message_note.write_text("Human replacement.\n", encoding="utf-8")
    (package_root / "msg" / "State.msg").unlink()

    result = module.sync(tmp_path, vault_output, check=False, dry_run=False)

    assert result.is_current is False
    assert message_note.read_text(encoding="utf-8") == "Human replacement.\n"
    assert module.PurePosixPath("_generated/Messages/demo/State.md") in result.conflicts
    manifest = json.loads((vault_output / "_generated" / ".manifest.json").read_text(encoding="utf-8"))
    assert "_generated/Messages/demo/State.md" not in manifest["files"]


def test_dry_run_reports_changes_without_writing(tmp_path: Path) -> None:
    make_workspace(tmp_path)
    vault_output = output(tmp_path)

    result = module.sync(tmp_path, vault_output, check=False, dry_run=True)

    assert result.is_current is False
    assert result.planned_writes > 0
    assert not (vault_output / "_generated").exists()


def test_sync_rejects_manifest_path_outside_generated_tree(tmp_path: Path) -> None:
    make_workspace(tmp_path)
    vault_output = output(tmp_path)
    escaped_note = vault_output / "notes" / "generated-looking.md"
    escaped_note.parent.mkdir(parents=True)
    escaped_note.write_text(module.GENERATED_MARKER + "Manual note.\n", encoding="utf-8")
    generated_root = vault_output / "_generated"
    generated_root.mkdir()
    (generated_root / ".manifest.json").write_text(
        json.dumps({"generator": "scripts/obsidian_sync.py", "files": ["../notes/generated-looking.md"]}),
        encoding="utf-8",
    )

    result = module.sync(tmp_path, vault_output, check=False, dry_run=False)

    assert result.is_current is False
    assert escaped_note.read_text(encoding="utf-8") == module.GENERATED_MARKER + "Manual note.\n"
    assert module.PurePosixPath("../notes/generated-looking.md") in result.conflicts


def test_sync_rejects_generated_note_symlink_to_outside_vault(tmp_path: Path) -> None:
    make_workspace(tmp_path)
    vault_output = output(tmp_path)
    generated_index = vault_output / "_generated" / "Index.md"
    generated_index.parent.mkdir(parents=True)
    outside = tmp_path / "outside.md"
    generated_index.symlink_to(outside)

    result = module.sync(tmp_path, vault_output, check=False, dry_run=False)

    assert result.is_current is False
    assert not outside.exists()
    assert module.PurePosixPath("_generated/Index.md") in result.conflicts


def test_sync_rejects_generated_note_symlink_inside_generated_tree(tmp_path: Path) -> None:
    make_workspace(tmp_path)
    vault_output = output(tmp_path)
    generated_root = vault_output / "_generated"
    package_note = generated_root / "Packages" / "demo.md"
    package_note.parent.mkdir(parents=True)
    package_note.write_text(module.GENERATED_MARKER + "Original package note.\n", encoding="utf-8")
    (generated_root / "Index.md").symlink_to("Packages/demo.md")

    result = module.sync(tmp_path, vault_output, check=False, dry_run=False)

    assert result.is_current is False
    assert (generated_root / "Index.md").is_symlink()
    assert module.PurePosixPath("_generated/Index.md") in result.conflicts


def test_cli_rejects_output_outside_docs_obsidian(tmp_path: Path) -> None:
    make_workspace(tmp_path)
    rejected_output = tmp_path / "src" / "demo"

    exit_code = module.main(["--repo-root", str(tmp_path), "--output", str(rejected_output)])

    assert exit_code == 2
    assert not (rejected_output / "_generated").exists()


def test_collect_workspace_excludes_dynamic_topic_prefixes(tmp_path: Path) -> None:
    make_package(
        tmp_path,
        "demo",
        source='''
            auto static_topic = "/ly/demo/static";
            auto plus_topic = "/ly/demo/plus" + std::to_string(id);
            stream << "/ly/demo/stream" << id;
        ''',
    )

    index = module.collect_workspace(tmp_path)

    assert set(index.topics) == {"/ly/demo/static"}
