from __future__ import annotations

import argparse
import importlib
import json
import sys
import tempfile
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from .config import resolve_path
from .web_stream import SimulatorWebStream, json_safe


@dataclass(frozen=True)
class Viewport:
    name: str
    width: int
    height: int


DEFAULT_VIEWPORTS = (
    Viewport("desktop", 1440, 900),
    Viewport("narrow", 390, 844),
)


class DemoSurface:
    def __init__(self, width: int, height: int, rgb: bytes) -> None:
        self.width = int(width)
        self.height = int(height)
        self.rgb = bytes(rgb)

    def get_size(self) -> tuple[int, int]:
        return (self.width, self.height)


class DemoPygameImage:
    @staticmethod
    def tostring(surface: DemoSurface, fmt: str) -> bytes:
        if fmt != "RGB":
            raise ValueError(f"unsupported demo surface format: {fmt}")
        return surface.rgb


class DemoPygame:
    image = DemoPygameImage()


PlaywrightSyncFactory = Callable[[], Any]


def demo_frame_rgb(width: int = 1500, height: int = 900) -> bytes:
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as exc:
        raise RuntimeError("Pillow is required for web visual check") from exc

    image = Image.new("RGB", (width, height), (7, 13, 24))
    draw = ImageDraw.Draw(image)
    for y in range(height):
        tone = int(18 + (y / max(1, height - 1)) * 42)
        draw.line([(0, y), (width, y)], fill=(7, tone, 38))
    for x in range(0, width, 75):
        color = (24, 41, 63) if (x // 75) % 2 == 0 else (18, 32, 48)
        draw.line([(x, 0), (x, height)], fill=color)
    for y in range(0, height, 75):
        draw.line([(0, y), (width, y)], fill=(24, 41, 63))

    field = (80, 80, width - 80, height - 80)
    draw.rectangle(field, outline=(71, 85, 105), width=3)
    draw.rectangle((110, 110, 360, 235), fill=(15, 23, 42), outline=(56, 189, 248), width=2)
    draw.rectangle((width - 360, height - 235, width - 110, height - 110), fill=(15, 23, 42), outline=(248, 113, 113), width=2)
    draw.line((field[0], height // 2, field[2], height // 2), fill=(148, 163, 184), width=2)
    draw.ellipse((width // 2 - 52, height // 2 - 52, width // 2 + 52, height // 2 + 52), outline=(34, 197, 94), width=4)

    units = [
        ("red Hero", (430, 300), (239, 68, 68)),
        ("red Sentry", (740, 475), (248, 113, 113)),
        ("blue Hero", (1020, 610), (59, 130, 246)),
        ("blue Infantry3", (910, 280), (96, 165, 250)),
        ("target", (640, 382), (245, 158, 11)),
    ]
    font = ImageFont.load_default()
    for label, (x, y), color in units:
        draw.ellipse((x - 22, y - 22, x + 22, y + 22), fill=color, outline=(248, 250, 252), width=2)
        draw.rectangle((x - 36, y + 30, x + 36, y + 38), fill=(15, 23, 42), outline=(51, 65, 85))
        draw.rectangle((x - 35, y + 31, x + 16, y + 37), fill=(34, 197, 94))
        draw.text((x - 36, y + 43), label, fill=(226, 232, 240), font=font)

    draw.line((740, 475, 640, 382), fill=(250, 204, 21), width=4)
    draw.line((740, 475, 940, 430, 1030, 500), fill=(34, 197, 94), width=3)
    draw.text((96, 48), "LY Sentry offline simulator web visual check", fill=(248, 250, 252), font=font)
    draw.text((96, 66), "synthetic pygame frame - non-runtime browser QA", fill=(148, 163, 184), font=font)
    return image.tobytes()


def demo_metadata() -> dict[str, Any]:
    return {
        "trace": {
            "name": "web_visual_check_sample.jsonl",
            "records": 6,
            "duration_sec": 1.5,
            "tick_range": {"first": 10, "last": 60},
        },
        "validation": {
            "schema": "ly_simulator_validation_report_v1",
            "status": "PASS",
            "issues": [],
            "issue_groups": {},
        },
        "replay": {
            "current_record": 4,
            "total_records": 6,
            "speed": 1.0,
            "panel_tab": "Layers",
            "match_time_left": 389,
            "match_running": True,
            "local_controls_available": True,
        },
        "current_record": {
            "tick": 60,
            "team": "red",
            "strategy": "RegionalControl",
            "aim": "external",
            "goal": {"name": "CentralLeft.Attack", "id": 22, "pos_cm": [1505.0, 905.0]},
            "output": {"kind": "goal_pos", "topic": "/ly/navi/goal_pos", "frame_id": "map"},
            "gimbal_feedback": {"available": True, "age_ms": 8, "fire_code": {"rotate": 1}},
            "control_output": {
                "available": True,
                "sequence": 60,
                "source": "normal",
                "angles": {"published": True, "yaw": 12.0, "pitch": -3.0},
                "fire_code": {"published": True, "follow_mode": True, "rotate": 0},
                "trajectory": {"available": True, "yaw": 12.0, "pitch": -3.0},
            },
            "tactical": {
                "available": True,
                "protect_castle": {"enabled": True, "rfid_event_active": False, "stay_when_rfid_enabled": True, "stay_when_rfid_active": True, "enemy_pos_active": True},
                "protect_hero": {"enabled": True, "active": False},
                "regional_defense": {"threat_active": True, "search_kind": "own_base", "own_base_enemy_count": 1},
            },
        },
        "simulator_inputs": {
            "enabled": True,
            "state": {
                "ownership_mode": "mock",
                "field": {"width_cm": 2800, "height_cm": 1500, "frame": "left_bottom_origin_cm"},
                "selected_entity_id": "enemy:hero:demo",
                "runtime": {
                    "self_health": 280,
                    "ammo_left": 80,
                    "posture": 2,
                    "self_position_cm": [1470.0, 870.0],
                },
                "summary": {
                    "friend_units": 2,
                    "enemy_units": 3,
                    "low_hp_units": ["enemy:Infantry3"],
                    "destroyed_structures": ["enemy_outpost"],
                },
                "structures": [
                    {
                        "key": "enemy_outpost",
                        "label": "Enemy Outpost",
                        "side": "enemy",
                        "kind": "outpost",
                        "hp": 0,
                        "max_hp": 1500,
                        "hp_ratio": 0.0,
                    },
                    {
                        "key": "enemy_base",
                        "label": "Enemy Base",
                        "side": "enemy",
                        "kind": "base",
                        "hp": 4200,
                        "max_hp": 5000,
                        "hp_ratio": 0.84,
                    },
                ],
                "units": [
                    {
                        "entity_id": "friend:hero:demo",
                        "side": "friend",
                        "field_side": "red",
                        "unit_key": "hero",
                        "asset_key": "hero",
                        "type": "Hero",
                        "hp": 420,
                        "max_hp": 500,
                        "position_cm": {"x": 1250.0, "y": 760.0},
                    },
                    {
                        "entity_id": "friend:sentry:demo",
                        "side": "friend",
                        "field_side": "red",
                        "unit_key": "sentry",
                        "asset_key": "sentry",
                        "type": "Sentry",
                        "hp": 280,
                        "max_hp": 400,
                        "position_cm": {"x": 1470.0, "y": 870.0},
                    },
                    {
                        "entity_id": "enemy:hero:demo",
                        "side": "enemy",
                        "field_side": "blue",
                        "unit_key": "hero",
                        "asset_key": "hero",
                        "type": "Hero",
                        "hp": 260,
                        "max_hp": 500,
                        "position_cm": {"x": 1940.0, "y": 1100.0},
                    },
                    {
                        "entity_id": "enemy:infantry3:demo",
                        "side": "enemy",
                        "field_side": "blue",
                        "unit_key": "infantry3",
                        "asset_key": "infantry3",
                        "type": "Infantry3",
                        "hp": 80,
                        "max_hp": 400,
                        "position_cm": {"x": 1710.0, "y": 940.0},
                    },
                ],
                "palette": [
                    {"side": "enemy", "unit_key": "hero", "asset_key": "hero", "type": "Hero", "hp": 200, "max_hp": 200}
                ],
            },
        },
    }


def find_sync_playwright() -> tuple[PlaywrightSyncFactory | None, str]:
    try:
        module = importlib.import_module("playwright.sync_api")
    except ImportError:
        return (None, "python package `playwright` is not installed")
    factory = getattr(module, "sync_playwright", None)
    if factory is None:
        return (None, "`playwright.sync_api.sync_playwright` is unavailable")
    return (factory, "")


def wait_for_healthz(url: str, timeout_sec: float) -> tuple[bool, str]:
    deadline = time.monotonic() + max(0.1, timeout_sec)
    last_error = ""
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(f"{url}/healthz", timeout=0.5) as response:
                if response.status == 200:
                    return (True, "")
        except (urllib.error.URLError, TimeoutError) as exc:
            last_error = str(exc)
        time.sleep(0.05)
    return (False, last_error or "timed out waiting for /healthz")


def inspect_dashboard(page: Any, viewport: Viewport) -> list[str]:
    metrics = page.evaluate(
        """() => {
          const f = document.getElementById('f');
          const dashboard = document.getElementById('dashboard');
          const shell = document.getElementById('shell');
          const cards = Array.from(document.querySelectorAll('.card'));
          const frameBox = f ? f.getBoundingClientRect() : null;
          const dashBox = dashboard ? dashboard.getBoundingClientRect() : null;
          const shellStyle = shell ? getComputedStyle(shell).gridTemplateColumns : '';
          return {
            title: document.title,
            readyText: document.getElementById('readyPill')?.textContent || '',
            validationText: document.getElementById('validationPill')?.textContent || '',
            naturalWidth: f ? f.naturalWidth : 0,
            naturalHeight: f ? f.naturalHeight : 0,
            frameWidth: frameBox ? frameBox.width : 0,
            frameHeight: frameBox ? frameBox.height : 0,
            dashboardWidth: dashBox ? dashBox.width : 0,
            dashboardHeight: dashBox ? dashBox.height : 0,
            cardCount: cards.length,
            emptyCards: cards.filter((card) => card.getBoundingClientRect().height <= 0).length,
            scrollWidth: document.documentElement.scrollWidth,
            clientWidth: document.documentElement.clientWidth,
            shellColumns: shellStyle,
            bodyText: document.body ? document.body.innerText : ''
          };
        }"""
    )
    issues: list[str] = []
    if metrics.get("title") != "Simulator Live":
        issues.append(f"{viewport.name}: unexpected title {metrics.get('title')!r}")
    if "ready" not in str(metrics.get("readyText", "")).lower():
        issues.append(f"{viewport.name}: stream did not reach ready state")
    if "PASS" not in str(metrics.get("validationText", "")):
        issues.append(f"{viewport.name}: validation PASS pill missing")
    if int(metrics.get("naturalWidth") or 0) <= 0 or int(metrics.get("naturalHeight") or 0) <= 0:
        issues.append(f"{viewport.name}: frame image did not load")
    if float(metrics.get("frameWidth") or 0) < 250:
        issues.append(f"{viewport.name}: frame pane is too narrow")
    if float(metrics.get("dashboardWidth") or 0) < 250:
        issues.append(f"{viewport.name}: dashboard pane is too narrow")
    if int(metrics.get("cardCount") or 0) < 6:
        issues.append(f"{viewport.name}: dashboard cards missing")
    if int(metrics.get("emptyCards") or 0) > 0:
        issues.append(f"{viewport.name}: dashboard card collapsed")
    if int(metrics.get("scrollWidth") or 0) > int(metrics.get("clientWidth") or 0) + 2:
        issues.append(f"{viewport.name}: page has horizontal overflow")
    body_text = str(metrics.get("bodyText") or "")
    for expected in ("Trace", "Replay", "Current Decision", "Simulator Inputs", "Placed Units", "Alerts"):
        if expected not in body_text:
            issues.append(f"{viewport.name}: missing dashboard text {expected!r}")
    return issues


def inspect_tactical(page: Any, viewport: Viewport) -> list[str]:
    metrics = page.evaluate(
        """() => {
          const board = document.getElementById('fieldBoard');
          const inspector = document.getElementById('inspector');
          const boardBox = board ? board.getBoundingClientRect() : null;
          const inspectorBox = inspector ? inspector.getBoundingClientRect() : null;
          return {
            title: document.title,
            owner: document.getElementById('ownerPill')?.textContent || '',
            boardWidth: boardBox ? boardBox.width : 0,
            boardHeight: boardBox ? boardBox.height : 0,
            pieceCount: document.querySelectorAll('.piece').length,
            inspectorWidth: inspectorBox ? inspectorBox.width : 0,
            scrollWidth: document.documentElement.scrollWidth,
            clientWidth: document.documentElement.clientWidth,
            bodyText: document.body ? document.body.innerText : '',
          };
        }"""
    )
    issues: list[str] = []
    if metrics.get("title") != "LY Sentinel Tactical Simulator":
        issues.append(f"{viewport.name}: unexpected tactical title {metrics.get('title')!r}")
    if "mock" not in str(metrics.get("owner", "")).lower():
        issues.append(f"{viewport.name}: tactical ownership pill did not reach mock mode")
    if float(metrics.get("boardWidth") or 0) < 250 or float(metrics.get("boardHeight") or 0) < 130:
        issues.append(f"{viewport.name}: tactical field is not visibly framed")
    if int(metrics.get("pieceCount") or 0) < 2:
        issues.append(f"{viewport.name}: tactical pieces are missing")
    if float(metrics.get("inspectorWidth") or 0) < 250:
        issues.append(f"{viewport.name}: tactical inspector is too narrow")
    if int(metrics.get("scrollWidth") or 0) > int(metrics.get("clientWidth") or 0) + 2:
        issues.append(f"{viewport.name}: tactical page has horizontal overflow")
    body_text = str(metrics.get("bodyText") or "")
    for expected in ("Overview", "Robot roster", "Tactical state", "Operations shelf"):
        if expected not in body_text:
            issues.append(f"{viewport.name}: missing tactical text {expected!r}")
    return issues


def run_browser_check(
    url: str,
    output_dir: Path,
    viewports: tuple[Viewport, ...],
    sync_playwright_factory: PlaywrightSyncFactory,
    timeout_ms: int,
) -> tuple[str, list[str], list[str]]:
    screenshots: list[str] = []
    issues: list[str] = []
    try:
        with sync_playwright_factory() as playwright:
            try:
                browser = playwright.chromium.launch(headless=True)
            except Exception as exc:
                return ("skip", [str(exc)], screenshots)
            try:
                for viewport in viewports:
                    context = browser.new_context(
                        viewport={"width": viewport.width, "height": viewport.height},
                        device_scale_factor=1,
                    )
                    try:
                        page = context.new_page()
                        page.goto(url, wait_until="domcontentloaded", timeout=timeout_ms)
                        page.wait_for_selector("#dashboard", timeout=timeout_ms)
                        page.wait_for_function(
                            "document.getElementById('readyPill')?.textContent.includes('ready')",
                            timeout=timeout_ms,
                        )
                        page.wait_for_function("document.getElementById('f')?.naturalWidth > 0", timeout=timeout_ms)
                        issues.extend(inspect_dashboard(page, viewport))
                        screenshot_path = output_dir / f"web-dashboard-{viewport.name}.png"
                        page.screenshot(path=screenshot_path.as_posix(), full_page=True)
                        screenshots.append(screenshot_path.as_posix())

                        tactical_page = context.new_page()
                        tactical_page.goto(f"{url}/tactical", wait_until="domcontentloaded", timeout=timeout_ms)
                        tactical_page.wait_for_selector("#fieldBoard", timeout=timeout_ms)
                        tactical_page.wait_for_function(
                            "document.getElementById('ownerPill')?.textContent.includes('mock')",
                            timeout=timeout_ms,
                        )
                        issues.extend(inspect_tactical(tactical_page, viewport))
                        tactical_screenshot_path = output_dir / f"web-tactical-{viewport.name}.png"
                        tactical_page.screenshot(path=tactical_screenshot_path.as_posix(), full_page=True)
                        screenshots.append(tactical_screenshot_path.as_posix())
                    except Exception as exc:
                        issues.append(f"{viewport.name}: browser check failed: {exc}")
                    finally:
                        try:
                            context.close()
                        except Exception as exc:
                            issues.append(f"{viewport.name}: browser context close failed: {exc}")
            finally:
                browser.close()
    except Exception as exc:
        return ("skip", [str(exc)], screenshots)
    return ("fail" if issues else "pass", issues, screenshots)


def run_visual_check(
    output_dir: Path,
    *,
    host: str = "127.0.0.1",
    port: int = 0,
    require_browser: bool = False,
    timeout_sec: float = 8.0,
    sync_playwright_factory: PlaywrightSyncFactory | None = None,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    control_dir = tempfile.TemporaryDirectory(prefix="ly-sim-web-visual-")
    stream: SimulatorWebStream | None = None
    try:
        control_file = Path(control_dir.name) / "control.jsonl"
        stream = SimulatorWebStream(
            host=host,
            port=port,
            fps=1000.0,
            jpeg_quality=82,
            control_file=control_file.as_posix(),
            default_step_sec=10,
            map_path=resolve_path("tools/maps/basemaps/buff_map_field.png").as_posix(),
        )
        stream.start()
        frame = DemoSurface(1500, 900, demo_frame_rgb())
        stream.update_metadata(demo_metadata())
        stream.publish_surface(frame, DemoPygame())
        url = f"http://127.0.0.1:{stream.port}" if host in ("0.0.0.0", "127.0.0.1", "localhost", "") else f"http://{host}:{stream.port}"

        ready, health_error = wait_for_healthz(url, timeout_sec=timeout_sec)
        if not ready:
            return {
                "status": "fail",
                "url": url,
                "reason": health_error,
                "screenshots": [],
                "output_dir": output_dir.as_posix(),
            }

        factory = sync_playwright_factory
        if factory is None:
            factory, reason = find_sync_playwright()
            if factory is None:
                return {
                    "status": "fail" if require_browser else "skip",
                    "url": url,
                    "reason": reason,
                    "screenshots": [],
                    "output_dir": output_dir.as_posix(),
                    "install_hint": "python3 -m pip install -r src/simulator/requirements-browser.txt && playwright install chromium",
                }

        status, findings, screenshots = run_browser_check(
            url,
            output_dir,
            DEFAULT_VIEWPORTS,
            factory,
            timeout_ms=int(max(1.0, timeout_sec) * 1000),
        )
        if status == "skip" and require_browser:
            status = "fail"
        return {
            "status": status,
            "url": url,
            "reason": findings[0] if findings else "",
            "findings": findings,
            "screenshots": screenshots,
            "output_dir": output_dir.as_posix(),
        }
    finally:
        if stream is not None:
            stream.stop()
        control_dir.cleanup()


def print_result(result: dict[str, Any], *, json_output: bool) -> None:
    clean = json_safe(result)
    if json_output:
        print(json.dumps(clean, ensure_ascii=False, allow_nan=False, indent=2, sort_keys=True))
        return
    status = str(result.get("status", "unknown")).upper()
    print(f"web visual check: {status}")
    if result.get("url"):
        print(f"url: {result['url']}")
    if result.get("output_dir"):
        print(f"output_dir: {result['output_dir']}")
    if result.get("reason"):
        print(f"reason: {result['reason']}")
    for path in result.get("screenshots", []) or []:
        print(f"screenshot: {path}")
    if result.get("install_hint"):
        print(f"install: {result['install_hint']}")
    findings = result.get("findings") or []
    for item in findings:
        print(f"finding: {item}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run optional browser visual QA for the simulator HTTP dashboard.")
    parser.add_argument(
        "--output-dir",
        default="/tmp/ly-simulator-web-visual",
        help="Directory for browser screenshots. Defaults to /tmp/ly-simulator-web-visual.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Temporary web stream bind host.")
    parser.add_argument("--port", type=int, default=0, help="Temporary web stream port. 0 lets the OS choose.")
    parser.add_argument("--timeout-sec", type=float, default=8.0, help="Browser and readiness timeout in seconds.")
    parser.add_argument(
        "--require-browser",
        action="store_true",
        help="Treat missing Playwright/browser tooling as a failure instead of a skip.",
    )
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON output.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.port < 0 or args.port > 65535:
        print("--port must be in [0, 65535]", file=sys.stderr)
        return 2
    if args.timeout_sec <= 0:
        print("--timeout-sec must be > 0", file=sys.stderr)
        return 2
    result = run_visual_check(
        Path(args.output_dir).expanduser().resolve(),
        host=str(args.host),
        port=int(args.port),
        require_browser=bool(args.require_browser),
        timeout_sec=float(args.timeout_sec),
    )
    print_result(result, json_output=bool(args.json))
    return 0 if result.get("status") in ("pass", "skip") else 1


if __name__ == "__main__":
    raise SystemExit(main())
