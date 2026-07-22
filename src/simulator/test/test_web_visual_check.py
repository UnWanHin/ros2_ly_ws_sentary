from __future__ import annotations

from pathlib import Path

from simulator import web_visual_check


class FakePage:
    def __init__(self) -> None:
        self.urls: list[str] = []
        self.current_url = ""

    def goto(self, url: str, wait_until: str, timeout: int) -> None:
        self.urls.append(url)
        self.current_url = url
        assert wait_until == "domcontentloaded"
        assert timeout > 0

    def wait_for_selector(self, selector: str, timeout: int) -> None:
        assert selector in {"#dashboard", "#fieldBoard"}
        assert timeout > 0

    def wait_for_function(self, expression: str, timeout: int) -> None:
        assert "readyPill" in expression or "naturalWidth" in expression or "ownerPill" in expression
        assert timeout > 0

    def evaluate(self, script: str) -> dict[str, object]:
        if "fieldBoard" in script:
            return {
                "title": "Sentinel Flight Deck",
                "owner": "mock",
                "boardWidth": 920,
                "boardHeight": 492,
                "pieceCount": 2,
                "inspectorWidth": 360,
                "scrollWidth": 390,
                "clientWidth": 390,
                "bodyText": "Overview Robot roster Tactical state Operations shelf",
            }
        assert "#dashboard" in script or "dashboard" in script
        return {
            "title": "Simulator Live",
            "readyText": "ready",
            "validationText": "validation PASS",
            "naturalWidth": 1500,
            "naturalHeight": 900,
            "frameWidth": 920,
            "frameHeight": 552,
            "dashboardWidth": 390,
            "dashboardHeight": 800,
            "cardCount": 6,
            "emptyCards": 0,
            "scrollWidth": 390,
            "clientWidth": 390,
            "shellColumns": "390px",
            "bodyText": "Trace Replay Current Decision Simulator Inputs Placed Units Alerts",
        }

    def screenshot(self, path: str, full_page: bool) -> None:
        assert full_page is True
        Path(path).write_bytes(b"\x89PNG\r\n\x1a\n")


class FakeContext:
    def __init__(self) -> None:
        self.page = FakePage()
        self.closed = False

    def new_page(self) -> FakePage:
        return self.page

    def close(self) -> None:
        self.closed = True


class FakeBrowser:
    def __init__(self) -> None:
        self.contexts: list[FakeContext] = []
        self.closed = False

    def new_context(self, viewport: dict[str, int], device_scale_factor: int) -> FakeContext:
        assert viewport["width"] > 0
        assert viewport["height"] > 0
        assert device_scale_factor == 1
        context = FakeContext()
        self.contexts.append(context)
        return context

    def close(self) -> None:
        self.closed = True


class FakeChromium:
    def __init__(self) -> None:
        self.browser = FakeBrowser()

    def launch(self, headless: bool) -> FakeBrowser:
        assert headless is True
        return self.browser


class FakePlaywright:
    def __init__(self) -> None:
        self.chromium = FakeChromium()


class FakePlaywrightContext:
    def __init__(self) -> None:
        self.playwright = FakePlaywright()

    def __enter__(self) -> FakePlaywright:
        return self.playwright

    def __exit__(self, exc_type, exc, tb) -> None:
        return None


def fake_sync_playwright() -> FakePlaywrightContext:
    return FakePlaywrightContext()


def test_visual_check_skips_cleanly_when_playwright_is_missing(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(web_visual_check, "find_sync_playwright", lambda: (None, "missing playwright"))

    result = web_visual_check.run_visual_check(tmp_path, timeout_sec=2.0)

    assert result["status"] == "skip"
    assert result["reason"] == "missing playwright"
    assert "requirements-browser.txt" in result["install_hint"]
    assert "playwright install chromium" in result["install_hint"]


def test_visual_check_can_require_browser_tooling(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(web_visual_check, "find_sync_playwright", lambda: (None, "missing browser"))

    result = web_visual_check.run_visual_check(tmp_path, require_browser=True, timeout_sec=2.0)

    assert result["status"] == "fail"
    assert result["reason"] == "missing browser"


def test_visual_check_captures_desktop_and_narrow_screenshots_with_browser(tmp_path: Path) -> None:
    result = web_visual_check.run_visual_check(
        tmp_path,
        timeout_sec=2.0,
        sync_playwright_factory=fake_sync_playwright,
    )

    assert result["status"] == "pass"
    assert result["findings"] == []
    assert [Path(path).name for path in result["screenshots"]] == [
        "web-dashboard-desktop.png",
        "web-tactical-desktop.png",
        "web-dashboard-narrow.png",
        "web-tactical-narrow.png",
    ]
    for path in result["screenshots"]:
        assert Path(path).read_bytes().startswith(b"\x89PNG")


def test_inspect_dashboard_reports_core_layout_failures() -> None:
    class BrokenPage(FakePage):
        def evaluate(self, script: str) -> dict[str, object]:
            metrics = super().evaluate(script)
            metrics.update(
                {
                    "readyText": "waiting",
                    "validationText": "validation -",
                    "naturalWidth": 0,
                    "frameWidth": 120,
                    "dashboardWidth": 180,
                    "cardCount": 2,
                    "emptyCards": 1,
                    "scrollWidth": 500,
                    "clientWidth": 390,
                    "bodyText": "Trace",
                }
            )
            return metrics

    issues = web_visual_check.inspect_dashboard(BrokenPage(), web_visual_check.Viewport("narrow", 390, 844))

    assert "narrow: stream did not reach ready state" in issues
    assert "narrow: validation PASS pill missing" in issues
    assert "narrow: frame image did not load" in issues
    assert "narrow: frame pane is too narrow" in issues
    assert "narrow: dashboard pane is too narrow" in issues
    assert "narrow: dashboard cards missing" in issues
    assert "narrow: dashboard card collapsed" in issues
    assert "narrow: page has horizontal overflow" in issues
    assert "narrow: missing dashboard text 'Replay'" in issues
