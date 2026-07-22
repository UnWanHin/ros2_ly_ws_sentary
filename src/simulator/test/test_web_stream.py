from __future__ import annotations

import json
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import pytest

from simulator.web_stream import SimulatorWebStream, build_index_html


class FakeSurface:
    def get_size(self) -> tuple[int, int]:
        return (2, 2)


class FakePygameImage:
    @staticmethod
    def tostring(surface: FakeSurface, fmt: str) -> bytes:
        assert surface.get_size() == (2, 2)
        assert fmt == "RGB"
        return bytes(
            [
                255,
                0,
                0,
                0,
                255,
                0,
                0,
                0,
                255,
                255,
                255,
                255,
            ]
        )


class FakePygame:
    image = FakePygameImage()


def started_stream(tmp_path: Path, control_file: Path | None = None) -> SimulatorWebStream:
    stream = SimulatorWebStream(
        host="127.0.0.1",
        port=0,
        fps=1000.0,
        jpeg_quality=75,
        control_file=control_file.as_posix() if control_file is not None else "",
        default_step_sec=12,
    )
    stream.start()
    return stream


def stream_url(stream: SimulatorWebStream, path: str) -> str:
    assert stream._httpd is not None
    port = int(stream._httpd.server_address[1])
    return f"http://127.0.0.1:{port}{path}"


def get_json(stream: SimulatorWebStream, path: str) -> tuple[int, dict[str, Any]]:
    try:
        with urllib.request.urlopen(stream_url(stream, path), timeout=2.0) as response:
            body = response.read().decode("utf-8")
            return response.status, json.loads(body)
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def get_json_body(stream: SimulatorWebStream, path: str) -> tuple[int, str, dict[str, Any]]:
    try:
        with urllib.request.urlopen(stream_url(stream, path), timeout=2.0) as response:
            body = response.read().decode("utf-8")
            return response.status, body, json.loads(body)
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8")
        return exc.code, body, json.loads(body)


def get_text(stream: SimulatorWebStream, path: str) -> tuple[int, str]:
    with urllib.request.urlopen(stream_url(stream, path), timeout=2.0) as response:
        return response.status, response.read().decode("utf-8")


def post_json(stream: SimulatorWebStream, path: str, payload: Any) -> tuple[int, dict[str, Any]]:
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        stream_url(stream, path),
        data=body,
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def post_raw(stream: SimulatorWebStream, path: str, body: bytes) -> tuple[int, dict[str, Any]]:
    request = urllib.request.Request(
        stream_url(stream, path),
        data=body,
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def test_status_and_healthz_report_readiness_before_and_after_frame(tmp_path: Path) -> None:
    control_file = tmp_path / "control.jsonl"
    stream = started_stream(tmp_path, control_file=control_file)
    try:
        status_code, status = get_json(stream, "/status.json")
        assert status_code == 200
        assert status["ok"] is True
        assert status["ready"] is False
        assert status["has_frame"] is False
        assert status["frame_id"] == 0
        assert status["last_frame_time"] == 0.0
        assert status["frame_age_sec"] is None
        assert status["control_enabled"] is True
        assert status["control_file"] == control_file.name
        assert control_file.parent.as_posix() not in json.dumps(status)
        assert status["host"] == "127.0.0.1"
        assert status["port"] > 0
        assert status["fps"] == 1000.0
        assert status["jpeg_quality"] == 75

        health_code, health = get_json(stream, "/healthz")
        assert health_code == 503
        assert health["ok"] is False
        assert health["ready"] is False

        stream.publish_surface(FakeSurface(), FakePygame())

        status_code, status = get_json(stream, "/status.json")
        assert status_code == 200
        assert status["ok"] is True
        assert status["ready"] is True
        assert status["has_frame"] is True
        assert status["frame_id"] == 1
        assert status["last_frame_time"] > 0
        assert 0 <= status["frame_age_sec"] < 2.0

        health_code, health = get_json(stream, "/healthz")
        assert health_code == 200
        assert health["ok"] is True
        assert health["ready"] is True
    finally:
        stream.stop()


def test_stream_rejects_non_finite_fps() -> None:
    with pytest.raises(ValueError, match="fps must be > 0"):
        SimulatorWebStream(host="127.0.0.1", port=0, fps=float("nan"))


def test_status_includes_json_safe_replay_metadata(tmp_path: Path) -> None:
    stream = started_stream(tmp_path)
    try:
        stream.update_metadata(
            {
                "trace": {"name": "sample_trace.jsonl", "records": 6},
                "validation": {"status": "PASS"},
                "replay": {"current_index": 2, "playing": False},
                "current_record": {"tick": 3, "goal": {"name": "OccupyArea"}},
                "simulator_inputs": {
                    "enabled": True,
                    "state": {
                        "runtime": {"self_health": 210, "ammo_left": 18, "posture": 2},
                        "summary": {"enemy_units": 1, "low_hp_units": ["enemy:Hero"]},
                    },
                },
            }
        )

        status_code, status = get_json(stream, "/status.json")

        assert status_code == 200
        assert status["trace"] == {"name": "sample_trace.jsonl", "records": 6}
        assert status["validation"] == {"status": "PASS"}
        assert status["replay"] == {"current_index": 2, "playing": False}
        assert status["current_record"] == {"tick": 3, "goal": {"name": "OccupyArea"}}
        assert status["simulator_inputs"]["state"]["runtime"]["self_health"] == 210
        assert status["simulator_inputs"]["state"]["summary"]["low_hp_units"] == ["enemy:Hero"]
    finally:
        stream.stop()


def test_status_metadata_cannot_override_core_stream_fields(tmp_path: Path) -> None:
    stream = started_stream(tmp_path)
    try:
        stream.update_metadata(
            {
                "ok": False,
                "ready": True,
                "frame_id": 99,
                "control_file": "/tmp/private/control.jsonl",
                "trace": {"name": "decision_trace.jsonl"},
            }
        )

        status_code, status = get_json(stream, "/status.json")

        assert status_code == 200
        assert status["ok"] is True
        assert status["ready"] is False
        assert status["frame_id"] == 0
        assert status["control_file"] == ""
        assert status["trace"] == {"name": "decision_trace.jsonl"}
        assert "/tmp/private" not in json.dumps(status)
    finally:
        stream.stop()


def test_status_metadata_replaces_non_finite_numbers_with_null(tmp_path: Path) -> None:
    stream = started_stream(tmp_path)
    try:
        stream.update_metadata(
            {
                "trace": {"duration_sec": float("nan")},
                "replay": {"speed": float("inf"), "match_time_left": -float("inf")},
                "current_record": {"t": float("nan"), "goal": {"pos_cm": [1075.0, float("inf")]}},
            }
        )

        status_code, body, status = get_json_body(stream, "/status.json")

        assert status_code == 200
        assert "NaN" not in body
        assert "Infinity" not in body
        assert status["trace"]["duration_sec"] is None
        assert status["replay"]["speed"] is None
        assert status["replay"]["match_time_left"] is None
        assert status["current_record"]["t"] is None
        assert status["current_record"]["goal"]["pos_cm"] == [1075.0, None]
    finally:
        stream.stop()


def test_frame_endpoint_returns_jpeg_after_publish(tmp_path: Path) -> None:
    stream = started_stream(tmp_path)
    try:
        try:
            urllib.request.urlopen(stream_url(stream, "/frame.jpg"), timeout=2.0)
        except urllib.error.HTTPError as exc:
            assert exc.code == 503
            assert exc.read().decode("utf-8") == "no frame yet"
        else:
            raise AssertionError("frame endpoint should be unavailable before first publish")

        stream.publish_surface(FakeSurface(), FakePygame())

        with urllib.request.urlopen(stream_url(stream, "/frame.jpg"), timeout=2.0) as response:
            frame = response.read()

        assert response.status == 200
        assert response.headers["Content-Type"] == "image/jpeg"
        assert frame.startswith(b"\xff\xd8")
    finally:
        stream.stop()


def test_control_api_writes_command_and_rejects_bad_payloads(tmp_path: Path) -> None:
    control_file = tmp_path / "control.jsonl"
    stream = started_stream(tmp_path, control_file=control_file)
    try:
        status_code, payload = post_json(stream, "/api/control", {"command": "rewind", "seconds": 3})
        assert status_code == 200
        assert payload["ok"] is True
        assert payload["command"] == "rewind"
        assert payload["payload"] == {"seconds": 3.0}
        assert payload["control_enabled"] is True
        assert payload["control_file"] == control_file.name
        assert control_file.parent.as_posix() not in json.dumps(payload)

        written = json.loads(control_file.read_text(encoding="utf-8").splitlines()[0])
        assert written["command"] == "rewind"
        assert written["seconds"] == 3.0

        status_code, payload = post_json(stream, "/api/control", {"command": "unknown"})
        assert status_code == 400
        assert payload["ok"] is False
        assert "unsupported command" in payload["message"]

        status_code, payload = post_raw(stream, "/api/control", b"{")
        assert status_code == 400
        assert payload["ok"] is False
        assert payload["message"] == "invalid json"
    finally:
        stream.stop()


def test_index_page_does_not_disclose_absolute_control_file_path(tmp_path: Path) -> None:
    control_file = tmp_path / "control.jsonl"
    stream = started_stream(tmp_path, control_file=control_file)
    try:
        status_code, body = get_text(stream, "/")

        assert status_code == 200
        assert "control: enabled (control.jsonl)" in body
        assert "simulator status dashboard" in body
        assert "Simulator Inputs" in body
        assert "Current Decision" in body
        assert "validationPill" in body
        assert "updateDashboard" in body
        assert control_file.parent.as_posix() not in body
    finally:
        stream.stop()


def test_index_page_escapes_control_file_label(tmp_path: Path) -> None:
    control_file = tmp_path / "control<&bad>.jsonl"
    stream = started_stream(tmp_path, control_file=control_file)
    try:
        status_code, body = get_text(stream, "/")

        assert status_code == 200
        assert "control: enabled (control&lt;&amp;bad&gt;.jsonl)" in body
        assert "control<&bad>.jsonl" not in body
    finally:
        stream.stop()


def test_build_index_html_disables_controls_without_control_file() -> None:
    body = build_index_html(port=9010, control_enabled=False, control_label="", default_step_sec=7).decode("utf-8")

    assert "Sentinel Flight Deck" in body
    assert "--fd-root:#111418" in body
    assert "border-radius:12px" in body
    assert "control: disabled (no control_file)" in body
    assert "const controlsDisabled = true;" in body
    assert "data-seconds=\"7\"" in body
    assert "simulator status dashboard" in body


def test_control_api_reports_disabled_control_file(tmp_path: Path) -> None:
    stream = started_stream(tmp_path)
    try:
        status_code, status = get_json(stream, "/status.json")
        assert status_code == 200
        assert status["control_enabled"] is False
        assert status["control_file"] == ""

        status_code, payload = post_json(stream, "/api/control", {"command": "start"})
        assert status_code == 503
        assert payload["ok"] is False
        assert payload["message"] == "control_file not configured"
    finally:
        stream.stop()
