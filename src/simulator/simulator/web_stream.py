from __future__ import annotations

import io
import json
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from .control_bus import append_command, normalize_api_control_payload


class SimulatorWebStream:
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 9000,
        fps: float = 12.0,
        jpeg_quality: int = 80,
        control_file: str = "",
        default_step_sec: int = 10,
    ) -> None:
        if fps <= 0:
            raise ValueError("fps must be > 0")
        if not (1 <= jpeg_quality <= 100):
            raise ValueError("jpeg_quality must be in [1, 100]")
        self.host = host
        self.port = int(port)
        self.frame_interval = 1.0 / float(fps)
        self.jpeg_quality = int(jpeg_quality)

        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._frame_jpeg: bytes | None = None
        self._frame_id = 0
        self._last_publish_monotonic = 0.0
        self._last_frame_wall_time = 0.0
        self.control_file: Path | None = None
        if str(control_file).strip():
            self.control_file = Path(control_file).expanduser().resolve()
        self.default_step_sec = max(1, int(default_step_sec))

        try:
            from PIL import Image  # noqa: F401
        except ImportError as exc:
            raise RuntimeError("Pillow is required for web stream: pip install Pillow") from exc

    def start(self) -> None:
        if self._httpd is not None:
            return
        handler_cls = self._build_handler()
        self._httpd = ThreadingHTTPServer((self.host, self.port), handler_cls)
        self._thread = threading.Thread(target=self._httpd.serve_forever, name="simulator-web-stream", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        httpd = self._httpd
        thread = self._thread
        self._httpd = None
        self._thread = None
        if httpd is not None:
            httpd.shutdown()
            httpd.server_close()
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

    def publish_surface(self, surface: Any, pygame: Any) -> None:
        now = time.monotonic()
        if (now - self._last_publish_monotonic) < self.frame_interval:
            return
        self._last_publish_monotonic = now

        width, height = surface.get_size()
        raw_rgb = pygame.image.tostring(surface, "RGB")
        from PIL import Image

        image = Image.frombytes("RGB", (width, height), raw_rgb)
        out = io.BytesIO()
        image.save(out, format="JPEG", quality=self.jpeg_quality, optimize=False)
        payload = out.getvalue()

        with self._lock:
            self._frame_jpeg = payload
            self._frame_id += 1
            self._last_frame_wall_time = time.time()

    def snapshot(self) -> tuple[bytes | None, int, float]:
        with self._lock:
            return self._frame_jpeg, self._frame_id, self._last_frame_wall_time

    def submit_control(self, command: str, payload: dict[str, Any] | None = None) -> tuple[bool, str]:
        if self.control_file is None:
            return (False, "control_file not configured")
        try:
            append_command(self.control_file, command, payload)
            return (True, "ok")
        except OSError as exc:
            return (False, str(exc))

    def _build_handler(self):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    self._serve_index()
                    return
                if path == "/frame.jpg":
                    self._serve_frame()
                    return
                if path == "/status.json":
                    self._serve_status()
                    return
                self.send_error(HTTPStatus.NOT_FOUND, "not found")

            def do_POST(self) -> None:  # noqa: N802
                path = urlparse(self.path).path
                if path == "/api/control":
                    self._serve_control()
                    return
                self.send_error(HTTPStatus.NOT_FOUND, "not found")

            def log_message(self, format: str, *args) -> None:  # noqa: A003
                return

            def _serve_index(self) -> None:
                disabled = "false" if outer.control_file is not None else "true"
                control_hint = (
                    f"control: {outer.control_file.as_posix()}"
                    if outer.control_file is not None
                    else "control: disabled (no control_file)"
                )
                body = (
                    "<!doctype html><html><head><meta charset='utf-8'>"
                    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
                    "<title>Simulator Live</title>"
                    "<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif}"
                    "#bar{padding:8px 12px;background:#1d232b;border-bottom:1px solid #333}"
                    "#wrap{padding:8px}img{max-width:100%;height:auto;display:block;background:#000}"
                    "#ctrl{padding:8px 12px;background:#14191f;border-bottom:1px solid #303844;display:flex;"
                    "gap:8px;flex-wrap:wrap;align-items:center}button{padding:6px 10px;background:#2a3240;"
                    "color:#e2e8f0;border:1px solid #3a4658;border-radius:4px;cursor:pointer}"
                    "button:disabled{opacity:.45;cursor:not-allowed}#msg{color:#a8b0ba;font-size:12px}"
                    "</style></head><body>"
                    f"<div id='bar'>Simulator Live Stream :{outer.port}</div>"
                    "<div id='ctrl'>"
                    "<button data-cmd='start'>Start</button>"
                    "<button data-cmd='pause'>Pause</button>"
                    f"<button data-cmd='rewind' data-seconds='{outer.default_step_sec}'>+{outer.default_step_sec}s</button>"
                    f"<button data-cmd='forward' data-seconds='{outer.default_step_sec}'>-{outer.default_step_sec}s</button>"
                    "<button data-cmd='reset'>Reset</button>"
                    f"<span id='msg'>{control_hint}</span>"
                    "</div>"
                    "<div id='wrap'><img id='f' alt='frame'></div>"
                    "<script>"
                    "const img=document.getElementById('f');"
                    "const msg=document.getElementById('msg');"
                    f"const controlsDisabled={disabled};"
                    "function tick(){img.src='/frame.jpg?t='+Date.now();}"
                    "setInterval(tick,80);tick();"
                    "function postControl(cmd,seconds){"
                    "if(controlsDisabled){return;}"
                    "const body=(seconds===undefined)?{command:cmd}:{command:cmd,seconds:Number(seconds)};"
                    "fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})"
                    ".then(r=>r.json().catch(()=>({ok:false,message:'bad response'})))"
                    ".then(d=>{msg.textContent=(d&&d.ok)?('ok: '+cmd):('fail: '+(d&&d.message?d.message:'unknown'));})"
                    ".catch(e=>{msg.textContent='fail: '+e;});}"
                    "for(const btn of document.querySelectorAll('button[data-cmd]')){"
                    "if(controlsDisabled){btn.disabled=true;continue;}"
                    "btn.addEventListener('click',()=>postControl(btn.dataset.cmd,btn.dataset.seconds));}"
                    "</script></body></html>"
                ).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_frame(self) -> None:
                frame, _, _ = outer.snapshot()
                if frame is None:
                    body = b"no frame yet"
                    self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)

            def _serve_status(self) -> None:
                _, frame_id, last_frame_time = outer.snapshot()
                payload = json.dumps(
                    {
                        "ok": True,
                        "frame_id": frame_id,
                        "last_frame_time": last_frame_time,
                        "host": outer.host,
                        "port": outer.port,
                    }
                ).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def _serve_control(self) -> None:
                raw_len = self.headers.get("Content-Length", "0").strip()
                try:
                    length = max(0, int(raw_len))
                except ValueError:
                    length = 0
                body = self.rfile.read(length) if length > 0 else b"{}"
                try:
                    data = json.loads(body.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self._json_response(HTTPStatus.BAD_REQUEST, {"ok": False, "message": "invalid json"})
                    return
                if not isinstance(data, dict):
                    self._json_response(HTTPStatus.BAD_REQUEST, {"ok": False, "message": "payload must be object"})
                    return

                command, payload, error = normalize_api_control_payload(data, outer.default_step_sec)
                if error is not None or command is None:
                    self._json_response(HTTPStatus.BAD_REQUEST, {"ok": False, "message": error or "invalid command"})
                    return

                ok, message = outer.submit_control(command, payload or None)
                status = HTTPStatus.OK if ok else HTTPStatus.SERVICE_UNAVAILABLE
                self._json_response(
                    status,
                    {
                        "ok": ok,
                        "message": message,
                        "command": command,
                        "payload": payload,
                        "control_file": outer.control_file.as_posix() if outer.control_file is not None else "",
                    },
                )

            def _json_response(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
                body = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        return Handler
