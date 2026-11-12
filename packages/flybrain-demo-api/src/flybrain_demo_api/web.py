"""A tiny local dashboard server for demos: one page plus a Server-Sent Events stream.
Standard library only; binds to 127.0.0.1 by default.

    GET  /                 the page you pass in (self-contained, works offline)
    GET  /api/stream       text/event-stream, one JSON frame per publish
    GET  /api/state        latest frame as JSON
    GET  /api/benchmark    optional offline results JSON, or 204
    POST /api/command      JSON object handed to your on_command; its reply is returned

    hub = FrameHub()
    server = DashboardServer(hub, on_command=lambda cmd: {"ok": True}, page=html).start()
    hub.publish({"yaw": 12.5})          # every connected page receives it
"""

from __future__ import annotations

import json
import logging
import threading
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

log = logging.getLogger(__name__)

DEFAULT_PAGE = (
    b"<!doctype html><meta charset=utf-8><title>demo</title><pre id=f>waiting...</pre>"
    b"<script>new EventSource('/api/stream').onmessage=e=>"
    b"f.textContent=JSON.stringify(JSON.parse(e.data),null,2)</script>"
)


class FrameHub:
    """Latest-frame mailbox with a sequence number that SSE clients wait on."""

    def __init__(self) -> None:
        self._cv = threading.Condition()
        self.seq = 0
        self.frame: dict[str, Any] | None = None
        self.closed = False

    def publish(self, frame: dict[str, Any]) -> None:
        with self._cv:
            self.frame = frame
            self.seq += 1
            self._cv.notify_all()

    def wait(self, after: int, timeout: float) -> tuple[int, dict[str, Any] | None]:
        with self._cv:
            if self.seq <= after and not self.closed:
                self._cv.wait(timeout)
            return self.seq, self.frame

    def close(self) -> None:
        with self._cv:
            self.closed = True
            self._cv.notify_all()


def make_handler(
    hub: FrameHub,
    on_command: Callable[[dict[str, Any]], dict[str, Any]],
    benchmark: Callable[[], dict[str, Any] | None] | None = None,
    page: bytes = b"",
):
    """``page``: the HTML served at /. ``benchmark``: optional offline results for
    /api/benchmark (204 when it returns None)."""
    benchmark = benchmark or (lambda: None)
    page = page or DEFAULT_PAGE

    class Handler(BaseHTTPRequestHandler):
        server_version = "flybrain-demo"
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt: str, *args: Any) -> None:  # quiet
            log.debug("http: " + fmt, *args)

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj: Any, code: int = 200) -> None:
            self._send(code, json.dumps(obj).encode(), "application/json")

        def do_GET(self) -> None:
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(200, page, "text/html; charset=utf-8")
            elif path == "/api/state":
                self._json(hub.frame or {})
            elif path == "/api/benchmark":
                b = benchmark()
                if b is None:
                    self.send_response(HTTPStatus.NO_CONTENT)
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                else:
                    self._json(b)
            elif path == "/api/stream":
                self._stream()
            elif path == "/favicon.ico":
                self.send_response(HTTPStatus.NO_CONTENT)
                self.send_header("Content-Length", "0")
                self.end_headers()
            else:
                self._send(404, b"not found", "text/plain")

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            self.close_connection = True
            seq = 0
            try:
                self.wfile.write(b"retry: 1000\n\n")
                while not hub.closed:
                    new_seq, frame = hub.wait(seq, timeout=1.0)
                    if new_seq == seq or frame is None:
                        self.wfile.write(b": keepalive\n\n")
                    else:
                        seq = new_seq
                        self.wfile.write(b"data: " + json.dumps(frame).encode() + b"\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        def do_POST(self) -> None:
            if self.path != "/api/command":
                self._send(404, b"not found", "text/plain")
                return
            n = int(self.headers.get("Content-Length") or 0)
            if n > 10_000:
                self._json({"ok": False, "error": "too large"}, 413)
                return
            try:
                cmd = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(cmd, dict):
                    raise TypeError("expected a JSON object")
            except (ValueError, TypeError) as e:
                self._json({"ok": False, "error": str(e)}, 400)
                return
            self._json(on_command(cmd))

    return Handler


class DashboardServer:
    def __init__(
        self,
        hub: FrameHub,
        on_command,
        benchmark=None,
        host: str = "127.0.0.1",
        port: int = 0,
        page: bytes = b"",
    ):
        """port 0 picks a free port; read it back from ``url``."""
        self.hub = hub
        self.httpd = ThreadingHTTPServer(
            (host, port), make_handler(hub, on_command, benchmark, page)
        )
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(
            target=self.httpd.serve_forever, name="dashboard", daemon=True
        )

    @property
    def url(self) -> str:
        host, port = self.httpd.server_address[:2]
        return f"http://{host}:{port}/"

    def start(self) -> DashboardServer:
        self.thread.start()
        return self

    def stop(self) -> None:
        self.hub.close()
        self.httpd.shutdown()
        self.httpd.server_close()
