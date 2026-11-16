"""The Flydeck hub: a local web page (127.0.0.1 only) to pick a robot and run one demo.

    GET  /                    the Flydeck page
    GET  /guide               the Flydeck guide
    GET  /demos/<id>/guide    a demo's guide
    GET  /assets/...          the shared stylesheet, fonts and marks (flybrain_demo_api.assets)
    GET  /docs/<name>.md      a doc from the checkout (plain text)
    GET  /api/state           everything the page shows, as JSON
    GET  /api/stream          the same, as Server-Sent Events, on every change
    POST /api/<action>        scan | ports | select | check | start | stop | reload | quit

POSTs must be JSON from this page's own origin, so another web page open in the browser cannot
start a robot (cross-site requests need a preflight we never grant; Host and Origin are checked
against DNS rebinding).
"""

from __future__ import annotations

import contextlib
import json
import logging
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources
from pathlib import Path
from typing import Any

from flybrain_demo_api import SpecError, assets
from flybrain_edge.bittle import find_serial_ports

from . import registry
from .robotcheck import PREFIX as RESULT_PREFIX
from .runner import BLE_KILLED, Runner, StartRefused
from .settings import Settings

log = logging.getLogger(__name__)

NAME = "Flydeck"
TAGLINE = "Pick a robot, pick a fly-brain demo, press Start."
DEFAULT_PORT = 8760
TERMINAL_BLE_HINT = (
    "Started from a terminal: macOS will stop any Bluetooth use. For Bluetooth, use the "
    "Flydeck app (see the README; it can run the demos of this folder too). USB and the "
    "simulator work here."
)


def in_app() -> bool:
    return os.environ.get("FLYDECK_APP") == "1"


def ble_blocked() -> bool:
    """True when this process is not inside an app bundle that may use Bluetooth (macOS)."""
    return sys.platform == "darwin" and os.environ.get("FLYBRAIN_BLE_BUNDLE") != "1"


class Hub:
    def __init__(
        self,
        root: Path | None = None,
        python: str = sys.executable,
        settings: Settings | None = None,
        workdir: Path | None = None,
        stop_timeout_s: float = 20.0,
        check_timeout_s: float = 45.0,
        entries: list[registry.DemoEntry] | None = None,
    ):
        self.root = root
        self.workdir = workdir or root  # where demos run (they write runs/ there)
        self.python = python
        self.settings = settings or Settings()
        self.check_timeout_s = check_timeout_s
        self._cv = threading.Condition()
        self.version = 0
        self.closed = False
        self.quit_event = threading.Event()
        self.runner = Runner(
            python=python, cwd=self.workdir, stop_timeout_s=stop_timeout_s, on_change=self.changed
        )
        self.entries = entries if entries is not None else registry.discover(root)
        self.robot: dict[str, Any] = {
            "selected": self.settings.get("robot"),
            "check": None,
            "checking": False,
            "scanning": False,
            "scan": None,
            "ports": [],
        }
        self.notice: dict[str, Any] | None = None
        self.refresh_ports()

    # ---- change notification ---------------------------------------------------------
    def changed(self) -> None:
        with self._cv:
            self.version += 1
            self._cv.notify_all()

    def wait_change(self, after: int, timeout: float) -> int:
        with self._cv:
            if self.version <= after and not self.closed:
                self._cv.wait(timeout)
            return self.version

    def say(self, text: str, kind: str = "info") -> None:
        self.notice = {"text": text, "kind": kind, "t": time.time()}
        self.changed()

    # ---- state -----------------------------------------------------------------------
    def entry(self, demo_id: str) -> registry.DemoEntry | None:
        return next((e for e in self.entries if e.id == demo_id), None)

    def state(self) -> dict[str, Any]:
        run = self.runner.snapshot()
        held_by = run["title"] if run and run["active"] and run["transport"] != "mock" else None
        return {
            "app": {
                "name": NAME,
                "tagline": TAGLINE,
                "in_app": in_app(),
                "ble_blocked": ble_blocked(),
                "ble_hint": TERMINAL_BLE_HINT if ble_blocked() else None,
                "root": str(self.root) if self.root else None,
                "workdir": str(self.workdir) if self.workdir else None,
            },
            "version": self.version,
            "demos": [
                {**e.to_json(), "saved": self.settings.demo_options(e.id)} for e in self.entries
            ],
            "robot": {**self.robot, "held_by": held_by},
            "run": run,
            "notice": self.notice,
        }

    # ---- robot -----------------------------------------------------------------------
    def refresh_ports(self) -> list[str]:
        try:
            self.robot["ports"] = find_serial_ports()
        except Exception as e:  # noqa: BLE001 - enumeration is best effort
            log.warning("serial port listing failed: %s", e)
            self.robot["ports"] = []
        self.changed()
        return self.robot["ports"]

    def _worker(self, args: list[str]) -> dict[str, Any]:
        """Run flydeck.robotcheck in a child process; its last line is the JSON result."""
        cmd = [self.python, "-m", "flydeck.robotcheck", *args]
        env = dict(os.environ, FLYBRAIN_BLE_NO_HINT="1", PYTHONUNBUFFERED="1")
        try:
            p = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self.check_timeout_s,
                cwd=self.workdir,
                env=env,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return {"ok": False, "error": f"no answer within {self.check_timeout_s:g} s"}
        for line in reversed(p.stdout.splitlines()):
            if line.startswith(RESULT_PREFIX):
                try:
                    return json.loads(line[len(RESULT_PREFIX) :])
                except ValueError:
                    break
        if p.returncode in (134, -6) and sys.platform == "darwin":
            return {"ok": False, "error": BLE_KILLED.replace("the demo", "the check")}
        tail = (p.stderr or p.stdout).strip().splitlines()[-1:] or ["no output"]
        return {"ok": False, "error": f"the check stopped (exit {p.returncode}): {tail[0]}"}

    def _busy_reason(self) -> str | None:
        snap = self.runner.snapshot()
        if snap and snap["active"] and snap["transport"] != "mock":
            return f"The robot is connected to {snap['title']}. Stop it first."
        return None

    def scan(self, background: bool = True) -> dict[str, Any]:
        if self.robot["scanning"]:
            return {"ok": True, "note": "already scanning"}
        if why := self._busy_reason():
            return {"ok": False, "error": why}
        self.robot["scanning"] = True
        self.changed()

        def go() -> None:
            res = self._worker(["scan", "--timeout", "5"])
            res["at"] = time.time()
            if res.get("ok") and not res.get("devices"):
                res["hint"] = (
                    "No robot found. Is its battery on (long-press the battery button)? Is the "
                    "Petoi phone app connected to it? Close the phone app and scan again."
                )
            self.robot["scan"] = res
            self.robot["scanning"] = False
            self.changed()

        if background:
            threading.Thread(target=go, daemon=True).start()
        else:
            go()
        return {"ok": True}

    def select(self, transport: str, target: str | None, name: str | None = None) -> dict:
        if transport not in ("ble", "usb"):
            return {"ok": False, "error": "choose Bluetooth or USB"}
        if not target:
            return {"ok": False, "error": "no robot given"}
        sel = {"transport": transport, "target": target, "name": name or target}
        self.robot["selected"] = sel
        self.robot["check"] = None
        self.settings.set("robot", sel)
        self.changed()
        return {"ok": True}

    def check(self, background: bool = True, mock: bool = False) -> dict[str, Any]:
        sel = self.robot["selected"]
        if not mock and not sel:
            return {"ok": False, "error": "Choose a robot first."}
        if self.robot["checking"]:
            return {"ok": True, "note": "already checking"}
        if why := self._busy_reason():
            return {"ok": False, "error": why}
        args = (
            ["check", "--mock"]
            if mock
            else ["check", "--ble" if sel["transport"] == "ble" else "--port", sel["target"]]
        )
        self.robot["checking"] = True
        self.changed()

        def go() -> None:
            res = self._worker(args)
            res["at"] = time.time()
            if res.get("ok") and res.get("low_power"):
                res["warning"] = (
                    f"The robot says '{res['low_power']}'. Its motors are off: long-press the "
                    "battery button to switch the battery on, or charge it."
                )
            self.robot["check"] = res
            self.robot["checking"] = False
            self.changed()

        if background:
            threading.Thread(target=go, daemon=True).start()
        else:
            go()
        return {"ok": True}

    # ---- demos -----------------------------------------------------------------------
    def start(
        self,
        demo_id: str,
        options: dict[str, Any] | None = None,
        simulator: bool = False,
        space: bool = False,
    ) -> dict[str, Any]:
        e = self.entry(demo_id)
        if e is None:
            return {"ok": False, "error": f"no demo called {demo_id!r}"}
        if not e.ok or e.spec is None:
            return {"ok": False, "error": f"{e.id} cannot start: {e.error}"}
        spec = e.spec
        known = {o.name for o in spec.options}
        opts = {k: v for k, v in (options or {}).items() if k in known}
        if simulator:
            if not spec.supports_mock:
                return {"ok": False, "error": f"{spec.title} has no simulator."}
            transport, target = "mock", None
        else:
            sel = self.robot["selected"]
            if not sel:
                return {
                    "ok": False,
                    "error": "No robot chosen. Scan and pick one in the Robot panel, or tick "
                    "'Try in simulator'.",
                }
            transport, target = sel["transport"], sel["target"]
            if self.robot["checking"] or self.robot["scanning"]:
                return {"ok": False, "error": "Wait until the robot check or scan finishes."}
        try:
            run = self.runner.start(spec, transport, opts, target, space_confirmed=space)
        except StartRefused as err:
            return {"ok": False, "error": str(err)}
        self.settings.set_demo_options(spec.id, run.options)
        return {"ok": True, "run": run.to_json()}

    def preview(self, demo_id: str, options: dict[str, Any], simulator: bool) -> dict[str, Any]:
        """What Start would run with these settings, and whether it needs the space check."""
        e = self.entry(demo_id)
        if e is None or e.spec is None:
            return {"ok": False, "error": "unknown demo"}
        spec = e.spec
        sel = self.robot["selected"]
        transport = "mock" if simulator else (sel["transport"] if sel else "ble")
        target = None if simulator or not sel else sel["target"]
        known = {o.name for o in spec.options}
        try:
            clean = spec.clean_options({k: v for k, v in options.items() if k in known}, transport)
            argv = spec.argv(transport, clean, spec.http_port or 0, target, True)
        except SpecError as err:
            return {"ok": False, "error": str(err)}
        return {
            "ok": True,
            "needs_space": spec.needs_space(clean, transport),
            "command": " ".join(["python -m", spec.module, *argv]),
        }

    def stop(self) -> dict[str, Any]:
        run = self.runner.stop()
        return {"ok": True, "running": bool(run and run.active)}

    def reload(self) -> dict[str, Any]:
        if self.runner.busy:
            return {"ok": False, "error": "Stop the running demo first."}
        self.entries = registry.discover(self.root, refresh=True)
        self.changed()
        return {"ok": True, "demos": len(self.entries)}

    def shutdown(self, timeout: float | None = None) -> None:
        """Stop any demo (the robot rests), then let the server go."""
        self.runner.stop(wait=True, timeout=timeout)
        with self._cv:
            self.closed = True
            self._cv.notify_all()
        self.quit_event.set()

    def handle(self, action: str, body: dict[str, Any]) -> dict[str, Any]:
        if action == "scan":
            return self.scan()
        if action == "ports":
            return {"ok": True, "ports": self.refresh_ports()}
        if action == "select":
            return self.select(body.get("transport", ""), body.get("target"), body.get("name"))
        if action == "check":
            return self.check(mock=bool(body.get("simulator")))
        if action == "start":
            return self.start(
                str(body.get("demo", "")),
                body.get("options") or {},
                simulator=bool(body.get("simulator")),
                space=bool(body.get("space")),
            )
        if action == "preview":
            return self.preview(
                str(body.get("demo", "")), body.get("options") or {}, bool(body.get("simulator"))
            )
        if action == "stop":
            return self.stop()
        if action == "reload":
            return self.reload()
        if action == "quit":
            threading.Thread(target=self.shutdown, daemon=True).start()
            return {"ok": True}
        return {"ok": False, "error": f"unknown action {action!r}"}


# ---- HTTP ------------------------------------------------------------------------------
def _page() -> bytes:
    return resources.files("flydeck").joinpath("static/index.html").read_bytes()


def _app_guide(root: Path | None) -> Path | None:
    for base in (root / "apps" / "flydeck" if root else None, Path(__file__).parents[2]):
        if base is not None and (base / "guide.html").is_file():
            return base / "guide.html"
    return None


def make_handler(hub: Hub, page: bytes | None = None):
    """``page``: fixed HTML for / (tests); by default the packaged page, read per request."""

    class Handler(BaseHTTPRequestHandler):
        server_version = "flydeck"
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt: str, *args: Any) -> None:
            log.debug("http: " + fmt, *args)

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj: Any, code: int = 200) -> None:
            self._send(code, json.dumps(obj).encode(), "application/json")

        def _local_host(self) -> bool:
            port = self.server.server_address[1]
            host = (self.headers.get("Host") or "").lower()
            return host in (f"127.0.0.1:{port}", f"localhost:{port}")

        def do_GET(self) -> None:
            if not self._local_host():
                self._send(403, b"forbidden", "text/plain")
                return
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(200, page or _page(), "text/html; charset=utf-8")
            elif path == "/api/state":
                self._json(hub.state())
            elif path == "/api/stream":
                self._stream()
            elif path == "/guide":
                g = _app_guide(hub.root)
                if g is None:
                    self._send(404, b"guide not found", "text/plain")
                else:
                    self._send(200, g.read_bytes(), "text/html; charset=utf-8")
            elif path.startswith("/docs/") and path.endswith(".md") and hub.root is not None:
                name = path[len("/docs/") :]
                doc = hub.root / "docs" / name
                if "/" in name or ".." in name or not doc.is_file():
                    self._send(404, b"not found", "text/plain")
                else:
                    self._send(200, doc.read_bytes(), "text/plain; charset=utf-8")
            elif path.startswith("/demos/") and path.endswith("/guide"):
                e = hub.entry(path[len("/demos/") : -len("/guide")])
                g = e.guide_path() if e else None
                if g is None:
                    self._send(404, b"no guide", "text/plain")
                else:
                    html = g.suffix.lower() in (".html", ".htm")
                    ctype = "text/html" if html else "text/plain"
                    self._send(200, g.read_bytes(), f"{ctype}; charset=utf-8")
            elif path.startswith(assets.PREFIX) and (found := assets.asset(path)):
                self._send(200, *found)
            elif path == "/favicon.ico":
                self._send(204, b"", "text/plain")
            else:
                self._send(404, b"not found", "text/plain")

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.close_connection = True
            seen = -1
            try:
                self.wfile.write(b"retry: 1000\n\n")
                while not hub.closed:
                    v = hub.wait_change(seen, timeout=2.0)
                    if v == seen:
                        self.wfile.write(b": keepalive\n\n")
                    else:
                        seen = v
                        data = json.dumps(hub.state()).encode()
                        self.wfile.write(b"data: " + data + b"\n\n")
                    self.wfile.flush()
                # tell the page we are gone on purpose
                self.wfile.write(b"event: bye\ndata: {}\n\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        def do_POST(self) -> None:
            path = self.path.split("?", 1)[0]
            if not path.startswith("/api/") or not self._local_host():
                self._send(404, b"not found", "text/plain")
                return
            origin = self.headers.get("Origin")
            port = self.server.server_address[1]
            if origin and origin not in (f"http://127.0.0.1:{port}", f"http://localhost:{port}"):
                self._json({"ok": False, "error": "cross-site request refused"}, 403)
                return
            if not (self.headers.get("Content-Type") or "").startswith("application/json"):
                self._json({"ok": False, "error": "send JSON"}, 415)
                return
            n = int(self.headers.get("Content-Length") or 0)
            if n > 50_000:
                self._json({"ok": False, "error": "too large"}, 413)
                return
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(body, dict):
                    raise TypeError("expected a JSON object")
            except (ValueError, TypeError) as e:
                self._json({"ok": False, "error": str(e)}, 400)
                return
            try:
                self._json(hub.handle(path[len("/api/") :], body))
            except Exception as e:
                log.exception("action failed")
                self._json({"ok": False, "error": f"{type(e).__name__}: {e}"}, 500)

    return Handler


class HubServer:
    def __init__(self, hub: Hub, host: str = "127.0.0.1", port: int = DEFAULT_PORT):
        self.hub = hub
        try:
            self.httpd = ThreadingHTTPServer((host, port), make_handler(hub))
        except OSError:
            if port == 0:
                raise
            self.httpd = ThreadingHTTPServer((host, 0), make_handler(hub))
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    @property
    def url(self) -> str:
        host, port = self.httpd.server_address[:2]
        return f"http://{host}:{port}/"

    def start(self) -> HubServer:
        self.thread.start()
        return self

    def stop(self) -> None:
        with contextlib.suppress(Exception):
            self.httpd.shutdown()
        self.httpd.server_close()


def watch_parent(pid: int, hub: Hub, interval: float = 1.0) -> threading.Thread:
    """If the app that started us disappears (force quit), stop the demo and exit."""

    def run() -> None:
        while not hub.quit_event.is_set():
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                log.warning("parent %d is gone: stopping", pid)
                hub.shutdown()
                return
            except PermissionError:
                pass
            hub.quit_event.wait(interval)

    t = threading.Thread(target=run, name="parent-watch", daemon=True)
    t.start()
    return t
