"""Run one demo at a time as a child process and watch it.

Start: ``python -u -m <spec.module> <argv>`` with FLYDECK_STATUS=1, in its own process group,
stdout and stderr merged into a log tail. Status lines (flybrain_demo_api.status) move the state:

    starting -> connecting -> (ready) running / calibrating / attention -> stopping -> stopped
                                                                                    -> failed

Stop: one SIGINT (the demo then stops its loop and rests the robot), wait up to
``stop_timeout_s``; if it is still alive, SIGKILL and say plainly that the robot may still be
standing.
"""

from __future__ import annotations

import contextlib
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from flybrain_demo_api import EXIT_CODES, DemoSpec, SpecError, status

ROBOT_MAY_STAND = (
    "The robot may still be standing or walking: switch it off with its battery button, or lift it."
)
BLE_KILLED = (
    "macOS stopped the demo when it tried to use Bluetooth: this Flydeck was started from a "
    "terminal, which has no Bluetooth permission. Install and open the Flydeck app "
    "(uv run flydeck install-app), or use the USB cable."
)
ACTIVE = ("starting", "connecting", "calibrating", "running", "attention", "stopping")


class StartRefused(Exception):
    """Start was refused; the message says why, in plain words."""


def port_is_free(port: int, host: str = "127.0.0.1") -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind((host, port))
        except OSError:
            return False
    return True


def pick_port(preferred: int = 0, host: str = "127.0.0.1") -> int:
    """``preferred`` when it is free, otherwise any free port."""
    if preferred and port_is_free(preferred, host):
        return preferred
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, 0))
        return s.getsockname()[1]


@dataclass
class DemoRun:
    demo_id: str
    title: str
    transport: str
    robot: str | None
    options: dict[str, Any]
    argv: list[str]
    http_port: int
    pid: int | None = None
    state: str = "starting"
    detail: str = ""
    url: str | None = None
    ready: bool = False
    warnings: list[str] = field(default_factory=list)
    log: deque = field(default_factory=lambda: deque(maxlen=600))
    log_seq: int = 0
    started: float = field(default_factory=time.time)
    ended: float | None = None
    exit_code: int | None = None
    reason: str | None = None
    stop_requested: bool = False
    killed: bool = False
    failed_event: str | None = None

    @property
    def active(self) -> bool:
        return self.ended is None

    def to_json(self, log_lines: int = 200) -> dict[str, Any]:
        return {
            "demo_id": self.demo_id,
            "title": self.title,
            "transport": self.transport,
            "robot": self.robot,
            "options": self.options,
            "command": self.argv,
            "http_port": self.http_port,
            "pid": self.pid,
            "state": self.state,
            "detail": self.detail,
            "url": self.url,
            "ready": self.ready,
            "warnings": self.warnings[-5:],
            "log": list(self.log)[-log_lines:],
            "log_seq": self.log_seq,
            "started": self.started,
            "ended": self.ended,
            "exit_code": self.exit_code,
            "reason": self.reason,
            "active": self.active,
        }


class Runner:
    def __init__(
        self,
        python: str = sys.executable,
        cwd: str | Path | None = None,
        env: dict[str, str] | None = None,
        stop_timeout_s: float = 20.0,
        on_change: Callable[[], None] | None = None,
    ):
        self.python = python
        self.cwd = str(cwd) if cwd else None
        self.env = env
        self.stop_timeout_s = stop_timeout_s
        self.on_change = on_change or (lambda: None)
        self.current: DemoRun | None = None
        self._proc: subprocess.Popen | None = None
        self._lock = threading.RLock()
        self._done = threading.Event()
        self._done.set()

    # ---- queries ------------------------------------------------------------------
    @property
    def busy(self) -> bool:
        return self.current is not None and self.current.active

    def snapshot(self) -> dict[str, Any] | None:
        with self._lock:
            return None if self.current is None else self.current.to_json()

    # ---- start --------------------------------------------------------------------
    def build(
        self,
        spec: DemoSpec,
        transport: str,
        options: dict[str, Any],
        robot: str | None = None,
        space_confirmed: bool = False,
        http_port: int | None = None,
    ) -> tuple[list[str], dict[str, Any], int]:
        """Check a start request; return (argv, cleaned options, dashboard port)."""
        if transport not in spec.transports:
            raise StartRefused(f"{spec.title} cannot run over {transport}.")
        try:
            clean = spec.clean_options(options, transport)
        except SpecError as e:
            raise StartRefused(str(e)) from e
        if spec.needs_space(clean, transport) and not space_confirmed:
            raise StartRefused(
                "These settings move the robot's legs. Make room around it, then tick "
                "'The robot has space' and press Start again."
            )
        port = pick_port(spec.http_port) if http_port is None else http_port
        argv = spec.argv(transport, clean, port, robot, space_confirmed)
        return argv, clean, port

    def start(
        self,
        spec: DemoSpec,
        transport: str,
        options: dict[str, Any],
        robot: str | None = None,
        space_confirmed: bool = False,
        http_port: int | None = None,
    ) -> DemoRun:
        with self._lock:
            if self.busy:
                assert self.current is not None
                raise StartRefused(
                    f"{self.current.title} is still running. Stop it first: one demo at a time."
                )
            argv, clean, port = self.build(
                spec, transport, options, robot, space_confirmed, http_port
            )
            run = DemoRun(
                demo_id=spec.id,
                title=spec.title,
                transport=transport,
                robot=robot,
                options=clean,
                argv=[spec.module, *argv],
                http_port=port,
            )
            env = dict(os.environ if self.env is None else self.env)
            env[status.ENV] = "1"
            env["PYTHONUNBUFFERED"] = "1"
            env.setdefault("FLYBRAIN_BLE_NO_HINT", "1")  # Flydeck explains a BLE kill itself
            cmd = [self.python, "-u", "-m", spec.module, *argv]
            self._add_log(run, "$ " + " ".join(["python", "-m", spec.module, *argv]))
            try:
                proc = subprocess.Popen(
                    cmd,
                    cwd=self.cwd,
                    env=env,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,  # its own process group: Stop reaches only it
                    text=True,
                    bufsize=1,
                    errors="replace",
                )
            except OSError as e:
                raise StartRefused(f"could not start {spec.module}: {e}") from e
            run.pid = proc.pid
            self._proc = proc
            self.current = run
            self._done.clear()
        threading.Thread(target=self._read, args=(run, proc), daemon=True).start()
        threading.Thread(target=self._watch_ready, args=(run, spec), daemon=True).start()
        self.on_change()
        return run

    # ---- stop ---------------------------------------------------------------------
    def stop(self, wait: bool = False, timeout: float | None = None) -> DemoRun | None:
        """Ask the demo to stop (one SIGINT); escalate to SIGKILL after stop_timeout_s."""
        with self._lock:
            run, proc = self.current, self._proc
            if run is None or proc is None or not run.active:
                return run
            first = not run.stop_requested
            if first:
                run.stop_requested = True
                run.state, run.detail = "stopping", "asking the demo to rest the robot"
                self._signal(proc, signal.SIGINT)
        if first:
            self.on_change()
            threading.Thread(target=self._escalate, args=(run, proc), daemon=True).start()
        if wait:
            self._done.wait(timeout if timeout is not None else self.stop_timeout_s + 10)
        return run

    def wait(self, timeout: float | None = None) -> bool:
        return self._done.wait(timeout)

    def _signal(self, proc: subprocess.Popen, sig: int) -> None:
        with contextlib.suppress(ProcessLookupError, PermissionError):
            os.killpg(proc.pid, sig)

    def _escalate(self, run: DemoRun, proc: subprocess.Popen) -> None:
        try:
            proc.wait(self.stop_timeout_s)
        except subprocess.TimeoutExpired:
            run.killed = True
            self._add_log(run, f"flydeck: no clean exit after {self.stop_timeout_s:g} s; killing")
            self._signal(proc, signal.SIGKILL)

    # ---- watching -----------------------------------------------------------------
    def _add_log(self, run: DemoRun, line: str) -> None:
        run.log.append(line)
        run.log_seq += 1

    def _read(self, run: DemoRun, proc: subprocess.Popen) -> None:
        assert proc.stdout is not None
        for raw in proc.stdout:
            line = raw.rstrip("\n")
            ev = status.parse(line)
            if ev is None:
                self._add_log(run, line)
                self.on_change()
                continue
            self._on_event(run, ev)
        code = proc.wait()
        self._finish(run, code)

    def _on_event(self, run: DemoRun, ev: dict[str, Any]) -> None:
        kind = ev.get("event")
        with self._lock:
            if kind == "ready":
                run.ready = True
                run.url = ev.get("url") or None
                if run.state in ("starting", "connecting") and not run.stop_requested:
                    run.state, run.detail = "running", ""
            elif kind == "state":
                name, detail = str(ev.get("state", "")), str(ev.get("detail", ""))
                if name == "attention" and detail:
                    run.warnings.append(detail)
                if not run.stop_requested and name:
                    run.state, run.detail = name, detail
            elif kind == "failed":
                run.failed_event = str(ev.get("reason", "")) or None
        self.on_change()

    def _watch_ready(self, run: DemoRun, spec: DemoSpec) -> None:
        deadline = time.monotonic() + spec.ready_timeout_s
        url = f"http://127.0.0.1:{run.http_port}/"
        while run.active and not run.ready and time.monotonic() < deadline:
            if spec.ready == "http":
                try:
                    with urllib.request.urlopen(url, timeout=1.0) as r:
                        if r.status == 200:
                            self._on_event(run, {"event": "ready", "url": url})
                            return
                except OSError:
                    pass
            time.sleep(0.3)
        if run.active and not run.ready and not run.stop_requested:
            run.failed_event = (
                f"it did not report ready within {spec.ready_timeout_s:g} s; stopped it"
            )
            self.stop()

    def _finish(self, run: DemoRun, code: int) -> None:
        with self._lock:
            run.exit_code = code
            run.ended = time.time()
            last = next((ln for ln in reversed(run.log) if ln.strip()), "")
            if run.killed:
                run.state = "failed"
                run.reason = (
                    f"It did not stop within {self.stop_timeout_s:g} s and was killed. "
                    + ROBOT_MAY_STAND
                )
            elif run.stop_requested and run.failed_event is None and code in (0, 130, -2):
                run.state, run.reason = "stopped", "Stopped. The robot is resting."
            elif code == 0 and run.failed_event is None:
                run.state, run.reason = "stopped", "Finished. The robot is resting."
            elif code in (134, -6) and run.transport == "ble" and sys.platform == "darwin":
                run.state, run.reason = "failed", BLE_KILLED
            else:
                run.state = "failed"
                why = run.failed_event or EXIT_CODES.get(code) or f"exit code {code}"
                run.reason = why[:1].upper() + why[1:]
                if run.failed_event is None and last and not last.startswith("$ "):
                    run.reason += f" (last output: {last[-200:]})"
            run.detail = ""
            self._done.set()
        self.on_change()
