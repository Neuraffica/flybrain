"""Where the Flydeck window gets its demos, and the hub process that serves them.

The window process runs exactly one hub (``python -m flydeck --app --no-open``) as a child:

    built-in  the demos inside Flydeck.app, run with the app's own Python
    folder    a flybrain checkout the user chose: the hub and every demo run with that folder's
              .venv Python, from that folder, so demos made with ``flydeck new-demo`` appear
    dev       ``uv run flydeck`` in a checkout: that checkout and the Python running it

Everything the hub starts descends from the app, so it shares the app's Bluetooth permission.
The hub watches the window process (--parent-pid): if the app is force-quit, the hub still stops
the running demo, which rests the robot.
"""

from __future__ import annotations

import contextlib
import json
import os
import shutil
import signal
import subprocess
import tempfile
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from flybrain_demo_api import user_dir

from . import registry
from .settings import Settings

SETTING = "checkout"  # the chosen flybrain folder, in flydeck.json; absent: built-in demos


class SourceError(Exception):
    """A folder cannot be used, or the hub did not start; the message says why, plainly."""


@dataclass(frozen=True)
class Source:
    kind: str  # "builtin" | "folder" | "dev"
    root: Path  # the flybrain folder (built-in: Flydeck.app/Contents/Resources/flybrain)
    python: str  # runs the hub and the demos
    workdir: Path  # where demos run and write runs/

    def hub_command(self, url_file: Path, parent_pid: int) -> list[str]:
        cmd = [self.python, "-m", "flydeck", "--app", "--no-open"]
        cmd += ["--parent-pid", str(parent_pid), "--url-file", str(url_file)]
        if self.kind == "builtin":
            cmd += ["--root", str(self.root), "--workdir", str(self.workdir)]
        return cmd

    def to_json(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "root": str(self.root),
            "name": self.root.name,
            "workdir": str(self.workdir),
        }


def bundle_path() -> Path | None:
    """Flydeck.app when this process runs inside it (its launcher sets FLYDECK_BUNDLE)."""
    p = os.environ.get("FLYDECK_BUNDLE")
    return Path(p) if p and (Path(p) / "Contents" / "Info.plist").is_file() else None


def builtin(bundle: Path) -> Source:
    res = bundle / "Contents" / "Resources"
    return Source(
        kind="builtin",
        root=res / "flybrain",
        python=str(res / "python" / "bin" / "python3"),
        workdir=user_dir(),
    )


def dev(root: Path, python: str) -> Source:
    return Source(kind="dev", root=root, python=python, workdir=root)


def venv_python(root: Path) -> Path:
    return root / ".venv" / "bin" / "python"


def check_folder(folder: str | Path, probe: bool = True, timeout: float = 90.0) -> Source:
    """A flybrain checkout (or a folder inside one) with a working .venv, as a Source."""
    folder = Path(folder).expanduser()
    if not folder.is_dir():
        raise SourceError(f"{folder} is not there any more.")
    root = registry.find_root(folder)
    if root is None or not (root / "apps" / "flydeck").is_dir():
        raise SourceError(
            f"{folder} is not a flybrain folder. Choose the folder you cloned from GitHub: "
            "it has demos and apps/flydeck in it."
        )
    py = venv_python(root)  # not resolved: the venv is found through this path
    if not os.access(py, os.X_OK):
        raise SourceError(
            f"{root} has no Python environment yet. In Terminal, in that folder, run: uv sync"
        )
    if probe:
        try:
            r = subprocess.run(
                [str(py), "-c", "import flydeck.hub"],
                cwd=root,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as e:
            raise SourceError(f"The Python environment in {root} does not run: {e}") from e
        if r.returncode != 0:
            last = (r.stderr.strip().splitlines() or ["no output"])[-1]
            raise SourceError(
                f"The Python environment in {root} cannot run Flydeck ({last}). In Terminal, "
                "in that folder, run: uv sync"
            )
    return Source(kind="folder", root=root, python=str(py), workdir=root)


def choose(settings: Settings, bundle: Path) -> tuple[Source, str | None]:
    """The saved folder if it is still usable, else the built-in demos and a note why."""
    saved = settings.get(SETTING)
    if saved:
        try:
            return check_folder(saved, probe=False), None
        except SourceError as e:
            return builtin(bundle), f"Showing the built-in demos. {e}"
    return builtin(bundle), None


def log_path() -> Path:
    return Path.home() / "Library" / "Logs" / "Flydeck.log"


class HubProcess:
    """One hub child: start it, learn its URL, stop it (any demo rests first)."""

    def __init__(
        self,
        source: Source,
        parent_pid: int | None = None,
        log: Path | None = None,
        ble_allowed: bool = False,
    ):
        self.source = source
        self.parent_pid = parent_pid or os.getpid()
        self.log = log  # None: the hub writes to this process's stdout/stderr
        self.ble_allowed = ble_allowed
        self.proc: subprocess.Popen | None = None
        self.url: str | None = None
        self._state = Path(tempfile.mkdtemp(prefix="flydeck-hub-"))

    @property
    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def env(self) -> dict[str, str]:
        env = dict(os.environ, FLYDECK_APP="1", PYTHONUNBUFFERED="1")
        if self.ble_allowed:
            env["FLYBRAIN_BLE_BUNDLE"] = "1"  # running inside Flydeck.app: Bluetooth is allowed
        else:
            env.pop("FLYBRAIN_BLE_BUNDLE", None)
        return env

    def start(self, timeout: float = 90.0) -> str:
        url_file = self._state / "url"
        url_file.unlink(missing_ok=True)
        self.source.workdir.mkdir(parents=True, exist_ok=True)
        cmd = self.source.hub_command(url_file, self.parent_pid)
        out: Any = None
        offset = 0
        if self.log is not None:
            self.log.parent.mkdir(parents=True, exist_ok=True)
            out = open(self.log, "ab")  # noqa: SIM115 - the child keeps writing to it
            offset = out.tell()
            stamp = time.strftime("%Y-%m-%d %H:%M:%S")
            out.write(f"--- Flydeck hub starting {stamp}: {' '.join(cmd)}\n".encode())
            out.flush()
        try:
            self.proc = subprocess.Popen(
                cmd,
                cwd=self.source.workdir,
                env=self.env(),
                stdin=subprocess.DEVNULL,
                stdout=out,
                stderr=subprocess.STDOUT if out is not None else None,
                start_new_session=True,  # Ctrl-C in a terminal reaches the window, which stops us
            )
        except OSError as e:
            raise SourceError(f"cannot start {cmd[0]}: {e}") from e
        finally:
            if out is not None:
                out.close()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if url_file.is_file() and url_file.read_text().strip():
                self.url = url_file.read_text().strip()
                return self.url
            if self.proc.poll() is not None:
                raise SourceError(
                    f"Flydeck stopped while starting (exit {self.proc.returncode})."
                    + self._tail(offset)
                )
            time.sleep(0.1)
        self.stop(timeout=5)
        raise SourceError(f"Flydeck did not start within {timeout:g} s." + self._tail(offset))

    def _tail(self, offset: int, lines: int = 8) -> str:
        if self.log is None:
            return ""
        try:
            with open(self.log, "rb") as f:
                f.seek(offset)
                text = f.read().decode(errors="replace")
        except OSError:
            return ""
        tail = [ln for ln in text.splitlines() if ln.strip()][-lines:]
        return ("\n" + "\n".join(tail)) if tail else ""

    def state(self) -> dict[str, Any] | None:
        if not self.url:
            return None
        try:
            with urllib.request.urlopen(self.url + "api/state", timeout=3) as r:
                return json.loads(r.read())
        except (OSError, ValueError):
            return None

    def stop(self, timeout: float = 45.0) -> int | None:
        """Ask the hub to quit (it stops a running demo first, which rests the robot)."""
        proc = self.proc
        if proc is None:
            return None
        if proc.poll() is None:
            asked = False
            if self.url:
                req = urllib.request.Request(
                    self.url + "api/quit",
                    data=b"{}",
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with contextlib.suppress(OSError):
                    urllib.request.urlopen(req, timeout=5).read()
                    asked = True
            if not asked:
                with contextlib.suppress(ProcessLookupError):
                    proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        shutil.rmtree(self._state, ignore_errors=True)
        return proc.returncode
