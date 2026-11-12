"""Flydeck: pick a robot, pick a fly-brain demo, press Start.

uv run flydeck                      # open Flydeck in the browser (USB and simulator)
uv run flydeck install-app          # make ~/Applications/Flydeck.app (needed for Bluetooth)
uv run flydeck new-demo fly-groom   # start your own demo from the template
uv run flydeck list                 # the demos Flydeck sees, and any problems
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import signal
import sys
import threading
import webbrowser
from pathlib import Path

from . import registry


def _root() -> Path:
    root = registry.find_root()
    if root is None:
        root = registry.find_root(Path.cwd())
    if root is None:
        raise SystemExit("flydeck: run me from inside the flybrain checkout")
    return root


def cmd_run(a: argparse.Namespace) -> int:
    from .hub import DEFAULT_PORT, TERMINAL_BLE_HINT, Hub, HubServer, ble_blocked, watch_parent

    logging.basicConfig(
        level=logging.INFO if a.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    hub = Hub(root=_root(), stop_timeout_s=a.stop_timeout)
    try:
        server = HubServer(hub, port=a.port if a.port is not None else DEFAULT_PORT).start()
    except OSError as e:
        print(f"flydeck: cannot start the page server: {e}", file=sys.stderr)
        return 5

    def on_signal(*_):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        print("\nflydeck: stopping (any running demo rests the robot first)", flush=True)
        threading.Thread(target=hub.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    if a.parent_pid:
        watch_parent(a.parent_pid, hub)

    print(f"Flydeck at {server.url}", flush=True)
    for e in hub.entries:
        print(f"  {'ok ' if e.ok else 'ERR'} {e.id}" + ("" if e.ok else f": {e.error}"))
    if ble_blocked() and not a.app:
        print("flydeck: " + TERMINAL_BLE_HINT, flush=True)
    if a.url_file:
        Path(a.url_file).write_text(server.url)
    if not a.no_open:
        webbrowser.open(server.url)
    try:
        while not hub.quit_event.wait(0.5):
            pass
    finally:
        hub.shutdown()
        server.stop()
        if a.url_file:
            with contextlib.suppress(OSError):
                Path(a.url_file).unlink()
    print("flydeck: bye", flush=True)
    return 0


def cmd_install_app(a: argparse.Namespace) -> int:
    from .bundle import build_app

    root = _root()
    try:
        app = build_app(root, directory=Path(a.dir) if a.dir else None)
    except (RuntimeError, OSError) as e:
        print(f"flydeck: {e}", file=sys.stderr)
        return 1
    print(
        f"Created {app}\n"
        f"  It runs Flydeck from {root} with {sys.executable}.\n"
        "  Open it from Finder or Launchpad (or: open " + str(app).replace(" ", "\\ ") + ").\n"
        "  The first time a robot is searched over Bluetooth, macOS asks whether Flydeck may\n"
        "  use Bluetooth: click Allow. If you move this folder, run install-app again."
    )
    if a.open:
        import subprocess

        subprocess.run(["open", str(app)], check=False)
    return 0


def cmd_new_demo(a: argparse.Namespace) -> int:
    from .scaffold import ScaffoldError, new_demo, next_steps

    root = _root()
    try:
        dest = new_demo(root, a.name, a.title)
    except ScaffoldError as e:
        print(f"flydeck: {e}", file=sys.stderr)
        return 2
    print(next_steps(root, dest))
    return 0


def cmd_list(_a: argparse.Namespace) -> int:
    entries = registry.discover(_root())
    for e in entries:
        if e.ok and e.spec:
            s = e.spec
            moves = "moves the robot" if s.moves_robot else "does not move"
            print(f"{s.id:<16} {s.title}: {s.summary}")
            names = {"mock": "simulator", "usb": "USB", "ble": "Bluetooth"}
            links = ", ".join(names[t] for t in s.transports)
            print(f"{'':<16} {moves}; runs on {links}; from {e.source}")
        else:
            print(f"{e.id:<16} PROBLEM: {e.error}")
        for p in e.problems:
            print(f"{'':<16} note: {p}")
    return 0 if all(e.ok for e in entries) else 1


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="flydeck", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--port", type=int, default=None, help="page port (default 8760, or any free)")
    ap.add_argument("--no-open", action="store_true", help="do not open a browser")
    ap.add_argument("--stop-timeout", type=float, default=20.0, help=argparse.SUPPRESS)
    ap.add_argument("--app", action="store_true", help=argparse.SUPPRESS)  # started by Flydeck.app
    ap.add_argument("--parent-pid", type=int, default=0, help=argparse.SUPPRESS)
    ap.add_argument("--url-file", default=None, help=argparse.SUPPRESS)
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd")
    p = sub.add_parser("install-app", help="create Flydeck.app for this checkout (macOS)")
    p.add_argument("--dir", help="where to put it (default ~/Applications)")
    p.add_argument("--open", action="store_true", help="open it when done")
    p = sub.add_parser("new-demo", help="start a new demo from the hello-robot template")
    p.add_argument("name", help="lowercase words joined by dashes, e.g. fly-groom")
    p.add_argument("--title", help="the card title (default from the name)")
    sub.add_parser("list", help="list the demos Flydeck sees")
    return ap


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    if a.cmd == "install-app":
        return cmd_install_app(a)
    if a.cmd == "new-demo":
        return cmd_new_demo(a)
    if a.cmd == "list":
        return cmd_list(a)
    return cmd_run(a)


if __name__ == "__main__":
    sys.exit(main())
