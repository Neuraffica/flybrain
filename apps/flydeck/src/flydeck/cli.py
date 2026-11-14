"""Flydeck: pick a robot, pick a fly-brain demo, press Start.

uv run flydeck                      # open Flydeck (its own window on a Mac, else the browser)
uv run flydeck --browser            # open it in the web browser instead
uv run flydeck new-demo fly-groom   # start your own demo from the template
uv run flydeck list                 # the demos Flydeck sees, and any problems
uv run flydeck build-app            # build dist/Flydeck.app and its .dmg (Apple Silicon Mac)
uv run flydeck install-app          # build it and put it in ~/Applications
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

from . import registry, sources


def _root(a: argparse.Namespace | None = None) -> Path:
    if a is not None and getattr(a, "root", None):
        return Path(a.root).resolve()
    bundle = sources.bundle_path()
    if bundle is not None:
        return sources.builtin(bundle).root
    root = registry.find_root()
    if root is None:
        root = registry.find_root(Path.cwd())
    if root is None:
        raise SystemExit("flydeck: run me from inside the flybrain checkout")
    return root


def cmd_run(a: argparse.Namespace) -> int:
    if not a.no_open and not a.browser:
        from . import app

        if app.window_available():
            return cmd_window(a)
    return cmd_serve(a)


def cmd_window(a: argparse.Namespace) -> int:
    """The Flydeck window; the hub runs as its child process."""
    from . import app
    from .settings import Settings

    bundle = sources.bundle_path()
    if bundle is not None:
        log = sources.log_path()
        log.parent.mkdir(parents=True, exist_ok=True)
        logging.basicConfig(
            filename=log,
            level=logging.INFO,
            format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        )
        source, note = sources.choose(Settings(), bundle)
    else:
        logging.basicConfig(
            level=logging.INFO if a.verbose else logging.WARNING,
            format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        )
        source, note = sources.dev(_root(a), sys.executable), None
    return app.run(source, bundle, note)


def cmd_serve(a: argparse.Namespace) -> int:
    """The hub in this process; the page opens in the browser unless --no-open."""
    from .hub import DEFAULT_PORT, TERMINAL_BLE_HINT, Hub, HubServer, ble_blocked, watch_parent

    logging.basicConfig(
        level=logging.INFO if a.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    root = _root(a)
    workdir = Path(a.workdir).expanduser() if a.workdir else root
    workdir.mkdir(parents=True, exist_ok=True)
    hub = Hub(root=root, workdir=workdir, stop_timeout_s=a.stop_timeout)
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


def cmd_build_app(a: argparse.Namespace) -> int:
    from .build_app import BuildError, build, install

    try:
        if a.cmd == "install-app":
            app = install(_root(), Path(a.dir).expanduser() if a.dir else None)
        else:
            app, dmg = build(_root(), Path(a.out).resolve() if a.out else None, dmg=not a.no_dmg)
            if dmg is not None:
                print(f"Built {dmg}")
    except BuildError as e:
        print(f"flydeck: {e}", file=sys.stderr)
        return 1
    print(f"Built {app}")
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
    ap.add_argument("--browser", action="store_true", help="open in the web browser, not a window")
    ap.add_argument("--no-open", action="store_true", help="only serve the page; open nothing")
    ap.add_argument("--stop-timeout", type=float, default=20.0, help=argparse.SUPPRESS)
    ap.add_argument("--app", action="store_true", help=argparse.SUPPRESS)  # hub of the window
    ap.add_argument("--parent-pid", type=int, default=0, help=argparse.SUPPRESS)
    ap.add_argument("--url-file", default=None, help=argparse.SUPPRESS)
    ap.add_argument("--root", default=None, help=argparse.SUPPRESS)  # Flydeck.app's demos
    ap.add_argument("--workdir", default=None, help=argparse.SUPPRESS)  # where demos run
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd")
    p = sub.add_parser("new-demo", help="start a new demo from the hello-robot template")
    p.add_argument("name", help="lowercase words joined by dashes, e.g. fly-groom")
    p.add_argument("--title", help="the card title (default from the name)")
    sub.add_parser("list", help="list the demos Flydeck sees")
    p = sub.add_parser("build-app", help="build dist/Flydeck.app and its .dmg (Apple Silicon)")
    p.add_argument("--out", help="output folder (default: dist in the checkout)")
    p.add_argument("--no-dmg", action="store_true", help="only the .app")
    p.add_argument("--open", action="store_true", help="open the app when done")
    p = sub.add_parser("install-app", help="build Flydeck.app and put it in ~/Applications")
    p.add_argument("--dir", help="where to put it (default ~/Applications)")
    p.add_argument("--open", action="store_true", help="open it when done")
    return ap


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    if a.cmd in ("build-app", "install-app"):
        return cmd_build_app(a)
    if a.cmd == "new-demo":
        return cmd_new_demo(a)
    if a.cmd == "list":
        return cmd_list(a)
    return cmd_run(a)


if __name__ == "__main__":
    sys.exit(main())
