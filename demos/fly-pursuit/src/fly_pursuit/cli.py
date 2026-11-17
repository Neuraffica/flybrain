"""fly-pursuit: a circuit of 265 neurons from the fly connectome turns a simulated robot dog
toward a target. This version runs in the simulator only.

uv run fly-pursuit --mock --open                  # simulator, dashboard in the browser
uv run fly-pursuit --mock --controller no-aotu019 --fov 30 --target jumps
uv run fly-pursuit --mock --no-open --http-port 8799

The robot version needs the camera (frame rate, delay, field of view) measured first, so
--port and --ble are refused for now.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import webbrowser
from importlib import resources

from flybrain_demo_api import status
from flybrain_demo_api.web import DashboardServer, FrameHub

from .flydeck import CONTROLLERS, FOVS, MOTIONS, SPEC

NOT_YET = (
    "this version runs only in the simulator: start it with --mock. The robot version needs "
    "the camera's frame rate, delay and field of view measured first."
)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="fly-pursuit",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # The Flydeck contract: --mock, --port, --ble. Not a mutually exclusive group, so that a
    # refused combination gets the plain message below instead of an argparse error.
    ap.add_argument("--mock", action="store_true", help="simulator (the only choice for now)")
    ap.add_argument("--port", metavar="PATH", help="USB serial device (not supported yet)")
    ap.add_argument(
        "--ble", nargs="?", const="", default=None, metavar="NAME", help="(not supported yet)"
    )
    ap.add_argument(
        "--http-port", type=int, default=SPEC.http_port, help="dashboard port (0 = any free)"
    )
    ap.add_argument("--host", default="127.0.0.1", help="dashboard bind address (keep it local)")
    ap.add_argument("--open", action="store_true", help="open the dashboard in a browser")
    ap.add_argument("--no-open", action="store_true", help="never open a browser (for Flydeck)")
    ap.add_argument(
        "--controller", choices=CONTROLLERS, default="connectome", help="switchable live"
    )
    ap.add_argument(
        "--fov",
        type=int,
        choices=FOVS,
        default=30,
        help="camera half field of view in degrees (100 = camera on a panning head)",
    )
    ap.add_argument("--target", choices=MOTIONS, default="still", help="target motion")
    ap.add_argument("--arousal", type=float, default=1.0, help="0..1 (circuit controllers)")
    ap.add_argument("--seed", type=int, default=0, help="seed for camera noise and motions")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds")
    return ap


def fail(code: int, message: str) -> int:
    """Say why we cannot run (terminal and Flydeck) and return the exit code."""
    print(f"fly-pursuit: {message}", file=sys.stderr)
    status.failed(message)
    return code


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    if not a.mock or a.port is not None or a.ble is not None:
        return fail(2, NOT_YET)
    if not 0.0 <= a.arousal <= 1.0:
        return fail(2, "--arousal must be between 0 and 1")
    if a.duration is not None and a.duration <= 0:
        return fail(2, "--duration must be positive")

    # The first Ctrl-C (or Flydeck's Stop) asks the loop to finish; later ones are ignored.
    stop = threading.Event()

    def on_signal(*_):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stop.set()

    previous = signal.signal(signal.SIGINT, on_signal), signal.signal(signal.SIGTERM, on_signal)
    try:
        return _run(a, stop)
    finally:
        signal.signal(signal.SIGINT, previous[0])
        signal.signal(signal.SIGTERM, previous[1])


def _run(a: argparse.Namespace, stop: threading.Event) -> int:
    status.state("starting", "loading the circuit")
    try:  # imported here so that a missing data file gets a plain message and exit code 3
        from .app import PursuitApp
        from .controllers import benchmark_summary
        from .sim import PursuitSim

        sim = PursuitSim(
            controller=a.controller,
            fov_deg=a.fov,
            motion=a.target,
            arousal=a.arousal,
            seed=a.seed,
        )
        hub = FrameHub()
        app = PursuitApp(sim, hub)
        bench = benchmark_summary()  # shown on the page; None (no numbers) if missing
    except (FileNotFoundError, KeyError) as e:
        return fail(3, f"model or data file missing ({e}); run uv sync")

    page = resources.files("fly_pursuit").joinpath("static/index.html").read_bytes()
    try:
        server = DashboardServer(
            hub, app.on_command, lambda: bench, host=a.host, port=a.http_port, page=page
        ).start()
    except OSError as e:
        return fail(5, f"dashboard port {a.http_port} unavailable ({e}); use --http-port")
    print(f"fly-pursuit: dashboard at {server.url}", flush=True)
    print(
        f"fly-pursuit: simulator | controller {a.controller} | field of view +-{a.fov} deg | "
        f"target {a.target}",
        flush=True,
    )
    if a.open and not a.no_open:
        webbrowser.open(server.url)

    code = 0
    try:
        status.ready(server.url)
        status.state("running", "simulator")
        app.run(stop, duration=a.duration)
    except Exception as e:  # noqa: BLE001 - report any error in plain words, then exit 1
        print(f"fly-pursuit: error: {e!r}", file=sys.stderr)
        status.failed(f"error: {e}")
        code = 1
    finally:
        server.stop()
    st = sim.stats()
    if st["mae_deg"] is not None:
        print(
            f"fly-pursuit: stopped after {sim.t:.0f} s. Since the last reset: mean |bearing "
            f"error| {st['mae_deg']:.1f} deg, within 10 deg {100 * st['within10']:.0f} % of "
            f"the time, in view {100 * st['in_view']:.0f} %."
        )
    else:
        print(f"fly-pursuit: stopped after {sim.t:.0f} s.")
    return code


if __name__ == "__main__":
    sys.exit(main())
