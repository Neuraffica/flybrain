"""fly-escape: a fly looming-escape circuit makes a Bittle back away from an approaching hand.

uv run fly-escape --mock --open                       # the visual simulator, escapes on
uv run fly-escape --mock --mock-random 7 --no-dashboard --duration 120   # scores, no browser
uv run fly-escape --port /dev/cu.usbmodem1101         # real dog over USB, observe
uv run fly-escape --port ... --i-have-space           # real dog, escapes enabled
uv run flybrain-bittle ble-run -- uv run fly-escape --ble   # Bluetooth on macOS
uv run fly-escape --replay packages/flybrain-models/tests/fixtures/ir_hand_approaches_usb.json

An infrared proximity sensor on the Grove socket labelled 39/36 stands in for the fly's eye.
"""

from __future__ import annotations

import argparse
import logging
import random
import signal
import sys
import time
import webbrowser
from importlib import resources

from fly_compass.benchmark import find_benchmark
from fly_compass.server import DashboardServer, FrameHub
from flybrain_demo_api import status
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport, TransportError, open_transport
from flybrain_edge.bittle.robot import BittleError

from .app import DEFAULT_PINS, MODES, EscapeApp
from .escape import ACTIONS, MAX_PER_MINUTE, MIN_INTERVAL_S, EscapeConfig
from .groundtruth import format_table
from .loop import EscapeLoop
from .models import MODEL_KEYS, ModelMissing, load_models
from .scripts import SCRIPTS
from .session import ReplaySource, SessionLog, load_samples
from .sides import SIDE_SETTINGS, resolve, robot_identity
from .sim import Simulator

BENCHMARK_CANDIDATES = [("flybrain_models", "escape_benchmark.json")]

SAFETY = """\
  Put the dog on the floor with about a metre clear behind it (it walks backwards to escape).
  Keep hands away from the sensor for the first 2 s while its resting level is measured.
  Ctrl-C at any time stops motion and rests the robot."""


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="fly-escape",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--mock", action="store_true", help="simulated robot and sensor")
    src.add_argument("--port", metavar="PATH", help="USB serial device (default: auto-detect)")
    src.add_argument(
        "--ble",
        nargs="?",
        const="",
        default=None,
        metavar="NAME",
        help="Bluetooth LE; optional name substring",
    )
    src.add_argument(
        "--replay",
        metavar="RUN",
        help="a runs/escape-<stamp> dir, its samples.csv, or a [t, raw_a, raw_b] JSON",
    )
    ap.add_argument(
        "--mode",
        choices=MODES,
        default=None,
        help="observe (no motion) or escape; default escape with --i-have-space, otherwise observe",
    )
    ap.add_argument(
        "--i-have-space",
        action="store_true",
        help="allow escape mode, which makes the robot walk backwards",
    )
    ap.add_argument(
        "--driver",
        choices=MODEL_KEYS,
        default="connectome",
        help="model whose trigger moves the robot (switchable in the dashboard)",
    )
    ap.add_argument(
        "--pins",
        default=",".join(map(str, DEFAULT_PINS)),
        help="the two analog pins of the sensor, left first (default 36,39)",
    )
    sides = ap.add_mutually_exclusive_group()
    sides.add_argument(
        "--sides",
        choices=SIDE_SETTINGS,
        default="auto",
        help="which pin is the robot's left. auto (default): what --calibrate-sides found for "
        "this robot, else the first pin; normal: the first pin; swapped: the second pin",
    )
    sides.add_argument(
        "--swap-sides",
        dest="sides",
        action="store_const",
        const="swapped",
        help="same as --sides swapped: the second pin is the robot's left",
    )
    sides.add_argument(
        "--no-swap-sides",
        dest="sides",
        action="store_const",
        const="normal",
        help="same as --sides normal: the first pin is the robot's left",
    )
    ap.add_argument(
        "--calibrate-sides",
        action="store_true",
        help="after the resting level, ask for a hand on the robot's left to find which pin is "
        "the left, and remember it for this robot (also a dashboard button)",
    )
    ap.add_argument(
        "--escape-action",
        choices=ACTIONS,
        default="back",
        help="back: walk straight back (kbkF); back-turn: walk back turning away from "
        "the threat (kbkL/kbkR)",
    )
    ap.add_argument(
        "--turn-sign",
        type=int,
        choices=(1, -1),
        default=1,
        help="flip which way back-turn turns (UNVERIFIED on hardware)",
    )
    ap.add_argument(
        "--action-seconds",
        type=float,
        default=1.0,
        help="how long the escape gait runs before the stand (default 1)",
    )
    ap.add_argument(
        "--min-interval",
        type=float,
        default=MIN_INTERVAL_S,
        help=f"seconds between escapes at least (default {MIN_INTERVAL_S:.0f})",
    )
    ap.add_argument(
        "--max-per-minute",
        type=int,
        default=MAX_PER_MINUTE,
        help=f"escapes per minute at most (default {MAX_PER_MINUTE})",
    )
    ap.add_argument(
        "--imu",
        action="store_true",
        help="also poll the IMU (off by default to keep the sensor rate high)",
    )
    ap.add_argument("--analog-rate", type=float, default=40.0, help="sensor reads per second, max")
    ap.add_argument("--yaw-sign", type=int, choices=(1, -1), default=None)
    ap.add_argument("--rate", type=float, default=20.0, help="dashboard/control rate in Hz")
    ap.add_argument("--http-port", type=int, default=8766)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--no-dashboard", action="store_true")
    ap.add_argument("--open", action="store_true", help="open the dashboard in a browser")
    ap.add_argument(
        "--no-open", action="store_true", help="never open a browser (the default; for Flydeck)"
    )
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--runs-dir", default="runs")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds")
    ap.add_argument("--replay-speed", type=float, default=1.0, help="0 = as fast as possible")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--benchmark", metavar="JSON", help="offline benchmark file to show")
    ap.add_argument(
        "--mock-script",
        choices=sorted(SCRIPTS),
        default=None,
        help="move the simulated hand by itself (cycle: every script in turn)",
    )
    ap.add_argument(
        "--mock-random",
        nargs="?",
        type=int,
        const=-1,
        default=None,
        metavar="SEED",
        help="simulator: keep launching random approaches (threats and non-threats) from this "
        "seed; without a value a seed is picked and printed",
    )
    ap.add_argument(
        "--sim-speed", type=float, default=1.0, help="simulator: approach speed factor (0.25-4)"
    )
    ap.add_argument("--boot-timeout", type=float, default=12.0)
    ap.add_argument("-v", "--verbose", action="store_true")
    return ap


def print_summary(summary: dict, driver: str) -> None:
    print(f"sensor samples: {summary['samples']}  |  {summary['calibration']['message']}")
    print(
        f"escapes: {summary['escapes']} (ignored triggers: {summary['blocked']}), "
        f"events: {summary['events']}"
    )
    print(
        f"  {'model':<11} {'triggers':>8} {'false':>6} {'events':>7} {'missed':>7} "
        f"{'dt vs ' + driver:>16}"
    )
    for k, v in summary["models"].items():
        dtv = "-" if v["mean_dt_vs_driver_ms"] is None else f"{v['mean_dt_vs_driver_ms']:+d} ms"
        print(
            f"  {k:<11} {v['triggers']:>8} {v['false_alarms']:>6} {v['events']:>7} "
            f"{v['missed']:>7} {dtv:>16}"
        )


_SOURCES = {
    "calibrated": "calibrated for this robot",
    "flag": "set by flag",
    "default": "default, not calibrated",
}


def _fail(code: int, message: str) -> int:
    """Print why the demo cannot run, tell Flydeck (when it started us), return the exit code."""
    print(message, file=sys.stderr)
    status.failed(message.removeprefix("fly-escape: "))
    return code


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO if a.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    # The simulator moves no real legs: escape needs no --i-have-space with --mock.
    allow_moves = a.i_have_space or a.mock
    mode = a.mode or ("escape" if allow_moves and not a.replay else "observe")
    if mode == "escape" and not allow_moves:
        return _fail(
            2,
            "escape mode makes the robot walk backwards. Clear a metre behind it and add "
            "--i-have-space.",
        )
    if a.replay and mode == "escape":
        return _fail(2, "fly-escape: a replay cannot move a robot; use --mode observe")
    if (a.mock_script or a.mock_random is not None) and not a.mock:
        return _fail(2, "fly-escape: --mock-script and --mock-random need --mock")
    if a.replay and a.calibrate_sides:
        return _fail(2, "fly-escape: --calibrate-sides needs a robot (or --mock), not a replay")
    if a.mock_script and a.mock_random is not None:
        return _fail(2, "fly-escape: choose --mock-script or --mock-random, not both")
    if a.mock_random == -1:
        a.mock_random = random.randrange(1, 10_000)
    try:
        pins = tuple(int(x) for x in a.pins.replace(" ", "").split(","))
        if len(pins) != 2:
            raise ValueError("give exactly two pins")
    except ValueError as e:
        return _fail(2, f"fly-escape: --pins: {e}")
    if a.yaw_sign is None:
        a.yaw_sign = 1 if (a.mock or a.replay) else -1

    try:
        models, raw_to_proximity = load_models(seed=a.seed)
    except ModelMissing as e:
        return _fail(3, f"fly-escape: {e}")
    loop = EscapeLoop(models, raw_to_proximity, driver=a.driver)
    cfg = EscapeConfig(
        action=a.escape_action,
        action_s=a.action_seconds,
        min_interval_s=a.min_interval,
        max_per_minute=a.max_per_minute,
        turn_sign=a.turn_sign,
    )

    # Only the FIRST Ctrl-C / SIGTERM interrupts; later ones must not abort the cleanup.
    def _stop(*_):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt

    prev = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    bittle = mock = replay = None
    if a.replay:
        try:
            rows = load_samples(a.replay)
        except (FileNotFoundError, ValueError) as e:
            return _fail(2, f"fly-escape: {e}")
        replay = ReplaySource(rows)
        label = f"replay {a.replay} ({len(rows)} samples, {replay.duration:.1f}s)"
    else:
        if a.mock:
            mock = MockBittle(seed=a.seed)
            transport = MockTransport(mock)
            label = "mock"
        else:
            transport = open_transport(port=a.port, ble=a.ble)
            label = transport.name
            status.state("connecting", label)
            print("fly-escape: real robot.\n" + SAFETY)
            if mode == "escape":
                print("  ESCAPE MODE: the legs will move. Keep cables clear.")
            print("  Starting in 3 s ...", flush=True)
            try:
                time.sleep(3)
            except KeyboardInterrupt:
                return 130
        bittle = Bittle(transport, yaw_sign=a.yaw_sign)
        try:
            bittle.connect(boot_timeout=0.5 if a.mock else a.boot_timeout)
        except (TransportError, BittleError) as e:
            _fail(4, f"fly-escape: cannot connect: {e}")
            try:
                transport.close()
            except Exception:  # noqa: BLE001, S110 - best-effort close after a failed connect
                pass
            return 4
        label = f"{label}: {transport.description}"
        loop.robot = "simulator" if a.mock else robot_identity(transport, bittle.boot_text)
    loop.swap_sides, loop.sides_source = resolve(a.sides, loop.robot, pins)

    session = None
    if not a.no_log and replay is None:
        session = SessionLog(
            a.runs_dir,
            meta={
                "argv": sys.argv,
                "source": label,
                "pins": pins,
                "swap_sides": loop.swap_sides,
                "sides": a.sides,
                "sides_source": loop.sides_source,
                "robot": loop.robot,
                "firmware": bittle and " ".join(x for x in (bittle.model, bittle.version) if x),
                "escape": vars(cfg),
                "note": "raw_a / raw_b are the readings of --pins in order; left is raw_a unless "
                "swap_sides (as at the start; a side check during the run can change it)",
            },
        )

    hub = FrameHub()
    fast = replay is not None and a.replay_speed <= 0
    app = EscapeApp(
        loop,
        hub,
        bittle=bittle,
        mock=mock,
        replay=replay,
        replay_speed=a.replay_speed,
        session=session,
        pins=pins,
        escape_cfg=cfg,
        mode=mode,
        allow_moves=allow_moves,
        source_label=label,
        exit_after_replay=a.no_dashboard or fast,
        rate_hz=a.rate,
    )
    if mock is not None:
        app.sim = Simulator(mock, seed=a.mock_random, speed=a.sim_speed)
    app.imu = a.imu
    app.calibrate_sides = a.calibrate_sides
    app.analog_rate_hz = a.analog_rate

    def _on_event(ev: str) -> None:
        print(f"fly-escape: {ev}", flush=True)
        if ev.startswith("calibrating"):
            status.state("calibrating", "keep hands away from the sensor")
        elif ev.startswith("side check:"):
            status.state("calibrating", ev.removeprefix("side check: "))
        elif ev.startswith(("calibration FAILED", "ESCAPE MODE REFUSED", "THE ROBOT WILL NOT")):
            status.state("attention", ev)
        elif ev.startswith(("calibrat", "sides calibrated", "side check FAILED")):
            status.state("running", ev)

    app.on_event = _on_event
    server = None
    if not a.no_dashboard and not fast:
        page = resources.files("fly_escape").joinpath("static/index.html").read_bytes()
        try:
            server = DashboardServer(
                hub,
                app.on_command,
                lambda: find_benchmark(a.benchmark, BENCHMARK_CANDIDATES),
                host=a.host,
                port=a.http_port,
                page=page,
            ).start()
        except OSError as e:
            app.shutdown()
            return _fail(
                5, f"fly-escape: dashboard port {a.http_port} unavailable ({e}); use --http-port"
            )
        print(f"fly-escape: dashboard at {server.url}", flush=True)
        if a.open and not a.no_open:
            webbrowser.open(server.url)
    print(
        f"fly-escape: {label} | mode {mode} (after calibration) | driver {loop.driver} | "
        f"left pin {pins[1] if loop.swap_sides else pins[0]} ({_SOURCES[loop.sides_source]})"
        + (f" | logging to {session.dir}" if session else ""),
        flush=True,
    )
    if not a.calibrate_sides:
        app.warn_if_uncalibrated()
    if a.mock_random is not None:
        print(f"fly-escape: random approaches, seed {a.mock_random}", flush=True)
    if mock is not None and a.mock_script:
        mock.hand.run_script(SCRIPTS[a.mock_script], mock.clock() + 3.0)  # after calibration

    code = 0
    try:
        status.ready(server.url if server is not None else None)
        app.start_robot()
        summary = app.run(duration=a.duration)
    except KeyboardInterrupt:
        summary = app.summary()
        print("\nfly-escape: stopping (Ctrl-C)")
    except Exception as e:  # noqa: BLE001 - any error must still rest the robot
        summary = app.summary()
        print(f"fly-escape: error: {e!r}; resting the robot", file=sys.stderr)
        status.failed(f"error: {e}")
        code = 1
    finally:
        app.shutdown()
        if server is not None:
            server.stop()
    print_summary(summary, loop.driver)
    gt = summary.get("ground_truth")
    if gt and gt["trials"]:
        print(format_table(gt, loop.driver))
    if session:
        print(f"session log: {session.dir}")
    signal.signal(signal.SIGINT, prev[0])
    signal.signal(signal.SIGTERM, prev[1])
    return code


if __name__ == "__main__":
    sys.exit(main())
