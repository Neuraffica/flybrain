"""fly-compass: a fruit-fly head-direction circuit keeps a Bittle's head pointing one way.

uv run fly-compass --mock                      # simulated dog + dashboard
uv run fly-compass --port /dev/cu.usbmodem1101 # real dog over USB (needle mode)
uv run fly-compass --ble                       # real dog over Bluetooth LE
uv run fly-compass --replay runs/20261111-101500
uv run fly-compass --port ... --mode hold --i-have-space
uv run fly-compass --mock --mode sweep --i-have-space   # the dog turns itself +-35 deg
uv run fly-compass --mock --mock-script memory      # cue on, then 20 s blindfolded and still
uv run fly-compass --mock --mock-script dark-turn   # blindfolded 90 deg turn
uv run fly-compass --port ... --blindfold-after 30

Cue: on every fresh IMU sample the measured yaw is also given to the models as a landmark cue.
Blindfold (dashboard button, key B, --blindfold-after) withholds it.
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
import time
import webbrowser

from flybrain_demo_api import status
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport, TransportError, open_transport
from flybrain_edge.bittle import protocol as P
from flybrain_edge.bittle.robot import BittleError

from .app import CompassApp
from .benchmark import find_benchmark
from .loop import LEG_MODES, MODES, CompassLoop, LoopConfig
from .models import MODEL_KEYS, ModelMissing, load_models
from .scenarios import SCENARIOS, SCRIPT_CHOICES
from .server import DashboardServer, FrameHub
from .session import ReplaySource, SessionLog, load_samples

SAFETY = """\
  Put the dog on a flat, clear surface (or on its stand), battery ON, arm free to swing
  +-{lim:.0f} deg left and right. Ctrl-C at any time stops motion and rests the robot."""


def sweep_trajectory(t: float) -> float:
    """Unattended mock demo: slow swings plus an occasional full turn (world yaw, degrees)."""
    base = 37.0 + 70.0 * math.sin(2 * math.pi * t / 14.0)
    turns = 360.0 * max(0.0, min(1.0, (t % 40.0 - 30.0) / 6.0))
    return base + turns


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="fly-compass",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--mock", action="store_true", help="simulated robot (no hardware)")
    src.add_argument("--port", metavar="PATH", help="USB serial device (default: auto-detect)")
    src.add_argument(
        "--ble",
        nargs="?",
        const="",
        default=None,
        metavar="NAME",
        help="Bluetooth LE; optional name substring (default: first Bittle*_BLE)",
    )
    src.add_argument(
        "--replay", metavar="RUN", help="feed a recorded runs/<stamp> (or samples.csv) back"
    )
    ap.add_argument(
        "--mode",
        choices=MODES,
        default="needle",
        help="needle (default), hold, sweep (the dog turns itself back and forth), observe",
    )
    ap.add_argument(
        "--i-have-space",
        action="store_true",
        help="allow hold and sweep modes, which make the robot step and turn in place",
    )
    sw = ap.add_argument_group("sweep mode")
    sw.add_argument(
        "--sweep-angle",
        type=float,
        default=35.0,
        metavar="DEG",
        help="turn to +DEG and -DEG of the start heading (default 35)",
    )
    sw.add_argument(
        "--sweep-pause",
        type=float,
        default=3.0,
        metavar="S",
        help="stand still this long at each end (default 3)",
    )
    sw.add_argument(
        "--sweep-timeout",
        type=float,
        default=25.0,
        metavar="S",
        help="give up on a turn after this long and turn back (default 25)",
    )
    sw.add_argument(
        "--sweep-cycles",
        type=int,
        default=None,
        metavar="N",
        help="stop after N left+right cycles (default: until Ctrl-C/--duration)",
    )
    ap.add_argument(
        "--rate", type=float, default=20.0, help="control/model rate in Hz (default 20)"
    )
    ap.add_argument(
        "--driver",
        choices=MODEL_KEYS,
        default="connectome",
        help="model whose heading drives the needle (switchable in the dashboard)",
    )
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--host", default="127.0.0.1", help="dashboard bind address (keep it local)")
    ap.add_argument("--no-dashboard", action="store_true")
    ap.add_argument("--open", action="store_true", help="open the dashboard in a browser")
    ap.add_argument(
        "--no-open", action="store_true", help="never open a browser (the default; for Flydeck)"
    )
    ap.add_argument("--no-log", action="store_true", help="do not write runs/<stamp>/")
    ap.add_argument("--runs-dir", default="runs")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds")
    ap.add_argument("--replay-speed", type=float, default=1.0, help="0 = as fast as possible")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--benchmark", metavar="JSON", help="offline benchmark file to show")
    hw = ap.add_argument_group("hardware calibration (UNVERIFIED defaults; see README)")
    hw.add_argument(
        "--yaw-sign",
        type=int,
        choices=(1, -1),
        default=None,
        help="+1 if the firmware yaw grows when the dog turns left (CCW). Default -1 on a real "
        "robot (measured on firmware B10_250330: stepping left lowers the yaw), +1 on the "
        "simulator",
    )
    hw.add_argument(
        "--pan-sign",
        type=int,
        choices=(1, -1),
        default=P.DEFAULT_PAN_SIGN,
        help="+1 if a positive head-pan angle looks left",
    )
    hw.add_argument(
        "--pan-limit",
        type=float,
        default=P.DEFAULT_PAN_LIMIT_DEG,
        help="host-side head pan limit, degrees (firmware allows 120)",
    )
    hw.add_argument("--pan-rate", type=float, default=120.0, help="max head pan speed, deg/s")
    hw.add_argument(
        "--joint-token",
        choices=("i", "I"),
        default="i",
        help="'i' ASCII (smooth, keeps the posture loop; beeps) or 'I' binary",
    )
    hw.add_argument(
        "--imu",
        choices=["auto", "poll", "stream"],
        default="auto",
        help="how to read the IMU: poll = one 'gp' per reading (default on a real robot; "
        "firmware B10_250330 floods the link when streaming), stream = 'gP'",
    )
    hw.add_argument(
        "--drift-cal",
        type=float,
        default=None,
        metavar="S",
        help="seconds to hold still at start while heading drift is measured and then "
        "subtracted (default 5 on a real robot, 0 on the simulator; 0 disables)",
    )
    hw.add_argument("--imu-rate", type=float, default=20.0, help="IMU polls per second")
    hw.add_argument("--joint-rate", type=float, default=15.0, help="max head commands per second")
    hw.add_argument("--boot-timeout", type=float, default=12.0)
    mk = ap.add_argument_group("mock")
    mk.add_argument(
        "--mock-script",
        choices=SCRIPT_CHOICES,
        default="none",
        help="sweep: keep rotating; memory: blindfolded and still for 20 s; "
        "dark-turn: blindfolded 90 deg turn (see fly_compass/scenarios.py)",
    )
    mk.add_argument(
        "--exit-after-script",
        action="store_true",
        help="quit when the scenario ends (implied by --no-dashboard)",
    )
    mk.add_argument("--mock-noise", type=float, default=0.25, help="IMU yaw noise, deg")
    mk.add_argument("--mock-drift", type=float, default=0.0, help="IMU yaw drift, deg/s")
    mk.add_argument(
        "--mock-turn-rate",
        type=float,
        default=8.0,
        metavar="DEG_S",
        help="how fast kvtL/kvtR turn the simulated dog (default 8; real carpet 2-7)",
    )
    mk.add_argument(
        "--mock-gait-stall",
        type=float,
        default=None,
        metavar="S",
        help="a turning gait silently stops after S s until it is re-issued "
        "(tests the sweep watchdog)",
    )
    ap.add_argument(
        "--blindfold-after",
        type=float,
        metavar="S",
        default=None,
        help="withhold the landmark cue after S seconds (toggle back in the dashboard)",
    )
    ap.add_argument("-v", "--verbose", action="store_true")
    return ap


def print_phase_table(phases: dict, out=None) -> None:
    """Per phase and model: mean |error|, error at the end of the phase, mean confidence."""
    out = out or sys.stdout
    if not phases:
        return
    print("per phase: mean |err| / err at phase end (deg), mean bump confidence", file=out)
    w = max(len(p) for p in phases) + 2
    print(f"  {'phase':<{w}}" + "".join(f"{k:>26}" for k in MODEL_KEYS), file=out)
    for ph, per in phases.items():
        cells = []
        for k in MODEL_KEYS:
            d = per.get(k) or {}
            if not d.get("samples"):
                cells.append(f"{'-':>26}")
                continue
            conf = "  -" if d.get("mean_conf") is None else f"{d['mean_conf']:.2f}"
            cells.append(f"{d['mae_deg']:>8.1f} /{d['final_err_deg']:>+7.1f}  c {conf:>4}")
        print(f"  {ph:<{w}}" + "".join(cells), file=out)


def _fail(code: int, message: str) -> int:
    """Print why the demo cannot run, tell Flydeck (when it started us), return the exit code."""
    print(message, file=sys.stderr)
    status.failed(message.removeprefix("fly-compass: "))
    return code


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    if a.yaw_sign is None:
        a.yaw_sign = 1 if (a.mock or a.replay) else -1
    logging.basicConfig(
        level=logging.INFO if a.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    if a.mode in LEG_MODES and not a.i_have_space:
        return _fail(
            2,
            f"{a.mode} mode makes the robot walk in place. Clear a 50 cm circle and add "
            "--i-have-space.",
        )
    if not 5 <= a.sweep_angle <= 90 or a.sweep_pause < 0 or a.sweep_timeout <= 0:
        return _fail(2, "--sweep-angle must be 5..90, --sweep-pause >= 0, --sweep-timeout > 0")
    if a.sweep_cycles is not None and a.sweep_cycles < 1:
        return _fail(2, "--sweep-cycles must be at least 1")
    if a.mode == "sweep" and a.mock_script != "none":
        return _fail(
            2,
            "fly-compass: --mode sweep turns the dog with its gaits; it cannot be combined "
            "with --mock-script, which moves the simulated dog itself",
        )
    if a.mock_script != "none" and not a.mock:
        return _fail(2, "fly-compass: --mock-script needs --mock")
    if a.rate <= 0 or a.rate > 100:
        return _fail(2, "--rate must be in (0, 100]")

    try:
        models = load_models(seed=a.seed)
    except ModelMissing as e:
        return _fail(3, f"fly-compass: {e}")

    cfg = LoopConfig(
        rate_hz=a.rate,
        pan_limit_deg=a.pan_limit,
        pan_sign=a.pan_sign,
        pan_rate_dps=a.pan_rate,
        sweep_angle_deg=a.sweep_angle,
        sweep_pause_s=a.sweep_pause,
        sweep_timeout_s=a.sweep_timeout,
        sweep_cycles=a.sweep_cycles,
    )
    loop = CompassLoop(models, cfg, mode=a.mode, allow_hold=a.i_have_space, driver=a.driver)

    # SIGTERM behaves like Ctrl-C so the robot is always rested. Only the FIRST signal
    # interrupts: a second Ctrl-C or SIGTERM must not abort the cleanup that stops the gait and
    # sends the rest command.
    def _stop(*_):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt

    prev_handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    bittle = mock = replay = None
    label = ""
    if a.replay:
        try:
            samples = load_samples(a.replay)
        except FileNotFoundError as e:
            return _fail(2, f"fly-compass: {e}")
        replay = ReplaySource(samples)
        label = f"replay {a.replay} ({len(samples)} samples, {replay.duration:.1f}s)"
        loop.set_mode("observe")
    else:
        if a.mock:
            mock = MockBittle(
                imu_noise_deg=a.mock_noise,
                imu_drift_deg_s=a.mock_drift,
                seed=a.seed,
                turn_rate_deg_s=a.mock_turn_rate,
                gait_stall_after_s=a.mock_gait_stall,
            )
            if a.mock_script == "sweep":
                mock.trajectory = sweep_trajectory
            transport = MockTransport(mock)
            label = "mock"
        else:
            transport = open_transport(port=a.port, ble=a.ble)
            label = transport.name
            status.state("connecting", label)
            print("fly-compass: real robot.\n" + SAFETY.format(lim=a.pan_limit))
            if a.mode in LEG_MODES:
                print(f"  {a.mode.upper()} MODE: the legs will step. Keep hands and cables clear.")
            print("  Starting in 3 s ...", flush=True)
            try:
                time.sleep(3)
            except KeyboardInterrupt:
                return 130
        bittle = Bittle(
            transport,
            yaw_sign=a.yaw_sign,
            joint_token=a.joint_token,
            joint_rate_hz=a.joint_rate,
            joint_limits={P.JOINT_HEAD_PAN: (-a.pan_limit, a.pan_limit)},
        )
        try:
            bittle.connect(boot_timeout=0.5 if a.mock else a.boot_timeout)
        except (TransportError, BittleError) as e:
            _fail(4, f"fly-compass: cannot connect: {e}")
            try:
                transport.close()
            except Exception:  # noqa: BLE001, S110 - best-effort close after a failed connect
                pass
            return 4
        label = f"{label}: {transport.description}"

    session = None
    if not a.no_log and replay is None:
        session = SessionLog(
            a.runs_dir,
            meta={
                "argv": sys.argv,
                "source": label,
                "firmware": bittle and " ".join(x for x in (bittle.model, bittle.version) if x),
                "config": vars(cfg),
                "yaw_sign": a.yaw_sign,
                "note": "yaw_deg in samples.csv is firmware yaw x yaw_sign (CCW positive)",
            },
        )

    hub = FrameHub()
    fast_replay = replay is not None and a.replay_speed <= 0
    app = CompassApp(
        loop,
        hub,
        bittle=bittle,
        mock=mock,
        replay=replay,
        replay_speed=a.replay_speed,
        session=session,
        source_label=label,
        exit_after_replay=a.no_dashboard or fast_replay,
        blindfold_after=a.blindfold_after,
        scenario=a.mock_script if a.mock and a.mock_script in SCENARIOS else None,
        exit_after_scenario=a.no_dashboard or a.exit_after_script,
    )
    app.imu_mode = ("stream" if a.mock else "poll") if a.imu == "auto" else a.imu
    app.imu_rate_hz = a.imu_rate
    app.drift_cal_s = (0.0 if a.mock else 5.0) if a.drift_cal is None else a.drift_cal
    app.on_event = lambda ev: print(f"fly-compass: {ev}", flush=True)
    if a.mock_script in SCENARIOS:

        def _done(res):
            print(f"fly-compass: scenario {a.mock_script} finished")
            print_phase_table(res)

        app.on_scenario_done = _done
    server = None
    if not a.no_dashboard and not fast_replay:
        try:
            server = DashboardServer(
                hub,
                app.on_command,
                lambda: find_benchmark(a.benchmark),
                host=a.host,
                port=a.http_port,
            ).start()
        except OSError as e:
            app.shutdown()
            return _fail(
                5, f"fly-compass: dashboard port {a.http_port} unavailable ({e}); use --http-port"
            )
        print(f"fly-compass: dashboard at {server.url}", flush=True)
        if a.open and not a.no_open:
            webbrowser.open(server.url)
    print(
        f"fly-compass: {label} | mode {loop.mode} | driver {loop.driver} | {a.rate:g} Hz"
        + (f" | logging to {session.dir}" if session else ""),
        flush=True,
    )
    if a.mock:
        print("  mock: drag the dial or use the arrow keys in the dashboard to turn the dog.")

    code = 0
    try:
        status.ready(server.url if server is not None else None)
        if app.drift_cal_s > 0 and bittle is not None:
            status.state("calibrating", f"hold the robot still for {app.drift_cal_s:.0f} s")
        app.start_robot()
        status.state("running", f"mode {loop.mode}")
        summary = app.run(duration=a.duration)
    except KeyboardInterrupt:
        summary = app.summary()
        print("\nfly-compass: stopping (Ctrl-C)")
    except Exception as e:  # noqa: BLE001 - any error must still rest the robot
        summary = app.summary()
        print(f"fly-compass: error: {e!r}; resting the robot", file=sys.stderr)
        status.failed(f"error: {e}")
        code = 1
    finally:
        app.shutdown()
        if server is not None:
            server.stop()
    print("mean |heading error| over IMU samples since the last reset (deg):")
    print(f"  {'model':<11} {'cue on':>8} {'blind':>8}")
    for k, v in summary.items():
        note = "" if v["accepts_cue"] else "  (model has no cue input)"
        print(
            f"  {k:<11} {v['mae_cue_deg']!s:>8} {v['mae_blind_deg']!s:>8}"
            f"   ({v['samples']} samples, {v['ms_per_step']} ms/step){note}"
        )
    print_phase_table(app.phase_summary())
    if session:
        print(f"session log: {session.dir}")
    signal.signal(signal.SIGINT, prev_handlers[0])
    signal.signal(signal.SIGTERM, prev_handlers[1])
    return code


if __name__ == "__main__":
    sys.exit(main())
