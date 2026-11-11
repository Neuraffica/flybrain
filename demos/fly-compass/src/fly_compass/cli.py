"""fly-compass: a fruit-fly head-direction circuit keeps a Bittle's head pointing one way.

uv run fly-compass --mock                      # simulated dog + dashboard
uv run fly-compass --port /dev/cu.usbmodem1101 # real dog over USB (needle mode)
uv run fly-compass --ble                       # real dog over Bluetooth LE
uv run fly-compass --replay runs/20261111-101500
uv run fly-compass --port ... --mode hold --i-have-space
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

from flybrain_edge.bittle import Bittle, MockBittle, MockTransport, TransportError, open_transport
from flybrain_edge.bittle import protocol as P
from flybrain_edge.bittle.robot import BittleError

from .app import CompassApp
from .benchmark import find_benchmark
from .loop import MODES, CompassLoop, LoopConfig
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
        "--mode", choices=MODES, default="needle", help="needle (default), hold, observe"
    )
    ap.add_argument(
        "--i-have-space",
        action="store_true",
        help="allow hold mode, which makes the robot step and turn in place",
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
        default=1,
        help="+1 if the firmware yaw grows when the dog turns left (CCW)",
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


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO if a.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    if a.mode == "hold" and not a.i_have_space:
        print(
            "hold mode makes the robot walk in place. Clear a 50 cm circle and add --i-have-space.",
            file=sys.stderr,
        )
        return 2
    if a.mock_script != "none" and not a.mock:
        print("fly-compass: --mock-script needs --mock", file=sys.stderr)
        return 2
    if a.rate <= 0 or a.rate > 100:
        print("--rate must be in (0, 100]", file=sys.stderr)
        return 2

    try:
        models = load_models(seed=a.seed)
    except ModelMissing as e:
        print(f"fly-compass: {e}", file=sys.stderr)
        return 3

    cfg = LoopConfig(
        rate_hz=a.rate, pan_limit_deg=a.pan_limit, pan_sign=a.pan_sign, pan_rate_dps=a.pan_rate
    )
    loop = CompassLoop(models, cfg, mode=a.mode, allow_hold=a.i_have_space, driver=a.driver)

    # SIGTERM behaves like Ctrl-C so the robot is always rested
    def _term(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _term)

    bittle = mock = replay = None
    label = ""
    if a.replay:
        try:
            samples = load_samples(a.replay)
        except FileNotFoundError as e:
            print(f"fly-compass: {e}", file=sys.stderr)
            return 2
        replay = ReplaySource(samples)
        label = f"replay {a.replay} ({len(samples)} samples, {replay.duration:.1f}s)"
        loop.set_mode("observe")
    else:
        if a.mock:
            mock = MockBittle(imu_noise_deg=a.mock_noise, imu_drift_deg_s=a.mock_drift, seed=a.seed)
            if a.mock_script == "sweep":
                mock.trajectory = sweep_trajectory
            transport = MockTransport(mock)
            label = "mock"
        else:
            transport = open_transport(port=a.port, ble=a.ble)
            label = transport.name
            print("fly-compass: real robot.\n" + SAFETY.format(lim=a.pan_limit))
            if a.mode == "hold":
                print("  HOLD MODE: the legs will step. Keep hands and cables clear.")
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
            print(f"fly-compass: cannot connect: {e}", file=sys.stderr)
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
            print(
                f"fly-compass: dashboard port {a.http_port} unavailable ({e}); use --http-port",
                file=sys.stderr,
            )
            app.shutdown()
            return 5
        print(f"fly-compass: dashboard at {server.url}", flush=True)
        if a.open:
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
        app.start_robot()
        summary = app.run(duration=a.duration)
    except KeyboardInterrupt:
        summary = app.summary()
        print("\nfly-compass: stopping (Ctrl-C)")
    except Exception as e:  # noqa: BLE001 - any error must still rest the robot
        summary = app.summary()
        print(f"fly-compass: error: {e!r}; resting the robot", file=sys.stderr)
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
    return code


if __name__ == "__main__":
    sys.exit(main())
