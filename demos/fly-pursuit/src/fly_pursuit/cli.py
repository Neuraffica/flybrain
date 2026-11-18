"""fly-pursuit: a circuit of 265 neurons from the fly connectome turns a robot dog toward a
target. In the simulator the whole dog turns; on the robot its head turns toward a face seen
by the camera in its mouth (the legs do not walk).

uv run fly-pursuit --mock --open                  # simulator, dashboard in the browser
uv run fly-pursuit --mock --controller no-aotu019 --fov 30 --target jumps
uv run fly-pursuit --port /dev/cu.usbserial-XXXX --open   # the robot over USB
uv run fly-pursuit --ble --open                   # the robot over Bluetooth
uv run fly-pursuit --ble --camera usb --open      # camera on its own USB cable to the laptop
uv run fly-pursuit --port /dev/cu.usbserial-XXXX --camera-off   # recovery: camera off, saved

On the robot: put it on the floor, away from edges. With --camera robot (default) the camera
must be plugged into the Grove socket labelled SCL SDA +5V GND, and the robot sits by itself
when the camera starts; its firmware can start the camera only once per power-on.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import webbrowser
from collections.abc import Callable
from importlib import resources

from flybrain_demo_api import status
from flybrain_demo_api.web import DashboardServer, FrameHub

from .flydeck import CONTROLLERS, FOVS, MOTIONS, SPEC

# Robot defaults (kept here so that the CLI and the Flydeck card agree; robot.HeadConfig has
# the same numbers).
CAMERA_FOV_DEG = 60.0
HEAD_RATE_DEG_S = 60.0
PAN_LIMIT_DEG = 50.0
COMMAND_HZ = 8.0
GAIN = 1.0
START_DELAY_S = 3.0  # on the robot: time to read the safety line (and press Stop) first

FLOOR = "the robot must be on the floor, away from edges (it sits when the camera starts)"


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="fly-pursuit",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # The Flydeck contract: --mock, --port, --ble. Not a mutually exclusive group, so that a
    # refused combination gets the plain message below instead of an argparse error.
    ap.add_argument("--mock", action="store_true", help="simulator, no robot")
    ap.add_argument("--port", metavar="PATH", help="USB serial device (default: auto-detect)")
    ap.add_argument(
        "--ble",
        nargs="?",
        const="",
        default=None,
        metavar="NAME",
        help="Bluetooth; optional name, e.g. BittleC2 (default: the first Bittle found)",
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
    ap.add_argument("--arousal", type=float, default=1.0, help="0..1 (circuit controllers)")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds")
    sim = ap.add_argument_group("simulator only")
    sim.add_argument(
        "--fov",
        type=int,
        choices=FOVS,
        default=30,
        help="camera half field of view in degrees (100 = camera on a panning head)",
    )
    sim.add_argument("--target", choices=MOTIONS, default="still", help="target motion")
    sim.add_argument("--seed", type=int, default=0, help="seed for camera noise and motions")
    rb = ap.add_argument_group("robot only")
    rb.add_argument(
        "--camera",
        choices=("robot", "usb"),
        default="robot",
        help="robot: through the robot's Grove socket (no cable to the laptop; one start per "
        "power-on). usb: the camera's own USB cable plugged into the laptop (default robot)",
    )
    rb.add_argument(
        "--camera-port",
        metavar="PATH",
        default=None,
        help="--camera usb: the camera's serial device (default: find it by asking each "
        "CH343 port for its name)",
    )
    rb.add_argument(
        "--gain",
        type=float,
        default=GAIN,
        help="scales the circuit's turn command on the robot, 0.1 to 2 (default 1)",
    )
    rb.add_argument(
        "--no-usb-restart",
        action="store_true",
        help="--camera robot over USB: do not restart the board before starting (the camera "
        "then works only if it was not used since the robot was switched on)",
    )
    rb.add_argument(
        "--camera-fov",
        type=float,
        default=CAMERA_FOV_DEG,
        metavar="DEG",
        help=f"the camera's field of view across the image, degrees (default "
        f"{CAMERA_FOV_DEG:g}; not properly measured, one head step gave about 57)",
    )
    rb.add_argument(
        "--head-rate",
        type=float,
        default=HEAD_RATE_DEG_S,
        metavar="DEG_S",
        help=f"fastest head turn, deg/s (default {HEAD_RATE_DEG_S:g}; this program's choice)",
    )
    rb.add_argument(
        "--pan-limit",
        type=float,
        default=PAN_LIMIT_DEG,
        metavar="DEG",
        help=f"furthest the head turns each way, degrees (default {PAN_LIMIT_DEG:g}; at most "
        "the firmware's 120)",
    )
    rb.add_argument(
        "--command-rate",
        type=float,
        default=COMMAND_HZ,
        metavar="HZ",
        help=f"head commands per second (default {COMMAND_HZ:g}); they compete with the "
        "camera on the robot",
    )
    rb.add_argument(
        "--pan-sign",
        type=int,
        choices=(1, -1),
        default=1,
        help="1 if a positive head angle looks left (default 1)",
    )
    rb.add_argument(
        "--leave-voice-off",
        action="store_true",
        help="do not switch the voice module back on at the end",
    )
    rb.add_argument(
        "--camera-off",
        action="store_true",
        help="only switch the camera off (and save that on the robot), rest, exit",
    )
    rb.add_argument("--boot-timeout", type=float, default=12.0, help=argparse.SUPPRESS)
    rb.add_argument("--camera-timeout", type=float, default=14.0, help=argparse.SUPPRESS)
    return ap


def fail(code: int, message: str) -> int:
    """Say why we cannot run (terminal and Flydeck) and return the exit code."""
    print(f"fly-pursuit: {message}", file=sys.stderr)
    status.failed(message)
    return code


def main(argv: list[str] | None = None, make_transport: Callable[[], object] | None = None) -> int:
    """``make_transport`` replaces the real link (tests pass a fake robot)."""
    a = build_parser().parse_args(argv)
    robot = not a.mock
    if a.mock and (a.port is not None or a.ble is not None):
        return fail(2, "choose the simulator (--mock) or a robot (--port / --ble), not both")
    if a.port is not None and a.ble is not None:
        return fail(2, "choose USB (--port) or Bluetooth (--ble), not both")
    if a.camera_off and not robot:
        return fail(2, "--camera-off needs a robot: add --port or --ble")
    if a.camera_port == "":
        a.camera_port = None  # Flydeck passes an empty field as ""
    if a.camera_port and a.camera != "usb":
        return fail(2, "--camera-port is for --camera usb")
    if (
        robot
        and not a.camera_off
        and a.camera == "usb"
        and a.ble is None
        and not a.port
        and not a.camera_port
    ):
        return fail(
            2,
            "with the robot on USB and the camera on its own USB cable, both look the same to "
            "the computer: give --port (the robot) or --camera-port (the camera)",
        )
    if not 0.0 <= a.arousal <= 1.0:
        return fail(2, "--arousal must be between 0 and 1")
    if a.duration is not None and a.duration <= 0:
        return fail(2, "--duration must be positive")
    cfg = None
    if robot:
        from .robot import HeadConfig

        cfg = HeadConfig(
            camera_fov_deg=a.camera_fov,
            head_rate_deg_s=a.head_rate,
            pan_limit_deg=a.pan_limit,
            command_hz=a.command_rate,
            pan_sign=a.pan_sign,
            voice_on_at_exit=not a.leave_voice_off,
            boot_timeout_s=a.boot_timeout,
            camera_timeout_s=a.camera_timeout,
            gain=a.gain,
            camera_source=a.camera,
            usb_restart=not a.no_usb_restart,
        )
        err = cfg.check()
        if err:
            return fail(2, err)
        if make_transport is None:
            make_transport = _real_transport(a)

    # The first Ctrl-C (or Flydeck's Stop) asks the loop to finish; later ones are ignored.
    stop = threading.Event()

    def on_signal(*_):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stop.set()

    previous = signal.signal(signal.SIGINT, on_signal), signal.signal(signal.SIGTERM, on_signal)
    try:
        if robot and a.camera_off:
            from .robot import camera_off

            status.state("connecting", "switching the camera off")
            code = camera_off(make_transport, cfg)
            if code == 4:
                status.failed("could not connect to the robot")
            elif code:
                status.failed("the robot did not confirm every command: run it again")
            return code
        return _run(a, stop, cfg, make_transport)
    finally:
        signal.signal(signal.SIGINT, previous[0])
        signal.signal(signal.SIGTERM, previous[1])


def _real_transport(a: argparse.Namespace) -> Callable[[], object]:
    """The robot's link from the flags. With the camera on its own USB cable and the robot on
    USB without --port, the robot's port is the first one that is not the camera."""
    from flybrain_edge.bittle import TransportError, find_serial_ports, open_transport

    def make():
        port = a.port
        if a.ble is None and port is None and a.camera == "usb" and a.camera_port:
            from .usbcam import same_device

            others = [p for p in find_serial_ports() if not same_device(p, a.camera_port)]
            if not others:
                raise TransportError("no robot USB port found besides the camera's")
            port = others[0]
        return open_transport(port=port, ble=a.ble)

    return make


def _run(a: argparse.Namespace, stop: threading.Event, cfg, make_transport) -> int:
    robot = cfg is not None
    if robot:  # the first status line says where the robot must be
        status.state("starting", FLOOR)
        print(f"fly-pursuit: real robot: {FLOOR}.", flush=True)
    else:
        status.state("starting", "loading the circuit")
    try:  # imported here so that a missing data file gets a plain message and exit code 3
        from .app import PursuitApp
        from .controllers import benchmark_summary

        if robot:
            from .camera import CameraReader
            from .robot import HeadRig
            from .usbcam import UsbCameraReader

            kind = UsbCameraReader if cfg.camera_source == "usb" else CameraReader
            reader = kind(fov_deg=cfg.camera_fov_deg)

            def attention(text):
                print(f"fly-pursuit: {text}", flush=True)
                status.state("attention", text)

            rig = HeadRig(
                reader, cfg, controller=a.controller, arousal=a.arousal, on_attention=attention
            )
        else:
            from .sim import PursuitSim

            rig = PursuitSim(
                controller=a.controller,
                fov_deg=a.fov,
                motion=a.target,
                arousal=a.arousal,
                seed=a.seed,
            )
        hub = FrameHub()
        app = PursuitApp(rig, hub, frame_hz=cfg.tick_hz if robot else 20.0)
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
    try:
        if robot:
            return _run_robot(a, stop, cfg, make_transport, app, rig, reader, server.url)
        return _run_sim(a, stop, app, rig, server.url)
    finally:
        server.stop()


def _run_sim(a, stop, app, sim, url) -> int:
    print(
        f"fly-pursuit: simulator | controller {a.controller} | field of view +-{a.fov} deg | "
        f"target {a.target}",
        flush=True,
    )
    if a.open and not a.no_open:
        webbrowser.open(url)
    code = 0
    try:
        status.ready(url)
        status.state("running", "simulator")
        app.run(stop, duration=a.duration)
    except Exception as e:  # noqa: BLE001 - report any error in plain words, then exit 1
        print(f"fly-pursuit: error: {e!r}", file=sys.stderr)
        status.failed(f"error: {e}")
        code = 1
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


def _run_robot(a, stop, cfg, make_transport, app, rig, reader, url) -> int:
    from . import usbcam
    from .robot import (
        CAMERA_CHECK,
        LINK_LABELS,
        ONCE_PER_BOOT,
        LinkLost,
        RestartingTransport,
        connect,
        shut_down,
        start_camera,
        start_robot_for_usb_camera,
    )

    usb_cam = cfg.camera_source == "usb"
    print(
        "fly-pursuit: head only, the legs do not walk | camera "
        + ("on its own USB cable" if usb_cam else "through the robot")
        + f" | controller {a.controller} | gain {cfg.gain:g} | camera field of view "
        f"{cfg.camera_fov_deg:g} deg (assumed) | head up to {cfg.head_rate_deg_s:g} deg/s, "
        f"+-{cfg.pan_limit_deg:g} deg | {cfg.command_hz:g} head commands/s | Ctrl-C stops and "
        "rests the robot",
        flush=True,
    )
    if not usb_cam and (a.ble is not None or not cfg.usb_restart):
        print(f"fly-pursuit: NOTE: {ONCE_PER_BOOT}.", flush=True)
        status.state("starting", "switch the robot off and on first if its camera was used")
    if stop.wait(START_DELAY_S):
        return 0

    camera_port = None
    if usb_cam:
        status.state("connecting", "looking for the camera on its USB cable")
        camera_port = a.camera_port or usbcam.find_camera_port(skip=[a.port])
        if camera_port is None:
            return fail(
                4,
                "could not find the camera on its own USB cable: plug its USB-C port into "
                "this computer, or give --camera-port",
            )
        print(f"fly-pursuit: camera on {camera_port}", flush=True)

    status.state("connecting", FLOOR)
    try:
        t = make_transport()
        if not usb_cam and cfg.usb_restart and getattr(t, "name", "") == "usb":
            t = RestartingTransport(t)
        b = connect(t, cfg, reader)
    except Exception as e:  # noqa: BLE001 - no port, no Bluetooth, robot off: all exit 4
        return fail(4, f"could not connect to the robot: {e}")
    link = LINK_LABELS.get(getattr(b.transport, "name", ""), "robot")
    print(f"fly-pursuit: connected over {link}: {b.transport.description}", flush=True)

    def reconnect():
        return connect(make_transport(), cfg, reader)

    code, problem, cam = 0, None, None
    try:
        if usb_cam:
            status.state("starting", "voice off, the robot sits, camera on its cable")
            notes = start_robot_for_usb_camera(b)
            why = None
            if not stop.is_set():
                cam = usbcam.UsbCamera(camera_port, reader)
                try:
                    cam.start()
                    rig.camera = cam
                except Exception as e:  # noqa: BLE001 - port busy or gone
                    why = f"cannot open {camera_port}: {e}"
                    cam = None
        else:
            status.state("starting", "switching the camera on; the robot sits by itself")
            why, notes = start_camera(b, cfg)
        for n in notes:
            print(f"fly-pursuit: {n}", flush=True)
        if why is not None:
            check = "the camera's USB cable" if usb_cam else CAMERA_CHECK
            code, problem = 4, f"could not start the camera: {why}. Check: {check}."
        elif not stop.is_set():
            rig.bittle = b
            status.ready(url)
            status.state("running", f"on the robot over {link}: the head follows the face")
            if a.open and not a.no_open:
                webbrowser.open(url)
            app.run(stop, duration=a.duration)
    except LinkLost as e:
        code, problem = 1, f"{e}"
    except Exception as e:  # noqa: BLE001 - any error must still switch the camera off
        code, problem = 1, f"error: {e!r}"
    finally:
        if problem:
            print(f"fly-pursuit: {problem}", file=sys.stderr, flush=True)
        print("fly-pursuit: stopping: head to centre, camera off, resting", flush=True)
        status.state("stopping", "head to centre, camera off, resting")
        if cam is not None and not cam.stop():
            print("fly-pursuit: could not send AT+BREAK to the camera", flush=True)
        off = shut_down(b, cfg, reconnect)
    if not off:
        code = code or 1
        problem = (problem + " " if problem else "") + (
            "The camera may still be on: switch the robot off, unplug the camera, switch it "
            "on, run fly-pursuit --camera-off."
            if not usb_cam
            else "The robot may not be resting: switch it off."
        )
    if problem:
        status.failed(problem)
    st = rig.stats()
    print(
        f"fly-pursuit: stopped after {rig.t:.0f} s; {reader.lines} camera positions; "
        + (
            f"mean |measured face bearing| {st['mae_deg']:.1f} deg over the positions, a bearing "
            f"{100 * st['in_view']:.0f} % of the time."
            if st["mae_deg"] is not None
            else "the camera never reported a face."
        ),
        flush=True,
    )
    return code


if __name__ == "__main__":
    sys.exit(main())
