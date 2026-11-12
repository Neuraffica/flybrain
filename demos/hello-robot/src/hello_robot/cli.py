"""hello-robot: connect, show identity and IMU, stand, wag the head, rest.

uv run hello-robot --mock --open             # simulated robot, dashboard in the browser
uv run hello-robot --port /dev/cu.usbmodem1101
uv run hello-robot --ble --walk --i-have-space
Start it from Flydeck to get the robot picker and the Stop button for free.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
import webbrowser
from importlib import resources

from flybrain_demo_api import status
from flybrain_demo_api.web import DashboardServer, FrameHub
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport, TransportError, open_transport
from flybrain_edge.bittle import protocol as P
from flybrain_edge.bittle.robot import BittleError


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="hello-robot",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    link = ap.add_mutually_exclusive_group()  # the Flydeck contract: --mock, --port, --ble
    link.add_argument("--mock", action="store_true", help="simulated robot (no hardware)")
    link.add_argument("--port", metavar="PATH", help="USB serial device (default: auto-detect)")
    link.add_argument(
        "--ble",
        nargs="?",
        const="",
        default=None,
        metavar="NAME",
        help="Bluetooth LE; optional name, e.g. BittleC2_BLE",
    )
    ap.add_argument("--http-port", type=int, default=8767, help="dashboard port (0 = any)")
    ap.add_argument("--open", action="store_true", help="open the dashboard in a browser")
    ap.add_argument("--no-open", action="store_true", help="never open a browser")
    ap.add_argument("--wags", type=int, default=3, help="head wags (1-10)")
    ap.add_argument("--walk", action="store_true", help="walk forward for about a second")
    ap.add_argument("--i-have-space", action="store_true", help="allow --walk (the legs move)")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds")
    return ap


def fail(code: int, message: str) -> int:
    """Say why we cannot run (terminal and Flydeck) and return the exit code."""
    print(f"hello-robot: {message}", file=sys.stderr)
    status.failed(message)
    return code


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    if not 1 <= a.wags <= 10:
        return fail(2, "--wags must be 1..10")
    if a.walk and not (a.i_have_space or a.mock):
        return fail(2, "--walk moves the legs: clear 50 cm in front and add --i-have-space")

    # Safety rule 1: the first Ctrl-C (or Flydeck's Stop) only asks us to finish. Later
    # signals are ignored so they cannot interrupt the rest command below.
    stop = threading.Event()

    def on_signal(*_):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stop.set()

    previous = signal.signal(signal.SIGINT, on_signal), signal.signal(signal.SIGTERM, on_signal)

    status.state("connecting")
    transport = MockTransport(MockBittle()) if a.mock else open_transport(port=a.port, ble=a.ble)
    bot = Bittle(transport)
    try:
        bot.connect(boot_timeout=0.5 if a.mock else 12.0)
    except (TransportError, BittleError) as e:
        try:
            transport.close()
        except Exception:  # noqa: BLE001, S110 - best effort after a failed connect
            pass
        return fail(4, f"cannot connect: {e}")

    hub = FrameHub()
    wag_again = threading.Event()

    def on_command(cmd: dict) -> dict:  # buttons on the dashboard POST here
        if cmd.get("action") == "wag":
            wag_again.set()
            return {"ok": True}
        return {"ok": False, "error": "unknown action"}

    page = resources.files("hello_robot").joinpath("static/index.html").read_bytes()
    try:
        server = DashboardServer(hub, on_command, port=a.http_port, page=page).start()
    except OSError as e:
        bot.close()
        return fail(5, f"dashboard port {a.http_port} unavailable ({e})")
    print(f"hello-robot: dashboard at {server.url}", flush=True)
    if a.open and not a.no_open:
        webbrowser.open(server.url)

    frame = {
        "model": bot.model,
        "version": bot.version,
        "link": transport.description,
        "phase": "standing up",
        "wags": 0,
        "voltage": None,
        "low_power": None,
    }

    def publish(**changes) -> None:
        frame.update(changes)
        s = bot.latest_sample
        if s is not None:
            frame.update(
                yaw=round(s.yaw_deg, 1), pitch=round(s.pitch_deg, 1), roll=round(s.roll_deg, 1)
            )
        frame["low_power"] = bot.low_power
        hub.publish(dict(frame))

    code = 0
    t0 = time.monotonic()
    try:
        status.ready(server.url)  # Flydeck now shows the dashboard
        publish(voltage=bot.read_voltage())
        bot.stand()
        bot.poll_imu(10.0)
        status.state("running")
        wag_again.set()
        while not stop.is_set():
            if a.duration is not None and time.monotonic() - t0 >= a.duration:
                break
            if wag_again.is_set():
                wag_again.clear()
                for _ in range(a.wags):  # Safety rule 2: one joint target at a time
                    for angle in (30, -30):
                        if stop.is_set():
                            break
                        bot.set_joint(P.JOINT_HEAD_PAN, angle)
                        publish(phase=f"wagging ({angle:+d} deg)")
                        stop.wait(0.6)
                    frame["wags"] += 1
                bot.set_joint(P.JOINT_HEAD_PAN, 0)
                if a.walk and not stop.is_set():  # Safety rule 3: legs only when allowed
                    publish(phase="walking")
                    bot.skill("wkF")
                    stop.wait(1.0)
                    bot.stand()
                publish(phase="listening (press Wag again)")
            publish()
            stop.wait(0.1)
    except Exception as e:  # noqa: BLE001 - any error must still rest the robot
        status.failed(f"error: {e}")
        print(f"hello-robot: error: {e!r}; resting the robot", file=sys.stderr)
        code = 1
    finally:
        publish(phase="resting")
        bot.close()  # always rests, whatever happened
        server.stop()
        signal.signal(signal.SIGINT, previous[0])
        signal.signal(signal.SIGTERM, previous[1])
    print(f"hello-robot: done, {frame['wags']} wags, robot resting")
    return code


if __name__ == "__main__":
    sys.exit(main())
