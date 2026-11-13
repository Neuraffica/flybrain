"""fly-escape-onboard: the fly escape circuit runs on the robot's own board; the laptop listens.

uv run fly-escape-onboard --mock --open          # simulated robot with the module
uv run fly-escape-onboard --port /dev/cu.usbmodem1101 --i-have-space
uv run flybrain-bittle ble-run -- uv run fly-escape-onboard --ble --i-have-space

Needs firmware with the fly_escape module (firmware/bittle/fly_escape/README.md). The robot
reads its IR sensor, runs the circuit every 35 ms and backs away by itself. This program only
switches the module on or off and shows its messages; it never sends a motion command.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
import webbrowser
from collections import deque
from importlib import resources
from typing import Any

from flybrain_demo_api import status
from flybrain_demo_api.web import DashboardServer, FrameHub
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport, TransportError, open_transport
from flybrain_edge.bittle.onboard import OnboardEscape
from flybrain_edge.bittle.robot import BittleError
from flybrain_models.escape import raw_to_proximity

README = "firmware/bittle/fly_escape/README.md"
TRACE_S = 30.0
PUBLISH_HZ = 5.0
MOCK_LINKS = {
    "usb": {},
    "ble": {"ble_notifications": True},
    "ble10": {"ble_notifications": True, "ble_chunk": 10},
}


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="fly-escape-onboard",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    link = ap.add_mutually_exclusive_group()
    link.add_argument("--mock", action="store_true", help="simulated robot with the module")
    link.add_argument("--port", metavar="PATH", help="USB serial device (default: auto-detect)")
    link.add_argument(
        "--ble",
        nargs="?",
        const="",
        default=None,
        metavar="NAME",
        help="Bluetooth LE; optional name, e.g. BittleC2_BLE",
    )
    ap.add_argument(
        "--enable",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="switch the module on if it is off (default; the robot stands and then backs away "
        "by itself, so a real robot needs --i-have-space)",
    )
    ap.add_argument(
        "--leave-running",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="leave the module on when this program ends (default); --no-leave-running "
        "switches it off",
    )
    ap.add_argument(
        "--i-have-space", action="store_true", help="allow switching the module on (legs move)"
    )
    ap.add_argument("--http-port", type=int, default=8768, help="dashboard port (0 = any)")
    ap.add_argument("--open", action="store_true", help="open the dashboard in a browser")
    ap.add_argument("--no-open", action="store_true", help="never open a browser")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds")
    ap.add_argument("--boot-timeout", type=float, default=12.0)
    ap.add_argument(
        "--mock-link",
        choices=sorted(MOCK_LINKS),
        default="usb",
        help="simulator: how the output arrives (ble: one notification per print, as firmware "
        "B10_250330; ble10: 10-byte pieces, as 32a1fcb)",
    )
    ap.add_argument("-v", "--verbose", action="store_true")
    return ap


def mock_hand(t: float) -> tuple[float, float]:
    """Simulator: every 8 s a hand comes in, from the left, the right, ahead, and then one that
    only hovers in the sensor's graded band (no looming, so no escape)."""
    if t < 0:
        return 80.0, 0.0
    i, u = divmod(t, 8.0)
    kind = int(i) % 4
    side = (0.8, -0.8, 0.0, 0.0)[kind]
    if not 4.0 <= u < 6.0:
        return 80.0, side
    if kind == 3:
        return 17.0, side
    return max(5.0, 60.0 - (u - 4.0) * 110.0), side  # 0.5 s from 60 to 5 cm, then held


def fail(code: int, message: str) -> int:
    print(f"fly-escape-onboard: {message}", file=sys.stderr)
    status.failed(message)
    return code


class Listener:
    """What the dashboard shows, built from the module's messages."""

    def __init__(self, bot: Bittle, onboard: OnboardEscape, allow_enable: bool, label: str):
        self.bot, self.on = bot, onboard
        self.allow_enable = allow_enable
        self.label = label
        self.t0 = time.monotonic()
        self.trace: deque[list[float]] = deque()
        self.triggers: deque[list[float]] = deque(maxlen=50)
        self.escapes = 0
        self.escaping = False
        self.events: deque[str] = deque(maxlen=8)
        self.want_telemetry = True
        self._tel_asked = 0.0

    def event(self, text: str) -> None:
        print(f"fly-escape-onboard: {text}", flush=True)
        self.events.append(f"{time.monotonic() - self.t0:6.1f} s  {text}")

    def update(self) -> None:
        for t, m in self.on.drain():
            ts = round(t - self.t0, 2)
            if m.kind == "t" and self.on.baseline:
                bl, br = self.on.baseline
                self.trace.append(
                    [
                        ts,
                        round(raw_to_proximity(m.left, bl), 3),
                        round(raw_to_proximity(m.right, br), 3),
                        m.gf_left,
                        m.gf_right,
                        m.left,
                        m.right,
                    ]
                )
            elif m.kind == "trig":
                self.triggers.append([ts, m.side, m.drive])
                self.escapes += 1
                self.escaping = True
                side = {1: "left", -1: "right"}.get(m.side, "ahead")
                self.event(f"the robot decided to escape: threat {side}, drive {m.drive:.2f}")
            elif m.kind == "done":
                self.escaping = False
                self.event("escape finished, the robot listens again")
            elif m.kind == "base":
                self.event(f"resting levels measured: left {m.left:.0f}, right {m.right:.0f}")
            elif m.kind == "status":
                self.event(
                    "module "
                    + ("on" if m.enabled else "off")
                    + (", telemetry on" if m.telemetry else "")
                )
                if not m.enabled:
                    self.escaping = False
        while self.trace and self.trace[0][0] < time.monotonic() - self.t0 - TRACE_S:
            self.trace.popleft()
        # telemetry on whenever the module is on (it is reset to off at every activation)
        now = time.monotonic()
        wants = self.want_telemetry and self.on.enabled and not self.on.telemetry
        if wants and now - self._tel_asked > 2.0:
            self._tel_asked = now
            self.on.set_telemetry(True)

    def command(self, cmd: dict[str, Any]) -> dict[str, Any]:
        action = cmd.get("action")
        if action in ("enable", "disable"):
            on = action == "enable"
            if on and not self.allow_enable:
                return {
                    "ok": False,
                    "error": "switching it on moves the legs: start with "
                    "--i-have-space (Flydeck asks for it)",
                }
            ok = self.on.set_enabled(on)
            return {"ok": ok} if ok else {"ok": False, "error": "the robot did not answer"}
        return {"ok": False, "error": f"unknown action {action!r}"}

    def frame(self) -> dict[str, Any]:
        b, on = self.bot, self.on
        last = self.trace[-1] if self.trace else None
        return {
            "t": round(time.monotonic() - self.t0, 2),
            "link": self.label,
            "connected": b.connected,
            "firmware": " ".join(x for x in (b.model, b.version) if x) or None,
            "low_power": b.low_power,
            "module": {"enabled": on.enabled, "telemetry": on.telemetry},
            "baseline": list(on.baseline) if on.baseline else None,
            "gf": [last[3], last[4]] if last else None,
            "trace": list(self.trace),
            "triggers": list(self.triggers),
            "escapes": self.escapes,
            "escaping": self.escaping,
            "events": list(self.events),
            "allow_enable": self.allow_enable,
        }


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO if a.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    allow_enable = a.mock or a.i_have_space
    if a.enable and not allow_enable:
        return fail(
            2,
            "switching the module on makes the robot stand and walk backwards: clear a metre "
            "behind it and add --i-have-space, or use --no-enable",
        )

    stop = threading.Event()

    def on_signal(*_):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stop.set()

    previous = signal.signal(signal.SIGINT, on_signal), signal.signal(signal.SIGTERM, on_signal)

    status.state("connecting")
    mock = MockBittle(fly_escape=True, **MOCK_LINKS[a.mock_link]) if a.mock else None
    transport = MockTransport(mock) if mock else open_transport(port=a.port, ble=a.ble)
    bot = Bittle(transport)
    try:
        bot.connect(boot_timeout=0.5 if a.mock else a.boot_timeout)
    except (TransportError, BittleError) as e:
        try:
            transport.close()
        except Exception:  # noqa: BLE001, S110 - best effort after a failed connect
            pass
        return fail(4, f"cannot connect: {e}")

    onboard = OnboardEscape(bot)
    present = onboard.query()
    problem = None
    if present is None and bot.model is None:
        problem = (
            "the robot is connected but answers nothing, not even who it is. This is the link, "
            "not the firmware: press Start again"
        )
    elif present is None:
        problem = (
            "no fly escape module answered. Either this firmware has none, or it has an older "
            f"build that does not report over Bluetooth. Build and flash it: {README}"
        )
    elif not present:
        problem = (
            f"this robot's firmware has no fly escape module (E). Build and flash it: {README}"
        )
    elif not onboard.reports:
        problem = (
            "this firmware has an older build of the fly escape module, which prints to USB only "
            f"and has no telemetry. Rebuild and flash it from {README}"
        )
    if problem:
        bot.close(rest=False)
        signal.signal(signal.SIGINT, previous[0])
        signal.signal(signal.SIGTERM, previous[1])
        return fail(1, problem)

    label = "simulator" if mock else transport.description
    lis = Listener(bot, onboard, allow_enable, label)
    hub = FrameHub()
    page = resources.files("fly_escape_onboard").joinpath("static/index.html").read_bytes()
    try:
        server = DashboardServer(hub, lis.command, port=a.http_port, page=page).start()
    except OSError as e:
        bot.close(rest=False)
        return fail(5, f"dashboard port {a.http_port} unavailable ({e})")
    print(f"fly-escape-onboard: dashboard at {server.url}", flush=True)
    if a.open and not a.no_open:
        webbrowser.open(server.url)

    code = 0
    t0 = time.monotonic()
    try:
        status.ready(server.url)
        if not onboard.enabled and a.enable:
            status.state("calibrating", "keep hands away from the sensor")
            lis.event("switching the module on: the robot stands up, keep hands away")
            if not onboard.set_enabled(True):
                lis.event("the robot did not confirm switching the module on")
        if mock is not None:
            mock.hand.run_script(mock_hand, mock.clock() + 1.0)
        status.state("running", "the brain runs on the robot")
        next_pub = 0.0
        while not stop.is_set():
            if a.duration is not None and time.monotonic() - t0 >= a.duration:
                break
            if not bot.connected:
                raise RuntimeError("robot link lost")
            lis.update()
            if bot.low_power:
                status.state("attention", "battery off or flat: " + bot.low_power)
            if time.monotonic() >= next_pub:
                next_pub = time.monotonic() + 1.0 / PUBLISH_HZ
                hub.publish(lis.frame())
            stop.wait(0.05)
    except Exception as e:  # noqa: BLE001 - still leave the module as the user chose
        status.failed(f"error: {e}")
        print(f"fly-escape-onboard: error: {e}", file=sys.stderr)
        code = 1
    finally:
        if bot.connected and onboard.enabled:
            lis.want_telemetry = False
            if a.leave_running:
                onboard.set_telemetry(False)
                lis.event("telemetry off; the module keeps running on the robot")
            else:
                onboard.set_enabled(False)
                lis.event("module switched off")
        lis.update()
        hub.publish(lis.frame())
        bot.close(rest=False)  # never a motion command: the robot stays as the module left it
        server.stop()
        signal.signal(signal.SIGINT, previous[0])
        signal.signal(signal.SIGTERM, previous[1])
    print(f"fly-escape-onboard: done, {lis.escapes} escape(s) decided on the robot")
    return code


if __name__ == "__main__":
    sys.exit(main())
