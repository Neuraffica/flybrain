"""``flybrain-bittle probe``: talk to a Bittle and print exactly what comes back.

Nothing here moves the robot: it sends only queries ('?', 'g?', 'gp', 'v', 'j') and the IMU
print switch ('gP', then 'gp' to stop). Paste the output into an issue if the parser and your
firmware disagree.

    uv run flybrain-bittle probe                 # first USB port that looks like a BiBoard
    uv run flybrain-bittle probe --port /dev/cu.usbmodem1101
    uv run flybrain-bittle probe --ble           # first BLE device named Bittle*_BLE
    uv run flybrain-bittle probe --mock          # the simulator, to see what "good" looks like
    uv run flybrain-bittle ports                 # list candidate serial ports (and BLE devices)
"""

from __future__ import annotations

import argparse
import itertools
import os
import shlex
import shutil
import sys
import time

from . import macos
from . import protocol as P
from .transport import (
    BleTransport,
    MockTransport,
    SerialTransport,
    Transport,
    TransportError,
    find_serial_ports,
)

QUERIES: list[tuple[str, bytes, str]] = [
    ("identify (model, version)", b"?\n", "?"),
    ("gyro state", b"g?\n", "g"),
    ("IMU once (OpenCatEsp32: gp)", b"gp\n", "g"),
    ("IMU once (legacy firmware: v)", b"v\n", "v"),
    ("joint angles", b"j\n", "j"),
]


class Probe:
    def __init__(self, transport: Transport, out=sys.stdout):
        self.t = transport
        self.out = out
        self.t0 = time.monotonic()
        self.split = P.LineSplitter()
        self.imu_by_cmd: dict[str, int] = {}
        self.stream_samples: list[tuple[float, P.ImuReading]] = []
        self.unparsed: list[str] = []
        self.model: str | None = None
        self.version: str | None = None
        self.echoes: dict[str, bool] = {}
        self.joints: dict[int, int] | None = None
        self.analog_reads: list[tuple[float, list[int]]] = []
        self.analog_errors = 0
        self.analog_pins: list[int] = []
        self.analog_side_test = False

    def p(self, msg: str) -> None:
        print(msg, file=self.out, flush=True)

    def stamp(self) -> str:
        return f"+{time.monotonic() - self.t0:7.3f}s"

    def tx(self, data: bytes) -> None:
        self.p(f"{self.stamp()}  TX  {data!r}")
        self.t.write(data)

    def rx_for(
        self, seconds: float, label: str = "", stop_on_echo: str | None = None, show: bool = True
    ) -> list[str]:
        end = time.monotonic() + seconds
        lines: list[str] = []
        while time.monotonic() < end:
            chunk = self.t.read(0.05)
            if not chunk:
                continue
            now = time.monotonic()
            if show:
                self.p(f"{self.stamp()}  RX  {chunk!r}")
            got = self.split.feed(chunk)
            for r in self.split.take_pending_imu():
                self._imu(label, now, r, "(unterminated)")
            for ln in got:
                readings, rest = P.parse_imu(ln)
                for r in readings:
                    self._imu(label, now, r, "")
                # BLE glues some prints together ("Bittle X+ArmB10_250330?"): split them
                for piece in P.split_glued(rest.strip()):
                    piece = piece.strip()
                    if not piece:
                        continue
                    lines.append(piece)
                    if not show:
                        continue
                    cam = P.parse_camera(piece)
                    if cam:
                        self.p(f"            -> camera target {cam}")
                    elif stop_on_echo and P.is_echo(piece, stop_on_echo):
                        self.p(f"            -> echo {piece!r} (command acknowledged)")
                    else:
                        self.unparsed.append(piece)
            if stop_on_echo and any(P.is_echo(x, stop_on_echo) for x in lines):
                if not show:
                    break  # compact mode: the echo ends the reply
                # keep listening briefly for trailing prints
                end = min(end, time.monotonic() + 0.15)
        return lines

    def _imu(self, label: str, now: float, r: P.ImuReading, note: str) -> None:
        self.imu_by_cmd[label] = self.imu_by_cmd.get(label, 0) + 1
        if label == "stream":
            self.stream_samples.append((now, r))
        self.p(
            f"            -> IMU {r.source} yaw={r.yaw_deg:+.1f} pitch={r.pitch_deg:+.1f} "
            f"roll={r.roll_deg:+.1f} acc={r.accel} {note}"
        )

    def run(
        self,
        stream_s: float = 3.0,
        boot_s: float = 10.0,
        rotate_test: bool = False,
        analog_pins: list[int] | None = None,
        analog_s: float = 3.0,
        side_test: bool = False,
    ) -> int:
        self.p(f"== flybrain-bittle probe :: {self.t.description}")
        if self.t.name == "ble":
            self.p(
                "   (BLE: each notification is shown as received, with a newline added when "
                "the firmware sends none)"
            )
        try:
            self.t.open()
        except TransportError as e:
            self.p(f"!! {e}")
            return 2
        try:
            self.p("== waiting for boot banner / quiet line (opening USB usually resets the board)")
            boot = self.rx_for_quiet(boot_s)
            if "(Y/n)" in boot:
                self.p(
                    "!! board is waiting for a first-boot Y/n answer; finish setup in the Petoi app"
                )
                return 3
            if "Ready!" not in boot:
                self.p("   (no 'Ready!' seen: board did not reset on open, or it is still booting)")
            for label, cmd, echo in QUERIES:
                self.p(f"== {label}")
                self.tx(cmd)
                lines = self.rx_for(1.5, label=label, stop_on_echo=echo)
                self.echoes[label] = any(P.is_echo(x, echo) for x in lines)
                if label.startswith("identify"):
                    self.model, self.version = P.parse_identify(lines)
                if label.startswith("joint"):
                    self.joints = P.parse_joint_table(lines)
                time.sleep(0.25)  # also clears print6Axis' 200 ms rate limit
            self.p(f"== continuous IMU print 'gP' for {stream_s:.1f}s")
            if rotate_test:
                self.p(
                    "   ROTATE THE DOG ~90 deg COUNTER-CLOCKWISE (to its left, seen from above) NOW"
                )
            self.tx(b"gP\n")
            self.rx_for(stream_s, label="stream")
            self.tx(b"gp\n")
            self.rx_for(0.6, label="stop", stop_on_echo="g")
            if analog_pins:
                self.analog_section(analog_pins, analog_s, side_test)
        finally:
            self.t.close()
        return self.summary(rotate_test)

    def analog_section(self, pins: list[int], seconds: float, side_test: bool) -> None:
        """Read-only: 'R' analog reads of ``pins`` for ``seconds``, one command at a time."""
        cmd = P.encode_analog_read(pins)
        self.p(f"== analog reads of pins {pins} for {seconds:.1f}s: TX {cmd!r} (read-only)")
        if side_test:
            self.p(
                f"   COVER THE ROBOT'S LEFT SIDE OF THE SENSOR WITH YOUR HAND NOW "
                f"(by default pin {pins[0]} is taken as left)"
            )
        t0 = time.monotonic()
        sec = -1
        per: dict[int, list[int]] = {p: [] for p in pins}
        while time.monotonic() - t0 < seconds:
            self.t.write(cmd)
            lines = self.rx_for(0.6, label="analog", stop_on_echo="R", show=False)
            vals = P.parse_analog_reply(lines, len(pins))
            el = time.monotonic() - t0
            if vals is None:
                self.p(f"{self.stamp()}  analog: no complete reply, got {lines!r}")
                self.analog_errors += 1
                continue
            self.analog_reads.append((el, vals))
            self.p(
                f"{self.stamp()}  "
                + "  ".join(f"{p}={v:5d}" for p, v in zip(pins, vals, strict=True))
            )
            if int(el) != sec:
                if sec >= 0:
                    self._analog_second(sec, per)
                sec, per = int(el), {p: [] for p in pins}
            for p, v in zip(pins, vals, strict=True):
                per[p].append(v)
        if sec >= 0:
            self._analog_second(sec, per)
        self.analog_pins = pins
        self.analog_side_test = side_test

    def _analog_second(self, sec: int, per: dict[int, list[int]]) -> None:
        parts = [f"{p}: min {min(v)} max {max(v)} (n={len(v)})" for p, v in per.items() if v]
        self.p(f"   second {sec}: " + "; ".join(parts))

    def rx_for_quiet(self, max_s: float, quiet_s: float = 1.5) -> str:
        end = time.monotonic() + max_s
        last = time.monotonic()
        text = ""
        while time.monotonic() < end:
            chunk = self.t.read(0.05)
            now = time.monotonic()
            if chunk:
                self.p(f"{self.stamp()}  RX  {chunk!r}")
                text += chunk.decode("latin-1")
                last = now
                if "Ready!" in text:
                    break
            elif now - last > quiet_s:
                break
        self.split = P.LineSplitter()
        return text

    def summary(self, rotate_test: bool) -> int:
        self.p("\n== summary")
        self.p(f"model:    {self.model or 'NOT IDENTIFIED'}")
        self.p(f"version:  {self.version or '?'}")
        if self.joints is not None:
            self.p(
                f"joints:   {len(self.joints)} parsed, "
                + " ".join(f"{i}:{a}" for i, a in sorted(self.joints.items()))
            )
        else:
            self.p("joints:   NOT PARSED")
        for label, ok in self.echoes.items():
            self.p(f"echo for {label!r}: {'yes' if ok else 'NO'}")
        for label, n in self.imu_by_cmd.items():
            self.p(f"IMU readings during {label!r}: {n}")
        n = len(self.stream_samples)
        if n >= 2:
            span = self.stream_samples[-1][0] - self.stream_samples[0][0]
            rate = (n - 1) / span if span > 0 else 0.0
            self.p(f"stream rate: {rate:.1f} Hz over {n} samples (firmware limit is 5 Hz)")
            if rotate_test:
                ys = [r.yaw_deg for _, r in self.stream_samples]
                total = 0.0
                for a, b in itertools.pairwise(ys):
                    d = (b - a + 180) % 360 - 180
                    total += d
                self.p(f"net yaw change: {total:+.1f} deg")
                if abs(total) > 20:
                    self.p(
                        "yaw sign: "
                        + (
                            "+ (CCW positive, default --yaw-sign 1 is right)"
                            if total > 0
                            else "- (use --yaw-sign -1)"
                        )
                    )
        else:
            self.p("stream rate: no continuous IMU lines seen")
        if self.analog_pins:
            n = len(self.analog_reads)
            span = self.analog_reads[-1][0] - self.analog_reads[0][0] if n > 1 else 0.0
            self.p(
                f"analog reads: {n} complete, {self.analog_errors} incomplete"
                + (f", {(n - 1) / span:.1f} reads/s" if span > 0 else "")
            )
            for i, pin in enumerate(self.analog_pins):
                vals = [v[i] for _, v in self.analog_reads]
                if vals:
                    self.p(f"analog pin {pin}: min {min(vals)} max {max(vals)}")
            if self.analog_side_test and n:
                mins = [
                    min(v[i] for _, v in self.analog_reads) for i in range(len(self.analog_pins))
                ]
                covered = self.analog_pins[mins.index(min(mins))]
                if max(mins) - min(mins) < 500:
                    self.p("side test: no clear difference between the pins; try again closer")
                elif covered == self.analog_pins[0]:
                    self.p(
                        f"side test: pin {covered} is the robot's LEFT: default mapping is right"
                    )
                else:
                    self.p(
                        f"side test: pin {covered} is the robot's LEFT: use --swap-sides "
                        "(or list the left pin first)"
                    )
        if self.unparsed:
            self.p(f"unrecognised lines: {len(self.unparsed)} (shown above as RX)")
        ok = bool(self.model) and n >= 2 and all(self.echoes.values())
        self.p(
            "verdict:  "
            + (
                "OK - parser understands this firmware"
                if ok
                else "CHECK - paste this whole output into an issue"
            )
        )
        return 0 if ok else 1


def _transport_from_args(a: argparse.Namespace) -> Transport:
    if a.mock:
        from .mock import MockBittle

        return MockTransport(MockBittle(chatter_per_s=0.5))
    if a.ble is not None:
        return BleTransport(name=a.ble or None)
    return SerialTransport(a.port)


def _list_ports(scan_ble: bool) -> int:
    ports = find_serial_ports()
    print("serial candidates (best first):")
    for p in ports or ["(none)"]:
        print("  ", p)
    if scan_ble:
        import asyncio

        macos.warn_if_ble_will_be_killed()

        from bleak import BleakScanner

        async def scan():
            found = await BleakScanner.discover(timeout=6.0, return_adv=True)
            for d, adv in found.values():
                nm = adv.local_name or d.name
                tag = "  <- Petoi?" if BleTransport.name_matches(nm, None) else ""
                print(f"   {d.address}  {nm!s:24} rssi={adv.rssi}{tag}")

        print("BLE devices:")
        asyncio.run(scan())
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="flybrain-bittle",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    pr = sub.add_parser("probe", help="send read-only queries and print raw replies")
    g = pr.add_mutually_exclusive_group()
    g.add_argument("--port", help="serial device, e.g. /dev/cu.usbmodem1101")
    g.add_argument(
        "--ble",
        nargs="?",
        const="",
        default=None,
        metavar="NAME",
        help="connect over BLE (optional name substring)",
    )
    g.add_argument("--mock", action="store_true", help="probe the simulator")
    pr.add_argument(
        "--seconds",
        type=float,
        default=3.0,
        help="length of the gP stream test, and of the analog section",
    )
    pr.add_argument(
        "--analog",
        metavar="PINS",
        help="also read these analog pins with 'R' (read-only), e.g. 36,39 for the infrared "
        "proximity sensor on the Grove socket labelled 39/36",
    )
    pr.add_argument(
        "--side-test",
        action="store_true",
        help="with --analog: asks you to cover the robot's LEFT side and reports which pin it is",
    )
    pr.add_argument(
        "--rotate-test",
        action="store_true",
        help="ask you to rotate the dog CCW during the stream and report the yaw sign",
    )
    pr.add_argument(
        "--no-reset",
        action="store_true",
        help="try not to reset the board when opening USB (DTR/RTS low)",
    )
    ls = sub.add_parser("ports", help="list candidate serial ports")
    ls.add_argument("--ble", action="store_true", help="also scan for BLE devices")
    mk = sub.add_parser(
        "macos-ble-app",
        help="create the macOS helper app that lets a command use Bluetooth (see ble-run)",
    )
    mk.add_argument(
        "--dir", help=f"where to put {macos.BUNDLE_NAME} (default {macos.default_dir()})"
    )
    br = sub.add_parser(
        "ble-run",
        help="macOS: run a command through the helper app so it may use Bluetooth; "
        "output appears here, Ctrl-C is passed on",
    )
    br.add_argument("--dir", help="where the helper app lives (created if missing)")
    br.add_argument("command", nargs=argparse.REMAINDER, help="-- <command ...>")
    a = ap.parse_args(argv)
    if a.cmd == "ports":
        return _list_ports(a.ble)
    if a.cmd == "macos-ble-app":
        app = macos.build_bundle(a.dir)
        example = ["uv", "run", "flybrain-bittle", "probe", "--ble"]
        if shutil.which("uv"):
            example = macos.resolve_command(example)
        state = app.parent / "last-run"
        state.mkdir(exist_ok=True)
        line = macos.open_command(app, example, os.getcwd(), state, os.environ.get("PATH", ""))
        print(f"created {app}  (bundle id {macos.BUNDLE_ID})")
        print("Easiest: let flybrain-bittle start it and show the output here:")
        print("    uv run flybrain-bittle ble-run -- uv run flybrain-bittle probe --ble")
        print("Or by hand (output goes to the two log files; the first run shows the macOS")
        print("Bluetooth permission prompt):")
        print("    " + shlex.join(line))
        return 0
    if a.cmd == "ble-run":
        cmd = a.command[1:] if a.command[:1] == ["--"] else a.command
        if not cmd:
            ap.error("ble-run needs a command after --")
        try:
            return macos.ble_run(cmd, a.dir)
        except FileNotFoundError as e:
            print(f"flybrain-bittle: {e}", file=sys.stderr)
            return 127
    t = _transport_from_args(a)
    if isinstance(t, SerialTransport) and a.no_reset:
        t.reset_on_open = False
    pins = None
    if a.analog:
        try:
            pins = [int(x) for x in a.analog.replace(" ", "").split(",") if x]
            P.encode_analog_read(pins)
        except ValueError as e:
            ap.error(f"--analog: {e}")
    return Probe(t).run(
        stream_s=a.seconds,
        rotate_test=a.rotate_test,
        analog_pins=pins,
        analog_s=a.seconds,
        side_test=a.side_test,
    )


if __name__ == "__main__":
    sys.exit(main())
