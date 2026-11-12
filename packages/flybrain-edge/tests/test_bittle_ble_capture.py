"""Replay a real BLE capture (Bittle X+Arm, firmware B10_250330) through the real BLE
transport's notification framing, the robot link and the probe.

The fixture is `flybrain-bittle probe --ble BittleC2` output; every `RX` line is one
notification as received, every `TX` line a command the probe sent."""

import ast
import io
import re
import time
from pathlib import Path

import pytest
from flybrain_edge.bittle import Bittle, BleTransport
from flybrain_edge.bittle import protocol as P
from flybrain_edge.bittle.probe import Probe

FIXTURE = Path(__file__).parent / "fixtures" / "ble_probe_b10_250330.txt"
LINE_RE = re.compile(r"^\+\s*[\d.]+s\s+(TX|RX)\s+(b(['\"]).*\3)\s*$")


def load_capture():
    """[(direction, bytes)] in capture order."""
    events = []
    for ln in FIXTURE.read_text().splitlines():
        m = LINE_RE.match(ln)
        if m:
            events.append((m.group(1), ast.literal_eval(m.group(2))))
    return events


EVENTS = load_capture()


def stream_readings() -> int:
    """IMU readings in the notifications that answered 'gP' (the capture is trimmed)."""
    n, in_stream = 0, False
    for d, data in EVENTS:
        if d == "TX":
            in_stream = data == b"gP\n"
        elif in_stream:
            n += len(P.parse_imu(data.decode("latin-1"))[0])
    return n


class CaptureBle(BleTransport):
    """BleTransport whose radio is the capture: open() delivers the greeting, each write of a
    captured command delivers the notifications that followed it. read() and the
    notification framing are the real ones."""

    def __init__(self):
        super().__init__(name="BittleC2")
        self.device_name = "BittleC2_SSP"
        self.writes: list[bytes] = []
        self._replies: dict[bytes, list[list[bytes]]] = {}
        self._greeting: list[bytes] = []
        current = None
        for d, data in EVENTS:
            if d == "TX":
                current = []
                self._replies.setdefault(data, []).append(current)
            elif current is None:
                self._greeting.append(data)
            else:
                current.append(data)

    def open(self):
        self._connected = True
        for chunk in self._greeting:
            self._on_notify(None, chunk)

    def write(self, data):
        self.writes.append(data)
        queue = self._replies.get(data)
        if queue:
            for chunk in queue.pop(0):
                self._on_notify(None, chunk)

    def close(self):
        self._connected = False


def test_fixture_is_present_and_raw():
    assert len(EVENTS) > 200 and stream_readings() > 250
    rx = [d for k, d in EVENTS if k == "RX"]
    assert b"Bittle X+ArmB10_250330?" in rx
    assert not any(b"\n" in d for d in rx)  # this firmware sends no newlines over BLE


def test_framer_turns_notifications_into_lines():
    f = P.NotificationFramer()
    assert f.frame(b"g") == b"g\n"
    assert f.frame(b"0123456789") == b"0123456789"  # 10 bytes: maybe the first piece of a print
    assert f.frame(b"rest\r\n") == b"rest\r\n" and f.newline_mode
    assert f.frame(b"later") == b"later"  # newer firmware: newlines come in the data


def test_split_and_identify_both_framings():
    assert P.split_glued("Bittle X+ArmB10_250330?") == ["Bittle X+Arm", "B10_250330", "?"]
    assert P.split_glued("Bittle XB01_261024") == ["Bittle X", "B01_261024"]
    assert P.split_glued("Petoi Bittle") == ["Petoi Bittle"]
    assert P.split_glued("g") == ["g"]
    ble = ["G", "Bittle X+ArmB10_250330?"]
    usb = ["Bittle X+Arm", "B10_250330", "?"]
    assert P.parse_identify(ble) == ("Bittle X+Arm", "B10_250330")
    assert P.parse_identify(usb) == ("Bittle X+Arm", "B10_250330")
    assert P.parse_identify(["Petoi Bittle", "p"]) == (None, None)


def test_joint_table_both_forms():
    usb = ["=", "0\t1\t2\t3", "0\t60\t-3\t75", "j"]
    assert P.parse_joint_table(usb) == {0: 0, 1: 60, 2: -3, 3: 75}
    ble = ["=", "0\t1\t2\t3\t", "0,\t60,\t0,\t-43,\t", "j"]
    assert P.parse_joint_table(ble) == {0: 0, 1: 60, 2: 0, 3: -43}
    assert P.parse_joint_table(["j"]) is None


def test_robot_link_over_captured_ble():
    t = CaptureBle()
    bot = Bittle(t)
    bot.connect(boot_timeout=3.0, quiet_s=0.3)
    try:
        assert "Petoi Bittle" in bot.boot_text
        assert bot.model == "Bittle X+Arm"
        assert bot.version == "B10_250330"
        assert bot.send(b"g?\n", echo="g", timeout=1.0) is not None
        s = bot.read_yaw()
        assert s is not None and s.source == "ICM" and s.yaw_deg == pytest.approx(6502.4)
        assert bot.send(b"v\n", echo="v", timeout=1.0) == ["Undefined token!", "v"]
        joints = bot.read_joints()
        assert joints is not None and len(joints) == 16
        assert (joints[0], joints[1], joints[8], joints[12]) == (0, 60, 65, -43)
        n0 = bot.stats.imu_samples
        bot.stream_imu(True)
        expected = stream_readings()
        deadline = time.monotonic() + 3
        while bot.stats.imu_samples - n0 < expected and time.monotonic() < deadline:
            time.sleep(0.02)
        # every reading in the capture arrives, including two glued into one notification
        assert bot.stats.imu_samples - n0 == expected
        assert bot.stats.echo_timeouts == 0
        assert bot.low_power is None
    finally:
        bot.close()


def test_probe_verdict_on_captured_ble():
    out = io.StringIO()
    code = Probe(CaptureBle(), out=out).run(stream_s=0.5, boot_s=3.0)
    text = out.getvalue()
    assert "model:    Bittle X+Arm" in text
    assert "version:  B10_250330" in text
    for label in ("identify", "gyro state", "IMU once (OpenCatEsp32", "legacy", "joint angles"):
        line = next(x for x in text.splitlines() if x.startswith("echo for") and label in x)
        assert line.endswith("yes"), line
    assert "joints:   16 parsed" in text
    m = re.search(r"IMU readings during 'stream': (\d+)", text)
    assert m and int(m.group(1)) == stream_readings()
    assert "verdict:  OK" in text
    assert code == 0
