"""Fakes for the robot-mode tests (not shipped). No hardware is touched.

``FakeRobot`` is a robot link: it records every command written, answers like the firmware
(echoes, the identify reply), can drop chosen echoes, refuse to start the camera, drop the link,
and lets a test feed camera lines. ``FakeCamera`` is the camera's own USB serial port (SSCMA AT
protocol); ``FakePorts`` hands them out by device name."""

import json
import queue
import threading
import time

from flybrain_edge.bittle import TransportError
from flybrain_edge.bittle.transport import Transport

FACE_LEFT = "-72.00\t-16.00\tsize = 69\t76\t\r\n"  # measured: a face on the dog's left


class FakeEnLines:
    """The DTR and RTS lines of a serial port; an RTS pulse restarts the fake board."""

    def __init__(self, robot):
        self.robot = robot
        self.log: list[tuple[str, bool]] = []
        self._rts = False

    @property
    def dtr(self):
        return False

    @dtr.setter
    def dtr(self, v):
        self.log.append(("dtr", bool(v)))

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, v):
        self.log.append(("rts", bool(v)))
        if self._rts and not v:  # end of the EN pulse: the board boots
            self.robot.boots += 1
            self.robot.feed("ets Jun  8 2016\r\nboot noise\r\nReady!\r\n")
        self._rts = bool(v)


class FakeRobot(Transport):
    description = "fake robot (test)"

    def __init__(
        self,
        camera="ok",  # ok | silent (no XCr~ echo, still starts) | fail (prints Fail to start)
        name="usb",
        face_on_start=None,
        lose_after_heads=None,
        drop_echo=(),  # commands whose echo never arrives, e.g. ("XCP~", "Xc~")
        mute_from=None,  # from this command on, the robot answers nothing at all
        serial_lines=False,  # give it DTR/RTS lines (a USB serial port)
    ):
        self.name = name
        self.camera = camera
        self.face_on_start = face_on_start  # a camera line sent right after XCP~
        self.lose_after_heads = lose_after_heads  # drop the link after this many head commands
        self.drop_echo = set(drop_echo)
        self.mute_from = mute_from
        self.muted = False
        self.writes: list[bytes] = []
        self._out: queue.Queue = queue.Queue()
        self._open = False
        self.lost = False
        self.opens = 0
        self.boots = 0
        if serial_lines:
            self._ser = FakeEnLines(self)

    # ---- Transport -------------------------------------------------------------------
    def open(self):
        if self.lost:
            raise TransportError("fake robot gone")
        self._open = True
        self.opens += 1
        self._out.put(b"Ready!\r\n")

    @property
    def is_open(self):
        return self._open

    def write(self, data):
        if self.lost or not self._open:
            raise TransportError("fake link lost")
        self.writes.append(bytes(data))
        cmd = bytes(data).decode("latin-1").strip()
        if cmd == self.mute_from:
            self.muted = True
        if not self.muted:
            self._reply(cmd)
        heads = sum(c.startswith(b"i") for c in self.writes)
        if self.lose_after_heads is not None and heads >= self.lose_after_heads:
            self.lost = True

    def read(self, timeout=0.05):
        if self.lost:
            raise TransportError("fake link lost")
        try:
            return self._out.get(timeout=timeout)
        except queue.Empty:
            return b""

    def close(self):
        self._open = False

    # ---- the firmware's answers ------------------------------------------------------
    def _say(self, text):
        self._out.put(text.encode("latin-1"))

    def _echo(self, cmd, token):
        if cmd not in self.drop_echo:
            self._say(token + "\r\n")

    def _reply(self, cmd):
        if cmd == "?":
            self._say("Bittle X\r\nB10_251121\r\n?\r\n")
        elif cmd.startswith("X"):
            code = cmd[1:].rstrip("~")
            if code == "Cr":
                if self.camera == "fail":
                    self._say("+ enable\tCamera\r\nFail to start\r\n")
                    self._echo(cmd, "X")
                    return
                self._say("+ enable\tCamera\r\nsit\r\n")
                if self.camera != "silent":
                    self._echo(cmd, "X")
                self._say("Camera task activated.\r\n")
                return
            self._echo(cmd, "X")
            if code == "CP" and self.face_on_start:
                self._say(self.face_on_start)
        elif cmd.startswith("i"):
            self._echo(cmd, "i")
        elif cmd.startswith("k"):
            self._say(cmd[1:] + "\r\n")
            self._echo(cmd, "k")
        elif cmd.startswith("g"):
            self._echo(cmd, "g")
        elif cmd == "d":
            self._echo(cmd, "d")

    # ---- for tests -----------------------------------------------------------------------
    def feed(self, text):
        self._say(text)

    def lose(self):
        self.lost = True

    def commands(self):
        return [w.decode("latin-1").strip() for w in self.writes]

    def non_head(self):
        """Commands other than head-joint commands."""
        return [c for c in self.commands() if not c.startswith("i")]


def factory(*robots):
    """make_transport for cli.main: hands out the given fakes in order (a reconnect gets the
    next one); raises like a missing robot when they run out."""
    left = list(robots)

    def make():
        if not left:
            raise TransportError("no robot found")
        return left.pop(0)

    return make


# ---- the camera on its own USB port ----------------------------------------------------------
def result(boxes):
    """One SSCMA INVOKE result line."""
    return (
        json.dumps({"type": 1, "name": "INVOKE", "code": 0, "data": {"boxes": boxes}}) + "\r\n"
    ).encode()


class FakeCamera:
    """A pyserial-like port: boot text and INIT@STAT when opened, NAME? answered, and while
    invoking, ``frames`` (a list of box lists, the last one repeated) sent every ``period``."""

    def __init__(self, port, name="Grove Vision AI V2", frames=((),), period=0.03):
        self.port = port
        self.name = name
        self.frames = [list(f) for f in frames]
        self.period = period
        self.writes: list[bytes] = []
        self._out = bytearray()
        self._lock = threading.Lock()
        self.closed = False
        self.invoking = False
        self._next = 0.0
        self.in_waiting = 0
        self._put(b"\x00boot rom noise\r\nSSCMA booting\r\n")
        self._put(b'{"type": 0, "name": "INIT@STAT?", "code": 0, "data": {"boot_count": 3}}\r\n')

    def _put(self, data):
        with self._lock:
            self._out.extend(data)
            self.in_waiting = len(self._out)

    def write(self, data):
        if self.closed:
            raise OSError("port closed")
        self.writes.append(bytes(data))
        if data == b"AT+NAME?\r":
            self._put(
                json.dumps({"type": 0, "name": "NAME?", "code": 0, "data": self.name}).encode()
                + b"\r\n"
            )
        elif data.startswith(b"AT+INVOKE"):
            self._put(b'{"type": 0, "name": "INVOKE", "code": 0, "data": {}}\r\n')
            self.invoking = True
        elif data == b"AT+BREAK\r":
            self.invoking = False

    def read(self, n=1):
        if self.closed:
            raise OSError("port closed")
        now = time.monotonic()
        if self.invoking and now >= self._next:
            self._next = now + self.period
            boxes = self.frames.pop(0) if len(self.frames) > 1 else self.frames[0]
            self._put(result(boxes))
        with self._lock:
            if not self._out:
                pass
            else:
                out = bytes(self._out[:n])
                del self._out[:n]
                self.in_waiting = len(self._out)
                return out
        time.sleep(0.01)
        return b""

    def close(self):
        self.closed = True

    def commands(self):
        return [w.decode().strip() for w in self.writes]


class FakePorts:
    """Stands in for usbcam.open_serial and usbcam.list_ch343_ports."""

    def __init__(self, **devices):
        self.devices = devices  # port name -> FakeCamera factory (or None: not a camera)
        self.opened: list[str] = []
        self.cameras: list[FakeCamera] = []

    def list(self):
        return list(self.devices)

    def open(self, port, baud=921600):
        assert baud == 921600
        self.opened.append(port)
        make = self.devices.get(port)
        if make is None:
            raise OSError(f"{port}: no such device")
        cam = make(port)
        self.cameras.append(cam)
        return cam

    def install(self, monkeypatch):
        from fly_pursuit import usbcam

        monkeypatch.setattr(usbcam, "open_serial", self.open)
        monkeypatch.setattr(usbcam, "list_ch343_ports", self.list)
        monkeypatch.setattr(usbcam, "BOOT_WAIT_S", 0.3)
        monkeypatch.setattr(usbcam, "NAME_WAIT_S", 0.3)
        return self
