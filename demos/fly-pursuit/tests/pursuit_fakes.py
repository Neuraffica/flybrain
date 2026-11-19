"""Fakes for the robot-mode tests (not shipped). No hardware is touched.

``FakeRobot`` is a robot link: it records every command written, answers like the firmware
(echoes, the identify reply, the module list of X?), can drop chosen echoes, refuse to start the
camera, drop the link, and lets a test feed camera lines. With ``module`` set it has our camera
module P (firmware/bittle/fly_pursuit): "@FP" status lines after X? and every X change, "cam run"
after XP~, and "@FP" result lines on demand (``fp_lines`` after XP~, ``onboard_lines`` after
XPB~, or ``feed`` at any time). With ``autostart`` set (True / False) it is the newer build
with the saved "follow after power-on" setting: "@FP auto <0|1>;" after every status line,
XPS~ / XPs~ / Xps~, and a bare XP~ switches on-board control off; ``autostarted`` makes it a
robot that has just started following a face by itself. ``FakeCamera`` is the camera's own USB serial port (SSCMA AT
protocol); ``FakePorts`` hands them out by device name."""

import json
import queue
import threading
import time

from flybrain_edge.bittle import TransportError
from flybrain_edge.bittle.transport import Transport

FACE_LEFT = "-72.00\t-16.00\tsize = 69\t76\t\r\n"  # measured: a face on the dog's left
FP_FACE_LEFT = "@FP 1 -72 -16 69 76 80;\r\n"  # the same face through our module: 18 deg left
STOCK_MODULES = "S,\tA,\tT,\tL,\tD,\tI,\tB,\tU,\tG,\tC,\tQ,\tE,\t"
OUR_MODULES = STOCK_MODULES + "P,\t"


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
        module=None,  # None: stock firmware; "off" / "on": our camera module P, in that state
        fp_lines=(),  # "@FP" lines sent right after XP~
        onboard_lines=(),  # "@FP" lines sent right after XPB~
        status_after_echo=False,  # X?~: the "@FP" status line comes after the X echo
        autostart=None,  # None: older build of the module (no S / s); True / False: newer
        autostarted=False,  # it followed a face by itself after power-on (control on)
        following_lines=(),  # "@FP" result lines with control fields sent after X?~
    ):
        self.autostart = autostart
        self.autostarted = autostarted
        self.following_lines = list(following_lines)
        self.control_at: list[tuple[str, bool]] = []  # (command, on-board control after it)
        self.module = module
        self.fp_lines = list(fp_lines)
        self.onboard_lines = list(onboard_lines)
        self.status_after_echo = status_after_echo
        self.control = False
        self.cam_runs = 0
        if autostarted:  # it beeped, sat and follows a face: module on, control on
            self.module, self.control, self.autostart, self.cam_runs = "on", True, True, 1
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
        if self.autostarted:
            self._say("@FP on l=1 b=1;\r\n@FP auto 1;\r\n@FP auto start;\r\n")

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
        self.control_at.append((cmd, self.control))
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

    def _status(self):
        if self.module == "on":
            self._say(f"@FP on l=1 b={int(self.control)};\r\n")
        elif self.module is not None:
            self._say("@FP off;\r\n")
        if self.module is not None and self.autostart is not None:
            self._say(f"@FP auto {int(self.autostart)};\r\n")

    def _module(self, cmd, code):
        """Our module's commands (README of firmware/bittle/fly_pursuit, "Commands")."""
        if code == "?":
            letters = OUR_MODULES if self.module else STOCK_MODULES
            flags = "".join(("1" if k in "A" else "0") + ",\t" for k in letters[::3])
            self._say(letters + "\r\n" + flags + "\r\n")
            if not self.status_after_echo:
                self._status()
            self._echo(cmd, "X")
            if self.status_after_echo:
                self._status()
            for ln in self.following_lines if self.control else ():
                self._say(ln)
            return True
        if self.module is None:
            return False
        if code.startswith("P"):
            was_on = self.module == "on"
            self.module = "on"
            if not was_on and self.autostart is not None:
                self._status()  # as the robot: the firmware's own switch prints the status
                # first, with the OLD setting; the module's options print it again below
            if self.autostart is not None:  # the newer build; an older one ignores S and s
                if code == "P":
                    self.control = False  # a bare XP~ is passive
                if "S" in code:
                    self.autostart = True
                if "s" in code:
                    self.autostart = False
            if "B" in code:
                self._say("sit\r\n")
                self.control = True
            if "b" in code:
                self.control = False
            self._status()
            self._echo(cmd, "X")
            if not was_on:
                self.cam_runs += 1
                self._say(f"@FP cam run {self.cam_runs};\r\n")
                for ln in self.fp_lines:
                    self._say(ln)
            if "B" in code:
                for ln in self.onboard_lines:
                    self._say(ln)
            return True
        if code in ("p", "ps"):  # an older build takes Xps~ as Xp~
            if self.module == "on":
                self._say("@FP cam stop;\r\n")
            was_on = self.module == "on"
            self.module, self.control = "off", False
            if self.autostart is not None and code == "ps":
                if was_on:
                    self._status()  # as the robot: first the status with the OLD setting
                self.autostart = False
            self._status()
            self._echo(cmd, "X")
            return True
        return False

    def _reply(self, cmd):
        if cmd == "?":
            self._say("Bittle X\r\nB10_251121\r\n?\r\n")
        elif cmd.startswith("X"):
            code = cmd[1:].rstrip("~")
            if self._module(cmd, code):
                return
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
            self._status()  # the firmware prints its status after every X change
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
