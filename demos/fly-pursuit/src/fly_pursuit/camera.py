"""The camera as a bearing: positions, timed, corrected for the head's own turn, and the rule
for "no target". Pure (no threads, no robot): a reader is given the camera's output as it
arrives, ``input(now)`` says what the controller gets.

Two sources (both measured 2026-11-16, packages/flybrain-edge/NOTES.md):

* ``CameraReader``: through the robot. After ``XCP~`` the robot's firmware prints

      <x - 120>\\t<y - 120>\\tsize = <w>\\t<h>\\t

  only when x or y changes, as often as every camera frame (about 68 ms). The frame is 240 x
  240 pixels; NEGATIVE x is on the DOG'S LEFT. A still target prints nothing and there is no
  "lost" message. ``-121.00\\t-121.00\\tsize = 0\\t0`` (the camera found nothing) is taken to
  mean "nothing seen".
* ``usbcam.UsbCameraReader``: the camera on its own USB cable, one result per frame, with a
  score and an empty list when nothing is seen (see usbcam.py).
* ``ModuleReader``: through the robot, read by our firmware module ``P``
  (firmware/bittle/fly_pursuit, its README has the line grammar): one ``@FP <seq> ...;`` line
  per camera frame, ``@FP <seq> -;`` when nothing is seen, with a score, plus status, camera and
  error lines. x is in pixels from the image centre, POSITIVE x on the DOG'S RIGHT (so a face
  on the left starts with a minus sign: ``@FP 300 -30 5 69 76 80;``).

Efference copy: the camera turns with the head. Every position is stored with the head's pan
target at the moment it arrived; when the head has turned by D since, the target's bearing has
changed by -D, so the controller gets ``bearing - (pan_now - pan_then)``. The staleness and
centre-band rules apply to that corrected bearing.

Angles in radians, positive = left, as in the controllers.
"""

from __future__ import annotations

import math
import re
import threading
from collections import deque
from dataclasses import dataclass

from flybrain_edge.bittle.protocol import LineSplitter

FRAME_PX = 240  # the camera's image is 240 x 240 pixels
DEFAULT_FOV_DEG = 60.0  # NOT properly measured (one head step gave about 57): field of view across
STALE_S = 1.0  # no position for longer than this: the target has not moved, or it is gone
CENTRE_DEG = 8.0  # a stale bearing within this of straight ahead is "still there, centred"
HOLD_AFTER_S = 3.0  # lost for this long: the head stops where it is

# Anywhere in a line, so that a print glued to another one (Bluetooth) is still found.
LINE_RE = re.compile(r"(-?\d+(?:\.\d+)?)\t(-?\d+(?:\.\d+)?)\tsize = (\d+)\t(\d+)")
_FRAGMENT_RE = re.compile(r"^[-0-9.\t sizeSIZE=]+$")  # a piece of a camera line, nothing else
_MAX_FRAGMENT = 80

STATES = ("waiting", "seen", "centred", "lost")


def bearing_rad(x_from_centre: float, fov_deg: float = DEFAULT_FOV_DEG) -> float:
    """Bearing of a camera x (pixels from the image centre, negative = dog's left) relative
    to the camera, in radians, positive = left."""
    return -float(x_from_centre) / FRAME_PX * math.radians(fov_deg)


@dataclass(frozen=True)
class CameraLine:
    """One position from the camera."""

    t: float  # host time when it arrived
    x: float  # pixels from the image centre (negative = dog's left)
    y: float
    width: int
    height: int
    bearing: float | None  # rad, + = left, relative to the camera then; None = nothing seen
    pan: float = 0.0  # rad, the head's pan target when it arrived
    score: float | None = None  # detection score (own USB cable only)


def parse_line(text: str) -> tuple[float, float, int, int] | None:
    """(x, y, width, height) from one camera print, or None if ``text`` holds none."""
    m = LINE_RE.search(text)
    if not m:
        return None
    return float(m.group(1)), float(m.group(2)), int(m.group(3)), int(m.group(4))


@dataclass(frozen=True)
class CameraInput:
    """What the controller gets this tick, and why."""

    bearing: float | None  # rad, + = left, corrected for the head's turn; None = no target
    state: str  # waiting | seen | centred | lost
    age_s: float | None  # seconds since the last position (None: none yet)
    lost_s: float = 0.0  # seconds since the target counts as lost
    raw: float | None = None  # rad: the bearing as the camera measured it, uncorrected
    score: float | None = None


def corrected_bearing(raw: float, pan_then: float, pan_now: float) -> float:
    """The head turned by ``pan_now - pan_then`` since the camera measured ``raw``."""
    return raw - (pan_now - pan_then)


def camera_input(
    now: float,
    last: CameraLine | None,
    stale_s: float = STALE_S,
    centre_deg: float = CENTRE_DEG,
    pan_now: float = 0.0,
    steady: bool = False,
    lost_since: float | None = None,
) -> CameraInput:
    """The staleness rule, on the corrected bearing.

    ``steady=False`` (through the robot): positions arrive only when the target moves, so the
    last one is kept. If none arrived for more than ``stale_s`` AND the corrected bearing is
    more than ``centre_deg`` off centre, the target is lost; within the centre band it is kept
    (a still, centred face is silent).
    ``steady=True`` (own USB cable): a result comes every frame and says when nothing is seen,
    so a position older than ``stale_s`` means the camera stopped: lost, whatever the bearing.
    ``lost_since``: host time of the first "nothing seen" after the last face (a steady source
    says "nothing" every frame, so the time lost is counted from the first of them).
    """
    if last is None:
        return CameraInput(None, "waiting", None)
    age = max(0.0, now - last.t)
    if last.bearing is None:  # the camera said it sees nothing
        since = last.t if lost_since is None else min(lost_since, last.t)
        return CameraInput(None, "lost", age, max(0.0, now - since), score=last.score)
    b = corrected_bearing(last.bearing, last.pan, pan_now)
    raw, score = last.bearing, last.score
    if age <= stale_s:
        return CameraInput(b, "seen", age, raw=raw, score=score)
    if not steady and abs(math.degrees(b)) <= centre_deg:
        return CameraInput(b, "centred", age, raw=raw, score=score)
    return CameraInput(None, "lost", age, age - stale_s, raw=raw, score=score)


class PositionStore:
    """The last camera position and the rule above. ``pan`` is set by the head loop to the
    current pan target; every new position is stored with it. Positions arrive on another
    thread (the link's reader); the last one is replaced as a whole."""

    source = "?"
    steady = False
    from_robot = False  # fed from the robot's output (attached to the link as a listener)

    def __init__(
        self,
        fov_deg: float = DEFAULT_FOV_DEG,
        stale_s: float = STALE_S,
        centre_deg: float = CENTRE_DEG,
    ):
        if not 0 < fov_deg < 180:
            raise ValueError("the camera field of view must be between 0 and 180 degrees")
        self.fov_deg = float(fov_deg)
        self.stale_s = stale_s
        self.centre_deg = centre_deg
        self.last: CameraLine | None = None
        self.lines = 0  # positions received (including "nothing seen")
        self.pan = 0.0  # rad: the head's pan target now
        self.lost_since: float | None = None  # first "nothing seen" after the last face

    def record(self, t, x, y, w, h, bearing, score=None) -> None:
        if bearing is None:
            if self.last is None or self.last.bearing is not None:
                self.lost_since = t
        else:
            self.lost_since = None
        self.last = CameraLine(t, x, y, w, h, bearing, self.pan, score)
        self.lines += 1

    def input(self, now: float, pan_now: float | None = None) -> CameraInput:
        pan = self.pan if pan_now is None else pan_now
        return camera_input(
            now, self.last, self.stale_s, self.centre_deg, pan, self.steady, self.lost_since
        )


class CameraReader(PositionStore):
    """Camera lines from the robot's own output (``feed`` is a ``Bittle`` rx listener)."""

    source = "robot"
    from_robot = True

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self._splitter = LineSplitter()
        self._fragment = ""

    def feed(self, data: bytes, t: float) -> None:
        """A chunk of the robot's output, received at host time ``t``."""
        for text in self._splitter.feed(data):
            self.feed_line(text, t)

    def feed_line(self, text: str, t: float) -> None:
        got = parse_line(text)
        if got is None and _FRAGMENT_RE.match(text):
            # a print split over several Bluetooth notifications: join the pieces
            self._fragment = (self._fragment + text)[-_MAX_FRAGMENT:]
            got = parse_line(self._fragment)
        if got is None:
            if not _FRAGMENT_RE.match(text):
                self._fragment = ""
            return
        self._fragment = ""
        x, y, w, h = got
        nothing = w == 0 or abs(x) > FRAME_PX / 2  # -121: the camera found nothing
        self.record(t, x, y, w, h, None if nothing else bearing_rad(x, self.fov_deg))


# ---- our firmware module P (firmware/bittle/fly_pursuit) --------------------------------------
# Every message is "@FP <fields>;" (README there, "Host lines"). The ';' ends a message even when
# Bluetooth drops or moves the newline, so messages are taken out of the raw text stream, across
# read boundaries and with several per read.
FP_MESSAGE_RE = re.compile(r"@FP ([^;@]{1,120});")
_FP_MAX_TAIL = 140  # keep at most this much of an unfinished message
FP_ERROR_KINDS = (
    "i2c",
    "lock",
    "stall",
    "overflow",
    "cmd",
    "nocam",
    "silent",
    "recover",
    "busheld",
)
# The error kinds that need the user (README, "Error kinds"); the others the module handles.
FP_ATTENTION = {
    "silent": (
        "the camera is on the I2C socket but its AI side is not answering (@FP err silent): "
        "power-cycle the camera: switch the robot off and on (or unplug the camera and plug it "
        "back in)"
    ),
    "busheld": (
        "the camera's I2C bus is still held low after a recovery (@FP err busheld): "
        "power-cycle the camera: switch the robot off and on"
    ),
    "nocam": (
        "nothing answers on the I2C socket (@FP err nocam): the camera is not plugged into the "
        "Grove socket labelled SCL SDA +5V GND (not the one by TX2/RX2)"
    ),
}
SEQ_WRAP = 65536
SEQ_MAX_GAP = 1000  # a larger jump is taken as a restart of the board, not as lost lines


@dataclass(frozen=True)
class FpMessage:
    """One message of the module. ``kind``: "result", "status", "cam", "err", "boot" or
    "auto" (the saved "follow after power-on" setting: ``autostart`` True/False after
    "auto 1" / "auto 0", ``started`` after "auto start", printed when it fired at boot)."""

    kind: str
    seq: int | None = None  # result
    box: tuple[int, int, int, int, int] | None = None  # result: x, y, w, h, score; None = no face
    bearing_deg: float | None = None  # b=: bearing given to the on-board circuit (+ = left)
    turn_deg_s: float | None = None  # t=: on-board head turn command after its limit
    pan_deg: float | None = None  # p=: on-board pan target (+ = left)
    hold: bool = False  # on-board: no face for 3 s, the head holds
    on: bool | None = None  # status: module on
    lines: bool | None = None  # status: result lines on (l=)
    control: bool | None = None  # status: on-board control on (b=)
    running: bool | None = None  # cam: True after "cam run", False after "cam stop"
    starts: int | None = None  # cam run <n>
    what: str | None = None  # err <what> <count>
    count: int | None = None
    autostart: bool | None = None  # auto 0 / auto 1
    started: bool = False  # auto start

    @property
    def has_control(self) -> bool:
        """A result printed with on-board control on (it carries t= and p=)."""
        return self.turn_deg_s is not None


def _finite(text: str) -> float:
    v = float(text)
    if not math.isfinite(v):
        raise ValueError(text)
    return v


def _flags(words: list[str]) -> dict[str, str] | None:
    kv = {}
    for x in words:
        if "=" not in x:
            return None
        k, v = x.split("=", 1)
        kv[k] = v
    return kv


def parse_fp(body: str) -> FpMessage | None:
    """The part between "@FP " and ";" -> a message, or None if it is not one we know.
    Tokenised: a face on the dog's left has a negative x ("300 -30 5 69 76 80") and "no face"
    is a lone "-" token ("301 -")."""
    w = body.split()
    if not w:
        return None
    head = w[0]
    try:
        if head == "on":
            kv = _flags(w[1:])
            if kv is None or kv.get("l") not in ("0", "1") or kv.get("b") not in ("0", "1"):
                return None
            return FpMessage("status", on=True, lines=kv["l"] == "1", control=kv["b"] == "1")
        if head == "off":
            return (
                FpMessage("status", on=False, lines=False, control=False) if len(w) == 1 else None
            )
        if head == "cam":
            if w[1:] == ["stop"]:
                return FpMessage("cam", running=False)
            if len(w) == 3 and w[1] == "run" and w[2].isdigit():
                return FpMessage("cam", running=True, starts=int(w[2]))
            return None
        if head == "err":
            if len(w) == 3 and w[1].isalnum() and w[1].islower() and w[2].isdigit():
                return FpMessage("err", what=w[1], count=int(w[2]))
            return None
        if head == "auto":  # newer builds only: "auto 0", "auto 1", "auto start"
            if w[1:] == ["start"]:
                return FpMessage("auto", started=True)
            if len(w) == 2 and w[1] in ("0", "1"):
                return FpMessage("auto", autostart=w[1] == "1")
            return None
        if head == "boot":
            return FpMessage("boot")
        if not head.isdigit() or int(head) >= SEQ_WRAP or len(w) < 2:
            return None
        seq = int(head)
        if w[1] == "-":
            box, rest = None, w[2:]
        else:
            if len(w) < 6:
                return None
            x, y, bw, bh, score = (int(v) for v in w[1:6])  # ValueError on anything else
            if bw < 0 or bh < 0 or not 0 <= score <= 100:
                return None
            box, rest = (x, y, bw, bh, score), w[6:]
        hold = bool(rest) and rest[-1] == "hold"
        if hold:
            rest = rest[:-1]
        kv = _flags(rest)
        if kv is None or set(kv) - {"b", "t", "p"}:
            return None
        control_ok = "t" in kv and "p" in kv and ("b" not in kv or box is not None)
        if (kv or hold) and not control_ok:  # control = [ b= ] t= p= [ hold ]
            return None
        return FpMessage(
            "result",
            seq=seq,
            box=box,
            bearing_deg=_finite(kv["b"]) if "b" in kv else None,
            turn_deg_s=_finite(kv["t"]) if "t" in kv else None,
            pan_deg=_finite(kv["p"]) if "p" in kv else None,
            hold=hold,
        )
    except ValueError:
        return None


class FpStream:
    """Pull "@FP ...;" messages out of the robot's output, fed in any pieces."""

    def __init__(self) -> None:
        self._buf = ""

    def feed(self, text: str) -> list[FpMessage]:
        buf = self._buf + text
        out, end = [], 0
        for m in FP_MESSAGE_RE.finditer(buf):
            msg = parse_fp(m.group(1))
            if msg is not None:
                out.append(msg)
            end = m.end()
        tail = buf[end:]
        at = tail.rfind("@")
        # keep only a possible start of a message ("@", "@F", "@FP 12 -3"); drop the rest
        self._buf = tail[at:] if at >= 0 and len(tail) - at <= _FP_MAX_TAIL else ""
        return out


class ModuleReader(PositionStore):
    """Positions and health from our firmware module (``feed`` is a ``Bittle`` rx listener).

    A result comes every camera frame and says when no face is seen, so "no face" is lost at
    once and no line for ``stale_s`` is lost too (steady). Results printed with on-board
    control on are also queued in ``control`` for the on-board mode's display."""

    source = "module"
    steady = True
    from_robot = True

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self._stream = FpStream()
        self._lock = threading.Lock()
        self.status_count = 0  # "@FP on/off" lines seen: the module is in the firmware
        self.module_on: bool | None = None
        self.lines_on: bool | None = None
        self.control_on: bool | None = None
        self.cam_running: bool | None = None
        self.cam_starts: int | None = None
        self.errors: dict[str, int] = {}
        # The saved "follow after power-on" setting ("@FP auto 0/1;", newer builds only): None
        # until the robot reports it (an older build never does).
        self.autostart: bool | None = None
        self.auto_count = 0  # "@FP auto 0/1" lines seen
        self.autostarted = False  # "@FP auto start;" seen: it fired at boot in this session
        self.results = 0  # result lines
        self.faces = 0  # result lines with a face
        self.missing = 0  # result lines missing between consecutive seq numbers
        self.last_seq: int | None = None
        self.problem: str | None = None  # an FP_ATTENTION kind, until the camera answers again
        self.last_control: FpMessage | None = None
        self.control: deque[tuple[float, FpMessage]] = deque(maxlen=200)

    def feed(self, data: bytes, t: float) -> None:
        """A chunk of the robot's output, received at host time ``t``."""
        for msg in self._stream.feed(data.decode("latin-1")):
            self.handle(msg, t)

    def handle(self, msg: FpMessage, t: float) -> None:
        with self._lock:
            if msg.kind == "status":
                self.status_count += 1
                self.module_on, self.lines_on, self.control_on = msg.on, msg.lines, msg.control
                if not msg.on:
                    self.cam_running = False
            elif msg.kind == "cam":
                self.cam_running = msg.running
                if msg.running:
                    self.cam_starts = msg.starts
                    self.problem = None
            elif msg.kind == "err":
                self.errors[msg.what] = msg.count
                if msg.what in FP_ATTENTION:
                    self.problem = msg.what
            elif msg.kind == "auto":
                if msg.started:
                    self.autostarted = True
                    self.autostart = True  # it fired, so it is saved on
                else:
                    self.auto_count += 1
                    self.autostart = msg.autostart
            elif msg.kind == "result":
                self._result(msg, t)

    def _result(self, msg: FpMessage, t: float) -> None:
        if self.last_seq is not None:
            gap = (msg.seq - self.last_seq) % SEQ_WRAP - 1
            if 0 < gap <= SEQ_MAX_GAP:
                self.missing += gap
        self.last_seq = msg.seq
        self.results += 1
        self.problem = None  # results arrive: the camera works
        if msg.box is None:
            self.record(t, 0.0, 0.0, 0, 0, None)
        else:
            x, y, w, h, score = msg.box
            self.faces += 1
            self.record(t, float(x), float(y), w, h, bearing_rad(x, self.fov_deg), float(score))
        if msg.has_control:
            self.last_control = msg
            self.control.append((t, msg))

    def take_control(self) -> list[tuple[float, FpMessage]]:
        """On-board results received since the last call, oldest first."""
        out = []
        while self.control:
            out.append(self.control.popleft())
        return out

    def summary(self) -> dict:
        """The module's health, for the frame."""
        with self._lock:
            return {
                "present": self.status_count > 0,
                "on": self.module_on,
                "lines_on": self.lines_on,
                "control_on": self.control_on,
                "camera": None
                if self.cam_running is None
                else ("running" if self.cam_running else "stopped"),
                "starts": self.cam_starts,
                "errors": dict(self.errors),
                "results": self.results,
                "faces": self.faces,
                "missing": self.missing,
                "last_seq": self.last_seq,
                "problem": self.problem,
                "autostart": self.autostart,  # saved "follow after power-on"; None: not reported
                "autostarted": self.autostarted,  # it fired at boot in this session
            }
