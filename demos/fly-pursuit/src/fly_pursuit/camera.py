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

Efference copy: the camera turns with the head. Every position is stored with the head's pan
target at the moment it arrived; when the head has turned by D since, the target's bearing has
changed by -D, so the controller gets ``bearing - (pan_now - pan_then)``. The staleness and
centre-band rules apply to that corrected bearing.

Angles in radians, positive = left, as in the controllers.
"""

from __future__ import annotations

import math
import re
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
) -> CameraInput:
    """The staleness rule, on the corrected bearing.

    ``steady=False`` (through the robot): positions arrive only when the target moves, so the
    last one is kept. If none arrived for more than ``stale_s`` AND the corrected bearing is
    more than ``centre_deg`` off centre, the target is lost; within the centre band it is kept
    (a still, centred face is silent).
    ``steady=True`` (own USB cable): a result comes every frame and says when nothing is seen,
    so a position older than ``stale_s`` means the camera stopped: lost, whatever the bearing.
    """
    if last is None:
        return CameraInput(None, "waiting", None)
    age = max(0.0, now - last.t)
    if last.bearing is None:  # the camera said it sees nothing
        return CameraInput(None, "lost", age, age, score=last.score)
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

    def record(self, t, x, y, w, h, bearing, score=None) -> None:
        self.last = CameraLine(t, x, y, w, h, bearing, self.pan, score)
        self.lines += 1

    def input(self, now: float, pan_now: float | None = None) -> CameraInput:
        pan = self.pan if pan_now is None else pan_now
        return camera_input(now, self.last, self.stale_s, self.centre_deg, pan, self.steady)


class CameraReader(PositionStore):
    """Camera lines from the robot's own output (``feed`` is a ``Bittle`` rx listener)."""

    source = "robot"

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
