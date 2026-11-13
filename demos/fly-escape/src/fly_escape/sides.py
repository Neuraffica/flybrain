"""Which sensor pin is the robot's left: a guided check, and what it found for each robot.

Which half of the double IR sensor sits on the robot's left depends on how the sensor is
strapped on, so it differs between robots. The check asks for a hand on the robot's LEFT and
watches both pins (raw readings, in --pins order): the pin whose reading falls well below its
resting level, while the other stays clearly higher, is the left. The answer is stored per robot
in fly-escape-sides.json next to Flydeck's settings (FLY_ESCAPE_SIDES overrides the file).

The side setting is "auto" (default: what the check stored for this robot, else not swapped),
"normal" (first pin is the left) or "swapped" (second pin is the left).
"""

from __future__ import annotations

import json
import os
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from flybrain_demo_api import user_dir
from flybrain_edge.bittle.protocol import ble_stem

SIDE_SETTINGS = ("auto", "normal", "swapped")
ENV = "FLY_ESCAPE_SIDES"
FILE_NAME = "fly-escape-sides.json"
HOW = "press Calibrate sides in the dashboard, or start with --calibrate-sides"

# The decision. The real sensor rests at about 3400 +- 50 and reads about 0 with a hand within
# about 12 cm, graded only over a narrow band, so a hand on one side takes that channel most of
# the way down while the other still reads high; a hand straight ahead or very close takes both
# down together, which tells nothing.
MIN_DROP = 0.5  # the left pin must fall to half its resting level or lower
MIN_MARGIN = 0.3  # ... and fall by at least 30 % of resting level more than the other pin
HOLD_S = 0.3  # ... for this long without a break
HOLD_SAMPLES = 3  # ... and at least this many readings (Bluetooth gives about 10 per second)
TIMEOUT_S = 15.0  # time to bring the hand
RELEASE_DROP = 0.2  # afterwards: both pins back within 20 % of resting level ...
RELEASE_S = 0.5  # ... for this long, before the models run again
RELEASE_MAX_S = 5.0  # or give up waiting for the hand to go after this long

PROMPT = (
    "side check: hold a hand about 10 cm from the robot's LEFT side of the sensor (the robot's "
    "own left: your right if you face it), not straight ahead"
)


def store_path() -> Path:
    env = os.environ.get(ENV)
    return Path(env).expanduser() if env else user_dir() / FILE_NAME


def load_store(path: Path | None = None) -> dict[str, Any]:
    try:
        data = json.loads((path or store_path()).read_text())
    except (OSError, ValueError):
        return {}
    robots = data.get("robots") if isinstance(data, dict) else None
    return robots if isinstance(robots, dict) else {}


def save_result(robot: str, left_pin: int, pins: tuple[int, int], path: Path | None = None) -> str:
    """Remember which pin is this robot's left. Returns an error text, or "" when saved."""
    path = path or store_path()
    robots = load_store(path)
    robots[robot] = {
        "left_pin": int(left_pin),
        "pins": list(pins),
        "at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"robots": robots}, indent=2) + "\n")
        tmp.replace(path)
    except OSError as e:
        return f"could not save {path}: {e}"
    return ""


def resolve(
    setting: str, robot: str, pins: tuple[int, int], path: Path | None = None
) -> tuple[bool, str]:
    """(swap, source): source is "flag", "calibrated" or "default" (auto, nothing stored)."""
    if setting == "normal":
        return False, "flag"
    if setting == "swapped":
        return True, "flag"
    rec = load_store(path).get(robot)
    left = rec.get("left_pin") if isinstance(rec, dict) else None
    if left in pins:
        return left == pins[1], "calibrated"
    return False, "default"


_NAME_RE = re.compile(r"\b([A-Za-z][\w-]*?)_(?:BLE|SSP)\b")


def robot_identity(transport: Any, boot_text: str = "") -> str:
    """A key for this robot that is the same over USB and Bluetooth: its Bluetooth name without
    the _BLE / _SSP suffix (the same robot shows either), e.g. "BittleA7". Over Bluetooth that is
    the advertised name; over USB the boot banner prints it ("BLE:\\tBittleA7_BLE") when opening
    the port reset the board. Otherwise the USB port ("usb:/dev/cu...")."""
    if getattr(transport, "name", "") == "ble":
        nm = getattr(transport, "device_name", None) or getattr(transport, "want_name", None)
        return ble_stem(nm) if nm else "ble:(scan)"
    m = _NAME_RE.search(boot_text or "")
    if m:
        return m.group(1)
    return f"usb:{getattr(transport, 'port', None) or '?'}"


def _drop(raw: float, rest: float) -> float:
    return (rest - raw) / rest if rest > 0 else 0.0


@dataclass
class SideCheck:
    """Feed raw samples (pins in --pins order). Phases: "idle", "hand" (waiting for a hand on
    the left), "release" (waiting for the hand to go), then "idle" again with ``left`` set
    to 0 or 1 (index into the pins) or None when nothing clear was seen."""

    phase: str = "idle"
    rest: tuple[float, float] = (3400.0, 3400.0)
    started: float = 0.0
    left: int | None = None
    message: str = ""
    _run: list = field(default_factory=list)  # (t, side) of the current unbroken clear run
    _seen_both: bool = False
    _seen_any: bool = False
    _release_t: float | None = None
    _phase_t: float = 0.0

    @property
    def active(self) -> bool:
        return self.phase != "idle"

    def start(self, now: float, rest_ab: tuple[float, float]) -> None:
        self.phase, self.rest, self.started, self._phase_t = "hand", rest_ab, now, now
        self.left, self.message = None, PROMPT
        self._run, self._seen_both, self._seen_any, self._release_t = [], False, False, None

    @staticmethod
    def classify(drop_a: float, drop_b: float) -> int | None:
        """0 if pin A clearly responded, 1 if pin B did, None if neither or both."""
        if drop_a >= MIN_DROP and drop_a - drop_b >= MIN_MARGIN:
            return 0
        if drop_b >= MIN_DROP and drop_b - drop_a >= MIN_MARGIN:
            return 1
        return None

    def add(self, t: float, raw_a: float, raw_b: float) -> bool:
        """Feed one sample; True when the check has just finished (back to idle)."""
        if self.phase == "idle":
            return False
        da, db = _drop(raw_a, self.rest[0]), _drop(raw_b, self.rest[1])
        if self.phase == "hand":
            side = self.classify(da, db)
            self._seen_any |= max(da, db) >= MIN_DROP
            self._seen_both |= min(da, db) >= MIN_DROP
            if side is None or (self._run and self._run[0][1] != side):
                self._run = []
            if side is not None:
                self._run.append((t, side))
                if len(self._run) >= HOLD_SAMPLES and t - self._run[0][0] >= HOLD_S:
                    self.left = side
                    self._to_release(t, "side check: got it, take the hand away")
            elif t - self.started >= TIMEOUT_S:
                if self._seen_both:
                    why = (
                        "both channels responded together; bring the hand from the side, "
                        "about 10 cm away, not straight ahead"
                    )
                elif self._seen_any:
                    why = "the response was not clear enough; hold the hand still at one side"
                else:
                    why = f"no hand seen within {TIMEOUT_S:.0f} s"
                self._to_release(t, f"side check FAILED: {why}. Sides unchanged")
            return False
        # release: wait until the hand has gone, so the models do not start on a hand
        if max(da, db) <= RELEASE_DROP:
            if self._release_t is None:
                self._release_t = t
        else:
            self._release_t = None
        away = self._release_t is not None and t - self._release_t >= RELEASE_S
        if away or t - self._phase_t >= RELEASE_MAX_S:
            self.phase = "idle"
            return True
        return False

    def _to_release(self, t: float, message: str) -> None:
        self.phase, self._phase_t, self.message, self._release_t = "release", t, message, None

    def status(self, now: float) -> dict[str, Any]:
        left = max(0.0, TIMEOUT_S - (now - self.started)) if self.phase == "hand" else None
        return {"phase": self.phase, "message": self.message, "seconds_left": left}
