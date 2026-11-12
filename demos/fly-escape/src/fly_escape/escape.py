"""The escape action: on a trigger from the driver model, back away, then stand again.

Skills that exist for Bittle X+Arm (src/InstinctBittleESP_arm.h, with ROBOT_ARM the loader
first tries the "Arm" variant of a gait, src/skill.h:465-480):
  * ``bkF`` (and plain ``bk``, which the firmware maps to ``bkF``): walk backwards; with the arm
    the firmware loads ``bkArmF`` (25 frames per cycle).
  * ``bkL`` / ``bkR``: walk backwards while turning (``bkArmL``, ``R`` is the mirror).
  * Other gaits (``wk``, ``tr``, ``cr``, ``vt``) walk forwards or turn in place; behaviours such as
    ``jmp``, ``lnd``, ``rc`` are violent and are not offered.

Actions (``--escape-action``):
  * ``back`` (default): ``kbkF`` for ESCAPE_ACTION_S, then ``kup``. Straight back, the clearest
    "move away" and the gentlest gait in the table.
  * ``back-turn``: ``kbkR`` for a threat on the left, ``kbkL`` on the right (swap with
    ``--turn-sign -1``), then ``kup``. Which way bkL actually rotates the body is not verified.

Sequence: pause the analog reads (an 'R' read stops a running gait: src/reaction.h:1064-1131
leaves the token unset), send the gait, wait ESCAPE_ACTION_S, send ``kup`` and wait for its echo
(the link sends commands one at a time and never on top of an unconfirmed one), resume reading.
If the stand is not confirmed it is re-sent; after STAND_RETRIES the robot is rested and the
demo drops to observe mode. Backing barely changes the heading, so there is no heading-based
stop check here (unlike fly-compass); the IMU is optional in this demo.

Safety net on top of the model's own refractory period: at least MIN_INTERVAL_S between the
starts of two escapes, and at most MAX_PER_MINUTE escapes in any 60 s.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any, Protocol

ESCAPE_ACTION_S = 1.0
MIN_INTERVAL_S = 3.0
MAX_PER_MINUTE = 6
STAND_TIMEOUT_S = 4.0
STAND_RETRIES = 2

ACTIONS = ("back", "back-turn")


def action_skill(action: str, side: int, turn_sign: int = 1) -> str:
    """Firmware skill for an action and the threat side (+1 left, -1 right, 0 unknown)."""
    if action == "back" or side == 0:
        return "bkF"
    if action == "back-turn":
        away = -side * turn_sign  # threat on the left (+1): turn right (-1)
        return "bkL" if away > 0 else "bkR"
    raise ValueError(f"unknown action {action!r}")


class Job(Protocol):
    done: Any  # threading.Event-like: is_set()
    result: Any  # None = no echo


class Robot(Protocol):
    def send_skill(self, name: str) -> Job: ...
    def send_rest(self) -> Job: ...
    def pause_sensing(self, paused: bool) -> None: ...


@dataclass
class EscapeConfig:
    action: str = "back"
    action_s: float = ESCAPE_ACTION_S
    min_interval_s: float = MIN_INTERVAL_S
    max_per_minute: int = MAX_PER_MINUTE
    stand_timeout_s: float = STAND_TIMEOUT_S
    stand_retries: int = STAND_RETRIES
    turn_sign: int = 1


@dataclass
class EscapeRecord:
    t: float
    side: int
    skill: str
    stood: bool | None = None


@dataclass
class EscapeController:
    robot: Robot
    cfg: EscapeConfig = field(default_factory=EscapeConfig)
    state: str = "idle"  # idle | acting | standing | failed
    since: float = 0.0
    stand_job: Any = None
    stand_tries: int = 0
    starts: deque = field(default_factory=lambda: deque(maxlen=100))
    history: list[EscapeRecord] = field(default_factory=list)
    blocked: int = 0
    last_block: str | None = None
    events: list[str] = field(default_factory=list)

    @property
    def busy(self) -> bool:
        return self.state in ("acting", "standing")

    def allowed(self, now: float) -> str | None:
        """None if an escape may start now, else the reason it may not."""
        if self.state == "failed":
            return "a previous stand was not confirmed: escape is disabled"
        if self.busy:
            return "already escaping"
        if self.starts and now - self.starts[-1] < self.cfg.min_interval_s:
            return f"less than {self.cfg.min_interval_s:.0f} s since the last escape"
        recent = [t for t in self.starts if now - t < 60.0]
        if len(recent) >= self.cfg.max_per_minute:
            return f"already {self.cfg.max_per_minute} escapes in the last minute"
        return None

    def trigger(self, now: float, side: int) -> bool:
        """The driver model fired. Returns True if an escape started."""
        why = self.allowed(now)
        if why is not None:
            self.blocked += 1
            self.last_block = why
            self.events.append(f"escape trigger ignored: {why}")
            return False
        skill = action_skill(self.cfg.action, side, self.cfg.turn_sign)
        self.robot.pause_sensing(True)  # waits for a read in flight; no 'R' during the gait
        self.robot.send_skill(skill)
        self.starts.append(now)
        self.history.append(EscapeRecord(now, side, skill))
        self.state, self.since = "acting", now
        self.events.append(
            f"ESCAPE: {skill} (threat {'left' if side > 0 else 'right' if side < 0 else 'ahead'})"
        )
        return True

    def update(self, now: float) -> str | None:
        """Advance; returns "observe" if the demo must drop to observe mode."""
        if self.state == "acting" and now - self.since >= self.cfg.action_s:
            self._stand(now)
        elif self.state == "standing":
            job = self.stand_job
            if job is not None and job.done.is_set():
                if job.result is not None:
                    self.history[-1].stood = True
                    self.robot.pause_sensing(False)
                    self.state, self.since = "idle", now
                    return None
                return self._stand_failed(now, "no echo")
            if now - self.since > self.cfg.stand_timeout_s + 2.0:
                return self._stand_failed(now, "no reply")
        return None

    def abort(self, now: float) -> None:
        """Mode switch or shutdown mid-escape: stand now."""
        if self.state == "acting":
            self._stand(now)

    def _stand(self, now: float) -> None:
        self.stand_job = self.robot.send_skill("up")
        self.stand_tries = 1 if self.state == "acting" else self.stand_tries + 1
        self.state, self.since = "standing", now

    def _stand_failed(self, now: float, why: str) -> str | None:
        if self.stand_tries <= self.cfg.stand_retries:
            self.events.append(
                f"stand not confirmed ({why}): re-sending (try {self.stand_tries + 1})"
            )
            self._stand(now)
            return None
        self.history[-1].stood = False
        self.robot.send_rest()
        self.robot.pause_sensing(False)
        self.state, self.since = "failed", now
        self.events.append(
            "STAND FAILED: no confirmation after "
            f"{self.stand_tries} tries; sent rest (servos off), switching to observe"
        )
        return "observe"

    def status(self, now: float) -> dict[str, Any]:
        return {
            "state": self.state,
            "time_in_state": round(now - self.since, 1),
            "count": len(self.history),
            "per_minute": sum(1 for t in self.starts if now - t < 60.0),
            "max_per_minute": self.cfg.max_per_minute,
            "min_interval_s": self.cfg.min_interval_s,
            "blocked": self.blocked,
            "last_block": self.last_block,
            "action": self.cfg.action,
            "last": None
            if not self.history
            else {
                "t": self.history[-1].t,
                "skill": self.history[-1].skill,
                "side": self.history[-1].side,
                "stood": self.history[-1].stood,
            },
        }
