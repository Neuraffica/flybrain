"""Simulated approaches with ground truth, for the visual simulator and seeded random runs.

An approach is a hand path ``fn(t) -> (distance_cm, side)`` relative to the dog's nose at the
moment it is launched, plus what it *is*:

  threats      fast   60 -> 3 cm in 0.35-0.6 s, holds, withdraws
               slow   60 -> 3 cm in 0.9-1.5 s, holds, withdraws
  non-threats  hover       comes into the sensor's graded band (15.5-17.5 cm) and stays
               stop-short  lunges in quickly but stops at the band's outer edge (17.5-19.5 cm)
               drift       creeps in over 5-7 s to about 13 cm and goes
               retreat     appears 25-30 cm away and moves off

side is +1 robot's left, -1 right, 0 head-on (the hand comes from 0.8 to the side). The speed
factor (dashboard slider) divides the durations of the moving parts.

The simulated sensor reads its resting level beyond 20 cm and about 0 closer than 12 cm
(flybrain_edge.bittle.mock.ProximityHand), so the band between is all the models get. Distances
for the non-threats are from the channel nearer the hand (a hand to one side is 4 cm closer to
that channel). Whether a lunge that stops at the band's edge "should" cause an escape is a
judgement: here it counts as a non-threat.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from flybrain_edge.bittle.mock import ProximityHand

AWAY_CM = 80.0
START_CM = 60.0
DETECT_CM = ProximityHand.FAR_CM  # the simulated sensor starts to respond here
SPREAD_CM = ProximityHand.SPREAD_CM  # a hand to one side is this much closer to that channel
CONTACT_CM = 6.0

THREAT_KINDS = ("fast", "slow")
SAFE_KINDS = ("hover", "stop-short", "drift", "retreat")
KINDS = THREAT_KINDS + SAFE_KINDS
SIDES = {"left": 1, "right": -1, "head-on": 0}


def _ramp(t: float, t0: float, t1: float, a: float, b: float) -> float:
    if t <= t0:
        return a
    if t >= t1:
        return b
    return a + (b - a) * (t - t0) / (t1 - t0)


@dataclass
class Approach:
    kind: str
    side: int
    move_s: float  # duration of the main movement
    hold_s: float
    target_cm: float  # where the movement ends
    start_cm: float = START_CM
    id: int = 0
    t_launch: float | None = None  # mock clock, set when launched
    threat: bool = field(init=False)

    def __post_init__(self):
        self.threat = self.kind in THREAT_KINDS

    @property
    def lateral(self) -> float:
        return 0.8 * self.side

    @property
    def nearer_cm(self) -> float:
        """How much closer the hand is to the channel on its side than to the sensor centre."""
        return SPREAD_CM * abs(self.lateral)

    @property
    def duration(self) -> float:
        """Until the hand is away again."""
        return self.move_s + self.hold_s + 0.4

    def __call__(self, t: float) -> tuple[float, float]:
        if t < 0 or t > self.duration:
            return AWAY_CM, self.lateral
        if t > self.move_s + self.hold_s:  # withdraw quickly
            return _ramp(
                t, self.move_s + self.hold_s, self.duration, self.target_cm, AWAY_CM
            ), self.lateral
        return _ramp(t, 0.0, self.move_s, self.start_cm, self.target_cm), self.lateral

    def detect_t(self) -> float | None:
        """Seconds after launch when the nearer channel first starts to respond, if ever."""
        t, dt = 0.0, 0.005
        while t <= self.duration:
            if self(t)[0] - self.nearer_cm < DETECT_CM:
                return t
            t += dt
        return None

    def label(self) -> str:
        where = {1: "left", -1: "right", 0: "head-on"}[self.side]
        return f"{self.kind} from the {where}" if self.side else f"{self.kind} head-on"


def make_approach(
    kind: str, side: int, rng: random.Random | None = None, speed: float = 1.0
) -> Approach:
    r = rng or random.Random(0)
    speed = max(0.25, min(4.0, speed))
    # non-threats stop at a distance measured from the channel nearer the hand, so a hand from
    # the side stops as far from "its" channel as a head-on one does from both
    off = SPREAD_CM * 0.8 * abs(side)
    if kind == "fast":
        a = Approach(kind, side, r.uniform(0.35, 0.6), 0.8, 3.0)
    elif kind == "slow":
        a = Approach(kind, side, r.uniform(0.9, 1.5), 0.8, 3.0)
    elif kind == "hover":
        a = Approach(kind, side, 1.0, r.uniform(2.5, 4.0), r.uniform(15.5, 17.5) + off)
    elif kind == "stop-short":
        a = Approach(kind, side, r.uniform(0.4, 0.7), 1.5, r.uniform(17.5, 19.5) + off)
    elif kind == "drift":
        a = Approach(kind, side, r.uniform(5.0, 7.0), 1.0, r.uniform(12.5, 14.0) + off)
    elif kind == "retreat":
        a = Approach(kind, side, r.uniform(1.0, 1.6), 0.0, AWAY_CM, start_cm=r.uniform(25.0, 30.0))
    else:
        raise ValueError(f"unknown kind {kind!r}; choose from {', '.join(KINDS)}")
    a.move_s /= speed
    return a


class RandomApproaches:
    """Seeded stream of approaches: half threats, half not, random side, launched 3-6 s after
    the previous one ended (longer than the app's 3 s minimum between escapes)."""

    def __init__(self, seed: int):
        self.seed = seed
        self.rng = random.Random(seed)
        self.next_at: float | None = None

    def reset(self, seed: int | None = None) -> None:
        if seed is not None:
            self.seed = seed
        self.rng = random.Random(self.seed)
        self.next_at = None

    def draw(self, speed: float = 1.0) -> Approach:
        r = self.rng
        kind = r.choice(THREAT_KINDS) if r.random() < 0.5 else r.choice(SAFE_KINDS)
        side = r.choice((1, 0, -1))
        return make_approach(kind, side, r, speed)

    def gap(self) -> float:
        return self.rng.uniform(3.0, 6.0)
