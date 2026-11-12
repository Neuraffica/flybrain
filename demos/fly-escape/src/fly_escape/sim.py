"""The visual simulator's controller: launches approaches (buttons or seeded random), keeps the
ground truth, and reports the simulated dog and hand exactly as the mock robot has them."""

from __future__ import annotations

from typing import Any

from flybrain_edge.bittle import MockBittle

from .approaches import KINDS, Approach, RandomApproaches, make_approach
from .groundtruth import GroundTruth


class Simulator:
    def __init__(self, mock: MockBittle, seed: int | None = None, speed: float = 1.0):
        self.mock = mock
        self.speed = speed
        self.random = RandomApproaches(0 if seed is None else seed)
        self.random_on = seed is not None
        self.paused = False
        self.gt = GroundTruth()
        self.current: Approach | None = None
        self._ids = 0

    def clock(self) -> float:
        return self.mock.clock()

    # ---- control ---------------------------------------------------------------------
    def launch(self, kind: str, side: int, now: float | None = None) -> Approach:
        if kind not in KINDS:
            raise ValueError(f"unknown kind {kind!r}")
        if side not in (-1, 0, 1):
            raise ValueError("side must be -1, 0 or 1")
        return self._start(make_approach(kind, side, self.random.rng, self.speed), now)

    def _start(self, a: Approach, now: float | None) -> Approach:
        now = self.clock() if now is None else now
        self._ids += 1
        a.id, a.t_launch = self._ids, now
        self.mock.hand.run_script(a, now)
        self.gt.start(a, now)
        self.current = a
        if self.paused:
            self.paused = False
        return a

    def set_random(self, on: bool, seed: int | None = None) -> None:
        if seed is not None:
            self.random.reset(seed)
        self.random_on = bool(on)
        self.random.next_at = None

    def set_paused(self, paused: bool) -> None:
        now = self.clock()
        self.paused = bool(paused)
        if self.paused:
            self.mock.hand.pause(now)
        else:
            self.mock.hand.resume(now)

    def stop(self) -> None:
        """Hand away now."""
        self.mock.hand.set(80.0, 0.0)
        self.current = None

    def reset(self) -> None:
        """Clear the ground truth, put the hand away, put the dog back on its start mark."""
        self.stop()
        self.gt.reset()
        self.mock.dog_x_cm = 0.0
        self.random.reset()
        self.paused = False

    # ---- loop ------------------------------------------------------------------------
    def busy(self, now: float) -> bool:
        a = self.current
        return a is not None and a.t_launch is not None and now < a.t_launch + a.duration

    def update(self, now: float, robot_idle: bool = True) -> None:
        if not self.random_on or self.paused:
            return
        if self.busy(now):
            return
        if self.random.next_at is None:
            end = (
                self.current.t_launch + self.current.duration
                if self.current is not None and self.current.t_launch is not None
                else now
            )
            self.random.next_at = max(now, end) + self.random.gap()
        if now >= self.random.next_at and robot_idle:
            self.random.next_at = None
            self._start(self.random.draw(self.speed), now)

    def on_trigger(self, model: str, t: float, side: int) -> None:
        d, _ = self.mock.hand.state(self.clock())
        self.gt.on_trigger(model, t, side, d)

    def on_escape(self, t: float) -> None:
        tr = self.gt.trial_at(t)
        if tr is not None:
            tr.escaped = True

    # ---- reporting ---------------------------------------------------------------------
    def frame(self, now: float) -> dict[str, Any]:
        m = self.mock
        d, side = m.hand.state(now)
        a = self.current if self.busy(now) else None
        return {
            "dog": {
                "x_cm": round(m.dog_x_cm, 2),
                "pose": m.pose,
                "skill": m.skill,
                "phase": round(m.gait_phase, 3),
            },
            "hand": {
                "distance": round(d, 2),
                "side": round(side, 3),
                "near": d < 40.0,
            },
            "approach": None
            if a is None
            else {
                "id": a.id,
                "label": a.label(),
                "threat": a.threat,
                "t": round(now - a.t_launch, 2),
            },
            "random": {
                "on": self.random_on,
                "seed": self.random.seed,
                "next_in": None
                if self.random.next_at is None
                else round(max(0.0, self.random.next_at - now), 1),
            },
            "speed": self.speed,
            "paused": self.paused,
            "score": self.gt.stats(now),
            "trials": self.gt.recent(6),
            "band": {"near_cm": m.hand.NEAR_CM, "far_cm": m.hand.FAR_CM},
        }
