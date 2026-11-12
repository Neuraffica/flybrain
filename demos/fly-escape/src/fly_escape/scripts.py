"""Scripted hand movements for the simulated sensor: (t seconds) -> (distance_cm, side).

side is +1 for the robot's left, -1 right. The simulated sensor reads its resting level beyond
about 20 cm and about 0 closer than 12 cm (flybrain_edge.bittle.mock.ProximityHand)."""

from __future__ import annotations

AWAY_CM = 80.0


def _ramp(t: float, t0: float, t1: float, a: float, b: float) -> float:
    if t <= t0:
        return a
    if t >= t1:
        return b
    return a + (b - a) * (t - t0) / (t1 - t0)


def approach(duration_s: float, side: float = 0.0, closest_cm: float = 5.0, hold_s: float = 1.0):
    def f(t: float) -> tuple[float, float]:
        if t > duration_s + hold_s:
            return AWAY_CM, side
        return _ramp(t, 0.0, duration_s, 60.0, closest_cm), side

    return f


def hover(t: float) -> tuple[float, float]:
    """Come into the graded band (about 17 cm) and stay there: a hand that does not loom."""
    if t > 5.0:
        return AWAY_CM, 0.0
    return _ramp(t, 0.0, 1.0, 60.0, 17.0), 0.0


def retreat(t: float) -> tuple[float, float]:
    """Start close and move away: proximity falls, nothing looms."""
    return _ramp(t, 1.0, 2.5, 5.0, 60.0), 0.0


SCRIPTS = {
    "slow": approach(3.0),
    "fast": approach(0.4),
    "left": approach(1.0, side=0.8),
    "right": approach(1.0, side=-0.8),
    "hover": hover,
    "retreat": retreat,
}


_CYCLE = ("slow", "fast", "left", "right", "hover", "retreat")
_CYCLE_S = 8.0


def cycle(t: float) -> tuple[float, float]:
    """Unattended mock demo: every script in turn, one every 8 s, forever."""
    if t < 0:
        return AWAY_CM, 0.0
    i = int(t // _CYCLE_S)
    return SCRIPTS[_CYCLE[i % len(_CYCLE)]](t - i * _CYCLE_S)


SCRIPTS["cycle"] = cycle
