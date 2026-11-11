"""Scripted mock scenarios: how the simulated dog turns and when the cue is taken away.

Each scenario is a list of phases. During a phase the dog's world yaw moves linearly from the
previous phase's end by ``turn_deg`` (0 = held still), and the cue is on or off. The loop keeps
error statistics per phase label, so the CLI prints one table per phase at the end.

    memory     turn to a new heading with the cue on, blindfold, hold still 20 s, cue back.
               Tests whether the bump stays where it was left (recurrent memory).
    dark-turn  blindfold, then rotate 90 deg in the dark, hold, cue back.
               Tests path integration from angular velocity alone.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Phase:
    label: str
    duration_s: float
    turn_deg: float = 0.0
    blindfold: bool = False


SCENARIOS: dict[str, list[Phase]] = {
    "memory": [
        Phase("settle (cue on)", 4.0),
        Phase("turn +90 (cue on)", 3.0, turn_deg=90.0),
        Phase("hold (cue on)", 4.0),
        Phase("blindfold, still 0-5 s", 5.0, blindfold=True),
        Phase("blindfold, still 5-20 s", 15.0, blindfold=True),
        Phase("cue back", 5.0),
    ],
    "dark-turn": [
        Phase("settle (cue on)", 4.0),
        Phase("blindfold, still", 3.0, blindfold=True),
        Phase("blindfold, turn +90", 4.5, turn_deg=90.0, blindfold=True),
        Phase("blindfold, after turn", 5.0, blindfold=True),
        Phase("cue back", 5.0),
    ],
}
SCRIPT_CHOICES = ("none", "sweep", *SCENARIOS)


class ScenarioRunner:
    """Advances a scenario on the loop's clock; drives the mock's yaw and the blindfold."""

    def __init__(self, name: str, start_t: float, start_yaw_deg: float):
        if name not in SCENARIOS:
            raise ValueError(f"unknown scenario {name!r}; choose from {', '.join(SCENARIOS)}")
        self.name = name
        self.phases = SCENARIOS[name]
        self.t0 = start_t
        self.yaw0 = start_yaw_deg
        self.index = -1
        self.done = False

    @property
    def total_s(self) -> float:
        return sum(p.duration_s for p in self.phases)

    def at(self, t: float) -> tuple[int, float, float]:
        """(phase index or len(phases) when finished, phase elapsed s, world yaw deg)."""
        e = t - self.t0
        yaw = self.yaw0
        for i, p in enumerate(self.phases):
            if e < p.duration_s:
                return i, e, yaw + p.turn_deg * max(0.0, e) / p.duration_s
            e -= p.duration_s
            yaw += p.turn_deg
        return len(self.phases), e, yaw

    def status(self, t: float) -> dict:
        i, e, _ = self.at(t)
        p = self.phases[min(i, len(self.phases) - 1)]
        return {
            "name": self.name,
            "phase": p.label if i < len(self.phases) else "finished",
            "index": i,
            "phases": len(self.phases),
            "phase_elapsed": round(e, 1),
            "phase_duration": p.duration_s,
            "done": i >= len(self.phases),
        }
