"""Scoring against ground truth: in the simulator every approach is known to be a threat or not.

A model's trigger belongs to the trial whose window (launch to TRAIL_S after the hand is away
again) contains it; only its first trigger per trial counts. Per model:

  caught         threats with a trigger / threats
  false escapes  non-threats with a trigger / non-threats
  reaction time  trigger time minus the moment the hand first came within the sensor's range
  side correct   trigger side equal to the approach side, over approaches from the left/right
  distance       the hand's distance from the sensor at the trigger
  spontaneous    triggers outside every trial
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .approaches import Approach
from .models import MODEL_KEYS

TRAIL_S = 0.8


@dataclass
class Fired:
    t: float
    side: int
    distance_cm: float


@dataclass
class Trial:
    approach: Approach
    t0: float
    t_detect: float | None
    fired: dict[str, Fired] = field(default_factory=dict)
    escaped: bool = False  # the driver made the dog escape during this trial

    @property
    def t_end(self) -> float:
        return self.t0 + self.approach.duration + TRAIL_S


class GroundTruth:
    def __init__(self) -> None:
        self.trials: list[Trial] = []
        self.spontaneous = {k: 0 for k in MODEL_KEYS}

    def reset(self) -> None:
        self.trials.clear()
        self.spontaneous = {k: 0 for k in MODEL_KEYS}

    def start(self, approach: Approach, t0: float) -> Trial:
        dt = approach.detect_t()
        tr = Trial(approach, t0, None if dt is None else t0 + dt)
        self.trials.append(tr)
        return tr

    def trial_at(self, t: float) -> Trial | None:
        for tr in reversed(self.trials):
            if tr.t0 <= t <= tr.t_end:
                return tr
        return None

    def on_trigger(self, model: str, t: float, side: int, distance_cm: float) -> None:
        tr = self.trial_at(t)
        if tr is None:
            self.spontaneous[model] = self.spontaneous.get(model, 0) + 1
            return
        tr.fired.setdefault(model, Fired(t, int(side), float(distance_cm)))

    def stats(self, now: float | None = None) -> dict[str, Any]:
        """Per-model totals over trials that are over (or all, if now is None)."""
        done = [t for t in self.trials if now is None or t.t_end <= now]
        threats = [t for t in done if t.approach.threat]
        safe = [t for t in done if not t.approach.threat]
        out: dict[str, Any] = {
            "trials": len(done),
            "threats": len(threats),
            "non_threats": len(safe),
            "models": {},
        }
        for k in MODEL_KEYS:
            caught = [t for t in threats if k in t.fired]
            false = [t for t in safe if k in t.fired]
            rts = [(t.fired[k].t - t.t_detect) * 1000.0 for t in caught if t.t_detect is not None]
            lateral = [t for t in caught + false if t.approach.side != 0]
            side_ok = [t for t in lateral if t.fired[k].side == t.approach.side]
            dists = [t.fired[k].distance_cm for t in caught]
            out["models"][k] = {
                "caught": len(caught),
                "false": len(false),
                "rt_ms": round(sum(rts) / len(rts)) if rts else None,
                "side_ok": len(side_ok),
                "side_n": len(lateral),
                "dist_cm": round(sum(dists) / len(dists), 1) if dists else None,
                "spontaneous": self.spontaneous.get(k, 0),
            }
        return out

    def recent(self, n: int = 8) -> list[dict[str, Any]]:
        rows = []
        for tr in self.trials[-n:][::-1]:
            a = tr.approach
            rows.append(
                {
                    "id": a.id,
                    "label": a.label(),
                    "threat": a.threat,
                    "fired": {
                        k: {
                            "rt_ms": None
                            if tr.t_detect is None
                            else round((f.t - tr.t_detect) * 1000.0),
                            "side": f.side,
                            "distance_cm": round(f.distance_cm, 1),
                        }
                        for k, f in tr.fired.items()
                    },
                    "escaped": tr.escaped,
                }
            )
        return rows


def format_table(stats: dict[str, Any], driver: str) -> str:
    head = (
        f"ground truth: {stats['trials']} approaches, {stats['threats']} threats, "
        f"{stats['non_threats']} non-threats"
    )
    cols = (
        f"  {'model':<11} {'caught':>9} {'false esc':>10} {'react ms':>9} {'side ok':>8} "
        f"{'dist cm':>8} {'spont':>6}"
    )
    lines = [head, cols]
    for k, m in stats["models"].items():
        mark = " *" if k == driver else ""
        caught = f"{m['caught']}/{stats['threats']}"
        false = f"{m['false']}/{stats['non_threats']}"
        rt = "-" if m["rt_ms"] is None else str(m["rt_ms"])
        side = "-" if not m["side_n"] else f"{m['side_ok']}/{m['side_n']}"
        dist = "-" if m["dist_cm"] is None else f"{m['dist_cm']:.1f}"
        lines.append(
            f"  {k + mark:<11} {caught:>9} {false:>10} {rt:>9} {side:>8} {dist:>8} "
            f"{m['spontaneous']:>6}"
        )
    lines.append(
        "  * = driver (moves the dog). React: from the hand entering the sensor's range to the"
        " trigger. Side: over approaches from the left or right. Dist: hand to sensor at the"
        " trigger on threats. Spont: triggers with no approach running."
    )
    return "\n".join(lines)
