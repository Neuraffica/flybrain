"""Session logs under ./runs/<stamp>/ and replay of a recorded session.

Files:
  meta.json    command line, transport, firmware id, config
  samples.csv  every IMU sample as consumed by the loop: t (s since start), yaw_deg, pitch_deg,
               roll_deg, source, blindfold (1 = no cue was given), phase (scenario phase or "")
  frames.csv   per tick: truth, omega, cue given this tick, blindfold, phase, hold state,
               sweep state ("turning left", "pausing", ...), and each model's heading,
               error and confidence

Replay reads samples.csv (yaw is the only sensor input the models ever see, and the blindfold
column says whether it was also given as a cue) and feeds it back through fresh models on a
simulated clock. Logs written before the blindfold columns existed replay with the cue on.
"""

from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .models import MODEL_KEYS

SAMPLE_FIELDS = ["t", "yaw_deg", "pitch_deg", "roll_deg", "source", "blindfold", "phase"]
FRAME_FIELDS = [
    "t",
    "mode",
    "driver",
    "truth",
    "omega",
    "cue",
    "blindfold",
    "phase",
    "pan_cmd",
    "hold_state",
    "sweep_state",
] + [f"{k}_{f}" for k in MODEL_KEYS for f in ("heading", "err", "confidence")]


class SessionLog:
    def __init__(self, root: str | Path = "runs", meta: dict[str, Any] | None = None):
        stamp = time.strftime("%Y%m%d-%H%M%S")
        base = Path(root) / stamp
        n = 1
        while base.exists():
            base = Path(root) / f"{stamp}-{n}"
            n += 1
        base.mkdir(parents=True)
        self.dir = base
        self.t0: float | None = None
        (base / "meta.json").write_text(json.dumps(meta or {}, indent=2, default=str))
        # kept open for the whole session, closed in close()
        self._sf = open(base / "samples.csv", "w", newline="")  # noqa: SIM115
        self._ff = open(base / "frames.csv", "w", newline="")  # noqa: SIM115
        self._sw = csv.writer(self._sf)
        self._fw = csv.writer(self._ff)
        self._sw.writerow(SAMPLE_FIELDS)
        self._fw.writerow(FRAME_FIELDS)
        self._rows = 0

    def _rel(self, t: float) -> float:
        if self.t0 is None:
            self.t0 = t
        return round(t - self.t0, 4)

    def sample(self, s, blindfold: bool = False, phase: str | None = None) -> None:
        self._sw.writerow(
            [
                self._rel(s.t),
                s.yaw_deg,
                s.pitch_deg,
                s.roll_deg,
                s.source,
                int(blindfold),
                phase or "",
            ]
        )

    def frame(self, now: float, fr: dict[str, Any]) -> None:
        row: list[Any] = [
            self._rel(now),
            fr["mode"],
            fr["driver"],
            fr["truth"],
            fr["omega"],
            int(fr["cue"]),
            int(fr["blindfold"]["on"]),
            fr["phase"],
            fr["needle"]["cmd"],
            fr["hold"]["state"],
            fr["sweep"]["label"] if fr["mode"] == "sweep" else "",
        ]
        for k in MODEL_KEYS:
            m = fr["models"][k]
            row += [m["heading"], m["err"], m["confidence"]]
        self._fw.writerow(row)
        self._rows += 1
        if self._rows % 50 == 0:
            self.flush()

    def flush(self) -> None:
        self._sf.flush()
        self._ff.flush()

    def close(self) -> None:
        for f in (self._sf, self._ff):
            try:
                f.close()
            except OSError:
                pass


@dataclass(frozen=True)
class ReplaySample:
    t: float
    yaw_deg: float
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    source: str = "replay"
    blindfold: bool = False
    phase: str = ""


def load_samples(path: str | Path) -> list[ReplaySample]:
    p = Path(path)
    if p.is_dir():
        p = p / "samples.csv"
    if not p.exists():
        raise FileNotFoundError(f"no samples.csv at {p}")
    out = []
    with open(p, newline="") as f:
        for row in csv.DictReader(f):
            try:
                out.append(
                    ReplaySample(
                        float(row["t"]),
                        float(row["yaw_deg"]),
                        float(row.get("pitch_deg") or 0),
                        float(row.get("roll_deg") or 0),
                        row.get("source") or "replay",
                        (row.get("blindfold") or "0").strip() in ("1", "true", "True"),
                        row.get("phase") or "",
                    )
                )
            except (KeyError, ValueError):
                continue
    out.sort(key=lambda s: s.t)
    return out


class ReplaySource:
    """Hands out recorded samples whose time has come on a simulated clock."""

    def __init__(self, samples: list[ReplaySample]):
        self.samples = samples
        self.i = 0

    @property
    def done(self) -> bool:
        return self.i >= len(self.samples)

    @property
    def duration(self) -> float:
        return self.samples[-1].t if self.samples else 0.0

    def poll(self, sim_t: float) -> list[ReplaySample]:
        out = []
        while self.i < len(self.samples) and self.samples[self.i].t <= sim_t:
            out.append(self.samples[self.i])
            self.i += 1
        return out
