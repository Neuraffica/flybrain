"""Session logs under ./runs/<stamp>/ and replay of recorded proximity samples.

Files:
  meta.json     command line, link, firmware, pins, calibration
  samples.csv   t (s since start), raw_a, raw_b: raw readings of the two pins in --pins order
  triggers.csv  t, model, side, hands_off
  escapes.csv   t, side, skill, stood

Replay accepts a run directory, its samples.csv, or a JSON list of [t, raw_a, raw_b] (the format
of packages/flybrain-models/tests/fixtures/ir_hand_approaches_usb.json).
"""

from __future__ import annotations

import csv
import json
import time
from pathlib import Path
from typing import Any


class SessionLog:
    def __init__(self, root: str | Path = "runs", meta: dict[str, Any] | None = None):
        stamp = time.strftime("%Y%m%d-%H%M%S")
        base = Path(root) / f"escape-{stamp}"
        n = 1
        while base.exists():
            base = Path(root) / f"escape-{stamp}-{n}"
            n += 1
        base.mkdir(parents=True)
        self.dir = base
        self.t0: float | None = None
        (base / "meta.json").write_text(json.dumps(meta or {}, indent=2, default=str))
        # kept open for the whole session, closed in close()
        self._files = {
            name: open(base / f"{name}.csv", "w", newline="")  # noqa: SIM115
            for name in ("samples", "triggers", "escapes")
        }
        self._w = {k: csv.writer(f) for k, f in self._files.items()}
        self._w["samples"].writerow(["t", "raw_a", "raw_b"])
        self._w["triggers"].writerow(["t", "model", "side", "hands_off"])
        self._w["escapes"].writerow(["t", "side", "skill", "stood"])
        self._rows = 0

    def rel(self, t: float) -> float:
        if self.t0 is None:
            self.t0 = t
        return round(t - self.t0, 4)

    def sample(self, t: float, raw_a: int, raw_b: int) -> None:
        self._w["samples"].writerow([self.rel(t), raw_a, raw_b])
        self._rows += 1
        if self._rows % 100 == 0:
            self.flush()

    def trigger(self, t: float, model: str, side: int, hands_off: bool) -> None:
        self._w["triggers"].writerow([self.rel(t), model, side, int(hands_off)])

    def escape(self, t: float, side: int, skill: str, stood: bool | None) -> None:
        self._w["escapes"].writerow([self.rel(t), side, skill, "" if stood is None else int(stood)])

    def flush(self) -> None:
        for f in self._files.values():
            f.flush()

    def close(self) -> None:
        for f in self._files.values():
            try:
                f.close()
            except OSError:
                pass


def load_samples(path: str | Path) -> list[tuple[float, int, int]]:
    p = Path(path)
    if p.is_dir():
        p = p / "samples.csv"
    if not p.exists():
        raise FileNotFoundError(f"no samples at {p}")
    if p.suffix == ".json":
        data = json.loads(p.read_text())
        rows = [(float(r[0]), int(r[1]), int(r[2])) for r in data]
    else:
        rows = []
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    rows.append((float(r["t"]), int(float(r["raw_a"])), int(float(r["raw_b"]))))
                except (KeyError, ValueError):
                    continue
    rows.sort(key=lambda r: r[0])
    return rows


class ReplaySource:
    def __init__(self, rows: list[tuple[float, int, int]]):
        self.rows = rows
        self.i = 0
        self.t0 = rows[0][0] if rows else 0.0

    @property
    def done(self) -> bool:
        return self.i >= len(self.rows)

    @property
    def duration(self) -> float:
        return self.rows[-1][0] - self.t0 if self.rows else 0.0

    def poll(self, sim_t: float) -> list[tuple[float, int, int]]:
        out = []
        while self.i < len(self.rows) and self.rows[self.i][0] - self.t0 <= sim_t:
            t, a, b = self.rows[self.i]
            out.append((t - self.t0, a, b))
            self.i += 1
        return out
