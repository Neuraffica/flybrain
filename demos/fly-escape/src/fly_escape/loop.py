"""Proximity samples -> every escape model -> triggers, events and scores.

Every model gets the same input on every sensor sample: the two channels mapped to
proximity in [0, 1] (0 = nothing, 1 = closest) with the resting level measured at start, and
the time since the previous sample. Only the driver model can move the robot (see
escape.EscapeController); the others are scored against it.

Without ground truth the scores are relative:
  * how often each model triggered;
  * for each event (triggers within EVENT_WINDOW_S of each other), which models fired and how
    much earlier or later than the driver;
  * false alarms: triggers while the operator has marked "hands off" on the dashboard.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from .calibration import Calibration
from .models import CIRCUITS, MODEL_KEYS, activity_of, n_neurons_of
from .sides import SideCheck

EVENT_WINDOW_S = 1.0  # triggers of different models this close belong to one event
TRACE_S = 12.0  # how much sample history frames carry for the dashboard trace
DT_MIN, DT_MAX = 0.005, 0.25


@dataclass
class Trigger:
    t: float
    model: str
    side: int


@dataclass
class Event:
    id: int
    t0: float
    hands_off: bool
    fired: dict[str, Trigger] = field(default_factory=dict)

    def summary(self, driver: str, t_origin: float) -> dict[str, Any]:
        ref = self.fired.get(driver)
        side = ref.side if ref else next(iter(self.fired.values())).side
        return {
            "id": self.id,
            "t": round(self.t0 - t_origin, 2),
            "side": side,
            "hands_off": self.hands_off,
            "fired": {
                k: {
                    "t": round(tr.t - t_origin, 3),
                    "side": tr.side,
                    "dt_ms": None if ref is None else round((tr.t - ref.t) * 1000.0),
                }
                for k, tr in self.fired.items()
            },
        }


@dataclass
class ModelScore:
    triggers: int = 0
    false_alarms: int = 0
    ms_ema: float = 0.0
    last: dict[str, float] = field(default_factory=dict)


class EscapeLoop:
    def __init__(
        self,
        models: dict[str, Any],
        raw_to_proximity,
        driver: str = "connectome",
        swap_sides: bool = False,
        sides_source: str = "flag",
        robot: str = "",
    ):
        missing = [k for k in MODEL_KEYS if k not in models]
        if missing:
            raise ValueError(f"missing models: {missing}")
        self.models = models
        self.raw_to_proximity = raw_to_proximity
        self.driver = driver if driver in models else "connectome"
        self.swap_sides = swap_sides
        self.sides_source = sides_source  # "calibrated", "flag" or "default" (see sides.resolve)
        self.robot = robot  # the key the side check result is stored under
        self.cal = Calibration()
        self.side_check = SideCheck()  # which pin is the left; models pause while it runs
        self.side_check_finished = False  # set when it ends; the app stores the result
        self.scores = {k: ModelScore() for k in MODEL_KEYS}
        self.events: deque[Event] = deque(maxlen=200)
        self._event_id = 0
        self.hands_off = False
        self.hands_off_spans: deque[list[float | None]] = deque(maxlen=50)
        self.trace: deque[tuple[float, float, float, int, int]] = deque()  # t, pL, pR, rawL, rawR
        self.recent_triggers: deque[Trigger] = deque()
        self.pending_driver: Trigger | None = None  # driver trigger not yet consumed by the app
        self.last_t: float | None = None
        self.t_origin: float | None = None
        self.n_samples = 0
        self.messages: list[str] = []
        self.raw: tuple[int, int] = (0, 0)
        self.prox: tuple[float, float] = (0.0, 0.0)

    # ---- control -------------------------------------------------------------------
    def set_driver(self, key: str) -> str | None:
        if key not in self.models:
            return f"unknown model {key!r}"
        self.driver = key
        return None

    def set_hands_off(self, on: bool, now: float) -> None:
        on = bool(on)
        if on == self.hands_off:
            return
        self.hands_off = on
        if on:
            self.hands_off_spans.append([now, None])
        elif self.hands_off_spans:
            self.hands_off_spans[-1][1] = now

    def recalibrate(self, now: float) -> None:
        self.cal.start(now)
        self.messages.append("calibrating: keep hands away from the sensor for 2 s")

    def start_side_check(self, now: float) -> str | None:
        if not self.cal.done:
            return "wait until the resting level is measured"
        if not self.cal.ok:
            return "measure the resting level first (Recalibrate): " + self.cal.message
        if self.side_check.active:
            return "the side check is already running"
        b = self.cal.baseline
        self.side_check.start(now, (b[1], b[0]) if self.swap_sides else b)  # in --pins order
        self.messages.append(self.side_check.message)
        return None

    def reset_scores(self) -> None:
        self.scores = {k: ModelScore() for k in MODEL_KEYS}
        self.events.clear()

    # ---- samples ---------------------------------------------------------------------
    def ingest(self, t: float, raw_a: float, raw_b: float) -> list[Trigger]:
        """One sensor sample (raw readings of the two pins in --pins order)."""
        if self.t_origin is None:
            self.t_origin = t
            if self.cal.started is None:
                self.cal.start(t)
        raw_l, raw_r = (raw_b, raw_a) if self.swap_sides else (raw_a, raw_b)
        self.raw = (int(raw_l), int(raw_r))
        self.n_samples += 1
        if self.side_check.active:
            # no model runs: nothing triggers, nothing is scored, the robot stays put
            phase = self.side_check.phase
            if self.side_check.add(t, raw_a, raw_b):
                if self.side_check.left is not None:
                    swap = self.side_check.left == 1
                    if swap != self.swap_sides:
                        self.cal.swap_sides()
                    self.swap_sides, self.sides_source = swap, "calibrated"
                self.side_check_finished = True
                for m in self.models.values():
                    m.reset()
            elif self.side_check.phase != phase:
                self.messages.append(self.side_check.message)
            p_l = float(self.raw_to_proximity(self.raw[0], baseline=self.cal.baseline[0]))
            p_r = float(self.raw_to_proximity(self.raw[1], baseline=self.cal.baseline[1]))
            self.prox = (p_l, p_r)
            self.last_t = t
            self.trace.append((t, p_l, p_r, *self.raw))
            self._trim(t)
            return []
        if not self.cal.done:
            if self.cal.add(t, raw_l, raw_r):
                self.messages.append(self.cal.message)
                for m in self.models.values():
                    m.reset()
            self.last_t = t
            self.trace.append((t, 0.0, 0.0, int(raw_l), int(raw_r)))
            self._trim(t)
            return []
        p_l = float(self.raw_to_proximity(raw_l, baseline=self.cal.baseline[0]))
        p_r = float(self.raw_to_proximity(raw_r, baseline=self.cal.baseline[1]))
        self.prox = (p_l, p_r)
        dt = DT_MIN if self.last_t is None else min(max(t - self.last_t, DT_MIN), DT_MAX)
        self.last_t = t
        fired = []
        for key in MODEL_KEYS:
            m = self.models[key]
            t0 = time.perf_counter()
            out = m.step(p_l, p_r, dt)
            ms = (time.perf_counter() - t0) * 1000.0
            sc = self.scores[key]
            sc.ms_ema = ms if sc.ms_ema == 0 else 0.9 * sc.ms_ema + 0.1 * ms
            gl = float(getattr(out, "gf_left", 0.0))
            gr = float(getattr(out, "gf_right", 0.0))
            thr = float(getattr(out, "threshold", 1.0)) or 1.0
            # Each side has its own threshold in the circuit models (gf_thresholds); drive is
            # already normalised so it fires at 1. Show each side as a fraction of its threshold.
            ths = getattr(m, "gf_thresholds", None)
            tl, tr_ = (float(ths[0]), float(ths[1])) if ths is not None else (thr, thr)
            sc.last = {
                "gf_left": gl,
                "gf_right": gr,
                "gf_left_n": gl / tl if tl else 0.0,
                "gf_right_n": gr / tr_ if tr_ else 0.0,
                "drive": float(getattr(out, "drive", 0.0)),
                "threshold": thr,
            }
            if getattr(out, "trigger", False):
                tr = Trigger(t, key, int(getattr(out, "side", 0)))
                fired.append(tr)
                self._record(tr)
        self.trace.append((t, p_l, p_r, int(raw_l), int(raw_r)))
        self._trim(t)
        return fired

    def _trim(self, t: float) -> None:
        while self.trace and self.trace[0][0] < t - TRACE_S:
            self.trace.popleft()
        while self.recent_triggers and self.recent_triggers[0].t < t - TRACE_S:
            self.recent_triggers.popleft()

    def _record(self, tr: Trigger) -> None:
        sc = self.scores[tr.model]
        sc.triggers += 1
        if self.hands_off:
            sc.false_alarms += 1
        self.recent_triggers.append(tr)
        ev = self.events[-1] if self.events else None
        if ev is None or tr.t - ev.t0 > EVENT_WINDOW_S:
            self._event_id += 1
            ev = Event(self._event_id, tr.t, self.hands_off)
            self.events.append(ev)
        if tr.model not in ev.fired:
            ev.fired[tr.model] = tr
        if tr.model == self.driver:
            self.pending_driver = tr

    def latency_stats(self) -> dict[str, dict[str, Any]]:
        """Per model: events it fired in, events (outside hands-off) it missed, and the mean
        time of its trigger minus the driver's over events where both fired (ms; negative =
        earlier than the driver)."""
        out = {}
        for k in MODEL_KEYS:
            dts, fired, missed = [], 0, 0
            for ev in self.events:
                if k in ev.fired:
                    fired += 1
                    ref = ev.fired.get(self.driver)
                    if ref is not None and k != self.driver:
                        dts.append((ev.fired[k].t - ref.t) * 1000.0)
                elif not ev.hands_off:
                    missed += 1
            out[k] = {
                "events": fired,
                "missed": missed,
                "mean_ms": round(sum(dts) / len(dts)) if dts else None,
            }
        return out

    def take_driver_trigger(self) -> Trigger | None:
        tr, self.pending_driver = self.pending_driver, None
        return tr

    # ---- reporting -----------------------------------------------------------------
    def frame(self, now: float, since: float | None = None) -> dict[str, Any]:
        o = self.t_origin or now
        lat = self.latency_stats()
        models = {}
        for key in MODEL_KEYS:
            m, sc = self.models[key], self.scores[key]
            models[key] = {
                "triggers": sc.triggers,
                "false_alarms": sc.false_alarms,
                "latency_ms": lat[key]["mean_ms"],
                "events": lat[key]["events"],
                "missed": lat[key]["missed"],
                "ms": round(sc.ms_ema, 3),
                "n_neurons": n_neurons_of(m) if key in CIRCUITS else None,
                **{k: round(v, 4) for k, v in sc.last.items()},
            }
        models["connectome"]["activity"] = activity_of(self.models["connectome"])
        t_cut = since if since is not None else now - TRACE_S
        return {
            "t": round(now - o, 3),
            "driver": self.driver,
            "swap_sides": self.swap_sides,
            "sides": {
                "swap": self.swap_sides,
                "source": self.sides_source,
                "robot": self.robot,
                "check": self.side_check.status(now),
            },
            "samples": self.n_samples,
            "raw": list(self.raw),
            "prox": [round(self.prox[0], 4), round(self.prox[1], 4)],
            "trace": [
                [round(t - o, 3), round(pl, 4), round(pr, 4), rl, rr]
                for t, pl, pr, rl, rr in self.trace
                if t > t_cut
            ],
            "triggers": [
                [round(tr.t - o, 3), tr.model, tr.side]
                for tr in self.recent_triggers
                if tr.t > t_cut
            ],
            "hands_off": self.hands_off,
            "hands_off_spans": [
                [round(a - o, 2), None if b is None else round(b - o, 2)]
                for a, b in self.hands_off_spans
            ],
            "calibration": self.cal.status(),
            "events": [e.summary(self.driver, o) for e in list(self.events)[-12:]][::-1],
            "models": models,
        }
