"""Simulated and recorded hand approaches for fitting and benchmarking the escape models.

Sensor model (Petoi double IR proximity sensor, measured on the robot): each channel reads
about 3400 with nothing in range and about 0 with a hand close; it is graded only over a
narrow band. We model raw = 3400 * clip((d - D_NEAR) / (D_FAR - D_NEAR), 0, 1) + noise, with
the hand distance d in metres, D_FAR = 0.12 m (first response) and D_NEAR = 0.05 m
(saturation). A hand at 0.3-1.0 m/s crosses the band in 70-230 ms, as measured.

Ground truth. A *threat* is a hand that approaches at >= 0.3 m/s and comes within D_NEAR.
Non-threats: slow drift to the sensor (<= 0.08 m/s), a fast hand that stops short inside the
band, hovering with tremor, a hand moving away, and sensor noise alone. Approaches at
0.15-0.3 m/s are ambiguous and only used for the speed curve. The ground-truth moment t_gt is
when the hand (nearer channel) crosses D_FAR, i.e. the first moment the sensor can respond.
A hit is the first trigger in [t_gt - 0.1 s, t_gt + 1.0 s]; every other trigger is a false
alarm. The first 0.5 s of every episode is warm-up and not scored.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .escape import BASELINE_RAW, raw_to_proximity

D_FAR = 0.12
D_NEAR = 0.05
THREAT_SPEED = 0.3
WARMUP_S = 0.5
HIT_WINDOW = (-0.1, 1.0)
NOISE_SD = {"normal": 25.0, "noisy": 80.0}


@dataclass
class Episode:
    name: str
    kind: str  # approach | stop_short | retreat | hover | drift | noise
    threat: bool | None  # None = ambiguous (speed curve only)
    t: np.ndarray  # sample times (s)
    raw_l: np.ndarray
    raw_r: np.ndarray
    d_l: np.ndarray  # true hand distance per channel at the sample times (m)
    d_r: np.ndarray
    t_gt: float | None = None
    side: int = 0  # +1 left, -1 right, 0 head-on/none
    speed: float = 0.0  # approach speed (m/s)
    meta: dict = field(default_factory=dict)
    unilateral: bool = False  # only one channel ever sees the hand

    @property
    def category(self) -> str:
        """left / right / head_on / left_only / right_only for approaches, else the kind."""
        if self.kind != "approach":
            return self.kind
        name = {1: "left", -1: "right", 0: "head_on"}[self.side]
        return name + ("_only" if self.unilateral else "")

    @property
    def duration(self) -> float:
        return float(self.t[-1])


def sensor_raw(d: np.ndarray, rng: np.random.Generator, noise_sd: float) -> np.ndarray:
    frac = np.clip((d - D_NEAR) / (D_FAR - D_NEAR), 0.0, 1.0)
    raw = BASELINE_RAW * frac + rng.normal(0.0, noise_sd, size=d.shape)
    return np.clip(np.round(raw), 0, 4095)


def sample_times(duration: float, rate_hz: float, rng: np.random.Generator) -> np.ndarray:
    """Sample times with +-25% jitter on every interval (USB 28 Hz, Bluetooth ~12 Hz)."""
    n = int(duration * rate_hz * 1.5) + 2
    dts = rng.uniform(0.75, 1.25, size=n) / rate_hz
    t = np.cumsum(dts)
    return t[t <= duration]


def _profile(kind: str, t: np.ndarray, rng: np.random.Generator, speed: float) -> np.ndarray:
    """Distance of the hand (nearer channel) over time."""
    t0 = rng.uniform(1.0, 1.5)
    d = np.full_like(t, 0.40)
    if kind in ("approach", "drift"):
        d_end = 0.02
        d_start = 0.40 if kind == "approach" else 0.16  # a drift starts just outside range
        t1 = t0 + (d_start - d_end) / speed
        hold = 0.6 if kind == "approach" else 1.0
        d = np.where(t < t0, d_start, np.maximum(d_end, d_start - speed * (t - t0)))
        back = t > t1 + hold
        d = np.where(back, np.minimum(0.40, d_end + 0.5 * (t - t1 - hold)), d)
    elif kind == "stop_short":
        d_stop = rng.uniform(0.085, 0.11)
        d = np.where(t < t0, 0.40, np.maximum(d_stop, 0.40 - speed * (t - t0)))
        t1 = t0 + (0.40 - d_stop) / speed
        d = np.where(t > t1 + 1.0, np.minimum(0.40, d_stop + 0.4 * (t - t1 - 1.0)), d)
    elif kind == "retreat":
        d = np.where(t < t0, 0.03, np.minimum(0.40, 0.03 + speed * (t - t0)))
    elif kind == "hover":
        d_h = rng.uniform(0.07, 0.10)
        d = np.maximum(d_h, 0.16 - 0.05 * t)  # slow entry (0.05 m/s) then hover
        f, ph = rng.uniform(3.0, 6.0), rng.uniform(0, 2 * np.pi)
        d = d + np.where(d <= d_h + 1e-9, 0.005 * np.sin(2 * np.pi * f * t + ph), 0.0)
    return d


def make_episode(
    kind: str,
    rng: np.random.Generator,
    rate_hz: float,
    noise: str,
    speed: float = 0.0,
    side: int = 0,
    duration: float = 5.0,
    unilateral: bool = False,
) -> Episode:
    if kind in ("drift", "hover"):
        duration = max(duration, 8.0)
    t = sample_times(duration, rate_hz, rng)
    if kind == "noise":
        d = np.full_like(t, 0.40)
    else:
        d = _profile(kind, t, rng, speed)
    # lateral approach: the far channel sees the hand 1.5-4 cm further away (so it lags);
    # head-on: both channels within 3 mm of each other
    off = rng.uniform(0.015, 0.04) if side else rng.uniform(-0.003, 0.003)
    if unilateral and side:
        off = 0.15  # the far channel never gets within range (as seen on the robot)
    if side > 0:
        d_l, d_r = d, d + off
    elif side < 0:
        d_l, d_r = d + off, d
    else:
        d_l, d_r = d + max(off, 0.0), d + max(-off, 0.0)
    sd = NOISE_SD[noise]
    raw_l, raw_r = sensor_raw(d_l, rng, sd), sensor_raw(d_r, rng, sd)
    if kind == "approach":
        threat = True if speed >= THREAT_SPEED else (None if speed >= 0.15 else False)
    else:
        threat = False
    t_gt = None
    if kind in ("approach", "drift", "stop_short"):
        # continuous-time crossing of D_FAR by the nearer channel
        dn = np.minimum(d_l, d_r)
        if (dn <= D_FAR).any():
            i = int(np.argmax(dn <= D_FAR))
            if i > 0 and dn[i - 1] > D_FAR:
                frac = (dn[i - 1] - D_FAR) / max(dn[i - 1] - dn[i], 1e-9)
                t_gt = float(t[i - 1] + frac * (t[i] - t[i - 1]))
            else:
                t_gt = float(t[i])
    name = f"{kind}_{speed:.2f}ms_side{side:+d}{'only' if unilateral else ''}_{rate_hz:g}Hz_{noise}"
    return Episode(
        name,
        kind,
        threat,
        t,
        raw_l,
        raw_r,
        d_l,
        d_r,
        t_gt,
        side if kind == "approach" else 0,
        speed,
        {"rate_hz": rate_hz, "noise": noise},
        unilateral=bool(unilateral and side),
    )


def episode_set(
    seed: int,
    reps: int = 2,
    speeds=(0.15, 0.2, 0.3, 0.45, 0.7, 1.0, 1.5),
    rates=(12.0, 28.0),
    noises=("normal", "noisy"),
) -> list[Episode]:
    rng = np.random.default_rng(seed)
    out = []
    for rate in rates:
        for noise in noises:
            for _ in range(reps):
                for v in speeds:
                    for side in (1, 0, -1):
                        out.append(make_episode("approach", rng, rate, noise, v, side))
                    for side in (1, -1):  # seen by one channel only
                        out.append(
                            make_episode("approach", rng, rate, noise, v, side, unilateral=True)
                        )
                for _ in range(3):
                    out.append(make_episode("stop_short", rng, rate, noise, rng.uniform(0.3, 1.2)))
                    out.append(make_episode("hover", rng, rate, noise))
                    out.append(make_episode("drift", rng, rate, noise, rng.uniform(0.02, 0.08)))
                for _ in range(2):
                    out.append(make_episode("retreat", rng, rate, noise, rng.uniform(0.3, 1.0)))
                    out.append(make_episode("noise", rng, rate, noise))
    return out


def training_set() -> list[Episode]:
    """Smaller than the benchmark (fitting runs it thousands of times): normal noise only,
    four speeds, one repetition, both sample rates; seed disjoint from the benchmark."""
    return episode_set(seed=1001, reps=1, speeds=(0.15, 0.3, 0.6, 1.2), noises=("normal",))


def benchmark_set() -> list[Episode]:
    return episode_set(seed=2026, reps=2)


# ---- running and scoring -------------------------------------------------------------------
def run_episode(model, ep: Episode) -> dict:
    model.reset()
    trig_t, trig_side, trig_gf, drive = [], [], [], np.empty(len(ep.t))
    prev = 0.0
    for i, ti in enumerate(ep.t):
        out = model.step(raw_to_proximity(ep.raw_l[i]), raw_to_proximity(ep.raw_r[i]), ti - prev)
        prev = ti
        drive[i] = out.drive
        if out.trigger:
            trig_t.append(float(ti))
            trig_side.append(out.side)
            gf = getattr(model, "side_from_gf", None)
            trig_gf.append(gf() if gf else out.side)
    return {"triggers": trig_t, "sides": trig_side, "sides_gf": trig_gf, "drive": drive}


def score_episode(ep: Episode, res: dict) -> dict:
    trig = [
        (t, s, g)
        for t, s, g in zip(res["triggers"], res["sides"], res["sides_gf"], strict=True)
        if t >= WARMUP_S
    ]
    hit, latency, side_ok, dist, side_gf_ok = None, None, None, None, None
    fa = 0
    scored_s = ep.duration - WARMUP_S
    if ep.threat and ep.t_gt is not None:
        lo, hi = ep.t_gt + HIT_WINDOW[0], ep.t_gt + HIT_WINDOW[1]
        scored_s -= hi - max(lo, WARMUP_S)
        inside = [(t, s, g) for t, s, g in trig if lo <= t <= hi]
        hit = bool(inside)
        fa = len(trig) - (1 if inside else 0)
        if inside:
            t_hit, s, g = inside[0]
            side_gf_ok = g == ep.side
            latency = t_hit - ep.t_gt
            i = int(np.searchsorted(ep.t, t_hit))
            dist = float(min(ep.d_l[i], ep.d_r[i]))
            side_ok = s == ep.side
    elif ep.threat is None:
        scored_s = 0.0  # ambiguous: not part of hit / false-alarm statistics
    else:
        fa = len(trig)
    first = res["triggers"][0] if res["triggers"] else None
    return {
        "hit": hit,
        "latency": latency,
        "side_ok": side_ok,
        "side_gf_ok": side_gf_ok,
        "dist_at_trigger": dist,
        "false_alarms": fa if ep.threat is not None else 0,
        "scored_s": max(scored_s, 0.0),
        "first_trigger": first,
    }


# ---- the real recording --------------------------------------------------------------------
RECORDINGS = {
    # 25 s, five hand approaches, recorded at the desk (USB, ~28 Hz)
    "hand_usb": "ir_hand_approaches_usb.json",
    # 45 s escape-mode run on the robot (USB, ~27 Hz); raw_a = pin 36 = prox_left
    "robot_run": "ir_escape_robot_run_usb.json",
}
REC_FILE = RECORDINGS["hand_usb"]
REAL_ONSET_RAW = BASELINE_RAW - 300.0  # 6 noise sd below the empty-scene level
REAL_NEAR_RAW = 500.0


def recording_path(name: str = "hand_usb") -> Path:
    """A recording shipped as a test fixture (packages/flybrain-models/tests/fixtures)."""
    here = Path(__file__).resolve()
    return here.parents[2] / "tests" / "fixtures" / RECORDINGS[name]


def load_recording(name: str = "hand_usb", pin36_is_left: bool = True):
    """Returns t, raw_left, raw_right, taking pin 36 as the left channel (as the demo does,
    prox_left = pin 36). Whether pin 36 is physically on the robot's left is not
    established; ``pin36_is_left`` flips it."""
    data = np.array(json.loads(recording_path(name).read_text()), float)
    t, a, b = data[:, 0], data[:, 1], data[:, 2]
    return (t, a, b) if pin36_is_left else (t, b, a)


def label_recording(t, raw_l, raw_r, quiet_s: float = 0.5) -> list[dict]:
    """Offline labelling rule (the recording has no labels): an event starts at the first
    sample where min(raw) < 3100 after >= 0.5 s at or above it; it is an approach if min(raw)
    goes below 500 within 0.5 s; it ends when both channels are back >= 3100. t_gt = event
    start. The leading side is the channel that first drops below 1700 (mid-band)."""
    m = np.minimum(raw_l, raw_r)
    events, i, n = [], 0, len(t)
    prev_end = t[0] - quiet_s
    while i < n:
        if m[i] >= REAL_ONSET_RAW:
            i += 1
            continue
        start = i
        quiet_before = t[start] - prev_end >= quiet_s
        j = start
        while j < n and m[j] < REAL_ONSET_RAW:
            j += 1
        end = min(j, n - 1)
        prev_end = t[end]
        near = [k for k in range(start, end + 1) if t[k] <= t[start] + 0.5 and m[k] < REAL_NEAR_RAW]
        lead = 0
        cl = np.flatnonzero(raw_l[start : end + 1] < 1700)
        cr = np.flatnonzero(raw_r[start : end + 1] < 1700)
        if len(cl) and len(cr):
            lead = 0 if cl[0] == cr[0] else (1 if cl[0] < cr[0] else -1)
        elif len(cl) or len(cr):
            lead = 1 if len(cl) else -1  # only one channel saw the hand
        events.append(
            {
                "t_gt": float(t[start]),
                "t_end": float(t[end]),
                "approach": bool(near) and quiet_before,
                "leading_side": lead,
                "min_left": float(raw_l[start : end + 1].min()),
                "min_right": float(raw_r[start : end + 1].min()),
            }
        )
        i = j + 1
    return events


def run_recording(model, name: str = "hand_usb", pin36_is_left: bool = True) -> dict:
    t, rl, rr = load_recording(name, pin36_is_left=pin36_is_left)
    ep = Episode("recording", "real", None, t, rl, rr, np.zeros_like(t), np.zeros_like(t))
    res = run_episode(model, ep)
    events = [e for e in label_recording(t, rl, rr) if e["approach"]]  # blips are not events
    trig = [(tt, s) for tt, s in zip(res["triggers"], res["sides"], strict=True) if tt >= WARMUP_S]
    used, rows = set(), []
    for ev in events:
        inside = [
            (k, tt, s)
            for k, (tt, s) in enumerate(trig)
            if ev["t_gt"] + HIT_WINDOW[0] <= tt <= ev["t_end"]
        ]
        for k, _, _ in inside:
            used.add(k)
        first = inside[0] if inside else None
        rows.append(
            {
                **ev,
                "hit": first is not None,
                "latency_ms": None if first is None else round(1000 * (first[1] - ev["t_gt"]), 1),
                "side": None if first is None else first[2],
                "extra_triggers": max(0, len(inside) - 1),
                "one_channel_only": max(ev["min_left"], ev["min_right"]) >= REAL_ONSET_RAW - 300,
            }
        )
    fa = [tt for k, (tt, _) in enumerate(trig) if k not in used]
    in_events = sum(ev["t_end"] - ev["t_gt"] - HIT_WINDOW[0] for ev in events)
    quiet_min = max((t[-1] - WARMUP_S - in_events) / 60.0, 1e-9)
    return {"events": rows, "false_alarm_times": fa, "quiet_minutes": quiet_min}
