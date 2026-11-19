"""Closed-loop target pursuit in simulation, for fitting and benchmarking the pursuit models.

World: the robot turns in place; its heading ``h(t)`` and the target's bearing in the world
``theta(t)`` (radians, + = left) give the true relative bearing ``wrap(theta - h)``.

Sensor: a camera that reports the centre of a detection box as a bearing. Actuator: a turn
rate command, clipped to a rate limit and followed with a first-order lag. ALL the numbers
below are ASSUMED defaults for a small robot with a microcontroller camera; none of them was
measured on the hardware:

=====================  ==========  ===================================================
SENSOR_RATE_HZ         15 Hz       frames per second; the controller runs once a frame
SENSOR_DELAY_S         0.10 s      capture to report
SENSOR_NOISE_DEG       2 deg       Gaussian noise on the reported bearing
FOV_HALF_DEG           30 deg      outside +-30 deg the target is not reported
FOV_WIDE_HALF_DEG      60 deg      the second field-of-view setting
FOV_BODY_HALF_DEG      100 deg     the third setting: head camera + neck pan (see below)
DROPOUT                5 %         frames in which a visible target is not reported
MAX_TURN_DEG_S         90 deg/s    turn-rate limit of the robot
LAG_S                  0.15 s      first-order lag between command and turn rate
=====================  ==========  ===================================================

The +-100 deg setting is ASSUMED to stand for a camera (+-30 deg) on a head that pans on the
neck (assumed about +-70 deg) and keeps the camera on the target, so that the target's bearing
relative to the BODY is known over about +-100 deg. The head loop itself is NOT simulated: the
sensor simply reports the body-relative bearing within +-100 deg, with the same rate, delay,
noise and dropouts as the other settings.

Target tracks: ``step`` (the target jumps by a fixed angle inside the field of view at
t = 1 s), ``appear`` (+-100 deg setting only: no target until t = 1 s, then a target at
60-95 deg to one side), ``sine`` (slow sinusoid, 0.05-0.25 Hz, 15-40 deg; 40-90 deg in the
+-100 deg setting), ``walk`` (low-pass random walk: the target's angular velocity is an
Ornstein-Uhlenbeck process, time constant 1 s, sd 30 deg/s), ``pass`` (from t = 1 s the target
sweeps at 60-180 deg/s and then stops: 120 deg to one side starting straight ahead in the
+-30/+-60 settings, at the faster speeds leaving the field of view; in the +-100 setting from
90 deg on one side across the whole field to 90 deg on the other). Noise and dropouts are drawn
per frame when the episode is made, so every model sees the same random numbers.

Metrics (per episode; aggregated as time-weighted means): mean absolute bearing error while
the target exists, fraction of time within +-10 deg, fraction of time inside the field of view,
and for steps and appearances the settling time (time after the jump until the error stays
within +-5 deg; None if it never does) and the overshoot (largest error past the target, deg). Per model:
the idle turn, |turn rate| the robot would make after 2 s with no target reported.

Loss (fitting, every model): mean absolute bearing error (deg) + IDLE_WEIGHT * idle turn
(deg/s). The idle term encodes a requirement, "do not turn when nothing is seen"; without it
the first circuit fit learned a tonic left spin that finds lost targets by going round.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# ---- ASSUMED rig parameters (not measured) -----------------------------------------------
SENSOR_RATE_HZ = 15.0
SENSOR_DELAY_S = 0.10
SENSOR_NOISE_DEG = 2.0
FOV_HALF_DEG = 30.0
FOV_WIDE_HALF_DEG = 60.0
FOV_BODY_HALF_DEG = 100.0  # ASSUMED head camera +-30 deg on a neck panning about +-70 deg
FOVS = (FOV_HALF_DEG, FOV_WIDE_HALF_DEG, FOV_BODY_HALF_DEG)
STEP_KINDS = ("step", "appear")
DROPOUT = 0.05
MAX_TURN_DEG_S = 90.0
LAG_S = 0.15
WORLD_SUBSTEPS = 10  # world integration steps per sensor frame

# ---- scoring ------------------------------------------------------------------------------
WITHIN_DEG = 10.0
SETTLE_BAND_DEG = 5.0
STEP_AT_S = 1.0
IDLE_S = 2.0
IDLE_WEIGHT = 1.0  # loss degrees per deg/s of turning with no target


@dataclass(frozen=True)
class Rig:
    rate_hz: float = SENSOR_RATE_HZ
    delay_s: float = SENSOR_DELAY_S
    noise_deg: float = SENSOR_NOISE_DEG
    fov_half_deg: float = FOV_HALF_DEG
    dropout: float = DROPOUT
    max_turn_deg_s: float = MAX_TURN_DEG_S
    lag_s: float = LAG_S

    @property
    def world_dt(self) -> float:
        return 1.0 / (self.rate_hz * WORLD_SUBSTEPS)


@dataclass
class Episode:
    name: str
    kind: str  # step | appear | sine | walk | pass
    t: np.ndarray  # world time grid (s), step Rig.world_dt
    theta: np.ndarray  # target bearing in the world (rad, + = left)
    exists: np.ndarray  # bool, target present
    noise: np.ndarray  # per-frame bearing noise (rad), already scaled
    drop: np.ndarray  # per-frame dropout flags
    rig: Rig
    step: float = 0.0  # step size / appearance bearing (rad) for kind "step" / "appear"
    meta: dict = field(default_factory=dict)

    @property
    def duration(self) -> float:
        return float(self.t[-1] + self.rig.world_dt)


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def _track(kind: str, t: np.ndarray, rng: np.random.Generator, **kw) -> np.ndarray:
    if kind in STEP_KINDS:
        return np.where(t >= STEP_AT_S, kw["step"], 0.0)
    if kind == "sine":
        return kw["amp"] * np.sin(2 * np.pi * kw["freq"] * t)
    if kind == "walk":
        dt = t[1] - t[0]
        tau, sd = 1.0, math.radians(30.0)
        v = np.empty_like(t)
        v[0] = 0.0
        a = math.exp(-dt / tau)
        z = rng.standard_normal(len(t))
        for i in range(1, len(t)):
            v[i] = a * v[i - 1] + sd * math.sqrt(1 - a * a) * z[i]
        return np.cumsum(v) * dt
    if kind == "pass":
        s, speed = kw["direction"], kw["speed"]
        start, sweep = kw.get("start", 0.0), kw.get("sweep", math.radians(120.0))
        return s * (-start + np.clip((t - STEP_AT_S) * speed, 0.0, sweep))
    raise ValueError(kind)


def make_episode(
    kind: str, rng: np.random.Generator, rig: Rig | None = None, duration: float = 0.0, **kw
) -> Episode:
    rig = rig or Rig()
    duration = duration or {"step": 5.0, "appear": 5.0, "sine": 12.0, "walk": 12.0}.get(kind, 6.0)
    n = round(duration / rig.world_dt)
    t = np.arange(n) * rig.world_dt
    theta = _track(kind, t, rng, **kw)
    frames = n // WORLD_SUBSTEPS + 1
    noise = rng.normal(0.0, math.radians(rig.noise_deg), frames)
    drop = rng.random(frames) < rig.dropout
    desc = ",".join(
        f"{k}={math.degrees(v):.0f}deg"
        if k in ("step", "amp", "speed", "start", "sweep")
        else f"{k}={v:.3g}"
        for k, v in kw.items()
    )
    return Episode(
        f"{kind}({desc})_fov{rig.fov_half_deg:g}",
        kind,
        t,
        theta,
        t >= STEP_AT_S if kind == "appear" else np.ones(n, dtype=bool),
        noise,
        drop,
        rig,
        step=float(kw.get("step", 0.0)),
        meta={k: float(v) for k, v in kw.items()},
    )


def episode_set(seed: int, reps: int, fovs=FOVS) -> list[Episode]:
    """Per field of view and repetition: six steps (0.35, 0.65, 0.9 of the half field of view,
    both directions; 35/65/90 deg at +-100), two sinusoids, two random walks and two passes
    (one at 60-100 deg/s, one at 120-180 deg/s, random direction); at +-100 deg also two
    appearances (60-95 deg, one each side). 12 episodes per repetition at +-30 and +-60 deg,
    14 at +-100 deg. The +-30/+-60 episodes are drawn first, so they are the same as before
    the +-100 setting was added."""
    rng = np.random.default_rng(seed)
    out = []
    for fov in fovs:
        rig = Rig(fov_half_deg=fov)
        body = fov >= FOV_BODY_HALF_DEG
        for _ in range(reps):
            for frac in (0.35, 0.65, 0.9):
                for s in (1, -1):
                    out.append(make_episode("step", rng, rig, step=s * math.radians(frac * fov)))
            if body:
                for s in (1, -1):
                    at = s * math.radians(rng.uniform(60.0, 95.0))
                    out.append(make_episode("appear", rng, rig, step=at))
            for _ in range(2):
                lo, hi = (40.0, 90.0) if body else (15.0, 40.0)
                amp = math.radians(rng.uniform(lo, hi))
                out.append(make_episode("sine", rng, rig, amp=amp, freq=rng.uniform(0.05, 0.25)))
                out.append(make_episode("walk", rng, rig))
            for lo, hi in ((60.0, 100.0), (120.0, 180.0)):
                kw = {"start": math.radians(90.0), "sweep": math.radians(180.0)} if body else {}
                out.append(
                    make_episode(
                        "pass",
                        rng,
                        rig,
                        direction=float(rng.choice([-1, 1])),
                        speed=math.radians(rng.uniform(lo, hi)),
                        **kw,
                    )
                )
    return out


def training_set() -> list[Episode]:
    """One repetition per field of view (38 episodes); seed disjoint from the benchmark."""
    return episode_set(seed=1001, reps=1)


def benchmark_set() -> list[Episode]:
    """Three repetitions per field of view (114 episodes), different seed."""
    return episode_set(seed=2026, reps=3)


# ---- running and scoring -------------------------------------------------------------------
def run_episode(model, ep: Episode, arousal: float = 1.0) -> dict:
    """Closed loop: world at Rig.world_dt, the model stepped once per frame with the bearing
    captured SENSOR_DELAY_S earlier (or None if the target was out of view, absent or dropped)."""
    rig = ep.rig
    model.reset()
    n = len(ep.t)
    dt = rig.world_dt
    dt_frame = dt * WORLD_SUBSTEPS
    delay = round(rig.delay_s / dt)
    fov = math.radians(rig.fov_half_deg)
    vmax = math.radians(rig.max_turn_deg_s)
    lag = 1.0 - math.exp(-dt / rig.lag_s)
    theta, exists = ep.theta, ep.exists
    az = np.empty(n)
    cmd = np.zeros(n)
    seen = []
    h = rate = c = 0.0
    for i in range(n):
        a = (theta[i] - h + math.pi) % (2 * math.pi) - math.pi
        az[i] = a
        if i % WORLD_SUBSTEPS == 0:
            k = i // WORLD_SUBSTEPS
            j = i - delay
            meas = None
            if j >= 0 and exists[j] and abs(az[j]) <= fov and not ep.drop[k]:
                meas = float(az[j] + ep.noise[k])
            seen.append(meas is not None)
            out = model.step(meas, dt_frame, arousal=arousal)
            c = min(vmax, max(-vmax, out.turn))
        cmd[i] = c
        rate += lag * (c - rate)
        h += rate * dt
    return {"az": az, "cmd": cmd, "seen": np.array(seen)}


def score_episode(ep: Episode, res: dict) -> dict:
    az = np.degrees(res["az"])
    ex = ep.exists
    err = np.abs(az[ex])
    out = {
        "kind": ep.kind,
        "fov": ep.rig.fov_half_deg,
        "seconds": float(ex.sum() * ep.rig.world_dt),
        "mae_deg": float(err.mean()),
        "within10": float((err <= WITHIN_DEG).mean()),
        "in_view": float((err <= ep.rig.fov_half_deg).mean()),
        "settle_s": None,
        "overshoot_deg": None,
    }
    if ep.kind in STEP_KINDS:
        after = ep.t >= STEP_AT_S
        e = az[after]
        outside = np.flatnonzero(np.abs(e) > SETTLE_BAND_DEG)
        if len(outside) == 0:
            out["settle_s"] = 0.0
        elif outside[-1] < len(e) - 1:
            out["settle_s"] = float((outside[-1] + 1) * ep.rig.world_dt)
        s = math.copysign(1.0, ep.step)
        out["overshoot_deg"] = float(max(0.0, (-s * e).max()))
        out["step_deg"] = float(math.degrees(ep.step))
    return out


def summarize(scores: list[dict]) -> dict:
    """Time-weighted means over episodes, plus step statistics."""
    if not scores:
        return {}
    w = np.array([s["seconds"] for s in scores])

    def tw(key):
        return float(np.average([s[key] for s in scores], weights=w))

    steps = [s for s in scores if s["kind"] in STEP_KINDS]
    settled = [s["settle_s"] for s in steps if s["settle_s"] is not None]
    return {
        "mae_deg": tw("mae_deg"),
        "within10": tw("within10"),
        "in_view": tw("in_view"),
        "n_episodes": len(scores),
        "step_settled_fraction": len(settled) / len(steps) if steps else None,
        "step_settle_s_median": float(np.median(settled)) if settled else None,
        "step_overshoot_deg_mean": (
            float(np.mean([s["overshoot_deg"] for s in steps])) if steps else None
        ),
    }


def idle_turn(model, arousal: float = 1.0, rig: Rig | None = None) -> float:
    """|turn rate| (deg/s, after the robot's rate limit) after IDLE_S seconds of frames with
    no target reported."""
    rig = rig or Rig()
    model.reset()
    turn = 0.0
    for _ in range(round(IDLE_S * rig.rate_hz)):
        turn = model.step(None, 1.0 / rig.rate_hz, arousal=arousal).turn
    model.reset()
    return min(abs(math.degrees(turn)), rig.max_turn_deg_s)


def evaluate(model, episodes, arousal: float = 1.0) -> tuple[dict, list[dict]]:
    scores = [score_episode(ep, run_episode(model, ep, arousal)) for ep in episodes]
    summary = summarize(scores)
    summary["idle_turn_deg_s"] = idle_turn(model, arousal)
    return summary, scores


def loss_of(summary: dict) -> float:
    """The fitting loss shared by every model: time-weighted mean absolute bearing error
    (degrees) over the training set + IDLE_WEIGHT * idle turn (deg/s)."""
    return float(summary["mae_deg"]) + IDLE_WEIGHT * float(summary["idle_turn_deg_s"])
