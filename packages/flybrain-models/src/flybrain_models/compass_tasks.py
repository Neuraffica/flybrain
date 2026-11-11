"""Simulated rotation trajectories for fitting and benchmarking the compass models.

A trajectory is a true angular-velocity trace ``omega`` (rad/s, positive = counter-clockwise)
sampled every ``dt`` seconds, the gyro reading the model sees (``measured``, optionally noisy)
and the true heading (integral of ``omega``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

DEG = math.pi / 180.0


@dataclass
class Trajectory:
    name: str
    dt: float
    omega: np.ndarray  # true angular velocity, rad/s
    measured: np.ndarray  # what the gyro reports, rad/s
    heading0: float = 0.0
    cue: np.ndarray | None = None  # landmark heading per step (rad); NaN = no cue this step

    @property
    def heading(self) -> np.ndarray:
        """True heading after each step (rad, unwrapped)."""
        return self.heading0 + np.cumsum(self.omega) * self.dt

    @property
    def duration(self) -> float:
        return len(self.omega) * self.dt


def _smooth(x: np.ndarray, dt: float, tau: float = 0.15) -> np.ndarray:
    """First-order low-pass so velocity steps look like a real robot turning."""
    a = dt / (tau + dt)
    y = np.empty_like(x)
    acc = 0.0
    for i, v in enumerate(x):
        acc += a * (v - acc)
        y[i] = acc
    return y


def piecewise(segments: list[tuple[float, float]], dt: float) -> np.ndarray:
    """``segments`` = [(velocity_deg_s, duration_s), ...] -> smoothed rad/s trace."""
    raw = np.concatenate([np.full(round(d / dt), v * DEG) for v, d in segments])
    return _smooth(raw, dt)


def add_noise(omega: np.ndarray, rng: np.random.Generator, white_deg_s: float = 10.0,
              bias_deg_s: float = 0.5) -> np.ndarray:
    """MEMS-gyro-like reading: white noise plus a small constant bias."""
    bias = bias_deg_s * DEG * rng.choice([-1.0, 1.0])
    return omega + bias + rng.normal(0.0, white_deg_s * DEG, size=omega.shape)


def random_segments(rng: np.random.Generator, duration: float,
                    vmax_deg_s: float = 200.0) -> list[tuple[float, float]]:
    segs, t = [], 0.0
    while t < duration:
        kind = rng.choice(["stop", "slow", "fast"], p=[0.25, 0.4, 0.35])
        if kind == "stop":
            v = 0.0
        elif kind == "slow":
            v = rng.uniform(10, 60) * rng.choice([-1, 1])
        else:
            v = rng.uniform(60, vmax_deg_s) * rng.choice([-1, 1])
        d = float(rng.uniform(0.5, 3.0))
        segs.append((float(v), d))
        t += d
    return segs


def training_set(seed: int = 123, n: int = 6, duration: float = 20.0,
                 dt: float = 0.02) -> list[Trajectory]:
    """Random turn sequences used for fitting (disjoint seeds from the benchmark)."""
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        om = piecewise(random_segments(rng, duration), dt)
        meas = add_noise(om, rng) if i % 2 else om.copy()
        out.append(Trajectory(f"train{i}", dt, om, meas, heading0=float(rng.uniform(-np.pi,
                                                                                      np.pi))))
    return out


def benchmark_set(dt: float = 0.02, seed: int = 2026) -> list[Trajectory]:
    """Fixed scenarios: slow/fast turns in both directions, stops, and a mixed sequence;
    each clean and with sensor noise."""
    scen = {
        "slow_ccw": [(0, 1), (30, 12), (0, 2)],
        "slow_cw": [(0, 1), (-30, 12), (0, 2)],
        "fast_ccw": [(0, 1), (180, 6), (0, 2)],
        "fast_cw": [(0, 1), (-180, 6), (0, 2)],
        "stop_and_go": [(0, 1), (90, 2), (0, 3), (-120, 2), (0, 3), (45, 4), (0, 3),
                        (-200, 1.5), (0, 3)],
        "mixed": random_segments(np.random.default_rng(seed), 40.0),
    }
    out = []
    rng = np.random.default_rng(seed + 1)
    for name, segs in scen.items():
        om = piecewise(segs, dt)
        out.append(Trajectory(name, dt, om, om.copy()))
        out.append(Trajectory(name + "+noise", dt, om, add_noise(om, rng)))
    return out


def stationary(duration: float = 60.0, dt: float = 0.02, noisy: bool = False,
               seed: int = 7) -> Trajectory:
    om = np.zeros(round(duration / dt))
    meas = add_noise(om, np.random.default_rng(seed)) if noisy else om.copy()
    return Trajectory("stationary" + ("+noise" if noisy else ""), dt, om, meas)


def with_cue(traj: Trajectory, rate_hz: float = 5.0, noise_deg: float = 0.0,
             seed: int = 0, until_s: float | None = None) -> Trajectory:
    """Copy of ``traj`` with a landmark cue = true heading sampled at ``rate_hz`` (plus
    Gaussian noise), None on the other steps; no cue after ``until_s`` seconds."""
    rng = np.random.default_rng(seed)
    n = len(traj.omega)
    every = max(1, round(1.0 / (rate_hz * traj.dt)))
    cue = np.full(n, np.nan)
    idx = np.arange(0, n, every)
    if until_s is not None:
        idx = idx[idx * traj.dt < until_s]
    cue[idx] = traj.heading[idx] + rng.normal(0.0, noise_deg * DEG, size=len(idx))
    suffix = f"+cue{rate_hz:g}Hz" + (f"~{noise_deg:g}deg" if noise_deg else "")
    return Trajectory(traj.name + suffix, traj.dt, traj.omega, traj.measured, traj.heading0,
                      cue=cue)


def run(model, traj: Trajectory, conf_threshold: float = 0.3) -> dict[str, np.ndarray]:
    """Run ``model`` over ``traj``; returns decoded heading, error and confidence per step."""
    model.reset(traj.heading0)
    n = len(traj.omega)
    dec = np.empty(n)
    conf = np.empty(n)
    cue = traj.cue
    for i in range(n):
        c = None if cue is None or np.isnan(cue[i]) else float(cue[i])
        dec[i] = model.step(float(traj.measured[i]), traj.dt, c)
        conf[i] = model.confidence
    err = np.angle(np.exp(1j * (dec - traj.heading)))
    return {"decoded": dec, "error": err, "confidence": conf,
            "coherent": conf > conf_threshold}


def memory_trial(model, heading: float, cue_s: float = 3.0, dark_s: tuple = (5.0, 30.0, 60.0),
                 dt: float = 0.02, rate_hz: float = 5.0, start: float | None = None,
                 omega_dark: float = 0.0, conf_threshold: float = 0.3) -> dict:
    """Heading memory. Reset at ``start`` (default: heading + 90 deg, so the cue has to move
    the bump), show the cue at ``heading`` at ``rate_hz`` for ``cue_s``, then darkness for
    max(dark_s) seconds while rotating at ``omega_dark`` (rad/s; 0 = stationary). Returns
    the error vs the true heading and confidence at the end of the cue and at each dark_s."""
    model.reset(heading + np.pi / 2 if start is None else start)
    every = max(1, round(1.0 / (rate_hz * dt)))
    n_cue = round(cue_s / dt)
    for i in range(n_cue):
        model.step(0.0, dt, heading if i % every == 0 else None)
    out = {"cue_end": (float(np.angle(np.exp(1j * (model.heading - heading)))),
                       model.confidence)}
    t, true = 0.0, heading
    for target in sorted(dark_s):
        while t < target - 1e-9:
            model.step(omega_dark, dt, None)
            true += omega_dark * dt
            t += dt
        out[target] = (float(np.angle(np.exp(1j * (model.heading - true)))), model.confidence)
    return out
