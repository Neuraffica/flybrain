"""The simulated world: a robot dog that turns in place and one target at a bearing around it.

A plain class, stepped by the caller (no threads, no sleeping), so tests can drive it. The
dynamics are those of ``flybrain_models.pursuit_tasks.run_episode``, with its ASSUMED rig
numbers (``Rig`` defaults): every world sub-step of ``Rig.world_dt`` the true relative bearing
is recorded; every WORLD_SUBSTEPS sub-steps the camera makes a frame that reports the bearing
of ``Rig.delay_s`` earlier, plus noise, unless the target was absent, outside the field of
view or the frame was dropped; the controller is stepped once per frame; its turn command is
clipped to the turn-rate limit and followed with a first-order lag; the heading integrates.

Angles in radians, positive = left (counter-clockwise), as in the model. ``theta`` is the
target's bearing in the world, ``heading`` the dog's; the bearing error is
``wrap(theta - heading)``, the target's bearing relative to the body.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import replace

import numpy as np
from flybrain_models.pursuit_tasks import FOVS, WITHIN_DEG, WORLD_SUBSTEPS, Rig

from .controllers import CONTROLLERS, is_circuit

MOTIONS = ("still", "sweep", "walk", "jumps", "hidden")
FOV_CHOICES = tuple(int(f) for f in FOVS)  # 30, 60, 100

# Target motions (choices for the demo, not part of the benchmark rig).
SWEEP_AMP_DEG = 40.0  # slow sweep: +-40 deg around where the target was, 10 s period
SWEEP_PERIOD_S = 10.0
WALK_TAU_S = 1.0  # random walk: the target's angular velocity is an OU process, as in the
WALK_SD_DEG_S = 30.0  # benchmark's "walk" track (time constant 1 s, sd 30 deg/s)
JUMP_EVERY_S = 4.0  # step jumps: every 4 s, to 0.35 / 0.65 / 0.9 of the half field of view,
JUMP_FRACTIONS = (0.35, 0.65, 0.9)  # alternating sides, measured from the dog's heading


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class PursuitSim:
    def __init__(
        self,
        controller: str = "connectome",
        fov_deg: float = Rig().fov_half_deg,
        motion: str = "still",
        arousal: float = 1.0,
        seed: int = 0,
        target_deg: float = 20.0,
    ):
        self.rng = np.random.default_rng(seed)
        self.rig = Rig()
        self._models: dict[str, object] = {}
        self.t = 0.0
        self.heading = 0.0  # rad, world
        self.rate = 0.0  # rad/s, actual turn rate (after the lag)
        self.cmd = 0.0  # rad/s, controller command after the rate limit
        self.turn = 0.0  # rad/s, controller command before the limit
        self.theta = math.radians(target_deg)  # rad, target bearing in the world
        self.exists = True
        self.motion = "still"
        self.arousal = 1.0
        self.measured: float | None = None  # bearing the camera reported in the last frame
        self.camera = "seen"  # seen | hidden | outside | dropped | waiting
        self.frames = 0
        self._i = 0  # world sub-steps since start
        self._acc = 0.0  # real time not yet simulated
        self._past: deque = deque()
        self._motion_t = 0.0
        self._sweep_centre = 0.0
        self._walk_v = 0.0
        self._jump_n = 0
        self.set_fov(fov_deg)
        self.set_arousal(arousal)
        self.set_controller(controller)
        self.set_motion(motion)
        self.reset_stats()

    # ---- settings (each returns an error string or None) ---------------------------
    def set_controller(self, key: str) -> str | None:
        if key not in CONTROLLERS:
            return f"unknown controller {key!r}"
        if key not in self._models:
            self._models[key] = CONTROLLERS[key].make()
        self.controller = key
        self.model = self._models[key]
        self.model.reset()  # the controller starts afresh; the world (heading, turn rate,
        self.turn = self.cmd = 0.0  # target, camera) carries on
        return None

    def set_fov(self, deg) -> str | None:
        deg = _number(deg)
        if deg is None or round(deg) not in FOV_CHOICES:
            return f"field of view must be one of {', '.join(map(str, FOV_CHOICES))} deg"
        self.rig = replace(self.rig, fov_half_deg=float(round(deg)))
        self._delay = round(self.rig.delay_s / self.rig.world_dt)
        self._past = deque(self._past, maxlen=self._delay + 1)
        return None

    def set_arousal(self, value) -> str | None:
        v = _number(value)
        if v is None or not 0.0 <= v <= 1.0:
            return "arousal must be between 0 and 1"
        self.arousal = float(v)
        return None

    def set_motion(self, name: str) -> str | None:
        if name not in MOTIONS:
            return f"unknown target motion {name!r}; choose one of {', '.join(MOTIONS)}"
        self.motion = name
        self.exists = name != "hidden"
        self._motion_t = 0.0
        self._sweep_centre = self.theta
        self._walk_v = 0.0
        if name == "jumps":
            self._jump()
        return None

    def set_target(self, deg) -> str | None:
        """Put the target at this world bearing (deg) and hold it there."""
        d = _number(deg)
        if d is None:
            return "target bearing must be a number of degrees"
        self.theta = wrap(math.radians(d))
        if self.motion != "still":
            self.set_motion("still")
        self.exists = True
        return None

    def nudge_target(self, deg) -> str | None:
        d = _number(deg)
        if d is None or abs(d) > 180:
            return "nudge must be a number of degrees, at most 180"
        return self.set_target(math.degrees(self.theta) + d)

    def reset_stats(self) -> None:
        self.stats_t = 0.0  # seconds with a target
        self._sum_err = 0.0
        self._within = 0.0
        self._in_view = 0.0

    # ---- simulation ------------------------------------------------------------------
    @property
    def error(self) -> float:
        """Bearing error now (rad): the target's bearing relative to the body."""
        return wrap(self.theta - self.heading)

    @property
    def fov(self) -> float:
        return math.radians(self.rig.fov_half_deg)

    def advance(self, seconds: float) -> int:
        """Simulate ``seconds`` of world time (carrying over any remainder); returns the
        number of world sub-steps run."""
        self._acc += max(0.0, float(seconds))
        dt = self.rig.world_dt
        n = int(self._acc / dt + 1e-9)
        self._acc -= n * dt
        for _ in range(n):
            self.substep()
        return n

    def substep(self) -> None:
        rig, dt = self.rig, self.rig.world_dt
        a = self.error
        self._past.append((a, self.exists))
        if self._i % WORLD_SUBSTEPS == 0:
            self._frame(dt * WORLD_SUBSTEPS)
        if self.exists:
            e = abs(math.degrees(a))
            self.stats_t += dt
            self._sum_err += e * dt
            self._within += dt if e <= WITHIN_DEG else 0.0
            self._in_view += dt if e <= rig.fov_half_deg else 0.0
        vmax = math.radians(rig.max_turn_deg_s)
        self.cmd = min(vmax, max(-vmax, self.turn))
        self.rate += (1.0 - math.exp(-dt / rig.lag_s)) * (self.cmd - self.rate)
        self.heading = wrap(self.heading + self.rate * dt)
        self._move_target(dt)
        self._i += 1
        self.t += dt

    def _frame(self, dt_frame: float) -> None:
        noise = self.rng.normal(0.0, math.radians(self.rig.noise_deg))
        dropped = self.rng.random() < self.rig.dropout
        meas = None
        if len(self._past) <= self._delay:
            self.camera = "waiting"  # the first frames after start: nothing captured yet
        else:
            past_a, past_exists = self._past[0]  # captured delay_s ago
            if not past_exists:
                self.camera = "hidden"
            elif abs(past_a) > self.fov:
                self.camera = "outside"
            elif dropped:
                self.camera = "dropped"
            else:
                self.camera = "seen"
                meas = past_a + noise
        self.measured = meas
        self.frames += 1
        self.turn = self.model.step(meas, dt_frame, arousal=self.arousal).turn

    def _move_target(self, dt: float) -> None:
        self._motion_t += dt
        if self.motion == "sweep":
            amp = math.radians(SWEEP_AMP_DEG)
            phase = 2 * math.pi * self._motion_t / SWEEP_PERIOD_S
            self.theta = wrap(self._sweep_centre + amp * math.sin(phase))
        elif self.motion == "walk":
            a = math.exp(-dt / WALK_TAU_S)
            sd = math.radians(WALK_SD_DEG_S)
            self._walk_v = a * self._walk_v + sd * math.sqrt(1 - a * a) * self.rng.normal()
            self.theta = wrap(self.theta + self._walk_v * dt)
        elif self.motion == "jumps" and self._motion_t >= JUMP_EVERY_S:
            self._motion_t = 0.0
            self._jump()

    def _jump(self) -> None:
        frac = JUMP_FRACTIONS[(self._jump_n // 2) % len(JUMP_FRACTIONS)]
        side = 1.0 if self._jump_n % 2 == 0 else -1.0
        self._jump_n += 1
        self.theta = wrap(self.heading + side * frac * self.fov)

    # ---- what the dashboard shows ------------------------------------------------------
    def stats(self) -> dict:
        t = self.stats_t
        return {
            "seconds": round(t, 1),
            "mae_deg": round(self._sum_err / t, 2) if t > 0 else None,
            "within10": round(self._within / t, 3) if t > 0 else None,
            "in_view": round(self._in_view / t, 3) if t > 0 else None,
        }

    def snapshot(self) -> dict:
        deg = math.degrees
        rig = self.rig
        return {
            "t": round(self.t, 3),
            "heading_deg": round(deg(self.heading), 2),
            "target": {
                "exists": self.exists,
                "bearing_deg": round(deg(self.theta), 2),
                "motion": self.motion,
            },
            "error_deg": round(deg(self.error), 2) if self.exists else None,
            "camera": self.camera,
            "seen": self.measured is not None,
            "measured_deg": None if self.measured is None else round(deg(self.measured), 2),
            "turn_deg_s": round(deg(self.turn), 2),
            "cmd_deg_s": round(deg(self.cmd), 2),
            "rate_deg_s": round(deg(self.rate), 2),
            "controller": self.controller,
            "circuit": is_circuit(self.model),
            "arousal": self.arousal,
            "fov_deg": rig.fov_half_deg,
            "rig": {
                "rate_hz": rig.rate_hz,
                "delay_s": rig.delay_s,
                "noise_deg": rig.noise_deg,
                "dropout": rig.dropout,
                "max_turn_deg_s": rig.max_turn_deg_s,
                "lag_s": rig.lag_s,
            },
            "stats": self.stats(),
        }


def _number(v) -> float | None:
    if isinstance(v, bool):
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None
