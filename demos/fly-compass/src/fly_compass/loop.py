"""The compass control loop, independent of where samples come from and of wall time.

Signal path, per tick:

    IMU fused yaw (deg, ~5 Hz)  --unwrap-->  truth (relative to the zero heading)
        --> finite difference of consecutive samples + slow drift correction
        --> angular velocity omega (every tick)
        --> cue: the same yaw, as a landmark, on ticks that bring a fresh sample, unless the
            demo is "blindfolded" (then None on every tick)
        --> step connectome ring, rewired ring, gyro integrator with the same (omega, dt, cue)
        --> needle: head pan = target - heading of the driving model (clamped, smoothed,
            rate-limited);  hold: in-place turn skills with hysteresis;
            sweep: the dog turns itself between +A and -A (measured heading) while the
            needle keeps working

Angular velocity is DERIVED FROM THE IMU's fused yaw: the OpenCatEsp32 firmware prints yaw,
pitch, roll and acceleration but no raw gyro rate (src/imu.h:599-611). omega is the rate
between the last two samples (piecewise constant, so the models get a smooth input) plus a
small term (yaw - integral of omega) / tau_correct that pulls the integral of omega back onto
the measured yaw (extrapolated to the current tick), so the input itself never drifts. Every
model sees a turn about one sample (~0.2 s) late.

The cue stands in for a visual landmark. "Truth" for scoring is that same fused yaw, so with
the cue on the error mostly measures lag; the informative numbers are the blindfolded ones:
does the bump stay put when the dog is still, and does anything follow a turn in the dark.
"""

from __future__ import annotations

import inspect
import math
import time
from dataclasses import dataclass, field
from typing import Any

from .models import MODEL_KEYS, bump_of, confidence_of, n_neurons_of

MODES = ("needle", "hold", "sweep", "observe")
LEG_MODES = ("hold", "sweep")  # modes that make the legs step: need --i-have-space

TURN_LEFT, TURN_RIGHT, STAND = "vtL", "vtR", "up"  # in-place turning gaits and the stand posture

# Sweep gait watchdog. On a real Bittle (firmware B10_250330) it is not certain that a turning
# gait keeps running while head commands are streamed, so a leg that makes too little progress
# re-issues its turn skill. Re-sending the same gait is harmless on the firmware: it does not
# reload a gait that is already running (src/reaction.h:1467-1471).
SWEEP_STALL_WINDOW_S = 3.0  # look back this far ...
SWEEP_STALL_MIN_DEG = 3.0  # ... and expect at least this much turn in the commanded direction
SWEEP_REISSUE_S = 3.0  # never re-issue the turn skill more often than this
SWEEP_MAX_TIMEOUTS = 3  # consecutive leg timeouts before sweep gives up (falls back to needle)


def wrap_deg(a: float) -> float:
    a = math.fmod(a + 180.0, 360.0)
    if a <= 0:
        a += 360.0
    return a - 180.0


CUE_LINK_HOLD_S = 2.0  # keep giving the last cue across link gaps this long
CUE_REISSUE_S = 0.19  # just under the models' 0.25 s cue hold and the 5 Hz sample period
RATE_WINDOW_S = 0.04  # shortest span over which a yaw rate is estimated


@dataclass
class LoopConfig:
    rate_hz: float = 20.0
    tau_correct_s: float = 0.5  # pulls the integral of omega back onto the measured yaw
    omega_clip_dps: float = 720.0
    stale_after_s: float = 1.0
    # needle
    pan_limit_deg: float = 80.0
    pan_sign: int = +1  # +1: positive pan looks left (CCW). UNVERIFIED on hardware.
    pan_rate_dps: float = 120.0
    pan_smooth_tau_s: float = 0.12
    pan_deadband_deg: float = 1.0
    # hold
    hold_enter_deg: float = 20.0
    hold_exit_deg: float = 6.0
    hold_max_turn_s: float = 6.0
    hold_cooldown_s: float = 1.5
    hold_max_failures: int = 3
    # sweep: turn left to +sweep_angle, pause, turn right to -sweep_angle, pause, repeat
    sweep_angle_deg: float = 35.0
    sweep_pause_s: float = 3.0
    sweep_timeout_s: float = 25.0  # per turning leg
    sweep_cycles: int | None = None  # stop after this many left+right cycles (None: forever)


@dataclass
class Actions:
    pan_deg: int | None = None  # head pan command to send (already clamped)
    skill: str | None = None  # skill to start, e.g. "vtL", "vtR", "up"
    notes: list[str] = field(default_factory=list)


@dataclass
class ErrStats:
    """|error| and confidence accumulated over scored IMU samples."""

    abs_err_sum: float = 0.0
    n: int = 0
    conf_sum: float = 0.0
    n_conf: int = 0
    last_err: float | None = None
    duration_s: float = 0.0

    def add(self, err: float, conf: float | None) -> None:
        self.abs_err_sum += abs(err)
        self.n += 1
        self.last_err = err
        if conf is not None:
            self.conf_sum += conf
            self.n_conf += 1

    @property
    def mae(self) -> float | None:
        return self.abs_err_sum / self.n if self.n else None

    @property
    def mean_conf(self) -> float | None:
        return self.conf_sum / self.n_conf if self.n_conf else None

    def summary(self) -> dict[str, Any]:
        r = lambda x: None if x is None else round(x, 2)
        return {
            "mae_deg": r(self.mae),
            "final_err_deg": r(self.last_err),
            "mean_conf": r(self.mean_conf),
            "samples": self.n,
        }


@dataclass
class ModelState:
    heading_deg: float = 0.0
    err_deg: float = 0.0
    ms_ema: float = 0.0
    cue: ErrStats = field(default_factory=ErrStats)  # scored while the cue is on
    blind: ErrStats = field(default_factory=ErrStats)  # scored while blindfolded
    phases: dict[str, ErrStats] = field(default_factory=dict)  # by phase label
    accepts_cue: bool = True

    @property
    def n(self) -> int:
        return self.cue.n + self.blind.n

    @property
    def mae(self) -> float | None:
        n = self.n
        return (self.cue.abs_err_sum + self.blind.abs_err_sum) / n if n else None


def accepts_cue(model: Any) -> bool:
    """True if ``model.step`` takes ``cue_rad`` (older model builds do not)."""
    try:
        params = inspect.signature(model.step).parameters
    except (TypeError, ValueError):
        return False
    return "cue_rad" in params or any(p.kind is p.VAR_KEYWORD for p in params.values())


class CompassLoop:
    def __init__(
        self,
        models: dict[str, Any],
        config: LoopConfig | None = None,
        mode: str = "observe",
        allow_hold: bool = False,
        driver: str = "connectome",
    ):
        missing = [k for k in MODEL_KEYS if k not in models]
        if missing:
            raise ValueError(f"missing models: {missing}")
        self.models = models
        self.cfg = config or LoopConfig()
        self.allow_hold = allow_hold
        # legs: True from issuing a turn gait until "up" has been issued
        self.gait_active = False
        self._stop_requested = False
        self._init_sweep()
        self.mode = "observe"
        self.set_mode(mode)
        self.driver = driver if driver in models else "connectome"
        self.state = {k: ModelState(accepts_cue=accepts_cue(models[k])) for k in MODEL_KEYS}
        # landmark cue / blindfold
        self.blindfolded = False
        self.blind_since: float | None = None
        self.blind_truth0 = 0.0
        self.phase_override: str | None = None  # set by a scripted scenario
        self.cue_this_tick = False
        self.n_cues = 0
        # yaw bookkeeping
        self._last_raw: float | None = None
        self._rate_anchor_t: float | None = None
        self._last_cue_t: float | None = None
        self._rate_anchor_yaw = 0.0
        self.yaw_unwrapped = 0.0
        self.zero_offset = 0.0
        self.zeroed = False
        self.fed = 0.0
        self.omega_dps = 0.0
        self.interval_ema = 0.2
        self.rate_est = 0.0
        self.last_sample_t: float | None = None
        self.n_samples = 0
        self.last_tick: float | None = None
        self.t_start: float | None = None
        # needle
        self.pan_filtered = 0.0
        self.pan_cmd = 0.0
        self.pan_sent: int | None = None
        self.pan_desired = 0.0
        self.pan_saturated = False
        # hold
        self.hold_state = "idle"
        self.hold_dir = 0
        self.hold_since = 0.0
        self.hold_failures = 0
        self.events: list[str] = []

    # ---- control surface ----------------------------------------------------------------
    def set_mode(self, mode: str) -> str | None:
        """Returns an error message, or None when the mode was applied."""
        if mode not in MODES:
            return f"unknown mode {mode!r}"
        if mode in LEG_MODES and not self.allow_hold:
            return f"{mode} mode moves the legs: restart with --i-have-space to allow it"
        if mode != self.mode:
            if self.gait_active:
                self._stop_requested = True  # the next update() issues "up"
            self.hold_state = "idle"
            self._init_sweep()
        self.mode = mode
        return None

    def _init_sweep(self) -> None:
        self.sweep_state = "idle"  # idle | turning | pausing | done
        self.sweep_dir = 0  # +1 turning/next turn left, -1 right
        self.sweep_since = 0.0
        self.sweep_cycles_done = 0
        self.sweep_timeouts = 0  # consecutive
        self.sweep_reissues = 0
        self.sweep_last_issue = 0.0
        self.sweep_legs_done = 0
        self._sweep_hist: list[tuple[float, float]] = []

    def set_driver(self, key: str) -> str | None:
        if key not in self.models:
            return f"unknown model {key!r}"
        self.driver = key
        return None

    def rezero(self) -> None:
        """Capture the current IMU heading as the target and restart every model at 0."""
        self.zero_offset = self.yaw_unwrapped
        self.fed = 0.0
        self.omega_dps = 0.0
        self.rate_est = 0.0
        for m in self.models.values():
            m.reset(heading_rad=0.0)
        self.reset_scores()
        self.blind_truth0 = 0.0  # truth_rel is 0 right after re-zeroing
        self.zeroed = True
        self.events.append("re-zeroed")

    def reset_scores(self) -> None:
        for s in self.state.values():
            s.cue = ErrStats()
            s.blind = ErrStats()
            s.phases = {}

    def set_blindfold(self, on: bool, now: float | None = None) -> None:
        """Blindfolded: no cue on any tick; the models run on recurrence and velocity alone."""
        on = bool(on)
        if on == self.blindfolded:
            return
        self.blindfolded = on
        self.blind_since = (self.last_tick if now is None else now) if on else None
        self.blind_truth0 = self.truth_rel
        self.events.append("blindfold on" if on else "blindfold off: cue restored")

    @property
    def phase(self) -> str:
        if self.phase_override:
            return self.phase_override
        return "blindfolded" if self.blindfolded else "cue on"

    @property
    def truth_rel(self) -> float:
        return self.yaw_unwrapped - self.zero_offset

    # ---- main update ----------------------------------------------------------------
    def ingest(self, samples) -> None:
        for s in samples:
            y = float(s.yaw_deg)
            step = 0.0 if self._last_raw is None else wrap_deg(y - self._last_raw)
            if self._last_raw is None:
                self.yaw_unwrapped = y
            else:
                self.yaw_unwrapped += step
            self._last_raw = y
            # Rate over a window of at least RATE_WINDOW_S, not between neighbouring samples.
            # Newer firmware prints at most 5 Hz, so each sample is its own window; firmware
            # B10_250330 streams ~360 lines/s in USB bursts with 0.1 deg resolution, where a
            # sample-to-sample difference is either zero or a huge spike.
            if self._rate_anchor_t is None:
                self._rate_anchor_t, self._rate_anchor_yaw = s.t, self.yaw_unwrapped
            else:
                dt = s.t - self._rate_anchor_t
                if dt >= 2.0:
                    self.rate_est = 0.0
                    self._rate_anchor_t, self._rate_anchor_yaw = s.t, self.yaw_unwrapped
                elif dt >= RATE_WINDOW_S:
                    self.interval_ema = 0.8 * self.interval_ema + 0.2 * dt
                    self.rate_est = (self.yaw_unwrapped - self._rate_anchor_yaw) / dt
                    self._rate_anchor_t, self._rate_anchor_yaw = s.t, self.yaw_unwrapped
            self.last_sample_t = s.t
            self.n_samples += 1
        if samples and not self.zeroed:
            self.rezero()

    def update(self, now: float, samples=()) -> Actions:
        self.ingest(samples)
        if self.t_start is None:
            self.t_start = now
        dt = 0.0 if self.last_tick is None else min(max(now - self.last_tick, 0.0), 0.25)
        self.last_tick = now
        acts = Actions()
        if not self.zeroed or dt <= 0:
            return acts

        # angular velocity derived from the IMU yaw (see module docstring)
        target = self.truth_rel
        if self.last_sample_t is not None and now - self.last_sample_t > self.cfg.stale_after_s:
            self.rate_est = 0.0  # link stalled: do not keep integrating an old rate
        # compare against the yaw extrapolated to `now` (samples are sample-and-hold), so the
        # correction only removes genuine mismatch, not the 0-200 ms staircase
        ahead = 0.0
        if self.last_sample_t is not None:
            ahead = min(max(now - self.last_sample_t, 0.0), 1.5 * self.interval_ema)
        omega = self.rate_est + (target + self.rate_est * ahead - self.fed) / self.cfg.tau_correct_s
        clip = self.cfg.omega_clip_dps
        omega = max(-clip, min(clip, omega))
        self.fed += omega * dt
        self.omega_dps = omega
        w = math.radians(omega)

        truth = wrap_deg(target)
        # Landmark cue: the measured yaw in the models' heading frame (re-zeroed to 0), only on
        # ticks with a fresh sample, never while blindfolded.
        # A short silence on the link is not darkness: a real Bittle (firmware B10_250330)
        # stops answering for about 1.3 s every 10 s, and the ring would slide to a preferred
        # position in that time and twitch the head. So the last cue is held for up to
        # CUE_LINK_HOLD_S after the latest sample; only the blindfold removes it.
        # The held cue is re-issued only when the last one is CUE_REISSUE_S old, so normal
        # operation still gives a cue just on fresh samples.
        link_gap = (
            not samples
            and self.last_sample_t is not None
            and now - self.last_sample_t <= CUE_LINK_HOLD_S
            and self._last_cue_t is not None
            and now - self._last_cue_t >= CUE_REISSUE_S
        )
        cue_rad = math.radians(truth) if (samples or link_gap) and not self.blindfolded else None
        if cue_rad is not None:
            self._last_cue_t = now
        self.cue_this_tick = cue_rad is not None
        self.n_cues += int(self.cue_this_tick)
        phase = self.phase
        for key in MODEL_KEYS:
            m = self.models[key]
            st = self.state[key]
            t0 = time.perf_counter()
            h = m.step(w, dt, cue_rad=cue_rad) if st.accepts_cue else m.step(w, dt)
            ms = (time.perf_counter() - t0) * 1000.0
            st.ms_ema = ms if st.ms_ema == 0 else 0.9 * st.ms_ema + 0.1 * ms
            st.heading_deg = wrap_deg(math.degrees(float(h)))
            # Score only on ticks that brought a fresh IMU sample: between samples "truth" is a
            # 200 ms staircase and comparing a smooth model against it adds a sawtooth error.
            ph = st.phases.setdefault(phase, ErrStats())
            ph.duration_s += dt
            if samples:
                st.err_deg = wrap_deg(st.heading_deg - truth)
                conf = confidence_of(m)
                (st.blind if self.blindfolded else st.cue).add(st.err_deg, conf)
                ph.add(st.err_deg, conf)

        h_drive = self.state[self.driver].heading_deg
        if self.mode in ("needle", "sweep"):
            acts.pan_deg = self._needle(h_drive, dt)  # sweep keeps the needle working
        elif self.mode == "hold":
            acts.pan_deg = self._pan_towards(0.0, dt)
        if self._stop_requested:
            # left a legs mode mid-turn: stand first, start any new leg logic next tick
            self._stop_requested = False
            acts.skill = STAND
        elif self.mode == "hold":
            acts.skill = self._hold(h_drive, now)
        elif self.mode == "sweep":
            acts.skill = self._sweep(now)
        if acts.skill in (TURN_LEFT, TURN_RIGHT):
            self.gait_active = True
        elif acts.skill == STAND:
            self.gait_active = False
        return acts

    def stop_gait(self) -> str | None:
        """For shutdown: the skill to send now ("up") if a turning gait may be running."""
        if not self.gait_active:
            return None
        self.gait_active = False
        self._stop_requested = False
        return STAND

    # ---- sweep ----------------------------------------------------------------------
    def _sweep(self, now: float) -> str | None:
        """Turn left to +A, pause, turn right to -A, pause, ... on the MEASURED heading
        (truth relative to the zero taken at start), not on a model's heading."""
        c = self.cfg
        h = self.truth_rel
        hist = self._sweep_hist
        hist.append((now, h))
        while len(hist) > 2 and hist[1][0] <= now - SWEEP_STALL_WINDOW_S:
            hist.pop(0)
        st = self.sweep_state
        if st == "done":
            return None
        if st == "idle":
            # first leg: towards the target we are further from, normally left
            return self._sweep_turn(1 if h < c.sweep_angle_deg else -1, now)
        if st == "pausing":
            if now - self.sweep_since < c.sweep_pause_s:
                return None
            if c.sweep_cycles is not None and self.sweep_cycles_done >= c.sweep_cycles:
                self.sweep_state = "done"
                self.sweep_since = now
                self.events.append(f"sweep finished after {self.sweep_cycles_done} cycles")
                return None
            return self._sweep_turn(self.sweep_dir, now)
        # turning
        d = self.sweep_dir
        if d * h >= c.sweep_angle_deg:
            self.sweep_timeouts = 0
            return self._sweep_leg_done(now)
        if now - self.sweep_since > c.sweep_timeout_s:
            self.sweep_timeouts += 1
            side = "left" if d > 0 else "right"
            self.events.append(
                f"sweep: turn {side} timed out after {c.sweep_timeout_s:.0f} s at {h:+.0f} deg"
            )
            if self.sweep_timeouts >= SWEEP_MAX_TIMEOUTS:
                self.events.append(
                    f"sweep: {SWEEP_MAX_TIMEOUTS} timeouts in a row, switching to needle mode"
                )
                self.mode = "needle"
                self._init_sweep()
                return STAND
            return self._sweep_leg_done(now)
        # watchdog: has the gait silently stopped?
        t_old, h_old = hist[0]
        if (
            now - self.sweep_since >= SWEEP_STALL_WINDOW_S
            and now - t_old >= SWEEP_STALL_WINDOW_S - 0.1
            and d * (h - h_old) < SWEEP_STALL_MIN_DEG
            and now - self.sweep_last_issue >= SWEEP_REISSUE_S
        ):
            self.sweep_last_issue = now
            self.sweep_reissues += 1
            self.events.append(
                f"sweep: only {d * (h - h_old):+.1f} deg in {SWEEP_STALL_WINDOW_S:.0f} s, "
                "re-issuing the turn"
            )
            return TURN_LEFT if d > 0 else TURN_RIGHT
        return None

    def _sweep_turn(self, direction: int, now: float) -> str:
        self.sweep_state = "turning"
        self.sweep_dir = direction
        self.sweep_since = now
        self.sweep_last_issue = now
        self._sweep_hist = [(now, self.truth_rel)]
        return TURN_LEFT if direction > 0 else TURN_RIGHT

    def _sweep_leg_done(self, now: float) -> str:
        """Stand, pause, and turn the other way next. A cycle is one left and one right leg."""
        self.sweep_legs_done += 1
        if self.sweep_dir < 0:
            self.sweep_cycles_done += 1
        self.sweep_state = "pausing"
        self.sweep_dir = -self.sweep_dir
        self.sweep_since = now
        return STAND

    def sweep_status(self, now: float) -> dict[str, Any]:
        st = self.sweep_state
        label = {
            "turning": "turning left" if self.sweep_dir > 0 else "turning right",
            "pausing": "pausing",
            "done": "done",
            "idle": "idle",
        }[st]
        a = self.cfg.sweep_angle_deg
        return {
            "state": st,
            "label": label,
            "direction": self.sweep_dir,
            "target": (a if self.sweep_dir > 0 else -a) if st in ("turning", "pausing") else None,
            "angle": a,
            "time_in_state": round(now - self.sweep_since, 1) if st != "idle" else 0.0,
            "cycles": self.sweep_cycles_done,
            "max_cycles": self.cfg.sweep_cycles,
            "timeouts": self.sweep_timeouts,
            "reissues": self.sweep_reissues,
            "allowed": self.allow_hold,
        }

    # ---- needle ---------------------------------------------------------------------
    def _needle(self, heading_deg: float, dt: float) -> int | None:
        c = self.cfg
        desired = c.pan_sign * wrap_deg(0.0 - heading_deg)  # target heading is 0 after re-zero
        self.pan_desired = desired
        lim = c.pan_limit_deg
        if abs(desired) > lim:
            self.pan_saturated = True
            # Target roughly behind the dog: the sign of `desired` flips at +-180 deg, so keep
            # the side we are already on instead of swinging the arm across the front.
            side = math.copysign(1.0, desired)
            if abs(desired) > 150 and abs(self.pan_cmd) > lim / 2:
                side = math.copysign(1.0, self.pan_cmd)
            desired = side * lim
        else:
            self.pan_saturated = False
        return self._pan_towards(desired, dt)

    def _pan_towards(self, desired: float, dt: float) -> int | None:
        c = self.cfg
        lim = c.pan_limit_deg
        desired = max(-lim, min(lim, desired))
        a = 1.0 - math.exp(-dt / c.pan_smooth_tau_s) if c.pan_smooth_tau_s > 0 else 1.0
        self.pan_filtered += a * (desired - self.pan_filtered)
        step = c.pan_rate_dps * dt
        self.pan_cmd += max(-step, min(step, self.pan_filtered - self.pan_cmd))
        self.pan_cmd = max(-lim, min(lim, self.pan_cmd))
        out = round(self.pan_cmd)
        if self.pan_sent is None or abs(out - self.pan_sent) >= c.pan_deadband_deg:
            self.pan_sent = out
            return out
        return None

    # ---- hold -----------------------------------------------------------------------
    def _hold(self, heading_deg: float, now: float) -> str | None:
        c = self.cfg
        err = wrap_deg(0.0 - heading_deg)  # + => must turn left (CCW)
        if self.hold_state == "disabled":
            return None
        if self.hold_state == "turning":
            elapsed = now - self.hold_since
            crossed = self.hold_dir * err < 0
            if abs(err) < c.hold_exit_deg or crossed:
                self.hold_state = "cooldown"
                self.hold_since = now
                self.hold_failures = 0
                return "up"
            if elapsed > c.hold_max_turn_s:
                self.hold_failures += 1
                self.hold_state = "cooldown"
                self.hold_since = now
                self.events.append(f"turn timed out after {c.hold_max_turn_s:.0f}s")
                if self.hold_failures >= c.hold_max_failures:
                    self.hold_state = "disabled"
                    self.events.append("hold disabled after repeated timeouts")
                return "up"
            return None
        if self.hold_state == "cooldown" and now - self.hold_since < c.hold_cooldown_s:
            return None
        self.hold_state = "idle"
        if abs(err) > c.hold_enter_deg:
            self.hold_state = "turning"
            self.hold_dir = 1 if err > 0 else -1
            self.hold_since = now
            return "vtL" if err > 0 else "vtR"
        return None

    # ---- reporting ----------------------------------------------------------------
    def frame(self, now: float) -> dict[str, Any]:
        truth = wrap_deg(self.truth_rel)
        models = {}
        for key in MODEL_KEYS:
            m = self.models[key]
            st = self.state[key]
            bump, angles = bump_of(m) if key != "gyro" else ([], [])
            conf = confidence_of(m)
            models[key] = {
                "heading": round(st.heading_deg, 2),
                "err": round(st.err_deg, 2),
                "mae": None if st.mae is None else round(st.mae, 2),
                "mae_cue": None if st.cue.mae is None else round(st.cue.mae, 2),
                "mae_blind": None if st.blind.mae is None else round(st.blind.mae, 2),
                "conf_blind": None if st.blind.mean_conf is None else round(st.blind.mean_conf, 3),
                "accepts_cue": st.accepts_cue,
                "confidence": None if conf is None else round(conf, 3),
                "ms": round(st.ms_ema, 3),
                "n_neurons": n_neurons_of(m) if key != "gyro" else None,
                "bump": bump,
                "bump_angles": angles,
            }
        age = None if self.last_sample_t is None else round(now - self.last_sample_t, 3)
        return {
            "t": round(now - (self.t_start or now), 3),
            "mode": self.mode,
            "driver": self.driver,
            "truth": round(truth, 2),
            "omega": round(self.omega_dps, 2),
            "target": 0.0,
            "pan_sign": self.cfg.pan_sign,
            "zeroed": self.zeroed,
            "samples": self.n_samples,
            "sample_age": age,
            "stale": age is None or age > self.cfg.stale_after_s,
            "imu_interval": round(self.interval_ema, 3),
            "needle": {
                "desired": round(self.pan_desired, 1),
                "cmd": round(self.pan_cmd, 1),
                "saturated": self.pan_saturated,
                "limit": self.cfg.pan_limit_deg,
            },
            "hold": {"state": self.hold_state, "allowed": self.allow_hold},
            "sweep": self.sweep_status(now),
            "gait_active": self.gait_active,
            "cue": self.cue_this_tick,
            "phase": self.phase,
            "blindfold": {
                "on": self.blindfolded,
                "elapsed": None if self.blind_since is None else round(now - self.blind_since, 1),
                "turned": round(self.truth_rel - self.blind_truth0, 1)
                if self.blindfolded
                else None,
            },
            "models": models,
        }

    def phase_summary(self) -> dict[str, dict[str, dict[str, Any]]]:
        """{phase: {model: {mae_deg, final_err_deg, mean_conf, samples}}} in phase order."""
        out: dict[str, dict[str, dict[str, Any]]] = {}
        for key in MODEL_KEYS:
            for ph, stats in self.state[key].phases.items():
                if not stats.n:
                    continue  # a phase that ended before any IMU sample was scored
                d = stats.summary()
                d["duration_s"] = round(stats.duration_s, 1)
                out.setdefault(ph, {})[key] = d
        return out
