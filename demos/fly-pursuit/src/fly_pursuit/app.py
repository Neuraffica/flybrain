"""Glue: the rig (the simulation or the robot's head), the dashboard's commands and the
frames it draws.

The rig is ``sim.PursuitSim`` or ``robot.HeadRig``; both have the same settings, the same
``advance(seconds)`` and frames with the same fields (``FRAME_DEFAULTS`` fills in the fields
that only the robot has). It runs in the thread that calls ``run`` (the main thread in the
CLI); commands arrive on the dashboard server's threads and change it under a lock. The
steering curve is computed on the command's thread, on a separate model instance, so the rig
keeps running meanwhile.

Commands (POST /api/command, JSON):

    {"action": "target", "deg": 35}          put the target at this world bearing (holds still)
    {"action": "nudge", "deg": -5}           move the target by this much (holds still)
    {"action": "motion", "motion": "sweep"}  still | sweep | walk | jumps | hidden
    {"action": "controller", "controller": "no-aotu019"}   see controllers.CONTROLLER_KEYS
    {"action": "arousal", "value": 0.5}      0..1 (circuit controllers only)
    {"action": "fov", "deg": 60}             half field of view: 30 | 60 | 100
    {"action": "reset"}                      restart the running numbers

Each reply is {"ok": true} or {"ok": false, "error": "..."}. On the robot, target, nudge,
motion and fov are refused: the target is the face the camera sees.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, Any

from flybrain_demo_api.web import FrameHub

from .controllers import (
    CONTROLLERS,
    CURVE_DEG,
    CircuitLayout,
    activity_scales,
    is_circuit,
    steering_curve,
)
from .sim import PursuitSim

if TYPE_CHECKING:
    from .robot import HeadRig

FRAME_HZ = 20.0  # frames published to the dashboard per second
MAX_CATCHUP_S = 0.25  # after a stall, simulate at most this much at once

# Fields only the robot's frames carry; the simulator's frames get these values.
FRAME_DEFAULTS: dict[str, Any] = {
    "mode": "sim",
    "head_deg": None,
    "camera_age_s": None,
    "camera_lines": None,
    "hold": False,
    "pan_limit_deg": None,
    "link": None,
    "raw_bearing_deg": None,
    "camera_score": None,
    "camera_source": None,
    "camera_error": None,
    "attention": None,
}


class PursuitApp:
    def __init__(
        self,
        sim: PursuitSim | HeadRig,
        hub: FrameHub | None = None,
        frame_hz: float = FRAME_HZ,
    ):
        self.sim = sim  # the rig: simulated, or the robot's head
        self.hub = hub or FrameHub()
        self.frame_hz = frame_hz
        self.lock = threading.Lock()
        self.scales = activity_scales()
        self._layouts: dict[str, CircuitLayout] = {}
        self._curve_models: dict[str, object] = {}
        self._curve_lock = threading.Lock()
        self._curve_for: tuple[str, float] | None = None
        self.curve: dict[str, Any] | None = None
        self.refresh_curve()

    # ---- commands ------------------------------------------------------------------------
    def on_command(self, cmd: dict[str, Any]) -> dict[str, Any]:
        action = cmd.get("action")
        with self.lock:
            s = self.sim
            if action == "target":
                err = s.set_target(cmd.get("deg"))
            elif action == "nudge":
                err = s.nudge_target(cmd.get("deg"))
            elif action == "motion":
                m = cmd.get("motion")
                err = s.set_motion(m) if isinstance(m, str) else "motion must be a name"
            elif action == "controller":
                c = cmd.get("controller")
                err = s.set_controller(c) if isinstance(c, str) else "controller must be a name"
            elif action == "arousal":
                err = s.set_arousal(cmd.get("value"))
            elif action == "fov":
                err = s.set_fov(cmd.get("deg"))
            elif action == "reset":
                s.reset_stats()
                err = None
            else:
                err = f"unknown action {action!r}"
        if err:
            return {"ok": False, "error": err}
        if action in ("controller", "arousal"):
            self.refresh_curve()
        return {"ok": True}

    def refresh_curve(self) -> None:
        """Recompute the steering curve if the controller or arousal changed since the last
        one (loops until it matches, in case they change again meanwhile)."""
        with self._curve_lock:
            while True:
                with self.lock:
                    key = (self.sim.controller, self.sim.arousal)
                if key == self._curve_for:
                    return
                name, arousal = key
                if name not in self._curve_models:
                    self._curve_models[name] = CONTROLLERS[name].make()
                turn = steering_curve(self._curve_models[name], arousal)
                self.curve = {
                    "controller": name,
                    "arousal": arousal,
                    "deg": list(CURVE_DEG),
                    "turn_deg_s": turn,
                }
                self._curve_for = key

    # ---- frames --------------------------------------------------------------------------
    def _activity(self) -> dict[str, Any] | None:
        s = self.sim
        if not is_circuit(s.model):
            return None
        if s.controller not in self._layouts:
            self._layouts[s.controller] = CircuitLayout(s.model)
        lay = self._layouts[s.controller]
        a = dict(s.model.activity)
        if s.model.mirror_average:
            # one-per-side types: the average of the brain and its mirror image, so that the
            # DNa02 difference drawn is the one that sets the turn. LC10a stay the real eyes.
            b = s.model.activity_mirror
            for t in ("AOTU019", "AOTU025", "DNa02"):
                a[t] = 0.5 * (a[t] + b[t][::-1])
        sc = self.scales

        def norm(values, scale, digits=3):
            return [round(float(v) / scale, digits) for v in values]

        rest = sc["DNa02_rest"]
        return {
            # every value as a fraction of the connectome's largest steady response
            "lc10a_l": norm(a["LC10a_L"][lay.order["LC10a_L"]], sc["LC10a"], 2),
            "lc10a_r": norm(a["LC10a_R"][lay.order["LC10a_R"]], sc["LC10a"], 2),
            "lc10a_l_deg": lay.azimuth_deg["LC10a_L"],
            "lc10a_r_deg": lay.azimuth_deg["LC10a_R"],
            "aotu019": norm(a["AOTU019"], sc["AOTU019"]),
            "aotu025": norm(a["AOTU025"], sc["AOTU025"]),
            "dna02": norm(a["DNa02"] - rest, sc["DNa02"]),  # change from the resting rate
            "ablated": list(CONTROLLERS[s.controller].ablated),
        }

    def frame(self) -> dict[str, Any]:
        """The frame the page draws (call with ``lock`` held, or from the sim thread)."""
        f = self.sim.snapshot()
        for k, v in FRAME_DEFAULTS.items():
            f.setdefault(k, v)
        info = CONTROLLERS[self.sim.controller]
        f["controller_label"] = info.label
        f["controller_note"] = info.note
        f["activity"] = self._activity()
        f["curve"] = self.curve
        gain = f["rig"].get("gain", 1.0)
        if self.curve is not None and gain != 1.0:  # the robot's --gain scales every turn
            f["curve"] = dict(self.curve, turn_deg_s=[gain * v for v in self.curve["turn_deg_s"]])
        return f

    # ---- running -------------------------------------------------------------------------
    def tick(self, seconds: float) -> dict[str, Any]:
        with self.lock:
            self.sim.advance(min(max(0.0, seconds), MAX_CATCHUP_S))
            f = self.frame()
        self.hub.publish(f)
        return f

    def run(self, stop: threading.Event, duration: float | None = None) -> None:
        """Step the rig in real time and publish frames until ``stop`` is set (or
        ``duration`` seconds have passed). Errors from the rig (a lost robot link) end it."""
        period = 1.0 / self.frame_hz
        start = last = time.monotonic()
        self.tick(0.0)
        while not stop.is_set():
            now = time.monotonic()
            if duration is not None and now - start >= duration:
                break
            self.tick(now - last)
            last = now
            stop.wait(max(0.0, period - (time.monotonic() - now)))
