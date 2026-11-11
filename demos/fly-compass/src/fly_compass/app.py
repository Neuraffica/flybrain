"""Glue: sample source (robot or replay) -> CompassLoop -> actuation, dashboard, session log."""

from __future__ import annotations

import logging
import math
import queue
import threading
import time
from typing import Any

from flybrain_edge.bittle import Bittle, MockBittle
from flybrain_edge.bittle import protocol as P

from .loop import CompassLoop, wrap_deg
from .models import MODEL_KEYS
from .scenarios import SCENARIOS, ScenarioRunner
from .server import FrameHub
from .session import ReplaySource, SessionLog

log = logging.getLogger(__name__)


class CompassApp:
    def __init__(
        self,
        loop: CompassLoop,
        hub: FrameHub | None = None,
        bittle: Bittle | None = None,
        mock: MockBittle | None = None,
        replay: ReplaySource | None = None,
        replay_speed: float = 1.0,
        session: SessionLog | None = None,
        source_label: str = "",
        exit_after_replay: bool = True,
        blindfold_after: float | None = None,
        scenario: str | None = None,
        exit_after_scenario: bool = False,
    ):
        if (bittle is None) == (replay is None):
            raise ValueError("exactly one of bittle / replay is required")
        self.loop = loop
        # "stream" = firmware prints continuously ('gP'); "poll" = one 'gp' per reading.
        self.imu_mode = "stream"
        self.imu_rate_hz = 20.0
        self.drift_cal_s = 0.0  # seconds of stillness used to measure IMU yaw drift at start
        self.hub = hub or FrameHub()
        self.bittle = bittle
        self.mock = mock
        self.replay = replay
        self.replay_speed = replay_speed
        self.session = session
        self.source_label = source_label
        self.exit_after_replay = exit_after_replay
        self.stop_event = threading.Event()
        self._commands: queue.Queue[tuple[dict[str, Any], queue.Queue]] = queue.Queue()
        self.last_frame: dict[str, Any] | None = None
        self.messages: list[str] = []
        self.finished = False
        self.blindfold_after = blindfold_after
        self._blindfold_after_done = False
        self.pending_scenario = scenario
        self.scenario: ScenarioRunner | None = None
        self.exit_after_scenario = exit_after_scenario
        self.exit_after_sweep = True  # --sweep-cycles N: end the run when the sweep is done
        self.scenario_results: dict[str, Any] | None = None
        self.on_scenario_done = None  # callable(phase_summary) or None
        self.on_event = None  # callable(str): loop events (re-zero, sweep turns, timeouts)

    # ---- dashboard commands (called from HTTP threads) --------------------------------
    def on_command(self, cmd: dict[str, Any]) -> dict[str, Any]:
        reply: queue.Queue = queue.Queue(maxsize=1)
        self._commands.put((cmd, reply))
        try:
            return reply.get(timeout=2.0)
        except queue.Empty:
            return {"ok": False, "error": "control loop busy"}

    def _apply_command(self, cmd: dict[str, Any]) -> dict[str, Any]:
        action = cmd.get("action")
        lp = self.loop
        if action == "rezero":
            lp.rezero()
            return {"ok": True}
        if action == "reset_scores":
            lp.reset_scores()
            return {"ok": True}
        if action == "mode":
            err = lp.set_mode(str(cmd.get("mode")))
            if err:
                return {"ok": False, "error": err}
            # leaving hold/sweep mid-turn: the loop issues "up" on its next tick
            if lp.mode == "observe" and self.bittle:
                self.bittle.set_joint(P.JOINT_HEAD_PAN, 0)
                lp.pan_cmd = lp.pan_filtered = 0.0
                lp.pan_sent = 0
            return {"ok": True}
        if action == "blindfold":
            if self.replay is not None:
                return {"ok": False, "error": "replay follows the recorded blindfold state"}
            on = cmd.get("on")
            lp.set_blindfold(not lp.blindfolded if on is None else bool(on))
            return {"ok": True, "on": lp.blindfolded}
        if action == "scenario":
            name = str(cmd.get("name"))
            if self.mock is None:
                return {"ok": False, "error": "scenarios need --mock"}
            if name == "stop":
                self._stop_scenario()
                return {"ok": True}
            if name not in SCENARIOS:
                return {"ok": False, "error": f"unknown scenario {name!r}"}
            self.pending_scenario = name
            return {"ok": True}
        if action == "driver":
            err = lp.set_driver(str(cmd.get("model")))
            return {"ok": err is None, **({"error": err} if err else {})}
        if action in ("mock_turn_to", "mock_nudge"):
            if self.mock is None:
                return {"ok": False, "error": "only available with --mock"}
            try:
                deg = float(cmd.get("deg", 0.0))
            except (TypeError, ValueError):
                return {"ok": False, "error": "deg must be a number"}
            if not math.isfinite(deg):
                return {"ok": False, "error": "deg must be finite"}
            sign = self.bittle.yaw_sign if self.bittle else 1
            delta = (
                wrap_deg(deg - lp.truth_rel)
                if action == "mock_turn_to"
                else max(-360, min(360, deg))
            )
            self.mock.nudge(sign * delta)
            return {"ok": True}
        return {"ok": False, "error": f"unknown action {action!r}"}

    def _skill(self, name: str) -> None:
        if self.bittle is not None:
            # gaits echo at once; do not block the loop waiting for postures
            self.bittle.skill(name, wait=False)

    # ---- main loop ---------------------------------------------------------------------
    def start_robot(self) -> None:
        b = self.bittle
        if b is None:
            return
        b.stand(balance=False)
        if self.imu_mode == "poll":
            b.poll_imu(self.imu_rate_hz)
        else:
            b.stream_imu(True)
        if b.low_power:
            print(
                f"fly-compass: THE ROBOT WILL NOT MOVE: it reports '{b.low_power}'. "
                "Long-press the battery button to switch the battery on (or charge it).",
                flush=True,
            )
        if self.drift_cal_s > 0:
            print(
                f"fly-compass: measuring heading drift for {self.drift_cal_s:.0f} s. "
                "Do not touch the dog ...",
                flush=True,
            )
            rate = b.measure_yaw_drift(self.drift_cal_s)
            used = "compensating" if b.yaw_drift_dps else "NOT compensating"
            print(f"fly-compass: heading drift {rate:+.2f} deg/s, {used}", flush=True)

    def status(self, now: float) -> dict[str, Any]:
        st: dict[str, Any] = {"source": self.source_label, "mock": self.mock is not None}
        if self.bittle is not None:
            b = self.bittle
            st.update(
                connected=b.connected,
                link=b.transport.description,
                firmware=" ".join(x for x in (b.model, b.version) if x) or None,
                imu_rate=round(b.stats.imu_rate_hz, 2),
                echo_timeouts=b.stats.echo_timeouts,
                joint_token=b.joint_token,
            )
        else:
            assert self.replay is not None
            st.update(
                connected=True,
                link="replay",
                firmware=None,
                replay_progress=round(
                    min(1.0, (self.replay.i / max(1, len(self.replay.samples)))), 3
                ),
                replay_done=self.replay.done,
            )
        st["messages"] = self.messages[-5:]
        st["scenario"] = self.scenario.status(now) if self.scenario else None
        st["scenarios"] = list(SCENARIOS) if self.mock is not None else []
        return st

    # ---- scenarios / blindfold schedule -----------------------------------------------
    def _start_scenario(self, name: str, now: float) -> None:
        assert self.mock is not None
        lp = self.loop
        lp.rezero()  # scores per phase start clean; models re-anchored at the current heading
        self.scenario = ScenarioRunner(name, now, self.mock.world_yaw)
        runner = self.scenario
        mock_t0 = self.mock.clock_origin()
        # the mock calls trajectory(t) with its own time since power-on
        self.mock.trajectory = lambda tm: runner.at(tm + mock_t0)[2]
        self.messages.append(f"scenario {name} started ({runner.total_s:.0f} s)")

    def _stop_scenario(self) -> None:
        if self.scenario is None:
            return
        if self.mock is not None:
            self.mock.trajectory = None
        self.scenario = None
        self.loop.phase_override = None
        self.loop.set_blindfold(False)

    def _advance_schedule(self, now: float, t_run: float) -> None:
        lp = self.loop
        if self.pending_scenario and self.mock is not None and lp.zeroed:
            name, self.pending_scenario = self.pending_scenario, None
            self._stop_scenario()
            self._start_scenario(name, now)
        if self.scenario is not None:
            i, _, _ = self.scenario.at(now)
            if i != self.scenario.index:
                self.scenario.index = i
                if i < len(self.scenario.phases):
                    ph = self.scenario.phases[i]
                    lp.phase_override = f"{self.scenario.name}: {ph.label}"
                    lp.set_blindfold(ph.blindfold, now)
                else:
                    self.scenario_results = lp.phase_summary()
                    self.messages.append(f"scenario {self.scenario.name} finished")
                    lp.phase_override = None
                    lp.set_blindfold(False, now)
                    self.scenario.done = True
                    if self.mock is not None:
                        self.mock.trajectory = None  # hand the dog back to the dial
                    if self.on_scenario_done is not None:
                        self.on_scenario_done(self.scenario_results)
        if (
            self.blindfold_after is not None
            and not self._blindfold_after_done
            and t_run >= self.blindfold_after
        ):
            self._blindfold_after_done = True
            lp.set_blindfold(True, now)

    def run(self, duration: float | None = None) -> dict[str, Any]:
        lp = self.loop
        dt = 1.0 / lp.cfg.rate_hz
        sim_t = 0.0
        t_wall0 = time.monotonic()
        next_tick = t_wall0
        fast = self.replay is not None and self.replay_speed <= 0
        while not self.stop_event.is_set():
            # commands from the dashboard
            while True:
                try:
                    cmd, reply = self._commands.get_nowait()
                except queue.Empty:
                    break
                try:
                    reply.put_nowait(self._apply_command(cmd))
                except Exception as e:  # noqa: BLE001 - report, never kill the loop
                    reply.put_nowait({"ok": False, "error": repr(e)})

            if self.replay is not None:
                sim_t += dt
                now = sim_t
                samples = self.replay.poll(sim_t)
                if samples:  # reproduce the recorded cue state
                    last = samples[-1]
                    lp.set_blindfold(last.blindfold, now)
                    lp.phase_override = last.phase or None
            else:
                now = time.monotonic()
                samples = self.bittle.drain_samples()  # type: ignore[union-attr]
                if not self.bittle.connected:  # type: ignore[union-attr]
                    raise RuntimeError("robot link lost")
                self._advance_schedule(now, now - t_wall0)

            acts = lp.update(now, samples)
            if self.session is not None:
                for smp in samples:
                    self.session.sample(smp, lp.blindfolded, lp.phase_override)
            for ev in lp.events:
                self.messages.append(ev)
                log.info(ev)
                if self.on_event is not None:
                    self.on_event(ev)
            lp.events.clear()
            if self.bittle is not None:
                if acts.pan_deg is not None:
                    self.bittle.set_joint(P.JOINT_HEAD_PAN, acts.pan_deg)
                if acts.skill:
                    self._skill(acts.skill)

            frame = lp.frame(now)
            frame["status"] = self.status(now)
            self.last_frame = frame
            self.hub.publish(frame)
            if self.session is not None:
                self.session.frame(now, frame)

            if lp.mode == "sweep" and lp.sweep_state == "done" and self.exit_after_sweep:
                break
            if self.scenario is not None and self.scenario.done and self.exit_after_scenario:
                break
            if self.replay is not None and self.replay.done and not self.finished:
                self.finished = True
                self.messages.append("replay finished")
                if fast or self.exit_after_replay:
                    break
            if (
                duration is not None
                and (sim_t if self.replay is not None else time.monotonic() - t_wall0) >= duration
            ):
                break
            if fast:
                continue
            speed = self.replay_speed if self.replay is not None else 1.0
            next_tick += dt / speed
            delay = next_tick - time.monotonic()
            if delay > 0:
                self.stop_event.wait(delay)
            else:
                next_tick = time.monotonic()  # fell behind: do not try to catch up
        return self.summary()

    def summary(self) -> dict[str, Any]:
        out = {}
        for k in MODEL_KEYS:
            st = self.loop.state[k]
            out[k] = {
                "mae_deg": None if st.mae is None else round(st.mae, 2),
                "mae_cue_deg": None if st.cue.mae is None else round(st.cue.mae, 2),
                "mae_blind_deg": None if st.blind.mae is None else round(st.blind.mae, 2),
                "samples": st.n,
                "ms_per_step": round(st.ms_ema, 3),
                "accepts_cue": st.accepts_cue,
            }
        return out

    def phase_summary(self) -> dict[str, Any]:
        return self.scenario_results or self.loop.phase_summary()

    def shutdown(self) -> None:
        """Stop motion and leave the robot resting. Safe to call twice."""
        self.stop_event.set()
        if self.bittle is not None:
            try:
                stop = self.loop.stop_gait()
                if stop:
                    self.bittle.skill(stop, wait=True, timeout=2.0)
            except Exception as e:  # noqa: BLE001 - still rest the robot below
                log.error("stop gait failed: %s", e)
            self.bittle.close()  # rests the robot
        if self.session is not None:
            self.session.close()
