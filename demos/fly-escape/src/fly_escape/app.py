"""Glue: proximity samples (robot or replay) -> EscapeLoop -> escape action, dashboard, log."""

from __future__ import annotations

import logging
import math
import queue
import threading
import time
from typing import Any

from fly_compass.server import FrameHub
from flybrain_edge.bittle import Bittle, MockBittle
from flybrain_edge.bittle import protocol as P

from .escape import STAND_TIMEOUT_S, EscapeConfig, EscapeController
from .loop import EscapeLoop
from .scripts import SCRIPTS
from .session import ReplaySource, SessionLog

log = logging.getLogger(__name__)

MODES = ("observe", "escape")
DEFAULT_PINS = (36, 39)  # the Grove socket labelled 39/36
FULL_TRACE_EVERY = 20  # frames; the others carry only new samples


class BittleRobot:
    """EscapeController's view of the robot: queued skills (sent one at a time, each waiting
    for its echo, never on top of another command) and the analog-read pause."""

    def __init__(self, bittle: Bittle):
        self.b = bittle

    def send_skill(self, name: str):
        timeout = STAND_TIMEOUT_S if name == "up" else 2.0
        return self.b.submit(P.encode_skill(name), echo="k", timeout=timeout)

    def send_rest(self):
        self.b.forget_joint_targets()
        return self.b.submit(P.encode_ascii(P.T_REST), echo=P.T_REST, timeout=4.0)

    def pause_sensing(self, paused: bool) -> None:
        self.b.pause_analog(paused)


class _NoRobot:
    """Replay: nothing to move."""

    class _Done:
        def __init__(self):
            self.done = threading.Event()
            self.done.set()
            self.result: list[str] | None = []

    def send_skill(self, name: str):
        return self._Done()

    def send_rest(self):
        return self._Done()

    def pause_sensing(self, paused: bool) -> None:
        pass


class EscapeApp:
    def __init__(
        self,
        loop: EscapeLoop,
        hub: FrameHub | None = None,
        bittle: Bittle | None = None,
        mock: MockBittle | None = None,
        replay: ReplaySource | None = None,
        replay_speed: float = 1.0,
        session: SessionLog | None = None,
        pins: tuple[int, int] = DEFAULT_PINS,
        escape_cfg: EscapeConfig | None = None,
        mode: str = "observe",
        allow_moves: bool = False,
        source_label: str = "",
        exit_after_replay: bool = True,
        rate_hz: float = 20.0,
    ):
        if (bittle is None) == (replay is None):
            raise ValueError("exactly one of bittle / replay is required")
        self.loop = loop
        self.hub = hub or FrameHub()
        self.bittle = bittle
        self.mock = mock
        self.replay = replay
        self.replay_speed = replay_speed
        self.session = session
        self.pins = tuple(pins)
        self.allow_moves = allow_moves and replay is None
        self.source_label = source_label
        self.exit_after_replay = exit_after_replay
        self.rate_hz = rate_hz
        robot = BittleRobot(bittle) if bittle is not None else _NoRobot()
        self.escape = EscapeController(robot, escape_cfg or EscapeConfig())
        self.mode = "observe"
        self.requested_mode = mode
        self.imu = False
        self.analog_rate_hz = 40.0
        self.stop_event = threading.Event()
        self._commands: queue.Queue = queue.Queue()
        self.messages: list[str] = []
        self.on_event = None  # callable(str)
        self.last_frame: dict[str, Any] | None = None
        self.finished = False
        self._frames = 0
        self._last_frame_t: float | None = None
        self._logged_escapes = 0
        self.sim = None  # Simulator in mock mode (fly_escape.sim)

    # ---- events / commands ---------------------------------------------------------
    def _event(self, text: str) -> None:
        self.messages.append(text)
        log.info(text)
        if self.on_event is not None:
            self.on_event(text)

    def on_command(self, cmd: dict[str, Any]) -> dict[str, Any]:
        reply: queue.Queue = queue.Queue(maxsize=1)
        self._commands.put((cmd, reply))
        try:
            return reply.get(timeout=2.0)
        except queue.Empty:
            return {"ok": False, "error": "control loop busy"}

    def set_mode(self, mode: str, now: float) -> str | None:
        if mode not in MODES:
            return f"unknown mode {mode!r}"
        if mode == "escape":
            if not self.allow_moves:
                return (
                    "escape mode moves the legs: restart with --i-have-space to allow it"
                    if self.replay is None
                    else "replay cannot move a robot"
                )
            if not self.loop.cal.done:
                return "still calibrating: keep hands away"
            if not self.loop.cal.ok:
                return "not calibrated: " + self.loop.cal.message
            if self.escape.state == "failed":
                return "a stand was not confirmed earlier; check the robot and restart"
        if mode == "observe" and self.mode == "escape":
            self.escape.abort(now)
        self.mode = mode
        return None

    def _apply_command(self, cmd: dict[str, Any], now: float) -> dict[str, Any]:
        action = cmd.get("action")
        lp = self.loop
        if action == "mode":
            err = self.set_mode(str(cmd.get("mode")), now)
            return {"ok": err is None, **({"error": err} if err else {})}
        if action == "driver":
            err = lp.set_driver(str(cmd.get("model")))
            return {"ok": err is None, **({"error": err} if err else {})}
        if action == "hands_off":
            on = cmd.get("on")
            lp.set_hands_off(not lp.hands_off if on is None else bool(on), now)
            return {"ok": True, "on": lp.hands_off}
        if action == "recalibrate":
            if self.escape.busy:
                return {"ok": False, "error": "wait until the escape has finished"}
            if self.mode == "escape":
                self.mode = "observe"
                self.requested_mode = "escape"
            lp.recalibrate(now)
            return {"ok": True}
        if action == "reset_scores":
            lp.reset_scores()
            return {"ok": True}
        if isinstance(action, str) and action.startswith("sim_"):
            return self._sim_command(action, cmd)
        if action in ("mock_hand", "mock_script"):
            if self.mock is None:
                return {"ok": False, "error": "only available with --mock"}
            if action == "mock_script":
                fn = SCRIPTS.get(str(cmd.get("name")))
                if fn is None:
                    return {"ok": False, "error": f"unknown script {cmd.get('name')!r}"}
                self.mock.hand.run_script(fn, self.mock.clock())
                return {"ok": True}
            try:
                d, side = float(cmd.get("distance", 80.0)), float(cmd.get("side", 0.0))
            except (TypeError, ValueError):
                return {"ok": False, "error": "distance and side must be numbers"}
            if not (math.isfinite(d) and math.isfinite(side)):
                return {"ok": False, "error": "distance and side must be finite"}
            self.mock.hand.set(min(max(d, 0.0), 200.0), side)
            return {"ok": True}
        return {"ok": False, "error": f"unknown action {action!r}"}

    def _sim_command(self, action: str, cmd: dict[str, Any]) -> dict[str, Any]:
        sim = self.sim
        if sim is None:
            return {"ok": False, "error": "only available with --mock"}
        try:
            if action == "sim_launch":
                a = sim.launch(str(cmd.get("kind")), int(cmd.get("side", 0)))
                return {"ok": True, "label": a.label()}
            if action == "sim_random":
                on = cmd.get("on")
                seed = cmd.get("seed")
                sim.set_random(
                    not sim.random_on if on is None else bool(on),
                    None if seed in (None, "") else int(seed),
                )
                return {"ok": True, "on": sim.random_on, "seed": sim.random.seed}
            if action == "sim_speed":
                v = float(cmd.get("value", 1.0))
                if not math.isfinite(v):
                    raise ValueError("speed must be finite")
                sim.speed = min(4.0, max(0.25, v))
                return {"ok": True, "speed": sim.speed}
            if action == "sim_pause":
                on = cmd.get("on")
                sim.set_paused(not sim.paused if on is None else bool(on))
                return {"ok": True, "paused": sim.paused}
            if action == "sim_stop":
                sim.stop()
                return {"ok": True}
            if action == "sim_reset":
                sim.reset()
                self.loop.reset_scores()
                return {"ok": True}
        except (TypeError, ValueError) as e:
            return {"ok": False, "error": str(e)}
        return {"ok": False, "error": f"unknown action {action!r}"}

    # ---- robot ---------------------------------------------------------------------
    def start_robot(self) -> None:
        b = self.bittle
        if b is None:
            return
        b.stand(balance=False)
        if b.low_power:
            self._event(
                f"THE ROBOT WILL NOT MOVE: it reports '{b.low_power}'. Long-press the battery "
                "button to switch the battery on (or charge it)."
            )
        if self.imu:
            b.poll_imu(10.0)
        b.poll_analog(self.pins, self.analog_rate_hz)
        self._event("calibrating: keep hands away from the sensor for 2 s")

    def status(self, now: float) -> dict[str, Any]:
        st: dict[str, Any] = {"source": self.source_label, "mock": self.mock is not None}
        if self.bittle is not None:
            b = self.bittle
            st.update(
                connected=b.connected,
                link=b.transport.description,
                firmware=" ".join(x for x in (b.model, b.version) if x) or None,
                analog_samples=b.stats.analog_samples,
                echo_timeouts=b.stats.echo_timeouts,
                low_power=b.low_power,
            )
        else:
            assert self.replay is not None
            st.update(
                connected=True,
                link="replay",
                firmware=None,
                replay_progress=round(self.replay.i / max(1, len(self.replay.rows)), 3),
                replay_done=self.replay.done,
            )
        st["messages"] = self.messages[-6:]
        st["scripts"] = list(SCRIPTS) if self.mock is not None else []
        return st

    # ---- main loop -----------------------------------------------------------------
    def _samples(self, now: float, sim_t: float) -> list[tuple[float, int, int]]:
        if self.replay is not None:
            return self.replay.poll(sim_t)
        out = []
        for s in self.bittle.drain_analog():  # type: ignore[union-attr]
            a, b = s.values.get(self.pins[0]), s.values.get(self.pins[1])
            if a is not None and b is not None:
                out.append((s.t, a, b))
        return out

    def run(self, duration: float | None = None) -> dict[str, Any]:
        lp = self.loop
        dt = 1.0 / self.rate_hz
        sim_t = 0.0
        t_wall0 = time.monotonic()
        next_tick = t_wall0
        fast = self.replay is not None and self.replay_speed <= 0
        while not self.stop_event.is_set():
            if self.replay is not None:
                sim_t += dt
                now = sim_t
            else:
                now = time.monotonic()
            while True:
                try:
                    cmd, reply = self._commands.get_nowait()
                except queue.Empty:
                    break
                try:
                    reply.put_nowait(self._apply_command(cmd, now))
                except Exception as e:  # noqa: BLE001 - report, never kill the loop
                    reply.put_nowait({"ok": False, "error": repr(e)})
            if self.bittle is not None and not self.bittle.connected:
                raise RuntimeError("robot link lost")

            was_done = lp.cal.done
            for t, a, b in self._samples(now, sim_t):
                if self.session is not None:
                    self.session.sample(t, a, b)
                for tr in lp.ingest(t, a, b):
                    if self.session is not None:
                        self.session.trigger(tr.t, tr.model, tr.side, lp.hands_off)
                    if self.sim is not None:
                        self.sim.on_trigger(tr.model, tr.t, tr.side)
            if lp.cal.done and not was_done:
                self._event(lp.cal.message)
                if lp.cal.ok and self.requested_mode == "escape":
                    err = self.set_mode("escape", now)
                    if err:
                        self._event(f"escape mode not started: {err}")
                elif not lp.cal.ok and self.requested_mode == "escape":
                    self._event("ESCAPE MODE REFUSED: " + lp.cal.message)
            for m in lp.messages:
                if m != lp.cal.message:
                    self._event(m)
            lp.messages.clear()

            tr = lp.take_driver_trigger()
            started = tr is not None and self.mode == "escape" and self.escape.trigger(now, tr.side)
            if started and self.sim is not None:
                self.sim.on_escape(tr.t)
            if self.sim is not None and lp.cal.done:
                self.sim.update(self.sim.clock(), robot_idle=not self.escape.busy)
            if self.escape.update(now) == "observe":
                self.mode = "observe"
            for ev in self.escape.events:
                self._event(ev)
            self.escape.events.clear()
            if self.session is not None:
                while self._logged_escapes < len(self.escape.history):
                    r = self.escape.history[self._logged_escapes]
                    if r.stood is None and self.escape.busy:
                        break
                    self.session.escape(r.t, r.side, r.skill, r.stood)
                    self._logged_escapes += 1

            self._publish(now)

            if self.replay is not None and self.replay.done and not self.finished:
                self.finished = True
                self._event("replay finished")
                if fast or self.exit_after_replay:
                    break
            elapsed = sim_t if self.replay is not None else time.monotonic() - t_wall0
            if duration is not None and elapsed >= duration:
                break
            if fast:
                continue
            speed = self.replay_speed if self.replay is not None else 1.0
            next_tick += dt / speed
            delay = next_tick - time.monotonic()
            if delay > 0:
                self.stop_event.wait(delay)
            else:
                next_tick = time.monotonic()
        return self.summary()

    def _publish(self, now: float) -> None:
        full = self._frames % FULL_TRACE_EVERY == 0
        frame = self.loop.frame(now, since=None if full else self._last_frame_t)
        frame["trace_full"] = full
        self._frames += 1
        self._last_frame_t = now
        frame["mode"] = self.mode
        frame["allow_moves"] = self.allow_moves
        frame["escape"] = self.escape.status(now)
        if self.escape.history:
            o = self.loop.t_origin or now
            frame["escape"]["last"]["t"] = round(self.escape.history[-1].t - o, 2)
        frame["status"] = self.status(now)
        if self.mock is not None:
            d, side = self.mock.hand.state(self.mock.clock())
            frame["hand"] = {"distance": round(d, 1), "side": round(side, 2)}
        if self.sim is not None:
            frame["sim"] = self.sim.frame(self.sim.clock())
        self.last_frame = frame
        self.hub.publish(frame)

    def summary(self) -> dict[str, Any]:
        lat = self.loop.latency_stats()
        return {
            "ground_truth": None if self.sim is None else self.sim.gt.stats(),
            "samples": self.loop.n_samples,
            "calibration": self.loop.cal.status(),
            "escapes": len(self.escape.history),
            "blocked": self.escape.blocked,
            "models": {
                k: {
                    "triggers": sc.triggers,
                    "false_alarms": sc.false_alarms,
                    "events": lat[k]["events"],
                    "missed": lat[k]["missed"],
                    "mean_dt_vs_driver_ms": lat[k]["mean_ms"],
                }
                for k, sc in self.loop.scores.items()
            },
            "events": len(self.loop.events),
        }

    def shutdown(self) -> None:
        """Stop any escape gait, stand, and leave the robot resting. Safe to call twice."""
        self.stop_event.set()
        if self.bittle is not None:
            try:
                if self.escape.state == "acting":
                    self.bittle.pause_analog(True)
                    self.bittle.skill("up", wait=True, timeout=STAND_TIMEOUT_S)
                self.bittle.flush_commands()
            except Exception as e:  # noqa: BLE001 - still rest the robot below
                log.error("stand before rest failed: %s", e)
            self.bittle.close()  # rests the robot
        if self.session is not None:
            self.session.close()
