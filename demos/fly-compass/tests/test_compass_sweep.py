"""Sweep mode: the dog turns itself between +A and -A while the needle keeps working."""

import itertools
import threading
import time
from dataclasses import dataclass

import pytest
from compass_fakes import fake_models
from fly_compass import cli
from fly_compass.app import CompassApp
from fly_compass.loop import (
    STOP_GRACE_S,
    STOP_MAX_RESENDS,
    STOP_RESEND_S,
    SWEEP_MAX_TIMEOUTS,
    SWEEP_REISSUE_S,
    SWEEP_STALL_WINDOW_S,
    CompassLoop,
    LoopConfig,
    wrap_deg,
)
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport


@dataclass
class S:
    t: float
    yaw_deg: float


class SimDog:
    """Plant for simulated-time tests: turns while a turn gait runs, optionally stalls."""

    def __init__(self, rate=8.0, stall_after=None, stuck=False, lose_stands=0):
        self.lose_stands = lose_stands  # this many "up" commands are lost (the BLE bug)
        self.rested = False
        self.yaw = 0.0
        self.gait = 0  # +1 vtL, -1 vtR, 0 standing
        self.rate = rate
        self.stall_after = stall_after
        self.stuck = stuck  # never turns at all
        self.started = 0.0
        self.skills: list[tuple[float, str]] = []

    def apply(self, t, skill):
        if skill is None:
            return
        self.skills.append((round(t, 2), skill))
        if skill == "up" and self.lose_stands:
            self.lose_stands -= 1
            return
        self.gait = {"vtL": 1, "vtR": -1}.get(skill, 0)
        self.started = t

    def advance(self, t, dt):
        stalled = self.stall_after is not None and t - self.started > self.stall_after
        if self.gait and not stalled and not self.stuck:
            self.yaw += self.gait * self.rate * dt


def run(loop, dog, seconds, rate=20.0, until=None):
    t, dt = 0.0, 1.0 / rate
    pans = []
    while t <= seconds:
        acts = loop.update(t, [S(t, wrap_deg(dog.yaw))])
        dog.apply(t, acts.skill)
        if acts.rest:
            dog.rested, dog.gait = True, 0
            dog.skills.append((round(t, 2), "REST"))
        if acts.pan_deg is not None:
            pans.append(acts.pan_deg)
        dog.advance(t, dt)
        t = round(t + dt, 6)
        if until and until(loop):
            break
    return pans


def sweep_loop(**cfg):
    c = LoopConfig(**cfg)
    return CompassLoop(fake_models(), c, mode="sweep", allow_hold=True)


def test_reaches_both_targets_pauses_and_cycles():
    loop = sweep_loop(sweep_angle_deg=20, sweep_pause_s=2, sweep_cycles=2)
    dog = SimDog(rate=10)
    pans = run(loop, dog, 60, until=lambda lp: lp.sweep_state == "done")
    names = [s for _, s in dog.skills]
    assert names == ["vtL", "up", "vtR", "up", "vtL", "up", "vtR", "up"]
    assert loop.sweep_state == "done" and loop.sweep_cycles_done == 2
    assert loop.mode == "sweep" and not loop.gait_active
    # the turns stop at the measured targets (one tick of overshoot at most)
    assert -20.6 <= dog.yaw <= -19.9
    # pauses last sweep_pause_s: each "up" is followed by the next turn 2 s later
    t_up = [t for t, s in dog.skills if s == "up"]
    t_turn = [t for t, s in dog.skills if s in ("vtL", "vtR")][1:]
    assert all(abs((b - a) - 2.0) < 0.11 for a, b in zip(t_up, t_turn, strict=False))
    # the needle worked throughout: the head counter-rotated against the dog's heading
    assert max(pans) > 10 and min(pans) < -10


def test_frame_reports_sweep_state():
    loop = sweep_loop(sweep_angle_deg=30)
    dog = SimDog(rate=10)
    run(loop, dog, 1.0)
    sw = loop.frame(1.0)["sweep"]
    assert sw["state"] == "turning" and sw["label"] == "turning left" and sw["target"] == 30
    assert sw["time_in_state"] > 0.5 and sw["cycles"] == 0


def test_timeout_turns_back_and_three_timeouts_fall_back_to_needle():
    loop = sweep_loop(sweep_angle_deg=30, sweep_timeout_s=4, sweep_pause_s=1)
    dog = SimDog(stuck=True)  # the gait never turns the dog
    run(loop, dog, 40, until=lambda lp: lp.mode == "needle")
    names = [s for _, s in dog.skills if s in ("vtL", "vtR")]
    # first legs alternate direction after each timeout (re-issues inside a leg repeat a name)
    legs = [n for i, n in enumerate(names) if i == 0 or n != names[i - 1]]
    assert legs[:3] == ["vtL", "vtR", "vtL"]
    assert loop.mode == "needle" and not loop.gait_active
    assert dog.skills[-1][1] == "up"
    assert sum("timed out" in e for e in loop.events) == SWEEP_MAX_TIMEOUTS
    assert any("switching to needle" in e for e in loop.events)


def test_timeout_counter_resets_after_a_good_leg():
    loop = sweep_loop(sweep_angle_deg=10, sweep_timeout_s=3, sweep_pause_s=0.5)
    dog = SimDog(rate=10)
    loop.sweep_timeouts = 2
    run(loop, dog, 5)
    assert loop.sweep_timeouts == 0 and loop.mode == "sweep"


def test_stalled_gait_is_reissued_but_not_too_often():
    loop = sweep_loop(sweep_angle_deg=30, sweep_timeout_s=60)
    dog = SimDog(rate=8, stall_after=1.0)  # moves 8 deg, then silently stops until re-issued
    run(loop, dog, 30, until=lambda lp: lp.sweep_state == "pausing")
    turns = [t for t, s in dog.skills if s == "vtL"]
    assert len(turns) >= 3, dog.skills  # first issue + re-issues
    gaps = [b - a for a, b in itertools.pairwise(turns)]
    assert min(gaps) >= SWEEP_REISSUE_S - 1e-6
    assert gaps[0] >= SWEEP_STALL_WINDOW_S - 1e-6
    assert loop.sweep_state == "pausing" and dog.yaw >= 30  # got there in the end
    assert loop.sweep_reissues == len(turns) - 1


def test_steady_gait_is_not_reissued():
    loop = sweep_loop(sweep_angle_deg=30, sweep_timeout_s=60)
    dog = SimDog(rate=2.0)  # slow, like carpet, but moving
    run(loop, dog, 14, until=lambda lp: lp.sweep_state == "pausing")
    assert [s for _, s in dog.skills].count("vtL") == 1
    assert loop.sweep_reissues == 0


def test_sweep_needs_permission():
    loop = CompassLoop(fake_models(), mode="observe")
    assert loop.set_mode("sweep") is not None and loop.mode == "observe"


@pytest.mark.parametrize("new_mode", ["needle", "observe", "hold"])
def test_switching_mode_mid_turn_stops_the_gait(new_mode):
    loop = sweep_loop(sweep_angle_deg=30)
    dog = SimDog(rate=5)
    run(loop, dog, 1.0)
    assert loop.gait_active and dog.gait == 1
    assert loop.set_mode(new_mode) is None
    acts = loop.update(1.05, [S(1.05, dog.yaw)])
    assert acts.skill == "up" and not loop.gait_active
    assert loop.sweep_state == "idle"


def test_cli_gates_sweep(capsys):
    assert cli.main(["--mock", "--mode", "sweep"]) == 2
    assert "--i-have-space" in capsys.readouterr().err


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_live_mock_sweep_with_stall_and_shutdown(tmp_path, link):
    mock = MockBittle(
        imu_noise_deg=0.1,
        turn_rate_deg_s=20.0,
        gait_stall_after_s=1.0,
        ble_notifications=link == "ble",
    )
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=1.0, quiet_s=0.3)
    cfg = LoopConfig(rate_hz=20, sweep_angle_deg=12, sweep_pause_s=0.5, sweep_cycles=1)
    loop = CompassLoop(fake_models(), cfg, mode="sweep", allow_hold=True)
    app = CompassApp(loop, bittle=bot, mock=mock)
    try:
        app.start_robot()
        app.run(duration=40)  # exits when the single cycle is done
    finally:
        app.shutdown()
    assert loop.sweep_state == "done" and loop.sweep_cycles_done == 1
    ks = [c for c in mock.commands if c.startswith("k")]
    assert "kvtL" in ks and "kvtR" in ks
    assert mock.turn_commands > 2  # the stalled gait was re-issued
    assert any(c.startswith("i0 ") for c in mock.commands)  # needle ran during the sweep
    assert mock.commands[-1] == "d" and mock.dropped_bytes == 0


def test_live_mode_switch_mid_turn_sends_up(tmp_path):
    mock = MockBittle(imu_noise_deg=0.1, turn_rate_deg_s=5.0)
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=1.0, quiet_s=0.3)
    loop = CompassLoop(fake_models(), LoopConfig(rate_hz=20), mode="sweep", allow_hold=True)
    app = CompassApp(loop, bittle=bot, mock=mock)
    th = threading.Thread(target=app.run, kwargs={"duration": 4}, daemon=True)
    try:
        app.start_robot()
        th.start()
        time.sleep(1.5)
        assert mock.skill == "vtL"
        assert app.on_command({"action": "mode", "mode": "needle"})["ok"]
        time.sleep(0.8)
        assert mock.skill == "up" and mock.period == 1
        th.join(timeout=5)
    finally:
        app.stop_event.set()
        app.shutdown()
    assert mock.commands[-1] == "d"


def test_shutdown_mid_turn_stands_then_rests():
    mock = MockBittle(imu_noise_deg=0.1, turn_rate_deg_s=5.0)
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=1.0, quiet_s=0.3)
    loop = CompassLoop(fake_models(), LoopConfig(rate_hz=20), mode="sweep", allow_hold=True)
    app = CompassApp(loop, bittle=bot, mock=mock)
    try:
        app.start_robot()
        app.run(duration=1.0)
        assert loop.gait_active
    finally:
        app.shutdown()
    ks = mock.commands
    last_turn = max(i for i, c in enumerate(ks) if c in ("kvtL", "kvtR"))
    assert "kup" in ks[last_turn:]  # the gait is stopped first ...
    assert ks[-1] == "d"  # ... then the robot rests


# ---- stop watchdog ------------------------------------------------------------------------


def test_lost_stand_is_resent_and_next_leg_waits_for_the_stop():
    loop = sweep_loop(sweep_angle_deg=20, sweep_pause_s=1.0, sweep_cycles=1)
    dog = SimDog(rate=20, lose_stands=2)  # like the BLE run: the dog keeps turning at 20 deg/s
    run(loop, dog, 60, until=lambda lp: lp.sweep_state == "done")
    names = [s for _, s in dog.skills]
    # first leg end: up (lost), up (lost), up (works), then the right turn
    assert names[:5] == ["vtL", "up", "up", "up", "vtR"], names
    t = {i: tt for i, (tt, _) in enumerate(dog.skills)}
    assert t[2] - t[1] >= STOP_GRACE_S - 1e-6
    assert t[3] - t[2] >= STOP_RESEND_S - 1e-6
    assert loop.stop_resends_total == 2 and loop.stop_failures == 0
    assert any("re-sending stand (1/" in e for e in loop.events)
    assert loop.sweep_state == "done" and loop.mode == "sweep"


def test_normal_stop_needs_no_resend():
    loop = sweep_loop(sweep_angle_deg=20, sweep_pause_s=1.0, sweep_cycles=1)
    dog = SimDog(rate=20)
    run(loop, dog, 30, until=lambda lp: lp.sweep_state == "done")
    assert loop.stop_resends_total == 0
    assert [s for _, s in dog.skills] == ["vtL", "up", "vtR", "up"]


def test_stop_that_never_takes_falls_back_to_rest_and_observe():
    loop = sweep_loop(sweep_angle_deg=20, sweep_pause_s=1.0)
    dog = SimDog(rate=20, lose_stands=99)
    run(loop, dog, 30, until=lambda lp: lp.mode == "observe")
    names = [s for _, s in dog.skills]
    assert names.count("up") == 1 + STOP_MAX_RESENDS  # bounded
    assert names[-1] == "REST" and dog.rested
    ups = [tt for tt, n in dog.skills if n == "up"]
    assert min(b - a for a, b in itertools.pairwise(ups)) >= STOP_RESEND_S - 1e-6
    assert loop.mode == "observe" and loop.stop_failures == 1 and not loop.gait_active
    assert any(e.startswith("STOP FAILED") for e in loop.events)
    assert "vtR" not in names  # no new leg while the stop was unconfirmed


def test_hold_mode_uses_the_stop_watchdog_too():
    loop = CompassLoop(
        fake_models(), LoopConfig(hold_enter_deg=20, hold_exit_deg=6), mode="hold", allow_hold=True
    )
    dog = SimDog(rate=20, lose_stands=1)
    dog.yaw = 0.0
    t, dt = 0.0, 0.05
    while t < 12:
        if 0.5 <= t < 0.55:
            dog.yaw = 45.0  # pushed 45 deg left by hand: hold turns right
        acts = loop.update(t, [S(t, wrap_deg(dog.yaw))])
        dog.apply(t, acts.skill)
        dog.advance(t, dt)
        t = round(t + dt, 4)
    names = [s for _, s in dog.skills]
    assert names[:3] == ["vtR", "up", "up"], names
    assert loop.stop_resends_total == 1
    assert not loop.stop_pending


def test_stall_message_says_wrong_way():
    loop = sweep_loop(sweep_angle_deg=30, sweep_timeout_s=60)
    dog = SimDog(rate=5)
    run(loop, dog, 1.0)
    dog.gait = -1  # the robot turns the other way while "turning left"
    run_more = [
        loop.update(1.0 + i * 0.05, [S(1.0 + i * 0.05, wrap_deg(dog.yaw - i * 0.5))])
        for i in range(80)
    ]
    assert any(a.skill == "vtL" for a in run_more)
    msg = next(e for e in loop.events if "re-issuing" in e)
    assert "turning left" in msg and "WRONG way" in msg, msg


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_live_sweep_with_merging_firmware_loses_no_stand(link):
    """The simulated firmware merges any write that arrives within 80 ms of the previous one
    (and over BLE that breaks the first command). With IMU polling and the needle streaming,
    a full sweep still has every stand take effect and no write is ever merged."""
    mock = MockBittle(
        imu_noise_deg=0.1,
        turn_rate_deg_s=25.0,
        ble_notifications=link == "ble",
        merge_window_s=0.08,
    )
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=3.0, quiet_s=0.3)
    cfg = LoopConfig(rate_hz=20, sweep_angle_deg=15, sweep_pause_s=1.0, sweep_cycles=1)
    loop = CompassLoop(fake_models(), cfg, mode="sweep", allow_hold=True)
    app = CompassApp(loop, bittle=bot, mock=mock)
    app.imu_mode = "poll"
    try:
        app.start_robot()
        app.run(duration=40)
    finally:
        app.shutdown()
    assert loop.sweep_state == "done"
    assert mock.merged_writes == []
    assert loop.stop_resends_total == 0 and loop.stop_failures == 0
    assert any(c.startswith("i0 ") for c in mock.commands)
    assert mock.commands[-1] == "d"
