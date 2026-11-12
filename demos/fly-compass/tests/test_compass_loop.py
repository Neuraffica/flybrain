import itertools
import math
from dataclasses import dataclass

import pytest
from compass_fakes import fake_models
from fly_compass.loop import CompassLoop, LoopConfig, wrap_deg


@dataclass
class S:
    t: float
    yaw_deg: float


def run(loop, traj, seconds, imu_hz=5.0, rate=20.0, yaw0=37.0):
    """Feed IMU samples of traj(t) (world deg, firmware wrap) at imu_hz; tick at rate."""
    out = []
    next_s = 0.0
    t = 0.0
    while t <= seconds + 1e-9:
        samples = []
        if t >= next_s - 1e-9:
            samples.append(S(t, wrap_deg(yaw0 + traj(t))))
            next_s += 1.0 / imu_hz
        acts = loop.update(t, samples)
        out.append((t, acts, loop.frame(t)))
        t += 1.0 / rate
    return out


def test_tracks_scripted_rotation_through_wrap():
    loop = CompassLoop(fake_models(), LoopConfig(), mode="observe")
    loop.set_blindfold(True, 0.0)  # velocity only: no cue can hide the integration error

    # 0 -> +200 deg at 50 deg/s (crosses the firmware's +-180 wrap), hold, then back
    def traj(t):
        return 50 * min(t, 4.0) - 50 * max(0.0, min(t - 6.0, 4.0))

    res = run(loop, traj, 12.0)
    _t, _, fr = res[-1]
    assert abs(fr["truth"]) < 0.5
    # truth itself is unwrapped correctly mid-way
    mid = next(f for (tt, _, f) in res if 4.9 < tt < 5.1)
    assert abs(wrap_deg(mid["truth"] - 200)) < 1.0
    # a perfect integrator settles onto truth; lag only during motion
    assert abs(fr["models"]["connectome"]["err"]) < 1.0
    assert abs(mid["models"]["connectome"]["err"]) < 3.0  # still settling 1 s after the stop
    assert fr["models"]["connectome"]["mae"] < 15
    # the 0.8-gain control drifts by 20% of the 400 deg travelled
    assert abs(mid["models"]["rewired"]["err"]) > 30
    assert len(fr["models"]["connectome"]["bump"]) == 16


def test_omega_is_derived_from_yaw_and_does_not_drift():
    loop = CompassLoop(fake_models(), LoopConfig(), mode="observe")
    res = run(loop, lambda t: 30 * t, 6.0)
    _, _, fr = res[-1]
    assert abs(fr["omega"] - 30) < 3
    assert abs(loop.fed - loop.truth_rel) < 10  # one sample interval of lag at 30 deg/s


def test_needle_counter_rotates_and_clamps_and_rate_limits():
    cfg = LoopConfig(pan_limit_deg=80, pan_rate_dps=120)
    loop = CompassLoop(fake_models(), cfg, mode="needle")
    res = run(loop, lambda t: 40 * min(t, 1.5), 4.0)  # turn left 60 deg
    assert abs(loop.pan_cmd - (-60)) < 2  # head turns right by 60
    # rate limit: no tick changes the command by more than rate*dt (+1 for rounding)
    sent = [a.pan_deg for _, a, _ in res if a.pan_deg is not None]
    steps = [abs(b - a) for a, b in itertools.pairwise(sent)]
    assert max(steps) <= 120 / 20 + 1
    # beyond the limit it saturates and never exceeds it
    loop2 = CompassLoop(fake_models(), cfg, mode="needle")
    res2 = run(loop2, lambda t: 50 * min(t, 3.0), 6.0)  # 150 deg left
    assert loop2.pan_saturated
    assert all(abs(a.pan_deg) <= 80 for _, a, _ in res2 if a.pan_deg is not None)
    assert loop2.pan_cmd == pytest.approx(-80)


def test_needle_does_not_flip_sides_behind_the_dog():
    loop = CompassLoop(fake_models(), LoopConfig(), mode="needle")
    # go to 170 left, then wobble across 180
    res = run(
        loop, lambda t: 170 + (20 * math.sin(t * 3) if t > 5 else 0) if t > 4 else 42.5 * t, 9.0
    )
    late = [a.pan_deg for t, a, _ in res if t > 5 and a.pan_deg is not None]
    assert all(p <= 0 for p in late) or not late
    assert loop.pan_cmd == pytest.approx(-80)


def test_pan_sign_inverts():
    loop = CompassLoop(fake_models(), LoopConfig(pan_sign=-1), mode="needle")
    run(loop, lambda t: 30 * min(t, 1.0), 3.0)
    assert loop.pan_cmd == pytest.approx(30, abs=2)


def test_hold_requires_permission():
    loop = CompassLoop(fake_models(), mode="observe")
    assert loop.set_mode("hold") is not None
    assert loop.mode == "observe"
    assert loop.set_mode("nope") is not None
    assert CompassLoop(fake_models(), mode="hold", allow_hold=True).mode == "hold"


def test_hold_turns_back_with_hysteresis():
    loop = CompassLoop(
        fake_models(), LoopConfig(hold_enter_deg=20, hold_exit_deg=6), mode="hold", allow_hold=True
    )
    skills = []
    state = {"yaw": 0.0}

    def traj(t):
        return state["yaw"]

    t = 0.0
    nxt = 0.0
    turning = 0
    while t < 20:
        samples = []
        if t >= nxt:
            samples = [S(t, wrap_deg(state["yaw"]))]
            nxt += 0.2
        a = loop.update(t, samples)
        if a.skill:
            skills.append((round(t, 2), a.skill))
            turning = {"vtL": 1, "vtR": -1}.get(a.skill, 0)
        if 1.0 <= t < 1.05:
            state["yaw"] = 45.0  # someone turns the dog 45 deg left
        state["yaw"] += turning * 40 * 0.05  # vtL turns CCW (+), vtR clockwise (-)
        t += 0.05
    names = [s for _, s in skills]
    assert names[:2] == ["vtR", "up"]
    assert abs(wrap_deg(state["yaw"])) <= 12
    assert loop.hold_state in ("idle", "cooldown")


def test_hold_times_out_and_disables():
    cfg = LoopConfig(hold_max_turn_s=1.0, hold_cooldown_s=0.2, hold_max_failures=2)
    loop = CompassLoop(fake_models(), cfg, mode="hold", allow_hold=True)
    t = 0.0
    skills = []
    while t < 10:
        samples = [S(t, 90.0 if t > 0.5 else 0.0)] if abs((t / 0.2) - round(t / 0.2)) < 1e-6 else []
        a = loop.update(t, samples)
        if a.skill:
            skills.append(a.skill)
        t = round(t + 0.05, 4)
    assert loop.hold_state == "disabled"
    assert skills == ["vtR", "up", "vtR", "up"]  # dog is 90 deg left: turn right


def test_rezero_resets_models_and_scores():
    models = fake_models()
    loop = CompassLoop(models, mode="observe")
    run(loop, lambda t: 20 * t, 2.0)
    assert loop.state["connectome"].n > 0
    loop.rezero()
    fr = loop.frame(99)
    assert fr["truth"] == pytest.approx(0, abs=1e-6)
    assert loop.state["connectome"].n == 0
    assert models["connectome"]._h == 0.0
