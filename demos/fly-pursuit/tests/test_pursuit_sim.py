"""The simulation core, stepped directly: no threads, no sleeping."""

import math

import pytest
from fly_pursuit.controllers import CONTROLLER_KEYS, steering_curve
from fly_pursuit.sim import FOV_CHOICES, MOTIONS, PursuitSim, wrap
from flybrain_models import pursuit_tasks as T


def settle_then_step(controller, step_deg, fov=30, seconds=5.0):
    """Target straight ahead for 1 s, then a jump of ``step_deg``; returns the sim after
    ``seconds`` more and the mean |error| of the last second."""
    sim = PursuitSim(controller=controller, fov_deg=fov, target_deg=0.0, seed=1)
    sim.advance(1.0)
    sim.set_target(math.degrees(sim.heading) + step_deg)
    sim.advance(seconds - 1.0)
    sim.reset_stats()
    sim.advance(1.0)
    return sim, sim.stats()["mae_deg"]


def test_connectome_turns_toward_a_frontal_step():
    sim, late = settle_then_step("connectome", 20.0)
    assert late < 6.0
    assert abs(math.degrees(sim.error)) < 8.0


def test_without_aotu019_a_frontal_target_is_lost():
    _, late = settle_then_step("no-aotu019", 20.0)
    assert late > 20.0  # it turns away from a target near the centre


def test_single_brain_holds_the_target_off_centre():
    sim = PursuitSim(controller="single-brain", fov_deg=60, target_deg=0.0, seed=2)
    sim.advance(6.0)
    sim.reset_stats()
    sim.advance(4.0)
    assert 8.0 < sim.stats()["mae_deg"] < 30.0


def test_dynamics_match_the_benchmark_rig():
    """Same numbers as pursuit_tasks.Rig, and a turn never faster than the limit."""
    sim = PursuitSim(controller="bang-bang", fov_deg=100, target_deg=90.0)
    assert sim.rig == T.Rig(fov_half_deg=100.0)
    peak = 0.0
    for _ in range(300):
        sim.substep()
        peak = max(peak, abs(math.degrees(sim.rate)))
    assert 0.0 < peak <= T.MAX_TURN_DEG_S + 1e-9
    assert sim.frames == 300 // T.WORLD_SUBSTEPS


def test_camera_reports_after_the_delay_and_only_in_view():
    sim = PursuitSim(controller="proportional", fov_deg=30, target_deg=0.0)
    sim.substep()
    assert sim.camera == "waiting"  # nothing captured delay_s ago yet
    sim.set_controller("bang-bang")
    sim.advance(1.0)
    assert sim.camera in ("seen", "dropped")
    sim.set_target(math.degrees(sim.heading) + 120.0)  # outside +-30
    sim.advance(1.0)
    assert sim.camera == "outside" and sim.measured is None
    sim.set_motion("hidden")
    sim.advance(0.5)
    assert sim.camera == "hidden" and not sim.exists


def test_switching_the_controller_keeps_the_world():
    sim = PursuitSim(controller="connectome", target_deg=25.0)
    sim.advance(0.5)
    heading, rate, theta = sim.heading, sim.rate, sim.theta
    assert sim.set_controller("proportional") is None
    assert (sim.heading, sim.rate, sim.theta) == (heading, rate, theta)
    assert sim.turn == 0.0 and sim.cmd == 0.0


def test_settings_change_state_and_reject_bad_values():
    sim = PursuitSim()
    assert sim.set_fov(60) is None and sim.rig.fov_half_deg == 60.0
    for bad in (45, "wide", None, float("nan"), True):
        assert sim.set_fov(bad)
    assert sim.rig.fov_half_deg == 60.0
    assert sim.set_arousal(0.25) is None and sim.arousal == 0.25
    for bad in (-0.1, 1.5, "x", None, float("inf")):
        assert sim.set_arousal(bad)
    assert sim.arousal == 0.25
    assert sim.set_controller("nope") and sim.controller == "connectome"
    assert sim.set_motion("dance") and sim.motion == "still"
    assert sim.set_target("left") and sim.set_target(None)
    assert sim.nudge_target(500)
    assert sim.set_target(-45) is None and math.isclose(sim.theta, math.radians(-45))
    assert sim.nudge_target(10) is None and math.isclose(sim.theta, math.radians(-35))
    assert sim.set_target(270) is None and math.isclose(sim.theta, wrap(math.radians(270)))


def test_motions():
    for m in MOTIONS:
        sim = PursuitSim(motion=m, fov_deg=60, seed=3)
        start = sim.theta
        sim.advance(4.5)
        if m == "still":
            assert sim.theta == start
        elif m == "hidden":
            assert not sim.exists and sim.stats()["mae_deg"] is None
        else:
            assert sim.theta != start and sim.exists
    sim = PursuitSim(motion="sweep")
    sim.set_target(10)  # dragging the target stops a motion
    assert sim.motion == "still"


def test_jumps_land_inside_the_field_of_view():
    sim = PursuitSim(motion="jumps", fov_deg=30)
    sides = []
    for _ in range(6):
        n = sim._jump_n
        while sim._jump_n == n:  # run until the next jump
            sim.substep()
        assert abs(sim.error) <= math.radians(0.9 * 30) + 1e-9
        sides.append(sim.error > 0)
    assert sides == [False, True] * 3  # alternating sides (the first jump, at the start, was left)


def test_stats_reset():
    sim = PursuitSim(target_deg=40.0)
    sim.advance(1.0)
    assert sim.stats()["seconds"] == pytest.approx(1.0, abs=0.02)
    sim.reset_stats()
    assert sim.stats() == {"seconds": 0.0, "mae_deg": None, "within10": None, "in_view": None}


def test_every_controller_runs():
    sim = PursuitSim(fov_deg=100, target_deg=60.0)
    for key in CONTROLLER_KEYS:
        assert sim.set_controller(key) is None
        sim.advance(0.3)
        assert math.isfinite(sim.turn)


def test_steering_curve_is_antisymmetric_for_the_connectome():
    sim = PursuitSim()
    curve = steering_curve(sim.model, 1.0)
    assert len(curve) == 73
    assert curve[36] == 0.0  # 0 deg
    assert curve[36 + 4] > 0 > curve[36 - 4]  # +20 deg: turn left
    assert curve[36 + 4] == pytest.approx(-curve[36 - 4], abs=1e-6)


def test_fov_choices_are_the_benchmark_settings():
    assert FOV_CHOICES == (30, 60, 100)
