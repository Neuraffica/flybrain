import math
from dataclasses import dataclass

import pytest
from compass_fakes import FakeRingNoCue, fake_models, fly_like_models
from fly_compass.loop import CompassLoop, wrap_deg
from fly_compass.scenarios import SCENARIOS, ScenarioRunner


@dataclass
class S:
    t: float
    yaw_deg: float


def drive(loop, yaw_at, seconds, imu_hz=5.0, rate=20.0, on_tick=None):
    t, nxt = 0.0, 0.0
    while t <= seconds + 1e-9:
        samples = []
        if t >= nxt - 1e-9:
            samples.append(S(t, wrap_deg(yaw_at(t))))
            nxt += 1.0 / imu_hz
        if on_tick:
            on_tick(t)
        loop.update(t, samples)
        t = round(t + 1.0 / rate, 6)


def test_cue_only_on_fresh_samples_and_never_blindfolded():
    models = fake_models()
    loop = CompassLoop(models, mode="observe")
    drive(loop, lambda t: 30.0, 2.0)
    cues = models["connectome"].cues
    given = [c for c in cues if c is not None]
    assert 8 <= len(given) <= 11  # 5 Hz over 2 s, with 20 Hz ticks
    assert len(cues) - len(given) > 25  # the other ticks are dark
    # the cue is the measured yaw in the re-zeroed heading frame (zero = first sample)
    assert all(abs(c) < 1e-9 for c in given)
    loop.set_blindfold(True, 2.0)
    n0 = len(cues)
    drive(loop, lambda t: 30.0 + 10 * t, 1.0)
    assert all(c is None for c in cues[n0:])
    fr = loop.frame(3.0)
    assert fr["blindfold"]["on"] and fr["cue"] is False and fr["phase"] == "blindfolded"


def test_cue_value_is_in_heading_frame():
    models = fake_models()
    loop = CompassLoop(models, mode="observe")
    drive(loop, lambda t: 37 + (40 if t > 1 else 0), 2.0)
    last = [c for c in models["gyro"].cues if c is not None][-1]
    assert math.degrees(last) == pytest.approx(40, abs=0.5)


def test_models_without_cue_input_still_run():
    models = fake_models()
    models["rewired"] = FakeRingNoCue()
    loop = CompassLoop(models, mode="observe")
    drive(loop, lambda t: 20 * t, 2.0)
    fr = loop.frame(2.0)
    assert fr["models"]["rewired"]["accepts_cue"] is False
    assert fr["models"]["connectome"]["accepts_cue"] is True
    assert abs(fr["models"]["rewired"]["err"]) < 10


def run_scenario(name, models):
    """Run a scenario on simulated time; returns the loop's per-phase summary."""
    loop = CompassLoop(models, mode="observe")
    runner = ScenarioRunner(name, 0.0, 37.0)
    state = {"i": -1}

    def tick(t):
        i, _, _ = runner.at(t)
        if i != state["i"] and i < len(runner.phases):
            state["i"] = i
            loop.phase_override = runner.phases[i].label
            loop.set_blindfold(runner.phases[i].blindfold, t)

    drive(loop, lambda t: runner.at(t)[2], runner.total_s - 0.05, on_tick=tick)
    return loop.phase_summary()


def test_memory_scenario_separates_holding_from_drifting():
    ph = run_scenario("memory", fly_like_models())
    late = ph["blindfold, still 5-20 s"]
    # cue-anchored before the blindfold: everyone is close
    assert ph["hold (cue on)"]["connectome"]["mae_deg"] < 5
    # held still in the dark: the holding ring keeps the heading, the pinned control does not
    assert abs(late["connectome"]["final_err_deg"]) < 5
    assert abs(late["rewired"]["final_err_deg"]) > 30
    assert abs(late["gyro"]["final_err_deg"]) < 5
    # cue back: re-anchored
    assert abs(ph["cue back"]["rewired"]["final_err_deg"]) < 5


def test_dark_turn_scenario_rings_lose_track_filter_does_not():
    ph = run_scenario("dark-turn", fly_like_models())
    after = ph["blindfold, after turn"]
    assert abs(after["connectome"]["final_err_deg"]) > 60  # integrates only 10 % of the turn
    assert abs(after["gyro"]["final_err_deg"]) < 8
    assert abs(ph["cue back"]["connectome"]["final_err_deg"]) < 5


def test_scenarios_are_well_formed():
    for name, phases in SCENARIOS.items():
        assert any(p.blindfold for p in phases), name
        assert not phases[-1].blindfold, f"{name} must end with the cue back"
        r = ScenarioRunner(name, 10.0, 0.0)
        i, _, yaw = r.at(10.0 + r.total_s + 1)
        assert i == len(phases) and yaw == pytest.approx(sum(p.turn_deg for p in phases))
