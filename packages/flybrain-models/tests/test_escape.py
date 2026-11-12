import json
import math

import numpy as np
import pytest
from flybrain_data import load_pack
from flybrain_models.escape import (
    EscapeCircuit,
    EscapeOutput,
    LoomingRateThreshold,
    ProximityThreshold,
    load_params,
    raw_to_proximity,
)
from flybrain_models.escape_benchmark import results_path
from flybrain_models.escape_tasks import (
    benchmark_set,
    label_recording,
    load_recording,
    make_episode,
    run_episode,
    run_recording,
)


@pytest.fixture(scope="module")
def model():
    return EscapeCircuit.from_pack("escape")


def test_raw_to_proximity():
    assert raw_to_proximity(3400) == 0.0
    assert raw_to_proximity(3300) == 0.0  # inside the noise deadband
    assert raw_to_proximity(0) == 1.0
    assert raw_to_proximity(5000) == 0.0 and raw_to_proximity(-10) == 1.0
    assert 0.4 < raw_to_proximity(1700) < 0.6
    assert raw_to_proximity(2900, baseline=3000) == 0.0 < raw_to_proximity(2900)


@pytest.mark.parametrize("variant", ["connectome", "rewired", "random"])
def test_api_contract(variant):
    m = EscapeCircuit.from_pack("escape", variant=variant, seed=0)
    assert isinstance(m.n_neurons, int) and m.n_neurons == 351
    m.reset()
    for dt in (0.035, 0.09, 0.05, 0.0):  # variable dt, including a zero step
        out = m.step(0.3, 0.1, dt)
        assert isinstance(out, EscapeOutput)
        assert isinstance(out.trigger, bool) and out.side in (-1, 0, 1)
        assert 0.0 <= out.gf_left <= 1.0 and 0.0 <= out.gf_right <= 1.0
        th_l, th_r = m.gf_thresholds
        assert out.drive == pytest.approx(max(out.gf_left / th_l, out.gf_right / th_r))
        assert out.threshold == m.threshold == 1.0
    act = m.activity
    for k in ("LPLC2_L", "LC4_L", "LPLC2_R", "LC4_R", "GF"):
        assert k in act
    assert act["GF"].shape == (2,)
    assert len(act["LC4_L"]) == 54 and len(act["LPLC2_R"]) == 102


def test_reset_clears_state(model):
    ep = make_episode("approach", np.random.default_rng(0), 28.0, "normal", 1.0, 1)
    a = run_episode(model, ep)
    b = run_episode(model, ep)  # run_episode resets first
    assert a["triggers"] == b["triggers"] and np.array_equal(a["drive"], b["drive"])
    model.reset()
    assert not model.r.any()


def test_determinism():
    ep = benchmark_set()[5]
    for v in ("connectome", "random"):
        a = run_episode(EscapeCircuit.from_pack(variant=v, seed=2), ep)
        b = run_episode(EscapeCircuit.from_pack(variant=v, seed=2), ep)
        assert np.array_equal(a["drive"], b["drive"])


def test_dt_invariance(model):
    # the same approach sampled at 12 Hz and 28 Hz triggers within one slow sample
    trig = []
    for rate in (12.0, 28.0):
        ep = make_episode("approach", np.random.default_rng(3), rate, "normal", 0.8, 0)
        r = run_episode(model, ep)
        assert r["triggers"], rate
        trig.append(r["triggers"][0] - ep.t_gt)
    assert abs(trig[0] - trig[1]) < 0.15


def test_quiet_scene_never_triggers(model):
    model.reset()
    for _ in range(300):
        assert not model.step(0.0, 0.0, 0.036).trigger


def test_refractory(model):
    model.reset()
    fired = []
    for i in range(200):  # repeated fast jabs every 0.36 s
        p = 1.0 if (i % 10) in (2, 3, 4) else 0.0
        fired.append(model.step(p, p, 0.036).trigger)
    t = np.flatnonzero(fired) * 0.036
    assert len(t) >= 1 and (np.diff(t) >= 1.0 - 1e-9).all()


def test_triggers_on_recorded_approaches(model):
    res = run_recording(model)
    assert sum(e["hit"] for e in res["events"]) >= 4
    assert len(res["false_alarm_times"]) <= 1


def test_recording_labels():
    t, left, right = load_recording()
    ev = label_recording(t, left, right)
    assert len(ev) == 5 and all(e["approach"] for e in ev)


def test_baselines_api():
    for cls in (ProximityThreshold, LoomingRateThreshold):
        m = cls()
        m.reset()
        outs = [m.step(p, 0.0, 0.036) for p in (0.0, 0.2, 0.9, 1.0, 1.0)]
        assert any(o.trigger for o in outs)
        assert all(isinstance(o, EscapeOutput) for o in outs)
        assert outs[-1].side in (0, 1)


def test_params_small_and_global():
    p = load_params("connectome")
    assert set(p["gain"]) == {"vpn", "exc", "inh", "gf"}
    assert set(p["bias"]) == {"vpn", "in", "gf"}
    assert EscapeCircuit.from_pack().n_params_fitted == 11
    assert 0 < p["threshold_left"] < 1 and 0 < p["threshold_right"] < 1


def test_benchmark_json():
    data = json.loads(results_path().read_text())
    for k in ("sim_hit_rate", "sim_fa_per_min", "sim_latency_ms", "real_hits_of_5"):
        assert set(data["key_numbers"][k]) >= {
            "connectome",
            "rewired",
            "random",
            "proximity_threshold",
            "looming_rate_threshold",
        }


def test_pack_object_and_ablation():
    pk = load_pack("escape")
    m = EscapeCircuit.from_pack(pk, variant="rewired", seed=1)
    assert m.variant == "rewired"
    a = EscapeCircuit.from_pack(params={**load_params("connectome"), "ablate_interneurons": True})
    assert not a._W[:, a._interneurons].any()
    assert math.isfinite(a.step(1.0, 1.0, 0.05).drive)


@pytest.mark.parametrize("side", [1, -1])
def test_each_gf_triggers_on_its_own_side(model, side):
    # regression for the robot finding: approaches seen by ONE channel only must trigger, on
    # either side, with the right side reported
    rng = np.random.default_rng(7)
    for v in (0.5, 1.0):
        ep = make_episode("approach", rng, 28.0, "normal", v, side, unilateral=True)
        assert ep.raw_l.min() > 3000 or ep.raw_r.min() > 3000  # far channel never responds
        res = run_episode(model, ep)
        assert res["triggers"], (side, v)
        assert res["sides"][0] == side


def test_one_sided_drive_reaches_threshold(model):
    for left, right in ((1.0, 0.0), (0.0, 1.0)):
        model.reset()
        fired = False
        for p in [0.0] * 10 + [0.3, 0.7] + [1.0] * 10:
            fired |= model.step(p * left, p * right, 0.036).trigger
        assert fired, (left, right)


def test_robot_run_both_sides():
    res = run_recording(EscapeCircuit.from_pack(), "robot_run")
    ev = res["events"]
    left = [e for e in ev if e["leading_side"] == 1]
    right = [e for e in ev if e["leading_side"] == -1]
    assert len(left) >= 2 and len(right) >= 5
    assert all(e["hit"] for e in left) and sum(e["hit"] for e in right) >= len(right) - 1


def test_looming_side_uses_nearer_channel():
    m = LoomingRateThreshold(threshold=1.0)
    m.reset()
    m.step(0.0, 0.0, 0.036)
    m.step(0.0, 1.0, 0.036)  # right channel already saturated
    out = m.step(0.6, 1.0, 0.036)  # left rises now: rate is on the left, the hand is right
    assert out.side == -1
