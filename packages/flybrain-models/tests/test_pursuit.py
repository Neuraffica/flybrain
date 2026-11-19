import json
import math

import numpy as np
import pytest
from flybrain_models.pursuit import (
    AROUSAL_THETA,
    DNA02_BIAS,
    BangBangSteer,
    ProportionalSteer,
    PursuitCircuit,
    PursuitOutput,
    load_params,
)
from flybrain_models.pursuit_benchmark import CURVE_DEG, results_path
from flybrain_models.pursuit_tasks import (
    Rig,
    benchmark_set,
    evaluate,
    idle_turn,
    make_episode,
    run_episode,
    score_episode,
    training_set,
)

DEG = math.pi / 180


@pytest.fixture(scope="module")
def model():
    return PursuitCircuit.from_pack("pursuit")


def test_api_contract(model):
    model.reset()
    assert (model.r[model._dn] == DNA02_BIAS).all()
    for az, dt in ((0.3, 0.066), (None, 0.1), (-0.2, 0.033), (0.1, 0.0)):
        out = model.step(az, dt, arousal=0.5)
        assert isinstance(out, PursuitOutput)
        assert min(out.dna02_left, out.dna02_right, out.aotu019_left, out.aotu025_right) >= 0
    act = model.activity
    assert len(act["LC10a_L"]) == 115 and len(act["LC10a_R"]) == 119
    for k in ("AOTU019", "AOTU025", "DNa02"):
        assert act[k].shape == (2,)
    az = model.lc10a_azimuth
    assert np.nanmedian(az["LC10a_L"]) > 0 > np.nanmedian(az["LC10a_R"])
    assert model.n_neurons == 265 and model.n_params_fitted == 1


def test_weights_are_input_fractions(model):
    pk = model.pack
    W = model._W
    expect = pk.counts * pk.sign[None, :] / pk.input_total[:, None]
    assert np.allclose(W, expect)
    assert np.abs(W).sum(1).max() <= 1.0


@pytest.mark.parametrize("deg", [5, 10, 20, 40, 60, 90])
def test_turns_toward_target(model, deg):
    assert model.steady_turn(deg * DEG) > 0  # target on the left -> turn left
    assert model.steady_turn(-deg * DEG) < 0


def test_mirror_average_is_antisymmetric_raw_is_not():
    m = PursuitCircuit.from_pack()
    raw = PursuitCircuit.from_pack(mirror_average=False)
    for deg in (5, 20, 60):
        assert m.steady_turn(deg * DEG) == pytest.approx(-m.steady_turn(-deg * DEG), abs=1e-9)
    assert abs(m.steady_turn(0.0)) < 1e-9
    # the single brain turns left for a target straight ahead (unequal halves)
    assert raw.steady_turn(0.0) > 0.5


@pytest.mark.parametrize("arousal", [0.0, 1.0])
def test_no_target_no_turn(model, arousal):
    model.reset()
    for _ in range(30):
        out = model.step(None, 1 / 15, arousal=arousal)
    assert abs(out.turn) < 1e-9
    assert idle_turn(model, arousal) < 1e-6


def test_arousal_silences_aotu019(model):
    diff = model.drive(None, 0.0) - model.drive(None, 1.0)
    assert np.allclose(diff[model._a19], -AROUSAL_THETA) and np.count_nonzero(diff) == 2
    for a, active in ((0.0, False), (1.0, True)):
        model.reset()
        model.step(15 * DEG, 0.5, arousal=a)
        assert (
            model.activity["AOTU019"].max() > 0 if active else not model.activity["AOTU019"].any()
        )


def test_ablation_and_sign_flip():
    base = PursuitCircuit.from_pack()
    no19 = PursuitCircuit.from_pack(ablate=("AOTU019",))
    no25 = PursuitCircuit.from_pack(ablate=("AOTU025",))
    exc19 = PursuitCircuit.from_pack(aotu019_sign=1)
    assert exc19.aotu019_sign == 1 and base.aotu019_sign == -1
    for deg in (10, 15):
        t = base.steady_turn(deg * DEG)
        # without AOTU019, or with it excitatory, a frontal target is turned AWAY from
        assert no19.steady_turn(deg * DEG) < 0 < t
        assert exc19.steady_turn(deg * DEG) < 0
    # silencing AOTU025 weakens turning toward lateral targets
    for deg in (60, 90):
        assert 0 < no25.steady_turn(deg * DEG) < 0.6 * base.steady_turn(deg * DEG)
    no25.reset()
    no25.step(80 * DEG, 0.5)
    assert not no25.activity["AOTU025"].any()


def test_controls_differ():
    az = np.radians(CURVE_DEG[::3])
    base = PursuitCircuit.from_pack()
    ref = np.array([base.steady_turn(a) for a in az])
    for v in ("rewired", "random"):
        m = PursuitCircuit.from_pack(variant=v, seed=0)
        assert m.variant == v and m.k_out == base.k_out
        assert np.array_equal(m.pack.azimuth, base.pack.azimuth, equal_nan=True)
        curve = np.array([m.steady_turn(a) for a in az])
        assert np.abs(curve - ref).max() > 0.5


@pytest.mark.parametrize("s", [1, -1])
def test_closed_loop_step_settles(model, s):
    ep = make_episode("step", np.random.default_rng(0), Rig(), step=s * 20 * DEG)
    res = run_episode(model, ep)
    err = np.abs(np.degrees(res["az"]))
    assert err[ep.t >= 1.0][0] > 15  # the robot jitters a little around 0 before the jump
    assert err[ep.t >= 4.0].max() < 6
    sc = score_episode(ep, res)
    assert sc["settle_s"] is not None and sc["mae_deg"] < 8


def test_baselines():
    p = ProportionalSteer(k=2.0)
    assert p.step(0.1, 0.066).turn == pytest.approx(0.2)
    assert p.step(None, 0.066).turn == 0.0
    b = BangBangSteer(threshold=5 * DEG)
    assert b.step(3 * DEG, 0.066).turn == 0.0
    assert b.step(10 * DEG, 0.066).turn == pytest.approx(math.radians(90))
    assert b.step(-10 * DEG, 0.066).turn == pytest.approx(-math.radians(90))
    assert b.step(None, 0.066).turn == 0.0
    assert ProportionalSteer().k > 0 and BangBangSteer().threshold > 0


def test_params_one_scalar_each():
    data = json.loads((results_path().parent / "pursuit_params.json").read_text())
    assert set(data["fits"]) == {"connectome", "proportional", "bang_bang"}
    for fit in data["fits"].values():
        assert len(fit["params"]) == 1 and fit["date"]
    assert load_params("connectome")["k_out"] > 0


def test_task_sets_disjoint():
    tr, bm = training_set(), benchmark_set()
    assert len(tr) == 38 and len(bm) == 114
    assert {e.rig.fov_half_deg for e in tr} == {e.rig.fov_half_deg for e in bm} == {30, 60, 100}
    assert not any(np.array_equal(a.noise, b.noise) for a in tr for b in bm[:24])
    wide = [e for e in tr if e.rig.fov_half_deg == 100 and e.kind in ("step", "appear")]
    assert max(abs(e.step) for e in wide) > 60 * DEG
    ap = next(e for e in wide if e.kind == "appear")
    assert not ap.exists[0] and ap.exists[-1]


def test_benchmark_json_matches_rerun():
    data = json.loads(results_path().read_text())
    kn = data["key_numbers"]
    models = {"connectome", "connectome_raw", "rewired", "random", "proportional", "bang_bang"}
    for k in ("mae_deg", "within10", "in_view", "idle_turn_deg_s"):
        assert models <= set(kn[k])
    assert set(kn["by_setting"]) == {"fov30", "fov60", "fov100"}
    assert kn["antisymmetry_error_deg_s"]["connectome"] < 1e-6
    assert kn["antisymmetry_error_deg_s"]["connectome_raw"] > 10
    # quick re-run of one case: the proportional baseline on the whole held-out set
    summ = evaluate(ProportionalSteer(), benchmark_set())[0]
    assert summ["mae_deg"] == pytest.approx(kn["mae_deg"]["proportional"], abs=1e-3)
    assert summ["within10"] == pytest.approx(kn["within10"]["proportional"], abs=1e-3)
    # and two points of the circuit's steering curve
    m = PursuitCircuit.from_pack()
    curve = dict(
        zip(data["steering_curve_azimuth_deg"], data["steering_curve_deg_s"]["connectome"])
    )
    for deg in (-30, 75):
        assert math.degrees(m.steady_turn(deg * DEG)) == pytest.approx(curve[deg], abs=1e-2)


def test_mirror_averaged_dna02_difference_is_the_turn():
    c = PursuitCircuit.from_pack()
    out = c.step(math.radians(25.0), 0.5)
    dn = 0.5 * (c.activity["DNa02"] + c.activity_mirror["DNa02"][::-1])  # [left, right]
    assert out.turn == pytest.approx(c.k_out * (dn[0] - dn[1]), rel=1e-9)
    assert dn[0] > dn[1]  # target on the left: the left steering neuron leads
