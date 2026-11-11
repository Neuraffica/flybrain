import json
import math
import time

import numpy as np
import pytest
from flybrain_data import load_pack
from flybrain_models.compass import CUE_HOLD_S, GyroIntegrator, RingAttractor, load_params, wrap
from flybrain_models.compass_benchmark import results_path
from flybrain_models.compass_tasks import Trajectory, benchmark_set, memory_trial, piecewise, run

VARIANTS = ["connectome", "rewired", "random"]
KEY_NUMBERS = [
    "dead_reckoning_mae_deg", "cue_tracking_mae_deg", "cue_tracking_noisy_mae_deg",
    "cue_tracking_lag_ms", "memory_held_frac_5s", "memory_held_frac_30s",
    "memory_held_frac_60s", "memory_err_deg_5s", "memory_err_deg_30s", "memory_err_deg_60s",
    "memory_bump_frac_60s", "dark_rotation_err_deg", "dark_rotation_held_frac",
]


@pytest.fixture(scope="module")
def model():
    return RingAttractor.from_pack("central_complex")


@pytest.mark.parametrize("variant", VARIANTS)
def test_api_contract(variant):
    m = RingAttractor.from_pack("central_complex", variant=variant, seed=0)
    assert isinstance(m.n_neurons, int) and m.n_neurons == 151
    m.reset(1.0)
    for w, cue in ((0.0, None), (1.5, 0.3), (-3.0, None), (0.0, -2.0)):
        h = m.step(w, 0.02, cue)
        assert isinstance(h, float) and -math.pi < h <= math.pi
    h = m.step(0.1, 0.02)  # positional call without cue still works
    assert isinstance(h, float)
    b, a = m.bump, m.bump_angles
    assert b.shape == a.shape == (47,)
    assert np.all(np.diff(a) > 0) and np.all((a > -math.pi) & (a <= math.pi))
    assert np.all(b >= 0) and np.all(b <= 1)
    assert 0.0 <= m.confidence <= 1.0


def test_from_pack_accepts_pack_object():
    pack = load_pack("central_complex")
    m1 = RingAttractor.from_pack(pack, variant="rewired", seed=2)
    m2 = RingAttractor.from_pack("central_complex", variant="rewired", seed=2)
    assert np.array_equal(m1._W, m2._W)


def test_bump_frame_matches_heading(model):
    for h in (-2.5, 0.0, 0.7, math.pi):
        model.reset(h)
        for _ in range(20):
            model.step(0.0, 0.02, h)
        if model.confidence > 0:
            pv = np.angle((model.bump * np.exp(1j * model.bump_angles)).sum())
            assert abs(wrap(pv - model.heading)) < 1e-6


def test_cue_hold_and_darkness(model):
    model.reset(0.0)
    model.step(0.0, 0.02, 1.0)
    assert model.cue_active
    for _ in range(round(CUE_HOLD_S / 0.02) + 1):
        model.step(0.0, 0.02)
    assert not model.cue_active  # no cue for longer than CUE_HOLD_S = darkness


def test_cue_moves_bump(model):
    # a 5 Hz cue held for 3 s pulls the bump to the cue at most headings
    hits = []
    for h in np.linspace(-np.pi, np.pi, 12, endpoint=False):
        r = memory_trial(model, float(h), cue_s=3.0, dark_s=(0.02,))
        err, conf = r["cue_end"]
        hits.append(conf > 0.3 and abs(err) < math.radians(30))
    assert np.mean(hits) >= 0.5


def test_no_recurrence_ablation_has_no_weights():
    p = {**load_params("connectome"), "ablate_recurrence": True}
    m = RingAttractor.from_pack(params=p)
    assert not m._W.any()


def test_determinism():
    tr = benchmark_set()[8]  # stop_and_go, clean
    a = run(RingAttractor.from_pack(), tr)["decoded"]
    b = run(RingAttractor.from_pack(), tr)["decoded"]
    assert np.array_equal(a, b)
    ra = RingAttractor.from_pack(variant="random", seed=4)
    rb = RingAttractor.from_pack(variant="random", seed=4)
    assert np.array_equal(ra._W, rb._W)


def test_step_is_cheap(model):
    model.reset(0.0)
    n = 500
    t0 = time.perf_counter()
    for i in range(n):
        model.step(math.sin(i / 50), 0.02, 0.5 if i % 10 == 0 else None)
    per_step = (time.perf_counter() - t0) / n
    assert per_step < 1e-3  # three models at 50 Hz must fit easily in real time


def test_complementary_filter():
    g = GyroIntegrator()
    g.reset(0.3)
    for _ in range(100):
        g.step(math.radians(90), 0.02)  # no cue: pure integration, 2 s at 90 deg/s
    assert g.heading == pytest.approx(wrap(0.3 + math.pi))
    g.reset(0.0)
    for _ in range(20):
        g.step(0.0, 0.02, 1.0)  # cue pulls the estimate toward 1.0 rad
    assert g.heading == pytest.approx(1.0, abs=1e-3)
    g.reset(3.0)
    g.step(0.0, 0.02, -3.0)  # pulls the short way round the circle
    assert abs(g.heading) > 3.0
    assert g.bump.shape == g.bump_angles.shape and g.confidence == 1.0


@pytest.mark.parametrize("deg_s", [30.0, 90.0, -45.0, -180.0])
def test_gyro_tracks_constant_rotation(deg_s):
    om = piecewise([(0, 0.5), (deg_s, 6.0)], 0.02)
    out = run(GyroIntegrator(), Trajectory("const", 0.02, om, om.copy()))
    assert np.degrees(np.abs(out["error"])).max() < 1e-6


def test_params_are_global_scalars():
    p = load_params("connectome")
    assert set(p["gain"]) == {"EPG", "PEN_a", "PEN_b", "PEG", "Delta7"}
    assert set(p["bias"]) == {"EPG", "PEN_a", "PEN_b", "PEG", "Delta7"}
    assert {"cue_gain", "cue_kappa", "tau", "vel_gain", "vel_amp"} <= set(p)
    assert RingAttractor.from_pack().n_params_fitted == 15


def test_benchmark_json_shipped():
    data = json.loads(results_path().read_text())
    names = [r["model"] for r in data["results"]]
    for n in ("connectome", "rewired", "random", "no_recurrence", "complementary_filter"):
        assert n in names
    for k in KEY_NUMBERS:
        assert k in data["key_numbers"]
        assert set(data["key_numbers"][k]) >= {"connectome", "rewired", "random",
                                               "no_recurrence", "complementary_filter"}


def _held_frac(m, dark_s: float = 30.0, n: int = 12) -> float:
    held = []
    for h in np.linspace(-np.pi, np.pi, n, endpoint=False) + 0.1:
        err, conf = memory_trial(m, float(h), cue_s=3.0, dark_s=(dark_s,))[dark_s]
        held.append(conf > 0.3 and abs(err) < math.radians(30))
    return float(np.mean(held))


def test_connectome_holds_heading_in_darkness(model):
    # The connectome-specific claim: after the cue is removed, recurrence keeps a bump near
    # the last cue (well above the 1/6 expected from a bump at a random place), while the
    # same scalars on rewired / random wiring, or with the wiring removed, do not.
    assert _held_frac(model) >= 0.4
    shipped = load_params("connectome")
    ablated = RingAttractor.from_pack(params={**shipped, "ablate_recurrence": True})
    assert _held_frac(ablated) == 0.0
    for variant in ("rewired", "random"):
        ctrl = [_held_frac(RingAttractor.from_pack(variant=variant, seed=s)) for s in range(2)]
        assert np.mean(ctrl) < 0.2


def test_dead_reckoning_in_darkness_fails(model):
    # Documented failure: turning in the dark is not integrated (see README, test D).
    def held(m):
        out = []
        for h in np.linspace(-np.pi, np.pi, 12, endpoint=False):
            r = memory_trial(m, float(h), cue_s=3.0, dark_s=(4.0,), omega_dark=math.radians(90))
            err, conf = r[4.0]
            out.append(conf > 0.3 and abs(err) < math.radians(30))
        return float(np.mean(out))

    assert held(GyroIntegrator()) == 1.0
    assert held(model) < 0.5
