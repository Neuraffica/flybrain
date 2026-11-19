import numpy as np
import pytest
from flybrain_data import CircuitPack, available_packs, load_pack, random_like, rewire

COUNTS = {
    "LC10a": (115, 119),
    "AOTU019": (1, 1),
    "AOTU025": (1, 1),
    "DNa02": (1, 1),
    "AOTU015a": (2, 2),
    "CB3127": (3, 3),
    "CB2070": (3, 3),
    "LAL027": (2, 1),
    "TuTuAa": (1, 1),
    "AOTU041": (2, 2),
}


@pytest.fixture(scope="module")
def pack():
    return load_pack("pursuit")


def _syn(pack, pre_t, pre_s, post_t, post_s):
    return int(pack.counts[np.ix_(pack.index(post_t, post_s), pack.index(pre_t, pre_s))].sum())


def test_pursuit_pack_shipped(pack):
    assert "pursuit" in available_packs()
    assert pack.n_neurons == 265
    assert pack.cell_types == list(COUNTS)
    for t, (nl, nr) in COUNTS.items():
        assert len(pack.index(t, "left")) == nl and len(pack.index(t, "right")) == nr
    assert pack.counts[pack.counts > 0].min() >= 5
    assert np.array_equal(pack.by_neuropil.sum(0), pack.counts)
    assert np.isnan(pack.ring_angle).all()
    m = pack.metadata
    assert "CC BY 4.0" in m["license"] and "Dorkenwald" in m["attribution"]
    assert "Matsliah" in m["sources"]["column_assignment"]
    assert m["pursuit"]["neurons_by_type_and_side"]["LC10a"] == {"left": 115, "right": 119}


def test_signs(pack):
    for t, s in (("AOTU019", -1), ("AOTU025", 1), ("DNa02", 1), ("TuTuAa", -1), ("AOTU041", -1)):
        assert (pack.sign[pack.index(t)] == s).all(), t


@pytest.mark.parametrize("side,other", [("left", "right"), ("right", "left")])
def test_ipsilateral_excitation_contralateral_inhibition(pack, side, other):
    # LC10a -> both AOTU types of the same side only
    for t in ("AOTU019", "AOTU025"):
        assert _syn(pack, "LC10a", side, t, side) > 1000
        assert _syn(pack, "LC10a", side, t, other) == 0
    # AOTU025 (excitatory) -> DNa02 same side; AOTU019 (inhibitory) -> DNa02 opposite side
    assert _syn(pack, "AOTU025", side, "DNa02", side) > 100
    assert _syn(pack, "AOTU025", side, "DNa02", other) == 0
    assert _syn(pack, "AOTU019", side, "DNa02", other) > 100
    assert _syn(pack, "AOTU019", side, "DNa02", side) == 0
    W = pack.W
    dn = pack.index("DNa02", side)[0]
    assert W[dn, pack.index("AOTU025", side)[0]] > 0
    assert W[pack.index("DNa02", other)[0], pack.index("AOTU019", side)[0]] < 0


def test_metadata_counts_match_matrix(pack):
    key = pack.metadata["pursuit"]["synapses_by_side_after_threshold"]
    assert key["AOTU025->DNa02"]["left->left"] == _syn(pack, "AOTU025", "left", "DNa02", "left")
    assert key["AOTU019->DNa02"]["left->right"] == _syn(pack, "AOTU019", "left", "DNa02", "right")
    assert key["LC10a->TuTuAa"]["left->left"] > 3000 and key["TuTuAa->LC10a"]["right->right"] > 3000


def test_bridge_rule_holds(pack):
    p = pack.metadata["pursuit"]
    rule = p["bridge_rule"]
    assert set(p["types_passing_bridge_rule"]) == {*p["aotu"], *p["bridges"]}
    for t in (*p["aotu"], *p["bridges"]):
        c = p["selection_counts_whole_brain"][t]
        assert c["from_LC10a"] >= rule["min_from_lc10a"] and c["to_DNa02"] >= rule["min_to_dna02"]


def test_rf_coverage_and_sign(pack):
    az = pack.azimuth
    assert az is not None and az.shape == (pack.n_neurons,)
    lc = pack.index("LC10a")
    others = np.setdiff1d(np.arange(pack.n_neurons), lc)
    assert np.isnan(az[others]).all()
    meta = pack.metadata["azimuth"]
    assert meta["n_with_estimate"] == int(np.isfinite(az[lc]).sum()) == 232
    assert meta["n_without_estimate"] == 2
    left, right = az[pack.index("LC10a", "left")], az[pack.index("LC10a", "right")]
    left, right = left[np.isfinite(left)], right[np.isfinite(right)]
    # left eye looks left (positive), right eye right (negative); both reach just across
    # the midline (binocular overlap), none beyond the assumed 15 deg
    assert np.median(left) > np.radians(30) and np.median(right) < -np.radians(30)
    assert left.min() > -np.radians(15.01) and right.max() < np.radians(15.01)
    assert left.max() > np.radians(120) and right.min() < -np.radians(120)


def test_aotu019_frontal_aotu025_lateral(pack):
    rf = pack.metadata["pursuit"]["lc10a_input_azimuth"]
    for sd, sgn in (("left", 1), ("right", -1)):
        a19 = rf[f"AOTU019_{sd}"]["weighted_mean_azimuth_deg"]
        a25 = rf[f"AOTU025_{sd}"]["weighted_mean_azimuth_deg"]
        assert np.sign(a19) == np.sign(a25) == sgn
        assert 15 < abs(a19) < 50 and 55 < abs(a25) < 100  # published peaks ~26 and ~72 deg
    # recomputed from the shipped matrix and azimuths
    lc = pack.index("LC10a")
    for t in ("AOTU019", "AOTU025"):
        post = pack.index(t, "left")[0]
        w = pack.counts[post, lc].astype(float)
        ok = np.isfinite(pack.azimuth[lc]) & (w > 0)
        mean = np.degrees(np.average(pack.azimuth[lc][ok], weights=w[ok]))
        assert mean == pytest.approx(rf[f"{t}_left"]["weighted_mean_azimuth_deg"], abs=0.06)


def test_input_total(pack):
    tot = pack.input_total
    assert tot is not None and tot.shape == (pack.n_neurons,) and tot.dtype.kind == "i"
    # whole-brain input is at least the (unthresholded) input from inside the pack
    assert (tot >= pack.counts.sum(1)).all() and tot.min() > 0
    meta = pack.metadata["input_total"]
    assert meta["min"] == tot.min() and meta["max"] == tot.max()
    for t in ("AOTU019", "DNa02"):
        assert tot[pack.index(t)].min() > 5000  # large integrating neurons
    frac = pack.counts / tot[:, None]
    assert frac.sum(1).max() <= 1.0


def test_controls_keep_azimuth(pack):
    r = rewire(pack, seed=1)
    assert np.array_equal(r.azimuth, pack.azimuth, equal_nan=True)
    for a in pack.cell_types:
        for b in pack.cell_types:
            blk = np.ix_(pack.index(b), pack.index(a))
            assert pack.counts[blk].sum() == r.counts[blk].sum()
    assert not np.array_equal(r.counts, pack.counts)
    q = random_like(pack, seed=1)
    assert q.n_edges == pack.n_edges and q.counts.sum() == pack.counts.sum()
    assert np.array_equal(q.azimuth, pack.azimuth, equal_nan=True)
    assert np.array_equal(r.input_total, pack.input_total)
    assert np.array_equal(q.input_total, pack.input_total)


def test_old_packs_load_without_azimuth(tmp_path):
    esc = load_pack("escape")
    assert esc.azimuth is None and esc.input_total is None
    path = esc.save(tmp_path / "e.npz")
    assert CircuitPack.from_npz(path).azimuth is None
    assert CircuitPack.from_npz(path).input_total is None
    pk = load_pack("pursuit")
    back = CircuitPack.from_npz(pk.save(tmp_path / "p.npz"))
    assert np.array_equal(back.azimuth, pk.azimuth, equal_nan=True)
    assert np.array_equal(back.input_total, pk.input_total)
    assert "azimuth" in pk.table().columns
