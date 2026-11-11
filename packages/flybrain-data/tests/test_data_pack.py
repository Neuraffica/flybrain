import json

import numpy as np
import pytest
from flybrain_data import CircuitPack, available_packs, load_pack, random_like, rewire

TYPES = ["EPG", "PEN_a(PEN1)", "PEN_b(PEN2)", "PEG", "Delta7"]


@pytest.fixture(scope="module")
def pack():
    return load_pack("central_complex")


def block_sums(p: CircuitPack) -> dict:
    return {
        (a, b): int(p.counts[np.ix_(p.index(b), p.index(a))].sum())
        for a in p.cell_types
        for b in p.cell_types
    }


def test_pack_shipped_and_shapes(pack):
    assert "central_complex" in available_packs()
    n = pack.n_neurons
    assert n == 151
    assert pack.counts.shape == (n, n) and pack.counts.dtype == np.int32
    assert pack.W.shape == (n, n) and pack.W.dtype == np.float32
    for arr in (pack.sign, pack.root_id, pack.cell_type, pack.side, pack.nt, pack.ring_angle):
        assert arr.shape == (n,)
    assert pack.root_id.dtype == np.int64 and len(set(pack.root_id)) == n
    assert pack.cell_types == TYPES
    assert set(pack.side) <= {"left", "right"}
    assert np.diag(pack.counts).sum() == 0


def test_counts_types_and_threshold(pack):
    counts = {t: len(pack.index(t)) for t in TYPES}
    assert counts == {"EPG": 47, "PEN_a(PEN1)": 20, "PEN_b(PEN2)": 22, "PEG": 20, "Delta7": 42}
    nz = pack.counts[pack.counts > 0]
    assert nz.min() >= pack.metadata["min_synapses"] == 5


def test_signs_follow_neurotransmitter(pack):
    inhib = np.isin(pack.nt, ["gaba", "glutamate"])
    assert np.all(pack.sign[inhib] == -1) and np.all(pack.sign[~inhib] == 1)
    W = pack.W
    # Dale's law: each presynaptic column has a single sign
    for j in range(pack.n_neurons):
        col = W[:, j][W[:, j] != 0]
        assert np.all(np.sign(col) == pack.sign[j])
    # Delta7 are glutamatergic (inhibitory) apart from at most one mispredicted neuron
    assert (pack.sign[pack.index("Delta7")] == -1).sum() >= 41


def test_ring_angles(pack):
    epg = pack.index("EPG")
    ang = pack.ring_angle[epg]
    assert np.all(np.isfinite(ang)) and np.all((ang >= 0) & (ang < 2 * np.pi))
    assert np.isnan(pack.ring_angle[pack.index("Delta7")]).all()
    assert len(np.unique(np.round(ang, 9))) == len(epg)
    meta = pack.metadata["ring_angle"]
    assert "INFERRED" in meta["status"]
    # bridge-only and ellipsoid-body-only connectivity give the same ring
    assert meta["cross_checks"]["EB_vs_PB_circular_corr"] > 0.9


def test_neuropil_split_sums(pack):
    assert pack.neuropils == ("EB", "PB", "NO", "other")
    assert np.array_equal(pack.by_neuropil.sum(0), pack.counts)


def test_metadata_attribution(pack):
    m = pack.metadata
    assert "CC BY 4.0" in m["license"]
    assert "Dorkenwald" in m["attribution"] and "Schlegel" in m["attribution"]
    assert "zenodo.org/records/10676866" in m["sources"]["connections"]
    json.dumps(m)


def test_npz_roundtrip(pack, tmp_path):
    path = pack.save(tmp_path / "x.npz")
    q = CircuitPack.from_npz(path)
    assert np.array_equal(q.counts, pack.counts) and np.array_equal(q.sign, pack.sign)
    assert np.array_equal(q.by_neuropil, pack.by_neuropil)
    assert np.allclose(q.ring_angle, pack.ring_angle, equal_nan=True)


def test_rewired_preserves_blocks_and_degrees(pack):
    r = rewire(pack, seed=3)
    assert r.variant == "rewired"
    assert r.n_edges == pack.n_edges and r.counts.sum() == pack.counts.sum()
    assert block_sums(r) == block_sums(pack)
    assert np.diag(r.counts).sum() == 0
    for a in pack.cell_types:
        for b in pack.cell_types:
            pre, post = pack.index(a), pack.index(b)
            B0 = pack.counts[np.ix_(post, pre)] > 0
            B1 = r.counts[np.ix_(post, pre)] > 0
            assert np.array_equal(B0.sum(0), B1.sum(0))  # out-degree per pre neuron
            assert np.array_equal(B0.sum(1), B1.sum(1))  # in-degree per post neuron
    # each presynaptic neuron keeps its own synapse counts
    for j in range(pack.n_neurons):
        assert sorted(pack.counts[:, j][pack.counts[:, j] > 0]) == sorted(
            r.counts[:, j][r.counts[:, j] > 0]
        )
    assert np.array_equal(r.by_neuropil.sum(0), r.counts)
    moved = (r.counts > 0) & (pack.counts == 0)
    assert moved.sum() > 0.3 * pack.n_edges  # wiring actually scrambled


def test_random_preserves_size_density_signs(pack):
    q = random_like(pack, seed=3)
    assert q.n_edges == pack.n_edges and q.counts.sum() == pack.counts.sum()
    assert np.array_equal(q.sign, pack.sign)
    for s in (1, -1):
        cols = pack.sign == s
        assert (q.counts[:, cols] > 0).sum() == (pack.counts[:, cols] > 0).sum()
    assert sorted(q.counts[q.counts > 0]) == sorted(pack.counts[pack.counts > 0])
    assert np.diag(q.counts).sum() == 0
    assert np.array_equal(q.by_neuropil.sum(0), q.counts)


@pytest.mark.parametrize("variant", ["rewired", "random"])
def test_controls_deterministic(variant):
    a = load_pack("central_complex", variant=variant, seed=7)
    b = load_pack("central_complex", variant=variant, seed=7)
    c = load_pack("central_complex", variant=variant, seed=8)
    assert np.array_equal(a.counts, b.counts)
    assert not np.array_equal(a.counts, c.counts)


def test_unknown_variant():
    with pytest.raises(ValueError):
        load_pack("central_complex", variant="shuffled")
