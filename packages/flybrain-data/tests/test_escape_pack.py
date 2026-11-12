import numpy as np
import pytest
from flybrain_data import available_packs, load_pack, random_like, rewire

COUNTS = {
    "LC4": (54, 50),
    "LPLC2": (108, 102),
    "DNp01": (1, 1),
    "DNp02": (1, 1),
    "DNp04": (1, 1),
    "DNp06": (1, 1),
    "DNp11": (1, 1),
    "PVLP122b": (2, 2),
    "LHAD1g1": (1, 1),
    "PVLP010": (1, 1),
    "PVLP151": (2, 2),
    "PVLP024": (2, 2),
    "PVLP017": (1, 1),
    "AMMC-A1": (2, 3),
    "AVLP396": (1, 1),
    "CB3513b": (1, 1),
}


@pytest.fixture(scope="module")
def pack():
    return load_pack("escape")


def test_escape_pack_shipped(pack):
    assert "escape" in available_packs()
    assert pack.n_neurons == 351
    assert pack.cell_types == list(COUNTS)
    for t, (nl, nr) in COUNTS.items():
        assert len(pack.index(t, "left")) == nl and len(pack.index(t, "right")) == nr
    assert pack.counts[pack.counts > 0].min() >= 5
    assert np.array_equal(pack.by_neuropil.sum(0), pack.counts)
    assert np.isnan(pack.ring_angle).all()
    m = pack.metadata
    assert "CC BY 4.0" in m["license"] and "Dorkenwald" in m["attribution"]


def test_gf_inputs_are_ipsilateral(pack):
    gf_in = pack.metadata["escape"]["gf_input_after_threshold"]
    assert set(gf_in) == {
        "LC4_left->GF_left",
        "LPLC2_left->GF_left",
        "LC4_right->GF_right",
        "LPLC2_right->GF_right",
    }
    assert all(v > 250 for v in gf_in.values())


def test_selection_rule_holds(pack):
    esc = pack.metadata["escape"]
    rule = esc["interneuron_rule"]
    for t in esc["interneurons"]:
        c = esc["selection_counts_whole_brain"][t]
        assert c["from_LC4_LPLC2"] >= rule["min_from_inputs"] and c["to_GF"] >= rule["min_to_gf"]
    for t in esc["descending_neurons"]:
        assert esc["selection_counts_whole_brain"][t]["from_LC4_LPLC2"] >= 500


def test_escape_controls(pack):
    r = rewire(pack, seed=1)
    for a in pack.cell_types:
        for b in pack.cell_types:
            blk = np.ix_(pack.index(b), pack.index(a))
            assert pack.counts[blk].sum() == r.counts[blk].sum()
    q = random_like(pack, seed=1)
    assert q.n_edges == pack.n_edges and q.counts.sum() == pack.counts.sum()
    assert np.array_equal(
        load_pack("escape", "random", 3).counts, load_pack("escape", "random", 3).counts
    )
