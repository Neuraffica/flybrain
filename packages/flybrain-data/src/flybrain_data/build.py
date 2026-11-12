"""Build circuit packs from the raw FlyWire v783 files in ``data/``."""

from __future__ import annotations

import datetime as _dt
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .download import ANNOTATIONS_REPO, FILES, ZENODO_RECORD, default_data_dir
from .pack import CircuitPack, pack_path
from .ring import circular_corr, partner_profiles, rank_angles, spectral_ring

# Shiu et al. 2024 convention: acetylcholine excitatory, GABA and glutamate inhibitory.
# The modulatory transmitters (dopamine, serotonin, octopamine) are treated as excitatory, as in
# that model; in this pack only one neuron (a PEN_b predicted serotonergic) is affected.
INHIBITORY_NT = {"gaba", "glutamate"}
NEUROPIL_GROUPS = ("EB", "PB", "NO", "other")  # ellipsoid body, protocerebral bridge, noduli


@dataclass(frozen=True)
class PackSpec:
    name: str
    cell_types: tuple[str, ...]
    min_synapses: int
    description: str
    ring_type: str | None = None
    neuropil_groups: tuple[str, ...] = NEUROPIL_GROUPS
    side_note: str = (
        "soma side from the annotation table; for these types it is the protocerebral-bridge side"
    )


# Looming-escape pack: selection rules (checked and recorded at build time, see escape_stats).
ESCAPE_INPUTS = ("LC4", "LPLC2")
ESCAPE_GF = "DNp01"
# Other looming-responsive descending neurons named in the brief; each receives >= 500
# LC4+LPLC2 synapses in FlyWire (checked at build time). Outputs only: the model reads the GF.
ESCAPE_DNS = ("DNp02", "DNp04", "DNp06", "DNp11")
# Interneurons between the inputs and the GF: every cell type receiving >= 200 synapses from
# LC4+LPLC2 and making >= 50 synapses onto the GF (both summed over all neurons of the type).
ESCAPE_INTERNEURON_RULE = {"min_from_inputs": 200, "min_to_gf": 50}
ESCAPE_INTERNEURONS = (
    "PVLP122b",
    "LHAD1g1",
    "PVLP010",
    "PVLP151",
    "PVLP024",
    "PVLP017",
    "AMMC-A1",
    "AVLP396",
    "CB3513b",
)


SPECS: dict[str, PackSpec] = {
    "central_complex": PackSpec(
        name="central_complex",
        cell_types=("EPG", "PEN_a(PEN1)", "PEN_b(PEN2)", "PEG", "Delta7"),
        min_synapses=5,
        ring_type="EPG",
        description=(
            "Head-direction ring of the Drosophila central complex: EPG (compass/bump "
            "neurons, ellipsoid body -> protocerebral bridge), PEN_a/PEN1 and PEN_b/PEN2 "
            "(angular-velocity shifters, bridge -> ellipsoid body), PEG (bridge -> gall) "
            "and Delta7 (glutamatergic bridge interneurons, ring-wide inhibition)."
        ),
    ),
    "escape": PackSpec(
        name="escape",
        cell_types=(*ESCAPE_INPUTS, ESCAPE_GF, *ESCAPE_DNS, *ESCAPE_INTERNEURONS),
        min_synapses=5,
        neuropil_groups=("PVLP", "PLP", "AVLP", "other"),
        side_note="soma side from the annotation table (LC4/LPLC2: the optic lobe and "
        "visual hemifield they sit in; DNp01: GF of that side, which gets only ipsilateral "
        "LC4/LPLC2 input in FlyWire)",
        description=(
            "Looming-escape pathway: LC4 (looming angular velocity) and LPLC2 (looming "
            "angular size) visual projection neurons of both optic lobes, the giant fiber "
            "descending neuron DNp01 of each side, other looming-responsive descending "
            "neurons DNp02/04/06/11 (outputs only), and the interneuron types connecting "
            "LC4/LPLC2 to the GF (rule: >= 200 synapses from LC4+LPLC2 and >= 50 onto the GF)."
        ),
    ),
}

CITATION = (
    "FlyWire connectome v783, CC BY 4.0. Dorkenwald et al. 2024, Neuronal wiring diagram of "
    "an adult brain, Nature 634:124-138; Schlegel et al. 2024, Whole-brain annotation and "
    "multi-connectome cell typing of Drosophila, Nature 634:139-152. Neurotransmitter "
    "predictions: Eckstein et al. 2024, Cell 187:2574-2594."
)


def _sign_of(nt: str) -> int:
    return -1 if str(nt).lower() in INHIBITORY_NT else 1


def load_annotations(path: Path):
    import pandas as pd

    cols = ["root_id", "cell_type", "side", "top_nt", "hemibrain_type"]
    return pd.read_csv(path, sep="\t", usecols=cols, dtype={"root_id": "int64"}, low_memory=False)


def load_edges(path: Path, root_ids: np.ndarray):
    """Edges among ``root_ids`` from the proofread connections feather, per neuropil."""
    import pyarrow as pa
    import pyarrow.compute as pc
    from pyarrow import feather

    table = feather.read_table(
        path,
        columns=["pre_pt_root_id", "post_pt_root_id", "neuropil", "syn_count"],
        memory_map=True,
    )
    ids = pa.array(root_ids, type=table.schema.field("pre_pt_root_id").type)
    mask = pc.and_(
        pc.is_in(table["pre_pt_root_id"], value_set=ids),
        pc.is_in(table["post_pt_root_id"], value_set=ids),
    )
    return table.filter(mask).to_pandas()


def build_pack(
    name: str = "central_complex",
    data_dir: Path | None = None,
    out: Path | None = None,
    verbose: bool = True,
) -> CircuitPack:
    spec = SPECS[name]
    data_dir = Path(data_dir) if data_dir else default_data_dir()
    ann_path = data_dir / FILES["annotations"].name
    con_path = data_dir / FILES["connections"].name
    for p in (ann_path, con_path):
        if not p.exists():
            raise FileNotFoundError(f"{p} missing; run `uv run flybrain-data download` first")

    ann = load_annotations(ann_path)
    sel = ann[ann.cell_type.isin(spec.cell_types)].copy()
    missing = set(spec.cell_types) - set(sel.cell_type)
    if missing:
        raise ValueError(f"cell types not found in annotations: {sorted(missing)}")
    sel["type_rank"] = sel.cell_type.map({t: i for i, t in enumerate(spec.cell_types)})
    sel = sel.sort_values(["type_rank", "side", "root_id"]).reset_index(drop=True)
    ids = sel.root_id.to_numpy(np.int64)
    pos = {r: i for i, r in enumerate(ids)}
    n = len(ids)

    edges = load_edges(con_path, ids)
    edges["post"] = edges.post_pt_root_id.map(pos)
    edges["pre"] = edges.pre_pt_root_id.map(pos)
    raw = np.zeros((n, n), dtype=np.int64)
    np.add.at(raw, (edges.post.to_numpy(), edges.pre.to_numpy()), edges.syn_count.to_numpy())
    counts = np.where(raw >= spec.min_synapses, raw, 0).astype(np.int32)

    by_np: dict[str, np.ndarray] = {}  # raw (unthresholded) counts per neuropil
    for neuropil, grp in edges.groupby("neuropil"):
        m = np.zeros((n, n), dtype=np.int64)
        np.add.at(m, (grp.post.to_numpy(), grp.pre.to_numpy()), grp.syn_count.to_numpy())
        by_np[str(neuropil)] = m

    nt = sel.top_nt.fillna("unknown").astype(str).to_numpy()
    sign = np.array([_sign_of(x) for x in nt], dtype=np.int8)
    side = sel.side.fillna("unknown").astype(str).to_numpy()
    cell_type = sel.cell_type.astype(str).to_numpy()

    ring_angle = np.full(n, np.nan)
    ring_meta: dict = {}
    if spec.ring_type:
        ring_angle, ring_meta = infer_ring(counts, by_np, cell_type, side, spec.ring_type)

    # Per-neuropil split of the kept pairs, in groups: the CX neuropils, then everything else.
    keep = counts > 0
    groups = [_neuropil_counts(by_np, g, counts.shape) for g in spec.neuropil_groups[:-1]]
    groups.append(raw - sum(groups))
    by_group = np.stack([g * keep for g in groups]).astype(np.int32)
    assert (by_group.sum(0) == counts).all()

    # Order the ring neurons by angle inside their (type, side) group for readability.
    side_rank = (sel.side != "left").to_numpy().astype(int)
    order = np.lexsort(
        (ids, side_rank, np.nan_to_num(ring_angle, nan=99.0), sel.type_rank.to_numpy())
    )
    counts = counts[np.ix_(order, order)]
    by_group = by_group[:, order][:, :, order]
    ids, cell_type, side, nt, sign, ring_angle = (
        a[order] for a in (ids, cell_type, side, nt, sign, ring_angle)
    )

    np_syn = {k: int(v.sum()) for k, v in sorted(by_np.items(), key=lambda kv: -kv[1].sum())}
    meta = {
        "dataset": "FlyWire FAFB proofread connections v783",
        "description": spec.description,
        "cell_types": list(spec.cell_types),
        "weight": (
            "synapse count summed over all neuropils for each (pre, post) pair, "
            "multiplied by the sign of the presynaptic neuron's predicted "
            "neurotransmitter (top_nt): acetylcholine +1, GABA/glutamate -1, "
            "other/unknown +1 (Shiu et al. 2024 convention)"
        ),
        "matrix_convention": "W[post, pre]",
        "min_synapses": spec.min_synapses,
        "threshold_rule": f"pairs with summed synapse count >= {spec.min_synapses} kept",
        "pairs_before_threshold": int(np.count_nonzero(raw)),
        "pairs_after_threshold": int(np.count_nonzero(counts)),
        "synapses_before_threshold": int(raw.sum()),
        "synapses_after_threshold": int(counts.sum()),
        "synapses_by_neuropil": np_syn,
        "side": spec.side_note,
        "ring_angle": ring_meta,
        "built": _dt.date.today().isoformat(),
        "sources": {
            "connections": f"{ZENODO_RECORD} ({FILES['connections'].name}, md5 "
            f"{FILES['connections'].md5})",
            "annotations": f"{ANNOTATIONS_REPO} (supplemental_files/{FILES['annotations'].name})",
        },
        "license": "CC BY 4.0",
        "attribution": CITATION,
    }
    if spec.name == "central_complex":
        io = input_fraction(data_dir, ids, cell_type)
        if io is not None:
            meta["input_fraction_by_neuropil"] = io
    if spec.name == "escape":
        meta["escape"] = escape_stats(
            con_path, ann, counts, raw[np.ix_(order, order)], ids, cell_type, side
        )
    pack = CircuitPack(
        name=name,
        counts=counts,
        sign=sign,
        root_id=ids,
        cell_type=cell_type,
        side=side,
        nt=nt,
        ring_angle=ring_angle,
        metadata=meta,
        by_neuropil=by_group,
        neuropils=spec.neuropil_groups,
    )
    path = pack.save(out or pack_path(name))
    if verbose:
        print(pack.summary())
        if ring_meta:
            print(f"ring angles: {ring_meta.get('method')}")
        if "escape" in meta:
            print("GF input after threshold:", meta["escape"]["gf_input_after_threshold"])
        print(f"saved {path} ({path.stat().st_size / 1024:.1f} KB)")
    return pack


def escape_stats(con_path, ann, counts, raw, ids, cell_type, side) -> dict:
    """Selection evidence (whole-brain counts) and GF input per side for the escape pack."""
    import pyarrow as pa
    import pyarrow.compute as pc
    from pyarrow import feather

    tmap = dict(zip(ann.root_id, ann.cell_type, strict=False))
    src = ann[ann.cell_type.isin(ESCAPE_INPUTS)].root_id.to_numpy()
    gf = ann[ann.cell_type == ESCAPE_GF].root_id.to_numpy()
    t = feather.read_table(
        con_path, columns=["pre_pt_root_id", "post_pt_root_id", "syn_count"], memory_map=True
    )
    frm = t.filter(pc.is_in(t["pre_pt_root_id"], value_set=pa.array(src))).to_pandas()
    to = t.filter(pc.is_in(t["post_pt_root_id"], value_set=pa.array(gf))).to_pandas()
    frm_by = frm.groupby(frm.post_pt_root_id.map(tmap)).syn_count.sum()
    to_by = to.groupby(to.pre_pt_root_id.map(tmap)).syn_count.sum()
    sel = {
        ct: {"from_LC4_LPLC2": int(frm_by.get(ct, 0)), "to_GF": int(to_by.get(ct, 0))}
        for ct in (*ESCAPE_DNS, *ESCAPE_INTERNEURONS)
    }

    def gf_in(m):
        out = {}
        for sd in ("left", "right"):
            g = np.flatnonzero((cell_type == ESCAPE_GF) & (side == sd))
            for st in ESCAPE_INPUTS:
                for ss in ("left", "right"):
                    pre = np.flatnonzero((cell_type == st) & (side == ss))
                    v = int(m[np.ix_(g, pre)].sum())
                    if v:
                        out[f"{st}_{ss}->GF_{sd}"] = v
        return out

    return {
        "inputs": list(ESCAPE_INPUTS),
        "giant_fiber": ESCAPE_GF,
        "descending_neurons": list(ESCAPE_DNS),
        "interneurons": list(ESCAPE_INTERNEURONS),
        "interneuron_rule": ESCAPE_INTERNEURON_RULE,
        "selection_counts_whole_brain": sel,
        "gf_input_before_threshold": gf_in(raw),
        "gf_input_after_threshold": gf_in(counts),
        "note": "DNp01 right is predicted glutamatergic (top_nt) and so signed -1 by the "
        "convention; the GF is known to be cholinergic. Only its outputs inside the pack are "
        "affected; the model reads GF activity, not GF output. GF electrical synapses are "
        "not in the connectome.",
    }


def input_fraction(data_dir: Path, ids: np.ndarray, cell_type: np.ndarray) -> dict | None:
    """Per cell type and neuropil group, the fraction of the type's synapses (with all
    partners in the brain, not just pack members) that are inputs: post / (pre + post).
    Near 1 = dendritic (input) arbor, near 0 = axonic (output) arbor. Needs the optional
    per-neuron neuropil count files; returns None if they are absent."""
    pre_p = data_dir / FILES["neuropil_pre"].name
    post_p = data_dir / FILES["neuropil_post"].name
    if not (pre_p.exists() and post_p.exists()):
        return None
    import pyarrow as pa
    import pyarrow.compute as pc
    from pyarrow import feather

    tmap = dict(zip(ids.tolist(), cell_type.tolist(), strict=True))
    out: dict[str, dict[str, dict[str, float]]] = {}
    tot: dict[tuple[str, str], list[int]] = {}
    for j, (path, col) in enumerate(((pre_p, "pre_pt_root_id"), (post_p, "post_pt_root_id"))):
        t = feather.read_table(path, memory_map=True)
        t = t.filter(pc.is_in(t[col], value_set=pa.array(ids))).to_pandas()
        for rid, npl, cnt in zip(t[col], t["neuropil"], t["count"], strict=True):
            g = next((x for x in NEUROPIL_GROUPS[:-1] if str(npl).upper().startswith(x)), "other")
            tot.setdefault((tmap[rid], g), [0, 0])[j] += int(cnt)
    for (ct, g), (npre, npost) in sorted(tot.items()):
        out.setdefault(ct, {})[g] = {
            "pre": npre,
            "post": npost,
            "input_fraction": round(npost / max(npre + npost, 1), 3),
        }
    return out


def _neuropil_counts(by_np: dict[str, np.ndarray], prefix: str, shape) -> np.ndarray:
    out = np.zeros(shape, dtype=np.int64)
    for k, v in by_np.items():
        if k.upper().startswith(prefix):
            out += v
    return out


def infer_ring(
    counts: np.ndarray,
    by_np: dict[str, np.ndarray],
    cell_type: np.ndarray,
    side: np.ndarray,
    ring_type: str,
) -> tuple[np.ndarray, dict]:
    """Ring angles for ``ring_type`` neurons from connectivity (see ``ring.py``)."""
    rows = np.flatnonzero(cell_type == ring_type)
    phase, diag = spectral_ring(partner_profiles(counts, rows))
    angles = rank_angles(phase)

    # Independent cross-checks: the ring recovered from ellipsoid-body synapses alone and from
    # protocerebral-bridge synapses alone should agree with each other and with the total.
    checks: dict = {}
    for label, prefix in (("EB_only", "EB"), ("PB_only", "PB")):
        sub = _neuropil_counts(by_np, prefix, counts.shape)
        if sub[rows].sum() + sub[:, rows].sum() == 0:
            continue
        ph, d = spectral_ring(partner_profiles(sub, rows))
        checks[label] = {"circular_corr_with_total": round(circular_corr(ph, phase), 3), "eigen": d}
    if "EB_only" in checks and "PB_only" in checks:
        eb = spectral_ring(partner_profiles(_neuropil_counts(by_np, "EB", counts.shape), rows))[0]
        pb = spectral_ring(partner_profiles(_neuropil_counts(by_np, "PB", counts.shape), rows))[0]
        checks["EB_vs_PB_circular_corr"] = round(circular_corr(eb, pb), 3)

    # Anatomy check: left-bridge and right-bridge EPGs each tile the whole ellipsoid body
    # (wedges alternate L/R), so each side's angles should cover the full circle: a small
    # resultant length and no large gap. A ring split by side would give ~0.64 and a 180 deg gap.
    side_cov = {}
    for sd in ("left", "right"):
        a = np.sort(angles[side[rows] == sd])
        if len(a) == 0:
            continue
        gaps = np.diff(np.concatenate([a, a[:1] + 2 * np.pi]))
        side_cov[sd] = {
            "n": len(a),
            "resultant_length": round(float(np.abs(np.exp(1j * a).mean())), 3),
            "max_gap_deg": round(float(np.degrees(gaps.max())), 1),
        }
    out = np.full(len(cell_type), np.nan)
    out[rows] = angles
    meta = {
        "status": "INFERRED from connectivity (no glomerulus/wedge labels in FlyWire v783 "
        "annotations)",
        "method": (
            "spectral circular seriation of EPG partner profiles (sqrt synapse counts, "
            "inputs+outputs, cosine similarity, normalised Laplacian eigenvectors 2-3); "
            "angles = evenly spaced ranks; defined up to rotation and reflection"
        ),
        "eigen": diag,
        "cross_checks": checks,
        "side_coverage": side_cov,
    }
    return out, meta
