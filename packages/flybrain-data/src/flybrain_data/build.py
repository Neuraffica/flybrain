"""Build circuit packs from the raw FlyWire v783 files in ``data/``."""

from __future__ import annotations

import datetime as _dt
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .download import ANNOTATIONS_REPO, FILES, ZENODO_RECORD, default_data_dir, local_path
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
    azimuth_type: str | None = None  # cell type that gets a receptive-field azimuth estimate


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

# Small-object pursuit pack (Ribeiro et al. 2018; Hindmarsh Sten et al. 2021; Collie et al.
# 2026): selection rules, checked and recorded at build time (see pursuit_stats).
PURSUIT_INPUT = "LC10a"
PURSUIT_DN = "DNa02"
PURSUIT_AOTU = ("AOTU019", "AOTU025")  # one per side each; GABA / acetylcholine
# Other cell types between LC10a and DNa02: every type receiving >= 500 synapses from LC10a
# and making >= 150 synapses onto DNa02 (summed over all neurons of the type, whole brain,
# edges of >= 5 synapses only). AOTU019 and AOTU025 must pass the same rule.
PURSUIT_BRIDGE_RULE = {"min_from_lc10a": 500, "min_to_dna02": 150, "min_edge_synapses": 5}
PURSUIT_BRIDGES = ("AOTU015a", "CB3127", "CB2070", "LAL027")
# Local inhibition: TuTuAa is reciprocally connected with LC10a; AOTU041 gets LC10a input and
# inhibits AOTU019 and AOTU025.
PURSUIT_LOCAL = ("TuTuAa", "AOTU041")

# LC10a receptive-field azimuth from eye columns (column_assignment.csv.gz, Matsliah et al.
# 2024). Each LC10a's position is the synapse-weighted mean column ``x`` of its column-assigned
# presynaptic neurons (all synapses, no threshold). ``x`` runs -9..8 in both eyes; +x is
# anterior. ASSUMED linear map from x to azimuth, measured toward the eye's own side:
# the most anterior column (x = 8) looks 15 deg across the midline (binocular overlap), the
# most posterior (x = -9) 160 deg to the side. Elevation (``y``) is ignored.
RF_X_FRONT, RF_AZ_FRONT_DEG = 8.0, -15.0
RF_X_BACK, RF_AZ_BACK_DEG = -9.0, 160.0


def column_x_to_azimuth_deg(x, side: str) -> np.ndarray:
    """Eye-column ``x`` to azimuth in degrees, + = fly's left (the assumed linear map)."""
    lateral = RF_AZ_BACK_DEG + (np.asarray(x, float) - RF_X_BACK) * (
        RF_AZ_FRONT_DEG - RF_AZ_BACK_DEG
    ) / (RF_X_FRONT - RF_X_BACK)
    return lateral if side == "left" else -lateral


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
    "pursuit": PackSpec(
        name="pursuit",
        cell_types=(PURSUIT_INPUT, *PURSUIT_AOTU, PURSUIT_DN, *PURSUIT_BRIDGES, *PURSUIT_LOCAL),
        min_synapses=5,
        neuropil_groups=("AOTU", "LAL", "other"),
        azimuth_type=PURSUIT_INPUT,
        side_note="soma side from the annotation table (LC10a: the optic lobe and eye it "
        "sits in, checked against the hemisphere of its column-assigned inputs; DNa02: the "
        "descending neuron of that side)",
        description=(
            "Small-object pursuit pathway: LC10a visual projection neurons of both optic "
            "lobes (small moving objects), AOTU025 (cholinergic, -> DNa02 of the same side) "
            "and AOTU019 (GABAergic, -> DNa02 of the opposite side), the steering descending "
            "neuron DNa02 of each side, the other types bridging LC10a to DNa02 (rule: >= 500 "
            "synapses from LC10a and >= 150 onto DNa02), and the local inhibitory types "
            "TuTuAa (reciprocal with LC10a) and AOTU041 (LC10a -> AOTU041 -| AOTU019/025)."
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

    col_path = local_path("column_assignment", data_dir)
    if spec.azimuth_type and not col_path.exists():
        raise FileNotFoundError(
            f"{col_path} missing; run `uv run flybrain-data download --only column_assignment`"
        )

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
    azimuth, az_meta = None, {}
    if spec.azimuth_type:
        azimuth, az_meta = infer_azimuth(con_path, col_path, ids, cell_type, side, spec)

    # Per-neuropil split of the kept pairs, in groups: the CX neuropils, then everything else.
    keep = counts > 0
    groups = [_neuropil_counts(by_np, g, counts.shape) for g in spec.neuropil_groups[:-1]]
    groups.append(raw - sum(groups))
    by_group = np.stack([g * keep for g in groups]).astype(np.int32)
    assert (by_group.sum(0) == counts).all()

    # Order the ring neurons by angle (visual neurons by azimuth) inside their (type, side)
    # group for readability.
    side_rank = (sel.side != "left").to_numpy().astype(int)
    angle_key = ring_angle if azimuth is None else np.where(np.isnan(ring_angle), azimuth, 0.0)
    order = np.lexsort(
        (ids, np.nan_to_num(angle_key, nan=99.0), side_rank, sel.type_rank.to_numpy())
    )
    counts = counts[np.ix_(order, order)]
    by_group = by_group[:, order][:, :, order]
    ids, cell_type, side, nt, sign, ring_angle = (
        a[order] for a in (ids, cell_type, side, nt, sign, ring_angle)
    )
    if azimuth is not None:
        azimuth = azimuth[order]

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
    input_total = None
    if spec.name == "pursuit":
        input_total = whole_brain_input_total(con_path, ids)
        meta["input_total"] = {
            "what": "total input synapses of each neuron in the whole brain: syn_count summed "
            "over all presynaptic partners and neuropils in proofread_connections_783, no "
            "threshold. counts[post, pre] / input_total[post] is the fraction of post's input "
            "that comes from pre (used by the pursuit model's fractional-input weights)",
            "min": int(input_total.min()),
            "median": float(np.median(input_total)),
            "max": int(input_total.max()),
            "fraction_of_input_inside_pack_median": round(
                float(np.median(raw[np.ix_(order, order)].sum(1) / input_total)), 4
            ),
            "by_type_mean": {
                ct: round(float(input_total[cell_type == ct].mean()), 1)
                for ct in dict.fromkeys(cell_type.tolist())
            },
        }
        meta["azimuth"] = az_meta
        meta["sources"]["column_assignment"] = (
            f"{FILES['column_assignment'].url} (md5 {FILES['column_assignment'].md5}); "
            "Matsliah et al. 2024, Neuronal parts list and wiring diagram for a visual system, "
            "Nature 634:166-180, CC BY 4.0"
        )
        meta["pursuit"] = pursuit_stats(
            con_path, ann, counts, raw[np.ix_(order, order)], ids, cell_type, side, nt, azimuth
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
        azimuth=azimuth,
        input_total=input_total,
    )
    path = pack.save(out or pack_path(name))
    if verbose:
        print(pack.summary())
        if ring_meta:
            print(f"ring angles: {ring_meta.get('method')}")
        if "escape" in meta:
            print("GF input after threshold:", meta["escape"]["gf_input_after_threshold"])
        if "pursuit" in meta:
            print(json.dumps(meta["pursuit"], indent=1))
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


def _pair_edges(t, col: str, ids: np.ndarray, min_syn: int):
    """Rows of the connections table with ``col`` in ``ids``, summed over neuropils per
    (pre, post) pair, keeping pairs with >= ``min_syn`` synapses."""
    import pyarrow as pa
    import pyarrow.compute as pc

    df = t.filter(pc.is_in(t[col], value_set=pa.array(ids))).to_pandas()
    df = df.groupby(["pre_pt_root_id", "post_pt_root_id"], as_index=False).syn_count.sum()
    return df[df.syn_count >= min_syn]


def whole_brain_input_total(con_path, ids: np.ndarray) -> np.ndarray:
    """Total input synapses of each neuron in ``ids`` (all partners, all neuropils)."""
    from pyarrow import feather

    t = feather.read_table(con_path, columns=["post_pt_root_id", "syn_count"], memory_map=True)
    df = _pair_edges_post(t, ids)
    tot = df.groupby("post_pt_root_id").syn_count.sum()
    return tot.reindex(ids).fillna(0).to_numpy(np.int64)


def _pair_edges_post(t, ids):
    import pyarrow as pa
    import pyarrow.compute as pc

    return t.filter(pc.is_in(t["post_pt_root_id"], value_set=pa.array(ids))).to_pandas()


def infer_azimuth(con_path, col_path, ids, cell_type, side, spec: PackSpec):
    """Receptive-field azimuth (radians, + = left) of every ``spec.azimuth_type`` neuron from
    the eye columns of its presynaptic partners; NaN elsewhere and where no input has a
    column. See RF_* above for the assumed column -> azimuth map."""
    import pandas as pd
    from pyarrow import feather

    rows = np.flatnonzero(cell_type == spec.azimuth_type)
    t = feather.read_table(
        con_path, columns=["pre_pt_root_id", "post_pt_root_id", "syn_count"], memory_map=True
    )
    inp = _pair_edges(t, "post_pt_root_id", ids[rows], 1)  # all synapses, no threshold
    col = pd.read_csv(col_path, usecols=["root_id", "hemisphere", "type", "x"])
    col = col.drop_duplicates("root_id").set_index("root_id")
    inp = inp[inp.pre_pt_root_id.isin(col.index)].copy()
    inp["x"] = inp.pre_pt_root_id.map(col.x).astype(float)
    inp["hemi"] = inp.pre_pt_root_id.map(col.hemisphere)
    inp["ctype"] = inp.pre_pt_root_id.map(col["type"])
    inp["side"] = inp.post_pt_root_id.map(dict(zip(ids[rows], side[rows], strict=True)))
    hemi_ok = int(inp.syn_count[inp.hemi == inp.side].sum())
    hemi_all = int(inp.syn_count.sum())
    if hemi_ok != hemi_all:
        raise ValueError(
            f"{hemi_all - hemi_ok} column-assigned synapses onto {spec.azimuth_type} come from "
            "the other eye; the per-side azimuth sign would be wrong"
        )
    g = inp.groupby("post_pt_root_id")
    xs = g.apply(lambda d: float(np.average(d.x, weights=d.syn_count)), include_groups=False)
    nsyn = g.syn_count.sum()
    pos = {r: i for i, r in enumerate(ids)}
    out = np.full(len(ids), np.nan)
    col_x = np.full(len(ids), np.nan)
    for rid, x in xs.items():
        i = pos[rid]
        col_x[i] = x
        out[i] = np.radians(column_x_to_azimuth_deg(x, str(side[i])))
    have = np.isfinite(out[rows])
    per_side = {}
    for sd in ("left", "right"):
        r = rows[side[rows] == sd]
        a = np.degrees(out[r][np.isfinite(out[r])])
        per_side[sd] = {
            "n": len(r),
            "with_estimate": len(a),
            "azimuth_deg_min": round(float(a.min()), 1),
            "azimuth_deg_median": round(float(np.median(a)), 1),
            "azimuth_deg_max": round(float(a.max()), 1),
            "column_x_min": round(float(np.nanmin(col_x[r])), 2),
            "column_x_max": round(float(np.nanmax(col_x[r])), 2),
        }
    by_type = inp.groupby("ctype").syn_count.sum().sort_values(ascending=False)
    meta = {
        "cell_type": spec.azimuth_type,
        "status": "ESTIMATED from the eye columns of each neuron's presynaptic partners; the "
        "column -> azimuth map is ASSUMED (linear), not measured",
        "method": "synapse-weighted mean column x of the column-assigned presynaptic neurons "
        "(column_assignment.csv.gz; all synapses, no threshold), then azimuth_deg = "
        f"{RF_AZ_BACK_DEG:g} + (x - ({RF_X_BACK:g})) * ({RF_AZ_FRONT_DEG:g} - {RF_AZ_BACK_DEG:g}) "
        f"/ ({RF_X_FRONT:g} - ({RF_X_BACK:g})) toward the eye's own side "
        "(left eye positive, right eye negative); elevation (y) ignored",
        "column_x_anterior": "+x is anterior (frontal); x = 8 most anterior, x = -9 most posterior",
        "map_endpoints_deg": {
            "x=8": f"{-RF_AZ_FRONT_DEG:g} deg across the midline",
            "x=-9": f"{RF_AZ_BACK_DEG:g} deg to the eye's side",
        },
        "convention": "radians, positive = fly's left (counter-clockwise), 0 = straight ahead",
        "n_neurons": len(rows),
        "n_with_estimate": int(have.sum()),
        "n_without_estimate": int((~have).sum()),
        "fraction_with_estimate": round(float(have.mean()), 4),
        "median_column_synapses_per_neuron": float(np.median(nsyn)),
        "column_synapses_by_presynaptic_type": {str(k): int(v) for k, v in by_type.head(8).items()},
        "column_synapses_same_hemisphere": f"{hemi_ok} of {hemi_all}",
        "per_side": per_side,
    }
    return out, meta


def pursuit_stats(con_path, ann, counts, raw, ids, cell_type, side, nt, azimuth) -> dict:
    """Selection evidence (whole-brain counts), side-resolved key connections, and the check
    that AOTU019 samples the frontal and AOTU025 the lateral visual field."""
    from pyarrow import feather

    tmap = dict(zip(ann.root_id, ann.cell_type, strict=False))
    t = feather.read_table(
        con_path, columns=["pre_pt_root_id", "post_pt_root_id", "syn_count"], memory_map=True
    )
    k = PURSUIT_BRIDGE_RULE["min_edge_synapses"]
    lc = ann[ann.cell_type == PURSUIT_INPUT].root_id.to_numpy()
    dn = ann[ann.cell_type == PURSUIT_DN].root_id.to_numpy()
    frm = _pair_edges(t, "pre_pt_root_id", lc, k)
    to = _pair_edges(t, "post_pt_root_id", dn, k)
    frm_by = frm.groupby(frm.post_pt_root_id.map(tmap)).syn_count.sum()
    to_by = to.groupby(to.pre_pt_root_id.map(tmap)).syn_count.sum()
    rule = PURSUIT_BRIDGE_RULE
    passing = sorted(
        str(ct)
        for ct in set(frm_by.index) & set(to_by.index)
        if ct not in (PURSUIT_INPUT, PURSUIT_DN)
        and frm_by[ct] >= rule["min_from_lc10a"]
        and to_by[ct] >= rule["min_to_dna02"]
    )
    for ct in PURSUIT_AOTU:
        if ct not in passing:
            raise ValueError(f"{ct} fails the bridging rule {rule}; the pack spec is wrong")
    if set(passing) != {*PURSUIT_AOTU, *PURSUIT_BRIDGES}:
        raise ValueError(f"bridging rule passes {passing}, spec lists {PURSUIT_BRIDGES}")
    near = {  # the closest misses, for the record
        str(ct): {"from_LC10a": int(frm_by[ct]), "to_DNa02": int(to_by[ct])}
        for ct in set(frm_by.index) & set(to_by.index)
        if ct not in passing
        and ct not in (PURSUIT_INPUT, PURSUIT_DN)
        and frm_by[ct] >= rule["min_from_lc10a"] / 2
        and to_by[ct] >= rule["min_to_dna02"] / 2
    }
    sel_counts = {
        ct: {"from_LC10a": int(frm_by.get(ct, 0)), "to_DNa02": int(to_by.get(ct, 0))}
        for ct in (*PURSUIT_AOTU, *PURSUIT_BRIDGES, *PURSUIT_LOCAL)
    }

    def by_side(m, pre_t, post_t):
        out = {}
        for ps in ("left", "right"):
            pre = np.flatnonzero((cell_type == pre_t) & (side == ps))
            for qs in ("left", "right"):
                post = np.flatnonzero((cell_type == post_t) & (side == qs))
                out[f"{ps}->{qs}"] = int(m[np.ix_(post, pre)].sum())
        return out

    pairs = [
        ("LC10a", "AOTU019"),
        ("LC10a", "AOTU025"),
        ("AOTU019", "DNa02"),
        ("AOTU025", "DNa02"),
        ("LC10a", "TuTuAa"),
        ("TuTuAa", "LC10a"),
        ("LC10a", "AOTU041"),
        ("AOTU041", "AOTU019"),
        ("AOTU041", "AOTU025"),
        ("LC10a", "DNa02"),
    ]
    key_after = {f"{a}->{b}": by_side(counts, a, b) for a, b in pairs}
    key_before = {f"{a}->{b}": by_side(raw, a, b) for a, b in pairs}

    # Where in the visual field each AOTU neuron's LC10a input comes from: synapse-weighted
    # mean azimuth of its presynaptic LC10a (pack counts, after threshold).
    lc_rows = np.flatnonzero(cell_type == PURSUIT_INPUT)
    rf_check = {}
    for ct in PURSUIT_AOTU:
        for sd in ("left", "right"):
            post = np.flatnonzero((cell_type == ct) & (side == sd))[0]
            w = counts[post, lc_rows].astype(float)
            a = azimuth[lc_rows]
            ok = np.isfinite(a) & (w > 0)
            mean = float(np.degrees(np.average(a[ok], weights=w[ok])))
            lo, hi = np.percentile(np.degrees(a[ok]), [10, 90])
            rf_check[f"{ct}_{sd}"] = {
                "lc10a_inputs": int(ok.sum()),
                "synapses": int(w[ok].sum()),
                "weighted_mean_azimuth_deg": round(mean, 1),
                "azimuth_deg_p10_p90": [round(float(lo), 1), round(float(hi), 1)],
            }
    for sd in ("left", "right"):
        a19 = abs(rf_check[f"AOTU019_{sd}"]["weighted_mean_azimuth_deg"])
        a25 = abs(rf_check[f"AOTU025_{sd}"]["weighted_mean_azimuth_deg"])
        if not a19 < a25:
            raise ValueError(f"AOTU019 {sd} input is not more frontal than AOTU025's")

    counts_by_type = {
        ct: {sd: int(((cell_type == ct) & (side == sd)).sum()) for sd in ("left", "right")}
        for ct in dict.fromkeys(cell_type.tolist())
    }
    nt_by_type = {}
    for ct in dict.fromkeys(cell_type.tolist()):
        vals, n = np.unique(nt[cell_type == ct], return_counts=True)
        nt_by_type[ct] = {str(v): int(c) for v, c in zip(vals, n, strict=True)}
    n_lc_inh = int(sum(_sign_of(x) < 0 for x in nt[lc_rows]))
    return {
        "input": PURSUIT_INPUT,
        "descending_neuron": PURSUIT_DN,
        "aotu": list(PURSUIT_AOTU),
        "bridges": list(PURSUIT_BRIDGES),
        "local_inhibition": list(PURSUIT_LOCAL),
        "bridge_rule": rule,
        "types_passing_bridge_rule": passing,
        "near_misses_of_bridge_rule": dict(sorted(near.items())),
        "selection_counts_whole_brain": sel_counts,
        "neurons_by_type_and_side": counts_by_type,
        "neurotransmitter_by_type": nt_by_type,
        "synapses_by_side_after_threshold": key_after,
        "synapses_by_side_before_threshold": key_before,
        "lc10a_input_azimuth": rf_check,
        "frontal_lateral_check": "passed: |mean azimuth| of LC10a input is smaller for "
        "AOTU019 than for AOTU025 on both sides",
        "note": f"{n_lc_inh} of {len(lc_rows)} LC10a are predicted GABA/glutamatergic (top_nt) "
        "and so signed -1 by the convention; LC10a are generally taken to be cholinergic. "
        f"TuTuAa predicted transmitters {nt_by_type.get('TuTuAa')}: inhibitory under the "
        "convention either way. Gap junctions are not in the connectome.",
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
