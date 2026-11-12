"""Compact C layout of the fly escape model (the default since 2026-11-12).

Lossless with respect to the model's decisions; it only changes how the same numbers are
stored:

* **Synapse counts instead of float weights.** Every weight is ``count * scale(pre)`` where
  ``count`` is the FlyWire synapse count (an integer) and ``scale(pre) = sign * gain /
  w_scale`` depends only on the presynaptic neuron's class. Counts are stored as ``uint8``;
  the few pairs with more than 255 synapses are split into several entries of the same row
  and column, whose sum is the same count.
* **Neuron classes instead of per-neuron floats.** Each neuron stores a ``uint8`` class index
  into a small table of (presynaptic scale, bias).
* **Index ranges instead of index lists** for the four input populations, which are contiguous
  in the pack (LC4 left/right, LPLC2 left/right), and no ``drive_in`` scratch array.
* **Neurons that cannot influence the readout are dropped** (``prune=True``): the readout is
  the two giant fibers and the mean LC4/LPLC2 activity per side, so a neuron with no directed
  path to any of those cannot change a trigger or a side. For the shipped pack these are three
  descending/interneurons (DNp02, DNp11 and one AMMC-A1).
* The looming-rate baseline is compiled only with ``#define FE_WITH_LOOMING 1``.
"""

from __future__ import annotations

import datetime as _dt
from dataclasses import dataclass, field

import numpy as np

from .escape_c import HEADER, _c_array, _f32


@dataclass
class CompactModel:
    n: int
    row_ptr: np.ndarray
    col: np.ndarray
    cnt: np.ndarray
    cls: np.ndarray
    cls_scale: np.ndarray
    cls_bias: np.ndarray
    seg: tuple[int, int, int, int, int]  # starts of LC4_L, LC4_R, LPLC2_L, LPLC2_R, end of VPNs
    gf: tuple[int, int]
    scalars: dict
    kept: np.ndarray  # indices into the full pack
    meta: dict = field(default_factory=dict)


def readout_closure(W: np.ndarray, targets) -> np.ndarray:
    """Neurons with a directed path (W[post, pre] != 0) to any target, plus the targets."""
    keep = {int(t) for t in targets}
    frontier = list(keep)
    while frontier:
        i = frontier.pop()
        for j in np.flatnonzero(W[i]):
            if int(j) not in keep:
                keep.add(int(j))
                frontier.append(int(j))
    return np.array(sorted(keep))


def build_compact(params_key: str = "connectome", prune: bool = True) -> CompactModel:
    from flybrain_models import escape as esc

    m = esc.EscapeCircuit.from_pack("escape", params=params_key)
    W, counts = m._W, m.pack.counts.astype(np.int64)
    n_full = W.shape[0]
    pops = [m._lc4["L"], m._lc4["R"], m._lplc2["L"], m._lplc2["R"]]
    targets = np.concatenate([*pops, m._gf])
    kept = readout_closure(W, targets) if prune else np.arange(n_full)
    remap = -np.ones(n_full, dtype=np.int64)
    remap[kept] = np.arange(len(kept))

    # per-presynaptic scale: W[i, j] / count[i, j], the same for every i of a column up to
    # float rounding; rounded to 12 significant digits to group neurons into classes
    scale = np.zeros(n_full)
    for j in range(n_full):
        nz = np.flatnonzero(counts[:, j])
        if len(nz):
            sj = W[nz, j] / counts[nz, j]
            assert np.allclose(sj, sj[0], rtol=1e-12, atol=0), j
            scale[j] = sj[0]

    def sig12(x):
        return np.array([float(f"{v:.12g}") for v in x])

    pairs = np.stack([sig12(scale[kept]), sig12(m._bias[kept])], 1)
    uniq, cls = np.unique(pairs, axis=0, return_inverse=True)
    cls = cls.ravel()
    assert len(uniq) < 256

    rows, cols, cnts = [], [], []
    for new_i, i in enumerate(kept):
        for j in np.flatnonzero(W[i]):
            if remap[j] < 0:
                continue
            c = int(counts[i, j])
            while c > 0:  # split counts above 255 into several entries
                rows.append(new_i)
                cols.append(int(remap[j]))
                cnts.append(min(c, 255))
                c -= 255
    rows, cols, cnts = map(np.array, (rows, cols, cnts))
    n = len(kept)
    row_ptr = np.zeros(n + 1, dtype=np.int64)
    np.add.at(row_ptr, rows + 1, 1)
    row_ptr = np.cumsum(row_ptr)

    # check the stored form reproduces W on the kept neurons
    Wc = np.zeros((n, n))
    np.add.at(Wc, (rows, cols), cnts * uniq[cls[cols], 0])
    assert np.allclose(Wc, W[np.ix_(kept, kept)], rtol=1e-9, atol=1e-12)

    newpops = [remap[p] for p in pops]
    starts = [int(p[0]) for p in newpops]
    for p in newpops:
        assert np.array_equal(p, np.arange(p[0], p[0] + len(p))), "population not contiguous"
    assert starts == sorted(starts) and starts[0] == 0
    assert all(newpops[k][-1] + 1 == starts[k + 1] for k in range(3))
    seg = (*starts, int(newpops[3][-1]) + 1)
    p = m.params
    return CompactModel(
        n=n,
        row_ptr=row_ptr.astype(np.uint16),
        col=cols.astype(np.uint16),
        cnt=cnts.astype(np.uint8),
        cls=cls.astype(np.uint8),
        cls_scale=uniq[:, 0].astype(np.float32),
        cls_bias=uniq[:, 1].astype(np.float32),
        seg=seg,
        gf=(int(remap[m._gf[0]]), int(remap[m._gf[1]])),
        scalars={
            "tau": m.tau,
            "k_in": m.k_in,
            "dt_sub": m.dt_sub,
            "th_l": m.gf_thresholds[0],
            "th_r": m.gf_thresholds[1],
            "refractory_s": float(p.get("refractory_s", esc.REFRACTORY_S)),
            "side_margin": esc.SIDE_MARGIN,
            "v_ref": esc.V_REF,
            "deadband": esc.DEADBAND_RAW,
            "baseline": esc.BASELINE_RAW,
            "loom_threshold": float(esc.load_params("looming_rate_threshold")["threshold"]),
        },
        kept=kept,
        meta={
            "params_key": params_key,
            "date": _dt.date.today().isoformat(),
            "pack": "escape (FlyWire v783, CC BY 4.0)",
            "dropped_neurons": [
                str(m.pack.cell_type[i]) for i in sorted(set(range(n_full)) - set(kept.tolist()))
            ],
            "n_full": n_full,
        },
    )


SOURCE_COMPACT = """\
/* {fname} - generated by flybrain-export ({date}); do not edit by hand.
 * Model: flybrain_models.escape.EscapeCircuit, params "{params_key}", pack {pack}.
 * Compact layout: {n} of {n_full} neurons (dropped, no path to the readout: {dropped}),
 * {nnz} synapse entries as uint8 counts x per-class float scale ({n_cls} classes).
 */
{include}
#include <math.h>
#include <stddef.h>

#ifndef FE_WITH_LOOMING
#define FE_WITH_LOOMING 0 /* 1 also compiles the looming-rate baseline (FE_MODEL_LOOMING) */
#endif

#define FE_NNZ {nnz}
#define FE_N_CLASSES {n_cls}
static const float FE_TAU = {tau};
static const float FE_K_IN = {k_in};
static const float FE_DT_SUB = {dt_sub};
static const float FE_TH_L = {th_l};
static const float FE_TH_R = {th_r};
static const float FE_REFRACTORY_S = {refractory_s};
static const float FE_SIDE_MARGIN = {side_margin};
static const float FE_V_REF = {v_ref};
static const float FE_DEADBAND = {deadband};
static const float FE_DEFAULT_BASELINE = {baseline};
#if FE_WITH_LOOMING
static const float FE_LOOM_THRESHOLD = {loom_threshold};
#endif
static const uint16_t FE_GF_L = {gf_l};
static const uint16_t FE_GF_R = {gf_r};
/* contiguous input populations: [FE_SEG[k], FE_SEG[k+1]) = LC4 L, LC4 R, LPLC2 L, LPLC2 R */
static const uint16_t FE_SEG[5] = {{{seg}}};

{arrays}
FE_API float fe_raw_to_proximity(float raw, float baseline) {{
  float top = baseline - FE_DEADBAND;
  float p;
  if (top <= 0.0f) return 0.0f;
  p = (top - raw) / top;
  if (p < 0.0f) p = 0.0f;
  if (p > 1.0f) p = 1.0f;
  return p;
}}

FE_API void fe_reset(fe_state_t *s) {{
  uint16_t i;
  for (i = 0; i < FE_N_NEURONS; i++) s->r[i] = 0.0f;
  s->prev_l = s->prev_r = 0.0f;
  s->has_prev = 0;
  s->above = 0;
  s->since_trigger_s = 1.0e9f;
  s->gf_l = s->gf_r = 0.0f;
  s->drive = 0.0f;
}}

FE_API void fe_init(fe_state_t *s, uint8_t model) {{
  s->model = model;
  s->baseline_l = s->baseline_r = FE_DEFAULT_BASELINE;
  fe_reset(s);
}}

FE_API void fe_set_baseline(fe_state_t *s, float baseline_l, float baseline_r) {{
  s->baseline_l = baseline_l;
  s->baseline_r = baseline_r;
}}

FE_API void fe_calibrate(fe_state_t *s, const uint16_t *raw_l, const uint16_t *raw_r, uint16_t n) {{
  uint32_t sl = 0, sr = 0;
  uint16_t i;
  if (n == 0) return;
  for (i = 0; i < n; i++) {{
    sl += raw_l[i];
    sr += raw_r[i];
  }}
  s->baseline_l = (float)sl / (float)n;
  s->baseline_r = (float)sr / (float)n;
}}

static int8_t fe_side(float left, float right) {{
  float big = left > right ? left : right;
  float diff = left - right;
  if (diff < 0.0f) diff = -diff;
  if (big <= 0.0f || diff <= FE_SIDE_MARGIN * big) return 0;
  return left > right ? 1 : -1;
}}

static uint8_t fe_trigger(fe_state_t *s, float dt) {{
  uint8_t above = s->drive >= 1.0f;
  uint8_t fire;
  s->since_trigger_s += dt;
  fire = above && !s->above && s->since_trigger_s >= FE_REFRACTORY_S;
  s->above = above;
  if (fire) s->since_trigger_s = 0.0f;
  return fire;
}}

static float fe_mean(const float *r, uint16_t a, uint16_t b) {{
  float acc = 0.0f;
  uint16_t i;
  for (i = a; i < b; i++) acc += r[i];
  return acc / (float)(b - a);
}}

/* One exponential-Euler sub-step: r += a * (tanh(max(0, W r + bias + input)) - r).
 * ext[k] is the external input of input population k (0 for the rest). */
static void fe_substep(float *r, const float *ext, float a, float *u) {{
  uint16_t i, k, seg = 0;
  for (i = 0; i < FE_N_NEURONS; i++) {{
    float acc;
    while (seg < 4 && i >= FE_SEG[seg + 1]) seg++;
    acc = FE_CLASS_BIAS[FE_CLASS[i]] + (seg < 4 && i >= FE_SEG[seg] ? ext[seg] : 0.0f);
    for (k = FE_ROW_PTR[i]; k < FE_ROW_PTR[i + 1]; k++) {{
      uint16_t j = FE_COL[k];
      acc += ((float)FE_CNT[k] * FE_CLASS_SCALE[FE_CLASS[j]]) * r[j];
    }}
    u[i] = acc;
  }}
  for (i = 0; i < FE_N_NEURONS; i++) {{
    float t = u[i] > 0.0f ? tanhf(u[i]) : 0.0f; /* tanh(0) == 0: skip the call */
    r[i] += a * (t - r[i]);
  }}
}}

FE_API fe_result_t fe_step(fe_state_t *s, float raw_l, float raw_r, float dt_ms) {{
  fe_result_t out;
  float dt = dt_ms * 0.001f;
  float pl = fe_raw_to_proximity(raw_l, s->baseline_l);
  float pr = fe_raw_to_proximity(raw_r, s->baseline_r);
  float vl = 0.0f, vr = 0.0f;
  if (s->has_prev && dt > 0.0f) {{
    vl = (pl - s->prev_l) / dt;
    vr = (pr - s->prev_r) / dt;
    if (vl < 0.0f) vl = 0.0f;
    if (vr < 0.0f) vr = 0.0f;
  }}
  s->prev_l = pl;
  s->prev_r = pr;
  s->has_prev = 1;

#if FE_WITH_LOOMING
  if (s->model == FE_MODEL_LOOMING) {{
    s->gf_l = vl;
    s->gf_r = vr;
    s->drive = (vl > vr ? vl : vr) / FE_LOOM_THRESHOLD;
    out.trigger = fe_trigger(s, dt);
    out.side = fe_side(pl, pr); /* the nearer channel */
    out.drive = s->drive;
    return out;
  }}
#endif

  {{
    static float u[FE_N_NEURONS]; /* scratch (not part of the model state) */
    float ext[4];
    ext[0] = FE_K_IN * vl / FE_V_REF;
    ext[1] = FE_K_IN * vr / FE_V_REF;
    ext[2] = FE_K_IN * pl;
    ext[3] = FE_K_IN * pr;
    if (dt > 0.0f) {{
      uint16_t i, n_sub = (uint16_t)ceilf(dt / FE_DT_SUB - 1e-6f);
      float a;
      if (n_sub < 1) n_sub = 1;
      a = 1.0f - expf(-(dt / (float)n_sub) / FE_TAU);
      for (i = 0; i < n_sub; i++) fe_substep(s->r, ext, a, u);
    }}
  }}
  s->gf_l = s->r[FE_GF_L];
  s->gf_r = s->r[FE_GF_R];
  {{
    float dl = s->gf_l / FE_TH_L, dr = s->gf_r / FE_TH_R;
    s->drive = dl > dr ? dl : dr;
  }}
  out.trigger = fe_trigger(s, dt);
  out.side = fe_side(fe_mean(s->r, FE_SEG[0], FE_SEG[1]) + fe_mean(s->r, FE_SEG[2], FE_SEG[3]),
                     fe_mean(s->r, FE_SEG[1], FE_SEG[2]) + fe_mean(s->r, FE_SEG[3], FE_SEG[4]));
  out.drive = s->drive;
  return out;
}}
"""


def render_compact(model: CompactModel) -> tuple[str, str, str]:
    arrays = [
        _c_array("uint16_t", "FE_ROW_PTR", model.row_ptr.tolist(), 16),
        _c_array("uint16_t", "FE_COL", model.col.tolist(), 16),
        _c_array("uint8_t", "FE_CNT", model.cnt.tolist(), 24),
        _c_array("uint8_t", "FE_CLASS", model.cls.tolist(), 24),
        _c_array("float", "FE_CLASS_SCALE", model.cls_scale.tolist(), 4, _f32),
        _c_array("float", "FE_CLASS_BIAS", model.cls_bias.tolist(), 4, _f32),
    ]
    sc = model.scalars
    fields = {
        "date": model.meta["date"],
        "params_key": model.meta["params_key"],
        "pack": model.meta["pack"],
        "n": model.n,
        "n_full": model.meta["n_full"],
        "dropped": ", ".join(model.meta["dropped_neurons"]) or "none",
        "nnz": len(model.cnt),
        "n_cls": len(model.cls_scale),
        "seg": ", ".join(str(x) for x in model.seg),
        "gf_l": model.gf[0],
        "gf_r": model.gf[1],
        "arrays": "\n".join(arrays),
        **{k: _f32(v) for k, v in sc.items()},
    }
    header = HEADER.format(**fields).replace(
        " * Usage (both models):",
        " * The looming-rate baseline is compiled only with #define FE_WITH_LOOMING 1.\n"
        " *\n * Usage (both models):",
    )
    source = SOURCE_COMPACT.format(
        fname="fly_escape.c", include='#include "fly_escape.h"\n#define FE_API', **fields
    )
    types = header.split("void fe_init(")[0]
    single = (
        types.replace("fly_escape.h - generated", "fly_escape_all.h - generated").replace(
            "FLY_ESCAPE_H", "FLY_ESCAPE_ALL_H"
        )
        + "#ifdef __cplusplus\n}\n#endif\n\n"
        + SOURCE_COMPACT.format(
            fname="fly_escape_all.h (implementation)",
            include="#define FE_API static inline",
            **fields,
        )
        + "\n#endif /* FLY_ESCAPE_ALL_H */\n"
    )
    return header, source, single


def cost_compact(model: CompactModel) -> dict:
    n, nnz, nc = model.n, len(model.cnt), len(model.cls_scale)
    return {
        "flash_data_bytes": 2 * (n + 1) + 2 * nnz + nnz + n + 8 * nc + 10 + 15 * 4,
        "state_ram_bytes": 4 * n + 4 * 11,
        "scratch_ram_bytes": 4 * n,
        "macs_per_substep": nnz,
    }
