"""Generate a single C header for the fly pursuit circuit (``fly_pursuit_all.h``).

The model is ``flybrain_models.pursuit.PursuitCircuit`` with the shipped ``k_out`` and mirror
averaging: a circuit of 265 neurons from the fly connectome (LC10a -> AOTU019/AOTU025 and the
bridging types -> DNa02), rectified-linear rate units, exponential-Euler sub-steps of at most
10 ms, two copies of the state (copy A gets the bearing, copy B the mirrored bearing) and
``turn = k_out * 0.5 * (d_A - d_B)`` with ``d = DNa02_left - DNa02_right``.

How the same numbers are stored (no change to the mathematics):

* ``W[post, pre] = sign(pre) * count[post, pre] / input_total[post]`` is kept as its three
  factors: the FlyWire synapse count as ``uint8`` (the largest is 216; counts above 255 would be
  split into several entries of the same row and column), one sign bit per presynaptic neuron,
  and one float ``1 / max(input_total, 1)`` per postsynaptic neuron. The sum over a row is taken
  over ``count * signed rate`` and multiplied by the reciprocal once.
* Rows in CSR form with ``uint16`` row pointers and column indices.
* The LC10a that receive input are listed by index with their receptive-field azimuth.
* Neurons with no directed path to either DNa02 are dropped (``prune=True``): the turn command
  and the DNa02 rates depend only on DNa02 and its upstream closure, so this cannot change the
  output. The export reports how many were dropped and of which types.

float32 everywhere except two scalars per step (the number of sub-steps and the decay factor),
which are computed in double so that the sub-step count is the same as Python's at exact
multiples of 10 ms. No heap; data are ``static const`` (flash on the ESP32); functions are
``static inline``. libm: ``expf``, ``floorf``, ``exp``, ``ceil``.
"""

from __future__ import annotations

import datetime as _dt
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .escape_c import _c_array, _f32
from .escape_c_compact import readout_closure

PREFIX = "fp"


@dataclass
class PursuitExport:
    n: int
    row_ptr: np.ndarray  # (n+1,) CSR row starts
    col: np.ndarray  # (nnz,) presynaptic index
    cnt: np.ndarray  # (nnz,) synapse count, uint8
    neg: np.ndarray  # (n,) bool, presynaptic sign is negative
    inv_total: np.ndarray  # (n,) float32, 1 / max(input_total, 1)
    lc_idx: np.ndarray  # LC10a with a receptive field (input units)
    lc_rf: np.ndarray  # their azimuths, rad
    dn: tuple[int, int]  # DNa02 left, right
    a19: tuple[int, int]  # AOTU019 left, right
    scalars: dict
    meta: dict = field(default_factory=dict)


def build_pursuit(prune: bool = True) -> PursuitExport:
    from flybrain_models import pursuit as pm

    m = pm.PursuitCircuit.from_pack()
    if m.ablate or not m.mirror_average:
        raise ValueError("only the unablated, mirror-averaged circuit is exported")
    pack = m.pack
    W = m._W
    counts = np.asarray(pack.counts).astype(np.int64)
    sign = np.asarray(pack.sign).astype(float)
    total = np.maximum(np.asarray(pack.input_total, float), 1.0)
    n_full = W.shape[0]
    kept = readout_closure(W, m._dn) if prune else np.arange(n_full)
    remap = -np.ones(n_full, dtype=np.int64)
    remap[kept] = np.arange(len(kept))

    rows, cols, cnts = [], [], []
    for new_i, i in enumerate(kept):
        for j in np.flatnonzero(W[i]):
            if remap[j] < 0:
                continue
            c = int(counts[i, j])
            while c > 0:
                rows.append(new_i)
                cols.append(int(remap[j]))
                cnts.append(min(c, 255))
                c -= 255
    rows, cols, cnts = map(np.array, (rows, cols, cnts))
    n = len(kept)
    row_ptr = np.zeros(n + 1, dtype=np.int64)
    np.add.at(row_ptr, rows + 1, 1)
    row_ptr = np.cumsum(row_ptr)
    neg = sign[kept] < 0
    inv_total = (1.0 / total[kept]).astype(np.float32)

    # the stored factors reproduce W on the kept neurons
    Wc = np.zeros((n, n))
    np.add.at(Wc, (rows, cols), cnts * np.where(neg[cols], -1.0, 1.0) / total[kept][rows])
    assert np.allclose(Wc, W[np.ix_(kept, kept)], rtol=1e-12, atol=0)

    lc_all = remap[m._lc_in]
    lc_keep = lc_all >= 0  # an LC10a without a path to DNa02 cannot change the output
    lc_idx = lc_all[lc_keep]
    return PursuitExport(
        n=n,
        row_ptr=row_ptr.astype(np.uint16),
        col=cols.astype(np.uint16),
        cnt=cnts.astype(np.uint8),
        neg=neg,
        inv_total=inv_total,
        lc_idx=lc_idx.astype(np.uint16),
        lc_rf=np.asarray(m._rf, float)[lc_keep].astype(np.float32),
        dn=(int(remap[m._dn[0]]), int(remap[m._dn[1]])),
        a19=(int(remap[m._a19[0]]), int(remap[m._a19[1]])),
        scalars={
            "tau": pm.TAU,
            "dt_sub": pm.DT_SUB,
            "sigma_rf": pm.SIGMA_RF,
            "lc_input": pm.LC10A_INPUT,
            "dn_bias": pm.DNA02_BIAS,
            "arousal_theta": pm.AROUSAL_THETA,
            "k_out": m.k_out,
        },
        meta={
            "date": _dt.date.today().isoformat(),
            "pack": "pursuit (FlyWire v783, CC BY 4.0)",
            "n_full": n_full,
            "dropped_neurons": [
                str(pack.cell_type[i]) for i in sorted(set(range(n_full)) - set(kept.tolist()))
            ],
        },
    )


def cost(px: PursuitExport) -> dict:
    """Bytes of the exported data in flash and of the state in RAM."""
    nnz, n, nlc = len(px.cnt), px.n, len(px.lc_idx)
    flash = {
        "row_ptr": 2 * (n + 1),
        "col": 2 * nnz,
        "cnt": nnz,
        "neg_bits": (n + 7) // 8,
        "inv_total": 4 * n,
        "lc_idx": 2 * nlc,
        "lc_rf": 4 * nlc,
    }
    return {
        "n": n,
        "nnz": nnz,
        "n_lc_inputs": nlc,
        "flash_data_bytes": sum(flash.values()),
        "flash_data_breakdown": flash,
        "float32_csr_bytes_for_comparison": 2 * (n + 1) + 2 * nnz + 4 * nnz,
        "state_ram_bytes": 4 * (2 * n + n + nlc),  # two copies + row sums + LC10a input
        "macs_per_substep_per_copy": nnz,
    }


SINGLE = """\
/* fly_pursuit_all.h - generated by flybrain-export ({date}); do not edit by hand.
 *
 * Fly pursuit circuit: {n} neurons from the fly connectome (FlyWire v783: LC10a ->
 * AOTU019/AOTU025 and bridging types -> DNa02; Dorkenwald et al. 2024, Schlegel et al. 2024,
 * CC BY 4.0), the same mathematics as flybrain_models.pursuit.PursuitCircuit with mirror
 * averaging and k_out = {k_out_txt} rad/s per unit of DNa02 rate difference.
 * {nnz} synapse entries (uint8 counts, one sign bit per presynaptic neuron, one reciprocal of
 * the whole-brain input total per neuron). Dropped (no path to DNa02): {dropped}.
 *
 * Usage:
 *   static fp_state_t s; fp_init(&s);
 *   float turn = fp_step(&s, has_target, azimuth_rad, dt_s, arousal);  // rad/s, + = turn left
 *   azimuth: bearing of the target, radians, + = LEFT. has_target = 0: nothing seen.
 *   fp_dna02_left(&s), fp_dna02_right(&s): DNa02 rates of copy A (the real bearing).
 * No heap; all data static const; functions static inline (include from one file).
 */
#ifndef FLY_PURSUIT_ALL_H
#define FLY_PURSUIT_ALL_H

#include <math.h>
#include <stdint.h>

#define FP_N_NEURONS {n}
#define FP_NNZ {nnz}
#define FP_N_LC {n_lc}
#define FP_DN_L {dn_l}
#define FP_DN_R {dn_r}
#define FP_A19_L {a19_l}
#define FP_A19_R {a19_r}
static const double FP_TAU_D = {tau_d};
static const double FP_DT_SUB_D = {dt_sub_d};
static const float FP_SIGMA_RF = {sigma_rf};
static const float FP_LC_INPUT = {lc_input};
static const float FP_DN_BIAS = {dn_bias};
static const float FP_AROUSAL_THETA = {arousal_theta};
static const float FP_K_OUT = {k_out};

{arrays}
typedef struct {{
  float r[2][FP_N_NEURONS]; /* rates: [0] copy A (real bearing), [1] copy B (mirrored) */
  float u[FP_N_NEURONS];    /* scratch: input of every unit in a sub-step */
  float lc[FP_N_LC];        /* scratch: LC10a input of the copy being advanced */
}} fp_state_t;

static inline void fp_reset(fp_state_t *s) {{
  int c, i;
  for (c = 0; c < 2; c++) {{
    for (i = 0; i < FP_N_NEURONS; i++) s->r[c][i] = 0.0f;
    s->r[c][FP_DN_L] = FP_DN_BIAS;
    s->r[c][FP_DN_R] = FP_DN_BIAS;
  }}
}}

static inline void fp_init(fp_state_t *s) {{ fp_reset(s); }}

static inline float fp_dna02_left(const fp_state_t *s) {{ return s->r[0][FP_DN_L]; }}
static inline float fp_dna02_right(const fp_state_t *s) {{ return s->r[0][FP_DN_R]; }}

static inline float fp_turn(const fp_state_t *s) {{
  float da = s->r[0][FP_DN_L] - s->r[0][FP_DN_R];
  float db = s->r[1][FP_DN_L] - s->r[1][FP_DN_R];
  return FP_K_OUT * (0.5f * (da - db));
}}

/* one copy: n sub-steps with decay factor a, LC10a input for a target at az (or none) */
static inline void fp_advance(fp_state_t *s, int copy, uint8_t has_target, float az, float arousal,
                              int n, float a) {{
  const float two_pi = 6.28318530718f, pi = 3.14159265359f;
  float *r = s->r[copy];
  float a19_in;
  int k, i, step;
  for (k = 0; k < FP_N_LC; k++) {{
    float d, z;
    if (!has_target) {{
      s->lc[k] = 0.0f;
      continue;
    }}
    d = FP_LC_RF[k] - az + pi;
    d = d - two_pi * floorf(d / two_pi) - pi;
    z = d / FP_SIGMA_RF;
    s->lc[k] = FP_LC_INPUT * expf(-0.5f * z * z);
  }}
  if (arousal < 0.0f) arousal = 0.0f;
  if (arousal > 1.0f) arousal = 1.0f;
  a19_in = -FP_AROUSAL_THETA * (1.0f - arousal);
  for (step = 0; step < n; step++) {{
    for (i = 0; i < FP_N_NEURONS; i++) {{
      float acc = 0.0f;
      uint16_t e;
      for (e = FP_ROW[i]; e < FP_ROW[i + 1]; e++) {{
        uint16_t j = FP_COL[e];
        float v = (float)FP_CNT[e] * r[j];
        if (FP_NEG[j >> 3] & (uint8_t)(1u << (j & 7))) acc -= v;
        else acc += v;
      }}
      s->u[i] = acc * FP_INV_TOTAL[i];
    }}
    s->u[FP_DN_L] += FP_DN_BIAS;
    s->u[FP_DN_R] += FP_DN_BIAS;
    s->u[FP_A19_L] += a19_in;
    s->u[FP_A19_R] += a19_in;
    for (k = 0; k < FP_N_LC; k++) s->u[FP_LC_IDX[k]] += s->lc[k];
    for (i = 0; i < FP_N_NEURONS; i++) {{
      float u = s->u[i] > 0.0f ? s->u[i] : 0.0f;
      r[i] += a * (u - r[i]);
    }}
  }}
}}

/* Advance by dt_s seconds with the latest bearing (radians, + = left; ignored when has_target
 * is 0) and return the turn command in rad/s (+ = turn left). arousal in [0, 1], 1 = default. */
static inline float fp_step(fp_state_t *s, uint8_t has_target, float azimuth_rad, float dt_s,
                            float arousal) {{
  if (dt_s > 0.0f) {{
    double dt = (double)dt_s;
    int n = (int)ceil(dt / FP_DT_SUB_D - 1e-9);
    float a;
    if (n < 1) n = 1;
    a = (float)(1.0 - exp(-(dt / n) / FP_TAU_D));
    fp_advance(s, 0, has_target, azimuth_rad, arousal, n, a);
    fp_advance(s, 1, has_target, -azimuth_rad, arousal, n, a);
  }}
  return fp_turn(s);
}}

#endif /* FLY_PURSUIT_ALL_H */
"""


def render(px: PursuitExport) -> str:
    neg_bits = np.zeros((px.n + 7) // 8, dtype=np.uint8)
    for j in np.flatnonzero(px.neg):
        neg_bits[j >> 3] |= 1 << (j & 7)
    arrays = "\n".join(
        [
            "/* CSR rows: entries [FP_ROW[i], FP_ROW[i+1]) are the inputs of neuron i */",
            _c_array("uint16_t", "FP_ROW", px.row_ptr.tolist()),
            _c_array("uint16_t", "FP_COL", px.col.tolist(), per_line=16),
            "/* synapse counts (FlyWire) */",
            _c_array("uint8_t", "FP_CNT", px.cnt.tolist(), per_line=20),
            "/* bit j set: presynaptic neuron j is inhibitory (sign -1) */",
            _c_array("uint8_t", "FP_NEG", neg_bits.tolist(), per_line=20),
            "/* 1 / max(input_total, 1): input_total = the neuron's input synapses over the brain */",
            _c_array("float", "FP_INV_TOTAL", px.inv_total.tolist(), per_line=6, fmt=_f32),
            "/* LC10a with a receptive-field estimate: index and azimuth (rad, + = left) */",
            _c_array("uint16_t", "FP_LC_IDX", px.lc_idx.tolist(), per_line=16),
            _c_array("float", "FP_LC_RF", px.lc_rf.tolist(), per_line=6, fmt=_f32),
        ]
    )
    s = px.scalars
    return SINGLE.format(
        date=px.meta["date"],
        n=px.n,
        nnz=len(px.cnt),
        n_lc=len(px.lc_idx),
        dn_l=px.dn[0],
        dn_r=px.dn[1],
        a19_l=px.a19[0],
        a19_r=px.a19[1],
        tau_d=repr(float(s["tau"])),
        dt_sub_d=repr(float(s["dt_sub"])),
        sigma_rf=_f32(s["sigma_rf"]),
        lc_input=_f32(s["lc_input"]),
        dn_bias=_f32(s["dn_bias"]),
        arousal_theta=_f32(s["arousal_theta"]),
        k_out=_f32(s["k_out"]),
        k_out_txt=f"{s['k_out']:.6g}",
        dropped=", ".join(px.meta["dropped_neurons"]) or "none",
        arrays=arrays,
    )


def export(out_dir: str | Path, prune: bool = True) -> dict:
    """Write ``fly_pursuit_all.h`` into ``out_dir``; return sizes and counts."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    px = build_pursuit(prune=prune)
    (out / "fly_pursuit_all.h").write_text(render(px))
    return {
        "dir": str(out),
        "files": ["fly_pursuit_all.h"],
        "k_out": px.scalars["k_out"],
        "dropped": px.meta["dropped_neurons"],
        **cost(px),
    }


def steering_azimuths_deg() -> np.ndarray:
    """Bearings at which the open-loop steering curve is compared (degrees)."""
    return np.arange(-90.0, 90.0 + 1e-9, 5.0)
