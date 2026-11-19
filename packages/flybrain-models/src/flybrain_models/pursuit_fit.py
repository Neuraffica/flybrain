"""Offline fit of the pursuit models' single scalars (never per-synapse weights).

    uv run python -m flybrain_models.pursuit_fit

Each model has ONE fitted scalar, found on ``pursuit_tasks.training_set()`` (seed disjoint
from the benchmark) with the same loss for all:

    loss = time-weighted mean absolute bearing error (deg) over the training episodes
           + 1.0 * |turn rate| (deg/s) after 2 s with no target (the idle turn)

* PursuitCircuit: ``k_out`` (rad/s per unit of DNa02 rate difference), 1-D search: a coarse
  log grid, then a finer log grid around the best coarse value. The circuit is the
  mirror-averaged connectome model at arousal 1.
* ProportionalSteer: its gain ``k``, log grid.
* BangBangSteer: its threshold, linear grid.

Everything runs in one process and takes seconds to a minute.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import time

import numpy as np

from .pursuit import BangBangSteer, ProportionalSteer, PursuitCircuit, params_path
from .pursuit_tasks import evaluate, loss_of, training_set

K_OUT_COARSE = np.geomspace(100.0, 30000.0, 13)
K_OUT_FINE_STEPS = 13  # points of the refinement grid, spanning the coarse neighbours


def _search(make, grid, eps) -> tuple[float, float, dict, list]:
    best, table = None, []
    for v in grid:
        summ = evaluate(make(float(v)), eps)[0]
        val = loss_of(summ)
        table.append([float(v), round(val, 4)])
        if best is None or val < best[0]:
            best = (val, float(v), summ)
    return best[0], best[1], best[2], table


def fit_circuit(log=print) -> dict:
    from flybrain_data import load_pack

    t0 = time.time()
    eps = training_set()
    pack = load_pack("pursuit")

    def make(k):
        return PursuitCircuit(pack, k_out=k)

    _, k0, _, coarse = _search(make, K_OUT_COARSE, eps)
    i = int(np.argmin(np.abs(K_OUT_COARSE - k0)))
    lo = K_OUT_COARSE[max(i - 1, 0)]
    hi = K_OUT_COARSE[min(i + 1, len(K_OUT_COARSE) - 1)]
    fine_grid = np.geomspace(lo, hi, K_OUT_FINE_STEPS)
    loss, k, summ, fine = _search(make, fine_grid, eps)
    log(f"k_out coarse best {k0:.0f}, refined {k:.1f}, train loss {loss:.4f}")
    return {
        "params": {"k_out": k},
        "train_loss": round(loss, 4),
        "train_summary": summ,
        "search": {"coarse": coarse, "fine": fine},
        "fit_seconds": round(time.time() - t0, 1),
        "date": _dt.date.today().isoformat(),
    }


def fit_baseline(make, grid, key: str) -> dict:
    t0 = time.time()
    loss, v, summ, _ = _search(make, grid, training_set())
    return {
        "params": {key: v},
        "train_loss": round(loss, 4),
        "train_summary": summ,
        "grid": [float(grid[0]), float(grid[-1]), len(grid)],
        "fit_seconds": round(time.time() - t0, 1),
        "date": _dt.date.today().isoformat(),
    }


ABOUT = (
    "Single fitted scalar of each pursuit model, fitted offline by flybrain_models.pursuit_fit "
    "on pursuit_tasks.training_set() (disjoint from the benchmark), loss = time-weighted mean "
    "absolute bearing error in degrees + 1.0 x idle turn (deg/s with no target). 'connectome' "
    "(k_out, rad/s per unit DNa02 rate difference, mirror-averaged circuit) is applied "
    "unchanged to the raw circuit, the rewired/random controls and the ablations; "
    "'proportional' (k, rad/s per rad) and 'bang_bang' (threshold, rad) are the baselines."
)


def save_fits(fits: dict) -> None:
    path = params_path()
    path.write_text(json.dumps({"about": ABOUT, "fits": dict(sorted(fits.items()))}, indent=1))
    print(f"wrote {path}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.parse_args(argv)
    fits = {
        "proportional": fit_baseline(
            lambda v: ProportionalSteer(k=v), np.geomspace(0.2, 20.0, 121), "k"
        ),
        "bang_bang": fit_baseline(
            lambda v: BangBangSteer(threshold=v),
            np.radians(np.linspace(0.5, 25.0, 99)),
            "threshold",
        ),
        "connectome": fit_circuit(),
    }
    for k, v in fits.items():
        print(f"{k:<13} {v['params']}  train loss {v['train_loss']}  ({v['fit_seconds']} s)")
    save_fits(fits)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
