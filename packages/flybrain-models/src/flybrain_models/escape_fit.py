"""Offline fit of the escape models' global scalars (never per-synapse weights).

    uv run python -m flybrain_models.escape_fit                       # connectome + baselines
    uv run python -m flybrain_models.escape_fit --variant rewired --seed 0   # control refit
    uv run python -m flybrain_models.escape_fit --ablate-interneurons        # ablation refit

EscapeCircuit: 11 scalars (gains for LC4+LPLC2 together, excitatory and inhibitory
interneurons/DNs, GF; biases for LC4+LPLC2, interneurons/DNs, GF; tau; input gain; one
trigger threshold per GF), CMA-ES. The training set includes approaches that only one
channel sees, so each GF must reach its own threshold on its own side's input. Baselines: their one threshold, grid search. Same loss for all, on
``escape_tasks.training_set()`` (seed disjoint from the benchmark):

    loss = 100 * miss rate + 10 * false alarms per minute + 0.1 * mean latency (ms)
           + 20 * side error rate on lateral hits
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from .compass_fit import cma_es
from .escape import EscapeCircuit, LoomingRateThreshold, ProximityThreshold, params_path
from .escape_tasks import run_episode, score_episode, training_set

FIXED = {"w_scale": 100.0}
SPACE = [
    ("gain.vpn", "log"),
    ("gain.exc", "log"),
    ("gain.inh", "log"),
    ("gain.gf", "log"),
    ("bias.vpn", "lin"),
    ("bias.in", "lin"),
    ("bias.gf", "lin"),
    ("tau", "log"),
    ("k_in", "log"),
    ("threshold_left", "logit"),
    ("threshold_right", "logit"),
]
X0 = np.array([0.0, 0.0, 0.0, 0.0, -0.2, -0.2, -0.2, math.log(0.03), math.log(2.0), 0.0, 0.0])
TAU_RANGE = (0.01, 0.3)


def decode(theta, ablate: bool = False) -> dict:
    p: dict = {**FIXED, "gain": {}, "bias": {}}
    for (name, tr), v in zip(SPACE, theta, strict=True):
        v = float(v)
        val = (
            math.exp(min(v, 30))
            if tr == "log"
            else (1 / (1 + math.exp(-v)) if tr == "logit" else v)
        )
        if "." in name:
            g, k = name.split(".")
            p[g][k] = val
        else:
            p[name] = val
    p["tau"] = min(max(p["tau"], TAU_RANGE[0]), TAU_RANGE[1])
    if ablate:
        p["ablate_interneurons"] = True
    return p


def summarize(scores: list[dict]) -> dict:
    threats = [s for s in scores if s["hit"] is not None]
    hits = [s for s in threats if s["hit"]]
    lat = [s["latency"] for s in hits]
    lateral = [s for s in hits if s["side_ok"] is not None and s.get("lateral")]
    minutes = sum(s["scored_s"] for s in scores) / 60.0
    return {
        "hit_rate": len(hits) / max(len(threats), 1),
        "fa_per_min": sum(s["false_alarms"] for s in scores) / max(minutes, 1e-9),
        "latency_ms": 1000 * float(np.mean(lat)) if lat else float("nan"),
        "side_acc": (float(np.mean([s["side_ok"] for s in lateral])) if lateral else float("nan")),
    }


def evaluate(model, episodes) -> tuple[dict, list[dict]]:
    scores = []
    for ep in episodes:
        s = score_episode(ep, run_episode(model, ep))
        s["lateral"] = ep.kind == "approach" and ep.side != 0
        scores.append(s)
    return summarize(scores), scores


def loss_of(summary: dict) -> float:
    lat = summary["latency_ms"] if np.isfinite(summary["latency_ms"]) else 1000.0
    side = summary["side_acc"] if np.isfinite(summary["side_acc"]) else 0.0
    return (
        100 * (1 - summary["hit_rate"]) + 10 * summary["fa_per_min"] + 0.1 * lat + 20 * (1 - side)
    )


_G: dict = {}


def _init(variant: str, seed: int) -> None:
    from flybrain_data import load_pack

    np.seterr(all="ignore")
    _G["pack"] = load_pack("escape", variant=variant, seed=seed)
    _G["eps"] = training_set()


def _eval(args) -> float:
    theta, ablate = args
    try:
        m = EscapeCircuit(_G["pack"], decode(theta, ablate))
        val = loss_of(evaluate(m, _G["eps"])[0])
    except (FloatingPointError, ValueError, OverflowError):
        return 1e6
    return val if np.isfinite(val) else 1e6


def fit_circuit(
    variant: str = "connectome",
    seed: int = 0,
    ablate: bool = False,
    iters: int = 40,
    popsize: int = 16,
    workers: int | None = None,
    log=print,
) -> dict:
    t0 = time.time()
    workers = workers or min(8, os.cpu_count() or 1)
    # one BLAS thread per worker process (spawned workers read this at numpy import)
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
        os.environ.setdefault(var, "1")
    with ProcessPoolExecutor(workers, initializer=_init, initargs=(variant, seed)) as ex:

        def f_batch(X):
            return list(ex.map(_eval, [(x, ablate) for x in X]))

        x, fbest, hist = cma_es(f_batch, X0.copy(), 0.6, iters, popsize, seed=seed, log=log)
    return {
        "params": decode(x, ablate),
        "theta": np.round(x, 6).tolist(),
        "variant": variant,
        "control_seed": seed,
        "ablate_interneurons": ablate,
        "train_loss": round(fbest, 4),
        "history": [round(h, 3) for h in hist],
        "fit_seconds": round(time.time() - t0, 1),
        "date": _dt.date.today().isoformat(),
    }


def fit_baseline(cls, grid) -> dict:
    eps = training_set()
    best = None
    for th in grid:
        summ = evaluate(cls(threshold=float(th)), eps)[0]
        val = loss_of(summ)
        if best is None or val < best[0]:
            best = (val, float(th), summ)
    return {
        "params": {"threshold": best[1]},
        "train_loss": round(best[0], 4),
        "train_summary": best[2],
        "grid": [float(grid[0]), float(grid[-1]), len(grid)],
        "date": _dt.date.today().isoformat(),
    }


ABOUT = (
    "Global scalars of the escape models, fitted offline by flybrain_models.escape_fit on "
    "escape_tasks.training_set() (disjoint from the benchmark). 'connectome' is shipped and "
    "applied unchanged to the rewired/random controls; '<variant>_seed<k>' are refits on a "
    "control, 'no_interneurons' a refit of the ablation; the baselines have one threshold each."
)


def save_fit(key: str, result: dict) -> None:
    import fcntl
    import tempfile

    path = params_path()
    with open(os.path.join(tempfile.gettempdir(), "flybrain_escape_params.lock"), "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        data = json.loads(path.read_text()) if path.exists() else {"fits": {}}
        data["about"] = ABOUT
        data["fits"][key] = result
        data["fits"] = dict(sorted(data["fits"].items()))
        path.write_text(json.dumps(data, indent=1) + "\n")
    print(f"wrote {path} [{key}]")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--variant", default="connectome", choices=["connectome", "rewired", "random"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--ablate-interneurons", action="store_true")
    ap.add_argument("--iters", type=int, default=40)
    ap.add_argument("--popsize", type=int, default=16)
    ap.add_argument("--workers", type=int, default=None)
    ap.add_argument("--baselines", action="store_true", help="fit only the two baselines")
    args = ap.parse_args(argv)
    if args.baselines:
        save_fit(
            "proximity_threshold", fit_baseline(ProximityThreshold, np.linspace(0.05, 0.99, 95))
        )
        save_fit(
            "looming_rate_threshold",
            fit_baseline(LoomingRateThreshold, np.geomspace(0.2, 30.0, 120)),
        )
        return 0
    res = fit_circuit(
        args.variant, args.seed, args.ablate_interneurons, args.iters, args.popsize, args.workers
    )
    print(json.dumps(res["params"], indent=1))
    if args.ablate_interneurons:
        key = "no_interneurons"
    elif args.variant == "connectome":
        key = "connectome"
    else:
        key = f"{args.variant}_seed{args.seed}"
    save_fit(key, res)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
