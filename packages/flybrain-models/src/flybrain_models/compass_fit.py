"""Offline fit of the compass model's global scalars (never per-synapse weights).

Shipped pipeline (README "Rebuild" has the exact commands), for the connectome and, identically,
for each refitted control:

1. ``--config pen_dendrites --tau-max 0.2 --jitter-seed {1,2} --probe-iters 100
   --traj-iters 0``: 13 scalars (5 type gains, 5 type biases, tau, 2 velocity-input scalars)
   on a velocity-response objective (hold 0, +-20, 60, 120, 200 deg/s for 1.5 s, match the
   bump speed, keep a bump);
2. ``--memory-from <stage-1 key>``: all 15 scalars (+ cue gain, cue width) jointly on
   ``memory_loss``: 8 headings, bump started 90 deg away, 5 Hz cue for 2 s, 10 s darkness;
   plus tracking a noisy 5 Hz cue on one training trajectory.
   The start with the lower stage-2 loss is kept.

Training data (8 headings offset from the benchmark's 24, ``compass_tasks.training_set``) is
disjoint from the benchmark. Results go to ``compass_params.json`` inside the package.

The older velocity-only fit (``--config point``, probe then trajectory stage) is kept as
``connectome_dead_reckoning``.
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

from .compass import RingAttractor, params_path
from .compass_tasks import run, training_set

TYPES = ("EPG", "PEN_a", "PEN_b", "PEG", "Delta7")

# Model configurations (fixed assumptions, not fitted).
CONFIGS: dict[str, dict] = {
    # plain point neurons: synapse counts summed over all neuropils, exactly the pack's W
    "point": {"compartments": {}},
    # PEN neurons ignore synapses they receive in the ellipsoid body. PEN arbors in the EB are
    # mostly presynaptic (FlyWire input fraction 0.28-0.31 vs 0.98-0.99 in the bridge), and
    # the EB EPG->PEN synapses sit at the PEN's own output position, cancelling the
    # bridge-to-EB offset that lets PENs push the bump.
    "pen_dendrites": {"compartments": {"PEN_a": {"EB": 0.0}, "PEN_b": {"EB": 0.0}}},
}
# Same assumptions, but one gain per pre-type -> post-type block instead of per pre-type.
CONFIGS["point_blocks"] = {**CONFIGS["point"], "fit_blocks": True}
CONFIGS["pen_dendrites_blocks"] = {**CONFIGS["pen_dendrites"], "fit_blocks": True}

# Blocks with at least 5 connected pairs in the connectome (others keep gain 1 x type gain 1).
BLOCKS = (
    "EPG->EPG",
    "EPG->PEN_a",
    "EPG->PEN_b",
    "EPG->PEG",
    "EPG->Delta7",
    "PEN_a->EPG",
    "PEN_a->PEN_a",
    "PEN_a->PEN_b",
    "PEN_a->PEG",
    "PEN_b->EPG",
    "PEN_b->PEN_a",
    "PEN_b->PEN_b",
    "PEN_b->PEG",
    "PEG->EPG",
    "PEG->PEN_b",
    "PEG->Delta7",
    "Delta7->EPG",
    "Delta7->PEN_a",
    "Delta7->PEN_b",
    "Delta7->PEG",
    "Delta7->Delta7",
)
DEFAULT_CONFIG = "point"

FIXED = {"w_scale": 100.0, "cue_gain": 1.0, "cue_kappa": 2.0}
TAU_RANGE = (0.01, 0.5)

# (name, transform) of each fitted scalar
SPACE: list[tuple[str, str]] = (
    [(f"gain.{t}", "log") for t in TYPES]
    + [(f"bias.{t}", "lin") for t in TYPES]
    + [("tau", "log"), ("vel_gain", "log"), ("vel_amp", "log")]
)
X0 = np.array([0.0, -1.0, -1.0, 0.0, 1.0, -0.5, 0.5, -0.5, 1.0, -0.5, math.log(0.1), 0.0, 0.0])


def space_for(config: str) -> list[tuple[str, str]]:
    if CONFIGS[config].get("fit_blocks"):
        return [(f"block_gain.{b}", "log") for b in BLOCKS] + SPACE[len(TYPES) :]
    return SPACE


def x0_for(config: str) -> np.ndarray:
    if CONFIGS[config].get("fit_blocks"):
        pre = {t: X0[i] for i, t in enumerate(TYPES)}
        return np.concatenate([[pre[b.split("->")[0]] for b in BLOCKS], X0[len(TYPES) :]])
    return X0.copy()


PROBE_W_DEG = (0, 20, 60, 120, 200, -20, -60, -120, -200)

# Landmark-cue scalars (gain and von Mises concentration of the EPG cue current).
CUE_SPACE: list[tuple[str, str]] = [("cue_gain", "log"), ("cue_kappa", "log")]
CUE_X0 = np.array([0.0, math.log(3.0)])
# Memory-fit headings: 8 headings offset from the benchmark's 24 so they are not reused.
MEMORY_FIT_HEADINGS = tuple(float(x) for x in np.linspace(-np.pi, np.pi, 8, endpoint=False) + 0.2)


def decode(
    theta: np.ndarray, config: str, ring_direction: float = 1.0, tau_max: float | None = None
) -> dict:
    cfg = {k: v for k, v in CONFIGS[config].items() if k != "fit_blocks"}
    p: dict = {
        **FIXED,
        **json.loads(json.dumps(cfg)),
        "gain": {},
        "bias": {},
        "ring_direction": ring_direction,
        "tau_max": tau_max,
    }
    space = space_for(config)
    if len(theta) == len(space) + len(CUE_SPACE):
        space = space + CUE_SPACE
        p["cue_fitted"] = True
    if CONFIGS[config].get("fit_blocks"):
        p["gain"] = {t: 1.0 for t in TYPES}
        p["block_gain"] = {}
    for (name, tr), v in zip(space, theta, strict=True):
        val = float(math.exp(min(v, 30.0)) if tr == "log" else v)
        if "." in name:
            group, key = name.split(".", 1)
            p[group][key] = val
        else:
            p[name] = val
    p["tau"] = min(max(p["tau"], TAU_RANGE[0]), p.pop("tau_max", None) or TAU_RANGE[1])
    p["dt_max"] = p["tau"] / 4.0
    return p


def probe_loss(params: dict, pack) -> float:
    m = RingAttractor(pack, params)
    total = 0.0
    for j, wdeg in enumerate(PROBE_W_DEG):
        h0 = 0.7 * j
        m.reset(h0)
        hs = np.array([m.step(math.radians(wdeg), 0.02) for _ in range(75)])
        u = np.unwrap(np.concatenate([[h0], hs]))
        speed = math.degrees(u[-1] - u[25])  # deg/s over the last 1 s
        total += abs(speed - wdeg) / 40.0 + 4.0 * max(0.0, 0.5 - m.confidence)
    return total


def traj_loss(params: dict, pack, trajs, window_s: float = 2.0) -> float:
    m = RingAttractor(pack, params)
    vals = []
    for t in trajs:
        out = run(m, t)
        dec = np.unwrap(out["decoded"])
        true = t.heading
        k = round(window_s / t.dt)
        win = (dec[k:] - dec[:-k]) - (true[k:] - true[:-k])
        vals.append(
            np.degrees(np.abs(win).mean() + 0.25 * np.abs(out["error"]).mean())
            + 100.0 * np.maximum(0.0, 0.5 - out["confidence"]).mean()
        )
    return float(np.mean(vals))


def memory_loss(params: dict, pack, trajs) -> float:
    """Cue + darkness objective (the 'memory' stage). For 8 headings: start the bump 90 deg
    away, give the cue at 5 Hz for 2 s, then 10 s of darkness at rest. Loss = mean error at
    the end of darkness (90 deg when no coherent bump is left, i.e. chance) + 0.5 * mean
    error at the end of the cue + 0.5 * MAE tracking a noisy 5 Hz cue on a training
    trajectory. Rewards a cue that moves the bump and recurrence that holds it."""
    from .compass_tasks import memory_trial, with_cue

    m = RingAttractor(pack, params)
    dark, cued = [], []
    for h in MEMORY_FIT_HEADINGS:
        r = memory_trial(m, h, cue_s=2.0, dark_s=(10.0,))
        err, conf = r[10.0]
        dark.append(abs(math.degrees(err)) if conf > 0.3 else 90.0)
        cued.append(abs(math.degrees(r["cue_end"][0])))
    out = run(m, with_cue(trajs[1], noise_deg=5.0, seed=11))
    track = float(np.degrees(np.abs(out["error"])).mean())
    return float(np.mean(dark) + 0.5 * np.mean(cued) + 0.5 * track)


def pen_shift_direction(pack) -> float:
    """+1 if left-bridge PENs project (EB synapses) to EPGs at larger ring angles than the EPGs
    they receive from in the bridge (PB synapses), and right PENs the other way; else -1.
    Read off the connectome; used to fix the reflection of the inferred ring."""
    epg = pack.index("EPG")
    z = np.exp(1j * pack.ring_angle[epg])
    eb, pb = pack.counts_in("EB").astype(float), pack.counts_in("PB").astype(float)
    shift = {}
    for side in ("left", "right"):
        d = []
        for t in ("PEN_a(PEN1)", "PEN_b(PEN2)"):
            for j in pack.index(t, side):
                zin, zout = (pb[j, epg] * z).sum(), (eb[epg, j] * z).sum()
                if abs(zin) > 0 and abs(zout) > 0:
                    d.append(np.angle(zout / zin))
        shift[side] = float(np.angle(np.mean(np.exp(1j * np.array(d)))))
    return 1.0 if shift["left"] > shift["right"] else -1.0


# ---- CMA-ES ------------------------------------------------------------------------------
_G: dict = {}


def _init_worker(variant: str, seed: int, fixed_theta=None) -> None:
    from flybrain_data import load_pack

    np.seterr(all="ignore")
    _G["fixed_theta"] = None if fixed_theta is None else np.asarray(fixed_theta, float)
    _G["pack"] = load_pack("central_complex", variant=variant, seed=seed)
    _G["trajs"] = training_set()


def _eval(args) -> float:
    theta, config, ring_dir, stage, tau_max = args
    try:
        if stage == "cue_only":  # optimise only the 2 cue scalars on top of fixed ones
            theta = np.concatenate([_G["fixed_theta"], theta])
        p = decode(theta, config, ring_dir, tau_max)
        if stage == "probe":
            val = probe_loss(p, _G["pack"])
        elif stage == "traj":
            val = traj_loss(p, _G["pack"], _G["trajs"])
        else:
            val = memory_loss(p, _G["pack"], _G["trajs"])
    except (FloatingPointError, ValueError, OverflowError):
        return 1e6
    return val if np.isfinite(val) else 1e6


def cma_es(
    f_batch, x0: np.ndarray, sigma0: float, iters: int, popsize: int, seed: int, log=print
) -> tuple[np.ndarray, float, list[float]]:
    """Minimal (mu/mu_w, lambda)-CMA-ES (Hansen's tutorial defaults)."""
    rng = np.random.default_rng(seed)
    n = len(x0)
    lam, mu = popsize, popsize // 2
    w = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
    w /= w.sum()
    mueff = 1.0 / (w**2).sum()
    cc = (4 + mueff / n) / (n + 4 + 2 * mueff / n)
    cs = (mueff + 2) / (n + mueff + 5)
    c1 = 2 / ((n + 1.3) ** 2 + mueff)
    cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((n + 2) ** 2 + mueff))
    damps = 1 + 2 * max(0, math.sqrt((mueff - 1) / (n + 1)) - 1) + cs
    chin = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
    mean, sigma = x0.astype(float).copy(), sigma0
    C = np.eye(n)
    pc, ps = np.zeros(n), np.zeros(n)
    best_x, best_f, hist = mean.copy(), math.inf, []
    for g in range(iters):
        evals, B = np.linalg.eigh(C)
        D = np.sqrt(np.maximum(evals, 1e-20))
        y = (rng.standard_normal((lam, n)) * D) @ B.T
        X = mean + sigma * y
        F = np.asarray(f_batch(X))
        idx = np.argsort(F)
        if F[idx[0]] < best_f:
            best_f, best_x = float(F[idx[0]]), X[idx[0]].copy()
        hist.append(float(F[idx[0]]))
        y_sel = y[idx[:mu]]
        y_w = w @ y_sel
        mean = mean + sigma * y_w
        invsqrt = B @ np.diag(1 / D) @ B.T
        ps = (1 - cs) * ps + math.sqrt(cs * (2 - cs) * mueff) * invsqrt @ y_w
        hsig = np.linalg.norm(ps) / math.sqrt(1 - (1 - cs) ** (2 * (g + 1))) / chin < 1.4 + 2 / (
            n + 1
        )
        pc = (1 - cc) * pc + hsig * math.sqrt(cc * (2 - cc) * mueff) * y_w
        C = (
            (1 - c1 - cmu) * C
            + c1 * (np.outer(pc, pc) + (1 - hsig) * cc * (2 - cc) * C)
            + cmu * (y_sel.T * w) @ y_sel
        )
        sigma *= math.exp((cs / damps) * (np.linalg.norm(ps) / chin - 1))
        if (g + 1) % 10 == 0 or g == 0:
            log(
                f"  gen {g + 1:3d}  best {F[idx[0]]:8.3f}  median {np.median(F):8.3f}  "
                f"overall {best_f:8.3f}  sigma {sigma:.3f}"
            )
    return best_x, best_f, hist


def fit(
    config: str = DEFAULT_CONFIG,
    variant: str = "connectome",
    seed: int = 0,
    probe_iters: int = 60,
    traj_iters: int = 60,
    popsize: int = 24,
    x0: np.ndarray | None = None,
    workers: int | None = None,
    log=print,
    tau_max: float | None = None,
    jitter_seed: int | None = None,
) -> dict:
    """Two-stage fit (velocity probe, then trajectories). ``tau_max`` lowers the time-constant
    bound; ``jitter_seed`` perturbs the default start (gains and biases, N(0, 0.7)) and starts
    tau at 50 ms, a multi-start device."""
    from flybrain_data import load_pack

    ring_dir = pen_shift_direction(load_pack("central_complex"))
    x = x0_for(config) if x0 is None else np.asarray(x0, float)
    if jitter_seed is not None:
        rng = np.random.default_rng(jitter_seed)
        x[:10] += rng.normal(0.0, 0.7, 10)
        x[10] = math.log(0.05)
    t0 = time.time()
    workers = workers or min(8, os.cpu_count() or 1)
    stages = {}
    with ProcessPoolExecutor(workers, initializer=_init_worker, initargs=(variant, seed)) as ex:
        for stage, iters, sigma in (("probe", probe_iters, 0.8), ("traj", traj_iters, 0.3)):
            if iters <= 0:
                continue
            log(f"[{config}/{variant}/seed{seed}] stage {stage}: {iters} generations")

            def f_batch(X, stage=stage):
                return list(ex.map(_eval, [(xi, config, ring_dir, stage, tau_max) for xi in X]))

            cma_seed = seed if jitter_seed is None else jitter_seed
            x, fbest, hist = cma_es(f_batch, x, sigma, iters, popsize, seed=cma_seed, log=log)
            stages[stage] = {"best_loss": round(fbest, 4), "history": [round(h, 3) for h in hist]}
    params = decode(x, config, ring_dir, tau_max)
    return {
        "params": params,
        "theta": x.round(6).tolist(),
        "config": config,
        "variant": variant,
        "control_seed": seed,
        "tau_max": tau_max,
        "jitter_seed": jitter_seed,
        "stages": stages,
        "popsize": popsize,
        "fit_seconds": round(time.time() - t0, 1),
        "date": _dt.date.today().isoformat(),
    }


def fit_memory(
    base_key: str,
    variant: str = "connectome",
    seed: int = 0,
    joint: bool = True,
    iters: int = 50,
    popsize: int = 24,
    workers: int | None = None,
    log=print,
) -> dict:
    """Fit the 2 cue scalars (``joint=False``: the 13 scalars of ``base_key`` stay fixed) or
    all 15 jointly, starting from ``base_key``, on ``memory_loss``."""

    from .compass import load_params

    base = json.loads(params_path().read_text())["fits"][base_key]
    config = base["config"]
    ring_dir = load_params(base_key)["ring_direction"]
    base_theta = np.asarray(base["theta"], float)[: len(space_for(config))]
    tau_max = base.get("tau_max")
    t0 = time.time()
    workers = workers or min(8, os.cpu_count() or 1)
    stage = "memory" if joint else "cue_only"
    x0 = np.concatenate([base_theta, CUE_X0]) if joint else CUE_X0.copy()
    with ProcessPoolExecutor(
        workers, initializer=_init_worker, initargs=(variant, seed, None if joint else base_theta)
    ) as ex:
        log(f"[{config}/{variant}/seed{seed}] stage {stage} from {base_key}: {iters} gens")

        def f_batch(X):
            return list(ex.map(_eval, [(xi, config, ring_dir, stage, tau_max) for xi in X]))

        x, fbest, hist = cma_es(
            f_batch, x0, 0.5 if joint else 1.0, iters, popsize, seed=seed, log=log
        )
    full = x if joint else np.concatenate([base_theta, x])
    params = decode(full, config, ring_dir, tau_max)
    return {
        "params": params,
        "theta": full.round(6).tolist(),
        "config": config,
        "variant": variant,
        "control_seed": seed,
        "tau_max": tau_max,
        "base_fit": base_key,
        "objective": "memory_loss (cue + darkness)",
        "joint": joint,
        "stages": {stage: {"best_loss": round(fbest, 4), "history": [round(h, 3) for h in hist]}},
        "popsize": popsize,
        "fit_seconds": round(time.time() - t0, 1),
        "date": _dt.date.today().isoformat(),
    }


ABOUT = (
    "Global scalars of flybrain_models.compass.RingAttractor fitted offline by "
    "flybrain_models.compass_fit (CMA-ES). 'connectome' is the shipped default: config "
    "pen_dendrites, stage 1 = velocity-probe fit with tau <= 0.2 s from two jittered starts "
    "(jitter seeds 1, 2), stage 2 = joint fit of all 15 scalars (13 + cue gain, cue width) on "
    "the cue+darkness memory objective; of the two starts the lower stage-2 training loss is "
    "kept. It is applied unchanged to the rewired/random controls. 'rewired_seed0' and "
    "'random_seed0' are the identical pipeline run on those controls. '*_base' are the stage-1 "
    "fits. 'connectome_dead_reckoning' is the earlier velocity-only fit (plain summed weights)."
)


def save_fit(key: str, result: dict) -> None:
    import fcntl
    import tempfile

    path = params_path()
    lock_path = os.path.join(tempfile.gettempdir(), "flybrain_compass_params.lock")
    with open(lock_path, "w") as lock:  # several fits may run in parallel
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = json.loads(path.read_text()) if path.exists() else {"fits": {}}
        data["about"] = ABOUT
        data["fits"][key] = result
        data["fits"] = dict(sorted(data["fits"].items()))
        path.write_text(json.dumps(data, indent=1) + "\n")
    print(f"wrote {path} [{key}]")


def default_key(config: str, variant: str, seed: int) -> str:
    suffix = "" if config == DEFAULT_CONFIG else f"_{config}"
    if variant == "connectome":
        return "connectome" + suffix
    return f"{variant}_seed{seed}" + suffix


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--config", default=DEFAULT_CONFIG, choices=sorted(CONFIGS))
    ap.add_argument("--variant", default="connectome", choices=["connectome", "rewired", "random"])
    ap.add_argument("--seed", type=int, default=0, help="control seed (also seeds CMA-ES)")
    ap.add_argument("--probe-iters", type=int, default=60)
    ap.add_argument("--traj-iters", type=int, default=60)
    ap.add_argument("--popsize", type=int, default=24)
    ap.add_argument("--workers", type=int, default=None)
    ap.add_argument("--key", default=None, help="key in compass_params.json")
    ap.add_argument("--tau-max", type=float, default=None)
    ap.add_argument("--jitter-seed", type=int, default=None)
    ap.add_argument(
        "--memory-from",
        default=None,
        metavar="BASE_KEY",
        help="fit the cue scalars on the cue+darkness objective starting from "
        "this fit (all 15 scalars jointly unless --cue-only)",
    )
    ap.add_argument("--cue-only", action="store_true")
    ap.add_argument("--iters", type=int, default=50, help="generations for --memory-from")
    args = ap.parse_args(argv)
    if args.memory_from:
        res = fit_memory(
            args.memory_from,
            args.variant,
            args.seed,
            not args.cue_only,
            args.iters,
            args.popsize,
            workers=args.workers,
        )
        print(json.dumps(res["params"], indent=1))
        save_fit(args.key or f"{args.variant}_seed{args.seed}_memory", res)
        return 0
    res = fit(
        args.config,
        args.variant,
        args.seed,
        args.probe_iters,
        args.traj_iters,
        args.popsize,
        workers=args.workers,
        tau_max=args.tau_max,
        jitter_seed=args.jitter_seed,
    )
    print(json.dumps(res["params"], indent=1))
    save_fit(args.key or default_key(args.config, args.variant, args.seed), res)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
