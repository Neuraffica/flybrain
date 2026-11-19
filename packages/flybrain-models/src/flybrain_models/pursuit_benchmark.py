"""Benchmark the fly pursuit circuit against controls, ablations and two baselines.

    uv run python -m flybrain_models.pursuit_benchmark            # prints tables, writes JSON
    uv run python -m flybrain_models.pursuit_benchmark --no-write

Closed-loop episodes from ``pursuit_tasks.benchmark_set()`` (114 episodes, seed disjoint
from the training set): steps, appearances (+-100 setting), slow sinusoids, low-pass random
walks and passes, with a +-30, +-60 and +-100 deg field of view (+-100 = head camera plus
neck pan, head loop not simulated). The rig numbers (15 Hz, 100 ms delay, 2 deg noise, 5 %
dropouts, 90 deg/s, 150 ms lag) are ASSUMED, see ``pursuit_tasks``. Every metric is reported
overall and per field-of-view setting, for every model.

Every circuit variant uses the one scalar fitted on the mirror-averaged connectome circuit
(``k_out`` in ``pursuit_params.json``); none is refitted. So the raw (not mirror-averaged)
circuit, the controls, the ablations and the excitatory-AOTU019 counterfactual show what
changes when the wiring or one cell type changes under the same output gain, not how well
each could do after its own fit.

Also reported: the open-loop steering curve (turn command after holding a target at one
azimuth for 1 s, -150..+150 deg) with and without AOTU019 and for the other variants, with
its antisymmetry error, mean |turn(az) + turn(-az)| over the sampled azimuths (0 for a
perfectly left/right symmetric controller). Results go to ``pursuit_benchmark.json``.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import platform
import time
from importlib import resources
from pathlib import Path

import numpy as np

from .pursuit import BangBangSteer, ProportionalSteer, PursuitCircuit, load_params
from .pursuit_tasks import (
    FOVS,
    STEP_KINDS,
    benchmark_set,
    idle_turn,
    loss_of,
    run_episode,
    score_episode,
    summarize,
)

RESULTS_FILE = "pursuit_benchmark.json"
CURVE_DEG = np.arange(-150, 151, 5)
KINDS = ("step", "appear", "sine", "walk", "pass")
SETTINGS = tuple(f"fov{f:g}" for f in FOVS)
METRICS = (
    ("mae_deg", "MAE", 2),
    ("within10", "<10", 2),
    ("in_view", "view", 2),
    ("step_settle_s_median", "settle s", 2),
    ("step_settled_fraction", "settled", 2),
    ("step_overshoot_deg_mean", "oversh", 1),
)


def results_path() -> Path:
    return Path(str(resources.files("flybrain_models") / RESULTS_FILE))


def model_specs(rewired_seeds: int = 5, random_seeds: int = 3) -> dict:
    """name -> (description, [spec]); spec = dict(kind, variant, seed, ablate, sign, arousal)."""

    def c(variant="connectome", seed=0, ablate=(), sign=None, arousal=1.0, mirror=True):
        return {
            "kind": "circuit",
            "variant": variant,
            "seed": seed,
            "ablate": tuple(ablate),
            "sign": sign,
            "arousal": arousal,
            "mirror": mirror,
        }

    return {
        "connectome": ("FlyWire wiring, mirror-averaged, fitted k_out, arousal 1", [c()]),
        "connectome_raw": (
            "FlyWire wiring, single brain (no mirror averaging), connectome k_out",
            [c(mirror=False)],
        ),
        "rewired": (
            f"degree-preserving rewire within type blocks, x{rewired_seeds}, connectome scalars",
            [c("rewired", s) for s in range(rewired_seeds)],
        ),
        "random": (
            f"random graph, same size/density/sign ratio, x{random_seeds}, connectome scalars",
            [c("random", s) for s in range(random_seeds)],
        ),
        "no_AOTU019": ("AOTU019 silenced, connectome scalars", [c(ablate=("AOTU019",))]),
        "no_AOTU025": ("AOTU025 silenced, connectome scalars", [c(ablate=("AOTU025",))]),
        "AOTU019_excitatory": (
            "counterfactual: AOTU019 sign +1, connectome scalars",
            [c(sign=1)],
        ),
        "arousal_0": (
            "connectome, arousal 0 (AOTU019 silenced by the arousal input)",
            [c(arousal=0.0)],
        ),
        "proportional": ("baseline: turn = k * azimuth, k fitted", [{"kind": "p"}]),
        "bang_bang": ("baseline: full-rate turn outside a fitted dead zone", [{"kind": "bb"}]),
    }


def make_model(spec: dict):
    if spec["kind"] == "p":
        return ProportionalSteer()
    if spec["kind"] == "bb":
        return BangBangSteer()
    return PursuitCircuit.from_pack(
        "pursuit",
        variant=spec["variant"],
        seed=spec["seed"],
        ablate=spec["ablate"],
        aotu019_sign=spec["sign"],
        k_out=load_params("connectome")["k_out"],
        mirror_average=spec["mirror"],
    )


def evaluate_spec(spec: dict, episodes) -> dict:
    m = make_model(spec)
    arousal = spec.get("arousal", 1.0)
    t0 = time.perf_counter()
    frames = 0
    scores = []
    for ep in episodes:
        res = run_episode(m, ep, arousal)
        frames += len(res["seen"])
        scores.append(score_episode(ep, res))
    sec = time.perf_counter() - t0
    out = {"all": summarize(scores)}
    out["all"]["idle_turn_deg_s"] = idle_turn(m, arousal)
    out["all"]["loss"] = loss_of(out["all"])
    out["ms_per_simulated_second"] = 1000 * sec / sum(s["seconds"] for s in scores)
    for k in KINDS:
        out[k] = summarize([s for s in scores if s["kind"] == k])
    out["steps_and_appearances"] = summarize([s for s in scores if s["kind"] in STEP_KINDS])
    for fov, name in zip(FOVS, SETTINGS, strict=True):
        out[name] = summarize([s for s in scores if s["fov"] == fov])
    out["frames"] = frames
    return out


def steering_curve(spec: dict) -> list[float]:
    """Turn command (deg/s, + = left) after holding a target at each CURVE_DEG azimuth."""
    m = make_model(spec)
    a = spec.get("arousal", 1.0)
    out = []
    for deg in CURVE_DEG:
        if isinstance(m, PursuitCircuit):
            turn = m.steady_turn(math.radians(float(deg)), arousal=a)
        else:
            m.reset()
            turn = m.step(math.radians(float(deg)), 1.0).turn
        out.append(round(math.degrees(turn), 3))
    return out


def curve_summary(curve: list[float]) -> dict:
    c = np.asarray(curve)
    az = CURVE_DEG
    near = np.abs(az) <= 10
    slope = float(np.polyfit(az[near], c[near], 1)[0])
    nz = np.abs(az) >= 5
    sign_ok = float(np.mean(np.sign(c[nz]) == np.sign(az[nz])))
    anti = np.abs(c + c[::-1])  # CURVE_DEG is symmetric about 0
    return {
        "antisymmetry_error_deg_s": round(float(anti.mean()), 3),
        "antisymmetry_error_max_deg_s": round(float(anti.max()), 3),
        "slope_deg_s_per_deg_within_10deg": round(slope, 3),
        "toward_target_fraction_abs_az_ge_5": round(sign_ok, 3),
        "turn_at_minus_120_deg_s": float(c[az == -120][0]),
        "turn_at_plus_120_deg_s": float(c[az == 120][0]),
        "turn_at_minus_90_deg_s": float(c[az == -90][0]),
        "turn_at_plus_90_deg_s": float(c[az == 90][0]),
        "turn_at_minus_20_deg_s": float(c[az == -20][0]),
        "turn_at_plus_20_deg_s": float(c[az == 20][0]),
        "turn_at_0_deg_s": float(c[az == 0][0]),
        "peak_left_deg": int(az[np.argmax(c)]),
        "peak_right_deg": int(az[np.argmin(c)]),
    }


def _mean(dicts):
    if len(dicts) == 1:
        return dicts[0]
    out = {}
    for k, v in dicts[0].items():
        if isinstance(v, dict):
            out[k] = _mean([d[k] for d in dicts])
        elif isinstance(v, (int, float)) and not isinstance(v, bool) or v is None:
            vals = [d[k] for d in dicts if isinstance(d[k], (int, float))]
            out[k] = float(np.mean(vals)) if vals else None
            if len(vals) > 1:
                out[k + "_sd"] = float(np.std(vals, ddof=1))
        else:
            out[k] = v
    return out


def _f(v, w=8, d=2):
    return f"{'-':>{w}}" if v is None else f"{v:{w}.{d}f}"


def run(rewired_seeds: int = 5, random_seeds: int = 3) -> dict:
    eps = benchmark_set()
    groups = model_specs(rewired_seeds, random_seeds)
    rows = []
    curves = {}
    for name, (desc, specs) in groups.items():
        res = [evaluate_spec(sp, eps) for sp in specs]
        rows.append(
            {
                "model": name,
                "description": desc,
                "n_instances": len(specs),
                "fitted_scalars": int(getattr(make_model(specs[0]), "n_params_fitted", 0)),
                "sim": _mean(res),
            }
        )
        if name not in ("rewired", "random"):
            curves[name] = steering_curve(specs[0])
        else:
            cs = np.array([steering_curve(sp) for sp in specs])
            curves[name] = np.round(cs.mean(0), 3).tolist()
    return {"rows": rows, "curves": curves, "n_episodes": len(eps)}


def _table(rows, scope: str, title: str) -> None:
    print(f"\n{title}")
    hdr = f"{'model':<20}" + "".join(f"{lab:>9}" for _, lab, _ in METRICS)
    if scope == "all":
        hdr += f"{'idle':>7}"
    print(hdr + "\n" + "-" * len(hdr))
    for r in rows:
        d = r["sim"][scope]
        line = f"{r['model']:<20}" + "".join(_f(d[k], 9, n) for k, _, n in METRICS)
        if scope == "all":
            line += _f(d["idle_turn_deg_s"], 7, 1)
        print(line)


def print_tables(res: dict) -> None:
    rows = res["rows"]
    print(
        f"Closed-loop pursuit, {res['n_episodes']} held-out episodes (ASSUMED rig: 15 Hz, "
        "100 ms delay, 2 deg noise, 5 % dropouts, 90 deg/s, 150 ms lag). MAE = mean |bearing "
        "error| deg; <10 = fraction of time within +-10 deg; view = fraction of time in view; "
        "settle/settled/oversh over steps and appearances; idle = turn deg/s with no target"
    )
    _table(rows, "all", "All settings")
    for name in SETTINGS:
        _table(rows, name, f"Field of view +-{name[3:]} deg")
    print("\nMAE per track (all settings)")
    print(f"{'model':<20}" + "".join(f"{k:>9}" for k in KINDS))
    for r in rows:
        print(f"{r['model']:<20}" + "".join(_f(r["sim"][k].get("mae_deg"), 9) for k in KINDS))
    print("\nOpen-loop steering curve: turn command (deg/s, + = left) after 1 s at each azimuth")
    show = [a for a in CURVE_DEG if -120 <= a <= 120 and (a % 15 == 0 or abs(a) <= 10)]
    idx = [int(np.flatnonzero(CURVE_DEG == a)[0]) for a in show]
    print(f"{'model':<20}" + "".join(f"{a:>7}" for a in show) + f"{'antisym':>9}")
    for name, c in res["curves"].items():
        anti = curve_summary(c)["antisymmetry_error_deg_s"]
        print(f"{name:<20}" + "".join(f"{c[i]:7.1f}" for i in idx) + f"{anti:9.1f}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--rewired-seeds", type=int, default=5)
    ap.add_argument("--random-seeds", type=int, default=3)
    ap.add_argument("--no-write", action="store_true")
    args = ap.parse_args(argv)
    t0 = time.time()
    res = run(args.rewired_seeds, args.random_seeds)
    print_tables(res)
    by = {r["model"]: r for r in res["rows"]}

    def pick(fn):
        return {k: (None if fn(v) is None else round(float(fn(v)), 3)) for k, v in by.items()}

    key_numbers = {
        "idle_turn_deg_s": pick(lambda r: r["sim"]["all"]["idle_turn_deg_s"]),
        "loss": pick(lambda r: r["sim"]["all"]["loss"]),
        **{k: pick(lambda r, k=k: r["sim"]["all"][k]) for k, _, _ in METRICS},
        "by_setting": {
            sc: {k: pick(lambda r, k=k, sc=sc: r["sim"][sc][k]) for k, _, _ in METRICS}
            for sc in SETTINGS
        },
        **{f"mae_deg_{k}": pick(lambda r, k=k: r["sim"][k]["mae_deg"]) for k in KINDS},
        "antisymmetry_error_deg_s": {
            k: curve_summary(c)["antisymmetry_error_deg_s"] for k, c in res["curves"].items()
        },
    }
    fits = json.loads((Path(__file__).parent / "pursuit_params.json").read_text())["fits"]
    result = {
        "about": "flybrain_models.pursuit_benchmark output; regenerate with "
        "`uv run python -m flybrain_models.pursuit_benchmark`",
        "date": _dt.date.today().isoformat(),
        "machine": f"{platform.system()} {platform.machine()} python {platform.python_version()}",
        "run_seconds": round(time.time() - t0, 1),
        "key_numbers": key_numbers,
        "key_numbers_doc": {
            "models": "connectome = mirror-averaged PursuitCircuit with the fitted k_out at "
            "arousal 1; connectome_raw = the same without mirror averaging; rewired / random = "
            "mean over 5 / 3 control seeds; no_AOTU019 / no_AOTU025 = that type silenced; "
            "AOTU019_excitatory = AOTU019 sign flipped to +1; arousal_0 = connectome at "
            "arousal 0; all with the connectome's k_out, none refitted. proportional and "
            "bang_bang = baselines. Every model has one fitted scalar",
            "idle_turn_deg_s": "|turn rate| after 2 s with no target reported (after the "
            "90 deg/s limit)",
            "loss": "the fitting loss on held-out data: mae_deg + 1.0 * idle_turn_deg_s",
            "mae_deg": "time-weighted mean |bearing error| (deg) over all held-out episodes "
            "(the fitting loss, on held-out data)",
            "within10": "fraction of time with |bearing error| <= 10 deg",
            "in_view": "fraction of time with the target inside the field of view",
            "step_settle_s_median": "median time after a step or appearance until |error| "
            "stays <= 5 deg (episodes that never settle excluded; see step_settled_fraction)",
            "step_settled_fraction": "fraction of steps and appearances that settle",
            "step_overshoot_deg_mean": "mean largest error past the target after a step or "
            "appearance",
            "by_setting": "the same metrics per field-of-view setting (+-30, +-60, +-100 deg)",
            "mae_deg_{step,appear,sine,walk,pass}": "MAE per target track, all settings",
            "antisymmetry_error_deg_s": "steering curve: mean |turn(az) + turn(-az)| over the "
            "sampled azimuths -150..150 deg",
        },
        "protocol": {
            "n_episodes": res["n_episodes"],
            "rig": "ASSUMED: 15 Hz frames, 100 ms delay, 2 deg Gaussian noise, field of view "
            "+-30, +-60 and +-100 deg (+-100 = head camera +-30 on a neck panning about +-70; "
            "the head loop is not simulated), 5 % dropouts, 90 deg/s turn limit, 150 ms "
            "first-order lag",
            "tracks": "steps of 0.35/0.65/0.9 of the half field of view both ways; at +-100 "
            "also appearances at 60-95 deg; sinusoids 0.05-0.25 Hz, 15-40 deg (40-90 at "
            "+-100); OU random walks (tau 1 s, sd 30 deg/s); passes at 60-100 and 120-180 "
            "deg/s, 120 deg from straight ahead (+-30, +-60) or from 90 deg on one side to "
            "90 deg on the other (+-100)",
            "steering_curve": "turn command after holding a target at each azimuth for 1 s, "
            "open loop, from rest; rewired/random averaged over seeds",
        },
        "params": {k: v["params"] for k, v in fits.items()},
        "fits": {
            k: {kk: v[kk] for kk in ("train_loss", "date", "fit_seconds") if kk in v}
            for k, v in fits.items()
        },
        "steering_curve_azimuth_deg": CURVE_DEG.tolist(),
        "steering_curve_deg_s": res["curves"],
        "steering_curve_summary": {k: curve_summary(c) for k, c in res["curves"].items()},
        "results": res["rows"],
    }
    if not args.no_write:
        results_path().write_text(json.dumps(result, indent=1, default=float) + "\n")
        print(f"\nwrote {results_path()}")
    print(f"benchmark took {result['run_seconds']} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
