"""Benchmark the fly compass against controls, an ablation and engineering baselines.

    uv run python -m flybrain_models.compass_benchmark            # prints tables, writes JSON
    uv run python -m flybrain_models.compass_benchmark --no-write

Four tests, all on simulated data:

A. Dead reckoning, no cue: integrate the gyro alone over ``compass_tasks.benchmark_set``
   (slow 30 deg/s and fast 180 deg/s turns both ways, stop-and-go to 200 deg/s, a 40 s random
   sequence; each clean and with gyro noise 10 deg/s white + 0.5 deg/s bias).
B. Cue tracking: the same six clean trajectories with a landmark cue (true heading) at 5 Hz,
   without and with 5 deg noise on the cue. Mean abs error and lag.
C. Heading memory (the key test): 24 headings; the bump starts 90 deg away, the cue is given
   at 5 Hz for 3 s, then darkness with the robot stationary for 5, 30 and 60 s.
D. Memory while rotating in darkness: as C, but the robot turns at 90 deg/s for 4 s in the
   dark (a dead-reckoning test).

Results go to ``compass_benchmark.json`` in this package; ``key_numbers`` holds stable keys.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import platform
import time
from importlib import resources
from pathlib import Path

import numpy as np

from .compass import GyroIntegrator, RingAttractor, load_params, params_path
from .compass_tasks import benchmark_set, memory_trial, run, stationary, with_cue

CONF_THRESHOLD = 0.3
HELD_DEG = 30.0  # a memory trial "holds" if a bump is present and within this of the cue
RESULTS_FILE = "compass_benchmark.json"
MEMORY_HEADINGS = np.linspace(-np.pi, np.pi, 24, endpoint=False)
DARK_S = (5.0, 30.0, 60.0)


def results_path() -> Path:
    return Path(str(resources.files("flybrain_models") / RESULTS_FILE))


class StaticHeading(GyroIntegrator):
    """Null reference: ignores the gyro and the cue, keeps the reset heading."""

    def step(self, angular_velocity_rad_s: float, dt: float, cue_rad=None) -> float:
        return self.heading


# ---- A. dead reckoning ---------------------------------------------------------------------
def dead_reckoning(model, trajs, stat_clean, stat_noisy) -> dict:
    errs, coh, n_steps, t_total = [], [], 0, 0.0
    names = []
    for tr in trajs:
        t0 = time.perf_counter()
        out = run(model, tr, CONF_THRESHOLD)
        t_total += time.perf_counter() - t0
        n_steps += len(tr.omega)
        errs.append(np.degrees(np.abs(out["error"])))
        coh.append(out["coherent"])
        names.append(tr.name)
    drift = {}
    for st in (stat_clean, stat_noisy):
        out = run(model, st, CONF_THRESHOLD)
        dec = np.unwrap(out["decoded"])
        drift[st.name] = float(abs(np.degrees(dec[-1] - dec[0])) / (st.duration / 60.0))
    clean = [e for n, e in zip(names, errs, strict=True) if "noise" not in n]
    noisy = [e for n, e in zip(names, errs, strict=True) if "noise" in n]
    return {
        "mae_deg": float(np.concatenate(errs).mean()),
        "mae_clean_deg": float(np.concatenate(clean).mean()),
        "mae_noisy_deg": float(np.concatenate(noisy).mean()),
        "coherent_frac": float(np.concatenate(coh).mean()),
        "drift_deg_per_min": drift["stationary"],
        "drift_noisy_deg_per_min": drift["stationary+noise"],
        "us_per_step": 1e6 * t_total / n_steps,
    }


def velocity_response(model) -> dict:
    """Bump speed / commanded speed over 1-3 s of constant rotation (mean of 4 starts)."""
    ratios = {}
    for wdeg in (30, 90, 180, -30, -90, -180):
        r = []
        for h in (0.0, 1.6, 3.2, 4.8):
            model.reset(h)
            hs = [model.step(np.radians(wdeg), 0.02) for _ in range(150)]
            u = np.unwrap(np.array([h, *hs]))
            r.append(np.degrees(u[-1] - u[50]) / 2.0 / wdeg)
        ratios[str(wdeg)] = float(np.mean(r))
    return {
        "speed_ratio_90": (ratios["90"] + ratios["-90"]) / 2,
        "speed_ratio_180": (ratios["180"] + ratios["-180"]) / 2,
    }


# ---- B. cue tracking -----------------------------------------------------------------------
def cue_tracking(model, trajs, noise_deg: float) -> dict:
    errs, lags, coh = [], [], []
    for i, tr in enumerate(trajs):
        out = run(model, with_cue(tr, 5.0, noise_deg, seed=100 + i), CONF_THRESHOLD)
        errs.append(np.degrees(np.abs(out["error"])))
        coh.append(out["coherent"])
        dec, true = out["decoded"], tr.heading
        shifts = np.arange(0, 51)  # 0..1 s
        cost = [
            np.abs(np.angle(np.exp(1j * (dec[s:] - true[: len(true) - s])))).mean() for s in shifts
        ]
        lags.append(shifts[int(np.argmin(cost))] * tr.dt * 1000.0)
    return {
        "mae_deg": float(np.concatenate(errs).mean()),
        "lag_ms": float(np.mean(lags)),
        "coherent_frac": float(np.concatenate(coh).mean()),
    }


# ---- C/D. memory -----------------------------------------------------------------------------
def memory(model, omega_dark: float = 0.0, dark_s=DARK_S) -> dict:
    trials = [
        memory_trial(model, float(h), cue_s=3.0, dark_s=dark_s, omega_dark=omega_dark)
        for h in MEMORY_HEADINGS
    ]
    out = {}
    for key in ("cue_end", *dark_s):
        err = np.degrees(np.abs([t[key][0] for t in trials]))
        conf = np.array([t[key][1] for t in trials])
        bump = conf > CONF_THRESHOLD
        tag = "cue_end" if key == "cue_end" else f"{key:g}s"
        out[tag] = {
            "held_frac": float(np.mean(bump & (err < HELD_DEG))),
            "bump_frac": float(np.mean(bump)),
            # trials without a bump count as chance (90 deg): the decoder would only be
            # repeating its last value
            "err_deg": float(np.mean(np.where(bump, err, 90.0))),
            "err_deg_raw": float(np.mean(err)),
            "confidence": float(conf.mean()),
        }
    return out


def _mean(rows: list[dict]) -> dict:
    """Average nested dicts of floats across model instances (adds *_sd for scalars)."""
    out = {}
    for k, v in rows[0].items():
        if isinstance(v, dict):
            out[k] = _mean([r[k] for r in rows])
        else:
            vals = np.array([r[k] for r in rows], float)
            out[k] = round(float(vals.mean()), 3)
            if len(rows) > 1:
                out[k + "_sd"] = round(float(vals.std(ddof=1)), 3)
    return out


def build_models(rewired_seeds: int, random_seeds: int):
    fits = json.loads(params_path().read_text())["fits"]

    def ring(variant, seed, params):
        return RingAttractor.from_pack("central_complex", variant=variant, seed=seed, params=params)

    shipped = load_params("connectome")
    groups = {
        "connectome": ("FlyWire connectome, shipped scalars", [ring("connectome", 0, shipped)]),
        "rewired": (
            (
                f"degree-preserving rewire within type blocks, x{rewired_seeds} seeds, "
                "connectome scalars"
            ),
            [ring("rewired", s, shipped) for s in range(rewired_seeds)],
        ),
        "random": (
            (
                f"random graph, same size/density/sign ratio, x{random_seeds} seeds, "
                "connectome scalars"
            ),
            [ring("random", s, shipped) for s in range(random_seeds)],
        ),
        "no_recurrence": (
            (
                "ablation: same neurons, leak, biases and cue input, all recurrent "
                "weights set to zero"
            ),
            [ring("connectome", 0, {**shipped, "ablate_recurrence": True})],
        ),
    }
    if "connectome_dead_reckoning" in fits:
        groups["connectome_dead_reckoning_fit"] = (
            "FlyWire connectome, earlier velocity-only fit (plain summed weights, no cue fit)",
            [ring("connectome", 0, "connectome_dead_reckoning")],
        )
    for key in sorted(fits):
        if key.startswith(("rewired_seed", "random_seed")) and not key.endswith("_base"):
            variant, seed = key.split("_seed")
            seed = seed.split("_")[0]
            groups[f"{key}_refit"] = (
                (
                    f"{variant} control (seed {seed}) with scalars "
                    f"re-fitted on it ({fits[key].get('objective', 'traj')})"
                ),
                [ring(variant, int(seed), key)],
            )
    groups["complementary_filter"] = (
        "engineering baseline: integrate gyro, pull 30% toward each cue sample",
        [GyroIntegrator()],
    )
    groups["static_null"] = (
        "null: ignores gyro and cue, keeps the start heading",
        [StaticHeading()],
    )
    return groups


def _fmt(v, sd=None, w=7, d=1):
    s = f"{v:.{d}f}" + (f"±{sd:.0f}" if sd is not None and sd >= 0.5 else "")
    return f"{s:>{w}}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--rewired-seeds", type=int, default=5)
    ap.add_argument("--random-seeds", type=int, default=3)
    ap.add_argument("--no-write", action="store_true")
    args = ap.parse_args(argv)

    trajs = benchmark_set()
    clean = [t for t in trajs if "noise" not in t.name]
    st_clean, st_noisy = stationary(), stationary(noisy=True)
    groups = build_models(args.rewired_seeds, args.random_seeds)

    rows = []
    for name, (desc, models) in groups.items():
        per = []
        for m in models:
            per.append(
                {
                    "dead_reckoning": {
                        **dead_reckoning(m, trajs, st_clean, st_noisy),
                        **(
                            velocity_response(m)
                            if isinstance(m, RingAttractor)
                            else {
                                "speed_ratio_90": 1.0 * (name != "static_null"),
                                "speed_ratio_180": 1.0 * (name != "static_null"),
                            }
                        ),
                    },
                    "cue_tracking": cue_tracking(m, clean, 0.0),
                    "cue_tracking_noisy": cue_tracking(m, clean, 5.0),
                    "memory": memory(m),
                    "dark_rotation": memory(m, omega_dark=np.radians(90.0), dark_s=(4.0,)),
                }
            )
        m0 = models[0]
        rows.append(
            {
                "model": name,
                "description": desc,
                "n_instances": len(models),
                "n_neurons": int(getattr(m0, "n_neurons", 0)),
                "fitted_scalars": int(getattr(m0, "n_params_fitted", 0)),
                "connectome_weights": int(getattr(m0, "n_synapse_weights", 0)),
                **_mean(per),
            }
        )

    def table(title, cols):
        print(f"\n{title}")
        hdr = f"{'model':<26}" + "".join(f"{c[0]:>12}" for c in cols)
        print(hdr)
        print("-" * len(hdr))
        for r in rows:
            cells = []
            for _, path in cols:
                node = r
                for k in path[:-1]:
                    node = node[k]
                cells.append(_fmt(node[path[-1]], node.get(path[-1] + "_sd"), 12))
            print(f"{r['model']:<26}" + "".join(cells))

    table(
        "A. Dead reckoning, no cue (errors deg, drift deg/min, speed = bump/commanded)",
        [
            ("MAE", ("dead_reckoning", "mae_deg")),
            ("MAE noisy", ("dead_reckoning", "mae_noisy_deg")),
            ("bump frac", ("dead_reckoning", "coherent_frac")),
            ("drift", ("dead_reckoning", "drift_deg_per_min")),
            ("speed@90", ("dead_reckoning", "speed_ratio_90")),
            ("speed@180", ("dead_reckoning", "speed_ratio_180")),
            ("us/step", ("dead_reckoning", "us_per_step")),
        ],
    )
    table(
        "B. Cue tracking, 5 Hz cue (MAE deg, lag ms)",
        [
            ("MAE", ("cue_tracking", "mae_deg")),
            ("lag", ("cue_tracking", "lag_ms")),
            ("MAE 5deg", ("cue_tracking_noisy", "mae_deg")),
            ("lag 5deg", ("cue_tracking_noisy", "lag_ms")),
        ],
    )
    table(
        "C. Heading memory, cue 3 s then darkness at rest (held = bump within 30 deg; "
        "err deg, no-bump trials count as 90)",
        [
            ("held@cue", ("memory", "cue_end", "held_frac")),
            ("held@5s", ("memory", "5s", "held_frac")),
            ("held@30s", ("memory", "30s", "held_frac")),
            ("held@60s", ("memory", "60s", "held_frac")),
            ("err@5s", ("memory", "5s", "err_deg")),
            ("err@30s", ("memory", "30s", "err_deg")),
            ("err@60s", ("memory", "60s", "err_deg")),
            ("bump@60s", ("memory", "60s", "bump_frac")),
        ],
    )
    table(
        "D. Darkness while turning 90 deg/s for 4 s (dead reckoning)",
        [
            ("held", ("dark_rotation", "4s", "held_frac")),
            ("err", ("dark_rotation", "4s", "err_deg")),
            ("bump", ("dark_rotation", "4s", "bump_frac")),
        ],
    )

    by = {r["model"]: r for r in rows}
    cmp = (
        "connectome",
        "rewired",
        "random",
        "no_recurrence",
        "complementary_filter",
        "static_null",
    )

    def pick(path):
        out = {}
        for k in cmp:
            node = by[k]
            for p in path:
                node = node[p]
            out[k] = round(float(node), 3)
        return out

    key_numbers = {
        "dead_reckoning_mae_deg": pick(("dead_reckoning", "mae_deg")),
        "cue_tracking_mae_deg": pick(("cue_tracking", "mae_deg")),
        "cue_tracking_noisy_mae_deg": pick(("cue_tracking_noisy", "mae_deg")),
        "cue_tracking_lag_ms": pick(("cue_tracking", "lag_ms")),
        **{f"memory_held_frac_{t}": pick(("memory", t, "held_frac")) for t in ("5s", "30s", "60s")},
        **{f"memory_err_deg_{t}": pick(("memory", t, "err_deg")) for t in ("5s", "30s", "60s")},
        "memory_bump_frac_60s": pick(("memory", "60s", "bump_frac")),
        "dark_rotation_err_deg": pick(("dark_rotation", "4s", "err_deg")),
        "dark_rotation_held_frac": pick(("dark_rotation", "4s", "held_frac")),
    }
    result = {
        "about": "flybrain_models.compass_benchmark output; regenerate with "
        "`uv run python -m flybrain_models.compass_benchmark`",
        "date": _dt.date.today().isoformat(),
        "machine": f"{platform.system()} {platform.machine()} python {platform.python_version()}",
        "key_numbers": key_numbers,
        "key_numbers_doc": {
            "models": "connectome = shipped RingAttractor; rewired / random = mean over 5 / 3 "
            "control seeds with the connectome's scalars; no_recurrence = ablation "
            "(W = 0); complementary_filter = GyroIntegrator; static_null = never moves",
            "dead_reckoning_mae_deg": "test A: mean abs heading error, gyro only (deg)",
            "cue_tracking_mae_deg": "test B: mean abs error with an exact 5 Hz cue (deg)",
            "cue_tracking_noisy_mae_deg": "test B with 5 deg sd noise on the cue (deg)",
            "cue_tracking_lag_ms": "test B: delay that best aligns decoded and true heading",
            "memory_held_frac_{5s,30s,60s}": "test C: fraction of 24 headings where a bump "
            "(confidence > 0.3) is within 30 deg of the last "
            "cue after that long in darkness at rest",
            "memory_err_deg_{5s,30s,60s}": "test C: mean abs error vs the last cue; trials "
            "without a bump count as 90 deg (chance)",
            "memory_bump_frac_60s": "test C: fraction of trials with any bump after 60 s",
            "dark_rotation_err_deg": "test D: error after turning 90 deg/s for 4 s in darkness",
            "dark_rotation_held_frac": "test D: fraction within 30 deg with a bump",
        },
        "protocols": {
            "cue": "landmark = true heading, sampled at 5 Hz; RingAttractor holds each sample "
            "for 0.25 s (CUE_HOLD_S), so the stream acts as a continuous cue",
            "memory": "reset 90 deg away from the target, cue 3 s at 5 Hz, then no cue",
            "trajectories": [
                {
                    "name": t.name,
                    "duration_s": round(t.duration, 2),
                    "max_abs_deg_s": round(float(np.degrees(np.abs(t.omega).max())), 1),
                }
                for t in trajs
            ],
        },
        "params_connectome": load_params("connectome"),
        "results": rows,
    }
    if not args.no_write:
        results_path().write_text(json.dumps(result, indent=1) + "\n")
        print(f"\nwrote {results_path()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
