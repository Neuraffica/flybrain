"""Benchmark the fly escape circuit against controls, an ablation and two baselines.

    uv run python -m flybrain_models.escape_benchmark            # prints tables, writes JSON
    uv run python -m flybrain_models.escape_benchmark --no-write

Simulated episodes (``escape_tasks.benchmark_set``, 384 episodes): approaches at 0.15-1.5 m/s
from the left, the right and head-on; fast hands stopping short inside the sensor band; slow
drifts to the sensor; hovering with tremor; a hand moving away; empty scene. Each at 12 Hz and
28 Hz with +-25% sample jitter, with normal (sd 25) and high (sd 80) sensor noise. Plus the
real 25 s USB recording with five hand approaches (labelled by the offline rule in
``escape_tasks.label_recording``). Results go to ``escape_benchmark.json``; ``key_numbers``
holds stable keys documented in ``key_numbers_doc``.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import platform
import time
from concurrent.futures import ProcessPoolExecutor
from importlib import resources
from pathlib import Path

import numpy as np

from .escape import (
    EscapeCircuit,
    LoomingRateThreshold,
    ProximityThreshold,
    load_params,
    params_path,
)
from .escape_tasks import RECORDINGS, benchmark_set, run_episode, run_recording, score_episode

RESULTS_FILE = "escape_benchmark.json"
SPEED_BINS = ((0.15, 0.3), (0.3, 0.5), (0.5, 0.8), (0.8, 1.6))


def results_path() -> Path:
    return Path(str(resources.files("flybrain_models") / RESULTS_FILE))


def detailed(model, episodes) -> dict:
    t0 = time.perf_counter()
    n_steps = 0
    rows = []
    for ep in episodes:
        res = run_episode(model, ep)
        n_steps += len(ep.t)
        s = score_episode(ep, res)
        s.update(
            category=ep.category,
            kind=ep.kind,
            side=ep.side,
            speed=ep.speed,
            rate=ep.meta["rate_hz"],
            noise=ep.meta["noise"],
            threat=ep.threat,
            n_trig=len(res["triggers"]),
        )
        rows.append(s)
    us = 1e6 * (time.perf_counter() - t0) / n_steps

    def summ(sel):
        thr = [r for r in sel if r["hit"] is not None]
        hits = [r for r in thr if r["hit"]]
        lat = [1000 * r["latency"] for r in hits]
        lateral = [r for r in hits if r["side"] != 0]
        head = [r for r in hits if r["side"] == 0 and r["kind"] == "approach"]
        mins = sum(r["scored_s"] for r in sel) / 60
        return {
            "hit_rate": len(hits) / len(thr) if thr else None,
            "fa_per_min": sum(r["false_alarms"] for r in sel) / mins if mins else None,
            "latency_ms_mean": float(np.mean(lat)) if lat else None,
            "latency_ms_median": float(np.median(lat)) if lat else None,
            "side_correct": float(np.mean([r["side_ok"] for r in lateral])) if lateral else None,
            "side_correct_gf_readout": (
                float(np.mean([r["side_gf_ok"] for r in lateral])) if lateral else None
            ),
            "headon_side_zero": (float(np.mean([r["side_ok"] for r in head])) if head else None),
        }

    out = {"all": summ(rows), "us_per_step": us}
    out["hit_rate_by_side"] = {}
    for cat in ("left", "right", "head_on", "left_only", "right_only"):
        sel = [r for r in rows if r["category"] == cat and r["hit"] is not None]
        out["hit_rate_by_side"][cat] = sum(r["hit"] for r in sel) / len(sel) if sel else None
    for cond in ("12Hz", "28Hz"):
        out[cond] = summ([r for r in rows if f"{r['rate']:g}Hz" == cond])
    for nz in ("normal", "noisy"):
        out[f"noise_{nz}"] = summ([r for r in rows if r["noise"] == nz])
    out["fa_per_min_by_kind"] = {}
    for kind in ("stop_short", "hover", "drift", "retreat", "noise"):
        sel = [r for r in rows if r["kind"] == kind]
        mins = sum(r["scored_s"] for r in sel) / 60
        out["fa_per_min_by_kind"][kind] = sum(r["false_alarms"] for r in sel) / mins
    curve = {}
    for lo, hi in SPEED_BINS:
        sel = [r for r in rows if r["kind"] == "approach" and lo <= r["speed"] < hi]
        trig = [r for r in sel if r["first_trigger"] is not None]
        hits = [r for r in sel if r["hit"]]
        curve[f"{lo:g}-{hi:g}"] = {
            "trigger_prob": len(trig) / len(sel),
            "latency_ms": (float(np.mean([1000 * r["latency"] for r in hits])) if hits else None),
            "distance_cm_at_trigger": (
                float(np.mean([100 * r["dist_at_trigger"] for r in hits])) if hits else None
            ),
        }
    out["speed_curve"] = curve
    return out


def recording(model, name: str) -> dict:
    r = run_recording(model, name)
    ev = r["events"]
    lat = [e["latency_ms"] for e in ev if e["hit"]]

    def hits(sel):
        sel = list(sel)
        return {"hits": sum(e["hit"] for e in sel), "events": len(sel)}

    return {
        "hits": sum(e["hit"] for e in ev),
        "events": len(ev),
        "latency_ms_mean": float(np.mean(lat)) if lat else None,
        "false_alarms": len(r["false_alarm_times"]),
        "fa_per_min": len(r["false_alarm_times"]) / r["quiet_minutes"],
        # per side: the channel that saw the hand first (pin 36 = left)
        "left": hits(e for e in ev if e["leading_side"] == 1),
        "right": hits(e for e in ev if e["leading_side"] == -1),
        "left_only": hits(e for e in ev if e["leading_side"] == 1 and e["one_channel_only"]),
        "right_only": hits(e for e in ev if e["leading_side"] == -1 and e["one_channel_only"]),
        "side_matches_leading_channel": sum(
            1 for e in ev if e["hit"] and e["leading_side"] and e["side"] == e["leading_side"]
        ),
        "events_with_leading_channel": sum(1 for e in ev if e["hit"] and e["leading_side"]),
        "detail": ev,
    }


def structure(model) -> dict:
    """What the wiring gives the GF, independent of fitted scalars: direct synapses from
    LC4 and LPLC2 per GF side, split ipsilateral/contralateral, and the two-step synapse
    paths through the interneurons (excitatory and inhibitory)."""
    pk = getattr(model, "pack", None)
    if pk is None:
        return {}
    ct, sd, C, sg = pk.cell_type, pk.side, pk.counts.astype(float), pk.sign
    out = {}
    for g in ("left", "right"):
        gi = np.flatnonzero((ct == "DNp01") & (sd == g))[0]
        for t in ("LC4", "LPLC2"):
            ipsi = np.flatnonzero((ct == t) & (sd == g))
            contra = np.flatnonzero((ct == t) & (sd != g))
            out[f"{t}->GF_{g[0].upper()}_ipsi"] = int(C[gi, ipsi].sum())
            out[f"{t}->GF_{g[0].upper()}_contra"] = int(C[gi, contra].sum())
    vpn = np.flatnonzero(np.isin(ct, ["LC4", "LPLC2"]))
    inn = model._interneurons
    gfs = model._gf
    two = (C[np.ix_(gfs, inn)] * sg[inn]) @ C[np.ix_(inn, vpn)] / 100.0
    if model.params.get("ablate_interneurons"):
        two = np.zeros_like(two)
    out["two_step_via_interneurons_exc"] = float(two[two > 0].sum())
    out["two_step_via_interneurons_inh"] = float(-two[two < 0].sum())
    lc4 = sum(out[f"LC4->GF_{s}_ipsi"] + out[f"LC4->GF_{s}_contra"] for s in "LR")
    lp = sum(out[f"LPLC2->GF_{s}_ipsi"] + out[f"LPLC2->GF_{s}_contra"] for s in "LR")
    ipsi = sum(out[k] for k in out if k.endswith("_ipsi"))
    out["LC4_to_LPLC2_ratio_at_GF"] = lc4 / lp if lp else None
    out["ipsilateral_fraction"] = ipsi / max(lc4 + lp, 1)
    return out


def _mean(dicts):
    if len(dicts) == 1:
        return dicts[0]
    out = {}
    for k, v in dicts[0].items():
        if isinstance(v, dict):
            out[k] = _mean([d[k] for d in dicts])
        elif isinstance(v, (int, float)) and not isinstance(v, bool):
            vals = [d[k] for d in dicts if d[k] is not None]
            out[k] = float(np.mean(vals)) if vals else None
            if len(vals) > 1:
                out[k + "_sd"] = float(np.std(vals, ddof=1))
        elif v is None:
            vals = [d[k] for d in dicts if isinstance(d[k], (int, float))]
            out[k] = float(np.mean(vals)) if vals else None
        else:
            out[k] = v
    return out


def build_models(rewired_seeds: int, random_seeds: int) -> dict:
    """name -> (description, [model specs]); a spec is (kind, variant, seed, params)."""
    fits = json.loads(params_path().read_text())["fits"]
    shipped = load_params("connectome")
    g = {
        "connectome": ("FlyWire wiring, shipped scalars", [("esc", "connectome", 0, shipped)]),
        "rewired": (
            f"degree-preserving rewire within type blocks, x{rewired_seeds}, connectome scalars",
            [("esc", "rewired", s, shipped) for s in range(rewired_seeds)],
        ),
        "random": (
            f"random graph, same size/density/sign ratio, x{random_seeds}, connectome scalars",
            [("esc", "random", s, shipped) for s in range(random_seeds)],
        ),
        "no_interneurons": (
            "ablation: interneurons removed, connectome scalars",
            [("esc", "connectome", 0, {**shipped, "ablate_interneurons": True})],
        ),
    }
    for key in ("no_interneurons", "rewired_seed0", "random_seed0"):
        if key in fits:
            variant = key.split("_seed")[0] if "_seed" in key else "connectome"
            seed = int(key.split("_seed")[1]) if "_seed" in key else 0
            g[f"{key}_refit"] = (
                f"{key} with its own fitted scalars",
                [("esc", variant, seed, load_params(key))],
            )
    g["proximity_threshold"] = (
        "baseline: max(prox) crosses a threshold",
        [("prox", None, 0, None)],
    )
    g["looming_rate_threshold"] = (
        "baseline: d(prox)/dt crosses a threshold",
        [("loom", None, 0, None)],
    )
    return g


def _make(spec):
    kind, variant, seed, params = spec
    if kind == "prox":
        return ProximityThreshold()
    if kind == "loom":
        return LoomingRateThreshold()
    return EscapeCircuit.from_pack("escape", variant=variant, seed=seed, params=params)


def _evaluate_spec(spec) -> dict:
    m = _make(spec)
    eps = benchmark_set()
    return {
        "sim": detailed(m, eps),
        "recording": {n: recording(m, n) for n in RECORDINGS},
        "structure": structure(m) if isinstance(m, EscapeCircuit) else {},
        "n_neurons": int(getattr(m, "n_neurons", 0)),
        "fitted_scalars": int(getattr(m, "n_params_fitted", 0)),
    }


def _f(v, w=9, d=2):
    return f"{'-':>{w}}" if v is None else f"{v:{w}.{d}f}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--rewired-seeds", type=int, default=5)
    ap.add_argument("--random-seeds", type=int, default=3)
    ap.add_argument("--no-write", action="store_true")
    ap.add_argument("--workers", type=int, default=None)
    args = ap.parse_args(argv)
    eps = benchmark_set()
    groups = build_models(args.rewired_seeds, args.random_seeds)
    specs = [(name, spec) for name, (_, sp) in groups.items() for spec in sp]
    workers = args.workers or min(8, os.cpu_count() or 1)
    with ProcessPoolExecutor(workers) as ex:
        results = list(ex.map(_evaluate_spec, [sp for _, sp in specs]))
    rows = []
    for name, (desc, sp) in groups.items():
        res = [r for (n, _), r in zip(specs, results, strict=True) if n == name]
        rows.append(
            {
                "model": name,
                "description": desc,
                "n_instances": len(sp),
                "n_neurons": res[0]["n_neurons"],
                "fitted_scalars": res[0]["fitted_scalars"],
                "sim": _mean([r["sim"] for r in res]),
                "recording": {n: _mean([r["recording"][n] for r in res]) for n in RECORDINGS},
                "structure": _mean([r["structure"] for r in res]) if res[0]["structure"] else {},
            }
        )

    print(
        "Simulated approaches (all conditions). FA = false alarms per minute of non-threat "
        "time; latency vs the hand entering sensor range"
    )
    hdr = (
        f"{'model':<26}{'hit':>7}{'FA/min':>8}{'lat ms':>8}{'side':>7}{'12Hz hit':>9}"
        f"{'12Hz FA':>8}{'noisyFA':>8}{'us/step':>9}"
    )
    print(hdr + "\n" + "-" * len(hdr))
    for r in rows:
        s = r["sim"]
        print(
            f"{r['model']:<26}{_f(s['all']['hit_rate'], 7)}{_f(s['all']['fa_per_min'], 8)}"
            f"{_f(s['all']['latency_ms_mean'], 8, 0)}{_f(s['all']['side_correct'], 7)}"
            f"{_f(s['12Hz']['hit_rate'], 9)}{_f(s['12Hz']['fa_per_min'], 8)}"
            f"{_f(s['noise_noisy']['fa_per_min'], 8)}{_f(s['us_per_step'], 9, 0)}"
        )
    print("\nFalse alarms per minute by non-threat kind")
    kinds = ("stop_short", "hover", "drift", "retreat", "noise")
    print(f"{'model':<26}" + "".join(f"{k:>11}" for k in kinds))
    for r in rows:
        print(
            f"{r['model']:<26}" + "".join(_f(r["sim"]["fa_per_min_by_kind"][k], 11) for k in kinds)
        )
    print("\nSpeed dependence: P(trigger) / latency ms / hand distance cm at trigger")
    bins = list(rows[0]["sim"]["speed_curve"])
    print(f"{'model':<26}" + "".join(f"{b + ' m/s':>20}" for b in bins))
    for r in rows:
        cells = []
        for b in bins:
            c = r["sim"]["speed_curve"][b]
            lat = "-" if c["latency_ms"] is None else f"{c['latency_ms']:.0f}"
            dist = (
                "-" if c["distance_cm_at_trigger"] is None else f"{c['distance_cm_at_trigger']:.1f}"
            )
            cells.append(f"{c['trigger_prob']:.2f}/{lat}/{dist}".rjust(20))
        print(f"{r['model']:<26}" + "".join(cells))
    print("\nHit rate per side (simulated). *_only = the other channel never sees the hand")
    cats = ("left", "right", "head_on", "left_only", "right_only")
    print(f"{'model':<26}" + "".join(f"{c:>11}" for c in cats))
    for r in rows:
        print(f"{r['model']:<26}" + "".join(_f(r["sim"]["hit_rate_by_side"][c], 11) for c in cats))
    for rec_name, title in (
        ("hand_usb", "Real recording 1: desk, 25 s, 5 approaches, USB ~28 Hz"),
        ("robot_run", "Real recording 2: robot escape run, 45 s, 9 approaches, USB ~27 Hz"),
    ):
        print(f"\n{title} (pin 36 = left)")
        print(
            f"{'model':<26}{'hits':>9}{'left':>9}{'right':>9}{'L-only':>9}{'R-only':>9}"
            f"{'lat ms':>8}{'FA':>5}{'side=lead':>11}"
        )
        for r in rows:
            c = r["recording"][rec_name]

            def hf(d):
                return f"{d['hits']:.1f}/{d['events']:.0f}".rjust(9)

            print(
                f"{r['model']:<26}{c['hits']:>5.1f}/{c['events']:<3.0f}{hf(c['left'])}"
                f"{hf(c['right'])}{hf(c['left_only'])}{hf(c['right_only'])}"
                f"{_f(c['latency_ms_mean'], 8, 0)}{c['false_alarms']:>5.1f}"
                f"{c['side_matches_leading_channel']:>7.1f}/{c['events_with_leading_channel']:.0f}"
            )
    print("\nWiring seen by the GF (synapses, after threshold)")
    for r in rows:
        st = r["structure"]
        if st:
            print(
                f"{r['model']:<26}LC4:LPLC2 {st['LC4_to_LPLC2_ratio_at_GF']:.2f}  ipsilateral "
                f"{st['ipsilateral_fraction']:.2f}  via interneurons exc "
                f"{st['two_step_via_interneurons_exc']:.0f} inh "
                f"{st['two_step_via_interneurons_inh']:.0f}"
            )

    by = {r["model"]: r for r in rows}
    cmp = (
        "connectome",
        "rewired",
        "random",
        "no_interneurons",
        "proximity_threshold",
        "looming_rate_threshold",
    )

    def pick(fn):
        return {k: (None if fn(by[k]) is None else round(float(fn(by[k])), 3)) for k in cmp}

    key_numbers = {
        "sim_hit_rate": pick(lambda r: r["sim"]["all"]["hit_rate"]),
        "sim_fa_per_min": pick(lambda r: r["sim"]["all"]["fa_per_min"]),
        "sim_latency_ms": pick(lambda r: r["sim"]["all"]["latency_ms_mean"]),
        "sim_side_correct": pick(lambda r: r["sim"]["all"]["side_correct"]),
        "sim_side_correct_gf_readout": pick(lambda r: r["sim"]["all"]["side_correct_gf_readout"]),
        "sim_hit_rate_12hz": pick(lambda r: r["sim"]["12Hz"]["hit_rate"]),
        "sim_fa_per_min_noisy": pick(lambda r: r["sim"]["noise_noisy"]["fa_per_min"]),
        "sim_fa_per_min_drift": pick(lambda r: r["sim"]["fa_per_min_by_kind"]["drift"]),
        "sim_fa_per_min_hover": pick(lambda r: r["sim"]["fa_per_min_by_kind"]["hover"]),
        "sim_fa_per_min_stop_short": pick(lambda r: r["sim"]["fa_per_min_by_kind"]["stop_short"]),
        "sim_trigger_distance_cm_fast": pick(
            lambda r: r["sim"]["speed_curve"]["0.8-1.6"]["distance_cm_at_trigger"]
        ),
        "sim_trigger_distance_cm_slow": pick(
            lambda r: r["sim"]["speed_curve"]["0.3-0.5"]["distance_cm_at_trigger"]
        ),
        "sim_hit_rate_left": pick(lambda r: r["sim"]["hit_rate_by_side"]["left"]),
        "sim_hit_rate_right": pick(lambda r: r["sim"]["hit_rate_by_side"]["right"]),
        "sim_hit_rate_head_on": pick(lambda r: r["sim"]["hit_rate_by_side"]["head_on"]),
        "sim_hit_rate_left_only": pick(lambda r: r["sim"]["hit_rate_by_side"]["left_only"]),
        "sim_hit_rate_right_only": pick(lambda r: r["sim"]["hit_rate_by_side"]["right_only"]),
        "real_hits_of_5": pick(lambda r: r["recording"]["hand_usb"]["hits"]),
        "real_latency_ms": pick(lambda r: r["recording"]["hand_usb"]["latency_ms_mean"]),
        "real_false_alarms": pick(lambda r: r["recording"]["hand_usb"]["false_alarms"]),
        "robot_hits_of_9": pick(lambda r: r["recording"]["robot_run"]["hits"]),
        "robot_hits_left": pick(lambda r: r["recording"]["robot_run"]["left"]["hits"]),
        "robot_hits_right": pick(lambda r: r["recording"]["robot_run"]["right"]["hits"]),
        "robot_latency_ms": pick(lambda r: r["recording"]["robot_run"]["latency_ms_mean"]),
        "robot_false_alarms": pick(lambda r: r["recording"]["robot_run"]["false_alarms"]),
        "gf_ipsilateral_fraction": {
            k: (
                round(by[k]["structure"]["ipsilateral_fraction"], 3) if by[k]["structure"] else None
            )
            for k in cmp
        },
        "gf_lc4_to_lplc2_ratio": {
            k: (
                round(by[k]["structure"]["LC4_to_LPLC2_ratio_at_GF"], 3)
                if by[k]["structure"]
                else None
            )
            for k in cmp
        },
    }
    result = {
        "about": "flybrain_models.escape_benchmark output; regenerate with "
        "`uv run python -m flybrain_models.escape_benchmark`",
        "date": _dt.date.today().isoformat(),
        "machine": f"{platform.system()} {platform.machine()} python {platform.python_version()}",
        "key_numbers": key_numbers,
        "key_numbers_doc": {
            "models": "connectome = shipped EscapeCircuit; rewired / random = mean over 5 / 3 "
            "control seeds with the connectome's scalars; no_interneurons = "
            "interneurons removed, connectome scalars; proximity_threshold and "
            "looming_rate_threshold = baselines (one fitted threshold each)",
            "sim_hit_rate": "fraction of simulated threats (>= 0.3 m/s to contact) with a "
            "trigger within -0.1..+1.0 s of the hand entering sensor range",
            "sim_fa_per_min": "triggers that are not hits, per minute of scored non-threat time",
            "sim_latency_ms": "mean trigger time after the hand enters range (D_FAR = 12 cm)",
            "sim_side_correct": "fraction of lateral hits with the correct side (shipped "
            "readout: LC4+LPLC2 left/right activity)",
            "sim_side_correct_gf_readout": "same, if the side were read from GF left vs right "
            "(the wiring-dependent readout)",
            "sim_hit_rate_12hz": "hit rate at 12 Hz (Bluetooth-like) sampling",
            "sim_fa_per_min_noisy": "false alarms per minute with sensor noise sd 80",
            "sim_fa_per_min_{drift,hover,stop_short}": "false alarms per minute in those "
            "non-threat episodes",
            "sim_trigger_distance_cm_{fast,slow}": "hand distance at trigger, 0.8-1.6 m/s "
            "and 0.3-0.5 m/s approaches",
            "sim_hit_rate_{left,right,head_on}": "hit rate for approaches from the left, right "
            "and head-on (both channels see the hand)",
            "sim_hit_rate_{left,right}_only": "hit rate when only that side's channel ever sees "
            "the hand (the far channel stays at baseline), as seen on the robot",
            "robot_hits_of_9": "approaches with a trigger in the 45 s robot escape run "
            "(runs/escape-20261111-183054, offline rule; 9 approach events)",
            "robot_hits_{left,right}": "same, split by the channel that saw the hand first "
            "(left = pin 36; robot run: 2 left events, both seen by pin 36 only, 6 right, "
            "and 1 that both channels saw at once, counted in neither)",
            "robot_latency_ms": "mean trigger time after labelled onset, robot run",
            "robot_false_alarms": "triggers outside approach events, robot run",
            "real_hits_of_5": "recorded approaches (offline rule) with a trigger",
            "real_latency_ms": "mean trigger time after the labelled onset (first sample "
            "< 3100) on the recording",
            "real_false_alarms": "triggers outside the five labelled events (25 s)",
            "gf_ipsilateral_fraction": "fraction of direct LC4+LPLC2 -> GF synapses that come "
            "from the GF's own side (wiring, not behaviour)",
            "gf_lc4_to_lplc2_ratio": "direct LC4 -> GF over LPLC2 -> GF synapses",
        },
        "protocols": {
            "sensor": "simulated raw = 3400 * clip((d - 5 cm) / 7 cm, 0, 1) + noise",
            "ground_truth": "threat = approach >= 0.3 m/s to within 5 cm; non-threats: stop "
            "short inside the band, slow drift <= 0.08 m/s, hover with tremor, "
            "moving away, empty; 0.15-0.3 m/s ambiguous (speed curve only)",
            "recording_labels": "offline rule: event onset = first sample with min(raw) < 3100 "
            "after >= 0.5 s quiet; approach if min(raw) < 500 within 0.5 s; "
            "event ends when both channels >= 3100; leading side = channel "
            "first below 1700. pin36 assumed left (not established)",
            "n_episodes": len(eps),
        },
        "params_connectome": load_params("connectome"),
        "results": rows,
    }
    if not args.no_write:
        results_path().write_text(json.dumps(result, indent=1, default=float) + "\n")
        print(f"\nwrote {results_path()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
