"""The controllers the dashboard can switch between, and what the page draws for them.

Every controller has the interface of ``flybrain_models.pursuit``: ``reset()`` and
``step(azimuth, dt, arousal) -> PursuitOutput`` (azimuth in radians, + = left, or None; turn
in rad/s, + = left). Circuit controllers (``PursuitCircuit``) also have ``activity`` and
``lc10a_azimuth``; the two baselines have no neurons.
"""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from flybrain_models.pursuit import BangBangSteer, ProportionalSteer, PursuitCircuit


@dataclass(frozen=True)
class ControllerInfo:
    key: str
    label: str
    group: str  # "fly" | "changed" | "baseline"
    note: str
    make: Callable[[], object]
    ablated: tuple[str, ...] = ()


def _circuit(**kw) -> Callable[[], PursuitCircuit]:
    return lambda: PursuitCircuit.from_pack(**kw)


CONTROLLERS: dict[str, ControllerInfo] = {
    c.key: c
    for c in (
        ControllerInfo(
            "connectome",
            "Connectome (mirror-averaged)",
            "fly",
            "The fly's wiring, averaged with its mirror image so that left and right match.",
            _circuit(),
        ),
        ControllerInfo(
            "single-brain",
            "Connectome, single brain",
            "fly",
            "The same wiring as measured in one fly. Its two halves are unequal, so it holds "
            "the target off centre, to the right.",
            _circuit(mirror_average=False),
        ),
        ControllerInfo(
            "no-aotu019",
            "Without AOTU019",
            "changed",
            "AOTU019 silenced. Frontal steering is lost: targets near the centre are turned "
            "away from.",
            _circuit(ablate=("AOTU019",)),
            ablated=("AOTU019",),
        ),
        ControllerInfo(
            "no-aotu025",
            "Without AOTU025",
            "changed",
            "AOTU025 silenced. Turns toward far-side targets are weaker, but still above the "
            "robot's turn limit.",
            _circuit(ablate=("AOTU025",)),
            ablated=("AOTU025",),
        ),
        ControllerInfo(
            "aotu019-excitatory",
            "AOTU019 excitatory",
            "changed",
            "AOTU019 made excitatory instead of inhibitory. Frontal targets are turned away from.",
            _circuit(aotu019_sign=1),
        ),
        ControllerInfo(
            "rewired",
            "Rewired",
            "changed",
            "The same neurons and the same number of synapses between each pair of cell "
            "types, but which neuron connects to which is shuffled, so left and right mix.",
            _circuit(variant="rewired"),
        ),
        ControllerInfo(
            "random",
            "Random",
            "changed",
            "A random network of the same size, density and share of inhibitory synapses.",
            _circuit(variant="random"),
        ),
        ControllerInfo(
            "proportional",
            "Proportional (baseline)",
            "baseline",
            "Turn = k x bearing, with k fitted. No neurons.",
            ProportionalSteer,
        ),
        ControllerInfo(
            "bang-bang",
            "Bang-bang (baseline)",
            "baseline",
            "Full-rate turn toward the target outside a fitted dead zone. No neurons.",
            BangBangSteer,
        ),
    )
}
CONTROLLER_KEYS = tuple(CONTROLLERS)


def is_circuit(model) -> bool:
    return isinstance(model, PursuitCircuit)


# Bearings (deg) at which the steering curve is sampled.
CURVE_DEG = tuple(range(-180, 181, 5))
CURVE_HOLD_S = 1.0  # as in the benchmark: the turn after holding a target for 1 s, from rest


def steering_curve(model, arousal: float) -> list[float]:
    """Open-loop turn command (deg/s) after holding a target at each of CURVE_DEG for
    CURVE_HOLD_S from rest. Resets ``model``: use an instance that is not driving."""
    out = []
    for d in CURVE_DEG:
        model.reset()
        turn = model.step(math.radians(d), CURVE_HOLD_S, arousal=arousal).turn
        out.append(round(math.degrees(turn), 2))
    model.reset()
    return out


class CircuitLayout:
    """Which entries of ``activity`` the page draws, in what order: the LC10a of each eye
    sorted by receptive-field azimuth (the 2 LC10a without an estimate get no input and are
    left out)."""

    def __init__(self, circuit: PursuitCircuit):
        az = circuit.lc10a_azimuth
        self.order = {}
        self.azimuth_deg = {}
        for key in ("LC10a_L", "LC10a_R"):
            a = np.asarray(az[key], float)
            idx = np.flatnonzero(np.isfinite(a))
            idx = idx[np.argsort(a[idx])[::-1]]  # left (+) first, as drawn on screen
            self.order[key] = idx
            self.azimuth_deg[key] = [round(float(math.degrees(a[i])), 1) for i in idx]


def activity_scales(arousal: float = 1.0) -> dict[str, float]:
    """The largest steady response of each stage of the connectome circuit over all target
    bearings, used to draw every circuit on one scale (so a weaker or stronger variant shows
    as such). DNa02 is measured as the change from its resting rate."""
    c = PursuitCircuit.from_pack()
    peak = {"LC10a": 0.0, "AOTU019": 0.0, "AOTU025": 0.0, "DNa02": 0.0}
    rest = c.activity["DNa02"].copy()
    for d in CURVE_DEG:
        c.reset()
        c.step(math.radians(d), CURVE_HOLD_S, arousal=arousal)
        a = c.activity
        peak["LC10a"] = max(peak["LC10a"], float(a["LC10a_L"].max()), float(a["LC10a_R"].max()))
        peak["AOTU019"] = max(peak["AOTU019"], float(a["AOTU019"].max()))
        peak["AOTU025"] = max(peak["AOTU025"], float(a["AOTU025"].max()))
        peak["DNa02"] = max(peak["DNa02"], float(np.abs(a["DNa02"] - rest).max()))
    return {k: (v if v > 0 else 1.0) for k, v in peak.items()} | {"DNa02_rest": float(rest.mean())}


# The benchmark's name for each controller (flybrain_models.pursuit_benchmark).
BENCHMARK_NAMES = {
    "connectome": "connectome",
    "single-brain": "connectome_raw",
    "no-aotu019": "no_AOTU019",
    "no-aotu025": "no_AOTU025",
    "aotu019-excitatory": "AOTU019_excitatory",
    "rewired": "rewired",
    "random": "random",
    "proportional": "proportional",
    "bang-bang": "bang_bang",
}


def benchmark_summary() -> dict | None:
    """The offline benchmark's numbers per controller and field of view, from
    ``pursuit_benchmark.json`` (None if the file is missing)."""
    from flybrain_models.pursuit_benchmark import results_path

    try:
        data = json.loads(results_path().read_text())
    except (OSError, ValueError):
        return None
    by_name = {r["model"]: r for r in data.get("results", [])}
    models: dict = {}
    episodes: dict = {}
    for key, name in BENCHMARK_NAMES.items():
        r = by_name.get(name)
        if r is None:
            continue
        m = {"instances": r.get("n_instances", 1)}
        for setting in ("all", "fov30", "fov60", "fov100"):
            s = r["sim"].get(setting)
            if s:
                m[setting] = {"mae_deg": s["mae_deg"], "within10": s["within10"]}
                episodes[setting] = s.get("n_episodes")
        models[key] = m
    return {
        "source": str(results_path().name),
        "date": data.get("date"),
        "models": models,
        "episodes": episodes,
    }
