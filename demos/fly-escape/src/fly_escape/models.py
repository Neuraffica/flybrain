"""The escape models the demo compares: three wirings of the circuit and two baselines, all fed
the same two proximity channels on every sensor sample."""

from __future__ import annotations

import itertools
from typing import Any

import numpy as np

MODEL_KEYS = ("connectome", "rewired", "random", "prox", "looming")
CIRCUITS = ("connectome", "rewired", "random")
LABELS = {
    "connectome": "Connectome",
    "rewired": "Rewired",
    "random": "Random",
    "prox": "Proximity threshold",
    "looming": "Looming-rate threshold",
}

MISSING_HELP = (
    "The escape models are not available: {err}\n"
    "fly-escape needs flybrain_models.escape (EscapeCircuit, ProximityThreshold, "
    "LoomingRateThreshold, raw_to_proximity) from packages/flybrain-models. Run `uv sync` from "
    "the repository root; if the module is still missing, that package has not been built yet."
)


class ModelMissing(RuntimeError):
    pass


def load_models(seed: int = 0, pack: str = "escape") -> tuple[dict[str, Any], Any]:
    """({key: model}, raw_to_proximity)."""
    try:
        from flybrain_models.escape import (
            EscapeCircuit,
            LoomingRateThreshold,
            ProximityThreshold,
            raw_to_proximity,
        )
    except ImportError as e:
        raise ModelMissing(MISSING_HELP.format(err=e)) from e
    try:
        models = {v: EscapeCircuit.from_pack(pack, variant=v, seed=seed) for v in CIRCUITS}
        models["prox"] = ProximityThreshold()
        models["looming"] = LoomingRateThreshold()
    except Exception as e:  # missing pack, bad params, ...
        raise ModelMissing(MISSING_HELP.format(err=repr(e))) from e
    return models, raw_to_proximity


def activity_of(model: Any, max_cells: int = 64) -> dict[str, list[float]]:
    """Population activity for display: each group normalised to 0..1, at most max_cells."""
    act = getattr(model, "activity", None)
    if not isinstance(act, dict):
        return {}
    out = {}
    for name, arr in act.items():
        a = np.asarray(arr, dtype=float).ravel()
        if a.size == 0:
            continue
        if a.size > max_cells:  # average neighbouring cells down to max_cells bins
            edges = np.linspace(0, a.size, max_cells + 1).astype(int)
            a = np.array([a[i:j].mean() for i, j in itertools.pairwise(edges)])
        hi = float(np.max(np.abs(a)))
        out[str(name)] = [round(float(x), 3) for x in (a / hi if hi > 1e-12 else a)]
    return out


def n_neurons_of(model: Any) -> int | None:
    n = getattr(model, "n_neurons", None)
    return None if n is None else int(n)
