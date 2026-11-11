"""Load the three heading models the demo compares, all fed the same angular velocity."""

from __future__ import annotations

from typing import Any, Protocol

import numpy as np

MODEL_KEYS = ("connectome", "rewired", "gyro")
LABELS = {
    "connectome": "Connectome ring",
    "rewired": "Rewired control",
    "gyro": "Gyro integrator",
}
SHORT = {"connectome": "Connectome", "rewired": "Rewired", "gyro": "Integrator"}


class HeadingModel(Protocol):
    def reset(self, heading_rad: float = 0.0) -> None: ...
    def step(self, angular_velocity_rad_s: float, dt: float) -> float: ...


class ModelMissing(RuntimeError):
    pass


MISSING_HELP = (
    "The compass models are not available: {err}\n"
    "fly-compass needs flybrain_models.compass (RingAttractor, GyroIntegrator) from "
    "packages/flybrain-models. Run `uv sync` from the repository root; if the module is still "
    "missing, that package has not been built yet."
)


def load_models(seed: int = 0, pack: str = "central_complex") -> dict[str, Any]:
    try:
        from flybrain_models.compass import GyroIntegrator, RingAttractor
    except ImportError as e:
        raise ModelMissing(MISSING_HELP.format(err=e)) from e
    try:
        return {
            "connectome": RingAttractor.from_pack(pack, variant="connectome", seed=seed),
            "rewired": RingAttractor.from_pack(pack, variant="rewired", seed=seed),
            "gyro": GyroIntegrator(),
        }
    except Exception as e:  # missing circuit pack, bad params, ...
        raise ModelMissing(MISSING_HELP.format(err=repr(e))) from e


def bump_of(model: Any) -> tuple[list[float], list[float]]:
    """(activity normalised to 0..1, angles in degrees) or two empty lists."""
    b = getattr(model, "bump", None)
    a = getattr(model, "bump_angles", None)
    if b is None or a is None:
        return [], []
    b = np.asarray(b, dtype=float).ravel()
    a = np.asarray(a, dtype=float).ravel()
    if b.size == 0 or b.size != a.size:
        return [], []
    lo = float(np.min(b))
    hi = float(np.max(b))
    span = hi - lo if hi - lo > 1e-12 else 1.0
    norm = (b - lo) / span if hi > 0 else np.zeros_like(b)
    return [round(float(x), 3) for x in norm], [round(float(np.degrees(x)), 1) for x in a]


def n_neurons_of(model: Any) -> int | None:
    n = getattr(model, "n_neurons", None)
    if n is not None:
        return int(n)
    b = getattr(model, "bump", None)
    return int(np.asarray(b).size) if b is not None else None


def confidence_of(model: Any) -> float | None:
    c = getattr(model, "confidence", None)
    return None if c is None else float(c)
