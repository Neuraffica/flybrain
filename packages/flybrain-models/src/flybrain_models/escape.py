"""Fly escape: the looming-escape pathway (LC4, LPLC2 -> giant fiber) on a range sensor.

The robot has no camera here. A two-channel infrared proximity sensor (left, right) stands in
for vision: each side's **proximity** (how close the hand is, 0..1) drives that side's LPLC2
population, which in the fly encodes the angular *size* of a looming object, and the
**positive rate of change of proximity** drives that side's LC4 population, which encodes the
looming *angular velocity* (von Reyn et al. 2017; Ache et al. 2019). Every neuron of a
population gets the same input: the sensor has no spatial resolution, so how a population is
distributed over the visual field is not modelled. The model does not see.

Network: firing-rate units on the ``escape`` circuit pack (FlyWire v783; LC4, LPLC2, the
giant fiber DNp01 of each side, interneurons between them, other looming DNs)::

    tau dr/dt = -r + tanh(max(0, W r + b + input))

``W = gain(pre) * sign(pre) * synapse_count / 100``. The connectome supplies who connects to
whom, how strongly and with what sign; only global scalars are fitted (``escape_fit.py``,
stored in ``escape_params.json``): 4 presynaptic gains (LC4+LPLC2 share one, so the LC4:LPLC2
balance at the GF comes from synapse counts), 3 biases, the time constant, one input gain and
one trigger threshold per GF. Each GF can trigger on its own: the decision variable is
``drive = max(GF_left / threshold_left, GF_right / threshold_right)`` against 1 (rising edge),
followed by a fixed refractory period. Per-GF thresholds are needed because the two GFs are
unequally wired in FlyWire (left GF: 634 LC4+LPLC2 synapses after the threshold, right GF:
967); with one shared threshold the left GF could not reach it on left-only input. The side
is read from the left/right LC4+LPLC2 activity (the two GFs are unequally wired in FlyWire).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Any

import numpy as np

PARAMS_FILE = "escape_params.json"
BASELINE_RAW = 3400.0  # Petoi IR proximity sensor, nothing in range (measured, noise ~ +-50)
DEADBAND_RAW = 150.0  # raw values within this of the baseline map to proximity 0 (3 sigma)
V_REF = 5.0  # proximity units / s that count as one unit of LC4 input (a full sweep in 0.2 s)
REFRACTORY_S = 1.0  # after a trigger, no new trigger for this long (the robot is jumping)
SIDE_MARGIN = 0.15  # |L - R| must exceed this fraction of the larger value to name a side

VPN_TYPES = ("LC4", "LPLC2")
GF_TYPE = "DNp01"
DN_TYPES = ("DNp02", "DNp04", "DNp06", "DNp11")


def raw_to_proximity(raw: float, baseline: float = BASELINE_RAW) -> float:
    """Map a raw IR reading (0..4095, lower = closer) to proximity in [0, 1].

    Linear: 0 at or above ``baseline - 150`` (the empty-scene level minus 3 noise sd), 1 at a
    raw value of 0 (as close as the sensor reads). ``baseline`` is the per-channel empty-scene
    reading; calibrate it on the robot (3400 measured)."""
    top = float(baseline) - DEADBAND_RAW
    if top <= 0:
        return 0.0
    return float(min(1.0, max(0.0, (top - float(raw)) / top)))


@dataclass(frozen=True)
class EscapeOutput:
    trigger: bool  # True on the tick an escape is commanded (already debounced/refractory)
    side: int  # where the threat is: +1 = robot's left, -1 = right, 0 = centre/unknown
    gf_left: float  # giant-fiber activity, left and right, for display
    gf_right: float
    drive: float  # the scalar the trigger decision is made on, and
    threshold: float  # its threshold, so the dashboard can draw both


def params_path() -> Path:
    return Path(str(resources.files("flybrain_models") / PARAMS_FILE))


def load_params(name: str = "connectome") -> dict[str, Any]:
    data = json.loads(params_path().read_text())
    if name not in data["fits"]:
        raise KeyError(f"no fitted params {name!r}; have {sorted(data['fits'])}")
    return data["fits"][name]["params"]


def _side(left: float, right: float) -> int:
    big = max(left, right)
    if big <= 0 or abs(left - right) <= SIDE_MARGIN * big:
        return 0
    return 1 if left > right else -1


class _Trigger:
    """Rising-edge threshold crossing with a refractory period, in model time."""

    def __init__(self, threshold: float, refractory_s: float = REFRACTORY_S):
        self.threshold = float(threshold)
        self.refractory_s = float(refractory_s)
        self.reset()

    def reset(self) -> None:
        self.t = 0.0
        self.last = -math.inf
        self.above = False

    def __call__(self, drive: float, dt: float) -> bool:
        self.t += dt
        above = drive >= self.threshold
        fire = above and not self.above and (self.t - self.last) >= self.refractory_s
        self.above = above
        if fire:
            self.last = self.t
        return fire


class EscapeCircuit:
    """Connectome firing-rate model of the looming-escape pathway."""

    def __init__(self, pack, params: dict[str, Any]):
        self.pack = pack
        self.params = dict(params)
        self.variant = getattr(pack, "variant", "connectome")
        p = self.params
        ct = np.asarray(pack.cell_type).astype(str)
        sd = np.asarray(pack.side).astype(str)
        self.n_neurons = int(pack.n_neurons)
        is_vpn = np.isin(ct, VPN_TYPES)
        is_gf = ct == GF_TYPE
        is_in = ~(is_vpn | is_gf)  # interneurons and the other descending neurons
        exc = pack.sign > 0
        g = p["gain"]
        pre_gain = np.where(
            is_vpn, g["vpn"], np.where(is_gf, g["gf"], np.where(exc, g["exc"], g["inh"]))
        )
        W = pack.counts.astype(np.float64) * pack.sign[None, :] / float(p["w_scale"])
        W = W * pre_gain[None, :]
        self._interneurons = np.flatnonzero(is_in & ~np.isin(ct, DN_TYPES))
        if p.get("ablate_interneurons"):
            W[:, self._interneurons] = 0.0
            W[self._interneurons, :] = 0.0
        self._W = np.ascontiguousarray(W)
        b = p["bias"]
        self._bias = np.where(is_vpn, b["vpn"], np.where(is_gf, b["gf"], b["in"])).astype(float)
        self.tau = float(p["tau"])
        # exponential-Euler sub-steps of at most 10 ms (stable for any tau; keeps step cheap)
        self.dt_sub = float(p.get("dt_sub", 0.01))
        self.k_in = float(p["k_in"])

        def idx(t, s):
            return np.flatnonzero((ct == t) & (sd == s))

        self._lplc2 = {"L": idx("LPLC2", "left"), "R": idx("LPLC2", "right")}
        self._lc4 = {"L": idx("LC4", "left"), "R": idx("LC4", "right")}
        self._gf = np.array([idx(GF_TYPE, "left")[0], idx(GF_TYPE, "right")[0]])
        self._dn = np.flatnonzero(np.isin(ct, DN_TYPES))
        th_l = p.get("threshold_left", p.get("threshold"))
        th_r = p.get("threshold_right", p.get("threshold"))
        self.gf_thresholds = (float(th_l), float(th_r))
        self._trigger = _Trigger(1.0, p.get("refractory_s", REFRACTORY_S))
        self.threshold = 1.0  # drive is normalised by each GF's own threshold
        self.reset()

    @classmethod
    def from_pack(
        cls,
        pack="escape",
        variant: str = "connectome",
        seed: int = 0,
        params: dict[str, Any] | str | None = None,
    ) -> EscapeCircuit:
        """``variant`` in {"connectome", "rewired", "random"}; the shipped scalars (fitted on
        the connectome) are used for every variant unless ``params`` names another fit."""
        from flybrain_data import CircuitPack, load_pack, make_control

        if isinstance(pack, CircuitPack):
            pk = make_control(pack, variant, seed=seed)
        else:
            pk = load_pack(str(pack), variant=variant, seed=seed)
        if params is None or isinstance(params, str):
            params = load_params(params or "connectome")
        return cls(pk, params)

    def reset(self) -> None:
        self.r = np.zeros(self.n_neurons)
        self._prev = None
        self._trigger.reset()

    def _inputs(self, pl: float, pr: float, dt: float) -> np.ndarray:
        if self._prev is None or dt <= 0:
            vl = vr = 0.0
        else:
            vl = max(0.0, (pl - self._prev[0]) / dt)
            vr = max(0.0, (pr - self._prev[1]) / dt)
        self._prev = (pl, pr)
        ext = np.zeros(self.n_neurons)
        ext[self._lplc2["L"]] = self.k_in * pl
        ext[self._lplc2["R"]] = self.k_in * pr
        ext[self._lc4["L"]] = self.k_in * vl / V_REF
        ext[self._lc4["R"]] = self.k_in * vr / V_REF
        return ext

    def step(self, prox_left: float, prox_right: float, dt: float) -> EscapeOutput:
        """Advance by ``dt`` seconds (any value; sub-stepped internally) with the latest
        proximity readings in [0, 1] and return the escape decision."""
        dt = float(dt)
        pl = min(1.0, max(0.0, float(prox_left)))
        pr = min(1.0, max(0.0, float(prox_right)))
        drive_in = self._bias + self._inputs(pl, pr, dt)
        if dt > 0:
            n = max(1, math.ceil(dt / self.dt_sub - 1e-9))
            a = 1.0 - math.exp(-(dt / n) / self.tau)
            r, W = self.r, self._W
            for _ in range(n):
                u = W @ r
                u += drive_in
                np.maximum(u, 0.0, out=u)
                r += a * (np.tanh(u) - r)
        gl, gr = float(self.r[self._gf[0]]), float(self.r[self._gf[1]])
        drive = max(gl / self.gf_thresholds[0], gr / self.gf_thresholds[1])
        fire = self._trigger(drive, dt)
        return EscapeOutput(fire, self.side_from_inputs(), gl, gr, drive, self.threshold)

    def side_from_inputs(self) -> int:
        """Side from the left/right LC4+LPLC2 population activity (mean rates). Used for
        ``EscapeOutput.side`` because the two GFs are not balanced in FlyWire: the right GF
        gets 967 LC4+LPLC2 synapses and the left 634, so comparing GF rates is biased to the
        right (see ``side_from_gf``)."""
        r = self.r
        left = r[self._lc4["L"]].mean() + r[self._lplc2["L"]].mean()
        right = r[self._lc4["R"]].mean() + r[self._lplc2["R"]].mean()
        return _side(float(left), float(right))

    def side_from_gf(self) -> int:
        """Side from comparing the two GF rates (diagnostic; biased, see above)."""
        return _side(float(self.r[self._gf[0]]), float(self.r[self._gf[1]]))

    @property
    def activity(self) -> dict[str, np.ndarray]:
        r = self.r
        return {
            "LPLC2_L": r[self._lplc2["L"]].copy(),
            "LC4_L": r[self._lc4["L"]].copy(),
            "LPLC2_R": r[self._lplc2["R"]].copy(),
            "LC4_R": r[self._lc4["R"]].copy(),
            "GF": r[self._gf].copy(),
            "IN": r[self._interneurons].copy(),
            "DN": r[self._dn].copy(),
        }

    @property
    def n_params_fitted(self) -> int:
        return 11  # 4 gains, 3 biases, tau, k_in, 2 GF thresholds

    @property
    def n_synapse_weights(self) -> int:
        return int(np.count_nonzero(self._W))


class ProximityThreshold:
    """Baseline: trigger when max(prox) crosses a threshold (rising edge, same refractory)."""

    n_neurons = 0
    n_params_fitted = 1

    def __init__(self, threshold: float | None = None):
        if threshold is None:
            threshold = load_params("proximity_threshold")["threshold"]
        self._trigger = _Trigger(threshold)
        self.threshold = self._trigger.threshold
        self._last = (0.0, 0.0)

    def reset(self) -> None:
        self._trigger.reset()
        self._last = (0.0, 0.0)

    def step(self, prox_left: float, prox_right: float, dt: float) -> EscapeOutput:
        pl, pr = float(prox_left), float(prox_right)
        self._last = (pl, pr)
        drive = max(pl, pr)
        return EscapeOutput(
            self._trigger(drive, float(dt)), _side(pl, pr), pl, pr, drive, self.threshold
        )

    @property
    def activity(self) -> dict[str, np.ndarray]:
        return {"GF": np.array(self._last)}


class LoomingRateThreshold:
    """Baseline: trigger when the positive rate of change of proximity, d(prox)/dt, on either
    side crosses a threshold (per second; rising edge, same refractory). The side is the
    channel with the higher proximity at the trigger (the nearer one), not the higher rate:
    the rate is noisy and is 0 on a channel that has already saturated."""

    n_neurons = 0
    n_params_fitted = 1

    def __init__(self, threshold: float | None = None):
        if threshold is None:
            threshold = load_params("looming_rate_threshold")["threshold"]
        self._trigger = _Trigger(threshold)
        self.threshold = self._trigger.threshold
        self.reset()

    def reset(self) -> None:
        self._trigger.reset()
        self._prev = None
        self._rate = (0.0, 0.0)

    def step(self, prox_left: float, prox_right: float, dt: float) -> EscapeOutput:
        pl, pr, dt = float(prox_left), float(prox_right), float(dt)
        if self._prev is None or dt <= 0:
            rl = rr = 0.0
        else:
            rl = max(0.0, (pl - self._prev[0]) / dt)
            rr = max(0.0, (pr - self._prev[1]) / dt)
        self._prev = (pl, pr)
        self._rate = (rl, rr)
        drive = max(rl, rr)
        return EscapeOutput(self._trigger(drive, dt), _side(pl, pr), rl, rr, drive, self.threshold)

    @property
    def activity(self) -> dict[str, np.ndarray]:
        return {"GF": np.array(self._rate)}
