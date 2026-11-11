"""Fly compass: a head-direction ring attractor built on the FlyWire central-complex pack.

The network is a firing-rate model whose recurrent weights are the connectome's signed
synapse counts (``flybrain_data.load_pack("central_complex")``: EPG, PEN_a, PEN_b, PEG,
Delta7). Only a handful of global scalars are fitted offline (``compass_fit.py``) and
shipped in ``compass_params.json``:

* one gain per presynaptic cell type (scales all output synapses of that type),
* one tonic bias per cell type,
* the time constant,
* two angular-velocity input scalars (gain ``k`` and amplitude ``a``).

No per-synapse weight is fitted. Dynamics, for every neuron i::

    tau dr_i/dt = -r_i + f( g_i(w) * sum_j W_ij r_j + b_type(i) ),   f(u) = tanh(max(u, 0))

where ``W_ij = gain_type(j) * sign_j * count_ij / 100`` and ``g_i(w) = 1 + s_i a tanh(k w)``
for PEN neurons (``s_i = +1`` on the left bridge, ``-1`` on the right) and 1 for all others:
angular velocity multiplicatively modulates PEN input with opposite sign on the two sides.

Compartment option (``params["compartments"]``): per (cell type, neuropil) multipliers on the
synapses a neuron receives in that neuropil. ``{}`` means the plain summed connectome. See
the package README for which setting is shipped and why.

Heading is the population-vector angle of EPG activity using the EPG ring angles stored in the
pack (inferred from connectivity; see the pack metadata), anchored to the requested heading at
``reset``. Convention: positive angular velocity = counter-clockwise (left turn), radians.
"""

from __future__ import annotations

import json
import math
from importlib import resources
from pathlib import Path
from typing import Any

import numpy as np

PARAMS_FILE = "compass_params.json"
TWO_PI = 2.0 * math.pi
# Below this summed EPG rate (47 EPGs, rates in [0, 1]) there is no bump: confidence is 0 and
# the decoded heading stays at its last value.
MIN_ACTIVITY = 1.0
CUE_HOLD_S = 0.25  # a cue sample keeps driving the EPGs this long (5 Hz stream -> continuous)


def wrap(a):
    """Wrap angle(s) to (-pi, pi]."""
    out = -((-np.asarray(a, dtype=float) + math.pi) % TWO_PI - math.pi)
    return float(out) if np.ndim(out) == 0 else out


def type_group(cell_type: str) -> str:
    """Short names used in params: PEN_a(PEN1) -> PEN_a, PEN_b(PEN2) -> PEN_b."""
    return {"PEN_a(PEN1)": "PEN_a", "PEN_b(PEN2)": "PEN_b"}.get(str(cell_type), str(cell_type))


def params_path() -> Path:
    return Path(str(resources.files("flybrain_models") / PARAMS_FILE))


def load_params(name: str = "connectome") -> dict[str, Any]:
    """Fitted scalars shipped with the package; ``name`` is a key of ``fits`` in the file."""
    data = json.loads(params_path().read_text())
    if name not in data["fits"]:
        raise KeyError(f"no fitted params {name!r}; have {sorted(data['fits'])}")
    return data["fits"][name]["params"]


def effective_counts(pack, compartments: dict | None) -> np.ndarray:
    """Synapse counts ``[post, pre]`` with per-(post type, neuropil) multipliers applied."""
    if not compartments:
        return pack.counts.astype(np.float64)
    if pack.by_neuropil is None:
        raise ValueError("compartment weighting needs a pack with a per-neuropil split")
    types = np.array([type_group(t) for t in pack.cell_type])
    out = np.zeros(pack.counts.shape)
    for k, npl in enumerate(pack.neuropils):
        mult = np.array([compartments.get(t, {}).get(npl, 1.0) for t in types])
        out += mult[:, None] * pack.by_neuropil[k]
    return out


class RingAttractor:
    """Rate network on the connectome; integrates angular velocity into a heading bump."""

    def __init__(self, pack, params: dict[str, Any]):
        self.pack = pack
        self.params = dict(params)
        self.variant = getattr(pack, "variant", "connectome")
        p = self.params
        types = np.array([type_group(t) for t in pack.cell_type])
        self.n_neurons = int(pack.n_neurons)

        counts = effective_counts(pack, p.get("compartments"))
        W = counts * pack.sign[None, :].astype(np.float64) / float(p["w_scale"])
        pre_gain = np.array([p["gain"].get(t, 0.0) for t in types])
        W = W * pre_gain[None, :]
        # optional extra gain per pre-type -> post-type block ("EPG->PEN_a": 0.5, ...)
        for key, g in p.get("block_gain", {}).items():
            a, b = key.split("->")
            W[np.ix_(types == b, types == a)] *= float(g)
        if p.get("ablate_recurrence"):  # control: same neurons, leak, bias and cue, no wiring
            W = np.zeros_like(W)
        self._W = np.ascontiguousarray(W)
        self._bias = np.array([p["bias"].get(t, 0.0) for t in types])
        is_pen = np.isin(types, ["PEN_a", "PEN_b"])
        side_sign = np.where(pack.side == "left", 1.0, -1.0)
        self._vel_sign = np.where(is_pen, side_sign, 0.0)
        self._vel_k = float(p["vel_gain"])
        self._vel_amp = float(p["vel_amp"])

        epg = np.flatnonzero(types == "EPG")
        ang = np.asarray(pack.ring_angle, dtype=float)[epg]
        if len(epg) == 0 or np.isnan(ang).any():
            raise ValueError("pack has no EPG neurons with ring angles")
        # The inferred ring is defined up to reflection; ``ring_direction`` (+1/-1) is read off
        # the connectome (PEN shift direction) so positive velocity turns the bump CCW.
        ang = np.mod(ang * float(p.get("ring_direction", 1.0)), TWO_PI)
        order = np.argsort(ang, kind="stable")
        self._epg = epg[order]
        self._angles = ang[order]
        self._bump_order = np.argsort(wrap(self._angles), kind="stable")
        self._cos = np.cos(self._angles)
        self._sin = np.sin(self._angles)

        self.tau = float(p["tau"])
        self.dt_max = float(p.get("dt_max", self.tau / 4.0))
        self._cue_gain = float(p.get("cue_gain", 1.0))
        self._cue_kappa = float(p.get("cue_kappa", 2.0))
        self._cue_hold = float(p.get("cue_hold_s", CUE_HOLD_S))
        self._settle = float(p.get("settle_s", max(0.5, 3.0 * self.tau)))
        self.r = np.zeros(self.n_neurons)
        self._heading = 0.0
        self._conf = 0.0
        self._cue_drive: np.ndarray | None = None
        self._cue_age = math.inf
        self.reset(0.0)

    # ---- construction --------------------------------------------------------------------
    @classmethod
    def from_pack(cls, pack="central_complex", variant: str = "connectome", seed: int = 0,
                  params: dict[str, Any] | str | None = None) -> RingAttractor:
        """Build from a pack name or ``CircuitPack``.

        ``variant`` in {"connectome", "rewired", "random"}; ``seed`` seeds the control.
        ``params`` defaults to the scalars fitted on the connectome, which are applied unchanged
        to the controls. Pass a dict, or another key of ``compass_params.json`` (for example
        ``"rewired_seed0"``, a refit on that control) to override.
        """
        from flybrain_data import CircuitPack, load_pack, make_control

        if isinstance(pack, CircuitPack):
            pk = make_control(pack, variant, seed=seed)
        else:
            pk = load_pack(str(pack), variant=variant, seed=seed)
        if params is None or isinstance(params, str):
            params = load_params(params or "connectome")
        return cls(pk, params)

    # ---- dynamics ------------------------------------------------------------------------
    def _run(self, drive: np.ndarray, duration: float, gain: np.ndarray | None = None) -> None:
        n_sub = max(1, math.ceil(duration / self.dt_max - 1e-9))
        a = (duration / n_sub) / self.tau
        r, W = self.r, self._W
        for _ in range(n_sub):
            u = W @ r
            if gain is not None:
                u *= gain
            u += drive
            np.maximum(u, 0.0, out=u)
            r += a * (np.tanh(u) - r)
        self.r = r

    def _decode(self) -> tuple[float, float]:
        e = self.r[self._epg]
        s = float(e.sum())
        if s <= MIN_ACTIVITY:
            return self._heading, 0.0
        c = float(e @ self._cos) / s
        sn = float(e @ self._sin) / s
        return math.atan2(sn, c), min(1.0, math.hypot(c, sn))

    def cue_current(self, cue_rad: float) -> np.ndarray:
        """External current onto EPGs: von Mises bump centred at ``cue_rad`` (heading frame)."""
        drive = np.zeros(self.n_neurons)
        drive[self._epg] = self._cue_gain * np.exp(
            self._cue_kappa * (np.cos(self._angles - cue_rad) - 1.0))
        return drive

    def reset(self, heading_rad: float = 0.0) -> None:
        """Start from rest, show the landmark cue at ``heading_rad`` for a few time constants,
        then remove it. The decoded heading is whatever the network then holds (no
        re-anchoring), so a network that cannot hold the cue position shows it immediately."""
        self.r = np.zeros(self.n_neurons)
        self._heading = float(wrap(heading_rad))
        self._run(self._bias + self.cue_current(heading_rad), self._settle)
        self._cue_drive, self._cue_age = None, math.inf
        self._heading, self._conf = self._decode()
        self._heading = float(wrap(self._heading))

    def step(self, angular_velocity_rad_s: float, dt: float,
             cue_rad: float | None = None) -> float:
        """Advance by ``dt`` seconds; return the decoded heading in (-pi, pi].

        ``cue_rad``: landmark heading (allothetic cue, e.g. an IMU yaw sample) in the heading
        frame, or None. A cue keeps driving the EPGs for ``cue_hold_s`` (0.25 s) after it was
        given, so a 5 Hz cue stream acts as a continuous cue; once no cue has arrived for
        longer than that the network is in darkness and runs on recurrence (plus velocity
        input) alone.
        """
        dt = float(dt)
        if cue_rad is not None:
            self._cue_drive, self._cue_age = self.cue_current(float(cue_rad)), 0.0
        w = float(angular_velocity_rad_s)
        gain = 1.0 + self._vel_sign * (self._vel_amp * math.tanh(self._vel_k * w))
        np.maximum(gain, 0.0, out=gain)
        drive = self._bias
        if self._cue_drive is not None and self._cue_age < self._cue_hold:
            drive = self._bias + self._cue_drive
        self._run(drive, dt, gain=gain)
        self._cue_age += dt
        raw, self._conf = self._decode()
        self._heading = float(wrap(raw))
        return self._heading

    @property
    def cue_active(self) -> bool:
        return self._cue_drive is not None and self._cue_age < self._cue_hold

    # ---- read-outs -----------------------------------------------------------------------
    @property
    def heading(self) -> float:
        return self._heading

    @property
    def bump(self) -> np.ndarray:
        """EPG activity ordered by ``bump_angles``, shape (n_epg,)."""
        return self.r[self._epg][self._bump_order].copy()

    @property
    def bump_angles(self) -> np.ndarray:
        """Angle of each EPG in the heading frame (radians, ascending in (-pi, pi]). The heading
        frame is the ring frame: the cue and the decoded heading use the same angles."""
        return wrap(self._angles)[self._bump_order]

    @property
    def confidence(self) -> float:
        """Population-vector length of EPG activity in [0, 1]; 0 when the summed EPG rate is
        below ``MIN_ACTIVITY`` (no bump)."""
        return self._conf

    @property
    def activity(self) -> float:
        """Summed EPG rate (bump amplitude)."""
        return float(self.r[self._epg].sum())

    @property
    def n_params_fitted(self) -> int:
        p = self.params
        # + tau, vel_gain, vel_amp
        n_gain = len(p["block_gain"]) if p.get("block_gain") else len(p["gain"])
        return n_gain + len(p["bias"]) + 3 + 2 * bool(p.get("cue_fitted"))

    @property
    def n_synapse_weights(self) -> int:
        return int(np.count_nonzero(self._W))


class GyroIntegrator:
    """Standard engineering baseline: a complementary filter. Integrates angular velocity
    (heading += w * dt); when a cue sample arrives it pulls the estimate toward the cue by a
    fixed fraction ``cue_alpha`` of the wrapped error. With no cue it is pure integration."""

    n_neurons = 0
    n_params_fitted = 0
    n_synapse_weights = 0

    def __init__(self, n_bins: int = 16, kappa: float = 4.0, cue_alpha: float = 0.3):
        self._angles = np.sort(wrap(np.arange(n_bins) * (TWO_PI / n_bins)))
        self._kappa = kappa
        self.cue_alpha = float(cue_alpha)
        self._heading = 0.0

    @classmethod
    def from_pack(cls, *args, **kwargs) -> GyroIntegrator:  # API symmetry with RingAttractor
        return cls()

    def reset(self, heading_rad: float = 0.0) -> None:
        self._heading = float(wrap(heading_rad))

    def step(self, angular_velocity_rad_s: float, dt: float,
             cue_rad: float | None = None) -> float:
        h = self._heading + float(angular_velocity_rad_s) * float(dt)
        if cue_rad is not None:
            h += self.cue_alpha * float(wrap(float(cue_rad) - h))
        self._heading = float(wrap(h))
        return self._heading
    @property
    def heading(self) -> float:
        return self._heading

    @property
    def bump(self) -> np.ndarray:
        """Synthetic von Mises bump at the integrated heading (display only, not a model)."""
        return np.exp(self._kappa * (np.cos(self._angles - self._heading) - 1.0))

    @property
    def bump_angles(self) -> np.ndarray:
        return self._angles.copy()

    @property
    def confidence(self) -> float:
        return 1.0
