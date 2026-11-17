"""Fly pursuit: the small-object pursuit pathway (LC10a -> AOTU019/AOTU025 -> DNa02) steering
toward a target seen by a camera.

Input is the target's bearing relative to the robot's heading (radians, positive = to the
LEFT, as in ``compass.py``), for example the centre of a detection box, or ``None`` when no
target is seen. Output is a turn command (rad/s, positive = turn left).

What comes from the connectome (``pursuit`` circuit pack, FlyWire v783, see its metadata):
which neurons connect, the synapse counts, the signs (predicted transmitter) and each neuron's
total number of input synapses in the whole brain. In particular: LC10a of each eye excite
AOTU025 and AOTU019 of the same side; AOTU025 (acetylcholine) excites DNa02 of the same side,
AOTU019 (GABA) inhibits DNa02 of the OPPOSITE side; so a target on the left raises left DNa02
and lowers right DNa02. Also in the pack: the other types that bridge LC10a to DNa02
(AOTU015a, CB3127, CB2070, LAL027; CB3127 and CB2070 excite the CONTRALATERAL DNa02),
TuTuAa (feedback inhibition onto LC10a) and AOTU041 (inhibits AOTU019 and AOTU025).

What comes from the eye map: each LC10a's receptive-field azimuth, ESTIMATED in the pack from
the eye columns of its inputs with an ASSUMED linear column -> azimuth map. AOTU019 ends up
sampling the frontal field (synapse-weighted mean about 34 deg) and AOTU025 the lateral field
(about 75 deg), close to the receptive fields reported by Collie et al. 2026 (Neuron,
doi:10.1016/j.neuron.2026.01.001; peaks about 26 and 72 deg).

Network (connectome-normalised, nothing per class)::

    tau dr/dt = -r + max(0, W r + b + input)
    W[post, pre] = sign(pre) * synapse_count[post, pre] / input_total[post]

``input_total`` is the postsynaptic neuron's input synapse count over the whole brain (all
partners, no threshold), so each weight is the fraction of that neuron's input that comes from
``pre``. There are no per-class gains and no fitted biases. Fixed choices (named constants,
not fitted): TAU = 0.02 s; LC10a input = LC10A_INPUT * exp(-0.5 * ((azimuth - rf) / SIGMA_RF)^2)
with amplitude 1 and SIGMA_RF = 10 deg (Hindmarsh Sten et al. 2021 report LC10a receptive
fields of roughly 7.5-10.5 deg), and no input to the two LC10a without a receptive-field
estimate; DNa02 tonic bias DNA02_BIAS = 1.0 (and DNa02 starts at 1.0 on reset), so that
inhibition from AOTU019 has something to lower; every other bias 0.

Turning follows the report that turning velocity tracks the left-right DNa02 difference
(Rayshubskiy et al. 2025, eLife). The ONLY fitted scalar is ``k_out`` (rad/s per unit of
DNa02 rate difference; ``pursuit_fit.py``), the same number of fitted parameters as each
baseline.

Mirror averaging (``mirror_average=True``, the default): two copies of the network state are
kept; copy A gets the bearing, copy B the mirrored bearing (-azimuth), and
``turn = k_out * 0.5 * (d_A - d_B)`` with ``d = DNa02_left - DNa02_right``. Reason: this is one
individual's brain and its two halves are not equal (for example AOTU019 -> DNa02 is 216
synapses left -> right but 121 right -> left); run raw, the closed loop holds the target
about 16-17 deg to the right of centre (the open-loop turn crosses zero at -19 deg).
Averaging the brain with its mirror image cancels that without fitting anything.
``mirror_average=False`` gives the raw single brain. ``activity`` and the per-side rates in
the output come from copy A (the real bearing).

Arousal (a MODELLING CHOICE motivated by Collie et al. 2026, who report that arousal mainly
recruits the AOTU019 pathway): AOTU019 units get an extra input
``-AROUSAL_THETA * (1 - arousal)`` with AROUSAL_THETA = 0.05. The largest LC10a-driven input
to AOTU019 is about 0.041, so at arousal 0 AOTU019 is silent and at arousal 1 it is unchanged.

Deliberately NOT modelled in this version:

* direction selectivity (LC10a respond to motion of small objects; here any visible target
  drives them, moving or not),
* forward-speed gain (pursuit gain in the fly depends on walking speed),
* elevation (the eye map's dorso-ventral axis is ignored),
* arousal dynamics (arousal is a fixed input level, not a state that builds up),
* object size and contrast (the camera reports a box centre only).

Ablations and the counterfactual: ``ablate=("AOTU019",)`` or ``("AOTU025",)`` silences that
type (its rate is held at 0); ``aotu019_sign=+1`` makes AOTU019 excitatory (the counterfactual
Collie et al. use to argue the inhibitory sign matters).

Dropped approach: an earlier version fitted 13 global scalars (per-class gains, biases, tau,
input and output gains). It reached a similar tracking error only by becoming a saturated
bang-bang controller in which the lateral AOTU025 pathway did nothing, and one of its fits
learned to spin when it saw no target.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Any

import numpy as np

PARAMS_FILE = "pursuit_params.json"
TAU = 0.02  # s, time constant of every unit (fixed choice)
DT_SUB = 0.01  # s, longest exponential-Euler sub-step
SIGMA_RF = math.radians(10.0)  # LC10a tuning width (Hindmarsh Sten et al. 2021: ~7.5-10.5 deg)
LC10A_INPUT = 1.0  # peak LC10a input (fixed choice)
DNA02_BIAS = 1.0  # tonic DNa02 drive, so AOTU019 inhibition can lower it (fixed choice)
AROUSAL_THETA = 0.05  # AOTU019 input removed at arousal 0 (fixed choice)
MAX_TURN = math.radians(90.0)  # rad/s, turn-rate limit of the robot (ASSUMED, see tasks)

LC10A = "LC10a"
DN = "DNa02"
AOTU019 = "AOTU019"
AOTU025 = "AOTU025"


@dataclass(frozen=True)
class PursuitOutput:
    turn: float  # turn command, rad/s, positive = turn left
    dna02_left: float  # DNa02 rates of copy A (the real bearing)
    dna02_right: float
    aotu019_left: float
    aotu019_right: float
    aotu025_left: float
    aotu025_right: float


def params_path() -> Path:
    return Path(str(resources.files("flybrain_models") / PARAMS_FILE))


def load_params(name: str = "connectome") -> dict[str, Any]:
    data = json.loads(params_path().read_text())
    if name not in data["fits"]:
        raise KeyError(f"no fitted params {name!r}; have {sorted(data['fits'])}")
    return data["fits"][name]["params"]


class PursuitCircuit:
    """Connectome-normalised rate model of the small-object pursuit pathway."""

    n_params_fitted = 1  # k_out

    def __init__(
        self,
        pack,
        k_out: float,
        ablate: tuple[str, ...] = (),
        aotu019_sign: int | None = None,
        mirror_average: bool = True,
    ):
        self.pack = pack
        self.k_out = float(k_out)
        self.variant = getattr(pack, "variant", "connectome")
        self.ablate = tuple(ablate)
        self.mirror_average = bool(mirror_average)
        ct = np.asarray(pack.cell_type).astype(str)
        sd = np.asarray(pack.side).astype(str)
        self.n_neurons = int(pack.n_neurons)
        if pack.azimuth is None or pack.input_total is None:
            raise ValueError("pack lacks azimuth or input_total; rebuild the 'pursuit' pack")
        unknown = set(self.ablate) - set(ct)
        if unknown:
            raise ValueError(f"cannot ablate unknown cell types {sorted(unknown)}")

        sign = pack.sign.astype(float).copy()
        if aotu019_sign is not None:
            sign[ct == AOTU019] = float(aotu019_sign)
        self.aotu019_sign = int(np.sign(sign[ct == AOTU019][0]))
        total = np.asarray(pack.input_total, float)
        W = pack.counts.astype(np.float64) * sign[None, :] / np.maximum(total, 1.0)[:, None]
        silenced = np.isin(ct, self.ablate)
        W[:, silenced] = 0.0
        W[silenced, :] = 0.0
        self._W = np.ascontiguousarray(W)
        self._alive = (~silenced).astype(float)

        def idx(t, s=None):
            m = ct == t
            if s is not None:
                m &= sd == s
            return np.flatnonzero(m)

        self._dn = np.array([idx(DN, "left")[0], idx(DN, "right")[0]])
        self._a19 = np.array([idx(AOTU019, "left")[0], idx(AOTU019, "right")[0]])
        self._a25 = np.array([idx(AOTU025, "left")[0], idx(AOTU025, "right")[0]])
        self._lc_side = {s: idx(LC10A, s) for s in ("left", "right")}
        lc = idx(LC10A)
        rf = np.asarray(pack.azimuth, float)[lc]
        self._lc_in = lc[np.isfinite(rf)]  # LC10a with a receptive-field estimate get input
        self._rf = rf[np.isfinite(rf)]
        self._bias = np.zeros(self.n_neurons)
        self._bias[self._dn] = DNA02_BIAS
        self.reset()

    @classmethod
    def from_pack(
        cls,
        pack="pursuit",
        variant: str = "connectome",
        seed: int = 0,
        k_out: float | None = None,
        ablate: tuple[str, ...] = (),
        aotu019_sign: int | None = None,
        mirror_average: bool = True,
    ) -> PursuitCircuit:
        """``variant`` in {"connectome", "rewired", "random"}. The shipped ``k_out`` (fitted on
        the connectome) is used for every variant unless given. The controls keep each
        neuron's receptive field and whole-brain input total; only the wiring changes."""
        from flybrain_data import CircuitPack, load_pack, make_control

        if isinstance(pack, CircuitPack):
            pk = make_control(pack, variant, seed=seed)
        else:
            pk = load_pack(str(pack), variant=variant, seed=seed)
        if k_out is None:
            k_out = load_params("connectome")["k_out"]
        return cls(pk, k_out, ablate, aotu019_sign, mirror_average)

    def reset(self) -> None:
        self.r = np.zeros(self.n_neurons)
        self.r[self._dn] = DNA02_BIAS
        self.r_mirror = self.r.copy()

    def drive(self, azimuth: float | None, arousal: float = 1.0) -> np.ndarray:
        """External input to every unit for a target at ``azimuth`` (or none)."""
        ext = self._bias.copy()
        if azimuth is not None:
            d = (self._rf - float(azimuth) + math.pi) % (2 * math.pi) - math.pi
            ext[self._lc_in] += LC10A_INPUT * np.exp(-0.5 * (d / SIGMA_RF) ** 2)
        a = min(1.0, max(0.0, float(arousal)))
        ext[self._a19] -= AROUSAL_THETA * (1.0 - a)
        return ext

    def _advance(self, r: np.ndarray, ext: np.ndarray, dt: float) -> None:
        n = max(1, math.ceil(dt / DT_SUB - 1e-9))
        a = 1.0 - math.exp(-(dt / n) / TAU)
        W, alive = self._W, self._alive
        for _ in range(n):
            u = W @ r
            u += ext
            np.maximum(u, 0.0, out=u)
            u *= alive
            r += a * (u - r)

    def step(self, azimuth: float | None, dt: float, arousal: float = 1.0) -> PursuitOutput:
        """Advance by ``dt`` seconds (sub-stepped internally) with the latest target bearing
        (radians, + = left) or ``None``, and return the turn command."""
        dt = float(dt)
        if dt > 0:
            self._advance(self.r, self.drive(azimuth, arousal), dt)
            if self.mirror_average:
                mirrored = None if azimuth is None else -float(azimuth)
                self._advance(self.r_mirror, self.drive(mirrored, arousal), dt)
        return self.output()

    def output(self) -> PursuitOutput:
        r = self.r
        d = r[self._dn[0]] - r[self._dn[1]]
        if self.mirror_average:
            dm = self.r_mirror[self._dn[0]] - self.r_mirror[self._dn[1]]
            d = 0.5 * (d - dm)
        return PursuitOutput(
            self.k_out * float(d),
            float(r[self._dn[0]]),
            float(r[self._dn[1]]),
            float(r[self._a19[0]]),
            float(r[self._a19[1]]),
            float(r[self._a25[0]]),
            float(r[self._a25[1]]),
        )

    def steady_turn(self, azimuth: float | None, arousal: float = 1.0, t: float = 1.0) -> float:
        """Open-loop turn command after holding a target at ``azimuth`` for ``t`` seconds."""
        self.reset()
        out = self.step(azimuth, t, arousal)
        self.reset()
        return out.turn

    @property
    def activity(self) -> dict[str, np.ndarray]:
        """Rates of copy A (the real bearing)."""
        return self._activity_of(self.r)

    @property
    def activity_mirror(self) -> dict[str, np.ndarray]:
        """Rates of copy B (the mirrored bearing), in that copy's own left/right. Its right
        side stands for the real left: the mirror-averaged rate of a one-per-side type is
        ``0.5 * (activity[t] + activity_mirror[t][::-1])``, and for DNa02 the left-minus-right
        difference of that average times ``k_out`` is the turn command."""
        return self._activity_of(self.r_mirror)

    def _activity_of(self, r: np.ndarray) -> dict[str, np.ndarray]:
        ct = np.asarray(self.pack.cell_type).astype(str)
        other = np.flatnonzero(~np.isin(ct, (LC10A, AOTU019, AOTU025, DN)))
        return {
            "LC10a_L": r[self._lc_side["left"]].copy(),
            "LC10a_R": r[self._lc_side["right"]].copy(),
            "AOTU019": r[self._a19].copy(),  # [left, right]
            "AOTU025": r[self._a25].copy(),
            "DNa02": r[self._dn].copy(),
            "other": r[other].copy(),  # bridging types, TuTuAa, AOTU041
        }

    @property
    def lc10a_azimuth(self) -> dict[str, np.ndarray]:
        """Receptive-field azimuths (radians) in the order of ``activity['LC10a_L'/'_R']``."""
        az = np.asarray(self.pack.azimuth, float)
        return {"LC10a_L": az[self._lc_side["left"]], "LC10a_R": az[self._lc_side["right"]]}

    @property
    def n_synapse_weights(self) -> int:
        return int(np.count_nonzero(self._W))


def _baseline_output(turn: float) -> PursuitOutput:
    return PursuitOutput(float(turn), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class ProportionalSteer:
    """Baseline: turn = k * azimuth (rad/s per rad), 0 when no target is reported."""

    n_neurons = 0
    n_params_fitted = 1

    def __init__(self, k: float | None = None):
        if k is None:
            k = load_params("proportional")["k"]
        self.k = float(k)
        self._last = 0.0

    def reset(self) -> None:
        self._last = 0.0

    def step(self, azimuth: float | None, dt: float, arousal: float = 1.0) -> PursuitOutput:
        self._last = 0.0 if azimuth is None else self.k * float(azimuth)
        return _baseline_output(self._last)

    @property
    def activity(self) -> dict[str, np.ndarray]:
        return {"turn": np.array([self._last])}


class BangBangSteer:
    """Baseline: three levels. Turn at the full rate ``level`` toward the target when
    |azimuth| > ``threshold``, else 0 (dead zone); 0 when no target is reported. Only the
    threshold is fitted; the level is the robot's turn-rate limit."""

    n_neurons = 0
    n_params_fitted = 1

    def __init__(self, threshold: float | None = None, level: float = MAX_TURN):
        if threshold is None:
            threshold = load_params("bang_bang")["threshold"]
        self.threshold = float(threshold)
        self.level = float(level)
        self._last = 0.0

    def reset(self) -> None:
        self._last = 0.0

    def step(self, azimuth: float | None, dt: float, arousal: float = 1.0) -> PursuitOutput:
        if azimuth is None or abs(azimuth) <= self.threshold:
            self._last = 0.0
        else:
            self._last = math.copysign(self.level, azimuth)
        return _baseline_output(self._last)

    @property
    def activity(self) -> dict[str, np.ndarray]:
        return {"turn": np.array([self._last])}
