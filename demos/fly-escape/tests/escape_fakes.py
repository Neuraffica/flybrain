"""Test-only stand-ins with the flybrain_models.escape interface (not shipped)."""

import types
from dataclasses import dataclass

import numpy as np


def raw_to_proximity(raw, baseline=3400.0):
    return float(min(1.0, max(0.0, 1.0 - float(raw) / float(baseline))))


@dataclass
class EscapeOutput:
    trigger: bool
    side: int
    gf_left: float
    gf_right: float
    drive: float
    threshold: float


class FakeEscape:
    """Triggers when proximity rises fast enough past a level; refractory 1.5 s."""

    def __init__(self, level=0.5, rate=2.0, delay_s=0.0, refractory_s=1.5, n=40):
        self.level, self.rate, self.delay_s, self.refractory_s = level, rate, delay_s, refractory_s
        self.n_neurons = n
        self.reset()

    def reset(self):
        self.t = 0.0
        self.prev = (0.0, 0.0)
        self.last_trigger = -1e9
        self.armed_at = None
        self.gf = (0.0, 0.0)
        self.steps = 0

    def step(self, prox_left, prox_right, dt):
        self.steps += 1
        self.t += dt
        rl = (prox_left - self.prev[0]) / dt
        rr = (prox_right - self.prev[1]) / dt
        self.prev = (prox_left, prox_right)
        self.gf = (prox_left * max(rl, 0.0), prox_right * max(rr, 0.0))
        p = max(prox_left, prox_right)
        looming = max(rl, rr) > self.rate
        trig = False
        if p > self.level and looming and self.armed_at is None:
            self.armed_at = self.t
        if self.armed_at is not None and self.t - self.armed_at >= self.delay_s:
            if self.t - self.last_trigger > self.refractory_s:
                trig = True
                self.last_trigger = self.t
            self.armed_at = None
        side = 0
        if abs(prox_left - prox_right) > 0.05:
            side = 1 if prox_left > prox_right else -1
        return EscapeOutput(trig, side, self.gf[0], self.gf[1], max(self.gf), 1.0)

    @property
    def activity(self):
        return {
            "LPLC2_L": np.full(10, self.prev[0]),
            "LC4_L": np.full(6, self.gf[0]),
            "LPLC2_R": np.full(10, self.prev[1]),
            "LC4_R": np.full(6, self.gf[1]),
            "GF": np.array(self.gf),
        }


class EscapeCircuit(FakeEscape):
    @classmethod
    def from_pack(cls, pack, variant="connectome", seed=0):
        if variant == "connectome":
            return cls()
        if variant == "rewired":
            return cls(level=0.7, delay_s=0.1)
        return cls(level=0.9, rate=4.0)


class ProximityThreshold(FakeEscape):
    def __init__(self):
        super().__init__(level=0.8, rate=-1e9)


class LoomingRateThreshold(FakeEscape):
    def __init__(self):
        super().__init__(level=0.0, rate=3.0)


def install():
    """Build a stand-in flybrain_models.escape module (for the CLI tests)."""
    mod = types.ModuleType("flybrain_models.escape")
    for name in (
        "EscapeCircuit",
        "ProximityThreshold",
        "LoomingRateThreshold",
        "EscapeOutput",
        "raw_to_proximity",
    ):
        setattr(mod, name, globals()[name])
    # Do NOT write sys.modules here: callers register the module with
    # monkeypatch.setitem, which can only restore the real one if it sees it first.
    return mod


def fake_models():
    return (
        {
            "connectome": EscapeCircuit.from_pack("escape", "connectome"),
            "rewired": EscapeCircuit.from_pack("escape", "rewired"),
            "random": EscapeCircuit.from_pack("escape", "random"),
            "prox": ProximityThreshold(),
            "looming": LoomingRateThreshold(),
        },
        raw_to_proximity,
    )
