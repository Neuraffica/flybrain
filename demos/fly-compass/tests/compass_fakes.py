"""Test-only stand-ins with the flybrain_models.compass interface (not shipped)."""

import math

import numpy as np


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class FakeRing:
    """Heading model with a von Mises bump.

    gain       fraction of the angular velocity it integrates (1 = perfect, 0 = none)
    cue_pull   fraction of the cue error removed per cue (0..1)
    pin_rate   1/s relaxation towards ``pin_rad`` without a cue (0 = holds its heading)
    """

    def __init__(self, n=16, gain=1.0, cue_pull=0.6, pin_rate=0.0, pin_rad=-2.0):
        self.n_neurons = n
        self.gain = gain
        self.cue_pull = cue_pull
        self.pin_rate = pin_rate
        self.pin_rad = pin_rad
        self.bump_angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
        self._h = 0.0
        self.steps = 0
        self.cues: list[float | None] = []

    def reset(self, heading_rad=0.0):
        self._h = heading_rad

    def step(self, angular_velocity_rad_s, dt, cue_rad=None):
        self.steps += 1
        self.cues.append(cue_rad)
        self._h = _wrap(self._h + self.gain * angular_velocity_rad_s * dt)
        if cue_rad is not None:
            self._h = _wrap(self._h + self.cue_pull * _wrap(cue_rad - self._h))
        elif self.pin_rate:
            self._h = _wrap(self._h + min(1.0, self.pin_rate * dt) * _wrap(self.pin_rad - self._h))
        return self._h

    @property
    def bump(self):
        return np.exp(3.0 * np.cos(self.bump_angles - self._h))

    @property
    def confidence(self):
        return 0.9 if not self.pin_rate else 0.4


class FakeGyro(FakeRing):
    def __init__(self):
        super().__init__(gain=1.0, cue_pull=0.3)


class FakeRingNoCue:
    """An older model build whose step() has no cue_rad parameter."""

    def __init__(self):
        self._h = 0.0

    def reset(self, heading_rad=0.0):
        self._h = heading_rad

    def step(self, angular_velocity_rad_s, dt):
        self._h = _wrap(self._h + angular_velocity_rad_s * dt)
        return self._h


def fake_models():
    """Perfect integrators (the rewired one with 0.8 gain)."""
    return {"connectome": FakeRing(), "rewired": FakeRing(gain=0.8), "gyro": FakeGyro()}


def fly_like_models():
    """What the real circuit is reported to do: the connectome ring holds its bump but barely
    integrates velocity; the rewired control does not hold (relaxes to a pinned angle); the
    complementary filter integrates and is pulled to the cue."""
    return {
        "connectome": FakeRing(gain=0.1),
        "rewired": FakeRing(gain=0.1, pin_rate=0.3),
        "gyro": FakeGyro(),
    }
