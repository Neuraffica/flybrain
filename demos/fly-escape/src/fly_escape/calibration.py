"""Resting level of each proximity channel, measured with hands away at start."""

from __future__ import annotations

from dataclasses import dataclass, field

CAL_SECONDS = 2.0
MIN_SAMPLES = 10
# The real sensor rests at about 3400 +- 50 (USB, firmware B10_250330) and reads about 0 with a
# hand close or when nothing is plugged in (an unconnected ESP32 pin reads about 0).
MIN_REST_LEVEL = 1500.0
MAX_REST_LEVEL = 4095.0
MAX_REST_NOISE = 300.0  # robust standard deviation (1.4826 x median absolute deviation)


def _median(xs: list[float]) -> float:
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


@dataclass
class Calibration:
    started: float | None = None
    seconds: float = CAL_SECONDS
    samples: list[tuple[float, float, float]] = field(default_factory=list)
    done: bool = False
    ok: bool = False
    baseline: tuple[float, float] = (3400.0, 3400.0)
    noise: tuple[float, float] = (0.0, 0.0)
    message: str = "calibrating: keep hands away from the sensor"

    def start(self, now: float) -> None:
        self.started, self.samples, self.done, self.ok = now, [], False, False
        self.message = "calibrating: keep hands away from the sensor"

    def add(self, t: float, left: float, right: float) -> bool:
        """Feed a sample; returns True when calibration has just finished."""
        if self.done or self.started is None:
            return False
        self.samples.append((t, left, right))
        if t - self.started < self.seconds:
            return False
        self.finish()
        return True

    def finish(self) -> None:
        self.done = True
        n = len(self.samples)
        if n < MIN_SAMPLES:
            self.ok = False
            self.message = (
                f"calibration FAILED: only {n} sensor readings in {self.seconds:.0f} s "
                "(is the link working?)"
            )
            return
        # Median and median absolute deviation, so a hand that arrives near the end of the
        # window (as in the recorded fixture, at 1.6 s) does not spoil the resting level.
        stats = []
        for i in (1, 2):
            xs = sorted(s[i] for s in self.samples)
            m = _median(xs)
            sd = 1.4826 * _median(sorted(abs(x - m) for x in xs))  # robust standard deviation
            stats.append((m, sd))
        self.baseline = (stats[0][0], stats[1][0])
        self.noise = (stats[0][1], stats[1][1])
        problems = []
        for name, (m, sd) in zip(("left", "right"), stats, strict=True):
            if m < MIN_REST_LEVEL:
                problems.append(
                    f"{name} channel rests at {m:.0f} (expected about 3400): sensor unplugged, "
                    "wrong pins, or something in front of it"
                )
            elif m > MAX_REST_LEVEL:
                problems.append(f"{name} channel reads {m:.0f}, above the ADC range")
            if sd > MAX_REST_NOISE:
                problems.append(f"{name} channel is noisy (sd {sd:.0f}): a hand moving nearby?")
        self.ok = not problems
        if self.ok:
            self._ok_message()
        else:
            self.message = "calibration FAILED: " + "; ".join(problems)

    def _ok_message(self) -> None:
        self.message = (
            f"calibrated: left {self.baseline[0]:.0f} +- {self.noise[0]:.0f}, "
            f"right {self.baseline[1]:.0f} +- {self.noise[1]:.0f}"
        )

    def swap_sides(self) -> None:
        """The side check found the pins the other way round: left and right trade places."""
        self.baseline, self.noise = self.baseline[::-1], self.noise[::-1]
        if self.done and self.ok:
            self._ok_message()

    def status(self) -> dict:
        return {
            "done": self.done,
            "ok": self.ok,
            "baseline": [round(b, 1) for b in self.baseline],
            "noise": [round(x, 1) for x in self.noise],
            "message": self.message,
        }
