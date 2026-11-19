"""Compile ``fly_pursuit_all.h`` on the host and compare it with ``PursuitCircuit``.

uv run python -m flybrain_edge.export.pursuit_equivalence     # prints a JSON summary

Three comparisons, all with the shipped ``k_out`` and mirror averaging:

1. **Replay of closed-loop episodes**: each episode of ``pursuit_tasks.benchmark_set()`` (every
   ``every``-th) is run in closed loop with the Python model; the inputs it was given (bearing
   or none, dt) are recorded and fed frame by frame to the C model, and the two turn commands
   are compared at every frame.
2. **Closed loop with the C model in the loop**: the same episodes are run once more with the C
   model driving the simulated robot, and the bearing trajectories of the two runs are compared
   (this shows whether float32 differences grow through the loop).
3. **Steering curve**: the open-loop turn after holding a target at -90..90 degrees for 1 s.

The C side is a shared library built from the header plus a few exported wrappers, called with
``ctypes``; it needs a host C compiler (cc, gcc or clang) and nothing from the raw connectome
data (the circuit pack ships with flybrain-data).
"""

from __future__ import annotations

import ctypes
import json
import math
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .equivalence import find_cc

WRAPPER = r"""
#include "fly_pursuit_all.h"
static fp_state_t S;
void fpw_reset(void) { fp_reset(&S); }
float fpw_step(int has, float az, float dt, float arousal) {
  return fp_step(&S, (uint8_t)(has != 0), az, dt, arousal);
}
float fpw_dn_left(void) { return fp_dna02_left(&S); }
float fpw_dn_right(void) { return fp_dna02_right(&S); }
"""


@dataclass(frozen=True)
class _Out:
    turn: float


class CPursuit:
    """The exported header as a model with the ``PursuitCircuit`` step interface."""

    def __init__(self, lib_path: Path):
        lib = ctypes.CDLL(str(lib_path))
        lib.fpw_step.restype = ctypes.c_float
        lib.fpw_step.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        lib.fpw_dn_left.restype = ctypes.c_float
        lib.fpw_dn_right.restype = ctypes.c_float
        self._lib = lib
        self.reset()

    def reset(self) -> None:
        self._lib.fpw_reset()

    def step(self, azimuth, dt, arousal: float = 1.0) -> _Out:
        has = azimuth is not None
        return _Out(float(self._lib.fpw_step(int(has), float(azimuth or 0.0), dt, arousal)))

    def dna02(self) -> tuple[float, float]:
        return float(self._lib.fpw_dn_left()), float(self._lib.fpw_dn_right())


def build_library(workdir: Path, header_dir: Path | None = None) -> Path:
    """Export the header into ``workdir`` (or use the one in ``header_dir``) and build it as a
    shared library with warnings as errors."""
    from .pursuit_c import export

    workdir = Path(workdir)
    if header_dir is None:
        export(workdir)
        header_dir = workdir
    src = workdir / "fp_wrapper.c"
    src.write_text(WRAPPER)
    lib = workdir / ("fp_wrapper.dylib" if sys.platform == "darwin" else "fp_wrapper.so")
    cc = find_cc()
    if cc is None:
        raise RuntimeError("no C compiler found")
    subprocess.run(
        [
            cc,
            "-std=c99",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-shared",
            "-fPIC",
            "-I",
            str(header_dir),
            "-o",
            str(lib),
            str(src),
            "-lm",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return lib


class _Recorder:
    """Wraps the Python model and records what it was given."""

    def __init__(self, model):
        self.model = model
        self.inputs: list[tuple[float | None, float, float]] = []
        self.turns: list[float] = []

    def reset(self) -> None:
        self.model.reset()
        self.inputs.clear()
        self.turns.clear()

    def step(self, azimuth, dt, arousal: float = 1.0):
        out = self.model.step(azimuth, dt, arousal=arousal)
        self.inputs.append((azimuth, dt, arousal))
        self.turns.append(out.turn)
        return out


def compare(every: int = 4, lib_path: Path | None = None) -> dict:
    from flybrain_models.pursuit import PursuitCircuit
    from flybrain_models.pursuit_tasks import benchmark_set, run_episode

    with tempfile.TemporaryDirectory() as tmp:
        lib = lib_path or build_library(Path(tmp))
        c = CPursuit(lib)
        py = PursuitCircuit.from_pack()
        rec = _Recorder(py)
        eps = benchmark_set()[::every]
        max_turn, max_rel, max_traj, frames, worst = 0.0, 0.0, 0.0, 0, None
        turn_scale = 0.0
        for ep in eps:
            res_py = run_episode(rec, ep)
            c.reset()
            ct = np.array([c.step(a, dt, ar).turn for a, dt, ar in rec.inputs])
            pt = np.array(rec.turns)
            d = float(np.abs(ct - pt).max())
            frames += len(pt)
            turn_scale = max(turn_scale, float(np.abs(pt).max()))
            if d > max_turn:
                max_turn, worst = d, ep.name
            max_rel = max(max_rel, d / max(1e-9, float(np.abs(pt).max())))
            res_c = run_episode(c, ep)
            max_traj = max(max_traj, float(np.abs(res_c["az"] - res_py["az"]).max()))
        # steering curve: 1 s of a held target, open loop
        curve = []
        for deg in np.arange(-90.0, 90.0 + 1e-9, 5.0):
            az = math.radians(float(deg))
            pv = py.steady_turn(az)  # one 1 s step from reset (100 sub-steps)
            c.reset()
            cv = c.step(az, 1.0).turn
            curve.append((float(deg), pv, cv))
        curve_arr = np.array(curve)
        dn_c = c.dna02()
    return {
        "episodes": len(eps),
        "frames": frames,
        "max_abs_turn_diff_rad_s": max_turn,
        "max_abs_turn_diff_deg_s": math.degrees(max_turn),
        "largest_python_turn_rad_s": turn_scale,
        "max_turn_diff_relative_to_episode_peak": max_rel,
        "worst_episode": worst,
        "closed_loop_max_bearing_diff_deg": math.degrees(max_traj),
        "steering_curve_points": len(curve),
        "steering_curve_max_abs_diff_rad_s": float(np.abs(curve_arr[:, 1] - curve_arr[:, 2]).max()),
        "steering_curve_peak_turn_rad_s": float(np.abs(curve_arr[:, 1]).max()),
        "dna02_after_curve_c": dn_c,
    }


if __name__ == "__main__":
    print(json.dumps(compare(every=int(sys.argv[1]) if len(sys.argv) > 1 else 4), indent=1))
