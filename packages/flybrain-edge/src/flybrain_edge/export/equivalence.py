"""Compile the generated C on the host and compare it with the Python model tick by tick.

uv run python -m flybrain_edge.export.equivalence     # prints a JSON summary
"""

from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np

HARNESS = Path(__file__).with_name("host_harness.c")


def find_cc() -> str | None:
    for name in ("cc", "gcc", "clang"):
        path = shutil.which(name)
        if path:
            return path
    return None


def build_harness(workdir: Path, params_key: str = "connectome") -> Path:
    from .escape_c import export

    export(workdir, params_key)
    shutil.copy(HARNESS, workdir / "host_harness.c")
    exe = workdir / "harness"
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
            "-o",
            str(exe),
            str(workdir / "host_harness.c"),
            str(workdir / "fly_escape.c"),
            "-lm",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return exe


def episodes(sim_every: int = 8) -> list[tuple[str, np.ndarray, np.ndarray, np.ndarray]]:
    """(name, raw_l, raw_r, dt_s) for both real recordings and every ``sim_every``-th
    simulated benchmark episode."""
    from flybrain_models.escape_tasks import RECORDINGS, benchmark_set, load_recording

    out = []
    for name in RECORDINGS:
        t, rl, rr = load_recording(name)
        out.append((f"real:{name}", rl, rr, np.diff(np.concatenate([[0.0], t]))))
    for ep in benchmark_set()[::sim_every]:
        out.append((f"sim:{ep.name}", ep.raw_l, ep.raw_r, np.diff(np.concatenate([[0.0], ep.t]))))
    return out


def run_python(model, rl, rr, dt) -> np.ndarray:
    from flybrain_models.escape import raw_to_proximity

    model.reset()
    rows = []
    for a, b, d in zip(rl, rr, dt, strict=True):
        o = model.step(raw_to_proximity(a), raw_to_proximity(b), float(d))
        gl, gr = (o.gf_left, o.gf_right)
        rows.append((int(o.trigger), o.side, gl, gr, o.drive))
    return np.array(rows, dtype=float)


def run_c(exe: Path, model_id: int, eps) -> list[np.ndarray]:
    lines = []
    for _, rl, rr, dt in eps:
        lines.append("R")
        lines += [f"{a:.6f} {b:.6f} {1000.0 * d:.9f}" for a, b, d in zip(rl, rr, dt, strict=True)]
    res = subprocess.run(
        [str(exe), str(model_id)],
        input="\n".join(lines) + "\n",
        capture_output=True,
        text=True,
        check=True,
    )
    vals = np.array([[float(x) for x in ln.split()] for ln in res.stdout.splitlines()])
    out, i = [], 0
    for _, rl, _, _ in eps:
        out.append(vals[i : i + len(rl)])
        i += len(rl)
    return out


def compare(sim_every: int = 8) -> dict:
    from flybrain_models.escape import EscapeCircuit, LoomingRateThreshold

    eps = episodes(sim_every)
    with tempfile.TemporaryDirectory() as tmp:
        exe = build_harness(Path(tmp))
        summary = {}
        for model_id, name, make in (
            (0, "connectome", EscapeCircuit.from_pack),
            (1, "looming_rate_threshold", LoomingRateThreshold),
        ):
            c_out = run_c(exe, model_id, eps)
            m = make()
            trig_mismatch, side_mismatch, n_trig, steps = 0, 0, 0, 0
            max_gf, max_drive = 0.0, 0.0
            worst = None
            for (ename, rl, rr, dt), c in zip(eps, c_out, strict=True):
                py = run_python(m, rl, rr, dt)
                steps += len(py)
                n_trig += int(py[:, 0].sum())
                tm = int((py[:, 0] != c[:, 0]).sum())
                trig_mismatch += tm
                fired = py[:, 0] == 1
                side_mismatch += int((py[fired, 1] != c[fired, 1]).sum())
                if model_id == 0:
                    d = float(np.abs(py[:, 2:4] - c[:, 2:4]).max())
                    if d > max_gf:
                        max_gf, worst = d, ename
                max_drive = max(
                    max_drive,
                    float(np.abs(py[:, 4] / (1 if model_id == 0 else m.threshold) - c[:, 4]).max()),
                )
            summary[name] = {
                "episodes": len(eps),
                "steps": steps,
                "python_triggers": n_trig,
                "trigger_tick_mismatches": trig_mismatch,
                "side_mismatches_on_triggers": side_mismatch,
                "max_abs_gf_diff": max_gf if model_id == 0 else None,
                "max_abs_drive_diff": max_drive,
                "worst_episode": worst,
            }
    return summary


if __name__ == "__main__":
    print(json.dumps(compare(), indent=1))
