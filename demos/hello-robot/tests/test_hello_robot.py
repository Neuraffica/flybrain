"""Smoke test on the simulator: connects, wags, rests, exits 0. Runs without a robot."""

import json
import os
import subprocess
import sys

from hello_robot.cli import main
from hello_robot.flydeck import SPEC


def test_spec_is_valid():
    SPEC.validate()
    assert SPEC.argv("mock", {}, 0)[0] == "--mock"


def test_runs_on_the_simulator():
    assert main(["--mock", "--http-port", "0", "--duration", "3", "--wags", "1"]) == 0


def test_walk_needs_space_on_a_real_robot():
    assert main(["--port", "/dev/null-robot", "--walk"]) == 2


def test_reports_ready_to_flydeck():
    env = dict(os.environ, FLYDECK_STATUS="1")
    out = subprocess.run(
        [sys.executable, "-m", "hello_robot.cli", "--mock", "--http-port", "0", "--duration", "2"],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
        check=True,
    ).stdout
    events = [json.loads(ln[8:])["event"] for ln in out.splitlines() if ln.startswith("FLYDECK ")]
    assert events[:2] == ["state", "ready"] and "running" in out
