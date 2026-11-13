"""The on-board escape demo on the simulator: finds the module, switches it on, listens,
never moves the robot itself, and leaves the module as asked. Runs without a robot."""

import json
import os
import subprocess
import sys

import pytest
from fly_escape_onboard import cli
from fly_escape_onboard.flydeck import SPEC
from flybrain_edge.bittle import MockBittle

LISTEN_ONLY = ("?", "X")  # identify, and the module switches: never a skill, joint or rest


@pytest.fixture
def robots(monkeypatch):
    """Every simulated robot the demo creates, for inspection after it has run."""
    made = []

    def make(**kw):
        made.append(MockBittle(**kw))
        return made[-1]

    monkeypatch.setattr(cli, "MockBittle", make)
    return made


def test_spec_and_flags():
    SPEC.validate()
    parse = cli.build_parser().parse_args
    a = parse(SPEC.argv("ble", {}, 8899, "BittleC2_BLE", space_confirmed=True))
    assert a.enable and a.leave_running and a.i_have_space and a.ble == "BittleC2_BLE"
    a = parse(SPEC.argv("usb", {"enable": False, "leave_running": False}, 1, "/dev/x"))
    assert not a.enable and not a.leave_running and not a.i_have_space
    assert SPEC.needs_space({}, "usb") and not SPEC.needs_space({"enable": False}, "usb")
    assert parse(SPEC.argv("mock", {}, 1)).mock


@pytest.mark.parametrize("link", ["usb", "ble", "ble10"])
def test_listens_to_the_robots_own_escapes(robots, link):
    code = cli.main(["--mock", "--mock-link", link, "--http-port", "0", "--duration", "9"])
    assert code == 0
    mock = robots[0]
    assert mock.fe_triggers >= 1  # the hand from the left at 5 s
    assert all(c.startswith(LISTEN_ONLY) for c in mock.commands), mock.commands
    assert mock.commands[-1] == "XEp"  # left running, telemetry off
    assert mock.modules["E"] and not mock.fe_telemetry


def test_switches_the_module_off_when_asked(robots):
    argv = ["--mock", "--no-leave-running", "--http-port", "0", "--duration", "1"]
    assert cli.main(argv) == 0
    assert robots[0].commands[-1] == "Xe" and not robots[0].modules["E"]


def test_stock_firmware_fails_with_a_pointer(monkeypatch, capsys):
    monkeypatch.setattr(cli, "MockBittle", lambda **kw: MockBittle(**{**kw, "fly_escape": False}))
    assert cli.main(["--mock", "--http-port", "0", "--duration", "1"]) == 1
    err = capsys.readouterr().err
    assert "no fly escape module" in err and "firmware/bittle/fly_escape/README.md" in err


def test_enable_needs_space_on_a_real_robot():
    assert cli.main(["--port", "/dev/null-robot"]) == 2


def test_reports_ready_to_flydeck():
    env = dict(os.environ, FLYDECK_STATUS="1")
    out = subprocess.run(
        [
            sys.executable,
            "-m",
            "fly_escape_onboard.cli",
            "--mock",
            "--http-port",
            "0",
            "--duration",
            "2",
        ],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
        check=True,
    ).stdout
    events = [json.loads(ln[8:]) for ln in out.splitlines() if ln.startswith("FLYDECK ")]
    assert [e["event"] for e in events][:2] == ["state", "ready"]
    assert "the brain runs on the robot" in out
