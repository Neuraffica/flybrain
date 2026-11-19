"""Commands, frames, the spec, the CLI, and one end-to-end run of the real program."""

import json
import os
import re
import socket
import subprocess
import sys
import time
import urllib.request
from importlib import resources
from pathlib import Path

import pytest
from fly_pursuit import cli, flydeck
from fly_pursuit.app import PursuitApp
from fly_pursuit.controllers import BENCHMARK_NAMES, CONTROLLER_KEYS, benchmark_summary
from fly_pursuit.sim import FOV_CHOICES, MOTIONS, PursuitSim

ROOT = Path(__file__).resolve().parents[3]
DEMO = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def app():
    return PursuitApp(PursuitSim(seed=4))


def test_commands_change_state(app):
    s = app.sim
    ok = {"ok": True}
    assert app.on_command({"action": "target", "deg": -50}) == ok
    assert round(s.theta * 180 / 3.141592653589793) == -50 and s.motion == "still"
    assert app.on_command({"action": "nudge", "deg": 5}) == ok
    assert round(s.theta * 180 / 3.141592653589793) == -45
    assert app.on_command({"action": "motion", "motion": "walk"}) == ok and s.motion == "walk"
    assert app.on_command({"action": "motion", "motion": "hidden"}) == ok and not s.exists
    assert app.on_command({"action": "fov", "deg": 100}) == ok and s.rig.fov_half_deg == 100
    assert app.on_command({"action": "arousal", "value": 0}) == ok and s.arousal == 0.0
    assert app.curve["arousal"] == 0.0  # the steering curve follows
    assert app.on_command({"action": "controller", "controller": "bang-bang"}) == ok
    assert s.controller == "bang-bang" and app.curve["controller"] == "bang-bang"
    app.tick(0.2)
    assert app.on_command({"action": "reset"}) == ok and s.stats()["seconds"] == 0.0
    app.on_command({"action": "controller", "controller": "connectome"})
    app.on_command({"action": "arousal", "value": 1})
    app.on_command({"action": "motion", "motion": "still"})


@pytest.mark.parametrize(
    "cmd",
    [
        {},
        {"action": "fly"},
        {"action": "target"},
        {"action": "target", "deg": "north"},
        {"action": "nudge", "deg": 1000},
        {"action": "motion", "motion": "dance"},
        {"action": "motion", "motion": ["still"]},
        {"action": "controller", "controller": "brain"},
        {"action": "controller", "controller": ["connectome"]},
        {"action": "arousal", "value": 2},
        {"action": "arousal", "value": "high"},
        {"action": "fov", "deg": 45},
    ],
)
def test_bad_commands_are_refused(app, cmd):
    before = app.sim.snapshot()
    reply = app.on_command(cmd)
    assert reply["ok"] is False and reply["error"]
    after = app.sim.snapshot()
    for k in ("controller", "arousal", "fov_deg", "target"):
        assert after[k] == before[k]


def test_frame_has_what_the_page_uses(app):
    app.on_command({"action": "target", "deg": 20})
    f = json.loads(json.dumps(app.tick(0.5)))  # as the page receives it
    page = resources.files("fly_pursuit").joinpath("static/index.html").read_text()
    # every top-level field the page reads as frame.<name>
    used = set(re.findall(r"\bframe\.([a-z_0-9]+)", page))
    assert used, "the page reads no frame fields?"
    assert used <= set(f), sorted(used - set(f))
    for k in ("exists", "bearing_deg", "motion"):
        assert k in f["target"]
    for k in ("rate_hz", "delay_s", "max_turn_deg_s"):
        assert k in f["rig"]
    for k in ("seconds", "mae_deg", "within10", "in_view"):
        assert k in f["stats"]
    a = f["activity"]
    for k in ("lc10a_l", "lc10a_r", "lc10a_l_deg", "lc10a_r_deg", "aotu019", "aotu025", "dna02"):
        assert k in a
    assert len(a["lc10a_l"]) == len(a["lc10a_l_deg"]) == 114
    assert len(a["lc10a_r"]) == len(a["lc10a_r_deg"]) == 118
    assert a["lc10a_l_deg"] == sorted(a["lc10a_l_deg"], reverse=True)
    assert len(a["aotu019"]) == len(a["aotu025"]) == len(a["dna02"]) == 2
    c = f["curve"]
    assert len(c["deg"]) == len(c["turn_deg_s"]) == 73
    assert f["camera"] in ("seen", "outside", "dropped", "hidden", "waiting")


def test_robot_frames_have_the_same_fields(app):
    """The page reads the same fields from the robot's frames as from the simulator's."""
    from fly_pursuit.camera import CameraReader
    from fly_pursuit.robot import HeadRig

    reader = CameraReader()
    head = PursuitApp(HeadRig(reader), frame_hz=15)
    reader.feed_line("-72.00\t-16.00\tsize = 69\t76\t", time.monotonic())
    f = json.loads(json.dumps(head.tick(1 / 15)))
    s = json.loads(json.dumps(app.tick(0.1)))
    assert set(f) == set(s)
    assert set(f["target"]) == set(s["target"]) and set(f["stats"]) == set(s["stats"])
    assert set(s["rig"]) <= set(f["rig"])
    page = resources.files("fly_pursuit").joinpath("static/index.html").read_text()
    used = set(re.findall(r"\bframe\.([a-z_0-9]+)", page))
    assert used <= set(f), sorted(used - set(f))
    assert f["mode"] == "robot" and s["mode"] == "sim" and s["link"] is None
    assert f["camera"] == "seen" and f["error_deg"] == pytest.approx(18.0)
    assert f["target"]["bearing_deg"] == pytest.approx(f["head_deg"] + 18.0)
    assert f["activity"] is not None and f["curve"]["controller"] == "connectome"
    for k in ("camera_fov_deg", "stale_s", "centre_deg", "hold_after_s", "max_turn_deg_s"):
        assert f["rig"][k] is not None
    # the simulator's own commands are refused on the robot
    for cmd in (
        {"action": "target", "deg": 10},
        {"action": "motion", "motion": "sweep"},
        {"action": "fov", "deg": 60},
    ):
        assert head.on_command(cmd)["ok"] is False
    assert head.on_command({"action": "controller", "controller": "bang-bang"}) == {"ok": True}
    assert head.on_command({"action": "arousal", "value": 0.5}) == {"ok": True}


def test_baselines_have_no_activity(app):
    app.on_command({"action": "controller", "controller": "proportional"})
    f = app.tick(0.1)
    assert f["activity"] is None and f["circuit"] is False
    app.on_command({"action": "controller", "controller": "connectome"})


def test_ablated_types_are_marked(app):
    app.on_command({"action": "controller", "controller": "no-aotu019"})
    f = app.tick(0.1)
    assert f["activity"]["ablated"] == ["AOTU019"] and f["activity"]["aotu019"] == [0.0, 0.0]
    app.on_command({"action": "controller", "controller": "connectome"})


def test_page_controls_match_the_python_choices():
    page = resources.files("fly_pursuit").joinpath("static/index.html").read_text()
    assert tuple(re.findall(r'<option value="([^"]+)"', page)) == CONTROLLER_KEYS
    assert tuple(int(x) for x in re.findall(r'data-fov="(\d+)"', page)) == FOV_CHOICES
    assert tuple(re.findall(r'data-motion="([^"]+)"', page)) == MOTIONS


def test_spec_is_valid_and_matches():
    spec = flydeck.SPEC
    spec.validate()
    assert flydeck.CONTROLLERS == CONTROLLER_KEYS
    assert flydeck.FOVS == FOV_CHOICES
    assert flydeck.MOTIONS == MOTIONS
    assert spec.transports == ("mock", "usb", "ble") and spec.moves_robot
    assert not spec.needs_space({}, "usb")  # the legs never walk: no space confirmation
    assert any("SCL SDA +5V GND" in n for n in spec.needs)
    assert any("floor" in n for n in spec.needs)
    assert (DEMO / spec.guide).is_file()
    argv = spec.argv("mock", {"fov": "100", "target": "jumps"}, 8123)
    a = cli.build_parser().parse_args(argv)
    assert a.mock and a.fov == 100 and a.target == "jumps" and a.http_port == 8123 and a.no_open
    # on the robot the simulator's settings are not passed; the robot's are
    opts = {"fov": "100", "camera_fov": "50", "head_rate": "45", "command_rate": "5"}
    argv = spec.argv("usb", opts, 8123, robot="/dev/cu.fake")
    assert "--fov" not in argv and "--target" not in argv
    a = cli.build_parser().parse_args(argv)
    assert a.port == "/dev/cu.fake" and not a.mock and a.camera_fov == 50 and a.head_rate == 45
    assert a.command_rate == 5 and a.pan_limit == 50 and not a.leave_voice_off
    a = cli.build_parser().parse_args(spec.argv("ble", {"leave_voice_off": True}, 8123))
    assert a.ble == "" and a.leave_voice_off
    for o in spec.options:  # the card's defaults are the program's
        if o.name not in ("fov", "target", "controller", "camera_port"):
            assert getattr(cli.build_parser().parse_args([]), o.name) == o.default, o.name
    a = cli.build_parser().parse_args(spec.argv("ble", {"camera": "usb", "gain": 0.5}, 8123))
    assert a.camera == "usb" and a.gain == 0.5 and a.camera_port == ""


def test_flydeck_lists_it():
    out = subprocess.run(
        [sys.executable, "-m", "flydeck", "list"],
        check=True,
        capture_output=True,
        text=True,
        timeout=120,
        cwd=ROOT,
    )
    line = next((ln for ln in out.stdout.splitlines() if "fly-pursuit" in ln), "")
    assert line, out.stdout + out.stderr
    assert "error" not in line.lower()


def test_benchmark_numbers_cover_every_controller():
    b = benchmark_summary()
    assert set(b["models"]) == set(BENCHMARK_NAMES) == set(CONTROLLER_KEYS)
    assert b["episodes"]["all"] == 114


def test_readme_table_is_the_benchmark_file():
    """The README's numbers are read from pursuit_benchmark.json, not retyped."""
    readme = (DEMO / "README.md").read_text()
    for key, m in benchmark_summary()["models"].items():
        row = f"{m['all']['mae_deg']:.2f} | {m['fov30']['mae_deg']:.2f} | "
        row += f"{m['fov60']['mae_deg']:.2f} | {m['fov100']['mae_deg']:.2f}"
        assert row in readme, (key, row)


@pytest.mark.parametrize(
    "argv",
    [
        ["--mock", "--ble", "Bittle"],
        ["--mock", "--port", "/dev/null-robot"],
        ["--port", "/dev/null-robot", "--ble"],
        ["--port", "/dev/null-robot", "--pan-limit", "130"],
        ["--port", "/dev/null-robot", "--camera-fov", "0"],
        ["--mock", "--camera-off"],
        ["--port", "/dev/null-robot", "--gain", "3"],
        ["--port", "/dev/null-robot", "--camera-port", "/dev/null-cam"],
        ["--camera", "usb"],
    ],
)
def test_cli_refuses_bad_combinations(argv, capsys, monkeypatch):
    """Refused before any link is opened (the fake factory would fail the test otherwise)."""
    monkeypatch.setenv("FLYDECK_STATUS", "1")

    def no_link():
        raise AssertionError("a link was opened")

    assert cli.main(argv, make_transport=no_link) == 2
    out, _ = capsys.readouterr()
    events = [json.loads(ln[8:]) for ln in out.splitlines() if ln.startswith("FLYDECK ")]
    assert events and events[-1]["event"] == "failed"


def test_cli_refuses_bad_arousal():
    assert cli.main(["--mock", "--arousal", "3"]) == 2


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_end_to_end_smoke():
    """Start the real program, fetch the page and one frame, send a command, stop it."""
    port = _free_port()
    env = dict(os.environ, FLYDECK_STATUS="1")
    p = subprocess.Popen(
        [sys.executable, "-m", "fly_pursuit.cli", "--mock", "--no-open", "--http-port", str(port)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )
    try:
        ready = None
        for line in p.stdout:  # wait for the "ready" status line
            if line.startswith("FLYDECK ") and json.loads(line[8:])["event"] == "ready":
                ready = json.loads(line[8:])["url"]
                break
        assert ready == f"http://127.0.0.1:{port}/"
        page = urllib.request.urlopen(ready, timeout=10).read().decode()
        assert "<title>Fly pursuit</title>" in page
        frame = json.loads(urllib.request.urlopen(ready + "api/state", timeout=10).read())
        assert frame["controller"] == "connectome" and "activity" in frame
        req = urllib.request.Request(
            ready + "api/command",
            data=json.dumps({"action": "fov", "deg": 60}).encode(),
            headers={"Content-Type": "application/json"},
        )
        assert json.loads(urllib.request.urlopen(req, timeout=10).read()) == {"ok": True}
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:  # the next published frame shows it
            frame = json.loads(urllib.request.urlopen(ready + "api/state", timeout=10).read())
            if frame["fov_deg"] == 60:
                break
        assert frame["fov_deg"] == 60
    finally:
        p.send_signal(2)  # SIGINT, as Flydeck's Stop
        out, err = p.communicate(timeout=30)
    assert p.returncode == 0, err
    assert "stopped after" in out
