"""The real demos' Flydeck specs: choices match the CLIs, argv per transport parses."""

import pytest
from fly_compass import cli as compass_cli
from fly_compass import flydeck as compass_deck
from fly_compass.loop import LEG_MODES, MODES
from fly_compass.models import MODEL_KEYS as COMPASS_MODELS
from fly_compass.scenarios import SCRIPT_CHOICES
from fly_escape import app as escape_app
from fly_escape import cli as escape_cli
from fly_escape import flydeck as escape_deck
from fly_escape.escape import ACTIONS
from fly_escape.models import MODEL_KEYS as ESCAPE_MODELS
from hello_robot import cli as hello_cli
from hello_robot import flydeck as hello_deck

PARSERS = {
    "fly-compass": compass_cli.build_parser,
    "fly-escape": escape_cli.build_parser,
    "hello-robot": hello_cli.build_parser,
}
SPECS = {
    "fly-compass": compass_deck.SPEC,
    "fly-escape": escape_deck.SPEC,
    "hello-robot": hello_deck.SPEC,
}


def test_choices_match_the_programs():
    assert compass_deck.MODES == MODES
    assert compass_deck.LEG_MODES == LEG_MODES
    assert compass_deck.DRIVERS == COMPASS_MODELS
    assert compass_deck.MOCK_SCRIPTS == SCRIPT_CHOICES
    assert escape_deck.MODES == escape_app.MODES
    assert escape_deck.DRIVERS == ESCAPE_MODELS
    assert escape_deck.ACTIONS == ACTIONS


@pytest.mark.parametrize("demo", sorted(SPECS))
@pytest.mark.parametrize(
    ("transport", "robot", "flag"),
    [
        ("mock", None, "mock"),
        ("usb", "/dev/cu.usbmodem1101", "port"),
        ("ble", "BittleC2_BLE", "ble"),
    ],
)
def test_argv_parses_with_the_demo_cli(demo, transport, robot, flag):
    spec = SPECS[demo]
    spec.validate()
    argv = spec.argv(transport, {}, 8899, robot, space_confirmed=True)
    a = PARSERS[demo]().parse_args(argv)
    assert a.http_port == 8899 and a.no_open
    if transport == "mock":
        assert a.mock
    else:
        assert getattr(a, flag) == robot


def test_leg_moving_settings_need_space():
    c, e, h = SPECS["fly-compass"], SPECS["fly-escape"], SPECS["hello-robot"]
    assert not c.needs_space({"mode": "needle"}, "ble")
    assert c.needs_space({"mode": "sweep"}, "ble") and c.needs_space({"mode": "hold"}, "usb")
    assert e.needs_space({"mode": "escape"}, "ble") and not e.needs_space(
        {"mode": "observe"}, "ble"
    )
    assert h.needs_space({"walk": True}, "usb") and not h.needs_space({}, "usb")
    for s in (c, e, h):
        assert not s.needs_space({}, "mock")
    # observe on a real robot: no space flag at all
    assert "--i-have-space" not in e.argv("ble", {"mode": "observe"}, 1, "B", True)
    a = escape_cli.build_parser().parse_args(e.argv("ble", {"mode": "escape"}, 1, "B", True))
    assert a.i_have_space and a.mode == "escape"


def test_mock_only_options_are_not_passed_to_a_real_robot():
    e = SPECS["fly-escape"]
    assert "--mock-random" in e.argv("mock", {"mock_random": True}, 1)
    assert "--mock-random" not in e.argv("usb", {"mock_random": True}, 1, "/dev/x", True)
    c = SPECS["fly-compass"]
    assert "--mock-script" not in c.argv("ble", {}, 1, "B")
