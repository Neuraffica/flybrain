"""The fly escape module on the robot's board: its "@FE ...;" messages, and the simulator."""

import time

import pytest
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport
from flybrain_edge.bittle import protocol as P
from flybrain_edge.bittle.onboard import OnboardEscape

# What the board prints (USB) for "XEP", a hand approach, and the escape, with the firmware's
# other prints in between (measured lines from the robot, @FE lines from flyEscape.h)
USB = (
    "+  enable\tFly_Escape\r\n"
    "S,\tA,\tT,\tL,\tD,\tI,\tB,\tU,\tG,\tC,\tQ,\tE,\t\r\n"
    "0,\t1,\t0,\t0,\t0,\t0,\t0,\t0,\t0,\t0,\t0,\t1,\t\r\n"
    "@FE on tel=0;\r\nup\r\n@FE base L=3576 R=3565;\r\n@FE on tel=1;\r\nX\r\n"
    "@FE t 3574 3566 0.00 0.00;\r\n@FE t 2210 3561 1.31 0.02;\r\n"
    "@FE trig side=1 drive=1.31;\r\nbkF\r\nk\r\nup\r\nk\r\n@FE done;\r\n"
    "Low power: 6.10V\r\n@FE t 3580 3570 0.00 0.00;\r\n"
)
KINDS = ["status", "base", "status", "t", "t", "trig", "done", "t"]


def test_messages_parse():
    msgs = P.OnboardEscapeReader().feed(USB)
    assert [m.kind for m in msgs] == KINDS
    assert msgs[0] == P.OnboardEscapeMsg("status", enabled=True, telemetry=False)
    assert msgs[1].left == 3576 and msgs[1].right == 3565
    assert msgs[2].telemetry is True
    assert (msgs[4].left, msgs[4].right, msgs[4].gf_left, msgs[4].gf_right) == (
        2210,
        3561,
        1.31,
        0.02,
    )
    assert (msgs[5].side, msgs[5].drive) == (1, 1.31)
    assert P.parse_onboard_escape("off") == P.OnboardEscapeMsg("status", False, False)
    assert P.parse_onboard_escape("future thing=1") is None  # unknown kinds are skipped
    assert P.parse_onboard_escape("t 1 2 x 4") is None


@pytest.mark.parametrize("piece", [1, 3, 10, 17])
def test_any_fragmentation_and_no_newlines(piece):
    """BLE: pieces of any size, newlines dropped (B10_250330) or not, still the same messages."""
    for text in (USB, USB.replace("\r\n", ""), USB.replace("\r\n", "\n\n")):
        r = P.OnboardEscapeReader()
        got = []
        for i in range(0, len(text), piece):
            got += r.feed(text[i : i + piece])
        assert [m.kind for m in got] == KINDS, (piece, text[:40])


def test_glued_neighbours_and_chatter():
    text = "ICM:  -1.3  -0.8   9.9  130.8   -7.6   -4.5\t@FE trig side=-1 drive=1.05;X"
    msgs = P.OnboardEscapeReader().feed(text)
    assert len(msgs) == 1 and msgs[0].side == -1
    assert P.split_glued("@FE on tel=0;X") == ["@FE on tel=0;", "X"]
    assert P.split_glued("k@FE done;3453R") == ["k", "@FE done;", "3453", "R"]
    assert P.split_glued("B10_250330?") == ["B10_250330", "?"]  # unchanged behaviour
    # chatter that looks a bit like it is ignored; a broken message does not eat the next one
    r = P.OnboardEscapeReader()
    assert r.feed("e-mail me@FEx; @FE t 1 2; Run delay\t5\r\n@FE base L=1") == []
    assert [m.kind for m in r.feed(" R=2;")] == ["base"]


def test_module_list():
    lines = [ln for ln in USB.split("\r\n")][:3]
    mods = P.parse_module_list(lines)
    assert mods["E"] is True and mods["A"] is True and mods["C"] is False
    assert P.parse_module_list(["X"]) is None  # Bluetooth: the list is USB only


def connect(**kw):
    mock = MockBittle(**kw)
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=2.0, quiet_s=0.3)
    return mock, bot, OnboardEscape(bot)


def test_stock_firmware_gets_only_the_query():
    mock, bot, on = connect()
    assert on.query() is False and not on.enabled
    on.set_telemetry(False)  # module off: nothing is sent
    bot.close(rest=False)
    assert [c for c in mock.commands if c.startswith("X")] == ["X?"]
    assert "d" not in mock.commands  # close(rest=False) sends no rest


def test_stock_firmware_over_ble_cannot_tell():
    _mock, bot, on = connect(ble_notifications=True)
    assert on.query() is None  # no module list and no status line over Bluetooth
    bot.close(rest=False)


@pytest.mark.parametrize(
    "link",
    [{}, {"ble_notifications": True}, {"ble_notifications": True, "ble_chunk": 10}],
    ids=["usb", "ble-b10_250330", "ble-10-byte-pieces"],
)
def test_simulated_module_end_to_end(link):
    mock, bot, on = connect(fly_escape=True, **link)
    try:
        assert on.query() is True and on.reports and on.enabled is False
        assert on.set_enabled(True) and on.enabled and on.baseline is not None
        assert mock.pose == "stand"  # the module's own "up" at activation
        assert on.set_telemetry(True) and on.telemetry
        mock.hand.run_script(
            lambda t: (80, 1) if t > 1.4 else (max(5, 60 - 140 * t), 1), mock.clock()
        )
        deadline = time.monotonic() + 6
        msgs = []
        while time.monotonic() < deadline and "done" not in [m.kind for _, m in msgs]:
            msgs += on.drain()
            time.sleep(0.05)
        kinds = [m.kind for _, m in msgs]
        assert "trig" in kinds and "done" in kinds and kinds.count("t") >= 3
        trig = next(m for _, m in msgs if m.kind == "trig")
        assert trig.side == 1 and trig.drive >= 1.0
        assert "kbk" not in "".join(mock.commands)  # the board walked back by itself
        assert on.set_enabled(False) and on.enabled is False and on.telemetry is False
    finally:
        bot.close(rest=False)
    assert mock.commands[-1] == "Xe" and mock.fe_triggers == 1


def test_enabling_another_module_switches_the_escape_off():
    mock, bot, on = connect(fly_escape=True)
    try:
        on.set_enabled(True)
        bot.send(P.encode_extension("C"), echo="X")  # the firmware keeps one sensor module
        assert on.enabled is False and mock.camera_on
    finally:
        bot.close(rest=False)
