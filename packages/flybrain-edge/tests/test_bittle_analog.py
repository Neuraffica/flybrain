"""Analog reads with the binary 'R' token: captured replies on both framings, concurrency, the
simulated proximity sensor, and the gait-stopping side effect."""

import threading
import time

import pytest
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport
from flybrain_edge.bittle import protocol as P
from flybrain_edge.bittle.mock import ProximityHand


def test_encoding_matches_captures():
    # captured on a real Bittle X+Arm, firmware B10_250330, USB
    assert P.encode_analog_read([36]) == b"Ra$~"
    assert P.encode_analog_read([36, 39]) == b"Ra$a'~"
    assert P.encode_analog_read([34, 35, 36, 39]) == b"Ra\"a#a$a'~"
    with pytest.raises(ValueError):
        P.encode_analog_read([126])
    with pytest.raises(ValueError):
        P.encode_analog_read([])


@pytest.mark.parametrize(
    "lines",
    [
        ["=", "3376", "=", "3453", "R"],  # USB capture, split into lines
        ["=", "3376", "=", "3453R"],  # BLE: echo glued to the last reading
        ["=3376", "=3453", "R"],  # BLE: marker glued to the reading
        ["Run delay\t5", "=", "3376", "Head overlay: hi", "=", "3453", "R"],  # chatter
    ],
)
def test_parse_reply_variants(lines):
    assert P.parse_analog_reply(lines, 2) == [3376, 3453]


def test_parse_reply_incomplete():
    assert P.parse_analog_reply(["=", "3376", "R"], 2) is None
    assert P.parse_analog_reply(["R"], 1) is None


def test_reader_on_captured_usb_bytes():
    """The reader turns the captured USB bytes into lines that parse."""
    bot = Bittle(MockTransport(MockBittle()))
    bot.handle_bytes(b"=\r\n3376\r\n=\r\n3453\r\nR\r\n")
    lines = [t for _, t in bot.recent_lines]
    assert P.parse_analog_reply(lines, 2) == [3376, 3453]
    assert P.is_echo(lines[-1], "R")


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_read_analog_on_mock(link):
    mock = MockBittle(ble_notifications=link == "ble")
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=3.0, quiet_s=0.3)
    try:
        vals = bot.read_analog([36, 39])
        assert set(vals) == {36, 39} and all(3200 < v < 3700 for v in vals.values())
        four = bot.read_analog([34, 35, 36, 39])
        assert four[34] < 50 and four[35] < 50  # unconnected pins read about 0
        mock.hand.set(5.0, 0.0)
        close = bot.read_analog([36, 39])
        assert all(v < 50 for v in close.values())
        assert bot.stats.echo_timeouts == 0
    finally:
        bot.close()


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_poll_analog_shares_the_line(link):
    """Analog polling, IMU polling and head commands run together; every reply parses, nothing
    is merged, and pausing stops the reads."""
    mock = MockBittle(ble_notifications=link == "ble", merge_window_s=0.03)
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=3.0, quiet_s=0.3)
    try:
        bot.stand()
        bot.poll_analog([36, 39], 40)
        bot.poll_imu(10)
        stop = threading.Event()

        def heads():
            a = 0
            while not stop.is_set():
                bot.set_joint(0, a % 30)
                a += 7
                time.sleep(0.05)

        th = threading.Thread(target=heads, daemon=True)
        th.start()
        time.sleep(2.0)
        stop.set()
        th.join()
        got = bot.drain_analog()
        assert len(got) > 5  # the line is shared: each command waits for its echo
        assert all(set(s.values) == {36, 39} for s in got)
        assert bot.stats.analog_errors == 0
        assert mock.merged_writes == []
        bot.pause_analog(True)
        n = bot.stats.analog_samples
        time.sleep(0.5)
        assert bot.stats.analog_samples == n
        bot.pause_analog(False)
        time.sleep(0.4)
        assert bot.stats.analog_samples > n
    finally:
        bot.close()


def test_read_during_a_gait_stops_it_like_the_firmware():
    mock = MockBittle()
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=3.0, quiet_s=0.3)
    try:
        assert bot.skill("bkF")
        assert mock.period > 1
        bot.read_analog([36])
        assert mock.period == 1 and mock.gaits_stopped_by_read == 1
    finally:
        bot.close()


def test_simulated_sensor_band_and_sides():
    import random

    hand = ProximityHand(random.Random(0))
    hand.set(80.0, 0.0)
    assert hand.raw("L", 0) > 3200 and hand.raw("R", 0) > 3200
    hand.set(5.0, 0.0)
    assert hand.raw("L", 0) < 50 and hand.raw("R", 0) < 50
    hand.set(16.0, 0.0)  # inside the narrow graded band
    assert 600 < hand.raw("L", 0) < 2800
    hand.set(16.0, 1.0)  # same distance, on the robot's left: the left channel drops first
    assert hand.raw("L", 0) < 200 < hand.raw("R", 0)
    hand.backed_cm = 40.0  # the dog backed away
    assert hand.raw("L", 0) > 3200
