"""Command spacing on the link: a command must never be written while the previous one may
still be unread by the firmware, or the two merge and one is lost. Over BLE that lost the stand
at the end of a sweep leg on a real robot."""

import time

import pytest
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport
from flybrain_edge.bittle.robot import (
    BLE_MIN_GAP_S,
    BLE_UNACKED_GUARD_S,
    USB_MIN_GAP_S,
    USB_UNACKED_GUARD_S,
)

MERGE_WINDOW_S = 0.08  # wider than a 20 Hz IMU poll interval's slack


def make(link, **kw):
    mock = MockBittle(ble_notifications=link == "ble", imu_noise_deg=0.0, **kw)
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=3.0, quiet_s=0.3)
    return mock, bot


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_mock_reproduces_the_lost_stop(link):
    """Two raw writes in one burst: over BLE the stand is lost and the gait keeps going;
    over USB the stand survives and the second command is dropped."""
    mock, bot = make(link, merge_window_s=MERGE_WINDOW_S)
    try:
        assert bot.skill("vtL")
        time.sleep(0.3)
        bot.transport.write(b"kup\n")  # bypasses the link's spacing on purpose
        bot.transport.write(b"gp\n")
        time.sleep(0.8)
        assert mock.merged_writes == [b"kup\ngp\n"]
        assert mock.skill == ("vtL" if link == "ble" else "up")
    finally:
        bot.close()


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_unwaited_stand_is_never_merged(link):
    """skill(wait=False) returns at once but nothing is written until its echo, even with
    IMU polling and head commands running in other threads."""
    mock, bot = make(link, merge_window_s=MERGE_WINDOW_S, turn_rate_deg_s=20.0)
    try:
        bot.stand()
        bot.poll_imu(20)
        for _ in range(3):
            t0 = time.monotonic()
            assert bot.skill("vtL", wait=False)
            assert time.monotonic() - t0 < 0.05  # the control loop is not blocked
            for a in (10, -10, 20):
                bot.set_joint(0, a)
                time.sleep(0.1)
            t0 = time.monotonic()
            assert bot.skill("up", wait=False)
            assert time.monotonic() - t0 < 0.05
            bot.set_joint(0, 0)
            time.sleep(1.0)
            assert mock.skill == "up" and mock.period == 1, "the stand was lost"
        assert mock.merged_writes == []
        assert bot.stats.echo_timeouts == 0
    finally:
        bot.close()
    assert mock.commands[-1] == "d"


def test_timing_defaults_per_transport():
    usb = Bittle(MockTransport(MockBittle()))
    ble = Bittle(MockTransport(MockBittle(ble_notifications=True)))
    assert (usb.min_gap, usb.unacked_guard_s) == (USB_MIN_GAP_S, USB_UNACKED_GUARD_S)
    assert (ble.min_gap, ble.unacked_guard_s) == (BLE_MIN_GAP_S, BLE_UNACKED_GUARD_S)
    assert Bittle(MockTransport(MockBittle()), min_command_gap_s=0.07).min_gap == 0.07


def test_usb_timing_is_unaffected():
    """Echo-confirmed commands still go out back to back at the USB pace."""
    mock, bot = make("usb")
    try:
        t0 = time.monotonic()
        for _ in range(10):
            assert bot.send(b"g?\n", echo="g", timeout=1.0) is not None
        assert time.monotonic() - t0 < 10 * (USB_MIN_GAP_S + 0.03)
        t0 = time.monotonic()
        bot.skill("vtL", wait=False)
        bot.skill("up", wait=False)
        assert time.monotonic() - t0 < 0.05
        assert bot.flush_commands(5.0)
        assert mock.commands[-2:] == ["kvtL", "kup"]
    finally:
        bot.close()


def test_guard_after_a_missing_echo():
    """If a command's echo never comes, the next write waits for the guard interval."""
    tx = []
    mock = MockBittle()
    bot = Bittle(MockTransport(mock), on_raw=lambda d, t, b: tx.append(t) if d == "tx" else None)
    bot.connect(boot_timeout=3.0, quiet_s=0.3)
    try:
        bot.send(b"g?\n", echo="never", timeout=0.05)
        n = len(tx)
        bot.send(b"g?\n", echo="g", timeout=1.0)
        assert tx[n] - tx[n - 1] >= USB_UNACKED_GUARD_S - 0.01
    finally:
        bot.close()
