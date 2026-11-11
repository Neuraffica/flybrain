import io
import time

from flybrain_edge.bittle import Bittle, MockBittle, MockTransport
from flybrain_edge.bittle.probe import Probe


def make(**kw):
    mock = MockBittle(**kw)
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=1.0, quiet_s=0.3)
    return mock, bot


def test_connect_identifies_firmware():
    _mock, bot = make()
    try:
        assert bot.model == "Bittle X+Arm"
        assert bot.version == "B10_261024"
        assert "Ready!" in bot.boot_text
    finally:
        bot.close()


def test_stand_stream_and_close_rests():
    mock, bot = make()
    bot.stand()
    assert mock.commands[-2:] == ["kup", "gb"]
    assert mock.balance is False and mock.update_gyro is True
    bot.stream_imu(True)
    time.sleep(1.1)
    n = len(bot.drain_samples())
    assert 4 <= n <= 7, n  # print6Axis is limited to one line per 200 ms
    bot.close()
    assert mock.commands[-1] == "d"
    assert mock.skill == "rest"
    assert mock.dropped_bytes == 0


def test_read_yaw_follows_robot_and_sign():
    mock, bot = make(imu_noise_deg=0.0)
    try:
        mock.nudge(30)
        time.sleep(0.4)
        s = bot.read_yaw()
        assert s is not None and abs(s.yaw_deg - 30) < 1.0
    finally:
        bot.close()
    mock2 = MockBittle(imu_noise_deg=0.0)
    bot2 = Bittle(MockTransport(mock2), yaw_sign=-1)
    bot2.connect(boot_timeout=1.0, quiet_s=0.3)
    try:
        mock2.nudge(30)
        time.sleep(0.4)
        time.sleep(0.25)
        assert abs(bot2.read_yaw().yaw_deg + 30) < 1.0
    finally:
        bot2.close()


def test_set_joint_is_clamped_and_rate_limited():
    mock, bot = make()
    try:
        bot.stand()
        assert bot.set_joint(0, 200) == 80  # host pan limit
        assert bot.set_joint(1, -90) == -10  # arm lift firmware limit
        t0 = time.monotonic()
        k = 0
        while time.monotonic() - t0 < 1.0:
            bot.set_joint(0, -60 + (k % 120))
            k += 1
            time.sleep(0.005)
        time.sleep(0.1)
        n_i = sum(1 for c in mock.commands if c.startswith("i"))
        assert n_i <= 20, n_i  # ~15/s, never one per call
        assert mock.max_head_cmd <= 80
        assert mock.dropped_bytes == 0
    finally:
        bot.close()


def test_binary_joint_token():
    mock = MockBittle()
    bot = Bittle(MockTransport(mock), joint_token="I")
    bot.connect(boot_timeout=1.0, quiet_s=0.3)
    try:
        bot.set_joint(0, -25)
        time.sleep(0.3)
        assert any(c.startswith("I") for c in mock.commands)
        assert mock.angles[0] == -25
    finally:
        bot.close()


def test_tolerates_unsolicited_prints():
    _mock, bot = make(chatter_per_s=40.0)
    try:
        bot.stand()
        bot.stream_imu(True)
        time.sleep(0.6)
        assert bot.stats.echo_timeouts == 0
        assert bot.drain_samples()
        assert bot.stats.other_lines > 0
    finally:
        bot.close()


def test_ble_framing_without_newlines():
    _mock, bot = make(ble_framing=True)
    try:
        bot.stand()
        bot.stream_imu(True)
        time.sleep(0.7)
        assert len(bot.drain_samples()) >= 2
    finally:
        bot.close()


def test_probe_on_mock_passes():
    out = io.StringIO()
    code = Probe(MockTransport(MockBittle()), out=out).run(stream_s=1.0, boot_s=1.0)
    text = out.getvalue()
    assert code == 0, text
    assert "model:    Bittle X+Arm" in text
    assert "Undefined token!" in text  # legacy 'v' on current firmware
