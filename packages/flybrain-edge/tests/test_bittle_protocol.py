"""Parser tests on output shaped exactly like the firmware's print statements."""

import struct

import pytest
from flybrain_edge.bittle import protocol as P

# Built with the same format strings as src/imu.h:599 and :610 (OpenCatEsp32 f5aba6b).
ICM_FMT = "ICM:%6.2f%6.2f%6.2f%7.1f%7.1f%7.1f\t"
MCU_FMT = "MCU:%6.2f%6.2f%6.2f%7.1f%7.1f%7.1f"


def test_icm_line_roundtrip():
    line = ICM_FMT % (0.12, -0.34, 9.81, -12.3, 1.5, -0.7)
    (r,), rest = P.parse_imu(line)
    assert r.source == "ICM"
    assert (r.yaw_deg, r.pitch_deg, r.roll_deg) == (-12.3, 1.5, -0.7)
    assert r.accel == (0.12, -0.34, 9.81)
    assert rest.strip() == ""


def test_fixed_width_fields_that_touch():
    # %6.2f of -10.05 is 6 chars wide: no space before it. Angles at +-180 are 6 of 7 chars.
    line = ICM_FMT % (-0.34, -10.05, -10.12, -179.9, -45.0, 180.0)
    assert "-0.34-10.05-10.12" in line
    (r,), _ = P.parse_imu(line)
    assert r.accel == (-0.34, -10.05, -10.12)
    assert (r.yaw_deg, r.pitch_deg, r.roll_deg) == (-179.9, -45.0, 180.0)


def test_wide_positive_numbers_run_together():
    line = ICM_FMT % (100.25, 0.0, 9.9, 123.4, 0.0, 0.0)
    (r,), _ = P.parse_imu(line)
    assert r.accel[0] == 100.25 and r.yaw_deg == 123.4


def test_both_imus_on_one_line_prefers_mpu():
    line = ICM_FMT % (0, 0, 10, 10.0, 0, 0) + MCU_FMT % (0, 0, 9.8, 11.5, 0, 0)
    rs, _ = P.parse_imu(line)
    assert [r.source for r in rs] == ["ICM", "MCU"]
    assert P.preferred_reading(rs).yaw_deg == 11.5


def test_ble_glued_line_still_yields_echo():
    # Over BLE print6Axis sends no newline, so the next print ends up on the same line.
    line = (ICM_FMT % (0.05, -0.1, 9.95, 42.0, 0.3, -0.2)) + "g"
    (r,), rest = P.parse_imu(line)
    assert r.yaw_deg == 42.0
    assert P.is_echo(rest, "g")


def test_no_accel_build_and_legacy():
    (r,), _ = P.parse_imu(f"ICM{5.0:7.1f}{1.0:7.1f}{2.0:7.1f}\t")
    assert r.yaw_deg == 5.0 and r.accel is None
    (r,), _ = P.parse_imu("ypr\t-33.20\t1.00\t2.00")
    assert r.source == "ypr" and r.yaw_deg == -33.2


@pytest.mark.parametrize(
    "line",
    [
        "Ready!",
        "Bittle X+Arm",
        "B10_261024",
        "Undefined token!",
        "Run delay\t5",
        "0\t1\t2\t3\t4\t5\t6\t7\t8\t9\t10\t11\t12\t13\t14\t15\t",
        "0\t60\t0\t0\t-3\t-3\t3\t3\t75\t75\t75\t75\t-55\t-55\t-55\t-55\t",
        "Gyro state: Update-1 Balance-1 Print-0 Frequency-1",
        "\x00\xff\xfe garbage",
        "MPU offsets: 1\t2\t3",
        "Turning target reached! Current yaw: \t12.30",
    ],
)
def test_unrelated_lines_are_not_imu(line):
    rs, rest = P.parse_imu(line)
    assert rs == [] and rest == line


def test_echo_matching():
    assert P.is_echo("k\r", "k")
    assert P.is_echo("G", "g")  # 'g' token echoes 'G' when it turns something on
    assert not P.is_echo("kup", "k")


def test_encoders():
    assert P.encode_ascii("i", 0, 30) == b"i0 30\n"
    assert P.encode_skill("vtL") == b"kvtL\n"
    assert P.encode_extension("Cp") == b"XCp~"
    b = P.encode_binary("I", [0, -45])
    assert b[:1] == b"I" and b[-1:] == b"~" and struct.unpack("bb", b[1:3]) == (0, -45)
    # 126 is '~' and would terminate the command early: clamp to 125
    assert P.encode_binary("I", [0, 126])[2] == 125
    assert P.encode_joint(0, 29.6) == b"i0 30\n"
    with pytest.raises(ValueError):
        P.encode_joint(0, 1, "q")


def test_clamp_joint():
    assert P.clamp_joint(0, 500) == 120
    assert P.clamp_joint(1, -50) == -10  # arm lift with ROBOT_ARM
    assert P.clamp_joint(0, float("nan")) == 0.0
    assert P.clamp_joint(0, 90, {0: (-80, 80)}) == 80


def test_line_splitter_variants():
    sp = P.LineSplitter()
    assert sp.feed(b"Bittle X+A") == []
    assert sp.feed(b"rm\r\nB10_261024\r\n?\r") == ["Bittle X+Arm", "B10_261024", "?"]
    assert sp.feed(b"\nk\nd\r") == ["k", "d"]  # "\r" then "\n" split across chunks
    assert sp.pending() == ""


def test_take_pending_imu_for_unterminated_ble_text():
    sp = P.LineSplitter()
    sp.feed((ICM_FMT % (0, 0, 10.0, 77.7, 0, 0)).encode())
    (r,) = sp.take_pending_imu()
    assert r.yaw_deg == 77.7
    assert sp.pending().strip() == ""
    # a half-received line is left alone
    sp.feed(b"ICM:  0.01  0.02 10.00   12")
    assert sp.take_pending_imu() == []


def test_camera_experimental_parse():
    # showRecognitionResult: x-120, y-120, "size = ", w, h (floats print with 2 decimals)
    c = P.parse_camera("-35.00\t12.00\tsize = 40\t52\t")
    assert c == P.CameraTarget(-35.0, 12.0, 40, 52)
    assert P.parse_camera("Ready!") is None


def test_parse_imu_real_b10_250330_line():
    """Captured from a real Bittle X+Arm, firmware B10_250330, over USB."""
    from flybrain_edge.bittle import protocol as P

    readings, rest = P.parse_imu("ICM:  -1.3  -0.8   9.9  130.8   -7.6   -4.5\t")
    assert len(readings) == 1
    r = readings[0]
    assert (r.source, r.yaw_deg, r.pitch_deg, r.roll_deg) == ("ICM", 130.8, -7.6, -4.5)
    assert r.accel == (-1.3, -0.8, 9.9)
    assert rest.strip() == ""
