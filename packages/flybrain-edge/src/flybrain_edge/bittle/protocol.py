"""OpenCat (ESP32) serial protocol: tokens, joint map, command encoding, output parsing.

Everything here was read from the firmware source, not from memory. References are to
PetoiCamp/OpenCatEsp32 at commit f5aba6b (DATE "261024"); see ../../../NOTES.md for the
full list with line numbers. Things that could not be confirmed from source are marked
UNVERIFIED and are surfaced by ``flybrain-bittle probe`` instead of being guessed silently.
"""

from __future__ import annotations

import math
import re
import struct
from dataclasses import dataclass, field

BAUD = 115200

# --- tokens (src/OpenCat.h:293-393) -------------------------------------------------------
T_QUERY = "?"  # prints MODEL and SoftwareVersion lines, then echoes '?'
T_REST = "d"  # rest posture + shut servos; prints 'g' (balance off) then echoes 'd'
T_GYRO = "g"  # gyro family; see GYRO_* below
T_JOINTS = "j"  # "=" line, index line, angle line, then echo 'j'
T_SKILL = "k"  # k<name>; prints the matched table name, echoes 'k'
T_INDEXED_SIMULTANEOUS_ASC = "i"  # i idx ang idx ang ... '\n'
T_INDEXED_SIMULTANEOUS_BIN = "I"  # I <int8 idx> <int8 ang> ... '~'
T_EXTENSION = "X"  # module switch, e.g. XC~ camera on, Xc~ off
T_LEGACY_PRINT_GYRO = "v"  # AVR OpenCat / old ESP32 firmware only; "Undefined token!" now

# 'g' sub-options (src/OpenCat.h:315-328, handled in src/reaction.h:501-618)
GYRO_PRINT_ONCE = "gp"  # print one IMU line (rate-limited, see IMU_MIN_PRINT_INTERVAL_S)
GYRO_PRINT_CONTINUOUS = "gP"  # print an IMU line whenever the IMU updates (still rate-limited)
GYRO_BALANCE_OFF = "gb"  # stop balance reflexes, IMU keeps updating (what we want)
GYRO_BALANCE_ON = "gB"
GYRO_STATE = "g?"  # "Gyro state: Update-1 Balance-0 Print-1 Frequency-1"
# NOT used: bare "g" toggles (unknown resulting state); "gu" stops IMU updates entirely.

# print6Axis() refuses to print more often than every 200 ms (src/imu.h:579-592), so the
# host sees at most ~5 IMU lines per second, whatever it asks for.
IMU_MIN_PRINT_INTERVAL_S = 0.200

# Firmware command buffer for ASCII tokens (src/moduleManager.h:380-450): reading stops at the
# terminator or after SERIAL_TIMEOUT=10 ms of silence. Two commands that arrive in one burst
# are read as ONE command and the second is truncated at its '\n', so the host must space
# commands out (wait for the echo, or at least ~20 ms).
FIRMWARE_SERIAL_TIMEOUT_S = 0.010

# --- BLE (src/bleCommon.h:6-8, src/bleUart.h, src/io.h:23-28) ------------------------------
BLE_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
BLE_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # host -> robot (write / write-nr)
BLE_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # robot -> host (notify)
BLE_NAME_SUFFIX = "_BLE"  # advertised name = uniqueName + "_BLE", e.g. "BittleA3_BLE"
BLE_NAME_PREFIX = "Bittle"  # genBleID(): "Bittle" + 2 random hex digits, user-renamable
BLE_WRITE_CHUNK = 20  # ardSerial.py slices at 20 bytes (fits the default 23-byte ATT MTU)
BLE_NOTIFY_CHUNK = 10  # firmware notifies in 10-byte pieces (src/io.h:24-28)

# --- joints for Bittle X + Arm (ROBOT_ARM build) ---------------------------------------------
# Index map: OpenCat index 0 = head pan (the arm's base rotates with it), 1 = arm lift (the
# "head tilt" slot), 2 = claw/pincer (the "tail" slot; src/OpenCat.h:128, docs robot-arm page
# "c-2" auto-calibrates the claw, src/reaction.h:912-931). 8-15 are legs.
JOINT_HEAD_PAN = 0
JOINT_ARM = 1
JOINT_CLAW = 2

# angleLimit for BITTLE with ROBOT_ARM (src/OpenCat.h:580-593). The firmware clamps to these in
# calibratedPWM() (src/motion.h:9); we clamp tighter on the host anyway.
FIRMWARE_ANGLE_LIMITS: dict[int, tuple[int, int]] = {
    0: (-120, 120),
    1: (-10, 180),  # with ROBOT_ARM; (-85, 85) on a plain Bittle X
    2: (-120, 120),
    3: (-120, 120),
    4: (-90, 60),
    5: (-90, 60),
    6: (-90, 90),
    7: (-90, 90),
    8: (-200, 80),
    9: (-200, 80),
    10: (-80, 200),
    11: (-80, 200),
    12: (-80, 200),
    13: (-80, 200),
    14: (-80, 200),
    15: (-80, 200),
}
# Host-side safe range for the head pan in this demo (well inside the firmware limit, so the
# arm cannot swing into the front legs). UNVERIFIED on hardware: tune with --pan-limit.
DEFAULT_PAN_LIMIT_DEG = 80.0

# Sign of the head pan joint. UNVERIFIED: inferred from skill.h ("offsetLR = 30" for skills
# ending in 'L', i.e. the head looks left on left turns) => positive pan = look left = CCW.
DEFAULT_PAN_SIGN = +1

# --- skills present in InstinctBittleESP_arm.h --------------------------------------------
# 'R' variants are mirrored from the stored 'L' table by loadBySkillName (src/skill.h:440-500).
SKILL_STAND = "up"  # same angles as "balance"; the firmware itself returns to "up" after gaits
SKILL_BALANCE = "balance"
SKILL_REST = "rest"
SKILL_SIT = "sit"
SKILL_TURN_LEFT = "vtL"  # stepping in place, turning left (CCW)
SKILL_TURN_RIGHT = "vtR"


# --- encoding -------------------------------------------------------------------------------
def encode_ascii(token: str, *args: object) -> bytes:
    """Lower-case tokens are ASCII, newline terminated (src/moduleManager.h:400)."""
    body = " ".join(str(a) for a in args)
    return (token + body + "\n").encode("ascii")


def encode_binary(token: str, values: list[int]) -> bytes:
    """Upper-case tokens are int8 payloads terminated by '~' (ardSerial.py:114-135).

    '~' is byte 126, so angles must stay <= 125 (src/moduleManager.h:401 comment).
    """
    vals = [max(-128, min(125, int(v))) for v in values]
    return token.encode("ascii") + struct.pack("b" * len(vals), *vals) + b"~"


def encode_skill(name: str) -> bytes:
    return encode_ascii(T_SKILL + name)


def encode_extension(code: str) -> bytes:
    """'X' is upper case, so the firmware waits for '~' (or a 10 ms gap)."""
    return ("X" + code + "~").encode("ascii")


def encode_joint(index: int, angle_deg: float, token: str = "i") -> bytes:
    a = round(angle_deg)
    if token == "I":
        return encode_binary("I", [index, a])
    if token in ("i", "m"):
        return encode_ascii(token, index, a)
    raise ValueError(f"unsupported joint token {token!r}")


def clamp_joint(
    index: int, angle_deg: float, limits: dict[int, tuple[float, float]] | None = None
) -> float:
    lo, hi = (limits or FIRMWARE_ANGLE_LIMITS).get(
        index, FIRMWARE_ANGLE_LIMITS.get(index, (-90, 90))
    )
    if math.isnan(angle_deg):
        return 0.0
    return max(lo, min(hi, angle_deg))


# --- parsing --------------------------------------------------------------------------------
# print6Axis() (src/imu.h:599-611):
#   "ICM:%6.2f%6.2f%6.2f%7.1f%7.1f%7.1f\t"  ax ay az (x GRAVITY=10) yaw pitch roll
#   "MCU:%6.2f%6.2f%6.2f%7.1f%7.1f%7.1f"
# then PTL() -> "\r\n" on USB only. Over BLE the newline is NOT sent (printToAllPorts(buf, 0)),
# so the IMU text gets glued to whatever prints next, e.g. "ICM: ...\tg". Fields are fixed
# width with no separator, so "-0.34-10.05" is two numbers; the regex keys on the exact
# number of decimals (2 for accel, 1 for angles) to split them.
_ACC = r"\s*([-+]?\d+\.\d{2})"
_ANG = r"\s*([-+]?\d+\.\d)(?!\d)"
IMU_RE = re.compile(r"(ICM|MCU):" + _ACC * 3 + _ANG * 3)
IMU_RE_NOACC = re.compile(r"(ICM|MCU)(?!:)" + _ANG * 3)  # build without PRINT_ACCELERATION
# Legacy/other builds: "ypr\t<yaw>\t<pitch>\t<roll>" (src/imu.h:395-399, OUTPUT_READABLE_YAWPITCHROLL)
LEGACY_YPR_RE = re.compile(
    r"ypr\s+([-+]?\d+(?:\.\d+)?)\s+([-+]?\d+(?:\.\d+)?)\s+([-+]?\d+(?:\.\d+)?)"
)


@dataclass(frozen=True)
class ImuReading:
    source: str  # "ICM" (ICM42670), "MCU" (MPU6050) or "ypr" (legacy)
    yaw_deg: float  # firmware convention: positive = counter-clockwise (src/imu.h:705)
    pitch_deg: float
    roll_deg: float
    accel: tuple[float, float, float] | None = None  # firmware units (about 10 per g)


def parse_imu(text: str) -> tuple[list[ImuReading], str]:
    """Extract every IMU reading in ``text``; return (readings, text with them removed)."""
    readings: list[ImuReading] = []
    rest = text
    for rx in (IMU_RE, IMU_RE_NOACC):
        out = []
        last = 0
        for m in rx.finditer(rest):
            g = m.groups()
            if len(g) == 7:
                acc = (float(g[1]), float(g[2]), float(g[3]))
                y, p, r = float(g[4]), float(g[5]), float(g[6])
            else:
                acc = None
                y, p, r = float(g[1]), float(g[2]), float(g[3])
            readings.append(ImuReading(g[0], y, p, r, acc))
            out.append(rest[last : m.start()])
            last = m.end()
        out.append(rest[last:])
        rest = "".join(out)
    m = LEGACY_YPR_RE.search(rest)
    if m:
        readings.append(ImuReading("ypr", float(m.group(1)), float(m.group(2)), float(m.group(3))))
        rest = rest[: m.start()] + rest[m.end() :]
    return readings, rest


def preferred_reading(readings: list[ImuReading]) -> ImuReading | None:
    """Both IMUs can print on one line; the firmware's global ypr comes from the MPU6050 when
    present (readIMU overwrites ICM values with MPU values, src/imu.h:694-729)."""
    for src in ("MCU", "ICM", "ypr"):
        for r in readings:
            if r.source == src:
                return r
    return None


def is_echo(line: str, token: str) -> bool:
    """Commands are acknowledged by printing the token back on its own line
    (src/reaction.h:1579-1581; ardSerial.py printSerialMessage compares case-insensitively)."""
    return line.strip().lower() == token.lower()


# Camera (EXPERIMENTAL): showRecognitionResult() (src/camera.h:248-261) prints
#   "<x - W/2>\t<y - H/2>\tsize = <w>\t<h>\t"   (W = H = 240 for the Grove Vision AI V2)
# after "XCp~" (once, preceded by an "=" line) or continuously after "XCP~".
CAMERA_RE = re.compile(
    r"^\s*([-+]?\d+(?:\.\d+)?)\t([-+]?\d+(?:\.\d+)?)\tsize = (\d+)(?:\t(\d+))?\t?\s*$"
)


@dataclass(frozen=True)
class CameraTarget:
    """EXPERIMENTAL. x/y are pixels from the image centre in a 240x240 frame. The direction of
    +x relative to the robot (left or right) is UNVERIFIED and depends on module mounting."""

    x: float
    y: float
    width: int
    height: int | None


def parse_camera(line: str) -> CameraTarget | None:
    m = CAMERA_RE.match(line)
    if not m:
        return None
    h = m.group(4)
    return CameraTarget(
        float(m.group(1)), float(m.group(2)), int(m.group(3)), int(h) if h else None
    )


@dataclass
class LineSplitter:
    """Turn a byte stream into text lines. Accepts \\n, \\r\\n and lone \\r. Never raises on
    garbage (boot ROM noise at 74880 baud shows up as junk at 115200)."""

    _buf: bytearray = field(default_factory=bytearray)
    max_line: int = 4096

    _after_cr: bool = False

    def feed(self, data: bytes) -> list[str]:
        if self._after_cr and data[:1] == b"\n":
            data = data[1:]  # second half of a "\r\n" split across reads
        self._after_cr = False
        self._buf.extend(data)
        lines: list[str] = []
        while True:
            idx = -1
            for i, b in enumerate(self._buf):
                if b in (10, 13):
                    idx = i
                    break
            if idx < 0:
                break
            raw = bytes(self._buf[:idx])
            # swallow "\r\n" as one terminator
            end = idx + 1
            if self._buf[idx] == 13 and end < len(self._buf) and self._buf[end] == 10:
                end += 1
            if self._buf[idx] == 13 and end == len(self._buf):
                self._after_cr = True
            del self._buf[:end]
            lines.append(raw.decode("latin-1"))
        if len(self._buf) > self.max_line:
            lines.append(bytes(self._buf).decode("latin-1"))
            self._buf.clear()
        return lines

    def take_pending_imu(self) -> list[ImuReading]:
        """Over BLE the IMU text arrives without a newline. Because every field has a fixed
        number of decimals, a regex match on the unterminated tail is already complete, so cut
        it out now instead of waiting (possibly seconds) for the next newline."""
        text = self.pending()
        if "ICM" not in text and "MCU" not in text:
            return []
        readings, rest = parse_imu(text)
        if readings:
            self._buf = bytearray(rest.encode("latin-1"))
        return readings

    def pending(self) -> str:
        return bytes(self._buf).decode("latin-1")

    def flush(self) -> str:
        s = self.pending()
        self._buf.clear()
        return s
