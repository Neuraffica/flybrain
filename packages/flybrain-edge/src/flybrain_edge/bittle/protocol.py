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
T_READ = "R"  # R a <pin> a <pin> ... ~ : per pin "=" then the reading, then echo 'R'
T_POWER = "P"  # prints "Voltage: 8.10 V" (UNVERIFIED on hardware; the simulator copies it)
TYPE_ANALOG = "a"  # (src/OpenCat.h:364-367, src/reaction.h:1064-1131)
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
# A real Bittle X+Arm (firmware B10_250330) advertised "BittleC2_SSP" over LE, i.e. the name of
# its classic-Bluetooth SSP interface, so both suffixes are accepted.
BLE_NAME_SUFFIXES = ("_BLE", "_SSP")
BLE_NAME_PREFIX = "Bittle"  # genBleID(): "Bittle" + 2 random hex digits, user-renamable
BLE_WRITE_CHUNK = 20  # ardSerial.py slices at 20 bytes (fits the default 23-byte ATT MTU)
BLE_NOTIFY_CHUNK = 10  # firmware notifies in 10-byte pieces (src/io.h:24-28)


def ble_stem(name: str) -> str:
    """The robot's name without the _BLE / _SSP suffix ("BittleA7_SSP" -> "BittleA7"). The same
    robot is seen under either suffix (macOS caches the name), so this is what identifies it."""
    for suffix in BLE_NAME_SUFFIXES:
        if name.upper().endswith(suffix):
            return name[: -len(suffix)]
    return name


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


def encode_analog_read(pins: list[int] | tuple[int, ...]) -> bytes:
    """One 'R' command reading several analog pins, e.g. pins (36, 39) -> b"Ra$a'~".

    The pin goes in as one raw byte, so it must stay below 126 ('~' ends the command). The
    firmware prints "=" and the reading for each pair (src/reaction.h:1101-1112). Note what the
    same code path also does (it is shared with the binary joint tokens): after a skill it
    leaves the token unset, so an 'R' sent while a gait is running stops the gait, and when
    the previous token was not a skill it re-asserts the current joint angles as a posture."""
    if not pins:
        raise ValueError("no pins")
    out = bytearray(T_READ.encode())
    for p in pins:
        if not 0 <= int(p) < 126:
            raise ValueError(f"pin {p} cannot be sent in an 'R' command")
        out += TYPE_ANALOG.encode() + bytes([int(p)])
    return bytes(out + b"~")


_INT_RE = re.compile(r"-?\d+")


def parse_analog_reply(lines: list[str], n: int) -> list[int] | None:
    """Readings from the lines received for an 'R' command with ``n`` pins, or None.

    USB: "=", "3381", "=", "3453", "R". BLE may deliver the same prints as separate
    notifications or glued ("=3381", "3453R"); other prints in between are skipped."""
    vals: list[int] = []
    expect = False
    for raw in lines:
        for ln in split_glued(raw.strip()):
            t = ln.strip()
            if t.startswith("="):
                rest = t[1:].strip()
                if _INT_RE.fullmatch(rest):
                    vals.append(int(rest))
                    expect = False
                else:
                    expect = True
            elif expect and _INT_RE.fullmatch(t):
                vals.append(int(t))
                expect = False
    return vals[:n] if len(vals) >= n else None


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
# Firmware B10_250330 (seen on a real Bittle X+Arm) prints acceleration with ONE decimal and
# spaces between fields: "ICM:  -1.3  -0.8   9.9  130.8   -7.6   -4.5\t". Accept 1 or 2.
_ACC = r"\s*([-+]?\d+\.\d{1,2})(?!\d)"
_ANG = r"\s*([-+]?\d+\.\d)(?!\d)"
IMU_RE = re.compile(r"(ICM|MCU):" + _ACC * 3 + _ANG * 3)


# The firmware's yaw is an unbounded running total. Once it reaches 10000 it overflows its
# %7.1f field and glues onto the third acceleration value: "... 9.910565.4 ..." (seen on a
# real robot after about 4 h powered). The first two acceleration fields are still separated,
# so their number of decimals says where the third one ends.
def _imu_glued_re(n: int) -> re.Pattern[str]:
    acc_sep = rf"\s*([-+]?\d+\.\d{{{n}}})(?!\d)"
    acc_glued = rf"\s*([-+]?\d+\.\d{{{n}}})"
    return re.compile(r"(ICM|MCU):" + acc_sep * 2 + acc_glued + _ANG * 3)


IMU_GLUED_RES = (_imu_glued_re(1), _imu_glued_re(2))
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
    for rx in (IMU_RE, *IMU_GLUED_RES, IMU_RE_NOACC):
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


# '?' reply. USB: model and version on their own lines, then '?'. BLE on firmware B10_250330:
# one notification "Bittle X+ArmB10_250330?" (no separators, echo glued on). Version is
# BOARD + "_" + DATE (src/OpenCat.h:70-80, 666), e.g. B10_250330, B01_261024, B_261024.
VERSION_RE = re.compile(r"B\d{0,2}_\d{6}")
_GLUED_ID_RE = re.compile(
    r"^(?P<model>.*?(?:Bittle|Nybble|DoF16|Chero).*?)\s*(?P<ver>B\d{0,2}_\d{6})(?P<echo>\?)?$"
)


_VERSION_ECHO_RE = re.compile(r"^([A-Z]\d{2}_\d{6})\?$")


_GLUED_NUM_ECHO_RE = re.compile(r"^(=?-?\d+)([A-Za-z?])$")


def split_glued(line: str) -> list[str]:
    """Split prints that a BLE notification glued together into the lines USB would show.

    Today this is the identify reply "Bittle X+ArmB10_250330?" -> ["Bittle X+Arm",
    "B10_250330", "?"], and on-board fly escape messages glued to a neighbour
    ("@FE on tel=0;X" -> ["@FE on tel=0;", "X"]). (IMU text glued to an echo, "ICM: ...\tg",
    is split by parse_imu.) Lines that do not match are returned unchanged."""
    if FE_PREFIX in line:
        parts = [p for p in FE_SPLIT_RE.split(line) if p.strip()]
        if len(parts) > 1:
            return [q for p in parts for q in ([p] if p.startswith(FE_PREFIX) else split_glued(p))]
    # Seen live over BLE: the version arrives as its own 10-byte notification ("B10_250330"),
    # which the framer joins to the next print, the "?" echo -> "B10_250330?".
    v = _VERSION_ECHO_RE.match(line.strip())
    if v:
        return [v.group(1), "?"]
    m = _GLUED_NUM_ECHO_RE.match(line.strip())
    if m:  # a reading with the echo glued on (BLE): "3453R" -> ["3453", "R"]
        return [m.group(1), m.group(2)]
    m = _GLUED_ID_RE.match(line.strip())
    if not m or not m.group("model").strip():
        return [line]
    out = [m.group("model").strip(), m.group("ver")]
    if m.group("echo"):
        out.append("?")
    return out


def parse_voltage(lines: list[str]) -> float | None:
    """Battery voltage from a 'P' reply ("Voltage: 8.10 V") or a "Low power: 2.18V" warning."""
    for line in reversed(lines):
        m = re.search(r"(?:Voltage|Low power)\s*:\s*([0-9]+(?:\.[0-9]+)?)", line)
        if m:
            return float(m.group(1))
    return None


def parse_identify(lines: list[str]) -> tuple[str | None, str | None]:
    """(model, version) from the lines received after '?', in USB or BLE form."""
    flat = [x for ln in lines for x in split_glued(ln)]
    for i, ln in enumerate(flat):
        if any(k in ln for k in ("Bittle", "Nybble", "DoF16", "Chero")) and not ln.startswith(
            "Petoi "
        ):
            ver = flat[i + 1].strip() if i + 1 < len(flat) else ""
            return ln.strip(), ver or None
    return None, None


def parse_joint_table(lines: list[str]) -> dict[int, int] | None:
    """The 'j' reply: '=', a row of joint indices, a row of angles (tab separated; firmware
    B10_250330 also puts a comma after each angle). Returns {index: angle} or None."""
    rows = []
    started = False
    for ln in lines:
        t = ln.strip()
        if t == "=":
            started, rows = True, []
            continue
        if not started or not t:
            continue
        nums = [x for x in re.split(r"[\s,]+", t) if x]
        if nums and all(re.fullmatch(r"-?\d+", x) for x in nums):
            rows.append([int(x) for x in nums])
        if len(rows) == 2:
            idx, ang = rows
            if len(idx) == len(ang):
                return dict(zip(idx, ang, strict=True))
            return None
    return None


# --- the fly escape module on the robot (firmware/bittle/fly_escape, extension letter E) -----
# Not in stock firmware. Every message is one print to all ports, "@FE <what>;" (README there):
#   @FE on tel=0;   @FE off;   @FE base L=3595 R=3580;   @FE trig side=-1 drive=1.05;
#   @FE done;       @FE t 3590 3580 0.06 0.01;   (raw L, raw R, giant fibre L, R / threshold)
# The ';' ends a message even where Bluetooth drops or moves the newline, so the reader below
# works on the raw text stream, across notification boundaries and glued neighbours.
FE_PREFIX = "@FE"
FE_MESSAGE_RE = re.compile(r"@FE ([^;@]{0,60});")
FE_SPLIT_RE = re.compile(r"(@FE [^;@]{0,60};)")
EXTENSION_FLY_ESCAPE = "E"
# module list printed by "X?" (USB only, src/moduleManager.h showModuleStatus): letters, then 0/1
_MODULE_LETTERS_RE = re.compile(r"^(?:[A-Z],\s*)+$")
_MODULE_FLAGS_RE = re.compile(r"^(?:[01],\s*)+$")


@dataclass(frozen=True)
class OnboardEscapeMsg:
    """One message of the on-board module. ``kind``: "status" (enabled, telemetry), "base"
    (left, right = resting levels), "trig" (side, drive), "done", or "t" (left, right = raw
    readings, gf_left, gf_right = giant-fibre activity as a fraction of its threshold)."""

    kind: str
    enabled: bool | None = None
    telemetry: bool | None = None
    left: float | None = None
    right: float | None = None
    side: int | None = None
    drive: float | None = None
    gf_left: float | None = None
    gf_right: float | None = None


def parse_onboard_escape(body: str) -> OnboardEscapeMsg | None:
    """The part between "@FE " and ";" -> a message, or None if it is not one we know."""
    w = body.split()
    if not w:
        return None
    kv = dict(x.split("=", 1) for x in w[1:] if "=" in x)
    try:
        if w[0] == "on":
            return OnboardEscapeMsg("status", enabled=True, telemetry=kv.get("tel") == "1")
        if w[0] == "off":
            return OnboardEscapeMsg("status", enabled=False, telemetry=False)
        if w[0] == "base":
            return OnboardEscapeMsg("base", left=float(kv["L"]), right=float(kv["R"]))
        if w[0] == "trig":
            return OnboardEscapeMsg("trig", side=int(kv["side"]), drive=float(kv["drive"]))
        if w[0] == "done":
            return OnboardEscapeMsg("done")
        if w[0] == "t" and len(w) == 5:
            a, b, c, d = (float(x) for x in w[1:])
            return OnboardEscapeMsg("t", left=a, right=b, gf_left=c, gf_right=d)
    except (KeyError, ValueError):
        return None
    return None


class OnboardEscapeReader:
    """Pull "@FE ...;" messages out of the robot's output, fed in any pieces (USB reads, BLE
    notifications cut every 10 bytes, newlines missing or inserted, other prints glued on)."""

    def __init__(self) -> None:
        self._buf = ""

    def feed(self, text: str) -> list[OnboardEscapeMsg]:
        buf = self._buf + text
        out, end = [], 0
        for m in FE_MESSAGE_RE.finditer(buf):
            msg = parse_onboard_escape(" ".join(m.group(1).split()))
            if msg is not None:
                out.append(msg)
            end = m.end()
        tail = buf[end:]
        at = tail.rfind("@")
        # keep only a possible start of a message ("@", "@F", "@FE t 35"); drop the rest
        self._buf = tail[at:] if at >= 0 and len(tail) - at <= 64 else ""
        return out


def parse_module_list(lines: list[str]) -> dict[str, bool] | None:
    """{letter: active} from an "X?" reply ("S,\\tA,\\t...", then "0,\\t1,\\t..."), or None.
    The firmware prints this list to USB only, so over Bluetooth there is none."""
    for i, ln in enumerate(lines[:-1]):
        if _MODULE_LETTERS_RE.match(ln.strip()) and _MODULE_FLAGS_RE.match(lines[i + 1].strip()):
            letters = re.findall(r"[A-Z]", ln)
            flags = re.findall(r"[01]", lines[i + 1])
            if len(letters) == len(flags):
                return {k: v == "1" for k, v in zip(letters, flags, strict=True)}
    return None


class NotificationFramer:
    """Turn BLE notifications into a byte stream with line boundaries.

    Firmware B10_250330 sends every print as its own notification and never sends a newline
    (capture: tests/data/ble_probe_b10_250330.txt), so a notification boundary IS a line
    boundary: append "\n" to each one. Newer source (src/io.h:11-48) appends "\r\n" and splits
    prints into 10-byte notifications; once any newline is seen the stream is passed through
    untouched, and until then a notification of exactly BLE_NOTIFY_CHUNK bytes is taken as the
    first piece of a split print rather than a whole line. (A complete 10-byte print on old
    firmware is therefore joined to the next one; none occurs in normal use of this kit.)"""

    def __init__(self, chunk: int = BLE_NOTIFY_CHUNK):
        self.chunk = chunk
        self.newline_mode = False

    def frame(self, data: bytes) -> bytes:
        if not data:
            return b""
        if b"\n" in data or b"\r" in data:
            self.newline_mode = True
        if self.newline_mode or len(data) == self.chunk:
            return data
        return data + b"\n"


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
