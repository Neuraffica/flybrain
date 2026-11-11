"""A simulated Bittle X + Arm that speaks the same serial dialect as OpenCatEsp32.

It is a behavioural model of the parts of the firmware this kit uses, written from the source
(see NOTES.md): command framing, token echoes, the rate-limited IMU print, the 'g' options,
head slewing inside a posture, in-place turning gaits, rest. It is NOT a physics simulator.

The dog's world yaw can be driven three ways (highest priority first):
  * a scripted trajectory ``trajectory(t_seconds) -> yaw_deg``;
  * the turning gaits it is told to run (``kvtL`` / ``kvtR``);
  * an external target set by the dashboard / tests (``set_world_yaw``, ``nudge``), approached
    at a hand-like slew rate.
"""

from __future__ import annotations

import math
import random
import struct
import threading
import time
from collections.abc import Callable

from . import protocol as P

MODEL = "Bittle X+Arm"
VERSION = "B10_261024"
BOOT_LINES = [
    "k",
    "Flush the serial buffer...",
    "",
    "* Start *",
    MODEL,
    "Software version: " + VERSION,
    "Buzzer volume: 5/10",
    "Initializing ICM42670...",
    "Creating IMU task...",
    "IMU task created successfully",
    "BLE:\tBittleA3_BLE",
    "Waiting for a BLE client connection to notify...",
    "Ready!",
]
# Strings the firmware prints on its own during normal use (copied from the source).
CHATTER = [
    "Run delay\t5",  # reaction.h:651
    "Completed cycle: \t1 / \t0",  # skill.h:420 (gaits)
    "Bluetooth BLE disconnected!",  # bleUart.h:142
    "Head overlay: hi",  # reaction.h:1458
]
SKILLS_ARM = {  # name -> period (1 = posture, >1 = gait), from InstinctBittleESP_arm.h
    "up": 1,
    "balance": 1,
    "rest": 1,
    "sit": 1,
    "str": 1,
    "buttUp": 1,
    "calib": 1,
    "zero": 1,
    "vtL": 72,
    "vtF": 72,
    "wkL": 40,
    "wkF": 40,
    "crL": 40,
    "crF": 40,
    "trL": 40,
    "trF": 40,
    "bkL": 40,
    "bkF": 40,
    "hi": -5,
    "pee": -5,
    "pick": -5,
}
REST_ANGLES = [0, 60, 0, 0, -3, -3, 3, 3, 75, 75, 75, 75, -55, -55, -55, -55]
UP_ANGLES = [0, 0, 0, 0, 0, 0, 0, 0, 25, 25, 20, 20, 50, 50, 50, 50]


def wrap_deg(a: float) -> float:
    a = math.fmod(a + 180.0, 360.0)
    if a <= 0:
        a += 360.0
    return a - 180.0


class MockBittle:
    def __init__(
        self,
        initial_world_yaw_deg: float = 37.0,
        imu_noise_deg: float = 0.25,
        imu_drift_deg_s: float = 0.0,
        turn_rate_deg_s: float = 38.0,
        hand_slew_deg_s: float = 240.0,
        imu: str = "ICM",
        ble_framing: bool = False,
        chatter_per_s: float = 0.0,
        seed: int | None = 0,
        clock: Callable[[], float] = time.monotonic,
        tick_s: float = 0.005,
    ):
        self.rng = random.Random(seed)
        self.clock = clock
        self.tick_s = tick_s
        self.imu_noise = imu_noise_deg
        self.imu_drift = imu_drift_deg_s
        self.turn_rate = turn_rate_deg_s
        self.hand_slew = hand_slew_deg_s
        self.imu_kind = imu
        self.ble_framing = ble_framing
        self.chatter_per_s = chatter_per_s

        self._lock = threading.RLock()
        self._out = bytearray()
        self._out_cv = threading.Condition(self._lock)
        self._thread: threading.Thread | None = None
        self._running = False

        self.world_yaw = initial_world_yaw_deg
        self._boot_world_yaw = initial_world_yaw_deg
        self._hand_target: float | None = None
        self._hand_rate = 0.0
        self.trajectory: Callable[[float], float] | None = None
        self._t0 = clock()
        self._last_t = self._t0
        self._busy_until = 0.0
        self._pending_echo: list[tuple[float, str]] = []
        self._inbox: list[bytes] = []

        # firmware state
        self.update_gyro = True
        self.balance = True
        self.print_continuous = False
        self._last_print = -1e9
        self.token = ""
        self.skill = "rest"
        self.period = 1
        self.angles = list(REST_ANGLES)
        self.head_target = [0.0, 60.0, 0.0, 0.0]
        self.manual_head = False
        self.camera_on = False
        self.powered = False
        self.turning_target: float | None = None
        # bookkeeping for tests
        self.commands: list[str] = []
        self.dropped_bytes = 0
        self.max_head_cmd = 0.0

    # ---- lifecycle -------------------------------------------------------------------
    def power_on(self) -> None:
        with self._lock:
            self.powered = True
            self._t0 = self._last_t = self.clock()
            for line in BOOT_LINES:
                self._emit(line)

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="mock-bittle", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None

    def _run(self) -> None:
        while self._running:
            self.advance()
            time.sleep(self.tick_s)

    # ---- external drive --------------------------------------------------------------
    def set_world_yaw(self, deg: float) -> None:
        """Move the dog (by hand) towards this world heading."""
        with self._lock:
            # choose the shortest way round from the current heading
            self._hand_target = self.world_yaw + wrap_deg(deg - self.world_yaw)

    def nudge(self, deg: float) -> None:
        with self._lock:
            base = self._hand_target if self._hand_target is not None else self.world_yaw
            self._hand_target = base + deg

    def set_hand_rate(self, deg_s: float) -> None:
        with self._lock:
            self._hand_rate = deg_s

    def clock_origin(self) -> float:
        """Clock value at power-on: ``trajectory`` is called with ``clock() - clock_origin()``."""
        return self._t0

    def imu_yaw(self) -> float:
        """What the IMU would report now, without noise (firmware convention, CCW positive)."""
        with self._lock:
            t = self._last_t - self._t0
            return wrap_deg(self.world_yaw - self._boot_world_yaw + self.imu_drift * t)

    # ---- byte interface --------------------------------------------------------------
    def receive(self, data: bytes) -> None:
        with self._lock:
            self._inbox.append(bytes(data))
        if not self._running:
            self.advance()

    def read_output(self, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        with self._out_cv:
            while not self._out:
                if not self._running:
                    self.advance()
                    if self._out:
                        break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return b""
                self._out_cv.wait(min(remaining, 0.01))
            data = bytes(self._out)
            self._out.clear()
            return data

    def _emit(self, text: str, newline: bool = True) -> None:
        self._out.extend((text + ("\r\n" if newline else "")).encode("latin-1"))
        self._out_cv.notify_all()

    # ---- simulation step -------------------------------------------------------------
    def advance(self, now: float | None = None) -> None:
        with self._lock:
            now = self.clock() if now is None else now
            dt = max(0.0, now - self._last_t)
            self._last_t = now
            t = now - self._t0
            if now >= self._busy_until:
                while self._inbox and now >= self._busy_until:
                    self._handle_write(self._inbox.pop(0), now)
            for due, tok in list(self._pending_echo):
                if now >= due:
                    self._emit(tok)
                    self._pending_echo.remove((due, tok))
            # world yaw
            if self.trajectory is not None:
                self.world_yaw = self.trajectory(t)
            else:
                if self.period > 1 and self.skill in ("vtL", "vtR") and self.powered:
                    rate = self.turn_rate if self.skill == "vtL" else -self.turn_rate
                    self.world_yaw += rate * dt * (1 + 0.05 * self.rng.uniform(-1, 1))
                    if self.turning_target is not None:
                        done = (rate > 0 and self.imu_yaw_raw() >= self.turning_target) or (
                            rate < 0 and self.imu_yaw_raw() <= self.turning_target
                        )
                        if done:
                            self.turning_target = None
                            self._load("up", now)
                            self._emit("k")
                if self._hand_rate:
                    self.world_yaw += self._hand_rate * dt
                    self._hand_target = None
                if self._hand_target is not None:
                    err = self._hand_target - self.world_yaw
                    step = math.copysign(min(abs(err), self.hand_slew * dt), err)
                    self.world_yaw += step
                    if abs(err) < 1e-6:
                        self._hand_target = None
            # head slew inside the skill loop: <=20 deg per frame (skill.h:415); ~20 ms frames
            if self.token == "k" and self.manual_head:
                max_step = 20.0 * dt / 0.02
                for j in range(4):
                    d = self.head_target[j] - self.angles[j]
                    self.angles[j] += math.copysign(min(abs(d), max_step), d)
            # continuous IMU print (rate-limited exactly like print6Axis)
            if self.print_continuous and self.update_gyro and self.powered:
                self._print6axis(now)
            if self.chatter_per_s and self.rng.random() < self.chatter_per_s * dt:
                self._emit(self.rng.choice(CHATTER))
            _ = t

    def imu_yaw_raw(self) -> float:
        # unwrapped, for turning-angle bookkeeping
        return self.world_yaw - self._boot_world_yaw

    # ---- firmware behaviour --------------------------------------------------------
    def _print6axis(self, now: float) -> None:
        if not self.update_gyro:
            return
        if now - self._last_print < P.IMU_MIN_PRINT_INTERVAL_S:
            return
        self._last_print = now
        t = now - self._t0
        yaw = wrap_deg(
            self.world_yaw
            - self._boot_world_yaw
            + self.imu_drift * t
            + self.rng.gauss(0, self.imu_noise)
        )
        pitch = self.rng.gauss(0.5, 0.2)
        roll = self.rng.gauss(-0.8, 0.2)
        ax, ay, az = (
            self.rng.gauss(0.05, 0.03),
            self.rng.gauss(-0.1, 0.03),
            self.rng.gauss(10.0, 0.05),
        )
        parts = []
        if self.imu_kind in ("ICM", "both"):
            parts.append(f"ICM:{ax:6.2f}{ay:6.2f}{az:6.2f}{yaw:7.1f}{pitch:7.1f}{roll:7.1f}\t")
        if self.imu_kind in ("MCU", "both"):
            parts.append(f"MCU:{ax:6.2f}{ay:6.2f}{az:6.2f}{yaw:7.1f}{pitch:7.1f}{roll:7.1f}")
        # printToAllPorts(buffer, 0) + PTL(): the newline goes to USB only
        self._emit("".join(parts), newline=not self.ble_framing)

    def _load(self, name: str, now: float) -> bool:
        base = name
        if name.endswith("R") and name[:-1] + "L" in SKILLS_ARM:
            base = name[:-1] + "L"
        if base not in SKILLS_ARM:
            return False
        self._emit(base)  # SkillList::lookUp prints the matched table name (skill.h:61-62)
        self.skill = name
        self.period = SKILLS_ARM[base]
        self.manual_head = False
        if self.period == 1:
            target = REST_ANGLES if name == "rest" else UP_ANGLES
            self.angles = list(target)
            self.head_target = [float(a) for a in target[:4]]
            # transform() blocks the firmware loop for roughly half a second
            self._busy_until = now + 0.4
        return True

    def _handle_write(self, data: bytes, now: float) -> None:
        if not data:
            return
        token = chr(data[0])
        term = b"~" if "A" <= token <= "Z" else b"\n"
        end = data.find(term, 1)
        body = data[1:end] if end >= 0 else data[1:]
        if end >= 0 and end + 1 < len(data):
            rest = data[end + 1 :].strip(b"\r\n")
            if rest:
                # the real read_serial() swallows the rest of a burst into the same buffer and
                # then truncates at the first '\n': the second command is lost.
                self.dropped_bytes += len(rest)
        self._command(token, body, now)

    def _command(self, token: str, body: bytes, now: float) -> None:
        self.commands.append(token + body.decode("latin-1", "replace"))
        text = body.decode("latin-1").strip(" \r\n") if token.islower() or token in "?X" else ""
        echo = token
        defer_echo = 0.0
        if token == "?":
            self._emit(MODEL)
            self._emit(VERSION)
        elif token == "g":
            if not text:
                self.balance = not self.balance
                echo = "G" if self.balance else "g"
            for c in text:
                if c in "bB":
                    self.balance = c == "B"
                    if not self.balance:
                        self._emit("g")
                elif c in "uU":
                    self.update_gyro = c == "U"
                    self.balance = self.update_gyro
                    if c == "u":
                        self._emit("g")
                elif c in "pP":
                    self.print_continuous = c == "P"
                    self._print6axis(now)
                elif c in "fF":
                    echo = "G" if c == "F" else "g"
                elif c == "?":
                    self._emit(
                        f"Gyro state: Update-{self.update_gyro:d} Balance-{self.balance:d} "
                        f"Print-{self.print_continuous:d} Frequency-1"
                    )
        elif token == "k":
            name, _, arg = text.partition(" ")
            prev = self.skill
            self._load(name, now)  # unknown names load nothing but still echo 'k'
            self.token = "k"
            if arg and self.period > 1 and name[-1] in "LR":
                ang = float(arg)
                self.turning_target = self.imu_yaw_raw() + (ang if name[-1] == "L" else -ang)
                return  # completion echo when the angle is reached (reaction.h:1502-1537)
            if self.period == 1 and name != prev:
                defer_echo = 0.4  # postures echo after the blocking transform
        elif token == "d":
            self.balance = False
            self._emit("g")
            self._load("rest", now)
            self.token = ""
        elif token in ("i", "m"):
            nums = [int(float(x)) for x in text.replace(",", " ").split()]
            for idx, ang in zip(nums[0::2], nums[1::2]):
                self._set_joint(idx, ang, smooth=(token == "i" and self.token == "k"))
        elif token == "I":
            vals = struct.unpack("b" * len(body), body) if body else ()
            for idx, ang in zip(vals[0::2], vals[1::2]):
                self._set_joint(idx, ang, smooth=False)
            self.token = ""  # resetCmd(): 'I' is not restored to 'k', the skill loop stops
        elif token == "j":
            self._emit("=")
            self._emit("\t".join(str(i) for i in range(16)) + "\t")
            self._emit("\t".join(str(round(a)) for a in self.angles) + "\t")
        elif token == "X":
            if text.startswith("C"):
                self.camera_on = True
            elif text.startswith("c"):
                self.camera_on = False
        elif token == "P":
            self._emit("Voltage: 8.10 V")
        elif token in ("b", "B"):
            pass
        else:
            self._emit("Undefined token!")
            self._emit(text)
        if defer_echo:
            self._pending_echo.append((now + defer_echo, echo))
        else:
            self._emit(echo)

    def _set_joint(self, idx: int, ang: float, smooth: bool) -> None:
        if not 0 <= idx < 16:
            return
        lo, hi = P.FIRMWARE_ANGLE_LIMITS[idx]
        ang = max(lo, min(hi, ang))
        if idx == P.JOINT_HEAD_PAN:
            self.max_head_cmd = max(self.max_head_cmd, abs(ang))
        if idx < 4:
            self.head_target[idx] = ang
            self.manual_head = True
            if not smooth:
                self.angles[idx] = ang
        else:
            self.angles[idx] = ang
