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
from typing import ClassVar

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
# Sensor modules of OpenCatEsp32 32a1fcb (src/OpenCat.h moduleList), as "X?" lists them. Voice
# (A) is on by default on a Bittle X. The fly escape module (firmware/bittle/fly_escape) adds E.
MODULES = "SATLDIBUGCQ"
MODULE_NAMES = {"C": "Camera", "E": "Fly_Escape"}
FE_STEP_S = 0.035  # flyEscape.h: FLY_ESCAPE_STEP_MS
FE_TEL_S = 0.2  # FLY_ESCAPE_TEL_MS
FE_MIN_INTERVAL_S = 3.0  # FLY_ESCAPE_MIN_INTERVAL_MS
FE_BACK_S = 1.0  # FLY_ESCAPE_BACK_MS
REST_ANGLES = [0, 60, 0, 0, -3, -3, 3, 3, 75, 75, 75, 75, -55, -55, -55, -55]
UP_ANGLES = [0, 0, 0, 0, 0, 0, 0, 0, 25, 25, 20, 20, 50, 50, 50, 50]


def wrap_deg(a: float) -> float:
    a = math.fmod(a + 180.0, 360.0)
    if a <= 0:
        a += 360.0
    return a - 180.0


class ProximityHand:
    """A hand in front of the simulated Petoi double infrared proximity sensor.

    Calibrated to the real sensor (USB, firmware B10_250330): about 3400 +- 50 with nothing in
    front, about 0 with a hand close, graded only over a narrow band, and the two channels fall
    at slightly different moments. ``distance_cm`` is from the sensor face; ``side`` is +1 for
    the robot's left, -1 right. ``script(t) -> (distance_cm, side)`` overrides the setting.
    While the dog walks backwards (an escape) the hand ends up further away."""

    REST: ClassVar[dict[str, float]] = {"L": 3400.0, "R": 3440.0}
    NEAR_CM = 12.0  # reads ~0 closer than this
    FAR_CM = 20.0  # reads the resting level beyond this
    SPREAD_CM = 5.0  # how much closer the hand is to the channel on its side
    NOISE = 25.0

    def __init__(self, rng):
        self.rng = rng
        self.distance_cm = 80.0
        self.side = 0.0
        self.script = None
        self.script_t0 = 0.0
        self.backed_cm = 0.0
        self.back_speed_cm_s = 15.0

    def set(self, distance_cm: float, side: float = 0.0) -> None:
        self.script = None
        self.distance_cm = max(0.0, float(distance_cm))
        self.side = max(-1.0, min(1.0, float(side)))
        self.backed_cm = 0.0

    def run_script(self, fn, now: float) -> None:
        self.script, self.script_t0, self.backed_cm = fn, now, 0.0
        self.paused_at = None

    paused_at: float | None = None

    def pause(self, now: float) -> None:
        """Freeze a running script where it is."""
        if self.paused_at is None:
            self.paused_at = now

    def resume(self, now: float) -> None:
        if self.paused_at is not None:
            self.script_t0 += now - self.paused_at
            self.paused_at = None

    def state(self, now: float) -> tuple[float, float]:
        if self.script is not None:
            t = (self.paused_at if self.paused_at is not None else now) - self.script_t0
            d, side = self.script(t)
            self.distance_cm, self.side = float(d), float(side)
        if self.distance_cm > 40.0:
            self.backed_cm = 0.0  # hand withdrawn: the next approach is to the dog's new spot
        return self.distance_cm + self.backed_cm, self.side

    def raw(self, channel: str, now: float) -> int:
        d, side = self.state(now)
        sign = 1.0 if channel == "L" else -1.0
        d_ch = d - self.SPREAD_CM * side * sign
        frac = min(1.0, max(0.0, (d_ch - self.NEAR_CM) / (self.FAR_CM - self.NEAR_CM)))
        if frac <= 0.0:
            return int(abs(self.rng.gauss(0.0, 4.0)))
        return int(max(0.0, self.REST[channel] * frac + self.rng.gauss(0.0, self.NOISE)))


class MockBittle:
    def __init__(
        self,
        initial_world_yaw_deg: float = 37.0,
        imu_noise_deg: float = 0.25,
        imu_drift_deg_s: float = 0.0,
        turn_rate_deg_s: float = 8.0,
        turn_unevenness: float = 0.4,
        gait_stall_after_s: float | None = None,
        hand_slew_deg_s: float = 240.0,
        imu: str = "ICM",
        ble_framing: bool = False,
        ble_notifications: bool = False,
        merge_window_s: float | None = None,
        chatter_per_s: float = 0.0,
        fly_escape: bool = False,
        ble_chunk: int = 0,
        seed: int | None = 0,
        clock: Callable[[], float] = time.monotonic,
        tick_s: float = 0.005,
    ):
        self.rng = random.Random(seed)
        self.clock = clock
        self.tick_s = tick_s
        self.imu_noise = imu_noise_deg
        self.imu_drift = imu_drift_deg_s
        # In-place turning on a real Bittle on carpet: 2-7 deg/s and uneven (NOTES.md).
        self.turn_rate = turn_rate_deg_s
        self.turn_unevenness = turn_unevenness  # +- fraction, varying over a few seconds
        # Test hook: a turning gait silently stops this long after it was (re)issued.
        self.gait_stall_after_s = gait_stall_after_s
        self._gait_started = 0.0
        self._uneven_phase = self.rng.uniform(0, 2 * math.pi)
        self.turn_commands = 0
        self.hand_slew = hand_slew_deg_s
        self.imu_kind = imu
        self.ble_framing = ble_framing
        # BLE as seen on firmware B10_250330 (tests/fixtures/ble_probe_b10_250330.txt): one
        # notification per print, no newlines at all, an IMU line glued to the echo that follows
        # it, the identify reply glued into "Bittle X+ArmB10_250330?". Read with read_chunks().
        self.ble_notifications = ble_notifications
        # BLE as in OpenCatEsp32 32a1fcb (src/io.h printToAllPorts): every print to all ports
        # gets "\r\n" and is notified in pieces of ble_chunk (10) bytes. 0: as B10_250330.
        self.ble_chunk = ble_chunk
        # Test hook for the firmware's command merging: a write that arrives within
        # merge_window_s of the previous one, before that one was processed, is appended to
        # it. Over USB the merged buffer loses the SECOND command (read_serial truncates at the
        # first newline); over BLE the FIRST command is broken too (the buffer is parsed as one
        # command with an embedded newline, src/bleUart.h:46-90), so e.g. "kup" + "gp" loads no
        # skill and the gait keeps running. Processing is delayed by the window to model a busy
        # firmware loop.
        self.merge_window_s = merge_window_s
        # Petoi double infrared proximity sensor on the Grove socket labelled 39/36
        self.hand = ProximityHand(self.rng)
        self.ir_pins = {36: "L", 39: "R"}  # which pin is the robot's left is not established
        self.gaits_stopped_by_read = 0
        # kinematics for a visual simulator: forward displacement since start (cm, negative =
        # moved back) and the phase of the current gait cycle (0..1)
        self.dog_x_cm = 0.0
        self.gait_phase = 0.0
        self.gait_cycle_s = 0.5
        self.merged_writes: list[bytes] = []  # every merged buffer, for tests
        self._last_rx = -1e9
        self._chunks: list[bytes] = []
        self._ble_pending: str | None = None
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
        self._inbox: list[list] = []  # [arrival time, bytes, merged?]

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
        # firmware built with the fly escape module (firmware/bittle/fly_escape): "XE" runs the
        # real escape model on the simulated sensor, as flyEscape.h does on the board
        self.fly_escape = fly_escape
        self.modules = dict.fromkeys(MODULES + ("E" if fly_escape else ""), False)
        self.modules["A"] = True
        self.fe_telemetry = False
        self.fe_triggers = 0
        self._fe_model = None
        self._fe_base = (3400.0, 3400.0)
        self._fe_calibrated = False
        self._fe_sum = [0.0, 0.0, 0]
        self._fe_last_step = self._fe_last_tel = 0.0
        self._fe_last_escape = -1e9
        self._fe_phase: tuple[str, float] | None = None  # ("back" | "up", until)
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
            if self.ble_notifications:
                # no boot banner over BLE (the board is already running); the connect greeting
                # (src/bleUart.h:160-178) as captured: "Petoi Bittle", then "p"
                self._emit("Petoi Bittle")
                self._emit("p")
                return
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
            now = self.clock()
            w = self.merge_window_s
            if w and self._inbox and now - self._last_rx < w:
                self._inbox[-1][1] += bytes(data)
                self._inbox[-1][2] = True
            else:
                self._inbox.append([now, bytes(data), False])
            self._last_rx = now
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

    def read_chunks(self, timeout: float) -> list[bytes]:
        """BLE-notification mode: the notifications sent since the last call."""
        deadline = time.monotonic() + timeout
        with self._out_cv:
            while not self._chunks:
                if not self._running:
                    self.advance()
                    if self._chunks:
                        break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return []
                self._out_cv.wait(min(remaining, 0.01))
            out, self._chunks = self._chunks, []
            return out

    def _emit(self, text: str, newline: bool = True, usb_only: bool = False) -> None:
        """One print. ``usb_only``: the firmware's PT()/PTL(), which never reach Bluetooth."""
        if self.ble_notifications and usb_only:
            return
        if self.ble_notifications and self.ble_chunk:
            data = (text + ("\r\n" if newline else "")).encode("latin-1")
            for i in range(0, len(data), self.ble_chunk):
                self._chunks.append(data[i : i + self.ble_chunk])
            self._out_cv.notify_all()
            return
        if self.ble_notifications:
            if not newline:  # IMU text: may get the next print glued on
                self._flush_ble()
                self._ble_pending = text
                return
            chunk = (self._ble_pending or "") + text
            self._ble_pending = None
            if chunk:
                self._chunks.append(chunk.encode("latin-1"))
                self._out_cv.notify_all()
            return
        self._out.extend((text + ("\r\n" if newline else "")).encode("latin-1"))
        self._out_cv.notify_all()

    def _flush_ble(self) -> None:
        if self._ble_pending:
            self._chunks.append(self._ble_pending.encode("latin-1"))
            self._ble_pending = None
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
                    if self.merge_window_s and now - self._inbox[0][0] < self.merge_window_s:
                        break  # still open for merging
                    _, data, merged = self._inbox.pop(0)
                    if merged:
                        self.merged_writes.append(data)
                    self._handle_write(data, now, merged)
            for due, tok in list(self._pending_echo):
                if now >= due:
                    self._emit(tok)
                    self._pending_echo.remove((due, tok))
            # escape gaits move the dog away from a hand in front of it
            if self.period > 1 and self.skill in ("bkF", "bkL", "bkR") and self.powered:
                self.hand.backed_cm += self.hand.back_speed_cm_s * dt
                self.dog_x_cm -= self.hand.back_speed_cm_s * dt
            if self.period > 1 and self.powered:
                self.gait_phase = (self.gait_phase + dt / self.gait_cycle_s) % 1.0
            # world yaw
            if self.trajectory is not None:
                self.world_yaw = self.trajectory(t)
            else:
                stalled = (
                    self.gait_stall_after_s is not None
                    and now - self._gait_started > self.gait_stall_after_s
                )
                if (
                    self.period > 1
                    and self.skill in ("vtL", "vtR")
                    and self.powered
                    and not stalled
                ):
                    rate = self.turn_rate if self.skill == "vtL" else -self.turn_rate
                    # slow wobble in speed (steps catch on carpet), plus a little jitter
                    wobble = self.turn_unevenness * math.sin(0.9 * t + self._uneven_phase)
                    self.world_yaw += rate * dt * (1 + wobble + 0.05 * self.rng.uniform(-1, 1))
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
            if self.modules.get("E") and self.powered:
                self._fe_poll(now)
            if self.chatter_per_s and self.rng.random() < self.chatter_per_s * dt:
                self._emit(self.rng.choice(CHATTER))
            self._flush_ble()  # an IMU line with nothing printed right after it
            _ = t

    @property
    def pose(self) -> str:
        """stand | back | turn | walk | rest, from the skill state (what the legs are doing)."""
        if not self.powered or self.skill == "rest":
            return "rest"
        if self.period > 1:
            if self.skill in ("bkF", "bkL", "bkR"):
                return "back"
            if self.skill in ("vtL", "vtR"):
                return "turn"
            return "walk"
        return "stand"

    def analog_read(self, pin: int, now: float | None = None) -> int:
        now = self.clock() if now is None else now
        ch = self.ir_pins.get(int(pin))
        if ch is None:
            return int(abs(self.rng.gauss(0.0, 3.0)))  # nothing connected reads about 0
        return self.hand.raw(ch, now)

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
        if name == "bk":
            name = "bkF"  # reaction.h:1472-1473
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

    def _handle_write(self, data: bytes, now: float, merged: bool = False) -> None:
        if not data:
            return
        token = chr(data[0])
        term = b"~" if "A" <= token <= "Z" else b"\n"
        if merged and (self.ble_notifications or self.ble_framing):
            # BLE: the whole merged buffer is one command; only its final terminator is
            # removed, so the first command's argument now contains "\n<next command>"
            body = data[1:]
            body = body.removesuffix(term)
            self._command(token, body, now)
            return
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
            if self.ble_notifications:
                self._emit(MODEL + VERSION + "?")  # one notification, as captured
                echo = ""
            else:
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
            if name in ("vtL", "vtR"):
                self.turn_commands += 1
                self._gait_started = now  # (re)issuing restarts a stalled gait
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
            nums = []
            for x in text.replace(",", " ").split():
                try:
                    nums.append(int(float(x)))
                except ValueError:
                    break  # garbage from a merged command: the firmware's atoi stops here too
            for idx, ang in zip(nums[0::2], nums[1::2]):
                self._set_joint(idx, ang, smooth=(token == "i" and self.token == "k"))
        elif token == "I":
            vals = struct.unpack("b" * len(body), body) if body else ()
            for idx, ang in zip(vals[0::2], vals[1::2]):
                self._set_joint(idx, ang, smooth=False)
            self.token = ""  # resetCmd(): 'I' is not restored to 'k', the skill loop stops
        elif token == "R":
            # R a <pin> ... (src/reaction.h:1064-1131): per pair "=" and the reading. The same
            # path leaves the token unset, so a gait that was running stops.
            vals = list(body)
            for typ, pin in zip(vals[0::2], vals[1::2]):
                self._emit("=")
                self._emit(str(self.analog_read(pin, now) if typ == ord("a") else 0))
            if self.period > 1:
                self.period = 1
                self.gaits_stopped_by_read += 1
            self.token = ""
        elif token == "j":
            self._emit("=")
            self._emit("\t".join(str(i) for i in range(16)) + "\t")
            sep = ",\t" if self.ble_notifications else "\t"  # B10_250330 over BLE: "0,\t60,\t"
            self._emit(sep.join(str(round(a)) for a in self.angles) + sep)
        elif token == "X":
            self._extension(text, now)
        elif token == "P":
            self._emit("Voltage: 8.10 V")
        elif token in ("b", "B"):
            pass
        else:
            self._emit("Undefined token!")
            self._emit(text)
        if not echo:
            return
        if defer_echo:
            self._pending_echo.append((now + defer_echo, echo))
        else:
            self._emit(echo)

    # ---- sensor modules ('X', src/moduleManager.h reconfigureTheActiveModule) ----------
    def _extension(self, text: str, now: float) -> None:
        code = text[:1]
        if code == "?":
            self._module_status()
            return
        changed = False
        for k, on in self.modules.items():
            # "X<k>" switches the others off (not voice, backtouch), "X<lower k>" only k,
            # a bare "X" all of them
            if on and (k == code.upper() if code.islower() else not code or k not in code + "AB"):
                self._emit("- disable\t" + MODULE_NAMES.get(k, k), usb_only=True)
                self.modules[k], changed = False, True
                if k == "E":  # flyEscapeStop()
                    self.fe_telemetry, self._fe_phase = False, None
        enabled = code.isupper() and not self.modules.get(code, True)
        if enabled:
            self._emit("+  enable\t" + MODULE_NAMES.get(code, code), usb_only=True)
            self.modules[code] = changed = True
        if changed:
            self._module_status()
        if enabled and code == "E":
            self._load("up", now)  # initModule: loadBySkillName("up"), then flyEscapeSetup()
            self._fe_setup(now)
        self.camera_on = self.modules["C"]
        if code == "E" and self.modules["E"]:  # flyEscapeOptions(): "XEP" / "XEp"
            said = False
            for c in text[1:]:
                if c in "Pp":
                    self.fe_telemetry, said = c == "P", True
            if said:
                self._fe_status()

    def _module_status(self) -> None:
        self._emit(",\t".join(self.modules) + ",\t", usb_only=True)
        self._emit(",\t".join(str(int(v)) for v in self.modules.values()) + ",\t", usb_only=True)
        if self.fly_escape:
            self._fe_status()

    def _fe_status(self) -> None:
        on = self.modules["E"]
        self._emit(f"@FE on tel={int(self.fe_telemetry)};" if on else "@FE off;")
        self._fe_calibrated &= on
        if self._fe_calibrated:  # as the board: the levels follow the status once measured
            self._emit(f"@FE base L={round(self._fe_base[0])} R={round(self._fe_base[1])};")

    def _fe_setup(self, now: float) -> None:
        from flybrain_models.escape import EscapeCircuit  # the model the board runs (exported)

        if self._fe_model is None:
            self._fe_model = EscapeCircuit.from_pack("escape", variant="connectome")
        self._fe_model.reset()
        n = 64
        self._fe_base = tuple(sum(self.analog_read(p, now) for _ in range(n)) / n for p in (36, 39))
        self._fe_calibrated = True
        self._emit(f"@FE base L={round(self._fe_base[0])} R={round(self._fe_base[1])};")
        self._fe_sum = [0.0, 0.0, 0]
        self._fe_last_step = now
        self._fe_last_escape = now - FE_MIN_INTERVAL_S
        self.fe_telemetry = False
        self._fe_phase = None

    def _fe_poll(self, now: float) -> None:
        """read_flyEscape(): runs only while the firmware is idle and its task queue empty."""
        from flybrain_models.escape import raw_to_proximity

        if self._fe_phase is not None:  # the queued "bk" (1 s), then "up"
            phase, until = self._fe_phase
            if now < until:
                return
            if phase == "back":
                self._load("up", now)
                self._emit("k")
                self._fe_phase = ("up", self._busy_until)
                return
            self._fe_phase = None
            self._emit("@FE done;")
        if now < self._busy_until or self._inbox:
            return
        s = self._fe_sum
        s[0] += self.analog_read(36, now)
        s[1] += self.analog_read(39, now)
        s[2] += 1
        elapsed = now - self._fe_last_step
        if elapsed < FE_STEP_S:
            return
        raw_l, raw_r = s[0] / s[2], s[1] / s[2]
        self._fe_sum = [0.0, 0.0, 0]
        self._fe_last_step = now
        m = self._fe_model
        if elapsed > 0.5:  # after a long pause the board restarts the dynamics
            m.reset()
            elapsed = FE_STEP_S
        out = m.step(
            raw_to_proximity(raw_l, self._fe_base[0]),
            raw_to_proximity(raw_r, self._fe_base[1]),
            elapsed,
        )
        if self.fe_telemetry and (now - self._fe_last_tel >= FE_TEL_S or out.trigger):
            self._fe_last_tel = now
            th_l, th_r = getattr(m, "gf_thresholds", (out.threshold, out.threshold))
            self._emit(
                f"@FE t {round(raw_l)} {round(raw_r)} {out.gf_left / th_l:.2f} "
                f"{out.gf_right / th_r:.2f};"
            )
        if out.trigger and now - self._fe_last_escape >= FE_MIN_INTERVAL_S:
            self._fe_last_escape = now
            self.fe_triggers += 1
            self._emit(f"@FE trig side={out.side} drive={out.drive:.2f};")
            self._load("bk", now)  # tQueue: "bk" for 1 s, then "up"
            self._emit("k")
            self._fe_phase = ("back", now + FE_BACK_S)

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
