"""High-level Bittle link: connect, stand, rest, stream IMU yaw, move one joint, run skills.

Threads:
  * a reader thread turns transport bytes into lines, extracts IMU readings anywhere in the
    stream (unsolicited prints are tolerated and kept in ``recent_lines``), and signals
    command echoes;
  * a joint streamer sends the latest requested joint angle at most ``joint_rate_hz`` times a
    second, so a control loop can call :meth:`Bittle.set_joint` every tick without blocking.

Every command goes through one lock and waits for its echo (or a short timeout) before the
next one is written: the firmware merges commands that arrive in one burst and drops the
second (src/moduleManager.h:405-445).
"""

from __future__ import annotations

import itertools
import logging
import queue
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Self

from . import protocol as P
from .transport import Transport, TransportError

log = logging.getLogger(__name__)


class BittleError(RuntimeError):
    pass


# Link timing. USB: the firmware reads a command until its terminator or 10 ms of silence.
# BLE: a GATT write without response returns before the robot has processed it, and while the
# firmware's loop is busy (a gait frame, a posture transition) a second write is appended to the
# first, which then fails to parse (src/bleUart.h:46-90). Hence wider spacing over BLE, and after
# any command whose echo was not seen, a guard interval before the next write.
USB_MIN_GAP_S = 0.02
BLE_MIN_GAP_S = 0.05
USB_UNACKED_GUARD_S = 0.1
BLE_UNACKED_GUARD_S = 0.5


class _Job:
    __slots__ = ("data", "done", "echo", "result", "timeout")

    def __init__(self, data: bytes, echo: str | None, timeout: float):
        self.data, self.echo, self.timeout = data, echo, timeout
        self.result: list[str] | None = None
        self.done = threading.Event()


@dataclass(frozen=True)
class ImuSample:
    t: float  # host time.monotonic() when the line was parsed
    yaw_deg: float  # firmware yaw times yaw_sign: + = counter-clockwise seen from above
    pitch_deg: float
    roll_deg: float
    source: str
    raw: str = ""


@dataclass(frozen=True)
class AnalogSample:
    t: float  # host time.monotonic() when the reply was complete
    values: dict[int, int]  # pin -> raw reading


@dataclass
class LinkStats:
    lines: int = 0
    imu_samples: int = 0
    other_lines: int = 0
    echo_timeouts: int = 0
    commands: int = 0
    imu_rate_hz: float = 0.0
    last_sample_t: float = 0.0
    analog_samples: int = 0
    analog_errors: int = 0


class Bittle:
    def __init__(
        self,
        transport: Transport,
        yaw_sign: int = +1,
        joint_token: str = "i",
        joint_rate_hz: float = 15.0,
        min_command_gap_s: float | None = None,
        joint_limits: dict[int, tuple[float, float]] | None = None,
        on_raw: Callable[[str, float, bytes], None] | None = None,
    ):
        if joint_token not in ("i", "I"):
            raise ValueError("joint_token must be 'i' (ASCII) or 'I' (binary)")
        self.transport = transport
        self.yaw_sign = 1 if yaw_sign >= 0 else -1
        self.joint_token = joint_token
        self.joint_rate_hz = joint_rate_hz
        # Link timing (see send()). BLE writes return before the robot has the bytes, and the
        # firmware appends a write that arrives before its loop consumed the previous command,
        # so BLE gets a wider spacing and a longer guard after an unconfirmed command.
        is_ble = getattr(transport, "name", "") == "ble"
        self.min_gap = (
            (BLE_MIN_GAP_S if is_ble else USB_MIN_GAP_S)
            if min_command_gap_s is None
            else min_command_gap_s
        )
        self.unacked_guard_s = BLE_UNACKED_GUARD_S if is_ble else USB_UNACKED_GUARD_S
        self._guard_until = 0.0
        self._cmd_q: queue.Queue = queue.Queue()
        self._cmd_thread: threading.Thread | None = None
        self.limits = dict(P.FIRMWARE_ANGLE_LIMITS)
        self.limits[P.JOINT_HEAD_PAN] = (-P.DEFAULT_PAN_LIMIT_DEG, P.DEFAULT_PAN_LIMIT_DEG)
        if joint_limits:
            self.limits.update(joint_limits)
        self.on_raw = on_raw

        self.model: str | None = None
        self.version: str | None = None
        self.boot_text = ""
        self.stats = LinkStats()
        self.recent_lines: deque[tuple[float, str]] = deque(maxlen=300)
        self._samples: deque[ImuSample] = deque(maxlen=2000)
        self._new_samples: deque[ImuSample] = deque(maxlen=2000)
        self._poll_thread: threading.Thread | None = None
        self.low_power: str | None = None  # last 'Low power' line from the firmware
        self.yaw_drift_dps = 0.0  # subtracted from every sample once measured
        self._drift_t0 = 0.0
        self._poll_stop = threading.Event()
        self._analog: deque[AnalogSample] = deque(maxlen=5000)
        self._analog_thread: threading.Thread | None = None
        self._analog_stop = threading.Event()
        self._analog_paused = threading.Event()
        self._analog_gate = threading.Lock()
        self._sample_listeners: list[Callable[[ImuSample], None]] = []
        self._rx_listeners: list[Callable[[bytes, float], None]] = []
        self._splitter = P.LineSplitter()
        self._cv = threading.Condition()
        self._line_seq = 0
        self._write_lock = threading.Lock()
        self._last_write = 0.0
        self._running = False
        self._reader: threading.Thread | None = None
        self._streamer: threading.Thread | None = None
        self._joint_targets: dict[int, float] = {}
        self._joint_sent: dict[int, int] = {}
        self._joint_cv = threading.Condition()
        self.streaming = False
        self.connected = False
        self.legacy_imu = False

    # ---- connection ---------------------------------------------------------------------
    def connect(self, boot_timeout: float = 12.0, quiet_s: float = 1.5) -> None:
        """Open the transport, wait out the boot banner, identify the firmware."""
        self.transport.open()
        self._wait_boot(boot_timeout, quiet_s)
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, name="bittle-reader", daemon=True)
        self._reader.start()
        self._streamer = threading.Thread(
            target=self._joint_loop, name="bittle-joints", daemon=True
        )
        self._streamer.start()
        self._cmd_thread = threading.Thread(target=self._cmd_loop, name="bittle-cmds", daemon=True)
        self._cmd_thread.start()
        self.connected = True
        lines = self.send(P.encode_ascii(P.T_QUERY), echo=P.T_QUERY, timeout=2.0)
        if lines is None:
            log.warning("no echo for '?': firmware may be busy or not OpenCat")
        else:
            # USB: model, version, '?' on separate lines; BLE (B10_250330): one glued print,
            # already split into those lines by handle_bytes
            self.model, self.version = P.parse_identify(lines)
        if self.model and "Arm" not in self.model:
            log.warning(
                "firmware reports %r, not 'Bittle X+Arm': arm joint 1 limits differ", self.model
            )

    def _wait_boot(self, max_s: float, quiet_s: float) -> None:
        buf = bytearray()
        t0 = time.monotonic()
        last = t0
        while time.monotonic() - t0 < max_s:
            chunk = self.transport.read(0.05)
            now = time.monotonic()
            if chunk:
                self._raw("rx", now, chunk)
                buf.extend(chunk)
                last = now
                text = buf.decode("latin-1")
                if "(Y/n)" in text:
                    self.boot_text = text
                    raise BittleError(
                        "The board is asking a first-boot setup question (Y/n). Finish setup in "
                        "the Arduino serial monitor or the Petoi Desktop App first:\n" + text[-400:]
                    )
                if "Ready!" in text:
                    # let the trailing beep/print settle
                    tail_end = time.monotonic() + 0.3
                    while time.monotonic() < tail_end:
                        more = self.transport.read(0.05)
                        if more:
                            self._raw("rx", time.monotonic(), more)
                            buf.extend(more)
                    break
            elif now - last >= quiet_s and not (
                # BLE: the firmware greets with "Petoi <MODEL>" ~1.5 s after connecting and then
                # forces a rest (src/bleUart.h:160-178); do not race it with our first commands
                self.transport.name == "ble"
                and "Petoi" not in buf.decode("latin-1")
                and now - t0 < 4.0
            ):
                break
        self.boot_text = buf.decode("latin-1")
        # anything after the boot text is fed through the normal parser
        self._splitter = P.LineSplitter()

    def _raw(self, direction: str, t: float, data: bytes) -> None:
        if self.on_raw:
            try:
                self.on_raw(direction, t, data)
            except Exception:  # noqa: BLE001, S110 - a logging hook must not kill the link
                pass

    # ---- reader -----------------------------------------------------------------------
    def _read_loop(self) -> None:
        while self._running:
            try:
                data = self.transport.read(0.05)
            except TransportError as e:
                log.error("link lost: %s", e)
                self.connected = False
                with self._cv:
                    self._cv.notify_all()
                return
            except Exception:
                log.exception("reader error")
                time.sleep(0.05)
                continue
            if not data:
                continue
            now = time.monotonic()
            self._raw("rx", now, data)
            for cb in list(self._rx_listeners):
                try:
                    cb(data, now)
                except Exception:  # a listener must not kill the link
                    log.exception("rx listener failed")
            self.handle_bytes(data, now)

    def handle_bytes(self, data: bytes, now: float | None = None) -> None:
        """Parse a chunk of firmware output (public so tests can feed captured text)."""
        now = time.monotonic() if now is None else now
        lines = self._splitter.feed(data)
        for r in self._splitter.take_pending_imu():
            self._add_sample(r, now, "")
        for line in lines:
            self.stats.lines += 1
            readings, rest = P.parse_imu(line)
            r = P.preferred_reading(readings)
            if r is not None:
                # One chip per line is preferred when both print (MCU over ICM); several
                # readings from the same chip are consecutive prints glued together (BLE).
                for q in readings:
                    if q.source == r.source:
                        self._add_sample(q, now, line)
            # a print glued to the end of another one (BLE) becomes its own line, so echoes
            # such as the 'g' after an IMU line or the '?' after the identify reply are seen
            for piece in P.split_glued(rest.strip()):
                self._handle_text_line(piece.strip(), now)

    def _handle_text_line(self, rest: str, now: float) -> None:
        if "Low power" in rest:
            # e.g. "Low power: 2.18V. The robot won't move." (battery off or flat). The
            # firmware repeats it every 10 s and blocks for about 1.3 s each time.
            if self.low_power is None:
                log.error(
                    "ROBOT REPORTS: %s  Servos are unpowered: long-press the "
                    "battery button to switch it on, or charge it.",
                    rest,
                )
            self.low_power = rest
        if rest:
            self.stats.other_lines += 1
            with self._cv:
                self.recent_lines.append((now, rest))
                self._line_seq += 1
                self._cv.notify_all()

    def _add_sample(self, r: P.ImuReading, now: float, raw: str) -> None:
        yaw = self.yaw_sign * r.yaw_deg
        if self.yaw_drift_dps:
            # remove the constant drift measured by measure_yaw_drift()
            yaw -= self.yaw_drift_dps * (now - self._drift_t0)
        s = ImuSample(now, yaw, r.pitch_deg, r.roll_deg, r.source, raw)
        if self.stats.last_sample_t:
            dt = now - self.stats.last_sample_t
            if dt > 1e-4:
                inst = 1.0 / dt
                a = 0.2
                self.stats.imu_rate_hz = (
                    inst
                    if self.stats.imu_rate_hz == 0
                    else (1 - a) * self.stats.imu_rate_hz + a * inst
                )
        self.stats.last_sample_t = now
        self.stats.imu_samples += 1
        with self._cv:
            self._samples.append(s)
            self._new_samples.append(s)
            self._cv.notify_all()
        for cb in list(self._sample_listeners):
            cb(s)

    # ---- commands ---------------------------------------------------------------------
    def send(
        self, data: bytes, echo: str | None = None, timeout: float = 1.0, quiet: bool = False
    ) -> list[str] | None:
        """Write one command; if ``echo`` is given wait for that token line.

        Returns the non-IMU lines received while waiting (including the echo), or None on
        timeout. A timeout is counted and logged, never raised: an unexpected firmware is
        reported by ``probe`` rather than crashing the demo mid-motion."""
        with self._write_lock:
            # Never write while the previous command may still be unread by the firmware: two
            # commands in one burst merge and one of them is lost (moduleManager.h:405-445 for
            # USB; bleUart.h:46-90 for BLE, where the merged buffer breaks the FIRST command).
            # An echo proves the firmware consumed the command; without one, wait a guard time.
            now = time.monotonic()
            wait = max(self.min_gap - (now - self._last_write), self._guard_until - now)
            if wait > 0:
                time.sleep(wait)
            with self._cv:
                seq0 = self._line_seq
            now = time.monotonic()
            self._raw("tx", now, data)
            self.transport.write(data)
            self.stats.commands += 1
            self._last_write = time.monotonic()
            if echo is None:
                self._guard_until = self._last_write + self.unacked_guard_s
                return []
            deadline = now + timeout
            with self._cv:
                while True:
                    got = (
                        list(self.recent_lines)[-(self._line_seq - seq0) :]
                        if self._line_seq > seq0
                        else []
                    )
                    texts = [t for _, t in got]
                    if any(P.is_echo(t, echo) for t in texts):
                        self._last_write = time.monotonic()
                        return texts
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or not self.connected:
                        self.stats.echo_timeouts += 1
                        self._guard_until = time.monotonic() + self.unacked_guard_s
                        (log.debug if quiet else log.warning)(
                            "no echo %r for %r within %.1fs", echo, data[:24], timeout
                        )
                        return None
                    self._cv.wait(remaining)

    def read_analog(self, pins, timeout: float = 0.5) -> dict[int, int] | None:
        """Raw analog readings {pin: value} with one 'R' command (0..4095 on the ESP32).

        Read-only on the sensor side, but see protocol.encode_analog_read: never call this
        while a gait runs, it stops the gait. Returns None if the echo or a value is missing."""
        pins = [int(p) for p in pins]
        lines = self.send(P.encode_analog_read(pins), echo=P.T_READ, timeout=timeout, quiet=True)
        if lines is None:
            return None
        vals = P.parse_analog_reply(lines, len(pins))
        if vals is None:
            self.stats.analog_errors += 1
            return None
        return dict(zip(pins, vals, strict=True))

    def poll_analog(self, pins, rate_hz: float = 40.0) -> None:
        """Read ``pins`` up to ``rate_hz`` times a second from a background thread; samples
        (AnalogSample) go to a queue read with drain_analog(). The link allows about 28 reads
        a second of two pins over USB when nothing else is polled. pause_analog() stops the
        reads without stopping the thread (needed while a gait runs: an 'R' stops it)."""
        if self._analog_thread is not None:
            return
        pins = tuple(int(p) for p in pins)
        self._analog_stop.clear()
        period = 1.0 / max(1.0, rate_hz)

        def run() -> None:
            nxt = time.monotonic()
            while not self._analog_stop.is_set() and self.connected:
                with self._analog_gate:  # pause_analog() waits for a read in flight
                    if not self._analog_paused.is_set():
                        vals = self.read_analog(pins)
                        if vals is not None:
                            s = AnalogSample(time.monotonic(), vals)
                            with self._cv:
                                self._analog.append(s)
                            self.stats.analog_samples += 1
                nxt = max(nxt + period, time.monotonic())
                self._analog_stop.wait(max(0.0, nxt - time.monotonic()))

        self._analog_thread = threading.Thread(target=run, name="bittle-analog", daemon=True)
        self._analog_thread.start()

    def pause_analog(self, paused: bool = True) -> None:
        """Pause (or resume) analog polling. Pausing returns only after any read already in
        flight has finished, so a command sent right after cannot be followed by an 'R'."""
        if paused:
            self._analog_paused.set()
            with self._analog_gate:
                pass
        else:
            self._analog_paused.clear()

    def stop_poll_analog(self) -> None:
        self._analog_stop.set()
        t, self._analog_thread = self._analog_thread, None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=1.0)

    def drain_analog(self) -> list[AnalogSample]:
        with self._cv:
            out = list(self._analog)
            self._analog.clear()
        return out

    def read_voltage(self, timeout: float = 1.5) -> float | None:
        """Battery voltage in volts, or None when the firmware does not say (the 'P' reply is
        UNVERIFIED on hardware; a "Low power: 2.18V" warning seen earlier also counts)."""
        lines = self.send(P.encode_ascii(P.T_POWER), echo=P.T_POWER, timeout=timeout, quiet=True)
        v = P.parse_voltage(lines or [])
        if v is None:
            with self._cv:
                v = P.parse_voltage([t for _, t in list(self.recent_lines)[-20:]])
        return v

    def read_joints(self, timeout: float = 1.5) -> dict[int, int] | None:
        """All joint angles via 'j' ({index: degrees}), USB or BLE framing."""
        lines = self.send(P.encode_ascii(P.T_JOINTS), echo=P.T_JOINTS, timeout=timeout)
        return None if lines is None else P.parse_joint_table(lines)

    def skill(self, name: str, wait: bool = True, timeout: float = 4.0) -> bool:
        """Run a skill. With wait=False this returns at once, but the command is still sent by
        the command thread, which waits for the 'k' echo while holding the write lock: no IMU
        poll or head command can be written in between and merge with it. (A stand at the end
        of a turn was lost exactly that way over BLE.) Skills run in the order requested."""
        job = self.submit(P.encode_skill(name), echo="k", timeout=timeout)
        if not wait:
            return True
        job.done.wait(timeout + 5.0)
        return job.result is not None

    def submit(self, data: bytes, echo: str | None, timeout: float = 4.0) -> _Job:
        """Queue a command for the command thread; returns a handle (``done``, ``result``)."""
        job = _Job(data, echo, timeout)
        if self._cmd_thread is None or not self.connected:
            job.result = self.send(data, echo=echo, timeout=timeout)
            job.done.set()
            return job
        self._cmd_q.put(job)
        return job

    def flush_commands(self, timeout: float = 8.0) -> bool:
        """Wait until every queued command has been sent (and echoed or timed out)."""
        marker = _Job(b"", None, 0.0)
        if self._cmd_thread is None:
            return True
        self._cmd_q.put(marker)
        return marker.done.wait(timeout)

    def _cmd_loop(self) -> None:
        while True:
            job = self._cmd_q.get()
            if job is None:
                return
            try:
                if job.data and self.connected:
                    job.result = self.send(job.data, echo=job.echo, timeout=job.timeout)
            except Exception as e:  # noqa: BLE001 - one failed command must not stop the queue
                log.error("command %r failed: %s", job.data[:24], e)
            finally:
                job.done.set()

    def stand(self, balance: bool = False) -> None:
        """Stand up, then switch the balance reflex off (default) so the legs stay still while
        someone turns the dog by hand. 'gb' keeps the IMU updating; 'gu' would stop it."""
        self.skill(P.SKILL_STAND, timeout=5.0)
        # must come AFTER the skill: any token following a rest re-enables balance
        # (src/reaction.h:370-376)
        cmd = P.GYRO_BALANCE_ON if balance else P.GYRO_BALANCE_OFF
        self.send((cmd + "\n").encode(), echo="g", timeout=1.0)

    def rest(self) -> None:
        self.send(P.encode_ascii(P.T_REST), echo="d", timeout=4.0)

    def stream_imu(self, on: bool = True) -> None:
        """'gP' prints an IMU line every time the IMU updates, but print6Axis() is rate-limited
        to one line per 200 ms, so expect ~5 Hz. 'gp' prints once and stops the stream."""
        cmd = P.GYRO_PRINT_CONTINUOUS if on else P.GYRO_PRINT_ONCE
        self.send((cmd + "\n").encode(), echo="g", timeout=1.0)
        self.streaming = on

    def poll_imu(self, rate_hz: float = 20.0) -> None:
        """Ask for one IMU line ('gp') ``rate_hz`` times a second from a background thread.

        Preferred over ``stream_imu`` on a real robot: firmware B10_250330 has no print rate
        limit, so 'gP' saturates the 115200 baud link and the host receives the lines in clumps
        about every 1.4 s with useless timestamps. A single 'gp' is answered in about 13 ms."""
        if self._poll_thread is not None:
            return
        self._poll_stop.clear()
        period = 1.0 / max(1.0, rate_hz)

        def run() -> None:
            nxt = time.monotonic()
            while not self._poll_stop.is_set() and self.connected:
                self.send((P.GYRO_PRINT_ONCE + "\n").encode(), echo="g", timeout=0.3)
                nxt = max(nxt + period, time.monotonic())
                self._poll_stop.wait(max(0.0, nxt - time.monotonic()))

        self._poll_thread = threading.Thread(target=run, name="bittle-imu-poll", daemon=True)
        self._poll_thread.start()

    def stop_poll_imu(self) -> None:
        self._poll_stop.set()
        t, self._poll_thread = self._poll_thread, None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=1.0)

    def measure_yaw_drift(self, seconds: float = 4.0, max_dps: float = 8.0) -> float:
        """Measure how fast the reported yaw moves while the robot is NOT moving, and subtract
        that rate from every later sample. Needs IMU readings flowing (poll_imu / stream_imu).

        A real Bittle X+Arm (firmware B10_250330) drifted about 2.5 deg/s while standing still.
        Returns the measured rate in deg/s (after yaw_sign). A rate above ``max_dps`` is taken
        to mean the robot was being moved, and nothing is subtracted."""
        self.yaw_drift_dps = 0.0
        time.sleep(0.3)
        self.drain_samples()
        time.sleep(seconds)
        pts = self.drain_samples()
        if len(pts) < 8:
            log.warning("yaw drift: only %d samples, no compensation", len(pts))
            return 0.0
        ys = [pts[0].yaw_deg]
        for a, b in itertools.pairwise(pts):
            ys.append(ys[-1] + (b.yaw_deg - a.yaw_deg + 180.0) % 360.0 - 180.0)
        ts = [q.t for q in pts]
        tm, ym = sum(ts) / len(ts), sum(ys) / len(ys)
        den = sum((t - tm) ** 2 for t in ts)
        slope = sum((t - tm) * (y - ym) for t, y in zip(ts, ys, strict=True)) / den if den else 0.0
        if abs(slope) > max_dps:
            log.warning("yaw moved %.1f deg/s during drift measurement: robot not still?", slope)
            return slope
        self._drift_t0 = ts[-1]
        self.yaw_drift_dps = slope
        return slope

    def read_yaw(self, timeout: float = 1.5) -> ImuSample | None:
        """Return the next IMU sample. Uses 'gp' (OpenCatEsp32) and, if that yields nothing,
        the legacy 'v' token of older firmware."""
        with self._cv:
            n0 = self.stats.imu_samples
        if not self.streaming:
            token = P.T_LEGACY_PRINT_GYRO if self.legacy_imu else P.GYRO_PRINT_ONCE
            self.send((token + "\n").encode(), echo=token[0], timeout=min(timeout, 1.0))
        s = self._wait_sample(n0, timeout)
        if s is None and not self.streaming and not self.legacy_imu:
            self.send(b"v\n", echo="v", timeout=0.5)
            s = self._wait_sample(n0, 0.5)
            if s is not None:
                self.legacy_imu = True
        return s

    def _wait_sample(self, n0: int, timeout: float) -> ImuSample | None:
        deadline = time.monotonic() + timeout
        with self._cv:
            while self.stats.imu_samples <= n0:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cv.wait(remaining)
            return self._samples[-1]

    def drain_samples(self) -> list[ImuSample]:
        with self._cv:
            out = list(self._new_samples)
            self._new_samples.clear()
        return out

    @property
    def latest_sample(self) -> ImuSample | None:
        with self._cv:
            return self._samples[-1] if self._samples else None

    def add_sample_listener(self, cb: Callable[[ImuSample], None]) -> None:
        self._sample_listeners.append(cb)

    def add_rx_listener(self, cb: Callable[[bytes, float], None]) -> None:
        """``cb(data, t)`` gets every chunk of robot output as received, before it is split into
        lines (for messages that must be read across chunk and line boundaries)."""
        self._rx_listeners.append(cb)

    # ---- joints -----------------------------------------------------------------------
    def clamp(self, index: int, deg: float) -> int:
        return round(P.clamp_joint(index, deg, self.limits))

    def set_joint(self, index: int, deg: float) -> int:
        """Request a joint angle; returns the clamped value that will be sent. Non-blocking:
        the streamer thread sends only the latest request, at most joint_rate_hz per second."""
        a = self.clamp(index, deg)
        with self._joint_cv:
            self._joint_targets[index] = a
            self._joint_cv.notify()
        return a

    def _joint_loop(self) -> None:
        period = 1.0 / max(1.0, self.joint_rate_hz)
        while self._running:
            with self._joint_cv:
                self._joint_cv.wait(period)
                todo = {
                    i: a for i, a in self._joint_targets.items() if self._joint_sent.get(i) != a
                }
            if not todo or not self.connected:
                continue
            t0 = time.monotonic()
            for i, a in todo.items():
                cmd = P.encode_joint(i, a, self.joint_token)
                # quiet: a head command sent while a posture transition blocks the firmware
                # (e.g. 'kup' at the end of a turn) is acknowledged late; that is expected and
                # still counted in stats.echo_timeouts
                self.send(cmd, echo=self.joint_token, timeout=0.25, quiet=True)
                self._joint_sent[i] = a
            spare = period - (time.monotonic() - t0)
            if spare > 0:
                time.sleep(spare)

    def forget_joint_targets(self) -> None:
        with self._joint_cv:
            self._joint_targets.clear()
            self._joint_sent.clear()

    # ---- shutdown -----------------------------------------------------------------------
    def close(self, rest: bool = True) -> None:
        """Stop streaming and leave the robot resting, whatever state it was in. ``rest=False``
        only for programs that never move the robot themselves (it then stays as it is)."""
        try:
            self.stop_poll_imu()
            self.stop_poll_analog()
            self.forget_joint_targets()
            self.flush_commands()  # a queued stand goes out before the rest
            if self.connected and self.transport.is_open:
                try:
                    if self.streaming:
                        self.stream_imu(False)
                finally:
                    if rest:
                        self.rest()
        except Exception as e:  # noqa: BLE001 - cleanup must not mask the original error
            log.error("error while resting the robot: %s", e)
        finally:
            self._running = False
            self.connected = False
            self._cmd_q.put(None)
            for t in (self._streamer, self._reader, self._cmd_thread):
                if t is not None and t is not threading.current_thread():
                    t.join(timeout=1.0)
            try:
                self.transport.close()
            except Exception:  # noqa: BLE001, S110 - best-effort close
                pass

    def __enter__(self) -> Self:
        self.connect()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
