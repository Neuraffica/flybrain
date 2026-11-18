"""The real robot: its head is the body the circuit turns.

The camera sits in the dog's mouth and turns with the head. Every tick the controller gets the
camera's bearing, corrected for how far the head turned since the camera measured it (or None,
see ``camera.camera_input``); its turn command times ``gain`` is clipped to the head's rate
limit, integrated into a pan angle, clamped to the pan limit and sent with ``Bittle.set_joint``.
The legs do not walk.

Camera through the robot (``--camera robot``):
  Start-up: [USB only: restart the board with an EN pulse, wait for Ready!; the firmware can
  start its camera only once per power-on], Xa~ (voice off), XCr~ (camera on, its own reaction
  off; the robot sits by itself; wait for the X echo, but go on without it), XCP~ (print the
  target whenever it moves; a missing echo is not a failure).
  Shut-down, always: head to centre, Xc~ (camera off; also saves the module list), XA~ (voice
  back on, unless asked not to), gU (IMU updates back on), rest.
Camera on its own USB cable (``--camera usb``, see usbcam.py):
  Start-up: Xa~, ksit (sit). No XC command at all. Then AT+INVOKE on the camera's port.
  Shut-down: AT+BREAK on the camera's port; head to centre, XA~, rest.
Every shut-down step is sent even when an echo is missing (over Bluetooth replies go missing
while the firmware's camera runs). A lost link is reconnected once to shut down.

Measurements behind these steps: packages/flybrain-edge/NOTES.md, "Petoi AI Vision camera".
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass

from flybrain_edge.bittle import Bittle
from flybrain_edge.bittle import protocol as P

from .camera import (
    CENTRE_DEG,
    DEFAULT_FOV_DEG,
    HOLD_AFTER_S,
    STALE_S,
    PositionStore,
    parse_line,
)
from .controllers import CONTROLLERS, is_circuit

TICK_HZ = 15.0  # controller steps per second on the robot
CAMERA_START_TIMEOUT_S = 14.0  # XCr~ until its X echo (measured 1.5 s over USB, 2.5 s over BLE)
X_TIMEOUT_S = 4.0  # the other X commands
FIRST_REPORT_S = 20.0  # no camera position at all by then: ask the user to check
EN_PULSE_S = 0.15  # USB restart: RTS high this long, with DTR low
READY_WAIT_S = 20.0  # after the restart, wait this long at most for "Ready!"
CAMERA_SOURCES = ("robot", "usb")
JOINT_SETTLE_S = 0.4  # after forgetting the head target: lets a head command in flight finish
PAN_FIRMWARE_LIMIT_DEG = float(min(abs(v) for v in P.FIRMWARE_ANGLE_LIMITS[P.JOINT_HEAD_PAN]))

CAMERA_CHECK = (
    "check that the camera is plugged into the Grove socket labelled SCL SDA +5V GND (not the "
    "one by the TX2/RX2 label) and that the robot is switched on"
)
RECOVERY = """\
The camera could not be switched off. Its setting is saved on the robot, so it would start
again at the next power-on, with its own head reaction on. To recover:
  1. switch the robot off,
  2. unplug the camera,
  3. switch the robot on,
  4. run: fly-pursuit --camera-off --port <device>   (or --ble)"""
NO_REPORT = (
    "no camera position in {s:g} s: show a face about 50 cm in front of the camera. If the head "
    "still does not react, switch the robot off and on and start again (its camera starts only "
    "once per power-on)"
)
NO_REPORT_USB = (
    "no result from the camera on its USB cable in {s:g} s: check its cable, then start again"
)
ONCE_PER_BOOT = (
    "the robot's firmware can start its camera only once per power-on. Over Bluetooth this "
    "program cannot restart the robot: switch it off and on before this run if the camera was "
    "used since it was switched on (a second start reports only the last old position)"
)


class LinkLost(RuntimeError):
    """The robot link stopped (cable out, Bluetooth dropped, robot off)."""


@dataclass
class HeadConfig:
    camera_fov_deg: float = DEFAULT_FOV_DEG  # ASSUMED: field of view across the image
    head_rate_deg_s: float = 60.0  # this program's choice: how fast the head may turn
    pan_limit_deg: float = 50.0  # never beyond the firmware's limit (PAN_FIRMWARE_LIMIT_DEG)
    command_hz: float = 8.0  # head commands per second; they compete with the camera task
    pan_sign: int = P.DEFAULT_PAN_SIGN  # +1 if a positive pan angle looks left
    voice_on_at_exit: bool = True
    tick_hz: float = TICK_HZ
    stale_s: float = STALE_S
    centre_deg: float = CENTRE_DEG
    hold_after_s: float = HOLD_AFTER_S
    camera_timeout_s: float = CAMERA_START_TIMEOUT_S
    boot_timeout_s: float = 12.0
    gain: float = 1.0  # scales the controller's turn command on the robot (for tuning)
    camera_source: str = "robot"  # robot | usb
    usb_restart: bool = True  # robot camera over USB: restart the board first
    first_report_s: float = FIRST_REPORT_S

    def check(self) -> str | None:
        """An error message for settings this program refuses, or None."""
        if not 10 <= self.camera_fov_deg <= 170:
            return "--camera-fov must be between 10 and 170 degrees"
        if not 1 <= self.head_rate_deg_s <= 360:
            return "--head-rate must be between 1 and 360 deg/s"
        if not 1 <= self.pan_limit_deg <= PAN_FIRMWARE_LIMIT_DEG:
            return f"--pan-limit must be between 1 and {PAN_FIRMWARE_LIMIT_DEG:g} degrees"
        if not 0.5 <= self.command_hz <= 30:
            return "--command-rate must be between 0.5 and 30 per second"
        if self.pan_sign not in (1, -1):
            return "--pan-sign must be 1 or -1"
        if not 0.1 <= self.gain <= 2.0:
            return "--gain must be between 0.1 and 2"
        if self.camera_source not in CAMERA_SOURCES:
            return "--camera must be robot or usb"
        return None


# ---- the head loop -----------------------------------------------------------------------
LINK_LABELS = {"usb": "USB", "ble": "Bluetooth", "mock": "test link"}


class HeadRig:
    """The robot as the dashboard's "rig": the same settings, commands and frame fields as
    ``sim.PursuitSim``, with the head as the turning body. Stepped by the caller."""

    def __init__(
        self,
        reader: PositionStore,
        cfg: HeadConfig | None = None,
        controller: str = "connectome",
        arousal: float = 1.0,
        bittle: Bittle | None = None,
        clock: Callable[[], float] = time.monotonic,
        on_attention: Callable[[str], None] | None = None,
    ):
        self.reader = reader
        self.on_attention = on_attention
        self.attention: str | None = None
        self.camera = None  # usbcam.UsbCamera when the camera is on its own cable
        self.cfg = cfg or HeadConfig()
        self.bittle = bittle
        self.clock = clock
        self._models: dict[str, object] = {}
        self.t = 0.0
        self.pan = 0.0  # rad, + = left: where the head is told to point
        self.turn = 0.0  # rad/s, controller command
        self.cmd = 0.0  # rad/s, after the rate limit (0 while holding)
        self.rate = 0.0  # rad/s, how fast the pan target actually moved (after the pan limit)
        self.input = reader.input(clock())
        self.holding = False
        self.sent_deg: int | None = None
        self.arousal = 1.0
        self.set_arousal(arousal)
        err = self.set_controller(controller)
        if err:
            raise ValueError(err)
        self.reset_stats()

    # ---- settings (each returns an error string or None, as PursuitSim) ------------------
    def set_controller(self, key: str) -> str | None:
        if key not in CONTROLLERS:
            return f"unknown controller {key!r}"
        if key not in self._models:
            self._models[key] = CONTROLLERS[key].make()
        self.controller = key
        self.model = self._models[key]
        self.model.reset()
        self.turn = self.cmd = 0.0
        return None

    def set_arousal(self, value) -> str | None:
        if isinstance(value, bool) or not isinstance(value, int | float):
            return "arousal must be between 0 and 1"
        if not (math.isfinite(value) and 0.0 <= value <= 1.0):
            return "arousal must be between 0 and 1"
        self.arousal = float(value)
        return None

    def _not_here(self, *_) -> str:
        return "on the robot the target is the face the camera sees; it cannot be set here"

    set_target = nudge_target = set_motion = _not_here

    def set_fov(self, *_) -> str:
        return "on the robot the field of view is set at start (--camera-fov)"

    def reset_stats(self) -> None:
        self.stats_t = 0.0  # seconds since the reset
        self._with = 0.0  # seconds with a bearing
        self._n = 0  # new camera positions with a face
        self._sum_err = 0.0  # sum of |measured bearing| over those positions, degrees
        self._within = 0  # how many of them were within 10 degrees
        self._lines_seen = self.reader.lines

    # ---- one tick ---------------------------------------------------------------------
    def advance(self, seconds: float) -> None:
        """One control step covering ``seconds`` of real time."""
        if self.bittle is not None and not self.bittle.connected:
            raise LinkLost("the link to the robot was lost")
        dt = max(1e-3, float(seconds))
        cfg = self.cfg
        inp = self.input = self.reader.input(self.clock(), self.pan)  # efference copy
        self.turn = cfg.gain * self.model.step(inp.bearing, dt, arousal=self.arousal).turn
        self.holding = inp.state == "lost" and inp.lost_s >= cfg.hold_after_s
        vmax = math.radians(cfg.head_rate_deg_s)
        self.cmd = 0.0 if self.holding else min(vmax, max(-vmax, self.turn))
        lim = math.radians(cfg.pan_limit_deg)
        pan = min(lim, max(-lim, self.pan + self.cmd * dt))
        self.rate = (pan - self.pan) / dt
        self.pan = pan
        self.reader.pan = pan  # positions that arrive from now on were seen from here
        if self.bittle is not None:
            self.sent_deg = self.bittle.set_joint(
                P.JOINT_HEAD_PAN, cfg.pan_sign * math.degrees(self.pan)
            )
        self.t += dt
        self.stats_t += dt
        if self.attention is None and self.t >= cfg.first_report_s and self.reader.lines == 0:
            text = NO_REPORT_USB if self.reader.source == "usb" else NO_REPORT
            self.attention = text.format(s=cfg.first_report_s)
            if self.on_attention is not None:
                self.on_attention(self.attention)
        if inp.bearing is not None:
            self._with += dt
        # The error is counted on what the camera MEASURED, once per new position. The bearing
        # fed to the controller is corrected for the head's own motion and so tends to zero by
        # construction; averaging that would flatter the result.
        last = self.reader.last
        if self.reader.lines != self._lines_seen:
            self._lines_seen = self.reader.lines
            if last is not None and last.bearing is not None:
                e = abs(math.degrees(last.bearing))
                self._n += 1
                self._sum_err += e
                self._within += e <= 10.0

    # ---- what the dashboard shows -------------------------------------------------------
    def stats(self) -> dict:
        w, t, n = self._with, self.stats_t, self._n
        return {
            "seconds": round(t, 1),
            "mae_deg": round(self._sum_err / n, 2) if n else None,  # measured, per position
            "within10": round(self._within / n, 3) if n else None,
            "in_view": round(w / t, 3) if t > 0 else None,  # share of time with a bearing
        }

    def link(self) -> dict | None:
        b = self.bittle
        if b is None:
            return None
        name = getattr(b.transport, "name", "")
        return {
            "name": name,
            "label": LINK_LABELS.get(name, name),
            "description": str(getattr(b.transport, "description", "")),
            "firmware": " ".join(x for x in (b.model, b.version) if x) or None,
            "connected": b.connected,
            "commands": b.stats.commands,
            "echo_timeouts": b.stats.echo_timeouts,
            "command_hz": self.cfg.command_hz,
            "head_sent_deg": self.sent_deg,
        }

    def snapshot(self) -> dict:
        deg = math.degrees
        cfg, inp = self.cfg, self.input
        head = deg(self.pan)
        b = inp.bearing
        cam = self.camera
        return {
            "mode": "robot",
            "t": round(self.t, 3),
            "heading_deg": 0.0,  # the body does not turn
            "head_deg": round(head, 2),
            "target": {
                "exists": b is not None,
                "bearing_deg": None if b is None else round(head + deg(b), 2),
                "motion": "camera",
            },
            "error_deg": None if b is None else round(deg(b), 2),
            "camera": inp.state,
            "camera_age_s": None if inp.age_s is None else round(inp.age_s, 2),
            "camera_lines": self.reader.lines,
            "hold": self.holding,
            "seen": inp.state == "seen",
            "measured_deg": None if b is None else round(deg(b), 2),
            "raw_bearing_deg": None if inp.raw is None else round(deg(inp.raw), 2),
            "camera_score": inp.score,
            "camera_source": self.reader.source,
            "camera_error": getattr(cam, "error", None),
            "attention": self.attention,
            "turn_deg_s": round(deg(self.turn), 2),
            "cmd_deg_s": round(deg(self.cmd), 2),
            "rate_deg_s": round(deg(self.rate), 2),
            "controller": self.controller,
            "circuit": is_circuit(self.model),
            "arousal": self.arousal,
            "fov_deg": cfg.camera_fov_deg / 2,  # half, as the simulator's
            "pan_limit_deg": cfg.pan_limit_deg,
            "rig": {
                "rate_hz": cfg.tick_hz,
                "delay_s": None,
                "noise_deg": None,
                "dropout": None,
                "max_turn_deg_s": cfg.head_rate_deg_s,
                "lag_s": None,
                "camera_fov_deg": cfg.camera_fov_deg,
                "stale_s": cfg.stale_s,
                "centre_deg": cfg.centre_deg,
                "hold_after_s": cfg.hold_after_s,
                "gain": cfg.gain,
            },
            "link": self.link(),
            "stats": self.stats(),
        }


# ---- start-up and shut-down ----------------------------------------------------------------
def x_cmd(code: str) -> bytes:
    return P.encode_extension(code)


class RestartingTransport:
    """Wraps a USB serial transport: after opening the port, restart the board with an EN
    pulse (RTS high for EN_PULSE_S with DTR low; this does not enter download mode) and wait
    for its "Ready!". The firmware can start its camera only once per power-on, so a fresh
    boot is needed for every run. Everything else is passed through."""

    def __init__(self, inner, say: Callable[[str], None] = print):
        self.inner = inner
        self.say = say
        self.restarted = False

    def __getattr__(self, name):  # name, description, is_open, read, write, close, ...
        return getattr(self.inner, name)

    def open(self) -> None:
        self.inner.open()
        ser = getattr(self.inner, "_ser", None)  # pyserial port of SerialTransport
        if ser is None:
            self.say("fly-pursuit: cannot restart the board over this link; continuing")
            return
        self.say("fly-pursuit: restarting the robot's board (its camera starts once per boot)")
        ser.dtr = False
        ser.rts = True
        time.sleep(EN_PULSE_S)
        ser.rts = False
        buf = bytearray()
        end = time.monotonic() + READY_WAIT_S
        while time.monotonic() < end and b"Ready!" not in buf:
            buf.extend(self.inner.read(0.1))
        if b"Ready!" in buf:
            self.restarted = True
            time.sleep(0.3)  # the trailing beep and prints
        else:
            self.say(f"fly-pursuit: no Ready! within {READY_WAIT_S:g} s after the restart")


def connect(transport, cfg: HeadConfig, reader=None) -> Bittle:
    """Open the link (raises TransportError or BittleError) and attach the camera reader
    (a robot-source reader parses the robot's output)."""
    b = Bittle(
        transport,
        joint_rate_hz=cfg.command_hz,
        joint_limits={P.JOINT_HEAD_PAN: (-cfg.pan_limit_deg, cfg.pan_limit_deg)},
    )
    if reader is not None and hasattr(reader, "feed_line"):
        b.add_rx_listener(reader.feed)
    try:
        b.connect(boot_timeout=cfg.boot_timeout_s)
    except BaseException:
        try:
            transport.close()
        except Exception:  # noqa: BLE001, S110 - best-effort close after a failed connect
            pass
        raise
    return b


def lines_since(b: Bittle, t0: float) -> list[str]:
    """Robot output lines received since host time ``t0`` (camera positions left out)."""
    return [txt for t, txt in list(b.recent_lines) if t >= t0 and parse_line(txt) is None]


def start_camera(b: Bittle, cfg: HeadConfig) -> tuple[str | None, list[str]]:
    """Voice off, camera on with its own reaction off, print on change. Returns (an error in
    plain words or None, notes). Missing echoes are notes, not errors: over Bluetooth replies
    go missing while the camera runs although the command was carried out."""
    notes: list[str] = []
    b.send(x_cmd("a"), echo="X", timeout=X_TIMEOUT_S)  # voice off: sounds would move the dog
    t0 = time.monotonic()
    reply = b.send(x_cmd("Cr"), echo="X", timeout=cfg.camera_timeout_s)
    got = lines_since(b, t0)
    if "fail to start" in "\n".join(got).lower():
        return "the robot reports that the camera failed to start", notes
    modules = P.parse_module_list(got)
    if modules is not None and modules.get("C") is False:
        return "the robot did not switch the camera on", notes
    if reply is None:
        notes.append(
            f"no reply to XCr~ within {cfg.camera_timeout_s:g} s; continuing (replies can go "
            "missing while the camera starts)"
        )
    if b.send(x_cmd("CP"), echo="X", timeout=X_TIMEOUT_S) is None:
        notes.append("no reply to XCP~; continuing")
    return None, notes


def start_robot_for_usb_camera(b: Bittle) -> list[str]:
    """The camera is on its own cable: voice off and sit. No XC command."""
    notes = []
    if b.send(x_cmd("a"), echo="X", timeout=X_TIMEOUT_S) is None:
        notes.append("no reply to Xa~ (voice off); continuing")
    if not b.skill(P.SKILL_SIT, timeout=5.0):
        notes.append("no reply to ksit; continuing")
    return notes


def _alive(b: Bittle) -> None:
    if not b.connected:
        raise LinkLost("the link to the robot was lost")


def shut_down_steps(b: Bittle, cfg: HeadConfig, say: Callable[[str], None] = print) -> bool:
    """Head to centre, [camera off], voice on, [IMU on], rest; every step is sent even if an
    echo is missing. Returns True if the camera is off: its echo came, or any later reply did
    (then the robot was answering after Xc~). Raises LinkLost / TransportError if the link
    dies on the way."""
    robot_cam = cfg.camera_source == "robot"
    _alive(b)
    b.forget_joint_targets()
    time.sleep(JOINT_SETTLE_S)  # a head command already on its way goes out first
    b.send(P.encode_joint(P.JOINT_HEAD_PAN, 0, b.joint_token), echo=b.joint_token, timeout=1.0)
    _alive(b)
    off = True
    if robot_cam:
        t_off = time.monotonic()
        off = b.send(x_cmd("c"), echo="X", timeout=X_TIMEOUT_S) is not None
        _alive(b)
    if cfg.voice_on_at_exit:
        b.send(x_cmd("A"), echo="X", timeout=X_TIMEOUT_S)
    if robot_cam:
        b.send(b"gU\n", echo="g", timeout=1.5)  # IMU updates back on (the camera stopped them)
    _alive(b)
    b.close(rest=True)
    if robot_cam and not off:
        off = bool(lines_since(b, t_off))  # e.g. the 'g' or 'd' echo: the robot was answering
        if off:
            say("fly-pursuit: no reply to Xc~, but the robot answered later: taken as off")
        else:
            say("fly-pursuit: the robot did not answer after Xc~")
    return off


def shut_down(
    b: Bittle | None,
    cfg: HeadConfig,
    reconnect: Callable[[], Bittle] | None,
    say: Callable[[str], None] = print,
) -> bool:
    """Always leave the robot with the camera off, the head centred and resting. After a lost
    link, ``reconnect()`` is tried once. Returns True if the camera is off (or was never the
    robot's); if not, the recovery steps have been printed."""
    for attempt in (1, 2):
        if b is not None:
            try:
                if shut_down_steps(b, cfg, say):
                    return True
                break
            except Exception as e:  # noqa: BLE001 - any failure: release the link, reconnect
                say(f"fly-pursuit: link lost while shutting down ({e})")
                try:
                    b.close(rest=False)  # stop its threads and close the port
                except Exception:  # noqa: BLE001, S110 - best-effort
                    pass
        if attempt == 2 or reconnect is None:
            break
        say("fly-pursuit: reconnecting once to shut down and rest the robot")
        try:
            b = reconnect()
        except Exception as e:  # noqa: BLE001 - whatever the reason, say so and give the steps
            say(f"fly-pursuit: could not reconnect ({e})")
            break
    if cfg.camera_source == "robot":
        say(RECOVERY)
    else:
        say("fly-pursuit: could not rest the robot: switch it off")
    return False


def camera_off(
    make_transport: Callable[[], object], cfg: HeadConfig, say: Callable[[str], None] = print
) -> int:
    """``--camera-off``: switch the camera off and make the robot save that, then rest.

    Xc~ alone saves nothing if the camera flag was only set in flash (a failed start clears it
    in memory only), so voice off and on again forces the module list to be written."""
    try:
        b = connect(make_transport(), cfg)
    except Exception as e:  # noqa: BLE001 - no port, no Bluetooth, robot off
        say(f"fly-pursuit: could not connect: {e}")
        return 4
    ok = True
    try:
        b.forget_joint_targets()
        for data, echo, timeout in (
            (x_cmd("c"), "X", X_TIMEOUT_S),
            (x_cmd("a"), "X", X_TIMEOUT_S),
            (x_cmd("A"), "X", X_TIMEOUT_S),
            (b"gU\n", "g", 1.5),
        ):
            ok &= b.send(data, echo=echo, timeout=timeout) is not None
    finally:
        b.close(rest=True)
    if not ok:
        say("fly-pursuit: some commands were not confirmed by the robot; run --camera-off again")
        return 1
    say("fly-pursuit: camera off and saved, voice on, IMU on, robot resting")
    return 0
