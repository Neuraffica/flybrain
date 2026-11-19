"""The real robot: its head is the body the circuit turns.

The camera sits in the dog's mouth and turns with the head. Every tick the controller gets the
camera's bearing, corrected for how far the head turned since the camera measured it (or None,
see ``camera.camera_input``); its turn command times ``gain`` is clipped to the head's rate
limit, integrated into a pan angle, clamped to the pan limit and sent with ``Bittle.set_joint``.
The legs do not walk.

Four ways to run (``--camera`` and ``--onboard``; the firmware module's README is
firmware/bittle/fly_pursuit/README.md):

On-board (``--onboard``, needs our firmware module P): the ROBOT computes.
  Start-up: X?~ (is the module there?), Xa~ (voice off), XP~ (module on, passive), XPB~
  (on-board control on; the firmware makes the robot sit). The laptop sends no head command
  while it runs; it reads each result line's b= t= p= hold and shows them, and recomputes the
  circuit's activity from the robot's bearing for display (a shadow model).
  Shut-down: XPb~ (on-board control off), head to centre, Xp~ (module off), XA~, rest.
Camera through our module, laptop computes (``--camera module``):
  Start-up: X?~, Xa~, XP~ (passive: the module does not move the robot), ksit (sit). No XC,
  no gU, no board restart.
  Shut-down: head to centre, Xp~, XA~ (voice back on, unless asked not to), rest.
Camera through the stock module (``--camera robot``):
  Start-up: [USB only: restart the board with an EN pulse, wait for Ready!; the firmware can
  start its camera only once per power-on], Xa~ (voice off), XCr~ (camera on, its own reaction
  off; the robot sits by itself; wait for the X echo, but go on without it), XCP~ (print the
  target whenever it moves; a missing echo is not a failure).
  Shut-down, always: head to centre, Xc~ (camera off; also saves the module list), XA~ (voice
  back on, unless asked not to), gU (IMU updates back on), rest.
Camera on its own USB cable (``--camera usb``, see usbcam.py):
  Start-up: Xa~, ksit (sit). No XC command at all. Then AT+INVOKE on the camera's port.
  Shut-down: AT+BREAK on the camera's port; head to centre, XA~, rest.
``--camera auto`` (the default) sends X?~ after connecting and uses ``module`` if an "@FP"
status line comes back, ``robot`` otherwise (the CLI does that, see cli.py).
A robot whose module has the saved "follow after power-on" setting on (``--autostart on``) may
already be sitting and turning its head by itself when the demo connects. Both module start-ups
still work: a bare XP~ switches on-board control off on the builds that have the setting, so
the laptop-computes mode takes the head over, and on-board mode switches it on again with XPB~.
``--autostart on``: X?~, XPS~ (module on, setting saved on; wait for "@FP auto 1;"), Xp~ (the
camera off again: the setting acts at the next power-on), rest. ``--autostart off``: X?~, Xps~
(module off, setting saved off; wait for "@FP auto 0;"), rest. Older builds of the module
ignore S and s and never print "@FP auto": that is reported, not taken as done.
Every shut-down step is sent even when an echo is missing (over Bluetooth replies go missing
while the stock camera module runs). A lost link is reconnected once to shut down.

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
    FP_ATTENTION,
    HOLD_AFTER_S,
    STALE_S,
    FpMessage,
    ModuleReader,
    PositionStore,
    bearing_rad,
    parse_line,
)
from .controllers import CONTROLLERS, is_circuit

TICK_HZ = 15.0  # controller steps per second on the robot
CAMERA_START_TIMEOUT_S = 14.0  # XCr~ until its X echo (measured 1.5 s over USB, 2.5 s over BLE)
X_TIMEOUT_S = 4.0  # the other X commands
FIRST_REPORT_S = 20.0  # no camera position at all by then: ask the user to check
EN_PULSE_S = 0.15  # USB restart: RTS high this long, with DTR low
READY_WAIT_S = 20.0  # after the restart, wait this long at most for "Ready!"
CAMERA_SOURCES = ("auto", "module", "robot", "usb")
FP_STATUS_WAIT_S = 0.6  # after an echo, wait this long at most for the module's "@FP" status
AUTOSTART_WAIT_S = 2.0  # after XPS~ / Xps~ and its echo, wait this long at most for "@FP auto"
ONBOARD_START_TIMEOUT_S = 5.0  # XPB~ sits first (its echo came after 0.85 s over Bluetooth)
# The firmware module's own constants (firmware/bittle/fly_pursuit/src/flyPursuitCore.h), the
# same numbers as this program's defaults: used for the shadow model and the display.
ONBOARD_HEAD_RATE_DEG_S = 60.0
ONBOARD_PAN_LIMIT_DEG = 50.0
ONBOARD_HOLD_AFTER_S = 3.0
ONBOARD_DT_MIN_S, ONBOARD_DT_MAX_S = 0.001, 0.5
ONBOARD_RESET_GAP_S = 1.0  # no result for this long: the firmware resets its circuit
ONBOARD_FIRST_DT_S = 1.0 / 15.0
MODULE_DIR = "firmware/bittle/fly_pursuit"
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
NO_REPORT_MODULE = (
    "no result line from the robot's camera module in {s:g} s: check that the camera is "
    "plugged into the Grove socket labelled SCL SDA +5V GND, then switch the robot off and on"
)
NO_MODULE = (
    "the robot's firmware does not have the fly_pursuit camera module (no '@FP' status line "
    "after X?~). Flash the firmware in " + MODULE_DIR + " (see its README), or run without "
    "--onboard and with --camera robot or --camera usb"
)
NO_MODULE_AUTOSTART = (
    "the robot's firmware does not have the fly_pursuit camera module (no '@FP' status line "
    "after X?~), so it cannot follow a face after power-on. Flash the newer build from "
    + MODULE_DIR
    + " (see its README)"
)
OLDER_MODULE = (
    "this robot's module is older and has no autostart (no '@FP auto' line after {cmd}): flash "
    "the newer build from " + MODULE_DIR + " (see its README). Nothing was saved"
)
AUTOSTART_ON_DONE = (
    "saved on the robot: follow a face after power-on is ON. At every power-on from now, with "
    "no computer connected, the robot waits a few seconds, beeps three times, then sits and "
    "moves its head toward a face by itself. Before switching it on, put it on the floor, away "
    "from edges. Switch this off with: fly-pursuit --autostart off (or --camera-off)"
)
AUTOSTART_OFF_DONE = (
    "saved on the robot: follow a face after power-on is OFF. At the next power-on the robot "
    "starts as usual and does not move by itself"
)
MODULE_HARMLESS = (
    "the camera module may still be switched on in the robot's saved settings; that is "
    "harmless: after a power-on it only prints result lines and does not move the robot"
)
ONBOARD_LOCKED = (
    "on-board mode: the controller is the circuit in the robot's firmware (connectome, "
    "arousal 1, gain 1); it cannot be changed from here"
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
    camera_source: str = "robot"  # auto | module | robot | usb
    onboard: bool = False  # the robot's firmware module computes (needs the module)
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
            return "--camera must be auto, module, robot or usb"
        if self.onboard and self.camera_source not in ("auto", "module"):
            return "--onboard uses the robot's camera module: use it with --camera auto or module"
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
        # On-board mode: the robot computes; self.model is the shadow model (the firmware's
        # circuit, recomputed here from the bearing the robot reports, for display only).
        self.onboard = bool(self.cfg.onboard)
        self._locked = False
        self.shadow_turn = 0.0  # rad/s, shadow model's command before the head's rate limit
        self.shadow_cmd = 0.0  # rad/s, after the limit (0 while the robot holds)
        self.shadow_steps = 0
        self._shadow_t: float | None = None
        self._fw_t: float | None = None  # host time of the last on-board result
        if self.onboard:
            controller, arousal = "connectome", 1.0
        self.set_arousal(arousal)
        err = self.set_controller(controller)
        if err:
            raise ValueError(err)
        self._locked = self.onboard
        self.reset_stats()

    def use_reader(self, reader: PositionStore) -> None:
        """Switch the camera source (``--camera auto`` falling back to the stock module)."""
        reader.pan = self.pan
        self.reader = reader
        self.input = reader.input(self.clock(), self.pan)
        self.reset_stats()

    @property
    def module(self) -> ModuleReader | None:
        return self.reader if isinstance(self.reader, ModuleReader) else None

    # ---- settings (each returns an error string or None, as PursuitSim) ------------------
    def set_controller(self, key: str) -> str | None:
        if self._locked:
            return ONBOARD_LOCKED
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
        if self._locked:
            return ONBOARD_LOCKED
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
        self._diff_n = 0  # on-board results compared with the shadow model
        self._diff_sum = 0.0
        self._diff_max = 0.0

    # ---- one tick ---------------------------------------------------------------------
    def advance(self, seconds: float) -> None:
        """One control step covering ``seconds`` of real time."""
        if self.bittle is not None and not self.bittle.connected:
            raise LinkLost("the link to the robot was lost")
        dt = max(1e-3, float(seconds))
        if self.onboard:
            self._advance_onboard(dt)
        else:
            self._advance_laptop(dt)
        self.t += dt
        self.stats_t += dt
        self._update_attention()
        if self.input.bearing is not None:
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

    def _advance_onboard(self, dt: float) -> None:
        """The robot computes: take its head turn and pan target from the result lines, and
        step the shadow model once per on-board result with the bearing the robot used."""
        now = self.clock()
        for t, msg in self.module.take_control():
            self._shadow_step(t, msg)
        fw = self.module.last_control
        if fw is not None:
            pan = math.radians(fw.pan_deg)
            self.rate = (pan - self.pan) / dt
            self.pan = pan
            self.cmd = math.radians(fw.turn_deg_s)
            self.holding = fw.hold
        else:
            self.rate = 0.0
        self.reader.pan = self.pan
        self.input = self.reader.input(now, self.pan)
        self.turn = self.shadow_turn

    def _shadow_step(self, t: float, msg: FpMessage) -> None:
        """One step of the shadow model, as flyPursuitCore.h's fpControlStep does it (dt from
        the time between results, here as received; circuit reset after a 1 s gap)."""
        if self._shadow_t is None:
            dt = ONBOARD_FIRST_DT_S
        else:
            gap = t - self._shadow_t
            if gap > ONBOARD_RESET_GAP_S:
                self.model.reset()
            dt = min(ONBOARD_DT_MAX_S, max(ONBOARD_DT_MIN_S, gap))
        self._shadow_t = t
        self._fw_t = t
        bearing = None
        if msg.box is not None:  # b= always comes with a box when control is on (grammar)
            b = msg.bearing_deg
            bearing = bearing_rad(msg.box[0], self.reader.fov_deg) if b is None else math.radians(b)
        self.shadow_turn = self.model.step(bearing, dt, arousal=1.0).turn
        vmax = math.radians(ONBOARD_HEAD_RATE_DEG_S)
        self.shadow_cmd = 0.0 if msg.hold else min(vmax, max(-vmax, self.shadow_turn))
        self.shadow_steps += 1
        d = abs(math.degrees(self.shadow_cmd) - msg.turn_deg_s)
        self._diff_n += 1
        self._diff_sum += d
        self._diff_max = max(self._diff_max, d)

    def _advance_laptop(self, dt: float) -> None:
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

    def _update_attention(self) -> None:
        """A problem the user must act on: the module's own report (camera to power-cycle, or
        not plugged in), else no camera line at all in the first ``first_report_s``. Cleared
        when the reason goes away; ``on_attention(None)`` says so."""
        text = None
        mod = self.module
        if mod is not None and mod.problem in FP_ATTENTION:
            text = FP_ATTENTION[mod.problem]
        elif self.t >= self.cfg.first_report_s and self.reader.lines == 0:
            source = self.reader.source
            text = {"usb": NO_REPORT_USB, "module": NO_REPORT_MODULE}.get(source, NO_REPORT)
            text = text.format(s=self.cfg.first_report_s)
        if text != self.attention:
            self.attention = text
            if self.on_attention is not None:
                self.on_attention(text)

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
        if self.onboard:  # the bearing the robot's own circuit got (b=), while a face is seen
            fw = self.module.last_control if self.module is not None else None
            seen = fw is not None and fw.bearing_deg is not None and inp.state == "seen"
            b = math.radians(fw.bearing_deg) if seen else None
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
            "camera_module": None if self.module is None else self.module.summary(),
            "onboard": self.onboard_info(),
        }

    def onboard_info(self) -> dict | None:
        """On-board mode: what the robot printed, and the shadow model's numbers beside it."""
        if not self.onboard:
            return None
        deg = math.degrees
        fw = self.module.last_control if self.module is not None else None
        n = self._diff_n
        return {
            "results": self.shadow_steps,  # results with on-board control fields
            "age_s": None if self._fw_t is None else round(max(0.0, self.clock() - self._fw_t), 2),
            "turn_deg_s": None if fw is None else fw.turn_deg_s,  # t=: the robot's command
            "pan_deg": None if fw is None else fw.pan_deg,  # p=
            "bearing_deg": None if fw is None else fw.bearing_deg,  # b=: None without a face
            "hold": False if fw is None else fw.hold,
            "shadow_turn_deg_s": round(deg(self.shadow_turn), 2),
            "shadow_cmd_deg_s": round(deg(self.shadow_cmd), 2),
            "difference_deg_s": None
            if fw is None
            else round(deg(self.shadow_cmd) - fw.turn_deg_s, 2),
            "mean_abs_difference_deg_s": round(self._diff_sum / n, 2) if n else None,
            "max_abs_difference_deg_s": round(self._diff_max, 2) if n else None,
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
    """Open the link (raises TransportError or BittleError) and attach the camera reader(s)
    that read the robot's output (``reader``: one reader, a list of them, or None)."""
    b = Bittle(
        transport,
        joint_rate_hz=cfg.command_hz,
        joint_limits={P.JOINT_HEAD_PAN: (-cfg.pan_limit_deg, cfg.pan_limit_deg)},
    )
    readers = reader if isinstance(reader, list | tuple) else [reader]
    for r in readers:
        if r is not None and getattr(r, "from_robot", False):
            b.add_rx_listener(r.feed)
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


def _wait_status(reader: ModuleReader, seen: int, timeout: float) -> bool:
    """Wait for an "@FP on/off" status line after the ``seen``-th one."""
    end = time.monotonic() + timeout
    while reader.status_count <= seen and time.monotonic() < end:
        time.sleep(0.02)
    return reader.status_count > seen


def _wait_auto(reader: ModuleReader, seen: int, timeout: float, want: bool) -> bool:
    """Wait for an "@FP auto 0/1" line after the ``seen``-th one that reports ``want``; return
    whether any such line came. MEASURED on the robot (USB, 2026-11-18): XPS~ and Xps~ print the
    status twice, first from the firmware's own module switch with the OLD value, then from the
    module with the new one, so the first line after the command is not the answer."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if reader.auto_count > seen and reader.autostart is want:
            break
        time.sleep(0.02)
    return reader.auto_count > seen


def detect_module(b: Bittle, reader: ModuleReader) -> bool:
    """X?~: is our camera module (letter P) in the firmware? It prints "@FP on ...;" or
    "@FP off;" on every port after the module list; over USB the list itself also shows P.
    X? exists in the stock firmware and changes nothing there."""
    seen = reader.status_count
    lines = b.send(x_cmd("?"), echo="X", timeout=X_TIMEOUT_S)
    if _wait_status(reader, seen, FP_STATUS_WAIT_S):
        return True
    modules = P.parse_module_list(lines or [])
    return bool(modules) and "P" in modules


def start_module(b: Bittle, reader: ModuleReader, onboard: bool) -> tuple[str | None, list[str]]:
    """Voice off, our camera module on (passive), then either sit (the laptop computes) or
    on-board control on (the firmware sits the robot itself). Returns (an error in plain
    words or None, notes). A missing echo is a note, not an error."""
    notes: list[str] = []
    if b.send(x_cmd("a"), echo="X", timeout=X_TIMEOUT_S) is None:
        notes.append("no reply to Xa~ (voice off); continuing")
    seen = reader.status_count
    if b.send(x_cmd("P"), echo="X", timeout=X_TIMEOUT_S) is None:
        notes.append("no reply to XP~; continuing")
    if _wait_status(reader, seen, FP_STATUS_WAIT_S):
        if reader.module_on is False:
            return "the robot did not switch its camera module on", notes
        if reader.lines_on is False:  # switched off earlier in this power-on with XPp~
            b.send(x_cmd("PP"), echo="X", timeout=X_TIMEOUT_S)
            notes.append("the module's result lines were off: XPP~ sent")
    if onboard:
        if b.send(x_cmd("PB"), echo="X", timeout=ONBOARD_START_TIMEOUT_S) is None:
            notes.append("no reply to XPB~ (on-board control on); continuing")
    elif not b.skill(P.SKILL_SIT, timeout=5.0):
        notes.append("no reply to ksit; continuing")
    return None, notes


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
    """[On-board control off], head to centre, [camera off], voice on, [IMU on], rest; every
    step is sent even if an echo is missing. Returns True if the camera is off: its echo came,
    or any later reply did (then the robot was answering after Xc~). With our module the
    answer is always True (its saved state is harmless; a note is printed if Xp~ was not
    confirmed). Raises LinkLost / TransportError if the link dies on the way."""
    robot_cam = cfg.camera_source == "robot"
    module = cfg.camera_source == "module"
    _alive(b)
    if module and cfg.onboard:
        b.send(x_cmd("Pb"), echo="X", timeout=X_TIMEOUT_S)  # the robot stops moving the head
        _alive(b)
    b.forget_joint_targets()
    time.sleep(JOINT_SETTLE_S)  # a head command already on its way goes out first
    b.send(P.encode_joint(P.JOINT_HEAD_PAN, 0, b.joint_token), echo=b.joint_token, timeout=1.0)
    _alive(b)
    off = True
    t_off = time.monotonic()
    if robot_cam:
        off = b.send(x_cmd("c"), echo="X", timeout=X_TIMEOUT_S) is not None
        _alive(b)
    elif module:
        off = b.send(x_cmd("p"), echo="X", timeout=X_TIMEOUT_S) is not None
        _alive(b)
    if cfg.voice_on_at_exit:
        b.send(x_cmd("A"), echo="X", timeout=X_TIMEOUT_S)
    if robot_cam:
        b.send(b"gU\n", echo="g", timeout=1.5)  # IMU updates back on (the camera stopped them)
    _alive(b)
    b.close(rest=True)
    if module:
        if not off:
            say(f"fly-pursuit: no reply to Xp~; {MODULE_HARMLESS}")
        return True
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
    link, ``reconnect()`` is tried once. Returns True if that was done (the camera off, or
    never the stock module's); if not, what to do has been printed."""
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
    elif cfg.camera_source == "module":
        say(f"fly-pursuit: could not rest the robot: switch it off. ({MODULE_HARMLESS}.)")
    else:
        say("fly-pursuit: could not rest the robot: switch it off")
    return False


def camera_off(
    make_transport: Callable[[], object], cfg: HeadConfig, say: Callable[[str], None] = print
) -> int:
    """``--camera-off``: switch the camera off and make the robot save that, then rest.

    Xc~ alone saves nothing if the camera flag was only set in flash (a failed start clears it
    in memory only), so voice off and on again forces the module list to be written. With our
    module in the firmware (asked with X?~) Xps~ is sent first: module off and its "follow after
    power-on" setting off, both saved (an older build of the module takes it as Xp~). The
    output says the setting was cleared only when the robot confirms it with "@FP auto 0;"."""
    reader = ModuleReader()
    try:
        b = connect(make_transport(), cfg, reader)
    except Exception as e:  # noqa: BLE001 - no port, no Bluetooth, robot off
        say(f"fly-pursuit: could not connect: {e}")
        return 4
    ok = True
    try:
        b.forget_joint_targets()
        module = detect_module(b, reader)
        cleared = False
        if module:
            newer = reader.autostart is not None  # X?~ was followed by "@FP auto"
            seen = reader.auto_count
            ok &= b.send(x_cmd("ps"), echo="X", timeout=X_TIMEOUT_S) is not None
            if newer:
                _wait_auto(reader, seen, FP_STATUS_WAIT_S, want=False)
            cleared = reader.auto_count > seen and reader.autostart is False
        steps = [
            (x_cmd("c"), "X", X_TIMEOUT_S),
            (x_cmd("a"), "X", X_TIMEOUT_S),
            (x_cmd("A"), "X", X_TIMEOUT_S),
            (b"gU\n", "g", 1.5),
        ]
        for data, echo, timeout in steps:
            ok &= b.send(data, echo=echo, timeout=timeout) is not None
    finally:
        b.close(rest=True)
    if not ok:
        say("fly-pursuit: some commands were not confirmed by the robot; run --camera-off again")
        return 1
    say(
        "fly-pursuit: camera off and saved"
        + (" (and our camera module off)" if module else "")
        + ", voice on, IMU on, robot resting"
        + (
            "; follow a face after power-on cleared (the robot confirmed '@FP auto 0;')"
            if cleared
            else ""
        )
    )
    return 0


def set_autostart(
    make_transport: Callable[[], object], cfg: HeadConfig, on: bool
) -> tuple[int, str]:
    """``--autostart on|off``: save our module's "follow after power-on" setting, then rest.

    X?~ first (is the module there?). On: XPS~ (module on, setting saved on), wait for
    "@FP auto 1;", then Xp~ so the camera is off again and the robot stays quiet now (the
    setting acts at the next power-on; Xp~ does not change it). Off: Xps~ (module off, setting
    saved off), wait for "@FP auto 0;". Returns (exit code, the final sentence or the reason).
    If the robot was following a face by itself when asked (the autostart stops the voice module
    for that power-on), XA~ is sent last so that voice is not left saved off.
    An older build of the module ignores S / s and prints no "@FP auto": exit 4 and say so."""
    reader = ModuleReader()
    try:
        b = connect(make_transport(), cfg, reader)
    except Exception as e:  # noqa: BLE001 - no port, no Bluetooth, robot off
        return 4, f"could not connect to the robot: {e}"
    rest = True
    try:
        b.forget_joint_targets()
        if not detect_module(b, reader):
            rest = False  # nothing was sent that moves it
            return 4, NO_MODULE_AUTOSTART
        by_itself = bool(reader.control_on)  # it autostarted at this power-on (voice stopped)
        cmd = "PS" if on else "ps"
        name = f"X{cmd}~"
        seen_status, seen_auto = reader.status_count, reader.auto_count
        echo = b.send(x_cmd(cmd), echo="X", timeout=X_TIMEOUT_S) is not None
        got = _wait_auto(reader, seen_auto, AUTOSTART_WAIT_S, want=on)
        value = reader.autostart
        status_seen = reader.status_count > seen_status
        if on:  # whatever happened: the camera off again (an older build switched it on)
            b.send(x_cmd("p"), echo="X", timeout=X_TIMEOUT_S)
        if by_itself:  # the autostart stopped voice without saving; the commands above saved
            b.send(x_cmd("A"), echo="X", timeout=X_TIMEOUT_S)  # the list, so put voice back
        if not got:
            if echo or status_seen:  # it answered, without "@FP auto": an older build
                return 4, OLDER_MODULE.format(cmd=name)
            return 1, f"the robot did not answer {name}: run it again"
        if value is not on:
            return 1, (
                f"the robot reported '@FP auto {int(bool(value))};' after {name}, not "
                f"'@FP auto {int(on)};': run it again"
            )
        return 0, AUTOSTART_ON_DONE if on else AUTOSTART_OFF_DONE
    finally:
        b.close(rest=rest)
