"""The camera on its own USB cable (``--camera usb``): the Grove Vision AI V2 plugged straight
into the laptop, read with its SSCMA AT protocol. The robot's firmware camera module is not used.

Measured 2026-11-16 (packages/flybrain-edge/NOTES.md):

* a CH343 serial port, VID:PID 1A86:55D3 (the same IDs as the robot's own USB port, so the two
  are told apart by asking ``AT+NAME?\\r``, answered with ``Grove Vision AI V2``), 921600 baud;
* opening the port restarts the module: boot text, then ``{"type": 0, "name": "INIT@STAT?",
  ...}``;
* ``AT+INVOKE=-1,0,1\\r`` gives one JSON result per frame, 15.4 a second, with
  ``"boxes": [[x, y, w, h, score, target], ...]`` (x, y = box centre in a 240 x 240 frame)
  and an EMPTY list when no face is seen; ``AT+BREAK\\r`` stops it;
* a face on the dog's left gave x = 41, one about ahead x = 150: bearing (+ = left) =
  (120 - x) / 240 x field of view.

``SSCMAParser`` and ``UsbCameraReader`` are pure (tests feed them bytes). ``UsbCamera`` owns
the serial port and a reader thread. ``open_serial`` and ``list_ch343_ports`` are module-level
so that tests can replace them with fakes.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from collections.abc import Callable, Iterable

from .camera import FRAME_PX, PositionStore, bearing_rad

log = logging.getLogger(__name__)

BAUD = 921600
VID, PID = 0x1A86, 0x55D3
NAME = "Grove Vision AI V2"
CMD_NAME = b"AT+NAME?\r"
CMD_INVOKE = b"AT+INVOKE=-1,0,1\r"
CMD_BREAK = b"AT+BREAK\r"
MAX_LINE = 16384  # a longer line is dropped (results are a few hundred bytes)
BOOT_WAIT_S = 4.0  # after opening: wait this long at most for INIT@STAT (the module restarts)
NAME_WAIT_S = 2.0


# ---- parsing ---------------------------------------------------------------------------
class SSCMAParser:
    """Bytes in, JSON objects out. Lines end with \\n (or \\r\\n); boot text and other
    non-JSON lines are ignored; a line that grows past ``max_line`` or does not parse is
    dropped."""

    def __init__(self, max_line: int = MAX_LINE):
        self.max_line = max_line
        self._buf = bytearray()
        self._skipping = False  # inside an oversized line: drop until its end
        self.dropped = 0

    def feed(self, data: bytes) -> list[dict]:
        out: list[dict] = []
        self._buf.extend(data)
        while True:
            i = self._buf.find(b"\n")
            if i < 0:
                if len(self._buf) > self.max_line:
                    self._buf.clear()
                    self._skipping = True
                    self.dropped += 1
                return out
            line = bytes(self._buf[:i]).strip()
            del self._buf[: i + 1]
            if self._skipping:
                self._skipping = False
                continue
            if len(line) > self.max_line:
                self.dropped += 1
                continue
            if not line.startswith(b"{"):
                continue  # boot text
            try:
                obj = json.loads(line)
            except ValueError:
                self.dropped += 1
                continue
            if isinstance(obj, dict):
                out.append(obj)


def boxes_of(result: dict) -> list | None:
    """The boxes of an INVOKE result, or None if ``result`` is something else."""
    data = result.get("data")
    if not isinstance(data, dict) or "boxes" not in data:
        return None
    boxes = data["boxes"]
    if not isinstance(boxes, list):
        return None
    return [b for b in boxes if isinstance(b, list | tuple) and len(b) >= 5 and _numbers(b[:5])]


def _numbers(vals) -> bool:
    return all(isinstance(v, int | float) and not isinstance(v, bool) for v in vals)


def best_box(boxes: list) -> list | None:
    """The box with the highest score (index 4), or None for an empty list."""
    return max(boxes, key=lambda b: b[4]) if boxes else None


class UsbCameraReader(PositionStore):
    """Positions from the camera's own USB port. A result arrives every frame; an empty box
    list means lost at once."""

    source = "usb"
    steady = True

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.parser = SSCMAParser()
        self.results = 0
        self.init_seen = False

    def feed(self, data: bytes, t: float) -> None:
        for obj in self.parser.feed(data):
            if "INIT@STAT" in str(obj.get("name", "")):
                self.init_seen = True
            boxes = boxes_of(obj)
            if boxes is None:
                continue
            self.results += 1
            box = best_box(boxes)
            if box is None:
                self.record(t, 0.0, 0.0, 0, 0, None)
                continue
            x, y, w, h, score = (float(v) for v in box[:5])
            xc = x - FRAME_PX / 2  # from the image centre; negative = dog's left
            self.record(
                t, xc, y - FRAME_PX / 2, int(w), int(h), bearing_rad(xc, self.fov_deg), score
            )


# ---- the port --------------------------------------------------------------------------
def open_serial(port: str, baud: int = BAUD):
    """A pyserial port (replaced by a fake in tests)."""
    import serial  # pyserial

    return serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)


def list_ch343_ports() -> list[str]:
    """Serial devices with the CH343 IDs of the camera (and of the robot's own USB port)."""
    from serial.tools import list_ports

    return [p.device for p in list_ports.comports() if (p.vid, p.pid) == (VID, PID)]


def _read_for(ser, seconds: float, until: bytes | None = None) -> bytes:
    buf = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        chunk = ser.read(max(1, getattr(ser, "in_waiting", 0) or 0))
        if chunk:
            buf.extend(chunk)
            if until is not None and until in buf:
                break
    return bytes(buf)


def is_camera(port: str) -> bool:
    """Open ``port``, wait out the module's restart, ask its name."""
    try:
        ser = open_serial(port, BAUD)
    except Exception as e:  # noqa: BLE001 - busy, gone, no permission: not ours
        log.info("cannot open %s: %s", port, e)
        return False
    try:
        _read_for(ser, BOOT_WAIT_S, b"INIT@STAT")
        ser.write(CMD_NAME)
        return NAME.encode() in _read_for(ser, NAME_WAIT_S, NAME.encode())
    except Exception as e:  # noqa: BLE001
        log.info("no answer on %s: %s", port, e)
        return False
    finally:
        try:
            ser.close()
        except Exception:  # noqa: BLE001, S110 - best-effort
            pass


_PREFIXES = ("/dev/cu.", "/dev/tty.", "usbmodem", "wchusbserial", "usbserial-", "usbserial")


def device_key(port: str) -> str:
    """macOS lists one USB serial device under several names (cu./tty., usbmodemXXXX and
    wchusbserialXXXX): reduce a name to what they share."""
    k = port
    for p in _PREFIXES:
        k = k.removeprefix(p)
    return k


def same_device(a: str | None, b: str | None) -> bool:
    return bool(a and b) and (a == b or device_key(a) == device_key(b))


def find_camera_port(skip: Iterable[str | None] = ()) -> str | None:
    """The first CH343 port, not the same device as one in ``skip``, that answers
    ``AT+NAME?`` as the camera. Opening a port restarts what is behind it, so pass the robot's
    own port in ``skip`` whenever it is known."""
    skipped = [s for s in skip if s]
    for port in list_ch343_ports():
        if any(same_device(port, s) for s in skipped):
            continue
        if is_camera(port):
            return port
    return None


class UsbCamera:
    """Opens the camera's port, starts the per-frame results, feeds ``reader`` from a thread;
    ``stop`` sends ``AT+BREAK`` and closes the port."""

    def __init__(
        self, port: str, reader: UsbCameraReader, clock: Callable[[], float] = time.monotonic
    ):
        self.port = port
        self.reader = reader
        self.clock = clock
        self.ser = None
        self.error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """Raises OSError (or pyserial's SerialException, an OSError) if the port fails."""
        self.ser = open_serial(self.port, BAUD)
        boot = _read_for(self.ser, BOOT_WAIT_S, b"INIT@STAT")  # the module restarts on open
        self.reader.feed(boot, self.clock())
        self.ser.write(CMD_INVOKE)
        self._thread = threading.Thread(target=self._run, name="usb-camera", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.ser.read(max(1, self.ser.in_waiting or 0))
            except Exception as e:  # noqa: BLE001 - cable out: say so, the head will hold
                self.error = f"camera cable: {e}"
                log.error("camera on USB stopped: %s", e)
                return
            if data:
                self.reader.feed(data, self.clock())

    def stop(self) -> bool:
        """AT+BREAK, then close. Returns False if it could not be sent."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        ok = False
        if self.ser is not None:
            try:
                self.ser.write(CMD_BREAK)
                ok = True
            except Exception as e:  # noqa: BLE001
                log.error("AT+BREAK failed: %s", e)
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001, S110 - best-effort
                pass
        return ok
