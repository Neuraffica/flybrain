"""Byte transports to the BiBoard: USB serial, BLE (Nordic-UART style) and an in-process mock.

A transport moves raw bytes only; framing and parsing live in :mod:`.protocol` and
:mod:`.robot`. ``read`` returns whatever bytes are available (possibly ``b""``) after waiting at
most ``timeout`` seconds, so a reader thread can poll it without blocking forever.
"""

from __future__ import annotations

import asyncio
import fnmatch
import glob
import queue
import sys
import threading
import time
from abc import ABC, abstractmethod
from collections.abc import Iterable
from typing import Self

from . import protocol as P

# macOS device names for the boards Petoi ships: the BiBoard's native USB CDC shows up as
# usbmodem, the CH340/CH9102 bridge as wchusbserial or usbserial. Order = preference.
# ardSerial.py deleteDuplicatedUsbSerial() also prefers "modem" over "wch" for the same serial.
MAC_PORT_PATTERNS = ("/dev/cu.usbmodem*", "/dev/cu.wchusbserial*", "/dev/cu.usbserial*")
LINUX_PORT_PATTERNS = ("/dev/ttyACM*", "/dev/ttyUSB*")
_IGNORED = ("*Bluetooth-Incoming-Port*", "*debug-console*", "/dev/cu.SLAB_USBtoUART*")


class TransportError(RuntimeError):
    pass


class Transport(ABC):
    name: str = "transport"
    description: str = ""

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def write(self, data: bytes) -> None: ...

    @abstractmethod
    def read(self, timeout: float = 0.05) -> bytes: ...

    @abstractmethod
    def close(self) -> None: ...

    @property
    def is_open(self) -> bool:  # pragma: no cover - overridden
        return True

    def __enter__(self) -> Self:
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


# --- USB serial -------------------------------------------------------------------------------
def rank_ports(candidates: Iterable[str], patterns: Iterable[str] = MAC_PORT_PATTERNS) -> list[str]:
    """Order candidate device paths by pattern preference; drop ones that match no pattern.

    When the same board appears as both usbmodemXXXX and wchusbserialXXXX keep only the modem
    device (same rule as ardSerial.deleteDuplicatedUsbSerial)."""
    pats = list(patterns)
    found: list[tuple[int, str]] = []
    for c in candidates:
        if any(fnmatch.fnmatch(c, ig) for ig in _IGNORED):
            continue
        for rank, pat in enumerate(pats):
            if fnmatch.fnmatch(c, pat):
                found.append((rank, c))
                break
    found.sort()
    ordered = [c for _, c in found]
    modem_serials = {c.split("usbmodem", 1)[1] for c in ordered if "usbmodem" in c}
    return [
        c
        for c in ordered
        if not ("wchusbserial" in c and c.split("wchusbserial", 1)[1] in modem_serials)
    ]


def find_serial_ports() -> list[str]:
    """Likely BiBoard ports on this machine, best first."""
    if sys.platform == "darwin":
        pats: tuple[str, ...] = MAC_PORT_PATTERNS
    elif sys.platform.startswith("linux"):
        pats = LINUX_PORT_PATTERNS
    else:
        pats = ()
    cands: list[str] = []
    for p in pats:
        cands.extend(glob.glob(p))
    if not pats:  # Windows: ask pyserial
        try:
            from serial.tools import list_ports

            return [p.device for p in list_ports.comports()]
        except Exception:  # noqa: BLE001 - optional enumeration
            return []
    return rank_ports(cands, pats)


class SerialTransport(Transport):
    name = "usb"

    def __init__(self, port: str | None = None, baud: int = P.BAUD, reset_on_open: bool = True):
        self.port = port
        self.baud = baud
        self.reset_on_open = reset_on_open
        self._ser = None

    @property
    def description(self) -> str:  # type: ignore[override]
        return f"USB serial {self.port or '(auto)'} @ {self.baud}"

    def open(self) -> None:
        import serial  # pyserial

        port = self.port
        if port is None:
            ports = find_serial_ports()
            if not ports:
                raise TransportError(
                    "No Bittle serial port found (looked for "
                    + ", ".join(MAC_PORT_PATTERNS)
                    + "). Plug in the USB cable, switch the battery on, or pass --port."
                )
            port = ports[0]
            self.port = port
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = self.baud
            ser.timeout = 0.05
            ser.write_timeout = 1.0
            if not self.reset_on_open:
                # Opening a CH340 port usually pulses DTR/RTS and reboots the ESP32
                # (ardSerial.py waits 3 s after opening for that reason).
                ser.dtr = False
                ser.rts = False
            ser.open()
        except Exception as e:  # serial.SerialException, PermissionError, ...
            raise TransportError(f"Cannot open {port}: {e}") from e
        self._ser = ser

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def write(self, data: bytes) -> None:
        if not self._ser:
            raise TransportError("serial port not open")
        self._ser.write(data)
        self._ser.flush()

    def read(self, timeout: float = 0.05) -> bytes:
        if not self._ser:
            raise TransportError("serial port not open")
        self._ser.timeout = timeout
        first = self._ser.read(1)
        if not first:
            return b""
        n = self._ser.in_waiting
        return first + (self._ser.read(n) if n else b"")

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None


# --- BLE --------------------------------------------------------------------------------------
class BleTransport(Transport):
    """Nordic-UART style link (src/bleUart.h). bleak is asyncio-only, so it runs on its own
    event loop in a background thread and this class exposes the same blocking interface.

    Firmware quirks that matter (src/bleUart.h:46-90, 126-160):
      * each GATT write whose first byte arrives while idle starts a NEW command; later writes
        are appended until the terminator or a 10 ms gap, so a command must not be split with
        long pauses and two commands must not share one write;
      * on connect the firmware sends "Petoi <MODEL>" and then forces token 'd' (REST);
      * replies are notified in 10-byte pieces and the IMU line has no newline over BLE.
    """

    name = "ble"

    def __init__(
        self, name: str | None = None, address: str | None = None, scan_timeout: float = 8.0
    ):
        self.want_name = name
        self.address = address
        self.scan_timeout = scan_timeout
        self.device_name: str | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._client = None
        self._rx: queue.Queue[bytes] = queue.Queue()
        self._connected = False

    @property
    def description(self) -> str:  # type: ignore[override]
        return f"BLE {self.device_name or self.want_name or '(scan)'}"

    @staticmethod
    def name_matches(adv_name: str | None, want: str | None) -> bool:
        if not adv_name:
            return False
        if want:
            return want.lower() in adv_name.lower()
        return adv_name.startswith(P.BLE_NAME_PREFIX) and adv_name.endswith(P.BLE_NAME_SUFFIX)

    def _run(self, coro, timeout: float):
        assert self._loop is not None
        fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return fut.result(timeout)

    def open(self) -> None:
        try:
            import bleak  # noqa: F401
        except ImportError as e:  # pragma: no cover
            raise TransportError("bleak is not installed") from e
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, name="ble-loop", daemon=True)
        self._thread.start()
        try:
            self._run(self._connect(), self.scan_timeout + 20)
        except TransportError:
            self.close()
            raise
        except Exception as e:
            self.close()
            raise TransportError(f"BLE connect failed: {e!r}") from e

    async def _connect(self) -> None:
        from bleak import BleakClient, BleakScanner

        target = self.address
        if target is None:
            devices = await BleakScanner.discover(timeout=self.scan_timeout, return_adv=True)
            matches = []
            for dev, adv in devices.values():
                nm = adv.local_name or dev.name
                uuids = [u.lower() for u in (adv.service_uuids or [])]
                if self.name_matches(nm, self.want_name) or (
                    self.want_name is None and P.BLE_SERVICE_UUID in uuids and nm
                ):
                    matches.append((adv.rssi, dev, nm))
            if not matches:
                seen = sorted({(adv.local_name or d.name or "?") for d, adv in devices.values()})
                raise TransportError(
                    "No Petoi BLE device found"
                    + (f" matching {self.want_name!r}" if self.want_name else " (name Bittle*_BLE)")
                    + f". Seen: {', '.join(seen[:20]) or 'nothing'}"
                )
            matches.sort(key=lambda m: -(m[0] or -999))
            _, dev, nm = matches[0]
            target = dev
            self.device_name = nm
        client = BleakClient(target)
        await client.connect()
        await client.start_notify(P.BLE_TX_CHAR_UUID, lambda _c, data: self._rx.put(bytes(data)))
        self._client = client
        self._connected = True

    @property
    def is_open(self) -> bool:
        return self._connected

    def write(self, data: bytes) -> None:
        if not self._client:
            raise TransportError("BLE not connected")

        async def _w() -> None:
            for i in range(0, len(data), P.BLE_WRITE_CHUNK):
                await self._client.write_gatt_char(
                    P.BLE_RX_CHAR_UUID, data[i : i + P.BLE_WRITE_CHUNK], response=False
                )
                # keep follow-up chunks well inside the firmware's 10 ms inter-chunk window
                if i + P.BLE_WRITE_CHUNK < len(data):
                    await asyncio.sleep(0.002)

        self._run(_w(), 5.0)

    def read(self, timeout: float = 0.05) -> bytes:
        chunks = []
        try:
            chunks.append(self._rx.get(timeout=timeout))
            while True:
                chunks.append(self._rx.get_nowait())
        except queue.Empty:
            pass
        return b"".join(chunks)

    def close(self) -> None:
        if self._loop is None:
            return
        if self._client is not None:
            try:
                self._run(self._client.disconnect(), 5.0)
            except Exception:  # noqa: BLE001, S110 - best-effort disconnect
                pass
        self._client = None
        self._connected = False
        self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=2)
        self._loop = None


# --- mock ---------------------------------------------------------------------------------------
class MockTransport(Transport):
    """Bytes to and from an in-process :class:`~flybrain_edge.bittle.mock.MockBittle`."""

    name = "mock"
    description = "simulated Bittle X + Arm"

    def __init__(self, robot=None, realtime: bool = True):
        from .mock import MockBittle

        self.robot = robot if robot is not None else MockBittle()
        self.realtime = realtime
        self._open = False

    def open(self) -> None:
        self.robot.power_on()
        if self.realtime:
            self.robot.start()
        self._open = True

    @property
    def is_open(self) -> bool:
        return self._open

    def write(self, data: bytes) -> None:
        if not self._open:
            raise TransportError("mock not open")
        self.robot.receive(data)

    def read(self, timeout: float = 0.05) -> bytes:
        if not self._open:
            raise TransportError("mock not open")
        return self.robot.read_output(timeout)

    def close(self) -> None:
        if self._open:
            self.robot.stop()
        self._open = False


def wait_quiet(t: Transport, quiet_s: float, max_s: float) -> bytes:
    """Read until nothing arrives for ``quiet_s`` (boot chatter), at most ``max_s``."""
    buf = bytearray()
    t0 = time.monotonic()
    last = t0
    while time.monotonic() - t0 < max_s:
        chunk = t.read(0.05)
        now = time.monotonic()
        if chunk:
            buf.extend(chunk)
            last = now
        elif now - last >= quiet_s:
            break
    return bytes(buf)
