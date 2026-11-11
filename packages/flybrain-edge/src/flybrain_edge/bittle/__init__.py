"""Robot link for the Petoi Bittle X (+ Arm) running OpenCatEsp32 firmware.

Quick use::

    from flybrain_edge.bittle import Bittle, open_transport
    bot = Bittle(open_transport(mock=True))
    bot.connect(); bot.stand(); bot.stream_imu()
    print(bot.read_yaw())
    bot.set_joint(0, 30)   # head pan, clamped and rate-limited
    bot.close()            # always rests

See NOTES.md in the package root for the protocol facts and their firmware sources.
"""

from . import protocol
from .mock import MockBittle
from .robot import Bittle, BittleError, ImuSample
from .transport import (
    BleTransport,
    MockTransport,
    SerialTransport,
    Transport,
    TransportError,
    find_serial_ports,
)


def open_transport(
    mock: bool = False,
    port: str | None = None,
    ble: str | bool | None = None,
    mock_robot: MockBittle | None = None,
) -> Transport:
    """Pick a transport from CLI-style options (mock > ble > serial)."""
    if mock:
        return MockTransport(mock_robot)
    if ble is not None and ble is not False:
        return BleTransport(name=ble if isinstance(ble, str) and ble else None)
    return SerialTransport(port)


__all__ = [
    "Bittle",
    "BittleError",
    "BleTransport",
    "ImuSample",
    "MockBittle",
    "MockTransport",
    "SerialTransport",
    "Transport",
    "TransportError",
    "find_serial_ports",
    "open_transport",
    "protocol",
]
