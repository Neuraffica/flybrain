"""Find and check robots. Flydeck runs this in a short-lived child process:

    python -m flydeck.robotcheck scan [--timeout S]       # Bluetooth LE scan
    python -m flydeck.robotcheck check --ble NAME | --port PATH | --mock

and reads the last line, ``FLYDECK_RESULT {json}``. A child process because macOS kills a
process that touches Bluetooth without permission (exit 134): then only this child dies, and
Flydeck can say what happened. A check connects, reads model, firmware and battery, and
disconnects (the robot is left resting); it never holds the link.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from typing import Any

PREFIX = "FLYDECK_RESULT "


def classify_ble_error(text: str) -> str:
    """A plain-language explanation for a bleak / CoreBluetooth failure."""
    t = text.lower()
    if "unauthorized" in t or "denied" in t or "not authorized" in t:
        return (
            "Flydeck is not allowed to use Bluetooth. Open System Settings > Privacy & "
            "Security > Bluetooth and switch Flydeck on, then quit and reopen Flydeck."
        )
    if "powered off" in t or "turned off" in t or "poweredoff" in t:
        return "Bluetooth is switched off on this Mac. Switch it on in Control Center."
    if "no petoi ble device found" in t or "not found" in t:
        return (
            "No robot found. Is its battery on (long-press the battery button)? Is the Petoi "
            "phone app connected to it? A robot connected to another device does not show up."
        )
    return text


def scan(timeout: float) -> dict[str, Any]:
    from bleak import BleakScanner
    from flybrain_edge.bittle import BleTransport
    from flybrain_edge.bittle import protocol as P

    async def go():
        return await BleakScanner.discover(timeout=timeout, return_adv=True)

    found = asyncio.run(go())
    devices = []
    for dev, adv in found.values():
        name = adv.local_name or dev.name
        uart = P.BLE_SERVICE_UUID in [u.lower() for u in (adv.service_uuids or [])]
        if uart or BleTransport.name_matches(name, None):
            devices.append(
                {
                    "name": name or "(no name)",
                    "address": dev.address,
                    "rssi": adv.rssi,
                    "uart": uart,
                }
            )
    devices.sort(key=lambda d: -(d["rssi"] if d["rssi"] is not None else -999))
    return {"ok": True, "devices": devices, "seen": len(found)}


def check(mock: bool, port: str | None, ble: str | None) -> dict[str, Any]:
    from flybrain_edge.bittle import Bittle, MockBittle, MockTransport, open_transport

    transport = MockTransport(MockBittle()) if mock else open_transport(port=port, ble=ble)
    bot = Bittle(transport)
    bot.connect(boot_timeout=0.5 if mock else 12.0)
    try:
        voltage = bot.read_voltage()
        return {
            "ok": True,
            "link": transport.description,
            "name": getattr(transport, "device_name", None) or ble or port or "simulator",
            "model": bot.model,
            "version": bot.version,
            "voltage": voltage,
            "low_power": bot.low_power,
        }
    finally:
        bot.close()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m flydeck.robotcheck")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("scan")
    s.add_argument("--timeout", type=float, default=5.0)
    c = sub.add_parser("check")
    g = c.add_mutually_exclusive_group(required=True)
    g.add_argument("--mock", action="store_true")
    g.add_argument("--port")
    g.add_argument("--ble")
    a = ap.parse_args(argv)
    try:
        res = scan(a.timeout) if a.cmd == "scan" else check(a.mock, a.port, a.ble)
    except Exception as e:  # noqa: BLE001 - every failure is reported as JSON
        text = f"{e}"
        res = {
            "ok": False,
            "error": classify_ble_error(text) if a.cmd == "scan" or a.ble else text,
            "detail": f"{type(e).__name__}: {text}",
        }
    print(PREFIX + json.dumps(res), flush=True)
    return 0 if res.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
