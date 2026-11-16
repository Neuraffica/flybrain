"""The Flydeck icon, drawn in code: a fly, seen from above, on the top paper card of a deck, on
a bio-teal tile, in the Living Signal palette. The favicon
(flybrain_demo_api/assets/flydeck.svg) is the same drawing on the same 64-unit grid.

``icns_bytes()`` returns a macOS .icns with PNG images from 16 to 1024 px.
"""

from __future__ import annotations

import struct
import zlib

import numpy as np

SIZE = 1024
# (icns type, pixel size)
ICNS_TYPES = (
    ("icp4", 16),
    ("icp5", 32),
    ("icp6", 64),
    ("ic07", 128),
    ("ic08", 256),
    ("ic09", 512),
    ("ic10", 1024),
    ("ic11", 32),
    ("ic12", 64),
    ("ic13", 256),
    ("ic14", 512),
)


def _hex(c: str) -> np.ndarray:
    return np.array([int(c[i : i + 2], 16) / 255 for i in (1, 3, 5)])


def _cover(sdf: np.ndarray, aa: float) -> np.ndarray:
    """Signed distance (negative inside) -> coverage 0..1 with a soft edge of ``aa`` pixels."""
    return np.clip(0.5 - sdf / aa, 0.0, 1.0)


def _round_rect(x, y, cx, cy, hw, hh, r):
    qx = np.abs(x - cx) - (hw - r)
    qy = np.abs(y - cy) - (hh - r)
    out = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
    return out + np.minimum(np.maximum(qx, qy), 0) - r


def _ellipse(x, y, cx, cy, rx, ry, angle_deg=0.0):
    a = np.deg2rad(angle_deg)
    dx, dy = x - cx, y - cy
    u = dx * np.cos(a) + dy * np.sin(a)
    v = -dx * np.sin(a) + dy * np.cos(a)
    k = np.hypot(u / rx, v / ry)
    return (k - 1.0) * min(rx, ry)  # approximate distance, good enough for an icon


# Living Signal (DESIGN.md): primary, primary-strong, paper, ink, and the fly's wing and eye
PRIMARY, PRIMARY_STRONG, PAPER, INK = "#0e6f66", "#0a5750", "#faf7f1", "#1c2a30"
WING, EYE = "#d6e8f0", "#a3302a"  # pip-wing, pip-eye
# The fly on the 64-unit grid of flydeck.svg: (cx, cy, rx, ry, rotation in degrees). Wings and
# legs are listed for the right side; the left side mirrors them about x = FLY_X.
FLY_X = 36.5
FLY_WING = (8.6, 2.8, 6.4, 13.6, -28.0)  # dx from FLY_X, dy from the front card's centre
FLY_LEGS = ((6.4, -4.7, 5.7, 0.7, -35.0), (8.7, 0.0, 5.7, 0.7, 0.0), (6.4, 4.7, 5.7, 0.7, 35.0))
FLY_BODY = ((0, 3.8, 4.3, 9.7), (0, -6.0, 4.6, 4.4), (0, -11.6, 3.5, 3.0))  # abdomen, thorax, head
FLY_EYE = (2.8, -12.5, 1.9, 2.2)
BACK_CARD = (29, 29, 21, 10, -9.0)  # centre x, y, half side, corner radius, rotation (deg)
FRONT_CARD = (36.5, 36.5, 21.5, 10, 0.0)


def render(size: int = SIZE) -> np.ndarray:
    """RGBA float image (size x size x 4), straight alpha."""
    s = size / 1024.0
    yy, xx = np.mgrid[0:size, 0:size].astype(float) + 0.5
    x, y = xx / s, yy / s  # draw in 1024 units
    aa = 1.5 / s
    k = 824 / 60  # the svg's tile (2..62) onto the macOS grid's 824 px tile (100..924)

    def g(v):
        return 100 + (v - 2) * k

    rgb = np.zeros((size, size, 3))
    alpha = np.zeros((size, size))

    def paint(cov, color, opacity=1.0):
        nonlocal rgb, alpha
        c = (cov * opacity)[..., None]
        rgb = rgb * (1 - c) + color * c
        alpha[:] = alpha + (1 - alpha) * cov * opacity

    # tile: macOS grid, 824 px square with a 185 px corner radius, primary to primary-strong
    tile = _cover(_round_rect(x, y, 512, 512, 412, 412, 185), aa)
    t = np.clip((y - 100) / 824, 0, 1)[..., None]
    rgb = (_hex(PRIMARY) * (1 - t) + _hex(PRIMARY_STRONG) * t) * tile[..., None]
    alpha = tile.copy()

    paper = _hex(PAPER)
    shadow = _hex("#04302b")  # on-primary (dark): the deck's shadow on the teal
    for (cx, cy, half, r, ang), opacity in ((BACK_CARD, 0.38), (FRONT_CARD, 1.0)):
        cx, cy, half, r = g(cx), g(cy), half * k, r * k
        a = np.deg2rad(ang)  # svg rotate(): inverse-rotate the pixel into the card's frame
        dx, dy = x - cx, y - cy
        u, v = cx + dx * np.cos(a) + dy * np.sin(a), cy - dx * np.sin(a) + dy * np.cos(a)
        if opacity == 1.0:
            paint(_cover(_round_rect(u, v, cx + 6, cy + 14, half, half, r), 28), shadow, 0.30)
        paint(_cover(_round_rect(u, v, cx, cy, half, half, r), aa) * tile, paper, opacity)

    # the fly, seen from above, on the front card
    fx, fy = g(FLY_X), g(FRONT_CARD[1])
    ink = _hex(INK)

    def part(dx, dy, rx, ry, ang=0.0, side=1):
        return _ellipse(x, y, fx + side * dx * k, fy + dy * k, rx * k, ry * k, side * ang)

    for side in (-1, 1):
        wing = part(*FLY_WING, side=side)
        paint(_cover(wing, aa), _hex(WING), 0.9)
        paint(_cover(np.abs(wing) - 0.22 * k, aa), ink, 0.55)
    for side in (-1, 1):
        for leg in FLY_LEGS:
            paint(_cover(part(*leg, side=side), aa), ink)
    for body in FLY_BODY:
        paint(_cover(part(*body), aa), ink)
    for side in (-1, 1):
        paint(_cover(part(*FLY_EYE, side=side), aa), _hex(EYE))

    img = np.concatenate([rgb, alpha[..., None]], axis=-1)
    return np.clip(img, 0, 1)


def _resize(img: np.ndarray, size: int) -> np.ndarray:
    n = img.shape[0] // size
    if n <= 1:
        return img
    # premultiplied area average
    pm = img.copy()
    pm[..., :3] *= pm[..., 3:4]
    small = pm.reshape(size, n, size, n, 4).mean(axis=(1, 3))
    a = small[..., 3:4]
    small[..., :3] = np.where(a > 0, small[..., :3] / np.maximum(a, 1e-6), 0)
    return small


def png_bytes(img: np.ndarray) -> bytes:
    h, w = img.shape[:2]
    data = (np.clip(img, 0, 1) * 255 + 0.5).astype(np.uint8)
    raw = b"".join(b"\x00" + data[row].tobytes() for row in range(h))

    def chunk(tag: bytes, body: bytes) -> bytes:
        return (
            struct.pack(">I", len(body))
            + tag
            + body
            + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def icns_bytes() -> bytes:
    big = render(SIZE)
    pngs: dict[int, bytes] = {}
    body = b""
    for typ, px in ICNS_TYPES:
        if px not in pngs:
            pngs[px] = png_bytes(_resize(big, px))
        data = pngs[px]
        body += typ.encode() + struct.pack(">I", len(data) + 8) + data
    return b"icns" + struct.pack(">I", len(body) + 8) + body
