"""The Flydeck icon, drawn in code (no image files in the repo): a fly on a stack of cards.

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


def _rotate(x, y, cx, cy, angle_deg):
    a = np.deg2rad(angle_deg)
    dx, dy = x - cx, y - cy
    return cx + dx * np.cos(a) + dy * np.sin(a), cy - dx * np.sin(a) + dy * np.cos(a)


def render(size: int = SIZE) -> np.ndarray:
    """RGBA float image (size x size x 4), straight alpha."""
    s = size / 1024.0
    yy, xx = np.mgrid[0:size, 0:size].astype(float) + 0.5
    x, y = xx / s, yy / s  # draw in 1024 units
    aa = 1.5 / s
    rgb = np.zeros((size, size, 3))
    alpha = np.zeros((size, size))

    def paint(cov, color, opacity=1.0):
        nonlocal rgb, alpha
        c = (cov * opacity)[..., None]
        rgb = rgb * (1 - c) + color * c
        alpha[:] = alpha + (1 - alpha) * cov * opacity

    # app tile: macOS grid, 824 px square with a 185 px corner radius, warm red gradient
    tile = _cover(_round_rect(x, y, 512, 512, 412, 412, 185), aa)
    t = np.clip((y - 100) / 824, 0, 1)[..., None]
    grad = _hex("#c9492f") * (1 - t) + _hex("#8a2c1c") * t
    c = tile[..., None]
    rgb = grad * c
    alpha = tile.copy()

    paper, ink = _hex("#f6f2ea"), _hex("#1f1b16")
    shadow = _hex("#5e1d12")
    # the deck: two cards behind, one in front (tilted)
    for ang, dx, dy, shade in ((-14, -70, 40, 0.80), (8, 60, 30, 0.90)):
        xr, yr = _rotate(x, y, 512 + dx, 540 + dy, ang)
        card = _cover(_round_rect(xr, yr, 512 + dx, 540 + dy, 230, 300, 40), aa)
        paint(
            _cover(_round_rect(xr, yr, 512 + dx + 10, 540 + dy + 16, 230, 300, 40), 18),
            shadow,
            0.35,
        )
        paint(card, paper * shade)
    card = _cover(_round_rect(x, y, 512, 520, 240, 310, 42), aa)
    paint(_cover(_round_rect(x, y, 522, 540, 240, 310, 42), 22), shadow, 0.45)
    paint(card, paper)

    # the fly, seen from above
    wing = _hex("#9fb3c8")
    for side in (-1, 1):
        paint(_cover(_ellipse(x, y, 512 + side * 92, 545, 70, 150, side * -28), aa), wing, 0.75)
        paint(
            _cover(np.abs(_ellipse(x, y, 512 + side * 92, 545, 70, 150, side * -28)) - 3, aa),
            ink,
            0.5,
        )
    for side in (-1, 1):  # legs
        for k, (ly, ang) in enumerate(((470, -35), (520, 0), (570, 35))):
            lx = 512 + side * (70 + 25 * (k == 1))
            paint(_cover(_ellipse(x, y, lx, ly, 62, 7, side * ang), aa), ink)
    paint(_cover(_ellipse(x, y, 512, 560, 46, 105), aa), ink)  # abdomen
    paint(_cover(_ellipse(x, y, 512, 455, 50, 48), aa), ink)  # thorax
    paint(_cover(_ellipse(x, y, 512, 395, 38, 32), aa), ink)  # head
    red = _hex("#d8432c")
    for side in (-1, 1):
        paint(_cover(_ellipse(x, y, 512 + side * 30, 385, 20, 24), aa), red)

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
