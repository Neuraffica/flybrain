"""The Flydeck icon: the fly of the hackathon's cover on the top paper card of a deck, on a
bio-teal tile, in the Living Signal palette. The tile and cards are drawn in code; the fly is
``icon_fly.png``, cut out of the cover by scripts/flydeck-icon/cut_fly.py, which also writes
the favicon (flybrain_demo_api/assets/flydeck.svg) with the same layout on the same 64-unit grid.

``icns_bytes()`` returns a macOS .icns with PNG images from 16 to 1024 px.
"""

from __future__ import annotations

import struct
import zlib
from pathlib import Path

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


# Living Signal (DESIGN.md): primary, primary-strong, paper
PRIMARY, PRIMARY_STRONG, PAPER = "#0e6f66", "#0a5750", "#faf7f1"
# the fly on the 64-unit grid of flydeck.svg: its centre and its width (its height follows)
FLY_CENTRE, FLY_WIDTH = (36.5, 36.7), 39.0
FLY_PNG = Path(__file__).with_name("icon_fly.png")
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

    # the fly, on the front card: the cut-out, resampled (bilinear) to its place
    fly = read_png(FLY_PNG.read_bytes())
    fh, fw = fly.shape[:2]
    scale = FLY_WIDTH * k / fw  # 1024-unit pixels per fly pixel
    u = (x - (g(FLY_CENTRE[0]) - fw * scale / 2)) / scale - 0.5
    v = (y - (g(FLY_CENTRE[1]) - fh * scale / 2)) / scale - 0.5
    inside = (u > -0.5) & (u < fw - 0.5) & (v > -0.5) & (v < fh - 0.5)
    u0 = np.clip(np.floor(u).astype(int), 0, fw - 2)
    v0 = np.clip(np.floor(v).astype(int), 0, fh - 2)
    du = np.clip(u - u0, 0, 1)[..., None]
    dv = np.clip(v - v0, 0, 1)[..., None]
    pm = fly.copy()
    pm[..., :3] *= pm[..., 3:4]  # premultiplied, so transparent pixels do not bleed colour
    top = pm[v0, u0] * (1 - du) + pm[v0, u0 + 1] * du
    bottom = pm[v0 + 1, u0] * (1 - du) + pm[v0 + 1, u0 + 1] * du
    sample = (top * (1 - dv) + bottom * dv) * inside[..., None]
    rgb = rgb * (1 - sample[..., 3:4]) + sample[..., :3]

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


def read_png(data: bytes) -> np.ndarray:
    """An RGBA float image from a PNG whose rows are stored unfiltered, as ``png_bytes`` writes."""
    pos, idat, w, h = 8, b"", 0, 0
    while pos < len(data):
        (n,), tag = struct.unpack(">I", data[pos : pos + 4]), data[pos + 4 : pos + 8]
        body = data[pos + 8 : pos + 8 + n]
        if tag == b"IHDR":
            w, h, depth, colour = struct.unpack(">IIBB", body[:10])
            if (depth, colour) != (8, 6):
                raise ValueError("expected an 8-bit RGBA PNG")
        elif tag == b"IDAT":
            idat += body
        pos += n + 12
    rows = np.frombuffer(zlib.decompress(idat), np.uint8).reshape(h, 1 + w * 4)
    if rows[:, 0].any():
        raise ValueError("expected unfiltered PNG rows")
    return rows[:, 1:].reshape(h, w, 4).astype(float) / 255


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
