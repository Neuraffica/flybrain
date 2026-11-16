"""Cut the fly out of the hackathon's event cover, for the Flydeck icon.

The cover is Neuraffica's own image (the website repo: apps/web/public/flydeck-cover.webp,
960 px). This fits its pale background, keys it out, un-mixes its colour from the edges, and
drops what is not the fly (the teal pointer and ring, the title's letters). It writes the two
files the icon uses:

    uv run --with pillow python scripts/flydeck-icon/cut_fly.py <cover image>

    apps/flydeck/src/flydeck/icon_fly.png   the cut-out, read by flydeck/icon.py (PNG rows
                                            stored unfiltered, so no image library is needed)
    packages/.../assets/flydeck.svg         the favicon, with a small copy of the cut-out
"""

import base64
import io
import pathlib
import sys

import numpy as np
from PIL import Image, ImageFilter

src = sys.argv[1]
ROOT = pathlib.Path(__file__).resolve().parents[2]
im = np.asarray(Image.open(src).convert("RGB")).astype(float) / 255
x0, y0, x1, y1 = 392, 216, 836, 604  # the fly and its shadow on the 960 px cover
crop = im[y0:y1, x0:x1]
h, w = crop.shape[:2]
yy, xx = np.mgrid[0:h, 0:w]
mx, mn = crop.max(-1), crop.min(-1)
sat = (mx - mn) / np.maximum(mx, 1e-6)

# background: a plane through the pale, unsaturated pixels, refitted twice on its own inliers
pale = (sat < 0.09) & (mx > 0.9)
A = np.c_[xx.ravel(), yy.ravel(), np.ones(h * w)]
for _ in range(3):
    coef = np.linalg.lstsq(A[pale.ravel()], crop.reshape(-1, 3)[pale.ravel()], rcond=None)[0]
    bg = (A @ coef).reshape(h, w, 3)
    pale = np.sqrt(((crop - bg) ** 2).sum(-1)) < 0.03

dist = np.sqrt(((crop - bg) ** 2).sum(-1))
alpha = np.clip((dist - 0.03) / 0.11, 0, 1)
# not the fly: the teal pointer and dashed ring, and the title's letters in the corner
r, g, b = crop[..., 0], crop[..., 1], crop[..., 2]
teal = (g > r + 0.05) & (g > b - 0.02) & (sat > 0.12)
teal = (
    np.asarray(Image.fromarray((teal * 255).astype(np.uint8)).filter(ImageFilter.MaxFilter(3))) > 0
)
alpha = np.where(teal, 0, alpha)
alpha = np.where((yy > 262) & (xx < 84), 0, alpha)
alpha = np.where((yy > 372) & (xx < 300), 0, alpha)
a = (
    np.asarray(
        Image.fromarray((alpha * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.6))
    )
    / 255
)
# the soft shadow runs past the crop: fade it out towards the edges instead of cutting it
edge = np.minimum(np.minimum(xx, w - 1 - xx), np.minimum(yy, h - 1 - yy))
a = a * np.clip(edge / 28, 0, 1)
rgb = (crop - (1 - a[..., None]) * bg) / np.maximum(a[..., None], 0.05)
rgba = np.dstack([np.clip(rgb, 0, 1), a])
cut = Image.fromarray((rgba * 255 + 0.5).astype(np.uint8), "RGBA")

# the app icon's copy: plain PNG rows (filter 0), which flydeck/icon.py can read with zlib alone
sys.path.insert(0, str(ROOT / "apps" / "flydeck" / "src"))
from flydeck import icon

(ROOT / "apps/flydeck/src/flydeck/icon_fly.png").write_bytes(icon.png_bytes(rgba))

# the favicon: the same tile and cards as icon.py, with a small copy of the fly on the top card
small = cut.resize((176, round(176 * h / w)), Image.LANCZOS)
buf = io.BytesIO()
small.save(buf, "PNG", optimize=True)
fw = icon.FLY_WIDTH
fh = fw * h / w
svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64">
  <!-- Flydeck: the fly of the hackathon's cover on the top paper card of a deck, on a
       bio-teal tile. Written by scripts/flydeck-icon/cut_fly.py; flydeck/icon.py draws the same. -->
  <defs>
    <linearGradient id="tile" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0" stop-color="#0E6F66"/>
      <stop offset="1" stop-color="#0A5750"/>
    </linearGradient>
  </defs>
  <rect x="2" y="2" width="60" height="60" rx="15" fill="url(#tile)"/>
  <rect x="8" y="8" width="42" height="42" rx="10" fill="#FAF7F1" fill-opacity="0.38"
        transform="rotate(-9 29 29)"/>
  <rect x="15" y="15" width="43" height="43" rx="10" fill="#FAF7F1"/>
  <image x="{icon.FLY_CENTRE[0] - fw / 2:g}" y="{icon.FLY_CENTRE[1] - fh / 2:g}" width="{fw:g}" height="{fh:g}"
         href="data:image/png;base64,{base64.b64encode(buf.getvalue()).decode()}"/>
</svg>
'''
(ROOT / "packages/flybrain-demo-api/src/flybrain_demo_api/assets/flydeck.svg").write_text(svg)
print(w, h, len(buf.getvalue()))
