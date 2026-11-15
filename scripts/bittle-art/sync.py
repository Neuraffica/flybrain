"""Publish identical artwork bytes to the explicitly supplied website worktree."""

import base64
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "packages/flybrain-demo-api/src/flybrain_demo_api/static/bittle"


def main():
    target = Path(sys.argv[1]).resolve()
    if not (target / "apps/web/src/flydeck/EscapeSimDemo.tsx").is_file():
        raise SystemExit("Expected the synaptix worktree, not a public asset directory")
    manifest = {
        p.name: {"bytes": p.stat().st_size, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
        for p in sorted(ASSETS.iterdir())
        if p.name != "manifest.json"
    }
    (ASSETS / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    dest = target / "apps/web/public/bittle"
    dest.mkdir(parents=True, exist_ok=True)
    for path in ASSETS.iterdir():
        shutil.copy2(path, dest / path.name)
    # Keep the file:// guide and bundled hub guide self-contained.
    import io

    from PIL import Image

    still = Image.open(ROOT / "work/bittle/renders/preview-side.png")
    out = io.BytesIO()
    still.save(out, format="WEBP", quality=88, method=6)
    uri = "data:image/webp;base64," + base64.b64encode(out.getvalue()).decode()
    guide = ROOT / "demos/fly-escape/guide.html"
    text = guide.read_text()
    replacement = (
        '<image data-bittle="still" href="'
        + uri
        + '" x="215" y="75" width="145" height="145" transform="translate(575 0) scale(-1 1)"/>'
    )
    if 'data-bittle="still"' in text:
        text = re.sub(r'<image data-bittle="still"[^>]+/>', replacement, text)
    else:
        text = re.sub(
            r'<rect x="262" y="110"[^>]+/>\s*<rect x="248" y="138"[^>]+/>', replacement, text
        )
    # The guide can be redistributed alone, so keep the MIT notices with its still.
    comment = "<!-- BITTLE ART NOTICE\n" + (ASSETS / "NOTICE").read_text() + "\n-->"
    if "<!-- BITTLE ART NOTICE" in text:
        text = re.sub(r"<!-- BITTLE ART NOTICE.*?-->", lambda _: comment, text, flags=re.DOTALL)
    else:
        text = text.replace("</svg>", comment + "\n  </svg>", 1)
    guide.write_text(text)


if __name__ == "__main__":
    main()
