"""``flybrain-export``: write C sources for a flybrain model.

uv run flybrain-export escape --out build/fly_escape                  # library + harness
uv run flybrain-export escape --single-header --out firmware/bittle/fly_escape/src
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="flybrain-export", description=__doc__)
    ap.add_argument("model", choices=["escape"])
    ap.add_argument("--out", type=Path, required=True, help="output directory")
    ap.add_argument("--params", default="connectome", help="key in escape_params.json")
    ap.add_argument("--harness", action="store_true", help="also copy the host test harness")
    ap.add_argument(
        "--single-header",
        action="store_true",
        help="write only fly_escape_all.h (for the OpenCatEsp32 module)",
    )
    ap.add_argument(
        "--layout",
        default="compact",
        choices=["compact", "compact-full", "full"],
        help="C data layout (default compact; full = original float32 CSR)",
    )
    args = ap.parse_args(argv)
    from .escape_c import export

    info = export(args.out, args.params, single_header_only=args.single_header, layout=args.layout)
    if args.harness:
        shutil.copy(Path(__file__).with_name("host_harness.c"), args.out / "host_harness.c")
        info["files"].append("host_harness.c")
    print(json.dumps(info, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
