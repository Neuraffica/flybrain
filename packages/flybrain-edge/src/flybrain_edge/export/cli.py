"""``flybrain-export``: write C sources for a flybrain model.

uv run flybrain-export escape --out build/fly_escape                  # library + harness
uv run flybrain-export escape --single-header --out firmware/bittle/fly_escape/src
uv run flybrain-export pursuit --single-header --out firmware/bittle/fly_pursuit/src
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="flybrain-export", description=__doc__)
    ap.add_argument("model", choices=["escape", "pursuit"])
    ap.add_argument("--out", type=Path, required=True, help="output directory")
    ap.add_argument("--params", default="connectome", help="key in escape_params.json")
    ap.add_argument("--harness", action="store_true", help="also copy the host test harness")
    ap.add_argument(
        "--single-header",
        action="store_true",
        help="write only the single header (fly_escape_all.h; pursuit always writes only "
        "fly_pursuit_all.h)",
    )
    ap.add_argument(
        "--layout",
        default="compact",
        choices=["compact", "compact-full", "full"],
        help="C data layout (default compact; full = original float32 CSR)",
    )
    args = ap.parse_args(argv)
    if args.model == "pursuit":
        from .pursuit_c import export as export_pursuit

        if args.harness or args.layout != "compact" or args.params != "connectome":
            ap.error("pursuit takes only --out (and --single-header, which is its only form)")
        print(json.dumps(export_pursuit(args.out), indent=1))
        return 0
    from .escape_c import export

    info = export(args.out, args.params, single_header_only=args.single_header, layout=args.layout)
    if args.harness:
        shutil.copy(Path(__file__).with_name("host_harness.c"), args.out / "host_harness.c")
        info["files"].append("host_harness.c")
    print(json.dumps(info, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
