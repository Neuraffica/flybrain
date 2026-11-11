"""``flybrain-data`` command line: download raw data, build and inspect circuit packs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="flybrain-data", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("download", help="fetch FlyWire v783 connections + annotations")
    d.add_argument("--data-dir", type=Path, default=None)
    d.add_argument(
        "--only",
        choices=["connections", "annotations", "neuropil_pre", "neuropil_post"],
        default=None,
    )
    d.add_argument(
        "--required-only",
        action="store_true",
        help="skip the optional per-neuron neuropil count files (~250 MB)",
    )

    b = sub.add_parser("build-pack", help="extract a circuit pack from the raw files")
    b.add_argument("name", nargs="?", default="central_complex")
    b.add_argument("--data-dir", type=Path, default=None)
    b.add_argument(
        "--out",
        type=Path,
        default=None,
        help="output .npz (default: the shipped pack inside the package)",
    )
    b.add_argument("--download", action="store_true", help="download raw files if missing")

    i = sub.add_parser("info", help="print a pack summary and its metadata")
    i.add_argument("name", nargs="?", default="central_complex")
    i.add_argument("--variant", default="connectome", choices=["connectome", "rewired", "random"])
    i.add_argument("--seed", type=int, default=0)
    i.add_argument("--json", action="store_true", help="print full metadata as JSON")

    args = p.parse_args(argv)
    if args.cmd == "download":
        from .download import FILES, REQUIRED, download

        keys = [args.only] if args.only else list(REQUIRED if args.required_only else FILES)
        for k in keys:
            print(download(FILES[k], args.data_dir))
    elif args.cmd == "build-pack":
        from .build import build_pack

        if args.download:
            from .download import fetch_flywire

            fetch_flywire(args.data_dir)
        build_pack(args.name, data_dir=args.data_dir, out=args.out)
    elif args.cmd == "info":
        from .pack import load_pack

        pack = load_pack(args.name, variant=args.variant, seed=args.seed)
        print(pack.summary())
        if args.json:
            print(json.dumps(pack.metadata, indent=1))
        else:
            print(pack.metadata.get("attribution", ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
