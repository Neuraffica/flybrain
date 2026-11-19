"""Download raw connectome files into ``data/`` with resume, progress and checksum.

Only the standard library is used, so the download works before anything else is set up.
"""

from __future__ import annotations

import hashlib
import os
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path

USER_AGENT = "flybrain-data/0.0.1 (+https://github.com/neuraffica/flybrain)"


@dataclass(frozen=True)
class RemoteFile:
    name: str
    url: str
    size: int | None = None
    md5: str | None = None
    page: str = ""  # human-facing source page, for attribution
    subdir: str = ""  # stored under data_dir / subdir (e.g. "extra")


ZENODO_RECORD = "https://zenodo.org/records/10676866"
ANNOTATIONS_REPO = "https://github.com/flyconnectome/flywire_annotations"
CODEX_DOWNLOADS = "https://codex.flywire.ai/api/download"

FILES: dict[str, RemoteFile] = {
    "connections": RemoteFile(
        name="proofread_connections_783.feather",
        url="https://zenodo.org/api/records/10676866/files/proofread_connections_783.feather/content",
        size=852_022_274,
        md5="f48f972d262323a102aed49af1396b8a",
        page=ZENODO_RECORD,
    ),
    "annotations": RemoteFile(
        name="Supplemental_file1_neuron_annotations.tsv",
        url=(
            "https://raw.githubusercontent.com/flyconnectome/flywire_annotations/main/"
            "supplemental_files/Supplemental_file1_neuron_annotations.tsv"
        ),
        page=ANNOTATIONS_REPO,
    ),
    # Optional: per-neuron synapse counts by neuropil (used only for the input/output
    # compartment table in the pack metadata).
    "neuropil_pre": RemoteFile(
        name="per_neuron_neuropil_count_pre_783.feather",
        url=(
            "https://zenodo.org/api/records/10676866/files/"
            "per_neuron_neuropil_count_pre_783.feather/content"
        ),
        size=16_853_770,
        md5="90fcdb42c1ba05ed92820840fa1e6ba0",
        page=ZENODO_RECORD,
    ),
    "neuropil_post": RemoteFile(
        name="per_neuron_neuropil_count_post_783.feather",
        url=(
            "https://zenodo.org/api/records/10676866/files/"
            "per_neuron_neuropil_count_post_783.feather/content"
        ),
        size=233_843_050,
        md5="bb5999f10920ade803d9f37097a43a56",
        page=ZENODO_RECORD,
    ),
    # Optional: eye-column assignment of ~45k columnar optic-lobe neurons (Matsliah et al.
    # 2024, CC BY 4.0), from the FlyWire Codex public bucket. Needed only to build the
    # ``pursuit`` pack (LC10a receptive-field azimuths).
    "column_assignment": RemoteFile(
        name="column_assignment.csv.gz",
        url=(
            "https://storage.googleapis.com/flywire-data/codex/data/fafb/783/"
            "column_assignment.csv.gz"
        ),
        size=462_838,
        md5="a5307208f0f89cad770cbb870d8d1a09",
        page=CODEX_DOWNLOADS,
        subdir="extra",
    ),
}
REQUIRED = ("connections", "annotations")


def local_path(key: str, data_dir: Path | None = None) -> Path:
    """Where ``FILES[key]`` lives (or will live) under ``data_dir``."""
    remote = FILES[key]
    root = Path(data_dir) if data_dir else default_data_dir()
    return root / remote.subdir / remote.name


def default_data_dir() -> Path:
    """``$FLYBRAIN_DATA`` if set, else ``data/`` at the workspace root (gitignored).

    The workspace root is the nearest parent of the working directory whose
    ``pyproject.toml`` declares ``[tool.uv.workspace]``; failing that, ``./data``.
    """
    env = os.environ.get("FLYBRAIN_DATA")
    if env:
        return Path(env)
    here = Path.cwd().resolve()
    for d in (here, *here.parents):
        pp = d / "pyproject.toml"
        if pp.is_file() and "[tool.uv.workspace]" in pp.read_text(errors="ignore"):
            return d / "data"
    return here / "data"


def _fmt_bytes(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.1f} {unit}" if unit != "B" else f"{int(n)} B"
        n /= 1024
    return f"{n:.1f} GB"


def _md5(path: Path, chunk: int = 1 << 22) -> str:
    h = hashlib.md5()
    with path.open("rb") as f:
        while block := f.read(chunk):
            h.update(block)
    return h.hexdigest()


def download(
    remote: RemoteFile, data_dir: Path | None = None, *, verify: bool = True, retries: int = 5
) -> Path:
    """Fetch ``remote`` into ``data_dir`` (resuming a ``.part`` file) and return its path."""
    data_dir = Path(data_dir) if data_dir else default_data_dir()
    dest = data_dir / remote.subdir / remote.name
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() and (remote.size is None or dest.stat().st_size == remote.size):
        print(f"[flybrain-data] {remote.name}: already present", file=sys.stderr)
        return dest

    part = dest.with_name(dest.name + ".part")
    for attempt in range(1, retries + 1):
        try:
            _fetch(remote, part)
            break
        except (urllib.error.URLError, TimeoutError, ConnectionError) as exc:
            print(f"\n[flybrain-data] attempt {attempt} failed: {exc}", file=sys.stderr)
            if attempt == retries:
                raise
            time.sleep(2 * attempt)

    if verify and remote.md5:
        print(f"[flybrain-data] {remote.name}: verifying md5 ...", file=sys.stderr)
        got = _md5(part)
        if got != remote.md5:
            raise RuntimeError(
                f"md5 mismatch for {remote.name}: got {got}, expected {remote.md5}. "
                f"Delete {part} and retry."
            )
    part.replace(dest)
    return dest


def _fetch(remote: RemoteFile, part: Path) -> None:
    have = part.stat().st_size if part.exists() else 0
    if remote.size is not None and have == remote.size:
        return
    req = urllib.request.Request(remote.url, headers={"User-Agent": USER_AGENT})
    if have:
        req.add_header("Range", f"bytes={have}-")
    with urllib.request.urlopen(req, timeout=60) as resp:
        if have and resp.status != 206:  # server ignored the range: start over
            have = 0
        total = remote.size
        if total is None:
            length = resp.headers.get("Content-Length")
            total = have + int(length) if length else None
        mode = "ab" if have else "wb"
        done = have
        t0 = time.monotonic()
        last = 0.0
        with part.open(mode) as f:
            while chunk := resp.read(1 << 20):
                f.write(chunk)
                done += len(chunk)
                now = time.monotonic()
                if now - last > 0.5:
                    last = now
                    rate = (done - have) / max(now - t0, 1e-6)
                    pct = f"{100 * done / total:5.1f}%" if total else "  ?  "
                    print(
                        f"\r[flybrain-data] {remote.name}: {pct} {_fmt_bytes(done)}"
                        f"{' / ' + _fmt_bytes(total) if total else ''} at {_fmt_bytes(rate)}/s",
                        end="",
                        file=sys.stderr,
                        flush=True,
                    )
        print(file=sys.stderr)
        if total is not None and done != total:
            raise ConnectionError(f"incomplete download: {done} of {total} bytes")


def fetch_flywire(data_dir: Path | None = None, optional: bool = True) -> dict[str, Path]:
    """Download the FlyWire v783 connections and neuron annotations (~880 MB), plus, if
    ``optional``, the per-neuron neuropil counts (~250 MB) and the eye-column assignment
    (0.5 MB). Never the 9.5 GB synapse table."""
    keys = list(FILES) if optional else list(REQUIRED)
    return {key: download(FILES[key], data_dir) for key in keys}
