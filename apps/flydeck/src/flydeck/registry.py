"""Find the demos: installed entry points in the group ``flybrain.demos``, plus a look at the
``demos/`` folder of this checkout so a demo that was added but not installed yet shows up as a
card that says "run uv sync" instead of silently missing.

A demo that fails to load or validate is still listed, with its error, so its author sees why.
"""

from __future__ import annotations

import importlib
import importlib.util
import site
import tomllib
from dataclasses import dataclass, field
from importlib.metadata import EntryPoint, entry_points
from pathlib import Path
from typing import Any

from flybrain_demo_api import DemoSpec, SpecError

GROUP = "flybrain.demos"


@dataclass
class DemoEntry:
    """One card: a valid spec, or the reason there is none."""

    id: str
    spec: DemoSpec | None = None
    error: str | None = None
    source: str = ""  # entry point "dist: module:attr" or the folder it was found in
    root: Path | None = None  # the demo's folder (for its guide)
    problems: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.spec is not None and self.error is None

    def guide_path(self) -> Path | None:
        if not self.spec or not self.spec.guide or self.root is None:
            return None
        p = (self.root / self.spec.guide).resolve()
        return p if p.is_file() and self.root.resolve() in p.parents else None

    def to_json(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "id": self.id,
            "ok": self.ok,
            "error": self.error,
            "source": self.source,
        }
        if self.spec is not None:
            d.update(self.spec.to_json())
            d["has_guide"] = self.guide_path() is not None
        else:
            d.update(title=self.id, summary="", options=[], transports=[], needs=[])
        d["id"] = self.id
        return d


def find_root(start: Path | None = None) -> Path | None:
    """The checkout: the nearest folder up from ``start`` with a uv workspace pyproject."""
    here = (start or Path(__file__)).resolve()
    for d in [here, *here.parents]:
        pp = d / "pyproject.toml"
        if pp.is_file():
            try:
                data = tomllib.loads(pp.read_text())
            except (OSError, tomllib.TOMLDecodeError):
                continue
            if data.get("tool", {}).get("uv", {}).get("workspace"):
                return d
    return None


def _module_root(module: str) -> Path | None:
    """The folder with pyproject.toml above the module's file (the demo's own folder), for an
    editable install; None for a module installed into site-packages."""
    try:
        spec = importlib.util.find_spec(module.split(".")[0])
    except (ImportError, ValueError):
        return None
    if spec is None or not spec.origin:
        return None
    for d in Path(spec.origin).resolve().parents:
        if d.name == "site-packages":
            return None  # installed from a wheel: no source folder (see discover)
        if (d / "pyproject.toml").is_file():
            return d
    return None


def load_entry(ep: EntryPoint) -> DemoEntry:
    dist = getattr(ep, "dist", None)
    source = f"{dist.name if dist else '?'}: {ep.value}"
    entry = DemoEntry(id=ep.name, source=source)
    try:
        spec = ep.load()
    except Exception as e:  # noqa: BLE001 - a broken demo becomes a card with its error
        entry.error = f"cannot load {ep.value}: {type(e).__name__}: {e}"
        return entry
    if not isinstance(spec, DemoSpec):
        entry.error = f"{ep.value} is a {type(spec).__name__}, not a flybrain_demo_api.DemoSpec"
        return entry
    try:
        spec.validate()
    except SpecError as e:
        entry.error = f"invalid spec: {e}"
        return entry
    entry.spec = spec
    entry.id = spec.id
    entry.root = _module_root(ep.value.split(":")[0])
    return entry


def declared_in_folder(demos_dir: Path) -> dict[str, Path]:
    """Entry-point names declared by ``demos/*/pyproject.toml`` -> that demo's folder."""
    out: dict[str, Path] = {}
    if not demos_dir.is_dir():
        return out
    for pp in sorted(demos_dir.glob("*/pyproject.toml")):
        try:
            data = tomllib.loads(pp.read_text())
        except (OSError, tomllib.TOMLDecodeError):
            continue
        eps = data.get("project", {}).get("entry-points", {}).get(GROUP, {})
        for name in eps:
            out[name] = pp.parent
    return out


def refresh_paths() -> None:
    """Pick up packages installed since start (``uv sync`` while Flydeck runs): re-read the
    .pth files of site-packages, where editable installs put their source folders."""
    for d in site.getsitepackages():
        site.addsitedir(d)
    importlib.invalidate_caches()


def discover(
    root: Path | None = None, eps: list[EntryPoint] | None = None, refresh: bool = False
) -> list[DemoEntry]:
    """Every demo card, sorted by title; duplicates and broken specs come back with errors."""
    if refresh:
        refresh_paths()
    eps = list(entry_points(group=GROUP)) if eps is None else eps
    declared = declared_in_folder(root / "demos") if root is not None else {}
    found: list[DemoEntry] = []
    by_id: dict[str, DemoEntry] = {}
    for ep in sorted(eps, key=lambda e: e.name):
        entry = load_entry(ep)
        if entry.root is None:
            # installed from a wheel (as in Flydeck.app): the guide is in root/demos/<name>
            entry.root = declared.get(ep.name)
        if entry.ok and entry.id in by_id:
            entry.error = (
                f"duplicate id {entry.id!r}: also published by {by_id[entry.id].source}; "
                "rename one of them"
            )
            entry.spec, entry.id = None, f"{entry.id}#{len(found)}"
        elif entry.ok:
            by_id[entry.id] = entry
        if entry.ok and entry.id != ep.name:
            entry.problems.append(
                f"entry point name {ep.name!r} differs from the spec id {entry.id!r}"
            )
        found.append(entry)
    if root is not None:
        known = {ep.name for ep in eps}
        for name, folder in declared.items():
            if name not in known:
                found.append(
                    DemoEntry(
                        id=name,
                        error=f"found in {folder.relative_to(root)} but not installed: run "
                        "`uv sync` in the checkout, then restart Flydeck",
                        source=str(folder),
                        root=folder,
                    )
                )
    found.sort(key=lambda e: (not e.ok, (e.spec.title if e.spec else e.id).lower()))
    return found
