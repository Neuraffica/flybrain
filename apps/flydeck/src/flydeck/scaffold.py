"""`flydeck new-demo NAME`: copy the hello-robot template into demos/NAME with the names filled
in, and add it to the workspace so `uv sync` installs it and Flydeck lists it."""

from __future__ import annotations

import importlib.util
import re
import shutil
import tomllib
import zlib
from pathlib import Path

TEMPLATE = "hello-robot"
TEMPLATE_PKG = "hello_robot"
TEMPLATE_TITLE = "Hello robot"
TEMPLATE_PORT = "8767"
SKIP = {"__pycache__", ".pytest_cache", ".ruff_cache", "runs"}
NAME_RE = re.compile(r"^[a-z][a-z0-9]*(-[a-z0-9]+)*$")


class ScaffoldError(Exception):
    pass


def check_name(name: str, root: Path) -> str:
    """The Python package name for demo ``name``; raises ScaffoldError if it is unusable."""
    if not NAME_RE.match(name) or not 2 <= len(name) <= 40:
        raise ScaffoldError(
            f"{name!r}: use 2-40 lowercase letters and digits, words joined by dashes "
            "(e.g. fly-groom)"
        )
    if (root / "demos" / name).exists():
        raise ScaffoldError(f"demos/{name} already exists")
    pkg = name.replace("-", "_")
    if pkg == TEMPLATE_PKG:
        raise ScaffoldError("pick a name other than the template's")
    if importlib.util.find_spec(pkg) is not None:
        raise ScaffoldError(f"a Python module called {pkg!r} already exists; pick another name")
    return pkg


def _add_to_root(pyproject: Path, name: str) -> None:
    text = pyproject.read_text()
    data = tomllib.loads(text)
    deps = data.get("project", {}).get("dependencies", [])
    if name not in deps:
        m = re.search(r"(?ms)^dependencies\s*=\s*\[(.*?)\]", text)
        if m is None:
            raise ScaffoldError("cannot find the dependencies list in the root pyproject.toml")
        inner = m.group(1)
        if "\n" in inner:
            body = inner.rstrip()
            sep = "" if body.endswith(",") or not body.strip() else ","
            new_inner = f'{body}{sep}\n    "{name}",\n'
        else:
            items = [i for i in (s.strip() for s in inner.split(",")) if i]
            new_inner = ", ".join([*items, f'"{name}"'])
        text = text[: m.start(1)] + new_inner + text[m.end(1) :]
    sources = data.get("tool", {}).get("uv", {}).get("sources", {})
    if name not in sources:
        m = re.search(r"(?m)^\[tool\.uv\.sources\]\s*$", text)
        line = f"{name} = {{ workspace = true }}\n"
        if m is None:
            text = text.rstrip("\n") + "\n\n[tool.uv.sources]\n" + line
        else:
            # after the last line of that table
            nxt = re.search(r"(?m)^\[", text[m.end() :])
            end = m.end() + (nxt.start() if nxt else len(text) - m.end())
            block = text[m.end() : end].rstrip("\n")
            text = text[: m.end()] + block + "\n" + line + ("\n" if nxt else "") + text[end:]
    tomllib.loads(text)  # still valid
    pyproject.write_text(text)


def new_demo(root: Path, name: str, title: str | None = None) -> Path:
    """Create demos/<name>; return its folder."""
    pkg = check_name(name, root)
    src = root / "demos" / TEMPLATE
    if not (src / "pyproject.toml").is_file():
        raise ScaffoldError(f"template {src} is missing")
    title = title or name.replace("-", " ").capitalize()
    port = str(8800 + zlib.crc32(name.encode()) % 150)
    dest = root / "demos" / name

    def ignore(_dir: str, names: list[str]) -> set[str]:
        return {n for n in names if n in SKIP or n.endswith((".egg-info", ".pyc"))}

    shutil.copytree(src, dest, ignore=ignore)
    try:
        (dest / "src" / TEMPLATE_PKG).rename(dest / "src" / pkg)
        old_test = dest / "tests" / f"test_{TEMPLATE_PKG}.py"
        if old_test.exists():
            old_test.rename(dest / "tests" / f"test_{pkg}.py")
        for f in dest.rglob("*"):
            if not f.is_file() or f.suffix not in (".py", ".toml", ".md", ".html", ".txt"):
                continue
            text = f.read_text()
            new = (
                text.replace(TEMPLATE, name)
                .replace(TEMPLATE_PKG, pkg)
                .replace(TEMPLATE_TITLE, title)
                .replace(TEMPLATE_PORT, port)
            )
            if new != text:
                f.write_text(new)
        _add_to_root(root / "pyproject.toml", name)
    except Exception:
        shutil.rmtree(dest, ignore_errors=True)
        raise
    return dest


def next_steps(root: Path, dest: Path) -> str:
    name = dest.name
    pkg = name.replace("-", "_")
    rel = dest.relative_to(root)
    return f"""Created {rel}/ (a copy of hello-robot) and added it to the workspace.

Next:
  1. uv sync                                # install it; Flydeck lists it after a restart
  2. uv run {name} --mock --open             # try it on the simulator, no robot needed
  3. edit {rel}/src/{pkg}/cli.py      # your idea goes in the main loop
     and {rel}/src/{pkg}/flydeck.py   # its card and settings in Flydeck
  4. uv run pytest {rel}                   # its smoke test, on the simulator
  5. in the Flydeck app: "Use my flybrain folder..." and choose {root}
     (or press Reload list if it shows this folder already)

Read docs/adding-a-demo.md for the rules (always rest on exit, gate leg motion)."""
