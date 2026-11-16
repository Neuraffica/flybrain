"""The look every flybrain page shares: Neuraffica's "Living Signal" stylesheet, its fonts
and the marks, served by the Flydeck hub and by every demo's dashboard server under
``/assets/`` (works offline: the fonts are in the package, nothing is fetched).

    <link rel="stylesheet" href="/assets/living-signal.css">
    <link rel="icon" href="/assets/neuraffica.svg">        (the Flydeck hub: /assets/flydeck.svg)

``asset(path)`` returns ``(bytes, content type)`` for a request path, or None.
"""

from __future__ import annotations

import base64
import re
from functools import cache
from importlib import resources

PREFIX = "/assets/"
TYPES = {
    ".css": "text/css; charset=utf-8",
    ".svg": "image/svg+xml",
    ".woff2": "font/woff2",
    ".txt": "text/plain; charset=utf-8",
}


def _root():
    return resources.files("flybrain_demo_api").joinpath("assets")


def asset(path: str) -> tuple[bytes, str] | None:
    """The file for a request path under /assets/ (one level of folder at most), or None."""
    if not path.startswith(PREFIX):
        return None
    parts = path[len(PREFIX) :].split("/")
    if not 1 <= len(parts) <= 2 or any(p in ("", ".", "..") or p.startswith(".") for p in parts):
        return None
    ext = "." + parts[-1].rsplit(".", 1)[-1] if "." in parts[-1] else ""
    ctype = TYPES.get(ext)
    f = _root().joinpath(*parts)
    if ctype is None or not f.is_file():
        return None
    return f.read_bytes(), ctype


def missing() -> list[str]:
    """Files the stylesheet or the pages need that are not in the package (empty when whole):
    a build that drops package data shows up here."""
    css = _root().joinpath("living-signal.css")
    names = (
        re.findall(r"url\((fonts/[^)]+)\)", css.read_text(encoding="utf-8"))
        if css.is_file()
        else []
    )
    names += ["living-signal.css", "neuraffica.svg", "flydeck.svg"]
    names += [f"fonts/OFL-{n}.txt" for n in ("Fraunces", "InstrumentSans", "IBMPlexMono")]
    return [n for n in dict.fromkeys(names) if asset(PREFIX + n) is None]


def data_uri(name: str) -> str:
    """A file from the assets folder (top level) as a data: URI."""
    data, ctype = asset(PREFIX + name) or (b"", "text/plain")
    return f"data:{ctype.split(';')[0]};base64,{base64.b64encode(data).decode()}"


# what a page with no server behind it needs: the headline, the body and the mono labels
INLINE_FONTS = (
    "fraunces-latin-opsz-normal.woff2",
    "instrument-sans-latin-wght-normal.woff2",
    "ibm-plex-mono-latin-600-normal.woff2",
)


@cache
def inline_css(fonts: tuple[str, ...] = INLINE_FONTS) -> str:
    """The stylesheet with ``fonts`` embedded and every other face dropped, for a page that
    has no server behind it (Flydeck's window before the hub answers)."""
    css = _root().joinpath("living-signal.css").read_text(encoding="utf-8")

    def face(m: re.Match) -> str:
        name = m.group(1)
        if name not in fonts:
            return ""
        data = base64.b64encode(_root().joinpath("fonts", name).read_bytes()).decode()
        return m.group(0).replace(f"url(fonts/{name})", f"url(data:font/woff2;base64,{data})")

    return re.sub(r"@font-face \{[^}]*url\(fonts/([^)]+)\)[^}]*\}\n?", face, css)
