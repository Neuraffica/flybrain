"""The Flydeck window: the hub page in a native macOS window (WKWebView, through pywebview).

The window process starts the hub as a child (see sources.py) and shows its page. Links:

    the page's guides and a demo's dashboard   another Flydeck window
    web links (https://...)                    the default browser

Closing the main window, Cmd-Q or Quit on the Dock icon first stops the hub, which stops a
running demo (the robot rests), then closes every window. In Flydeck.app the page can switch
between the built-in demos and a flybrain folder of the user's (also in the Demos menu).
"""

from __future__ import annotations

import html
import json
import logging
import os
import signal
import sys
import threading
import webbrowser
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from . import sources
from .settings import Settings

log = logging.getLogger(__name__)

LOCAL_HOSTS = ("127.0.0.1", "localhost")

# Injected into every extra window: web links go to the default browser, local links that ask
# for a new tab open another Flydeck window. (The main page has the same code built in.)
LINKS_JS = r"""
(function () {
  if (window.__flydeckLinks) return;
  window.__flydeckLinks = true;
  document.addEventListener("click", function (e) {
    var a = e.target && e.target.closest ? e.target.closest("a[href]") : null;
    if (!a || !window.pywebview || !window.pywebview.api) return;
    var u = new URL(a.getAttribute("href"), location.href);
    if (u.protocol !== "http:" && u.protocol !== "https:") return;
    var local = u.hostname === "127.0.0.1" || u.hostname === "localhost";
    if (!local) { e.preventDefault(); window.pywebview.api.open_external(u.href); }
    else if (a.target === "_blank") {
      e.preventDefault();
      window.pywebview.api.open_window(u.href, (a.textContent || "").trim());
    }
  }, true);
})();
"""

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Flydeck</title><style>
:root {{ --paper:#f6f2ea; --card:#fffdf8; --ink:#1f1b16; --muted:#6b6257; --line:#e2d9ca;
  --accent:#b8402a; --bad-soft:#f6d9d4; --bad:#a3271b; color-scheme: light; }}
@media (prefers-color-scheme: dark) {{ :root {{ --paper:#15130f; --card:#1d1a15; --ink:#ece6da;
  --muted:#a0968a; --line:#322d25; --accent:#e8735a; --bad-soft:#3c1d18; --bad:#f08b7a;
  color-scheme: dark; }} }}
body {{ margin:0; background:var(--paper); color:var(--ink);
  font:15px/1.5 system-ui,-apple-system,sans-serif; display:flex; min-height:100vh;
  align-items:center; justify-content:center; }}
main {{ max-width:620px; padding:24px; }}
h1 {{ font-family:"Iowan Old Style",Palatino,Georgia,serif; font-weight:600; margin:0 0 8px; }}
p {{ color:var(--muted); }}
pre {{ background:var(--card); border:1px solid var(--line); border-left:4px solid var(--bad);
  border-radius:8px; padding:10px; white-space:pre-wrap; word-break:break-word;
  font:12px/1.45 ui-monospace,Menlo,monospace; color:var(--ink); max-height:40vh; overflow:auto; }}
button {{ font:inherit; font-weight:600; border-radius:9px; padding:8px 14px; cursor:pointer;
  border:1px solid var(--line); background:var(--card); color:var(--ink); margin:4px 8px 0 0; }}
button.primary {{ background:var(--accent); border-color:var(--accent); color:#fff; }}
</style></head><body><main>{body}</main></body></html>"""


def loading_page(text: str) -> str:
    return PAGE.format(body=f"<h1>Flydeck</h1><p>{html.escape(text)}</p>")


def error_page(title: str, detail: str, switchable: bool, kind: str) -> str:
    buttons = ['<button class="primary" onclick="pywebview.api.retry()">Try again</button>']
    if switchable:
        buttons.append('<button onclick="pywebview.api.choose_folder()">Choose a folder…</button>')
        if kind != "builtin":
            buttons.append('<button onclick="pywebview.api.use_builtin()">Built-in demos</button>')
    buttons.append('<button onclick="pywebview.api.quit()">Quit</button>')
    log_hint = "" if kind == "dev" else f"<p>More in {html.escape(str(sources.log_path()))}</p>"
    return PAGE.format(
        body=f"<h1>{html.escape(title)}</h1><pre>{html.escape(detail)}</pre>{log_hint}"
        + "".join(buttons)
    )


def is_local(url: str) -> bool:
    u = urlsplit(url)
    return u.scheme == "http" and u.hostname in LOCAL_HOSTS


class LinkApi:
    """What every Flydeck window's page may ask of the app: open links."""

    def __init__(self, shell: Shell):
        self._shell = shell

    def open_external(self, url: str) -> dict[str, Any]:
        if urlsplit(str(url)).scheme not in ("http", "https", "mailto"):
            return {"ok": False, "error": "not a web link"}
        webbrowser.open(str(url))
        return {"ok": True}

    def open_window(self, url: str, title: str = "") -> dict[str, Any]:
        if not is_local(str(url)):
            return self.open_external(url)
        self._shell.open_window(str(url), str(title or "Flydeck"))
        return {"ok": True}


class Api(LinkApi):
    """The main window's page: links, plus where the demos come from, and quitting."""

    def source(self) -> dict[str, Any]:
        s = self._shell
        return {
            **s.source.to_json(),
            "switchable": s.switchable,
            "note": s.note,
            "app": s.bundle is not None,
        }

    def choose_folder(self) -> dict[str, Any]:
        return self._shell.choose_folder()

    def use_builtin(self) -> dict[str, Any]:
        return self._shell.use_builtin()

    def retry(self) -> dict[str, Any]:
        self._shell.switch(self._shell.source)
        return {"ok": True}

    def quit(self) -> dict[str, Any]:
        self._shell.request_quit()
        return {"ok": True}


class Shell:
    def __init__(
        self,
        source: sources.Source,
        bundle: Path | None = None,
        settings: Settings | None = None,
        note: str | None = None,
        debug: bool = False,
    ):
        self.source = source
        self.bundle = bundle
        self.switchable = bundle is not None
        self.settings = settings or Settings()
        self.note = note
        self.debug = debug
        self.hub: sources.HubProcess | None = None
        self.main: Any = None
        self.quitting = False
        self.may_close = False
        self._switch_lock = threading.Lock()

    # ---- the hub ------------------------------------------------------------------------
    def _new_hub(self, source: sources.Source) -> sources.HubProcess:
        return sources.HubProcess(
            source,
            log=None if source.kind == "dev" else sources.log_path(),
            ble_allowed=self.bundle is not None,
        )

    def switch(self, source: sources.Source, save: bool = False) -> None:
        """Stop the current hub (a running demo rests the robot), then serve ``source``."""
        with self._switch_lock:
            if self.quitting:
                return
            if self.hub is not None:
                self.main.load_html(
                    loading_page("Stopping the running demo first (the robot rests)…")
                    if self._demo_running()
                    else loading_page("Switching…")
                )
                old, self.hub = self.hub, None  # first, so _watch does not take this as Quit
                old.stop()
            if save:
                if source.kind == "folder":
                    self.settings.set(sources.SETTING, str(source.root))
                else:
                    self.settings.set(sources.SETTING, None)
            self.source = source
            what = "your folder " + source.root.name if source.kind == "folder" else "Flydeck"
            self.main.load_html(loading_page(f"Starting {what}…"))
            hub = self._new_hub(source)
            try:
                url = hub.start()
            except sources.SourceError as e:
                log.warning("hub did not start: %s", e)
                title = (
                    f"Cannot show the demos of {source.root.name}"
                    if source.kind == "folder"
                    else "Flydeck did not start"
                )
                self.main.load_html(error_page(title, str(e), self.switchable, source.kind))
                return
            self.hub = hub
            self.main.load_url(url)
        threading.Thread(target=self._watch, args=(hub,), daemon=True).start()

    def _demo_running(self) -> bool:
        st = self.hub.state() if self.hub else None
        run = (st or {}).get("run") or {}
        return bool(run.get("active"))

    def _watch(self, hub: sources.HubProcess) -> None:
        assert hub.proc is not None
        code = hub.proc.wait()
        if self.quitting or self.hub is not hub:
            return
        log.warning("hub exited with %s", code)
        if code == 0:  # Quit Flydeck on the page: the hub stopped on purpose
            self.request_quit()
            return
        self.hub = None
        self.main.load_html(
            error_page(
                "Flydeck stopped",
                f"The Flydeck hub stopped unexpectedly (exit {code}).",
                self.switchable,
                self.source.kind,
            )
        )

    # ---- choosing the demos -----------------------------------------------------------
    def choose_folder(self) -> dict[str, Any]:
        import webview

        start = str(self.source.root) if self.source.kind == "folder" else str(Path.home())
        picked = self.main.create_file_dialog(webview.FileDialog.FOLDER, directory=start)
        if not picked:
            return {"ok": False, "cancelled": True}
        folder = picked[0] if isinstance(picked, (list, tuple)) else picked
        try:
            src = sources.check_folder(folder)
        except sources.SourceError as e:
            return {"ok": False, "error": str(e)}
        if self._demo_running() and not self._confirm_stop():
            return {"ok": False, "cancelled": True}
        self.note = None
        threading.Thread(target=self.switch, args=(src, True), daemon=True).start()
        return {"ok": True}

    def use_builtin(self) -> dict[str, Any]:
        if self.bundle is None:
            return {"ok": False, "error": "Only Flydeck.app has built-in demos."}
        if self._demo_running() and not self._confirm_stop():
            return {"ok": False, "cancelled": True}
        self.note = None
        threading.Thread(
            target=self.switch, args=(sources.builtin(self.bundle), True), daemon=True
        ).start()
        return {"ok": True}

    def _confirm_stop(self) -> bool:
        return bool(
            self.main.create_confirmation_dialog(
                "Flydeck",
                "A demo is running. Switching stops it first (the robot rests). Switch?",
            )
        )

    # ---- windows ----------------------------------------------------------------------
    def open_window(self, url: str, title: str) -> None:
        import webview

        w = webview.create_window(
            title,
            url,
            js_api=LinkApi(self),
            width=1100,
            height=820,
            min_size=(480, 360),
            text_select=True,
        )

        def on_loaded() -> None:
            w.evaluate_js(LINKS_JS)

        w.events.loaded += on_loaded

    def _on_closing(self) -> bool:
        """The main window is closing (its close button, Cmd-Q, Dock Quit)."""
        if self.may_close:
            return True
        self.request_quit()
        return False  # closed by request_quit once the hub has stopped

    def request_quit(self) -> None:
        if self.quitting:
            return
        self.quitting = True
        threading.Thread(target=self._quit, daemon=True).start()

    def _quit(self) -> None:
        import webview

        hub = self.hub
        if hub is not None and hub.alive:
            hub.stop()  # the page shows the demo stopping, then the window closes
        self.may_close = True
        for w in list(webview.windows):
            w.destroy()

    # ---- run --------------------------------------------------------------------------
    def menu(self) -> list:
        from webview.menu import Menu, MenuAction

        items = [MenuAction("Flydeck guide", lambda: self._open_path("guide"))]
        if self.switchable:
            items = [
                MenuAction("Use my flybrain folder…", lambda: self._from_menu(self.choose_folder)),
                MenuAction("Use the built-in demos", lambda: self._from_menu(self.use_builtin)),
                *items,
            ]
        return [Menu("Demos", items)]

    def _from_menu(self, action) -> None:
        res = action()
        if res.get("error"):
            self.main.evaluate_js(f"alert({json.dumps(res['error'])})")

    def _open_path(self, path: str) -> None:
        if self.hub is not None and self.hub.url:
            self.open_window(self.hub.url + path, "Flydeck guide")

    def _boot(self) -> None:
        threading.Thread(target=self._signal_pump, daemon=True).start()
        self.switch(self.source)

    def _signal_pump(self) -> None:
        """Python runs a signal handler only when the main thread runs Python code; the Cocoa
        event loop does not, so wake it twice a second (Ctrl-C, kill: quit cleanly)."""
        from PyObjCTools import AppHelper

        while not self.may_close:
            AppHelper.callAfter(lambda: None)
            threading.Event().wait(0.5)

    def run(self) -> int:
        import webview

        self.main = webview.create_window(
            "Flydeck",
            html=loading_page("Starting…"),
            js_api=Api(self),
            width=1240,
            height=860,
            min_size=(720, 520),
            text_select=True,
            background_color="#F6F2EA",
        )
        self.main.events.closing += self._on_closing
        signal.signal(signal.SIGINT, lambda *_: self.request_quit())
        signal.signal(signal.SIGTERM, lambda *_: self.request_quit())
        webview.start(
            self._boot,
            debug=self.debug,
            menu=self.menu(),
            private_mode=True,
        )
        # every window is closed; normally the hub has stopped already
        if self.hub is not None and self.hub.alive:
            self.hub.stop()
        return 0


def window_available() -> bool:
    """True when a Flydeck window can open here (macOS with pywebview installed)."""
    if sys.platform != "darwin":
        return False
    try:
        import webview  # noqa: F401
    except ImportError:
        return False
    return True


def run(source: sources.Source, bundle: Path | None, note: str | None = None) -> int:
    debug = os.environ.get("FLYDECK_DEBUG") == "1"
    return Shell(source, bundle=bundle, note=note, debug=debug).run()
