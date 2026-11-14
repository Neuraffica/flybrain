"""The Flydeck window's side: where demos come from, the hub child process, links."""

import json
import os
import sys
import textwrap
from importlib.metadata import EntryPoint
from pathlib import Path

import pytest
from flydeck import app, registry, sources
from flydeck.cli import build_parser
from flydeck.settings import Settings


def make_checkout(tmp: Path, venv: bool = True) -> Path:
    root = tmp / "my flybrain"
    (root / "apps" / "flydeck").mkdir(parents=True)
    (root / "pyproject.toml").write_text('[tool.uv.workspace]\nmembers = ["apps/*"]\n')
    if venv:
        py = root / ".venv" / "bin" / "python"
        py.parent.mkdir(parents=True)
        py.write_text(f'#!/bin/sh\nexec "{sys.executable}" "$@"\n')  # this test's Python
        py.chmod(0o755)
    return root


def test_check_folder_explains_what_is_wrong(tmp_path):
    with pytest.raises(sources.SourceError, match="not there"):
        sources.check_folder(tmp_path / "nope")
    with pytest.raises(sources.SourceError, match="not a flybrain folder"):
        sources.check_folder(tmp_path)
    root = make_checkout(tmp_path, venv=False)
    with pytest.raises(sources.SourceError, match="run: uv sync"):
        sources.check_folder(root)


def test_check_folder_accepts_a_subfolder_and_uses_its_venv(tmp_path):
    root = make_checkout(tmp_path)
    (root / "demos" / "mine").mkdir(parents=True)
    src = sources.check_folder(root / "demos" / "mine", probe=False)
    assert src.kind == "folder" and src.root == root.resolve() and src.workdir == src.root
    assert src.python == str(root.resolve() / ".venv" / "bin" / "python")  # not resolved
    cmd = src.hub_command(Path("/tmp/u"), 42)
    assert cmd[:3] == [src.python, "-m", "flydeck"] and "--root" not in cmd
    assert cmd[cmd.index("--parent-pid") + 1] == "42"


def test_check_folder_probes_the_venv(tmp_path):
    root = make_checkout(tmp_path)  # its python has flydeck: this test runs in the checkout
    assert sources.check_folder(root).kind == "folder"
    bad = tmp_path / "bad"
    (bad / "apps" / "flydeck").mkdir(parents=True)
    (bad / "pyproject.toml").write_text("[tool.uv.workspace]\nmembers = []\n")
    py = bad / ".venv" / "bin" / "python"
    py.parent.mkdir(parents=True)
    py.write_text("#!/bin/sh\necho 'No module named flydeck' >&2\nexit 1\n")
    py.chmod(0o755)
    with pytest.raises(sources.SourceError, match="cannot run Flydeck.*uv sync"):
        sources.check_folder(bad)


def test_builtin_source_lives_in_the_bundle(tmp_path, monkeypatch):
    monkeypatch.setenv("FLYDECK_HOME", str(tmp_path / "home"))
    bundle = tmp_path / "Flydeck.app"
    src = sources.builtin(bundle)
    assert src.root == bundle / "Contents" / "Resources" / "flybrain"
    assert src.python == str(bundle / "Contents" / "Resources" / "python" / "bin" / "python3")
    assert src.workdir == tmp_path / "home"  # writable, outside the bundle
    cmd = src.hub_command(tmp_path / "u", 1)
    assert cmd[cmd.index("--root") + 1] == str(src.root)
    assert cmd[cmd.index("--workdir") + 1] == str(src.workdir)


def test_bundle_path_needs_a_real_bundle(tmp_path, monkeypatch):
    monkeypatch.setenv("FLYDECK_BUNDLE", str(tmp_path / "Flydeck.app"))
    assert sources.bundle_path() is None
    (tmp_path / "Flydeck.app" / "Contents").mkdir(parents=True)
    (tmp_path / "Flydeck.app" / "Contents" / "Info.plist").write_text("x")
    assert sources.bundle_path() == tmp_path / "Flydeck.app"


def test_choose_falls_back_to_builtin_with_a_note(tmp_path):
    bundle = tmp_path / "Flydeck.app"
    s = Settings(tmp_path / "s.json")
    src, note = sources.choose(s, bundle)
    assert src.kind == "builtin" and note is None
    s.set(sources.SETTING, str(tmp_path / "moved away"))
    src, note = sources.choose(s, bundle)
    assert src.kind == "builtin" and "built-in demos" in note and "moved away" in note
    root = make_checkout(tmp_path)
    s.set(sources.SETTING, str(root))
    src, note = sources.choose(s, bundle)
    assert src.kind == "folder" and src.root == root.resolve() and note is None


def test_settings_keep_what_another_process_wrote(tmp_path):
    a = Settings(tmp_path / "s.json")
    b = Settings(tmp_path / "s.json")
    a.set("robot", {"target": "x"})
    b.set(sources.SETTING, "/some/where")
    assert json.loads((tmp_path / "s.json").read_text()) == {
        "robot": {"target": "x"},
        sources.SETTING: "/some/where",
    }


def test_hub_process_starts_serves_and_stops(tmp_path, monkeypatch):
    monkeypatch.setenv("FLYDECK_HOME", str(tmp_path / "home"))
    root = registry.find_root()
    src = sources.Source("builtin", root, sys.executable, tmp_path / "work")  # --root, --workdir
    hub = sources.HubProcess(src, log=tmp_path / "Flydeck.log", ble_allowed=True)
    try:
        url = hub.start(timeout=60)
        assert url.startswith("http://127.0.0.1:")
        st = hub.state()
        assert st["app"]["in_app"] and not st["app"]["ble_blocked"]
        assert st["app"]["workdir"] == str(tmp_path / "work")
        assert {d["id"] for d in st["demos"]} >= {"hello-robot", "fly-compass"}
    finally:
        code = hub.stop(timeout=30)
    assert code == 0 and not hub.alive
    log = (tmp_path / "Flydeck.log").read_text()
    assert "Flydeck hub starting" in log and "flydeck: bye" in log


def test_hub_process_reports_a_failed_start(tmp_path):
    src = sources.Source("folder", tmp_path, "/usr/bin/false", tmp_path)
    hub = sources.HubProcess(src, log=tmp_path / "Flydeck.log")
    with pytest.raises(sources.SourceError, match="stopped while starting"):
        hub.start(timeout=10)


def test_hub_env_allows_bluetooth_only_inside_the_app(tmp_path, monkeypatch):
    monkeypatch.setenv("FLYBRAIN_BLE_BUNDLE", "1")
    src = sources.dev(tmp_path, sys.executable)
    assert "FLYBRAIN_BLE_BUNDLE" not in sources.HubProcess(src).env()
    assert sources.HubProcess(src, ble_allowed=True).env()["FLYBRAIN_BLE_BUNDLE"] == "1"


def test_links_local_or_web():
    assert app.is_local("http://127.0.0.1:8765/")
    assert app.is_local("http://localhost:8760/guide")
    assert not app.is_local("https://127.0.0.1/")
    assert not app.is_local("https://github.com/Neuraffica/flybrain")
    api = app.LinkApi(shell=None)
    assert api.open_external("file:///etc/passwd") == {"ok": False, "error": "not a web link"}


def test_error_page_escapes_and_offers_a_way_out():
    page = app.error_page("Cannot <start>", "a & b", switchable=True, kind="folder")
    assert "Cannot &lt;start&gt;" in page and "a &amp; b" in page
    assert "use_builtin" in page and "choose_folder" in page and "retry" in page
    page = app.error_page("x", "y", switchable=False, kind="dev")
    assert "use_builtin" not in page and "choose_folder" not in page


def test_cli_modes():
    p = build_parser()
    assert p.parse_args(["--browser"]).browser
    assert p.parse_args(["--no-open", "--root", "/r", "--workdir", "/w"]).root == "/r"
    assert p.parse_args(["build-app", "--no-dmg"]).no_dmg
    assert p.parse_args(["install-app", "--dir", "/x"]).dir == "/x"


def test_guide_found_for_a_demo_installed_from_a_wheel(tmp_path, monkeypatch):
    """In Flydeck.app the demo packages have no source folder: the guide comes from
    root/demos/<name>, next to the pyproject.toml that declares the entry point."""
    mods = tmp_path / "lib" / "site-packages"  # below a folder with a pyproject.toml
    mods.mkdir(parents=True)
    (tmp_path / "pyproject.toml").write_text("[project]\nname = 'not-the-demo'\n")
    (mods / "wheeldemo.py").write_text(
        textwrap.dedent(
            """
            from flybrain_demo_api import DemoSpec
            SPEC = DemoSpec(id="wheel-demo", title="Wheel demo", summary="s",
                            module="wheeldemo", moves_robot=False, guide="guide.html")
            """
        )
    )
    monkeypatch.syspath_prepend(str(mods))
    root = tmp_path / "root"
    demo = root / "demos" / "wheel-demo"
    demo.mkdir(parents=True)
    (demo / "pyproject.toml").write_text(
        '[project.entry-points."flybrain.demos"]\nwheel-demo = "wheeldemo:SPEC"\n'
    )
    (demo / "guide.html").write_text("<html>guide</html>")
    ep = EntryPoint("wheel-demo", "wheeldemo:SPEC", registry.GROUP)
    [entry] = registry.discover(root, eps=[ep])
    assert entry.ok and entry.root == demo
    assert entry.guide_path() == (demo / "guide.html").resolve()


@pytest.mark.skipif(not app.window_available(), reason="needs macOS with pywebview")
def test_window_library_imports():
    import webview

    assert hasattr(webview, "FileDialog") and os.environ.get("FLYDECK_BUNDLE") is None


class FakeHub:
    """Stands in for a hub child: exits 0 when stopped, as the real one does."""

    def __init__(self, source):
        import threading

        self.source, self.url, self.alive = source, None, False
        self._gone = threading.Event()
        self.proc = self

    def start(self):
        self.url, self.alive = f"http://127.0.0.1:1/{self.source.root.name}/", True
        return self.url

    def wait(self):
        self._gone.wait(10)
        return 0

    def stop(self):
        self.alive = False
        self._gone.set()
        return 0

    def state(self):
        return None


class FakeWindow:
    def __init__(self):
        self.loaded = []

    def load_url(self, url):
        self.loaded.append(url)

    def load_html(self, html):
        self.loaded.append("html")


def test_switching_folders_does_not_quit_and_is_remembered(tmp_path, monkeypatch):
    import time

    bundle = tmp_path / "Flydeck.app"
    settings = Settings(tmp_path / "s.json")
    shell = app.Shell(sources.builtin(bundle), bundle=bundle, settings=settings)
    shell.main = FakeWindow()
    monkeypatch.setattr(shell, "_new_hub", FakeHub)
    quits = []
    monkeypatch.setattr(shell, "request_quit", lambda: quits.append(1))
    shell.switch(shell.source)
    first = shell.hub
    folder = sources.check_folder(make_checkout(tmp_path), probe=False)
    shell.switch(folder, save=True)
    time.sleep(0.3)  # let the first hub's watcher see its exit
    assert not quits and not first.alive and shell.hub.source == folder
    assert settings.get(sources.SETTING) == str(folder.root)
    assert shell.main.loaded[-1].endswith("/my flybrain/")
    shell.switch(sources.builtin(bundle), save=True)
    time.sleep(0.3)
    assert not quits and settings.get(sources.SETTING) is None
