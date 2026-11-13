import textwrap
from importlib.metadata import EntryPoint

from flydeck import registry


def make_module(tmp_path, monkeypatch, name, body):
    (tmp_path / f"{name}.py").write_text(textwrap.dedent(body))
    monkeypatch.syspath_prepend(str(tmp_path))


GOOD = """
from flybrain_demo_api import DemoSpec
SPEC = DemoSpec(id="{id}", title="{title}", summary="s", module="x.cli")
"""


def ep(name, value):
    return EntryPoint(name=name, value=value, group=registry.GROUP)


def test_good_broken_invalid_and_duplicate(tmp_path, monkeypatch):
    make_module(tmp_path, monkeypatch, "demo_a", GOOD.format(id="demo-a", title="A demo"))
    make_module(tmp_path, monkeypatch, "demo_a2", GOOD.format(id="demo-a", title="Copy of A"))
    make_module(tmp_path, monkeypatch, "demo_bad", "raise RuntimeError('boom')\n")
    make_module(
        tmp_path,
        monkeypatch,
        "demo_invalid",
        "from flybrain_demo_api import DemoSpec\n"
        "SPEC = DemoSpec(id='Not OK', title='t', summary='s', module='m')\n",
    )
    make_module(tmp_path, monkeypatch, "demo_wrong", "SPEC = {'id': 'x'}\n")
    entries = registry.discover(
        eps=[
            ep("demo-a", "demo_a:SPEC"),
            ep("demo-a2", "demo_a2:SPEC"),
            ep("demo-bad", "demo_bad:SPEC"),
            ep("demo-invalid", "demo_invalid:SPEC"),
            ep("demo-wrong", "demo_wrong:SPEC"),
        ]
    )
    ok = [e for e in entries if e.ok]
    assert [e.id for e in ok] == ["demo-a"]
    errors = {e.id.split("#")[0]: e.error for e in entries if not e.ok}
    assert "duplicate id 'demo-a'" in errors["demo-a"]
    assert "RuntimeError: boom" in errors["demo-bad"]
    assert "invalid spec" in errors["demo-invalid"]
    assert "not a flybrain_demo_api.DemoSpec" in errors["demo-wrong"]
    # broken demos still become cards (JSON for the page)
    cards = [e.to_json() for e in entries]
    assert all("error" in c and "title" in c for c in cards)
    assert entries[0].ok  # valid ones first


def test_declared_but_not_installed_demo_is_a_card(tmp_path):
    d = tmp_path / "demos" / "new-idea"
    d.mkdir(parents=True)
    (d / "pyproject.toml").write_text(
        '[project]\nname = "new-idea"\n'
        '[project.entry-points."flybrain.demos"]\nnew-idea = "new_idea.flydeck:SPEC"\n'
    )
    (tmp_path / "demos" / "README.md").write_text("not a demo")
    entries = registry.discover(root=tmp_path, eps=[])
    assert len(entries) == 1
    assert entries[0].id == "new-idea" and "uv sync" in entries[0].error


def test_the_real_demos_are_registered():
    root = registry.find_root()
    entries = {e.id: e for e in registry.discover(root)}
    for demo in ("fly-compass", "fly-escape", "fly-escape-onboard", "hello-robot"):
        assert entries[demo].ok, entries[demo].error
        assert entries[demo].guide_path() is not None, demo
