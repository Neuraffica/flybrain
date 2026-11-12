import json
import os
import shutil
import subprocess
import sys
import tomllib

import pytest
from flydeck import registry, scaffold


def run(*args, **kw):
    """subprocess.run that never raises on a non-zero exit: the tests check it."""
    return subprocess.run(*args, check=False, **kw)


ROOT_PYPROJECT = """[project]
name = "flybrain"
dependencies = [
    "flybrain-edge",
    "hello-robot",
]

[tool.uv.workspace]
members = ["packages/*", "apps/*", "demos/*"]

[tool.uv.sources]
flybrain-edge = { workspace = true }
hello-robot = { workspace = true }

[tool.ruff]
line-length = 100
"""


@pytest.fixture
def fake_root(tmp_path):
    real = registry.find_root()
    (tmp_path / "pyproject.toml").write_text(ROOT_PYPROJECT)
    shutil.copytree(
        real / "demos" / "hello-robot",
        tmp_path / "demos" / "hello-robot",
        ignore=shutil.ignore_patterns("__pycache__", "*.egg-info"),
    )
    return tmp_path


def test_new_demo_is_a_valid_registered_demo(fake_root):
    dest = scaffold.new_demo(fake_root, "fly-groom", title="Fly grooming")
    assert dest == fake_root / "demos" / "fly-groom"
    pkg = dest / "src" / "fly_groom"
    assert (pkg / "cli.py").is_file() and (dest / "tests" / "test_fly_groom.py").is_file()
    assert not (dest / "src" / "hello_robot").exists()
    text = "".join(p.read_text() for p in dest.rglob("*.py"))
    assert "hello_robot" not in text and "hello-robot" not in text
    pp = tomllib.loads((dest / "pyproject.toml").read_text())
    assert pp["project"]["name"] == "fly-groom"
    assert pp["project"]["entry-points"]["flybrain.demos"] == {
        "fly-groom": "fly_groom.flydeck:SPEC"
    }
    root = tomllib.loads((fake_root / "pyproject.toml").read_text())
    assert "fly-groom" in root["project"]["dependencies"]
    assert root["tool"]["uv"]["sources"]["fly-groom"] == {"workspace": True}
    assert root["tool"]["ruff"]["line-length"] == 100  # the rest is untouched
    # Flydeck sees it even before uv sync, as "not installed"
    cards = {e.id: e for e in registry.discover(fake_root, eps=[])}
    assert "uv sync" in cards["fly-groom"].error
    # its spec loads and validates
    env = dict(
        os.environ, PYTHONPATH=str(dest / "src") + os.pathsep + os.environ.get("PYTHONPATH", "")
    )
    code = (
        "import json; from fly_groom.flydeck import SPEC; SPEC.validate(); "
        "print(json.dumps(SPEC.to_json()))"
    )
    r = run([sys.executable, "-c", code], env=env, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    card = json.loads(r.stdout)
    assert card["id"] == "fly-groom" and card["title"] == "Fly grooming"
    assert card["options"][0]["name"] == "wags"
    # and its own smoke test passes on the simulator
    r = run(
        [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", str(dest / "tests")],
        env=env,
        capture_output=True,
        text=True,
        timeout=180,
        cwd=dest,
    )
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]


@pytest.mark.parametrize(
    ("name", "msg"),
    [
        ("Fly Groom", "lowercase"),
        ("x", "lowercase"),
        ("hello-robot", "already exists"),
        ("json", "already exists"),
    ],
)
def test_bad_names_are_refused(fake_root, name, msg):
    with pytest.raises(scaffold.ScaffoldError, match=msg):
        scaffold.new_demo(fake_root, name)


def test_inline_dependency_list_is_extended(tmp_path):
    p = tmp_path / "pyproject.toml"
    p.write_text('[project]\nname = "x"\ndependencies = ["a", "b"]\n')
    scaffold._add_to_root(p, "fly-groom")
    data = tomllib.loads(p.read_text())
    assert data["project"]["dependencies"] == ["a", "b", "fly-groom"]
    assert data["tool"]["uv"]["sources"]["fly-groom"] == {"workspace": True}
