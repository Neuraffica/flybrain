"""`flydeck build-app`: the pieces that need no network, and (opt-in) a whole build."""

import os
import platform
import plistlib
import struct
import subprocess
import sys
from pathlib import Path

import pytest
from flydeck import build_app, icon, registry


def test_icon_is_a_valid_icns_with_all_sizes():
    data = icon.icns_bytes()
    assert data[:4] == b"icns" and struct.unpack(">I", data[4:8])[0] == len(data)
    pos, types = 8, []
    while pos < len(data):
        typ, n = data[pos : pos + 4].decode(), struct.unpack(">I", data[pos + 4 : pos + 8])[0]
        assert data[pos + 8 : pos + 16] == b"\x89PNG\r\n\x1a\n"
        types.append(typ)
        pos += n
    assert pos == len(data)
    assert {"icp4", "ic07", "ic10"} <= set(types)


def test_icon_is_the_teal_tile_with_the_cover_fly_on_a_card():
    img = icon.render(128)
    assert img[2, 2, 3] == 0  # outside the macOS tile: transparent
    corner = img[20, 64, :3] * 255  # top of the tile: the Living Signal primary (#0e6f66)
    assert abs(corner - (14, 111, 102)).max() < 12
    card = img[100, 46, :3] * 255  # the front card, clear of the fly: paper (#faf7f1)
    assert abs(card - (250, 247, 241)).max() < 6
    # the fly of the event cover sits on the card: orange-brown body, red eyes
    on_card = img[40:108, 38:106, :3] * 255
    r, g, b = on_card[..., 0], on_card[..., 1], on_card[..., 2]
    assert ((r > 150) & (g > 80) & (g < 170) & (b < 110)).mean() > 0.03  # body
    assert ((r > 150) & (g < 80) & (b < 80)).sum() >= 6  # eyes


def test_icon_fly_cutout_is_read_without_an_image_library():
    fly = icon.read_png(icon.FLY_PNG.read_bytes())
    assert fly.shape[2] == 4 and fly.shape[1] > 300
    assert fly[0, 0, 3] == 0 and fly[..., 3].max() == 1  # clear corner, solid body
    assert icon.read_png(icon.png_bytes(fly)).shape == fly.shape


def test_guides_and_pages_use_the_shared_look():
    root = registry.find_root()
    pages = [root / "apps" / "flydeck" / "guide.html", *root.glob("demos/*/guide.html")]
    pages += list(root.glob("demos/*/src/*/static/index.html"))
    pages.append(root / "apps" / "flydeck" / "src" / "flydeck" / "static" / "index.html")
    assert len(pages) >= 9
    for p in pages:
        text = p.read_text()
        assert 'href="/assets/living-signal.css"' in text, p
        assert 'rel="icon"' in text, p
        assert "fonts.googleapis" not in text and "@import" not in text, p


def test_info_plist():
    info = build_app.info_plist("1.2.3")
    assert info["CFBundleIdentifier"] == "com.neuraffica.flydeck"
    assert info["CFBundleName"] == info["CFBundleDisplayName"] == info["CFBundleExecutable"]
    assert info["CFBundleShortVersionString"] == "1.2.3"
    assert info["NSBluetoothAlwaysUsageDescription"] == build_app.USAGE_TEXT
    assert info["LSMinimumSystemVersion"] == build_app.MIN_MACOS
    assert not info.get("LSUIElement")  # a real Dock app
    plistlib.dumps(info)
    assert build_app.dmg_name("1.2.3") == "Flydeck-1.2.3-macos-arm64.dmg"


def test_app_version_is_the_flydeck_package_version():
    root = registry.find_root()
    assert build_app.app_version(root) == build_app.app_version(root).strip()
    assert build_app.app_version(root)[0].isdigit()


def test_requirements_drop_only_the_data_build_packages():
    export = """
bleak==3.1.0
numpy==2.5.4 ; python_full_version >= '3.12'
pandas==3.1.1
pyarrow==26.0.1
python-dateutil==4.0.0
tzdata==2026.6
pyobjc-core==13.0 ; sys_platform == 'darwin'
# a comment
"""
    assert build_app.requirements(export) == [
        "bleak==3.1.0",
        "numpy==2.5.4 ; python_full_version >= '3.12'",
        "pyobjc-core==13.0 ; sys_platform == 'darwin'",
    ]


@pytest.mark.skipif(sys.platform != "darwin", reason="Mach-O")
def test_min_macos_reads_a_real_binary(tmp_path):
    exe = Path(sys.executable).resolve()
    assert build_app.is_macho(exe)
    v = build_app.min_macos_of(exe)
    assert v is not None and 10 <= v[0] <= 30
    (tmp_path / "x.py").write_text("print()")
    assert not build_app.is_macho(tmp_path / "x.py")


@pytest.mark.skipif(
    os.environ.get("FLYDECK_BUILD_TEST") != "1"
    or sys.platform != "darwin"
    or platform.machine() != "arm64",
    reason="slow, needs network: set FLYDECK_BUILD_TEST=1 on an Apple Silicon Mac",
)
def test_build_app(tmp_path):
    app, dmg = build_app.build(registry.find_root(), tmp_path, dmg=True)
    info = plistlib.loads((app / "Contents" / "Info.plist").read_bytes())
    assert info["CFBundleIdentifier"] == build_app.BUNDLE_ID
    assert (app / "Contents" / "MacOS" / "Flydeck").is_file()
    assert (
        app / "Contents" / "Resources" / "flybrain" / "apps" / "flydeck" / "guide.html"
    ).is_file()
    fonts = list(app.rglob("flybrain_demo_api/assets/fonts/*.woff2"))
    assert len(fonts) == 6, fonts
    assert not list(app.rglob("__pycache__"))
    v = subprocess.run(["codesign", "--verify", "--deep", "--strict", str(app)], check=False)
    assert v.returncode == 0
    assert dmg is not None and dmg.is_file()
