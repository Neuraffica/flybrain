"""`flydeck build-app`: build Flydeck.app, a self-contained Mac app (Apple Silicon).

    dist/Flydeck.app/Contents/
      Info.plist                  bundle id, version, Bluetooth usage text, minimum macOS
      MacOS/Flydeck               a small C launcher (launcher.c): runs `python -m flydeck` in
                                  its own process, so macOS sees one app called Flydeck
      Resources/flydeck.icns      the icon (icon.py)
      Resources/python/           a relocatable CPython (python-build-standalone, as uv installs
                                  it) with Flydeck, the demos and what they need installed
      Resources/flybrain/         the demos' guides, docs/ and the Flydeck guide (the hub's root)
    dist/Flydeck-<version>-macos-arm64.dmg   the app and an Applications link

The app needs no checkout, no uv and no Python on the Mac it runs on. Building needs this
checkout, uv, network access (wheels from PyPI), and the Xcode command line tools (clang,
codesign, hdiutil). It is signed ad hoc only: there is no Developer ID, so it is not notarized
and macOS blocks it the first time it is opened from a download (see the README).
"""

from __future__ import annotations

import os
import platform
import plistlib
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import tomllib
from pathlib import Path

from .icon import icns_bytes

APP_NAME = "Flydeck"
BUNDLE_ID = "com.neuraffica.flydeck"
ICON = "flydeck"
USAGE_TEXT = (
    "Flydeck uses Bluetooth to find and talk to your Petoi robot (Bittle) and to run the "
    "flybrain demos on it."
)
PYTHON = "3.13"  # the bundled CPython
MIN_MACOS = "12.0"  # wheels are chosen for this macOS, and the launcher is built for it
# needed only to build circuit packs from raw connectome downloads (flybrain_data.build imports
# them lazily); no demo uses them at run time, and pyarrow alone is about 100 MB
DATA_BUILD_ONLY = {"pandas", "pyarrow", "python-dateutil", "six", "tzdata"}
STDLIB_DROP = {"idlelib", "tkinter", "turtledemo", "turtle.py", "ensurepip", "test"}
SITE_DROP_DIRS = {"__pycache__", "tests", "test", "PyObjCTest"}
MACHO_MAGIC = {b"\xcf\xfa\xed\xfe", b"\xca\xfe\xba\xbe", b"\xbe\xba\xfe\xca"}


class BuildError(Exception):
    pass


def _run(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    r = subprocess.run(cmd, check=False, capture_output=True, text=True, **kw)
    if r.returncode != 0:
        msg = (r.stderr or r.stdout).strip().splitlines()[-12:]
        raise BuildError(f"{' '.join(cmd[:4])} ... failed:\n" + "\n".join(msg))
    return r


def _uv() -> str:
    uv = os.environ.get("UV") or shutil.which("uv")
    if not uv:
        raise BuildError("needs uv (https://docs.astral.sh/uv/)")
    return uv


def app_version(root: Path) -> str:
    data = tomllib.loads((root / "apps" / "flydeck" / "pyproject.toml").read_text())
    return data["project"]["version"]


def dmg_name(version: str) -> str:
    return f"{APP_NAME}-{version}-macos-arm64.dmg"


def info_plist(version: str, min_macos: str = MIN_MACOS) -> dict:
    return {
        "CFBundleDevelopmentRegion": "en",
        "CFBundleDisplayName": APP_NAME,
        "CFBundleExecutable": APP_NAME,
        "CFBundleIconFile": ICON,
        "CFBundleIdentifier": BUNDLE_ID,
        "CFBundleInfoDictionaryVersion": "6.0",
        "CFBundleName": APP_NAME,
        "CFBundlePackageType": "APPL",
        "CFBundleShortVersionString": version,
        "CFBundleVersion": version,
        "LSApplicationCategoryType": "public.app-category.education",
        "LSArchitecturePriority": ["arm64"],
        "LSMinimumSystemVersion": min_macos,
        "NSAppTransportSecurity": {"NSAllowsLocalNetworking": True},
        "NSBluetoothAlwaysUsageDescription": USAGE_TEXT,
        "NSHighResolutionCapable": True,
        "NSHumanReadableCopyright": "Copyright (c) 2026 Neuraffica, Inc. PolyForm Noncommercial 1.0.0",
        "NSPrincipalClass": "NSApplication",
        "NSSupportsAutomaticGraphicsSwitching": True,
    }


def requirements(export: str) -> list[str]:
    """`uv export` lines (pinned, with markers) minus what only the data build needs."""
    keep = []
    for line in export.splitlines():
        line = line.strip()
        if not line or line.startswith(("#", "-")):
            continue
        name = re.split(r"[=<>!~ ;\[]", line, maxsplit=1)[0].lower().replace("_", "-")
        if name not in DATA_BUILD_ONLY:
            keep.append(line)
    return keep


# ---- Mach-O --------------------------------------------------------------------------
def is_macho(path: Path) -> bool:
    if path.is_symlink() or not path.is_file():
        return False
    with open(path, "rb") as f:
        return f.read(4) in MACHO_MAGIC


def macho_files(top: Path) -> list[Path]:
    return [p for p in sorted(top.rglob("*")) if is_macho(p)]


def _version(v: int) -> tuple[int, int, int]:
    return (v >> 16, (v >> 8) & 0xFF, v & 0xFF)


def min_macos_of(path: Path) -> tuple[int, int, int] | None:
    """The minimum macOS a Mach-O file (its arm64 slice) was built for."""
    data = path.read_bytes()
    off = 0
    if data[:4] in (b"\xca\xfe\xba\xbe", b"\xbe\xba\xfe\xca"):  # fat: find the arm64 slice
        (n,) = struct.unpack(">I", data[4:8])
        for i in range(n):
            cputype, _sub, offset, _size, _align = struct.unpack(
                ">iiIII", data[8 + 20 * i : 28 + 20 * i]
            )
            if cputype == 0x0100000C:
                off = offset
                break
        else:
            return None
    if data[off : off + 4] != b"\xcf\xfa\xed\xfe":
        return None
    ncmds = struct.unpack("<I", data[off + 16 : off + 20])[0]
    pos = off + 32
    for _ in range(ncmds):
        cmd, size = struct.unpack("<II", data[pos : pos + 8])
        if cmd == 0x32:  # LC_BUILD_VERSION: platform, minos, sdk
            plat, minos = struct.unpack("<II", data[pos + 8 : pos + 16])
            if plat == 1:  # macOS
                return _version(minos)
        elif cmd == 0x24:  # LC_VERSION_MIN_MACOSX: version, sdk
            return _version(struct.unpack("<I", data[pos + 8 : pos + 12])[0])
        pos += size
    return None


def thin_to_arm64(path: Path) -> None:
    archs = _run(["lipo", "-archs", str(path)]).stdout.split()
    if len(archs) > 1 and "arm64" in archs:
        _run(["lipo", str(path), "-thin", "arm64", "-output", str(path)])


# ---- steps ---------------------------------------------------------------------------
def find_python(uv: str) -> Path:
    """The prefix of uv's managed CPython PYTHON (installed if missing)."""
    _run([uv, "python", "install", PYTHON])
    exe = _run([uv, "python", "find", "--managed-python", "--no-project", PYTHON]).stdout
    prefix = Path(exe.strip()).resolve().parent.parent
    if not (prefix / "lib" / f"libpython{PYTHON}.dylib").is_file():
        raise BuildError(f"{prefix} has no lib/libpython{PYTHON}.dylib")
    return prefix


def copy_python(prefix: Path, dest: Path) -> None:
    stdlib = f"lib/python{PYTHON}"
    keep_bin = {"python3", f"python{PYTHON}", "python"}

    def ignore(d: str, names: list[str]) -> set[str]:
        rel = Path(d).relative_to(prefix).as_posix()
        drop = set()
        for n in names:
            if (
                (rel == "." and n in ("include", "share"))
                or (rel == "bin" and n not in keep_bin)
                or (rel == "lib" and re.match(r"(pkgconfig|tcl|tk|itcl|thread|libtcl|libtk)", n))
                or (rel == stdlib and (n in STDLIB_DROP or n.startswith("config-")))
                or (rel == stdlib and n == "EXTERNALLY-MANAGED")
                or (rel == f"{stdlib}/site-packages" and n != "README.txt")
                or (rel == f"{stdlib}/lib-dynload" and re.match(r"_(tkinter|test|ctypes_test)", n))
                or n == "__pycache__"
            ):
                drop.add(n)
        return drop

    shutil.copytree(prefix, dest, symlinks=True, ignore=ignore)
    for f in (dest / stdlib).glob("_sysconfig*"):  # uv wrote the build machine's path here
        f.write_text(f.read_text().replace(str(prefix), "/install"))
    lib = dest / "lib" / f"libpython{PYTHON}.dylib"
    lib.chmod(0o755)
    _run(["install_name_tool", "-id", f"@rpath/libpython{PYTHON}.dylib", str(lib)])


def install_packages(uv: str, root: Path, python: Path, work: Path) -> None:
    export = _run(
        [
            uv,
            "export",
            "--frozen",
            "--no-dev",
            "--no-hashes",
            "--no-emit-workspace",
            "--all-packages",
            "--no-header",
            "--no-annotate",
        ],
        cwd=root,
    ).stdout
    reqs = work / "requirements.txt"
    reqs.write_text("\n".join(requirements(export)) + "\n")
    wheels = work / "wheels"
    _run([uv, "build", "--all-packages", "--wheel", "--out-dir", str(wheels)], cwd=root)
    env = dict(os.environ, MACOSX_DEPLOYMENT_TARGET=MIN_MACOS)
    env.pop("VIRTUAL_ENV", None)
    _run(
        [
            uv,
            "pip",
            "install",
            "--python",
            str(python),
            "--python-platform",
            "aarch64-apple-darwin",
            "--no-deps",  # everything is pinned in requirements.txt, from uv.lock
            "--link-mode",
            "copy",
            "-r",
            str(reqs),
            *sorted(str(w) for w in wheels.glob("*.whl")),
        ],
        cwd=work,
        env=env,
    )


def prune(python_home: Path) -> None:
    site = python_home / "lib" / f"python{PYTHON}" / "site-packages"
    for p in sorted(site.rglob("*"), reverse=True):
        if p.is_dir() and not p.is_symlink() and p.name in SITE_DROP_DIRS:
            shutil.rmtree(p)
    for p in list(site.rglob("*.pyi")) + list(site.rglob("direct_url.json")):
        p.unlink()
    for p in sorted(python_home.rglob("__pycache__"), reverse=True):
        shutil.rmtree(p, ignore_errors=True)
    for p in (python_home / "bin").iterdir():  # console scripts point at the build machine
        if not p.name.startswith("python"):
            p.unlink()


def copy_root(root: Path, dest: Path) -> None:
    """The hub's root inside the app: each demo's pyproject.toml and guides, docs, the guide."""
    for pp in sorted((root / "demos").glob("*/pyproject.toml")):
        d = dest / "demos" / pp.parent.name
        d.mkdir(parents=True)
        for f in pp.parent.iterdir():
            if f.is_file() and f.suffix in (".toml", ".md", ".html", ".htm"):
                shutil.copy2(f, d / f.name)
    (dest / "docs").mkdir()
    for f in sorted((root / "docs").glob("*.md")):
        shutil.copy2(f, dest / "docs" / f.name)
    (dest / "apps" / "flydeck").mkdir(parents=True)
    shutil.copy2(root / "apps" / "flydeck" / "guide.html", dest / "apps" / "flydeck")
    shutil.copy2(root / "LICENSE.md", dest / "LICENSE.md")


def compile_launcher(prefix: Path, python_home: Path, exe: Path) -> None:
    src = Path(__file__).with_name("launcher.c")
    _run(
        [
            "clang",
            "-arch",
            "arm64",
            f"-mmacosx-version-min={MIN_MACOS}",
            "-O2",
            "-Wall",
            "-I",
            str(prefix / "include" / f"python{PYTHON}"),
            str(src),
            "-o",
            str(exe),
            "-L",
            str(python_home / "lib"),
            f"-lpython{PYTHON}",
            "-Wl,-rpath,@executable_path/../Resources/python/lib",
        ]
    )


def smoke_test(app: Path) -> None:
    """Run the app's own launcher: every demo must load from the bundle alone."""
    env = {
        "HOME": os.environ.get("HOME", "/tmp"),
        "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
        "FLYDECK_HOME": tempfile.mkdtemp(prefix="flydeck-smoke-"),
    }
    exe = str(app / "Contents" / "MacOS" / APP_NAME)
    r = subprocess.run([exe, "list"], capture_output=True, text=True, env=env, cwd="/", check=False)
    if r.returncode != 0:
        raise BuildError(f"the built app cannot load its demos:\n{r.stdout}{r.stderr}")
    py = str(app / "Contents" / "Resources" / "python" / "bin" / "python3")
    check = (
        "import bleak, serial, numpy, webview, flydeck.app, flydeck.robotcheck;"
        "import fly_compass.cli, fly_escape.cli, fly_escape_onboard.cli, hello_robot.cli;"
        "from flybrain_demo_api.web import bittle_asset;"
        "assert all(bittle_asset('/bittle/' + name)[0] for name in "
        "('robot.js', 'top.webp', 'head.webp', 'side.webp', 'NOTICE', 'manifest.json'))"
    )
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    r = subprocess.run(
        [py, "-I", "-c", check], capture_output=True, text=True, env=env, cwd="/", check=False
    )
    if r.returncode != 0:
        raise BuildError(f"the built app is missing a package:\n{r.stderr}")


def sign(app: Path) -> None:
    """Ad hoc: every Mach-O inside first, then the bundle (no Developer ID here)."""
    files = [str(p) for p in macho_files(app / "Contents" / "Resources")]
    for i in range(0, len(files), 200):
        _run(["codesign", "--force", "--sign", "-", *files[i : i + 200]])
    _run(["codesign", "--force", "--sign", "-", str(app)])
    _run(["codesign", "--verify", "--deep", "--strict", str(app)])


def make_dmg(app: Path, dmg: Path, work: Path) -> None:
    stage = work / "dmg"
    stage.mkdir()
    _run(["ditto", str(app), str(stage / app.name)])
    (stage / "Applications").symlink_to("/Applications")
    dmg.unlink(missing_ok=True)
    _run(
        [
            "hdiutil",
            "create",
            "-volname",
            APP_NAME,
            "-srcfolder",
            str(stage),
            "-fs",
            "HFS+",
            "-format",
            "UDZO",
            "-ov",
            str(dmg),
        ]
    )


def check_host() -> None:
    if sys.platform != "darwin" or platform.machine() != "arm64":
        raise BuildError("Flydeck.app is built on an Apple Silicon Mac (macOS, arm64)")
    for tool in ("clang", "codesign", "hdiutil", "install_name_tool", "lipo", "ditto"):
        if shutil.which(tool) is None:
            raise BuildError(f"needs {tool}: install the Xcode command line tools")


def build(root: Path, out: Path | None = None, dmg: bool = True) -> tuple[Path, Path | None]:
    """Build out/Flydeck.app (and the .dmg); return their paths."""
    check_host()
    uv = _uv()
    out = out or root / "dist"
    version = app_version(root)
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="flydeck-build-") as tmp:
        work = Path(tmp)
        app = work / f"{APP_NAME}.app"
        contents = app / "Contents"
        res = contents / "Resources"
        (contents / "MacOS").mkdir(parents=True)
        res.mkdir()
        print("flydeck: copying Python", flush=True)
        prefix = find_python(uv)
        home = res / "python"
        copy_python(prefix, home)
        print("flydeck: installing Flydeck, the demos and their packages", flush=True)
        install_packages(uv, root, home / "bin" / "python3", work)
        prune(home)
        copy_root(root, res / "flybrain")
        (res / f"{ICON}.icns").write_bytes(icns_bytes())
        compile_launcher(prefix, home, contents / "MacOS" / APP_NAME)
        machos = macho_files(app)
        for p in machos:
            thin_to_arm64(p)
        found = [v for v in (min_macos_of(p) for p in machos) if v]
        need = max(found, default=(11, 0, 0))
        floor = tuple(int(x) for x in MIN_MACOS.split(".")) + (0,)
        if need > floor:
            raise BuildError(
                f"a binary needs macOS {'.'.join(map(str, need))}, newer than {MIN_MACOS}"
            )
        with open(contents / "Info.plist", "wb") as f:
            plistlib.dump(info_plist(version), f)
        print("flydeck: checking the app runs on its own", flush=True)
        smoke_test(app)
        prune(home)  # nothing the smoke test may have left
        print("flydeck: signing (ad hoc)", flush=True)
        sign(app)
        final = out / app.name
        if final.exists():
            shutil.rmtree(final)
        _run(["ditto", str(app), str(final)])
        dmg_path = None
        if dmg:
            print("flydeck: making the disk image", flush=True)
            dmg_path = out / dmg_name(version)
            make_dmg(final, dmg_path, work)
    return final, dmg_path


def install(root: Path, directory: Path | None = None) -> Path:
    """Build the app and copy it to ``directory`` (default ~/Applications)."""
    app, _ = build(root, dmg=False)
    target = (directory or Path.home() / "Applications") / app.name
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        shutil.rmtree(target)
    _run(["ditto", str(app), str(target)])
    lsregister = Path(
        "/System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework"
        "/Support/lsregister"
    )
    if lsregister.exists():  # refresh the Dock/Finder icon cache; harmless if it fails
        subprocess.run([str(lsregister), "-f", str(target)], check=False, capture_output=True)
    return target
