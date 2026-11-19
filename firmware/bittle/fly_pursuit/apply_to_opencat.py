"""Apply BOTH flybrain modules to an OpenCatEsp32 sketch folder: fly_escape (E) and fly_pursuit
(P), so that one build has both.

    python3 apply_to_opencat.py path/to/OpenCatEsp32            # all stock features kept
    python3 apply_to_opencat.py path/to/OpenCatEsp32 --keep-all # the same (default)
    python3 apply_to_opencat.py path/to/OpenCatEsp32 --slim     # fly_escape's slim switches too

Edits the folder in place (work on a copy of the checkout). Idempotent: running it again on an
already patched folder only refreshes the copied headers. fly_escape is applied with its own
``apply_to_opencat.py`` (unchanged), only if it is not there yet. fly_pursuit adds:

* ``src/flyPursuit.h``, ``src/flyPursuitCore.h``, ``src/fly_pursuit_all.h`` (copied);
* ``EXTENSION_FLY_PURSUIT 'P'`` and one entry in ``moduleList``, ``moduleNames`` and every
  ``moduleActivatedQ`` list in ``src/OpenCat.h``; ``flyPursuitBootBusCheck()`` just before the
  ``Wire.begin()`` in ``initRobot()`` (same file);
* in ``src/moduleManager.h``: the include, the ``initModule`` / ``stopModule`` cases, the
  ``@FP on/off`` line in ``showModuleStatus()``, and ``read_flyPursuit()`` on every pass of
  ``readSignal()`` (next to the voice module's poll, so camera lines are printed while the robot
  is busy too);
* in ``src/reaction.h``: the module's options in the ``X`` command, for ``XP...`` and (a second
  case label, lowercase P) ``Xp...``, so that ``XPs~`` and ``Xps~`` work with the module off;
  a folder patched before that label existed gets it on the next run;
* ``#define FLY_PURSUIT`` in ``OpenCatEsp32.ino``.

Written against OpenCatEsp32 ``f5aba6b``; the anchors are the same ones fly_escape's script
uses on ``32a1fcb`` and ``c481e47`` (README, "Build").
"""

from __future__ import annotations

import argparse
import importlib.util
import re
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
ESCAPE = HERE.parent / "fly_escape" / "apply_to_opencat.py"
FILES = ("flyPursuit.h", "flyPursuitCore.h", "fly_pursuit_all.h")
MARK = "EXTENSION_FLY_PURSUIT"
LOWER = "case EXTENSION_FLY_PURSUIT + ('a' - 'A')"  # the Xp... case label


def _escape_module():
    spec = importlib.util.spec_from_file_location("fly_escape_apply", ESCAPE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _sub1(pattern: str, repl, text: str, what: str, flags: int = 0) -> str:
    out, k = re.subn(pattern, repl, text, count=1, flags=flags)
    if k != 1:
        raise SystemExit(f"{what} did not match the expected layout")
    return out


def apply_pursuit(sketch: Path) -> bool:
    """Add fly_pursuit; returns False if it was already there (headers refreshed only)."""
    src = sketch / "src"
    for f in FILES:
        shutil.copy(HERE / "src" / f, src / f)
    oc = (src / "OpenCat.h").read_text()
    if MARK in oc:
        _lowercase_case(src)  # a folder patched before the Xp... options existed
        return False

    oc = _sub1(
        r"(#define EXTENSION_FLY_ESCAPE 'E'[^\n]*\n)",
        r"\1#define EXTENSION_FLY_PURSUIT 'P'  // fly pursuit, Petoi AI Vision camera on Grove I2C\n",
        oc,
        "OpenCat.h: EXTENSION_FLY_ESCAPE define",
    )
    oc = _sub1(
        r"(\n(\s*)EXTENSION_FLY_ESCAPE,\n)(\s*\};)",
        r"\1\2EXTENSION_FLY_PURSUIT,\n\3",
        oc,
        "OpenCat.h: moduleList",
    )
    oc = _sub1(r'"Fly_Escape"\s*\}', '"Fly_Escape", "Fly_Pursuit"}', oc, "OpenCat.h: moduleNames")
    oc, k = re.subn(r"(bool moduleActivatedQ\[\] = \{[0-9, ]*?)\s*\};", r"\1, 0};", oc)
    if k < 1:
        raise SystemExit("OpenCat.h: moduleActivatedQ did not match the expected layout")
    oc = _sub1(
        r"(void initRobot\(\) \{.*?\n)(  Wire\.begin\(\);\n)",
        r"\1#ifdef FLY_PURSUIT\n  flyPursuitBootBusCheck();  // camera left holding SDA low\n"
        r"#endif\n\2",
        oc,
        "OpenCat.h: Wire.begin() in initRobot()",
        flags=re.DOTALL,
    )
    (src / "OpenCat.h").write_text(oc)

    ino = (sketch / "OpenCatEsp32.ino").read_text()
    ino = _sub1(
        r'(#include "src/OpenCat.h")',
        r"#define FLY_PURSUIT               // fly pursuit with the AI Vision camera (flybrain)\n\1",
        ino,
        "OpenCatEsp32.ino: include",
    )
    (sketch / "OpenCatEsp32.ino").write_text(ino)

    mm = (src / "moduleManager.h").read_text()
    inc = '#ifdef FLY_ESCAPE\n#include "flyEscape.h"\n#endif'
    if inc not in mm:
        raise SystemExit("moduleManager.h: fly_escape include not found")
    mm = mm.replace(inc, inc + '\n\n#ifdef FLY_PURSUIT\n#include "flyPursuit.h"\n#endif', 1)
    init_case = (
        "#ifdef FLY_PURSUIT\n    case EXTENSION_FLY_PURSUIT:\n      {\n"
        "        flyPursuitSetup();  // passive: the robot does not move\n        break;\n"
        "      }\n#endif\n"
    )
    stop_case = (
        "#ifdef FLY_PURSUIT\n    case EXTENSION_FLY_PURSUIT:\n      {\n"
        "        flyPursuitStop();\n        break;\n      }\n#endif\n"
    )
    case_re = re.compile(r"#ifdef FLY_ESCAPE\n\s*case EXTENSION_FLY_ESCAPE:")
    j = case_re.search(mm, mm.index("void initModule(")).start()
    mm = mm[:j] + init_case + mm[j:]
    j = case_re.search(mm, mm.index("void stopModule(")).start()
    mm = mm[:j] + stop_case + mm[j:]
    mm = _sub1(
        r"(  flyEscapeStatus\(moduleActivatedQ\[indexOfModule\(EXTENSION_FLY_ESCAPE\)\]\);\n#endif\n)",
        r"\1#ifdef FLY_PURSUIT\n  flyPursuitStatus(moduleActivatedQ[indexOfModule(EXTENSION_FLY_PURSUIT)]);\n"
        r"#endif\n",
        mm,
        "moduleManager.h: showModuleStatus()",
    )
    mm = _sub1(
        r"(#ifdef VOICE\n\s*if \(moduleActivatedQ\[indexOfModule\(EXTENSION_VOICE\)\]\)\n"
        r"\s*read_voice\(\);\n#endif\n)",
        r"\1#ifdef FLY_PURSUIT\n  read_flyPursuit();  // every pass: camera results and errors\n"
        r"#endif\n",
        mm,
        "moduleManager.h: read_voice() in readSignal()",
    )
    (src / "moduleManager.h").write_text(mm)

    rh = (src / "reaction.h").read_text()
    option_case = (
        "#ifdef FLY_PURSUIT\n            case EXTENSION_FLY_PURSUIT:\n              {\n"
        "                flyPursuitOptions(newCmd, "
        "moduleActivatedQ[indexOfModule(EXTENSION_FLY_PURSUIT)]);\n"
        "                break;\n              }\n#endif\n"
    )
    rh = _sub1(
        r"(// deal with the following command\n\s*switch \(newCmd\[0\]\) \{\n)",
        lambda m: m.group(1) + option_case,
        rh,
        "reaction.h: the X command switch",
    )
    (src / "reaction.h").write_text(rh)
    _lowercase_case(src)
    return True


def _lowercase_case(src: Path) -> None:
    """Let ``Xp...`` reach ``flyPursuitOptions`` too (for ``XPs~``/``Xps~``): a second case label,
    lowercase P, on the module's case in reaction.h's X command switch. Idempotent."""
    rh = (src / "reaction.h").read_text()
    if LOWER in rh:
        return
    rh = _sub1(
        r"(\n(\s*)case EXTENSION_FLY_PURSUIT:\n)",
        lambda m: m.group(1) + m.group(2) + LOWER + ":  // Xp...: XPs / Xps\n",
        rh,
        "reaction.h: the fly_pursuit case in the X command switch",
    )
    (src / "reaction.h").write_text(rh)


def apply(sketch: Path, slim: bool = False) -> None:
    sketch = Path(sketch)
    oc = (sketch / "src" / "OpenCat.h").read_text()
    esc = _escape_module()
    if "EXTENSION_FLY_ESCAPE" not in oc:
        esc.apply(sketch, slim=slim)
    else:
        for f in ("flyEscape.h", "fly_escape_all.h"):  # refresh the copies only
            shutil.copy(ESCAPE.parent / "src" / f, sketch / "src" / f)
        print("fly_escape: already applied; headers refreshed")
    if apply_pursuit(sketch):
        print(f"applied fly_pursuit to {sketch}")
    else:
        print("fly_pursuit: already applied; headers refreshed")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("sketch", type=Path, help="the OpenCatEsp32 folder (with OpenCatEsp32.ino)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--keep-all", action="store_true", help="keep every stock feature (the default)")
    g.add_argument(
        "--slim",
        action="store_true",
        help="also apply fly_escape's slim switches (drops the stock camera, quick demo, "
        "micro:bit client; only for the 1.3 MB default partition layout)",
    )
    args = ap.parse_args()
    apply(args.sketch, slim=args.slim)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
