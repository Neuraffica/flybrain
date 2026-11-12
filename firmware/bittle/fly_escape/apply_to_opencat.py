"""Apply the fly_escape module to an OpenCatEsp32 sketch folder, as README.md describes.

    python3 apply_to_opencat.py path/to/OpenCatEsp32            # module + slim switches
    python3 apply_to_opencat.py path/to/OpenCatEsp32 --keep-all  # module only (stock switches)

Edits the copy in place (work on a copy of the checkout):

* copies ``src/flyEscape.h`` and ``src/fly_escape_all.h`` into ``<sketch>/src/``;
* adds ``EXTENSION_FLY_ESCAPE 'E'`` and one entry to ``moduleList``, ``moduleNames`` and every
  ``moduleActivatedQ`` list in ``src/OpenCat.h``;
* adds the include, the ``initModule``/``stopModule`` cases and the poll to
  ``src/moduleManager.h``, and ``#define FLY_ESCAPE`` to ``OpenCatEsp32.ino``;
* unless ``--keep-all``: comments out the slim-build switches (README, "Slim build").

Checked against OpenCatEsp32 ``c481e47`` (the robot's B10_250330) and ``f5aba6b`` (main).
"""

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
SLIM_INO = ("CAMERA", "QUICK_DEMO")  # in OpenCatEsp32.ino
SLIM_OPENCAT = ("BT_CLIENT",)  # in src/OpenCat.h


def disable(text: str, macro: str) -> tuple[str, int]:
    return re.subn(rf"(?m)^#define {macro}(?=\s|$)", f"// #define {macro}", text)


def apply(sketch: Path, slim: bool = True) -> None:
    src = sketch / "src"
    for f in ("flyEscape.h", "fly_escape_all.h"):
        shutil.copy(HERE / "src" / f, src / f)

    oc = (src / "OpenCat.h").read_text()
    oc, n = re.subn(
        r"(#define EXTENSION_QUICK_DEMO 'Q'[^\n]*\n)",
        r"\1#define EXTENSION_FLY_ESCAPE 'E'  // fly looming escape, IR sensor on ANALOG3, ANALOG4\n",
        oc,
        count=1,
    )
    oc, n0 = re.subn(
        r"(\n(\s*)EXTENSION_QUICK_DEMO,\n)(\s*\};)", r"\1\2EXTENSION_FLY_ESCAPE,\n\3", oc, count=1
    )
    oc, n1 = re.subn(r'"Quick_Demo"\s*\}', '"Quick_Demo", "Fly_Escape"}', oc, count=1)
    oc, n2 = re.subn(r"(bool moduleActivatedQ\[\] = \{[0-9, ]*?)\s*\};", r"\1, 0};", oc)
    if not (n == n0 == n1 == 1 and n2 >= 1):
        raise SystemExit(f"OpenCat.h did not match the expected layout ({n}, {n0}, {n1}, {n2})")
    if slim:
        for m in SLIM_OPENCAT:
            oc, k = disable(oc, m)
            print(f"OpenCat.h: disabled {m} ({k} line(s))")
    (src / "OpenCat.h").write_text(oc)

    ino = (sketch / "OpenCatEsp32.ino").read_text()
    ino = ino.replace(
        '#include "src/OpenCat.h"',
        '#define FLY_ESCAPE                // fly looming escape (flybrain)\n#include "src/OpenCat.h"',
        1,
    )
    if slim:
        for m in SLIM_INO:
            ino, k = disable(ino, m)
            print(f"OpenCatEsp32.ino: disabled {m} ({k} line(s))")
    (sketch / "OpenCatEsp32.ino").write_text(ino)

    mm = (src / "moduleManager.h").read_text()
    inc = '#ifdef DOUBLE_TOUCH\n#include "doubleTouch.h"\n#endif'
    if inc not in mm:
        raise SystemExit("moduleManager.h did not match the expected layout")
    mm = mm.replace(inc, inc + '\n\n#ifdef FLY_ESCAPE\n#include "flyEscape.h"\n#endif', 1)
    init_case = (
        "#ifdef FLY_ESCAPE\n    case EXTENSION_FLY_ESCAPE:\n      {\n"
        '        loadBySkillName("up");\n        flyEscapeSetup();\n        break;\n      }\n'
        "#endif\n"
    )
    stop_case = (
        "#ifdef FLY_ESCAPE\n    case EXTENSION_FLY_ESCAPE:\n      {\n"
        "        flyEscapeStop();\n        break;\n      }\n#endif\n"
    )
    case_re = re.compile(r"#ifdef DOUBLE_TOUCH\n\s*case EXTENSION_DOUBLE_TOUCH:")
    j = case_re.search(mm, mm.index("void initModule(")).start()
    mm = mm[:j] + init_case + mm[j:]
    j = case_re.search(mm, mm.index("void stopModule(")).start()
    mm = mm[:j] + stop_case + mm[j:]
    poll = (
        "#ifdef FLY_ESCAPE\n    if (moduleActivatedQ[indexOfModule(EXTENSION_FLY_ESCAPE)])\n"
        "      read_flyEscape();\n#endif\n"
    )
    j = re.search(
        r"#ifdef DOUBLE_TOUCH\n\s*if \(moduleActivatedQ\[indexOfModule\(EXTENSION_DOUBLE_TOUCH\)\]\)",
        mm,
    ).start()
    mm = mm[:j] + poll + mm[j:]
    (src / "moduleManager.h").write_text(mm)
    print(f"applied fly_escape to {sketch}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("sketch", type=Path, help="the OpenCatEsp32 folder (with OpenCatEsp32.ino)")
    ap.add_argument("--keep-all", action="store_true", help="do not disable any stock feature")
    args = ap.parse_args()
    apply(args.sketch, slim=not args.keep_all)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
