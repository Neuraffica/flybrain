"""How fly-escape-onboard appears in Flydeck."""

from typing import Any

from flybrain_demo_api import DemoSpec, Option
from flybrain_demo_api.spec import default_argv

ON_BY_DEFAULT = ("enable", "leave_running")  # bool options that default to True


def build_argv(
    spec: DemoSpec,
    transport: str,
    options: dict[str, Any],
    http_port: int,
    robot: str | None,
    space_confirmed: bool,
) -> list[str]:
    """default_argv, except that an unticked default-on option becomes ``--no-<flag>``."""
    argv = default_argv(spec, transport, options, http_port, robot, space_confirmed)
    for name in ON_BY_DEFAULT:
        flag = "--" + name.replace("_", "-")
        if flag in argv:
            argv.remove(flag)
        if not options.get(name, True):
            argv.append("--no-" + flag[2:])
    return argv


SPEC = DemoSpec(
    id="fly-escape-onboard",
    title="Fly escape (on the robot)",
    summary="The escape circuit runs on the robot's own board; the laptop only listens and "
    "shows what the robot's brain sees and decides.",
    module="fly_escape_onboard.cli",
    needs=(
        "firmware with the fly_escape module (firmware/bittle/fly_escape)",
        "IR proximity sensor on the Grove socket labelled 39/36",
        "about 1 m clear behind",
    ),
    moves_robot=True,
    supports_mock=True,
    options=(
        Option(
            "enable",
            "bool",
            True,
            label="Switch the module on if it is off",
            help="the robot stands up, measures the resting level (keep hands away) and from "
            "then on backs away from a hand by itself",
        ),
        Option(
            "leave_running",
            "bool",
            True,
            label="Leave the module running when this closes",
            help="unticked: the module is switched off at Stop. Either way only the live "
            "telemetry is switched off; the laptop never moves the robot itself.",
        ),
    ),
    guide="guide.html",
    legs_move=lambda o: bool(o.get("enable", True)),
    build_argv=build_argv,
    http_port=8768,
)
