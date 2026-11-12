"""DemoSpec: what Flydeck needs to know to show a demo and start it."""

from __future__ import annotations

import re
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Literal

TRANSPORTS = ("mock", "usb", "ble")  # simulator, USB serial, Bluetooth LE
OptionType = Literal["choice", "bool", "int", "float", "str"]

# Exit codes a demo should use, so Flydeck can say what went wrong in plain words.
EXIT_CODES = {
    0: "finished",
    1: "stopped by an error",
    2: "bad options",
    3: "a model or data file is missing (run uv sync)",
    4: "could not connect to the robot",
    5: "its dashboard port was in use",
    130: "stopped (Ctrl-C)",
}

_ID_RE = re.compile(r"^[a-z][a-z0-9-]{1,40}$")


class SpecError(ValueError):
    """A DemoSpec that Flydeck cannot use; the message says why."""


@dataclass(frozen=True)
class Option:
    """One setting shown in the demo's form and turned into a command-line flag.

    ``flag`` defaults to ``--<name with dashes>``. A bool option adds the bare flag when true.
    ``mock_only`` options are only offered (and passed) when running in the simulator."""

    name: str
    type: OptionType
    default: Any
    label: str = ""
    help: str = ""
    choices: tuple[Any, ...] = ()
    flag: str = ""
    minimum: float | None = None
    maximum: float | None = None
    mock_only: bool = False

    @property
    def cli_flag(self) -> str:
        return self.flag or "--" + self.name.replace("_", "-")

    def coerce(self, value: Any) -> Any:
        """Convert a value from the form (often a string) to this option's type, or raise."""
        if value is None or value == "":
            return self.default
        try:
            if self.type == "bool":
                if isinstance(value, str):
                    return value.strip().lower() in ("1", "true", "yes", "on")
                return bool(value)
            if self.type == "int":
                v: Any = int(value)
            elif self.type == "float":
                v = float(value)
            else:
                v = str(value)
        except (TypeError, ValueError) as e:
            raise SpecError(f"{self.label or self.name}: {value!r} is not a {self.type}") from e
        if self.type == "choice" and v not in [str(c) for c in self.choices]:
            raise SpecError(
                f"{self.label or self.name}: choose one of {', '.join(map(str, self.choices))}"
            )
        if self.type in ("int", "float"):
            if self.minimum is not None and v < self.minimum:
                raise SpecError(f"{self.label or self.name}: at least {self.minimum}")
            if self.maximum is not None and v > self.maximum:
                raise SpecError(f"{self.label or self.name}: at most {self.maximum}")
        return v

    def to_json(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "type": self.type,
            "default": self.default,
            "label": self.label or self.name.replace("_", " ").capitalize(),
            "help": self.help,
            "choices": list(self.choices),
            "min": self.minimum,
            "max": self.maximum,
            "mock_only": self.mock_only,
        }


ArgvBuilder = Callable[["DemoSpec", str, dict[str, Any], int, str | None, bool], list[str]]


@dataclass(frozen=True)
class DemoSpec:
    """Everything Flydeck needs to show a demo card and start the demo.

    id            short name, lowercase, e.g. "fly-escape"
    title         card title; summary: one line
    module        what Flydeck runs: ``python -m <module>`` with the demo's arguments
    needs         hardware it needs, in words, e.g. ("IR proximity sensor on socket 39/36",)
    moves_robot   True if it moves any joint; leg_motion tells when the legs move (see below)
    supports_mock True if ``--mock`` runs it on the simulator, with no robot
    transports    which links it accepts, from TRANSPORTS
    options       the settings form
    guide         path to its guide (HTML or Markdown), relative to the demo's folder
    space_flag    the flag that allows leg motion, e.g. "--i-have-space"
    legs_move     function(options) -> bool: does this run move the legs? Flydeck then asks the
                  user to confirm the robot has space, every start
    build_argv    optional custom argv builder; the default (default_argv) maps transport,
                  options, dashboard port and the space confirmation to flags
    ready         "status": wait for status.ready(); "http": wait until the dashboard answers
    http_port     default dashboard port, or 0 to let Flydeck pick a free one
    ready_timeout_s  how long Flydeck waits for "ready" before calling the start failed
    """

    id: str
    title: str
    summary: str
    module: str
    needs: tuple[str, ...] = ()
    moves_robot: bool = False
    supports_mock: bool = True
    transports: tuple[str, ...] = TRANSPORTS
    options: tuple[Option, ...] = ()
    guide: str = ""
    space_flag: str = "--i-have-space"
    legs_move: Callable[[dict[str, Any]], bool] | None = None
    build_argv: ArgvBuilder | None = None
    ready: Literal["status", "http"] = "status"
    http_port: int = 0
    ready_timeout_s: float = 60.0
    extra: dict[str, Any] = field(default_factory=dict)

    # ---- checks ---------------------------------------------------------------------
    def validate(self) -> None:
        """Raise SpecError with a plain message if Flydeck cannot use this spec."""
        if not _ID_RE.match(self.id or ""):
            raise SpecError(f"id {self.id!r}: use 2-41 lowercase letters, digits or dashes")
        if not self.title or not self.summary:
            raise SpecError(f"{self.id}: title and summary are required")
        if not self.module or not re.match(r"^[A-Za-z_][\w.]*$", self.module):
            raise SpecError(f"{self.id}: module {self.module!r} is not a Python module name")
        bad = [t for t in self.transports if t not in TRANSPORTS]
        if bad or not self.transports:
            raise SpecError(
                f"{self.id}: transports must be from {TRANSPORTS}, got {self.transports}"
            )
        if self.supports_mock and "mock" not in self.transports:
            raise SpecError(f"{self.id}: supports_mock needs 'mock' in transports")
        names = [o.name for o in self.options]
        if len(names) != len(set(names)):
            raise SpecError(f"{self.id}: option names repeat")
        for o in self.options:
            if o.type not in ("choice", "bool", "int", "float", "str"):
                raise SpecError(f"{self.id}: option {o.name}: unknown type {o.type!r}")
            if o.type == "choice" and o.default not in o.choices:
                raise SpecError(
                    f"{self.id}: option {o.name}: default {o.default!r} is not a choice"
                )
        if self.ready not in ("status", "http"):
            raise SpecError(f"{self.id}: ready must be 'status' or 'http'")
        if not 0 <= self.http_port < 65536:
            raise SpecError(f"{self.id}: http_port out of range")

    # ---- use ------------------------------------------------------------------------
    def clean_options(self, values: dict[str, Any], transport: str) -> dict[str, Any]:
        """Defaults filled in, types checked; mock-only options dropped off the simulator."""
        out = {}
        for o in self.options:
            if o.mock_only and transport != "mock":
                continue
            out[o.name] = o.coerce(values.get(o.name, o.default))
        unknown = set(values) - {o.name for o in self.options}
        if unknown:
            raise SpecError(f"unknown option(s): {', '.join(sorted(unknown))}")
        return out

    def needs_space(self, options: dict[str, Any], transport: str) -> bool:
        """Does this run move the legs of a real robot (so the user must confirm space)?"""
        if transport == "mock" or not self.moves_robot:
            return False
        return True if self.legs_move is None else bool(self.legs_move(options))

    def argv(
        self,
        transport: str,
        options: dict[str, Any],
        http_port: int,
        robot: str | None = None,
        space_confirmed: bool = False,
    ) -> list[str]:
        """Arguments after ``python -m <module>``."""
        if transport not in self.transports:
            raise SpecError(f"{self.title} does not support {transport}")
        opts = self.clean_options(options, transport)
        build = self.build_argv or default_argv
        return build(self, transport, opts, http_port, robot, space_confirmed)

    def to_json(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "title": self.title,
            "summary": self.summary,
            "needs": list(self.needs),
            "moves_robot": self.moves_robot,
            "supports_mock": self.supports_mock,
            "transports": list(self.transports),
            "options": [o.to_json() for o in self.options],
            "guide": self.guide,
        }


def default_argv(
    spec: DemoSpec,
    transport: str,
    options: dict[str, Any],
    http_port: int,
    robot: str | None,
    space_confirmed: bool,
) -> list[str]:
    """Transport flags, ``--http-port N --no-open``, every option as ``--flag value``, and the
    space flag when the legs move: on a real robot only after the user confirmed, in the
    simulator always (no real legs)."""
    argv: list[str] = []
    if transport == "mock":
        argv.append("--mock")
    elif transport == "usb":
        argv += ["--port", robot] if robot else []
    elif transport == "ble":
        argv += ["--ble", robot] if robot else ["--ble"]
    argv += ["--http-port", str(int(http_port)), "--no-open"]
    for o in spec.options:
        if o.name not in options:
            continue
        v = options[o.name]
        if o.type == "bool":
            if v:
                argv.append(o.cli_flag)
        else:
            argv += [o.cli_flag, str(v)]
    legs = spec.moves_robot and (spec.legs_move is None or spec.legs_move(options))
    if legs and spec.space_flag and (transport == "mock" or space_confirmed):
        argv.append(spec.space_flag)
    return argv
