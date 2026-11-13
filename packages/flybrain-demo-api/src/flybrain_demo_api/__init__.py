"""The contract between a flybrain demo and Flydeck, the demo launcher.

A demo is a command-line program. To appear as a card in Flydeck it publishes a ``DemoSpec``
through the Python entry-point group ``flybrain.demos``::

    # in the demo's pyproject.toml
    [project.entry-points."flybrain.demos"]
    my-demo = "my_demo.flydeck:SPEC"

and follows the run contract (see ``docs/adding-a-demo.md``):

* transport flags ``--mock``, ``--port PATH``, ``--ble [NAME]``; ``--http-port N``; ``--no-open``;
* reports progress with ``status.ready(url)``, ``status.state(...)``, ``status.failed(reason)``;
* exits cleanly on the first SIGINT, leaving the robot resting.

This package has no dependencies, so a demo can use it without pulling in Flydeck.
"""

from . import status
from .paths import user_dir
from .spec import (
    EXIT_CODES,
    TRANSPORTS,
    DemoSpec,
    Option,
    SpecError,
    default_argv,
)

__all__ = [
    "EXIT_CODES",
    "TRANSPORTS",
    "DemoSpec",
    "Option",
    "SpecError",
    "default_argv",
    "status",
    "user_dir",
]
