"""How hello-robot appears in Flydeck: a card, a settings form, and how to start it."""

from flybrain_demo_api import DemoSpec, Option

SPEC = DemoSpec(
    id="hello-robot",
    title="Hello robot",
    summary="Connects, shows who the robot is and how it is tilted, wags its head, rests.",
    module="hello_robot.cli",  # Flydeck runs: python -m hello_robot.cli --mock|--port|--ble ...
    needs=("Bittle X (any link)",),
    moves_robot=True,
    supports_mock=True,
    options=(
        Option(
            "wags",
            "int",
            3,
            label="Head wags",
            minimum=1,
            maximum=10,
            help="how many times the head turns left and right",
        ),
        Option(
            "walk",
            "bool",
            False,
            label="Take a few steps forward",
            help="the legs walk for about a second; Flydeck asks you to confirm space",
        ),
    ),
    guide="README.md",
    legs_move=lambda options: bool(options.get("walk")),  # only then ask "has space?"
    http_port=8767,
)
