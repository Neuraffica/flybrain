"""How fly-escape appears in Flydeck. Kept import-light: no numpy, no models."""

from flybrain_demo_api import DemoSpec, Option

MODES = ("observe", "escape")  # = app.MODES (a test keeps these equal)
DRIVERS = ("connectome", "rewired", "random", "prox", "looming")  # = models.MODEL_KEYS
ACTIONS = ("back", "back-turn")  # = escape.ACTIONS
SIDES = ("auto", "normal", "swapped")  # = sides.SIDE_SETTINGS

SPEC = DemoSpec(
    id="fly-escape",
    title="Fly escape",
    summary="A fly looming-escape circuit makes the robot back away from an approaching hand.",
    module="fly_escape.cli",
    needs=("IR proximity sensor on the Grove socket labelled 39/36", "about 1 m clear behind"),
    moves_robot=True,
    supports_mock=True,
    options=(
        Option(
            "mode",
            "choice",
            "escape",
            label="Mode",
            choices=MODES,
            help="escape: the robot walks back when the model sees a threat. observe: it only "
            "watches and scores.",
        ),
        Option(
            "driver",
            "choice",
            "connectome",
            label="Model that moves the robot",
            choices=DRIVERS,
            help="connectome: the fly's wiring. The others are controls; switchable in the "
            "dashboard.",
        ),
        Option(
            "escape_action",
            "choice",
            "back",
            label="Escape move",
            choices=ACTIONS,
            help="back: walk straight back. back-turn: walk back turning away from the hand.",
        ),
        Option(
            "sides",
            "choice",
            "auto",
            label="Which sensor side is the robot's left",
            choices=SIDES,
            help="auto: what the side check found for this robot (sides are not swapped until "
            "it has run once). normal / swapped: force it.",
        ),
        Option(
            "calibrate_sides",
            "bool",
            False,
            label="Calibrate sides at start",
            help="after the resting level, hold a hand at the robot's left when asked; the "
            "answer is remembered for this robot. Also a button in the dashboard.",
        ),
        Option(
            "mock_random",
            "bool",
            False,
            label="Random approaches",
            mock_only=True,
            help="the simulator keeps sending hands (threats and harmless ones) by itself.",
        ),
    ),
    guide="guide.html",
    legs_move=lambda o: o.get("mode") == "escape",
    http_port=8766,
    ready_timeout_s=60.0,
)
