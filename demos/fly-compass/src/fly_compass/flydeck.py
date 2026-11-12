"""How fly-compass appears in Flydeck. Kept import-light: no numpy, no models."""

from flybrain_demo_api import DemoSpec, Option

MODES = ("needle", "hold", "sweep", "observe")  # = loop.MODES (a test keeps them equal)
LEG_MODES = ("hold", "sweep")
DRIVERS = ("connectome", "rewired", "gyro")  # = models.MODEL_KEYS
MOCK_SCRIPTS = ("none", "sweep", "memory", "dark-turn")  # = scenarios.SCRIPT_CHOICES

SPEC = DemoSpec(
    id="fly-compass",
    title="Fly compass",
    summary="A fruit-fly head-direction circuit keeps the robot's head pointing one way.",
    module="fly_compass.cli",
    needs=("Bittle X with head (any link)",),
    moves_robot=True,
    supports_mock=True,
    options=(
        Option(
            "mode",
            "choice",
            "needle",
            label="Mode",
            choices=MODES,
            help="needle: only the head turns. hold: the legs step to keep the body on course. "
            "sweep: the robot turns itself left and right. observe: nothing moves.",
        ),
        Option(
            "driver",
            "choice",
            "connectome",
            label="Model that drives the head",
            choices=DRIVERS,
            help="connectome: the fly's wiring; rewired: same neurons, shuffled wiring; "
            "gyro: plain integration, for comparison. Switchable in the dashboard.",
        ),
        Option(
            "sweep_angle",
            "float",
            35.0,
            label="Sweep angle (deg)",
            minimum=5,
            maximum=90,
            help="sweep mode turns to +angle and -angle of the start heading.",
        ),
        Option(
            "mock_script",
            "choice",
            "none",
            label="Simulator script",
            choices=MOCK_SCRIPTS,
            mock_only=True,
            help="memory: blindfolded and still for 20 s; dark-turn: blindfolded 90 deg turn. "
            "Not with sweep mode.",
        ),
    ),
    guide="guide.html",
    legs_move=lambda o: o.get("mode") in LEG_MODES,
    http_port=8765,
    ready_timeout_s=60.0,
)
