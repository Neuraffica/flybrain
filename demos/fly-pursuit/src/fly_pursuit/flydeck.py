"""How fly-pursuit appears in Flydeck. Kept import-light: no numpy, no models."""

from flybrain_demo_api import DemoSpec, Option

# = controllers.CONTROLLER_KEYS, sim.FOV_CHOICES and sim.MOTIONS (a test keeps them equal)
CONTROLLERS = (
    "connectome",
    "single-brain",
    "no-aotu019",
    "no-aotu025",
    "aotu019-excitatory",
    "rewired",
    "random",
    "proportional",
    "bang-bang",
)
FOVS = (30, 60, 100)
MOTIONS = ("still", "sweep", "walk", "jumps", "hidden")

SPEC = DemoSpec(
    id="fly-pursuit",
    title="Fly pursuit",
    summary="A circuit of 265 neurons from the fly connectome turns a simulated robot dog "
    "toward a target. Simulator only for now: the robot version needs the camera measured "
    "first.",
    module="fly_pursuit.cli",
    needs=(),  # simulator only; the robot version will need a camera
    moves_robot=False,
    supports_mock=True,
    transports=("mock",),
    options=(
        Option(
            "controller",
            "choice",
            "connectome",
            label="Controller",
            choices=CONTROLLERS,
            help="connectome: the fly's wiring. The others are changed circuits and two "
            "simple baselines. Switchable in the dashboard.",
        ),
        Option(
            "fov",
            "choice",
            30,
            label="Camera field of view (+- deg)",
            choices=FOVS,
            help="30 and 60: a fixed camera. 100: a camera on a panning head (the head itself "
            "is not simulated). Switchable in the dashboard.",
        ),
        Option(
            "target",
            "choice",
            "still",
            label="Target motion",
            choices=MOTIONS,
            help="still: drag it in the dashboard. sweep, walk (random), jumps, or hidden "
            "(no target). Switchable in the dashboard.",
        ),
    ),
    guide="guide.html",
    http_port=8769,
)
