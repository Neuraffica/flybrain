"""How fly-pursuit appears in Flydeck. Kept import-light: no numpy, no models."""

from typing import Any

from flybrain_demo_api import DemoSpec, Option
from flybrain_demo_api.spec import default_argv

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
CAMERAS = ("auto", "module", "robot", "usb")  # = robot.CAMERA_SOURCES (a test keeps them equal)
AUTOSTART = ("leave as it is", "on", "off")  # the card's choice; on / off = cli --autostart
KEEP = AUTOSTART[0]


def build_argv(
    spec: DemoSpec,
    transport: str,
    options: dict[str, Any],
    http_port: int,
    robot: str | None,
    space_confirmed: bool,
) -> list[str]:
    """default_argv, except that "autostart: leave as it is" passes no flag at all."""
    if options.get("autostart") == KEEP:
        options = {k: v for k, v in options.items() if k != "autostart"}
    return default_argv(spec, transport, options, http_port, robot, space_confirmed)


SPEC = DemoSpec(
    id="fly-pursuit",
    title="Fly pursuit",
    summary="A circuit of 265 neurons from the fly connectome turns a robot dog toward a "
    "target: the whole dog in the simulator, the head toward a face seen by the camera in its "
    "mouth on the robot (the legs do not walk). With our firmware module the robot can run "
    "the circuit (264 of its neurons) on its own board.",
    module="fly_pursuit.cli",
    needs=(
        (
            "On the robot: the Petoi AI Vision camera in the dog's mouth, plugged into the "
            "Grove socket labelled SCL SDA +5V GND, or (camera: usb) its own USB-C cable into "
            "this computer"
        ),
        "The robot on the floor, away from edges: it sits when the demo starts",
        (
            "Optional but recommended: our camera module in the robot's firmware "
            "(firmware/bittle/fly_pursuit, see its README). It reads the camera every frame, "
            "starts any number of times and is needed for 'onboard'. Without it, camera: auto "
            "uses the stock camera module"
        ),
        (
            "Stock camera module over Bluetooth: switch the robot off and on first if its "
            "camera was used since; it starts only once per power-on"
        ),
    ),
    moves_robot=True,  # the head; the firmware makes the robot sit
    supports_mock=True,
    transports=("mock", "usb", "ble"),
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
            mock_only=True,
        ),
        Option(
            "target",
            "choice",
            "still",
            label="Target motion",
            choices=MOTIONS,
            help="still: drag it in the dashboard. sweep, walk (random), jumps, or hidden "
            "(no target). Switchable in the dashboard.",
            mock_only=True,
        ),
        Option(
            "onboard",
            "bool",
            False,
            label="Robot: the robot computes (on-board)",
            help="The circuit runs in our firmware module on the robot's board and turns the "
            "head by itself; the dashboard shows the robot's numbers and recomputes the "
            "circuit's activity from them. Needs firmware/bittle/fly_pursuit. Controller, "
            "arousal and gain are then fixed (connectome, 1, 1).",
        ),
        Option(
            "camera",
            "choice",
            "auto",
            label="Robot: camera connection",
            choices=CAMERAS,
            help="auto: our firmware module if the robot has it, else the stock one. module: "
            "through the robot, read by our module: a result every frame with a score and a "
            "real 'no face', any number of starts. robot: the stock camera module: one start "
            "per power-on, positions only when they change, no IMU while it runs. usb: the "
            "camera's own USB cable into this computer (a cable from the dog's mouth).",
        ),
        Option(
            "camera_port",
            "str",
            "",
            label="Robot: camera's USB port (camera: usb)",
            help="Empty: found by asking each port for its name. Give it when the robot is "
            "also on USB.",
        ),
        Option(
            "gain",
            "float",
            1.0,
            label="Robot: turn gain",
            minimum=0.1,
            maximum=2.0,
            help="Scales the circuit's turn command on the robot, for tuning. Ignored in the "
            "simulator.",
        ),
        Option(
            "camera_fov",
            "float",
            60.0,
            label="Robot: camera field of view across (deg)",
            minimum=10,
            maximum=170,
            help="Not properly measured (one head step gave about 57). Turns the camera's x position into a bearing. "
            "Ignored in the simulator.",
        ),
        Option(
            "head_rate",
            "float",
            60.0,
            label="Robot: fastest head turn (deg/s)",
            minimum=1,
            maximum=360,
            help="This program's limit on the turn command. Ignored in the simulator.",
        ),
        Option(
            "pan_limit",
            "float",
            50.0,
            label="Robot: head turn limit (+- deg)",
            minimum=1,
            maximum=120,
            help="How far the head may turn each way; the firmware allows 120. Ignored in "
            "the simulator.",
        ),
        Option(
            "command_rate",
            "float",
            8.0,
            label="Robot: head commands per second",
            minimum=0.5,
            maximum=30,
            help="Head commands compete with the camera on the robot's board. Ignored in the "
            "simulator.",
        ),
        Option(
            "no_usb_restart",
            "bool",
            False,
            label="Robot over USB: do not restart the board first",
            help="With the stock camera module, the board is restarted before each run "
            "because that camera starts only once per power-on. Our module needs no restart.",
        ),
        Option(
            "leave_voice_off",
            "bool",
            False,
            label="Robot: leave the voice module off at the end",
            help="The voice module is switched off while the demo runs (sounds can make the "
            "robot move) and back on at the end unless this is ticked.",
        ),
        Option(
            "camera_off",
            "bool",
            False,
            label="Robot: only switch the camera off",
            help="Recovery: switch the camera off (both the stock module and ours) and save "
            "that on the robot, with our module's 'follow a face after power-on' off too, "
            "rest, exit.",
        ),
        Option(
            "autostart",
            "choice",
            KEEP,
            label="Robot: follow a face after power-on",
            choices=AUTOSTART,
            help="on / off: only save this on the robot, rest, exit (no dashboard). With it "
            "on, at every power-on and with no computer connected, the robot beeps three times, "
            "sits and moves its head toward a face by itself: switch it on only on the floor, "
            "away from edges. Needs the newer build of firmware/bittle/fly_pursuit. leave as "
            "it is: a normal run, the saved setting is not changed.",
        ),
    ),
    guide="guide.html",
    legs_move=lambda o: False,  # nothing in this demo makes the legs walk
    build_argv=build_argv,
    http_port=8769,
)
