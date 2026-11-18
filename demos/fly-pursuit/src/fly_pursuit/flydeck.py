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
    summary="A circuit of 265 neurons from the fly connectome turns a robot dog toward a "
    "target: the whole dog in the simulator, the head toward a face seen by the camera in its "
    "mouth on the robot (the legs do not walk).",
    module="fly_pursuit.cli",
    needs=(
        (
            "On the robot: the Petoi AI Vision camera in the dog's mouth, plugged into the "
            "Grove socket labelled SCL SDA +5V GND, or (camera: usb) its own USB-C cable into "
            "this computer"
        ),
        "The robot on the floor, away from edges: it sits when the demo starts",
        (
            "Camera through the robot over Bluetooth: switch the robot off and on first if its "
            "camera was used since; the firmware starts its camera only once per power-on"
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
            "camera",
            "choice",
            "robot",
            label="Robot: camera connection",
            choices=("robot", "usb"),
            help="robot: through the robot's Grove socket. No cable to the laptop, but one "
            "start per power-on, positions only when they change, and no IMU or infrared "
            "sensor while it runs. usb: the camera's own USB cable into this computer: 15 "
            "results a second with a score and a real 'lost', any number of starts, but a "
            "cable from the dog's mouth.",
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
            help="With the camera through the robot, the board is restarted before each run "
            "because its camera starts only once per power-on.",
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
            help="Recovery: switch the camera off and save that on the robot, rest, exit.",
        ),
    ),
    guide="guide.html",
    legs_move=lambda o: False,  # nothing in this demo makes the legs walk
    http_port=8769,
)
