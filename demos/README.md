# Demos

| Demo | What it shows |
|---|---|
| [fly-compass](fly-compass/README.md) | A connectome model of the fly head-direction ring keeps a Bittle X's head (and arm) pointing one way, anchored by the IMU yaw as a landmark cue; a blindfold removes the cue to test what the circuit holds on its own. Runs with a simulated robot: `uv run fly-compass --mock` |
| [fly-escape](fly-escape/README.md) | A connectome model of the fly's looming-escape circuit, fed by an infrared proximity sensor, makes the dog walk back from an approaching hand; compared live against rewired and random wirings and two simple thresholds. Simulated: `uv run fly-escape --mock --i-have-space` |
| [fly-escape-onboard](fly-escape-onboard/README.md) | The same escape circuit running on the robot's own board (firmware module `E`): the robot decides by itself, the laptop only listens and draws the robot's sensor, giant fibres and decisions. Simulated: `uv run fly-escape-onboard --mock --open` |
| [fly-pursuit](fly-pursuit/README.md) | A circuit of 265 neurons from the fly connectome (the small-object pursuit pathway: LC10a, AOTU019, AOTU025, DNa02) turns a simulated robot dog toward a target you drag around it; switch off cell types, shuffle the wiring or compare with two simple rules. On the robot it turns the head toward a face seen by the Petoi AI Vision camera in the dog's mouth, read through the robot or over the camera's own USB cable (the legs do not walk). Simulated: `uv run fly-pursuit --mock --open`; robot: `uv run fly-pursuit --ble --open` or `--ble --camera usb` |
| [hello-robot](hello-robot/README.md) | The template for your own demo: connects, shows identity and tilt, wags the head, rests. `uv run flydeck new-demo my-idea` copies it. Simulated: `uv run hello-robot --mock --open` |

All of them appear in [Flydeck](../apps/flydeck/guide.html) (the Mac app, or `uv run flydeck`). To add one, see [docs/adding-a-demo.md](../docs/adding-a-demo.md).

Planned: the reference maze demo and the smaller demos that come out of it. See [docs/demo-plan.md](../docs/demo-plan.md).
