# Demos

| Demo | What it shows |
|---|---|
| [fly-compass](fly-compass/README.md) | A connectome model of the fly head-direction ring keeps a Bittle X's head (and arm) pointing one way, anchored by the IMU yaw as a landmark cue; a blindfold removes the cue to test what the circuit holds on its own. Runs with a simulated robot: `uv run fly-compass --mock` |
| [fly-escape](fly-escape/README.md) | A connectome model of the fly's looming-escape circuit, fed by an infrared proximity sensor, makes the dog walk back from an approaching hand; compared live against rewired and random wirings and two simple thresholds. Simulated: `uv run fly-escape --mock --i-have-space` |
| [hello-robot](hello-robot/README.md) | The template for your own demo: connects, shows identity and tilt, wags the head, rests. `uv run flydeck new-demo my-idea` copies it. Simulated: `uv run hello-robot --mock --open` |

All of them appear in [Flydeck](../apps/flydeck/guide.html) (`uv run flydeck`). To add one, see [docs/adding-a-demo.md](../docs/adding-a-demo.md).

Planned: the reference maze demo and the smaller demos that come out of it. See [docs/demo-plan.md](../docs/demo-plan.md).
