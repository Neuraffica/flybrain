# What a team can deliver in 5 hours

Hacking runs 11:00 to 16:00. That is enough for one circuit, one task, one baseline and one
device, provided the plumbing already works. This page lists projects sized to fit, and
how the starter kit maps to the hackathon's suggested ideas.

## How the kit maps to the official ideas

| Official idea | Kit pieces |
|---|---|
| Image classification on the fly's visual circuits | Motion-pathway circuit pack, mushroom-body classifier, XIAO camera firmware |
| Looming detection | Looming detector, recorded looming clips, XIAO firmware. Works on a desk with no drone |
| Robot dog gait from locomotion circuits | MANC leg circuit pack, gait generator, Bittle link and sim |
| RL policy shaped like the connectome | Circuit pack as policy mask, Bittle sim environment |
| Something nobody has tried | Compass + route memory for maze navigation (see the demo plan) |

## Project menu

Difficulty assumes a team of 2 to 4 using the pretrained pieces.

### Starter (working demo by 14:00)

1. **Swatter dodge.** XIAO camera runs the looming detector; the dog jumps back when a hand
   approaches fast. Compare against frame differencing: detection rate, false alarms,
   latency.
2. **Teach the dog a trick by showing it once.** Mushroom-body classifier on the XIAO;
   show an object once per class, the dog responds with a different skill each. Compare
   against a small CNN given the same single example.
3. **Which floor am I on?** A reservoir built from a fly circuit classifies terrain or
   gait state from IMU traces. Compare against rewired and random reservoirs and an MLP.
4. **Gesture or clap commands.** Same reservoir pipeline on microphone or IMU input.

### Core (demo by 15:30)

5. **Fly-coupled gait.** Replace the built-in walk with the connectome-coupled gait
   generator. Measure speed, straightness and stability (IMU variance) against `kwkF`.
6. **Corridor following.** Left/right optic flow balance from the motion detector steers
   the dog down a corridor. Compare against ultrasonic wall following.
7. **Compass homing.** The dog is led away by hand commands, then returns to the start
   using the ring-attractor path integrator. Measure the return error against plain
   dead reckoning.
8. **Maze walk.** Combine 1, 6 and 7: looming stop, optic-flow steering, compass.

### Stretch

9. **Route memory.** Drive the maze once; mushroom-body view familiarity lets the dog
   retrace it. One-shot learning on a microcontroller.
10. **Fetch with the arm.** Looming or visual detector finds an object; the arm picks it up
    with `kpickF`.
11. **Whole fly brain drives the dog.** The spiking whole-brain model on a laptop; sensor
    input to sensory neurons, descending-neuron rates to walk and turn commands.
12. **Connectome-shaped RL.** Train a policy in the Bittle sim with the circuit pack as its
    wiring; compare sample efficiency against an MLP policy; transfer to the dog.

## On the maze

A maze is a good shared arena because it scores itself: time to exit, collisions, and
whether the dog makes it at all. Keep it small.

- Corridors about 40 cm wide (the dog is about 20 cm long), walls 25 cm high, cardboard or
  foam board. One T-junction and two turns is plenty.
- Give walls a printed texture. Optic flow and looming detectors need contrast; blank white
  walls give them nothing.
- Matte floor with grip. Bittle slips on polished surfaces.
- Build two identical mazes so teams are not queueing.
- Walking speed is slow and battery life is about an hour, so plan for runs of 1 to 2
  minutes and have spare charged batteries.

## A realistic day for one team

| Time | Goal |
|---|---|
| 11:00 | Run the kit's hello-world: laptop makes the dog walk, XIAO blinks on a camera event |
| 11:30 | Pick one circuit and one task. Run the pretrained example and its baseline |
| 12:30 | Own data collected on own device; first trained readout |
| 13:30 | Model exported and running on the device; latency and memory measured |
| 14:30 | Controls run (rewired, random, standard baseline). Numbers in a table |
| 15:30 | Demo rehearsed twice; video recorded as a backup |
| 16:00 | Submit |

The most common way to fail is spending three hours on connectivity. The kit's first
tutorial exists to get that under 30 minutes.
