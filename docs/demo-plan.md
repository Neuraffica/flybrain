# Demo plan: FlyDog in the maze

Status: plan only. Nothing here is implemented yet.

A reference demo for the organizers to show at 10:00 and for teams to take apart. A Bittle X
walks a small maze using three fly circuits running on a XIAO ESP32-S3 on its head, with no
laptop in the loop, and every circuit is compared against a standard baseline.

## What the audience sees (3 minutes)

1. The dog is placed at the maze entrance. A hand swipes at it; it flinches back
   (looming detector).
2. It walks the maze: stays centred in corridors, stops before walls, turns at openings.
3. At the exit it turns around and walks back to the start on its compass alone.
4. A slide shows the scoreboard: fly circuit, rewired control and standard baseline, with
   maze time, collisions, latency and memory.

## Architecture

```
XIAO ESP32-S3 Sense (on the head)             BiBoard (in the body)
  camera 32x32 gray, 20 Hz                      OpenCat firmware, unchanged
    -> motion detector (T4/T5)                  servos, IMU
        -> looming (LPLC2)   -> stop/back
        -> flow balance      -> steer      UART
  IMU yaw (from BiBoard)                  <------>  'k' skills, 'i' joint angles,
    -> ring-attractor compass -> heading            'v' IMU readout
    -> path integrator        -> home vector
  arbiter: looming > steering > goal heading
```

The laptop is used only to train, flash and plot telemetry (sent over BLE or WiFi, optional).

| Function | Fly circuit | Control | Standard baseline |
|---|---|---|---|
| Obstacle stop | LPLC2-style looming on T4/T5 motion | Same detector with shuffled direction wiring | Frame differencing; ultrasonic threshold |
| Corridor steering | Left/right wide-field flow balance (HS cells) | | Small CNN regressing steering from the frame |
| Heading and homing | Central-complex ring attractor + path integrator | Rewired ring | Direct yaw integration |
| Gait (stretch) | MANC-coupled oscillators | Rewired coupling | Built-in `kwkF` |

## Milestones

Each one ends in something that can be shown. Stop at any point and there is still a demo.

| # | Milestone | Done when | Fallback if it slips |
|---|---|---|---|
| M0 | **Bring-up.** Laptop drives the dog over USB; XIAO streams camera frames; the six hardware checks in `docs/hardware.md` are answered | Measured command rate, frame rate and IMU format are written into the hardware doc | None. Everything depends on this |
| M1 | **Circuit packs.** `flybrain-data` downloads FlyWire v783 and emits the motion, looming, central-complex and mushroom-body packs with controls | Packs load in a notebook; neuron counts match the annotation table | Hand-write the small circuits from the literature; note that the wiring is schematic |
| M2 | **Looming on the desk.** Motion + looming detector in Python on recorded XIAO clips, then in C on the XIAO, LED on detection | Detection and false-alarm rates against frame differencing, on-device latency | Ship as the "swatter dodge" starter project on its own |
| M3 | **Tethered maze.** Tier B: XIAO frames to laptop, Python controller, skills over USB. Looming stop and flow steering | Dog exits the maze 3 runs out of 5 | Simplify the maze to one corridor and one turn |
| M4 | **Compass homing.** Ring attractor from IMU yaw; path integration from commanded steps; return to start | Return error under one body length on an out-and-back path | Demo the compass as a live heading display only |
| M5 | **Untethered.** All of M2 to M4 in C on the XIAO, wired to BiBoard UART | Full run with the laptop closed | Keep M3/M4 tethered and say so. Show M2 as the on-device proof |
| M6 | **Scoreboard and tutorial.** Baselines and controls run through the benchmark harness; the build is written up as tutorials | Table of results reproducible with one command | Report the subset that was measured |
| M7 | **Stretch: route memory or fly gait** | | Drop |

Order of work: M0 first and alone. M1 and M2 can run in parallel with M3 once M0 is done.
M5 is the riskiest step and should start no later than one week before the event.

## Schedule to Dec 5

| Week | Target |
|---|---|
| Nov 11 to 15 | M0, M1. Build the maze. Record datasets on the real hardware |
| Nov 16 to 22 | M2, M3. First tutorials (hello-world, circuit packs, reservoir) |
| Nov 23 to 29 | M4, M5. Pretrained model zoo and baselines. Benchmark harness |
| Nov 30 to Dec 4 | M6. Dry run with someone who has not seen the kit. Freeze; flash all devices; load USB sticks |

## Risks

| Risk | Mitigation |
|---|---|
| Camera sees motion blur and its own head bobbing while walking | Sample frames at the stance phase of the gait, or stop-look-go; low resolution helps |
| Optic flow fails on plain walls | Textured maze walls (already in the maze spec) |
| Path integration drifts because step length is not constant | Short maze; calibrate distance per gait cycle; reset heading at known walls |
| UART link to BiBoard V1 does not work as on V0 | Found in M0. Fall back to BLE between XIAO and BiBoard, or tier B |
| Fly circuit does no better than its control | Report it. A measured null result satisfies the rules better than a claim |
| Daylight blinds the infrared proximity sensor (measured: beside a window at midday it reads as permanently covered) | Put sensor demos away from windows or shield the sensor; the app refuses to start in that state |
| Venue radio congestion | The demo is wired or on-device; telemetry is optional |

## Smaller demos that fall out of this

- **Swatter dodge** (M2 alone): a XIAO on a desk, no robot.
- **Compass display** (M4 alone): rotate the dog by hand, watch the activity bump move
  around the ring on a laptop plot.
- **One-shot classifier**: mushroom-body pack on the XIAO, taught live by the audience.
