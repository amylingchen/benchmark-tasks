# Put four pegs into a shape-sorter board one by one without knocking the others over

Four pegs of different shapes stand close together beside a shape-sorter board. The robot puts each one through the
hole of its own shape, where it has 1.5 mm of clearance per side, without knocking any of the others over.

| File | What it is |
|---|---|
| `four_pegs.py` | The `yam_arena` task file: scene, scoring (`success`, `score`, `distance`) and the scripted `demo()` |
| `partkit.py` | Helpers: USD parts at a full pose, gripper states for scoring, full-rotation waypoints (see Credits) |
| `make_assets.py` | Writes `assets/*.usda` from the FMB peg-board meshes |
| `assets/board.usda`, `assets/peg_*.usda` | The board and the four pegs, one rigid body each |
| `task.json` | The task's details for the board |

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=four_pegs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/four-pegs-dream

# Scripted demo (one arm at a time), 2,000 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=four_pegs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/four-pegs-demo
```

- `-E object_jitter=0` keeps the layout fixed, so no seed is needed. A trial takes 2 minutes of simulated time at
  most (2,400 steps at 20 Hz); the scripted demo succeeds at about 86 s.
- The log prints a `[four_pegs] t=...  <peg> knocked over` line if that happens. Set `FMB_DEBUG=1` to print every
  peg's position and tilt and both grippers every 0.5 s.
- The scripted demo: the left arm puts in the rectangle and then the oval, the right arm then the hexagon and the
  round peg. Each peg is gripped from above (about 35 N), carried over its hole, eased into the hole's mouth with a
  small circling search (1.2 mm) and pushed down, released, and pressed home with the closed fingers. Its offsets
  over the holes were measured in earlier runs of this deterministic layout.

## Scoring

The score is the peg count: 0.25 for each peg fully in its own hole, judged at the end from the video and a ruler:

- its bottom within 10 mm of the table, so it has gone all the way through the 50 mm board;
- its centre within 5 mm of the hole's centre (it is then within the hole, which is only 3 mm wider than the peg);
- its axis within 5 degrees of vertical.

A peg in the wrong hole scores nothing (the hexagon would fit through the round hole). With all four in, the run ends
with 1.0 once they have stayed still for 1 s.

0 for the whole trial if any peg is ever knocked over: tilted more than 45 degrees from upright, or off the table, even
if it is stood up again later. In a real trial, watch the video for any peg falling.

## Real-world setup

- **Board and pegs:** the FMB peg board (medium) and its rectangle, oval, hexagon and round pegs (medium, short,
  100 mm) from the FMB CAD files at <https://functional-manipulation-benchmark.github.io/files/>, 3D-printed in PLA.
  Clamp the board to the table at its mark, edges square to the table.
- **Layout:** as in `task.json`'s setup and the screenshot; mark each peg's spot on the table with tape.
- **Reset:** pull the pegs up out of the board and stand each on its mark (about 20 s).

## Simulation

Rigid bodies, 240 Hz physics, 20 Hz control. The board and pegs are the FMB meshes at full size, converted by
`make_assets.py`. The board collides as a signed distance field, so its holes are the real through-holes with 1.5 mm
of clearance per side; each peg collides as its convex hull (all four shapes are convex). PLA: density 1.24 g/cm³,
friction 0.5–0.6. Contact offset 0.4 mm. The board is kinematic (clamped). No deformation.

## Credits and licences

- **FMB peg board and pegs:** Functional Manipulation Benchmark (Luo et al., 2024),
  <https://functional-manipulation-benchmark.github.io>, released under CC BY 4.0. The meshes are converted unchanged
  apart from the physics settings.
- **`partkit.py`** adapts code from two accepted tasks in this repository: `flashlight-battery-insertion` (`UsdPart`,
  the scene patch, gripper states, `Waypoint.rot`) and `snapfit-close` (`on_step`, `on_reset`, `write_pose`).
- Everything else here is released under CC BY 4.0.
