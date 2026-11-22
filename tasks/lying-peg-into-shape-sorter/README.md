# Stand a peg lying on its side up and push it through its shape-sorter hole

A rectangular peg has fallen over and lies on its side in front of the left arm; the shape-sorter board is clamped in
front of the right arm. The peg has to be picked up, stood on end and lined up within 1.5 mm before it goes through
its hole. Neither arm reaches both the peg's spot and the hole, so the peg also has to change hands.

| File | What it is |
|---|---|
| `lying_peg.py` | The `yam_arena` task file: scene, step scoring (`success`, `score`, `distance`) and the scripted `demo()` |
| `partkit.py` | Helpers: USD parts at a full pose, gripper states for scoring, full-rotation waypoints (see Credits) |
| `make_assets.py` | Writes `assets/*.usda` from the FMB peg-board meshes |
| `assets/board.usda`, `assets/peg_rectangle.usda` | The board and the peg, one rigid body each |
| `task.json` | The task's details for the board |

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`). The instruction is `task.json`'s, word for word:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lying_peg.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/lying-peg-dream

# Scripted demo (both arms), 900 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lying_peg.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/lying-peg-demo
```

- `-E object_jitter=0` keeps the layout fixed, so no seed is needed. A trial takes 2 minutes of simulated time at
  most (2,400 steps at 20 Hz); the scripted demo succeeds at about 31 s.
- The log prints a `[lying_peg] t=...` line as each step is reached. Set `FMB_DEBUG=1` to print the peg's position
  and tilt and both grippers every 0.5 s.
- The scripted demo: the left gripper takes the lying peg from above, tilted 50 degrees, near its far end, lifts it
  and stands it on end about that grip (the same move as the battery pick in `flashlight-battery-insertion`), then
  sets it down upright between the arms and holds it. The right gripper takes its top from above, the left lets go
  and backs off, and the right turns it a quarter turn to the hole's long side, carries it over the hole, eases it
  into the hole's mouth with a small circling search (1.2 mm), pushes it down, lets go and presses it home with the
  closed fingers.

## Scoring

Step by step, judged from the video; each step counts once reached:

| Score | Step |
|---|---|
| 0.25 | Grasped: one gripper alone has lifted the peg with its lowest point 3 cm above the table. |
| 0.5 | Handed over: after that, the other gripper alone holds the peg with its lowest point off the table. |
| 0.75 | In the target hole: the peg's lower end is in the rectangular hole (centre within 5 mm of the hole's centre, axis within 5 degrees of vertical, lowest point at most 30 mm above the table). |
| 1.0 | Fully in (lowest point within 10 mm of the table, through the 50 mm board), released by both grippers and still for 1 s; this ends the run. |

0 if the peg ends off the table. A gripper holds the peg when its fingertip centre is within the peg's outline (grown
by 6 mm) and both fingers press on it. In a real trial the peg's height in the hole is measured with a ruler after
the run.

## Real-world setup

- **Board and peg:** the FMB peg board (medium) and its rectangular peg (medium, short, 100 x 40.3 x 25.9 mm) from
  the FMB CAD files at <https://functional-manipulation-benchmark.github.io/files/>, 3D-printed in PLA. Clamp the board
  to the table at its mark, edges square to the table.
- **Layout:** as in `task.json`'s setup and the screenshot; mark the peg's two ends on the table with tape.
- **Reset:** pull the peg out and lay it on its mark (about 10 s).

## Simulation

Rigid bodies, 240 Hz physics, 20 Hz control. The board and peg are the FMB meshes at full size, converted by
`make_assets.py`. The board collides as a signed distance field, so its holes are the real through-holes with 1.5 mm
of clearance per side; the peg collides as its convex hull. PLA: density 1.24 g/cm³, friction 0.5–0.6. Contact offset
0.4 mm. The board is kinematic (clamped). No deformation.

## Credits and licences

- **FMB peg board and peg:** Functional Manipulation Benchmark (Luo et al., 2024),
  <https://functional-manipulation-benchmark.github.io>, released under CC BY 4.0. The meshes are converted unchanged
  apart from the physics settings.
- **`partkit.py`** adapts code from two accepted tasks in this repository: `flashlight-battery-insertion` (`UsdPart`,
  the scene patch, gripper states, `Waypoint.rot`) and `snapfit-close` (`on_step`, `on_reset`, `write_pose`). The
  left arm's stand-up move follows `flashlight-battery-insertion`'s battery pick.
- Everything else here is released under CC BY 4.0.
