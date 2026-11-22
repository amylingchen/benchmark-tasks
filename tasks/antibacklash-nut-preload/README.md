# Thread a spring-preloaded anti-backlash nut onto a lead screw, keeping its two halves pressed together

Domain: impact on AI timelines (robot building). Lead-screw axes with spring-preloaded anti-backlash nuts drive linear
joints and Z stages in robots, in lab-automation gantries and in the 3D printers that make robot parts. StepperOnline's
TR8x8 lead-screw stepper 17LS19-1684E-200G-AR4 is "the J5 motor in AR4-MK2 kit", an open robot-arm kit. When the nut is
fitted, its preload is set too.

The nut is a two-piece brass anti-backlash nut for an 8 mm, 8 mm-lead TR8x8 screw. It has a flanged front nut, a
compression spring and a rear nut, which two guide pins keep from turning against the front nut. The spring pushes the
two nuts against opposite flanks of the thread, and that removes the play. The makers give the procedure and its catch:

- ReliaBot (T8 anti-backlash nut, Amazon B079JQ7S3Y): "Please install the 3 parts at the same time, instead of
  installing them piece by piece. That is to say, aligning the 2 brass nuts before installing them to the screw, with
  the spring between them."
- Zyltech (ABN-Tr8x8): "Adjustable preload from 4 N to 8 N with stock spring", and "Preload is varied by adjusting the
  spacing between front and rear nuts."
- Novanta IMS, on moving an anti-backlash nut from one screw straight onto another: "hold the nut together between
  your thumb and forefinger so that it cannot expand axially" (their nut is a cam and torsion-spring type; the same
  rule, keep the halves together while threading on, is what this task tests).
- US patent 9618104 B2, on the conventional spring nut: the parts "have to be held in place while the lead nut
  assembly is threaded on to the screw. This process is difficult and requires considerable physical dexterity."

## What the robot does

The actuator stands upright on the table: a NEMA-17 stepper with a 100 mm TR8x8 lead screw, its motor hanging under
a small plate on standoffs. The nut sits flange down on the screw's top end. It rests on the thread start and has not
been turned on yet. The robot grips the 14 mm rear nut from above and presses it down onto the front nut until the
spring is home (8 N). It turns the whole nut clockwise while following it down the lead (8 mm a turn) and keeps
pressing. Then it eases off, opens, turns its wrist back and grips again. The wrist reaches about 165 degrees a grasp,
so the demo takes ten grasps. At 14 mm of travel the screw's end reaches the rear nut. If the rear nut is pressed home
by then, it catches the thread nested, 1 mm from the front nut's body, with 8 N of preload. The robot runs the nut on down
until it hides a red mark 32 mm down, then lets go.

## Why it is hard (and how it differs from what exists)

1. **Press and turn at once, against a spring.** Keeping the 14 x 9 mm rear nut pressed home takes 8 N, held through
   every turn while the hand follows the nut down 3.7 mm a grasp. The YAM's joints are soft (about 0.55 N/mm
   vertically at this pose), so the arm's command has to go about 20 mm under the nut. The fingertips grip the rear
   nut's upper 6 mm only, because the spring and guide pins are just below it.
2. **Timing.** The spring pushes the rear nut back out at every regrasp. It has to be pressed home again before the
   screw's end reaches it, between about 11 and 14 mm of travel. Let it go then, or press too late, and it catches one
   or two thread starts out: 27 or 29 mm long, with 4 N or no preload, and the axis has play. Fixing that means
   turning the nut back until the screw leaves the rear nut, then pressing and threading again (the Novanta
   procedure). The simulator follows that too.
3. **No hidden numbers.** The preload comes from the spacing of the nuts, so a caliper reads it. Scoring needs a
   caliper, a rule and the video.

Closest earlier work, all checked against the full PR list (3,620 PRs on Nov 21) and the reviewer comments:

- #3241 (restore the preload of a robot leadscrew nut) was not accepted (final): a cam is turned while the other hand
  counterholds the housing, and a spring then sets the preload by itself. Here there is no counterhold (the screw is
  held by its motor), one hand presses and turns the same part, and the preload is whatever spacing the robot's own
  press gives at the moment the rear nut catches the thread: pressed too late, the nut catches one or two starts out.

- #779 (backlash dial setpoint) and #70 (geared pointer backlash) were not accepted (final). They operate a dial with
  backlash, and #779 was rejected for having "no physical stage or mechanical lock". Here the robot assembles the real
  backlash-removing mechanism.
- #2504 (move a ball nut from its shipping arbor onto a ball screw) was not accepted (final) because it overlapped
  existing tasks: transferring a nut off an arbor, like sliding a rail carriage on. There is no transfer here. The
  difficulty is the spring preload during threading.
- #813 (carriage preload) and #2802 (cam follower preload) were not accepted (final). A preset detent or remote
  control stood in for the adjustment. Here the preload follows from how the robot handles the real nut, and a caliper
  reads it.
- Covered ability "hold one part while the other hand turns something on it": in this task the screw is held by its
  motor, and one hand presses and turns the same part. The difficulty is keeping the spring compressed and the halves
  indexed through thread engagement, with precise force and timing, not holding one part steady for the other hand.

## The objects (real sizes, each number's source)

| Item | Value used | Source |
|---|---|---|
| Lead screw | TR8x8: 8 mm, 4 starts, 2 mm pitch, 8 mm lead, right hand | Zyltech, Micro Robotics TR8x8 listings |
| Motor and screw | NEMA 17, 42.3 x 42.3 x 38.7 mm body, TR8x8 100 mm, 0.44 N m holding, M3 on 31 mm | VXB "Nema 17 Stepper Motor With TR8x8 100mm Lead Screw" |
| Nut | brass two-piece spring nut for TR8x8, 44 g, preload 4-8 N by spacing | Zyltech ABN-Tr8x8 ($8.95) |
| Flange | 22 mm across, 4 holes 3.5 mm on a 16 mm circle | Micro Robotics TR8x8 nut; Stelltron T8 nut |
| Length | 25 mm (spring pressed home) | Stelltron T8 anti-backlash nut ("Nut Length: 25mm") |
| Rear nut, spring | 14 mm and 22 mm diameters, 12 mm spring | ACNTENY T8 anti-backlash nut listing |
| Front body | 10.2 mm (fits inside the 16 mm hole circle) | standard T8 flange nut body |
| Spring rate | 2 N/mm: one 2 mm thread start of spacing = 4 N, so 8 N nested, 4 N one start out, 0 two out | derived from Zyltech's 4-8 N; same order as Helix ATA 8-10 mm low-force springs (8.4-12.8 N) |
| Drag torque | about 0.06 N m to start, 0.06-0.08 N m turning at 1-2 rad/s (measured in the simulator; the code's value is 0.04 N m and the joint's damping adds half), about 1.7 times the makers' 5 oz-in max | Kerk CMP 5/16 in (8 mm) anti-backlash nut, 5 oz-in max; Helix ATA 8-10 mm, 5 oz-in max |
| Plate | 60 x 60 x 3 mm aluminium, 23 mm centre hole, on four M3 x 40 mm standoffs | ordinary hardware |

The nut's internal split (15 mm front nut, 9 mm rear nut, 5 mm free gap) is taken from the listings' overall sizes,
and the guide pins and spring are drawn to match. The scoring uses only the overall length, 25 mm pressed home, which a
lab measures on its own nut before the first trial (see Reset).

Links: Zyltech nut https://www.zyltech.com/8mm-lead-screw-trapezoidal-acme-brass-anti-backlash-nut-t8x8-or-t8x2/ and
notes https://www.zyltech.com/lead-screw--anti-backlash-nut/ ; ReliaBot https://www.amazon.com/dp/B079JQ7S3Y ;
Stelltron https://stelltron.co.za/product/t8-anti-backlash-spring-loaded-nut/ ; Micro Robotics
https://www.robotics.org.za/TR8X8-ABN-POM ; ACNTENY https://www.amazon.com/dp/B0HFM6PZM8 ; Novanta
https://www.novantaims.com/application-note/installing-anti-backlash-nut/ ; US9618104B2
https://patents.google.com/patent/US9618104B2/en ; Kerk CMP
https://www.haydonkerkpittman.com/products/lead-screws-and-nuts/lead-screws-and-nut-assemblies/cmp ; Helix Linear 2024
lead screw catalog, ATA nut table
https://9088664.fs1.hubspotusercontent-na1.net/hubfs/9088664/2024%20Lead%20Screw%20Assemblies%20Catalog-22APR25.pdf ; VXB motor https://vxb.com/products/nema-17-stepper-motor-with-tr8x8-100mm-lead-screw ;
StepperOnline AR4 J5 motor 17LS19-1684E-200G-AR4.

Props: motor with screw about $89 (any NEMA-17 with a TR8x8 screw cut to 100 mm works), nut $9, plate, standoffs and
screws about $10, a stepper driver and a 12 V supply about $15, a red paint marker. Total about $125. The motor is
powered and holding (driver enabled), as on an enabled axis. Its 0.44 N m holding torque is about seven times the nut's starting drag,
so the screw does not turn. Nothing is sharp; the pads touch only brass.

## Scoring (task.json has the same text)

Scored at the end of the trial (the run limit, or once 1.0 has held 1 s).

- Travel: how far the flange's underside has gone down from its start (138 mm above the table).
- Nut length: flange underside to the rear nut's top, with calipers. It is 29 mm with the spring free and 25 mm
  pressed home.
- Through: the screw's end stands level with or above the rear nut's top (within 0.1 mm), so both nuts are on the thread.

| Score | Condition |
|---|---|
| 1.0 | through, nut 25.5 mm long or less (nested: 8 N nominal, no play), flange underside at or below the red band's lower edge (32.75 mm of travel: the whole 1.5 mm band is hidden), both grippers 30 mm or more from the nut and the screw's top |
| 0.75 | through and 25.5 mm or less, but less travel than that or a gripper still near the nut |
| 0.5 | through, but longer than 25.5 mm (caught one or two thread starts out: 27 or 29 mm, 4 N or no preload) |
| 0.25 | not through, but the nut turned 8 mm or more down |
| 0 | anything else (doing nothing scores 0) |

In the simulator the nut length after the rear nut catches the thread is its caught spacing, so pressing the rear nut
down afterwards cannot raise a wrongly caught nut to 0.75. The 30 mm release check uses mechkit's `released()` on a box
round the nut.

## Fidelity

PhysX at 240 Hz. The screw and both nuts are real-size colliders. The thread is kinematic (mechkit `screw`): the front
nut runs 8 mm per turn against a drag of about 0.06 N m to start and 0.06-0.08 N m turning. The rear nut rides on the front nut on a 2,000 N/m spring, with its
stop at the nested position. Thread catch is a rule, run every control step (`_thread`). When the screw's end reaches
the rear nut, the nut takes the first thread start (2 mm apart) at or beyond where it is held, and its spring rest
moves there. It keeps that spacing until it is unthreaded past the screw's end. Simplified:

- the thread drag (about 0.06 N m to start, 0.06-0.08 N m turning) acts from the first turn and does not grow with the press. A real bare front nut may drag
  less: a TR8x8 thread (20 degree lead angle) is self-locking only above a friction of about 0.35, so check once per nut
  that it stays where it is left on the vertical screw
- the 8 N of preload follows from the caught spacing and a nominal 2 N/mm spring (our estimate; Zyltech gives 4 to 8 N
  without a spring rate)
- the catch is where the screw's end first meets the rear nut, worked out between two control steps. A press made later
  than about 14.5 mm of travel (0.5 mm after the nested position), however quick, catches one or two thread starts out
- the motor's holding torque is a fixed screw
- in the simulator a nut set on the screw's end cannot tip off it

## Files and running

- `task.py`: scene, the thread-catch rule, `score()`, `success()`, `demo_script(ctx)` (closed-loop scripted demo)
- `mechkit.py`, `closedloop.py`: copied unchanged from the accepted task plug-a-night-light-through-a-sliding-outlet-cover
  (MIT)
- `task.json`, this README

```
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --policy yam_scripted --instruction "<task.json instruction>" --scorer task_score --max-steps 3000
```

`ABN_DEBUG=1` prints the travel, nut length, catch and preload every 10 steps, and one line per grasp from the demo.
The layout is fixed (`object_jitter=0`); seeds change nothing. The demo reached full score in 1,339 steps (67 s of
simulated time; about 26 min wall on one GPU). The run limit is 3,000 steps.

Simulator run (scripted demo, `task_score: 1`, outcome "success", with the catch rule as above): each press brought the
rear nut home (3.7 to 4.0 mm of its 4 mm). It caught the thread nested at 14.2 mm of travel with 8 N. Its spacing stayed nested to the end, at
35.8 mm of travel with the red mark hidden, and both grippers let go.

Offline checks: `task_file.load` OK. `closedloop.dry_run`: 1,312 steps, worst IK 0.10 mm and 0.01 degrees (solver
tolerance; offline nothing moves, so the demo follows its own plan). An explicit IK check covers every live grip pose:
travel 0 to 37 mm, spring free or pressed, sag compensation 0 to 30 mm, press 0 to 35 mm and yaw -25 to 140 degrees,
worst 0.08 mm. Our scoring states and thread-catch timelines are right (rechecked after the catch rule was tightened) (pressed in time, never pressed,
pressed late, half pressed, released before the catch, unthreaded and redone).

## Reset

Nothing is consumed; a reset takes about 1 minute.

1. Turn the nut anticlockwise (seen from above) up and off the screw. The spring pushes the rear nut back out.
2. Set the nut flange down on the screw's end and turn it slowly anticlockwise until it drops onto the thread start.
   Leave it resting there, not turned on.

Once per nut: thread it on by hand pressed together, and measure its nested length. A lab whose nut nests at another
length scores against that length plus 0.5 mm.

## Credits and assets

All geometry is procedural (task.py, mechkit). There are no third-party meshes or textures. mechkit.py and
closedloop.py are copied unchanged from the accepted task plug-a-night-light-through-a-sliding-outlet-cover (MIT).
