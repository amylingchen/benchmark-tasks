# Pry a nailed 1x4 off two joists with a Crescent FB15-06 flat bar

Instruction given to the policy: `Pry the board off the joists with the pry bar and lay it flat on the table.`

A 600 mm length of 1x4 is nailed across two 2x4 joists, one 3d box nail into each: a light cleat of the kind framers
tack across joists to hold their spacing. A Crescent FB15-06 15 in flat pry bar lies on the table in front. The robot
works the bar's ground edge into the small rounding under the board's front edge at a joist, rests the bar's bend on
that joist, pushes the hook end down and levers the board up; it re-sets the blade deeper and levers again until the
nail is out, does the same at the other joist, lifts the loose board off and lays it flat on the table.

## Credit: this is a takeover

The idea, the scene's layout, the choice of nails, the 12.7 mm release and the score tiers come from
**`tasks/pry-board-off-joists`, pull request #2787 (TK-JBZFMC)**. Its last review (Nov 21, 2026) asked for one thing:
"specify the make and model of pry bar". We remodelled the bar from a real product, and wrote our own task file, nail
physics, scorer and demo; no code of that task is used. What differs, and why:

| | #2787 | Here | Why (source) |
|---|---|---|---|
| Pry bar | "any common 15 in flat bar": 380 mm, 32 x 6 mm, 0.57 kg, blade bent 15 deg | **Crescent FB15-06**: 381 mm, 38.1 mm wide, 55.9 mm high as it lies, 0.771 kg, rubber grip, blade bent 16 deg | the review; maker's page (below) |
| Bar's balance point, where the hand holds it | 200 mm from the edge | 216 mm | hook and width of the real bar |
| Board's long edges | 3 x 3 mm square step, as "S4S eased to about 1/8 in" | rounded 1/8 in (3.2 mm), an E4E board | The grading rule allows at most 1/16 in on 1 in nominal lumber (NELMA glossary 710(a)); 1x4 E4E is milled with 1/16 to 1/8 in (Timberline Lumber, "Square Edge Boards"), so the setup asks for E4E 1/8 in. A rounding leaves less room than a square step |
| Board | common pine, 0.46 kg | Eastern white pine, 0.40 kg | E4E is stocked in it (Timberline); 24.6 lb/ft3 (engineersedge.com, "Specific Gravity and Weight of Wood") |
| How the board may move while nailed | only straight up and end to end (a hidden carrier) | freely: it also rolls on its back edge | With a rigid board both nails can give a little at once; a carrier forbids it |
| A nail, sideways and in tilt | rigid | bends above 65 N / 0.83 N m | 689 MPa bending yield strength for nails under 3.61 mm (Rammer and Zelinka, USFS Treesearch 49146) x d^3 / 6 |
| Withdrawal force | 150 N | 152 N | the same formula without rounding |
| Laid flat: highest point | 104 mm above the table | 60 mm, and the board and the bar may not overlap seen from above | 104 mm let a board lying on the bar pass, which #2787's text excludes; on its nail points a board's high edge is at 44 mm |
| Still | speed under 10 mm/s | no two positions 2.5 mm or more apart, no two orientations 1.5 deg or more, in the last 0.5 s | can be read off the video |
| Joists' length | 612 mm, front end 106 mm before the board | 572 mm, 75 mm | room for the real bar's 12 mm thick grip below the joists' tops |

Kept, after checking the sources: 3d box nail 1-1/4 in x 0.076 in (Wood Handbook table 8-2); withdrawal p = 54.12 G^2.5
D L (N, mm), equation 8-1a (FPL-GTR-282 chapter 8, Treesearch 62253), with G 0.42 (spruce-pine-fir in the NDS) and 12.7 mm in the joist; resistance falling with the length still in;
score tiers 0.25 / 0.5 / 0.75 / 1.0; 16 in joist spacing; 6 mm for "loosened"; 20 deg for "flat" (nails down, the board
rests at 16.6 deg).

## The bar

Crescent FB15-06, "15 in Flat Pry Bar with Grip" (UPC 037103360717; 14.97 USD at Home Depot when read, Nov 22, 2026).
Maker's page (crescenttool.com, "15" Flat Pry Bar with Grip"): overall length 15.0 in, overall width 1.5 in, overall
height 2.2 in, weight 1.700 lb, alloy steel, powder paint, ground tip, three nail slots, over-molded grip. The page has
no drawing, so the side profile (bend, blade, grip, hook) is traced from the maker's product photo, scaled so the bar is
381 mm long. Two checks on the trace: lying hook down it is 56.0 mm high (maker: 55.9 mm), and its steel and rubber
volume weighs 0.75 kg (maker: 0.771 kg; the model uses 0.771). Traced: blade 47 mm, bent 16 deg off the shaft over a
42 mm arc; shaft 6 mm thick (a retailer, adamstarpntool.com, lists 0.236 in for the plain FB15); blade 4.6 mm thinning to
3 mm, its last 8 mm ground to a 0.6 mm edge (estimate); grip 171 mm long and 12 mm thick, 128 mm from the edge; hook
35 mm deep. Balance point 215 mm from the edge (computed).

Force at the hand. The bend bears 47 to 60 mm from the edge and the hand is 216 mm from it, so the bar gives about 3:1.
A fresh nail holds 152 N and the board's edge must be lifted with about 159 N (the far nail is the pivot), so the hand
pushes with **about 50 N** at the start of a stroke, falling as the nail comes out. That is more than
the arm's 2 kg nominal payload (i2rt.com), but the hand works 15 cm in front of its own base: 50 N there asks 8 N m of
joint 2, 12 to 15 N m of joint 3 (limit 27 N m) and 2.5 N m of the wrist (6 N m with 30 N forward at the same time; limit
7 N m). It is the arm's push, not the fingers' grip; the
fingers only must not slip down the rubber grip: 2 x 40 N needs a friction coefficient of 0.63. Holding 285 mm from the
edge instead lowers it to about 30 N.

## Why it matters

Stripping nailed lumber off framing with a flat bar is regular work for construction laborers (O*NET 47-2061.00: "Erect
or dismantle scaffolding, shoring, braces, traffic barricades, ramps, or other temporary structures"; "Position or
dismantle forms for pouring concrete, using saws, hammers, nails, or bolts"; "Raze buildings or salvage useful
materials") and carpenters (47-2031.00: "Remove damaged or defective parts or sections of structures and repair or
replace, using hand tools").

## Why it is hard

1. The board sits tight on the joist; the only way in without a hammer is the 3 mm rounding, under which the blade's
   edge catches by about 2 mm.
2. The bar rocks on its bend, so the fulcrum, the edge and the hand all move; pushed wrongly the edge slides out.
3. Strokes and re-setting: the first stroke lifts the board's edge a few millimetres, then the blade has to go in deeper.
4. Two nails, two joists, then a loose board that must come off and down flat.

## Files

| File | What |
|---|---|
| `task.py` | scene, nails, scorer, demo entry point |
| `demo.py` | the scripted demo, closed loop on the bar's and the board's poses |
| `task.json` | submission text |
| `mechkit.py`, `closedloop.py` | shared helpers, copied unchanged from the accepted task `plug-a-night-light-through-a-sliding-outlet-cover` (MIT) |

No meshes, scans or other third-party assets: every part is built from boxes, cylinders and convex pieces here. The
maker's photo was used for measuring only and is not included. The O*NET text is from O*NET OnLine (USDOL/ETA, CC BY 4.0).

## The nails

PhysX has no wood, so a rule stands in for its grip on a nail, and the robot still has to apply the real force through
the bar. At each nail a D6 joint ties the board to the joist's top. Along the vertical it is a spring (300 N/mm) to where
the nail has slid, its force limited to what the nail holds: 152 N x (1 - drawn / 12.7 mm). Every control step (20 Hz) the
rule moves the spring's rest to follow a nail that was pulled (or pushed) harder than that, and lowers the limit as the
nail comes out; at 12.7 mm the joint is switched off for good. Sideways and in tilt, springs limited to 65 N and
0.83 N m stand for the bending of the nail; it turns freely about its own axis. The rule never moves the board: it only
resists. Checked with a known force at the right nail: the board gives 0.33 mm at 100 N, 0.49 mm at 150 N, stays there,
and slides at 155 N (152 N plus its share of the board's weight). Nail heads and shanks are drawn but do not collide, so in
the simulator the loose board lies flush; a real one lies on its nail points at 16.6 deg, which the 20 deg limit allows.

Friction: board and joists 0.4, bar 0.6 everywhere, its rubber grip too (Engineering ToolBox: wood on wood 0.25-0.5, wood
on metal 0.2-0.6; rubber on rubber, 1.16, would help the hand); a contact takes the mean of its two values. The gripper's
pads are the benchmark's own.

Why the bar is needed (tried in the simulator: one or two hands pulling the nailed board up without it move it 0.1 mm and
score 0): without it, freeing one end means lifting that end's overhang with about 128 N (the far joist is the pivot), and
lifting the board level takes 300 N; an arm gives at most about 74 N there (27 N m at joint 2, 0.37 m away). Once one end is free, pulling the board up by that end is a normal technique and
counts.

## Scoring

At the end of the trial (4,800 steps, 4 minutes of simulated time), with a ruler, a photo from above and the last second of
video:

- **1.0**: both nails out, and the board lies flat on the table (a broad face within 20 deg of level, its lowest point at
  most 10 mm and its highest at most 60 mm above the table top, all of it over the table), not overlapping the bar seen
  from above, every finger of both grippers at least 3 cm from it, and still (within the last 0.5 s no two positions of its
  centre 2.5 mm or more apart and no two orientations 1.5 deg or more);
- **0.75**: both nails out, anything else of that missing;
- **0.5** / **0.25**: both / one of the nails loosened: at the nail, the board's underside at least 6 mm above the joist's
  top, or that nail out;
- **0** otherwise. A board that splits or breaks scores at most 0.75 (it cannot in the simulation).

Success is 1.0 held for 1 s. A nail is out once the board has stood at least 12.7 mm off the joist's top at it: its point has
left the joist, that end lifts with a fingertip, and it stays out if the board drops back. The simulator looks at the board
at every physics step, 240 times a second (nails out, still); on a real trial, compare video frames. The placement checks
use the board's real outline with its rounded edges.

## Run it

From this folder, with the benchmark's environment:

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 \
        -E cam_width=320 -E cam_height=176 --instruction "Pry the board off the joists with the pry bar and lay it flat on the table." \
        --scorer task_score --max-steps 4800 --store-frames

The Dream-1 run is the same command with `--policy dream`.

The layout is fixed (`object_jitter=0`); there are no seeds. The demo takes about 2,500 to 3,100 steps (2.1 to 2.6 minutes of
simulated time). Runtime, measured: about 2 minutes on a CPU and 4 to 8 on a shared GPU without cameras; the recorded run with
cameras every step took 20 min 36 s on a shared GPU (2,581 steps). `PRY_DEBUG=1` prints a trace line every 5 control steps.
