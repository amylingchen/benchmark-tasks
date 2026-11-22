"""On the station table, right of centre: a lead-screw linear actuator standing upright, a NEMA-17 stepper with an
integrated TR8x8 lead screw (8 mm, 4 starts, 8 mm per turn) 100 mm long, its motor hanging under a 60 x 60 x 3 mm
aluminium plate on four 40 mm standoffs. A two-piece brass anti-backlash nut sits flange down on the screw's top end,
not yet turned on: the flanged front nut (22 mm flange, 10.2 mm body), a 12 mm compression spring round its body and
the 14 mm rear nut on top, kept from turning against the front nut by two guide pins. A red mark is painted round the
screw 34 mm below its top.

The spring pushes the two nuts apart. Pressed together (8 N), the rear nut meets the thread at the nested position, 1 mm
above the front nut's body (8 N of preload). Left loose, the screw's end lifts it to the next thread start, 2 mm or 4 mm
further out: 4 N or no preload.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score --max-steps 3000 ...
"""

import dataclasses
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402  (hooks yam_scripted to run demo_script)
from mechkit import Joint, Mech, box, cyl, on_reset, on_step, released, rod, screw, set_rest, sweep, tube  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
BRASS, STEEL, ALU = (0.80, 0.62, 0.30), (0.70, 0.71, 0.73), (0.83, 0.84, 0.86)
BLACK, RED, DARK = (0.08, 0.08, 0.09), (0.85, 0.10, 0.10), (0.25, 0.20, 0.12)


def _look(shape):
    """A look-only copy of a shape (no collider)."""
    return dataclasses.replace(shape, collide=False)


# -- the actuator (fixed). Frame: origin on the table under the screw axis, z up. -----------------------------------
AX, AY = 0.25, -0.12
PLATE_Z = 0.040  # standoffs 40 mm: the plate's underside and the motor's face
SCREW_TOP = PLATE_Z + 0.100  # TR8x8 screw, 100 mm out of the motor face
LEAD, PITCH = 0.008, 0.002  # 4 starts, 2 mm apart
Z0 = SCREW_TOP - 0.002  # flange underside at the start: the screw's end 2 mm into the nut, on the thread start
FRONT_L, REAR_L, GAP_FREE = 0.015, 0.009, 0.005  # front nut (3.5 flange + 11.5 body), rear nut, free gap between
REAR_OFF = FRONT_L + GAP_FREE  # rear nut's lower face above the flange underside, spring free
Q_NEST = -0.004  # rear nut pressed home: gap 1 mm, spring 4 mm shorter than free
SPRING_K = 2000.0  # N/m: one thread pitch (2 mm) of spacing changes the preload by 4 N (Zyltech: 4 to 8 N)
D_ENG = REAR_OFF - 0.002  # front-nut travel at which the screw's end reaches a free rear nut (less q when pressed)
MARK_D = 0.032  # travel that hides the red mark (flange underside at the mark)
TURN = 0.04  # N m: the nut's drag torque (Kerk CMP / Helix ATA 8 mm anti-backlash nuts: 5 oz-in max)

_rings = [cyl((0, 0, PLATE_Z + 0.001 + PITCH * i), 0.004, 0.0008, STEEL) for i in range(50)]  # thread crests
_LOOK = [*_rings, cyl((0, 0, Z0 - MARK_D), 0.0042, 0.0015, RED)]
_coil = [(0.0056 * math.cos(t), 0.0056 * math.sin(t), 0.0035 + 0.0165 * t / (12 * math.pi))
         for t in np.linspace(0.0, 12 * math.pi, 181)]
FRONT_TRAVEL, FRONT = screw("front", [
    tube((0, 0, 0.00175), 0.0045, 0.011, 0.0035, BRASS),  # flange 22 mm
    tube((0, 0, 0.00925), 0.0045, 0.0051, 0.0115, BRASS),  # body 10.2 mm
    *[_look(cyl((0.008 * math.cos(a), 0.008 * math.sin(a), 0.0036), 0.00175, 0.0003, DARK))
      for a in np.arange(4) * math.pi / 2],  # 4 x 3.5 mm holes on 16 mm
    _look(sweep((0, 0, 0), _coil, 0.00045, STEEL, sides=6))],  # spring 12 mm
    lead=LEAD, axis="z", at=(0, 0, Z0), low=-0.060, high=0.0, hold=TURN, mass=0.032, friction=0.6)
REAR = Joint("rear", "slide", (
    tube((0, 0, REAR_L / 2), 0.0045, 0.007, REAR_L, BRASS),
    *[_look(rod((0.0047 * s, 0.0047 * s, 0.0), (0.0047 * s, 0.0047 * s, -0.0185), 0.0006, STEEL))
      for s in (1, -1)]),  # guide pins
    axis="z", at=(0, 0, REAR_OFF), low=Q_NEST, high=0.0, spring=SPRING_K, damping=8.0, mass=0.012, parent="front",
    friction=0.6)
ACTUATOR = Mech("actuator", pos=(AX, AY, TOP), mass=None, friction=0.6, shapes=[
    box((0, 0, PLATE_Z + 0.0015), (0.060, 0.060, 0.003), ALU),
    *[cyl((sx * 0.025, sy * 0.025, PLATE_Z / 2), 0.0028, PLATE_Z, ALU) for sx in (1, -1) for sy in (1, -1)],
    box((0, 0, 0.0053), (0.0423, 0.0423, 0.008), ALU), box((0, 0, 0.0207), (0.0423, 0.0423, 0.0227), BLACK),
    box((0, 0, 0.036), (0.0423, 0.0423, 0.008), ALU),  # 42.3 mm motor, 38.7 mm long, end caps
    cyl((0, 0, PLATE_Z + 0.050), 0.0035, 0.100, STEEL),  # the screw's core
    *[_look(s) for s in _LOOK]],
    joints=[FRONT_TRAVEL, FRONT, REAR])
OBJECTS = [ACTUATOR]

# -- the rear nut meeting the thread ---------------------------------------------------------------------------------
# The rear nut cannot turn against the front nut (guide pins), so it can meet the 4-start thread only at spacings a
# 2 mm pitch apart. When the screw's end reaches it, it takes the first of those at or beyond where it is held: pressed
# home it stays nested; anywhere looser, the thread lifts it one or two pitches out. Once on the thread its spacing is
# fixed (its spring rest moves there); unthreaded until the screw's end leaves it, it is free again.


def _fresh():
    return {"on": False, "k": 0, "n": 0, "prev": None}


_S = _fresh()


def _index(q):
    """Thread starts beyond nested (0, 1 or 2) at which a rear nut held at q meets the thread."""
    return int(min(2, max(0, math.ceil((q - Q_NEST - 0.0005) / PITCH))))


@on_reset
def _reset(env=None):
    _S.clear()
    _S.update(_fresh())
    if env is not None:
        set_rest(env, "actuator", "rear", 0.0)


@on_step
def _thread(env):
    j = mechkit._mech_state(env, ACTUATOR).joints
    d, q = -j["front_travel"], j["rear"]
    if not _S["on"] and d >= D_ENG + q:
        # the catch is where the screw's end first met the rear nut: between the last control step and this one (a press
        # after that point, or a fast one, counts from where the rear nut then was)
        d0, q0 = _S["prev"] or (d, q)
        g0, g1 = d0 - D_ENG - q0, d - D_ENG - q
        # (already past the screw's end at the last step, as after a wrong catch: it cannot be pressed below that point)
        qc = q0 - g0 / (g1 - g0) * (q - q0) if g0 < 0 else max(q, d - D_ENG)
        _S.update(on=True, k=_index(qc))
        set_rest(env, "actuator", "rear", Q_NEST + PITCH * _S["k"])
    elif _S["on"] and d < D_ENG + Q_NEST + PITCH * _S["k"] - 0.001:
        _S.update(on=False, k=0)
        set_rest(env, "actuator", "rear", 0.0)
    _S["prev"] = (d, q)


# -- scoring ---------------------------------------------------------------------------------------------------------
def nut_state(objects):
    """The facts a person checks: travel (mm, from the start), nut length flange to rear top (mm), whether the screw's
    end stands level with or above the rear nut's top (both nuts on the thread), and the preload (N)."""
    j = objects["actuator"].joints
    d = -j["front_travel"]
    gap = GAP_FREE + (Q_NEST + PITCH * _S["k"] if _S["on"] else j["rear"])
    through = _S["on"] and d >= D_ENG + Q_NEST + PITCH * _S["k"] + REAR_L - 1e-4
    return {"travel": 1000 * d, "length": 1000 * (FRONT_L + gap + REAR_L), "through": through,
            "preload": SPRING_K * (GAP_FREE - gap) if _S["on"] else 0.0}


def _let_go(objects):
    z = Z0 - 0.065
    return released(objects["actuator"], lo=(-0.012, -0.012, z), hi=(0.012, 0.012, SCREW_TOP + 0.01))


def score(objects):
    """1.0: both nuts on the thread (the screw's end level with or above the rear nut's top), nested (nut 25.5 mm
    long or less: 8 N preload), turned down until the red mark is hidden (32.75 mm of travel) and let go (pads 3 cm off).
    0.75: on the thread and nested, but the mark still showing or the nut still held. 0.5: both on the thread but
    longer than 25.5 mm (one or two thread starts out: 4 N or no preload). 0.25: the front nut turned at least 8 mm on.
    Else 0."""
    s = nut_state(objects)
    if os.environ.get("ABN_DEBUG"):
        _debug(s)
    if s["through"]:
        if s["length"] <= 25.5:
            return 1.0 if s["travel"] >= 1000 * MARK_D + 0.75 and _let_go(objects) else 0.75  # past the band's lower edge
        return 0.5
    return 0.25 if s["travel"] >= 8.0 else 0.0


def success(objects):
    return score(objects) >= 1.0


def _debug(s):
    _S["n"] += 1
    if _S["n"] % 10 == 0:
        print(f"ABNDBG step {_S['n']} travel {s['travel']:.2f} length {s['length']:.2f} on {_S['on']} k {_S['k']} "
              f"through {s['through']} preload {s['preload']:.1f}N", flush=True)


# -- demo ------------------------------------------------------------------------------------------------------------
demo = placeholder_demo
YAW_HI, YAW_LO = math.radians(140.0), math.radians(-25.0)  # each grasp turns the nut 165 deg clockwise from above
DZ = LEAD * (YAW_HI - YAW_LO) / (2 * math.pi)  # 3.67 mm down per grasp
GRIP_UP = 0.003 + PAD_TIP_PAST_TCP  # TCP above the rear nut's lower face: pad tips 3 mm up its 9 mm
OPEN = -0.014  # pads 28 mm apart round the 14 mm rear nut
PRESS_EXTRA, PRESS_MAX = 0.006, 0.035  # past the point the rear nut bottoms; never more than 35 mm under the grip
PRESS_OFFLINE = 0.022  # offline (nothing moves): 4 mm of spring plus the arm's ~0.55 N/mm vertical compliance


def demo_script(ctx):
    """Right arm, pointing straight down. Each grasp: close on the rear nut, press it down onto the front nut until the
    spring is home (the arm's command goes about 20 mm under the nut: the joints are soft), turn the nut 165 degrees
    clockwise while following it down the lead, ease off, open and turn back. The rear nut meets the thread during the
    fourth grasp, held nested; ten grasps take the nut about 37 mm down, past the red mark."""
    R = ctx.right
    base = np.array([AX, AY, TOP])
    d_plan = 0.0

    def nut():
        """(travel, rear spacing) as measured; offline, where nothing moves, the plan's."""
        if ctx.offline:
            return d_plan, q_plan
        j = ctx.obj("actuator").joints
        return -float(j["front_travel"]), float(j["rear"])

    q_plan = 0.0
    yield from R.move(base + (0.0, 0.0, Z0 + 0.050), 2.0, OPEN, yaw=math.radians(60))
    above = base + (0.0, 0.0, Z0 + 0.045)
    yield from R.move(above, 1.0, OPEN, yaw=YAW_HI)
    yield from R.trim_to_measured(above)
    comp = R.pos - above  # the arm's sag, measured in free space
    for _ in range(13):
        d, q = nut()
        if d >= MARK_D + 0.0035:
            break
        grip = base + (0.0, 0.0, Z0 - d + REAR_OFF + q + GRIP_UP)
        yield from R.move(grip + comp + (0.0, 0.0, 0.004), 0.6, OPEN, yaw=YAW_HI)
        yield from R.move(grip + comp, 0.4, OPEN, yaw=YAW_HI)
        yield from R.trim_to_measured(grip, max_duration=0.5)
        comp = R.pos - grip
        yield from R.grip(FINGER_CLOSED, 0.5)
        yield from R.hold(0.2)
        press = PRESS_OFFLINE if ctx.offline else None
        depth = 0.0
        while press is None and depth < PRESS_MAX:  # down 1 mm a step until the spring is home
            depth += 0.001
            yield from R.move(grip + comp - (0.0, 0.0, depth), 0.05, yaw=YAW_HI)
            if nut()[1] <= Q_NEST + 0.0003:
                press = min(PRESS_MAX, depth + PRESS_EXTRA)
        press = PRESS_MAX if press is None else press
        if os.environ.get("ABN_DEBUG") and not ctx.offline:
            print(f"ABNDEMO step {ctx.step_count} travel {1000 * d:.2f} rear {1000 * nut()[1]:.2f} press "
                  f"{1000 * press:.1f} comp {np.round(1000 * comp, 1).tolist()}", flush=True)
        yield from R.move(grip + comp - (0.0, 0.0, press), 0.3, yaw=YAW_HI)
        yield from R.hold(0.2)
        yield from R.move(grip + comp - (0.0, 0.0, press + DZ), 2.0, yaw=YAW_LO)  # turn, following the lead
        yield from R.move(grip + comp - (0.0, 0.0, DZ), 0.4, yaw=YAW_LO)  # ease off before opening
        yield from R.grip(OPEN, 0.4)
        d_plan, q_plan = d_plan + DZ, (Q_NEST if d_plan + DZ >= D_ENG + Q_NEST else 0.0)
        yield from R.move(R.pos + (0.0, 0.0, 0.006), 0.8, OPEN, yaw=YAW_HI)
    yield from R.move(R.pos + (0.0, 0.0, 0.05), 1.0, OPEN, yaw=YAW_HI)
    yield from R.home()
