"""On the station table, a piece of floor framing and a flat bar. Two 2x4 joists stand on edge, running away from the
robot, 16 in apart, joined at the back by a 2x4 rim joist that two C-clamps hold to the table's back edge. A 600 mm
1x4 board lies flat across the joists, held by one 3d box nail into each. In front of the joists, on the table, lies a
Crescent FB15-06 15 in flat pry bar (red steel, black rubber grip), parallel to the board.

The nails are the only thing the simulator cannot make from contact, so a rule stands in for the wood's grip (see
``Nails`` and ``_nail_joints``): each nail is a stiff spring on the board at that nail, along the joist's vertical, that
slips at the nail's withdrawal force, stays where it was left, and lets go when the nail's 12.7 mm are out. Everything else
is contact: the blade lifts the board's edge, the bar's bend bears on the joist, and the board may rise, tip end to end and
roll on its back edge.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<task.json instruction>" --scorer task_score --max-steps 4800 ...
"""

import dataclasses
import math
import os
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402  (hooks yam_scripted to run demo_script)
from mechkit import Mech, box, cyl, hull, on_reset, on_step, released  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
IN, MM = 0.0254, 0.001

# -- frame: 2x4 SPF (38.1 x 88.9 mm), joists 16 in on centre, rim joist across their back ends -----------------------
J_W, J_H = 1.5 * IN, 3.5 * IN
JT = TOP + J_H  # the joists' tops
NAIL_Y = {"left": 8 * IN, "right": -8 * IN}
# -- board: 600 mm of 1x4 E4E Eastern white pine (19.05 x 88.9 mm, four long edges rounded 1/8 in), 0.40 kg
B_W, B_T, B_L, B_R = 3.5 * IN, 0.75 * IN, 0.600, IN / 8
B_MASS = 0.40  # Eastern white pine, 394 kg/m3 (24.6 lb/ft3) x 1.016 dm3
FRONT = 0.365  # the board's front (robot side) face
B_AT = np.array([FRONT + B_W / 2, 0.0, JT + B_T / 2])
J_X = (FRONT - 0.075, 0.8619)  # joists: front end, back end against the rim
# -- nails: 3d box, 1-1/4 in x 0.076 in (Wood Handbook table 8-2): 12.7 mm in the joist
N_LEN, N_D, N_HEAD = 1.25 * IN, 0.076 * IN, 7 / 32 * IN
EMBED = round(N_LEN - B_T, 6)  # 12.7 mm
PULL = 54.12 * 0.42 ** 2.5 * (N_D / MM) * (EMBED / MM)  # N: Wood Handbook eq. 8-1a, SPF G 0.42: 152 N
K_PULL = 3.0e5  # N/m: elastic give of a nail before it slips, 0.5 mm at full load
M_NAIL = 690e6 * N_D ** 3 / 6  # N m: the nail's plastic bending moment (bending yield strength 100 ksi): 0.83 N m
K_NAIL = 200e9 * math.pi * N_D ** 4 / 64 / 0.010  # N m/rad: its bending stiffness over 10 mm: 13.6 N m/rad
BEND_N, BEND_K = M_NAIL / EMBED, 1.0e5  # N, N/m: sideways it bends above this force (65 N)
# -- bar: Crescent FB15-06 (maker's page: 15.0 in long, 1.5 in wide, 2.2 in high, 1.700 lb). Its side profile is traced
# from the maker's product photo at 381 mm = the photo's length (README): blade 47 mm, bent 16 deg off the shaft over a
# 42 mm arc, rubber grip from 128 to 299 mm, hook at the far end.
W, BAR_MASS = 1.5 * IN, 1.700 * 0.45359237
BEND, BLADE, ARC = math.radians(16), 0.047, 0.042
R_ARC = ARC / BEND
GRIP = (0.128, 0.299)  # along the bar from the blade's edge

WOOD, PINE, RED = (0.83, 0.70, 0.49), (0.93, 0.85, 0.67), (0.80, 0.16, 0.12)
BLACK, STEEL, BRIGHT, BLUE = (0.05, 0.05, 0.05), (0.32, 0.33, 0.35), (0.66, 0.67, 0.69), (0.10, 0.22, 0.45)


def _look(shape):
    return dataclasses.replace(shape, collide=False)


def _rot_y(a):  # raises the +x end for a > 0
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


def _profile():
    """(x, z, thickness) of the bar's underside from the blade's edge to the hook's end, the shaft lying along x."""
    cb, sb = math.cos(BEND), math.sin(BEND)
    edge = BLADE * sb + R_ARC * (1 - cb)
    pts = [(s * cb, edge - s * sb, t) for s, t in ((0, 0.6 * MM), (0.008, 3.0 * MM), (0.025, 3.8 * MM), (BLADE, 4.6 * MM))]
    for k in (1, 2, 3, 4):
        a = BEND * (1 - k / 4)
        pts.append((BLADE * cb + R_ARC * (sb - math.sin(a)), R_ARC * (1 - math.cos(a)), (4.6 + 0.35 * k) * MM))
    x0 = pts[-1][0] - BLADE - ARC  # straight length lost in the bend
    pts += [(GRIP[0] + x0, 0.0, 6 * MM), (GRIP[1] + x0, 0.0, 6 * MM)]
    hook = ((328, 0.3, 6), (338, 2.2, 6), (345, 6.0, 6), (351, 11.0, 6), (358, 15.5, 6), (364.5, 16.6, 6),
            (369.5, 14.5, 6), (372.5, 10.0, 6), (373.3, 4.0, 5.5), (373.6, -24.0, 5), (375, -35.0, 1.2))
    return pts + [(x * MM, z * MM, t * MM) for x, z, t in hook]


PROFILE = _profile()
X_GRIP = (PROFILE[8][0], PROFILE[9][0])


def _sections():
    """(underside, top) points of the profile, the top offset by the thickness along the vertex normal."""
    xz = np.array([p[:2] for p in PROFILE])
    d = np.diff(xz, axis=0)
    d /= np.linalg.norm(d, axis=1)[:, None]
    tang = np.vstack([d[:1], (d[:-1] + d[1:]), d[-1:]])
    tang /= np.linalg.norm(tang, axis=1)[:, None]
    return xz, xz + np.stack([-tang[:, 1], tang[:, 0]], 1) * np.array([p[2] for p in PROFILE])[:, None]


def _rest():
    """How it lies, hook down: tilt (hook end raised) and the height of the profile's origin."""
    xz, _ = _sections()
    best = None
    for a in np.radians(np.arange(0, 12, 0.005)):
        z = xz[:, 0] * math.sin(a) + xz[:, 1] * math.cos(a)
        gap = abs(z[:9].min() - z[-1])
        if best is None or gap < best[0]:
            best = (gap, float(a), float(-z.min()))
    return best[1:]


TILT, LIFT0 = _rest()


def bar_point(x, z, y=0.0):
    """A point of the profile's frame (shaft along x) in the bar's Mech frame (lying on the table, hook down)."""
    return _rot_y(TILT) @ np.array([x, y, z]) + [0.0, 0.0, LIFT0]


def _bar_shapes():
    lo, hi = _sections()
    out = []
    ext = lambda i, y: [bar_point(*lo[i], y), bar_point(*hi[i], y)]  # noqa: E731
    for i in range(len(lo) - 1):
        col = BRIGHT if i == 0 or i == len(lo) - 2 else RED
        if i < 2:  # the nail slot splits the blade's end into two prongs
            inner = (5 * MM, 3.6 * MM, 1 * MM)
            for s in (1, -1):
                pts = [p for k in (i, i + 1) for y in (s * W / 2, s * inner[k]) for p in ext(k, y)]
                out.append(hull((0, 0, 0), pts, col))
        elif i == 8:  # the rubber grip round the shaft: 12 mm thick
            pts = [bar_point(x, z, y) for x in X_GRIP for z in (-3 * MM, 9 * MM) for y in (-W / 2, W / 2)]
            out.append(hull((0, 0, 0), pts, BLACK))
        else:
            out.append(hull((0, 0, 0), [p for k in (i, i + 1) for y in (-W / 2, W / 2) for p in ext(k, y)], col))
    slot = [bar_point(x, z, y) for x, hw in ((0.062, 4 * MM), (0.092, 1.2 * MM)) for z in (5.5 * MM, 6.5 * MM) for y in (-hw, hw)]
    out.append(_look(hull((0, 0, 0), slot, (0.12, 0.03, 0.03))))  # the teardrop nail slot, look only
    return out


def _board_shapes():
    pts = []
    for sx, sz, a0, step in ((1, 1, 0, 45), (-1, 1, 90, 45), (-1, -1, 180, 10), (1, -1, 270, 10)):  # the bottom edges finer
        for a in range(0, 91, step):
            r = math.radians(a0 + a)
            pts += [(sx * (B_W / 2 - B_R) + B_R * math.cos(r), y, sz * (B_T / 2 - B_R) + B_R * math.sin(r)) for y in (-B_L / 2, B_L / 2)]
    out = [hull((0, 0, 0), pts, PINE)]
    for y in NAIL_Y.values():  # nails: heads flush (drawn 0.2 mm proud), shanks; look only, the rule holds them
        out.append(_look(cyl((0, y, B_T / 2 - 0.3 * MM), N_HEAD / 2, 1.0 * MM, STEEL)))
        out.append(_look(cyl((0, y, B_T / 2 - N_LEN / 2), N_D / 2, N_LEN, STEEL)))
    return out


def _frame_shapes():
    out = [box(((J_X[0] + J_X[1]) / 2, y, J_H / 2), (J_X[1] - J_X[0], J_W, J_H), WOOD) for y in NAIL_Y.values()]
    out.append(box((J_X[1] + J_W / 2, 0, J_H / 2), (J_W, 16 * IN + J_W, J_H), WOOD))
    for y in (-0.12, 0.12):  # 6 in C-clamps: spine behind the table's edge, arm, screw and pad on the rim
        out += [box((0.920, y, 0.035), (0.016, 0.022, 0.190), BLUE), box((0.900, y, J_H + 0.033), (0.056, 0.022, 0.016), BLUE),
                cyl((0.881, y, J_H + 0.015), 0.005, 0.022, BRIGHT), cyl((0.881, y, J_H + 0.002), 0.010, 0.004, BRIGHT)]
    return out


BAR_AT = np.array([0.185, 0.110, TOP + 0.001])  # the blade's edge end; the bar lies along y, hook to the right
FRAME = Mech("frame", pos=(0.0, 0.0, float(TOP)), mass=None, friction=0.4, shapes=_frame_shapes())
BOARD = Mech("board", pos=tuple(float(v) for v in B_AT), mass=B_MASS, friction=0.4, shapes=_board_shapes())
BAR = Mech("pry_bar", pos=tuple(float(v) for v in BAR_AT), yaw=-math.pi / 2, mass=BAR_MASS, friction=0.6, shapes=_bar_shapes())
OBJECTS = [FRAME, BOARD, BAR]


# -- the nails --------------------------------------------------------------------------------------------------------
class Nails:
    """How far each nail has slid out of its joist, and what it still holds: proportional to the length left in. A nail
    gives elastically (``K_PULL``) until the board pulls (or pushes) on it harder than that; then it slides."""

    def __init__(self):
        self.slid = {n: 0.0 for n in NAIL_Y}
        self.drawn = {n: 0.0 for n in NAIL_Y}  # the furthest it has slid
        self.out = {n: False for n in NAIL_Y}

    def hold(self, n):
        return 0.0 if self.out[n] else PULL * max(0.0, 1.0 - self.drawn[n] / EMBED)

    def watch(self, gaps):
        """Every physics step: a nail is out once the board has stood its 12.7 mm off the joist there (it stays out)."""
        for n, g in gaps.items():
            self.out[n] = self.out[n] or g >= EMBED

    def step(self, gaps):
        """Every control step: a nail slides when the board pulls (or pushes) it harder than it holds."""
        self.watch(gaps)
        for n, g in gaps.items():
            if not self.out[n]:
                give = self.hold(n) / K_PULL
                self.slid[n] = max(0.0, min(max(self.slid[n], g - give), g + give))
                self.drawn[n] = max(self.drawn[n], self.slid[n])


NAILS = Nails()
HZ = 240  # physics steps a second
HIST = deque(maxlen=HZ // 2 + 1)  # the board's position and orientation at every physics step of the last 0.5 s
_HOOK = {"scene": None, "skip": False, "freed": set()}
_JOINTS = mechkit.__dict__.setdefault("_nail_joint_paths", {})  # nail -> USD prim path (shared: the file is loaded twice)


def board_point(p, R, local):
    return np.asarray(p, float) + R @ np.asarray(local, float)


def nail_gaps(p, R):
    """Ruler at each nail: the board's underside above the joist's top."""
    return {n: float(board_point(p, R, (0, y, -B_T / 2))[2]) - JT for n, y in NAIL_Y.items()}


def _nail_joints(prim_path):
    """One D6 joint per nail, world to board, at the nail on the joist's top: along z a stiff spring to where the nail has
    slid, its force limited to the nail's hold, so it slips there; sideways and in tilt, springs limited to what bends the
    nail. It turns freely about z."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, Sdf, UsdPhysics

    stage = get_current_stage()
    for n, y in NAIL_Y.items():
        path = f"{prim_path}/nail_{n}"
        j = UsdPhysics.Joint.Define(stage, path)
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/base")])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(float(B_AT[0]), float(y), float(JT)))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, float(y), float(-B_T / 2)))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateCollisionEnabledAttr().Set(True)
        deg = math.pi / 180  # USD's angular drives are per degree
        for axis, k, c, f in (("transX", BEND_K, 200.0, BEND_N), ("transY", BEND_K, 200.0, BEND_N), ("transZ", K_PULL, 400.0, PULL),
                              ("rotX", K_NAIL * deg, 0.05 * deg, M_NAIL), ("rotY", K_NAIL * deg, 0.05 * deg, M_NAIL)):
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), axis)
            d.CreateTypeAttr().Set("force")
            d.CreateStiffnessAttr().Set(float(k))
            d.CreateDampingAttr().Set(float(c))
            d.CreateMaxForceAttr().Set(float(f))
            d.CreateTargetPositionAttr().Set(0.0)
            d.CreateTargetVelocityAttr().Set(0.0)
        _JOINTS.setdefault(n, path)


def _install_nails():
    plain = mechkit._author
    if getattr(plain, "_nails", False):
        return

    def author(prim_path, cfg, translation=None, orientation=None, **kw):
        prim = plain(prim_path, cfg, translation=translation, orientation=orientation, **kw)
        if cfg.mech_name == "board":
            _nail_joints(prim_path)
        return prim

    author._nails = True
    mechkit._author = author


_install_nails()


def _set_nail(n, force=None, slid=None, on=None):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    prim = get_current_stage().GetPrimAtPath(_JOINTS[n])
    if force is not None:
        UsdPhysics.DriveAPI(prim, "transZ").GetMaxForceAttr().Set(float(force))
    if slid is not None:
        UsdPhysics.DriveAPI(prim, "transZ").GetTargetPositionAttr().Set(float(slid))
    if on is not None:
        UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(bool(on))


def _np(v):
    return getattr(v, "torch", v).detach().cpu().numpy().astype(np.float64)


def _pose(env, name):
    base = env.unwrapped
    q = _np(base.scene[name].data.root_link_pose_w)[0]
    return q[:3] - _np(base.scene.env_origins)[0], q[3:7]


def _sample(env):
    """One look at the board (240 a second): is a nail's point out now; its pose, for stillness."""
    p, q = _pose(env, "board")
    NAILS.watch(nail_gaps(p, mechkit._quat_xyzw_matrix(q)))
    HIST.append((p, q))


@on_reset
def _new_trial(env):
    NAILS.__init__()
    HIST.clear()
    _HOOK.update(skip=False, freed=set())
    for n in NAIL_Y:
        _set_nail(n, force=PULL, slid=0.0, on=True)
    scene = env.unwrapped.scene
    if _HOOK["scene"] is scene:
        return
    write = scene.write_data_to_sim

    def write_data_to_sim(*a, **k):  # Isaac Lab calls this before every physics step
        if not _HOOK["skip"]:
            _sample(env)
        _HOOK["skip"] = False
        return write(*a, **k)

    scene.write_data_to_sim, _HOOK["scene"] = write_data_to_sim, scene


@on_step
def _draw(env):
    """Every control step: how far each nail has slid and what it still holds go to its joint; out, the joint lets go."""
    _sample(env)
    _HOOK["skip"] = True  # the next physics step starts from this same pose
    p, q = HIST[-1]
    NAILS.step(nail_gaps(p, mechkit._quat_xyzw_matrix(q)))
    for n in NAIL_Y:
        if not NAILS.out[n]:
            _set_nail(n, force=max(NAILS.hold(n), 1.0), slid=NAILS.slid[n])
        elif n not in _HOOK["freed"]:
            _HOOK["freed"].add(n)
            _set_nail(n, on=False)
            print(f"NAIL {n} out at step {mechkit.step_count()}", flush=True)


# -- scoring ----------------------------------------------------------------------------------------------------------
LOOSE = 6 * MM  # a nail is loosened when the board's underside is this far off the joist at it (or the nail is out)
FLAT, LOW, HIGH = math.radians(20), 10 * MM, 60 * MM  # laid flat: face off level, lowest and highest point above the table
STILL_M, STILL_DEG = 2.5 * MM, 1.5  # the most it may move and turn within the last 0.5 s
CLEAR = 0.03
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)
HALF = np.array([B_W, B_L, B_T]) / 2
BOARD_PTS = np.array(BOARD.shapes[0].hulls[0][0]) + BOARD.shapes[0].at  # the board's outline, edges rounded


def laid_flat(p, R):
    """On a broad face on the table top, whole, not propped up."""
    c = BOARD_PTS @ R.T + p
    z = c[:, 2] - TOP
    return bool(abs(R[2, 2]) >= math.cos(FLAT) and -5 * MM <= z.min() <= LOW and z.max() <= HIGH
                and TABLE_X[0] <= c[:, 0].min() and c[:, 0].max() <= TABLE_X[1]
                and TABLE_Y[0] <= c[:, 1].min() and c[:, 1].max() <= TABLE_Y[1])


BAR_HULLS = [np.array(s.hulls[0][0]) + s.at for s in BAR.shapes if s.collide]  # the bar's convex pieces, its frame


def _outline(xy):
    """Convex outline of points in the plane, counter-clockwise."""
    pts = sorted(map(tuple, np.round(xy, 9)))
    out = []
    for seq in (pts, pts[::-1]):
        part = []
        for q in seq:
            while len(part) > 1 and (part[-1][0] - part[-2][0]) * (q[1] - part[-2][1]) <= (part[-1][1] - part[-2][1]) * (q[0] - part[-2][0]):
                part.pop()
            part.append(q)
        out += part[:-1]
    return np.array(out)


def _apart(a, b):
    """Two convex outlines do not overlap: some edge of one has all of the other beyond it."""
    for u, v in ((a, b), (b, a)):
        e = np.roll(u, -1, 0) - u
        n = np.stack([e[:, 1], -e[:, 0]], 1)
        if ((v @ n.T).min(0) >= (u * n).sum(1)).any():
            return True
    return False


def clear_of_bar(p, R, bar_p, bar_R):
    """Seen from straight above, the outlines of the board and of the bar do not overlap (so neither lies on the other)."""
    board = _outline((BOARD_PTS @ R.T + p)[:, :2])
    return all(_apart(board, _outline((h @ bar_R.T + bar_p)[:, :2])) for h in BAR_HULLS)


def still(hist):
    """Between any two looks of the last 0.5 s (121 of them, 240 a second) the board's centre is under 2.5 mm apart and the
    board is turned by under 1.5 deg."""
    if len(hist) < HIST.maxlen:
        return False
    p, q = np.array([h[0] for h in hist]), np.array([h[1] for h in hist])
    far = np.linalg.norm(p[:, None] - p[None], axis=2).max()
    turn = math.degrees(2 * math.acos(min(1.0, float(np.abs(q @ q.T).min()))))
    return bool(far < STILL_M and turn < STILL_DEG)


def judge(p, R, bar_p, bar_R, out, let_go, at_rest):
    """The score of one moment; ``out``: nail -> has left its joist."""
    if not all(np.isfinite(v).all() for v in (p, R, bar_p, bar_R)):
        return 0.0
    g = nail_gaps(p, R)
    loose = sum(bool(out[n] or g[n] >= LOOSE) for n in NAIL_Y)
    if all(out.values()):
        return 1.0 if laid_flat(p, R) and clear_of_bar(p, R, bar_p, bar_R) and let_go and at_rest else 0.75
    return 0.25 * loose


def score(objects):
    b, bar = objects["board"], objects["pry_bar"]
    s = judge(b.origin, b.rotation(), bar.origin, bar.rotation(), NAILS.out,
              released(b, CLEAR, lo=tuple(-HALF), hi=tuple(HALF)), still(HIST))
    if os.environ.get("PRY_DEBUG"):
        _debug(objects, s)
    return s


def success(objects):
    return score(objects) >= 1.0


def _debug(objects, s):
    if mechkit.step_count() % 5:
        return
    b, bar = objects["board"], objects["pry_bar"]
    g, t, R = nail_gaps(b.origin, b.rotation()), bar.to_world(EDGE), bar.rotation()
    print(f"PRYDBG {mechkit.step_count()} s={s} gap L{1000 * g['left']:.1f} R{1000 * g['right']:.1f} drawn "
          f"L{1000 * NAILS.drawn['left']:.1f} R{1000 * NAILS.drawn['right']:.1f} out {int(NAILS.out['left'])}{int(NAILS.out['right'])} "
          f"board {np.round(1000 * (b.origin - B_AT), 1).tolist()} roll {math.degrees(math.asin(-b.rotation()[2, 0])):.2f} "
          f"tip {math.degrees(math.asin(b.rotation()[2, 1])):.2f} edge {np.round(1000 * (t - [FRONT, 0, JT]), 1).tolist()} "
          f"bar elev {math.degrees(math.asin(R[2, 0])) + math.degrees(TILT):.1f} roll {math.degrees(math.asin(R[2, 1])):.1f} "
          f"yaw {math.degrees(math.atan2(R[1, 0], R[0, 0])):.1f} still {int(still(HIST))} flat {int(laid_flat(b.origin, b.rotation()))}", flush=True)


# -- demo -------------------------------------------------------------------------------------------------------------
EDGE = bar_point(PROFILE[0][0], PROFILE[0][1])  # the middle of the blade's edge, underside
HOLD = bar_point(0.216, 3 * MM)  # where the hand holds the bar: its balance point, on the rubber grip
FLAT_E = BEND - TILT  # elevation of the bar's frame (hook end up) when the blade lies flat
demo = placeholder_demo


def demo_script(ctx):
    from demo import run

    yield from run(ctx)
