"""On the station table: an off-white PLA rectangular peg (100 x 40 x 26 mm) lying on its side in front of the
left arm, and a blue 3D-printed peg board (230 x 230 x 50 mm, nine shaped through-holes) clamped in front of the right
arm. The robot puts the peg fully into the board's rectangular hole (1.5 mm of clearance per side), through the board
down to the table: lying on its side, the peg has to be stood on end before it can go in. The instruction does not say
how; the scripted demo stands it up with the left gripper and the right gripper takes it by its top and puts it in.

Scoring, step by step (each step counts once reached): 0.25 grasped, a gripper alone has lifted the peg 3 cm off
the table; 0.5 handed over, after that the other gripper alone holds the peg off the table; 0.75 in the target hole,
the peg's lower end in the rectangular hole (centre within 5 mm, upright within 5 deg, at most 30 mm above the table);
1.0, ending the run, fully in (lower end within 10 mm of the table), released by both grippers and still for 1 s.
0 if the peg ends off the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=lying_peg.py --scorer task_score ...
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import partkit  # noqa: E402
from partkit import DOWN, UsdPart, Waypoint, X, Y, Z, finger_for_width, tilt_deg, yaw_xyzw  # noqa: E402
from yam_arena import demo as yam_demo  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix  # noqa: E402

partkit.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
ASSETS = HERE / "assets"
REST = 0.0005

# -- the peg (its own frame: the bottom face's centre at the origin, long axis +z) ------------------------------------
PEG_L, PEG_W, PEG_T = 0.100, 0.0403, 0.0259  # length (z), width (x), thickness (y)
PEG_MASS = 0.1287
# Lying: turned -90 deg about y, so its long axis lies along x with its bottom end forward (away from the robot), its
# 40 mm side stands up and its 26 mm sides face the arms' sides.
LYING_XYZW = (0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5))
PEG_START = np.array([0.40, 0.20])  # the bottom end's centre (the far end), on the table
PEG_Z = TOP + PEG_W / 2 + REST  # the lying peg's axis height

# -- the board ----------------------------------------------------------------------------------------------------------
BOARD_XY = np.array([0.24, -0.21])
RECT_HOLE = np.array([0.06626, 0.07031])  # in the board's frame (board yaw 0)
HOLE_XY = BOARD_XY + RECT_HOLE
BOARD_TOP = TOP + 0.050

PEG = UsdPart("peg", asset="peg", xy=tuple(PEG_START - (PEG_L / 2, 0.0)), extent=PEG_L, mass=PEG_MASS,
              usd_path=str(ASSETS / "peg_rectangle.usda"), pos=(*PEG_START, PEG_Z), rot_xyzw=LYING_XYZW)
BOARD = UsdPart("board", asset="board", xy=tuple(BOARD_XY), extent=0.23, mass=None,
                usd_path=str(ASSETS / "board.usda"), pos=(*BOARD_XY, TOP + REST), rot_xyzw=yaw_xyzw(0.0))
OBJECTS = [BOARD, PEG]

# -- scoring ----------------------------------------------------------------------------------------------------------
LIFTED = 0.03  # m: the peg's lowest point this high off the table
IN_XY, IN_TILT = 0.005, 5.0
IN_PARTLY = 0.030  # m: bottom at most this far above the table: it is in the hole (the board is 50 mm thick)
IN_FULLY = 0.010
FALLEN_Z = -0.02
GRIP_FORCE, GRIP_OPENING, GRIP_SLACK = 0.5, 0.003, 0.006


def _peg_axis(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))[:, 2]


def _lower_end(state) -> np.ndarray:
    """World position of the centre of the peg's lower end face (its two ends are the same)."""
    axis = _peg_axis(state)
    if axis[2] < 0:
        axis = -axis
    return np.asarray(state.pos, float) - axis * PEG_L / 2


def _upright_deg(state) -> float:
    """Angle (deg) between the peg's long axis and the vertical, either end up."""
    return float(np.degrees(np.arccos(min(1.0, abs(_peg_axis(state)[2])))))


def holds(objects, side: str) -> bool:
    """The gripper's TCP is within the peg's (grown) bounding box and both fingers press on something."""
    g = objects.get(f"{side}_gripper")
    if g is None:
        return False
    s = objects["peg"]
    lo, hi = np.asarray(s.low) - GRIP_SLACK, np.asarray(s.high) + GRIP_SLACK
    inside = bool(np.all(g.tcp >= lo) and np.all(g.tcp <= hi))
    return inside and g.contact >= GRIP_FORCE and g.opening >= GRIP_OPENING


def in_hole(objects, depth: float) -> bool:
    s = objects["peg"]
    b = _lower_end(s)
    return bool(np.linalg.norm(b[:2] - HOLE_XY) < IN_XY and _upright_deg(s) < IN_TILT and s.low[2] < TOP + depth)


class _Progress:
    """Steps reached this episode: ``grasped_by`` the gripper that first lifted the peg alone, ``handed_over`` once
    the other gripper alone has held it off the table after that, ``in_hole`` once the peg has been in the hole."""

    def __init__(self):
        self.t = -1.0
        self.reset()

    def reset(self):
        self.grasped_by = None
        self.handed_over = False
        self.entered = False

    def update(self, objects) -> None:
        t = float(objects.get("episode_time", 0.0))
        if t < self.t:
            self.reset()
        self.t = t
        s = objects["peg"]
        left, right = holds(objects, "left"), holds(objects, "right")
        alone = "left" if left and not right else "right" if right and not left else None
        if self.grasped_by is None and alone and s.low[2] > TOP + LIFTED:
            self.grasped_by = alone
            print(f"[lying_peg] t={t:.2f}  grasped and lifted by the {alone} gripper", flush=True)
        if (self.grasped_by and not self.handed_over and alone and alone != self.grasped_by
                and s.low[2] > TOP + HANDED_OFF_TABLE):
            self.handed_over = True
            print(f"[lying_peg] t={t:.2f}  handed over to the {alone} gripper", flush=True)
        if not self.entered and in_hole(objects, IN_PARTLY):
            self.entered = True
            print(f"[lying_peg] t={t:.2f}  in the target hole", flush=True)


HANDED_OFF_TABLE = 0.005  # m: the receiving gripper holds the peg with its lowest point this high off the table


_PROGRESS = _Progress()


def _released(objects) -> bool:
    return not holds(objects, "left") and not holds(objects, "right")


def success(objects) -> bool:
    _PROGRESS.update(objects)
    return bool(in_hole(objects, IN_FULLY) and _released(objects) and objects["peg"].resting())


def score(objects) -> float:
    _PROGRESS.update(objects)
    partkit.debug(objects, ["peg"], "lying_peg")
    s = objects["peg"]
    if s.pos[2] < TOP + FALLEN_Z:
        return 0.0
    if _PROGRESS.entered:
        return 0.75
    if _PROGRESS.handed_over:
        return 0.5
    return 0.25 if _PROGRESS.grasped_by else 0.0


def distance(objects) -> float:
    return float(np.linalg.norm(_lower_end(objects["peg"])[:2] - HOLE_XY))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left (mirroring the right arm's cell pick in the accepted flashlight-battery-insertion task): a grip tilted 50 deg
# forward across the lying peg's 26 mm faces, 30 mm in from its far (bottom) end; lift; stand it on end about the grip
# point, far end down, the hand ending up pointing toward the middle of the table and 50 deg down; carry it upright to
# the handover. Right: a straight-down grip on the standing peg's top 30 mm, fingers closing across the 26 mm faces;
# the left lets go and backs off; the right turns it a quarter turn to the hole's long side, carries it over the hole
# and pushes it in.
LEFT_GRIP_IN = 0.030  # m from the bottom end, along the peg
LEFT_GRIP_DEPTH = 0.012  # the left pad tips this far past the peg's axis
GRASP_TILT = math.radians(-50)
LEFT_PICK_YAW = 0.0
TIP_TURN = None  # set below: the peg's turn from lying to standing, in the world frame
RIGHT_GRIP_DOWN = 0.030  # right pad tips this far below the standing peg's top
RIGHT_TAKE_YAW = math.pi / 2  # closes along x, across the standing peg's 26 mm faces
RIGHT_PLACE_YAW = 0.0  # closes along y: the peg's 40 mm side along x, as the hole's
SQUEEZE, MARGIN = 0.007, 0.012  # about 35 N of grip (the YAM's fingers manage ~40 N)
HAND_XY = np.array([0.28, 0.03])  # the standing peg's axis during the handover
HAND_BOTTOM_Z = TOP + 0.0005  # the standing peg's bottom during the handover
LIFT_Z = TOP + 0.12  # the grip point's height while standing the peg up
TIP_STEPS = 4
DEMO_FIX = np.array([-0.0020, 0.0016])  # measured (runs v3, v4): where the held peg sits off its commanded place
ENTRY_DEPTH = 0.008  # the circling search ends with the peg's lower end this far into the hole
SEARCH_R, SEARCH_TURNS, SEARCH_STEPS, SEARCH_S = 0.0012, 2, 20, 4.0
PRESS_DEPTH = 0.004  # closed pad tips commanded this far into the seated peg's top

OPEN_T = finger_for_width(PEG_T, MARGIN)
HOLD_T = finger_for_width(PEG_T, -SQUEEZE)
LEFT_REACH = PAD_TIP_PAST_TCP - LEFT_GRIP_DEPTH  # the peg's axis this far out along the left hand's pointing axis


def _turn() -> np.ndarray:
    return axis_angle_matrix(Z, math.pi / 2) @ axis_angle_matrix(Y, math.pi / 2)


def left_grips() -> tuple[np.ndarray, np.ndarray]:
    pick = axis_angle_matrix(Y, GRASP_TILT) @ yam_demo.grip_rotation(LEFT_PICK_YAW)
    return pick, _turn() @ pick


def _tcp_at(point, rot: np.ndarray) -> np.ndarray:
    """The TCP that holds the peg's axis at ``point`` (the grip point) with the hand at ``rot``."""
    return np.asarray(point, float) - LEFT_REACH * (rot @ Z)


def left_plan(hand_s: float) -> list[Waypoint]:
    pick, upright = left_grips()
    grip_pt = np.array([PEG_START[0] - LEFT_GRIP_IN, PEG_START[1], PEG_Z])
    lift_pt = np.array([grip_pt[0], grip_pt[1], LIFT_Z])
    stand_pt = np.array([*HAND_XY, HAND_BOTTOM_Z + LEFT_GRIP_IN])
    plan = [
        Waypoint(2.0, tuple(_tcp_at(grip_pt, pick) - 0.05 * (pick @ Z)), OPEN_T, "over the peg", rot=pick),
        Waypoint(1.5, tuple(_tcp_at(grip_pt, pick)), OPEN_T, "down around it", rot=pick),
        Waypoint(0.8, tuple(_tcp_at(grip_pt, pick)), HOLD_T, "grip", rot=pick),
        Waypoint(1.5, tuple(_tcp_at(lift_pt, pick)), HOLD_T, "lift", rot=pick),
    ]
    for k in range(1, TIP_STEPS + 1):  # stand it on end about the grip point
        rot = ik.slerp_matrix(pick, upright, k / TIP_STEPS)
        plan.append(Waypoint(1.0, tuple(_tcp_at(lift_pt, rot)), HOLD_T, f"stand it on end {k}", rot=rot))
    at = _tcp_at(stand_pt, upright)
    back = at - 0.05 * (upright @ Z)  # straight back out along the hand's pointing axis
    plan += [
        Waypoint(2.5, tuple(at), HOLD_T, "to the handover", rot=upright),
        Waypoint(hand_s, tuple(at), HOLD_T, "hold it for the right arm", rot=upright),
        Waypoint(0.6, tuple(at), OPEN_T, "let go", rot=upright),
        Waypoint(1.5, tuple(back), OPEN_T, "back off", rot=upright),
        Waypoint(2.0, tuple(back + (0.0, 0.05, 0.06)), FINGER_OPEN, "clear", rot=upright),
    ]
    return plan


LEFT_READY_S = 2.0 + 1.5 + 0.8 + 1.5 + TIP_STEPS * 1.0 + 2.5  # until the peg stands at the handover


def right_plan() -> tuple[list[Waypoint], float]:
    top_z = HAND_BOTTOM_Z + PEG_L
    grip_tcp_z = top_z - RIGHT_GRIP_DOWN + PAD_TIP_PAST_TCP
    hx, hy = HAND_XY
    ix, iy = HOLE_XY + DEMO_FIX
    below = (PEG_L - RIGHT_GRIP_DOWN) + PAD_TIP_PAST_TCP  # TCP height above the peg's bottom
    travel = BOARD_TOP + 0.015 + below
    hover = BOARD_TOP + 0.010 + below
    seated = TOP + 0.002 + below
    a, b = RIGHT_TAKE_YAW, RIGHT_PLACE_YAW
    plan = [
        Waypoint(LEFT_READY_S - 1.0, (hx, hy - 0.08, grip_tcp_z + 0.03), OPEN_T, "beside the handover", a),
        Waypoint(1.5, (hx, hy, grip_tcp_z + 0.03), OPEN_T, "over the standing peg", a),
        Waypoint(1.2, (hx, hy, grip_tcp_z), OPEN_T, "down around its top", a),
        Waypoint(0.8, (hx, hy, grip_tcp_z), HOLD_T, "grip", a),
    ]
    taken = sum(w.duration for w in plan)
    up = max(grip_tcp_z, travel) + 0.01
    plan += [
        Waypoint(0.6 + 1.5, (hx, hy, grip_tcp_z), HOLD_T, "wait for the left to let go and back off", a),
        Waypoint(1.2, (hx, hy, up), HOLD_T, "lift", a),
        Waypoint(2.5, (ix, iy, up), HOLD_T, "carry, turning it a quarter turn", b),
        Waypoint(1.5, (ix, iy, hover), HOLD_T, "over the hole", b),
        Waypoint(1.0, (ix, iy, hover), HOLD_T, "settle", b),
    ]
    mouth = BOARD_TOP - ENTRY_DEPTH + below  # a small circling search into the hole's mouth, then straight down
    for i in range(1, SEARCH_STEPS + 1):
        f = i / SEARCH_STEPS
        a_ = 2 * math.pi * SEARCH_TURNS * f
        r = SEARCH_R * (1 - f)
        plan.append(Waypoint(SEARCH_S / SEARCH_STEPS, (ix + r * math.cos(a_), iy + r * math.sin(a_),
                                                        hover + f * (mouth - hover)), HOLD_T, "into the mouth", b))
    press = TOP + PEG_L + PAD_TIP_PAST_TCP - PRESS_DEPTH
    plan += [
        Waypoint(2.5, (ix, iy, seated), HOLD_T, "in", b),
        Waypoint(0.6, (ix, iy, seated), OPEN_T, "let go", b),
        Waypoint(1.2, (ix, iy, press + 0.025), OPEN_T, "up over it", b),
        Waypoint(0.5, (ix, iy, press + 0.025), 0.0, "close", b),
        Waypoint(1.2, (ix, iy, press), 0.0, "press it home", b),
        Waypoint(0.5, (ix, iy, press), 0.0, "hold", b),
        Waypoint(1.2, (ix, iy, press + 0.08), 0.0, "up", b),
        Waypoint(3.0, (ix, iy, press + 0.08), FINGER_OPEN, "wait", b),
    ]
    return plan, taken


def demo():
    right, taken = right_plan()
    hand_s = taken - LEFT_READY_S + 0.3
    return {"left": left_plan(hand_s), "right": right}


_ = (DOWN, X, Z, ik)
