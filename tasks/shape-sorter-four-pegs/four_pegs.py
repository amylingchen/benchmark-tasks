"""On the station table: a blue 3D-printed peg board (230 x 230 x 50 mm, nine shaped through-holes) clamped midway
between the arms, and four off-white PLA pegs, each 100 mm tall, standing upright on the table beside it, two on each
arm's side: a rectangle (40 x 26 mm) and an oval (43 x 29 mm) on the left, a hexagon (41 mm across corners) and a
round peg (40 mm) on the right. Each peg fits only the hole of its own shape, with 1.5 mm of clearance per side.
The robot puts every peg fully into its hole (through the board, down to the table) without knocking any peg over.

Scoring: 0.25 for each peg fully in its own hole (centre within 5 mm of the hole's, upright within 5 deg, its bottom
within 10 mm of the table); the run ends with 1.0 once all four are in and still for 1 s. 0 for the whole trial if
any peg is ever knocked over (tilted more than 45 deg from upright, or off the table).

Run it:  inspect-robots run --embodiment yam_arena -E task=four_pegs.py --scorer task_score ...
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
from partkit import UsdPart, Waypoint, finger_for_width, tilt_deg, yaw_xyzw  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

partkit.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
ASSETS = HERE / "assets"
REST = 0.0005  # parts spawn this far above the table

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
BOARD_XY = np.array([0.26, 0.0])
PEG_H = 0.100
# name: (usd, mass kg, width across the grip m, start xy, start yaw, grip yaw, hole centroid in the board frame,
#        the peg's end centroid in its own frame, arm)
PEGS = {
    "peg_rectangle": ("peg_rectangle.usda", 0.1287, 0.0259, (0.150, 0.215), 0.0, 0.0, (0.06626, 0.07031), (0.0, 0.0),
                      "left"),
    "peg_oval": ("peg_oval.usda", 0.1211, 0.0288, (0.240, 0.215), 0.0, 0.0, (-0.0049, 0.07041), (0.0, 0.0), "left"),
    "peg_hexagon": ("peg_hexagon.usda", 0.1434, 0.0366, (0.150, -0.215), math.pi / 2, 0.0, (-0.07329, -0.06815),
                    (0.0, 0.0), "right"),
    "peg_round": ("peg_round.usda", 0.1582, 0.0403, (0.240, -0.215), 0.0, 0.0, (-0.0078, 0.0042), (0.0, 0.0),
                  "right"),
}
# Hexagon: it stands turned 90 deg, flats facing the arms' sides, so a straight grip closes on two flats; the hole
# has its flats facing front and back, so the wrist turns the peg back 90 deg on the way.
# The hexagon is symmetric under a half turn, so its hole yaw pi is the same hole: the right wrist turns +90 deg
# instead of -90 deg, which would take joint6 past its limit.
HOLE_YAW = {"peg_rectangle": 0.0, "peg_oval": 0.0, "peg_hexagon": math.pi, "peg_round": 0.0}

BOARD = UsdPart("board", asset="board", xy=tuple(BOARD_XY), extent=0.23, mass=None,
                usd_path=str(ASSETS / "board.usda"), pos=(*BOARD_XY, TOP + REST), rot_xyzw=yaw_xyzw(0.0))


def _peg(name: str) -> UsdPart:
    usd, mass, _, xy, yaw, *_ = PEGS[name]
    return UsdPart(name, asset=name, xy=tuple(xy), extent=0.05, mass=mass, usd_path=str(ASSETS / usd),
                   pos=(*xy, TOP + REST), rot_xyzw=yaw_xyzw(yaw))


OBJECTS = [BOARD] + [_peg(n) for n in PEGS]


def hole_xy(name: str) -> np.ndarray:
    """World xy of the peg's frame origin when it sits in its hole (board yaw 0)."""
    hole, end = PEGS[name][6], PEGS[name][7]
    return BOARD_XY + np.asarray(hole) - np.asarray(end)


def hole_centroid(name: str) -> np.ndarray:
    return BOARD_XY + np.asarray(PEGS[name][6])


# -- scoring ----------------------------------------------------------------------------------------------------------
IN_XY = 0.005  # m: the peg's end centroid within this of its hole's
IN_TILT = 5.0  # deg
IN_LOW = 0.010  # m: the peg's bottom within this of the table: it has gone through the 50 mm board
TOPPLED_TILT = 45.0  # deg: knocked over
FALLEN_Z = -0.02  # m above the table: off the table
POINTS = 0.25


def _end_centroid_xy(name: str, state) -> np.ndarray:
    """World xy of the peg's bottom-end centroid, from its bounding box (upright pegs) corrected for its offset."""
    from yam_arena.urdf import quat_xyzw_to_matrix

    rot = quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))
    centre = (np.asarray(state.low) + np.asarray(state.high)) / 2
    end = np.asarray([*PEGS[name][7], 0.0])
    return (centre + rot @ end)[:2]


def peg_in(name: str, objects) -> bool:
    s = objects[name]
    return bool(
        np.linalg.norm(_end_centroid_xy(name, s) - hole_centroid(name)) < IN_XY
        and tilt_deg(s) < IN_TILT
        and s.low[2] < TOP + IN_LOW
    )


def peg_toppled(name: str, objects) -> bool:
    s = objects[name]
    return bool(tilt_deg(s) > TOPPLED_TILT or s.pos[2] < TOP + FALLEN_Z)


class _Progress:
    """Remembers, per episode, whether any peg has been knocked over."""

    def __init__(self):
        self.t = -1.0
        self.toppled: set[str] = set()

    def update(self, objects) -> None:
        t = float(objects.get("episode_time", 0.0))
        if t < self.t:  # a new episode
            self.toppled = set()
        self.t = t
        for name in PEGS:
            if peg_toppled(name, objects):
                if name not in self.toppled:
                    print(f"[four_pegs] t={t:.2f}  {name} knocked over", flush=True)
                self.toppled.add(name)


_PROGRESS = _Progress()


def pegs_in(objects) -> list[str]:
    return [n for n in PEGS if peg_in(n, objects)]


def success(objects) -> bool:
    _PROGRESS.update(objects)
    return not _PROGRESS.toppled and len(pegs_in(objects)) == len(PEGS) and all(
        objects[n].resting() for n in PEGS)


def score(objects) -> float:
    _PROGRESS.update(objects)
    partkit.debug(objects, list(PEGS), "four_pegs")
    if _PROGRESS.toppled:
        return 0.0
    return POINTS * len(pegs_in(objects))


def distance(objects) -> float:
    """Summed xy distance (m) of the pegs not yet in from their holes."""
    return float(sum(np.linalg.norm(_end_centroid_xy(n, objects[n]) - hole_centroid(n)) for n in PEGS
                     if not peg_in(n, objects)))


# -- scripted demo ----------------------------------------------------------------------------------------------------
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import grip_rotation  # noqa: E402

GRIP_FROM_BOTTOM = 0.060  # pad tips this high up the peg
PAD = 0.013  # PAD_TIP_PAST_TCP
CARRY_CLEAR = 0.015  # the carried peg's bottom this far above the board
APPROACH_MARGIN = 0.012  # pads this much wider than the peg on each side while approaching
SQUEEZE = 0.007  # commanded this far inside the peg's half-width: about 35 N of grip (the YAM's fingers manage ~40 N)
ENTRY_DEPTH = 0.008  # the circling search ends with the peg's bottom this far into the hole
SEARCH_R, SEARCH_TURNS, SEARCH_STEPS, SEARCH_S = 0.0012, 2, 20, 4.0
PRESS_DEPTH = 0.004  # closed pad tips commanded this far into the seated peg's top
FINGER_CLOSED = 0.0
HOVER_ABOVE_HOLE = 0.010  # the peg's bottom this far above the board before going in
# Measured in run v4 (deterministic layout): where the held peg sits relative to its commanded place over the hole,
# cancelled when placing (m).
DEMO_FIX = {name: np.zeros(2) for name in PEGS}
DEMO_FIX["peg_hexagon"] = np.array([0.0017, 0.0019])


def _tcp_z(bottom_z: float) -> float:
    return bottom_z + GRIP_FROM_BOTTOM + PAD


def _peg_plan(name: str, start_wait: float) -> list[Waypoint]:
    _, _, width, xy, start_yaw, grip_yaw, *_ = PEGS[name]
    open_ = finger_for_width(width, APPROACH_MARGIN)
    hold = finger_for_width(width, -SQUEEZE)
    px, py = xy
    hx, hy = hole_xy(name) + DEMO_FIX[name]
    place_yaw = grip_yaw - start_yaw + HOLE_YAW[name]  # turns the peg from its start yaw to the hole's
    board_top = TOP + 0.050
    travel = _tcp_z(board_top + CARRY_CLEAR)  # clear of the board; the routes pass beside the other pegs
    above_peg = _tcp_z(TOP + PEG_H + 0.02 - GRIP_FROM_BOTTOM)  # pads over the peg's top
    grip = _tcp_z(TOP)
    hover = _tcp_z(board_top + HOVER_ABOVE_HOLE)
    seated = _tcp_z(TOP + 0.002)
    w = [Waypoint(start_wait, (px, py, max(above_peg, travel)), open_, f"{name}: over it", grip_yaw)] if start_wait \
        else []
    w += [
        Waypoint(1.5, (px, py, max(above_peg, travel)), open_, f"{name}: over it", grip_yaw),
        Waypoint(1.8, (px, py, grip), open_, f"{name}: down around it", grip_yaw),
        Waypoint(0.8, (px, py, grip), hold, f"{name}: grip", grip_yaw),
        Waypoint(1.5, (px, py, travel), hold, f"{name}: lift", grip_yaw),
        Waypoint(2.0, (hx, hy, travel), hold, f"{name}: carry", place_yaw),
        Waypoint(1.5, (hx, hy, hover), hold, f"{name}: over the hole", place_yaw),
        Waypoint(1.0, (hx, hy, hover), hold, f"{name}: settle", place_yaw),
    ]
    # Into the hole's mouth with a small circling search (the hole is 1.5 mm wider than the peg on each side), as a
    # person wiggles a tight peg in; then straight down.
    mouth = _tcp_z(board_top - ENTRY_DEPTH)
    for i in range(1, SEARCH_STEPS + 1):
        f = i / SEARCH_STEPS
        a = 2 * math.pi * SEARCH_TURNS * f
        r = SEARCH_R * (1 - f)
        w.append(Waypoint(SEARCH_S / SEARCH_STEPS, (hx + r * math.cos(a), hy + r * math.sin(a), hover + f * (mouth - hover)),
                          hold, f"{name}: into the mouth", place_yaw))
    top_seated = TOP + PEG_H  # the peg's top once it is down on the table
    press = top_seated + PAD - PRESS_DEPTH  # closed pad tips this far into the peg's top: pushes it down
    w += [
        Waypoint(2.5, (hx, hy, seated), hold, f"{name}: in", place_yaw),
        Waypoint(0.6, (hx, hy, seated), open_, f"{name}: let go", place_yaw),
        Waypoint(1.2, (hx, hy, press + 0.025), open_, f"{name}: up over it", place_yaw),
        Waypoint(0.5, (hx, hy, press + 0.025), FINGER_CLOSED, f"{name}: close", place_yaw),
        Waypoint(1.2, (hx, hy, press), FINGER_CLOSED, f"{name}: press it home", place_yaw),
        Waypoint(0.5, (hx, hy, press), FINGER_CLOSED, f"{name}: hold", place_yaw),
        Waypoint(1.2, (hx, hy, travel + 0.03), FINGER_CLOSED, f"{name}: up", place_yaw),
        Waypoint(0.4, (hx, hy, travel + 0.03), open_, f"{name}: open", place_yaw),
    ]
    return w


def _duration(plan) -> float:
    return sum(w.duration for w in plan)


def demo():
    """One peg at a time: the left arm puts in the rectangle, then the oval; then the right arm the hexagon, then the
    round peg. Each arm waits, raised over its own side, while the other works."""
    left = _peg_plan("peg_rectangle", 0.0) + _peg_plan("peg_oval", 0.0)
    left_s = _duration(left)
    right = _peg_plan("peg_hexagon", left_s + 0.5) + _peg_plan("peg_round", 0.0)
    last = left[-1]
    left.append(Waypoint(1.5, (0.20, 0.22, last.pos[2]), FINGER_OPEN, "out of the way", 0.0))
    left.append(Waypoint(_duration(right) - left_s - 1.5 + 3.0, (0.20, 0.22, last.pos[2]), FINGER_OPEN, "wait", 0.0))
    last = right[-1]
    right.append(Waypoint(1.5, (0.20, -0.22, last.pos[2]), FINGER_OPEN, "out of the way", 0.0))
    right.append(Waypoint(3.0, (0.20, -0.22, last.pos[2]), FINGER_OPEN, "wait", 0.0))
    return {"left": left, "right": right}


_ = grip_rotation
