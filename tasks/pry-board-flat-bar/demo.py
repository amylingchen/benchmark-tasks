"""The scripted demo. Closed loop: every stage reads where the bar and the board are now (as a policy's perception
could); nothing is teleported or edited.

The right hand takes the bar by its grip, turns it blade forward, lays the blade flat on the right joist with its edge
in the rounding under the board's front edge and pushes the hook end down: the bend rocks on the joist and the edge of
the board rises. It re-sets the blade deeper and levers again until that nail is out, lays the bar between the joists,
and the left hand does the same at the left joist. Then the left hand puts the bar down outside the left joist and both
hands lift the loose board off and lay it flat on the table in front of the joists.
"""

import math

import numpy as np

import task as T
from closedloop import grip_rot
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

MM = 0.001
TOP, JT, FRONT = T.TOP, T.JT, T.FRONT
P0 = T.FLAT_E  # the hand takes the bar leaning this much toward the blade: upright once the blade lies flat
NOSE = math.radians(1.5)  # blade tipped edge down while it is brought to the joist
PULLED = T.EMBED + 1.0 * MM  # the board this far off the joist at a nail: that nail is out


class Scene:
    """Poses of the bar and the board now; offline (dry run) a held part follows the commanded hand."""

    def __init__(self, ctx):
        self.ctx, self.held, self.rest, self.lift = ctx, {}, {}, {"left": 0.0, "right": 0.0}
        self.drawn = {"left": False, "right": False}  # the board has stood the nail's 12.7 mm off the joist there

    def pose(self, name):
        if self.ctx.offline and name in self.held:
            A, dp, dR = self.held[name]
            return A.pos + A.rot @ dp, A.rot @ dR
        if self.ctx.offline and name in self.rest:
            return self.rest[name]
        s = self.ctx.obj(name)
        return np.asarray(s.origin, float), s.rotation()

    def hold(self, name, A):
        p, R = self.pose(name)
        self.held[name] = (A, A.rot.T @ (p - A.pos), A.rot.T @ R)

    def drop(self, name):
        self.rest[name] = self.pose(name)
        self.held.pop(name, None)

    def bar(self, local):
        p, R = self.pose("pry_bar")
        return p + R @ local

    def board(self, local):
        p, R = self.pose("board")
        return p + R @ np.asarray(local, float)

    def gap(self, side):
        g = self.lift[side] if self.ctx.offline else T.nail_gaps(*self.pose("board"))[side]
        self.drawn[side] = self.drawn[side] or g >= T.EMBED - 0.1 * MM
        return g

    def edge_up(self, side):
        """The board's front bottom edge above the joist, at the nail's end."""
        if self.ctx.offline:
            return self.lift[side]
        return float(self.board((-T.B_W / 2, T.NAIL_Y[side], -T.B_T / 2))[2]) - JT


def frame(psi, e):
    """The bar's orientation: hook end along ``psi`` (about z) and raised by ``e``."""
    x = np.array([math.cos(psi) * math.cos(e), math.sin(psi) * math.cos(e), math.sin(e)])
    y = np.array([-math.sin(psi), math.cos(psi), 0.0])
    return np.stack([x, y, np.cross(x, y)], 1)


def turned(D):
    return math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(D)) - 1) / 2))))


def say(S, A, text):
    e = S.bar(T.EDGE) - [FRONT, 0, JT]
    print(f"[demo] {A.side} {text}: edge {np.round(1000 * e, 1).tolist()} cmd {np.round(A.pos, 3).tolist()} seen "
          f"{np.round(A.measured_pos, 3).tolist()} gaps L{1000 * S.gap('left'):.1f} R{1000 * S.gap('right'):.1f}", flush=True)


def goto(A, pos, t, rot, finger=None):
    """Move, then once more with the hand's measured sag taken out."""
    pos = np.asarray(pos, float)
    yield from A.move(pos, t, finger, rot=rot)
    yield from A.hold(0.3)
    err = pos - A.measured_pos
    if np.linalg.norm(err) < 0.04:
        yield from A.move(A.pos + err, 0.5, rot=rot)
        yield from A.hold(0.2)


def steer(S, A, edge, psi, e, t, fixes=2):
    """Bring the held bar's blade edge to ``edge`` with the bar at (psi, e), then correct on the bar's measured pose."""
    for k in range(fixes + 1):
        p, R = S.pose("pry_bar")
        now, D = p + R @ T.EDGE, frame(psi, e) @ R.T
        err = np.asarray(edge, float) - now
        if k and np.linalg.norm(err) < 1.0 * MM and turned(D) < 1.0:
            break
        yield from A.move(now + err + D @ (A.pos - now), t if k == 0 else 0.5, rot=D @ A.rot)
        yield from A.hold(0.25)


def take_bar(S, A, yaw):
    rot = grip_rot(yaw=yaw, pitch=P0)
    at = S.bar(T.HOLD) + [0, 0, 0.004]
    yield from goto(A, at + [0, 0, 0.07], 2.0, rot, FINGER_OPEN)
    d = A.pos - A.measured_pos  # sag there
    yield from A.move(at + d, 1.2, rot=rot)
    yield from A.grip(FINGER_CLOSED, 0.6)
    yield from A.hold(0.4)
    S.hold("pry_bar", A)


def lever(S, A, side, done, push, most=math.radians(45), rate=math.radians(8)):
    """Rock the bar back on its bend, hook end down, a little each step, until ``done()`` or until the edge slips out
    from under the board. The commanded motion rolls the bend along the joist; ``push`` keeps the edge in."""
    heel = S.bar(T.bar_point(T.PROFILE[3][0], T.PROFILE[3][1]))
    p0, r0, a, ok = A.pos.copy(), A.rot.copy(), 0.0, False
    while a < most and not ok and (S.ctx.offline or under(S, side)):
        a = min(most, a + rate * A.dt)
        Q, b = T._rot_y(a), min(a, T.BEND)
        roll = np.array([push * min(1.0, a / 0.05) - T.R_ARC * (b - math.sin(b)), 0, T.R_ARC * (1 - math.cos(b))])
        A._set_cartesian(heel + Q @ (p0 - heel) + roll, Q @ r0, A.finger)
        if S.ctx.offline:
            S.lift[side] += 0.02 * A.dt
        yield
        ok = done()
    in_place = S.ctx.offline or under(S, side)
    say(S, A, f"levered {math.degrees(a):.1f} deg at {side}: edge of board {1000 * S.edge_up(side):.1f} up, "
              f"{'out' if ok else 'not yet' if in_place else 'slipped'}")
    if in_place:
        back = A.move(p0, 0.9, rot=r0)  # hook end back up: the nails hold the board where it is
    else:
        back = A.move(A.measured_pos + [-0.045, 0, 0], 0.8, rot=A.measured_rot)
    for _ in back:
        S.gap(side)  # a nail may let go now
        yield


def under(S, side):
    """The blade's edge is still under the board's front edge."""
    e = S.bar(T.EDGE)
    return e[0] > FRONT - 0.5 * MM and e[2] - JT < S.edge_up(side) + 4 * MM


def set_blade(S, A, side, depth, lead=0.014):
    """Blade flat on the joist, its edge just in front of the board (or still under it), then straight in under the
    board, the hand leading by ``lead``, until it stops or is ``depth`` in."""
    y = T.NAIL_Y[side]
    if not (under(S, side) and S.bar(T.EDGE)[0] > FRONT + 1 * MM) or S.ctx.offline:
        if np.linalg.norm(S.bar(T.EDGE) - [FRONT, y, JT]) > 0.020:
            yield from steer(S, A, (FRONT - 0.030, y, JT + 0.020), math.pi, T.FLAT_E + NOSE, 1.0, fixes=1)
        yield from steer(S, A, (FRONT - 0.008, y, JT + 0.003), math.pi, T.FLAT_E + NOSE, 0.8)
        yield from A.move(A.pos + [0, 0, -0.006], 0.5, rot=A.rot)
    best, idle = -1.0, 0
    for _ in range(60):
        x = S.bar(T.EDGE)[0] - FRONT
        idle, best = (idle + 1, best) if x < best + 0.1 * MM else (0, x)
        if x >= depth or idle > 12:
            break
        step = np.clip(A.measured_pos[0] + lead - A.pos[0], -0.0015, 0.0015)
        A._set_cartesian(A.pos + [0.0015 if S.ctx.offline else step, 0, 0], A.rot, A.finger)
        yield
    say(S, A, "blade in")


def pry(S, A, side):
    """Stroke by stroke: the edge in the rounding lifts the board's edge 2 mm, past it 9 mm, deep under it the nail
    comes out."""
    loose = lambda: S.gap(side) >= PULLED  # noqa: E731
    yield from steer(S, A, (FRONT - 0.030, T.NAIL_Y[side], JT + 0.045), math.pi, T.FLAT_E + NOSE, 2.5)
    for _ in range(10):
        yield from set_blade(S, A, side, 0.035)
        depth = S.bar(T.EDGE)[0] - FRONT
        want = 2 * MM if depth < 2.5 * MM else 9 * MM if depth < 12 * MM else None
        yield from lever(S, A, side, (lambda: loose() or S.edge_up(side) >= want) if want else loose, 0.012 if want else 0.0)
        if S.drawn[side]:
            break
    say(S, A, f"{side} nail done")


def draw_out(S, A):
    """The blade straight out from under the board (the other hand holds it), then up."""
    yield from A.move(A.pos + [-0.080, 0, 0.002], 1.8, rot=A.rot)
    yield from A.move(A.pos + [0, 0, 0.020], 0.8, rot=A.rot)


def lay_bar(S, A, edge, x):
    """Put the held bar down on the table, hook down, blade forward: down at ``x``, then forward to ``edge``."""
    edge = np.asarray(edge, float)
    yield from steer(S, A, (x, edge[1], JT + 0.055), math.pi, 0.0, 2.5, fixes=1)
    yield from steer(S, A, (x, edge[1], TOP + T.EDGE[2] + 0.010), math.pi, 0.0, 2.0, fixes=1)
    yield from steer(S, A, (edge[0], edge[1], TOP + T.EDGE[2] + 0.004), math.pi, 0.0, 1.2, fixes=2)
    yield from A.move(A.pos + [0, 0, -0.004], 0.4, rot=A.rot)
    yield from A.grip(FINGER_OPEN, 0.6)
    S.drop("pry_bar")
    yield from A.move(A.pos + [0, 0, 0.09], 1.2, rot=A.rot)


HANDS = {"left": 0.150, "right": -0.150}  # along the board: where each hand takes it, inside its joist


def take_board(S, A):
    """One hand across the board's width, from above."""
    at = lambda up: S.board((0, HANDS[A.side], 0)) + [0, 0, up]  # noqa: E731
    p, R = S.pose("board")
    rot = grip_rot(yaw=math.atan2(R[1, 0], R[0, 0]) + (math.pi / 2 if A.side == "right" else -math.pi / 2))
    yield from goto(A, at(0.028), 2.5, rot, FINGER_OPEN)
    d = A.pos - A.measured_pos
    yield from A.move(at(0.028) + d, 0.4, rot=rot)
    yield from A.move(at(0.004) + d, 1.2, rot=rot)
    yield from A.grip(FINGER_CLOSED, 0.8)
    yield from A.hold(0.3)


def leave_board(A):
    yield from A.grip(FINGER_OPEN, 0.8)
    yield from A.move(A.pos + [0, 0, 0.030], 0.8, rot=A.rot)
    yield from A.home(2.0)


def lay_board(S, ctx):
    """Both hands: up, toward the robot, down on the table in front of the joists, let go."""
    def shift(v, t):
        v = np.asarray(v, float)
        yield from ctx.both(ctx.left.move(ctx.left.pos + v, t, rot=ctx.left.rot), ctx.right.move(ctx.right.pos + v, t, rot=ctx.right.rot))

    S.hold("board", ctx.right)
    yield from shift((0, 0, 0.022), 1.2)
    x = float(S.board((0, 0, 0))[0])
    yield from shift((-0.07, 0, 0.012), 1.2)
    yield from shift((0.200 - x + 0.07, 0, 0), 2.0)
    z = float(S.board((0, 0, 0))[2])
    yield from shift((0, 0, TOP + T.B_T / 2 + 0.003 - z), 2.5)
    z = float(S.board((0, 0, 0))[2])
    yield from shift((0, 0, TOP + T.B_T / 2 - z), 0.6)
    S.drop("board")
    yield from ctx.both(leave_board(ctx.left), leave_board(ctx.right))


def run(ctx):
    S, L, R = Scene(ctx), ctx.left, ctx.right
    yield from take_bar(S, R, math.pi / 2)
    yield from R.move(R.pos + [0, 0, 0.10], 2.0, rot=R.rot)
    yield from steer(S, R, (FRONT - 0.045, T.NAIL_Y["right"], JT + 0.060), math.pi, 0.0, 4.0, fixes=1)
    yield from pry(S, R, "right")
    yield from take_board(S, L)  # the free hand holds the board while the bar is drawn out
    yield from draw_out(S, R)
    yield from leave_board(L)
    yield from lay_bar(S, R, (0.400, 0.04), FRONT - 0.025)
    yield from R.home(2.0)
    yield from take_bar(S, L, 0.0)
    yield from steer(S, L, (FRONT - 0.025, 0.04, TOP + T.EDGE[2] + 0.010), math.pi, 0.0, 1.2, fixes=1)
    yield from steer(S, L, (FRONT - 0.025, 0.04, JT + 0.045 + T.EDGE[2]), math.pi, 0.0, 2.0, fixes=1)
    yield from steer(S, L, (FRONT - 0.035, T.NAIL_Y["left"], JT + 0.055), math.pi, 0.0, 2.0, fixes=1)
    yield from pry(S, L, "left")
    yield from take_board(S, R)
    yield from draw_out(S, L)
    yield from lay_bar(S, L, (0.480, 0.350), 0.480)  # outside the left joist, well in front of the arm's own base
    yield from L.home(2.0)
    yield from take_board(S, L)
    yield from lay_board(S, ctx)
