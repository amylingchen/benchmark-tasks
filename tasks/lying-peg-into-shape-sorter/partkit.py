"""Shared helpers for this task file: USD parts at a full pose, gripper states for scoring, step and reset hooks, and
scripted waypoints with full gripper rotations. Adapted from two accepted tasks, which extend the stock task-file
format without changing yam_arena: ``flashlight-battery-insertion`` (``UsdPart``, the scene patch, ``GripperState``
and ``Waypoint.rot``) and ``snapfit-close`` (``on_step``, ``on_reset`` and ``write_pose``).

Pure Python until the simulator imports ``yam_arena.scene``: the patch is applied right after that import.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP
from yam_arena.task_file import Asset
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

# -- parts ------------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned as one rigid body at a full pose, with the physics authored in it (a part whose USD sets
    ``physics:kinematicEnabled`` stays where it is put). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


def yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _spawn(scene, spec: UsdPart):
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                rotation_xyzw=tuple(float(v) for v in spec.rot_xyzw)),
    )


# -- grippers ---------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class GripperState:
    """One gripper after a control step (world frame, m): its TCP, its pointing axis, how far apart the pads are, and
    the smaller of the two fingers' grip forces (N)."""

    tcp: np.ndarray
    axis: np.ndarray
    opening: float
    contact: float


def _gripper_states(env) -> dict:
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {"episode_time": float(base.episode_length_buf[0]) * float(base.step_dt)}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        axis = runtime.link_poses(robot)["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)
        out[f"{side}_gripper"] = GripperState(
            tcp=np.asarray(tcp, float),
            axis=np.asarray(axis, float),
            opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
        )
    return out


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        return {**stock_states(env, names), **_gripper_states(env)}

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states


_STEP: list = []
_RESET: list = []


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset."""
    _RESET.append(fn)
    return fn


def write_pose(environment, name: str, pos, quat_xyzw, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)) -> None:
    """Teleport a part's body (env 0) to a world pose (relative to the env origin) with the given velocity."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = base.scene.env_origins[0]
    origin = origin.torch if hasattr(origin, "torch") else origin
    pose = torch.tensor([*map(float, pos), *map(float, quat_xyzw)], dtype=origin.dtype, device=origin.device)[None]
    pose[:, :3] += origin
    asset.write_root_pose_to_sim_index(root_pose=pose)
    v = torch.tensor([*map(float, lin_vel), *map(float, ang_vel)], dtype=origin.dtype, device=origin.device)[None]
    asset.write_root_velocity_to_sim_index(root_velocity=v)


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_partkit_hooks", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._partkit_hooks = True


class _AfterSceneImport(importlib.abc.MetaPathFinder):
    """Patches ``yam_arena.scene`` and ``yam_arena.inspect_embodiment`` right after each is first imported."""

    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        run, patch = spec.loader.exec_module, self.targets[fullname]

        def exec_module(module):
            run(module)
            patch(module)

        spec.loader.exec_module = exec_module
        return spec


def install() -> None:
    for name, patch in _AfterSceneImport.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _AfterSceneImport) for f in sys.meta_path):
        sys.meta_path.insert(0, _AfterSceneImport())


# -- geometry helpers -------------------------------------------------------------------------------------------------


def up_axis(state) -> np.ndarray:
    """The part's own +z axis in the world."""
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))[:, 2]


def tilt_deg(state) -> float:
    """Angle (deg) between the part's own +z and the world vertical."""
    return float(np.degrees(np.arccos(np.clip(up_axis(state)[2], -1.0, 1.0))))


# -- scripted waypoints with full rotations ---------------------------------------------------------------------------


@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None  # a full gripper (TCP) rotation; replaces ``yaw`` when set


def _target(self, t: float):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = w.rot if getattr(w, "rot", None) is not None else yam_demo.grip_rotation(w.yaw)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (
                prev_pos + s * (goal - prev_pos),
                ik.slerp_matrix(prev_rot, rot, s),
                prev_finger + s * (w.finger - prev_finger),
                w.label,
            )
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


yam_demo.ArmPlan.target = _target

DOWN = quat_xyzw_to_matrix(np.asarray(yam_demo.DOWN_GRIP))
X, Y, Z = np.eye(3)


def side_grip(approach: float, roll: float = 0.0) -> np.ndarray:
    """The gripper pointing horizontally along ``approach`` (rad from +x), fingers closing horizontally at roll 0,
    turned ``roll`` about its pointing axis."""
    forward = axis_angle_matrix(Y, -math.pi / 2) @ DOWN
    return axis_angle_matrix(Z, approach) @ axis_angle_matrix(X, roll) @ forward


def finger_for_width(width: float, margin: float) -> float:
    """Finger joint target (m) that leaves the pads ``width + 2 * margin`` apart (negative margin squeezes)."""
    return float(np.clip(-(width / 2 + margin), FINGER_OPEN, 0.0))


__all__ = [
    "PAD_TIP_PAST_TCP",
    "DOWN",
    "GripperState",
    "UsdPart",
    "Waypoint",
    "X",
    "Y",
    "Z",
    "finger_for_width",
    "install",
    "on_reset",
    "on_step",
    "side_grip",
    "write_pose",
    "tilt_deg",
    "up_axis",
    "yaw_xyzw",
]


# -- debugging --------------------------------------------------------------------------------------------------------
import os as _os

_DEBUG_EVERY = 0.5
_last_debug = [-1.0]


def debug(objects, names, tag: str) -> None:
    """With FMB_DEBUG=1, print each part's position, tilt and the grippers every 0.5 s of episode time."""
    if not _os.environ.get("FMB_DEBUG"):
        return
    t = float(objects.get("episode_time", 0.0))
    if t < _last_debug[0]:
        _last_debug[0] = -1.0
    if t - _last_debug[0] < _DEBUG_EVERY:
        return
    _last_debug[0] = t
    parts = " ".join(
        f"{n}=({objects[n].pos[0]:.3f},{objects[n].pos[1]:.3f},{objects[n].low[2]:.3f}|{tilt_deg(objects[n]):.0f}d)"
        for n in names)
    grips = " ".join(
        f"{s}:tcp=({g.tcp[0]:.3f},{g.tcp[1]:.3f},{g.tcp[2]:.3f}) open={g.opening*1000:.0f}mm F={g.contact:.0f}"
        for s in ("left", "right") if (g := objects.get(f"{s}_gripper")) is not None)
    print(f"[{tag} dbg] t={t:.1f} {parts} {grips}", flush=True)
