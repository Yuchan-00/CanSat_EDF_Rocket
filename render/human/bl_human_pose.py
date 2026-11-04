"""Blender-side helper: put a pose solved by ``tools/human_figure.py`` (the numpy runtime the PyVista videos use) onto
``human_rig`` in ``human_rigged.blend``, so a Blender render shows exactly the pose of the PyVista video frame.

Usage inside Blender (Python, numpy is bundled with Blender; nothing else is needed):

    import sys; sys.path[:0] = [r"<project>/render/human", r"<project>/tools"]
    import bl_human_pose as BHP, human_figure as HF
    poser = BHP.Poser()                                  # loads render/human/human_lbs.npz once
    inp = HF.PersonInput(feet=..., yaw=..., look_at=..., push_w=..., grips=..., pole_dir=...)   # world frame
    poser.apply(inp, frame=42)                          # sets human_root + every pose bone, keys them at frame 42

Invariants: the result equals the numpy solve (the pose bones receive its local rotations, the pelvis its position,
``human_root`` the placement); IK / look-at constraints are switched off (push_ik = look = leg_ik = 0) because the
numpy solve already contains them.
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(os.path.dirname(HERE)), "tools")
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)
import human_figure as HF  # noqa: E402


def set_local_pose(rig_obj, rig_np: "HF.Rig", L: np.ndarray, pelvis_pos: np.ndarray) -> None:
    """Pose bones <- local rotations L (B,3,3) and the pelvis position (figure frame), Blender's bone convention."""
    pb = rig_obj.pose.bones
    for i, n in enumerate(rig_np.names):
        q = HF.quat_from_mat(L[i])
        b = pb[n]
        b.rotation_mode = "QUATERNION"
        b.rotation_quaternion = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        b.location = (0.0, 0.0, 0.0)
    ip = rig_np.ix["pelvis"]
    loc = rig_np.rest[ip, :3, :3].T @ (np.asarray(pelvis_pos, dtype=float) - rig_np.rest[ip, :3, 3])
    pb["pelvis"].location = tuple(float(x) for x in loc)


def key_pose(rig_obj, rig_np: "HF.Rig", frame: int) -> None:
    pb = rig_obj.pose.bones
    for n in rig_np.names:
        pb[n].keyframe_insert("rotation_quaternion", frame=frame, group=n)
    pb["pelvis"].keyframe_insert("location", frame=frame, group="pelvis")


class Poser:
    """Solve with the numpy runtime and apply to the Blender objects ``human_root`` / ``human_rig``."""

    def __init__(self, npz: str = HF.HUMAN_NPZ, root_name: str = "human_root", rig_name: str = "human_rig"):
        import bpy
        self.bpy = bpy
        self.rig_np = HF.Rig(npz)
        self.solver = HF.Solver(self.rig_np)
        self.root = bpy.data.objects[root_name]
        self.rig = bpy.data.objects[rig_name]

    def apply(self, inp: "HF.PersonInput", frame: int | None = None) -> "HF.PoseParams":
        P = HF.person_params(self.rig_np, inp)
        G, L = self.solver.solve(P)
        r = self.root
        for k in ("push_ik", "look", "leg_ik"):
            if k in r.keys():
                r[k] = 0.0
        r.location = tuple(float(x) for x in inp.feet)
        r.rotation_mode = "XYZ"
        r.rotation_euler = (0.0, 0.0, float(inp.yaw))
        if self.rig.animation_data is not None:
            self.rig.animation_data.action = None
        set_local_pose(self.rig, self.rig_np, L, G[self.rig_np.ix["pelvis"], :3, 3])
        if frame is not None:
            r.keyframe_insert("location", frame=frame)
            r.keyframe_insert("rotation_euler", frame=frame)
            key_pose(self.rig, self.rig_np, frame)
        return P


__all__ = ["Poser", "set_local_pose", "key_pose"]
