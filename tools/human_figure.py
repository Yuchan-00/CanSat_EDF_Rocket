"""Rigged 1.75 m human for the realistic flight videos: numpy linear-blend skinning (LBS) + analytic IK, and a PyVista
actor that draws it.  Data: ``render/human/human_lbs.npz`` (built by ``render/human/build_human.py`` from the CC0
Blender Studio Human Base Meshes; see ``render/human/README.md``).

Frames (all metres): the *figure frame* has x forward (toward what the person faces), y to the person's left, z up,
origin on the floor between the ankles; the world placement is ``figure_matrix(feet, yaw)`` (x axis at ``yaw`` from
world +x).  Bones follow Blender's convention: Y along the bone (head -> tail), and each bone's roll puts Z on the side
its tip moves under flexion, so a positive rotation about local X is flexion for the spine, neck, elbows, knees and
fingers (the rest pose is the bundle's A-pose, every local rotation = identity).

Invariants:
  * Pure and deterministic: the posed mesh is a function of the arguments only (no clock, no RNG, no history), so any
    frame can be rendered alone (``--at`` previews) and two calls give identical arrays.
  * The IK never changes a bone length: arms and legs are two-bone analytic chains (``two_bone``); when a target is out
    of reach the chain points at it fully extended and ``ArmResult.reached`` is False (the caller can report it).
  * Grips are exact when reachable: the pole axis passes through the grip cavity of the posed hand (measured on the
    mesh when the npz is built: ``grip_center`` / ``grip_axis`` in each hand bone's frame) at the requested point.
  * Joint limits used here (``LIMITS``) are written into the npz meta by the generator; the solver clamps the head
    look-at, the spine and the pelvis to them; elbows / knees cannot hyper-extend (the two-bone solver bends only
    toward the pole).
  * LBS: v' = sum_k w_k S_{b_k} v with S_b = G_b Rest_b^-1 (top-4 weights per vertex); normals are rotated with the
    same blended matrix and renormalised.  ``Rig.skin`` costs ~4-6 ms for the 50k-triangle mesh on the dev PC.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.environ.get("SIM_VIDEO_ROOT") or os.path.dirname(TOOLS_DIR)
HUMAN_NPZ = os.path.join(PROJECT_ROOT, "render", "human", "human_lbs.npz")

# Monochrome satin "digital human" look (user reference: pale blue-grey, ~sRGB #a8c2dc on screen).  VTK's PBR takes the
# colour value as LINEAR albedo (see PLA_BLACK in render_scene.py): #a8c2dc written directly (albedo 0.66/0.76/0.86)
# rendered off-white in the hall and washed-out blue-white outdoors (measured 2026-10-21 against #7d98b8 / #5f7d9f
# in both scenes); #6a86a2 (albedo 0.42/0.53/0.64: near the sRGB decode of #a8c2dc, a little less saturated so the
# blue sky IBL does not turn it cyan) reads as the reference's pale blue-grey under the warm hall light and the bright
# field sky alike.
HUMAN_COLOR = "#6a86a2"
HUMAN_ROUGHNESS = 0.42
HUMAN_SPECULAR_IOR = 1.42

LIMITS = {                       # degrees; what the solver relies on (written into the npz meta)
    "look_yaw": 75.0, "look_pitch_up": 40.0, "look_pitch_down": 50.0, "look_neck_share": 0.4,
    "spine_flex": (-15.0, 35.0), "spine_side": 15.0, "spine_twist": 35.0,
    "pelvis_tilt": 12.0, "elbow_flex_max": 155.0, "knee_flex_max": 150.0, "leg_reach_frac": 0.992,
    "forearm_twist_share": 0.5,
}


# --------------------------------------------------------------------------- #
# rotations
# --------------------------------------------------------------------------- #
def unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def perp(v: np.ndarray, axis: np.ndarray) -> np.ndarray:
    a = unit(axis)
    v = np.asarray(v, dtype=float)
    return unit(v - a * float(v @ a))


_I3 = np.eye(3)


def cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """np.cross without its axis bookkeeping (3-vectors or (..., 3) arrays; ~10x faster for small inputs)."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return np.stack([a[..., 1] * b[..., 2] - a[..., 2] * b[..., 1], a[..., 2] * b[..., 0] - a[..., 0] * b[..., 2],
                     a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]], axis=-1)


def mats_from_quats(q: np.ndarray) -> np.ndarray:
    """(n, 4) unit quaternions (w, x, y, z) -> (n, 3, 3)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], axis=1),
                     np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], axis=1),
                     np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], axis=1)], axis=1)


def slerp_quats(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Row-wise slerp of (n, 4) quaternion arrays."""
    d = np.sum(q0 * q1, axis=1)
    q1 = np.where(d[:, None] < 0.0, -q1, q1)
    d = np.abs(d)
    th = np.arccos(np.clip(d, -1.0, 1.0))
    sn = np.sin(th)
    lin = sn < 1e-4
    a = np.where(lin, 1.0 - t, np.sin((1.0 - t) * th) / np.where(lin, 1.0, sn))
    b = np.where(lin, t, np.sin(t * th) / np.where(lin, 1.0, sn))
    q = a[:, None] * q0 + b[:, None] * q1
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def rot(axis: Sequence[float], ang: float) -> np.ndarray:
    """Rotation matrix about ``axis`` by ``ang`` rad (right hand)."""
    a = unit(np.asarray(axis, dtype=float))
    x, y, z = a
    c, s = math.cos(ang), math.sin(ang)
    C = 1.0 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rx(a: float) -> np.ndarray:
    return rot((1.0, 0.0, 0.0), a)


def ry(a: float) -> np.ndarray:
    return rot((0.0, 1.0, 0.0), a)


def rz(a: float) -> np.ndarray:
    return rot((0.0, 0.0, 1.0), a)


def quat_from_mat(R: np.ndarray) -> np.ndarray:
    """(w, x, y, z), w >= 0."""
    R = np.asarray(R, dtype=float)
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    q = q / np.linalg.norm(q)
    return q if q[0] >= 0.0 else -q


def mat_from_quat(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = unit(np.asarray(q, dtype=float))
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def slerp_mat(R0: np.ndarray, R1: np.ndarray, t: float) -> np.ndarray:
    if t <= 0.0:
        return np.asarray(R0, dtype=float)
    if t >= 1.0:
        return np.asarray(R1, dtype=float)
    q0, q1 = quat_from_mat(R0), quat_from_mat(R1)
    d = float(q0 @ q1)
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
    else:
        th = math.acos(min(1.0, d))
        q = (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)
    return mat_from_quat(q)


def twist_angle(R: np.ndarray, axis: np.ndarray) -> float:
    """Swing-twist decomposition: the twist angle (rad, -pi..pi) of rotation R about the unit ``axis``."""
    q = quat_from_mat(R)
    p = float(q[1:] @ unit(axis))
    return 2.0 * math.atan2(p, q[0])


def smoothstep(e0: float, e1: float, x: float) -> float:
    t = min(max((x - e0) / (e1 - e0), 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def figure_matrix(feet: Sequence[float], yaw: float) -> np.ndarray:
    """4x4 figure frame -> world: x axis at ``yaw`` from world +x, origin at ``feet``."""
    M = np.eye(4)
    M[:3, :3] = rz(yaw)
    M[:3, 3] = np.asarray(feet, dtype=float)
    return M


def two_bone(root: np.ndarray, target: np.ndarray, l1: float, l2: float, pole: np.ndarray
             ) -> tuple[np.ndarray, np.ndarray, bool]:
    """(joint, end, reached) of a two-bone chain from ``root`` toward ``target`` bending toward ``pole`` (a
    direction).  Lengths are kept exactly; out of reach -> fully extended toward the target (or folded when too
    near), ``reached`` False."""
    root = np.asarray(root, dtype=float)
    d = np.asarray(target, dtype=float) - root
    D = float(np.linalg.norm(d))
    u = d / D if D > 1e-9 else np.array([0.0, 0.0, -1.0])
    lo, hi = abs(l1 - l2) + 1e-9, l1 + l2 - 1e-9
    Dc = min(max(D, lo), hi)
    x = (l1 * l1 - l2 * l2 + Dc * Dc) / (2.0 * Dc)
    h = math.sqrt(max(l1 * l1 - x * x, 0.0))
    p = np.asarray(pole, dtype=float)
    p = p - u * float(p @ u)
    n = float(np.linalg.norm(p))
    if n < 1e-9:
        p = cross(u, [0.0, 1.0, 0.0])
        n = float(np.linalg.norm(p))
        if n < 1e-9:
            p, n = cross(u, [1.0, 0.0, 0.0]), 1.0
    p = p / n
    return root + u * x + p * h, root + u * Dc, bool(abs(D - Dc) < 1e-6)


def frame_yz(y: np.ndarray, z_hint: np.ndarray) -> np.ndarray:
    """3x3 with columns X, Y, Z: Y = unit(y), Z = z_hint made perpendicular, X = Y x Z."""
    Y = unit(y)
    Z = perp(z_hint, Y)
    return np.column_stack([cross(Y, Z), Y, Z])


# --------------------------------------------------------------------------- #
# the rig
# --------------------------------------------------------------------------- #
class Rig:
    """The npz: bones (names, parents, rest matrices in the figure frame), the render mesh and the shadow mesh with
    their top-4 skin weights, pose presets (local rotations) and the grip cavities of both hands."""

    def __init__(self, path: str = HUMAN_NPZ):
        d = np.load(path, allow_pickle=False)
        self.path = path
        self.meta: dict[str, Any] = json.loads(str(d["meta"]))
        self.names = [str(n) for n in d["bone_names"]]
        self.parents = d["bone_parents"].astype(int)
        self.rest = d["bone_rest"].astype(float)
        self.lengths = d["bone_length"].astype(float)
        self.B = len(self.names)
        self.ix = {n: i for i, n in enumerate(self.names)}
        self.rest_inv = np.linalg.inv(self.rest)
        self.rest_rel = np.empty_like(self.rest)
        for b, p in enumerate(self.parents):
            self.rest_rel[b] = self.rest[b] if p < 0 else self.rest_inv[p] @ self.rest[b]
        self.verts = d["rest_verts"].astype(np.float32)
        self.normals = d["rest_normals"].astype(np.float32)
        self.faces = d["faces"].astype(np.int64)
        self.w = d["weights"].astype(np.float32)
        self.wi = d["bone_idx"].astype(np.int64)
        self.s_verts = d["shadow_verts"].astype(np.float32)
        self.s_faces = d["shadow_faces"].astype(np.int64)
        self.s_w = d["shadow_weights"].astype(np.float32)
        self.s_wi = d["shadow_bone_idx"].astype(np.int64)
        self.presets: dict[str, np.ndarray] = {}
        pq = d["preset_quats"] if "preset_quats" in d else np.zeros((0, self.B, 4))
        for name, q in zip([str(n) for n in d["preset_names"]] if "preset_names" in d else [], pq):
            self.presets[name] = np.array([mat_from_quat(qq) for qq in q])
        self.grip: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for s in ("L", "R"):
            if f"grip_center_{s}" in d:
                self.grip[s] = (d[f"grip_center_{s}"].astype(float), unit(d[f"grip_axis_{s}"].astype(float)))
        self.grip_radius = float(d["grip_radius"]) if "grip_radius" in d else 0.012
        self.height = float(self.meta.get("height_m", 1.75))

    # rest-pose landmarks (figure frame)
    def head_pos(self, name: str) -> np.ndarray:
        return self.rest[self.ix[name], :3, 3].copy()

    def tail_pos(self, name: str) -> np.ndarray:
        b = self.ix[name]
        return self.rest[b, :3, 3] + self.rest[b, :3, 1] * self.lengths[b]

    # ---- forward kinematics ----
    def fk(self, local: np.ndarray | None = None,
           override: dict[int, Callable[[np.ndarray, np.ndarray], np.ndarray]] | None = None,
           pelvis_pos: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Global 4x4 (figure frame) of every bone from local rotations (B,3,3) (None = rest).  ``override[b](G, F)``
        returns bone b's global rotation given the globals computed so far and its parent-carried rest frame F (it may
        read G of bones before b in the order); its local rotation is then derived.  Returns (G, local)."""
        L = np.repeat(np.eye(3)[None], self.B, axis=0) if local is None else np.array(local, dtype=float)
        G = np.empty((self.B, 4, 4))
        for b in range(self.B):
            p = self.parents[b]
            F = self.rest_rel[b] if p < 0 else G[p] @ self.rest_rel[b]
            if b == 1 and pelvis_pos is not None:
                F = F.copy()
                F[:3, 3] = pelvis_pos
            if override and b in override:
                R = override[b](G, F)
                L[b] = F[:3, :3].T @ R
            G[b] = F
            G[b, :3, :3] = F[:3, :3] @ L[b]
        return G, L

    def skin_mats(self, G: np.ndarray) -> np.ndarray:
        """(B, 3, 4) skinning matrices S_b = G_b Rest_b^-1."""
        return np.einsum("bij,bjk->bik", G, self.rest_inv)[:, :3, :].astype(np.float32)

    @staticmethod
    def _blend(S: np.ndarray, w: np.ndarray, wi: np.ndarray) -> np.ndarray:
        M = S[wi[:, 0]] * w[:, 0, None, None]
        for k in range(1, wi.shape[1]):
            M += S[wi[:, k]] * w[:, k, None, None]
        return M

    def _wmat(self, which: str) -> Any:
        """(N, B) sparse weight matrix (scipy CSR) of the render ('r') or shadow ('s') mesh, built once; None
        without scipy (then the gather blend above is used)."""
        key = "_W_" + which
        if not hasattr(self, key):
            try:
                from scipy.sparse import csr_matrix
                w, wi = (self.w, self.wi) if which == "r" else (self.s_w, self.s_wi)
                n = w.shape[0]
                rows = np.repeat(np.arange(n), w.shape[1])
                setattr(self, key, csr_matrix((w.ravel().astype(np.float32), (rows, wi.ravel())), shape=(n, self.B)))
            except ImportError:
                setattr(self, key, None)
        return getattr(self, key)

    def _blended(self, S: np.ndarray, which: str) -> np.ndarray:
        """Per-vertex blended (N, 3, 4) matrices sum_k w_k S_{b_k}."""
        Wm = self._wmat(which)
        if Wm is None:
            return self._blend(S, *((self.w, self.wi) if which == "r" else (self.s_w, self.s_wi)))
        return np.asarray(Wm @ S.reshape(self.B, 12)).reshape(-1, 3, 4)

    def skin(self, G: np.ndarray, world: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Posed render-mesh vertices and unit normals (float32), in the figure frame or ``world`` (4x4) @ it."""
        S = self.skin_mats(G)
        if world is not None:
            Wm = np.asarray(world, dtype=np.float32)
            S = np.einsum("ij,bjk->bik", Wm[:3, :3], S)
            S[:, :, 3] += Wm[:3, 3]
        M = self._blended(S, "r")
        R = M[:, :, :3]
        v = np.einsum("nij,nj->ni", R, self.verts) + M[:, :, 3]
        n = np.einsum("nij,nj->ni", R, self.normals)
        n /= np.maximum(np.sqrt(np.einsum("ni,ni->n", n, n))[:, None], 1e-12)
        return v, n

    def skin_shadow(self, G: np.ndarray, world: np.ndarray | None = None) -> np.ndarray:
        """(m, 3, 3) world triangles of the posed shadow mesh."""
        S = self.skin_mats(G)
        if world is not None:
            Wm = np.asarray(world, dtype=np.float32)
            S = np.einsum("ij,bjk->bik", Wm[:3, :3], S)
            S[:, :, 3] += Wm[:3, 3]
        M = self._blended(S, "s")
        v = np.einsum("nij,nj->ni", M[:, :, :3], self.s_verts) + M[:, :, 3]
        return v[self.s_faces].astype(float)


# --------------------------------------------------------------------------- #
# pose description
# --------------------------------------------------------------------------- #
@dataclass
class LegGoal:
    ankle: np.ndarray                    # ankle joint target, figure frame
    yaw_deg: float = 0.0                 # foot heading (toe out > 0 on the left side means toward +y)
    heel_deg: float = 0.0                # heel lift about the ball of the foot (walking push-off)
    knee_out: float = 0.15               # knee pole: forward + this much outward


@dataclass
class ArmGoal:
    wrist: np.ndarray                    # hand-bone head (wrist) target without a grip, figure frame
    pole: np.ndarray                     # elbow direction, figure frame
    grip_pt: np.ndarray | None = None    # pole point at the hand's grip centre, figure frame
    grip_dir: np.ndarray | None = None   # pole direction (unit, toward the tip), figure frame
    grip_w: float = 0.0                  # 0 = free hand (rest relation to the forearm), 1 = exact grip
    grip_pole: np.ndarray | None = None  # elbow direction while gripping


@dataclass
class PoseParams:
    pelvis_off: np.ndarray = field(default_factory=lambda: np.zeros(3))
    pelvis_ypr: tuple[float, float, float] = (0.0, 0.0, 0.0)     # deg: yaw (left +), pitch (forward +), roll (left up +)
    spine_fst: tuple[float, float, float] = (0.0, 0.0, 0.0)      # deg: flex (forward +), side (right +), twist (left +)
    legs: dict[str, LegGoal] = field(default_factory=dict)
    arms: dict[str, ArmGoal] = field(default_factory=dict)
    finger_mix: dict[str, float] = field(default_factory=lambda: {"L": 0.0, "R": 0.0})   # 0 relaxed .. 1 grip
    look_at: np.ndarray | None = None                            # figure frame
    look_w: float = 1.0


@dataclass
class ArmResult:
    reached: bool = True
    wrist_err_m: float = 0.0
    grip_err_m: float = 0.0


SPINE = ("spine_01", "spine_02", "spine_03")
SPINE_SHARE = (0.30, 0.35, 0.35)
FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def finger_bones(side: str) -> list[str]:
    return [f"{f}_0{i}_{side}" for f in FINGERS for i in (1, 2, 3)]


class Solver:
    """Full-body pose from ``PoseParams``: pelvis + spine (distributed), two-bone legs with flat feet, two-bone arms
    to wrist targets or exact pole grips (the grip roll about the pole chosen to keep the wrist straight), forearm twist
    share, finger presets, head look-at with limits."""

    def __init__(self, rig: Rig):
        self.rig = rig
        r = rig
        ix = r.ix
        self.ix = ix
        self.len = {n: float(r.lengths[i]) for n, i in ix.items()}
        self.relax = r.presets.get("relaxed_hands")
        self.gripP = r.presets.get("grip")
        eye = np.repeat(np.eye(3)[None], r.B, axis=0)
        self.relax_q = np.array([quat_from_mat(R) for R in (self.relax if self.relax is not None else eye)])
        self.grip_q = np.array([quat_from_mat(R) for R in (self.gripP if self.gripP is not None else eye)])
        self.finger_ids = {s: np.array([ix[n] for n in finger_bones(s)]) for s in "LR"}
        self.rest_R = r.rest[:, :3, :3]
        self.chest_rest = self.rest_R[ix["spine_03"]]
        # arm lengths: shoulder -> elbow, elbow -> wrist (head to head)
        self.arm_l = {s: (float(np.linalg.norm(r.head_pos(f"forearm_{s}") - r.head_pos(f"upperarm_{s}"))),
                          float(np.linalg.norm(r.head_pos(f"hand_{s}") - r.head_pos(f"forearm_{s}")))) for s in "LR"}
        self.leg_l = {s: (float(np.linalg.norm(r.head_pos(f"shin_{s}") - r.head_pos(f"thigh_{s}"))),
                          float(np.linalg.norm(r.head_pos(f"foot_{s}") - r.head_pos(f"shin_{s}")))) for s in "LR"}
        self.eye_off = np.array([0.09, 0.0, 0.07])     # eyes from the head pivot, figure frame at rest
        self.last: dict[str, ArmResult] = {}

    # ---- grip: hand rotation + wrist so the cavity axis lies on the pole at grip_pt ----
    def grip_solution(self, side: str, S: np.ndarray, g: np.ndarray, wdir: np.ndarray, pole: np.ndarray
                      ) -> tuple[np.ndarray, np.ndarray]:
        """(wrist target, hand global rotation) for a grip centred at ``g`` on a pole along ``wdir``; the roll about
        the pole is chosen (deterministically, coarse grid then refinement) to minimise the wrist bend plus a share
        of the forearm twist, given the shoulder S and the elbow pole."""
        c_h, a_h = self.rig.grip[side]
        R_rest_h = self.rest_R[self.ix[f"hand_{side}"]]
        rel_fh = self.rest_R[self.ix[f"forearm_{side}"]].T @ R_rest_h      # hand in the forearm frame at rest
        # the cavity axis runs index -> pinky; the thumb side faces the pole's tip
        target_axis = -unit(wdir)
        R0 = _align(R_rest_h @ a_h, target_axis) @ R_rest_h
        l1, l2 = self.arm_l[side]

        ax = target_axis
        K_ = np.array([[0.0, -ax[2], ax[1]], [ax[2], 0.0, -ax[0]], [-ax[1], ax[0], 0.0]])
        aaT = np.outer(ax, ax)
        pole = unit(pole)

        def evaluate(phis: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
            """Vectorised over the roll candidates: (cost, wrist targets, hand rotations)."""
            c, s_ = np.cos(phis)[:, None, None], np.sin(phis)[:, None, None]
            Rax = c * _I3 + (1.0 - c) * aaT + s_ * K_                           # Rodrigues, (K, 3, 3)
            Rh = Rax @ R0
            W = g - Rh @ c_h
            d = W - S
            D = np.linalg.norm(d, axis=1)
            u = d / np.maximum(D, 1e-9)[:, None]
            Dc = np.clip(D, abs(l1 - l2) + 1e-9, l1 + l2 - 1e-9)
            x = (l1 * l1 - l2 * l2 + Dc * Dc) / (2.0 * Dc)
            h = np.sqrt(np.maximum(l1 * l1 - x * x, 0.0))
            pp = pole[None, :] - u * (u @ pole)[:, None]
            pp /= np.maximum(np.linalg.norm(pp, axis=1), 1e-9)[:, None]
            E = S + u * x[:, None] + pp * h[:, None]
            Wr = S + u * Dc[:, None]
            Yu = (E - S) / l1
            Yf = (Wr - E) / np.maximum(np.linalg.norm(Wr - E, axis=1), 1e-9)[:, None]
            Zu = Yf - Yu * np.sum(Yf * Yu, axis=1)[:, None]                     # the forearm's side = flexion side
            Zu /= np.maximum(np.linalg.norm(Zu, axis=1), 1e-9)[:, None]
            Xu = cross(Yu, Zu)
            Rf = np.stack([Xu, Yf, cross(Xu, Yf)], axis=2)
            R_free = Rf @ rel_fh                                                 # hand left at its rest relation
            bend = np.arccos(np.clip(np.sum(Rh[:, :, 1] * R_free[:, :, 1], axis=1), -1.0, 1.0))
            xf = R_free[:, :, 0] - Yf * np.sum(R_free[:, :, 0] * Yf, axis=1)[:, None]
            xh = Rh[:, :, 0] - Yf * np.sum(Rh[:, :, 0] * Yf, axis=1)[:, None]
            tw = np.arctan2(np.sum(cross(xf, xh) * Yf, axis=1), np.sum(xf * xh, axis=1))
            miss = np.maximum(D - (l1 + l2), 0.0)
            return bend * bend + 0.35 * tw * tw + 50.0 * miss, W, Rh

        phis = np.linspace(-math.pi, math.pi, 36, endpoint=False)              # 10 deg grid, then 1.7 deg + parabola
        c, _, _ = evaluate(phis)
        best = float(phis[int(np.argmin(c))])
        h_ = 2.0 * math.pi / 36 / 6.0
        cand = best + h_ * np.arange(-6, 7)
        c, _, _ = evaluate(cand)
        j = min(max(int(np.argmin(c)), 1), len(cand) - 2)
        den = float(c[j - 1] - 2.0 * c[j] + c[j + 1])
        off = 0.5 * float(c[j - 1] - c[j + 1]) / den if den > 1e-12 else 0.0
        _, Ws, Rs = evaluate(np.array([float(cand[j]) + h_ * min(max(off, -1.0), 1.0)]))
        return Ws[0], Rs[0]

    def solve(self, P: PoseParams) -> tuple[np.ndarray, np.ndarray]:
        """(G (B,4,4) global bone matrices in the figure frame, local rotations (B,3,3))."""
        r, ix = self.rig, self.ix
        lim = LIMITS
        ov: dict[int, Callable[[np.ndarray, np.ndarray], np.ndarray]] = {}
        local = np.repeat(np.eye(3)[None], r.B, axis=0)
        # pelvis
        yaw, pit, rol = P.pelvis_ypr
        tl = lim["pelvis_tilt"]
        Rp = rz(math.radians(yaw)) @ ry(math.radians(min(max(pit, -tl), tl))) @ rx(math.radians(min(max(rol, -tl), tl)))
        pel_rest = r.head_pos("pelvis")
        pel_pos = pel_rest + np.asarray(P.pelvis_off, dtype=float)
        # keep both legs reachable: lower the pelvis when an ankle target is beyond leg_reach_frac x leg length
        for s, lg in P.legs.items():
            hip = pel_pos + Rp @ (r.head_pos(f"thigh_{s}") - pel_rest)
            Lr = lim["leg_reach_frac"] * sum(self.leg_l[s])
            d = hip - np.asarray(lg.ankle, dtype=float)
            dh2 = float(d[0] ** 2 + d[1] ** 2)
            if float(d @ d) > Lr * Lr and dh2 < Lr * Lr:
                pel_pos[2] -= float(d[2]) - math.sqrt(Lr * Lr - dh2)
        ov[ix["pelvis"]] = lambda G, F: Rp @ self.rest_R[ix["pelvis"]]
        # spine (distributed)
        fl, sd, tw = P.spine_fst
        fl = min(max(fl, lim["spine_flex"][0]), lim["spine_flex"][1])
        sd = min(max(sd, -lim["spine_side"]), lim["spine_side"])
        tw = min(max(tw, -lim["spine_twist"]), lim["spine_twist"])
        for name, k in zip(SPINE, SPINE_SHARE):
            local[ix[name]] = ry(math.radians(tw * k)) @ rx(math.radians(fl * k)) @ rz(math.radians(sd * k))
        # fingers (relaxed <-> grip, slerp per bone)
        for s in "LR":
            m = min(max(float(P.finger_mix.get(s, 0.0)), 0.0), 1.0)
            ids = self.finger_ids[s]
            local[ids] = mats_from_quats(slerp_quats(self.relax_q[ids], self.grip_q[ids], m))
        # head look-at (neck + head share the rotation, in the chest frame)
        if P.look_at is not None and P.look_w > 0.0:
            def look(share: float, name: str) -> Callable[[np.ndarray, np.ndarray], np.ndarray]:
                def f(G: np.ndarray, F: np.ndarray) -> np.ndarray:
                    C = G[ix["spine_03"], :3, :3]
                    Crel = C @ self.chest_rest.T                       # chest rotation from rest
                    eye = G[ix["spine_03"], :3, 3] + Crel @ (r.head_pos("head") + self.eye_off - r.head_pos("spine_03"))
                    d = Crel.T @ (np.asarray(P.look_at, dtype=float) - eye)      # in the rest chest's axes
                    ya = math.atan2(d[1], d[0])
                    pi_ = math.atan2(d[2], math.hypot(d[0], d[1]))
                    ya = min(max(ya, -math.radians(lim["look_yaw"])), math.radians(lim["look_yaw"]))
                    pi_ = min(max(pi_, -math.radians(lim["look_pitch_down"])), math.radians(lim["look_pitch_up"]))
                    ya, pi_ = ya * share * P.look_w, pi_ * share * P.look_w
                    Rl = Crel @ rz(ya) @ rot((0.0, 1.0, 0.0), -pi_) @ Crel.T
                    base = Crel @ self.rest_R[ix[name]]
                    return Rl @ base
                return f
            nshare = lim["look_neck_share"]
            ov[ix["neck"]] = look(nshare, "neck")
            ov[ix["head"]] = look(1.0, "head")
        # arms
        self.last = {}
        for s, ag in P.arms.items():
            self._arm(s, ag, ov)
        # legs
        for s, lg in P.legs.items():
            self._leg(s, lg, ov)
        return r.fk(local, ov, pel_pos)

    def _arm(self, s: str, ag: ArmGoal, ov: dict) -> None:
        ix = self.ix
        l1, l2 = self.arm_l[s]
        st: dict[str, Any] = {}
        res = ArmResult()
        self.last[s] = res

        def upper(G: np.ndarray, F: np.ndarray) -> np.ndarray:
            S = F[:3, 3]
            gw = float(ag.grip_w)
            pole = unit(ag.pole)
            W = np.asarray(ag.wrist, dtype=float)
            if gw > 0.0 and ag.grip_pt is not None and s in self.rig.grip:
                gpole = unit(ag.grip_pole if ag.grip_pole is not None else ag.pole)
                Wg, Rg = self.grip_solution(s, S, np.asarray(ag.grip_pt, float), unit(ag.grip_dir), gpole)
                W = (1.0 - gw) * W + gw * Wg
                pole = unit((1.0 - gw) * pole + gw * gpole)
                st["Rg"], st["gw"] = Rg, gw
            E, Wr, ok = two_bone(S, W, l1, l2, pole)
            res.reached = ok
            res.wrist_err_m = float(np.linalg.norm(Wr - W))
            Yu = unit(E - S)
            Zu = perp(Wr - E, Yu) if float(np.linalg.norm(cross(Yu, unit(Wr - E)))) > 1e-3 else perp(-pole, Yu)
            Ru = np.column_stack([cross(Yu, Zu), Yu, Zu])
            st["Ru"], st["E"], st["W"] = Ru, E, Wr
            return Ru

        def fore(G: np.ndarray, F: np.ndarray) -> np.ndarray:
            Ru = st["Ru"]
            Yf = unit(st["W"] - st["E"])
            X = Ru[:, 0]
            Rf = np.column_stack([X, Yf, cross(X, Yf)])
            st["Rf"] = Rf
            return Rf

        def hand(G: np.ndarray, F: np.ndarray) -> np.ndarray:
            R_free = F[:3, :3]                                    # rest relation to the forearm
            Rh = R_free if "Rg" not in st else slerp_mat(R_free, st["Rg"], st["gw"])
            st["Rh"] = Rh
            return Rh

        def twist(G: np.ndarray, F: np.ndarray) -> np.ndarray:
            # share of the hand's twist about the forearm axis (forearm_twist sits before hand in the bone order:
            # compute the hand rotation here the same way the hand override will)
            Fh = G[ix[f"forearm_{s}"]] @ self.rig.rest_rel[ix[f"hand_{s}"]]
            R_free = Fh[:3, :3]
            Rh = R_free if "Rg" not in st else slerp_mat(R_free, st["Rg"], st["gw"])
            rel = Rh @ R_free.T                                   # rotation of the hand away from its rest relation
            ax = st["Rf"][:, 1]
            ang = twist_angle(rel, ax) * LIMITS["forearm_twist_share"]
            return rot(ax, ang) @ F[:3, :3]

        ov[ix[f"upperarm_{s}"]] = upper
        ov[ix[f"forearm_{s}"]] = fore
        ov[ix[f"forearm_twist_{s}"]] = twist
        ov[ix[f"hand_{s}"]] = hand

    def _leg(self, s: str, lg: LegGoal, ov: dict) -> None:
        ix = self.ix
        l1, l2 = self.leg_l[s]
        st: dict[str, Any] = {}
        sgn = 1.0 if s == "L" else -1.0
        Ryaw = rz(math.radians(lg.yaw_deg))
        pole = unit(Ryaw @ np.array([1.0, sgn * lg.knee_out, 0.0]))
        lat = Ryaw @ np.array([0.0, 1.0, 0.0])
        Rh = rot(lat, math.radians(lg.heel_deg))
        # heel lift pivots about the ball of the foot (toes stay down), a toe-up about the heel: move the ankle target
        ank = np.asarray(lg.ankle, dtype=float)
        a_rest = self.rig.head_pos(f"foot_{s}")
        piv = (self.rig.head_pos(f"toe_{s}") if lg.heel_deg >= 0.0
               else np.array([a_rest[0] - 0.055, a_rest[1], 0.012]))
        v = Ryaw @ (piv - a_rest)
        target = ank + v - Rh @ v

        def thigh(G: np.ndarray, F: np.ndarray) -> np.ndarray:
            Hp = F[:3, 3]
            K, A, ok = two_bone(Hp, target, l1, l2, pole)
            Yt = unit(K - Hp)
            Rt = frame_yz(Yt, pole)
            st["K"], st["A"], st["Rt"] = K, A, Rt
            return Rt

        def shin(G: np.ndarray, F: np.ndarray) -> np.ndarray:
            Ys = unit(st["A"] - st["K"])
            X = -st["Rt"][:, 0]                                   # the shin's Z points back: its X is the thigh's -X
            return np.column_stack([X, Ys, cross(X, Ys)])

        def foot(G: np.ndarray, F: np.ndarray) -> np.ndarray:
            return Rh @ Ryaw @ self.rest_R[ix[f"foot_{s}"]]

        def toe(G: np.ndarray, F: np.ndarray) -> np.ndarray:
            return Ryaw @ self.rest_R[ix[f"toe_{s}"]]

        ov[ix[f"thigh_{s}"]] = thigh
        ov[ix[f"shin_{s}"]] = shin
        ov[ix[f"foot_{s}"]] = foot
        ov[ix[f"toe_{s}"]] = toe


def _align(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Smallest rotation taking unit a to unit b."""
    a, b = unit(a), unit(b)
    v = cross(a, b)
    c = float(a @ b)
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        if c > 0:
            return np.eye(3)
        o = perp(np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0]), a)
        return rot(o, math.pi)
    return rot(v / s, math.atan2(s, c))


# --------------------------------------------------------------------------- #
# behaviour: idle stand / watching / walking / pole push, blended
# --------------------------------------------------------------------------- #
WALK_STRIDE_M = 1.15          # one full gait cycle (two steps)
WALK_STANCE = 0.60            # fraction of the cycle a foot is on the ground
BLEND_S = 0.3                 # pose transitions (push in / out) take this long


@dataclass
class PersonInput:
    """What the video knows about the person at one instant (world frame unless noted)."""
    feet: np.ndarray                          # origin between the feet (floor), world
    yaw: float                                # facing, rad (world)
    t: float = 0.0                            # time (s): drives the slow idle sway only
    look_at: np.ndarray | None = None         # world point to look at (the vehicle)
    push_w: float = 0.0                       # 0..1 smoothed push weight
    grips: tuple[np.ndarray, np.ndarray] | None = None   # world grip centres (right / rear, left / front)
    pole_dir: np.ndarray | None = None        # world unit direction of the pole (toward the vehicle)
    walk_w: float = 0.0                       # 0..1 smoothed walking weight
    walk_phase: float = 0.0                   # gait phase, rad (2 pi per WALK_STRIDE_M travelled)
    walk_dir: np.ndarray | None = None        # walking direction in the FIGURE frame (2,), unit


def _lerp(a: Any, b: Any, w: float) -> Any:
    return (1.0 - w) * np.asarray(a, dtype=float) + w * np.asarray(b, dtype=float)


def _gait_foot(u: float, d: np.ndarray, step: float) -> tuple[np.ndarray, float, float]:
    """Foot offset (figure frame, along walk dir d), lift height, heel lift deg at gait fraction u of one leg."""
    half = 0.5 * WALK_STANCE * WALK_STRIDE_M
    if u < WALK_STANCE:
        s = u / WALK_STANCE
        x = half - 2.0 * half * s
        heel = 22.0 * smoothstep(0.65, 1.0, s)
        lift = 0.0
    else:
        s = (u - WALK_STANCE) / (1.0 - WALK_STANCE)
        e = 0.5 - 0.5 * math.cos(math.pi * s)
        x = -half + 2.0 * half * e
        lift = 0.07 * math.sin(math.pi * s)
        heel = 22.0 * (1.0 - smoothstep(0.0, 0.35, s)) - 8.0 * smoothstep(0.6, 1.0, s)
    return np.array([d[0] * x * step, d[1] * x * step, 0.0]), lift, heel


def person_params(rig: Rig, inp: PersonInput) -> PoseParams:
    """Blend idle (relaxed, weight on the right leg, slow sway), walking and the pole push into one ``PoseParams``
    (figure frame).  Weights come from the caller (``push_w``, ``walk_w``), already smoothed over ``BLEND_S``."""
    Mf = figure_matrix(inp.feet, inp.yaw)
    Mi = np.linalg.inv(Mf)

    def to_fig(p: np.ndarray) -> np.ndarray:
        return Mi[:3, :3] @ np.asarray(p, dtype=float) + Mi[:3, 3]

    pw = float(min(max(inp.push_w, 0.0), 1.0))
    ww = float(min(max(inp.walk_w, 0.0), 1.0)) * (1.0 - pw)
    iw = 1.0 - pw - ww
    sway = 0.012 * math.sin(2.0 * math.pi * inp.t / 7.3)          # slow weight shift / breathing sway (idle)
    ank_z = float(rig.head_pos("foot_L")[2])
    # ---- idle: weight on the right leg, left knee relaxed, left foot a little forward and out ----
    idle = dict(pel=np.array([0.0, -0.035 + sway, -0.012]), ypr=(4.0, 0.0, -3.5), spine=(2.0, -2.5, -3.0),
                L=(np.array([0.06, 0.125, ank_z]), 14.0), R=(np.array([-0.01, -0.105, ank_z]), -7.0))
    # ---- push: left foot forward, right foot back and out, low centre of mass, lean toward the pole ----
    push = dict(pel=np.array([0.02, 0.0, -0.045]), ypr=(-14.0, 4.0, 0.0), spine=(10.0, 0.0, 10.0),
                L=(np.array([0.24, 0.12, ank_z]), 6.0), R=(np.array([-0.30, -0.15, ank_z]), -32.0))
    # ---- walk ----
    d = np.array([1.0, 0.0]) if inp.walk_dir is None else unit(np.asarray(inp.walk_dir, dtype=float))
    uL = (inp.walk_phase / (2.0 * math.pi)) % 1.0
    uR = (uL + 0.5) % 1.0
    oL, hL, eL = _gait_foot(uL, d, 1.0)
    oR, hR, eR = _gait_foot(uR, d, 1.0)
    bob = -0.035 + 0.012 * math.cos(4.0 * math.pi * uL)
    walk = dict(pel=np.array([0.0, 0.015 * math.sin(2 * math.pi * uL), bob]), ypr=(6.0 * math.sin(2 * math.pi * uL), 2.0, 0.0),
                spine=(4.0, 0.0, -8.0 * math.sin(2 * math.pi * uL)),
                L=(np.array([oL[0], 0.10 + oL[1], ank_z + hL]), 0.0), R=(np.array([oR[0], -0.10 + oR[1], ank_z + hR]), 0.0))
    P = PoseParams()
    P.pelvis_off = iw * idle["pel"] + ww * walk["pel"] + pw * push["pel"]
    P.pelvis_ypr = tuple(iw * a + ww * b + pw * c for a, b, c in zip(idle["ypr"], walk["ypr"], push["ypr"]))
    P.spine_fst = tuple(iw * a + ww * b + pw * c for a, b, c in zip(idle["spine"], walk["spine"], push["spine"]))
    for s, heel in (("L", eL), ("R", eR)):
        a = iw * idle[s][0] + ww * walk[s][0] + pw * push[s][0]
        yaw_f = iw * idle[s][1] + ww * walk[s][1] + pw * push[s][1]
        P.legs[s] = LegGoal(ankle=a, yaw_deg=yaw_f, heel_deg=ww * heel,
                            knee_out=0.15 + 0.10 * pw)
    # ---- arms ----
    swing = math.sin(2.0 * math.pi * uL)                          # left leg forward -> right arm forward
    iw_off = np.array([0.0, 0.0, float(idle["pel"][2])])
    for s, sg in (("L", 1.0), ("R", -1.0)):
        # free arms hang from the shoulder: targets move with the pelvis (bob / lowering) so the elbow bend stays put
        idle_w = np.array([0.085, sg * 0.212, 0.872]) + iw_off
        sh = rig.head_pos(f"upperarm_{s}")
        th = math.radians(24.0) * (-sg * swing) * float(d[0])        # pendulum swing along the walking direction
        walk_w = np.array([sh[0] + 0.03 + 0.50 * math.sin(th), sg * 0.205, sh[2] - 0.50 * math.cos(th)]) + walk["pel"]
        wrist = (iw + pw) * idle_w + ww * walk_w
        ag = ArmGoal(wrist=wrist, pole=np.array([-1.0, sg * 0.35, -0.15]))
        if pw > 0.0 and inp.grips is not None and inp.pole_dir is not None:
            g = inp.grips[1 if s == "L" else 0]
            ag.grip_pt = to_fig(g)
            ag.grip_dir = Mi[:3, :3] @ unit(inp.pole_dir)
            ag.grip_w = pw
            ag.grip_pole = np.array([-0.25, sg * 0.85, -0.9])
        P.arms[s] = ag
    P.finger_mix = {"L": pw, "R": pw}
    if inp.look_at is not None:
        P.look_at = to_fig(inp.look_at)
    return P


# --------------------------------------------------------------------------- #
# PyVista actor
# --------------------------------------------------------------------------- #
class HumanActor:
    """The skinned mesh as one PBR actor (points + normals replaced in place every frame) and its shadow triangles."""

    def __init__(self, pl: Any, rig: Rig | None = None, color: str = HUMAN_COLOR, roughness: float = HUMAN_ROUGHNESS):
        import pyvista as pv
        self.rig = rig if rig is not None else Rig()
        self.solver = Solver(self.rig)
        r = self.rig
        faces = np.hstack([np.full((r.faces.shape[0], 1), 3), r.faces]).astype(np.int64).ravel()
        self.mesh = pv.PolyData(r.verts.astype(np.float32).copy(), faces=faces)
        self.mesh.point_data["Normals"] = r.normals.copy()
        self.mesh.point_data.active_normals_name = "Normals"
        self.actor = pl.add_mesh(self.mesh, color=color, pbr=True, metallic=0.0, roughness=roughness,
                                 smooth_shading=False, show_scalar_bar=False)
        self.actor.prop.interpolation = "pbr"
        try:
            self.actor.GetProperty().SetBaseIOR(HUMAN_SPECULAR_IOR)
        except AttributeError:                                   # older VTK without SetBaseIOR
            pass
        self.G: np.ndarray | None = None
        self.world: np.ndarray | None = None

    def update(self, inp: PersonInput, opacity: float = 1.0, visible: bool = True) -> PoseParams:
        P = person_params(self.rig, inp)
        G, _ = self.solver.solve(P)
        Mf = figure_matrix(inp.feet, inp.yaw)
        v, n = self.rig.skin(G, Mf)
        pts = self.mesh.GetPoints().GetData()
        from vtk.util.numpy_support import vtk_to_numpy
        vtk_to_numpy(pts)[:] = v
        self.mesh.GetPoints().Modified()
        vtk_to_numpy(self.mesh.GetPointData().GetNormals())[:] = n
        self.mesh.GetPointData().GetNormals().Modified()
        self.mesh.Modified()
        self.actor.prop.opacity = float(opacity)
        self.actor.visibility = bool(visible)
        self.G, self.world = G, Mf
        return P

    def triangles_world(self) -> np.ndarray:
        if self.G is None:
            return np.zeros((0, 3, 3))
        return self.rig.skin_shadow(self.G, self.world)

    def top_world(self) -> np.ndarray:
        """World position of the top of the head (for labels)."""
        if self.G is None:
            return np.zeros(3)
        ih = self.rig.ix["head"]
        p = self.G[ih] @ np.array([0.0, self.rig.lengths[ih], 0.0, 1.0])
        return (self.world @ p)[:3]
