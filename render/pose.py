"""The operator's pose for the Blender videos (pure numpy; runs in the venv and inside Blender).

``tools/human_figure.py`` (read-only here, owned by the human-figure agent) solves the body; this module only builds its
``PoseParams`` the way this pipeline needs them:

  * the pole never leaves the hands: both fists grip it in the carry as well as in the push (human_figure grips only
    while pushing);
  * the push stance is this pipeline's own (art review round 2: the human_figure push stance read as a fencing lunge -
    legs scissored, rear toe pointed): a planted staggered stance, both soles flat, 0.45 m between the ankles along
    the push, the front knee flexed ~15-20 deg by lowering the pelvis 5 cm, the trunk leaning a little into the pole.
    It replaces human_figure's push stance in the same blend: params = (1 - push_w) * (idle / walk at push_w = 0) +
    push_w * PUSH_STANCE, so idle and walking are exactly human_figure's;
  * walking never mixes into a push (render.plan zeroes walk_w while push_w > 0.05).

``pose_metrics`` measures what the art director checks on screen: sole heights above the floor, pole-to-head clearance,
fist-to-face distance, knee flexion, unreached grips.  render.plan / tools/blender_render.py gate on them.

Invariant: deterministic; never fakes a reach (the solver's ``last[side].reached`` is reported as is).
"""
from __future__ import annotations

import math
import os
import sys
from typing import Any

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# figure frame (x forward, y left, z up), metres / degrees
# (probed with render scratch grip_probe_r2: front knee 18 deg, rear 20 deg, both soles 0.0 cm off the floor; with the
#  feet 0.22 / -0.23 m the rear leg's reach dropped the pelvis and the front knee folded 43 deg - a lunge again)
PUSH_STANCE = {
    "L": (np.array([0.30, 0.115, 0.0]), 8.0),        # front (left) ankle: ahead, foot pointing along the push
    "R": (np.array([-0.15, -0.135, 0.0]), -24.0),    # rear (right) ankle: 0.45 m behind, toes turned out (braced)
    "pel": np.array([0.0, -0.012, -0.03]),            # pelvis 3 cm lower: both knees flex
    "ypr": (-12.0, 4.0, 0.0),                         # pelvis yawed toward the right-hand (rear) grip, slight tilt
    "spine": (9.0, 0.0, 8.0),                         # trunk leaning into the pole, chest turned to it
}
LEAN_REACH_M = 0.50            # front fist -> left shoulder (rest) beyond which the operator leans into the push ...
LEAN_SPAN_M = 0.14             # ... fully over this much more
LEAN_PELVIS_M = 0.07           # pelvis forward at full lean
LEAN_FLEX_DEG = 12.0           # extra trunk flexion at full lean
LEAN_TWIST_DEG = 10.0          # chest turned to bring the left shoulder forward
CARRY_GRIP_POLE = (-0.6, 0.7, -0.9)                 # elbow directions (x, |y|, z) in the carry ...
PUSH_GRIP_POLE = (-0.25, 0.85, -0.9)                  # ... and in the push (y signed per side)


def human_figure() -> Any:
    tools = os.path.join(ROOT, "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import human_figure as HF
    return HF


def pose_params(HF: Any, rig: Any, inp: Any) -> Any:
    """``PoseParams`` for one frame (``inp`` = human_figure.PersonInput, world frame)."""
    import dataclasses
    pw = float(min(max(inp.push_w, 0.0), 1.0))
    P0 = HF.person_params(rig, dataclasses.replace(inp, push_w=0.0))
    P = HF.person_params(rig, inp)
    ank_z = float(rig.head_pos("foot_L")[2])
    P.pelvis_off = (1.0 - pw) * np.asarray(P0.pelvis_off, float) + pw * PUSH_STANCE["pel"]
    P.pelvis_ypr = tuple((1.0 - pw) * a + pw * b for a, b in zip(P0.pelvis_ypr, PUSH_STANCE["ypr"]))
    P.spine_fst = tuple((1.0 - pw) * a + pw * b for a, b in zip(P0.spine_fst, PUSH_STANCE["spine"]))
    for s in ("L", "R"):
        lg0 = P0.legs[s]
        a1, y1 = PUSH_STANCE[s]
        a1 = a1 + np.array([0.0, 0.0, ank_z])
        P.legs[s] = HF.LegGoal(ankle=(1.0 - pw) * np.asarray(lg0.ankle, float) + pw * a1,
                               yaw_deg=(1.0 - pw) * lg0.yaw_deg + pw * y1, heel_deg=(1.0 - pw) * lg0.heel_deg,
                               knee_out=0.15 + 0.08 * pw)
    # the pole is always in both hands
    if inp.grips is not None and inp.pole_dir is not None:
        Mf = HF.figure_matrix(inp.feet, inp.yaw)
        Mi = np.linalg.inv(Mf)
        # the feet stay planted while the vehicle gives way: when the front (left) fist must go further than a
        # comfortable reach from its shoulder, the operator leans in - pelvis forward (weight onto the front foot)
        # and the trunk flexes - instead of the arm falling short
        gl = Mi[:3, :3] @ np.asarray(inp.grips[1], float) + Mi[:3, 3]
        excess = float(np.linalg.norm(gl - rig.head_pos("upperarm_L"))) - LEAN_REACH_M
        lean = pw * float(np.clip(excess / LEAN_SPAN_M, 0.0, 1.0))
        if lean > 0.0:
            P.pelvis_off = np.asarray(P.pelvis_off, float) + np.array([LEAN_PELVIS_M * lean, 0.0, -0.01 * lean])
            fl, sd, tw = P.spine_fst
            P.spine_fst = (fl + LEAN_FLEX_DEG * lean, sd, tw - LEAN_TWIST_DEG * lean)
        for s, gi, sg in (("R", 0, -1.0), ("L", 1, 1.0)):
            ag = P.arms[s]
            ag.grip_pt = Mi[:3, :3] @ np.asarray(inp.grips[gi], float) + Mi[:3, 3]
            ag.grip_dir = Mi[:3, :3] @ np.asarray(inp.pole_dir, float)
            ag.grip_w = 1.0
            c, p = np.array(CARRY_GRIP_POLE), np.array(PUSH_GRIP_POLE)
            gp = (1.0 - pw) * c + pw * p
            ag.grip_pole = np.array([gp[0], sg * gp[1], gp[2]])
        P.finger_mix = {"L": 1.0, "R": 1.0}
    return P


def person_input(HF: Any, P: dict[str, np.ndarray], k: int) -> Any:
    """human_figure.PersonInput of plan row k (render.plan person_* arrays)."""
    return HF.PersonInput(feet=P["person_feet"][k], yaw=float(P["person_yaw"][k]), t=float(P["t"][k]),
                          look_at=P["person_look"][k], push_w=float(P["person_push_w"][k]),
                          grips=(P["person_grip_r"][k], P["person_grip_l"][k]), pole_dir=P["person_pole_dir"][k],
                          walk_w=float(P["person_walk_w"][k]), walk_phase=float(P["person_walk_phase"][k]),
                          walk_dir=P["person_walk_dir"][k])


def _knee_flex(G: np.ndarray, ix: dict, s: str) -> float:
    h, k, a = G[ix[f"thigh_{s}"], :3, 3], G[ix[f"shin_{s}"], :3, 3], G[ix[f"foot_{s}"], :3, 3]
    u, v = k - h, a - k
    return math.degrees(math.acos(float(np.clip(u @ v / (np.linalg.norm(u) * np.linalg.norm(v)), -1.0, 1.0))))


def pose_metrics(HF: Any, rig: Any, solver: Any, inp: Any, butt: np.ndarray, pad: np.ndarray,
                 floor_z: float) -> dict[str, Any]:
    """Solve one frame and measure it (world metres / degrees)."""
    prm = pose_params(HF, rig, inp)
    G, _ = solver.solve(prm)
    ix = rig.ix
    Mf = HF.figure_matrix(inp.feet, inp.yaw)

    def W(p: np.ndarray) -> np.ndarray:
        return Mf[:3, :3] @ p + Mf[:3, 3]
    sole = {}
    for s in "LR":
        dz_ank = float(G[ix[f"foot_{s}"], 2, 3]) - float(rig.head_pos(f"foot_{s}")[2])
        dz_toe = float(G[ix[f"toe_{s}"], 2, 3]) - float(rig.head_pos(f"toe_{s}")[2])
        sole[s] = float(inp.feet[2]) - floor_z + max(abs(dz_ank), abs(dz_toe))
    Rh = G[ix["head"], :3, :3] @ rig.rest[ix["head"], :3, :3].T
    hp = G[ix["head"], :3, 3]
    face = W(hp + Rh @ np.array([0.10, 0.0, 0.0]))          # mouth / chin level, in front of the head pivot
    headc = W(hp + Rh @ np.array([0.03, 0.0, 0.08]))        # skull centre
    u = np.asarray(pad, float) - np.asarray(butt, float)
    L = float(np.linalg.norm(u))
    u /= max(L, 1e-9)
    rel = headc - butt
    tt = float(np.clip(rel @ u, 0.0, L))
    pole_head = float(np.linalg.norm(rel - u * tt)) - 0.10 - 0.012           # skull radius ~0.10 m, pole radius
    fist_face = min(float(np.linalg.norm(np.asarray(g, float) - face)) for g in inp.grips) if inp.grips is not None else 9.9
    last = getattr(solver, "last", {}) or {}
    return {"sole_L_m": round(sole["L"], 4), "sole_R_m": round(sole["R"], 4),
            "pole_head_m": round(pole_head, 3), "fist_face_m": round(fist_face, 3),
            "knee_L_deg": round(_knee_flex(G, ix, "L"), 1), "knee_R_deg": round(_knee_flex(G, ix, "R"), 1),
            "reached": all(getattr(v, "reached", True) for v in last.values()),
            "pole_pitch_deg": round(math.degrees(math.asin(float(np.clip(u[2], -1, 1)))), 1),
            "grip_h_m": [round(float(np.asarray(g, float)[2]) - floor_z, 3) for g in inp.grips] if inp.grips is not None else []}


# gates (art review round 2)
SOLE_MAX_M = 0.01              # both soles within 1 cm of the floor whenever push_w > 0.5
POLE_HEAD_MIN_M = 0.15         # pole surface to skull surface
FIST_FACE_MIN_M = 0.25


def pose_qa(P: dict[str, np.ndarray], floor_z: float, frames: Any = None, every: int = 1) -> dict[str, Any]:
    """Run ``pose_metrics`` over plan rows (all, or ``frames`` 1-based) and apply the gates."""
    HF = human_figure()
    rig = HF.Rig()
    solver = HF.Solver(rig)
    n = int(P["t"].shape[0])
    rows = [f - 1 for f in frames] if frames is not None else list(range(0, n, max(1, every)))
    out: dict[str, Any] = {"frames": 0, "fail": [], "worst": {}}
    worst = {"sole_m": 0.0, "pole_head_m": 9.9, "fist_face_m": 9.9, "walk_w_in_push": 0.0}
    for k in rows:
        inp = person_input(HF, P, k)
        m = pose_metrics(HF, rig, solver, inp, P["person_butt"][k], P["person_pad"][k], floor_z)
        out["frames"] += 1
        pw = float(P["person_push_w"][k])
        bad = []
        if pw > 0.5:
            s = max(m["sole_L_m"], m["sole_R_m"])
            worst["sole_m"] = max(worst["sole_m"], s)
            worst["walk_w_in_push"] = max(worst["walk_w_in_push"], float(P["person_walk_w"][k]))
            if s > SOLE_MAX_M:
                bad.append(f"sole {s * 100:.1f} cm")
            if float(P["person_walk_w"][k]) > 0.0:
                bad.append("walking while pushing")
        worst["pole_head_m"] = min(worst["pole_head_m"], m["pole_head_m"])
        worst["fist_face_m"] = min(worst["fist_face_m"], m["fist_face_m"])
        if m["pole_head_m"] < POLE_HEAD_MIN_M:
            bad.append(f"pole-head {m['pole_head_m']:.2f} m")
        if m["fist_face_m"] < FIST_FACE_MIN_M:
            bad.append(f"fist-face {m['fist_face_m']:.2f} m")
        if not m["reached"]:
            bad.append("grip out of reach")
        if bad:
            out["fail"].append({"frame": k + 1, "push_w": round(pw, 2), "issues": bad})
    out["worst"] = {k: round(v, 3) for k, v in worst.items()}
    out["ok"] = not out["fail"]
    return out
