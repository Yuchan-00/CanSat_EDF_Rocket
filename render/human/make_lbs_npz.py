"""Step 4 (plain Python, the project venv): turn ``build/lbs_raw.npz`` (Blender frame, from ``bl_build_rig.py``) into
``render/human/human_lbs.npz`` for ``tools/human_figure.py``, the PyVista runtime.

What it does (deterministic: no RNG, no clock):
  * converts to the figure frame (x forward, y left, z up: x = -y_blender, y = x_blender) - vertices, bone rest
    matrices; area-weighted vertex normals; top-4 skin weights per vertex (renormalised) for the render and shadow
    meshes;
  * finger presets ``relaxed_hands`` and ``grip`` as local rotations; the grip is closed on the posed MESH
    (``close_grip``): the pole axis is fixed in the palm along the knuckle line, then each finger and the thumb curl
    until their skin touches the pole (radius GRIP_RADIUS_M); the axis point / direction in each hand bone's frame
    are ``grip_center_*`` / ``grip_axis_*`` (what the runtime puts on the pole);
  * whole-body presets solved with ``human_figure.Solver`` (``idle``, ``watch_up``, ``push_ref``, ``walk_000`` ..
    ``walk_270``) stored as local rotations (w, x, y, z) + the pelvis position, for the Blender actions;
  * ``meta`` (JSON): source + licence, height, frames, the joint limits the runtime relies on, the generator.

Run: python make_lbs_npz.py [BUILD_DIR] [OUT_NPZ]
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import human_figure as HF  # noqa: E402

GRIP_RADIUS_M = 0.012          # the video's push pole (tools/sim_video.py draws it 24 mm thick)
C3 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])    # Blender -> figure frame


def vertex_normals(V: np.ndarray, F: np.ndarray) -> np.ndarray:
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])       # area-weighted
    n = np.zeros_like(V)
    for k in range(3):
        np.add.at(n, F[:, k], fn)
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def top4(W: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    order = np.argsort(-W, axis=1, kind="stable")[:, :4]
    w = np.take_along_axis(W, order, axis=1)
    s = w.sum(axis=1, keepdims=True)
    w = w / np.maximum(s, 1e-12)
    return w.astype(np.float32), order.astype(np.int16)


def finger_rot(curl_deg: float, spread_deg: float = 0.0) -> np.ndarray:
    return HF.rz(math.radians(spread_deg)) @ HF.rx(math.radians(curl_deg))


RELAX_CURL = {"index": (6, 10, 6, 2), "middle": (8, 12, 7, 0), "ring": (10, 14, 8, -2), "pinky": (12, 15, 8, -4)}
THUMB_RELAX = ((4, 0), (6, 0), (6, 0))


def finger_preset(rig: HF.Rig, curls: dict, thumb: tuple, scale: float = 1.0) -> np.ndarray:
    L = np.repeat(np.eye(3)[None], rig.B, axis=0)
    for s in "LR":
        sg = 1.0 if s == "L" else -1.0
        for f, (a, b, c, sp) in curls.items():
            for i, ang in enumerate((a, b, c)):
                L[rig.ix[f"{f}_0{i + 1}_{s}"]] = finger_rot(ang * scale, sp * sg * (1.0 if i == 0 else 0.0))
        for i, (ang, sp) in enumerate(thumb):
            L[rig.ix[f"thumb_0{i + 1}_{s}"]] = finger_rot(ang * scale, sp * sg)
    return L


def close_grip(rig: HF.Rig, side: str, r_pole: float, relax: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """Grasp a pole the way a hand closes on it: the pole axis is fixed in the palm (along the knuckle line, 12 mm
    proximal of the index / pinky MCP joints, its surface 1 mm off the palmar skin), then every finger curls in three
    stages - all joints, then PIP + DIP, then DIP - each stage stopping at the first skin contact with the pole (or the
    joint's limit); the thumb closes the same way until it touches the pole or the curled index / middle finger.
    Returns (local rotations of the side's finger bones in L, axis point and direction in the hand bone's frame,
    report)."""
    L = relax.copy()
    hb = rig.ix[f"hand_{side}"]
    Rh, oh = rig.rest[hb, :3, :3], rig.rest[hb, :3, 3]

    def to_h(p: np.ndarray) -> np.ndarray:
        return (np.asarray(p, dtype=float) - oh) @ Rh

    def bone_w(names: list[str]) -> np.ndarray:
        ids = [rig.ix[n] for n in names]
        return np.sum(np.where(np.isin(rig.wi, ids), rig.w, 0.0), axis=1)

    i1, i5 = to_h(rig.head_pos(f"index_01_{side}")), to_h(rig.head_pos(f"pinky_01_{side}"))
    a = HF.perp(i5 - i1, np.array([0.0, 0.0, 1.0]))                   # in the palm plane (hand Z = palm side)
    mid = 0.5 * (i1 + i5)
    c = mid + np.array([0.0, -0.012, 0.0])
    # palmar skin around the knuckle line: palm + finger-base vertices (hand Z points to the palm side)
    palm = to_h(rig.verts[bone_w([f"hand_{side}"] + [f"{f}_01_{side}" for f in ("index", "middle", "ring", "pinky")])
                          > 0.5])
    t = (palm - c) @ a
    band = palm[(np.abs(palm[:, 1] - c[1]) < 0.012) & (t > -0.005) & (t < float((i5 - i1) @ a) + 0.005)]
    c[2] = float(band[:, 2].max()) + r_pole + 0.001

    def axis_dist(P: np.ndarray) -> np.ndarray:
        d = P - c
        return np.linalg.norm(d - np.outer(d @ a, a), axis=1)

    def posed_subset(Lc: np.ndarray, idx: np.ndarray) -> np.ndarray:
        G, _ = rig.fk(Lc)
        S = rig.skin_mats(G)
        M = rig._blend(S, rig.w[idx], rig.wi[idx])
        v = np.einsum("nij,nj->ni", M[:, :, :3], rig.verts[idx]) + M[:, :, 3]
        Gh = G[hb]
        return (v - Gh[:3, 3]) @ Gh[:3, :3]

    report = {}
    limits = (95.0, 110.0, 85.0)
    for f in ("index", "middle", "ring", "pinky", "thumb"):
        bones = [f"{f}_0{i}_{side}" for i in (1, 2, 3)]
        idx = np.flatnonzero(bone_w(bones) > 0.3)
        own = [bone_w([bn])[idx] for bn in bones]
        base = [L[rig.ix[bn]].copy() for bn in bones]
        ang = [0.0, 0.0, 0.0]
        others = None
        if f == "thumb":
            oi = np.flatnonzero(bone_w([f"index_0{i}_{side}" for i in (1, 2, 3)] +
                                       [f"middle_0{i}_{side}" for i in (1, 2, 3)]) > 0.5)
            others = posed_subset(L, oi)
            # palmar abduction first (about the thumb's local Z, toward the palm side) until the thumb clears the pole
            sg = 1.0 if side == "L" else -1.0
            for ab in range(0, 91):
                Lt = L.copy()
                Lt[rig.ix[bones[0]]] = base[0] @ HF.rz(math.radians(sg * ab))
                if float(axis_dist(posed_subset(Lt, idx)).min()) > r_pole + 0.002:
                    break
            base[0] = base[0] @ HF.rz(math.radians(sg * (ab + 6)))
            L[rig.ix[bones[0]]] = base[0]
            report["thumb_abduction"] = ([float(ab + 6), 0.0, 0.0], 1.0)
        for stage in range(3):
            ratio = [(1.0, 0.3, 0.2), (0.0, 1.0, 0.7), (0.0, 0.0, 1.0)][stage]
            while True:
                trial = [ang[j] + ratio[j] for j in range(3)]
                if any(trial[j] > limits[j] for j in range(3) if ratio[j] > 0):
                    break
                Lt = L.copy()
                for j, bn in enumerate(bones):
                    Lt[rig.ix[bn]] = base[j] @ HF.rx(math.radians(trial[j]))
                P = posed_subset(Lt, idx)
                dist = axis_dist(P)
                seg = own[stage] > 0.5                                  # this stage's phalanx (and beyond)
                hit = bool(np.any(dist[seg | (own[2] > 0.5)] < r_pole + 0.0005))
                if others is not None:
                    from scipy.spatial import cKDTree
                    tip = (own[1] > 0.5) | (own[2] > 0.5)
                    hit = hit or bool(np.any(cKDTree(others).query(P[tip])[0] < 0.0025))
                if hit:
                    break
                ang = trial
        for j, bn in enumerate(bones):
            L[rig.ix[bn]] = base[j] @ HF.rx(math.radians(ang[j]))
        P = posed_subset(L, idx)
        report[f] = (ang, float(axis_dist(P).min()))
    return L, c, a, report


def main(argv: list[str]) -> None:
    build = argv[0] if argv else os.path.join(HERE, "build")
    out = argv[1] if len(argv) > 1 else os.path.join(HERE, "human_lbs.npz")
    d = np.load(os.path.join(build, "lbs_raw.npz"))
    names = [str(n) for n in d["names"]]
    V = d["V"] @ C3.T
    F = d["F"].astype(np.int32)
    Vs = d["Vs"] @ C3.T
    Fs = d["Fs"].astype(np.int32)
    C4 = np.eye(4)
    C4[:3, :3] = C3
    rest = np.einsum("ij,bjk->bik", C4, d["rest"])
    w, wi = top4(d["W"])
    sw, swi = top4(d["Ws"])
    j = json.load(open(os.path.join(build, "joints.json"), encoding="utf-8"))
    meta = {
        "source": "Blender Studio Human Base Meshes v1.4.1, object GEO-body_male_realistic (CC0 1.0)",
        "license": "CC0 1.0 (mesh); rig, weights and presets generated by render/human/build_human.py",
        "height_m": float(j["info"]["height_m"]),
        "frame": "figure frame: x forward, y left, z up, metres, origin on the floor between the ankles",
        "bones": "Blender convention (Y along the bone); +X rotation = flexion (spine, neck, elbows, knees, fingers)",
        "limits_deg": HF.LIMITS, "grip_radius_m": GRIP_RADIUS_M,
        "generator": "render/human/make_lbs_npz.py",
        "tris": int(F.shape[0]), "shadow_tris": int(Fs.shape[0]),
    }
    base = dict(meta=json.dumps(meta), bone_names=np.array(names), bone_parents=d["parents"].astype(np.int32),
                bone_rest=rest, bone_length=d["lengths"].astype(float),
                rest_verts=V.astype(np.float32), rest_normals=vertex_normals(V, F).astype(np.float32),
                faces=F, weights=w, bone_idx=wi,
                shadow_verts=Vs.astype(np.float32), shadow_faces=Fs, shadow_weights=sw, shadow_bone_idx=swi,
                grip_radius=GRIP_RADIUS_M)
    np.savez_compressed(out, **base)
    rig = HF.Rig(out)
    # ---- finger presets: relaxed + grip tuned on the mesh ----
    relax = finger_preset(rig, RELAX_CURL, THUMB_RELAX)
    grip = relax.copy()
    for s_ in "LR":
        Lg, c, a, rep = close_grip(rig, s_, GRIP_RADIUS_M, relax)
        for n in HF.finger_bones(s_):
            grip[rig.ix[n]] = Lg[rig.ix[n]]
        base[f"grip_center_{s_}"], base[f"grip_axis_{s_}"] = c, a
        meta[f"grip_{s_}"] = {f: {"curl_deg": [round(x, 1) for x in v[0]], "min_skin_to_axis_mm": round(v[1] * 1000, 2)}
                             for f, v in rep.items()}
        print(f"grip {s_}: centre (hand frame) {np.round(c, 4)}, axis {np.round(a, 3)}")
        for f, v in rep.items():
            print(f"   {f:7s} curl {np.round(v[0], 1)}  min skin-axis {v[1] * 1000:.1f} mm")
    presets = {"rest": np.repeat(np.eye(3)[None], rig.B, axis=0), "relaxed_hands": relax, "grip": grip}
    pelvis = {k: rig.head_pos("pelvis") for k in presets}
    base["preset_names"] = np.array(list(presets))
    base["preset_quats"] = np.array([[HF.quat_from_mat(R) for R in presets[k]] for k in presets])
    np.savez_compressed(out, **base)
    # ---- whole-body presets with the solver (needs the finger presets in the file) ----
    rig = HF.Rig(out)
    sol = HF.Solver(rig)
    hands = np.array([0.45, 0.0, 1.25])                       # sim_video.FIGURE_HANDS: the pole's held end
    tip = np.array([0.45 + 1.10, 0.0, 2.00])                  # a contact 2 m up, 1.1 m further (dev_push_synth)
    wdir = HF.unit(tip - hands)
    cases = {
        "idle": HF.PersonInput(feet=np.zeros(3), yaw=0.0, look_at=np.array([3.0, 0.0, 1.6])),
        "watch_up": HF.PersonInput(feet=np.zeros(3), yaw=0.0, look_at=np.array([1.6, 0.4, 3.2])),
        "push_ref": HF.PersonInput(feet=np.zeros(3), yaw=0.0, look_at=tip, push_w=1.0,
                                   grips=(hands + wdir * -0.12, hands + wdir * 0.10), pole_dir=wdir),
    }
    for k in range(4):
        cases[f"walk_{k * 90:03d}"] = HF.PersonInput(feet=np.zeros(3), yaw=0.0, look_at=np.array([3.0, 0.0, 1.6]),
                                                     walk_w=1.0, walk_phase=k * 0.5 * math.pi, walk_dir=np.array([1.0, 0.0]))
    for name, inp in cases.items():
        G, Lc = sol.solve(HF.person_params(rig, inp))
        presets[name] = Lc
        pelvis[name] = G[rig.ix["pelvis"], :3, 3]
        if name == "push_ref":
            for s in "LR":
                ar = sol.last[s]
                c_h, _ = rig.grip[s]
                gc = G[rig.ix[f"hand_{s}"]] @ np.append(c_h, 1.0)
                want = inp.grips[1 if s == "L" else 0]
                print(f"push_ref {s}: reached {ar.reached}, grip centre error {np.linalg.norm(gc[:3] - want) * 1000:.2f} mm")
    base["preset_names"] = np.array(list(presets))
    base["preset_quats"] = np.array([[HF.quat_from_mat(R) for R in presets[k]] for k in presets])
    base["preset_pelvis"] = np.array([pelvis[k] for k in presets])
    base["meta"] = json.dumps(meta)
    np.savez_compressed(out, **base)
    print(f"wrote {out}: {V.shape[0]} verts, {F.shape[0]} tris, shadow {Fs.shape[0]} tris, {len(names)} bones, "
          f"presets {list(presets)}")


if __name__ == "__main__":
    main(sys.argv[1:])
