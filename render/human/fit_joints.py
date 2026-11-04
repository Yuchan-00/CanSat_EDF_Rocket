"""Step 2 (plain Python + numpy): fit the skeleton's joint centres to the prepared body mesh (``build/body_base.npz``
from ``bl_prepare_body.py``) and write ``build/joints.json`` (bone heads / tails / roll hints in Blender's frame:
the body faces -Y, its left is +X, z up, metres; origin moved to between the ankles on the floor).

Method (every joint comes from the mesh, nothing is typed in as a coordinate):
  * the bundle's face sets split the body into parts and every finger into phalanges; the boundary ring between two
    face sets (vertices used by faces of both) lies around a joint, so its centroid is the joint centre for the
    elbow, wrist, knee, ankle and the finger joints (PIP / DIP, thumb MCP / IP);
  * where a face-set boundary is not at the joint the centre is taken from cross-sections: the shoulder (the arm's
    cross-section centroid just below the acromion), the hip (the thigh's cross-section centroid below the crotch,
    moved medially / up along the femoral neck by an anthropometric ratio: hip-joint height 0.53 H, HJC half-spacing
    0.051 H), the spine (60 % of the torso depth toward the back at each level), the head pivot (ear level), the
    finger knuckles (the web boundary moved back along the finger by 45 % of the web-to-PIP distance) and the ball of
    the foot (the toe / foot boundary centroid);
  * the left side is fitted and mirrored (x -> -x) for the right; the mesh is symmetric to < 1 mm.

Roll hints: each bone gets the direction its tip moves under flexion ("curl"), so a positive rotation about the bone's
local X is flexion for elbows, knees, fingers, the spine and the neck (Blender: rotating about X turns Y toward Z).

Run: python fit_joints.py [BUILD_DIR] [--plot PNG]
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# face-set ids of the bundle's realistic male body (left side = +X); found by listing every face set's bounding box
FS = {
    "chest": 1, "neck_front": 22, "head": 17, "abdomen": 19, "pelvis": 18,
    "upperarm": 21, "forearm": 12, "hand": 9, "thigh": 24, "shin": 15, "foot": 14,
    "toes": (25, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63),
    # fingers: (proximal, middle, distal, nail); thumb: (metacarpal, proximal, distal, nail)
    "thumb": (80, 81, 82, 83), "index": (76, 77, 78, 79), "middle": (72, 73, 74, 75),
    "ring": (68, 69, 70, 71), "pinky": (64, 65, 66, 67),
}
FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def perp(v: np.ndarray, axis: np.ndarray) -> np.ndarray:
    a = unit(axis)
    return unit(v - a * float(v @ a))


class Body:
    def __init__(self, path: str):
        d = np.load(path)
        self.V = d["V"].astype(float)
        self.V1 = d["V1"].astype(float)
        self.height = float(d["height_m"])
        fs, loops, ls, lt = d["face_set"], d["loops"], d["loop_start"], d["loop_total"]
        self.faces = [loops[a:a + n] for a, n in zip(ls, lt)]
        self.fs = fs
        self.vsets: dict[int, set[int]] = {}
        for f, fa in zip(fs, self.faces):
            for v in fa:
                self.vsets.setdefault(int(v), set()).add(int(f))

    @staticmethod
    def _ids(x) -> set[int]:
        return set(x) if isinstance(x, (tuple, list, set)) else {int(x)}

    def verts_of(self, sets) -> np.ndarray:
        S = self._ids(sets)
        return np.array(sorted(v for v, s in self.vsets.items() if s & S), dtype=int)

    def ring(self, a, b) -> np.ndarray:
        A, B = self._ids(a), self._ids(b)
        idx = [v for v, s in self.vsets.items() if s & A and s & B]
        if not idx:
            raise ValueError(f"no boundary between face sets {a} and {b}")
        return self.V[np.array(idx)]

    def ring_c(self, a, b) -> np.ndarray:
        return self.ring(a, b).mean(axis=0)

    def section(self, pts: np.ndarray, z: float, dz: float = 0.006) -> np.ndarray:
        return pts[np.abs(pts[:, 2] - z) < dz]


def fit(build_dir: str) -> dict:
    b = Body(os.path.join(build_dir, "body_base.npz"))
    H = b.height
    V = b.V
    L: dict[str, np.ndarray] = {}
    # ---- legs (left: x > 0) ----
    knee = b.ring_c(FS["thigh"], FS["shin"])
    ankle = b.ring_c(FS["shin"], FS["foot"])
    ball = b.ring_c(FS["foot"], FS["toes"])
    toes = V[b.verts_of(FS["toes"])]
    toes = toes[toes[:, 0] > 0]
    foot_dir = unit(np.array([ball[0] - ankle[0], ball[1] - ankle[1], 0.0]))
    tip = toes[np.argmax(toes @ foot_dir)]
    toe_tip = np.array([tip[0], tip[1], ball[2]])
    ball = np.array([ball[0], ball[1], max(ball[2], 0.025)])
    # hip joint: thigh shaft centroid below the crotch, then up the femoral neck to the anthropometric hip height
    thigh = V[b.verts_of(FS["thigh"])]
    crotch = float(V[(np.abs(V[:, 0]) < 0.01) & (V[:, 2] < 0.95) & (V[:, 2] > 0.6)][:, 2].min())
    sec = b.section(thigh[thigh[:, 0] > 0], crotch - 0.04, 0.01)
    shaft = sec.mean(axis=0)
    hip = np.array([0.051 * H, shaft[1], 0.530 * H])
    # ---- spine (sagittal plane x = 0): 60 % of the torso depth toward the back ----
    torso = b.V1[(np.abs(b.V1[:, 0]) < 0.11)]                          # dense level-1 surface for sections

    def spine_pt(z: float, frac: float = 0.60) -> np.ndarray:
        s = b.section(torso, z, 0.004)
        y0, y1 = float(s[:, 1].min()), float(s[:, 1].max())        # front (-y) .. back (+y)
        return np.array([0.0, y0 + frac * (y1 - y0), z])

    z_belt = float(b.ring_c(FS["pelvis"], FS["abdomen"])[2])
    z_ribs = float(b.ring_c(FS["abdomen"], FS["chest"])[2])
    neck_ring = b.ring(FS["head"], FS["chest"])                       # the neck's base all around (back + sides)
    z_neck = float(neck_ring[:, 2].mean())
    pelvis_c = np.array([0.0, hip[1], hip[2]])
    s1 = spine_pt(z_belt)
    s2 = spine_pt(z_ribs)
    s3 = spine_pt(0.5 * (z_ribs + z_neck) + 0.02)
    neck_base = spine_pt(z_neck, 0.62)
    ears = V[b.verts_of((4, 5))]
    ear_c = ears.mean(axis=0)
    head_pivot = np.array([0.0, ear_c[1] - 0.012, ear_c[2] - 0.005])
    head_top = np.array([0.0, head_pivot[1], float(V[:, 2].max())])
    # ---- arm (left) ----
    elbow = b.ring_c(FS["upperarm"], FS["forearm"])
    wrist = b.ring_c(FS["forearm"], FS["hand"])
    arm = V[b.verts_of((FS["upperarm"], FS["chest"]))]
    top_sh = float(arm[(arm[:, 0] > 0.15)][:, 2].max())               # acromion (top of the shoulder)
    upper = V[b.verts_of(FS["upperarm"])]
    # the arm's axis: centroids of two sections of the upper arm, extended up to 4.5 cm below the acromion
    a1 = b.section(upper, elbow[2] + 0.08, 0.01).mean(axis=0)
    a2 = b.section(upper[upper[:, 0] > 0.17], top_sh - 0.12, 0.01).mean(axis=0)
    ax_ = unit(a2 - a1)
    z_sh = top_sh - 0.045
    shoulder = a2 + ax_ * (z_sh - a2[2]) / ax_[2]
    lat = b.section(b.V1[b.V1[:, 0] > 0], z_sh, 0.004)
    shoulder[0] = float(lat[:, 0].max()) - 0.048                        # 4.8 cm inside the deltoid's lateral surface
    sternal = np.array([0.022, float(spine_pt(z_neck - 0.02, 0.0)[1]) + 0.035, z_neck - 0.025])
    # ---- hand (left) ----
    fingers: dict[str, list[np.ndarray]] = {}
    for f in FINGERS:
        p, m, d_, n = FS[f]
        j1 = b.ring_c(FS["hand"], p)
        j2 = b.ring_c(p, m)
        j3 = b.ring_c(m, d_)
        tipv = V[b.verts_of((d_, n))]
        dirn = unit(j3 - j2)
        t_ = tipv[np.argmax((tipv - j3) @ dirn)]
        tip_ = j3 + dirn * float((t_ - j3) @ dirn) * 0.92               # the bone ends just inside the fingertip
        if f != "thumb":
            j1 = j1 - unit(j2 - j1) * 0.45 * float(np.linalg.norm(j2 - j1))   # knuckle behind the web
        else:
            j1 = j1 + (wrist - j1) * 0.35                             # thumb CMC: inside the thenar towards the wrist
        fingers[f] = [j1, j2, j3, tip_]
    hand_tail = fingers["middle"][0]
    # palm normal (direction the fingers curl toward): from the fingers' own rest curl
    curl = np.zeros(3)
    for f in ("index", "middle", "ring", "pinky"):
        j1, j2, j3, tp = fingers[f]
        curl += perp(tp - j1, j2 - j1)
    palm_n = unit(curl)
    # ---- move the origin between the ankles (floor) ----
    shift = np.array([0.0, -ankle[1], 0.0])

    def S(p):
        return (np.asarray(p, dtype=float) + shift).tolist()

    fwd = np.array([0.0, -1.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    bones: list[dict] = []

    def add(name, parent, head, tail, zhint, deform=True):
        head, tail = np.asarray(head, float), np.asarray(tail, float)
        bones.append({"name": name, "parent": parent, "head": S(head), "tail": S(tail),
                      "z": perp(np.asarray(zhint, float), tail - head).tolist(), "deform": deform})

    add("root", None, [0, -shift[1], 0], [0, -shift[1] + 0.25, 0], up, deform=False)
    add("pelvis", "root", pelvis_c, s1, fwd)
    add("spine_01", "pelvis", s1, s2, fwd)
    add("spine_02", "spine_01", s2, s3, fwd)
    add("spine_03", "spine_02", s3, neck_base, fwd)
    add("neck", "spine_03", neck_base, head_pivot, fwd)
    add("head", "neck", head_pivot, head_top, fwd)
    mir = np.array([-1.0, 1.0, 1.0])
    for side, M in (("L", np.ones(3)), ("R", mir)):
        def m(p, M=M):
            return np.asarray(p, float) * M
        add(f"clavicle_{side}", "spine_03", m(sternal), m(shoulder), up)
        # arm flexion (shoulder forward / elbow) moves the tip forward; the forearm's flexion side is its front
        add(f"upperarm_{side}", f"clavicle_{side}", m(shoulder), m(elbow), fwd)
        add(f"forearm_{side}", f"upperarm_{side}", m(elbow), m(wrist), fwd)
        add(f"forearm_twist_{side}", f"forearm_{side}", m(wrist + (elbow - wrist) * 0.5), m(wrist), fwd)
        add(f"hand_{side}", f"forearm_{side}", m(wrist), m(hand_tail), m(palm_n))
        for f in FINGERS:
            j = fingers[f]
            for i in range(3):
                zf = perp(palm_n, j[i + 1] - j[i]) if f != "thumb" else perp(
                    unit(fingers["middle"][1] - j[i]) + 0.3 * palm_n, j[i + 1] - j[i])
                add(f"{f}_0{i + 1}_{side}", f"hand_{side}" if i == 0 else f"{f}_0{i}_{side}", m(j[i]), m(j[i + 1]),
                    np.asarray(zf) * M)
        add(f"thigh_{side}", "pelvis", m(hip), m(knee), fwd)
        add(f"shin_{side}", f"thigh_{side}", m(knee), m(ankle), -fwd)        # knee flexion moves the foot back
        add(f"foot_{side}", f"shin_{side}", m(ankle), m(ball), up)
        add(f"toe_{side}", f"foot_{side}", m(ball), m(toe_tip), up)
    info = {"height_m": H, "origin_shift_m": shift.tolist(), "crotch_z_m": crotch, "shoulder_top_z_m": top_sh,
            "palm_normal_L": palm_n.tolist(), "source": "render/human/fit_joints.py"}
    return {"bones": bones, "info": info}


def plot(build_dir: str, res: dict, png: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = np.load(os.path.join(build_dir, "body_base.npz"))
    V = d["V"] + np.asarray(res["info"]["origin_shift_m"])
    fig, axs = plt.subplots(1, 4, figsize=(22, 11))
    views = (("front", 0, 2, None), ("side", 1, 2, None), ("hand L (front)", 0, 2, (0.30, 0.48, 0.68, 0.96)),
             ("hand L (side)", 1, 2, (-0.25, 0.05, 0.68, 0.96)))
    for ax, (title, i, j, lim) in zip(axs, views):
        P = V
        if lim:
            P = V[V[:, 0] > 0.25]
        ax.scatter(P[:, i], P[:, j], s=0.3, c="#9aa", alpha=0.5)
        for bn in res["bones"]:
            h, t = np.array(bn["head"]), np.array(bn["tail"])
            if lim and not bn["name"].endswith("_L"):
                continue
            ax.plot([h[i], t[i]], [h[j], t[j]], "-o", ms=2.5, lw=1.2, color="#c22" if bn["name"].endswith("_L") else "#22c")
        ax.set_aspect("equal")
        ax.set_title(title)
        if lim:
            ax.set_xlim(lim[0], lim[1])
            ax.set_ylim(lim[2], lim[3])
    fig.tight_layout()
    fig.savefig(png, dpi=90)


def main(argv: list[str]) -> None:
    build = os.path.join(HERE, "build")
    png = None
    rest = list(argv)
    if "--plot" in rest:
        i = rest.index("--plot")
        png = rest[i + 1]
        del rest[i:i + 2]
    if rest:
        build = rest[0]
    res = fit(build)
    with open(os.path.join(build, "joints.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1)
    print(f"{len(res['bones'])} bones -> {os.path.join(build, 'joints.json')}")
    for bn in res["bones"]:
        if not bn["name"].endswith("_R"):
            print(f"  {bn['name']:18s} head {np.round(bn['head'], 3)} tail {np.round(bn['tail'], 3)}")
    if png:
        plot(build, res, png)


if __name__ == "__main__":
    main(sys.argv[1:])
