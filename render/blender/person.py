"""The person with the padded pole (run inside Blender).

USER DECISION (2026-10-20 23:45, binding): the person is the anatomically realistic CC0 "digital human" built by a
parallel agent from Blender Studio's Human Base Meshes - ``render/human/human_rigged.blend`` (collection ``human``:
``human_root`` placement empty facing local +X, ``human_rig`` armature, ``human_body`` mesh with the pale blue-grey
satin material ``human_satin``) described by ``render/human/README.md``.  This module never builds a person of its
own: it appends that collection read-only and poses it per frame with the README's recommended route - the numpy
solver ``tools/human_figure.py`` (the same one the PyVista analysis videos use, exact parity) - fed from the frame
plan (render.plan.person_track: feet, yaw, push weight, walking, grips on the pole, look-at).  The solved local bone
rotations are keyed in bulk (one F-curve write per channel).  Without the asset the person is a clearly marked
PLACEHOLDER HOOK: nothing of the person (and no pole, which would float) is drawn and the build reports it.

Pole (a prop, built here): rigid 24 mm aluminium tube, two sections joined by a sleeve, 2.33 m from the butt
(rubber cap) to the centre of a 45 mm foam ball pad (dimensions from the plan meta 'pole' = render.plan POLE_*): long
enough to reach a 1.5 m hover's contact (~2.0 m above the floor) within 15 deg of horizontal, so the pole's axial
force matches the logged horizontal push.  Its empty is keyed to the plan's butt -> pad.  Between pushes the person
carries it upright at the ready (both hands on it); before a push the arms bring it down onto the contact over 0.5 s
(render.plan.PUSH_BLEND_S).
"""
from __future__ import annotations

import math
import os
import sys
from typing import Any

import bpy
import numpy as np

from . import bl_util as U
from .vehicle import _lathe

HUMAN_SUBDIR = ("render", "human")
# the pole's dimensions come from the plan meta ('pole': render.plan POLE_*); these are the fallbacks
POLE_BUTT_M, POLE_HOLD_TO_PAD, POLE_PAD_R, POLE_R = 0.33, 2.00, 0.045, 0.012
POLE_LEN = POLE_BUTT_M + POLE_HOLD_TO_PAD


def human_paths(root: str) -> tuple[str, str]:
    d = os.path.join(root, *HUMAN_SUBDIR)
    return os.path.join(d, "human_rigged.blend"), os.path.join(d, "README.md")


def human_available(root: str) -> bool:
    b, r = human_paths(root)
    return os.path.exists(b) and os.path.exists(r) and os.path.exists(os.path.join(root, "tools", "human_figure.py"))


# --------------------------------------------------------------------------- #
# pole
# --------------------------------------------------------------------------- #
def _pole_materials() -> tuple[Any, Any, Any]:
    alu, nb, out = U.new_material("pole_aluminium")
    tc = nb.n("ShaderNodeTexCoord")
    noise = nb.n("ShaderNodeTexNoise", inputs={"Scale": 300.0, "Detail": 2.0})
    nb.l(tc.outputs["Object"], noise.inputs["Vector"])
    rr = nb.n("ShaderNodeMapRange", inputs={1: 0.3, 2: 0.7, 3: 0.26, 4: 0.36})
    nb.l(noise.outputs["Fac"], rr.inputs[0])
    p = U.principled(nb, Base_Color=(0.80, 0.81, 0.82, 1.0), Metallic=1.0, Anisotropic=0.5)
    nb.l(rr.outputs[0], p.inputs["Roughness"])
    nb.l(p.outputs[0], out.inputs["Surface"])
    foam, nb, out = U.new_material("pole_foam")
    tc = nb.n("ShaderNodeTexCoord")
    noise = nb.n("ShaderNodeTexNoise", inputs={"Scale": 500.0, "Detail": 2.0})
    nb.l(tc.outputs["Object"], noise.inputs["Vector"])
    bump = nb.n("ShaderNodeBump", inputs={"Strength": 0.35, "Distance": 0.0012})
    nb.l(noise.outputs["Fac"], bump.inputs["Height"])
    p = U.principled(nb, Base_Color=U.srgb("#1f4fa8"), Roughness=0.85, Sheen_Weight=0.3)
    nb.l(bump.outputs["Normal"], p.inputs["Normal"])
    nb.l(p.outputs[0], out.inputs["Surface"])
    rub, nb, out = U.new_material("pole_rubber")
    p = U.principled(nb, Base_Color=(0.015, 0.015, 0.015, 1.0), Roughness=0.8)
    nb.l(p.outputs[0], out.inputs["Surface"])
    return alu, foam, rub


def build_pole(coll: Any, dims: dict | None = None) -> Any:
    """Empty 'pole' at the butt, +z toward the pad (pad centre at z = the pole length).  ``dims`` = plan meta 'pole'."""
    dims = dims or {}
    L = float(dims.get("len_m", POLE_LEN))
    pr = float(dims.get("pad_r_m", POLE_PAD_R))
    r = float(dims.get("r_m", POLE_R))
    alu, foam, rub = _pole_materials()
    e = U.empty("pole", coll, size=0.1)
    e.rotation_mode = "QUATERNION"
    v, f = _lathe([(0.0, 0.0), (r, 0.0), (r, L - 0.03), (0.0, L - 0.03)], n=24)
    U.mesh_object("pole_tube", v, f, coll, parent=e, smooth_angle_deg=50.0, mat=alu)
    prof = [(pr * math.sin(a), L - pr * math.cos(a)) for a in np.linspace(0.0, math.pi, 14)]
    prof[0] = (0.0, prof[0][1])
    prof[-1] = (0.0, prof[-1][1])
    v, f = _lathe(prof, n=32)
    U.mesh_object("pole_pad", v, f, coll, parent=e, smooth_angle_deg=70.0, mat=foam)
    v, f = _lathe([(0.0, -0.025), (r + 0.003, -0.025), (r + 0.003, 0.03), (0.0, 0.03)], n=20)
    U.mesh_object("pole_cap", v, f, coll, parent=e, smooth_angle_deg=50.0, mat=rub)
    # a joint sleeve at mid length (a 2.33 m pole is two sections) - a small real-world detail
    v, f = _lathe([(0.0, 0.5 * L - 0.04), (r + 0.0018, 0.5 * L - 0.04), (r + 0.0018, 0.5 * L + 0.04), (0.0, 0.5 * L + 0.04)], n=20)
    U.mesh_object("pole_joint", v, f, coll, parent=e, smooth_angle_deg=50.0, mat=rub)
    # foam sleeve from ahead of the front grip to the pad (the padded pole of the demo safety rule; art review r2)
    s0, s1 = (float(x) for x in dims.get("sleeve_m", (0.62, min(2.22, L - pr - 0.02))))
    rs = r + 0.009
    prof = [(0.0, s0), (rs - 0.004, s0), (rs, s0 + 0.006), (rs, s1 - 0.006), (rs - 0.004, s1), (0.0, s1)]
    v, f = _lathe(prof, n=28)
    sm, nb, out = U.new_material("pole_sleeve_foam")           # charcoal closed-cell foam (pipe-insulation look)
    tc = nb.n("ShaderNodeTexCoord")
    noise = nb.n("ShaderNodeTexNoise", inputs={"Scale": 700.0, "Detail": 2.0})
    nb.l(tc.outputs["Object"], noise.inputs["Vector"])
    bump = nb.n("ShaderNodeBump", inputs={"Strength": 0.3, "Distance": 0.001})
    nb.l(noise.outputs["Fac"], bump.inputs["Height"])
    p = U.principled(nb, Base_Color=U.srgb("#2b2f35"), Roughness=0.9, Sheen_Weight=0.25)
    nb.l(bump.outputs["Normal"], p.inputs["Normal"])
    nb.l(p.outputs[0], out.inputs["Surface"])
    U.mesh_object("pole_sleeve", v, f, coll, parent=e, smooth_angle_deg=60.0, mat=sm)
    return e


def pole_quats(butt: np.ndarray, tip: np.ndarray) -> np.ndarray:
    """(N,4) w,x,y,z rotating +z onto tip - butt (shortest arc), hemisphere-continuous."""
    d = np.asarray(tip, float) - np.asarray(butt, float)
    d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
    z = np.array([0.0, 0.0, 1.0])
    out = np.zeros((d.shape[0], 4))
    for i, u in enumerate(d):
        c = float(np.dot(z, u))
        ax = np.cross(z, u)
        if c < -0.9999:
            out[i] = (0.0, 1.0, 0.0, 0.0)
            continue
        q = np.array([1.0 + c, ax[0], ax[1], ax[2]])
        out[i] = q / np.linalg.norm(q)
    for i in range(1, out.shape[0]):
        if float(np.dot(out[i], out[i - 1])) < 0:
            out[i] = -out[i]
    return out


# --------------------------------------------------------------------------- #
# the CC0 human (render/human/**, read-only)
# --------------------------------------------------------------------------- #
def load_human(root: str, coll: Any, warnings: list[str]) -> dict[str, Any] | None:
    """Append the asset's collection ``human`` (objects keep their names); None = PLACEHOLDER (asset missing)."""
    blend, _ = human_paths(root)
    if not human_available(root):
        warnings.append("person: PLACEHOLDER (render/human/human_rigged.blend + README.md or tools/human_figure.py "
                        "not found) - person and pole omitted")
        return None
    with bpy.data.libraries.load(blend, link=False) as (src, dst):
        if "human" in src.collections:
            dst.collections = ["human"]
        else:
            dst.objects = list(src.objects)
    if dst.collections:
        hc = dst.collections[0]
        coll.children.link(hc)
        objs = list(hc.all_objects)
    else:
        objs = [o for o in dst.objects if o is not None]
        for o in objs:
            coll.objects.link(o)
    by = {o.name.split(".")[0]: o for o in objs}        # a second copy gets '.001' suffixes
    h = {"objects": objs, "root": by.get("human_root"), "rig": by.get("human_rig"), "body": by.get("human_body")}
    if h["root"] is None or h["rig"] is None:
        warnings.append("person: human asset loaded but human_root / human_rig missing; person not posed")
        return None
    return h


def neutralise_groin(h: dict[str, Any], warnings: list[str]) -> int:
    """A shop-mannequin groin (art review round 2: the nude satin figure's anatomy distracted at close range in a
    school competition): a vertex group over the pubic region (rest pose, figure frame: centre (0.15, 0, 0.85) m,
    radius 0.10 m, soft falloff) drives a Smooth modifier placed BEFORE the Armature, so the cage is smoothed in rest
    space and then posed.  Runtime only - render/human/** is not touched.  Returns the vertex count in the group."""
    body, root_o = h.get("body"), h.get("root")
    if body is None or body.type != "MESH" or root_o is None:
        return 0
    from mathutils import Matrix
    M = root_o.matrix_world.inverted() @ body.matrix_world if root_o is not None else Matrix.Identity(4)
    me = body.data
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    A = np.array(M)
    pf = co @ A[:3, :3].T + A[:3, 3]
    c = np.array([0.15, 0.0, 0.85])
    d = np.linalg.norm((pf - c) * np.array([1.0, 1.4, 1.0]), axis=1)
    w = np.clip(1.0 - (d - 0.05) / 0.07, 0.0, 1.0)
    w = w * w * (3.0 - 2.0 * w)
    idx = np.flatnonzero(w > 0.01)
    if idx.size == 0:
        warnings.append("person: groin region not found on the body mesh (no smoothing)")
        return 0
    vg = body.vertex_groups.new(name="groin_neutral")
    for i in idx:
        vg.add([int(i)], float(w[i]), "REPLACE")
    mod = body.modifiers.new("groin_smooth", "SMOOTH")
    mod.factor = 1.0
    mod.iterations = 40
    mod.vertex_group = vg.name
    # first in the stack (rest space)
    try:
        body.modifiers.move(body.modifiers.find(mod.name), 0)
    except (AttributeError, RuntimeError, TypeError):
        warnings.append("person: could not move the groin smoothing before the armature")
    return int(idx.size)


def _gear_materials() -> tuple[Any, Any, Any]:
    sat, nb, out = U.new_material("gear_satin")
    p = U.principled(nb, Base_Color=U.srgb("#4d5c6c"), Roughness=0.42, Coat_Weight=0.2, Coat_Roughness=0.3)
    nb.l(p.outputs[0], out.inputs["Surface"])
    lens, nb, out = U.new_material("gear_lens")
    p = U.principled(nb, Base_Color=U.srgb("#1e252c"), Roughness=0.06, Coat_Weight=1.0, Coat_Roughness=0.03)
    nb.l(p.outputs[0], out.inputs["Surface"])
    cup, nb, out = U.new_material("gear_cup")
    p = U.principled(nb, Base_Color=U.srgb("#e2b33c"), Roughness=0.38, Coat_Weight=0.3, Coat_Roughness=0.25)
    nb.l(p.outputs[0], out.inputs["Surface"])
    return sat, lens, cup


def _arc_band(name: str, coll: Any, parent: Any, cx: float, a: float, b: float, z0: float, z1: float,
              t0: float, t1: float, th: float, mat: Any, n: int = 40) -> Any:
    """An elliptic band (centre (cx, 0), semi-axes a (x) / b (y), angles t0..t1 from +x, z0..z1 tall, ``th`` thick)."""
    ts = np.linspace(t0, t1, n)
    v, f = [], []
    for r_off in (0.0, th):
        for z in (z0, z1):
            for t in ts:
                v.append([cx + (a + r_off) * math.cos(t), (b + r_off) * math.sin(t), z])
    m = len(ts)
    blk = lambda r, zz: (2 * r + zz) * m              # noqa: E731
    for i in range(m - 1):
        f.append([blk(0, 0) + i, blk(0, 0) + i + 1, blk(0, 1) + i + 1, blk(0, 1) + i])     # inner
        f.append([blk(1, 0) + i, blk(1, 1) + i, blk(1, 1) + i + 1, blk(1, 0) + i + 1])     # outer
        f.append([blk(0, 1) + i, blk(0, 1) + i + 1, blk(1, 1) + i + 1, blk(1, 1) + i])     # top
        f.append([blk(0, 0) + i, blk(1, 0) + i, blk(1, 0) + i + 1, blk(0, 0) + i + 1])     # bottom
    return U.mesh_object(name, np.array(v), f, coll, parent=parent, smooth_angle_deg=60.0, mat=mat)


def build_safety_gear(root: str, coll: Any, head_world: np.ndarray, frames: np.ndarray, cut: set) -> Any:
    """Safety goggles and ear defenders on the operator (the project's demo rule, params.json mission.vtol demo note:
    eye and ear protection), modelled around the asset's rest head (measured from render/human/human_lbs.npz: head
    x 0.00-0.21 m, |y| <= 0.095 m, top 1.75 m in the figure frame) and keyed per frame to the solved head bone
    (``head_world`` = figure matrix @ the head bone's global matrix, from drive_human)."""
    import sys
    tools = os.path.join(root, "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import human_figure as HF
    rig_np = HF.Rig(os.path.join(root, *HUMAN_SUBDIR, "human_lbs.npz"))
    rest_head = rig_np.rest[rig_np.ix["head"]]
    sat, lens, cup = _gear_materials()
    g = U.empty("safety_gear", coll, size=0.05)
    g.rotation_mode = "QUATERNION"
    # (built in the rest figure frame, then parented under an empty carrying inv(rest head))
    inner = U.empty("safety_gear_rest", coll, parent=g, size=0.02)
    Ri = np.linalg.inv(rest_head)
    from mathutils import Matrix
    inner.matrix_parent_inverse = Matrix.Identity(4)
    inner.matrix_basis = Matrix([list(r) for r in Ri])
    cx = 0.100
    # goggles: a curved lens across the eyes + a strap round the head
    _arc_band("gear_goggle_lens", coll, inner, cx, 0.118, 0.100, 1.648, 1.700, -1.15, 1.15, 0.006, lens)
    _arc_band("gear_goggle_frame_t", coll, inner, cx, 0.116, 0.098, 1.698, 1.706, -1.18, 1.18, 0.010, sat)
    _arc_band("gear_goggle_frame_b", coll, inner, cx, 0.116, 0.098, 1.642, 1.650, -1.18, 1.18, 0.010, sat)
    _arc_band("gear_goggle_strap", coll, inner, cx - 0.005, 0.107, 0.096, 1.664, 1.684, 1.10, 2 * math.pi - 1.10, 0.004, sat)
    # ear defenders: two cups on the ears and a band over the top of the head
    for sgn in (1.0, -1.0):
        v, f = _lathe([(0.0, 0.0), (0.040, 0.0), (0.044, 0.012), (0.042, 0.030), (0.030, 0.038), (0.0, 0.040)], n=28)
        v = v[:, [0, 2, 1]] * np.array([1.0, sgn, 1.0]) + np.array([0.095, sgn * 0.088, 1.640])
        o = U.mesh_object(f"gear_ear_cup_{'L' if sgn > 0 else 'R'}", v, f if sgn < 0 else [list(reversed(ff)) for ff in f],
                          coll, parent=inner, smooth_angle_deg=60.0, mat=cup)
        _ = o
    ts = np.linspace(-1.0, 1.0, 36) * (math.pi / 2 - 0.05)
    v, f = [], []
    for k, t in enumerate(ts):
        y, z = 0.126 * math.sin(t), 1.645 + 0.122 * math.cos(t)
        for dx in (-0.012, 0.012):
            for rr in (0.0, 0.006):
                v.append([0.095 + dx, y * (1 + rr / 0.126), 1.645 + (z - 1.645) * (1 + rr / 0.122)])
    for k in range(len(ts) - 1):
        b0, b1 = 4 * k, 4 * (k + 1)
        f += [[b0, b1, b1 + 2, b0 + 2], [b0 + 1, b0 + 3, b1 + 3, b1 + 1], [b0, b0 + 1, b1 + 1, b1], [b0 + 2, b1 + 2, b1 + 3, b0 + 3]]
    U.mesh_object("gear_ear_band", np.array(v), f, coll, parent=inner, smooth_angle_deg=60.0, mat=sat)
    # key the carrier: location / rotation of head_world per frame
    from mathutils import Matrix as Mx
    locs = head_world[:, :3, 3]
    qs = np.array([list(Mx([list(r) for r in hw[:3, :3]]).to_quaternion()) for hw in head_world])
    for k in range(1, qs.shape[0]):
        if float(np.dot(qs[k], qs[k - 1])) < 0:
            qs[k] = -qs[k]
    U.key_array(g, "location", frames, locs, cut)
    U.key_array(g, "rotation_quaternion", frames, qs, cut)
    return g


def drive_human(root: str, h: dict[str, Any], P: dict[str, np.ndarray], frames: np.ndarray, cut: set,
                warnings: list[str]) -> dict[str, Any]:
    """Solve every frame with tools/human_figure (README 'Exact parity') and key human_root + every pose bone."""
    tools = os.path.join(root, "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import human_figure as HF
    from render import pose as PO                   # this pipeline's push stance + grips (render/pose.py)
    rig_np = HF.Rig(os.path.join(root, *HUMAN_SUBDIR, "human_lbs.npz"))
    solver = HF.Solver(rig_np)
    hr, rig = h["root"], h["rig"]
    for k in ("push_ik", "look", "leg_ik"):
        if k in hr.keys():
            hr[k] = 0.0
    if rig.animation_data is not None:
        rig.animation_data.action = None
    n = frames.size
    B = rig_np.B
    quats = np.zeros((n, B, 4))
    pel = np.zeros((n, 3))
    ip = rig_np.ix["pelvis"]
    unreached = 0
    heads = np.zeros((n, 4, 4))
    ih = rig_np.ix["head"]
    for k in range(n):
        inp = PO.person_input(HF, P, k)
        # the pole never leaves the hands; this pipeline's planted push stance (render/pose.py)
        prm = PO.pose_params(HF, rig_np, inp)
        G, L = solver.solve(prm)
        heads[k] = HF.figure_matrix(inp.feet, inp.yaw) @ G[ih]
        last = getattr(solver, "last", None)
        if isinstance(last, dict):
            unreached += sum(1 for v in last.values() if hasattr(v, "reached") and not v.reached)
        for i in range(B):
            quats[k, i] = HF.quat_from_mat(L[i])
        pp = G[ip, :3, 3]
        pel[k] = rig_np.rest[ip, :3, :3].T @ (pp - rig_np.rest[ip, :3, 3])
    for i in range(B):
        q = quats[:, i, :].copy()
        for k in range(1, n):
            if float(np.dot(q[k], q[k - 1])) < 0:
                q[k] = -q[k]
        nm = rig_np.names[i]
        pb = rig.pose.bones.get(nm)
        if pb is None:
            continue
        pb.rotation_mode = "QUATERNION"
        U.key_array(rig, f'pose.bones["{nm}"].rotation_quaternion', frames, q, cut)
    U.key_array(rig, 'pose.bones["pelvis"].location', frames, pel, cut)
    hr.rotation_mode = "XYZ"
    U.key_array(hr, "location", frames, P["person_feet"], cut)
    U.key_array(hr, "rotation_euler", frames, np.column_stack([np.zeros(n), np.zeros(n), P["person_yaw"]]), cut)
    if unreached:
        warnings.append(f"person: {unreached} arm solves reported a grip out of reach (shown as solved, not faked)")
    return {"frames": n, "unreached": unreached, "head_world": heads}


# --------------------------------------------------------------------------- #
# the safety officer at the KILL switch (second figure, wide shots)
# --------------------------------------------------------------------------- #
def _transmitter(coll: Any) -> Any:
    """A hand-held kill-switch box: dark grey case, yellow collar, red mushroom button, stub antenna (origin = the
    box centre, +x forward, +z up)."""
    case, nb, out = U.new_material("kill_case")
    p = U.principled(nb, Base_Color=U.srgb("#2a2d31"), Roughness=0.5)
    nb.l(p.outputs[0], out.inputs["Surface"])
    red, nb, out = U.new_material("kill_button")
    p = U.principled(nb, Base_Color=U.srgb("#d42a1e"), Roughness=0.35, Coat_Weight=0.5)
    nb.l(p.outputs[0], out.inputs["Surface"])
    yel, nb, out = U.new_material("kill_collar")
    p = U.principled(nb, Base_Color=U.srgb("#f2c200"), Roughness=0.45)
    nb.l(p.outputs[0], out.inputs["Surface"])
    e = U.empty("kill_switch", coll, size=0.05)
    e.rotation_mode = "QUATERNION"
    U.mesh_object("kill_case", U.box_verts(-0.05, 0.05, -0.085, 0.085, -0.022, 0.022), U.box_faces(), coll, parent=e,
                  smooth_angle_deg=None, mat=case)
    v, f = _lathe([(0.0, 0.022), (0.03, 0.022), (0.03, 0.030), (0.0, 0.030)], n=24)
    U.mesh_object("kill_collar", v, f, coll, parent=e, smooth_angle_deg=40.0, mat=yel)
    v, f = _lathe([(0.0, 0.030), (0.014, 0.030), (0.016, 0.040), (0.021, 0.046), (0.018, 0.052), (0.0, 0.054)], n=24)
    U.mesh_object("kill_button", v, f, coll, parent=e, smooth_angle_deg=60.0, mat=red)
    v, f = _lathe([(0.0, 0.0), (0.004, 0.0), (0.003, 0.09), (0.0, 0.092)], n=10)
    o = U.mesh_object("kill_antenna", v, f, coll, parent=e, smooth_angle_deg=40.0, mat=case)
    o.location = (-0.03, 0.07, 0.02)
    return e


def drive_kill_operator(root: str, h: dict[str, Any], P: dict[str, np.ndarray], frames: np.ndarray, cut: set,
                        ko: dict, floor_z: float, coll: Any, warnings: list[str]) -> dict[str, Any]:
    """Pose the second copy of the CC0 human as the safety officer: standing at ``ko['xy']`` facing the vehicle (relaxed
    weight shift, the head following the vehicle), both hands holding the kill switch at the belly; the switch box is
    keyed to the midpoint of the solved hands.  Same solver as the operator (tools/human_figure)."""
    tools = os.path.join(root, "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import human_figure as HF
    rig_np = HF.Rig(os.path.join(root, *HUMAN_SUBDIR, "human_lbs.npz"))
    solver = HF.Solver(rig_np)
    hr, rig = h["root"], h["rig"]
    for k in ("push_ik", "look", "leg_ik"):
        if k in hr.keys():
            hr[k] = 0.0
    if rig.animation_data is not None:
        rig.animation_data.action = None
    n = frames.size
    B = rig_np.B
    quats = np.zeros((n, B, 4))
    pel = np.zeros((n, 3))
    ip = rig_np.ix["pelvis"]
    feet = np.array([float(ko["xy"][0]), float(ko["xy"][1]), floor_z])
    yaw = float(ko["yaw"])
    box_loc = np.zeros((n, 3))
    box_q = np.zeros((n, 4))
    heads = np.zeros((n, 4, 4))
    Mf = HF.figure_matrix(feet, yaw)
    for k in range(n):
        inp = HF.PersonInput(feet=feet, yaw=yaw, t=float(P["t"][k]) + 3.1, look_at=P["person_look"][k], push_w=0.0)
        prm = HF.person_params(rig_np, inp)
        for s_, sg in (("L", 1.0), ("R", -1.0)):
            prm.arms[s_] = HF.ArmGoal(wrist=np.array([0.25, sg * 0.07, 1.00]), pole=np.array([-1.0, sg * 0.7, -0.2]))
        prm.finger_mix = {"L": 0.6, "R": 0.6}
        G, L = solver.solve(prm)
        for i in range(B):
            quats[k, i] = HF.quat_from_mat(L[i])
        pp = G[ip, :3, 3]
        pel[k] = rig_np.rest[ip, :3, :3].T @ (pp - rig_np.rest[ip, :3, 3])
        heads[k] = Mf @ G[rig_np.ix["head"]]
        hl, hrr = G[rig_np.ix["hand_L"], :3, 3], G[rig_np.ix["hand_R"], :3, 3]
        mid = 0.5 * (hl + hrr) + np.array([0.06, 0.0, -0.01])
        box_loc[k] = Mf[:3, :3] @ mid + Mf[:3, 3]
        c, s2 = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
        box_q[k] = (c, 0.0, 0.0, s2)
    for i in range(B):
        q = quats[:, i, :].copy()
        for k in range(1, n):
            if float(np.dot(q[k], q[k - 1])) < 0:
                q[k] = -q[k]
        nm = rig_np.names[i]
        pb = rig.pose.bones.get(nm)
        if pb is None:
            continue
        pb.rotation_mode = "QUATERNION"
        U.key_array(rig, f'pose.bones["{nm}"].rotation_quaternion', frames, q, cut)
    U.key_array(rig, 'pose.bones["pelvis"].location', frames, pel, cut)
    hr.rotation_mode = "XYZ"
    hr.location = tuple(feet)
    hr.rotation_euler = (0.0, 0.0, yaw)
    box = _transmitter(coll)
    U.key_array(box, "location", frames, box_loc, cut)
    U.key_array(box, "rotation_quaternion", frames, box_q, cut)
    return {"xy": [float(x) for x in feet[:2]], "frames": n, "head_world": heads}
