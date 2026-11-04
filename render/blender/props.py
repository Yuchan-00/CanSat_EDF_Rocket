"""Set dressing (run inside Blender): windsock, safety cones, Poly Haven props (fire extinguisher, crate).

Windsock: a 6 m (plan meta 'windsock_h') galvanised mast with a swivel at the top and a 1.2 m orange / white aviation sock of 5 segments
(0.30 -> 0.16 m diameter).  ``sock_yaw`` (the direction the air goes) turns the swivel; ``sock_droop[i]`` bends
segment i down about its local y axis (render.plan.windsock: full extension at 7.7 m/s ~ 15 kt).  It is a cue for
the logged wind, not a simulated fabric.

Props are imported from the downloaded glTF files; a missing file is reported and skipped (never faked).
"""
from __future__ import annotations

import math
import os
from typing import Any, Sequence

import bpy
import numpy as np

from . import bl_util as U
from .vehicle import _lathe

SOCK_LEN = 1.2
SOCK_R0, SOCK_R1 = 0.15, 0.08
POLE_H = 6.0
SOCK_ARM = 0.40          # swivel arm: the hoop stands 0.4 m out from the mast (round 1: the mast ran through the mouth)


def _fabric(name: str, hexcol: str) -> Any:
    m, nb, out = U.new_material(name)
    tc = nb.n("ShaderNodeTexCoord")
    noise = nb.n("ShaderNodeTexNoise", inputs={"Scale": 160.0, "Detail": 2.0})
    nb.l(tc.outputs["Object"], noise.inputs["Vector"])
    bump = nb.n("ShaderNodeBump", inputs={"Strength": 0.15, "Distance": 0.001})
    nb.l(noise.outputs["Fac"], bump.inputs["Height"])
    p = U.principled(nb, Base_Color=U.srgb(hexcol), Roughness=0.7, Sheen_Weight=0.35)
    nb.l(bump.outputs["Normal"], p.inputs["Normal"])
    tl = nb.n("ShaderNodeBsdfTranslucent")
    tl.inputs["Color"].default_value = U.srgb(hexcol)
    mix = nb.n("ShaderNodeMixShader", inputs={0: 0.18})
    nb.l(p.outputs[0], mix.inputs[1])
    nb.l(tl.outputs[0], mix.inputs[2])
    nb.l(mix.outputs[0], out.inputs["Surface"])
    return m


def _galv() -> Any:
    m, nb, out = U.new_material("galvanised_steel")
    tc = nb.n("ShaderNodeTexCoord")
    noise = nb.n("ShaderNodeTexNoise", inputs={"Scale": 30.0, "Detail": 6.0})
    nb.l(tc.outputs["Object"], noise.inputs["Vector"])
    rr = nb.n("ShaderNodeMapRange", inputs={1: 0.3, 2: 0.7, 3: 0.32, 4: 0.55})
    nb.l(noise.outputs["Fac"], rr.inputs[0])
    p = U.principled(nb, Base_Color=(0.55, 0.56, 0.57, 1.0), Metallic=1.0)
    nb.l(rr.outputs[0], p.inputs["Roughness"])
    nb.l(p.outputs[0], out.inputs["Surface"])
    return m


def build_windsock(coll: Any, xy: tuple[float, float], ground_z: float, mast_h: float = POLE_H) -> dict[str, Any]:
    """Windsock on a ``mast_h`` galvanised mast (the plan's meta 'windsock_h', 6 m airfield type)."""
    POLE_H = mast_h  # noqa: N806
    galv = _galv()
    base = U.empty("windsock_base", coll, loc=(xy[0], xy[1], ground_z), size=0.2)
    v, f = _lathe([(0.0, 0.0), (0.045, 0.0), (0.038, 0.4 * POLE_H), (0.026, POLE_H - 0.05), (0.018, POLE_H), (0.0, POLE_H + 0.01)], n=24)
    U.mesh_object("windsock_pole", v, f, coll, parent=base, smooth_angle_deg=50.0, mat=galv)
    v, f = _lathe([(0.0, -0.02), (0.09, -0.02), (0.09, 0.01), (0.0, 0.01)], n=24)
    U.mesh_object("windsock_footing", v, f, coll, parent=base, smooth_angle_deg=40.0, mat=galv)
    swivel = U.empty("windsock_swivel", coll, parent=base, loc=(0.0, 0.0, POLE_H - 0.12), size=0.15)
    swivel.rotation_mode = "XYZ"
    # throat ring (steel hoop the sock is sewn to) on a swivel arm, SOCK_ARM downwind of the mast
    a = np.linspace(0, 2 * math.pi, 48, endpoint=False)
    ring_v, ring_f = [], []
    hx = SOCK_ARM
    for k, (rr, xx) in enumerate(((SOCK_R0 + 0.006, hx), (SOCK_R0 + 0.006, hx + 0.02), (SOCK_R0 - 0.002, hx + 0.02),
                                  (SOCK_R0 - 0.002, hx))):
        ring_v += [[xx, rr * math.cos(t), rr * math.sin(t)] for t in a]
    for k in range(4):
        for i in range(48):
            j = (i + 1) % 48
            ring_f.append([k * 48 + i, k * 48 + j, ((k + 1) % 4) * 48 + j, ((k + 1) % 4) * 48 + i])
    U.mesh_object("windsock_hoop", np.array(ring_v), ring_f, coll, parent=swivel, smooth_angle_deg=60.0, mat=galv)
    # the swivel arm (a horizontal tube) and two braces from the sleeve to the top / bottom of the hoop
    v, f = _lathe([(0.0, 0.0), (0.011, 0.0), (0.011, hx), (0.0, hx)], n=16)
    v = v[:, [2, 1, 0]] * np.array([1.0, 1.0, 1.0])            # lathe axis z -> x
    U.mesh_object("windsock_arm", v, [list(reversed(ff)) for ff in f], coll, parent=swivel, smooth_angle_deg=50.0, mat=galv)
    for nm, zz in (("windsock_rod_top", 1.0), ("windsock_rod_bot", -1.0)):
        p0 = np.array([0.0, 0.0, zz * (SOCK_R0 + 0.02)])
        p1 = np.array([hx, 0.0, zz * SOCK_R0])
        d = p1 - p0
        L = float(np.linalg.norm(d))
        v, f = _lathe([(0.0, 0.0), (0.006, 0.0), (0.006, L), (0.0, L)], n=10)
        ang = math.atan2(float(d[2]), float(d[0]))                # rotate +z onto d (in the x-z plane)
        c_, s_ = math.cos(math.pi / 2 - ang), math.sin(math.pi / 2 - ang)
        Ry = np.array([[c_, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c_]])
        U.mesh_object(nm, v @ Ry.T + p0, f, coll, parent=swivel, smooth_angle_deg=50.0, mat=galv)
    v, f = _lathe([(0.0, -SOCK_R0 - 0.01), (0.032, -SOCK_R0 - 0.01), (0.032, SOCK_R0 + 0.01), (0.0, SOCK_R0 + 0.01)], n=24)
    U.mesh_object("windsock_sleeve", v, f, coll, parent=swivel, smooth_angle_deg=50.0, mat=galv)
    orange, white = _fabric("sock_orange", "#f0581e"), _fabric("sock_white", "#efefe9")
    n = 5
    seg_len = SOCK_LEN / n
    segs = []
    parent = swivel
    for i in range(n):
        e = U.empty(f"windsock_seg_{i}", coll, parent=parent, loc=(hx + 0.01 if i == 0 else seg_len, 0.0, 0.0), size=0.08)
        e.rotation_mode = "XYZ"
        r0 = SOCK_R0 + (SOCK_R1 - SOCK_R0) * i / n
        r1 = SOCK_R0 + (SOCK_R1 - SOCK_R0) * (i + 1) / n
        # fabric tube along +x with a slight waist and longitudinal cloth folds (art review round 2: a rigid, wrinkle-
        # free plastic cone): the radius ripples +-3.5 % in 9 folds that drift around the tube from segment to
        # segment, strongest mid-segment; the lower side sags a little (the fabric's weight)
        na, nl = 64, 6
        verts, faces = [], []
        aa = np.linspace(0, 2 * math.pi, na, endpoint=False)
        for k in range(nl + 1):
            u = k / nl
            xx = u * (seg_len + 0.004)
            rr = r0 + (r1 - r0) * u
            rr *= 1.0 - 0.015 * math.sin(math.pi * u)
            fold = 0.035 * (0.4 + 0.6 * math.sin(math.pi * u))
            for t in aa:
                rt = rr * (1.0 + fold * math.sin(9.0 * t + 1.7 * i + 2.3 * u))
                zz = rt * math.sin(t) - 0.012 * rr / SOCK_R0 * math.sin(math.pi * u) * (1.0 if math.sin(t) < 0 else 0.3)
                verts.append([xx, rt * math.cos(t), zz])
        for k in range(nl):
            for j in range(na):
                jj = (j + 1) % na
                faces.append([k * na + j, k * na + jj, (k + 1) * na + jj, (k + 1) * na + j])
        o = U.mesh_object(f"windsock_fabric_{i}", np.array(verts), faces, coll, parent=e, smooth_angle_deg=80.0,
                          mat=orange if i % 2 == 0 else white)
        sol = o.modifiers.new("thickness", "SOLIDIFY")
        sol.thickness = 0.002
        segs.append(e)
        parent = e
    objs = [o for o in coll.all_objects if o.name.startswith("windsock")]
    return {"base": base, "swivel": swivel, "segs": segs, "objects": objs}


def cone_material() -> Any:
    m, nb, out = U.new_material("safety_cone")
    tc = nb.n("ShaderNodeTexCoord")
    sep = nb.n("ShaderNodeSeparateXYZ")
    nb.l(tc.outputs["Object"], sep.inputs[0])
    band = nb.math("MULTIPLY", nb.math("GREATER_THAN", sep.outputs[2], 0.17), nb.math("LESS_THAN", sep.outputs[2], 0.24))
    col = nb.mix_rgb(band, U.srgb("#ff4f12"), U.srgb("#e6e6e3"))
    rough = nb.n("ShaderNodeMix", data_type="FLOAT")
    nb.l(band, rough.inputs["Factor"])
    rough.inputs[2].default_value = 0.45
    rough.inputs[3].default_value = 0.3
    p = U.principled(nb)
    nb.l(col, p.inputs["Base Color"])
    nb.l(rough.outputs[0], p.inputs["Roughness"])
    nb.l(p.outputs[0], out.inputs["Surface"])
    return m


def build_cones(coll: Any, ground_z: float, positions: list[tuple[float, float]]) -> list[Any]:
    """45 cm PVC traffic cones (square base 28 cm) marking the keep-out circle."""
    mat = cone_material()
    base_m, nb, out = U.new_material("cone_base_black")
    p = U.principled(nb, Base_Color=(0.02, 0.02, 0.02, 1.0), Roughness=0.7)
    nb.l(p.outputs[0], out.inputs["Surface"])
    out_objs = []
    prof = [(0.0, 0.0315), (0.125, 0.0315), (0.118, 0.04), (0.035, 0.44), (0.028, 0.452), (0.0, 0.452)]
    cv, cf = _lathe(prof, n=40)
    bv = U.box_verts(-0.14, 0.14, -0.14, 0.14, 0.0, 0.032)
    for k, (x, y) in enumerate(positions):
        e = U.empty(f"cone_{k}", coll, loc=(x, y, ground_z), size=0.1)
        e.rotation_euler = (0.0, 0.0, 0.37 * k)
        U.mesh_object(f"cone_{k}_body", cv, cf, coll, parent=e, smooth_angle_deg=50.0, mat=mat)
        b = U.mesh_object(f"cone_{k}_base", bv, U.box_faces(), coll, parent=e, smooth_angle_deg=None, mat=base_m)
        bev = b.modifiers.new("bevel", "BEVEL")
        bev.width = 0.01
        bev.segments = 2
        out_objs.append(e)
    return out_objs


def import_gltf(path: str, coll: Any, loc: tuple[float, float, float], rot_z: float, warnings: list[str],
                name: str) -> Any:
    if not os.path.exists(path):
        warnings.append(f"prop {name}: {path} missing, skipped")
        return None
    before = set(bpy.data.objects)
    try:
        bpy.ops.import_scene.gltf(filepath=path)
    except (RuntimeError, AttributeError) as e:
        warnings.append(f"prop {name}: glTF import failed ({e}), skipped")
        return None
    new = [o for o in bpy.data.objects if o not in before]
    root = U.empty(f"prop_{name}", coll, loc=loc, size=0.2)
    root.rotation_euler = (0.0, 0.0, rot_z)
    for o in new:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        coll.objects.link(o)
        if o.parent is None:
            o.parent = root
    return root


def prop_angle(avoid_xy: np.ndarray | None, default_deg: float = 221.0, r: float = 2.2) -> float:
    """Azimuth (deg) around the pad for the extinguisher / crate group: the default, unless the person comes within
    1.2 m of it - then the 15-deg step whose position is farthest from every person position."""
    if avoid_xy is None or len(avoid_xy) == 0:
        return default_deg
    pts = np.asarray(avoid_xy, float)[:, :2]

    def clearance(a_deg: float) -> float:
        p = np.array([r * math.cos(math.radians(a_deg)), r * math.sin(math.radians(a_deg))])
        return float(np.min(np.hypot(*(pts - p).T)))
    if clearance(default_deg) >= 1.2:
        return default_deg
    return max(np.arange(0.0, 360.0, 15.0), key=clearance)


def frame_safe(base_xyz: Sequence[float], radius: float, height: float, cams: dict | None,
               side: float = 0.05, bottom: float = 0.85) -> bool:
    """render.qa.frame_safe (pure; tested in tools/test_blender_render.py)."""
    import sys
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if root not in sys.path:
        sys.path.insert(0, root)
    from render import qa as QA
    return QA.frame_safe(base_xyz, radius, height, cams, side, bottom)


def behind_subject(pts: np.ndarray, views: np.ndarray | None, deg: float = 11.0) -> np.ndarray:
    """Boolean per point: would it sit right behind the vehicle (within ``deg`` of the camera -> vehicle line, farther
    than the vehicle) in any of ``views`` (K, 4) = camera xy + vehicle xy?  A red extinguisher or a cone straight
    behind the black body reads as part of it."""
    p = np.asarray(pts, float).reshape(-1, 2)
    out = np.zeros(p.shape[0], bool)
    if views is None or len(views) == 0:
        return out
    c = np.cos(np.radians(deg))
    for cx, cy, vx, vy in np.asarray(views, float):
        a = np.array([vx - cx, vy - cy])
        la = float(np.linalg.norm(a))
        if la < 1e-6:
            continue
        b = p - np.array([cx, cy])
        lb = np.linalg.norm(b, axis=1)
        out |= ((b @ a) / np.maximum(lb * la, 1e-9) > c) & (lb > la)
    return out


def stage_prop_sites(hall_fit: dict, avoid_xy: np.ndarray | None, n: int = 2,
                     views: np.ndarray | None = None) -> list[tuple[float, float]]:
    """Indoor: world xy for the extinguisher / crate on the stage - >= 0.45 m inside the stage (render/hall.py), >= 1.7 m
    from the pad, >= 1.1 m from every operator position, preferring spots ~0.6 m in front of a curtain (where stage
    kit stands), the second at least 0.7 m from the first."""
    import sys
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if root not in sys.path:
        sys.path.insert(0, root)
    from render import hall as HL
    h = HL.Hall(rot=float(hall_fit["rot_rad"]), veh_img=np.array(hall_fit["veh_img"], float), floor_z=0.0)
    x0, x1, y0, y1 = HL.STAGE
    cand = np.array([[x, y] for x in np.arange(x0 + 0.3, x1, 0.25) for y in np.arange(y0 + 0.3, y1, 0.25)])
    w = h.to_world(cand)
    marg = HL.stage_margin(cand)
    ok = (marg >= 0.45) & (np.hypot(w[:, 0], w[:, 1]) >= 1.7)
    if avoid_xy is not None and len(avoid_xy):
        av = np.asarray(avoid_xy, float)[:, :2]
        dmin = np.min(np.hypot(w[:, None, 0] - av[None, :, 0], w[:, None, 1] - av[None, :, 1]), axis=1)
        ok &= dmin >= 1.1
    ok &= ~behind_subject(w, views)
    score = -np.abs(marg - 0.6) - 0.05 * np.hypot(w[:, 0], w[:, 1])
    out: list[tuple[float, float]] = []
    for i in np.argsort(-score):
        if not ok[i]:
            continue
        if all(math.hypot(w[i, 0] - p[0], w[i, 1] - p[1]) >= 0.7 for p in out):
            out.append((float(w[i, 0]), float(w[i, 1])))
        if len(out) >= n:
            break
    return out


def build_tape_end(coll: Any, z: float, root_xy: Sequence[float], direction: Sequence[float], seg_m: float = 0.04,
                   n: int = 6, width: float = 0.05) -> list[Any]:
    """The loose, peeled-up end of the hazard tape at a corner of the keep-out square: a chain of ``n`` segment empties
    (each bends about its local y; + = up) carrying 5 cm wide strips with the tape's stripes; keyed from the plan's
    ``tape_lift`` (render.plan.tape_flutter: the wall jet lifts and flutters it)."""
    m = bpy.data.materials.get("hazard_tape")
    ang = math.atan2(float(direction[1]), float(direction[0]))
    base = U.empty("tape_end_root", coll, loc=(float(root_xy[0]), float(root_xy[1]), z), size=0.05)
    base.rotation_euler = (0.0, 0.0, ang)
    segs, parent = [], base
    hw = 0.5 * width
    for i in range(n):
        e = U.empty(f"tape_end_seg_{i}", coll, parent=parent, loc=(0.0 if i == 0 else seg_m, 0.0, 0.0), size=0.02)
        e.rotation_mode = "XYZ"
        v = np.array([[0.0, -hw, 0.0004], [seg_m + 0.001, -hw, 0.0004], [seg_m + 0.001, hw, 0.0004], [0.0, hw, 0.0004]])
        o = U.mesh_object(f"tape_end_strip_{i}", v, [[0, 1, 2, 3]], coll, parent=e, smooth_angle_deg=None, mat=m)
        sol = o.modifiers.new("thickness", "SOLIDIFY")
        sol.thickness = 0.0004
        segs.append(e)
        parent = e
    return segs


def build_props(asset_dir: str, coll: Any, ground_z: float, warnings: list[str], indoor: bool,
                avoid_xy: np.ndarray | None = None, hall_fit: dict | None = None,
                views: np.ndarray | None = None, cams: dict | None = None) -> dict[str, Any]:
    """Fire extinguisher + crate beside the launch area (kept clear of the person's positions ``avoid_xy``) and safety
    cones: outdoors four on the 3 m keep-out circle; indoors on the stage (``stage_prop_sites``) and cones at the
    corners of the taped keep-out square (only those >= 0.8 m from the operator's positions)."""
    out: dict[str, Any] = {}
    ext = os.path.join(asset_dir, "korean_fire_extinguisher_01", "korean_fire_extinguisher_01_2k.gltf")
    crate = os.path.join(asset_dir, "plastic_crate_01", "plastic_crate_01_1k.gltf")
    zf = ground_z - (0.018 if indoor else 0.0)
    if indoor and hall_fit:
        sites = stage_prop_sites(hall_fit, avoid_xy, n=6, views=views)
        sites = [p for p in sites if frame_safe((p[0], p[1], zf), 0.25, 0.6, cams)][:2]
        if len(sites) >= 1:
            out["extinguisher"] = import_gltf(ext, coll, (sites[0][0], sites[0][1], zf), 0.6, warnings, "extinguisher")
        if len(sites) >= 2:
            out["crate"] = import_gltf(crate, coll, (sites[1][0], sites[1][1], zf), 0.3, warnings, "crate")
        c = 0.80 + 0.30
        pos = [(c, c), (-c, c), (-c, -c), (c, -c)]
        if avoid_xy is not None and len(avoid_xy):
            av = np.asarray(avoid_xy, float)[:, :2]
            pos = [p for p in pos if float(np.min(np.hypot(av[:, 0] - p[0], av[:, 1] - p[1]))) >= 0.8]
        pos = [p for p, b in zip(pos, behind_subject(np.array(pos).reshape(-1, 2), views)) if not b]
        pos = [p for p in pos if frame_safe((p[0], p[1], zf), 0.17, 0.46, cams)]
        out["cones"] = build_cones(coll, zf, pos[:3])
        return out
    # outdoors farther out and >= 2 m from every camera position (round 1b: an extinguisher 1.5 m from the lens
    # stood cut in half at the frame edge of the lift-off shot)
    r1, r2 = (1.9, 3.3) if indoor else (2.6, 4.0)
    min_clear = 1.2 if indoor else 2.0
    a_deg = prop_angle(avoid_xy)
    found = False
    for da in [0.0] + [sg * k * 15.0 for k in range(1, 12) for sg in (1, -1)]:
        aa = math.radians(a_deg + da)
        pts = np.array([[r1 * math.cos(aa), r1 * math.sin(aa)], [r2 * math.cos(aa + 0.12), r2 * math.sin(aa + 0.12)]])
        clear = avoid_xy is None or len(avoid_xy) == 0 or float(np.min(np.hypot(
            *(np.asarray(avoid_xy, float)[:, None, :2] - pts[None, :, :]).transpose(2, 0, 1)))) >= min_clear
        in_frame_ok = frame_safe((pts[0, 0], pts[0, 1], zf), 0.25, 0.6, cams) and \
            frame_safe((pts[1, 0], pts[1, 1], zf), 0.35, 0.4, cams)
        if clear and not np.any(behind_subject(pts, views, 14.0)) and in_frame_ok:
            a_deg += da
            found = True
            break
    a = math.radians(a_deg)
    if found:
        out["extinguisher"] = import_gltf(ext, coll, (r1 * math.cos(a), r1 * math.sin(a), zf), 0.6, warnings, "extinguisher")
        out["crate"] = import_gltf(crate, coll, (r2 * math.cos(a + 0.12), r2 * math.sin(a + 0.12),
                                                 zf - (0.0 if indoor else 0.01)), a + 0.25, warnings, "crate")
    else:
        warnings.append("props: no extinguisher / crate site clear of the operator and the frame edges - omitted")
    r = 2.4 if indoor else 3.0
    pos = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (20, 110, 200, 290)]
    pos = [p for p, b in zip(pos, behind_subject(np.array(pos), views)) if not b]
    pos = [p for p in pos if frame_safe((p[0], p[1], zf), 0.17, 0.46, cams)]
    out["cones"] = build_cones(coll, ground_z - (0.018 if indoor else 0.0), pos)
    return out
