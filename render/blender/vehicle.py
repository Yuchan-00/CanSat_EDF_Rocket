"""The vehicle in Blender: CadQuery STLs (mm, vehicle frame, Z = 0 at the duct exit plane) + a procedural
EDF unit + the launch pad, with printed-part materials (run inside Blender only).

Hierarchy (all keyed by ``anim.py``):
  veh_root (empty)            world pose: location = logged pos (exit-plane centre), rotation = logged q
    <module meshes>           static children in the vehicle frame (metres)
    vane_hinge_i (empty)      at (0, 0, hinge_z), Euler XYZ = (-delta_i, 0, theta_i)  ->  Rz(theta) Rx(-delta)
      vane_i, horn_i          mesh baked as T(0,0,-hinge_z) Rz(-theta_stl) v
  edf_*                       procedural EDF (housing, lip, stator, motor, rotor blades + blur disc)
  pad                         launch pad STL at the world origin (static)

Vane convention (tools/sim_video.py ``vane_local_matrix``, SPEC 8.2): the world transform of vane i is
  R(q) Rz(theta_i) T(0,0,hinge_z) Rx(-delta_i) T(0,0,-hinge_z) Rz(-theta_stl)
so delta > 0 moves the trailing edge (below the hinge) toward -e_theta and the vehicle feels +e_theta.
``render.plan.vane_euler`` holds the same numbers and tools/test_blender_render.py checks them against
sim_video.  When an STL is missing a labelled stand-in is built and reported (never silently).
"""
from __future__ import annotations

import json
import math
import os
from typing import Any

import bpy
import numpy as np

from . import bl_util as U
from . import materials as M

BODY_PARTS = (("propulsion_module", "propulsion"), ("intake_module", "intake"), ("avionics_module", "avionics"),
              ("payload_module", "payload"), ("nose_cone", "nose"), ("latch_arm", "dark"))
VANE_COLORS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")      # same slots as tools/sim_video.py


def _params(root: str) -> dict[str, Any]:
    with open(os.path.join(root, "params.json"), "r", encoding="utf-8") as f:
        return json.load(f)


def _ring(r0: float, r1: float, z0: float, z1: float, n: int = 96) -> tuple[np.ndarray, list[list[int]]]:
    """Closed annular tube (outer r1, inner r0) from z0 to z1."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    c, s = np.cos(a), np.sin(a)
    v = np.concatenate([np.column_stack([r * c, r * s, np.full(n, z)]) for r, z in ((r1, z0), (r1, z1), (r0, z1), (r0, z0))])
    f: list[list[int]] = []
    for k in range(4):
        b0, b1 = k * n, ((k + 1) % 4) * n
        for i in range(n):
            j = (i + 1) % n
            f.append([b0 + i, b0 + j, b1 + j, b1 + i])
    return v, f


def _lathe(profile: list[tuple[float, float]], n: int = 64, cap_top: bool = False, cap_bottom: bool = False) -> tuple[np.ndarray, list[list[int]]]:
    """Surface of revolution of (r, z) points about z."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    rows = [np.column_stack([r * np.cos(a), r * np.sin(a), np.full(n, z)]) for r, z in profile]
    v = np.concatenate(rows)
    f: list[list[int]] = []
    for k in range(len(profile) - 1):
        for i in range(n):
            j = (i + 1) % n
            f.append([k * n + i, k * n + j, (k + 1) * n + j, (k + 1) * n + i])
    if cap_bottom:
        f.append(list(range(n))[::-1])
    if cap_top:
        f.append(list(range((len(profile) - 1) * n, len(profile) * n)))
    return v, f


def _blade(r0: float, r1: float, zc: float, chord: float, pitch0_deg: float, pitch1_deg: float, t: float,
           nr: int = 8, nc: int = 5) -> tuple[np.ndarray, list[list[int]]]:
    """One twisted fan blade along +x (radial), a thin closed shell, centred on z = zc."""
    verts = []
    for side in (+1.0, -1.0):
        for i in range(nr + 1):
            r = r0 + (r1 - r0) * i / nr
            p = math.radians(pitch0_deg + (pitch1_deg - pitch0_deg) * i / nr)
            ch = chord * (1.0 - 0.25 * i / nr)
            for j in range(nc + 1):
                u = (j / nc - 0.5) * ch
                camber = 0.06 * ch * (1.0 - (2.0 * j / nc - 1.0) ** 2)
                thick = 0.5 * t * (1.0 - (2.0 * j / nc - 1.0) ** 2) ** 0.5
                w = camber + side * thick
                # chord line in the tangential (y) - axial (z) plane, rotated by the pitch
                y = u * math.cos(p) - w * math.sin(p)
                z = u * math.sin(p) + w * math.cos(p)
                verts.append((r, y, zc + z))
    v = np.array(verts)
    nrow = nc + 1
    half = (nr + 1) * nrow
    f: list[list[int]] = []
    for s, flip in ((0, False), (half, True)):
        for i in range(nr):
            for j in range(nc):
                a, b = s + i * nrow + j, s + i * nrow + j + 1
                c, d = s + (i + 1) * nrow + j + 1, s + (i + 1) * nrow + j
                f.append([a, d, c, b] if flip else [a, b, c, d])
    return v, f


def build_vehicle(root: str, coll: Any, warnings: list[str], quality: str = "final") -> dict[str, Any]:
    """Build the vehicle under ``coll``; returns handles: root, hinges, vanes, rotor, disc, cg, length."""
    P = _params(root)
    stl_dir = os.path.join(root, "cad", "exports", "cadquery", "stl")
    mats = M.vehicle_materials()
    veh = U.empty("veh_root", coll, size=0.3)
    veh.rotation_mode = "QUATERNION"
    parts = {}
    top = 0.0
    for name, mk in BODY_PARTS:
        p = os.path.join(stl_dir, f"{name}.stl")
        if os.path.exists(p):
            v, f = U.read_stl(p)
            o = U.mesh_object(name, v, f, coll, parent=veh, smooth_angle_deg=32.0, mat=mats[mk])
        else:
            warnings.append(f"{name}.stl missing: stand-in cylinder")
            spans = {"propulsion_module": (0.0, 0.19), "intake_module": (0.175, 0.348), "avionics_module": (0.333, 0.533),
                     "payload_module": (0.518, 0.716), "nose_cone": (0.70, 0.856), "latch_arm": (0.70, 0.71)}
            z0, z1 = spans[name]
            v, f = _lathe([(0.06, z0), (0.06, z1)], cap_top=True, cap_bottom=True)
            o = U.mesh_object(name + "_STANDIN", v, f, coll, parent=veh, mat=mats[mk])
        parts[name] = o
        top = max(top, float(np.max(np.asarray(o.bound_box)[:, 2])))
    # vanes + horns
    vp = P["vane"]
    n = int(vp["count"])
    thetas = [math.radians(float(vp["theta0_deg"]) + i * 360.0 / n) for i in range(n)]
    hinge_z = float(vp["hinge_z"]) * 1e-3
    vpath = os.path.join(stl_dir, "vane.stl")
    hpath = os.path.join(stl_dir, "vane_horn.stl")
    if os.path.exists(vpath):
        vv, vf = U.read_stl(vpath)
    else:
        warnings.append("vane.stl missing: stand-in plate")
        vv = np.array([[x, y, z] for x in (0.013, 0.043) for y in (-0.0015, 0.0015) for z in (0.0146, 0.0466)])
        vf = np.array([[0, 1, 3, 2], [4, 6, 7, 5], [0, 2, 6, 4], [1, 5, 7, 3], [0, 4, 5, 1], [2, 3, 7, 6]])
    theta_stl = math.atan2(float(np.mean(vv[:, 1])), float(np.mean(vv[:, 0])))
    Rm = np.array([[math.cos(-theta_stl), -math.sin(-theta_stl), 0], [math.sin(-theta_stl), math.cos(-theta_stl), 0], [0, 0, 1]])
    vv_local = vv @ Rm.T - np.array([0.0, 0.0, hinge_z])
    hv = hf = None
    if os.path.exists(hpath):
        hv, hf = U.read_stl(hpath)
        hv = hv @ Rm.T - np.array([0.0, 0.0, hinge_z])
    hinges, vanes = [], []
    for i, th in enumerate(thetas):
        h = U.empty(f"vane_hinge_{i + 1}", coll, parent=veh, loc=(0.0, 0.0, hinge_z), size=0.02)
        h.rotation_mode = "XYZ"
        h.rotation_euler = (0.0, 0.0, th)
        mv = M.vane_material(i, VANE_COLORS[i % 4])
        o = U.mesh_object(f"vane_{i + 1}", vv_local, vf, coll, parent=h, smooth_angle_deg=40.0, mat=mv)
        if hv is not None:
            U.mesh_object(f"vane_horn_{i + 1}", hv, hf, coll, parent=h, smooth_angle_deg=40.0, mat=mats["horn"])
        hinges.append(h)
        vanes.append(o)
    edf = build_edf(P, coll, veh, mats, quality)
    build_leads(coll, veh)
    build_joint_screws(P, coll, veh)
    # CG marker (focus / look-at target)
    zcg = 0.376
    cg = U.empty("veh_cg", coll, parent=veh, loc=(0.0, 0.0, zcg), size=0.05)
    return {"root": veh, "hinges": hinges, "vanes": vanes, "thetas": thetas, "hinge_z": hinge_z, "theta_stl": theta_stl,
            "length": top, "cg": cg, "parts": parts, **edf}


def build_edf(P: dict[str, Any], coll: Any, parent: Any, mats: dict[str, Any], quality: str) -> dict[str, Any]:
    """A 90 mm 12-blade EDF stand-in placed in the propulsion module's housing bore (the real unit is bought,
    not in the CAD): housing r 49 / 46 mm from z 105 to 186 mm, intake lip r 56 mm, 8 stator vanes, motor
    can, tail cone, 12 rotor blades at z 171 mm and a spinner.  Dimensions from params.edf where given."""
    e = P.get("edf", {})
    mm = 1e-3
    r_h = 0.5 * float(e.get("housing_od", 98.0)) * mm
    r_lip = 0.5 * float(e.get("lip_od", 112.0)) * mm
    r_fan = 0.5 * float(e.get("fan_d", 90.0)) * mm
    r_hub = 0.5 * float(e.get("hub_d_ASSUMED", 40.0)) * mm
    z_top = 0.190
    z_bot = z_top - float(e.get("len_lip_top_to_housing_bottom_ASSUMED", 85.0)) * mm
    out: dict[str, Any] = {}
    v, f = _ring(r_fan + 0.0008, r_h, z_bot, z_top - 0.004)
    U.mesh_object("edf_housing", v, f, coll, parent=parent, smooth_angle_deg=40.0, mat=mats["alu"])
    v, f = _lathe([(r_fan + 0.0008, z_top - 0.006), (r_fan + 0.003, z_top - 0.001), (r_fan + 0.007, z_top + 0.001),
                   (r_lip, z_top + 0.0005), (r_lip, z_top - 0.004), (r_h, z_top - 0.004)])
    U.mesh_object("edf_lip", v, f, coll, parent=parent, smooth_angle_deg=50.0, mat=mats["alu"])
    # motor can + tail cone + spinner (one lathe, dark anodised)
    z_rot = z_top - 0.019
    prof = [(0.0, z_bot - 0.030), (0.006, z_bot - 0.028), (0.012, z_bot - 0.010), (0.016, z_bot + 0.010),
            (0.017, z_rot - 0.012), (r_hub, z_rot - 0.008), (r_hub, z_rot + 0.006), (0.017, z_rot + 0.012),
            (0.011, z_rot + 0.019), (0.004, z_rot + 0.024), (0.0, z_rot + 0.025)]
    v, f = _lathe(prof, n=64)
    U.mesh_object("edf_core", v, f, coll, parent=parent, smooth_angle_deg=50.0, mat=mats["edf_core"])
    # stator vanes
    nst = 8
    for k in range(nst):
        a = 2 * math.pi * k / nst + math.radians(11.0)
        bv, bf = _blade(0.016, r_fan + 0.0005, z_bot + 0.032, 0.030, 82.0, 82.0, 0.0016, nr=4, nc=3)
        R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        U.mesh_object(f"edf_stator_{k}", bv @ R.T, bf, coll, parent=parent, smooth_angle_deg=60.0, mat=mats["edf_core"])
    # rotor: blades parented to a spinning empty, plus the blur disc
    rotor = U.empty("edf_rotor", coll, parent=parent, loc=(0, 0, 0), size=0.05)
    rotor.rotation_mode = "XYZ"
    nb = int(round(float(e.get("blades", 12)))) if "blades" in e else 12
    blades = []
    for k in range(nb):
        a = 2 * math.pi * k / nb
        bv, bf = _blade(r_hub - 0.002, r_fan - 0.0006, z_rot, 0.017, 52.0, 30.0, 0.0012, nr=8, nc=5)
        R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        blades.append(U.mesh_object(f"edf_blade_{k}", bv @ R.T, bf, coll, parent=rotor, smooth_angle_deg=70.0,
                                    mat=mats["rotor"]))
    v, f = _lathe([(r_hub - 0.001, z_rot), (r_fan - 0.0004, z_rot)], n=96)
    disc = U.mesh_object("edf_rotor_disc", v, f, coll, parent=parent, smooth_angle_deg=None, mat=mats["rotor_disc"])
    out["rotor"] = rotor
    out["rotor_disc"] = disc
    out["blades"] = blades
    return out


def build_leads(coll: Any, parent: Any, wire_r: float = 0.0016) -> list[Any]:
    """The three EDF motor phase leads (black silicone, ~3.2 mm OD) the open intake bay must show: up between the EDF
    housing and the duct wall at the 0-deg strut, over the lip, then curving in across the bay to the bulkhead's
    centre cable hole (params intake_module.cable_hole_d) on their way to the ESC in the avionics bay.  Not in the
    CAD (bought wiring); route illustrative, dimensions from the EDF datasheet class (12 AWG)."""
    m, nb, out = U.new_material("silicone_wire")
    p = U.principled(nb, Base_Color=(0.016, 0.016, 0.017, 1.0), Roughness=0.42, Specular_IOR_Level=0.5)
    nb.l(p.outputs[0], out.inputs["Surface"])
    objs = []
    path = ((0.0535, 0.00, 0.140), (0.0535, 0.00, 0.198), (0.0505, 0.02, 0.238), (0.039, 0.16, 0.288),
            (0.019, 0.26, 0.323), (0.006, 0.30, 0.343), (0.004, 0.30, 0.375))
    for j, (dth, dr) in enumerate(((-0.065, 0.0), (0.0, 0.0012), (0.065, 0.0))):
        cd = bpy.data.curves.new(f"edf_lead_{j}", "CURVE")
        cd.dimensions = "3D"
        cd.bevel_depth = wire_r
        cd.bevel_resolution = 2
        cd.resolution_u = 10
        sp = cd.splines.new("NURBS")
        sp.points.add(len(path) - 1)
        for i, (r, th, z) in enumerate(path):
            rr = r + dr + 0.0007 * math.sin(3.0 * i + j)            # a little slack / twist, deterministic
            tt = th + dth * (1.0 - 0.8 * i / (len(path) - 1))         # the three converge at the cable hole
            sp.points[i].co = (rr * math.cos(tt), rr * math.sin(tt), z + 0.002 * j * (i == len(path) - 1), 1.0)
        sp.order_u = 4
        sp.use_endpoint_u = True
        o = bpy.data.objects.new(f"lead_{j}", cd)
        o.data.materials.append(m)
        o.parent = parent
        U.link(o, coll)
        objs.append(o)
    return objs


# module-joint M3 screw holes, located on the CAD STLs (outer-surface hole clusters at theta 45 deg, round 1b):
# propulsion/intake, intake/avionics, avionics/payload; the nose is not screwed (recovery latch interface)
JOINT_SCREW_Z = (0.1825, 0.3404, 0.5254)


def build_joint_screws(P: dict[str, Any], coll: Any, parent: Any) -> list[Any]:
    """M3 button-head screws (ISO 7380: head 5.7 mm x 1.65 mm, black-oxide steel, hex socket) in the module-joint holes:
    params body.joint_screw_count (4) at body.joint_screw_theta0_deg (45) + k 360/count, radial, heads on the outer
    skin (r = body.od / 2).  Not printed parts, so not PLA (USER DECISION 1 covers printed parts only)."""
    b = P.get("body", {})
    n = int(b.get("joint_screw_count", 4))
    th0 = math.radians(float(b.get("joint_screw_theta0_deg", 45.0)))
    r_out = 0.5 * float(b.get("od", 120.0)) * 1e-3
    m, nb, out = U.new_material("steel_black_oxide")
    p = U.principled(nb, Base_Color=(0.035, 0.035, 0.038, 1.0), Metallic=0.85, Roughness=0.38)
    nb.l(p.outputs[0], out.inputs["Surface"])
    hm, nb, out = U.new_material("hex_socket_dark")
    p = U.principled(nb, Base_Color=(0.004, 0.004, 0.004, 1.0), Roughness=0.9)
    nb.l(p.outputs[0], out.inputs["Surface"])
    mm = 1e-3
    head_v, head_f = _lathe([(0.0, 1.65 * mm), (1.2 * mm, 1.62 * mm), (2.2 * mm, 1.3 * mm), (2.7 * mm, 0.7 * mm),
                             (2.85 * mm, 0.15 * mm), (2.85 * mm, -0.3 * mm), (0.0, -0.3 * mm)], n=24)
    a6 = np.linspace(0, 2 * math.pi, 6, endpoint=False) + math.pi / 6
    hx = 1.15 * mm                                                     # 2 mm across flats
    sock_v = np.array([[hx * math.cos(a), hx * math.sin(a), 1.66 * mm] for a in a6] + [[0.0, 0.0, 1.66 * mm]])
    sock_f = [[i, (i + 1) % 6, 6] for i in range(6)]
    objs = []
    for j, z in enumerate(JOINT_SCREW_Z):
        for k in range(n):
            th = th0 + 2 * math.pi * k / n
            # columns: local x = tangent x radial (= -z), y = tangent, z = radial outward (the head's axis)
            c, s_ = math.cos(th), math.sin(th)
            Rm = np.column_stack([np.cross([-s_, c, 0.0], [c, s_, 0.0]), [-s_, c, 0.0], [c, s_, 0.0]])
            off = np.array([r_out * c, r_out * s_, z])
            o = U.mesh_object(f"joint_screw_{j}{k}", head_v @ Rm.T + off, head_f, coll, parent=parent, smooth_angle_deg=60.0,
                              mat=m)
            U.mesh_object(f"joint_screw_socket_{j}{k}", sock_v @ Rm.T + off, sock_f, coll, parent=parent, smooth_angle_deg=None,
                          mat=hm)
            objs.append(o)
    return objs


def build_pad(root: str, coll: Any, warnings: list[str]) -> Any:
    p = os.path.join(root, "cad", "exports", "cadquery", "stl", "launch_pad.stl")
    mat = M.pad_material()
    if os.path.exists(p):
        v, f = U.read_stl(p)
        return U.mesh_object("launch_pad", v, f, coll, smooth_angle_deg=35.0, mat=mat)
    warnings.append("launch_pad.stl missing: stand-in disc")
    v, f = _lathe([(0.0, 0.0), (0.095, 0.0), (0.095, -0.005), (0.0, -0.005)])
    return U.mesh_object("launch_pad_STANDIN", v, f, coll, mat=mat)
