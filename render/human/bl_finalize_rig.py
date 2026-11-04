"""Step 5 (headless Blender): turn ``build/rig_weighted.blend`` into ``render/human/human_rigged.blend`` for the Blender
render pipeline: placement root, IK targets + constraints (driven by custom properties, no Python expressions, so
nothing needs "auto-run scripts"), pose actions solved by the SAME numpy runtime as the PyVista videos
(``tools/human_figure.py``, imported here: it needs only numpy), and the satin material.

Run: blender --background --factory-startup --python bl_finalize_rig.py -- BUILD_DIR HUMAN_LBS_NPZ OUT_BLEND

Objects (collection ``human``):
  human_root   EMPTY   placement: location = the point between the feet on the floor, rotation Z = facing (the figure
                       faces its local +X).  Custom props: push_ik (0..1, arms on the pole IK), look (0..1, head look-at),
                       leg_ik (0..1, feet on ik_foot_* instead of the action's FK legs).
  human_rig    ARMATURE  child of human_root, rotated +90 deg about Z (Blender bone convention: built facing -Y).
  human_body   MESH    Armature modifier, then Subdivision Surface (viewport 1, render 2); material ``human_satin``.
  pole_grip    EMPTY   child of human_root at the pole's held end (sim_video.FIGURE_HANDS = 0.45 m ahead, 1.25 m up);
                       Track To pole_tip (local +Y along the pole, Z up), so moving pole_tip swings the pole.
  pole_tip     EMPTY   world: the push contact point (pad centre).
  ik_hand_R/L  EMPTY   children of pole_grip: wrist position + hand rotation of the rear (R) / front (L) grip (the
                       pole passes through each fist's grip cavity); forearm IK + hand Copy Rotation follow them.
  pole_elbow_R/L EMPTY elbow pole targets (children of human_root).
  ik_foot_R/L, pole_knee_R/L  EMPTY  leg IK targets at the push stance (children of human_root; used when leg_ik = 1).
  look_at      EMPTY   world: what the head looks at (neck 40 % + head Damped Track).
Actions: human_rest, human_idle, human_watch_up, human_push (FK reference of the push, arms also IK-ready),
human_walk (16 keys per gait cycle, cyclic).
"""
import json
import math
import os
import sys

import bpy
import numpy as np
from mathutils import Matrix, Vector

argv = sys.argv[sys.argv.index("--") + 1:]
BUILD, NPZ, OUT = argv[0], argv[1], argv[2]
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "tools"))
import human_figure as HF  # noqa: E402

sys.path.insert(0, HERE)
import bl_human_pose as BHP  # noqa: E402

FIGURE_HANDS = np.array([0.45, 0.0, 1.25])         # tools/sim_video.py FIGURE_HANDS
POLE_GRIP_M = (-0.12, 0.10)                         # tools/sim_video.py POLE_GRIP_M (rear = right, front = left)
TIP = np.array([1.55, 0.0, 2.0])                    # reference contact: 1.1 m beyond the held end, 2 m up

bpy.ops.wm.open_mainfile(filepath=os.path.join(BUILD, "rig_weighted.blend"))
scene = bpy.context.scene
rig = bpy.data.objects["human_rig"]
body = bpy.data.objects["human_body"]
rigd = HF.Rig(NPZ)
sol = HF.Solver(rigd)

col = bpy.data.collections.new("human")
scene.collection.children.link(col)
for ob in (rig, body):
    for c in list(ob.users_collection):
        c.objects.unlink(ob)
    col.objects.link(ob)


def empty(name, parent=None, size=0.08, shape="PLAIN_AXES", M=None):
    e = bpy.data.objects.new(name, None)
    e.empty_display_type = shape
    e.empty_display_size = size
    col.objects.link(e)
    if parent is not None:
        e.parent = parent
        e.matrix_parent_inverse = Matrix.Identity(4)
    if M is not None:
        e.matrix_basis = Matrix([list(r) for r in np.asarray(M, dtype=float)])
    return e


root = empty("human_root", size=0.3, shape="ARROWS")
root["push_ik"] = 0.0
root["look"] = 1.0
root["leg_ik"] = 0.0
for k, desc in (("push_ik", "0 = arms from the action (idle / walk), 1 = hands on the pole (IK to ik_hand_*)"),
                ("look", "head / neck look-at influence toward look_at"),
                ("leg_ik", "0 = legs from the action, 1 = feet on ik_foot_* (push stance)")):
    ui = root.id_properties_ui(k)
    ui.update(min=0.0, max=1.0, soft_min=0.0, soft_max=1.0, description=desc)
rig.parent = root
rig.matrix_parent_inverse = Matrix.Identity(4)
rig.location = (0.0, 0.0, 0.0)
rig.rotation_mode = "XYZ"
rig.rotation_euler = (0.0, 0.0, math.pi / 2)                    # bones built facing -Y -> the figure faces +X

# ---- reference push solve (figure frame = human_root frame) ----
wdir = HF.unit(TIP - FIGURE_HANDS)
grips_ref = (FIGURE_HANDS + wdir * POLE_GRIP_M[0], FIGURE_HANDS + wdir * POLE_GRIP_M[1])
push_in = HF.PersonInput(feet=np.zeros(3), yaw=0.0, look_at=TIP, push_w=1.0, grips=grips_ref, pole_dir=wdir)
P_push = HF.person_params(rigd, push_in)
G_push, L_push = sol.solve(P_push)
Y = wdir
Zp = HF.perp(np.array([0.0, 0.0, 1.0]), Y)
pole_frame = np.eye(4)
pole_frame[:3, :3] = np.column_stack([HF.cross(Y, Zp), Y, Zp])
pole_frame[:3, 3] = FIGURE_HANDS
pg = empty("pole_grip", root, 0.12, "ARROWS", pole_frame)
tip = empty("pole_tip", None, 0.08, "SPHERE")
tip.location = Vector(TIP.tolist())
look = empty("look_at", None, 0.1, "SPHERE")
look.location = Vector(TIP.tolist())
tt = pg.constraints.new("TRACK_TO")
tt.target, tt.track_axis, tt.up_axis = tip, "TRACK_Y", "UP_Z"
# pole_grip's own rotation now comes from the constraint: keep only its location
pg.matrix_basis = Matrix.Translation(Vector(FIGURE_HANDS.tolist()))
ix = rigd.ix
for s, sg in (("L", 1.0), ("R", -1.0)):
    Gh = G_push[ix[f"hand_{s}"]]
    empty(f"ik_hand_{s}", pg, 0.06, "CUBE", np.linalg.inv(pole_frame) @ Gh)
    E = G_push[ix[f"forearm_{s}"], :3, 3]
    empty(f"pole_elbow_{s}", root, 0.04, "SPHERE", Matrix.Translation(Vector((E + 0.35 * HF.unit(
        np.array([-0.25, sg * 0.85, -0.9]))).tolist())))
    Gf = G_push[ix[f"foot_{s}"]]
    empty(f"ik_foot_{s}", root, 0.06, "CUBE", Gf)
    K = G_push[ix[f"shin_{s}"], :3, 3]
    yawf = math.radians(P_push.legs[s].yaw_deg)
    empty(f"pole_knee_{s}", root, 0.04, "SPHERE", Matrix.Translation(Vector((K + 0.4 * np.array(
        [math.cos(yawf), math.sin(yawf), 0.0])).tolist())))


# ---- constraints with drivers ----
def drive(con, prop):
    fc = con.driver_add("influence")
    d = fc.driver
    d.type = "SUM"
    v = d.variables.new()
    v.name = "w"
    v.type = "SINGLE_PROP"
    v.targets[0].id_type = "OBJECT"
    v.targets[0].id = root
    v.targets[0].data_path = f'["{prop}"]'


pb = rig.pose.bones
for b in pb:
    b.rotation_mode = "QUATERNION"
ik_cons = {}
for s in ("L", "R"):
    c = pb[f"forearm_{s}"].constraints.new("IK")
    c.name = "push_IK"
    c.target = bpy.data.objects[f"ik_hand_{s}"]
    c.pole_target = bpy.data.objects[f"pole_elbow_{s}"]
    c.chain_count = 2
    c.use_tail = True
    drive(c, "push_ik")
    ik_cons[("arm", s)] = c
    c = pb[f"hand_{s}"].constraints.new("COPY_ROTATION")
    c.name = "push_hand"
    c.target = bpy.data.objects[f"ik_hand_{s}"]
    drive(c, "push_ik")
    c = pb[f"shin_{s}"].constraints.new("IK")
    c.name = "leg_IK"
    c.target = bpy.data.objects[f"ik_foot_{s}"]
    c.pole_target = bpy.data.objects[f"pole_knee_{s}"]
    c.chain_count = 2
    c.use_tail = True
    drive(c, "leg_ik")
    ik_cons[("leg", s)] = c
    c = pb[f"foot_{s}"].constraints.new("COPY_ROTATION")
    c.name = "leg_foot"
    c.target = bpy.data.objects[f"ik_foot_{s}"]
    drive(c, "leg_ik")
for bn, infl in (("neck", 0.4), ("head", 1.0)):
    c = pb[bn].constraints.new("DAMPED_TRACK")
    c.name = "look_at"
    c.target = look
    c.track_axis = "TRACK_Z"                         # the face looks along the head bone's +Z (roll hint = forward)
    drive(c, "look")
    # the neck takes a share: the SUM driver gives `look`; the driver F-curve's Generator scales it (no Python)
    for fcv in rig.animation_data.drivers if rig.animation_data else []:
        if fcv.data_path == f'pose.bones["{bn}"].constraints["look_at"].influence':
            gens = [m for m in fcv.modifiers if m.type == "GENERATOR"]
            gen = gens[0] if gens else fcv.modifiers.new("GENERATOR")
            gen.coefficients = (0.0, infl)


# ---- poses -> actions ----
def set_pose(L: np.ndarray, pel: np.ndarray) -> None:
    BHP.set_local_pose(rig, rigd, L, pel)


def key_all(frame: int) -> None:
    BHP.key_pose(rig, rigd, frame)


def make_action(name: str, poses: list[tuple[int, np.ndarray, np.ndarray]], cyclic: bool = False):
    rig.animation_data_create()
    act = bpy.data.actions.new(name)
    act.use_fake_user = True
    rig.animation_data.action = act
    for fr, L, pel in poses:
        set_pose(L, pel)
        key_all(fr)
    if cyclic:
        for fc in _fcurves(act):
            fc.modifiers.new("CYCLES")
    return act


def _fcurves(act):
    if hasattr(act, "fcurves"):
        return list(act.fcurves)
    out = []                                         # layered actions (Blender 4.4+): layers / strips / channelbags
    for layer in act.layers:
        for strip in layer.strips:
            for cb in strip.channelbags:
                out.extend(cb.fcurves)
    return out


def solve(inp: HF.PersonInput) -> tuple[np.ndarray, np.ndarray]:
    G, L = sol.solve(HF.person_params(rigd, inp))
    return L, G[ix["pelvis"], :3, 3]


look_far = np.array([3.0, 0.0, 1.6])
acts = {}
acts["rest"] = make_action("human_rest", [(1, np.repeat(np.eye(3)[None], rigd.B, axis=0), rigd.head_pos("pelvis"))])
acts["idle"] = make_action("human_idle", [(1, *solve(HF.PersonInput(feet=np.zeros(3), yaw=0.0, look_at=look_far)))])
acts["watch_up"] = make_action("human_watch_up", [(1, *solve(HF.PersonInput(feet=np.zeros(3), yaw=0.0,
                                                                              look_at=np.array([1.6, 0.4, 3.2]))))])
acts["push"] = make_action("human_push", [(1, L_push, G_push[ix["pelvis"], :3, 3])])
n_walk = 16
frames_per_cycle = 26                                # 1.15 m per cycle at ~1.3 m/s, 30 fps
walk = []
for i in range(n_walk + 1):
    ph = 2.0 * math.pi * i / n_walk
    L, pel = solve(HF.PersonInput(feet=np.zeros(3), yaw=0.0, look_at=look_far, walk_w=1.0, walk_phase=ph,
                                  walk_dir=np.array([1.0, 0.0])))
    walk.append((1 + round(i * frames_per_cycle / n_walk), L, pel))
acts["walk"] = make_action("human_walk", walk, cyclic=True)
rig.animation_data.action = acts["idle"]

# ---- IK pole angles: chosen so the IK result matches the numpy solver's elbow / knee for the push reference ----
root["push_ik"] = 1.0
root["leg_ik"] = 1.0
rig.animation_data.action = acts["push"]
scene.frame_set(1)


def joint_err(kind: str, s: str) -> float:
    bpy.context.view_layer.update()
    bn = f"forearm_{s}" if kind == "arm" else f"shin_{s}"
    head_arm = np.array(pb[bn].head)                               # armature space (Blender frame)
    world = np.array(rig.matrix_world) @ np.append(head_arm, 1.0)
    want = G_push[ix[bn], :3, 3]                                   # root = identity at this point
    return float(np.linalg.norm(world[:3] - want))


report = {}
for (kind, s), c in ik_cons.items():
    best = (1e9, 0.0)
    for deg in range(-180, 180, 5):
        c.pole_angle = math.radians(deg)
        e = joint_err(kind, s)
        if e < best[0]:
            best = (e, float(deg))
    for deg in np.arange(best[1] - 5.0, best[1] + 5.01, 0.5):
        c.pole_angle = math.radians(float(deg))
        e = joint_err(kind, s)
        if e < best[0]:
            best = (e, float(deg))
    c.pole_angle = math.radians(best[1])
    report[f"{kind}_{s}"] = {"pole_angle_deg": best[1], "joint_error_mm": round(joint_err(kind, s) * 1000.0, 2)}
# hands: the IK'd wrist vs the target
bpy.context.view_layer.update()
for s in ("L", "R"):
    wr = np.array(rig.matrix_world) @ np.append(np.array(pb[f"hand_{s}"].head), 1.0)
    report[f"wrist_{s}_error_mm"] = round(float(np.linalg.norm(wr[:3] - G_push[ix[f"hand_{s}"], :3, 3])) * 1000.0, 2)
print("IK check:", json.dumps(report))
root["push_ik"] = 0.0
root["leg_ik"] = 0.0

# ---- parity: a pose applied with bl_human_pose.Poser equals the numpy solve (bone heads, world) ----
poser = BHP.Poser(NPZ)
feet_t, yaw_t = np.array([0.8, -1.3, 0.0]), 0.7
Mf = HF.figure_matrix(feet_t, yaw_t)
gr = tuple((Mf @ np.append(g, 1.0))[:3] for g in grips_ref)
inp_t = HF.PersonInput(feet=feet_t, yaw=yaw_t, look_at=(Mf @ np.append(TIP, 1.0))[:3], push_w=0.7, grips=gr,
                       pole_dir=Mf[:3, :3] @ wdir, walk_w=0.2, walk_phase=1.0, walk_dir=np.array([0.0, 1.0]))
poser.apply(inp_t)
bpy.context.view_layer.update()
Gt, _ = sol.solve(HF.person_params(rigd, inp_t))
err = max(float(np.linalg.norm((np.array(rig.matrix_world) @ np.append(np.array(pb[n].head), 1.0))[:3]
                               - (Mf @ np.append(Gt[ix[n], :3, 3], 1.0))[:3])) for n in rigd.names)
report["parity_max_bone_head_error_mm"] = round(err * 1000.0, 3)
print("parity (Poser vs numpy): max bone-head error %.3f mm" % (err * 1000.0))
root.location = (0.0, 0.0, 0.0)
root.rotation_euler = (0.0, 0.0, 0.0)
root["look"] = 1.0
rig.animation_data.action = acts["idle"]

# ---- material ----
mat = bpy.data.materials.new("human_satin")
mat.use_nodes = True
bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
lin = [((c / 255.0 + 0.055) / 1.055) ** 2.4 for c in (0xA8, 0xC2, 0xDC)]   # sRGB #a8c2dc -> linear
bsdf.inputs["Base Color"].default_value = (lin[0], lin[1], lin[2], 1.0)
bsdf.inputs["Roughness"].default_value = 0.40
for name, val in (("Coat Weight", 0.12), ("Coat Roughness", 0.35), ("Sheen Weight", 0.15), ("Sheen Roughness", 0.5)):
    if name in bsdf.inputs:
        bsdf.inputs[name].default_value = val
body.data.materials.clear()
body.data.materials.append(mat)
body["source"] = "Blender Studio Human Base Meshes v1.4.1 (CC0) - GEO-body_male_realistic"
body["height_m"] = float(rigd.height)
rig["report"] = json.dumps(report)
for ob in list(bpy.data.objects):
    if ob.users_collection and ob.users_collection[0] is not col and ob.type in ("CAMERA", "LIGHT"):
        bpy.data.objects.remove(ob, do_unlink=True)
try:
    bpy.ops.outliner.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
except RuntimeError as e:                                # purge is a tidy-up only
    print("orphans_purge skipped:", e)
bpy.context.preferences.filepaths.save_version = 0           # no .blend1 backups next to the outputs
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=True)
print("saved", OUT)
