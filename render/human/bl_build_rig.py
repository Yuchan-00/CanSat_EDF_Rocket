"""Step 3 (headless Blender): build the armature from ``build/joints.json`` (``fit_joints.py``), bind the prepared body
(``build/body_prepared.blend``) with automatic (bone-heat) weights, clean the weights, and export the skinning data the
PyVista runtime needs (``build/lbs_raw.npz``: the render mesh = the cage subdivided once and decimated to about
``--tris`` triangles, a ~2.5k-triangle shadow mesh, both with their interpolated vertex-group weights, and the bones).

Run: blender --background --factory-startup --python bl_build_rig.py -- BUILD_DIR [--tris 50000] [--shadow-tris 2500]

Weight clean-up (each step is deterministic numpy on the bone-heat result):
  * ``forearm_twist_*`` is not in the heat solve; afterwards the forearm's weight is split along the forearm
    (0 at 25 % of the way from the elbow, 85 % of the weight at the wrist) so a wrist twist is spread over the forearm
    instead of collapsing at the wrist (candy-wrapper);
  * vertices of a face set get no weight from bones of an unrelated chain (fingers only from their own finger, the
    hand or the forearm; the left side never from right bones and vice versa) - bone heat leaks between touching parts;
  * arm bones fade out toward the sternum (7-12 cm from the midline), clavicles at the midline, thighs above the belt
    line (hip joint + 0-10 cm): bone heat put small arm / leg weights there, which made ridges on the chest and a
    crease at the waist when the arms or legs moved;
  * 8 passes of Laplacian smoothing (0.5 x the neighbour mean over the cage edges) on everything but the hands, feet
    and head, then the cross-side rule again - softens the pelvis / spine and knee / elbow transitions;
  * at most 4 bones per vertex, normalised (what the LBS runtime and game engines expect).
Output objects: armature ``human_rig`` (Blender convention: faces -Y, left = +X), mesh ``human_body`` (Armature
modifier, then Subdivision Surface viewport 1 / render 2), saved as ``build/rig_weighted.blend``.
"""
import json
import math
import os
import sys

import bpy
import numpy as np
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
BUILD = argv[0]


def opt(name, default):
    return type(default)(argv[argv.index(name) + 1]) if name in argv else default


TRIS = opt("--tris", 50000)
SHADOW_TRIS = opt("--shadow-tris", 2500)
J = json.load(open(os.path.join(BUILD, "joints.json"), encoding="utf-8"))
shift = Vector(J["info"]["origin_shift_m"])

bpy.ops.wm.open_mainfile(filepath=os.path.join(BUILD, "body_prepared.blend"))
for ob in list(bpy.data.objects):
    if ob.name != "human_body":
        bpy.data.objects.remove(ob, do_unlink=True)
body = bpy.data.objects["human_body"]
me = body.data
co = np.empty(len(me.vertices) * 3)
me.vertices.foreach_get("co", co)
co = co.reshape(-1, 3) + np.array(shift)
me.vertices.foreach_set("co", co.ravel())
me.update()
for p in me.polygons:
    p.use_smooth = True

# ---- armature ----
arm = bpy.data.armatures.new("human_rig")
rig = bpy.data.objects.new("human_rig", arm)
bpy.context.scene.collection.objects.link(rig)
bpy.context.view_layer.objects.active = rig
rig.select_set(True)
bpy.ops.object.mode_set(mode="EDIT")
for b in J["bones"]:
    eb = arm.edit_bones.new(b["name"])
    eb.head, eb.tail = Vector(b["head"]), Vector(b["tail"])
    eb.align_roll(Vector(b["z"]))
    eb.use_deform = bool(b["deform"]) and not b["name"].startswith("forearm_twist")
for b in J["bones"]:
    if b["parent"]:
        arm.edit_bones[b["name"]].parent = arm.edit_bones[b["parent"]]
        arm.edit_bones[b["name"]].use_connect = False
bpy.ops.object.mode_set(mode="OBJECT")
arm.display_type = "STICK"

# ---- automatic weights ----
for ob in bpy.context.view_layer.objects:
    ob.select_set(False)
body.select_set(True)
rig.select_set(True)
bpy.context.view_layer.objects.active = rig
with bpy.context.temp_override(active_object=rig, object=rig, selected_objects=[body, rig],
                               selected_editable_objects=[body, rig]):
    r = bpy.ops.object.parent_set(type="ARMATURE_AUTO")
print("parent_set", r)
for b in arm.bones:
    b.use_deform = b.name != "root"

# ---- weight clean-up in numpy ----
names = [b["name"] for b in J["bones"]]
bidx = {n: i for i, n in enumerate(names)}
N, B = len(me.vertices), len(names)
W = np.zeros((N, B))
gi2b = {g.index: bidx.get(g.name, -1) for g in body.vertex_groups}
for v in me.vertices:
    for g in v.groups:
        k = gi2b.get(g.group, -1)
        if k >= 0:
            W[v.index, k] = g.weight
unweighted = int(np.sum(W.sum(axis=1) < 1e-6))
print("bone-heat: vertices without weight", unweighted)

fs = np.zeros(len(me.polygons), dtype=np.int32)
a = me.attributes.get(".sculpt_face_set")
if a is not None:
    a.data.foreach_get("value", fs)
vfs = [set() for _ in range(N)]
for p, f in zip(me.polygons, fs):
    for vi in p.vertices:
        vfs[vi].add(int(f))
FING = {"thumb": (80, 81, 82, 83), "index": (76, 77, 78, 79), "middle": (72, 73, 74, 75), "ring": (68, 69, 70, 71),
        "pinky": (64, 65, 66, 67)}
FING_R = {"thumb": (84, 85, 86, 87), "index": (88, 89, 90, 91), "middle": (92, 93, 94, 95), "ring": (96, 97, 98, 99),
          "pinky": (100, 101, 102, 103)}
X = co[:, 0]
for side, fmap in (("L", FING), ("R", FING_R)):
    for f, sets in fmap.items():
        S = set(sets)
        allowed = {bidx[f"{f}_0{i}_{side}"] for i in (1, 2, 3)} | {bidx[f"hand_{side}"]}
        for vi in range(N):
            if vfs[vi] and vfs[vi] <= S:
                keep = np.zeros(B, dtype=bool)
                keep[list(allowed)] = True
                W[vi, ~keep] = 0.0
# no cross-side weights for limbs (bone heat leaks between the thighs at the crotch, arm to torso is fine)
for side, sgn in (("L", 1.0), ("R", -1.0)):
    other = "R" if side == "L" else "L"
    cols = [i for n, i in bidx.items() if n.endswith("_" + other)]
    m = X * sgn > 0.015
    W[np.ix_(m, cols)] = 0.0
# arm bones off the chest's middle (bone heat gives the upper arm a little of the sternum: ridges when the arms move),
# clavicles fade out at the midline, thighs do not reach above the belt line (the belly must not follow the legs)
Z = co[:, 2]
heads = {b["name"]: np.array(b["head"]) for b in J["bones"]}
tails = {b["name"]: np.array(b["tail"]) for b in J["bones"]}


def ramp(x, e0, e1):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


for side, sgn in (("L", 1.0), ("R", -1.0)):
    lat = X * sgn
    arm_cols = [i for n, i in bidx.items() if n.endswith("_" + side) and not n.startswith(("clavicle", "thigh", "shin",
                                                                                            "foot", "toe"))]
    W[:, arm_cols] *= ramp(lat, 0.07, 0.12)[:, None]
    W[:, bidx[f"clavicle_{side}"]] *= ramp(lat, 0.005, 0.045)
    hz = float(heads[f"thigh_{side}"][2])
    W[:, bidx[f"thigh_{side}"]] *= 1.0 - ramp(Z, hz, hz + 0.10)
# Laplacian smoothing of the weights over the cage (bone heat leaves sharp borders between pelvis / spine and at the
# knee / elbow creases): 8 passes of 0.5 x the neighbour mean, not on hands, feet or the head (they keep exact
# per-part weights), renormalised
edges = set()
for p in me.polygons:
    vs = list(p.vertices)
    for i in range(len(vs)):
        a_, b_ = vs[i], vs[(i + 1) % len(vs)]
        edges.add((min(a_, b_), max(a_, b_)))
E = np.array(sorted(edges), dtype=np.int64)
deg = np.bincount(E.ravel(), minlength=N).astype(float)
fixed_cols = [i for n, i in bidx.items() if n.startswith(("hand", "thumb", "index", "middle", "ring", "pinky", "foot",
                                                          "toe", "head"))]
free = W[:, fixed_cols].sum(axis=1) < 0.02
for _ in range(8):
    acc = np.zeros_like(W)
    np.add.at(acc, E[:, 0], W[E[:, 1]])
    np.add.at(acc, E[:, 1], W[E[:, 0]])
    mean = acc / np.maximum(deg, 1.0)[:, None]
    W[free] = 0.5 * W[free] + 0.5 * mean[free]
    W[:, fixed_cols] = np.where(free[:, None], 0.0, W[:, fixed_cols])
    W /= np.maximum(W.sum(axis=1, keepdims=True), 1e-12)
for side, sgn in (("L", 1.0), ("R", -1.0)):                 # smoothing must not carry weights across the midline
    other = "R" if side == "L" else "L"
    cols = [i for n, i in bidx.items() if n.endswith("_" + other)]
    W[np.ix_(X * sgn > 0.015, cols)] = 0.0
W /= np.maximum(W.sum(axis=1, keepdims=True), 1e-12)
# forearm twist split
for side in ("L", "R"):
    e, w_ = heads[f"forearm_{side}"], tails[f"forearm_{side}"]
    d = w_ - e
    t = np.clip(((co - e) @ d) / float(d @ d), 0.0, 1.0)
    s = np.clip((t - 0.25) / 0.75, 0.0, 1.0)
    s = 0.85 * s * s * (3.0 - 2.0 * s)
    fa, tw = bidx[f"forearm_{side}"], bidx[f"forearm_twist_{side}"]
    W[:, tw] = W[:, fa] * s
    W[:, fa] = W[:, fa] * (1.0 - s)
# top-4 + normalise
order = np.argsort(-W, axis=1)
W4 = np.take_along_axis(W, order[:, :4], axis=1)
W[:] = 0.0
np.put_along_axis(W, order[:, :4], W4, axis=1)
sw = W.sum(axis=1, keepdims=True)
bad = sw[:, 0] < 1e-9
W[~bad] /= sw[~bad]
if np.any(bad):                                    # nearest weighted neighbour (bone heat misses are rare)
    good = np.flatnonzero(~bad)
    for vi in np.flatnonzero(bad):
        j = good[np.argmin(np.sum((co[good] - co[vi]) ** 2, axis=1))]
        W[vi] = W[j]
print("after clean-up: max bones/vertex", int((W > 0).sum(axis=1).max()), "unweighted", int(np.sum(W.sum(1) < 1e-6)))
# write back
body.vertex_groups.clear()
for n in names:
    if n == "root":
        continue
    g = body.vertex_groups.new(name=n)
    col = W[:, bidx[n]]
    nz = np.flatnonzero(col > 0)
    for vi in nz:
        g.add([int(vi)], float(col[vi]), "REPLACE")
amod = next(m for m in body.modifiers if m.type == "ARMATURE")
amod.name = "Armature"
amod.object = rig
amod.use_deform_preserve_volume = False
sub = body.modifiers.new("Subdivision", "SUBSURF")
sub.levels, sub.render_levels = 1, 2


# ---- export: render mesh (subdivided once, decimated) + shadow mesh, with weights ----
def export_mesh(subdiv: int, target_tris: int):
    tmp = body.copy()
    tmp.data = body.data.copy()
    bpy.context.scene.collection.objects.link(tmp)
    for m in list(tmp.modifiers):
        tmp.modifiers.remove(m)
    base_tris = sum(len(p.vertices) - 2 for p in tmp.data.polygons) * (4 ** subdiv)
    if subdiv:
        s_ = tmp.modifiers.new("sub", "SUBSURF")
        s_.levels = s_.render_levels = subdiv
    if target_tris < base_tris:
        dm = tmp.modifiers.new("dec", "DECIMATE")
        dm.decimate_type = "COLLAPSE"
        dm.ratio = target_tris / base_tris
        dm.use_symmetry = True
        dm.symmetry_axis = "X"
    tri = tmp.modifiers.new("tri", "TRIANGULATE")
    dg = bpy.context.evaluated_depsgraph_get()
    ev = tmp.evaluated_get(dg)
    m = ev.to_mesh()
    Vv = np.empty(len(m.vertices) * 3)
    m.vertices.foreach_get("co", Vv)
    Vv = Vv.reshape(-1, 3)
    F = np.empty(len(m.polygons) * 3, dtype=np.int64)
    m.polygons.foreach_get("vertices", F)
    F = F.reshape(-1, 3)
    gi = {g.index: bidx[g.name] for g in tmp.vertex_groups}
    Wm = np.zeros((len(m.vertices), B))
    for v in m.vertices:
        for g in v.groups:
            if g.group in gi:
                Wm[v.index, gi[g.group]] = g.weight
    ev.to_mesh_clear()
    bpy.data.objects.remove(tmp, do_unlink=True)
    return Vv, F, Wm


Vr, Fr, Wr = export_mesh(1, TRIS)
Vs, Fs, Ws = export_mesh(0, SHADOW_TRIS)
print("render mesh", Vr.shape[0], "verts", Fr.shape[0], "tris; shadow mesh", Vs.shape[0], "verts", Fs.shape[0], "tris")
rest = np.array([np.array(arm.bones[n].matrix_local) for n in names])
parents = np.array([names.index(b["parent"]) if b["parent"] else -1 for b in J["bones"]], dtype=np.int32)
np.savez_compressed(os.path.join(BUILD, "lbs_raw.npz"), V=Vr, F=Fr, W=Wr, Vs=Vs, Fs=Fs, Ws=Ws, rest=rest,
                    parents=parents, names=np.array(names), lengths=np.array([arm.bones[n].length for n in names]),
                    cage_V=co, cage_W=W)
bpy.context.preferences.filepaths.save_version = 0           # no .blend1 backups next to the outputs
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(BUILD, "rig_weighted.blend"), compress=True)
print("saved", os.path.join(BUILD, "rig_weighted.blend"))
