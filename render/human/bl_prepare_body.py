"""Step 1 (headless Blender): take ``GEO-body_male_realistic`` from the Blender Studio Human Base Meshes bundle (CC0),
fold its sculpt into the base cage (Multires "Apply Base"), scale it to the target standing height with the soles on
z = 0 and the origin between the ankles, and write the cage + its face sets + the level-1 surface to an ``.npz`` for the
landmark fit (``fit_joints.py``) and a ``.blend`` with just the prepared body for ``bl_build_rig.py``.

Run: blender --background --factory-startup --python bl_prepare_body.py -- BUNDLE.blend OUT_DIR [HEIGHT_M]

Invariants: the mesh topology of the bundle is kept (no remesh); the face sets of the bundle (one per body part and per
finger phalanx) are carried in the npz; the result does not depend on the scene the script runs in (factory startup).
Blender convention: the body faces -Y, its left side is +X, z up, metres.
"""
import os
import sys

import bpy
import numpy as np

argv = sys.argv[sys.argv.index("--") + 1:]
BUNDLE, OUT = argv[0], argv[1]
HEIGHT = float(argv[2]) if len(argv) > 2 else 1.75
SRC_OBJECT = "GEO-body_male_realistic"
os.makedirs(OUT, exist_ok=True)

for ob in list(bpy.data.objects):
    bpy.data.objects.remove(ob, do_unlink=True)
with bpy.data.libraries.load(BUNDLE, link=False) as (src, dst):
    if SRC_OBJECT not in src.objects:
        raise SystemExit(f"{SRC_OBJECT} not in {BUNDLE}")
    dst.objects = [SRC_OBJECT]
ob = bpy.data.objects[SRC_OBJECT]
bpy.context.scene.collection.objects.link(ob)
ob.name = "human_body"
ob.data.name = "human_body"
ob.location = (0.0, 0.0, 0.0)
ob.rotation_euler = (0.0, 0.0, 0.0)
ob.scale = (1.0, 1.0, 1.0)
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
mr = next(m for m in ob.modifiers if m.type == "MULTIRES")
mr.levels = mr.total_levels
with bpy.context.temp_override(object=ob, active_object=ob, selected_objects=[ob]):
    bpy.ops.object.multires_base_apply(modifier=mr.name)


def mesh_arrays(level: int):
    mr.levels = level
    dg = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(dg)
    m = ev.to_mesh()
    V = np.empty(len(m.vertices) * 3)
    m.vertices.foreach_get("co", V)
    loops = np.empty(len(m.loops), dtype=np.int64)
    m.loops.foreach_get("vertex_index", loops)
    ls = np.empty(len(m.polygons), dtype=np.int64)
    m.polygons.foreach_get("loop_start", ls)
    lt = np.empty(len(m.polygons), dtype=np.int64)
    m.polygons.foreach_get("loop_total", lt)
    ev.to_mesh_clear()
    return V.reshape(-1, 3), loops, ls, lt


# scale / place from the level-1 surface (the cage of a subdivision surface sits slightly outside it)
V1, _, _, _ = mesh_arrays(1)
z0, z1 = float(V1[:, 2].min()), float(V1[:, 2].max())
s = HEIGHT / (z1 - z0)
me = ob.data
co = np.empty(len(me.vertices) * 3)
me.vertices.foreach_get("co", co)
co = co.reshape(-1, 3)
co[:, 2] -= z0
co *= s
co[:, 0] -= 0.5 * (co[:, 0].max() + co[:, 0].min())        # left / right symmetric about x = 0
me.vertices.foreach_set("co", co.ravel())
me.update()
# the sculpt is folded into the cage ("Apply Base"); the finest multires detail (skin pores / wrinkles) is dropped on
# purpose: the target look is a smooth satin anatomy model, and a plain cage + Subdivision Surface keeps the file small
for m in list(ob.modifiers):
    if m.type == "MULTIRES":
        ob.modifiers.remove(m)
loops0 = np.empty(len(me.loops), dtype=np.int64)
me.loops.foreach_get("vertex_index", loops0)
ls0 = np.empty(len(me.polygons), dtype=np.int64)
me.polygons.foreach_get("loop_start", ls0)
lt0 = np.empty(len(me.polygons), dtype=np.int64)
me.polygons.foreach_get("loop_total", lt0)
fs = np.zeros(len(me.polygons), dtype=np.int32)
a = me.attributes.get(".sculpt_face_set")
if a is not None and a.domain == "FACE":
    a.data.foreach_get("value", fs)
# level-1 surface of the scaled cage (Catmull-Clark, as the Subdivision Surface modifier evaluates it)
sub = ob.modifiers.new("Subdivision", "SUBSURF")
sub.levels, sub.render_levels = 1, 1
dg = bpy.context.evaluated_depsgraph_get()
ev = ob.evaluated_get(dg)
m1 = ev.to_mesh()
V1 = np.empty(len(m1.vertices) * 3)
m1.vertices.foreach_get("co", V1)
V1 = V1.reshape(-1, 3)
ev.to_mesh_clear()
ob.modifiers.remove(sub)
# shift so the soles of the level-1 surface touch z = 0 exactly (the cage was scaled about its own minimum), then
# correct the scale so that surface is HEIGHT tall (the plain cage subdivides ~1 mm differently from the multires)
dz = float(V1[:, 2].min())
co[:, 2] -= dz
V1[:, 2] -= dz
s2 = HEIGHT / float(V1[:, 2].max())
co *= s2
V1 *= s2
s *= s2
me.vertices.foreach_set("co", co.ravel())
me.update()
np.savez_compressed(os.path.join(OUT, "body_base.npz"), V=co, loops=loops0, loop_start=ls0, loop_total=lt0,
                    face_set=fs, V1=V1, height_m=HEIGHT, scale=s)
bpy.context.preferences.filepaths.save_version = 0           # no .blend1 backups next to the outputs
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, "body_prepared.blend"), compress=True)
print(f"prepared: scale {s:.5f}, cage verts {co.shape[0]}, level-1 verts {V1.shape[0]}, "
      f"height {V1[:, 2].max() - V1[:, 2].min():.4f} m")
