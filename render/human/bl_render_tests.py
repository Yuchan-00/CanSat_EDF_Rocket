"""Step 6 (optional, headless Blender): test stills of ``human_rigged.blend`` for checking the deformation by eye -
whole body (front / side / three-quarter / back) in the idle, push (IK on the pole), watch-up and walk poses, and
close-ups of the shoulders, elbows, hips, knees and the hands on the pole.  Render
subdivision 2, Cycles CPU (headless Eevee / Workbench give one good still per session on the dev PC).  Writes PNGs to OUT_DIR (contact sheets ``sheet_*.png``).

Run: blender --background --factory-startup --python bl_render_tests.py -- HUMAN_RIGGED_BLEND OUT_DIR
"""
import math
import os
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
SRC, OUT = argv[0], argv[1]
os.makedirs(OUT, exist_ok=True)
bpy.ops.wm.open_mainfile(filepath=SRC)
bpy.context.preferences.system.use_gpu_subdivision = False
sc = bpy.context.scene
# Cycles on the CPU: in a headless session on the dev PC only the FIRST Workbench / Eevee render has pixels (later
# ones come out empty), Cycles CPU renders every still
sc.render.engine = "CYCLES"
sc.cycles.device = "CPU"
sc.cycles.samples = 24
sc.cycles.use_denoising = True
sc.render.resolution_x, sc.render.resolution_y = 560, 700
sc.render.film_transparent = False
world = bpy.data.worlds.new("test_world")
world.use_nodes = True
world.node_tree.nodes["Background"].inputs[0].default_value = (0.55, 0.6, 0.66, 1.0)
world.node_tree.nodes["Background"].inputs[1].default_value = 0.8
sc.world = world
sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
sun.data.energy = 3.5
sun.rotation_euler = (math.radians(50), 0.0, math.radians(35))
sc.collection.objects.link(sun)
bpy.ops.mesh.primitive_plane_add(size=6.0)
floor = bpy.context.active_object
fm = bpy.data.materials.new("floor")
fm.diffuse_color = (0.35, 0.33, 0.3, 1.0)
floor.data.materials.append(fm)
root = bpy.data.objects["human_root"]
rig = bpy.data.objects["human_rig"]
pg = bpy.data.objects["pole_grip"]
tip = bpy.data.objects["pole_tip"]
# the push pole: 24 mm thick, from 0.25 m behind the held end to the tip, as in the PyVista videos
bpy.ops.mesh.primitive_cylinder_add(radius=0.012, depth=1.0, vertices=24)
pole = bpy.context.active_object
pole.data.transform(__import__("mathutils").Matrix.Translation((0, 0, 0.5)))
pm = bpy.data.materials.new("pole")
pm.diffuse_color = (0.7, 0.45, 0.2, 1.0)
pole.data.materials.append(pm)
cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
sc.collection.objects.link(cam)
sc.camera = cam


def place_pole(show: bool) -> None:
    pole.hide_render = not show
    if not show:
        return
    bpy.context.view_layer.update()
    a = pg.matrix_world.translation
    b = tip.matrix_world.translation
    d = (b - a).normalized()
    start = a - d * 0.25
    L = (b - start).length
    pole.location = start
    pole.rotation_mode = "QUATERNION"
    pole.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(d)
    pole.scale = (1.0, 1.0, L)


def shoot(name: str, target, az: float, el: float, dist: float, lens: float = 50.0) -> str:
    t = Vector(target)
    d = Vector((math.cos(math.radians(el)) * math.cos(math.radians(az)),
                math.cos(math.radians(el)) * math.sin(math.radians(az)), math.sin(math.radians(el))))
    cam.location = t + d * dist
    cam.rotation_mode = "QUATERNION"
    cam.rotation_quaternion = (-d).to_track_quat("-Z", "Y")
    cam.data.lens = lens
    path = os.path.join(OUT, name + ".png")
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)
    return path


def pose(action: str, push: float, frame: int = 1) -> None:
    rig.animation_data.action = bpy.data.actions[action]
    root["push_ik"] = push
    sc.frame_set(frame)
    bpy.context.view_layer.update()


def bone_world(name: str):
    return rig.matrix_world @ rig.pose.bones[name].head


shots = []
pose("human_idle", 0.0)
place_pole(False)
for az in (0, 90, 35, 180):
    shots.append(shoot(f"idle_az{az}", (0.05, 0, 0.9), az, 5, 4.6))
pose("human_push", 1.0)
place_pole(True)
for az in (0, 90, 35, 180, -60):
    shots.append(shoot(f"push_az{az}", (0.15, 0, 1.0), az, 8, 5.0))
for bn, az in (("upperarm_R", -70), ("upperarm_L", 70), ("forearm_R", -100), ("forearm_L", 100),
               ("hand_R", -40), ("hand_L", 40), ("thigh_L", 60), ("shin_L", 80), ("shin_R", -110)):
    shots.append(shoot(f"push_close_{bn}", tuple(bone_world(bn)), az, 15, 0.9 if "hand" in bn else 1.3))
pose("human_watch_up", 0.0)
place_pole(False)
shots.append(shoot("watch_up_az30", (0.05, 0, 0.9), 30, 5, 4.6))
for f in (1, 7, 14, 20):
    pose("human_walk", 0.0, f)
    shots.append(shoot(f"walk_f{f:02d}", (0.05, 0, 0.9), 90, 5, 4.6))
print("rendered", len(shots))
