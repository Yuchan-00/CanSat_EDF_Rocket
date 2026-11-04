# Rigged human (1.75 m) for the CanSat videos

A smooth anatomical "digital human" (pale blue-grey satin, no clothing) that stands, watches the vehicle, walks and
pushes the vehicle with a pole. One rig, two consumers:

| file | for | what |
|---|---|---|
| `human_rigged.blend` | Blender pipeline | mesh + armature + IK targets + actions + material (open or append the collection `human`) |
| `human_lbs.npz` | `tools/human_figure.py` (PyVista videos) | rest mesh (50k tris) + shadow mesh (2.5k tris), top-4 skin weights, bones, presets, grip cavities |
| `bl_human_pose.py` | Blender pipeline | apply the exact per-frame pose of the PyVista video to `human_rig` |

Source mesh: Blender Studio **Human Base Meshes v1.4.1**, object `GEO-body_male_realistic`, **CC0**
(bundle unzipped under `render/assets/human_base_meshes/`, git-ignored). Everything else (rig, weights, poses) is
generated: `python render/human/build_human.py` (venv; ~15 s; `--stills` adds the Blender test renders).

## Scale and frames

* Standing height **1.75 m** (soles on z = 0). Origin: on the floor between the ankles.
* `human_root` (EMPTY): the person's placement. **location = point between the feet on the ground (world)**,
  **rotation Z = facing** - the figure faces its local **+X** (same convention as `tools/sim_video.py`: the figure's
  x axis at `yaw` from world +x). Scale stays 1.
* `human_rig` (ARMATURE) is a child of `human_root`, rotated +90 deg about Z (bones are built in Blender's -Y-forward
  convention). Do not move it; move `human_root`.
* `human_body` (MESH): modifiers **Armature** then **Subdivision Surface (viewport 1, render 2)**; the cage has
  10.6k quads. Material slot 0 = **`human_satin`** (Principled: base colour = sRGB #a8c2dc, roughness 0.40, a little
  coat and sheen). Replace or tweak that one material to restyle the person.

## Objects (collection `human`)

| name | type | role |
|---|---|---|
| `human_root` | EMPTY | placement; custom props `push_ik` (0..1), `look` (0..1), `leg_ik` (0..1) drive the constraint influences (plain SUM drivers - no Python expressions, works without "auto-run scripts") |
| `human_rig` | ARMATURE | 55 bones (below) |
| `human_body` | MESH | the body |
| `pole_grip` | EMPTY | child of `human_root` at the pole's held end, **(0.45 m forward, 0, 1.25 m up)** = `sim_video.FIGURE_HANDS`; Track To `pole_tip` (local +Y along the pole, Z up) |
| `pole_tip` | EMPTY | **world**: the push contact point (pad centre) |
| `ik_hand_R`, `ik_hand_L` | EMPTY | children of `pole_grip`: wrist position + hand rotation of the rear (right, 0.12 m behind the held end) and front (left, 0.10 m ahead) grips - the pole passes through each fist's grip cavity |
| `pole_elbow_R`, `pole_elbow_L` | EMPTY | elbow pole targets (children of `human_root`) |
| `ik_foot_R`, `ik_foot_L`, `pole_knee_R`, `pole_knee_L` | EMPTY | push-stance leg IK targets (used when `leg_ik` = 1) |
| `look_at` | EMPTY | **world**: what the head looks at (neck 40 % + head Damped Track, face = head bone +Z) |

Bones: `root`, `pelvis`, `spine_01..03`, `neck`, `head`, and per side `_L`/`_R`: `clavicle`, `upperarm`, `forearm`,
`forearm_twist` (takes half the wrist twist, skin only), `hand`, `thumb_01..03`, `index_01..03`, `middle_01..03`,
`ring_01..03`, `pinky_01..03`, `thigh`, `shin`, `foot`, `toe`. Roll convention: a positive rotation about a bone's
local X is flexion (spine forward, elbows, knees, finger curl).

Actions (fake user): `human_rest`, `human_idle` (relaxed, weight on the right leg), `human_watch_up` (looking up at
a hovering vehicle), `human_push` (push stance + grip; with `push_ik` = 1 the arms follow the IK targets),
`human_walk` (16 keys per 1.15 m gait cycle, 26 frames at 30 fps, cyclic). `human_idle` is assigned.

## Drive the push from a contact point and a pole direction

1. `human_root.location` = where the person stands (feet, world); `human_root.rotation_euler.z` = facing (toward the
   push direction). `sim_video.figure_track` computes both: feet = contact_xy - u (1.10 + 0.45) m, facing u = the
   horizontal push direction.
2. Assign `human_push` to `human_rig` and set `human_root["push_ik"] = 1` (arms + hands follow `ik_hand_*`).
3. Put `pole_tip` on the contact point (the pad centre). The pole runs from `pole_grip` (held end) to `pole_tip`;
   `pole_grip` aims at it, the hand targets ride along, the IK puts both fists on the pole. The pole direction is
   therefore `pole_tip - pole_grip`; to use a given direction `d` instead, place `pole_tip` at
   `pole_grip + d * L`. Draw the pole yourself from `pole_grip - 0.25 m * d` to the tip (24 mm thick in the videos;
   a padded tip ball r = 45 mm).
4. `look_at` on the vehicle (its CG), `human_root["look"] = 1`.
5. Optional `leg_ik` = 1 plants the feet on `ik_foot_*` (the push stance as built).

IK pole angles were fitted so Blender's IK equals the numpy solver for the reference push (elbow / wrist error 0.0 mm,
knee <= 0.2 mm; stored in `human_rig["report"]`).

## Exact parity with the PyVista videos (recommended for intercut shots)

The PyVista videos pose the person with `tools/human_figure.py` (pure numpy; runs inside Blender's Python). Feed the
same inputs and key the result:

```python
import sys; sys.path[:0] = [r"<project>/render/human", r"<project>/tools"]
import bl_human_pose as BHP, human_figure as HF
poser = BHP.Poser()                       # turns push_ik / look / leg_ik off: the numpy solve already contains them
for frame, inp in frames:                 # inp = HF.PersonInput(feet, yaw, t, look_at, push_w, grips, pole_dir,
    poser.apply(inp, frame=frame)         #                     walk_w, walk_phase, walk_dir) - world frame
```

`tools/sim_video.py` builds those inputs in `Scene._pose_person` (grips = held end `fig.hands[k]` + pole direction x
`POLE_GRIP_M` = (-0.12 m rear/right, +0.10 m front/left); push weight from `person_push_weight` = 1 during a push,
smoothstep over 0.3 s before / after it; walking from `person_timeline`). Measured parity: 0.02 mm (bone heads).

## How it was built (deterministic)

1. `bl_prepare_body.py` - bundle body, Multires "Apply Base" (sculpt folded into the cage), scaled to 1.75 m.
2. `fit_joints.py` - joint centres from the mesh: the bundle's face sets split every body part and finger phalanx, so
   the boundary ring between two face sets gives elbow / wrist / knee / ankle / finger joints; shoulder, hip, spine and
   head pivot from cross-sections and anthropometric ratios (`build/joints.json`).
3. `bl_build_rig.py` - armature, bone-heat weights, clean-up (fingers only from their own finger, no cross-side
   weights, arm weights off the sternum, thighs below the belt, 8 Laplacian smoothing passes, forearm twist split,
   top-4), LBS export.
4. `make_lbs_npz.py` - figure frame, normals, finger presets; the grip is closed on the mesh (pole axis fixed in the
   palm, every finger curls until its skin touches a 12 mm-radius pole), whole-body presets.
5. `bl_finalize_rig.py` - this file's objects, constraints, drivers, actions, material, IK pole-angle fit, parity check.
6. `bl_render_tests.py` - Cycles CPU stills (headless Eevee / Workbench give only one good still per session here).

Limits the runtime relies on (also in the npz meta): head yaw +-75 deg, pitch -50..+40 deg (neck takes 40 %),
spine flex -15..35 deg, side +-15 deg, twist +-35 deg, pelvis tilt +-12 deg; legs keep 0.992 of their length (the
pelvis lowers instead of locking the knees); out-of-reach grips are reported (`Solver.last[side].reached`), never faked.

## Note for the Blender pipeline (architect, 2026-10-21)

- Measured by the human-figure agent on this PC: headless Blender 5.2.2 (`--background`) produced only the FIRST Eevee or
  Workbench render of a session; later renders in the same session came out empty. Cycles (CPU) rendered every frame.
  Either render each Eevee frame / chunk in its own Blender process, or use Cycles, and verify every output frame is
  non-empty.
- The grip is fitted to a 24 mm pole (`GRIP_RADIUS_M` in `make_lbs_npz.py`); a thicker padded pole needs that value
  changed and `build_human.py` re-run (about 15 s).
