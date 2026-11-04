"""Blender entry point: build the scene from a frame plan (``render.plan``), key it, render frames resumably.

  blender --background --factory-startup --python render/blender/main.py -- --plan PLAN.npz --out DIR
          [--engine EEVEE|CYCLES] [--quality draft|final] [--frames a:b] [--stills f1,f2] [--samples N]
          [--res WxH] [--threads N] [--save-blend PATH] [--no-grass] [--hdri ID] [--timing JSON]

Engines (measured 2026-10-21, i7-11700 + Intel UHD 750, Blender 5.2.2 headless): Cycles CPU is the production
engine (adaptive sampling + OpenImageDenoise, motion blur [k, k + 0.5], DOF, AgX Medium High Contrast).  EEVEE is
kept selectable but is unreliable here: after the first render of a session every further frame comes out black
(a smaller shadow pool, a 2k texture limit, motion blur off and ray tracing off did not help; Blender refuses the
Vulkan backend for this driver) - the driver deletes empty frames.

Frames are Blender frames 1..N (frame k + 1 = plan row k).  ``DIR/frame_{k:05d}.png`` that already exist are
skipped (resumable).  Each rendered frame's wall time goes to ``--timing`` (JSON, merged across runs).

Invariant: the scene is a deterministic function of the plan + the options (no random state, no clock); the only
inputs from disk are the plan, params.json, the CAD STLs, the Poly Haven assets and render/human/** (read-only).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import bpy  # noqa: E402
import numpy as np  # noqa: E402

from render.blender import bl_util as U  # noqa: E402
from render.blender import env as E  # noqa: E402
from render.blender import fx as FX  # noqa: E402
from render.blender import grass as GR  # noqa: E402
from render.blender import person as PE  # noqa: E402
from render.blender import props as PR  # noqa: E402
from render.blender import vehicle as V  # noqa: E402

ASSETS = os.path.join(ROOT, "render", "assets", "polyhaven")
CACHE = os.path.join(ROOT, "render", "assets", "cache")
SENSOR_W = 36.0
BORE_W = 0.45         # W: light-linked disc in the duct exit, aimed up the bore (tail cone, stators, rotor blur disc):
#                       a softly lit grey cone behind the vanes silhouettes them; 6 W burned the cone into a white
#                       crescent (round 1b), 1.6 W still clipped it into a 'lamp in the exhaust' (round 2 review)


def parse(argv: list[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--engine", default="CYCLES", choices=("EEVEE", "CYCLES"))
    ap.add_argument("--quality", default="draft", choices=("draft", "final"))
    ap.add_argument("--frames", default="")
    ap.add_argument("--stills", default="")
    ap.add_argument("--frames-file", default="", help="JSON list of Blender frames to render")
    ap.add_argument("--samples", type=int, default=0)
    ap.add_argument("--res", default="1920x1080")
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--save-blend", default="")
    ap.add_argument("--no-grass", action="store_true")
    ap.add_argument("--no-dust-volume", action="store_true", help="skip the volumetric dust skirt (outdoor)")
    ap.add_argument("--hdri", default="alps_field")
    ap.add_argument("--timing", default="")
    ap.add_argument("--build-only", action="store_true")
    return ap.parse_args(argv)


def load_plan(path: str) -> tuple[dict[str, np.ndarray], dict]:
    with np.load(path, allow_pickle=False) as d:
        arr = {k: d[k] for k in d.files if k != "meta"}
        meta = json.loads(str(d["meta"]))
    return arr, meta


def reset_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)


# --------------------------------------------------------------------------- #
# render settings
# --------------------------------------------------------------------------- #
def setup_render(engine: str, quality: str, samples: int, res: str, threads: int) -> None:
    scn = bpy.context.scene
    w, h = (int(x) for x in res.lower().split("x"))
    r = scn.render
    r.resolution_x, r.resolution_y, r.resolution_percentage = w, h, 100
    r.fps = 30
    r.image_settings.file_format = "PNG"
    r.image_settings.color_mode = "RGB"
    r.image_settings.color_depth = "8"
    r.image_settings.compression = 15
    r.use_motion_blur = True
    r.motion_blur_shutter = 0.5
    if hasattr(r, "motion_blur_position"):
        r.motion_blur_position = "START"           # [k, k + 0.5]: a cut never smears into the next shot
    r.film_transparent = False
    vs = scn.view_settings
    vs.view_transform = "AgX"
    for look in ("AgX - Medium High Contrast", "Medium High Contrast"):
        try:
            vs.look = look
            break
        except TypeError:
            continue
    vs.exposure = 0.0
    if engine == "CYCLES":
        r.engine = "CYCLES"
        c = scn.cycles
        c.device = "CPU"
        c.samples = samples or (96 if quality == "final" else 20)
        c.use_adaptive_sampling = True
        c.adaptive_threshold = 0.02 if quality == "final" else 0.06
        c.adaptive_min_samples = 0
        c.use_denoising = True
        try:
            c.denoiser = "OPENIMAGEDENOISE"
        except TypeError:
            pass
        if hasattr(c, "denoising_input_passes"):
            c.denoising_input_passes = "RGB_ALBEDO_NORMAL"
        if hasattr(c, "denoising_prefilter"):
            c.denoising_prefilter = "ACCURATE" if quality == "final" else "FAST"
        c.max_bounces = 6
        c.diffuse_bounces = 2
        c.glossy_bounces = 2
        c.transmission_bounces = 4
        c.transparent_max_bounces = 8
        c.volume_bounces = 0
        c.caustics_reflective = False
        c.caustics_refractive = False
        # fireflies: a low indirect clamp + glossy filter keep 16-32 spp frames free of sparkle on the parquet /
        # the grit (art review round 1)
        c.sample_clamp_indirect = 3.0
        if hasattr(c, "blur_glossy"):
            c.blur_glossy = 1.0
        c.use_animated_seed = True
        c.seed = 0
        r.use_persistent_data = True
        r.threads_mode = "FIXED" if threads > 0 else "AUTO"
        if threads > 0:
            r.threads = threads
        if hasattr(c, "texture_limit_render"):
            c.texture_limit_render = "OFF"
        c.film_exposure = 1.0
        c.pixel_filter_type = "BLACKMAN_HARRIS"
        c.filter_width = 1.5
    else:
        r.engine = "BLENDER_EEVEE" if "BLENDER_EEVEE" in [e.identifier for e in type(r).bl_rna.properties["engine"].enum_items] \
            else "BLENDER_EEVEE_NEXT"
        e = scn.eevee
        e.taa_render_samples = samples or (64 if quality == "final" else 16)
        # Intel UHD 750 (1 GB shared): the default 512 MB shadow pool + the 4k HDRI + 2k PBR sets exhaust GPU memory
        # and EEVEE then renders black (measured 2026-10-21); 256 MB works
        # (shader_compilation_method = SUBPROCESS rendered black frames here: keep the default THREAD)
        bpy.context.preferences.system.gl_texture_limit = os.environ.get("EEVEE_TEXTURE_LIMIT", "CLAMP_2048")
        for k, v in (("shadow_pool_size", os.environ.get("EEVEE_SHADOW_POOL", "256")), ("gi_irradiance_pool_size", "16"),
                     ("use_raytracing", True), ("use_shadows", True), ("shadow_ray_count", 2), ("shadow_step_count", 8),
                     ("use_volumetric_shadows", False), ("gi_diffuse_bounces", 2), ("fast_gi_resolution", "2"),
                     ("use_fast_gi", True), ("motion_blur_steps", 1)):
            if hasattr(e, k):
                try:
                    setattr(e, k, v)
                except (TypeError, AttributeError):
                    pass
        if hasattr(e, "ray_tracing_options"):
            o = e.ray_tracing_options
            for k, v in (("resolution_scale", "2"), ("use_denoise", True), ("trace_max_roughness", 0.5)):
                if hasattr(o, k):
                    try:
                        setattr(o, k, v)
                    except (TypeError, AttributeError):
                        pass


# --------------------------------------------------------------------------- #
# camera
# --------------------------------------------------------------------------- #
def build_camera() -> tuple[object, object]:
    cd = bpy.data.cameras.new("cam")
    cd.sensor_fit = "HORIZONTAL"
    cd.sensor_width = SENSOR_W
    cd.clip_start = 0.02
    cd.clip_end = 6000.0
    cd.dof.use_dof = True
    cd.dof.aperture_blades = 7
    cd.dof.aperture_rotation = math.radians(12.0)
    co = bpy.data.objects.new("cam", cd)
    U.link(co)
    co.rotation_mode = "QUATERNION"
    bpy.context.scene.camera = co
    return co, cd


# --------------------------------------------------------------------------- #
# build + key
# --------------------------------------------------------------------------- #
def build_and_key(P: dict[str, np.ndarray], meta: dict, args: argparse.Namespace, warnings: list[str]) -> dict:
    scn = bpy.context.scene
    n = int(meta["frames"])
    frames = np.arange(1, n + 1, dtype=float)
    cut = set(int(k) + 1 for k in np.flatnonzero(P["cut_after"]))
    gz = float(meta["ground_z"])
    kind = meta.get("scene", "outdoor")
    scn.view_settings.exposure = float(meta.get("exposure", 0.0))   # indoor: the hall HDRI is bright (windows)
    wb = meta.get("white_balance_k")
    if wb and hasattr(scn.view_settings, "use_white_balance"):
        # neutral grade (art review round 2: the warm hall graded the black PLA brown, the floor R/B 3.6)
        scn.view_settings.use_white_balance = True
        scn.view_settings.white_balance_temperature = float(os.environ.get("WB_K", wb))
        scn.view_settings.white_balance_tint = float(os.environ.get("WB_TINT", "10.0"))
    coll = U.collection("main")
    info: dict = {"scene": kind}
    if kind == "outdoor":
        env = E.build_outdoor(ASSETS, CACHE, gz, float(meta.get("sun_az_deg", 135.0)), coll, warnings, hdri_id=args.hdri)
        soil = env["soil_z"]
    elif kind == "indoor":
        env = E.build_indoor(ASSETS, gz, coll, warnings, hall_fit=meta.get("hall"))
        soil = env["soil_z"]
        if meta.get("key_light"):
            env["key_spot"] = E.stage_key_spot(coll, meta["key_light"])
    else:
        env = E.build_studio(ASSETS, gz - 0.03, coll, warnings)
        soil = gz
    info["env"] = {k: v for k, v in env.items() if isinstance(v, (int, float, str))}
    veh = V.build_vehicle(ROOT, coll, warnings, args.quality)
    pad = V.build_pad(ROOT, coll, warnings)
    root = veh["root"]
    key = U.key_array
    if kind == "studio" and "link" in env:
        vobjs = [o for o in root.children_recursive if o.type == "MESH"] + [pad]
        ok1 = E.link_lights(env["link"]["vehicle_only"], vobjs, "studio_strip_receivers")
        ok2 = E.link_lights(env["link"]["cyc_only"], [env["cyc"]], "studio_glow_receivers")
        info["studio_light_linking"] = bool(ok1 and ok2)
        if "studio_rim" in P:
            # the reveal: rims first, then key / fill / top (render.plan.turntable_plan)
            L = env["lights"]
            for lo, arr in ((L[1], P["studio_rim"]), (L[2], P["studio_rim"]), (L[0], P["studio_key"]),
                            (L[3], P["studio_key"]), (L[4], P["studio_key"])):
                key(lo.data, "energy", frames, float(lo.data.energy) * np.clip(arr, 0.0, 1.0), cut)
    # turntable (studio reveal): pad + vehicle on a rotating root
    if "tt_angle" in P:
        tt = U.empty("turntable_root", coll, size=0.3)
        for o in (root, pad, env.get("turntable")):
            if o is not None:
                o.parent = tt
        tt.rotation_mode = "XYZ"
        key(tt, "rotation_euler", frames, np.column_stack([np.zeros(n), np.zeros(n), P["tt_angle"]]), cut)
    key(root, "location", frames, P["pos"], cut)
    key(root, "rotation_quaternion", frames, P["q"], cut)
    for i, hg in enumerate(veh["hinges"]):
        th = veh["thetas"][i]
        e = np.column_stack([-P["delta"][:, i], np.zeros(n), np.full(n, th)])
        key(hg, "rotation_euler", frames, e, cut)
    # rotor: blades fade into the blur disc with spin; the visible angle advances at <= 600 rpm (cosmetic)
    spin = np.clip(P["spin"], 0.0, 1.0)
    dt = np.concatenate([[0.0], np.diff(P["t"])])
    dt = np.where(np.concatenate([[False], P["cut_after"][:-1]]), 0.0, np.maximum(dt, 0.0))
    vis = np.cumsum(np.minimum(spin * 2500.0, 600.0) / 60.0 * 2 * math.pi * dt)
    rot = veh["rotor"]
    key(rot, "rotation_euler", frames, np.column_stack([np.zeros(n), np.zeros(n), vis]), cut)
    for o in veh["blades"] + [veh["rotor_disc"]]:
        o["spin"] = 0.0
        key(o, '["spin"]', frames, spin, cut)
    # fx drivers
    fx_jet = U.empty("fx_jet", coll, size=0.2)
    fx_wind = U.empty("fx_wind", coll, size=0.2)
    fx_time = U.empty("fx_time", coll, size=0.2)
    imp = P["jet_impact"].copy()
    imp[:, 2] = soil
    hit = P["jet_hit"].astype(float)
    jk = P["jet_k"] if "jet_k" in P else np.zeros(n)
    key(fx_jet, "location", frames, imp, cut)
    # scale = (centreline speed at the ground, jet half-width there, wall-jet constant K = sqrt(T / rho)):
    # wall-jet speed u(r) = min(u_ground, K / max(r, b))  (render.plan.wall_jet_speed)
    key(fx_jet, "scale", frames, np.column_stack([np.maximum(P["jet_u"], 1e-3), np.maximum(P["jet_b"], 0.02),
                                                  np.maximum(jk * hit, 1e-4)]), cut)
    key(fx_wind, "location", frames, np.column_stack([P["wind"][:, 0], P["wind"][:, 1], np.zeros(n)]), cut)
    lens = P["cam_lens"]
    cull = np.cos(np.minimum(np.arctan(0.5 * SENSOR_W / lens) + math.radians(15.0), math.radians(89.0)))
    key(fx_time, "location", frames, np.column_stack([P["t"], cull, np.zeros(n)]), cut)
    if kind != "studio":
        # (no heat-haze cone any more: an EDF exhaust is a few kelvin above ambient - art review round 1)
        FX.build_dust(coll, soil, fx_jet, fx_time, indoor=(kind == "indoor"), debris=45 if kind == "indoor" else 700)
        dv = FX.build_dust_volume(coll, indoor=(kind == "indoor"))
        R, Hh = FX.dust_volume_size(P["jet_u"], P["jet_b"])
        # dust only where there is loose dirt: the paver joints of the launch slab (outdoors); over turf the jet
        # flattens grass and throws clippings instead (grass.py / fx debris)
        half = float(env.get("slab_half", 0.0))
        on_slab = ((np.abs(imp[:, 0]) < half + 0.3) & (np.abs(imp[:, 1]) < half + 0.3)).astype(float)
        dust_on = np.clip(P["dust"] * hit, 0.0, 1.0) * on_slab * 0.6
        # the bottom cap 3 cm below the soil: a cap coplanar with the ground confused the volume stack (the ground
        # inside the cylinder rendered as a lighter disc)
        key(dv, "location", frames, imp - np.array([0.0, 0.0, 0.03]), cut)
        key(dv, "scale", frames, np.column_stack([R, R, Hh + 0.03]), cut)
        key(dv, '["dust"]', frames, dust_on, cut)
        key(dv, '["dust_t"]', frames, P["t"], cut)
        # no dust in the frame -> the volume is not rendered at all (no ray-marching cost)
        key(dv, "hide_render", frames, (dust_on < 0.01).astype(float), set(int(f) for f in frames), interp="CONSTANT")
        if kind == "indoor" or args.no_dust_volume or os.environ.get("NO_DUSTVOL"):
            # a clean hall floor raises no visible dust (only the grit flecks above stay)
            bpy.data.objects.remove(dv)
    if kind == "outdoor" and not args.no_grass:
        GR.build_grass(coll, soil, fx_jet, fx_wind, fx_time, exclude_half=float(env.get("slab_half", 1.1)) + 0.02,
                       quality=args.quality, albedo=env.get("sun_info", {}).get("ground_albedo_rgb"),
                       env_img=env.get("env_img"), rot_z=float(env.get("hdri_rot", 0.0)))
    if kind == "outdoor":
        sx, sy = meta.get("windsock_xy", (7.0, 5.0))
        ws = PR.build_windsock(coll, (float(sx), float(sy)), soil, float(meta.get("windsock_h", PR.POLE_H)))
        key(ws["swivel"], "rotation_euler", frames, np.column_stack([np.zeros(n), np.zeros(n), P["sock_yaw"]]), cut)
        for i, sg in enumerate(ws["segs"]):
            key(sg, "rotation_euler", frames, np.column_stack([np.zeros(n), P["sock_droop"][:, i], np.zeros(n)]), cut)
        if "sock_vis" in P and float(np.min(P["sock_vis"])) < 0.5:
            # hidden where only its bare mast would be in frame (render.plan.windsock_visibility)
            for o in ws["objects"]:
                if o.type == "MESH":
                    key(o, "hide_render", frames, (P["sock_vis"] < 0.5).astype(float), set(int(f) for f in frames),
                        interp="CONSTANT")
            info["windsock_hidden_frames"] = int(np.sum(P["sock_vis"] < 0.5))
    if kind in ("outdoor", "indoor"):
        # props keep clear of the operator AND of every camera position (round 1: an extinguisher filled a corner)
        cams = P["cam_pos"][::3]
        avoid = np.concatenate([P["person_feet"][::5], cams]) if (meta.get("person") and "person_feet" in P) else cams
        if meta.get("kill_operator"):
            kx, ky = meta["kill_operator"]["xy"]
            avoid = np.concatenate([avoid, np.array([[kx, ky, 0.0]])])
        views = np.column_stack([P["cam_pos"][::3, :2], P["pos"][::3, :2]])
        shift = P["cam_shift"] if "cam_shift" in P else np.zeros((n, 2))
        cams = {"pos": P["cam_pos"][::2], "quat": P["cam_quat"][::2], "lens": P["cam_lens"][::2], "shift": shift[::2],
                "aspect": scn.render.resolution_y / max(scn.render.resolution_x, 1)}
        PR.build_props(ASSETS, coll, gz, warnings, indoor=(kind == "indoor"), avoid_xy=avoid, hall_fit=meta.get("hall"),
                       views=views, cams=cams)
    if kind == "indoor" and meta.get("tape_end") and "tape_lift" in P:
        te = meta["tape_end"]
        segs = PR.build_tape_end(coll, float(env["soil_z"]) + 0.0006, te["root"], te["dir"], float(te["seg_m"]),
                                 n=P["tape_lift"].shape[1])
        for i, sg in enumerate(segs):
            key(sg, "rotation_euler", frames, np.column_stack([np.zeros(n), -P["tape_lift"][:, i], np.zeros(n)]), cut)
    # person + pole
    info["person"] = "none"
    if meta.get("person"):
        h = PE.load_human(ROOT, coll, warnings)
        if h is None:
            info["person"] = "PLACEHOLDER"
        else:
            pole = PE.build_pole(coll, meta.get("pole"))
            key(pole, "location", frames, P["person_butt"], cut)
            key(pole, "rotation_quaternion", frames, PE.pole_quats(P["person_butt"], P["person_pad"]), cut)
            t0 = time.time()
            PE.neutralise_groin(h, warnings)
            r = PE.drive_human(ROOT, h, P, frames, cut, warnings)
            PE.build_safety_gear(ROOT, coll, r["head_world"], frames, cut)
            ko = meta.get("kill_operator")
            if ko:
                # the second person: the safety officer at the KILL switch (demo rule), in eye / ear protection too
                h2 = PE.load_human(ROOT, coll, warnings)
                if h2 is not None:
                    PE.neutralise_groin(h2, warnings)
                    r2 = PE.drive_kill_operator(ROOT, h2, P, frames, cut, ko, float(meta.get("floor_z", gz - 0.018)), coll,
                                                warnings)
                    PE.build_safety_gear(ROOT, coll, r2["head_world"], frames, cut)
                    info["kill_operator"] = r2["xy"]
            info["person"] = f"render/human/human_rigged.blend posed by tools/human_figure + render.pose, goggles + ear defenders ({r['frames']} frames, " \
                             f"{time.time() - t0:.1f} s)"
    if env.get("key_spot") is not None:
        # The stage key spot is a 'floor key': light-linked to everything EXCEPT the vehicle's own meshes, which still
        # block it (light linking keeps them as shadow casters).  It gives the hover its readable shadow on the board
        # and models the operator, while the matte-black shell stays exposed by the hall light alone (round-2 probe:
        # lighting the shell too put its median at 87-123 sRGB, beige-grey, against the 35-55 target).
        rc = bpy.data.collections.new("key_spot_vehicle_excluded")
        for o in root.children_recursive:
            if o.type == "MESH":
                rc.objects.link(o)
        ks = env["key_spot"]
        ok = False
        if hasattr(ks, "light_linking"):
            ks.light_linking.receiver_collection = rc
            for cob in rc.collection_objects:
                cob.light_linking.link_state = "EXCLUDE"
            ok = True
        info["key_spot"] = {"power_w": float(ks.data.energy), "vehicle_excluded": ok, "excluded_meshes": len(rc.objects)}
        if not ok:
            warnings.append("light linking unavailable: the stage key spot lights the vehicle too")
    # camera
    co, cd = build_camera()
    key(co, "location", frames, P["cam_pos"], cut)
    key(co, "rotation_quaternion", frames, P["cam_quat"], cut)
    key(cd, "lens", frames, P["cam_lens"], cut)
    key(cd, "dof.focus_distance", frames, P["cam_focus"], cut)
    key(cd, "dof.aperture_fstop", frames, P["cam_fstop"], cut)
    if "cam_shift" in P and float(np.max(np.abs(P["cam_shift"]))) > 0:
        key(cd, "shift_x", frames, P["cam_shift"][:, 0], cut)
        key(cd, "shift_y", frames, P["cam_shift"][:, 1], cut)
    if "cam_ev" in P and float(np.max(np.abs(P["cam_ev"]))) > 0:
        # per-shot exposure (render.plan.shot_ev): stepped at the cuts
        key(scn, "view_settings.exposure", frames, float(meta.get("exposure", 0.0)) + P["cam_ev"],
            set(int(f) for f in frames), interp="CONSTANT")
    if "cam_fill" in P and float(np.max(P["cam_fill"])) > 0:
        # Vane close-ups from below.  Round 1 lit the whole shell with a 3 W camera fill and the matte-black body read
        # beige; now two LIGHT-LINKED lights touch only what must read inside the black duct exit:
        #   cam_fill  on-camera soft panel, 3.5 W, receivers = vanes, vane horns, the EDF unit (never the outer shell)
        #   bore      a small disc just inside the exit plane aimed up the bore, receivers = edf_* only (tail cone,
        #             stators, rotor blur disc): the duct no longer reads as a plugged disc
        recv_fill = bpy.data.collections.new("fill_receivers")
        recv_bore = bpy.data.collections.new("bore_receivers")
        for o in bpy.data.objects:
            if o.type != "MESH":
                continue
            if o.name.startswith(("vane_", "edf_")):
                recv_fill.objects.link(o)
            if o.name.startswith("edf_"):
                recv_bore.objects.link(o)       # not the vanes: 1-3 cm from the disc they burn white
        ld = bpy.data.lights.new("cam_fill", "AREA")
        ld.shape, ld.size = "SQUARE", 0.25
        ld.color = (1.0, 0.97, 0.92)
        lo = bpy.data.objects.new("cam_fill", ld)
        U.link(lo)
        lo.parent = co
        lo.location = (0.0, -0.10, 0.0)
        # 3.5 W (round 2: at 1.2 W the vanes were bare silhouettes, sRGB p90 25; now their edges read 20-35)
        fill_w = float(os.environ.get("CAMFILL_W", "3.5"))          # (env override: lighting probes only)
        key(ld, "energy", frames, fill_w * P["cam_fill"], cut)
        bd = bpy.data.lights.new("bore_light", "AREA")
        bd.shape, bd.size = "DISK", 0.07
        bd.color = (0.95, 0.97, 1.0)
        bo = bpy.data.objects.new("bore_light", bd)
        U.link(bo)
        bo.parent = root
        bo.location = (0.0, 0.0, 0.006)
        bo.rotation_euler = (math.pi, 0.0, 0.0)          # area lights emit along local -Z: flip to shine up +Z
        # visible to nothing but its receivers; never seen by the camera itself (a disc light in the exit plane)
        if hasattr(bo, "visible_camera"):
            bo.visible_camera = False
        if hasattr(bo, "visible_glossy"):
            # diffuse light only: its mirror image in the tail cone's tip read as a lamp in the exhaust (round 2)
            bo.visible_glossy = False
        key(bd, "energy", frames, float(os.environ.get("BORE_W", BORE_W)) * P["cam_fill"], cut)
        linked = 0
        for lobj, rc in ((lo, recv_fill), (bo, recv_bore)):
            if hasattr(lobj, "light_linking"):
                lobj.light_linking.receiver_collection = rc
                linked += 1
        if linked < 2:
            warnings.append("light linking unavailable: close-up fill lights the whole vehicle")
        info["fill"] = {"cam_fill_w": fill_w, "bore_w": float(os.environ.get("BORE_W", BORE_W)), "light_linked": linked == 2,
                        "fill_receivers": len(recv_fill.objects), "bore_receivers": len(recv_bore.objects)}
    scn.frame_start, scn.frame_end = 1, n
    info["warnings"] = warnings
    return info


# --------------------------------------------------------------------------- #
# frames
# --------------------------------------------------------------------------- #
def frame_list(args: argparse.Namespace, n: int) -> list[int]:
    if args.frames_file:
        with open(args.frames_file, "r", encoding="utf-8") as f:
            return [int(x) for x in json.load(f)]
    if args.stills:
        return [int(x) for x in args.stills.split(",") if x.strip()]
    if args.frames:
        a, b = args.frames.split(":")
        return list(range(max(1, int(a)), min(n, int(b)) + 1))
    return list(range(1, n + 1))


def render_frames(frames: list[int], out: str, timing_path: str) -> list[tuple[int, float]]:
    scn = bpy.context.scene
    os.makedirs(out, exist_ok=True)
    done: list[tuple[int, float]] = []
    timing = {}
    if timing_path and os.path.exists(timing_path):
        try:
            with open(timing_path, "r", encoding="utf-8") as f:
                timing = json.load(f)
        except (OSError, ValueError):
            timing = {}
    for fr in frames:
        path = os.path.abspath(os.path.join(out, f"frame_{fr:05d}.png"))
        if os.path.exists(path) and os.path.getsize(path) > 0:
            continue
        scn.frame_set(fr)
        scn.render.filepath = path
        t0 = time.time()
        bpy.ops.render.render(write_still=True)
        dt = time.time() - t0
        done.append((fr, dt))
        timing[str(fr)] = round(dt, 2)
        print(f"FRAME {fr} {dt:.2f} s", flush=True)
        if timing_path:
            tmp = timing_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(timing, f)
            os.replace(tmp, timing_path)
    return done


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parse(argv)
    t0 = time.time()
    P, meta = load_plan(args.plan)
    reset_scene()
    warnings: list[str] = []
    setup_render(args.engine, args.quality, args.samples, args.res, args.threads)
    info = build_and_key(P, meta, args, warnings)
    t_build = time.time() - t0
    print("BUILD", json.dumps({"s": round(t_build, 1), **info}, default=str), flush=True)
    if args.save_blend:
        os.makedirs(os.path.dirname(os.path.abspath(args.save_blend)), exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=args.save_blend, compress=True)
    if args.build_only:
        return
    frames = frame_list(args, int(meta["frames"]))
    done = render_frames(frames, args.out, args.timing)
    if done:
        dts = [d for _, d in done]
        print("RENDERED", json.dumps({"n": len(done), "mean_s": round(float(np.mean(dts)), 2),
                                      "first_s": round(dts[0], 2), "build_s": round(t_build, 1)}), flush=True)


if __name__ == "__main__":
    main()
