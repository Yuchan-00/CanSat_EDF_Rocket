"""Environments (run inside Blender): outdoor field, indoor demo hall, studio (vehicle reveal).

Outdoor: HDRI world (Poly Haven alps_field) with its sun painted out and replaced by a Sun lamp of the same
irradiance and direction (``extract_sun``: crisp shadows in both engines, no double sun), a large ground
whose near field is a PBR lawn (instanced grass blades on top, see grass.py) and whose far field shows the
HDRI itself projected along the camera ray (seamless horizon), a concrete launch slab under the pad.

Indoor: the hall HDRI (school_hall) projected onto a box of the hall's size from the capture point (walls and
ceiling keep parallax when the camera moves), a maple strip floor patch around the demo area that blends into
the projected floor, a plywood blast board under the pad, hazard-tape demo square, ceiling area lights.

Studio: dark cyclorama, key / fill / rim area lights, studio HDRI for reflections, turntable disc.

Invariant: the world z = 0 plane is the pad top; the ground is at the log's ground_z (soil 3 cm lower outdoors).
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


# --------------------------------------------------------------------------- #
# HDRI sun extraction
# --------------------------------------------------------------------------- #
def extract_sun(hdr_path: str, cache_dir: str, radius_deg: float = 3.0) -> dict[str, Any]:
    """Find the sun in an equirect HDRI (brightest region), paint it out with the surrounding sky level and
    return its direction (azimuth / elevation in the image's own frame, Blender's equirect convention) and
    irradiance (sum of the removed radiance x solid angle, W/m2-equivalent in world-strength units).
    The cleaned image is cached as OpenEXR next to a JSON with the numbers."""
    os.makedirs(cache_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(hdr_path))[0]
    out_img = os.path.join(cache_dir, base + "_nosun.exr")
    out_js = os.path.join(cache_dir, base + "_sun.json")
    if os.path.exists(out_img) and os.path.exists(out_js):
        with open(out_js, "r", encoding="utf-8") as f:
            info = json.load(f)
        info["image"] = out_img
        return info
    img = bpy.data.images.load(hdr_path, check_existing=True)
    w, h = img.size
    px = np.empty(w * h * 4, dtype=np.float32)
    img.pixels.foreach_get(px)
    px = px.reshape(h, w, 4)
    lum = 0.2126 * px[..., 0] + 0.7152 * px[..., 1] + 0.0722 * px[..., 2]
    r, c = np.unravel_index(int(np.argmax(lum)), lum.shape)
    v = (r + 0.5) / h
    u = (c + 0.5) / w
    el = (v - 0.5) * math.pi
    az = (0.5 - u) * 2 * math.pi            # Blender equirect: u = -atan2(y, x) / 2pi + 0.5
    rows = (np.arange(h) + 0.5) / h
    cols = (np.arange(w) + 0.5) / w
    EL = ((rows - 0.5) * math.pi)[:, None]
    AZ = ((0.5 - cols) * 2 * math.pi)[None, :]
    sd = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    dots = np.cos(EL) * np.cos(AZ) * sd[0] + np.cos(EL) * np.sin(AZ) * sd[1] + np.sin(EL) * sd[2]
    ang = np.degrees(np.arccos(np.clip(dots, -1, 1)))
    disk = ang < radius_deg
    ring = (ang >= radius_deg) & (ang < radius_deg * 2.0)
    sky = np.median(px[ring][:, :3], axis=0)
    dOmega = (2 * math.pi / w) * (math.pi / h) * np.cos(EL)              # solid angle per pixel
    dOm = np.broadcast_to(dOmega, lum.shape)
    excess = np.clip(px[..., :3] - sky[None, None, :], 0.0, None)
    E = (excess[disk] * dOm[disk][:, None]).sum(axis=0)                  # irradiance normal to the sun, RGB
    # sky irradiance on a horizontal plane (upper hemisphere, cosine weighted) for a sanity ratio
    up = (EL > 0)
    E_sky = float(((lum - np.where(disk, lum - (0.2126 * sky[0] + 0.7152 * sky[1] + 0.0722 * sky[2]), 0)) * dOm * np.sin(EL) * up).sum())
    px2 = px.copy()
    px2[disk, :3] = sky[None, :]
    new = bpy.data.images.new(base + "_nosun", w, h, alpha=False, float_buffer=True)
    new.pixels.foreach_set(px2.reshape(-1))
    new.filepath_raw = out_img
    new.file_format = "OPEN_EXR"
    try:
        new.save()
    except RuntimeError:
        new.save_render(out_img)
    lumE = float(0.2126 * E[0] + 0.7152 * E[1] + 0.0722 * E[2])
    # the photographed ground just below the horizon (2 - 12 deg down): its radiance and the albedo it implies
    band = (EL[:, 0] < -math.radians(2.0)) & (EL[:, 0] > -math.radians(12.0))
    g_rgb = px[band][:, :, :3].reshape(-1, 3).mean(axis=0)
    E_ground = lumE * max(math.sin(el), 0.0) + E_sky
    albedo = g_rgb * math.pi / max(E_ground, 1e-6)
    info = {"az_rad": az, "el_rad": el, "irradiance_rgb": [float(x) for x in E], "irradiance": lumE,
            "ground_rgb": [float(x) for x in g_rgb], "ground_albedo_rgb": [float(x) for x in albedo],
            "sky_horizontal_irradiance": E_sky, "peak": float(lum[r, c]), "sky_rgb": [float(x) for x in sky],
            "radius_deg": radius_deg, "source": os.path.basename(hdr_path)}
    with open(out_js, "w", encoding="utf-8") as f:
        json.dump(info, f, indent=1)
    info["image"] = out_img
    bpy.data.images.remove(new)
    return info


def image_mean_linear(path: str) -> list[float]:
    """Mean linear RGB of an sRGB image file (Blender converts on load)."""
    img = bpy.data.images.load(path, check_existing=True)
    img.colorspace_settings.name = "sRGB"
    w, h = img.size
    px = np.empty(w * h * 4, dtype=np.float32)
    img.pixels.foreach_get(px)
    return [float(x) for x in px.reshape(-1, 4)[:, :3].mean(axis=0)]


# --------------------------------------------------------------------------- #
# worlds
# --------------------------------------------------------------------------- #
def world_hdri(path: str, rot_z_rad: float = 0.0, strength: float = 1.0, name: str = "world") -> Any:
    w = bpy.data.worlds.get(name) or bpy.data.worlds.new(name)
    bpy.context.scene.world = w
    try:
        w.use_nodes = True
    except AttributeError:
        pass
    nb = U.NodeBuilder(w.node_tree)
    tc = nb.n("ShaderNodeTexCoord")
    mp = nb.n("ShaderNodeMapping")
    mp.inputs["Rotation"].default_value = (0.0, 0.0, rot_z_rad)
    nb.l(tc.outputs["Generated"], mp.inputs["Vector"])
    env = nb.n("ShaderNodeTexEnvironment", name="env")
    env.image = bpy.data.images.load(path, check_existing=True)
    nb.l(mp.outputs["Vector"], env.inputs["Vector"])
    bg = nb.n("ShaderNodeBackground", inputs={"Strength": strength})
    nb.l(env.outputs["Color"], bg.inputs["Color"])
    out = nb.n("ShaderNodeOutputWorld")
    nb.l(bg.outputs[0], out.inputs["Surface"])
    return w


def sun_lamp(az: float, el: float, energy: float, angle_deg: float = 0.6, color: tuple = (1.0, 0.97, 0.92)) -> Any:
    """Sun light arriving FROM azimuth az / elevation el (rad)."""
    ld = bpy.data.lights.new("sun", "SUN")
    ld.energy = energy
    ld.angle = math.radians(angle_deg)
    ld.color = color
    o = bpy.data.objects.new("sun", ld)
    U.link(o)
    d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    from mathutils import Vector
    o.rotation_mode = "QUATERNION"
    o.rotation_quaternion = Vector((-d[0], -d[1], -d[2])).to_track_quat("-Z", "Y")
    return o


def env_vector_from_position(nb: U.NodeBuilder, capture: tuple[float, float, float], rot_z: float) -> Any:
    """Direction from the HDRI capture point to the shaded point, rotated like the world mapping."""
    geo = nb.n("ShaderNodeNewGeometry")
    sub = nb.n("ShaderNodeVectorMath", operation="SUBTRACT")
    nb.l(geo.outputs["Position"], sub.inputs[0])
    sub.inputs[1].default_value = capture
    mp = nb.n("ShaderNodeMapping")
    mp.inputs["Rotation"].default_value = (0.0, 0.0, rot_z)
    nb.l(sub.outputs[0], mp.inputs["Vector"])
    return mp.outputs["Vector"]


# --------------------------------------------------------------------------- #
# outdoor
# --------------------------------------------------------------------------- #
def grid_mesh(name: str, size: float, n: int, z: float, coll: Any) -> Any:
    xs = np.linspace(-size / 2, size / 2, n + 1)
    X, Y = np.meshgrid(xs, xs)
    v = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, z)])
    idx = np.arange((n + 1) * (n + 1)).reshape(n + 1, n + 1)
    f = np.stack([idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, 1:].ravel(), idx[1:, :-1].ravel()], axis=1)
    o = U.mesh_object(name, v, f, coll, smooth_angle_deg=None)
    # UVs not needed: materials use world position
    return o


def ground_material_outdoor(asset_dir: str, env_img: Any, rot_z: float, blend_near: float = 30.0, blend_far: float = 110.0,
                            albedo: Any = None) -> Any:
    """Lawn soil + short grass (PBR, two scales against tiling, macro colour variation); camera rays further than
    ``blend_near``..``blend_far`` metres show the HDRI's own ground instead (seamless horizon)."""
    m, nb, out = U.new_material("ground_lawn")
    files = M.pbr_set(asset_dir, "leafy_grass")
    geo = nb.n("ShaderNodeNewGeometry")
    # macro variation
    mac = nb.n("ShaderNodeTexNoise", inputs={"Scale": 0.08, "Detail": 4.0, "Roughness": 0.55})
    nb.l(geo.outputs["Position"], mac.inputs["Vector"])
    mac2 = nb.n("ShaderNodeTexNoise", inputs={"Scale": 0.6, "Detail": 3.0})
    nb.l(geo.outputs["Position"], mac2.inputs["Vector"])
    if files:
        v1 = M.world_xy_vector(nb, 2.0)
        v2 = M.world_xy_vector(nb, 5.3, rot_deg=37.0, offset=(0.31, 0.77))
        a = M.pbr_nodes(nb, files, v1, nor_strength=0.8)
        b = M.pbr_nodes(nb, files, v2, nor_strength=0.6)
        col = nb.mix_rgb(mac2.outputs["Fac"], a["color"], b["color"])
        if albedo is not None:
            # scale the texture so its mean equals the albedo the photographed field implies (colour match)
            mean = image_mean_linear(files["diff"])
            # x1.25: the band 2-12 deg below the horizon under-reads the near meadow (round-1 measurement)
            gain = tuple(float(np.clip(1.25 * albedo[i] / max(mean[i], 1e-4), 0.05, 4.0)) for i in range(3)) + (1.0,)
            col = nb.mix_rgb(1.0, col, gain, blend="MULTIPLY")
        rough = nb.math("ADD", nb.math("MULTIPLY", a["rough"], 0.5), nb.math("MULTIPLY", b["rough"], 0.5))
        nmix = nb.n("ShaderNodeMix", data_type="VECTOR")
        nb.l(mac2.outputs["Fac"], nmix.inputs["Factor"])
        nb.l(a["normal"], nmix.inputs[4])
        nb.l(b["normal"], nmix.inputs[5])
        normal = nmix.outputs[1]
    else:
        col = (0.09, 0.16, 0.05, 1.0)
        rough = 0.8
        normal = None
    # greener / drier patches
    tint = nb.n("ShaderNodeMapRange", inputs={1: 0.35, 2: 0.65, 3: 0.0, 4: 1.0})
    nb.l(mac.outputs["Fac"], tint.inputs[0])
    dry = nb.mix_rgb(1.0, col, (0.95, 0.85, 0.6, 1.0), blend="MULTIPLY")
    col2 = nb.mix_rgb(nb.math("MULTIPLY", tint.outputs[0], 0.55), col, dry)
    p = U.principled(nb)
    nb.l(col2, p.inputs["Base Color"])
    if isinstance(rough, (int, float)):
        p.inputs["Roughness"].default_value = rough
    else:
        nb.l(rough, p.inputs["Roughness"])
    if normal is not None:
        nb.l(normal, p.inputs["Normal"])
    p.inputs["Specular IOR Level"].default_value = 0.15
    # far field: the HDRI along the view ray (emission), camera rays only
    env = nb.n("ShaderNodeTexEnvironment")
    env.image = env_img
    inc = nb.n("ShaderNodeVectorMath", operation="SCALE", inputs={"Scale": -1.0})
    nb.l(geo.outputs["Incoming"], inc.inputs[0])
    mp = nb.n("ShaderNodeMapping")
    mp.inputs["Rotation"].default_value = (0.0, 0.0, rot_z)
    nb.l(inc.outputs[0], mp.inputs["Vector"])
    nb.l(mp.outputs["Vector"], env.inputs["Vector"])
    em = nb.n("ShaderNodeEmission", inputs={"Strength": 1.0})
    nb.l(env.outputs["Color"], em.inputs["Color"])
    cam = nb.n("ShaderNodeCameraData")
    far = nb.n("ShaderNodeMapRange", inputs={1: blend_near, 2: blend_far, 3: 0.0, 4: 1.0})
    far.clamp = True
    far.interpolation_type = "SMOOTHSTEP"
    # a noisy, feathered transition (+-7 m) instead of one hard ring where the CG lawn meets the photographed meadow
    fn = nb.n("ShaderNodeTexNoise", inputs={"Scale": 0.09, "Detail": 3.0, "Roughness": 0.6})
    nb.l(geo.outputs["Position"], fn.inputs["Vector"])
    vd = nb.math("ADD", cam.outputs["View Distance"], nb.math("MULTIPLY", nb.math("SUBTRACT", fn.outputs["Fac"], 0.5), 14.0))
    nb.l(vd, far.inputs[0])
    lp = nb.n("ShaderNodeLightPath")
    fac = nb.math("MULTIPLY", far.outputs[0], lp.outputs["Is Camera Ray"])
    mix = nb.n("ShaderNodeMixShader")
    nb.l(fac, mix.inputs[0])
    nb.l(p.outputs[0], mix.inputs[1])
    nb.l(em.outputs[0], mix.inputs[2])
    nb.l(mix.outputs[0], out.inputs["Surface"])
    return m


PAVER = 0.5          # 50 cm square concrete pavers, 3 x 3, 12 mm joints, 6 cm thick: a 1.5 m launch square
PAVER_N = 3
PAVER_GAP = 0.012
# fixed per-paver height offset (m) and yaw (deg): laid by hand, not perfectly flush
PAVER_JITTER = ((0.002, 0.4), (-0.001, -0.3), (0.0015, 0.2), (-0.002, -0.5), (0.0, 0.0), (0.001, 0.6), (0.0025, -0.2),
                (-0.0015, 0.3), (0.0005, -0.6))


def concrete_slab(asset_dir: str, coll: Any, top_z: float, half: float = 0.75, thick: float = 0.06) -> Any:
    """The outdoor launch area: 3 x 3 weathered concrete pavers set into the lawn (top ~8 mm above the soil, the turf
    grows up to the edge), sandy dirt in the 12 mm joints, warm dirt / grass-stain wear toward the edges and a faint
    darkened ring around the pad from repeated run-ups."""
    m, nb, out = U.new_material("concrete_slab")
    files = M.pbr_set(asset_dir, "concrete_floor_worn_001")
    p = U.principled(nb)
    geo = nb.n("ShaderNodeNewGeometry")
    if files:
        # warmer, less saturated than the bare texture (round 1 read clean and bluish)
        t = M.pbr_nodes(nb, files, M.world_xy_vector(nb, 1.6), rough_scale=1.0, nor_strength=1.0,
                        tint=(0.5, 0.45, 0.74))
        # (round 1b: still measured bluish, sRGB 98/101/104 under the sky - warmer, sandier)
        col0 = nb.mix_rgb(0.65, t["color"], (0.55, 0.47, 0.36, 1.0), blend="MULTIPLY")
        ln = nb.n("ShaderNodeVectorMath", operation="LENGTH")
        sep = nb.n("ShaderNodeSeparateXYZ")
        nb.l(geo.outputs["Position"], sep.inputs[0])
        comb = nb.n("ShaderNodeCombineXYZ")
        nb.l(sep.outputs[0], comb.inputs[0])
        nb.l(sep.outputs[1], comb.inputs[1])
        nb.l(comb.outputs[0], ln.inputs[0])
        ring = nb.n("ShaderNodeMapRange", inputs={1: 0.05, 2: 0.55, 3: 0.30, 4: 0.0})
        ring.clamp = True
        nb.l(ln.outputs["Value"], ring.inputs[0])
        col1 = nb.mix_rgb(ring.outputs[0], col0, (0.10, 0.095, 0.085, 1.0))
        # dirt / grass stain creeping in from the slab edge (Chebyshev distance to the edge, noisy)
        ax = nb.math("MAXIMUM", nb.math("ABSOLUTE", sep.outputs[0]), nb.math("ABSOLUTE", sep.outputs[1]))
        dn = nb.n("ShaderNodeTexNoise", inputs={"Scale": 6.0, "Detail": 5.0, "Roughness": 0.65})
        nb.l(geo.outputs["Position"], dn.inputs["Vector"])
        edge = nb.n("ShaderNodeMapRange", inputs={1: half - 0.22, 2: half, 3: 0.0, 4: 0.75})
        edge.clamp = True
        nb.l(nb.math("ADD", ax, nb.math("MULTIPLY", nb.math("SUBTRACT", dn.outputs["Fac"], 0.5), 0.25)), edge.inputs[0])
        col2 = nb.mix_rgb(edge.outputs[0], col1, (0.12, 0.11, 0.06, 1.0))
        # art review round 2 ("uniform grey plastic with an even sheen"): every paver its own cast (+-12 % value, a
        # little hue), blotchy weathering stains, aggregate speckle, darker chipped / dirty arrises
        oi = nb.n("ShaderNodeObjectInfo")
        pv = nb.n("ShaderNodeHueSaturation", inputs={"Fac": 1.0})
        nb.l(nb.math("ADD", 0.48, nb.math("MULTIPLY", oi.outputs["Random"], 0.04)), pv.inputs["Hue"])
        nb.l(nb.math("ADD", 0.85, nb.math("MULTIPLY", oi.outputs["Random"], 0.25)), pv.inputs["Saturation"])
        nb.l(nb.math("ADD", 0.88, nb.math("MULTIPLY", oi.outputs["Random"], 0.24)), pv.inputs["Value"])
        nb.l(col2, pv.inputs["Color"])
        stain = nb.n("ShaderNodeTexNoise", inputs={"Scale": 3.2, "Detail": 6.0, "Roughness": 0.7})
        nb.l(geo.outputs["Position"], stain.inputs["Vector"])
        st = nb.n("ShaderNodeMapRange", inputs={1: 0.48, 2: 0.70, 3: 0.0, 4: 0.45})
        st.clamp = True
        nb.l(stain.outputs["Fac"], st.inputs[0])
        col3 = nb.mix_rgb(st.outputs[0], pv.outputs["Color"], (0.07, 0.065, 0.05, 1.0))
        spk = nb.n("ShaderNodeTexVoronoi", inputs={"Scale": 260.0})
        nb.l(geo.outputs["Position"], spk.inputs["Vector"])
        sk = nb.n("ShaderNodeMapRange", inputs={1: 0.0, 2: 0.12, 3: 0.35, 4: 0.0})
        sk.clamp = True
        nb.l(spk.outputs["Distance"], sk.inputs[0])
        col4 = nb.mix_rgb(sk.outputs[0], col3, (0.32, 0.30, 0.27, 1.0))
        # arrises: object-space distance to the paver's own edge (pavers are 0.5 m boxes centred on their origin)
        tco = nb.n("ShaderNodeTexCoord")
        so = nb.n("ShaderNodeSeparateXYZ")
        nb.l(tco.outputs["Object"], so.inputs[0])
        ed = nb.math("SUBTRACT", PAVER / 2, nb.math("MAXIMUM", nb.math("ABSOLUTE", so.outputs[0]), nb.math("ABSOLUTE", so.outputs[1])))
        en = nb.n("ShaderNodeTexNoise", inputs={"Scale": 40.0, "Detail": 3.0})
        nb.l(geo.outputs["Position"], en.inputs["Vector"])
        arr = nb.n("ShaderNodeMapRange", inputs={1: 0.0, 2: 0.03, 3: 0.55, 4: 0.0})
        arr.clamp = True
        nb.l(nb.math("ADD", ed, nb.math("MULTIPLY", nb.math("SUBTRACT", en.outputs["Fac"], 0.5), 0.02)), arr.inputs[0])
        col5 = nb.mix_rgb(arr.outputs[0], col4, (0.06, 0.055, 0.045, 1.0))
        nb.l(col5, p.inputs["Base Color"])
        nb.l(nb.math("ADD", t["rough"], 0.30, clamp=True), p.inputs["Roughness"])
        p.inputs["Specular IOR Level"].default_value = 0.22
        nb.l(t["normal"], p.inputs["Normal"])
    else:
        p.inputs["Base Color"].default_value = (0.24, 0.22, 0.20, 1)
    nb.l(p.outputs[0], out.inputs["Surface"])
    root = U.empty("launch_slab", coll, size=0.3)
    # sandy dirt in the joints (a plane just below the paver tops, visible only through the gaps)
    jm, nb2, out2 = U.new_material("paver_joint_dirt")
    jn = nb2.n("ShaderNodeTexNoise", inputs={"Scale": 140.0, "Detail": 4.0})
    g2 = nb2.n("ShaderNodeNewGeometry")
    nb2.l(g2.outputs["Position"], jn.inputs["Vector"])
    jc = nb2.mix_rgb(jn.outputs["Fac"], (0.10, 0.085, 0.06, 1.0), (0.17, 0.15, 0.11, 1.0))
    jp = U.principled(nb2, Roughness=0.95, Specular_IOR_Level=0.2)
    nb2.l(jc, jp.inputs["Base Color"])
    jb = nb2.n("ShaderNodeBump", inputs={"Strength": 0.6, "Distance": 0.002})
    nb2.l(jn.outputs["Fac"], jb.inputs["Height"])
    nb2.l(jb.outputs["Normal"], jp.inputs["Normal"])
    nb2.l(jp.outputs[0], out2.inputs["Surface"])
    hj = half + 0.01
    U.mesh_object("paver_joints", U.box_verts(-hj, hj, -hj, hj, top_z - 0.06, top_z - 0.007), U.box_faces(), coll,
                  parent=root, smooth_angle_deg=None, mat=jm)
    k = 0
    pitch = PAVER + PAVER_GAP
    for i in range(PAVER_N):
        for j in range(PAVER_N):
            dz, yaw = PAVER_JITTER[k % len(PAVER_JITTER)]
            cx, cy = (i - (PAVER_N - 1) / 2) * pitch, (j - (PAVER_N - 1) / 2) * pitch
            v = U.box_verts(-PAVER / 2, PAVER / 2, -PAVER / 2, PAVER / 2, -thick, 0.0)
            o = U.mesh_object(f"paver_{i}{j}", v, U.box_faces(), coll, parent=root, smooth_angle_deg=None, mat=m)
            o.location = (cx, cy, top_z + (0.0 if (i, j) == (1, 1) else dz))
            o.rotation_euler = (0.0, 0.0, math.radians(yaw if (i, j) != (1, 1) else 0.0))
            bev = o.modifiers.new("bevel", "BEVEL")
            bev.width = 0.006
            bev.segments = 2
            k += 1
    return root


def build_outdoor(asset_dir: str, cache_dir: str, ground_z: float, sun_az_deg: float, coll: Any, warnings: list[str],
                  hdri_id: str = "meadow_2") -> dict[str, Any]:
    hdr = os.path.join(asset_dir, hdri_id, f"{hdri_id}_4k.hdr")
    if not os.path.exists(hdr):
        warnings.append(f"{hdri_id} HDRI missing: plain sky colour")
        w = bpy.data.worlds.new("world")
        bpy.context.scene.world = w
        return {"sun": sun_lamp(math.radians(sun_az_deg), math.radians(35), 4.0)}
    info = extract_sun(hdr, cache_dir)
    # rotate the HDRI so its sun comes from sun_az_deg
    rot = math.radians(sun_az_deg) - info["az_rad"]
    world_hdri(info["image"], rot_z_rad=rot, strength=1.0)
    E = float(info["irradiance"])
    E_sky = float(info["sky_horizontal_irradiance"])
    # sanity: a clear-sky sun gives 3-10x the diffuse sky irradiance on the sun-normal plane; clipped HDRIs less
    if E < 2.5 * E_sky:
        warnings.append(f"HDRI sun looks clipped (E_sun {E:.2f} vs sky {E_sky:.2f}): sun lamp raised to 4x sky")
        E = 4.0 * E_sky
    rgb = np.array(info["irradiance_rgb"])
    col = tuple((rgb / max(rgb.max(), 1e-9)).tolist())
    sun = sun_lamp(math.radians(sun_az_deg), float(info["el_rad"]), E, angle_deg=0.6, color=col)
    env_img = bpy.data.images.load(info["image"], check_existing=True)
    soil_z = ground_z - 0.008            # pavers set flush into the lawn (round 1: a raw 3 cm step on top of the turf)
    g = grid_mesh("ground", 4000.0, 1, soil_z, coll)
    # far field: CG lawn -> the photographed meadow between 14 and 30 m (the grass blades fade into the same
    # projection from 9 m on, grass.FAR_BLEND): beyond the CG grass ring (30 m) only the photograph remains
    g.data.materials.append(ground_material_outdoor(asset_dir, env_img, rot, blend_near=14.0, blend_far=30.0,
                                                     albedo=info.get("ground_albedo_rgb")))
    slab = concrete_slab(asset_dir, coll, ground_z)
    half = 0.5 * (PAVER_N * PAVER + (PAVER_N - 1) * PAVER_GAP)
    return {"sun": sun, "sun_info": info, "hdri_rot": rot, "soil_z": soil_z, "ground": g, "slab": slab,
            "slab_half": half, "env_img": env_img}


# --------------------------------------------------------------------------- #
# indoor hall
# --------------------------------------------------------------------------- #
HALL = {"half_x": 12.0, "half_y": 9.0, "height": 8.5, "capture_h": 1.6}
HALL_WORLD_STRENGTH = 0.75        # the hall HDRI as the world light (the projected backplate keeps its own 1.0)


TAPE_HALF = 0.80                   # the keep-out square (tape) around the plywood board; the operator stays outside


def _quads_object(name: str, quads: list, coll: Any, mat: Any) -> Any:
    verts, faces = [], []
    for q in quads:
        b = len(verts)
        verts += [list(p) for p in q]
        faces.append([b, b + 1, b + 2, b + 3])
    return U.mesh_object(name, np.array(verts, float), faces, coll, smooth_angle_deg=None, mat=mat)


def build_indoor(asset_dir: str, ground_z: float, coll: Any, warnings: list[str], hall_fit: dict | None = None
                 ) -> dict[str, Any]:
    """The demo on the school stage (render/hall.py): the HDRI is projected from its capture point onto proxy geometry
    that follows the real room - the curtained stage (back + wings + ceiling) and, beyond the stage edge, the
    auditorium 1 m lower (floor, windowed side walls, back wall, ceiling, proscenium wall); the whole stage floor is CG
    maple (no projected floor anywhere on the stage) ending at a CG stage-front apron.  The projection proxies are seen
    by camera and glossy rays only; the HDRI world lights the scene, plus four warm stage lights overhead."""
    import sys
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if root not in sys.path:
        sys.path.insert(0, root)
    from render import hall as HL
    floor_z = ground_z - 0.018                      # the pad stands on an 18 mm plywood board on the floor
    if hall_fit:
        hall = HL.Hall(rot=float(hall_fit["rot_rad"]), veh_img=np.array(hall_fit["veh_img"], float), floor_z=floor_z)
    else:
        hall = HL.Hall(rot=math.radians(90.0), veh_img=np.zeros(2), floor_z=floor_z)
        warnings.append("indoor: no hall fit in the plan; default stage placement")
    rot = hall.rot
    hdr = os.path.join(asset_dir, "school_hall", "school_hall_4k.hdr")
    if os.path.exists(hdr):
        world_hdri(hdr, rot_z_rad=rot, strength=float(os.environ.get("HALL_WORLD_S", str(HALL_WORLD_STRENGTH))))
        env_img = bpy.data.images.load(hdr, check_existing=True)
        if tuple(env_img.size) == (4096, 2048):
            paint_out_signs(env_img)            # (the world node shares this image datablock)
    else:
        warnings.append("school_hall HDRI missing: grey hall")
        env_img = None
    cx, cy = (float(x) for x in hall.cap_xy)
    cap = (cx, cy, floor_z + HL.CAP_H)
    m, nb, out = U.new_material("hall_projection")
    if env_img is not None:
        env = nb.n("ShaderNodeTexEnvironment")
        env.image = env_img
        nb.l(env_vector_from_position(nb, cap, rot), env.inputs["Vector"])
        em = nb.n("ShaderNodeEmission", inputs={"Strength": 1.0})
        nb.l(env.outputs["Color"], em.inputs["Color"])
        nb.l(em.outputs[0], out.inputs["Surface"])
    quads = HL.proxy_quads(hall)
    proxies = []
    for role in ("stage_walls", "auditorium"):
        o = _quads_object(f"hall_{role}", quads[role], coll, m)
        for attr in ("visible_diffuse", "visible_shadow", "visible_transmission", "visible_volume_scatter"):
            if hasattr(o, attr):
                setattr(o, attr, False)
        proxies.append(o)
    # the stage-front apron (CG, black-painted plywood) and the stage floor (CG maple, all of it)
    am, nb, out = U.new_material("stage_apron")
    p = U.principled(nb, Base_Color=(0.025, 0.024, 0.023, 1.0), Roughness=0.7)
    nb.l(p.outputs[0], out.inputs["Surface"])
    apron = _quads_object("stage_front", quads["stage_front"], coll, am)
    fm, nb, out = U.new_material("hall_floor")
    files = M.pbr_set(asset_dir, "rectangular_parquet")
    p = U.principled(nb)
    if files:
        t = M.pbr_nodes(nb, files, M.world_xy_vector(nb, 2.25, rot_deg=90.0 - math.degrees(rot)),
                        tint=(0.5, 0.85, 1.05), nor_strength=0.5)
        nb.l(t["color"], p.inputs["Base Color"])
        nb.l(nb.math("ADD", nb.math("MULTIPLY", t["rough"], 0.75), 0.06), p.inputs["Roughness"])
        nb.l(t["normal"], p.inputs["Normal"])
    p.inputs["Coat Weight"].default_value = 0.30
    p.inputs["Coat Roughness"].default_value = 0.16
    nb.l(p.outputs[0], out.inputs["Surface"])
    floor = _quads_object("hall_floor", quads["stage_floor"], coll, fm)
    board = plywood_board(asset_dir, coll, ground_z)
    tape = hazard_square(coll, floor_z + 0.0006, TAPE_HALF)
    lights = stage_lights(coll, hall, floor_z)
    return {"proxies": proxies, "apron": apron, "floor": floor, "board": board, "tape": tape, "lights": lights,
            "hall": hall.to_json(), "hdri_rot": rot, "env_img": env_img, "soil_z": floor_z, "capture": cap,
            "tape_half": TAPE_HALF}


def stage_lights(coll: Any, hall: Any, floor_z: float, power_w: float = 70.0) -> list[Any]:
    """Four warm soft stage lights 6.2 m above the stage floor over the demo area (the photo's own rig is out of
    frame): top light on the operator and the floor, a gentle top edge on the black vehicle."""
    out = []
    for i, (x, y) in enumerate(((-1.8, -0.5), (2.2, -0.5), (-1.8, 2.6), (2.2, 2.6))):
        w = hall.to_world(np.array([x, y]))
        ld = bpy.data.lights.new(f"stage_light_{i}", "AREA")
        ld.shape = "RECTANGLE"
        ld.size, ld.size_y = 1.0, 0.5
        ld.energy = power_w
        ld.color = (1.0, 0.96, 0.91)          # (round 1b: less amber - the black body read brown under 0.93 / 0.84)
        o = bpy.data.objects.new(f"stage_light_{i}", ld)
        o.location = (float(w[0]), float(w[1]), floor_z + 6.2)
        U.link(o, coll)
        out.append(o)
    return out


# Lettering on the auditorium's balcony front in the school_hall photo ("GLO EN DIEN", twice; art review round 2: a
# large foreign sign right behind the operator's head).  Equirect boxes (u0, u1, v0, v1; v from the TOP), measured on
# the 4k file (rows 997-1021, columns 3045-3168 and 3295-3418).
SIGN_BOXES = ((3045 / 4096, 3168 / 4096, 997 / 2048, 1021 / 2048), (3295 / 4096, 3418 / 4096, 997 / 2048, 1021 / 2048))


def paint_out_signs(img: Any, boxes: Any = SIGN_BOXES) -> int:
    """Paint the lettering out of the loaded hall HDRI in memory (the file is untouched): inside each box every column
    is interpolated linearly between the clean pixel just above and just below it - the balcony front is vertical
    wooden slats, so each column continues its slat.  Returns the number of pixels replaced."""
    w, h = img.size
    px = np.empty(w * h * 4, dtype=np.float32)
    img.pixels.foreach_get(px)
    px = px.reshape(h, w, 4)
    n = 0
    for u0, u1, v0, v1 in boxes:
        c0, c1 = int(u0 * w), int(math.ceil(u1 * w))
        r_top, r_bot = int(v0 * h) - 1, int(math.ceil(v1 * h)) + 1          # image rows from the top (clean)
        b_top, b_bot = h - 1 - r_top, h - 1 - r_bot                          # Blender rows (bottom-up)
        a = px[b_top, c0:c1, :3].copy()
        b = px[b_bot, c0:c1, :3].copy()
        for rb in range(b_bot + 1, b_top):
            t = (rb - b_bot) / max(b_top - b_bot, 1)
            px[rb, c0:c1, :3] = (1.0 - t) * b + t * a
            n += c1 - c0
    img.pixels.foreach_set(px.reshape(-1))
    img.update()
    return n


def stage_key_spot(coll: Any, kl: dict) -> Any:
    """The motivated stage spot over the demo square (render.plan KEY_SPOT, plan meta 'key_light'): a soft-edged spot
    (0.25 m radius source) 4.2 m above the floor, 25 deg off vertical, warm-white, so the vehicle, its stand, the pole
    and the operator cast soft but readable shadows on the board and the stage (art review round 2)."""
    from mathutils import Vector
    ld = bpy.data.lights.new("stage_key_spot", "SPOT")
    ld.energy = float(os.environ.get("KEY_W", kl.get("power_w", 900.0)))
    ld.shadow_soft_size = float(kl.get("radius_m", 0.25))
    ld.spot_size = math.radians(float(kl.get("spot_deg", 60.0)))
    ld.spot_blend = float(kl.get("blend", 0.45))
    ld.color = (1.0, 0.90, 0.80)                    # a warm stage lamp, like the hall's own light (white-balanced)
    o = bpy.data.objects.new("stage_key_spot", ld)
    o.location = tuple(float(x) for x in kl["pos"])
    o.rotation_mode = "QUATERNION"
    o.rotation_quaternion = (Vector(kl["aim"]) - Vector(kl["pos"])).to_track_quat("-Z", "Y")
    U.link(o, coll)
    return o


def plywood_board(asset_dir: str, coll: Any, ground_z: float, sx: float = 1.22, sy: float = 1.22, t: float = 0.018) -> Any:
    m, nb, out = U.new_material("plywood")
    files = M.pbr_set(asset_dir, "plywood")
    p = U.principled(nb)
    if files:
        tx = M.pbr_nodes(nb, files, M.world_xy_vector(nb, 0.5), nor_strength=0.6)
        nb.l(tx["color"], p.inputs["Base Color"])
        nb.l(tx["rough"], p.inputs["Roughness"])
        nb.l(tx["normal"], p.inputs["Normal"])
    nb.l(p.outputs[0], out.inputs["Surface"])
    z1 = ground_z + 0.0
    z0 = ground_z - t
    v = U.box_verts(-sx / 2, sx / 2, -sy / 2, sy / 2, z0, z1)
    return U.mesh_object("plywood_board", v, U.box_faces(), coll, smooth_angle_deg=None, mat=m)


def hazard_square(coll: Any, z: float, half: float, w: float = 0.05) -> Any:
    """Yellow / black floor tape square marking the demo area."""
    m, nb, out = U.new_material("hazard_tape")
    geo = nb.n("ShaderNodeNewGeometry")
    sep = nb.n("ShaderNodeSeparateXYZ")
    nb.l(geo.outputs["Position"], sep.inputs[0])
    diag = nb.math("ADD", sep.outputs[0], sep.outputs[1])
    stripes = nb.math("GREATER_THAN", nb.math("FRACT", nb.math("MULTIPLY", diag, 1.0 / 0.1)), 0.5)
    col = nb.mix_rgb(stripes, U.srgb("#f2c200"), U.srgb("#151515"))
    p = U.principled(nb, Roughness=0.35)
    nb.l(col, p.inputs["Base Color"])
    nb.l(p.outputs[0], out.inputs["Surface"])
    verts, faces = [], []
    for (x0, y0, x1, y1) in ((-half, -half, half, -half + w), (-half, half - w, half, half),
                             (-half, -half + w, -half + w, half - w), (half - w, -half + w, half, half - w)):
        b = len(verts)
        verts += [[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]]
        faces.append([b, b + 1, b + 2, b + 3])
    return U.mesh_object("hazard_tape", np.array(verts), faces, coll, smooth_angle_deg=None, mat=m)


def ceiling_lights(coll: Any, z: float, hx: float, hy: float, power_w: float = 40.0) -> list[Any]:
    out = []
    for i, x in enumerate(np.linspace(-hx * 0.6, hx * 0.6, 4)):
        for j, y in enumerate(np.linspace(-hy * 0.55, hy * 0.55, 3)):
            ld = bpy.data.lights.new(f"ceiling_{i}{j}", "AREA")
            ld.shape = "RECTANGLE"
            ld.size, ld.size_y = 1.2, 0.3
            ld.energy = power_w
            ld.color = (1.0, 0.96, 0.9)
            o = bpy.data.objects.new(f"ceiling_{i}{j}", ld)
            o.location = (float(x), float(y), z)
            U.link(o, coll)
            out.append(o)
    return out


# --------------------------------------------------------------------------- #
# studio (vehicle reveal)
# --------------------------------------------------------------------------- #
STUDIO_CAM_AZ_DEG = -60.0      # render.plan.turntable_plan: the camera sits at this azimuth (the back wall opposite)
STUDIO_STRIP_W = float(os.environ.get("STUDIO_STRIP_W", "8.0"))     # rim strip power (W); env override: probes only


def _studio_light(coll: Any, name: str, loc: tuple, aim: tuple, shape: str, size: tuple, energy: float,
                  spread_deg: float, col: tuple = (1.0, 1.0, 1.0)) -> Any:
    from mathutils import Vector
    ld = bpy.data.lights.new(name, "AREA")
    ld.shape = shape
    ld.size = size[0]
    if shape in ("RECTANGLE", "ELLIPSE"):
        ld.size_y = size[1]
    ld.energy = energy
    ld.color = col
    if hasattr(ld, "spread"):
        ld.spread = math.radians(spread_deg)          # honeycomb grid: light only where it is aimed
    o = bpy.data.objects.new("studio_" + name, ld)
    o.location = loc
    o.rotation_mode = "QUATERNION"
    o.rotation_quaternion = (Vector(aim) - Vector(loc)).to_track_quat("-Z", "Y")
    U.link(o, coll)
    return o


def build_studio(asset_dir: str, ground_z: float, coll: Any, warnings: list[str]) -> dict[str, Any]:
    """Product-reveal studio: a near-black seamless cyclorama, a gridded key softbox, two strip lights behind the
    vehicle that draw its outline on the matte black body (the classic black-product look), a soft top light pooling
    on a glossy black turntable, a faint glow on the back wall; the studio HDRI only for reflections (0.12)."""
    hdr = os.path.join(asset_dir, "studio_small_09", "studio_small_09_2k.hdr")
    if os.path.exists(hdr):
        world_hdri(hdr, strength=0.12)
    else:
        warnings.append("studio HDRI missing")
    m, nb, out = U.new_material("cyclorama")
    p = U.principled(nb, Base_Color=(0.030, 0.031, 0.034, 1.0), Roughness=0.62)
    nb.l(p.outputs[0], out.inputs["Surface"])
    prof = []
    R, H, L = 3.0, 7.0, 14.0
    prof.append((-L, ground_z))
    for k in range(0, 17):
        a = math.radians(90 * k / 16)
        prof.append((4.0 + R * math.sin(a), ground_z + R * (1 - math.cos(a))))
    prof.append((4.0 + R, ground_z + H))
    xs = [x for x, _ in prof]
    zs = [z for _, z in prof]
    ys = np.linspace(-16, 16, 2)
    verts = [[x, y, z] for y in ys for x, z in zip(xs, zs)]
    nprof = len(prof)
    faces = [[i, i + 1, nprof + i + 1, nprof + i] for i in range(nprof - 1)]
    cyc = U.mesh_object("cyclorama", np.array(verts), faces, coll, smooth_angle_deg=60.0, mat=m)
    back = math.radians(STUDIO_CAM_AZ_DEG + 180.0)
    cyc.rotation_euler[2] = back                        # the back wall (+x of the profile) opposite the camera
    c = (0.0, 0.0, 0.42)

    def at(az_deg: float, r: float, z: float) -> tuple:
        a = back + math.radians(az_deg)
        return (r * math.cos(a), r * math.sin(a), z)
    # Black-product lighting (art review round 1: the key made the 4 % body read grey, the strips pooled on the floor
    # and missed the body): the key at a third, two tall strips BEHIND the vehicle (+-50 deg off the camera's back
    # axis = +-130 deg from the camera) light-linked to the vehicle + pad only (main.py) so they draw its silhouette
    # as soft rim lines and never touch the floor, a soft overhead box for a gradient down the nose, a dim cool fill,
    # and a glow linked to the cyclorama only (a soft backdrop gradient).
    lights = [
        # (round 1b, measured on turntable r1 frame 202: lit body ~25 sRGB, rim lines ~126 -> the key / fill / strips
        #  x1.6-1.8 for the 35-55 body / 150-200 rim targets)
        _studio_light(coll, "key", at(-115.0, 2.6, 2.1), c, "RECTANGLE", (1.2, 0.9), 4.5, 55.0, (1.0, 0.96, 0.9)),
        # round 2: the strips at +-50 deg off the back axis lit a broad sheen over a third of the body (a rough 0.87
        # dielectric shows a wide lobe): now 7 cm wide, gridded to 15 deg and only 22 deg off the back axis, so only the
        # silhouette catches them (thin rim lines), ~1.7 EV less power
        _studio_light(coll, "strip_l", at(-22.0, 1.7, 0.85), c, "RECTANGLE", (0.07, 1.8), STUDIO_STRIP_W, 15.0),
        _studio_light(coll, "strip_r", at(22.0, 1.7, 0.85), c, "RECTANGLE", (0.07, 1.8), STUDIO_STRIP_W, 15.0),
        # the top box ungridded (a 60 deg grid drew a hard-edged light pool on the floor / cove)
        _studio_light(coll, "top", (0.0, 0.0, 2.4), (0.0, 0.0, 0.0), "RECTANGLE", (0.9, 0.9), 4.0, 180.0),
        _studio_light(coll, "fill", at(150.0, 3.0, 1.2), c, "RECTANGLE", (1.5, 1.0), 2.5, 80.0, (0.86, 0.92, 1.0)),
        _studio_light(coll, "wall_glow", at(0.0, 1.4, 0.15), at(0.0, 6.0, 1.6), "DISK", (0.6, 0.6), 18.0, 120.0,
                      (0.82, 0.88, 1.0)),
    ]
    tm, nb, out = U.new_material("turntable")
    p = U.principled(nb, Base_Color=(0.012, 0.012, 0.014, 1.0), Roughness=0.18, Coat_Weight=0.5, Coat_Roughness=0.05)
    nb.l(p.outputs[0], out.inputs["Surface"])
    a = np.linspace(0, 2 * math.pi, 128, endpoint=False)
    rr = 0.42
    v = np.concatenate([np.column_stack([rr * np.cos(a), rr * np.sin(a), np.full(a.size, z)]) for z in (ground_z, ground_z + 0.03)])
    nseg = a.size
    f = [list(range(nseg))[::-1], list(range(nseg, 2 * nseg))] + [[i, (i + 1) % nseg, nseg + (i + 1) % nseg, nseg + i] for i in range(nseg)]
    tt = U.mesh_object("turntable", v, f, coll, smooth_angle_deg=40.0, mat=tm)
    bev = tt.modifiers.new("bevel", "BEVEL")
    bev.width = 0.004
    bev.segments = 3
    return {"cyc": cyc, "lights": lights, "turntable": tt, "soil_z": ground_z,
            "link": {"vehicle_only": [lights[1], lights[2]], "cyc_only": [lights[5]]}}


def link_lights(lights: list[Any], objs: list[Any], name: str) -> bool:
    """Light linking (Cycles): ``lights`` illuminate only ``objs``.  False when this Blender has no light linking."""
    rc = bpy.data.collections.new(name)
    for o in objs:
        if o is not None and o.type == "MESH" and o.name not in rc.objects:
            rc.objects.link(o)
    ok = True
    for lo in lights:
        if hasattr(lo, "light_linking"):
            lo.light_linking.receiver_collection = rc
        else:
            ok = False
    return ok
