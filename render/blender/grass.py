"""Instanced lawn (geometry nodes) that bends under the exhaust jet and sways with the logged wind.

Tufts of grass blades are instanced on a world-anchored jittered grid around the active camera in three
densities (3.5 cm within 5 m, 7.5 cm to 12 m, 18 cm to 30 m; bigger tufts further out: the blade density per
square metre is the same up to 12 m and a third beyond; the last ring thins out over its outer 35 % so the CG
lawn feathers into the photographed meadow), cross-faded at the ring borders and only inside a 120 deg cone in front
of the camera.  Every random number comes from the integer grid cell of the point (not its index), so a
tuft keeps its place, size and yaw when the camera moves.  Each tuft tilts:

  jet:  radially away from the impact point by k = 1.3 (1 - exp(-max(u - 1, 0) / 4.5)) rad (grass is stiff: a wall
        jet under ~1 m/s does not visibly bend it - round 1 laid the whole field down radially, which read as a
        zoom-blur smear in the landing shot), u = the radial wall-jet speed
        min(u_ground, K / max(r, b)) (render.plan.jet_state / wall_jet_speed; r = distance to the impact point),
        with a turbulent flutter of +-0.35 k (eddies travelling outward at 6.5 Hz / 0.35 m + an 11 Hz buffet); the lean is
        stored per tuft ('bend') and the blade shader pales / glosses flattened grass (the downwash ring)
  wind: toward the air's motion by 0.025 |w| + 0.02 |w| sin(2 pi (0.8 t - d / 2.5) + phase)
        (d = distance along the wind: gusts travel across the lawn as waves)

Inputs come from empties keyed per frame: ``fx_jet`` (location = impact point, scale = (u_ground, b, 1)),
``fx_wind`` (location = (w_x, w_y, 0)), ``fx_time`` (location = (sim time, cos of the camera cull half-angle, 0)).
Illustration, not CFD.

Seam with the photographed meadow (art review: a hard horizontal line where the CG lawn met the HDRI ground): for
CAMERA rays the blades fade into the HDRI itself sampled along the view ray - the same projection the ground plane
uses for its far field (env.ground_material_outdoor) - between ``FAR_BLEND`` metres, with the same position noise
feathering the boundary, so the CG lawn has no edge of its own.  The near blade colour is graded to the meadow's
measured value and saturation (``MEADOW_GAIN``).
"""
from __future__ import annotations

import math
from typing import Any

import bpy
import numpy as np

from . import bl_util as U

LODS = ((0.035, 0.0, 5.0), (0.075, 5.0, 12.0), (0.18, 12.0, 30.0))
LOD_TUFTS = ((7, 0.012, 3), (28, 0.034, 2), (60, 0.085, 2))  # blades per tuft, tuft radius, segments per blade:
#   LOD1 matches the LOD0 blade density (no visible ring 5 m around an elevated camera); LOD2 a third of it
DITHER = 0.18                                            # LOD cross-fade: +-18 % of the ring radius
FAR_BLEND = (9.0, 22.0)          # camera distance (m) over which blades fade into the HDRI ground (camera rays only)
FEATHER_M = 14.0                 # +-7 m position-noise feather of that boundary (same noise as the ground's far field)
# linear RGB gain of the blade colour: CG lawn measured sRGB ~97/96/66 vs the photographed meadow ~111/112/66 in still
# 02 r1 (same light, same distance band) -> x(1.33, 1.38, 1.0); slightly less, the near field is in full sun
MEADOW_GAIN = (1.22, 1.30, 0.94)
JET_U_MIN = 1.0                  # m/s: wall-jet speed below which the grass does not visibly bend


class GN:
    def __init__(self, name: str):
        self.ng = bpy.data.node_groups.new(name, "GeometryNodeTree")
        self.ng.interface.new_socket("Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
        self.ng.interface.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
        self.inp = self.n("NodeGroupInput")
        self.out = self.n("NodeGroupOutput")

    def n(self, kind: str, inputs: dict | None = None, **props: Any) -> Any:
        nd = self.ng.nodes.new(kind)
        for k, v in props.items():
            setattr(nd, k, v)
        for k, v in (inputs or {}).items():
            nd.inputs[k].default_value = v
        return nd

    def l(self, a: Any, b: Any) -> None:
        self.ng.links.new(a, b)

    def _set(self, sock: Any, v: Any) -> None:
        if isinstance(v, (int, float, tuple)):
            sock.default_value = v
        else:
            self.l(v, sock)

    def math(self, op: str, a: Any, b: Any = None, c: Any = None) -> Any:
        nd = self.n("ShaderNodeMath", operation=op)
        for i, v in enumerate((a, b, c)):
            if v is not None:
                self._set(nd.inputs[i], float(v) if isinstance(v, (int, float)) else v)
        return nd.outputs[0]

    def vmath(self, op: str, a: Any, b: Any = None, scale: Any = None) -> Any:
        nd = self.n("ShaderNodeVectorMath", operation=op)
        for i, v in enumerate((a, b)):
            if v is not None:
                self._set(nd.inputs[i], v)
        if scale is not None:
            self._set(nd.inputs["Scale"], float(scale) if isinstance(scale, (int, float)) else scale)
        return nd.outputs["Value"] if op in ("LENGTH", "DOT_PRODUCT", "DISTANCE") else nd.outputs["Vector"]

    def sep(self, v: Any) -> Any:
        nd = self.n("ShaderNodeSeparateXYZ")
        self.l(v, nd.inputs[0])
        return nd.outputs

    def comb(self, x: Any, y: Any, z: Any) -> Any:
        nd = self.n("ShaderNodeCombineXYZ")
        for i, v in enumerate((x, y, z)):
            self._set(nd.inputs[i], float(v) if isinstance(v, (int, float)) else v)
        return nd.outputs[0]

    def rand_float(self, lo: float, hi: float, seed: int) -> Any:
        """Random Value (FLOAT); Min / Max / output looked up by name and socket type (multi-type node)."""
        nd = self.n("FunctionNodeRandomValue", data_type="FLOAT")
        [x for x in nd.inputs if x.name == "Min" and x.type == "VALUE"][0].default_value = lo
        [x for x in nd.inputs if x.name == "Max" and x.type == "VALUE"][0].default_value = hi
        nd.inputs["Seed"].default_value = seed
        return nd

    @staticmethod
    def fout(nd: Any) -> Any:
        return [x for x in nd.outputs if x.type == "VALUE"][0]

    def obj(self, o: Any) -> Any:
        nd = self.n("GeometryNodeObjectInfo", transform_space="ORIGINAL")
        nd.inputs["Object"].default_value = o
        return nd


def tuft_mesh(blades: int = 7, radius: float = 0.012, seed: int = 3, segs: int = 3) -> tuple[np.ndarray, list[list[int]]]:
    """A tuft of ``blades`` grass blades (3-segment tapered strips, 4.5 - 8.5 cm, curved outward) within ``radius``."""
    rng = np.random.default_rng(seed)
    verts: list[list[float]] = []
    faces: list[list[int]] = []
    for _ in range(blades):
        a = rng.uniform(0, 2 * math.pi)
        r0 = radius * math.sqrt(rng.uniform(0.0, 1.0))
        x0, y0 = r0 * math.cos(a), r0 * math.sin(a)
        h = rng.uniform(0.045, 0.085)
        lean = rng.uniform(0.15, 0.55)
        la = rng.uniform(0, 2 * math.pi)
        w0 = rng.uniform(0.0022, 0.0035)
        face_a = rng.uniform(0, math.pi)
        base = len(verts)
        for s in range(segs + 1):
            u = s / segs
            cx = x0 + math.cos(la) * lean * h * u * u
            cy = y0 + math.sin(la) * lean * h * u * u
            cz = h * u * (1.0 - 0.25 * lean * u)
            w = w0 * (1.0 - u) + 0.0002
            dx, dy = math.cos(face_a) * w, math.sin(face_a) * w
            verts.append([cx - dx, cy - dy, cz])
            verts.append([cx + dx, cy + dy, cz])
        for s in range(segs):
            i = base + 2 * s
            faces.append([i, i + 1, i + 3, i + 2])
    return np.array(verts), faces


def blade_material(albedo: tuple[float, float, float] | None = None, env_img: Any = None, rot_z: float = 0.0) -> Any:
    """Two-sided blades: base -> tip ramp scaled from the ground albedo implied by the HDRI (colour match with
    the photographed field), ~7 % dry blades, per-instance hue jitter, 25 % translucency; for camera rays beyond
    FAR_BLEND[0] the blades fade into the HDRI ground along the view ray (``env_img`` = the sun-free HDRI, rotated
    by ``rot_z`` like the world)."""
    alb = np.clip(np.asarray(albedo if albedo is not None else (0.09, 0.14, 0.035), dtype=float), 0.01, 0.5)
    m, nb, out = U.new_material("grass_blades")
    oi = nb.n("ShaderNodeObjectInfo")
    tc = nb.n("ShaderNodeTexCoord")
    sep = nb.n("ShaderNodeSeparateXYZ")
    nb.l(tc.outputs["Object"], sep.inputs[0])
    tip = nb.n("ShaderNodeMapRange", inputs={1: 0.0, 2: 0.08, 3: 0.0, 4: 1.0})
    nb.l(sep.outputs[2], tip.inputs[0])
    ramp = nb.n("ShaderNodeValToRGB")
    cr = ramp.color_ramp
    c0, c1, c2 = alb * 0.45, alb * 1.25, np.clip(alb * 1.75, 0, 1)
    cr.elements[0].color = (float(c0[0]), float(c0[1]), float(c0[2]), 1.0)
    cr.elements[1].color = (float(c2[0]), float(c2[1]), float(c2[2]), 1.0)
    e = cr.elements.new(0.5)
    e.color = (float(c1[0]), float(c1[1]), float(c1[2]), 1.0)
    nb.l(tip.outputs[0], ramp.inputs[0])
    dry = nb.n("ShaderNodeMapRange", inputs={1: 0.90, 2: 0.97, 3: 0.0, 4: 1.0})
    dry.clamp = True
    nb.l(oi.outputs["Random"], dry.inputs[0])
    col = nb.mix_rgb(dry.outputs[0], ramp.outputs[0], (0.26, 0.22, 0.10, 1.0))
    # graded toward the photographed meadow (round 1: the CG lawn read darker and yellower than the HDRI field)
    # (measured on still 02 r1: CG lawn sRGB ~97/96/67 vs the photographed meadow 116/115/73 -> x1.3 linear)
    hsv = nb.n("ShaderNodeHueSaturation", inputs={"Saturation": 1.0, "Value": 1.45, "Fac": 1.0})
    nb.l(nb.math("ADD", nb.math("MULTIPLY", oi.outputs["Random"], 0.04), 0.488), hsv.inputs["Hue"])
    nb.l(col, hsv.inputs["Color"])
    # value / saturation of the photographed meadow (MEADOW_GAIN, measured)
    graded = nb.mix_rgb(1.0, hsv.outputs["Color"], tuple(MEADOW_GAIN) + (1.0,), blend="MULTIPLY")
    # grass flattened by the jet (instance attribute 'bend' = lean, rad) turns its paler, glossier blade sides to
    # the sky: the classic lighter downwash ring around a hovering rotor / fan (milder than round 1, which read as
    # bleached straw)
    bend = nb.n("ShaderNodeAttribute", attribute_type="INSTANCER", attribute_name="bend")
    bw = nb.n("ShaderNodeMapRange", inputs={1: 0.15, 2: 1.0, 3: 0.0, 4: 1.0})
    bw.clamp = True
    nb.l(bend.outputs["Fac"], bw.inputs[0])
    pale = nb.n("ShaderNodeHueSaturation", inputs={"Saturation": 0.85, "Value": 1.30, "Fac": 1.0})
    nb.l(graded, pale.inputs["Color"])
    colb = nb.mix_rgb(nb.math("MULTIPLY", bw.outputs[0], 0.65), graded, pale.outputs["Color"])
    p = U.principled(nb, Specular_IOR_Level=0.12)
    nb.l(nb.math("SUBTRACT", 0.72, nb.math("MULTIPLY", bw.outputs[0], 0.30)), p.inputs["Roughness"])
    nb.l(nb.math("ADD", 0.12, nb.math("MULTIPLY", bw.outputs[0], 0.35)), p.inputs["Specular IOR Level"])
    nb.l(colb, p.inputs["Base Color"])
    tl = nb.n("ShaderNodeBsdfTranslucent")
    nb.l(colb, tl.inputs["Color"])
    mix = nb.n("ShaderNodeMixShader", inputs={0: 0.32})
    nb.l(p.outputs[0], mix.inputs[1])
    nb.l(tl.outputs[0], mix.inputs[2])
    surf = mix.outputs[0]
    if env_img is not None:
        # far field: the photographed meadow along the camera ray (exactly what the ground plane shows there)
        geo = nb.n("ShaderNodeNewGeometry")
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
        fn = nb.n("ShaderNodeTexNoise", inputs={"Scale": 0.09, "Detail": 3.0, "Roughness": 0.6})
        nb.l(geo.outputs["Position"], fn.inputs["Vector"])
        vd = nb.math("ADD", cam.outputs["View Distance"], nb.math("MULTIPLY", nb.math("SUBTRACT", fn.outputs["Fac"], 0.5),
                                                                   FEATHER_M))
        far = nb.n("ShaderNodeMapRange", inputs={1: FAR_BLEND[0], 2: FAR_BLEND[1], 3: 0.0, 4: 1.0})
        far.clamp = True
        far.interpolation_type = "SMOOTHSTEP"
        nb.l(vd, far.inputs[0])
        lp = nb.n("ShaderNodeLightPath")
        mx2 = nb.n("ShaderNodeMixShader")
        nb.l(nb.math("MULTIPLY", far.outputs[0], lp.outputs["Is Camera Ray"]), mx2.inputs[0])
        nb.l(surf, mx2.inputs[1])
        nb.l(em.outputs[0], mx2.inputs[2])
        surf = mx2.outputs[0]
    nb.l(surf, out.inputs["Surface"])
    return m


def build_grass(coll: Any, soil_z: float, jet: Any, wind: Any, ftime: Any, exclude_half: float = 1.15,
                quality: str = "final", albedo: tuple[float, float, float] | None = None, env_img: Any = None,
                rot_z: float = 0.0) -> Any:
    hidden = U.collection("_instances_hidden")
    hidden.hide_render = True
    hidden.hide_viewport = True
    mat = blade_material(albedo, env_img, rot_z)
    tufts = []
    for li, (nbl, rad, segs) in enumerate(LOD_TUFTS):
        v, f = tuft_mesh(nbl, rad, seed=3 + li, segs=segs)
        t = U.mesh_object(f"grass_tuft_{li}", v, f, hidden, smooth_angle_deg=None)
        t.data.polygons.foreach_set("use_smooth", np.ones(len(t.data.polygons), dtype=bool))
        t.data.materials.append(mat)
        if li == 2:
            t.visible_shadow = False            # far tufts cast no shadow (EEVEE shadow-map acne at a grazing sun)
        tufts.append(t)
    g = GN("grass_field")
    cam = g.n("GeometryNodeInputActiveCamera")
    cinfo = g.n("GeometryNodeObjectInfo", transform_space="ORIGINAL")
    g.l(cam.outputs[0], cinfo.inputs["Object"])
    jinfo, winfo, tinfo = g.obj(jet), g.obj(wind), g.obj(ftime)
    cpos = cinfo.outputs["Location"]
    fwd_n = g.n("FunctionNodeRotateVector")
    g._set(fwd_n.inputs["Vector"], (0.0, 0.0, -1.0))
    g.l(cinfo.outputs["Rotation"], fwd_n.inputs["Rotation"])
    fwd_h = g.vmath("NORMALIZE", g.vmath("MULTIPLY", fwd_n.outputs[0], (1.0, 1.0, 0.0)))
    # ---- bending fields (evaluated on each LOD's points) ----
    P = g.n("GeometryNodeInputPosition").outputs[0]
    cell = g.n("GeometryNodeInputNamedAttribute", data_type="INT")
    cell.inputs["Name"].default_value = "cell"
    cid = cell.outputs["Attribute"]
    yaw, hs, ph = g.rand_float(0.0, 6.2832, 5), g.rand_float(0.75, 1.3, 6), g.rand_float(0.0, 6.2832, 8)
    for nd in (yaw, hs, ph):
        g.l(cid, nd.inputs["ID"])
    jl = jinfo.outputs["Location"]
    js = g.sep(jinfo.outputs["Scale"])           # (u_ground, b, K)
    rel = g.vmath("MULTIPLY", g.vmath("SUBTRACT", P, jl), (1.0, 1.0, 0.0))
    r = g.math("MAXIMUM", g.vmath("LENGTH", rel), 0.01)
    radial = g.vmath("NORMALIZE", rel)
    # radial wall-jet speed at the tuft: u = min(u_ground, K / max(r, b))  (render.plan.wall_jet_speed)
    uw = g.math("MINIMUM", js[0], g.math("DIVIDE", js[2], g.math("MAXIMUM", r, js[1])))
    # lean 1.3 (1 - exp(-max(u - 1, 0) / 4.5)) rad: 0 below 1 m/s, 8 deg at 1.5 m/s, 27 at 3, 49 at 6, 67 at 12
    ue = g.math("MAXIMUM", g.math("SUBTRACT", uw, JET_U_MIN), 0.0)
    k = g.math("MULTIPLY", 1.30, g.math("SUBTRACT", 1.0, g.math("EXPONENT", g.math("DIVIDE", ue, -4.5))))
    tsep = g.sep(tinfo.outputs["Location"])
    tsim = tsep[0]
    cull_cos = tsep[1]          # cos(half horizontal FOV + 15 deg) of the active camera, keyed by anim.py
    # turbulent wall jet: +-35 % of the lean, eddies travelling outward (6.5 Hz, 0.35 m) + an 11 Hz buffet per tuft
    eddy = g.math("SINE", g.math("ADD", g.math("MULTIPLY", 6.2832, g.math("SUBTRACT", g.math("MULTIPLY", tsim, 6.5),
                                                                           g.math("DIVIDE", r, 0.35))), g.fout(ph)))
    buff = g.math("SINE", g.math("ADD", g.math("MULTIPLY", tsim, 71.0), g.math("MULTIPLY", g.fout(ph), 2.0)))
    flutter = g.math("MULTIPLY", g.math("MULTIPLY", k, 0.35),
                     g.math("ADD", g.math("MULTIPLY", eddy, 0.6), g.math("MULTIPLY", buff, 0.4)))
    jet_vec = g.vmath("SCALE", radial, None,
                      scale=g.math("TANGENT", g.math("MINIMUM", g.math("MAXIMUM", g.math("ADD", k, flutter), 0.0), 1.3)))
    wh = g.vmath("MULTIPLY", winfo.outputs["Location"], (1.0, 1.0, 0.0))
    wsp = g.vmath("LENGTH", wh)
    wdir = g.vmath("NORMALIZE", wh)
    along = g.vmath("DOT_PRODUCT", P, wdir)
    wave = g.math("SINE", g.math("ADD", g.math("MULTIPLY", 6.2832, g.math("SUBTRACT", g.math("MULTIPLY", 0.8, tsim),
                                                                               g.math("DIVIDE", along, 2.5))), g.fout(ph)))
    sway = g.math("ADD", g.math("MULTIPLY", 0.025, wsp), g.math("MULTIPLY", g.math("MULTIPLY", 0.02, wsp), wave))
    up = g.vmath("ADD", g.vmath("ADD", (0.0, 0.0, 1.0), jet_vec), g.vmath("SCALE", wdir, None, scale=sway))
    rot0 = g.n("FunctionNodeEulerToRotation")
    g.l(g.comb(0.0, 0.0, g.fout(yaw)), rot0.inputs[0])
    align = g.n("FunctionNodeAlignRotationToVector", axis="Z", pivot_axis="AUTO")
    g.l(rot0.outputs[0], align.inputs["Rotation"])
    g.l(up, align.inputs["Vector"])
    hsv = g.fout(hs)
    # trimmed turf along the slab (art review round 2: a hard vertical wall of full-height blades at the slab edge):
    # Chebyshev distance outside the slab d; blades within 14 cm are cut down to 55 % height, tufts may root up to
    # 2.5 cm under the paver edge (only their tips show, creeping over it)
    Pe = g.sep(g.n("GeometryNodeInputPosition").outputs[0])
    d_out = g.math("SUBTRACT", g.math("MAXIMUM", g.math("ABSOLUTE", Pe[0]), g.math("ABSOLUTE", Pe[1])), exclude_half)
    trim_t = g.math("DIVIDE", g.math("SUBTRACT", d_out, -0.02), 0.16)
    trim_t = g.math("MINIMUM", g.math("MAXIMUM", trim_t, 0.0), 1.0)
    sm = g.math("MULTIPLY", g.math("MULTIPLY", trim_t, trim_t), g.math("SUBTRACT", 3.0, g.math("MULTIPLY", 2.0, trim_t)))
    trim = g.math("ADD", 0.55, g.math("MULTIPLY", 0.45, sm))
    hz = g.math("MULTIPLY", hsv, trim)
    scale = g.comb(hsv, hsv, hz)
    lods = LODS if quality == "final" else tuple((s_ * 1.5, a, b) for s_, a, b in LODS)
    nl = len(lods)
    parts = []
    for li, (sp, r0, r1) in enumerate(lods):
        r1x = r1 * (1 + DITHER) if li < nl - 1 else r1
        r0x = r0 * (1 - DITHER)
        nverts = int(math.ceil(2 * r1x / sp)) + 1
        grid = g.n("GeometryNodeMeshGrid", inputs={"Size X": (nverts - 1) * sp, "Size Y": (nverts - 1) * sp,
                                                    "Vertices X": nverts, "Vertices Y": nverts})
        pts = g.n("GeometryNodeMeshToPoints")
        g.l(grid.outputs["Mesh"], pts.inputs["Mesh"])
        snap = g.vmath("SNAP", g.vmath("MULTIPLY", cpos, (1.0, 1.0, 0.0)), (sp, sp, 1.0))
        tr = g.n("GeometryNodeTransform")
        g.l(pts.outputs["Points"], tr.inputs["Geometry"])
        g.l(snap, tr.inputs["Translation"])
        # integer cell id from the world position (stable while the grid follows the camera)
        Pc = g.sep(g.n("GeometryNodeInputPosition").outputs[0])
        ix = g.n("FunctionNodeFloatToInt", rounding_mode="ROUND")
        g.l(g.math("DIVIDE", Pc[0], sp), ix.inputs[0])
        iy = g.n("FunctionNodeFloatToInt", rounding_mode="ROUND")
        g.l(g.math("DIVIDE", Pc[1], sp), iy.inputs[0])
        idn = g.n("FunctionNodeIntegerMath", operation="MULTIPLY_ADD")
        g.l(ix.outputs[0], idn.inputs[0])
        idn.inputs[1].default_value = 92821
        g.l(iy.outputs[0], idn.inputs[2])
        idn2 = g.n("FunctionNodeIntegerMath", operation="ADD")
        g.l(idn.outputs[0], idn2.inputs[0])
        idn2.inputs[1].default_value = 7919 * (li + 1)
        store = g.n("GeometryNodeStoreNamedAttribute", data_type="INT", domain="POINT")
        store.inputs["Name"].default_value = "cell"
        g.l(tr.outputs["Geometry"], store.inputs["Geometry"])
        g.l(idn2.outputs[0], store.inputs["Value"])
        jit = g.n("FunctionNodeRandomValue", data_type="FLOAT_VECTOR")
        jit.inputs["Min"].default_value = (-0.5 * sp, -0.5 * sp, 0.0)
        jit.inputs["Max"].default_value = (0.5 * sp, 0.5 * sp, 0.0)
        jit.inputs["Seed"].default_value = 11 + li
        g.l(idn2.outputs[0], jit.inputs["ID"])
        setp = g.n("GeometryNodeSetPosition")
        g.l(store.outputs["Geometry"], setp.inputs["Geometry"])
        g.l(g.vmath("ADD", jit.outputs["Value"], (0.0, 0.0, soil_z)), setp.inputs["Offset"])
        # keep probability: 1 inside the ring, a linear cross-fade over the dither bands, 0 outside
        P2 = g.n("GeometryNodeInputPosition").outputs[0]
        d = g.vmath("MULTIPLY", g.vmath("SUBTRACT", P2, cpos), (1.0, 1.0, 0.0))
        dist = g.vmath("LENGTH", d)
        cosang = g.vmath("DOT_PRODUCT", g.vmath("NORMALIZE", d), fwd_h)
        if li < nl - 1:
            keep_out = g.math("DIVIDE", g.math("SUBTRACT", r1x, dist), r1x - r1 * (1 - DITHER))
        else:
            # the far edge thins out over the last 35 % (no hard line where the HDRI meadow takes over)
            keep_out = g.math("DIVIDE", g.math("SUBTRACT", r1, dist), 0.35 * r1)
        if li > 0:
            keep_in = g.math("DIVIDE", g.math("SUBTRACT", dist, r0x), r0 * (1 + DITHER) - r0x)
        else:
            keep_in = g.math("ADD", 1.0, 0.0)
        keep = g.math("MINIMUM", g.math("MINIMUM", keep_out, keep_in), 1.0)
        rnd = g.rand_float(0.0, 1.0, 21 + li)
        g.l(idn2.outputs[0], rnd.inputs["ID"])
        drop = g.math("GREATER_THAN", g.fout(rnd), keep)
        behind = g.math("MULTIPLY", g.math("LESS_THAN", cosang, cull_cos), g.math("GREATER_THAN", dist, 1.2))
        sp_ = g.sep(P2)
        on_slab = g.math("MULTIPLY", g.math("LESS_THAN", g.math("ABSOLUTE", sp_[0]), exclude_half - 0.025),
                         g.math("LESS_THAN", g.math("ABSOLUTE", sp_[1]), exclude_half - 0.025))
        kill = g.math("MAXIMUM", g.math("MAXIMUM", drop, behind), on_slab)
        dele = g.n("GeometryNodeDeleteGeometry", domain="POINT")
        g.l(setp.outputs["Geometry"], dele.inputs["Geometry"])
        g.l(kill, dele.inputs["Selection"])
        ti = g.obj(tufts[li])
        ti.inputs["As Instance"].default_value = True
        inst = g.n("GeometryNodeInstanceOnPoints")
        # the jet lean as a point attribute -> the blade shader (flattened grass shows its lighter, sheenier sides)
        stb = g.n("GeometryNodeStoreNamedAttribute", data_type="FLOAT", domain="POINT")
        stb.inputs["Name"].default_value = "bend"
        g.l(dele.outputs["Geometry"], stb.inputs["Geometry"])
        g.l(k, stb.inputs["Value"])
        g.l(stb.outputs["Geometry"], inst.inputs["Points"])
        g.l(ti.outputs["Geometry"], inst.inputs["Instance"])
        g.l(align.outputs[0], inst.inputs["Rotation"])
        g.l(scale, inst.inputs["Scale"])
        parts.append(inst.outputs["Instances"])
    join = g.n("GeometryNodeJoinGeometry")
    for p in reversed(parts):
        g.l(p, join.inputs[0])
    g.l(join.outputs[0], g.out.inputs[0])
    me = bpy.data.meshes.new("grass_host")
    me.from_pydata([[0.0, 0.0, 0.0]], [], [])
    host = bpy.data.objects.new("grass_field", me)
    U.link(host, coll)
    mod = host.modifiers.new("grass", "NODES")
    mod.node_group = g.ng
    return host
