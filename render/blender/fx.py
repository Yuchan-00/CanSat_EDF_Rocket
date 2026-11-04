"""Exhaust effects (run inside Blender): a real EDF jet is cold, clean air - invisible - so it is shown only through
what it does where it reaches the ground (grass bending is in grass.py).

  wall jet  after impingement the jet spreads radially along the ground; its peak speed u(r) = min(u_ground,
            K / max(r, b)), K = sqrt(T / rho) (render.plan.jet_state / wall_jet_speed: the jet's momentum flux is
            conserved; checked against Poreh et al.'s impinging-jet correlation).
  debris    cut-grass clippings outdoors / dark grit on the indoor plywood, skating radially outward with low hops
            wherever u(r) exceeds each particle's threshold (``build_dust``).
  dust      a procedural volume (``dust_volume``): a cylinder at the jet's ground impact, radius
            R = b + clamp(0.035 u, 0.2, 0.9) m, height 0.08 + 0.25 clamp(u / 20, 0, 1) m, bottom cap 3 cm below the soil
            (a cap coplanar with the ground lit the ground inside it as a disc); density = dust x 9/m x ring(r) x
            exp(-6 z) x (0.25 + 0.75 n^1.5).  Only over the launch slab's sandy joints (main.py gates it): turf gives
            clippings, not dust; a clean hall floor gives nothing.
  (no heat-haze cone: an EDF exhaust is only a few kelvin above ambient; the round-0 cone read as a rod / streaks.
   ``build_haze`` is kept for reference and not called.)

Invariant: deterministic - every random number comes from a node seed and the particle id; time enters only
through the keyed ``fx_time`` empty (location.x = sim time).
"""
from __future__ import annotations

import math
from typing import Any

import bpy
import numpy as np

from . import bl_util as U
from .grass import GN

L_REF = 2.0          # haze cone mesh length (m); the object's z scale = jet length / L_REF


# --------------------------------------------------------------------------- #
# haze
# --------------------------------------------------------------------------- #
def haze_material() -> Any:
    m, nb, out = U.new_material("jet_haze")
    tc = nb.n("ShaderNodeTexCoord")
    t_attr = nb.n("ShaderNodeAttribute", attribute_type="OBJECT", attribute_name="haze_t")
    h_attr = nb.n("ShaderNodeAttribute", attribute_type="OBJECT", attribute_name="haze")
    # flowing noise: object coords advected down the jet (z decreases along the jet) + time as 4th dimension
    flow = nb.n("ShaderNodeVectorMath", operation="ADD")
    nb.l(tc.outputs["Object"], flow.inputs[0])
    comb = nb.n("ShaderNodeCombineXYZ")
    nb.l(nb.math("MULTIPLY", t_attr.outputs["Fac"], 9.0), comb.inputs[2])
    nb.l(comb.outputs[0], flow.inputs[1])
    noise = nb.n("ShaderNodeTexNoise", noise_dimensions="4D", inputs={"Scale": 9.0, "Detail": 3.0, "Roughness": 0.55})
    nb.l(flow.outputs[0], noise.inputs["Vector"])
    nb.l(nb.math("MULTIPLY", t_attr.outputs["Fac"], 3.0), noise.inputs["W"])
    bump = nb.n("ShaderNodeBump", inputs={"Strength": 0.3, "Distance": 0.02})
    nb.l(noise.outputs["Fac"], bump.inputs["Height"])
    refr = nb.n("ShaderNodeBsdfRefraction", inputs={"IOR": 1.0, "Roughness": 0.0})
    refr.inputs["Color"].default_value = (1.0, 1.0, 1.0, 1.0)
    nb.l(bump.outputs["Normal"], refr.inputs["Normal"])
    tr = nb.n("ShaderNodeBsdfTransparent")
    # axial fade: object z runs 0 (exit) .. -L_REF (end)
    sep = nb.n("ShaderNodeSeparateXYZ")
    nb.l(tc.outputs["Object"], sep.inputs[0])
    ax = nb.n("ShaderNodeMapRange", inputs={1: 0.0, 2: -L_REF, 3: 1.0, 4: 0.0})
    ax.clamp = True
    nb.l(sep.outputs[2], ax.inputs[0])
    lw = nb.n("ShaderNodeLayerWeight", inputs={"Blend": 0.5})
    edge = nb.math("SUBTRACT", 1.0, lw.outputs["Facing"])
    fac = nb.math("MULTIPLY", nb.math("MULTIPLY", nb.math("MULTIPLY", h_attr.outputs["Fac"], ax.outputs[0]), edge), 0.20, clamp=True)
    mix = nb.n("ShaderNodeMixShader")
    nb.l(fac, mix.inputs[0])
    nb.l(tr.outputs[0], mix.inputs[1])
    nb.l(refr.outputs[0], mix.inputs[2])
    nb.l(mix.outputs[0], out.inputs["Surface"])
    for attr, val in (("surface_render_method", "DITHERED"), ("use_raytrace_refraction", True),
                      ("use_screen_refraction", True), ("blend_method", "HASHED")):
        if hasattr(m, attr):
            try:
                setattr(m, attr, val)
            except (TypeError, AttributeError):
                pass
    return m


def build_haze(coll: Any, parent: Any, duct_r: float = 0.045, spread: float = 0.10) -> Any:
    n_a, n_z = 32, 14
    a = np.linspace(0, 2 * math.pi, n_a, endpoint=False)
    rows = []
    for k in range(n_z + 1):
        x = L_REF * (k / n_z) ** 1.3
        r = 0.92 * duct_r + spread * x
        rows.append(np.column_stack([r * np.cos(a), r * np.sin(a), np.full(n_a, -x - 0.004)]))
    v = np.concatenate(rows)
    f = [[k * n_a + i, k * n_a + (i + 1) % n_a, (k + 1) * n_a + (i + 1) % n_a, (k + 1) * n_a + i]
         for k in range(n_z) for i in range(n_a)]
    o = U.mesh_object("jet_haze", v, f, coll, parent=parent, smooth_angle_deg=80.0, mat=haze_material())
    o["haze"] = 0.0
    o["haze_t"] = 0.0
    for attr in ("visible_shadow", "visible_diffuse"):
        if hasattr(o, attr):
            setattr(o, attr, False)
    return o


# --------------------------------------------------------------------------- #
# dust / debris (geometry nodes)
# --------------------------------------------------------------------------- #
def _quad(name: str, size: float, coll: Any) -> Any:
    h = 0.5 * size
    v = np.array([[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]])
    o = U.mesh_object(name, v, [[0, 1, 2, 3]], coll, smooth_angle_deg=None)
    uv = o.data.uv_layers.new(name="UVMap")
    uv.data.foreach_set("uv", np.array([0, 0, 1, 0, 1, 1, 0, 1], dtype=np.float32))
    return o


def dust_material(color: tuple[float, float, float], opacity: float) -> Any:
    m, nb, out = U.new_material("jet_dust")
    uvn = nb.n("ShaderNodeUVMap")
    uvn.uv_map = "UVMap"
    a_attr = nb.n("ShaderNodeAttribute", attribute_type="INSTANCER", attribute_name="dust_a")
    s_attr = nb.n("ShaderNodeAttribute", attribute_type="INSTANCER", attribute_name="dust_s")
    cen = nb.n("ShaderNodeVectorMath", operation="SUBTRACT")
    nb.l(uvn.outputs["UV"], cen.inputs[0])
    cen.inputs[1].default_value = (0.5, 0.5, 0.0)
    ln = nb.n("ShaderNodeVectorMath", operation="LENGTH")
    nb.l(cen.outputs[0], ln.inputs[0])
    disc = nb.n("ShaderNodeMapRange", inputs={1: 0.0, 2: 0.5, 3: 1.0, 4: 0.0})
    disc.clamp = True
    disc.interpolation_type = "SMOOTHSTEP"
    nb.l(ln.outputs["Value"], disc.inputs[0])
    noise = nb.n("ShaderNodeTexNoise", noise_dimensions="4D", inputs={"Scale": 2.2, "Detail": 5.0, "Roughness": 0.65})
    nb.l(uvn.outputs["UV"], noise.inputs["Vector"])
    nb.l(nb.math("MULTIPLY", s_attr.outputs["Fac"], 50.0), noise.inputs["W"])
    wisp = nb.n("ShaderNodeMapRange", inputs={1: 0.30, 2: 0.85, 3: 0.0, 4: 1.0})
    wisp.clamp = True
    nb.l(noise.outputs["Fac"], wisp.inputs[0])
    alpha = nb.math("MULTIPLY", nb.math("MULTIPLY", disc.outputs[0], wisp.outputs[0]),
                    nb.math("MULTIPLY", a_attr.outputs["Fac"], opacity), clamp=True)
    p = U.principled(nb, Base_Color=(color[0], color[1], color[2], 1.0), Roughness=1.0, Specular_IOR_Level=0.0)
    tr = nb.n("ShaderNodeBsdfTransparent")
    mix = nb.n("ShaderNodeMixShader")
    nb.l(alpha, mix.inputs[0])
    nb.l(tr.outputs[0], mix.inputs[1])
    nb.l(p.outputs[0], mix.inputs[2])
    nb.l(mix.outputs[0], out.inputs["Surface"])
    for attr, val in (("surface_render_method", "DITHERED"), ("blend_method", "HASHED")):
        if hasattr(m, attr):
            try:
                setattr(m, attr, val)
            except (TypeError, AttributeError):
                pass
    return m


def dust_volume_material(indoor: bool) -> Any:
    m, nb, out = U.new_material("dust_volume")
    tc = nb.n("ShaderNodeTexCoord")
    d_attr = nb.n("ShaderNodeAttribute", attribute_type="OBJECT", attribute_name="dust")
    t_attr = nb.n("ShaderNodeAttribute", attribute_type="OBJECT", attribute_name="dust_t")
    sep = nb.n("ShaderNodeSeparateXYZ")
    nb.l(tc.outputs["Object"], sep.inputs[0])
    r = nb.math("SQRT", nb.math("ADD", nb.math("MULTIPLY", sep.outputs[0], sep.outputs[0]),
                                 nb.math("MULTIPLY", sep.outputs[1], sep.outputs[1])))
    rin = nb.n("ShaderNodeMapRange", inputs={1: 0.0, 2: 0.25, 3: 0.0, 4: 1.0})
    rin.clamp = True
    rin.interpolation_type = "SMOOTHSTEP"
    nb.l(r, rin.inputs[0])
    rout = nb.n("ShaderNodeMapRange", inputs={1: 0.25, 2: 0.80, 3: 1.0, 4: 0.0})
    rout.clamp = True
    rout.interpolation_type = "SMOOTHSTEP"
    nb.l(r, rout.inputs[0])
    ring = nb.math("MULTIPLY", rin.outputs[0], rout.outputs[0])
    top = nb.n("ShaderNodeMapRange", inputs={1: 0.75, 2: 1.0, 3: 1.0, 4: 0.0})
    top.clamp = True
    nb.l(sep.outputs[2], top.inputs[0])
    vert = nb.math("MULTIPLY", nb.math("EXPONENT", nb.math("MULTIPLY", sep.outputs[2], -6.0)), top.outputs[0])
    # outward advection: p - (x, y, 0) / r * t * 1.3, plus slow rise
    rr = nb.math("MAXIMUM", r, 0.05)
    adv = nb.n("ShaderNodeCombineXYZ")
    nb.l(nb.math("SUBTRACT", sep.outputs[0], nb.math("MULTIPLY", nb.math("DIVIDE", sep.outputs[0], rr), nb.math("MULTIPLY", t_attr.outputs["Fac"], 1.3))), adv.inputs[0])
    nb.l(nb.math("SUBTRACT", sep.outputs[1], nb.math("MULTIPLY", nb.math("DIVIDE", sep.outputs[1], rr), nb.math("MULTIPLY", t_attr.outputs["Fac"], 1.3))), adv.inputs[1])
    nb.l(nb.math("SUBTRACT", sep.outputs[2], nb.math("MULTIPLY", t_attr.outputs["Fac"], 0.4)), adv.inputs[2])
    noise = nb.n("ShaderNodeTexNoise", noise_dimensions="4D", inputs={"Scale": 3.6, "Detail": 6.0, "Roughness": 0.62})
    nb.l(adv.outputs[0], noise.inputs["Vector"])
    nb.l(nb.math("MULTIPLY", t_attr.outputs["Fac"], 0.5), noise.inputs["W"])
    nm = nb.n("ShaderNodeMapRange", inputs={1: 0.40, 2: 0.75, 3: 0.0, 4: 1.0})
    nm.clamp = True
    nb.l(noise.outputs["Fac"], nm.inputs[0])
    n2 = nb.math("ADD", 0.25, nb.math("MULTIPLY", nb.math("POWER", nm.outputs[0], 1.5), 0.75))   # haze floor + wisps
    k = 1.0 if indoor else 9.0          # (indoors the volume is not built: main.py)
    dens = nb.math("MULTIPLY", nb.math("MULTIPLY", nb.math("MULTIPLY", ring, vert), n2),
                   nb.math("MULTIPLY", d_attr.outputs["Fac"], k))
    pv = nb.n("ShaderNodeVolumePrincipled")
    col = (0.80, 0.78, 0.74) if indoor else (0.84, 0.77, 0.64)
    pv.inputs["Color"].default_value = (col[0], col[1], col[2], 1.0)
    if "Absorption Color" in pv.inputs:                       # light dust scatters, absorbs little
        pv.inputs["Absorption Color"].default_value = (0.78, 0.72, 0.62, 1.0)
    pv.inputs["Anisotropy"].default_value = 0.35
    nb.l(dens, pv.inputs["Density"])
    nb.l(pv.outputs[0], out.inputs["Volume"])
    return m


def debris_material(indoor: bool) -> Any:
    """Outdoors: cut-grass clippings (dark olive -> straw, a little translucent); indoors: dark brown grit.  Both
    matte and dark (round 1's light grey flecks read as white glitter / fireflies)."""
    m, nb, out = U.new_material("jet_debris")
    oi = nb.n("ShaderNodeObjectInfo")
    if indoor:
        base, rough = (0.030, 0.022, 0.016), 0.85
    else:
        # dry-ish clippings: lighter and yellower than the living turf so they read against it (round 1b: the dark
        # olive flecks vanished in the lawn)
        base, rough = (0.10, 0.10, 0.035), 0.7
    hsv = nb.n("ShaderNodeHueSaturation", inputs={"Saturation": 1.0, "Fac": 1.0})
    nb.l(nb.math("ADD", nb.math("MULTIPLY", oi.outputs["Random"], 0.06), 0.47), hsv.inputs["Hue"])
    nb.l(nb.math("ADD", nb.math("MULTIPLY", oi.outputs["Random"], 0.7), 0.6), hsv.inputs["Value"])
    hsv.inputs["Color"].default_value = (base[0], base[1], base[2], 1.0)
    p = U.principled(nb, Roughness=rough, Specular_IOR_Level=0.2)
    nb.l(hsv.outputs["Color"], p.inputs["Base Color"])
    if indoor:
        nb.l(p.outputs[0], out.inputs["Surface"])
        return m
    # green at the cut base -> straw toward the tip (object x along the blade), a fine vein texture
    tc = nb.n("ShaderNodeTexCoord")
    sx = nb.n("ShaderNodeSeparateXYZ")
    nb.l(tc.outputs["Object"], sx.inputs[0])
    ramp = nb.n("ShaderNodeMapRange", inputs={1: -0.5, 2: 0.5, 3: 0.0, 4: 1.0})
    nb.l(sx.outputs[0], ramp.inputs[0])
    vein = nb.n("ShaderNodeTexWave", inputs={"Scale": 14.0, "Distortion": 1.5})
    nb.l(tc.outputs["Object"], vein.inputs["Vector"])
    green = nb.mix_rgb(1.0, hsv.outputs["Color"], (0.55, 1.35, 0.55, 1.0), blend="MULTIPLY")
    col = nb.mix_rgb(ramp.outputs[0], green, hsv.outputs["Color"])
    col = nb.mix_rgb(nb.math("MULTIPLY", vein.outputs["Fac"], 0.25), col, (0.05, 0.06, 0.02, 1.0))
    nb.l(col, p.inputs["Base Color"])
    tl = nb.n("ShaderNodeBsdfTranslucent")
    nb.l(col, tl.inputs["Color"])
    mix = nb.n("ShaderNodeMixShader", inputs={0: 0.25})
    nb.l(p.outputs[0], mix.inputs[1])
    nb.l(tl.outputs[0], mix.inputs[2])
    nb.l(mix.outputs[0], out.inputs["Surface"])
    return m


def _rand(g: GN, lo: float, hi: float, seed: int, idv: Any) -> Any:
    nd = g.rand_float(lo, hi, seed)
    g.l(idv, nd.inputs["ID"])
    return g.fout(nd)


def build_dust_volume(coll: Any, indoor: bool = False) -> Any:
    """Unit cylinder (r 1, z 0..1); the caller keys location (impact), scale (R, R, H), ['dust'], ['dust_t']."""
    n = 40
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    v = np.concatenate([np.column_stack([np.cos(a), np.sin(a), np.full(n, z)]) for z in (0.0, 1.0)])
    f = [list(range(n))[::-1], list(range(n, 2 * n))] + [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    o = U.mesh_object("dust_volume", v, f, coll, smooth_angle_deg=None, mat=dust_volume_material(indoor))
    o["dust"] = 0.0
    o["dust_t"] = 0.0
    return o


def dust_volume_size(u_ground: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(R, H) of the dust skirt per frame (module docstring)."""
    u = np.asarray(u_ground, float)
    R = np.asarray(b, float) + np.clip(0.035 * u, 0.2, 0.9)
    H = 0.08 + 0.25 * np.clip(u / 20.0, 0.0, 1.0)
    return R, H


def build_dust(coll: Any, soil_z: float, jet: Any, ftime: Any, indoor: bool = False, count: int = 0,
               debris: int = 160) -> Any:
    """Debris blown along the ground by the radial wall jet (geometry nodes on the keyed ``fx_jet`` empty: location =
    impact point, scale = (u_ground, b, K)).  Each particle id has a home radius r0 (b .. 1.6 m outdoors, .. 0.6 m on the
    indoor plywood board), a speed threshold (3-7 m/s clippings, 4-9 m/s grit) and a life of 0.5-1.0 s; it exists
    only while the wall-jet speed at r0, u0 = min(u_ground, K / max(r0, b)), exceeds its threshold, then skates
    radially outward at min(0.45 u0, 4 m/s) with hops (clippings up to 20 cm, grit ~5 mm) and is removed at the end of its life
    (a new one starts: statistically steady while the jet blows).  ``count`` is ignored (the old dust billboards)."""
    hidden = U.collection("_instances_hidden")
    hidden.hide_render = True
    hidden.hide_viewport = True
    if indoor:
        piece = _quad("debris_grit", 1.0, hidden)
    else:
        # a cut grass blade: 1.0 long, 0.11 -> 0.02 wide, curled (art review round 2: the flat 1 x 0.28 card rendered
        # as a grey square); instances scale it to 2-4.5 cm
        nseg = 5
        v, f = [], []
        for i in range(nseg + 1):
            u = i / nseg
            x = u - 0.5
            w = 0.5 * (0.11 - 0.09 * u)
            z = 0.10 * math.sin(math.pi * u) + 0.06 * u * u
            yo = 0.03 * math.sin(2.0 * math.pi * u)
            v += [[x, yo - w, z], [x, yo + w, z + 0.01]]
        for i in range(nseg):
            f.append([2 * i, 2 * i + 2, 2 * i + 3, 2 * i + 1])
        piece = U.mesh_object("debris_clipping", np.array(v), f, hidden, smooth_angle_deg=60.0)
    piece.data.materials.append(debris_material(indoor))
    g = GN("jet_debris")
    jinfo, tinfo = g.obj(jet), g.obj(ftime)
    js = g.sep(jinfo.outputs["Scale"])
    u_g, b, K = js[0], js[1], js[2]
    tsim = g.sep(tinfo.outputs["Location"])[0]
    n, seed0 = debris, 300
    pts0 = g.n("GeometryNodePoints", inputs={"Count": n})
    sid = g.n("GeometryNodeStoreNamedAttribute", data_type="INT", domain="POINT")
    sid.inputs["Name"].default_value = "id"
    g.l(pts0.outputs[0], sid.inputs["Geometry"])
    g.l(g.n("GeometryNodeInputIndex").outputs[0], sid.inputs["Value"])
    idx = g.n("GeometryNodeInputID").outputs[0]
    life = _rand(g, 0.5, 1.0, seed0 + 1, idx)
    ph = _rand(g, 0.0, 1.0, seed0 + 2, idx)
    age = g.math("FRACT", g.math("ADD", g.math("DIVIDE", tsim, life), ph))
    tau = g.math("MULTIPLY", age, life)
    ang = _rand(g, 0.0, 6.2832, seed0 + 3, idx)
    dirv = g.comb(g.math("COSINE", ang), g.math("SINE", ang), 0.0)
    r_hi = 0.6 if indoor else 1.6
    # home radius: uniform in area between b and r_hi
    rr = _rand(g, 0.0, 1.0, seed0 + 4, idx)
    r0 = g.math("SQRT", g.math("ADD", g.math("MULTIPLY", b, b),
                               g.math("MULTIPLY", rr, g.math("SUBTRACT", r_hi * r_hi, g.math("MULTIPLY", b, b)))))
    u0 = g.math("MINIMUM", u_g, g.math("DIVIDE", K, g.math("MAXIMUM", r0, b)))
    thr = _rand(g, 4.0, 9.0, seed0 + 5, idx) if indoor else _rand(g, 3.0, 7.0, seed0 + 5, idx)
    active = g.math("GREATER_THAN", u0, thr)
    # skating speed 0.45 u0, at most 4 m/s (drag on the turf / board; round 1b: uncapped, the clippings left the
    # frame within a tenth of a second)
    travel = g.math("MULTIPLY", g.math("MINIMUM", g.math("MULTIPLY", u0, 0.45), 4.0), tau)
    r = g.math("ADD", r0, travel)
    t_hop = _rand(g, 0.12, 0.30, seed0 + 6, idx)
    h_hop = g.math("MULTIPLY", _rand(g, 0.0, 0.006 if indoor else 0.20, seed0 + 7, idx),
                   g.math("MINIMUM", g.math("DIVIDE", u0, 8.0), 1.0))
    hop = g.math("MULTIPLY", h_hop, g.math("ABSOLUTE", g.math("SINE", g.math("MULTIPLY", math.pi, g.math("DIVIDE", tau, t_hop)))))
    jsep = g.sep(jinfo.outputs["Location"])
    dsep = g.sep(dirv)
    pos = g.comb(g.math("ADD", jsep[0], g.math("MULTIPLY", dsep[0], r)),
                 g.math("ADD", jsep[1], g.math("MULTIPLY", dsep[1], r)),
                 g.math("ADD", soil_z + (0.003 if indoor else 0.012), hop))
    setp = g.n("GeometryNodeSetPosition")
    g.l(sid.outputs[0], setp.inputs["Geometry"])
    g.l(pos, setp.inputs["Position"])
    dele = g.n("GeometryNodeDeleteGeometry", domain="POINT")
    g.l(setp.outputs["Geometry"], dele.inputs["Geometry"])
    kill = g.math("MAXIMUM", g.math("SUBTRACT", 1.0, active), g.math("GREATER_THAN", age, 0.92))
    g.l(kill, dele.inputs["Selection"])
    src = g.obj(piece)
    src.inputs["As Instance"].default_value = True
    inst = g.n("GeometryNodeInstanceOnPoints")
    g.l(dele.outputs["Geometry"], inst.inputs["Points"])
    g.l(src.outputs["Geometry"], inst.inputs["Instance"])
    e = g.n("FunctionNodeEulerToRotation")
    spin = g.math("MULTIPLY", tsim, _rand(g, 4.0, 14.0, seed0 + 9, idx))
    if indoor:
        # grit slides and turns flat on the board
        g.l(g.comb(0.0, 0.0, g.math("ADD", spin, _rand(g, 0, 6.28, seed0 + 10, idx))), e.inputs[0])
        size = _rand(g, 0.0025, 0.006, seed0 + 8, idx)
    else:
        # clippings tumble while hopping, lie on the turf otherwise
        tumble = g.math("MULTIPLY", g.math("MINIMUM", g.math("DIVIDE", hop, 0.01), 1.0), spin)
        g.l(g.comb(g.math("ADD", tumble, _rand(g, -0.4, 0.4, seed0 + 11, idx)), g.math("MULTIPLY", tumble, 0.7),
                   g.math("ADD", ang, _rand(g, -0.8, 0.8, seed0 + 10, idx))), e.inputs[0])
        size = _rand(g, 0.015, 0.035, seed0 + 8, idx)
    g.l(e.outputs[0], inst.inputs["Rotation"])
    g.l(g.comb(size, size, size), inst.inputs["Scale"])
    g.l(inst.outputs["Instances"], g.out.inputs[0])
    me = bpy.data.meshes.new("debris_host")
    me.from_pydata([[0.0, 0.0, 0.0]], [], [])
    host = bpy.data.objects.new("jet_debris", me)
    U.link(host, coll)
    mod = host.modifiers.new("debris", "NODES")
    mod.node_group = g.ng
    return host
