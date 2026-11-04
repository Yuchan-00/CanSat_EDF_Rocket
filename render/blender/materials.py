"""Materials (Principled BSDF node trees) for the vehicle, the pad, the person and the environments.

USER DECISION (2026-10-20 23:45, binding): every printed part of the vehicle - modules, nose, vanes, horns,
latch - AND the launch pad is Bambu Lab PLA Matte BLACK (params.json printer.material / color): ONE
material ``PLA_matte_black`` - linear albedo 0.04 (sRGB ~#383838), roughness 0.87 +- 0.03, specular F0 3.4 %, a
faint velvety sheen (0.03) at grazing angles and 0.2 mm print layer lines + the slicer's vertical z-seam as a bump
(object space), faded out with camera distance so far views do not alias.  No other
filament colour exists in this pipeline.  Non-printed parts keep realistic materials: bare / anodised
aluminium EDF, black nylon rotor, black servo plastic.

Invariant: no image texture is used without its file on disk; missing assets fall back to procedural
colour and the builder reports it.
"""
from __future__ import annotations

import os
from typing import Any

import bpy

from . import bl_util as U

PLA_BLACK_ALBEDO = 0.04            # linear diffuse albedo of matte black PLA (USER DECISION 1)
PLA_BLACK_ROUGH = 0.87
PALETTE = {"servo": "#141517", "pcb": "#1d3b2a"}


SEAM_THETA = 3.6                   # rad: azimuth of the FDM z-seam (where every layer starts) on the round parts


def _layer_bump(nb: U.NodeBuilder, pitch_m: float = 0.0002, strength: float = 0.12) -> Any:
    """Normal output: printed layer lines (sine along object z) + fine noise + the vertical z-seam (a 1.2 mm band of
    small per-layer blobs at SEAM_THETA, as a Bambu slicer leaves on a round part), faded with camera distance."""
    tc = nb.n("ShaderNodeTexCoord")
    sep = nb.n("ShaderNodeSeparateXYZ")
    nb.l(tc.outputs["Object"], sep.inputs[0])
    ph = nb.math("MULTIPLY", sep.outputs[2], 2.0 * 3.14159265 / pitch_m)
    s = nb.math("ADD", nb.math("MULTIPLY", nb.math("SINE", ph), 0.5), 0.5)
    noise = nb.n("ShaderNodeTexNoise", inputs={"Scale": 900.0, "Detail": 2.0})
    nb.l(tc.outputs["Object"], noise.inputs["Vector"])
    h0 = nb.math("ADD", s, nb.math("MULTIPLY", noise.outputs["Fac"], 0.6))
    # z-seam: angular distance to SEAM_THETA times the radius = arc distance (m); a 0.6 mm half-width band
    ang = nb.math("ARCTAN2", sep.outputs[1], sep.outputs[0])
    dth = nb.math("ABSOLUTE", nb.math("SUBTRACT", ang, SEAM_THETA - 2 * 3.14159265))
    rad = nb.math("SQRT", nb.math("ADD", nb.math("MULTIPLY", sep.outputs[0], sep.outputs[0]),
                                   nb.math("MULTIPLY", sep.outputs[1], sep.outputs[1])))
    arc = nb.math("MULTIPLY", dth, rad)
    band = nb.n("ShaderNodeMapRange", inputs={1: 0.0, 2: 0.0006, 3: 1.0, 4: 0.0})
    band.clamp = True
    band.interpolation_type = "SMOOTHSTEP"
    nb.l(arc, band.inputs[0])
    blobs = nb.math("MULTIPLY", band.outputs[0], nb.math("ADD", 1.2, nb.math("MULTIPLY", nb.math("SINE", ph), 0.8)))
    h = nb.math("ADD", h0, blobs)
    cam = nb.n("ShaderNodeCameraData")
    # (round 2: from 0.25-1.2 m to 0.3-2.0 m - the faint layer sheen the user welcomed now shows in the close-ups and the
    #  studio rims, still gone before it could alias in the wide shots)
    fade = nb.n("ShaderNodeMapRange", inputs={1: 0.3, 2: 2.0, 3: 1.0, 4: 0.0})
    fade.clamp = True
    nb.l(cam.outputs["View Distance"], fade.inputs[0])
    st = nb.math("MULTIPLY", fade.outputs[0], strength)
    bump = nb.n("ShaderNodeBump", inputs={"Distance": 0.00003})
    nb.l(st, bump.inputs["Strength"])
    nb.l(h, bump.inputs["Height"])
    return bump.outputs["Normal"]


def pla_matte_black(name: str = "PLA_matte_black", layer_strength: float = 0.34) -> Any:
    """Bambu Lab PLA Matte Black (USER DECISION 1): the one material of every printed part and the pad."""
    m = bpy.data.materials.get(name)
    if m is not None and m.get("_built"):
        return m
    m, nb, out = U.new_material(name)
    m["_built"] = 1
    a = PLA_BLACK_ALBEDO
    tc = nb.n("ShaderNodeTexCoord")
    noise = nb.n("ShaderNodeTexNoise", inputs={"Scale": 22.0, "Detail": 3.0, "Roughness": 0.5})
    nb.l(tc.outputs["Object"], noise.inputs["Vector"])
    # +-6 % albedo variation (filament / layer non-uniformity), roughness +-0.03
    var = nb.n("ShaderNodeMapRange", inputs={1: 0.3, 2: 0.7, 3: 0.94 * a, 4: 1.06 * a})
    nb.l(noise.outputs["Fac"], var.inputs[0])
    col = nb.n("ShaderNodeCombineColor")
    for i in range(3):
        nb.l(var.outputs[0], col.inputs[i])
    rr = nb.n("ShaderNodeMapRange", inputs={1: 0.0, 2: 1.0, 3: PLA_BLACK_ROUGH - 0.03, 4: PLA_BLACK_ROUGH + 0.03})
    nb.l(noise.outputs["Fac"], rr.inputs[0])
    # specular level 0.42 = F0 3.4 % (PLA, n ~ 1.46); round 1 used 0.5 (F0 4 %) + sheen 0.06 and the body read grey
    p = U.principled(nb, Specular_IOR_Level=0.42)
    nb.l(col.outputs[0], p.inputs["Base Color"])
    nb.l(rr.outputs[0], p.inputs["Roughness"])
    # matte PLA: a soft velvety lift at grazing angles (keeps the black shapes readable against dark ground)
    for k, v in (("Sheen Weight", 0.03), ("Sheen Roughness", 0.45)):
        if k in p.inputs:
            p.inputs[k].default_value = v
    if "Sheen Tint" in p.inputs:
        p.inputs["Sheen Tint"].default_value = (0.55, 0.56, 0.58, 1.0)
    nb.l(_layer_bump(nb, strength=layer_strength), p.inputs["Normal"])
    nb.l(p.outputs[0], out.inputs["Surface"])
    return m


def pla(name: str, hexcol: str = "", rough: float = 0.0, sss: float = 0.0, layer: bool = True, metallic: float = 0.0) -> Any:
    """Every printed part is PLA Matte Black (USER DECISION 1); the colour arguments are ignored on purpose."""
    return pla_matte_black()


def metal(name: str, rgb: tuple[float, float, float], rough: float = 0.28, aniso: float = 0.5) -> Any:
    m, nb, out = U.new_material(name)
    tc = nb.n("ShaderNodeTexCoord")
    noise = nb.n("ShaderNodeTexNoise", inputs={"Scale": 400.0, "Detail": 2.0})
    nb.l(tc.outputs["Object"], noise.inputs["Vector"])
    rr = nb.n("ShaderNodeMapRange", inputs={1: 0.3, 2: 0.7, 3: rough - 0.05, 4: rough + 0.05})
    nb.l(noise.outputs["Fac"], rr.inputs[0])
    p = U.principled(nb, Metallic=1.0, Base_Color=(rgb[0], rgb[1], rgb[2], 1.0), Anisotropic=aniso)
    nb.l(rr.outputs[0], p.inputs["Roughness"])
    tan = nb.n("ShaderNodeTangent", direction_type="RADIAL", axis="Z")
    nb.l(tan.outputs[0], p.inputs["Tangent"])
    nb.l(p.outputs[0], out.inputs["Surface"])
    return m


def plastic(name: str, hexcol: str, rough: float = 0.5) -> Any:
    m, nb, out = U.new_material(name)
    p = U.principled(nb, Base_Color=U.srgb(hexcol), Roughness=rough)
    nb.l(p.outputs[0], out.inputs["Surface"])
    return m


def vehicle_materials() -> dict[str, Any]:
    black = pla_matte_black()
    mats = {
        "propulsion": black, "intake": black, "avionics": black, "payload": black, "nose": black, "dark": black,
        "horn": black, "vane": black,
        # EDF housing + intake lip: grey-anodised aluminium (bare brushed alu mirrored the bore light into a white
        # crescent inside the duct)
        "alu": metal("aluminium_anodised_grey", (0.20, 0.205, 0.21), rough=0.40, aniso=0.45),
        # satin aluminium motor can / tail cone / stators: what the bore light shows inside the black duct
        # (rough 0.5: a satin-blasted cone scatters the bore light toward the lens instead of mirroring the dark duct)
        # (round 2: darker grey anodised - the near-axis close-up looks straight at the cone's tip, which clipped white)
        "edf_core": metal("edf_satin_alu", (0.36, 0.37, 0.39), rough=0.55, aniso=0.2),
        "servo": plastic("servo_black_plastic", PALETTE["servo"], rough=0.45),
        "rotor": rotor_blade_material(),
        "rotor_disc": rotor_disc_material(),
    }
    return mats


def object_attr(nb: U.NodeBuilder, name: str) -> Any:
    a = nb.n("ShaderNodeAttribute", attribute_type="OBJECT", attribute_name=name)
    return a.outputs["Fac"]


def rotor_blade_material() -> Any:
    """Black nylon blades that fade out as the rotor spins up (object property 'spin' 0..1); the blur disc
    takes over (a real 25 000 rpm rotor is a blur at a 1/60 s shutter)."""
    m, nb, out = U.new_material("rotor_blades")
    p = U.principled(nb, Base_Color=U.srgb("#202124"), Roughness=0.38)
    tr = nb.n("ShaderNodeBsdfTransparent")
    mix = nb.n("ShaderNodeMixShader")
    nb.l(nb.math("MULTIPLY", object_attr(nb, "spin"), 0.92), mix.inputs[0])
    nb.l(p.outputs[0], mix.inputs[1])
    nb.l(tr.outputs[0], mix.inputs[2])
    nb.l(mix.outputs[0], out.inputs["Surface"])
    return m


def rotor_disc_material() -> Any:
    m, nb, out = U.new_material("rotor_blur_disc")
    p = U.principled(nb, Base_Color=U.srgb("#3c3f44"), Roughness=0.45)
    tr = nb.n("ShaderNodeBsdfTransparent")
    mix = nb.n("ShaderNodeMixShader")
    # opacity = spin x 0.55 (the blades cover ~50 % of the annulus)
    op = nb.math("MULTIPLY", object_attr(nb, "spin"), 0.55)
    nb.l(nb.math("SUBTRACT", 1.0, op), mix.inputs[0])
    nb.l(p.outputs[0], mix.inputs[1])
    nb.l(tr.outputs[0], mix.inputs[2])
    nb.l(mix.outputs[0], out.inputs["Surface"])
    return m


def vane_material(i: int = 0, mark_hex: str = "") -> Any:
    """Vanes are printed: PLA Matte Black like every printed part (USER DECISION 1), no paint marks."""
    return pla_matte_black()


def pad_material() -> Any:
    """The launch pad is printed in the same filament (USER DECISION 1)."""
    return pla_matte_black()


# --------------------------------------------------------------------------- #
# image-based PBR sets (Poly Haven, 2k jpg: diff / nor_gl / rough)
# --------------------------------------------------------------------------- #
def pbr_set(asset_dir: str, aid: str) -> dict[str, str] | None:
    d = os.path.join(asset_dir, aid)
    if not os.path.isdir(d):
        return None
    out = {}
    for f in os.listdir(d):
        fl = f.lower()
        if "_diff_" in fl:
            out["diff"] = os.path.join(d, f)
        elif "_nor_gl_" in fl:
            out["nor"] = os.path.join(d, f)
        elif "_rough_" in fl:
            out["rough"] = os.path.join(d, f)
    return out if "diff" in out else None


def pbr_nodes(nb: U.NodeBuilder, files: dict[str, str], vec: Any, tint: tuple[float, float, float] | None = None,
              rough_scale: float = 1.0, nor_strength: float = 1.0) -> dict[str, Any]:
    """Image texture nodes for a PBR set at vector ``vec``; returns sockets 'color', 'rough', 'normal'."""
    out: dict[str, Any] = {}
    td = nb.n("ShaderNodeTexImage", interpolation="Linear")
    td.image = U.image(files["diff"], "sRGB")
    nb.l(vec, td.inputs["Vector"])
    col = td.outputs["Color"]
    if tint is not None:
        hsv = nb.n("ShaderNodeHueSaturation", inputs={"Hue": tint[0], "Saturation": tint[1], "Value": tint[2], "Fac": 1.0})
        nb.l(col, hsv.inputs["Color"])
        col = hsv.outputs["Color"]
    out["color"] = col
    if "rough" in files:
        tr = nb.n("ShaderNodeTexImage")
        tr.image = U.image(files["rough"], "Non-Color")
        nb.l(vec, tr.inputs["Vector"])
        out["rough"] = nb.math("MULTIPLY", tr.outputs["Color"], rough_scale, clamp=True)
    if "nor" in files:
        tn = nb.n("ShaderNodeTexImage")
        tn.image = U.image(files["nor"], "Non-Color")
        nb.l(vec, tn.inputs["Vector"])
        nm = nb.n("ShaderNodeNormalMap", inputs={"Strength": nor_strength})
        nb.l(tn.outputs["Color"], nm.inputs["Color"])
        out["normal"] = nm.outputs["Normal"]
    return out


def world_xy_vector(nb: U.NodeBuilder, tile_m: float, rot_deg: float = 0.0, offset: tuple[float, float] = (0.0, 0.0)) -> Any:
    """Texture vector = world XY / tile size (one image per tile_m metres)."""
    tc = nb.n("ShaderNodeTexCoord")
    mp = nb.n("ShaderNodeMapping", vector_type="POINT")
    mp.inputs["Scale"].default_value = (1.0 / tile_m, 1.0 / tile_m, 1.0)
    mp.inputs["Rotation"].default_value = (0.0, 0.0, rot_deg * 3.14159265 / 180.0)
    mp.inputs["Location"].default_value = (offset[0], offset[1], 0.0)
    geo = nb.n("ShaderNodeNewGeometry")
    nb.l(geo.outputs["Position"], mp.inputs["Vector"])
    _ = tc
    return mp.outputs["Vector"]
