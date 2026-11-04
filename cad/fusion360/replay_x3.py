"""X3 without Fusion: replay the Fusion script's feature calls (recorded in a mock run) in OCC and slice-compare the result
with the CadQuery twin.

What it proves: the Fusion 360 script's OWN construction logic for a component - which CB primitive it calls, with which
numbers (full float precision, not the 4-digit FEAT log), which operation (NewBody / Join / Cut) and in which order -
builds the same solid as the independent CadQuery twin.  What it does NOT prove: how the Fusion kernel executes those
calls (construction-plane conventions, sketch profiles, feature health) - that still needs the real GUI run and
tools/compare_stl_slices.py on its exports.  Every primitive is re-implemented here from the CB method's documented
geometry (box_pod: box in the pod frame from z0 to z1; prism: (r, z) polygon in the radial plane at theta extruded
symmetrically by width; poly_pod: pod-frame polygon from z0 to z1; sector_z: annular sector; cyl_z; revolve_z: (r, z)
polygon revolved 360 deg about Z; radial_cyl: cylinder Ø d on the radial line at theta / z from r0 to r1), not taken
from rocket_geometry.  A component that calls anything else (or passes participant bodies) is refused, not guessed.

Each replayed part must match the twin by boolean difference (replay - twin and twin - replay < 1 mm^3) and, for printed
parts, pass tools/compare_stl_slices.py's X3 on STLs of both.  The boolean test is the strict one: a corner chamfer 4 -> 3
mm or the lid posts moved 0.5 mm pass the slice test but fail the boolean one (the v0.8 negative controls).

0.9.6 (livery, user 2026-11-04): also models CB.inlay - one logo island at a copy azimuth: annular sector (NEW body)
INTERSECT the island's pre-warped (v, z) outline (holes subtracted) extruded radially; op NEW keeps it as a body of the
component (the orange part ``avionics_logo``), op CUT subtracts it from the main body (the white ``avionics_module``'s
pocket).  Run with ``--parts avionics_module,avionics_logo`` in both configurations.
v0.8 (GPS pod): written to verify the pod / lid / entry-hole code of CanSatEDFRocket.py 0.8.0 without Fusion 360.
v0.8.1 (GNS board): also models CB.tprism ((v, z) polygon in the plane perpendicular to the radial direction theta at
u_mid, extruded radially u_mid +- length / 2) and CB.cut_between (a cylinder Ø d from model point A to B, cut from the main
body) - the GNS bosses and tap holes of CanSatEDFRocket.py 0.8.1 - and replays the placeholder gns by default.
v0.9 (direct-drive vanes, open motor-lead groove): also models CB.cyl_between's NEW cylinder body (``_cyl_body_between``,
the servo-lead pipes, joined by combine_all) and CB.through_circle (a symmetric through-cut circle on the vertical plane
whose normal is theta, optionally a D profile keeping the larger side of z = flat_z - the coupler's D-bore).  Run with
``--parts propulsion_module,servo_cover_1,...,vane_coupler_1,wire_cover,launch_pad,servo_vane_1,servo_horn_1`` for the v0.9
parts.  Known limits (not modelled, so the slice / boolean test FAILs for a reason outside the part's own code): the
intake's gable window cuts (extrude INTERSECT with participant bodies + Combine/Cut of the tool - the four window volumes
stay in the replay) and the vane's LE / TE fillets (CB.soft in the mock run finds no edges, so no fillet is recorded).

Usage: python cad/fusion360/replay_x3.py [--root PROJECT_ROOT] [--out DIR] [--preset NAME] [--parts a,b,...]
Defaults: the project this file sits in; OUT = cad/exports/fusion_replay[_<preset>].  Writes OUT/replay_stl/<part>.stl,
OUT/cq_stl/<part>.stl, OUT/replay_brep/<part>.brep, OUT/x3_replay.json and
cad/exports/crosscheck_fusion_replay_vs_cadquery[_<preset>].txt; exit 1 on any FAIL or unmodelled part.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import shutil
import sys
import tempfile

_HERE_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument("--root", default=_HERE_ROOT)
ap.add_argument("--out", default=None)
ap.add_argument("--preset", default=None)
ap.add_argument("--parts", default="payload_module,avionics_module,gps_pod_lid,gps,gps_ground_plane,piston,gns")
ap.add_argument("--config", default=None, help="v0.9 CAD-2 hardware configuration (rocket | vtol)")
ap.add_argument("--power", default=None, help="v0.9 CAD-2 VTOL power source")
a = ap.parse_args()
ROOT = os.path.abspath(a.root)
_CSFX = ("" if a.config in (None, "rocket") else f"_{a.config}") + (f"_{a.power}" if a.power and a.power != "battery" else "")
OUT = os.path.abspath(a.out or os.path.join(ROOT, "cad", "exports", "fusion_replay" + (f"_{a.preset}" if a.preset else "") + _CSFX))
PARTS = [p for p in a.parts.split(",") if p]
os.makedirs(os.path.join(OUT, "replay_stl"), exist_ok=True)
os.makedirs(os.path.join(OUT, "cq_stl"), exist_ok=True)

sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "cad", "cadquery"))
sys.path.insert(0, os.path.join(ROOT, "cad"))
import fusion_mock_run as fm  # noqa: E402  (ROOT = that file's parent: run it from the tree being checked)

if os.path.normcase(fm.ROOT) != os.path.normcase(ROOT):
    raise SystemExit(f"fusion_mock_run.ROOT {fm.ROOT} is not {ROOT}")
adsk = fm.make_fake_adsk()
tmp = tempfile.mkdtemp(prefix="cansat_replay_", dir=OUT)
os.environ["CANSAT_EXPORT_ROOT"] = tmp
os.environ["CANSAT_PROJECT_ROOT"] = ROOT
if a.preset:
    os.environ["CANSAT_PRESET"] = a.preset
else:
    os.environ.pop("CANSAT_PRESET", None)
for _k, _v in (("CANSAT_CONFIG", a.config), ("CANSAT_POWER", a.power)):   # v0.9 CAD-2
    if _v:
        os.environ[_k] = _v
    else:
        os.environ.pop(_k, None)
spec = importlib.util.spec_from_file_location("CanSatEDFRocket", fm.SCRIPT)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

FO = adsk.fusion.FeatureOperations
OPS = {id(FO.NewBodyFeatureOperation): "NEW", id(FO.JoinFeatureOperation): "JOIN", id(FO.CutFeatureOperation): "CUT",
       id(FO.IntersectFeatureOperation): "INTERSECT"}
REC: list[tuple[str, str, tuple, dict]] = []
_depth = [0]
WRAPPED = ("box_pod", "poly_pod", "prism", "sector_z", "cyl_z", "revolve_z", "radial_cyl", "revolve_sector", "plate",
           "through_cut", "through_circle", "fillet", "shell_remove_face", "_cyl_body_between", "combine_all",
           "tprism", "cut_between",      # v0.8.1
           "inlay")                      # 0.9.6 livery


def _wrap(name: str):
    orig = getattr(mod.CB, name)

    def w(self, *args, **kw):
        if _depth[0] == 0:
            REC.append((self.name, name, args, dict(kw)))
        _depth[0] += 1
        try:
            return orig(self, *args, **kw)
        finally:
            _depth[0] -= 1
    setattr(mod.CB, name, w)


for n in WRAPPED:
    _wrap(n)
mod.run("replay")
shutil.rmtree(tmp, ignore_errors=True)

import cadquery as cq  # noqa: E402
import numpy as np  # noqa: E402

V = cq.Vector


def pod_xy(th: float, u: float, v: float) -> tuple[float, float]:
    t = math.radians(th)
    return (u * math.cos(t) - v * math.sin(t), u * math.sin(t) + v * math.cos(t))


def poly_z(pts_xy, z0: float, z1: float) -> cq.Shape:
    lo, hi = min(z0, z1), max(z0, z1)
    return cq.Workplane("XY").workplane(offset=lo).polyline(list(pts_xy)).close().extrude(hi - lo).val()


def op_of(args: tuple, kw: dict, idx: int):
    op = kw.get("op", args[idx] if len(args) > idx else None)
    return OPS.get(id(op), repr(op))


def participants_of(args: tuple, kw: dict, idx: int):
    return kw.get("participants", args[idx] if len(args) > idx else None)


def shape_of(method: str, args: tuple, kw: dict) -> tuple[cq.Shape, str]:
    """(solid, op) of one recorded call; raises for anything not modelled."""
    if method == "box_pod":
        th, u0, u1, v0, v1, z0, z1 = args[:7]
        pts = [pod_xy(th, u0, v0), pod_xy(th, u1, v0), pod_xy(th, u1, v1), pod_xy(th, u0, v1)]
        return poly_z(pts, z0, z1), op_of(args, kw, 7), participants_of(args, kw, 9)
    if method == "poly_pod":
        th, pts_uv, z0, z1 = args[:4]
        return poly_z([pod_xy(th, u, v) for u, v in pts_uv], z0, z1), op_of(args, kw, 4), participants_of(args, kw, 6)
    if method == "prism":
        th, pts_rz, width = args[:3]
        t = math.radians(th)
        c, s = math.cos(t), math.sin(t)
        # polygon in the plane through the axis at theta (local x = radial, local y = +Z: normal (sin, -cos, 0), since
        # CadQuery's yDir = normal x xDir), extruded symmetrically along the tangential direction
        wp = cq.Workplane(cq.Plane(origin=(0, 0, 0), xDir=(c, s, 0), normal=(s, -c, 0)))
        sol = wp.workplane(offset=-width / 2).polyline([(r, z) for r, z in pts_rz]).close().extrude(width).val()
        return sol, op_of(args, kw, 3), participants_of(args, kw, 5)
    if method == "sector_z":
        th0, th1, r0, r1, z0, z1 = args[:6]
        # independent of a revolve: the tube r0..r1 intersected with a fan polygon th0..th1 (vertices far outside r1)
        tube = cq.Solid.makeCylinder(r1, z1 - z0, V(0, 0, z0)).cut(cq.Solid.makeCylinder(r0, z1 - z0 + 2, V(0, 0, z0 - 1)))
        n = 64
        R = 2.0 * r1 / math.cos(math.radians((th1 - th0) / n) / 2)
        fan = [(0.0, 0.0)] + [(R * math.cos(math.radians(th0 + (th1 - th0) * k / n)),
                                R * math.sin(math.radians(th0 + (th1 - th0) * k / n))) for k in range(n + 1)]
        sol = tube.intersect(poly_z(fan, z0 - 1.0, z1 + 1.0))
        return sol, op_of(args, kw, 6), participants_of(args, kw, 8)
    if method == "cyl_z":
        x, y, z0, z1, d = args[:5]
        lo, hi = min(z0, z1), max(z0, z1)
        return cq.Solid.makeCylinder(d / 2.0, hi - lo, V(x, y, lo), V(0, 0, 1)), op_of(args, kw, 5), participants_of(args, kw, 7)
    if method == "revolve_z":
        pts_rz = args[0]
        sol = cq.Workplane("XZ").polyline([(r, z) for r, z in pts_rz]).close().revolve(360.0, (0, 0, 0), (0, 1, 0)).val()
        return sol, op_of(args, kw, 1), participants_of(args, kw, 3)
    if method == "radial_cyl":
        th, z, r0, r1, d = args[:5]
        t = math.radians(th)
        e = V(math.cos(t), math.sin(t), 0.0)
        lo, hi = min(r0, r1), max(r0, r1)
        p0 = V(lo * e.x, lo * e.y, z)
        return cq.Solid.makeCylinder(d / 2.0, hi - lo, p0, e), op_of(args, kw, 5), participants_of(args, kw, 7)
    if method == "tprism":                                    # v0.8.1
        th, u_mid, pts_vz, length = args[:4]
        # independent of rocket_geometry._prism_vz: the (v, z) polygon on the plane x = u_mid - length/2 of the theta = 0
        # frame (CadQuery's "YZ" workplane: local x = global Y, local y = global Z, normal +X), extruded along +X, turned
        sol = cq.Workplane("YZ").workplane(offset=u_mid - length / 2).polyline([(v, z) for v, z in pts_vz]).close().extrude(length).val()
        sol = sol.rotate(V(0, 0, 0), V(0, 0, 1), th)
        return sol, op_of(args, kw, 4), participants_of(args, kw, 6)
    if method == "_cyl_body_between":                         # v0.9: CB.cyl_between's NEW cylinder body A -> B (lead pipes)
        a_mm, b_mm, d = args[:3]
        pa, pb = V(*a_mm), V(*b_mm)
        return cq.Solid.makeCylinder(d / 2.0, (pb - pa).Length, pa, (pb - pa).normalized()), "NEW", None
    if method == "through_circle":                            # v0.9: symmetric through-cut circle (optional D flat) on the
        th, z, d, length = args[:4]                           # vertical plane whose normal is theta (vane coupler D-bore)
        flat_z = kw.get("flat_z", args[5] if len(args) > 5 else None)
        t = math.radians(th)
        e = V(math.cos(t), math.sin(t), 0.0)
        sol = cq.Solid.makeCylinder(d / 2.0, length, V(-length / 2 * e.x, -length / 2 * e.y, z), e)
        if flat_z is not None:                                # keep the larger region of the circle cut by z = flat_z
            big = 4.0 * (length + d)
            z_lo, z_hi = (flat_z, z + d) if flat_z < z else (z - d, flat_z)
            sol = sol.intersect(cq.Solid.makeBox(big, big, z_hi - z_lo, V(-big / 2, -big / 2, z_lo)))
        return sol, "CUT", kw.get("participants", args[6] if len(args) > 6 else None)
    if method == "inlay":                                     # 0.9.6 livery: one logo island (CB.inlay)
        th, rings_vz, r_in, r_out, sector, u_mid, length = args[:7]
        op = op_of(args, kw, 7)
        lo, hi, z0, z1 = sector
        # independent of rocket_geometry.livery_inlay_solids: the band = the tube r_in..r_out intersected with a fan
        # polygon lo..hi (as sector_z above); the outline = the outer ring's (v, z) prism on the theta = 0 frame's plane
        # x = u_mid - length/2 extruded along +X (as tprism above) MINUS one such prism per hole ring, turned to theta
        tube = cq.Solid.makeCylinder(r_out, z1 - z0, V(0, 0, z0)).cut(cq.Solid.makeCylinder(r_in, z1 - z0 + 2, V(0, 0, z0 - 1)))
        n = 64
        Rf = 2.0 * r_out / math.cos(math.radians((hi - lo) / n) / 2)
        fan = [(0.0, 0.0)] + [(Rf * math.cos(math.radians(lo + (hi - lo) * k / n)),
                                 Rf * math.sin(math.radians(lo + (hi - lo) * k / n))) for k in range(n + 1)]
        band = tube.intersect(poly_z(fan, z0 - 1.0, z1 + 1.0))

        def pr(ring):
            return cq.Workplane("YZ").workplane(offset=u_mid - length / 2).polyline([(v, z) for v, z in ring]).close() \
                .extrude(length).val()
        outline = pr(rings_vz[0])
        for h in rings_vz[1:]:
            outline = outline.cut(pr(h))
        outline = outline.rotate(V(0, 0, 0), V(0, 0, 1), th)
        return band.intersect(outline), op, None
    if method == "cut_between":                               # v0.8.1: Combine/Cut of a cylinder A -> B from the main body
        a_mm, b_mm, d = args[:3]
        pa, pb = V(*a_mm), V(*b_mm)
        target = kw.get("target", args[4] if len(args) > 4 else None)
        return cq.Solid.makeCylinder(d / 2.0, (pb - pa).Length, pa, (pb - pa).normalized()), "CUT", target
    raise NotImplementedError(method)


def replay(comp: str) -> cq.Shape:
    calls = [(m, ar, kw) for c, m, ar, kw in REC if c == comp]
    if not calls:
        raise RuntimeError(f"{comp}: no recorded calls")
    bodies: list[cq.Shape] = []
    for m, ar, kw in calls:
        if m == "combine_all":
            if len(bodies) > 1:
                fused = bodies[0].fuse(*bodies[1:]).clean()
                bodies = [fused]
            continue
        sol, op, part = shape_of(m, ar, kw)
        if part is not None:
            raise NotImplementedError(f"{comp}: {m} with explicit participant bodies")
        if op == "NEW":
            bodies.append(sol)
        elif op == "JOIN":
            bodies[0] = bodies[0].fuse(sol).clean()
        elif op == "CUT":
            bodies[0] = bodies[0].cut(sol)
        else:
            raise NotImplementedError(f"{comp}: {m} op {op}")
    if len(bodies) == 1:
        return bodies[0]
    return cq.Compound.makeCompound(bodies)


import compare_stl_slices as csl  # noqa: E402
import rocket_geometry as rg  # noqa: E402

L = rg.layout_for(a.preset) if not (a.config or a.power) else rg.layout_for(a.preset, a.config, a.power)
twin = rg.build_all(L)
preset = L.p["meta"]["preset"]
lines: list[str] = []


def say(s: str) -> None:
    print(s)
    lines.append(s)


say(f"X3 replay (Fusion script feature calls from a mock run, rebuilt in OCC) vs the CadQuery twin - preset {preset}")
say(f"script {fm.SCRIPT} v{mod.SCRIPT_VERSION}; {len(REC)} recorded calls")
csl.FUSION_DIR = os.path.join(OUT, "replay_stl")
csl.CQ_DIR = os.path.join(OUT, "cq_stl")
results: dict[str, dict] = {}
ok_all = True
for name in PARTS:
    comp = "vane_1" if name == "vane" else name
    if comp not in twin:
        say(f"\n== {name}: not in the twin for this preset - skipped")
        continue
    try:
        rep_shape = replay(comp)
    except NotImplementedError as ex:
        say(f"\n== {name}: NOT MODELLED by the replay ({ex}) - no evidence")
        results[name] = {"status": "NOT_VERIFIED", "reason": str(ex)}
        ok_all = False
        continue
    tw = twin[comp].shape
    vr, vt = rep_shape.Volume(), tw.Volume()
    d_and = rep_shape.cut(tw).Volume() if rep_shape.isValid() else math.nan
    d_or = tw.cut(rep_shape).Volume() if rep_shape.isValid() else math.nan
    cr = rep_shape.Center() if hasattr(rep_shape, "Center") else None
    say(f"\n== {name}: replay V {vr:.2f} mm3 (valid {rep_shape.isValid()}, {len(rep_shape.Solids())} solid(s)), twin V {vt:.2f} "
        f"mm3, dV {100 * (vr / vt - 1):+.4f} %; boolean replay - twin {d_and:.4f} mm3, twin - replay {d_or:.4f} mm3")
    entry = {"replay_volume_mm3": vr, "twin_volume_mm3": vt, "replay_minus_twin_mm3": d_and, "twin_minus_replay_mm3": d_or,
             "replay_valid": bool(rep_shape.isValid()), "replay_solids": len(rep_shape.Solids())}
    bool_ok = (math.isfinite(d_and) and math.isfinite(d_or) and max(d_and, d_or) < 1.0 and rep_shape.isValid())
    os.makedirs(os.path.join(OUT, "replay_brep"), exist_ok=True)
    rep_shape.exportBrep(os.path.join(OUT, "replay_brep", f"{name}.brep"))
    if twin[comp].kind == "printed":
        rep_shape.exportStl(os.path.join(csl.FUSION_DIR, f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        tw.exportStl(os.path.join(csl.CQ_DIR, f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        slice_ok = csl.compare(name, 1.0, 0.5, say=say)
        entry["slice_x3"] = "PASS" if slice_ok else "FAIL"
    else:
        slice_ok = True
        entry["slice_x3"] = "n/a (placeholder: boolean difference only)"
    entry["status"] = "PASS" if (bool_ok and slice_ok) else "FAIL"
    say(f"   replay X3 {entry['status']}: boolean differences < 1 mm3 ({bool_ok}); slices {entry['slice_x3']}")
    ok_all = ok_all and entry["status"] == "PASS"
    results[name] = entry
say(f"\nRESULT: {'all replayed parts match the twin' if ok_all else 'differences / unmodelled parts above'}")
sfx = ("" if a.preset is None else f"_{preset}") + _CSFX
with open(os.path.join(ROOT, "cad", "exports", f"crosscheck_fusion_replay_vs_cadquery{sfx}.txt"), "w", encoding="utf-8",
          newline="\n") as f:
    f.write("\n".join(lines) + "\n")
with open(os.path.join(OUT, "x3_replay.json"), "w", encoding="utf-8", newline="\n") as f:
    json.dump({"preset": preset, "script_version": mod.SCRIPT_VERSION, "recorded_calls": len(REC), "parts": results,
               "ok": ok_all}, f, indent=1)
sys.exit(0 if ok_all else 1)
