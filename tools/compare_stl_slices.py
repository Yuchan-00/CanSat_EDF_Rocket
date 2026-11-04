"""Compare two generators' STL exports of the same part slice by slice (SPEC s7 X3).

Invariant enforced: the Fusion 360 script and the CadQuery twin are two
independent compilers of the same spec, so every printed part must have the
same cross-section area at every height.  This tool reports where along Z (and
how much) the two meshes disagree, which localises a modelling difference to a
feature instead of a single total-volume number.

X3 verdict per part (all must hold, no evidence is never a PASS):
  * |total dV| <= --tol percent (0.5 %, SPEC s7 X3);
  * every Z band of disagreeing slices has |integrated dV| <= --band-max mm^3
    (BAND_DV_MAX_MM3, above the tessellation noise, see below);
  * every band with |integrated dV| > --band-min mm^3 (BAND_DV_MIN_MM3) differs by
    at most --band-rel (BAND_REL_MAX) of its own volume: tessellation spreads a
    small fraction over many slices, a missing / extra feature concentrates in a
    few - this is what catches a feature far below the 0.5 % total;
  * no slice area is NaN (a section trimesh could not evaluate is no evidence).
Slice planes sit at z0 + step/2 + k*step + EPS_MM and are nudged further (in EPS_MM
steps) until they are at least EPS_MM/2 from every vertex height of either mesh,
so a plane never lies on a horizontal feature face (v0.4 put one on the
propulsion deck: one slice read 33131 vs 3011 mm^2, a +15060 mm^3 "band").

Usage:
    python tools/compare_stl_slices.py [part ...] [--step 1.0] [--tol 0.5] [--band-max 250] [--band-min 20]
                                       [--band-rel 0.03] [--out PATH|-]
Defaults compare every STL present in both cad/exports/fusion/stl and
cad/exports/cadquery/stl and write what is printed to
cad/exports/crosscheck_fusion_vs_cadquery.txt (``--out -``: stdout only).
Exit code 1 if any part fails X3.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Callable

import numpy as np
import trimesh

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
FUSION_DIR = os.path.join(ROOT, "cad", "exports", "fusion", "stl")
CQ_DIR = os.path.join(ROOT, "cad", "exports", "cadquery", "stl")
DEFAULT_OUT = os.path.join(ROOT, "cad", "exports", "crosscheck_fusion_vs_cadquery.txt")

# slice-plane offset: far above float32 STL rounding (~3e-5 mm at z = 850 mm), far below any feature
EPS_MM = 1e-3
# Band thresholds, from the v0.5c exports (the last Fusion + CadQuery pair before v0.5d) compared by this tool at
# step 1 mm.  The long thin tubes give bands where Fusion's coarse facets (about 30 per circle) lose 0.6-0.8 % of
# the annulus against CadQuery's fine mesh: payload_module 551.4..604.4 dV -181.4 mm^3 (A 444.6 / 448.0 mm^2; the
# exact annulus pi (60^2 - 58.8^2) is 447.9) and 632.4..659.4 -94.0, avionics_module 356.4..401.4 +138.9 and
# 449.4..502.4 -129.8; the nose shell (Fusion ShellFeature vs CadQuery normal offset, -0.45 % in total) gives 29
# bands, the largest 705.8..719.8 -95.9 mm^3; propulsion_module 108.5..111.5 +34.1 (liner ring circles).  So:
#   BAND_DV_MAX_MM3 = 250: 1.4 x the largest tessellation band (181.4);
#   BAND_REL_MAX = 3 % of the band's own volume for bands above BAND_DV_MIN_MM3 = 20 mm^3: the largest of those in
#   the v0.5c output is 1.66 % (nose 737.8..740.8, 28.7 mm^3 over 4 slices of ~433 mm^2), tessellation bands are
#   0.6-0.8 %.  A feature concentrates in few slices: the two v0.5d tray stops (48 mm^3 in 2 slices of ~515 mm^2)
#   are 4.7 %, the channel floor wedge (186 mm^3 in ~5 slices of ~520 mm^2) ~7 %, the v0.4 on-face slice +15060.
# (Before the EPS_MM offset the v0.5c avionics_tray also showed a one-slice "band" +144.6 mm^3 at z 459.4, where the
# CadQuery section read 0 on an FC-hole face; with the offset that band is gone.)
BAND_DV_MAX_MM3 = 250.0
# v0.5d: slice areas cannot see a feature rotated about Z (the tabs moving from 0/180 to 90/270 deg) or differences
# that cancel inside a band; the per-part centroid and the inertia diagonal can.  Noise on the good v0.5c pair:
# |dCOM| <= 0.017 mm, inertia diagonal <= 0.042 %; tabs rotated by 90 deg: dCOM 0.43 mm, Ixx/Iyy +2.57 / -2.50 %.
# The per-part inertia limit is 1 %: the nose's Haack profile is a spline in Fusion and a polyline in the twin, which
# alone gives 0.61 % on the nose (volume 0.45 %); rotated tabs (2.5 %) still fail it.
COM_TOL_MM = 0.10
INERTIA_TOL = 0.01
BAND_DV_MIN_MM3 = 20.0
BAND_REL_MAX = 0.03


def load_mm(path: str) -> trimesh.Trimesh:
    """Load an STL as one mesh (``compare`` rescales cm -> mm when a Fusion mesh is 10x too small)."""
    return trimesh.load(path, force="mesh")


def slice_heights(z0: float, z1: float, step: float, vertex_z: np.ndarray, eps: float = EPS_MM) -> np.ndarray:
    """Slice planes z0 + step/2 + k step + eps, each moved up by eps until it is >= eps/2 from every vertex
    height in ``vertex_z``: a plane through a vertex ring (a horizontal face, a cylinder's end) makes the section
    ambiguous - one mesh reports the face, the other the material beside it."""
    vz = np.unique(np.asarray(vertex_z, dtype=float))
    zs = np.arange(z0 + step / 2, z1, step) + eps
    for i in range(len(zs)):
        z = zs[i]
        for _ in range(1000):
            j = int(np.searchsorted(vz, z))
            gap = min((abs(z - vz[k]) for k in (j - 1, j) if 0 <= k < len(vz)), default=np.inf)
            if gap >= eps / 2:
                break
            z += eps
        else:
            raise RuntimeError(f"no slice plane clear of the vertex heights near z = {zs[i]:.3f}")
        zs[i] = z
    return zs


def section_areas(mesh: trimesh.Trimesh, zs: np.ndarray) -> np.ndarray:
    """Cross-section area at each z (0 where the plane misses the mesh, NaN where trimesh fails)."""
    out = np.zeros(len(zs))
    for i, z in enumerate(zs):
        sec = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if sec is None:
            continue
        try:
            planar, _ = sec.to_2D()
            out[i] = planar.area
        except Exception:  # noqa: BLE001 - degenerate slice: no evidence
            out[i] = np.nan
    return out


def bands(bad: np.ndarray, zs: np.ndarray, d: np.ndarray, a_f: np.ndarray, a_c: np.ndarray,
          step: float) -> list[tuple[float, float, float, float, float, float]]:
    """Consecutive disagreeing slices grouped into Z bands: (z0, z1, integrated dV mm^3, mean A fusion, mean A cq,
    |dV| / band volume) - the band volume taken from the larger of the two areas slice by slice."""
    out = []
    start = None
    for i, b in enumerate(bad):
        if b and start is None:
            start = i
        if (not b or i == len(bad) - 1) and start is not None:
            end = i if b else i - 1
            band = slice(start, end + 1)
            dv = float(np.nansum(d[band]) * step)
            vol = float(np.nansum(np.fmax(a_f[band], a_c[band])) * step)
            with np.errstate(all="ignore"):
                af, ac = float(np.nanmean(a_f[band])), float(np.nanmean(a_c[band]))
            out.append((float(zs[start]), float(zs[end]), dv, af, ac, abs(dv) / vol if vol > 0 else np.inf))
            start = None
    return out


def compare(name: str, step: float, tol_pct: float, band_max: float = BAND_DV_MAX_MM3,
            say: Callable[[str], None] = print, band_min: float = BAND_DV_MIN_MM3,
            band_rel: float = BAND_REL_MAX) -> bool:
    """X3 for one part; True when the total volume, every band and every slice pass (module docstring)."""
    fa = load_mm(os.path.join(FUSION_DIR, f"{name}.stl"))
    ca = load_mm(os.path.join(CQ_DIR, f"{name}.stl"))
    scale = 1.0
    if fa.extents.max() * 5 < ca.extents.max():
        scale = 10.0
        fa.apply_scale(scale)
    dv = (fa.volume / ca.volume - 1.0) * 100.0
    say(f"\n== {name}: V fusion {fa.volume:.0f} mm3, cadquery {ca.volume:.0f} mm3, dV {dv:+.2f} %"
        f"{'  (fusion STL scaled x10: exported in cm)' if scale != 1.0 else ''}")
    say(f"   bbox fusion {np.round(fa.bounds, 2).tolist()}")
    say(f"   bbox cq     {np.round(ca.bounds, 2).tolist()}")
    z0 = min(fa.bounds[0, 2], ca.bounds[0, 2])
    z1 = max(fa.bounds[1, 2], ca.bounds[1, 2])
    zs = slice_heights(z0, z1, step, np.concatenate([fa.vertices[:, 2], ca.vertices[:, 2]]))
    a_f = section_areas(fa, zs)
    a_c = section_areas(ca, zs)
    d = a_f - a_c
    nan = np.isnan(a_f) | np.isnan(a_c)
    bad = (np.abs(d) > np.maximum(0.5, 0.005 * np.maximum(a_f, a_c))) | nan
    bl = bands(bad, zs, d, a_f, a_c, step) if bad.any() else []
    if bl:
        say("   Z bands where the sections differ (z0..z1, integrated dV mm3, mean area fusion / cq mm2):")
        for (za, zb, vol, af, ac, _) in bl[:25]:
            say(f"     {za:8.1f} .. {zb:8.1f}  dV {vol:+9.1f}   A {af:9.1f} / {ac:9.1f}")
        if len(bl) > 25:
            say(f"     ... {len(bl) - 25} more bands")
    else:
        say("   sections agree at every slice")
    # centroid and inertia diagonal (unit density, about the centroid, vehicle axes)
    try:
        dcom = float(np.linalg.norm(np.asarray(fa.center_mass) - np.asarray(ca.center_mass)))
        i_f, i_c = np.diag(fa.moment_inertia), np.diag(ca.moment_inertia)
        di = float(np.max(np.abs(i_f / i_c - 1.0)))
    except Exception:  # noqa: BLE001 - no evidence
        dcom, di = math.inf, math.inf
    ok_m = bool(dcom <= COM_TOL_MM and di <= INERTIA_TOL)
    say(f"   centroid shift {dcom:.3f} mm (<= {COM_TOL_MM:g}), inertia diagonal max |d| {100 * di:.3f} % (<= {100 * INERTIA_TOL:g})")
    band_worst = max((abs(b[2]) for b in bl), default=0.0)
    rel_worst = max((b[5] for b in bl if abs(b[2]) > band_min), default=0.0)
    ok_v = bool(abs(dv) <= tol_pct)
    ok_b = bool(band_worst <= band_max)
    ok_r = bool(rel_worst <= band_rel)
    ok_n = not nan.any()
    ok = ok_v and ok_b and ok_r and ok_n and ok_m
    say(f"   X3 {'PASS' if ok else 'FAIL'}: |dV| {abs(dv):.2f} % (<= {tol_pct:g}: {ok_v}); max |band dV| "
        f"{band_worst:.1f} mm3 (<= {band_max:g}: {ok_b}); bands above {band_min:g} mm3 differ by at most "
        f"{100 * rel_worst:.2f} % of their volume (<= {100 * band_rel:g}: {ok_r}); NaN slice areas {int(nan.sum())} "
        f"(= 0: {ok_n}); {len(zs)} planes, step {step:g} mm, each >= {EPS_MM / 2:g} mm off every vertex height")
    return ok


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("parts", nargs="*")
    ap.add_argument("--step", type=float, default=1.0, help="slice spacing in mm")
    ap.add_argument("--tol", type=float, default=0.5, help="volume tolerance in percent")
    ap.add_argument("--band-max", type=float, default=BAND_DV_MAX_MM3,
                    help="largest |integrated dV| of one Z band in mm3 (default: above the measured tessellation noise)")
    ap.add_argument("--band-min", type=float, default=BAND_DV_MIN_MM3,
                    help="bands with |dV| above this (mm3) must also pass --band-rel")
    ap.add_argument("--band-rel", type=float, default=BAND_REL_MAX,
                    help="largest |dV| of such a band as a fraction of its own volume")
    ap.add_argument("--out", default=DEFAULT_OUT, help="summary file ('-' = stdout only)")
    a = ap.parse_args(argv)
    names = a.parts or sorted(os.path.splitext(f)[0] for f in os.listdir(FUSION_DIR)
                              if f.endswith(".stl") and os.path.exists(os.path.join(CQ_DIR, f)))
    lines: list[str] = []

    def say(s: str) -> None:
        print(s)
        lines.append(s)

    if not names:
        say("\nno part has both a Fusion and a CadQuery STL: nothing compared")
    ok = bool(names)
    failed = []
    for n in names:
        if not compare(n, a.step, a.tol, a.band_max, say, a.band_min, a.band_rel):
            ok = False
            failed.append(n)
    say("\nRESULT: " + ("all parts within tolerance" if ok else "some parts differ beyond tolerance")
        + f" (X3: |dV| <= {a.tol:g} %, every band |dV| <= {a.band_max:g} mm3 and, above {a.band_min:g} mm3, "
          f"<= {100 * a.band_rel:g} % of its volume, no NaN slice)"
        + (f"; FAIL: {', '.join(failed)}" if failed else ""))
    if a.out != "-":
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(lines) + "\n")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
