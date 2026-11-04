"""Generator of the YUCHAN logo polygons (livery, user decisions 2026-11-04) - cad/livery/yuchan_logo_polygons.json.

Invariant: the logo is DESIGN DATA made by this deterministic script from the parameters below (same parameters ->
byte-identical JSON / SVG); nothing downstream draws letters.  params.json ``livery`` names this file, cad/layout.py
(``Layout._livery``) reads its polygons and places them on the part, the CadQuery twin and the Fusion script build the
orange inlay from Layout, and check_geometry's livery check re-measures the placed polygons.

The user picked the "launch" concept, VERTICAL version (reads along the vehicle axis), and accepted its AMS cost; the
refine step (scratch livery/refine, 2026-11-04) fixed the judges' must-fix items.  FINAL / REFINE_NOTES are that step's
final values, copied unchanged; the placement rule (word centred across the copy azimuth, bottom at print z 3.05 = the
3 mm clearance to the part's lower end + 0.05) is the refine step's result on the skin maps of the then-current STLs
(the clamp it hit), written here as a number so the generator needs no STL.

Frame of the saved polygons ("s_z", mm), the same for every copy and both configurations:
  s = arc length on the module's OUTER skin r = body_r from the copy's centre azimuth theta_c: theta = theta_c + s / r
      (vehicle frame, theta from +X toward +Y).  Seen from outside with the nose up, +s points to the RIGHT at every
      azimuth (outside view, not mirrored), so one polygon set serves both copies (67 / 293 deg).
  z = print z = vehicle z - the part's lower end (avionics_module printed upright, bottom on the bed).

Usage: python cad/livery/make_logo.py [--check]   (--check: regenerate in memory and compare with the files on disk)
"""
from __future__ import annotations

import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import shapely  # noqa: E402
from shapely import affinity  # noqa: E402
from shapely.geometry import mapping  # noqa: E402

import compose as C  # noqa: E402

OUT_JSON = os.path.join(HERE, "yuchan_logo_polygons.json")
OUT_SVG = os.path.join(HERE, "yuchan_logo_unrolled.svg")

FINAL = dict(H=11.8, w=2.6, wh=2.3, ro=0.42, gap=2.7, slant=12.0, a_top=0.36, a_bar=0.24,
             kern={"YU": -0.9, "CH": -0.4}, ears=0.0, chamfer=1.3,
             wid={"Y": 1.19, "U": 1.1895, "C": 1.131, "H": 1.1895, "A": 1.287, "N": 1.209},
             stripe=dict(gap=2.6, t0=2.2, t1=4.0, dash_t=2.2, start=-0.4, over=3.4, dgap=[1.8, 2.1, 2.4],
                         dashes=[5.2, 4.4, 3.9]),
             r_concave=C.R_CONCAVE, r_convex=C.R_CONVEX)
BOTTOM_PRINT_Z = 3.05      # lowest orange point, print z (3 mm clearance to the part's lower end + 0.05, refine step)
PRECISION = 1e-4           # coordinate grid of the saved polygons (mm)
CAD_SIMPLIFY = 0.005       # CAD set: Douglas-Peucker tolerance (mm) - below the slicer's own 0.012 mm resolution; the
                           # r 0.35 / 0.70 corner arcs of the design set carry 32 segments per quarter (5x the faces)
REFINE_NOTES = [
    "Y and A terminals: the small upright 'ears' (0.11 H) are removed; every convex corner sharper than 75 deg (the two "
    "Y arm tips and the A's left foot) gets the same square chamfer (1.3 mm facet, cut perpendicular to its stroke); no "
    "corner below 78 deg is left.",
    "Letter gap 2.5 -> 2.7 mm, C-H kern -0.5 -> -0.4; letter-to-stripe gap 2.0 -> 2.6 mm.",
    "Exhaust dashes 5.2 / 4.4 / 3.9 mm long (2.2 mm wide), gaps 1.8 / 2.1 / 2.4 mm; smallest island 8.46 mm2.",
    "Letter widths x 0.975 (Y alone 1.19 H, kerned 0.9 mm closer to the U); cap height 11.8, stem 2.6, bar 2.3, "
    "slant 12 deg, stripe 2.2 -> 4.0 mm unchanged.",
    "Corners: convex orange r 0.35, concave orange corner / white wedge r 0.70, by construction (closing, then opening).",
    "Composition: reads bottom -> top along the axis (letter tops to the left seen from outside, nose up); the word is "
    "centred across the copy azimuth; the exhaust dashes trail below it toward the tail.",
]


def geometry():
    """The placed logo (shapely MultiPolygon, s_z frame) and the letters-only geometry (same transform)."""
    geo, let, _st, _feat = C.build(FINAL)
    g = C.printable(geo, FINAL["r_concave"], FINAL["r_convex"])
    # reading bottom -> top along the axis: rotate +90 deg, so reading x -> +z and the letter tops (+y) -> -s
    g = affinity.rotate(g, 90.0, origin=(0, 0))
    let = affinity.rotate(let, 90.0, origin=(0, 0))
    x0, y0, x1, _y1 = g.bounds
    ds, dz = -(x0 + x1) / 2.0, BOTTOM_PRINT_Z - y0
    g = shapely.set_precision(affinity.translate(g, ds, dz), PRECISION)
    let = affinity.translate(let, ds, dz)
    return g, let


def cad_set(g):
    """The CAD polygon set: the design set simplified (Douglas-Peucker, topology kept, original vertices only)."""
    return g.simplify(CAD_SIMPLIFY, preserve_topology=True)


def _nverts(g) -> int:
    return sum(len(q.exterior.coords) - 1 + sum(len(i.coords) - 1 for i in q.interiors)
               for q in (g.geoms if hasattr(g, "geoms") else [g]))


def document(g, let) -> dict:
    polys = list(g.geoms) if hasattr(g, "geoms") else [g]
    c = cad_set(g)
    cpolys = list(c.geoms) if hasattr(c, "geoms") else [c]
    if len(cpolys) != len(polys) or not c.is_valid:
        raise RuntimeError("the CAD simplification changed the island count or made an invalid polygon")
    return {
        "name": "YUCHAN - LAUNCH concept, vertical version, refined (user decisions 2026-11-04)",
        "generator": "cad/livery/make_logo.py (deterministic; glyphs.py + compose.py)",
        "units": "mm",
        "frame": "s_z: s = arc length on the avionics_module's outer skin (r = body_r) from the copy azimuth theta_c, "
                 "theta = theta_c + s / r (vehicle frame, from +X toward +Y); +s = RIGHT seen from outside with the nose "
                 "up (not mirrored) at every azimuth.  z = print z = vehicle z - the part's lower end "
                 "(Layout.z_avionics_bottom).  One polygon set for both copies and both configurations.",
        "reading": "Reads bottom -> top along the vehicle axis.  Seen beside the upright vehicle the letter tops point to "
                   "the LEFT (head tilted left), on both copies; laid down nose-right it reads normally, upright.",
        "colours": {"body": "Bambu PLA Matte Ivory White #FFFFFF (code 11100)",
                    "logo": "Bambu PLA Matte Mandarin Orange #F99963 (code 11300)"},
        "inlay": "flush 2-colour inlay on the OUTER wall line only (0.42 mm deep), 2 white lines behind it; no relief",
        "parameters": FINAL,
        "bottom_print_z_mm": BOTTOM_PRINT_Z,
        "precision_mm": PRECISION,
        "refine_changes": REFINE_NOTES,
        "summary_M": {"islands": len(polys), "orange_area_mm2_per_copy": round(g.area, 2),
                      "bbox_s_mm": [round(g.bounds[0], 4), round(g.bounds[2], 4)],
                      "print_z_mm": [round(g.bounds[1], 4), round(g.bounds[3], 4)],
                      "letters_print_z_mm": [round(let.bounds[1], 3), round(let.bounds[3], 3)],
                      "note": "measured on these polygons by this script; check_geometry's livery check re-measures "
                              "strokes / gaps / islands / clearances on the placed polygons"},
        "polygons_s_z": [mapping(q) for q in polys],
        "cad_set": {"simplify_mm": CAD_SIMPLIFY, "vertices_design": _nverts(g), "vertices_cad": _nverts(c),
                    "hausdorff_mm_M": round(float(shapely.hausdorff_distance(g, c, densify=0.05)), 5),
                    "area_change_mm2_M": round(c.area - g.area, 4),
                    "note": "cad/layout.py builds the inlay from polygons_s_z_cad (fewer faces for the booleans / BRep "
                            "distances; the design set's 32-segment corner arcs are below the slicer's resolution)"},
        "polygons_s_z_cad": [mapping(q) for q in cpolys],
    }


def svg(g) -> str:
    x0, y0, x1, y1 = g.bounds
    pad = 3.0
    w, h = (x1 - x0) + 2 * pad, (y1 - y0) + 2 * pad
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.2f}mm" height="{h:.2f}mm" '
           f'viewBox="{x0 - pad:.4f} {-(y1 + pad):.4f} {w:.4f} {h:.4f}">',
           '<!-- YUCHAN logo, unrolled outer skin (s right, print z up), cad/livery/make_logo.py -->',
           f'<rect x="{x0 - pad:.4f}" y="{-(y1 + pad):.4f}" width="{w:.4f}" height="{h:.4f}" fill="#FFFFFF"/>']
    for q in (g.geoms if hasattr(g, "geoms") else [g]):
        d = []
        for ring in [q.exterior] + list(q.interiors):
            pts = list(ring.coords)
            d.append("M" + " L".join(f"{x:.4f},{-y:.4f}" for x, y in pts) + " Z")
        out.append(f'<path fill="#F99963" fill-rule="evenodd" d="{" ".join(d)}"/>')
    out.append("</svg>")
    return "\n".join(out) + "\n"


def render() -> tuple[str, str]:
    g, let = geometry()
    return json.dumps(document(g, let), indent=1) + "\n", svg(g)


def main(argv: list[str]) -> int:
    js, sv = render()
    if "--check" in argv:
        ok = True
        for path, txt in ((OUT_JSON, js), (OUT_SVG, sv)):
            with open(path, "r", encoding="utf-8") as f:
                same = f.read() == txt
            print(f"{os.path.basename(path)}: {'identical' if same else 'DIFFERS'}")
            ok = ok and same
        return 0 if ok else 1
    for path, txt in ((OUT_JSON, js), (OUT_SVG, sv)):
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(txt)
        print(f"{path}: sha256 {hashlib.sha256(txt.encode('utf-8')).hexdigest()}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
