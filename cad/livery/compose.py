"""Composition of the refined LAUNCH logo (vertical version): slanted constructed caps YUCHAN + tapered ascent stripe +
exhaust-trail dashes.  Same drawing rules as the chosen concept (concept_launch/compose.py); only the parameters and the
printability radii differ (refine step 2026-11-04; the parameters live in make_logo.FINAL, the notes in REFINE_NOTES).
Byte-for-byte the refine step's compose_refined.py below this docstring; used only by cad/livery/make_logo.py.

Frame of build(): x = reading direction (toward the nose once applied), y = toward the letter tops, mm, unsheared parts
sheared together by the slant so every cut end carries the same forward slant.

printable(): closing with radius R_CONCAVE (every concave orange corner = every white wedge tip gets radius >= R_CONCAVE,
white gaps narrower than 2 * R_CONCAVE are filled), then opening with radius R_CONVEX (every convex orange corner gets
radius R_CONVEX, about the 0.4 mm nozzle's own radius, so the slicer outline equals the drawn outline).  The opening
cannot narrow a white gap (it only removes orange), and it keeps the concave radii (erode r, dilate r).
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

import glyphs as G

R_CONCAVE = 0.70   # white wedge / concave orange corner radius (mm); rule: >= 0.65
R_CONVEX = 0.35    # convex orange corner radius (mm); rule: 0.3 - 0.4
QS = 32            # quad segments of the buffers (deterministic)


def tapered(x0, x1, y_top, t0, t1):
    """Bar whose bottom edge descends linearly: thickness t0 at x0 -> t1 at x1, top edge flat at y_top."""
    return Polygon([(x0, y_top), (x1, y_top), (x1, y_top - t1), (x0, y_top - t0)])


def chamfer_acute(g, facet, max_angle=75.0):
    """Square chamfer of every convex corner sharper than max_angle (deg): the corner triangle is cut off by a line
    PERPENDICULAR to the corner's longer edge (the stroke edge), so that the new facet is `facet` mm long at every such
    corner (d0 = facet / tan(angle) along the stroke edge).  The two new corners are 90 deg and 90 + angle deg, so no
    acute corner is left and every terminal treated gets the same visible facet (Y arms, A foot)."""
    if facet <= 0:
        return g
    out = []
    for q in (g.geoms if hasattr(g, "geoms") else [g]):
        q = orient(q.simplify(1e-4, preserve_topology=True), 1.0)   # drop near-duplicate vertices first
        pts = np.asarray(q.exterior.coords)[:-1]
        n = len(pts)
        cuts = []
        for i in range(n):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % n]
            v1, v2 = a - b, c - b
            l1, l2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if l1 < 1e-9 or l2 < 1e-9:
                continue
            cross = v2[0] * v1[1] - v2[1] * v1[0]          # > 0: convex vertex of a CCW ring
            ang = math.degrees(math.atan2(abs(cross), float(v1 @ v2)))
            if cross <= 0 or ang >= max_angle:
                continue
            u = v1 / l1 if l1 >= l2 else v2 / l2            # unit vector along the longer (stroke) edge
            d0 = facet / math.tan(math.radians(ang))
            p0 = b + u * d0
            # half-plane behind the perpendicular line through p0, on the corner's side, clipped to a small disc
            far = 20.0
            nrm = np.array([-u[1], u[0]])
            tri = Polygon([p0 + nrm * far, p0 - nrm * far, p0 - nrm * far - u * far, p0 + nrm * far - u * far])
            cuts.append(tri.intersection(Point(b).buffer(3 * d0 + 1.0)))
        out.append(q.difference(unary_union(cuts)) if cuts else q)
    return unary_union(out)


def build(p: dict):
    """Returns (whole design, letters only, style, features); both geometries sheared, not yet made printable."""
    s = G.Style(H=p["H"], w=p["w"], wh=p["wh"], ro=p["ro"], gap=p["gap"], slant=p["slant"], wid=p.get("wid"),
                a_top=p["a_top"], a_bar=p["a_bar"], ears=p.get("ears", 0.0),
                ears_Y=p.get("ears_Y"), ears_A=p.get("ears_A"), y_term=p.get("y_term", "cap"))
    letters, pos, xend = G.word("YUCHAN", s, p.get("kern"))
    st = p["stripe"]
    y_top = -st["gap"]
    x0 = pos[0][0] + st["start"]
    x1 = xend + st["over"]
    parts = list(letters) + [tapered(x0, x1, y_top, st["t0"], st["t1"])]
    dashes = []
    x = x0
    for L, gg in zip(st["dashes"], st["dgap"]):
        x -= gg
        d = box(x - L, y_top - st["dash_t"], x, y_top)
        parts.append(d)
        dashes.append(d)
        x -= L
    feat = {"letters_x": [pos[0][0], xend], "stripe_x": [x0, x1], "tail_x": x, "glyph_x": pos}
    # letters: sheared, then acute corners chamfered square to their stroke (refine step; 0 = concept behaviour)
    let = chamfer_acute(G.shear(unary_union(letters), s.slant), p.get("chamfer", 0.0))
    rest = G.shear(unary_union(parts[len(letters):]), s.slant)
    geo = unary_union([let, rest])
    return geo, let, s, feat


def printable(geo, r_concave=R_CONCAVE, r_convex=R_CONVEX):
    g = geo.buffer(r_concave, join_style=1, quad_segs=QS).buffer(-r_concave, join_style=1, quad_segs=QS)
    g = g.buffer(-r_convex, join_style=1, quad_segs=QS).buffer(r_convex, join_style=1, quad_segs=QS)
    return g
