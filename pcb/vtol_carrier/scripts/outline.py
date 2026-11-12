"""The board outline of the VTOL carrier rev 3 - the ONE definition every generator and check reads.

Invariant: pure geometry, no dependencies (imported by the ai_eda scripts AND by the KiCad-python checks). The outline is
the rev 2c / rev 3 rectangle (KiCad x 62..138, y 65..135, corner radius 1.0 mm, unchanged) plus a TAB on the kicad +y
(bottom, vehicle -y) edge that carries the boost converter (redesign2 Layout stage, user request 2026-11-12 "보드의 사이즈를
조정하는 방법은 없어?"). The tab lies inside the envelope the Fit stage measured on the airframe CAD (redesign2/fit/
tab_clearance.json: x 106.0..134.0, outer edge y <= 143.74 with outer corner radius 1.0, inner fillets r 1.0).

Edges keep their meaning for the checks: the two long x edges (x 62 / x 138) and the y-min side (y 65) are unchanged;
the y-max side ('-y' in the spec frame, the side of the JLC rails) is the y 135 line beside the tab PLUS every tab
segment (fillets, tab sides, outer arcs, tab bottom). The y 135 line beside the tab is still THE '-y edge' for the
rules that name it (cable-tie band end, J7 GPS lead at the edge): MAIN_Y1 = 135.0 for those.
"""

from __future__ import annotations

import math

#: the unchanged main rectangle (KiCad mm) and its corner radius (spec board.outline_mm.corner_radius_mm, A)
MAIN_X0, MAIN_Y0, MAIN_X1, MAIN_Y1 = 62.0, 65.0, 138.0, 135.0
CORNER_R = 1.0
#: the tab on the y 135 edge: x TAB_X0..TAB_X1 (straight sides), bottom edge at TAB_Y1; outer corner radius
#: TAB_R_OUT, inner fillets TAB_R_IN where the tab meets the y 135 edge (the fillets add material beyond the sides).
TAB_X0, TAB_X1, TAB_Y1 = 117.3, 132.0, 142.2
TAB_R_OUT, TAB_R_IN = 1.0, 1.0
#: Fit-stage envelope the tab must stay inside (tab_clearance.json allowed_tab, label M)
FIT_ENVELOPE = {"x_min": 106.0, "x_max": 134.0, "y_edge_max_r1": 143.74}
#: bounding box of the whole outline (bitmaps, plots)
BBOX = (MAIN_X0, MAIN_Y0, MAIN_X1, TAB_Y1)
#: copper pull-back of the zones from the y-side edges (rail sides, routing_checks RAIL_EDGE_MM; unchanged 0.4 mm)
RAIL_PULLBACK = 0.4


def _check() -> None:
    if not (FIT_ENVELOPE["x_min"] <= TAB_X0 - TAB_R_IN and TAB_X1 + TAB_R_IN <= FIT_ENVELOPE["x_max"] + TAB_R_IN
            and TAB_X1 <= FIT_ENVELOPE["x_max"] and TAB_Y1 <= FIT_ENVELOPE["y_edge_max_r1"] and TAB_R_OUT >= 1.0):
        raise ValueError("outline tab outside the Fit-stage envelope")


_check()


def primitives() -> list[dict]:
    """Edge.Cuts primitives in drawing order (clockwise on screen): lines {'kind': 'line', 'a', 'b'} and arcs
    {'kind': 'arc', 'a', 'm', 'b', 'c', 'r'} (start, mid, end, centre, radius). Every primitive carries 'side': '+y'
    (y 65), '-y' (y 135 line beside the tab and every tab segment), '-x' (x 62), '+x' (x 138), 'corner' (the four
    unchanged corner arcs of the main rectangle)."""
    r, k = CORNER_R, CORNER_R * (1 - math.sqrt(0.5))
    x0, y0, x1, y1 = MAIN_X0, MAIN_Y0, MAIN_X1, MAIN_Y1
    ri, ro = TAB_R_IN, TAB_R_OUT
    ki, ko = ri * (1 - math.sqrt(0.5)), ro * (1 - math.sqrt(0.5))
    tx0, tx1, ty1 = TAB_X0, TAB_X1, TAB_Y1
    out = [
        {"kind": "line", "a": (x0 + r, y0), "b": (x1 - r, y0), "side": "+y"},
        {"kind": "arc", "a": (x1 - r, y0), "m": (x1 - k, y0 + k), "b": (x1, y0 + r), "c": (x1 - r, y0 + r), "r": r, "side": "corner"},
        {"kind": "line", "a": (x1, y0 + r), "b": (x1, y1 - r), "side": "+x"},
        {"kind": "arc", "a": (x1, y1 - r), "m": (x1 - k, y1 - k), "b": (x1 - r, y1), "c": (x1 - r, y1 - r), "r": r, "side": "corner"},
        # y 135 edge right of the tab, inner fillet (concave: material outside the circle), tab right side
        {"kind": "line", "a": (x1 - r, y1), "b": (tx1 + ri, y1), "side": "-y"},
        {"kind": "arc", "a": (tx1 + ri, y1), "m": (tx1 + ki, y1 + ri - (ri - ki)), "b": (tx1, y1 + ri), "c": (tx1 + ri, y1 + ri), "r": ri, "side": "-y"},
        {"kind": "line", "a": (tx1, y1 + ri), "b": (tx1, ty1 - ro), "side": "-y"},
        {"kind": "arc", "a": (tx1, ty1 - ro), "m": (tx1 - ko, ty1 - ko), "b": (tx1 - ro, ty1), "c": (tx1 - ro, ty1 - ro), "r": ro, "side": "-y"},
        {"kind": "line", "a": (tx1 - ro, ty1), "b": (tx0 + ro, ty1), "side": "-y"},
        {"kind": "arc", "a": (tx0 + ro, ty1), "m": (tx0 + ko, ty1 - ko), "b": (tx0, ty1 - ro), "c": (tx0 + ro, ty1 - ro), "r": ro, "side": "-y"},
        {"kind": "line", "a": (tx0, ty1 - ro), "b": (tx0, y1 + ri), "side": "-y"},
        {"kind": "arc", "a": (tx0, y1 + ri), "m": (tx0 - ki, y1 + ri - (ri - ki)), "b": (tx0 - ri, y1), "c": (tx0 - ri, y1 + ri), "r": ri, "side": "-y"},
        {"kind": "line", "a": (tx0 - ri, y1), "b": (x0 + r, y1), "side": "-y"},
        {"kind": "arc", "a": (x0 + r, y1), "m": (x0 + k, y1 - k), "b": (x0, y1 - r), "c": (x0 + r, y1 - r), "r": r, "side": "corner"},
        {"kind": "line", "a": (x0, y1 - r), "b": (x0, y0 + r), "side": "-x"},
        {"kind": "arc", "a": (x0, y0 + r), "m": (x0 + k, y0 + k), "b": (x0 + r, y0), "c": (x0 + r, y0 + r), "r": r, "side": "corner"},
    ]
    # the fillet mid points: on the circle, 45 deg toward the board corner they round
    for p in out:
        if p["kind"] == "arc":
            cx, cy = p["c"]
            ax, ay = p["a"]
            bx, by = p["b"]
            t0, t1 = math.atan2(ay - cy, ax - cx), math.atan2(by - cy, bx - cx)
            d = (t1 - t0 + math.pi) % (2 * math.pi) - math.pi
            tm = t0 + d / 2
            p["m"] = (cx + p["r"] * math.cos(tm), cy + p["r"] * math.sin(tm))
    return out


def _arc_points(p: dict, n: int) -> list[tuple[float, float]]:
    cx, cy = p["c"]
    t0 = math.atan2(p["a"][1] - cy, p["a"][0] - cx)
    t1 = math.atan2(p["b"][1] - cy, p["b"][0] - cx)
    d = (t1 - t0 + math.pi) % (2 * math.pi) - math.pi
    return [(cx + p["r"] * math.cos(t0 + d * i / n), cy + p["r"] * math.sin(t0 + d * i / n)) for i in range(n + 1)]


def polygon(n_arc: int = 16) -> list[tuple[float, float]]:
    """Outline as a closed polygon (arc points ON the arcs), clockwise on screen, no repeated closing point."""
    pts: list[tuple[float, float]] = []
    for p in primitives():
        seg = [p["a"], p["b"]] if p["kind"] == "line" else _arc_points(p, n_arc)
        for q in seg:
            if not pts or math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > 1e-9:
                pts.append(q)
    if math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-9:
        pts.pop()
    return pts


def area_mm2() -> float:
    pts = polygon(64)
    return abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1]))) / 2


def zone_polygon() -> list[tuple[float, float]]:
    """Copper-zone outline: the main rectangle pulled back RAIL_PULLBACK from y 65 / y 135 (x edges unchanged: the
    0.3 mm board rule applies through the fill) - exactly the rev 3 zone - plus the tab pulled back RAIL_PULLBACK from
    EVERY tab segment. Concave fillets use points outside the offset circle (chords never come nearer than 0.4 mm);
    convex outer corners use points on the offset circle (chords lie further inside)."""
    pb = RAIL_PULLBACK
    x0, y0, x1, y1 = MAIN_X0, MAIN_Y0 + pb, MAIN_X1, MAIN_Y1 - pb
    ri, ro = TAB_R_IN, TAB_R_OUT
    tx0, tx1, ty1 = TAB_X0, TAB_X1, TAB_Y1
    n = 16
    half = (math.pi / 2) / n / 2
    rin = (ri + pb) / math.cos(half) + 1e-4   # concave fillet: polygon vertices on a slightly larger circle
    rout = ro - pb                              # convex corner: vertices on the offset circle
    pts = [(x0, y0), (x1, y0), (x1, y1)]
    # right fillet: centre (tx1 + ri, y1_edge + ri); from angle -90 deg (toward the y 135 edge) to 180 deg (tab side)
    cxr, cyr = tx1 + ri, MAIN_Y1 + ri
    for i in range(n + 1):
        t = -math.pi / 2 - (math.pi / 2) * i / n
        pts.append((cxr + rin * math.cos(t), cyr + rin * math.sin(t)))
    # right outer corner: centre (tx1 - ro, ty1 - ro), from 0 to 90 deg
    cx, cy = tx1 - ro, ty1 - ro
    for i in range(n + 1):
        t = (math.pi / 2) * i / n
        pts.append((cx + rout * math.cos(t), cy + rout * math.sin(t)))
    # left outer corner: centre (tx0 + ro, ty1 - ro), from 90 to 180 deg
    cx = tx0 + ro
    for i in range(n + 1):
        t = math.pi / 2 + (math.pi / 2) * i / n
        pts.append((cx + rout * math.cos(t), cy + rout * math.sin(t)))
    # left fillet: centre (tx0 - ri, y1_edge + ri), from 0 deg (tab side) to -90 deg (the y 135 edge)
    cxl, cyl = tx0 - ri, MAIN_Y1 + ri
    for i in range(n + 1):
        t = -(math.pi / 2) * i / n
        pts.append((cxl + rin * math.cos(t), cyl + rin * math.sin(t)))
    pts.append((x0, y1))
    # clip the fillet points to the pulled-back y 135 line (the first / last fillet vertex lies 1e-4 beyond it)
    return [(round(x, 4), round(y1 if abs(y - y1) < 0.01 else y, 4)) for x, y in pts]


# ------------------------------------------------------------------------------------------------ distances (pure)
def _pt_seg(px: float, py: float, a, b) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _seg_seg(a, b, c, d) -> float:
    def ccw(p, q, r):
        return (r[1] - p[1]) * (q[0] - p[0]) - (q[1] - p[1]) * (r[0] - p[0])
    d1, d2, d3, d4 = ccw(c, d, a), ccw(c, d, b), ccw(a, b, c), ccw(a, b, d)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(_pt_seg(*a, c, d), _pt_seg(*b, c, d), _pt_seg(*c, a, b), _pt_seg(*d, a, b))


def _on_arc(p: dict, x: float, y: float) -> bool:
    cx, cy = p["c"]
    t = math.atan2(y - cy, x - cx)
    t0 = math.atan2(p["a"][1] - cy, p["a"][0] - cx)
    d = (math.atan2(p["b"][1] - cy, p["b"][0] - cx) - t0 + math.pi) % (2 * math.pi) - math.pi
    u = (t - t0 + math.pi) % (2 * math.pi) - math.pi
    return (0 <= u <= d) if d >= 0 else (d <= u <= 0)


def seg_prim_distance(a, b, p: dict) -> float:
    """Exact distance between segment a-b (a == b: a point) and an outline primitive."""
    if p["kind"] == "line":
        return _seg_seg(a, b, p["a"], p["b"]) if a != b else _pt_seg(*a, p["a"], p["b"])
    cx, cy = p["c"]
    best = min(math.hypot(q[0] - x, q[1] - y) for q in (p["a"], p["b"]) for (x, y) in ((a[0], a[1]), (b[0], b[1])))
    best = min(best, _pt_seg(*p["a"], a, b), _pt_seg(*p["b"], a, b))
    # nearest point of the segment to the centre -> radial distance where the direction hits the arc
    for (x, y) in (a, b, _closest_on_seg(cx, cy, a, b)):
        dc = math.hypot(x - cx, y - cy)
        if dc > 1e-12 and _on_arc(p, x, y):
            best = min(best, abs(dc - p["r"]))
    # segment crossing the circle inside the arc span
    for (x, y) in _seg_circle(a, b, (cx, cy), p["r"]):
        if _on_arc(p, x, y):
            return 0.0
    return best


def _closest_on_seg(px, py, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return (ax + t * dx, ay + t * dy)


def _seg_circle(a, b, c, r):
    ax, ay = a[0] - c[0], a[1] - c[1]
    dx, dy = b[0] - a[0], b[1] - a[1]
    A = dx * dx + dy * dy
    if A == 0:
        return []
    B = 2 * (ax * dx + ay * dy)
    C = ax * ax + ay * ay - r * r
    disc = B * B - 4 * A * C
    if disc < 0:
        return []
    out = []
    for s in (-1, 1):
        t = (-B + s * math.sqrt(disc)) / (2 * A)
        if 0 <= t <= 1:
            out.append((a[0] + t * dx, a[1] + t * dy))
    return out


def rect_prim_distance(rect, p: dict) -> float:
    """Distance from an axis-aligned rectangle (x0, y0, x1, y1) to a primitive (0 if they touch / cross; a primitive
    fully inside the rectangle also gives 0)."""
    x0, y0, x1, y1 = rect
    if p["kind"] == "line":
        ends = [p["a"], p["b"]]
    else:
        ends = _arc_points(p, 32)
    if any(x0 <= q[0] <= x1 and y0 <= q[1] <= y1 for q in ends):
        return 0.0
    edges = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    if x0 == x1 and y0 == y1:
        edges = [((x0, y0), (x0, y0))]
    return min(seg_prim_distance(a, b, p) for a, b in edges)


def inside(x: float, y: float) -> bool:
    """Point inside the outline polygon (ray casting on polygon(32))."""
    pts = _POLY
    c = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            c = not c
        j = i
    return c


_POLY = polygon(32)


def minus_y_distance(rect) -> float:
    """Distance of an axis-aligned box (x0, y0, x1, y1) to the '-y' side (spec frame; KiCad y 135 + tab).

    Identical to the rev 3 rule (``MAIN_Y1 - y1`` against the straight y 135 line) for a box that lies completely beside
    the tab and its fillets (x1 < TAB_X0 - TAB_R_IN or x0 > TAB_X1 + TAB_R_IN). Otherwise the y 135 line no longer runs
    there and the distance is the exact distance to every '-y' primitive (the y 135 pieces, fillets, tab sides, outer
    arcs, tab bottom); negative (minus the overhang) when a box corner lies outside the outline."""
    x0, y0, x1, y1 = rect
    if x1 < TAB_X0 - TAB_R_IN or x0 > TAB_X1 + TAB_R_IN:
        return MAIN_Y1 - y1
    d = min(rect_prim_distance(rect, p) for p in _MINUS_Y)
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    out = [(x, y) for x, y in corners if not inside(x, y)]
    if out:
        return -max(min(rect_prim_distance((x, y, x, y), p) for p in _ALL) for x, y in out)
    return d


_ALL = primitives()
_MINUS_Y = [p for p in _ALL if p["side"] == "-y"]
MINUS_Y_PRIMITIVES = _MINUS_Y


def rect_inside(rect, margin: float) -> bool:
    """Axis-aligned box inside the outline with ``margin`` to every edge. For a box above y 135 - margin this is the
    rev 3 rectangle test (x 62..138, y 65..135 with the margin; corner arcs ignored, as before). A box that reaches
    below y 135 - margin must lie inside the tab: x within TAB_X0 + margin .. TAB_X1 - margin, bottom <= TAB_Y1 -
    margin, and its corners inside the outer corner arcs by the margin (the fillets only add material, unused)."""
    x0, y0, x1, y1 = rect
    e = 1e-6
    if not (x0 >= MAIN_X0 + margin - e and x1 <= MAIN_X1 - margin + e and y0 >= MAIN_Y0 + margin - e):
        return False
    if y1 <= MAIN_Y1 - margin + e:
        return True
    if not (x0 >= TAB_X0 + margin - e and x1 <= TAB_X1 - margin + e and y1 <= TAB_Y1 - margin + e):
        return False
    cy = TAB_Y1 - TAB_R_OUT
    cl, cr = TAB_X0 + TAB_R_OUT, TAB_X1 - TAB_R_OUT
    if y1 > cy and x0 < cl and math.hypot(x0 - cl, y1 - cy) > TAB_R_OUT - margin + e:
        return False
    if y1 > cy and x1 > cr and math.hypot(x1 - cr, y1 - cy) > TAB_R_OUT - margin + e:
        return False
    return True


def describe() -> dict:
    return {"main_rect_kicad_mm": [MAIN_X0, MAIN_Y0, MAIN_X1, MAIN_Y1], "corner_radius_mm": CORNER_R,
            "tab_kicad_mm": {"x": [TAB_X0, TAB_X1], "y": [MAIN_Y1, TAB_Y1], "outer_corner_radius_mm": TAB_R_OUT,
                             "inner_fillet_radius_mm": TAB_R_IN,
                             "edge_cuts_material_x": [TAB_X0 - TAB_R_IN, TAB_X1 + TAB_R_IN]},
            "bbox_kicad_mm": list(BBOX), "area_mm2": round(area_mm2(), 1),
            "tab_area_mm2": round(area_mm2() - ((MAIN_X1 - MAIN_X0) * (MAIN_Y1 - MAIN_Y0) - (4 - math.pi) * CORNER_R ** 2), 1),
            "fit_envelope": FIT_ENVELOPE, "source": "scripts/outline.py (redesign2 Layout stage)"}
