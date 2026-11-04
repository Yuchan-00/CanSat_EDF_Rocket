"""YUCHAN logo (livery, user decision 2026-11-04): constructed letterforms of the chosen "launch" concept (no font file).
Byte-for-byte the refine step's glyph code (concept_launch/glyphs.py + per-letter ear heights ears_Y / ears_A; the final
logo uses ears 0 and the square chamfer of compose.chamfer_acute instead).  Used only by cad/livery/make_logo.py.

Every glyph is built from straight-edged polygons and circular arcs in an upright frame
(x = reading direction, y = up, baseline y = 0, cap height H, units mm), then the whole word is
sheared forward by SLANT degrees.  The family is an extended, square-shouldered geometric sans:
U and C use rounded-rectangle bowls, A is a flat-topped trapezoid with a crossbar, Y has
straight diagonal arms, N a straight diagonal.  Deterministic: same parameters -> same polygons.
"""
from __future__ import annotations

import math
from shapely.geometry import Polygon, box, LineString
from shapely.ops import unary_union
from shapely import affinity

ARC_N = 24  # points per quarter arc


def rrect(x0, y0, x1, y1, r_bl=0.0, r_br=0.0, r_tr=0.0, r_tl=0.0):
    """Rectangle with per-corner radii (counter-clockwise outline)."""
    pts = []
    for (cx, cy, r, a0, dx, dy) in ((x0, y0, r_bl, 180, 1, 1), (x1, y0, r_br, 270, -1, 1),
                                    (x1, y1, r_tr, 0, -1, -1), (x0, y1, r_tl, 90, 1, -1)):
        if r <= 0:
            pts.append((cx, cy))
        else:
            ccx, ccy = cx + dx * r, cy + dy * r
            for k in range(ARC_N + 1):
                a = math.radians(a0 + 90.0 * k / ARC_N)
                pts.append((ccx + r * math.cos(a), ccy + r * math.sin(a)))
    return Polygon(pts)


class Style:
    def __init__(self, H=13.0, w=2.8, wh=2.4, ro=0.42, gap=2.6, slant=12.0, wid=None, a_top=0.30, a_bar=0.30, ears=0.0, ears_Y=None, ears_A=None, y_term="cap"):
        self.H, self.w, self.wh, self.gap, self.slant = H, w, wh, gap, slant
        self.a_top, self.a_bar = a_top, a_bar
        self.ears = ears
        # refine step: per-letter ear height (fraction of H); None = the common value
        self.ears_Y = ears if ears_Y is None else ears_Y
        self.ears_A = ears if ears_A is None else ears_A
        # refine step: Y arm terminal, "cap" = cut on the cap line (concept), "perp" = cut square to the arm
        self.y_term = y_term
        self.ro = ro * H                     # outer bowl radius
        self.wid = wid or {"Y": 1.12, "U": 1.04, "C": 1.0, "H": 1.04, "A": 1.14, "N": 1.06}


def _sk(g, slant):
    return affinity.skew(g, xs=slant, ys=0, origin=(0, 0))


def _round(g, r):
    return g.buffer(-r, quad_segs=24, join_style=1).buffer(r, quad_segs=24, join_style=1) if r > 0 else g


def g_U(s: Style):
    """Bowl built in the final (sheared) frame: rounded parallelogram minus its inward offset by w, so the stroke
    is exactly w (perpendicular) everywhere, also in the shear-thinned corner; open at the top."""
    H, w, W, ro = s.H, s.w, s.wid["U"] * s.H, s.ro
    outer = _round(_sk(box(0, 0, W, H + 3 * ro), s.slant), ro)
    ring = outer.difference(outer.buffer(-w, quad_segs=24, join_style=1))
    ring = ring.intersection(_sk(box(-5, 0, W + 5, H), s.slant))
    return _sk(ring, -s.slant), W


def g_C(s: Style):
    """Same construction as U; big radius on the left corners, small on the right; slot open to the right."""
    H, w, W, ro = s.H, s.w, s.wid["C"] * s.H, s.ro
    full = _sk(box(0, 0, W, H), s.slant)
    lh, rh = _sk(box(-5, -5, 0.5 * W, H + 5), s.slant), _sk(box(0.5 * W, -5, W + 5, H + 5), s.slant)
    outer = unary_union([_round(full, ro).intersection(lh), _round(full, 0.18 * ro).intersection(rh)])
    inner = outer.buffer(-w, quad_segs=24, join_style=1)
    slot = _sk(box(0.5 * W, w, W + 5, H - w), s.slant)
    c = outer.difference(unary_union([inner, slot]))
    return _sk(c, -s.slant), W


def g_H(s: Style):
    H, w, wh, W = s.H, s.w, s.wh, s.wid["H"] * s.H
    yb = 0.5 * H - 0.5 * wh
    g = unary_union([box(0, 0, w, H), box(W - w, 0, W, H), box(0, yb, W, yb + wh)])
    return g, W


def g_A(s: Style):
    """Flat-topped trapezoid A: legs of perpendicular width w, crossbar at a_bar * H.  With ears > 0 each leg
    stands on a short upright foot of height ears*H, so no foot corner is acute (the knee is obtuse)."""
    H, w, wh, W = s.H, s.w, s.wh, s.wid["A"] * s.H
    top_frac, bar_frac = s.a_top, s.a_bar
    e = s.ears_A * H
    a = 0.5 * W * (1 - top_frac)                 # top inset each side
    k = a / (H - e)                              # dx per dy on the slanted part of the left leg
    tt = math.tan(math.radians(s.slant))
    # horizontal leg thickness chosen so that the perpendicular width is w AFTER the word shear (shear keeps
    # horizontal widths): left leg leans k + tt, right leg leans k - tt from vertical in the final frame
    th_l = w / math.cos(math.atan(k + tt))
    # only the LEFT foot gets the upright ear (after the shear it is the acute one; the right leg ends up near
    # vertical and keeps a plain ~76 deg foot corner)
    kr = a / H
    th_r = w / math.cos(math.atan(abs(kr - tt)))
    outer = Polygon([(0, 0), (W, 0), (W - a, H), (a, H), (0, e)])
    yt = H - wh
    inner = Polygon([(th_l, -1), (W - th_r + kr, -1), (W - th_r - kr * yt, yt), (th_l + k * (yt - e), yt), (th_l, e)])
    yb = bar_frac * H
    bar = box(0, yb, W, yb + wh)
    g = outer.difference(inner).union(bar.intersection(outer))
    return g, W


def g_N(s: Style):
    H, w, W = s.H, s.w, s.wid["N"] * s.H
    # diagonal of perpendicular width ~ w: horizontal thickness th = w / cos(phi)
    phi = math.atan2(W - w, H)
    for _ in range(20):
        th = w / math.cos(phi)
        phi = math.atan2(W - th, H)
    diag = Polygon([(0, H), (th, H), (W, 0), (W - th, 0)])
    g = unary_union([box(0, 0, w, H), box(W - w, 0, W, H), diag])
    return g.intersection(box(0, 0, W, H)), W


def g_Y(s: Style, yj_frac=0.44):
    """Arms are drawn in the final (sheared) frame so both arms keep the perpendicular width w after the
    word shear; the result is un-sheared so that the global shear() puts it back.  With ears > 0 the outer arm
    edge reaches the glyph side at height H - ears*H and the tip above it is cut upright: no acute corner."""
    H, w, W = s.H, s.w, s.wid["Y"] * s.H
    t = math.tan(math.radians(s.slant))
    ey = s.ears_Y if isinstance(s.ears_Y, (tuple, list)) else (s.ears_Y, s.ears_Y)   # (left arm, right arm)
    yj = yj_frac * H
    hw = 0.5 * w
    jx = 0.5 * W + yj * t                      # junction centre, sheared frame
    arms = []
    for side in (-1, 1):
        e = (ey[0] if side < 0 else ey[1]) * H
        ye = H - e                             # height where this arm's outer edge meets the glyph side
        ox = (W + ye * t) if side > 0 else (ye * t)     # sheared side point the outer edge passes through
        qx = ox - side * hw
        for _ in range(60):
            phi = math.atan2(abs(qx - jx), ye - yj)
            qx = ox - side * hw / math.cos(phi)
        dx, dy = qx - jx, ye - yj
        L = math.hypot(dx, dy)
        ux, uy = dx / L, dy / L                 # unit vector junction -> tip
        far_up = (H + 2 - yj) / uy
        if s.y_term == "perp":
            # square-cut end: the arm keeps its direction, its higher (inner) end corner touches the cap line
            far_up = (H - yj - hw * abs(ux)) / uy
        arm = LineString([(jx - ux * 3 * w, yj - uy * 3 * w), (jx + ux * far_up, yj + uy * far_up)]).buffer(hw, cap_style=2)
        # keep the arm on its own side of the stem axis x = W/2 + y*t (sheared frame)
        far = 50.0
        half = Polygon([(0.5 * W - 20 * t, -20), (0.5 * W + (H + 20) * t, H + 20),
                        (0.5 * W + (H + 20) * t + side * far, H + 20), (0.5 * W - 20 * t + side * far, -20)])
        arms.append(arm.intersection(half))
    arms_up = affinity.skew(unary_union(arms), xs=-s.slant, ys=0, origin=(0, 0))
    stem = box(0.5 * W - 0.5 * w, 0, 0.5 * W + 0.5 * w, yj + 0.2 * w)
    g = unary_union([arms_up, stem]).intersection(box(0, 0, W, H))
    return g, W


GLYPH = {"U": g_U, "C": g_C, "H": g_H, "A": g_A, "N": g_N, "Y": g_Y}


def word(text: str, s: Style, kern=None):
    """Upright (unsheared) word geometry and the x positions of each glyph."""
    kern = kern or {}
    parts, x, pos = [], 0.0, []
    for i, ch in enumerate(text):
        g, W = GLYPH[ch](s)
        parts.append(affinity.translate(g, x, 0))
        pos.append((x, x + W))
        x += W
        if i < len(text) - 1:
            x += s.gap + kern.get(text[i:i + 2], 0.0)
    return parts, pos, x


def shear(g, slant_deg):
    return affinity.skew(g, xs=slant_deg, ys=0, origin=(0, 0))
