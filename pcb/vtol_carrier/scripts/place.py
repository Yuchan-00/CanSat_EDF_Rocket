"""Deterministic placement of the VTOL carrier rev 3 (writes kicad/placement.json).

Invariants
----------
* Pure function of ``spec/carrier_spec.json`` + the KiCad footprint libraries on disk + the archived rev 2c
  placement (KEEP_FROM, sha256 recorded in the output): no randomness, no time, fixed iteration order -> the same
  placement every run.
* Courtyards are the library F.CrtYd outlines (polygons, e.g. the cross-shaped TQFP-64 courtyard), rasterised
  conservatively on a 0.05 mm grid per board side; two courtyards on the same side never overlap (a placement
  that would is refused, never "fixed up" silently).
* Hard zones (spec board.keepouts / mounting / edges): 0.25 mm outline margin, the cable-tie band x -3..+3 mm
  (only the unpopulated programming footprints of common.IN_BAND), the ESP-12E antenna rule area (no parts on
  either side), the motor-lead EMI band (+x edge, |y| < 14, 12 mm deep: no crystal / I2C / radio parts), module
  bodies (only parts lower than the module's clearance underneath, never the reachable headers / jumpers).
* Each MCU (U1-U4) gets its support cluster at fixed MCU-local positions with pre-routed, locked copper
  (review F1 / F2): VCC / AVCC 100 nF and the crystal + load caps <= 3 mm copper from their pins.
* rev 2b: the 5th ATmega128A is removed (user decision 2026-11-05); the bottom side carries no parts at all.
* rev 2c (rev-2b review): J14 / J15 in the -x / +y corner next to battery spot A (user decision 2026-11-05), J13
  removed, J3 (ESP flash, 2x3) at the top of the tie band beside the ESP-12E, each MCU GND pin 22 / 53 / 63 with its
  own locked GND via inside the pad ring (review E-B2), ESP 100 nF / 10 uF 2.4 mm from pin 8 (review PF-m2).
* rev 3 (spec/REV3_PLAN.md section 12): the Pololu module J6 and its body are gone; the on-board boost converter
  (U8, L5, C57, C43, C41, FB divider, LDO U6 + C45) sits at explicit spots in the former Pololu corner (POWER_BLOCK,
  how = "power"), the cut-off comparator cluster and the remaining power parts are aimed at it (POWER_AUTO). No
  anchor moved. Every other greedy part first tries its rev 2c spot (how = "keep"): the rev 2c placement was routed
  to 0 open connections, so only the parts the new block displaces are placed afresh (how = "auto").
* Option B of the programming headers (spec prog_headers.option): the eight headers are fitted parts and are placed
  by PROG_B (outside the tie band); option A keeps them as bare holes in the band exactly as rev 2c.
* Every footprint is the spec package (common.FOOTPRINT_OVERRIDES is empty: the sourced LCSC parts fix the package).
* Everything else: greedy, nearest free spot (0.1 mm grid, rotations 0/90/180/270) to the pads it connects to.

Board frame: spec (x, y) with +y = CAD +y maps to KiCad (100 + x, 100 - y).
"""

from __future__ import annotations

import json
import math
import os
import re
import sys
from dataclasses import dataclass, field

from ai_eda.ir import BoardSide, LibraryRef, Placement
from ai_eda.tools.kicad import sexpr
from ai_eda.tools.kicad.geometry import pad_center, to_board
from ai_eda.tools.kicad.library import KicadLibrary

import common

CX, CY = 100.0, 100.0  # board centre in KiCad coordinates (= vehicle axis)
BX0, BX1, BY0, BY1 = 62.0, 138.0, 65.0, 135.0  # outline 76 x 70
TIE_X0, TIE_X1 = 97.0, 103.0  # cable-tie band x -3..+3 mm (spec board.mounting.tie_band)
EDGE_MARGIN = 0.25
CELL = 0.05
STEP = 0.1
TOP, BOT = "top", "bottom"

BROAD_NETS = {"GND", "+5V", "+3V3", "GND_PWR", "SERVO_5V"}


def spec_xy(x: float, y: float) -> tuple[float, float]:
    """KiCad -> spec frame."""
    return round(x - CX, 3), round(CY - y, 3)


def common_key(ref: str) -> tuple:
    m = re.match(r"([A-Z]+)(\d+)", ref)
    return (m.group(1), int(m.group(2))) if m else (ref, 0)


# --------------------------------------------------------------------------- geometry
@dataclass
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    def overlaps(self, o: "Rect") -> bool:
        return self.x0 < o.x1 and o.x0 < self.x1 and self.y0 < o.y1 and o.y0 < self.y1

    def as_list(self) -> list[float]:
        return [round(self.x0, 3), round(self.y0, 3), round(self.x1, 3), round(self.y1, 3)]


NX = int(round((BX1 - BX0) / CELL))
NY = int(round((BY1 - BY0) / CELL))


class Bitmap:
    """Board-sized bitmap (rows are Python ints, bit i = cell column i)."""

    def __init__(self) -> None:
        self.rows = [0] * NY

    def mark_rect(self, r: Rect) -> None:
        i0 = max(0, math.floor((r.x0 - BX0) / CELL + 1e-9))
        i1 = min(NX, math.ceil((r.x1 - BX0) / CELL - 1e-9))
        j0 = max(0, math.floor((r.y0 - BY0) / CELL + 1e-9))
        j1 = min(NY, math.ceil((r.y1 - BY0) / CELL - 1e-9))
        if i1 <= i0 or j1 <= j0:
            return
        m = ((1 << (i1 - i0)) - 1) << i0
        for j in range(j0, j1):
            self.rows[j] |= m

    def mark_shape(self, sh: "Shape", x: float, y: float) -> None:
        io, jo = _offsets(x, y)
        for dj, bits in sh.rows:
            j = jo + sh.jmin + dj
            if 0 <= j < NY:
                s = io + sh.imin
                self.rows[j] |= (bits << s) if s >= 0 else (bits >> -s)

    def hits(self, sh: "Shape", x: float, y: float, inner: bool = False) -> bool:
        """Conservative test; ``inner=True`` tests only cells fully inside the courtyard (a part that merely
        touches an already marked courtyard on an off-grid line does not count as overlapping)."""
        io, jo = _offsets(x, y)
        s = io + sh.imin
        for dj, bits in (sh.rows_in if inner else sh.rows):
            j = jo + sh.jmin + dj
            if not 0 <= j < NY or s < 0 or (bits << s) >> NX:
                return True  # outside the board bitmap counts as occupied
            if self.rows[j] & (bits << s):
                return True
        return False


def _offsets(x: float, y: float) -> tuple[int, int]:
    return math.floor((x - BX0) / CELL + 1e-6), math.floor((y - BY0) / CELL + 1e-6)


def _fraction(x: float, y: float) -> tuple[float, float]:
    io, jo = _offsets(x, y)
    return round(x - BX0 - io * CELL, 4), round(y - BY0 - jo * CELL, 4)


@dataclass
class Shape:
    """Conservative raster of a courtyard relative to the footprint origin (origin on the 0.05 mm grid)."""

    rows: list[tuple[int, int]]  # (row index from jmin, bits from imin): every cell the courtyard touches
    rows_in: list[tuple[int, int]]  # cells fully inside the courtyard
    imin: int
    jmin: int
    width: int
    height: int
    bbox: Rect  # exact courtyard bbox relative to the origin
    polys: list[list[tuple[float, float]]] = field(default_factory=list)

    def inside_board(self, x: float, y: float, margin: float) -> bool:
        return (x + self.bbox.x0 >= BX0 + margin - 1e-6 and x + self.bbox.x1 <= BX1 - margin + 1e-6
                and y + self.bbox.y0 >= BY0 + margin - 1e-6 and y + self.bbox.y1 <= BY1 - margin + 1e-6)


def _xy(node) -> tuple[float, float]:
    return sexpr.to_float(node[1]), sexpr.to_float(node[2])


def courtyard_segments(fp, layer: str = "F.CrtYd") -> list[tuple[tuple[float, float], tuple[float, float]]]:
    segs = []
    for item in fp.node:
        h = sexpr.head(item)
        if h not in ("fp_line", "fp_rect", "fp_poly", "fp_arc", "fp_circle"):
            continue
        lay = sexpr.find(item, "layer")
        if lay is None or str(lay[1]) != layer:
            continue
        if h == "fp_line":
            segs.append((_xy(sexpr.find(item, "start")), _xy(sexpr.find(item, "end"))))
        elif h == "fp_rect":
            (x0, y0), (x1, y1) = _xy(sexpr.find(item, "start")), _xy(sexpr.find(item, "end"))
            pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            segs += [(pts[k], pts[(k + 1) % 4]) for k in range(4)]
        elif h == "fp_poly":
            pts = [_xy(p) for p in sexpr.find_all(sexpr.find(item, "pts"), "xy")]
            segs += [(pts[k], pts[(k + 1) % len(pts)]) for k in range(len(pts))]
        elif h == "fp_arc":
            a, m, b = _xy(sexpr.find(item, "start")), _xy(sexpr.find(item, "mid")), _xy(sexpr.find(item, "end"))
            pts = _arc_points(a, m, b)
            segs += list(zip(pts, pts[1:]))
        elif h == "fp_circle":
            c, e = _xy(sexpr.find(item, "center")), _xy(sexpr.find(item, "end"))
            r = math.hypot(e[0] - c[0], e[1] - c[1]) / math.cos(math.pi / 16)  # circumscribed polygon
            pts = [(c[0] + r * math.cos(k * math.pi / 8), c[1] + r * math.sin(k * math.pi / 8)) for k in range(16)]
            segs += [(pts[k], pts[(k + 1) % 16]) for k in range(16)]
    return segs


def _arc_points(a, m, b, n: int = 12):
    ax, ay = a
    bx, by = m
    cx, cy = b
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return [a, b]
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    r = math.hypot(ax - ux, ay - uy) / math.cos(math.pi / (2 * n))
    t0, tm, t1 = (math.atan2(p[1] - uy, p[0] - ux) for p in (a, m, b))

    def norm(t):
        return (t - t0) % (2 * math.pi)

    sweep = norm(t1)
    if norm(tm) > sweep:
        sweep -= 2 * math.pi
    return [(ux + r * math.cos(t0 + sweep * k / n), uy + r * math.sin(t0 + sweep * k / n)) for k in range(n + 1)]


def make_shape(fp, rot: float, side: str, fx: float = 0.0, fy: float = 0.0) -> Shape:
    """Raster of the courtyard of ``fp`` placed at (grid point + (fx, fy)), relative to that grid point."""
    pl = Placement(component_ref="X", x_mm=0.0, y_mm=0.0, rotation_deg=rot, side=BoardSide.TOP if side == TOP else BoardSide.BOTTOM)
    segs = courtyard_segments(fp)
    if segs:
        bsegs = [(to_board(pl, *a), to_board(pl, *b)) for a, b in segs]
    else:  # e.g. NetTie: pads + 0.15 mm
        pts = []
        for p in fp.pads:
            pts += [(p.x - p.size_w / 2 - 0.15, p.y - p.size_h / 2 - 0.15), (p.x + p.size_w / 2 + 0.15, p.y + p.size_h / 2 + 0.15)]
        x0, y0 = min(q[0] for q in pts), min(q[1] for q in pts)
        x1, y1 = max(q[0] for q in pts), max(q[1] for q in pts)
        c = [to_board(pl, x, y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
        bsegs = [(c[k], c[(k + 1) % 4]) for k in range(4)]
    bsegs = [((a[0] + fx, a[1] + fy), (b[0] + fx, b[1] + fy)) for a, b in bsegs]
    xs = [p[0] for s in bsegs for p in s]
    ys = [p[1] for s in bsegs for p in s]
    bb = Rect(min(xs), min(ys), max(xs), max(ys))
    jmin = math.floor(bb.y0 / CELL + 1e-9)
    jmax = math.ceil(bb.y1 / CELL - 1e-9)
    imin = math.floor(bb.x0 / CELL + 1e-9)
    imax = math.ceil(bb.x1 / CELL - 1e-9)
    rows, rows_in = [], []
    odd = False

    def crossings(yy: float) -> list[tuple[float, float]] | None:
        xs_c = []
        for (xa, ya), (xb, yb) in bsegs:
            if ya == yb:
                continue
            lo, hi = (ya, yb) if ya < yb else (yb, ya)
            if lo <= yy < hi:
                xs_c.append(xa + (yy - ya) * (xb - xa) / (yb - ya))
        xs_c.sort()
        if len(xs_c) % 2:
            return None
        return [(xs_c[k], xs_c[k + 1]) for k in range(0, len(xs_c), 2)]

    for j in range(jmin, jmax):
        samples = [crossings(yy) for yy in (j * CELL + 1e-6, (j + 1) * CELL - 1e-6, (j + 0.5) * CELL)]
        if any(v is None for v in samples):
            odd = True
            break
        bits = 0
        for ivals in samples:
            for xa, xb in ivals:
                i0 = math.floor(xa / CELL + 1e-9) - imin
                i1 = math.ceil(xb / CELL - 1e-9) - imin
                if i1 > i0:
                    bits |= ((1 << (i1 - i0)) - 1) << i0
        if bits:
            rows.append((j - jmin, bits))
        inner = None
        for ivals in samples[:2]:
            m = 0
            for xa, xb in ivals:
                i0 = math.ceil(xa / CELL - 1e-9) - imin
                i1 = math.floor(xb / CELL + 1e-9) - imin
                if i1 > i0:
                    m |= ((1 << (i1 - i0)) - 1) << i0
            inner = m if inner is None else inner & m
        if inner:
            rows_in.append((j - jmin, inner))
    if odd or not rows:  # open outline: fall back to the bounding box
        full = (1 << (imax - imin)) - 1
        rows = [(j, full) for j in range(jmax - jmin)]
        rows_in = list(rows)
    bb = Rect(bb.x0 - fx, bb.y0 - fy, bb.x1 - fx, bb.y1 - fy)  # exact bbox relative to the true origin
    return Shape(rows=rows, rows_in=rows_in, imin=imin, jmin=jmin, width=imax - imin, height=jmax - jmin, bbox=bb)


# --------------------------------------------------------------------------- floorplan (decisions, each with its reason)
#: ESP-12E (U7): rotation 0 puts the footprint's antenna end (local y -12.12) at the +y board edge (KiCad y 65)
ESP_X, ESP_Y = 86.75, 77.2
ESP_KEEPOUT = (round(ESP_X - 8.12 - 4.0, 3), BY0, round(ESP_X + 8.12 + 4.0, 3), round(ESP_Y - 4.8, 3))
#: servo column (-x edge): right-angle 1x3 headers rotated 180 deg, pin tips at the board edge (pin 1 x 72.80);
#: slot pitch 8.62 mm (courtyards touching). J18 (6S sense) must sit at y < -12 (courtyard top y 112.1 > 112).
SERVO_X = 72.80
J18_Y = 121.49
_slot = [round(J18_Y - 9.39 - 1.77 - k * 8.62, 3) for k in range(5)]  # pin-1 y of J11, J10, J9, J8, J13

ANCHORS: dict[str, tuple[float, float, float, str]] = {
    # -x edge (spec board.edges): vane servos J8-J11 (the top slot stays free of right-angle plugs: its plug would hit
    # the bore corner, rev-1 review MF-2), 6S sense J18 below at y < -12
    "J8": (SERVO_X, _slot[3], 180, "edge -x: vane servo 1 (spec board.edges)"),
    "J9": (SERVO_X, _slot[2], 180, "edge -x: vane servo 2"),
    "J10": (SERVO_X, _slot[1], 180, "edge -x: vane servo 3"),
    "J11": (SERVO_X, _slot[0], 180, "edge -x: vane servo 4"),
    "J18": (SERVO_X, J18_Y, 180, "edge -x, y < -12 mm: 6S power-sense lead (courtyard top at y -12.1)"),
    # -y edge, x < -5: GPS lead, vertical JST-GH plugged from above (rev-1 review MF-6), in the -x/-y corner
    "J7": (66.98, 131.45, 0, "edge -y, x < -5 mm: GPS lead JST-GH vertical in the corner (top-cap lead hole at x -23.8, y -28.3)"),
    # ESP-12E: antenna end flush with the +y edge (spec esp12e.antenna), module x -21.4..-5.2 (spec A: -24..-8;
    # moved +2.7 mm so the 4 mm antenna rule area clears the servo column - placement decision)
    "U7": (ESP_X, ESP_Y, 0, "ESP-12E, antenna end at the +y edge (y +35), rule area 4 mm each side (spec esp12e.antenna)"),
    # +y edge, x > +5: arm switch lead, right-angle, pins toward +y
    "J16": (121.12, 75.79, 90, "edge +y, x > +5 mm: arm switch lead, right of the BMP280 socket (top cap switch hole at 90 deg)"),
    # -x / +y corner (rev 2c, user decision 2026-11-05 'battery A' = 6S pack -x face): logic battery (Molex SPOX
    # vertical, lead leaves upward - bend room, MF-4) in the corner, the power-switch lead (JST-PH vertical) below it
    # in the former J13 slot; both vertical: a right-angle plug here would hit the bore (MF-2). Courtyards x 62.25..
    # 70.65 / 69.15 stay 4 mm clear of the antenna rule area (x 74.63)
    "J14": (65.20, 68.85, 0, "-x/+y corner: 1S logic battery Molex SPOX vertical, 6 mm in plan from battery spot A (rev-2b review PF-M4 / MF2-1)"),
    "J15": (64.70, 73.45, 0, "-x/+y corner below J14: logic power-switch lead JST-PH vertical (rev-2b review MF2-1 / MF2-4)"),
    # +x edge, y > +14: ESC lead (upper slot, MF-3) and buzzer below it
    "J12": (127.21, 73.17, 0, "edge +x, y > +14 mm: ESC lead, upper slot (rev-1 review MF-3)"),
    "NT1": (124.25, 78.25, 0, "single GND <-> GND_PWR tie right next to J12 pin 3 (spec grounding.single_tie_point)"),
    "J17": (133.30, 82.35, 0, "edge +x, y > +14 mm: buzzer lead JST-PH vertical below the ESC lead"),
}

#: unpopulated programming footprints in the cable-tie band, top to bottom (ESP rule area ends at y 72.4)
BAND_ORDER = ["J3", "J24", "J20", "J25", "J21", "J26", "J22", "J19"]  # J3 (ESP flash) beside the ESP's right pad row


def band_anchors(fps: dict) -> dict[str, tuple[float, float, float, str]]:
    out = {}
    y = ESP_KEEPOUT[3] + 0.05  # top of the band stack: below the antenna rule area
    for ref in BAND_ORDER:
        bb = fps[ref].courtyard
        x0 = (TIE_X0 + TIE_X1) / 2 - (bb.x1 + bb.x2) / 2
        oy = y - bb.y1
        out[ref] = (round(round(x0 / CELL) * CELL, 3), round(math.ceil(oy / CELL - 1e-9) * CELL, 3), 0,
                    "programming-only footprint in the cable-tie band, unpopulated (common.IN_BAND)")
        y = out[ref][1] + bb.y2
    if y > BY1 - EDGE_MARGIN:
        raise ValueError(f"band stack ends at y {y:.2f} > {BY1 - EDGE_MARGIN}")
    return out


# MCU placement (centre x, centre y, rotation): clusters are fixed MCU-local, see CLUSTER
MCUS: dict[str, tuple[float, float, float, str]] = {
    "U3": (85.85, 103.45, 0, "COM under the ESP-12E (left column between the servo headers and the band)"),
    "U4": (85.85, 126.10, 270, "GNS in the -x/-y quadrant next to the GPS lead; crystal group in the corner gap between J18 and J7"),
    "U2": (112.80, 97.75, 0, "FCU right of the band; its top decoupling sits under the BMP280 module, the TQFP just below it"),
    "U1": (116.60, 121.00, 90, "NAV under the FCU (nlink SPI short); rotated so its crystal group points +x, under the GY-521 body"),
}

#: module outlines in the connector's footprint frame (A: typical module sizes, measure the bought ones)
#:   J1 PinHeader_1x08 vertical: pins along local +y 0..17.78; GY-521 21.2 x 16.4, header row 1.3 mm inside the
#:      long edge, body toward local +x, soldered flat on the header (2.54 mm spacer)
#:   J2 / J5 PinSocket_1x06 horizontal: opening at local x -10.03; the module lies flat beyond it (from x -10.55)
#:   rev 3: the Pololu module (J6) and its 2.0 mm under-module limit are gone (on-board boost converter, POWER_BLOCK)
MODULES = {
    "J1": ("GY-521 module ~21.2 x 16.4 mm (A)", (-1.3, 15.1, 8.89 - 10.6, 8.89 + 10.6), 2.0,
           "parts <= 2.0 mm high only (module on the 2.54 mm header spacer, A)"),
    "J2": ("BMP280 module ~15.2 x 11.5 mm (A)", (-10.55 - 11.5, -10.55, 6.35 - 7.6, 6.35 + 7.6), 1.1,
           "placement rule: parts <= 1.1 mm high under the module (module height above the board: spec/module_dimensions.json, A)"),
    "J5": ("OpenLog ~19 x 16 mm (A)", (-10.55 - 19.0, -10.55, 6.35 - 8.0, 6.35 + 8.0), 1.1,
           "placement rule: parts <= 1.1 mm high under the module (module height above the board: spec/module_dimensions.json, A)"),
}


def module_bodies(anch: dict) -> list[dict]:
    """Module outlines above the board: parts underneath must be lower than max_h; reachable parts never."""
    out = []
    for ref, (name, (x0, x1, y0, y1), max_h, rule) in MODULES.items():
        a = anch.get(ref)
        if not a:
            continue
        pl = Placement(component_ref=ref, x_mm=a[0], y_mm=a[1], rotation_deg=a[2], side=BoardSide.TOP)
        c = [to_board(pl, px, py) for px in (x0, x1) for py in (y0, y1)]
        out.append({"name": name, "ref": ref, "rect": [round(min(p[0] for p in c), 3), round(min(p[1] for p in c), 3),
                                                       round(max(p[0] for p in c), 3), round(max(p[1] for p in c), 3)],
                    "rule": rule, "max_h": max_h})
    return out


#: parts that must stay reachable (headers / shunts / jumpers / plugs) - never under a module body
#: rev 3: extra height-limited zone (KiCad rect, max part height, reason). The OpenLog outline built from
#: spec/module_dimensions.json (PCB length 19.05 mm F, hole-row offset A) ends at y 120.81, 2.26 mm beyond the rev 2c
#: MODULES["J5"] rectangle (y 118.55): no part taller than 1.1 mm and no reachable part is placed up to y 121.25
#: either (A: both outlines are assumptions until the bought module is measured; scripts/fit_checks.py has the numbers)
LOW_ZONES: list[tuple[tuple[float, float, float, float], float, str]] = [
    ((121.6, 118.55, 137.6, 121.25), 1.1, "OpenLog outline by spec/module_dimensions.json + 0.44 mm (A)"),
    # The module's own right-angle header has a 2.5 mm plastic bar between the socket opening and the module PCB; by
    # spec/module_dimensions.json (contact axis 1.25 mm A, plastic 2.5 mm A) it rests on the carrier (z 0..2.5 mm), so
    # nothing at all may stand in that strip (limit -1 = no part, fitted or not). Measured on the first rev 3
    # placement by scripts/fit_checks.py: the rev 2c spots of R29 / R43 / R45 were in the OpenLog strip.
    ((121.95, 99.55, 137.25, 101.55), -1.0, "OpenLog header plastic bar rests on the carrier (A)"),
    ((103.65, 77.55, 118.70, 79.55), -1.0, "BMP280 header plastic bar rests on the carrier (A)"),
    # BMP280 outline by spec/module_dimensions.json ends at y 91.23, 2.18 mm beyond the rev 2c MODULES["J2"] rectangle
    ((103.65, 89.05, 118.70, 91.25), 1.1, "BMP280 outline by spec/module_dimensions.json (A)"),
]

ACCESS = {"J15", "J5", "J2", "J1", "JP1", "J14", "J17", "J7", "J12", "J16", "C47", "D1", "D2", "D3", "D4"}  # LEDs stay visible

#: body height above the board per footprint (mm, A: typical package heights, not measured)
FP_HEIGHT = {
    "R_0402_1005Metric": 0.4, "C_0402_1005Metric": 0.55, "LED_0402_1005Metric": 0.5, "R_0603_1608Metric": 0.55,
    "C_0603_1608Metric": 0.9, "L_0603_1608Metric": 0.9, "LED_0603_1608Metric": 0.8, "C_0805_2012Metric": 1.35,
    "L_0805_2012Metric": 1.0, "C_1206_3216Metric": 1.8, "SOT-23": 1.1, "SOT-223-3_TabPin2": 1.8,
    "Crystal_SMD_3225-4Pin_3.2x2.5mm": 0.9, "D_SOD-123": 1.35, "TQFP-64_14x14mm_P0.8mm": 1.2,
    "NetTie-2_SMD_Pad0.5mm": 0.05, "SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm": 0.05,
    "CP_Elec_6.3x5.8": 5.8, "CP_Elec_6.3x5.4": 5.4, "CP_EIA-3216-18_Kemet-A": 1.8, "CP_EIA-3528-21_Kemet-B": 2.1, "ESP-12E": 3.2,
    # rev 3 power block: L5 2.0 mm = datasheet maximum (F, REV3_PLAN section 12), SOT-563 0.6 / SOT-23-6 1.45 nominal (E)
    "L_Changjiang_FXL0420": 2.0, "SOT-563": 0.6, "SOT-23-6": 1.45,
    # connectors: height of the mated plug / the fitted module above the board top (A, scout current_design.md: none
    # measured on a bought part) - used for the 13.4 mm limit only, connectors are anchors
    "Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical": 11.0, "JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical": 11.0,
    "JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical": 9.0, "PinHeader_1x02_P2.54mm_Horizontal": 3.0,
    "PinHeader_1x03_P2.54mm_Horizontal": 3.0, "PinHeader_1x04_P2.54mm_Horizontal": 3.0,
    "PinHeader_1x08_P2.54mm_Vertical": 8.6, "PinSocket_1x06_P2.54mm_Horizontal": 5.9,
    "PinHeader_2x03_P2.54mm_Vertical": 8.6, "PinHeader_1x02_P2.54mm_Vertical": 8.6,
    "PinHeader_2x03_P2.54mm_Vertical_SMD": 8.6, "PinHeader_1x02_P2.54mm_Vertical_SMD_Pin1Right": 8.6,
}
HEIGHT_LIMIT = 13.4  # mm above the board top (spec board.height_limit_top_mm)

# --------------------------------------------------------------------------- MCU support clusters (review F1 / F2)
# MCU-local coordinates (MCU at rotation 0, KiCad y down): TQFP-64 pads 17-32 at y +7.662, 49-64 at y -7.662,
# pitch 0.8 mm, pad ends at +-8.4 (Package_QFP:TQFP-64_14x14mm_P0.8mm on disk). Each part's courtyard touches the
# MCU courtyard, so its pads are ~0.4 mm from the pin ends:
#   dec21: 0402 100 nF, pad 1 (+5V) on the pin-21 (VCC) axis, pad 2 (GND) on the pin-22 (GND) axis
#   dec52: 0402 100 nF, pad 1 on pin 52 (VCC), pad 2 on pin 53 (GND)
#   avcc : 0402 100 nF, pad 1 on pin 64 (AVCC), pad 2 on pin 63 (GND)
#   xtal : 3225 crystal rotated 90 deg: pad 3 (XTAL2) under pin 23, pad 1 (XTAL1) diagonal; pin 24 runs down the
#          0.5 mm channel between the pad columns (0.15 mm each side)
#   cx1 / cx2: 0402 22 pF load caps beside crystal pads 1 / 3
CLUSTER: dict[str, tuple[float, float, float]] = {
    "dec21": (-2.32, 9.11, 0), "dec52": (3.12, -9.11, 180), "avcc": (-5.52, -9.11, 0),
    "xtal": (-0.40, 11.67, 90), "cx2": (-3.06, 10.57, 180), "cx1": (2.26, 12.77, 0),
}
#: FCU only (rev 2c; rev-2b review PF-B1: FCU_AVCC / FCU_AREF stayed open on rev 2b, and in the rev-2c routing tries
#: the last open connections were again AVCC / AREF of the FCU, whose top edge lies under the BMP280 module): the AREF
#: 100 nF (0603, rot 90: pad 1 toward the MCU) under pin 62 and the AVCC 10 uH (0805, rot 270: pad 2 = AVCC toward the
#: AVCC cap) under the AVCC cap, pre-routed and locked like the decoupling; the AREF cap's GND pad gets its own via.
#: Not for NAV / GNS / COM: there the same spots collide with J1 / J22 / the COM AVCC cap (M, place.py trial).
CLUSTER_EXTRA: dict[str, dict[str, tuple[float, float, float]]] = {
    "FCU": {"aref": (-4.20, -11.06, 90), "lavcc": (-6.00, -11.55, 270)},
}
#: pre-routed copper (net key, width mm, MCU-local polyline); "XTAL1" / "XTAL2" / "AVCC" get the block prefix
CLUSTER_ROUTES: list[tuple[str, float, list[tuple[float, float]]]] = [
    ("+5V", 0.3, [(-2.8, 7.662), (-2.8, 9.11)]),            # pin 21 -> dec21 pad 1
    ("GND", 0.25, [(-2.0, 7.662), (-2.0, 9.11)]),           # pin 22 -> dec21 pad 2
    ("+5V", 0.3, [(3.6, -7.662), (3.6, -9.11)]),            # pin 52 -> dec52 pad 1
    ("GND", 0.25, [(2.8, -7.662), (2.8, -9.11)]),           # pin 53 -> dec52 pad 2
    ("AVCC", 0.3, [(-6.0, -7.662), (-6.0, -9.11)]),         # pin 64 -> avcc pad 1
    ("GND", 0.25, [(-5.2, -7.662), (-5.2, -9.11)]),         # pin 63 -> avcc pad 2
    ("XTAL2", 0.2, [(-1.2, 7.662), (-1.2, 10.57)]),         # pin 23 -> crystal pad 3
    ("XTAL2", 0.2, [(-2.58, 10.57), (-1.25, 10.57)]),       # cx2 pad 1 -> crystal pad 3
    ("XTAL1", 0.2, [(-0.4, 7.662), (-0.4, 12.77), (1.78, 12.77)]),  # pin 24 -> crystal pad 1 -> cx1 pad 1
    # rev 2c (review E-B2: on rev 2b the F.Cu pour left pin 22 + its 100 nF, or pin 63 + the AVCC cap, as GND islands;
    # the fan-out found no via spot at those cap pads): each MCU GND pin gets its own via inside the pad ring, 0.73 mm
    # from the pad's inner end (via edge 0.6 mm from the neighbouring pads)
    ("GND", 0.3, [(-2.0, 7.0), (-2.0, 6.2)]),               # pin 22 -> own GND via
    ("GND", 0.3, [(2.8, -7.0), (2.8, -6.2)]),               # pin 53 -> own GND via
    ("GND", 0.3, [(-5.2, -7.0), (-5.2, -6.2)]),             # pin 63 -> own GND via
]
#: FCU-only pre-routes / vias for CLUSTER_EXTRA
CLUSTER_ROUTES_EXTRA: dict[str, list[tuple[str, float, list[tuple[float, float]]]]] = {
    "FCU": [("AREF", 0.25, [(-4.4, -7.662), (-4.4, -10.29)]),     # pin 62 -> AREF cap pad 1
            ("AVCC", 0.3, [(-6.0, -9.11), (-6.0, -10.6)]),        # AVCC cap pad 1 -> inductor pad 2
            ("GND", 0.3, [(-4.2, -11.84), (-4.2, -12.9)])],       # AREF cap pad 2 -> own GND via
}
#: locked GND vias of the clusters (net, MCU-local x, y), 0.6 / 0.3 mm through vias to the In1.Cu GND plane
CLUSTER_VIAS: list[tuple[str, float, float]] = [("GND", -2.0, 6.2), ("GND", 2.8, -6.2), ("GND", -5.2, -6.2)]
CLUSTER_VIAS_EXTRA: dict[str, list[tuple[str, float, float]]] = {"FCU": [("GND", -4.2, -12.9)]}
CLUSTER_BLOCKS = {"NAV": "U1", "FCU": "U2", "COM": "U3", "GNS": "U4"}

#: where to aim parts whose nets are all broad rails (ref -> (ref, pad))
TARGET_HINT = {
    # rev 3 power block (POWER_AUTO): the cut-off cluster aims at the comparator pins, the second input capacitor at
    # the switch-lead header (no 1206 spot is left at the L5 / VIN end - M, scripts/place.py run 2026-11-11)
    "C42": ("J15", "2"), "C44": ("U6", "3"), "R68": ("U8", "2"),
    "C47": ("J12", "2"), "C52": ("U7", "8"), "C53": ("U7", "8"), "C54": ("U7", "8"), "C56": ("J7", "1"),
    "D6": ("J17", "2"),
    # rev 2c: ESP GPIO0 / GPIO2 strap pull-ups and the Q11 LV pull-up aim at the ESP's lower right corner (the free
    # strip between the ESP and the COM): aimed at their pads they landed right of the tie band under the BMP280
    # module, the most congested area of rev 2b (review E-B1 / PF-B1)
    "R50": ("U7", "15"), "R51": ("U7", "15"), "R56": ("U7", "15"),
    # rev 2c: the 6S-sense divider TOP resistors (PF0 / PF1 / PF2) sit at the J18 sense header (-x / -y), so the long
    # run across the tie band is the divided ADC node (47k / 4.7k: ~4.3 kOhm source, E) and the 6S-level VS_* nets stay short
    # (M: aimed at the FCU they left VS_VIN / VS_VTETHER / VS_VBUFFER unrouted in 3 of 4 routing tries)
    "R32": ("J18", "1"), "R34": ("J18", "2"), "R36": ("J18", "3"),
    "C3": ("U1", "21"), "C11": ("U2", "21"), "C19": ("U3", "21"), "C27": ("U4", "21"),
    "L1": ("U1", "64"), "L2": ("U2", "64"), "L3": ("U3", "64"), "L4": ("U4", "64"),
}
#: fixed spots for parts whose position is part of a rule (ESP VCC decoupling at pin 8, spec layout_rules)
#: filled in Placer.esp_parts() relative to the ESP origin
#: courtyard gap LED / diode <-> crystal (rev 2c, rev 2b DRC silk_overlap Y2 / D2): 0.1 mm moves only D2 by 0.1 mm
#: (M: 0.3 mm moved D2 8.9 mm and re-shuffled the FCU area -> 5 connections open after routing instead of 1);
#: CARRIER_LED_XTAL_GAP overrides it for experiments
LED_XTAL_GAP = float(os.environ.get("CARRIER_LED_XTAL_GAP", "0.1"))
#: CARRIER_EXTRA_HINTS="Q8=J5.2,R41=J5.2" adds TARGET_HINT entries (experiments only)
for _h in filter(None, os.environ.get("CARRIER_EXTRA_HINTS", "").split(",")):
    _r, _t = _h.split("=")
    TARGET_HINT[_r] = tuple(_t.split("."))
EMI_SENSITIVE_PREFIX = ("Y",)
EMI_SENSITIVE = {"Q1", "Q2", "R42", "R43", "R44", "R45", "J2", "J1", "U7"}


class Placer:
    def __init__(self, spec: dict, library: KicadLibrary) -> None:
        self.spec = spec
        self.lib = library
        self.spec = spec
        self.parts = {p["refdes"]: p for p in spec["parts"]}
        self.ir = common.build_ir(spec, library)
        self.fp = {}
        for p in spec["parts"]:
            lib, name = common.split_lib_id(common.footprint_of(p))
            self.fp[p["refdes"]] = library.load_footprint(LibraryRef(library=lib, name=name))
        self.pin_net = {}
        self.net_pins: dict[str, list[tuple[str, str]]] = {}
        for n in self.ir.nets:
            for pr in n.pins:
                self.pin_net[(pr.component_ref, pr.pin_number)] = n.name
                self.net_pins.setdefault(n.name, []).append((pr.component_ref, pr.pin_number))
        self.placed: dict[str, dict] = {}
        self.shapes: dict[tuple[str, float, str], Shape] = {}
        self.occ = {TOP: Bitmap(), BOT: Bitmap()}
        self.problems: list[str] = []
        self.anchors = dict(ANCHORS)
        if common.prog_headers(spec) == "A":  # option B: the headers are fitted parts outside the band (PROG_B)
            self.anchors.update(band_anchors(self.fp))
        self.anchors.update(EXTRA_ANCHORS)
        self.bodies = module_bodies(self.anchors)
        # static zone bitmaps
        self.z_margin = Bitmap()
        for r in (Rect(BX0, BY0, BX1, BY0 + EDGE_MARGIN), Rect(BX0, BY1 - EDGE_MARGIN, BX1, BY1),
                  Rect(BX0, BY0, BX0 + EDGE_MARGIN, BY1), Rect(BX1 - EDGE_MARGIN, BY0, BX1, BY1)):
            self.z_margin.mark_rect(r)
        self.z_band = Bitmap()
        self.z_band.mark_rect(Rect(TIE_X0, BY0, TIE_X1, BY1))
        self.z_rule = Bitmap()
        self.z_rule.mark_rect(Rect(*ESP_KEEPOUT))
        self.z_emi = Bitmap()
        self.z_emi.mark_rect(Rect(BX1 - 12.0, CY - 14.0, BX1, CY + 14.0))
        self.z_body = []
        for b in self.bodies:
            bm = Bitmap()
            bm.mark_rect(Rect(*b["rect"]))
            self.z_body.append((b, bm))
        self.z_low = []
        for z in LOW_ZONES:
            bm = Bitmap()
            bm.mark_rect(Rect(*z[0]))
            self.z_low.append((z, bm))
        self.keep_from = keep_placements()
        self.z_tht = Bitmap()  # bottom side: THT pads of top parts (+0.35 mm)

    # --- geometry ------------------------------------------------------------------
    def shape(self, ref: str, rot: float, side: str, x: float = BX0, y: float = BY0) -> Shape:
        fx, fy = _fraction(x, y)
        key = (ref, float(rot) % 360, side, fx, fy)
        if key not in self.shapes:
            self.shapes[key] = make_shape(self.fp[ref], key[1], side, fx, fy)
        return self.shapes[key]

    def placement(self, ref: str) -> Placement:
        p = self.placed[ref]
        return Placement(component_ref=ref, x_mm=p["x"], y_mm=p["y"], rotation_deg=float(p["rot"]),
                         side=BoardSide.TOP if p["side"] == TOP else BoardSide.BOTTOM)

    def pad_xy(self, ref: str, number: str) -> tuple[float, float] | None:
        if ref not in self.placed:
            return None
        pad = self.fp[ref].pad(number)
        if pad is None:
            return None
        return pad_center(self.placement(ref), pad)

    def height(self, ref: str) -> float:
        if not common.populated(self.parts[ref]):
            return 0.0
        return FP_HEIGHT.get(self.fp[ref].name, 99.0)

    # --- placing ---------------------------------------------------------------------
    def put(self, ref: str, x: float, y: float, rot: float, side: str, how: str, why: str) -> None:
        x, y = round(x, 4), round(y, 4)
        sh = self.shape(ref, rot, side, x, y)
        if self.occ[side].hits(sh, x, y, inner=True):
            self.problems.append(f"courtyard overlap when placing {ref} ({how})")
        self.placed[ref] = {"x": x, "y": y, "rot": float(rot) % 360, "side": side, "how": how, "why": why}
        self.occ[side].mark_shape(sh, x, y)

    def allowed(self, ref: str, x: float, y: float, rot: float, side: str) -> bool:
        sh = self.shape(ref, rot, side, x, y)
        if not sh.inside_board(x, y, EDGE_MARGIN):
            return False
        if self.occ[side].hits(sh, x, y) or self.z_rule.hits(sh, x, y):
            return False
        if ref not in common.IN_BAND and self.z_band.hits(sh, x, y):
            return False
        if side == BOT:
            return not self.z_tht.hits(sh, x, y)
        if ref.startswith("D") and LED_XTAL_GAP > 0:
            # rev 2c (rev-2b review PF-m5): the LED / diode silkscreen touched the crystal's (Y2 / D2 silk_overlap):
            # keep LED_XTAL_GAP between their courtyards
            bb = sh.bbox
            g = LED_XTAL_GAP
            r0 = Rect(x + bb.x0 - g, y + bb.y0 - g, x + bb.x1 + g, y + bb.y1 + g)
            for o in self.placed:
                if o.startswith("Y") and r0.overlaps(courtyard_rect(self, o)):
                    return False
        if (ref.startswith(EMI_SENSITIVE_PREFIX) or ref in EMI_SENSITIVE) and self.z_emi.hits(sh, x, y):
            return False
        h = self.height(ref)
        for b, bm in self.z_body:
            if (ref in ACCESS or h > b["max_h"]) and bm.hits(sh, x, y):
                return False
        for (_rect, max_h, _why), bm in self.z_low:
            if (ref in ACCESS or h > max_h) and bm.hits(sh, x, y):
                return False
        return True

    def target(self, ref: str) -> tuple[float, float] | None:
        """Mean position of the placed pads this part connects to (signal nets first, broad nets as a fallback)."""
        if ref in TARGET_HINT:
            xy = self.pad_xy(*TARGET_HINT[ref])
            if xy is not None:
                return xy
        comp = self.ir.component(ref)
        pts = []
        for broad in (False, True):
            for pin in comp.pins:
                net = self.pin_net.get((ref, pin.number))
                if net is None or ((net in BROAD_NETS) != broad):
                    continue
                for oref, onum in self.net_pins[net]:
                    if oref == ref or (broad and len(self.net_pins[net]) > 12):
                        continue
                    xy = self.pad_xy(oref, onum)
                    if xy is not None:
                        pts.append(xy)
            if pts:
                break
        if not pts:
            m = re.search(r"pin (\d+)", self.parts[ref]["role"])
            mcu = {"NAV": "U1", "FCU": "U2", "COM": "U3", "GNS": "U4"}.get(self.parts[ref]["block"])
            if mcu:
                xy = self.pad_xy(mcu, m.group(1) if m else "21")
                if xy:
                    pts.append(xy)
        if not pts:
            return None
        return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))

    def auto(self, ref: str, side: str = TOP, max_r: float = 45.0) -> bool:
        t = self.target(ref)
        if t is None:
            self.problems.append(f"{ref}: no placed neighbour to aim at")
            return False
        tx, ty = round(t[0] / STEP) * STEP, round(t[1] / STEP) * STEP
        best = None
        n = int(max_r / STEP)
        rots = (0, 90, 180, 270)
        for k in range(0, n + 1):
            ring = []
            for i in range(-k, k + 1):
                for j in ((-k, k) if abs(i) != k else range(-k, k + 1)):
                    ring.append((i * i + j * j, i, j))
            ring.sort()
            for d2, i, j in ring:
                if best is not None and d2 > best[0]:
                    break
                x, y = round(tx + i * STEP, 3), round(ty + j * STEP, 3)
                for rot in rots:
                    if self.allowed(ref, x, y, rot, side):
                        cand = (d2, x, y, rot)
                        if best is None or cand < best:
                            best = cand
                        break
            if best is not None and (k + 1) ** 2 > best[0]:
                break
        if best is None:
            self.problems.append(f"{ref}: no free spot within {max_r} mm of {tuple(round(v, 2) for v in t)}")
            return False
        _, x, y, rot = best
        self.put(ref, x, y, rot, side, "auto", f"greedy: nearest free spot to its connected pads at {tuple(round(v, 2) for v in t)}")
        return True

    def cluster_members(self) -> dict[str, dict[str, str]]:
        """block -> {cluster key: refdes}, found from the spec roles / nets (never from refdes numbers)."""
        out: dict[str, dict[str, str]] = {}
        for block in CLUSTER_BLOCKS:
            m: dict[str, str] = {}
            for ref, p in sorted(self.parts.items()):
                if p["block"] != block:
                    continue
                role = p["role"]
                if role == f"{block} VCC pin 21 decoupling":
                    m["dec21"] = ref
                elif role == f"{block} VCC pin 52 decoupling":
                    m["dec52"] = ref
                elif role == f"{block} AVCC decoupling":
                    m["avcc"] = ref
                elif role == f"{block} AREF capacitor (internal 2.56 V reference)":
                    m["aref"] = ref
                elif role == f"{block} AVCC filter inductor":
                    m["lavcc"] = ref
                elif role == f"{block} crystal":
                    m["xtal"] = ref
                elif role == f"{block} crystal load cap":
                    m["cx1" if self.pin_net.get((ref, "1")) == f"{block}_XTAL1" else "cx2"] = ref
            for k in list(m):
                if k not in CLUSTER and k not in CLUSTER_EXTRA.get(block, {}):
                    del m[k]
            if set(m) != set(CLUSTER) | set(CLUSTER_EXTRA.get(block, {})):
                raise ValueError(f"{block}: cluster parts not found by role: {m}")
            out[block] = m
        return out

    def run(self) -> None:
        for ref, (x, y, rot, why) in self.anchors.items():
            self.put(ref, x, y, rot, TOP, "anchor", why)
        for ref, (x, y, rot, why) in MCUS.items():
            self.put(ref, x, y, rot, TOP, "anchor", why)
        self.clusters = self.cluster_members()
        for block, m in self.clusters.items():
            u = CLUSTER_BLOCKS[block]
            pl = self.placement(u)
            for key, (lx, ly, lrot) in {**CLUSTER, **CLUSTER_EXTRA.get(block, {})}.items():
                x, y = to_board(pl, lx, ly)
                self.put(m[key], x, y, (pl.rotation_deg + lrot) % 360, TOP, "cluster",
                         f"{block} support cluster '{key}' at fixed MCU-local ({lx}, {ly}) rot {lrot} (pre-routed, locked)")
            # pre-routed cluster vias outside the TQFP courtyard reserve their disk (+0.25 mm): auto-placed parts keep off
            for _n, lx, ly in CLUSTER_VIAS + CLUSTER_VIAS_EXTRA.get(block, []):
                x, y = to_board(pl, lx, ly)
                self.occ[TOP].mark_rect(Rect(x - 0.55, y - 0.55, x + 0.55, y + 0.55))
        for ref, (x, y, rot, why) in esp_parts(self).items():
            self.put(ref, x, y, rot, TOP, "esp", why)
        # rev 3: boost converter + LDO at explicit spots (each one must pass every placement rule - never forced)
        for ref, (x, y, rot, why) in {**POWER_BLOCK, **MOVED}.items():
            if not self.allowed(ref, x, y, rot, TOP):
                self.problems.append(f"{ref}: POWER_BLOCK spot ({x}, {y}, {rot}) breaks a placement rule")
            self.put(ref, x, y, rot, TOP, "power" if ref in POWER_BLOCK else "moved", why)
        if common.prog_headers(self.spec) == "B":
            for ref, (x, y, rot, why) in PROG_B.items():
                if not self.allowed(ref, x, y, rot, TOP):
                    self.problems.append(f"{ref}: PROG_B spot ({x}, {y}, {rot}) breaks a placement rule")
                self.put(ref, x, y, rot, TOP, "prog", why)
        order = ["D6"]  # buzzer flyback diode (1.35 mm): only the 2.0 mm module bodies take it - place it before the fill
        for block in ("COM", "GNS", "NAV", "FCU"):
            refs = [r for r, p in self.parts.items() if p["block"] == block and r not in self.placed]

            def rank(r: str) -> tuple:
                role = self.parts[r]["role"].lower()
                keys = ["avcc filter", "local bulk", "aref", "reset", "esp", "led", "level shifter"]
                for i, k in enumerate(keys):
                    if k in role:
                        return (i, common_key(r))
                return (len(keys), common_key(r))
            order += sorted(refs, key=rank)
        order += [r for r in PWR_FIRST if r not in self.placed and r not in order]
        for block in PRE_BLOCK_ORDER:
            order += sorted([r for r, p in self.parts.items() if p["block"] == block and r not in self.placed and r not in order],
                            key=common_key)
        order = [r for r in order if r not in POWER_AUTO]
        # rev 3 keep pass: a greedy part whose rev 2c spot is still legal stays there (same routing neighbourhood)
        rest = []
        for ref in order:
            k = self.keep_from.get(ref)
            if k and ref not in KEEP_EXCLUDE and k["side"] == TOP and self.allowed(ref, k["x"], k["y"], k["rot"], TOP):
                self.put(ref, k["x"], k["y"], k["rot"], TOP, "keep", "rev 2c spot kept (archive/rev2c_pololu_1s/kicad/placement.json)")
            else:
                rest.append(ref)
        for ref in [r for r in POWER_AUTO if r not in self.placed] + rest:
            self.auto(ref)

    def auto_spot(self, ref: str) -> bool:
        """Any free top-side spot (scan the board on a 0.5 mm grid) - used only by the CARRIER_U5_TOP experiment."""
        for rot in (0, 90):
            y = BY0
            while y < BY1:
                x = BX0
                while x < BX1:
                    if self.allowed(ref, x, y, rot, TOP):
                        self.put(ref, x, y, rot, TOP, "auto", "experiment: first free spot")
                        return True
                    x = round(x + 0.5, 3)
                y = round(y + 0.5, 3)
        self.problems.append(f"{ref}: no free top-side spot anywhere on the board")
        return False


#: blocks in placement order after the four MCU blocks
PRE_BLOCK_ORDER = ("PWR", "SHIFT", "LINK", "LOG", "IO")
#: power parts first, in this order (their targets chain: battery corner -> lead headers -> LDO)
PWR_FIRST = ["Q10", "R14", "C47"]

# --------------------------------------------------------------------------- rev 3 power block (REV3_PLAN section 12)
# Former Pololu corner (+x / -y). Library geometry (read from disk, rotation 0): SOT-563 pads 1 FB / 2 EN / 3 VIN at
# x -0.7125, 4 GND / 5 SW / 6 VOUT at x +0.7125 (y +0.5 / 0 / -0.5); L_Changjiang_FXL0420 pads at x -+1.85;
# C_1206 pads at x -+1.475. Reasons for the spots:
#   L5   rot 180 in the corner: pad 2 (BOOST_SW) at (133.05, 132.40), 1.5 mm right of U8 pin 5, pad 1 (VBAT) in the
#        far corner (fed from B.Cu / a pour - no F.Cu corridor is left, routing note)
#   U8   rot 0: hot pins 4 / 5 / 6 face L5, FB / EN / VIN face away from it
#   (the lines below describe the first rev 3 layout; the relayout step re-arranged U8 / C57 / C43 / C41 / R63 / R64 /
#    C58 - the reasons are in the POWER_BLOCK entries)
#   C57  first output capacitor directly above U8: pad 1 (+5V) 0.95 mm above pin 6, pad 2 (GND) to In1 by vias
#   C43  second output capacitor left of U8 in the edge strip (courtyard gap 0.6 mm = escape channel for pins 1-3)
#   C41  input capacitor left of C43 (the only 1206 spot near pin 3; none exists at the L5 pad-1 end)
#   R63 / R64 / C58: feedback parts on the side away from L5 and BOOST_SW, FB pads toward each other
#   U6   LDO above L5 (tab = +3V3 toward C45), body below the OpenLog outline (y >= 121.25)
#   U9   comparator in the edge strip under the NAV with C60 / R65 / R67 / R66 / C59 around it (UVLO_SENSE stays
#        inside x 113..117); the pull-down R68, the LDO input capacitor C44 and the second input capacitor C42 find
#        no spot in the corner and are placed greedily (POWER_AUTO)
POWER_BLOCK: dict[str, tuple[float, float, float, str]] = {
    # relayout step (rev 3 reviews E-R1 / E-R2 / E-R3): U8 above its input capacitor, the two output capacitors upright
    # between U8 and L5 (+5V pads up, GND pads down, BOOST_SW between their pads), feedback parts west of U8
    "L5": (134.90, 132.40, 180, "boost inductor in the +x/-y corner, pad 2 (BOOST_SW) toward U8"),
    "C43": (130.90, 132.40, 270, "boost output capacitor 2 upright beside L5: pad 1 (+5V) up, pad 2 (GND) down on the common F.Cu GND bar"),
    "C57": (128.60, 132.40, 270, "boost output capacitor 1 upright beside C43: pad 1 (+5V) 0.95 mm from U8 pin 6, pad 2 (GND) on the F.Cu GND bar with U8 pin 4 (hot loop on the top layer)"),
    "U8": (125.70, 131.50, 0, "boost converter: pins 4-6 (GND / SW / VOUT) toward C57 / L5, pins 1-3 away from them, pins 3 / 4 above the C41 pads"),
    "C41": (125.15, 133.60, 0, "boost input capacitor directly below U8: pad 1 (VBAT) under pin 3 (VIN), pad 2 (GND) under pin 4 on the F.Cu GND bar"),
    "U6": (134.05, 125.65, 270, "3.3 V LDO above L5, tab (+3V3) toward C45, body clear of the OpenLog outline"),
    "C45": (127.65, 128.10, 180, "LDO output tantalum between the NAV bulk capacitor and the boost block, pad 1 (+3V3) toward the U6 tab"),
    "R63": (123.00, 130.40, 0, "FB divider top resistor west of U8: pad 2 (BOOST_FB) 1.3 mm from U8 pin 1, pad 1 (+5V) sensed at C57 pad 1 by its own track"),
    "R64": (122.08, 133.10, 270, "FB divider bottom resistor upright below R63: pad 1 (BOOST_FB) up, pad 2 (GND) down"),
    "C58": (120.60, 133.10, 90, "feed-forward capacitor upright beside R64: pad 2 (BOOST_FB) up, level with R64 pad 1"),
    "U9": (117.80, 133.05, 0, "cut-off comparator in the edge strip under the NAV (BOOST_EN is low impedance)"),
    "C60": (115.00, 133.20, 90, "UVLO_SENSE filter capacitor beside U9 pin 3 (pad 1 down, level with pin 3)"),
    "R65": (113.50, 133.20, 90, "UVLO divider top resistor beside C60: pad 2 (UVLO_SENSE) up toward R67"),
    "R67": (113.90, 130.90, 0, "hysteresis resistor above R65 / C60 (0.1 mm below the former R46 spot, 1.0 mm clear of the NAV pad ring): pad 2 (BOOST_EN) toward U9 pin 1"),
    "R66": (116.90, 130.40, 0, "UVLO divider bottom resistor above U9: pad 1 (UVLO_SENSE) toward R67 / pin 3 side"),
    "C59": (119.90, 130.40, 0, "comparator bypass above U9: pad 1 (VBAT) 1.7 mm from U9 pin 6"),
    # the three greedy power parts keep the spots the greedy placer gave them on the delivered rev 3 board (their
    # targets moved with U8; a greedy re-run would move them and every part placed after them)
    "R68": (131.90, 120.50, 0, "BOOST_EN pull-down at its delivered rev 3 spot (first free spot found from the former U8 pin 2 position)"),
    "C44": (95.90, 131.90, 90, "LDO input capacitor at its delivered rev 3 spot"),
    "C42": (72.60, 69.30, 90, "second input capacitor at the switch-lead header J15, delivered rev 3 spot (no 1206 spot is left in the boost corner)"),
}
#: power parts placed greedily right after the keep pass, in this order (targets: TARGET_HINT)
POWER_AUTO: list[str] = []
#: greedy rev 2c parts the power block displaced, at explicit spots (how = "moved")
MOVED: dict[str, tuple[float, float, float, str]] = {
    "D1": (129.70, 125.50, 90, "NAV status LED moved 7 mm from its rev 2c spot (now C58) into the slot between C3 and U6: reachable, not under a module"),
}
#: greedy parts that must not go back to their rev 2c spot (the power block needs the room): NAV status LED D1
#: (spot taken by C58) and the Q3 pull-up R46 (spot taken by R67; Q3 itself is an unpopulated option)
KEEP_EXCLUDE: set[str] = {"D1", "R46"}
#: option B spots of the programming headers (ref -> x, y, rot, reason); empty = no legal set found (see
#: out/reports/programming_access.json) -> option B cannot be built and the spec must stay on option A
PROG_B: dict[str, tuple[float, float, float, str]] = {}
#: rev 2c placement the keep pass reads (archived, never rewritten)
KEEP_FROM = common.ROOT / "archive" / "rev2c_pololu_1s" / "kicad" / "placement.json"


def keep_placements() -> dict[str, dict]:
    """rev 2c greedy placements (how == 'auto') by refdes; CARRIER_NO_KEEP=1 switches the keep pass off (experiments)."""
    if os.environ.get("CARRIER_NO_KEEP") == "1" or not KEEP_FROM.is_file():
        return {}
    old = json.loads(KEEP_FROM.read_text(encoding="utf-8"))["placements"]
    return {r: p for r, p in old.items() if p["how"] == "auto"}

#: anchors that depend on decisions made while iterating (filled below)
EXTRA_ANCHORS: dict[str, tuple[float, float, float, str]] = {
    # BMP280 right-angle socket in the top middle, opening toward -y: the module lies over the FCU's top decoupling
    "J2": (104.83, 67.00, 90, "BMP280 socket top middle next to the band (outside the motor-lead EMI band), module lies toward -y over the FCU decoupling"),
    # OpenLog right-angle socket below the buzzer lead, opening toward -y: the logger lies over small parts at the +x edge
    "J5": (123.25, 89.00, 90, "OpenLog socket under the servo bulk cap, the logger lies flat toward -y over parts <= 1.1 mm (rev-1 review MF-1)"),
    # servo-rail 220 uF (5.4 mm tall, never under a module) right at the ESC BEC input J12 pin 2 (spec parts C47)
    "C47": (125.30, 83.62, 0, "servo-rail 220 uF between J12 (ESC BEC +5 V) and the OpenLog socket"),
    # GY-521 header along the -y edge, module body over the NAV (TQFP 1.2 mm < 2.0 mm clearance, A)
    "J1": (104.90, 115.10, 0, "GY-521 header vertical right of the tie band, module flat toward +x over the NAV"),
    # rev 3: the Pololu module header J6 is gone; the +x/-y corner carries POWER_BLOCK
}


def esp_parts(pl: Placer) -> dict[str, tuple[float, float, float, str]]:
    """ESP-12E VCC decoupling at pin 8 (spec esp12e.pins_used '8 VCC', layout_rules 'VCC caps at pin 8')."""
    out = {}
    p8 = pl.pad_xy("U7", "8")  # (ESP_X - 7.6, ESP_Y + 10.5)
    # 100 nF in the 3 mm gap between the servo column and the ESP pads, pad 1 level with pin 8
    # rotation 270: pad 1 (+3V3) level with pin 8, pad 2 (GND) below it with its own GND via (build_pcb.preroutes)
    # rev 2c: 2.4 mm from the pin-8 pad centre (rev 2b 3.55 mm > the 3 mm rule, review PF-m2 / E-m1); the 0402
    # courtyard (x +-0.47) stays 0.48 mm off the ESP courtyard (pin 8 pad - 1.45 mm)
    out["C54"] = (round(p8[0] - 2.4, 3), round(p8[1] + 0.48, 3), 270, "ESP VCC 100 nF beside pin 8, 2.4 mm (pre-routed, own GND via)")
    # 10 uF ceramic right above it in the same gap, clear of the pre-routed pin-8 track
    out["C53"] = (round(p8[0] - 2.6, 3), round(p8[1] - 2.2, 3), 270, "ESP VCC 10 uF in the gap beside pins 7-8 (pad 2 0.6 mm from the pin-8 track)")
    return out


def checks(pl: Placer) -> dict:
    out: dict = {"problems": list(pl.problems)}
    refs = sorted(pl.placed, key=common_key)
    # independent re-check of courtyard overlaps on each side with fresh bitmaps
    ov = []
    for side in (TOP, BOT):
        bm = Bitmap()
        for r in refs:
            p = pl.placed[r]
            if p["side"] != side:
                continue
            sh = pl.shape(r, p["rot"], side, p["x"], p["y"])
            if bm.hits(sh, p["x"], p["y"], inner=True):
                ov.append(r)
            bm.mark_shape(sh, p["x"], p["y"])
    out["courtyard_overlaps"] = ov
    out["outside_outline"] = [r for r in refs if not pl.shape(r, pl.placed[r]["rot"], pl.placed[r]["side"], pl.placed[r]["x"], pl.placed[r]["y"]).inside_board(pl.placed[r]["x"], pl.placed[r]["y"], -0.05)]
    out["in_tie_band"] = [r for r in refs if r not in common.IN_BAND and pl.z_band.hits(pl.shape(r, pl.placed[r]["rot"], pl.placed[r]["side"], pl.placed[r]["x"], pl.placed[r]["y"]), pl.placed[r]["x"], pl.placed[r]["y"], inner=True)]
    out["in_antenna_rule_area"] = [r for r in refs if r != "U7" and pl.z_rule.hits(pl.shape(r, pl.placed[r]["rot"], pl.placed[r]["side"], pl.placed[r]["x"], pl.placed[r]["y"]), pl.placed[r]["x"], pl.placed[r]["y"], inner=True)]
    out["unpopulated_footprints_in_tie_band"] = list(common.IN_BAND)
    cy = {r: courtyard_rect(pl, r) for r in refs}
    edges = {
        "-x": (["J8", "J9", "J10", "J11", "J18"], lambda r: r.x0 <= BX0 + 0.5),
        "-x,y<-12": (["J18"], lambda r: spec_xy(0, r.y0)[1] < -12),
        "+x,y>+14": (["J12", "J17"], lambda r: r.x1 >= BX1 - 0.5 and spec_xy(0, r.y1)[1] > 14.0),
        "-x/+y corner (x<-25, y>+18)": (["J14", "J15"], lambda r: spec_xy(r.x1, 0)[0] < -25.0 and spec_xy(0, r.y1)[1] > 18.0),
        "+y,x>+5": (["J16"], lambda r: r.y0 <= BY0 + 0.7 and spec_xy(r.x0, 0)[0] > 3.0),
        "-y,x<-5": (["J7"], lambda r: r.y1 >= BY1 - 0.5 and spec_xy(r.x1, 0)[0] < -5),
        "+y antenna edge": (["U7"], lambda r: r.y0 <= BY0 + 0.1),
    }
    out["edge_rules"] = {k: {ref: bool(fn(cy[ref])) for ref in refs_} for k, (refs_, fn) in edges.items()}
    out["esp_module_spec_x"] = [spec_xy(ESP_X - 8.12, 0)[0], spec_xy(ESP_X + 8.12, 0)[0]]
    out["antenna_rule_area_kicad"] = list(ESP_KEEPOUT)
    bodies = []
    for b, bm in pl.z_body:
        under = sorted([r for r in refs if pl.placed[r]["side"] == TOP and bm.hits(pl.shape(r, pl.placed[r]["rot"], TOP, pl.placed[r]["x"], pl.placed[r]["y"]), pl.placed[r]["x"], pl.placed[r]["y"], inner=True)], key=common_key)
        bad = [r for r in under if r in ACCESS or pl.height(r) > b["max_h"]]
        x0, y0, x1, y1 = b["rect"]
        bodies.append({**b, "rect": [round(v, 3) for v in b["rect"]], "parts_under": under, "too_tall_or_access_under": bad,
                       "inside_outline": x0 >= BX0 - 0.01 and x1 <= BX1 + 0.01 and y0 >= BY0 - 0.01 and y1 <= BY1 + 0.01,
                       "over_tie_band": x0 < TIE_X1 and x1 > TIE_X0})
    out["module_bodies"] = bodies
    area = {TOP: 0.0, BOT: 0.0}
    for r in refs:
        c = cy[r]
        area[pl.placed[r]["side"]] += (c.x1 - c.x0) * (c.y1 - c.y0)
    out["courtyard_bbox_area_mm2"] = {k: round(v, 1) for k, v in area.items()}
    out["board_area_minus_tie_band_mm2"] = round((BX1 - BX0) * (BY1 - BY0) - (TIE_X1 - TIE_X0) * (BY1 - BY0), 1)
    out["courtyard_occupancy_percent"] = round(100.0 * area[TOP] / out["board_area_minus_tie_band_mm2"], 1)
    out["rev3"] = rev3_report(pl, cy)
    return out


#: pads whose distance to the NAV crystal is compared with the rev 2c Pololu header pins (REV3_PLAN section 12):
#: the switching node, the converter pins and the hot-loop / input capacitors
SWITCHING_PARTS = ["U8", "L5", "C57", "C43", "C41"]
#: bulk capacitors of the four MCUs that are not cluster members (TARGET_HINT aims them at the MCU)
MCU_BULK = {"C3": "U1", "C11": "U2", "C19": "U3", "C27": "U4"}


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rev3_report(pl: Placer, cy: dict[str, Rect]) -> dict:
    """Hard checks of the rev 3 placement against the archived rev 2c placement (all numbers computed here, M).

    * heights: every fitted footprint has an FP_HEIGHT entry (the heights themselves are assumptions, A) and none
      exceeds HEIGHT_LIMIT; no part taller than a LOW_ZONES limit inside that zone.
    * fixed parts: every rev 2c anchor / cluster / ESP part still has the same x, y, rotation and side.
    * decoupling: centre distance of every MCU capacitor to the nearest MCU pad on its non-GND net, per MCU, is not
      worse than the worst rev 2c value of that MCU.
    * crystal distance (placement note, soft): pads of SWITCHING_PARTS vs the rev 2c J6 pins, measured to the Y1 pads
      and U1 pins 23 / 24.
    """
    out: dict = {}
    refs = sorted(pl.placed, key=common_key)
    fitted = [r for r in refs if common.populated(pl.parts[r])]
    hs = {r: pl.height(r) for r in fitted}
    out["heights"] = {
        "limit_mm": HEIGHT_LIMIT,
        "source": "FP_HEIGHT in scripts/place.py (A: body heights assumed per footprint, not measured parts)",
        "tallest": [[r, hs[r]] for r in sorted(fitted, key=lambda r: (-hs[r], common_key(r)))[:6]],
        "no_height_entry": [r for r in fitted if pl.fp[r].name not in FP_HEIGHT],
        "over_limit": [r for r in fitted if hs[r] > HEIGHT_LIMIT],
    }
    zones = []
    for (rect, max_h, why), bm in pl.z_low:
        inside = sorted([r for r in refs if pl.placed[r]["side"] == TOP and bm.hits(
            pl.shape(r, pl.placed[r]["rot"], TOP, pl.placed[r]["x"], pl.placed[r]["y"]), pl.placed[r]["x"], pl.placed[r]["y"], inner=True)],
            key=common_key)
        tall = [r for r in inside if r in ACCESS or pl.height(r) > max_h]
        # an anchored part (MCU / connector, position fixed by the plan) is reported, not failed: scripts/fit_checks.py
        # measures its real clearance to the module
        zones.append({"rect": list(rect), "max_h": max_h, "why": why, "parts_inside": inside,
                      "too_tall_or_access_inside": [r for r in tall if pl.placed[r]["how"] != "anchor"],
                      "anchored_parts_above_limit": [r for r in tall if pl.placed[r]["how"] == "anchor"]})
    out["low_zones"] = zones
    if not KEEP_FROM.is_file():
        out["rev2c"] = {"status": "NOT_VERIFIED", "why": f"{KEEP_FROM.name} not found in the archive"}
        return out
    old_all = json.loads(KEEP_FROM.read_text(encoding="utf-8"))
    old = old_all["placements"]
    opt_b = common.prog_headers(pl.spec) == "B"
    fixed = sorted([r for r, p in old.items() if p["how"] in ("anchor", "cluster", "esp") and not (opt_b and r in common.IN_BAND)], key=common_key)
    moved = [r for r in fixed if r in pl.placed and any(pl.placed[r][k] != old[r][k] for k in ("x", "y", "rot", "side"))]
    how = {}
    for r in refs:
        how[pl.placed[r]["how"]] = how.get(pl.placed[r]["how"], 0) + 1
    out["rev2c"] = {
        "file": "archive/rev2c_pololu_1s/kicad/placement.json", "sha256": common.sha256_file(KEEP_FROM),
        "removed_refs": sorted([r for r in old if r not in pl.placed], key=common_key),
        "new_refs": sorted([r for r in pl.placed if r not in old], key=common_key),
        "fixed_parts_compared": len(fixed), "fixed_parts_moved": moved,
        "crystals_unchanged": {r: pl.placed[r] == {**pl.placed[r], **{k: old[r][k] for k in ("x", "y", "rot", "side")}}
                               for r in refs if r.startswith("Y") and r in old},
        "how_counts": dict(sorted(how.items())),
        "greedy_parts_not_at_rev2c_spot": sorted([r for r in refs if r in old and old[r]["how"] == "auto" and any(
            pl.placed[r][k] != old[r][k] for k in ("x", "y", "rot", "side"))], key=common_key),
        "courtyard_bbox_area_mm2": old_all["placement_checks"]["courtyard_bbox_area_mm2"],
    }
    # decoupling distances (capacitor centre -> nearest MCU pad on the capacitor's non-GND net)
    dec: dict = {}
    for block, u in sorted(CLUSTER_BLOCKS.items(), key=lambda kv: kv[1]):
        rows = []
        caps = [r for r in refs if r.startswith("C") and r in old and r in pl.placed
                and ((old[r]["how"] == "cluster" and pl.parts[r]["block"] == block) or MCU_BULK.get(r) == u)]
        for c in caps:
            nets = {pl.pin_net.get((c, pin.number)) for pin in pl.ir.component(c).pins} - {None, "GND"}
            pads = [pl.pad_xy(u, num) for (ref, num) in sorted({(ref, num) for n in nets for (ref, num) in pl.net_pins[n]}) if ref == u]
            pads = [p for p in pads if p is not None]
            if not pads:
                continue
            now = min(_dist((pl.placed[c]["x"], pl.placed[c]["y"]), p) for p in pads)
            was = min(_dist((old[c]["x"], old[c]["y"]), p) for p in pads)
            rows.append({"ref": c, "net": sorted(nets), "rev2c_mm": round(was, 3), "rev3_mm": round(now, 3)})
        worst_old = max((r["rev2c_mm"] for r in rows), default=0.0)
        dec[u] = {"capacitors": rows, "worst_rev2c_mm": worst_old, "worst_rev3_mm": max((r["rev3_mm"] for r in rows), default=0.0),
                  "worse_than_rev2c_worst": [r["ref"] for r in rows if r["rev3_mm"] > worst_old + 1e-6]}
    out["decoupling"] = dec
    # distance of the switching parts to the NAV crystal vs the rev 2c Pololu header pins
    ref_pts = {f"Y1.{p.number}": pl.pad_xy("Y1", p.number) for p in pl.fp["Y1"].pads}
    ref_pts.update({"U1.23": pl.pad_xy("U1", "23"), "U1.24": pl.pad_xy("U1", "24")})
    ref_pts = {k: v for k, v in ref_pts.items() if v is not None}
    xt = {"reference_points": {k: [round(v[0], 3), round(v[1], 3)] for k, v in sorted(ref_pts.items())}}
    old_spec_path = KEEP_FROM.parent.parent / "spec" / "carrier_spec.json"
    j6_min = None
    if "J6" in old and old_spec_path.is_file():
        j6 = next((p for p in json.loads(old_spec_path.read_text(encoding="utf-8"))["parts"] if p["refdes"] == "J6"), None)
        if j6 is not None:
            lib, name = common.split_lib_id(common.footprint_of(j6))
            fp = pl.lib.load_footprint(LibraryRef(library=lib, name=name))
            plc = Placement(component_ref="J6", x_mm=old["J6"]["x"], y_mm=old["J6"]["y"], rotation_deg=float(old["J6"]["rot"]), side=BoardSide.TOP)
            pins = {p.number: pad_center(plc, p) for p in fp.pads}
            xt["rev2c_J6_pins"] = {k: [round(v[0], 3), round(v[1], 3)] for k, v in sorted(pins.items())}
            j6_min = {k: round(min(_dist(v, q) for v in pins.values()), 3) for k, q in sorted(ref_pts.items())}
            xt["rev2c_J6_min_mm"] = {"to_Y1_pads": min(v for k, v in j6_min.items() if k.startswith("Y1")),
                                     "to_U1_xtal_pins": min(v for k, v in j6_min.items() if k.startswith("U1"))}
    rows = {}
    for r in SWITCHING_PARTS:
        if r not in pl.placed:
            continue
        pads = [(p.number, pl.pad_xy(r, p.number)) for p in pl.fp[r].pads]
        dy = min((_dist(xy, q), f"{r}.{num} -> {k}") for num, xy in pads for k, q in ref_pts.items() if k.startswith("Y1"))
        du = min((_dist(xy, q), f"{r}.{num} -> {k}") for num, xy in pads for k, q in ref_pts.items() if k.startswith("U1"))
        rows[r] = {"to_Y1_pads_mm": round(dy[0], 3), "nearest_Y1": dy[1], "to_U1_xtal_pins_mm": round(du[0], 3), "nearest_U1": du[1]}
    xt["rev3_parts"] = rows
    sw = [pl.pad_xy("L5", "2"), pl.pad_xy("U8", "5")]
    if all(sw):
        xt["rev3_BOOST_SW_pads_min_mm"] = {
            "to_Y1_pads": round(min(_dist(a, q) for a in sw for k, q in ref_pts.items() if k.startswith("Y1")), 3),
            "to_U1_xtal_pins": round(min(_dist(a, q) for a in sw for k, q in ref_pts.items() if k.startswith("U1")), 3)}
    if j6_min is not None:
        lim = xt["rev2c_J6_min_mm"]
        xt["closer_than_rev2c_pins"] = [r for r, v in rows.items() if v["to_Y1_pads_mm"] < lim["to_Y1_pads"] - 1e-6
                                        or v["to_U1_xtal_pins_mm"] < lim["to_U1_xtal_pins"] - 1e-6]
        xt["placement_note_met"] = not xt["closer_than_rev2c_pins"]
    else:
        xt["placement_note_met"] = "NOT_VERIFIED"
    out["crystal_distance"] = xt
    hard = []
    if out["heights"]["no_height_entry"]:
        hard.append("fitted footprints without FP_HEIGHT entry: " + ", ".join(out["heights"]["no_height_entry"]))
    if out["heights"]["over_limit"]:
        hard.append("parts above the height limit: " + ", ".join(out["heights"]["over_limit"]))
    for z in zones:
        if z["too_tall_or_access_inside"]:
            hard.append("too tall / reachable parts in a low zone: " + ", ".join(z["too_tall_or_access_inside"]))
    if moved:
        hard.append("rev 2c anchor / cluster / ESP parts moved: " + ", ".join(moved))
    for u, d in dec.items():
        if d["worse_than_rev2c_worst"]:
            hard.append(f"{u} decoupling capacitors farther than the rev 2c worst: " + ", ".join(d["worse_than_rev2c_worst"]))
    out["hard_failures"] = hard
    return out


def courtyard_rect(pl: Placer, r: str) -> Rect:
    p = pl.placed[r]
    b = pl.shape(r, p["rot"], p["side"], p["x"], p["y"]).bbox
    return Rect(p["x"] + b.x0, p["y"] + b.y0, p["x"] + b.x1, p["y"] + b.y1)


def cluster_vias_board(placements: dict) -> list[dict]:
    """Locked GND vias of every clustered MCU in board coordinates (CLUSTER_VIAS)."""
    out = []
    for block, u in CLUSTER_BLOCKS.items():
        pu = placements[u]
        pl = Placement(component_ref=u, x_mm=pu["x"], y_mm=pu["y"], rotation_deg=float(pu["rot"]), side=BoardSide.TOP)
        for net, x, y in CLUSTER_VIAS + CLUSTER_VIAS_EXTRA.get(block, []):
            bx, by = to_board(pl, x, y)
            out.append({"net": net, "at": [round(bx, 4), round(by, 4)], "size": 0.6, "drill": 0.3, "block": block})
    return out


def cluster_routes_board(placements: dict) -> list[dict]:
    """Pre-routed segments in board coordinates for every clustered MCU (placements: ref -> {x, y, rot})."""
    segs = []
    for block, u in CLUSTER_BLOCKS.items():
        pu = placements[u]
        pl = Placement(component_ref=u, x_mm=pu["x"], y_mm=pu["y"], rotation_deg=float(pu["rot"]), side=BoardSide.TOP)
        for key, w, pts in CLUSTER_ROUTES + CLUSTER_ROUTES_EXTRA.get(block, []):
            net = key if key in ("+5V", "GND") else f"{block}_{key}"
            b = [to_board(pl, x, y) for x, y in pts]
            for a, c in zip(b, b[1:]):
                segs.append({"net": net, "width": w, "start": [round(a[0], 4), round(a[1], 4)],
                             "end": [round(c[0], 4), round(c[1], 4)], "block": block})
    return segs


def main() -> int:
    spec = common.load_spec()
    pl = Placer(spec, KicadLibrary())
    pl.run()
    chk = checks(pl)
    data = {
        "frame": "KiCad coordinates (mm, y down); board outline x 62..138, y 65..135; spec (x, y) = (X - 100, 100 - Y)",
        "generator": "scripts/place.py rev 3 (deterministic: anchored floorplan + MCU clusters + power block + rev 2c keep pass + greedy fill, polygon courtyards)",
        "prog_headers_option": common.prog_headers(spec),
        "spec_sha256": common.sha256_file(common.SPEC_PATH),
        "placements": {r: pl.placed[r] for r in sorted(pl.placed, key=common_key)},
        "courtyards": {r: courtyard_rect(pl, r).as_list() for r in sorted(pl.placed, key=common_key)},
        "clusters": {b: {"mcu": CLUSTER_BLOCKS[b], **m} for b, m in sorted(pl.clusters.items())},
        "cluster_routes": cluster_routes_board({r: pl.placed[r] for r in pl.placed}),
        "cluster_vias": cluster_vias_board({r: pl.placed[r] for r in pl.placed}),
        "antenna_rule_area": list(ESP_KEEPOUT),
        "part_heights_mm": {r: pl.height(r) for r in sorted(pl.placed, key=common_key)},  # A (FP_HEIGHT); 0.0 = not fitted
        "placement_checks": chk,
    }
    path = common.KICAD_DIR / "placement.json"
    if os.environ.get("CARRIER_PLACEMENT_OUT"):  # experiments write elsewhere (never the design file)
        from pathlib import Path

        path = Path(os.environ["CARRIER_PLACEMENT_OUT"])
    path.write_bytes((json.dumps(data, indent=1) + "\n").encode("utf-8"))
    print(json.dumps({k: v for k, v in chk.items() if k != "module_bodies"}, indent=1))
    for b in chk["module_bodies"]:
        print(b["name"], "under:", b["parts_under"], "BAD:", b["too_tall_or_access_under"], b["inside_outline"])
    unplaced = sorted(set(pl.parts) - set(pl.placed), key=common_key)
    print("placed", len(pl.placed), "unplaced", unplaced)
    print("rev3 hard failures:", chk["rev3"].get("hard_failures"))
    bad = bool(unplaced or chk["courtyard_overlaps"] or pl.problems or chk["outside_outline"] or chk["in_tie_band"]
               or chk["in_antenna_rule_area"] or chk["rev3"].get("hard_failures"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
