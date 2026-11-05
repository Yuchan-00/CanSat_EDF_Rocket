"""Deterministic placement of the VTOL carrier rev 2c (writes kicad/placement.json).

Invariants
----------
* Pure function of ``spec/carrier_spec.json`` + the KiCad footprint libraries on disk: no randomness, no time,
  fixed iteration order -> the same placement every run.
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
#:   J6 PinHeader_1x05 vertical: pins along local +y 0..10.16; Pololu S9V11F5S6CMA 12.7 x 15.3 mm (spec keep-outs,
#:      pololu.com/product/2870), pin row ~1.27 mm inside one 12.7 mm edge (A), body toward local -x, soldered flat on
#:      the header spacer (2.54 mm, A); its cut-off trimmer faces up (reachable from above)
MODULES = {
    "J1": ("GY-521 module ~21.2 x 16.4 mm (A)", (-1.3, 15.1, 8.89 - 10.6, 8.89 + 10.6), 2.0,
           "parts <= 2.0 mm high only (module on the 2.54 mm header spacer, A)"),
    "J2": ("BMP280 module ~15.2 x 11.5 mm (A)", (-10.55 - 11.5, -10.55, 6.35 - 7.6, 6.35 + 7.6), 1.1,
           "parts <= 1.1 mm high only (module ~1.5 mm above the board in a right-angle socket, A)"),
    "J5": ("OpenLog ~19 x 16 mm (A)", (-10.55 - 19.0, -10.55, 6.35 - 8.0, 6.35 + 8.0), 1.1,
           "parts <= 1.1 mm high only (module ~1.5 mm above the board in a right-angle socket, A)"),
    "J6": ("Pololu S9V11F5S6CMA 12.7 x 15.3 mm (vendor size, pin-row offset A)", (-15.3 + 1.27, 1.27, 5.08 - 6.35, 5.08 + 6.35), 2.0,
           "parts <= 2.0 mm high only (module on the 2.54 mm header spacer, A); trimmer on top, reachable"),
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
ACCESS = {"J15", "J5", "J2", "J1", "J6", "JP1", "J14", "J17", "J7", "J12", "J16", "C47", "D1", "D2", "D3", "D4"}  # LEDs stay visible

#: body height above the board per footprint (mm, A: typical package heights, not measured)
FP_HEIGHT = {
    "R_0402_1005Metric": 0.4, "C_0402_1005Metric": 0.55, "LED_0402_1005Metric": 0.5, "R_0603_1608Metric": 0.55,
    "C_0603_1608Metric": 0.9, "L_0603_1608Metric": 0.9, "LED_0603_1608Metric": 0.8, "C_0805_2012Metric": 1.35,
    "L_0805_2012Metric": 1.0, "C_1206_3216Metric": 1.8, "SOT-23": 1.1, "SOT-223-3_TabPin2": 1.8,
    "Crystal_SMD_3225-4Pin_3.2x2.5mm": 0.9, "D_SOD-123": 1.35, "TQFP-64_14x14mm_P0.8mm": 1.2,
    "NetTie-2_SMD_Pad0.5mm": 0.05, "SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm": 0.05,
    "CP_Elec_6.3x5.8": 5.8, "CP_Elec_6.3x5.4": 5.4, "CP_EIA-3216-18_Kemet-A": 1.8, "CP_EIA-3528-21_Kemet-B": 2.1, "ESP-12E": 3.2,
}

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
    "C41": ("J6", "3"), "C42": ("J6", "3"), "C43": ("J6", "1"), "C44": ("U6", "3"), "C45": ("U6", "2"),
    "C47": ("J12", "2"), "U6": ("J6", "1"), "C52": ("U7", "8"), "C53": ("U7", "8"), "C54": ("U7", "8"), "C56": ("J7", "1"),
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
        for ref in order:
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
PWR_FIRST = ["Q10", "R14", "U6", "C45", "C44", "C43", "C41", "C42", "C47"]
#: anchors that depend on decisions made while iterating (filled below)
EXTRA_ANCHORS: dict[str, tuple[float, float, float, str]] = {
    # BMP280 right-angle socket in the top middle, opening toward -y: the module lies over the FCU's top decoupling
    "J2": (104.83, 67.00, 90, "BMP280 socket top middle next to the band (outside the motor-lead EMI band), module lies toward -y over the FCU decoupling"),
    # OpenLog right-angle socket below the buzzer lead, opening toward -y: the logger lies over small parts at the +x edge
    "J5": (123.25, 89.00, 90, "OpenLog socket under the servo bulk cap, the logger lies flat toward -y over parts <= 1.1 mm (rev-1 review MF-1)"),
    # servo-rail 220 uF (5.4 mm tall, never under a module) right at the ESC BEC input J12 pin 2 (spec parts C47)
    "C47": (125.30, 83.62, 0, "servo-rail 220 uF between J12 (ESC BEC +5 V) and the OpenLog socket"),
    # GY-521 header along the -y edge, module body over the NAV (TQFP 1.2 mm < 2.0 mm clearance, A)
    "J1": (104.90, 115.10, 0, "GY-521 header vertical right of the tie band, module flat toward +x over the NAV (the 12.7 mm Pololu strip stays free at the +x edge)"),
    # tall power parts (not allowed under any module) packed in the +x/-y corner below the OpenLog outline
    # Pololu 5 V module in the +x/-y corner: header along the -y edge (pin 1 VOUT at +x), module flat toward +y
    # between the GY-521 body (x <= 124.4) and the OpenLog body (y <= 117.0); the only 12.7 x 15.3 mm spot left
    # (no module over another module, no tall / reachable part under it - checked below)
    "J6": (135.95, 132.90, 270, "Pololu 5 V module header in the +x/-y corner, module flat toward +y (trimmer on top)"),
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
        "generator": "scripts/place.py (deterministic: anchored floorplan + MCU clusters + greedy fill, polygon courtyards)",
        "spec_sha256": common.sha256_file(common.SPEC_PATH),
        "placements": {r: pl.placed[r] for r in sorted(pl.placed, key=common_key)},
        "courtyards": {r: courtyard_rect(pl, r).as_list() for r in sorted(pl.placed, key=common_key)},
        "clusters": {b: {"mcu": CLUSTER_BLOCKS[b], **m} for b, m in sorted(pl.clusters.items())},
        "cluster_routes": cluster_routes_board({r: pl.placed[r] for r in pl.placed}),
        "cluster_vias": cluster_vias_board({r: pl.placed[r] for r in pl.placed}),
        "antenna_rule_area": list(ESP_KEEPOUT),
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
    return 0 if not unplaced and not chk["courtyard_overlaps"] and not pl.problems else 1


if __name__ == "__main__":
    sys.exit(main())
