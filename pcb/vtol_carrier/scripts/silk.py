"""Silkscreen text for the routed carrier (KiCad python, called from route.py; review F3, rev-2b review PF-M2 / MF2-9).

Invariants
----------
* Front first: every off-board connector gets its name on F.SilkS within 3 mm of its courtyard and, where the pins
  allow, one short pin label beside each pin, all labels of a connector on the same side of its pin row (aligned).
  The pin labels are derived from the pad nets of the routed board (PIN_GLYPH), never typed per connector.
* A text never touches a pad (+0.2 mm), a footprint silkscreen shape (+0.15 mm), another text (+0.15 mm), the board
  edge (0.5 mm), the BODY of another footprint (its F.Fab outline box) or a reserved module body (Dwgs.User
  rectangles: GY-521, BMP280, OpenLog, Pololu) - rev 2b put 7 front labels inside IC / module bodies where the parts
  hide them. Courtyard margins between parts (visible board) may carry text.
* A name that finds no front spot within 3 mm goes to B.SilkS (mirrored) under its connector and is listed; a pin
  label set that does not fit is left out (listed) - the name and the square pin-1 pad remain.
* A name that carries the pin glyphs and is printed along its pin row lists them as the pads lie on the side it is
  printed on (pin_name; a mirrored bottom text otherwise reads against the row - '1S BAT +-' put '+' on the GND
  side); a name printed across the row lists them from pin 1. routing_checks.py measures this on the board.
* Sizes: names 1.0 mm high / 0.15 mm stroke (JLC recommendation, A), 0.8 mm (board minimum) when 1.0 does not fit;
  pin labels 0.8 mm. Deterministic: fixed tables, fixed candidate order, no randomness.
* silk-id step (user request): the two identification lines ID_LINES (repository address, contact e-mail) are placed
  LAST, so no earlier text moves. They never lie on a pad, a hole, a via (tented or not), another silkscreen item,
  the 0.5 mm edge strip or - if avoidable - the cable-tie band; the front is used only for a clearly free spot
  outside every part body and module body, else the bottom (mirrored). The strings exist only here and on the
  board: the log and the reports name the lines by kind and character count, never by content.
"""

from __future__ import annotations

import math

import numpy as np
import pcbnew

BX0, BY0, BX1, BY1 = 62.0, 65.0, 138.0, 135.0
CELL = 0.05
T = 0.15
NAME_H = (1.0, 0.8)
PIN_H = 0.8
MAX_D = 3.0

#: connector / jumper / module names (sources: spec parts / nets)
LABELS: dict[str, str] = {
    "J8": "V1", "J9": "V2", "J10": "V3", "J11": "V4",
    "J12": "ESC", "J17": "BUZ", "J14": "1S BAT", "J18": "6S NO GND",
    "J7": "GPS", "J15": "PWR SW", "J16": "ARM SW", "J24": "NAV RST", "J25": "FCU RST", "J26": "COM RST",
    "J19": "ISP NAV", "J20": "ISP FCU", "J21": "ISP COM", "J22": "ISP GNS",
    "J1": "GY-521", "J2": "BMP280", "J3": "ESP 3V3", "J5": "OPENLOG",
    "JP1": "GPS 3V3|5V", "U7": "ESP-12F",
}
#: placement priority of the names (front space is scarce: the off-board leads that are plugged in blind first)
PRIORITY = ("J14", "J15", "J18", "J12", "J8", "J9", "J10", "J11", "J7", "J16", "J17", "JP1", "J3")
#: connectors whose name carries the pin order (off-board leads + the ESP flash header)
#: rev 3: + the three sensor module sockets (the user checks the module's printed pin order against the board)
PIN_LABELLED = ("J8", "J9", "J10", "J11", "J12", "J17", "J14", "J18", "J7", "J16", "J3", "J1", "J2", "J5")
#: rev 3 (plan section 12): board revision text, F.SilkS
REV_TEXT = "rev 3"
#: rev 3: reference designators that must be readable on F.SilkS (the boost block) - placed before all others
REFS_FIRST = ("L5", "U8", "U9")
#: rev 3: polarity / pin-1 marks added on F.SilkS beside pad 1, outside every part body (the library marks stay):
#: "+" = positive pad of a polarised capacitor (pad 1 of the CP_* footprints), "K" = cathode (pad 1 of the LED_* /
#: D_* footprints), "dot" = filled dot at pin 1
MARKS: dict[str, str] = {
    "C45": "+", "C52": "+", "C47": "+",
    "D1": "K", "D2": "K", "D3": "K", "D4": "K", "D6": "K",
    **{f"Q{i}": "dot" for i in range(1, 12)},
    "U6": "dot", "U7": "dot", "U8": "dot", "U9": "dot", "J7": "dot", "U1": "dot", "U2": "dot", "U3": "dot", "U4": "dot",
}
MARK_MAX_D = 1.6
REF_FIRST_MAX_D = 2.5
DOT_R = 0.2
#: silk-id step (user request): the two identification lines, exactly these strings, line 1 above line 2
ID_LINES: tuple[tuple[str, str], ...] = (
    ("repository address", "github.com/Yuchan-00/CanSat_EDF_Rocket"),
    ("contact e-mail", "yuchan.fbu@gmail.com"),
)
#: (height, stroke) mm, first that fits. F: https://jlcpcb.com/capabilities/pcb-capabilities, section Legend, read
#: 2026-11-11: line width >= 0.15 mm, text height >= 1.0 mm, preferred stroke : height 1 : 6, pad to silkscreen
#: 0.15 mm. 1.2 / 0.2 is the 1 : 6 ratio with margin over both minima; 1.0 / 0.15 is the minimum itself
ID_SIZES = ((1.2, 0.2), (1.0, 0.15))
ID_GAP = 0.4  # mm between the ink boxes of the two lines
#: clearance tried around the ink boxes on top of the obstacle growth (pads + 0.2, silkscreen + 0.15, vias + 0.1),
#: largest first: the block lands where it has the most room
ID_MARGINS = tuple(round(1.0 - 0.05 * i, 2) for i in range(18))  # 1.0 .. 0.15
ID_FRONT_MARGIN = 0.5  # the front is used only for a clearly free spot
#: cable-tie band (plan: x -3 .. +3 mm about the board centre line x 100), both sides of the board
TIE_BAND = (97.0, 103.0)


def pin_glyph(net: str, ref: str = "") -> str | None:
    """Short pin label from the pad's net name (routed board = spec nets)."""
    if ref in ("J14", "J17") and net in ("GND", "+5V"):
        return "-" if net == "GND" else "+"
    if net.endswith("_SIG"):
        return "S"
    table = {"SERVO_5V": "+", "GND_PWR": "-", "GND": "G", "+5V": "5V", "VBAT_LOGIC_RAW": "+", "BUZ_N": "-",
             "VS_VIN": "VI", "VS_VTETHER": "VT", "VS_VBUFFER": "VB", "VS_BUF_ISIGN": "IS", "ARM_SW_EXT": "A",
             "GPS_VCC": "V", "ESP_GPIO0": "IO0", "ESP_TXD0": "TX", "ESP_RXD0": "RX", "ESP_RST": "RST",
             "VBAT_LOGIC_SW": "VI", "+3V3": "3V", "I2C_SCL_3V3": "SCL", "I2C_SDA_3V3": "SDA", "IMU_INT_3V3": "INT",
             "OPENLOG_RXI": "RXI"}
    if net in table:
        return table[net]
    if net.startswith("GPS_TXD"):
        return "TX"
    if net.startswith("GPS_RXD"):
        return "RX"
    if net.startswith("unconnected-"):
        return None
    return None


class Bitmap:
    def __init__(self) -> None:
        self.nx = int(round((BX1 - BX0) / CELL))
        self.ny = int(round((BY1 - BY0) / CELL))
        self.rows = [0] * self.ny

    def _span(self, x0: float, y0: float, x1: float, y1: float):
        i0 = max(0, int(math.floor((x0 - BX0) / CELL)))
        i1 = min(self.nx, int(math.ceil((x1 - BX0) / CELL)))
        j0 = max(0, int(math.floor((y0 - BY0) / CELL)))
        j1 = min(self.ny, int(math.ceil((y1 - BY0) / CELL)))
        return i0, i1, j0, j1

    def mark(self, box) -> None:
        i0, i1, j0, j1 = self._span(*box)
        if i1 <= i0 or j1 <= j0:
            return
        m = ((1 << (i1 - i0)) - 1) << i0
        for j in range(j0, j1):
            self.rows[j] |= m

    def free(self, box) -> bool:
        x0, y0, x1, y1 = box
        if x0 < BX0 + 0.5 or y0 < BY0 + 0.5 or x1 > BX1 - 0.5 or y1 > BY1 - 0.5:
            return False
        i0, i1, j0, j1 = self._span(*box)
        m = ((1 << (i1 - i0)) - 1) << i0
        return all(not (self.rows[j] & m) for j in range(j0, j1))


def _mm_box(bb, grow: float):
    return (pcbnew.ToMM(bb.GetLeft()) - grow, pcbnew.ToMM(bb.GetTop()) - grow,
            pcbnew.ToMM(bb.GetRight()) + grow, pcbnew.ToMM(bb.GetBottom()) + grow)


def _obstacles(board, side: str = "F") -> Bitmap:
    bm = Bitmap()
    silk = pcbnew.F_SilkS if side == "F" else pcbnew.B_SilkS
    cu = pcbnew.F_Cu if side == "F" else pcbnew.B_Cu
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.IsOnLayer(cu):
                bm.mark(_mm_box(pad.GetBoundingBox(), 0.2))
        for g in fp.GraphicalItems():
            if g.GetLayer() == silk:
                _mark_stroke(bm, g)
    for d in board.GetDrawings():
        if d.GetLayer() == silk:
            _mark_stroke(bm, d)
    return bm


def _mark_stroke(bm: Bitmap, g) -> None:
    """Mark a silkscreen shape by its strokes (+0.15 mm): a rectangle / polygon outline marks its edges only, not
    its whole interior (rev 2c: a header's outline rectangle used to block every label spot inside it)."""
    w = pcbnew.ToMM(g.GetWidth()) / 2 + 0.15 if hasattr(g, "GetWidth") else 0.15
    pts = None
    if g.GetClass() == "PCB_SHAPE" and g.GetShape() == pcbnew.SHAPE_T_RECT:
        bb = g.GetBoundingBox()
        x0, y0, x1, y1 = (pcbnew.ToMM(v) for v in (bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom()))
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    elif g.GetClass() == "PCB_SHAPE" and g.GetShape() == pcbnew.SHAPE_T_POLY:
        ol = g.GetPolyShape().Outline(0)
        pts = [(pcbnew.ToMM(ol.CPoint(i).x), pcbnew.ToMM(ol.CPoint(i).y)) for i in range(ol.PointCount())]
        pts.append(pts[0])
    if pts is None:
        bm.mark(_mm_box(g.GetBoundingBox(), 0.15))
        return
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        if abs(ax - bx) < 1e-6 or abs(ay - by) < 1e-6:
            bm.mark((min(ax, bx) - w, min(ay, by) - w, max(ax, bx) + w, max(ay, by) + w))
        else:  # oblique edge: its bounding box (conservative)
            bm.mark((min(ax, bx) - w, min(ay, by) - w, max(ax, bx) + w, max(ay, by) + w))


def _courtyard_box(fp):
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    if cy.OutlineCount():
        return _mm_box(cy.BBox(), 0.0)
    return _mm_box(fp.GetBoundingBox(False), 0.0)


def _overlaps(a, b) -> bool:
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


class Ctx:
    def __init__(self, board) -> None:
        self.board = board
        self.bm = {"F": _obstacles(board, "F"), "B": _obstacles(board, "B")}
        self.cy = {fp.GetReference(): _courtyard_box(fp) for fp in board.GetFootprints()}
        self.fab = {}
        for fp in board.GetFootprints():
            boxes = [_mm_box(g.GetBoundingBox(), 0.0) for g in fp.GraphicalItems() if g.GetLayer() == pcbnew.F_Fab
                     and g.GetClass() != "PCB_TEXT"]
            if boxes:
                self.fab[fp.GetReference()] = (min(b[0] for b in boxes), min(b[1] for b in boxes),
                                               max(b[2] for b in boxes), max(b[3] for b in boxes))
            else:
                c = self.cy[fp.GetReference()]
                self.fab[fp.GetReference()] = (c[0] + 0.25, c[1] + 0.25, c[2] - 0.25, c[3] - 0.25)
        # reserved module bodies (build_pcb.drawings: Dwgs.User dashed rectangles)
        self.bodies = []
        for d in board.GetDrawings():
            if d.GetLayer() == pcbnew.Dwgs_User and d.GetClass() == "PCB_SHAPE" and d.GetShape() == pcbnew.SHAPE_T_RECT:
                bb = _mm_box(d.GetBoundingBox(), 0.0)
                self.bodies.append(bb)  # module bodies + the cable-tie band (the strap covers it)

    def free(self, side: str, box, own: str, bodies: bool = True) -> bool:
        """``bodies=False`` (rev 3): the reserved module bodies / the tie band are ignored - used for marks that are
        read at SMT assembly and inspection, before the user fits the modules."""
        if not self.bm[side].free(box):
            return False
        if side == "F":
            for r, c in self.fab.items():
                if r != own and _overlaps(box, c):
                    return False
            if bodies:
                for b in self.bodies:
                    if _overlaps(box, b):
                        return False
        return True


def _text(board, s: str, layer, h: float):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(h), pcbnew.FromMM(h)))
    t.SetTextThickness(pcbnew.FromMM(T))
    if layer == pcbnew.B_SilkS:
        t.SetMirrored(True)
    return t


def _size(t, ang: float):
    t.SetTextAngle(pcbnew.EDA_ANGLE(ang, pcbnew.DEGREES_T))
    t.SetPosition(pcbnew.VECTOR2I(0, 0))
    bb = t.GetBoundingBox()
    return pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight()), pcbnew.ToMM(bb.GetCenter().x), pcbnew.ToMM(bb.GetCenter().y)


def _ink(t, ang: float):
    """rev 3: the true extent of a text's strokes (pen width included) - ``GetBoundingBox`` adds about 0.25 mm of
    empty line spacing on every side, which is too coarse for the marks and designators of the dense boost block."""
    t.SetTextAngle(pcbnew.EDA_ANGLE(ang, pcbnew.DEGREES_T))
    t.SetPosition(pcbnew.VECTOR2I(0, 0))
    bb = t.GetEffectiveTextShape().BBox()
    return pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight()), pcbnew.ToMM(bb.GetCenter().x), pcbnew.ToMM(bb.GetCenter().y)


def _put(ctx: Ctx, side: str, t, x: float, y: float, w: float, h: float, ox: float, oy: float) -> None:
    t.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(round(x - ox, 3)), pcbnew.FromMM(round(y - oy, 3))))
    ctx.bm[side].mark((x - w / 2 - 0.15, y - h / 2 - 0.15, x + w / 2 + 0.15, y + h / 2 + 0.15))


def _name_candidates(cbox, w: float, h: float, max_d: float = MAX_D):
    """Centres around a courtyard box up to MAX_D mm from it, nearest first (below / above / left / right, sliding
    along each side in 0.25 mm steps)."""
    x0, y0, x1, y1 = cbox
    out = []
    k = 0
    while 0.1 * k <= max_d + 1e-9:
        d = round(0.1 * k, 3)
        nx = max(1, int((x1 - x0) / 0.25))
        ny = max(1, int((y1 - y0) / 0.25))
        for i in range(nx + 1):
            fx = x0 + w / 2 + (x1 - x0 - w) * i / nx if x1 - x0 > w else (x0 + x1) / 2
            out.append((d, abs(i - nx / 2), fx, y1 + h / 2 + d))
            out.append((d, abs(i - nx / 2), fx, y0 - h / 2 - d))
        for j in range(ny + 1):
            fy = y0 + h / 2 + (y1 - y0 - h) * j / ny if y1 - y0 > h else (y0 + y1) / 2
            out.append((d, abs(j - ny / 2), x0 - w / 2 - d, fy))
            out.append((d, abs(j - ny / 2), x1 + w / 2 + d, fy))
        k += 1
    out.sort(key=lambda t: (t[0], t[1], round(t[2], 3), round(t[3], 3)))
    return [(x, y, d) for d, _s, x, y in out]


def pin_row(fp) -> tuple[int, int] | None:
    """Board axis pin 1 -> last pin of a single straight pin row along a board axis; None for anything else (two
    rows, a diagonal, one pin)."""
    pads = sorted((p for p in fp.Pads() if p.GetNumber().isdigit()), key=lambda p: int(p.GetNumber()))
    if len(pads) < 2:
        return None
    xs = [p.GetPosition().x for p in pads]
    ys = [p.GetPosition().y for p in pads]
    for along, across in ((xs, ys), (ys, xs)):
        steps = [b - a for a, b in zip(along, along[1:])]
        if len(set(across)) == 1 and (all(d > 0 for d in steps) or all(d < 0 for d in steps)):
            return _axis(xs[-1] - xs[0], ys[-1] - ys[0])
    return None


def pin_name(board, fp, label: str, glyphs: list[str], layer, h: float, ang: float) -> tuple[str, str]:
    """(text, order) of a name that carries the pin glyphs (esp-user-fix step, verification finding V1).

    A name printed ALONG its pin row lists the glyphs in the direction the pads lie on the side the text is printed
    on, so that a reader who maps the letters onto the pads by position is right: on the bottom a mirrored text runs
    against a row whose pin 1 is on the left in the top view, and '1S BAT +-' then showed '+' on the side of the GND
    pad. A name printed ACROSS the row (or beside a two-row header) has no such reading and keeps the order from
    pin 1 (the square pad). The reading direction is measured on pcbnew's stroke geometry (id_directions), and the
    same cosine holds seen from either side of the board (both vectors flip together)."""
    sep = "" if all(len(g) == 1 for g in glyphs) else " "
    row = pin_row(fp)
    run, _down = id_directions(board, layer, h, T, ang)
    cos = 0 if row is None else run[0] * row[0] + run[1] * row[1]
    if cos < 0:
        return f"{label} {sep.join(reversed(glyphs))}", "along the row, as the pads lie (last pin first)"
    if cos > 0:
        return f"{label} {sep.join(glyphs)}", "along the row, as the pads lie (pin 1 first)"
    return f"{label} {sep.join(glyphs)}", "from pin 1 (text not along a single pin row)"


def place_name(ctx: Ctx, board, fp, s: str, glyphs: list[str] | None = None) -> dict | None:
    """``glyphs`` (pin 1 first) makes ``s`` the bare label of a pin-ordered name: the text is then built per
    candidate layer / size / angle by pin_name, BEFORE its box is measured and searched."""
    ref = fp.GetReference()
    cbox = ctx.cy[ref]
    order = None
    label = s
    for side, layer in (("F", pcbnew.F_SilkS), ("B", pcbnew.B_SilkS)):
        for h in NAME_H:
            t = _text(board, s, layer, h)
            for ang in (0, 90):
                if glyphs is not None:
                    s, order = pin_name(board, fp, label, glyphs, layer, h, ang)
                    t.SetText(s)
                w, hh, ox, oy = _size(t, ang)
                for x, y, d in _name_candidates(cbox, w, hh):
                    box = (x - w / 2 - 0.15, y - hh / 2 - 0.15, x + w / 2 + 0.15, y + hh / 2 + 0.15)
                    # text over the own body only for right-angle headers (bare pins over the board, readable
                    # with the plug out); never over a housing (J14 / J7 / J15 / J17 / vertical headers)
                    own = ref if "Horizontal" in fp.GetFPID().GetLibItemName().wx_str() else None
                    if ctx.free(side, box, own):
                        _put(ctx, side, t, x, y, w, hh, ox, oy)
                        board.Add(t)
                        out = {"ref": ref, "text": s, "layer": "F.SilkS" if side == "F" else "B.SilkS", "height_mm": h,
                               "distance_to_courtyard_mm": d}
                        if order:
                            out["pin_glyph_order"] = order
                        return out
    return None


def place_pin_labels(ctx: Ctx, board, fp) -> dict | None:
    """One glyph per pin, all on the same side of the pins (right, left, below, above - first that fits for every
    labelled pin). Pins whose net has no glyph are skipped."""
    ref = fp.GetReference()
    pads = sorted((p for p in fp.Pads() if p.GetNumber().isdigit()), key=lambda p: int(p.GetNumber()))
    items = [(p, pin_glyph(p.GetNetname(), ref)) for p in pads]
    items = [(p, g) for p, g in items if g]
    if not items:
        return None
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        plan = []
        trial = Bitmap()
        trial.rows = list(ctx.bm["F"].rows)
        saved = ctx.bm["F"]
        ctx.bm["F"] = trial
        ok = True
        for p, g in items:
            t = _text(board, g, pcbnew.F_SilkS, PIN_H)
            w, h, ox, oy = _size(t, 0)
            bb = _mm_box(p.GetBoundingBox(), 0.0)
            px, py = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
            rx, ry = (bb[2] - bb[0]) / 2, (bb[3] - bb[1]) / 2
            placed = False
            for gap in (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5):
                x = px + dx * (rx + gap + w / 2 + 0.15)
                y = py + dy * (ry + gap + h / 2 + 0.15)
                box = (x - w / 2 - 0.15, y - h / 2 - 0.15, x + w / 2 + 0.15, y + h / 2 + 0.15)
                if ctx.free("F", box, None):
                    _put(ctx, "F", t, x, y, w, h, ox, oy)
                    plan.append((t, g, p.GetNumber()))
                    placed = True
                    break
            if not placed:
                ok = False
                break
        ctx.bm["F"] = saved
        if ok:
            for t, g, num in plan:
                bb = t.GetBoundingBox()
                ctx.bm["F"].mark(_mm_box(bb, 0.15))
                board.Add(t)
            return {"ref": ref, "side": {(1, 0): "+x", (-1, 0): "-x", (0, 1): "+y (down)", (0, -1): "-y (up)"}[(dx, dy)],
                    "pins": {num: g for _t, g, num in plan}}
    return None


def _rect_dist(x: float, y: float, box) -> float:
    """Distance from a point to an axis-aligned box (0 inside)."""
    dx = max(box[0] - x, 0.0, x - box[2])
    dy = max(box[1] - y, 0.0, y - box[3])
    return math.hypot(dx, dy)


def library_mark(ctx: Ctx, fp) -> dict:
    """What the library footprint itself prints on F.SilkS for orientation, measured on the board: the number of
    silkscreen graphics, whether one of them is the pin-1 triangle (polygon), and whether any of them lies under
    ANOTHER part's body (F.Fab box) or on a pad (then the mark may be hidden / clipped). Evidence, not a verdict."""
    ref = fp.GetReference()
    items = [g for g in fp.GraphicalItems() if g.GetLayer() == pcbnew.F_SilkS and g.GetClass() != "PCB_TEXT"]
    polys = [g for g in items if g.GetClass() == "PCB_SHAPE" and g.GetShape() == pcbnew.SHAPE_T_POLY]
    covered = []
    pad_boxes = [(o.GetReference() + "." + p.GetNumber(), _mm_box(p.GetBoundingBox(), 0.0))
                 for o in ctx.board.GetFootprints() for p in o.Pads() if p.IsOnLayer(pcbnew.F_Cu)]
    for g in items:
        bb = _mm_box(g.GetBoundingBox(), 0.0)
        covered += [r for r, c in ctx.fab.items() if r != ref and _overlaps(bb, c)]
        covered += [n for n, c in pad_boxes if _overlaps(bb, c)]
    p1 = [p for p in fp.Pads() if p.GetNumber() == "1"][0]
    pb = _mm_box(p1.GetBoundingBox(), 0.0)
    tri = None
    if polys:
        c = polys[0].GetBoundingBox().GetCenter()
        tri = round(_rect_dist(pcbnew.ToMM(c.x), pcbnew.ToMM(c.y), pb), 3)
    return {"silk_graphics": len(items), "pin1_triangle": bool(polys), "triangle_to_pad1_mm": tri,
            "covered_by": sorted(set(covered))}


def place_mark(ctx: Ctx, board, fp, kind: str) -> dict:
    """rev 3: one extra polarity / pin-1 mark on F.SilkS beside pad 1 of ``fp``, in addition to the library
    footprint's own mark (``library_mark``). The spot is the free one nearest to the pad-1 rectangle (0.1 mm grid, up
    to MARK_MAX_D from the pad edge) that is outside EVERY part body (F.Fab boxes, the own one included -> never
    hidden by a part), off all pads / silkscreen, and at least 0.3 mm nearer to pad 1 than to any other pad of the
    same footprint (unambiguous). The reserved module bodies are not obstacles: the marks are read at SMT assembly
    and inspection, before the modules are fitted. A polarised capacitor whose pad 1 is on GND is an error (the mark
    would label the wrong pad)."""
    ref = fp.GetReference()
    pads = list(fp.Pads())
    p1 = [p for p in pads if p.GetNumber() == "1"][0]
    if kind == "+" and p1.GetNetname() in ("GND", "GND_PWR"):
        raise ValueError(f"{ref}: pad 1 of a polarised capacitor is on {p1.GetNetname()}")
    if kind == "K" and "LED" in fp.GetFPID().GetLibItemName().wx_str() and p1.GetNetname() != "GND":
        raise ValueError(f"{ref}: LED pad 1 (cathode) is expected on GND, found {p1.GetNetname()}")
    pb = _mm_box(p1.GetBoundingBox(), 0.0)
    others = [_mm_box(p.GetBoundingBox(), 0.0) for p in pads if p.GetNumber() != "1"]
    if kind == "dot":
        t, w, h, ox, oy = None, 2 * DOT_R + T, 2 * DOT_R + T, 0.0, 0.0
    else:
        t = _text(board, kind, pcbnew.F_SilkS, PIN_H)
        w, h, ox, oy = _ink(t, 0)
    gx0, gy0 = round(pb[0] - MARK_MAX_D - w / 2, 1), round(pb[1] - MARK_MAX_D - h / 2, 1)
    cand = []
    for i in range(int((pb[2] - pb[0] + 2 * MARK_MAX_D + w) / 0.1) + 2):
        for j in range(int((pb[3] - pb[1] + 2 * MARK_MAX_D + h) / 0.1) + 2):
            x, y = round(gx0 + 0.1 * i, 3), round(gy0 + 0.1 * j, 3)
            box = (x - w / 2, y - h / 2, x + w / 2, y + h / 2)
            # gap between the mark's own rectangle and the pad-1 rectangle
            d = math.hypot(max(pb[0] - box[2], 0.0, box[0] - pb[2]), max(pb[1] - box[3], 0.0, box[1] - pb[3]))
            if d <= MARK_MAX_D:
                cand.append((round(d, 4), x, y))
    cand.sort()
    lib = library_mark(ctx, fp)
    for d, x, y in cand:
        d1 = _rect_dist(x, y, pb)
        if any(_rect_dist(x, y, q) < d1 + 0.3 for q in others):
            continue
        box = (x - w / 2 - 0.15, y - h / 2 - 0.15, x + w / 2 + 0.15, y + h / 2 + 0.15)
        if not ctx.free("F", box, None, bodies=False):
            continue
        if t is None:
            s = pcbnew.PCB_SHAPE(board)
            s.SetShape(pcbnew.SHAPE_T_CIRCLE)
            s.SetLayer(pcbnew.F_SilkS)
            s.SetCenter(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
            s.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(round(x + DOT_R, 3)), pcbnew.FromMM(y)))
            s.SetWidth(pcbnew.FromMM(T))
            s.SetFilled(True)
            board.Add(s)
            ctx.bm["F"].mark(box)
        else:
            _put(ctx, "F", t, x, y, w, h, ox, oy)
            board.Add(t)
        return {"ref": ref, "mark": kind, "pad1_net": p1.GetNetname(), "at": [x, y], "gap_to_pad1_mm": d,
                "library_mark": lib}
    return {"ref": ref, "mark": kind, "pad1_net": p1.GetNetname(), "library_mark": lib,
            "why": f"no free spot for an extra mark within {MARK_MAX_D} mm of pad 1"}


def _grid_candidates(cbox, w: float, h: float, max_d: float):
    """Every 0.1 mm grid centre whose text box lies within ``max_d`` of a courtyard box, nearest first."""
    x0, y0, x1, y1 = cbox
    out = []
    gx0, gy0 = round(x0 - max_d - w / 2, 1), round(y0 - max_d - h / 2, 1)
    for i in range(int((x1 - x0 + 2 * max_d + w) / 0.1) + 2):
        for j in range(int((y1 - y0 + 2 * max_d + h) / 0.1) + 2):
            x, y = round(gx0 + 0.1 * i, 3), round(gy0 + 0.1 * j, 3)
            d = math.hypot(max(x0 - (x + w / 2), 0.0, (x - w / 2) - x1), max(y0 - (y + h / 2), 0.0, (y - h / 2) - y1))
            if d <= max_d:
                out.append((round(d, 3), x, y))
    out.sort()
    return [(x, y, d) for d, x, y in out]


def place_rev_text(ctx: Ctx, board) -> dict | None:
    """rev 3: the board revision on F.SilkS (1.0 mm), at the free spot nearest to the bottom-left corner."""
    t = _text(board, REV_TEXT, pcbnew.F_SilkS, 1.0)
    w, h, ox, oy = _ink(t, 0)
    cand = []
    for i in range(0, int((BX1 - BX0) / 0.5)):
        for j in range(0, int((BY1 - BY0) / 0.5)):
            x, y = BX0 + 0.5 * i, BY0 + 0.5 * j
            cand.append((round(math.hypot(x - BX0, y - BY1), 3), x, y))
    cand.sort()
    for bodies in (True, False):  # visible on the finished vehicle first; else under a module body / the tie band
        for _d, x, y in cand:
            box = (x - w / 2 - 0.15, y - h / 2 - 0.15, x + w / 2 + 0.15, y + h / 2 + 0.15)
            if ctx.free("F", box, None, bodies=bodies):
                _put(ctx, "F", t, x, y, w, h, ox, oy)
                board.Add(t)
                return {"text": REV_TEXT, "layer": "F.SilkS", "height_mm": 1.0, "at": [x, y],
                        "outside_module_bodies_and_tie_band": bodies}
    return None


def _ref_beneath(ctx: Ctx, board, fp) -> dict:
    """rev 3: the designator as a mirrored B.SilkS text straight beneath a top-side part (nearest free spot to the
    part centre, 0.1 mm grid, at most 3 mm away)."""
    ref = fp.GetReference()
    c = ctx.cy[ref]
    cx, cy = round((c[0] + c[2]) / 2, 1), round((c[1] + c[3]) / 2, 1)
    t = _text(board, ref, pcbnew.B_SilkS, 0.8)
    for ang in (0, 90):
        w, h, ox, oy = _ink(t, ang)
        cand = sorted((round(math.hypot(i * 0.1, j * 0.1), 4), i, j) for i in range(-30, 31) for j in range(-30, 31)
                      if math.hypot(i * 0.1, j * 0.1) <= 3.0)
        for d, i, j in cand:
            x, y = round(cx + i * 0.1, 3), round(cy + j * 0.1, 3)
            box = (x - w / 2 - 0.15, y - h / 2 - 0.15, x + w / 2 + 0.15, y + h / 2 + 0.15)
            if ctx.free("B", box, None):
                _put(ctx, "B", t, x, y, w, h, ox, oy)
                board.Add(t)
                return {"ref": ref, "layer": "B.SilkS", "mirrored": True, "at": [x, y], "angle_deg": ang,
                        "distance_to_part_centre_mm": d,
                        "why": f"no free F.SilkS spot within {REF_FIRST_MAX_D} mm of the courtyard"}
    return {"ref": ref, "layer": None, "why": "no free spot on F.SilkS or beneath the part on B.SilkS"}


# ------------------------------------------------------------------------------------------ identification lines
def _id_text(board, s: str, layer, h: float, th: float):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(h), pcbnew.FromMM(h)))
    t.SetTextThickness(pcbnew.FromMM(th))
    if layer == pcbnew.B_SilkS:
        t.SetMirrored(True)
    return t


def _axis(dx: float, dy: float) -> tuple[int, int]:
    """Board axis (unit vector) a measured offset points along."""
    if abs(dx) > abs(dy):
        return (1 if dx > 0 else -1, 0)
    return (0, 1 if dy > 0 else -1)


def id_directions(board, layer, h: float, th: float, ang: float) -> tuple[tuple[int, int], tuple[int, int]]:
    """(run, down) of a text of this layer / angle in board axes, MEASURED on pcbnew's own stroke geometry, not
    assumed: ``run`` = where a left-justified text extends from its anchor (reading direction), ``down`` = where
    descenders shift the ink (the side the second line goes)."""
    p = _id_text(board, "MMMMMM", layer, h, th)
    p.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    _w, _h, rx, ry = _ink(p, ang)
    _w, _h, gx, gy = _ink(_id_text(board, "gjpqy", layer, h, th), ang)
    _w, _h, tx, ty = _ink(_id_text(board, "TTTTT", layer, h, th), ang)
    run, down = _axis(rx, ry), _axis(gx - tx, gy - ty)
    if run[0] * down[0] + run[1] * down[1] != 0:
        raise ValueError(f"text directions not perpendicular: run {run}, down {down}")
    return run, down


def _id_layout(texts, ang: float, run, down) -> list[dict]:
    """Ink rectangles of the lines relative to the ink centre of line 1: line n + 1 lies ``ID_GAP`` further 'down'
    and starts at the same edge (left-aligned block)."""
    rows = []
    cx = cy = 0.0
    prev = None
    for t in texts:
        w, h, ox, oy = _ink(t, ang)
        length = w * abs(run[0]) + h * abs(run[1])
        ext = w * abs(down[0]) + h * abs(down[1])
        if prev is not None:
            step = (prev["ext"] + ext) / 2 + ID_GAP
            shift = -(rows[0]["length"] - length) / 2  # same start edge as line 1
            first = rows[0]
            along = (first["cx"] * run[0] + first["cy"] * run[1]) + shift
            across = (prev["cx"] * down[0] + prev["cy"] * down[1]) + step
            cx = along * run[0] + across * down[0]
            cy = along * run[1] + across * down[1]
        row = {"t": t, "w": w, "h": h, "ox": ox, "oy": oy, "cx": cx, "cy": cy, "length": length, "ext": ext}
        rows.append(row)
        prev = row
    return rows


def _occupancy(bm: Bitmap) -> np.ndarray:
    """Summed-area table of a bitmap ([ny + 1, nx + 1]): the number of occupied cells of any cell rectangle in O(1)."""
    nb = (bm.nx + 7) // 8
    raw = b"".join(r.to_bytes(nb, "little") for r in bm.rows)
    cells = np.unpackbits(np.frombuffer(raw, dtype=np.uint8).reshape(bm.ny, nb), axis=1, bitorder="little")[:, :bm.nx]
    s = np.zeros((bm.ny + 1, bm.nx + 1), dtype=np.int64)
    s[1:, 1:] = cells.astype(np.int64).cumsum(0).cumsum(1)
    return s


def id_obstacles(ctx: Ctx, side: str, avoid_band: bool) -> Bitmap:
    """Everything an identification line must not touch on one side: the pads / footprint silkscreen / texts already
    in the side's bitmap, every pad with a hole or a mask opening on that side (+0.2), every via pad (+0.1; the vias
    are tented, but ink over a via hole breaks the stroke), the 0.5 mm edge strip, on the front every part body
    (F.Fab box) and reserved module body, and - ``avoid_band`` - the cable-tie band."""
    board = ctx.board
    cu, mask = (pcbnew.F_Cu, pcbnew.F_Mask) if side == "F" else (pcbnew.B_Cu, pcbnew.B_Mask)
    bm = Bitmap()
    bm.rows = list(ctx.bm[side].rows)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetDrillSize().x > 0 or pad.IsOnLayer(mask) or pad.IsOnLayer(cu):
                bm.mark(_mm_box(pad.GetBoundingBox(), 0.2))
    for v in board.GetTracks():
        if v.GetClass() == "PCB_VIA" and v.IsOnLayer(cu):
            x, y = pcbnew.ToMM(v.GetPosition().x), pcbnew.ToMM(v.GetPosition().y)
            r = pcbnew.ToMM(v.GetWidth(cu)) / 2 + 0.1
            bm.mark((x - r, y - r, x + r, y + r))
    if side == "F":
        for c in ctx.fab.values():
            bm.mark(c)
        for b in ctx.bodies:
            bm.mark(b)
    if avoid_band:
        bm.mark((TIE_BAND[0], BY0, TIE_BAND[1], BY1))
    for strip in ((BX0, BY0, BX0 + 0.5, BY1), (BX1 - 0.5, BY0, BX1, BY1), (BX0, BY0, BX1, BY0 + 0.5),
                  (BX0, BY1 - 0.5, BX1, BY1)):
        bm.mark(strip)
    return bm


def _id_spots(s: np.ndarray, rows: list[dict], m: float) -> np.ndarray:
    """[l, k] -> True where the block fits with line 1's ink centre at (BX0 + 0.1 k, BY0 + 0.1 l): every ink
    rectangle grown by ``m`` covers no occupied cell and stays on the board."""
    ny, nx = s.shape[0] - 1, s.shape[1] - 1
    step = int(round(0.1 / CELL))
    k = np.arange(0, nx // step + 1) * step
    l = np.arange(0, ny // step + 1) * step
    ok = np.ones((len(l), len(k)), dtype=bool)
    for r in rows:
        ia = math.floor(round((r["cx"] - r["w"] / 2 - m) / CELL, 6))
        ib = math.ceil(round((r["cx"] + r["w"] / 2 + m) / CELL, 6))
        ja = math.floor(round((r["cy"] - r["h"] / 2 - m) / CELL, 6))
        jb = math.ceil(round((r["cy"] + r["h"] / 2 + m) / CELL, 6))
        c0, c1, r0, r1 = k + ia, k + ib, l + ja, l + jb
        vk, vl = (c0 >= 0) & (c1 <= nx), (r0 >= 0) & (r1 <= ny)
        c0, c1, r0, r1 = (np.clip(a, 0, n) for a, n in ((c0, nx), (c1, nx), (r0, ny), (r1, ny)))
        n = s[np.ix_(r1, c1)] - s[np.ix_(r0, c1)] - s[np.ix_(r1, c0)] + s[np.ix_(r0, c0)]
        ok &= (n == 0) & vl[:, None] & vk[None, :]
    return ok


def place_id_lines(ctx: Ctx, board) -> dict:
    """silk-id step: the ID_LINES block. Front only for a clearly free spot (>= ID_FRONT_MARGIN around the ink,
    outside every part and module body and the tie band); else the bottom, mirrored, off the tie band; last resort the
    bottom under the tie band. Per side: the larger size first, the reading direction along x before along y, and of
    all spots the one with the largest clearance tier, nearest to the board centre. Nothing is logged by content."""
    tried = []
    front = tuple(m for m in ID_MARGINS if m >= ID_FRONT_MARGIN)
    for side, layer, avoid_band, margins in (("F", pcbnew.F_SilkS, True, front), ("B", pcbnew.B_SilkS, True, ID_MARGINS),
                                             ("B", pcbnew.B_SilkS, False, ID_MARGINS)):
        obst = id_obstacles(ctx, side, avoid_band)
        s = _occupancy(obst)
        for h, th in ID_SIZES:
            for ang in (0, 90):
                run, down = id_directions(board, layer, h, th, ang)
                texts = [_id_text(board, line, layer, h, th) for _kind, line in ID_LINES]
                rows = _id_layout(texts, ang, run, down)
                ux = (min(r["cx"] - r["w"] / 2 for r in rows) + max(r["cx"] + r["w"] / 2 for r in rows)) / 2
                uy = (min(r["cy"] - r["h"] / 2 for r in rows) + max(r["cy"] + r["h"] / 2 for r in rows)) / 2
                for m in margins:
                    ok = _id_spots(s, rows, m)
                    n = int(ok.sum())
                    if not n:
                        continue
                    ls, ks = np.nonzero(ok)
                    xs, ys = BX0 + 0.1 * ks, BY0 + 0.1 * ls
                    d2 = (xs + ux - (BX0 + BX1) / 2) ** 2 + (ys + uy - (BY0 + BY1) / 2) ** 2
                    i = int(np.argmin(np.round(d2, 6)))
                    x0, y0 = round(float(xs[i]), 3), round(float(ys[i]), 3)
                    lines = []
                    for n_line, (r, (kind, line)) in enumerate(zip(rows, ID_LINES), start=1):
                        x, y = round(x0 + r["cx"], 3), round(y0 + r["cy"], 3)
                        box = (x - r["w"] / 2 - m, y - r["h"] / 2 - m, x + r["w"] / 2 + m, y + r["h"] / 2 + m)
                        if not obst.free(box):
                            raise ValueError(f"identification line {n_line}: the chosen spot is not free")
                        _put(ctx, side, r["t"], x, y, r["w"], r["h"], r["ox"], r["oy"])
                        board.Add(r["t"])
                        ink = _mm_box(r["t"].GetEffectiveTextShape().BBox(), 0.0)
                        lines.append({"line": n_line, "what": kind, "characters": len(line),
                                      "ink_box_mm": [round(v, 3) for v in ink]})
                    return {"placed": True, "layer": "F.SilkS" if side == "F" else "B.SilkS", "mirrored": side == "B",
                            "height_mm": h, "stroke_mm": th, "angle_deg": ang, "reads_towards": list(run),
                            "line_2_towards": list(down), "clearance_tier_mm": m, "spots_in_tier": n,
                            "tie_band_avoided": avoid_band, "via_pads_avoided": True, "lines": lines,
                            "not_possible_before": tried}
                tried.append({"layer": "F.SilkS" if side == "F" else "B.SilkS", "tie_band_avoided": avoid_band,
                              "height_mm": h, "angle_deg": ang, "smallest_clearance_tried_mm": margins[-1], "spots": 0})
    return {"placed": False, "not_possible_before": tried}


def add_silk(board) -> dict:
    ctx = Ctx(board)
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}

    def key(r: str):
        p = r.rstrip("0123456789")
        return (p, int(r[len(p):]) if r[len(p):].isdigit() else 0)

    pin_sets, pin_missed = [], []
    # rev 3: polarity / pin-1 marks first (they need the spot next to pad 1), then the names
    marks, marks_missed = [], []
    for ref in sorted(MARKS, key=key):
        if ref not in fps:
            marks_missed.append({"ref": ref, "why": "no such footprint"})
            continue
        m = place_mark(ctx, board, fps[ref], MARKS[ref])
        (marks if m.get("at") else marks_missed).append(m)
    names = dict(LABELS)
    glyphs: dict[str, list[str]] = {}
    for ref in PIN_LABELLED:  # the name carries the pin glyphs, e.g. 'V1 S+-'; their order: see pin_name
        if ref in fps:
            pads = sorted((p for p in fps[ref].Pads() if p.GetNumber().isdigit()), key=lambda p: int(p.GetNumber()))
            glyphs[ref] = [pin_glyph(p.GetNetname(), ref) or "nc" for p in pads]
    names_f, names_b, names_missed = [], [], []
    # off-board leads first (what is plugged in blind), then modules, then programming-only footprints
    order = [r for r in PRIORITY if r in names] + sorted((r for r in names if r not in PRIORITY), key=key)
    for ref in order:
        if ref not in fps:
            continue
        r = place_name(ctx, board, fps[ref], names[ref], glyphs.get(ref))
        if r is None:
            names_missed.append(ref)
        elif r["layer"] == "F.SilkS":
            names_f.append(r)
        else:
            names_b.append(r)
    rev = place_rev_text(ctx, board)
    refs_on_silk, refs_on_fab, refs_first = [], [], []
    for ref in [r for r in REFS_FIRST if r in fps] + [r for r in sorted(fps, key=key) if r not in REFS_FIRST]:
        fp = fps[ref]
        if fp.IsDNP():
            refs_on_fab.append(ref)
            continue
        f = fp.Reference()
        old_layer, old_pos, old_ang = f.GetLayer(), f.GetPosition(), f.GetTextAngle()
        f.SetLayer(pcbnew.F_SilkS)
        f.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(0.8), pcbnew.FromMM(0.8)))
        f.SetTextThickness(pcbnew.FromMM(T))
        f.SetVisible(True)
        done = False
        # rev 3: the boost-block designators are searched on the full 0.1 mm grid up to REF_FIRST_MAX_D from the
        # courtyard (further away a designator would point at a neighbour); if F.SilkS has no such spot the
        # designator goes on B.SilkS straight beneath the part (the bottom side carries no parts) and is listed
        tiers = [(MAX_D, True, 400)] if ref not in REFS_FIRST else [(REF_FIRST_MAX_D, True, None)]
        for max_d, bodies, limit in tiers:
            for ang in (0, 90):
                w, h, ox, oy = _ink(f, ang) if ref in REFS_FIRST else _size(f, ang)
                cands = _grid_candidates(ctx.cy[ref], w, h, max_d) if ref in REFS_FIRST else \
                    _name_candidates(ctx.cy[ref], w, h, max_d)[:limit]
                for x, y, d in cands:
                    box = (x - w / 2 - 0.15, y - h / 2 - 0.15, x + w / 2 + 0.15, y + h / 2 + 0.15)
                    if ctx.free("F", box, None, bodies=bodies):
                        _put(ctx, "F", f, x, y, w, h, ox, oy)
                        done = True
                        if ref in REFS_FIRST:
                            refs_first.append({"ref": ref, "layer": "F.SilkS", "at": [round(x, 3), round(y, 3)],
                                               "angle_deg": ang, "distance_to_courtyard_mm": d})
                        break
                if done:
                    break
            if done:
                break
        if done:
            refs_on_silk.append(ref)
        else:
            f.SetLayer(old_layer)
            f.SetPosition(old_pos)
            f.SetTextAngle(old_ang)
            refs_on_fab.append(ref)
            if ref in REFS_FIRST:
                refs_first.append(_ref_beneath(ctx, board, fp))
    # silk-id step: last of all, so that no other text or mark changes its place
    ids = place_id_lines(ctx, board)
    return {"identification_lines": ids, "names_front": names_f, "names_back_mirrored": names_b, "names_missed": names_missed,
            "polarity_marks": marks, "polarity_marks_missed": marks_missed, "revision_text": rev,
            "refs_first_on_silk": refs_first,
            "pin_label_sets": pin_sets, "pin_label_sets_missed": pin_missed,
            "refs_on_silk": len(refs_on_silk), "refs_on_fab_only": refs_on_fab}
