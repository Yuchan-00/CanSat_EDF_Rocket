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
* Sizes: names 1.0 mm high / 0.15 mm stroke (JLC recommendation, A), 0.8 mm (board minimum) when 1.0 does not fit;
  pin labels 0.8 mm. Deterministic: fixed tables, fixed candidate order, no randomness.
"""

from __future__ import annotations

import math

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
    "J6": "5V MOD", "JP1": "GPS 3V3|5V", "U7": "ESP-12E",
}
#: placement priority of the names (front space is scarce: the off-board leads that are plugged in blind first)
PRIORITY = ("J14", "J15", "J18", "J12", "J8", "J9", "J10", "J11", "J7", "J16", "J17", "J6", "JP1", "J3")
#: connectors whose name carries the pin order (off-board leads + the ESP flash header)
PIN_LABELLED = ("J8", "J9", "J10", "J11", "J12", "J17", "J14", "J18", "J7", "J16", "J3", "J6")


def pin_glyph(net: str, ref: str = "") -> str | None:
    """Short pin label from the pad's net name (routed board = spec nets)."""
    if ref in ("J14", "J17") and net in ("GND", "+5V"):
        return "-" if net == "GND" else "+"
    if ref == "J6" and net == "+5V":
        return "VO"
    if net.endswith("_SIG"):
        return "S"
    table = {"SERVO_5V": "+", "GND_PWR": "-", "GND": "G", "+5V": "5V", "VBAT_LOGIC_RAW": "+", "BUZ_N": "-",
             "VS_VIN": "VI", "VS_VTETHER": "VT", "VS_VBUFFER": "VB", "VS_BUF_ISIGN": "IS", "ARM_SW_EXT": "A",
             "GPS_VCC": "V", "ESP_GPIO0": "IO0", "ESP_TXD0": "TX", "ESP_RXD0": "RX", "ESP_RST": "RST",
             "VBAT_LOGIC_SW": "VI", "PG_5V": "PG", "EN_5V": "EN"}
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

    def free(self, side: str, box, own: str) -> bool:
        if not self.bm[side].free(box):
            return False
        if side == "F":
            for r, c in self.fab.items():
                if r != own and _overlaps(box, c):
                    return False
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


def _put(ctx: Ctx, side: str, t, x: float, y: float, w: float, h: float, ox: float, oy: float) -> None:
    t.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(round(x - ox, 3)), pcbnew.FromMM(round(y - oy, 3))))
    ctx.bm[side].mark((x - w / 2 - 0.15, y - h / 2 - 0.15, x + w / 2 + 0.15, y + h / 2 + 0.15))


def _name_candidates(cbox, w: float, h: float):
    """Centres around a courtyard box up to MAX_D mm from it, nearest first (below / above / left / right, sliding
    along each side in 0.25 mm steps)."""
    x0, y0, x1, y1 = cbox
    out = []
    k = 0
    while 0.1 * k <= MAX_D + 1e-9:
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


def place_name(ctx: Ctx, board, fp, s: str) -> dict | None:
    ref = fp.GetReference()
    cbox = ctx.cy[ref]
    for side, layer in (("F", pcbnew.F_SilkS), ("B", pcbnew.B_SilkS)):
        for h in NAME_H:
            t = _text(board, s, layer, h)
            for ang in (0, 90):
                w, hh, ox, oy = _size(t, ang)
                for x, y, d in _name_candidates(cbox, w, hh):
                    box = (x - w / 2 - 0.15, y - hh / 2 - 0.15, x + w / 2 + 0.15, y + hh / 2 + 0.15)
                    # text over the own body only for right-angle headers (bare pins over the board, readable
                    # with the plug out); never over a housing (J14 / J7 / J15 / J17 / vertical headers)
                    own = ref if "Horizontal" in fp.GetFPID().GetLibItemName().wx_str() else None
                    if ctx.free(side, box, own):
                        _put(ctx, side, t, x, y, w, hh, ox, oy)
                        board.Add(t)
                        return {"ref": ref, "text": s, "layer": "F.SilkS" if side == "F" else "B.SilkS", "height_mm": h,
                                "distance_to_courtyard_mm": d}
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


def add_silk(board) -> dict:
    ctx = Ctx(board)
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}

    def key(r: str):
        p = r.rstrip("0123456789")
        return (p, int(r[len(p):]) if r[len(p):].isdigit() else 0)

    pin_sets, pin_missed = [], []
    names = dict(LABELS)
    for ref in PIN_LABELLED:  # the name carries the pin order from pin 1 (square pad), e.g. 'V1 S+-'
        if ref in fps:
            pads = sorted((p for p in fps[ref].Pads() if p.GetNumber().isdigit()), key=lambda p: int(p.GetNumber()))
            gl = [pin_glyph(p.GetNetname(), ref) or "nc" for p in pads]
            sep = "" if all(len(g) == 1 for g in gl) else " "
            names[ref] = f"{LABELS[ref]} {sep.join(gl)}"
    names_f, names_b, names_missed = [], [], []
    # off-board leads first (what is plugged in blind), then modules, then programming-only footprints
    order = [r for r in PRIORITY if r in names] + sorted((r for r in names if r not in PRIORITY), key=key)
    for ref in order:
        if ref not in fps:
            continue
        r = place_name(ctx, board, fps[ref], names[ref])
        if r is None:
            names_missed.append(ref)
        elif r["layer"] == "F.SilkS":
            names_f.append(r)
        else:
            names_b.append(r)
    refs_on_silk, refs_on_fab = [], []
    for ref in sorted(fps, key=key):
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
        for ang in (0, 90):
            w, h, ox, oy = _size(f, ang)
            for x, y, _d in _name_candidates(ctx.cy[ref], w, h)[:400]:
                box = (x - w / 2 - 0.15, y - h / 2 - 0.15, x + w / 2 + 0.15, y + h / 2 + 0.15)
                if ctx.free("F", box, None):
                    _put(ctx, "F", f, x, y, w, h, ox, oy)
                    done = True
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
    return {"names_front": names_f, "names_back_mirrored": names_b, "names_missed": names_missed,
            "pin_label_sets": pin_sets, "pin_label_sets_missed": pin_missed,
            "refs_on_silk": len(refs_on_silk), "refs_on_fab_only": refs_on_fab}
