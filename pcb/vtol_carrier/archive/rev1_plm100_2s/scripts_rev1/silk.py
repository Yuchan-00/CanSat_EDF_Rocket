"""Silkscreen text for the routed carrier (KiCad python, called from route.py; review F3).

Invariants
----------
* Text never touches a pad, another silkscreen item or the board edge: every candidate position is tested
  against an occupancy bitmap of pad boxes (+0.2 mm), footprint F.SilkS shapes (+0.15 mm), already placed texts
  (+0.15 mm) and a 0.5 mm edge margin; a text that fits nowhere stays on F.Fab (assembly drawing) and is listed.
* Deterministic: fixed label table, fixed candidate order (closest first), no randomness.
* Sizes: 0.8 mm high, 0.15 mm stroke (board rule min_text_height 0.8 mm; JLCPCB minimum, A).
* A connector label that finds no room on the top goes to B.SilkS (mirrored) under its connector: the bottom
  carries no components, only THT pins and vias (labels there are readable while the board is out for wiring).
"""

from __future__ import annotations

import math

import pcbnew

BX0, BY0, BX1, BY1 = 62.0, 65.0, 138.0, 135.0
CELL = 0.05
H = 0.8
T = 0.15

#: connector / jumper / module labels (pin 1 = square pad; pin order spelled from pin 1). Sources: spec nets.
LABELS: dict[str, str] = {
    "J8": "V1 S+-", "J9": "V2 S+-", "J10": "V3 S+-", "J11": "V4 S+-", "J13": "REL S+-",
    "J12": "ESC S+-", "J17": "BUZ +-", "J14": "2S ONLY +-", "J18": "6S! VIN VT VB IS",
    "J7": "GPS V TX RX G", "J15": "PWR", "J16": "ARM", "J24": "NAV RST", "J25": "FCU RST",
    "J19": "ISP NAV", "J20": "ISP FCU", "J21": "ISP COM", "J22": "ISP GNS", "J23": "ISP U5",
    "J1": "GY-521", "J2": "BMP280", "J3": "PLM100", "J5": "OPENLOG", "J6": "BUCK 5V", "JP1": "GPS 3V3|5V",
}


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
                bm.mark(_mm_box(g.GetBoundingBox(), 0.15))
    for d in board.GetDrawings():
        if d.GetLayer() == silk:
            bm.mark(_mm_box(d.GetBoundingBox(), 0.15))
    return bm


def _candidates(cbox, w: float, h: float):
    """Centres around a courtyard box, nearest first: below / above / left / right, then further out."""
    x0, y0, x1, y1 = cbox
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    out = []
    for k in range(0, 61):
        d = 0.1 * k
        out += [(cx, y1 + h / 2 + d, d), (cx, y0 - h / 2 - d, d), (x0 - w / 2 - d, cy, d), (x1 + w / 2 + d, cy, d)]
        for s in (-1, 1):  # slide along the sides, then the corners
            out += [(cx + s * (x1 - x0) / 3, y1 + h / 2 + d, d + 0.05), (cx + s * (x1 - x0) / 3, y0 - h / 2 - d, d + 0.05),
                    (x0 - w / 2 - d, cy + s * (y1 - y0) / 3, d + 0.05), (x1 + w / 2 + d, cy + s * (y1 - y0) / 3, d + 0.05),
                    (x0 - w / 2 - d, cy + s * (y1 - y0) / 2 + s * h / 2, d + 0.1),
                    (x1 + w / 2 + d, cy + s * (y1 - y0) / 2 + s * h / 2, d + 0.1)]
    return [(x, y) for x, y, _d in sorted(out, key=lambda t: (round(t[2], 3), round(t[0], 3), round(t[1], 3)))]


def _try_place(bm: Bitmap, text, cbox) -> bool:
    for ang in (0, 90):
        text.SetTextAngle(pcbnew.EDA_ANGLE(ang, pcbnew.DEGREES_T))
        text.SetPosition(pcbnew.VECTOR2I(0, 0))
        bb = text.GetBoundingBox()
        w, h = pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight())
        ox = pcbnew.ToMM(bb.GetCenter().x)
        oy = pcbnew.ToMM(bb.GetCenter().y)
        for x, y in _candidates(cbox, w, h):
            box = (x - w / 2 - 0.15, y - h / 2 - 0.15, x + w / 2 + 0.15, y + h / 2 + 0.15)
            if bm.free(box):
                text.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(round(x - ox, 3)), pcbnew.FromMM(round(y - oy, 3))))
                bm.mark(box)
                return True
    return False


def _courtyard_box(fp):
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    if cy.OutlineCount():
        return _mm_box(cy.BBox(), 0.0)
    return _mm_box(fp.GetBoundingBox(False), 0.0)


def add_silk(board) -> dict:
    bm = _obstacles(board, "F")
    bmb = _obstacles(board, "B")
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    placed_labels, bottom_labels, missed_labels, refs_on_silk, refs_on_fab = [], [], [], [], []
    for ref in sorted(LABELS, key=lambda r: (r.rstrip("0123456789"), int("".join(c for c in r if c.isdigit()) or 0))):
        fp = fps.get(ref)
        if fp is None:
            continue
        t = pcbnew.PCB_TEXT(board)
        t.SetText(LABELS[ref])
        t.SetLayer(pcbnew.F_SilkS)
        t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(H), pcbnew.FromMM(H)))
        t.SetTextThickness(pcbnew.FromMM(T))
        if _try_place(bm, t, _courtyard_box(fp)):
            board.Add(t)
            placed_labels.append(ref)
            continue
        t.SetLayer(pcbnew.B_SilkS)
        t.SetMirrored(True)
        if _try_place(bmb, t, _courtyard_box(fp)):
            board.Add(t)
            bottom_labels.append(ref)
        else:
            missed_labels.append(ref)
    # reference designators: on silk where they fit, else they stay on F.Fab
    def key(r: str):
        p = r.rstrip("0123456789")
        return (p, int(r[len(p):]) if r[len(p):].isdigit() else 0)

    for ref in sorted(fps, key=key):
        fp = fps[ref]
        if fp.IsDNP():
            refs_on_fab.append(ref)
            continue
        f = fp.Reference()
        old_layer, old_pos, old_ang = f.GetLayer(), f.GetPosition(), f.GetTextAngle()
        f.SetLayer(pcbnew.F_SilkS)
        f.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(H), pcbnew.FromMM(H)))
        f.SetTextThickness(pcbnew.FromMM(T))
        f.SetVisible(True)
        if _try_place(bm, f, _courtyard_box(fp)):
            refs_on_silk.append(ref)
        else:
            f.SetLayer(old_layer)
            f.SetPosition(old_pos)
            f.SetTextAngle(old_ang)
            refs_on_fab.append(ref)
    return {"labels_top": placed_labels, "labels_bottom_mirrored": bottom_labels, "labels_missed": missed_labels, "refs_on_silk": len(refs_on_silk),
            "refs_on_fab_only": refs_on_fab}
