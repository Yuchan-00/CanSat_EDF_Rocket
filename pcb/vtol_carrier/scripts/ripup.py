"""Deterministic local rip-up and reroute of the last open signal connections (KiCad python, from route.py finish()).

Why (rev 3, M): Freerouting and close_leftovers (attempts a1, a4, a5) end with 9-13 connections open in the congested
centre of the board, because a connection with no legal path cannot move the copper that blocks it. This step can.
Each open connection (a net whose copper falls into several clusters) is searched on a 0.1 mm grid on F.Cu / B.Cu,
0.2 mm track, 0.6 / 0.3 mm vias. The search may cross the copper of another ripable signal net at a cost: only the
crossed piece of that net is cut out (a segment is split and trimmed around the crossing, the rest stays where it is)
and that net is reconnected the same way, with a cost that grows every time the net is ripped (negotiated congestion).

Invariants
----------
* Only Default-class signal nets are cut or rerouted. Never moved: power / servo / plane nets, *_AVCC, crystal nets,
  BOOST_SW / BOOST_FB, pre-routes, fan-out, every pad.
* The rules are not touched: every grid point of a path keeps the class clearance (0.13 mm) + half the track width
  + a 0.02 mm margin to every foreign copper item; every via keeps the same on both outer layers, 0.5 mm + 0.05 mm
  hole to hole, and the rule areas / the 0.3 mm board edge rule hold. kicad-cli DRC is the judge afterwards.
* New copper is the class width, 0.2 mm, always.
* Deterministic: fixed grid, sorted nets, heap ties broken by a counter, an iteration limit and no clock in any
  decision. The result is only written to the board if it leaves fewer open nets than it started with.
"""

from __future__ import annotations

import heapq
import math

import numpy as np
import pcbnew

STEP = 0.1
W = 0.2
CLR = 0.13
MARGIN = 0.02
VIA_D, VIA_DRILL = 0.6, 0.3
HOLE_GAP = 0.5 + 0.05
EDGE = 0.4   # relayout step (review J2): new copper keeps 0.4 mm from every edge (board rule 0.3 mm, unchanged)
VIA_COST = 2.5
RIP_COST = 1.0
TRIM = 0.6            # mm of a cut segment removed on each side of the crossing
BX0, BY0, BX1, BY1 = 62.0, 65.0, 138.0, 135.0
NX = int(round((BX1 - BX0) / STEP)) + 1
NY = int(round((BY1 - BY0) / STEP)) + 1
MAX_ITER = 2500
MAX_FAIL = 6
NEVER = {"GND", "+5V", "GPS_VCC", "SERVO_5V", "GND_PWR", "VBAT_LOGIC", "VBAT_LOGIC_RAW", "VBAT_LOGIC_SW",
         "BOOST_SW", "BOOST_FB"}
#: nets that may only be COMPLETED (no rip-up by them, never cut by others), widths tried in order: class width first
WIDE = {"+3V3": (0.5, 0.3, 0.2), "GND": (0.4, 0.3, 0.2)}
#: pad clusters the copper model cannot see as joined through a pour: (net, island pad, pad of the main cluster).
#: NT1.1 (ground side of the net tie) has no fan-out via and sits in a pour island; J16.2 is a PTH pad on In1.Cu GND
#: (M: rev 3 attempts a6 / a7, DRC "zone GND - J16.2"; the same 2.5 mm join closed it in a6).
ISLAND_JOINS = [("GND", "NT1.1", "J16.2")]
VIRTUAL = {"GND"}
PROTECT = 1e6
OCT = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]


def ripable(net: str) -> bool:
    return bool(net) and net not in NEVER and not net.startswith("unconnected-") and not net.endswith("_AVCC") \
        and "XTAL" not in net


def _mm(v) -> float:
    return pcbnew.ToMM(v)


def _nm(x: float) -> int:
    return int(round(x * 1e6))


def _grid_box(x0, y0, x1, y1):
    i0 = max(0, int(math.floor((x0 - BX0) / STEP)))
    i1 = min(NX - 1, int(math.ceil((x1 - BX0) / STEP)))
    j0 = max(0, int(math.floor((y0 - BY0) / STEP)))
    j1 = min(NY - 1, int(math.ceil((y1 - BY0) / STEP)))
    return i0, j0, i1, j1


def seg_mask(ax, ay, bx, by, r):
    """(i0, j0, boolean array) of the grid points within r of the segment; None when it misses the grid."""
    i0, j0, i1, j1 = _grid_box(min(ax, bx) - r, min(ay, by) - r, max(ax, bx) + r, max(ay, by) + r)
    if i1 < i0 or j1 < j0:
        return None
    X, Y = np.meshgrid(BX0 + STEP * np.arange(i0, i1 + 1), BY0 + STEP * np.arange(j0, j1 + 1))
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-12 else np.clip(((X - ax) * dx + (Y - ay) * dy) / l2, 0.0, 1.0)
    return i0, j0, (X - ax - t * dx) ** 2 + (Y - ay - t * dy) ** 2 <= r * r


class Item:
    """One track segment (kind 's') or via (kind 'v') of a ripable net with its rasters on the shared grids."""
    __slots__ = ("kind", "ax", "ay", "bx", "by", "layer", "w", "rasters", "factor")

    def __init__(self, kind, ax, ay, bx=None, by=None, layer=0, w=W):
        self.kind, self.ax, self.ay, self.bx, self.by, self.layer, self.w = kind, ax, ay, bx, by, layer, w
        self.rasters = None
        self.factor = 0.0

    def key(self):
        if self.kind == "v":
            return ("v", _nm(self.ax), _nm(self.ay))
        a, b = sorted([(_nm(self.ax), _nm(self.ay)), (_nm(self.bx), _nm(self.by))])
        return ("s", self.layer, a, b)

    def build(self):
        out = []
        if self.kind == "s":
            for kind, r in ((self.layer, self.w / 2 + CLR + W / 2 + MARGIN), (2, self.w / 2 + CLR + VIA_D / 2 + MARGIN)):
                m = seg_mask(self.ax, self.ay, self.bx, self.by, r)
                if m:
                    out.append((kind, *m))
        else:
            for lay in (0, 1):
                m = seg_mask(self.ax, self.ay, self.ax, self.ay, VIA_D / 2 + CLR + W / 2 + MARGIN)
                if m:
                    out.append((lay, *m))
            m = seg_mask(self.ax, self.ay, self.ax, self.ay, max(VIA_D + CLR + MARGIN, VIA_DRILL + HOLE_GAP))
            if m:
                out.append((2, *m))
        self.rasters = out


class Router:
    def __init__(self, board):
        self.board = board
        self.fixed = [np.zeros((NY, NX), dtype=np.int32) for _ in range(3)]    # F, B, via grid: permanent obstacles
        self.cost = [np.zeros((NY, NX), dtype=np.float32) for _ in range(3)]   # rip-up cost of the ripable copper
        self.own_pad: dict[str, list] = {}
        self.nets: dict[str, list] = {}    # net -> items
        self.pads: dict[str, list] = {}    # net -> pad dicts
        self.hist: dict[str, int] = {}
        self._load()

    def _add_fixed(self, kind, m):
        i0, j0, mask = m
        self.fixed[kind][j0:j0 + mask.shape[0], i0:i0 + mask.shape[1]] += mask.astype(np.int32)

    def _shape_mask(self, sh, bb, r):
        i0, j0, i1, j1 = _grid_box(_mm(bb.GetLeft()) - r, _mm(bb.GetTop()) - r, _mm(bb.GetRight()) + r, _mm(bb.GetBottom()) + r)
        if i1 < i0 or j1 < j0:
            return None
        mask = np.zeros((j1 - j0 + 1, i1 - i0 + 1), dtype=bool)
        rr = pcbnew.FromMM(r)
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                if sh.Collide(pcbnew.VECTOR2I(pcbnew.FromMM(BX0 + i * STEP), pcbnew.FromMM(BY0 + j * STEP)), rr):
                    mask[j - j0, i - i0] = True
        return i0, j0, mask

    def _pad_rasters(self, pad):
        out = []
        bb = pad.GetBoundingBox()
        for li, lay in enumerate((pcbnew.F_Cu, pcbnew.B_Cu)):
            if pad.IsOnLayer(lay):
                sh = pad.GetEffectiveShape(lay)
                for kind, r in ((li, CLR + W / 2 + MARGIN), (2, CLR + VIA_D / 2 + MARGIN)):
                    m = self._shape_mask(sh, bb, r)
                    if m:
                        out.append((kind, *m))
        return out

    def _load(self):
        b = self.board
        X, Y = np.meshgrid(BX0 + STEP * np.arange(NX), BY0 + STEP * np.arange(NY))
        for kind, r in ((0, EDGE + W / 2 + MARGIN), (1, EDGE + W / 2 + MARGIN), (2, EDGE + VIA_D / 2 + MARGIN)):
            self.fixed[kind] += ((X < BX0 + r) | (X > BX1 - r) | (Y < BY0 + r) | (Y > BY1 - r)).astype(np.int32)
        for t in sorted(b.GetTracks(), key=lambda t: (t.GetNetname(), t.GetClass(), t.GetStart().x, t.GetStart().y, t.GetEnd().x,
                                                      t.GetEnd().y, 0 if t.GetClass() == "PCB_VIA" else t.GetLayer())):
            net = t.GetNetname()
            if t.GetClass() == "PCB_VIA":
                x, y = _mm(t.GetPosition().x), _mm(t.GetPosition().y)
                if ripable(net):
                    self.nets.setdefault(net, []).append(Item("v", x, y))
                    continue
                d, dr = _mm(t.GetWidth(pcbnew.F_Cu)), _mm(t.GetDrillValue())
                for lay in (0, 1):
                    self._add_fixed(lay, seg_mask(x, y, x, y, d / 2 + CLR + W / 2 + MARGIN))
                self._add_fixed(2, seg_mask(x, y, x, y, max(d / 2 + VIA_D / 2 + CLR + MARGIN, dr / 2 + VIA_DRILL / 2 + HOLE_GAP)))
                continue
            lay = {pcbnew.F_Cu: 0, pcbnew.B_Cu: 1}.get(t.GetLayer())
            if lay is None:
                continue  # inner-layer copper: no obstacle for the outer-layer search (vias get antipads from the fill)
            w = _mm(t.GetWidth())
            ax, ay, bx, by = _mm(t.GetStart().x), _mm(t.GetStart().y), _mm(t.GetEnd().x), _mm(t.GetEnd().y)
            if ripable(net):
                self.nets.setdefault(net, []).append(Item("s", ax, ay, bx, by, lay, w))
                continue
            for kind, r in ((lay, w / 2 + CLR + W / 2 + MARGIN), (2, w / 2 + CLR + VIA_D / 2 + MARGIN)):
                self._add_fixed(kind, seg_mask(ax, ay, bx, by, r))
        for fp in sorted(b.GetFootprints(), key=lambda f: f.GetReference()):
            for pad in sorted(fp.Pads(), key=lambda p: (p.GetNumber(), p.GetPosition().x, p.GetPosition().y)):
                net = pad.GetNetname()
                rs = self._pad_rasters(pad)
                for kind, i0, j0, mask in rs:
                    self._add_fixed(kind, (i0, j0, mask))
                if pad.HasHole():
                    p, ds = pad.GetPosition(), pad.GetDrillSize()
                    self._add_fixed(2, seg_mask(_mm(p.x), _mm(p.y), _mm(p.x), _mm(p.y), _mm(max(ds.x, ds.y)) / 2 + VIA_DRILL / 2 + HOLE_GAP))
                if ripable(net):
                    self.own_pad.setdefault(net, []).extend(rs)
                    bb = pad.GetBoundingBox()
                    self.pads.setdefault(net, []).append({
                        "ref": f"{fp.GetReference()}.{pad.GetNumber()}", "x": _mm(pad.GetPosition().x), "y": _mm(pad.GetPosition().y),
                        "box": (_mm(bb.GetLeft()), _mm(bb.GetTop()), _mm(bb.GetRight()), _mm(bb.GetBottom())),
                        "layers": [0, 1] if pad.HasHole() else ([0] if pad.IsOnLayer(pcbnew.F_Cu) else [1])})
            for g in fp.GraphicalItems():
                for li, lay in enumerate((pcbnew.F_Cu, pcbnew.B_Cu)):
                    if g.IsOnLayer(lay):
                        sh, bb = g.GetEffectiveShape(lay), g.GetBoundingBox()
                        for kind, r in ((li, CLR + W / 2 + MARGIN), (2, CLR + VIA_D / 2 + MARGIN)):
                            m = self._shape_mask(sh, bb, r)
                            if m:
                                self._add_fixed(kind, m)
        for z in b.Zones():
            if not z.GetIsRuleArea():
                continue
            ol, bb = z.Outline(), z.GetBoundingBox()
            i0, j0, i1, j1 = _grid_box(_mm(bb.GetLeft()) - 0.5, _mm(bb.GetTop()) - 0.5, _mm(bb.GetRight()) + 0.5, _mm(bb.GetBottom()) + 0.5)
            for li, lay in enumerate((pcbnew.F_Cu, pcbnew.B_Cu)):
                if not z.IsOnLayer(lay):
                    continue
                for j in range(j0, j1 + 1):
                    for i in range(i0, i1 + 1):
                        p = pcbnew.VECTOR2I(pcbnew.FromMM(BX0 + i * STEP), pcbnew.FromMM(BY0 + j * STEP))
                        if ol.Collide(p, pcbnew.FromMM(W / 2 + MARGIN + 0.05)):
                            self.fixed[li][j, i] += 1
                        if ol.Collide(p, pcbnew.FromMM(VIA_D / 2 + MARGIN + 0.05)):
                            self.fixed[2][j, i] += 1
        for net in sorted(self.nets):
            for it in self.nets[net]:
                self._add_item(net, it)

    def virtual_net(self, net, refs):
        """Model of only these pads of a plane net (the rest of its copper stays an ordinary obstacle)."""
        self.pads[net], self.own_pad[net] = [], []
        self.nets.setdefault(net, [])
        for ref in refs:
            r0, num = ref.split(".")
            fp = self.board.FindFootprintByReference(r0)
            pad = [p for p in fp.Pads() if p.GetNumber() == num][0]
            self.own_pad[net].extend(self._pad_rasters(pad))
            bb = pad.GetBoundingBox()
            self.pads[net].append({"ref": ref, "x": _mm(pad.GetPosition().x), "y": _mm(pad.GetPosition().y),
                                   "box": (_mm(bb.GetLeft()), _mm(bb.GetTop()), _mm(bb.GetRight()), _mm(bb.GetBottom())),
                                   "layers": [0, 1] if pad.HasHole() else ([0] if pad.IsOnLayer(pcbnew.F_Cu) else [1])})

    # -- items on the grids --------------------------------------------------------------------------------------------
    def _add_item(self, net, it, register=False):
        if it.rasters is None:
            it.build()
        it.factor = PROTECT if net in WIDE else RIP_COST * (1.0 + 2.0 * self.hist.get(net, 0))
        for kind, i0, j0, mask in it.rasters:
            self.cost[kind][j0:j0 + mask.shape[0], i0:i0 + mask.shape[1]] += it.factor * mask.astype(np.float32)
        if register:
            self.nets.setdefault(net, []).append(it)

    def _remove_item(self, net, it):
        for kind, i0, j0, mask in it.rasters:
            self.cost[kind][j0:j0 + mask.shape[0], i0:i0 + mask.shape[1]] -= it.factor * mask.astype(np.float32)
        self.nets[net].remove(it)

    # -- connectivity of a net -----------------------------------------------------------------------------------------
    @staticmethod
    def _on_seg(px, py, s):
        dx, dy = s.bx - s.ax, s.by - s.ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((px - s.ax) * dx + (py - s.ay) * dy) / l2))
        return math.hypot(px - s.ax - t * dx, py - s.ay - t * dy) < 0.003

    def clusters(self, net):
        """Components of pads + items of the net, biggest first; each = {'pads': [...], 'items': [...]}."""
        items = self.nets.get(net, [])
        pads = self.pads.get(net, [])
        n = len(pads) + len(items)
        parent = list(range(n))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        def union(a, b):
            a, b = find(a), find(b)
            if a != b:
                parent[max(a, b)] = min(a, b)

        offs = len(pads)
        for ii, it in enumerate(items):
            pts = [(it.ax, it.ay)] + ([(it.bx, it.by)] if it.kind == "s" else [])
            for pi, p in enumerate(pads):
                x0, y0, x1, y1 = p["box"]
                if it.kind == "s":
                    if it.layer not in p["layers"]:
                        continue
                if any(x0 - 1e-6 <= x <= x1 + 1e-6 and y0 - 1e-6 <= y <= y1 + 1e-6 for x, y in pts):
                    union(ii + offs, pi)
            for jj in range(ii + 1, len(items)):
                ot = items[jj]
                same = it.kind == "v" or ot.kind == "v" or it.layer == ot.layer
                if not same:
                    continue
                if it.kind == "s" and ot.kind == "s":
                    a = any(self._on_seg(x, y, ot) for x, y in pts) or any(
                        self._on_seg(x, y, it) for x, y in ((ot.ax, ot.ay), (ot.bx, ot.by)))
                elif it.kind == "v" and ot.kind == "s":
                    a = self._on_seg(it.ax, it.ay, ot)
                elif it.kind == "s" and ot.kind == "v":
                    a = self._on_seg(ot.ax, ot.ay, it)
                else:
                    a = abs(it.ax - ot.ax) < 1e-6 and abs(it.ay - ot.ay) < 1e-6
                if a:
                    union(ii + offs, jj + offs)
        comp: dict[int, dict] = {}
        for pi, p in enumerate(pads):
            comp.setdefault(find(pi), {"pads": [], "items": []})["pads"].append(p)
        for ii, it in enumerate(items):
            comp.setdefault(find(ii + offs), {"pads": [], "items": []})["items"].append(it)
        out = list(comp.values())
        out.sort(key=lambda c: (-len(c["pads"]), -len(c["items"]), min((p["ref"] for p in c["pads"]), default="")))
        return out

    def prune(self, net) -> int:
        """Remove dead ends of a net: a segment end that touches no pad, via or other segment of the net (left by a
        cut, M: kicad-cli 'track_dangling' warnings after the first rev 3 replays), and vias that touch nothing."""
        n = 0
        while True:
            items = self.nets.get(net, [])
            pads = self.pads.get(net, [])
            drop = None
            for it in items:
                ends = [(it.ax, it.ay)] + ([(it.bx, it.by)] if it.kind == "s" else [])
                for x, y in ends:
                    lay = it.layer if it.kind == "s" else None
                    ok = any(p["box"][0] - 1e-6 <= x <= p["box"][2] + 1e-6 and p["box"][1] - 1e-6 <= y <= p["box"][3] + 1e-6
                             and (lay is None or lay in p["layers"]) for p in pads)
                    if not ok and it.kind == "v":  # a via needs two segments (or a pad)
                        ok = sum(1 for ot in items if ot.kind == "s" and self._on_seg(x, y, ot)) >= 2
                    elif not ok:
                        for ot in items:
                            if ot is it:
                                continue
                            if ot.kind == "v":
                                if abs(ot.ax - x) < 0.003 and abs(ot.ay - y) < 0.003:
                                    ok = True
                                    break
                            elif (lay is None or ot.layer == lay) and self._on_seg(x, y, ot):
                                ok = True
                                break
                    if not ok:
                        drop = it
                        break
                if drop is not None:
                    break
            if drop is None:
                return n
            self._remove_item(net, drop)
            n += 1

    # -- search --------------------------------------------------------------------------------------------------------
    def _cell(self, x, y):
        return int(round((x - BX0) / STEP)), int(round((y - BY0) / STEP))

    def _terminals(self, comp) -> list[tuple]:
        """(x, y, layers) of everything a new path may start from / end on in a component."""
        out = [(p["x"], p["y"], tuple(p["layers"])) for p in comp["pads"]]
        for it in comp["items"]:
            if it.kind == "v":
                out.append((it.ax, it.ay, (0, 1)))
            else:
                out.append((it.ax, it.ay, (it.layer,)))
                out.append((it.bx, it.by, (it.layer,)))
        return out

    def search(self, net: str, starts: list, targets: list, region: float, allow_rip: bool = True, width: float = W):
        """A* from the terminals ``starts`` to ``targets``. Returns (cells, start_terminal, target_terminal) or None.
        ``width`` > W (WIDE nets, never with rip-up): obstacles are dilated by the extra half width."""
        cs = {}
        for t in sorted(starts):
            cs.setdefault(self._cell(t[0], t[1]), t)
        ct = {}
        for t in sorted(targets):
            ct.setdefault(self._cell(t[0], t[1]), t)
        ii = [c[0] for c in list(cs) + list(ct)]
        jj = [c[1] for c in list(cs) + list(ct)]
        m = int(region / STEP)
        i0, j0, i1, j1 = max(0, min(ii) - m), max(0, min(jj) - m), min(NX - 1, max(ii) + m), min(NY - 1, max(jj) + m)
        nx = i1 - i0 + 1
        ny = j1 - j0 + 1
        fx = [self.fixed[k][j0:j1 + 1, i0:i1 + 1].copy() for k in range(3)]
        for kind, a0, b0, mask in self.own_pad.get(net, []):
            ax0, bx0 = max(a0, i0), max(b0, j0)
            ax1, bx1 = min(a0 + mask.shape[1] - 1, i1), min(b0 + mask.shape[0] - 1, j1)
            if ax1 >= ax0 and bx1 >= bx0:
                fx[kind][bx0 - j0:bx1 - j0 + 1, ax0 - i0:ax1 - i0 + 1] -= \
                    mask[bx0 - b0:bx1 - b0 + 1, ax0 - a0:ax1 - a0 + 1].astype(np.int32)
        blocked = [fx[k] > 0 for k in range(3)]
        # the net's own copper is no obstacle and costs nothing (a path has to start and end on it)
        lc = [self.cost[k][j0:j1 + 1, i0:i1 + 1].copy() for k in range(3)]
        for it in self.nets.get(net, []):
            for kind, a0, b0, mask in it.rasters:
                ax0, bx0 = max(a0, i0), max(b0, j0)
                ax1, bx1 = min(a0 + mask.shape[1] - 1, i1), min(b0 + mask.shape[0] - 1, j1)
                if ax1 >= ax0 and bx1 >= bx0:
                    lc[kind][bx0 - j0:bx1 - j0 + 1, ax0 - i0:ax1 - i0 + 1] -= \
                        it.factor * mask[bx0 - b0:bx1 - b0 + 1, ax0 - a0:ax1 - a0 + 1].astype(np.float32)
        for k in range(3):
            # a protected net (WIDE, factor PROTECT) is never crossed; without rip-up nothing foreign is crossed
            blocked[k] |= lc[k] > (1e-3 if not allow_rip else PROTECT / 2)
        if width > W + 1e-6:
            r = (width - W) / 2 + 0.0707
            off = [(a, b) for a in range(-4, 5) for b in range(-4, 5) if (a or b) and math.hypot(a, b) * STEP <= r]
            for k in (0, 1):
                base = blocked[k]
                grown = base.copy()
                for a, b in off:
                    src = base[max(0, -b):base.shape[0] - max(0, b), max(0, -a):base.shape[1] - max(0, a)]
                    grown[max(0, b):max(0, b) + src.shape[0], max(0, a):max(0, a) + src.shape[1]] |= src
                blocked[k] = grown
        Xg, Yg = np.meshgrid(BX0 + STEP * np.arange(i0, i1 + 1), BY0 + STEP * np.arange(j0, j1 + 1))
        # own vias keep the hole-to-hole rule
        for it in self.nets.get(net, []):
            if it.kind == "v":
                blocked[2] |= (Xg - it.ax) ** 2 + (Yg - it.ay) ** 2 <= (VIA_DRILL + HOLE_GAP) ** 2
        for p in self.pads.get(net, []):
            x0, y0, x1, y1 = p["box"]
            inside = (Xg >= x0 + W / 2) & (Xg <= x1 - W / 2) & (Yg >= y0 + W / 2) & (Yg <= y1 - W / 2)
            for lay in p["layers"]:
                blocked[lay] &= ~inside
            if len(p["layers"]) == 1:
                r = VIA_D / 2 + 0.1
                blocked[2] |= (Xg >= x0 - r) & (Xg <= x1 + r) & (Yg >= y0 - r) & (Yg <= y1 + r)
        blk = [blocked[k].ravel().tolist() for k in range(3)]
        cst = [np.clip(lc[k], 0.0, None).ravel().tolist() for k in range(3)]
        tset = {(c[0] - i0, c[1] - j0): v[2] for c, v in ct.items()}
        goals = sorted(tset)
        sample = goals if len(goals) <= 6 else [goals[k * len(goals) // 6] for k in range(6)]
        h0 = lambda i, j: STEP * min(math.hypot(i - a, j - b) for a, b in sample)
        pq, dist, prev, cnt = [], {}, {}, 0
        for c, t in cs.items():
            sx, sy = c[0] - i0, c[1] - j0
            for L in t[2]:
                s = (sx, sy, L)
                dist[s] = 0.0
                heapq.heappush(pq, (h0(sx, sy), cnt, 0.0, s))
                cnt += 1
        goal = None
        pops = 0
        while pq and pops < 400_000:
            _f, _c, d, u = heapq.heappop(pq)
            if d > dist.get(u, 1e18) + 1e-9:
                continue
            pops += 1
            i, j, L = u
            ls = tset.get((i, j))
            if ls is not None and L in ls and not (u in dist and dist[u] == 0.0):
                goal = u
                break
            k = j * nx + i
            for di, dj, c in OCT:
                ni, nj = i + di, j + dj
                if not (0 <= ni < nx and 0 <= nj < ny):
                    continue
                kk = nj * nx + ni
                if blk[L][kk]:
                    continue
                nd = d + c * STEP + cst[L][kk]
                v = (ni, nj, L)
                if nd < dist.get(v, 1e18) - 1e-9:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd + h0(ni, nj), cnt, nd, v))
                    cnt += 1
            if not blk[2][k] and not blk[1 - L][k]:
                v = (i, j, 1 - L)
                nd = d + VIA_COST + cst[2][k] + cst[1 - L][k]
                if nd < dist.get(v, 1e18) - 1e-9:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd + h0(i, j), cnt, nd, v))
                    cnt += 1
        if goal is None:
            return None
        path = [goal]
        while path[-1] in prev:
            path.append(prev[path[-1]])
        path.reverse()
        cells = [(i + i0, j + j0, L) for i, j, L in path]
        return cells, cs[(cells[0][0], cells[0][1])], ct[(cells[-1][0], cells[-1][1])]

    @staticmethod
    def polyline(cells, start, end, W=W):
        """Items for a grid path, the end points replaced by the exact terminal coordinates."""
        pts = [(BX0 + i * STEP, BY0 + j * STEP, L) for i, j, L in cells]
        pts[0] = (start[0], start[1], pts[0][2])
        pts[-1] = (end[0], end[1], pts[-1][2])
        items = []
        anchor = 0
        for k in range(1, len(pts)):
            if pts[k][2] != pts[k - 1][2]:
                if k - 1 > anchor:
                    items.append(Item("s", pts[anchor][0], pts[anchor][1], pts[k - 1][0], pts[k - 1][1], pts[k - 1][2], W))
                items.append(Item("v", pts[k][0], pts[k][1]))
                anchor = k
                continue
            if k + 1 < len(pts) and pts[k + 1][2] == pts[k][2]:
                ax, ay, _ = pts[anchor]
                bx, by, _ = pts[k]
                cx, cy, _ = pts[k + 1]
                if abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) < 1e-6 and k + 1 < len(pts) - 1:
                    continue
                items.append(Item("s", ax, ay, bx, by, pts[k][2], W))
                anchor = k
        if anchor < len(pts) - 1:
            items.append(Item("s", pts[anchor][0], pts[anchor][1], pts[-1][0], pts[-1][1], pts[-1][2], W))
        return items

    # -- rip-up --------------------------------------------------------------------------------------------------------
    def rip_conflicts(self, net: str, cells, width: float = W) -> list[str]:
        """Cut every piece of other ripable nets the path passes through; returns those nets."""
        vias = [(b[0], b[1]) for a, b in zip(cells, cells[1:]) if a[2] != b[2]]
        if width > W + 1e-6:  # a wider track occupies the neighbouring cells too
            r = (width - W) / 2 + 0.0707
            off = [(a, b) for a in range(-4, 5) for b in range(-4, 5) if math.hypot(a, b) * STEP <= r]
            cells = sorted({(i + a, j + b, L) for i, j, L in cells for a, b in off})
        hit = set()
        cset = set(cells)
        for other in sorted(self.nets):
            if other == net:
                continue
            for it in list(self.nets[other]):
                touched = []
                for kind, i0, j0, mask in it.rasters:
                    h, w = mask.shape
                    if kind == 2:
                        touched += [(i, j) for i, j in vias if i0 <= i < i0 + w and j0 <= j < j0 + h and mask[j - j0, i - i0]]
                    else:
                        touched += [(i, j) for i, j, L in cells if L == kind and i0 <= i < i0 + w and j0 <= j < j0 + h and mask[j - j0, i - i0]]
                if not touched:
                    continue
                hit.add(other)
                self.cut(other, it, touched)
        return sorted(hit)

    def cut(self, net, it, touched):
        self._remove_item(net, it)
        self.hist[net] = self.hist.get(net, 0) + 1
        if it.kind == "v":
            return
        dx, dy = it.bx - it.ax, it.by - it.ay
        ln = math.hypot(dx, dy)
        if ln < 1e-6:
            return
        ts = [((BX0 + i * STEP - it.ax) * dx + (BY0 + j * STEP - it.ay) * dy) / (ln * ln) for i, j in touched]
        t0 = max(0.0, min(ts) - TRIM / ln)
        t1 = min(1.0, max(ts) + TRIM / ln)
        for ta, tb in ((0.0, t0), (t1, 1.0)):
            if (tb - ta) * ln > 0.3:
                self._add_item(net, Item("s", it.ax + dx * ta, it.ay + dy * ta, it.ax + dx * tb, it.ay + dy * tb, it.layer, it.w), register=True)


class Solver:
    """Transactional rip-up: an open connection is completed only if every net it cuts can be reconnected too (up to
    ``depth`` levels of cutting); otherwise everything is restored and the victims get a higher crossing cost."""

    KEEP = 6            # alternative paths tried per connection and level
    BUDGET = 3000       # searches in total (deterministic stop)

    def __init__(self, r: Router, log: dict):
        self.r, self.log = r, log
        self.searches = 0

    def snapshot(self):
        r = self.r
        return [a.copy() for a in r.cost], {k: list(v) for k, v in r.nets.items()}, dict(r.hist)

    def restore(self, snap):
        r = self.r
        r.cost, r.nets, r.hist = snap[0], snap[1], snap[2]

    def recost(self, net):
        r = self.r
        items = r.nets.get(net, [])
        for n, it in enumerate(items):
            # a clone: outer snapshots still hold the old item, whose factor must match their cost grids (M: mutating
            # it left negative cost residue, a cell looked free and a via landed on a track of another net)
            new = Item(it.kind, it.ax, it.ay, it.bx, it.by, it.layer, it.w)
            new.rasters = it.rasters
            new.factor = PROTECT if net in WIDE else RIP_COST * (1.0 + 4.0 * r.hist.get(net, 0))
            for kind, i0, j0, mask in it.rasters:
                r.cost[kind][j0:j0 + mask.shape[0], i0:i0 + mask.shape[1]] += (new.factor - it.factor) * mask.astype(np.float32)
            items[n] = new

    def live_clusters(self, name):
        r = self.r
        live = []
        for c in r.clusters(name):
            if c["pads"]:
                live.append(c)
            else:
                for it in c["items"]:
                    r._remove_item(name, it)
        return live

    def find(self, name, src, main, allow, width=W):
        r = self.r
        for region in ((3.0, 8.0, 20.0) if not allow else (3.0, 8.0, 20.0)):
            if self.searches >= self.BUDGET:
                return None
            self.searches += 1
            res = r.search(name, r._terminals(src), r._terminals(main), region, allow, width)
            if res:
                return res
        return None

    def connect(self, name, depth):
        """Join all clusters of ``name``. True: done (state changed). False: state as on entry."""
        r = self.r
        entry = self.snapshot()
        while True:
            live = self.live_clusters(name)
            if len(live) <= 1:
                return True
            main, src = live[0], live[-1]
            found = None
            used_w = W
            for w in WIDE.get(name, (W,)):
                found = self.find(name, src, main, False, w)
                if found:
                    used_w = w
                    break
            if found:
                cells, a, b = found
                for it in Router.polyline(cells, a, b, used_w):
                    r._add_item(name, it, register=True)
                if name in WIDE:
                    self.log.setdefault("wide_net_completions", []).append(
                        {"net": name, "width_mm": used_w, "cells": len(cells)})
                continue
            if depth <= 0:
                self.restore(entry)
                return False
            ok = False
            attempts = [(w, k) for w in WIDE.get(name, (W,)) for k in range(self.KEEP)]
            for used_w, _k in attempts:
                snap = self.snapshot()
                found = self.find(name, src, main, True, used_w)
                if not found:
                    continue
                cells, a, b = found
                victims = r.rip_conflicts(name, cells, used_w)
                for it in Router.polyline(cells, a, b, used_w):
                    r._add_item(name, it, register=True)
                if name in WIDE:
                    self.log.setdefault("wide_net_completions", []).append(
                        {"net": name, "width_mm": used_w, "cells": len(cells), "ripped": victims})
                done = True
                for v in victims:
                    if not self.connect(v, depth - 1):
                        done = False
                        bad = v
                        break
                if done:
                    for v in victims:
                        self.log["ripped"][v] = self.log["ripped"].get(v, 0) + 1
                    ok = True
                    break
                self.restore(snap)
                r.hist[bad] = r.hist.get(bad, 0) + 1
                self.recost(bad)
            if not ok:
                self.restore(entry)
                return False


def run(board, open_nets: list[str]) -> dict:
    r = Router(board)
    open_rip = sorted({n for n in open_nets if ripable(n)})
    log = {"open_nets_in": sorted(set(open_nets)), "not_ripable": sorted({n for n in open_nets if not ripable(n)}),
           "iterations": 0, "ripped": {}, "solved_at_depth": {}}
    original = {net: {it.key() for it in items} for net, items in r.nets.items()}
    s = Solver(r, log)
    queue = list(open_rip)
    joined = []
    for depth in (0, 1, 2, 3):
        if depth == 3:  # the deepest level only for the nets that may not stay open at class width
            queue[:] = [n for n in queue if n in WIDE] + [n for n in queue if n not in WIDE]
        for name in list(queue):
            if depth == 3 and name not in WIDE:
                continue
            if s.connect(name, depth):
                log["solved_at_depth"][name] = depth
                queue.remove(name)
            log["iterations"] = s.searches
            print(f"ripup depth {depth} {name}: {'OK' if name not in queue else 'open'} (searches {s.searches})", flush=True)
        if not queue:
            break
    fail_set = set()
    for net, a_ref, b_ref in ISLAND_JOINS:
        if net in log["not_ripable"]:
            r.virtual_net(net, [a_ref, b_ref])
            for depth in (0, 1, 2):
                if s.connect(net, depth):
                    joined.append(net)
                    log.setdefault("islands_joined", []).append({"net": net, "pads": [a_ref, b_ref], "depth": depth})
                    break
    log["not_ripable"] = sorted(set(log["not_ripable"]) - set(joined))
    still = sorted(set(queue) | fail_set)
    log["still_open"] = still
    log["iterations_used"] = log.pop("iterations")
    log["converged"] = not still
    log["open_nets_before"], log["open_nets_after"] = len(set(open_nets)), len(still) + len(log["not_ripable"])
    log["applied"] = bool(log["open_nets_after"] < log["open_nets_before"])
    if not log["applied"]:
        return log
    pruned = {}
    for net in sorted(r.nets):
        if net not in VIRTUAL and {it.key() for it in r.nets[net]} != original.get(net, set()):
            k = r.prune(net)
            if k:
                pruned[net] = k
    log["dead_ends_pruned"] = pruned
    changed = sorted(n for n, items in r.nets.items() if {it.key() for it in items} != original.get(n, set()))
    # a virtual net's model holds only the new copper: its other tracks are not the model's to delete
    log["nets_changed"] = changed
    by_net: dict[str, dict] = {}
    for t in board.GetTracks():
        if t.GetNetname() in changed:
            k = (Item("v", _mm(t.GetPosition().x), _mm(t.GetPosition().y)).key() if t.GetClass() == "PCB_VIA" else
                 Item("s", _mm(t.GetStart().x), _mm(t.GetStart().y), _mm(t.GetEnd().x), _mm(t.GetEnd().y),
                      {pcbnew.F_Cu: 0, pcbnew.B_Cu: 1}[t.GetLayer()], 0).key())
            by_net.setdefault(t.GetNetname(), {})[k] = t
    for name in changed:
        keep = {it.key(): it for it in r.nets[name]}
        for k, t in by_net.get(name, {}).items():
            if k not in keep and name not in VIRTUAL:
                board.Delete(t)
        nobj = board.FindNet(name)
        for k in sorted(keep):
            if k in by_net.get(name, {}):
                continue
            it = keep[k]
            if it.kind == "v":
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pcbnew.VECTOR2I(_nm(it.ax), _nm(it.ay)))
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetWidth(pcbnew.F_Cu, pcbnew.FromMM(VIA_D))
                v.SetWidth(pcbnew.B_Cu, pcbnew.FromMM(VIA_D))
                v.SetDrill(pcbnew.FromMM(VIA_DRILL))
                v.SetNet(nobj)
                v.SetLocked(True)
                board.Add(v)
            else:
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(pcbnew.VECTOR2I(_nm(it.ax), _nm(it.ay)))
                t.SetEnd(pcbnew.VECTOR2I(_nm(it.bx), _nm(it.by)))
                t.SetWidth(pcbnew.FromMM(it.w))
                t.SetLayer(pcbnew.F_Cu if it.layer == 0 else pcbnew.B_Cu)
                t.SetNet(nobj)
                t.SetLocked(True)
                board.Add(t)
    return log
