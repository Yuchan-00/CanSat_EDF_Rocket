"""Deterministic plane fan-out for the 4-layer board (KiCad python, called from route.py).

Invariant: every SMD pad of a plane net (GND -> In1.Cu, +5V -> In2.Cu) gets, where a legal spot exists, a short
F.Cu stub to its own through via; the via and the stub are checked here against every copper item of another net
on F.Cu and B.Cu (clearance CLR), every drill hole (hole-to-hole MIN_HOLE_GAP, rule 0.5 mm), the board edge
(copper-to-edge 0.3 mm), the ESP-12E antenna rule area and the pads themselves (no via in a pad). Fixed candidate
order (pads by reference / number, spots by distance then angle) -> the same result every run. Pads with no legal
spot are listed and left to Freerouting's plane stage.

Why (M, route.py logs): Freerouting's own fan-out to the In1 / In2 planes kept only 7 vias of 133 fanned pads in its
final session, which left 35 +5V and 33 GND connections open; with the vias placed first every later stage treats
them as fixed obstacles and the planes connect the pads.
"""

from __future__ import annotations

import math

import pcbnew

VIA_D = 0.6
DRILL = 0.3
CLR = 0.15          # >= the 0.13 mm class clearance (M: 0.15 -> 25 open after routing, 0.2 -> 30 open; KiCad DRC clean)
STUB_W = 0.3        # F.Cu stub pad -> via (short; current of one pad)
MIN_HOLE_GAP = 0.55  # drill edge to drill edge (board rule 0.5 mm)
EDGE = 0.3 + VIA_D / 2 + 0.05
#: relayout step (review J2): via copper >= 0.4 mm from the two rail edges y 65 / y 135 (stricter than the rule)
EDGE_Y = 0.4 + VIA_D / 2 + 0.05
BX0, BY0, BX1, BY1 = 62.0, 65.0, 138.0, 135.0
MCU_REFS = ("U1", "U2", "U3", "U4")
#: rev 3: pads whose plane vias / tracks build_pcb.power_preroutes already placed (boost block) are skipped as well
#: relayout step: R63 pad 1 / C58 pad 1 are the feedback sense node (own track to C57 pad 1) - no plane via at them
#: redesign2 Layout stage: R64 pad 2 (GND) carries its own locked GND via (build_pcb.power_preroutes) - no second via.
#: The candidate box below stays the main rectangle (BY1 = 135): no fan-out via is placed in the outline tab, whose
#: pads are all pre-routed (U8, C57, C43, C41, R63, C58, R64; L5 carries no GND / +5V pad).
PREROUTED_REFS = MCU_REFS + ("U8", "C57", "C43", "C41", "R63", "C58", "R64")


def mm(v: float) -> int:
    return pcbnew.FromMM(v)


def _ref_key(ref: str) -> tuple:
    p = ref.rstrip("0123456789")
    n = ref[len(p):]
    return (p, int(n) if n.isdigit() else 0)


def _bb(item) -> tuple[float, float, float, float]:
    b = item.GetBoundingBox()
    return (pcbnew.ToMM(b.GetLeft()), pcbnew.ToMM(b.GetTop()), pcbnew.ToMM(b.GetRight()), pcbnew.ToMM(b.GetBottom()))


def run(board, nets: tuple[str, ...] = ("GND", "+5V")) -> dict:
    cu = (pcbnew.F_Cu, pcbnew.B_Cu)
    obst = {lay: [] for lay in cu}  # (netcode, SHAPE, bbox mm)
    pads_f = []
    holes = []  # (x, y, r) mm
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            for lay in cu:
                if pad.IsOnLayer(lay):
                    obst[lay].append((pad.GetNetCode(), pad.GetEffectiveShape(lay), _bb(pad)))
            if pad.IsOnLayer(pcbnew.F_Cu):
                pads_f.append(pad)
            if pad.HasHole():  # rev 2c: every THT pad hole (rev 2b took only one stale pad per footprint)
                ds = pad.GetDrillSize()
                holes.append((pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y), pcbnew.ToMM(max(ds.x, ds.y)) / 2))
        for g in fp.GraphicalItems():  # copper drawings inside footprints (the NT1 net-tie bridge) - no net
            for lay in cu:
                if g.IsOnLayer(lay):
                    obst[lay].append((-1, g.GetEffectiveShape(lay), _bb(g)))
    for t in board.GetTracks():
        for lay in cu:
            if t.IsOnLayer(lay):
                obst[lay].append((t.GetNetCode(), t.GetEffectiveShape(lay), _bb(t)))
        if t.GetClass() == "PCB_VIA":
            holes.append((pcbnew.ToMM(t.GetPosition().x), pcbnew.ToMM(t.GetPosition().y), pcbnew.ToMM(t.GetDrillValue()) / 2))
    rule_areas = [z.Outline() for z in board.Zones() if z.GetIsRuleArea()]
    track_ends = {}
    for t in board.GetTracks():
        if t.GetClass() == "PCB_TRACK":
            track_ends.setdefault(t.GetNetCode(), []).extend([t.GetStart(), t.GetEnd()])

    todo = []
    for pad in pads_f:
        if pad.GetNetname() not in nets or pad.HasHole():
            continue
        ref = pad.GetParentFootprint().GetReference()
        if ref in PREROUTED_REFS and any(pad.HitTest(p) for p in track_ends.get(pad.GetNetCode(), [])):
            continue  # MCU pin pre-routed to its decoupling cap: the cap pad gets the via
        todo.append((_ref_key(ref), pad.GetNumber(), pad))
    todo.sort(key=lambda t: (t[0], t[1]))

    placed, failed = [], []
    for _k, num, pad in todo:
        ref = pad.GetParentFootprint().GetReference()
        net = pad.GetNetCode()
        c = pad.GetPosition()
        cx, cy = pcbnew.ToMM(c.x), pcbnew.ToMM(c.y)
        bb = pad.GetBoundingBox()
        half = max(pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight())) / 2
        own = pad.GetEffectiveShape(pcbnew.F_Cu)
        R = half + 3.0
        local = {lay: [(n, sh) for n, sh, (x0, y0, x1, y1) in obst[lay]
                       if x1 > cx - R and x0 < cx + R and y1 > cy - R and y0 < cy + R] for lay in cu}
        lholes = [h for h in holes if abs(h[0] - cx) < R and abs(h[1] - cy) < R]
        done = False
        cands = []
        for k in range(0, 16):
            d = round(half * 0.5 + VIA_D / 2 + 0.25 + 0.1 * k, 3)
            for a in range(0, 360, 30):
                cands.append((d, a))
        for d, a in cands:
            vx = round(cx + d * math.cos(math.radians(a)), 3)
            vy = round(cy + d * math.sin(math.radians(a)), 3)
            if not (BX0 + EDGE <= vx <= BX1 - EDGE and BY0 + EDGE_Y <= vy <= BY1 - EDGE_Y):
                continue
            vp = pcbnew.VECTOR2I(mm(vx), mm(vy))
            if any(r.Contains(vp) or r.Collide(vp, mm(VIA_D / 2 + 0.3)) for r in rule_areas):
                continue
            if any(math.hypot(hx - vx, hy - vy) - hr - DRILL / 2 < MIN_HOLE_GAP for hx, hy, hr in lholes):
                continue
            vc = pcbnew.SHAPE_CIRCLE(vp, mm(VIA_D / 2))
            if own.Collide(vc, mm(0.05)):
                continue  # no via in the pad itself
            stub = pcbnew.SHAPE_SEGMENT(c, vp, mm(STUB_W))
            bad = False
            for lay in cu:
                for n, sh in local[lay]:
                    if lay == pcbnew.F_Cu and n == net:
                        continue
                    if n == net and lay == pcbnew.B_Cu:
                        continue
                    if sh.Collide(vc, mm(CLR)):
                        bad = True
                        break
                    if lay == pcbnew.F_Cu and sh.Collide(stub, mm(CLR)):
                        bad = True
                        break
                if bad:
                    break
            if bad:
                continue
            # other pads of the same net on F.Cu: the via must not sit in them either
            if any(p is not pad and p.GetNetCode() == net and p.GetEffectiveShape(pcbnew.F_Cu).Collide(vc, mm(0.05))
                   for p in pads_f if abs(pcbnew.ToMM(p.GetPosition().x) - vx) < 3 and abs(pcbnew.ToMM(p.GetPosition().y) - vy) < 3):
                continue
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(vp)
            v.SetViaType(pcbnew.VIATYPE_THROUGH)
            v.SetWidth(pcbnew.F_Cu, mm(VIA_D))
            v.SetDrill(mm(DRILL))
            v.SetNet(pad.GetNet())
            v.SetLocked(True)
            board.Add(v)
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(c)
            t.SetEnd(vp)
            t.SetWidth(mm(STUB_W))
            t.SetLayer(pcbnew.F_Cu)
            t.SetNet(pad.GetNet())
            t.SetLocked(True)
            board.Add(t)
            vbb = (vx - VIA_D / 2, vy - VIA_D / 2, vx + VIA_D / 2, vy + VIA_D / 2)
            for lay in cu:  # the items' own effective shapes (SWIG: a bare SHAPE_CIRCLE only collides with a SEG)
                obst[lay].append((net, v.GetEffectiveShape(lay), vbb))
            obst[pcbnew.F_Cu].append((net, t.GetEffectiveShape(pcbnew.F_Cu),
                                      (min(cx, vx), min(cy, vy), max(cx, vx), max(cy, vy))))
            holes.append((vx, vy, DRILL / 2))
            placed.append([ref, num, pad.GetNetname(), vx, vy])
            done = True
            break
        if not done:
            failed.append([ref, num, pad.GetNetname()])
    return {"tool": "scripts/fanout.py", "pads_considered": len(todo), "vias_placed": len(placed), "not_placed": failed,
            "via_mm": [VIA_D, DRILL], "stub_mm": STUB_W, "clearance_mm": CLR}
