"""Scripted, deterministic routing of the last unconnected items Freerouting leaves (KiCad python, from route.py).

Why (rev 2c, M): the rev-2c Freerouting runs end with 1-2 connections open in the FCU top area (U2 pin 61 PF0,
OPENLOG_RXI at Q8); a Freerouting rip-up pass over the whole board did not close them. Instead of hand edits, this
step routes each open connection that kicad-cli DRC reports, as a 0.2 mm signal track on F.Cu / B.Cu with 0.6 / 0.3 mm
through vias, by A* on a 0.1 mm grid around the two items.

Invariants
----------
* Only DRC 'unconnected_items' are touched, only signal nets and the low-current *_AVCC filter nets (never GND /
  +5V / +3V3 / servo / logic-battery nets), only new copper is added - nothing existing is moved or deleted.
* Every grid point of the path keeps the class clearance (0.13 mm) + half the track width + a 0.02 mm margin (covers
  the diagonal steps between grid points: at most 0.009 mm sagitta on a 0.14 mm step) to every pad, track, via and footprint copper of another net on that layer,
  0.3 mm + margin to the board edge, and stays outside the rule areas (ESP-12E antenna keep-out). A via also keeps
  0.5 mm drill-to-drill (+ margin) to every other hole, its 0.6 mm pad the clearance on BOTH outer layers, and never
  sits in or touches an SMD pad, not even one of its own net.
* Deterministic: fixed grid, fixed neighbour order, ties broken by coordinates. The result is checked by kicad-cli DRC
  afterwards (checks_exports.py); a connection with no legal path is left open and listed.
"""

from __future__ import annotations

import heapq
import math
import re

import pcbnew

STEP = 0.1
W = 0.2
CLR = 0.13
MARGIN = 0.02
VIA_D, VIA_DRILL = 0.6, 0.3
HOLE_GAP = 0.5 + 0.05
VIA_COST = 2.5
MARGIN_REGION = 5.0
BX0, BY0, BX1, BY1 = 62.0, 65.0, 138.0, 135.0
NO_TOUCH = {"GND", "+5V", "+3V3", "GPS_VCC", "SERVO_5V", "GND_PWR", "VBAT_LOGIC", "VBAT_LOGIC_RAW", "VBAT_LOGIC_SW", "BOOST_SW"}


def _mm(v) -> float:
    return pcbnew.ToMM(v)


def _bb(item):
    b = item.GetBoundingBox()
    return _mm(b.GetLeft()), _mm(b.GetTop()), _mm(b.GetRight()), _mm(b.GetBottom())


def track_widths(net: str) -> tuple[float, ...]:
    """Signal nets 0.2 mm; the AVCC filter nets 0.3 mm, else 0.2 mm (class 0.5 mm, but an ATmega128A AVCC pin draws
    < 1 mA with the ADC on - A, datasheet not on disk; 0.2 mm 1 oz carries ~1 A, E)."""
    # rev 3: never below the 0.2 mm class width (the rev 2c 0.15 mm fallback is gone; ripup.py takes over)
    return (0.3, W) if net.endswith("_AVCC") else (W,)


def route_one(board, net: str, a: tuple, b: tuple, la: int | None, lb: int | None, region: float = 5.0,
              w: float = W) -> dict:
    """a / b: (x, y) mm; la / lb: required layer (0 = F.Cu, 1 = B.Cu) or None (THT pad / via: either)."""
    netcode = board.FindNet(net).GetNetCode()
    lays = (pcbnew.F_Cu, pcbnew.B_Cu)
    x0 = max(BX0 + 0.5, min(a[0], b[0]) - region)
    x1 = min(BX1 - 0.5, max(a[0], b[0]) + region)
    y0 = max(BY0 + 0.5, min(a[1], b[1]) - region)
    y1 = min(BY1 - 0.5, max(a[1], b[1]) + region)
    nx, ny = int((x1 - x0) / STEP) + 1, int((y1 - y0) / STEP) + 1
    blocked = [bytearray(nx * ny) for _ in lays]   # track centre not allowed
    via_bad = bytearray(nx * ny)                    # via centre not allowed
    items = []
    for t in board.GetTracks():
        items.append(t)
    pads = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            items.append(p)
            pads.append(p)
        own_tie = any(p.GetNetCode() == netcode for p in fp.Pads())
        for g in fp.GraphicalItems():
            # copper drawings (the NT1 net-tie bridge) are obstacles, except in a footprint that carries this net:
            # the bridge already joins its own pads (DRC checks the result)
            if (g.IsOnLayer(pcbnew.F_Cu) or g.IsOnLayer(pcbnew.B_Cu)) and not own_tie:
                items.append(g)
    holes = []
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            holes.append((_mm(t.GetPosition().x), _mm(t.GetPosition().y), _mm(t.GetDrillValue()) / 2))
    for p in pads:
        if p.HasHole():
            ds = p.GetDrillSize()
            holes.append((_mm(p.GetPosition().x), _mm(p.GetPosition().y), _mm(max(ds.x, ds.y)) / 2))
    r_trk = CLR + w / 2 + MARGIN
    r_via = CLR + VIA_D / 2 + MARGIN

    def mark(item, li, lay, r, arr):
        bx0, by0, bx1, by1 = _bb(item)
        i0 = max(0, int(math.floor((bx0 - r - x0) / STEP)))
        i1 = min(nx - 1, int(math.ceil((bx1 + r - x0) / STEP)))
        j0 = max(0, int(math.floor((by0 - r - y0) / STEP)))
        j1 = min(ny - 1, int(math.ceil((by1 + r - y0) / STEP)))
        if i1 < i0 or j1 < j0:
            return
        sh = item.GetEffectiveShape(lay)
        rr = pcbnew.FromMM(r)
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                k = j * nx + i
                if arr[k]:
                    continue
                if sh.Collide(pcbnew.VECTOR2I(pcbnew.FromMM(x0 + i * STEP), pcbnew.FromMM(y0 + j * STEP)), rr):
                    arr[k] = 1

    for it in items:
        if it.GetNetCode() == netcode:
            if it.GetClass() == "PAD" and not it.HasHole():  # no via in or touching an own SMD pad (review PF-M3)
                for li, lay in enumerate(lays):
                    if it.IsOnLayer(lay):
                        mark(it, li, lay, VIA_D / 2 + 0.1, via_bad)
            continue
        for li, lay in enumerate(lays):
            if it.IsOnLayer(lay):
                mark(it, li, lay, r_trk, blocked[li])
                mark(it, li, lay, r_via, via_bad)
    # inside an own-net pad (shrunk by half the track width) a track adds no copper: always free (lets the path leave
    # a pad that other-net tracks pass at the minimum clearance)
    for p in pads:
        if p.GetNetCode() != netcode:
            continue
        px0, py0, px1, py1 = _bb(p)
        for li, lay in enumerate(lays):
            if not p.IsOnLayer(lay):
                continue
            for j in range(max(0, int(math.ceil((py0 + w / 2 - y0) / STEP))), min(ny - 1, int(math.floor((py1 - w / 2 - y0) / STEP))) + 1):
                for i in range(max(0, int(math.ceil((px0 + w / 2 - x0) / STEP))), min(nx - 1, int(math.floor((px1 - w / 2 - x0) / STEP))) + 1):
                    blocked[li][j * nx + i] = 0
    for z in board.Zones():  # rule areas (antenna keep-out): no copper at all
        if z.GetIsRuleArea():
            ol = z.Outline()
            for j in range(ny):
                for i in range(nx):
                    p = pcbnew.VECTOR2I(pcbnew.FromMM(x0 + i * STEP), pcbnew.FromMM(y0 + j * STEP))
                    if ol.Collide(p, pcbnew.FromMM(VIA_D / 2 + 0.05)):
                        blocked[0][j * nx + i] = blocked[1][j * nx + i] = via_bad[j * nx + i] = 1
    for j in range(ny):
        for i in range(nx):
            x, y = x0 + i * STEP, y0 + j * STEP
            if any(math.hypot(hx - x, hy - y) - hr - VIA_DRILL / 2 < HOLE_GAP for hx, hy, hr in holes):
                via_bad[j * nx + i] = 1

    def cell(p):
        return int(round((p[0] - x0) / STEP)), int(round((p[1] - y0) / STEP))

    sa, sb = cell(a), cell(b)
    starts = [0, 1] if la is None else [la]
    goals = [0, 1] if lb is None else [lb]
    # the end cells sit inside own-net copper: free them (own copper is not an obstacle; neighbours may be blocked by
    # another net's clearance ring only if the pad itself is that close - then the path search fails honestly)
    pq, dist, prev = [], {}, {}
    for L in starts:
        s = (sa[0], sa[1], L)
        dist[s] = 0.0
        heapq.heappush(pq, (math.hypot(sa[0] - sb[0], sa[1] - sb[1]) * STEP, 0.0, s))
    goal = None
    nbr = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    while pq:
        f, d, u = heapq.heappop(pq)
        if d > dist.get(u, 1e18) + 1e-12:
            continue
        if (u[0], u[1]) == sb and u[2] in goals:
            goal = u
            break
        i, j, L = u
        cand = []
        for di, dj, c in nbr:
            ii, jj = i + di, j + dj
            if 0 <= ii < nx and 0 <= jj < ny:
                free = not blocked[L][jj * nx + ii] or (abs(ii - sb[0]) <= 1 and abs(jj - sb[1]) <= 1 and (ii, jj) == sb)
                if free:
                    cand.append(((ii, jj, L), c * STEP))
        if not via_bad[j * nx + i] and not blocked[1 - L][j * nx + i]:
            cand.append(((i, j, 1 - L), VIA_COST))
        for v, c in cand:
            nd = d + c
            if nd < dist.get(v, 1e18) - 1e-12:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd + math.hypot(v[0] - sb[0], v[1] - sb[1]) * STEP, nd, v))
    if goal is None:
        return {"net": net, "routed": False, "reason": "no legal path on the 0.1 mm grid"}
    path = [goal]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    path.reverse()
    # exact end points (pad centre / track point) instead of the grid cells
    pts = [((x0 + i * STEP), (y0 + j * STEP), L) for i, j, L in path]
    pts[0] = (a[0], a[1], pts[0][2])
    pts[-1] = (b[0], b[1], pts[-1][2])
    # merge collinear runs
    simp = [pts[0]]
    for k in range(1, len(pts) - 1):
        p0, p1, p2 = simp[-1], pts[k], pts[k + 1]
        if p1[2] != p0[2] or p2[2] != p1[2]:
            simp.append(p1)
            continue
        if abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])) > 1e-6:
            simp.append(p1)
    simp.append(pts[-1])
    nseg = nvia = 0
    length = 0.0
    nobj = board.FindNet(net)
    for p, q in zip(simp, simp[1:]):
        if p[2] != q[2]:
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(round(q[0], 4)), pcbnew.FromMM(round(q[1], 4))))
            v.SetViaType(pcbnew.VIATYPE_THROUGH)
            v.SetWidth(pcbnew.F_Cu, pcbnew.FromMM(VIA_D))
            v.SetDrill(pcbnew.FromMM(VIA_DRILL))
            v.SetNet(nobj)
            v.SetLocked(True)
            board.Add(v)
            nvia += 1
            continue
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(round(p[0], 4)), pcbnew.FromMM(round(p[1], 4))))
        t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(round(q[0], 4)), pcbnew.FromMM(round(q[1], 4))))
        t.SetWidth(pcbnew.FromMM(w))
        t.SetLayer(lays[p[2]])
        t.SetNet(nobj)
        t.SetLocked(True)
        board.Add(t)
        nseg += 1
        length += math.hypot(q[0] - p[0], q[1] - p[1])
    return {"net": net, "routed": True, "from": [round(a[0], 3), round(a[1], 3)], "to": [round(b[0], 3), round(b[1], 3)],
            "segments": nseg, "vias": nvia, "length_mm": round(length, 2)}


def _endpoint(board, it: dict):
    """(x, y, layer index or None) of a DRC unconnected item."""
    pos = it["pos"]
    x, y = float(pos["x"]), float(pos["y"])
    desc = it.get("description", "")
    m = re.search(r"\b([FB])\.Cu\b", desc)
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if abs(_mm(p.GetPosition().x) - x) < 1e-3 and abs(_mm(p.GetPosition().y) - y) < 1e-3:
                if p.HasHole():
                    return x, y, None
                return x, y, 0 if p.IsOnLayer(pcbnew.F_Cu) else 1
    return x, y, (0 if m.group(1) == "F" else 1) if m else None


def close_islands(board, nets: tuple[str, ...] = ("GND", "+5V")) -> list[dict]:
    """Plane nets: a pad cluster cut off from the rest of its net (an F.Cu pour island whose pad got no fan-out via,
    e.g. the NT1 ground tie) is joined to the nearest pad / via of the main cluster by a short track (0.4 / 0.3 /
    0.2 mm, same clearance rules). pcbnew connectivity with zones filled, deterministic order."""
    out = []
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.BuildConnectivity()
    conn = board.GetConnectivity()
    for net in nets:
        pads = sorted((p for p in board.GetPads() if p.GetNetname() == net),
                      key=lambda p: (p.GetParentFootprint().GetReference(), p.GetNumber(), p.GetPosition().x, p.GetPosition().y))
        seen, clusters = set(), []
        for p in pads:
            if id(p) in seen:
                continue
            cl = [p] + [it for it in conn.GetConnectedItems(p, False) if it.GetClass() == "PAD" and it.GetNetname() == net]
            ids = {id(q) for q in cl}
            # GetConnectedItems returns new SWIG wrappers: match by position + parent instead of id
            keys = {(q.GetParentFootprint().GetReference(), q.GetNumber(), q.GetPosition().x, q.GetPosition().y) for q in cl}
            members = [q for q in pads if (q.GetParentFootprint().GetReference(), q.GetNumber(), q.GetPosition().x,
                                           q.GetPosition().y) in keys]
            for q in members:
                seen.add(id(q))
            seen |= ids
            clusters.append(members)
        if len(clusters) < 2:
            continue
        clusters.sort(key=lambda c: -len(c))
        main = clusters[0]
        targets = [(_mm(q.GetPosition().x), _mm(q.GetPosition().y), None if q.HasHole() else (0 if q.IsOnLayer(pcbnew.F_Cu) else 1))
                   for q in main]
        targets += [(_mm(v.GetPosition().x), _mm(v.GetPosition().y), None) for v in board.GetTracks()
                     if v.GetClass() == "PCB_VIA" and v.GetNetname() == net]
        for cl in clusters[1:]:
            q = cl[0]
            ax, ay = _mm(q.GetPosition().x), _mm(q.GetPosition().y)
            la = None if q.HasHole() else (0 if q.IsOnLayer(pcbnew.F_Cu) else 1)
            r = {"net": net, "routed": False, "island": [f"{x.GetParentFootprint().GetReference()}.{x.GetNumber()}" for x in cl]}
            for bx, by, lb in sorted(targets, key=lambda t: (math.hypot(t[0] - ax, t[1] - ay), t[0], t[1]))[:25]:
                for w in (0.4, 0.3, W):
                    rr = route_one(board, net, (ax, ay), (bx, by), la, lb, 4.0, w)
                    if rr["routed"]:
                        r = {**rr, "island": r["island"], "width_mm": w}
                        break
                if r["routed"]:
                    break
            out.append(r)
    return out


def _canonical(drc_report: dict) -> list[list[dict]]:
    """rev 3: kicad-cli lists the unconnected items in a varying order and with either end first (M: two replays of
    the same sessions routed BOOST_EN from different ends). Sorted by net and end position -> deterministic result."""
    out = []
    for v in drc_report.get("unconnected_items", []):
        its = v.get("items", [])
        if len(its) != 2:
            continue
        key = lambda it: (round(float(it["pos"]["x"]), 4), round(float(it["pos"]["y"]), 4), it.get("description", ""))
        its = sorted(its, key=key)
        out.append(its)
    out.sort(key=lambda its: (its[0].get("description", ""), [(round(float(i["pos"]["x"]), 4), round(float(i["pos"]["y"]), 4)) for i in its]))
    return out


def close(board, drc_report: dict) -> list[dict]:
    out = []
    for its in _canonical(drc_report):
        m = re.search(r"\[([^\]]+)\]", its[0].get("description", ""))
        if not m:
            continue
        net = m.group(1)
        if net in NO_TOUCH:
            out.append({"net": net, "routed": False, "reason": "power / plane net - not routed by this step"})
            continue
        ax, ay, la = _endpoint(board, its[0])
        bx, by, lb = _endpoint(board, its[1])
        r = None
        for w in track_widths(net):
            for region in (MARGIN_REGION, 12.0, 25.0):  # widen the search window when the local one has no path
                r = route_one(board, net, (ax, ay), (bx, by), la, lb, region, w)
                if r["routed"]:
                    r["search_window_mm"] = region
                    r["width_mm"] = w
                    break
            if r["routed"]:
                break
        out.append(r)
    return out
