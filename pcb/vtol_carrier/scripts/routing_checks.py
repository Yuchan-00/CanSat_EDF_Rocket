"""rev 3 routing requirements (spec/REV3_PLAN.md section 13), measured on the routed board with pcbnew.

Invariant: nothing here edits the design, and no check is a judgement call - every line is a number measured on
``kicad/vtol_carrier.kicad_pcb`` (label M) or computed from measured geometry with a stated formula (label E). A
requirement that is not met is reported ``FAIL`` with the number reached; no tolerance is added to make it pass.

Checks -> ``out/reports/routing_rules.json``:
  boost_sw          BOOST_SW copper on F.Cu only, no via, length against the pad-to-pad distance
  vout_vias         +5V vias at the C57 / C43 output pads (>= 3), each inside the In2 +5V fill
  gnd_vias          GND vias at U8.4 and the block's capacitor ground pads (>= 3), each inside the In1 GND fill
  in1_under_block   In1 GND fill under the boost block: no track on In1, covered share, one connected piece
  boost_fb          feedback tap: R63.1 joined to C43.1 by F.Cu track without a via on the way; distance of the
                    BOOST_FB copper from BOOST_SW copper / L5
  l5_keepout        no F.Cu track or via of another net under L5 (rule area and body box)
  battery_path      J14.1 -> Q10.3, Q10.2 -> J15.1, J15.2 -> L5.1: resistance of the copper (resistor network of the
                    track segments and vias, E), current-carrying segments narrower than 2.0 mm, target 0.035 Ohm
  hot_loop          plan line 448: C57 / C43 pads within 2.0 mm of U8.6 / U8.4, and an output-capacitor GND pad on the
                    same F.Cu GND copper piece as U8.4 (added after the rev 3 electrical review, finding E-R1)
  input_cap         plan line 449: C41 or C42 pad 1 within 2.0 mm of U8.3 / L5.1; copper path lengths (review E-R2)
  crystals          XTAL cluster paths and XTAL net lengths against the archived rev 2c board_metrics.json
  u8_u9_pads        pad -> net of U8 / U9 on the board against rev3_checks.expected_pad_nets (datasheet table)
  mcu_pinmap        the 256 pads of U1..U4 against the rev 2c pin map
  rule_file         design-rule sections of kicad/vtol_carrier.kicad_pro against the archived rev 2c project file
  silkscreen        polarity / pin-1 marks, 'rev 3' text, pin-order names of J1 / J2 / J5 / J7

Usage (KiCad's python): python routing_checks.py   [CARRIER_BOARD=<scratch board> CARRIER_ROUTING_OUT=<json>]
"""

from __future__ import annotations

import hashlib
import heapq
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pcbnew

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import board_metrics as BM  # noqa: E402
import silk as SILK  # noqa: E402

BOARD = Path(os.environ["CARRIER_BOARD"]) if os.environ.get("CARRIER_BOARD") else ROOT / "kicad" / "vtol_carrier.kicad_pcb"
OUT = Path(os.environ["CARRIER_ROUTING_OUT"]) if os.environ.get("CARRIER_ROUTING_OUT") else ROOT / "out" / "reports" / "routing_rules.json"
PRO = BOARD.with_suffix(".kicad_pro")
REV2C = ROOT / "archive" / "rev2c_pololu_1s"
PLACEMENT = ROOT / "kicad" / "placement.json"
mm = pcbnew.ToMM
PASS, FAIL, NOT_VERIFIED = "PASS", "FAIL", "NOT_VERIFIED"

#: copper resistance model (E): 1 oz outer foil 35 um, rho 1.72e-8 Ohm m at 20 C; via barrel 18 um plating (A, JLC
#: nominal), board 1.6 mm (the full barrel length is charged for every layer change)
RHO = 1.72e-8
SHEET_OHM = RHO / 35e-6
PLATING_M = 18e-6
BOARD_T_M = 1.6e-3
TARGET_OHM = 0.035
WIDE_MM = 2.0
BLOCK_REFS = ("U8", "L5", "C57", "C43", "R63", "R64", "C58")
VIA_NEAR_MM = 1.5


def sha256_file(p: Path) -> str | None:
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None


def box(bb, grow: float = 0.0):
    return (mm(bb.GetLeft()) - grow, mm(bb.GetTop()) - grow, mm(bb.GetRight()) + grow, mm(bb.GetBottom()) + grow)


def rect_dist(x: float, y: float, b) -> float:
    return math.hypot(max(b[0] - x, 0.0, x - b[2]), max(b[1] - y, 0.0, y - b[3]))


def pt(v) -> tuple[float, float]:
    return (mm(v.x), mm(v.y))


def pad(fps, ref: str, num: str):
    return [p for p in fps[ref].Pads() if p.GetNumber() == num][0]


def tracks_of(board, net: str):
    return [t for t in board.GetTracks() if t.GetNetname() == net and t.GetClass() != "PCB_VIA"]


def vias_of(board, net: str):
    return [t for t in board.GetTracks() if t.GetNetname() == net and t.GetClass() == "PCB_VIA"]


def zone_of(board, net: str, layer):
    return [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net and z.IsOnLayer(layer)]


def in_fill(zones, layer, x: float, y: float) -> bool:
    p = pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))
    return any(z.HitTestFilledArea(layer, p, 0) for z in zones)


def shape_gap(a, b) -> float:
    """Copper-to-copper distance of two SHAPEs in mm (bisection on SHAPE.Collide with a clearance, 1 um)."""
    if a.Collide(b, 0):
        return 0.0
    lo, hi = 0, pcbnew.FromMM(40.0)
    while hi - lo > 1000:
        mid = (lo + hi) // 2
        if a.Collide(b, mid):
            hi = mid
        else:
            lo = mid
    return round(mm(hi), 3)


def copper_shapes(board, fps, net: str, pads: bool = True):
    out = []
    for t in board.GetTracks():
        if t.GetNetname() == net:
            lay = t.GetLayer() if t.GetClass() != "PCB_VIA" else pcbnew.F_Cu
            what = f"via ({mm(t.GetPosition().x):.2f}, {mm(t.GetPosition().y):.2f})" if t.GetClass() == "PCB_VIA" else \
                f"track {t.GetLayerName()} ({mm(t.GetStart().x):.2f}, {mm(t.GetStart().y):.2f})-({mm(t.GetEnd().x):.2f}, {mm(t.GetEnd().y):.2f})"
            out.append((what, t.GetEffectiveShape(lay)))
    if pads:
        for fp in fps.values():
            for p in fp.Pads():
                if p.GetNetname() == net:
                    out.append((f"pad {fp.GetReference()}.{p.GetNumber()}", p.GetEffectiveShape(pcbnew.F_Cu)))
    return out


# ---------------------------------------------------------------------------------------------------- BOOST_SW
def check_boost_sw(board, fps) -> dict:
    tr, vs = tracks_of(board, "BOOST_SW"), vias_of(board, "BOOST_SW")
    layers = sorted({t.GetLayerName() for t in tr})
    length = round(sum(mm(t.GetLength()) for t in tr), 3)
    a, b = pad(fps, "U8", "5"), pad(fps, "L5", "2")
    centre = round(math.hypot(*(p - q for p, q in zip(pt(a.GetPosition()), pt(b.GetPosition())))), 3)
    gap = shape_gap(a.GetEffectiveShape(pcbnew.F_Cu), b.GetEffectiveShape(pcbnew.F_Cu))
    ok = layers == ["F.Cu"] and not vs and length <= centre + 0.01
    return {"result": PASS if ok else FAIL, "label": "M", "layers": layers, "vias": len(vs), "track_length_mm": length,
            "pad_centre_distance_U8.5_L5.2_mm": centre, "pad_edge_gap_mm": gap,
            "widths_mm": sorted({round(mm(t.GetWidth()), 3) for t in tr}),
            "criterion": "F.Cu only, 0 vias, track length <= straight distance between the two pad centres"}


# ---------------------------------------------------------------------------------------------- plane vias
def vias_near(board, fps, net: str, pads: list[tuple[str, str]], layer, zones) -> dict:
    rows, seen = [], {}
    for ref, num in pads:
        p = pad(fps, ref, num)
        if p.GetNetname() != net:
            rows.append({"pad": f"{ref}.{num}", "error": f"pad net is {p.GetNetname()}, expected {net}"})
            continue
        pb = box(p.GetBoundingBox())
        near = []
        for v in vias_of(board, net):
            x, y = pt(v.GetPosition())
            d = rect_dist(x, y, pb)
            if d <= VIA_NEAR_MM:
                key = (round(x, 3), round(y, 3))
                inside = in_fill(zones, layer, x, y)
                seen[key] = inside
                near.append({"at": list(key), "to_pad_mm": round(d, 3), "drill_mm": mm(v.GetDrillValue()),
                             "in_plane_fill": inside})
        rows.append({"pad": f"{ref}.{num}", "vias_within_1.5mm": near})
    n = sum(1 for ok in seen.values() if ok)
    return {"rows": rows, "distinct_vias_in_plane": n, "distinct_vias": len(seen)}


def check_vout_vias(board, fps) -> dict:
    r = vias_near(board, fps, "+5V", [("C57", "1"), ("C43", "1")], pcbnew.In2_Cu, zone_of(board, "+5V", pcbnew.In2_Cu))
    r.update({"result": PASS if r["distinct_vias_in_plane"] >= 3 and not any("error" in x for x in r["rows"]) else FAIL,
              "label": "M", "criterion": ">= 3 distinct +5V vias within 1.5 mm of the C57.1 / C43.1 pads, each inside the In2 +5V fill"})
    return r


def check_gnd_vias(board, fps) -> dict:
    r = vias_near(board, fps, "GND", [("U8", "4"), ("C57", "2"), ("C43", "2"), ("C41", "2")], pcbnew.In1_Cu,
                  zone_of(board, "GND", pcbnew.In1_Cu))
    per_pad = {x["pad"]: len(x.get("vias_within_1.5mm", [])) for x in r["rows"]}
    ok = r["distinct_vias_in_plane"] >= 3 and all(per_pad.get(k, 0) >= 1 for k in ("U8.4", "C57.2", "C43.2"))
    r.update({"result": PASS if ok and not any("error" in x for x in r["rows"]) else FAIL, "label": "M",
              "criterion": ">= 3 distinct GND vias within 1.5 mm of U8.4 / C57.2 / C43.2 / C41.2 inside the In1 GND "
                           "fill, at least one at each of U8.4, C57.2, C43.2"})
    return r


def check_in1(board, fps) -> dict:
    zones = zone_of(board, "GND", pcbnew.In1_Cu)
    bs = [SILK._courtyard_box(fps[r]) for r in BLOCK_REFS]
    x0, y0, x1, y1 = min(b[0] for b in bs), min(b[1] for b in bs), max(b[2] for b in bs), max(b[3] for b in bs)
    step = 0.1
    nx, ny = int(round((x1 - x0) / step)) + 1, int(round((y1 - y0) / step)) + 1
    grid = [[in_fill(zones, pcbnew.In1_Cu, x0 + i * step, y0 + j * step) for i in range(nx)] for j in range(ny)]
    covered = sum(sum(1 for c in row if c) for row in grid)
    # connected pieces of the covered cells (4-neighbour flood fill)
    seen, pieces = set(), 0
    for j in range(ny):
        for i in range(nx):
            if grid[j][i] and (i, j) not in seen:
                pieces += 1
                stack = [(i, j)]
                seen.add((i, j))
                while stack:
                    a, b = stack.pop()
                    for c, d in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                        if 0 <= c < nx and 0 <= d < ny and grid[d][c] and (c, d) not in seen:
                            seen.add((c, d))
                            stack.append((c, d))
    in1_tracks = [t for t in board.GetTracks() if t.GetClass() != "PCB_VIA" and t.GetLayer() == pcbnew.In1_Cu
                  and BM_overlap(box(t.GetBoundingBox()), (x0, y0, x1, y1))]
    holes = []
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA" and t.GetNetname() != "GND":
            x, y = pt(t.GetPosition())
            if x0 <= x <= x1 and y0 <= y <= y1:
                holes.append({"net": t.GetNetname(), "at": [round(x, 3), round(y, 3)]})
    share = round(100.0 * covered / (nx * ny), 1)
    ok = bool(zones) and not in1_tracks and pieces == 1
    return {"result": PASS if ok else FAIL, "label": "M", "block_box_kicad_mm": [round(v, 2) for v in (x0, y0, x1, y1)],
            "block_refs": list(BLOCK_REFS), "grid_mm": step, "covered_percent": share, "covered_pieces": pieces,
            "tracks_on_In1_in_box": len(in1_tracks), "non_gnd_vias_through_the_plane_in_box": holes,
            "criterion": "no track on In1 inside the block box and the In1 GND fill inside the box is one connected "
                         "piece (the anti-pads of the listed vias are the only openings); covered share reported"}


def BM_overlap(a, b) -> bool:
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


# ---------------------------------------------------------------------------------------------------- BOOST_FB
def check_boost_fb(board, fps) -> dict:
    # (1) sense tap: R63.1 (+5V) -> C43.1 on F.Cu track without passing a via
    net = "+5V"
    a, b = pad(fps, "R63", "1"), pad(fps, "C43", "1")
    segs = [t for t in tracks_of(board, net) if t.GetLayer() == pcbnew.F_Cu]
    via_pts = [pt(v.GetPosition()) for v in vias_of(board, net)]
    nodes: dict[tuple[int, int], list] = {}
    for t in segs:
        for e, o in ((t.GetStart(), t.GetEnd()), (t.GetEnd(), t.GetStart())):
            nodes.setdefault((e.x, e.y), []).append(((o.x, o.y), mm(t.GetLength())))
    start = [k for k in nodes if a.HitTest(pcbnew.VECTOR2I(*k))]
    best, vias_on_way = None, None
    import heapq

    heap = [(0.0, k, 0) for k in start]
    done = {}
    while heap:
        d, k, nv = heapq.heappop(heap)
        if k in done:
            continue
        done[k] = (d, nv)
        if b.HitTest(pcbnew.VECTOR2I(*k)):
            best, vias_on_way = round(d, 3), nv
            break
        for o, ln in nodes.get(k, []):
            if o not in done:
                at_via = any(abs(mm(o[0]) - vx) < 0.05 and abs(mm(o[1]) - vy) < 0.05 for vx, vy in via_pts) and \
                    not b.HitTest(pcbnew.VECTOR2I(*o))
                heapq.heappush(heap, (d + ln, o, nv + (1 if at_via else 0)))
    r63_vias = [list(map(lambda v: round(v, 3), p)) for p in via_pts if rect_dist(p[0], p[1], box(a.GetBoundingBox())) <= 0.3]
    tap_ok = best is not None and vias_on_way == 0 and not r63_vias
    # (2) distance of the BOOST_FB copper from BOOST_SW copper and from L5
    fb_tracks = copper_shapes(board, fps, "BOOST_FB", pads=False)
    fb_pads = [(w, s) for w, s in copper_shapes(board, fps, "BOOST_FB") if w.startswith("pad ")]
    sw = copper_shapes(board, fps, "BOOST_SW")
    l5 = [(f"pad L5.{p.GetNumber()}", p.GetEffectiveShape(pcbnew.F_Cu)) for p in fps["L5"].Pads()]
    pkg = min(shape_gap(s, o) for w, s in fb_pads if w == "pad U8.1" for _n, o in sw + l5)
    rows = []
    for w, s in fb_tracks + [x for x in fb_pads if x[0] != "pad U8.1"]:
        g = min(((shape_gap(s, o), n) for n, o in sw + l5), key=lambda v: v[0])
        rows.append({"item": w, "gap_mm": g[0], "nearest": g[1]})
    l5c = SILK._courtyard_box(fps["L5"])
    inside = [w for w, s in fb_tracks if BM_overlap(box(s.BBox()), l5c)]
    worst = min((r["gap_mm"] for r in rows), default=None)
    away_ok = worst is not None and worst >= pkg - 1e-6 and not inside
    return {"result": PASS if tap_ok and away_ok else FAIL, "label": "M",
            "sense_tap": {"from": "R63.1", "to": "C43.1", "f_cu_track_path_mm": best, "vias_on_the_way": vias_on_way,
                          "+5V_vias_at_R63.1": r63_vias, "ok": tap_ok},
            "distance_from_switch_node": {"package_gap_U8.1_to_BOOST_SW_or_L5_mm": pkg, "min_gap_of_tracks_and_other_pads_mm": worst,
                                          "items": sorted(rows, key=lambda r: r["gap_mm"])[:6],
                                          "tracks_inside_L5_courtyard": inside, "ok": away_ok},
            "criterion": "R63.1 reaches C43.1 on F.Cu track with no via on the way and no +5V via at R63.1 (the divider "
                         "senses the capacitor pad, not the plane); no BOOST_FB track / via / other pad is nearer to "
                         "BOOST_SW copper or an L5 pad than the U8.1 pad is by its package, none inside the L5 courtyard"}


# ------------------------------------------------------------------------------------------------- under L5
def check_l5(board, fps) -> dict:
    fab = SILK.Ctx(board).fab["L5"]
    own = {p.GetNetname() for p in fps["L5"].Pads()}
    areas = [box(z.GetBoundingBox()) for z in board.Zones() if z.GetIsRuleArea() and z.IsOnLayer(pcbnew.F_Cu)
             and BM_overlap(box(z.GetBoundingBox()), fab)]
    hits_area, hits_body, gnd_body = [], [], []
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            x, y = pt(t.GetPosition())
            r = mm(t.GetWidth(pcbnew.F_Cu)) / 2
            pts, what = [(x, y)], f"via {t.GetNetname()} ({x:.2f}, {y:.2f})"
        else:
            if t.GetLayer() != pcbnew.F_Cu:
                continue
            (ax, ay), (bx, by) = pt(t.GetStart()), pt(t.GetEnd())
            n = max(1, int(math.hypot(bx - ax, by - ay) / 0.02))
            pts = [(ax + (bx - ax) * i / n, ay + (by - ay) * i / n) for i in range(n + 1)]
            r = mm(t.GetWidth()) / 2
            what = f"track {t.GetNetname()} ({ax:.2f}, {ay:.2f})-({bx:.2f}, {by:.2f}) w {2 * r:.2f}"
        for a in areas:
            if any(rect_dist(x, y, a) < r for x, y in pts):
                hits_area.append(what)
        if t.GetNetname() not in own and any(rect_dist(x, y, fab) < r for x, y in pts):
            # GND is the reference, not a signal (plan section 13: "no signal trace under L5"): listed, not failed
            (gnd_body if t.GetNetname() == "GND" else hits_body).append(what)
    ok = bool(areas) and not hits_area and not hits_body
    return {"result": PASS if ok else FAIL, "label": "M", "rule_areas_under_L5_kicad_mm": [[round(v, 2) for v in a] for a in areas],
            "L5_body_box_kicad_mm": [round(v, 2) for v in fab], "copper_in_rule_area": sorted(set(hits_area)),
            "signal_copper_under_body_on_F_Cu": sorted(set(hits_body)),
            "gnd_copper_under_body_on_F_Cu_information": sorted(set(gnd_body)),
            "criterion": "an F.Cu rule area exists between the L5 pads, no F.Cu track or via of any net touches it, "
                         "and no F.Cu track / via of a signal net (any net other than L5's own two nets and GND) lies "
                         "under the L5 body box; GND tracks / vias under the body are listed as information"}


# ------------------------------------------------------------------------------------------- battery path (E)
def via_ohm(drill_mm: float) -> float:
    d = drill_mm * 1e-3
    return RHO * BOARD_T_M / (math.pi * (d + PLATING_M) * PLATING_M)


def network(board, fps, net: str, src: tuple[str, str], dst: tuple[str, str]) -> dict:
    """Resistance src pad -> dst pad through the tracks and vias of ``net`` (nodal analysis). Segments are split
    where another segment end or a via lands on them; an end that lies inside the copper of a wider segment is tied to
    that segment's centre line; all track ends inside a pad are tied to the pad (the pad copper itself is not charged).
    Zone copper is not part of the model."""
    segs = [{"layer": t.GetLayer(), "a": pt(t.GetStart()), "b": pt(t.GetEnd()), "w": mm(t.GetWidth())} for t in tracks_of(board, net)]
    segs = [s for s in segs if math.hypot(s["a"][0] - s["b"][0], s["a"][1] - s["b"][1]) > 1e-6]
    vias = [{"p": pt(v.GetPosition()), "drill": mm(v.GetDrillValue()), "r": mm(v.GetWidth(pcbnew.F_Cu)) / 2}
            for v in vias_of(board, net)]
    layers = sorted({s["layer"] for s in segs})
    idx: dict = {}
    edges: list[tuple[int, int, float, dict | None]] = []

    def node(key) -> int:
        return idx.setdefault(key, len(idx))

    def pkey(layer, p):
        return (layer, round(p[0], 3), round(p[1], 3))

    for s in segs:
        (ax, ay), (bx, by) = s["a"], s["b"]
        L = math.hypot(bx - ax, by - ay)
        cuts = {0.0, 1.0}
        ties = []
        cand = [(o["a"], None) for o in segs if o is not s and o["layer"] == s["layer"]] + \
               [(o["b"], None) for o in segs if o is not s and o["layer"] == s["layer"]] + [(v["p"], v) for v in vias]
        for (px, py), _v in cand:
            u = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / (L * L)
            if not 0.0 < u < 1.0:
                continue
            qx, qy = ax + u * (bx - ax), ay + u * (by - ay)
            d = math.hypot(px - qx, py - qy)
            if d <= s["w"] / 2 and min(u, 1 - u) * L > 1e-3:
                cuts.add(round(u, 6))
                # always tie: the cut point is rebuilt from the rounded u, so it can land on the other side of a
                # 1 um key rounding than the end that caused it (measured: B.Cu end x 129.3005 on a 14.3 mm track)
                ties.append(((px, py), (ax + round(u, 6) * (bx - ax), ay + round(u, 6) * (by - ay))))
        us = sorted(cuts)
        for u0, u1 in zip(us, us[1:]):
            p0 = (ax + u0 * (bx - ax), ay + u0 * (by - ay))
            p1 = (ax + u1 * (bx - ax), ay + u1 * (by - ay))
            ln = (u1 - u0) * L
            edges.append((node(pkey(s["layer"], p0)), node(pkey(s["layer"], p1)), SHEET_OHM * ln / s["w"],
                          {"layer": board.GetLayerName(s["layer"]), "w": round(s["w"], 3), "len": round(ln, 3),
                           "a": [round(v, 2) for v in p0], "b": [round(v, 2) for v in p1]}))
        for p, q in ties:
            edges.append((node(pkey(s["layer"], p)), node(pkey(s["layer"], q)), 1e-7, None))
    for v in vias:
        # a track end that lies on the via's copper annulus is tied to the via centre on that layer (track ends are
        # not always on the exact via centre: e.g. 0.5 um off after a rounding of the router)
        for key in [k for k in idx if k[0] != "pad"]:
            d = math.hypot(key[1] - v["p"][0], key[2] - v["p"][1])
            if 1e-6 < d <= v["r"]:
                edges.append((idx[key], node(pkey(key[0], v["p"])), 1e-7, None))
        ks = [node(pkey(lay, v["p"])) for lay in layers if pkey(lay, v["p"]) in idx]
        for a, b in zip(ks, ks[1:]):
            edges.append((a, b, via_ohm(v["drill"]), {"via": [round(c, 2) for c in v["p"]], "drill": v["drill"]}))
    pad_nodes = {}
    for fp in fps.values():
        for p in fp.Pads():
            if p.GetNetname() != net:
                continue
            k = node(("pad", fp.GetReference(), p.GetNumber()))
            pad_nodes[(fp.GetReference(), p.GetNumber())] = k
            for key, i in list(idx.items()):
                if key[0] == "pad" or not p.IsOnLayer(key[0]):
                    continue
                if p.HitTest(pcbnew.VECTOR2I(pcbnew.FromMM(key[1]), pcbnew.FromMM(key[2]))):
                    edges.append((i, k, 1e-7, None))
    if src not in pad_nodes or dst not in pad_nodes:
        return {"net": net, "from": ".".join(src), "to": ".".join(dst), "ohm": None, "error": "pad not on this net"}
    n = len(idx)
    G = np.zeros((n, n))
    for a, b, r, _m in edges:
        if a == b:
            continue
        g = 1.0 / r
        G[a, a] += g
        G[b, b] += g
        G[a, b] -= g
        G[b, a] -= g
    s_i, d_i = pad_nodes[src], pad_nodes[dst]
    # keep the nodes connected to the source
    adj: dict[int, list[int]] = {}
    for a, b, _r, _m in edges:
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)
    comp, stack = {s_i}, [s_i]
    while stack:
        for o in adj.get(stack.pop(), []):
            if o not in comp:
                comp.add(o)
                stack.append(o)
    if d_i not in comp:
        return {"net": net, "from": ".".join(src), "to": ".".join(dst), "ohm": None, "error": "no track path between the pads"}
    # shortest copper path (track centre-line length; ties and vias count 0 mm, vias are counted)
    wadj: dict[int, list[tuple[int, float, int]]] = {}
    for a, b, _r, m in edges:
        ln = m["len"] if m and "len" in m else 0.0
        nv = 1 if m and "via" in m else 0
        wadj.setdefault(a, []).append((b, ln, nv))
        wadj.setdefault(b, []).append((a, ln, nv))
    best = {s_i: (0.0, 0)}
    heap = [(0.0, 0, s_i)]
    while heap:
        dl, dv, u_ = heapq.heappop(heap)
        if best.get(u_, (1e18, 0)) < (dl, dv):
            continue
        for o, ln, nv in wadj.get(u_, []):
            cand_ = (dl + ln, dv + nv)
            if cand_ < best.get(o, (1e18, 0)):
                best[o] = cand_
                heapq.heappush(heap, (cand_[0], cand_[1], o))
    path_mm, path_vias = best[d_i]
    keep = sorted(comp - {d_i})
    pos = {k: i for i, k in enumerate(keep)}
    A = G[np.ix_(keep, keep)]
    rhs = np.zeros(len(keep))
    rhs[pos[s_i]] = 1.0
    V = np.linalg.solve(A, rhs)
    volt = {k: V[pos[k]] for k in keep}
    volt[d_i] = 0.0
    carrying, n_vias = [], 0
    for a, b, r, m in edges:
        if m is None or a not in volt or b not in volt:
            continue
        i = abs(volt[a] - volt[b]) / r
        if i >= 0.2:
            if "via" in m:
                n_vias += 1
            else:
                carrying.append(dict(m, share=round(i, 2)))
    narrow = [m for m in carrying if m["w"] < WIDE_MM - 1e-6]
    return {"net": net, "from": ".".join(src), "to": ".".join(dst), "ohm": round(float(volt[s_i]), 5),
            "shortest_copper_path_mm": round(path_mm, 2), "vias_on_shortest_path": path_vias,
            "segments_carrying_ge_20pct": len(carrying), "vias_carrying_ge_20pct": n_vias,
            "min_width_carrying_mm": min((m["w"] for m in carrying), default=None),
            "length_carrying_mm": round(sum(m["len"] * m["share"] for m in carrying), 2),
            "narrower_than_2mm": {"length_mm": round(sum(m["len"] for m in narrow), 2),
                                  "widths_mm": sorted({m["w"] for m in narrow}),
                                  "segments": [m for m in narrow if m["len"] >= 0.3][:12]}}


def check_battery(board, fps) -> dict:
    legs = [network(board, fps, "VBAT_LOGIC_RAW", ("J14", "1"), ("Q10", "3")),
            network(board, fps, "VBAT_LOGIC", ("Q10", "2"), ("J15", "1")),
            network(board, fps, "VBAT_LOGIC_SW", ("J15", "2"), ("L5", "1"))]
    if any(leg["ohm"] is None for leg in legs):
        return {"result": FAIL, "label": "E", "legs": legs, "reason": "a leg has no track path"}
    total = round(sum(leg["ohm"] for leg in legs), 5)
    narrow = round(sum(leg["narrower_than_2mm"]["length_mm"] for leg in legs), 2)
    return {"result": PASS if total <= TARGET_OHM and narrow == 0 else FAIL, "label": "E",
            "copper_resistance_ohm_20C": total, "target_ohm": TARGET_OHM, "resistance_target_met": total <= TARGET_OHM,
            "copper_resistance_ohm_60C": round(total * (1 + 0.00393 * 40), 5),
            "width_rule_met": narrow == 0, "current_carrying_length_narrower_than_2mm": narrow, "legs": legs,
            "model": f"sheet resistance {SHEET_OHM * 1000:.3f} mOhm/square (35 um, rho 1.72e-8 Ohm m, 20 C); via barrel "
                     f"{via_ohm(0.3) * 1000:.2f} mOhm at 0.3 mm drill / {via_ohm(0.4) * 1000:.2f} mOhm at 0.4 mm (18 um "
                     "plating A, 1.6 mm); pads and zone copper not charged; NOT included: Q10 channel, the switch and its "
                     "lead, connector contacts, the battery lead, the GND return in the In1 plane",
            "criterion": "copper resistance J14.1 -> Q10.3 + Q10.2 -> J15.1 + J15.2 -> L5.1 <= 0.035 Ohm AND no "
                         "current-carrying (>= 20 %) segment narrower than 2.0 mm"}


# ------------------------------------------------------------------------------- boost hot loop / input capacitor
def fcu_gnd_islands(board, fps, items: list[tuple[str, str]]) -> dict:
    """F.Cu GND copper pieces (filled F.Cu GND zone islands joined by F.Cu GND tracks and F.Cu GND pads) that each
    listed pad touches; returns {pad: piece id}. Vias are NOT joins here (the question is the top-copper loop)."""
    polys = []
    for z in zone_of(board, "GND", pcbnew.F_Cu):
        ps = z.GetFilledPolysList(pcbnew.F_Cu)
        polys += [ps.UnitSet(i) for i in range(ps.OutlineCount())]
    shapes = [("poly", p) for p in polys]
    shapes += [("track", t.GetEffectiveShape(pcbnew.F_Cu)) for t in tracks_of(board, "GND") if t.GetLayer() == pcbnew.F_Cu]
    shapes += [("pad", p.GetEffectiveShape(pcbnew.F_Cu)) for fp in fps.values() for p in fp.Pads()
               if p.GetNetname() == "GND" and p.IsOnLayer(pcbnew.F_Cu)]
    parent = list(range(len(shapes)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i in range(len(shapes)):
        for j in range(i + 1, len(shapes)):
            if shapes[i][0] == "poly" and shapes[j][0] == "poly":
                continue  # islands of one fill never touch each other
            if find(i) != find(j) and shapes[i][1].Collide(shapes[j][1], 0):
                parent[find(i)] = find(j)
    out = {}
    for ref, num in items:
        sh = pad(fps, ref, num).GetEffectiveShape(pcbnew.F_Cu)
        hit = [find(i) for i, (_k, s) in enumerate(shapes) if s.Collide(sh, 0)]
        out[f"{ref}.{num}"] = min(hit) if hit else None
    return out


def check_hot_loop(board, fps) -> dict:
    """plan line 448: U8 pin 6 (VOUT) -> C57 / C43 -> U8 pin 4 (GND) as small as the courtyards allow, both capacitors
    within 2 mm of the IC; datasheet tps61023.txt:1008-1011: the output capacitor close to VOUT AND to the GND pin."""
    rows, ok = [], True
    for c in ("C57", "C43"):
        g_out = shape_gap(pad(fps, c, "1").GetEffectiveShape(pcbnew.F_Cu), pad(fps, "U8", "6").GetEffectiveShape(pcbnew.F_Cu))
        g_gnd = shape_gap(pad(fps, c, "2").GetEffectiveShape(pcbnew.F_Cu), pad(fps, "U8", "4").GetEffectiveShape(pcbnew.F_Cu))
        rows.append({"cap": c, "gap_pad1_to_U8.6_mm": g_out, "gap_pad2_to_U8.4_mm": g_gnd})
        ok &= g_out <= 2.0 and g_gnd <= 2.0
    isl = fcu_gnd_islands(board, fps, [("U8", "4"), ("C57", "2"), ("C43", "2")])
    joined = [k for k in ("C57.2", "C43.2") if isl[k] is not None and isl[k] == isl["U8.4"]]
    ok &= len(joined) >= 1
    # information (E-R3 of the rev 3 review): BOOST_FB copper inside the body box of an output capacitor
    fb_under = []
    for c in ("C57", "C43"):
        bb = SILK._courtyard_box(fps[c])
        for t in tracks_of(board, "BOOST_FB"):
            if t.GetLayer() != pcbnew.F_Cu:
                continue
            (ax, ay), (bx, by) = pt(t.GetStart()), pt(t.GetEnd())
            if any(bb[0] <= x <= bb[2] and bb[1] <= y <= bb[3] for x, y in ((ax, ay), (bx, by), ((ax + bx) / 2, (ay + by) / 2))):
                fb_under.append({"cap": c, "track": [round(ax, 2), round(ay, 2), round(bx, 2), round(by, 2)],
                                 "w": round(mm(t.GetWidth()), 3)})
    return {"result": PASS if ok else FAIL, "label": "M", "caps": rows,
            "fcu_gnd_piece": isl, "output_cap_gnd_joined_to_U8.4_on_F.Cu": joined,
            "information_boost_fb_under_output_cap": fb_under,
            "criterion": "plan line 448: each output capacitor (C57, C43) has pad 1 within 2.0 mm (copper edge gap) of "
                         "U8.6 and pad 2 within 2.0 mm of U8.4, AND at least one of C57.2 / C43.2 is on the same F.Cu GND "
                         "copper piece as U8.4 (zone islands + tracks + pads, vias not counted)"}


def check_input_cap(board, fps) -> dict:
    """plan line 449: C41 / C42 (input capacitors) at the L5 / VIN end; datasheet tps61023.txt:881-882 / 1006-1007."""
    rows, best = [], None
    for c in ("C41", "C42"):
        for tgt in (("U8", "3"), ("L5", "1")):
            g = shape_gap(pad(fps, c, "1").GetEffectiveShape(pcbnew.F_Cu), pad(fps, *tgt).GetEffectiveShape(pcbnew.F_Cu))
            n = network(board, fps, "VBAT_LOGIC_SW", (c, "1"), tgt)
            rows.append({"cap": c, "to": ".".join(tgt), "pad_edge_gap_mm": g, "copper_path_mm": n.get("shortest_copper_path_mm"),
                         "vias_on_path": n.get("vias_on_shortest_path"), "copper_ohm_E": n.get("ohm"), "error": n.get("error")})
            if n.get("shortest_copper_path_mm") is not None:
                best = min(best, n["shortest_copper_path_mm"]) if best is not None else n["shortest_copper_path_mm"]
    gap_ok = any(r["pad_edge_gap_mm"] <= 2.0 for r in rows)
    return {"result": PASS if gap_ok else FAIL, "label": "M (path lengths and ohm from the track model, E)",
            "rows": rows, "shortest_copper_path_mm": best,
            "criterion": "plan line 449: at least one input capacitor (C41 / C42) with pad 1 within 2.0 mm (copper edge "
                         "gap) of U8.3 (VIN) or L5.1 (the 2.0 mm figure is the plan's hot-loop distance of line 448); "
                         "copper paths over tracks and vias only (zone copper not modelled); a gap of 40.0 is the search "
                         "limit of shape_gap and means >= 40 mm"}


# ---------------------------------------------------------------------------------------------------- crystals
def check_crystals(board, fps) -> dict:
    ref = json.loads((REV2C / "out" / "reports" / "board_metrics.json").read_text(encoding="utf-8"))
    pdata = json.loads(PLACEMENT.read_text(encoding="utf-8"))
    now_rows = [r for r in BM.cluster_paths(board, fps, pdata["clusters"]) if "XTAL" in r["net"]]
    old_rows = {(r["net"], r["what"]): r for r in ref["cluster_paths"] if "XTAL" in r["net"]}
    diffs = []
    for r in now_rows:
        o = old_rows.get((r["net"], r["what"]))
        if o is None or o["copper_path_mm"] != r["copper_path_mm"] or o["ok_le_3mm"] != r["ok_le_3mm"]:
            diffs.append({"net": r["net"], "what": r["what"], "rev2c": o and o["copper_path_mm"], "rev3": r["copper_path_mm"]})
    now_nets, old_nets = BM.xtal_nets(board), ref["xtal_nets"]
    net_diffs = [{"net": n, "rev2c": old_nets.get(n), "rev3": now_nets.get(n)} for n in sorted(set(now_nets) | set(old_nets))
                 if now_nets.get(n) != old_nets.get(n)]
    x1 = [r["copper_path_mm"] for r in now_rows if r["what"].startswith("XTAL1 pin 24")]
    worst = max((v for v in x1 if v is not None), default=None)
    ok = len(now_rows) == len(old_rows) == 16 and not diffs and not net_diffs and worst is not None and worst <= 5.96
    return {"result": PASS if ok else FAIL, "label": "M", "xtal_path_rows": len(now_rows), "rows_differing_from_rev2c": diffs,
            "xtal_nets_differing_from_rev2c": net_diffs, "XTAL1_pin24_to_crystal_mm": x1, "XTAL1_worst_mm": worst,
            "XTAL1_limit_mm": 5.96, "xtal_nets": now_nets,
            "reference": {"file": str(REV2C / "out" / "reports" / "board_metrics.json").replace("\\", "/"),
                          "sha256": sha256_file(REV2C / "out" / "reports" / "board_metrics.json")},
            "criterion": "the 16 XTAL copper paths (4 MCUs x 4) and the 8 XTAL net lengths / via counts / layers equal "
                         "the archived rev 2c values; XTAL1 pin 24 -> crystal <= 5.96 mm"}


# ------------------------------------------------------------------------------------------------- pad nets
def check_u8_u9(fps) -> dict:
    try:
        import rev3_checks as RC

        want = {r: RC.expected_pad_nets(r) for r in ("U8", "U9")}
    except Exception as e:  # noqa: BLE001
        return {"result": NOT_VERIFIED, "label": "M", "reason": f"rev3_checks.expected_pad_nets not importable: {e}"}
    rows, bad = {}, []
    for r in ("U8", "U9"):
        have = {p.GetNumber(): p.GetNetname() for p in fps[r].Pads()}
        rows[r] = {"footprint": fps[r].GetFPID().GetUniStringLibItemName(), "pads": have, "expected": want[r]}
        bad += [f"{r}.{n}: board {have.get(n)} / datasheet table {want[r].get(n)}" for n in sorted(set(have) | set(want[r]))
                if have.get(n) != want[r].get(n)]
    return {"result": PASS if not bad else FAIL, "label": "M", "differences": bad, "parts": rows,
            "source": "scripts/rev3_checks.py expected_pad_nets (datasheet pin table x design intent)"}


def check_mcu(fps) -> dict:
    sys.path.insert(0, str(ROOT / "scripts"))
    try:
        import rev3_checks as RC

        path = RC.REV2C_PINMAP
    except Exception as e:  # noqa: BLE001
        return {"result": NOT_VERIFIED, "label": "M", "reason": f"rev3_checks not importable: {e}"}
    if not Path(path).is_file():
        return {"result": NOT_VERIFIED, "label": "M", "reason": f"no reference pin map at {path}"}
    ref = json.loads(Path(path).read_text(encoding="utf-8"))
    diffs, n = [], 0
    for u in ("U1", "U2", "U3", "U4"):
        have = {}
        for p in fps[u].Pads():
            name = p.GetNetname()
            have[p.GetNumber()] = "NC" if (not name or name.startswith("unconnected-")) else name
        for pin in sorted(set(have) | set(ref.get(u, {})), key=lambda s: int(s) if s.isdigit() else 999):
            n += 1
            if have.get(pin) != ref.get(u, {}).get(pin):
                diffs.append({"pad": f"{u}.{pin}", "rev2c": ref.get(u, {}).get(pin), "board": have.get(pin)})
    return {"result": PASS if n == 256 and not diffs else FAIL, "label": "M", "pads_compared": n, "differences": diffs,
            "reference": {"file": str(path).replace("\\", "/"), "sha256": sha256_file(Path(path))}}


# ------------------------------------------------------------------------------------------------ rule file
RULE_KEYS = (("board", "design_settings", "rules"), ("board", "design_settings", "rule_severities"),
             ("board", "design_settings", "track_widths"), ("board", "design_settings", "via_dimensions"),
             ("net_settings", "classes"), ("net_settings", "netclass_patterns"))


def rule_sections(p: Path) -> dict:
    d = json.loads(p.read_text(encoding="utf-8"))
    out = {}
    for path in RULE_KEYS:
        v = d
        for k in path:
            v = v.get(k) if isinstance(v, dict) else None
        out["/".join(path)] = v
    return out


def canon(v) -> str:
    return hashlib.sha256(json.dumps(v, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def check_rules() -> dict:
    old_p = REV2C / "kicad" / "vtol_carrier.kicad_pro"
    if not PRO.is_file() or not old_p.is_file():
        return {"result": NOT_VERIFIED, "label": "M", "reason": f"missing {PRO if not PRO.is_file() else old_p}"}
    new, old = rule_sections(PRO), rule_sections(old_p)
    rows = {}
    for k in new:
        if k == "net_settings/netclass_patterns":
            # the pattern list names nets; rev 3 adds / removes nets, the classes themselves must not change
            a = sorted((p.get("netclass"), p.get("pattern")) for p in (new[k] or []))
            b = sorted((p.get("netclass"), p.get("pattern")) for p in (old[k] or []))
            rows[k] = {"equal": a == b, "only_rev3": [list(x) for x in a if x not in b], "only_rev2c": [list(x) for x in b if x not in a]}
        else:
            rows[k] = {"equal": canon(new[k]) == canon(old[k]), "sha256_rev3": canon(new[k]), "sha256_rev2c": canon(old[k])}
    strict = [k for k in rows if k != "net_settings/netclass_patterns" and not rows[k]["equal"]]
    return {"result": PASS if not strict else FAIL, "label": "M", "sections": rows, "sections_differing": strict,
            "file_sha256": {"rev3": sha256_file(PRO), "rev2c": sha256_file(old_p)},
            "note": "the raw file hashes differ whenever pcbnew rewrites the project file on save (window / layer "
                    "presets, net-name pattern list); the rule sections are compared as canonical JSON. Pattern-list "
                    "changes (nets added / removed by rev 3) are listed, not judged",
            "criterion": "design rules, rule severities, predefined track / via sizes and the net classes are equal to "
                         "the archived rev 2c project file"}


# ------------------------------------------------------------------------------------------------ silkscreen
def check_silk(board, fps) -> dict:
    ctx = SILK.Ctx(board)
    texts = [(d.GetText(), d.GetLayerName(), pt(d.GetPosition())) for d in board.GetDrawings() if d.GetClass() == "PCB_TEXT"
             and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS)]
    dots = [pt(d.GetCenter()) for d in board.GetDrawings() if d.GetClass() == "PCB_SHAPE" and d.GetLayer() == pcbnew.F_SilkS
            and d.GetShape() == pcbnew.SHAPE_T_CIRCLE]
    rows, bad = [], []
    for ref, kind in sorted(SILK.MARKS.items()):
        fp = fps[ref]
        lib = SILK.library_mark(ctx, fp)
        pb = box(pad(fps, ref, "1").GetBoundingBox())
        if kind == "dot":
            extra = [list(map(lambda v: round(v, 2), p)) for p in dots if rect_dist(p[0], p[1], pb) <= SILK.MARK_MAX_D + 0.5]
        else:
            extra = [list(map(lambda v: round(v, 2), p)) for s, lay, p in texts if s == kind and lay == "F.SilkS"
                     and rect_dist(p[0], p[1], pb) <= SILK.MARK_MAX_D + 0.8]
        ok = lib["silk_graphics"] > 0 and not lib["covered_by"]
        rows.append({"ref": ref, "footprint": fp.GetFPID().GetUniStringLibItemName(), "library_mark": lib,
                     "extra_mark": kind if extra else None, "extra_mark_at": extra[:1], "ok": ok})
        if not ok:
            bad.append(ref)
    rev = [(s, lay) for s, lay, _p in texts if s == SILK.REV_TEXT]
    names = {r: [(s, lay) for s, lay, _p in texts if s.startswith(SILK.LABELS[r] + " ")] for r in ("J1", "J2", "J5", "J7")}
    refs = {}
    for r in SILK.REFS_FIRST:
        f = fps[r].Reference()
        if f.GetLayer() == pcbnew.F_SilkS and f.IsVisible():
            refs[r] = "F.SilkS (footprint designator)"
        else:
            hit = [lay for s, lay, _p in texts if s == r]
            refs[r] = f"{hit[0]} (board text beneath the part)" if hit else None
    ok = not bad and bool(rev) and all(names.values()) and all(refs.values())
    return {"result": PASS if ok else FAIL, "label": "M", "marks": rows, "marks_not_ok": bad,
            "extra_marks_placed": sum(1 for r in rows if r["extra_mark"]),
            "library_mark_only": [r["ref"] for r in rows if not r["extra_mark"]],
            "revision_text": rev, "pin_order_names": names, "boost_block_designators": refs,
            "criterion": "every listed footprint keeps its library F.SilkS polarity / pin-1 graphics with none of them "
                         "under another part's body (F.Fab box) or on a pad (extra '+', 'K' or dot marks are counted "
                         "separately); the text 'rev 3' exists; J1 / J2 / J5 / J7 carry a name with the pin order; "
                         "U8 / U9 / L5 designators exist on a silkscreen layer"}


def main() -> int:
    board = pcbnew.LoadBoard(str(BOARD))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.BuildConnectivity()
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    checks = {
        "boost_sw": check_boost_sw(board, fps),
        "vout_vias": check_vout_vias(board, fps),
        "gnd_vias": check_gnd_vias(board, fps),
        "in1_under_block": check_in1(board, fps),
        "boost_fb": check_boost_fb(board, fps),
        "l5_keepout": check_l5(board, fps),
        "battery_path": check_battery(board, fps),
        "hot_loop": check_hot_loop(board, fps),
        "input_cap": check_input_cap(board, fps),
        "crystals": check_crystals(board, fps),
        "u8_u9_pads": check_u8_u9(fps),
        "mcu_pinmap": check_mcu(fps),
        "rule_file": check_rules(),
        "silkscreen": check_silk(board, fps),
    }
    results = {k: v["result"] for k, v in checks.items()}
    res = {"tool": "scripts/routing_checks.py (pcbnew " + pcbnew.GetBuildVersion() + ")",
           "board": {"file": str(BOARD).replace("\\", "/"), "sha256": sha256_file(BOARD)},
           "result": FAIL if FAIL in results.values() else (NOT_VERIFIED if NOT_VERIFIED in results.values() else PASS),
           "results": results, "checks": checks}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(res, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"result": res["result"], "results": results,
                      "battery_path_ohm": checks["battery_path"].get("copper_resistance_ohm_20C"),
                      "narrower_than_2mm_mm": checks["battery_path"].get("current_carrying_length_narrower_than_2mm"),
                      "XTAL1_worst_mm": checks["crystals"].get("XTAL1_worst_mm")}, indent=1))
    return 0 if res["result"] == PASS else 1


if __name__ == "__main__":
    sys.exit(main())
