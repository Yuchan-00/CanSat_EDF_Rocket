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
  hot_loop_top_gnd  relayout step: U8.4, C57.2, C43.2 and the input capacitor's GND pad on ONE F.Cu GND copper piece,
                    each with its own GND via into the In1 fill on that piece (review E-R1)
  input_cap_vin     relayout step: copper path of an input capacitor (C41 / C42) to U8.3 <= 5.0 mm and its GND pad on
                    the U8.4 piece (review E-R2, datasheet: input capacitor close to VIN and GND)
  input_cap_l5      relayout step: copper path of an input capacitor to L5.1 <= 5.0 mm (review E-R2, target 'a few mm')
  fb_clear_of_caps  relayout step: no BOOST_FB copper inside the pad span of an output capacitor (review E-R3)
  rail_edge_copper  relayout step: copper >= 0.4 mm from the two 76 mm edges y 65 / y 135 that carry the JLC rails
                    (review J2); the 0.3 mm board rule itself is unchanged
  sw_node           relayout-fix step (review RR-1): plan lines 449 / 466 - U8.5 and L5.2 next to each other (<= 2.0 mm
                    pad gap, E) and the BOOST_SW track under no other part. boost_sw above stays as it was: its
                    criterion (track <= pad centre distance) holds for any placement and says nothing about nearness
  crystal_distance  relayout-fix step (review RR-1): plan line 452 - switching parts and BOOST_SW copper not nearer to
                    the Y1 pads / U1 XTAL pins than the J6 (Pololu) pads of the archived rev 2c board
  information       (not a check) C59.2 ground via and the battery vias at L5.1 (review RR-4 / RR-5)
  crystals          XTAL cluster paths and XTAL net lengths against the archived rev 2c board_metrics.json
  u8_u9_pads        pad -> net of U8 / U9 on the board against rev3_checks.expected_pad_nets (datasheet table)
  mcu_pinmap        the 256 pads of U1..U4 against the rev 2c pin map
  rule_file         design-rule sections of kicad/vtol_carrier.kicad_pro against the archived rev 2c project file
  silkscreen        polarity / pin-1 marks, 'rev 3' text, pin-order names of J1 / J2 / J5 / J7; esp-user-fix step: the
                    glyph order of every pin-ordered name against the pad positions (pin_order_labels); silk-id step: the two
                    identification lines (repository address, contact e-mail) - size, mirroring, gaps to pads / vias /
                    other silkscreen / the outline (the strings are never written to the report)

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
import outline as OL  # noqa: E402  (redesign2: the real outline with the tab)

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


# ------------------------------------------------------------------ relayout step checks (reviews E-R1 / E-R2 / E-R3 / J2)
FEW_MM = 5.0          # E: this script's reading of the step target "a few mm" for the input-capacitor copper paths
RAIL_EDGE_MM = 0.4    # F: JLC capabilities page as read by the JLC reviewer (copper to V-cut edge >= 0.4 mm)
BOARD_BOX = (62.0, 65.0, 138.0, 135.0)


class _FcuGnd:
    """Union of the F.Cu GND copper (zone islands, tracks, pads; vias are NOT joins): piece id of any F.Cu shape."""

    def __init__(self, board, fps):
        polys = []
        for z in zone_of(board, "GND", pcbnew.F_Cu):
            ps = z.GetFilledPolysList(pcbnew.F_Cu)
            polys += [ps.UnitSet(i) for i in range(ps.OutlineCount())]
        self.shapes = [("poly", p) for p in polys]
        self.shapes += [("track", t.GetEffectiveShape(pcbnew.F_Cu)) for t in tracks_of(board, "GND") if t.GetLayer() == pcbnew.F_Cu]
        self.shapes += [("pad", p.GetEffectiveShape(pcbnew.F_Cu)) for fp in fps.values() for p in fp.Pads()
                        if p.GetNetname() == "GND" and p.IsOnLayer(pcbnew.F_Cu)]
        n = len(self.shapes)
        self.parent = list(range(n))
        for i in range(n):
            for j in range(i + 1, n):
                if self.shapes[i][0] == "poly" and self.shapes[j][0] == "poly":
                    continue
                if self.find(i) != self.find(j) and self.shapes[i][1].Collide(self.shapes[j][1], 0):
                    self.parent[self.find(i)] = self.find(j)

    def find(self, i: int) -> int:
        while self.parent[i] != i:
            self.parent[i] = self.parent[self.parent[i]]
            i = self.parent[i]
        return i

    def piece(self, shape):
        hit = sorted({self.find(i) for i, (_k, s) in enumerate(self.shapes) if s.Collide(shape, 0)})
        return hit[0] if hit else None


def input_cap_choice(board, fps) -> tuple[str, list[dict]]:
    """The input capacitor (C41 / C42) with the shortest copper path to U8.3; rows for both."""
    rows = []
    for c in ("C41", "C42"):
        a = network(board, fps, "VBAT_LOGIC_SW", (c, "1"), ("U8", "3"))
        b = network(board, fps, "VBAT_LOGIC_SW", (c, "1"), ("L5", "1"))
        rows.append({"cap": c, "value": fps[c].GetValue(),
                     "to_U8.3_copper_path_mm": a.get("shortest_copper_path_mm"), "to_U8.3_vias": a.get("vias_on_shortest_path"),
                     "to_U8.3_ohm_E": a.get("ohm"),
                     "to_L5.1_copper_path_mm": b.get("shortest_copper_path_mm"), "to_L5.1_vias": b.get("vias_on_shortest_path"),
                     "to_L5.1_ohm_E": b.get("ohm"),
                     "pad1_edge_gap_to_U8.3_mm": shape_gap(pad(fps, c, "1").GetEffectiveShape(pcbnew.F_Cu),
                                                           pad(fps, "U8", "3").GetEffectiveShape(pcbnew.F_Cu)),
                     "pad1_edge_gap_to_L5.1_mm": shape_gap(pad(fps, c, "1").GetEffectiveShape(pcbnew.F_Cu),
                                                           pad(fps, "L5", "1").GetEffectiveShape(pcbnew.F_Cu))})
    best = min(rows, key=lambda r: (r["to_U8.3_copper_path_mm"] if r["to_U8.3_copper_path_mm"] is not None else 1e9, r["cap"]))
    return best["cap"], rows


def check_hot_loop_top_gnd(board, fps) -> dict:
    cin, _rows = input_cap_choice(board, fps)
    g = _FcuGnd(board, fps)
    items = [("U8", "4"), ("C57", "2"), ("C43", "2"), (cin, "2")]
    piece = {f"{r}.{n}": g.piece(pad(fps, r, n).GetEffectiveShape(pcbnew.F_Cu)) for r, n in items}
    same = len({v for v in piece.values()}) == 1 and None not in piece.values()
    in1 = zone_of(board, "GND", pcbnew.In1_Cu)
    vias = []
    for v in vias_of(board, "GND"):
        x, y = pt(v.GetPosition())
        near = {f"{r}.{n}": round(rect_dist(x, y, box(pad(fps, r, n).GetBoundingBox())), 3) for r, n in items}
        near = {k: d for k, d in near.items() if d <= VIA_NEAR_MM}
        if not near:
            continue
        vias.append({"at": [round(x, 3), round(y, 3)], "drill_mm": mm(v.GetDrillValue()), "within_1.5mm_of": near,
                     "in_In1_fill": in_fill(in1, pcbnew.In1_Cu, x, y), "fcu_gnd_piece": g.piece(v.GetEffectiveShape(pcbnew.F_Cu))})
    # own via: a matching pad -> distinct via (in the In1 fill, on the pad's F.Cu piece, within 1.5 mm)
    cand = {k: [i for i, v in enumerate(vias) if k in v["within_1.5mm_of"] and v["in_In1_fill"] and v["fcu_gnd_piece"] == piece[k]]
            for k in piece}
    match: dict[int, str] = {}

    def assign(k: str, seen: set) -> bool:
        for i in sorted(cand[k], key=lambda i: vias[i]["within_1.5mm_of"][k]):
            if i in seen:
                continue
            seen.add(i)
            if i not in match or assign(match[i], seen):
                match[i] = k
                return True
        return False
    own = {k: assign(k, set()) for k in sorted(piece, key=lambda k: (len(cand[k]), k))}
    own_via = {k: vias[i]["at"] for i, k in sorted(match.items())}
    gaps = {f"{c}.2 -> U8.4": shape_gap(pad(fps, c, "2").GetEffectiveShape(pcbnew.F_Cu), pad(fps, "U8", "4").GetEffectiveShape(pcbnew.F_Cu))
            for c in ("C57", "C43", cin)}
    ok = same and all(own.values())
    return {"result": PASS if ok else FAIL, "label": "M", "input_capacitor": cin, "fcu_gnd_piece": piece,
            "all_on_one_piece": same, "pads_with_own_via": own, "own_via_at": own_via, "gnd_vias_near_the_pads": vias,
            "pad_edge_gap_mm": gaps,
            "criterion": "U8.4, C57.2, C43.2 and the GND pad of the input capacitor nearest to U8.3 (by copper path) lie on ONE "
                         "F.Cu GND copper piece (zone islands + tracks + pads; vias are not joins), and each of the four pads "
                         "can be given its own GND via: within 1.5 mm of the pad, inside the In1 GND fill, on that F.Cu piece"}


def check_input_cap_vin(board, fps) -> dict:
    cin, rows = input_cap_choice(board, fps)
    r = [x for x in rows if x["cap"] == cin][0]
    g = _FcuGnd(board, fps)
    pc, pu = g.piece(pad(fps, cin, "2").GetEffectiveShape(pcbnew.F_Cu)), g.piece(pad(fps, "U8", "4").GetEffectiveShape(pcbnew.F_Cu))
    path = r["to_U8.3_copper_path_mm"]
    ok = path is not None and path <= FEW_MM and pc is not None and pc == pu
    return {"result": PASS if ok else FAIL, "label": "M (path length from the track / via model of network(), zone copper not modelled)",
            "input_capacitor": cin, "copper_path_to_U8.3_mm": path, "vias_on_path": r["to_U8.3_vias"], "limit_mm": FEW_MM,
            "gnd_pad_piece": pc, "U8.4_piece": pu, "gnd_pad_on_the_U8.4_piece": pc is not None and pc == pu, "rows": rows,
            "criterion": f"an input capacitor (C41 / C42, 22 uF) reaches U8.3 (VIN) over <= {FEW_MM} mm of track (E: this "
                         "script's reading of 'a few mm') and its GND pad is on the same F.Cu GND piece as U8.4"}


def check_input_cap_l5(board, fps) -> dict:
    _cin, rows = input_cap_choice(board, fps)
    vals = [(x["to_L5.1_copper_path_mm"], x["cap"]) for x in rows if x["to_L5.1_copper_path_mm"] is not None]
    best = min(vals) if vals else (None, None)
    straight = shape_gap(pad(fps, "U8", "3").GetEffectiveShape(pcbnew.F_Cu), pad(fps, "L5", "1").GetEffectiveShape(pcbnew.F_Cu))
    ok = best[0] is not None and best[0] <= FEW_MM
    return {"result": PASS if ok else FAIL, "label": "M (path length from the track / via model of network(), zone copper not modelled)",
            "input_capacitor": best[1], "copper_path_to_L5.1_mm": best[0], "limit_mm": FEW_MM,
            "pad_edge_distance_U8.3_to_L5.1_mm": straight, "rows": rows,
            "criterion": f"an input capacitor (C41 / C42) reaches L5.1 over <= {FEW_MM} mm of track (E: reading of 'a few mm'). "
                         "The straight pad distance U8.3 - L5.1 is reported: one capacitor cannot be nearer to both than that"}


def check_fb_clear_of_caps(board, fps) -> dict:
    hits, spans = [], {}
    fb = [(t, t.GetEffectiveShape(pcbnew.F_Cu)) for t in tracks_of(board, "BOOST_FB") if t.GetLayer() == pcbnew.F_Cu]
    fb += [(v, v.GetEffectiveShape(pcbnew.F_Cu)) for v in vias_of(board, "BOOST_FB")]
    for c in ("C57", "C43"):
        bs = [box(p.GetBoundingBox()) for p in fps[c].Pads()]
        x0, y0, x1, y1 = min(b[0] for b in bs), min(b[1] for b in bs), max(b[2] for b in bs), max(b[3] for b in bs)
        spans[c] = [round(v, 3) for v in (x0, y0, x1, y1)]
        rect = pcbnew.SHAPE_RECT(pcbnew.VECTOR2I(pcbnew.FromMM(x0), pcbnew.FromMM(y0)), pcbnew.FromMM(x1 - x0), pcbnew.FromMM(y1 - y0))
        for t, sh in fb:
            if sh.Collide(rect, 0):
                if t.GetClass() == "PCB_VIA":
                    hits.append({"cap": c, "via": [round(v, 2) for v in pt(t.GetPosition())]})
                else:
                    hits.append({"cap": c, "track": [round(v, 2) for v in (*pt(t.GetStart()), *pt(t.GetEnd()))], "w": round(mm(t.GetWidth()), 3)})
    gap = min((shape_gap(sh, p.GetEffectiveShape(pcbnew.F_Cu)) for _t, sh in fb for c in ("C57", "C43") for p in fps[c].Pads()),
              default=None)
    length = round(sum(mm(t.GetLength()) for t, _s in fb if t.GetClass() != "PCB_VIA"), 3)
    return {"result": PASS if fb and not hits else FAIL, "label": "M", "pad_span_boxes_kicad_mm": spans, "boost_fb_copper_inside": hits,
            "min_gap_boost_fb_track_to_output_cap_pad_mm": gap, "boost_fb_f_cu_track_length_mm": length,
            "boost_fb_vias": len(vias_of(board, "BOOST_FB")),
            "criterion": "no BOOST_FB F.Cu track and no BOOST_FB via touches the pad span box (bounding box of both pads) of "
                         "the output capacitors C57 / C43"}


def check_rail_edges(board, fps) -> dict:
    x0, y0, x1, y1 = BOARD_BOX
    # outline adaptation (redesign2 Layout stage): the y-max side is no longer one straight line. A box that lies
    # beside the tab is measured exactly as before (y 135 - bottom); otherwise the exact distance to every y-max side
    # segment of scripts/outline.py (y 135 pieces, fillets, tab sides, outer arcs, tab bottom) - OL.minus_y_distance
    sides = {"y65 (top, rail)": lambda b: b[1] - y0, "y135 (bottom, rail)": lambda b: OL.minus_y_distance(b),
             "x62": lambda b: b[0] - x0, "x138": lambda b: x1 - b[2]}
    rows: dict = {s: {} for s in sides}

    def put(kind: str, b, what: str, only: tuple = ()) -> None:
        for s, fn in sides.items():
            if only and s not in only:
                continue
            d = round(fn(b), 4)
            cur = rows[s].get(kind)
            if cur is None or d < cur["min_mm"]:
                rows[s][kind] = {"min_mm": d, "item": what}
    beside = (OL.TAB_X0 - OL.TAB_R_IN, OL.TAB_X1 + OL.TAB_R_IN)
    for z in board.Zones():
        if z.GetIsRuleArea():
            continue
        for lay in z.GetLayerSet().Seq():
            ps = z.GetFilledPolysList(lay)
            if ps.OutlineCount():
                what = f"{z.GetNetname()} fill on {board.GetLayerName(lay)}"
                kind = f"zone fill {board.GetLayerName(lay)}"
                put(kind, box(ps.BBox()), what, ("y65 (top, rail)", "x62", "x138"))
                # y-max side: the fill outlines themselves (bounding box of a fill that reaches into the tab says nothing)
                best = None
                for i in range(ps.OutlineCount()):
                    ch = ps.Outline(i)
                    pts = [pt(ch.CPoint(k)) for k in range(ch.PointCount())]
                    for a_, b_ in zip(pts, pts[1:] + pts[:1]):
                        if max(a_[1], b_[1]) < y1 - 3.0:
                            continue
                        if (a_[0] < beside[0] and b_[0] < beside[0]) or (a_[0] > beside[1] and b_[0] > beside[1]):
                            d = y1 - max(a_[1], b_[1])
                        else:
                            d = min(OL.seg_prim_distance(a_, b_, q) for q in OL.MINUS_Y_PRIMITIVES)
                        best = d if best is None else min(best, d)
                if best is not None:
                    d = round(best, 4)
                    cur = rows["y135 (bottom, rail)"].get(kind)
                    if cur is None or d < cur["min_mm"]:
                        rows["y135 (bottom, rail)"][kind] = {"min_mm": d, "item": what}
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            x, y = pt(t.GetPosition())
            r = mm(t.GetWidth(pcbnew.F_Cu)) / 2
            put("via", (x - r, y - r, x + r, y + r), f"{t.GetNetname()} via ({x:.3f}, {y:.3f})")
        else:
            (ax, ay), (bx, by) = pt(t.GetStart()), pt(t.GetEnd())
            r = mm(t.GetWidth()) / 2
            put("track", (min(ax, bx) - r, min(ay, by) - r, max(ax, bx) + r, max(ay, by) + r),
                f"{t.GetNetname()} {t.GetLayerName()} ({ax:.3f}, {ay:.3f})-({bx:.3f}, {by:.3f}) w {2 * r:.3f}")
    for fp in fps.values():
        for p in fp.Pads():
            put("pad", box(p.GetBoundingBox()), f"{fp.GetReference()}.{p.GetNumber()} {p.GetNetname()}")
    mins = {s: min(v["min_mm"] for v in k.values()) for s, k in rows.items()}
    rail = [s for s in sides if "rail" in s]
    bad = [f"{s}: {k} {v['min_mm']} mm ({v['item']})" for s in rail for k, v in sorted(rows[s].items()) if v["min_mm"] < RAIL_EDGE_MM - 1e-9]
    return {"result": PASS if not bad else FAIL,
            "label": "M (zones refilled in memory; distances to the outline lines / segments of scripts/outline.py, bounding boxes)",
            "limit_mm_rail_edges": RAIL_EDGE_MM, "min_copper_to_edge_mm": mins, "per_edge": rows, "below_limit": bad,
            "outline": OL.describe(),
            "criterion": f"zone fills (all copper layers), tracks, vias and pads keep >= {RAIL_EDGE_MM} mm from the two 76 mm "
                         "edges y 65 and y 135 (JLC edge rails, review J2) - on the y 135 side from EVERY segment of the "
                         "outline there, the tab included (redesign2 outline adaptation); the other two edges are reported "
                         "against the unchanged 0.3 mm board rule"}


# ---------------------------------------------------- relayout-fix step checks (review of the relayout, RR-1 / RR-4 / RR-5)
SW_NEXT_MM = 2.0      # E: reading of 'L5 next to U8 pins 3 / 5' with the plan's own 2.0 mm of line 448 (no number in line 449)
#: the switching parts whose distance to the NAV crystal the plan compares with the rev 2c Pololu pins (= place.SWITCHING_PARTS)
SWITCHING_REFS = ("U8", "L5", "C57", "C43", "C41")
L5_VIA_MM = 4.0       # information only: VBAT_LOGIC_SW vias this close to the L5.1 pad are listed (review RR-5)


def _pad_span(fp):
    """Bounding box of all pads of a footprint (the strip a track crosses when it runs 'under the part')."""
    bs = [box(p.GetBoundingBox()) for p in fp.Pads()]
    return (min(b[0] for b in bs), min(b[1] for b in bs), max(b[2] for b in bs), max(b[3] for b in bs))


def check_sw_node(board, fps) -> dict:
    """plan line 449: 'L5 next to U8 pins 3 / 5 ... BOOST_SW copper only as large as the two pads need'; line 466:
    'BOOST_SW: F.Cu only, no via, as short as the pads allow'. check_boost_sw only compares the track with the pad
    centre distance, which holds for ANY placement (review RR-1), so the placement itself is judged here."""
    a, b = pad(fps, "U8", "5"), pad(fps, "L5", "2")
    gap = shape_gap(a.GetEffectiveShape(pcbnew.F_Cu), b.GetEffectiveShape(pcbnew.F_Cu))
    tr = tracks_of(board, "BOOST_SW")
    length = round(sum(mm(t.GetLength()) for t in tr), 3)
    area = round(sum(mm(t.GetLength()) * mm(t.GetWidth()) for t in tr), 3)
    ohm = round(sum(SHEET_OHM * mm(t.GetLength()) / mm(t.GetWidth()) for t in tr if t.GetWidth()), 5)
    under = []
    for ref in sorted(fps):
        if ref in ("U8", "L5") or not list(fps[ref].Pads()):
            continue
        span = _pad_span(fps[ref])
        n = 0
        for t in tr:
            if t.GetLayer() != pcbnew.F_Cu:
                continue
            (ax, ay), (bx, by) = pt(t.GetStart()), pt(t.GetEnd())
            steps = max(1, int(math.hypot(bx - ax, by - ay) / 0.02))
            r = mm(t.GetWidth()) / 2
            # the track's copper (centre line sampled every 0.02 mm, half width r) reaches into the pad span box
            if any(rect_dist(ax + (bx - ax) * i / steps, ay + (by - ay) * i / steps, span) < r for i in range(steps + 1)):
                n += 1
        if n and not fps[ref].IsFlipped():
            gaps = [shape_gap(t.GetEffectiveShape(pcbnew.F_Cu), p.GetEffectiveShape(pcbnew.F_Cu))
                    for t in tr for p in fps[ref].Pads() if p.IsOnLayer(pcbnew.F_Cu)]
            under.append({"ref": ref, "sw_segments_inside_pad_span": n, "min_gap_sw_track_to_its_pads_mm": min(gaps)})
    ok = gap <= SW_NEXT_MM and not under
    return {"result": PASS if ok else FAIL, "label": "M (resistance from track length / width, E)",
            "pad_edge_gap_U8.5_L5.2_mm": gap, "limit_mm": SW_NEXT_MM, "track_length_mm": length,
            "widths_mm": sorted({round(mm(t.GetWidth()), 3) for t in tr}), "track_copper_area_mm2": area,
            "track_resistance_ohm_20C_E": ohm, "parts_the_sw_track_passes_under": under,
            "criterion": f"plan lines 449 / 466: the SW pads U8.5 and L5.2 are within {SW_NEXT_MM} mm of each other (copper "
                         "edge gap; E: this script's reading of 'L5 next to U8 pins 3 / 5' with the 2.0 mm of plan line 448 - "
                         "the plan gives no number) AND the BOOST_SW track crosses the pad span of no other part "
                         "('copper only as large as the two pads need')"}


def check_crystal_distance(board, fps) -> dict:
    """plan line 452: 'Keep the block at least as far from the NAV crystal Y1 / U1 XTAL pins as the rev 2c Pololu pins
    were.' Reference = the J6 pads of the archived rev 2c board, measured here the same way (copper edge to copper
    edge in plan view). place.py reports the same note with pad centres (placement_checks.rev3.crystal_distance)."""
    old_path = REV2C / "kicad" / "vtol_carrier.kicad_pcb"
    if not old_path.is_file():
        return {"result": NOT_VERIFIED, "label": "-", "why": f"{old_path.name} not found in the rev 2c archive"}
    old = pcbnew.LoadBoard(str(old_path))
    ofps = {fp.GetReference(): fp for fp in old.GetFootprints()}

    def targets(f) -> dict:
        return {"Y1 pads": [p.GetEffectiveShape(pcbnew.F_Cu) for p in f["Y1"].Pads()],
                "U1 XTAL pins 23 / 24": [pad(f, "U1", n).GetEffectiveShape(pcbnew.F_Cu) for n in ("23", "24")]}

    def xtal_copper(brd, f) -> tuple[list, list]:
        nets = sorted({pad(f, "U1", n).GetNetname() for n in ("23", "24")})
        return nets, [s for n in nets for _w, s in copper_shapes(brd, f, n)]

    def gaps(shapes: list, tg: dict, xt: list) -> dict:
        out = {k: min(shape_gap(s, t) for s in shapes for t in v) for k, v in tg.items()}
        out["NAV XTAL net copper (tracks, vias, pads)"] = min(shape_gap(s, t) for s in shapes for t in xt)
        return out
    if "J6" not in ofps:
        return {"result": NOT_VERIFIED, "label": "-", "why": "no J6 on the archived rev 2c board"}
    onets, oxt = xtal_copper(old, ofps)
    nnets, nxt = xtal_copper(board, fps)
    ref = gaps([p.GetEffectiveShape(pcbnew.F_Cu) for p in ofps["J6"].Pads()], targets(ofps), oxt)
    tg = targets(fps)
    parts = {r: gaps([p.GetEffectiveShape(pcbnew.F_Cu) for p in fps[r].Pads()], tg, nxt) for r in SWITCHING_REFS}
    sw_shapes = [t.GetEffectiveShape(t.GetLayer()) for t in tracks_of(board, "BOOST_SW")]
    sw_shapes += [pad(fps, "U8", "5").GetEffectiveShape(pcbnew.F_Cu), pad(fps, "L5", "2").GetEffectiveShape(pcbnew.F_Cu)]
    sw = gaps(sw_shapes, tg, nxt)
    hard = ("Y1 pads", "U1 XTAL pins 23 / 24")
    closer = sorted(r for r, g in parts.items() if any(g[k] < ref[k] - 1e-6 for k in hard))
    sw_closer = [k for k in hard if sw[k] < ref[k] - 1e-6]
    ok = not closer and not sw_closer
    return {"result": PASS if ok else FAIL, "label": "M (this board and the archived rev 2c board, pcbnew)",
            "rev2c_J6_pins_min_gap_mm": ref, "rev3_parts_min_gap_mm": parts, "rev3_BOOST_SW_copper_min_gap_mm": sw,
            "parts_closer_than_the_rev2c_pins": closer, "BOOST_SW_copper_closer_than_the_rev2c_pins_to": sw_closer,
            "xtal_nets": {"rev2c": onets, "rev3": nnets},
            "reference": {"file": str(old_path).replace("\\", "/"), "sha256": sha256_file(old_path)},
            "criterion": "plan line 452: no pad of U8 / L5 / C57 / C43 / C41 and no BOOST_SW copper (track + its two pads) is "
                         "nearer to the Y1 pads or to U1 pins 23 / 24 than the nearest J6 (Pololu) pad was on the rev 2c "
                         "board (copper edge gap in plan view, 1 um). The gap to the NAV XTAL net copper is reported, not judged"}


def information(board, fps) -> dict:
    """Measured facts that are not requirements of the plan (review RR-4 / RR-5): reported, never judged."""
    g = _FcuGnd(board, fps)
    c59 = pad(fps, "C59", "2")
    piece = g.piece(c59.GetEffectiveShape(pcbnew.F_Cu))
    cb = box(c59.GetBoundingBox())
    in1 = zone_of(board, "GND", pcbnew.In1_Cu)
    near = []
    for v in vias_of(board, "GND"):
        x, y = pt(v.GetPosition())
        d = rect_dist(x, y, cb)
        if d <= VIA_NEAR_MM:
            near.append({"at": [round(x, 3), round(y, 3)], "to_pad_mm": round(d, 3), "in_In1_fill": in_fill(in1, pcbnew.In1_Cu, x, y),
                         "on_the_pad_F.Cu_gnd_piece": g.piece(v.GetEffectiveShape(pcbnew.F_Cu)) == piece})
    on_piece = [v for v in vias_of(board, "GND") if piece is not None and g.piece(v.GetEffectiveShape(pcbnew.F_Cu)) == piece]
    lb = box(pad(fps, "L5", "1").GetBoundingBox())
    l5 = []
    for v in vias_of(board, "VBAT_LOGIC_SW"):
        x, y = pt(v.GetPosition())
        d = rect_dist(x, y, lb)
        if d <= L5_VIA_MM:
            l5.append({"at": [round(x, 3), round(y, 3)], "to_pad_mm": round(d, 3), "drill_mm": mm(v.GetDrillValue()),
                       "size_mm": mm(v.GetWidth(pcbnew.F_Cu))})
    l5.sort(key=lambda r: (r["to_pad_mm"], r["at"]))
    return {"label": "M", "note": "not requirements of the plan: reported for the user, never part of the result",
            "C59.2_gnd": {"gnd_vias_within_1.5mm_of_the_pad": near,
                          "own_via": any(v["on_the_pad_F.Cu_gnd_piece"] and v["in_In1_fill"] for v in near),
                          "gnd_vias_on_its_F.Cu_gnd_piece": len(on_piece),
                          "nearest_via_on_its_piece_mm": min((round(rect_dist(*pt(v.GetPosition()), cb), 3) for v in on_piece), default=None)},
            "L5.1_battery_vias": {"radius_mm": L5_VIA_MM, "vias_within_radius_of_the_pad": l5, "count": len(l5)}}


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
    # rev 3.1: pins a plan amendment changes on purpose (U2.7 -> RELEASE_PWM) are compared with the declared net
    declared_problems: list[str] = []
    declared = RC.declared_mcu_pin_changes()
    ref = RC.apply_declared(ref, declared, declared_problems)
    diffs, n = [{"declared_change_problem": p} for p in declared_problems], 0
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
            "declared_changes_by_plan_amendment": declared,
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
def _box_gap(a, b) -> float:
    """Distance of two axis-aligned boxes (0 when they touch or overlap)."""
    return math.hypot(max(b[0] - a[2], 0.0, a[0] - b[2]), max(b[1] - a[3], 0.0, a[1] - b[3]))


def _ink_box(item):
    """Extent of what is printed: the strokes of a text (its bounding box carries about 0.25 mm of empty line
    spacing), the bounding box of a graphic shape."""
    if hasattr(item, "GetEffectiveTextShape"):
        return box(item.GetEffectiveTextShape().BBox())
    return box(item.GetBoundingBox())


def id_lines(board) -> dict:
    """silk-id step: the two identification lines (silk.ID_LINES) measured on the board, independently of the bitmap
    silk.py placed them with: stroke shape against every pad / via shape (SHAPE.Collide), ink box against the other
    silkscreen items, the outline and the cable-tie band. The strings themselves are not written to the report."""
    texts = [d for d in board.GetDrawings() if d.GetClass() == "PCB_TEXT"]
    eb = box(board.GetBoardEdgesBoundingBox())
    rows, problems, found = [], [], []
    for n, (kind, s) in enumerate(SILK.ID_LINES, start=1):
        hit = [d for d in texts if d.GetText() == s]
        if len(hit) != 1:
            problems.append(f"line {n} ({kind}): on the board {len(hit)} times, expected once")
            continue
        found.append((n, kind, s, hit[0]))
    for n, kind, s, d in found:
        lay = d.GetLayer()
        back = lay == pcbnew.B_SilkS
        cu, mask = (pcbnew.B_Cu, pcbnew.B_Mask) if back else (pcbnew.F_Cu, pcbnew.F_Mask)
        shape = d.GetEffectiveTextShape()
        ink = box(shape.BBox())
        pad_gap, pad_name = None, None
        for fp in board.GetFootprints():
            for p in fp.Pads():
                if not (p.IsOnLayer(cu) or p.IsOnLayer(mask) or p.GetDrillSize().x > 0):
                    continue
                g = _box_gap(ink, box(p.GetBoundingBox()))
                if g < 5.0 and p.IsOnLayer(cu):
                    g = shape_gap(shape, p.GetEffectiveShape(cu))
                if pad_gap is None or g < pad_gap:
                    pad_gap, pad_name = g, f"{fp.GetReference()}.{p.GetNumber()}"
        via_gap, via_at, under = None, None, 0
        for v in board.GetTracks():
            if v.GetClass() != "PCB_VIA" or not v.IsOnLayer(cu):
                continue
            c = pt(v.GetPosition())
            g = rect_dist(c[0], c[1], ink) - mm(v.GetWidth(cu)) / 2
            if g < 5.0:
                g = shape_gap(shape, v.GetEffectiveShape(cu))
            under += g <= 0
            if via_gap is None or g < via_gap:
                via_gap, via_at = g, [round(c[0], 3), round(c[1], 3)]
        me = d.m_Uuid.AsString()
        others = [_ink_box(o) for o in board.GetDrawings() if o.GetLayer() == lay and o.m_Uuid.AsString() != me]
        for fp in board.GetFootprints():
            others += [_ink_box(g) for g in fp.GraphicalItems() if g.GetLayer() == lay]
            others += [_ink_box(f) for f in fp.GetFields() if f.GetLayer() == lay and f.IsVisible()]
        silk_gap = min((_box_gap(ink, o) for o in others), default=None)
        # outline adaptation (redesign2 fix stage): the edge bounding box now ends at the tab bottom, so the y-max side
        # is measured against the real '-y' edge segments (y 135 beside the tab, fillets, tab sides / arcs / bottom)
        edge_gap = min(ink[0] - eb[0], ink[1] - eb[1], eb[2] - ink[2], OL.minus_y_distance(ink))
        h, th = mm(d.GetTextHeight()), mm(d.GetTextThickness())
        row = {"line": n, "what": kind, "characters": len(s), "layer": d.GetLayerName(), "mirrored": bool(d.IsMirrored()),
               "height_mm": round(h, 3), "stroke_mm": round(th, 3), "angle_deg": round(d.GetTextAngle().AsDegrees(), 1),
               "ink_box_mm": [round(v, 3) for v in ink], "gap_to_nearest_pad_mm": round(pad_gap, 3), "nearest_pad": pad_name,
               "gap_to_nearest_via_pad_mm": round(via_gap, 3), "nearest_via": via_at, "vias_under_the_text": int(under),
               "gap_to_other_silkscreen_mm": None if silk_gap is None else round(silk_gap, 3),
               "gap_to_board_edge_mm": round(edge_gap, 3),
               "under_cable_tie_band": bool(ink[0] < SILK.TIE_BAND[1] and ink[2] > SILK.TIE_BAND[0])}
        rows.append(row)
        if lay not in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            problems.append(f"line {n}: on {d.GetLayerName()}, not on a silkscreen layer")
        if bool(d.IsMirrored()) != back:
            problems.append(f"line {n}: mirrored = {bool(d.IsMirrored())} on {d.GetLayerName()}")
        if h < 1.0 - 1e-6 or th < 0.15 - 1e-6:
            problems.append(f"line {n}: {h} mm high / {th} mm stroke, below 1.0 / 0.15 mm")
        if pad_gap < 0.15:
            problems.append(f"line {n}: {pad_gap} mm to pad {pad_name}, below 0.15 mm")
        if under:
            problems.append(f"line {n}: {under} via(s) under the strokes")
        if silk_gap is not None and silk_gap < 0.15:
            problems.append(f"line {n}: {silk_gap} mm to another silkscreen item, below 0.15 mm")
        if edge_gap < 0.3:
            problems.append(f"line {n}: {edge_gap} mm to the board edge, below 0.3 mm")
    block = None
    if len(rows) == 2:
        (d1, d2), (r1, r2) = (found[0][3], found[1][3]), rows
        if d1.GetLayer() != d2.GetLayer() or r1["angle_deg"] != r2["angle_deg"]:
            problems.append("the two lines differ in layer or angle")
        else:
            run, down = SILK.id_directions(board, d1.GetLayer(), r1["height_mm"], r1["stroke_mm"], r1["angle_deg"])
            a, b = r1["ink_box_mm"], r2["ink_box_mm"]

            def start(k):  # coordinate of the edge the text starts at, along the reading direction
                lo, hi = (k[0], k[2]) if run[0] else (k[1], k[3])
                return lo if run[0] + run[1] > 0 else hi

            across = ((b[0] + b[2]) / 2 - (a[0] + a[2]) / 2) * down[0] + ((b[1] + b[3]) / 2 - (a[1] + a[3]) / 2) * down[1]
            block = {"reads_towards": list(run), "line_2_towards": list(down),
                     "line_2_below_line_1": bool(across > 0), "start_edges_differ_mm": round(abs(start(a) - start(b)), 3),
                     "gap_between_the_lines_mm": round(_box_gap(a, b), 3)}
            if not block["line_2_below_line_1"] or block["start_edges_differ_mm"] > 0.05:
                problems.append("line 2 is not below line 1 with the same start edge")
    return {"lines": rows, "block": block, "problems": problems, "ok": not problems and len(rows) == len(SILK.ID_LINES),
            "limits": "F: https://jlcpcb.com/capabilities/pcb-capabilities (Legend, read 2026-11-11): line width >= "
                      "0.15 mm, text height >= 1.0 mm, pad to silkscreen 0.15 mm; edge 0.3 mm = board rule; no via "
                      "under a stroke (the vias are tented; an ink break over the hole is avoided anyway)",
            "under_cable_tie_band_is": "reported, not a criterion (avoided if possible)"}


def pin_order_labels(board, fps) -> dict:
    """esp-user-fix step (verification finding V1: the bottom label '1S BAT +-' showed '+' on the side of the GND
    pad): every connector name that carries pin glyphs, measured on the board without silk.pin_name. The reading
    direction is taken from the label's own strokes (first stroke -> last stroke of its effective text shape), the
    pad order from the pad positions and nets. A label printed along a single pin row must list the glyphs in the
    order the pads lie along the reading direction (seen from either side: both vectors flip together); any other
    label must list them from pin 1."""
    other = set(SILK.LABELS.values())
    rows, problems = [], []
    for ref in SILK.PIN_LABELLED:
        if ref not in fps:
            continue
        label = SILK.LABELS[ref]
        hits = [d for d in board.GetDrawings() if d.GetClass() == "PCB_TEXT"
                and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS)
                and d.GetText().startswith(label + " ") and d.GetText() not in other]
        if len(hits) != 1:
            problems.append(f"{ref}: {len(hits)} pin-ordered name texts on the silkscreen (expected 1)")
            continue
        d = hits[0]
        pads = sorted((p for p in fps[ref].Pads() if p.GetNumber().isdigit()), key=lambda p: int(p.GetNumber()))
        gl = [SILK.pin_glyph(p.GetNetname(), ref) or "nc" for p in pads]
        sep = "" if all(len(g) == 1 for g in gl) else " "
        shape = d.GetEffectiveTextShape()  # kept in a name: the sub-shapes are only valid while it lives
        subs = shape.GetSubshapes()
        a, b = subs[0].BBox().GetCenter(), subs[len(subs) - 1].BBox().GetCenter()
        rx, ry = mm(b.x - a.x), mm(b.y - a.y)
        n = math.hypot(rx, ry)
        xs = [mm(p.GetPosition().x) for p in pads]
        ys = [mm(p.GetPosition().y) for p in pads]
        single_row = len(pads) > 1 and (len({round(v, 4) for v in xs}) == 1 or len({round(v, 4) for v in ys}) == 1)
        dx, dy = xs[-1] - xs[0], ys[-1] - ys[0]
        m = math.hypot(dx, dy)
        cos = (rx * dx + ry * dy) / (n * m) if single_row and n > 0 and m > 0 else 0.0
        row = {"ref": ref, "text": d.GetText(), "layer": d.GetLayerName(), "mirrored": bool(d.IsMirrored()),
               "glyphs_from_pin_1": sep.join(gl),
               "pad_1": {"net": pads[0].GetNetname(), "at": [round(xs[0], 3), round(ys[0], 3)]},
               "pad_last": {"number": pads[-1].GetNumber(), "net": pads[-1].GetNetname(),
                            "at": [round(xs[-1], 3), round(ys[-1], 3)]},
               "reading_direction_top_view": [round(rx / n, 2), round(ry / n, 2)] if n else None}
        if single_row and abs(cos) > 0.9:
            along = sorted(range(len(pads)), key=lambda i: xs[i] * rx + ys[i] * ry)
            want = sep.join(gl[i] for i in along)
            row.update(relation="along the pin row",
                       glyph_order="as the pads lie (pin 1 first)" if cos > 0 else "as the pads lie (last pin first)",
                       first_glyph_is_pad=pads[along[0]].GetNumber(), last_glyph_is_pad=pads[along[-1]].GetNumber())
        elif abs(cos) < 0.1:
            want = sep.join(gl)
            row.update(relation="across the pin row" if single_row else "not a single pin row", glyph_order="from pin 1")
        else:
            problems.append(f"{ref}: label neither along nor across the pin row (cos {cos:.2f})")
            continue
        got = d.GetText()[len(label) + 1:]
        row["ok"] = got == want
        rows.append(row)
        if got != want:
            problems.append(f"{ref}: label '{d.GetText()}' lists '{got}', the pads lie '{want}' ({row['relation']})")
    return {"ok": not problems, "problems": problems, "labels": rows,
            "method": "reading direction = first stroke -> last stroke of the label's effective text shape (pcbnew); "
                      "pad order = pad positions projected on it; glyphs from the pad nets (silk.pin_glyph)"}


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
    ids = id_lines(board)
    pol = pin_order_labels(board, fps)
    ok = not bad and bool(rev) and all(names.values()) and all(refs.values()) and ids["ok"] and pol["ok"]
    return {"result": PASS if ok else FAIL, "label": "M", "identification_lines": ids, "pin_order_labels": pol,
            "marks": rows, "marks_not_ok": bad,
            "extra_marks_placed": sum(1 for r in rows if r["extra_mark"]),
            "library_mark_only": [r["ref"] for r in rows if not r["extra_mark"]],
            "revision_text": rev, "pin_order_names": names, "boost_block_designators": refs,
            "criterion": "every listed footprint keeps its library F.SilkS polarity / pin-1 graphics with none of them "
                         "under another part's body (F.Fab box) or on a pad (extra '+', 'K' or dot marks are counted "
                         "separately); the text 'rev 3' exists; J1 / J2 / J5 / J7 carry a name with the pin order; "
                         "every name with pin glyphs lists them as the pads lie when it is printed along its pin "
                         "row (e.g. the battery connector: '+' on the side of the '+' pad), else from pin 1; "
                         "U8 / U9 / L5 designators exist on a silkscreen layer; the two identification lines "
                         "(repository address, contact e-mail) exist once each, >= 1.0 mm high with >= 0.15 mm stroke, "
                         "mirrored on the bottom, >= 0.15 mm from every pad and silkscreen item, on no via, >= 0.3 mm "
                         "inside the outline, line 2 below line 1 with the same start edge"}


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
        "hot_loop_top_gnd": check_hot_loop_top_gnd(board, fps),
        "input_cap_vin": check_input_cap_vin(board, fps),
        "input_cap_l5": check_input_cap_l5(board, fps),
        "fb_clear_of_caps": check_fb_clear_of_caps(board, fps),
        "rail_edge_copper": check_rail_edges(board, fps),
        "sw_node": check_sw_node(board, fps),
        "crystal_distance": check_crystal_distance(board, fps),
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
           "results": results, "checks": checks, "information": information(board, fps)}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(res, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"result": res["result"], "results": results,
                      "battery_path_ohm": checks["battery_path"].get("copper_resistance_ohm_20C"),
                      "narrower_than_2mm_mm": checks["battery_path"].get("current_carrying_length_narrower_than_2mm"),
                      "XTAL1_worst_mm": checks["crystals"].get("XTAL1_worst_mm")}, indent=1))
    return 0 if res["result"] == PASS else 1


if __name__ == "__main__":
    sys.exit(main())
