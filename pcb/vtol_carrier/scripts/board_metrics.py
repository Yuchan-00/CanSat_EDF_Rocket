"""Scripted measurements of the routed board (run with KiCad's python: needs pcbnew). Writes out/reports/board_metrics.json.

Measures (M = from the board file; plug sizes are A):
  * cluster copper paths: MCU supply / crystal pin -> its 100 nF / crystal / load-cap pad along the tracks of the net
    (Dijkstra over track segments; pads joined to the track ends they contain) - review F1 / F2
  * crystal nets: track length and via count per XTAL net
  * GND: vias (all / stitching), filled polygons per layer, share of non-GND track length with GND fill on the other layer
  * track widths of the power / servo nets, via sizes on the servo nets
  * smallest drill-edge distance between holes of different nets (hole_to_hole rule 0.5 mm)
  * silkscreen texts on F.SilkS / B.SilkS
  * AMS1117 U6 -> input / output capacitor pad distances
  * ESP-12F (U7, footprint RF_Module:ESP-12E): pin 8 (VCC) -> 100 nF copper path, its GND via, bulk capacitor
    distances; copper / pads / pours inside the antenna rule area on all copper layers (spec esp12e.antenna)
  * minimum track width per power / servo net (spec layout_rules: logic-battery path >= 1.0 mm, servo >= 1.5 mm)
  * mated-plug envelopes of the fitted lead connectors vs the CAD bore, joint screws and motor-lead joint zone
    (kicad/work/cad_envelope.json from scripts/cad_envelope.py; review MF-1..MF-7)
  rev 2c (rev-2b independent review):
  * net_clusters: for EVERY net, the pads grouped by real copper connectivity (pcbnew, zones filled) - a net in more
    than one cluster is open even where DRC shows a zone island only (review E-B2: three MCU GND pins + their caps sat
    on F.Cu GND islands while the pin -> cap path check said 'OK')
  * vias_touching_pads: vias whose disk touches a pad of an outer layer (review PF-M3)
  * silk_over_bodies: F.SilkS texts over another part's body (F.Fab box) or a reserved module body (review PF-M2)
  * supply_pads: zone connection / attached track width of the pads that feed a supply (review PF-M1)
  * power_narrow_outside_pad_entry: power / servo segments below the class width that are not just the last 0.5 mm
    into a pad (review PF-M6)
Nothing here edits the design.
"""

from __future__ import annotations

import collections
import heapq
import json
import math
import sys
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
import os  # noqa: E402

#: CARRIER_BOARD / CARRIER_METRICS_OUT: measure a scratch copy (experiments); default = the design board
BOARD = Path(os.environ["CARRIER_BOARD"]) if os.environ.get("CARRIER_BOARD") else ROOT / "kicad" / "vtol_carrier.kicad_pcb"
PLACEMENT = ROOT / "kicad" / "placement.json"
CAD = ROOT / "kicad" / "work" / "cad_envelope.json"
OUT = Path(os.environ["CARRIER_METRICS_OUT"]) if os.environ.get("CARRIER_METRICS_OUT") else ROOT / "out" / "reports" / "board_metrics.json"
mm = pcbnew.ToMM
CX, CY = 100.0, 100.0  # board centre (KiCad) = vehicle axis; spec (x, y) = (X - 100, 100 - Y)


def pad_of(fps, ref: str, num: str):
    return [p for p in fps[ref].Pads() if p.GetNumber() == num][0]


def net_graph(board, net: str):
    g = collections.defaultdict(list)
    ends = []
    for t in board.GetTracks():
        if t.GetNetname() != net:
            continue
        if t.GetClass() == "PCB_VIA":
            continue
        a = (t.GetStart().x, t.GetStart().y, t.GetLayer())
        b = (t.GetEnd().x, t.GetEnd().y, t.GetLayer())
        L = mm(t.GetLength())
        g[a].append((b, L))
        g[b].append((a, L))
        ends += [a, b]
    for v in board.GetTracks():  # vias join the layers at zero length
        if v.GetClass() == "PCB_VIA" and v.GetNetname() == net:
            p = v.GetPosition()
            fa, ba = (p.x, p.y, pcbnew.F_Cu), (p.x, p.y, pcbnew.B_Cu)
            g[fa].append((ba, 0.0))
            g[ba].append((fa, 0.0))
    return g, ends


def path_len(board, net: str, pad_a, pad_b) -> float | None:
    g, ends = net_graph(board, net)
    src = ("A",)
    dst = ("B",)
    for node in set(ends):
        p = pcbnew.VECTOR2I(node[0], node[1])
        for pad, tag in ((pad_a, src), (pad_b, dst)):
            if pad.IsOnLayer(node[2]) and pad.HitTest(p):
                d = math.hypot(mm(node[0] - pad.GetPosition().x), mm(node[1] - pad.GetPosition().y))
                g[tag].append((node, d))
                g[node].append((tag, d))
    # a segment that runs THROUGH a pad (no end inside it) also connects it: both ends at their distance to the
    # pad centre (rev 2c: the XTAL1 pre-route passes through crystal pad 1 on its way to the load cap)
    for t in board.GetTracks():
        if t.GetNetname() != net or t.GetClass() == "PCB_VIA":
            continue
        for pad, tag in ((pad_a, src), (pad_b, dst)):
            if pad.IsOnLayer(t.GetLayer()) and pad.GetEffectiveShape(t.GetLayer()).Collide(t.GetEffectiveShape(t.GetLayer()), 0):
                for q in (t.GetStart(), t.GetEnd()):
                    node = (q.x, q.y, t.GetLayer())
                    d = math.hypot(mm(q.x - pad.GetPosition().x), mm(q.y - pad.GetPosition().y))
                    g[tag].append((node, d))
                    g[node].append((tag, d))
    dist = {src: 0.0}
    pq = [(0.0, 0, src)]
    k = 0
    while pq:
        d, _, u = heapq.heappop(pq)
        if u == dst:
            return round(d, 2)
        if d > dist.get(u, 1e18):
            continue
        for v, L in g[u]:
            nd = d + L
            if nd < dist.get(v, 1e18):
                dist[v] = nd
                k += 1
                heapq.heappush(pq, (nd, k, v))
    return None


def cluster_paths(board, fps, clusters) -> list[dict]:
    rows = []
    for block, m in sorted(clusters.items()):
        u = m["mcu"]
        checks = [("+5V", u, "21", m["dec21"], "1", "VCC pin 21 -> 100 nF"), ("GND", u, "22", m["dec21"], "2", "GND pin 22 -> 100 nF"),
                  ("+5V", u, "52", m["dec52"], "1", "VCC pin 52 -> 100 nF"), ("GND", u, "53", m["dec52"], "2", "GND pin 53 -> 100 nF"),
                  (f"{block}_AVCC", u, "64", m["avcc"], "1", "AVCC pin 64 -> 100 nF"), ("GND", u, "63", m["avcc"], "2", "GND pin 63 -> 100 nF"),
                  (f"{block}_XTAL2", u, "23", m["xtal"], "3", "XTAL2 pin 23 -> crystal"), (f"{block}_XTAL1", u, "24", m["xtal"], "1", "XTAL1 pin 24 -> crystal"),
                  (f"{block}_AREF", u, "62", m.get("aref", "-"), "1", "AREF pin 62 -> 100 nF"),
                  (f"{block}_AVCC", m["avcc"], "1", m.get("lavcc", "-"), "2", "AVCC 100 nF -> 10 uH"),
                  (f"{block}_XTAL2", m["xtal"], "3", m["cx2"], "1", "crystal -> XTAL2 load cap"),
                  (f"{block}_XTAL1", m["xtal"], "1", m["cx1"], "1", "crystal -> XTAL1 load cap")]
        checks = [c for c in checks if c[1] in m.values() or c[1] == u]
        checks = [c for c in checks if c[3] in m.values()]
        for net, ra, na, rb, nb, what in checks:
            pa, pb = pad_of(fps, ra, na), pad_of(fps, rb, nb)
            straight = math.hypot(mm(pa.GetPosition().x - pb.GetPosition().x), mm(pa.GetPosition().y - pb.GetPosition().y))
            L = path_len(board, net, pa, pb)
            rows.append({"block": block, "what": what, "from": f"{ra}.{na}", "to": f"{rb}.{nb}", "net": net,
                         "straight_mm": round(straight, 2), "copper_path_mm": L, "ok_le_3mm": L is not None and L <= 3.0})
    return rows


def xtal_nets(board) -> dict:
    out = {}
    for t in board.GetTracks():
        n = t.GetNetname()
        if "_XTAL" not in n:
            continue
        d = out.setdefault(n, {"track_mm": 0.0, "vias": 0, "layers": set()})
        if t.GetClass() == "PCB_VIA":
            d["vias"] += 1
        else:
            d["track_mm"] += mm(t.GetLength())
            d["layers"].add(t.GetLayerName())
    return {k: {"track_mm": round(v["track_mm"], 2), "vias": v["vias"], "layers": sorted(v["layers"])} for k, v in sorted(out.items())}


def gnd_metrics(board) -> dict:
    fills = {}
    for z in board.Zones():
        if z.GetNetname() == "GND" and not z.GetIsRuleArea():
            for lay in (pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.In1_Cu):
                if z.IsOnLayer(lay):
                    fills[lay] = z.GetFilledPolysList(lay)
    if pcbnew.In1_Cu in fills:
        return gnd_metrics_4(board, fills)
    vias = [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA" and t.GetNetname() == "GND"]
    tot = cov = 0.0
    per_net = collections.defaultdict(lambda: [0.0, 0.0])
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA" or t.GetNetname() == "GND":
            continue
        other = pcbnew.B_Cu if t.GetLayer() == pcbnew.F_Cu else pcbnew.F_Cu
        a, b = t.GetStart(), t.GetEnd()
        L = mm(t.GetLength())
        n = max(1, int(L / 0.25))
        for i in range(n):
            f = (i + 0.5) / n
            p = pcbnew.VECTOR2I(int(a.x + (b.x - a.x) * f), int(a.y + (b.y - a.y) * f))
            inside = fills[other].Contains(p)
            tot += L / n
            per_net[t.GetNetname()][0] += L / n
            if inside:
                cov += L / n
                per_net[t.GetNetname()][1] += L / n
    gnd_tracks = [t for t in board.GetTracks() if t.GetClass() != "PCB_VIA" and t.GetNetname() == "GND"]
    nl = {n: round(100 * c / l, 1) for n, (l, c) in per_net.items() if n.startswith("NLINK")}
    return {"gnd_vias": len(vias), "gnd_fill_polygons": {"F.Cu": fills[pcbnew.F_Cu].OutlineCount(), "B.Cu": fills[pcbnew.B_Cu].OutlineCount()},
            "gnd_fill_area_mm2": {"F.Cu": round(fills[pcbnew.F_Cu].Area() / 1e12, 1), "B.Cu": round(fills[pcbnew.B_Cu].Area() / 1e12, 1)},
            "gnd_track_mm": round(sum(mm(t.GetLength()) for t in gnd_tracks), 1),
            "gnd_track_widths": dict(collections.Counter(round(mm(t.GetWidth()), 3) for t in gnd_tracks)),
            "signal_length_with_gnd_fill_on_other_layer_pct": round(100 * cov / tot, 1) if tot else None,
            "nlink_pct": nl}


def gnd_metrics_4(board, fills) -> dict:
    """4 layers: F.Cu signals reference the In1.Cu GND plane (0.21 mm below, A stackup); B.Cu signals reference In2.Cu
    (+5V plane). Share of track length with the adjacent plane filled under it, per layer."""
    p5 = {}
    for z in board.Zones():
        if z.GetNetname() == "+5V" and not z.GetIsRuleArea() and z.IsOnLayer(pcbnew.In2_Cu):
            p5 = z.GetFilledPolysList(pcbnew.In2_Cu)
    ref = {pcbnew.F_Cu: fills[pcbnew.In1_Cu], pcbnew.B_Cu: p5}
    tot = collections.Counter()
    cov = collections.Counter()
    nl = collections.defaultdict(lambda: [0.0, 0.0])
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA" or t.GetLayer() not in ref:
            continue
        a, b = t.GetStart(), t.GetEnd()
        L = mm(t.GetLength())
        n = max(1, int(L / 0.25))
        for i in range(n):
            f = (i + 0.5) / n
            p = pcbnew.VECTOR2I(int(a.x + (b.x - a.x) * f), int(a.y + (b.y - a.y) * f))
            tot[t.GetLayerName()] += L / n
            inside = bool(ref[t.GetLayer()]) and ref[t.GetLayer()].Contains(p)
            if inside:
                cov[t.GetLayerName()] += L / n
            if t.GetNetname().startswith("NLINK"):
                nl[t.GetNetname()][0] += L / n
                nl[t.GetNetname()][1] += L / n if (t.GetLayer() == pcbnew.F_Cu and inside) else 0.0
    vias = [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA" and t.GetNetname() == "GND"]
    gnd_tracks = [t for t in board.GetTracks() if t.GetClass() != "PCB_VIA" and t.GetNetname() == "GND"]
    return {"copper_layers": 4, "gnd_vias": len(vias),
            "gnd_fill_polygons": {board.GetLayerName(k): v.OutlineCount() for k, v in fills.items()},
            "gnd_fill_area_mm2": {board.GetLayerName(k): round(v.Area() / 1e12, 1) for k, v in fills.items()},
            "plus5v_in2_fill_area_mm2": round(p5.Area() / 1e12, 1) if p5 else 0.0,
            "plus5v_in2_fill_polygons": p5.OutlineCount() if p5 else 0,
            "gnd_track_mm": round(sum(mm(t.GetLength()) for t in gnd_tracks), 1),
            "gnd_track_widths": dict(collections.Counter(round(mm(t.GetWidth()), 3) for t in gnd_tracks)),
            "track_length_over_adjacent_plane_pct": {k: round(100 * cov[k] / tot[k], 1) for k in tot if tot[k]},
            "track_mm_per_layer": {k: round(v, 1) for k, v in tot.items()},
            "nlink_pct_on_F_over_In1_GND": {k: round(100 * c / l, 1) for k, (l, c) in nl.items() if l}}


def widths(board, nets) -> dict:
    out = {}
    for n in nets:
        c = collections.Counter()
        vs = collections.Counter()
        L = 0.0
        for t in board.GetTracks():
            if t.GetNetname() != n:
                continue
            if t.GetClass() == "PCB_VIA":
                vs[f"{mm(t.GetWidth(pcbnew.F_Cu))}/{mm(t.GetDrillValue())}"] += 1
            else:
                c[round(mm(t.GetWidth()), 3)] += round(mm(t.GetLength()), 2)
                L += mm(t.GetLength())
        out[n] = {"length_by_width_mm": {str(k): round(v, 1) for k, v in sorted(c.items())}, "vias": dict(vs), "track_mm": round(L, 1)}
    return out


def hole_to_hole(board) -> dict:
    hs = []
    for v in board.GetTracks():
        if v.GetClass() == "PCB_VIA":
            hs.append((mm(v.GetPosition().x), mm(v.GetPosition().y), mm(v.GetDrillValue()) / 2, v.GetNetname()))
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.HasHole():
                ds = pad.GetDrillSize()
                hs.append((mm(pad.GetPosition().x), mm(pad.GetPosition().y), mm(max(ds.x, ds.y)) / 2, pad.GetNetname()))
    best = (1e9, None)
    for i, a in enumerate(hs):
        for b in hs[i + 1:]:
            if a[3] == b[3]:
                continue
            d = math.hypot(a[0] - b[0], a[1] - b[1]) - a[2] - b[2]
            if d < best[0]:
                best = (d, [a, b])
    return {"min_drill_edge_mm_different_nets": round(best[0], 3), "pair": best[1], "rule_mm": 0.5}


def silk(board) -> dict:
    c = collections.Counter()
    for d in board.GetDrawings():
        if d.GetClass() == "PCB_TEXT":
            c[d.GetLayerName()] += 1
    refs = collections.Counter(fp.Reference().GetLayerName() for fp in board.GetFootprints())
    return {"board_texts": dict(c), "reference_layers": dict(refs)}


# ---------------------------------------------------------------- plug fit
#: plug envelopes (A: typical housings, measure the bought ones). Right-angle headers: plug from the header face
#: (PinHeader_1x0N_P2.54mm_Horizontal: plastic body x 1.5..4.04 in the library file) outward, length L, width W
#: across the pin row, z from board top -0.3 to +3.0. Vertical JST: mated height H above the board top.
RA_PLUGS = {"J8": (7.8, 14.5), "J9": (7.8, 14.5), "J10": (7.8, 14.5), "J11": (7.8, 14.5), "J12": (7.8, 14.5),
            "J16": (5.1, 14.0), "J18": (10.2, 14.0)}
#: vertical connectors, mated plug height above the board (A): Molex 5264 housing + lead bend, JST-PH, JST-GH
VERT_PLUGS = {"J14": 11.0, "J15": 11.0, "J17": 11.0, "J7": 9.0}  # J15 JST-PH since rev 2c (review MF2-4)


def plug_fit(board, fps, cad) -> list[dict]:
    R = cad["spigot_bore_r"]
    zt = cad["board_top_z"]
    jz = cad["motor_joint_zone"]
    th0 = math.radians(jz["theta_deg"])
    rows = []

    def spec(x, y):
        return x - CX, CY - y

    def box_gap(a, b):
        return math.sqrt(sum(max(a[k][0] - b[k][1], 0.0, b[k][0] - a[k][1]) ** 2 for k in range(3)))

    # joint zone box in its own frame (u radial along theta, v tangential); theta 0 -> x = u, y = v
    jzb = ((jz["u"][0], jz["u"][1]), (-jz["half_width"], jz["half_width"]), tuple(jz["z"]))

    def seg_box_dist(a, b, box, n=400):
        best = 1e9
        for i in range(n + 1):
            t = i / n
            p = [a[k] + (b[k] - a[k]) * t for k in range(3)]
            best = min(best, math.sqrt(sum(max(box[k][0] - p[k], 0.0, p[k] - box[k][1]) ** 2 for k in range(3))))
        return best

    for ref, (W, L) in RA_PLUGS.items():
        fp = fps[ref]
        if fp.IsDNP():
            continue
        pads = sorted(fp.Pads(), key=lambda p: int(p.GetNumber()))
        rot = fp.GetOrientationDegrees()
        ux, uy = math.cos(math.radians(rot)), -math.sin(math.radians(rot))  # footprint local +x on the board
        p0 = fp.GetPosition()
        fx, fy = mm(p0.x) + 4.04 * ux, mm(p0.y) + 4.04 * uy
        cx = sum(mm(p.GetPosition().x) for p in pads) / len(pads) + 4.04 * ux
        cy = sum(mm(p.GetPosition().y) for p in pads) / len(pads) + 4.04 * uy
        ex, ey = cx + L * ux, cy + L * uy
        vx, vy = -uy, ux
        corners = [spec(px + s * W / 2 * vx, py + s * W / 2 * vy) for px, py in ((cx, cy), (ex, ey)) for s in (-1, 1)]
        xs, ys = [c[0] for c in corners], [c[1] for c in corners]
        box = ((min(xs), max(xs)), (min(ys), max(ys)), (zt - 0.3, zt + 3.0))
        r_out = max(math.hypot(x, y) for x, y in corners)
        screws = {}
        for th in cad["joint_thetas_deg"]:
            c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
            g52 = seg_box_dist((52 * c, 52 * s, cad["joint_screw_z"]), (R * c, R * s, cad["joint_screw_z"]), box) - 1.5
            if g52 < 3.0:
                g54 = seg_box_dist((54 * c, 54 * s, cad["joint_screw_z"]), (R * c, R * s, cad["joint_screw_z"]), box) - 1.5
                screws[str(int(th))] = {"gap_tip_r52": round(g52, 2), "gap_tip_r54": round(g54, 2)}
        jb = box if abs(th0) < 1e-9 else box  # joint zone at theta 0 (the only value in the CAD)
        rows.append({"ref": ref, "kind": "right-angle", "plug_box_spec_xy": [[round(v, 2) for v in a] for a in box[:2]],
                     "outer_corner_r": round(r_out, 2), "gap_to_bore": round(R - r_out, 2),
                     "gap_to_motor_joint_zone": round(box_gap(jb, jzb), 2), "joint_zone_min_clear": jz["min_clear"],
                     "joint_screws_within_3mm": screws,
                     "ok": (R - r_out) > 0.5 and box_gap(jb, jzb) >= jz["min_clear"]})
    for ref, Hm in VERT_PLUGS.items():
        fp = fps[ref]
        bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
        x0, y0 = spec(mm(bb.GetLeft()), mm(bb.GetBottom()))
        x1, y1 = spec(mm(bb.GetRight()), mm(bb.GetTop()))
        r_out = max(math.hypot(x, y) for x in (x0, x1) for y in (y0, y1))
        rows.append({"ref": ref, "kind": "vertical (lead from above)", "courtyard_spec_xy": [[round(x0, 2), round(x1, 2)], [round(y0, 2), round(y1, 2)]],
                     "mated_height_A_mm": Hm, "height_limit_mm": 13.4, "outer_corner_r": round(r_out, 2),
                     "gap_to_bore": round(R - r_out, 2), "ok": Hm <= 13.4 and r_out < R})
    return rows


def u6_caps(fps) -> dict:
    """AMS1117: C44 (10 uF) at VI pin 3, C45 (22 uF tantalum) at VO pin 2 (spec parts)."""
    out = {}
    for c, upin in (("C44", "3"), ("C45", "2")):
        cp = [p for p in fps[c].Pads() if p.GetNumber() == "1"][0]
        up = [p for p in fps["U6"].Pads() if p.GetNumber() == upin][0]
        out[f"{c}.1 -> U6.{upin}"] = round(math.hypot(mm(cp.GetPosition().x - up.GetPosition().x), mm(cp.GetPosition().y - up.GetPosition().y)), 2)
    return out


def esp_metrics(board, fps, rule_area) -> dict:
    """ESP-12F (U7) VCC decoupling and the antenna rule area (KiCad coords x0, y0, x1, y1)."""
    p8 = pad_of(fps, "U7", "8")
    c54 = pad_of(fps, "C54", "1")
    out = {"pin8_to_C54_copper_mm": path_len(board, "+3V3", p8, c54)}
    for ref in ("C53", "C52"):
        pp = pad_of(fps, ref, "1")
        out[f"pin8_to_{ref}_pad1_straight_mm"] = round(math.hypot(mm(pp.GetPosition().x - p8.GetPosition().x), mm(pp.GetPosition().y - p8.GetPosition().y)), 2)
    g = pad_of(fps, "C54", "2")
    vias = [v for v in board.GetTracks() if v.GetClass() == "PCB_VIA" and v.GetNetname() == "GND"]
    out["C54_gnd_pad_to_nearest_gnd_via_mm"] = round(min(math.hypot(mm(v.GetPosition().x - g.GetPosition().x), mm(v.GetPosition().y - g.GetPosition().y)) for v in vias), 2)
    x0, y0, x1, y1 = rule_area
    rect = pcbnew.SHAPE_POLY_SET()
    rect.NewOutline()
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        rect.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
    inside = []
    for t in board.GetTracks():
        bb = t.GetBoundingBox()
        if mm(bb.GetRight()) > x0 and mm(bb.GetLeft()) < x1 and mm(bb.GetBottom()) > y0 and mm(bb.GetTop()) < y1:
            inside.append(f"{t.GetClass()} {t.GetNetname()} at ({mm(t.GetPosition().x):.2f}, {mm(t.GetPosition().y):.2f})")
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            bb = pad.GetBoundingBox()
            if mm(bb.GetRight()) > x0 and mm(bb.GetLeft()) < x1 and mm(bb.GetBottom()) > y0 and mm(bb.GetTop()) < y1:
                inside.append(f"pad {fp.GetReference()}.{pad.GetNumber()}")
    fill_area = 0.0
    for z in board.Zones():
        if z.GetIsRuleArea():
            continue
        for lay in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
            if z.IsOnLayer(lay):
                f = pcbnew.SHAPE_POLY_SET(z.GetFilledPolysList(lay))
                f.BooleanIntersection(rect)
                fill_area += f.Area() / 1e12
    out["rule_area_kicad"] = [x0, y0, x1, y1]
    out["copper_items_inside_rule_area"] = inside
    out["pour_area_inside_rule_area_mm2"] = round(fill_area, 3)
    u7 = fps["U7"].GetPosition()
    out["module_antenna_end_y_kicad"] = round(mm(u7.y) - 12.12, 3)
    out["antenna_end_to_board_edge_mm"] = round(mm(u7.y) - 12.12 - 65.0, 3)
    out["ok"] = not inside and fill_area < 1e-6
    return out


def net_clusters(board) -> dict:
    """Pads of every net grouped by copper connectivity (tracks, vias, filled zones)."""
    conn = board.GetConnectivity()
    by_net = collections.defaultdict(list)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            n = pad.GetNetname()
            if n and not n.startswith("unconnected-") and pad.GetNumber():
                by_net[n].append(pad)
    split = {}
    for n, pads in sorted(by_net.items()):
        if len(pads) < 2:
            continue
        seen, clusters = set(), []
        for p in pads:
            key = (p.GetParentFootprint().GetReference(), p.GetNumber(), p.GetPosition().x, p.GetPosition().y)
            if key in seen:
                continue
            cl = {key}
            for it in conn.GetConnectedItems(p, False):
                if it.GetClass() == "PAD" and it.GetNetname() == n:
                    cl.add((it.GetParentFootprint().GetReference(), it.GetNumber(), it.GetPosition().x, it.GetPosition().y))
            seen |= cl
            clusters.append(sorted(f"{r}.{num}" for r, num, _x, _y in cl))
        if len(clusters) > 1:
            clusters.sort(key=len, reverse=True)
            split[n] = {"clusters": len(clusters), "main_size": len(clusters[0]), "isolated": clusters[1:]}
    return {"nets_checked": sum(1 for v in by_net.values() if len(v) > 1), "nets_split": split, "ok": not split}


def vias_touching_pads(board) -> dict:
    out = []
    pads = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
                if pad.IsOnLayer(lay) and not pad.HasHole():
                    pads.append((lay, pad, pad.GetBoundingBox()))
    for v in board.GetTracks():
        if v.GetClass() != "PCB_VIA":
            continue
        vp = v.GetPosition()
        for lay, pad, bb in pads:
            if not (bb.GetLeft() - pcbnew.FromMM(1) < vp.x < bb.GetRight() + pcbnew.FromMM(1)
                    and bb.GetTop() - pcbnew.FromMM(1) < vp.y < bb.GetBottom() + pcbnew.FromMM(1)):
                continue
            if pad.GetEffectiveShape(lay).Collide(v.GetEffectiveShape(lay), 0):
                out.append({"via_net": v.GetNetname(), "at": [round(mm(vp.x), 3), round(mm(vp.y), 3)],
                            "pad": f"{pad.GetParentFootprint().GetReference()}.{pad.GetNumber()}", "pad_net": pad.GetNetname(),
                            "via_centre_in_pad": pad.HitTest(vp)})
    return {"count": len(out), "items": out}


def silk_over_bodies(board) -> dict:
    fab = {}
    for fp in board.GetFootprints():
        boxes = [g.GetBoundingBox() for g in fp.GraphicalItems() if g.GetLayer() == pcbnew.F_Fab and g.GetClass() != "PCB_TEXT"]
        if boxes and "Horizontal" not in fp.GetFPID().GetLibItemName().wx_str():  # right-angle headers: bare pins
            fab[fp.GetReference()] = (min(mm(b.GetLeft()) for b in boxes), min(mm(b.GetTop()) for b in boxes),
                                      max(mm(b.GetRight()) for b in boxes), max(mm(b.GetBottom()) for b in boxes))
    bodies = [(mm(d.GetBoundingBox().GetLeft()), mm(d.GetBoundingBox().GetTop()), mm(d.GetBoundingBox().GetRight()),
               mm(d.GetBoundingBox().GetBottom())) for d in board.GetDrawings()
              if d.GetLayer() == pcbnew.Dwgs_User and d.GetClass() == "PCB_SHAPE" and d.GetShape() == pcbnew.SHAPE_T_RECT]
    texts = []
    for d in board.GetDrawings():
        if d.GetClass() == "PCB_TEXT" and d.GetLayer() == pcbnew.F_SilkS:
            texts.append((d.GetText(), d.GetBoundingBox(), None))
    for fp in board.GetFootprints():
        f = fp.Reference()
        if f.GetLayer() == pcbnew.F_SilkS and f.IsVisible():
            texts.append((f.GetText(), f.GetBoundingBox(), fp.GetReference()))

    def ov(a, b):
        return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]

    bad = []
    for txt, bb, own in texts:
        box = (mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom()))
        hit = [r for r, c in fab.items() if r != own and ov(box, c)]
        hb = [i for i, c in enumerate(bodies) if ov(box, c)]
        if hit or hb:
            bad.append({"text": txt, "over_part_bodies": hit, "over_module_or_band_rects": len(hb)})
    return {"front_texts": len(texts), "over_bodies": bad, "ok": not bad}


def supply_pads(board, fps) -> dict:
    out = {}
    # rev 3: the Pololu module socket J6 is gone; the battery path J14 -> Q10 -> J15 -> L5 and the boost block feed +5V
    for ref, num in (("J14", "1"), ("J14", "2"), ("Q10", "3"), ("Q10", "2"), ("J15", "1"), ("J15", "2"), ("L5", "1"), ("L5", "2"),
                     ("U8", "3"), ("U8", "4"), ("U8", "5"), ("U8", "6"), ("C57", "1"), ("C43", "1"), ("C41", "1"),
                     ("U6", "2"), ("U6", "3"), ("J12", "2"), ("J12", "3")):
        if ref not in fps:
            continue
        pad = pad_of(fps, ref, num)
        ws = [round(mm(t.GetWidth()), 3) for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"
              and t.GetNetCode() == pad.GetNetCode() and (pad.HitTest(t.GetStart()) or pad.HitTest(t.GetEnd()))]
        zc = pad.GetLocalZoneConnection()
        out[f"{ref}.{num}"] = {"net": pad.GetNetname(), "tracks_attached_mm": sorted(ws),
                               "zone_connection": {pcbnew.ZONE_CONNECTION_FULL: "solid", pcbnew.ZONE_CONNECTION_THERMAL: "thermal",
                                                   pcbnew.ZONE_CONNECTION_NONE: "none"}.get(zc, "inherited (THT thermal / SMD solid)")}
    return out


CLASS_W = {"SERVO_5V": 1.5, "GND_PWR": 1.5, "VBAT_LOGIC_RAW": 1.0, "VBAT_LOGIC": 1.0, "VBAT_LOGIC_SW": 1.0,
           "+5V": 0.5, "+3V3": 0.5, "GPS_VCC": 0.5}


def power_narrow(board) -> dict:
    """Segments below the class width, except those whose whole length lies within 0.5 mm of a pad of the same net
    (pad entry)."""
    pads = collections.defaultdict(list)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname() in CLASS_W:
                pads[pad.GetNetname()].append(pad)
    out = collections.defaultdict(lambda: {"segments": 0, "length_mm": 0.0, "min_mm": 99.0})
    for t in board.GetTracks():
        n = t.GetNetname()
        if t.GetClass() != "PCB_TRACK" or n not in CLASS_W or mm(t.GetWidth()) >= CLASS_W[n] - 1e-6:
            continue
        a, b = t.GetStart(), t.GetEnd()
        entry = False
        for pad in pads[n]:
            sh = pad.GetEffectiveShape(t.GetLayer()) if pad.IsOnLayer(t.GetLayer()) else None
            if sh is None:
                continue
            if sh.Collide(pcbnew.SHAPE_SEGMENT(a, b, 1), pcbnew.FromMM(0.0)) and mm(t.GetLength()) <= 0.5:
                entry = True
                break
        if entry:
            continue
        o = out[n]
        o["segments"] += 1
        o["length_mm"] = round(o["length_mm"] + mm(t.GetLength()), 2)
        o["min_mm"] = min(o["min_mm"], round(mm(t.GetWidth()), 3))
    return dict(out)


#: module top above the board top (A: header spacer + module + parts; rev-2b review MF2-12 estimates, measure)
MODULE_TOP_A = {"J1": 5.3, "J2": 5.9, "J5": 5.9}  # rev 3: no J6


def cad_round_envelopes(pdata: dict, plugs: list[dict], cad: dict) -> dict:
    """Boxes the CAD round should add to the carrier placeholder (vehicle x / y = spec frame, z = local height above
    the board top; review MF2-6): mated plugs (A sizes) and the reserved module bodies (A heights)."""
    out = []
    for r in plugs:
        if "plug_box_spec_xy" in r:
            out.append({"what": f"{r['ref']} mated right-angle plug (A)", "x": r["plug_box_spec_xy"][0],
                        "y": r["plug_box_spec_xy"][1], "z_above_board_top": [-0.3, 3.0]})
        else:
            out.append({"what": f"{r['ref']} mated vertical plug + lead bend (A)", "x": r["courtyard_spec_xy"][0],
                        "y": r["courtyard_spec_xy"][1], "z_above_board_top": [0.0, r["mated_height_A_mm"]]})
    for b in pdata["placement_checks"]["module_bodies"]:
        x0, y0, x1, y1 = b["rect"]
        out.append({"what": b["name"], "x": [round(x0 - CX, 2), round(x1 - CX, 2)], "y": [round(CY - y1, 2), round(CY - y0, 2)],
                    "z_above_board_top": [0.0, MODULE_TOP_A.get(b["ref"])]})
    return {"board_top_z_vehicle": cad.get("board_top_z"), "boxes": out,
            "note": "vehicle x / y (spec frame); add these boxes to the CAD carrier envelope (the CAD carries a bare 76 x 70 x 15 box)"}


def min_widths(board, nets) -> dict:
    out = {}
    for n in nets:
        ws = [round(mm(t.GetWidth()), 3) for t in board.GetTracks() if t.GetClass() == "PCB_TRACK" and t.GetNetname() == n]
        out[n] = {"min_mm": min(ws) if ws else None, "segments": len(ws)}
    return out


def main() -> int:
    board = pcbnew.LoadBoard(str(BOARD))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.BuildConnectivity()
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    pdata = json.loads(PLACEMENT.read_text(encoding="utf-8"))
    cad = json.loads(CAD.read_text(encoding="utf-8"))
    cp = cluster_paths(board, fps, pdata["clusters"])
    res = {
        "tool": "scripts/board_metrics.py (pcbnew, KiCad 10)",
        "cluster_paths": cp,
        "cluster_paths_all_le_3mm": all(r["ok_le_3mm"] for r in cp),
        "xtal_nets": xtal_nets(board),
        "gnd": gnd_metrics(board),
        "power_and_servo_nets": widths(board, ["SERVO_5V", "GND_PWR", "+5V", "+3V3", "VBAT_LOGIC", "VBAT_LOGIC_SW", "VBAT_LOGIC_RAW", "BOOST_SW"]),
        "hole_to_hole": hole_to_hole(board),
        "silk": silk(board),
        "u6_cap_distance_mm": u6_caps(fps),
        "esp12f": esp_metrics(board, fps, pdata["antenna_rule_area"]),
        "min_track_width_mm": min_widths(board, ["VBAT_LOGIC_RAW", "VBAT_LOGIC", "VBAT_LOGIC_SW", "BOOST_SW", "SERVO_5V", "GND_PWR", "+5V", "+3V3", "GPS_VCC"]),
        "plug_fit": plug_fit(board, fps, cad),
        "net_clusters": net_clusters(board),
        "vias_touching_pads": vias_touching_pads(board),
        "silk_over_bodies": silk_over_bodies(board),
        "supply_pads": supply_pads(board, fps),
        "power_narrow_outside_pad_entry": power_narrow(board),
    }
    res["cad_round_envelopes"] = cad_round_envelopes(pdata, res["plug_fit"], cad)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8", newline="\n")
    brief = {k: v for k, v in res.items() if k not in ("cluster_paths", "plug_fit")}
    brief["cluster_paths_max_mm"] = max((r["copper_path_mm"] or 99) for r in cp)
    brief["plug_fit"] = [{k: r[k] for k in ("ref", "gap_to_bore", "ok") if k in r} | {"jz": r.get("gap_to_motor_joint_zone")} for r in res["plug_fit"]]
    print(json.dumps(brief, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
