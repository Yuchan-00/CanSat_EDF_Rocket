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
  * mated-plug envelopes of the fitted lead connectors vs the CAD bore, joint screws and motor-lead joint zone
    (kicad/work/cad_envelope.json from scripts/cad_envelope.py; review MF-1..MF-7)
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
BOARD = ROOT / "kicad" / "vtol_carrier.kicad_pcb"
PLACEMENT = ROOT / "kicad" / "placement.json"
CAD = ROOT / "kicad" / "work" / "cad_envelope.json"
OUT = ROOT / "out" / "reports" / "board_metrics.json"
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
                  (f"{block}_XTAL2", m["xtal"], "3", m["cx2"], "1", "crystal -> XTAL2 load cap"),
                  (f"{block}_XTAL1", m["xtal"], "1", m["cx1"], "1", "crystal -> XTAL1 load cap")]
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
            for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
                if z.IsOnLayer(lay):
                    fills[lay] = z.GetFilledPolysList(lay)
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
VERT_PLUGS = {"J14": 11.0, "J17": 11.0, "J7": 9.0}


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
    out = {}
    for c, upin in (("C43", "3"), ("C44", "2")):
        cp = [p for p in fps[c].Pads() if p.GetNumber() == "1"][0]
        up = [p for p in fps["U6"].Pads() if p.GetNumber() == upin][0]
        out[f"{c}.1 -> U6.{upin}"] = round(math.hypot(mm(cp.GetPosition().x - up.GetPosition().x), mm(cp.GetPosition().y - up.GetPosition().y)), 2)
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
        "power_and_servo_nets": widths(board, ["SERVO_5V", "GND_PWR", "+5V", "+3V3", "VBAT_LOGIC", "VBAT_LOGIC_SW", "VBAT_LOGIC_RAW"]),
        "hole_to_hole": hole_to_hole(board),
        "silk": silk(board),
        "u6_cap_distance_mm": u6_caps(fps),
        "plug_fit": plug_fit(board, fps, cad),
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8", newline="\n")
    brief = {k: v for k, v in res.items() if k not in ("cluster_paths", "plug_fit")}
    brief["cluster_paths_max_mm"] = max((r["copper_path_mm"] or 99) for r in cp)
    brief["plug_fit"] = [{k: r[k] for k in ("ref", "gap_to_bore", "ok") if k in r} | {"jz": r.get("gap_to_motor_joint_zone")} for r in res["plug_fit"]]
    print(json.dumps(brief, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
