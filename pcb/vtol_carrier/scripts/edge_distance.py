"""Edge-distance table of the VTOL carrier (REV3_PLAN review OM-1) -> out/edge_distance.csv + out/reports/edge_distance.json.

Run with KiCad's python (pcbnew):
    <KiCad>/bin/python.exe scripts/edge_distance.py [board.kicad_pcb]
default board: kicad/vtol_carrier.kicad_pcb (the routed board); the place step runs it on
kicad/work/vtol_carrier_unrouted.kicad_pcb.

Invariant: this script only MEASURES (M: bounding boxes read from the board file with pcbnew). It never moves a part
and never decides that JLC accepts the board - that stays NOT_VERIFIED until JLC's DFM result after the upload.

* body  = bounding box of the footprint's F.Fab / B.Fab graphics without texts (courtyard, then pads, as a fallback)
* pads  = bounding box of all pads
* distance = smallest distance of that box to the outline x 62..138, y 65..135 (KiCad mm); negative = overhang
* "under 2.5 mm": JLC assembly terms clause 2.3 (component body >= 2.5 mm from the board edge) - the reason the order
  is a Standard PCBA order with 5 mm edge rails; "under 0.3 mm": the Economic rule of the plan.
* rail sides: JLC adds rails on two opposite sides. The pair is chosen here as the one that leaves fewer JLC-fitted
  bodies closer than 2.5 mm to an edge WITHOUT a rail (tie: the pair with the smaller distance). E: derived from the
  table, to be written in the order note; JLC may still decide otherwise.
"""
from __future__ import annotations

import csv
import hashlib
import io
import json
import sys
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
MM = 1e-6
OX0, OY0, OX1, OY1 = 62.0, 65.0, 138.0, 135.0  # outline centre lines (spec board 76 x 70 mm, centre 100 / 100)
SIDES = ("-x", "+y", "+x", "-y")  # spec frame: +y = KiCad up (y 65), -y = KiCad down (y 135)
LIMIT_BODY = 2.5
LIMIT_ECONOMIC = 0.3


def bbox_of(items) -> list[float] | None:
    x0 = y0 = 1e9
    x1 = y1 = -1e9
    n = 0
    for it in items:
        bb = it.GetBoundingBox()
        x0, y0 = min(x0, bb.GetLeft() * MM), min(y0, bb.GetTop() * MM)
        x1, y1 = max(x1, bb.GetRight() * MM), max(y1, bb.GetBottom() * MM)
        n += 1
    return [x0, y0, x1, y1] if n else None


def side_dist(bb: list[float]) -> dict[str, float]:
    return {"-x": bb[0] - OX0, "+y": bb[1] - OY0, "+x": OX1 - bb[2], "-y": OY1 - bb[3]}


def main() -> int:
    board_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "kicad" / "vtol_carrier.kicad_pcb"
    spec = json.loads((ROOT / "spec" / "carrier_spec.json").read_text(encoding="utf-8"))
    parts = {p["refdes"]: p for p in spec["parts"]}
    b = pcbnew.LoadBoard(str(board_path))
    eb = b.GetBoardEdgesBoundingBox()
    outline = [round(v * MM, 3) for v in (eb.GetLeft(), eb.GetTop(), eb.GetRight(), eb.GetBottom())]
    rows = []
    for fp in sorted(b.GetFootprints(), key=lambda f: (f.GetReference().rstrip("0123456789"), int("".join(c for c in f.GetReference() if c.isdigit()) or 0))):
        ref = fp.GetReference()
        back = fp.GetLayer() == pcbnew.B_Cu
        fab_layer, crt_layer = (pcbnew.B_Fab, pcbnew.B_CrtYd) if back else (pcbnew.F_Fab, pcbnew.F_CrtYd)
        fab = bbox_of([g for g in fp.GraphicalItems() if g.GetLayer() == fab_layer and g.GetClass() != "PCB_TEXT"])
        crt = bbox_of([g for g in fp.GraphicalItems() if g.GetLayer() == crt_layer])
        pads = bbox_of(list(fp.Pads()))
        body = fab or crt or pads
        if body is None:
            continue
        part = parts.get(ref, {})
        ds = side_dist(body)
        near = min(SIDES, key=lambda s: ds[s])
        dp = side_dist(pads) if pads else None
        pnear = min(SIDES, key=lambda s: dp[s]) if dp else ""
        rows.append({
            "ref": ref, "value": part.get("value", ""), "footprint": str(fp.GetFPID().GetLibItemName()),
            "fit": part.get("fit", "?"), "populate": str(part.get("populate", "?")), "side": "bottom" if back else "top",
            "tht": any(p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH for p in fp.Pads()),
            "body_source": "fab" if fab else ("courtyard" if crt else "pads"),
            "body_to_edge_mm": round(ds[near], 3), "body_nearest_edge": near,
            "pad_to_edge_mm": round(dp[pnear], 3) if dp else "", "pad_nearest_edge": pnear,
            "body_under_2p5": ds[near] < LIMIT_BODY, "body_under_0p3": ds[near] < LIMIT_ECONOMIC,
            "pad_under_0p3": bool(dp) and dp[pnear] < LIMIT_ECONOMIC,
            "per_side": {s: round(ds[s], 3) for s in SIDES},
        })
    fitted = [r for r in rows if r["fit"] in ("jlc", "jlc_optional")]
    per_side = {}
    for s in SIDES:
        close = sorted([r for r in fitted if r["per_side"][s] < LIMIT_BODY], key=lambda r: r["per_side"][s])
        per_side[s] = {"jlc_fitted_bodies_under_2p5": [[r["ref"], r["per_side"][s]] for r in close],
                       "count": len(close), "min_mm": close[0]["per_side"][s] if close else None,
                       "overhanging": [r["ref"] for r in close if r["per_side"][s] < 0]}
    pairs = {"-x / +x": ("-x", "+x"), "+y / -y": ("+y", "-y")}
    score = {}
    for name, (a, c) in pairs.items():
        others = [s for s in SIDES if s not in (a, c)]
        left = sorted({ref for s in others for ref, _ in per_side[s]["jlc_fitted_bodies_under_2p5"]})
        mins = [per_side[s]["min_mm"] for s in (a, c) if per_side[s]["min_mm"] is not None]
        score[name] = {"bodies_under_2p5_at_edges_without_rail": left, "count_without_rail": len(left),
                       "min_distance_at_rail_edges_mm": min(mins) if mins else None,
                       "overhang_at_rail_edges": sorted({r for s in (a, c) for r in per_side[s]["overhanging"]})}
    best = sorted(score, key=lambda n: (score[n]["count_without_rail"],
                                        score[n]["min_distance_at_rail_edges_mm"] if score[n]["min_distance_at_rail_edges_mm"] is not None else 99.0, n))[0]
    out_csv = ROOT / "out" / "edge_distance.csv"
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    cols = ["ref", "value", "footprint", "fit", "populate", "side", "tht", "body_source", "body_to_edge_mm", "body_nearest_edge",
            "pad_to_edge_mm", "pad_nearest_edge", "body_under_2p5", "body_under_0p3", "pad_under_0p3"]
    w.writerow(cols + ["dist_" + s for s in SIDES])
    for r in rows:
        w.writerow([r[c] for c in cols] + [r["per_side"][s] for s in SIDES])
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    out_csv.write_bytes(buf.getvalue().encode("utf-8"))
    try:
        board_rel = board_path.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        board_rel = board_path.name
    summary = {
        "board": board_rel, "board_sha256": hashlib.sha256(board_path.read_bytes()).hexdigest(),
        "tool": "pcbnew " + pcbnew.GetBuildVersion(), "label": "M (bounding boxes measured on the board file)",
        "outline_centre_lines": [OX0, OY0, OX1, OY1], "edge_cuts_bbox_incl_line_width": outline,
        "footprints": len(rows),
        "bodies_under_2p5": sorted([[r["ref"], r["body_to_edge_mm"], r["body_nearest_edge"], r["fit"]] for r in rows if r["body_under_2p5"]], key=lambda v: v[1]),
        "bodies_under_0p3": sorted([[r["ref"], r["body_to_edge_mm"], r["body_nearest_edge"], r["fit"]] for r in rows if r["body_under_0p3"]], key=lambda v: v[1]),
        "pads_under_0p3": sorted([[r["ref"], r["pad_to_edge_mm"], r["pad_nearest_edge"], r["fit"]] for r in rows if r["pad_under_0p3"]], key=lambda v: v[1]),
        "per_side": per_side, "rail_pairs": score,
        "rail_sides_for_standard_order": {"pair": best, "label": "E (chosen from this table: fewest JLC-fitted bodies under 2.5 mm at an edge without a rail)"},
        "jlc_acceptance": "NOT_VERIFIED (only JLC's DFM result and quote after the upload decide)",
        "csv": "out/edge_distance.csv", "csv_sha256": hashlib.sha256(out_csv.read_bytes()).hexdigest(),
    }
    rep = ROOT / "out" / "reports" / "edge_distance.json"
    rep.write_bytes((json.dumps(summary, indent=1) + "\n").encode("utf-8"))
    print(json.dumps({k: summary[k] for k in ("board", "footprints", "bodies_under_0p3", "pads_under_0p3", "rail_sides_for_standard_order")}, indent=1))
    print("bodies under 2.5 mm:", len(summary["bodies_under_2p5"]), "| per side:", {s: per_side[s]["count"] for s in SIDES})
    for n, v in score.items():
        print("rails", n, "-> left without rail:", v["count_without_rail"], v["bodies_under_2p5_at_edges_without_rail"], "overhang at rail edges:", v["overhang_at_rail_edges"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
