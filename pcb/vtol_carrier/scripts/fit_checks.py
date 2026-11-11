"""Fit tables of the VTOL carrier rev 3 -> out/fit_checks.json (REV3_PLAN section 14).

Run with KiCad's python (pcbnew):
    <KiCad>/bin/python.exe scripts/fit_checks.py [board.kicad_pcb]
default board: kicad/vtol_carrier.kicad_pcb; the place step runs it on kicad/work/vtol_carrier_unrouted.kicad_pcb.

Invariant: this script only MEASURES positions on the board file (M) and combines them with part heights and module
dimensions that are documents (F) or assumptions (A). Nothing here was measured on hardware: every module-fit result
is labelled "A / NOT_VERIFIED" and never PASS. No part is moved and no limit is changed here.

 (a) mcu_access   parts around the four TQFP-64 pad rings (hand rework access), same method as the rev 2c review
                  (gap between the MCU pad bounding box and the neighbour's F.Fab bounding box)
 (b) module_keepouts  outlines of the plugged modules built from spec/module_dimensions.json (tags carried through),
                  parts under them with height and clearance against CLEARANCE_MIN
 (c) tht_leads    lead protrusion under the board against the 1.5 mm foam-tape rule
"""
from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
MM = 1e-6
CLEARANCE_MIN = 0.5  # mm between a carrier part and the underside of a module (A, spec/module_dimensions.json)
LEAD_LIMIT = 1.5  # mm under the board (foam tape rule, REV3_PLAN section 14)
BOARD_T = 1.6  # mm (order parameter of the plan)
REV2C_ACCESS = {"U1": 14, "U2": 15, "U3": 20, "U4": 14}  # M: rev 2c board, all footprints closer than 1.0 mm
#: solder tail below the seating plane by JLC part: (nominal, plus tolerance or None, label, source)
TAILS = {
    "C32713261": (3.0, 0.2, "F", "HANXIA PZ2.54-1xNP WZ drawing (design note connectors_mounting section 5, D5)"),
    "C32713262": (3.0, 0.2, "F", "HANXIA PZ2.54-1xNP WZ drawing (design note connectors_mounting section 5, D5)"),
    "C32713263": (3.0, 0.2, "F", "HANXIA PZ2.54-1xNP WZ drawing (design note connectors_mounting section 5, D5)"),
    "C22436147": (3.0, 0.25, "F", "Hong Cheng HC-PM254-8.5H-1xNPW drawing (D6)"),
    "C55629861": (3.2, None, "F", "JLC part attribute only, no drawing (alternate of J2 / J5)"),
    "C131337": (3.4, None, "F", "JST ePH.pdf page 2, reference dimension (3.4)"),
    "C5364642": (None, None, "NOT_VERIFIED", "Molex 22035025 drawing not read (3 timeouts in the design step); the 3.0 mm in spec/module_dimensions.json is an assumption for the picture only"),
}


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


def gap(a: list[float], c: list[float]) -> float:
    dx = max(c[0] - a[2], a[0] - c[2], 0.0)
    dy = max(c[1] - a[3], a[1] - c[3], 0.0)
    return math.hypot(dx, dy)


def overlap(a: list[float], c: list[float]) -> float:
    w = min(a[2], c[2]) - max(a[0], c[0])
    h = min(a[3], c[3]) - max(a[1], c[1])
    return w * h if w > 1e-6 and h > 1e-6 else 0.0


def r3(v):
    return [round(x, 3) for x in v] if isinstance(v, (list, tuple)) else round(v, 3)


def main() -> int:
    board_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "kicad" / "vtol_carrier.kicad_pcb"
    spec = json.loads((ROOT / "spec" / "carrier_spec.json").read_text(encoding="utf-8"))
    parts = {p["refdes"]: p for p in spec["parts"]}
    placement_path = ROOT / "kicad" / "placement.json"
    placement = json.loads(placement_path.read_text(encoding="utf-8"))
    heights = placement.get("part_heights_mm", {})
    dims_path = ROOT / "spec" / "module_dimensions.json"
    dims = json.loads(dims_path.read_text(encoding="utf-8")) if dims_path.is_file() else None
    b = pcbnew.LoadBoard(str(board_path))
    fps = {}
    for fp in b.GetFootprints():
        ref = fp.GetReference()
        fab = bbox_of([g for g in fp.GraphicalItems() if g.GetLayer() == pcbnew.F_Fab and g.GetClass() != "PCB_TEXT"])
        crt = bbox_of([g for g in fp.GraphicalItems() if g.GetLayer() == pcbnew.F_CrtYd])
        pads = bbox_of(list(fp.Pads()))
        part = parts.get(ref, {})
        fps[ref] = {
            "fp": fp, "name": str(fp.GetFPID().GetLibItemName()), "body": fab or crt or pads, "pads": pads,
            "x": fp.GetPosition().x * MM, "y": fp.GetPosition().y * MM, "rot": fp.GetOrientationDegrees(),
            "tht": [p for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH],
            "fit": part.get("fit", "?"), "populate": part.get("populate"), "h": heights.get(ref),
            "jlc": (part.get("bom") or {}).get("jlc_code"),
        }
    errors: list[str] = []
    missing_h = sorted(r for r, f in fps.items() if f["h"] is None)
    if missing_h:
        errors.append("no height in kicad/placement.json part_heights_mm for: " + ", ".join(missing_h))

    def fitted(ref: str) -> bool:
        return fps[ref]["fit"] in ("jlc", "jlc_optional", "user") and fps[ref]["populate"] is True

    # ---------------------------------------------------------------- (a) MCU access
    access = {}
    for u in ("U1", "U2", "U3", "U4"):
        ring = fps[u]["pads"]
        rows = []
        for ref, f in fps.items():
            if ref == u or f["body"] is None:
                continue
            g = gap(ring, f["body"])
            if g <= 5.0:
                rows.append({"ref": ref, "footprint": f["name"], "gap_mm": round(g, 2), "height_mm": f["h"],
                             "fit": f["fit"], "fitted": fitted(ref)})
        rows.sort(key=lambda r: (r["gap_mm"], r["ref"]))
        near_all = [r for r in rows if r["gap_mm"] < 1.0]
        near_fit = [r for r in near_all if r["fitted"]]
        tall = [r for r in rows if r["fitted"] and (r["height_mm"] or 0.0) > 2.0]
        access[u] = {
            "pad_ring_bbox": r3(ring),
            "rev2c_all_footprints_within_1mm": REV2C_ACCESS[u],
            "all_footprints_within_1mm": len(near_all),
            "fitted_parts_within_1mm": len(near_fit),
            "fitted_within_1mm": [[r["ref"], r["gap_mm"], r["height_mm"]] for r in near_fit],
            "unfitted_within_1mm": [[r["ref"], r["gap_mm"]] for r in near_all if not r["fitted"]],
            "fitted_parts_higher_than_2mm_within_5mm": [[r["ref"], r["gap_mm"], r["height_mm"], r["footprint"]] for r in tall],
            "worse_than_rev2c": len(near_all) > REV2C_ACCESS[u],
        }

    # ---------------------------------------------------------------- (b) module keep-outs
    def to_board(ref: str, lx: float, ly: float) -> tuple[float, float]:
        f = fps[ref]
        a = math.radians(f["rot"])
        return (f["x"] + lx * math.cos(a) + ly * math.sin(a), f["y"] - lx * math.sin(a) + ly * math.cos(a))

    def local_rect(ref: str, lx0: float, ly0: float, lx1: float, ly1: float) -> list[float]:
        pts = [to_board(ref, x, y) for x in (lx0, lx1) for y in (ly0, ly1)]
        return [min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts)]

    def check_pads(ref: str, pitch: float) -> None:
        """the local frame used here (pad n at local (0, (n-1) * pitch)) must reproduce the pads of the board file"""
        for p in fps[ref]["fp"].Pads():
            n = int(p.GetNumber())
            ex, ey = to_board(ref, 0.0, (n - 1) * pitch)
            px, py = p.GetPosition().x * MM, p.GetPosition().y * MM
            if math.hypot(ex - px, ey - py) > 0.01:
                errors.append(f"{ref} pad {n}: local frame gives ({ex:.3f}, {ey:.3f}), board has ({px:.3f}, {py:.3f})")

    def under(zone: list[float], own: str) -> list[dict]:
        rows = []
        for ref, f in sorted(fps.items()):
            if ref == own or f["body"] is None:
                continue
            ov = overlap(zone, f["body"])
            if ov > 0.0:
                rows.append({"ref": ref, "footprint": f["name"], "fit": f["fit"], "fitted": fitted(ref), "height_mm": f["h"],
                             "overlap_mm2": round(ov, 2), "how": placement["placements"].get(ref, {}).get("how")})
        return rows

    def zone_report(name: str, own: str, rect: list[float], z_under: float, z_note: str, tags: str, z_cons: float | None = None,
                    cons_note: str = "") -> dict:
        rows = []
        for r in under(rect, own):
            h = r["height_mm"] if r["fitted"] else 0.0
            r["clearance_mm"] = round(z_under - h, 2)
            if z_cons is not None:
                r["clearance_conservative_mm"] = round(z_cons - h, 2)
            rows.append(r)
        fit_rows = [r for r in rows if r["fitted"]]
        out = {"zone": name, "rect_kicad": r3(rect), "underside_z_mm": round(z_under, 3), "underside_note": z_note, "tags": tags,
               "parts_under": rows,
               "min_clearance_mm": min((r["clearance_mm"] for r in fit_rows), default=None),
               "below_clearance_min": [[r["ref"], r["height_mm"], r["clearance_mm"]] for r in fit_rows if r["clearance_mm"] < CLEARANCE_MIN],
               "collisions": [[r["ref"], r["height_mm"], r["clearance_mm"]] for r in fit_rows if r["clearance_mm"] <= 0.0]}
        if z_cons is not None:
            out["conservative_underside_z_mm"] = round(z_cons, 3)
            out["conservative_note"] = cons_note
            out["min_clearance_conservative_mm"] = min((r["clearance_conservative_mm"] for r in fit_rows), default=None)
            out["below_clearance_min_conservative"] = [[r["ref"], r["height_mm"], r["clearance_conservative_mm"]] for r in fit_rows
                                                       if r["clearance_conservative_mm"] < CLEARANCE_MIN]
        return out

    modules: dict = {}
    if dims is None:
        modules = {"status": "NOT_VERIFIED", "why": "spec/module_dimensions.json does not exist"}
    else:
        def v(sec: str, key: str):
            return dims[sec][key]["v"]

        def t(sec: str, key: str) -> str:
            return f"{sec}.{key}={dims[sec][key]['v']}({dims[sec][key]['tag']})"

        pitch = v("pin_strip_2p54", "pitch")
        hole_x = v("socket_j2_j5", "opening_x") - v("pin_strip_2p54", "ra_plastic") - v("pin_strip_2p54", "ra_hole_to_plastic")
        z_ra = v("socket_j2_j5", "contact_axis_z") + v("pin_strip_2p54", "ra_plastic") / 2.0
        stroke = v("pin_strip_2p54", "ra_mating_pin_len")
        ra_tags = ", ".join([t("socket_j2_j5", "opening_x"), t("pin_strip_2p54", "ra_plastic"), t("pin_strip_2p54", "ra_hole_to_plastic"),
                             t("socket_j2_j5", "contact_axis_z"), t("pin_strip_2p54", "ra_mating_pin_len")])
        row_len = 5 * pitch
        # ---- OpenLog in J5
        check_pads("J5", pitch)
        ol_w, ol_l = v("openlog", "pcb_w"), v("openlog", "pcb_l")
        near = hole_x + v("openlog", "header_row_y")
        far = near - ol_w
        side = (ol_l - row_len) / 2.0
        ly0, ly1 = -side, row_len + side
        ol_tags = ", ".join([ra_tags, t("openlog", "pcb_w"), t("openlog", "pcb_l"), t("openlog", "header_row_y"), t("openlog", "pcb_t"),
                             t("openlog", "tqfp32_h_max"), t("openlog", "face_up")])
        z_cons = z_ra - v("openlog", "tqfp32_h_max")
        card_end = near - v("openlog", "microsd_drawing_end_y")
        z_card = z_ra + v("openlog", "pcb_t")
        atx, aty = dims["openlog"]["top_parts"]["v"]["U1_atmega328_tqfp32"] if "v" in dims["openlog"]["top_parts"] else dims["openlog"]["top_parts"]["U1_atmega328_tqfp32"]
        half = 4.5  # TQFP-32 body 7 x 7 mm + leads (9 x 9 mm over the leads, A)
        a_lx0, a_lx1 = near - (aty + half), near - (aty - half)
        a_ly = sorted([atx - half - v("openlog", "header_row_y"), atx + half - v("openlog", "header_row_y"),
                       ol_l - (atx + half) - v("openlog", "header_row_y"), ol_l - (atx - half) - v("openlog", "header_row_y")])
        j5 = {
            "module": dims["openlog"]["what"], "socket": "J5", "label": "A / NOT_VERIFIED (module not measured; assumed dimensions in the chain)",
            "rev2c_reserved_rect_kicad": next((m["rect"] for m in placement["placement_checks"].get("module_bodies", []) if m["ref"] == "J5"), None),
            "zones": [
                zone_report("OpenLog PCB, plugged", "J5", local_rect("J5", far, ly0, near, ly1), z_ra,
                            "underside of the bare module PCB (ATmega side faces the carrier)", ol_tags, z_cons,
                            "whole underside lowered by openlog.tqfp32_h_max (the module's own parts hang below its PCB)"),
                zone_report("OpenLog ATmega328 TQFP-32 (both mirror cases)", "J5", local_rect("J5", a_lx0, a_ly[0], a_lx1, a_ly[3]), z_cons,
                            "underside of the module's TQFP-32 incl. leads, 9 x 9 mm (A)", ol_tags + ", " + t("openlog", "top_parts")),
                zone_report("right-angle header plastic of the module (rests on the carrier)", "J5",
                            local_rect("J5", v("socket_j2_j5", "opening_x") - v("pin_strip_2p54", "ra_plastic"), ly0, v("socket_j2_j5", "opening_x"), ly1),
                            0.0, "plastic bar z 0..2.5 mm: no part may stand here", ra_tags),
                zone_report("insertion stroke beyond the plugged PCB end", "J5", local_rect("J5", far - stroke, ly0, far, ly1), z_ra,
                            "the far PCB end passes here while plugging (horizontal insertion, stroke = mating pin length)", ol_tags, z_cons,
                            "whole underside lowered by openlog.tqfp32_h_max"),
                zone_report("microSD card overhang beyond the PCB end", "J5", local_rect("J5", card_end, ly0, far, ly1), z_card,
                            "card lies on the top side of the module: underside = module PCB top; full PCB width taken (card side offset not used)",
                            ol_tags + ", " + t("openlog", "microsd_drawing_end_y")),
            ],
        }
        modules["J5_openlog"] = j5
        # ---- BMP280 in J2
        check_pads("J2", pitch)
        bw, bl = v("bmp280", "pcb_w"), v("bmp280", "pcb_l")
        bnear = hole_x + v("bmp280", "header_row_inset")
        bfar = bnear - bw
        bside = (bl - row_len) / 2.0
        b_tags = ", ".join([ra_tags, t("bmp280", "pcb_w"), t("bmp280", "pcb_l"), t("bmp280", "header_row_inset"), t("bmp280", "part_h_max"), t("bmp280", "face_up")])
        zb_cons = z_ra - v("bmp280", "part_h_max")
        modules["J2_bmp280"] = {
            "module": dims["bmp280"]["what"], "socket": "J2", "label": "A / NOT_VERIFIED (module not measured; assumed dimensions in the chain)",
            "rev2c_reserved_rect_kicad": next((m["rect"] for m in placement["placement_checks"].get("module_bodies", []) if m["ref"] == "J2"), None),
            "zones": [
                zone_report("BMP280 PCB, plugged", "J2", local_rect("J2", bfar, -bside, bnear, row_len + bside), z_ra,
                            "underside of the bare module PCB", b_tags, zb_cons, "whole underside lowered by bmp280.part_h_max (component side toward the carrier, A)"),
                zone_report("right-angle header plastic of the module (rests on the carrier)", "J2",
                            local_rect("J2", v("socket_j2_j5", "opening_x") - v("pin_strip_2p54", "ra_plastic"), -bside, v("socket_j2_j5", "opening_x"), row_len + bside),
                            0.0, "plastic bar z 0..2.5 mm: no part may stand here", ra_tags),
                zone_report("insertion stroke beyond the plugged PCB end", "J2", local_rect("J2", bfar - stroke, -bside, bfar, row_len + bside), z_ra,
                            "the far PCB end passes here while plugging", b_tags, zb_cons, "whole underside lowered by bmp280.part_h_max"),
            ],
        }
        # ---- GY-521 on J1 (straight strip, component side up)
        check_pads("J1", pitch)
        gw, gl = v("gy521", "pcb_w"), v("gy521", "pcb_l")
        gin = v("gy521", "header_row_inset")
        gside = (gl - 7 * pitch) / 2.0
        g_tags = ", ".join([t("gy521", "pcb_w"), t("gy521", "pcb_l"), t("gy521", "header_row_inset"), t("pin_strip_2p54", "straight_plastic_h"),
                            "mounting.J1.body_side=" + str(dims["mounting"]["J1"]["body_side"]["v"]) + "(" + dims["mounting"]["J1"]["body_side"]["tag"] + ")"])
        sgn = 1.0 if dims["mounting"]["J1"]["body_side"]["v"] == "+x" else -1.0
        gx = sorted([-sgn * gin, sgn * (gw - gin)])
        modules["J1_gy521"] = {
            "module": dims["gy521"]["what"], "socket": "J1", "label": "A / NOT_VERIFIED (module not measured; every dimension assumed)",
            "rev2c_reserved_rect_kicad": next((m["rect"] for m in placement["placement_checks"].get("module_bodies", []) if m["ref"] == "J1"), None),
            "zones": [
                zone_report("GY-521 PCB on its straight strip", "J1", local_rect("J1", gx[0], -gside, gx[1], 7 * pitch + gside),
                            v("pin_strip_2p54", "straight_plastic_h"), "underside of the module PCB (component side up; strip solder joints not modelled)", g_tags),
            ],
        }
    worst = []
    for key, m in modules.items():
        for z in m.get("zones", []):
            for ref, h, c in z["collisions"]:
                worst.append(f"{key} / {z['zone']}: {ref} ({h} mm) clearance {c} mm")

    # ---------------------------------------------------------------- (c) THT leads
    leads = []
    for ref, f in sorted(fps.items(), key=lambda kv: (kv[0].rstrip("0123456789"), int("".join(c for c in kv[0] if c.isdigit()) or 0))):
        if not f["tht"]:
            continue
        drills = sorted({round(p.GetDrillSize().x * MM, 3) for p in f["tht"]})
        row = {"ref": ref, "footprint": f["name"], "fit": f["fit"], "jlc_code": f["jlc"], "plated_holes": len(f["tht"]), "drill_mm": drills}
        if f["fit"] == "none" or f["populate"] is not True:
            row.update({"under_board_nominal_mm": 0.0, "result": "no part fitted: bare plated holes, nothing protrudes", "label": "M"})
        elif f["fit"] == "user":
            row.update({"under_board_nominal_mm": None, "label": "NOT_VERIFIED",
                        "result": "soldered by the user (module strip): the user cuts the leads to <= 1.5 mm after soldering"})
        else:
            tail = TAILS.get(f["jlc"])
            if tail is None or tail[0] is None:
                row.update({"under_board_nominal_mm": None, "label": "NOT_VERIFIED", "source": tail[3] if tail else "no tail length recorded for this part",
                            "result": "NOT_VERIFIED: the user measures the leads after delivery and cuts what exceeds 1.5 mm"})
            else:
                nom = round(tail[0] - BOARD_T, 2)
                mx = round(nom + tail[1], 2) if tail[1] is not None else None
                if nom > LEAD_LIMIT:
                    res = f"OVER the {LEAD_LIMIT} mm rule: the user flush-cuts {len(f['tht'])} leads"
                elif mx is not None and mx > LEAD_LIMIT:
                    res = f"nominal inside, {round(mx - LEAD_LIMIT, 2)} mm over at the upper tolerance: the user checks and cuts"
                elif mx is None:
                    res = "nominal inside, tolerance unknown: the user checks"
                else:
                    res = "inside the rule"
                row.update({"tail_mm": tail[0], "tail_plus_tol_mm": tail[1], "under_board_nominal_mm": nom, "under_board_max_mm": mx,
                            "label": tail[2] + " (document value; not measured on a delivered part)", "source": tail[3], "result": res})
        leads.append(row)
    over = [r["ref"] for r in leads if isinstance(r.get("under_board_nominal_mm"), float) and r["under_board_nominal_mm"] > LEAD_LIMIT]
    unknown = [r["ref"] for r in leads if r.get("under_board_nominal_mm") is None]

    try:
        board_rel = board_path.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        board_rel = board_path.name
    out = {
        "schema": "vtol_carrier.fit_checks/1",
        "board": board_rel, "board_sha256": hashlib.sha256(board_path.read_bytes()).hexdigest(),
        "placement_sha256": hashlib.sha256(placement_path.read_bytes()).hexdigest(),
        "module_dimensions_sha256": hashlib.sha256(dims_path.read_bytes()).hexdigest() if dims else None,
        "tool": "pcbnew " + pcbnew.GetBuildVersion(),
        "labels": "positions M (board file); part heights A (kicad/placement.json part_heights_mm); module dimensions F / A as tagged; no hardware measurement",
        "errors": errors,
        "mcu_access": {"method": "gap between the MCU pad bounding box and the neighbour's F.Fab bounding box (same as the rev 2c review)",
                       "label": "M", "per_mcu": access},
        "module_keepouts": {"clearance_min_mm": CLEARANCE_MIN, "status": "A / NOT_VERIFIED", "collisions_under_stated_assumptions": worst, "modules": modules},
        "tht_leads": {"rule_mm": LEAD_LIMIT, "board_thickness_mm": BOARD_T, "over_rule_nominal": over, "not_verified": unknown,
                      "note": "solder fillet height is not included (NOT_VERIFIED); alternate socket C55629861 has 3.2 mm tails -> 1.6 mm under the board",
                      "rows": leads},
    }
    rep = ROOT / "out" / "fit_checks.json"
    rep.write_bytes((json.dumps(out, indent=1, ensure_ascii=False) + "\n").encode("utf-8"))
    for u, a in access.items():
        print(u, "within 1 mm: all", a["all_footprints_within_1mm"], "(rev 2c", str(a["rev2c_all_footprints_within_1mm"]) + ") fitted", a["fitted_parts_within_1mm"],
              "| fitted > 2 mm within 5 mm:", [(r[0], r[1], r[2]) for r in a["fitted_parts_higher_than_2mm_within_5mm"]])
    for key, m in modules.items():
        for z in m.get("zones", []):
            print(key, "|", z["zone"], r3(z["rect_kicad"]), "z", z["underside_z_mm"], "min", z["min_clearance_mm"], "below 0.5:", z["below_clearance_min"],
                  "| cons", z.get("conservative_underside_z_mm"), z.get("below_clearance_min_conservative"))
    print("THT over rule:", over, "not verified:", unknown)
    print("errors:", errors)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
