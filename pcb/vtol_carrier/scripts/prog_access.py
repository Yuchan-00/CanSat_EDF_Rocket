"""Decision D6 search: can the eight programming headers leave the tie band as JLC-fitted parts (option B)?

    python scripts/prog_access.py            -> out/reports/programming_access.json
    (picture: scripts/prog_access_plot.py, any python with matplotlib, reads only that JSON)

Invariant: this script only SEARCHES and REPORTS. It never writes kicad/placement.json and never changes the spec; the
board gets option B only if place.PROG_B holds eight legal spots and the spec is rebuilt with VTOL_PROG_HEADERS=B.
Option B is all or nothing (REV3_PLAN D6): J3, J19..J22 (2x3) and J24..J26 (1x2) all outside the band, or none.

Rules for a header spot (none of them is weaker than the placer's):
  * courtyard inside the outline with the 0.25 mm edge margin, outside the tie band x 97..103 and the ESP antenna area
  * courtyard clear of every other courtyard (0.05 mm raster, conservative like scripts/place.py)
  * not under a module body, not in a module's insertion path, card overhang or header-plastic strip (place.MODULES,
    place.LOW_ZONES, out/fit_checks.json zones) - a header is 8.6 mm tall and must be reachable from above
  * 2x3 only: room for the plug body PLUG (A) centred on the header, long side along the 3-pin axis, plugged from
    above: no part taller than PLUG_UNDER_H and no module under it (the plug may hang over the board edge / the band)
Two occupancy cases:
  final       the rev 3 placement as written (kicad/placement.json rules re-run in memory), only the band holes removed
  fixed_only  only the parts that may not move (anchors, MCUs, MCU clusters, ESP parts, power block): every greedy
              passive taken off the board. No spot here = no spot at all without moving a fixed part (upper bound).
  fixed_without_power_block  as fixed_only, the boost / UVLO block also taken off (information: the block is part of
              the plan and has no other legal corner; this case only shows how far away option B is).
For each case: number of legal origins (0.05 mm grid), an upper bound of how many headers fit at once (union of all
legal courtyards / one courtyard) and a greedy packing (lower bound).
"""
from __future__ import annotations

import json
import math
import sys

from ai_eda.ir import LibraryRef
from ai_eda.tools.kicad.library import KicadLibrary

import common
import outline
import place
from place import BX0, BX1, BY0, BY1, CELL, NX, NY, TOP, Rect

PAD = 320  # canvas margin in cells (16 mm): windows may reach beyond the board
W = NX + 2 * PAD
H = NY + 2 * PAD
FULL = (1 << W) - 1
PLUG = (6.4, 12.7)  # mm across / along the 3-pin axis: 2x3 IDC socket body (A - no drawing read; measure the plug in hand)
PLUG_UNDER_H = 2.0  # mm: taller neighbours block the plug body (the plug sits on the 2.54 mm header plastic, A)
HEADER_H = 8.6
NEED = {"2x3": 5, "1x2": 3}
CANDIDATES = {
    "smd": {"2x3": "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical_SMD",
            "1x2": "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical_SMD_Pin1Right"},
    "tht": {"2x3": "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical",
            "1x2": "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical"},
}
FIXED_HOW = ("anchor", "cluster", "esp", "power")


class Canvas:
    """Bit rows on a padded canvas; bit (PAD + i) of row (PAD + j) = board cell (i, j)."""

    def __init__(self, outside: bool) -> None:
        board = ((1 << NX) - 1) << PAD
        out_row = FULL if outside else 0
        self.rows = [out_row] * PAD + [(FULL & ~board) if outside else 0 for _ in range(NY)] + [out_row] * PAD

    def mark(self, r: Rect) -> None:
        i0 = max(-PAD, math.floor((r.x0 - BX0) / CELL + 1e-9)) + PAD
        i1 = min(NX + PAD, math.ceil((r.x1 - BX0) / CELL - 1e-9)) + PAD
        j0 = max(-PAD, math.floor((r.y0 - BY0) / CELL + 1e-9)) + PAD
        j1 = min(NY + PAD, math.ceil((r.y1 - BY0) / CELL - 1e-9)) + PAD
        if i1 <= i0 or j1 <= j0:
            return
        m = ((1 << (i1 - i0)) - 1) << i0
        for j in range(j0, j1):
            self.rows[j] |= m

    def copy(self) -> "Canvas":
        c = Canvas(False)
        c.rows = list(self.rows)
        return c


def _and_run(x: int, n: int) -> int:
    k = 1
    while k * 2 <= n:
        x &= x >> k
        k *= 2
    if k < n:
        x &= x >> (n - k)
    return x


def window_free(blocked: list[int], w: int, h: int) -> list[int]:
    """bit i of row j set = the w x h window whose top-left cell is (i, j) has no blocked cell"""
    hz = [_and_run((~r) & FULL, w) for r in blocked]
    k = 1
    n = len(hz)
    while k * 2 <= h:
        hz = [hz[j] & hz[j + k] if j + k < n else 0 for j in range(n)]
        k *= 2
    if k < h:
        d = h - k
        hz = [hz[j] & hz[j + d] if j + d < n else 0 for j in range(n)]
    return hz


def count(rows: list[int]) -> int:
    return sum(bin(r).count("1") for r in rows)


def union_cells(legal: list[int], w: int, h: int) -> int:
    """number of cells covered by at least one legal window"""
    cov = [0] * len(legal)
    for j, r in enumerate(legal):
        if not r:
            continue
        x = r
        k = 1
        while k * 2 <= w:
            x |= x << k
            k *= 2
        if k < w:
            x |= x << (w - k)
        for jj in range(j, min(len(legal), j + h)):
            cov[jj] |= x
    return count(cov)


def cells(rect: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
    """cell offsets of a rectangle relative to an origin on the 0.05 mm grid (conservative)"""
    return (math.floor(rect[0] / CELL + 1e-9), math.floor(rect[1] / CELL + 1e-9),
            math.ceil(rect[2] / CELL - 1e-9), math.ceil(rect[3] / CELL - 1e-9))


def rot_rect(r: tuple[float, float, float, float], rot: int) -> tuple[float, float, float, float]:
    pts = []
    for lx in (r[0], r[2]):
        for ly in (r[1], r[3]):
            pts.append({0: (lx, ly), 90: (ly, -lx), 180: (-lx, -ly), 270: (-ly, lx)}[rot])
    return (min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts))


def bbox(fp, layer: str) -> tuple[float, float, float, float]:
    segs = place.courtyard_segments(fp, layer)
    xs = [p[0] for s in segs for p in s]
    ys = [p[1] for s in segs for p in s]
    return (min(xs), min(ys), max(xs), max(ys))


def runs(rows: list[int], di: int, dj: int) -> list[list[float]]:
    """legal origins as [y, x_first, x_last] runs (KiCad mm); (di, dj) = origin cell minus window top-left cell"""
    out = []
    for j, r in enumerate(rows):
        i = 0
        while r:
            low = (r & -r).bit_length() - 1
            r >>= low
            i += low
            n = ((~r) & (r + 1)).bit_length() - 1  # length of the run of ones
            out.append([round(BY0 + (j - PAD + dj) * CELL, 2), round(BX0 + (i - PAD + di) * CELL, 2), round(BX0 + (i + n - 1 - PAD + di) * CELL, 2)])
            r >>= n
            i += n
    return out


def block_outside_outline(c: Canvas) -> dict:
    """Outline adaptation (redesign2 fix stage): place.NY now spans the outline's bounding box (y down to the tab
    bottom, outline.BBOX), so the rows below y 135 are board-wide. Every cell of those rows that is not inside the
    outline with the placer's EDGE_MARGIN (outline.rect_inside on the 0.05 mm cell, the rule place.Shape.inside_board
    applies to a courtyard) is blocked: beside the tab, below the tab bottom and outside its outer corner arcs. Without
    this the band y 135 .. tab bottom across x 62..138 (about 500 mm2 off the board, the tie band included) counted as
    free board. Rows above y 135 - EDGE_MARGIN are unchanged (the rev 3 rectangle with its margin strips)."""
    m = place.EDGE_MARGIN
    j_first = math.floor((BY1 - m - BY0) / CELL + 1e-9)
    n = 0
    for j in range(max(0, j_first), NY):
        y0, y1 = BY0 + j * CELL, BY0 + (j + 1) * CELL
        bits = 0
        for i in range(NX):
            x0 = BX0 + i * CELL
            if not outline.rect_inside((x0, y0, x0 + CELL, y1), m):
                bits |= 1 << i
        n += bin(bits).count("1")
        c.rows[PAD + j] |= bits << PAD
    return {"rows_kicad_y": [round(BY0 + max(0, j_first) * CELL, 3), round(BY0 + NY * CELL, 3)], "cells_blocked": n,
            "area_blocked_mm2": round(n * CELL * CELL, 1),
            "rule": "cell blocked unless outline.rect_inside(cell, place.EDGE_MARGIN) (outline adaptation)"}


def main() -> int:
    spec = common.load_spec()
    lib = KicadLibrary()
    pl = place.Placer(spec, lib)
    pl.run()
    if pl.problems:
        print("placer problems:", pl.problems)
        return 1
    fit_path = common.OUT_DIR / "fit_checks.json"
    if not fit_path.is_file():
        print("run scripts/fit_checks.py first (module zones)")
        return 1
    fit = json.loads(fit_path.read_text(encoding="utf-8"))
    zones = []  # (rect, why): no header courtyard and no plug body here
    for b in pl.bodies:
        zones.append((tuple(b["rect"]), "module body (place.MODULES): " + b["name"]))
    for rect, _max_h, why in place.LOW_ZONES:
        zones.append((tuple(rect), "place.LOW_ZONES: " + why))
    for key, m in fit["module_keepouts"]["modules"].items():
        for z in m.get("zones", []):
            zones.append((tuple(z["rect_kicad"]), f"out/fit_checks.json {key}: {z['zone']}"))
    static = Canvas(True)  # header courtyards: outside the board is blocked
    for r in (Rect(BX0, BY0, BX1, BY0 + place.EDGE_MARGIN), Rect(BX0, BY1 - place.EDGE_MARGIN, BX1, BY1),
              Rect(BX0, BY0, BX0 + place.EDGE_MARGIN, BY1), Rect(BX1 - place.EDGE_MARGIN, BY0, BX1, BY1),
              Rect(place.TIE_X0, BY0, place.TIE_X1, BY1), Rect(*place.ESP_KEEPOUT)):
        static.mark(r)
    outside = block_outside_outline(static)
    plug_static = Canvas(False)  # plug body: outside the board is free
    for rect, _why in zones:
        static.mark(Rect(*rect))
        plug_static.mark(Rect(*rect))
    cy = {r: place.courtyard_rect(pl, r) for r in pl.placed}
    cases = {
        "final": [r for r in pl.placed if r not in common.IN_BAND],
        "fixed_only": [r for r in pl.placed if r not in common.IN_BAND and pl.placed[r]["how"] in FIXED_HOW],
        # information only: the boost / UVLO block is fixed by the plan; this case shows what the corner alone would give
        "fixed_without_power_block": [r for r in pl.placed if r not in common.IN_BAND and pl.placed[r]["how"] in ("anchor", "cluster", "esp")],
    }
    fps = {}
    for fam, d in CANDIDATES.items():
        for kind, lib_id in d.items():
            l, name = common.split_lib_id(lib_id)
            fp = lib.load_footprint(LibraryRef(library=l, name=name))
            fps[(fam, kind)] = {"lib_id": lib_id, "courtyard": bbox(fp, "F.CrtYd"), "fab": bbox(fp, "F.Fab"),
                                "tht": any(getattr(p, "drill", None) for p in fp.pads)}
    report: dict = {
        "schema": "vtol_carrier.programming_access/1",
        "decision": "D6",
        "spec_sha256": common.sha256_file(common.SPEC_PATH),
        "spec_option": common.prog_headers(spec),
        "need": NEED,
        "assumptions": {"plug_body_mm": {"across": PLUG[0], "along_3_pin_axis": PLUG[1], "label": "A (2x3 IDC socket body, no drawing read)"},
                        "plug_under_height_mm": {"v": PLUG_UNDER_H, "label": "A"}, "header_height_mm": {"v": HEADER_H, "label": "A (place.FP_HEIGHT)"},
                        "grid_mm": CELL, "rotations": [0, 90, 180, 270]},
        "forbidden_zones": [{"rect_kicad": [round(v, 3) for v in rect], "why": why} for rect, why in zones],
        "tie_band_kicad": [place.TIE_X0, BY0, place.TIE_X1, BY1], "antenna_rule_area_kicad": list(place.ESP_KEEPOUT),
        "outline_cells_blocked": outside,
        "candidates": {f"{fam}_{kind}": {"footprint": v["lib_id"], "courtyard_local": [round(x, 3) for x in v["courtyard"]],
                                           "courtyard_mm": [round(v["courtyard"][2] - v["courtyard"][0], 2), round(v["courtyard"][3] - v["courtyard"][1], 2)]}
                       for (fam, kind), v in fps.items()},
        "cases": {},
    }
    for case, refs in cases.items():
        occ = static.copy()
        tall = plug_static.copy()
        for r in refs:
            occ.mark(cy[r])
            if pl.height(r) > PLUG_UNDER_H:
                tall.mark(cy[r])
        free_cells = count([(~r) & FULL for r in occ.rows])
        greedy_area = sum((cy[r].x1 - cy[r].x0) * (cy[r].y1 - cy[r].y0) for r in pl.placed
                          if r not in common.IN_BAND and pl.placed[r]["how"] not in FIXED_HOW)
        res: dict = {"parts_on_board": len(refs), "free_area_for_a_header_courtyard_mm2": round(free_cells * CELL * CELL, 1),
                     "courtyard_area_of_the_greedy_parts_mm2": round(greedy_area, 1), "families": {}}
        for fam in CANDIDATES:
            fres: dict = {"kinds": {}}
            legal_all: dict = {}
            for kind in ("2x3", "1x2"):
                f = fps[(fam, kind)]
                per_rot = {}
                best_union = 0
                for rot in (0, 90, 180, 270):
                    c = cells(rot_rect(f["courtyard"], rot))
                    w, h = c[2] - c[0], c[3] - c[1]
                    legal = window_free(occ.rows, w, h)
                    n_cy = count(legal)
                    ub_cy = (union_cells(legal, w, h) // (w * h)) if n_cy else 0
                    if kind == "2x3":
                        fb = f["fab"]
                        cx, cyy = (fb[0] + fb[2]) / 2.0, (fb[1] + fb[3]) / 2.0  # header centre (3-pin axis = local y, read from the fab outline)
                        assert (fb[3] - fb[1]) > (fb[2] - fb[0]), "3-pin axis expected along local y"
                        pc = cells(rot_rect((cx - PLUG[0] / 2, cyy - PLUG[1] / 2, cx + PLUG[0] / 2, cyy + PLUG[1] / 2), rot))
                        pfree = window_free(tall.rows, pc[2] - pc[0], pc[3] - pc[1])
                        di, dj = pc[0] - c[0], pc[1] - c[1]
                        legal = [legal[j] & ((pfree[j + dj] >> di) if di >= 0 else (pfree[j + dj] << -di) & FULL) if 0 <= j + dj < H else 0 for j in range(H)]
                    n = count(legal)
                    un = union_cells(legal, w, h)
                    best_union += un
                    per_rot[rot] = {"legal_origins_courtyard_only": n_cy, "legal_origins": n,
                                    "upper_bound_fit_at_once": (un // (w * h)) if n else 0,
                                    "upper_bound_courtyard_only": ub_cy}
                    legal_all[(kind, rot)] = (legal, c, w, h)
                    if n and n <= 20000:
                        per_rot[rot]["origin_runs_y_x0_x1"] = runs(legal, -c[0], -c[1])
                fres["kinds"][kind] = {"need": NEED[kind], "per_rotation": per_rot,
                                       "legal_origins_total": sum(v["legal_origins"] for v in per_rot.values()),
                                       "upper_bound_fit_at_once": per_rot[0]["upper_bound_fit_at_once"] + per_rot[90]["upper_bound_fit_at_once"],
                                       "upper_bound_courtyard_only": per_rot[0]["upper_bound_courtyard_only"] + per_rot[90]["upper_bound_courtyard_only"]}
            # greedy packing: 2x3 first (the plug body of a placed header blocks nothing but taller parts: the header itself is tall)
            packs = []
            for order in ("top_left", "bottom_right", "left_top", "right_bottom"):
                o2, t2 = occ.copy(), tall.copy()
                placed = []
                for kind in ("2x3", "1x2"):
                    f = fps[(fam, kind)]
                    for _ in range(NEED[kind]):
                        spot = None
                        for rot in (0, 90, 180, 270):
                            c = cells(rot_rect(f["courtyard"], rot))
                            w, h = c[2] - c[0], c[3] - c[1]
                            legal = window_free(o2.rows, w, h)
                            if kind == "2x3":
                                fb = f["fab"]
                                cx, cyy = (fb[0] + fb[2]) / 2.0, (fb[1] + fb[3]) / 2.0
                                pc = cells(rot_rect((cx - PLUG[0] / 2, cyy - PLUG[1] / 2, cx + PLUG[0] / 2, cyy + PLUG[1] / 2), rot))
                                pfree = window_free(t2.rows, pc[2] - pc[0], pc[3] - pc[1])
                                di, dj = pc[0] - c[0], pc[1] - c[1]
                                legal = [legal[j] & ((pfree[j + dj] >> di) if di >= 0 else (pfree[j + dj] << -di) & FULL) if 0 <= j + dj < H else 0 for j in range(H)]
                            cand = None
                            for j in (range(H) if order in ("top_left", "left_top", "right_bottom") else range(H - 1, -1, -1)):
                                r = legal[j]
                                if not r:
                                    continue
                                lo = (r & -r).bit_length() - 1
                                hi = r.bit_length() - 1
                                if order == "top_left":
                                    cand = (j, lo)
                                    break
                                if order == "bottom_right":
                                    cand = (j, hi)
                                    break
                                if order == "left_top" and (cand is None or lo < cand[1]):
                                    cand = (j, lo)
                                if order == "right_bottom" and (cand is None or hi >= cand[1]):
                                    cand = (j, hi)
                            if cand is not None:
                                key = {"top_left": (cand[0], cand[1]), "bottom_right": (-cand[0], -cand[1]),
                                       "left_top": (cand[1], cand[0]), "right_bottom": (-cand[1], -cand[0])}[order]
                                if spot is None or key < spot[0]:
                                    spot = (key, cand, rot, c, w, h)
                        if spot is None:
                            break
                        _key, (j, i), rot, c, w, h = spot
                        x0, y0 = BX0 + (i - PAD) * CELL, BY0 + (j - PAD) * CELL
                        rect = Rect(x0, y0, x0 + w * CELL, y0 + h * CELL)
                        o2.mark(rect)
                        t2.mark(rect)
                        placed.append({"kind": kind, "rot": rot, "x": round(x0 - c[0] * CELL, 2), "y": round(y0 - c[1] * CELL, 2),
                                       "courtyard_kicad": [round(v, 2) for v in (rect.x0, rect.y0, rect.x1, rect.y1)]})
                packs.append({"order": order, "placed_2x3": sum(1 for p in placed if p["kind"] == "2x3"),
                              "placed_1x2": sum(1 for p in placed if p["kind"] == "1x2"), "headers": placed})
            best = max(packs, key=lambda p: (p["placed_2x3"] + p["placed_1x2"], p["placed_2x3"]))
            fres["greedy_packings"] = [{k: v for k, v in p.items() if k != "headers"} for p in packs]
            fres["best_greedy_packing"] = best
            fres["all_eight_fit_in_greedy_packing"] = best["placed_2x3"] == NEED["2x3"] and best["placed_1x2"] == NEED["1x2"]
            ub = fres["kinds"]["2x3"]["upper_bound_fit_at_once"]
            fres["impossible_by_upper_bound"] = ub < NEED["2x3"] or fres["kinds"]["1x2"]["upper_bound_fit_at_once"] < NEED["1x2"]
            fres["impossible_by_courtyard_only_bound"] = (fres["kinds"]["2x3"]["upper_bound_courtyard_only"] < NEED["2x3"]
                                                          or fres["kinds"]["1x2"]["upper_bound_courtyard_only"] < NEED["1x2"])
            hdr_area = sum(NEED[k] * (fps[(fam, k)]["courtyard"][2] - fps[(fam, k)]["courtyard"][0]) * (fps[(fam, k)]["courtyard"][3] - fps[(fam, k)]["courtyard"][1]) for k in NEED)
            fres["courtyard_area_of_the_eight_headers_mm2"] = round(hdr_area, 1)
            res["families"][fam] = fres
        report["cases"][case] = res
    # ---- verdict
    fin, fix = report["cases"]["final"]["families"], report["cases"]["fixed_only"]["families"]
    verdict = {}
    for fam in CANDIDATES:
        area_short = None
        fo = report["cases"]["fixed_only"]
        need_area = fix[fam]["courtyard_area_of_the_eight_headers_mm2"] + fo["courtyard_area_of_the_greedy_parts_mm2"]
        area_short = round(need_area - fo["free_area_for_a_header_courtyard_mm2"], 1)
        verdict[fam] = {
            "fits_on_the_final_placement": fin[fam]["all_eight_fit_in_greedy_packing"],
            "final_2x3_legal_origins": fin[fam]["kinds"]["2x3"]["legal_origins_total"],
            "final_1x2_legal_origins": fin[fam]["kinds"]["1x2"]["legal_origins_total"],
            "fixed_only_2x3_upper_bound": fix[fam]["kinds"]["2x3"]["upper_bound_fit_at_once"],
            "fixed_only_1x2_upper_bound": fix[fam]["kinds"]["1x2"]["upper_bound_fit_at_once"],
            "fixed_only_best_greedy": [fix[fam]["best_greedy_packing"]["placed_2x3"], fix[fam]["best_greedy_packing"]["placed_1x2"]],
            "impossible_even_with_every_greedy_part_removed": fix[fam]["impossible_by_upper_bound"],
            "impossible_even_without_the_plug_body_rule": fix[fam]["impossible_by_courtyard_only_bound"],
            "fixed_only_2x3_upper_bound_courtyard_only": fix[fam]["kinds"]["2x3"]["upper_bound_courtyard_only"],
            "fixed_only_1x2_upper_bound_courtyard_only": fix[fam]["kinds"]["1x2"]["upper_bound_courtyard_only"],
            "area_needed_headers_plus_greedy_parts_mm2": round(need_area, 1),
            "area_free_with_fixed_parts_only_mm2": fo["free_area_for_a_header_courtyard_mm2"],
            "area_shortfall_mm2": area_short,
            "without_power_block_2x3_upper_bound": report["cases"]["fixed_without_power_block"]["families"][fam]["kinds"]["2x3"]["upper_bound_fit_at_once"],
            "without_power_block_2x3_upper_bound_courtyard_only": report["cases"]["fixed_without_power_block"]["families"][fam]["kinds"]["2x3"]["upper_bound_courtyard_only"],
            "without_power_block_best_greedy": [report["cases"]["fixed_without_power_block"]["families"][fam]["best_greedy_packing"]["placed_2x3"],
                                                report["cases"]["fixed_without_power_block"]["families"][fam]["best_greedy_packing"]["placed_1x2"]],
            "area_note": "information only, not a proof: greedy parts may also stand under module bodies, where no header may stand",
        }
    report["verdict_per_family"] = verdict
    possible = [fam for fam in CANDIDATES if verdict[fam]["fits_on_the_final_placement"]]
    open_fams = [fam for fam in CANDIDATES if not verdict[fam]["fits_on_the_final_placement"]
                 and not verdict[fam]["impossible_even_with_every_greedy_part_removed"]]
    report["option"] = "A"
    report["option_B_search_result"] = ("legal set found on the final placement for: " + ", ".join(possible)) if possible else (
        "no legal set of eight headers on the final placement" + ("; not excluded by the bounds for: " + ", ".join(open_fams) + " (needs a full re-placement trial)" if open_fams else
                                                                   "; excluded for every family by the fixed-parts-only bound (no room for five 2x3 headers even with every greedy passive taken off the board)"))
    report["bare_holes_option_A"] = {r: {"x": pl.placed[r]["x"], "y": pl.placed[r]["y"], "rot": pl.placed[r]["rot"], "courtyard_kicad": cy[r].as_list()} for r in common.IN_BAND}
    report["picture_inputs"] = {
        "outline_kicad": [BX0, BY0, BX1, BY1],
        "outline_polygon_kicad": [[round(x, 3), round(y, 3)] for x, y in outline.polygon(8)],
        "courtyards": {r: {"rect": cy[r].as_list(), "how": pl.placed[r]["how"], "height_mm": pl.height(r)} for r in sorted(pl.placed, key=place.common_key)},
    }
    out = common.OUT_DIR / "reports" / "programming_access.json"
    out.write_bytes((json.dumps(report, indent=1) + "\n").encode("utf-8"))
    print(json.dumps(verdict, indent=1))
    print(report["option_B_search_result"])
    for case in report["cases"]:
        for fam in CANDIDATES:
            f = report["cases"][case]["families"][fam]
            print(case, fam, {k: (v["legal_origins_total"], v["upper_bound_fit_at_once"]) for k, v in f["kinds"].items()}, "greedy", f["greedy_packings"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
