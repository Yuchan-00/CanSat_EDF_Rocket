"""v0.9 provisional print files + slicer part masses for EVERY printed part, made from the CadQuery twin's STLs.

Invariant: a printed part's mass in the CAD report (printer.mass_model 'slicer_part_g') is the grams Bambu Studio's own
slice extrudes for THAT part (G-code extrusion minus supports / brim / skirt / prime), sliced from the current STL with the
part's print class (printer.part_class -> printer.print_classes: walls, top / bottom layers, infill).  Nothing here is
estimated: every number is read from the slicer's G-code / result; a part that does not slice is listed as failed and keeps
the class fallback density in the report (build_rocket.py flags it).

Why provisional (as v0.8 / v0.8.1): the official print set (cad/exports/print, tools/make_print_files.py) is made only from
a Fusion 360 run of the current params.json and script; that run needs the Fusion GUI.  Until it exists this writes a
separate set - cad/exports/print/provisional_v09_cadquery/ - from cad/exports/cadquery/stl (the report must be from the
current params.json), oriented and sliced with tools/make_print_files.py's own orient / slice_one /
slice_intake_with_blockers (P2S 0.4, 0.20 mm Standard, Bambu PLA Matte at printer.density_g_cm3), plus:

* the class settings per part (v0.9 print-mass calibration, docs/print_mass_calibration.json; top_shell_thickness 0 so
  the layer counts rule, except class D which keeps the v0.6 plan's process exactly),
* no supports for the propulsion module (v0.9: the CLI refused tree(auto) on it and normal(auto) filled the closed
  servo pockets, +73 g; the pockets' ceilings now bridge onto the service-side lintel) and normal(auto) for the payload
  module (the CLI refuses tree(auto) on it since v0.8),
* the G-code parsed per ; FEATURE: block: part grams, removed grams, the E-weighted extrusion centroid height
  (converted to the design frame for upright and flipped parts).

Outputs (cad/exports/print/provisional_v09_cadquery/): <part>.stl / .3mf (oriented / sliced project), slicer_part_masses.json
(the table build_rocket.py reads: parts.<base>.part_g + solid_volume_mm3 of the exact solid it was sliced from),
print_plan_provisional.json / .md, slice_log.txt.  Then rebuild the twin report (build_rocket.py --no-export) so the
masses come from the table.

Usage: python cad/cadquery/make_v09_print_files.py [--parts a,b] [--no-slice]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import sys
import tempfile
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import make_print_files as mpf  # noqa: E402
import trimesh  # noqa: E402

SRC = os.path.join(ROOT, "cad", "exports", "cadquery", "stl")
OUT = os.path.join(ROOT, "cad", "exports", "print", "provisional_v09_cadquery")
CQ_REPORT = os.path.join(ROOT, "cad", "exports", "cadquery_report.json")
NON_PART_FEATURES = ("Support", "Brim", "Skirt", "Custom", "Prime", "none")

# orientation (vehicle frame -> print frame) and supports: make_print_files.PARTS, plus the v0.8 lid and the v0.9 parts
PARTS = dict(mpf.PARTS)
PARTS["propulsion_module"] = (1, [[("z", 45.0)]], False,
                              "upright, exhaust exit on the bed, turned 45 deg on the plate (v0.9: the fins on the axes span "
                              "246.8 mm; axis-aligned the P2S plate refuses it - its wrap-detection zone - so the fins print "
                              "diagonally, 176.2 mm); no support: the fin-root fairings' keels are 45 deg, each servo pocket's "
                              "ceiling bridges its width onto the service-side lintel and the lintel bridges the opening "
                              "length (40 mm); the lead pipes rise at 33 deg from vertical")
PARTS["gps_pod_lid"] = (1, [[("x", 180.0)]], False, "flat: lid top face on the bed, the three ground-plane posts up")
PARTS["vane_coupler"] = (4, [[("y", 90.0)]], False, "upright, flange on the bed, D-bore vertical")
PARTS["servo_cover"] = (4, [[("x", 90.0)]], False, "flat")
PARTS["wire_cover"] = (1, [[("y", 90.0)]], False, "flat")
# v0.9 CAD-2 VTOL: the top cap prints top face down (skirt, ribs, hub and rim ring up: no overhang); the windowed avionics
# module keeps make_print_files' "upright", no support (the windows' 45 deg pointed tops need none)
PARTS["top_cap"] = (1, [[("x", 180.0)]], False, "flat: plate top face on the bed, skirt / ribs / hub / rim ring up")


def config_paths(config: str | None) -> tuple[str, str, str, str]:
    """v0.9 CAD-2: (STL source dir, output dir, CAD report, config) of a hardware configuration ('rocket' = the v0.9 set)."""
    cfg = config or mpf.layout.load_params()["mission"].get("config") or "rocket"
    sfx = mpf.layout.config_suffix(cfg)
    return (os.path.join(ROOT, "cad", "exports", "cadquery", f"stl{sfx}"),
            os.path.join(ROOT, "cad", "exports", "print", f"provisional_v09_cadquery{sfx}"),
            os.path.join(ROOT, "cad", "exports", f"cadquery_report{sfx}.json"), cfg)
# the CLI refuses tree(auto) on the payload module (v0.8) and on the v0.9 propulsion module ("One of the plate is empty
# or has no object fully inside it", also seen in the v0.9 mass calibration for a supported propulsion slice): normal(auto)
SUPPORT_OVERRIDES = {"payload_module": {"support_type": "normal(auto)"},
                     "propulsion_module": {"support_type": "normal(auto)"}}


def sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def class_overrides(pr: dict, base: str) -> tuple[str, dict, str]:
    """(class name, slicer overrides, settings text) of a printed part's print class."""
    cls = pr["part_class"][base]
    c = pr["print_classes"][cls]
    ov = {"wall_loops": str(c["wall_loops"]), "top_shell_layers": str(c["top_shell_layers"]),
          "bottom_shell_layers": str(c["bottom_shell_layers"]),
          "sparse_infill_density": f"{c['sparse_infill_density_pct']}%", "sparse_infill_pattern": c["sparse_infill_pattern"]}
    if cls != "D_small_keep":
        ov["top_shell_thickness"] = "0"          # the layer counts rule (masscal sets); D keeps the v0.6 plan process
    txt = (f"{c['wall_loops']} walls, {c['top_shell_layers']} top / {c['bottom_shell_layers']} bottom layers, "
           f"{c['sparse_infill_density_pct']} % {c['sparse_infill_pattern']}")
    return cls, ov, txt


def parse_gcode(path: str, rho: float) -> dict:
    """Extrusion per ; FEATURE: block (relative E): part grams (no supports / brim / skirt / prime), removed grams and the
    E-weighted centroid height of the part's own extrusion (layer mid-plane, plate frame)."""
    feat_mm: dict[str, float] = {}
    feat, rel, z = "none", True, 0.0
    part_e = part_ez = 0.0
    lh = 0.2
    num, znum = re.compile(r"E(-?[0-9.]+)"), re.compile(r"Z(-?[0-9.]+)")
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith(";"):
                if line.startswith("; FEATURE:"):
                    feat = line.split(":", 1)[1].strip()
                m = re.match(r"; layer_height = ([0-9.]+)", line)
                if m:
                    lh = float(m.group(1))
                continue
            if line.startswith("M82"):
                rel = False
            elif line.startswith("M83"):
                rel = True
            if line.startswith(("G1 ", "G2 ", "G3 ", "G0 ")):
                body = line.split(";", 1)[0]
                mz = znum.search(body)
                if mz:
                    z = float(mz.group(1))
                if "X" not in body and "Y" not in body:
                    continue
                m = num.search(body)
                if m:
                    e = float(m.group(1))
                    if e > 0 and rel:
                        feat_mm[feat] = feat_mm.get(feat, 0.0) + e
                        if not any(s in feat for s in NON_PART_FEATURES):
                            part_e += e
                            part_ez += e * z
    a = math.pi / 4 * mpf.FILAMENT_D_MM ** 2
    feat_g = {k: v * a * rho / 1000.0 for k, v in sorted(feat_mm.items(), key=lambda kv: -kv[1])}
    return {"feature_g": feat_g, "relative_e": rel, "part_g": part_e * a * rho / 1000.0,
            "removed_g": sum(v for k, v in feat_g.items() if any(s in k for s in NON_PART_FEATURES)),
            "part_z_centroid_plate_mm": (part_ez / part_e - lh / 2) if part_e > 0 else None}


def two_colour_slice(L, white, inlay, names: tuple[str, str], tmp: str, rho: float, extra: dict) -> dict:
    """livery: the avionics module as one 2-filament Bambu object (make_print_files.slice_two_colour) with the params
    colours and the params flush pair (livery.ams); adds the per-tool part centroids under the private key '_zc'."""
    lv = L.p["livery"]
    fl = lv["ams"]["flush_volumes_mm3"]
    res = mpf.slice_two_colour(white, inlay, names, tmp, rho, extra,
                               (L.livery_colours["body"]["hex"], L.livery_colours["logo"]["hex"]),
                               (float(fl["white_to_orange"]), float(fl["orange_to_white"])),
                               inlay_depth_mm=float(L.livery_depth))
    if res.get("ok"):
        res["_zc"] = (res["runs"]["two_colour"].get("part_z_centroid_plate_mm_by_tool") or {})
    return res


def two_colour_summary(entry: dict) -> dict:
    """The 2-colour slice in a few numbers (all M, Bambu Studio CLI): time, grams per colour, purge, tower, swaps, the
    single-colour control and the CLI-default-flush comparison."""
    add = entry["added_vs_single_colour"]
    k_sel = next(k for k in add if not k.endswith("_cli_default"))
    k_def = next((k for k in add if k.endswith("_cli_default")), None)
    return {"time_s": entry["time_s"], "swaps": entry["swaps"], "swaps_by_direction": entry["swaps_by_direction"],
            "swap_z_range_plate_mm": entry["swap_z_range_plate_mm"],
            "filament_g_by_colour": entry["filament_g_by_colour"],
            "part_g_white": entry["part_g_by_tool"]["0"], "part_g_orange": entry["part_g_by_tool"]["1"],
            "purge_g_by_tool": entry["purge_g_by_tool"], "prime_tower_g_by_tool": entry["prime_tower_g_by_tool"],
            "purge_g": sum(entry["purge_g_by_tool"].values()), "prime_tower_g": sum(entry["prime_tower_g_by_tool"].values()),
            "flush_volumes_mm3": entry["flush_volumes_mm3"], "added_time_s": add[k_sel]["time_s"],
            "added_filament_g": add[k_sel]["filament_g"], "cli_default_flush": add.get(k_def) if k_def else None,
            "single_colour_control": entry["single_colour_control"], "orange_features_g": entry["orange_features_g"],
            "settings_in_project": entry["settings_in_project"],
            "inlay_print_check": entry.get("inlay_print_check"), "mass_basis": entry.get("mass_basis")}


def consistent_module_grams(pipeline_plain_g: float, two: dict) -> dict:
    """reviewfix F3 (2026-11-04, independent review): the module's grams on ONE slicer basis.  Every other part's table
    grams come from the pipeline slice (slice_one); the 2-colour project is sliced with the RESOLVED P2S profile, and the
    pipeline's white-body slice differs from the resolved one (arachne vs classic walls fill the pocket differently), so
    'pipeline white + resolved orange' made the livery delta a settings artefact (+0.22 / +0.16 g).  Here (all M):
    total = the PLAIN module (white body + inlay, both on one filament) sliced by the pipeline + the livery delta measured
    inside the resolved slices (2-colour white + orange - the single-colour control of the same geometry); the inlay part
    = the resolved orange grams; the white body = total - inlay."""
    w, o = float(two["part_g_by_tool"]["0"]), float(two["part_g_by_tool"]["1"])
    ctrl = float(two["single_colour_control"]["part_g"])
    delta = w + o - ctrl
    total = pipeline_plain_g + delta
    return {"pipeline_plain_module_g": pipeline_plain_g, "resolved_white_g": w, "resolved_orange_g": o,
            "resolved_single_colour_control_g": ctrl, "livery_delta_resolved_g": delta, "module_total_g": total,
            "avionics_module_g": total - o, "inlay_g": o,
            "how": "total = pipeline slice of the plain module (white + inlay on filament 1) + (resolved 2-colour white + "
                   "orange - resolved single-colour control); inlay = resolved orange; white = total - inlay (all M)"}


SPARE_FRACTION = 0.10       # purchase spare per colour (failed prints), as tools/budget_bom.py's PLA line


def part_colour(pr: dict, name: str) -> dict:
    """The filament colour of a printed part: printer.color_exceptions (the launch pad stays black) else printer.color."""
    ex = (pr.get("color_exceptions") or {}).get(name)
    if ex:
        return {"name": ex.get("bambu_name", ex.get("color")), "hex": ex.get("hex"), "code": ex.get("code")}
    return {"name": pr.get("color"), "hex": pr.get("color_hex"), "code": pr.get("color_code")}


def colour_totals(pr: dict, L, ok: list) -> dict:
    """livery: slicer filament grams per colour (part + support + brim + purge + prime tower, M) over the plan, the purge
    and tower of the 2-colour part, and the purchase per colour (+ SPARE_FRACTION, 1 kg rolls)."""
    import math as _m
    by: dict[str, dict] = {}
    for n, v in ok:
        if v.get("two_colour"):
            names = {L.livery_colours["body"]["hex"]: L.livery_colours["body"], L.livery_colours["logo"]["hex"]:
                     L.livery_colours["logo"]}
            for hx, g in v["filament_g_by_colour"].items():
                c = names.get(hx, {"name": hx, "hex": hx, "code": None})
                e = by.setdefault(c["name"], {"hex": c["hex"], "code": c.get("code"), "filament_g": 0.0, "parts": []})
                e["filament_g"] += g * v["quantity"]
                e["parts"].append(n)
            continue
        c = part_colour(pr, n)
        e = by.setdefault(c["name"], {"hex": c["hex"], "code": c.get("code"), "filament_g": 0.0, "parts": []})
        e["filament_g"] += v["filament_g"] * v["quantity"]
        e["parts"].append(n)
    for e in by.values():
        e["with_spare_g"] = e["filament_g"] * (1.0 + SPARE_FRACTION)
        e["rolls_1kg"] = _m.ceil(e["with_spare_g"] / 1000.0)
    two = [v for _, v in ok if v.get("two_colour")]
    return {"filament_g_by_colour": by, "spare_fraction": SPARE_FRACTION,
            "two_colour_waste_g": {"purge": sum(sum(v["purge_g_by_tool"].values()) for v in two),
                                   "prime_tower": sum(sum(v["prime_tower_g_by_tool"].values()) for v in two)}}


def colour_lines(plan: dict, L) -> list[str]:
    """Korean summary of the 2-colour part and the per-colour filament totals for the print plan."""
    t = plan["totals"]
    out = ["## 색별 필라멘트 (슬라이서 M, 퍼지·프라임 타워·서포트 포함)", "",
           "| 색 (Bambu PLA Matte) | 코드 | 색값 | 슬라이스 g | 예비 10 % 포함 g | 1 kg 롤 | 부품 |", "|---|---|---|---|---|---|---|"]
    for nm, e in t["filament_g_by_colour"].items():
        out.append(f"| {nm} | {e.get('code') or '-'} | {e.get('hex') or '-'} | {e['filament_g']:.0f} | {e['with_spare_g']:.0f} | "
                   f"{e['rolls_1kg']} | {', '.join(e['parts'])} |")
    for n, v in plan["parts"].items():
        if not (v.get("ok") and v.get("two_colour")):
            continue
        s = two_colour_summary(v)
        d = s.get("cli_default_flush") or {}
        c = s["single_colour_control"]
        out += ["", f"## 2색 부품: {n} + {v['print_file_stl_inlay'][:-4]} (한 출력물, 필라멘트 2개)", "",
                f"* AMS 슬롯: 1번 {L.livery_colours['body']['name']} ({L.livery_colours['body']['hex']}, 코드 "
                f"{L.livery_colours['body']['code']}), 2번 {L.livery_colours['logo']['name']} ({L.livery_colours['logo']['hex']}, "
                f"코드 {L.livery_colours['logo']['code']}). 출력 파일 `{v.get('print_file', '-')}` (색 지정이 들어 있는 Bambu 프로젝트).",
                f"* 퍼지량(플러시): 흰→주황 {s['flush_volumes_mm3']['white_to_orange']:g} mm³, 주황→흰 "
                f"{s['flush_volumes_mm3']['orange_to_white']:g} mm³ (**A, 가정값**: Bambu Studio 측정표에서 가장 가까운 색 쌍 "
                f"연회색 #D1D3D5 ↔ 주황 #FF6A13; 흰 #FFFFFF / Mandarin #F99963 쌍은 표에 없음). 프라임 타워 켬. "
                f"얇은 벽 감지(detect_thin_wall) 켬 - 주황 한 줄이 한 번만 지나가게 합니다.",
                f"* 슬라이스(M, 위 퍼지 A 기준): 시간 {mpf.fmt_t(s['time_s'])} (한 색일 때 {mpf.fmt_t(c['time_s'])}, **+{mpf.fmt_t(s['added_time_s'])}**), "
                f"색 바꿈 **{s['swaps']}번** (출력 높이 {s['swap_z_range_plate_mm'][0]:.1f}-{s['swap_z_range_plate_mm'][1]:.1f} mm).",
                f"* 무게(M, 2색 슬라이스, 퍼지 A 기준): 흰 부품 {s['part_g_white']:.2f} g, 주황 부품 {s['part_g_orange']:.2f} g, 퍼지 {s['purge_g']:.1f} g, "
                f"프라임 타워 {s['prime_tower_g']:.1f} g; 필라멘트 흰 {s['filament_g_by_colour'].get(L.livery_colours['body']['hex'], 0):.1f} g, "
                f"주황 {s['filament_g_by_colour'].get(L.livery_colours['logo']['hex'], 0):.1f} g (한 색일 때 {c['filament_g']:.1f} g).",
                (f"* 비교(M): 표 없이 CLI 기본 퍼지 280 mm³이면 +{mpf.fmt_t(d['time_s'])}, 퍼지 {d['purge_g']:.1f} g, 타워 "
                 f"{d['prime_tower_g']:.1f} g (위쪽 기준값)." if d else "* 비교: CLI 기본 퍼지 슬라이스 없음."),
                "* 주황이 들어간 출력 경로(M, 필라멘트 2): " + ", ".join(f"{k.split('|')[0]} {g_:.2f} g"
                                                               for k, g_ in s["orange_features_g"].items()
                                                               if g_ > 0.005 and not k.startswith("_")) + ". 부품에서는 바깥 벽(Outer wall)만 주황입니다."]
        ic = s.get("inlay_print_check") or {}
        if ic:
            pl = ic.get("per_layer_info") or {}
            dabs = pl.get(f"dab_layers_path_lt_{mpf.DAB_PATH_MM:g}_mm") or []
            out.append(f"* 주황 한 번 지나가기 검사(M, G-code): **{ic['status']}** - 주황 경로 {ic['orange_part_path_mm']:.0f} mm, "
                       f"한 번 지나갈 때 {ic['single_pass_path_mm']:.0f} mm (비 {ic['path_ratio']:.3f}, 기준 ≤ {ic['path_ratio_max']}); "
                       f"주황 {ic['orange_part_g']:.3f} g, 고체 부피 × 밀도 {ic['solid_g']:.3f} g ({100 * ic['g_rel']:+.1f} %, 기준 ±"
                       f"{100 * ic['g_rel_tol']:.0f} %). 정보: 주황이 찍히는 층 {pl.get('layers_with_part')}개, 주황 없이 색만 바꾸는 층 "
                       f"{pl.get('empty_layers')}개 (출력 높이 {', '.join(f'{z_ - 0.1:.1f}' for z_ in pl.get('empty_layer_z_mm') or [])} mm), 0.4 mm보다 짧게 찍히는 층 {len(dabs)}개.")
        mb = v.get("mass_basis") or {}
        if mb:
            out.append(f"* 질량 표(`slicer_part_masses.json`, 리뷰 F3): 다른 부품과 같은 기준으로 맞췄습니다. 로고를 채운 원래 모듈을 "
                       f"파이프라인 설정으로 슬라이스한 값 {mb['pipeline_plain_module_g']:.2f} g + 2색 슬라이스 안에서 잰 로고 차이 "
                       f"{mb['livery_delta_resolved_g']:+.3f} g = 모듈 합계 {mb['module_total_g']:.2f} g. 주황 {mb['inlay_g']:.3f} g, "
                       f"흰 몸통 {mb['avionics_module_g']:.2f} g (모두 M).")
        out += ["* 시간 합계: 위 표의 모듈 시간은 2색 프로젝트(P2S 프로필을 상속까지 푼 설정)의 시간이고, 다른 부품은 기존 파이프라인 "
                "설정의 시간입니다. 그래서 출력 계획 합계의 차이는 위의 '한 색일 때 대비 +' 값과 같지 않습니다(두 설정의 한 색 시간이 다름).",
                "* 이 2색 프로젝트는 Bambu Studio의 P2S 시스템 프로필을 풀어서(상속 포함) 썼습니다 - 사용자가 여는 그대로입니다. "
                "3MF는 매번 바이트까지 같지는 않습니다(부품 uuid, 예상 시간 몇 초). 비교는 해시가 아니라 G-code 숫자(색 바꿈, 색별 g)로 합니다."]
    return out + [""]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--parts", default=None, help="comma list (default: every printed part of the report)")
    ap.add_argument("--no-slice", action="store_true")
    ap.add_argument("--config", default=None, help="v0.9 CAD-2 hardware configuration (rocket | vtol)")
    a = ap.parse_args(argv)
    global SRC, OUT, CQ_REPORT
    SRC, OUT, CQ_REPORT, cfg = config_paths(a.config)
    with open(CQ_REPORT, "r", encoding="utf-8") as f:
        rep = json.load(f)
    params_sha = sha256(os.path.join(ROOT, "params.json"))
    if rep.get("params_sha256") != params_sha:
        print(f"REFUSED: {os.path.relpath(CQ_REPORT, ROOT)} is not from the current params.json - rebuild the twin first "
              "(python cad/cadquery/build_rocket.py" + (f" --config {cfg})" if cfg != "rocket" else ")"))
        return 2
    L = mpf.layout.build() if cfg == "rocket" else mpf.layout.build(config=cfg)
    pr = L.p["printer"]
    rho = float(pr["density_g_cm3"])
    stl_files = rep["extra"].get("stl_files") or {}
    if not stl_files:
        # v0.9 CAD-2: a report written by a --no-export build lists no STL; slicing "every part" of it used to slice nothing
        # and overwrite the table with an empty one (found when re-slicing the VTOL set)
        print(f"REFUSED: {os.path.relpath(CQ_REPORT, ROOT)} lists no STL files (written by a --no-export build) - rebuild "
              "with the STL export first (python cad/cadquery/build_rocket.py" + (f" --config {cfg})" if cfg != "rocket" else ")"))
        return 2
    # exact solid volume and quantity per base part, from the report (first instance; launch pad counted once)
    vol, qty = {}, {}
    for name, e in rep["parts"].items():
        if e["kind"] != "printed":
            continue
        base = re.sub(r"_\d+$", "", name) if re.sub(r"_\d+$", "", name) in stl_files else name
        vol.setdefault(base, e["volume_mm3"])
        qty[base] = qty.get(base, 0) + 1
    names = [n for n in (a.parts.split(",") if a.parts else list(stl_files)) if n]
    os.makedirs(OUT, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix="cansat_v09_slice_")
    log: list[str] = [f"v0.9 provisional slices {time.strftime('%Y-%m-%d %H:%M')} - Bambu Studio CLI {mpf.BAMBU}",
                      f"params.json sha256 {params_sha}, preset {L.p['meta']['preset']}, filament density {rho}"]
    table = {"meta": {"params_sha256": params_sha, "preset": L.p["meta"]["preset"], "density_g_cm3": rho,
                      **({"config": cfg} if cfg != "rocket" else {}),
                      "source": f"{os.path.relpath(SRC, ROOT).replace(os.sep, '/')} (CadQuery twin of this params.json)",
                      "slicer": f"{mpf.MACHINE} / {mpf.PROCESS} / {mpf.FILAMENT} + class overrides",
                      "part_g": "the part's own extrusion (G-code features minus Support*, Brim, Skirt, Custom / prime)",
                      "date": time.strftime("%Y-%m-%d %H:%M")}, "parts": {}}
    plan = {"provisional": True, "version": "v0.9",
            "source": {"kind": "CadQuery twin STL (cad/exports/cadquery/stl) - NOT the Fusion 360 path",
                       "why": "make_print_files.py makes print files only from a Fusion run of the current params.json and "
                              "script; the v0.9 Fusion GUI run has not been made",
                       "replaced_by": "python tools/make_print_files.py after the Fusion run (then delete this folder)",
                       "params_sha256": params_sha, "preset": L.p["meta"]["preset"], "stl_sha256": {}},
            "machine": mpf.MACHINE, "process_base": mpf.PROCESS, "filament": mpf.FILAMENT, "parts": {}}
    old = {}
    tpath = os.path.join(OUT, "slicer_part_masses.json")
    if a.parts and os.path.exists(tpath):          # partial run: keep the other parts' entries
        with open(tpath, "r", encoding="utf-8") as f:
            old = json.load(f).get("parts", {})
        table["parts"].update({k: v for k, v in old.items() if k not in names})
    try:
        inlay_name = getattr(L, "livery_inlay_part", None) if getattr(L, "has_livery", False) else None
        for name in names:
            if inlay_name and name == inlay_name:
                continue                          # livery: sliced WITH the avionics module (2-filament project)
            if name not in PARTS:
                log.append(f"{name}: no orientation rule - skipped")
                continue
            q, cands, sup, why = PARTS[name]
            if name == "propulsion_module":     # fins on the plate diagonals (rocket_geometry.plate_rot_z)
                rz = (45.0 - L.fin_thetas[0]) % 90.0
                cands = [[("z", rz)]] if rz else [[]]
            src = os.path.join(ROOT, stl_files[name])
            mesh = trimesh.load(src, force="mesh")
            plan["source"]["stl_sha256"][name] = sha256(src)
            rots = cands[0]
            m = mpf.orient(mesh, rots)
            cls, ov, txt = class_overrides(pr, name)
            extra = {**(mpf.PART_OVERRIDES.get(name) or {}), **SUPPORT_OVERRIDES.get(name, {}), **ov}
            entry = {"rotations": rots, "extents_mm": [round(float(x), 1) for x in m.extents], "class": cls,
                     "settings": txt, "overrides": extra}
            t0 = time.time()
            proj = None
            two = bool(inlay_name) and name == L.livery_part and inlay_name in stl_files
            if two:     # livery: the inlay gets the module's print transform (one object, two parts)
                inlay_src = os.path.join(ROOT, stl_files[inlay_name])
                inlay_mesh = trimesh.load(inlay_src, force="mesh")
                plan["source"]["stl_sha256"][inlay_name] = sha256(inlay_src)
                m_inlay = inlay_mesh.copy()
                m_inlay.apply_transform(mpf.orient_matrix(mesh, rots))
            if not a.no_slice and os.path.exists(mpf.BAMBU):
                if name == "intake_module" and L.has_window_gable:
                    res, proj = mpf.slice_intake_with_blockers(mesh, rots, L, tmp, rho, sup, extra)
                else:
                    p_ = os.path.join(tmp, f"{name}.stl")
                    m.export(p_)
                    res = mpf.slice_one(p_, sup, tmp, rho, extra)
                    proj = res.get("_project")
                entry.update({k: v for k, v in res.items() if not k.startswith("_")})
                if res.get("ok"):
                    g = parse_gcode(res["_gcode"], rho)
                    entry.update({k: g[k] for k in ("part_g", "removed_g", "feature_g", "part_z_centroid_plate_mm")})
                    zc = g["part_z_centroid_plate_mm"]
                    lo, hi = mesh.bounds[0][2], mesh.bounds[1][2]
                    upright = all(ax == "z" for ax, _ in rots)
                    entry["part_z_centroid_design_mm"] = (lo + zc if upright else hi - zc if rots == [("x", 180.0)]
                                                          else None) if zc is not None else None
                if res.get("ok") and two:
                    # livery: the white body above is sliced alone like every other part (its table grams stay in the
                    # pipeline's settings); the print file is the 2-filament project with the RESOLVED P2S profile -
                    # the orange part grams, time, swaps, purge and tower come from it (M)
                    r2 = two_colour_slice(L, m, m_inlay, (name, inlay_name), tmp, rho, extra)
                    # reviewfix F3: the plain module (white body + inlay on ONE filament) sliced by the pipeline, the
                    # basis of the module's table grams (consistent_module_grams)
                    proj_plain = os.path.join(tmp, f"{name}_plain_pipeline.3mf")
                    mpf.write_bambu_multi_project(proj_plain, name, [(m, name, 1), (m_inlay, inlay_name, 1)],
                                                  mpf.project_settings_of(res["_project"]))
                    rp = mpf.slice_one(proj_plain, sup, tmp, rho, extra)
                    gp = parse_gcode(rp["_gcode"], rho) if rp.get("ok") else None
                    if r2.get("ok") and gp is not None:
                        zc2 = r2.get("_zc") or {}
                        entry["two_colour"] = True
                        entry.update({k: v for k, v in r2.items() if not k.startswith("_") and k not in ("ok", "time_s",
                                                                                                   "filament_g")})
                        entry["single_colour_slice"] = {"time_s": entry["time_s"], "filament_g": entry["filament_g"],
                                                        "part_g": entry["part_g"],
                                                        "note": "the white body (with the pocket) sliced alone, pipeline "
                                                                "settings (slice_one) - information; the table grams are "
                                                                "mass_basis (reviewfix F3)"}
                        mb = consistent_module_grams(gp["part_g"], r2)
                        mb["pipeline_plain_module_time_s"] = rp.get("time_s")
                        entry["mass_basis"] = mb
                        entry["time_s"] = r2["time_s"]
                        entry["filament_g"] = r2["filament_g"]
                        entry["part_g"] = mb["avionics_module_g"]
                        entry["inlay_part_g"] = mb["inlay_g"]
                        entry["inlay_part_z_centroid_design_mm"] = (lo + zc2["1"]) if "1" in zc2 else None
                        entry["removed_g"] = entry["filament_g"] - entry["part_g"] - entry["inlay_part_g"]
                        proj = r2.get("_project")
                    elif r2.get("ok"):
                        entry.update({"ok": False, "error": f"pipeline slice of the plain module: {rp.get('error')}"})
                    else:
                        entry.update({"ok": False, "error": f"2-colour slice: {r2.get('error')}",
                                      "two_colour_failure": {k: v for k, v in r2.items() if not k.startswith("_")}})
            if proj and os.path.exists(proj):
                shutil.copyfile(proj, os.path.join(OUT, f"{name}.3mf"))
                entry["print_file"] = f"{name}.3mf"
            m.export(os.path.join(OUT, f"{name}.stl"))
            entry["print_file_stl"] = f"{name}.stl"
            if two:
                m_inlay.export(os.path.join(OUT, f"{inlay_name}.stl"))
                entry["print_file_stl_inlay"] = f"{inlay_name}.stl"
            used = mpf.used_support_type(name, sup, entry)
            plan["parts"][name] = {"quantity": qty.get(name, q), "supports": sup, "support_type_used": used, "why": why,
                                   **entry}
            if entry.get("ok"):
                table["parts"][name] = {"part_g": entry["part_g"], "filament_g": entry["filament_g"],
                                        "removed_g": entry["removed_g"], "solid_volume_mm3": vol[name],
                                        "solid_mass_g_at_rho": vol[name] * rho / 1000.0, "class": cls, "settings": txt,
                                        "stl": stl_files[name], "stl_sha256": plan["source"]["stl_sha256"][name],
                                        "part_z_centroid_design_mm": entry.get("part_z_centroid_design_mm"),
                                        "time_s": entry["time_s"], "support": used}
                if two:     # livery: per-filament part grams of the SAME 2-colour slice (tool 0 white, tool 1 orange)
                    summ = two_colour_summary(entry)
                    table["parts"][name].update({"filament": "1 (white)", "two_colour": summ})
                    table["parts"][inlay_name] = {
                        "part_g": entry["inlay_part_g"],
                        "filament_g": entry["filament_g_by_colour"][L.livery_colours["logo"]["hex"]],
                        "removed_g": None, "solid_volume_mm3": vol[inlay_name],
                        "solid_mass_g_at_rho": vol[inlay_name] * rho / 1000.0, "class": cls, "settings": txt,
                        "stl": stl_files[inlay_name], "stl_sha256": plan["source"]["stl_sha256"][inlay_name],
                        "part_z_centroid_design_mm": entry.get("inlay_part_z_centroid_design_mm"),
                        "time_s": 0.0, "support": used, "filament": "2 (orange)",
                        "note": f"printed with {name} (one 2-filament object); its time / purge / tower are in that entry",
                        "two_colour": summ}
            ws = entry.get("window_support")
            line = (f"{name}: {cls} ({txt}), {entry['extents_mm']}, "
                    + (f"part {entry['part_g']:.2f} g (solid {vol[name] * rho / 1000:.2f} g), filament {entry['filament_g']:.2f} g, "
                       f"{mpf.fmt_t(entry['time_s'])}, support {used}" if entry.get("ok") else str(entry.get("error", "not sliced")))
                    + (f", window support check {ws.get('status')}" if ws else "")
                    + (f"; 2 filaments: {inlay_name} {entry['inlay_part_g']:.2f} g, {entry['swaps']} swaps, purge "
                       f"{sum(entry['purge_g_by_tool'].values()):.1f} g, tower {sum(entry['prime_tower_g_by_tool'].values()):.1f} g, "
                       f"+{mpf.fmt_t(two_colour_summary(entry)['added_time_s'])} vs one colour"
                       if two and entry.get("ok") else "") + f" [{time.time() - t0:.0f} s]")
            log.append(line)
            print(line, flush=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    ok = [(n, v) for n, v in plan["parts"].items() if v.get("ok")]
    pg = lambda v: v["part_g"] + (v.get("inlay_part_g") or 0.0)      # noqa: E731  (livery: + the orange inlay)
    plan["totals"] = {"time_s": sum(v["time_s"] * v["quantity"] for _, v in ok),
                      "filament_g": sum(v["filament_g"] * v["quantity"] for _, v in ok),
                      "part_g": sum(pg(v) * v["quantity"] for _, v in ok),
                      "vehicle_part_g": sum(pg(v) * v["quantity"] for n, v in ok if n != "launch_pad")}
    plan["totals"].update(colour_totals(pr, L, ok))
    for path, obj in ((tpath, table), (os.path.join(OUT, "print_plan_provisional.json"), plan)):
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(obj, f, indent=2, ensure_ascii=False)
            f.write("\n")
    lines = ["# 임시 출력 계획 v0.9 (CadQuery 트윈 STL, 부품 등급별 설정)", "",
             "**임시본입니다.** 공식 출력 파일은 Fusion 360 실행 뒤 `tools/make_print_files.py`로 만듭니다. "
             "질량은 Bambu Studio가 이 STL을 등급 설정으로 슬라이스한 G-code의 부품 압출량(서포트·브림 제외)입니다.", "",
             f"프린터 `{mpf.MACHINE}`, 공정 `{mpf.PROCESS}`, 필라멘트 `{mpf.FILAMENT}` (밀도 {rho} g/cm³).", "",
             "| 부품 | 등급 | 설정 | 수량 | 출력 방향 | 서포트 | 1개 부품 g | 1개 필라멘트 g | 1개 시간 |",
             "|---|---|---|---|---|---|---|---|---|"]
    for n, v in plan["parts"].items():
        lines.append(f"| {n} | {v['class']} | {v['settings']} | {v['quantity']} | {v['why']} | "
                     f"{mpf.support_label(v['support_type_used'], (v.get('settings_in_gcode') or {}).get('support_style'))} | "
                     + (f"{v['part_g']:.1f} | {v['filament_g']:.1f} | {mpf.fmt_t(v['time_s'])} |" if v.get("ok") else
                        f"- | - | {v.get('error', '슬라이스 안 함')} |"))
        if v.get("ok") and v.get("two_colour"):        # livery: the orange inlay of the same object
            lines.append(f"| └ {v['print_file_stl_inlay'][:-4]} (주황 상감, 같은 출력물) | {v['class']} | 바깥 벽 1줄 | "
                         f"{v['quantity']} | 모듈과 함께 | - | {v['inlay_part_g']:.2f} | "
                         f"{v['filament_g_by_colour'].get(L.livery_colours['logo']['hex'], 0.0):.1f} | 모듈 시간에 포함 |")
    t = plan["totals"]
    total_line = (f"**합계 (수량 반영)**: {mpf.fmt_t(t['time_s'])}, 필라멘트 {t['filament_g']:.0f} g, 부품 {t['part_g']:.0f} g "
                  f"(발사대 제외 기체 부품 {t['vehicle_part_g']:.0f} g)")
    lines += ["", total_line, ""]
    lines += colour_lines(plan, L)
    with open(os.path.join(OUT, "print_plan_provisional.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
    log.append(total_line)
    with open(os.path.join(OUT, "slice_log.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(log) + "\n")
    print(total_line)
    fails = [n for n in names if n in plan["parts"] and not plan["parts"][n].get("ok")]
    # reviewfix F1: the 2-colour G-code must print the inlay as ONE outer-line pass of the drawn logo
    for n, v in plan["parts"].items():
        if v.get("two_colour"):
            ic = v.get("inlay_print_check") or {}
            print(f"{n}: inlay print check {ic.get('status')} - orange path {ic.get('orange_part_path_mm', 0):.0f} mm / "
                  f"single pass {ic.get('single_pass_path_mm', 0):.0f} mm = {ic.get('path_ratio', float('nan')):.3f} "
                  f"(<= {ic.get('path_ratio_max')}), orange {ic.get('orange_part_g', 0):.3f} g vs solid "
                  f"{ic.get('solid_g', 0):.3f} g ({100 * ic.get('g_rel', float('nan')):+.1f} %)")
            if ic.get("status") != "PASS":
                fails.append(f"{n} inlay print check")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
