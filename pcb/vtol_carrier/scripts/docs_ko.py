"""User documents of the rev 3 carrier: README_ko.md (board folder) and out/ORDER_ko.md (ordering guide).

Invariant: every number in the two documents is read from a generated file in this run (spec, plan, reports, BOM /
CPL / CSV files) through ``Facts``; the documents end with a table that names the file each number came from and its
label (M measured by a tool here, E estimate, A assumption, F from a document). A number that has no source file is
not written. Fixed wording (procedures, explanations) is Korean prose without numbers, or quotes the plan by section.
Nothing here edits the design or an order file. The output is deterministic for the same input files.

Run: cd <board>/scripts && B:/Claude/.venv/Scripts/python.exe -X utf8 -B docs_ko.py   (step 'docs' of make_all.py)
"""

from __future__ import annotations

import collections
import csv
import hashlib
import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "out"
REP = OUT / "reports"
R = ROOT.as_posix()

BLOCK_KO = {
    "PWR": "전원: 1S 로직 배터리 입력 J14, 역극성 보호 Q10, 스위치 리드 J15, 5 V 승압 U8 TPS61023 + L5, 저전압 차단 U9 TLV3012B, "
           "3.3 V 레귤레이터 U6, 접지 연결 NT1, 배터리 전압 분압",
    "NAV": "NAV 칩 U1 ATmega128A + GY-521 IMU 자리 J1 + BMP280 소켓 J2",
    "FCU": "FCU 칩 U2 ATmega128A",
    "COM": "COM 칩 U3 ATmega128A + ESP-12F Wi-Fi 모듈 U7, 레벨 시프터, 부트 스트랩, ESP 굽기 단자 J3",
    "GNS": "GNS 칩 U4 ATmega128A + GPS 리드 커넥터 J7 (NEO-7M)",
    "SHIFT": "BSS138 레벨 시프터 5 V <-> 3.3 V",
    "IO": "서보 J8~J11, ESC J12, 아밍 스위치 J16, 부저 J17, 6S 전압 감지 J18 등 입출력 커넥터",
    "LOG": "OpenLog 소켓 J5",
    "LINK": "칩 사이 연결선 (UART / SPI) 부품",
}
TAG_KO = [  # first matching prefix wins
    ("optional sourcing", "선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음)"),
    ("forced", "필수: 요청(모든 부품을 JLC가 실장) 때문에"),
    ("keeps a rev 2c function", "rev 2c 기능 유지: Pololu 모듈 안에 있던 저전압 차단 (D2에서 뺄 수 있음)"),
    ("sourcing", "조달: JLC 주문 방식·재고 때문에 (핀 호환 부품)"),
    ("unchanged", "변경 없음: 부품 없는 자리 (rev 2c와 같음)"),
]
ACTION_KO = {"replace": "교체", "add": "추가", "remove": "삭제", "keep-dnp": "그대로 (부품 없음)"}
FIT_KO = {"jlc": "JLC가 실장 (주문 BOM / CPL에 들어감)", "jlc_optional": "기본은 사용자 납땜, with_mcu 파일 쌍을 쓰면 JLC 실장",
          "user": "사용자가 납땜", "none": "부품 없음 (빈 도금 구멍 / 미실장 / 구리 패턴만)"}


def load(rel: str) -> dict:
    return json.loads((ROOT / rel).read_text(encoding="utf-8"))


def rows(rel: str) -> list[dict]:
    with open(ROOT / rel, encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def common_sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def p(rel: str) -> str:
    """absolute path in backticks (user documents name full paths)"""
    return f"`{R}/{rel}`"


class Facts:
    """key -> (text, source file, label); ``v(key)`` returns the text and marks the key as used in a document."""

    def __init__(self) -> None:
        self.data: dict[str, tuple[str, str, str]] = {}
        self.used: list[str] = []

    def put(self, key: str, value, source: str, label: str) -> None:
        if isinstance(value, float):
            value = f"{value:g}"
        elif isinstance(value, (list, tuple)):
            value = ", ".join(str(x) for x in value) if value else "없음"
        self.data[key] = (str(value), source, label)

    def v(self, key: str) -> str:
        if key not in self.data:
            raise KeyError(f"docs_ko: no fact {key!r} - a number without a source is not written")
        if key not in self.used:
            self.used.append(key)
        return self.data[key][0]

    def table(self, keys: list[str]) -> list[str]:
        out = ["| 값 이름 | 값 | 출처 파일 | 표시 |", "|---|---|---|---|"]
        for k in keys:
            t, s, lab = self.data[k]
            out.append(f"| {k} | {t} | {s} | {lab} |")
        return out


def gather() -> tuple[Facts, dict]:
    F = Facts()
    spec = load("spec/carrier_spec.json")
    plan = load("spec/rev3_plan.json")
    man = load("out/manifest.json")
    rr = load("out/reports/routing_rules.json")
    edge = load("out/reports/edge_distance.json")
    fit = load("out/fit_checks.json")
    rot = load("out/reports/cpl_rotation.json")
    gate = load("out/jlc_gate_report.json")
    prog = load("out/reports/programming_access.json")
    pin = load("out/reports/rev3_mcu_pinmap.json")
    u89 = load("out/reports/rev3_u8_u9_pads.json")
    ef = load("out/reports/rev3_export_files.json")
    ed = load("out/reports/rev3_export_determinism.json")
    reuse = load("kicad/work/route_reuse_log.json")
    dims = load("spec/module_dimensions.json")
    fees = rows("out/jlc_fee_estimate.csv")
    offb = rows("out/off_board_purchase.csv")
    bom = rows("out/jlcpcb_bom.csv")
    bom_mcu = rows("out/jlcpcb_bom_with_mcu.csv")
    det_path = REP / "determinism.json"
    det = json.loads(det_path.read_text(encoding="utf-8")) if det_path.is_file() else None

    S, M = "spec/carrier_spec.json", "out/manifest.json"
    s = spec["summary"]
    F.put("부품 자리", s["parts"], S, "M (스크립트 집계)")
    F.put("실장 부품(populated)", s["parts_populated"], S, "M")
    F.put("네트", s["nets"], S, "M")
    for k, n in s["fit_counts"].items():
        F.put(f"fit {k}", n, S, "M")
    b = spec["board"]
    F.put("외곽", f"{b['outline_mm']['size'][0]:g} x {b['outline_mm']['size'][1]:g} mm", S, b["outline_mm"]["label"])
    F.put("층 수", b["layers"], S, "F (계획 불변조건 d)")
    F.put("두께", f"{b['thickness_mm']:g} mm", S, "F")
    F.put("높이 한계", f"{dims['height_limit']['limit_above_board']['v']:g} mm", "spec/module_dimensions.json", "F (계획)")
    F.put("GY-521 받침 높이", f"{dims['gy521']['support_h']['v']:g} mm", "spec/module_dimensions.json gy521.support_h", "A")
    F.put("BMP280 헤더 종류", "직각" if dims["bmp280"]["header_type"]["v"] == "right_angle" else dims["bmp280"]["header_type"]["v"],
          "spec/module_dimensions.json bmp280.header_type", "A")
    F.put("프로그래밍 헤더 선택", spec["prog_headers"]["option"], S + " prog_headers.option", "M")

    pairs = man["bom_cpl"]["pairs"]
    for name, pr in pairs.items():
        for k in ("bom_lines", "bom_lines_smt", "bom_lines_tht", "parts", "cpl_rows"):
            F.put(f"{name} {k}", pr[k], M + f" bom_cpl.pairs.{name}", "M")
        F.put(f"{name} 회전 미확인", pr["orientation_critical_rotation_not_verified"], M, "NOT_VERIFIED 목록")
    F.put("BOM 줄(파일)", len(bom), "out/jlcpcb_bom.csv", "M")
    F.put("BOM 줄 with_mcu(파일)", len(bom_mcu), "out/jlcpcb_bom_with_mcu.csv", "M")
    for name, st in man["fab"]["sets"].items():
        F.put(f"zip {name} sha256", st["zip_sha256"][:16] + "...", M + " fab.sets", "M")
        F.put(f"F.Paste 패드 {name}", st["f_paste_flashes"], M + " fab.sets", "M")
    sha = man["sha256"]
    for rel in ("jlcpcb_bom.csv", "jlcpcb_cpl.csv", "jlcpcb_bom_with_mcu.csv", "jlcpcb_cpl_with_mcu.csv"):
        F.put(f"{rel} sha256", sha[rel][:16] + "...", M + " sha256", "M")
    F.put("ERC", f"오류 {man['erc']['errors']}, 경고 {man['erc']['warnings']} (시트 {man['erc']['sheets_checked']}장)",
          "out/reports/erc.json (kicad-cli " + man["erc"]["kicad_version"] + ")", "M")
    d = man["drc"]
    F.put("DRC", f"위반 {d['errors']}, 경고 {d['warnings']}, 미연결 {d['unconnected']}, 회로도 불일치 {d['parity_count']}",
          "out/reports/drc.json (kicad-cli " + d["kicad_version"] + ")", "M")
    F.put("넷리스트=사양", f"{man['netlist_vs_spec']['result']} (부품 {man['netlist_vs_spec']['components']}, 네트 "
          f"{man['netlist_vs_spec']['spec_nets']})", M + " netlist_vs_spec", "M")
    F.put("보드=IR", f"{man['board_vs_ir']['result']} (핀 {man['board_vs_ir']['pins_found_on_board']} / "
          f"{man['board_vs_ir']['ir_pins']})", M + " board_vs_ir", "M")
    F.put("MCU 핀맵", f"{pin['result']} (핀 {pin['pins_compared']}, 차이 {len(pin['differences'])})",
          "out/reports/rev3_mcu_pinmap.json", "M")
    F.put("U8/U9 패드표", u89["result"], "out/reports/rev3_u8_u9_pads.json", "M (표는 데이터시트에서 옮김, F)")
    F.put("주문 파일 검사", ef["result"], "out/reports/rev3_export_files.json", "M")
    F.put("주문 파일 재현", ed["result"], "out/reports/rev3_export_determinism.json", "M")
    bm = man["board_metrics"]
    F.put("구리 섬", "없음 (모든 네트 한 덩어리)" if bm["net_clusters_ok"] else "있음", M + " board_metrics", "M")
    F.put("패드에 닿는 비아", bm["vias_touching_pads"], M + " board_metrics", "M")
    # copper counts of the FINAL board file (rev 3 review E-R4 / MR-3: route_reuse_log.json holds the counts of the
    # Freerouting import stage, before fan-out, leftovers, rip-up and stitching)
    pcb_text = (ROOT / "kicad" / "vtol_carrier.kicad_pcb").read_text(encoding="utf-8")
    vias = collections.Counter(re.findall(r"\n\t\(via\n\t\t\(at [^)]*\)\n\t\t\(size ([\d.]+)\)\n\t\t\(drill ([\d.]+)\)", pcb_text))
    n_via = len(re.findall(r"\n\t\(via\n", pcb_text))
    if sum(vias.values()) != n_via:
        raise SystemExit(f"docs_ko: via parse mismatch {sum(vias.values())} vs {n_via}")
    BRD = "kicad/vtol_carrier.kicad_pcb"
    F.put("배선 세그먼트", len(re.findall(r"\n\t\(segment\n", pcb_text)), BRD + " (최종 보드 파일에서 셈)", "M")
    F.put("배선 원호", len(re.findall(r"\n\t\(arc\n", pcb_text)), BRD + " (최종 보드 파일에서 셈)", "M")
    F.put("배선 비아", n_via, BRD + " (최종 보드 파일에서 셈)", "M")
    F.put("비아 크기별", ", ".join(f"{s}/{d} mm {n}개" for (s, d), n in sorted(vias.items())), BRD + " (지름/드릴)", "M")
    F.put("보드 sha256", common_sha(ROOT / BRD)[:16] + "...", BRD, "M")
    F.put("배선 재생 기준 보드", reuse["routed_board_sha256"][:16] + "...", "kicad/work/route_reuse_log.json", "M")

    res = rr["results"]
    F.put("배선 규칙 PASS 수", sum(1 for x in res.values() if x == "PASS"), "out/reports/routing_rules.json", "M")
    F.put("배선 규칙 수", len(res), "out/reports/routing_rules.json", "M")
    F.put("배선 규칙 FAIL", [k for k, x in res.items() if x != "PASS"], "out/reports/routing_rules.json", "M")
    bp = rr["checks"]["battery_path"]
    F.put("배터리 경로 저항 20C", f"{bp['copper_resistance_ohm_20C']:g} Ohm", "out/reports/routing_rules.json battery_path", "E")
    F.put("배터리 경로 저항 60C", f"{bp['copper_resistance_ohm_60C']:g} Ohm", "out/reports/routing_rules.json battery_path", "E")
    F.put("배터리 경로 목표", f"{bp['target_ohm']:g} Ohm", "out/reports/routing_rules.json battery_path", "F (계획 13절)")
    F.put("2 mm보다 좁은 길이", f"{bp['current_carrying_length_narrower_than_2mm']:g} mm",
          "out/reports/routing_rules.json battery_path", "M")
    legs = []
    for leg in bp["legs"]:
        n = leg["narrower_than_2mm"]
        if n["length_mm"] > 0:
            legs.append(f"{leg['net']} {n['length_mm']:g} mm (폭 {'/'.join(f'{w:g}' for w in n['widths_mm'])} mm)")
    F.put("좁은 구간", "; ".join(legs), "out/reports/routing_rules.json battery_path.legs", "M")
    cr = rr["checks"]["crystals"]
    F.put("XTAL1 최악", f"{cr['XTAL1_worst_mm']:g} mm (한계 {cr['XTAL1_limit_mm']:g} mm)", "out/reports/routing_rules.json crystals", "M")
    F.put("규칙 파일", rr["checks"]["rule_file"]["result"], "out/reports/routing_rules.json rule_file", "M")
    # silk-id step: the two identification lines; the strings themselves are never repeated in a document
    idl = rr["checks"]["silkscreen"].get("identification_lines")
    if not idl or len(idl["lines"]) != 2:
        raise SystemExit("docs_ko: routing_rules.json has no identification_lines block with two lines - run routing_checks.py")
    side_ko = {"B.Silkscreen": "아랫면 실크(B.SilkS, 아랫면에서 볼 때 바로 읽히도록 좌우 반전)",
               "F.Silkscreen": "윗면 실크(F.SilkS)"}
    if len({(r_["layer"], r_["height_mm"], r_["stroke_mm"]) for r_ in idl["lines"]}) != 1 or idl["lines"][0]["layer"] not in side_ko:
        raise SystemExit("docs_ko: the identification lines differ in layer / size")
    F.put("식별 문구 위치", side_ko[idl["lines"][0]["layer"]], "out/reports/routing_rules.json silkscreen.identification_lines", "M")
    F.put("식별 문구 크기", f"글자 높이 {idl['lines'][0]['height_mm']:g} mm, 선 굵기 {idl['lines'][0]['stroke_mm']:g} mm",
          "out/reports/routing_rules.json silkscreen.identification_lines", "M")
    F.put("식별 문구 검사", "PASS" if idl["ok"] else "FAIL: " + "; ".join(idl["problems"]),
          "out/reports/routing_rules.json silkscreen.identification_lines", "M")
    F.put("식별 문구 간격", f"패드까지 {min(r_['gap_to_nearest_pad_mm'] for r_ in idl['lines']):g} mm 이상, 비아까지 "
          f"{min(r_['gap_to_nearest_via_pad_mm'] for r_ in idl['lines']):g} mm 이상, 기판 변까지 "
          f"{min(r_['gap_to_board_edge_mm'] for r_ in idl['lines']):g} mm 이상, 글자 밑 비아 "
          f"{sum(r_['vias_under_the_text'] for r_ in idl['lines'])}개, 케이블타이 띠와 겹침: "
          + ("있음" if any(r_["under_cable_tie_band"] for r_ in idl["lines"]) else "없음"),
          "out/reports/routing_rules.json silkscreen.identification_lines", "M")

    F.put("가장자리 2.5 mm 안 몸체", len(edge["bodies_under_2p5"]), "out/reports/edge_distance.json", "M")
    F.put("가장자리 0.3 mm 안 몸체", [f"{r_[0]} {r_[1]:g} mm" for r_ in edge["bodies_under_0p3"]],
          "out/reports/edge_distance.json", "M")
    pair = edge["rail_sides_for_standard_order"]["pair"]
    F.put("레일 쪽", pair, "out/reports/edge_distance.json rail_sides_for_standard_order", "E (표에서 고름)")
    F.put("레일 없는 쪽 2.5 mm 안", edge["rail_pairs"][pair]["count_without_rail"], "out/reports/edge_distance.json rail_pairs", "M")
    tl = fit["tht_leads"]
    F.put("뒷면 다리 규칙", f"{tl['rule_mm']:g} mm", "out/fit_checks.json tht_leads", "F (계획)")
    F.put("다리 규칙 초과", tl["over_rule_nominal"], "out/fit_checks.json tht_leads", "F (도면값)")
    F.put("다리 길이 미확인", tl["not_verified"], "out/fit_checks.json tht_leads", "NOT_VERIFIED")
    rf = fit["reflow_rating"]
    F.put("리플로 한계", [x.split(" ")[0] for x in rf["exceeds_or_at_limit"]], "out/fit_checks.json reflow_rating", "F (데이터시트)")
    F.put("리플로 미확인 줄 수", len(rf["not_verified"]), "out/fit_checks.json reflow_rating", "NOT_VERIFIED")
    # module keep-outs: the nominal case AND the conservative case (rev 3 review MR-1)
    mk = fit["module_keepouts"]
    nom_col, nom_near, con_col, con_near = [], [], [], []
    for mname, m in mk["modules"].items():
        for z in m.get("zones", []):
            for ref, h, c in z.get("collisions", []):
                nom_col.append(f"{ref} {c:g} mm ({mname} '{z['zone']}')")
            for ref, h, c in z.get("below_clearance_min", []):
                nom_near.append(ref)
            for ref, h, c in z.get("below_clearance_min_conservative", []):
                (con_col if c <= 0.0 else con_near).append(f"{ref} {c:g} mm" if c <= 0.0 else ref)
                if c <= 0.0:
                    con_col[-1] += f" ({mname} '{z['zone']}')"
    uniq = lambda xs: sorted(set(xs), key=lambda s: (s.rstrip("0123456789"), s))  # noqa: E731
    F.put("모듈 간섭", f"{mk['status']}, 가정 치수(보통 경우)로 충돌 {len(mk['collisions_under_stated_assumptions'])}건",
          "out/fit_checks.json module_keepouts", "A")
    F.put("모듈 간섭 보통 0.5 mm 미만", uniq(nom_near), "out/fit_checks.json module_keepouts.modules[].zones[].below_clearance_min", "A")
    F.put("모듈 간섭 보수적 충돌", con_col, "out/fit_checks.json module_keepouts.modules[].zones[].below_clearance_min_conservative (<= 0)", "A")
    F.put("모듈 간섭 보수적 0.5 mm 미만", uniq(con_near), "out/fit_checks.json module_keepouts.modules[].zones[].below_clearance_min_conservative", "A")
    F.put("모듈 최소 간격", f"{mk['clearance_min_mm']:g} mm", "out/fit_checks.json module_keepouts.clearance_min_mm", "F (계획)")
    # boost block layout checks added after the rev 3 electrical review (E-R1 / E-R2)
    hl, ic = rr["checks"].get("hot_loop"), rr["checks"].get("input_cap")
    if hl is None or ic is None:
        raise SystemExit("docs_ko: routing_rules.json has no hot_loop / input_cap check - run routing_checks.py")
    F.put("출력 커패시터 간격", "; ".join(f"{c['cap']} +5V 패드-U8.6 {c['gap_pad1_to_U8.6_mm']:g} mm, GND 패드-U8.4 {c['gap_pad2_to_U8.4_mm']:g} mm"
                                    for c in hl["caps"]), "out/reports/routing_rules.json hot_loop", "M")
    F.put("출력 커패시터 GND 윗면 연결", ", ".join(hl["output_cap_gnd_joined_to_U8.4_on_F.Cu"]) or "없음 (내층 GND와 비아로만 돌아감)",
          "out/reports/routing_rules.json hot_loop", "M")
    F.put("FB 선이 출력 커패시터 아래", f"{len(hl['information_boost_fb_under_output_cap'])}개 구간",
          "out/reports/routing_rules.json hot_loop.information_boost_fb_under_output_cap", "M")
    r41 = {(r["cap"], r["to"]): r for r in ic["rows"]}
    a, b_ = r41[("C41", "U8.3")], r41[("C41", "L5.1")]
    F.put("입력 커패시터 경로", f"C41 → U8 VIN: 패드 간격 {a['pad_edge_gap_mm']:g} mm, 구리 길이 {a['copper_path_mm']:g} mm (비아 {a['vias_on_path']}개); "
          f"C41 → L5 입력: 구리 길이 {b_['copper_path_mm']:g} mm", "out/reports/routing_rules.json input_cap", "M (길이는 배선 모델, E)")
    # relayout step checks (measured on the routed board): top-layer GND piece, input capacitor paths, FB, rail edges
    BOOST_KEYS = ("hot_loop", "input_cap", "hot_loop_top_gnd", "input_cap_vin", "input_cap_l5", "fb_clear_of_caps",
                  "sw_node", "crystal_distance")
    tg, iv, il = rr["checks"].get("hot_loop_top_gnd"), rr["checks"].get("input_cap_vin"), rr["checks"].get("input_cap_l5")
    fbc, rec = rr["checks"].get("fb_clear_of_caps"), rr["checks"].get("rail_edge_copper")
    if None in (tg, iv, il, fbc, rec):
        raise SystemExit("docs_ko: routing_rules.json lacks the relayout checks - run routing_checks.py")
    F.put("승압부 배치 FAIL", [k for k in BOOST_KEYS if res.get(k) != "PASS"], "out/reports/routing_rules.json", "M")
    F.put("승압부 배치 PASS", [k for k in BOOST_KEYS if res.get(k) == "PASS"], "out/reports/routing_rules.json", "M")
    F.put("윗면 GND 한 덩어리", f"{tg['result']} ({', '.join(tg['fcu_gnd_piece'])} 이 "
          + ("같은 윗면 GND 구리 한 덩어리 위에 있음" if tg["all_on_one_piece"] else "서로 다른 윗면 GND 구리 조각에 있음")
          + f"; 자기 비아가 있는 패드 {sum(1 for v in tg['pads_with_own_via'].values() if v)} / {len(tg['pads_with_own_via'])})",
          "out/reports/routing_rules.json hot_loop_top_gnd", "M")
    F.put("입력 커패시터", iv["input_capacitor"], "out/reports/routing_rules.json input_cap_vin.input_capacitor", "M")
    # review RR-7: the 5 mm limit is this pipeline's own reading of 'a few mm' (E), and no doubled PASS / FAIL prefix
    FEW = f"기준 {iv['limit_mm']:g} mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E"
    F.put("입력 커패시터 VIN 경로", f"{iv['input_capacitor']} → U8 VIN 구리 길이 {iv['copper_path_to_U8.3_mm']:g} mm "
          f"(비아 {iv['vias_on_path']}개; {FEW}), GND 패드가 U8 GND와 같은 윗면 구리: "
          + ("예" if iv["gnd_pad_on_the_U8.4_piece"] else "아니오"), "out/reports/routing_rules.json input_cap_vin", "M (길이는 배선 모델, E)")
    F.put("입력 커패시터 L5 경로", f"{il['input_capacitor']} → L5 입력 패드 구리 길이 {il['copper_path_to_L5.1_mm']:g} mm "
          f"({FEW}; U8 VIN 패드와 L5 입력 패드 사이 직선 거리 {il['pad_edge_distance_U8.3_to_L5.1_mm']:g} mm)",
          "out/reports/routing_rules.json input_cap_l5", "M (길이는 배선 모델, E)")
    for k_ in ("input_cap_vin", "input_cap_l5", "hot_loop", "boost_sw", "sw_node", "crystal_distance"):
        if k_ not in res:
            raise SystemExit(f"docs_ko: routing_rules.json has no check {k_} - run routing_checks.py")
        F.put(f"검사 {k_}", res[k_], "out/reports/routing_rules.json results", "M")
    F.put("FB 선과 출력 커패시터", f"{fbc['result']} (출력 커패시터 패드 사이를 지나는 FB 구리 {len(fbc['boost_fb_copper_inside'])}곳, "
          f"가장 가까운 간격 {fbc['min_gap_boost_fb_track_to_output_cap_pad_mm']:g} mm, FB 선 길이 {fbc['boost_fb_f_cu_track_length_mm']:g} mm)",
          "out/reports/routing_rules.json fb_clear_of_caps", "M")
    rmin = rec["min_copper_to_edge_mm"]
    rail_min = min(v for k, v in rmin.items() if "rail" in k)
    F.put("레일 변 구리 이격", f"{rec['result']}: 레일이 붙는 두 76 mm 변에서 {rail_min:g} mm 이상 (기준 {rec['limit_mm_rail_edges']:g} mm), "
          f"나머지 두 변 {min(v for k, v in rmin.items() if 'rail' not in k):g} mm (보드 규칙 0.3 mm)",
          "out/reports/routing_rules.json rail_edge_copper", "M")
    F.put("레일 변 구리 이격 값", f"{rail_min:g}", "out/reports/routing_rules.json rail_edge_copper", "M")
    sw = rr["checks"]["boost_sw"]
    F.put("SW 선", f"길이 {sw['track_length_mm']:g} mm, 폭 {'/'.join(f'{w:g}' for w in sw['widths_mm'])} mm, 비아 {sw['vias']}개, "
          + ("F.Cu만" if sw["layers"] == ["F.Cu"] else "층 " + "/".join(sw["layers"])),
          "out/reports/routing_rules.json boost_sw", "M")
    # relayout-fix step (review RR-1): the placement of the switch node and the distance to the NAV crystal are judged
    sn, cd = rr["checks"]["sw_node"], rr["checks"]["crystal_distance"]
    under = [u["ref"] for u in sn["parts_the_sw_track_passes_under"]]
    F.put("SW 노드", f"U8의 SW 패드와 L5의 SW 패드 사이 간격 {sn['pad_edge_gap_U8.5_L5.2_mm']:g} mm "
          f"(기준 {sn['limit_mm']:g} mm 이하 - 계획 12장의 'L5는 U8 핀 바로 옆'에 숫자가 없어 스크립트가 계획 12장의 2 mm를 빌려 읽은 값, E), "
          f"SW 선 구리 넓이 {sn['track_copper_area_mm2']:g} mm2, 구리 저항 약 {1000 * sn['track_resistance_ohm_20C_E']:.1f} mOhm (20 C, E), "
          "SW 선이 패드 사이를 지나는 다른 부품: " + (", ".join(under) if under else "없음"),
          "out/reports/routing_rules.json sw_node", "M (기준과 저항은 E)")
    r2c, swc = cd["rev2c_J6_pins_min_gap_mm"], cd["rev3_BOOST_SW_copper_min_gap_mm"]
    KY, KU, KX = "Y1 pads", "U1 XTAL pins 23 / 24", "NAV XTAL net copper (tracks, vias, pads)"
    near = {k: min(cd["rev3_parts_min_gap_mm"].items(), key=lambda kv: (kv[1][k], kv[0])) for k in (KY, KU)}
    F.put("크리스털 거리", f"rev 2c의 Pololu 핀은 NAV 크리스털 Y1 패드에서 {r2c[KY]:g} mm, U1의 XTAL 핀에서 {r2c[KU]:g} mm 떨어져 있었습니다. "
          f"지금은 가장 가까운 부품이 Y1 패드에서 {near[KY][1][KY]:g} mm ({near[KY][0]}), XTAL 핀에서 {near[KU][1][KU]:g} mm ({near[KU][0]}) 이고, "
          f"SW 구리(선과 두 패드)는 Y1 패드에서 {swc[KY]:g} mm, XTAL 핀에서 {swc[KU]:g} mm, 크리스털 배선 구리에서 {swc[KX]:g} mm "
          f"(rev 2c Pololu 핀은 크리스털 배선에서 {r2c[KX]:g} mm) 입니다. rev 2c보다 가까운 부품: "
          + (", ".join(cd["parts_closer_than_the_rev2c_pins"]) or "없음"),
          "out/reports/routing_rules.json crystal_distance (rev 2c 값은 archive/rev2c_pololu_1s/kicad/vtol_carrier.kicad_pcb 에서 같은 방법으로 잼)", "M")
    info = rr.get("information")
    if info is None:
        raise SystemExit("docs_ko: routing_rules.json has no 'information' block - run routing_checks.py")
    c59 = info["C59.2_gnd"]
    F.put("C59 GND 비아", ("패드 1.5 mm 안에 자기 GND 비아 있음" if c59["own_via"] else
                         "패드 1.5 mm 안에 GND 비아 없음; 같은 윗면 GND 구리 조각에서 가장 가까운 GND 비아까지 "
                         + (f"{c59['nearest_via_on_its_piece_mm']:g} mm" if c59["nearest_via_on_its_piece_mm"] is not None else "(없음)")),
          "out/reports/routing_rules.json information.C59.2_gnd", "M")
    l5v = collections.Counter(f"{v['drill_mm']:g}" for v in info["L5.1_battery_vias"]["vias_within_radius_of_the_pad"])
    F.put("L5 입력 비아", ", ".join(f"드릴 {d} mm {n}개" for d, n in sorted(l5v.items(), reverse=True)) or "없음",
          "out/reports/routing_rules.json information.L5.1_battery_vias (L5 입력 패드에서 4 mm 안의 VBAT_LOGIC_SW 비아)", "M")
    # barrel copper of the 0.4 mm-drill vias (pi x drill x 18 um plating, A) against a 2.0 mm x 35 um track (E)
    big = [v["drill_mm"] for v in info["L5.1_battery_vias"]["vias_within_radius_of_the_pad"] if v["drill_mm"] >= 0.4 - 1e-9]
    barrel = sum(math.pi * d * 0.018 for d in big)
    F.put("L5 입력 비아 단면", f"드릴 0.4 mm 이상 비아 {len(big)}개의 도금 구리 단면은 합쳐서 약 {barrel:.3f} mm2 로, 폭 2.0 mm 1 oz 배선의 "
          f"{2.0 * 0.035:.2f} mm2 보다 {'작습니다' if barrel < 2.0 * 0.035 else '작지 않습니다'} (도금 18 um 가정, E).",
          "out/reports/routing_rules.json information.L5.1_battery_vias 에서 계산 (pi x 드릴 x 0.018 mm)", "E")
    # the same checks measured on the board before the relayout (evidence file written once by the relayout step)
    BEF = "out/reports/routing_rules_before_relayout.json"
    bef_all = load(BEF)
    bef = bef_all["checks"]
    if "sw_node" not in bef or "crystal_distance" not in bef or "information" not in bef_all:
        raise SystemExit(f"docs_ko: {BEF} lacks the relayout-fix checks (sw_node / crystal_distance / information)")
    F.put("재배치 전 SW 패드 간격", f"{bef['sw_node']['pad_edge_gap_U8.5_L5.2_mm']:g} mm", BEF + " sw_node (재배치 전 보드)", "M")
    bsw = bef["crystal_distance"]["rev3_BOOST_SW_copper_min_gap_mm"]
    F.put("재배치 전 SW 구리-크리스털", f"Y1 패드에서 {bsw[KY]:g} mm, XTAL 핀에서 {bsw[KU]:g} mm, 크리스털 배선에서 {bsw[KX]:g} mm",
          BEF + " crystal_distance (재배치 전 보드)", "M")
    F.put("재배치 전 크리스털보다 가까운 부품", bef["crystal_distance"]["parts_closer_than_the_rev2c_pins"],
          BEF + " crystal_distance (재배치 전 보드)", "M")
    b59 = bef_all["information"]["C59.2_gnd"]
    F.put("재배치 전 C59 GND 비아", "패드 1.5 mm 안에 자기 GND 비아 있음" if b59["own_via"] else "패드 1.5 mm 안에 GND 비아 없음",
          BEF + " information.C59.2_gnd (재배치 전 보드)", "M")
    bl5 = collections.Counter(f"{v['drill_mm']:g}" for v in bef_all["information"]["L5.1_battery_vias"]["vias_within_radius_of_the_pad"])
    F.put("재배치 전 L5 입력 비아", ", ".join(f"드릴 {d} mm {n}개" for d, n in sorted(bl5.items(), reverse=True)) or "없음",
          BEF + " information.L5.1_battery_vias (재배치 전 보드)", "M")
    F.put("재배치 전 VIN 경로", f"{bef['input_cap_vin']['copper_path_to_U8.3_mm']:g} mm", BEF + " input_cap_vin (재배치 전 보드)", "M (배선 모델, E)")
    F.put("재배치 전 L5 경로", f"{bef['input_cap_l5']['copper_path_to_L5.1_mm']:g} mm", BEF + " input_cap_l5 (재배치 전 보드)", "M (배선 모델, E)")
    F.put("재배치 전 SW 선 길이", f"{bef['boost_sw']['track_length_mm']:g} mm", BEF + " boost_sw (재배치 전 보드)", "M")
    F.put("재배치 전 좁은 길이", f"{bef['battery_path']['current_carrying_length_narrower_than_2mm']:g} mm", BEF + " battery_path (재배치 전 보드)", "M")
    F.put("재배치 전 레일 변 구리", f"{min(v for k, v in bef['rail_edge_copper']['min_copper_to_edge_mm'].items() if 'rail' in k):g} mm",
          BEF + " rail_edge_copper (재배치 전 보드)", "M")
    # edge: who fits the bodies near the edge (review J7) and copper to the rail edges (review J2)
    bf = edge["bodies_under_2p5_by_fit"]
    F.put("가장자리 2.5 mm 안 JLC 실장", bf["jlc"], "out/reports/edge_distance.json bodies_under_2p5_by_fit", "M")
    F.put("가장자리 2.5 mm 안 with_mcu", bf["with_mcu_jlc_plus_jlc_optional"], "out/reports/edge_distance.json bodies_under_2p5_by_fit", "M")
    F.put("가장자리 2.5 mm 안 JLC 미실장", f"{bf['not_fitted_by_jlc_in_default_order']}개 ("
          + ", ".join(bf["refs_by_fit"]["jlc_optional"] + bf["refs_by_fit"]["user"] + bf["refs_by_fit"]["none"]) + ")",
          "out/reports/edge_distance.json bodies_under_2p5_by_fit", "M")
    ce = edge["copper_to_edge_mm"]["min_per_side"]
    F.put("구리-가장자리 최소", f"{min(ce.values()):g} mm (내층 면 포함, 네 변 모두)" if len(set(ce.values())) == 1
          else ", ".join(f"{s} {v:g} mm" for s, v in ce.items()), "out/reports/edge_distance.json copper_to_edge_mm", "M")
    JLC_CAP = "https://jlcpcb.com/capabilities/pcb-capabilities (rev 3 JLC 검토가 2026-11-11에 읽음; 이 단계에서 다시 읽지 않음)"
    F.put("V-cut 구리 이격", "0.4 mm 이상", JLC_CAP, "F")
    F.put("라우팅·마우스바이트 구리 이격", "0.2 mm 이상", JLC_CAP, "F")
    job = json.loads((OUT / "gerber" / "vtol_carrier-job.gbrjob").read_text(encoding="utf-8"))
    finish = job.get("GeneralSpecs", {}).get("Finish") or next(
        (v for k, v in job.get("GeneralSpecs", {}).items() if k.lower() == "finish"), "-")
    F.put("gbrjob 표면처리", finish, "out/gerber/vtol_carrier-job.gbrjob GeneralSpecs.Finish", "M")
    rot_rows = rows("out/cpl_rotation_table.csv")
    mirrored = sorted({r_["Designator"] for r_ in rot_rows if "mirrored" in r_.get("notes", "")},
                      key=lambda s: (s.rstrip("0123456789"), int("".join(c for c in s if c.isdigit()) or 0)))
    F.put("1번 핀 번호 반대", ", ".join(mirrored) or "없음", "out/cpl_rotation_table.csv notes ('mirrored')", "M")
    empty = sorted((q["refdes"] for q in spec["parts"] if q["fit"] == "none"),
                   key=lambda s: (s.rstrip("0123456789"), int("".join(c for c in s if c.isdigit()) or 0)))
    F.put("빈 자리", ", ".join(empty), "spec/carrier_spec.json parts[].fit == none", "M")
    gp = load("out/reports/gerber_check.json")
    pv = gp.get("paste_variant", {})
    F.put("페이스트 없는 자리 main", ", ".join(pv.get("main", {}).get("refs", [])) or "-", "out/reports/gerber_check.json paste_variant.main", "M")
    F.put("페이스트 없는 자리 with_mcu", ", ".join(pv.get("with_mcu", {}).get("refs", [])) or "-", "out/reports/gerber_check.json paste_variant.with_mcu", "M")
    acc = fit["mcu_access"]["per_mcu"]
    F.put("MCU 1 mm 안 부품", ", ".join(f"{k} {acc[k]['fitted_parts_within_1mm']}" for k in sorted(acc)),
          "out/fit_checks.json mcu_access", "M")
    F.put("회전 미확인", rot["summary"]["orientation_critical_not_verified"], "out/reports/cpl_rotation.json", "NOT_VERIFIED")
    F.put("방향 중요 부품 수", rot["summary"]["orientation_critical_rows"], "out/reports/cpl_rotation.json", "M")

    F.put("JLC 확인 시각", gate["started_at"], "out/jlc_gate_report.json", "M (그 시각의 재고)")
    F.put("JLC 확인 결과", f"{gate['status']} ({gate['main_pair_lines_pass']}/{gate['main_pair_lines']}줄), with_mcu {gate['with_mcu_status']}",
          "out/jlc_gate_report.json", "M")
    F.put("JLC 기록 사양 일치", "예" if man["jlc_gate"]["status"] == "PASS" else man["jlc_gate"]["status"], M + " jlc_gate", "M")
    # watch: fewer than 2000 available, OR the stock fell to less than half of the plan's snapshot (rev 3 review J9)
    plan_stock = {ln["jlc_code"]: ln.get("stock") for ln in plan["bom_lines"]}
    watch = []
    for ln in gate["lines"]:
        was = plan_stock.get(ln["jlc_code"])
        falling = bool(was) and ln.get("stock", 10 ** 9) < 0.5 * was
        if ln.get("available", 10 ** 9) < 2000 or falling:
            watch.append(f"{ln['jlc_code']} ({','.join(ln['designators'])}) {ln['available']}개, 필요 {ln['need_20_boards']}"
                         + (f", 재고가 계획 때 {was}개에서 {ln['stock']}개로 절반 넘게 줄어듦" if falling else ""))
    F.put("재고 주의 줄", watch, "out/jlc_gate_report.json lines (available < 2000, 또는 stock < 계획 spec/rev3_plan.json bom_lines.stock 의 절반)", "M")
    F.put("얇은 재고 기준", f"{gate['thin_threshold']}개", "out/jlc_gate_report.json", "F (계획 I7)")
    F.put("얇은 재고 줄", gate["thin_lines"], "out/jlc_gate_report.json", "M")
    alt = collections.defaultdict(list)
    for a in gate["alternates_and_options"]:
        if a["group"] == "alternate":
            alt[a["for"]].append(f"{a['jlc_code']} {a.get('mpn', '')} ({a['available']}개, {a['verdict']})")
    for k, lst in alt.items():
        F.put(f"대체 {k}", "; ".join(lst), "out/jlc_gate_report.json alternates_and_options", "M")

    for r_ in fees:
        key = f"비용 {r_['pair']} {r_['order']} {r_['boards']}장"
        F.put(key, f"{float(r_['sum_E']):.2f} USD", "out/jlc_fee_estimate.csv sum_E", "E")
        if r_.get("sum_E_with_standard_only_fees"):
            F.put(key + " (Standard 전용 수수료 포함)", f"{float(r_['sum_E_with_standard_only_fees']):.2f} USD",
                  "out/jlc_fee_estimate.csv", "E (적용 여부 NOT_VERIFIED)")
            F.put("Standard 전용 수수료", f"{float(r_['standard_only_fees_applicability_NOT_VERIFIED']):.2f} USD",
                  "out/jlc_fee_estimate.csv", "E")
        F.put(f"THT 납땜점 {r_['pair']}", r_["tht_joints_per_board"], "out/jlc_fee_estimate.csv", "M")
    F.put("따로 살 품목 수", len(offb), "out/off_board_purchase.csv", "M")

    v_ = prog["verdict_per_family"]
    F.put("B안 2x3 자리", f"SMD {v_['smd']['final_2x3_legal_origins']}곳, THT {v_['tht']['final_2x3_legal_origins']}곳",
          "out/reports/programming_access.json verdict_per_family", "M")
    F.put("B안 결론", "불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음)" if v_["smd"]["impossible_even_with_every_greedy_part_removed"]
          else "가능성 있음", "out/reports/programming_access.json", "M")

    uv = plan["power_block"]["uvlo"]
    vo = plan["power_block"]["vout_V"]
    F.put("켜짐 전압", f"{uv['V_on_worst'][0]:.2f} ~ {uv['V_on_worst'][1]:.2f} V", "spec/rev3_plan.json power_block.uvlo", "E")
    F.put("꺼짐 전압", f"{uv['V_off_worst'][0]:.2f} ~ {uv['V_off_worst'][1]:.2f} V", "spec/rev3_plan.json power_block.uvlo", "E")
    F.put("+5V 범위", f"{vo['min_V']:.2f} ~ {vo['max_V_pfm_E']:.2f} V", "spec/rev3_plan.json power_block.vout_V", "E")
    parts = {q["refdes"]: q for q in spec["parts"]}
    F.put("UVLO 저항", f"R65 {parts['R65']['value']}, R66 {parts['R66']['value']}, R67 {parts['R67']['value']}", S, "M")
    F.put("분압 저항", f"R15 {parts['R15']['value']}, R16 {parts['R16']['value']}", S, "M")
    code = {}
    for row in bom:
        for ref in row["Designator"].split(","):
            code[ref.strip()] = row["JLCPCB Part #\uff08optional\uff09"]
    for ref in ("U8", "U9", "L5", "C45", "J2", "J5", "J8", "J15", "J17", "Q1", "U7", "J14"):
        F.put(f"코드 {ref}", code.get(ref, "BOM에 없음"), "out/jlcpcb_bom.csv", "M")
    F.put("U1-U4 기본 BOM", "없음" if not any(code.get(f"U{i}") for i in range(1, 5)) else "있음", "out/jlcpcb_bom.csv", "M")
    mcu_codes = {ref.strip(): row["JLCPCB Part #\uff08optional\uff09"] for row in bom_mcu for ref in row["Designator"].split(",")}
    F.put("U1-U4 with_mcu", ", ".join(sorted({mcu_codes.get(f"U{i}", "-") for i in range(1, 5)})), "out/jlcpcb_bom_with_mcu.csv", "M")
    F.put("툴링 홀(H) 부품", s["by_prefix"].get("H", 0), S + " summary.by_prefix", "M")
    F.put("출고 판정", man["release_gate"]["status"], M + " release_gate", "M (도구 검사)")
    F.put("출고 판정 이유", man["release_gate"]["reasons"], M + " release_gate.reasons", "M")
    if det and "build1_vs_build2" in det:
        bb = det["build1_vs_build2"]
        txt = (f"{det['result']} - 바이트 비교 파일 {bb['deterministic']['files']}개 중 다른 것 {len(bb['deterministic']['differing'])}개; "
               f"시각 줄을 뺀 비교 {bb['normalised']['files']}개 중 다른 것 {len(bb['normalised']['differing'])}개")
        if bb["normalised"]["differing"]:
            canon = bb.get("step_canonical", {})
            txt += (f" ({', '.join(bb['normalised']['differing'])}: kicad-cli가 STEP을 내보낼 때마다 항목 순서가 바뀜. "
                    f"번호·순서를 뺀 내용 비교는 {'같음' if canon and not canon.get('differing') else '다름'})")
        F.put("결정성", txt, "out/reports/determinism.json", "M")
        F.put("결정성 시각", det.get("finished", "-"), "out/reports/determinism.json", "M")
        F.put("빌드 시간", ", ".join(f"{b_['minutes']}분" for b_ in det["builds"]), "out/reports/determinism.json builds", "M")
    elif det:
        F.put("결정성", f"{det.get('result')} ({det.get('why', '')})", "out/reports/determinism.json", "M")
        F.put("결정성 시각", det.get("finished", "-"), "out/reports/determinism.json", "M")
        F.put("빌드 시간", "-", "out/reports/determinism.json", "M")
    else:
        F.put("결정성", "NOT_VERIFIED (scripts/determinism.py 아직 실행 안 됨)", "out/reports/determinism.json 없음", "NOT_VERIFIED")
        F.put("결정성 시각", "-", "-", "NOT_VERIFIED")
        F.put("빌드 시간", "-", "-", "NOT_VERIFIED")
    # relayout-fix step (review RR-2): the same inputs built in a second folder
    xf = (det or {}).get("cross_folder") or {}
    if xf.get("result") == "PASS":
        F.put("다른 폴더 결정성", f"PASS - 다른 폴더에 복사해 다시 만든 결정적 파일 {xf['compared']}개 중 {xf['identical']}개가 이 폴더의 것과 "
              f"바이트까지 같음 (보드, 회로도, 거버와 zip, BOM, CPL 등 주문에 쓰는 파일 {xf['order_files_boards_schematic_checked']}개는 모두 여기에 듦). "
              f"나머지 {len(xf['identical_after_renaming'])}개({', '.join(xf['identical_after_renaming'])})는 자기 폴더 이름이나 "
              "그런 파일의 sha256을 담고 있어, 폴더 이름과 그 sha256을 바꿔 넣은 뒤에 같음. "
              f"그림 단계 산출물 {len(xf['not_rebuilt_in_the_second_folder'])}개는 그 폴더에서 다시 만들지 않아 비교하지 않음",
              "out/reports/determinism.json cross_folder", "M")
    elif xf.get("result") == "FAIL":
        F.put("다른 폴더 결정성", "FAIL - 다른 폴더에서 다시 만들면 달라지는 파일: "
              + (", ".join(xf.get("differing", []) + xf.get("normalised_netlists_differing", [])
                           + xf.get("order_files_boards_schematic_not_raw_identical", [])) or xf.get("why", "-")),
              "out/reports/determinism.json cross_folder", "M")
    else:
        F.put("다른 폴더 결정성", "NOT_VERIFIED - 다른 폴더(다른 체크아웃)에서 다시 만들어도 바이트까지 같은지는 확인하지 않았음 "
              "(`determinism.py --cross-folder <빈 폴더>` 를 실행하지 않음). 같은 폴더 안 두 번 빌드만 비교했습니다",
              "out/reports/determinism.json cross_folder", "NOT_VERIFIED")
    ctx = {"spec": spec, "plan": plan, "man": man, "gate": gate, "fees": fees, "offb": offb, "code": code,
           "parts": parts, "alt": alt, "s17": plan17_facts(F), "edge": edge}
    return F, ctx


#: plan section 17 (open risks): every bullet must be covered by a user-document line (rev 3 review MR-2).
#: keyword of the bullet -> ("new", Korean sentence for README section 5 using the bullet's own numbers) or
#: ("covered", where the existing text says it). Numbers in a 'new' sentence are checked against the bullet text.
S17: list[tuple[str, str, str, list[str]]] = [
    ("Nothing was built or measured", "covered", "README 5장 '보드 동작 전체'", []),
    ("JLC's acceptance of this board", "covered", "README 5장 'JLC의 수락', ORDER 2·3장", []),
    ("'JLC solders the 6 THT lines'", "covered", "README 5장 'JLC의 수락', ORDER 3장 3번과 5장 SMD 대체 표", []),
    ("TPS61023 valley current limit", "new",
     "승압 IC TPS61023의 전류 한계(valley)는 데이터시트에 입력 3.6 V 기준으로만 나옵니다. 입력 3.0 V에서는 1.50 A가 필요하고, "
     "이는 최소 2.7 A의 56 % 입니다 (E, 계획 17장).", ["3.6 V", "1.50 A", "56 %", "2.7 A"]),
    ("Cut-off hysteresis against the voltage rebound", "new",
     "저전압 차단의 히스테리시스 여유는 28 mV뿐입니다 (E). 전원 경로 저항이 0.29 Ohm을 넘으면 방전 끝 최대 부하에서 전원이 켜졌다 꺼졌다 할 수 있어, "
     "첫 보드에서 저항을 재야 합니다 (ORDER 7장 5번).", ["28 mV", "0.29 Ohm"]),
    ("The cut-off acts on the loaded voltage", "new",
     "차단은 부하가 걸린 전압으로 판단합니다 (필터 16.3 ms). 최대 부하가 이어지면 쉬는 전압이 3.62 V 아래인 팩에서 차단이 걸릴 수 있습니다 (E).",
     ["16.3 ms", "3.62 V"]),
    ("Switch-on threshold up to 3.73 V", "new",
     "켜짐 전압이 최대 3.73 V라서, 그보다 낮게 쉬고 있는 팩으로는 보드가 켜지지 않을 수 있습니다 (E).", ["3.73 V"]),
    ("Start-up into 226.9 uF", "new",
     "+5V 쪽 226.9 uF로 시동할 때 1.9 ms 리셋, 감지 필터와의 순서는 확인하지 않았습니다 (첫 보드에서 오실로스코프로, NOT_VERIFIED).",
     ["226.9 uF", "1.9 ms"]),
    ("+3V3 headroom", "new",
     "+3V3 여유: +5V 최저 4.844 V에서 AMS1117 강하 최대 1.3 V를 빼면 3.3 V보다 0.244 V 위입니다 (E). Wi-Fi 송신 중 최저값은 확인 안 됨 (ORDER 7장 9번).",
     ["4.844 V", "1.3 V", "0.244 V"]),
    ("Drain after cut-off", "new",
     "차단 뒤에도 새 회로 6.7 uA (최대 8.0 uA)와 R15 / R16 분압 34..62 uA가 흐릅니다. 차단은 보관용 스위치가 아니므로 스위치를 꺼야 합니다 (E).",
     ["6.7 uA", "8.0 uA", "34..62 uA"]),
    ("22 uF 1206 capacitors", "new",
     "22 uF 1206 커패시터의 5 V 직류 바이어스 감소량은 제조사 곡선으로 확인하지 않았습니다 (NOT_VERIFIED). 출력 실효 용량은 4 uF 이상이어야 하며 "
     "레일 전체 용량으로 보면 여유가 있습니다 (E).", ["4 uF"]),
    ("Placement of the power block", "covered", "ORDER 0장 B (승압부 재배치와 남은 FAIL), README 1장", []),
    ("U8 uses a generic connector symbol", "new",
     "U8은 일반 커넥터 심볼을 써서 ERC가 U8 핀에 대한 근거를 주지 않습니다. 대신 패드-네트 표 검사(U8/U9 패드표)가 근거입니다.", []),
    ("CPL rotations for SOT-563", "covered", "README 5장 'CPL 회전', ORDER 3장 4번", []),
    ("BOM / CPL file format against JLC's parser", "covered", "ORDER 1장 마지막 줄", []),
    ("Molex 22035025 (J14)", "covered", "README 5장 'J14 Molex 커넥터'", []),
    ("THT lead protrusion", "covered", "README 5장 '뒷면 다리 길이', ORDER 7장 3번", []),
    ("Stock moves", "covered", "README 5장 '재고', ORDER 5장", []),
    ("Reflow ratings of C47", "covered", "README 5장 '리플로 온도'", []),
    ("ESP-12F RF behaviour", "new",
     "ESP-12F의 무선 성능은 같은 금지 영역에서 ESP-12E와 같다고 가정했습니다 (A). 펌웨어 이미지도 같은 ESP8266 / 4 MB 구성으로 가정합니다 (A).",
     ["4 MB"]),
    ("BMP280 module header type", "covered", "README 5장 '모듈 치수와 끼움'", []),
    ("OpenLog fit over J5", "covered", "README 5장 '모듈 치수와 끼움'과 모듈 간섭 줄, ORDER 7장 7번", []),
    ("Programming access", "new",
     "프로그래밍: 빈 구멍에 포고핀 어댑터를 댈 때의 접촉은 확인하지 않았습니다 (NOT_VERIFIED). B안(JLC가 띠 밖에 헤더 실장)은 배치 검색에서 자리가 없었습니다.", []),
    ("Hand soldering U1..U4", "covered", "ORDER 7장 6번 (MCU 1 mm 안 부품 수)", []),
    ("AVCC choke C1046", "new",
     "AVCC 초크 C1046은 정격 15 mA (1.15 Ohm) 입니다. ATmega128A의 AVCC 전류와 대조하지 않았습니다 (NOT_VERIFIED).", ["15 mA", "1.15 Ohm"]),
    ("Fees are estimates", "covered", "README 5장 '비용', ORDER 4장", []),
    ("Render tool chain", "covered", "해소됨: 그림 d2를 이번 판에서 실제로 만들었음 (out/render/d2_neo7m_top_cap.json)", []),
    ("Rev 2c documents contain inconsistencies", "covered", "해소됨: 이 문서들은 rev 3 파일에서 새로 생성됨 (rev 2c 문서를 옮기지 않음)", []),
]


def plan17_facts(F: Facts) -> list[tuple[str, str, str]]:
    """read the section 17 bullets of spec/REV3_PLAN.md, map each to S17 (error if a bullet is not mapped or a number
    of a 'new' sentence is not in the bullet), register the 'new' sentences as facts (label from the plan, E / A / F)"""
    text = (ROOT / "spec" / "REV3_PLAN.md").read_text(encoding="utf-8")
    sec = text.split("## 17.", 1)[1].split("\n## ", 1)[0]
    bullets = [ln[2:].strip() for ln in sec.splitlines() if ln.startswith("- ")]
    out, unmapped = [], []
    for bl in bullets:
        hit = [s for s in S17 if bl.startswith(s[0]) or s[0] in bl[:80]]
        if len(hit) != 1:
            unmapped.append(bl[:70])
            continue
        key, kind, txt, nums = hit[0]
        missing = [n for n in nums if n not in bl]
        if missing:
            raise SystemExit(f"docs_ko: plan section 17 bullet {key!r} does not contain {missing}")
        out.append((key, kind, txt))
        if kind == "new":
            F.put(f"계획17 {key}", txt, "spec/REV3_PLAN.md 17장", "계획 17장의 표시 (E / A / NOT_VERIFIED)")
    if unmapped or len(out) != len(S17):
        raise SystemExit(f"docs_ko: plan section 17 bullets not mapped to a document line: {unmapped}; "
                         f"mapped {len(out)} of {len(S17)} S17 rows")
    return out


def change_table(F: Facts, ctx: dict) -> list[str]:
    """plan changes that are not 'keep', with the tag in Korean; the rev 3 code is checked against the BOM file."""
    out = ["| Ref | 변경 | rev 2c | rev 3 | 누가 붙임 | 분류 (태그) | BOM 파일과 대조 |", "|---|---|---|---|---|---|---|"]
    bad = []
    for c in ctx["plan"]["changes"]:
        if c["action"] == "keep":
            continue
        tag = c.get("tag", "")
        tko = next((k for pre, k in TAG_KO if tag.startswith(pre)), tag)
        r2 = c.get("rev2c") or {}
        r3 = c.get("rev3") or {}
        old = f"{r2.get('value', '-')} / {r2.get('part_number', '-')}" if r2 else "-"
        # the rev 3 value as built (spec), not the plan's text (review MR-5: the plan's J3 value still names the ESP-12E)
        built_val = (ctx["parts"].get(c["refdes"]) or {}).get("value") or r3.get("value", "-")
        new = f"{built_val} / {r3.get('jlc_code') or '-'}" if r3 else "-"
        who = {"JLC": "JLC"}.get(r3.get("populate_by", ""), r3.get("populate_by") or "-") if r3 else "-"
        in_bom = ctx["code"].get(c["refdes"])
        want = r3.get("jlc_code") if r3 and r3.get("fit") == "jlc" else None
        if want and in_bom != want:
            bad.append(c["refdes"])
        chk = "일치" if want and in_bom == want else ("BOM에 없음 (맞음)" if not want and not in_bom else "불일치")
        out.append(f"| {c['refdes']} | {ACTION_KO.get(c['action'], c['action'])} | {old} | {new} | {who} | {tko} | {chk} |")
    if bad:
        raise SystemExit(f"docs_ko: plan change rows disagree with out/jlcpcb_bom.csv: {bad}")
    n = collections.Counter(c["action"] for c in ctx["plan"]["changes"])
    F.put("변경 행", ", ".join(f"{ACTION_KO.get(k, k)} {n[k]}" for k in ("replace", "add", "remove", "keep-dnp"))
          + f", 그대로 {n['keep']}", "spec/rev3_plan.json changes", "F (계획)")
    return out


def readme(F: Facts, ctx: dict) -> str:
    spec, man = ctx["spec"], ctx["man"]
    L: list[str] = []
    A = L.append
    A("# VTOL 아비오닉스 캐리어 PCB rev 3 (JLC가 실장해서 보내는 판)")
    A("")
    A("**이 보드는 아직 만들거나 전원을 넣어 본 적이 없습니다 (NOT_VERIFIED).** 아래의 PASS는 모두 도구(kicad-cli ERC/DRC, 스크립트)의 "
      "결과이며 보드가 동작한다는 뜻이 아닙니다. JLC가 이 파일을 받아 주는지도 업로드 후 JLC의 DFM 결과와 견적을 봐야 압니다.")
    A("")
    A("표시: M = 이 PC에서 도구로 측정, E = 추정(계산), A = 가정(실물로 확인 필요), F = 문서·데이터시트 근거. 근거가 없으면 NOT_VERIFIED. "
      "이 문서는 `scripts/docs_ko.py` 가 보고서 파일에서 숫자를 읽어 만든 것입니다 (손으로 고치지 말고 다시 생성). 측정·계산된 숫자의 출처는 맨 끝 8장에 있고, "
      "그 표에 없는 숫자(칩 종류, 규칙 값, 시험 조건)는 계획서 `spec/REV3_PLAN.md` 와 사양 `spec/carrier_spec.json` 에서 옮긴 고정값(F)입니다.")
    A("")
    A("## 1. 지금 상태")
    A("")
    A(f"**출고 판정(도구 검사 기준): {F.v('출고 판정')}** - 이유: {F.v('출고 판정 이유')}.")
    A("")
    A(f"배선이 끝난 보드에서 배터리 전원 경로(J14 → Q10 → J15 → L5)의 구리 저항은 {F.v('배터리 경로 저항 20C')} (20 C, E)로 "
      f"계획 목표 {F.v('배터리 경로 목표')} 를 만족하지만, 계획이 요구한 '2.0 mm 이상 폭'을 {F.v('2 mm보다 좁은 길이')} 구간에서 "
      f"지키지 못했습니다 ({F.v('좁은 구간')}). 60 C에서는 {F.v('배터리 경로 저항 60C')} (E)로 목표를 넘습니다. "
      "도구는 이 규칙을 느슨하게 하지 않았습니다. 그대로 주문할지, 배선을 고칠지는 사용자가 정할 일입니다 "
      f"({p('out/ORDER_ko.md')} 0장).")
    A("")
    A("독립 검토(E-R1 / E-R2 / E-R3) 뒤 승압 IC U8 주변을 다시 배치하고 새로 배선했습니다 (재배치 단계). 지금 보드에서 잰 값(M):")
    A("")
    A(f"- 윗면 GND: {F.v('윗면 GND 한 덩어리')}. 출력 커패시터 C57이 U8 바로 옆에서 VOUT 핀과 GND 핀을 윗면 구리로 잇습니다.")
    A(f"- 입력 커패시터 (검사 input_cap_vin {F.v('검사 input_cap_vin')}): {F.v('입력 커패시터 VIN 경로')}.")
    A(f"- 되먹임(FB) 선: {F.v('FB 선과 출력 커패시터')}.")
    A(f"- 통과한 승압부 검사: {F.v('승압부 배치 PASS')}. **아직 FAIL인 검사: {F.v('승압부 배치 FAIL')}.**")
    A(f"- 남은 편차 1 (hot_loop {F.v('검사 hot_loop')}): {F.v('출력 커패시터 간격')}. 계획 12장은 두 출력 커패시터 모두 IC에서 2 mm 안을 요구하지만, "
      "1206 커패시터는 U8의 VOUT / GND 핀 옆에 하나만 놓을 수 있어 둘째(C43)는 C57 바로 옆, 같은 윗면 +5V / GND 구리 위에 두었습니다.")
    A(f"- 남은 편차 2 (input_cap_l5 {F.v('검사 input_cap_l5')}): {F.v('입력 커패시터 L5 경로')}. U8의 VIN 핀과 L5 입력 패드가 떨어져 있어 커패시터 하나로 "
      "두 곳 모두 몇 mm 안에 둘 수 없습니다. 둘째 입력 커패시터 C42는 스위치 단자 J15 옆 그대로입니다 (모서리에 1206 자리가 더 없음).")
    A(f"- **남은 편차 3 (sw_node {F.v('검사 sw_node')}) - 재배치로 나빠진 점:** {F.v('SW 노드')}. SW 선은 {F.v('SW 선')} 이고, "
      f"재배치 전에는 길이 {F.v('재배치 전 SW 선 길이')}, 패드 간격 {F.v('재배치 전 SW 패드 간격')} 였습니다. 계획 12장·13장은 "
      "'L5는 U8 핀 바로 옆, SW 구리는 두 패드에 필요한 만큼만, 패드가 허락하는 한 짧게'를 요구하는데 지금 보드는 이를 지키지 못합니다. "
      f"층·비아만 보는 검사 boost_sw는 {F.v('검사 boost_sw')} 이지만, 이것은 SW 선이 짧다는 뜻이 아닙니다.")
    A(f"- **남은 편차 4 (crystal_distance {F.v('검사 crystal_distance')}) - 재배치 전에도 지키지 못했고 재배치로 더 가까워진 점:** {F.v('크리스털 거리')}. "
      f"재배치 전의 SW 구리는 {F.v('재배치 전 SW 구리-크리스털')} 였고, 그때 rev 2c보다 가까웠던 부품은 {F.v('재배치 전 크리스털보다 가까운 부품')} 입니다. "
      "계획 12장은 승압부를 rev 2c의 Pololu 핀보다 크리스털에 가깝게 두지 말라고 했습니다. 16 MHz 크리스털(NAV 칩 U1)에 잡음이 "
      "얼마나 실리는지는 보드에서 재기 전까지 NOT_VERIFIED입니다.")
    A("- 편차 3, 4의 실제 영향(스위칭 과전압, 입력 리플, SW 선 발열, 크리스털 잡음)은 보드에서 오실로스코프로 재기 전까지 NOT_VERIFIED입니다.")
    A(f"- 자세한 내용과 선택지: {p('out/ORDER_ko.md')} 0장 B.")
    A("")
    A("| 항목 | 결과 | 표시 |")
    A("|---|---|---|")
    A(f"| 회로도 ERC | {F.v('ERC')} | M |")
    A(f"| PCB DRC (`--schematic-parity --severity-all --refill-zones`) | {F.v('DRC')} | M |")
    A(f"| 넷리스트 = 사양 | {F.v('넷리스트=사양')} | M |")
    A(f"| 보드 패드 넷 = IR | {F.v('보드=IR')} | M |")
    A(f"| MCU 4개 핀 = rev 2c | {F.v('MCU 핀맵')} | M |")
    A(f"| U8 / U9 핀-네트 표 | {F.v('U8/U9 패드표')} | M (표는 F) |")
    A(f"| 배선 규칙 (계획 13장) | {F.v('배선 규칙 PASS 수')} / {F.v('배선 규칙 수')} PASS, FAIL: {F.v('배선 규칙 FAIL')} | M / E |")
    A(f"| 크리스털 XTAL1 길이 (rev 2c보다 나빠지면 안 됨) | {F.v('XTAL1 최악')} | M |")
    A(f"| 설계 규칙 파일 = rev 2c | {F.v('규칙 파일')} | M |")
    A(f"| 구리 섬 / 패드에 닿는 비아 | {F.v('구리 섬')} / {F.v('패드에 닿는 비아')} | M |")
    A(f"| JLC 부품 재고·주문 가능 확인 | {F.v('JLC 확인 결과')}, {F.v('JLC 확인 시각')} | M (그 시각 기준) |")
    A(f"| 주문 파일 형식·fit 집합 검사 | {F.v('주문 파일 검사')} | M |")
    A(f"| 주문 파일을 두 번 써서 같은지 | {F.v('주문 파일 재현')} | M |")
    A(f"| 전체 다시 만들기 두 번 = 같은 파일 | {F.v('결정성')} | M |")
    A(f"| 다른 폴더에서 다시 만들어도 같은 파일 | {F.v('다른 폴더 결정성')} | {F.data['다른 폴더 결정성'][2]} |")
    A(f"| 가장자리 2.5 mm 안 부품 몸체 | 모든 자리 {F.v('가장자리 2.5 mm 안 몸체')}개 = JLC 실장 {F.v('가장자리 2.5 mm 안 JLC 실장')}개 "
      f"(with_mcu {F.v('가장자리 2.5 mm 안 with_mcu')}개) + JLC가 붙이지 않는 자리 {F.v('가장자리 2.5 mm 안 JLC 미실장')} "
      f"(0.3 mm 안: {F.v('가장자리 0.3 mm 안 몸체')}) | M |")
    A(f"| 모듈(OpenLog / BMP280 / GY-521) 간섭, 보통 경우 | {F.v('모듈 간섭')}; 최소 간격 {F.v('모듈 최소 간격')} 미만: "
      f"{F.v('모듈 간섭 보통 0.5 mm 미만')} | A / NOT_VERIFIED |")
    A(f"| 모듈 간섭, 보수적 경우 (OpenLog 아랫면 부품 1.2 mm 가정) | 충돌: {F.v('모듈 간섭 보수적 충돌')}; {F.v('모듈 최소 간격')} 미만: "
      f"{F.v('모듈 간섭 보수적 0.5 mm 미만')} | A / NOT_VERIFIED |")
    A("")
    A("## 2. 이 기판은 무엇인가")
    A("")
    A(f"- 크기 {F.v('외곽')}, 두께 {F.v('두께')}, {F.v('층 수')}층 (F.Cu 신호와 부품 / In1 GND 전면 / In2 +5V 전면 / B.Cu 신호). "
      "부착 구멍 없음: 3 mm 폼테이프 + 케이블타이 1개. 케이블타이 띠(x -3..+3 mm)에는 실장 부품이 없습니다.")
    A(f"- 부품은 모두 윗면에만 있습니다. 부품 높이 한계는 기판 윗면에서 {F.v('높이 한계')} 입니다. 아랫면은 폼테이프 자리라 비어 있습니다.")
    A("- 비행 컴퓨터 칩 4개: U1 NAV, U2 FCU, U3 COM, U4 GNS (ATmega128A-AU, 16 MHz, 5 V). 무선은 U7 ESP-12F (Wi-Fi 중계).")
    A("- 센서: GY-521 (IMU, J1에 납땜), BMP280 (기압, 소켓 J2), NEO-7M GPS (기체 윗뚜껑, 케이블로 J7). 기록기 OpenLog는 소켓 J5.")
    A("- 전원 흐름: 1S 로직 배터리 J14 → Q10 역극성 보호 → 스위치 리드 J15 → U8 승압 5 V (+ U9 저전압 차단) → +5V 면 → U6 → +3V3.")
    A(f"- 기판 {F.v('식별 문구 위치')}에 이 프로젝트의 깃 저장소 주소와 연락용 이메일 주소가 두 줄로 인쇄됩니다 "
      f"({F.v('식별 문구 크기')}; {F.v('식별 문구 간격')}; 검사 {F.v('식별 문구 검사')}, M). 주소 글자 자체는 문서에 다시 적지 않습니다. "
      "윗면에는 두 줄이 들어갈 빈자리가 없어 아랫면에 넣었고, 폼테이프를 붙이면 그 밑은 가려집니다.")
    A("")
    A("블록 (회로도 시트 기준, 부품 수는 사양에서 셈):")
    A("")
    A("| 블록 | 내용 | 부품 자리 | 그중 JLC 실장 |")
    A("|---|---|---|---|")
    cnt = collections.Counter(q["block"] for q in spec["parts"])
    cj = collections.Counter(q["block"] for q in spec["parts"] if q["fit"] == "jlc")
    for blk in ("PWR", "NAV", "FCU", "COM", "GNS", "SHIFT", "IO", "LOG", "LINK"):
        if cnt.get(blk):
            F.put(f"블록 {blk}", f"{cnt[blk]} / {cj.get(blk, 0)}", "spec/carrier_spec.json parts[].block, parts[].fit", "M")
            a, bb = F.v(f"블록 {blk}").split(" / ")
            A(f"| {blk} | {BLOCK_KO.get(blk, blk)} | {a} | {bb} |")
    A("")
    A("## 3. rev 2c에서 바뀐 것과 이유")
    A("")
    A("- **5 V 전원 모듈을 없앴습니다.** 꽂는 Pololu 모듈(J6, 디바이스마트 부품)은 JLC가 실장할 수 없어서, JLC 부품으로 만든 승압 회로 "
      f"U8 TPS61023 ({F.v('코드 U8')}) + L5 ({F.v('코드 L5')})를 예전 모듈 자리(+x / -y 모서리)에 올렸습니다. 이유: 요청(모든 부품 JLC 실장).")
    A(f"- **저전압 차단을 유지했습니다.** 예전 모듈 안에 있던 기능입니다. U9 TLV3012B ({F.v('코드 U9')}) + {F.v('UVLO 저항')}: "
      f"켜짐 {F.v('켜짐 전압')}, 꺼짐 {F.v('꺼짐 전압')} (계산값 E, 실측 전).")
    A(f"- **Wi-Fi 모듈 ESP-12E → ESP-12F** ({F.v('코드 U7')}). 핀과 풋프린트가 같고, ESP-12E는 JLC에서 재고가 적고 Standard 주문에만 쓸 수 있었습니다.")
    A(f"- **스루홀 커넥터도 JLC가 납땜합니다.** 소켓 J2 / J5 ({F.v('코드 J2')}), 서보 헤더 J8~J12 ({F.v('코드 J8')}), J16, J18, "
      f"배터리 J14 ({F.v('코드 J14')}), 스위치·부저 J15 / J17 ({F.v('코드 J15')}). 탄탈 C45 ({F.v('코드 C45')})와 헤더·소켓 번호 일부는 재고나 도면 근거로 "
      "바꿨고 D3 / D12에서 rev 2c 번호로 되돌릴 수 있습니다.")
    A(f"- **그대로인 것:** MCU 4개의 핀 256개가 가는 네트 ({F.v('MCU 핀맵')}), 외곽, 커넥터·MCU·크리스털·모듈 소켓·ESP 위치와 방향, 설계 규칙 "
      f"({F.v('규칙 파일')}), 크리스털 배선 ({F.v('XTAL1 최악')}).")
    A(f"- **배치·배선을 다시 했습니다.** 새 부품 13개를 넣느라 주변 수동 부품 일부가 옮겨졌고, 배선은 새로 했습니다 (최종 보드 파일 기준 세그먼트 {F.v('배선 세그먼트')}개, "
      f"원호 {F.v('배선 원호')}개, 비아 {F.v('배선 비아')}개: {F.v('비아 크기별')}). 배선은 Freerouting 실행 기록(세션 파일)을 다시 넣는 방식으로 재현됩니다.")
    A(f"- 부품마다 바뀐 내용과 이유 분류(태그)의 전체 표: {p('out/ORDER_ko.md')} 10장 ({F.v('변경 행')}).")
    A("")
    A("## 4. 누가 무엇을 붙이나")
    A("")
    A("| 구분 | 뜻 | 개수 |")
    A("|---|---|---|")
    for k in ("jlc", "jlc_optional", "user", "none"):
        A(f"| {k} | {FIT_KO[k]} | {F.v('fit ' + k)} |")
    A("")
    A(f"- JLC가 붙이는 것: 기본 파일 쌍 기준 BOM {F.v('main bom_lines')}줄 (SMT {F.v('main bom_lines_smt')} + THT {F.v('main bom_lines_tht')}), "
      f"부품 {F.v('main parts')}개. MCU까지 맡기는 with_mcu 쌍은 {F.v('with_mcu bom_lines')}줄 / {F.v('with_mcu parts')}개 "
      f"(U1~U4 = {F.v('U1-U4 with_mcu')}).")
    A("- 사용자가 붙이는 것: ATmega128A-AU 4개 (기본, with_mcu 쌍이면 JLC), GY-521 (J1 구멍에 모듈 핀을 납땜), BMP280 (J2에 꽂음), "
      "OpenLog (모듈에 직각 6핀 헤더를 납땜한 뒤 J5에 꽂음), NEO-7M (케이블로 J7).")
    A(f"- 부품 없이 비워 둔 자리: 프로그래밍 단자 J3, J19~J22 (2x3), J24~J26 (1x2) - 현재 선택 {F.v('프로그래밍 헤더 선택')}안 (빈 도금 구멍). "
      f"JLC가 헤더를 띠 밖에 실장하는 B안은 배치 검색 결과 {F.v('B안 결론')} ({F.v('B안 2x3 자리')}).")
    A(f"- MCU·센서 말고 사용자가 해야 하는 일(레일 떼기, 다리 자르기, 프로그래밍 단자 등)은 {p('out/ORDER_ko.md')} 8장에 모았습니다.")
    A("")
    A("## 5. 확인되지 않은 것 (NOT_VERIFIED)")
    A("")
    for t in [
        f"배터리 경로 폭 규칙 FAIL ({F.v('2 mm보다 좁은 길이')})과 승압부에 남은 FAIL ({F.v('승압부 배치 FAIL')}): "
        f"사용자 결정 전까지 도구 판정은 {F.v('출고 판정')} 입니다.",
        f"재배치한 승압부의 전기적 효과: SW 선이 두 출력 커패시터 아래를 지나 길어졌고(검사 sw_node {F.v('검사 sw_node')}), 승압부와 SW 구리가 "
        f"크리스털 Y1 / U1 XTAL 핀에 rev 2c의 Pololu 핀보다 가까워졌습니다(검사 crystal_distance {F.v('검사 crystal_distance')}; 숫자는 1장). "
        "스위칭 과전압, 입력 리플, SW 선 발열, 크리스털 잡음 영향은 보드에서 재기 전까지 NOT_VERIFIED입니다.",
        f"모듈 끼움, 보수적 경우: OpenLog 아랫면 부품을 1.2 mm로 잡으면 꽂는 길에서 {F.v('모듈 간섭 보수적 충돌')} 이 겹치고, "
        f"{F.v('모듈 간섭 보수적 0.5 mm 미만')} 은 {F.v('모듈 최소 간격')} 보다 가깝습니다 (A). OpenLog 아랫면 높이를 재고, 막을 수 있는 부품을 "
        "납땜하기 전에 실제로 꽂아 보십시오.",
        "보드 동작 전체: 만들거나 전원을 넣은 적이 없습니다. 전원부 수치(5 V 출력, 차단 전압, 전류 여유)는 데이터시트 계산(E)입니다.",
        f"JLC의 수락: 가장자리 가까운 부품, THT {F.v('main bom_lines_tht')}줄 손납땜, BOM / CPL 형식, Standard 주문의 레일은 업로드 후 DFM 결과와 견적으로만 확인됩니다.",
        f"CPL 회전: 방향이 중요한 {F.v('방향 중요 부품 수')}개 중 {F.v('회전 미확인')} 은 자료끼리 어긋나 NOT_VERIFIED입니다. "
        "나머지도 JLC 미리보기로만 확정됩니다. 탄탈 C45 / C52, 전해 C47을 거꾸로 붙이면 화재 위험이 있습니다.",
        f"리플로 온도: Economic(255 C)에서 한계에 걸리는 줄 {F.v('리플로 한계')}; 문서에서 정격을 찾지 못한 부품 번호 {F.v('리플로 미확인 줄 수')}개.",
        f"뒷면 다리 길이: {F.v('다리 규칙 초과')} 은 규칙 {F.v('뒷면 다리 규칙')} 보다 길어 잘라야 합니다. {F.v('다리 길이 미확인')} 은 확인 못 함.",
        f"모듈 치수와 끼움: GY-521, BMP280, OpenLog, NEO-7M 치수는 가정값입니다 (GY-521 받침 {F.v('GY-521 받침 높이')}, BMP280 헤더 {F.v('BMP280 헤더 종류')} - 모두 A). "
        "그림과 간섭 검사도 이 가정 위에 있습니다. 실물을 버니어로 재야 확정됩니다 (D15).",
        "J14 Molex 커넥터: 라이브러리 3D 모델이 없고 구멍 지름을 도면과 대조하지 못했습니다.",
        f"재고: {F.v('JLC 확인 시각')} 기준입니다. 주문 직전에 scripts/jlc_gate.py 를 다시 돌려야 합니다.",
        "비용: 수수료 표로 계산한 추정(E)입니다. 실제 금액은 JLC 견적뿐입니다.",
        "GPS 케이블이 기체 안에서 지나갈 길은 설계되지 않았습니다 (그림은 예시).",
    ]:
        A(f"- {t}")
    A("")
    A("계획서 17장의 나머지 위험 항목 (계획에서 그대로 옮김; 17장의 모든 항목이 이 문서나 주문 안내서의 어느 줄에 대응하는지 "
      "`scripts/docs_ko.py` 가 확인함):")
    A("")
    for key, kind, _txt in ctx["s17"]:
        if kind == "new":
            A(f"- {F.v('계획17 ' + key)}")
    A("")
    A("## 6. 다시 만드는 방법")
    A("")
    A(f"작업 폴더 {p('scripts')} 에서 (환경변수 `PYTHONDONTWRITEBYTECODE=1`):")
    A("")
    A("```")
    A("B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py           # 전체 (사양 -> 회로도 -> 배치 -> 배선 재생 -> 검사 -> 주문 파일 -> 그림 -> 문서 -> manifest)")
    A("B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py --list    # 단계 목록")
    A("B:/Claude/.venv/Scripts/python.exe -X utf8 -B determinism.py        # 전체를 두 번 만들어 파일이 바이트까지 같은지 비교")
    A("B:/Claude/.venv/Scripts/python.exe -X utf8 -B jlc_gate.py           # 주문 당일 JLC 재고 확인 (인터넷, 읽기만)")
    A("```")
    A("")
    A(f"- 한 번 만드는 데 걸린 시간: {F.v('빌드 시간')} (determinism.py의 두 빌드, M). 마지막 결정성 검사: {F.v('결정성 시각')}.")
    A("- make_all.py는 인터넷을 쓰지 않습니다. JLC 부품 근거는 저장된 기록으로 다시 평가하고, 재고 확인(jlc_gate.py)은 따로 실행합니다.")
    A("- 배선은 기록된 Freerouting 세션을 다시 넣어 같은 보드를 만듭니다. 배치가 바뀌면 route.py가 재생을 거부하고, 새 배선(`make_all.py --reroute`)이 "
      "필요합니다. 새 배선은 실행마다 결과가 달라질 수 있습니다.")
    A(f"- 결정성 검사 결과: {F.v('결정성')}. 비교 대상은 회로도, 보드, BOM, CPL, 거버와 zip, 회전 표, 3D 모델, 사양, 이 문서와 주문 안내서이고, "
      "STEP 파일은 머리줄의 시각을 뺀 뒤 비교합니다 (계획 14장). PNG 그림, 보고서(실행 시각 포함), 회로도 PDF는 증거 자료이며 결정적 산출물이 아닙니다. "
      "STEP 파일(기구 확인용)은 주문 파일이 아닙니다.")
    A(f"- 다른 폴더에서 다시 만들기 (`determinism.py --cross-folder <빈 폴더>`): {F.v('다른 폴더 결정성')}. "
      "재배치 검토(RR-2)에서 같은 입력을 다른 폴더에서 재생하면 구리는 같아도 배선의 uuid가 달라 보드 파일 바이트가 달랐습니다 (M). "
      "그래서 route.py가 배선·비아·실크 글자의 uuid를 그 항목의 내용(네트, 층, 폭, 좌표)에서 만들도록 바꾸었고, "
      "겹쳐 놓인 똑같은 배선 조각도 하나만 남깁니다. "
      "주의: 사양 파일 `spec/carrier_spec.json` 은 자기가 만들어진 폴더 이름을 담고 있어서, 다른 폴더(예: 다른 체크아웃으로 합친 뒤)에서 "
      "다시 만들면 사양의 sha256이 달라지고 JLC 재고 기록(`out/jlc_gate_report.json`)은 '다른 사양'(STALE)으로 판정됩니다. "
      "그때는 그 폴더에서 `jlc_gate.py` 를 다시 실행해야 합니다 (주문 당일에는 어차피 다시 실행합니다).")
    A("- 인터프리터: 대부분 `B:/Claude/.venv`, 보드 처리는 KiCad 10.0.6의 python, 3D 모델·그림 일부는 "
      "`B:/0)AI_Cad/CanSat_EDF_Rocket/.venv` (실행만 함).")
    A("")
    A("## 7. 파일 지도")
    A("")
    A("| 무엇 | 경로 |")
    A("|---|---|")
    for what, rel in [
        ("주문 안내서 (먼저 읽기)", "out/ORDER_ko.md"),
        ("최종 보고서 (납품 단계가 손으로 쓴 시점 기록, 다시 만들어지지 않음)", "out/REV3_REPORT_ko.md"),
        ("주문 파일: 거버 zip / BOM / CPL (기본 쌍)", "out/vtol_carrier_gerber.zip"),
        ("", "out/jlcpcb_bom.csv"),
        ("", "out/jlcpcb_cpl.csv"),
        ("주문 파일: MCU까지 JLC 실장 쌍", "out/vtol_carrier_gerber_with_mcu.zip"),
        ("", "out/jlcpcb_bom_with_mcu.csv"),
        ("", "out/jlcpcb_cpl_with_mcu.csv"),
        ("부품 회전 표 / 미리보기 대조 그림", "out/cpl_rotation_table.csv"),
        ("", "out/cpl_rotation_check.png"),
        ("따로 살 것", "out/off_board_purchase.csv"),
        ("비용 추정 (E)", "out/jlc_fee_estimate.csv"),
        ("JLC 재고 확인 결과", "out/jlc_gate_report.json"),
        ("검사 결과 모음 + 파일 해시 + 출고 판정 (계획의 MANIFEST.json)", "out/manifest.json"),
        ("검사 보고서 폴더", "out/reports"),
        ("회로도 PDF", "out/schematic/vtol_carrier_schematic.pdf"),
        ("3D STEP (J14 모델 없음)", "out/vtol_carrier.step"),
        ("장착 그림·슬라이드와 설명", "out/render/INDEX_ko.md"),
        ("회로도·보드 발표용 그림 (보드 그림에는 모든 부품 모델이 보임)", "out/presentation"),
        ("옛 rev 2c 시험 그림 (보관용, 납품물 아님)", "out/render/_spike_rev2c"),
        ("KiCad 프로젝트 (회로도·보드)", "kicad/vtol_carrier.kicad_pro"),
        ("", "kicad/vtol_carrier.kicad_sch"),
        ("", "kicad/vtol_carrier.kicad_pcb"),
        ("사양 / 사양 요약", "spec/carrier_spec.json"),
        ("", "spec/SPEC_ko.md"),
        ("계획 (설계 기준)", "spec/REV3_PLAN.md"),
        ("", "spec/rev3_plan.json"),
        ("모듈 치수 (대부분 가정)", "spec/module_dimensions.json"),
        ("스크립트", "scripts"),
        ("rev 2c 보관본", "archive/rev2c_pololu_1s"),
    ]:
        A(f"| {what} | {p(rel)} |")
    A("")
    return "\n".join(L)


def order(F: Facts, ctx: dict) -> str:
    gate, alt = ctx["gate"], ctx["alt"]
    L: list[str] = []
    A = L.append
    A("# JLCPCB 주문 안내서 - VTOL 캐리어 PCB rev 3")
    A("")
    A("이 안내서는 PCB 실장 주문(PCBA)을 처음 하는 사람도 따라 할 수 있게 썼습니다. **이 도구는 아무것도 업로드하거나 주문하지 않았습니다.** "
      "업로드, 확인, 결제는 모두 사용자가 JLCPCB 사이트에서 직접 합니다.")
    A("")
    A("표시: M = 도구로 측정, E = 추정, A = 가정, F = 문서 근거, NOT_VERIFIED = 근거 없음. 측정·계산된 숫자의 출처는 맨 끝 11장에 있습니다. "
      "그 표에 없는 숫자(시험 전류·전압, 규칙 값, 수량 기본값)는 계획서 `spec/REV3_PLAN.md` 의 괄호 안 장(15장, 16장의 D 번호 등)에서 옮긴 값(F, 계산값은 E)입니다. "
      "이 파일은 `scripts/docs_ko.py` 가 만든 것입니다.")
    A("")
    A("## 0. 먼저 결정할 것: 도구 판정은 아직 '주문 불가'입니다")
    A("")
    A(f"출고 판정: **{F.v('출고 판정')}** ({F.v('출고 판정 이유')}).")
    A("")
    A("결정할 문제는 두 가지(A, B)입니다. B의 큰 문제(승압 IC 주변의 고전류 루프와 입력 커패시터)는 재배치 단계에서 고쳤지만, "
      "그 대가로 스위칭 노드(SW) 선이 길어지고 승압부가 NAV 크리스털에 가까워졌습니다. 아래에는 고친 것, 남은 것, 나빠진 것을 모두 적었습니다.")
    A("")
    A("### A. 배터리 전원 경로의 구리 폭")
    A("")
    A(f"- 무엇이 문제인가: 배터리 → 승압 회로로 가는 전원 경로의 구리 폭을 계획은 2.0 mm 이상(또는 면)으로 정했는데, {F.v('2 mm보다 좁은 길이')} 구간이 "
      f"더 좁습니다: {F.v('좁은 구간')}.")
    A(f"- 재배치 단계에서 줄인 것: 좁은 길이 {F.v('재배치 전 좁은 길이')} → {F.v('2 mm보다 좁은 길이')} (Q10 → J15 구간을 2.0 mm로 넓히고 L5 앞 비아를 패드 바로 위로 옮김). "
      "남은 구간은 모양 때문에 2.0 mm 한 줄이 들어가지 않습니다: 케이블타이 띠를 건너는 곳은 프로그래밍 단자 구멍 사이 틈이 1.86 mm라 1.56 mm가 한계이고 "
      "(scripts/build_pcb.py VBAT_TRUNK 설명, M), Q10(SOT-23) 3번 패드로 들어가는 짧은 구간은 옆 패드와 C42 패드에 막히며, L5 입력 패드는 폭이 1.5 mm입니다. "
      "이 검사와 2.0 mm / 0.035 Ohm 기준은 바꾸지 않았으므로 FAIL로 남습니다.")
    A(f"- 영향(E): 구리 저항은 20 C에서 {F.v('배터리 경로 저항 20C')} 로 목표 {F.v('배터리 경로 목표')} 안이지만, 60 C에서는 {F.v('배터리 경로 저항 60C')} 로 넘습니다. "
      "저항이 크면 방전 끝에서 저전압 차단이 켜졌다 꺼졌다 할 여유가 줄어듭니다 (계획 17장).")
    A("- 선택지 (도구가 정하지 않습니다):")
    A("  1. 이대로 주문: 폭 규칙 위반을 알고 받아들인다고 사용자가 정합니다. 파일은 그대로 쓸 수 있지만 도구 판정은 바뀌지 않습니다. "
      "보드를 받으면 7장의 '배터리에서 U8 입력까지 저항' 측정을 꼭 하십시오.")
    A("  2. 고친 뒤 주문: 해당 구간의 미리 그린 배선이나 배치를 바꾸고 배선을 새로 해야 합니다 (시간이 걸리고, 결과가 달라질 수 있음). 사용자의 승인이 필요합니다.")
    A(f"- 참고 (M, 폭 검사는 비아를 세지 않음): L5 입력 패드에서 4 mm 안에 있는 배터리 네트(VBAT_LOGIC_SW) 비아는 {F.v('L5 입력 비아')} 입니다 "
      f"(재배치 전 {F.v('재배치 전 L5 입력 비아')}). {F.v('L5 입력 비아 단면')} "
      "배선을 다시 한다면 이 자리에 드릴 0.4 mm 비아를 3~4개 두는 것이 좋습니다.")
    A("")
    A("### B. 승압 IC U8 주변 배치 (독립 전기 검토 E-R1 / E-R2 / E-R3 뒤 재배치함)")
    A("")
    A("- 고친 것 (재배치 단계, 모두 지금 보드에서 잰 값 M):")
    A(f"  - 윗면 GND: {F.v('윗면 GND 한 덩어리')}. 전에는 출력 커패시터의 GND가 내층과 비아 하나로만 U8 GND 핀에 돌아갔습니다.")
    A(f"  - 입력 커패시터 (검사 input_cap_vin {F.v('검사 input_cap_vin')}): {F.v('입력 커패시터 VIN 경로')}. 전에는 구리 길이 {F.v('재배치 전 VIN 경로')} 였습니다.")
    A(f"  - 되먹임(FB) 선: {F.v('FB 선과 출력 커패시터')}. 전에는 C57 패드 사이를 지났습니다.")
    A(f"- 남은 것 1 (검사 hot_loop {F.v('검사 hot_loop')}): {F.v('출력 커패시터 간격')}. 계획 12장은 두 출력 커패시터 모두 IC에서 2 mm 안을 요구합니다. "
      "U8(SOT-563)의 VOUT / SW / GND 핀은 한 줄에 0.5 mm 간격이라, 1206 커패시터는 SW 선을 사이에 두고 그 옆에 하나(C57)만 설 수 있습니다. "
      "둘째(C43)는 C57 바로 옆에서 같은 윗면 +5V / GND 구리에 붙였습니다.")
    A(f"- 남은 것 2 (검사 input_cap_l5 {F.v('검사 input_cap_l5')}): {F.v('입력 커패시터 L5 경로')}. 전에는 {F.v('재배치 전 L5 경로')} 였습니다. 인덕터 L5의 입력 패드는 U8의 VIN 핀 반대쪽 "
      "끝에 있어, 커패시터 하나를 두 곳 모두 몇 mm 안에 둘 수 없습니다. 이 모서리에 1206 자리가 하나 더 없어서 둘째 입력 커패시터 C42는 "
      "스위치 단자 J15 옆에 그대로 두었습니다.")
    A(f"- **나빠진 것 1 (검사 sw_node {F.v('검사 sw_node')}; 계획 12장·13장을 지키지 못함):** {F.v('SW 노드')}. "
      f"SW 선은 {F.v('SW 선')} 입니다. 재배치 전에는 SW 선 길이 {F.v('재배치 전 SW 선 길이')}, 패드 간격 {F.v('재배치 전 SW 패드 간격')} 였습니다. "
      "출력 커패시터를 U8 옆에 붙이느라 인덕터 L5가 U8에서 멀어졌고, SW 선이 두 출력 커패시터의 패드 사이를 지납니다. "
      f"층과 비아만 보는 검사 boost_sw는 {F.v('검사 boost_sw')} 이지만 그것은 SW 선이 짧다는 뜻이 아닙니다. "
      "SW 선은 전압이 빠르게 오르내리는 선이라 길수록 잡음을 더 내고, 선의 인덕턴스 때문에 순간 과전압이 커질 수 있습니다. "
      "데이터시트의 SW / VOUT 절대 최대는 7 V (10 ns 동안 8 V) 입니다 (F, tps61023.txt:172-174). 과전압·리플·발열이 얼마인지는 "
      "보드에서 재기 전에는 모릅니다 (NOT_VERIFIED).")
    A(f"- **나빠진 것 2 (검사 crystal_distance {F.v('검사 crystal_distance')}; 계획 12장을 재배치 전에도 지키지 못했고 재배치로 더 가까워짐):** {F.v('크리스털 거리')}. "
      f"재배치 전의 SW 구리는 {F.v('재배치 전 SW 구리-크리스털')} 였고, 그때 rev 2c보다 가까웠던 부품은 {F.v('재배치 전 크리스털보다 가까운 부품')} 입니다. "
      "NAV 칩 U1의 16 MHz 크리스털에 스위칭 잡음이 실리면 시계가 흔들릴 수 있습니다. 실제로 그런지는 보드에서 재기 전에는 모릅니다 (NOT_VERIFIED).")
    A(f"- 알려진 작은 항목 (M, 검사 대상 아님): 저전압 차단 비교기 U9의 바이패스 커패시터 C59는 GND 패드 옆에 자기 비아가 없습니다 "
      f"({F.v('C59 GND 비아')}; 재배치 전: {F.v('재배치 전 C59 GND 비아')}). U9는 전류가 아주 작은 비교기라 영향은 작을 것으로 보지만 (E), "
      "다음에 배선을 다시 한다면 비아를 하나 붙이는 것이 좋습니다.")
    A("- 선택지: 1. 이대로 주문하고 7장의 오실로스코프 측정(켜질 때 VIN, 0.78 A 부하에서 SW와 +5V 과전압, 입력 리플, NAV 칩 시계)을 반드시 함; "
      "2. 남은 것과 나빠진 것까지 없애려면 저전압 차단 회로(U9 주변)까지 옮겨 모서리 전체를 다시 배치해야 합니다 (사용자 승인 필요); "
      "3. 재배치 전 배치로 되돌림 (SW 선은 짧지만 E-R1 / E-R2 / E-R3 지적이 그대로 남음; 이것도 사용자 승인과 새 빌드가 필요).")
    A("")
    A("- 아래 1~9장은 A와 B 모두 '이대로 주문'을 고를 경우를 기준으로 쓴 주문 절차입니다.")
    A("")
    A("## 1. 올릴 파일 3개")
    A("")
    A("기본 (MCU 4개는 사용자가 직접 납땜, 요청대로):")
    A("")
    A("| 순서 | 파일 | 어디에 올리나 | sha256 앞부분 |")
    A("|---|---|---|---|")
    A(f"| 1 | {p('out/vtol_carrier_gerber.zip')} | PCB 주문 첫 화면 'Add gerber file' | {F.v('zip main sha256')} |")
    A(f"| 2 | {p('out/jlcpcb_bom.csv')} | PCB Assembly 단계 'BOM file' | {F.v('jlcpcb_bom.csv sha256')} |")
    A(f"| 3 | {p('out/jlcpcb_cpl.csv')} | PCB Assembly 단계 'CPL file' (Pick and Place) | {F.v('jlcpcb_cpl.csv sha256')} |")
    A("")
    A("다른 선택 (MCU 4개도 JLC가 실장, 결정 D4의 권장안): 아래 3개를 **한 묶음으로** 올립니다.")
    A("")
    A("| 순서 | 파일 | sha256 앞부분 |")
    A("|---|---|---|")
    A(f"| 1 | {p('out/vtol_carrier_gerber_with_mcu.zip')} | {F.v('zip with_mcu sha256')} |")
    A(f"| 2 | {p('out/jlcpcb_bom_with_mcu.csv')} | {F.v('jlcpcb_bom_with_mcu.csv sha256')} |")
    A(f"| 3 | {p('out/jlcpcb_cpl_with_mcu.csv')} | {F.v('jlcpcb_cpl_with_mcu.csv sha256')} |")
    A("")
    A(f"- **두 묶음을 섞지 마십시오.** 두 zip은 F.Paste(솔더 페이스트) 층만 다릅니다: 기본 zip은 U1~U4 패드에 페이스트 구멍이 없고 "
      f"(페이스트 패드 {F.v('F.Paste 패드 main')}개), with_mcu zip은 있습니다 ({F.v('F.Paste 패드 with_mcu')}개). "
      f"페이스트를 뺀 자리: 기본 {F.v('페이스트 없는 자리 main')}; with_mcu {F.v('페이스트 없는 자리 with_mcu')} (부품을 달지 않는 자리).")
    A(f"- 기본 BOM: {F.v('main bom_lines')}줄 (SMT {F.v('main bom_lines_smt')} + THT {F.v('main bom_lines_tht')}), 부품 {F.v('main parts')}개, CPL {F.v('main cpl_rows')}행. "
      f"with_mcu: {F.v('with_mcu bom_lines')}줄, 부품 {F.v('with_mcu parts')}개, CPL {F.v('with_mcu cpl_rows')}행.")
    A(f"- 기본 BOM에 MCU(U1~U4)가 들어 있는지: {F.v('U1-U4 기본 BOM')}. 센서 자리 J1과 프로그래밍 단자도 들어 있지 않습니다 (사용자 몫).")
    A("- 파일 형식은 JLC 견본 파일(Sample-BOM_JLCSMT, Sample-CPL_JLCSMT)과 같은 머리줄로 만들었지만, JLC가 받아들이는지는 업로드 미리보기에서 "
      "모든 줄이 매칭될 때까지 NOT_VERIFIED입니다.")
    A(f"- 두 거버 zip 모두 기판 {F.v('식별 문구 위치')}에 깃 저장소 주소와 연락용 이메일 주소 두 줄이 들어 있습니다 "
      f"({F.v('식별 문구 크기')}, 검사 {F.v('식별 문구 검사')}); 거버 미리보기의 그 면에서 두 줄이 끊기지 않고 보이는지 확인하십시오 "
      "(JLC 인쇄 품질은 NOT_VERIFIED).")
    A("")
    A("## 2. 주문 화면 입력표 (결정 D9, D10, D11, D14, D16)")
    A("")
    A("화면의 항목 이름은 확인하지 못했습니다 (NOT_VERIFIED - 업로드한 적 없음). 비슷한 이름의 항목을 찾아 아래 값으로 맞추십시오.")
    A("")
    A("| 화면 항목 | 넣을 값 | 근거 |")
    A("|---|---|---|")
    A(f"| Layers | {F.v('층 수')} | 보드 {F.v('층 수')}층 (거버 검사 M) |")
    A(f"| Dimensions | {F.v('외곽')} (거버를 올리면 자동으로 읽힘 - 같은지 확인) | 사양 |")
    A("| PCB Qty | 5 | D11 기본 (PCB 5장 중 2장 실장) |")
    A(f"| PCB Thickness | {F.v('두께')} | 사양 |")
    finish_note = (f"거버 zip 안의 job 파일에도 '{F.v('gbrjob 표면처리')}' 로 적혀 있습니다 (M). " if F.v('gbrjob 표면처리') == "ENIG" else
                   f"주의: 거버 zip 안의 job 파일에는 '{F.v('gbrjob 표면처리')}' 로 적혀 있습니다. ")
    A("| Surface Finish | ENIG | D16 (MCU 0.8 mm 피치 패드와 나중에 납땜할 구멍에 평평한 표면; 추가 요금 NOT_VERIFIED). " + finish_note
      + "표면처리는 주문 화면에서 고른 값이 정하므로 화면에서 ENIG를 고르십시오 (JLC가 job 파일을 따르는지는 NOT_VERIFIED) |")
    A("| Solder Mask / Silkscreen | Green / White | D16 (JLC 기본값) |")
    A("| 적층(Stackup), 동박 두께, Impedance Control | JLC 기본 4층 적층, 기본 동박, 임피던스 제어 없음 | D16 |")
    A("| Via Covering | Tented | D16 |")
    A("| 주문번호 표시 (Mark on PCB / Remove Order Number) | 기본값 (JLC가 위치를 정함) | D16 |")
    A("| PCB Assembly | 켬 (On) | 요청 |")
    A("| PCBA Type | Standard | D9 (아래 설명) |")
    A("| Assembly Side | Top Side | 모든 부품이 윗면 (사양) |")
    A("| PCBA Qty | 2 | D11 (JLC 최소 2장) |")
    A("| Edge Rails / Tooling holes | JLC가 붙임 (Added by JLCPCB) | D10: 설계에 툴링 홀 없음 |")
    A("| Confirm Parts Placement | Yes | 회전이 NOT_VERIFIED인 부품이 있음 (3장) |")
    A("| 메모 (Remark) | 아래 문장 붙여넣기 | D14, 가장자리 거리 표 |")
    A("")
    A(f"- Standard를 고른 이유 (D9): 부품 몸체 {F.v('가장자리 2.5 mm 안 몸체')}개가 기판 끝에서 2.5 mm 안에 있고, 그중 {F.v('가장자리 0.3 mm 안 몸체')} 는 "
      "Economic 방식이 권하는 0.3 mm보다 가깝습니다 (M; 0.3 mm와 2.5 mm는 계획 10장의 JLC 규정, F). 커넥터 위치는 기체에 맞춰 고정이라 옮길 수 없습니다. "
      "Standard는 JLC가 공정 레일을 붙이고, 리플로 온도도 더 낮습니다 (240 C, 계획 10장, F).")
    A(f"- 2.5 mm 안 몸체 {F.v('가장자리 2.5 mm 안 몸체')}개 중 JLC가 실장하는 것은 {F.v('가장자리 2.5 mm 안 JLC 실장')}개입니다 "
      f"(with_mcu {F.v('가장자리 2.5 mm 안 with_mcu')}개). 나머지 {F.v('가장자리 2.5 mm 안 JLC 미실장')} 은 JLC가 부품을 달지 않는 자리입니다 (M).")
    A(f"- 레일 쪽 (E): 표에서 '{F.v('레일 쪽')}' 쌍(거버 화면에서 위·아래의 76 mm 긴 변)을 골랐습니다. 이 경우 레일이 없는 쪽에 2.5 mm 안 부품이 "
      f"{F.v('레일 없는 쪽 2.5 mm 안')}개 남습니다. JLC가 받아 주는지는 DFM 결과로만 압니다.")
    A(f"- **레일을 붙이는 방식:** 구리 이격(M): {F.v('레일 변 구리 이격')}. JLC 기준(F)으로 V-cut 가장자리는 구리 이격이 "
      f"{F.v('V-cut 구리 이격')}, 라우팅·마우스바이트 가장자리는 {F.v('라우팅·마우스바이트 구리 이격')} 이어야 합니다. "
      f"레일이 붙는 두 변의 구리 이격 {F.v('레일 변 구리 이격 값')} mm는 라우팅 탭 기준에는 여유가 있지만, V-cut 기준값에는 여유 없이 딱 맞는 값입니다 "
      "(재배치 단계에서 면과 배선을 0.4 mm로 물림). 그 JLC 기준값은 검토 단계가 읽은 값(F)이고 이 단계에서 다시 읽지 않았으므로 "
      "**V-cut이 가능한지는 NOT_VERIFIED** 입니다. ESP-12F 몸체도 윗변에서 0.15 mm라, 메모에는 라우팅 탭 / 마우스바이트를 첫째로 부탁합니다. "
      "JLC가 어떤 방식을 받아 주는지는 DFM 결과로만 압니다 (NOT_VERIFIED).")
    A(f"- 툴링 홀 부품(H) 수: {F.v('툴링 홀(H) 부품')} (설계에 없음, M).")
    A("")
    A("메모 문장 (기본 묶음, 그대로 복사):")
    A("")
    remark_rails = ("Please add 5 mm process edge rails on the two 76 mm long edges (top and bottom in the Gerber view), "
                    "preferably attached with routed tabs / mouse bites: the ESP-12F body is 0.15 mm from the top edge. "
                    f"Copper (incl. the inner planes) keeps {F.v('레일 변 구리 이격 값')} mm from those two edges. "
                    "Several parts are closer than 2.5 mm to the board edge.")
    e_ = ctx["edge"]
    rail_cu = [e_["copper_to_edge_mm"]["min_per_side"][k] for k in ("+y", "-y")]
    if f"{min(rail_cu):g}" != F.v("레일 변 구리 이격 값") or \
            not any(r_[0] == "U7" and f"{r_[1]:g}" == "0.15" for r_ in e_["bodies_under_0p3"]):
        raise SystemExit("docs_ko: the English remark states the rail-edge copper distance and U7 0.15 mm from the edge; "
                         "edge_distance.json / routing_rules.json say otherwise")
    remark_tail = [
        f"The {F.v('main bom_lines_tht')} THT lines (J2/J5, J8-J12, J14, J15/J17, J16, J18) are to be soldered by JLCPCB.",
        f"{F.v('1번 핀 번호 반대')}: the pin-1 numbering of your library footprints is mirrored against ours. The CPL "
        "rotations already match the body / latch side to our silkscreen outline - please do NOT rotate them by 180 deg.",
        f"Empty by design, please do not add parts: {F.v('빈 자리')}.",
    ]
    A("```")
    A("Please bake the moisture-sensitive parts (ESP-12F C82891, LEDs C2286) before assembly.")
    A(remark_rails)
    for t in remark_tail:
        A(t)
    A("U1-U4 and J1 are intentionally NOT assembled (no BOM/CPL rows). Please do not add parts for them.")
    A("```")
    A("")
    A("with_mcu 묶음이면 첫 줄과 마지막 줄을 아래처럼 바꾼 메모를 쓰십시오 (ATmega128A C6510도 습기 민감 등급 MSL 3, F: JLC 부품 기록):")
    A("")
    A("```")
    A("Please bake the moisture-sensitive parts (ESP-12F C82891, LEDs C2286, ATmega128A C6510) before assembly.")
    A(remark_rails)
    for t in remark_tail:
        A(t)
    A("J1 is intentionally NOT assembled (no BOM/CPL row).")
    A("```")
    A("")
    A("## 3. 결제하기 전에 확인할 것 (하나라도 다르면 멈추고 결제하지 마십시오)")
    A("")
    A(f"1. 주문 당일, 업로드 직전에 재고를 다시 확인합니다: {p('scripts')} 에서 `B:/Claude/.venv/Scripts/python.exe -X utf8 -B jlc_gate.py` "
      f"(인터넷, 읽기만, 약 2~3분). 결과 {p('out/jlc_gate_report.json')} 의 status가 PASS여야 합니다. 마지막 확인: {F.v('JLC 확인 시각')} {F.v('JLC 확인 결과')}.")
    A(f"2. BOM 매칭 화면: 기본 묶음이면 {F.v('main bom_lines')}줄 모두, with_mcu면 {F.v('with_mcu bom_lines')}줄 모두 부품이 매칭되어야 합니다. "
      "'부품 없음', '다른 부품 추천', '재고 부족'이 하나라도 있으면 멈춥니다 (대체는 5장 표 1의 정해진 대체 부품만, 그것도 다시 확인 후; 표 2의 SMD 부품은 이 파일로 쓸 수 없음).")
    A(f"3. THT {F.v('main bom_lines_tht')}줄(J2/J5, J8~J12, J14, J15/J17, J16, J18; 납땜점 {F.v('THT 납땜점 main')}개)이 견적에 'JLC가 납땜'으로 들어가 있어야 합니다. "
      "빠져 있거나 거절되면 멈춥니다.")
    A(f"4. 부품 위치 미리보기(Confirm Parts Placement)를 {p('out/cpl_rotation_check.png')} 그림과 하나씩 비교합니다. 특히 극성: 탄탈 C45 / C52, 전해 C47 "
      f"(거꾸로면 화재 위험), 그리고 회전이 확인 안 된 {F.v('회전 미확인')}. U7 / U8 / U9 / J7의 1번 핀 방향. "
      f"회전 근거는 {p('out/cpl_rotation_table.csv')} 의 status 칸에 있습니다. 이것은 결제 전 브라우저 미리보기에서 사용자가 하는 확인입니다.")
    A(f"   **{F.v('1번 핀 번호 반대')}:** JLC 쪽 풋프린트는 1번 핀 번호가 KiCad와 반대로 매겨져 있어, 미리보기의 1번 표시가 실크와 반대쪽에 "
      "보이는 것이 정상입니다. 부품 몸체(커넥터는 걸쇠·몸통 쪽)가 실크 윤곽과 맞는지로 판단하고, **180도 돌리지 마십시오.** "
      "J14(배터리)를 돌리면 배터리 극성이 바뀝니다.")
    A(f"5. DFM 결과: 가장자리 가까운 부품({F.v('가장자리 0.3 mm 안 몸체')} 등)에 대한 이의, 레일 위치, 구멍·간격 지적이 없어야 합니다. "
      "레일을 V-cut으로 붙이겠다고 하면 ESP-12F 몸체(윗변에서 0.15 mm)에 문제가 없는지 JLC에 확인받은 뒤에만 진행합니다 (2장). "
      "JLC가 이의를 달면 주문 방식이나 부품을 마음대로 바꾸지 말고 멈춘 뒤 다시 검토합니다.")
    A("6. 견적 금액을 4장의 추정과 비교합니다. 크게 다르면 이유를 확인합니다.")
    A("7. 결제 뒤에 올 수 있는 확인: JLC 엔지니어의 DFM 검토와 유료 'Confirm Parts Placement'(부품 위치 확인)가 결제 전인지 후인지는 "
      "확인하지 못했습니다 (NOT_VERIFIED - 주문 흐름을 실제로 거친 적 없음). 결제 뒤 JLC가 위치 확인 사진이나 문제를 보내면 4번과 같은 기준으로 "
      "비교하고, 틀린 것이 있으면 바로잡아 달라고 답하거나, 판단이 서지 않으면 주문 보류(hold) 또는 취소를 요청한 뒤 다시 검토합니다.")
    A("")
    A("## 4. 예상 비용 (E, 추정)")
    A("")
    A("| 묶음 | 방식 | 실장 장수 | 합계 (E) | Standard 전용 수수료까지 붙으면 |")
    A("|---|---|---|---|---|")
    for r_ in ctx["fees"]:
        if (r_["pair"], r_["order"], r_["boards"]) in {("main", "Standard", "2"), ("main", "Standard", "5"), ("main", "Economic", "2"),
                                                        ("with_mcu", "Standard", "2")}:
            key = f"비용 {r_['pair']} {r_['order']} {r_['boards']}장"
            extra = F.v(key + " (Standard 전용 수수료 포함)") if r_.get("sum_E_with_standard_only_fees") else "-"
            A(f"| {'기본' if r_['pair'] == 'main' else 'with_mcu'} | {r_['order']} | {r_['boards']} | {F.v(key)} | {extra} |")
    A("")
    A(f"- 포함: 셋업, 스텐실, 부품 종류 수수료, 부품 장착, THT 손납땜, 위치 확인 수수료, 부품값(손실분·최소 수량 포함). Standard 전용 수수료 {F.v('Standard 전용 수수료')} "
      "(취급·고정구·포장)는 적용 여부가 NOT_VERIFIED입니다.")
    A("- **빠진 것:** PCB 제작비, ENIG 추가 요금, 베이킹 요금, 레일 분리 요금, 배송비, 세금. 실제 금액은 JLC 견적뿐입니다.")
    A(f"- 전체 표: {p('out/jlc_fee_estimate.csv')}")
    A("")
    A("## 5. 재고가 적은 줄과 대체 부품")
    A("")
    A(f"- 얇은 재고(기준 {F.v('얇은 재고 기준')} 미만): {F.v('얇은 재고 줄')}.")
    A(f"- 지켜볼 줄 (2000개 미만): {F.v('재고 주의 줄')}.")
    A("- 대체는 계획에 적힌 순서대로만 씁니다. 다른 부품으로 바꾸는 것은 설계 변경이라 다시 검사하고 사용자가 정해야 합니다.")
    A("")
    # split by footprint compatibility from the plan's own alternate notes (rev 3 review J1)
    notes = {a_["for"]: a_.get("note", "") for a_ in ctx["plan"]["alternates"]}
    same, check, newfp = [], [], []
    for k in alt:
        n = notes.get(k)
        if n is None:
            raise SystemExit(f"docs_ko: gate alternate group {k!r} has no note in spec/rev3_plan.json alternates")
        if "new footprint" in n or "footprint not chosen" in n:
            newfp.append(k)
        elif "same series and land pattern" in n or "same MPN" in n:
            same.append(k)
        else:
            check.append(k)
    A("**표 1. 같은 풋프린트에 쓸 수 있는 대체 부품** (이 파일 그대로, BOM 줄의 부품 번호만 바꿈). 'PASS'는 주문 가능 여부(재고)만 뜻합니다.")
    A("")
    A("| 원래 부품 | 대체 부품 (주문 가능 수량, 판정) | 풋프린트 호환 | 계획 메모 (spec/rev3_plan.json) |")
    A("|---|---|---|---|")
    for k in same:
        A(f"| {k} | {F.v('대체 ' + k)} | 같은 패키지 (F, 계획 메모); 미리보기로 확인 | {notes[k]} |")
    for k in check:
        A(f"| {k} | {F.v('대체 ' + k)} | **NOT_VERIFIED** - 쓰기 전에 제조사 도면과 KiCad 풋프린트를 대조해야 함 | {notes[k]} |")
    A("| U9 (C20345924) | 없음 - 같은 사양의 두 번째 부품이 없습니다. 재고가 없으면 주문을 미룹니다 (계획 7장, F) | - | - |")
    A("")
    A("**표 2. 이 파일로는 쓸 수 없는 대체 부품** (SMD 부품이라 새 풋프린트와 새 보드 판이 필요; JLC가 THT 손납땜을 거절할 때를 위한 계획의 예비안, "
      "사용자 승인 필요). BOM 매칭 화면에서 이 부품으로 바꾸면 안 됩니다.")
    A("")
    A("| 원래 부품 | 대체 부품 (주문 가능 수량, 판정) | 계획 메모 |")
    A("|---|---|---|")
    for k in newfp:
        A(f"| {k} | {F.v('대체 ' + k)} | {notes[k]} |")
    A("")
    A("## 6. 따로 사야 하는 것 (JLC는 낱개 부품을 보내주지 않습니다)")
    A("")
    A(f"목록 {F.v('따로 살 품목 수')}개: {p('out/off_board_purchase.csv')}. 재고와 가격은 확인하지 않았습니다 (NOT_VERIFIED).")
    A("")
    A("| 품목 | 보드당 | 2장 분 | 어디에 | 구입처 (제안) |")
    A("|---|---|---|---|---|")
    for r_ in ctx["offb"]:
        A(f"| {r_['item']} | {r_['qty per board']} | {r_['qty for 2 boards']} | {r_['plugs into / used at']} | {r_['purchase channel (suggestion)']} |")
    A("")
    A("## 7. 보드를 받은 뒤 할 일 (계획 15장 순서)")
    A("")
    A("1. **검사:** 탄탈 C45 / C52와 전해 C47의 극성을 실크 표시와 비교, U7 / U8 / U9 / J7 방향, U8(0.5 mm 피치) 납땜 브리지가 없는지 확대해서 봅니다.")
    A("2. **레일 떼기:** Standard 주문이면 보드 둘레의 레일을 꺾어 떼고 모서리를 다듬습니다.")
    A(f"3. **뒷면 다리 자르기:** 뒷면으로 나온 다리를 재서 {F.v('뒷면 다리 규칙')} 보다 긴 것은 모두 자릅니다. {F.v('다리 규칙 초과')} 은 반드시 (다리 4개). "
      "소켓·핀헤더(J2, J5, J8~J12, J16, J18)도 공차 끝이면 넘을 수 있으니 재 봅니다. 폼테이프는 그 뒤에 붙입니다.")
    A("4. **모듈·MCU 없이 벤치 전원 시험:** J14에 전류 제한(100 mA) 전원, J15는 단락. 아래 값을 적습니다. 범위를 벗어나면 멈춥니다.")
    A("")
    A("| 측정 | 허용 범위 (E, 계획) | 측정값 |")
    A("|---|---|---|")
    A(f"| 전압을 천천히 올릴 때 5 V가 켜지는 전압 | {F.v('켜짐 전압')} (3.3 V에서는 꺼져 있어야 함) | |")
    A(f"| 켜진 뒤 +5V | {F.v('+5V 범위')} | |")
    A("| 켜진 뒤 +3V3 | 3.0 V 이상 (ESP-12F 공급 범위, F) | |")
    A(f"| 전압을 천천히 내릴 때 꺼지는 전압 | {F.v('꺼짐 전압')} | |")
    A("")
    A("   켜지는 순간의 +5V와 EN 파형을 오실로스코프로 한 장 남깁니다. 같은 때 VBAT_LOGIC_SW(U8 입력, C41)의 파형도 남깁니다: 스위치 리드로 "
      "들어오는 전원은 세라믹 커패시터만 있어 울릴 수 있습니다 (VIN 절대 최대 7 V, F; 검토 E-R6, NOT_VERIFIED).")
    A("5. **부하 시험:** +5V에 0.27 A와 0.78 A, 전원 3.4 V와 3.7 V (전류 제한 2 A). 전압이 유지되고 U8 / L5가 약 60 C 아래인지 봅니다 (E, 계획 15장). "
      "실제 배터리·스위치·리드로 배터리에서 U8 입력까지 저항을 잽니다 (전압 강하 / 입력 전류): 0.29 Ohm 이하여야 합니다. 넘으면 비행 전에 보고합니다. "
      "(0장의 배선 폭 문제 때문에 이 측정이 특히 중요합니다.) "
      "0.78 A 부하에서 U8 SW 핀(L5 쪽 패드)과 +5V(C57)의 순간 과전압, U8 입력(VIN)의 리플을 짧은 접지선으로 오실로스코프에 담습니다 "
      "(0장 B: 승압부를 재배치했고 SW 선이 길어짐). SW가 7 V에 가까우면 비행 전에 보고합니다. "
      "MCU를 납땜한 뒤에는 같은 0.78 A 부하를 건 채로 NAV 칩 U1의 시리얼 통신이 깨지지 않는지도 확인합니다 "
      "(0장 B: 승압부가 U1의 16 MHz 크리스털에 rev 2c보다 가까움, 영향은 NOT_VERIFIED).")
    A(f"6. **MCU 4개 납땜** (JLC가 안 붙였을 때): 1번 핀을 표시에 맞추고, 가는 인두·플럭스로 0.8 mm 피치 핀을 모두 확대 검사합니다. "
      f"패드 1 mm 안에 이미 붙은 부품 수: {F.v('MCU 1 mm 안 부품')} (M). U1을 GY-521보다 먼저 납땜하십시오 (GY-521이 U1 위에 걸침). "
      f"그림: {p('out/render/c5_mcu_pin1.png')}")
    A(f"7. **모듈 장착:** 각 모듈에 인쇄된 핀 순서를 보드 실크와 먼저 비교합니다. GY-521은 자기 1x8 핀으로 J1에 납땜, 받침 {F.v('GY-521 받침 높이')} (A) "
      f"({p('out/render/e2a_gy521_J1.png')}); BMP280은 J2에 꽂음 (헤더 {F.v('BMP280 헤더 종류')}이 아니면 직각 핀을 먼저 납땜, {p('out/render/e2b_bmp280_J2.png')}); "
      f"OpenLog는 직각 6핀 헤더를 납땜한 뒤 microSD 쪽이 위로 오게 J5에 꽂음 ({p('out/render/e2c_openlog_J5.png')}); "
      f"NEO-7M은 윗뚜껑에 두고 JST-GH 케이블을 J7에 ({p('out/render/e3_gps_lead_J7.png')}). 받은 상태와 완성 상태 비교: "
      f"{p('out/render/e1_delivered_vs_completed.png')}, 전체 안내: {p('out/render/e4_overview_callouts.png')}. "
      f"OpenLog는 먼저 아랫면(ATmega328 쪽) 높이를 재고 끼워 보십시오: 아랫면 부품이 1.2 mm라고 보는 보수적 경우(A) 꽂는 길에서 "
      f"{F.v('모듈 간섭 보수적 충돌')} 이 겹칩니다.")
    A("8. **프로그래밍 (전원은 한 가지만):** 로직 배터리(3.73 V 넘게 충전) 또는 J14에 4.0 V 벤치 전원으로 보드를 켜고, USBasp의 타깃 전원 점퍼는 "
      "**빼 둡니다** (5 V를 두 곳에서 넣지 않기). 단자는 D6 선택대로 (헤더 납땜 + 점퍼, 또는 포고핀 어댑터). ESP-12F는 J3로 3.3 V 신호로만 굽습니다.")
    A("9. 첫 Wi-Fi 송신 때 +5V와 +3V3의 최저값을 기록합니다.")
    A("10. 커넥터 연결: 배터리 J14, 스위치 J15, GPS J7, 서보 J8~J11 / ESC J12, 아밍 J16, 부저 J17. 장착은 rev 2c와 같이 3 mm 폼테이프 + 케이블타이 1개 "
      "(x -3..+3 mm 띠). **장착 전에 띠를 다시 비우십시오:** 프로그래밍 헤더를 띠 구멍에 납땜했다면 떼거나 잘라 내야 케이블타이와 폼테이프가 "
      "제자리에 앉습니다 (뒷면으로 나온 다리도 1.5 mm 규칙). 비행용 보드는 포고핀 어댑터로만 굽는 것이 가장 간단합니다.")
    A("")
    A("## 8. MCU·센서 말고 사용자가 해야 하는 일 (숨기지 않고 모음)")
    A("")
    A("요청은 'MCU와 센서만 빼고 모두 JLC가 실장'이었지만, 아래 일은 이 보드에서 사용자 몫으로 남습니다:")
    A("")
    A("- Standard 주문의 레일 떼기 (7장 2).")
    A(f"- 뒷면 다리 자르기: {F.v('다리 규칙 초과')} (다리 4개, 도면값으로 규칙 초과), 공차 끝인 소켓·헤더 확인 (7장 3).")
    A(f"- 프로그래밍 단자 ({F.v('프로그래밍 헤더 선택')}안): J3, J19~J22 (2x3) 와 J24~J26 (1x2)는 빈 구멍입니다. 헤더를 직접 납땜(36곳, 계획 D6)하고 점퍼 3개를 끼우거나, "
      f"포고핀 어댑터를 대고 굽습니다 (접촉 NOT_VERIFIED). JLC가 띠 밖에 헤더를 실장하는 B안은 {F.v('B안 결론')} ({F.v('B안 2x3 자리')}). "
      "이 구멍들은 케이블타이 띠 안에 있으므로, 헤더를 납땜했다면 장착 전에 떼거나 잘라서 띠를 비워야 합니다 (7장 10번).")
    A("- OpenLog에 직각 6핀 헤더 납땜 (모듈 쪽 6곳; 보드에는 납땜 없음). OpenLog는 센서가 아니라 기록기라 D5에서 '센서처럼 사용자가 장착'으로 정했습니다.")
    A("- BMP280 헤더가 일자형으로 오면 직각 핀으로 바꿔 납땜.")
    A("- GPS 케이블(JST-GH 4선), 스위치·부저·아밍 리드 준비 (6장 목록).")
    A("- 벤치 전원 시험과 기록 (7장 4~5) - 보드가 만들어진 적이 없어서 필요합니다.")
    A("- 폼테이프와 케이블타이로 장착.")
    A("")
    A("## 9. 사용자 결정 D1~D17과 이번 파일에 쓴 값")
    A("")
    A("계획의 기본값으로 만들었습니다. '이번 파일에서 확인한 값'은 생성된 파일에서 읽은 것입니다.")
    A("")
    A("| Id | 질문과 기본값 (계획 19장) | 이번 파일에서 확인한 값 |")
    A("|---|---|---|")
    built = {
        "D1": f"U8 {F.v('코드 U8')}, U9 {F.v('코드 U9')}, L5 {F.v('코드 L5')} (BOM)",
        "D2": f"{F.v('UVLO 저항')} → 켜짐 {F.v('켜짐 전압')}, 꺼짐 {F.v('꺼짐 전압')} (E)",
        "D3": f"C45 {F.v('코드 C45')}, J8 {F.v('코드 J8')}, J2 / J5 {F.v('코드 J2')} (BOM)",
        "D4": f"기본 BOM에 U1~U4 {F.v('U1-U4 기본 BOM')}; with_mcu 쌍 U1~U4 = {F.v('U1-U4 with_mcu')} (두 쌍 모두 생성)",
        "D5": f"소켓 J5 {F.v('코드 J5')} (JLC), OpenLog는 따로 구매",
        "D6": f"{F.v('프로그래밍 헤더 선택')}안 (빈 구멍); B안 검색 {F.v('B안 2x3 자리')} → {F.v('B안 결론')}",
        "D7": f"J15 / J17 {F.v('코드 J15')} (THT); 다리 자르기 {F.v('다리 규칙 초과')}",
        "D8": f"Q1 등 {F.v('코드 Q1')} (BOM)",
        "D9": "주문 화면에서 사용자가 선택 (2장: Standard)",
        "D10": f"툴링 홀 부품 {F.v('툴링 홀(H) 부품')}개 (설계에 없음)",
        "D11": "주문 화면에서 사용자가 선택 (2장: PCB 5 / 실장 2)",
        "D12": f"J2 / J5 {F.v('코드 J2')}; 대체 순서는 5장",
        "D13": f"{F.v('분압 저항')} (그대로)",
        "D14": "주문 메모에 베이킹 요청 (2장)",
        "D15": "가정 치수 사용 (spec/module_dimensions.json, 그림과 검사에 '가정' 표시)",
        "D16": "주문 화면에서 사용자가 선택 (2장: ENIG, 녹색, 기본 적층)",
        "D17": "여분 기판 3장은 기구 확인용 (주문 화면 수량으로 정해짐)",
    }
    for d in ctx["plan"]["user_report_ko"]["decisions"]:
        did = d.split(":")[0]
        A(f"| {did} | {d.split(':', 1)[1].strip()} | {built.get(did, '-')} |")
    A(f"| 추가 (배선 후 발견) | 배터리 경로 폭 규칙 FAIL을 받아들이고 주문할지, 고친 뒤 주문할지 (0장 A) | 결정 전: 도구 판정 {F.v('출고 판정')} |")
    A(f"| 추가 (독립 검토 후) | 재배치 뒤에도 남은 승압부 FAIL ({F.v('승압부 배치 FAIL')})을 받아들이고 주문할지 (0장 B) | 결정 전 |")
    A("")
    A("## 10. 변경 범위 표 (rev 2c 대비, 부품마다 이유 분류)")
    A("")
    table = change_table(F, ctx)
    A(f"계획 3장의 표를 계획 파일에서 읽고, rev 3 부품 번호를 주문 BOM 파일과 대조했습니다 ({F.v('변경 행')}). 그대로인 부품은 표에서 뺐습니다.")
    A("")
    L.extend(table)
    A("")
    return "\n".join(L)


def main() -> int:
    F, ctx = gather()
    order_text = order(F, ctx)
    order_keys = list(F.used)
    F.used.clear()
    readme_text = readme(F, ctx)
    readme_keys = list(F.used)
    readme_text += "\n## 8. 이 문서의 숫자 출처\n\n" + "\n".join(F.table(readme_keys)) + "\n"
    order_text += "\n## 11. 이 문서의 숫자 출처\n\n" + "\n".join(F.table(order_keys)) + "\n"
    (ROOT / "README_ko.md").write_text(readme_text, encoding="utf-8", newline="\n")
    (OUT / "ORDER_ko.md").write_text(order_text, encoding="utf-8", newline="\n")
    print(json.dumps({"README_ko.md": {"facts": len(readme_keys)}, "out/ORDER_ko.md": {"facts": len(order_keys)},
                      "release_gate": F.data["출고 판정"][0]}, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
