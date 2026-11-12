"""User documents of the rev 3 carrier: README_ko.md (board folder), out/ORDER_ko.md (ordering guide) and the
purchase list out/따로_살_부품_ko.md (rows from scripts/purchase_list.py).

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

import purchase_list
from purchase_list import KO_NAME

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
#: redesign2 (2026-11-12): the user's requests, verbatim (task text of the redesign; F = the user's own words)
REDESIGN_REQUESTS = ("그럼 다시 설계를 진행해줘. 실패한게 있다면 완료되었다고 말하지마", "보드의 사이즈를 조정하는 방법은 없어?",
                     "옮겨도 됨")
#: the routing report of the committed board before redesign2 (evidence copy, written once by the redesign2 pipeline stage)
BEFORE_RD2 = "out/reports/routing_rules_before_redesign2.json"


def gnd_chain_mm(pcb_text: str, start: tuple[float, float]) -> tuple[float, list[tuple[float, float]]]:
    """Length of the F.Cu GND track run that starts at ``start`` (a via / track end) and follows connected GND F.Cu
    segments in both directions while the run does not branch (M: segment ends from the board file, 1 um tolerance).
    Returns the total length and the two end points."""
    segs = []
    for m in re.finditer(r"\n\t\(segment\n\t\t\(start ([\d.\-]+) ([\d.\-]+)\)\n\t\t\(end ([\d.\-]+) ([\d.\-]+)\)\n\t\t\(width [\d.]+\)\n"
                         r"(?:\t\t\(locked yes\)\n)?\t\t\(layer \"F\.Cu\"\)\n\t\t\(net \"GND\"\)", pcb_text):
        x1, y1, x2, y2 = (round(float(v), 3) for v in m.groups())
        segs.append(((x1, y1), (x2, y2)))
    at = collections.defaultdict(list)
    for i, (a, b) in enumerate(segs):
        at[a].append(i)
        at[b].append(i)
    s0 = (round(start[0], 3), round(start[1], 3))
    used: set[int] = set()
    total, ends = 0.0, []
    for first in list(at.get(s0, [])):
        if first in used:
            continue
        node, i = s0, first
        while True:
            used.add(i)
            a, b = segs[i]
            total += math.dist(a, b)
            node = b if a == node else a
            nxt = [j for j in at[node] if j not in used]
            if len(nxt) != 1:
                break
            i = nxt[0]
        ends.append(node)
    return round(total, 3), ends


def gnd_tracks_in_box(pcb_text: str, p: tuple[float, float], q: tuple[float, float], grow: float = 0.5) -> list[tuple[float, float]]:
    """[(width, total length)] of the GND F.Cu segments with both ends inside the box spanned by the via centres ``p``
    and ``q`` grown by ``grow``, without the via stubs (segments with an end on ``p`` or ``q``) (M: segment ends and
    widths from the board file)."""
    x0, x1 = min(p[0], q[0]) - grow, max(p[0], q[0]) + grow
    y0, y1 = min(p[1], q[1]) - grow, max(p[1], q[1]) + grow
    by_w: dict[float, float] = collections.defaultdict(float)
    for m in re.finditer(r"\n\t\(segment\n\t\t\(start ([\d.\-]+) ([\d.\-]+)\)\n\t\t\(end ([\d.\-]+) ([\d.\-]+)\)\n\t\t\(width ([\d.]+)\)\n"
                         r"(?:\t\t\(locked yes\)\n)?\t\t\(layer \"F\.Cu\"\)\n\t\t\(net \"GND\"\)", pcb_text):
        ax, ay, bx, by, w = (float(v) for v in m.groups())
        if any(math.dist(e, v) < 1e-3 for e in ((ax, ay), (bx, by)) for v in (p, q)):
            continue
        if all(x0 <= x <= x1 for x in (ax, bx)) and all(y0 <= y <= y1 for y in (ay, by)):
            by_w[w] += math.dist((ax, ay), (bx, by))
    return sorted((w, round(L, 3)) for w, L in by_w.items())


FIT_KO ={"jlc": "JLC가 실장 (주문 BOM / CPL에 들어감)", "jlc_optional": "기본은 사용자 납땜, with_mcu 파일 쌍을 쓰면 JLC 실장",
          "user": "사용자가 납땜", "none": "부품 없음 (빈 도금 구멍 / 미실장 / 구리 패턴만)"}


def load(rel: str) -> dict:
    return json.loads((ROOT / rel).read_text(encoding="utf-8"))


def ref_key(ref: str) -> tuple[str, int]:
    return (ref.rstrip("0123456789"), int("".join(c for c in ref if c.isdigit()) or 0))


def compact_refs(refs: list[str], sep: str = ", ") -> str:
    """['U1', 'U2', 'U3', 'U4', 'U7', 'J1'] -> 'U1-U4, U7, J1': three or more consecutive numbers of one prefix become
    a range; the prefix U comes first, the others in alphabetical order."""
    groups: dict[str, list[int]] = {}
    for r in refs:
        groups.setdefault(ref_key(r)[0], []).append(ref_key(r)[1])
    out: list[str] = []
    for pre in sorted(groups, key=lambda s: (s != "U", s)):
        nums = sorted(groups[pre])
        run = [nums[0]]
        for n in [*nums[1:], None]:
            if n is not None and n == run[-1] + 1:
                run.append(n)
                continue
            out += [f"{pre}{run[0]}-{pre}{run[-1]}"] if len(run) >= 3 else [f"{pre}{k}" for k in run]
            run = [n]
    return sep.join(out)


def order_remarks(spec: dict, tht_user: list[str], limit: int, rail_cu: str) -> dict:
    """The English assembly remark of both file pairs (one paragraph each) and the sentence to append when the order
    falls back to Standard PCBA. The order page's remark box takes ``limit`` characters (M, order page 2026-11-12):
    a longer text is an error, with and without the appended sentence. Since 2026-11-12 JLC fits SMT parts only: the
    remark says so, forbids any through-hole part and names every footprint that stays empty - the list is the set of
    spec parts whose fit is not JLC in that pair (never typed)."""
    out: dict = {"limit": limit}
    # redesign2: the y 135 side carries the boost tab, so it is no straight edge any more (no V-cut there)
    rails = ("If edge rails are needed: routed tabs / mouse bites please, no V-cut (one long edge has a tab; "
             f"copper is {rail_cu} mm from both long edges).")
    bs = spec["bom_sources"]
    if any(ln["process"] != "SMT" for ln in bs["board_lines"] + bs["with_mcu_extra_lines"]):
        raise SystemExit("docs_ko: a JLC BOM line is not SMT - the remark text says 'SMT assembly only'")
    for name, with_mcu in (("main", False), ("with_mcu", True)):
        fitted = ("jlc", "jlc_optional") if with_mcu else ("jlc",)
        absent = [q["refdes"] for q in spec["parts"] if q["fit"] not in fitted]
        if not tht_user or not set(tht_user) <= set(absent):
            raise SystemExit("docs_ko: the remark names the through-hole connectors as not assembled, the spec does not")
        mcus = [q["refdes"] for q in spec["parts"] if q["fit"] == "jlc_optional"]
        text = ("SMT assembly only, top side. Please fit NO through-hole part and do NO hand soldering: every "
                "through-hole connector is bought and soldered by the customer. "
                + (f"{compact_refs(mcus)} (in this BOM/CPL) are to be assembled. " if with_mcu else "")
                + "NOT assembled on purpose (no BOM/CPL rows) - please leave these footprints empty and add no parts: "
                f"{compact_refs(absent)}.")
        for what, t in ((name, text), (name + " + Standard sentence", text + " " + rails)):
            if len(t) > limit or "\n" in t or not t.isascii():
                raise SystemExit(f"docs_ko: assembly remark '{what}' has {len(t)} characters (limit {limit}), or a line "
                                 "break / non-ASCII character - shorten it")
        out[name] = {"text": text, "length": len(text), "length_with_standard_sentence": len(text) + 1 + len(rails),
                     "not_assembled": absent}
    out["standard_sentence"] = rails
    return out


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
    if b["outline_mm"].get("tab"):   # redesign2: the tab on the vehicle -y edge (scripts/outline.py)
        t_ = b["outline_mm"]["tab"]
        F.put("외곽 탭 (-y 변)", f"x {t_['x'][0]:g}..{t_['x'][1]:g}, y {t_['y'][0]:g}..{t_['y'][1]:g} mm, {t_['area_mm2']:g} mm2 "
              f"(전체 외접 {b['outline_mm']['bbox_size'][0]:g} x {b['outline_mm']['bbox_size'][1]:g} mm)", S + ".tab", t_["label"])
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
    F.put("레일 변 구리 이격", f"{rec['result']}: 레일이 붙는 두 긴 변(y 65 변, 그리고 y 135 변과 탭의 모든 변)에서 {rail_min:g} mm 이상 "
          f"(기준 {rec['limit_mm_rail_edges']:g} mm), "
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
    F.put("1번 핀 번호 반대(묶음)", compact_refs(mirrored).replace("-", "~") or "없음", "out/cpl_rotation_table.csv notes ('mirrored')", "M")
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
    cn_ = (man.get("step_docs", {}).get("step", {}) or {}).get("canonical_numbering")
    if cn_:
        F.put("STEP 번호 정렬", (f"kicad-cli가 쓴 STEP 항목 {cn_['entities']}개의 번호와 순서를 내용으로 다시 매김 (scripts/step_canon.py); "
                              f"다시 읽어 모든 항목이 같은 그래프인지 확인: {'통과' if cn_['ok'] else '실패 - kicad-cli 파일 그대로 둠'}")
              if cn_.get("entities") else f"실패 ({cn_.get('why', '')}) - kicad-cli 파일 그대로 둠",
              M + " step_docs.step.canonical_numbering", "M")
    else:
        F.put("STEP 번호 정렬", "없음 (kicad-cli 파일 그대로)", M + " step_docs.step", "M")
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
    # ------------------------------------------------------------------------------------------------------------
    # plan amendments 2026-11-12 (user decisions): ESP-12F U7 and the 12 THT connectors bought and soldered by the
    # user, Economic PCBA (SMT only), lead-free HASL
    am = [a_ for a_ in plan.get("amendments", []) if a_.get("date") == "2026-11-12"]
    if [a_.get("id") for a_ in am] != ["esp_by_user_economic_hasl", "tht_by_user"]:
        raise SystemExit("docs_ko: spec/rev3_plan.json does not carry the two amendments of 2026-11-12")
    F.put("변경 날짜", am[0]["date"], "spec/rev3_plan.json amendments", "F (사용자 결정)")
    F.put("변경 요청 원문 1", am[0]["request_verbatim"], "spec/rev3_plan.json amendments[0].request_verbatim", "F (사용자 요청 그대로)")
    F.put("변경 요청 원문 2", am[1]["request_verbatim"], "spec/rev3_plan.json amendments[1].request_verbatim", "F (사용자 요청 그대로)")
    bought = plan["bom_lines_now_bought_by_the_user"]
    ref_code = {r_: b_["jlc_code"] for b_ in bought for r_ in b_["designators"]}
    F.put("ESP 참고 부품", ref_code["U7"], "spec/rev3_plan.json bom_lines_now_bought_by_the_user (U7)", "F")
    for ref in ("J2", "J8", "J14", "J15", "J16", "J18"):
        F.put(f"참고 코드 {ref}", ref_code[ref], f"spec/rev3_plan.json bom_lines_now_bought_by_the_user ({ref})", "F")
    if parts["U7"]["fit"] != "user" or code.get("U7") or mcu_codes.get("U7"):
        raise SystemExit("docs_ko: the documents are written for U7 fitted by the user and absent from both BOMs")
    us = fit.get("user_soldered_smd") or {}
    if sorted(us) != ["U7"]:
        raise SystemExit(f"docs_ko: fit_checks.json user_soldered_smd is {sorted(us)}; the text is written for U7 only")
    u7 = us["U7"]
    FC = "out/fit_checks.json user_soldered_smd.U7"
    F.put("ESP 패드 수", u7["pads"], FC + " pads", "M")
    F.put("ESP 네트 있는 패드", f"{len(u7['pads_with_a_net'])}개 ({', '.join(u7['pads_with_a_net'])}번)", FC + " pads_with_a_net", "M")
    F.put("ESP 네트 없는 패드", u7["pads_without_a_net"], FC + " pads_without_a_net", "M")
    F.put("ESP 패드 크기", " 와 ".join(f"{a_:g} x {b__:g} mm" for a_, b__ in sorted(u7["pad_sizes_mm"], reverse=True)), FC + " pad_sizes_mm", "M")
    F.put("ESP 옆 JLC 부품", ", ".join(f"{r_} {g_:g} mm" for r_, g_, _h in u7["jlc_fitted_neighbours_within_1mm_of_the_pads"]),
          FC + " jlc_fitted_neighbours_within_1mm_of_the_pads (ESP 패드에서 1 mm 안)", "M")
    F.put("ESP 패드-이웃 패드 최소", f"{u7['min_pad_to_foreign_pad_mm']:g} mm", FC + " min_pad_to_foreign_pad_mm", "M")
    F.put("ESP 몸체 높이", f"{u7['body_height_mm']:g} mm", FC + " body_height_mm", "A")
    mring = u7["mcu_pad_ring_to_body_mm"]
    near_mcu = sorted(mring, key=lambda k_: (mring[k_], k_))[:2]
    F.put("ESP-MCU 거리", ", ".join(f"{k_} 패드에서 {mring[k_]:g} mm" for k_ in near_mcu), FC + " mcu_pad_ring_to_body_mm", "M")
    F.put("ESP에 가까운 MCU", near_mcu[0], FC + " mcu_pad_ring_to_body_mm", "M")
    F.put("MCU 몸체 높이", f"{u7['mcu_body_height_mm'][near_mcu[0]]:g} mm", FC + " mcu_body_height_mm", "A")
    F.put("ESP 옆 빈 헤더 구멍", ", ".join(f"{r_} {g_:g} mm" for r_, g_, _b in u7["unfitted_header_holes_within_3mm"]) or "없음",
          FC + " unfitted_header_holes_within_3mm (ESP 패드에서)", "M")
    for name in ("main", "with_mcu"):
        n_ = pv.get(name, {}).get("pads_changed_per_ref", {}).get("U7", 0)
        if n_ != u7["pads"] or pv.get(name, {}).get("paste_pads_left_on_parts_not_fitted_by_jlc") != 0:
            raise SystemExit(f"docs_ko: {name} gerber set: paste removed on {n_} of {u7['pads']} U7 pads")
        F.put(f"ESP 페이스트 뺀 패드 {name}", n_, f"out/reports/gerber_check.json paste_variant.{name}.pads_changed_per_ref.U7", "M")
    # ---- the 12 through-hole connectors the user buys and solders (amendment 'tht_by_user')
    ut = fit.get("user_soldered_tht") or {}
    per = ut.get("per_ref") or {}
    tht_lines = [b_ for b_ in bought if b_["assembly_process"] == "THT"]
    tht_refs = sorted((r_ for b_ in tht_lines for r_ in b_["designators"]), key=ref_key)
    if sorted(per, key=ref_key) != tht_refs or any(parts[r_]["fit"] != "user" or code.get(r_) or mcu_codes.get(r_) for r_ in tht_refs):
        raise SystemExit("docs_ko: the documents are written for the 12 THT connectors fitted by the user and absent from both BOMs")
    if any(pairs[n_]["bom_lines_tht"] != 0 for n_ in pairs):
        raise SystemExit("docs_ko: a JLC BOM still has a THT line - the documents say 'SMT only'")
    FT = "out/fit_checks.json user_soldered_tht"
    F.put("스루홀 부품 수", len(tht_refs), FT + ".summary.parts", "M")
    F.put("스루홀 종류 수", len(tht_lines), "spec/rev3_plan.json bom_lines_now_bought_by_the_user (THT)", "F")
    F.put("스루홀 납땜점", ut["summary"]["joints_per_board"], FT + ".summary.joints_per_board", "M")
    F.put("스루홀 목록", compact_refs(tht_refs).replace("-", "~"), S + " parts[].fit == user (스루홀 커넥터)", "M")
    F.put("스루홀 참고 번호", ", ".join(f"{b_['jlc_code']} ({compact_refs(b_['designators'], '/').replace('-', '~')})" for b_ in tht_lines),
          "spec/rev3_plan.json bom_lines_now_bought_by_the_user", "F")
    F.put("J1 납땜점", ut["summary"]["other_user_soldered_holes"].get("J1", "-"), FT + ".summary.other_user_soldered_holes", "M")
    DIR_KO = {"left": "왼쪽", "right": "오른쪽", "up": "위쪽", "down": "아래쪽"}
    tht_table = []
    for ref in tht_refs:
        t_ = per[ref]
        name_ko = purchase_list.THT_TEXT[t_["reference_part"]["lcsc"]]["item_ko"]
        horizontal = "Horizontal" in t_["footprint"]
        side = t_["body_outline_from_the_pin_row_mm"]
        big = t_["body_outline_larger_side_on_the_top_view"]
        if big not in DIR_KO:
            raise SystemExit(f"docs_ko: {ref} has a symmetric body outline - the orientation text needs an asymmetric one")
        small = next(k_ for k_ in side if k_ != big)
        if horizontal:
            body = f"직각형: 몸체가 핀 줄에서 {DIR_KO[big]}으로 {side[big]:g} mm 뻗음 (반대쪽 {side[small]:g} mm) → {DIR_KO[big]}을 보게 눕힘"
        else:
            body = (f"수직형: 핀 줄에서 몸체 윤곽까지 {DIR_KO[big]} {side[big]:g} mm, {DIR_KO[small]} {side[small]:g} mm "
                    f"→ 윤곽이 넓은 쪽이 {DIR_KO[big]}")
        p1 = t_["pad1"]
        F.put(f"커넥터 {ref} 1번 핀", f"{p1['net'] if not p1['net'].startswith('unconnected-') else '연결 없음'}, 핀 줄의 "
              f"{DIR_KO[p1['side_of_the_pin_row_on_the_top_view']]} 끝 ({'네모난' if 'rectangle' in p1['shape'] else p1['shape']} 패드)",
              FT + f".per_ref.{ref}.pad1", "M")
        F.put(f"커넥터 {ref} 몸체", body, FT + f".per_ref.{ref}.body_outline_from_the_pin_row_mm", "M")
        F.put(f"커넥터 {ref} 가까운 변", f"{DIR_KO[t_['nearest_board_edge_on_the_top_view']]} 변에서 {t_['body_to_that_edge_mm']:g} mm",
              FT + f".per_ref.{ref}.body_to_that_edge_mm", "M")
        F.put(f"커넥터 {ref} 납땜점", t_["joints"], FT + f".per_ref.{ref}.joints", "M")
        F.put(f"커넥터 {ref} 높이", f"{t_['height_mm']:g} mm", FT + f".per_ref.{ref}.height_mm", "A")
        tht_table.append((ref, name_ko, t_["reference_part"]["mpn"], t_["reference_part"]["lcsc"]))
    # which connectors stand close to a chip the user solders: gap body -> MCU pad ring / ESP pads below 5 mm
    NEAR = 5.0
    near_m, near_e = [], []
    for ref in tht_refs:
        t_ = per[ref]
        g_ = t_["gap_to_mcu_pad_rings_mm"]
        hit = sorted((u_ for u_ in g_ if g_[u_] < NEAR), key=lambda u_: (g_[u_], u_))
        if hit:
            near_m.append(f"{ref} ({', '.join(f'{u_} 패드에서 {g_[u_]:g} mm' for u_ in hit)}; 높이 {t_['height_mm']:g} mm)")
        e_ = t_["gap_to_user_smd_pads_mm"].get("U7")
        if e_ is not None and e_ < NEAR:
            near_e.append(f"{ref} ({e_:g} mm)")
    F.put("MCU 가까운 커넥터", near_m, FT + ".per_ref[].gap_to_mcu_pad_rings_mm (5 mm 미만; 높이는 A)", "M")
    F.put("ESP 가까운 커넥터", near_e, FT + ".per_ref[].gap_to_user_smd_pads_mm.U7 (5 mm 미만)", "M")
    for ref in ("J14", "J15"):
        t_ = per[ref]
        F.put(f"{ref} 칩 거리", f"MCU 패드에서 {min(t_['gap_to_mcu_pad_rings_mm'].values()):g} mm 이상, ESP 패드에서 "
              f"{t_['gap_to_user_smd_pads_mm']['U7']:g} mm", FT + f".per_ref.{ref}.gap_to_mcu_pad_rings_mm / gap_to_user_smd_pads_mm", "M")
    F.put("J14 1번 핀 네트", per["J14"]["pad1"]["net"], FT + ".per_ref.J14.pad1.net", "M")
    j14_side = per["J14"]["pad1"]["side_of_the_pin_row_on_the_top_view"]
    if j14_side not in ("left", "right") or per["J14"]["pad1"]["net"] != "VBAT_LOGIC_RAW":
        raise SystemExit("docs_ko: the J14 polarity text is written for a left-right pin row with pad 1 = VBAT_LOGIC_RAW")
    F.put("J14 아랫면에서 1번 핀", {"left": "오른쪽", "right": "왼쪽"}[j14_side],
          FT + ".per_ref.J14.pad1.side_of_the_pin_row_on_the_top_view (윗면 기준을 좌우로 뒤집음)", "M")
    # esp-user-fix step (finding V1): the glyph order of the pin-ordered names, measured by routing_checks.py
    PL = "out/reports/routing_rules.json silkscreen.pin_order_labels"
    pol = rr["checks"]["silkscreen"].get("pin_order_labels")
    if not pol or not pol["ok"]:
        raise SystemExit("docs_ko: routing_rules.json has no passing pin_order_labels block - run routing_checks.py "
                         f"({(pol or {}).get('problems')})")
    lab = {r_["ref"]: r_ for r_ in pol["labels"]}
    j14l = lab.get("J14")
    if (not j14l or j14l["relation"] != "along the pin row" or j14l["layer"] != "B.Silkscreen"
            or j14l["glyphs_from_pin_1"] != "+-" or j14l["first_glyph_is_pad"] != "2"):
        raise SystemExit("docs_ko: the J14 label text is written for a bottom label along the pin row that reads "
                         "'-' (pad 2) first and '+' (pad 1) last")
    side_short = {"B.Silkscreen": "아랫면", "F.Silkscreen": "윗면"}
    F.put("J14 라벨", f"'{j14l['text']}'", PL + ".labels[J14].text", "M")
    F.put("J14 라벨 끝 글자", "'+' = 1번 핀 (네모난 패드)", PL + ".labels[J14].last_glyph_is_pad", "M")
    F.put("라벨 검사", "PASS", PL + ".ok", "M")
    along_l = [f"{r_['ref']} {side_short[r_['layer']]} '{r_['text']}'" for r_ in pol["labels"] if r_["relation"] == "along the pin row"]
    from_1 = [r_["ref"] for r_ in pol["labels"] if r_["relation"] != "along the pin row"]
    F.put("핀 줄과 나란한 라벨", along_l, PL + ".labels (relation = along the pin row)", "M")
    F.put("아랫면 나란한 라벨", [f"{r_['ref']} '{r_['text']}'" for r_ in pol["labels"]
                          if r_["relation"] == "along the pin row" and r_["layer"] == "B.Silkscreen"],
          PL + ".labels (relation = along the pin row, layer B.Silkscreen)", "M")
    F.put("1번 핀부터 적은 라벨", compact_refs(sorted(from_1, key=lambda r_: int(r_[1:]))).replace("-", "~"),
          PL + ".labels (relation = across the pin row / not a single pin row)", "M")
    lead_rows = {r_["ref"]: r_ for r_ in tl["rows"]}
    tol_over = [r_ for r_ in tht_refs if lead_rows[r_].get("under_board_max_mm") is not None
                and lead_rows[r_]["under_board_nominal_mm"] <= tl["rule_mm"] < lead_rows[r_]["under_board_max_mm"]]
    F.put("다리 공차 끝 초과", compact_refs(tol_over).replace("-", "~") or "없음",
          "out/fit_checks.json tht_leads.rows (도면값은 규칙 안, 공차 끝은 규칙 초과; 참고 부품의 도면값)", "F (도면값)")
    F.put("다리 규칙 초과 다리 수", sum(lead_rows[r_]["plated_holes"] for r_ in tl["over_rule_nominal"]),
          "out/fit_checks.json tht_leads.rows (plated_holes)", "M")
    by_h = collections.defaultdict(list)
    for ref in tht_refs:
        if ref not in ("J14", "J15"):
            by_h[per[ref]["height_mm"]].append(ref)
    F.put("나머지 커넥터 높이순", " → ".join(f"{compact_refs(by_h[h_]).replace('-', '~')} (높이 {h_:g} mm)" for h_ in sorted(by_h)),
          FT + ".per_ref[].height_mm (J14, J15 제외, 낮은 것부터)", "A (높이는 가정값)")
    F.put("나머지 커넥터 수", sum(len(v_) for v_ in by_h.values()), FT + ".per_ref (J14, J15 제외)", "M")
    F.put("나머지 커넥터 납땜점", sum(per[r_]["joints"] for v_ in by_h.values() for r_ in v_), FT + ".per_ref[].joints (J14, J15 제외)", "M")
    # ---- purchase list (the same rows as out/off_board_purchase.csv)
    prow = purchase_list.build(spec, plan, fit)
    want_csv = [[str(c_) for c_ in row] for row in purchase_list.csv_rows(prow)]
    have_csv = [[r_[h_] for h_ in purchase_list.CSV_HEADER] for r_ in offb]
    if want_csv != have_csv:
        raise SystemExit("docs_ko: out/off_board_purchase.csv differs from scripts/purchase_list.py - run checks_exports.py")
    cost_a = purchase_list.group_a_cost(prow)
    snap = purchase_list.load_lcsc()
    times = sorted(r_["fetched_at"] for r_ in snap.get("records", {}).values() if r_.get("ok"))
    F.put("LCSC 기록 시각", f"{times[0]} ~ {times[-1][11:]}" if times else "없음", "spec/lcsc_snapshot_20261112.json records[].fetched_at",
          "M" if times else "NOT_VERIFIED")
    if cost_a:
        F.put("LCSC (a) 묶음 값", f"{cost_a['sum_usd']:.2f} USD", "spec/lcsc_snapshot_20261112.json 의 단가 x (최소 주문 수량을 지킨) 수량, "
              "scripts/purchase_list.py group_a_cost", "E (단가는 M, 그 시각 기준)")
        F.put("LCSC 커넥터 값", f"{cost_a['tht_sum_usd']:.2f} USD", "spec/lcsc_snapshot_20261112.json, scripts/purchase_list.py group_a_cost", "E (단가는 M, 그 시각 기준)")
        F.put("LCSC 재고 부족", cost_a["short"], "spec/lcsc_snapshot_20261112.json records[].stock", "M (그 시각 기준)")
    else:
        for k_ in ("LCSC (a) 묶음 값", "LCSC 커넥터 값", "LCSC 재고 부족"):
            F.put(k_, "NOT_VERIFIED", "spec/lcsc_snapshot_20261112.json 없음", "NOT_VERIFIED")
    fin = re.findall(r'\(copper_finish "([^"]*)"\)', pcb_text)
    if len(fin) != 1:
        raise SystemExit(f"docs_ko: {len(fin)} copper_finish entries in the board file")
    F.put("보드 표면처리", fin[0], BRD + " stackup copper_finish", "M")
    g_ = reuse.get("unrouted_board_guard") or {}
    F.put("배선 재생 입력 보드", g_.get("accepted", "-"), "kicad/work/route_reuse_log.json unrouted_board_guard", "M")
    jp = edge["jlc_placed_bodies"]["default_order"]
    EJ = "out/reports/edge_distance.json jlc_placed_bodies.default_order"
    F.put("JLC 실장 몸체 0.3 mm 안", [f"{r_} {d_:g} mm" for r_, d_, _s in jp["bodies_under_0p3"]], EJ + " bodies_under_0p3", "M")
    F.put("JLC 실장 가까운 몸체", ", ".join(f"{r_} {d_:g} mm" for r_, d_, _s in jp["closest"]), EJ + " closest", "M")
    F.put("JLC 실장 가장 가까운 몸체", f"{jp['closest'][0][0]} {jp['closest'][0][1]:g} mm", EJ + " closest[0]", "M")
    F.put("JLC 실장 패드 0.3 mm 안", [f"{r_} {d_:g} mm" for r_, d_, _s in jp["pads_under_0p3"]], EJ + " pads_under_0p3", "M")
    F.put("JLC 미실장 몸체 0.3 mm 안", [f"{r_[0]} {r_[1]:g} mm" for r_ in edge["bodies_under_0p3"] if r_[3] != "jlc"],
          "out/reports/edge_distance.json bodies_under_0p3 (fit != jlc)", "M")
    F.put("U7 몸체-변 거리", next((f"{r_[1]:g} mm" for r_ in edge["bodies_under_0p3"] if r_[0] == "U7"), "0.3 mm 이상"),
          "out/reports/edge_distance.json bodies_under_0p3 (U7)", "M")
    placement = load("kicad/placement.json")
    F.put("부품 자리 점유율", f"{placement['placement_checks']['courtyard_occupancy_percent']:g} %",
          "kicad/placement.json placement_checks.courtyard_occupancy_percent (윗면 부품 자리 면적 / 케이블타이 띠를 뺀 기판 면적)", "M")
    # ---- redesign2 (2026-11-12): outline tab, three programming holes, C42, boost block, battery path
    F.put("재설계 요청 원문", " / ".join(f"\"{t_}\"" for t_ in REDESIGN_REQUESTS),
          "사용자 요청 2026-11-12 (재설계 작업 지시에 옮겨 적힌 원문)", "F (사용자 요청 그대로)")
    rd_all = load(BEFORE_RD2)
    rd, rdr = rd_all["checks"], rd_all["results"]
    F.put("재설계 전 배선 규칙", f"{sum(1 for x in rdr.values() if x == 'PASS')} / {len(rdr)} PASS, FAIL: "
          + (", ".join(k for k, x in rdr.items() if x != "PASS") or "없음"), BEFORE_RD2 + " results (재설계 전 보드)", "M")
    F.put("재설계 전 보드 sha256", rd_all["board"]["sha256"][:16] + "...", BEFORE_RD2 + " board.sha256", "M")
    rbp = rd["battery_path"]
    F.put("재설계 전 배터리 경로", f"20 C {rbp['copper_resistance_ohm_20C']:g} Ohm, 60 C {rbp['copper_resistance_ohm_60C']:g} Ohm, "
          f"2.0 mm보다 좁은 길이 {rbp['current_carrying_length_narrower_than_2mm']:g} mm", BEFORE_RD2 + " battery_path (재설계 전 보드)",
          "E (저항), M (좁은 길이)")
    F.put("재설계 전 출력 커패시터 간격", "; ".join(f"{c['cap']} {c['gap_pad1_to_U8.6_mm']:g} / {c['gap_pad2_to_U8.4_mm']:g} mm"
                                          for c in rd["hot_loop"]["caps"]), BEFORE_RD2 + " hot_loop (재설계 전 보드)", "M")
    F.put("재설계 전 SW 패드 간격", f"{rd['sw_node']['pad_edge_gap_U8.5_L5.2_mm']:g} mm", BEFORE_RD2 + " sw_node (재설계 전 보드)", "M")
    F.put("재설계 전 C41-L5 경로", f"{rd['input_cap_l5']['copper_path_to_L5.1_mm']:g} mm", BEFORE_RD2 + " input_cap_l5 (재설계 전 보드)", "M (배선 모델, E)")
    F.put("재설계 전 크리스털보다 가까운 부품", rd["crystal_distance"]["parts_closer_than_the_rev2c_pins"],
          BEFORE_RD2 + " crystal_distance (재설계 전 보드)", "M")
    BM = "out/reports/board_metrics.json board_fit"
    bfit = load("out/reports/board_metrics.json")["board_fit"]
    ol = bfit["outline"]
    tk, mr, bb = ol["tab_kicad_mm"], ol["main_rect_kicad_mm"], ol["bbox_kicad_mm"]
    F.put("본체 사각형", f"KiCad x {mr[0]:g}..{mr[2]:g}, y {mr[1]:g}..{mr[3]:g} mm ({mr[2] - mr[0]:g} x {mr[3] - mr[1]:g} mm, "
          f"모서리 r {ol['corner_radius_mm']:g} mm)", BM + ".outline", "M")
    F.put("탭 위치", f"KiCad x {tk['x'][0]:g}..{tk['x'][1]:g}, y {tk['y'][0]:g}..{tk['y'][1]:g} mm (바깥 모서리 r {tk['outer_corner_radius_mm']:g} mm, "
          f"본체와 만나는 안쪽 모서리 r {tk['inner_fillet_radius_mm']:g} mm, 기판 재료는 x {tk['edge_cuts_material_x'][0]:g}..{tk['edge_cuts_material_x'][1]:g})",
          BM + ".outline.tab_kicad_mm", "M")
    F.put("탭 넓이", f"{ol['tab_area_mm2']:g} mm2", BM + ".outline.tab_area_mm2", "M")
    F.put("외접 크기", f"{bb[2] - bb[0]:g} x {bb[3] - bb[1]:g} mm", BM + ".outline.bbox_kicad_mm", "M")
    F.put("기판 넓이", f"{ol['area_mm2']:g} mm2", BM + ".outline.area_mm2", "M")
    F.put("탭 질량 추정", f"{ol['tab_area_mm2'] * 1.6 * 1.85 / 1000:.2f} g", BM + ".outline.tab_area_mm2 x 1.6 mm x 1.85 g/cm3 (FR4 밀도 A)", "E")
    fe = ol["fit_envelope"]
    F.put("Fit 허용 범위", f"KiCad x {fe['x_min']:g}..{fe['x_max']:g}, 바깥 변 y {fe['y_edge_max_r1']:g} mm 이하 (바깥 모서리 r 1 mm)",
          BM + ".outline.fit_envelope (redesign2 Fit 단계가 기체 CAD에서 잰 허용 범위)", "M")
    rl = bfit["rules"]
    F.put("기체 맞춤 판정", "PASS" if bfit["ok"] else "FAIL: " + "; ".join(bfit["problems"]), BM + ".ok", "M")
    F.put("기체 맞춤", f"흡입구 보어 벽까지 {bfit['gap_to_bore_mm']:g} mm (기준 {rl['bore_gap_min_mm']:g} mm 이상), "
          f"탭에서 270도 통풍구까지 {bfit['tab_to_vent_270_mm']:g} mm (기준 {rl['tab_vent_gap_min_mm']:g} mm 이상), "
          f"탭에서 모터 리드 연결부 영역까지 {bfit['motor_joint_zone_gap_mm']:g} mm (본체 사각형 쪽은 재설계 전과 같음), "
          f"315도 조인트 나사에서 평면 {rl['screw_plan_gap_mm']:g} mm 안의 부품: "
          + (", ".join(bfit["parts_within_plan_gap_of_the_315_screw"]) or "없음")
          + f", 나사 아래 기판 윗면 높이 여유 {bfit['screw_z_gap_board_top_mm']:g} mm (기준 {rl['screw_z_gap_min_mm']:g} mm 이상), "
          f"탭 안 부품 {', '.join(bfit['parts_in_the_tab'])} (높이 한계 {rl['height_limit_mm']:g} mm 넘는 것: "
          + (", ".join(bfit["parts_in_the_tab_above_limit"]) or "없음") + ")",
          BM, "M (나사 길이는 A / NOT_VERIFIED)")
    rv = placement["placement_checks"]["rev3"]["rev2c"]
    ud = rv["fixed_parts_at_user_decided_spot"]
    if sorted(ud) != ["J19", "J22", "J26"] or rv["user_decided_reference_offsets_mm"] != ud:
        raise SystemExit("docs_ko: placement.json does not report J19 / J22 / J26 at the user-decided spot")
    PR = "kicad/placement.json placement_checks.rev3.rev2c"
    F.put("옮긴 프로그래밍 구멍", ", ".join(f"{r_} (x {o_[0]:+g}, y {o_[1]:+g} mm)" for r_, o_ in sorted(ud.items(), key=lambda kv: ref_key(kv[0]))),
          PR + ".fixed_parts_at_user_decided_spot", "M")
    F.put("고정 부품 비교", f"{rv['fixed_parts_compared']}개 중 rev 2c 자리에서 움직인 것: " + (", ".join(rv["fixed_parts_moved"]) or "없음"),
          PR + " (fixed_parts_compared, fixed_parts_moved)", "M")
    pl = placement["placements"]
    c42w = re.search(r"([+-]\d+(?:\.\d+)?) mm in x", pl["C42"]["why"])
    if not c42w:
        raise SystemExit("docs_ko: placement.json C42 'why' does not state its x shift")
    F.put("C42 이동", f"x {c42w.group(1)} mm (지금 x {pl['C42']['x']:g}, y {pl['C42']['y']:g} mm)", "kicad/placement.json placements.C42 (x, y, why)", "M")
    F.put("승압부 자리", "; ".join(f"{r_} ({pl[r_]['x']:g}, {pl[r_]['y']:g}) {pl[r_]['rot']:g}도" for r_ in ("U8", "L5", "C43", "C57", "C41", "R63", "C58", "R64")),
          "kicad/placement.json placements (KiCad mm, 회전)", "M")
    legs_t = "; ".join(f"{lg['from']} → {lg['to']} {lg['ohm']:g} Ohm, {lg['length_carrying_mm']:g} mm, 가장 좁은 곳 {lg['min_width_carrying_mm']:g} mm, "
                       f"비아 {lg['vias_carrying_ge_20pct']}개" for lg in bp["legs"])
    F.put("배터리 경로 구간", legs_t, "out/reports/routing_rules.json battery_path.legs", "E (저항), M (길이·폭)")
    F.put("배터리 경로 최소 폭", f"{min(lg['min_width_carrying_mm'] for lg in bp['legs']):g} mm", "out/reports/routing_rules.json battery_path.legs", "M")
    tgc = rr["checks"]["hot_loop_top_gnd"]
    c41v = tgc["own_via_at"]["C41.2"]
    glen, gends = gnd_chain_mm(pcb_text, tuple(c41v))
    F.put("C41 GND 윗면 길", f"C41 GND 패드의 비아 ({c41v[0]:g}, {c41v[1]:g})에서 윗면 GND 선을 따라 {glen:g} mm "
          f"(끝점 {', '.join(f'({x_:g}, {y_:g})' for x_, y_ in gends)}; 가는 0.2 mm 선이 FB 분압 저항 두 줄 사이를 지나 C43 GND 패드를 거쳐 U8 GND 핀으로 감), "
          f"C41 GND 패드-U8.4 직선 간격 {tgc['pad_edge_gap_mm']['C41.2 -> U8.4']:g} mm",
          "kicad/vtol_carrier.kicad_pcb GND F.Cu 선 조각 (scripts/docs_ko.py gnd_chain_mm), out/reports/routing_rules.json hot_loop_top_gnd", "M")
    F.put("C41 GND 비아 거리", f"{next(v_['within_1.5mm_of']['C41.2'] for v_ in tgc['gnd_vias_near_the_pads'] if 'C41.2' in v_['within_1.5mm_of']):g} mm",
          "out/reports/routing_rules.json hot_loop_top_gnd.gnd_vias_near_the_pads", "M")
    F.put("SW 간격 여유", f"{sn['limit_mm'] - sn['pad_edge_gap_U8.5_L5.2_mm']:.3f} mm", "out/reports/routing_rules.json sw_node (기준 - 간격)", "M")
    # redesign2 fix stage (verifier findings): the real HF return paths on In1 and the thin C43 GND track
    ov_ = tgc["own_via_at"]
    F.put("내층 귀환 거리", "; ".join(f"{a_} GND 비아 → U8.4 GND 비아 {math.dist(ov_[a_], ov_['U8.4']):.2f} mm" for a_ in ("C43.2", "C57.2", "C41.2"))
          + " (비아 중심 사이 직선, In1 GND 면 위)",
          "out/reports/routing_rules.json hot_loop_top_gnd.own_via_at (비아 위치 M) 사이 직선 거리", "E")
    cwl_ = gnd_tracks_in_box(pcb_text, ov_["C43.2"], ov_["U8.4"])
    F.put("C43 GND 선", "C43 GND 패드에서 U8 GND 핀까지 윗면 GND 선 " + ", ".join(f"폭 {w__:g} mm {l__:g} mm" for w__, l__ in cwl_)
          + " (비아 연결선 제외; SOT-563 패드 두 줄 사이를 지나는 가는 선; 패드마다 0.3 mm GND 비아 하나씩)",
          "kicad/vtol_carrier.kicad_pcb GND F.Cu 선 (C43.2 비아와 U8.4 비아가 이루는 상자 + 0.5 mm 안, scripts/docs_ko.py gnd_tracks_in_box)", "M")
    oth_ = bfit.get("information_parts_within_plan_gap_of_the_other_screws_unchanged_corners", [])
    F.put("다른 조인트 나사 옆 부품", ", ".join(f"{o_['screw_deg']}도 나사 - {o_['ref']} {o_['plan_gap_courtyard_mm']:g} mm" for o_ in oth_) or "없음",
          BM + ".information_parts_within_plan_gap_of_the_other_screws_unchanged_corners (나사 끝 r 49.5 mm 가정 A, 음수 = 평면에서 겹침)", "M (나사 길이 A)")
    marg = []
    for part_, gaps_ in sorted(cd["rev3_parts_min_gap_mm"].items(), key=lambda kv: kv[1][KY]):
        marg.append(f"{part_} Y1 {gaps_[KY]:g} / XTAL 핀 {gaps_[KU]:g} mm")
    F.put("크리스털 거리 부품별", "; ".join(marg) + f" (rev 2c 기준 Y1 {r2c[KY]:g} / XTAL 핀 {r2c[KU]:g} mm)",
          "out/reports/routing_rules.json crystal_distance.rev3_parts_min_gap_mm", "M")
    F.put("리플로 Economic", rf["oven_C"]["Economic"].replace(" +-", "±") + " C", "out/fit_checks.json reflow_rating.oven_C", "F (계획 10장)")
    F.put("리플로 Standard", rf["oven_C"]["Standard"].replace(" +-", "±") + " C", "out/fit_checks.json reflow_rating.oven_C", "F (계획 10장)")
    at_limit = [r_ for r_ in rf["rows"] if str(r_.get("Economic", "")).startswith("AT THE LIMIT")]
    if [r_["jlc_code"] for r_ in at_limit] != [x.split(" ")[0] for x in rf["exceeds_or_at_limit"]]:
        raise SystemExit("docs_ko: reflow rows at the limit differ from reflow_rating.exceeds_or_at_limit")
    F.put("리플로 한계 부품", ", ".join(f"{r_['jlc_code']} ({'/'.join(r_['designators'])}, 정격 {r_['rated_peak_C']} C)" for r_ in at_limit),
          "out/fit_checks.json reflow_rating.rows (Economic: AT THE LIMIT)", "F (데이터시트)")
    c47 = next((r_ for r_ in rf["rows"] if r_["designators"] == ["C47"]), None)
    if c47 is None or "reflow ONE time (twice only with 30 min between)" not in c47.get("limits", ""):
        raise SystemExit("docs_ko: the C47 reflow note (one reflow) is not in fit_checks.json reflow_rating")
    F.put("C47 리플로 횟수", "리플로는 한 번만 (두 번은 사이에 30분을 둘 때만)", "out/fit_checks.json reflow_rating.rows C47 limits", "F (데이터시트)")
    cpt = sorted({b__.get("component_product_type") for b__ in plan["bom_lines"]})
    F.put("주문 방식 호환(계획 기록)", f"BOM {len(plan['bom_lines'])}줄 모두 componentProductType {', '.join(str(c_) for c_ in cpt)}",
          "spec/rev3_plan.json bom_lines[].component_product_type (2026-11-10의 JLC 부품 기록)", "F")
    # ---- what the order page showed (hand-written evidence file of the main session)
    OP = "spec/jlc_order_page_20261112.json"
    op = load(OP)
    OPL = "M (주문 화면 2026-11-12)"
    F.put("주문 화면 확인 시각", op["read_at"], OP + " read_at", OPL)
    up_ = op["files_uploaded_then"]
    F.put("주문 화면 거버 인식", f"'{up_['gerber_message']}'", OP + " files_uploaded_then", OPL)
    F.put("주문 화면 매칭", f"부품 {up_['parts_detected']}줄을 인식하고 {up_['parts_confirmed']}줄을 확정", OP + " files_uploaded_then", OPL)
    F.put("주문 화면 올린 줄 수", up_["parts_detected"], OP + " files_uploaded_then.parts_detected", OPL)
    F.put("주문 화면에 올렸던 파일", f"거버 zip {up_['gerber_zip_sha256_prefix']}..., BOM {up_['bom_sha256_prefix']}..., CPL {up_['cpl_sha256_prefix']}...",
          OP + " files_uploaded_then", OPL)
    now3 = (man["fab"]["sets"]["main"]["zip_sha256"], sha["jlcpcb_bom.csv"], sha["jlcpcb_cpl.csv"])
    then3 = (up_["gerber_zip_sha256_prefix"], up_["bom_sha256_prefix"], up_["cpl_sha256_prefix"])
    if any(n_.startswith(t_) for n_, t_ in zip(now3, then3)):
        raise SystemExit("docs_ko: an order file equals the one uploaded on 2026-11-12 - the text says the current files were never uploaded")
    ctx_op_rows = []
    for sec in ("pcb_fields", "pcba_fields", "pcba_advanced_fields"):
        for row in op[sec]:
            val = {"read from the gerber": "거버에서 자동으로 읽힘", "not available": "고를 수 없음 (Economic에는 없는 항목)"}.get(
                row["this_order"], "화면에 보인 대로 둠" if row["this_order"].startswith("as shown")
                else row["this_order"].replace("+-", "±"))
            F.put(f"화면 {row['field']}", val + (f" + '{row['also'].split(chr(39))[1]}' 체크" if row.get("also") else ""),
                  OP + f" {sec}", OPL + ": 항목 이름·선택지; 넣을 값은 이 주문의 선택")
            ctx_op_rows.append((sec, row))
    by_field = {row["field"]: row for _sec, row in ctx_op_rows}
    if by_field["Layers"]["this_order"] != str(spec["board"]["layers"]) or \
            by_field["PCB Thickness"]["this_order"] != f"{spec['board']['thickness_mm']:g}mm":
        raise SystemExit("docs_ko: layers / thickness of the order-page table differ from the spec")
    if by_field["PCBA Type"]["this_order"] != "Economic" or by_field["Surface Finish"]["this_order"] != "LeadFree HASL" \
            or by_field["Bake Components"]["this_order"] != "not available" or by_field["Via Covering"]["this_order"] != "Plugged":
        raise SystemExit("docs_ko: the order-page file is not the Economic / LeadFree HASL / no-bake / Plugged order the text describes")
    F.put("화면 Via 말풍선", f"'{by_field['Via Covering']['tooltip']}'", OP + " pcb_fields[Via Covering].tooltip", OPL)
    tool_row = by_field["Tooling holes"]
    F.put("화면 Standard 레일 칸", tool_row["standard_row"], OP + " pcba_fields[Tooling holes].standard_row", OPL)
    F.put("화면 Standard 레일 문구", f"'{tool_row['standard_page_text']}'", OP + " pcba_fields[Tooling holes].standard_page_text", OPL)
    F.put("위치 확인 창 안내", f"'{by_field['Confirm Parts Placement']['dialog_text']}'", OP + " pcba_fields[Confirm Parts Placement].dialog_text", OPL)
    F.put("베이킹 요금", f"{by_field['Bake Components']['fee_usd']:.2f} USD", OP + " pcba_advanced_fields[Bake Components].fee_usd", OPL)
    F.put("메모 글자 한도", by_field["Assembly remark"]["limit_characters"], OP + " pcba_advanced_fields[Assembly remark].limit_characters", OPL)
    F.put("Economic 받은 줄 수", op["economic_page_behaviour"]["lines_accepted_count"], OP + " economic_page_behaviour.lines_accepted_count", OPL)
    F.put("미리보기 화면 문구", f"'{op['placement_preview']['page_text']}'", OP + " placement_preview.page_text", OPL)
    F.put("미리보기 구분 못 한 부품", op["placement_preview"]["not_resolved"], OP + " placement_preview.not_resolved", OPL)
    F.put("품목 설명", f"'{op['product_description']['used_by_the_main_session']}'", OP + " product_description", OPL)
    # quotes read on the page: every sum is re-added here; a quote whose parts do not add up is an error
    QK = (("setup", "셋업 (Setup fee)"), ("stencil", "스텐실 (Stencil)"), ("components", "부품값 (Components)"),
          ("extended_components_fee", "Extended 부품 수수료 (Extended components fee)"),
          ("feeders_loading_fee", "피더 수수료 (Feeders loading fee)"), ("smt_assembly", "SMT 장착 (SMT Assembly)"),
          ("confirm_parts_placement", "부품 위치 확인 (Confirm Parts Placement)"),
          ("hand_soldering_labor_fee", "손납땜 인건비 (Hand-soldering labor fee)"), ("manual_assembly", "수작업 조립 (Manual Assembly)"),
          ("nitrogen_reflow_soldering", "질소 리플로 (Nitrogen reflow soldering)"), ("packaging", "포장 (Packaging)"))
    QID = ("economic_no_tht", "economic_tht_placed", "standard_no_tht", "standard_enig_as_uploaded")
    quotes = {q_["id"]: q_ for q_ in op["quotes_usd"]}
    if tuple(quotes) != QID or [q_["id"] for q_ in op["quotes_usd"] if q_.get("the_order")] != ["economic_no_tht"]:
        raise SystemExit(f"docs_ko: quotes in {OP} are {list(quotes)}, expected {QID} with economic_no_tht as the order")
    for qid, q_ in quotes.items():
        QS = OP + f" quotes_usd[{qid}]"
        extra = set(q_["pcba_parts"]) - {k_ for k_, _ in QK}
        if extra:
            raise SystemExit(f"docs_ko: quote {qid} has fee lines without a Korean name: {sorted(extra)}")
        if abs(sum(q_["pcba_parts"].values()) - q_["pcba"]) > 0.015 or abs(q_["pcb"] + q_["pcba"] - q_["total_without_shipping"]) > 0.011 \
                or (q_["pcb_parts"] and abs(sum(q_["pcb_parts"].values()) - q_["pcb"]) > 0.011):
            raise SystemExit(f"docs_ko: the numbers of quote {qid} do not add up")
        F.put(f"견적 {qid} 합계", f"{q_['total_without_shipping']:.2f}", QS + " total_without_shipping", OPL)
        F.put(f"견적 {qid} PCB", f"{q_['pcb']:.2f}" + (f" (special offer {q_['pcb_parts']['special_offer']:.2f} + 표면처리 "
              f"{q_['pcb_parts']['surface_finish']:.2f})" if q_["pcb_parts"] else ""), QS + " pcb", OPL)
        F.put(f"견적 {qid} PCBA", f"{q_['pcba']:.2f}", QS + " pcba", OPL)
        F.put(f"견적 {qid} 조건", f"{q_['pcba_type']}, {q_['surface_finish']}, 스루홀 {q_['tht_lines_placed']}줄 실장, 부품 {q_['component_items']}줄",
              QS + " (pcba_type, surface_finish, tht_lines_placed, component_items)", OPL)
        F.put(f"견적 {qid} 조립 기간", q_["assembly_time_days"] + "일", QS + " assembly_time_days", OPL)
        for k_, _ko in QK:
            F.put(f"견적 {qid} {k_}", f"{q_['pcba_parts'][k_]:.2f}" if k_ in q_["pcba_parts"] else "-", QS + f" pcba_parts.{k_}", OPL)
    qo = quotes["economic_no_tht"]
    if qo["component_items"] != len(bom) or quotes["economic_tht_placed"]["component_items"] != len(bom) + len(tht_lines):
        raise SystemExit("docs_ko: the quote 'economic_no_tht' counts another number of component lines than out/jlcpcb_bom.csv")
    F.put("견적 표면처리 추가", f"{qo['pcb_parts']['surface_finish']:.2f} USD", OP + " quotes_usd[economic_no_tht].pcb_parts.surface_finish", OPL)
    F.put("스루홀 JLC 납땜 값", f"{quotes['economic_tht_placed']['total_without_shipping'] - qo['total_without_shipping']:.2f} USD",
          OP + " quotes_usd (economic_tht_placed 합계 - economic_no_tht 합계)", OPL + " 값의 차")
    F.put("Standard 추가 값", f"{quotes['standard_no_tht']['total_without_shipping'] - qo['total_without_shipping']:.2f} USD",
          OP + " quotes_usd (standard_no_tht 합계 - economic_no_tht 합계)", OPL + " 값의 차")
    F.put("견적 as_uploaded 항목 합", f"{sum(quotes['standard_enig_as_uploaded']['pcba_parts'].values()):.2f}",
          OP + " quotes_usd[standard_enig_as_uploaded].pcba_parts 를 더한 값", OPL + " 값의 합")
    fee_rows = {(r_["pair"], r_["order"], r_["boards"]): r_ for r_ in fees}
    r_ = fee_rows[("main", "Economic", "2")]
    if r_["tht_joints_per_board"] != "0":
        raise SystemExit("docs_ko: the fee estimate still counts THT joints for the main pair")
    if float(r_["tht_hand_soldering"]) != 0.0:
        raise SystemExit("docs_ko: the fee estimate charges hand soldering although the main pair has no THT line")
    F.put("추정 Economic 합", f"{float(r_['sum_E']):.2f} USD", "out/jlc_fee_estimate.csv (main, Economic, 2장) sum_E", "E")
    F.put("추정 Economic 피더 종류 수", r_["feeder_types"], "out/jlc_fee_estimate.csv (main, Economic, 2장) feeder_types", "M (BOM에서 셈)")
    F.put("추정 Economic 피더", f"{float(r_['feeder_loading']):.2f} USD", "out/jlc_fee_estimate.csv (main, Economic, 2장) feeder_loading", "E")
    remarks = order_remarks(spec, tht_refs, int(F.data["메모 글자 한도"][0]), F.data["레일 변 구리 이격 값"][0])
    for name in ("main", "with_mcu"):
        F.put(f"메모 글자 수 {name}", remarks[name]["length"], "scripts/docs_ko.py order_remarks (생성한 메모 문장의 글자 수)", "M")
        F.put(f"메모 글자 수 {name} + Standard 문장", remarks[name]["length_with_standard_sentence"],
              "scripts/docs_ko.py order_remarks (Standard 대안 문장을 덧붙였을 때)", "M")
    ctx = {"spec": spec, "plan": plan, "man": man, "gate": gate, "fees": fees, "offb": offb, "code": code,
           "parts": parts, "alt": alt, "s17": plan17_facts(F), "edge": edge, "op": op, "op_rows": ctx_op_rows,
           "remarks": remarks, "quotes": quotes, "QK": QK, "QID": QID, "tht_table": tht_table, "prow": prow,
           "bought_codes": {b_["jlc_code"] for b_ in bought}, "tht_refs": tht_refs}
    return F, ctx


#: plan section 17 (open risks): every bullet must be covered by a user-document line (rev 3 review MR-2).
#: keyword of the bullet -> ("new", Korean sentence for README section 5 using the bullet's own numbers) or
#: ("covered", where the existing text says it). Numbers in a 'new' sentence are checked against the bullet text.
S17: list[tuple[str, str, str, list[str]]] = [
    ("Nothing was built or measured", "covered", "README 5장 '보드 동작 전체'", []),
    ("JLC's acceptance of this board", "covered", "README 5장 'JLC의 수락', ORDER 2·3장 (2026-11-12 주문 화면 기록 포함)", []),
    ("'JLC solders the 6 THT lines'", "covered", "해소됨: 2026-11-12부터 JLC는 스루홀을 납땜하지 않음 (사용자 납땜; README 4장, ORDER 7장 3번·7번)", []),
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
     "+3V3 여유: +5V 최저 4.844 V에서 AMS1117 강하 최대 1.3 V를 빼면 3.3 V보다 0.244 V 위입니다 (E). Wi-Fi 송신 중 최저값은 확인 안 됨 (ORDER 7장 10번).",
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
    ("THT lead protrusion", "covered", "README 5장 '뒷면 다리 길이', ORDER 7장 3번 (모든 스루홀을 사용자가 납땜하고 자름)", []),
    ("Stock moves", "covered", "README 5장 '재고', ORDER 5장", []),
    ("Reflow ratings of C47", "covered", "README 5장 '리플로 온도'", []),
    ("ESP-12F RF behaviour", "new",
     "ESP-12F의 무선 성능은 같은 금지 영역에서 ESP-12E와 같다고 가정했습니다 (A). 펌웨어 이미지도 같은 ESP8266 / 4 MB 구성으로 가정합니다 (A).",
     ["4 MB"]),
    ("BMP280 module header type", "covered", "README 5장 '모듈 치수와 끼움'", []),
    ("OpenLog fit over J5", "covered", "README 5장 '모듈 치수와 끼움'과 모듈 간섭 줄, ORDER 7장 8번", []),
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
    A(f"**{F.v('변경 날짜')} 변경 (사용자 요청: \"{F.v('변경 요청 원문 1')}\", \"{F.v('변경 요청 원문 2')}\"):** "
      f"(1) Wi-Fi 모듈 ESP-12F (U7)는 JLC가 아니라 사용자가 직접 납땜합니다 "
      f"(JLC BOM / CPL에서 빠짐, 두 거버 묶음 모두 U7 패드 {F.v('ESP 패드 수')}개에 솔더 페이스트 구멍 없음). "
      f"(2) 스루홀 커넥터 {F.v('스루홀 부품 수')}개({F.v('스루홀 목록')}, 납땜 {F.v('스루홀 납땜점')}곳)도 JLC가 달지 않고 사용자가 사서 납땜합니다. "
      f"(3) 주문은 Economic PCBA이고 JLC는 SMT 부품 {F.v('main parts')}개(BOM {F.v('main bom_lines')}줄)만 실장합니다. "
      f"(4) 기판 표면처리는 무연 HASL입니다 (보드 파일 표기 '{F.v('보드 표면처리')}'). "
      "이 변경 때는 풋프린트·네트·위치와 구리가 그대로였습니다. "
      f"실크에서는 같은 날 검증 지적으로 아랫면 이름표의 핀 글자 순서만 고쳤습니다: 핀 줄과 나란히 적힌 이름표는 이제 그 면에서 본 패드 순서대로입니다 "
      f"(아랫면: {F.v('아랫면 나란한 라벨')}; 검사 {F.v('라벨 검사')}, M). "
      f"주문 방법은 {p('out/ORDER_ko.md')}, 따로 살 부품은 {p('out/' + KO_NAME)}.")
    A("")
    A(f"**{F.v('변경 날짜')} 재설계 (사용자 요청: {F.v('재설계 요청 원문')}):** 재설계 전 보드(sha256 {F.v('재설계 전 보드 sha256')})는 "
      f"배선 규칙 {F.v('재설계 전 배선 규칙')} 이었습니다 (M). 부품을 바꾸지 않고 자리를 주려고 (1) 기판 -y 변에 승압 회로용 탭을 붙였고 "
      f"(외접 크기 {F.v('외접 크기')}; 본체 76 x 70 mm 사각형과 그 변은 그대로), (2) 케이블타이 띠 안의 프로그래밍 구멍 "
      f"{F.v('옮긴 프로그래밍 구멍')} 을 옮겼고 (사용자 승인), (3) C42를 {F.v('C42 이동')} 옮겼고, (4) 승압부(U8, L5, C41, C43, C57, R63, C58, R64)를 탭에 "
      "다시 놓고, (5) 배터리 전원 경로를 2.0 mm 폭으로 다시 그린 뒤 전체를 새로 배선했습니다. 회로도와 부품(BOM)은 바뀌지 않았습니다. "
      "자세한 내용은 1장과 3장, 검사를 글자로만 통과한 부분은 1장의 '목적과 다른 점'에 있습니다.")
    A("")
    A("표시: M = 이 PC에서 도구로 측정, E = 추정(계산), A = 가정(실물로 확인 필요), F = 문서·데이터시트 근거. 근거가 없으면 NOT_VERIFIED. "
      "이 문서는 `scripts/docs_ko.py` 가 보고서 파일에서 숫자를 읽어 만든 것입니다 (손으로 고치지 말고 다시 생성). 측정·계산된 숫자의 출처는 맨 끝 8장에 있고, "
      "그 표에 없는 숫자(칩 종류, 규칙 값, 시험 조건)는 계획서 `spec/REV3_PLAN.md` 와 사양 `spec/carrier_spec.json` 에서 옮긴 고정값(F)입니다.")
    A("")
    A("## 1. 지금 상태")
    A("")
    A(f"**출고 판정(도구 검사 기준): {F.v('출고 판정')}** - 판정을 막는 항목: {F.v('출고 판정 이유')}.")
    A("")
    A(f"전체 다시 만들기 두 번이 같은 파일을 내는지 (determinism.py, 출고 판정과 따로 봄): {F.v('결정성')}. "
      f"다른 폴더에서: {F.v('다른 폴더 결정성')}. STEP: {F.v('STEP 번호 정렬')}.")
    A("")
    A("이 판정은 도구 검사만 본 것입니다. 보드는 만들어지거나 전원이 들어간 적이 없고, JLC 업로드 미리보기·DFM·견적, "
      "위치 확인과 결제는 사용자가 합니다 (주문 안내서 0장과 3장).")
    A("")
    A(f"**배선 규칙 {F.v('배선 규칙 PASS 수')} / {F.v('배선 규칙 수')} PASS** (FAIL: {F.v('배선 규칙 FAIL')}; 재설계 전 보드: {F.v('재설계 전 배선 규칙')}). "
      "규칙과 기준값은 바꾸지 않았습니다. 바꾼 검사 코드는 새 외곽에 맞춘 부분, 추가한 검사, 사용자가 승인한 구멍 이동, 그리고 결정성 검사의 폴더 이름 바꾸기뿐입니다 "
      "(목록은 3장 끝). 지금 보드에서 잰 값:")
    A("")
    A(f"- 배터리 전원 경로 (J14 → Q10 → J15 → L5): 구리 저항 {F.v('배터리 경로 저항 20C')} (20 C, E; 목표 {F.v('배터리 경로 목표')}), "
      f"전류가 흐르는 구간의 가장 좁은 폭 {F.v('배터리 경로 최소 폭')} (M), 2.0 mm보다 좁은 길이 {F.v('2 mm보다 좁은 길이')}. "
      f"구간별: {F.v('배터리 경로 구간')}. **60 C에서는 {F.v('배터리 경로 저항 60C')} (E)로 0.035 Ohm 목표를 넘습니다** "
      f"(검사는 계획대로 20 C 값으로 판정). 재설계 전: {F.v('재설계 전 배터리 경로')}.")
    A(f"- 출력 커패시터 (검사 hot_loop {F.v('검사 hot_loop')}): {F.v('출력 커패시터 간격')} (기준 각 2.0 mm 이하; 재설계 전 {F.v('재설계 전 출력 커패시터 간격')}). "
      f"윗면에서 U8 GND 핀과 이어진 출력 커패시터 GND: {F.v('출력 커패시터 GND 윗면 연결')}.")
    A(f"- 입력 커패시터 C41 (검사 input_cap_l5 {F.v('검사 input_cap_l5')}, input_cap_vin {F.v('검사 input_cap_vin')}): {F.v('입력 커패시터 L5 경로')}; "
      f"{F.v('입력 커패시터 VIN 경로')}. 재설계 전 C41 → L5 경로는 {F.v('재설계 전 C41-L5 경로')} 였습니다.")
    A(f"- 윗면 GND: {F.v('윗면 GND 한 덩어리')}.")
    A(f"- SW 노드 (검사 sw_node {F.v('검사 sw_node')}): {F.v('SW 노드')}. SW 선: {F.v('SW 선')}. 재설계 전 패드 간격 {F.v('재설계 전 SW 패드 간격')}.")
    A(f"- 크리스털 거리 (검사 crystal_distance {F.v('검사 crystal_distance')}): {F.v('크리스털 거리')}. 부품별 (가까운 순): {F.v('크리스털 거리 부품별')}. "
      f"재설계 전 rev 2c보다 가까웠던 부품: {F.v('재설계 전 크리스털보다 가까운 부품')}.")
    A(f"- 되먹임(FB) 선: {F.v('FB 선과 출력 커패시터')}.")
    A(f"- 레일 쪽 변의 구리: {F.v('레일 변 구리 이격')}.")
    A(f"- 기체 맞춤 (탭 포함, 검사 board_fit {F.v('기체 맞춤 판정')}): {F.v('기체 맞춤')}.")
    A("")
    A("**목적과 다른 점 (검사는 통과했지만 글자로만 통과했거나 여유가 작은 곳 - 숨기지 않음):**")
    A("")
    A(f"- 입력 커패시터 C41의 GND: {F.v('C41 GND 윗면 길')}. 검사 hot_loop_top_gnd와 input_cap_vin은 이 가는 선 덕분에 통과합니다. "
      f"짧은 고리가 아닙니다: C41의 고주파 귀환은 실제로는 자기 비아({F.v('C41 GND 비아 거리')} 떨어짐)를 거쳐 내층 GND 면으로 갑니다. "
      "탭 안에서는 U8 VIN 선, L5 입력 패드와 배터리 선이 C41 GND 패드와 U8 GND 핀 사이를 막고 있습니다. "
      f"내층에서의 귀환 거리: {F.v('내층 귀환 거리')}. TI 데이터시트 10.1절은 입력 커패시터를 VIN 핀과 GND 핀 가까이 두라고 하지만, "
      "C41은 L5 입력 패드 옆에 있고 U8 VIN 핀(3번) 옆에는 따로 커패시터가 없습니다 (작은 커패시터를 더하려면 회로도 변경 = 사용자 결정).")
    A("- C41의 VIN 쪽 경로(위 숫자)는 L5 입력 패드를 지나 U8 VIN 핀으로 갑니다.")
    A(f"- 출력 커패시터 C43의 GND: {F.v('C43 GND 선')}. 고리는 작지만 TI 예시(10.2절)·발열 안내(10.3절)처럼 넓은 GND 구리와 여러 비아가 아니라 "
      "가는 선입니다. U8의 발열은 측정 전까지 NOT_VERIFIED입니다.")
    A(f"- 배터리 경로는 L5 입력 패드 바로 앞에서 비아 두 개를 지납니다: {F.v('L5 입력 비아')}. {F.v('L5 입력 비아 단면')} "
      "폭 검사(2.0 mm)는 선만 재므로 이 비아들은 글자로만 통과합니다.")
    A("- 출력 커패시터 C57의 GND 패드는 SW 선(U8.5 → L5.2) 건너편에 있어, 윗면에서는 L5 둘레·아래의 GND 면(구리 채움)을 통해서만 U8 GND 핀과 이어집니다. "
      "검사는 그 면이 한 덩어리라서 통과합니다. C57의 실제 귀환은 자기 비아와 U8 GND 핀의 비아를 거친 내층 GND입니다. 배선 프로그램이 이 면을 끊지 못하게 "
      "탭 위 윗면에 배선 금지 영역을 두었습니다 (Freerouting 입력 파일에만 있고 보드 파일에는 없음).")
    A(f"- SW 패드 간격 여유는 {F.v('SW 간격 여유')} 뿐입니다 (부품 자리가 맞닿아 배치로 정해짐; 배선으로는 줄일 수 없음).")
    A("- 미리 그린 배선(승압부, 배터리 경로의 갈래)은 이제 배선 마무리 단계의 자동 넓히기에서 빠져 설계한 폭 그대로입니다. 재설계 전 흐름은 이 갈래들을 넓혔습니다. "
      "2.0 mm 배터리 경로는 이와 무관합니다.")
    A("- 회로 IR(설계 데이터)은 외곽을 사각형으로만 담을 수 있어 76 x 70 mm 사각형을 그대로 갖고, 탭이 있는 실제 외곽은 `scripts/outline.py` 에서 옵니다. "
      "외곽에 대해서는 IR이 유일한 설계 데이터가 아닙니다.")
    A("- 이 영향(스위칭 과전압, 입력 리플, 잡음, 크리스털 흔들림)은 보드에서 오실로스코프로 재기 전까지 NOT_VERIFIED입니다 (주문 안내서 7장).")
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
      f"(0.3 mm 안: {F.v('가장자리 0.3 mm 안 몸체')} - 둘 다 사용자가 납땜하는 자리; JLC 실장 부품 중 가장 가까운 것 {F.v('JLC 실장 가장 가까운 몸체')}) | M |")
    A(f"| 모듈(OpenLog / BMP280 / GY-521) 간섭, 보통 경우 | {F.v('모듈 간섭')}; 최소 간격 {F.v('모듈 최소 간격')} 미만: "
      f"{F.v('모듈 간섭 보통 0.5 mm 미만')} | A / NOT_VERIFIED |")
    A(f"| 모듈 간섭, 보수적 경우 (OpenLog 아랫면 부품 1.2 mm 가정) | 충돌: {F.v('모듈 간섭 보수적 충돌')}; {F.v('모듈 최소 간격')} 미만: "
      f"{F.v('모듈 간섭 보수적 0.5 mm 미만')} | A / NOT_VERIFIED |")
    A("")
    A("## 2. 이 기판은 무엇인가")
    A("")
    A(f"- 크기: 본체 {F.v('외곽')} 사각형 + -y 변의 탭 ({F.v('탭 위치')}, 넓이 {F.v('탭 넓이')}) → 외접 {F.v('외접 크기')}, "
      f"기판 넓이 {F.v('기판 넓이')}. 두께 {F.v('두께')}, {F.v('층 수')}층 (F.Cu 신호와 부품 / In1 GND 전면 / In2 +5V 전면 / B.Cu 신호). "
      "부착 구멍 없음: 3 mm 폼테이프 + 케이블타이 1개. 케이블타이 띠(x -3..+3 mm)에는 실장 부품이 없습니다.")
    A(f"- 부품은 모두 윗면에만 있습니다. 부품 높이 한계는 기판 윗면에서 {F.v('높이 한계')} 입니다. 아랫면은 폼테이프 자리라 비어 있습니다.")
    A("- 비행 컴퓨터 칩 4개: U1 NAV, U2 FCU, U3 COM, U4 GNS (ATmega128A-AU, 16 MHz, 5 V). 무선은 U7 ESP-12F (Wi-Fi 중계; 사용자가 직접 납땜).")
    A("- 센서: GY-521 (IMU, J1에 납땜), BMP280 (기압, 소켓 J2), NEO-7M GPS (기체 윗뚜껑, 케이블로 J7). 기록기 OpenLog는 소켓 J5.")
    A("- 전원 흐름: 1S 로직 배터리 J14 → Q10 역극성 보호 → 스위치 리드 J15 → U8 승압 5 V (+ U9 저전압 차단) → +5V 면 → U6 → +3V3.")
    A(f"- 커넥터: GPS용 J7 (JST-GH, SMD)만 JLC가 실장합니다. 스루홀 커넥터 {F.v('스루홀 부품 수')}개({F.v('스루홀 목록')})는 "
      f"{F.v('변경 날짜')} 결정으로 사용자가 사서 납땜합니다 (받은 보드에는 빈 도금 구멍과 실크 윤곽만 있음).")
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
    A(f"- **Wi-Fi 모듈 ESP-12E → ESP-12F** (참고 부품 LCSC {F.v('ESP 참고 부품')}). 핀과 풋프린트가 같습니다. 계획은 JLC가 실장할 수 있는 부품이라 "
      f"ESP-12F를 골랐는데, {F.v('변경 날짜')} 사용자 결정으로 이 모듈은 사용자가 직접 납땜합니다 (JLC BOM의 U7: {F.v('코드 U7')}).")
    A(f"- **스루홀 커넥터 {F.v('스루홀 부품 수')}개는 사용자가 사서 납땜합니다** ({F.v('변경 날짜')} 사용자 결정; 계획은 JLC가 손납땜). "
      f"참고 부품 번호 (계획 때 JLC BOM에 있던 번호, F): 소켓 J2 / J5 {F.v('참고 코드 J2')}, 서보 헤더 J8~J12 {F.v('참고 코드 J8')}, J16 {F.v('참고 코드 J16')}, "
      f"J18 {F.v('참고 코드 J18')}, 배터리 J14 {F.v('참고 코드 J14')}, 스위치·부저 J15 / J17 {F.v('참고 코드 J15')}. "
      f"JLC BOM에는 없습니다 (J2: {F.v('코드 J2')}). 탄탈 C45 ({F.v('코드 C45')}, JLC 실장)와 헤더·소켓 번호 일부는 재고나 도면 근거로 "
      "바꿨고 D3 / D12에서 rev 2c 번호로 되돌릴 수 있습니다.")
    A(f"- **그대로인 것:** MCU 4개의 핀 256개가 가는 네트 ({F.v('MCU 핀맵')}), 본체 76 x 70 mm 사각형과 그 변, 커넥터·MCU·크리스털·모듈 소켓·ESP "
      f"위치와 방향 (고정 부품 {F.v('고정 부품 비교')}; 프로그래밍 구멍 3개는 아래 재설계에서 사용자 승인으로 옮김), 설계 규칙 "
      f"({F.v('규칙 파일')}), 크리스털 배선 ({F.v('XTAL1 최악')}).")
    A(f"- **{F.v('변경 날짜')} 재설계 - 외곽에 탭 추가** (사용자 요청 {F.v('재설계 요청 원문')}): 지금 부품(U8 TPS61023, L5 FXL0420, 22 uF 1206 커패시터)으로 "
      "승압부 검사와 크리스털 거리 검사를 모두 지키려면 76 x 70 mm 안에 자리가 모자랐습니다 (재설계 전 연구, M). 그래서 본체의 변은 그대로 두고 -y 변에 탭을 붙였습니다: "
      f"{F.v('탭 위치')}, 넓이 {F.v('탭 넓이')}, 외접 {F.v('외접 크기')}. 탭은 기체 CAD에서 잰 허용 범위({F.v('Fit 허용 범위')}) 안에 있습니다. "
      "이것은 계획 불변조건 (b) '외곽 76 x 70 mm 그대로'에서 벗어난 사용자 결정입니다.")
    A(f"- **승압부 자리 (재설계):** {F.v('승압부 자리')}. U8의 VOUT / GND 핀 옆에 출력 커패시터 C43(위)과 C57(오른쪽), 아래에 L5, L5 입력 패드 옆에 C41. "
      "저전압 차단(U9 주변), LDO U6, 그 밖의 부품 자리는 그대로입니다.")
    A(f"- **프로그래밍 구멍 3개 이동 (사용자 승인 \"{REDESIGN_REQUESTS[2]}\"):** {F.v('옮긴 프로그래밍 구멍')}. 케이블타이 띠를 건너는 배터리 선이 2.0 mm 폭으로 "
      "지나갈 틈을 만들기 위해서입니다. 배치 검사(place.py)는 이 세 부품만 'rev 2c 자리 + 이 거리'와 정확히 같은지 봅니다 (다른 고정 부품은 rev 2c 자리 그대로인지 봄). "
      "외곽 맞춤과 추가한 검사 말고, 설계 판정에 쓰이는 검사 코드에서 사용자 결정 때문에 바꾼 곳은 이것 하나입니다.")
    A(f"- **C42 이동:** {F.v('C42 이동')} - 2.0 mm 배터리 선이 Q10 3번 패드로 들어가게.")
    A(f"- **배터리 전원 경로:** 선은 모든 구간 {F.v('배터리 경로 최소 폭')} 이상 (1장의 숫자). 다만 L5 입력 패드 앞의 비아 두 개({F.v('L5 입력 비아')})는 "
      "선이 아니라서 폭 검사 대상이 아니고, 도금 단면이 2.0 mm 선보다 작습니다 (1장 '목적과 다른 점').")
    A(f"- **바꾼 검사 코드 (규칙·기준값은 그대로):** 외곽 맞춤 - 판의 모서리·변으로 재던 검사(routing_checks 레일 변 구리와 식별 문구의 가장자리 거리, "
      "place.py 기판 안쪽, fit_checks와 edge_distance의 -y 변 거리, silk.py 글자 자리, prog_access.py 헤더 자리 검색의 기판 밖 칸)가 이제 탭의 모든 변을 "
      "실제 선분으로 잽니다 (y 135 선 옆에서는 예전과 같은 값). 추가한 검사 - 기체 맞춤 board_fit (board_metrics.py)과 315도 조인트 나사 아래 부품 금지 "
      "영역 (place.py). 사용자 결정 - J26 / J22 / J19의 기준 자리 (위). 결정성 검사 determinism.py - 다른 폴더 비교의 폴더 이름 바꾸기 규칙만 "
      "작업 복사본에서도 돌도록 고침 (비교 종류와 PASS 규칙은 그대로).")
    A(f"- **내보내기 (검사 아님):** STEP 파일 - {F.v('STEP 번호 정렬')}. kicad-cli가 내보낼 때마다 같은 항목을 다른 순서로 써서 결정성 검사가 "
      "실패하던 원인을 생성기에서 고친 것이며, 결정성 검사 자체는 바꾸지 않았습니다.")
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
    A(f"- JLC가 붙이는 것: SMT 부품만. 기본 파일 쌍 기준 BOM {F.v('main bom_lines')}줄 (SMT {F.v('main bom_lines_smt')}줄, 스루홀 {F.v('main bom_lines_tht')}줄), "
      f"부품 {F.v('main parts')}개. MCU까지 맡기는 with_mcu 쌍은 {F.v('with_mcu bom_lines')}줄 / {F.v('with_mcu parts')}개 "
      f"(U1~U4 = {F.v('U1-U4 with_mcu')}).")
    A(f"- 사용자가 붙이는 것: ATmega128A-AU 4개 (기본, with_mcu 쌍이면 JLC), ESP-12F 1개 (U7, 패드 {F.v('ESP 패드 수')}개), "
      f"스루홀 커넥터 {F.v('스루홀 부품 수')}개 ({F.v('스루홀 목록')}, 납땜 {F.v('스루홀 납땜점')}곳) - ESP-12F와 커넥터는 두 파일 쌍 모두에서 사용자 몫. "
      "그리고 GY-521 (J1 구멍에 모듈 핀을 납땜), BMP280 (직접 단 소켓 J2에 꽂음), "
      "OpenLog (모듈에 직각 6핀 헤더를 납땜한 뒤 직접 단 소켓 J5에 꽂음), NEO-7M (케이블로 J7).")
    A(f"- 부품 없이 비워 둔 자리: 프로그래밍 단자 J3, J19~J22 (2x3), J24~J26 (1x2) - 현재 선택 {F.v('프로그래밍 헤더 선택')}안 (빈 도금 구멍). "
      f"JLC가 헤더를 띠 밖에 실장하는 B안은 배치 검색 결과 {F.v('B안 결론')} ({F.v('B안 2x3 자리')}).")
    A(f"- MCU·센서 말고 사용자가 해야 하는 일(ESP-12F와 스루홀 커넥터 납땜, 다리 자르기, 프로그래밍 단자 등)은 {p('out/ORDER_ko.md')} 8장에, "
      f"납땜 순서·커넥터 방향·받은 뒤 전원 시험은 7장에, 따로 살 부품은 {p('out/' + KO_NAME)} 에 모았습니다.")
    A("")
    A("## 5. 확인되지 않은 것 (NOT_VERIFIED)")
    A("")
    for t in [
        "재설계한 승압부의 전기적 효과: 검사는 모두 통과했지만 1장의 '목적과 다른 점'(C41 GND가 가는 선으로만 윗면에서 이어짐, C57 GND가 GND 면으로만 "
        f"이어짐, SW 간격 여유 {F.v('SW 간격 여유')})이 실제로 어떤 영향을 주는지 - 스위칭 과전압, 입력 리플, SW 선 발열, 크리스털 잡음 - 는 "
        "보드에서 오실로스코프로 재기 전까지 NOT_VERIFIED입니다.",
        f"배터리 경로 60 C: 구리 저항이 {F.v('배터리 경로 저항 60C')} (E)로 0.035 Ohm 목표를 넘습니다. 검사는 계획대로 20 C 값({F.v('배터리 경로 저항 20C')})으로 "
        "판정합니다. 뜨거운 곳에서 오래 쓰면 저전압 차단 여유가 줄 수 있습니다 (계획 17장, 첫 보드에서 저항 측정: 주문 안내서 7장).",
        "새 외곽의 JLC 수락과 값: 탭이 있는 거버는 아직 JLC에 올려 본 적이 없습니다. 주문 화면이 읽을 크기(외접 사각형으로 읽을 것으로 봄, E), "
        "PCB 값, DFM 결과는 NOT_VERIFIED입니다. 2026-11-12에 저장한 견적은 모두 재설계 전 76 x 70 mm 거버의 것입니다 (주문 안내서 4장).",
        "Standard 대안의 레일: y 135 쪽 긴 변에 탭이 있어 그 변은 곧은 선이 아니므로 V-cut 레일은 쓸 수 없고, 라우팅 탭 / 마우스바이트 레일이 필요합니다. "
        "JLC가 그렇게 해 주는지와 값은 NOT_VERIFIED입니다 (Economic 주문은 레일이 없어 해당 없음).",
        "기체 CAD는 아직 탭이 없는 76 x 70 x 15 mm 상자로 캐리어를 그립니다 (이 단계는 CAD 저장소를 읽기만 했음). 남은 일: params.json의 캐리어 항목에 탭 추가, "
        "cad/layout.py의 캐리어 검사(보어 거리, 통풍구, 케이블타이 길, 폼테이프 면적, 조인트 나사 여유)를 사각형 + 탭으로 바꾸기, C25 등 CAD 검사와 내보내기 다시 실행, "
        f"캐리어 무게에 탭 몫 약 {F.v('탭 질량 추정')} 더하기, 76 x 70 을 적은 기체 문서·그림 고치기. "
        f"이 PCB 쪽 검사(board_fit)는 Fit 단계가 CAD에서 잰 값으로 탭을 확인했습니다 ({F.v('기체 맞춤 판정')}).",
        "흡입구/아비오닉스 연결부 315도 조인트 나사의 길이: params.json에는 'M3'만 있습니다. 부품 금지 영역은 나사 끝이 반지름 49.5 mm까지 들어온다고 보고 잡았습니다 (A). "
        "M3 x 6 이하를 권합니다. 나사 길이는 NOT_VERIFIED입니다.",
        f"모듈 끼움, 보수적 경우: OpenLog 아랫면 부품을 1.2 mm로 잡으면 꽂는 길에서 {F.v('모듈 간섭 보수적 충돌')} 이 겹치고, "
        f"{F.v('모듈 간섭 보수적 0.5 mm 미만')} 은 {F.v('모듈 최소 간격')} 보다 가깝습니다 (A). OpenLog 아랫면 높이를 재고, 막을 수 있는 부품을 "
        "납땜하기 전에 실제로 꽂아 보십시오.",
        "보드 동작 전체: 만들거나 전원을 넣은 적이 없습니다. 전원부 수치(5 V 출력, 차단 전압, 전류 여유)는 데이터시트 계산(E)입니다.",
        f"JLC의 수락: Economic 주문에서 JLC가 기판 안에 툴링 홀 자리를 찾는지 (부품 자리 점유율 {F.v('부품 자리 점유율')}; 못 찾으면 레일 = Standard를 요구할 수 있음), "
        f"가장자리 가까운 부품 (JLC 실장 중 가장 가까운 것 {F.v('JLC 실장 가장 가까운 몸체')}; 0.3 mm 안: {F.v('JLC 실장 몸체 0.3 mm 안')}), "
        "지금 파일의 BOM / CPL 매칭은 업로드 후 화면과 DFM 결과, 견적으로만 확인됩니다.",
        f"주문 화면에서 본 것 ({F.v('주문 화면 확인 시각')}, M): ESP·스루홀 변경과 재설계 전 빌드의 파일(76 x 70 mm 거버)을 올려 {F.v('주문 화면 매칭')}했고, 화면에서 ESP-12F와 스루홀 줄을 뺀 "
        f"Economic 견적은 배송 제외 {F.v('견적 economic_no_tht 합계')} USD였습니다 (Economic이 ESP-12F를 뺀 {F.v('Economic 받은 줄 수')}줄을 모두 받음). "
        "지금 파일(처음부터 SMT만 든 BOM)로는 아직 올려 보지 않았습니다.",
        f"사용자가 사는 스루홀 커넥터 {F.v('스루홀 종류 수')}종과 ESP-12F: 낱개로 산 부품이 구멍·패드에 맞는지, 그 재고와 가격 "
        f"(LCSC 기록은 {F.v('LCSC 기록 시각')} 기준 M, 그 밖의 쇼핑몰은 확인 안 함), 무연 HASL 구멍의 납땜성은 해 보기 전에는 NOT_VERIFIED입니다. "
        "배터리 커넥터 J14는 방향이 틀리면 극성이 뒤집힙니다 (주문 안내서 7장 3번).",
        f"CPL 회전: 방향이 중요한 {F.v('방향 중요 부품 수')}개 중 {F.v('회전 미확인')} 은 자료끼리 어긋나 NOT_VERIFIED입니다. "
        "나머지도 JLC 미리보기로만 확정됩니다. 탄탈 C45 / C52, 전해 C47을 거꾸로 붙이면 화재 위험이 있습니다.",
        f"리플로 온도: 이번 주문 방식 Economic은 최고 {F.v('리플로 Economic')} 입니다. 정격이 그 최악값과 같아 여유가 없는 부품: {F.v('리플로 한계 부품')} "
        f"(C47은 {F.v('C47 리플로 횟수')}); 문서에서 정격을 찾지 못한 부품 번호 {F.v('리플로 미확인 줄 수')}개.",
        "ESP-12F 손납땜: 모듈과 무연 HASL 패드의 납땜성, 패드 바로 옆 부품과의 간격에서 작업이 되는지는 해 보기 전에는 NOT_VERIFIED입니다.",
        f"뒷면 다리 길이 (모든 스루홀을 사용자가 납땜하고 자름; 도면값은 참고 부품 번호 기준 F): {F.v('다리 규칙 초과')} 은 규칙 {F.v('뒷면 다리 규칙')} 보다 길어 "
        f"잘라야 하고, {F.v('다리 공차 끝 초과')} 는 공차 끝에서 넘습니다. {F.v('다리 길이 미확인')} 은 확인 못 함.",
        f"모듈 치수와 끼움: GY-521, BMP280, OpenLog, NEO-7M 치수는 가정값입니다 (GY-521 받침 {F.v('GY-521 받침 높이')}, BMP280 헤더 {F.v('BMP280 헤더 종류')} - 모두 A). "
        "그림과 간섭 검사도 이 가정 위에 있습니다. 실물을 버니어로 재야 확정됩니다 (D15).",
        "J14 Molex 커넥터: 라이브러리 3D 모델이 없고 구멍 지름을 도면과 대조하지 못했습니다.",
        f"재고: {F.v('JLC 확인 시각')} 기준입니다. 주문 직전에 scripts/jlc_gate.py 를 다시 돌려야 합니다.",
        f"비용: 주문 화면의 견적 {F.v('견적 economic_no_tht 합계')} USD (M, 배송 제외)는 재설계 전 76 x 70 mm 거버의 이전 업로드에서 줄을 빼고 읽은 값입니다. "
        f"지금 파일(외접 {F.v('외접 크기')})의 견적, 메모 검토 뒤 금액, "
        "배송비와 세금은 NOT_VERIFIED입니다.",
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
      "필요합니다. 새 배선은 실행마다 결과가 달라질 수 있습니다. "
      f"{F.v('변경 날짜')} 재설계는 배치가 바뀌어 새 배선(Freerouting 실시간 실행)을 했고, 그 실행의 세션을 기록해 두었습니다. "
      "route.py는 배선 전 보드가 세션을 기록한 보드와 바이트까지 같을 때(표면처리 글자 한 곳만 다른 경우 포함)에만 재생을 받아들입니다 "
      f"(이번 재생의 판정: '{F.v('배선 재생 입력 보드')}', M).")
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
        ("따로 살 것 (읽기 쉬운 목록 / CSV)", "out/" + KO_NAME),
        ("", "out/off_board_purchase.csv"),
        ("따로 살 부품의 LCSC 재고·가격 기록 (2026-11-12에 읽기만 함)", "spec/lcsc_snapshot_20261112.json"),
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
        ("주문 화면에서 본 항목·견적 기록 (2026-11-12, 손으로 적은 입력 파일)", "spec/jlc_order_page_20261112.json"),
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
    A("이 안내서는 PCB 실장 주문(PCBA)을 처음 하는 사람도 따라 할 수 있게 썼습니다. **이 문서를 만든 스크립트는 아무것도 업로드하거나 주문하지 않습니다. "
      "이 보드는 한 번도 만들어진 적이 없습니다.** 업로드, 확인, 결제는 모두 사용자가 JLCPCB 사이트에서 직접 합니다 "
      "(2026-11-12에 사용자의 브라우저에서 이전 빌드의 파일을 주문 화면에 올려 견적까지 본 기록이 2장과 4장에 있습니다. 그 기록에 주문이나 결제는 없습니다).")
    A("")
    A("표시: M = 도구로 측정, E = 추정, A = 가정, F = 문서 근거, NOT_VERIFIED = 근거 없음. 'M (주문 화면 2026-11-12)' 는 그날 사용자의 "
      "로그인된 브라우저에서 JLCPCB 주문 화면을 직접 보고 적은 값입니다. 측정·계산된 숫자의 출처는 맨 끝 11장에 있습니다. "
      "그 표에 없는 숫자(시험 전류·전압, 규칙 값, 수량 기본값)는 계획서 `spec/REV3_PLAN.md` 의 괄호 안 장(15장, 16장의 D 번호 등)에서 옮긴 값(F, 계산값은 E)입니다. "
      "이 파일은 `scripts/docs_ko.py` 가 만든 것입니다.")
    A("")
    A("## 0. 먼저 읽을 것: 도구 판정과 2026-11-12 재설계")
    A("")
    ok_ = F.data["출고 판정"][0].startswith("ORDERABLE")
    A(f"출고 판정 (도구 검사): **{F.v('출고 판정')}**. 판정을 막는 항목: {F.v('출고 판정 이유')}.")
    A("")
    A(f"결정성 검사 (determinism.py, 출고 판정과 따로 봄): {F.v('결정성')}. 다른 폴더에서: {F.v('다른 폴더 결정성')}.")
    A("")
    if ok_:
        A("이 판정은 도구 검사(회로도 ERC, 보드 DRC, 배선 규칙 21개, 기체 맞춤, 주문 파일 검사, 오늘의 JLC 재고 확인)를 모두 통과했다는 뜻일 뿐입니다. "
          "**보드는 한 번도 만들어지거나 전원이 들어간 적이 없고 (NOT_VERIFIED), 지금 파일은 JLC에 올려 본 적이 없습니다.** "
          "도구가 볼 수 없는 아래 항목은 사용자가 주문 화면에서 확인합니다 (3장):")
    else:
        A("**판정이 '주문 불가'인 동안에는 주문하지 마십시오.** 위 항목을 먼저 해결해야 합니다. 도구가 보지 않는 항목도 그대로 남습니다 (3장):")
    A("")
    for g_ in ctx["man"]["release_gate"]["user_gates_not_covered"]:
        A(f"- {g_}")
    A("")
    A("위 목록은 out/manifest.json 의 release_gate.user_gates_not_covered 를 그대로 옮긴 것입니다 (영어 원문). 특히 C45 / C47 / C52 / U9의 회전·극성은 "
      "JLC의 위치 확인 화면에서 직접 확인하고, 주문 당일 결제 직전에 jlc_gate.py를 다시 실행하며, 결제는 사용자가 합니다.")
    A("")
    A(f"### A. 이번 재설계에서 바뀐 것 (사용자 요청 {F.v('재설계 요청 원문')})")
    A("")
    A(f"- 재설계 전 보드의 배선 규칙: {F.v('재설계 전 배선 규칙')} (M). 지금: {F.v('배선 규칙 PASS 수')} / {F.v('배선 규칙 수')} PASS.")
    A(f"- 외곽: 본체 {F.v('본체 사각형')}는 그대로이고 -y 변에 탭을 붙였습니다: {F.v('탭 위치')}, 넓이 {F.v('탭 넓이')}. 외접 크기 {F.v('외접 크기')}. "
      f"기체 맞춤 (Fit 단계가 기체 CAD에서 잰 값으로 확인, board_fit {F.v('기체 맞춤 판정')}): {F.v('기체 맞춤')}.")
    A(f"- 프로그래밍 구멍 이동 (사용자 승인): {F.v('옮긴 프로그래밍 구멍')}. 이 세 구멍(빈 도금 구멍, 부품 없음)의 위치가 바뀌었으므로 "
      "옛 위치에 맞춰 만든 포고핀 어댑터나 지그가 있다면 맞춰 고쳐야 합니다 (위치: out/reports/programming_access.json bare_holes_option_A). "
      f"C42 이동: {F.v('C42 이동')}.")
    A(f"- 승압부 자리: {F.v('승압부 자리')}.")
    A(f"- 배터리 전원 경로: 저항 {F.v('배터리 경로 저항 20C')} (20 C, E; 목표 {F.v('배터리 경로 목표')}), 가장 좁은 폭 {F.v('배터리 경로 최소 폭')}; "
      f"{F.v('배터리 경로 구간')}. 60 C에서는 {F.v('배터리 경로 저항 60C')} (E)로 목표를 넘습니다 (판정은 20 C 값). 재설계 전: {F.v('재설계 전 배터리 경로')}.")
    A(f"- 출력 커패시터: {F.v('출력 커패시터 간격')} (재설계 전 {F.v('재설계 전 출력 커패시터 간격')}). 입력 커패시터: {F.v('입력 커패시터 L5 경로')}. "
      f"SW: {F.v('SW 노드')}. 크리스털: {F.v('크리스털 거리 부품별')}.")
    A("")
    A("### B. 검사는 통과했지만 목적과 다른 점 (숨기지 않음)")
    A("")
    A(f"- 입력 커패시터 C41의 GND: {F.v('C41 GND 윗면 길')}. 검사 hot_loop_top_gnd / input_cap_vin은 이 가는 선으로 통과합니다. 데이터시트가 말하는 "
      f"'입력 커패시터 GND를 IC GND 바로 옆에' 같은 짧은 고리는 아닙니다. C41의 고주파 귀환은 자기 비아({F.v('C41 GND 비아 거리')})를 거친 내층 GND입니다.")
    A("- C41의 VIN 쪽 경로는 L5 입력 패드를 지나 U8 VIN 핀으로 갑니다.")
    A("- 출력 커패시터 C57의 GND는 윗면에서 L5 둘레·아래의 GND 면(구리 채움)으로만 U8 GND 핀과 이어집니다 (SW 선이 사이에 있어 선을 그을 수 없음). "
      "자동 배선이 이 면을 끊지 않도록 Freerouting 입력에만 탭 위 윗면 배선 금지 영역을 두었습니다.")
    A(f"- SW 패드 간격 여유 {F.v('SW 간격 여유')} (부품 자리가 맞닿은 배치로 정해짐).")
    A("- 미리 그린 배선(승압부와 배터리 경로의 갈래)은 자동 넓히기에서 빠져 설계한 폭 그대로입니다 (재설계 전 흐름은 넓혔음).")
    A("- 배치 검사 place.py는 J26 / J22 / J19만 'rev 2c 자리 + 사용자가 승인한 이동'과 비교합니다. 외곽 맞춤과 추가한 검사 말고 사용자 결정 때문에 "
      "바꾼 판정 검사 코드는 이것 하나입니다. 그 밖에 바꾼 검사 스크립트: determinism.py의 폴더 이름 바꾸기 규칙 (비교 종류·PASS 규칙은 그대로). "
      "전체 목록은 README 3장.")
    A(f"- 입력 커패시터 C41은 L5 입력 패드 옆에 있고 U8 VIN 핀 옆에는 커패시터가 없습니다. 내층 귀환 거리: {F.v('내층 귀환 거리')}.")
    A(f"- 출력 커패시터 C43의 GND: {F.v('C43 GND 선')} (넓은 GND 구리가 아님; U8 발열 NOT_VERIFIED).")
    A(f"- 배터리 경로의 L5 입력 비아: {F.v('L5 입력 비아 단면')} 폭 검사는 선만 잽니다.")
    A("- 설계 데이터(IR)는 외곽을 76 x 70 mm 사각형으로만 갖고, 탭이 있는 외곽은 scripts/outline.py 에서 옵니다.")
    A("- 이것들이 실제로 문제가 되는지는 보드를 받은 뒤 7장의 측정(켜질 때 VIN, 부하에서 SW와 +5V 과전압, 입력 리플, NAV 칩 시계)으로만 압니다 (NOT_VERIFIED).")
    A("")
    A("### C. 이번 재설계로 새로 생긴 확인 거리 (NOT_VERIFIED)")
    A("")
    A("- 새 거버(탭 있음)는 JLC에 올려 본 적이 없습니다. 주문 화면의 크기·PCB 값·DFM 결과는 올려 봐야 압니다. 4장의 견적은 모두 재설계 전 76 x 70 mm 거버의 것입니다.")
    A("- Standard 대안(레일)으로 가면 탭이 있는 변에는 V-cut을 쓸 수 없습니다 (2장).")
    A("- 기체 CAD는 아직 탭이 없는 상자로 캐리어를 그립니다. 315도 조인트 나사 길이는 정해지지 않았습니다 (README 5장).")
    A(f"- 조인트 나사 4개 모두 길이 M3x6 이하를 권합니다: 나사 끝이 r 49.5 mm까지 들어온다고 가정하면(A) 다른 나사 옆 부품은 {F.v('다른 조인트 나사 옆 부품')} "
      "(재설계 전과 같은 자리의 커넥터 J14 / J7이 길이를 정함; 315도 나사 옆에는 부품 없음).")
    A("- 탭 안쪽 모서리 r 1.0 mm를 JLC의 라우터 날이 만들 수 있는지는 근거 문서가 없어 NOT_VERIFIED입니다 (업로드 미리보기에서 확인). "
      "탭 부품 C41 / L5 / C57의 몸체는 가장자리에서 2.5 mm보다 가깝습니다 (README 1장 표) - JLC DFM 결과를 보십시오.")
    A(f"- 알려진 작은 항목 (M, 검사 대상 아님): U9 바이패스 C59의 GND 비아 - {F.v('C59 GND 비아')}; L5 입력 패드 4 mm 안의 배터리 네트 비아 - "
      f"{F.v('L5 입력 비아')}. {F.v('L5 입력 비아 단면')}")
    A("")
    A("- 아래 1~9장은 주문 절차입니다 (Economic PCBA로 SMT 부품만 실장, 무연 HASL; ESP-12F와 스루홀 커넥터는 사용자가 사서 납땜).")
    A("")
    A("## 1. 올릴 파일 3개")
    A("")
    A(f"**{F.v('변경 날짜')} 변경 (사용자 요청 \"{F.v('변경 요청 원문 1')}\", \"{F.v('변경 요청 원문 2')}\"):** "
      "(1) ESP-12F(U7)는 JLC가 아니라 사용자가 직접 납땜합니다. "
      f"(2) 스루홀 커넥터 {F.v('스루홀 부품 수')}개({F.v('스루홀 목록')})도 JLC가 달지 않고 사용자가 사서 납땜합니다. "
      "(3) 주문은 Economic PCBA로 하고, JLC는 SMT 부품만 실장합니다 (JLC의 손납땜 없음). "
      "(4) 기판 표면처리는 무연 HASL입니다. 아래 파일과 절차는 이 결정대로 다시 만든 것입니다. "
      f"따로 살 부품은 6장과 {p('out/' + KO_NAME)} 에 정리했습니다.")
    A("")
    A("기본 (MCU 4개, ESP-12F, 스루홀 커넥터는 사용자가 직접 납땜):")
    A("")
    A("| 순서 | 파일 | 어디에 올리나 | sha256 앞부분 |")
    A("|---|---|---|---|")
    A(f"| 1 | {p('out/vtol_carrier_gerber.zip')} | PCB 주문 첫 화면 'Add gerber file' | {F.v('zip main sha256')} |")
    A(f"| 2 | {p('out/jlcpcb_bom.csv')} | PCB Assembly 단계 'BOM file' | {F.v('jlcpcb_bom.csv sha256')} |")
    A(f"| 3 | {p('out/jlcpcb_cpl.csv')} | PCB Assembly 단계 'CPL file' (Pick and Place) | {F.v('jlcpcb_cpl.csv sha256')} |")
    A("")
    A("다른 선택 (MCU 4개는 JLC가 실장, 결정 D4의 권장안; ESP-12F와 스루홀 커넥터는 이 묶음에서도 사용자가 납땜): 아래 3개를 **한 묶음으로** 올립니다.")
    A("")
    A("| 순서 | 파일 | sha256 앞부분 |")
    A("|---|---|---|")
    A(f"| 1 | {p('out/vtol_carrier_gerber_with_mcu.zip')} | {F.v('zip with_mcu sha256')} |")
    A(f"| 2 | {p('out/jlcpcb_bom_with_mcu.csv')} | {F.v('jlcpcb_bom_with_mcu.csv sha256')} |")
    A(f"| 3 | {p('out/jlcpcb_cpl_with_mcu.csv')} | {F.v('jlcpcb_cpl_with_mcu.csv sha256')} |")
    A("")
    A(f"- **두 묶음을 섞지 마십시오.** 두 zip은 F.Paste(솔더 페이스트) 층만 다릅니다: 기본 zip은 U1~U4 패드에 페이스트 구멍이 없고 "
      f"(페이스트 패드 {F.v('F.Paste 패드 main')}개), with_mcu zip은 있습니다 ({F.v('F.Paste 패드 with_mcu')}개). "
      f"**두 zip 모두 ESP-12F(U7) 패드에는 페이스트 구멍이 없습니다** (U7 패드 {F.v('ESP 패드 수')}개 중 기본 {F.v('ESP 페이스트 뺀 패드 main')}개, "
      f"with_mcu {F.v('ESP 페이스트 뺀 패드 with_mcu')}개를 뺌, M): 사용자가 납땜할 패드는 무연 HASL 코팅만 된 채로 옵니다. "
      f"페이스트를 뺀 자리: 기본 {F.v('페이스트 없는 자리 main')}; with_mcu {F.v('페이스트 없는 자리 with_mcu')} (JLC가 부품을 달지 않는 자리; "
      "스루홀 커넥터 자리는 원래 페이스트 패드가 없는 도금 구멍입니다).")
    A(f"- 기본 BOM: {F.v('main bom_lines')}줄 (SMT {F.v('main bom_lines_smt')}줄, 스루홀 {F.v('main bom_lines_tht')}줄), 부품 {F.v('main parts')}개, CPL {F.v('main cpl_rows')}행. "
      f"with_mcu: {F.v('with_mcu bom_lines')}줄 (스루홀 {F.v('with_mcu bom_lines_tht')}줄), 부품 {F.v('with_mcu parts')}개, CPL {F.v('with_mcu cpl_rows')}행.")
    A(f"- 기본 BOM에 MCU(U1~U4)가 들어 있는지: {F.v('U1-U4 기본 BOM')}. ESP-12F(U7): {F.v('코드 U7')}. 스루홀 커넥터 J2 / J5: {F.v('코드 J2')}, "
      f"J8~J12: {F.v('코드 J8')}, J14: {F.v('코드 J14')}, J15 / J17: {F.v('코드 J15')} (두 BOM 모두). "
      "센서 자리 J1과 프로그래밍 단자도 들어 있지 않습니다 (모두 사용자 몫).")
    A(f"- 파일 형식: {F.v('주문 화면 확인 시각')}의 주문 화면은 이 변경 전 빌드의 BOM / CPL을 CSV 그대로 받아들였고 "
      f"{F.v('주문 화면 매칭')}했습니다 (M, 주문 화면 2026-11-12). 지금 파일은 같은 형식에서 ESP-12F와 스루홀 커넥터 줄이 빠진 것이지만, "
      "**지금 파일을 올려 모든 줄이 매칭되는 것을 볼 때까지는 NOT_VERIFIED입니다.**")
    A(f"- 두 거버 zip 모두 기판 {F.v('식별 문구 위치')}에 깃 저장소 주소와 연락용 이메일 주소 두 줄이 들어 있습니다 "
      f"({F.v('식별 문구 크기')}, 검사 {F.v('식별 문구 검사')}); 거버 미리보기의 그 면에서 두 줄이 끊기지 않고 보이는지 확인하십시오 "
      "(JLC 인쇄 품질은 NOT_VERIFIED).")
    A(f"- {F.v('변경 날짜')} 재설계로 외곽(탭), 승압부 배치, 프로그래밍 구멍 3개, C42, 배선이 바뀌었습니다 (0장 A). 회로도와 BOM 줄은 그대로이고, CPL은 옮긴 "
      f"부품의 좌표가 바뀌었습니다. 배선은 재설계 때 기록한 세션을 다시 넣어 만든 것이고, 재생에 쓴 배선 전 보드는 기록된 보드와 "
      f"'{F.v('배선 재생 입력 보드')}' 입니다 (M). 보드 파일의 표면처리 표기: '{F.v('보드 표면처리')}'.")
    A("")
    A("## 2. 주문 화면 입력표 (결정 D9, D10, D11, D14, D16)")
    A("")
    A(f"아래 항목 이름과 선택지는 {F.v('주문 화면 확인 시각')}에 사용자의 로그인된 브라우저에서 JLCPCB 주문 화면을 직접 보고 적은 것입니다 "
      f"(M, 주문 화면 2026-11-12; 기록 파일 {p('spec/jlc_order_page_20261112.json')}). 이 도구(스크립트)가 화면을 열거나 로그인한 것은 아닙니다. "
      f"그때 화면에 올라가 있던 파일은 {F.v('주문 화면에 올렸던 파일')} 로, ESP·스루홀 변경과 재설계 전 빌드의 파일(BOM {F.v('주문 화면 올린 줄 수')}줄, "
      "탭이 없는 76 x 70 mm 거버)입니다. "
      f"화면은 거버를 {F.v('주문 화면 거버 인식')} 로 읽었고, BOM / CPL에서 {F.v('주문 화면 매칭')}했습니다 "
      "(모든 줄이 BOM에 적은 JLC 번호 그대로 매칭, 부족·대체 제안 없음). Economic 견적은 그 화면에서 ESP-12F와 스루홀 줄을 실장 목록에서 뺀 상태로 봤습니다 (4장). "
      "**지금 파일(스루홀과 ESP-12F가 처음부터 없는 BOM)은 그 뒤에 만들어져 아직 올려 보지 않았습니다 (NOT_VERIFIED).**")
    A("")
    op = ctx["op"]
    opt = lambda row: ("선택지: " + ", ".join(row["options"]) + (f" ({row['options_note']})" if row.get("options_note") else "")) \
        if row.get("options") else "화면에 보인 값"  # noqa: E731
    note_ko = {
        "Layers": f"보드 {F.v('층 수')}층 (거버 검사 M)",
        "Dimensions": f"거버를 올리면 읽힘. 재설계 전 거버는 {F.v('주문 화면 거버 인식')} 로 읽혔습니다 (M). 새 거버는 탭 때문에 외접 사각형 "
                      f"{F.v('외접 크기')} 로 읽힐 것으로 봅니다 (E; 본체 {F.v('외곽')} + 탭). JLC는 이 크기로 PCB 값을 매기므로 값이 달라질 수 있습니다 "
                      "(새 거버로는 견적을 받지 않았음: NOT_VERIFIED). 화면 값이 외접 크기와 같은지 확인",
        "PCB Qty": "D11 (PCB 5장 중 2장 실장)",
        "PCB Thickness": f"사양 {F.v('두께')}",
        "PCB Color": "D16", "Silkscreen": "D16",
        "Surface Finish": f"D16, {F.v('변경 날짜')} 사용자 결정 (무연). {{opt}}. 보드 파일에는 '{F.v('보드 표면처리')}', 거버 zip 안의 job 파일에는 "
                          f"'{F.v('gbrjob 표면처리')}' 로 적혀 있습니다 (M). 주문은 화면에서 고른 값이 정하므로 화면에서 반드시 고르십시오 "
                          "(JLC가 job 파일을 따르는지는 NOT_VERIFIED)",
        "Outer Copper Weight": "D16 (JLC 기본 4층 적층)", "Inner Copper Weight": "D16 (JLC 기본 4층 적층)",
        "Via Covering": f"D16. {{opt}}. 화면에서 본 것: 이 4층 기판에서 LeadFree HASL을 고르면 Tented가 막히고 Plugged(0 USD)가 선택됨 "
                        f"(말풍선 {F.v('화면 Via 말풍선')}). 화면이 정한 대로 둠 (M)",
        "Min via hole size/diameter": f"화면에 보인 값. 보드의 비아(지름/드릴): {F.v('비아 크기별')} (M)",
        "Mark on PCB": "화면에 보인 값 그대로 둠 (D16: 기본값)",
        "PCBA Type": f"D9, {F.v('변경 날짜')} 사용자 결정. {{opt}}. 아래 'Economic으로 주문하면 달라지는 것'",
        "Assembly Side": "모든 부품이 윗면 (사양)",
        "PCBA Qty": "D11. {opt}",
        "Tooling holes": f"D10: 설계에 툴링 홀 없음 (H 부품 {F.v('툴링 홀(H) 부품')}개, M). {{opt}}. Economic을 고르면 레일 칸 대신 이 칸이 나옴 (M)",
        "Confirm Parts Placement": f"회전이 NOT_VERIFIED인 부품이 있음 (3장). Yes를 고르면 안내 창({F.v('위치 확인 창 안내')})이 뜨고 "
                                   "그 창에 'Do not confirm automatically' 체크 칸이 있음 (M)",
        "Bake Components": f"D14: 주문하지 않음. Economic PCB Assembly에서는 이 항목을 고를 수 없음 (M). "
                           f"Standard에서는 Yes를 고르면 {F.v('베이킹 요금')}가 검토 뒤 청구됨 (M)",
        "Depanel boards & edge rail before delivery": "Economic 주문에는 레일이 없음",
        "Nitrogen reflow soldering": f"화면에 이렇게 표시되고, 견적에 {F.v('견적 economic_no_tht nitrogen_reflow_soldering')} USD 줄로 들어감 (M)",
        "Assembly remark": f"{{opt}}. Yes를 고르면 메모 칸이 열림: {F.v('메모 글자 한도')}자 한도, 'Quote after review'. "
                           "Economic에서도 Yes로 둘 수 있음 (M). 아래 메모를 붙여넣기",
        "Photo Confirmation": "Economic PCB Assembly에서는 고를 수 없음 (M)",
        "Function test": "Economic PCB Assembly에서는 고를 수 없음 (M)",
    }
    head_ko = {"pcb_fields": "PCB", "pcba_fields": "PCB Assembly", "pcba_advanced_fields": "PCB Assembly 추가 옵션"}
    A("| 화면 항목 | 넣을 값 | 근거·화면에서 본 것 |")
    A("|---|---|---|")
    last = None
    for sec, row in ctx["op_rows"]:
        if sec != last:
            A(f"| **{head_ko[sec]}** | | |")
            last = sec
        A(f"| {row['field']} | {F.v('화면 ' + row['field'])} | {note_ko.get(row['field'], '{opt}').replace('{opt}', opt(row))} |")
    A("")
    A(f"- 'Save to cart' 전에 품목 설명(통관 분류)을 골라야 합니다 (M). {F.v('주문 화면 확인 시각')}에는 {F.v('품목 설명')} 을 골랐습니다.")
    A("- 장바구니에 담은 항목의 'Edit Order' 화면은 열 때마다 장바구니에 저장된 값을 다시 불러옵니다 (M). 값을 고친 뒤에는 저장됐는지 다시 열어 확인하십시오.")
    A("")
    A("### Economic으로 주문하면 달라지는 것 (D9)")
    A("")
    A(f"- **레일 대신 툴링 홀 (NOT_VERIFIED):** Standard는 JLC가 기판에 공정 레일을 붙이고 툴링 홀과 기준점을 레일에 둡니다 (화면 문구: {F.v('화면 Standard 레일 문구')}, M). "
      "Economic에는 레일이 없어 JLC가 기판 안에서 툴링 홀 자리를 찾아야 합니다 ('Tooling holes: Added by JLCPCB'). "
      f"이 기판은 윗면 부품 자리 점유율이 {F.v('부품 자리 점유율')} 이고 (M), 부품 자리 {F.v('가장자리 2.5 mm 안 몸체')}곳이 기판 끝에서 2.5 mm 안에 있습니다 (M). "
      "설계에는 툴링 홀이 없습니다. 계획 단계에서 rev 2c 보드의 네 모서리를 찾아봤을 때 한 모서리에는 빈자리가 없었고, 계획은 "
      "JLC가 구리가 있는 곳에 구멍을 낼 수도 있다고 적었습니다 (계획 16장 D10, F). "
      "**JLC가 구멍 자리를 찾는지, 구리(내층 면 포함)를 건드리지 않는지, 아니면 레일을 요구하는지는 주문 뒤 JLC의 검토로만 알 수 있습니다.** "
      "JLC가 구멍 위치를 묻거나 레일을 요구하면 멈추고 아래 Standard 대안을 검토하십시오.")
    A(f"- **가장자리 거리 (JLC가 실장하는 SMT 부품 {F.v('main parts')}개만 다시 계산, M):** 기판 끝에서 가장 가까운 몸체는 {F.v('JLC 실장 가장 가까운 몸체')} 이고, "
      f"가까운 순서로 {F.v('JLC 실장 가까운 몸체')} 입니다. Economic 권장 거리 0.3 mm (계획 10장, F)보다 가까운 JLC 실장 몸체: {F.v('JLC 실장 몸체 0.3 mm 안')} "
      f"(패드: {F.v('JLC 실장 패드 0.3 mm 안')}). 0.3 mm보다 가까운 자리({F.v('JLC 미실장 몸체 0.3 mm 안')})는 모두 사용자가 납땜하는 부품이라 JLC 실장과 무관합니다. "
      f"2.5 mm 안에 있는 JLC 실장 몸체는 {F.v('가장자리 2.5 mm 안 JLC 실장')}개입니다 (with_mcu {F.v('가장자리 2.5 mm 안 with_mcu')}개). "
      "JLC가 받아 주는지는 DFM 결과로만 압니다 (NOT_VERIFIED).")
    A(f"- **리플로 온도:** Economic은 최고 {F.v('리플로 Economic')}, Standard는 {F.v('리플로 Standard')} 입니다 (계획 10장, F). "
      f"데이터시트 정격이 Economic 최악값과 같아 **여유가 없는('한계') 부품**: {F.v('리플로 한계 부품')} (F). "
      f"전해 커패시터 C47의 데이터시트는 {F.v('C47 리플로 횟수')} 허용합니다 (F): 한 면 실장이라 JLC 공정은 한 번이지만, "
      "받은 뒤 열풍기로 주변을 다시 달구는 재작업은 피하십시오. "
      f"문서에서 리플로 정격을 찾지 못한 부품 번호는 {F.v('리플로 미확인 줄 수')}개입니다 (NOT_VERIFIED).")
    A(f"- **부품 종류:** {F.v('주문 화면 확인 시각')}의 화면에서 PCBA Type을 Economic으로 두었을 때 ESP-12F를 뺀 {F.v('Economic 받은 줄 수')}줄이 모두 받아들여졌고 "
      f"'Standard only' 안내는 없었습니다 (M, 이전 업로드 기준). 지금 BOM {F.v('main bom_lines')}줄은 그중 SMT 줄입니다 (Extended {F.v('추정 Economic 피더 종류 수')}종). "
      "지금 파일로 매칭되는지는 올려 봐야 압니다 (NOT_VERIFIED).")
    A("- **베이킹 (D14):** 주문하지 않습니다. 주문 화면에서 Economic PCB Assembly는 Bake Components를 고를 수 없습니다 (M). "
      "습기에 민감한 ESP-12F는 이제 사용자가 납땜하고, JLC가 실장하는 것 중에는 LED (C2286)가 남습니다. "
      "with_mcu 묶음이면 ATmega128A (C6510)도 습기 민감 등급 MSL 3 (F: JLC 부품 기록) 입니다. 베이킹이 꼭 필요하다고 보면 Standard 대안을 검토하십시오.")
    A("- **Economic에서 고를 수 없는 다른 항목 (M):** Photo Confirmation, Function test. 이 주문에는 필요하지 않습니다.")
    A("- **비용:** 4장.")
    A("")
    A("### JLC가 Economic을 거절하면: Standard 대안")
    A("")
    A(f"PCBA Type을 Standard로 바꾸고, Tooling holes 칸 대신 나오는 '{F.v('화면 Standard 레일 칸')}' 을 'Added by JLCPCB' 로 둡니다. "
      f"화면은 {F.v('화면 Standard 레일 문구')} 라고 알려 줍니다 (M) = 긴 변 두 곳에 5 mm 레일. "
      f"그 두 변(y 135 쪽은 탭의 모든 변 포함)의 구리 이격은 {F.v('레일 변 구리 이격 값')} mm (M)로, JLC 기준(F) 라우팅·마우스바이트 "
      f"{F.v('라우팅·마우스바이트 구리 이격')} 에는 여유가 있습니다. **재설계로 y 135 쪽 변에 탭이 생겨 그 변은 곧은 선이 아니므로 V-cut 레일은 쓸 수 없습니다** "
      f"(V-cut 기준 {F.v('V-cut 구리 이격')} 와 무관). 라우팅 탭 / 마우스바이트 레일을 JLC가 해 주는지와 그 값은 NOT_VERIFIED입니다. 그래서 메모 끝에 아래 한 문장을 덧붙입니다. "
      f"같은 조건(무연 HASL, 스루홀 없음, 재설계 전 76 x 70 mm 거버)의 Standard 견적은 PCBA {F.v('견적 standard_no_tht PCBA')} USD, PCB 포함 {F.v('견적 standard_no_tht 합계')} USD로 "
      f"Economic보다 {F.v('Standard 추가 값')} 비쌌습니다 (M, 4장). Standard는 리플로가 {F.v('리플로 Standard')} 로 낮고 베이킹을 고를 수 있으며, 받은 뒤 레일을 꺾어 떼야 합니다.")
    A("")
    rm = ctx["remarks"]
    A("```")
    A(rm["standard_sentence"])
    A("```")
    A("")
    A(f"메모 문장 (기본 묶음, Assembly remark 칸에 그대로 복사; {F.v('메모 글자 수 main')}자 / 한도 {F.v('메모 글자 한도')}자, "
      f"Standard 문장을 덧붙이면 {F.v('메모 글자 수 main + Standard 문장')}자):")
    A("")
    A("```")
    A(rm["main"]["text"])
    A("```")
    A("")
    A(f"with_mcu 묶음의 메모 ({F.v('메모 글자 수 with_mcu')}자, Standard 문장을 덧붙이면 {F.v('메모 글자 수 with_mcu + Standard 문장')}자):")
    A("")
    A("```")
    A(rm["with_mcu"]["text"])
    A("```")
    A("")
    A("- 메모가 말하는 것: (1) 윗면 SMT 실장만 할 것, (2) 스루홀 부품은 하나도 달지 말고 손납땜도 하지 말 것 (스루홀 커넥터는 고객이 사서 납땜), "
      "(3) 일부러 비워 둔 자리 목록 - 그 자리에 부품을 달지 말 것. 목록은 사양 파일에서 JLC가 실장하지 않는 자리를 모두 읽어 만든 것입니다. "
      f"스크립트는 메모가 {F.v('메모 글자 한도')}자를 넘으면 문서를 만들지 않고 멈춥니다.")
    A("- 예전 메모에 있던 'THT는 JLC가 납땜', 'J8~J12 / J14 / J16의 1번 핀 번호가 반대이니 돌리지 말 것' 문장은 뺐습니다: JLC가 그 부품을 놓지 않으므로 "
      "해당이 없습니다. 커넥터 방향과 J14 극성 주의는 사용자 납땜 안내(7장)로 옮겼습니다.")
    A("")
    A("## 3. 결제하기 전에 확인할 것 (하나라도 다르면 멈추고 결제하지 마십시오)")
    A("")
    A(f"1. 주문 당일, 업로드 직전에 재고를 다시 확인합니다: {p('scripts')} 에서 `B:/Claude/.venv/Scripts/python.exe -X utf8 -B jlc_gate.py` "
      f"(인터넷, 읽기만, 약 2~3분). 결과 {p('out/jlc_gate_report.json')} 의 status가 PASS여야 합니다. 마지막 확인: {F.v('JLC 확인 시각')} {F.v('JLC 확인 결과')}.")
    A(f"2. BOM 매칭 화면: 기본 묶음이면 {F.v('main bom_lines')}줄 모두, with_mcu면 {F.v('with_mcu bom_lines')}줄 모두 부품이 매칭되어야 합니다. "
      "'부품 없음', '다른 부품 추천', '재고 부족', 'Economic에서는 쓸 수 없음' 이 하나라도 있으면 멈춥니다 "
      "(대체는 5장 표 1의 정해진 대체 부품만, 그것도 다시 확인 후). "
      f"**목록에 ESP-12F ({F.v('ESP 참고 부품')})나 스루홀 커넥터({F.v('스루홀 참고 번호')})가 보이면 옛 파일을 올린 것이니 멈춥니다.**")
    A("3. 견적에 손납땜 줄(Hand-soldering labor fee)과 수작업 조립 줄(Manual Assembly)이 **없어야** 합니다. 있으면 스루홀 부품이 실장 목록에 들어간 것이니 멈춥니다. "
      f"스루홀을 뺀 Economic 견적에는 두 줄이 없었고, 넣은 견적에는 {F.v('견적 economic_tht_placed hand_soldering_labor_fee')} USD와 "
      f"{F.v('견적 economic_tht_placed manual_assembly')} USD로 있었습니다 (M).")
    A(f"4. 부품 위치 미리보기(Confirm Parts Placement)를 {p('out/cpl_rotation_check.png')} 그림과 하나씩 비교합니다. 특히 극성: 탄탈 C45 / C52, 전해 C47 "
      f"(거꾸로면 화재 위험), 그리고 회전이 확인 안 된 {F.v('회전 미확인')}. U8 / U9 / J7의 1번 핀 방향. "
      f"**U7 (ESP-12F), U1~U4, 스루홀 커넥터 자리({F.v('스루홀 목록')})는 비어 있어야 합니다.** "
      f"회전 근거는 {p('out/cpl_rotation_table.csv')} 의 status 칸에 있습니다. "
      f"미리보기 화면에는 {F.v('미리보기 화면 문구')} 라고 적혀 있습니다 (M): **최종 위치는 주문 뒤 4~6시간 후 Order History의 DFM Analysis에서 다시 확인합니다.** "
      f"{F.v('주문 화면 확인 시각')}의 미리보기(이 변경 전 파일, 낮은 확대, 눈으로 봄)에서는 C45 / C47 / C52의 '+' 표시가 기판 실크의 '+' 와 같은 쪽에, "
      f"U8의 1번 핀 점이 실크 표시 자리에 있었습니다 (M). {F.v('미리보기 구분 못 한 부품')} 는 그 확대로 구분하지 못했습니다 (NOT_VERIFIED). "
      f"JLC 풋프린트와 1번 핀 번호가 반대로 매겨진 부품(지금 CPL 기준): {F.v('1번 핀 번호 반대')} - 그런 부품이었던 J8~J12, J14, J16은 이제 CPL에 없습니다.")
    A(f"5. DFM 결과: 툴링 홀 위치(기판 안, 구리를 건드리지 않는지), 가장자리 가까운 부품({F.v('JLC 실장 가장 가까운 몸체')} 등)에 대한 이의, "
      "구멍·간격·선폭 지적이 없어야 합니다. JLC가 툴링 홀 자리가 없다며 레일을 요구하면 2장의 Standard 대안으로 견적을 다시 받아 비교한 뒤 정합니다. "
      "JLC가 이의를 달면 주문 방식이나 부품을 마음대로 바꾸지 말고 멈춘 뒤 다시 검토합니다.")
    A(f"6. 견적 금액을 4장과 비교합니다 (재설계 전 76 x 70 mm 거버의 기본 묶음: 배송 제외 {F.v('견적 economic_no_tht 합계')} USD, M). "
      f"지금 거버는 외접 {F.v('외접 크기')} 라 PCB 값이 다를 수 있습니다 (NOT_VERIFIED). 크게 다르면 이유를 확인합니다.")
    A("7. 'Confirm Parts Placement'는 Yes로 하고, 뜨는 창에서 'Do not confirm automatically'를 체크합니다 (2장). 체크하지 않으면 "
      f"자동으로 확정됩니다 (화면 안내: {F.v('위치 확인 창 안내')}, M). 주문 뒤 JLC가 위치 확인 사진이나 문제를 보내면 4번과 같은 기준으로 "
      "비교하고, 틀린 것이 있으면 바로잡아 달라고 답하거나, 판단이 서지 않으면 주문 보류(hold) 또는 취소를 요청한 뒤 다시 검토합니다. "
      "결제와 이 확인의 정확한 순서는 주문 흐름을 끝까지 거친 적이 없어 NOT_VERIFIED입니다.")
    A("8. 메모(Assembly remark)를 넣으면 화면에 'Quote after review' 라고 나옵니다 (M): 검토 뒤 금액이 바뀔 수 있으니 최종 금액을 다시 봅니다.")
    A("")
    A("## 4. 비용: 주문 화면에서 본 견적 (M)")
    A("")
    A(f"**아래 견적은 모두 재설계 전 76 x 70 mm 거버로 받은 것입니다. 지금 파일(탭 포함, 외접 {F.v('외접 크기')})의 값은 NOT_VERIFIED입니다** "
      "(PCB 크기가 커져 PCB 값이 달라질 수 있고, 실장 부품 줄 수는 같습니다).")
    A("")
    A(f"아래 네 열은 모두 {F.v('주문 화면 확인 시각')}에 주문 화면에서 본 견적입니다 (M, 주문 화면 2026-11-12). 공통 조건: PCB 5장 / 실장 2장, 배송비·세금 제외, 단위 USD. "
      "'-' 는 그 견적에 그 줄이 없었다는 뜻입니다. **이번 주문은 첫 열 (Economic, 스루홀 없음)입니다.**")
    A("")
    qid = ctx["QID"]
    head = {"economic_no_tht": "**이번 주문**: Economic, 스루홀 없음", "economic_tht_placed": "Economic, 스루홀 6줄을 JLC가 납땜",
            "standard_no_tht": "Standard, 스루홀 없음", "standard_enig_as_uploaded": "Standard, ENIG, 처음 올린 그대로"}
    A("| 항목 | " + " | ".join(head[q_] for q_ in qid) + " |")
    A("|---|" + "---|" * len(qid))
    A("| 조건 | " + " | ".join(F.v(f"견적 {q_} 조건") for q_ in qid) + " |")
    for k_, ko in ctx["QK"]:
        A(f"| {ko} | " + " | ".join(F.v(f"견적 {q_} {k_}") for q_ in qid) + " |")
    A("| **PCBA 합** | " + " | ".join(f"**{F.v(f'견적 {q_} PCBA')}**" for q_ in qid) + " |")
    A("| PCB 5장 | " + " | ".join(F.v(f"견적 {q_} PCB") for q_ in qid) + " |")
    A("| **합계 (배송 제외)** | " + " | ".join(f"**{F.v(f'견적 {q_} 합계')}**" for q_ in qid) + " |")
    A("| 조립 기간 (화면 표시) | " + " | ".join(F.v(f"견적 {q_} 조립 기간") for q_ in qid) + " |")
    A("")
    A(f"- 견적을 본 방법: 화면에는 이 변경 전 빌드의 파일(BOM {F.v('주문 화면 올린 줄 수')}줄)이 올라가 있었고, 표면처리를 LeadFree HASL로 바꾸고 "
      "ESP-12F(와 열에 따라 스루홀 6줄)를 실장 목록에서 뺀 상태로 읽었습니다. 넷째 열만 처음 올린 그대로(ENIG, ESP-12F와 스루홀 포함)입니다. "
      f"지금 BOM은 {F.v('main bom_lines')}줄로 첫 열의 부품 줄 수와 같지만, **지금 파일을 올려서 받은 견적은 아닙니다 (NOT_VERIFIED).**")
    A(f"- 넷째 열의 PCBA 항목을 더하면 {F.v('견적 as_uploaded 항목 합')} 이고 화면의 합은 {F.v('견적 standard_enig_as_uploaded PCBA')}이었습니다 (차이는 화면 반올림).")
    A(f"- 스루홀 6줄을 JLC에 맡길 때와의 차이: Economic 기준 합계 {F.v('스루홀 JLC 납땜 값')} (M, 첫 열과 둘째 열의 차). "
      f"같은 커넥터를 LCSC에서 2장 분 사면 약 {F.v('LCSC 커넥터 값')} 입니다 (E, 배송비 제외; {F.v('LCSC 기록 시각')}의 LCSC 기록). "
      "이번 주문에서는 사용자가 사서 납땜하는 쪽으로 정했습니다.")
    A(f"- 표면처리: 무연 HASL의 추가 요금은 {F.v('견적 표면처리 추가')} 이었습니다 (M; ENIG는 넷째 열의 PCB 칸).")
    A(f"- 수수료 표로 계산한 추정(E)과 비교: 지금 BOM의 Economic PCBA 추정은 {F.v('추정 Economic 합')} (Extended {F.v('추정 Economic 피더 종류 수')}종 "
      f"수수료 {F.v('추정 Economic 피더')} 포함; 스루홀 줄이 없어 손납땜 수수료는 0)이고, 화면의 PCBA 합은 {F.v('견적 economic_no_tht PCBA')} USD입니다. 추정표는 질소 리플로 줄을 모르고 부품값 계산 방식이 달라 "
      f"차이가 납니다. 전체 추정표: {p('out/jlc_fee_estimate.csv')} (E, 참고용).")
    A("- **빠진 것 / 바뀔 수 있는 것:** 배송비, 세금, 메모 검토('Quote after review') 뒤의 금액 변동, 주문일의 부품 단가. 실제 금액은 결제 직전 화면의 견적뿐입니다.")
    A(f"- Economic은 베이킹을 고를 수 없습니다 (2장). Standard에서 베이킹을 넣으면 {F.v('베이킹 요금')}가 더해집니다 (M).")
    A("")
    A("## 5. 재고가 적은 줄과 대체 부품")
    A("")
    A(f"- 얇은 재고(기준 {F.v('얇은 재고 기준')} 미만): {F.v('얇은 재고 줄')}.")
    A(f"- 지켜볼 줄 (2000개 미만): {F.v('재고 주의 줄')}.")
    A("- 대체는 계획에 적힌 순서대로만 씁니다. 다른 부품으로 바꾸는 것은 설계 변경이라 다시 검사하고 사용자가 정해야 합니다.")
    A("")
    # split by footprint compatibility from the plan's own alternate notes (rev 3 review J1); alternates of parts the
    # user now buys (2026-11-12) are not JLC substitutions any more
    notes = {a_["for"]: a_.get("note", "") for a_ in ctx["plan"]["alternates"]}
    same, check, newfp, user_alt = [], [], [], []
    for k in alt:
        n = notes.get(k)
        if n is None:
            raise SystemExit(f"docs_ko: gate alternate group {k!r} has no note in spec/rev3_plan.json alternates")
        refs_k = set(re.findall(r"\b[A-Z]+\d+\b", k.replace("..", " ")))
        of_user = any(c_ in k for c_ in ctx["bought_codes"]) or "THT" in k
        if of_user:
            if not any(c_ in k for c_ in ctx["bought_codes"]) and not (refs_k & set(ctx["tht_refs"])):
                raise SystemExit(f"docs_ko: alternate group {k!r} is read as 'user-bought' but names no user-fitted part")
            (newfp if ("new footprint" in n or "footprint not chosen" in n) else user_alt).append(k)
        elif "new footprint" in n or "footprint not chosen" in n:
            raise SystemExit(f"docs_ko: alternate group {k!r} needs a new footprint but is not a user-bought part - not covered by the text")
        elif "same series and land pattern" in n or "same MPN" in n:
            same.append(k)
        else:
            check.append(k)
    A("**표 1. JLC가 실장하는 부품의 대체 부품** (이 파일 그대로, BOM 줄의 부품 번호만 바꿈). 'PASS'는 주문 가능 여부(재고)만 뜻합니다.")
    A("")
    A("| 원래 부품 | 대체 부품 (주문 가능 수량, 판정) | 풋프린트 호환 | 계획 메모 (spec/rev3_plan.json) |")
    A("|---|---|---|---|")
    for k in same:
        A(f"| {k} | {F.v('대체 ' + k)} | 같은 패키지 (F, 계획 메모); 미리보기로 확인 | {notes[k]} |")
    for k in check:
        A(f"| {k} | {F.v('대체 ' + k)} | **NOT_VERIFIED** - 쓰기 전에 제조사 도면과 KiCad 풋프린트를 대조해야 함 | {notes[k]} |")
    A("| U9 (C20345924) | 없음 - 같은 사양의 두 번째 부품이 없습니다. 재고가 없으면 주문을 미룹니다 (계획 7장, F) | - | - |")
    A("")
    A("**표 2. 사용자가 사는 스루홀 부품의 대체 부품** (JLC 주문과 무관; 구하기 어려울 때의 계획 순서. 수량은 JLC 조립 창고의 것이고 LCSC 소매 재고가 아닙니다).")
    A("")
    A("| 원래 부품 | 대체 부품 (JLC 창고 수량, 판정) | 풋프린트 호환 | 계획 메모 |")
    A("|---|---|---|---|")
    for k in user_alt:
        A(f"| {k} | {F.v('대체 ' + k)} | **NOT_VERIFIED** - 사기 전에 제조사 도면과 KiCad 풋프린트를 대조해야 함 | {notes[k]} |")
    A("")
    A(f"- 계획에는 'JLC가 스루홀 손납땜을 거절할 때'를 위한 SMD 예비안({'; '.join(newfp)})이 있었습니다. "
      "이제 JLC가 스루홀을 달지 않으므로 이번 주문과 무관하고, 새 풋프린트가 필요한 부품이라 이 파일로는 어차피 쓸 수 없습니다.")
    A("")
    A("## 6. 따로 사야 하는 것 (JLC는 낱개 부품을 보내주지 않습니다)")
    A("")
    A(f"목록 {F.v('따로 살 품목 수')}줄. 읽기 쉬운 전체 목록(비고, LCSC 재고·가격 포함): {p('out/' + KO_NAME)}; 같은 내용의 CSV: {p('out/off_board_purchase.csv')}. "
      f"(a) 묶음 = JLC BOM에서 빠져 이번에 사용자가 사게 된 ESP-12F와 스루홀 커넥터 {F.v('스루홀 종류 수')}종 (보드당 커넥터 {F.v('스루홀 부품 수')}개). "
      f"(a) 묶음을 LCSC에서 2장 분 사면 약 {F.v('LCSC (a) 묶음 값')} (E; 단가는 {F.v('LCSC 기록 시각')}의 LCSC 기록 M, 최소 주문 수량 반영, 배송비 제외; "
      f"그 시각에 재고가 모자란 것: {F.v('LCSC 재고 부족')}). LCSC 말고는 재고와 가격을 확인하지 않았습니다 (NOT_VERIFIED).")
    A("")
    gname = {g_: ko_ for g_, _en, ko_ in purchase_list.GROUPS}
    A("| 묶음 | 품목 | 부품 번호 | 보드당 | 2장 분 | 어디에 | 구입처 (제안) |")
    A("|---|---|---|---|---|---|---|")
    for r_ in ctx["prow"]:
        pn = " / ".join(x_ for x_ in (r_["mpn"], f"LCSC {r_['lcsc']}" if r_["lcsc"] else "") if x_) or "-"
        A(f"| ({r_['group']}) {gname[r_['group']]} | {r_['item_ko']} | {pn} | {r_['qty_board']} | {r_['qty_2']} | {r_['where_ko']} | {r_['channel_ko']} |")
    A("")
    A("## 7. 보드를 받은 뒤 할 일 (계획 15장 순서 + ESP-12F와 스루홀 커넥터 납땜)")
    A("")
    A("순서의 뼈대: 검사 → 전원 커넥터 J14·J15 납땜 → **MCU·ESP 없이 전원 시험 (+5V, +3V3)** → 부하 시험 → MCU → ESP-12F → 나머지 스루홀 커넥터 → 다리 자르기 → 모듈 → 프로그래밍. "
      "받은 보드에는 SMT 부품만 붙어 있고 커넥터가 하나도 없습니다 (GPS용 J7만 SMD라 붙어 옴). "
      "ESP-12F와 MCU를 달기 전에 전원을 넣어 두 전압을 잴 수 있습니다: 승압 회로와 3.3 V 레귤레이터는 JLC가 실장하고, 켜는 데 MCU나 ESP가 필요하지 않습니다.")
    A("")
    A("1. **검사:** 탄탈 C45 / C52와 전해 C47의 극성을 실크 표시와 비교, U8 / U9 / J7 방향, U8(0.5 mm 피치) 납땜 브리지가 없는지 확대해서 봅니다. "
      f"U7 (ESP-12F), U1~U4, 스루홀 커넥터 자리({F.v('스루홀 목록')})는 비어 있어야 하고, U7과 U1~U4 패드에는 납(페이스트 자국)이 없이 무연 HASL 코팅만 있어야 합니다.")
    A("2. **레일:** Economic 주문에는 떼어낼 레일이 없습니다. (Standard 대안으로 주문했다면 여기서 보드 둘레의 레일을 꺾어 떼고 모서리를 다듬습니다.)")
    A(f"3. **전원 커넥터 J14 (배터리)와 J15 (스위치) 납땜** (납땜 {F.v('커넥터 J14 납땜점')} + {F.v('커넥터 J15 납땜점')}곳): 전원 시험에 필요해서 먼저 답니다. "
      f"두 커넥터는 칩 자리에서 멀어 먼저 달아도 MCU·ESP 납땜을 가리지 않습니다 (J14: {F.v('J14 칩 거리')}; J15: {F.v('J15 칩 거리')}, M). "
      "방향은 7번의 표를 따릅니다.")
    A(f"   - **J14를 180도 돌려 달면 배터리의 +와 -가 뒤바뀝니다.** J14의 1번 핀({F.v('커넥터 J14 1번 핀')})이 배터리 + 입니다 (M: 보드 파일의 네트). "
      f"윗면에는 J14 옆에 +/- 글자가 없습니다. 아랫면 실크에는 {F.v('J14 라벨')} 이 있고, 아랫면에서 보면 (좌우가 뒤집혀) "
      f"네모난 패드가 {F.v('J14 아랫면에서 1번 핀')}에 있습니다. 라벨의 두 기호는 아랫면에서 본 패드 순서대로 적혀 있습니다: "
      f"끝 글자 {F.v('J14 라벨 끝 글자')} "
      f"(M: 보드 파일에서 글자 획의 위치와 패드 좌표를 비교한 검사 {F.v('라벨 검사')}; 그림 {p('out/render/a3_as_delivered_bottom.png')} 에서 눈으로 확인). "
      "2026-11-12 03:57까지의 파일에는 이 라벨이 '1S BAT +-' 로 적혀 '+' 가 GND 패드 쪽에 있었습니다 (검증에서 지적, 고침). "
      "**네모난 패드 = + 입니다.** "
      "납땜하기 **전에** 커넥터를 구멍에 끼운 채 배터리 플러그를 꽂아 보고, 멀티미터로 배터리 +가 네모난 패드 쪽 핀에 오는지 확인하십시오 "
      "(팩의 극성과 맞물림은 NOT_VERIFIED; 역극성 보호 Q10이 있지만 시험된 적이 없습니다).")
    A(f"   - **뒷면 다리 규칙 (모든 스루홀 커넥터에 적용):** 납땜한 뒤 뒷면으로 나온 다리를 재서 {F.v('뒷면 다리 규칙')} 보다 긴 것은 모두 자릅니다 (폼테이프가 뒷면에 붙기 때문). "
      f"{F.v('다리 규칙 초과')} 은 도면값이 규칙을 넘어 반드시 자릅니다 (다리 {F.v('다리 규칙 초과 다리 수')}개). "
      f"{F.v('다리 공차 끝 초과')} 는 도면값은 규칙 안이지만 공차 끝에서 넘으므로 재 보고 자릅니다. {F.v('다리 길이 미확인')} 은 도면값을 확인하지 못했으니 재 봅니다. "
      "이 도면값은 6장 (a) 묶음의 참고 부품 번호를 샀을 때의 값입니다 (F); 다른 부품을 샀다면 모두 재 보십시오.")
    A("4. **MCU·ESP-12F·모듈 없이 벤치 전원 시험:** J14에 전류 제한(100 mA) 전원, J15는 단락 (스위치 리드를 꽂아 켜거나, 두 핀을 이은 PH 2핀 플러그를 꽂음). "
      "아래 값을 적습니다. 범위를 벗어나면 멈춥니다. "
      "ESP-12F를 달기 전에 +3V3을 재 두면, 3.3 V가 잘못 나와도 모듈(공급 3.0~3.6 V, F)이 상하지 않습니다.")
    A("")
    A("| 측정 | 허용 범위 (E, 계획) | 측정값 |")
    A("|---|---|---|")
    A(f"| 전압을 천천히 올릴 때 5 V가 켜지는 전압 | {F.v('켜짐 전압')} (3.3 V에서는 꺼져 있어야 함) | |")
    A(f"| 켜진 뒤 +5V | {F.v('+5V 범위')} | |")
    A("| 켜진 뒤 +3V3 (ESP-12F를 달기 전) | 3.0 V 이상 3.6 V 이하 (ESP-12F 공급 범위, F) | |")
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
      f"패드 1 mm 안의 부품 자리 수 (JLC가 붙인 것과 나중에 달 커넥터 포함): {F.v('MCU 1 mm 안 부품')} (M). U1을 GY-521보다 먼저 납땜하십시오 (GY-521이 U1 위에 걸침). "
      f"그림: {p('out/render/c5_mcu_pin1.png')}. "
      "패드는 무연 HASL입니다. 무연 HASL 표면은 ENIG보다 덜 평평한 것이 일반적이므로 (이 보드에서 잰 것은 아님, NOT_VERIFIED) "
      "칩이 한쪽으로 뜨지 않았는지 네 변을 확대해 보십시오.")
    A(f"   **ESP-12F (U7) 납땜** (두 파일 묶음 모두에서 사용자 몫; {F.v('변경 날짜')} 사용자 결정): 모듈은 옆면에 반원 홈이 있는 패드 "
      f"{F.v('ESP 패드 수')}개로 붙습니다 (패드 크기 {F.v('ESP 패드 크기')}, M). 그중 이 보드의 회로에 이어진 패드는 {F.v('ESP 네트 있는 패드')} 이고, "
      f"나머지 {F.v('ESP 네트 없는 패드')}개는 어떤 네트에도 이어지지 않아 모듈을 붙잡는 역할만 합니다 (M). "
      "거버에 이 패드의 솔더 페이스트 구멍이 없으므로 패드는 무연 HASL 코팅만 된 상태입니다 (1장).")
    A("   - 언제 (보드에서 잰 거리로 정한 순서, M):")
    A("     - 4번 전원 시험 **뒤에**: +3V3이 범위 안인 것을 본 다음에 모듈을 답니다.")
    A(f"     - MCU **뒤에** (권장): ESP-12F 몸체는 {F.v('ESP-MCU 거리')} 떨어져 있습니다. 모듈 높이 {F.v('ESP 몸체 높이')} (A)가 "
      f"MCU {F.v('MCU 몸체 높이')} (A)보다 높아서, 모듈을 먼저 달면 {F.v('ESP에 가까운 MCU')} 의 그쪽 핀 줄을 납땜하고 확대 검사할 때 인두와 시야를 가립니다. "
      "MCU를 먼저 달면 ESP 납땜을 가리는 것이 없습니다. 꼭 지켜야 하는 순서는 아니고, 이 거리에서 권하는 순서입니다.")
    A(f"     - 서보 커넥터 {F.v('ESP 가까운 커넥터')} 와 J3 굽기 헤더(직접 납땜한다면)보다 **먼저**: 이 커넥터들은 ESP 패드에서 5 mm 안에 서고, "
      f"빈 헤더 구멍 {F.v('ESP 옆 빈 헤더 구멍')} 는 ESP 패드 바로 옆입니다. 먼저 세우면 그쪽 ESP 패드에 인두가 들어가기 어렵습니다.")
    A(f"   - 어떻게: 안테나 쪽(패드가 없는 끝)이 기판 변 쪽을 향합니다 (모듈 몸체는 그 변에서 {F.v('U7 몸체-변 거리')}). 1번 핀을 실크 표시에 맞추고, "
      "모서리 패드 하나만 먼저 붙여 실크 윤곽과 위치를 맞춘 뒤 나머지를 납땜합니다. "
      f"ESP 패드에서 1 mm 안에 JLC가 이미 붙인 부품이 있습니다: {F.v('ESP 옆 JLC 부품')} (M). 가장 가까운 이웃 패드까지 {F.v('ESP 패드-이웃 패드 최소')} 입니다. "
      "납이 넘어가 붙지 않게 하고, 탄탈 C52를 오래 달구지 마십시오.")
    A("   - 끝난 뒤: 전원을 넣기 전에 멀티미터로 8번 패드(+3V3)와 15번 패드(GND) 사이가 단락이 아닌지 재고, 4번의 +3V3 측정을 한 번 더 합니다. "
      f"그림: {p('out/render/c4_esp12f_keepout.png')}")
    A(f"7. **나머지 스루홀 커넥터 {F.v('나머지 커넥터 수')}개 납땜** (납땜 {F.v('나머지 커넥터 납땜점')}곳; J14·J15까지 합치면 커넥터 {F.v('스루홀 부품 수')}개, {F.v('스루홀 납땜점')}곳). "
      "MCU와 ESP-12F **뒤에** 답니다. 이유 (M): 다음 커넥터는 MCU 패드에서 5 mm 안에 섭니다 - "
      f"{F.v('MCU 가까운 커넥터')}. 특히 J5 소켓은 U2 핀 줄 바로 옆이라 먼저 달면 U2의 그 변을 납땜하고 검사하기 어렵습니다. "
      f"ESP 패드에서 5 mm 안: {F.v('ESP 가까운 커넥터')}. "
      "(with_mcu 묶음으로 MCU를 JLC가 붙였다면 MCU 쪽 제약은 없고, ESP-12F만 먼저 달면 됩니다.)")
    A(f"   - 순서: 낮은 것부터 답니다 - {F.v('나머지 커넥터 높이순')}. 낮은 것을 먼저 달아야 기판을 뒤집어 납땜할 때 부품이 빠지지 않게 받치기 쉽습니다. "
      "기판을 뒤집어 뒷면에서 납땜하므로, 핀 하나만 먼저 붙이고 윗면에서 방향과 밀착(몸체가 기판에 붙었는지)을 확인한 뒤 나머지를 납땜합니다.")
    A("   - 방향: 커넥터 몸체를 실크 윤곽에 맞춥니다. 아래 표의 '위쪽 / 아래쪽 / 왼쪽 / 오른쪽'은 윗면 그림 "
      f"{p('out/render/a1_as_delivered_top.png')} (부품 면을 위에서 본 그림; ESP-12F 자리가 위쪽 변, J8~J11이 왼쪽 변)에서 본 방향입니다 (M: 보드 파일의 패드와 몸체 윤곽에서 잼). "
      "직각형은 핀(또는 소켓 입구)이 표의 방향을 보게 눕히고, 수직형(J14, J15, J17)은 핀 줄에서 윤곽까지의 거리가 양쪽이 달라 한 방향으로만 윤곽에 맞습니다. "
      "1번 핀은 네모난 패드이고 나머지는 둥근 패드입니다. 풋프린트 대조 기록: J14 / J15 / J17은 `out/fit_checks.json` connector_drawings "
      "(J14 구멍 지름은 NOT_VERIFIED), 소켓과 핀헤더는 계획서의 도면 대조 (F). 산 부품을 실제로 꽂아 본 적은 없습니다 (NOT_VERIFIED).")
    A("")
    A("| 자리 | 부품 (참고 번호) | 납땜 | 1번 핀 (네트, 위치) | 몸체 방향 | 가까운 기판 변 | 높이 (A) |")
    A("|---|---|---|---|---|---|---|")
    for ref, name_ko, mpn, lcsc in ctx["tht_table"]:
        A(f"| {ref} | {name_ko} ({mpn}, LCSC {lcsc}) | {F.v(f'커넥터 {ref} 납땜점')}곳 | {F.v(f'커넥터 {ref} 1번 핀')} | {F.v(f'커넥터 {ref} 몸체')} | "
          f"{F.v(f'커넥터 {ref} 가까운 변')} | {F.v(f'커넥터 {ref} 높이')} |")
    A("")
    A("   - 다 달았으면 3번의 다리 규칙대로 뒷면 다리를 모두 재고 자릅니다. 폼테이프는 그 뒤에 붙입니다.")
    A("   - J3 굽기 헤더와 다른 프로그래밍 헤더를 직접 납땜한다면 이 단계 뒤에 답니다 (D6).")
    A(f"8. **모듈 장착:** 각 모듈에 인쇄된 핀 순서를 보드 실크와 먼저 비교합니다. "
      f"보드 실크 이름표 읽는 법 (M, 검사 {F.v('라벨 검사')}): 핀 줄과 나란히 적힌 이름표는 그 면에서 본 패드 순서대로 적혀 있습니다 "
      f"({F.v('핀 줄과 나란한 라벨')}; 아랫면 것은 아랫면에서 볼 때의 순서라서 윗면에서 보면 좌우가 반대입니다). "
      f"핀 줄과 직각으로 적혔거나 두 줄짜리 헤더 옆에 적힌 이름표({F.v('1번 핀부터 적은 라벨')})는 1번 핀(네모난 패드)부터의 순서입니다. "
      f"GY-521은 자기 1x8 핀으로 J1에 납땜 ({F.v('J1 납땜점')}곳), 받침 {F.v('GY-521 받침 높이')} (A) "
      f"({p('out/render/e2a_gy521_J1.png')}); BMP280은 직접 단 소켓 J2에 꽂음 (헤더 {F.v('BMP280 헤더 종류')}이 아니면 직각 핀을 먼저 납땜, {p('out/render/e2b_bmp280_J2.png')}); "
      f"OpenLog는 직각 6핀 헤더를 납땜한 뒤 microSD 쪽이 위로 오게 직접 단 소켓 J5에 꽂음 ({p('out/render/e2c_openlog_J5.png')}); "
      f"NEO-7M은 윗뚜껑에 두고 JST-GH 케이블을 J7에 ({p('out/render/e3_gps_lead_J7.png')}). 받은 상태와 완성 상태 비교: "
      f"{p('out/render/e1_delivered_vs_completed.png')}, 전체 안내: {p('out/render/e4_overview_callouts.png')}. "
      f"OpenLog는 먼저 아랫면(ATmega328 쪽) 높이를 재고 끼워 보십시오: 아랫면 부품이 1.2 mm라고 보는 보수적 경우(A) 꽂는 길에서 "
      f"{F.v('모듈 간섭 보수적 충돌')} 이 겹칩니다.")
    A("9. **프로그래밍 (전원은 한 가지만):** 로직 배터리(3.73 V 넘게 충전) 또는 J14에 4.0 V 벤치 전원으로 보드를 켜고, USBasp의 타깃 전원 점퍼는 "
      "**빼 둡니다** (5 V를 두 곳에서 넣지 않기). 단자는 D6 선택대로 (헤더 납땜 + 점퍼, 또는 포고핀 어댑터). ESP-12F는 J3로 3.3 V 신호로만 굽습니다.")
    A("10. 첫 Wi-Fi 송신 때 +5V와 +3V3의 최저값을 기록합니다.")
    A("11. 커넥터 연결: 배터리 J14, 스위치 J15, GPS J7, 서보 J8~J11 / ESC J12, 아밍 J16, 부저 J17. 장착은 rev 2c와 같이 3 mm 폼테이프 + 케이블타이 1개 "
      "(x -3..+3 mm 띠). **장착 전에 띠를 다시 비우십시오:** 프로그래밍 헤더를 띠 구멍에 납땜했다면 떼거나 잘라 내야 케이블타이와 폼테이프가 "
      "제자리에 앉습니다 (뒷면으로 나온 다리도 1.5 mm 규칙). 비행용 보드는 포고핀 어댑터로만 굽는 것이 가장 간단합니다.")
    A("")
    A("## 8. MCU·센서 말고 사용자가 해야 하는 일 (숨기지 않고 모음)")
    A("")
    A("처음 요청은 'MCU와 센서만 빼고 모두 JLC가 실장'이었습니다. 2026-11-12 결정 뒤 아래 일은 이 보드에서 사용자 몫입니다:")
    A("")
    A(f"- **ESP-12F (U7) 구매와 납땜** ({F.v('변경 날짜')} 사용자 결정): 패드 {F.v('ESP 패드 수')}개 (7장 6번). 모듈은 따로 삽니다 "
      f"(6장; 참고 부품 LCSC {F.v('ESP 참고 부품')}).")
    A(f"- **스루홀 커넥터 {F.v('스루홀 부품 수')}개 구매와 납땜** ({F.v('변경 날짜')} 사용자 결정): {F.v('스루홀 목록')}, 납땜 {F.v('스루홀 납땜점')}곳 (7장 3번, 7번). "
      f"부품 {F.v('스루홀 종류 수')}종은 따로 삽니다 (6장 (a) 묶음). J14는 방향이 틀리면 배터리 극성이 뒤집힙니다.")
    A("- 레일 떼기: Economic 주문에는 없습니다 (Standard 대안으로 주문했을 때만, 7장 2).")
    A(f"- 뒷면 다리 자르기: 직접 납땜한 모든 스루홀 다리가 대상입니다. {F.v('다리 규칙 초과')} (다리 {F.v('다리 규칙 초과 다리 수')}개)는 도면값으로 규칙 초과, "
      f"{F.v('다리 공차 끝 초과')} 는 공차 끝에서 초과, {F.v('다리 길이 미확인')} 은 미확인 (7장 3번).")
    A(f"- 프로그래밍 단자 ({F.v('프로그래밍 헤더 선택')}안): J3, J19~J22 (2x3) 와 J24~J26 (1x2)는 빈 구멍입니다. 헤더를 직접 납땜(36곳, 계획 D6)하고 점퍼 3개를 끼우거나, "
      f"포고핀 어댑터를 대고 굽습니다 (접촉 NOT_VERIFIED). JLC가 띠 밖에 헤더를 실장하는 B안은 {F.v('B안 결론')} ({F.v('B안 2x3 자리')}). "
      "이 구멍들은 케이블타이 띠 안에 있으므로, 헤더를 납땜했다면 장착 전에 떼거나 잘라서 띠를 비워야 합니다 (7장 11번).")
    A("- OpenLog에 직각 6핀 헤더 납땜 (모듈 쪽 6곳). OpenLog는 센서가 아니라 기록기라 D5에서 '센서처럼 사용자가 장착'으로 정했습니다.")
    A("- BMP280 헤더가 일자형으로 오면 직각 핀으로 바꿔 납땜.")
    A("- GPS 케이블(JST-GH 4선), 스위치·부저·아밍 리드 준비 (6장 목록).")
    A("- 벤치 전원 시험과 기록 (7장 4~5) - 보드가 만들어진 적이 없어서 필요합니다.")
    A("- 폼테이프와 케이블타이로 장착.")
    A("")
    A("## 9. 사용자 결정 D1~D17과 이번 파일에 쓴 값")
    A("")
    A("계획의 기본값에 2026-11-12 사용자 결정(D9 Economic, D14 베이킹 없음, D16 무연 HASL, ESP-12F와 스루홀 커넥터 직접 실장)을 반영해 만들었습니다. "
      "'이번 파일에서 확인한 값'은 생성된 파일에서 읽은 것입니다.")
    A("")
    A("| Id | 질문과 기본값 (계획 19장, 2026-11-12 변경 반영) | 이번 파일에서 확인한 값 |")
    A("|---|---|---|")
    built = {
        "D1": f"U8 {F.v('코드 U8')}, U9 {F.v('코드 U9')}, L5 {F.v('코드 L5')} (BOM)",
        "D2": f"{F.v('UVLO 저항')} → 켜짐 {F.v('켜짐 전압')}, 꺼짐 {F.v('꺼짐 전압')} (E)",
        "D3": f"C45 {F.v('코드 C45')} (JLC BOM); J8~J12 {F.v('참고 코드 J8')}, J2 / J5 {F.v('참고 코드 J2')} 는 사용자가 사는 부품의 참고 번호 (JLC BOM: {F.v('코드 J8')})",
        "D4": f"기본 BOM에 U1~U4 {F.v('U1-U4 기본 BOM')}; with_mcu 쌍 U1~U4 = {F.v('U1-U4 with_mcu')} (두 쌍 모두 생성)",
        "D5": f"소켓 J5는 사용자가 사서 납땜 (참고 {F.v('참고 코드 J2')}; JLC BOM: {F.v('코드 J5')}), OpenLog는 따로 구매",
        "D6": f"{F.v('프로그래밍 헤더 선택')}안 (빈 구멍); B안 검색 {F.v('B안 2x3 자리')} → {F.v('B안 결론')}",
        "D7": f"J15 / J17 참고 부품 {F.v('참고 코드 J15')} (THT, 사용자가 납땜; JLC BOM: {F.v('코드 J15')}); 다리 자르기 {F.v('다리 규칙 초과')}",
        "D8": f"Q1 등 {F.v('코드 Q1')} (BOM)",
        "D9": f"주문 화면에서 사용자가 선택 (2장: PCBA Type = {F.v('화면 PCBA Type')}; Standard는 대안)",
        "D10": f"툴링 홀 부품 {F.v('툴링 홀(H) 부품')}개 (설계에 없음); 주문 화면 Tooling holes = {F.v('화면 Tooling holes')}",
        "D11": "주문 화면에서 사용자가 선택 (2장: PCB 5 / 실장 2)",
        "D12": f"J2 / J5 참고 부품 {F.v('참고 코드 J2')} (사용자가 구매); 대체 순서는 5장 표 2",
        "D13": f"{F.v('분압 저항')} (그대로)",
        "D14": f"주문하지 않음 (2장: Bake Components = {F.v('화면 Bake Components')}; 메모에 베이킹 문장 없음)",
        "D15": "가정 치수 사용 (spec/module_dimensions.json, 그림과 검사에 '가정' 표시)",
        "D16": f"주문 화면에서 사용자가 선택 (2장: {F.v('화면 Surface Finish')}, 녹색, 기본 적층, Via Covering {F.v('화면 Via Covering')}); "
               f"보드 파일 표기 '{F.v('보드 표면처리')}', job 파일 '{F.v('gbrjob 표면처리')}'",
        "D17": "여분 기판 3장은 기구 확인용 (주문 화면 수량으로 정해짐)",
    }
    for d in ctx["plan"]["user_report_ko"]["decisions"]:
        did = d.split(":")[0]
        A(f"| {did} | {d.split(':', 1)[1].strip()} | {built.get(did, '-')} |")
    A(f"| 추가 ({F.v('변경 날짜')} 사용자 결정) | ESP-12F (U7)를 JLC가 아니라 사용자가 직접 실장 | 두 BOM의 U7: {F.v('코드 U7')}; "
      f"두 zip 모두 U7 패드 {F.v('ESP 패드 수')}개에 페이스트 없음 |")
    A(f"| 추가 ({F.v('변경 날짜')} 사용자 결정) | 스루홀 커넥터 {F.v('스루홀 부품 수')}개를 JLC가 아니라 사용자가 사서 납땜 (JLC는 SMT만) | "
      f"사양 fit user {F.v('fit user')}개 (J1, U7, 커넥터 {F.v('스루홀 부품 수')}개); 두 BOM의 스루홀 줄 {F.v('main bom_lines_tht')} / {F.v('with_mcu bom_lines_tht')} |")
    A(f"| 추가 ({F.v('변경 날짜')} 사용자 결정) | 실패한 배선 규칙을 없애려고 기판 크기를 조정 (-y 변 탭) | 탭 {F.v('탭 위치')}, 외접 {F.v('외접 크기')} |")
    A(f"| 추가 ({F.v('변경 날짜')} 사용자 결정) | 프로그래밍 구멍 J26 / J22 / J19 이동 (\"{REDESIGN_REQUESTS[2]}\") | {F.v('옮긴 프로그래밍 구멍')} |")
    A("")
    A("## 10. 변경 범위 표 (rev 2c 대비, 부품마다 이유 분류)")
    A("")
    table = change_table(F, ctx)
    A(f"계획 3장의 표를 계획 파일에서 읽고, rev 3 부품 번호를 주문 BOM 파일과 대조했습니다 ({F.v('변경 행')}). 그대로인 부품은 표에서 뺐습니다. "
      "'누가 붙임'이 user인 줄의 rev 3 번호 칸은 '-' 입니다 (JLC BOM에 없음; 참고 부품 번호는 6장).")
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
    val = lambda k: F.data[k][0]  # noqa: E731
    purchase_list.write_ko(ctx["prow"], OUT / KO_NAME, {
        "date": val("변경 날짜"), "request": val("변경 요청 원문 2"), "root": R, "jlc_lines": val("main bom_lines"),
        "jlc_parts": val("main parts"), "joints": val("스루홀 납땜점"), "esp_pads": val("ESP 패드 수"),
        "quote_no_tht": val("견적 economic_no_tht 합계"), "quote_tht": val("견적 economic_tht_placed 합계"),
        "quote_label": "M, 주문 화면 2026-11-12, 재설계 전 76 x 70 mm 거버의 견적 - 지금 파일의 값은 NOT_VERIFIED",
        "snapshot_time": val("LCSC 기록 시각")})
    print(json.dumps({"README_ko.md": {"facts": len(readme_keys)}, "out/ORDER_ko.md": {"facts": len(order_keys)},
                      "out/" + KO_NAME: {"rows": len(ctx["prow"])},
                      "release_gate": F.data["출고 판정"][0]}, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
