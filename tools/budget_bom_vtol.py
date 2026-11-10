"""v0.9 CAD-2 budget of the VTOL configuration -> docs/budget/예산안_vtol.md (+ bom_vtol.json).

Invariant: rows and prices come from tools/budget_bom.py's researched rows (docs/budget/price_research_2026-10-13.json via
that module) or the tether report's cost table (docs/tether_power_proposal.json costs_krw); every new row without a
researched price is marked ASSUMED.  The vehicle masses are the CadQuery VTOL reports' totals (refused when they are not
from this params.json), PLA comes from the provisional VTOL print plan (cad/exports/print/provisional_v09_cadquery_vtol).
Totals are computed, never typed.

What changes against the rocket budget (tools/budget_bom.py): the four ATmega128 minimum-system boards, their AMS1117 /
level-shifter modules and the ilink / nlink / glink harnesses become ONE carrier PCB (fabrication + parts, ASSUMED: no
quote taken; pcb/vtol_carrier rev 2c: 4 layers, 4 chips and no 5th-chip footprint, the ESP-12E soldered on, no LoRa
nets); the 5 V logic UBEC and the user's LoRa modules go (2026-11-11): the carrier takes its 5 V from a separate 1S
logic pack through a Pololu S9V11F5S6CMA module switched by an MTS-102 toggle - three devicemart rows (budget_bom.DM); the GPS pod parts become the cap's GPS parts; the recovery rows (chute fabric, lines, ejection spring) go;
the release servo stays in the 5-servo set as a spare.  The tether / hybrid variants add the on-vehicle kit and list the
ground equipment kits of the tether report.

Usage: python tools/budget_bom_vtol.py [--out-dir DIR]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import budget_bom as bb  # noqa: E402  (the rocket budget's researched rows)

OUT_DIR = os.path.join(ROOT, "docs", "budget")
G = 9.80665
DROP = ("ATmega128 최소 시스템 보드", "I2C 레벨 시프터", "AMS1117-3.3", "ilink 배선", "nlink 배선", "glink 배선",
        "GPS 포드 부품", "립스톱 나일론", "낙하산 줄", "사출 스프링",
        # 2026-11-11 (carrier rev 2c): no UBEC (1S logic pack + Pololu module on the board), no LoRa nets
        "5 V 3 A UBEC", "LoRa 모듈 2개")
# the v0.9 CAD-2 guesses, made for a 2-layer 76 x 70 mm board; rev 2c is 4 layers (pcb/vtol_carrier) - NOT re-estimated,
# no quote taken (a 4-layer order costs more: ASSUMED low)
CARRIER_PCB_KRW_ASSUMED = 15000      # 5 pcs + shipping (JLCPCB class; no quote taken)
CARRIER_PARTS_KRW_ASSUMED = 45000    # 4 x ATmega128A TQFP-64 (~8,000 each) + crystals, regulators, BSS138, passives, headers
CAP_GPS_PARTS_KRW_ASSUMED = 3000     # FR4 copper-clad 40 x 40, JST-GH pair, 28 AWG 4-core 0.4 m, foam tape


def sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def vtol_mass(power: str, sha: str) -> float:
    sfx = bb.layout.config_suffix("vtol", power)
    path = os.path.join(ROOT, "cad", "exports", f"cadquery_report{sfx}.json")
    with open(path, "r", encoding="utf-8") as f:
        rep = json.load(f)
    if rep.get("params_sha256") != sha or rep.get("failed_components") or rep.get("power_source") != power:
        raise SystemExit(f"{path} is not a complete {power} VTOL report of this params.json: rebuild the twin")
    return float(rep["totals"]["mass_g"])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-dir", default=OUT_DIR)
    a = ap.parse_args(argv)
    sha = sha256(os.path.join(ROOT, "params.json"))
    L = bb.layout.build(config="vtol")
    with open(os.path.join(ROOT, "docs", "tether_power_proposal.json"), "r", encoding="utf-8") as f:
        tether = json.load(f)
    tcost = {i["item"]: i["krw"] for i in tether["costs_krw"]["items"]}
    rows = [r for r in bb.MUST if not any(r[1].startswith(d) or d in r[1] for d in DROP)]
    dropped = [r for r in bb.MUST if r not in rows]
    rows.append(("비행 컴퓨터", "VTOL 캐리어 PCB 제작 (rev 2c, 4층, 5장) (ASSUMED)", CARRIER_PCB_KRW_ASSUMED, 0, "",
                 "추정값 - 견적 안 받음 (v0.9 CAD-2의 2층 기판 추정값 그대로; 4층은 더 비쌀 수 있음). ATmega128A 4개"
                 "(NAV·FCU·COM·GNS; 5번째 칩 자리 없음), ESP-12E 납땜 자리, BMP280·GY-521·OpenLog·Pololu 5 V 모듈 헤더, "
                 "레벨 시프터·레귤레이터 온보드 (pcb/vtol_carrier). 흡입 격벽 위 FCU 자리에 폼 테이프 + 케이블 타이"))
    rows.append(("비행 컴퓨터", "캐리어 부품: ATmega128A TQFP-64 x4, 16 MHz 크리스털 x4, AMS1117, BSS138, 수동 소자, 헤더·커넥터 "
                 "(ASSUMED)", CARRIER_PARTS_KRW_ASSUMED, 0, "",
                 "추정값 - 가격 조사 안 함 (v0.9 CAD-2 추정값 그대로, rev 2c 부품표로 다시 계산하지 않음). 기존 보드 4개·레벨 "
                 "시프터 3개·AMS1117 3개·하네스 3개를 대신함. ESP-12E·BMP280·GY-521·OpenLog·GPS는 각자의 줄"))
    for key, name, note in (
            ("logic_battery", "로직 전원 1S 리튬폴리머 3.7 V 4200 mAh [DTP105565P]",
             "캐리어 J14. 6S 팩과 분리된 로직 전원 (configs.vtol.logic_power). 충전기 1S 설정 필요"),
            ("boost", "Pololu 5 V 승강압 모듈 S9V11F5S6CMA (#2870, 저전압 차단 조정)",
             "캐리어 J6. 1S 팩 -> 5 V 로직. 로켓 예산의 5 V UBEC를 대신함"),
            ("toggle", "로직 전원 토글 스위치 MTS-102 (2단 3P, 듀폰 하네스)", "캐리어 J15")):
        rows.append(("비행 컴퓨터", name, bb.DM[key][1], 0, bb.dm_url(key), bb.DM_NOTE + ". " + note))
    rows.append(("항법", f"GPS 캡 부품: FR4 동박 {L.p['top_cap']['gps']['ground_plane_side']:g} mm 접지판, JST-GH 1쌍, 28 AWG "
                 "4심 0.4 m, 폼 테이프 (ASSUMED)", CAP_GPS_PARTS_KRW_ASSUMED, 0, "", "추정값. GPS는 탑캡 위에 테이프로 고정"))
    ext = list(L.sl_ext_len)
    wire_m = math.ceil((sum(ext) + bb.SERVO_WIRE_SPARE_MM) / 1000.0)
    rows.insert(next(i for i, r in enumerate(rows) if r[0] == "배선") + 1,
                ("배선", f"베인 서보 연장선: 26 AWG 3심 서보선 {wire_m} m + JR 3핀 하우징 {len(ext)}세트 + 열수축 (ASSUMED)",
                 wire_m * bb.SERVO_WIRE_KRW_PER_M_ASSUMED + bb.SERVO_HOUSINGS_PINS_KRW_ASSUMED + bb.SERVO_HEATSHRINK_KRW_ASSUMED,
                 0, "", "추정값 (로켓 예산과 같은 단가)"))
    plan_path = os.path.join(ROOT, "cad", "exports", "print", "provisional_v09_cadquery_vtol", "print_plan_provisional.json")
    with open(plan_path, "r", encoding="utf-8") as f:
        plan = json.load(f)
    if (plan.get("source") or {}).get("params_sha256") != sha:
        raise SystemExit(f"{plan_path} is not from this params.json: re-slice (make_v09_print_files.py --config vtol)")
    pla_g = plan["totals"]["filament_g"]
    # livery 2026-11-04: one row per colour (white body, orange logo)
    pla_rows, pla_by_colour = bb.pla_colour_rows(plan, pla_g, "임시 VTOL 출력 계획 (CadQuery 트윈 STL 슬라이스)", "VTOL ")
    pla_rolls = sum(v["rolls"] for v in pla_by_colour.values())
    rows += pla_rows
    total = sum(p + (s or 0) for (_, _, p, s, _, _) in rows)
    vehicle_tether = [("tether_kit", "기체 쪽 유선 키트: 커패시터·TVS·브레이크 초퍼·하드포인트(아이볼트·보강 와셔·케이블 부트)",
                       tcost["vehicle kit: caps, TVS, brake chopper, hardpoint"], "유선전원 검토 costs_krw (ASSUMED)"),
                      ("xt90", "XT90-S 1쌍 (기체-테더)", tcost["XT90-S pairs x2"] // 2, "유선전원 검토 (ASSUMED, 2쌍 가격의 절반)")]
    vehicle_hybrid = vehicle_tether + [
        ("diode", "D1/D2 이상적 다이오드 보드 (LM74700-Q1 x2 + FET 6개 + PCB)", tcost["D1 / D2 ideal-diode board (LM74700-Q1 x2 + 6 FETs + PCB)"],
         "유선전원 검토 (ASSUMED)"),
        ("buffer", "버퍼 배터리 6S 650 mAh 120C (GNB)", tcost["buffer 6S 650 mAh 120C GNB (HV)"], "유선전원 검토 (ASSUMED)")]
    kits = tether["costs_krw"]["kits"]
    masses = {pw: vtol_mass(pw, sha) for pw in ("battery", "tether", "hybrid")}
    thrust_n = float(L.p["edf"]["thrust_nominal_N"])
    out = ["# 예산안 - VTOL 구성 (v0.9 CAD-2)", "",
           "로켓 예산(`예산안.md`)의 조사 가격을 그대로 쓰고, VTOL에서 빠지는 줄을 빼고 새 줄을 더했습니다. "
           "(ASSUMED) 표시는 조사하지 않은 추정가입니다.", "",
           f"**배터리 VTOL 합계: {total:,}원**", "",
           "기체 질량 (CadQuery 트윈, 출력 무게는 슬라이서 값): " + ", ".join(
               f"{k} {m:.1f} g (T/W {thrust_n / (m * 1e-3 * G):.2f} @ 미검증 공칭 {thrust_n:.1f} N)" for k, m in masses.items()), "",
           "| 분류 | 품목 | 가격 | 배송 | 비고 | 판매처 |", "|---|---|---|---|---|---|"]
    for g, n, p, s, u, note in rows:
        out.append(f"| {g} | {n} | {p:,} | {'?' if s is None else f'{s:,}'} | {note} | {f'[링크]({u})' if u else '-'} |")
    out += ["", "## 로켓 예산에서 빠진 줄", "", "| 분류 | 품목 | 가격 + 배송 |", "|---|---|---|"]
    for g, n, p, s, _, _ in dropped:
        out.append(f"| {g} | {n} | {p + (s or 0):,} |")
    t_add = sum(x[2] for x in vehicle_tether)
    h_add = sum(x[2] for x in vehicle_hybrid)
    out += ["", "## 유선(배터리 없음) / 하이브리드 변형 - 기체 쪽 추가 (배터리 줄은 그대로 둠: 시험용)", "",
            "| 품목 | 가격 | 출처 |", "|---|---|---|"]
    for _, n, p, src in vehicle_hybrid:
        out.append(f"| {n} | {p:,} | {src} |")
    out += ["", f"유선 변형 기체 쪽 {t_add:,}원, 하이브리드 기체 쪽 {h_add:,}원.", "",
            "## 지상 장비 묶음 (유선전원 검토 costs_krw.kits, 합계에 미포함)", "", "| 묶음 | 가격 | 내용 |", "|---|---|---|"]
    for k, v in kits.items():
        out.append(f"| {k} | {v['krw']:,} | {v['contents']} |")
    out += ["", "전원 장치(RSP-1500-24)는 추력 시험 뒤에 삽니다 (유선전원 검토 권고)."]
    os.makedirs(a.out_dir, exist_ok=True)
    with open(os.path.join(a.out_dir, "예산안_vtol.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    with open(os.path.join(a.out_dir, "bom_vtol.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump({"params_sha256": sha, "total_krw_battery_vtol": total, "rows": rows, "dropped_rows": dropped,
                   "tether_vehicle_add_krw": t_add, "hybrid_vehicle_add_krw": h_add, "vehicle_kits": vehicle_hybrid,
                   "devicemart_parts": {k: {"goods_no": v[0], "krw_incl_vat": v[1], "url": bb.dm_url(k)}
                                        for k, v in bb.DM.items()},
                   "devicemart_read": list(bb.DM_READ),
                   "ground_kits": kits, "pla_g": pla_g, "pla_rolls": pla_rolls, "pla_by_colour": pla_by_colour,
                   "vehicle_mass_g": masses,
                   "thrust_nominal_N_unverified": thrust_n}, f, ensure_ascii=False, indent=1)
    print(f"VTOL battery total {total:,} KRW (dropped {len(dropped)} rocket rows); tether +{t_add:,}, hybrid +{h_add:,} on the "
          f"vehicle; PLA {pla_g:.0f} g -> {pla_rolls} rolls; masses " + ", ".join(f"{k} {m:.1f} g" for k, m in masses.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
