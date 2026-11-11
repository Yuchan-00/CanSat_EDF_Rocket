"""Everything the user buys elsewhere: one table for out/off_board_purchase.csv (English) and for the Korean list
out/따로_살_부품_ko.md (plan amendments of 2026-11-12: the ESP-12F and the 12 through-hole connectors left the JLC BOM).

Invariant: quantities and part numbers are never typed here. Per-board quantities come from the spec (parts[].fit), the
MPN / LCSC reference code of every former BOM line from the plan (``bom_lines_now_bought_by_the_user``), pad and joint
counts from out/fit_checks.json. Stock and price come ONLY from spec/lcsc_snapshot_20261112.json (LCSC's public product
record, read once on 2026-11-12, label M at that time); a code without a record there is NOT_VERIFIED. Shop names
other than LCSC are suggestions and were not fetched. The fixed wording (what a part is for) is checked against the
spec: a row whose reference designators are not fitted by the user any more is an error.

Used by checks_exports.py (CSV) and docs_ko.py (Korean list). Deterministic for the same input files.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import fitsets

ROOT = Path(__file__).resolve().parent.parent
LCSC = ROOT / "spec" / "lcsc_snapshot_20261112.json"
KO_NAME = "따로_살_부품_ko.md"
BOARDS = 2            # assembled boards of the order (plan D11)

GROUPS = (
    ("a", "parts that were on the JLC BOM and are now bought and soldered by the user",
     "JLC BOM에 있다가 이번에 사용자가 직접 사서 납땜하게 된 부품"),
    ("b", "MCUs", "MCU"),
    ("c", "sensor / logger / GPS modules and their headers and leads", "센서·기록기·GPS 모듈과 그 헤더·리드선"),
    ("d", "battery, switches, buzzer and their leads", "배터리, 스위치, 부저와 그 리드선"),
    ("e", "programming headers and tools", "프로그래밍 헤더와 도구"),
    ("f", "mounting material", "장착 재료"),
)

#: fixed wording of the six THT lines (LCSC code -> texts); the designators are checked against the plan line
THT_TEXT = {
    "C22436147": {
        "item_ko": "1x6 직각(ㄱ자) 핀 소켓 2.54 mm (암)",
        "where": "J2: the BMP280 module's right-angle 6-pin header plugs in; J5: the OpenLog (with its soldered right-angle header) plugs in",
        "where_ko": "J2 (BMP280 모듈의 ㄱ자 6핀 헤더가 꽂힘), J5 (ㄱ자 헤더를 납땜한 OpenLog가 꽂힘)",
        "note_ko": "모듈이 기판과 나란히 눕도록 구멍이 옆을 보는 직각형이어야 합니다. 다른 제품을 쓰면 몸체 치수와 다리 길이가 다를 수 있습니다 (호환 NOT_VERIFIED)."},
    "C32713262": {
        "item_ko": "1x3 직각(ㄱ자) 핀헤더 2.54 mm (수)",
        "where": "J8-J11: vane servo leads (3-pin 2.54 mm servo plug: signal, +, -); J12: ESC lead",
        "where_ko": "J8~J11 (날개 서보 4개의 3핀 플러그: 신호, +, -), J12 (ESC 리드선)",
        "note_ko": "핀이 기판 변 바깥을 향하는 직각형. 같은 규격의 40핀 직각 스트립을 3핀씩 잘라 써도 되지만 다리 길이는 달라질 수 있습니다 (NOT_VERIFIED)."},
    "C5364642": {
        "item_ko": "Molex SPOX 2.50 mm 2핀 수직 헤더 (5267-02A)",
        "where": "J14: logic battery plug (the pack's Molex 5264-type 2-pin housing; mating and polarity NOT_VERIFIED - meter first)",
        "where_ko": "J14 (로직 배터리 플러그 - 팩의 Molex 5264형 2핀 하우징; 맞물림과 극성은 NOT_VERIFIED, 꽂기 전에 멀티미터로 확인)",
        "note_ko": "**거꾸로 납땜하면 배터리 극성이 뒤집힙니다.** 반드시 이 부품 번호(또는 같은 5267-02A)를 쓰고, 방향은 주문 안내서 7장 표를 따르십시오."},
    "C131337": {
        "item_ko": "JST PH 2.0 mm 2핀 수직 헤더 (B2B-PH-K-S)",
        "where": "J15: power-switch lead (JST PHR-2 housing); J17: buzzer lead (JST PHR-2 housing)",
        "where_ko": "J15 (전원 스위치 리드선, JST PHR-2 하우징), J17 (부저 리드선, JST PHR-2 하우징)",
        "note_ko": "스위치와 부저 리드선이 꽂히는 자리입니다."},
    "C32713261": {
        "item_ko": "1x2 직각(ㄱ자) 핀헤더 2.54 mm (수)",
        "where": "J16: arming-switch lead (2-pin 2.54 mm Dupont housing)",
        "where_ko": "J16 (아밍 스위치 리드선, 2핀 2.54 mm 듀폰 하우징)",
        "note_ko": "같은 규격의 직각 스트립을 2핀 잘라 써도 됩니다 (다리 길이 NOT_VERIFIED)."},
    "C32713263": {
        "item_ko": "1x4 직각(ㄱ자) 핀헤더 2.54 mm (수)",
        "where": "J18: power-sense lead (6S voltage sense: VIN, VTET, VBUF, ISGN; 4-pin 2.54 mm Dupont housing; no GND pin on purpose)",
        "where_ko": "J18 (전원 감지 리드선: VIN, VTET, VBUF, ISGN; 4핀 2.54 mm 듀폰 하우징. GND 핀은 일부러 없음)",
        "note_ko": "같은 규격의 직각 스트립을 4핀 잘라 써도 됩니다 (다리 길이 NOT_VERIFIED)."},
}


def load_lcsc() -> dict:
    if not LCSC.is_file():
        return {"records": {}, "label": "NOT_VERIFIED (no snapshot file)"}
    return json.loads(LCSC.read_text(encoding="utf-8"))


def price_for(rec: dict, need: int) -> dict | None:
    """What an order of at least ``need`` pieces costs at LCSC's record: buy = need rounded up to the minimum order
    quantity and the order multiple; price tier of that quantity. None without a usable record."""
    if not rec or not rec.get("ok") or not rec.get("price_tiers_usd"):
        return None
    moq = int(rec.get("min_order_qty") or 1)
    mult = int(rec.get("order_multiple") or 1)
    buy = max(need, moq)
    buy = int(math.ceil(buy / mult) * mult)
    unit = None
    for t in sorted(rec["price_tiers_usd"], key=lambda t: t["from_qty"]):
        if buy >= t["from_qty"]:
            unit = float(t["usd"])
    if unit is None:
        unit = float(sorted(rec["price_tiers_usd"], key=lambda t: t["from_qty"])[0]["usd"])
    return {"buy": buy, "unit_usd": unit, "sum_usd": round(buy * unit, 2), "moq": moq, "stock": rec.get("stock"),
            "enough": rec.get("stock") is not None and rec["stock"] >= buy}


def build(spec: dict, plan: dict, fit: dict) -> list[dict]:
    """The purchase rows, in group order. Every row: group, item, item_ko, mpn, lcsc, qty_board, qty_2, where,
    where_ko, channel, channel_ko, note_ko, source, stock_price (text), stock_price_ko, price (dict | None)."""
    snap = load_lcsc()
    recs = snap.get("records", {})
    fits = fitsets.fit_sets(spec["parts"])
    parts = {p["refdes"]: p for p in spec["parts"]}
    up = spec["user_parts"]
    usmd = fit.get("user_soldered_smd", {})
    utht = fit.get("user_soldered_tht", {}).get("per_ref", {})
    bought = plan.get("bom_lines_now_bought_by_the_user", [])
    rows: list[dict] = []

    def add(group: str, item: str, item_ko: str, qty_board, qty_2, where: str, where_ko: str, channel: str,
            channel_ko: str, note_ko: str, source: str, mpn: str = "", lcsc: str = "") -> None:
        rec = recs.get(lcsc) if lcsc else None
        pr = price_for(rec, int(qty_2)) if rec and str(qty_2).isdigit() else None
        if lcsc and rec and rec.get("ok"):
            if mpn and rec.get("mpn") and rec["mpn"] != mpn:
                raise RuntimeError(f"purchase list: LCSC record of {lcsc} is {rec['mpn']!r}, the row says {mpn!r}")
            t = rec["fetched_at"]
            sp = (f"LCSC {lcsc}: stock {rec['stock']}, minimum order {pr['moq']}, {pr['unit_usd']:.4f} USD each when "
                  f"{pr['buy']} are bought = {pr['sum_usd']:.2f} USD (M, LCSC record {t}; stock and price move; "
                  "shipping not included)")
            sp_ko = (f"LCSC {lcsc}: 재고 {rec['stock']}개, 최소 주문 {pr['moq']}개, {pr['buy']}개 살 때 개당 "
                     f"{pr['unit_usd']:.4f} USD = {pr['sum_usd']:.2f} USD (M, {t} 기준; 재고·가격은 움직임, 배송비 제외)")
        else:
            sp = "NOT_VERIFIED (not fetched)"
            sp_ko = "NOT_VERIFIED (확인하지 않음)"
        rows.append({"group": group, "item": item, "item_ko": item_ko, "mpn": mpn, "lcsc": lcsc, "qty_board": qty_board,
                     "qty_2": qty_2, "where": where, "where_ko": where_ko, "channel": channel, "channel_ko": channel_ko,
                     "note_ko": note_ko, "source": source, "stock_price": sp, "stock_price_ko": sp_ko, "price": pr})

    # ---- (a) former JLC BOM lines
    if sorted(usmd) != ["U7"] or parts["U7"]["fit"] != "user":
        raise RuntimeError(f"purchase list: user-soldered SMD parts {sorted(usmd)}; the ESP row is written for U7 only")
    for ln in bought:
        refs = ln["designators"]
        if any(parts[r]["fit"] != "user" for r in refs):
            raise RuntimeError(f"purchase list: {refs} are listed as bought by the user but the spec fit is not 'user'")
        n = len(refs)
        if n != ln["qty_per_board"]:
            raise RuntimeError(f"purchase list: line {ln['jlc_code']} quantity {ln['qty_per_board']} != {n} designators")
        src = (f"plan amendment 2026-11-12 ({ln['removed_by_amendment']}); spec fit user; former JLC BOM line "
               f"{ln['line']}")
        if ln["assembly_process"] == "SMT":
            pads = usmd[refs[0]]["pads"]
            add("a", "ESP-12F Wi-Fi module (Ai-Thinker ESP8266MOD, 4 MB flash, PCB antenna)",
                "ESP-12F Wi-Fi 모듈 (Ai-Thinker ESP8266MOD, 플래시 4 MB, 기판 안테나)", n, n * BOARDS,
                f"{'/'.join(refs)} footprint (user solders {pads} castellated pads; no solder paste on them)",
                f"{'/'.join(refs)} 자리 (옆면 홈 패드 {pads}개를 직접 납땜; 패드에 솔더 페이스트 없음)",
                f"LCSC {ln['jlc_code']} (reference part) or a domestic shop - needed with both file pairs",
                f"LCSC {ln['jlc_code']} (참고 부품) 또는 국내 쇼핑몰 - 두 주문 파일 묶음 모두에서 필요",
                "ESP-12E도 핀과 풋프린트가 같습니다 (계획은 재고 때문에 ESP-12F를 골랐음). 3.3 V 전용.",
                src, mpn=ln["mpn"], lcsc=ln["jlc_code"])
            continue
        t = THT_TEXT.get(ln["jlc_code"])
        if t is None or any(r not in utht for r in refs):
            raise RuntimeError(f"purchase list: no wording / no board data for the THT line {ln['jlc_code']} {refs}")
        joints = sum(utht[r]["joints"] for r in refs)
        # lead length under the board: the drawing value of this reference part against the foam-tape rule
        tl = fit["tht_leads"]
        lead = [x for x in tl["rows"] if x["ref"] in refs]
        if len(lead) != n or len({(x.get("under_board_nominal_mm"), x.get("under_board_max_mm")) for x in lead}) != 1:
            raise RuntimeError(f"purchase list: lead rows of {refs} are missing or differ within one part number")
        nom, mx, rule = lead[0].get("under_board_nominal_mm"), lead[0].get("under_board_max_mm"), tl["rule_mm"]
        holes = sum(x["plated_holes"] for x in lead)
        if nom is None:
            lead_ko = f" 뒷면 다리 길이는 도면에서 확인하지 못했습니다 (NOT_VERIFIED): 납땜 뒤 재 보고 {rule:g} mm를 넘으면 자릅니다."
        elif nom > rule:
            lead_ko = (f" 뒷면 다리가 도면값으로 {nom:g} mm 나와 {rule:g} mm 규칙을 넘습니다 (F): 납땜 뒤 다리 {holes}개를 반드시 자릅니다.")
        elif mx is not None and mx > rule:
            lead_ko = f" 뒷면 다리는 도면값 {nom:g} mm, 공차 끝 {mx:g} mm (F): 재 보고 {rule:g} mm를 넘으면 자릅니다."
        else:
            lead_ko = f" 뒷면 다리는 도면값 {nom:g} mm (F): 재 보고 {rule:g} mm를 넘으면 자릅니다."
        add("a", f"{ln['comment']} ({ln.get('brand') or ''} {ln['mpn']})".replace("( ", "("), t["item_ko"], n, n * BOARDS,
            f"{t['where']} - user solders {joints} joints per board",
            f"{t['where_ko']} - 보드당 납땜 {joints}곳",
            f"LCSC {ln['jlc_code']} (the part JLC would have fitted) or a domestic shop with the same part number",
            f"LCSC {ln['jlc_code']} (JLC가 실장하려던 바로 그 부품) 또는 같은 부품 번호를 파는 국내 쇼핑몰",
            t["note_ko"] + lead_ko, src, mpn=ln["mpn"], lcsc=ln["jlc_code"])
    tht_rows = [r for r in rows if r["lcsc"] in THT_TEXT]
    if sorted(x for r in bought if r["assembly_process"] == "THT" for x in r["designators"]) != sorted(utht) \
            or len(tht_rows) != len(THT_TEXT):
        raise RuntimeError("purchase list: THT rows differ from the user-soldered connectors of out/fit_checks.json")

    # ---- (b) MCUs
    mcus = fits["jlc_optional"]
    add("b", "ATmega128A-AU (TQFP-64)", "ATmega128A-AU (TQFP-64, 0.8 mm 피치)", len(mcus), len(mcus) * BOARDS,
        " ".join(mcus) + " footprints (user solders 64 pins each)", f"{mcus[0]}~{mcus[-1]} 자리 (칩당 64핀 직접 납땜)",
        "LCSC C6510 or a domestic distributor - not needed if the *_with_mcu files are ordered",
        "LCSC C6510 또는 국내 대리점 - with_mcu 파일 묶음으로 주문하면 사지 않음",
        "with_mcu 묶음(JLC가 MCU도 실장)을 고르면 필요 없습니다.", "REV3_PLAN 8 / 9; spec fit jlc_optional",
        mpn="ATMEGA128A-AU", lcsc="C6510")

    # ---- (c) modules, their headers and leads
    add("c", "GY-521 MPU-6050 module (SZH-EK007)", "GY-521 MPU-6050 모듈 (SZH-EK007)", 1, BOARDS,
        "J1 (solder the module's own 1x8 strip; support about 2.5 mm = strip plastic height, A)",
        "J1 (모듈에 딸린 1x8 핀을 구멍 8개에 납땜; 받침 약 2.5 mm, A)", "devicemart no=1247052", "디바이스마트 no=1247052",
        "U1을 먼저 납땜한 뒤에 답니다 (모듈이 U1 위에 걸침).", "spec user_parts.imu: " + up["imu"])
    add("c", "BMP280 module TYE-BMP29", "BMP280 모듈 TYE-BMP29", 1, BOARDS,
        "J2 socket (right-angle 6-pin header of the module)", "J2 소켓 (모듈의 ㄱ자 6핀 헤더)", "devicemart no=15442588",
        "디바이스마트 no=15442588", "받은 모듈의 헤더가 일자형이면 아래 ㄱ자 핀으로 바꿔 답니다 (헤더 종류 NOT_VERIFIED).",
        "spec user_parts.barometer")
    add("c", "right-angle 1x6 2.54 mm pin strip (only if the BMP280 comes with a straight header)",
        "ㄱ자 1x6 핀 스트립 2.54 mm (BMP280이 일자 헤더로 올 때만)", 1, BOARDS, "BMP280 module -> J2", "BMP280 모듈 → J2",
        "domestic shop (any RA 2.54 mm strip cut to 6) or LCSC C55629820", "국내 쇼핑몰 (ㄱ자 2.54 mm 스트립을 6핀 잘라 씀) 또는 LCSC C55629820",
        "OpenLog용과 같은 부품입니다.", "REV3_PLAN 8 (header type NOT_VERIFIED)", mpn="SH-PZ254R-1x6P", lcsc="C55629820")
    add("c", "NEO-7M GPS module SZH-NT07", "NEO-7M GPS 모듈 SZH-NT07", 1, BOARDS, "vehicle top cap; lead to J7",
        "기체 윗뚜껑; 리드선으로 J7에 연결", "devicemart no=10895400", "디바이스마트 no=10895400",
        "J7 커넥터(JST-GH, SMD)는 JLC가 실장합니다.", "spec user_parts.gps")
    add("c", "GPS lead JST-GH 1.25 mm 4-pin, both ends", "GPS 리드선 JST-GH 1.25 mm 4핀 (양쪽 끝)", 1, BOARDS, "J7 <-> NEO-7M",
        "J7 ↔ NEO-7M",
        "ready-made lead (domestic shop) or 2 x GHR-04V-S (LCSC C160418) + 8 x SSHL-002T-P0.2 (LCSC C189897) + crimp tool",
        "완제품 리드선 (국내 쇼핑몰), 또는 GHR-04V-S 하우징 2개 (LCSC C160418) + SSHL-002T-P0.2 단자 8개 (LCSC C189897) + 압착기",
        "GPS 쪽 핀 순서는 NEO-7M 보드의 인쇄를 따릅니다 (선 순서 A).", "REV3_PLAN 9")
    add("c", "SparkFun OpenLog DEV-13712", "SparkFun OpenLog DEV-13712 (기록기)", 1, BOARDS, "J5 socket", "J5 소켓",
        "SparkFun distributor / domestic shop", "SparkFun 대리점 / 국내 쇼핑몰",
        "아랫면 부품 높이를 재고 끼워 본 뒤 결정 (맞춤 NOT_VERIFIED).", "REV3_PLAN 8 (D5)")
    add("c", "microSD card", "microSD 카드", 1, BOARDS, "OpenLog", "OpenLog", "any shop", "아무 곳", "", "REV3_PLAN 9")
    add("c", "right-angle 1x6 2.54 mm pin header for the OpenLog", "OpenLog용 ㄱ자 1x6 핀헤더 2.54 mm", 1, BOARDS,
        "solder onto the OpenLog, plug into J5", "OpenLog에 납땜(6곳)한 뒤 J5에 꽂음", "domestic shop or LCSC C55629820",
        "국내 쇼핑몰 또는 LCSC C55629820", "", "REV3_PLAN 8 / 9", mpn="SH-PZ254R-1x6P", lcsc="C55629820")

    # ---- (d) battery, switches, buzzer and their leads
    add("d", "logic battery 1S Li-Po DTP105565P 3.7 V 4200 mAh", "로직 배터리 1S 리튬폴리머 DTP105565P 3.7 V 4200 mAh", 1, BOARDS,
        "J14 (Molex 5264-type plug; mating and polarity NOT_VERIFIED - meter first)",
        "J14 (Molex 5264형 플러그; 맞물림과 극성 NOT_VERIFIED - 먼저 멀티미터로 확인)", "devicemart no=16021596",
        "디바이스마트 no=16021596", "1S 전용. 2S / 3S 팩을 꽂으면 안 됩니다.", "spec user_parts.logic_battery")
    add("d", "power switch MTS-102 + JST-PH 2-pin lead", "전원 스위치 MTS-102 + JST-PH 2핀 리드선", 1, BOARDS, "J15", "J15",
        "switch: domestic shop; lead: ready-made PH 2.0 mm 2-pin or PHR-2 (LCSC C157955) + 2 x SPH-002T-P0.5S (LCSC C111515)",
        "스위치: 국내 쇼핑몰; 리드선: PH 2.0 mm 2핀 완제품, 또는 PHR-2 하우징 (LCSC C157955) + SPH-002T-P0.5S 단자 2개 (LCSC C111515)",
        "스위치는 2 A 이상 (A).", "REV3_PLAN 9")
    add("d", "arm switch + 2-pin 2.54 mm Dupont lead", "아밍 스위치 + 2핀 2.54 mm 듀폰 리드선", 1, BOARDS, "J16", "J16",
        "domestic shop", "국내 쇼핑몰", "", "REV3_PLAN 9")
    add("d", "buzzer + JST-PH 2-pin lead", "부저 (5 V 능동형) + JST-PH 2핀 리드선", 1, BOARDS, "J17", "J17",
        "domestic shop (PHR-2 C157955 + SPH-002T-P0.5S C111515 at LCSC)",
        "국내 쇼핑몰 (하우징 PHR-2 = LCSC C157955, 단자 SPH-002T-P0.5S = LCSC C111515)", "", "REV3_PLAN 9")

    # ---- (e) programming headers and tools
    add("e", "2x3 2.54 mm pin header (programming, option A)", "2x3 핀헤더 2.54 mm (프로그래밍용, A안)", 5, 5 * BOARDS,
        "J3, J19, J20, J21, J22 (bare holes in the tie band)", "J3, J19, J20, J21, J22 (케이블타이 띠 안의 빈 구멍)",
        "domestic shop or LCSC C65114 - or a 2x3 pogo-pin ISP adapter instead (contact NOT_VERIFIED)",
        "국내 쇼핑몰 또는 LCSC C65114 - 대신 2x3 포고핀 ISP 어댑터를 써도 됨 (접촉 NOT_VERIFIED)",
        "납땜했다면 기체에 붙이기 전에 떼거나 잘라 띠를 비워야 합니다. 포고핀 어댑터를 쓰면 사지 않습니다.",
        "REV3_PLAN 8 (D6 A)", lcsc="C65114")
    add("e", "1x2 2.54 mm pin header + shunt (reset hold)", "1x2 핀헤더 2.54 mm + 점퍼 (리셋 유지)", 3, 3 * BOARDS,
        "J24, J25, J26", "J24, J25, J26", "domestic shop or LCSC C55629809 + C100114",
        "국내 쇼핑몰 또는 LCSC C55629809 (헤더) + C100114 (점퍼)", "위와 같음.", "REV3_PLAN 8 (D6 A)", lcsc="C55629809")
    add("e", "USBasp programmer (target power jumper OPEN)", "USBasp 프로그래머 (타깃 전원 점퍼는 빼 둠)", 1, 1,
        "J19..J22 (ISP of the four MCUs)", "J19~J22 (MCU 4개 굽기)", "domestic shop", "국내 쇼핑몰", "보드 수와 무관하게 1개.",
        "REV3_PLAN 9 / 15")
    add("e", "USB-serial adapter, 3.3 V logic", "USB-시리얼 어댑터 (3.3 V 신호)", 1, 1, "J3 (ESP-12F flashing, 3.3 V only)",
        "J3 (ESP-12F 굽기, 3.3 V 전용)", "domestic shop", "국내 쇼핑몰", "5 V 신호 어댑터는 쓰면 안 됩니다.", "REV3_PLAN 9 / 15")
    add("e", "soldering tools: fine tip, flux, solder wire, flush cutter, multimeter (if not at hand)",
        "납땜 도구: 가는 인두 팁, 플럭스, 실납, 니퍼(다리 절단용), 멀티미터 (없을 때만)", "-", "1 set",
        "MCUs, ESP-12F, 12 connectors, lead cutting, checks before power", "MCU, ESP-12F, 커넥터 12개 납땜, 다리 자르기, 전원 넣기 전 확인",
        "domestic shop", "국내 쇼핑몰", "이미 있으면 사지 않습니다.", "out/ORDER_ko.md section 7 (work the user does)")

    # ---- (f) mounting material
    add("f", "3 mm foam tape", "3 mm 폼테이프", 1, BOARDS, "board underside (after the THT leads are flush-cut)",
        "기판 아랫면 (스루홀 다리를 자른 뒤)", "domestic shop", "국내 쇼핑몰", "", "REV3_PLAN 9 / 15")
    add("f", "cable tie", "케이블타이", 1, BOARDS, "across the tie band x -3..+3 mm", "케이블타이 띠 (x -3..+3 mm)를 가로질러",
        "domestic shop", "국내 쇼핑몰", "", "REV3_PLAN 9 / 15")
    return rows


CSV_HEADER = ["group", "item", "part number (MPN)", "LCSC code (reference)", "qty per board", "qty for 2 boards",
              "plugs into / used at", "purchase channel (suggestion)", "stock / price", "source"]


def csv_rows(rows: list[dict]) -> list[list]:
    name = {g: en for g, en, _ko in GROUPS}
    return [[f"({r['group']}) {name[r['group']]}", r["item"], r["mpn"], r["lcsc"], r["qty_board"], r["qty_2"], r["where"],
             r["channel"], r["stock_price"], r["source"]] for r in rows]


def group_a_cost(rows: list[dict]) -> dict | None:
    """Sum of the LCSC orders of group (a) for two boards at the snapshot prices (minimum order quantities respected)."""
    a = [r for r in rows if r["group"] == "a"]
    if not a or any(r["price"] is None for r in a):
        return None
    return {"sum_usd": round(sum(r["price"]["sum_usd"] for r in a), 2), "lines": len(a),
            "short": [r["lcsc"] for r in a if not r["price"]["enough"]],
            "tht_sum_usd": round(sum(r["price"]["sum_usd"] for r in a if r["lcsc"] in THT_TEXT), 2)}


def write_ko(rows: list[dict], path: Path, ctx: dict) -> None:
    """out/따로_살_부품_ko.md. ``ctx``: request (verbatim), date, root (board folder, posix), jlc_lines / jlc_parts
    (what JLC still fits), joints (THT joints per board), esp_pads, quote_no_tht / quote_tht (order-page totals, M),
    quote_label, snapshot_time."""
    cost = group_a_cost(rows)
    L: list[str] = []
    A = L.append
    A("# 따로 사야 하는 부품 목록 - VTOL 캐리어 PCB rev 3")
    A("")
    A(f"{ctx['date']} 사용자 요청 \"{ctx['request']}\" 에 따라 정리한 목록입니다. "
      "**JLCPCB 주문은 Economic PCBA로 SMT 부품만 실장합니다. 스루홀(구멍에 꽂는) 부품과 ESP-12F, MCU, 센서 모듈은 "
      "JLC가 달지 않으므로 아래 목록대로 따로 사서 직접 납땜하거나 꽂아야 합니다.** "
      "JLC는 PCBA 주문에 낱개 부품을 함께 보내주지 않습니다.")
    A("")
    A("**이 보드는 아직 만들어진 적이 없고, 아래 부품을 실제로 사서 구멍에 맞춰 본 적도 없습니다 (NOT_VERIFIED).** "
      "이 문서는 `scripts/docs_ko.py` 가 사양 파일에서 만든 것입니다 (손으로 고치지 말고 다시 생성).")
    A("")
    A("표시: M = 도구로 측정 (보드 파일, 또는 적힌 시각의 LCSC 기록), E = 추정, A = 가정, F = 문서 근거, NOT_VERIFIED = 근거 없음. "
      f"수량은 사양 파일의 부품 수에서 셌습니다 (M). '2장 분'은 이번 주문에서 실장하는 보드 {BOARDS}장 기준입니다.")
    A("")
    A("## 한눈에 보기")
    A("")
    A(f"- JLC가 실장해서 보내는 것: SMT 부품 {ctx['jlc_parts']}개 (BOM {ctx['jlc_lines']}줄). GPS 커넥터 J7(SMD)은 여기에 들어 있습니다 (M).")
    A(f"- 이번에 JLC BOM에서 빠져 사용자가 사게 된 것 ((a) 묶음): ESP-12F 1종 + 스루홀 커넥터 {len([r for r in rows if r['lcsc'] in THT_TEXT])}종 "
      f"(보드당 커넥터 {sum(r['qty_board'] for r in rows if r['lcsc'] in THT_TEXT)}개, 납땜 {ctx['joints']}곳; ESP-12F는 패드 {ctx['esp_pads']}개) (M).")
    if cost:
        short = f" 재고가 모자란 것: {', '.join(cost['short'])}." if cost["short"] else " 일곱 가지 모두 그 시각에 재고가 필요 수량보다 많았습니다."
        A(f"- (a) 묶음을 LCSC에서 최소 주문 수량대로 2장 분 사면 약 {cost['sum_usd']:.2f} USD입니다 "
          f"(그중 커넥터 6종 {cost['tht_sum_usd']:.2f} USD; E: {ctx['snapshot_time']} LCSC 기록의 단가 x 수량, 배송비·세금 제외).{short}")
    A(f"- 참고 ({ctx['quote_label']}): 스루홀 6줄을 JLC에 맡기면 주문 합계가 {ctx['quote_tht']} USD, 빼면 {ctx['quote_no_tht']} USD였습니다 (배송 제외, ESP-12F 미실장 기준).")
    A("- 사는 곳은 제안일 뿐이고, LCSC 말고는 재고·가격을 확인하지 않았습니다. 디바이스마트 번호는 rev 2c 때 정한 품목 번호입니다 (이번에 다시 열어 보지 않음).")
    A("")
    for g, _en, ko in GROUPS:
        sel = [r for r in rows if r["group"] == g]
        A(f"## ({g}) {ko}")
        A("")
        A("| 품목 | 부품 번호 (제조사 번호 / LCSC) | 보드당 | 2장 분 | 기판의 어디에 | 사는 곳 (제안) | 재고·가격 | 비고 |")
        A("|---|---|---|---|---|---|---|---|")
        for r in sel:
            pn = " / ".join(x for x in (r["mpn"], f"LCSC {r['lcsc']}" if r["lcsc"] else "") if x) or "-"
            A(f"| {r['item_ko']} | {pn} | {r['qty_board']} | {r['qty_2']} | {r['where_ko']} | {r['channel_ko']} | "
              f"{r['stock_price_ko']} | {r['note_ko'] or '-'} |")
        A("")
        if g == "a":
            A("- 이 일곱 가지는 계획 때 JLC BOM에 있던 바로 그 부품 번호입니다. 풋프린트(구멍 위치·지름)와 실크는 그 부품에 맞춰져 있고 바뀌지 않았습니다. "
              "다른 번호의 부품으로 바꾸면 맞는지 직접 확인해야 합니다.")
            A(f"- 납땜 순서, 커넥터마다 어느 쪽을 보게 꽂는지, 다리 자르기는 주문 안내서 `{ctx['root']}/out/ORDER_ko.md` 7장에 있습니다. "
              "**J14를 거꾸로 달면 배터리 극성이 뒤집힙니다.**")
            A("- 소켓 J2 / J5 (C22436147)를 구하기 어려우면 계획의 대체 순서는 C55629861, 그다음 C54876735입니다 "
              "(같은 풋프린트에 맞는지는 NOT_VERIFIED; C55629861은 다리가 더 길어 반드시 잘라야 함).")
            A("")
    A("## 이 목록에 없는 것")
    A("")
    A("- 서보, ESC, 6S 주 배터리와 그 배선은 기체 부품이라 이 목록에 넣지 않았습니다.")
    A("- 저항, 커패시터, IC 등 SMT 부품은 모두 JLC가 실장하므로 따로 사지 않습니다.")
    A("")
    A("## 숫자의 출처")
    A("")
    A("| 무엇 | 출처 파일 | 표시 |")
    A("|---|---|---|")
    A(f"| 보드당 수량, 누가 붙이는지 | `{ctx['root']}/spec/carrier_spec.json` parts[].fit | M (스크립트 집계) |")
    A(f"| (a) 묶음의 부품 번호와 LCSC 번호 | `{ctx['root']}/spec/rev3_plan.json` bom_lines_now_bought_by_the_user (2026-11-10의 JLC BOM 줄 그대로) | F |")
    A(f"| 납땜 곳 수, ESP 패드 수 | `{ctx['root']}/out/fit_checks.json` user_soldered_tht / user_soldered_smd (보드 파일에서 셈) | M |")
    A(f"| LCSC 재고·가격 | `{ctx['root']}/spec/lcsc_snapshot_20261112.json` (LCSC 공개 상품 기록을 읽기만 함, 로그인·장바구니 없음) | M (그 시각 기준) |")
    A(f"| JLC 주문 합계 비교 | `{ctx['root']}/spec/jlc_order_page_20261112.json` quotes_usd | {ctx['quote_label']} |")
    A(f"| 같은 내용의 CSV | `{ctx['root']}/out/off_board_purchase.csv` | - |")
    A("")
    path.write_text("\n".join(L), encoding="utf-8", newline="\n")
