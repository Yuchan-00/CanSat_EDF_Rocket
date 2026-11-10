"""Budget bill of materials -> docs/budget/예산안.md (+ bom.json).

Invariant enforced: every price is copied from docs/budget/price_research_2026-10-13.json
(listing URL, KRW incl. known shipping, read 2026-10-13) or marked ASSUMED; PLA cost comes
from the Bambu Studio print plan (cad/exports/print/print_plan.json); the vehicle mass is the
CadQuery report's total (refused when it is for another preset, lists failed components or
misses a SPEC s6 part) and the thrust-to-weight ratio is computed from that same mass and
``edf.thrust_nominal_N``, so the two numbers printed side by side agree; lengths and walls
come from ``cad/layout.py``.  Totals are computed, never typed.

2026-11-11 (control round, the parts actually bought): the barometer row is the BMP280 module TYE-BMP29, the GPS row the
NEO-7M module SZH-NT07 and the vehicle radio an ESP-12E (LoRa = the fallback, user-owned).  Their prices are the
devicemart goods pages recorded in pcb/vtol_carrier/spec/parts_sources.json (sale price incl. VAT, read 2026-11-05 and
again 2026-11-11 - ``DM``); devicemart shipping is counted once, on the M3 screw-kit row (one order ASSUMED).  The
version paragraphs below are history: they name the parts of their time (MS5611 GY-63, GY-NEO6MV2, LoRa).

v0.6 (two ATmega128A, ``flight_computer.architecture``): the ATmega128 minimum-system board row counts one board per
chip (FCU + COM: 1 + ``flight_computer.com`` present) at the same AliExpress listing and unit price, and its shipping
once PER BOARD - the price research gives 1,953 KRW for one board and says nothing about combined shipping, so the
conservative reading is used.  The ilink wiring (1 kOhm series resistor, 4-pin connector pair, signal wire) is one
small ASSUMED row.  The "likely owned" figure subtracts ONE board (and the screw kit), not every board.

v0.6.2: the AMS1117-3.3 row counts one module per 3.3 V rail (FCU sensors + the radio / logger box,
``electronics.radio_log``) at the same listing and 1-pc price (the 2-pc option was not priced: 2 x the 1-pc price), and
the vane-servo extension wiring is one plainly ASSUMED row (no matching item in the price research), its wire length
from ``Layout.sl_ext_len``.

v0.6.3 (three ATmega128A: NAV + FCU + COM, ``flight_computer.architecture``; user 2026-10-15): the board row counts
1 (FCU) + 1 per ``flight_computer.nav`` / ``flight_computer.com`` present - three boards at the same listing, unit price and
per-board shipping as before; "likely owned" still subtracts ONE board.  The nlink harness (NAV <-> FCU SPI: SS, SCK, MOSI,
MISO + GND, a connector pair - the FCU is programmed with it unplugged - and ~30 cm of wire) is its own small ASSUMED row
like the ilink row (no matching item in the price research), not folded into an existing wiring row.

v0.8 (GPS position hold, ``electronics.gps`` / ``payload_module.gps_pod``): a "항법" group with the GPS board (then the GY-NEO6MV2 at eleparts,
8,000 KRW excluding VAT from the v0.8 GPS research, +10 % VAT computed, shipping unknown) and one ASSUMED row for the pod's
other parts (FR4 ground plane, JST-GH pair, lead, M2 screws / standoffs, grommet); no level-shifter row (the NAV's
BSS138 shifter has two free channels); the mass the pod adds is the CadQuery report's ``extra.gps.added_total_g``.

v0.8.1 (four ATmega128A: + GNS, ``flight_computer.gns`` / ``electronics.gns``; architect 2026-10-21): the board row counts the
GNS board too (same listing, unit price and per-board shipping); the GPS now plugs into the GNS, so its level shifting needs
the GNS board's own 4-channel BSS138 module (the "I2C 레벨 시프터 4채널" listing, a third one) and a 3.3 V reference for its low
side (one more AMS1117-3.3 module - ASSUMED necessary: the GY-NEO6MV2 of v0.8 does not break out its 3.3 V; not checked for the
NEO-7M SZH-NT07 bought in 2026-11); the glink harness (NAV
USART1 <-> GNS USART0 + 5 V + GND, a connector pair, ~30 cm) is its own small ASSUMED row like the ilink / nlink rows.  The
mass the GNS board adds is the CadQuery report's ``extra.gns.added_total_g`` (board ASSUMED + screws + bosses + the GPS lead
change).

Usage: ``python tools/budget_bom.py [--out-dir DIR]`` (default docs/budget).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
OUT_DIR = os.path.join(ROOT, "docs", "budget")
CAD_REPORT = os.path.join(ROOT, "cad", "exports", "cadquery_report.json")
for _p in (os.path.join(ROOT, "cad"), os.path.join(ROOT, "sizing")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import layout  # noqa: E402
import structure_estimate as se  # noqa: E402

G = 9.80665                      # m/s^2 (as sizing/sizing.py)
FULL_PRESET = "KAIST_90mm_12S"   # the original full design the budget table compares against

# ATmega128 minimum-system board, per board (price_research_2026-10-13.json: AliExpress listing 1005005368023927,
# price_krw 8735, shipping_krw 1953 for ONE board; no combined-shipping information -> shipping counted per board)
ATMEGA_BOARD_URL = "https://ko.aliexpress.com/item/1005005368023927.html"
ATMEGA_BOARD_KRW, ATMEGA_BOARD_SHIP_KRW = 8735, 1953
_P = layout.load_params()
HAS_NAV = bool(_P["flight_computer"].get("nav"))     # v0.6.3: the NAV ATmega128A (sensors + attitude estimate)
HAS_COM = bool(_P["flight_computer"].get("com"))     # v0.6: the COM ATmega128A (vehicle radio)
HAS_GNS = bool(_P["flight_computer"].get("gns")) and bool(_P["electronics"].get("gns"))   # v0.8.1: GPS + position hold
ATMEGA_BOARD_QTY = 1 + int(HAS_NAV) + int(HAS_COM) + int(HAS_GNS)  # FCU (+ NAV) (+ COM) (+ GNS), one chip each
ATMEGA_BOARD_ROLES = "+".join(["NAV"] * HAS_NAV + ["FCU"] + ["COM"] * HAS_COM + ["GNS"] * HAS_GNS)
ATMEGA_BOARD_ITEM = (f"ATmega128 최소 시스템 보드 x{ATMEGA_BOARD_QTY}"
                     + (f" ({ATMEGA_BOARD_ROLES.replace('+', ' + ')})" if ATMEGA_BOARD_QTY > 1 else ""))
NLINK_HARNESS_KRW_ASSUMED = 1500   # v0.6.3: 5-pin connector pair + ~30 cm signal wire (as the ilink row, no resistor)
# v0.6.2: AMS1117-3.3 module, one per 3.3 V rail - the sensor level shifter (v0.6.3: on the NAV) and (electronics.radio_log) the radio
# box's radio module (ESP-12E since 2026-11-06, LoRa the fallback) + level shifter.  Same AliExpress listing and unit price as before (price_research_2026-10-13.json:
# 1,401 KRW for ONE piece, free shipping; the listing's 2-pc option was not priced -> 2 x the 1-pc price, an upper bound)
AMS1117_URL = "https://ko.aliexpress.com/item/1005009310410637.html"
AMS1117_KRW = 1401
AMS1117_QTY = 1 + (1 if _P["electronics"].get("radio_log") else 0) + (1 if HAS_GNS else 0)
_SENSOR_BOARD = "NAV" if HAS_NAV else "FCU"         # the board that carries the I2C sensors
AMS1117_ITEM = "AMS1117-3.3 모듈" + (
    f" x{AMS1117_QTY} ({_SENSOR_BOARD} 센서용 + 무선·로거 상자용" + (" + GNS GPS 레벨 시프터용" if HAS_GNS else "") + ")"
    if AMS1117_QTY >= 2 else "")
GLINK_HARNESS_KRW_ASSUMED = 1500   # v0.8.1: 4-pin connector pair + ~30 cm signal wire (as the ilink / nlink rows)
# v0.6.2: vane-servo extension wiring (servo-lead session, SPEC 3.8) - no matching item in the price research (it holds
# only 12-24 AWG two-wire silicone sets), so the whole row is a plainly ASSUMED price: 26 AWG 3-core flat silicone servo
# wire per metre, 4 JR/Dupont 3-pin female housings with crimp pins, heat-shrink.  Length = the four extensions
# Layout.sl_ext_len + SERVO_WIRE_SPARE_MM, rounded up to whole metres.
SERVO_WIRE_KRW_PER_M_ASSUMED = 1500
SERVO_HOUSINGS_PINS_KRW_ASSUMED = 1500       # 4 x JR/Dupont 3-pin female housing + 12 crimp pins
SERVO_HEATSHRINK_KRW_ASSUMED = 1000
SERVO_WIRE_SPARE_MM = 300.0                  # handling / re-crimp spare on top of the cut lengths (ASSUMED)
# 2026-11 parts bought at devicemart: goods number and sale price incl. VAT (pcb/vtol_carrier/spec/parts_sources.json,
# fetched 2026-11-05; read again 2026-11-11, unchanged).  Shipping: one devicemart order ASSUMED, already counted on the
# M3 screw-kit row.  logic_battery / boost / toggle are VTOL-carrier parts (tools/budget_bom_vtol.py).
DM = {"bmp280": ("15442588", 2750), "neo7m": ("10895400", 9460), "esp12e": ("1378118", 2640),
      "logic_battery": ("16021596", 18150), "boost": ("15855364", 19360), "toggle": ("13029107", 1100)}
DM_READ = ("2026-11-05", "2026-11-11")
DM_NOTE = ("디바이스마트 판매가(부가세 포함), 2026-11-05 조회·2026-11-11 다시 확인. 배송비는 M3 볼트 키트 줄에 1회 "
           "(같은 주문 가정 - ASSUMED)")


def dm_url(key: str) -> str:
    """devicemart goods page of a ``DM`` part."""
    return f"https://www.devicemart.co.kr/goods/view?no={DM[key][0]}"


# v0.8 GPS (user 2026-10-21: "a standard Arduino GPS"; electronics.gps).  2026-11: the part bought is the NEO-7M module
# SZH-NT07 at devicemart (DM["neo7m"], incl. VAT; the excl.-VAT figure is computed from it).  v0.8 .. v0.9 priced the
# GY-NEO6MV2 at eleparts (8,000 KRW excluding VAT, scratchpad v08/report_gps-research.json).  The pod's other parts (FR4 ground plane, connector pair, lead, M2 screws /
# standoffs, grommet) are one plainly ASSUMED row.  Level shifting needs no part: the NAV's 4-channel BSS138 shifter has
# two channels the I2C bus leaves free (electronics.gps.interface; verify on the board).
HAS_GPS = bool(_P["electronics"].get("gps")) and bool(_P["payload_module"].get("gps_pod"))
GPS_BOARD_URL = dm_url("neo7m")
GPS_BOARD_KRW_INCL_VAT = DM["neo7m"][1]
GPS_BOARD_KRW_EXCL_VAT = round(GPS_BOARD_KRW_INCL_VAT / 1.10)
GPS_POD_PARTS_KRW_ASSUMED = 4000
GPS_GP_SIDE = float(_P["payload_module"]["gps_pod"]["ground_plane"]["side"]) if HAS_GPS else 0.0
# v0.9 (decision 1, vane_drive.mode 'direct'): no pushrods / clevises / servo arms (they were never rows of their own -
# the stock MG90S accessories); new: the M2 screws of the servo covers, the coupler flanges and the motor-lead groove cover
# plus the servo float's foam pad - one plainly ASSUMED row.  Counts from params (vane_drive.cover.screw: 2 per cover;
# vane_drive.coupler.note: 2 x M2 x 4 per flange; the groove cover's pads: 2 sides x len(pad_zs) screws).
_LAY0 = layout.build()
HAS_DIRECT_DRIVE = bool(getattr(_LAY0, "direct_drive", False))
M2_WIRE_COVER_QTY = 2 * len(getattr(_LAY0, "wire_pad_zs", [])) if getattr(_LAY0, "wire_ch_open", False) else 0
M2_SELF_TAP_QTY = 2 * len(_LAY0.fin_thetas) + M2_WIRE_COVER_QTY if HAS_DIRECT_DRIVE else 0
# v0.9 fix round: no coupler screws any more (the coupler is a socket on the cut horn, locked to the shaft by one M2 set
# screw); new hardware: 4 M2 x 3 set screws, 4 brass bushings (6 x 0.45 mm tube), EVA foam, cable ties for the motor-lead
# tie anchors - one more plainly ASSUMED row
HAS_SET_SCREW = HAS_DIRECT_DRIVE and hasattr(_LAY0, "dd_set_screw_u")
M2_SHORT_QTY = 0 if HAS_SET_SCREW else (2 * len(_LAY0.fin_thetas) if HAS_DIRECT_DRIVE else 0)
M2_KIT_KRW_ASSUMED = 3000
VANE_FIX_HW_KRW_ASSUMED = 6000 if HAS_SET_SCREW else 0      # set screws + 0.9 mm hex key, brass tube 6 mm x 300 mm, EVA foam
TIE_ANCHOR_COUNT = len(getattr(_LAY0, "ml_anchor_z", []) or [])
# v0.9 (decision 3, params motor_leads): the motor leads carry their bullets through the open groove to the avionics bay.
# The extension wire is priced at the researched RCbank 12 AWG silicone metre of the XT60H row (6,000 KRW for 2 m); the
# length is Layout.ml_lead_len x count, rounded up to whole metres; whether an extension is needed at all depends on the
# EDF's stock lead length (not measured - ASSUMED needed).
HAS_MOTOR_LEADS = bool(getattr(_LAY0, "has_motor_leads", False))
_ML = _P.get("motor_leads") or {}
ML_COUNT = int(_ML.get("count", 3))
ML_AWG = _ML.get("awg", 12)
ML_BULLET_D = float((_ML.get("bullet") or {}).get("d_ASSUMED", 8.0))
ML_BULLET_NAME = "5.5 mm"                     # params motor_leads note: 5.5 mm bullets (the 8 mm is their sleeve OD, ASSUMED)
ML_LEAD_LEN = float(getattr(_LAY0, "ml_lead_len", 0.0))
ML_WIRE_M = math.ceil(ML_COUNT * ML_LEAD_LEN / 1000.0) if HAS_MOTOR_LEADS else 0
ML_GROOVE_W = float(getattr(_LAY0, "wire_ch_w", 0.0))
WIRE_12AWG_KRW_PER_M = 6000 // 2

# (group, item, price_krw, shipping_krw or None if unknown, url, note)
MUST = [
    ("추진", "EDF Hobbyocean 90mm 12엽 6S 1450KV", 83400, 0, "https://ko.aliexpress.com/item/1005007300026404.html",
     "추력표 없음 - Powerfun 90mm 6S 1450KV 표(3.09 kg @ 22.2 V / 102 A)를 대용으로 사용"),
    ("추진", "ESC Hobbywing SkyWalker 100A V2 (5 V/7 A BEC)", 51300, 0, "https://ko.aliexpress.com/item/1005012523818954.html",
     "100 A 연속 / 120 A 순간. BEC로 서보 5개 구동"),
    ("추진", "LiPo CNHL G+Plus 6S 2200 mAh 70C XT60", 40207, 9372,
     "https://chinahobbyline.com/products/cnhl-gplus-series-2200mah-22-2v-6s-70c-lipo-battery-with-xt60-plug",
     "349 g, 109 x 35 x 51 mm. 해외 배송 15-20 영업일"),
    ("구동", "MG90S 서보 5개 세트", 13932, 0, "https://ko.aliexpress.com/item/1005008490187744.html",
     "베인 4 + 노즈 분리 1" + ((". v0.9: 베인 서보는 핀 뿌리 페어링 안에서 베인 축을 직접 돌린다 (동봉 2암 혼을 8.8 mm로 잘라 "
                               "인쇄 커플러의 홈에 끼움; 푸시로드·클레비스·암 없음)" if HAS_SET_SCREW else
                               ". v0.9: 베인 서보는 핀 뿌리 페어링 안에서 베인 축을 직접 돌린다 (동봉 혼을 원판으로 잘라 인쇄 "
                               "커플러에 고정; 푸시로드·클레비스·암 없음)") if HAS_DIRECT_DRIVE else "")),
    ("비행 컴퓨터", ATMEGA_BOARD_ITEM, ATMEGA_BOARD_QTY * ATMEGA_BOARD_KRW, ATMEGA_BOARD_QTY * ATMEGA_BOARD_SHIP_KRW,
     ATMEGA_BOARD_URL,
     f"개당 {ATMEGA_BOARD_KRW:,}원 + 배송 {ATMEGA_BOARD_SHIP_KRW:,}원. "
     + (("NAV(센서·자세 추정) 1개 + FCU(감시·제어·PWM·로그) 1개 + COM(무선 통신) 1개 + GNS(GPS·위치 유지) 1개"
          if ATMEGA_BOARD_QTY == 4 else
          "NAV(센서·자세 추정) 1개 + FCU(감시·제어·PWM·로그) 1개 + COM(무선 통신) 1개" if ATMEGA_BOARD_QTY == 3 else
          "FCU(센서·제어·감시) 1개 + COM(무선 통신) 1개")
        + ", 같은 판매 페이지. 배송비는 보드마다 따로 계산 (가격 조사에 묶음 배송 정보 없음 - 보수적). "
        if ATMEGA_BOARD_QTY > 1 else "")
     + "크리스털이 16 MHz인지 확인 (8 MHz로 온 사례 있음)"),
    ("비행 컴퓨터", "MPU-6050 GY-521", 2572, 0, "https://ko.aliexpress.com/item/1005010057794277.html", ""),
    ("비행 컴퓨터", "BMP280 기압계 모듈 [TYE-BMP29]", DM["bmp280"][1], 0, dm_url("bmp280"),
     "I2C 0x77, NAV 기본 기압계 드라이버 (flight_computer.nav.baro_sensor). " + DM_NOTE + ". v0.9까지의 MS5611 GY-63"
     "(4,780원 + 배송 1,973원, 2026-10-13 조사)은 펌웨어 빌드 옵션(nav_ms5611)으로만 남고 사지 않음"),
    ("비행 컴퓨터", "I2C 레벨 시프터 4채널", 344, 1417, "https://ko.aliexpress.com/item/32621797482.html", ""),
    ("비행 컴퓨터", AMS1117_ITEM, AMS1117_QTY * AMS1117_KRW, 0, AMS1117_URL,
     (f"개당 {AMS1117_KRW:,}원, 무료 배송, 같은 판매 페이지 (묶음 가격은 조사 안 함 - 1개 가격 x{AMS1117_QTY}). "
      f"1개는 {_SENSOR_BOARD} 센서 레벨 시프터 3.3 V 쪽, 1개는 페이로드 격벽 아래 무선·로거 상자의 무선 모듈(ESP-12E, 예비 LoRa)과 두 번째 레벨 "
      "시프터용 (800 mA급; ESP 송신 피크 약 400 mA - ASSUMED, LoRa 송신 약 120 mA)"
      + (". v0.8.1: 1개는 GNS 보드의 GPS 레벨 시프터 3.3 V 쪽 (ASSUMED 필요 - v0.8의 GY-NEO6MV2는 3.3 V 핀을 내놓지 않았음; "
         "NEO-7M SZH-NT07은 확인 안 함 - 보드에서 확인)" if HAS_GNS else "") if AMS1117_QTY >= 2 else
      "레벨 시프터 3.3 V 쪽. SPI형(SX127x) LoRa 모듈 전원도 여기서 (800 mA급, 송신 약 120 mA)")),
    ("비행 컴퓨터", "OpenLog 호환 로거", 3980, 0, "https://ko.aliexpress.com/item/1005006370090065.html", "microSD 카드는 보유 가정"),
    ("비행 컴퓨터", "5 V 3 A UBEC (2-6S) 로직 전원", 2256, 1940, "https://ko.aliexpress.com/item/4001219424525.html",
     "MCU를 서보 전원과 분리 (서보 부하로 인한 브라운아웃 리셋 방지)"),
    *([("항법", "GPS 모듈 NEO-7M [SZH-NT07] (u-blox NEO-7M)", GPS_BOARD_KRW_INCL_VAT, 0, GPS_BOARD_URL,
        f"{DM_NOTE} (부가세 별도로 환산하면 {GPS_BOARD_KRW_EXCL_VAT:,}원). v0.8-v0.9 예산은 GY-NEO6MV2(NEO-6M, eleparts "
        "8,000원 부가세 별도)였음. 보드·안테나 치수와 무게는 다시 재지 않음 (ASSUMED: GY-NEO6MV2와 같은 급). "
        + ("v0.8.1: GNS 보드 USART1 (PD2/PD3) 38400 bps, 레벨 변환은 GNS 보드의 4채널 BSS138 시프터 (아래 줄). "
           if HAS_GNS else
           "NAV USART1 (PD2/PD3) 38400 bps. 레벨 변환은 NAV의 기존 4채널 BSS138 시프터에서 I2C가 쓰지 않는 2채널을 사용 "
           "(추가 부품 없음 - 보드에서 확인). ")
        + "업그레이드: NEO-M8N (정품 모듈 약 55,364원 부가세 별도)"),
       ("항법", f"GPS 포드 부품: 단면 FR4 동박 기판 {GPS_GP_SIDE:g} x {GPS_GP_SIDE:g} x 0.8 mm (접지판), JST-GH 4핀 커넥터 1쌍, "
        "28 AWG 4심 선 0.5 m, M2 나사 5개 + 나일론 M2 스페이서 4개, 고무 그로밋 (ASSUMED)", GPS_POD_PARTS_KRW_ASSUMED, 0, "",
        "추정값 - 가격 조사 안 함. 포드와 뚜껑은 PLA로 출력 (아래 PLA 줄에 포함)")]
      if HAS_GPS else []),
    *([("항법", "I2C 레벨 시프터 4채널 (GNS 보드의 GPS 신호선용 세 번째)", 344, 1417, "https://ko.aliexpress.com/item/32621797482.html",
        "v0.8.1: GPS TX -> GNS PD2/RXD1, GNS PD3/TXD1 -> GPS RX (GPS RX는 5 V 비허용). 2채널 사용, 2채널 여유. 저전압 쪽 3.3 V는 "
        "AMS1117 모듈에서"),
       ("항법", "glink 배선: 4핀 커넥터 1쌍 + 신호선 30 cm (ASSUMED)", GLINK_HARNESS_KRW_ASSUMED, 0, "",
        "추정값 - 가격 조사 안 함. NAV USART1 <-> GNS USART0 (TX, RX) + 5 V + GND, 250 kbaud. 트레이 위를 넘어 NAV에서 "
        "-x 벽의 GNS까지. GNS를 프로그래밍할 때는 이 커넥터를 뽑는다 (RXD0/TXD0가 ISP와 공유)")]
      if HAS_GNS else []),
    ("통신", "ESP-12E Wi-Fi 모듈 (ESP8266, 기체 무선)", DM["esp12e"][1], 0, dm_url("esp12e"),
     DM_NOTE + ". 기체 무선 기본값 (flight_computer.link.vehicle_radio = esp_bridge, firmware/esp_bridge). 로켓은 "
     "통신·로거 상자에 손배선 (firmware/README.md 0.10.4 - 거리 시험과 상자 맞춤은 사용자 결정 대기), VTOL은 캐리어 rev 2c의 "
     "U7 자리 (JLC 조립이면 LCSC C89297, 손납땜이면 이 모듈). 부팅 핀 저항·커패시터는 '분압 저항·커패시터' 줄 몫 (ASSUMED). "
     "지상 쪽 ESP는 아래 지상 표 (합계에 미포함)"),
    ("통신", "LoRa 모듈 2개 (예비 링크: 기체용, 지상용)", 0, 0, "",
     "사용자 보유. Wi-Fi가 지상 거리 시험(1 km)을 통과하지 못하면 돌아갈 예비 링크 (vehicle_radio). 지상 쪽은 USB-UART 변환기 "
     "또는 SX127x용 아두이노 브리지 필요"),
    ("통신", "I2C 레벨 시프터 4채널 (무선 모듈 신호선용 두 번째)", 344, 1417, "https://ko.aliexpress.com/item/32621797482.html",
     "ESP-12E: COM TX·RST -> 모듈, 모듈 TX -> COM, OpenLog 로그선 (4채널, firmware/README.md 0.10.4). 예비 LoRa: MCU -> 모듈 "
     "방향 신호를 3.3 V로"),
    ("비행 컴퓨터", "분압 저항·커패시터 (ASSUMED)", 1000, 0, "", "추정값"),
    *([("비행 컴퓨터", "ilink 배선: 1 kΩ 직렬 저항 + 4핀 커넥터 1쌍 + 신호선 30 cm (ASSUMED)", 1500, 0, "",
        "추정값 - 가격 조사 안 함. FCU USART1 <-> COM USART0 (TX, RX, GND, 5 V), FCU TX 쪽에 1 kΩ")]
      if HAS_COM else []),
    *([("비행 컴퓨터", "nlink 배선: 5핀 커넥터 1쌍 + 신호선 30 cm (ASSUMED)", NLINK_HARNESS_KRW_ASSUMED, 0, "",
        "추정값 - 가격 조사 안 함. NAV(SPI 마스터) <-> FCU(SPI 슬레이브) PB0-PB3: SS, SCK, MOSI, MISO + GND. "
        "트레이의 NAV에서 흡입 격벽 위 FCU까지. FCU를 프로그래밍할 때는 이 커넥터를 뽑는다 (SCK가 ISP와 공유)")]
      if HAS_NAV else []),
    ("배선", "XT60H 커넥터 + 12 AWG 실리콘 전선 1 m x2", 1800 + 6000, 3500,
     "https://www.rcbank.co.kr/shop/goods/goods_view.php?goodsno=4825", "RCbank, 한 번에 주문 시 배송비 1회"),
    *([("배선", f"{ML_BULLET_NAME} 총알(불릿) 커넥터 {ML_COUNT}쌍 + 열수축 튜브 (ASSUMED)", 5000, 0, "",
        f"추정값 - 가격 조사 안 함. v0.9: 모터 선 {ML_COUNT}가닥은 총알 커넥터를 단 채 추진 모듈 슬롯 -> 흡입 모듈의 열린 홈"
        f"(폭 {ML_GROOVE_W:g} mm, 인쇄 덮개) -> 격벽 구멍을 지나 항공전자 칸에서 ESC 선과 잇는다 (커넥터 외경 "
        f"{ML_BULLET_D:g} mm ASSUMED가 한 줄로 지나감, C29). 모터 쪽 {ML_COUNT}개 + ESC 쪽 {ML_COUNT}개")]
      if HAS_MOTOR_LEADS else
      [("배선", "4 mm 총알(불릿) 커넥터 3쌍 이상 + 열수축 튜브 (ASSUMED)", 5000, 0, "",
        "추정값 - 가격 조사 안 함. EDF 모터 선은 커넥터 없이 피복만 벗긴 채 6 mm 배선 통로로 통과시킨 뒤 "
        "커넥터를 납땜하므로 (SPEC 5.1) 모터 쪽 3개를 새로 단다. 연장선을 쓰면 6쌍")]),
    *([("배선", f"모터 연장선: {ML_AWG} AWG 실리콘 전선 {ML_WIRE_M} m (ASSUMED 필요)", ML_WIRE_M * WIRE_12AWG_KRW_PER_M, 0,
        "https://www.rcbank.co.kr/shop/goods/goods_view.php?goodsno=4825",
        f"단가는 위 XT60H 줄의 같은 판매처 12 AWG 1 m 가격 ({WIRE_12AWG_KRW_PER_M:,}원/m, 같은 주문이면 배송비 없음). "
        f"cad/layout.py 모터 선 경로 {ML_LEAD_LEN:.0f} mm x {ML_COUNT}가닥 -> {ML_WIRE_M} m. EDF 기본 선이 항공전자 칸까지 "
        "닿으면 필요 없음 (기본 선 길이 미확인 - ASSUMED)")] if HAS_MOTOR_LEADS else []),
    *([("구조", f"M2 나사: M2 x 8 셀프태핑 {M2_SELF_TAP_QTY}개" + (f" + M2 x 4 {M2_SHORT_QTY}개" if M2_SHORT_QTY else "")
        + " (ASSUMED)",
        M2_KIT_KRW_ASSUMED, 0, "",
        f"추정값 - 가격 조사 안 함. v0.9 직결 베인: 서보 덮개 4개 x 2 + 모터 선 홈 덮개 {M2_WIRE_COVER_QTY}개 (셀프태핑)"
        + ("" if HAS_SET_SCREW else ", 커플러 플랜지 -> 서보 혼 4 x 2 (또는 접착)"))]
      if HAS_DIRECT_DRIVE else []),
    *([("구조", f"베인 고정 부품: M2 x 3 무두볼트(컵 포인트) 4개 + 0.9 mm 육각 렌치, 황동 파이프 6 x 0.45 mm 30 cm (부싱 4개), "
                f"EVA 폼 3 mm, 케이블 타이 2.5 mm {TIE_ANCHOR_COUNT}개 이상 (ASSUMED)", VANE_FIX_HW_KRW_ASSUMED, 0, "",
        "추정값 - 가격 조사 안 함. v0.9 수정: 커플러를 베인 축 D컷에 무두볼트로 고정(축 빠짐·D컷 유격 제거), 힌지 보스에 황동 "
        "부싱(유격 0.1 mm), 서보와 덮개 사이 폼(흔들림 제거), 흡입부 지주의 모터 선 고정점에 케이블 타이")]
      if HAS_SET_SCREW else []),
    ("구조", "M3 볼트·너트 키트 340개", 9680, 2700, "https://www.devicemart.co.kr/goods/view?no=1385067",
     "5-20 mm. 모듈 결합 (EDF 탭 고정 나사는 2026-11 키웨이 시계 장착으로 없어짐)"),
    ("회수", "립스톱 나일론 1 x 1.5 m x2 (1.4 m 낙하산 자작)", 2 * 3151, 0,
     "https://ko.aliexpress.com/item/1005010168641968.html", "가장 싼 색상 기준 (색상별 3,151-7,602원)"),
    ("회수", "낙하산 줄·쇼크 코드·스위블 (ASSUMED)", 3000, 0, "", "추정값"),
    ("회수", "사출 스프링 OD 40 (ASSUMED)", 3000, 0, "", "추정값"),
]
# items the builder likely owns: row name -> KRW owned (None = the whole row).  v0.6: ONE ATmega128 board (and its
# shipping) of the FCU + COM pair, not both; v0.6.3: still ONE of the three
OWNED_LIKELY: dict[str, int | None] = {ATMEGA_BOARD_ITEM: ATMEGA_BOARD_KRW + ATMEGA_BOARD_SHIP_KRW,
                                       "M3 볼트·너트 키트 340개": None}
EQUIPMENT = [
    ("LiPo 충전기 iMAX B6AC 호환", 31400, 0, "https://ko.aliexpress.com/item/32856467525.html", "AC 내장"),
    ("USBasp ISP 프로그래머", 4400, 2700, "https://www.devicemart.co.kr/goods/view?no=1326948", ""),
]
# ground side of the ESP8266 link (firmware/esp_bridge/README.md "지상 ESP-07S 배선과 굽기"; params
# flight_computer.link.esp_bridge.range_plan); not in the total (prices ASSUMED, not researched - no listing read)
GROUND_ESP_ASSUMED = [
    ("지상 ESP8266 모듈 ESP-07S급 (u.FL 안테나 단자) (ASSUMED)", 4000, "firmware/esp_bridge 지상 역할 (ground_esp07s)"),
    ("2.4 GHz 8-9 dBi 무지향 안테나 + u.FL-SMA 피그테일 (ASSUMED)", 8000,
     "1 km 지상 거리 시험 계획(range_plan)의 안테나. EIRP 한도·KC 인증 확인 (ASSUMED)"),
    ("USB-UART 변환기 3.3 V (CH340/CP2102) + 3.3 V 레귤레이터·저항·커패시터·만능 기판 (ASSUMED)", 5000,
     "노트북 연결과 굽기. 변환기의 3.3 V 핀으로 ESP를 먹이지 않음 (전류 부족)"),
]
# ground side of the LoRa FALLBACK link, depends on the type of the user's LoRa modules; not in the total (prices ASSUMED,
# not researched)
GROUND_CONDITIONAL = [
    ("UART형 (Ebyte) 지상 모듈용 USB-UART 변환기 (CH340/CP2102, 3.3 V 신호) (ASSUMED)", 3000,
     "모듈에 USB 보드가 딸려 있으면 불필요"),
    ("SPI형 (SX127x) 지상 브리지용 Arduino Nano 호환 보드 (ASSUMED)", 6000, "firmware/ground_bridge"),
    ("SPI형 (SX127x) 지상 브리지용 레벨 시프터 4채널 (ASSUMED, 기체용과 같은 품목)", 1761, "Nano 5 V -> 모듈 3.3 V"),
]
PLA_URL = "https://kr.store.bambulab.com/products/pla-matte-filament"
PLA_KRW_PER_KG = 18000
PLA_SPARE = 0.10            # purchase spare per colour (failed prints; purge and prime tower are in the slice already)
# livery (user 2026-11-04): Korean names of the Bambu PLA Matte colours the print plans use
PLA_COLOUR_KO = {"Ivory White": "아이보리 화이트(흰색)", "Mandarin Orange": "만다린 오렌지(주황)", "Charcoal": "차콜(검정)",
                 "Black": "차콜(검정)"}


def pla_colour_rows(plan: dict, pla_g: float, src: str, label: str = "") -> tuple[list[tuple], dict]:
    """livery (2026-11-04): one PLA row per filament colour of the print plan (totals.filament_g_by_colour: slicer grams
    incl. purge / prime tower / supports, M) at PLA_KRW_PER_KG per 1 kg refill (the 2026-10-13 PLA Matte listing price,
    the same price ASSUMED for every colour), rolls = ceil(g x (1 + PLA_SPARE) / 1000).  A plan without the colour split
    (an older or the Fusion-path plan) gives one row of printer.color."""
    by = ((plan.get("totals") or {}).get("filament_g_by_colour")) or {"Ivory White": {"filament_g": pla_g, "code": "11100",
                                                                                     "hex": "#FFFFFF", "parts": ["all"]}}
    rows, out = [], {}
    for name, e in by.items():
        g = float(e["filament_g"])
        rolls = math.ceil(g * (1.0 + PLA_SPARE) / 1000.0)
        ko = PLA_COLOUR_KO.get(name, name)
        parts = e.get("parts") or []
        what = ("발사대" if parts == ["launch_pad"] else "YUCHAN 로고 상감 + 퍼지·프라임 타워 몫" if name == "Mandarin Orange"
                else "기체 출력 부품" + (" + 2색 부품의 퍼지·타워 몫" if any(p == "avionics_module" for p in parts) else ""))
        rows.append(("구조", f"Bambu PLA Matte {ko} ({name}, 코드 {e.get('code') or '-'}) 리필 1 kg x{rolls} "
                             f"({label}{what}: 슬라이스 {g:.0f} g + {100 * PLA_SPARE:.0f} %)",
                     PLA_KRW_PER_KG * rolls, 0, PLA_URL,
                     f"가격: 공식 스토어 PLA Matte 리필 {PLA_KRW_PER_KG:,}원/kg (2026-10-13 조사), 색마다 같은 값으로 가정(ASSUMED). "
                     f"구매할 때 색 이름·코드를 확인하세요. 7만원 이상 무료배송. 슬라이스 출처: {src}"))
        out[name] = {"filament_g": g, "rolls": rolls, "code": e.get("code"), "hex": e.get("hex"), "parts": parts}
    return rows, out


def cad_mass_g(L, path: str = CAD_REPORT) -> float:
    """Vehicle mass from the CadQuery report's totals - only a complete report of this preset is evidence."""
    with open(path, "r", encoding="utf-8") as f:
        cad = json.load(f)
    preset = L.p["meta"]["preset"]
    if cad.get("preset") != preset:
        raise SystemExit(f"{path} is for preset {cad.get('preset')!r}, not {preset!r}")
    if cad.get("failed_components"):
        raise SystemExit(f"{path} lists failed components {cad['failed_components']}")
    missing = sorted(set(se.expected_report_parts(L, L.p)) - set(cad.get("parts") or {}))
    if missing:
        raise SystemExit(f"{path} is missing components {missing}")
    return float(cad["totals"]["mass_g"])


GPS_ADDED_G = 0.0
GNS_ADDED_G = 0.0


def cad_gns_added_g(path: str = CAD_REPORT) -> float:
    """v0.8.1: what the GNS board adds to the vehicle, from the CadQuery report's ``extra.gns`` (measured there)."""
    with open(path, "r", encoding="utf-8") as f:
        cad = json.load(f)
    gx = (cad.get("extra") or {}).get("gns")
    if not gx:
        raise SystemExit(f"{path} has no extra.gns although params carry the GNS board: rebuild the twin")
    return float(gx["added_total_g"])


def cad_gps_added_g(path: str = CAD_REPORT) -> float:
    """v0.8: what the GPS pod adds to the vehicle, from the CadQuery report's ``extra.gps`` (measured there)."""
    with open(path, "r", encoding="utf-8") as f:
        cad = json.load(f)
    gx = (cad.get("extra") or {}).get("gps")
    if not gx:
        raise SystemExit(f"{path} has no extra.gps although params carry the GPS pod: rebuild the twin")
    return float(gx["added_total_g"])


def walls(L) -> str:
    """Body wall, duct wall (vane section) and fin thickness of a preset, from Layout."""
    return (f"{L.body_wall:.1f} mm (덕트 {(L.vane_section_od - L.duct_id) / 2:.1f} mm, 핀 {L.fin_t:.1f} mm)")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-dir", default=OUT_DIR, help="where 예산안.md and bom.json go (default docs/budget)")
    a = ap.parse_args(argv)
    L = layout.build()                       # the budget preset (params meta.preset)
    if FULL_PRESET not in L.p.get("alt_presets", {}):
        raise SystemExit(f"alt preset {FULL_PRESET} not in params.json")
    L_full = layout.build(FULL_PRESET)
    plan_path = os.path.join(ROOT, "cad", "exports", "print", "print_plan.json")
    with open(plan_path, "r", encoding="utf-8") as f:
        plan = json.load(f)
    pla_g = plan["totals"]["filament_g"]
    pla_src = "print_plan.json (Fusion 경로)"
    # v0.9: the official plan comes only from a Fusion GUI run; when it is for another params.json and the provisional
    # v0.9 plan (CadQuery twin STLs, sliced per print class) is for THIS one, the provisional filament total is used
    with open(os.path.join(ROOT, "params.json"), "rb") as f:
        params_sha = hashlib.sha256(f.read()).hexdigest()
    prov_path = os.path.join(ROOT, "cad", "exports", "print", "provisional_v09_cadquery", "print_plan_provisional.json")
    if (plan.get("source") or {}).get("params_sha256") != params_sha and os.path.exists(prov_path):
        with open(prov_path, "r", encoding="utf-8") as f:
            prov = json.load(f)
        if (prov.get("source") or {}).get("params_sha256") == params_sha:
            pla_g = prov["totals"]["filament_g"]
            pla_src = "임시 v0.9 출력 계획 (CadQuery 트윈 STL 슬라이스 - Fusion 실행 후 교체)"
    pla_plan = prov if pla_src.startswith("임시") else plan
    pla_rows, pla_by_colour = pla_colour_rows(pla_plan, pla_g, pla_src)
    pla_rolls = sum(v["rolls"] for v in pla_by_colour.values())
    rows = [(g, n, p, s, u, note) for (g, n, p, s, u, note) in MUST]
    if getattr(L, "servo_leads", False):     # v0.6.2: vane-servo extension wiring (ASSUMED, see SERVO_WIRE_*)
        ext = list(L.sl_ext_len)
        wire_m = math.ceil((sum(ext) + SERVO_WIRE_SPARE_MM) / 1000.0)
        servo_row = ("배선", f"베인 서보 연장선: 26 AWG 3심 플랫 실리콘 서보선 {wire_m} m + JR/듀폰 3핀 암 하우징·크림프 핀 "
                     f"{len(ext)}세트 + 열수축 튜브 (ASSUMED)",
                     wire_m * SERVO_WIRE_KRW_PER_M_ASSUMED + SERVO_HOUSINGS_PINS_KRW_ASSUMED + SERVO_HEATSHRINK_KRW_ASSUMED,
                     0, "",
                     f"추정값 - 가격 조사에 맞는 품목 없음 (실리콘 전선은 12-24 AWG 2선 세트만 조사됨). 연장 길이 "
                     + "/".join(f"{x:.0f}" for x in ext) + f" mm (합 {sum(ext):,.0f} mm, cad/layout.py 서보 배선 경로) + 여유 "
                     f"{SERVO_WIRE_SPARE_MM:.0f} mm -> {wire_m} m. 선 {SERVO_WIRE_KRW_PER_M_ASSUMED:,}원/m x {wire_m} + 하우징·핀 "
                     f"{SERVO_HOUSINGS_PINS_KRW_ASSUMED:,} + 수축 튜브 {SERVO_HEATSHRINK_KRW_ASSUMED:,}")
        rows.insert(next(i for i, r in enumerate(rows) if r[0] == "배선") + 1, servo_row)
    rows += pla_rows       # livery 2026-11-04: one row per colour (white body, orange logo, black launch pad)
    total = sum(p + (s or 0) for (_, _, p, s, _, _) in rows)
    names = [n for (_, n, _, _, _, _) in rows]
    missing_owned = [n for n in OWNED_LIKELY if n not in names]
    if missing_owned:
        raise SystemExit(f"OWNED_LIKELY names rows that do not exist: {missing_owned}")
    owned = sum(p + (s or 0) if OWNED_LIKELY[n] is None else min(OWNED_LIKELY[n], p + (s or 0))
                for (_, n, p, s, _, _) in rows if n in OWNED_LIKELY)
    equip = sum(p + (s or 0) for (_, p, s, _, _) in [(e[0], e[1], e[2], e[3], e[4]) for e in EQUIPMENT])

    mass = cad_mass_g(L)
    global GPS_ADDED_G, GNS_ADDED_G
    GPS_ADDED_G = cad_gps_added_g() if HAS_GPS else 0.0
    GNS_ADDED_G = cad_gns_added_g() if HAS_GNS else 0.0
    thrust_n = float(L.p["edf"]["thrust_nominal_N"])
    # T/W is the C12 evidence of the sizing report (flight mass = CAD + the hardware balance it adds); fall back to the
    # CAD mass only when that report is missing or stale, and say which one was used
    flight_mass, tw_src = mass, "CAD 질량"
    sz_path = os.path.join(ROOT, "sizing", "sizing_report.json")
    try:
        with open(sz_path, "r", encoding="utf-8") as f:
            sz = json.load(f)
        with open(os.path.join(ROOT, "params.json"), "rb") as f:
            sha = hashlib.sha256(f.read()).hexdigest()
        if sz.get("params_sha256") == sha and sz.get("preset") == L.p["meta"]["preset"]:
            flight_mass, tw_src = float(sz["thrust"]["mass_g"]), "크기 산정 비행 질량(스프링·피스톤 등 포함)"
    except (OSError, KeyError, ValueError):
        pass
    tw = thrust_n / (flight_mass * 1e-3 * G)

    out = ["# 예산안 (기체만, 캔위성 제외)", "",
           "가격은 2026-10-13에 각 판매 페이지에서 읽은 값입니다 (BMP280·NEO-7M·ESP-12E는 디바이스마트에서 2026-11-05에 읽고 "
           "2026-11-11에 다시 확인). 해외 가격은 1 USD = 1,341원으로 환산했습니다. "
           "알리익스프레스는 신규 회원 할인가가 아닌 정가를 썼습니다. 가격은 자주 바뀝니다. "
           "(ASSUMED) 표시 품목은 조사하지 않은 추정가입니다.", "",
           f"**합계: {total:,}원** (보유 가능성이 높은 ATmega128 보드 1개와 나사를 빼면 {total - owned:,}원)", "",
           *([f"비행 컴퓨터는 ATmega128A 4개입니다 (v0.8.1): NAV가 센서(MPU-6050·BMP280)·자세 추정·고도 필터를 맡아 매 틱 "
              f"항법 프레임을 SPI(nlink)로 FCU에 보내고, FCU가 감시(인터록)·자세 제어·PWM·로그를, COM이 무선(ESP-12E Wi-Fi, 예비 LoRa)·명령 해독·"
              f"텔레메트리를, GNS가 GPS 수신(UBX)·GPS 필터·위치 제어(50 Hz)를 맡습니다 (GNS는 NAV와 glink 직렬로 연결, 비행 "
              f"판단은 FCU만). GNS는 항공전자 튜브 -x 벽(배터리 바깥쪽, GPS 선 입구 바로 아래)에 인쇄 보스 4개로 나사 고정합니다. "
              f"네 보드는 같은 판매 페이지에서 사며 배송비 {ATMEGA_BOARD_SHIP_KRW:,}원을 보드마다 따로 넣었습니다 (묶음 배송 여부 "
              f"미확인). GNS 보드·나사·보스·GPS 선 변경으로 기체가 약 {GNS_ADDED_G:.0f} g 무거워지고, 이를 일부 상쇄하려고 GPS "
              f"포드 벽·바닥·뚜껑을 1.2 mm로 얇게 했습니다.", ""] if ATMEGA_BOARD_QTY == 4 else
             [f"비행 컴퓨터는 ATmega128A 3개입니다 (v0.6.3): NAV가 센서(MPU-6050·BMP280)·자세 추정·고도 필터를 맡아 "
              f"매 틱 항법 프레임을 SPI(nlink)로 FCU에 보내고, FCU가 감시(인터록)·자세 제어·PWM·로그를, COM이 무선(ESP-12E Wi-Fi, 예비 LoRa)·명령 "
              f"해독·텔레메트리 작성을 맡습니다 (NAV·COM은 비행 판단 없음). FCU는 흡입 모듈 격벽 위에 폼 테이프로 눕혀 붙입니다. "
              f"세 보드는 같은 판매 페이지에서 사며 배송비 {ATMEGA_BOARD_SHIP_KRW:,}원을 보드마다 따로 넣었습니다 (묶음 배송 여부 "
              f"미확인).", ""] if ATMEGA_BOARD_QTY == 3 else
             [f"비행 컴퓨터는 ATmega128A 2개입니다 (v0.6): FCU가 센서·추정·감시(인터록)·자세 제어·PWM·로그를 모두 맡고, "
              f"COM은 무선(ESP-12E Wi-Fi, 예비 LoRa)·명령 해독·텔레메트리 작성만 합니다 (비행 판단 없음). 두 보드는 같은 판매 페이지에서 사며 "
              f"배송비 {ATMEGA_BOARD_SHIP_KRW:,}원을 보드마다 따로 넣었습니다 (묶음 배송 여부 미확인).", ""]
             if ATMEGA_BOARD_QTY == 2 else []),
           *([f"v0.8: 위치 유지(바람·밀기)를 위해 GPS(NEO-7M 모듈 SZH-NT07; v0.8 설계 때는 GY-NEO6MV2) 1개를 더합니다. 안테나는 페이로드 모듈 바깥 "
              f"포드(θ {L.gps_theta:g}°, 노즈 분리면 바로 아래)에 {GPS_GP_SIDE:g} mm 접지판과 함께 넣고, 선은 튜브 바깥 통로를 "
              f"따라 항공전자 모듈로 들어가 {'GNS 보드' if HAS_GNS else 'NAV'}에 연결됩니다. 포드·뚜껑·GPS·배선을 합쳐 기체가 약 "
              f"{GPS_ADDED_G:.0f} g 무거워집니다 (CAD 측정).", ""] if HAS_GPS else []),
           f"기체 질량 CAD {mass:.0f} g, 비행 {flight_mass:.0f} g ({L.p['payload']['mass_g']:.0f} g 캔위성 포함), "
           f"추력/중량비 {tw:.2f} (대용 추력표의 공칭 추력 {thrust_n:.1f} N / {tw_src})", "",
           "| 분류 | 품목 | 가격 | 배송 | 비고 | 판매처 |", "|---|---|---|---|---|---|"]
    for g, n, p, s, u, note in rows:
        link = f"[링크]({u})" if u else "-"
        out.append(f"| {g} | {n} | {p:,} | {'?' if s is None else f'{s:,}'} | {note} | {link} |")
    out += ["", f"## 없으면 추가로 필요한 장비 (합계 {equip:,}원)", "", "| 품목 | 가격 | 배송 | 비고 | 판매처 |", "|---|---|---|---|---|"]
    for n, p, s, u, note in EQUIPMENT:
        out.append(f"| {n} | {p:,} | {s:,} | {note} | [링크]({u}) |")
    gmax = max(GROUND_CONDITIONAL[0][1], GROUND_CONDITIONAL[1][1] + GROUND_CONDITIONAL[2][1])
    gesp = sum(p_ for (_, p_, _) in GROUND_ESP_ASSUMED)
    out += ["", f"## Wi-Fi(ESP8266) 링크의 지상 쪽에 필요한 것 (합계에 미포함, {gesp:,}원)", "",
            "가격은 조사하지 않은 추정값(ASSUMED)입니다. 배선은 `firmware/esp_bridge/README.md`의 지상 ESP-07S 절을 따릅니다.", "",
            "| 품목 | 가격 | 비고 |", "|---|---|---|"]
    for n, p_, note in GROUND_ESP_ASSUMED:
        out.append(f"| {n} | {p_:,} | {note} |")
    out += ["", f"## 예비 LoRa 링크를 쓸 때 모듈 종류에 따라 지상 쪽에 추가로 필요할 수 있는 것 (합계에 미포함, 최대 {gmax:,}원)", "",
            "가격은 조사하지 않은 추정값(ASSUMED)입니다. 모듈 종류를 확인한 뒤 해당하는 줄만 사면 됩니다.", "",
            "| 품목 | 가격 | 비고 |", "|---|---|---|"]
    for n, p_, note in GROUND_CONDITIONAL:
        out.append(f"| {n} | {p_:,} | {note} |")
    chute = lambda lay: lay.p["recovery"]["vehicle_chute_d_m"]   # noqa: E731
    out += ["", "## 원래 설계와 무엇이 달라졌나", "",
            f"| 항목 | 원래 ({FULL_PRESET}) | 예산 ({L.p['meta']['preset']}) |", "|---|---|---|",
            "| 캔위성 | KAIST 규격 Ø110 x 200 mm, 1000 g | 국제 규격 Ø66 x 115 mm, 350 g |",
            "| EDF | JP Hobby 90 mm 12S | Hobbyocean 90 mm 6S |",
            "| ESC / 배터리 | 160 A HV / 6S 2200 x2 (12S) | 100 A / 6S 2200 x1 |",
            "| 전원 | 12S UBEC 2개 | ESC BEC (서보) + 6S UBEC (로직) |",
            "| 조종 링크 | SBUS 수신기 + RC 송신기 | ESP-12E Wi-Fi(ESP-NOW) 브리지 (명령·텔레메트리; 보유한 LoRa 모듈은 예비), "
            "노트북 지상국 |",
            f"| 벽 두께 | {walls(L_full)} | {walls(L)} |",
            f"| 전장 | {L_full.vehicle_h:.0f} mm | {L.vehicle_h:.0f} mm |",
            f"| 낙하산 | {chute(L_full):g} m 완제품 가정 | {chute(L):g} m 립스톱 자작 |"]
    os.makedirs(a.out_dir, exist_ok=True)
    with open(os.path.join(a.out_dir, "예산안.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    with open(os.path.join(a.out_dir, "bom.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump({"total_krw": total, "owned_likely_krw": owned,
                   "owned_likely_items": {n: (v if v is not None else "whole row") for n, v in OWNED_LIKELY.items()},
                   "atmega128_boards": {"quantity": ATMEGA_BOARD_QTY, "roles": ATMEGA_BOARD_ROLES.split("+"),
                                        "unit_krw": ATMEGA_BOARD_KRW,
                                        "shipping_krw_each": ATMEGA_BOARD_SHIP_KRW, "shipping_counted": "per board",
                                        "url": ATMEGA_BOARD_URL},
                   "nlink_harness": ({"krw": NLINK_HARNESS_KRW_ASSUMED, "assumed": True,
                                      "row": "own ASSUMED row (no matching item in the price research)"} if HAS_NAV else None),
                   "ams1117_modules": {"quantity": AMS1117_QTY, "unit_krw": AMS1117_KRW, "shipping_krw": 0,
                                       "url": AMS1117_URL, "priced_as": "quantity x the 1-pc listing price"},
                   "gns": ({"board": "ATmega128 minimum-system board #4 (in the board row)",
                            "level_shifter_krw": 344, "level_shifter_shipping_krw": 1417,
                            "ams1117_for_shifter": "one of the AMS1117 row (ASSUMED necessary)",
                            "glink_harness_krw_assumed": GLINK_HARNESS_KRW_ASSUMED, "added_mass_g_cad": GNS_ADDED_G}
                           if HAS_GNS else None),
                   "gps": ({"board_krw_excl_vat": GPS_BOARD_KRW_EXCL_VAT, "board_krw_incl_vat": GPS_BOARD_KRW_INCL_VAT,
                            "board_url": GPS_BOARD_URL, "board_shipping_krw": 0,
                            "board_shipping_note": "devicemart shipping counted once on the M3 screw-kit row (ASSUMED)",
                            "pod_parts_krw_assumed": GPS_POD_PARTS_KRW_ASSUMED,
                            "level_shifter": "GNS board's BSS138 (v0.8.1)" if HAS_GNS else "NAV's BSS138, 2 free channels",
                            "added_mass_g_cad": GPS_ADDED_G, "price_source": "devicemart goods page no=" + DM["neo7m"][0]
                            + " (pcb/vtol_carrier/spec/parts_sources.json 2026-11-05, re-read 2026-11-11)"}
                           if HAS_GPS else None),
                   "equipment_krw": equip, "pla_g": pla_g, "pla_source": pla_src, "pla_rolls": pla_rolls,
                   "pla_by_colour": pla_by_colour,
                   "vehicle_mass_g_cad": mass, "vehicle_mass_g_flight": flight_mass, "thrust_nominal_N": thrust_n,
                   "tw_nominal": tw, "tw_mass_source": tw_src,
                   "vehicle_h_mm": {L.p["meta"]["preset"]: L.vehicle_h, FULL_PRESET: L_full.vehicle_h},
                   "devicemart_parts": {k: {"goods_no": v[0], "krw_incl_vat": v[1], "url": dm_url(k)} for k, v in DM.items()},
                   "devicemart_read": list(DM_READ),
                   "rows": rows, "equipment": EQUIPMENT, "ground_esp_assumed": GROUND_ESP_ASSUMED,
                   "ground_esp_assumed_krw": gesp, "ground_conditional_assumed": GROUND_CONDITIONAL,
                   "ground_conditional_max_krw": gmax}, f, ensure_ascii=False, indent=1)
    print(f"total {total:,} KRW (without likely-owned {total - owned:,}); equipment if not owned {equip:,}; "
          f"ground-side ESP (ASSUMED) {gesp:,}, LoRa-fallback conditional (ASSUMED) up to {gmax:,}; PLA {pla_g:.0f} g -> {pla_rolls} rolls; "
          f"CAD mass {mass:.0f} g, flight mass {flight_mass:.0f} g -> T/W {tw:.3f}; length {L.vehicle_h:.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
