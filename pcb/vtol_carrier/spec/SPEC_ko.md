# VTOL 캐리어 PCB rev 3.1 - 전기 사양 요약

이 문서는 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts/build_spec.py` 가 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/carrier_spec.json` 과 함께 자동으로 만든 것입니다. 손으로 고치지 말고 스크립트를 다시 돌리십시오.
설계의 기준은 계획서 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/REV3_PLAN.md` (기계가 읽는 사본 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/rev3_plan.json`, 계획 날짜 2026-11-10) 이며, 사양이 계획과 부품 하나하나 같은지 스크립트가 비교합니다 (비교 177항목, 핀 692개, 차이 0건).

표시: F = 파일·데이터시트에서 읽음, M = 여기서 도구로 측정, E = 추정(계산), A = 가정. 근거가 없으면 NOT_VERIFIED.
이 문서의 수치는 사양을 만드는 스크립트의 계산·대조 결과입니다. ERC / DRC 결과는 kicad-cli 보고서(`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/reports/`)에만 있습니다.

## 1. rev 2c에서 달라진 것

- 5 V 전원: 꽂는 Pololu 모듈(J6)을 없애고 기판 위 승압 회로로 바꿨습니다. U8 TPS61023DRLR + L5 1.0 uH + 분압 R63 / R64 + C57 / C58, 그리고 저전압 차단용 비교기 U9 TLV3012BIDBVR + R65~R68 + C59 / C60 (새 부품 13개, 새 네트 5개: BOOST_SW, BOOST_FB, BOOST_EN, UVLO_SENSE, UVLO_REF).
- 무선 모듈: ESP-12E 대신 핀이 같은 ESP-12F (참고 부품 LCSC C82891). 풋프린트는 KiCad의 RF_Module:ESP-12E 를 그대로 씁니다 (라이브러리에 ESP-12F 풋프린트가 따로 없음). **2026-11-12 사용자 결정으로 이 모듈은 JLC가 아니라 사용자가 직접 납땜합니다** (fit user: JLC BOM / CPL에 없음. 풋프린트·네트·위치는 그대로).
- **스루홀 커넥터 12개도 2026-11-12 사용자 결정으로 JLC가 아니라 사용자가 사서 납땜합니다** (J2/J5, J8/J9/J10/J11/J12, J14, J15/J17, J16, J18; fit user). JLC 주문은 SMT 부품만 실장하고 손납땜은 없습니다. 풋프린트·네트·위치와 풋프린트의 실크 윤곽은 그대로입니다. (같은 날 검증 지적으로 아랫면 실크 이름표 가운데 핀 줄과 나란히 적힌 것의 핀 글자 순서만 아랫면에서 본 패드 순서로 고쳤습니다: 주문 안내서 7장.)
- MCU, 센서 모듈, ESP-12F 모듈, 스루홀 커넥터를 뺀 모든 실장 부품(모두 SMT)을 JLCPCB 조립 라이브러리의 주문 가능한 부품으로 정했습니다 (재고 근거: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/parts_sources.json`).
- 부품마다 `fit` (누가 붙이는가) 값을 가집니다. JLC BOM / CPL은 이 값에서만 만들어집니다.
- **rev 3.1 (2026-11-13, 사용자 결정 "캐리어 PCB를 로켓에도 씀")**: 같은 캐리어를 로켓에도 씁니다. 로켓은 FCU U2 PE5 / OC3C (핀 7; 펌웨어 src/hal/pwm.c:49, main.c:518 - 바꾸지 않음)로 CanSat 노즈 분리 서보를 움직이므로 분리 서보 출력을 되살렸습니다: PE5 -> R22 220 Ohm (JLC BOM 21줄 C22962에 합류) -> J13. J13은 +x / +y 모서리의 도금 스루홀 패드 1개(지름 2.0 mm, 구멍 1.0 mm)이고 붙일 부품이 없습니다: 로켓에서는 ESC와 J12 사이에 서보 Y 케이블을 끼우고, 그 분리 서보 쪽 가지의 신호선을 J13에 납땜합니다 (+ / -는 J12의 2 / 3번에서 Y 케이블로). VTOL에서는 비워 둡니다. 이 패드 말고 rev 3에서 바뀐 MCU 핀은 없습니다.
- 사용자가 고른 B안(-x 변 서보 열에 직각 헤더 J13, J18을 1핀으로)은 배치할 수 없었습니다 (M): 서보 열은 위아래가 rev 3 배터리 선 미리 배선(F.Cu 2.0 mm VBAT_LOGIC, B.Cu 2.4 mm VBAT_LOGIC_SW)으로 막혀 있고, 헤더 5개 + 1핀 J18에는 45.06 mm가 필요한데 44.60 mm뿐입니다 (가장 좋은 자리에서도 양 끝이 0.13 mm 간격 규칙보다 0.23 mm 부족). 배터리 배선은 바꾸지 않는다는 조건이라 사용자가 정해 둔 대안 C안으로 했습니다. 그래서 J18은 rev 3 그대로(1x4)입니다.
- MCU 4개의 핀 256개가 가는 네트(rev 3.1의 U2 PE5 하나만 예외), 커넥터·MCU·크리스털·모듈 소켓·ESP 위치는 rev 2c 그대로입니다. 기판 본체 76 x 70 mm 사각형과 그 변도 그대로이고, 2026-11-12 사용자 요청("보드의 사이즈를 조정하는 방법은 없어?")으로 -y 변(차체 -y 쪽)에 승압 회로를 올리는 탭을 붙였습니다: x 17.3..32, y -42.2..-35 mm (차체 좌표), 넓이 105.8 mm2, 외접 크기 76 x 77.2 mm. 이것은 계획 불변조건 (b) '외곽 76 x 70 mm 그대로'에서 벗어난 사용자 결정입니다. rev 2c 자료는 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/archive/rev2c_pololu_1s/` 에 보관했습니다.

## 2. 누가 무엇을 붙이는가

부품 자리 176개 (네트 107개).

| fit | 뜻 | 개수 | 부품 |
|---|---|---|---|
| jlc | JLC가 실장 (BOM / CPL에 들어감) | 145 | C1, C2, C3, C4, C5, C6 ... (목록은 carrier_spec.json 의 parts[].fit) |
| jlc_optional | 기본은 사용자 납땜, with_mcu 파일 쌍에서만 JLC 실장 | 4 | U1, U2, U3, U4 |
| user | 사용자가 납땜 | 14 | J1, J2, J5, J8, J9, J10, J11, J12, J14, J15, J16, J17, J18, U7 |
| none | 부품 없음 (빈 도금 구멍 / 미실장 / 구리 패턴만) | 13 | J3, J13, J19, J20, J21, J22, J24, J25, J26, JP1, NT1, Q3, R47 |

JLC 실장 부품 145개, BOM 33줄 (SMT 33줄 + THT 0줄). MCU까지 JLC에 맡기는 with_mcu 쌍은 149개 / 34줄입니다.

## 3. 5 V 전원부 (모두 E: 데이터시트 계산, 측정 아님)

- 출력 전압: 정격 5.06 V (공차 범위 4.84 ~ 5.28 V).
- 부하 가정: +5V 평소 273 mA / 최대 783 mA. IC 입력 3.0 V에서 낼 수 있는 출력 전류 1.815 A (최대 부하의 2.32배).
- 저전압 차단: 꺼짐 3.13 V (3.005 ~ 3.253 V), 켜짐 3.59 V (3.444 ~ 3.733 V). 저항값으로 고정되어 조정할 것이 없습니다.
- 차단 뒤에도 약 6.67 uA(새 회로) + 34~62 uA(R15 / R16 분압)가 흐릅니다. 보관할 때는 반드시 스위치를 끄십시오.
- U8, U9 핀 번호는 데이터시트 표에서 옮겼습니다: U8 TPS61023DRLR (SOT-563, DRL): 1 FB, 2 EN, 3 VIN, 4 GND, 5 SW, 6 VOUT; U9 TLV3012BIDBVR (SOT-23-6, DBV): 1 OUT, 2 V-, 3 IN+, 4 IN-, 5 REF, 6 V+.

## 4. JLC BOM 줄

| 줄 | Comment | Designator | Footprint | LCSC Part # | 공정 | 수량 |
|---|---|---|---|---|---|---|
| 1 | 100n 16V X7R 0402 | C1, C2, C4, C9, C10, C12, C17, C18, C20, C25, C26, C28, C54 | C_0402_1005Metric | C1525 | SMT | 13 |
| 2 | 10u 25V X5R 0805 | C3, C11, C19, C27, C44, C53, C56 | C_0805_2012Metric | C15850 | SMT | 7 |
| 3 | 100n 50V X7R 0603 | C5, C8, C13, C16, C21, C24, C29, C32, C46, C48, C49, C50, C51, C55, C59, C60 | C_0603_1608Metric | C14663 | SMT | 16 |
| 4 | 27p 50V C0G 0402 | C6, C7, C14, C15, C22, C23, C30, C31 | C_0402_1005Metric | C1557 | SMT | 8 |
| 5 | 22u 25V X5R 1206 | C41, C42, C43, C57 | C_1206_3216Metric | C12891 | SMT | 4 |
| 6 | 22u 10V tantalum A | C45 | CP_EIA-3216-18_Kemet-A | C11366 | SMT | 1 |
| 7 | 220u 10V electrolytic D6.3xL5.4 | C47 | CP_Elec_6.3x5.4 | C2833309 | SMT | 1 |
| 8 | 100u 10V tantalum B | C52 | CP_EIA-3528-21_Kemet-B | C7196 | SMT | 1 |
| 9 | 1n 50V X7R 0603 | C58 | C_0603_1608Metric | C1588 | SMT | 1 |
| 10 | LED red 0603 | D1, D2, D3, D4 | LED_0603_1608Metric | C2286 | SMT | 4 |
| 11 | B5819W 1A 40V Schottky SOD-123 | D6 | D_SOD-123 | C8598 | SMT | 1 |
| 12 | JST-GH BM04B-GHS-TBT 4P vertical | J7 | JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical | C161692 | SMT | 1 |
| 13 | 10uH 0805 multilayer inductor | L1, L2, L3, L4 | L_0805_2012Metric | C1046 | SMT | 4 |
| 14 | 1.0uH FXL0420-1R0-MA1 | L5 | L_Changjiang_FXL0420 | C53426685 | SMT | 1 |
| 15 | BSS138 | Q1, Q2, Q4, Q5, Q6, Q7, Q8, Q11 | SOT-23 | C40912 | SMT | 8 |
| 16 | MMBT3904 | Q9 | SOT-23 | C20526 | SMT | 1 |
| 17 | AO3401A | Q10 | SOT-23 | C15127 | SMT | 1 |
| 18 | 10k 1% | R1, R3, R5, R7, R11, R14, R23, R24, R25, R26, R27, R29, R31, R38, R40, R41, R43, R45, R46, R48, R49, R50, R51, R52, R53, R55, R56, R57, R58, R59, R60, R61, R62 | R_0603_1608Metric | C25804 | SMT | 33 |
| 19 | 1k 1% | R2, R4, R6, R8, R12, R13, R28, R30, R54 | R_0603_1608Metric | C21190 | SMT | 9 |
| 20 | 47k 1% | R15, R16, R32, R34, R36 | R_0603_1608Metric | C25819 | SMT | 5 |
| 21 | 220R 1% | R17, R18, R19, R20, R21, R22 | R_0603_1608Metric | C22962 | SMT | 6 |
| 22 | 4.7k 1% | R33, R35, R37, R42, R44 | R_0603_1608Metric | C23162 | SMT | 5 |
| 23 | 100k 1% | R39 | R_0603_1608Metric | C25803 | SMT | 1 |
| 24 | 150k 1% | R63 | R_0603_1608Metric | C22807 | SMT | 1 |
| 25 | 20k 1% | R64 | R_0603_1608Metric | C4184 | SMT | 1 |
| 26 | 470k 1% | R65 | R_0603_1608Metric | C23178 | SMT | 1 |
| 27 | 270k 1% | R66 | R_0603_1608Metric | C22965 | SMT | 1 |
| 28 | 3.3M 1% | R67 | R_0603_1608Metric | C26096 | SMT | 1 |
| 29 | 1M 1% | R68 | R_0603_1608Metric | C22935 | SMT | 1 |
| 30 | AMS1117-3.3 | U6 | SOT-223-3_TabPin2 | C6186 | SMT | 1 |
| 32 | TPS61023DRLR | U8 | SOT-563 | C919459 | SMT | 1 |
| 33 | TLV3012BIDBVR | U9 | SOT-23-6 | C20345924 | SMT | 1 |
| 34 | 16MHz 18pF 3225 | Y1, Y2, Y3, Y4 | Crystal_SMD_3225-4Pin_3.2x2.5mm | C70562 | SMT | 4 |

with_mcu 쌍에만 추가되는 줄: ATmega128A-AU TQFP-64 | U1, U2, U3, U4 | TQFP-64_14x14mm_P0.8mm | C6510 | SMT | 4

Footprint 칸은 KiCad 풋프린트 이름입니다 (주문용 BOM 파일의 Footprint 칸과 같은 값). 주문용 파일은 내보내기 단계(`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts/checks_exports.py`)가 이 사양의 `fit` 값으로 만든 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom.csv` / `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl.csv` (MCU 포함 쌍: `*_with_mcu.csv`) 이고, 위 표는 그 파일의 근거입니다. 주문 방법은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 에 있습니다.

## 5. 프로그래밍 헤더 (결정 D6)

현재 설정: **A** (환경변수 `VTOL_PROG_HEADERS`, 기본 A).
- A: J3, J19~J22 (2x3), J24~J26 (1x2) 은 rev 2c와 같은 자리의 빈 도금 구멍입니다. 프로그래밍하려면 사용자가 헤더를 납땜하거나 포고핀 어댑터를 씁니다 (포고핀 접촉은 NOT_VERIFIED).
- B: 같은 부품번호를 케이블타이 띠 밖에 JLC가 실장하는 SMD 헤더로 둡니다 (2x3: C32713283, 1x2: C41417359; BOM 35줄, JLC 실장 153개). 사용자의 승인이 있어야 적용합니다. 띠 밖에 자리가 있는지 배치 단계에서 찾은 결과는 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/reports/programming_access.json` 에 있습니다 (사양 문서는 배치 결과를 읽지 않습니다; 요약은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/README_ko.md`).

## 6. 사용자가 붙이거나 꽂는 것

| 자리 | 부품 | 방법 |
|---|---|---|
| U1~U4 | ATmega128A-AU (TQFP-64) 4개 | 기본: 사용자가 납땜 (칩당 64핀, 모두 256곳). JLC에 맡기려면 with_mcu BOM / CPL 쌍으로 주문 (LCSC C6510) |
| U7 | ESP-12F Wi-Fi 모듈 (참고 부품 LCSC C82891) | 사용자가 납땜 (2026-11-12 사용자 결정; 그 전 계획은 JLC 실장). JLC BOM / CPL에 없고, 두 거버 묶음 모두 이 패드에 솔더 페이스트 구멍이 없음. 패드 수와 납땜 순서는 주문 안내서 7장 |
| J1 | GY-521 MPU-6050 모듈 | 모듈에 딸린 1x8 핀을 기판의 도금 구멍 8개에 납땜 (모듈은 약 2.5 mm 받침 위에 눕힘 - 핀 스트립 플라스틱 높이, 가정 A). 기판에는 구멍만 있음 |
| J2 | BMP280 모듈 TYE-BMP29 | 직각 6핀 헤더를 소켓 J2에 꽂음 (소켓은 사용자가 납땜; 받은 모듈의 헤더가 일자형이면 직각 핀을 먼저 납땜 - 헤더 종류 NOT_VERIFIED) |
| J7 | NEO-7M GPS 모듈 SZH-NT07 | 기체 상단 캡에 두고 4선 JST-GH 리드를 J7에 꽂음 |
| J5 | SparkFun OpenLog DEV-13712 | OpenLog에 직각 1x6 핀을 납땜한 뒤 소켓 J5에 꽂음 (소켓은 사용자가 납땜; 맞는지 NOT_VERIFIED) |
| J2, J5 | 스루홀 커넥터: socket 1x6 right-angle 2.54 (MPN HC-PM254-8.5H-1x6PW, 참고 부품 LCSC C22436147) 2개 | 사용자가 사서 납땜 (2026-11-12 사용자 결정; 그 전 계획은 JLC가 손납땜). 납땜점 수·순서·방향은 주문 안내서 7장 |
| J8, J9, J10, J11, J12 | 스루홀 커넥터: pin header 1x3 right-angle 2.54 (MPN HX PZ2.54-1x3P WZ, 참고 부품 LCSC C32713262) 5개 | 사용자가 사서 납땜 (2026-11-12 사용자 결정; 그 전 계획은 JLC가 손납땜). 납땜점 수·순서·방향은 주문 안내서 7장 |
| J14 | 스루홀 커넥터: Molex 22035025 SPOX 2P vertical (MPN 22035025, 참고 부품 LCSC C5364642) 1개 | 사용자가 사서 납땜 (2026-11-12 사용자 결정; 그 전 계획은 JLC가 손납땜). 납땜점 수·순서·방향은 주문 안내서 7장 |
| J15, J17 | 스루홀 커넥터: JST B2B-PH-K-S 2P vertical (MPN B2B-PH-K-S(LF)(SN), 참고 부품 LCSC C131337) 2개 | 사용자가 사서 납땜 (2026-11-12 사용자 결정; 그 전 계획은 JLC가 손납땜). 납땜점 수·순서·방향은 주문 안내서 7장 |
| J16 | 스루홀 커넥터: pin header 1x2 right-angle 2.54 (MPN HX PZ2.54-1x2P WZ, 참고 부품 LCSC C32713261) 1개 | 사용자가 사서 납땜 (2026-11-12 사용자 결정; 그 전 계획은 JLC가 손납땜). 납땜점 수·순서·방향은 주문 안내서 7장 |
| J18 | 스루홀 커넥터: pin header 1x4 right-angle 2.54 (MPN HX PZ2.54-1x4P WZ, 참고 부품 LCSC C32713263) 1개 | 사용자가 사서 납땜 (2026-11-12 사용자 결정; 그 전 계획은 JLC가 손납땜). 납땜점 수·순서·방향은 주문 안내서 7장 |

기판 밖에서 따로 사는 것(배터리, 스위치와 리드, GPS 리드, 부저, 모듈, MCU 등)의 목록은 계획서의 off_board_purchase 항목(`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/rev3_plan.json`)에 있고, 구매 목록 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/off_board_purchase.csv` 와 주문 안내서 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 6장에 표로 있습니다. 이 품목들의 재고·가격은 NOT_VERIFIED입니다.

## 7. 사용자 결정 (기본값으로 진행 중)

- D1 Pololu 모듈을 대신할 5 V 변환기: 기본 A (TPS61023 승압 + TLV3012B 저전압 차단), 권장 A
- D2 로직 배터리(1S) 차단 전압: 기본 유지 (꺼짐 약 3.13 V, 켜짐 약 3.59 V), 권장 유지
- D3 요청 때문이 아니라 재고·도면 근거로 바꾼 부품번호 3건: 기본 유지 (2026-11-12부터 그중 2건 J8~J12, J2 / J5는 사용자가 사서 납땜하는 부품, 번호는 구매 참고용)
- D4 ATmega128A-AU 4개를 누가 붙이는가: 기본 사용자 납땜, 권장 JLC 실장 (with_mcu 파일 쌍)
- D5 OpenLog 장착 방식: 기본 소켓 J5에 꽂기 (센서 모듈처럼 사용자가 장착), 권장 동일
- D6 프로그래밍 헤더 (J3, J19~J22, J24~J26): 기본 A (케이블타이 띠 안의 빈 도금 구멍), 자리가 확인되면 권장 B (띠 밖에 JLC 실장)
- D7 스위치·부저 커넥터 J15 / J17 (JST-PH): 기본 THT (2026-11-12부터 사용자가 사서 납땜)
- D8 레벨시프터 FET (보드당 8개): 기본 C40912, 권장 C40912
- D9 JLC 주문 종류: 기본 Economic (2026-11-12 사용자 결정; 2026-11-10 계획의 기본·권장은 Standard)
- D10 툴링 홀: 기본 설계에 넣지 않음 (Economic 주문에는 레일이 없어 JLC가 기판 안에 뚫음; 위치는 NOT_VERIFIED)
- D11 수량: 기본 PCB 5장 / 실장 2장, 권장 먼저 2장 실장
- D12 모듈 소켓 J2 / J5 (1x6 직각): 기본 C22436147 (2026-11-12부터 사용자가 사서 납땜; 구하기 어려우면 C55629861, 그다음 C54876735 - 풋프린트 호환 NOT_VERIFIED)
- D13 배터리 전압 분압 R15 / R16 (47k / 47k): 기본 유지
- D14 습기에 민감한 부품(LED; ESP-12F는 2026-11-12부터 사용자가 실장) 베이킹 요청: 기본 아니오 (2026-11-12 변경; 주문 화면에서 Economic PCBA는 베이킹을 고를 수 없음; 계획은 예)
- D15 렌더와 맞춤 검사에 쓰는 모듈 치수: 기본 가정값 (실측 전까지 모든 그림·검사에 A 표시), 권장 실측
- D16 주문 화면의 PCB 옵션: 기본 무연 HASL (2026-11-12 사용자 결정; 계획의 권장은 ENIG), 녹색 마스크, 흰색 실크, JLC 기본 4층 적층, 비아 덮기 (주문 화면이 Plugged로 정함)
- D17 실장하지 않는 여분 기판: 기본 수락

## 8. 확인되지 않은 것

- 만들거나 측정한 것은 없습니다. 전원부 수치는 모두 데이터시트 계산값(E)입니다.
- JLC가 이 기판을 받아 주는지는 사용자가 파일을 올리고 DFM 결과와 견적을 본 뒤에야 알 수 있습니다 (결제 전 확인).
- 스루홀 커넥터 12개와 ESP-12F는 JLC가 실장하지 않습니다 (2026-11-12 사용자 결정). 사용자가 따로 사서 납땜하며, 낱개로 산 부품이 구멍에 맞는지와 그 재고·가격은 NOT_VERIFIED입니다 (주문 안내서 6장, 7장).
- 이 문서는 사양 단계의 내용만 담습니다. 배치·배선 뒤의 결과(전원부 자리, 배터리 경로 폭 검사, 가장자리 거리, 모듈 간섭)는 README_ko.md 와 out/reports/ 의 보고서에 있습니다.
- U8은 라이브러리에 전용 심볼이 없어 일반 6핀 심볼을 씁니다. ERC는 U8 핀에 대해 아무 근거도 주지 않으며, 핀-네트 표 검사(rev3_checks.py u8-u9)가 그 역할을 합니다.
- 차단 히스테리시스 여유는 최대 부하에서 28 mV(E)입니다. 전원 경로 저항이 0.29 Ohm을 넘으면 방전 끝에서 켜짐/꺼짐을 반복할 수 있습니다 (첫 기판에서 측정).
- 켜짐 문턱이 최대 3.73 V입니다. 쉬는 전압이 그보다 낮은 팩은 보드를 켜지 못할 수 있습니다.
- 극성 부품과 IC의 CPL 회전은 NOT_VERIFIED입니다. 주문 때 'Confirm Parts Placement'를 켜고, 전원을 넣기 전에 탄탈 방향을 눈으로 확인하십시오.
- 재고는 움직입니다. spec/parts_sources.json 기록(2026-11-10) 기준으로 재고가 얇은 줄: 없음. 주문일에는 scripts/jlc_gate.py 를 다시 돌려 모든 줄을 새로 확인합니다.
- 센서 모듈·OpenLog의 치수와 핀 순서는 가정(A)입니다. 받은 모듈의 실크를 확인하십시오.

## 9. 파일과 다시 만드는 방법

- 사양: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/carrier_spec.json`
- 계획: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/REV3_PLAN.md`, `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/rev3_plan.json`
- JLC 부품 근거: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/parts_sources.json`
- 라이브러리 확인: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/library_probe.json`
- rev 2c 보관본: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/archive/rev2c_pololu_1s/`

전체(사양 → 회로도 → 배치 → 기록된 배선 재생 → 검사 → 주문 파일 → 그림 → 문서)는 작업 폴더 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts` 에서 `B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py` 한 줄로 다시 만듭니다 (단계 목록: `make_all.py --list`). 사양 부분만 다시 만들 때의 순서:

1. `probe_library.py` - KiCad 라이브러리에서 심볼·풋프린트·핀을 읽어 기록
2. `jlc_sources.py --today 2026-11-10` - 저장된 JLC 기록으로 부품 근거를 다시 평가 (네트워크 없음; 주문일 재고 확인은 `jlc_gate.py`)
3. `build_spec.py` - 이 사양과 이 문서를 생성
4. `build_schematic.py` - 회로도 생성
5. `rev3_checks.py all` - kicad-cli ERC·네트리스트 내보내기와 네트리스트 기준 검사
