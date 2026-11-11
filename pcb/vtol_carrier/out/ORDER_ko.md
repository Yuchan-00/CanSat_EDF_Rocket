# JLCPCB 주문 안내서 - VTOL 캐리어 PCB rev 3

이 안내서는 PCB 실장 주문(PCBA)을 처음 하는 사람도 따라 할 수 있게 썼습니다. **이 도구는 아무것도 업로드하거나 주문하지 않았습니다.** 업로드, 확인, 결제는 모두 사용자가 JLCPCB 사이트에서 직접 합니다.

표시: M = 도구로 측정, E = 추정, A = 가정, F = 문서 근거, NOT_VERIFIED = 근거 없음. 측정·계산된 숫자의 출처는 맨 끝 11장에 있습니다. 그 표에 없는 숫자(시험 전류·전압, 규칙 값, 수량 기본값)는 계획서 `spec/REV3_PLAN.md` 의 괄호 안 장(15장, 16장의 D 번호 등)에서 옮긴 값(F, 계산값은 E)입니다. 이 파일은 `scripts/docs_ko.py` 가 만든 것입니다.

## 0. 먼저 결정할 것: 도구 판정은 아직 '주문 불가'입니다

출고 판정: **NOT ORDERABLE** (routing_rules FAIL (battery_path)).

- 무엇이 문제인가: 배터리 → 승압 회로로 가는 전원 경로의 구리 폭을 계획은 2.0 mm 이상(또는 면)으로 정했는데, 16.1 mm 구간이 더 좁습니다: VBAT_LOGIC_RAW 1.07 mm (폭 1 mm); VBAT_LOGIC 3.93 mm (폭 1 mm); VBAT_LOGIC_SW 11.1 mm (폭 1.5/1.56 mm).
- 영향(E): 구리 저항은 20 C에서 0.03231 Ohm 로 목표 0.035 Ohm 안이지만, 60 C에서는 0.03739 Ohm 로 넘습니다. 저항이 크면 방전 끝에서 저전압 차단이 켜졌다 꺼졌다 할 여유가 줄어듭니다 (계획 17장).
- 선택지 (도구가 정하지 않습니다):
  1. 이대로 주문: 폭 규칙 위반을 알고 받아들인다고 사용자가 정합니다. 파일은 그대로 쓸 수 있지만 도구 판정은 바뀌지 않습니다. 보드를 받으면 7장의 '배터리에서 U8 입력까지 저항' 측정을 꼭 하십시오.
  2. 고친 뒤 주문: 해당 구간의 미리 그린 배선이나 배치를 바꾸고 배선을 새로 해야 합니다 (시간이 걸리고, 결과가 달라질 수 있음). 사용자의 승인이 필요합니다.
- 아래 1~9장은 1번을 고를 경우를 기준으로 쓴 주문 절차입니다.

## 1. 올릴 파일 3개

기본 (MCU 4개는 사용자가 직접 납땜, 요청대로):

| 순서 | 파일 | 어디에 올리나 | sha256 앞부분 |
|---|---|---|---|
| 1 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier_gerber.zip` | PCB 주문 첫 화면 'Add gerber file' | 9cea8391a5c3da90... |
| 2 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom.csv` | PCB Assembly 단계 'BOM file' | fa4c8e2622885f4d... |
| 3 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl.csv` | PCB Assembly 단계 'CPL file' (Pick and Place) | 2dbd7ed2d8b13150... |

다른 선택 (MCU 4개도 JLC가 실장, 결정 D4의 권장안): 아래 3개를 **한 묶음으로** 올립니다.

| 순서 | 파일 | sha256 앞부분 |
|---|---|---|
| 1 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier_gerber_with_mcu.zip` | 3c8df77ed79e4aaf... |
| 2 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom_with_mcu.csv` | 465199b917ff914d... |
| 3 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl_with_mcu.csv` | 346adddc4f12d4b0... |

- **두 묶음을 섞지 마십시오.** 두 zip은 F.Paste(솔더 페이스트) 층만 다릅니다: 기본 zip은 U1~U4 패드에 페이스트 구멍이 없고 (페이스트 패드 347개), with_mcu zip은 있습니다 (603개).
- 기본 BOM: 40줄 (SMT 34 + THT 6), 부품 157개, CPL 157행. with_mcu: 41줄, 부품 161개, CPL 161행.
- 기본 BOM에 MCU(U1~U4)가 들어 있는지: 없음. 센서 자리 J1과 프로그래밍 단자도 들어 있지 않습니다 (사용자 몫).
- 파일 형식은 JLC 견본 파일(Sample-BOM_JLCSMT, Sample-CPL_JLCSMT)과 같은 머리줄로 만들었지만, JLC가 받아들이는지는 업로드 미리보기에서 모든 줄이 매칭될 때까지 NOT_VERIFIED입니다.

## 2. 주문 화면 입력표 (결정 D9, D10, D11, D14, D16)

화면의 항목 이름은 확인하지 못했습니다 (NOT_VERIFIED - 업로드한 적 없음). 비슷한 이름의 항목을 찾아 아래 값으로 맞추십시오.

| 화면 항목 | 넣을 값 | 근거 |
|---|---|---|
| Layers | 4 | 보드 4층 (거버 검사 M) |
| Dimensions | 76 x 70 mm (거버를 올리면 자동으로 읽힘 - 같은지 확인) | 사양 |
| PCB Qty | 5 | D11 기본 (PCB 5장 중 2장 실장) |
| PCB Thickness | 1.6 mm | 사양 |
| Surface Finish | ENIG | D16 (MCU 0.8 mm 피치 패드와 나중에 납땜할 구멍에 평평한 표면; 추가 요금 NOT_VERIFIED) |
| Solder Mask / Silkscreen | Green / White | D16 (JLC 기본값) |
| 적층(Stackup), 동박 두께, Impedance Control | JLC 기본 4층 적층, 기본 동박, 임피던스 제어 없음 | D16 |
| Via Covering | Tented | D16 |
| 주문번호 표시 (Mark on PCB / Remove Order Number) | 기본값 (JLC가 위치를 정함) | D16 |
| PCB Assembly | 켬 (On) | 요청 |
| PCBA Type | Standard | D9 (아래 설명) |
| Assembly Side | Top Side | 모든 부품이 윗면 (사양) |
| PCBA Qty | 2 | D11 (JLC 최소 2장) |
| Edge Rails / Tooling holes | JLC가 붙임 (Added by JLCPCB) | D10: 설계에 툴링 홀 없음 |
| Confirm Parts Placement | Yes | 회전이 NOT_VERIFIED인 부품이 있음 (3장) |
| 메모 (Remark) | 아래 문장 붙여넣기 | D14, 가장자리 거리 표 |

- Standard를 고른 이유 (D9): 부품 몸체 32개가 기판 끝에서 2.5 mm 안에 있고, 그중 U7 0.15 mm, J15 0.29 mm 는 Economic 방식이 권하는 0.3 mm보다 가깝습니다 (M; 0.3 mm와 2.5 mm는 계획 10장의 JLC 규정, F). 커넥터 위치는 기체에 맞춰 고정이라 옮길 수 없습니다. Standard는 JLC가 공정 레일을 붙이고, 리플로 온도도 더 낮습니다 (240 C, 계획 10장, F).
- 레일 쪽 (E): 표에서 '+y / -y' 쌍(거버 화면에서 위·아래의 76 mm 긴 변)을 골랐습니다. 이 경우 레일이 없는 쪽에 2.5 mm 안 부품이 14개 남습니다. JLC가 받아 주는지는 DFM 결과로만 압니다.
- 툴링 홀 부품(H) 수: 0 (설계에 없음, M).

메모 문장 (기본 묶음, 그대로 복사):

```
Please bake the moisture-sensitive parts (ESP-12F C82891, LEDs C2286) before assembly.
Please add 5 mm process edge rails on the two 76 mm long edges (top and bottom in the Gerber view):
several parts are closer than 2.5 mm to the board edge.
The 6 THT lines (J2/J5, J8-J12, J14, J15/J17, J16, J18) are to be soldered by JLCPCB.
U1-U4 and J1 are intentionally NOT assembled (no BOM/CPL rows). Please do not add parts for them.
```

with_mcu 묶음이면 마지막 줄을 `J1 is intentionally NOT assembled (no BOM/CPL row).` 로 바꾸십시오.

## 3. 결제하기 전에 확인할 것 (하나라도 다르면 멈추고 결제하지 마십시오)

1. 주문 당일, 업로드 직전에 재고를 다시 확인합니다: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts` 에서 `B:/Claude/.venv/Scripts/python.exe -X utf8 -B jlc_gate.py` (인터넷, 읽기만, 약 2~3분). 결과 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlc_gate_report.json` 의 status가 PASS여야 합니다. 마지막 확인: 2026-11-11T16:19:27+09:00 PASS (40/40줄), with_mcu PASS.
2. BOM 매칭 화면: 기본 묶음이면 40줄 모두, with_mcu면 41줄 모두 부품이 매칭되어야 합니다. '부품 없음', '다른 부품 추천', '재고 부족'이 하나라도 있으면 멈춥니다 (대체는 5장의 정해진 대체 부품만, 그것도 다시 확인 후).
3. THT 6줄(J2/J5, J8~J12, J14, J15/J17, J16, J18; 납땜점 39개)이 견적에 'JLC가 납땜'으로 들어가 있어야 합니다. 빠져 있거나 거절되면 멈춥니다.
4. 부품 위치 미리보기(Confirm Parts Placement)를 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/cpl_rotation_check.png` 그림과 하나씩 비교합니다. 특히 극성: 탄탈 C45 / C52, 전해 C47 (거꾸로면 화재 위험), 그리고 회전이 확인 안 된 C45, C47, C52, U9. U7 / U8 / U9 / J7의 1번 핀 방향. 회전 근거는 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/cpl_rotation_table.csv` 의 status 칸에 있습니다.
5. DFM 결과: 가장자리 가까운 부품(U7 0.15 mm, J15 0.29 mm 등)에 대한 이의, 레일 위치, 구멍·간격 지적이 없어야 합니다. JLC가 이의를 달면 주문 방식이나 부품을 마음대로 바꾸지 말고 멈춘 뒤 다시 검토합니다.
6. 견적 금액을 4장의 추정과 비교합니다. 크게 다르면 이유를 확인합니다.

## 4. 예상 비용 (E, 추정)

| 묶음 | 방식 | 실장 장수 | 합계 (E) | Standard 전용 수수료까지 붙으면 |
|---|---|---|---|---|
| 기본 | Economic | 2 | 80.24 USD | - |
| 기본 | Standard | 2 | 122.55 USD | 154.40 USD |
| 기본 | Standard | 5 | 152.56 USD | 184.41 USD |
| with_mcu | Standard | 2 | 172.98 USD | 204.83 USD |

- 포함: 셋업, 스텐실, 부품 종류 수수료, 부품 장착, THT 손납땜, 위치 확인 수수료, 부품값(손실분·최소 수량 포함). Standard 전용 수수료 31.85 USD (취급·고정구·포장)는 적용 여부가 NOT_VERIFIED입니다.
- **빠진 것:** PCB 제작비, ENIG 추가 요금, 베이킹 요금, 레일 분리 요금, 배송비, 세금. 실제 금액은 JLC 견적뿐입니다.
- 전체 표: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlc_fee_estimate.csv`

## 5. 재고가 적은 줄과 대체 부품

- 얇은 재고(기준 1000개 미만): C22436147 (J2,J5) available 503.
- 지켜볼 줄 (2000개 미만): C53426685 (L5) 1065개, 필요 22, C20345924 (U9) 1088개, 필요 20, C22436147 (J2,J5) 503개, 필요 40.
- 대체는 계획에 적힌 순서대로만 씁니다. 다른 부품으로 바꾸는 것은 설계 변경이라 다시 검사하고 사용자가 정해야 합니다.

| 원래 부품 | 대체 부품 (주문 가능 수량, 판정) |
|---|---|
| J2, J5 (C22436147) | C55629861 SH-PM254R-1x6P-H85 (985개, PASS); C54876735 PM2.54-1x6PWZ-H8.5 (595개, PASS) |
| L5 (C53426685) | C167203 FXL0420-1R0-M (106884개, PASS) |
| J7 (C161692) | C52037397 GH1.25-4PLT (1843개, PASS); C2829262 GH1.25-4PLTPZ (55720개, PASS) |
| U8 (C919459) | C1852149 TPS61023DRLT (526개, PASS) |
| J8..J12, J16, J18 (THT right-angle headers) if JLC refuses hand soldering | C46061676 HX PZ2.54-1x3P WT (20111개, PASS); C46061675 HX PZ2.54-1x2P WT (18761개, PASS); C46061677 HX PZ2.54-1x4P WT (13006개, PASS) |
| J15, J17 (C131337) | C160352 B2B-PH-SM4-TB(LF)(SN) (46998개, PASS) |
| U9 (C20345924) | 없음 - 같은 사양의 두 번째 부품이 없습니다. 재고가 없으면 주문을 미룹니다 (계획 7장, F) |

## 6. 따로 사야 하는 것 (JLC는 낱개 부품을 보내주지 않습니다)

목록 19개: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/off_board_purchase.csv`. 재고와 가격은 확인하지 않았습니다 (NOT_VERIFIED).

| 품목 | 보드당 | 2장 분 | 어디에 | 구입처 (제안) |
|---|---|---|---|---|
| ATmega128A-AU (TQFP-64) | 4 | 8 | U1 U2 U3 U4 footprints (user solders 64 pins each) | LCSC C6510 or a domestic distributor - not needed if the *_with_mcu files are ordered |
| GY-521 MPU-6050 module (SZH-EK007) | 1 | 2 | J1 (solder the module's own 1x8 strip; support about 2.5 mm = strip plastic height, A) | devicemart no=1247052 |
| BMP280 module TYE-BMP29 | 1 | 2 | J2 socket (right-angle 6-pin header of the module) | devicemart no=15442588 |
| right-angle 1x6 2.54 mm pin strip (only if the BMP280 comes with a straight header) | 1 | 2 | BMP280 module -> J2 | domestic shop (any RA 2.54 mm strip cut to 6) |
| NEO-7M GPS module SZH-NT07 | 1 | 2 | vehicle top cap; lead to J7 | devicemart no=10895400 |
| GPS lead JST-GH 1.25 mm 4-pin, both ends | 1 | 2 | J7 <-> NEO-7M | ready-made lead (domestic shop) or 2 x GHR-04V-S (LCSC C160418) + 8 x SSHL-002T-P0.2 (LCSC C189897) + crimp tool |
| SparkFun OpenLog DEV-13712 | 1 | 2 | J5 socket | SparkFun distributor / domestic shop |
| microSD card | 1 | 2 | OpenLog | any shop |
| right-angle 1x6 2.54 mm pin header for the OpenLog | 1 | 2 | solder onto the OpenLog, plug into J5 | domestic shop or LCSC C55629820 |
| logic battery 1S Li-Po DTP105565P 3.7 V 4200 mAh | 1 | 2 | J14 (Molex 5264-type plug; mating and polarity NOT_VERIFIED - meter first) | devicemart no=16021596 |
| power switch MTS-102 + JST-PH 2-pin lead | 1 | 2 | J15 | switch: domestic shop; lead: ready-made PH 2.0 mm 2-pin or PHR-2 (LCSC C157955) + 2 x SPH-002T-P0.5S (LCSC C111515) |
| arm switch + 2-pin 2.54 mm Dupont lead | 1 | 2 | J16 | domestic shop |
| buzzer + JST-PH 2-pin lead | 1 | 2 | J17 | domestic shop (PHR-2 C157955 + SPH-002T-P0.5S C111515 at LCSC) |
| 2x3 2.54 mm pin header (programming, option A) | 5 | 10 | J3, J19, J20, J21, J22 (bare holes in the tie band) | domestic shop or LCSC C65114 - or a 2x3 pogo-pin ISP adapter instead (contact NOT_VERIFIED) |
| 1x2 2.54 mm pin header + shunt (reset hold) | 3 | 6 | J24, J25, J26 | domestic shop or LCSC C55629809 + C100114 |
| USBasp programmer (target power jumper OPEN) | 1 | 1 | J19..J22 (ISP of the four MCUs) | domestic shop |
| USB-serial adapter, 3.3 V logic | 1 | 1 | J3 (ESP-12F flashing, 3.3 V only) | domestic shop |
| 3 mm foam tape | 1 | 2 | board underside (after the THT leads are flush-cut) | domestic shop |
| cable tie | 1 | 2 | across the tie band x -3..+3 mm | domestic shop |

## 7. 보드를 받은 뒤 할 일 (계획 15장 순서)

1. **검사:** 탄탈 C45 / C52와 전해 C47의 극성을 실크 표시와 비교, U7 / U8 / U9 / J7 방향, U8(0.5 mm 피치) 납땜 브리지가 없는지 확대해서 봅니다.
2. **레일 떼기:** Standard 주문이면 보드 둘레의 레일을 꺾어 떼고 모서리를 다듬습니다.
3. **뒷면 다리 자르기:** 뒷면으로 나온 다리를 재서 1.5 mm 보다 긴 것은 모두 자릅니다. J15, J17 은 반드시 (다리 4개). 소켓·핀헤더(J2, J5, J8~J12, J16, J18)도 공차 끝이면 넘을 수 있으니 재 봅니다. 폼테이프는 그 뒤에 붙입니다.
4. **모듈·MCU 없이 벤치 전원 시험:** J14에 전류 제한(100 mA) 전원, J15는 단락. 아래 값을 적습니다. 범위를 벗어나면 멈춥니다.

| 측정 | 허용 범위 (E, 계획) | 측정값 |
|---|---|---|
| 전압을 천천히 올릴 때 5 V가 켜지는 전압 | 3.44 ~ 3.73 V (3.3 V에서는 꺼져 있어야 함) | |
| 켜진 뒤 +5V | 4.84 ~ 5.33 V | |
| 켜진 뒤 +3V3 | 3.0 V 이상 (ESP-12F 공급 범위, F) | |
| 전압을 천천히 내릴 때 꺼지는 전압 | 3.00 ~ 3.25 V | |

   켜지는 순간의 +5V와 EN 파형을 오실로스코프로 한 장 남깁니다.
5. **부하 시험:** +5V에 0.27 A와 0.78 A, 전원 3.4 V와 3.7 V (전류 제한 2 A). 전압이 유지되고 U8 / L5가 약 60 C 아래인지 봅니다 (E, 계획 15장). 실제 배터리·스위치·리드로 배터리에서 U8 입력까지 저항을 잽니다 (전압 강하 / 입력 전류): 0.29 Ohm 이하여야 합니다. 넘으면 비행 전에 보고합니다. (0장의 배선 폭 문제 때문에 이 측정이 특히 중요합니다.)
6. **MCU 4개 납땜** (JLC가 안 붙였을 때): 1번 핀을 표시에 맞추고, 가는 인두·플럭스로 0.8 mm 피치 핀을 모두 확대 검사합니다. 패드 1 mm 안에 이미 붙은 부품 수: U1 14, U2 14, U3 20, U4 15 (M). U1을 GY-521보다 먼저 납땜하십시오 (GY-521이 U1 위에 걸침). 그림: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/c5_mcu_pin1.png`
7. **모듈 장착:** 각 모듈에 인쇄된 핀 순서를 보드 실크와 먼저 비교합니다. GY-521은 자기 1x8 핀으로 J1에 납땜, 받침 2.54 mm (A) (`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e2a_gy521_J1.png`); BMP280은 J2에 꽂음 (헤더 직각이 아니면 직각 핀을 먼저 납땜, `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e2b_bmp280_J2.png`); OpenLog는 직각 6핀 헤더를 납땜한 뒤 microSD 쪽이 위로 오게 J5에 꽂음 (`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e2c_openlog_J5.png`); NEO-7M은 윗뚜껑에 두고 JST-GH 케이블을 J7에 (`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e3_gps_lead_J7.png`). 받은 상태와 완성 상태 비교: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e1_delivered_vs_completed.png`, 전체 안내: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e4_overview_callouts.png`.
8. **프로그래밍 (전원은 한 가지만):** 로직 배터리(3.73 V 넘게 충전) 또는 J14에 4.0 V 벤치 전원으로 보드를 켜고, USBasp의 타깃 전원 점퍼는 **빼 둡니다** (5 V를 두 곳에서 넣지 않기). 단자는 D6 선택대로 (헤더 납땜 + 점퍼, 또는 포고핀 어댑터). ESP-12F는 J3로 3.3 V 신호로만 굽습니다.
9. 첫 Wi-Fi 송신 때 +5V와 +3V3의 최저값을 기록합니다.
10. 커넥터 연결: 배터리 J14, 스위치 J15, GPS J7, 서보 J8~J11 / ESC J12, 아밍 J16, 부저 J17. 장착은 rev 2c와 같이 3 mm 폼테이프 + 케이블타이 1개 (x -3..+3 mm 띠).

## 8. MCU·센서 말고 사용자가 해야 하는 일 (숨기지 않고 모음)

요청은 'MCU와 센서만 빼고 모두 JLC가 실장'이었지만, 아래 일은 이 보드에서 사용자 몫으로 남습니다:

- Standard 주문의 레일 떼기 (7장 2).
- 뒷면 다리 자르기: J15, J17 (다리 4개, 도면값으로 규칙 초과), 공차 끝인 소켓·헤더 확인 (7장 3).
- 프로그래밍 단자 (A안): J3, J19~J22 (2x3) 와 J24~J26 (1x2)는 빈 구멍입니다. 헤더를 직접 납땜(36곳, 계획 D6)하고 점퍼 3개를 끼우거나, 포고핀 어댑터를 대고 굽습니다 (접촉 NOT_VERIFIED). JLC가 띠 밖에 헤더를 실장하는 B안은 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) (SMD 0곳, THT 0곳).
- OpenLog에 직각 6핀 헤더 납땜 (모듈 쪽 6곳; 보드에는 납땜 없음). OpenLog는 센서가 아니라 기록기라 D5에서 '센서처럼 사용자가 장착'으로 정했습니다.
- BMP280 헤더가 일자형으로 오면 직각 핀으로 바꿔 납땜.
- GPS 케이블(JST-GH 4선), 스위치·부저·아밍 리드 준비 (6장 목록).
- 벤치 전원 시험과 기록 (7장 4~5) - 보드가 만들어진 적이 없어서 필요합니다.
- 폼테이프와 케이블타이로 장착.

## 9. 사용자 결정 D1~D17과 이번 파일에 쓴 값

계획의 기본값으로 만들었습니다. '이번 파일에서 확인한 값'은 생성된 파일에서 읽은 것입니다.

| Id | 질문과 기본값 (계획 19장) | 이번 파일에서 확인한 값 |
|---|---|---|
| D1 | 5 V 변환 회로 방식 (기본: A안 TPS61023 승압 + TLV3012B 차단) | U8 C919459, U9 C20345924, L5 C53426685 (BOM) |
| D2 | 배터리 차단 전압 (기본: 켜짐 3.59 V / 꺼짐 3.13 V(공칭) 유지) | R65 470k 1%, R66 270k 1%, R67 3.3M 1% → 켜짐 3.44 ~ 3.73 V, 꺼짐 3.00 ~ 3.25 V (E) |
| D3 | 요청과 무관하게 바꾼 부품 번호 3건을 유지할지 (기본: 유지(재고가 더 많거나 도면이 확인된 부품)) | C45 C11366, J8 C32713262, J2 / J5 C22436147 (BOM) |
| D4 | MCU를 누가 붙일지 (기본: 직접 납땜(요청대로). 권장은 JLC 실장) | 기본 BOM에 U1~U4 없음; with_mcu 쌍 U1~U4 = C6510 (두 쌍 모두 생성) |
| D5 | OpenLog 장착 방법 (기본: 헤더를 납땜해 J5에 꽂기. 안 맞으면 6선 케이블) | 소켓 J5 C22436147 (JLC), OpenLog는 따로 구매 |
| D6 | 프로그래밍 단자 (기본: A안 빈 구멍(직접 납땜 36핀 또는 포고핀). B안은 가능 여부 보고 후 결정) | A안 (빈 구멍); B안 검색 SMD 0곳, THT 0곳 → 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) |
| D7 | J15/J17 커넥터 (기본: 스루홀, 다리 4개 직접 절단) | J15 / J17 C131337 (THT); 다리 자르기 J15, J17 |
| D8 | 레벨시프터 FET (기본: C40912 유지) | Q1 등 C40912 (BOM) |
| D9 | 주문 방식 (기본: Standard(권장과 같음)) | 주문 화면에서 사용자가 선택 (2장: Standard) |
| D10 | 툴링 홀 (기본: 설계에 넣지 않음. JLC 레일에 생김) | 툴링 홀 부품 0개 (설계에 없음) |
| D11 | 수량 (기본: PCB 5장, 실장 2장) | 주문 화면에서 사용자가 선택 (2장: PCB 5 / 실장 2) |
| D12 | 모듈 소켓 J2/J5 (기본: C22436147, 주문일 재고 부족 시 C55629861, 그다음 C54876735) | J2 / J5 C22436147; 대체 순서는 5장 |
| D13 | 배터리 전압 분압 저항 (기본: 그대로. 차단 후에도 약 40 uA가 흐르므로 스위치를 꺼야 함) | R15 47k 1%, R16 47k 1% (그대로) |
| D14 | 흡습 부품 베이킹 요청 (기본: 요청함(수수료 미확인)) | 주문 메모에 베이킹 요청 (2장) |
| D15 | 모듈 치수 (기본: 가정값 사용, 그림에 '가정' 표시. 실측을 권장) | 가정 치수 사용 (spec/module_dimensions.json, 그림과 검사에 '가정' 표시) |
| D16 | 표면처리·색상 등 (기본: ENIG, 녹색, JLC 기본값(추가 요금 미확인)) | 주문 화면에서 사용자가 선택 (2장: ENIG, 녹색, 기본 적층) |
| D17 | 실장하지 않은 여분 보드 (기본: 기구 확인용으로만 사용) | 여분 기판 3장은 기구 확인용 (주문 화면 수량으로 정해짐) |
| 추가 (배선 후 발견) | 배터리 경로 폭 규칙 FAIL을 받아들이고 주문할지, 고친 뒤 주문할지 (0장) | 결정 전: 도구 판정 NOT ORDERABLE |

## 10. 변경 범위 표 (rev 2c 대비, 부품마다 이유 분류)

계획 3장의 표를 계획 파일에서 읽고, rev 3 부품 번호를 주문 BOM 파일과 대조했습니다 (교체 11, 추가 13, 삭제 1, 그대로 (부품 없음) 10, 그대로 140). 그대로인 부품은 표에서 뺐습니다.

| Ref | 변경 | rev 2c | rev 3 | 누가 붙임 | 분류 (태그) | BOM 파일과 대조 |
|---|---|---|---|---|---|---|
| C45 | 교체 | 22u 10V tantalum / C7183 | 22u 10V tantalum (TAJA226K010RNJ, +-10 %) / C11366 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J2 | 교체 | 1x6 2.54 mm right-angle socket (module lies flat) / C55629861 | 1x6 2.54 mm right-angle socket (module lies flat) / C22436147 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J3 | 그대로 (부품 없음) | 2x3 2.54 mm vertical: ESP-12E programming header (3.3 V logic ONLY) / C65114 | 2x3 2.54 mm vertical: ESP-12E programming header (3.3 V logic ONLY) / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J5 | 교체 | 1x6 2.54 mm right-angle socket: OpenLog lies FLAT beside it (outline reserved) / C55629861 | 1x6 2.54 mm right-angle socket: OpenLog lies FLAT beside it (outline reserved) / C22436147 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J6 | 삭제 | 1x5 2.54 mm: Pololu S9V11F5S6CMA 5 V step-up/step-down module (VOUT, GND, VIN, EN, PG) / no=15855364 | - | - | 필수: 요청(모든 부품을 JLC가 실장) 때문에 | BOM에 없음 (맞음) |
| J8 | 교체 | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / C32713262 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J9 | 교체 | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / C32713262 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J10 | 교체 | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / C32713262 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J11 | 교체 | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / C32713262 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J12 | 교체 | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / C32713262 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J16 | 교체 | 1x2 right-angle header / C55629816 | 1x2 right-angle header / C32713261 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J18 | 교체 | 1x4 right-angle header / C55629818 | 1x4 right-angle header / C32713263 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J19 | 그대로 (부품 없음) | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J20 | 그대로 (부품 없음) | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J21 | 그대로 (부품 없음) | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J22 | 그대로 (부품 없음) | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J24 | 그대로 (부품 없음) | 2-pin header + shunt / C55629809 | 2-pin header + shunt / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J25 | 그대로 (부품 없음) | 2-pin header + shunt / C55629809 | 2-pin header + shunt / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J26 | 그대로 (부품 없음) | 2-pin header + shunt / C55629809 | 2-pin header + shunt / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| Q3 | 그대로 (부품 없음) | BSS138 / C40912 | BSS138 / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| R47 | 그대로 (부품 없음) | 10k / C25804 | 10k / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| U7 | 교체 | ESP-12E (ESP8266EX + 4 MB SPI flash, PCB antenna) / C89297 | ESP-12F (ESP8266MOD, 4 MB flash, PCB antenna) / C82891 | JLC | 조달: JLC 주문 방식·재고 때문에 (핀 호환 부품) | 일치 |
| U8 | 추가 | - | TPS61023DRLR / C919459 | JLC | 필수: 요청(모든 부품을 JLC가 실장) 때문에 | 일치 |
| U9 | 추가 | - | TLV3012BIDBVR / C20345924 | JLC | rev 2c 기능 유지: Pololu 모듈 안에 있던 저전압 차단 (D2에서 뺄 수 있음) | 일치 |
| L5 | 추가 | - | 1.0 uH FXL0420-1R0-MA1 / C53426685 | JLC | 필수: 요청(모든 부품을 JLC가 실장) 때문에 | 일치 |
| C57 | 추가 | - | 22u / C12891 | JLC | 필수: 요청(모든 부품을 JLC가 실장) 때문에 | 일치 |
| C58 | 추가 | - | 1n 50V X7R / C1588 | JLC | 필수: 요청(모든 부품을 JLC가 실장) 때문에 | 일치 |
| C59 | 추가 | - | 100n / C14663 | JLC | rev 2c 기능 유지: Pololu 모듈 안에 있던 저전압 차단 (D2에서 뺄 수 있음) | 일치 |
| C60 | 추가 | - | 100n / C14663 | JLC | rev 2c 기능 유지: Pololu 모듈 안에 있던 저전압 차단 (D2에서 뺄 수 있음) | 일치 |
| R63 | 추가 | - | 150k 1% / C22807 | JLC | 필수: 요청(모든 부품을 JLC가 실장) 때문에 | 일치 |
| R64 | 추가 | - | 20k 1% / C4184 | JLC | 필수: 요청(모든 부품을 JLC가 실장) 때문에 | 일치 |
| R65 | 추가 | - | 470k 1% / C23178 | JLC | rev 2c 기능 유지: Pololu 모듈 안에 있던 저전압 차단 (D2에서 뺄 수 있음) | 일치 |
| R66 | 추가 | - | 270k 1% / C22965 | JLC | rev 2c 기능 유지: Pololu 모듈 안에 있던 저전압 차단 (D2에서 뺄 수 있음) | 일치 |
| R67 | 추가 | - | 3.3M 1% / C26096 | JLC | rev 2c 기능 유지: Pololu 모듈 안에 있던 저전압 차단 (D2에서 뺄 수 있음) | 일치 |
| R68 | 추가 | - | 1M 1% / C22935 | JLC | rev 2c 기능 유지: Pololu 모듈 안에 있던 저전압 차단 (D2에서 뺄 수 있음) | 일치 |

## 11. 이 문서의 숫자 출처

| 값 이름 | 값 | 출처 파일 | 표시 |
|---|---|---|---|
| 출고 판정 | NOT ORDERABLE | out/manifest.json release_gate | M (도구 검사) |
| 출고 판정 이유 | routing_rules FAIL (battery_path) | out/manifest.json release_gate.reasons | M |
| 2 mm보다 좁은 길이 | 16.1 mm | out/reports/routing_rules.json battery_path | M |
| 좁은 구간 | VBAT_LOGIC_RAW 1.07 mm (폭 1 mm); VBAT_LOGIC 3.93 mm (폭 1 mm); VBAT_LOGIC_SW 11.1 mm (폭 1.5/1.56 mm) | out/reports/routing_rules.json battery_path.legs | M |
| 배터리 경로 저항 20C | 0.03231 Ohm | out/reports/routing_rules.json battery_path | E |
| 배터리 경로 목표 | 0.035 Ohm | out/reports/routing_rules.json battery_path | F (계획 13절) |
| 배터리 경로 저항 60C | 0.03739 Ohm | out/reports/routing_rules.json battery_path | E |
| zip main sha256 | 9cea8391a5c3da90... | out/manifest.json fab.sets | M |
| jlcpcb_bom.csv sha256 | fa4c8e2622885f4d... | out/manifest.json sha256 | M |
| jlcpcb_cpl.csv sha256 | 2dbd7ed2d8b13150... | out/manifest.json sha256 | M |
| zip with_mcu sha256 | 3c8df77ed79e4aaf... | out/manifest.json fab.sets | M |
| jlcpcb_bom_with_mcu.csv sha256 | 465199b917ff914d... | out/manifest.json sha256 | M |
| jlcpcb_cpl_with_mcu.csv sha256 | 346adddc4f12d4b0... | out/manifest.json sha256 | M |
| F.Paste 패드 main | 347 | out/manifest.json fab.sets | M |
| F.Paste 패드 with_mcu | 603 | out/manifest.json fab.sets | M |
| main bom_lines | 40 | out/manifest.json bom_cpl.pairs.main | M |
| main bom_lines_smt | 34 | out/manifest.json bom_cpl.pairs.main | M |
| main bom_lines_tht | 6 | out/manifest.json bom_cpl.pairs.main | M |
| main parts | 157 | out/manifest.json bom_cpl.pairs.main | M |
| main cpl_rows | 157 | out/manifest.json bom_cpl.pairs.main | M |
| with_mcu bom_lines | 41 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| with_mcu parts | 161 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| with_mcu cpl_rows | 161 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| U1-U4 기본 BOM | 없음 | out/jlcpcb_bom.csv | M |
| 층 수 | 4 | spec/carrier_spec.json | F (계획 불변조건 d) |
| 외곽 | 76 x 70 mm | spec/carrier_spec.json | F (size) / A (corner radius) |
| 두께 | 1.6 mm | spec/carrier_spec.json | F |
| 가장자리 2.5 mm 안 몸체 | 32 | out/reports/edge_distance.json | M |
| 가장자리 0.3 mm 안 몸체 | U7 0.15 mm, J15 0.29 mm | out/reports/edge_distance.json | M |
| 레일 쪽 | +y / -y | out/reports/edge_distance.json rail_sides_for_standard_order | E (표에서 고름) |
| 레일 없는 쪽 2.5 mm 안 | 14 | out/reports/edge_distance.json rail_pairs | M |
| 툴링 홀(H) 부품 | 0 | spec/carrier_spec.json summary.by_prefix | M |
| JLC 확인 시각 | 2026-11-11T16:19:27+09:00 | out/jlc_gate_report.json | M (그 시각의 재고) |
| JLC 확인 결과 | PASS (40/40줄), with_mcu PASS | out/jlc_gate_report.json | M |
| THT 납땜점 main | 39 | out/jlc_fee_estimate.csv | M |
| 회전 미확인 | C45, C47, C52, U9 | out/reports/cpl_rotation.json | NOT_VERIFIED |
| 비용 main Economic 2장 | 80.24 USD | out/jlc_fee_estimate.csv sum_E | E |
| 비용 main Standard 2장 (Standard 전용 수수료 포함) | 154.40 USD | out/jlc_fee_estimate.csv | E (적용 여부 NOT_VERIFIED) |
| 비용 main Standard 2장 | 122.55 USD | out/jlc_fee_estimate.csv sum_E | E |
| 비용 main Standard 5장 (Standard 전용 수수료 포함) | 184.41 USD | out/jlc_fee_estimate.csv | E (적용 여부 NOT_VERIFIED) |
| 비용 main Standard 5장 | 152.56 USD | out/jlc_fee_estimate.csv sum_E | E |
| 비용 with_mcu Standard 2장 (Standard 전용 수수료 포함) | 204.83 USD | out/jlc_fee_estimate.csv | E (적용 여부 NOT_VERIFIED) |
| 비용 with_mcu Standard 2장 | 172.98 USD | out/jlc_fee_estimate.csv sum_E | E |
| Standard 전용 수수료 | 31.85 USD | out/jlc_fee_estimate.csv | E |
| 얇은 재고 기준 | 1000개 | out/jlc_gate_report.json | F (계획 I7) |
| 얇은 재고 줄 | C22436147 (J2,J5) available 503 | out/jlc_gate_report.json | M |
| 재고 주의 줄 | C53426685 (L5) 1065개, 필요 22, C20345924 (U9) 1088개, 필요 20, C22436147 (J2,J5) 503개, 필요 40 | out/jlc_gate_report.json lines (available < 2000) | M |
| 대체 J2, J5 (C22436147) | C55629861 SH-PM254R-1x6P-H85 (985개, PASS); C54876735 PM2.54-1x6PWZ-H8.5 (595개, PASS) | out/jlc_gate_report.json alternates_and_options | M |
| 대체 L5 (C53426685) | C167203 FXL0420-1R0-M (106884개, PASS) | out/jlc_gate_report.json alternates_and_options | M |
| 대체 J7 (C161692) | C52037397 GH1.25-4PLT (1843개, PASS); C2829262 GH1.25-4PLTPZ (55720개, PASS) | out/jlc_gate_report.json alternates_and_options | M |
| 대체 U8 (C919459) | C1852149 TPS61023DRLT (526개, PASS) | out/jlc_gate_report.json alternates_and_options | M |
| 대체 J8..J12, J16, J18 (THT right-angle headers) if JLC refuses hand soldering | C46061676 HX PZ2.54-1x3P WT (20111개, PASS); C46061675 HX PZ2.54-1x2P WT (18761개, PASS); C46061677 HX PZ2.54-1x4P WT (13006개, PASS) | out/jlc_gate_report.json alternates_and_options | M |
| 대체 J15, J17 (C131337) | C160352 B2B-PH-SM4-TB(LF)(SN) (46998개, PASS) | out/jlc_gate_report.json alternates_and_options | M |
| 따로 살 품목 수 | 19 | out/off_board_purchase.csv | M |
| 뒷면 다리 규칙 | 1.5 mm | out/fit_checks.json tht_leads | F (계획) |
| 다리 규칙 초과 | J15, J17 | out/fit_checks.json tht_leads | F (도면값) |
| 켜짐 전압 | 3.44 ~ 3.73 V | spec/rev3_plan.json power_block.uvlo | E |
| +5V 범위 | 4.84 ~ 5.33 V | spec/rev3_plan.json power_block.vout_V | E |
| 꺼짐 전압 | 3.00 ~ 3.25 V | spec/rev3_plan.json power_block.uvlo | E |
| MCU 1 mm 안 부품 | U1 14, U2 14, U3 20, U4 15 | out/fit_checks.json mcu_access | M |
| GY-521 받침 높이 | 2.54 mm | spec/module_dimensions.json gy521.support_h | A |
| BMP280 헤더 종류 | 직각 | spec/module_dimensions.json bmp280.header_type | A |
| 프로그래밍 헤더 선택 | A | spec/carrier_spec.json prog_headers.option | M |
| B안 결론 | 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) | out/reports/programming_access.json | M |
| B안 2x3 자리 | SMD 0곳, THT 0곳 | out/reports/programming_access.json verdict_per_family | M |
| 코드 U8 | C919459 | out/jlcpcb_bom.csv | M |
| 코드 U9 | C20345924 | out/jlcpcb_bom.csv | M |
| 코드 L5 | C53426685 | out/jlcpcb_bom.csv | M |
| UVLO 저항 | R65 470k 1%, R66 270k 1%, R67 3.3M 1% | spec/carrier_spec.json | M |
| 코드 C45 | C11366 | out/jlcpcb_bom.csv | M |
| 코드 J8 | C32713262 | out/jlcpcb_bom.csv | M |
| 코드 J2 | C22436147 | out/jlcpcb_bom.csv | M |
| U1-U4 with_mcu | C6510 | out/jlcpcb_bom_with_mcu.csv | M |
| 코드 J5 | C22436147 | out/jlcpcb_bom.csv | M |
| 코드 J15 | C131337 | out/jlcpcb_bom.csv | M |
| 코드 Q1 | C40912 | out/jlcpcb_bom.csv | M |
| 분압 저항 | R15 47k 1%, R16 47k 1% | spec/carrier_spec.json | M |
| 변경 행 | 교체 11, 추가 13, 삭제 1, 그대로 (부품 없음) 10, 그대로 140 | spec/rev3_plan.json changes | F (계획) |
