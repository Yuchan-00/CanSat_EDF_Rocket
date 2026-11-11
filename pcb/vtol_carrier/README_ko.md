# VTOL 아비오닉스 캐리어 PCB rev 3 (JLC가 실장해서 보내는 판)

**이 보드는 아직 만들거나 전원을 넣어 본 적이 없습니다 (NOT_VERIFIED).** 아래의 PASS는 모두 도구(kicad-cli ERC/DRC, 스크립트)의 결과이며 보드가 동작한다는 뜻이 아닙니다. JLC가 이 파일을 받아 주는지도 업로드 후 JLC의 DFM 결과와 견적을 봐야 압니다.

표시: M = 이 PC에서 도구로 측정, E = 추정(계산), A = 가정(실물로 확인 필요), F = 문서·데이터시트 근거. 근거가 없으면 NOT_VERIFIED. 이 문서는 `scripts/docs_ko.py` 가 보고서 파일에서 숫자를 읽어 만든 것입니다 (손으로 고치지 말고 다시 생성). 측정·계산된 숫자의 출처는 맨 끝 8장에 있고, 그 표에 없는 숫자(칩 종류, 규칙 값, 시험 조건)는 계획서 `spec/REV3_PLAN.md` 와 사양 `spec/carrier_spec.json` 에서 옮긴 고정값(F)입니다.

## 1. 지금 상태

**출고 판정(도구 검사 기준): NOT ORDERABLE** - 이유: routing_rules FAIL (battery_path).

배선이 끝난 보드에서 배터리 전원 경로(J14 → Q10 → J15 → L5)의 구리 저항은 0.03231 Ohm (20 C, E)로 계획 목표 0.035 Ohm 를 만족하지만, 계획이 요구한 '2.0 mm 이상 폭'을 16.1 mm 구간에서 지키지 못했습니다 (VBAT_LOGIC_RAW 1.07 mm (폭 1 mm); VBAT_LOGIC 3.93 mm (폭 1 mm); VBAT_LOGIC_SW 11.1 mm (폭 1.5/1.56 mm)). 60 C에서는 0.03739 Ohm (E)로 목표를 넘습니다. 도구는 이 규칙을 느슨하게 하지 않았습니다. 그대로 주문할지, 배선을 고칠지는 사용자가 정할 일입니다 (`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 0장).

| 항목 | 결과 | 표시 |
|---|---|---|
| 회로도 ERC | 오류 0, 경고 0 (시트 8장) | M |
| PCB DRC (`--schematic-parity --severity-all --refill-zones`) | 위반 0, 경고 0, 미연결 0, 회로도 불일치 0 | M |
| 넷리스트 = 사양 | PASS (부품 174, 네트 105) | M |
| 보드 패드 넷 = IR | PASS (핀 506 / 506) | M |
| MCU 4개 핀 = rev 2c | PASS (핀 256, 차이 0) | M |
| U8 / U9 핀-네트 표 | PASS | M (표는 F) |
| 배선 규칙 (계획 13장) | 11 / 12 PASS, FAIL: battery_path | M / E |
| 크리스털 XTAL1 길이 (rev 2c보다 나빠지면 안 됨) | 5.96 mm (한계 5.96 mm) | M |
| 설계 규칙 파일 = rev 2c | PASS | M |
| 구리 섬 / 패드에 닿는 비아 | 없음 (모든 네트 한 덩어리) / 0 | M |
| JLC 부품 재고·주문 가능 확인 | PASS (40/40줄), with_mcu PASS, 2026-11-11T16:19:27+09:00 | M (그 시각 기준) |
| 주문 파일 형식·fit 집합 검사 | PASS | M |
| 주문 파일을 두 번 써서 같은지 | PASS | M |
| 전체 다시 만들기 두 번 = 같은 파일 | FAIL - 바이트 비교 파일 70개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 1개 (out/vtol_carrier.step: kicad-cli가 STEP을 내보낼 때마다 항목 순서가 바뀜. 번호·순서를 뺀 내용 비교는 같음) | M |
| 가장자리 2.5 mm 안 부품 몸체 | 32개 (0.3 mm 안: U7 0.15 mm, J15 0.29 mm) | M |
| 모듈(OpenLog / BMP280 / GY-521) 간섭 | A / NOT_VERIFIED, 가정 치수로 충돌 0건 | A / NOT_VERIFIED |

## 2. 이 기판은 무엇인가

- 크기 76 x 70 mm, 두께 1.6 mm, 4층 (F.Cu 신호와 부품 / In1 GND 전면 / In2 +5V 전면 / B.Cu 신호). 부착 구멍 없음: 3 mm 폼테이프 + 케이블타이 1개. 케이블타이 띠(x -3..+3 mm)에는 실장 부품이 없습니다.
- 부품은 모두 윗면에만 있습니다. 부품 높이 한계는 기판 윗면에서 13.4 mm 입니다. 아랫면은 폼테이프 자리라 비어 있습니다.
- 비행 컴퓨터 칩 4개: U1 NAV, U2 FCU, U3 COM, U4 GNS (ATmega128A-AU, 16 MHz, 5 V). 무선은 U7 ESP-12F (Wi-Fi 중계).
- 센서: GY-521 (IMU, J1에 납땜), BMP280 (기압, 소켓 J2), NEO-7M GPS (기체 윗뚜껑, 케이블로 J7). 기록기 OpenLog는 소켓 J5.
- 전원 흐름: 1S 로직 배터리 J14 → Q10 역극성 보호 → 스위치 리드 J15 → U8 승압 5 V (+ U9 저전압 차단) → +5V 면 → U6 → +3V3.

블록 (회로도 시트 기준, 부품 수는 사양에서 셈):

| 블록 | 내용 | 부품 자리 | 그중 JLC 실장 |
|---|---|---|---|
| PWR | 전원: 1S 로직 배터리 입력 J14, 역극성 보호 Q10, 스위치 리드 J15, 5 V 승압 U8 TPS61023 + L5, 저전압 차단 U9 TLV3012B, 3.3 V 레귤레이터 U6, 접지 연결 NT1, 배터리 전압 분압 | 27 | 26 |
| NAV | NAV 칩 U1 ATmega128A + GY-521 IMU 자리 J1 + BMP280 소켓 J2 | 17 | 14 |
| FCU | FCU 칩 U2 ATmega128A | 15 | 13 |
| COM | COM 칩 U3 ATmega128A + ESP-12F Wi-Fi 모듈 U7, 레벨 시프터, 부트 스트랩, ESP 굽기 단자 J3 | 36 | 32 |
| GNS | GNS 칩 U4 ATmega128A + GPS 리드 커넥터 J7 (NEO-7M) | 18 | 15 |
| SHIFT | BSS138 레벨 시프터 5 V <-> 3.3 V | 18 | 16 |
| IO | 서보 J8~J11, ESC J12, 아밍 스위치 J16, 부저 J17, 6S 전압 감지 J18 등 입출력 커넥터 | 37 | 37 |
| LOG | OpenLog 소켓 J5 | 1 | 1 |
| LINK | 칩 사이 연결선 (UART / SPI) 부품 | 5 | 3 |

## 3. rev 2c에서 바뀐 것과 이유

- **5 V 전원 모듈을 없앴습니다.** 꽂는 Pololu 모듈(J6, 디바이스마트 부품)은 JLC가 실장할 수 없어서, JLC 부품으로 만든 승압 회로 U8 TPS61023 (C919459) + L5 (C53426685)를 예전 모듈 자리(+x / -y 모서리)에 올렸습니다. 이유: 요청(모든 부품 JLC 실장).
- **저전압 차단을 유지했습니다.** 예전 모듈 안에 있던 기능입니다. U9 TLV3012B (C20345924) + R65 470k 1%, R66 270k 1%, R67 3.3M 1%: 켜짐 3.44 ~ 3.73 V, 꺼짐 3.00 ~ 3.25 V (계산값 E, 실측 전).
- **Wi-Fi 모듈 ESP-12E → ESP-12F** (C82891). 핀과 풋프린트가 같고, ESP-12E는 JLC에서 재고가 적고 Standard 주문에만 쓸 수 있었습니다.
- **스루홀 커넥터도 JLC가 납땜합니다.** 소켓 J2 / J5 (C22436147), 서보 헤더 J8~J12 (C32713262), J16, J18, 배터리 J14 (C5364642), 스위치·부저 J15 / J17 (C131337). 탄탈 C45 (C11366)와 헤더·소켓 번호 일부는 재고나 도면 근거로 바꿨고 D3 / D12에서 rev 2c 번호로 되돌릴 수 있습니다.
- **그대로인 것:** MCU 4개의 핀 256개가 가는 네트 (PASS (핀 256, 차이 0)), 외곽, 커넥터·MCU·크리스털·모듈 소켓·ESP 위치와 방향, 설계 규칙 (PASS), 크리스털 배선 (5.96 mm (한계 5.96 mm)).
- **배치·배선을 다시 했습니다.** 새 부품 13개를 넣느라 주변 수동 부품 일부가 옮겨졌고, 배선은 새로 했습니다 (세그먼트 2308개, 비아 327개 중 GND 스티칭 102개). 배선은 Freerouting 실행 기록(세션 파일)을 다시 넣는 방식으로 재현됩니다.
- 부품마다 바뀐 내용과 이유 분류(태그)의 전체 표: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 10장 (교체 11, 추가 13, 삭제 1, 그대로 (부품 없음) 10, 그대로 140).

## 4. 누가 무엇을 붙이나

| 구분 | 뜻 | 개수 |
|---|---|---|
| jlc | JLC가 실장 (주문 BOM / CPL에 들어감) | 157 |
| jlc_optional | 기본은 사용자 납땜, with_mcu 파일 쌍을 쓰면 JLC 실장 | 4 |
| user | 사용자가 납땜 | 1 |
| none | 부품 없음 (빈 도금 구멍 / 미실장 / 구리 패턴만) | 12 |

- JLC가 붙이는 것: 기본 파일 쌍 기준 BOM 40줄 (SMT 34 + THT 6), 부품 157개. MCU까지 맡기는 with_mcu 쌍은 41줄 / 161개 (U1~U4 = C6510).
- 사용자가 붙이는 것: ATmega128A-AU 4개 (기본, with_mcu 쌍이면 JLC), GY-521 (J1 구멍에 모듈 핀을 납땜), BMP280 (J2에 꽂음), OpenLog (모듈에 직각 6핀 헤더를 납땜한 뒤 J5에 꽂음), NEO-7M (케이블로 J7).
- 부품 없이 비워 둔 자리: 프로그래밍 단자 J3, J19~J22 (2x3), J24~J26 (1x2) - 현재 선택 A안 (빈 도금 구멍). JLC가 헤더를 띠 밖에 실장하는 B안은 배치 검색 결과 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) (SMD 0곳, THT 0곳).
- MCU·센서 말고 사용자가 해야 하는 일(레일 떼기, 다리 자르기, 프로그래밍 단자 등)은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 8장에 모았습니다.

## 5. 확인되지 않은 것 (NOT_VERIFIED)

- 배터리 경로 폭 규칙 FAIL (16.1 mm): 사용자 결정 전까지 도구 판정은 NOT ORDERABLE 입니다.
- 보드 동작 전체: 만들거나 전원을 넣은 적이 없습니다. 전원부 수치(5 V 출력, 차단 전압, 전류 여유)는 데이터시트 계산(E)입니다.
- JLC의 수락: 가장자리 가까운 부품, THT 6줄 손납땜, BOM / CPL 형식, Standard 주문의 레일은 업로드 후 DFM 결과와 견적으로만 확인됩니다.
- CPL 회전: 방향이 중요한 43개 중 C45, C47, C52, U9 은 자료끼리 어긋나 NOT_VERIFIED입니다. 나머지도 JLC 미리보기로만 확정됩니다. 탄탈 C45 / C52, 전해 C47을 거꾸로 붙이면 화재 위험이 있습니다.
- 리플로 온도: Economic(255 C)에서 한계에 걸리는 줄 C2833309, C2286, C53426685; 문서에서 정격을 찾지 못한 부품 번호 32개.
- 뒷면 다리 길이: J15, J17 은 규칙 1.5 mm 보다 길어 잘라야 합니다. J1, J14 은 확인 못 함.
- 모듈 치수와 끼움: GY-521, BMP280, OpenLog, NEO-7M 치수는 가정값입니다 (GY-521 받침 2.54 mm, BMP280 헤더 직각 - 모두 A). 그림과 간섭 검사도 이 가정 위에 있습니다. 실물을 버니어로 재야 확정됩니다 (D15).
- J14 Molex 커넥터: 라이브러리 3D 모델이 없고 구멍 지름을 도면과 대조하지 못했습니다.
- 재고: 2026-11-11T16:19:27+09:00 기준입니다. 주문 직전에 scripts/jlc_gate.py 를 다시 돌려야 합니다.
- 비용: 수수료 표로 계산한 추정(E)입니다. 실제 금액은 JLC 견적뿐입니다.
- GPS 케이블이 기체 안에서 지나갈 길은 설계되지 않았습니다 (그림은 예시).

## 6. 다시 만드는 방법

작업 폴더 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts` 에서 (환경변수 `PYTHONDONTWRITEBYTECODE=1`):

```
B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py           # 전체 (사양 -> 회로도 -> 배치 -> 배선 재생 -> 검사 -> 주문 파일 -> 그림 -> 문서 -> manifest)
B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py --list    # 단계 목록
B:/Claude/.venv/Scripts/python.exe -X utf8 -B determinism.py        # 전체를 두 번 만들어 파일이 바이트까지 같은지 비교
B:/Claude/.venv/Scripts/python.exe -X utf8 -B jlc_gate.py           # 주문 당일 JLC 재고 확인 (인터넷, 읽기만)
```

- 한 번 만드는 데 걸린 시간: 10.7분, 11.2분 (determinism.py의 두 빌드, M). 마지막 결정성 검사: 2026-11-11T17:53:25+0900.
- make_all.py는 인터넷을 쓰지 않습니다. JLC 부품 근거는 저장된 기록으로 다시 평가하고, 재고 확인(jlc_gate.py)은 따로 실행합니다.
- 배선은 기록된 Freerouting 세션을 다시 넣어 같은 보드를 만듭니다. 배치가 바뀌면 route.py가 재생을 거부하고, 새 배선(`make_all.py --reroute`)이 필요합니다. 새 배선은 실행마다 결과가 달라질 수 있습니다.
- 결정성 검사 결과: FAIL - 바이트 비교 파일 70개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 1개 (out/vtol_carrier.step: kicad-cli가 STEP을 내보낼 때마다 항목 순서가 바뀜. 번호·순서를 뺀 내용 비교는 같음). 비교 대상은 회로도, 보드, BOM, CPL, 거버와 zip, 회전 표, 3D 모델, 사양, 이 문서와 주문 안내서이고, STEP 파일은 머리줄의 시각을 뺀 뒤 비교합니다 (계획 14장). PNG 그림, 보고서(실행 시각 포함), 회로도 PDF는 증거 자료이며 결정적 산출물이 아닙니다. STEP 파일(기구 확인용)은 주문 파일이 아닙니다.
- 인터프리터: 대부분 `B:/Claude/.venv`, 보드 처리는 KiCad 10.0.6의 python, 3D 모델·그림 일부는 `B:/0)AI_Cad/CanSat_EDF_Rocket/.venv` (실행만 함).

## 7. 파일 지도

| 무엇 | 경로 |
|---|---|
| 주문 안내서 (먼저 읽기) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` |
| 주문 파일: 거버 zip / BOM / CPL (기본 쌍) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier_gerber.zip` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom.csv` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl.csv` |
| 주문 파일: MCU까지 JLC 실장 쌍 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier_gerber_with_mcu.zip` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom_with_mcu.csv` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl_with_mcu.csv` |
| 부품 회전 표 / 미리보기 대조 그림 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/cpl_rotation_table.csv` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/cpl_rotation_check.png` |
| 따로 살 것 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/off_board_purchase.csv` |
| 비용 추정 (E) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlc_fee_estimate.csv` |
| JLC 재고 확인 결과 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlc_gate_report.json` |
| 검사 결과 모음 + 파일 해시 + 출고 판정 (계획의 MANIFEST.json) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/manifest.json` |
| 검사 보고서 폴더 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/reports` |
| 회로도 PDF | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/schematic/vtol_carrier_schematic.pdf` |
| 3D STEP (J14 모델 없음) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier.step` |
| 장착 그림·슬라이드와 설명 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/INDEX_ko.md` |
| 회로도·보드 발표용 그림 (보드 그림에는 모든 부품 모델이 보임) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/presentation` |
| 옛 rev 2c 시험 그림 (보관용, 납품물 아님) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/_spike_rev2c` |
| KiCad 프로젝트 (회로도·보드) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/kicad/vtol_carrier.kicad_pro` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/kicad/vtol_carrier.kicad_sch` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/kicad/vtol_carrier.kicad_pcb` |
| 사양 / 사양 요약 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/carrier_spec.json` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/SPEC_ko.md` |
| 계획 (설계 기준) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/REV3_PLAN.md` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/rev3_plan.json` |
| 모듈 치수 (대부분 가정) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/module_dimensions.json` |
| 스크립트 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts` |
| rev 2c 보관본 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/archive/rev2c_pololu_1s` |

## 8. 이 문서의 숫자 출처

| 값 이름 | 값 | 출처 파일 | 표시 |
|---|---|---|---|
| 출고 판정 | NOT ORDERABLE | out/manifest.json release_gate | M (도구 검사) |
| 출고 판정 이유 | routing_rules FAIL (battery_path) | out/manifest.json release_gate.reasons | M |
| 배터리 경로 저항 20C | 0.03231 Ohm | out/reports/routing_rules.json battery_path | E |
| 배터리 경로 목표 | 0.035 Ohm | out/reports/routing_rules.json battery_path | F (계획 13절) |
| 2 mm보다 좁은 길이 | 16.1 mm | out/reports/routing_rules.json battery_path | M |
| 좁은 구간 | VBAT_LOGIC_RAW 1.07 mm (폭 1 mm); VBAT_LOGIC 3.93 mm (폭 1 mm); VBAT_LOGIC_SW 11.1 mm (폭 1.5/1.56 mm) | out/reports/routing_rules.json battery_path.legs | M |
| 배터리 경로 저항 60C | 0.03739 Ohm | out/reports/routing_rules.json battery_path | E |
| ERC | 오류 0, 경고 0 (시트 8장) | out/reports/erc.json (kicad-cli 10.0.6) | M |
| DRC | 위반 0, 경고 0, 미연결 0, 회로도 불일치 0 | out/reports/drc.json (kicad-cli 10.0.6) | M |
| 넷리스트=사양 | PASS (부품 174, 네트 105) | out/manifest.json netlist_vs_spec | M |
| 보드=IR | PASS (핀 506 / 506) | out/manifest.json board_vs_ir | M |
| MCU 핀맵 | PASS (핀 256, 차이 0) | out/reports/rev3_mcu_pinmap.json | M |
| U8/U9 패드표 | PASS | out/reports/rev3_u8_u9_pads.json | M (표는 데이터시트에서 옮김, F) |
| 배선 규칙 PASS 수 | 11 | out/reports/routing_rules.json | M |
| 배선 규칙 수 | 12 | out/reports/routing_rules.json | M |
| 배선 규칙 FAIL | battery_path | out/reports/routing_rules.json | M |
| XTAL1 최악 | 5.96 mm (한계 5.96 mm) | out/reports/routing_rules.json crystals | M |
| 규칙 파일 | PASS | out/reports/routing_rules.json rule_file | M |
| 구리 섬 | 없음 (모든 네트 한 덩어리) | out/manifest.json board_metrics | M |
| 패드에 닿는 비아 | 0 | out/manifest.json board_metrics | M |
| JLC 확인 결과 | PASS (40/40줄), with_mcu PASS | out/jlc_gate_report.json | M |
| JLC 확인 시각 | 2026-11-11T16:19:27+09:00 | out/jlc_gate_report.json | M (그 시각의 재고) |
| 주문 파일 검사 | PASS | out/reports/rev3_export_files.json | M |
| 주문 파일 재현 | PASS | out/reports/rev3_export_determinism.json | M |
| 결정성 | FAIL - 바이트 비교 파일 70개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 1개 (out/vtol_carrier.step: kicad-cli가 STEP을 내보낼 때마다 항목 순서가 바뀜. 번호·순서를 뺀 내용 비교는 같음) | out/reports/determinism.json | M |
| 가장자리 2.5 mm 안 몸체 | 32 | out/reports/edge_distance.json | M |
| 가장자리 0.3 mm 안 몸체 | U7 0.15 mm, J15 0.29 mm | out/reports/edge_distance.json | M |
| 모듈 간섭 | A / NOT_VERIFIED, 가정 치수로 충돌 0건 | out/fit_checks.json module_keepouts | A |
| 외곽 | 76 x 70 mm | spec/carrier_spec.json | F (size) / A (corner radius) |
| 두께 | 1.6 mm | spec/carrier_spec.json | F |
| 층 수 | 4 | spec/carrier_spec.json | F (계획 불변조건 d) |
| 높이 한계 | 13.4 mm | spec/module_dimensions.json | F (계획) |
| 블록 PWR | 27 / 26 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 NAV | 17 / 14 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 FCU | 15 / 13 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 COM | 36 / 32 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 GNS | 18 / 15 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 SHIFT | 18 / 16 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 IO | 37 / 37 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 LOG | 1 / 1 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 LINK | 5 / 3 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 코드 U8 | C919459 | out/jlcpcb_bom.csv | M |
| 코드 L5 | C53426685 | out/jlcpcb_bom.csv | M |
| 코드 U9 | C20345924 | out/jlcpcb_bom.csv | M |
| UVLO 저항 | R65 470k 1%, R66 270k 1%, R67 3.3M 1% | spec/carrier_spec.json | M |
| 켜짐 전압 | 3.44 ~ 3.73 V | spec/rev3_plan.json power_block.uvlo | E |
| 꺼짐 전압 | 3.00 ~ 3.25 V | spec/rev3_plan.json power_block.uvlo | E |
| 코드 U7 | C82891 | out/jlcpcb_bom.csv | M |
| 코드 J2 | C22436147 | out/jlcpcb_bom.csv | M |
| 코드 J8 | C32713262 | out/jlcpcb_bom.csv | M |
| 코드 J14 | C5364642 | out/jlcpcb_bom.csv | M |
| 코드 J15 | C131337 | out/jlcpcb_bom.csv | M |
| 코드 C45 | C11366 | out/jlcpcb_bom.csv | M |
| 배선 세그먼트 | 2308 | kicad/work/route_reuse_log.json | M |
| 배선 비아 | 327 | kicad/work/route_reuse_log.json | M |
| GND 스티칭 비아 | 102 | kicad/work/route_reuse_log.json | M |
| 변경 행 | 교체 11, 추가 13, 삭제 1, 그대로 (부품 없음) 10, 그대로 140 | spec/rev3_plan.json changes | F (계획) |
| fit jlc | 157 | spec/carrier_spec.json | M |
| fit jlc_optional | 4 | spec/carrier_spec.json | M |
| fit user | 1 | spec/carrier_spec.json | M |
| fit none | 12 | spec/carrier_spec.json | M |
| main bom_lines | 40 | out/manifest.json bom_cpl.pairs.main | M |
| main bom_lines_smt | 34 | out/manifest.json bom_cpl.pairs.main | M |
| main bom_lines_tht | 6 | out/manifest.json bom_cpl.pairs.main | M |
| main parts | 157 | out/manifest.json bom_cpl.pairs.main | M |
| with_mcu bom_lines | 41 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| with_mcu parts | 161 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| U1-U4 with_mcu | C6510 | out/jlcpcb_bom_with_mcu.csv | M |
| 프로그래밍 헤더 선택 | A | spec/carrier_spec.json prog_headers.option | M |
| B안 결론 | 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) | out/reports/programming_access.json | M |
| B안 2x3 자리 | SMD 0곳, THT 0곳 | out/reports/programming_access.json verdict_per_family | M |
| 방향 중요 부품 수 | 43 | out/reports/cpl_rotation.json | M |
| 회전 미확인 | C45, C47, C52, U9 | out/reports/cpl_rotation.json | NOT_VERIFIED |
| 리플로 한계 | C2833309, C2286, C53426685 | out/fit_checks.json reflow_rating | F (데이터시트) |
| 리플로 미확인 줄 수 | 32 | out/fit_checks.json reflow_rating | NOT_VERIFIED |
| 다리 규칙 초과 | J15, J17 | out/fit_checks.json tht_leads | F (도면값) |
| 뒷면 다리 규칙 | 1.5 mm | out/fit_checks.json tht_leads | F (계획) |
| 다리 길이 미확인 | J1, J14 | out/fit_checks.json tht_leads | NOT_VERIFIED |
| GY-521 받침 높이 | 2.54 mm | spec/module_dimensions.json gy521.support_h | A |
| BMP280 헤더 종류 | 직각 | spec/module_dimensions.json bmp280.header_type | A |
| 빌드 시간 | 10.7분, 11.2분 | out/reports/determinism.json builds | M |
| 결정성 시각 | 2026-11-11T17:53:25+0900 | out/reports/determinism.json | M |
