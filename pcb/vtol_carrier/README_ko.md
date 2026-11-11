# VTOL 아비오닉스 캐리어 PCB rev 3 (JLC가 실장해서 보내는 판)

**이 보드는 아직 만들거나 전원을 넣어 본 적이 없습니다 (NOT_VERIFIED).** 아래의 PASS는 모두 도구(kicad-cli ERC/DRC, 스크립트)의 결과이며 보드가 동작한다는 뜻이 아닙니다. JLC가 이 파일을 받아 주는지도 업로드 후 JLC의 DFM 결과와 견적을 봐야 압니다.

**2026-11-12 변경 (사용자 요청: "아, esp도 내가 직접 실장할거야. 그럼 이코노믹으로 진행하고 pcb의 코팅도 무연으로 해줘", "이코노믹으로 스루홀 제외하고 어떤 부품을 따로 구매해야하는건지 리스트를 정리해줘"):** (1) Wi-Fi 모듈 ESP-12F (U7)는 JLC가 아니라 사용자가 직접 납땜합니다 (JLC BOM / CPL에서 빠짐, 두 거버 묶음 모두 U7 패드 22개에 솔더 페이스트 구멍 없음). (2) 스루홀 커넥터 12개(J2, J5, J8~J12, J14~J18, 납땜 39곳)도 JLC가 달지 않고 사용자가 사서 납땜합니다. (3) 주문은 Economic PCBA이고 JLC는 SMT 부품 144개(BOM 33줄)만 실장합니다. (4) 기판 표면처리는 무연 HASL입니다 (보드 파일 표기 'HAL lead-free'). 풋프린트·네트·위치는 모두 그대로이고, 구리와 배치는 바뀌지 않았습니다 (배선 재생에 쓴 배선 전 보드: 'byte-identical to the recorded unrouted board except the copper_finish value', M). 실크에서는 같은 날 검증 지적으로 아랫면 이름표의 핀 글자 순서만 고쳤습니다: 핀 줄과 나란히 적힌 이름표는 이제 그 면에서 본 패드 순서대로입니다 (아랫면: J14 '1S BAT -+', J7 'GPS G RX TX V', J2 'BMP280 3V 3V SDA SCL G 3V'; 검사 PASS, M). 주문 방법은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md`, 따로 살 부품은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/따로_살_부품_ko.md`.

표시: M = 이 PC에서 도구로 측정, E = 추정(계산), A = 가정(실물로 확인 필요), F = 문서·데이터시트 근거. 근거가 없으면 NOT_VERIFIED. 이 문서는 `scripts/docs_ko.py` 가 보고서 파일에서 숫자를 읽어 만든 것입니다 (손으로 고치지 말고 다시 생성). 측정·계산된 숫자의 출처는 맨 끝 8장에 있고, 그 표에 없는 숫자(칩 종류, 규칙 값, 시험 조건)는 계획서 `spec/REV3_PLAN.md` 와 사양 `spec/carrier_spec.json` 에서 옮긴 고정값(F)입니다.

## 1. 지금 상태

**출고 판정(도구 검사 기준): NOT ORDERABLE** - 이유: routing_rules FAIL (battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance).

배선이 끝난 보드에서 배터리 전원 경로(J14 → Q10 → J15 → L5)의 구리 저항은 0.03173 Ohm (20 C, E)로 계획 목표 0.035 Ohm 를 만족하지만, 계획이 요구한 '2.0 mm 이상 폭'을 8.57 mm 구간에서 지키지 못했습니다 (VBAT_LOGIC_RAW 1.07 mm (폭 1 mm); VBAT_LOGIC_SW 7.5 mm (폭 1.5/1.56 mm)). 60 C에서는 0.03672 Ohm (E)로 목표를 넘습니다. 도구는 이 규칙을 느슨하게 하지 않았습니다. 그대로 주문할지, 배선을 고칠지는 사용자가 정할 일입니다 (`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 0장).

독립 검토(E-R1 / E-R2 / E-R3) 뒤 승압 IC U8 주변을 다시 배치하고 새로 배선했습니다 (재배치 단계). 지금 보드에서 잰 값(M):

- 윗면 GND: PASS (U8.4, C57.2, C43.2, C41.2 이 같은 윗면 GND 구리 한 덩어리 위에 있음; 자기 비아가 있는 패드 4 / 4). 출력 커패시터 C57이 U8 바로 옆에서 VOUT 핀과 GND 핀을 윗면 구리로 잇습니다.
- 입력 커패시터 (검사 input_cap_vin PASS): C41 → U8 VIN 구리 길이 1.54 mm (비아 0개; 기준 5 mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E), GND 패드가 U8 GND와 같은 윗면 구리: 예.
- 되먹임(FB) 선: PASS (출력 커패시터 패드 사이를 지나는 FB 구리 0곳, 가장 가까운 간격 2.613 mm, FB 선 길이 4.575 mm).
- 통과한 승압부 검사: input_cap, hot_loop_top_gnd, input_cap_vin, fb_clear_of_caps. **아직 FAIL인 검사: hot_loop, input_cap_l5, sw_node, crystal_distance.**
- 남은 편차 1 (hot_loop FAIL): C57 +5V 패드-U8.6 0.95 mm, GND 패드-U8.4 1.611 mm; C43 +5V 패드-U8.6 3.25 mm, GND 패드-U8.4 3.537 mm. 계획 12장은 두 출력 커패시터 모두 IC에서 2 mm 안을 요구하지만, 1206 커패시터는 U8의 VOUT / GND 핀 옆에 하나만 놓을 수 있어 둘째(C43)는 C57 바로 옆, 같은 윗면 +5V / GND 구리 위에 두었습니다.
- 남은 편차 2 (input_cap_l5 FAIL): C41 → L5 입력 패드 구리 길이 13.53 mm (기준 5 mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E; U8 VIN 패드와 L5 입력 패드 사이 직선 거리 10.675 mm). U8의 VIN 핀과 L5 입력 패드가 떨어져 있어 커패시터 하나로 두 곳 모두 몇 mm 안에 둘 수 없습니다. 둘째 입력 커패시터 C42는 스위치 단자 J15 옆 그대로입니다 (모서리에 1206 자리가 더 없음).
- **남은 편차 3 (sw_node FAIL) - 재배치로 나빠진 점:** U8의 SW 패드와 L5의 SW 패드 사이 간격 5.551 mm (기준 2 mm 이하 - 계획 12장의 'L5는 U8 핀 바로 옆'에 숫자가 없어 스크립트가 계획 12장의 2 mm를 빌려 읽은 값, E), SW 선 구리 넓이 2.317 mm2, 구리 저항 약 9.3 mOhm (20 C, E), SW 선이 패드 사이를 지나는 다른 부품: C43, C57. SW 선은 길이 6.619 mm, 폭 0.35 mm, 비아 0개, F.Cu만 이고, 재배치 전에는 길이 1.174 mm, 패드 간격 0.464 mm 였습니다. 계획 12장·13장은 'L5는 U8 핀 바로 옆, SW 구리는 두 패드에 필요한 만큼만, 패드가 허락하는 한 짧게'를 요구하는데 지금 보드는 이를 지키지 못합니다. 층·비아만 보는 검사 boost_sw는 PASS 이지만, 이것은 SW 선이 짧다는 뜻이 아닙니다.
- **남은 편차 4 (crystal_distance FAIL) - 재배치 전에도 지키지 못했고 재배치로 더 가까워진 점:** rev 2c의 Pololu 핀은 NAV 크리스털 Y1 패드에서 9.217 mm, U1의 XTAL 핀에서 9.616 mm 떨어져 있었습니다. 지금은 가장 가까운 부품이 Y1 패드에서 7.501 mm (C57), XTAL 핀에서 8.35 mm (U8) 이고, SW 구리(선과 두 패드)는 Y1 패드에서 8.475 mm, XTAL 핀에서 8.943 mm, 크리스털 배선 구리에서 7.465 mm (rev 2c Pololu 핀은 크리스털 배선에서 8.243 mm) 입니다. rev 2c보다 가까운 부품: C43, C57, L5, U8. 재배치 전의 SW 구리는 Y1 패드에서 8.714 mm, XTAL 핀에서 11.497 mm, 크리스털 배선에서 8.891 mm 였고, 그때 rev 2c보다 가까웠던 부품은 C57, L5 입니다. 계획 12장은 승압부를 rev 2c의 Pololu 핀보다 크리스털에 가깝게 두지 말라고 했습니다. 16 MHz 크리스털(NAV 칩 U1)에 잡음이 얼마나 실리는지는 보드에서 재기 전까지 NOT_VERIFIED입니다.
- 편차 3, 4의 실제 영향(스위칭 과전압, 입력 리플, SW 선 발열, 크리스털 잡음)은 보드에서 오실로스코프로 재기 전까지 NOT_VERIFIED입니다.
- 자세한 내용과 선택지: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 0장 B.

| 항목 | 결과 | 표시 |
|---|---|---|
| 회로도 ERC | 오류 0, 경고 0 (시트 8장) | M |
| PCB DRC (`--schematic-parity --severity-all --refill-zones`) | 위반 0, 경고 0, 미연결 0, 회로도 불일치 0 | M |
| 넷리스트 = 사양 | PASS (부품 174, 네트 105) | M |
| 보드 패드 넷 = IR | PASS (핀 506 / 506) | M |
| MCU 4개 핀 = rev 2c | PASS (핀 256, 차이 0) | M |
| U8 / U9 핀-네트 표 | PASS | M (표는 F) |
| 배선 규칙 (계획 13장) | 16 / 21 PASS, FAIL: battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance | M / E |
| 크리스털 XTAL1 길이 (rev 2c보다 나빠지면 안 됨) | 5.96 mm (한계 5.96 mm) | M |
| 설계 규칙 파일 = rev 2c | PASS | M |
| 구리 섬 / 패드에 닿는 비아 | 없음 (모든 네트 한 덩어리) / 0 | M |
| JLC 부품 재고·주문 가능 확인 | PASS (33/33줄), with_mcu PASS, 2026-11-12T02:54:37+09:00 | M (그 시각 기준) |
| 주문 파일 형식·fit 집합 검사 | PASS | M |
| 주문 파일을 두 번 써서 같은지 | PASS | M |
| 전체 다시 만들기 두 번 = 같은 파일 | FAIL - 바이트 비교 파일 71개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 1개 (out/vtol_carrier.step: kicad-cli가 STEP을 내보낼 때마다 항목 순서가 바뀜. 번호·순서를 뺀 내용 비교는 같음) | M |
| 다른 폴더에서 다시 만들어도 같은 파일 | PASS - 다른 폴더에 복사해 다시 만든 결정적 파일 60개 중 51개가 이 폴더의 것과 바이트까지 같음 (보드, 회로도, 거버와 zip, BOM, CPL 등 주문에 쓰는 파일 43개는 모두 여기에 듦). 나머지 9개(README_ko.md, kicad/placement.json, kicad/schematic_manifest.json, kicad/work/unrouted_manifest.json, out/ORDER_ko.md, out/따로_살_부품_ko.md, spec/SPEC_ko.md, spec/carrier_spec.json, spec/parts_sources.json)는 자기 폴더 이름이나 그런 파일의 sha256을 담고 있어, 폴더 이름과 그 sha256을 바꿔 넣은 뒤에 같음. 그림 단계 산출물 11개는 그 폴더에서 다시 만들지 않아 비교하지 않음 | M |
| 가장자리 2.5 mm 안 부품 몸체 | 모든 자리 34개 = JLC 실장 15개 (with_mcu 16개) + JLC가 붙이지 않는 자리 19개 (U4, J1, J2, J5, J8, J9, J10, J11, J12, J14, J15, J16, J17, J18, U7, J19, JP1, Q3, R47) (0.3 mm 안: U7 0.15 mm, J15 0.29 mm - 둘 다 사용자가 납땜하는 자리; JLC 실장 부품 중 가장 가까운 것 L5 0.45 mm) | M |
| 모듈(OpenLog / BMP280 / GY-521) 간섭, 보통 경우 | A / NOT_VERIFIED, 가정 치수(보통 경우)로 충돌 0건; 최소 간격 0.5 mm 미만: U1 | A / NOT_VERIFIED |
| 모듈 간섭, 보수적 경우 (OpenLog 아랫면 부품 1.2 mm 가정) | 충돌: C3 -0.05 mm (J5_openlog 'insertion stroke beyond the plugged PCB end'), U6 -0.5 mm (J5_openlog 'insertion stroke beyond the plugged PCB end'); 0.5 mm 미만: C48, Q2, Q8, U1, U2, Y1 | A / NOT_VERIFIED |

## 2. 이 기판은 무엇인가

- 크기 76 x 70 mm, 두께 1.6 mm, 4층 (F.Cu 신호와 부품 / In1 GND 전면 / In2 +5V 전면 / B.Cu 신호). 부착 구멍 없음: 3 mm 폼테이프 + 케이블타이 1개. 케이블타이 띠(x -3..+3 mm)에는 실장 부품이 없습니다.
- 부품은 모두 윗면에만 있습니다. 부품 높이 한계는 기판 윗면에서 13.4 mm 입니다. 아랫면은 폼테이프 자리라 비어 있습니다.
- 비행 컴퓨터 칩 4개: U1 NAV, U2 FCU, U3 COM, U4 GNS (ATmega128A-AU, 16 MHz, 5 V). 무선은 U7 ESP-12F (Wi-Fi 중계; 사용자가 직접 납땜).
- 센서: GY-521 (IMU, J1에 납땜), BMP280 (기압, 소켓 J2), NEO-7M GPS (기체 윗뚜껑, 케이블로 J7). 기록기 OpenLog는 소켓 J5.
- 전원 흐름: 1S 로직 배터리 J14 → Q10 역극성 보호 → 스위치 리드 J15 → U8 승압 5 V (+ U9 저전압 차단) → +5V 면 → U6 → +3V3.
- 커넥터: GPS용 J7 (JST-GH, SMD)만 JLC가 실장합니다. 스루홀 커넥터 12개(J2, J5, J8~J12, J14~J18)는 2026-11-12 결정으로 사용자가 사서 납땜합니다 (받은 보드에는 빈 도금 구멍과 실크 윤곽만 있음).
- 기판 아랫면 실크(B.SilkS, 아랫면에서 볼 때 바로 읽히도록 좌우 반전)에 이 프로젝트의 깃 저장소 주소와 연락용 이메일 주소가 두 줄로 인쇄됩니다 (글자 높이 1.2 mm, 선 굵기 0.2 mm; 패드까지 1.221 mm 이상, 비아까지 1.147 mm 이상, 기판 변까지 4.697 mm 이상, 글자 밑 비아 0개, 케이블타이 띠와 겹침: 없음; 검사 PASS, M). 주소 글자 자체는 문서에 다시 적지 않습니다. 윗면에는 두 줄이 들어갈 빈자리가 없어 아랫면에 넣었고, 폼테이프를 붙이면 그 밑은 가려집니다.

블록 (회로도 시트 기준, 부품 수는 사양에서 셈):

| 블록 | 내용 | 부품 자리 | 그중 JLC 실장 |
|---|---|---|---|
| PWR | 전원: 1S 로직 배터리 입력 J14, 역극성 보호 Q10, 스위치 리드 J15, 5 V 승압 U8 TPS61023 + L5, 저전압 차단 U9 TLV3012B, 3.3 V 레귤레이터 U6, 접지 연결 NT1, 배터리 전압 분압 | 27 | 24 |
| NAV | NAV 칩 U1 ATmega128A + GY-521 IMU 자리 J1 + BMP280 소켓 J2 | 17 | 13 |
| FCU | FCU 칩 U2 ATmega128A | 15 | 13 |
| COM | COM 칩 U3 ATmega128A + ESP-12F Wi-Fi 모듈 U7, 레벨 시프터, 부트 스트랩, ESP 굽기 단자 J3 | 36 | 31 |
| GNS | GNS 칩 U4 ATmega128A + GPS 리드 커넥터 J7 (NEO-7M) | 18 | 15 |
| SHIFT | BSS138 레벨 시프터 5 V <-> 3.3 V | 18 | 16 |
| IO | 서보 J8~J11, ESC J12, 아밍 스위치 J16, 부저 J17, 6S 전압 감지 J18 등 입출력 커넥터 | 37 | 29 |
| LOG | OpenLog 소켓 J5 | 1 | 0 |
| LINK | 칩 사이 연결선 (UART / SPI) 부품 | 5 | 3 |

## 3. rev 2c에서 바뀐 것과 이유

- **5 V 전원 모듈을 없앴습니다.** 꽂는 Pololu 모듈(J6, 디바이스마트 부품)은 JLC가 실장할 수 없어서, JLC 부품으로 만든 승압 회로 U8 TPS61023 (C919459) + L5 (C53426685)를 예전 모듈 자리(+x / -y 모서리)에 올렸습니다. 이유: 요청(모든 부품 JLC 실장).
- **저전압 차단을 유지했습니다.** 예전 모듈 안에 있던 기능입니다. U9 TLV3012B (C20345924) + R65 470k 1%, R66 270k 1%, R67 3.3M 1%: 켜짐 3.44 ~ 3.73 V, 꺼짐 3.00 ~ 3.25 V (계산값 E, 실측 전).
- **Wi-Fi 모듈 ESP-12E → ESP-12F** (참고 부품 LCSC C82891). 핀과 풋프린트가 같습니다. 계획은 JLC가 실장할 수 있는 부품이라 ESP-12F를 골랐는데, 2026-11-12 사용자 결정으로 이 모듈은 사용자가 직접 납땜합니다 (JLC BOM의 U7: BOM에 없음).
- **스루홀 커넥터 12개는 사용자가 사서 납땜합니다** (2026-11-12 사용자 결정; 계획은 JLC가 손납땜). 참고 부품 번호 (계획 때 JLC BOM에 있던 번호, F): 소켓 J2 / J5 C22436147, 서보 헤더 J8~J12 C32713262, J16 C32713261, J18 C32713263, 배터리 J14 C5364642, 스위치·부저 J15 / J17 C131337. JLC BOM에는 없습니다 (J2: BOM에 없음). 탄탈 C45 (C11366, JLC 실장)와 헤더·소켓 번호 일부는 재고나 도면 근거로 바꿨고 D3 / D12에서 rev 2c 번호로 되돌릴 수 있습니다.
- **그대로인 것:** MCU 4개의 핀 256개가 가는 네트 (PASS (핀 256, 차이 0)), 외곽, 커넥터·MCU·크리스털·모듈 소켓·ESP 위치와 방향, 설계 규칙 (PASS), 크리스털 배선 (5.96 mm (한계 5.96 mm)).
- **배치·배선을 다시 했습니다.** 새 부품 13개를 넣느라 주변 수동 부품 일부가 옮겨졌고, 배선은 새로 했습니다 (최종 보드 파일 기준 세그먼트 2557개, 원호 0개, 비아 442개: 0.6/0.3 mm 439개, 0.8/0.4 mm 3개). 배선은 Freerouting 실행 기록(세션 파일)을 다시 넣는 방식으로 재현됩니다.
- 부품마다 바뀐 내용과 이유 분류(태그)의 전체 표: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 10장 (교체 11, 추가 13, 삭제 1, 그대로 (부품 없음) 10, 그대로 140).

## 4. 누가 무엇을 붙이나

| 구분 | 뜻 | 개수 |
|---|---|---|
| jlc | JLC가 실장 (주문 BOM / CPL에 들어감) | 144 |
| jlc_optional | 기본은 사용자 납땜, with_mcu 파일 쌍을 쓰면 JLC 실장 | 4 |
| user | 사용자가 납땜 | 14 |
| none | 부품 없음 (빈 도금 구멍 / 미실장 / 구리 패턴만) | 12 |

- JLC가 붙이는 것: SMT 부품만. 기본 파일 쌍 기준 BOM 33줄 (SMT 33줄, 스루홀 0줄), 부품 144개. MCU까지 맡기는 with_mcu 쌍은 34줄 / 148개 (U1~U4 = C6510).
- 사용자가 붙이는 것: ATmega128A-AU 4개 (기본, with_mcu 쌍이면 JLC), ESP-12F 1개 (U7, 패드 22개), 스루홀 커넥터 12개 (J2, J5, J8~J12, J14~J18, 납땜 39곳) - ESP-12F와 커넥터는 두 파일 쌍 모두에서 사용자 몫. 그리고 GY-521 (J1 구멍에 모듈 핀을 납땜), BMP280 (직접 단 소켓 J2에 꽂음), OpenLog (모듈에 직각 6핀 헤더를 납땜한 뒤 직접 단 소켓 J5에 꽂음), NEO-7M (케이블로 J7).
- 부품 없이 비워 둔 자리: 프로그래밍 단자 J3, J19~J22 (2x3), J24~J26 (1x2) - 현재 선택 A안 (빈 도금 구멍). JLC가 헤더를 띠 밖에 실장하는 B안은 배치 검색 결과 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) (SMD 0곳, THT 0곳).
- MCU·센서 말고 사용자가 해야 하는 일(ESP-12F와 스루홀 커넥터 납땜, 다리 자르기, 프로그래밍 단자 등)은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 8장에, 납땜 순서·커넥터 방향·받은 뒤 전원 시험은 7장에, 따로 살 부품은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/따로_살_부품_ko.md` 에 모았습니다.

## 5. 확인되지 않은 것 (NOT_VERIFIED)

- 배터리 경로 폭 규칙 FAIL (8.57 mm)과 승압부에 남은 FAIL (hot_loop, input_cap_l5, sw_node, crystal_distance): 사용자 결정 전까지 도구 판정은 NOT ORDERABLE 입니다.
- 재배치한 승압부의 전기적 효과: SW 선이 두 출력 커패시터 아래를 지나 길어졌고(검사 sw_node FAIL), 승압부와 SW 구리가 크리스털 Y1 / U1 XTAL 핀에 rev 2c의 Pololu 핀보다 가까워졌습니다(검사 crystal_distance FAIL; 숫자는 1장). 스위칭 과전압, 입력 리플, SW 선 발열, 크리스털 잡음 영향은 보드에서 재기 전까지 NOT_VERIFIED입니다.
- 모듈 끼움, 보수적 경우: OpenLog 아랫면 부품을 1.2 mm로 잡으면 꽂는 길에서 C3 -0.05 mm (J5_openlog 'insertion stroke beyond the plugged PCB end'), U6 -0.5 mm (J5_openlog 'insertion stroke beyond the plugged PCB end') 이 겹치고, C48, Q2, Q8, U1, U2, Y1 은 0.5 mm 보다 가깝습니다 (A). OpenLog 아랫면 높이를 재고, 막을 수 있는 부품을 납땜하기 전에 실제로 꽂아 보십시오.
- 보드 동작 전체: 만들거나 전원을 넣은 적이 없습니다. 전원부 수치(5 V 출력, 차단 전압, 전류 여유)는 데이터시트 계산(E)입니다.
- JLC의 수락: Economic 주문에서 JLC가 기판 안에 툴링 홀 자리를 찾는지 (부품 자리 점유율 88.5 %; 못 찾으면 레일 = Standard를 요구할 수 있음), 가장자리 가까운 부품 (JLC 실장 중 가장 가까운 것 L5 0.45 mm; 0.3 mm 안: 없음), 지금 파일의 BOM / CPL 매칭은 업로드 후 화면과 DFM 결과, 견적으로만 확인됩니다.
- 주문 화면에서 본 것 (2026-11-12 01:30-02:06 KST, M): 이 변경 전 빌드의 파일을 올려 부품 40줄을 인식하고 40줄을 확정했고, 화면에서 ESP-12F와 스루홀 줄을 뺀 Economic 견적은 배송 제외 76.82 USD였습니다 (Economic이 ESP-12F를 뺀 39줄을 모두 받음). 지금 파일(처음부터 SMT만 든 BOM)로는 아직 올려 보지 않았습니다.
- 사용자가 사는 스루홀 커넥터 6종과 ESP-12F: 낱개로 산 부품이 구멍·패드에 맞는지, 그 재고와 가격 (LCSC 기록은 2026-11-12T02:50:37+09:00 ~ 02:51:09+09:00 기준 M, 그 밖의 쇼핑몰은 확인 안 함), 무연 HASL 구멍의 납땜성은 해 보기 전에는 NOT_VERIFIED입니다. 배터리 커넥터 J14는 방향이 틀리면 극성이 뒤집힙니다 (주문 안내서 7장 3번).
- CPL 회전: 방향이 중요한 30개 중 C45, C47, C52, U9 은 자료끼리 어긋나 NOT_VERIFIED입니다. 나머지도 JLC 미리보기로만 확정됩니다. 탄탈 C45 / C52, 전해 C47을 거꾸로 붙이면 화재 위험이 있습니다.
- 리플로 온도: 이번 주문 방식 Economic은 최고 255±5 C 입니다. 정격이 그 최악값과 같아 여유가 없는 부품: C2833309 (C47, 정격 260 C), C2286 (D1/D2/D3/D4, 정격 260 C), C53426685 (L5, 정격 260 C) (C47은 리플로는 한 번만 (두 번은 사이에 30분을 둘 때만)); 문서에서 정격을 찾지 못한 부품 번호 31개.
- ESP-12F 손납땜: 모듈과 무연 HASL 패드의 납땜성, 패드 바로 옆 부품과의 간격에서 작업이 되는지는 해 보기 전에는 NOT_VERIFIED입니다.
- 뒷면 다리 길이 (모든 스루홀을 사용자가 납땜하고 자름; 도면값은 참고 부품 번호 기준 F): J15, J17 은 규칙 1.5 mm 보다 길어 잘라야 하고, J2, J5, J8~J12, J16, J18 는 공차 끝에서 넘습니다. J1, J14 은 확인 못 함.
- 모듈 치수와 끼움: GY-521, BMP280, OpenLog, NEO-7M 치수는 가정값입니다 (GY-521 받침 2.54 mm, BMP280 헤더 직각 - 모두 A). 그림과 간섭 검사도 이 가정 위에 있습니다. 실물을 버니어로 재야 확정됩니다 (D15).
- J14 Molex 커넥터: 라이브러리 3D 모델이 없고 구멍 지름을 도면과 대조하지 못했습니다.
- 재고: 2026-11-12T02:54:37+09:00 기준입니다. 주문 직전에 scripts/jlc_gate.py 를 다시 돌려야 합니다.
- 비용: 주문 화면의 견적 76.82 USD (M, 배송 제외)는 이전 업로드에서 줄을 빼고 읽은 값입니다. 지금 파일의 견적, 메모 검토 뒤 금액, 배송비와 세금은 NOT_VERIFIED입니다.
- GPS 케이블이 기체 안에서 지나갈 길은 설계되지 않았습니다 (그림은 예시).

계획서 17장의 나머지 위험 항목 (계획에서 그대로 옮김; 17장의 모든 항목이 이 문서나 주문 안내서의 어느 줄에 대응하는지 `scripts/docs_ko.py` 가 확인함):

- 승압 IC TPS61023의 전류 한계(valley)는 데이터시트에 입력 3.6 V 기준으로만 나옵니다. 입력 3.0 V에서는 1.50 A가 필요하고, 이는 최소 2.7 A의 56 % 입니다 (E, 계획 17장).
- 저전압 차단의 히스테리시스 여유는 28 mV뿐입니다 (E). 전원 경로 저항이 0.29 Ohm을 넘으면 방전 끝 최대 부하에서 전원이 켜졌다 꺼졌다 할 수 있어, 첫 보드에서 저항을 재야 합니다 (ORDER 7장 5번).
- 차단은 부하가 걸린 전압으로 판단합니다 (필터 16.3 ms). 최대 부하가 이어지면 쉬는 전압이 3.62 V 아래인 팩에서 차단이 걸릴 수 있습니다 (E).
- 켜짐 전압이 최대 3.73 V라서, 그보다 낮게 쉬고 있는 팩으로는 보드가 켜지지 않을 수 있습니다 (E).
- +5V 쪽 226.9 uF로 시동할 때 1.9 ms 리셋, 감지 필터와의 순서는 확인하지 않았습니다 (첫 보드에서 오실로스코프로, NOT_VERIFIED).
- +3V3 여유: +5V 최저 4.844 V에서 AMS1117 강하 최대 1.3 V를 빼면 3.3 V보다 0.244 V 위입니다 (E). Wi-Fi 송신 중 최저값은 확인 안 됨 (ORDER 7장 10번).
- 차단 뒤에도 새 회로 6.7 uA (최대 8.0 uA)와 R15 / R16 분압 34..62 uA가 흐릅니다. 차단은 보관용 스위치가 아니므로 스위치를 꺼야 합니다 (E).
- 22 uF 1206 커패시터의 5 V 직류 바이어스 감소량은 제조사 곡선으로 확인하지 않았습니다 (NOT_VERIFIED). 출력 실효 용량은 4 uF 이상이어야 하며 레일 전체 용량으로 보면 여유가 있습니다 (E).
- U8은 일반 커넥터 심볼을 써서 ERC가 U8 핀에 대한 근거를 주지 않습니다. 대신 패드-네트 표 검사(U8/U9 패드표)가 근거입니다.
- ESP-12F의 무선 성능은 같은 금지 영역에서 ESP-12E와 같다고 가정했습니다 (A). 펌웨어 이미지도 같은 ESP8266 / 4 MB 구성으로 가정합니다 (A).
- 프로그래밍: 빈 구멍에 포고핀 어댑터를 댈 때의 접촉은 확인하지 않았습니다 (NOT_VERIFIED). B안(JLC가 띠 밖에 헤더 실장)은 배치 검색에서 자리가 없었습니다.
- AVCC 초크 C1046은 정격 15 mA (1.15 Ohm) 입니다. ATmega128A의 AVCC 전류와 대조하지 않았습니다 (NOT_VERIFIED).

## 6. 다시 만드는 방법

작업 폴더 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts` 에서 (환경변수 `PYTHONDONTWRITEBYTECODE=1`):

```
B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py           # 전체 (사양 -> 회로도 -> 배치 -> 배선 재생 -> 검사 -> 주문 파일 -> 그림 -> 문서 -> manifest)
B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py --list    # 단계 목록
B:/Claude/.venv/Scripts/python.exe -X utf8 -B determinism.py        # 전체를 두 번 만들어 파일이 바이트까지 같은지 비교
B:/Claude/.venv/Scripts/python.exe -X utf8 -B jlc_gate.py           # 주문 당일 JLC 재고 확인 (인터넷, 읽기만)
```

- 한 번 만드는 데 걸린 시간: 8.3분, 9.0분 (determinism.py의 두 빌드, M). 마지막 결정성 검사: 2026-11-12T05:24:10+0900.
- make_all.py는 인터넷을 쓰지 않습니다. JLC 부품 근거는 저장된 기록으로 다시 평가하고, 재고 확인(jlc_gate.py)은 따로 실행합니다.
- 배선은 기록된 Freerouting 세션을 다시 넣어 같은 보드를 만듭니다. 배치가 바뀌면 route.py가 재생을 거부하고, 새 배선(`make_all.py --reroute`)이 필요합니다. 새 배선은 실행마다 결과가 달라질 수 있습니다. 2026-11-12에 표면처리 표기만 'HAL lead-free' 로 바꿨을 때는 새 배선 없이 재생했습니다: route.py는 배선 전 보드가 세션을 기록한 보드와 표면처리 글자 한 곳만 다르고 나머지 바이트가 모두 같을 때에만 재생을 받아들입니다 (이번 재생의 판정: 'byte-identical to the recorded unrouted board except the copper_finish value', M).
- 결정성 검사 결과: FAIL - 바이트 비교 파일 71개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 1개 (out/vtol_carrier.step: kicad-cli가 STEP을 내보낼 때마다 항목 순서가 바뀜. 번호·순서를 뺀 내용 비교는 같음). 비교 대상은 회로도, 보드, BOM, CPL, 거버와 zip, 회전 표, 3D 모델, 사양, 이 문서와 주문 안내서이고, STEP 파일은 머리줄의 시각을 뺀 뒤 비교합니다 (계획 14장). PNG 그림, 보고서(실행 시각 포함), 회로도 PDF는 증거 자료이며 결정적 산출물이 아닙니다. STEP 파일(기구 확인용)은 주문 파일이 아닙니다.
- 다른 폴더에서 다시 만들기 (`determinism.py --cross-folder <빈 폴더>`): PASS - 다른 폴더에 복사해 다시 만든 결정적 파일 60개 중 51개가 이 폴더의 것과 바이트까지 같음 (보드, 회로도, 거버와 zip, BOM, CPL 등 주문에 쓰는 파일 43개는 모두 여기에 듦). 나머지 9개(README_ko.md, kicad/placement.json, kicad/schematic_manifest.json, kicad/work/unrouted_manifest.json, out/ORDER_ko.md, out/따로_살_부품_ko.md, spec/SPEC_ko.md, spec/carrier_spec.json, spec/parts_sources.json)는 자기 폴더 이름이나 그런 파일의 sha256을 담고 있어, 폴더 이름과 그 sha256을 바꿔 넣은 뒤에 같음. 그림 단계 산출물 11개는 그 폴더에서 다시 만들지 않아 비교하지 않음. 재배치 검토(RR-2)에서 같은 입력을 다른 폴더에서 재생하면 구리는 같아도 배선의 uuid가 달라 보드 파일 바이트가 달랐습니다 (M). 그래서 route.py가 배선·비아·실크 글자의 uuid를 그 항목의 내용(네트, 층, 폭, 좌표)에서 만들도록 바꾸었고, 겹쳐 놓인 똑같은 배선 조각도 하나만 남깁니다. 주의: 사양 파일 `spec/carrier_spec.json` 은 자기가 만들어진 폴더 이름을 담고 있어서, 다른 폴더(예: 다른 체크아웃으로 합친 뒤)에서 다시 만들면 사양의 sha256이 달라지고 JLC 재고 기록(`out/jlc_gate_report.json`)은 '다른 사양'(STALE)으로 판정됩니다. 그때는 그 폴더에서 `jlc_gate.py` 를 다시 실행해야 합니다 (주문 당일에는 어차피 다시 실행합니다).
- 인터프리터: 대부분 `B:/Claude/.venv`, 보드 처리는 KiCad 10.0.6의 python, 3D 모델·그림 일부는 `B:/0)AI_Cad/CanSat_EDF_Rocket/.venv` (실행만 함).

## 7. 파일 지도

| 무엇 | 경로 |
|---|---|
| 주문 안내서 (먼저 읽기) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` |
| 최종 보고서 (납품 단계가 손으로 쓴 시점 기록, 다시 만들어지지 않음) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/REV3_REPORT_ko.md` |
| 주문 파일: 거버 zip / BOM / CPL (기본 쌍) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier_gerber.zip` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom.csv` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl.csv` |
| 주문 파일: MCU까지 JLC 실장 쌍 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier_gerber_with_mcu.zip` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom_with_mcu.csv` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl_with_mcu.csv` |
| 부품 회전 표 / 미리보기 대조 그림 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/cpl_rotation_table.csv` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/cpl_rotation_check.png` |
| 따로 살 것 (읽기 쉬운 목록 / CSV) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/따로_살_부품_ko.md` |
|  | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/off_board_purchase.csv` |
| 따로 살 부품의 LCSC 재고·가격 기록 (2026-11-12에 읽기만 함) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/lcsc_snapshot_20261112.json` |
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
| 주문 화면에서 본 항목·견적 기록 (2026-11-12, 손으로 적은 입력 파일) | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/jlc_order_page_20261112.json` |
| 스크립트 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts` |
| rev 2c 보관본 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/archive/rev2c_pololu_1s` |

## 8. 이 문서의 숫자 출처

| 값 이름 | 값 | 출처 파일 | 표시 |
|---|---|---|---|
| 변경 날짜 | 2026-11-12 | spec/rev3_plan.json amendments | F (사용자 결정) |
| 변경 요청 원문 1 | 아, esp도 내가 직접 실장할거야. 그럼 이코노믹으로 진행하고 pcb의 코팅도 무연으로 해줘 | spec/rev3_plan.json amendments[0].request_verbatim | F (사용자 요청 그대로) |
| 변경 요청 원문 2 | 이코노믹으로 스루홀 제외하고 어떤 부품을 따로 구매해야하는건지 리스트를 정리해줘 | spec/rev3_plan.json amendments[1].request_verbatim | F (사용자 요청 그대로) |
| ESP 패드 수 | 22 | out/fit_checks.json user_soldered_smd.U7 pads | M |
| 스루홀 부품 수 | 12 | out/fit_checks.json user_soldered_tht.summary.parts | M |
| 스루홀 목록 | J2, J5, J8~J12, J14~J18 | spec/carrier_spec.json parts[].fit == user (스루홀 커넥터) | M |
| 스루홀 납땜점 | 39 | out/fit_checks.json user_soldered_tht.summary.joints_per_board | M |
| main parts | 144 | out/manifest.json bom_cpl.pairs.main | M |
| main bom_lines | 33 | out/manifest.json bom_cpl.pairs.main | M |
| 보드 표면처리 | HAL lead-free | kicad/vtol_carrier.kicad_pcb stackup copper_finish | M |
| 배선 재생 입력 보드 | byte-identical to the recorded unrouted board except the copper_finish value | kicad/work/route_reuse_log.json unrouted_board_guard | M |
| 아랫면 나란한 라벨 | J14 '1S BAT -+', J7 'GPS G RX TX V', J2 'BMP280 3V 3V SDA SCL G 3V' | out/reports/routing_rules.json silkscreen.pin_order_labels.labels (relation = along the pin row, layer B.Silkscreen) | M |
| 라벨 검사 | PASS | out/reports/routing_rules.json silkscreen.pin_order_labels.ok | M |
| 출고 판정 | NOT ORDERABLE | out/manifest.json release_gate | M (도구 검사) |
| 출고 판정 이유 | routing_rules FAIL (battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance) | out/manifest.json release_gate.reasons | M |
| 배터리 경로 저항 20C | 0.03173 Ohm | out/reports/routing_rules.json battery_path | E |
| 배터리 경로 목표 | 0.035 Ohm | out/reports/routing_rules.json battery_path | F (계획 13절) |
| 2 mm보다 좁은 길이 | 8.57 mm | out/reports/routing_rules.json battery_path | M |
| 좁은 구간 | VBAT_LOGIC_RAW 1.07 mm (폭 1 mm); VBAT_LOGIC_SW 7.5 mm (폭 1.5/1.56 mm) | out/reports/routing_rules.json battery_path.legs | M |
| 배터리 경로 저항 60C | 0.03672 Ohm | out/reports/routing_rules.json battery_path | E |
| 윗면 GND 한 덩어리 | PASS (U8.4, C57.2, C43.2, C41.2 이 같은 윗면 GND 구리 한 덩어리 위에 있음; 자기 비아가 있는 패드 4 / 4) | out/reports/routing_rules.json hot_loop_top_gnd | M |
| 검사 input_cap_vin | PASS | out/reports/routing_rules.json results | M |
| 입력 커패시터 VIN 경로 | C41 → U8 VIN 구리 길이 1.54 mm (비아 0개; 기준 5 mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E), GND 패드가 U8 GND와 같은 윗면 구리: 예 | out/reports/routing_rules.json input_cap_vin | M (길이는 배선 모델, E) |
| FB 선과 출력 커패시터 | PASS (출력 커패시터 패드 사이를 지나는 FB 구리 0곳, 가장 가까운 간격 2.613 mm, FB 선 길이 4.575 mm) | out/reports/routing_rules.json fb_clear_of_caps | M |
| 승압부 배치 PASS | input_cap, hot_loop_top_gnd, input_cap_vin, fb_clear_of_caps | out/reports/routing_rules.json | M |
| 승압부 배치 FAIL | hot_loop, input_cap_l5, sw_node, crystal_distance | out/reports/routing_rules.json | M |
| 검사 hot_loop | FAIL | out/reports/routing_rules.json results | M |
| 출력 커패시터 간격 | C57 +5V 패드-U8.6 0.95 mm, GND 패드-U8.4 1.611 mm; C43 +5V 패드-U8.6 3.25 mm, GND 패드-U8.4 3.537 mm | out/reports/routing_rules.json hot_loop | M |
| 검사 input_cap_l5 | FAIL | out/reports/routing_rules.json results | M |
| 입력 커패시터 L5 경로 | C41 → L5 입력 패드 구리 길이 13.53 mm (기준 5 mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E; U8 VIN 패드와 L5 입력 패드 사이 직선 거리 10.675 mm) | out/reports/routing_rules.json input_cap_l5 | M (길이는 배선 모델, E) |
| 검사 sw_node | FAIL | out/reports/routing_rules.json results | M |
| SW 노드 | U8의 SW 패드와 L5의 SW 패드 사이 간격 5.551 mm (기준 2 mm 이하 - 계획 12장의 'L5는 U8 핀 바로 옆'에 숫자가 없어 스크립트가 계획 12장의 2 mm를 빌려 읽은 값, E), SW 선 구리 넓이 2.317 mm2, 구리 저항 약 9.3 mOhm (20 C, E), SW 선이 패드 사이를 지나는 다른 부품: C43, C57 | out/reports/routing_rules.json sw_node | M (기준과 저항은 E) |
| SW 선 | 길이 6.619 mm, 폭 0.35 mm, 비아 0개, F.Cu만 | out/reports/routing_rules.json boost_sw | M |
| 재배치 전 SW 선 길이 | 1.174 mm | out/reports/routing_rules_before_relayout.json boost_sw (재배치 전 보드) | M |
| 재배치 전 SW 패드 간격 | 0.464 mm | out/reports/routing_rules_before_relayout.json sw_node (재배치 전 보드) | M |
| 검사 boost_sw | PASS | out/reports/routing_rules.json results | M |
| 검사 crystal_distance | FAIL | out/reports/routing_rules.json results | M |
| 크리스털 거리 | rev 2c의 Pololu 핀은 NAV 크리스털 Y1 패드에서 9.217 mm, U1의 XTAL 핀에서 9.616 mm 떨어져 있었습니다. 지금은 가장 가까운 부품이 Y1 패드에서 7.501 mm (C57), XTAL 핀에서 8.35 mm (U8) 이고, SW 구리(선과 두 패드)는 Y1 패드에서 8.475 mm, XTAL 핀에서 8.943 mm, 크리스털 배선 구리에서 7.465 mm (rev 2c Pololu 핀은 크리스털 배선에서 8.243 mm) 입니다. rev 2c보다 가까운 부품: C43, C57, L5, U8 | out/reports/routing_rules.json crystal_distance (rev 2c 값은 archive/rev2c_pololu_1s/kicad/vtol_carrier.kicad_pcb 에서 같은 방법으로 잼) | M |
| 재배치 전 SW 구리-크리스털 | Y1 패드에서 8.714 mm, XTAL 핀에서 11.497 mm, 크리스털 배선에서 8.891 mm | out/reports/routing_rules_before_relayout.json crystal_distance (재배치 전 보드) | M |
| 재배치 전 크리스털보다 가까운 부품 | C57, L5 | out/reports/routing_rules_before_relayout.json crystal_distance (재배치 전 보드) | M |
| ERC | 오류 0, 경고 0 (시트 8장) | out/reports/erc.json (kicad-cli 10.0.6) | M |
| DRC | 위반 0, 경고 0, 미연결 0, 회로도 불일치 0 | out/reports/drc.json (kicad-cli 10.0.6) | M |
| 넷리스트=사양 | PASS (부품 174, 네트 105) | out/manifest.json netlist_vs_spec | M |
| 보드=IR | PASS (핀 506 / 506) | out/manifest.json board_vs_ir | M |
| MCU 핀맵 | PASS (핀 256, 차이 0) | out/reports/rev3_mcu_pinmap.json | M |
| U8/U9 패드표 | PASS | out/reports/rev3_u8_u9_pads.json | M (표는 데이터시트에서 옮김, F) |
| 배선 규칙 PASS 수 | 16 | out/reports/routing_rules.json | M |
| 배선 규칙 수 | 21 | out/reports/routing_rules.json | M |
| 배선 규칙 FAIL | battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance | out/reports/routing_rules.json | M |
| XTAL1 최악 | 5.96 mm (한계 5.96 mm) | out/reports/routing_rules.json crystals | M |
| 규칙 파일 | PASS | out/reports/routing_rules.json rule_file | M |
| 구리 섬 | 없음 (모든 네트 한 덩어리) | out/manifest.json board_metrics | M |
| 패드에 닿는 비아 | 0 | out/manifest.json board_metrics | M |
| JLC 확인 결과 | PASS (33/33줄), with_mcu PASS | out/jlc_gate_report.json | M |
| JLC 확인 시각 | 2026-11-12T02:54:37+09:00 | out/jlc_gate_report.json | M (그 시각의 재고) |
| 주문 파일 검사 | PASS | out/reports/rev3_export_files.json | M |
| 주문 파일 재현 | PASS | out/reports/rev3_export_determinism.json | M |
| 결정성 | FAIL - 바이트 비교 파일 71개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 1개 (out/vtol_carrier.step: kicad-cli가 STEP을 내보낼 때마다 항목 순서가 바뀜. 번호·순서를 뺀 내용 비교는 같음) | out/reports/determinism.json | M |
| 다른 폴더 결정성 | PASS - 다른 폴더에 복사해 다시 만든 결정적 파일 60개 중 51개가 이 폴더의 것과 바이트까지 같음 (보드, 회로도, 거버와 zip, BOM, CPL 등 주문에 쓰는 파일 43개는 모두 여기에 듦). 나머지 9개(README_ko.md, kicad/placement.json, kicad/schematic_manifest.json, kicad/work/unrouted_manifest.json, out/ORDER_ko.md, out/따로_살_부품_ko.md, spec/SPEC_ko.md, spec/carrier_spec.json, spec/parts_sources.json)는 자기 폴더 이름이나 그런 파일의 sha256을 담고 있어, 폴더 이름과 그 sha256을 바꿔 넣은 뒤에 같음. 그림 단계 산출물 11개는 그 폴더에서 다시 만들지 않아 비교하지 않음 | out/reports/determinism.json cross_folder | M |
| 가장자리 2.5 mm 안 몸체 | 34 | out/reports/edge_distance.json | M |
| 가장자리 2.5 mm 안 JLC 실장 | 15 | out/reports/edge_distance.json bodies_under_2p5_by_fit | M |
| 가장자리 2.5 mm 안 with_mcu | 16 | out/reports/edge_distance.json bodies_under_2p5_by_fit | M |
| 가장자리 2.5 mm 안 JLC 미실장 | 19개 (U4, J1, J2, J5, J8, J9, J10, J11, J12, J14, J15, J16, J17, J18, U7, J19, JP1, Q3, R47) | out/reports/edge_distance.json bodies_under_2p5_by_fit | M |
| 가장자리 0.3 mm 안 몸체 | U7 0.15 mm, J15 0.29 mm | out/reports/edge_distance.json | M |
| JLC 실장 가장 가까운 몸체 | L5 0.45 mm | out/reports/edge_distance.json jlc_placed_bodies.default_order closest[0] | M |
| 모듈 간섭 | A / NOT_VERIFIED, 가정 치수(보통 경우)로 충돌 0건 | out/fit_checks.json module_keepouts | A |
| 모듈 최소 간격 | 0.5 mm | out/fit_checks.json module_keepouts.clearance_min_mm | F (계획) |
| 모듈 간섭 보통 0.5 mm 미만 | U1 | out/fit_checks.json module_keepouts.modules[].zones[].below_clearance_min | A |
| 모듈 간섭 보수적 충돌 | C3 -0.05 mm (J5_openlog 'insertion stroke beyond the plugged PCB end'), U6 -0.5 mm (J5_openlog 'insertion stroke beyond the plugged PCB end') | out/fit_checks.json module_keepouts.modules[].zones[].below_clearance_min_conservative (<= 0) | A |
| 모듈 간섭 보수적 0.5 mm 미만 | C48, Q2, Q8, U1, U2, Y1 | out/fit_checks.json module_keepouts.modules[].zones[].below_clearance_min_conservative | A |
| 외곽 | 76 x 70 mm | spec/carrier_spec.json | F (size) / A (corner radius) |
| 두께 | 1.6 mm | spec/carrier_spec.json | F |
| 층 수 | 4 | spec/carrier_spec.json | F (계획 불변조건 d) |
| 높이 한계 | 13.4 mm | spec/module_dimensions.json | F (계획) |
| 식별 문구 위치 | 아랫면 실크(B.SilkS, 아랫면에서 볼 때 바로 읽히도록 좌우 반전) | out/reports/routing_rules.json silkscreen.identification_lines | M |
| 식별 문구 크기 | 글자 높이 1.2 mm, 선 굵기 0.2 mm | out/reports/routing_rules.json silkscreen.identification_lines | M |
| 식별 문구 간격 | 패드까지 1.221 mm 이상, 비아까지 1.147 mm 이상, 기판 변까지 4.697 mm 이상, 글자 밑 비아 0개, 케이블타이 띠와 겹침: 없음 | out/reports/routing_rules.json silkscreen.identification_lines | M |
| 식별 문구 검사 | PASS | out/reports/routing_rules.json silkscreen.identification_lines | M |
| 블록 PWR | 27 / 24 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 NAV | 17 / 13 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 FCU | 15 / 13 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 COM | 36 / 31 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 GNS | 18 / 15 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 SHIFT | 18 / 16 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 IO | 37 / 29 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 LOG | 1 / 0 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 블록 LINK | 5 / 3 | spec/carrier_spec.json parts[].block, parts[].fit | M |
| 코드 U8 | C919459 | out/jlcpcb_bom.csv | M |
| 코드 L5 | C53426685 | out/jlcpcb_bom.csv | M |
| 코드 U9 | C20345924 | out/jlcpcb_bom.csv | M |
| UVLO 저항 | R65 470k 1%, R66 270k 1%, R67 3.3M 1% | spec/carrier_spec.json | M |
| 켜짐 전압 | 3.44 ~ 3.73 V | spec/rev3_plan.json power_block.uvlo | E |
| 꺼짐 전압 | 3.00 ~ 3.25 V | spec/rev3_plan.json power_block.uvlo | E |
| ESP 참고 부품 | C82891 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (U7) | F |
| 코드 U7 | BOM에 없음 | out/jlcpcb_bom.csv | M |
| 참고 코드 J2 | C22436147 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (J2) | F |
| 참고 코드 J8 | C32713262 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (J8) | F |
| 참고 코드 J16 | C32713261 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (J16) | F |
| 참고 코드 J18 | C32713263 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (J18) | F |
| 참고 코드 J14 | C5364642 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (J14) | F |
| 참고 코드 J15 | C131337 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (J15) | F |
| 코드 J2 | BOM에 없음 | out/jlcpcb_bom.csv | M |
| 코드 C45 | C11366 | out/jlcpcb_bom.csv | M |
| 배선 세그먼트 | 2557 | kicad/vtol_carrier.kicad_pcb (최종 보드 파일에서 셈) | M |
| 배선 원호 | 0 | kicad/vtol_carrier.kicad_pcb (최종 보드 파일에서 셈) | M |
| 배선 비아 | 442 | kicad/vtol_carrier.kicad_pcb (최종 보드 파일에서 셈) | M |
| 비아 크기별 | 0.6/0.3 mm 439개, 0.8/0.4 mm 3개 | kicad/vtol_carrier.kicad_pcb (지름/드릴) | M |
| 변경 행 | 교체 11, 추가 13, 삭제 1, 그대로 (부품 없음) 10, 그대로 140 | spec/rev3_plan.json changes | F (계획) |
| fit jlc | 144 | spec/carrier_spec.json | M |
| fit jlc_optional | 4 | spec/carrier_spec.json | M |
| fit user | 14 | spec/carrier_spec.json | M |
| fit none | 12 | spec/carrier_spec.json | M |
| main bom_lines_smt | 33 | out/manifest.json bom_cpl.pairs.main | M |
| main bom_lines_tht | 0 | out/manifest.json bom_cpl.pairs.main | M |
| with_mcu bom_lines | 34 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| with_mcu parts | 148 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| U1-U4 with_mcu | C6510 | out/jlcpcb_bom_with_mcu.csv | M |
| 프로그래밍 헤더 선택 | A | spec/carrier_spec.json prog_headers.option | M |
| B안 결론 | 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) | out/reports/programming_access.json | M |
| B안 2x3 자리 | SMD 0곳, THT 0곳 | out/reports/programming_access.json verdict_per_family | M |
| 부품 자리 점유율 | 88.5 % | kicad/placement.json placement_checks.courtyard_occupancy_percent (윗면 부품 자리 면적 / 케이블타이 띠를 뺀 기판 면적) | M |
| JLC 실장 몸체 0.3 mm 안 | 없음 | out/reports/edge_distance.json jlc_placed_bodies.default_order bodies_under_0p3 | M |
| 주문 화면 확인 시각 | 2026-11-12 01:30-02:06 KST | spec/jlc_order_page_20261112.json read_at | M (주문 화면 2026-11-12) |
| 주문 화면 매칭 | 부품 40줄을 인식하고 40줄을 확정 | spec/jlc_order_page_20261112.json files_uploaded_then | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht 합계 | 76.82 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] total_without_shipping | M (주문 화면 2026-11-12) |
| Economic 받은 줄 수 | 39 | spec/jlc_order_page_20261112.json economic_page_behaviour.lines_accepted_count | M (주문 화면 2026-11-12) |
| 스루홀 종류 수 | 6 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (THT) | F |
| LCSC 기록 시각 | 2026-11-12T02:50:37+09:00 ~ 02:51:09+09:00 | spec/lcsc_snapshot_20261112.json records[].fetched_at | M |
| 방향 중요 부품 수 | 30 | out/reports/cpl_rotation.json | M |
| 회전 미확인 | C45, C47, C52, U9 | out/reports/cpl_rotation.json | NOT_VERIFIED |
| 리플로 Economic | 255±5 C | out/fit_checks.json reflow_rating.oven_C | F (계획 10장) |
| 리플로 한계 부품 | C2833309 (C47, 정격 260 C), C2286 (D1/D2/D3/D4, 정격 260 C), C53426685 (L5, 정격 260 C) | out/fit_checks.json reflow_rating.rows (Economic: AT THE LIMIT) | F (데이터시트) |
| C47 리플로 횟수 | 리플로는 한 번만 (두 번은 사이에 30분을 둘 때만) | out/fit_checks.json reflow_rating.rows C47 limits | F (데이터시트) |
| 리플로 미확인 줄 수 | 31 | out/fit_checks.json reflow_rating | NOT_VERIFIED |
| 다리 규칙 초과 | J15, J17 | out/fit_checks.json tht_leads | F (도면값) |
| 뒷면 다리 규칙 | 1.5 mm | out/fit_checks.json tht_leads | F (계획) |
| 다리 공차 끝 초과 | J2, J5, J8~J12, J16, J18 | out/fit_checks.json tht_leads.rows (도면값은 규칙 안, 공차 끝은 규칙 초과; 참고 부품의 도면값) | F (도면값) |
| 다리 길이 미확인 | J1, J14 | out/fit_checks.json tht_leads | NOT_VERIFIED |
| GY-521 받침 높이 | 2.54 mm | spec/module_dimensions.json gy521.support_h | A |
| BMP280 헤더 종류 | 직각 | spec/module_dimensions.json bmp280.header_type | A |
| 계획17 TPS61023 valley current limit | 승압 IC TPS61023의 전류 한계(valley)는 데이터시트에 입력 3.6 V 기준으로만 나옵니다. 입력 3.0 V에서는 1.50 A가 필요하고, 이는 최소 2.7 A의 56 % 입니다 (E, 계획 17장). | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 Cut-off hysteresis against the voltage rebound | 저전압 차단의 히스테리시스 여유는 28 mV뿐입니다 (E). 전원 경로 저항이 0.29 Ohm을 넘으면 방전 끝 최대 부하에서 전원이 켜졌다 꺼졌다 할 수 있어, 첫 보드에서 저항을 재야 합니다 (ORDER 7장 5번). | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 The cut-off acts on the loaded voltage | 차단은 부하가 걸린 전압으로 판단합니다 (필터 16.3 ms). 최대 부하가 이어지면 쉬는 전압이 3.62 V 아래인 팩에서 차단이 걸릴 수 있습니다 (E). | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 Switch-on threshold up to 3.73 V | 켜짐 전압이 최대 3.73 V라서, 그보다 낮게 쉬고 있는 팩으로는 보드가 켜지지 않을 수 있습니다 (E). | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 Start-up into 226.9 uF | +5V 쪽 226.9 uF로 시동할 때 1.9 ms 리셋, 감지 필터와의 순서는 확인하지 않았습니다 (첫 보드에서 오실로스코프로, NOT_VERIFIED). | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 +3V3 headroom | +3V3 여유: +5V 최저 4.844 V에서 AMS1117 강하 최대 1.3 V를 빼면 3.3 V보다 0.244 V 위입니다 (E). Wi-Fi 송신 중 최저값은 확인 안 됨 (ORDER 7장 10번). | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 Drain after cut-off | 차단 뒤에도 새 회로 6.7 uA (최대 8.0 uA)와 R15 / R16 분압 34..62 uA가 흐릅니다. 차단은 보관용 스위치가 아니므로 스위치를 꺼야 합니다 (E). | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 22 uF 1206 capacitors | 22 uF 1206 커패시터의 5 V 직류 바이어스 감소량은 제조사 곡선으로 확인하지 않았습니다 (NOT_VERIFIED). 출력 실효 용량은 4 uF 이상이어야 하며 레일 전체 용량으로 보면 여유가 있습니다 (E). | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 U8 uses a generic connector symbol | U8은 일반 커넥터 심볼을 써서 ERC가 U8 핀에 대한 근거를 주지 않습니다. 대신 패드-네트 표 검사(U8/U9 패드표)가 근거입니다. | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 ESP-12F RF behaviour | ESP-12F의 무선 성능은 같은 금지 영역에서 ESP-12E와 같다고 가정했습니다 (A). 펌웨어 이미지도 같은 ESP8266 / 4 MB 구성으로 가정합니다 (A). | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 Programming access | 프로그래밍: 빈 구멍에 포고핀 어댑터를 댈 때의 접촉은 확인하지 않았습니다 (NOT_VERIFIED). B안(JLC가 띠 밖에 헤더 실장)은 배치 검색에서 자리가 없었습니다. | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 계획17 AVCC choke C1046 | AVCC 초크 C1046은 정격 15 mA (1.15 Ohm) 입니다. ATmega128A의 AVCC 전류와 대조하지 않았습니다 (NOT_VERIFIED). | spec/REV3_PLAN.md 17장 | 계획 17장의 표시 (E / A / NOT_VERIFIED) |
| 빌드 시간 | 8.3분, 9.0분 | out/reports/determinism.json builds | M |
| 결정성 시각 | 2026-11-12T05:24:10+0900 | out/reports/determinism.json | M |
