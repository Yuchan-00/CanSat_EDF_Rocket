# JLCPCB 주문 안내서 - VTOL 캐리어 PCB rev 3.1

이 안내서는 PCB 실장 주문(PCBA)을 처음 하는 사람도 따라 할 수 있게 썼습니다. **이 문서를 만든 스크립트는 아무것도 업로드하거나 주문하지 않습니다. 이 보드는 한 번도 만들어진 적이 없습니다.** 업로드, 확인, 결제는 모두 사용자가 JLCPCB 사이트에서 직접 합니다 (2026-11-12에 사용자의 브라우저에서 이전 빌드의 파일을 주문 화면에 올려 견적까지 본 기록이 2장과 4장에 있습니다. 그 기록에 주문이나 결제는 없습니다).

표시: M = 도구로 측정, E = 추정, A = 가정, F = 문서 근거, NOT_VERIFIED = 근거 없음. 'M (주문 화면 2026-11-12)' 는 그날 사용자의 로그인된 브라우저에서 JLCPCB 주문 화면을 직접 보고 적은 값입니다. 측정·계산된 숫자의 출처는 맨 끝 11장에 있습니다. 그 표에 없는 숫자(시험 전류·전압, 규칙 값, 수량 기본값)는 계획서 `spec/REV3_PLAN.md` 의 괄호 안 장(15장, 16장의 D 번호 등)에서 옮긴 값(F, 계산값은 E)입니다. 이 파일은 `scripts/docs_ko.py` 가 만든 것입니다.

## 0. 먼저 읽을 것: 도구 판정과 2026-11-12 재설계

출고 판정 (도구 검사): **ORDERABLE (tool checks only; NOT_VERIFIED - never built)**. 판정을 막는 항목: 없음.

결정성 검사 (determinism.py, 출고 판정과 따로 봄): PASS - 바이트 비교 파일 71개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 0개. 다른 폴더에서: PASS - 다른 폴더에 복사해 다시 만든 결정적 파일 60개 중 55개가 이 폴더의 것과 바이트까지 같음 (보드, 회로도, 거버와 zip, BOM, CPL 등 주문에 쓰는 파일 43개는 모두 여기에 듦). 나머지 5개(README_ko.md, out/ORDER_ko.md, out/따로_살_부품_ko.md, spec/SPEC_ko.md, spec/parts_sources.json)는 자기 폴더 이름이나 그런 파일의 sha256을 담고 있어, 폴더 이름과 그 sha256을 바꿔 넣은 뒤에 같음. 그림 단계 산출물 11개는 그 폴더에서 다시 만들지 않아 비교하지 않음.

이 판정은 도구 검사(회로도 ERC, 보드 DRC, 배선 규칙 21개, 기체 맞춤, 주문 파일 검사, 오늘의 JLC 재고 확인)를 모두 통과했다는 뜻일 뿐입니다. **보드는 한 번도 만들어지거나 전원이 들어간 적이 없고 (NOT_VERIFIED), 지금 파일은 JLC에 올려 본 적이 없습니다.** 도구가 볼 수 없는 아래 항목은 사용자가 주문 화면에서 확인합니다 (3장):

- JLC upload preview of the current files: every BOM line matched (SMT only since 2026-11-12; no THT line, no hand soldering by JLC)
- JLC 'Confirm Parts Placement': rotation of the parts listed as NOT_VERIFIED in out/cpl_rotation_table.csv and the polarity of C45 / C47 / C52
- JLC DFM result (edge distances; Economic PCBA: where JLC puts its tooling holes, or its request for edge rails)
- jlc_gate.py again on the order day, directly before paying

위 목록은 out/manifest.json 의 release_gate.user_gates_not_covered 를 그대로 옮긴 것입니다 (영어 원문). 특히 C45 / C47 / C52 / U9의 회전·극성은 JLC의 위치 확인 화면에서 직접 확인하고, 주문 당일 결제 직전에 jlc_gate.py를 다시 실행하며, 결제는 사용자가 합니다.

### A. 이번 재설계에서 바뀐 것 (사용자 요청 "그럼 다시 설계를 진행해줘. 실패한게 있다면 완료되었다고 말하지마" / "보드의 사이즈를 조정하는 방법은 없어?" / "옮겨도 됨")

- 재설계 전 보드의 배선 규칙: 16 / 21 PASS, FAIL: battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance (M). 지금: 21 / 21 PASS.
- 외곽: 본체 KiCad x 62..138, y 65..135 mm (76 x 70 mm, 모서리 r 1 mm)는 그대로이고 -y 변에 탭을 붙였습니다: KiCad x 117.3..132, y 135..142.2 mm (바깥 모서리 r 1 mm, 본체와 만나는 안쪽 모서리 r 1 mm, 기판 재료는 x 116.3..133), 넓이 105.8 mm2. 외접 크기 76 x 77.2 mm. 기체 맞춤 (Fit 단계가 기체 CAD에서 잰 값으로 확인, board_fit PASS): 흡입구 보어 벽까지 4.44 mm (기준 2 mm 이상), 탭에서 270도 통풍구까지 13.05 mm (기준 2 mm 이상), 탭에서 모터 리드 연결부 영역까지 21.869 mm (본체 사각형 쪽은 재설계 전과 같음), 315도 조인트 나사에서 평면 2 mm 안의 부품: 없음, 나사 아래 기판 윗면 높이 여유 1.3 mm (기준 1 mm 이상), 탭 안 부품 C41, C57, C58, L5, R63, R64, U8 (높이 한계 13.4 mm 넘는 것: 없음).
- 프로그래밍 구멍 이동 (사용자 승인): J19 (x +0, y +0.55 mm), J22 (x +0, y +0.55 mm), J26 (x +0, y +0.55 mm). 이 세 구멍(빈 도금 구멍, 부품 없음)의 위치가 바뀌었으므로 옛 위치에 맞춰 만든 포고핀 어댑터나 지그가 있다면 맞춰 고쳐야 합니다 (위치: out/reports/programming_access.json bare_holes_option_A). C42 이동: x +0.20 mm (지금 x 72.8, y 69.3 mm).
- 승압부 자리: U8 (126.5, 135.45) 0도; L5 (125.15, 138.75) 0도; C43 (127.1, 133.3) 180도; C57 (129.2, 136.75) 270도; C41 (120, 138.75) 180도; R63 (123.85, 133.85) 270도; C58 (122.35, 133.85) 270도; R64 (120.85, 133.85) 90도.
- 배터리 전원 경로: 저항 0.03096 Ohm (20 C, E; 목표 0.035 Ohm), 가장 좁은 폭 2 mm; J14.1 → Q10.3 0.00191 Ohm, 7.78 mm, 가장 좁은 곳 2 mm, 비아 0개; Q10.2 → J15.1 0.00388 Ohm, 15.77 mm, 가장 좁은 곳 2 mm, 비아 0개; J15.2 → L5.1 0.02517 Ohm, 132.22 mm, 가장 좁은 곳 2 mm, 비아 2개. 60 C에서는 0.03583 Ohm (E)로 목표를 넘습니다 (판정은 20 C 값). 재설계 전: 20 C 0.03173 Ohm, 60 C 0.03672 Ohm, 2.0 mm보다 좁은 길이 8.57 mm.
- 출력 커패시터: C57 +5V 패드-U8.6 0.75 mm, GND 패드-U8.4 1.819 mm; C43 +5V 패드-U8.6 0.868 mm, GND 패드-U8.4 1.827 mm (재설계 전 C57 0.95 / 1.611 mm; C43 3.25 / 3.537 mm). 입력 커패시터: C41 → L5 입력 패드 구리 길이 0.9 mm (기준 5 mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E; U8 VIN 패드와 L5 입력 패드 사이 직선 거리 2.103 mm). SW: U8의 SW 패드와 L5의 SW 패드 사이 간격 1.876 mm (기준 2 mm 이하 - 계획 12장의 'L5는 U8 핀 바로 옆'에 숫자가 없어 스크립트가 계획 12장의 2 mm를 빌려 읽은 값, E), SW 선 구리 넓이 1.061 mm2, 구리 저항 약 4.3 mOhm (20 C, E), SW 선이 패드 사이를 지나는 다른 부품: 없음. 크리스털: C43 Y1 9.55 / XTAL 핀 9.935 mm; C57 Y1 11.851 / XTAL 핀 12.753 mm; U8 Y1 11.925 / XTAL 핀 12.319 mm; L5 Y1 14.65 / XTAL 핀 15.025 mm; C41 Y1 15.762 / XTAL 핀 15.485 mm (rev 2c 기준 Y1 9.217 / XTAL 핀 9.616 mm).

### B. 검사는 통과했지만 목적과 다른 점 (숨기지 않음)

- 입력 커패시터 C41의 GND: C41 GND 패드의 비아 (118.525, 137.2)에서 윗면 GND 선을 따라 13.923 mm (끝점 (127.1, 135.95), (118.525, 138.05); 가는 0.2 mm 선이 FB 분압 저항 두 줄 사이를 지나 C43 GND 패드를 거쳐 U8 GND 핀으로 감), C41 GND 패드-U8.4 직선 간격 8.033 mm. 검사 hot_loop_top_gnd / input_cap_vin은 이 가는 선으로 통과합니다. 데이터시트가 말하는 '입력 커패시터 GND를 IC GND 바로 옆에' 같은 짧은 고리는 아닙니다. C41의 고주파 귀환은 자기 비아(0.65 mm)를 거친 내층 GND입니다.
- C41의 VIN 쪽 경로는 L5 입력 패드를 지나 U8 VIN 핀으로 갑니다.
- 출력 커패시터 C57의 GND는 윗면에서 L5 둘레·아래의 GND 면(구리 채움)으로만 U8 GND 핀과 이어집니다 (SW 선이 사이에 있어 선을 그을 수 없음). 자동 배선이 이 면을 끊지 않도록 Freerouting 입력에만 탭 위 윗면 배선 금지 영역을 두었습니다.
- SW 패드 간격 여유 0.124 mm (부품 자리가 맞닿은 배치로 정해짐).
- 미리 그린 배선(승압부와 배터리 경로의 갈래)은 자동 넓히기에서 빠져 설계한 폭 그대로입니다 (재설계 전 흐름은 넓혔음).
- 배치 검사 place.py는 J26 / J22 / J19만 'rev 2c 자리 + 사용자가 승인한 이동'과 비교합니다. 외곽 맞춤과 추가한 검사 말고 사용자 결정 때문에 바꾼 판정 검사 코드는 이것과, rev 3.1 사용자 결정에 따른 MCU 핀맵·네트 검사의 선언된 핀 하나(U2.7 → RELEASE_PWM)뿐입니다. 그 밖에 바꾼 검사 스크립트: determinism.py의 폴더 이름 바꾸기 규칙 (비교 종류·PASS 규칙은 그대로), place.py의 새 부품 J13 / R22 항목. 전체 목록은 README 3장.
- 입력 커패시터 C41은 L5 입력 패드 옆에 있고 U8 VIN 핀 옆에는 커패시터가 없습니다. 내층 귀환 거리: C43.2 GND 비아 → U8.4 GND 비아 5.15 mm; C57.2 GND 비아 → U8.4 GND 비아 4.02 mm; C41.2 GND 비아 → U8.4 GND 비아 8.33 mm (비아 중심 사이 직선, In1 GND 면 위).
- 출력 커패시터 C43의 GND: C43 GND 패드에서 U8 GND 핀까지 윗면 GND 선 폭 0.2 mm 0.358 mm, 폭 0.25 mm 3.1 mm (비아 연결선 제외; SOT-563 패드 두 줄 사이를 지나는 가는 선; 패드마다 0.3 mm GND 비아 하나씩) (넓은 GND 구리가 아님; U8 발열 NOT_VERIFIED).
- 배터리 경로의 L5 입력 비아: 드릴 0.4 mm 이상 비아 2개의 도금 구리 단면은 합쳐서 약 0.045 mm2 로, 폭 2.0 mm 1 oz 배선의 0.07 mm2 보다 작습니다 (도금 18 um 가정, E). 폭 검사는 선만 잽니다.
- 설계 데이터(IR)는 외곽을 76 x 70 mm 사각형으로만 갖고, 탭이 있는 외곽은 scripts/outline.py 에서 옵니다.
- 이것들이 실제로 문제가 되는지는 보드를 받은 뒤 7장의 측정(켜질 때 VIN, 부하에서 SW와 +5V 과전압, 입력 리플, NAV 칩 시계)으로만 압니다 (NOT_VERIFIED).

### C. 이번 재설계로 새로 생긴 확인 거리 (NOT_VERIFIED)

- 새 거버(탭 있음)는 JLC에 올려 본 적이 없습니다. 주문 화면의 크기·PCB 값·DFM 결과는 올려 봐야 압니다. 4장의 견적은 모두 재설계 전 76 x 70 mm 거버의 것입니다.
- Standard 대안(레일)으로 가면 탭이 있는 변에는 V-cut을 쓸 수 없습니다 (2장).
- 기체 CAD는 아직 탭이 없는 상자로 캐리어를 그립니다. 315도 조인트 나사 길이는 정해지지 않았습니다 (README 5장).
- 조인트 나사 4개 모두 길이 M3x6 이하를 권합니다: 나사 끝이 r 49.5 mm까지 들어온다고 가정하면(A) 다른 나사 옆 부품은 45도 나사 - Q9 1.985 mm, 135도 나사 - J14 -1.393 mm, 225도 나사 - J7 -1.393 mm (재설계 전과 같은 자리의 커넥터 J14 / J7이 길이를 정함; 315도 나사 옆에는 부품 없음).
- 탭 안쪽 모서리 r 1.0 mm를 JLC의 라우터 날이 만들 수 있는지는 근거 문서가 없어 NOT_VERIFIED입니다 (업로드 미리보기에서 확인). 탭 부품 C41 / L5 / C57의 몸체는 가장자리에서 2.5 mm보다 가깝습니다 (README 1장 표) - JLC DFM 결과를 보십시오.
- 알려진 작은 항목 (M, 검사 대상 아님): U9 바이패스 C59의 GND 비아 - 패드 1.5 mm 안에 자기 GND 비아 있음; L5 입력 패드 4 mm 안의 배터리 네트 비아 - 드릴 0.4 mm 2개. 드릴 0.4 mm 이상 비아 2개의 도금 구리 단면은 합쳐서 약 0.045 mm2 로, 폭 2.0 mm 1 oz 배선의 0.07 mm2 보다 작습니다 (도금 18 um 가정, E).

### D. rev 3.1 (2026-11-13): 로켓용 분리 서보 출력 (사용자 결정 "캐리어 PCB를 로켓에도 씀" / "B: 서보 열에 직각 헤더")

- **장바구니의 Y24는 rev 3 거버입니다. 결제하지 마십시오.** 그보다 옛 Y23, Y20도 결제하지 마십시오 (장바구니는 이 작업에서 열지 않음 - 결제 전에 항목별 거버를 직접 확인). rev 3.1 주문은 이 폴더의 새 파일 3개(1장)를 새로 올려야 합니다. 새 거버의 견적과 DFM 결과는 올려 보기 전까지 NOT_VERIFIED이고, 4장의 견적은 모두 그 전 거버의 것입니다.
- 바뀐 것: R22 (220 Ohm, JLC 실장) 추가 → BOM 줄 220R 1%, R17-R22 (6개), C22962. BOM 줄 수는 그대로 33줄이고 CPL에 R22 한 줄이 늘었습니다. J13은 부품 없는 도금 구멍 패드라 BOM / CPL에 없습니다 (KiCad (127.05, 66.85) mm, J12(ESC) 옆 +x / +y 모서리). MCU 핀 변경: U2.7 (PE5 / OC3C): NC → RELEASE_PWM.
- 사용자가 고른 B안(서보 열의 직각 헤더, J18을 VIN 1핀으로)은 rev 3 배터리 선을 바꾸지 않는다는 조건에서는 놓을 수 없음: -x 서보 열에 직각 1x3 J13을 J8~J11과 같은 8.62 mm 간격으로 넣고 J18을 1핀으로 줄이면, 배터리 선 두 개(윗면 2.0 mm VBAT_LOGIC, 아랫면 2.4 mm VBAT_LOGIC_SW) 사이에 45.06 mm가 필요한데 44.60 mm뿐입니다. 801곳을 찾아본 가장 좋은 자리도 열의 양 끝에서 0.13 mm 간격 규칙보다 0.23 mm 모자랐습니다. 배터리 선을 바꾸지 않는다는 조건은 사용자 결정도 검사 기준도 아니고 (검사는 배터리 경로 폭 2.0 mm 이상, 0.035 Ohm 이하만 요구), rev 3.1 작업이 rev 3 재설계 때 build_pcb.py에 깔아 둔 미리 그린 배선을 건드리지 않으려고 스스로 둔 것입니다 (M). 그래서 사용자가 정해 둔 C안으로 만들었고, **J18은 rev 3 그대로 1x4**입니다 (테더·하이브리드용 VTET / VBUF / ISGN 입력과 분압 부품도 그대로 - 잃는 것 없음). B안용 새 헤더(C32713262 1x3)나 1핀 헤더는 사지 않습니다. B안은 배터리 선 구간을 다시 그리면 될 수도 있지만 그것은 사용자가 정할 설계 변경입니다 (README 맨 위 참고; E, NOT_VERIFIED).
- 로켓에 넣는 보드에만 따로 살 것: 서보 Y자 리드선 3핀 2.54 mm (JR / 후타바형) 1개 (로켓에 넣는 보드용) (6장 (g) 묶음). 신호선을 J13에 납땜합니다 (7장 11번).
- 펌웨어와 params는 바꾸지 않았습니다 (분리 핀은 계속 PE5). 분리 서보 동작, Y자 리드, 납땜한 신호선은 NOT_VERIFIED입니다.

- 아래 1~9장은 주문 절차입니다 (Economic PCBA로 SMT 부품만 실장, 무연 HASL; ESP-12F와 스루홀 커넥터는 사용자가 사서 납땜).

## 1. 올릴 파일 3개

**2026-11-12 변경 (사용자 요청 "아, esp도 내가 직접 실장할거야. 그럼 이코노믹으로 진행하고 pcb의 코팅도 무연으로 해줘", "이코노믹으로 스루홀 제외하고 어떤 부품을 따로 구매해야하는건지 리스트를 정리해줘"):** (1) ESP-12F(U7)는 JLC가 아니라 사용자가 직접 납땜합니다. (2) 스루홀 커넥터 12개(J2, J5, J8~J12, J14~J18)도 JLC가 달지 않고 사용자가 사서 납땜합니다. (3) 주문은 Economic PCBA로 하고, JLC는 SMT 부품만 실장합니다 (JLC의 손납땜 없음). (4) 기판 표면처리는 무연 HASL입니다. 아래 파일과 절차는 이 결정대로 다시 만든 것입니다. 따로 살 부품은 6장과 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/따로_살_부품_ko.md` 에 정리했습니다.

기본 (MCU 4개, ESP-12F, 스루홀 커넥터는 사용자가 직접 납땜):

| 순서 | 파일 | 어디에 올리나 | sha256 앞부분 |
|---|---|---|---|
| 1 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier_gerber.zip` | PCB 주문 첫 화면 'Add gerber file' | 6a1e22caf6ce1ae4... |
| 2 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom.csv` | PCB Assembly 단계 'BOM file' | 2f4827ad0b5fe97c... |
| 3 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl.csv` | PCB Assembly 단계 'CPL file' (Pick and Place) | 9cf2527f4505e309... |

다른 선택 (MCU 4개는 JLC가 실장, 결정 D4의 권장안; ESP-12F와 스루홀 커넥터는 이 묶음에서도 사용자가 납땜): 아래 3개를 **한 묶음으로** 올립니다.

| 순서 | 파일 | sha256 앞부분 |
|---|---|---|
| 1 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier_gerber_with_mcu.zip` | b9246444b0348e2a... |
| 2 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom_with_mcu.csv` | a00f1407a2825cd1... |
| 3 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl_with_mcu.csv` | fa780830e806c36c... |

- **두 묶음을 섞지 마십시오.** 두 zip은 F.Paste(솔더 페이스트) 층만 다릅니다: 기본 zip은 U1~U4 패드에 페이스트 구멍이 없고 (페이스트 패드 322개), with_mcu zip은 있습니다 (578개). **두 zip 모두 ESP-12F(U7) 패드에는 페이스트 구멍이 없습니다** (U7 패드 22개 중 기본 22개, with_mcu 22개를 뺌, M): 사용자가 납땜할 패드는 무연 HASL 코팅만 된 채로 옵니다. 페이스트를 뺀 자리: 기본 J1, J2, J3, J5, J8, J9, J10, J11, J12, J13, J14, J15, J16, J17, J18, J19, J20, J21, J22, J24, J25, J26, JP1, NT1, Q3, R47, U1, U2, U3, U4, U7; with_mcu J1, J2, J3, J5, J8, J9, J10, J11, J12, J13, J14, J15, J16, J17, J18, J19, J20, J21, J22, J24, J25, J26, JP1, NT1, Q3, R47, U7 (JLC가 부품을 달지 않는 자리; 스루홀 커넥터 자리는 원래 페이스트 패드가 없는 도금 구멍입니다).
- 기본 BOM: 33줄 (SMT 33줄, 스루홀 0줄), 부품 145개, CPL 145행. with_mcu: 34줄 (스루홀 0줄), 부품 149개, CPL 149행.
- 기본 BOM에 MCU(U1~U4)가 들어 있는지: 없음. ESP-12F(U7): BOM에 없음. 스루홀 커넥터 J2 / J5: BOM에 없음, J8~J12: BOM에 없음, J14: BOM에 없음, J15 / J17: BOM에 없음 (두 BOM 모두). 센서 자리 J1과 프로그래밍 단자도 들어 있지 않습니다 (모두 사용자 몫).
- 파일 형식: 2026-11-12 01:30-02:06 KST의 주문 화면은 이 변경 전 빌드의 BOM / CPL을 CSV 그대로 받아들였고 부품 40줄을 인식하고 40줄을 확정했습니다 (M, 주문 화면 2026-11-12). 지금 파일은 같은 형식에서 ESP-12F와 스루홀 커넥터 줄이 빠진 것이지만, **지금 파일을 올려 모든 줄이 매칭되는 것을 볼 때까지는 NOT_VERIFIED입니다.**
- 두 거버 zip 모두 기판 아랫면 실크(B.SilkS, 아랫면에서 볼 때 바로 읽히도록 좌우 반전)에 깃 저장소 주소와 연락용 이메일 주소 두 줄이 들어 있습니다 (글자 높이 1.2 mm, 선 굵기 0.2 mm, 검사 PASS); 거버 미리보기의 그 면에서 두 줄이 끊기지 않고 보이는지 확인하십시오 (JLC 인쇄 품질은 NOT_VERIFIED).
- 2026-11-12 재설계로 외곽(탭), 승압부 배치, 프로그래밍 구멍 3개, C42, 배선이 바뀌었습니다 (0장 A). 회로도와 BOM 줄은 그대로이고, CPL은 옮긴 부품의 좌표가 바뀌었습니다. 배선은 재설계 때 기록한 세션을 다시 넣어 만든 것이고, 재생에 쓴 배선 전 보드는 기록된 보드와 'byte-identical to the recorded unrouted board' 입니다 (M). 보드 파일의 표면처리 표기: 'HAL lead-free'.

## 2. 주문 화면 입력표 (결정 D9, D10, D11, D14, D16)

아래 항목 이름과 선택지는 2026-11-12 01:30-02:06 KST에 사용자의 로그인된 브라우저에서 JLCPCB 주문 화면을 직접 보고 적은 것입니다 (M, 주문 화면 2026-11-12; 기록 파일 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/jlc_order_page_20261112.json`). 이 도구(스크립트)가 화면을 열거나 로그인한 것은 아닙니다. 그때 화면에 올라가 있던 파일은 거버 zip 5e7d74ec..., BOM fa4c8e26..., CPL 738c4ea4... 로, ESP·스루홀 변경과 재설계 전 빌드의 파일(BOM 40줄, 탭이 없는 76 x 70 mm 거버)입니다. 화면은 거버를 'Detected 4 layer board of 70x76mm' 로 읽었고, BOM / CPL에서 부품 40줄을 인식하고 40줄을 확정했습니다 (모든 줄이 BOM에 적은 JLC 번호 그대로 매칭, 부족·대체 제안 없음). Economic 견적은 그 화면에서 ESP-12F와 스루홀 줄을 실장 목록에서 뺀 상태로 봤습니다 (4장). **지금 파일(스루홀과 ESP-12F가 처음부터 없는 BOM)은 그 뒤에 만들어져 아직 올려 보지 않았습니다 (NOT_VERIFIED).**

| 화면 항목 | 넣을 값 | 근거·화면에서 본 것 |
|---|---|---|
| **PCB** | | |
| Base Material | FR-4 | 화면에 보인 값 |
| Layers | 4 | 보드 4층 (거버 검사 M) |
| Dimensions | 거버에서 자동으로 읽힘 | 거버를 올리면 읽힘. 재설계 전 거버는 'Detected 4 layer board of 70x76mm' 로 읽혔습니다 (M). 새 거버는 탭 때문에 외접 사각형 76 x 77.2 mm 로 읽힐 것으로 봅니다 (E; 본체 76 x 70 mm + 탭). JLC는 이 크기로 PCB 값을 매기므로 값이 달라질 수 있습니다 (새 거버로는 견적을 받지 않았음: NOT_VERIFIED). 화면 값이 외접 크기와 같은지 확인 |
| PCB Qty | 5 | D11 (PCB 5장 중 2장 실장) |
| Delivery Format | Single PCB | 화면에 보인 값 |
| PCB Thickness | 1.6mm | 사양 1.6 mm |
| PCB Color | Green | D16 |
| Silkscreen | White | D16 |
| Material Type | FR4 TG135 | 화면에 보인 값 |
| Surface Finish | LeadFree HASL | D16, 2026-11-12 사용자 결정 (무연). 선택지: OSP, HASL(with lead), LeadFree HASL, ENIG. 보드 파일에는 'HAL lead-free', 거버 zip 안의 job 파일에는 'HAL lead-free' 로 적혀 있습니다 (M). 주문은 화면에서 고른 값이 정하므로 화면에서 반드시 고르십시오 (JLC가 job 파일을 따르는지는 NOT_VERIFIED) |
| Outer Copper Weight | 1 oz | D16 (JLC 기본 4층 적층) |
| Inner Copper Weight | 0.5 oz | D16 (JLC 기본 4층 적층) |
| Via Covering | Plugged | D16. 선택지: Tented, Untented, Plugged, Epoxy Filled & Capped (the list on the page is longer). 화면에서 본 것: 이 4층 기판에서 LeadFree HASL을 고르면 Tented가 막히고 Plugged(0 USD)가 선택됨 (말풍선 'The Via Tented option is upgraded to Via Plugged for free'). 화면이 정한 대로 둠 (M) |
| Min via hole size/diameter | 0.3mm/(0.4/0.45mm) | 화면에 보인 값. 보드의 비아(지름/드릴): 0.6/0.3 mm 449개, 0.8/0.4 mm 3개 (M) |
| Board Outline Tolerance | ±0.2mm | 화면에 보인 값 |
| Confirm Production file | No | 화면에 보인 값 |
| Mark on PCB | Remove Mark | 화면에 보인 값 그대로 둠 (D16: 기본값) |
| Electrical Test | Flying Probe Fully Test | 화면에 보인 값 |
| **PCB Assembly** | | |
| PCBA Type | Economic | D9, 2026-11-12 사용자 결정. 선택지: Economic, Standard. 아래 'Economic으로 주문하면 달라지는 것' |
| Assembly Side | Top Side | 모든 부품이 윗면 (사양) |
| PCBA Qty | 2 | D11. 선택지: 5, 2 (for 5 PCBs) |
| Tooling holes | Added by JLCPCB | D10: 설계에 툴링 홀 없음 (H 부품 0개, M). 선택지: Added by JLCPCB, Added by Customer. Economic을 고르면 레일 칸 대신 이 칸이 나옴 (M) |
| Confirm Parts Placement | Yes + 'Do not confirm automatically' 체크 | 회전이 NOT_VERIFIED인 부품이 있음 (3장). Yes를 고르면 안내 창('confirm within 72 hours, otherwise automatically confirmed')이 뜨고 그 창에 'Do not confirm automatically' 체크 칸이 있음 (M) |
| Parts Selection | By Customer (Self-Service) | 화면에 보인 값 |
| **PCB Assembly 추가 옵션** | | |
| Bake Components | 고를 수 없음 (Economic에는 없는 항목) | D14: 주문하지 않음. Economic PCB Assembly에서는 이 항목을 고를 수 없음 (M). Standard에서는 Yes를 고르면 7.94 USD가 검토 뒤 청구됨 (M) |
| Depanel boards & edge rail before delivery | 화면에 보인 대로 둠 | Economic 주문에는 레일이 없음 |
| Solder Paste | High temp. | 화면에 보인 값 |
| Nitrogen reflow soldering | Yes (for Economic) | 화면에 이렇게 표시되고, 견적에 0.86 USD 줄로 들어감 (M) |
| Assembly remark | Yes | 선택지: Yes, No. Yes를 고르면 메모 칸이 열림: 500자 한도, 'Quote after review'. Economic에서도 Yes로 둘 수 있음 (M). 아래 메모를 붙여넣기 |
| Photo Confirmation | 고를 수 없음 (Economic에는 없는 항목) | Economic PCB Assembly에서는 고를 수 없음 (M) |
| Board Cleaning | 화면에 보인 대로 둠 | 화면에 보인 값 |
| Flying Probe Test | 화면에 보인 대로 둠 | 화면에 보인 값 |
| Function test | 고를 수 없음 (Economic에는 없는 항목) | Economic PCB Assembly에서는 고를 수 없음 (M) |

- 'Save to cart' 전에 품목 설명(통관 분류)을 골라야 합니다 (M). 2026-11-12 01:30-02:06 KST에는 'DIY Hobby Circuit Board - HS Code 902300' 을 골랐습니다.
- 장바구니에 담은 항목의 'Edit Order' 화면은 열 때마다 장바구니에 저장된 값을 다시 불러옵니다 (M). 값을 고친 뒤에는 저장됐는지 다시 열어 확인하십시오.

### Economic으로 주문하면 달라지는 것 (D9)

- **레일 대신 툴링 홀 (NOT_VERIFIED):** Standard는 JLC가 기판에 공정 레일을 붙이고 툴링 홀과 기준점을 레일에 둡니다 (화면 문구: 'The board size is modified to be 76mm*80mm due to adding two 5mm edge rails on the shorter sides', M). Economic에는 레일이 없어 JLC가 기판 안에서 툴링 홀 자리를 찾아야 합니다 ('Tooling holes: Added by JLCPCB'). 이 기판은 윗면 부품 자리 점유율이 86.9 % 이고 (M), 부품 자리 33곳이 기판 끝에서 2.5 mm 안에 있습니다 (M). 설계에는 툴링 홀이 없습니다. 계획 단계에서 rev 2c 보드의 네 모서리를 찾아봤을 때 한 모서리에는 빈자리가 없었고, 계획은 JLC가 구리가 있는 곳에 구멍을 낼 수도 있다고 적었습니다 (계획 16장 D10, F). **JLC가 구멍 자리를 찾는지, 구리(내층 면 포함)를 건드리지 않는지, 아니면 레일을 요구하는지는 주문 뒤 JLC의 검토로만 알 수 있습니다.** JLC가 구멍 위치를 묻거나 레일을 요구하면 멈추고 아래 Standard 대안을 검토하십시오.
- **가장자리 거리 (JLC가 실장하는 SMT 부품 145개만 다시 계산, M):** 기판 끝에서 가장 가까운 몸체는 R60 0.537 mm 이고, 가까운 순서로 R60 0.537 mm, U6 0.55 mm, U9 0.589 mm, J7 0.805 mm, C60 0.95 mm 입니다. Economic 권장 거리 0.3 mm (계획 10장, F)보다 가까운 JLC 실장 몸체: 없음 (패드: 없음). 0.3 mm보다 가까운 자리(U7 0.15 mm, J15 0.29 mm)는 모두 사용자가 납땜하는 부품이라 JLC 실장과 무관합니다. 2.5 mm 안에 있는 JLC 실장 몸체는 13개입니다 (with_mcu 14개). JLC가 받아 주는지는 DFM 결과로만 압니다 (NOT_VERIFIED).
- **리플로 온도:** Economic은 최고 255±5 C, Standard는 240±5 C 입니다 (계획 10장, F). 데이터시트 정격이 Economic 최악값과 같아 **여유가 없는('한계') 부품**: C2833309 (C47, 정격 260 C), C2286 (D1/D2/D3/D4, 정격 260 C), C53426685 (L5, 정격 260 C) (F). 전해 커패시터 C47의 데이터시트는 리플로는 한 번만 (두 번은 사이에 30분을 둘 때만) 허용합니다 (F): 한 면 실장이라 JLC 공정은 한 번이지만, 받은 뒤 열풍기로 주변을 다시 달구는 재작업은 피하십시오. 문서에서 리플로 정격을 찾지 못한 부품 번호는 31개입니다 (NOT_VERIFIED).
- **부품 종류:** 2026-11-12 01:30-02:06 KST의 화면에서 PCBA Type을 Economic으로 두었을 때 ESP-12F를 뺀 39줄이 모두 받아들여졌고 'Standard only' 안내는 없었습니다 (M, 이전 업로드 기준). 지금 BOM 33줄은 그중 SMT 줄입니다 (Extended 10종). 지금 파일로 매칭되는지는 올려 봐야 압니다 (NOT_VERIFIED).
- **베이킹 (D14):** 주문하지 않습니다. 주문 화면에서 Economic PCB Assembly는 Bake Components를 고를 수 없습니다 (M). 습기에 민감한 ESP-12F는 이제 사용자가 납땜하고, JLC가 실장하는 것 중에는 LED (C2286)가 남습니다. with_mcu 묶음이면 ATmega128A (C6510)도 습기 민감 등급 MSL 3 (F: JLC 부품 기록) 입니다. 베이킹이 꼭 필요하다고 보면 Standard 대안을 검토하십시오.
- **Economic에서 고를 수 없는 다른 항목 (M):** Photo Confirmation, Function test. 이 주문에는 필요하지 않습니다.
- **비용:** 4장.

### JLC가 Economic을 거절하면: Standard 대안

PCBA Type을 Standard로 바꾸고, Tooling holes 칸 대신 나오는 'Edge Rails/Fiducials' 을 'Added by JLCPCB' 로 둡니다. 화면은 'The board size is modified to be 76mm*80mm due to adding two 5mm edge rails on the shorter sides' 라고 알려 줍니다 (M) = 긴 변 두 곳에 5 mm 레일. 그 두 변(y 135 쪽은 탭의 모든 변 포함)의 구리 이격은 0.4 mm (M)로, JLC 기준(F) 라우팅·마우스바이트 0.2 mm 이상 에는 여유가 있습니다. **재설계로 y 135 쪽 변에 탭이 생겨 그 변은 곧은 선이 아니므로 V-cut 레일은 쓸 수 없습니다** (V-cut 기준 0.4 mm 이상 와 무관). 라우팅 탭 / 마우스바이트 레일을 JLC가 해 주는지와 그 값은 NOT_VERIFIED입니다. 그래서 메모 끝에 아래 한 문장을 덧붙입니다. 같은 조건(무연 HASL, 스루홀 없음, 재설계 전 76 x 70 mm 거버)의 Standard 견적은 PCBA 111.64 USD, PCB 포함 124.84 USD로 Economic보다 48.02 USD 비쌌습니다 (M, 4장). Standard는 리플로가 240±5 C 로 낮고 베이킹을 고를 수 있으며, 받은 뒤 레일을 꺾어 떼야 합니다.

```
If edge rails are needed: routed tabs / mouse bites please, no V-cut (one long edge has a tab; copper is 0.4 mm from both long edges).
```

메모 문장 (기본 묶음, Assembly remark 칸에 그대로 복사; 312자 / 한도 500자, Standard 문장을 덧붙이면 447자):

```
SMT assembly only, top side. Please fit NO through-hole part and do NO hand soldering: every through-hole connector is bought and soldered by the customer. NOT assembled on purpose (no BOM/CPL rows) - please leave these footprints empty and add no parts: U1-U4, U7, J1-J3, J5, J8-J22, J24-J26, JP1, NT1, Q3, R47.
```

with_mcu 묶음의 메모 (350자, Standard 문장을 덧붙이면 485자):

```
SMT assembly only, top side. Please fit NO through-hole part and do NO hand soldering: every through-hole connector is bought and soldered by the customer. U1-U4 (in this BOM/CPL) are to be assembled. NOT assembled on purpose (no BOM/CPL rows) - please leave these footprints empty and add no parts: U7, J1-J3, J5, J8-J22, J24-J26, JP1, NT1, Q3, R47.
```

- 메모가 말하는 것: (1) 윗면 SMT 실장만 할 것, (2) 스루홀 부품은 하나도 달지 말고 손납땜도 하지 말 것 (스루홀 커넥터는 고객이 사서 납땜), (3) 일부러 비워 둔 자리 목록 - 그 자리에 부품을 달지 말 것. 목록은 사양 파일에서 JLC가 실장하지 않는 자리를 모두 읽어 만든 것입니다. 스크립트는 메모가 500자를 넘으면 문서를 만들지 않고 멈춥니다.
- 예전 메모에 있던 'THT는 JLC가 납땜', 'J8~J12 / J14 / J16의 1번 핀 번호가 반대이니 돌리지 말 것' 문장은 뺐습니다: JLC가 그 부품을 놓지 않으므로 해당이 없습니다. 커넥터 방향과 J14 극성 주의는 사용자 납땜 안내(7장)로 옮겼습니다.

## 3. 결제하기 전에 확인할 것 (하나라도 다르면 멈추고 결제하지 마십시오)

1. 주문 당일, 업로드 직전에 재고를 다시 확인합니다: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts` 에서 `B:/Claude/.venv/Scripts/python.exe -X utf8 -B jlc_gate.py` (인터넷, 읽기만, 약 2~3분). 결과 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlc_gate_report.json` 의 status가 PASS여야 합니다. 마지막 확인: 2026-11-13T20:26:50+09:00 PASS (33/33줄), with_mcu PASS.
2. BOM 매칭 화면: 기본 묶음이면 33줄 모두, with_mcu면 34줄 모두 부품이 매칭되어야 합니다. '부품 없음', '다른 부품 추천', '재고 부족', 'Economic에서는 쓸 수 없음' 이 하나라도 있으면 멈춥니다 (대체는 5장 표 1의 정해진 대체 부품만, 그것도 다시 확인 후). **목록에 ESP-12F (C82891)나 스루홀 커넥터(C22436147 (J2/J5), C32713262 (J8~J12), C5364642 (J14), C131337 (J15/J17), C32713261 (J16), C32713263 (J18))가 보이면 옛 파일을 올린 것이니 멈춥니다.**
3. 견적에 손납땜 줄(Hand-soldering labor fee)과 수작업 조립 줄(Manual Assembly)이 **없어야** 합니다. 있으면 스루홀 부품이 실장 목록에 들어간 것이니 멈춥니다. 스루홀을 뺀 Economic 견적에는 두 줄이 없었고, 넣은 견적에는 3.61 USD와 1.35 USD로 있었습니다 (M).
4. 부품 위치 미리보기(Confirm Parts Placement)를 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/cpl_rotation_check.png` 그림과 하나씩 비교합니다. 특히 극성: 탄탈 C45 / C52, 전해 C47 (거꾸로면 화재 위험), 그리고 회전이 확인 안 된 C45, C47, C52, U9. U8 / U9 / J7의 1번 핀 방향. **U7 (ESP-12F), U1~U4, 스루홀 커넥터 자리(J2, J5, J8~J12, J14~J18)는 비어 있어야 합니다.** 회전 근거는 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/cpl_rotation_table.csv` 의 status 칸에 있습니다. 미리보기 화면에는 'The preview for reference only. Check final part placement at DFM Analysis in Order History in 4-6 hours after placing the order.' 라고 적혀 있습니다 (M): **최종 위치는 주문 뒤 4~6시간 후 Order History의 DFM Analysis에서 다시 확인합니다.** 2026-11-12 01:30-02:06 KST의 미리보기(이 변경 전 파일, 낮은 확대, 눈으로 봄)에서는 C45 / C47 / C52의 '+' 표시가 기판 실크의 '+' 와 같은 쪽에, U8의 1번 핀 점이 실크 표시 자리에 있었습니다 (M). U9 는 그 확대로 구분하지 못했습니다 (NOT_VERIFIED). JLC 풋프린트와 1번 핀 번호가 반대로 매겨진 부품(지금 CPL 기준): 없음 - 그런 부품이었던 J8~J12, J14, J16은 이제 CPL에 없습니다.
5. DFM 결과: 툴링 홀 위치(기판 안, 구리를 건드리지 않는지), 가장자리 가까운 부품(R60 0.537 mm 등)에 대한 이의, 구멍·간격·선폭 지적이 없어야 합니다. JLC가 툴링 홀 자리가 없다며 레일을 요구하면 2장의 Standard 대안으로 견적을 다시 받아 비교한 뒤 정합니다. JLC가 이의를 달면 주문 방식이나 부품을 마음대로 바꾸지 말고 멈춘 뒤 다시 검토합니다.
6. 견적 금액을 4장과 비교합니다 (재설계 전 76 x 70 mm 거버의 기본 묶음: 배송 제외 76.82 USD, M). 지금 거버는 외접 76 x 77.2 mm 라 PCB 값이 다를 수 있습니다 (NOT_VERIFIED). 크게 다르면 이유를 확인합니다.
7. 'Confirm Parts Placement'는 Yes로 하고, 뜨는 창에서 'Do not confirm automatically'를 체크합니다 (2장). 체크하지 않으면 자동으로 확정됩니다 (화면 안내: 'confirm within 72 hours, otherwise automatically confirmed', M). 주문 뒤 JLC가 위치 확인 사진이나 문제를 보내면 4번과 같은 기준으로 비교하고, 틀린 것이 있으면 바로잡아 달라고 답하거나, 판단이 서지 않으면 주문 보류(hold) 또는 취소를 요청한 뒤 다시 검토합니다. 결제와 이 확인의 정확한 순서는 주문 흐름을 끝까지 거친 적이 없어 NOT_VERIFIED입니다.
8. 메모(Assembly remark)를 넣으면 화면에 'Quote after review' 라고 나옵니다 (M): 검토 뒤 금액이 바뀔 수 있으니 최종 금액을 다시 봅니다.

## 4. 비용: 주문 화면에서 본 견적 (M)

**아래 견적은 모두 재설계 전 76 x 70 mm 거버로 받은 것입니다. 지금 파일(탭 포함, 외접 76 x 77.2 mm)의 값은 NOT_VERIFIED입니다** (PCB 크기가 커져 PCB 값이 달라질 수 있고, 실장 부품 줄 수는 같습니다).

아래 네 열은 모두 2026-11-12 01:30-02:06 KST에 주문 화면에서 본 견적입니다 (M, 주문 화면 2026-11-12). 공통 조건: PCB 5장 / 실장 2장, 배송비·세금 제외, 단위 USD. '-' 는 그 견적에 그 줄이 없었다는 뜻입니다. **이번 주문은 첫 열 (Economic, 스루홀 없음)입니다.**

| 항목 | **이번 주문**: Economic, 스루홀 없음 | Economic, 스루홀 6줄을 JLC가 납땜 | Standard, 스루홀 없음 | Standard, ENIG, 처음 올린 그대로 |
|---|---|---|---|---|
| 조건 | Economic, LeadFree HASL, 스루홀 0줄 실장, 부품 33줄 | Economic, LeadFree HASL, 스루홀 6줄 실장, 부품 39줄 | Standard, LeadFree HASL, 스루홀 0줄 실장, 부품 33줄 | Standard, ENIG, 스루홀 6줄 실장, 부품 40줄 |
| 셋업 (Setup fee) | 8.24 | 8.24 | 25.75 | 25.75 |
| 스텐실 (Stencil) | 1.55 | 1.55 | 8.27 | 8.27 |
| 부품값 (Components) | 20.32 | 21.69 | 23.23 | 30.37 |
| Extended 부품 수수료 (Extended components fee) | 30.90 | 30.90 | - | - |
| 피더 수수료 (Feeders loading fee) | - | - | 51.15 | 52.70 |
| SMT 장착 (SMT Assembly) | 1.40 | 1.40 | 2.29 | 2.36 |
| 부품 위치 확인 (Confirm Parts Placement) | 0.45 | 0.45 | 0.45 | 0.45 |
| 손납땜 인건비 (Hand-soldering labor fee) | - | 3.61 | - | 3.61 |
| 수작업 조립 (Manual Assembly) | - | 1.35 | - | 1.35 |
| 질소 리플로 (Nitrogen reflow soldering) | 0.86 | 0.86 | - | - |
| 포장 (Packaging) | - | - | 0.50 | 0.50 |
| **PCBA 합** | **63.72** | **70.05** | **111.64** | **125.37** |
| PCB 5장 | 13.10 (special offer 8.00 + 표면처리 5.10) | 13.10 (special offer 8.00 + 표면처리 5.10) | 13.20 | 25.30 (special offer 8.00 + 표면처리 17.30) |
| **합계 (배송 제외)** | **76.82** | **83.15** | **124.84** | **150.67** |
| 조립 기간 (화면 표시) | 3-4일 | 3-4일 | 3-4일 | 5-6일 |

- 견적을 본 방법: 화면에는 이 변경 전 빌드의 파일(BOM 40줄)이 올라가 있었고, 표면처리를 LeadFree HASL로 바꾸고 ESP-12F(와 열에 따라 스루홀 6줄)를 실장 목록에서 뺀 상태로 읽었습니다. 넷째 열만 처음 올린 그대로(ENIG, ESP-12F와 스루홀 포함)입니다. 지금 BOM은 33줄로 첫 열의 부품 줄 수와 같지만, **지금 파일을 올려서 받은 견적은 아닙니다 (NOT_VERIFIED).**
- 넷째 열의 PCBA 항목을 더하면 125.36 이고 화면의 합은 125.37이었습니다 (차이는 화면 반올림).
- 스루홀 6줄을 JLC에 맡길 때와의 차이: Economic 기준 합계 6.33 USD (M, 첫 열과 둘째 열의 차). 같은 커넥터를 LCSC에서 2장 분 사면 약 4.56 USD 입니다 (E, 배송비 제외; 2026-11-12T02:50:37+09:00 ~ 02:51:09+09:00의 LCSC 기록). 이번 주문에서는 사용자가 사서 납땜하는 쪽으로 정했습니다.
- 표면처리: 무연 HASL의 추가 요금은 5.10 USD 이었습니다 (M; ENIG는 넷째 열의 PCB 칸).
- 수수료 표로 계산한 추정(E)과 비교: 지금 BOM의 Economic PCBA 추정은 65.13 USD (Extended 10종 수수료 30.70 USD 포함; 스루홀 줄이 없어 손납땜 수수료는 0)이고, 화면의 PCBA 합은 63.72 USD입니다. 추정표는 질소 리플로 줄을 모르고 부품값 계산 방식이 달라 차이가 납니다. 전체 추정표: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlc_fee_estimate.csv` (E, 참고용).
- **빠진 것 / 바뀔 수 있는 것:** 배송비, 세금, 메모 검토('Quote after review') 뒤의 금액 변동, 주문일의 부품 단가. 실제 금액은 결제 직전 화면의 견적뿐입니다.
- Economic은 베이킹을 고를 수 없습니다 (2장). Standard에서 베이킹을 넣으면 7.94 USD가 더해집니다 (M).

## 5. 재고가 적은 줄과 대체 부품

- 얇은 재고(기준 1000개 미만): C20345924 (U9) available 597.
- 지켜볼 줄 (2000개 미만): C8598 (D6) 184940개, 필요 28, 재고가 계획 때 725265개에서 361060개로 절반 넘게 줄어듦, C53426685 (L5) 1058개, 필요 22, C919459 (U8) 4431개, 필요 21, 재고가 계획 때 31489개에서 5278개로 절반 넘게 줄어듦, C20345924 (U9) 597개, 필요 20.
- 대체는 계획에 적힌 순서대로만 씁니다. 다른 부품으로 바꾸는 것은 설계 변경이라 다시 검사하고 사용자가 정해야 합니다.

**표 1. JLC가 실장하는 부품의 대체 부품** (이 파일 그대로, BOM 줄의 부품 번호만 바꿈). 'PASS'는 주문 가능 여부(재고)만 뜻합니다.

| 원래 부품 | 대체 부품 (주문 가능 수량, 판정) | 풋프린트 호환 | 계획 메모 (spec/rev3_plan.json) |
|---|---|---|---|
| L5 (C53426685) | C167203 FXL0420-1R0-M (129406개, PASS) | 같은 패키지 (F, 계획 메모); 미리보기로 확인 | same series and land pattern, Isat 5.6 A (first plan's part), stock 107845 |
| U8 (C919459) | C1852149 TPS61023DRLT (464개, PASS) | 같은 패키지 (F, 계획 메모); 미리보기로 확인 | same MPN, other reel; stock 550 (thin) |
| J7 (C161692) | C52037397 GH1.25-4PLT (1679개, PASS); C2829262 GH1.25-4PLTPZ (55682개, PASS) | **NOT_VERIFIED** - 쓰기 전에 제조사 도면과 KiCad 풋프린트를 대조해야 함 | other makers' GH 1.25 mm 4P vertical; footprint check required before use |
| U9 (C20345924) | 없음 - 같은 사양의 두 번째 부품이 없습니다. 재고가 없으면 주문을 미룹니다 (계획 7장, F) | - | - |

**표 2. 사용자가 사는 스루홀 부품의 대체 부품** (JLC 주문과 무관; 구하기 어려울 때의 계획 순서. 수량은 JLC 조립 창고의 것이고 LCSC 소매 재고가 아닙니다).

| 원래 부품 | 대체 부품 (JLC 창고 수량, 판정) | 풋프린트 호환 | 계획 메모 |
|---|---|---|---|
| J2, J5 (C22436147) | C55629861 SH-PM254R-1x6P-H85 (985개, PASS); C54876735 PM2.54-1x6PWZ-H8.5 (560개, PASS) | **NOT_VERIFIED** - 사기 전에 제조사 도면과 KiCad 풋프린트를 대조해야 함 | same KiCad footprint assumed for both (no datasheet for C55629861: NOT_VERIFIED); C55629861 tails 3.2 mm -> protrusion about 1.6 mm, flush-cut |

- 계획에는 'JLC가 스루홀 손납땜을 거절할 때'를 위한 SMD 예비안(J8..J12, J16, J18 (THT right-angle headers) if JLC refuses hand soldering; J15, J17 (C131337))이 있었습니다. 이제 JLC가 스루홀을 달지 않으므로 이번 주문과 무관하고, 새 풋프린트가 필요한 부품이라 이 파일로는 어차피 쓸 수 없습니다.

## 6. 따로 사야 하는 것 (JLC는 낱개 부품을 보내주지 않습니다)

목록 28줄. 읽기 쉬운 전체 목록(비고, LCSC 재고·가격 포함): `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/따로_살_부품_ko.md`; 같은 내용의 CSV: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/off_board_purchase.csv`. (a) 묶음 = JLC BOM에서 빠져 이번에 사용자가 사게 된 ESP-12F와 스루홀 커넥터 6종 (보드당 커넥터 12개). (a) 묶음을 LCSC에서 2장 분 사면 약 10.39 USD (E; 단가는 2026-11-12T02:50:37+09:00 ~ 02:51:09+09:00의 LCSC 기록 M, 최소 주문 수량 반영, 배송비 제외; 그 시각에 재고가 모자란 것: 없음). LCSC 말고는 재고와 가격을 확인하지 않았습니다 (NOT_VERIFIED).

| 묶음 | 품목 | 부품 번호 | 보드당 | 2장 분 | 어디에 | 구입처 (제안) |
|---|---|---|---|---|---|---|
| (a) JLC BOM에 있다가 이번에 사용자가 직접 사서 납땜하게 된 부품 | ESP-12F Wi-Fi 모듈 (Ai-Thinker ESP8266MOD, 플래시 4 MB, 기판 안테나) | ESP-12F(ESP8266MOD) / LCSC C82891 | 1 | 2 | U7 자리 (옆면 홈 패드 22개를 직접 납땜; 패드에 솔더 페이스트 없음) | LCSC C82891 (참고 부품) 또는 국내 쇼핑몰 - 두 주문 파일 묶음 모두에서 필요 |
| (a) JLC BOM에 있다가 이번에 사용자가 직접 사서 납땜하게 된 부품 | 1x6 직각(ㄱ자) 핀 소켓 2.54 mm (암) | HC-PM254-8.5H-1x6PW / LCSC C22436147 | 2 | 4 | J2 (BMP280 모듈의 ㄱ자 6핀 헤더가 꽂힘), J5 (ㄱ자 헤더를 납땜한 OpenLog가 꽂힘) - 보드당 납땜 12곳 | LCSC C22436147 (JLC가 실장하려던 바로 그 부품) 또는 같은 부품 번호를 파는 국내 쇼핑몰 |
| (a) JLC BOM에 있다가 이번에 사용자가 직접 사서 납땜하게 된 부품 | 1x3 직각(ㄱ자) 핀헤더 2.54 mm (수) | HX PZ2.54-1x3P WZ / LCSC C32713262 | 5 | 10 | J8~J11 (날개 서보 4개의 3핀 플러그: 신호, +, -), J12 (ESC 리드선) - 보드당 납땜 15곳 | LCSC C32713262 (JLC가 실장하려던 바로 그 부품) 또는 같은 부품 번호를 파는 국내 쇼핑몰 |
| (a) JLC BOM에 있다가 이번에 사용자가 직접 사서 납땜하게 된 부품 | Molex SPOX 2.50 mm 2핀 수직 헤더 (5267-02A) | 22035025 / LCSC C5364642 | 1 | 2 | J14 (로직 배터리 플러그 - 팩의 Molex 5264형 2핀 하우징; 맞물림과 극성은 NOT_VERIFIED, 꽂기 전에 멀티미터로 확인) - 보드당 납땜 2곳 | LCSC C5364642 (JLC가 실장하려던 바로 그 부품) 또는 같은 부품 번호를 파는 국내 쇼핑몰 |
| (a) JLC BOM에 있다가 이번에 사용자가 직접 사서 납땜하게 된 부품 | JST PH 2.0 mm 2핀 수직 헤더 (B2B-PH-K-S) | B2B-PH-K-S(LF)(SN) / LCSC C131337 | 2 | 4 | J15 (전원 스위치 리드선, JST PHR-2 하우징), J17 (부저 리드선, JST PHR-2 하우징) - 보드당 납땜 4곳 | LCSC C131337 (JLC가 실장하려던 바로 그 부품) 또는 같은 부품 번호를 파는 국내 쇼핑몰 |
| (a) JLC BOM에 있다가 이번에 사용자가 직접 사서 납땜하게 된 부품 | 1x2 직각(ㄱ자) 핀헤더 2.54 mm (수) | HX PZ2.54-1x2P WZ / LCSC C32713261 | 1 | 2 | J16 (아밍 스위치 리드선, 2핀 2.54 mm 듀폰 하우징) - 보드당 납땜 2곳 | LCSC C32713261 (JLC가 실장하려던 바로 그 부품) 또는 같은 부품 번호를 파는 국내 쇼핑몰 |
| (a) JLC BOM에 있다가 이번에 사용자가 직접 사서 납땜하게 된 부품 | 1x4 직각(ㄱ자) 핀헤더 2.54 mm (수) | HX PZ2.54-1x4P WZ / LCSC C32713263 | 1 | 2 | J18 (전원 감지 리드선: VIN, VTET, VBUF, ISGN; 4핀 2.54 mm 듀폰 하우징. GND 핀은 일부러 없음) - 보드당 납땜 4곳 | LCSC C32713263 (JLC가 실장하려던 바로 그 부품) 또는 같은 부품 번호를 파는 국내 쇼핑몰 |
| (b) MCU | ATmega128A-AU (TQFP-64, 0.8 mm 피치) | ATMEGA128A-AU / LCSC C6510 | 4 | 8 | U1~U4 자리 (칩당 64핀 직접 납땜) | LCSC C6510 또는 국내 대리점 - with_mcu 파일 묶음으로 주문하면 사지 않음 |
| (c) 센서·기록기·GPS 모듈과 그 헤더·리드선 | GY-521 MPU-6050 모듈 (SZH-EK007) | - | 1 | 2 | J1 (모듈에 딸린 1x8 핀을 구멍 8개에 납땜; 받침 약 2.5 mm, A) | 디바이스마트 no=1247052 |
| (c) 센서·기록기·GPS 모듈과 그 헤더·리드선 | BMP280 모듈 TYE-BMP29 | - | 1 | 2 | J2 소켓 (모듈의 ㄱ자 6핀 헤더) | 디바이스마트 no=15442588 |
| (c) 센서·기록기·GPS 모듈과 그 헤더·리드선 | ㄱ자 1x6 핀 스트립 2.54 mm (BMP280이 일자 헤더로 올 때만) | SH-PZ254R-1x6P / LCSC C55629820 | 1 | 2 | BMP280 모듈 → J2 | 국내 쇼핑몰 (ㄱ자 2.54 mm 스트립을 6핀 잘라 씀) 또는 LCSC C55629820 |
| (c) 센서·기록기·GPS 모듈과 그 헤더·리드선 | NEO-7M GPS 모듈 SZH-NT07 | - | 1 | 2 | 기체 윗뚜껑; 리드선으로 J7에 연결 | 디바이스마트 no=10895400 |
| (c) 센서·기록기·GPS 모듈과 그 헤더·리드선 | GPS 리드선 JST-GH 1.25 mm 4핀 (양쪽 끝) | - | 1 | 2 | J7 ↔ NEO-7M | 완제품 리드선 (국내 쇼핑몰), 또는 GHR-04V-S 하우징 2개 (LCSC C160418) + SSHL-002T-P0.2 단자 8개 (LCSC C189897) + 압착기 |
| (c) 센서·기록기·GPS 모듈과 그 헤더·리드선 | SparkFun OpenLog DEV-13712 (기록기) | - | 1 | 2 | J5 소켓 | SparkFun 대리점 / 국내 쇼핑몰 |
| (c) 센서·기록기·GPS 모듈과 그 헤더·리드선 | microSD 카드 | - | 1 | 2 | OpenLog | 아무 곳 |
| (c) 센서·기록기·GPS 모듈과 그 헤더·리드선 | OpenLog용 ㄱ자 1x6 핀헤더 2.54 mm | SH-PZ254R-1x6P / LCSC C55629820 | 1 | 2 | OpenLog에 납땜(6곳)한 뒤 J5에 꽂음 | 국내 쇼핑몰 또는 LCSC C55629820 |
| (d) 배터리, 스위치, 부저와 그 리드선 | 로직 배터리 1S 리튬폴리머 DTP105565P 3.7 V 4200 mAh | - | 1 | 2 | J14 (Molex 5264형 플러그; 맞물림과 극성 NOT_VERIFIED - 먼저 멀티미터로 확인) | 디바이스마트 no=16021596 |
| (d) 배터리, 스위치, 부저와 그 리드선 | 전원 스위치 MTS-102 + JST-PH 2핀 리드선 | - | 1 | 2 | J15 | 스위치: 국내 쇼핑몰; 리드선: PH 2.0 mm 2핀 완제품, 또는 PHR-2 하우징 (LCSC C157955) + SPH-002T-P0.5S 단자 2개 (LCSC C111515) |
| (d) 배터리, 스위치, 부저와 그 리드선 | 아밍 스위치 + 2핀 2.54 mm 듀폰 리드선 | - | 1 | 2 | J16 | 국내 쇼핑몰 |
| (d) 배터리, 스위치, 부저와 그 리드선 | 부저 (5 V 능동형) + JST-PH 2핀 리드선 | - | 1 | 2 | J17 | 국내 쇼핑몰 (하우징 PHR-2 = LCSC C157955, 단자 SPH-002T-P0.5S = LCSC C111515) |
| (e) 프로그래밍 헤더와 도구 | 2x3 핀헤더 2.54 mm (프로그래밍용, A안) | LCSC C65114 | 5 | 10 | J3, J19, J20, J21, J22 (케이블타이 띠 안의 빈 구멍) | 국내 쇼핑몰 또는 LCSC C65114 - 대신 2x3 포고핀 ISP 어댑터를 써도 됨 (접촉 NOT_VERIFIED) |
| (e) 프로그래밍 헤더와 도구 | 1x2 핀헤더 2.54 mm + 점퍼 (리셋 유지) | LCSC C55629809 | 3 | 6 | J24, J25, J26 | 국내 쇼핑몰 또는 LCSC C55629809 (헤더) + C100114 (점퍼) |
| (e) 프로그래밍 헤더와 도구 | USBasp 프로그래머 (타깃 전원 점퍼는 빼 둠) | - | 1 | 1 | J19~J22 (MCU 4개 굽기) | 국내 쇼핑몰 |
| (e) 프로그래밍 헤더와 도구 | USB-시리얼 어댑터 (3.3 V 신호) | - | 1 | 1 | J3 (ESP-12F 굽기, 3.3 V 전용) | 국내 쇼핑몰 |
| (e) 프로그래밍 헤더와 도구 | 납땜 도구: 가는 인두 팁, 플럭스, 실납, 니퍼(다리 절단용), 멀티미터 (없을 때만) | - | - | 1 set | MCU, ESP-12F, 커넥터 12개 납땜, 다리 자르기, 전원 넣기 전 확인 | 국내 쇼핑몰 |
| (f) 장착 재료 | 3 mm 폼테이프 | - | 1 | 2 | 기판 아랫면 (스루홀 다리를 자른 뒤) | 국내 쇼핑몰 |
| (f) 장착 재료 | 케이블타이 | - | 1 | 2 | 케이블타이 띠 (x -3..+3 mm)를 가로질러 | 국내 쇼핑몰 |
| (g) 로켓에 쓸 때만 (rev 3.1): 분리 서보 출력 | 서보 Y자 리드선 3핀 2.54 mm (JR / 후타바형) | - | 1 (rocket board) | 1 | ESC와 J12 사이; 두 번째 갈래가 분리 서보의 + / - 를 주고, 그 갈래의 신호선은 잘라서 J13 패드(RELEASE_SIG)에 납땜 | 국내 쇼핑몰 |

## 7. 보드를 받은 뒤 할 일 (계획 15장 순서 + ESP-12F와 스루홀 커넥터 납땜)

순서의 뼈대: 검사 → 전원 커넥터 J14·J15 납땜 → **MCU·ESP 없이 전원 시험 (+5V, +3V3)** → 부하 시험 → MCU → ESP-12F → 나머지 스루홀 커넥터 → 다리 자르기 → 모듈 → 프로그래밍. 받은 보드에는 SMT 부품만 붙어 있고 커넥터가 하나도 없습니다 (GPS용 J7만 SMD라 붙어 옴). ESP-12F와 MCU를 달기 전에 전원을 넣어 두 전압을 잴 수 있습니다: 승압 회로와 3.3 V 레귤레이터는 JLC가 실장하고, 켜는 데 MCU나 ESP가 필요하지 않습니다.

1. **검사:** 탄탈 C45 / C52와 전해 C47의 극성을 실크 표시와 비교, U8 / U9 / J7 방향, U8(0.5 mm 피치) 납땜 브리지가 없는지 확대해서 봅니다. U7 (ESP-12F), U1~U4, 스루홀 커넥터 자리(J2, J5, J8~J12, J14~J18)는 비어 있어야 하고, U7과 U1~U4 패드에는 납(페이스트 자국)이 없이 무연 HASL 코팅만 있어야 합니다.
2. **레일:** Economic 주문에는 떼어낼 레일이 없습니다. (Standard 대안으로 주문했다면 여기서 보드 둘레의 레일을 꺾어 떼고 모서리를 다듬습니다.)
3. **전원 커넥터 J14 (배터리)와 J15 (스위치) 납땜** (납땜 2 + 2곳): 전원 시험에 필요해서 먼저 답니다. 두 커넥터는 칩 자리에서 멀어 먼저 달아도 MCU·ESP 납땜을 가리지 않습니다 (J14: MCU 패드에서 25.41 mm 이상, ESP 패드에서 8.1 mm; J15: MCU 패드에서 20.69 mm 이상, ESP 패드에서 9.2 mm, M). 방향은 7번의 표를 따릅니다.
   - **J14를 180도 돌려 달면 배터리의 +와 -가 뒤바뀝니다.** J14의 1번 핀(VBAT_LOGIC_RAW, 핀 줄의 왼쪽 끝 (네모난 패드))이 배터리 + 입니다 (M: 보드 파일의 네트). 윗면에는 J14 옆에 +/- 글자가 없습니다. 아랫면 실크에는 '1S BAT -+' 이 있고, 아랫면에서 보면 (좌우가 뒤집혀) 네모난 패드가 오른쪽에 있습니다. 라벨의 두 기호는 아랫면에서 본 패드 순서대로 적혀 있습니다: 끝 글자 '+' = 1번 핀 (네모난 패드) (M: 보드 파일에서 글자 획의 위치와 패드 좌표를 비교한 검사 PASS; 그림 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/a3_as_delivered_bottom.png` 에서 눈으로 확인). 2026-11-12 03:57까지의 파일에는 이 라벨이 '1S BAT +-' 로 적혀 '+' 가 GND 패드 쪽에 있었습니다 (검증에서 지적, 고침). **네모난 패드 = + 입니다.** 납땜하기 **전에** 커넥터를 구멍에 끼운 채 배터리 플러그를 꽂아 보고, 멀티미터로 배터리 +가 네모난 패드 쪽 핀에 오는지 확인하십시오 (팩의 극성과 맞물림은 NOT_VERIFIED; 역극성 보호 Q10이 있지만 시험된 적이 없습니다).
   - **뒷면 다리 규칙 (모든 스루홀 커넥터에 적용):** 납땜한 뒤 뒷면으로 나온 다리를 재서 1.5 mm 보다 긴 것은 모두 자릅니다 (폼테이프가 뒷면에 붙기 때문). J15, J17 은 도면값이 규칙을 넘어 반드시 자릅니다 (다리 4개). J2, J5, J8~J12, J16, J18 는 도면값은 규칙 안이지만 공차 끝에서 넘으므로 재 보고 자릅니다. J1, J14 은 도면값을 확인하지 못했으니 재 봅니다. 이 도면값은 6장 (a) 묶음의 참고 부품 번호를 샀을 때의 값입니다 (F); 다른 부품을 샀다면 모두 재 보십시오.
4. **MCU·ESP-12F·모듈 없이 벤치 전원 시험:** J14에 전류 제한(100 mA) 전원, J15는 단락 (스위치 리드를 꽂아 켜거나, 두 핀을 이은 PH 2핀 플러그를 꽂음). 아래 값을 적습니다. 범위를 벗어나면 멈춥니다. ESP-12F를 달기 전에 +3V3을 재 두면, 3.3 V가 잘못 나와도 모듈(공급 3.0~3.6 V, F)이 상하지 않습니다.

| 측정 | 허용 범위 (E, 계획) | 측정값 |
|---|---|---|
| 전압을 천천히 올릴 때 5 V가 켜지는 전압 | 3.44 ~ 3.73 V (3.3 V에서는 꺼져 있어야 함) | |
| 켜진 뒤 +5V | 4.84 ~ 5.33 V | |
| 켜진 뒤 +3V3 (ESP-12F를 달기 전) | 3.0 V 이상 3.6 V 이하 (ESP-12F 공급 범위, F) | |
| 전압을 천천히 내릴 때 꺼지는 전압 | 3.00 ~ 3.25 V | |

   켜지는 순간의 +5V와 EN 파형을 오실로스코프로 한 장 남깁니다. 같은 때 VBAT_LOGIC_SW(U8 입력, C41)의 파형도 남깁니다: 스위치 리드로 들어오는 전원은 세라믹 커패시터만 있어 울릴 수 있습니다 (VIN 절대 최대 7 V, F; 검토 E-R6, NOT_VERIFIED).
5. **부하 시험:** +5V에 0.27 A와 0.78 A, 전원 3.4 V와 3.7 V (전류 제한 2 A). 전압이 유지되고 U8 / L5가 약 60 C 아래인지 봅니다 (E, 계획 15장). 실제 배터리·스위치·리드로 배터리에서 U8 입력까지 저항을 잽니다 (전압 강하 / 입력 전류): 0.29 Ohm 이하여야 합니다. 넘으면 비행 전에 보고합니다. (0장의 배선 폭 문제 때문에 이 측정이 특히 중요합니다.) 0.78 A 부하에서 U8 SW 핀(L5 쪽 패드)과 +5V(C57)의 순간 과전압, U8 입력(VIN)의 리플을 짧은 접지선으로 오실로스코프에 담습니다 (0장 B: 승압부를 재배치했고 SW 선이 길어짐). SW가 7 V에 가까우면 비행 전에 보고합니다. MCU를 납땜한 뒤에는 같은 0.78 A 부하를 건 채로 NAV 칩 U1의 시리얼 통신이 깨지지 않는지도 확인합니다 (0장 B: 승압부가 U1의 16 MHz 크리스털에 rev 2c보다 가까움, 영향은 NOT_VERIFIED).
6. **MCU 4개 납땜** (JLC가 안 붙였을 때): 1번 핀을 표시에 맞추고, 가는 인두·플럭스로 0.8 mm 피치 핀을 모두 확대 검사합니다. 패드 1 mm 안의 부품 자리 수 (JLC가 붙인 것과 나중에 달 커넥터 포함): U1 13, U2 14, U3 20, U4 15 (M). U1을 GY-521보다 먼저 납땜하십시오 (GY-521이 U1 위에 걸침). 그림: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/c5_mcu_pin1.png`. 패드는 무연 HASL입니다. 무연 HASL 표면은 ENIG보다 덜 평평한 것이 일반적이므로 (이 보드에서 잰 것은 아님, NOT_VERIFIED) 칩이 한쪽으로 뜨지 않았는지 네 변을 확대해 보십시오.
   **ESP-12F (U7) 납땜** (두 파일 묶음 모두에서 사용자 몫; 2026-11-12 사용자 결정): 모듈은 옆면에 반원 홈이 있는 패드 22개로 붙습니다 (패드 크기 2.5 x 1 mm 와 1 x 1.8 mm, M). 그중 이 보드의 회로에 이어진 패드는 9개 (1, 3, 8, 15, 16, 17, 18, 21, 22번) 이고, 나머지 13개는 어떤 네트에도 이어지지 않아 모듈을 붙잡는 역할만 합니다 (M). 거버에 이 패드의 솔더 페이스트 구멍이 없으므로 패드는 무연 HASL 코팅만 된 상태입니다 (1장).
   - 언제 (보드에서 잰 거리로 정한 순서, M):
     - 4번 전원 시험 **뒤에**: +3V3이 범위 안인 것을 본 다음에 모듈을 답니다.
     - MCU **뒤에** (권장): ESP-12F 몸체는 U3 패드에서 5.8 mm, U2 패드에서 9.6 mm 떨어져 있습니다. 모듈 높이 3.2 mm (A)가 MCU 1.2 mm (A)보다 높아서, 모듈을 먼저 달면 U3 의 그쪽 핀 줄을 납땜하고 확대 검사할 때 인두와 시야를 가립니다. MCU를 먼저 달면 ESP 납땜을 가리는 것이 없습니다. 꼭 지켜야 하는 순서는 아니고, 이 거리에서 권하는 순서입니다.
     - 서보 커넥터 J8 (4.73 mm), J9 (4.73 mm) 와 J3 굽기 헤더(직접 납땜한다면)보다 **먼저**: 이 커넥터들은 ESP 패드에서 5 mm 안에 서고, 빈 헤더 구멍 J20 1.78 mm, J3 1.78 mm 는 ESP 패드 바로 옆입니다. 먼저 세우면 그쪽 ESP 패드에 인두가 들어가기 어렵습니다.
   - 어떻게: 안테나 쪽(패드가 없는 끝)이 기판 변 쪽을 향합니다 (모듈 몸체는 그 변에서 0.15 mm). 1번 핀을 실크 표시에 맞추고, 모서리 패드 하나만 먼저 붙여 실크 윤곽과 위치를 맞춘 뒤 나머지를 납땜합니다. ESP 패드에서 1 mm 안에 JLC가 이미 붙인 부품이 있습니다: Q5 0.4 mm, C52 0.45 mm, R58 0.54 mm, C55 0.55 mm, C53 0.67 mm, C54 0.85 mm (M). 가장 가까운 이웃 패드까지 0.525 mm 입니다. 납이 넘어가 붙지 않게 하고, 탄탈 C52를 오래 달구지 마십시오.
   - 끝난 뒤: 전원을 넣기 전에 멀티미터로 8번 패드(+3V3)와 15번 패드(GND) 사이가 단락이 아닌지 재고, 4번의 +3V3 측정을 한 번 더 합니다. 그림: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/c4_esp12f_keepout.png`
7. **나머지 스루홀 커넥터 10개 납땜** (납땜 35곳; J14·J15까지 합치면 커넥터 12개, 39곳). MCU와 ESP-12F **뒤에** 답니다. 이유 (M): 다음 커넥터는 MCU 패드에서 5 mm 안에 섭니다 - J5 (U2 패드에서 0.73 mm; 높이 5.9 mm), J9 (U3 패드에서 4.33 mm; 높이 3 mm), J10 (U3 패드에서 4.28 mm; 높이 3 mm), J11 (U3 패드에서 4.28 mm; 높이 3 mm), J18 (U4 패드에서 4.28 mm, U3 패드에서 4.34 mm; 높이 3 mm). 특히 J5 소켓은 U2 핀 줄 바로 옆이라 먼저 달면 U2의 그 변을 납땜하고 검사하기 어렵습니다. ESP 패드에서 5 mm 안: J8 (4.73 mm), J9 (4.73 mm). (with_mcu 묶음으로 MCU를 JLC가 붙였다면 MCU 쪽 제약은 없고, ESP-12F만 먼저 달면 됩니다.)
   - 순서: 낮은 것부터 답니다 - J8~J12, J16, J18 (높이 3 mm) → J2, J5 (높이 5.9 mm) → J17 (높이 11 mm). 낮은 것을 먼저 달아야 기판을 뒤집어 납땜할 때 부품이 빠지지 않게 받치기 쉽습니다. 기판을 뒤집어 뒷면에서 납땜하므로, 핀 하나만 먼저 붙이고 윗면에서 방향과 밀착(몸체가 기판에 붙었는지)을 확인한 뒤 나머지를 납땜합니다.
   - 방향: 커넥터 몸체를 실크 윤곽에 맞춥니다. 아래 표의 '위쪽 / 아래쪽 / 왼쪽 / 오른쪽'은 윗면 그림 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/a1_as_delivered_top.png` (부품 면을 위에서 본 그림; ESP-12F 자리가 위쪽 변, J8~J11이 왼쪽 변)에서 본 방향입니다 (M: 보드 파일의 패드와 몸체 윤곽에서 잼). 직각형은 핀(또는 소켓 입구)이 표의 방향을 보게 눕히고, 수직형(J14, J15, J17)은 핀 줄에서 윤곽까지의 거리가 양쪽이 달라 한 방향으로만 윤곽에 맞습니다. 1번 핀은 네모난 패드이고 나머지는 둥근 패드입니다. 풋프린트 대조 기록: J14 / J15 / J17은 `out/fit_checks.json` connector_drawings (J14 구멍 지름은 NOT_VERIFIED), 소켓과 핀헤더는 계획서의 도면 대조 (F). 산 부품을 실제로 꽂아 본 적은 없습니다 (NOT_VERIFIED).

| 자리 | 부품 (참고 번호) | 납땜 | 1번 핀 (네트, 위치) | 몸체 방향 | 가까운 기판 변 | 높이 (A) |
|---|---|---|---|---|---|---|
| J2 | 1x6 직각(ㄱ자) 핀 소켓 2.54 mm (암) (HC-PM254-8.5H-1x6PW, LCSC C22436147) | 6곳 | +3V3, 핀 줄의 왼쪽 끝 (네모난 패드) | 직각형: 몸체가 핀 줄에서 아래쪽으로 10.08 mm 뻗음 (반대쪽 0.05 mm) → 아래쪽을 보게 눕힘 | 위쪽 변에서 1.98 mm | 5.9 mm |
| J5 | 1x6 직각(ㄱ자) 핀 소켓 2.54 mm (암) (HC-PM254-8.5H-1x6PW, LCSC C22436147) | 6곳 | 연결 없음, 핀 줄의 왼쪽 끝 (네모난 패드) | 직각형: 몸체가 핀 줄에서 아래쪽으로 10.08 mm 뻗음 (반대쪽 0.05 mm) → 아래쪽을 보게 눕힘 | 오른쪽 변에서 0.76 mm | 5.9 mm |
| J8 | 1x3 직각(ㄱ자) 핀헤더 2.54 mm (수) (HX PZ2.54-1x3P WZ, LCSC C32713262) | 3곳 | VANE1_SIG, 핀 줄의 아래쪽 끝 (네모난 패드) | 직각형: 몸체가 핀 줄에서 왼쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 왼쪽을 보게 눕힘 | 왼쪽 변에서 0.73 mm | 3 mm |
| J9 | 1x3 직각(ㄱ자) 핀헤더 2.54 mm (수) (HX PZ2.54-1x3P WZ, LCSC C32713262) | 3곳 | VANE2_SIG, 핀 줄의 아래쪽 끝 (네모난 패드) | 직각형: 몸체가 핀 줄에서 왼쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 왼쪽을 보게 눕힘 | 왼쪽 변에서 0.73 mm | 3 mm |
| J10 | 1x3 직각(ㄱ자) 핀헤더 2.54 mm (수) (HX PZ2.54-1x3P WZ, LCSC C32713262) | 3곳 | VANE3_SIG, 핀 줄의 아래쪽 끝 (네모난 패드) | 직각형: 몸체가 핀 줄에서 왼쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 왼쪽을 보게 눕힘 | 왼쪽 변에서 0.73 mm | 3 mm |
| J11 | 1x3 직각(ㄱ자) 핀헤더 2.54 mm (수) (HX PZ2.54-1x3P WZ, LCSC C32713262) | 3곳 | VANE4_SIG, 핀 줄의 아래쪽 끝 (네모난 패드) | 직각형: 몸체가 핀 줄에서 왼쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 왼쪽을 보게 눕힘 | 왼쪽 변에서 0.73 mm | 3 mm |
| J12 | 1x3 직각(ㄱ자) 핀헤더 2.54 mm (수) (HX PZ2.54-1x3P WZ, LCSC C32713262) | 3곳 | ESC_SIG, 핀 줄의 위쪽 끝 (네모난 패드) | 직각형: 몸체가 핀 줄에서 오른쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 오른쪽을 보게 눕힘 | 오른쪽 변에서 0.73 mm | 3 mm |
| J14 | Molex SPOX 2.50 mm 2핀 수직 헤더 (5267-02A) (22035025, LCSC C5364642) | 2곳 | VBAT_LOGIC_RAW, 핀 줄의 왼쪽 끝 (네모난 패드) | 수직형: 핀 줄에서 몸체 윤곽까지 위쪽 3.15 mm, 아래쪽 1.85 mm → 윤곽이 넓은 쪽이 위쪽 | 왼쪽 변에서 0.73 mm | 11 mm |
| J15 | JST PH 2.0 mm 2핀 수직 헤더 (B2B-PH-K-S) (B2B-PH-K-S(LF)(SN), LCSC C131337) | 2곳 | VBAT_LOGIC, 핀 줄의 왼쪽 끝 (네모난 패드) | 수직형: 핀 줄에서 몸체 윤곽까지 아래쪽 2.85 mm, 위쪽 2.16 mm → 윤곽이 넓은 쪽이 아래쪽 | 왼쪽 변에서 0.32 mm | 11 mm |
| J16 | 1x2 직각(ㄱ자) 핀헤더 2.54 mm (수) (HX PZ2.54-1x2P WZ, LCSC C32713261) | 2곳 | ARM_SW_EXT, 핀 줄의 왼쪽 끝 (네모난 패드) | 직각형: 몸체가 핀 줄에서 위쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 위쪽을 보게 눕힘 | 위쪽 변에서 0.73 mm | 3 mm |
| J17 | JST PH 2.0 mm 2핀 수직 헤더 (B2B-PH-K-S) (B2B-PH-K-S(LF)(SN), LCSC C131337) | 2곳 | +5V, 핀 줄의 왼쪽 끝 (네모난 패드) | 수직형: 핀 줄에서 몸체 윤곽까지 아래쪽 2.85 mm, 위쪽 2.16 mm → 윤곽이 넓은 쪽이 아래쪽 | 오른쪽 변에서 0.73 mm | 11 mm |
| J18 | 1x4 직각(ㄱ자) 핀헤더 2.54 mm (수) (HX PZ2.54-1x4P WZ, LCSC C32713263) | 4곳 | VS_VIN, 핀 줄의 아래쪽 끝 (네모난 패드) | 직각형: 몸체가 핀 줄에서 왼쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 왼쪽을 보게 눕힘 | 왼쪽 변에서 0.73 mm | 3 mm |

   - 다 달았으면 3번의 다리 규칙대로 뒷면 다리를 모두 재고 자릅니다. 폼테이프는 그 뒤에 붙입니다.
   - J3 굽기 헤더와 다른 프로그래밍 헤더를 직접 납땜한다면 이 단계 뒤에 답니다 (D6).
8. **모듈 장착:** 각 모듈에 인쇄된 핀 순서를 보드 실크와 먼저 비교합니다. 보드 실크 이름표 읽는 법 (M, 검사 PASS): 핀 줄과 나란히 적힌 이름표는 그 면에서 본 패드 순서대로 적혀 있습니다 (J8 윗면 'V1 S+-', J17 윗면 'BUZ +-', J14 아랫면 '1S BAT -+', J7 아랫면 'GPS G RX TX V', J16 아랫면 'ARM SW GA', J2 아랫면 'BMP280 3V 3V SDA SCL G 3V'; 아랫면 것은 아랫면에서 볼 때의 순서라서 윗면에서 보면 좌우가 반대입니다). 핀 줄과 직각으로 적혔거나 두 줄짜리 헤더 옆에 적힌 이름표(J1, J3, J5, J9~J13, J18)는 1번 핀(네모난 패드)부터의 순서입니다. GY-521은 자기 1x8 핀으로 J1에 납땜 (8곳), 받침 2.54 mm (A) (`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e2a_gy521_J1.png`); BMP280은 직접 단 소켓 J2에 꽂음 (헤더 직각이 아니면 직각 핀을 먼저 납땜, `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e2b_bmp280_J2.png`); OpenLog는 직각 6핀 헤더를 납땜한 뒤 microSD 쪽이 위로 오게 직접 단 소켓 J5에 꽂음 (`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e2c_openlog_J5.png`); NEO-7M은 윗뚜껑에 두고 JST-GH 케이블을 J7에 (`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e3_gps_lead_J7.png`). 받은 상태와 완성 상태 비교: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e1_delivered_vs_completed.png`, 전체 안내: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/e4_overview_callouts.png`. OpenLog는 먼저 아랫면(ATmega328 쪽) 높이를 재고 끼워 보십시오: 아랫면 부품이 1.2 mm라고 보는 보수적 경우(A) 꽂는 길에서 C3 -0.05 mm (J5_openlog 'insertion stroke beyond the plugged PCB end'), U6 -0.5 mm (J5_openlog 'insertion stroke beyond the plugged PCB end') 이 겹칩니다.
9. **프로그래밍 (전원은 한 가지만):** 로직 배터리(3.73 V 넘게 충전) 또는 J14에 4.0 V 벤치 전원으로 보드를 켜고, USBasp의 타깃 전원 점퍼는 **빼 둡니다** (5 V를 두 곳에서 넣지 않기). 단자는 D6 선택대로 (헤더 납땜 + 점퍼, 또는 포고핀 어댑터). ESP-12F는 J3로 3.3 V 신호로만 굽습니다.
10. 첫 Wi-Fi 송신 때 +5V와 +3V3의 최저값을 기록합니다.
11. 커넥터 연결: 배터리 J14, 스위치 J15, GPS J7, 서보 J8~J11 / ESC J12, 아밍 J16, 부저 J17. 장착은 rev 2c와 같이 3 mm 폼테이프 + 케이블타이 1개 (x -3..+3 mm 띠). **장착 전에 띠를 다시 비우십시오:** 프로그래밍 헤더를 띠 구멍에 납땜했다면 떼거나 잘라 내야 케이블타이와 폼테이프가 제자리에 앉습니다 (뒷면으로 나온 다리도 1.5 mm 규칙). 비행용 보드는 포고핀 어댑터로만 굽는 것이 가장 간단합니다.
   로켓에 넣는 보드만 (rev 3.1): ESC 리드와 J12 사이에 서보 Y자 리드를 끼우고, 분리 서보로 가는 갈래의 신호선을 잘라 J13 패드(KiCad (127.05, 66.85) mm, J12(ESC) 옆 +x / +y 모서리, 실크 'REL')에 납땜합니다. 서보의 + / - 는 그 갈래가 J12의 2번 / 3번에서 받습니다. 선은 당겨지지 않게 묶어 두십시오 (납땜부 강도 NOT_VERIFIED). VTOL에서는 J13을 비워 둡니다.

## 8. MCU·센서 말고 사용자가 해야 하는 일 (숨기지 않고 모음)

처음 요청은 'MCU와 센서만 빼고 모두 JLC가 실장'이었습니다. 2026-11-12 결정 뒤 아래 일은 이 보드에서 사용자 몫입니다:

- **ESP-12F (U7) 구매와 납땜** (2026-11-12 사용자 결정): 패드 22개 (7장 6번). 모듈은 따로 삽니다 (6장; 참고 부품 LCSC C82891).
- **스루홀 커넥터 12개 구매와 납땜** (2026-11-12 사용자 결정): J2, J5, J8~J12, J14~J18, 납땜 39곳 (7장 3번, 7번). 부품 6종은 따로 삽니다 (6장 (a) 묶음). J14는 방향이 틀리면 배터리 극성이 뒤집힙니다.
- 레일 떼기: Economic 주문에는 없습니다 (Standard 대안으로 주문했을 때만, 7장 2).
- 뒷면 다리 자르기: 직접 납땜한 모든 스루홀 다리가 대상입니다. J15, J17 (다리 4개)는 도면값으로 규칙 초과, J2, J5, J8~J12, J16, J18 는 공차 끝에서 초과, J1, J14 은 미확인 (7장 3번).
- 프로그래밍 단자 (A안): J3, J19~J22 (2x3) 와 J24~J26 (1x2)는 빈 구멍입니다. 헤더를 직접 납땜(36곳, 계획 D6)하고 점퍼 3개를 끼우거나, 포고핀 어댑터를 대고 굽습니다 (접촉 NOT_VERIFIED). JLC가 띠 밖에 헤더를 실장하는 B안은 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) (SMD 0곳, THT 0곳). 이 구멍들은 케이블타이 띠 안에 있으므로, 헤더를 납땜했다면 장착 전에 떼거나 잘라서 띠를 비워야 합니다 (7장 11번).
- OpenLog에 직각 6핀 헤더 납땜 (모듈 쪽 6곳). OpenLog는 센서가 아니라 기록기라 D5에서 '센서처럼 사용자가 장착'으로 정했습니다.
- BMP280 헤더가 일자형으로 오면 직각 핀으로 바꿔 납땜.
- GPS 케이블(JST-GH 4선), 스위치·부저·아밍 리드 준비 (6장 목록).
- 벤치 전원 시험과 기록 (7장 4~5) - 보드가 만들어진 적이 없어서 필요합니다.
- 폼테이프와 케이블타이로 장착.

## 9. 사용자 결정 D1~D17과 이번 파일에 쓴 값

계획의 기본값에 2026-11-12 사용자 결정(D9 Economic, D14 베이킹 없음, D16 무연 HASL, ESP-12F와 스루홀 커넥터 직접 실장)을 반영해 만들었습니다. '이번 파일에서 확인한 값'은 생성된 파일에서 읽은 것입니다.

| Id | 질문과 기본값 (계획 19장, 2026-11-12 변경 반영) | 이번 파일에서 확인한 값 |
|---|---|---|
| D1 | 5 V 변환 회로 방식 (기본: A안 TPS61023 승압 + TLV3012B 차단) | U8 C919459, U9 C20345924, L5 C53426685 (BOM) |
| D2 | 배터리 차단 전압 (기본: 켜짐 3.59 V / 꺼짐 3.13 V(공칭) 유지) | R65 470k 1%, R66 270k 1%, R67 3.3M 1% → 켜짐 3.44 ~ 3.73 V, 꺼짐 3.00 ~ 3.25 V (E) |
| D3 | 요청과 무관하게 바꾼 부품 번호 3건을 유지할지 (기본: 유지. 2026-11-12부터 그중 2건(J8~J12, J2/J5)은 사용자가 직접 사서 납땜하는 부품이며, 번호는 구매 참고용) | C45 C11366 (JLC BOM); J8~J12 C32713262, J2 / J5 C22436147 는 사용자가 사는 부품의 참고 번호 (JLC BOM: BOM에 없음) |
| D4 | MCU를 누가 붙일지 (기본: 직접 납땜(요청대로). 권장은 JLC 실장) | 기본 BOM에 U1~U4 없음; with_mcu 쌍 U1~U4 = C6510 (두 쌍 모두 생성) |
| D5 | OpenLog 장착 방법 (기본: 헤더를 납땜해 J5에 꽂기. 안 맞으면 6선 케이블) | 소켓 J5는 사용자가 사서 납땜 (참고 C22436147; JLC BOM: BOM에 없음), OpenLog는 따로 구매 |
| D6 | 프로그래밍 단자 (기본: A안 빈 구멍(직접 납땜 36핀 또는 포고핀). B안은 가능 여부 보고 후 결정) | A안 (빈 구멍); B안 검색 SMD 0곳, THT 0곳 → 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) |
| D7 | J15/J17 커넥터 (기본: 스루홀. 2026-11-12부터 사용자가 직접 사서 납땜하고 다리도 직접 절단) | J15 / J17 참고 부품 C131337 (THT, 사용자가 납땜; JLC BOM: BOM에 없음); 다리 자르기 J15, J17 |
| D8 | 레벨시프터 FET (기본: C40912 유지) | Q1 등 C40912 (BOM) |
| D9 | 주문 방식 (기본: Economic - 2026-11-12 사용자 결정. 2026-11-10 계획의 기본·권장은 Standard) | 주문 화면에서 사용자가 선택 (2장: PCBA Type = Economic; Standard는 대안) |
| D10 | 툴링 홀 (기본: 설계에 넣지 않음. Economic 주문에는 레일이 없어 JLC가 기판 안에 뚫음 - 주문 화면 'Tooling holes: Added by JLCPCB', 위치는 NOT_VERIFIED) | 툴링 홀 부품 0개 (설계에 없음); 주문 화면 Tooling holes = Added by JLCPCB |
| D11 | 수량 (기본: PCB 5장, 실장 2장) | 주문 화면에서 사용자가 선택 (2장: PCB 5 / 실장 2) |
| D12 | 모듈 소켓 J2/J5 (기본: C22436147. 2026-11-12부터 사용자가 직접 사서 납땜. 구하기 어려우면 C55629861, 그다음 C54876735 - 풋프린트 호환은 NOT_VERIFIED) | J2 / J5 참고 부품 C22436147 (사용자가 구매); 대체 순서는 5장 표 2 |
| D13 | 배터리 전압 분압 저항 (기본: 그대로. 차단 후에도 약 40 uA가 흐르므로 스위치를 꺼야 함) | R15 47k 1%, R16 47k 1% (그대로) |
| D14 | 흡습 부품 베이킹 요청 (기본: 요청 안 함 - 2026-11-12 변경. 주문 화면에서 Economic PCBA는 베이킹을 고를 수 없음. 계획은 요청함) | 주문하지 않음 (2장: Bake Components = 고를 수 없음 (Economic에는 없는 항목); 메모에 베이킹 문장 없음) |
| D15 | 모듈 치수 (기본: 가정값 사용, 그림에 '가정' 표시. 실측을 권장) | 가정 치수 사용 (spec/module_dimensions.json, 그림과 검사에 '가정' 표시) |
| D16 | 표면처리·색상 등 (기본: 무연 HASL - 2026-11-12 사용자 결정. 계획의 권장은 ENIG. 녹색, JLC 기본값. 비아 덮기는 주문 화면이 Plugged로 정함) | 주문 화면에서 사용자가 선택 (2장: LeadFree HASL, 녹색, 기본 적층, Via Covering Plugged); 보드 파일 표기 'HAL lead-free', job 파일 'HAL lead-free' |
| D17 | 실장하지 않은 여분 보드 (기본: 기구 확인용으로만 사용) | 여분 기판 3장은 기구 확인용 (주문 화면 수량으로 정해짐) |
| 추가 (2026-11-12 사용자 결정) | ESP-12F (U7)를 JLC가 아니라 사용자가 직접 실장 | 두 BOM의 U7: BOM에 없음; 두 zip 모두 U7 패드 22개에 페이스트 없음 |
| 추가 (2026-11-12 사용자 결정) | 스루홀 커넥터 12개를 JLC가 아니라 사용자가 사서 납땜 (JLC는 SMT만) | 사양 fit user 14개 (J1, U7, 커넥터 12개); 두 BOM의 스루홀 줄 0 / 0 |
| 추가 (2026-11-12 사용자 결정) | 실패한 배선 규칙을 없애려고 기판 크기를 조정 (-y 변 탭) | 탭 KiCad x 117.3..132, y 135..142.2 mm (바깥 모서리 r 1 mm, 본체와 만나는 안쪽 모서리 r 1 mm, 기판 재료는 x 116.3..133), 외접 76 x 77.2 mm |
| 추가 (2026-11-12 사용자 결정) | 프로그래밍 구멍 J26 / J22 / J19 이동 ("옮겨도 됨") | J19 (x +0, y +0.55 mm), J22 (x +0, y +0.55 mm), J26 (x +0, y +0.55 mm) |

## 10. 변경 범위 표 (rev 2c 대비, 부품마다 이유 분류)

계획 3장의 표를 계획 파일에서 읽고, rev 3 부품 번호를 주문 BOM 파일과 대조했습니다 (교체 11, 추가 15, 삭제 1, 그대로 (부품 없음) 10, 그대로 140). 그대로인 부품은 표에서 뺐습니다. '누가 붙임'이 user인 줄의 rev 3 번호 칸은 '-' 입니다 (JLC BOM에 없음; 참고 부품 번호는 6장).

| Ref | 변경 | rev 2c | rev 3 | 누가 붙임 | 분류 (태그) | BOM 파일과 대조 |
|---|---|---|---|---|---|---|
| C45 | 교체 | 22u 10V tantalum / C7183 | 22u 10V tantalum (TAJA226K010RNJ, +-10 %) / C11366 | JLC | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | 일치 |
| J2 | 교체 | 1x6 2.54 mm right-angle socket (module lies flat) / C55629861 | 1x6 2.54 mm right-angle socket (module lies flat) / - | user | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | BOM에 없음 (맞음) |
| J3 | 그대로 (부품 없음) | 2x3 2.54 mm vertical: ESP-12E programming header (3.3 V logic ONLY) / C65114 | 2x3 2.54 mm vertical: ESP-12F programming header (3.3 V logic ONLY) / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J5 | 교체 | 1x6 2.54 mm right-angle socket: OpenLog lies FLAT beside it (outline reserved) / C55629861 | 1x6 2.54 mm right-angle socket: OpenLog lies FLAT beside it (outline reserved) / - | user | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | BOM에 없음 (맞음) |
| J6 | 삭제 | 1x5 2.54 mm: Pololu S9V11F5S6CMA 5 V step-up/step-down module (VOUT, GND, VIN, EN, PG) / no=15855364 | - | - | 필수: 요청(모든 부품을 JLC가 실장) 때문에 | BOM에 없음 (맞음) |
| J8 | 교체 | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / - | user | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | BOM에 없음 (맞음) |
| J9 | 교체 | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / - | user | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | BOM에 없음 (맞음) |
| J10 | 교체 | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / - | user | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | BOM에 없음 (맞음) |
| J11 | 교체 | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / - | user | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | BOM에 없음 (맞음) |
| J12 | 교체 | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / - | user | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | BOM에 없음 (맞음) |
| J13 | 추가 | - | 1-pin THT pad (release servo signal) / - | none | user decision 2026-11-13 (rev 3.1, option C as the user's fallback) | BOM에 없음 (맞음) |
| J16 | 교체 | 1x2 right-angle header / C55629816 | 1x2 right-angle header / - | user | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | BOM에 없음 (맞음) |
| J18 | 교체 | 1x4 right-angle header / C55629818 | 1x4 right-angle header / - | user | 선택: 재고·도면 근거 (D3 / D12에서 rev 2c 번호로 되돌릴 수 있음) | BOM에 없음 (맞음) |
| J19 | 그대로 (부품 없음) | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J20 | 그대로 (부품 없음) | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J21 | 그대로 (부품 없음) | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J22 | 그대로 (부품 없음) | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J24 | 그대로 (부품 없음) | 2-pin header + shunt / C55629809 | 2-pin header + shunt / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J25 | 그대로 (부품 없음) | 2-pin header + shunt / C55629809 | 2-pin header + shunt / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| J26 | 그대로 (부품 없음) | 2-pin header + shunt / C55629809 | 2-pin header + shunt / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| Q3 | 그대로 (부품 없음) | BSS138 / C40912 | BSS138 / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| R22 | 추가 | - | 220 / C22962 | JLC | user decision 2026-11-13 (rev 3.1) | 일치 |
| R47 | 그대로 (부품 없음) | 10k / C25804 | 10k / - | none | 변경 없음: 부품 없는 자리 (rev 2c와 같음) | BOM에 없음 (맞음) |
| U7 | 교체 | ESP-12E (ESP8266EX + 4 MB SPI flash, PCB antenna) / C89297 | ESP-12F (ESP8266MOD, 4 MB flash, PCB antenna) / - | user | 조달: JLC 주문 방식·재고 때문에 (핀 호환 부품) | BOM에 없음 (맞음) |
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
| 출고 판정 | ORDERABLE (tool checks only; NOT_VERIFIED - never built) | out/manifest.json release_gate | M (도구 검사) |
| 출고 판정 이유 | 없음 | out/manifest.json release_gate.reasons | M |
| 결정성 | PASS - 바이트 비교 파일 71개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 0개 | out/reports/determinism.json | M |
| 다른 폴더 결정성 | PASS - 다른 폴더에 복사해 다시 만든 결정적 파일 60개 중 55개가 이 폴더의 것과 바이트까지 같음 (보드, 회로도, 거버와 zip, BOM, CPL 등 주문에 쓰는 파일 43개는 모두 여기에 듦). 나머지 5개(README_ko.md, out/ORDER_ko.md, out/따로_살_부품_ko.md, spec/SPEC_ko.md, spec/parts_sources.json)는 자기 폴더 이름이나 그런 파일의 sha256을 담고 있어, 폴더 이름과 그 sha256을 바꿔 넣은 뒤에 같음. 그림 단계 산출물 11개는 그 폴더에서 다시 만들지 않아 비교하지 않음 | out/reports/determinism.json cross_folder | M |
| 재설계 요청 원문 | "그럼 다시 설계를 진행해줘. 실패한게 있다면 완료되었다고 말하지마" / "보드의 사이즈를 조정하는 방법은 없어?" / "옮겨도 됨" | 사용자 요청 2026-11-12 (재설계 작업 지시에 옮겨 적힌 원문) | F (사용자 요청 그대로) |
| 재설계 전 배선 규칙 | 16 / 21 PASS, FAIL: battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance | out/reports/routing_rules_before_redesign2.json results (재설계 전 보드) | M |
| 배선 규칙 PASS 수 | 21 | out/reports/routing_rules.json | M |
| 배선 규칙 수 | 21 | out/reports/routing_rules.json | M |
| 본체 사각형 | KiCad x 62..138, y 65..135 mm (76 x 70 mm, 모서리 r 1 mm) | out/reports/board_metrics.json board_fit.outline | M |
| 탭 위치 | KiCad x 117.3..132, y 135..142.2 mm (바깥 모서리 r 1 mm, 본체와 만나는 안쪽 모서리 r 1 mm, 기판 재료는 x 116.3..133) | out/reports/board_metrics.json board_fit.outline.tab_kicad_mm | M |
| 탭 넓이 | 105.8 mm2 | out/reports/board_metrics.json board_fit.outline.tab_area_mm2 | M |
| 외접 크기 | 76 x 77.2 mm | out/reports/board_metrics.json board_fit.outline.bbox_kicad_mm | M |
| 기체 맞춤 판정 | PASS | out/reports/board_metrics.json board_fit.ok | M |
| 기체 맞춤 | 흡입구 보어 벽까지 4.44 mm (기준 2 mm 이상), 탭에서 270도 통풍구까지 13.05 mm (기준 2 mm 이상), 탭에서 모터 리드 연결부 영역까지 21.869 mm (본체 사각형 쪽은 재설계 전과 같음), 315도 조인트 나사에서 평면 2 mm 안의 부품: 없음, 나사 아래 기판 윗면 높이 여유 1.3 mm (기준 1 mm 이상), 탭 안 부품 C41, C57, C58, L5, R63, R64, U8 (높이 한계 13.4 mm 넘는 것: 없음) | out/reports/board_metrics.json board_fit | M (나사 길이는 A / NOT_VERIFIED) |
| 옮긴 프로그래밍 구멍 | J19 (x +0, y +0.55 mm), J22 (x +0, y +0.55 mm), J26 (x +0, y +0.55 mm) | kicad/placement.json placement_checks.rev3.rev2c.fixed_parts_at_user_decided_spot | M |
| C42 이동 | x +0.20 mm (지금 x 72.8, y 69.3 mm) | kicad/placement.json placements.C42 (x, y, why) | M |
| 승압부 자리 | U8 (126.5, 135.45) 0도; L5 (125.15, 138.75) 0도; C43 (127.1, 133.3) 180도; C57 (129.2, 136.75) 270도; C41 (120, 138.75) 180도; R63 (123.85, 133.85) 270도; C58 (122.35, 133.85) 270도; R64 (120.85, 133.85) 90도 | kicad/placement.json placements (KiCad mm, 회전) | M |
| 배터리 경로 저항 20C | 0.03096 Ohm | out/reports/routing_rules.json battery_path | E |
| 배터리 경로 목표 | 0.035 Ohm | out/reports/routing_rules.json battery_path | F (계획 13절) |
| 배터리 경로 최소 폭 | 2 mm | out/reports/routing_rules.json battery_path.legs | M |
| 배터리 경로 구간 | J14.1 → Q10.3 0.00191 Ohm, 7.78 mm, 가장 좁은 곳 2 mm, 비아 0개; Q10.2 → J15.1 0.00388 Ohm, 15.77 mm, 가장 좁은 곳 2 mm, 비아 0개; J15.2 → L5.1 0.02517 Ohm, 132.22 mm, 가장 좁은 곳 2 mm, 비아 2개 | out/reports/routing_rules.json battery_path.legs | E (저항), M (길이·폭) |
| 배터리 경로 저항 60C | 0.03583 Ohm | out/reports/routing_rules.json battery_path | E |
| 재설계 전 배터리 경로 | 20 C 0.03173 Ohm, 60 C 0.03672 Ohm, 2.0 mm보다 좁은 길이 8.57 mm | out/reports/routing_rules_before_redesign2.json battery_path (재설계 전 보드) | E (저항), M (좁은 길이) |
| 출력 커패시터 간격 | C57 +5V 패드-U8.6 0.75 mm, GND 패드-U8.4 1.819 mm; C43 +5V 패드-U8.6 0.868 mm, GND 패드-U8.4 1.827 mm | out/reports/routing_rules.json hot_loop | M |
| 재설계 전 출력 커패시터 간격 | C57 0.95 / 1.611 mm; C43 3.25 / 3.537 mm | out/reports/routing_rules_before_redesign2.json hot_loop (재설계 전 보드) | M |
| 입력 커패시터 L5 경로 | C41 → L5 입력 패드 구리 길이 0.9 mm (기준 5 mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E; U8 VIN 패드와 L5 입력 패드 사이 직선 거리 2.103 mm) | out/reports/routing_rules.json input_cap_l5 | M (길이는 배선 모델, E) |
| SW 노드 | U8의 SW 패드와 L5의 SW 패드 사이 간격 1.876 mm (기준 2 mm 이하 - 계획 12장의 'L5는 U8 핀 바로 옆'에 숫자가 없어 스크립트가 계획 12장의 2 mm를 빌려 읽은 값, E), SW 선 구리 넓이 1.061 mm2, 구리 저항 약 4.3 mOhm (20 C, E), SW 선이 패드 사이를 지나는 다른 부품: 없음 | out/reports/routing_rules.json sw_node | M (기준과 저항은 E) |
| 크리스털 거리 부품별 | C43 Y1 9.55 / XTAL 핀 9.935 mm; C57 Y1 11.851 / XTAL 핀 12.753 mm; U8 Y1 11.925 / XTAL 핀 12.319 mm; L5 Y1 14.65 / XTAL 핀 15.025 mm; C41 Y1 15.762 / XTAL 핀 15.485 mm (rev 2c 기준 Y1 9.217 / XTAL 핀 9.616 mm) | out/reports/routing_rules.json crystal_distance.rev3_parts_min_gap_mm | M |
| C41 GND 윗면 길 | C41 GND 패드의 비아 (118.525, 137.2)에서 윗면 GND 선을 따라 13.923 mm (끝점 (127.1, 135.95), (118.525, 138.05); 가는 0.2 mm 선이 FB 분압 저항 두 줄 사이를 지나 C43 GND 패드를 거쳐 U8 GND 핀으로 감), C41 GND 패드-U8.4 직선 간격 8.033 mm | kicad/vtol_carrier.kicad_pcb GND F.Cu 선 조각 (scripts/docs_ko.py gnd_chain_mm), out/reports/routing_rules.json hot_loop_top_gnd | M |
| C41 GND 비아 거리 | 0.65 mm | out/reports/routing_rules.json hot_loop_top_gnd.gnd_vias_near_the_pads | M |
| SW 간격 여유 | 0.124 mm | out/reports/routing_rules.json sw_node (기준 - 간격) | M |
| 내층 귀환 거리 | C43.2 GND 비아 → U8.4 GND 비아 5.15 mm; C57.2 GND 비아 → U8.4 GND 비아 4.02 mm; C41.2 GND 비아 → U8.4 GND 비아 8.33 mm (비아 중심 사이 직선, In1 GND 면 위) | out/reports/routing_rules.json hot_loop_top_gnd.own_via_at (비아 위치 M) 사이 직선 거리 | E |
| C43 GND 선 | C43 GND 패드에서 U8 GND 핀까지 윗면 GND 선 폭 0.2 mm 0.358 mm, 폭 0.25 mm 3.1 mm (비아 연결선 제외; SOT-563 패드 두 줄 사이를 지나는 가는 선; 패드마다 0.3 mm GND 비아 하나씩) | kicad/vtol_carrier.kicad_pcb GND F.Cu 선 (C43.2 비아와 U8.4 비아가 이루는 상자 + 0.5 mm 안, scripts/docs_ko.py gnd_tracks_in_box) | M |
| L5 입력 비아 단면 | 드릴 0.4 mm 이상 비아 2개의 도금 구리 단면은 합쳐서 약 0.045 mm2 로, 폭 2.0 mm 1 oz 배선의 0.07 mm2 보다 작습니다 (도금 18 um 가정, E). | out/reports/routing_rules.json information.L5.1_battery_vias 에서 계산 (pi x 드릴 x 0.018 mm) | E |
| 다른 조인트 나사 옆 부품 | 45도 나사 - Q9 1.985 mm, 135도 나사 - J14 -1.393 mm, 225도 나사 - J7 -1.393 mm | out/reports/board_metrics.json board_fit.information_parts_within_plan_gap_of_the_other_screws_unchanged_corners (나사 끝 r 49.5 mm 가정 A, 음수 = 평면에서 겹침) | M (나사 길이 A) |
| C59 GND 비아 | 패드 1.5 mm 안에 자기 GND 비아 있음 | out/reports/routing_rules.json information.C59.2_gnd | M |
| L5 입력 비아 | 드릴 0.4 mm 2개 | out/reports/routing_rules.json information.L5.1_battery_vias (L5 입력 패드에서 4 mm 안의 VBAT_LOGIC_SW 비아) | M |
| rev 3.1 날짜 | 2026-11-13 | spec/rev3_plan.json amendments[rocket_release_output].date | F |
| rev 3.1 요청 원문 | "캐리어 PCB를 로켓에도 씀" / "B: 서보 열에 직각 헤더" | spec/rev3_plan.json amendments[rocket_release_output].request_verbatim | F (사용자 결정 그대로) |
| rev 3 장바구니 항목 | Y24 | 작업 지시 2026-11-13 (주 세션이 전한 장바구니 상태; 이 단계는 장바구니를 열지 않음) | F (지시) - 이 단계에서 확인 안 함 |
| 그 전 장바구니 항목 | Y23, Y20 | B:/0)AI_Cad/CanSat_v0.9/08_PCB/README.md (2026-11-12 재설계 납품 때 적은 장바구니 상태: 둘 다 옛 거버) | F (문서) - 장바구니를 열지 않아 어느 항목에 무엇이 들었는지 확인 안 함 |
| R22 BOM 줄 | 220R 1%, R17-R22 (6개), C22962 | out/jlcpcb_bom.csv (R22가 든 줄) | M |
| main bom_lines | 33 | out/manifest.json bom_cpl.pairs.main | M |
| J13 자리 | KiCad (127.05, 66.85) mm, J12(ESC) 옆 +x / +y 모서리 | kicad/placement.json placements.J13 (x, y, why) | M |
| 분리 서보 MCU 핀 | U2.7 (PE5 / OC3C): NC → RELEASE_PWM | out/reports/rev3_mcu_pinmap.json declared_changes_by_plan_amendment | M |
| B안 결과 (rev 3.1) | rev 3 배터리 선을 바꾸지 않는다는 조건에서는 놓을 수 없음: -x 서보 열에 직각 1x3 J13을 J8~J11과 같은 8.62 mm 간격으로 넣고 J18을 1핀으로 줄이면, 배터리 선 두 개(윗면 2.0 mm VBAT_LOGIC, 아랫면 2.4 mm VBAT_LOGIC_SW) 사이에 45.06 mm가 필요한데 44.60 mm뿐입니다. 801곳을 찾아본 가장 좋은 자리도 열의 양 끝에서 0.13 mm 간격 규칙보다 0.23 mm 모자랐습니다. 배터리 선을 바꾸지 않는다는 조건은 사용자 결정도 검사 기준도 아니고 (검사는 배터리 경로 폭 2.0 mm 이상, 0.035 Ohm 이하만 요구), rev 3.1 작업이 rev 3 재설계 때 build_pcb.py에 깔아 둔 미리 그린 배선을 건드리지 않으려고 스스로 둔 것입니다 | spec/carrier_spec.json rev_3_1_changes.option_b_result | M (배치 검색과 kicad-cli DRC, 2026-11-13) |
| 로켓 Y자 리드 | 서보 Y자 리드선 3핀 2.54 mm (JR / 후타바형) 1개 (로켓에 넣는 보드용) | scripts/purchase_list.py (g) 묶음 = spec/rev3_plan.json off_board_purchase | F (계획) |
| 변경 날짜 | 2026-11-12 | spec/rev3_plan.json amendments | F (사용자 결정) |
| 변경 요청 원문 1 | 아, esp도 내가 직접 실장할거야. 그럼 이코노믹으로 진행하고 pcb의 코팅도 무연으로 해줘 | spec/rev3_plan.json amendments[0].request_verbatim | F (사용자 요청 그대로) |
| 변경 요청 원문 2 | 이코노믹으로 스루홀 제외하고 어떤 부품을 따로 구매해야하는건지 리스트를 정리해줘 | spec/rev3_plan.json amendments[1].request_verbatim | F (사용자 요청 그대로) |
| 스루홀 부품 수 | 12 | out/fit_checks.json user_soldered_tht.summary.parts | M |
| 스루홀 목록 | J2, J5, J8~J12, J14~J18 | spec/carrier_spec.json parts[].fit == user (스루홀 커넥터) | M |
| zip main sha256 | 6a1e22caf6ce1ae4... | out/manifest.json fab.sets | M |
| jlcpcb_bom.csv sha256 | 2f4827ad0b5fe97c... | out/manifest.json sha256 | M |
| jlcpcb_cpl.csv sha256 | 9cf2527f4505e309... | out/manifest.json sha256 | M |
| zip with_mcu sha256 | b9246444b0348e2a... | out/manifest.json fab.sets | M |
| jlcpcb_bom_with_mcu.csv sha256 | a00f1407a2825cd1... | out/manifest.json sha256 | M |
| jlcpcb_cpl_with_mcu.csv sha256 | fa780830e806c36c... | out/manifest.json sha256 | M |
| F.Paste 패드 main | 322 | out/manifest.json fab.sets | M |
| F.Paste 패드 with_mcu | 578 | out/manifest.json fab.sets | M |
| ESP 패드 수 | 22 | out/fit_checks.json user_soldered_smd.U7 pads | M |
| ESP 페이스트 뺀 패드 main | 22 | out/reports/gerber_check.json paste_variant.main.pads_changed_per_ref.U7 | M |
| ESP 페이스트 뺀 패드 with_mcu | 22 | out/reports/gerber_check.json paste_variant.with_mcu.pads_changed_per_ref.U7 | M |
| 페이스트 없는 자리 main | J1, J2, J3, J5, J8, J9, J10, J11, J12, J13, J14, J15, J16, J17, J18, J19, J20, J21, J22, J24, J25, J26, JP1, NT1, Q3, R47, U1, U2, U3, U4, U7 | out/reports/gerber_check.json paste_variant.main | M |
| 페이스트 없는 자리 with_mcu | J1, J2, J3, J5, J8, J9, J10, J11, J12, J13, J14, J15, J16, J17, J18, J19, J20, J21, J22, J24, J25, J26, JP1, NT1, Q3, R47, U7 | out/reports/gerber_check.json paste_variant.with_mcu | M |
| main bom_lines_smt | 33 | out/manifest.json bom_cpl.pairs.main | M |
| main bom_lines_tht | 0 | out/manifest.json bom_cpl.pairs.main | M |
| main parts | 145 | out/manifest.json bom_cpl.pairs.main | M |
| main cpl_rows | 145 | out/manifest.json bom_cpl.pairs.main | M |
| with_mcu bom_lines | 34 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| with_mcu bom_lines_tht | 0 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| with_mcu parts | 149 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| with_mcu cpl_rows | 149 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| U1-U4 기본 BOM | 없음 | out/jlcpcb_bom.csv | M |
| 코드 U7 | BOM에 없음 | out/jlcpcb_bom.csv | M |
| 코드 J2 | BOM에 없음 | out/jlcpcb_bom.csv | M |
| 코드 J8 | BOM에 없음 | out/jlcpcb_bom.csv | M |
| 코드 J14 | BOM에 없음 | out/jlcpcb_bom.csv | M |
| 코드 J15 | BOM에 없음 | out/jlcpcb_bom.csv | M |
| 주문 화면 확인 시각 | 2026-11-12 01:30-02:06 KST | spec/jlc_order_page_20261112.json read_at | M (주문 화면 2026-11-12) |
| 주문 화면 매칭 | 부품 40줄을 인식하고 40줄을 확정 | spec/jlc_order_page_20261112.json files_uploaded_then | M (주문 화면 2026-11-12) |
| 식별 문구 위치 | 아랫면 실크(B.SilkS, 아랫면에서 볼 때 바로 읽히도록 좌우 반전) | out/reports/routing_rules.json silkscreen.identification_lines | M |
| 식별 문구 크기 | 글자 높이 1.2 mm, 선 굵기 0.2 mm | out/reports/routing_rules.json silkscreen.identification_lines | M |
| 식별 문구 검사 | PASS | out/reports/routing_rules.json silkscreen.identification_lines | M |
| 배선 재생 입력 보드 | byte-identical to the recorded unrouted board | kicad/work/route_reuse_log.json unrouted_board_guard | M |
| 보드 표면처리 | HAL lead-free | kicad/vtol_carrier.kicad_pcb stackup copper_finish | M |
| 주문 화면에 올렸던 파일 | 거버 zip 5e7d74ec..., BOM fa4c8e26..., CPL 738c4ea4... | spec/jlc_order_page_20261112.json files_uploaded_then | M (주문 화면 2026-11-12) |
| 주문 화면 올린 줄 수 | 40 | spec/jlc_order_page_20261112.json files_uploaded_then.parts_detected | M (주문 화면 2026-11-12) |
| 주문 화면 거버 인식 | 'Detected 4 layer board of 70x76mm' | spec/jlc_order_page_20261112.json files_uploaded_then | M (주문 화면 2026-11-12) |
| 층 수 | 4 | spec/carrier_spec.json | F (계획 불변조건 d) |
| 외곽 | 76 x 70 mm | spec/carrier_spec.json | F (size) / A (corner radius) |
| 두께 | 1.6 mm | spec/carrier_spec.json | F |
| gbrjob 표면처리 | HAL lead-free | out/gerber/vtol_carrier-job.gbrjob GeneralSpecs.Finish | M |
| 화면 Via 말풍선 | 'The Via Tented option is upgraded to Via Plugged for free' | spec/jlc_order_page_20261112.json pcb_fields[Via Covering].tooltip | M (주문 화면 2026-11-12) |
| 비아 크기별 | 0.6/0.3 mm 449개, 0.8/0.4 mm 3개 | kicad/vtol_carrier.kicad_pcb (지름/드릴) | M |
| 툴링 홀(H) 부품 | 0 | spec/carrier_spec.json summary.by_prefix | M |
| 위치 확인 창 안내 | 'confirm within 72 hours, otherwise automatically confirmed' | spec/jlc_order_page_20261112.json pcba_fields[Confirm Parts Placement].dialog_text | M (주문 화면 2026-11-12) |
| 베이킹 요금 | 7.94 USD | spec/jlc_order_page_20261112.json pcba_advanced_fields[Bake Components].fee_usd | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht nitrogen_reflow_soldering | 0.86 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.nitrogen_reflow_soldering | M (주문 화면 2026-11-12) |
| 메모 글자 한도 | 500 | spec/jlc_order_page_20261112.json pcba_advanced_fields[Assembly remark].limit_characters | M (주문 화면 2026-11-12) |
| 화면 Base Material | FR-4 | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Layers | 4 | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Dimensions | 거버에서 자동으로 읽힘 | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 PCB Qty | 5 | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Delivery Format | Single PCB | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 PCB Thickness | 1.6mm | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 PCB Color | Green | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Silkscreen | White | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Material Type | FR4 TG135 | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Surface Finish | LeadFree HASL | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Outer Copper Weight | 1 oz | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Inner Copper Weight | 0.5 oz | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Via Covering | Plugged | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Min via hole size/diameter | 0.3mm/(0.4/0.45mm) | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Board Outline Tolerance | ±0.2mm | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Confirm Production file | No | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Mark on PCB | Remove Mark | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Electrical Test | Flying Probe Fully Test | spec/jlc_order_page_20261112.json pcb_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 PCBA Type | Economic | spec/jlc_order_page_20261112.json pcba_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Assembly Side | Top Side | spec/jlc_order_page_20261112.json pcba_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 PCBA Qty | 2 | spec/jlc_order_page_20261112.json pcba_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Tooling holes | Added by JLCPCB | spec/jlc_order_page_20261112.json pcba_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Confirm Parts Placement | Yes + 'Do not confirm automatically' 체크 | spec/jlc_order_page_20261112.json pcba_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Parts Selection | By Customer (Self-Service) | spec/jlc_order_page_20261112.json pcba_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Bake Components | 고를 수 없음 (Economic에는 없는 항목) | spec/jlc_order_page_20261112.json pcba_advanced_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Depanel boards & edge rail before delivery | 화면에 보인 대로 둠 | spec/jlc_order_page_20261112.json pcba_advanced_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Solder Paste | High temp. | spec/jlc_order_page_20261112.json pcba_advanced_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Nitrogen reflow soldering | Yes (for Economic) | spec/jlc_order_page_20261112.json pcba_advanced_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Assembly remark | Yes | spec/jlc_order_page_20261112.json pcba_advanced_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Photo Confirmation | 고를 수 없음 (Economic에는 없는 항목) | spec/jlc_order_page_20261112.json pcba_advanced_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Board Cleaning | 화면에 보인 대로 둠 | spec/jlc_order_page_20261112.json pcba_advanced_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Flying Probe Test | 화면에 보인 대로 둠 | spec/jlc_order_page_20261112.json pcba_advanced_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 화면 Function test | 고를 수 없음 (Economic에는 없는 항목) | spec/jlc_order_page_20261112.json pcba_advanced_fields | M (주문 화면 2026-11-12): 항목 이름·선택지; 넣을 값은 이 주문의 선택 |
| 품목 설명 | 'DIY Hobby Circuit Board - HS Code 902300' | spec/jlc_order_page_20261112.json product_description | M (주문 화면 2026-11-12) |
| 화면 Standard 레일 문구 | 'The board size is modified to be 76mm*80mm due to adding two 5mm edge rails on the shorter sides' | spec/jlc_order_page_20261112.json pcba_fields[Tooling holes].standard_page_text | M (주문 화면 2026-11-12) |
| 부품 자리 점유율 | 86.9 % | kicad/placement.json placement_checks.courtyard_occupancy_percent (윗면 부품 자리 면적 / 케이블타이 띠를 뺀 기판 면적) | M |
| 가장자리 2.5 mm 안 몸체 | 33 | out/reports/edge_distance.json | M |
| JLC 실장 가장 가까운 몸체 | R60 0.537 mm | out/reports/edge_distance.json jlc_placed_bodies.default_order closest[0] | M |
| JLC 실장 가까운 몸체 | R60 0.537 mm, U6 0.55 mm, U9 0.589 mm, J7 0.805 mm, C60 0.95 mm | out/reports/edge_distance.json jlc_placed_bodies.default_order closest | M |
| JLC 실장 몸체 0.3 mm 안 | 없음 | out/reports/edge_distance.json jlc_placed_bodies.default_order bodies_under_0p3 | M |
| JLC 실장 패드 0.3 mm 안 | 없음 | out/reports/edge_distance.json jlc_placed_bodies.default_order pads_under_0p3 | M |
| JLC 미실장 몸체 0.3 mm 안 | U7 0.15 mm, J15 0.29 mm | out/reports/edge_distance.json bodies_under_0p3 (fit != jlc) | M |
| 가장자리 2.5 mm 안 JLC 실장 | 13 | out/reports/edge_distance.json bodies_under_2p5_by_fit | M |
| 가장자리 2.5 mm 안 with_mcu | 14 | out/reports/edge_distance.json bodies_under_2p5_by_fit | M |
| 리플로 Economic | 255±5 C | out/fit_checks.json reflow_rating.oven_C | F (계획 10장) |
| 리플로 Standard | 240±5 C | out/fit_checks.json reflow_rating.oven_C | F (계획 10장) |
| 리플로 한계 부품 | C2833309 (C47, 정격 260 C), C2286 (D1/D2/D3/D4, 정격 260 C), C53426685 (L5, 정격 260 C) | out/fit_checks.json reflow_rating.rows (Economic: AT THE LIMIT) | F (데이터시트) |
| C47 리플로 횟수 | 리플로는 한 번만 (두 번은 사이에 30분을 둘 때만) | out/fit_checks.json reflow_rating.rows C47 limits | F (데이터시트) |
| 리플로 미확인 줄 수 | 31 | out/fit_checks.json reflow_rating | NOT_VERIFIED |
| Economic 받은 줄 수 | 39 | spec/jlc_order_page_20261112.json economic_page_behaviour.lines_accepted_count | M (주문 화면 2026-11-12) |
| 추정 Economic 피더 종류 수 | 10 | out/jlc_fee_estimate.csv (main, Economic, 2장) feeder_types | M (BOM에서 셈) |
| 화면 Standard 레일 칸 | Edge Rails/Fiducials | spec/jlc_order_page_20261112.json pcba_fields[Tooling holes].standard_row | M (주문 화면 2026-11-12) |
| 레일 변 구리 이격 값 | 0.4 | out/reports/routing_rules.json rail_edge_copper | M |
| 라우팅·마우스바이트 구리 이격 | 0.2 mm 이상 | https://jlcpcb.com/capabilities/pcb-capabilities (rev 3 JLC 검토가 2026-11-11에 읽음; 이 단계에서 다시 읽지 않음) | F |
| V-cut 구리 이격 | 0.4 mm 이상 | https://jlcpcb.com/capabilities/pcb-capabilities (rev 3 JLC 검토가 2026-11-11에 읽음; 이 단계에서 다시 읽지 않음) | F |
| 견적 standard_no_tht PCBA | 111.64 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht 합계 | 124.84 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] total_without_shipping | M (주문 화면 2026-11-12) |
| Standard 추가 값 | 48.02 USD | spec/jlc_order_page_20261112.json quotes_usd (standard_no_tht 합계 - economic_no_tht 합계) | M (주문 화면 2026-11-12) 값의 차 |
| 메모 글자 수 main | 312 | scripts/docs_ko.py order_remarks (생성한 메모 문장의 글자 수) | M |
| 메모 글자 수 main + Standard 문장 | 447 | scripts/docs_ko.py order_remarks (Standard 대안 문장을 덧붙였을 때) | M |
| 메모 글자 수 with_mcu | 350 | scripts/docs_ko.py order_remarks (생성한 메모 문장의 글자 수) | M |
| 메모 글자 수 with_mcu + Standard 문장 | 485 | scripts/docs_ko.py order_remarks (Standard 대안 문장을 덧붙였을 때) | M |
| JLC 확인 시각 | 2026-11-13T20:26:50+09:00 | out/jlc_gate_report.json | M (그 시각의 재고) |
| JLC 확인 결과 | PASS (33/33줄), with_mcu PASS | out/jlc_gate_report.json | M |
| ESP 참고 부품 | C82891 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (U7) | F |
| 스루홀 참고 번호 | C22436147 (J2/J5), C32713262 (J8~J12), C5364642 (J14), C131337 (J15/J17), C32713261 (J16), C32713263 (J18) | spec/rev3_plan.json bom_lines_now_bought_by_the_user | F |
| 견적 economic_tht_placed hand_soldering_labor_fee | 3.61 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.hand_soldering_labor_fee | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed manual_assembly | 1.35 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.manual_assembly | M (주문 화면 2026-11-12) |
| 회전 미확인 | C45, C47, C52, U9 | out/reports/cpl_rotation.json | NOT_VERIFIED |
| 미리보기 화면 문구 | 'The preview for reference only. Check final part placement at DFM Analysis in Order History in 4-6 hours after placing the order.' | spec/jlc_order_page_20261112.json placement_preview.page_text | M (주문 화면 2026-11-12) |
| 미리보기 구분 못 한 부품 | U9 | spec/jlc_order_page_20261112.json placement_preview.not_resolved | M (주문 화면 2026-11-12) |
| 1번 핀 번호 반대 | 없음 | out/cpl_rotation_table.csv notes ('mirrored') | M |
| 견적 economic_no_tht 합계 | 76.82 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] total_without_shipping | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht 조건 | Economic, LeadFree HASL, 스루홀 0줄 실장, 부품 33줄 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] (pcba_type, surface_finish, tht_lines_placed, component_items) | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed 조건 | Economic, LeadFree HASL, 스루홀 6줄 실장, 부품 39줄 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] (pcba_type, surface_finish, tht_lines_placed, component_items) | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht 조건 | Standard, LeadFree HASL, 스루홀 0줄 실장, 부품 33줄 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] (pcba_type, surface_finish, tht_lines_placed, component_items) | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded 조건 | Standard, ENIG, 스루홀 6줄 실장, 부품 40줄 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] (pcba_type, surface_finish, tht_lines_placed, component_items) | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht setup | 8.24 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.setup | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed setup | 8.24 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.setup | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht setup | 25.75 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.setup | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded setup | 25.75 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.setup | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht stencil | 1.55 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.stencil | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed stencil | 1.55 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.stencil | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht stencil | 8.27 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.stencil | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded stencil | 8.27 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.stencil | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht components | 20.32 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.components | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed components | 21.69 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.components | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht components | 23.23 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.components | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded components | 30.37 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.components | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht extended_components_fee | 30.90 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.extended_components_fee | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed extended_components_fee | 30.90 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.extended_components_fee | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht extended_components_fee | - | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.extended_components_fee | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded extended_components_fee | - | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.extended_components_fee | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht feeders_loading_fee | - | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.feeders_loading_fee | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed feeders_loading_fee | - | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.feeders_loading_fee | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht feeders_loading_fee | 51.15 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.feeders_loading_fee | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded feeders_loading_fee | 52.70 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.feeders_loading_fee | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht smt_assembly | 1.40 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.smt_assembly | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed smt_assembly | 1.40 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.smt_assembly | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht smt_assembly | 2.29 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.smt_assembly | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded smt_assembly | 2.36 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.smt_assembly | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht confirm_parts_placement | 0.45 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.confirm_parts_placement | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed confirm_parts_placement | 0.45 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.confirm_parts_placement | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht confirm_parts_placement | 0.45 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.confirm_parts_placement | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded confirm_parts_placement | 0.45 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.confirm_parts_placement | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht hand_soldering_labor_fee | - | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.hand_soldering_labor_fee | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht hand_soldering_labor_fee | - | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.hand_soldering_labor_fee | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded hand_soldering_labor_fee | 3.61 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.hand_soldering_labor_fee | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht manual_assembly | - | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.manual_assembly | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht manual_assembly | - | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.manual_assembly | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded manual_assembly | 1.35 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.manual_assembly | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed nitrogen_reflow_soldering | 0.86 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.nitrogen_reflow_soldering | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht nitrogen_reflow_soldering | - | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.nitrogen_reflow_soldering | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded nitrogen_reflow_soldering | - | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.nitrogen_reflow_soldering | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht packaging | - | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba_parts.packaging | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed packaging | - | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba_parts.packaging | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht packaging | 0.50 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcba_parts.packaging | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded packaging | 0.50 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba_parts.packaging | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht PCBA | 63.72 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcba | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed PCBA | 70.05 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcba | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded PCBA | 125.37 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcba | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht PCB | 13.10 (special offer 8.00 + 표면처리 5.10) | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] pcb | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed PCB | 13.10 (special offer 8.00 + 표면처리 5.10) | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] pcb | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht PCB | 13.20 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] pcb | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded PCB | 25.30 (special offer 8.00 + 표면처리 17.30) | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] pcb | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed 합계 | 83.15 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] total_without_shipping | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded 합계 | 150.67 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] total_without_shipping | M (주문 화면 2026-11-12) |
| 견적 economic_no_tht 조립 기간 | 3-4일 | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht] assembly_time_days | M (주문 화면 2026-11-12) |
| 견적 economic_tht_placed 조립 기간 | 3-4일 | spec/jlc_order_page_20261112.json quotes_usd[economic_tht_placed] assembly_time_days | M (주문 화면 2026-11-12) |
| 견적 standard_no_tht 조립 기간 | 3-4일 | spec/jlc_order_page_20261112.json quotes_usd[standard_no_tht] assembly_time_days | M (주문 화면 2026-11-12) |
| 견적 standard_enig_as_uploaded 조립 기간 | 5-6일 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded] assembly_time_days | M (주문 화면 2026-11-12) |
| 견적 as_uploaded 항목 합 | 125.36 | spec/jlc_order_page_20261112.json quotes_usd[standard_enig_as_uploaded].pcba_parts 를 더한 값 | M (주문 화면 2026-11-12) 값의 합 |
| 스루홀 JLC 납땜 값 | 6.33 USD | spec/jlc_order_page_20261112.json quotes_usd (economic_tht_placed 합계 - economic_no_tht 합계) | M (주문 화면 2026-11-12) 값의 차 |
| LCSC 커넥터 값 | 4.56 USD | spec/lcsc_snapshot_20261112.json, scripts/purchase_list.py group_a_cost | E (단가는 M, 그 시각 기준) |
| LCSC 기록 시각 | 2026-11-12T02:50:37+09:00 ~ 02:51:09+09:00 | spec/lcsc_snapshot_20261112.json records[].fetched_at | M |
| 견적 표면처리 추가 | 5.10 USD | spec/jlc_order_page_20261112.json quotes_usd[economic_no_tht].pcb_parts.surface_finish | M (주문 화면 2026-11-12) |
| 추정 Economic 합 | 65.13 USD | out/jlc_fee_estimate.csv (main, Economic, 2장) sum_E | E |
| 추정 Economic 피더 | 30.70 USD | out/jlc_fee_estimate.csv (main, Economic, 2장) feeder_loading | E |
| 얇은 재고 기준 | 1000개 | out/jlc_gate_report.json | F (계획 I7) |
| 얇은 재고 줄 | C20345924 (U9) available 597 | out/jlc_gate_report.json | M |
| 재고 주의 줄 | C8598 (D6) 184940개, 필요 28, 재고가 계획 때 725265개에서 361060개로 절반 넘게 줄어듦, C53426685 (L5) 1058개, 필요 22, C919459 (U8) 4431개, 필요 21, 재고가 계획 때 31489개에서 5278개로 절반 넘게 줄어듦, C20345924 (U9) 597개, 필요 20 | out/jlc_gate_report.json lines (available < 2000, 또는 stock < 계획 spec/rev3_plan.json bom_lines.stock 의 절반) | M |
| 대체 L5 (C53426685) | C167203 FXL0420-1R0-M (129406개, PASS) | out/jlc_gate_report.json alternates_and_options | M |
| 대체 U8 (C919459) | C1852149 TPS61023DRLT (464개, PASS) | out/jlc_gate_report.json alternates_and_options | M |
| 대체 J7 (C161692) | C52037397 GH1.25-4PLT (1679개, PASS); C2829262 GH1.25-4PLTPZ (55682개, PASS) | out/jlc_gate_report.json alternates_and_options | M |
| 대체 J2, J5 (C22436147) | C55629861 SH-PM254R-1x6P-H85 (985개, PASS); C54876735 PM2.54-1x6PWZ-H8.5 (560개, PASS) | out/jlc_gate_report.json alternates_and_options | M |
| 따로 살 품목 수 | 28 | out/off_board_purchase.csv | M |
| 스루홀 종류 수 | 6 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (THT) | F |
| LCSC (a) 묶음 값 | 10.39 USD | spec/lcsc_snapshot_20261112.json 의 단가 x (최소 주문 수량을 지킨) 수량, scripts/purchase_list.py group_a_cost | E (단가는 M, 그 시각 기준) |
| LCSC 재고 부족 | 없음 | spec/lcsc_snapshot_20261112.json records[].stock | M (그 시각 기준) |
| 커넥터 J14 납땜점 | 2 | out/fit_checks.json user_soldered_tht.per_ref.J14.joints | M |
| 커넥터 J15 납땜점 | 2 | out/fit_checks.json user_soldered_tht.per_ref.J15.joints | M |
| J14 칩 거리 | MCU 패드에서 25.41 mm 이상, ESP 패드에서 8.1 mm | out/fit_checks.json user_soldered_tht.per_ref.J14.gap_to_mcu_pad_rings_mm / gap_to_user_smd_pads_mm | M |
| J15 칩 거리 | MCU 패드에서 20.69 mm 이상, ESP 패드에서 9.2 mm | out/fit_checks.json user_soldered_tht.per_ref.J15.gap_to_mcu_pad_rings_mm / gap_to_user_smd_pads_mm | M |
| 커넥터 J14 1번 핀 | VBAT_LOGIC_RAW, 핀 줄의 왼쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J14.pad1 | M |
| J14 라벨 | '1S BAT -+' | out/reports/routing_rules.json silkscreen.pin_order_labels.labels[J14].text | M |
| J14 아랫면에서 1번 핀 | 오른쪽 | out/fit_checks.json user_soldered_tht.per_ref.J14.pad1.side_of_the_pin_row_on_the_top_view (윗면 기준을 좌우로 뒤집음) | M |
| J14 라벨 끝 글자 | '+' = 1번 핀 (네모난 패드) | out/reports/routing_rules.json silkscreen.pin_order_labels.labels[J14].last_glyph_is_pad | M |
| 라벨 검사 | PASS | out/reports/routing_rules.json silkscreen.pin_order_labels.ok | M |
| 뒷면 다리 규칙 | 1.5 mm | out/fit_checks.json tht_leads | F (계획) |
| 다리 규칙 초과 | J15, J17 | out/fit_checks.json tht_leads | F (도면값) |
| 다리 규칙 초과 다리 수 | 4 | out/fit_checks.json tht_leads.rows (plated_holes) | M |
| 다리 공차 끝 초과 | J2, J5, J8~J12, J16, J18 | out/fit_checks.json tht_leads.rows (도면값은 규칙 안, 공차 끝은 규칙 초과; 참고 부품의 도면값) | F (도면값) |
| 다리 길이 미확인 | J1, J14 | out/fit_checks.json tht_leads | NOT_VERIFIED |
| 켜짐 전압 | 3.44 ~ 3.73 V | spec/rev3_plan.json power_block.uvlo | E |
| +5V 범위 | 4.84 ~ 5.33 V | spec/rev3_plan.json power_block.vout_V | E |
| 꺼짐 전압 | 3.00 ~ 3.25 V | spec/rev3_plan.json power_block.uvlo | E |
| MCU 1 mm 안 부품 | U1 13, U2 14, U3 20, U4 15 | out/fit_checks.json mcu_access | M |
| ESP 패드 크기 | 2.5 x 1 mm 와 1 x 1.8 mm | out/fit_checks.json user_soldered_smd.U7 pad_sizes_mm | M |
| ESP 네트 있는 패드 | 9개 (1, 3, 8, 15, 16, 17, 18, 21, 22번) | out/fit_checks.json user_soldered_smd.U7 pads_with_a_net | M |
| ESP 네트 없는 패드 | 13 | out/fit_checks.json user_soldered_smd.U7 pads_without_a_net | M |
| ESP-MCU 거리 | U3 패드에서 5.8 mm, U2 패드에서 9.6 mm | out/fit_checks.json user_soldered_smd.U7 mcu_pad_ring_to_body_mm | M |
| ESP 몸체 높이 | 3.2 mm | out/fit_checks.json user_soldered_smd.U7 body_height_mm | A |
| MCU 몸체 높이 | 1.2 mm | out/fit_checks.json user_soldered_smd.U7 mcu_body_height_mm | A |
| ESP에 가까운 MCU | U3 | out/fit_checks.json user_soldered_smd.U7 mcu_pad_ring_to_body_mm | M |
| ESP 가까운 커넥터 | J8 (4.73 mm), J9 (4.73 mm) | out/fit_checks.json user_soldered_tht.per_ref[].gap_to_user_smd_pads_mm.U7 (5 mm 미만) | M |
| ESP 옆 빈 헤더 구멍 | J20 1.78 mm, J3 1.78 mm | out/fit_checks.json user_soldered_smd.U7 unfitted_header_holes_within_3mm (ESP 패드에서) | M |
| U7 몸체-변 거리 | 0.15 mm | out/reports/edge_distance.json bodies_under_0p3 (U7) | M |
| ESP 옆 JLC 부품 | Q5 0.4 mm, C52 0.45 mm, R58 0.54 mm, C55 0.55 mm, C53 0.67 mm, C54 0.85 mm | out/fit_checks.json user_soldered_smd.U7 jlc_fitted_neighbours_within_1mm_of_the_pads (ESP 패드에서 1 mm 안) | M |
| ESP 패드-이웃 패드 최소 | 0.525 mm | out/fit_checks.json user_soldered_smd.U7 min_pad_to_foreign_pad_mm | M |
| 나머지 커넥터 수 | 10 | out/fit_checks.json user_soldered_tht.per_ref (J14, J15 제외) | M |
| 나머지 커넥터 납땜점 | 35 | out/fit_checks.json user_soldered_tht.per_ref[].joints (J14, J15 제외) | M |
| 스루홀 납땜점 | 39 | out/fit_checks.json user_soldered_tht.summary.joints_per_board | M |
| MCU 가까운 커넥터 | J5 (U2 패드에서 0.73 mm; 높이 5.9 mm), J9 (U3 패드에서 4.33 mm; 높이 3 mm), J10 (U3 패드에서 4.28 mm; 높이 3 mm), J11 (U3 패드에서 4.28 mm; 높이 3 mm), J18 (U4 패드에서 4.28 mm, U3 패드에서 4.34 mm; 높이 3 mm) | out/fit_checks.json user_soldered_tht.per_ref[].gap_to_mcu_pad_rings_mm (5 mm 미만; 높이는 A) | M |
| 나머지 커넥터 높이순 | J8~J12, J16, J18 (높이 3 mm) → J2, J5 (높이 5.9 mm) → J17 (높이 11 mm) | out/fit_checks.json user_soldered_tht.per_ref[].height_mm (J14, J15 제외, 낮은 것부터) | A (높이는 가정값) |
| 커넥터 J2 납땜점 | 6 | out/fit_checks.json user_soldered_tht.per_ref.J2.joints | M |
| 커넥터 J2 1번 핀 | +3V3, 핀 줄의 왼쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J2.pad1 | M |
| 커넥터 J2 몸체 | 직각형: 몸체가 핀 줄에서 아래쪽으로 10.08 mm 뻗음 (반대쪽 0.05 mm) → 아래쪽을 보게 눕힘 | out/fit_checks.json user_soldered_tht.per_ref.J2.body_outline_from_the_pin_row_mm | M |
| 커넥터 J2 가까운 변 | 위쪽 변에서 1.98 mm | out/fit_checks.json user_soldered_tht.per_ref.J2.body_to_that_edge_mm | M |
| 커넥터 J2 높이 | 5.9 mm | out/fit_checks.json user_soldered_tht.per_ref.J2.height_mm | A |
| 커넥터 J5 납땜점 | 6 | out/fit_checks.json user_soldered_tht.per_ref.J5.joints | M |
| 커넥터 J5 1번 핀 | 연결 없음, 핀 줄의 왼쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J5.pad1 | M |
| 커넥터 J5 몸체 | 직각형: 몸체가 핀 줄에서 아래쪽으로 10.08 mm 뻗음 (반대쪽 0.05 mm) → 아래쪽을 보게 눕힘 | out/fit_checks.json user_soldered_tht.per_ref.J5.body_outline_from_the_pin_row_mm | M |
| 커넥터 J5 가까운 변 | 오른쪽 변에서 0.76 mm | out/fit_checks.json user_soldered_tht.per_ref.J5.body_to_that_edge_mm | M |
| 커넥터 J5 높이 | 5.9 mm | out/fit_checks.json user_soldered_tht.per_ref.J5.height_mm | A |
| 커넥터 J8 납땜점 | 3 | out/fit_checks.json user_soldered_tht.per_ref.J8.joints | M |
| 커넥터 J8 1번 핀 | VANE1_SIG, 핀 줄의 아래쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J8.pad1 | M |
| 커넥터 J8 몸체 | 직각형: 몸체가 핀 줄에서 왼쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 왼쪽을 보게 눕힘 | out/fit_checks.json user_soldered_tht.per_ref.J8.body_outline_from_the_pin_row_mm | M |
| 커넥터 J8 가까운 변 | 왼쪽 변에서 0.73 mm | out/fit_checks.json user_soldered_tht.per_ref.J8.body_to_that_edge_mm | M |
| 커넥터 J8 높이 | 3 mm | out/fit_checks.json user_soldered_tht.per_ref.J8.height_mm | A |
| 커넥터 J9 납땜점 | 3 | out/fit_checks.json user_soldered_tht.per_ref.J9.joints | M |
| 커넥터 J9 1번 핀 | VANE2_SIG, 핀 줄의 아래쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J9.pad1 | M |
| 커넥터 J9 몸체 | 직각형: 몸체가 핀 줄에서 왼쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 왼쪽을 보게 눕힘 | out/fit_checks.json user_soldered_tht.per_ref.J9.body_outline_from_the_pin_row_mm | M |
| 커넥터 J9 가까운 변 | 왼쪽 변에서 0.73 mm | out/fit_checks.json user_soldered_tht.per_ref.J9.body_to_that_edge_mm | M |
| 커넥터 J9 높이 | 3 mm | out/fit_checks.json user_soldered_tht.per_ref.J9.height_mm | A |
| 커넥터 J10 납땜점 | 3 | out/fit_checks.json user_soldered_tht.per_ref.J10.joints | M |
| 커넥터 J10 1번 핀 | VANE3_SIG, 핀 줄의 아래쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J10.pad1 | M |
| 커넥터 J10 몸체 | 직각형: 몸체가 핀 줄에서 왼쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 왼쪽을 보게 눕힘 | out/fit_checks.json user_soldered_tht.per_ref.J10.body_outline_from_the_pin_row_mm | M |
| 커넥터 J10 가까운 변 | 왼쪽 변에서 0.73 mm | out/fit_checks.json user_soldered_tht.per_ref.J10.body_to_that_edge_mm | M |
| 커넥터 J10 높이 | 3 mm | out/fit_checks.json user_soldered_tht.per_ref.J10.height_mm | A |
| 커넥터 J11 납땜점 | 3 | out/fit_checks.json user_soldered_tht.per_ref.J11.joints | M |
| 커넥터 J11 1번 핀 | VANE4_SIG, 핀 줄의 아래쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J11.pad1 | M |
| 커넥터 J11 몸체 | 직각형: 몸체가 핀 줄에서 왼쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 왼쪽을 보게 눕힘 | out/fit_checks.json user_soldered_tht.per_ref.J11.body_outline_from_the_pin_row_mm | M |
| 커넥터 J11 가까운 변 | 왼쪽 변에서 0.73 mm | out/fit_checks.json user_soldered_tht.per_ref.J11.body_to_that_edge_mm | M |
| 커넥터 J11 높이 | 3 mm | out/fit_checks.json user_soldered_tht.per_ref.J11.height_mm | A |
| 커넥터 J12 납땜점 | 3 | out/fit_checks.json user_soldered_tht.per_ref.J12.joints | M |
| 커넥터 J12 1번 핀 | ESC_SIG, 핀 줄의 위쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J12.pad1 | M |
| 커넥터 J12 몸체 | 직각형: 몸체가 핀 줄에서 오른쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 오른쪽을 보게 눕힘 | out/fit_checks.json user_soldered_tht.per_ref.J12.body_outline_from_the_pin_row_mm | M |
| 커넥터 J12 가까운 변 | 오른쪽 변에서 0.73 mm | out/fit_checks.json user_soldered_tht.per_ref.J12.body_to_that_edge_mm | M |
| 커넥터 J12 높이 | 3 mm | out/fit_checks.json user_soldered_tht.per_ref.J12.height_mm | A |
| 커넥터 J14 몸체 | 수직형: 핀 줄에서 몸체 윤곽까지 위쪽 3.15 mm, 아래쪽 1.85 mm → 윤곽이 넓은 쪽이 위쪽 | out/fit_checks.json user_soldered_tht.per_ref.J14.body_outline_from_the_pin_row_mm | M |
| 커넥터 J14 가까운 변 | 왼쪽 변에서 0.73 mm | out/fit_checks.json user_soldered_tht.per_ref.J14.body_to_that_edge_mm | M |
| 커넥터 J14 높이 | 11 mm | out/fit_checks.json user_soldered_tht.per_ref.J14.height_mm | A |
| 커넥터 J15 1번 핀 | VBAT_LOGIC, 핀 줄의 왼쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J15.pad1 | M |
| 커넥터 J15 몸체 | 수직형: 핀 줄에서 몸체 윤곽까지 아래쪽 2.85 mm, 위쪽 2.16 mm → 윤곽이 넓은 쪽이 아래쪽 | out/fit_checks.json user_soldered_tht.per_ref.J15.body_outline_from_the_pin_row_mm | M |
| 커넥터 J15 가까운 변 | 왼쪽 변에서 0.32 mm | out/fit_checks.json user_soldered_tht.per_ref.J15.body_to_that_edge_mm | M |
| 커넥터 J15 높이 | 11 mm | out/fit_checks.json user_soldered_tht.per_ref.J15.height_mm | A |
| 커넥터 J16 납땜점 | 2 | out/fit_checks.json user_soldered_tht.per_ref.J16.joints | M |
| 커넥터 J16 1번 핀 | ARM_SW_EXT, 핀 줄의 왼쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J16.pad1 | M |
| 커넥터 J16 몸체 | 직각형: 몸체가 핀 줄에서 위쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 위쪽을 보게 눕힘 | out/fit_checks.json user_soldered_tht.per_ref.J16.body_outline_from_the_pin_row_mm | M |
| 커넥터 J16 가까운 변 | 위쪽 변에서 0.73 mm | out/fit_checks.json user_soldered_tht.per_ref.J16.body_to_that_edge_mm | M |
| 커넥터 J16 높이 | 3 mm | out/fit_checks.json user_soldered_tht.per_ref.J16.height_mm | A |
| 커넥터 J17 납땜점 | 2 | out/fit_checks.json user_soldered_tht.per_ref.J17.joints | M |
| 커넥터 J17 1번 핀 | +5V, 핀 줄의 왼쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J17.pad1 | M |
| 커넥터 J17 몸체 | 수직형: 핀 줄에서 몸체 윤곽까지 아래쪽 2.85 mm, 위쪽 2.16 mm → 윤곽이 넓은 쪽이 아래쪽 | out/fit_checks.json user_soldered_tht.per_ref.J17.body_outline_from_the_pin_row_mm | M |
| 커넥터 J17 가까운 변 | 오른쪽 변에서 0.73 mm | out/fit_checks.json user_soldered_tht.per_ref.J17.body_to_that_edge_mm | M |
| 커넥터 J17 높이 | 11 mm | out/fit_checks.json user_soldered_tht.per_ref.J17.height_mm | A |
| 커넥터 J18 납땜점 | 4 | out/fit_checks.json user_soldered_tht.per_ref.J18.joints | M |
| 커넥터 J18 1번 핀 | VS_VIN, 핀 줄의 아래쪽 끝 (네모난 패드) | out/fit_checks.json user_soldered_tht.per_ref.J18.pad1 | M |
| 커넥터 J18 몸체 | 직각형: 몸체가 핀 줄에서 왼쪽으로 10.09 mm 뻗음 (반대쪽 0.37 mm) → 왼쪽을 보게 눕힘 | out/fit_checks.json user_soldered_tht.per_ref.J18.body_outline_from_the_pin_row_mm | M |
| 커넥터 J18 가까운 변 | 왼쪽 변에서 0.73 mm | out/fit_checks.json user_soldered_tht.per_ref.J18.body_to_that_edge_mm | M |
| 커넥터 J18 높이 | 3 mm | out/fit_checks.json user_soldered_tht.per_ref.J18.height_mm | A |
| 핀 줄과 나란한 라벨 | J8 윗면 'V1 S+-', J17 윗면 'BUZ +-', J14 아랫면 '1S BAT -+', J7 아랫면 'GPS G RX TX V', J16 아랫면 'ARM SW GA', J2 아랫면 'BMP280 3V 3V SDA SCL G 3V' | out/reports/routing_rules.json silkscreen.pin_order_labels.labels (relation = along the pin row) | M |
| 1번 핀부터 적은 라벨 | J1, J3, J5, J9~J13, J18 | out/reports/routing_rules.json silkscreen.pin_order_labels.labels (relation = across the pin row / not a single pin row) | M |
| J1 납땜점 | 8 | out/fit_checks.json user_soldered_tht.summary.other_user_soldered_holes | M |
| GY-521 받침 높이 | 2.54 mm | spec/module_dimensions.json gy521.support_h | A |
| BMP280 헤더 종류 | 직각 | spec/module_dimensions.json bmp280.header_type | A |
| 모듈 간섭 보수적 충돌 | C3 -0.05 mm (J5_openlog 'insertion stroke beyond the plugged PCB end'), U6 -0.5 mm (J5_openlog 'insertion stroke beyond the plugged PCB end') | out/fit_checks.json module_keepouts.modules[].zones[].below_clearance_min_conservative (<= 0) | A |
| 프로그래밍 헤더 선택 | A | spec/carrier_spec.json prog_headers.option | M |
| B안 결론 | 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) | out/reports/programming_access.json | M |
| B안 2x3 자리 | SMD 0곳, THT 0곳 | out/reports/programming_access.json verdict_per_family | M |
| 코드 U8 | C919459 | out/jlcpcb_bom.csv | M |
| 코드 U9 | C20345924 | out/jlcpcb_bom.csv | M |
| 코드 L5 | C53426685 | out/jlcpcb_bom.csv | M |
| UVLO 저항 | R65 470k 1%, R66 270k 1%, R67 3.3M 1% | spec/carrier_spec.json | M |
| 코드 C45 | C11366 | out/jlcpcb_bom.csv | M |
| 참고 코드 J8 | C32713262 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (J8) | F |
| 참고 코드 J2 | C22436147 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (J2) | F |
| U1-U4 with_mcu | C6510 | out/jlcpcb_bom_with_mcu.csv | M |
| 코드 J5 | BOM에 없음 | out/jlcpcb_bom.csv | M |
| 참고 코드 J15 | C131337 | spec/rev3_plan.json bom_lines_now_bought_by_the_user (J15) | F |
| 코드 Q1 | C40912 | out/jlcpcb_bom.csv | M |
| 분압 저항 | R15 47k 1%, R16 47k 1% | spec/carrier_spec.json | M |
| fit user | 14 | spec/carrier_spec.json | M |
| 변경 행 | 교체 11, 추가 15, 삭제 1, 그대로 (부품 없음) 10, 그대로 140 | spec/rev3_plan.json changes | F (계획) |
