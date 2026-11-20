# VTOL 아비오닉스 캐리어 PCB rev 3.1 (JLC가 실장해서 보내는 판)

**이 보드는 아직 만들거나 전원을 넣어 본 적이 없습니다 (NOT_VERIFIED).** 아래의 PASS는 모두 도구(kicad-cli ERC/DRC, 스크립트)의 결과이며 보드가 동작한다는 뜻이 아닙니다. JLC가 이 파일을 받아 주는지도 업로드 후 JLC의 DFM 결과와 견적을 봐야 압니다.

**2026-11-13 rev 3.1 - 로켓용 분리 서보 출력 (사용자 결정: "캐리어 PCB를 로켓에도 씀" / "B: 서보 열에 직각 헤더"):** 같은 캐리어를 로켓에도 씁니다. 로켓은 CanSat 앞부분을 놓아 주는 분리 서보를 FCU U2의 PE5 / OC3C 핀으로 움직이는데 (펌웨어 src/hal/pwm.c:49, main.c:518 - 바꾸지 않음), rev 3에서는 이 핀이 비어 있었습니다. 지금은 U2.7 (PE5 / OC3C): NC → RELEASE_PWM → R22 (220 Ohm 0603, JLC가 실장; BOM 줄 220R 1%, R17-R22 (6개), C22962) → RELEASE_SIG → J13 입니다. **J13은 부품이 아니라 도금 구멍 패드 하나**(KiCad (127.05, 66.85) mm, J12(ESC) 옆 +x / +y 모서리)이고, 로켓에서는 사용자가 서보 Y자 리드의 신호선을 여기에 납땜합니다. 서보의 + / - 는 그 Y자 리드가 ESC 리드 J12의 2번(SERVO_5V) / 3번(GND_PWR)에서 받습니다. VTOL에서는 J13을 비워 둡니다.

사용자가 고른 것은 B안(-x 서보 열에 직각 1x3 헤더 J13, J18을 VIN 1핀으로 줄임)이었지만, B안은 rev 3 배터리 선을 바꾸지 않는다는 조건에서는 놓을 수 없음: -x 서보 열에 직각 1x3 J13을 J8~J11과 같은 8.62 mm 간격으로 넣고 J18을 1핀으로 줄이면, 배터리 선 두 개(윗면 2.0 mm VBAT_LOGIC, 아랫면 2.4 mm VBAT_LOGIC_SW) 사이에 45.06 mm가 필요한데 44.60 mm뿐입니다. 801곳을 찾아본 가장 좋은 자리도 열의 양 끝에서 0.13 mm 간격 규칙보다 0.23 mm 모자랐습니다. 배터리 선을 바꾸지 않는다는 조건은 사용자 결정도 검사 기준도 아니고 (검사는 배터리 경로 폭 2.0 mm 이상, 0.035 Ohm 이하만 요구), rev 3.1 작업이 rev 3 재설계 때 build_pcb.py에 깔아 둔 미리 그린 배선을 건드리지 않으려고 스스로 둔 것입니다 (M). 검사 규칙을 느슨하게 하지 않고, 사용자가 정해 둔 대안인 C안(구석의 1핀 패드 + R22, 서보 전원은 J12의 Y자 리드)으로 만들었습니다. **그래서 J18은 rev 3 그대로 1x4 (VIN / VTET / VBUF / ISGN) 이고, 테더·하이브리드용 입력과 그 분압 부품도 그대로 남아 있습니다** (B안에서만 필요했던 변경이라 하지 않았음; 테더·하이브리드 빌드에서 잃는 것은 없음). B안의 새 헤더(C32713262)와 1핀 J18용 부품도 사지 않습니다. **B안이 안 된 것은 배터리 선을 그대로 둔 조건 때문입니다.** B안을 꼭 원하면 그 배터리 선 구간을 다시 그리는 설계 변경을 사용자가 정해야 합니다. 예를 들어 아랫면 VBAT_LOGIC_SW 띠를 2.4 → 2.0 mm로 줄이면 약 0.4 mm가 생기고 배터리 경로 저항은 약 0.0310 → 0.0313 Ohm (기준 0.035 Ohm)으로 추정되지만, 모자란 0.46 mm를 다 채우지 못해 J15 쪽 윗면 띠도 옮겨야 할 수 있습니다 (E, 2026-11-13 검증 단계의 추정; 만들거나 배선·검사해 보지 않음 - NOT_VERIFIED).

**JLC 장바구니의 Y24는 rev 3 거버입니다. 결제하지 마십시오.** 그보다 옛 거버가 든 Y23, Y20도 결제하지 마십시오 (08_PCB README에 적힌 상태). 장바구니는 이 작업에서 열지 않았으므로 어느 항목에 어느 거버가 들었는지는 결제 전에 직접 확인하십시오. rev 3.1 거버(이 폴더의 새 파일)의 JLC 견적과 DFM 결과는 올려 보기 전까지 NOT_VERIFIED입니다.

**2026-11-12 변경 (사용자 요청: "아, esp도 내가 직접 실장할거야. 그럼 이코노믹으로 진행하고 pcb의 코팅도 무연으로 해줘", "이코노믹으로 스루홀 제외하고 어떤 부품을 따로 구매해야하는건지 리스트를 정리해줘"):** (1) Wi-Fi 모듈 ESP-12F (U7)는 JLC가 아니라 사용자가 직접 납땜합니다 (JLC BOM / CPL에서 빠짐, 두 거버 묶음 모두 U7 패드 22개에 솔더 페이스트 구멍 없음). (2) 스루홀 커넥터 12개(J2, J5, J8~J12, J14~J18, 납땜 39곳)도 JLC가 달지 않고 사용자가 사서 납땜합니다. (3) 주문은 Economic PCBA이고 JLC는 SMT 부품 145개(BOM 33줄)만 실장합니다. (4) 기판 표면처리는 무연 HASL입니다 (보드 파일 표기 'HAL lead-free'). 이 변경 때는 풋프린트·네트·위치와 구리가 그대로였습니다. 실크에서는 같은 날 검증 지적으로 아랫면 이름표의 핀 글자 순서만 고쳤습니다: 핀 줄과 나란히 적힌 이름표는 이제 그 면에서 본 패드 순서대로입니다 (아랫면: J14 '1S BAT -+', J7 'GPS G RX TX V', J16 'ARM SW GA', J2 'BMP280 3V 3V SDA SCL G 3V'; 검사 PASS, M). 주문 방법은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md`, 따로 살 부품은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/따로_살_부품_ko.md`.

**2026-11-12 재설계 (사용자 요청: "그럼 다시 설계를 진행해줘. 실패한게 있다면 완료되었다고 말하지마" / "보드의 사이즈를 조정하는 방법은 없어?" / "옮겨도 됨"):** 재설계 전 보드(sha256 cc750ce9e23322b2...)는 배선 규칙 16 / 21 PASS, FAIL: battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance 이었습니다 (M). 부품을 바꾸지 않고 자리를 주려고 (1) 기판 -y 변에 승압 회로용 탭을 붙였고 (외접 크기 76 x 77.2 mm; 본체 76 x 70 mm 사각형과 그 변은 그대로), (2) 케이블타이 띠 안의 프로그래밍 구멍 J19 (x +0, y +0.55 mm), J22 (x +0, y +0.55 mm), J26 (x +0, y +0.55 mm) 을 옮겼고 (사용자 승인), (3) C42를 x +0.20 mm (지금 x 72.8, y 69.3 mm) 옮겼고, (4) 승압부(U8, L5, C41, C43, C57, R63, C58, R64)를 탭에 다시 놓고, (5) 배터리 전원 경로를 2.0 mm 폭으로 다시 그린 뒤 전체를 새로 배선했습니다. 회로도와 부품(BOM)은 바뀌지 않았습니다. 자세한 내용은 1장과 3장, 검사를 글자로만 통과한 부분은 1장의 '목적과 다른 점'에 있습니다.

표시: M = 이 PC에서 도구로 측정, E = 추정(계산), A = 가정(실물로 확인 필요), F = 문서·데이터시트 근거. 근거가 없으면 NOT_VERIFIED. 이 문서는 `scripts/docs_ko.py` 가 보고서 파일에서 숫자를 읽어 만든 것입니다 (손으로 고치지 말고 다시 생성). 측정·계산된 숫자의 출처는 맨 끝 8장에 있고, 그 표에 없는 숫자(칩 종류, 규칙 값, 시험 조건)는 계획서 `spec/REV3_PLAN.md` 와 사양 `spec/carrier_spec.json` 에서 옮긴 고정값(F)입니다.

## 1. 지금 상태

**출고 판정(도구 검사 기준): ORDERABLE (tool checks only; NOT_VERIFIED - never built)** - 판정을 막는 항목: 없음.

전체 다시 만들기 두 번이 같은 파일을 내는지 (determinism.py, 출고 판정과 따로 봄): PASS - 바이트 비교 파일 71개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 0개. 다른 폴더에서: PASS - 다른 폴더에 복사해 다시 만든 결정적 파일 60개 중 55개가 이 폴더의 것과 바이트까지 같음 (보드, 회로도, 거버와 zip, BOM, CPL 등 주문에 쓰는 파일 43개는 모두 여기에 듦). 나머지 5개(README_ko.md, out/ORDER_ko.md, out/따로_살_부품_ko.md, spec/SPEC_ko.md, spec/parts_sources.json)는 자기 폴더 이름이나 그런 파일의 sha256을 담고 있어, 폴더 이름과 그 sha256을 바꿔 넣은 뒤에 같음. 그림 단계 산출물 11개는 그 폴더에서 다시 만들지 않아 비교하지 않음. STEP: kicad-cli가 쓴 STEP 항목 131461개의 번호와 순서를 내용으로 다시 매김 (scripts/step_canon.py); 다시 읽어 모든 항목이 같은 그래프인지 확인: 통과.

이 판정은 도구 검사만 본 것입니다. 보드는 만들어지거나 전원이 들어간 적이 없고, JLC 업로드 미리보기·DFM·견적, 위치 확인과 결제는 사용자가 합니다 (주문 안내서 0장과 3장).

**배선 규칙 21 / 21 PASS** (FAIL: 없음; 재설계 전 보드: 16 / 21 PASS, FAIL: battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance). 규칙과 기준값은 바꾸지 않았습니다. 바꾼 검사 코드는 새 외곽에 맞춘 부분, 추가한 검사, 사용자가 승인한 구멍 이동, 결정성 검사의 폴더 이름 바꾸기, 그리고 rev 3.1의 선언된 MCU 핀 하나(U2.7)와 새 부품 J13 / R22 항목뿐입니다 (목록은 3장 끝). 지금 보드에서 잰 값:

- 배터리 전원 경로 (J14 → Q10 → J15 → L5): 구리 저항 0.03096 Ohm (20 C, E; 목표 0.035 Ohm), 전류가 흐르는 구간의 가장 좁은 폭 2 mm (M), 2.0 mm보다 좁은 길이 0 mm. 구간별: J14.1 → Q10.3 0.00191 Ohm, 7.78 mm, 가장 좁은 곳 2 mm, 비아 0개; Q10.2 → J15.1 0.00388 Ohm, 15.77 mm, 가장 좁은 곳 2 mm, 비아 0개; J15.2 → L5.1 0.02517 Ohm, 132.22 mm, 가장 좁은 곳 2 mm, 비아 2개. **60 C에서는 0.03583 Ohm (E)로 0.035 Ohm 목표를 넘습니다** (검사는 계획대로 20 C 값으로 판정). 재설계 전: 20 C 0.03173 Ohm, 60 C 0.03672 Ohm, 2.0 mm보다 좁은 길이 8.57 mm.
- 출력 커패시터 (검사 hot_loop PASS): C57 +5V 패드-U8.6 0.75 mm, GND 패드-U8.4 1.819 mm; C43 +5V 패드-U8.6 0.868 mm, GND 패드-U8.4 1.827 mm (기준 각 2.0 mm 이하; 재설계 전 C57 0.95 / 1.611 mm; C43 3.25 / 3.537 mm). 윗면에서 U8 GND 핀과 이어진 출력 커패시터 GND: C57.2, C43.2.
- 입력 커패시터 C41 (검사 input_cap_l5 PASS, input_cap_vin PASS): C41 → L5 입력 패드 구리 길이 0.9 mm (기준 5 mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E; U8 VIN 패드와 L5 입력 패드 사이 직선 거리 2.103 mm); C41 → U8 VIN 구리 길이 4.17 mm (비아 0개; 기준 5 mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E), GND 패드가 U8 GND와 같은 윗면 구리: 예. 재설계 전 C41 → L5 경로는 13.53 mm 였습니다.
- 윗면 GND: PASS (U8.4, C57.2, C43.2, C41.2 이 같은 윗면 GND 구리 한 덩어리 위에 있음; 자기 비아가 있는 패드 4 / 4).
- SW 노드 (검사 sw_node PASS): U8의 SW 패드와 L5의 SW 패드 사이 간격 1.876 mm (기준 2 mm 이하 - 계획 12장의 'L5는 U8 핀 바로 옆'에 숫자가 없어 스크립트가 계획 12장의 2 mm를 빌려 읽은 값, E), SW 선 구리 넓이 1.061 mm2, 구리 저항 약 4.3 mOhm (20 C, E), SW 선이 패드 사이를 지나는 다른 부품: 없음. SW 선: 길이 3.032 mm, 폭 0.35 mm, 비아 0개, F.Cu만. 재설계 전 패드 간격 5.551 mm.
- 크리스털 거리 (검사 crystal_distance PASS): rev 2c의 Pololu 핀은 NAV 크리스털 Y1 패드에서 9.217 mm, U1의 XTAL 핀에서 9.616 mm 떨어져 있었습니다. 지금은 가장 가까운 부품이 Y1 패드에서 9.55 mm (C43), XTAL 핀에서 9.935 mm (C43) 이고, SW 구리(선과 두 패드)는 Y1 패드에서 12.426 mm, XTAL 핀에서 12.969 mm, 크리스털 배선 구리에서 11.415 mm (rev 2c Pololu 핀은 크리스털 배선에서 8.243 mm) 입니다. rev 2c보다 가까운 부품: 없음. 부품별 (가까운 순): C43 Y1 9.55 / XTAL 핀 9.935 mm; C57 Y1 11.851 / XTAL 핀 12.753 mm; U8 Y1 11.925 / XTAL 핀 12.319 mm; L5 Y1 14.65 / XTAL 핀 15.025 mm; C41 Y1 15.762 / XTAL 핀 15.485 mm (rev 2c 기준 Y1 9.217 / XTAL 핀 9.616 mm). 재설계 전 rev 2c보다 가까웠던 부품: C43, C57, L5, U8.
- 되먹임(FB) 선: PASS (출력 커패시터 패드 사이를 지나는 FB 구리 0곳, 가장 가까운 간격 0.65 mm, FB 선 길이 4.787 mm).
- 레일 쪽 변의 구리: PASS: 레일이 붙는 두 긴 변(y 65 변, 그리고 y 135 변과 탭의 모든 변)에서 0.4 mm 이상 (기준 0.4 mm), 나머지 두 변 0.3005 mm (보드 규칙 0.3 mm).
- 기체 맞춤 (탭 포함, 검사 board_fit PASS): 흡입구 보어 벽까지 4.44 mm (기준 2 mm 이상), 탭에서 270도 통풍구까지 13.05 mm (기준 2 mm 이상), 탭에서 모터 리드 연결부 영역까지 21.869 mm (본체 사각형 쪽은 재설계 전과 같음), 315도 조인트 나사에서 평면 2 mm 안의 부품: 없음, 나사 아래 기판 윗면 높이 여유 1.3 mm (기준 1 mm 이상), 탭 안 부품 C41, C57, C58, L5, R63, R64, U8 (높이 한계 13.4 mm 넘는 것: 없음).

**목적과 다른 점 (검사는 통과했지만 글자로만 통과했거나 여유가 작은 곳 - 숨기지 않음):**

- 입력 커패시터 C41의 GND: C41 GND 패드의 비아 (118.525, 137.2)에서 윗면 GND 선을 따라 13.923 mm (끝점 (127.1, 135.95), (118.525, 138.05); 가는 0.2 mm 선이 FB 분압 저항 두 줄 사이를 지나 C43 GND 패드를 거쳐 U8 GND 핀으로 감), C41 GND 패드-U8.4 직선 간격 8.033 mm. 검사 hot_loop_top_gnd와 input_cap_vin은 이 가는 선 덕분에 통과합니다. 짧은 고리가 아닙니다: C41의 고주파 귀환은 실제로는 자기 비아(0.65 mm 떨어짐)를 거쳐 내층 GND 면으로 갑니다. 탭 안에서는 U8 VIN 선, L5 입력 패드와 배터리 선이 C41 GND 패드와 U8 GND 핀 사이를 막고 있습니다. 내층에서의 귀환 거리: C43.2 GND 비아 → U8.4 GND 비아 5.15 mm; C57.2 GND 비아 → U8.4 GND 비아 4.02 mm; C41.2 GND 비아 → U8.4 GND 비아 8.33 mm (비아 중심 사이 직선, In1 GND 면 위). TI 데이터시트 10.1절은 입력 커패시터를 VIN 핀과 GND 핀 가까이 두라고 하지만, C41은 L5 입력 패드 옆에 있고 U8 VIN 핀(3번) 옆에는 따로 커패시터가 없습니다 (작은 커패시터를 더하려면 회로도 변경 = 사용자 결정).
- C41의 VIN 쪽 경로(위 숫자)는 L5 입력 패드를 지나 U8 VIN 핀으로 갑니다.
- 출력 커패시터 C43의 GND: C43 GND 패드에서 U8 GND 핀까지 윗면 GND 선 폭 0.2 mm 0.358 mm, 폭 0.25 mm 3.1 mm (비아 연결선 제외; SOT-563 패드 두 줄 사이를 지나는 가는 선; 패드마다 0.3 mm GND 비아 하나씩). 고리는 작지만 TI 예시(10.2절)·발열 안내(10.3절)처럼 넓은 GND 구리와 여러 비아가 아니라 가는 선입니다. U8의 발열은 측정 전까지 NOT_VERIFIED입니다.
- 배터리 경로는 L5 입력 패드 바로 앞에서 비아 두 개를 지납니다: 드릴 0.4 mm 2개. 드릴 0.4 mm 이상 비아 2개의 도금 구리 단면은 합쳐서 약 0.045 mm2 로, 폭 2.0 mm 1 oz 배선의 0.07 mm2 보다 작습니다 (도금 18 um 가정, E). 폭 검사(2.0 mm)는 선만 재므로 이 비아들은 글자로만 통과합니다.
- 출력 커패시터 C57의 GND 패드는 SW 선(U8.5 → L5.2) 건너편에 있어, 윗면에서는 L5 둘레·아래의 GND 면(구리 채움)을 통해서만 U8 GND 핀과 이어집니다. 검사는 그 면이 한 덩어리라서 통과합니다. C57의 실제 귀환은 자기 비아와 U8 GND 핀의 비아를 거친 내층 GND입니다. 배선 프로그램이 이 면을 끊지 못하게 탭 위 윗면에 배선 금지 영역을 두었습니다 (Freerouting 입력 파일에만 있고 보드 파일에는 없음).
- SW 패드 간격 여유는 0.124 mm 뿐입니다 (부품 자리가 맞닿아 배치로 정해짐; 배선으로는 줄일 수 없음).
- 미리 그린 배선(승압부, 배터리 경로의 갈래)은 이제 배선 마무리 단계의 자동 넓히기에서 빠져 설계한 폭 그대로입니다. 재설계 전 흐름은 이 갈래들을 넓혔습니다. 2.0 mm 배터리 경로는 이와 무관합니다.
- 회로 IR(설계 데이터)은 외곽을 사각형으로만 담을 수 있어 76 x 70 mm 사각형을 그대로 갖고, 탭이 있는 실제 외곽은 `scripts/outline.py` 에서 옵니다. 외곽에 대해서는 IR이 유일한 설계 데이터가 아닙니다.
- 이 영향(스위칭 과전압, 입력 리플, 잡음, 크리스털 흔들림)은 보드에서 오실로스코프로 재기 전까지 NOT_VERIFIED입니다 (주문 안내서 7장).

| 항목 | 결과 | 표시 |
|---|---|---|
| 회로도 ERC | 오류 0, 경고 0 (시트 8장) | M |
| PCB DRC (`--schematic-parity --severity-all --refill-zones`) | 위반 0, 경고 0, 미연결 0, 회로도 불일치 0 | M |
| 넷리스트 = 사양 | PASS (부품 176, 네트 107) | M |
| 보드 패드 넷 = IR | PASS (핀 510 / 510) | M |
| MCU 4개 핀 = rev 2c | PASS (핀 256, 차이 0, 계획 수정으로 선언한 변경: U2.7 NC → RELEASE_PWM) | M |
| U8 / U9 핀-네트 표 | PASS | M (표는 F) |
| 배선 규칙 (계획 13장) | 21 / 21 PASS, FAIL: 없음 | M / E |
| 크리스털 XTAL1 길이 (rev 2c보다 나빠지면 안 됨) | 5.96 mm (한계 5.96 mm) | M |
| 설계 규칙 파일 = rev 2c | PASS | M |
| 구리 섬 / 패드에 닿는 비아 | 없음 (모든 네트 한 덩어리) / 0 | M |
| JLC 부품 재고·주문 가능 확인 | PASS (33/33줄), with_mcu PASS, 2026-11-13T20:26:50+09:00 | M (그 시각 기준) |
| 주문 파일 형식·fit 집합 검사 | PASS | M |
| 주문 파일을 두 번 써서 같은지 | PASS | M |
| 전체 다시 만들기 두 번 = 같은 파일 | PASS - 바이트 비교 파일 71개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 0개 | M |
| 다른 폴더에서 다시 만들어도 같은 파일 | PASS - 다른 폴더에 복사해 다시 만든 결정적 파일 60개 중 55개가 이 폴더의 것과 바이트까지 같음 (보드, 회로도, 거버와 zip, BOM, CPL 등 주문에 쓰는 파일 43개는 모두 여기에 듦). 나머지 5개(README_ko.md, out/ORDER_ko.md, out/따로_살_부품_ko.md, spec/SPEC_ko.md, spec/parts_sources.json)는 자기 폴더 이름이나 그런 파일의 sha256을 담고 있어, 폴더 이름과 그 sha256을 바꿔 넣은 뒤에 같음. 그림 단계 산출물 11개는 그 폴더에서 다시 만들지 않아 비교하지 않음 | M |
| 가장자리 2.5 mm 안 부품 몸체 | 모든 자리 33개 = JLC 실장 13개 (with_mcu 14개) + JLC가 붙이지 않는 자리 20개 (U4, J1, J2, J5, J8, J9, J10, J11, J12, J14, J15, J16, J17, J18, U7, J13, J19, JP1, Q3, R47) (0.3 mm 안: U7 0.15 mm, J15 0.29 mm - 둘 다 사용자가 납땜하는 자리; JLC 실장 부품 중 가장 가까운 것 R60 0.537 mm) | M |
| 모듈(OpenLog / BMP280 / GY-521) 간섭, 보통 경우 | A / NOT_VERIFIED, 가정 치수(보통 경우)로 충돌 0건; 최소 간격 0.5 mm 미만: U1 | A / NOT_VERIFIED |
| 모듈 간섭, 보수적 경우 (OpenLog 아랫면 부품 1.2 mm 가정) | 충돌: C3 -0.05 mm (J5_openlog 'insertion stroke beyond the plugged PCB end'), U6 -0.5 mm (J5_openlog 'insertion stroke beyond the plugged PCB end'); 0.5 mm 미만: C48, Q2, Q8, U1, U2, Y1 | A / NOT_VERIFIED |

## 2. 이 기판은 무엇인가

- 크기: 본체 76 x 70 mm 사각형 + -y 변의 탭 (KiCad x 117.3..132, y 135..142.2 mm (바깥 모서리 r 1 mm, 본체와 만나는 안쪽 모서리 r 1 mm, 기판 재료는 x 116.3..133), 넓이 105.8 mm2) → 외접 76 x 77.2 mm, 기판 넓이 5425 mm2. 두께 1.6 mm, 4층 (F.Cu 신호와 부품 / In1 GND 전면 / In2 +5V 전면 / B.Cu 신호). 부착 구멍 없음: 3 mm 폼테이프 + 케이블타이 1개. 케이블타이 띠(x -3..+3 mm)에는 실장 부품이 없습니다.
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
| IO | 서보 J8~J11, ESC J12, 아밍 스위치 J16, 부저 J17, 6S 전압 감지 J18 등 입출력 커넥터 | 39 | 30 |
| LOG | OpenLog 소켓 J5 | 1 | 0 |
| LINK | 칩 사이 연결선 (UART / SPI) 부품 | 5 | 3 |

## 3. rev 2c에서 바뀐 것과 이유

- **5 V 전원 모듈을 없앴습니다.** 꽂는 Pololu 모듈(J6, 디바이스마트 부품)은 JLC가 실장할 수 없어서, JLC 부품으로 만든 승압 회로 U8 TPS61023 (C919459) + L5 (C53426685)를 예전 모듈 자리(+x / -y 모서리)에 올렸습니다. 이유: 요청(모든 부품 JLC 실장).
- **저전압 차단을 유지했습니다.** 예전 모듈 안에 있던 기능입니다. U9 TLV3012B (C20345924) + R65 470k 1%, R66 270k 1%, R67 3.3M 1%: 켜짐 3.44 ~ 3.73 V, 꺼짐 3.00 ~ 3.25 V (계산값 E, 실측 전).
- **Wi-Fi 모듈 ESP-12E → ESP-12F** (참고 부품 LCSC C82891). 핀과 풋프린트가 같습니다. 계획은 JLC가 실장할 수 있는 부품이라 ESP-12F를 골랐는데, 2026-11-12 사용자 결정으로 이 모듈은 사용자가 직접 납땜합니다 (JLC BOM의 U7: BOM에 없음).
- **스루홀 커넥터 12개는 사용자가 사서 납땜합니다** (2026-11-12 사용자 결정; 계획은 JLC가 손납땜). 참고 부품 번호 (계획 때 JLC BOM에 있던 번호, F): 소켓 J2 / J5 C22436147, 서보 헤더 J8~J12 C32713262, J16 C32713261, J18 C32713263, 배터리 J14 C5364642, 스위치·부저 J15 / J17 C131337. JLC BOM에는 없습니다 (J2: BOM에 없음). 탄탈 C45 (C11366, JLC 실장)와 헤더·소켓 번호 일부는 재고나 도면 근거로 바꿨고 D3 / D12에서 rev 2c 번호로 되돌릴 수 있습니다.
- **rev 3.1 (2026-11-13, 사용자 결정 "캐리어 PCB를 로켓에도 씀" / "B: 서보 열에 직각 헤더") - 로켓용 분리 서보 출력:** 부품 두 자리를 더했습니다. R22 (220 Ohm, 서보 신호선 R17~R21과 같은 JLC 줄 → 220R 1%, R17-R22 (6개), C22962; 자리 KiCad (130.125, 67.15) mm) 와 J13 (도금 구멍 패드 1개, 부품 없음, KiCad (127.05, 66.85) mm, J12(ESC) 옆 +x / +y 모서리). 새 네트 두 개: RELEASE_PWM (U2.7 → R22.1; 배선 61.28 mm (B.Cu / F.Cu), 비아 6개), RELEASE_SIG (R22.2 → J13; 배선 4.38 mm (F.Cu), 비아 0개). MCU 핀은 하나만 바뀌었습니다: U2.7 (PE5 / OC3C): NC → RELEASE_PWM (계획 수정 rocket_release_output에 선언). 그 밖의 rev 3 부품은 자리·방향이 모두 그대로이고, 잠가 둔 미리 그린 배선(배터리 경로, 승압부 등)도 하나도 바뀌지 않았습니다 (rev 3.1 배치·배선 단계에서 rev 3 보드와 비교, M). 새 배선 때문에 주변 네트 몇 개의 자동 배선이 바뀌었습니다 (Freerouting을 새로 돌림; 세션 파일로 재현). J18은 그대로 1x4입니다 (B안이 안 되어 줄이지 않음).
- **rev 3.1 ESP 안테나 구역 표시:** false - 겉상자 검사의 거짓 경보: board_metrics.py는 선의 겉상자(bounding box)가 안테나 금지 상자 (74.63, 65.0, 98.87, 72.4)에 닿으면 적는데, 적힌 것(PCB_TRACK ESP_TXD0 at (74.18, 72.48))은 대각선 선이라 실제 구리는 상자 밖에 있습니다 (그 네트 구리에서 상자까지 가장 가까운 거리 0.274 mm, 잰 선·비아 1개). KiCad 규칙 영역 ESP12E_antenna_keepout (네 층 모두 선·비아·면 금지)을 넣은 DRC는 위반 0개입니다. 이 값은 출고 판정에 들어가지 않으며, 기준(board_metrics.py)은 바꾸지 않았습니다 (rev 3에서는 true).
- **rev 3.1에서 바꾼 검사 코드 (규칙·기준값은 그대로):** MCU 핀맵 검사(routing_checks.py, rev3_checks.py)는 계획 수정에 선언된 핀(U2.7 NC → RELEASE_PWM)만 새 값과 비교하고, 그 핀의 옛 값도 rev 2c 기준과 같아야 합니다. 나머지 핀은 예전과 똑같이 비교합니다. 네트 검사는 새 네트에 기존 부품 핀이 들어갈 때 계획에 적힌 핀(U2.7)과 정확히 같아야 합니다. place.py에는 새 패드의 높이 값과 J13 / R22 자리만 더했습니다. rev3_spec.py의 계획 대조는 추가 부품 수와 계획 수정 목록을 J13 / R22를 넣은 새 값으로 비교합니다.
- **그대로인 것:** MCU 4개의 핀 256개가 가는 네트 (rev 3.1의 U2.7 하나만 빼고; PASS (핀 256, 차이 0, 계획 수정으로 선언한 변경: U2.7 NC → RELEASE_PWM)), 본체 76 x 70 mm 사각형과 그 변, 커넥터·MCU·크리스털·모듈 소켓·ESP 위치와 방향 (고정 부품 58개 중 rev 2c 자리에서 움직인 것: 없음; 프로그래밍 구멍 3개는 아래 재설계에서 사용자 승인으로 옮김), 설계 규칙 (PASS), 크리스털 배선 (5.96 mm (한계 5.96 mm)).
- **2026-11-12 재설계 - 외곽에 탭 추가** (사용자 요청 "그럼 다시 설계를 진행해줘. 실패한게 있다면 완료되었다고 말하지마" / "보드의 사이즈를 조정하는 방법은 없어?" / "옮겨도 됨"): 지금 부품(U8 TPS61023, L5 FXL0420, 22 uF 1206 커패시터)으로 승압부 검사와 크리스털 거리 검사를 모두 지키려면 76 x 70 mm 안에 자리가 모자랐습니다 (재설계 전 연구, M). 그래서 본체의 변은 그대로 두고 -y 변에 탭을 붙였습니다: KiCad x 117.3..132, y 135..142.2 mm (바깥 모서리 r 1 mm, 본체와 만나는 안쪽 모서리 r 1 mm, 기판 재료는 x 116.3..133), 넓이 105.8 mm2, 외접 76 x 77.2 mm. 탭은 기체 CAD에서 잰 허용 범위(KiCad x 106..134, 바깥 변 y 143.74 mm 이하 (바깥 모서리 r 1 mm)) 안에 있습니다. 이것은 계획 불변조건 (b) '외곽 76 x 70 mm 그대로'에서 벗어난 사용자 결정입니다.
- **승압부 자리 (재설계):** U8 (126.5, 135.45) 0도; L5 (125.15, 138.75) 0도; C43 (127.1, 133.3) 180도; C57 (129.2, 136.75) 270도; C41 (120, 138.75) 180도; R63 (123.85, 133.85) 270도; C58 (122.35, 133.85) 270도; R64 (120.85, 133.85) 90도. U8의 VOUT / GND 핀 옆에 출력 커패시터 C43(위)과 C57(오른쪽), 아래에 L5, L5 입력 패드 옆에 C41. 저전압 차단(U9 주변), LDO U6, 그 밖의 부품 자리는 그대로입니다.
- **프로그래밍 구멍 3개 이동 (사용자 승인 "옮겨도 됨"):** J19 (x +0, y +0.55 mm), J22 (x +0, y +0.55 mm), J26 (x +0, y +0.55 mm). 케이블타이 띠를 건너는 배터리 선이 2.0 mm 폭으로 지나갈 틈을 만들기 위해서입니다. 배치 검사(place.py)는 이 세 부품만 'rev 2c 자리 + 이 거리'와 정확히 같은지 봅니다 (다른 고정 부품은 rev 2c 자리 그대로인지 봄). 외곽 맞춤과 추가한 검사 말고, 설계 판정에 쓰이는 검사 코드에서 사용자 결정 때문에 바꾼 곳은 이것과 rev 3.1의 선언된 MCU 핀 하나(아래)뿐입니다.
- **C42 이동:** x +0.20 mm (지금 x 72.8, y 69.3 mm) - 2.0 mm 배터리 선이 Q10 3번 패드로 들어가게.
- **배터리 전원 경로:** 선은 모든 구간 2 mm 이상 (1장의 숫자). 다만 L5 입력 패드 앞의 비아 두 개(드릴 0.4 mm 2개)는 선이 아니라서 폭 검사 대상이 아니고, 도금 단면이 2.0 mm 선보다 작습니다 (1장 '목적과 다른 점').
- **바꾼 검사 코드 (규칙·기준값은 그대로):** 외곽 맞춤 - 판의 모서리·변으로 재던 검사(routing_checks 레일 변 구리와 식별 문구의 가장자리 거리, place.py 기판 안쪽, fit_checks와 edge_distance의 -y 변 거리, silk.py 글자 자리, prog_access.py 헤더 자리 검색의 기판 밖 칸)가 이제 탭의 모든 변을 실제 선분으로 잽니다 (y 135 선 옆에서는 예전과 같은 값). 추가한 검사 - 기체 맞춤 board_fit (board_metrics.py)과 315도 조인트 나사 아래 부품 금지 영역 (place.py). 사용자 결정 - J26 / J22 / J19의 기준 자리 (위). 결정성 검사 determinism.py - 다른 폴더 비교의 폴더 이름 바꾸기 규칙만 작업 복사본에서도 돌도록 고침 (비교 종류와 PASS 규칙은 그대로).
- **내보내기 (검사 아님):** STEP 파일 - kicad-cli가 쓴 STEP 항목 131461개의 번호와 순서를 내용으로 다시 매김 (scripts/step_canon.py); 다시 읽어 모든 항목이 같은 그래프인지 확인: 통과. kicad-cli가 내보낼 때마다 같은 항목을 다른 순서로 써서 결정성 검사가 실패하던 원인을 생성기에서 고친 것이며, 결정성 검사 자체는 바꾸지 않았습니다.
- **배치·배선을 다시 했습니다.** 새 부품 13개를 넣느라 주변 수동 부품 일부가 옮겨졌고, 배선은 새로 했습니다 (최종 보드 파일 기준 세그먼트 2665개, 원호 0개, 비아 452개: 0.6/0.3 mm 449개, 0.8/0.4 mm 3개). 배선은 Freerouting 실행 기록(세션 파일)을 다시 넣는 방식으로 재현됩니다.
- 부품마다 바뀐 내용과 이유 분류(태그)의 전체 표: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 10장 (교체 11, 추가 15, 삭제 1, 그대로 (부품 없음) 10, 그대로 140).

## 4. 누가 무엇을 붙이나

| 구분 | 뜻 | 개수 |
|---|---|---|
| jlc | JLC가 실장 (주문 BOM / CPL에 들어감) | 145 |
| jlc_optional | 기본은 사용자 납땜, with_mcu 파일 쌍을 쓰면 JLC 실장 | 4 |
| user | 사용자가 납땜 | 14 |
| none | 부품 없음 (빈 도금 구멍 / 미실장 / 구리 패턴만) | 13 |

- JLC가 붙이는 것: SMT 부품만. 기본 파일 쌍 기준 BOM 33줄 (SMT 33줄, 스루홀 0줄), 부품 145개. MCU까지 맡기는 with_mcu 쌍은 34줄 / 149개 (U1~U4 = C6510).
- 사용자가 붙이는 것: ATmega128A-AU 4개 (기본, with_mcu 쌍이면 JLC), ESP-12F 1개 (U7, 패드 22개), 스루홀 커넥터 12개 (J2, J5, J8~J12, J14~J18, 납땜 39곳) - ESP-12F와 커넥터는 두 파일 쌍 모두에서 사용자 몫. 그리고 GY-521 (J1 구멍에 모듈 핀을 납땜), BMP280 (직접 단 소켓 J2에 꽂음), OpenLog (모듈에 직각 6핀 헤더를 납땜한 뒤 직접 단 소켓 J5에 꽂음), NEO-7M (케이블로 J7).
- 부품 없이 비워 둔 자리: 프로그래밍 단자 J3, J19~J22 (2x3), J24~J26 (1x2) - 현재 선택 A안 (빈 도금 구멍). JLC가 헤더를 띠 밖에 실장하는 B안은 배치 검색 결과 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) (SMD 0곳, THT 0곳).
- rev 3.1의 J13 (KiCad (127.05, 66.85) mm, J12(ESC) 옆 +x / +y 모서리): 부품이 없는 도금 구멍 패드입니다. 로켓에 넣는 보드에서만 사용자가 서보 Y자 리드의 신호선을 납땜하고, VTOL에서는 비워 둡니다. R22는 JLC가 실장합니다.
- MCU·센서 말고 사용자가 해야 하는 일(ESP-12F와 스루홀 커넥터 납땜, 다리 자르기, 프로그래밍 단자 등)은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 8장에, 납땜 순서·커넥터 방향·받은 뒤 전원 시험은 7장에, 따로 살 부품은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/따로_살_부품_ko.md` 에 모았습니다.

## 5. 확인되지 않은 것 (NOT_VERIFIED)

- 재설계한 승압부의 전기적 효과: 검사는 모두 통과했지만 1장의 '목적과 다른 점'(C41 GND가 가는 선으로만 윗면에서 이어짐, C57 GND가 GND 면으로만 이어짐, SW 간격 여유 0.124 mm)이 실제로 어떤 영향을 주는지 - 스위칭 과전압, 입력 리플, SW 선 발열, 크리스털 잡음 - 는 보드에서 오실로스코프로 재기 전까지 NOT_VERIFIED입니다.
- 배터리 경로 60 C: 구리 저항이 0.03583 Ohm (E)로 0.035 Ohm 목표를 넘습니다. 검사는 계획대로 20 C 값(0.03096 Ohm)으로 판정합니다. 뜨거운 곳에서 오래 쓰면 저전압 차단 여유가 줄 수 있습니다 (계획 17장, 첫 보드에서 저항 측정: 주문 안내서 7장).
- 새 외곽의 JLC 수락과 값: 탭이 있는 거버는 아직 JLC에 올려 본 적이 없습니다. 주문 화면이 읽을 크기(외접 사각형으로 읽을 것으로 봄, E), PCB 값, DFM 결과는 NOT_VERIFIED입니다. 2026-11-12에 저장한 견적은 모두 재설계 전 76 x 70 mm 거버의 것입니다 (주문 안내서 4장).
- Standard 대안의 레일: y 135 쪽 긴 변에 탭이 있어 그 변은 곧은 선이 아니므로 V-cut 레일은 쓸 수 없고, 라우팅 탭 / 마우스바이트 레일이 필요합니다. JLC가 그렇게 해 주는지와 값은 NOT_VERIFIED입니다 (Economic 주문은 레일이 없어 해당 없음).
- 기체 CAD는 아직 탭이 없는 76 x 70 x 15 mm 상자로 캐리어를 그립니다 (이 단계는 CAD 저장소를 읽기만 했음). 남은 일: params.json의 캐리어 항목에 탭 추가, cad/layout.py의 캐리어 검사(보어 거리, 통풍구, 케이블타이 길, 폼테이프 면적, 조인트 나사 여유)를 사각형 + 탭으로 바꾸기, C25 등 CAD 검사와 내보내기 다시 실행, 캐리어 무게에 탭 몫 약 0.31 g 더하기, 76 x 70 을 적은 기체 문서·그림 고치기. 이 PCB 쪽 검사(board_fit)는 Fit 단계가 CAD에서 잰 값으로 탭을 확인했습니다 (PASS).
- 흡입구/아비오닉스 연결부 315도 조인트 나사의 길이: params.json에는 'M3'만 있습니다. 부품 금지 영역은 나사 끝이 반지름 49.5 mm까지 들어온다고 보고 잡았습니다 (A). M3 x 6 이하를 권합니다. 나사 길이는 NOT_VERIFIED입니다.
- 모듈 끼움, 보수적 경우: OpenLog 아랫면 부품을 1.2 mm로 잡으면 꽂는 길에서 C3 -0.05 mm (J5_openlog 'insertion stroke beyond the plugged PCB end'), U6 -0.5 mm (J5_openlog 'insertion stroke beyond the plugged PCB end') 이 겹치고, C48, Q2, Q8, U1, U2, Y1 은 0.5 mm 보다 가깝습니다 (A). OpenLog 아랫면 높이를 재고, 막을 수 있는 부품을 납땜하기 전에 실제로 꽂아 보십시오.
- 보드 동작 전체: 만들거나 전원을 넣은 적이 없습니다. 전원부 수치(5 V 출력, 차단 전압, 전류 여유)는 데이터시트 계산(E)입니다.
- JLC의 수락: Economic 주문에서 JLC가 기판 안에 툴링 홀 자리를 찾는지 (부품 자리 점유율 86.9 %; 못 찾으면 레일 = Standard를 요구할 수 있음), 가장자리 가까운 부품 (JLC 실장 중 가장 가까운 것 R60 0.537 mm; 0.3 mm 안: 없음), 지금 파일의 BOM / CPL 매칭은 업로드 후 화면과 DFM 결과, 견적으로만 확인됩니다.
- 주문 화면에서 본 것 (2026-11-12 01:30-02:06 KST, M): ESP·스루홀 변경과 재설계 전 빌드의 파일(76 x 70 mm 거버)을 올려 부품 40줄을 인식하고 40줄을 확정했고, 화면에서 ESP-12F와 스루홀 줄을 뺀 Economic 견적은 배송 제외 76.82 USD였습니다 (Economic이 ESP-12F를 뺀 39줄을 모두 받음). 지금 파일(처음부터 SMT만 든 BOM)로는 아직 올려 보지 않았습니다.
- 사용자가 사는 스루홀 커넥터 6종과 ESP-12F: 낱개로 산 부품이 구멍·패드에 맞는지, 그 재고와 가격 (LCSC 기록은 2026-11-12T02:50:37+09:00 ~ 02:51:09+09:00 기준 M, 그 밖의 쇼핑몰은 확인 안 함), 무연 HASL 구멍의 납땜성은 해 보기 전에는 NOT_VERIFIED입니다. 배터리 커넥터 J14는 방향이 틀리면 극성이 뒤집힙니다 (주문 안내서 7장 3번).
- CPL 회전: 방향이 중요한 30개 중 C45, C47, C52, U9 은 자료끼리 어긋나 NOT_VERIFIED입니다. 나머지도 JLC 미리보기로만 확정됩니다. 탄탈 C45 / C52, 전해 C47을 거꾸로 붙이면 화재 위험이 있습니다.
- 리플로 온도: 이번 주문 방식 Economic은 최고 255±5 C 입니다. 정격이 그 최악값과 같아 여유가 없는 부품: C2833309 (C47, 정격 260 C), C2286 (D1/D2/D3/D4, 정격 260 C), C53426685 (L5, 정격 260 C) (C47은 리플로는 한 번만 (두 번은 사이에 30분을 둘 때만)); 문서에서 정격을 찾지 못한 부품 번호 31개.
- ESP-12F 손납땜: 모듈과 무연 HASL 패드의 납땜성, 패드 바로 옆 부품과의 간격에서 작업이 되는지는 해 보기 전에는 NOT_VERIFIED입니다.
- 뒷면 다리 길이 (모든 스루홀을 사용자가 납땜하고 자름; 도면값은 참고 부품 번호 기준 F): J15, J17 은 규칙 1.5 mm 보다 길어 잘라야 하고, J2, J5, J8~J12, J16, J18 는 공차 끝에서 넘습니다. J1, J14 은 확인 못 함.
- 모듈 치수와 끼움: GY-521, BMP280, OpenLog, NEO-7M 치수는 가정값입니다 (GY-521 받침 2.54 mm, BMP280 헤더 직각 - 모두 A). 그림과 간섭 검사도 이 가정 위에 있습니다. 실물을 버니어로 재야 확정됩니다 (D15).
- J14 Molex 커넥터: 라이브러리 3D 모델이 없고 구멍 지름을 도면과 대조하지 못했습니다.
- 재고: 2026-11-13T20:26:50+09:00 기준입니다. 주문 직전에 scripts/jlc_gate.py 를 다시 돌려야 합니다.
- 비용: 주문 화면의 견적 76.82 USD (M, 배송 제외)는 재설계 전 76 x 70 mm 거버의 이전 업로드에서 줄을 빼고 읽은 값입니다. 지금 파일(외접 76 x 77.2 mm)의 견적, 메모 검토 뒤 금액, 배송비와 세금은 NOT_VERIFIED입니다.
- GPS 케이블이 기체 안에서 지나갈 길은 설계되지 않았습니다 (그림은 예시).
- rev 3.1 분리 서보 출력: 분리 서보, Y자 리드(서보 Y자 리드선 3핀 2.54 mm (JR / 후타바형) 1개 (로켓에 넣는 보드용)), J13에 납땜한 신호선은 만들거나 움직여 본 적이 없습니다. 분리 서보가 J12의 SERVO_5V(ESC의 BEC 출력)에서 전원을 받을 때 BEC의 전류 여유, 긴 신호선과 R22 220 Ohm으로 서보가 제대로 반응하는지, J13 패드에 직접 납땜한 선의 기계적 강도(진동, 당김)는 NOT_VERIFIED입니다.
- rev 3.1 거버의 JLC 수락과 값: 새 거버(J13 / R22 추가, 배선 일부 변경)는 JLC에 올려 본 적이 없습니다. 견적, DFM 결과, R22가 21번 줄에 들어간 BOM / CPL 매칭은 NOT_VERIFIED입니다. 장바구니의 Y24는 rev 3 거버이므로 결제하면 안 됩니다.
- 로켓 안 분리 서보 선의 길: J13은 +x / +y 모서리라 선이 분리 서보와 같은 +x 쪽으로 나가 전자 트레이 판을 건너지 않는다는 것은 상자 모델로만 본 것입니다 (A). 탑재부 칸막이와 COM 보드 돌기를 지나는 길, 45도 조인트 나사 몸통과의 간격은 모델에 없어 NOT_VERIFIED입니다.
- B안(서보 열 직각 헤더)을 배터리 선을 다시 그려서 넣을 수 있는지: 추정(E)만 있고 배치·배선·검사해 보지 않았습니다 (NOT_VERIFIED).

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

- 한 번 만드는 데 걸린 시간: 11.8분, 10.2분 (determinism.py의 두 빌드, M). 마지막 결정성 검사: 2026-11-13T21:24:30+0900.
- make_all.py는 인터넷을 쓰지 않습니다. JLC 부품 근거는 저장된 기록으로 다시 평가하고, 재고 확인(jlc_gate.py)은 따로 실행합니다.
- 배선은 기록된 Freerouting 세션을 다시 넣어 같은 보드를 만듭니다. 배치가 바뀌면 route.py가 재생을 거부하고, 새 배선(`make_all.py --reroute`)이 필요합니다. 새 배선은 실행마다 결과가 달라질 수 있습니다. 2026-11-12 재설계는 배치가 바뀌어 새 배선(Freerouting 실시간 실행)을 했고, 그 실행의 세션을 기록해 두었습니다. route.py는 배선 전 보드가 세션을 기록한 보드와 바이트까지 같을 때(표면처리 글자 한 곳만 다른 경우 포함)에만 재생을 받아들입니다 (이번 재생의 판정: 'byte-identical to the recorded unrouted board', M).
- 결정성 검사 결과: PASS - 바이트 비교 파일 71개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 0개. 비교 대상은 회로도, 보드, BOM, CPL, 거버와 zip, 회전 표, 3D 모델, 사양, 이 문서와 주문 안내서이고, STEP 파일은 머리줄의 시각을 뺀 뒤 비교합니다 (계획 14장). PNG 그림, 보고서(실행 시각 포함), 회로도 PDF는 증거 자료이며 결정적 산출물이 아닙니다. STEP 파일(기구 확인용)은 주문 파일이 아닙니다.
- 다른 폴더에서 다시 만들기 (`determinism.py --cross-folder <빈 폴더>`): PASS - 다른 폴더에 복사해 다시 만든 결정적 파일 60개 중 55개가 이 폴더의 것과 바이트까지 같음 (보드, 회로도, 거버와 zip, BOM, CPL 등 주문에 쓰는 파일 43개는 모두 여기에 듦). 나머지 5개(README_ko.md, out/ORDER_ko.md, out/따로_살_부품_ko.md, spec/SPEC_ko.md, spec/parts_sources.json)는 자기 폴더 이름이나 그런 파일의 sha256을 담고 있어, 폴더 이름과 그 sha256을 바꿔 넣은 뒤에 같음. 그림 단계 산출물 11개는 그 폴더에서 다시 만들지 않아 비교하지 않음. 재배치 검토(RR-2)에서 같은 입력을 다른 폴더에서 재생하면 구리는 같아도 배선의 uuid가 달라 보드 파일 바이트가 달랐습니다 (M). 그래서 route.py가 배선·비아·실크 글자의 uuid를 그 항목의 내용(네트, 층, 폭, 좌표)에서 만들도록 바꾸었고, 겹쳐 놓인 똑같은 배선 조각도 하나만 남깁니다. 주의: 사양 파일 `spec/carrier_spec.json` 은 자기가 만들어진 폴더 이름을 담고 있어서, 다른 폴더(예: 다른 체크아웃으로 합친 뒤)에서 다시 만들면 사양의 sha256이 달라지고 JLC 재고 기록(`out/jlc_gate_report.json`)은 '다른 사양'(STALE)으로 판정됩니다. 그때는 그 폴더에서 `jlc_gate.py` 를 다시 실행해야 합니다 (주문 당일에는 어차피 다시 실행합니다).
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
| rev 3.1 날짜 | 2026-11-13 | spec/rev3_plan.json amendments[rocket_release_output].date | F |
| rev 3.1 요청 원문 | "캐리어 PCB를 로켓에도 씀" / "B: 서보 열에 직각 헤더" | spec/rev3_plan.json amendments[rocket_release_output].request_verbatim | F (사용자 결정 그대로) |
| 분리 서보 MCU 핀 | U2.7 (PE5 / OC3C): NC → RELEASE_PWM | out/reports/rev3_mcu_pinmap.json declared_changes_by_plan_amendment | M |
| R22 BOM 줄 | 220R 1%, R17-R22 (6개), C22962 | out/jlcpcb_bom.csv (R22가 든 줄) | M |
| J13 자리 | KiCad (127.05, 66.85) mm, J12(ESC) 옆 +x / +y 모서리 | kicad/placement.json placements.J13 (x, y, why) | M |
| B안 결과 (rev 3.1) | rev 3 배터리 선을 바꾸지 않는다는 조건에서는 놓을 수 없음: -x 서보 열에 직각 1x3 J13을 J8~J11과 같은 8.62 mm 간격으로 넣고 J18을 1핀으로 줄이면, 배터리 선 두 개(윗면 2.0 mm VBAT_LOGIC, 아랫면 2.4 mm VBAT_LOGIC_SW) 사이에 45.06 mm가 필요한데 44.60 mm뿐입니다. 801곳을 찾아본 가장 좋은 자리도 열의 양 끝에서 0.13 mm 간격 규칙보다 0.23 mm 모자랐습니다. 배터리 선을 바꾸지 않는다는 조건은 사용자 결정도 검사 기준도 아니고 (검사는 배터리 경로 폭 2.0 mm 이상, 0.035 Ohm 이하만 요구), rev 3.1 작업이 rev 3 재설계 때 build_pcb.py에 깔아 둔 미리 그린 배선을 건드리지 않으려고 스스로 둔 것입니다 | spec/carrier_spec.json rev_3_1_changes.option_b_result | M (배치 검색과 kicad-cli DRC, 2026-11-13) |
| rev 3 장바구니 항목 | Y24 | 작업 지시 2026-11-13 (주 세션이 전한 장바구니 상태; 이 단계는 장바구니를 열지 않음) | F (지시) - 이 단계에서 확인 안 함 |
| 그 전 장바구니 항목 | Y23, Y20 | B:/0)AI_Cad/CanSat_v0.9/08_PCB/README.md (2026-11-12 재설계 납품 때 적은 장바구니 상태: 둘 다 옛 거버) | F (문서) - 장바구니를 열지 않아 어느 항목에 무엇이 들었는지 확인 안 함 |
| 변경 날짜 | 2026-11-12 | spec/rev3_plan.json amendments | F (사용자 결정) |
| 변경 요청 원문 1 | 아, esp도 내가 직접 실장할거야. 그럼 이코노믹으로 진행하고 pcb의 코팅도 무연으로 해줘 | spec/rev3_plan.json amendments[0].request_verbatim | F (사용자 요청 그대로) |
| 변경 요청 원문 2 | 이코노믹으로 스루홀 제외하고 어떤 부품을 따로 구매해야하는건지 리스트를 정리해줘 | spec/rev3_plan.json amendments[1].request_verbatim | F (사용자 요청 그대로) |
| ESP 패드 수 | 22 | out/fit_checks.json user_soldered_smd.U7 pads | M |
| 스루홀 부품 수 | 12 | out/fit_checks.json user_soldered_tht.summary.parts | M |
| 스루홀 목록 | J2, J5, J8~J12, J14~J18 | spec/carrier_spec.json parts[].fit == user (스루홀 커넥터) | M |
| 스루홀 납땜점 | 39 | out/fit_checks.json user_soldered_tht.summary.joints_per_board | M |
| main parts | 145 | out/manifest.json bom_cpl.pairs.main | M |
| main bom_lines | 33 | out/manifest.json bom_cpl.pairs.main | M |
| 보드 표면처리 | HAL lead-free | kicad/vtol_carrier.kicad_pcb stackup copper_finish | M |
| 아랫면 나란한 라벨 | J14 '1S BAT -+', J7 'GPS G RX TX V', J16 'ARM SW GA', J2 'BMP280 3V 3V SDA SCL G 3V' | out/reports/routing_rules.json silkscreen.pin_order_labels.labels (relation = along the pin row, layer B.Silkscreen) | M |
| 라벨 검사 | PASS | out/reports/routing_rules.json silkscreen.pin_order_labels.ok | M |
| 재설계 요청 원문 | "그럼 다시 설계를 진행해줘. 실패한게 있다면 완료되었다고 말하지마" / "보드의 사이즈를 조정하는 방법은 없어?" / "옮겨도 됨" | 사용자 요청 2026-11-12 (재설계 작업 지시에 옮겨 적힌 원문) | F (사용자 요청 그대로) |
| 재설계 전 보드 sha256 | cc750ce9e23322b2... | out/reports/routing_rules_before_redesign2.json board.sha256 | M |
| 재설계 전 배선 규칙 | 16 / 21 PASS, FAIL: battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance | out/reports/routing_rules_before_redesign2.json results (재설계 전 보드) | M |
| 외접 크기 | 76 x 77.2 mm | out/reports/board_metrics.json board_fit.outline.bbox_kicad_mm | M |
| 옮긴 프로그래밍 구멍 | J19 (x +0, y +0.55 mm), J22 (x +0, y +0.55 mm), J26 (x +0, y +0.55 mm) | kicad/placement.json placement_checks.rev3.rev2c.fixed_parts_at_user_decided_spot | M |
| C42 이동 | x +0.20 mm (지금 x 72.8, y 69.3 mm) | kicad/placement.json placements.C42 (x, y, why) | M |
| 출고 판정 | ORDERABLE (tool checks only; NOT_VERIFIED - never built) | out/manifest.json release_gate | M (도구 검사) |
| 출고 판정 이유 | 없음 | out/manifest.json release_gate.reasons | M |
| 결정성 | PASS - 바이트 비교 파일 71개 중 다른 것 0개; 시각 줄을 뺀 비교 3개 중 다른 것 0개 | out/reports/determinism.json | M |
| 다른 폴더 결정성 | PASS - 다른 폴더에 복사해 다시 만든 결정적 파일 60개 중 55개가 이 폴더의 것과 바이트까지 같음 (보드, 회로도, 거버와 zip, BOM, CPL 등 주문에 쓰는 파일 43개는 모두 여기에 듦). 나머지 5개(README_ko.md, out/ORDER_ko.md, out/따로_살_부품_ko.md, spec/SPEC_ko.md, spec/parts_sources.json)는 자기 폴더 이름이나 그런 파일의 sha256을 담고 있어, 폴더 이름과 그 sha256을 바꿔 넣은 뒤에 같음. 그림 단계 산출물 11개는 그 폴더에서 다시 만들지 않아 비교하지 않음 | out/reports/determinism.json cross_folder | M |
| STEP 번호 정렬 | kicad-cli가 쓴 STEP 항목 131461개의 번호와 순서를 내용으로 다시 매김 (scripts/step_canon.py); 다시 읽어 모든 항목이 같은 그래프인지 확인: 통과 | out/manifest.json step_docs.step.canonical_numbering | M |
| 배선 규칙 PASS 수 | 21 | out/reports/routing_rules.json | M |
| 배선 규칙 수 | 21 | out/reports/routing_rules.json | M |
| 배선 규칙 FAIL | 없음 | out/reports/routing_rules.json | M |
| 배터리 경로 저항 20C | 0.03096 Ohm | out/reports/routing_rules.json battery_path | E |
| 배터리 경로 목표 | 0.035 Ohm | out/reports/routing_rules.json battery_path | F (계획 13절) |
| 배터리 경로 최소 폭 | 2 mm | out/reports/routing_rules.json battery_path.legs | M |
| 2 mm보다 좁은 길이 | 0 mm | out/reports/routing_rules.json battery_path | M |
| 배터리 경로 구간 | J14.1 → Q10.3 0.00191 Ohm, 7.78 mm, 가장 좁은 곳 2 mm, 비아 0개; Q10.2 → J15.1 0.00388 Ohm, 15.77 mm, 가장 좁은 곳 2 mm, 비아 0개; J15.2 → L5.1 0.02517 Ohm, 132.22 mm, 가장 좁은 곳 2 mm, 비아 2개 | out/reports/routing_rules.json battery_path.legs | E (저항), M (길이·폭) |
| 배터리 경로 저항 60C | 0.03583 Ohm | out/reports/routing_rules.json battery_path | E |
| 재설계 전 배터리 경로 | 20 C 0.03173 Ohm, 60 C 0.03672 Ohm, 2.0 mm보다 좁은 길이 8.57 mm | out/reports/routing_rules_before_redesign2.json battery_path (재설계 전 보드) | E (저항), M (좁은 길이) |
| 검사 hot_loop | PASS | out/reports/routing_rules.json results | M |
| 출력 커패시터 간격 | C57 +5V 패드-U8.6 0.75 mm, GND 패드-U8.4 1.819 mm; C43 +5V 패드-U8.6 0.868 mm, GND 패드-U8.4 1.827 mm | out/reports/routing_rules.json hot_loop | M |
| 재설계 전 출력 커패시터 간격 | C57 0.95 / 1.611 mm; C43 3.25 / 3.537 mm | out/reports/routing_rules_before_redesign2.json hot_loop (재설계 전 보드) | M |
| 출력 커패시터 GND 윗면 연결 | C57.2, C43.2 | out/reports/routing_rules.json hot_loop | M |
| 검사 input_cap_l5 | PASS | out/reports/routing_rules.json results | M |
| 검사 input_cap_vin | PASS | out/reports/routing_rules.json results | M |
| 입력 커패시터 L5 경로 | C41 → L5 입력 패드 구리 길이 0.9 mm (기준 5 mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E; U8 VIN 패드와 L5 입력 패드 사이 직선 거리 2.103 mm) | out/reports/routing_rules.json input_cap_l5 | M (길이는 배선 모델, E) |
| 입력 커패시터 VIN 경로 | C41 → U8 VIN 구리 길이 4.17 mm (비아 0개; 기준 5 mm 이하 - 계획에 숫자가 없어 스크립트가 '몇 mm'를 이렇게 읽은 값, E), GND 패드가 U8 GND와 같은 윗면 구리: 예 | out/reports/routing_rules.json input_cap_vin | M (길이는 배선 모델, E) |
| 재설계 전 C41-L5 경로 | 13.53 mm | out/reports/routing_rules_before_redesign2.json input_cap_l5 (재설계 전 보드) | M (배선 모델, E) |
| 윗면 GND 한 덩어리 | PASS (U8.4, C57.2, C43.2, C41.2 이 같은 윗면 GND 구리 한 덩어리 위에 있음; 자기 비아가 있는 패드 4 / 4) | out/reports/routing_rules.json hot_loop_top_gnd | M |
| 검사 sw_node | PASS | out/reports/routing_rules.json results | M |
| SW 노드 | U8의 SW 패드와 L5의 SW 패드 사이 간격 1.876 mm (기준 2 mm 이하 - 계획 12장의 'L5는 U8 핀 바로 옆'에 숫자가 없어 스크립트가 계획 12장의 2 mm를 빌려 읽은 값, E), SW 선 구리 넓이 1.061 mm2, 구리 저항 약 4.3 mOhm (20 C, E), SW 선이 패드 사이를 지나는 다른 부품: 없음 | out/reports/routing_rules.json sw_node | M (기준과 저항은 E) |
| SW 선 | 길이 3.032 mm, 폭 0.35 mm, 비아 0개, F.Cu만 | out/reports/routing_rules.json boost_sw | M |
| 재설계 전 SW 패드 간격 | 5.551 mm | out/reports/routing_rules_before_redesign2.json sw_node (재설계 전 보드) | M |
| 검사 crystal_distance | PASS | out/reports/routing_rules.json results | M |
| 크리스털 거리 | rev 2c의 Pololu 핀은 NAV 크리스털 Y1 패드에서 9.217 mm, U1의 XTAL 핀에서 9.616 mm 떨어져 있었습니다. 지금은 가장 가까운 부품이 Y1 패드에서 9.55 mm (C43), XTAL 핀에서 9.935 mm (C43) 이고, SW 구리(선과 두 패드)는 Y1 패드에서 12.426 mm, XTAL 핀에서 12.969 mm, 크리스털 배선 구리에서 11.415 mm (rev 2c Pololu 핀은 크리스털 배선에서 8.243 mm) 입니다. rev 2c보다 가까운 부품: 없음 | out/reports/routing_rules.json crystal_distance (rev 2c 값은 archive/rev2c_pololu_1s/kicad/vtol_carrier.kicad_pcb 에서 같은 방법으로 잼) | M |
| 크리스털 거리 부품별 | C43 Y1 9.55 / XTAL 핀 9.935 mm; C57 Y1 11.851 / XTAL 핀 12.753 mm; U8 Y1 11.925 / XTAL 핀 12.319 mm; L5 Y1 14.65 / XTAL 핀 15.025 mm; C41 Y1 15.762 / XTAL 핀 15.485 mm (rev 2c 기준 Y1 9.217 / XTAL 핀 9.616 mm) | out/reports/routing_rules.json crystal_distance.rev3_parts_min_gap_mm | M |
| 재설계 전 크리스털보다 가까운 부품 | C43, C57, L5, U8 | out/reports/routing_rules_before_redesign2.json crystal_distance (재설계 전 보드) | M |
| FB 선과 출력 커패시터 | PASS (출력 커패시터 패드 사이를 지나는 FB 구리 0곳, 가장 가까운 간격 0.65 mm, FB 선 길이 4.787 mm) | out/reports/routing_rules.json fb_clear_of_caps | M |
| 레일 변 구리 이격 | PASS: 레일이 붙는 두 긴 변(y 65 변, 그리고 y 135 변과 탭의 모든 변)에서 0.4 mm 이상 (기준 0.4 mm), 나머지 두 변 0.3005 mm (보드 규칙 0.3 mm) | out/reports/routing_rules.json rail_edge_copper | M |
| 기체 맞춤 판정 | PASS | out/reports/board_metrics.json board_fit.ok | M |
| 기체 맞춤 | 흡입구 보어 벽까지 4.44 mm (기준 2 mm 이상), 탭에서 270도 통풍구까지 13.05 mm (기준 2 mm 이상), 탭에서 모터 리드 연결부 영역까지 21.869 mm (본체 사각형 쪽은 재설계 전과 같음), 315도 조인트 나사에서 평면 2 mm 안의 부품: 없음, 나사 아래 기판 윗면 높이 여유 1.3 mm (기준 1 mm 이상), 탭 안 부품 C41, C57, C58, L5, R63, R64, U8 (높이 한계 13.4 mm 넘는 것: 없음) | out/reports/board_metrics.json board_fit | M (나사 길이는 A / NOT_VERIFIED) |
| C41 GND 윗면 길 | C41 GND 패드의 비아 (118.525, 137.2)에서 윗면 GND 선을 따라 13.923 mm (끝점 (127.1, 135.95), (118.525, 138.05); 가는 0.2 mm 선이 FB 분압 저항 두 줄 사이를 지나 C43 GND 패드를 거쳐 U8 GND 핀으로 감), C41 GND 패드-U8.4 직선 간격 8.033 mm | kicad/vtol_carrier.kicad_pcb GND F.Cu 선 조각 (scripts/docs_ko.py gnd_chain_mm), out/reports/routing_rules.json hot_loop_top_gnd | M |
| C41 GND 비아 거리 | 0.65 mm | out/reports/routing_rules.json hot_loop_top_gnd.gnd_vias_near_the_pads | M |
| 내층 귀환 거리 | C43.2 GND 비아 → U8.4 GND 비아 5.15 mm; C57.2 GND 비아 → U8.4 GND 비아 4.02 mm; C41.2 GND 비아 → U8.4 GND 비아 8.33 mm (비아 중심 사이 직선, In1 GND 면 위) | out/reports/routing_rules.json hot_loop_top_gnd.own_via_at (비아 위치 M) 사이 직선 거리 | E |
| C43 GND 선 | C43 GND 패드에서 U8 GND 핀까지 윗면 GND 선 폭 0.2 mm 0.358 mm, 폭 0.25 mm 3.1 mm (비아 연결선 제외; SOT-563 패드 두 줄 사이를 지나는 가는 선; 패드마다 0.3 mm GND 비아 하나씩) | kicad/vtol_carrier.kicad_pcb GND F.Cu 선 (C43.2 비아와 U8.4 비아가 이루는 상자 + 0.5 mm 안, scripts/docs_ko.py gnd_tracks_in_box) | M |
| L5 입력 비아 | 드릴 0.4 mm 2개 | out/reports/routing_rules.json information.L5.1_battery_vias (L5 입력 패드에서 4 mm 안의 VBAT_LOGIC_SW 비아) | M |
| L5 입력 비아 단면 | 드릴 0.4 mm 이상 비아 2개의 도금 구리 단면은 합쳐서 약 0.045 mm2 로, 폭 2.0 mm 1 oz 배선의 0.07 mm2 보다 작습니다 (도금 18 um 가정, E). | out/reports/routing_rules.json information.L5.1_battery_vias 에서 계산 (pi x 드릴 x 0.018 mm) | E |
| SW 간격 여유 | 0.124 mm | out/reports/routing_rules.json sw_node (기준 - 간격) | M |
| ERC | 오류 0, 경고 0 (시트 8장) | out/reports/erc.json (kicad-cli 10.0.6) | M |
| DRC | 위반 0, 경고 0, 미연결 0, 회로도 불일치 0 | out/reports/drc.json (kicad-cli 10.0.6) | M |
| 넷리스트=사양 | PASS (부품 176, 네트 107) | out/manifest.json netlist_vs_spec | M |
| 보드=IR | PASS (핀 510 / 510) | out/manifest.json board_vs_ir | M |
| MCU 핀맵 | PASS (핀 256, 차이 0, 계획 수정으로 선언한 변경: U2.7 NC → RELEASE_PWM) | out/reports/rev3_mcu_pinmap.json (result, pins_compared, differences, declared_changes_by_plan_amendment) | M |
| U8/U9 패드표 | PASS | out/reports/rev3_u8_u9_pads.json | M (표는 데이터시트에서 옮김, F) |
| XTAL1 최악 | 5.96 mm (한계 5.96 mm) | out/reports/routing_rules.json crystals | M |
| 규칙 파일 | PASS | out/reports/routing_rules.json rule_file | M |
| 구리 섬 | 없음 (모든 네트 한 덩어리) | out/manifest.json board_metrics | M |
| 패드에 닿는 비아 | 0 | out/manifest.json board_metrics | M |
| JLC 확인 결과 | PASS (33/33줄), with_mcu PASS | out/jlc_gate_report.json | M |
| JLC 확인 시각 | 2026-11-13T20:26:50+09:00 | out/jlc_gate_report.json | M (그 시각의 재고) |
| 주문 파일 검사 | PASS | out/reports/rev3_export_files.json | M |
| 주문 파일 재현 | PASS | out/reports/rev3_export_determinism.json | M |
| 가장자리 2.5 mm 안 몸체 | 33 | out/reports/edge_distance.json | M |
| 가장자리 2.5 mm 안 JLC 실장 | 13 | out/reports/edge_distance.json bodies_under_2p5_by_fit | M |
| 가장자리 2.5 mm 안 with_mcu | 14 | out/reports/edge_distance.json bodies_under_2p5_by_fit | M |
| 가장자리 2.5 mm 안 JLC 미실장 | 20개 (U4, J1, J2, J5, J8, J9, J10, J11, J12, J14, J15, J16, J17, J18, U7, J13, J19, JP1, Q3, R47) | out/reports/edge_distance.json bodies_under_2p5_by_fit | M |
| 가장자리 0.3 mm 안 몸체 | U7 0.15 mm, J15 0.29 mm | out/reports/edge_distance.json | M |
| JLC 실장 가장 가까운 몸체 | R60 0.537 mm | out/reports/edge_distance.json jlc_placed_bodies.default_order closest[0] | M |
| 모듈 간섭 | A / NOT_VERIFIED, 가정 치수(보통 경우)로 충돌 0건 | out/fit_checks.json module_keepouts | A |
| 모듈 최소 간격 | 0.5 mm | out/fit_checks.json module_keepouts.clearance_min_mm | F (계획) |
| 모듈 간섭 보통 0.5 mm 미만 | U1 | out/fit_checks.json module_keepouts.modules[].zones[].below_clearance_min | A |
| 모듈 간섭 보수적 충돌 | C3 -0.05 mm (J5_openlog 'insertion stroke beyond the plugged PCB end'), U6 -0.5 mm (J5_openlog 'insertion stroke beyond the plugged PCB end') | out/fit_checks.json module_keepouts.modules[].zones[].below_clearance_min_conservative (<= 0) | A |
| 모듈 간섭 보수적 0.5 mm 미만 | C48, Q2, Q8, U1, U2, Y1 | out/fit_checks.json module_keepouts.modules[].zones[].below_clearance_min_conservative | A |
| 외곽 | 76 x 70 mm | spec/carrier_spec.json | F (size) / A (corner radius) |
| 탭 위치 | KiCad x 117.3..132, y 135..142.2 mm (바깥 모서리 r 1 mm, 본체와 만나는 안쪽 모서리 r 1 mm, 기판 재료는 x 116.3..133) | out/reports/board_metrics.json board_fit.outline.tab_kicad_mm | M |
| 탭 넓이 | 105.8 mm2 | out/reports/board_metrics.json board_fit.outline.tab_area_mm2 | M |
| 기판 넓이 | 5425 mm2 | out/reports/board_metrics.json board_fit.outline.area_mm2 | M |
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
| 블록 IO | 39 / 30 | spec/carrier_spec.json parts[].block, parts[].fit | M |
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
| R22 자리 | KiCad (130.125, 67.15) mm | kicad/placement.json placements.R22 | M |
| RELEASE_PWM 배선 | 61.28 mm (B.Cu / F.Cu), 비아 6개 | kicad/vtol_carrier.kicad_pcb (그 네트의 segment / arc / via) | M |
| RELEASE_SIG 배선 | 4.38 mm (F.Cu), 비아 0개 | kicad/vtol_carrier.kicad_pcb (그 네트의 segment / arc / via) | M |
| ESP 안테나 구역 검사 (esp12f_ok) | false - 겉상자 검사의 거짓 경보: board_metrics.py는 선의 겉상자(bounding box)가 안테나 금지 상자 (74.63, 65.0, 98.87, 72.4)에 닿으면 적는데, 적힌 것(PCB_TRACK ESP_TXD0 at (74.18, 72.48))은 대각선 선이라 실제 구리는 상자 밖에 있습니다 (그 네트 구리에서 상자까지 가장 가까운 거리 0.274 mm, 잰 선·비아 1개). KiCad 규칙 영역 ESP12E_antenna_keepout (네 층 모두 선·비아·면 금지)을 넣은 DRC는 위반 0개입니다. 이 값은 출고 판정에 들어가지 않으며, 기준(board_metrics.py)은 바꾸지 않았습니다 (rev 3에서는 true) | out/reports/board_metrics.json esp12f; kicad/vtol_carrier.kicad_pcb (선분 거리); out/reports/drc.json | M |
| 고정 부품 비교 | 58개 중 rev 2c 자리에서 움직인 것: 없음 | kicad/placement.json placement_checks.rev3.rev2c (fixed_parts_compared, fixed_parts_moved) | M |
| Fit 허용 범위 | KiCad x 106..134, 바깥 변 y 143.74 mm 이하 (바깥 모서리 r 1 mm) | out/reports/board_metrics.json board_fit.outline.fit_envelope (redesign2 Fit 단계가 기체 CAD에서 잰 허용 범위) | M |
| 승압부 자리 | U8 (126.5, 135.45) 0도; L5 (125.15, 138.75) 0도; C43 (127.1, 133.3) 180도; C57 (129.2, 136.75) 270도; C41 (120, 138.75) 180도; R63 (123.85, 133.85) 270도; C58 (122.35, 133.85) 270도; R64 (120.85, 133.85) 90도 | kicad/placement.json placements (KiCad mm, 회전) | M |
| 배선 세그먼트 | 2665 | kicad/vtol_carrier.kicad_pcb (최종 보드 파일에서 셈) | M |
| 배선 원호 | 0 | kicad/vtol_carrier.kicad_pcb (최종 보드 파일에서 셈) | M |
| 배선 비아 | 452 | kicad/vtol_carrier.kicad_pcb (최종 보드 파일에서 셈) | M |
| 비아 크기별 | 0.6/0.3 mm 449개, 0.8/0.4 mm 3개 | kicad/vtol_carrier.kicad_pcb (지름/드릴) | M |
| 변경 행 | 교체 11, 추가 15, 삭제 1, 그대로 (부품 없음) 10, 그대로 140 | spec/rev3_plan.json changes | F (계획) |
| fit jlc | 145 | spec/carrier_spec.json | M |
| fit jlc_optional | 4 | spec/carrier_spec.json | M |
| fit user | 14 | spec/carrier_spec.json | M |
| fit none | 13 | spec/carrier_spec.json | M |
| main bom_lines_smt | 33 | out/manifest.json bom_cpl.pairs.main | M |
| main bom_lines_tht | 0 | out/manifest.json bom_cpl.pairs.main | M |
| with_mcu bom_lines | 34 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| with_mcu parts | 149 | out/manifest.json bom_cpl.pairs.with_mcu | M |
| U1-U4 with_mcu | C6510 | out/jlcpcb_bom_with_mcu.csv | M |
| 프로그래밍 헤더 선택 | A | spec/carrier_spec.json prog_headers.option | M |
| B안 결론 | 불가 (고정 부품만 남겨도 2x3 다섯 개 자리가 없음) | out/reports/programming_access.json | M |
| B안 2x3 자리 | SMD 0곳, THT 0곳 | out/reports/programming_access.json verdict_per_family | M |
| 탭 질량 추정 | 0.31 g | out/reports/board_metrics.json board_fit.outline.tab_area_mm2 x 1.6 mm x 1.85 g/cm3 (FR4 밀도 A) | E |
| 부품 자리 점유율 | 86.9 % | kicad/placement.json placement_checks.courtyard_occupancy_percent (윗면 부품 자리 면적 / 케이블타이 띠를 뺀 기판 면적) | M |
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
| 로켓 Y자 리드 | 서보 Y자 리드선 3핀 2.54 mm (JR / 후타바형) 1개 (로켓에 넣는 보드용) | scripts/purchase_list.py (g) 묶음 = spec/rev3_plan.json off_board_purchase | F (계획) |
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
| 빌드 시간 | 11.8분, 10.2분 | out/reports/determinism.json builds | M |
| 결정성 시각 | 2026-11-13T21:24:30+0900 | out/reports/determinism.json | M |
| 배선 재생 입력 보드 | byte-identical to the recorded unrouted board | kicad/work/route_reuse_log.json unrouted_board_guard | M |
