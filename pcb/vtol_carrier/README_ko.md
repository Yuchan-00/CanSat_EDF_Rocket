# VTOL 아비오닉스 캐리어 PCB rev 2c (ESP-12E + 1S 로직 배터리, 4층)

**NOT_VERIFIED: 이 보드는 만들거나 전원을 넣어 본 적이 없습니다.** 아래의 "통과"는 모두 도구(kicad-cli ERC/DRC, Freerouting, 스크립트) 결과일 뿐이고, 보드가 동작한다는 뜻이 아닙니다.

표시: M = 도구로 측정, E = 추정, A = 가정(구입한 부품으로 확인 필요), F = 파일·업체 페이지 근거.

rev 2c = rev 2b에 독립 검토 3건(전기, PCB·제조, 기계 맞춤)의 지적을 반영한 판입니다. 지적별 처리는 11장 표에 있습니다.

## 1. 결과 요약 (M, kicad-cli 10.0.6)

- 회로도 ERC: **오류 0, 경고 0** (시트 8장).
- PCB DRC (`--schematic-parity --severity-all --refill-zones`): **오류 0, 경고 0, 미연결 0, 회로도 불일치 0**.
- 넷마다 구리가 한 덩어리인지 스크립트로 셌습니다 (pcbnew 연결, 채움 포함): **100개 넷 모두 한 덩어리** (rev 2b에서 놓친 GND 섬 검사).
- 패드에 닿는 비아 0개, 몸체 밑 앞면 실크 글자 0개, 구멍 사이 최소 0.537 mm (규칙 0.5 mm).
- 넷리스트 = 스펙 (부품 162, 넷 100, 무연결 핀 185) PASS. 보드 패드 넷 475개 = IR PASS.
- 거버 12장 + 드릴 1장 형식 검사 PASS (4층). BOM·CPL 구매처 검사 PASS (구매처 없는 실장 부품 0).
- 출고 판정 스크립트(`out/manifest.json` `release_gate`): **"ORDERABLE (도구 검사만)"**. 이 판정은 도구 검사만 뜻합니다. 7장의 확인 항목(모듈 핀 순서, JLC 회전 미리보기, 벤치 절차)은 사람이 해야 합니다.
- 결정성 PASS: 두 번 만든 회로도·배치·IR·배선 전 보드가 바이트 단위로 같고, 기록된 세션을 다시 넣은 배선 보드는 형상이 같습니다 (uuid만 다름, `out/reports/determinism.json`).

## 2. 보드에 있는 것

- 크기 76 × 70 × 1.6 mm, 모서리 R1 (F: params `configs.vtol.electronics.carrier.dims_lwh`; R1은 A). 부착 구멍 없음: 3 mm 폼테이프 + 케이블타이. 케이블타이 띠(x −3..+3 mm)에는 실장 부품이 없습니다 (프로그래밍용 미실장 풋프린트만).
- 4층: F.Cu 신호 / In1.Cu GND 전면 / In2.Cu +5V 전면 / B.Cu 신호. JLCPCB 표준 4층 1.6 mm (JLC04161H-7628 계열, 외층 1 oz, 내층 0.5 oz - A, 주문 때 스택업 이름 확인).
- 부품 162개 (스펙): 실장 152, 미실장 10 (프로그래밍 헤더 8 + 스펙상 선택 부품 2).
- 칩: U1 NAV, U2 FCU, U3 COM, U4 GNS = ATmega128A-AU 16 MHz 5 V. 칩마다 16 MHz 크리스털 + **27 pF** 2개, VCC 100 nF 2개, AVCC 10 µH + 100 nF, AREF 100 nF, 리셋 10 kΩ + 100 nF, LED, ISP 2×3(미실장).
- 5번째 ATmega128A는 없습니다 (사용자 결정). **J13(사출 서보 헤더, 미실장)도 뺐습니다**: VTOL에는 사출 서보가 없고(params `configs.vtol`), 사용자가 "미실장인데 왜 만들어둔거야"라며 5번째 칩을 뺀 것과 같은 이유입니다. U2 PE5는 무연결입니다. 되돌리려면 `scripts/build_spec.py`의 J13 줄을 살리면 됩니다.
- U7 ESP-12E (Wi-Fi 중계, 직접 납땜). 안테나 끝이 +y 모서리에서 0.08 mm 안쪽 (M), 안테나 아래와 양옆 4 mm는 4개 층 모두 구리·비아·부품 금지 (안에 구리 0, M).
- J1 GY-521 (헤더에 눕혀 납땜), J2 BMP280 (직각 소켓), J5 OpenLog (직각 소켓), J6 Pololu S9V11F5S6CMA 5 V 승강압 모듈 (1×5 헤더에 눕혀 납땜, 트리머 위).
- 전원: **J14 1S 배터리(Molex SPOX 2핀, −x/+y 모서리)** → Q10 AO3401A 역극성 보호 → **J15 전원 스위치 리드 (JST-PH 2핀 세로형, J14 바로 아래)** → C41/C42 → J6 Pololu VIN → +5V (In2 전면) → U6 AMS1117-3.3 → +3V3.
- 서보 J8-J11 (−x 모서리), ESC J12, 아밍 J16, 부저 J17, 6S 감지 J18, GPS 리드 J7 (JST-GH), ESP 굽기 헤더 J3 (2×3, 케이블타이 띠 맨 위, 미실장).
- 접지: 로직 GND와 추진 GND_PWR는 NT1(J12 3번 핀 옆) 한 곳에서만 만납니다.

```
 1S Li-Po (J14) --Q10 역극성--J15 스위치--+--J6 Pololu 5V 승강압-- +5V (In2.Cu 전면) --U6 AMS1117-- +3V3
                                          |                        |                                 |
                                       R15/R16 -> FCU PF3          +-- U1 NAV  U2 FCU  U3 COM  U4 GNS  +-- U7 ESP-12E, J2 BMP280,
                                                                   |    (ATmega128A x4, 16 MHz)        |   GPS(JP1), 시프터 LV 쪽
 ESC BEC (J12 2번) -- SERVO_5V -- J8..J11 서보 (+ C47 220 uF)       +-- J1 GY-521, J5 OpenLog, 부저
 GND_PWR (J12 3번) == NT1 == GND (In1.Cu 전면)
 NAV --nlink SPI--> FCU --ilink UART--> COM --UART1 (Q4/Q11 BSS138)--> ESP-12E (Wi-Fi)
 NAV --I2C (Q1/Q2)--> GY-521, BMP280      NAV <--glink UART--> GNS --UART1 (Q6/Q7)--> J7 GPS (NEO-7M)
 FCU --PWM--> J8..J12 (220 Ω + 10 kΩ 풀다운)   FCU --UART0 (Q8)--> J5 OpenLog   FCU PF0..PF2 <-- J18 6S 분압기
```

## 3. 층 수와 배선 (사용자 규칙 "답이 안 보이면 4층")

- 2층은 rev 2b에서 약 65분 시험해 미연결 63개에서 멈췄습니다 (M, `kicad/work/two_layer_attempt/`). 그래서 4층입니다. 2층 시험은 이번에 다시 하지 않았습니다.
- 4층 규칙은 2층 규칙 그대로입니다 (신호 0.2 mm, 간격 0.13 mm, 비아 0.3/0.6 mm 관통만, 가장자리 0.3 mm, 구멍-구멍 0.5 mm). 보드 최소 선폭만 0.1 mm입니다 (JLC 4층 0.09 mm, A).
- 기판값: 4층은 2층보다 비쌉니다 (E, 가격 조회 안 함).
- rev 2c 배선 순서 (`scripts/route.py`): 팬아웃(GND·+5V 패드마다 비아) → Freerouting 단계 servo → power_high → power → signals → gnd → complete → 마무리 스크립트.
- 채택한 Freerouting 실행 "vM" (2026-11-05 11:51-12:12, 스크래치 폴더): 단계 뒤 KiCad 미연결 181 → 175 → 140 → 2 → 1 → 1 (M). rev 2b는 같은 방식으로 25개가 남았습니다.
- rev 2c에서 배치를 바꿔 11개 변형을 돌려 비교했습니다 (M, 모두 스크래치): 마무리 스크립트 뒤 미연결 1-4개였고, vM이 마무리 뒤 0이 된 유일한 것이라 채택했습니다. Freerouting은 같은 입력이면 같은 결과를 냈습니다 (vA = vB, vD = vE).
- **마무리 스크립트 `scripts/close_leftovers.py` (새로)**: Freerouting이 남긴 연결을 0.1 mm 격자 A*로 잇습니다. 신호 넷과 AVCC만 (GND·+5V·+3V3·서보·배터리 넷은 손대지 않음), 새 구리만 추가, 기존 것은 옮기지 않음, 간격 0.13 mm + 여유, 가장자리 0.3 mm, 안테나 금지 영역 밖, 비아는 0.6/0.3 mm이고 **패드 안이나 패드에 닿는 자리에는 두지 않음**. 결과는 kicad-cli DRC가 다시 판정합니다. 이번 보드에서 이은 것 (M, `kicad/work/route_reuse_log.json`):
  - ESP_RXD_SH: 0.2 mm, 0.8 mm, 비아 1개.
  - NT1 1번 패드(GND 쪽)가 F.Cu GND 채움 섬에 갇혀 있었습니다 (팬아웃 비아 자리 없음). 0.4 mm 선 10.3 mm(비아 2개)로 주 GND에 이었습니다. NT1을 지나는 전류는 로직 GND 기준 전류뿐이고, 서보·ESC 전류는 GND_PWR에 머뭅니다 (스펙 `grounding`).
- 그 밖의 마무리: 전원선 가늘어진 곳을 간격이 허락하는 만큼 다시 넓힘 (91구간, M), 끝선 제거, GND 스티칭 비아 158개 (패드에 닿는 자리 제외), 실크 라벨.
- Freerouting "rip-up" 단계(`route.py --reuse-ses --polish`)도 만들어 시험했지만 남은 연결을 줄이지 못했습니다 (M, 2 → 2). 채택 배선에는 쓰지 않았습니다.

## 4. 측정 (M, `out/reports/board_metrics.json`)

- 디커플링: 칩 4개 × VCC·AVCC 핀 → 100 nF 동박 1.45-1.46 mm. **MCU GND 핀 22/53/63마다 패드 링 안쪽에 잠근 GND 비아** (rev 2b의 GND 섬 원인 제거).
- FCU는 AREF 100 nF와 AVCC 10 µH도 고정 배치·고정 배선입니다: AREF 핀 62 → 콘덴서 2.63 mm, AVCC 콘덴서 → 인덕터 1.38 mm. (NAV·GNS·COM은 같은 자리가 J1·J22·다른 부품과 겹쳐 자동 배치 그대로입니다.)
- 크리스털: 4개 모두 F.Cu, 비아 0, 판 가장자리에서 8 mm 이상. XTAL2 핀 23 → 크리스털 2.91 mm (통과). **XTAL1 핀 24 → 크리스털 5.96 mm (직선 5.18 mm), XTAL1 넷 7.29 mm: 3 mm 규칙을 못 맞춥니다.** 이유 (M, 라이브러리 치수): 3225 크리스털의 XTAL 패드 1과 3은 대각선에 있어, 어느 방향으로 놓아도 한 패드는 TQFP 핀 끝에서 3.2 mm 이상 멉니다 (패드 중심 기준). 16 MHz에서 7 mm 선은 부하용량 약 1 pF 정도 (E)라 동작에는 문제가 적다고 봅니다 (E). 
- ESP-12E: 100 nF(C54)가 핀 8에서 **2.4 mm** (rev 2b 3.55 mm). 10 µF 4.1 mm, 100 µF 7.5 mm (직선).
- 전원 패드: J6 1번(+5V 출력)·2번(GND), J14 2번(GND)은 면에 꽉 붙음(solid). rev 2b는 J6 +5V가 0.5 mm 스포크 하나였습니다.
- 선폭 (길이별): SERVO_5V 1.5 mm 96.6 mm, 1.45 mm 4.3 mm, 1.3 mm 6.0 mm (rev 2b 최소 0.9). GND_PWR 1.5 mm 94.1 mm + NT1 쪽 0.5 mm 0.1 mm. VBAT_LOGIC_RAW·VBAT_LOGIC 1.0 mm. **VBAT_LOGIC_SW 1.0 mm 95.4 mm, 0.95 mm 9.7 mm, 0.8 mm 35.0 mm** (스펙 1.0 mm보다 좁은 곳이 남음). +3V3 0.5 mm 267 mm, 0.45 mm 14.9 mm, 0.3-0.4 mm 1.9 mm.
- 배터리 경로가 깁니다: J15(−x/+y)에서 Pololu(+x/−y)까지 VBAT_LOGIC_SW 140 mm. 저항 약 0.08 Ω, 1.34 A 순간에 약 0.11 V 강하 (E, 1 oz). Pololu 차단 전압은 이만큼 일찍 걸립니다 (E).
- 서보 레일 비아: C47로 가는 1.2/0.6 mm 비아 1개뿐입니다. 서보 직류 전류는 THT 패드 사이 B.Cu 선으로 흐르고, 이 비아는 콘덴서 리플만 나릅니다 (E).
- 신호선이 바로 옆 전면 위에 있는 비율: F.Cu 88.6 %, B.Cu 88.1 %. NLINK(SPI)은 F.Cu(In1 GND 위) 비율 29-53 %, 나머지는 B.Cu(In2 +5V 면 위). +5V 면은 칩마다 100 nF로 GND와 결합되어 있어 그대로 두었습니다 (리뷰 PF-m4, 고치지 않음).
- 플러그 맞춤 (CAD 보어·조인트 나사·모터 선 구역): J8-J12, J16, J18, J14, **J15**, J17, J7 모두 통과. 최소 보어 여유 3.26 mm (J12). 플러그 크기는 A.
- CAD 라운드용 상자 목록 (꽂힌 플러그, 모듈 외곽): `board_metrics.json`의 `cad_round_envelopes` (차량 좌표 x/y, 보드 윗면 기준 높이).
- 실크: 앞면 이름 5개 (J14 "1S BAT +-", J8 "V1 S+-", J16 "ARM SW AG", J17 "BUZ +-", U7 "ESP-12E") + 참조번호 10개. **나머지 이름 20개는 뒷면(B.SilkS, 거울 글자)**입니다. 앞면 빈자리가 모자랍니다 (부품 면적 약 88 %). 규칙: 어떤 글자도 다른 부품 몸체나 모듈 몸체 위에 두지 않음 (위반 0, M). 커넥터 이름에는 1번 핀(네모 패드)부터 핀 뜻이 붙어 있습니다 (예: "6S NO GND VI VT VB IS").

## 5. 다시 만드는 방법 (정확한 명령)

작업 폴더: `B:/0)AI_Cad/CanSat_EDF_Rocket/pcb/vtol_carrier/scripts`

```
B:/Claude/.venv/Scripts/python.exe -X utf8 fetch_parts_sources.py   # (온라인, 읽기만) 구매처 다시 확인 -> spec/parts_sources.json
B:/Claude/.venv/Scripts/python.exe -X utf8 build_spec.py            # 스펙 JSON (네트워크 없음)
B:/Claude/.venv/Scripts/python.exe -X utf8 make_all.py              # 회로도 -> 배치 -> 배선 전 보드 -> 세션 재생 + 마무리 -> 검사·출력
B:/Claude/.venv/Scripts/python.exe -X utf8 determinism.py           # 두 번 만들어 비교
```

- `make_all.py` 순서: `build_schematic.py` → `place.py` → `build_pcb.py` → KiCad python `route.py --reuse-ses` → `checks_exports.py`.
- Freerouting을 새로 돌리려면 `make_all.py --reroute` (같은 입력이면 같은 결과였지만, 입력이 바뀌면 결과가 크게 바뀝니다).
- 채택 실행의 단계 명령: `java -Dfile.encoding=UTF-8 -jar C:/Users/yc012/AppData/Local/Temp/claude/B--Claude/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/tools_dl/freerouting/freerouting-2.5.0.jar -de <stage>.dsn -do <stage>.ses -mp 40 -mt 5 --gui.enabled=false` (signals 외 단계는 `-mp 20`). 로그·DSN·실행 기록: `kicad/work/freerouting_4layer_rev2c/`.
- 세션 6개(`kicad/work/vtol_carrier_<단계>.ses`)와 순서(`kicad/work/ses_replay.json`)를 보관하고 `route.py --reuse-ses`가 같은 순서로 다시 넣습니다. rev 2b 세션과 로그는 `kicad/work/rev2b_sessions/`, `kicad/work/rev2b_freerouting_4layer/`에 남겨 두었습니다.

## 6. 스펙과 다르게 한 것 (측정 근거)

1. 프로그래밍 전용 헤더 8개(J3 ESP 굽기, ISP J19-J22, 리셋 고정 J24-J26)는 케이블타이 띠 안 미실장 풋프린트입니다. 스펙은 실장이지만 놓을 자리가 없습니다. 프로그래밍할 때 헤더를 눌러 꽂거나 납땜합니다 (7장 절차).
2. 모듈 밑에 낮은 부품: 높이 제한(GY-521·Pololu 2.0 mm, BMP280·OpenLog 1.1 mm, A) 아래 부품만. U6 AMS1117은 Pololu 밑입니다 (열, A).
3. XTAL1 3 mm 규칙 (4장, 기하 한계).
4. VBAT_LOGIC_SW 일부 0.8-0.95 mm, +3V3 일부 0.3-0.45 mm (4장).
5. 마무리 스크립트가 이은 선은 Freerouting 선보다 꺾임이 많습니다 (격자 45°). DRC는 통과입니다.
6. AMS1117 콘덴서: C45(22 µF 출력) 5.1 mm, C44(10 µF 입력) 7.0 mm (리뷰 PF-m2, 고치지 않음).

## 7. 주문·조립 전에 꼭 할 것

### 7.1 확인 (A)

- 모듈 핀 순서 (모두 A, 산 물건의 실크로 확인):
  - J1 GY-521: VCC, GND, SCL, SDA, XDA, XCL, AD0, INT.
  - J2 BMP280: VCC, GND, SCL, SDA, CSB, SDO (I2C 0x77). 직각 헤더 플라스틱이 부품 없는 면에 있어야 모듈이 바로 눕습니다.
  - J5 OpenLog: GRN, RXI, TXO, VCC, GND, BLK (직각 수 헤더 C55629820를 납땜).
  - J6 Pololu: VOUT, GND, VIN, EN, PG (pololu.com 확인, 리뷰). SEL은 비워 둠.
  - J7 GPS 리드: 1 VCC, 2 GPS TXD, 3 GPS RXD, 4 GND. JP1 기본 3.3 V.
  - J14 배터리: 1번 = +. 첫 연결 전에 계측기로 팩 극성 확인 (Q10이 역극성을 막지만 확인).
  - J15: 극성 없음 (스위치). 토글 스위치의 듀폰 끝을 잘라 JST-PH 하우징으로 다시 끼움.
  - J3 ESP 굽기: 1 IO0, 2 GND, 3 TXD0, 4 RXD0, 5 RST, 6 GND (2×3). **3.3 V 신호 전용**, 전원 핀 없음.
- JLC CPL 회전: 커뮤니티 표(`spec/jlc_cpl_rotations_db.csv`, github matthewlai/JLCKicadTools, 2026-11-05 읽음, A)로 SOT-23 −90°, SOT-223·탄탈 +180°, TQFP +270°를 더했습니다 (17행, `out/reports/jlcpcb_cpl_rotation_offsets.csv`). **JLC 미리보기에서 극성 부품(C45, C47, C52, D1-D4, D6, Q1-Q11, U1-U4, U6, U7, Y1-Y4, J7) 방향을 하나씩 확인하십시오.**
- 4층 스택업(JLC04161H-7628 계열)과 0.09 mm 최소 규격은 가져와 확인하지 않았습니다 (A). 주문할 때 `.kicad_pro`의 규칙(간격 0.127/0.13 mm)을 같이 보내십시오: `.kicad_pro` 없이 DRC를 돌리면 KiCad 기본 규칙이라 위반이 많이 나옵니다.
- 가장자리에 붙은 부품 때문에 JLC가 조립 레일을 요구할 수 있습니다 (A).

### 7.2 벤치 절차 (리뷰 지적, 문서로 반영)

1. **Pololu 저전압 차단 먼저 설정** (E-m3): 보드에 꽂기 전에 벤치 전원으로 차단 3.2 V를 맞추고(업체: EN이 VIN을 나눠 0.7 V 아래에서 꺼짐) 5.0 V 출력을 확인합니다. SEL 패드는 비워 둡니다 (1.1 V 넘으면 6 V 출력 → ATmega128A 최대 5.5 V 초과, A). 공장 기본값은 업체 페이지에 없습니다. 3.0 V 입력에서 ESP 송신 순간 5 V 처짐을 재 보십시오.
2. **퓨즈가 없습니다** (E-m4): 1S 팩과 보드 사이 전류 제한은 팩의 PCM(A)뿐입니다. 선이 합선되면 팩 전류가 그대로 흐릅니다. PTC 퓨즈 추가는 사용자 결정 (9장).
3. **6S 감지 역급전** (E-m7): 로직을 켠 다음 6S를 꽂고, 6S를 먼저 뺀 다음 로직을 끕니다 (로직이 꺼진 채 6S가 꽂히면 PF0 분압기로 약 0.4 mA가 꺼진 +5V로 흘러 들어감, E). J18에는 GND 핀이 없고 기준은 ESC 리드 GND(NT1)입니다.
4. **BUF_ISIGN 입력** (E-m7): 테더 키트가 정해질 때까지 0-5 V 논리 신호만 넣으십시오 (5.6 V를 넘으면 (V − 5.6 V)/10 kΩ가 PC3 클램프로, 25 V에서 약 2 mA, E).
5. **I2C 풀업** (E-m6): 산 GY-521·BMP280 모듈의 풀업을 재 보고, 10 kΩ 이하면 R43/R45를 빼는(DNP) 것을 검토. 400 kHz 상승 시간 확인.
6. **서보 전원** (E-m5): 서보 4개 전류가 모두 J12 2번 핀(ESC BEC 선 하나)으로 들어옵니다 (MG90S 정지 토크 때 약 0.7 A × 4, A). 벤치에서 서보 4개를 동시에 막아 BEC 리드 전압 강하를 재 보십시오.
7. **프로그래밍** (E-m8): ISP는 USBasp 전원 점퍼를 끄고 보드를 로직 배터리로 켠 상태에서. 칩별 헤더와 리셋 고정: FCU·GNS 굽기 = J24(NAV 리셋 고정), COM 굽기 = J25(FCU 리셋 고정), ESP 굽기 = J26(COM 리셋 고정) + J3 1-2 점퍼(IO0) 후 리셋. 헤더 J19-J22, J24-J26, J3는 미실장이라 눌러 꽂거나 납땜합니다. USBasp 2번 핀은 +5V에 이어져 있어 USBasp가 5 V를 내면 Pololu 출력 쪽으로 역급전됩니다 (A).
8. **THT 핀 자르기** (MF2-8): 보드 밑으로 나온 핀(J17 1.8 mm, J2·J5 1.7 mm)을 1.0 mm 이하로 잘라 폼테이프에 닿지 않게 합니다.
9. **조인트 나사** (MF2-5): 흡입구-아비오닉스 조인트 나사는 M3 × 6 권장 (보어 안쪽 돌출 약 3 mm, E). J14·J7 세로 플러그 때문에 5.5 mm 이상 들어가면 안 됩니다 (E). 135° 나사는 나일론 권장 (안테나 쪽).
10. 모듈 외곽 (MF2-7): OpenLog과 Pololu 사이 간격 약 0.2 mm (E). 산 모듈을 재고, OpenLog·BMP280·GY-521 자유단 밑에 2-3 mm 폼 패드를 넣으십시오.

## 8. 부품 구매처 (사용자 규칙: JLCPCB 또는 디바이스마트만)

- 2026-11-05 이번 단계에서 **모든 줄을 다시 온라인으로 확인**했습니다 (읽기만, 로그인·장바구니·구매 없음): LCSC 45줄, 디바이스마트 8줄 모두 found. 디바이스마트 상품 페이지 8개를 이번에는 직접 읽었습니다 (rev 2b 때는 3개를 검색 목록으로만 확인). 원본 응답: `out/reports/sourcing/`.
- 새로 확인한 부품: 27 pF 0402 C0G **LCSC C1557** (0402CG270J500NT, Extended, 재고 292,832), 100 µF 10 V 탄탈 B형 **LCSC C7196** (TAJB107M010RNJ, Extended, 재고 105,178).
- J15는 J17과 같은 JST-PH(C131337), J3는 ISP와 같은 2×3(C65114)이라 새 부품 줄이 없습니다. PH 하우징 2개, 크림프 4개로 늘었습니다.
- 디바이스마트 '재고' 숫자 칸은 뜻이 문서화되어 있지 않습니다. GY-521, BMP280, NEO-7M, OpenLog, 토글, ESP-12E 대안은 0으로 보였고 판매 상태는 'normal'입니다. 배터리 93, Pololu 100. 주문 직전에 `fetch_parts_sources.py`를 다시 돌리십시오.
- 파일: `out/jlcpcb_bom.csv` (SMT 25줄, 136개, LCSC Part # 열), `out/jlcpcb_cpl.csv` (136행), `out/jlcpcb_bom_tht_hand_solder.csv` (손납땜 THT 6줄), `out/devicemart_purchase_list.csv` (7줄), `out/lcsc_loose_items.csv`, `out/bom_sources_all_lines.csv` (출처·번호·URL·조회일·재고).

## 9. 사용자가 정할 것 / 후속 작업 (이번에 바꾸지 않음)

### 9.1 로직 배터리 (사용자 결정 2026-11-05 "배터리는 A 위치")

- 가정한 팩 위치 (CAD 라운드용, 차량 좌표, E): **x −48.4..−37.9, y −27.75..+27.75, z 346.1..413.1 mm**, 리드 끝이 +y. 보드에서는 J14·J15를 −x/+y 모서리로 옮겨 리드가 평면으로 약 6 mm만 내려옵니다 (M, 리뷰 계산). 서보 리드(−x 가장자리 y −12..+22에서 180° 구멍으로)와 ESC·모터 리드(+x)를 건너지 않습니다.
- A 자리는 배터리 구성에서만 맞습니다. 테더·하이브리드 구성에서는 버스 클램프(약 13,500 mm³)·XT90(약 190-940 mm³)과 겹칩니다 (E, MF2-2). 다른 구성의 자리는 사용자 결정입니다.
- A 자리 주변: 서보 리드 구멍과 평면 1.6 mm, GPS 리드(J7)와 0.37 mm, J18 플러그가 팩 아래 약 14 mm (E, MF2-10). CAD 라운드에서 2 mm 이상 여유 상자를 넣으십시오.
- 전원 스위치와 배터리 플러그는 기체를 닫으면 손이 닿지 않습니다 (MF2-3). J15 리드를 158° 창의 작은 토글로 빼는 방법 (리드 약 28 mm, E) 또는 팩 플러그를 창 높이에서 빼는 방법 중 선택 (CAD 변경). 그 전에는 보관할 때 J14를 뽑으십시오.
- 무게: 84 g 팩으로 VTOL 배터리 구성이 목표보다 약 88 g 무겁습니다 (E). params.json은 고치지 않았습니다.
- PTC 퓨즈(약 2 A 유지) 추가 여부 (E-m4) - 자리 확보가 필요합니다.

### 9.2 무선 (사용자 결정 2026-11-05 "가능하면 wifi, 아니면 lora")

- Wi-Fi(ESP8266) 채택. 기체 쪽은 납땜한 ESP-12E(PCB 안테나) 그대로입니다. 거리는 프로토콜과 지상국 쪽에서 늘립니다:
  - ESP-NOW (또는 802.11b 고정) 1 Mbps: 수신 감도 약 −98 dBm (802.11n 고속 약 −72 dBm), TX +20 dBm (A, Espressif ESP8266EX 데이터시트 - 디스크에 없음).
  - 지상국 = 두 번째 ESP8266 + 외부 안테나 모듈(ESP-07S 계열, u.FL) + 8-9 dBi 무지향 또는 12-14 dBi 지향성 2.4 GHz 안테나.
  - 링크 계산 (E): 20 dBm − 2 dBi(기체 안) + 8 dBi(지상) − 자유공간 손실 − 10 dB 여유 = 500 m 약 −78 dBm, 1 km 약 −84 dBm (여유 14 dB), 2 km 약 −90 dBm.
- **비행 조건: 비행 전에 예상 최대 경사 거리의 2배 이상(예: 1 km) 지상 거리 시험.** 실패하면 LoRa로 돌아갑니다 (PLM100 1차 빌드: `archive/rev1_plm100_2s/`).
- 한국 2.4 GHz EIRP 제한을 고이득 지상 안테나에 대해 확인해야 합니다 (A).
- 보드: 안테나는 +y 모서리, 금지 영역 유지, PLA 벽(2.8 mm)을 향함. 6S 팩 18.4 mm, 1S A 자리 21.8 mm, ESC 26.9 mm, 모터 선 49.5 mm (E, 리뷰 MF2-12). 안테나 −x 쪽 2-6 mm에 있던 J13은 없앴고, J8 서보 리드는 안테나 모서리에서 떨어지게 정리하십시오 (PF-m6).
- 로켓 구성(params `configs.rocket`, 펌웨어 lora 빌드)은 아직 LoRa입니다. 같은 변경이 필요한지는 사용자 결정 (읽기만 함).
- 후속(펌웨어): ESP 중계 펌웨어·ESP-NOW 프로토콜, COM 링크 드라이버, `sim/link_model.py`, 지상국.

### 9.3 그 밖의 후속

- 펌웨어: FCU PF3 로직 배터리 읽기, BMP280 드라이버와 기압 모델 (제어 관련 → 전/후 비교 영상 규칙), NEO-7M 설정, `sim/tether.py:171`, PE5(사출 서보 PWM)는 이제 보드에 이어진 것이 없음.
- params/문서 충돌 (고치지 않음): 5번째 칩, LoRa ↔ ESP-12E, 6S ↔ 1S 로직 전원, 사출 서보 헤더.
- 2026-11-11 갱신 (2026-11-10 제어 라운드): `spec/carrier_spec.json`의 `esp12e.uart` 설명을 9600에서 38400으로 고쳤습니다 (params `flight_computer.link.esp_bridge.uart_baud`. `link.uart_baud` 9600은 예비 LoRa 링크 것). 글만 고쳤고 `scripts/build_spec.py`의 같은 문장도 고쳤습니다. 넷리스트·배치·거버는 다시 만들지 않았습니다. 그래서 `kicad/placement.json`과 `kicad/schematic_manifest.json`의 `spec_sha256`은 고치기 전 사양 파일의 값입니다. 위 두 줄 중 BMP280 드라이버와 기압 모델은 만들었고(NAV 기본 빌드), params.json의 설명 글(5번째 칩, LoRa, 로직 전원)도 rev 2c에 맞췄습니다.
- 앞면 실크 공간: 이름 20개가 뒷면입니다. 하네스 끝에 라벨을 붙이는 것을 권합니다.

## 10. 파일

- KiCad 프로젝트: `B:/0)AI_Cad/CanSat_EDF_Rocket/pcb/vtol_carrier/kicad/vtol_carrier.kicad_pro`, `vtol_carrier.kicad_sch` (+ 시트 7장), `vtol_carrier.kicad_pcb` (배선된 4층 보드, 제목 블록 rev 2c).
- 스펙: `spec/carrier_spec.json` (rev 2c, `rev_2c_changes`, `battery_spot_A_assumed`, `radio_decision_2026_11_05`), `spec/SPEC_ko.md`, `spec/parts_sources.json`, `spec/jlc_cpl_rotations_db.csv`.
- 배치: `kicad/placement.json`. IR: `kicad/ir/`.
- 배선 기록: `kicad/work/ses_replay.json`, `kicad/work/vtol_carrier_*.ses`, `kicad/work/freerouting_4layer_rev2c/`, `kicad/work/route_reuse_log.json`.
- 출력: `out/vtol_carrier_gerber.zip` (+ `out/gerber/`), `out/vtol_carrier.net`, `out/schematic/` (PDF + SVG 8장), `out/vtol_carrier_bom.csv`, `out/vtol_carrier_cpl.csv`, JLC·디바이스마트 목록 (8장), `out/vtol_carrier.step` (미실장 제외), `out/render_top.png`, `render_bottom.png`, `render_iso.png`, `out/manifest.json` (sha256 + `release_gate`).
- 보고서: `out/reports/erc.json`, `drc.json`, `netlist_vs_spec.json`, `board_vs_ir.json`, `gerber_check.json`, `board_metrics.json`, `determinism.json`, `jlcpcb_cpl_rotation_offsets.csv`.
- 스크립트: `common.py`, `build_spec.py`, `fetch_parts_sources.py`, `build_schematic.py`, `place.py`, `build_pcb.py`, `fanout.py`, `route.py`, `close_leftovers.py` (새로), `silk.py`, `board_metrics.py`, `checks_exports.py`, `cad_envelope.py`, `determinism.py`, `make_all.py`.
- AI_EDA 엔진(`B:/Claude`)은 가져다 쓰기만 했고 고치지 않았습니다. params.json, CAD, 펌웨어, `archive/`도 고치지 않았습니다.

## 11. rev 2b 독립 검토 지적 처리표 (2026-11-05)

표시: 고침 = 스크립트/스펙을 고치고 다시 만들어 도구로 확인함, 일부 = 일부만 고침, 사용자 = 사용자 결정 필요(이번에 적용 안 함), 문서 = README·조립 절차에만 반영.

| 지적 | 내용 | 처리 |
|---|---|---|
| E-B1 / PF-B1 | 미연결 25개 | 고침: 미연결 0 (M). J3를 ESP 옆으로, Q10·R14를 새 모서리로, ESP 스트랩 저항을 ESP 쪽으로, FCU AREF·AVCC 고정 배치, 6S 분압기 윗저항을 J18 옆으로 + 마무리 배선 스크립트 |
| E-B2 | MCU GND 핀 3개 + 콘덴서가 GND 섬 | 고침: MCU GND 핀 22/53/63마다 패드 링 안쪽에 잠근 GND 비아. 모든 넷이 한 덩어리인지 스크립트로 검사(`net_clusters`) |
| E-m1 / PF-m1 | XTAL1 경로 5.1 mm (규칙 3 mm) | 못 고침 (기하 한계, 4장) |
| E-m1 / PF-m2 | ESP 100 nF 3.55 mm | 고침: 2.4 mm (M) |
| E-m2 | 22 pF ↔ CL 18 pF | 고침: 27 pF (LCSC C1557, 온라인 확인) |
| E-m3 | Pololu 차단 전압·SEL | 문서: 첫 연결 전 벤치 설정 절차 (7장) |
| E-m4 | 탄탈 6.3 V, 퓨즈 없음 | 탄탈 10 V로 고침 (LCSC C7196). 퓨즈는 사용자 결정 (팩 PCM에 의존, 9장) |
| E-m5 | 서보 전류가 J12 2번 핀 하나 | 사용자 결정 (별도 서보 전원 입력) + 벤치 측정 절차 |
| E-m6 | I2C 풀업 중복 | 문서: 모듈 풀업 측정 후 R43/R45 DNP 판단 |
| E-m7 | 6S 분압기 역급전, BUF_ISIGN 입력 | 문서 (7장) |
| E-m8 | 프로그래밍 헤더 미실장 | 일부: J3를 ESP 옆 띠로 옮김. 미실장 유지 (자리 없음) + 절차 문서화 |
| E-m9 | 디바이스마트 출처 약함 | 고침: 8개 상품 페이지를 직접 다시 읽음 (2026-11-05) |
| PF-M1 | J6 +5V가 스포크 1개 | 고침: J6.1·J6.2·J14.2 패드를 면에 꽉 붙임 (solid) |
| PF-M2 / MF2-9 | 앞면 실크 라벨 | 일부: 몸체 밑 라벨 0개(M). 앞면에 다 들어가지 않아 나머지는 뒷면 |
| PF-M3 | 스티칭 비아가 패드 안 | 고침: 패드에 닿는 비아 0개 (M) |
| PF-M4 / MF2-1 | J14·J15 위치 | 고침: −x/+y 모서리 (배터리 A 자리 옆) |
| PF-M5 | JLC CPL 회전 | 고침: 회전 보정표 적용 (A, JLC 미리보기 확인 필수) |
| PF-M6 | 전원선 가늘어짐 | 일부: 서보 최소 1.3 mm (rev 2b 0.9), VBAT_LOGIC_SW 0.8 mm 35 mm 남음 (4장) |
| PF-m3 | 0.1-0.12 mm 선 | 측정값 4장 |
| PF-m4 | NLINK이 B.Cu | 그대로 (In2 +5V 면은 100 nF로 GND와 결합), 측정값 4장 |
| PF-m5 | 끝선·실크 겹침 | 고침: 끝선 자동 제거, LED-크리스털 간격 규칙 |
| PF-m6 | 안테나 옆 J13 | 고침: J13 제거 |
| PF-m7 | J14 3D 모델 없음, 리비전 | 리비전 2c 기록. 3D 모델은 라이브러리에 없음 → CAD 상자 목록으로 대체 |
| MF2-2 | A 자리는 테더/하이브리드와 겹침 | 사용자 결정 |
| MF2-3 | 스위치·플러그 손 닿지 않음 | 사용자 결정 (CAD) |
| MF2-4 | J15 플러그 높이 | 고침: JST-PH 세로형 |
| MF2-5 | 조인트 나사 길이 | 문서: M3 × 6 권장 |
| MF2-6 | STEP·CAD 상자 | 고침: STEP에서 미실장 제외, CAD용 상자 목록(`board_metrics.json` `cad_round_envelopes`) |
| MF2-7 | 모듈 외곽 간격 | 문서: 산 모듈 측정 |
| MF2-8 | THT 핀 1.7-1.8 mm | 문서: 1.0 mm 이하로 자르기 |
| MF2-10 | A 자리 주변 리드 | CAD 라운드 (문서) |
| MF2-11 | README에 결정 없음 | 고침 (9장) |
