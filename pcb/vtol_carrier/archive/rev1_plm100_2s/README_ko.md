# VTOL 아비오닉스 캐리어 PCB (KiCad 10, kicad-cli 구현)

**NOT_VERIFIED: 이 기판은 만들지도, 전원을 넣지도 않았습니다. 동작한다고 말할 수 없습니다.**
아래의 "통과"는 모두 도구 검사(ERC, DRC, 스크립트 비교) 결과입니다. 회로가 맞게 동작한다는 증거가 아닙니다.

표시: M = 도구로 측정, E = 추정, A = 가정(구매한 모듈로 확인 필요), F = 파일에서 옮김.

## 1. 파일

| 무엇 | 경로 |
|---|---|
| 사양(입력, 이번에 바꾸지 않음) | `B:\0)AI_Cad\CanSat_EDF_Rocket\pcb\vtol_carrier\spec\carrier_spec.json`, `spec\SPEC_ko.md` |
| KiCad 프로젝트 | `B:\0)AI_Cad\CanSat_EDF_Rocket\pcb\vtol_carrier\kicad\vtol_carrier.kicad_pro` |
| 회로도(계층, 루트 + 8장) | `kicad\vtol_carrier.kicad_sch`, `power`, `nav`, `fcu`, `com`, `gns`, `spare_mcu`, `level_shift`, `io` `.kicad_sch` |
| 기판(배선 완료) | `B:\0)AI_Cad\CanSat_EDF_Rocket\pcb\vtol_carrier\kicad\vtol_carrier.kicad_pcb` |
| 배치 결과 | `kicad\placement.json` (좌표, 이유, 검사) |
| IR(설계 원본 데이터) | `kicad\ir\vtol_carrier.ir.json`, `kicad\ir\vtol_carrier.ir_with_pcb.json` |
| 라우팅 기록 | `kicad\work\route_log.json`, `route_log_run2_stage1.json`, `route_reuse_log.json`, `vtol_carrier.ses` |
| 출력 | `B:\0)AI_Cad\CanSat_EDF_Rocket\pcb\vtol_carrier\out\` |
| 거버 + 드릴 zip | `out\vtol_carrier_gerber.zip` (폴더 `out\gerber\`) |
| BOM / CPL | `out\vtol_carrier_bom.csv`, `out\vtol_carrier_cpl.csv` |
| STEP / 렌더 | `out\vtol_carrier.step`, `out\render_top.png`, `out\render_bottom.png`, `out\render_iso.png` |
| 회로도 PDF / SVG / 넷리스트 | `out\schematic\vtol_carrier_schematic.pdf`, `out\schematic\*.svg`, `out\vtol_carrier.net` |
| 검사 보고서 | `out\reports\erc.json`, `drc.json`, `netlist_vs_spec.json`, `board_vs_ir.json`, `gerber_check.json`, `determinism.json` |
| 모든 출력의 sha256 | `out\manifest.json` |
| 스크립트 | `scripts\common.py`, `build_schematic.py`, `place.py`, `build_pcb.py`, `route.py`, `checks_exports.py`, `determinism.py`, `make_all.py` |

## 2. 기판에 있는 것

- 76 × 70 × 1.6 mm, 2층, 모서리 R1 (F/A, spec board.outline_mm). 장착 구멍 없음(폼 테이프 + 케이블 타이, spec).
- ATmega128A-AU 4개: U1 NAV, U2 FCU, U3 COM, U4 GNS. U5는 5번째 칩 자리(미실장).
- 칩마다 16 MHz 크리스털, 22 pF, 디커플링, AVCC LC, 리셋, LED, ISP 자리.
- 센서·무선: GY-521(J1, 1×8 헤더), BMP280(J2, 직각 소켓), PLM100(J3·J4, 1.27 mm 소켓 두 줄), OpenLog(J5), NEO-7M 리드(J7, JST-GH 4핀).
- 전원: 로직 배터리 J14(JST-PH) → Q10 역전압 P-FET → J15 전원 단락핀 → J6 5 V 강압 모듈 → U6 AMS1117-3.3.
- BSS138 레벨 시프터 Q1–Q8(Q3은 선택, 미실장).
- 서보 J8–J11(J13 미실장), ESC J12, 아밍 J16, 부저 J17, 6S 전압 감지 J18.
- 부품 165개, 넷 107개(M, 넷리스트). 기본 실장 138개, 미실장 27개(사양 23개 + 아래 ISP 4개).

### 블록 그림(텍스트)

```
J14 2S 로직 배터리 ─ Q10 역전압 ─ J15 스위치 ─ J6 5V 강압 ─ +5V ─┬─ U1 NAV ─ I2C ─ Q1/Q2 ─ +3V3 ─┬─ J1 GY-521(5V)
                                                              │                               └─ J2 BMP280
                                                              ├─ U2 FCU ─ 서보 J8–J11, ESC J12, 아밍 J16, 부저 J17,
                                                              │           6S 분압기(J18), OpenLog J5(Q8)
                                                              ├─ U3 COM ─ Q4/Q5 ─ J3·J4 PLM100(3.3V)
                                                              ├─ U4 GNS ─ Q6/Q7 ─ J7 NEO-7M 리드
                                                              ├─ U5 (미실장)
                                                              └─ U6 AMS1117 ─ +3V3
칩 사이: NAV↔FCU nlink(SPI), FCU↔COM ilink(UART, 1k), NAV↔GNS glink(UART, 1k)
ESC BEC 5V(J12-2) ─ SERVO_5V ─ 서보 헤더 2번 핀 (로직 +5V와 분리)
GND(로직) ─ NT1(J12 3번 핀 옆, 유일한 연결점) ─ GND_PWR(ESC·서보)
```

## 3. 다시 만드는 방법(정확한 명령)

엔진 파이썬: `B:\Claude\.venv\Scripts\python.exe`. KiCad 파이썬: `C:\Users\yc012\AppData\Local\Programs\KiCad\10.0\bin\python.exe`.

```
cd "B:\0)AI_Cad\CanSat_EDF_Rocket\pcb\vtol_carrier\scripts"
B:\Claude\.venv\Scripts\python.exe -X utf8 make_all.py
```

`make_all.py`는 차례로 실행합니다.

1. `python -X utf8 build_schematic.py` → IR, 회로도 9장, `.kicad_pro`
2. `python -X utf8 place.py` → `kicad\placement.json` (약 70초)
3. `python -X utf8 build_pcb.py` → `kicad\work\vtol_carrier_unrouted.kicad_pcb`
4. `"C:\...\KiCad\10.0\bin\python.exe" route.py --reuse-ses` → 기록된 Freerouting 세션을 다시 가져와 `kicad\vtol_carrier.kicad_pcb`
5. `python -X utf8 checks_exports.py` → ERC, DRC, 넷 비교, 거버, BOM, CPL, STEP, 렌더, `out\manifest.json`

Freerouting을 새로 돌리려면 `make_all.py --reroute`(= `route.py 40 4`)입니다. 결과는 매번 다를 수 있습니다.
결정성 검사: `python -X utf8 determinism.py`.

## 4. 검사 결과

| 검사 | 결과 |
|---|---|
| ERC `kicad-cli sch erc --severity-all` | 오류 0, 경고 0 (M, KiCad 10.0.6, 시트 9장 모두) |
| 넷리스트 대 사양 | 넷 107/107 일치, 부품 165, 미연결 선언 핀 235 = 단일 핀 넷 235 (M, PASS) |
| DRC `kicad-cli pcb drc --schematic-parity --severity-all --refill-zones` | 오류 0, 미연결 0, 회로도 패리티 문제 0, 경고 6 (M) |
| 기판 패드 넷 대 IR | 핀 494/494 일치 (M, PASS) |
| 거버 형식 검사(엔진 `check_gerber_set`) | PASS: 동박 2층, 마스크·실크·페이스트·외곽, 양면 GND 채움 확인 (M) |
| 드릴 | PASS, 1 파일 (M) |
| 결정성 | 회로도·배치·배선 전 기판·IR: 두 번 실행해 sha256 같음. 배선된 기판: uuid를 뺀 형상이 같음 (M, PASS) |

DRC 경고 6개는 모두 `hole_to_hole`입니다. 서로 다른 넷의 비아 구멍 사이가 0.453–0.4985 mm입니다.
규칙은 0.5 mm로 정했습니다(A, JLCPCB 표준). 위치는 `out\reports\drc.json`에 있습니다.
JLCPCB DFM에서 걸리면 해당 비아를 옮겨야 합니다. **열린 문제입니다.**

라우팅(M, `kicad\work\route_log*.json`):
- 1단계: `java -jar freerouting-2.5.0.jar -de vtol_carrier.dsn -do vtol_carrier.ses -mp 40 -mt 4 --gui.enabled=false`. 403초, 23패스에서 멈춤, 미배선 1(LORA_GPIO5).
- 원인: R48 패드가 u.FL 아래 동박 금지 구역 안에 있었습니다. 구역을 6 → 5 mm로 줄였습니다(배치는 그대로).
- 2단계: 배선된 기판을 다시 내보내 완성 실행(`-mp 20`). 302초, 미배선 0.
- 정리: Freerouting이 패드 근처에서 0.10–0.12 mm로 좁힌 배선 7개를 최소 0.127 mm로 넓혔습니다. 한쪽 층에만 붙은 비아 1개(VANE4_PWM)를 지웠습니다.
- 결과: 배선 2,454개, 비아 209개(M).
- Freerouting은 결정적이지 않습니다. 최종 세션 `kicad\work\vtol_carrier.ses`(sha256 a9a6d809…)를 산출물로 둡니다.

## 5. 설계 규칙(`.kicad_pro`)

JLCPCB 2층 표준을 최소값으로 넣었습니다(A, 제조사 문서를 이번에 다시 확인하지 않음).

| 항목 | 값 |
|---|---|
| 최소 배선 폭 / 간격 | 0.127 / 0.127 mm |
| 최소 비아 | 0.6 / 0.3 mm, 링 0.13 mm |
| 구리–기판 가장자리 | 0.3 mm |
| 구멍–구멍 | 0.5 mm |
| Default 넷 클래스 | 배선 0.2 mm, 간격 0.13 mm, 비아 0.6/0.3 |
| Power(+5V, +3V3, GPS_VCC, *_AVCC) | 배선 0.4 mm, 간격 0.2 mm |
| PowerHigh(SERVO_5V, GND_PWR, VBAT_LOGIC*) | 배선 0.8 mm, 간격 0.2 mm, 비아 0.8/0.4 |
| GND | 0.2 mm 배선으로 연결 + 양면 GND 채움(간격 0.25 mm) |

작업 지시(전원 0.5–0.8 mm)와 다른 점:
- Power 클래스를 0.4 mm로 했습니다. 0.5 mm로는 Freerouting이 37–94개를 못 이었습니다(M, 시험 실행). 0.4 mm로는 3개였습니다(M).
  5 V / 3.3 V 부하는 최대 0.38 A입니다(E, 사양 부하 추정).
- Default 간격 0.13 mm: 넓힌 배선 하나가 패드에서 0.14 mm입니다(M). 나머지는 0.15 mm 이상으로 배선했습니다.

## 6. 배치(`scripts\place.py`)

- 좌표: 사양 (x, y) = KiCad (X − 100, 100 − Y). 기판 중심 = 기체 축.
- 커넥터는 사양의 가장자리에 고정했습니다. −x 서보 J8–J13, −x·y<−12 J18, +x·y>+14 J14·J17·J12(+NT1), +y·x>+5 J16, −y·x<−5 J7. 스크립트 검사 모두 통과(M).
- 칩마다 크리스털, 부하 콘덴서, 디커플링, AVCC, AREF, 리셋을 먼저 핀 가까이 놓습니다. 0.2 mm 격자에서 가장 가까운 빈자리를 찾습니다.
- 코트야드 겹침 0, 외곽 밖 0, 타이 띠(x −3…+3 mm) 안 부품 0(M). 코트야드 합 4,256 mm², 쓸 수 있는 면적 4,900 mm²의 87 %(M). 매우 빽빽합니다.
- 모듈 몸체는 `Dwgs.User`에 점선으로 그렸습니다. 몸체 아래에는 낮은 부품만 둡니다(A, 높이는 일반 패키지 값).
  - PLM100(22 × 19 mm, F): 2.0 mm 이하 부품만.
  - GY-521(약 21 × 16 mm, A): 2.0 mm 이하. FCU 칩(U2) 위에 겹칩니다.
  - BMP280(약 15 × 11.5 mm, A): 1.1 mm 이하.
  - 5 V 강압 모듈(약 17 × 11 mm급, A): 2.0 mm 이하. U6 AMS1117이 그 아래에 있습니다(열, A).
- PLM100 u.FL 쪽 모서리 5 × 2.6 mm는 동박 채움 금지입니다. 안테나는 u.FL 케이블로 나가므로 기판에 RF 배선은 없습니다.

### 사양과 다르게 한 점(배치 때문)

1. **ISP J19–J23을 타이 띠 안에 미실장 자리로 두었습니다.** 사양은 J19–J22를 실장 헤더로 했습니다.
   실장 헤더로는 76 × 70 mm에 J19–J23과 C46이 들어가지 않았습니다(M, place.py 실행 기록).
   2×3 2.54 mm 자리는 그대로라 USBasp와 맞습니다. 프로그래밍할 때 헤더나 포고 어댑터를 구멍에 대고 누르십시오.
   타이를 다른 곳으로 돌리면 헤더를 납땜해도 됩니다. **사용자 결정입니다.**
2. 부품 번호 글자를 실크가 아니라 F.Fab(조립도)에 두었습니다. 실크 겹침 경고를 없애려는 것입니다.
3. J12 코트야드 아래 끝이 사양 y = 13.1 mm입니다. 모터선 띠(|y| < 14)에 0.9 mm 들어갑니다(M). 핀은 띠 밖입니다.
4. GY-521 헤더 중심이 사양 (13.7, −7.45) mm입니다. FCU와 NAV 사이입니다. 중심 쪽이지만 정중앙은 아닙니다.
5. BMP280 모듈 몸체가 +x 가장자리 12 mm 띠의 |y| < 14 구역에 약 3 mm 들어갑니다(A 규칙과 충돌, 보고만).
6. C46(서보 220 µF)은 J12 옆이 아니라 서보 헤더 쪽 왼쪽 아래에 있습니다. J12 옆에는 자리가 없었습니다.
7. U3(COM)은 180° 돌렸습니다. UART1 핀이 PLM100 쪽을 봅니다.
8. 사양 JSON에서 `populate`가 `"optional (DNP)"`인 6개(C45, Q3, R15, R16, R42, R43)는 미실장으로 했습니다. SPEC_ko.md와 같습니다.

## 7. 주문 전에 확인할 가정(A 목록)

모듈은 KiCad 라이브러리에 없습니다. 라이브러리의 헤더·소켓 풋프린트로 대신했습니다. **핀 순서는 구매한 모듈의 실크로 확인하십시오.**

| 커넥터 | 가정한 핀 순서 |
|---|---|
| J1 GY-521 | 1 VCC(5 V), 2 GND, 3 SCL, 4 SDA, 5 XDA, 6 XCL, 7 AD0(GND → 0x68), 8 INT |
| J2 BMP280 [TYE-BMP29] | 1 VCC(3.3 V), 2 GND, 3 SCL, 4 SDA, 5 CSB(3.3 V → I2C), 6 SDO(3.3 V → 0x77) |
| J5 OpenLog | 1 GRN, 2 RXI, 3 TXO, 4 VCC(5 V), 5 GND, 6 BLK |
| J6 5 V 강압 모듈 | 1 VIN, 2 GND, 3 VOUT, 4 EN/NC. 입력 6–8.4 V, 1 A 이상 |
| J7 NEO-7M 리드(직접 만듦) | 1 VCC, 2 GPS TXD → GNS, 3 GPS RXD ← GNS, 4 GND. 모듈 쪽은 VCC, GND, TXD, RXD(, PPS 안 씀) |
| J14 로직 배터리 | 1 +, 2 −. JST-PH 극성은 제조사마다 다릅니다. Q10이 역전압을 막습니다 |
| J3·J4 PLM100 | 데이터시트 그림 5(p.12): 줄 간격 11.60 mm, 1.27 mm 피치, 핀 1/24는 먼 쪽 끝에서 2.82 mm. **1:1로 인쇄해 모듈을 대어 보십시오** |

그 밖의 A:
- 모듈 크기와 높이(위 6절). OpenLog를 세로 소켓에 꽂으면 높이 13.4 mm 제한을 넘을 수 있습니다. 직각 핀으로 눕혀야 합니다.
- 로직 배터리 2S 300 mAh, 약 20 g(E). params.json 질량 예산에 없습니다. VTOL은 이미 약 3.7 g 초과이므로 약 24 g 초과가 됩니다(E). **사용자 결정입니다.**
- BMP280 직각 소켓에 꽂힌 모듈이 기판 위 약 2.5 mm에 눕는다고 가정했습니다.
- JLCPCB 규칙 값(5절).

## 8. 엔진(AI_EDA, `B:\Claude`)으로 못 한 것

엔진은 고치지 않았습니다. 아래는 이 폴더의 스크립트로 처리했습니다.
- 회로도: 엔진은 평면 한 장만 씁니다. 계층 시트, DNP, `in_bom`, PWR_FLAG는 `build_schematic.py`가 덧붙입니다.
- 엔진은 겹친 핀(ATmega128A의 VCC 21/52, GND 22/53/63)을 거부합니다. 같은 넷일 때만 허용하도록 검사를 바꿔 썼습니다.
- 엔진의 핀 정렬은 "1"과 "MP"가 섞이면 TypeError가 납니다(J7). 안전한 정렬로 같은 검사를 다시 썼습니다.
- 기판: 둥근 외곽, 계층 경로, DNP 속성, 미연결 핀의 `unconnected-(…)` 넷, 동박 금지 구역은 `build_pcb.py`가 덧붙입니다.
- `.kicad_pro` 작성기, 배치 도구, 실제 라우터가 엔진에 없습니다. `place.py`와 Freerouting으로 했습니다.

## 9. 펌웨어 후속 작업(이번에는 바꾸지 않음, SPEC_ko.md 10절과 같음)

1. COM: PLM100용 AT 명령 드라이버. `sim/link_model.py` 다시 맞추기.
2. 지상국: 지상 쪽 PLM100과 `tools/ground_station.py` AT 명령.
3. NAV: BMP280 드라이버와 보정식. 시뮬레이션 기압 잡음 모델이 바뀝니다. **제어 관련 변경이라 이전 상태 보관과 전/후 비교 영상이 필요합니다.**
4. GNS: NEO-7M MON-VER가 UNKNOWN으로 읽힘(보고용).
5. 시뮬레이션: `sim/tether.py` bec_min_v 가정.

**다시 말합니다: NOT_VERIFIED. 이 기판은 만들지도, 전원을 넣지도 않았습니다.**
