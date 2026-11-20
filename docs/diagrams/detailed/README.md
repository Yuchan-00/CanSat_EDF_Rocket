# CanSat EDF Rocket with ATmega128 - 블록도·순서도

<!-- detailed-banner -->
> **상세판 (Mermaid 원본, 검증용 출처 표 포함).** 교과서식 그림은 [README.md](../README.md)에 있습니다.

작성일: **2026-11-19**

- **형식:** 모든 그림은 Mermaid로 그렸습니다. GitHub가 마크다운 안의 `mermaid` 코드 블록을 그대로 그려 줍니다.
- **오프라인용 이미지:** `img/` 폴더에 SVG와 PNG가 따로 있습니다. 로컬에서 Mermaid 11.17.2와 헤드리스 Chrome으로 렌더했습니다.
- **출처:** 그림의 모든 노드는 아래 출처 키(파일:행)로 추적할 수 있습니다. 각 문서의 "노드 출처" 표를 보십시오.


> 참고: 출처 표의 `sim_tools/...` 경로는 개발 PC의 작업 폴더(새 본체 사양 단계)라 이 저장소에는 아직 없습니다. 브랜치 이름이 적힌 출처(`body-redesign`, `pcb-rev3`)는 이 저장소의 브랜치입니다.

## 목차

| 번호 | 문서 | 내용 | 그림 종류 |
|---|---|---|---|
| 01 | [시스템 블록도](01_시스템_블록도.md) | 기체 하위 시스템, 지상국, 링크 | flowchart |
| 02 | [전기·전원 블록도](02_전기_전원_블록도.md) | 6S / 1S 전원 트리, 전압, 퓨즈(없음), 스위치, 접지, 커넥터 J1-J26 표 | flowchart + 표 |
| 03 | [항공전자 데이터 블록도](03_항공전자_데이터_블록도.md) | MCU 3(+1)개, 센서, 버스, 핀·포트, 서보, ESC, GPS, 무선 | flowchart + 표 |
| 04 | [기계 구성도](04_기계_구성도.md) | 노즈 → 꼬리 적층과 z 위치, 숨은 배선, 조립 순서 | flowchart |
| 05 | [비행 순서도](05_비행_순서도.md) | 패드 → 착지, 중단과 페일세이프 | flowchart |
| 06 | [펌웨어 상태도](06_펌웨어_상태도.md) | FCU 감독기 6개 상태와 가드, 인터록 순서 | stateDiagram-v2 + flowchart |
| 07 | [제어 루프 순서도](07_제어_루프_순서도.md) | 센서 → 추정기 → 자세 PID → 믹서 → 베인, 주기 | flowchart + sequence |
| 08 | [사출 순서도](08_사출_순서도.md) | 정점 판단, CanSat 분리, 낙하산, 타이머와 백업 | flowchart + sequence |
| 09 | [개발·검증 흐름도](09_개발_검증_흐름도.md) | 설계·시뮬·펌웨어·PCB 파이프라인, 시험 계획 | flowchart |

## 각 그림이 반영한 버전

| 대상 | 기준 | 사용한 그림 |
|---|---|---|
| 펌웨어, 시뮬레이션, 지상국, params.json | 워크트리 `CanSat_EDF_Rocket_body`, 브랜치 **`body-redesign` @ 78f59fa** (2026-11-19, 로켓 전용, VTOL 제거) | 01, 03, 05, 06, 07, 08, 09 |
| 캐리어 PCB | **rev 3.1**, 커밋 **4051623** (`pcb-rev3`, `main`에도 있음), 넷리스트 `out/vtol_carrier.net` (title block rev "3") | 01, 02, 03 |
| 기체 구조 (새 본체) | `sim_tools/runs/body_redesign/spec/새_본체_상세사양.md` **rev 3.4.2**, 꼬리 V 1안 (사양·질량 모델 단계, 출력용 CAD 아님) | 01, 04, 05, 08, 09 |
| 요구사항·사용자 결정 | `REQUIREMENTS.md`, `사용자_결정_목록.md` rev 3.4.2 | 전체 |

**PCB 버전 주의:** `body-redesign` 안의 `pcb/vtol_carrier` 폴더는 아직 **rev 2c**입니다. 이 폴더에는 J13이 없습니다. 실제로 주문해서 비행할 보드는 rev 3.1이고, 그림은 rev 3.1을 따릅니다. 폴더 이름의 "vtol"은 예전 이름이 남은 것입니다.

## 읽는 법·범례

모든 그림에서 노드 색이 같은 의미를 가집니다.

| 모양 / 색 | 의미 |
|---|---|
| 초록 실선 상자 | **IMPL**: 저장소 코드나 보드에 구현된 것 |
| 파랑 실선 상자 | 지상국(PC, 지상 ESP)에서 하는 일 |
| 노랑 상자 | 전원(배터리, 레일, 레귤레이터, ESC) |
| 보라 상자 | 기계 부품, 기계 동작 |
| 빨강 상자 | 중단(ABORT), 안전 동작, 거부 |
| 주황 점선 상자 | **PLANNED**: 요구·사양에만 있고 아직 구현·수행되지 않은 것 |
| 회색 점선 상자 | 보드에 배선은 되어 있지만 로켓 펌웨어가 쓰지 않는 것 (U4 GNS, GPS, 1S 감시) |
| 연분홍 점선 상자 | **ASSUMED**: 저장소에 없는 내용 |
| 굵은 화살표 `==>` | 주 데이터 링크 (nlink, ilink, ESP-NOW) |
| 점선 화살표 `-.->` | 대체·미사용·계획 경로 |

**수치 라벨:**

- **M** = 도구로 측정한 값
- **F** = 설계 파일 값
- **E** = 추정값
- **A** = 가정값

라벨이 없는 수치는 설계 파일이나 펌웨어 상수입니다.

## 출처 키

| 키 | 경로 |
|---|---|
| P | `CanSat_EDF_Rocket_body/params.json` (body-redesign) |
| PG | `firmware/atmega128_fc/src/params_gen.h` (params.json에서 생성) |
| PIO | `firmware/atmega128_fc/platformio.ini` |
| FW | `firmware/README.md` |
| `*.c`, `*.h` | `firmware/atmega128_fc/src/` 아래 (예: `core/onboard.c`, `main.c`, `nav/nav_main.c`, `com/com_link.c`) |
| SPEC | `docs/SPEC.md` (body-redesign) |
| RD, CS, NET, BOM, BUY | rev 3.1 PCB 폴더(`pcb-rev3` 워크트리)의 `pcb/vtol_carrier/` 아래 파일<br/>RD = `README_ko.md`, CS = `spec/carrier_spec.json`, NET = `out/vtol_carrier.net`, BOM = `out/vtol_carrier_bom.csv`, BUY = `out/따로_살_부품_ko.md` |
| PCB31 | RD와 같은 파일 |
| NBS | `sim_tools/runs/body_redesign/spec/새_본체_상세사양.md` (rev 3.4.2) |
| REQ | `sim_tools/runs/body_redesign/REQUIREMENTS.md` |
| DEC | `sim_tools/runs/body_redesign/spec/사용자_결정_목록.md` |
| SP | `sim_tools/runs/body_redesign/spec/spec_params.json` |
| R2 | `sim_tools/runs/body_redesign/spec/rev2_archive/새_본체_상세사양.md` |
| VAL | `sim/validation_2026-11-18/README.md` (body-redesign) |
| ST | `sim_tools/README_ko.md` |
| facts_hw / facts_fw / facts_sys | 이 그림을 만들 때 쓴 조사 메모 (`sim_tools/runs/diagrams/facts_*.md`) |

`sim_tools/...` 아래 파일(NBS, REQ, DEC, SP, R2, ST)은 이 저장소 밖의 로컬 작업 폴더에 있습니다.

## ASSUMED (저장소에 없는 내용)

1. **6S → ESC 하네스와 J18.1 VS_VIN 분기 위치:** 도면이 없습니다. XT60은 예산 BOM(`docs/budget/bom.json:161`)에만 나옵니다. (02)
2. **LoRa 대체 링크의 기체 쪽 연결:** rev 3.1에서는 COM PC3-PC5가 미연결입니다. LoRa를 쓰려면 보드를 수정해야 한다고 봅니다. (01, 03)
3. **GPS 프레임 형식(NMEA/UBX):** 저장소에 없습니다. 어차피 로켓 펌웨어는 GPS를 읽지 않습니다. (03)
4. **분리 서보 부팅 튐 위험:** RELEASE_PWM에 풀다운이 없다는 것은 넷리스트에서 확인한 사실입니다. 다만 그 결과 서보가 실제로 튀는지는 가정이고, 측정이 필요합니다. (08)
5. **지상 ESP-07S의 USB 연결 방식:** 펌웨어 `firmware/esp_bridge`에는 있습니다. 하드웨어 구성은 가정입니다. (01)
6. **시험 순서:** 측정 → 낙하 시험 → 단계별 시험의 연결 순서는 문서에 명시되어 있지 않습니다. (09)
7. **스톨 전류:** 서보 스톨 전류 약 0.7 A x 4, 1S 팩 PCM만으로 하는 전류 제한, MTS-102 정격 2 A 이상은 원문에서도 가정(A)입니다. (02)

## 문서·코드·보드 사이의 불일치 (그림은 코드와 rev 3.1 보드를 따름)

1. **칩 수:** 보드에는 ATmega128A가 4개 있지만 로켓 펌웨어는 3칩(NAV, FCU, COM)만 씁니다.
   - U4 GNS와 GPS(J7)는 배선만 있고 쓰이지 않습니다.
   - `firmware/README.md:1`과 저장소 README에는 아직 "네 칩"이라고 적혀 있습니다.
2. **GPS 연결:** params.json:1134는 GPS가 "NAV USART1"에 붙어 있다고 적고 있습니다. 보드에서 NAV USART1은 GNS와의 링크(glink)입니다.
3. **낙하산 크기:** params.json:1426은 1.4 m입니다. 사용자는 **1.2 m**로 결정했습니다(REQ:32; NBS:1283).
4. **탑재 정점 감지 백업:** 기압 + 가속도 + 시간 상한 방식이 REQ:18에 요구되어 있지만 **펌웨어에 없습니다**. 현재 정점 분리는 지상국 보조가 판단합니다.
5. **플랜트 상수와 게인:** 펌웨어의 값은 예전 본체 것입니다(`gains.h:39` STALE = 1). 예: l_v 0.3495 m. 새 본체 V 꼬리 팔은 272 mm입니다.
6. **기압계 이름:** 주석에는 MS5611이 남아 있지만 기본 빌드는 BMP280입니다.
7. **분리 출력 풀다운:** 분리 출력(PE5)에만 부팅 풀다운이 없습니다. 베인 출력과 ESC 출력에는 있습니다.
8. **로직 쪽 퓨즈:** 로직 쪽에 퓨즈나 PTC가 없습니다.
9. **추력 합격 기준:** 시험 문서의 2단계 합격 기준은 30 N입니다. 새 규칙은 꼬리 링 장착 상태에서 25.0 N 이상입니다.
10. **이륙 가능 여부:** 설계 추력 20 N에서 T/W는 0.981이므로 이륙하지 못합니다. 25.0 N 이상을 실측하기 전에는 자유 비행이 금지입니다.
11. **하드웨어 검증 상태:** 보드(rev 3.1)는 제작·통전한 적이 없습니다. 펌웨어 타이밍은 모두 시뮬레이션 값입니다. 둘 다 NOT_VERIFIED입니다.

## 이미지 다시 만들기

`img/`의 SVG와 PNG는 각 `.md`의 `mermaid` 블록을 순서대로 렌더한 것입니다.

- **파일 이름:** `<문서이름>_<블록번호>.svg` / `.png`
- **렌더 환경:** Mermaid 11.17.2, Playwright-core, 로컬 Chrome. 인터넷 연결은 쓰지 않습니다.
- **라벨 형식:** 렌더할 때 `htmlLabels: false`로 SVG 텍스트 라벨을 만들었습니다. 그래서 다른 SVG 뷰어에서도 글자가 보입니다.
- **원본:** 마크다운이 원본입니다. 그림을 고칠 때는 `.md`를 고치고 이미지를 다시 렌더하십시오.
