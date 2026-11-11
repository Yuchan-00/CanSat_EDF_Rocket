# VTOL 캐리어 PCB rev 3 최종 보고서 (2026-11-12)

**이 보드는 한 번도 만들어지거나 전원이 들어간 적이 없습니다.** 이 보고서의 PASS는 모두 이 PC에서 돌린 도구(kicad-cli, 스크립트)의 결과이고, 보드가 동작한다는 뜻이 아닙니다. JLCPCB에는 아무것도 올리지 않았고, 로그인·장바구니·주문도 하지 않았습니다.

표시: M = 이 PC에서 도구로 측정, E = 추정(계산), A = 가정(실물로 확인 필요), F = 문서 근거. 근거가 없으면 NOT_VERIFIED.

이 보고서는 납품 단계에서 손으로 쓴 시점 기록입니다 (`scripts/make_all.py` 가 다시 만들지 않습니다). 숫자는 2026-11-12 00:08 ~ 00:40 (KST)에 디스크의 파일로 다시 돌린 검사에서 읽었습니다. 자세한 설명은 아래 두 문서에 있고, 이 보고서와 다르면 그 문서를 다시 만든 쪽이 최신입니다.

- 설계 설명: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/README_ko.md`
- 주문 안내서: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md`

## 1. 한눈에 보기

| 항목 | 상태 |
|---|---|
| 도구의 출고 판정 | **NOT ORDERABLE** (배선 규칙 검사 21개 중 5개 FAIL: battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance) |
| 주문 파일 | 모두 만들어져 있음 (거버 zip, BOM, CPL 두 묶음). 형식·내용 검사 PASS (M) |
| 회로도 ERC / 보드 DRC | 오류 0 / 위반 0, 미연결 0, 회로도 불일치 0 (M, kicad-cli 10.0.6) |
| JLC 부품 재고 | 2026-11-12T00:08:01+09:00 기준 40줄 모두 PASS (M, 그 시각 기준). 주문 당일 다시 확인해야 함 |
| 사용자가 정할 것 | (1) 배터리 경로 폭 FAIL과 승압부 배치 FAIL을 알고 그대로 주문할지, 고친 뒤 주문할지 (2) 6장의 D1~D17 |
| 결제 전 관문 | **JLC의 DFM 결과와 견적을 보기 전에는 결제하지 마십시오** (7장) |

도구 판정이 NOT ORDERABLE인 것은 파일이 깨졌다는 뜻이 아니라, 계획서가 정한 전원부 배선 규칙 5개를 이 보드가 지키지 못했고 그 규칙을 느슨하게 하지 않았다는 뜻입니다. 그대로 주문할지는 사용자가 정합니다 (`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 0장).

## 2. 무엇을 만들었나 (요청과 비교)

요청: "mcu가 장착될 pcb를 다시 설계하고 센서 장착 렌더링도 함께, 모든 부품은 JLCPCB 부품 라이브러리에 실제로 있고 주문 가능한 것으로, MCU와 센서를 빼고 모두 JLCPCB BOM 양식으로 써서 최종 파일과 BOM만 보내면 SMT 실장된 PCB를 받도록."

| 요청 | 결과 | 표시 |
|---|---|---|
| PCB 재설계 | 76 x 70 mm, 4층, 부품 자리 174개, 네트 105개. MCU 4개의 핀 256개는 rev 2c와 같은 네트 (차이 0). 외곽, 커넥터·MCU·크리스털·모듈 소켓·ESP 위치는 rev 2c 그대로. 꽂는 5 V 전원 모듈(Pololu)을 JLC 부품으로 만든 승압 회로(U8 TPS61023 + L5)와 저전압 차단(U9)으로 바꿈. Wi-Fi 모듈은 ESP-12F | M |
| 센서 장착 렌더링 | 그림 20장: 받은 상태 3장, 장착 뒤 4장, 확대 6장, 기체 안 GPS 1장, 설명 슬라이드 6장. 모듈 모양은 대부분 가정 치수(그림에 빨간 '가정' 표시) | M (그림), A (모듈 치수) |
| 모든 부품이 JLC에 있고 주문 가능 | BOM 40줄 모두 JLC 부품 번호가 있고, 2026-11-12 00:08에 JLC 부품 기록을 읽어 재고·주문 가능을 확인 (PASS 40/40). 재고가 적은 줄: C22436147 (J2, J5 소켓) 523개 | M (그 시각 기준) |
| MCU·센서 빼고 모두 JLC BOM | JLC 실장 157개 (SMT 34줄 + 스루홀 6줄). BOM·CPL은 JLC 견본 파일과 같은 머리줄 | M (형식), JLC 수락은 NOT_VERIFIED |
| 파일만 보내면 실장된 PCB | 올릴 파일 3개(거버 zip, BOM, CPL)가 준비됨. 다만 요청과 달리 사용자 몫으로 남는 일이 있음 (5장) | - |

요청대로 되지 않은 점 (숨기지 않고 적음):

- 프로그래밍 단자 8곳(J3, J19~J22, J24~J26)은 빈 구멍으로 남습니다 (A안). JLC가 실장하는 B안은 자리가 없어 불가능했습니다 (4장).
- 스루홀 커넥터 6줄을 JLC가 손납땜해 주는지는 견적에서만 확인됩니다.
- 전원부 배선 규칙 5개가 FAIL입니다 (3장 표, 8장).

## 3. 최종 재검사 결과 (이 단계에서 디스크의 파일로 다시 실행, 2026-11-12)

| 검사 | 결과 | 숫자 | 표시 |
|---|---|---|---|
| 회로도 ERC (kicad-cli 10.0.6, `--severity-all`, 납품 파일과 바이트가 같은 사본에서 직접 실행) | PASS | 위반 0, 시트 8장 | M |
| 보드 DRC (`--severity-all --schematic-parity`, `--refill-zones` 를 넣은 실행도 같음) | PASS | 위반 0, 미연결 0, 회로도 불일치 0 | M |
| `rev3_checks.py all` (전체 빌드 안에서 실행, 종료 코드 0) | PASS | determinism(파일 13개), export(ERC), netlist-vs-spec(부품 174, 네트 105), mcu-pinmap(핀 256, 차이 0), u8-u9, fit, nets 모두 PASS | M |
| `rev3_checks.py export-files` / `export-determinism` | PASS / PASS | 주문 파일 형식·fit 집합, 두 번 써서 같은 파일 | M |
| BOM = CPL = 사양의 fit 집합 (이 단계가 따로 쓴 검사 35항목) | PASS | 기본: BOM 40줄 / 부품 157 / CPL 157행. with_mcu: 41줄 / 161 / 161행, 더해진 것은 U1~U4 = C6510뿐. 중복 0, 미실장 부품 0, MCU·J1 없음(기본) | M |
| 두 거버 zip | PASS | 파일 13개씩 (구리 4층 + 드릴 포함). 두 zip은 F.Paste만 다름: 페이스트 패드 342개 / 598개 (차이 256 = U1~U4 패드). zip 안 파일 = `out/gerber/` 파일 | M |
| JLC 재고 관문 `jlc_gate.py` (인터넷, 읽기만) | PASS | 2026-11-12T00:08:01+09:00, 40/40줄, with_mcu PASS (C6510 3637개). 적은 줄: C22436147 523개(필요 40), C53426685 (L5) 1065개, C20345924 (U9) 1086개, C919459 (U8) 5850개 | M (그 시각 기준) |
| 설계 규칙 파일 | PASS | `kicad/vtol_carrier.kicad_pro` sha256 a64b7bfe58b8532d... 가 rev 2c 보관본과 바이트까지 같음. 간격 0.13 mm, 기본 선폭 0.2 mm, 비아 0.6 / 0.3 mm, 가장자리 0.3 mm, 구멍 사이 0.5 mm | M |
| 배선 규칙 (계획 13장) | **FAIL** | 21개 중 16개 PASS. FAIL: battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance. 크리스털 XTAL1 길이는 5.96 mm로 rev 2c와 같음(PASS) | M / E |
| 결정성 `determinism.py --cross-folder` (전체를 두 번 + 다른 폴더에서 한 번 다시 만들기) | **FAIL (STEP 파일 하나 때문)** | 바이트 비교 70개 중 다른 것 0개. 시각 줄을 뺀 비교 3개 중 1개 다름 (`out/vtol_carrier.step`: kicad-cli가 내보낼 때마다 항목 순서를 바꿈, 번호·순서를 뺀 내용은 같음). 다른 폴더 비교 59개: 51개 바이트 동일, 8개는 폴더 이름을 바꿔 넣은 뒤 동일, 다른 것 0개. 빌드 10.0분 / 10.0분 / 5.2분 | M |
| 출고 판정 (`out/manifest.json`) | **NOT ORDERABLE** | 이유: routing_rules FAIL 5개 | M |
| 그림 파일 | 모두 있음 | `out/render` 20장, `out/presentation` 13장, 회전 대조 그림 1장, 프로그래밍 접근 그림 1장 = 35장 모두 열림. 자동 기준(색 편차)에 걸린 1장(`sch_IO_connectors.png`)은 눈으로 열어 정상(내용이 성긴 회로도 시트)임을 확인. 렌더 20장과 보드 그림은 축소판으로 눈으로 확인 | M |

덧붙임:

- 결정성 검사에서 '실행 전 디스크의 파일'과 '첫 빌드'가 달랐던 결정적 파일은 `README_ko.md`, `out/ORDER_ko.md`, `out/jlc_fee_estimate.csv` 세 개입니다. 검사 직전에 JLC 재고를 새로 읽어서 확인 시각·재고 수·부품 단가(비용 추정 0.01~0.03 USD 차이)가 바뀐 것이 원인입니다 (M). 보드, 회로도, 거버 zip, BOM, CPL은 실행 전과 바이트까지 같습니다.
- STEP 파일(기구 확인용)은 주문 파일이 아닙니다. 그래도 계획의 규칙(머리줄만 빼고 바이트 동일)을 지키지 못했으므로 FAIL로 둡니다.
- kicad-cli가 그린 PNG 그림은 실행할 때마다 바이트가 달라집니다 (내용은 같음). 그림은 증거 자료이고 결정적 산출물이 아닙니다.
- 기본 선폭 0.2 mm보다 좁은 배선 조각이 300개 있습니다: 0.15 mm 282개(합 194.5 mm), 0.12 mm 16개(합 9.1 mm), 0.1 mm 2개(합 0.04 mm) (M). 자동 배선기가 패드 사이를 지날 때 좁힌 것이고 rev 2c에도 같은 모양이 있었습니다. 규칙 파일의 최소 선폭(0.1 mm, rev 2c와 같음)은 지키므로 DRC는 통과하지만, '선폭 0.2 mm'를 모든 구간에서 지킨 것은 아닙니다. JLC가 받는지는 DFM 결과로 확인됩니다 (NOT_VERIFIED).
- 이 단계의 재검사 기록: `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/impl/deliver/NOTES.md` (같은 폴더에 ERC/DRC 보고서, 검사 스크립트와 결과).

## 4. JLC에 보낼 파일

두 묶음 중 **하나만** 고릅니다. 두 묶음의 파일을 섞으면 안 됩니다 (zip과 BOM/CPL은 짝이 맞아야 합니다).

묶음 1 - 기본 (MCU 4개는 사용자가 직접 납땜, 요청대로):

| 파일 | 결과물 폴더 | 작업 폴더 (원본) | sha256 앞부분 |
|---|---|---|---|
| 거버 zip | `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/1_기본_MCU_직접납땜/vtol_carrier_gerber.zip` | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier_gerber.zip` | 5e7d74ecf66b8c5d |
| BOM | `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/1_기본_MCU_직접납땜/jlcpcb_bom.csv` | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom.csv` | fa4c8e2622885f4d |
| CPL | `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/1_기본_MCU_직접납땜/jlcpcb_cpl.csv` | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl.csv` | 738c4ea451000e0e |

묶음 2 - with_mcu (MCU 4개도 JLC가 실장, 계획의 권장안. BOM에 U1~U4 = C6510 한 줄이 더 있음):

| 파일 | 결과물 폴더 | 작업 폴더 (원본) | sha256 앞부분 |
|---|---|---|---|
| 거버 zip | `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/2_with_mcu_MCU도_JLC실장/vtol_carrier_gerber_with_mcu.zip` | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/vtol_carrier_gerber_with_mcu.zip` | 63eb3f0234e53e6b |
| BOM | `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/2_with_mcu_MCU도_JLC실장/jlcpcb_bom_with_mcu.csv` | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_bom_with_mcu.csv` | 465199b917ff914d |
| CPL | `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/2_with_mcu_MCU도_JLC실장/jlcpcb_cpl_with_mcu.csv` | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/jlcpcb_cpl_with_mcu.csv` | b0ab3d4240db4473 |

같이 볼 파일 (JLC에 올리는 파일이 아님):

- 주문 안내서 (주문 화면 입력표, 메모 문장, 결제 전 확인표): `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/ORDER_ko.md`
- 부품 방향 대조 그림 (JLC 미리보기와 비교): `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/cpl_rotation_check.png`, 근거 표 `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/cpl_rotation_table.csv`
- 따로 사야 하는 것 19줄 (MCU, 센서 모듈, 케이블, 배터리 등; 재고·가격은 NOT_VERIFIED): `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/off_board_purchase.csv`
- 주문 안내서 안에 적힌 경로는 작업 폴더(`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/`) 기준입니다. 결과물 폴더 `B:/0)AI_Cad/CanSat_v0.9/08_PCB/` 의 같은 하위 경로에 sha256이 같은 사본이 있습니다 (`B:/0)AI_Cad/CanSat_v0.9/08_PCB/copy_sha256.json`).

주문 설정의 기본값 (계획 D9, D11, D16; 화면 항목 이름은 NOT_VERIFIED): 4층, 1.6 mm, ENIG, 녹색, PCB 5장 중 2장 실장, PCBA Standard, 윗면만 실장, 레일은 JLC가 붙임, 부품 위치 확인(Confirm Parts Placement) 켬. 비용 추정(E): 기본 묶음 Standard 2장 122.56 USD (Standard 전용 수수료가 붙으면 154.41 USD). PCB 제작비, ENIG, 베이킹, 배송, 세금은 빠져 있고 실제 금액은 JLC 견적뿐입니다.

## 5. 프로그래밍 단자 A안 / B안, 그리고 사용자가 직접 해야 하는 일

프로그래밍 단자 (계획 D6):

- **A안 (지금 파일)**: MCU 굽기 단자 J19~J22와 ESP 굽기 단자 J3 (2x3), 리셋 점퍼 J24~J26 (1x2)은 케이블타이 띠 안의 **빈 도금 구멍**입니다. JLC는 여기에 아무것도 달지 않습니다. 사용자가 핀헤더를 납땜(36핀)하거나 포고핀 어댑터를 대고 굽습니다 (포고핀 접촉은 NOT_VERIFIED). 헤더를 납땜했다면 기체에 붙이기 전에 떼거나 잘라 띠를 비워야 합니다.
- **B안 (JLC가 헤더를 띠 밖에 실장)**: 배치 검색에서 놓을 자리가 0곳이었습니다 (M). 고정 부품만 남겨도 2x3 다섯 개가 들어가지 않아, 커넥터나 MCU 위치를 옮기지 않고는 불가능합니다. 그래서 B안 파일은 만들지 않았습니다.

MCU와 센서 말고도 사용자 몫으로 남는 일:

1. (묶음 1이면) ATmega128A-AU 4개 납땜. U1을 GY-521보다 먼저 납땜합니다.
2. 센서·모듈 장착: GY-521을 J1 구멍에 납땜, BMP280을 J2 소켓에, OpenLog에 직각 6핀 헤더를 납땜해 J5 소켓에, NEO-7M은 기체 윗뚜껑에 두고 JST-GH 케이블로 J7에. 그림: `B:/0)AI_Cad/CanSat_v0.9/08_PCB/out/render/e1_delivered_vs_completed.png` 과 같은 폴더의 e2a, e2b, e2c, e3, e4.
3. Standard 주문의 공정 레일 떼기.
4. 뒷면 다리 자르기: J15, J17은 도면값 1.8 mm로 규칙 1.5 mm를 넘어 반드시 자릅니다 (다리 4개). 다른 핀헤더·소켓도 재 보고 넘으면 자릅니다. J1, J14 다리 길이는 확인하지 못했습니다.
5. 프로그래밍 단자 준비 (위 A안).
6. GPS 케이블, 스위치·부저·아밍 리드, 배터리 준비 (`off_board_purchase.csv`).
7. 처음 켜기 전 벤치 시험과 기록 (켜짐·꺼짐 전압, +5V, +3V3, 부하 시험, 오실로스코프로 SW·+5V·VIN 파형, 배터리에서 U8 입력까지 저항): `ORDER_ko.md` 7장. 전원부 규칙 FAIL 때문에 이 측정이 특히 중요합니다.
8. 폼테이프와 케이블타이로 장착. 아랫면 실크의 두 줄 글자는 폼테이프 밑에 가려집니다.

## 6. 사용자 결정 D1~D17 (계획의 기본값으로 만들었음)

| Id | 질문 | 이번 파일에 쓴 기본값 |
|---|---|---|
| D1 | 5 V 변환 회로 방식 | TPS61023 승압(U8, C919459) + TLV3012B 차단(U9, C20345924) + L5 (C53426685) |
| D2 | 배터리 차단 전압 | 켜짐 3.44 ~ 3.73 V, 꺼짐 3.00 ~ 3.25 V (E, 실측 전) |
| D3 | 요청과 무관하게 바꾼 부품 번호 3건 유지 여부 | 유지: C45 C11366, J8~J12 C32713262, J2 / J5 C22436147 |
| D4 | MCU를 누가 붙일지 | 기본은 직접 납땜(묶음 1). JLC 실장용 묶음 2도 함께 만듦 |
| D5 | OpenLog 장착 방법 | 직각 헤더를 납땜해 J5 소켓에 꽂기 |
| D6 | 프로그래밍 단자 | A안 (빈 구멍). B안은 자리 없음 |
| D7 | J15 / J17 커넥터 | 스루홀 (C131337), 다리 4개 직접 절단 |
| D8 | 레벨시프터 FET | C40912 유지 |
| D9 | 주문 방식 | Standard (주문 화면에서 사용자가 선택) |
| D10 | 툴링 홀 | 설계에 넣지 않음 (JLC 레일에 생김) |
| D11 | 수량 | PCB 5장, 실장 2장 (주문 화면에서 선택) |
| D12 | 모듈 소켓 J2 / J5 | C22436147, 재고 부족 시 C55629861 → C54876735 (풋프린트 호환은 NOT_VERIFIED) |
| D13 | 배터리 전압 분압 저항 | 그대로 (R15, R16 47k). 차단 뒤에도 약 40 uA가 흐르므로 스위치를 꺼야 함 |
| D14 | 흡습 부품 베이킹 요청 | 주문 메모에 요청 문장 넣음 (수수료 미확인) |
| D15 | 모듈 치수 | 가정값 사용, 그림에 '가정' 표시. 실측 권장 |
| D16 | 표면처리·색상 | ENIG, 녹색, JLC 기본 적층 (추가 요금 미확인) |
| D17 | 실장하지 않은 여분 보드 | 기구 확인용으로만 |
| 추가 1 | 배터리 경로 폭 규칙 FAIL (2.0 mm보다 좁은 구간 8.57 mm)을 받아들이고 주문할지 | **결정 전** (`ORDER_ko.md` 0장 A) |
| 추가 2 | 승압부에 남은 FAIL (hot_loop, input_cap_l5, sw_node, crystal_distance)을 받아들이고 주문할지 | **결정 전** (`ORDER_ko.md` 0장 B) |

각 결정의 근거와 대안은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 9장과 계획서 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/REV3_PLAN.md` 에 있습니다.

## 7. 결제 전 관문

**JLC의 DFM 결과와 견적을 받기 전에는 결제하지 마십시오.** 순서:

1. 추가 결정 1, 2를 정합니다 (그대로 주문 / 고친 뒤 주문).
2. 주문 당일, 올리기 직전에 재고를 다시 확인합니다: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts` 에서 `B:/Claude/.venv/Scripts/python.exe -X utf8 -B jlc_gate.py` (읽기만, 약 2~3분). status가 PASS여야 합니다.
3. 파일 3개를 올리고, BOM 매칭 화면에서 모든 줄(40줄 또는 41줄)이 매칭되는지 봅니다. '부품 없음', '다른 부품 추천', '재고 부족'이 하나라도 있으면 멈춥니다.
4. 스루홀 6줄이 'JLC가 납땜'으로 견적에 들어 있는지 봅니다.
5. 부품 위치 미리보기를 `cpl_rotation_check.png` 와 하나씩 비교합니다. 특히 탄탈 C45 / C52, 전해 C47의 극성(거꾸로면 화재 위험)과 U9. 이 네 부품의 회전은 자료끼리 어긋나 NOT_VERIFIED입니다. J8~J12, J14, J16은 미리보기의 1번 핀 표시가 반대로 보이는 것이 정상이니 180도 돌리지 않습니다.
6. DFM 결과에 가장자리 부품(U7 0.15 mm, J15 0.29 mm), 레일, 구멍·간격·선폭 지적이 없는지 봅니다. 지적이 있으면 마음대로 바꾸지 말고 멈춥니다.
7. 견적 금액을 추정과 비교한 뒤에만 결제합니다.

## 8. 알려진 문제 (도구가 찾았고 고치지 않은 것)

- **배터리 전원 경로 폭 (battery_path FAIL)**: 계획은 2.0 mm 이상을 요구하는데 8.57 mm 구간이 좁습니다 (1.0 / 1.5 / 1.56 mm). 구리 저항은 20 C에서 0.03173 Ohm (목표 0.035 Ohm 안, E), 60 C에서 0.03672 Ohm (목표 초과, E).
- **승압부 배치 (hot_loop, input_cap_l5, sw_node, crystal_distance FAIL)**: 출력 커패시터 C43이 U8에서 2 mm 밖(3.25 / 3.537 mm), 입력 커패시터 C41에서 L5 입력까지 구리 13.53 mm, 스위칭 선(SW)이 6.619 mm로 길고 두 커패시터 밑을 지남, 승압부가 NAV 크리스털에 rev 2c의 전원 모듈보다 가까움 (가장 가까운 부품 7.501 mm, rev 2c 9.217 mm). 과전압·리플·발열·크리스털 잡음은 보드에서 재기 전에는 모릅니다. 5 mm, 2 mm 기준은 계획에 숫자가 없어 스크립트가 읽은 값(E)입니다.
- 결정성 검사 FAIL: STEP 파일 하나 (3장).
- 기본 선폭 0.2 mm보다 좁은 배선 조각 300개 (3장).
- 저전압 차단 비교기 U9의 바이패스 커패시터 C59에 자기 GND 비아가 없습니다 (가장 가까운 GND 비아 3.287 mm). BOOST_EN 선은 마무리 스크립트가 이은 긴 선(45.6 mm, 비아 8개)입니다.
- 가장자리 2.5 mm 안에 부품 몸체 34개 (JLC 실장 28개). U7 0.15 mm, J15 0.29 mm는 0.3 mm보다 가깝습니다 (rev 2c와 같은 위치).
- 모듈 끼움: 가정 치수로는 충돌 0건이지만, OpenLog 아랫면 부품을 1.2 mm로 잡는 보수적 경우 U6 -0.5 mm, C3 -0.05 mm가 겹칩니다 (A).
- CPL 회전 4개(C45, C47, C52, U9)는 자료가 서로 어긋납니다.
- J14 (Molex) 3D 모델이 KiCad 라이브러리에 없고, 구멍 지름을 제조사 도면과 대조하지 못했습니다.
- 아랫면 실크에 저장소 주소와 연락용 이메일 두 줄이 인쇄됩니다 (윗면에 빈자리 없음). 주소 글자는 `scripts/silk.py` 와 보드 파일 4개에만 들어 있습니다.
- 검토 기록: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/reports/review_findings_rev3.md` (독립 검토 지적과 처리).

## 9. 확인되지 않은 것 (NOT_VERIFIED)

- 보드 동작 전체: 5 V 출력, 차단 전압, 전류 여유, 발열, 배터리 경로 저항, Wi-Fi 송신 중 전압. 모두 데이터시트 계산(E)이고 실측이 없습니다.
- JLC의 DFM 결과, 견적, BOM / CPL 업로드 미리보기, 스루홀 손납땜 수락, 레일 붙이는 방식, 가장자리 부품 수락, 실크 인쇄 품질, 실제 비용.
- 주문 당일의 재고 (지금 기록은 2026-11-12T00:08:01+09:00 기준).
- CPL 회전: C45, C47, C52, U9는 근거가 어긋남. 나머지도 JLC 미리보기로만 확정됩니다. JLC의 0도 기준이 EasyEDA 풋프린트와 같다는 것도 가정입니다.
- 모듈 치수와 끼움 (GY-521, BMP280, OpenLog, NEO-7M): 하나도 재지 않았습니다. BMP280 헤더 종류(직각/일자)와 핀 인쇄 순서, 모듈 핀 순서, GPS 선 색도 가정입니다. 그림은 맞춤 검사가 아닙니다.
- 기체 안 GPS 케이블 경로 (그림은 예시, 간섭 검사 없음).
- 리플로 온도 정격: 문서에서 찾지 못한 부품 번호가 32개입니다. Economic 방식에서는 C2833309, C2286, C53426685가 한계에 걸립니다.
- 대체 부품(J2 / J5, J7)의 풋프린트 호환.
- 따로 살 부품의 재고와 가격.
- 뒷면 다리 길이: J1, J14.
- 다른 PC나 다른 KiCad 판에서 다시 만들어도 같은 파일이 나오는지 (이 PC의 두 폴더에서만 확인).
- 새로 자동 배선을 돌렸을 때 같은 결과가 나오는지 (기록된 세션을 다시 넣는 방식만 재현됨).

## 10. 파일 위치와 다시 만드는 방법

- 결과물 폴더: `B:/0)AI_Cad/CanSat_v0.9/08_PCB/` (안내 `B:/0)AI_Cad/CanSat_v0.9/08_PCB/README.md`). 주문 파일은 `00_주문파일/`, 그림은 `out/render/` 와 `out/presentation/`, KiCad 프로젝트는 `kicad/`, 사양은 `spec/`, 스크립트는 `scripts/`. 이전 판(rev 2c, 2026-11-05)은 지우지 않고 `B:/0)AI_Cad/CanSat_v0.9/08_PCB/이전_rev2c_2026-11-05/` 로 옮겼습니다.
- 작업 폴더 (원본, git 브랜치 `pcb-rev3`): `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/`
- 그림 목록과 설명: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/INDEX_ko.md`
- 사양 요약: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/SPEC_ko.md`
- 다시 만들기 (작업 폴더의 `scripts` 에서, 환경변수 `PYTHONDONTWRITEBYTECODE=1`): `B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py` (약 10분, 배선 규칙 FAIL 때문에 종료 코드 2가 정상). 결과물 폴더의 사본은 보관용이고, 거기서 다시 만들면 사양 파일이 폴더 이름을 담고 있어 JLC 재고 기록이 '다른 사양'으로 판정됩니다.
- 이 단계에서는 커밋하지 않았습니다. 바뀐 파일은 작업 폴더에만 있습니다 (브랜치 `pcb-rev3`, 마지막 커밋 812a8ba 이후의 변경).
