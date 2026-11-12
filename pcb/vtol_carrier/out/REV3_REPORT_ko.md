# VTOL 캐리어 PCB rev 3 최종 보고서 (2026-11-12)

**이 보드는 한 번도 만들어지거나 전원이 들어간 적이 없습니다.** 이 보고서의 PASS는 모두 이 PC에서 돌린 도구(kicad-cli, 스크립트)의 결과이고, 보드가 동작한다는 뜻이 아닙니다. JLCPCB에는 아무것도 올리지 않았고, 로그인·장바구니·주문도 하지 않았습니다.

표시: M = 이 PC에서 도구로 측정, E = 추정(계산), A = 가정(실물로 확인 필요), F = 문서 근거. 근거가 없으면 NOT_VERIFIED.

이 보고서는 납품 단계에서 손으로 쓴 시점 기록입니다 (`scripts/make_all.py` 가 다시 만들지 않습니다). 1~10장의 숫자는 2026-11-12 00:08 ~ 00:40 (KST)에 디스크의 파일로 다시 돌린 검사에서 읽었습니다. 자세한 설명은 아래 두 문서에 있고, 이 보고서와 다르면 그 문서를 다시 만든 쪽이 최신입니다.

**같은 날 그 뒤에 사용자 결정으로 바뀐 내용은 바로 아래 '2026-11-12 변경' 절에 있습니다. 1~10장은 00:40 시점의 기록으로 지우지 않고 남겨 두었고, 지금과 달라진 줄에는 `[변경됨]` 표시와 지금 값을 덧붙였습니다. 서로 다르면 '2026-11-12 변경' 절이 맞습니다.** 맨 위 문단의 "JLCPCB에는 아무것도 올리지 않았고, 로그인·장바구니·주문도 하지 않았습니다"도 00:40 시점의 문장입니다 `[변경됨]`: 그 뒤 01:30 ~ 02:06에 사용자의 로그인된 브라우저에서 그때의 주문 파일을 주문 화면에 올려 항목 이름과 견적을 읽었습니다 (장바구니 항목의 'Edit Order' 화면 포함; 기록 `spec/jlc_order_page_20261112.json`). 이 도구(스크립트)는 지금도 화면을 열거나 로그인하지 않으며, 그 기록에 주문·결제는 없습니다.

- 설계 설명: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/README_ko.md`
- 주문 안내서: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md`

## 2026-11-12 재설계: 외곽 탭, 승압부 재배치, 배터리 경로 2.0 mm (이 절이 가장 최신)

이 절은 2026-11-12 20:10 (KST) 무렵 손으로 덧붙였습니다. **이 절 아래의 모든 절(‘2026-11-12 변경’과 1~10장)은 재설계 전 보드(sha256 cc750ce9...)의 기록입니다.** 그 안의 '76 x 70 mm', 'NOT ORDERABLE (배선 규칙 5개 FAIL)', 견적 76.82 / 83.15 / 124.84 / 150.67 USD, 파일 해시는 재설계 전 값이며 지우지 않고 남겨 두었습니다. 지금 값과 설명은 다시 생성되는 README_ko.md 와 out/ORDER_ko.md 가 맞습니다.

작업 폴더에 설치 (22:15 ~ 23:00, 손으로 덧붙임): 재설계는 임시 사본에서 만들고 검증했습니다. 그 사본의 스크립트(바이트까지 같음), 배선 세션과 그 기록, 이 보고서, 그림 목록, 재설계 전 배선 보고서 사본만 이 작업 폴더에 옮겼고, 나머지 결과물은 여기서 깨끗한 상태부터 다시 만들었습니다 (make_all.py, 세션 재생). 보드, 배선 전 보드, 배치, 사양, BOM, CPL, 두 거버 zip은 사본의 것과 바이트까지 같습니다 (M). JLC 재고 확인은 여기서 다시 했습니다 (아래 표).

사용자 요청 (그대로): "그럼 다시 설계를 진행해줘. 실패한게 있다면 완료되었다고 말하지마", "보드의 사이즈를 조정하는 방법은 없어?", 프로그래밍 구멍 이동에 대해 "옮겨도 됨".

| 무엇 | 지금 값 | 출처 / 표시 |
|---|---|---|
| 외곽 | 본체 76 x 70 mm 사각형(변·모서리 그대로) + -y 변 탭 KiCad x 117.3~132, y 135~142.2 mm (모서리 r 1, 안쪽 필렛 r 1), 탭 넓이 105.8 mm2, 외접 76 x 77.2 mm | out/reports/board_metrics.json board_fit.outline, M |
| 기체 맞춤 | board_fit PASS: 보어 벽까지 4.44 mm, 탭-270도 통풍구 13.05 mm, 315도 나사 2 mm 안 부품 없음, 나사 아래 높이 1.3 mm | 같은 파일, M (나사 길이 A) |
| 옮긴 부품 | J19 / J22 / J26 +0.55 mm (y, 사용자 승인), C42 +0.20 mm (x), 승압부 U8 / L5 / C41 / C43 / C57 / R63 / C58 / R64 | kicad/placement.json, M |
| 배선 규칙 | 21 / 21 PASS (재설계 전 16 / 21) | out/reports/routing_rules.json (sha256 f839493c...; 작업 폴더에 설치한 뒤 다시 만든 것 - 검사한 보드 파일의 경로 3줄만 설치 전 사본 0d0d09fb...와 다름), M |
| 배터리 경로 | 0.03096 Ohm (20 C, E; 목표 0.035), 60 C 0.03583 Ohm (E, 목표 넘음 - 판정은 20 C), 가장 좁은 폭 2.0 mm | routing_rules.json battery_path |
| ERC / DRC | ERC 0 / 0; DRC 위반 0, 경고 0, 미연결 0, 회로도 불일치 0 (kicad-cli 10.0.6) | out/manifest.json, M |
| JLC 재고 | PASS 33 / 33줄, with_mcu PASS, 2026-11-12T22:27:21+09:00 (U9 C20345924 얇은 재고 637개; 설치 전 사본의 19:32:28 확인은 653개) | out/jlc_gate_report.json, M (그 시각) |
| BOM | 33줄 / 144개 (재설계 전과 같은 파일), CPL은 옮긴 부품 좌표가 바뀜 | out/manifest.json bom_cpl, M |
| 출고 판정 | 도구 값 'ORDERABLE (tool checks only; NOT_VERIFIED - never built)', 막는 항목 없음 | out/manifest.json release_gate, M |
| 결정성 | 20:10 기록: STEP 정규화 비교 FAIL → 전체 FAIL. **21:42 검증 후 수정 뒤: determinism.py --cross-folder 전체 PASS** (바이트 비교 FAIL 0, 시각 줄을 뺀 비교 FAIL 0, 다른 폴더 PASS). **작업 폴더에 설치한 뒤 22:39 ~ 22:56에 다시 실행: 전체 PASS** (바이트 비교 71개 중 다른 것 0, 시각 줄을 뺀 비교 3개 중 0, STEP 번호 정렬 비교 PASS, 다른 폴더 60개 비교 = 55개 그대로 같음 + 5개 폴더 이름을 바꾼 뒤 같음, 다른 것 0). STEP은 이제 내보낸 뒤 `scripts/step_canon.py` 가 항목 번호를 내용으로 다시 매기고, 같은 그래프인지 항목마다 확인한 뒤에만 바꿉니다 (결정성 검사 규칙은 그대로) | out/reports/determinism.json, M |

글자로만 통과했거나 여유가 작은 곳 (README 1장에 숫자): C41 GND는 윗면에서 FB 분압 저항 사이를 지나는 0.2 mm 선(약 14 mm)으로만 U8 GND와 이어짐 (내층 비아 사이 직선 약 8.3 mm); U8 VIN 핀 옆에는 입력 커패시터가 없음 (C41은 L5 입력 패드 옆); C41 VIN 경로는 L5 입력 패드를 지남; C57 GND는 L5 둘레 GND 면으로만 이어짐; C43 GND는 0.25 mm 가는 선; L5 입력 앞 0.4 mm 비아 두 개의 도금 단면이 2.0 mm 선보다 작음; SW 패드 간격 여유 0.124 mm; 미리 그린 배선은 자동 넓히기에서 뺌; place.py가 J19 / J22 / J26을 'rev 2c 자리 + 승인한 이동'과 비교 (외곽 맞춤·추가 검사 말고 사용자 결정으로 바꾼 판정 검사 코드는 이것 하나; 그 밖에 determinism.py의 폴더 이름 바꾸기 규칙과, 21:00 검증 후 수정에서 외곽 맞춤으로 prog_access.py의 기판 밖 칸 막기와 routing_checks.py 식별 문구 가장자리 거리를 고침 - README 3장); IR은 76 x 70 사각형만 가짐.

검증 후 수정 (21:00 ~ 21:45, 손으로 덧붙임): 프로그래밍 헤더 B안 검색(prog_access.py)이 탭 때문에 넓어진 격자에서 기판 밖 띠(약 460 mm2, 케이블타이 띠 포함)를 빈 자리로 셌던 것을 고쳤습니다. 결과는 재설계 전과 같이 'B안: 모든 종류에서 불가' (SMD / THT 2x3 자리 0 / 0, M, out/reports/programming_access.json).

확인되지 않은 것 (NOT_VERIFIED): 새 거버의 JLC 견적·DFM (저장된 견적은 모두 재설계 전 거버의 것), 탭 쪽 변의 V-cut 불가로 Standard 레일의 방식·값, 기체 CAD(아직 76 x 70 상자) 반영, 315도 조인트 나사 길이, 22 uF 커패시터 직류 바이어스, 보드 동작 전체. 그대로 남는 주의: C45 / C47 / C52 / U9 회전·극성은 JLC 위치 확인에서 직접 확인, 주문 당일 jlc_gate.py 다시 실행, 결제는 사용자만.

## 2026-11-12 변경: ESP 직접 실장, Economic, 무연 HASL (스루홀 커넥터도 직접 납땜)

이 절은 2026-11-12 02:35 ~ 04:00 (KST)에 추가했습니다. 숫자는 그 시간에 디스크의 파일로 다시 돌린 빌드와 검사에서 읽었습니다. **이 보드는 여전히 한 번도 만들어지지 않았고, 도구의 출고 판정은 그대로 NOT ORDERABLE입니다 (배선 규칙 5개 FAIL - 이번 변경과 무관하며 바뀌지 않았습니다).**

사용자 요청 (그대로):

1. "아, esp도 내가 직접 실장할거야. 그럼 이코노믹으로 진행하고 pcb의 코팅도 무연으로 해줘"
2. "이코노믹으로 스루홀 제외하고 어떤 부품을 따로 구매해야하는건지 리스트를 정리해줘"

이 요청을 다음 뜻으로 받아 적용했습니다: (1) ESP-12F(U7)는 사용자가 납땜, (2) 주문은 JLC Economic PCBA, (3) 표면처리는 무연 HASL, (4) 스루홀 커넥터 6줄(부품 12개)도 JLC 실장에서 빼고 사용자가 사서 납땜, (5) 베이킹은 주문하지 않음 (Economic에서는 고를 수 없음). 풋프린트, 네트, 부품 위치, 실크, 구리는 하나도 바꾸지 않았고 새 배선도 하지 않았습니다. `[변경됨: 04:05 이후 아랫면 실크 이름표 3개의 글자 순서를 고쳤습니다 - 바로 아래 절. 풋프린트·네트·부품 위치·구리는 그대로이고 새 배선도 없습니다.]`

### 04:05 ~ 05:30 추가: 검증에서 나온 지적 처리 (아랫면 실크 이름표 3개, 비용 추정표, 근거 파일)

독립 검증이 지적 4건을 냈고 (주문을 막는 문제는 없음), 이 시간에 처리했습니다. **구리·배치·풋프린트·네트·BOM·CPL은 바뀌지 않았고 새 배선도 하지 않았습니다 (기록된 세션의 재생).** 보드에서 바뀐 것은 아랫면 실크의 글자 3줄뿐입니다. 이 절 아래의 표와 문장 가운데 보드·거버 zip의 해시, 검사 시각, 비용 추정은 03:57 시점 값이고, 이 절의 값이 최신입니다.

| 지적 | 처리 | 표시 |
|---|---|---|
| V1 (중요): 아랫면 실크 '1S BAT +-' 를 위치대로 읽으면 '+' 가 J14의 GND 패드 쪽 | **고침.** '1S BAT -+' 로 바뀌어, 아랫면에서 볼 때 '-' 가 둥근 패드(GND), '+' 가 네모난 패드(1번 핀, 배터리 +) 쪽에 옵니다 | M |
| V2: 비용 추정표(E)가 스루홀 0곳인데 손납땜 고정비 3.58 USD를 더함 | 고침. 기본 묶음 Economic 2장 추정 68.69 → 65.11 USD (E). 주문 화면에서 읽은 견적 76.82 USD (M)는 그대로 | E |
| V3: 근거 파일 경로가 세션 임시 폴더를 가리킴 | 구리 동일성 비교 기록 3개를 `out/reports/evidence_20261112/` 에, LCSC 원본 응답 18개를 `spec/lcsc_raw_20261112/` 에 사본으로 넣음. 이 보고서와 계획서(`spec/REV3_PLAN.md`)의 그 밖의 `C:/Users/.../scratchpad/...` 경로는 세션 임시 폴더이고 보존되지 않을 수 있습니다 | M |
| V4: `out/reports/rev3_fit_sets.json` 의 넷리스트 해시가 디스크 파일과 다름 | 보고서에 '검사 당시 파일의 해시'라고 적고, 날짜 줄을 뺀 해시(c4475e142ba6d994)를 함께 기록. 디스크의 `out/vtol_carrier.net` 과 같음 | M |

**V1의 원인과 고친 범위 (M):**

- 이름표 끝의 핀 글자를 언제나 1번 핀부터 적고 있었습니다. 아랫면 글자는 아랫면에서 바로 읽히도록 좌우를 뒤집어 인쇄하므로, 핀 줄과 **나란히** 적힌 아랫면 이름표는 글자 순서가 패드 순서와 반대였습니다. J14만이 아니라 같은 모양인 J2(BMP280 소켓)와 J7(GPS 커넥터)의 이름표도 그랬습니다 (고치기 전 보드 588fb390에 새 검사를 돌리면 이 셋이 FAIL: `out/reports/evidence_20261112/esp_user_fix_routing_rules_on_board_588fb390.json`).
- `scripts/silk.py`: 핀 줄과 나란히 적히는 이름표는 **그 면에서 본 패드 순서대로** 적습니다. 핀 줄과 직각으로 적히거나 두 줄짜리 헤더 옆에 적히는 이름표는 종전대로 1번 핀(네모난 패드)부터입니다.
- 바뀐 글자 3줄 (모두 아랫면): '1S BAT +-' → '1S BAT -+', 'BMP280 3V G SCL SDA 3V 3V' → 'BMP280 3V 3V SDA SCL G 3V', 'GPS V TX RX G' → 'GPS G RX TX V'. 세 글자열의 위치·크기·각도·층·글자 상자는 그대로이고, 보드의 다른 글자 40개와 도형, 풋프린트의 실크는 하나도 움직이지 않았습니다.
- 새 검사 `scripts/routing_checks.py` silkscreen.pin_order_labels: 핀 글자가 있는 이름표 14개를 보드에서 다시 재서 (글자 획의 위치와 패드 좌표) 모두 맞음. 윗면의 J8, J16, J17 이름표는 원래 맞았고 바뀌지 않았습니다.
- 그림 `out/render/a3_as_delivered_bottom.png` 에서 눈으로 확인: '1S BAT -+' 오른쪽에 둥근 패드, 그 오른쪽에 네모난 패드.

**파일 해시 (sha256 앞부분, 03:57 → 지금, M):**

| 파일 | 03:57 | 지금 | 달라진 것 |
|---|---|---|---|
| 보드 `kicad/vtol_carrier.kicad_pcb` | 588fb390b516894c | cc750ce9e23322b2 | 83474줄 중 6줄: 글자 3줄과 그 글자의 id 3줄. 커밋 6fc8ec3의 보드(131c4d63)와 비교하면 여기에 표면처리 표기 한 줄이 더해진 7줄 |
| 거버 zip (기본) | fc4a439eba571880 | f4c0e0bbd1ed460f | 13개 파일 중 아랫면 실크 `vtol_carrier-B_Silkscreen.gbo` 하나만 다름 |
| 거버 zip (with_mcu) | 39597ca679c84ca7 | 03c4bb5ec750a9a6 | 위와 같음 |
| BOM / CPL 4개 | 6dff9dbc / 8112abe6 / 43aea871 / 1c2503e9 | 같음 | 바뀌지 않음 |
| 배선 전 보드, 배치, 세션 파일, 회로도 8장, 규칙 파일, 사양, 계획 | fcbd2115 (배선 전 보드), c8309c9c (사양), c5b2ec8a (계획), a64b7bfe (규칙 파일) | 같음 | 바뀌지 않음 (이 줄과 위아래 줄의 파일을 합쳐 76개의 sha256을 앞뒤로 비교) |
| `out/off_board_purchase.csv`, `out/따로_살_부품_ko.md` | 18ad634e / 59aae80b | 같음 | 바뀌지 않음 (27줄) |

- pcbnew로 읽어 비교 (03:57 보드와, 그리고 커밋 6fc8ec3 보드와): 배선 조각 2557, 비아 442, 구리 면 6(채움 포함), 풋프린트 174, 패드 691개와 네트, 풋프린트의 그림·글자 2790개, 보드 도형 27개가 모두 같고, 보드 글자 43개 중 위의 3개만 글자열이 다릅니다. 근거: `out/reports/evidence_20261112/esp_user_fix_proof3_board_588fb390_to_cc750ce9.json` (결과 PASS).

**다시 돌린 검사 (M, 2026-11-12 04:26 ~ 05:24):**

| 검사 | 결과 | 숫자 |
|---|---|---|
| 회로도 ERC / 보드 DRC (kicad-cli 10.0.6, 보드 cc750ce9의 사본에서 따로 실행; DRC는 `--refill-zones` 를 넣은 실행도) | PASS | ERC 위반 0 (시트 8장); DRC 위반 0, 미연결 0, 회로도 불일치 0 |
| `make_all.py` 전체 빌드 | 종료 코드 2 (그대로) | 실패한 검사는 routing 하나. `rev3_checks.py all` / `export-files` / `export-determinism` PASS |
| 배선 규칙 (계획 13장) | **FAIL (그대로)** | 21개 중 16개 PASS. FAIL은 같은 5개 (battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance). silkscreen은 새 이름표 검사를 포함해 PASS |
| 결정성 `determinism.py --cross-folder` (마지막 실행 05:02:50 ~ 05:24:10) | **FAIL (STEP 파일 하나, 그대로)** | 바이트 비교 71개 중 다른 것 0개 (검사 전 디스크 = 빌드 1 = 빌드 2). 시각 줄을 뺀 비교 3개 중 1개 다름 (`out/vtol_carrier.step`; 번호·순서를 뺀 내용은 같음). 다른 폴더 비교 60개: 51개 바이트 동일, 9개는 폴더 이름을 바꿔 넣은 뒤 동일, 다른 것 0개 (보드·회로도·거버·zip·BOM·CPL 43개는 바이트 동일). 빌드 8.3분 / 9.0분 / 4.1분. 이 실행은 설명 문장 두 곳(`README_ko.md`, `spec/SPEC_ko.md` 의 '실크는 그대로' 문장)을 고친 뒤 다시 돌린 것이고, 그 전 실행(04:36:16 ~ 04:56:52)도 숫자가 같았습니다 (보드와 주문 파일은 두 실행에서 같은 해시) |
| JLC 재고 관문 `jlc_gate.py` | 다시 돌리지 않음 | BOM, CPL, 사양 파일이 바이트까지 그대로라서 2026-11-12T02:54:37+09:00의 결과(33/33줄 PASS)가 그대로 이 파일들의 것입니다. 주문 당일에는 다시 돌려야 합니다 |
| 출고 판정 | **NOT ORDERABLE (그대로)** | 이유: routing_rules FAIL 5개 |

- 주문 메모 두 개(321자 / 359자)와 주문 화면 입력표, 따로 살 부품 목록은 바뀌지 않았습니다.
- 주문 화면과 장바구니에는 여전히 맨 처음 올린 옛 파일이 있습니다. 지금 파일은 올려 보지 않았습니다 (NOT_VERIFIED).
- 바뀐 스크립트: `scripts/silk.py`, `scripts/routing_checks.py`, `scripts/docs_ko.py` (J14 문장, 이름표 읽는 법, 추정 문장), `scripts/checks_exports.py` (손납땜 고정비), `scripts/rev3_checks.py` (넷리스트 해시 기록), `scripts/rev3_spec.py` (`spec/SPEC_ko.md` 의 문장 하나; 사양 파일 `spec/carrier_spec.json` 은 c8309c9c 그대로). 입력 파일 `spec/lcsc_snapshot_20261112.json` 은 원본 응답의 경로와 sha256만 바뀌었습니다 (재고·단가 값은 그대로).
- 이 단계의 작업 기록: `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/impl/esp_user_fix/NOTES.md` (세션 임시 폴더). 커밋하지 않았습니다.

### 무엇이 바뀌었나

| 항목 | 전 (00:40) | 지금 | 표시 |
|---|---|---|---|
| ESP-12F (U7) | JLC 실장 | 사용자 납땜 (패드 22개). 두 거버 zip 모두 U7 패드에 솔더 페이스트 구멍 없음 | M |
| 스루홀 커넥터 12개 (J2, J5, J8~J12, J14~J18) | JLC 손납땜 (BOM 6줄) | 사용자가 사서 납땜 (납땜 39곳). GPS용 SMD 커넥터 J7은 그대로 JLC 실장 | M |
| JLC가 실장하는 것 (기본 묶음) | 157개, BOM 40줄 (SMT 34 + 스루홀 6), CPL 157행 | 144개, BOM 33줄 (SMT 33 + 스루홀 0), CPL 144행 | M |
| with_mcu 묶음 | 161개, 41줄 | 148개, 34줄 (더해지는 것은 U1~U4 = C6510뿐; U7과 스루홀은 이 묶음에도 없음) | M |
| 사양의 fit | jlc 157 / jlc_optional 4 / user 1 / none 12 | jlc 144 / jlc_optional 4 / user 14 / none 12 (부품 자리 174, 네트 105는 그대로) | M |
| 페이스트 패드 (기본 / with_mcu zip) | 342 / 598 | 320 / 576 (각각 22개 줄었고, 없어진 22개의 좌표 = U7 패드 22개의 중심) | M |
| 주문 방식 | Standard PCBA | Economic PCBA (주문 화면 'PCBA Type') | 사용자 결정 |
| 표면처리 | ENIG | 무연 HASL: 보드 파일과 거버 job 파일 표기 'HAL lead-free', 주문 화면에서는 'LeadFree HASL' | M |
| 베이킹 | 주문 메모로 요청 | 주문하지 않음 (주문 화면: Economic PCB Assembly에서는 고를 수 없음) | M (주문 화면 2026-11-12) |
| 비아 덮기 | (계획: Tented) | 주문 화면이 Plugged로 정함 (이 4층 기판에서 LeadFree HASL을 고르면 Tented가 막힘, 0 USD) | M (주문 화면 2026-11-12) |
| 레일 | Standard: JLC가 76 mm 변 두 곳에 5 mm 레일 | 없음. 'Tooling holes: Added by JLCPCB' - JLC가 기판 안에 구멍 자리를 찾아야 함 | M (화면 항목), 위치는 NOT_VERIFIED |
| 주문 메모 | 스루홀은 JLC가 납땜, J8~J12 / J14 / J16을 돌리지 말 것, 베이킹 요청 | SMT만 실장, 스루홀 부품을 달지 말 것, 비워 둘 자리 목록 (기본 321자, with_mcu 359자; 화면 한도 500자) | M |
| JLC 실장 부품 중 기판 끝에 가장 가까운 몸체 | U7 0.15 mm, J15 0.29 mm | L5 0.45 mm (그다음 U9 0.45, R60 0.537, C41 0.55, U6 0.55 mm). 0.3 mm보다 가까운 것 없음. U7과 J15는 사용자가 납땜하는 자리가 됨 | M |
| 따로 살 것 | 19줄 | 27줄. (a) JLC BOM에서 빠진 7종 (ESP-12F + 스루홀 커넥터 6종), (b) MCU, (c) 센서·기록기·GPS, (d) 배터리·스위치·부저, (e) 프로그래밍, (f) 장착 재료 | M |

### 파일 해시 (sha256 앞부분, 전 → 지금)

`[변경됨: 이 표의 '지금'은 03:57 시점입니다. 그 뒤 보드와 두 거버 zip의 해시가 한 번 더 바뀌었습니다 (보드 cc750ce9e23322b2, zip f4c0e0bbd1ed460f / 03c4bb5ec750a9a6) - 위 '04:05 ~ 05:30 추가' 절. BOM, CPL, 사양, 계획의 해시는 이 표 그대로입니다.]`

| 파일 | 전 (00:40) | 지금 | 달라진 것 |
|---|---|---|---|
| 보드 `kicad/vtol_carrier.kicad_pcb` | 131c4d63d10c5158 | 588fb390b516894c | 98번째 줄 `(copper_finish "ENIG")` → `(copper_finish "HAL lead-free")` 한 줄뿐 (M) |
| 배선 전 보드 `kicad/work/vtol_carrier_unrouted.kicad_pcb` | c94bd350abf67d8d | fcbd2115248d38f3 | 같은 한 줄뿐 (M) |
| 거버 zip (기본) | 5e7d74ecf66b8c5d | fc4a439eba571880 | 13개 파일 중 F.Paste와 job 파일만 다름 (M) |
| 거버 zip (with_mcu) | 63eb3f0234e53e6b | 39597ca679c84ca7 | 위와 같음 (M) |
| `out/jlcpcb_bom.csv` | fa4c8e2622885f4d | 6dff9dbcbcc55720 | 7줄 빠짐 (U7, J2/J5, J8~J12, J14, J15/J17, J16, J18), 나머지 33줄은 글자와 순서가 그대로 (M) |
| `out/jlcpcb_cpl.csv` | 738c4ea451000e0e | 8112abe695cb42e2 | 13행 빠짐 (같은 부품 13개), 나머지 144행 그대로 (M) |
| `out/jlcpcb_bom_with_mcu.csv` | 465199b917ff914d | 43aea871c5965ec2 | 7줄 빠짐, 나머지 34줄 그대로 (M) |
| `out/jlcpcb_cpl_with_mcu.csv` | b0ab3d4240db4473 | 1c2503e959057e7e | 13행 빠짐, 나머지 148행 그대로 (M) |
| 회로도 시트 8장, `kicad/vtol_carrier.kicad_pro` | - | 바뀌지 않음 | 8장 모두 커밋 6fc8ec3의 파일과 sha256이 같고, 규칙 파일은 a64b7bfe58b8532d 그대로 (M) |
| 사양 `spec/carrier_spec.json` | 457e9b800990c2c7 | c8309c9c27361d69 | fit 값과 설명 문장 |
| 계획 `spec/rev3_plan.json` | 415d1e20a0903103 | c5b2ec8a5fe386b3 | 키 `amendments` 에 두 변경이 옛 값·새 값과 함께 기록됨. 빠진 BOM 7줄은 `bom_lines_now_bought_by_the_user` 에 그대로 보관 |

### 구리가 그대로라는 증거 (M, 새 배선 없음)

- 보드 파일 글자 비교: 83474줄 중 다른 줄은 `copper_finish` 한 줄.
- pcbnew로 두 보드를 읽어 비교: 배선 조각 2557개, 비아 442개, 구리 면 6개(채워진 모양 포함), 풋프린트 174개(위치·회전·층), 패드 691개(번호·네트·위치·크기·층)가 모두 같음.
- 배선은 기록된 Freerouting 세션의 재생입니다. route.py는 배선 전 보드가 세션을 기록한 보드와 `copper_finish` 값만 다르고 나머지 바이트가 같을 때에만 재생을 받아들이도록 했고 (그 값을 비운 뒤의 sha256 86f3c006550c7fa4 이 기록된 보드와 같음), 이번 재생의 판정은 'byte-identical to the recorded unrouted board except the copper_finish value' 입니다.
- 두 거버 zip: 구리 4층, 마스크, 실크, 외곽, 드릴 등 11개 파일은 전과 바이트까지 같고, F.Paste(U7 패드 22개가 빠짐)와 job 파일(`"Finish": "ENIG"` → `"Finish": "HAL lead-free"`)만 다름.
- 근거 파일: `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/impl/esp_user/proof2.json` (결과 PASS). `[변경됨: 이 경로는 세션 임시 폴더입니다. 같은 파일의 사본을 out/reports/evidence_20261112/esp_user_proof2_board_131c4d63_to_588fb390.json 에 넣었습니다. 이 비교는 03:57 보드(588fb390)의 것이고, 지금 보드(cc750ce9)의 비교는 같은 폴더의 esp_user_fix_proof3_board_588fb390_to_cc750ce9.json 입니다.]`

### 지금 파일의 검사 결과 (2026-11-12 03:27 ~ 03:52에 다시 실행)

`[변경됨: 03:57 시점 파일의 결과입니다. 지금 파일(보드 cc750ce9)로 04:26 ~ 05:24에 다시 돌린 결과는 위 '04:05 ~ 05:30 추가' 절의 표에 있고, 판정은 모두 같습니다.]`

| 검사 | 결과 | 숫자 | 표시 |
|---|---|---|---|
| 회로도 ERC (kicad-cli 10.0.6, `--severity-all`, 사본에서 따로 실행) | PASS | 위반 0, 시트 8장 | M |
| 보드 DRC (`--severity-all --schematic-parity`, `--refill-zones` 를 넣은 실행도 같음; 사본에서 따로 실행) | PASS | 위반 0, 미연결 0, 회로도 불일치 0 | M |
| `rev3_checks.py all` / `export-files` / `export-determinism` (전체 빌드 안에서 실행) | PASS / PASS / PASS | fit 집합 = 계획, BOM = CPL = 사양, 두 zip은 F.Paste만 다름 (차이 256 = U1~U4 패드), JLC가 달지 않는 자리에 남은 페이스트 패드 0 | M |
| JLC 재고 관문 `jlc_gate.py` (인터넷, 읽기만) | PASS | 2026-11-12T02:54:37+09:00, 33/33줄, with_mcu PASS, 얇은 재고 줄 없음 | M (그 시각 기준) |
| 배선 규칙 (계획 13장) | **FAIL (그대로)** | 21개 중 16개 PASS. FAIL: battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance | M / E |
| 결정성 `determinism.py --cross-folder` (2026-11-12T03:27:37 ~ 03:51:49) | **FAIL (STEP 파일 하나 때문, 그대로)** | 바이트 비교 71개 중 다른 것 0개. 시각 줄을 뺀 비교 3개 중 1개 다름 (`out/vtol_carrier.step`; 번호·순서를 뺀 내용은 같음). 다른 폴더 비교 60개: 51개 바이트 동일, 9개는 폴더 이름을 바꿔 넣은 뒤 동일, 다른 것 0개 (보드·회로도·거버·zip·BOM·CPL 43개는 모두 바이트 동일). 빌드 10.3분 / 9.2분 / 4.7분 | M |
| 출고 판정 (`out/manifest.json`) | **NOT ORDERABLE (그대로)** | 이유: routing_rules FAIL 5개 | M |
| 그림 | 다시 만듦 | '받은 상태' 그림 a1~a3과 슬라이드 e1, e4에서 ESP-12F와 스루홀 커넥터가 JLC 실장으로 보이거나 적히지 않음. a1, a2, a3, e1, e4를 눈으로 확인 | M |

### 주문 화면에서 읽은 것 (M, 주문 화면 2026-11-12 01:30 ~ 02:06; 사용자의 로그인된 브라우저)

- 그때 올린 파일은 이 변경 전의 것(zip 5e7d74ec, BOM fa4c8e26, CPL 738c4ea4)이고 화면은 'Detected 4 layer board of 70x76mm', 부품 40줄 인식·40줄 확정으로 받아들였습니다. **지금의 새 파일은 아직 올려 보지 않았습니다 (NOT_VERIFIED). 주문 화면과 장바구니에 남아 있는 것은 옛 파일이므로, 주문하려면 새 파일 3개를 다시 올려야 합니다.**
- 견적 (PCB 5장 / 실장 2장, 배송 제외, USD; ESP-12F는 실장 목록에서 뺀 상태):

| 조건 | PCB | PCBA | 합계 | 조립 기간 |
|---|---|---|---|---|
| **이번 주문: Economic, 무연 HASL, 스루홀 없음** | 13.10 | 63.72 | **76.82** | 3-4일 |
| Economic, 무연 HASL, 스루홀 6줄을 JLC가 납땜 | 13.10 | 70.05 | 83.15 | 3-4일 |
| Standard, 무연 HASL, 스루홀 없음 | 13.20 | 111.64 | 124.84 | 3-4일 |
| Standard, ENIG, 처음 올린 그대로 (ESP-12F와 스루홀 포함) | 25.30 | 125.37 | 150.67 | 5-6일 |

- 이번 주문의 PCBA 63.72 = 셋업 8.24 + 스텐실 1.55 + 부품 33줄 20.32 + Extended 부품 수수료 30.90 + SMT 장착 1.40 + 부품 위치 확인 0.45 + 질소 리플로 0.86.
- Economic 화면은 ESP-12F를 뺀 39줄을 모두 받아들였습니다 ('Standard only' 안내 없음). Bake Components, Photo Confirmation, Function test는 Economic에서 고를 수 없습니다. 메모 칸은 500자 한도이고 'Quote after review' 입니다.
- 스루홀 커넥터 6종을 LCSC에서 2장 분 사면 약 4.56 USD, ESP-12F 2개까지 합치면 약 10.39 USD입니다 (E; 단가와 재고는 2026-11-12T02:50:37 ~ 02:51:09의 LCSC 공개 기록 M, 최소 주문 수량 반영, 배송비 제외).

### 사용자가 새로 하게 된 일

- ESP-12F와 스루홀 커넥터 6종(보드당 12개)을 따로 삽니다: `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/따로_살_부품_ko.md` (부품 번호, 수량, 어디에 쓰는지, LCSC 재고·가격).
- 납땜 순서 (`ORDER_ko.md` 7장): 검사 → J14·J15 납땜 → MCU·ESP 없이 전원 시험 → 부하 시험 → MCU → ESP-12F → 나머지 커넥터 10개(낮은 것부터) → 뒷면 다리 자르기 → 모듈 → 프로그래밍. J5 소켓은 U2 패드에서 0.73 mm, J8·J9는 ESP 패드에서 4.73 mm라서 MCU와 ESP를 먼저 답니다 (M).
- **J14(배터리 커넥터)를 거꾸로 달면 배터리 극성이 뒤집힙니다.** 1번 핀(네모난 패드)이 +입니다 (M).
- 뒷면 다리 1.5 mm 규칙은 이제 직접 납땜한 모든 스루홀에 적용됩니다: J15, J17은 반드시 자르고 (도면값 1.8 mm, F), 나머지도 재 보고 자릅니다.

### 이번에 새로 찾은 것 (고치지 않음)

`[변경됨: 04:05 이후 고쳤습니다. 지금 라벨은 '1S BAT -+' 이고 아랫면에서 본 패드 순서와 맞습니다 - 위 '04:05 ~ 05:30 추가' 절. 아래 문장은 03:57 시점 기록입니다.]`

- **아랫면 실크의 '1S BAT +-' 라벨은 글자 순서가 핀 위치와 맞지 않습니다.** 아랫면에서 보면 J14의 네모난 패드(1번 핀 = +)가 오른쪽인데 라벨은 '+' 가 왼쪽입니다 (M: 패드 좌표, 그림 a3에서 눈으로 확인). 라벨은 이름표로 넣은 것이지만, 이제 사용자가 아랫면에서 이 커넥터를 직접 납땜하므로 헷갈릴 수 있습니다. 실크를 바꾸면 보드 파일이 바뀌므로 이번 단계에서는 고치지 않고 `ORDER_ko.md` 7장 3번에 '글자 순서가 아니라 네모난 패드 = +' 로 적었습니다. 고칠지는 사용자가 정합니다.

### 여전히 열려 있는 것 / NOT_VERIFIED

- 출고 판정 NOT ORDERABLE: 배선 규칙 5개 FAIL을 받아들이고 주문할지는 사용자 결정 (바뀌지 않음, 6장의 추가 1, 2).
- Economic 주문에서 JLC가 기판 안(윗면 부품 자리 점유율 88.5 %, M)에 툴링 홀 자리를 찾는지, 구리를 건드리지 않는지, 아니면 레일(= Standard)을 요구하는지.
- 지금 파일의 업로드·BOM 매칭·견적, 메모 검토 뒤 금액, DFM 결과.
- 리플로: Economic은 최고 255±5 C (F)이고 C2833309 (C47), C2286 (LED D1~D4), C53426685 (L5)는 정격 260 C로 여유가 없습니다. C47은 리플로 한 번만 허용 (F).
- 사용자가 산 ESP-12F와 스루홀 커넥터가 패드·구멍에 맞는지, 무연 HASL 패드에서의 손납땜, J14 구멍 지름과 다리 길이.
- 결정성 검사의 STEP 파일 FAIL (그대로).

### 바뀐 스크립트와 문서

- 사양: `scripts/build_spec.py` (라이브러리 풋프린트가 스루홀인 실장 부품은 fit user), `scripts/rev3_spec.py`, `spec/rev3_plan.json` (변경 2건), `spec/carrier_spec.json`, `spec/SPEC_ko.md`, `spec/parts_sources.json`, `spec/REV3_PLAN.md` (맨 위 안내문).
- 보드·배선: `scripts/build_pcb.py` (표면처리 표기), `scripts/route.py` (재생 보호 장치), `kicad/work/ses_replay.json`.
- 내보내기·검사: `scripts/paste_variant.py`, `scripts/checks_exports.py`, `scripts/rev3_checks.py`, `scripts/edge_distance.py`, `scripts/fit_checks.py`, `scripts/cpl_rotation.py`, `scripts/make_manifest.py`, `scripts/determinism.py`.
- 문서·그림: `scripts/docs_ko.py`, 새 파일 `scripts/purchase_list.py`, `scripts/render_rev3.py`, `out/render/INDEX_ko.md`.
- 새 입력 파일 (손으로 적거나 한 번 읽어 온 기록): `spec/jlc_order_page_20261112.json` (주문 화면 기록), `spec/lcsc_snapshot_20261112.json` (LCSC 공개 상품 기록, 읽기만 함).
- 새 문서: `out/따로_살_부품_ko.md`.
- 이 단계의 작업 기록: `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/impl/esp_user/NOTES.md`.
- 커밋하지 않았습니다. 변경은 커밋 6fc8ec3 위의 작업 폴더에만 있습니다.

## 1. 한눈에 보기

| 항목 | 상태 |
|---|---|
| 도구의 출고 판정 | **NOT ORDERABLE** (배선 규칙 검사 21개 중 5개 FAIL: battery_path, hot_loop, input_cap_l5, sw_node, crystal_distance) |
| 주문 파일 | 모두 만들어져 있음 (거버 zip, BOM, CPL 두 묶음). 형식·내용 검사 PASS (M) |
| 회로도 ERC / 보드 DRC | 오류 0 / 위반 0, 미연결 0, 회로도 불일치 0 (M, kicad-cli 10.0.6) |
| JLC 부품 재고 | 2026-11-12T00:08:01+09:00 기준 40줄 모두 PASS (M, 그 시각 기준). 주문 당일 다시 확인해야 함 `[변경됨: 지금 BOM은 33줄, 2026-11-12T02:54:37+09:00 기준 33줄 모두 PASS]` |
| 사용자가 정할 것 | (1) 배터리 경로 폭 FAIL과 승압부 배치 FAIL을 알고 그대로 주문할지, 고친 뒤 주문할지 (2) 6장의 D1~D17 |
| 결제 전 관문 | **JLC의 DFM 결과와 견적을 보기 전에는 결제하지 마십시오** (7장) |

도구 판정이 NOT ORDERABLE인 것은 파일이 깨졌다는 뜻이 아니라, 계획서가 정한 전원부 배선 규칙 5개를 이 보드가 지키지 못했고 그 규칙을 느슨하게 하지 않았다는 뜻입니다. 그대로 주문할지는 사용자가 정합니다 (`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 0장).

## 2. 무엇을 만들었나 (요청과 비교)

요청: "mcu가 장착될 pcb를 다시 설계하고 센서 장착 렌더링도 함께, 모든 부품은 JLCPCB 부품 라이브러리에 실제로 있고 주문 가능한 것으로, MCU와 센서를 빼고 모두 JLCPCB BOM 양식으로 써서 최종 파일과 BOM만 보내면 SMT 실장된 PCB를 받도록."

| 요청 | 결과 | 표시 |
|---|---|---|
| PCB 재설계 | 76 x 70 mm, 4층, 부품 자리 174개, 네트 105개. MCU 4개의 핀 256개는 rev 2c와 같은 네트 (차이 0). 외곽, 커넥터·MCU·크리스털·모듈 소켓·ESP 위치는 rev 2c 그대로. 꽂는 5 V 전원 모듈(Pololu)을 JLC 부품으로 만든 승압 회로(U8 TPS61023 + L5)와 저전압 차단(U9)으로 바꿈. Wi-Fi 모듈은 ESP-12F | M |
| 센서 장착 렌더링 | 그림 20장: 받은 상태 3장, 장착 뒤 4장, 확대 6장, 기체 안 GPS 1장, 설명 슬라이드 6장. 모듈 모양은 대부분 가정 치수(그림에 빨간 '가정' 표시) | M (그림), A (모듈 치수) |
| 모든 부품이 JLC에 있고 주문 가능 | BOM 40줄 모두 JLC 부품 번호가 있고, 2026-11-12 00:08에 JLC 부품 기록을 읽어 재고·주문 가능을 확인 (PASS 40/40). 재고가 적은 줄: C22436147 (J2, J5 소켓) 523개 `[변경됨: 지금 BOM 33줄, 02:54 기준 PASS 33/33, 얇은 재고 줄 없음. C22436147은 이제 사용자가 사는 부품]` | M (그 시각 기준) |
| MCU·센서 빼고 모두 JLC BOM | JLC 실장 157개 (SMT 34줄 + 스루홀 6줄). BOM·CPL은 JLC 견본 파일과 같은 머리줄 `[변경됨: 사용자 결정으로 ESP-12F와 스루홀 커넥터 12개도 JLC BOM에서 빠짐. 지금 JLC 실장 144개 (SMT 33줄, 스루홀 0줄)]` | M (형식), JLC 수락은 NOT_VERIFIED |
| 파일만 보내면 실장된 PCB | 올릴 파일 3개(거버 zip, BOM, CPL)가 준비됨. 다만 요청과 달리 사용자 몫으로 남는 일이 있음 (5장) | - |

요청대로 되지 않은 점 (숨기지 않고 적음):

- 프로그래밍 단자 8곳(J3, J19~J22, J24~J26)은 빈 구멍으로 남습니다 (A안). JLC가 실장하는 B안은 자리가 없어 불가능했습니다 (4장).
- 스루홀 커넥터 6줄을 JLC가 손납땜해 주는지는 견적에서만 확인됩니다. `[변경됨: 스루홀 커넥터는 JLC가 달지 않고 사용자가 사서 납땜합니다. 주문 화면의 견적에는 손납땜 줄이 실제로 있었습니다 - '2026-11-12 변경' 절]`
- 전원부 배선 규칙 5개가 FAIL입니다 (3장 표, 8장).

## 3. 최종 재검사 결과 (이 단계에서 디스크의 파일로 다시 실행, 2026-11-12)

`[변경됨: 이 장은 00:08 ~ 00:40의 기록입니다. 그때의 파일(보드 131c4d63, BOM 40줄 / 157개, 페이스트 패드 342 / 598)에 대한 결과이고, 지금 파일의 재검사 결과는 '2026-11-12 변경' 절에 있습니다.]`

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

`[변경됨: 아래 두 표의 경로는 그대로지만 파일은 새로 만든 것으로 바뀌었고, 표의 sha256 앞부분은 00:40 시점의 옛 파일 것입니다. 지금 파일의 sha256은 '2026-11-12 변경' 절의 표에 있습니다. 묶음 1은 이제 'MCU, ESP-12F, 스루홀 커넥터를 사용자가 납땜', 묶음 2는 'MCU만 JLC가 추가로 실장'이고, with_mcu BOM은 34줄입니다.]`

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
- 따로 사야 하는 것 19줄 (MCU, 센서 모듈, 케이블, 배터리 등; 재고·가격은 NOT_VERIFIED): `B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/off_board_purchase.csv` `[변경됨: 지금 27줄 (ESP-12F와 스루홀 커넥터 6종이 더해짐). 읽기 쉬운 목록 B:/0)AI_Cad/CanSat_v0.9/08_PCB/00_주문파일/따로_살_부품_ko.md]`
- 주문 안내서 안에 적힌 경로는 작업 폴더(`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/`) 기준입니다. 결과물 폴더 `B:/0)AI_Cad/CanSat_v0.9/08_PCB/` 의 같은 하위 경로에 sha256이 같은 사본이 있습니다 (`B:/0)AI_Cad/CanSat_v0.9/08_PCB/copy_sha256.json`).

주문 설정의 기본값 (계획 D9, D11, D16; 화면 항목 이름은 NOT_VERIFIED): 4층, 1.6 mm, ENIG, 녹색, PCB 5장 중 2장 실장, PCBA Standard, 윗면만 실장, 레일은 JLC가 붙임, 부품 위치 확인(Confirm Parts Placement) 켬. 비용 추정(E): 기본 묶음 Standard 2장 122.56 USD (Standard 전용 수수료가 붙으면 154.41 USD). PCB 제작비, ENIG, 베이킹, 배송, 세금은 빠져 있고 실제 금액은 JLC 견적뿐입니다. `[변경됨: 지금 기본값은 Economic PCBA, 무연 HASL (LeadFree HASL), 레일 없음 (Tooling holes: Added by JLCPCB), 베이킹 없음. 화면 항목 이름과 견적은 주문 화면에서 읽었습니다 (M) - '2026-11-12 변경' 절]`

## 5. 프로그래밍 단자 A안 / B안, 그리고 사용자가 직접 해야 하는 일

프로그래밍 단자 (계획 D6):

- **A안 (지금 파일)**: MCU 굽기 단자 J19~J22와 ESP 굽기 단자 J3 (2x3), 리셋 점퍼 J24~J26 (1x2)은 케이블타이 띠 안의 **빈 도금 구멍**입니다. JLC는 여기에 아무것도 달지 않습니다. 사용자가 핀헤더를 납땜(36핀)하거나 포고핀 어댑터를 대고 굽습니다 (포고핀 접촉은 NOT_VERIFIED). 헤더를 납땜했다면 기체에 붙이기 전에 떼거나 잘라 띠를 비워야 합니다.
- **B안 (JLC가 헤더를 띠 밖에 실장)**: 배치 검색에서 놓을 자리가 0곳이었습니다 (M). 고정 부품만 남겨도 2x3 다섯 개가 들어가지 않아, 커넥터나 MCU 위치를 옮기지 않고는 불가능합니다. 그래서 B안 파일은 만들지 않았습니다.

MCU와 센서 말고도 사용자 몫으로 남는 일 `[변경됨: 아래 목록은 00:40 시점 것입니다. 지금은 여기에 'ESP-12F 구매와 납땜 (패드 22개)'과 '스루홀 커넥터 12개 구매와 납땜 (39곳)'이 더해지고, 2번의 소켓 J2 / J5도 사용자가 납땜하며, 3번의 레일 떼기는 Economic 주문에는 없고, 4번의 다리 자르기는 직접 납땜한 모든 스루홀 다리가 대상입니다. 지금 목록과 순서: ORDER_ko.md 7장, 8장]`:

1. (묶음 1이면) ATmega128A-AU 4개 납땜. U1을 GY-521보다 먼저 납땜합니다.
2. 센서·모듈 장착: GY-521을 J1 구멍에 납땜, BMP280을 J2 소켓에, OpenLog에 직각 6핀 헤더를 납땜해 J5 소켓에, NEO-7M은 기체 윗뚜껑에 두고 JST-GH 케이블로 J7에. 그림: `B:/0)AI_Cad/CanSat_v0.9/08_PCB/out/render/e1_delivered_vs_completed.png` 과 같은 폴더의 e2a, e2b, e2c, e3, e4.
3. Standard 주문의 공정 레일 떼기.
4. 뒷면 다리 자르기: J15, J17은 도면값 1.8 mm로 규칙 1.5 mm를 넘어 반드시 자릅니다 (다리 4개). 다른 핀헤더·소켓도 재 보고 넘으면 자릅니다. J1, J14 다리 길이는 확인하지 못했습니다.
5. 프로그래밍 단자 준비 (위 A안).
6. GPS 케이블, 스위치·부저·아밍 리드, 배터리 준비 (`off_board_purchase.csv`).
7. 처음 켜기 전 벤치 시험과 기록 (켜짐·꺼짐 전압, +5V, +3V3, 부하 시험, 오실로스코프로 SW·+5V·VIN 파형, 배터리에서 U8 입력까지 저항): `ORDER_ko.md` 7장. 전원부 규칙 FAIL 때문에 이 측정이 특히 중요합니다.
8. 폼테이프와 케이블타이로 장착. 아랫면 실크의 두 줄 글자는 폼테이프 밑에 가려집니다.

## 6. 사용자 결정 D1~D17 (계획의 기본값으로 만들었음)

`[변경됨: 아래 표는 00:40 시점의 기본값입니다. 2026-11-12 사용자 결정으로 D9 = Economic, D14 = 베이킹 주문 안 함 (Economic에서는 고를 수 없음), D16 = 무연 HASL (비아 덮기는 주문 화면이 Plugged로 정함)이 되었고, D3 / D5 / D7 / D12에 나오는 스루홀 부품(J2 / J5 소켓, J8~J12 헤더, J15 / J17)은 JLC가 아니라 사용자가 사서 납땜합니다. D10의 툴링 홀은 레일이 아니라 JLC가 기판 안에 뚫습니다 (위치 NOT_VERIFIED). 지금 표: ORDER_ko.md 9장]`

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

**JLC의 DFM 결과와 견적을 받기 전에는 결제하지 마십시오.** 순서 `[변경됨: 아래 3~6번은 00:40 시점 것입니다. 지금은 3번 = 33줄(또는 34줄) 매칭, 4번 = 견적에 손납땜·수작업 조립 줄이 없어야 함 (스루홀을 JLC가 달지 않음), 5번의 J8~J12 / J14 / J16 문장은 해당 없음 (CPL에 없음; U7과 스루홀 자리가 비어 있는지 확인), 6번 = 툴링 홀 위치와 가장자리 부품 (JLC 실장 중 가장 가까운 것 L5 0.45 mm, M). 지금 확인표: ORDER_ko.md 3장]`:

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
- 가장자리 2.5 mm 안에 부품 몸체 34개 (JLC 실장 28개). U7 0.15 mm, J15 0.29 mm는 0.3 mm보다 가깝습니다 (rev 2c와 같은 위치). `[변경됨: 자리는 그대로 34곳이지만 JLC 실장은 15개입니다. U7과 J15는 이제 사용자가 납땜하는 자리이고, JLC가 실장하는 몸체 중 0.3 mm보다 가까운 것은 없습니다 (가장 가까운 것 L5 0.45 mm, M)]`
- 모듈 끼움: 가정 치수로는 충돌 0건이지만, OpenLog 아랫면 부품을 1.2 mm로 잡는 보수적 경우 U6 -0.5 mm, C3 -0.05 mm가 겹칩니다 (A).
- CPL 회전 4개(C45, C47, C52, U9)는 자료가 서로 어긋납니다.
- J14 (Molex) 3D 모델이 KiCad 라이브러리에 없고, 구멍 지름을 제조사 도면과 대조하지 못했습니다.
- 아랫면 실크에 저장소 주소와 연락용 이메일 두 줄이 인쇄됩니다 (윗면에 빈자리 없음). 주소 글자는 `scripts/silk.py` 와 보드 파일 4개에만 들어 있습니다.
- 검토 기록: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/reports/review_findings_rev3.md` (독립 검토 지적과 처리).

## 9. 확인되지 않은 것 (NOT_VERIFIED)

- 보드 동작 전체: 5 V 출력, 차단 전압, 전류 여유, 발열, 배터리 경로 저항, Wi-Fi 송신 중 전압. 모두 데이터시트 계산(E)이고 실측이 없습니다.
- JLC의 DFM 결과, 견적, BOM / CPL 업로드 미리보기, 스루홀 손납땜 수락, 레일 붙이는 방식, 가장자리 부품 수락, 실크 인쇄 품질, 실제 비용. `[변경됨: 견적과 업로드 형식은 이전 파일로 주문 화면에서 읽었습니다 (M). 스루홀 손납땜과 레일은 이번 주문에 없습니다. 새로 NOT_VERIFIED인 것: Economic 주문에서 JLC가 기판 안에 툴링 홀 자리를 찾는지, 지금 파일의 업로드·매칭, 사용자가 산 스루홀 커넥터와 ESP-12F가 구멍·패드에 맞는지]`
- 주문 당일의 재고 (지금 기록은 2026-11-12T00:08:01+09:00 기준).
- CPL 회전: C45, C47, C52, U9는 근거가 어긋남. 나머지도 JLC 미리보기로만 확정됩니다. JLC의 0도 기준이 EasyEDA 풋프린트와 같다는 것도 가정입니다.
- 모듈 치수와 끼움 (GY-521, BMP280, OpenLog, NEO-7M): 하나도 재지 않았습니다. BMP280 헤더 종류(직각/일자)와 핀 인쇄 순서, 모듈 핀 순서, GPS 선 색도 가정입니다. 그림은 맞춤 검사가 아닙니다.
- 기체 안 GPS 케이블 경로 (그림은 예시, 간섭 검사 없음).
- 리플로 온도 정격: 문서에서 찾지 못한 부품 번호가 32개입니다. Economic 방식에서는 C2833309, C2286, C53426685가 한계에 걸립니다. `[변경됨: 지금 31개 (ESP-12F가 빠짐). 주문이 Economic이 되었으므로 세 부품의 '한계'는 이제 실제로 해당합니다]`
- 대체 부품(J2 / J5, J7)의 풋프린트 호환.
- 따로 살 부품의 재고와 가격. `[변경됨: LCSC 번호가 있는 품목은 2026-11-12 02:50의 LCSC 공개 기록을 읽었습니다 (M, 그 시각 기준). 그 밖의 쇼핑몰은 여전히 NOT_VERIFIED]`
- 뒷면 다리 길이: J1, J14.
- 다른 PC나 다른 KiCad 판에서 다시 만들어도 같은 파일이 나오는지 (이 PC의 두 폴더에서만 확인).
- 새로 자동 배선을 돌렸을 때 같은 결과가 나오는지 (기록된 세션을 다시 넣는 방식만 재현됨).

## 10. 파일 위치와 다시 만드는 방법

- 결과물 폴더: `B:/0)AI_Cad/CanSat_v0.9/08_PCB/` (안내 `B:/0)AI_Cad/CanSat_v0.9/08_PCB/README.md`). 주문 파일은 `00_주문파일/`, 그림은 `out/render/` 와 `out/presentation/`, KiCad 프로젝트는 `kicad/`, 사양은 `spec/`, 스크립트는 `scripts/`. 이전 판(rev 2c, 2026-11-05)은 지우지 않고 `B:/0)AI_Cad/CanSat_v0.9/08_PCB/이전_rev2c_2026-11-05/` 로 옮겼습니다.
- 작업 폴더 (원본, git 브랜치 `pcb-rev3`): `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/`
- 그림 목록과 설명: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render/INDEX_ko.md`
- 사양 요약: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/SPEC_ko.md`
- 다시 만들기 (작업 폴더의 `scripts` 에서, 환경변수 `PYTHONDONTWRITEBYTECODE=1`): `B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py` (약 10분, 배선 규칙 FAIL 때문에 종료 코드 2가 정상). 결과물 폴더의 사본은 보관용이고, 거기서 다시 만들면 사양 파일이 폴더 이름을 담고 있어 JLC 재고 기록이 '다른 사양'으로 판정됩니다.
- 이 단계에서는 커밋하지 않았습니다. 바뀐 파일은 작업 폴더에만 있습니다 (브랜치 `pcb-rev3`, 마지막 커밋 812a8ba 이후의 변경). `[변경됨: 00:40 시점의 상태는 그 뒤 커밋 6fc8ec3 이 되었습니다. '2026-11-12 변경' 절의 내용은 6fc8ec3 이후의 커밋되지 않은 변경입니다]`
