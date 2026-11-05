# Fusion 360 스크립트 — `CanSatEDFRocket` (지금 0.9.8; 아래 설명의 첫 부분은 v0.8.1 기준)

`docs/SPEC.md` 1–6절의 전체 어셈블리를 Fusion 360 안에서 자동 생성하고 STL/STEP/f3d/미리보기/질량특성
보고서를 내보내는 API 스크립트입니다. 스크립트 판은 `SCRIPT_VERSION = "0.8.1"`이고, 실행 로그 첫 줄과 보고서의
`generator_version`에 찍힙니다. 모든 치수는 `params.json` → `cad/layout.py`에서 읽으며, 스크립트
자체에는 치수가 없습니다 (API 단위는 cm이므로 모든 mm 값은 `cm()`을 거칩니다).

## 설치

```bash
.venv/Scripts/python.exe tools/install_fusion_script.py
```

`%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\CanSatEDFRocket\` 에 `CanSatEDFRocket.py`,
`CanSatEDFRocket.manifest`, `project_root.txt`(프로젝트 절대 경로)를 복사합니다. 스크립트를 수정한
뒤에는 같은 명령으로 다시 설치(갱신)하면 됩니다. `--uninstall` 로 제거합니다.

## Fusion 360에서 실행

1. Fusion 360 → **유틸리티(Utilities)** 탭 → **추가 기능(Add-Ins)** → **스크립트(Scripts)** 탭.
2. `CanSatEDFRocket` 선택 → **실행(Run)**. (스크립트 폴더가 목록에 없으면 녹색 **+** 로 위 폴더를 추가합니다.)
3. 새 디자인 문서가 열리고(파라메트릭, 표시 단위 mm) 컴포넌트 45개(v0.8.1; v0.8은 44개, v0.6.3은 41개)가 하나씩 생성됩니다.
   BUDGET 프리셋은 피처 273개, 스케치 236개입니다. 이 PC(Fusion 2705.1.30)에서는 151.8 s 걸렸습니다
   (2026-10-15 22:49 실행). 끝나면 메시지 상자 하나로 성공/실패를 요약합니다.
4. 생성된 문서는 저장되지 않습니다(필요하면 직접 저장). 재실행하면 새 문서가 또 열립니다.

실행 중 각 피처 생성은 `cad/exports/fusion/run.log`에 기록됩니다 (컴포넌트, 피처 종류, 좌표/치수,
컴포넌트별 소요 시간, 실패 시 전체 traceback). Fusion이 멈춘 것처럼 보이면 이 파일을 보면 진행 상황을
알 수 있습니다.

## 실행 로그에서 확인할 것

BUDGET 프리셋(`edf.mount` = tabs)이면 `run.log`에 다음 줄이 있어야 합니다 (2026-10-15 실행의 실제 줄):

```
INFO  CanSatEDFRocket.py v0.6.3 started 2026-10-15T22:49:26
INFO  propulsion_module: tab mount 21/21 required features, 12/12 tab-screw cuts
INFO  propulsion_module: servo lead route 17/17 required cuts; point containment unanswered for 0
INFO  intake_module: band + windows 5/5 required features; point containment unanswered for 0
INFO  intake_module: wire path 10/10 required features
INFO  payload_module: COM mount 8/8 required features; point containment unanswered for 0
INFO  finished in 151.8s: 41/41 components OK, 0 failed, 0 optional features skipped
```

* 필수 피처 줄은 `성공 수/전체 수`입니다. 두 수가 같아야 합니다.
* `point containment unanswered for 0`: 절삭 판정에 쓴 점마다 Fusion이 안/밖을 답했다는 뜻입니다. 0이 아니면 그 피처는
  확인되지 않은 형상입니다. 이때 `tools/make_print_files.py`는 이 내보내기로 출력 파일을 만들지 않습니다
  (`--allow-unanswered`를 주면 만들지만 출력 계획에 그 목록이 남습니다).
* 마지막 줄은 `45/45 components OK, 0 failed`여야 합니다 (v0.8.1; 아래 로그 예시는 v0.6.3 실행이라 41/41).
* 보고서(`fusion360_report.json`)에서도 확인합니다: `failed_components` = `{}`, `soft_failures` = `[]`,
  `derived.extra`의 `servo_leads`, `intake_windows`, `com_mount` 각각 `containment_unanswered` = `[]`, `failed` = `[]`.
  `intake_servo_leads`에는 `failed` 키가 없고 `containment_unanswered` = `[]`만 있습니다. 흡입 모듈 서보 통로의 피처(볼록부, 통로,
  선반 홈, 바닥 쐐기)는 전선 통로와 같은 필수 피처 묶음으로 세므로 `intake_wire`의 `failed` = `[]`(10/10, 로그 "wire path 10/10")로
  확인합니다. 2026-10-15 실행은 모두 그렇습니다.
* KAIST 프리셋(`lip_clamp`)에는 탭 고정, 서보 리드, 전선 통로 줄이 없습니다. 이 피처들은 탭 고정 프리셋에서만 만듭니다.
  흡입 창과 COM 받침은 두 프리셋 모두 만듭니다.

## 내보내는 파일 (`cad/exports/`)

| 경로 | 내용 |
|---|---|
| `fusion360_report.json` | SPEC 6절 스키마의 질량특성 보고서 (`params_sha256`, 부품별 체적/밀도/질량/bbox/무게중심/관성텐서, totals, post_ejection, derived) |
| `fusion/stl/<name>.stl` | 출력 부품별 바이너리 STL (MeshRefinementHigh), 차량 좌표계 그대로. `vane.stl`/`vane_horn.stl`은 1번 베인/혼에서 추출 |
| `fusion/CanSatEDFRocket.step` | 루트 컴포넌트 STEP |
| `fusion/CanSatEDFRocket.f3d` | Fusion 아카이브 |
| `fusion/preview.png` | 뷰포트 이미지(fit 후 1600×1200) |
| `fusion/run.log` | 실행 로그 |

출력 파일(`cad/exports/print/`)은 `tools/make_print_files.py`가 `fusion/stl`에서 만듭니다. 이 도구는 보고서가 지금의
`params.json`(같은 해시), 같은 스크립트 판, 같은 프리셋에서 나왔고 실패한 컴포넌트와 답 없는 점이 없을 때만 씁니다.
`python tools/make_print_files.py --check`로 출력 파일이 최신인지 확인합니다.

보고서의 추가 키(스키마 확장): `failed_components`(실패한 컴포넌트와 첫 오류 줄), `soft_failures`
(선택적 피처의 실패 목록: 베인 모서리 필렛, 추진 모듈 외 모듈 결합 구멍, 서보 귀·통풍·케이블 구멍), `timings_s`, `exports`,
`derived.extra.tab_mount`(v0.5 탭 고정: 러그 Z 범위, 러그 각도, 카운터보어 t 범위, 성공한 나사 구멍 절삭 수 / 전체,
성공한 피처 수 / 전체, 실패 목록), `derived.extra.intake_wire`(흡입 모듈 전선 통로 피처 성공 수 / 전체, 실패 목록),
`derived.extra.tray_stop`(v0.5d 트레이 멈춤 블록), `derived.extra.prop_joint_holes`(v0.5d 추진 모듈 막힌 결합 구멍),
`derived.extra.servo_leads`(v0.6 서보 리드 경로: 포드·기둥 각도, 리드 질량과 무게중심, 성공 수 / 전체, 답 없는 점),
`derived.extra.intake_servo_leads`(v0.6 흡입 모듈 서보 통로·격벽 구멍), `derived.extra.intake_windows`(0.6.1 창 띠 두께·지주·
흡입 면적비, 0.6.2 창 아래 끝 `window_bottom_end` 박공), `derived.extra.com_mount`(v0.6 COM 보드 보스 4개와 막힌 구멍),
`derived.extra.nose_shell_method`, `derived.inertia_offdiag_convention`
(실행 시 시험 정육면체로 측정한 Fusion 관성곱 부호 규약: `product_integral`/`tensor`/`unknown`),
`derived.excluded_from_totals = ["launch_pad"]` (발사대는 차량 질량에서 제외).

다음 피처는 모두 필수(`CB.required`)입니다: v0.5 탭 고정과 전선 통로(라이너, 러그, 키 홈, 탭 나사 구멍 12개, 전선 홈,
볼록 통로, 통로 공간, 선반 홈, 45° 통로 바닥, 격벽 전선 구멍), v0.5d 피처(트레이 멈춤, 추진 모듈 막힌 결합 구멍),
v0.6 서보 리드 경로(포드 창, 기둥 슬롯, 데크 홈, 스피곳 홈, 흡입 서보 통로·바닥 쐐기·격벽 구멍), v0.6 COM 받침(보스 4개,
막힌 구멍 4개), 0.6.1 흡입 창 띠 두께와 창 절삭, 0.6.2 창 아래 끝 박공. 하나라도 실패하거나 부품 질량 계산이 실패하면
그 컴포넌트가 `failed_components`에 들어가고, 시뮬레이션과 크기 산정은 그 보고서를 쓰지 않습니다
(`sim/vehicle.py`, `sizing/sizing.py`; 부품이 빠진 보고서도 거부합니다).

작은 절삭은 부피 차이 대신 점 판정(`BRepBody.pointContainment`)으로 확인합니다. 지워질 곳의 점은 절삭 뒤 몸체 밖에,
남아야 할 벽의 점은 안에 있어야 합니다. 탭 나사 구멍(24.5 mm³), COM 받침 막힌 구멍(Ø2.5), 서보 리드 절삭,
흡입 창 절삭이 이 방식입니다. Fusion의 부피 값은 정확도가 정해져 있지 않아(가장 높은 정확도에서도 이 부품 기준 약
±34 mm³) 이런 작은 구멍을 부피 차이로 판정할 수 없기 때문입니다. Fusion이 답하지 못한 점은 `containment_unanswered`에
남고, 그때만 부피 감소로 대신 판정합니다. 모의 실행(`tools/fusion_mock_run.py`)에는 이 API가 없어 부피 감소로 대신 판정합니다.

Fusion 각도 평면 주의: `setByAngle`로 만든 기울어진 작업 평면은 45° 배수가 아닌 각도(예: 13.13°)에서 실기 실행이
실패했습니다. 부채꼴 형상은 수평 스케치(`CB.sector_z`)로 만들고, 스케치 호는 모서리 점을 먼저 만든 뒤 연결합니다
(Fusion은 호를 반시계 방향으로 저장해 시작·끝 점이 바뀔 수 있음). 0.6.2 박공 창은 부채꼴 도구와, 창 중심 + 90°
수직 평면(BUDGET에서는 135/225/315/45°)에 그린 육각형을 교차(Intersect)해서 만든 뒤 본체에서 뺍니다.

## 컴포넌트 이름

SPEC 1절 규칙대로 `Layout.printed_parts` 키와 6절의 플레이스홀더 이름을 그대로 사용합니다. 4개씩 있는
부품은 번호를 붙입니다: `vane_1..4`, `vane_horn_1..4`, `servo_vane_1..4`, `servo_arm_1..4`, `pushrod_1..4`.
전자 부품 상자는 `nav`(트레이의 NAV 보드, 0.6.3에서 `fc` 이름 변경), `fcu`(흡입 격벽 위 FCU 보드, 0.6.3),
`com`(페이로드 격벽 아래 COM 보드, v0.6), `radio_log`(COM 옆 무선·로거 상자, 0.6.2), `misc_electronics`, `battery`, `esc`입니다.
집중질량(`joint_screws_inserts`, `pushrods_horns_clevises`, `servo_leads`)은 바디 없이 보고서에만 들어갑니다.
그래서 보고서의 부품은 44개, 생성되는 컴포넌트는 41개입니다 (v0.6.3). v0.8은 출력 부품 `gps_pod_lid`, 플레이스홀더 `gps`(패치 + 보드, 몸체 2개)·`gps_ground_plane`, 집중질량 `gps_lead`가 더해져 보고서 48개, 컴포넌트 44개입니다. v0.8.1은 플레이스홀더 `gns`(GNS 보드)가 더해져 보고서 49개, 컴포넌트 45개입니다.
모든 컴포넌트는 전역 차량 좌표(항등 변환)로 생성됩니다.

## 프리셋 전환

`cad/fusion360/preset.txt`의 첫 번째 주석이 아닌 줄이 프리셋 이름입니다
(`BUDGET_90mm_6S` 기본 = 예산 설계, `KAIST_90mm_12S` = 원래 12S 설계). 파일을 바꾸고 스크립트를 다시 실행하면 됩니다
(스크립트는 매 실행마다 `project_root.txt`가 가리키는 저장소의 `params.json`/`layout.py`/`preset.txt`를 읽습니다).

다이렉트 모델링(타임라인 없음, 더 빠름)으로 만들려면 `cad/fusion360/direct_mode.txt` 파일을 만들거나
환경변수 `CANSAT_FUSION_DIRECT=1`을 설정합니다.

## 검증 도구 (Fusion 없이)

```bash
.venv/Scripts/python.exe tools/fusion_api_lint.py     # 설치된 Fusion API 스텁(core.py/fusion.py)과 대조한 정적 검사
.venv/Scripts/python.exe tools/fusion_mock_run.py     # MagicMock adsk로 run() 전체를 실행 (우리 로직/보고서 생성 검증)
<Fusion Python>/python.exe -m py_compile cad/fusion360/CanSatEDFRocket/CanSatEDFRocket.py   # Fusion의 Python 3.14로 컴파일
```

Fusion 실행 결과는 CadQuery 모델과 비교합니다(`cad/exports/crosscheck_fusion_vs_cadquery.txt`). 2026-10-15 실행은 출력 부품
11종 모두 체적 0.45 % 이내, 무게중심 0.082 mm 이내, 관성모멘트 0.62 % 이내이고, 전체 질량은 2585.80 g / 2585.73 g
(차이 0.003 %)입니다.

2026-10-15 실행의 보고서는 `params_sha256` `3f31c0a1…`를 기록하고 있고, 지금 `params.json`은 `0a5c261b…`입니다. 지금
`params.json`으로 다시 실행할 예정입니다. 그때까지 `make_print_files.py --check`는 STALE을 냅니다.

## 알려진 제한

* **사용자 매개변수는 미러일 뿐 형상을 구동하지 않습니다.** `params.json`의 body/edf/propulsion_module/vane/fin/
  intake_module/payload_module/nose 숫자 값이 `body_od = 120 mm` 식으로 등록되지만, 형상은 `layout.py` 수치로
  직접 그려집니다. Fusion 안에서 매개변수를 바꿔도 모델은 바뀌지 않습니다 — `params.json`을 바꾸고 재실행하세요.
* `params.json`을 바꾸면(형상과 무관한 값이라도) 보고서의 `params_sha256`이 달라져 출력 도구가 이전 내보내기를 쓰지 않습니다.
  스크립트를 다시 실행하세요.
* 노즈콘은 ShellFeature(바닥면 제거)를 먼저 시도하고, 실패하거나 체적 검증에 실패하면 SPEC 5.4가 허용하는
  내부 프로파일 컷(`r_in = r_out − wall`, 팁 8 mm는 솔리드)으로 대체합니다. 어느 쪽을 썼는지
  `derived.extra.nose_shell_method`에 기록됩니다 (2026-10-15 실행: ShellFeature). 팁 블런트(선택 사항)는 적용하지 않습니다.
* 베인 LE/TE 라운드(1.4 mm, 3 mm 판)는 선택 피처입니다. 필렛이 실패하면 `soft_failures`에 기록되고 계속 진행합니다.
* 서보 포드 밑면 45° 챔퍼는 "박스 하단을 챔퍼선 발끝까지 연장한 뒤 선 아래를 웨지 컷"으로 구현했습니다
  (SPEC 3.4 문구 그대로면 컷이 박스 아래 빈 공간만 지워 아무 효과가 없음 — `spec_conflicts` 참조).
* 축에 대칭인 관통 구멍/슬롯(45°·225° 등)은 하나의 관통 컷으로 처리합니다. 서로 다른 컴포넌트 사이에는
  불리언을 하지 않으므로 간섭 검사는 CadQuery 트윈의 `check_geometry.py`가 담당합니다.
* 질량특성은 Fusion 기본 재질(강)로 계산한 뒤 SPEC 6절 레시피대로 밀도 비로 스케일합니다(출력 부품 1.31 g/cm³,
  플레이스홀더는 질량/체적). 관성곱 부호 규약은 실행 시 시험 정육면체로 자동 판별하며, 판별 실패 시
  `warnings`에 남기고 대각 성분만 신뢰해야 합니다.
* 모든 평면은 생성 직후 탐침 스케치로 검증합니다(`modelToSketchSpace`의 z가 0인지). Fusion의 각도/오프셋
  부호 규약이 예상과 다르면 자동으로 부호를 바꾸고 로그에 `sign flipped`를 남깁니다.
* 스크립트는 Fusion 밖에서는 실행할 수 없습니다. `tools/fusion_mock_run.py`는 API 호출을 가짜로 바꿔
  파이썬 로직만 확인하므로, 실제 형상 생성 성공 여부는 Fusion에서 실행한 `run.log`로만 확인됩니다.

## v0.8 GPS 포드 (0.8.0, 아직 Fusion 실기 실행 전)

`params.json`의 `payload_module.gps_pod` / `avionics_module.gps_entry`가 있으면 (두 프리셋 모두):

* `payload_module`: `build_gps_pod`가 완성된 모듈에 GPS 포드를 CadQuery 트윈(`rocket_geometry.gps_pod_features`)과 같은 순서로
  붙입니다 - 바깥 상자 Join, 45° 바닥선 위 공간 Cut, 모서리 보스 2개 Join, 바닥 밑면 아래 Cut, 바깥 세로 모서리 45° 모따기 2개
  (새 도우미 `CB.poly_pod`: 포드 좌표 다각형을 수평 스케치에 그려 대칭 돌출 - `box_pod`와 같은 방식), 튜브 벽 부채꼴 다시 Join
  (`CB.sector_z`), 포드 구간 보어 다시 Cut, 막힌 뚜껑 나사 구멍 2개, 선 통로·커넥터 칸 상자 Join, 그 빈 공간 Cut. 모두
  `CB.required`(16개)이고 구멍·빈 공간은 `CB.cut_verified`, 껍데기는 벽·바닥·보스·복원된 튜브 벽의 점 판정(`CB.verify_points`)으로
  확인합니다. 로그: `payload_module: GPS pod 16/16 required features`, 보고서 `derived.extra.gps_pod`.
* `avionics_module`: GPS 선 입구 구멍 Ø5 (`gps_entry_hole`, 필수 1개, `derived.extra.gps_entry_hole`).
* 새 컴포넌트 `gps_pod_lid`(출력 부품: 판, 튜브 쪽 호, 모따기, 기둥 3개, 기둥 구멍 3개, 나사 구멍 2개 - 필수 11개,
  `derived.extra.gps_pod_lid`), `gps`, `gps_ground_plane`, 집중질량 `gps_lead`.
* 포드 각도 θ 192°의 수직 평면(12° 계열)은 v0.6.3 실기 실행에서 분리 서보 포드 쐐기(θ 12°)로 이미 만들어진 각도입니다.

실기 실행 뒤 확인할 것: 로그의 `GPS pod 16/16`, `derived.extra.gps_pod` / `gps_pod_lid` / `gps_entry_hole`의 `failed` = `[]`,
`containment_unanswered` = `[]` (모의 실행에서는 점 판정 API가 없어 모두 "답 없음"으로 남습니다). 그다음
`python tools/compare_stl_slices.py`로 CadQuery와 비교하고 `python tools/make_print_files.py`로 공식 출력 파일을 만듭니다
(`tools/make_print_files.py`의 PARTS에 `gps_pod_lid` 줄이 들어간 뒤).

## v0.8.1 GNS 보드 (0.8.1, 아직 Fusion 실기 실행 전)

`params.json`의 `electronics.gns` / `avionics_module.gns_mount`가 있으면 (두 프리셋 모두):

* `avionics_module`: `build_gns_mount`가 완성된 모듈의 -x 벽에 GNS 보드 보스 4개를 붙입니다. 순서는 다음과 같습니다.
  - 각 보스: 마름모(`Layout.gns_diamond_vz`, 면 45°)를 반경 방향에 수직인 평면에 그려 반경 방향으로 대칭 돌출합니다(Join). 새 도우미
    `CB.tprism`이며, 스케치 평면은 `vplane(theta + 90, -u_mid)`(θ 180°이면 yZ 평면을 u_mid만큼 옮긴 것, 0/90 계열, 부호는 탐침으로
    확인)입니다.
  - 튜브 바깥면 밖을 잘라냅니다(`gns_clip`, revolve_z Cut, 레일 자르기와 같은 방식).
  - 막힌 탭 구멍 4개: `CB.cut_between`(두 점 사이 원기둥, 자유 이동, Combine/Cut, 축 위 탐침점 - v0.6.3 실기에서 EDF 탭 나사 구멍을 만든
    방식). 구멍 끝 너머 점이 재료로 남는지 `CB.verify_points`로 확인합니다.
  - 모두 `CB.required`(9개). 로그: `avionics_module: GNS mount 9/9 required features`, 보고서 `derived.extra.gns_mount`.
* 새 컴포넌트 `gns`(GNS 보드 상자, `Layout.gns_mass_g` = 보드 가정 25 g + 나사 1.6 g).
* GPS 포드 벽·바닥·뚜껑이 params에서 1.2 mm로 얇아졌습니다(코드 변경 없음). 집중질량 `gps_lead`는 GNS에서 끝나는 경로를 따릅니다.

실기 실행 뒤 확인할 것: 로그의 `GNS mount 9/9`, `derived.extra.gns_mount`의 `failed` = `[]`, `containment_unanswered` = `[]`.

### Fusion 없이 하는 X3: `cad/fusion360/replay_x3.py`

```bash
.venv/Scripts/python.exe cad/fusion360/replay_x3.py                         # BUDGET
.venv/Scripts/python.exe cad/fusion360/replay_x3.py --preset KAIST_90mm_12S
```

모의 실행 중 Fusion 스크립트가 부른 CB 기본 도형 호출(종류, 전체 정밀도의 수치, Join/Cut/NewBody, 순서)을 기록해 OCC로 다시 만들고
CadQuery 트윈과 불리언 차이(< 1 mm³)와 단면 비교(X3)로 맞춰 봅니다. 결과: `cad/exports/crosscheck_fusion_replay_vs_cadquery*.txt`.
2026-10-21: payload_module, avionics_module, gps_pod_lid, gps, gps_ground_plane, piston 모두 불리언 차이 0.000 mm³ (두 프리셋).
v0.8.1: `CB.tprism`과 `CB.cut_between`도 재현합니다. 항공전자 모듈(GNS 보스 포함), 얇아진 페이로드 모듈·뚜껑, `gns` 모두 불리언 차이
0.000 mm³ (두 프리셋). 음성 대조(스크립트만 바꿈: 보스 4.0 mm, 구멍 0.5 mm 얕게, 보스 끝 0.3 mm 튀어나옴, 바깥 자르기 빠짐)는
불리언 비교가 4/4, 단면 비교만으로는 2/4를 잡았습니다.
Fusion 커널이 그 호출을 실제로 어떻게 실행하는지(평면 규약, 프로파일 선택, 피처 상태)는 증명하지 않으므로 실기 실행은 여전히 필요합니다.

## v0.9 모터선 출구 수정 (0.9.3, 2026-10-29, 아직 Fusion 실기 실행 전)

사용자 정보(2026-10-29): 모터선은 하우징 옆 중간이 아니라 **모터 맨 아래 끝의 옆면**에서 옆으로 나와, EDF 자체의 고정 날개(스테이터) 하나를
따라 하우징 벽까지 갑니다. 그래서 선은 하우징 뒤 끝(림) 아래로 EDF를 빠져나옵니다.

* `propulsion_module`: `build_tab_mount`에서 전선 슬롯 상자 두 개 뒤에 **필수 피처 하나** `wire_low_cut`를 자릅니다.
  `Layout.ml_low_cut_uz`의 (u, z) 다각형을 `wire_theta` 수직 평면에 그려 `ml_low_cut_w`(12.4 mm)로 대칭 돌출(`CB.prism`, Cut)합니다.
  덕트 안쪽(`duct_r` − 0.5)에서 시작해 라이너와 슬리브를 지나고, 바깥 굽힘을 따라 45° 모따기, 위쪽은 22 mm 슬롯 상자 안으로 0.5 mm 넘어갑니다.
  라이너는 이 함수 첫머리에서 붙이므로 자르는 순서는 CadQuery 트윈과 같습니다.
* 로그의 탭 고정 줄 피처 수가 하나 늘어납니다(`tab mount N/N required features`).
* 모터선 집중질량 `motor_leads`는 `Layout.ml_mass_g` / `ml_com`(새 경로)을 그대로 씁니다.
* 독립 검토 반영(2026-10-29): 케이블 타이 대신 고정 날개 뒤 끝 RTV 접착, RTV·테이프 1.0 g(ASSUMED)을 `ml_mass_g`에 더함, 배기 속 선 항력은
  검사(C33)·크기 산정에만 씁니다. 형상은 그대로라 스크립트는 0.9.3 그대로이고, 재생 비교(X3) 추진 모듈 0.0000 mm³(로켓, VTOL 3개 구성)입니다.

실기 실행 뒤 확인할 것: `derived.extra.tab_mount`의 `failed` = `[]`, 필수 피처 수가 같음. 재생 비교(X3)는 `replay_x3.py --parts propulsion_module,...`.

### 사진 라운드 (0.9.4, 2026-10-29, 아직 Fusion 실기 실행 전)

사용자 사진 2장(`docs/실물사진/EDF_모터선_출구_2026-10-29_1.jpg`, `_2.jpg`)으로 위 1차 모델이 틀린 것이 확인되었습니다. 선은 모터 **앞쪽 링**
(고정 날개·하우징 뒤 끝 높이)에서 **수축 튜브 한 묶음**(지름 8.4 mm ASSUMED)으로 나와 고정 날개 하나를 따라 림 아래로 나갑니다. 모터 몸통을 타고
올라가지 않습니다. 1차 파일은 `history/v0.9_wire_round1/`에 있습니다.

* 스크립트 코드는 그대로이고 `wire_low_cut`도 같은 피처 하나입니다. 다각형 `Layout.ml_low_cut_uz`가 바뀌었습니다: 묶음이 지나가도록 바닥이
  로켓 115.24 → **111.40**, VTOL 106.36 → **103.40**으로 내려갔고, 모따기·바깥면은 새 경로에서 다시 계산됩니다(폭 12.4 그대로).
  형상이 바뀌었으므로 `SCRIPT_VERSION`을 0.9.4로 올렸습니다.
* `motor_leads` 집중질량: 50.98 g (VTOL, 1차 51.78 g).
* 확인(2026-10-29): API 검사 0건, 모의 실행 5개 구성 실패 0, 재생 비교(X3) 추진 모듈 불리언 차이 0.0000 mm³(로켓, VTOL 3개 구성).
  흡입부·베인 실패는 예전과 같은 재생기 한계입니다. **Fusion GUI 실행은 아직 안 했습니다(사람이 해야 함).**
* 수정 라운드(독립 검토 반영, 2026-10-29): 공장 수축 튜브 대신 모터 몸통에서 14 ± 1 mm만 다시 씌우고(조립 규칙), 그 뒤 선 3가닥을 슬롯 폭 방향으로
  나란히 펴는 구간(약 7.45 mm, E)을 경로에 넣었습니다. 노치 다각형 `ml_low_cut_uz`는 **그대로**이고(출력 STL 53개 해시 같음), 스크립트도 0.9.4 그대로입니다.
  바뀐 것은 검사용 부피(C29, C8)와 항력 면적(210 → 232 mm², C33)뿐입니다.

## 시계 장착 (0.9.5, 2026-11-03, 아직 Fusion 실기 실행 전)

* 사용자 요청(2026-10-30, "재지 않아도 맞게"): `propulsion_module.tab_mount` = `clock_keyways`이면 `build_tab_mount`가 `build_clock_mount`로 갑니다.
  탭 나사·러그·22 mm 슬롯·노치는 만들지 않습니다.
* 새 피처(모두 `CB.required`, 기존 도구만 사용): 키웨이 바닥 패드(`prism`, `Layout.ck_pad_rz`, 45° 받침), 서보선 기둥 안내벽(U 프리즘 Join 뒤 가운데 통로 Cut),
  블리스터(`poly_pod`, `Layout.lp_blister_xy`), 45° V 용골(`tprism` 14개, `Layout.lp_keel`), 키웨이 25개(`box_pod`, 2026-11-04 수정 라운드 전 27개; 각도는 `Layout.ck_keyways`), 선 통로(`poly_pod` Cut, `Layout.lp_fan_xy`).
* 서보 리베이트 확인 점(`Layout.sl_rebate_kept_theta`)은 키웨이를 피해 고릅니다.
* 확인(실기 아님): API 검사 0건, 모의 실행 5개 구성 실패 0, 재생 비교 추진 모듈 불리언 차이 0.0113 / 0.0000 mm³(로켓, VTOL 3개 구성).
  `tools/fusion_api_lint.py`는 나사 장착 레이아웃도 함께 읽어 두 모드의 속성 이름을 모두 확인합니다.

## 흰색 몸통 + YUCHAN 로고 (0.9.6, 2026-11-04, 아직 Fusion 실기 실행 전)

* 사용자 결정(2026-11-04): 기체의 출력 부품은 모두 흰색(Bambu PLA Matte Ivory White, #FFFFFF, 코드 11100),
  전자부 모듈 양옆(67° / 293°)에 주황(Mandarin Orange, #F99963, 코드 11300) "YUCHAN" 로고를 평평한 상감으로 넣습니다.
  발사대는 요청 범위가 아니라서 검정(Charcoal) 그대로입니다. `SCRIPT_VERSION`을 0.9.6으로 올렸습니다.
* 로고 다각형은 `cad/livery/yuchan_logo_polygons.json`(생성기 `cad/livery/make_logo.py`)에 있고, `cad/layout.py`의 `Layout._livery`가
  모듈 위에 놓습니다(`livery_polys_vz`). 스크립트에는 치수가 없습니다.
* 새 도구 `CB.inlay`: 섬(로고 조각) 하나마다 부채꼴 띠(`sector_z`, 새 바디, r 59.58-60)를 만들고, 그 섬의 윤곽(펴서 미리 휜 (v, z) 다각형, 구멍 포함)을
  tprism 평면에 그려 반지름 방향으로 대칭 돌출하며 그 띠와 **교집합**(participantBodies)을 냅니다(0.6.2 흡입 창과 같은 방법).
  * `avionics_module`: 마지막에 이 도구로 0.42 mm 홈을 팝니다(`build_livery` CUT: Combine/Cut으로 본체에서 빼고 도구는 없앰). 흰 몸통입니다.
  * 새 출력 컴포넌트 `avionics_logo`: 같은 도구를 새 바디로 남깁니다(섬 10개 × 2 = 20개). 주황 상감입니다.
  * 모두 `CB.required`이고 보고서 `derived.extra.logo_pocket` / `logo_inlay`에 성공 수가 남습니다.
* 색(모양에는 영향 없음): `Ctx.apply_appearance`가 출력 부품에 흰색, 로고에 주황, 발사대에 검정 외관을 줍니다(라이브러리 플라스틱 외관을
  복사해서 색만 바꿈). API가 거부하면 로그에 남기고 넘어갑니다(실패로 치지 않음). 보고서 `derived.extra.appearances`.
* 확인(2026-11-04, 실기 아님): API 검사 **0건**, 모의 실행 **5개 구성 실패 0**(교집합 피처 44개 = 로고 40 + 흡입 창 4),
  재생 비교(X3, `replay_x3.py`가 `CB.inlay`를 따로 구현): `avionics_module` 불리언 차이 **0.0000 / 0.0000 mm³**(로켓, VTOL 배터리·유선·하이브리드),
  `avionics_logo` **0.0000 / 0.0000 mm³**(부피 680.99 mm³, 20개 고체). 흡입부·베인·트레이는 예전과 같은 재생기 한계입니다.
  **Fusion GUI 실행은 아직 안 했습니다(사람이 해야 함).** 실기에서는 스케치 하나에 선이 최대 120개(CAD용 단순화 0.005 mm, 스케치 40개)라 이 부분이 몇 분 더 걸릴 수 있습니다.

## 스위치 보호 칼라 (0.9.8, 2026-11-05, 아직 Fusion 실기 실행 전)

* 사용자 결정(2026-11-05): "상단 캡과 한 몸으로 출력하는 보호 칼라를 달아줘". 로직 스위치 레버 둘레에 상단 캡과 한 몸인 칼라를 둡니다.
  **VTOL 배터리 구성만**입니다(`Layout.has_lp_collar`). `SCRIPT_VERSION`을 0.9.8로 올렸습니다.
* `build_top_cap`이 판 구멍을 뚫기 **전에** 네 단계를 더합니다(치수는 모두 `Layout.lp_col_*`, 스크립트에 숫자 없음):
  `cap_collar`(cyl_z JOIN, 바깥 Ø25.8, 판 두께 가운데부터 테두리 z 492.5까지), `cap_collar_bore`(cyl_z CUT, 안 Ø21, 판 윗면부터),
  레버가 젖혀지는 y 방향 손가락 홈 2개를 **U자 홈**으로(독립 검토 반영 2026-11-05, 처음은 폭 16 mm 네모 홈): `cap_collar_notches`(box_xy CUT, 폭 17 mm, 반원 중심 z 488.80부터 위), `cap_collar_notch_round`(cut_between CUT, Ø17 원기둥, y 방향, 축 z 488.80, 가장 낮은 곳 판 위 5.0 mm). 같은 라운드라 버전은 0.9.8 그대로이고 변경 기록 줄에 적었습니다.
* 칼라도 로직 전원 부품이라 질량 결정 대기 중(`in_flight_mass_model` false)에는 합계에서 뺍니다:
  `__top_cap_logic_collar_removed`(Layout 칼라 부피 2,104.03 mm³ × 캡 밀도의 음의 점질량, 칼라 무게중심). 구멍 마개와 같은 방식입니다.
* 확인(2026-11-05 검토 반영 뒤, 실기 아님): API 검사 **0건**, 모의 실행 **5개 구성 실패 0, ERROR 줄 0**, 재생 비교(X3): 상단 캡(구멍 + U자 홈 칼라)
  **0.0000 / 0.0000 mm³**(부피 29,573.8 mm³), `logic_pack`·`logic_switch` 0.0000 / 0.0000, 유선·하이브리드 상단 캡(칼라 없음) 0.0000 / 0.0000.
  흡입부·트레이는 예전과 같은 재생기 한계입니다. **Fusion GUI 실행은 아직 안 했습니다.**

## 로직 전원 (0.9.7, 2026-11-05, 아직 Fusion 실기 실행 전)

* 사용자 결정(2026-11-05): 캐리어 PCB rev 2c의 1S 로직 배터리를 6S 팩 −x 면(A 자리)에, MTS-102 스위치를 VTOL 상단 캡의 구멍에 넣습니다.
  **VTOL 배터리 구성만**입니다(`Layout.has_logic_power`; 유선·하이브리드는 이 캐리어로 더 맞추지 않음). `SCRIPT_VERSION`을 0.9.7로 올렸습니다.
* 상단 캡: 스위치 구멍(Ø6.2, r 43, 140°)은 `Layout.cap_holes`에 들어 있어 `build_top_cap`이 다른 구멍처럼 뚫습니다(스크립트에 치수 없음).
* 새 자리표시 컴포넌트 `logic_pack`(배터리 상자), `logic_switch`(`build_logic_switch`: 안쪽 너트, 몸통, 단자, 부싱, 바깥 너트, 레버),
  묶음 질량 `logic_switch_lead`, `logic_retention`.
* 질량 결정 대기(`params.json` `configs.vtol.logic_power.in_flight_mass_model` false): 이 넷은 만들고 재지만 보고서 합계에는 넣지 않습니다
  (`derived.extra.logic_power.pending`). 합계에는 스위치 구멍만큼의 마개(Layout 부피 × 캡 밀도)를 더해 CadQuery의 비행 질량 모델과 맞춥니다.
* `printed_density`는 슬라이서 표에서 이 컴포넌트의 부피에 가장 가까운 항목(`top_cap` 또는 `top_cap@no_logic_switch_hole`)을 씁니다.
* 확인(2026-11-05, 실기 아님): API 검사 **0건**, 모의 실행 **5개 구성 실패 0**, 재생 비교(X3): 상단 캡(구멍 포함) **0.0000 / 0.0000 mm³**,
  `logic_pack`·`logic_switch` **0.0000 / 0.0000 mm³**, 유선·하이브리드 상단 캡(구멍 없음) 0.0000 / 0.0000 mm³. **Fusion GUI 실행은 아직 안 했습니다.**
