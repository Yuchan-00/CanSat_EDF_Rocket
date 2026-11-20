# rev 3 독립 검토 지적 사항 판정과 조치

## 추가 기록 (2026-11-11, 재배치 단계와 재배치 수정 단계 뒤) - 먼저 읽으십시오

**이 아래 '수정 단계 기록'(표와 English detail)은 수정(fix) 단계 당시의 기록이며, 그때의 보드(sha256 b1fe1813...)를 설명합니다. 지금 보드와 다릅니다.**
그 기록은 고치지 않고 보존했습니다. 그 뒤에 한 일은 두 가지입니다.

1. 재배치 단계: 승압 IC U8 주변 부품 7개(U8, C57, C43, C41, R63, R64, C58)를 옮기고 배선을 새로 했습니다.
2. 재배치 수정 단계: 재배치에 대한 독립 검토(RR-1 ~ RR-9)의 지적을 확인하고 조치했습니다. 부품 위치와 구리 모양은 바꾸지 않았습니다.

지금 납품 보드: `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/kicad/vtol_carrier.kicad_pcb`,
sha256 b8648809d12090d5bdfc8cb9d6c71ae6eb0c80db95c8ae288a5727fa634db65d.
지금의 숫자와 판정은 생성 문서인 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/ORDER_ko.md` 0장과
`B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/README_ko.md` 1장이 기준입니다. 이 추가 기록의 숫자는 2026-11-11 재배치 수정 단계의 측정값(M,
`out/reports/routing_rules.json`)입니다. 출고 판정은 여전히 NOT ORDERABLE입니다.

### 아래 표에서 상태가 바뀐 줄

| Id | 아래 표의 상태 | 지금 상태 | 근거 (M) |
|---|---|---|---|
| E-R1 | 인정 (미수정) | 대부분 고침 | U8.4 / C57.2 / C43.2 / C41.2가 윗면 GND 구리 한 덩어리 위 (검사 hot_loop_top_gnd PASS). 다만 계획 448줄의 '두 출력 커패시터 모두 2 mm 안'(검사 hot_loop)은 FAIL로 남음: C43 간격 3.25 / 3.537 mm |
| E-R2 | 인정 (미수정) | 일부 고침 | C41 → U8 VIN 구리 길이 1.54 mm (검사 input_cap, input_cap_vin PASS). C41 → L5 입력 13.53 mm (검사 input_cap_l5 FAIL; 5 mm 기준은 스크립트가 정한 값, E) |
| E-R3 | 인정 (미수정) | 고침 | FB 선이 출력 커패시터 패드 사이를 지나지 않음 (검사 fb_clear_of_caps PASS) |
| E-R4 / MR-3 | 인정 (수정) | 숫자 바뀜 | 세그먼트·비아 수는 README가 최종 보드 파일에서 다시 셉니다 (겹쳐 있던 똑같은 배선 조각 173개를 지운 뒤의 수) |
| E-R5 / J5 | 일부 인정 (문서만) | 고침 | gbrjob 표면처리가 ENIG (build_pcb.py 적층 설정) |
| J2 | 인정 (수정) | 더 고침 | 레일이 붙는 두 변의 구리 이격 0.400 mm (검사 rail_edge_copper PASS). V-cut 기준값(0.4 mm, F)에 여유 없이 딱 맞으므로 V-cut 가능 여부는 NOT_VERIFIED, 메모는 라우팅 탭을 첫째로 요청 |
| J7 | 인정 (수정) | 숫자 바뀜 | 가장자리 2.5 mm 안 몸체 34개 = JLC 실장 28 / with_mcu 29 / 미실장 6 (edge_distance.json) |
| MR-5 | 일부 인정 | 대부분 고침 | 보드 주석층(Dwgs.User) 글자는 고침. B.SilkS의 U8 / U9 글자는 그대로임 |

배터리 경로 폭(검사 battery_path)은 여전히 FAIL입니다: 2.0 mm보다 좁은 길이 16.1 mm → 8.57 mm, 구리 저항 0.03173 Ohm (20 C, E).

### 재배치로 나빠진 것 (재배치 검토 RR-1, 인정)

- 검사 sw_node FAIL (재배치 전 보드에서는 PASS: 간격 0.464 mm, SW 선 1.174 mm): U8의 SW 패드와 L5의 SW 패드 사이 간격 5.551 mm, SW 선 길이 6.619 mm, SW 선이 C57 / C43의 패드 사이를 지남.
  계획 449줄·466줄('L5는 U8 핀 바로 옆', 'SW 구리는 두 패드에 필요한 만큼만', '패드가 허락하는 한 짧게')을 지키지 못합니다.
  2.0 mm 기준은 계획에 숫자가 없어 스크립트가 계획 448줄의 값을 빌려 읽은 것(E)입니다.
- 검사 crystal_distance FAIL: rev 2c의 Pololu 핀은 NAV 크리스털 Y1 패드에서 9.217 mm, U1 XTAL 핀에서 9.616 mm였습니다 (보관된 rev 2c 보드에서 같은 방법으로 잼).
  지금은 C57이 Y1 패드에서 7.501 mm, U8이 XTAL 핀에서 8.35 mm, SW 구리가 Y1 패드에서 8.475 mm / XTAL 핀에서 8.943 mm / 크리스털 배선에서 7.465 mm입니다.
  계획 452줄을 지키지 못합니다. 재배치 전 보드도 이 검사는 FAIL이었습니다 (C57 7.557 mm, L5 8.714 mm가 Y1 패드에 rev 2c보다 가까움);
  재배치로 U8과 C43도 더 가까워졌고, SW 구리와 XTAL 핀 사이는 11.497 mm → 8.943 mm, SW 구리와 크리스털 배선 사이는 8.891 mm → 7.465 mm가 되었습니다.
- 이 두 검사는 재배치 수정 단계에서 새로 넣었고, 출고 판정 이유에 들어갑니다. 전기적 영향(과전압, 리플, 발열, 크리스털 잡음)은 보드에서 재기 전까지 NOT_VERIFIED입니다.

### 재배치 검토(RR) 지적별 조치

| Id | 심각도 | 판정 | 조치 |
|---|---|---|---|
| RR-1 | major | 인정 (수정: 검사·문서) | 검사 sw_node, crystal_distance 추가 (둘 다 FAIL), 출고 판정 이유와 ORDER 0장 B·README 1장에 숫자와 함께 기재. 보드는 바꾸지 않음 (배치 변경은 사용자 결정) |
| RR-2 | major | 인정 (수정) | route.py가 배선·비아·실크 글자의 uuid를 항목 내용에서 만들도록 바꿈. determinism.py에 `--cross-folder` 추가: 다른 폴더에서 다시 만든 파일과 바이트 비교. 결과는 README 1장·6장 |
| RR-3 | major | 인정 (수정) | 이 추가 기록 |
| RR-4 | minor | 인정 (문서) | C59 GND 패드 옆에 자기 비아 없음을 routing_rules.json information에 측정값으로 기록, ORDER 0장 B에 기재. 비아 추가는 다음 새 배선 때 |
| RR-5 | minor | 인정 (문서) | L5 입력 앞 비아 수를 측정값으로 기록, ORDER 0장 A에 기재 |
| RR-6 | minor | 인정 (수정) | 겹쳐 있던 똑같은 배선 조각 173개 제거 (구리 모양은 같음), make_all.py 설명 수정 |
| RR-7 | minor | 인정 (수정) | 5 mm 기준이 스크립트가 정한 값(E)임을 문서에 표기, 겹친 PASS / FAIL 말머리 제거 |
| RR-8 | minor | 인정 (수정) | 뒤 단계가 다시 쓰는 파일은 manifest 첫 목록에서 빼고 build.written_after_checks_exports 에만 둠 |
| RR-9 | minor | 인정 (문서) | V-cut은 'JLC 기준값에 딱 맞음, NOT_VERIFIED'로 표기, 라우팅 탭을 첫째로 요청 |

---

## 수정 단계 기록 (재배치 전 보드 b1fe1813... 기준, 보존)

세 검토(전기, JLC 주문, 기구·그림·문서)가 낸 지적 21건을 1차 자료(보드 파일, 도구 보고서, 계획서, 데이터시트 발췌)와 직접 대조해 판정했습니다.
막는 문제(blocker)는 없었습니다. 표시: M = 이 PC에서 도구로 측정, E = 추정, A = 가정, F = 문서 근거.
판정: 인정 = 지적이 맞음, 일부 인정 = 일부만 맞거나 일부만 고침, 기각 = 근거와 함께 받아들이지 않음 (이번에는 기각 없음).

보드 파일 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/kicad/vtol_carrier.kicad_pcb` 는 이번 단계에서 바뀌지 않았습니다
(sha256 b1fe181392074abbfcc1acc4c36b6b6ae51c150fc4f4a96effd6c2150a884274). 보드 안의 글자·적층 설정을 바꾸면 배선 전 보드의 해시가 달라져
기록된 Freerouting 세션의 재생(route.py --reuse-ses)이 거부되므로, 보드를 바꾸는 조치는 모두 다음 새 배선(사용자 결정) 때로 미뤘습니다.

| Id | 심각도 | 판정 | 한 줄 요약 (한국어) | 조치 |
|---|---|---|---|---|
| E-R1 | major | 인정 (미수정) | 출력 커패시터 C57 / C43의 GND가 윗면 구리로 U8 GND 핀에 이어지지 않음 | 새 검사 hot_loop 추가 (FAIL), ORDER 0장 B와 7장 측정 항목. 배치 변경 + 새 배선은 사용자 결정 |
| E-R2 | major | 인정 (미수정) | U8 VIN 가까이에 입력 커패시터가 없음 (구리 길이 약 23 mm); 배치 단계 보고의 '5.4 mm'는 과소 표기 | 새 검사 input_cap 추가 (FAIL), 문서에 실측 경로 기재. 배치 변경은 사용자 결정 |
| E-R3 | minor | 인정 (미수정) | 되먹임(FB) 선이 C57 몸체 아래를 지남 | hot_loop 검사에 정보로 기록; E-R1과 함께 고칠 일 |
| E-R4 | minor | 인정 (수정) | README의 세그먼트 2308 / 비아 327은 중간 단계 숫자 | docs_ko.py가 최종 보드 파일에서 셈 (세그먼트 2610, 비아 444) |
| E-R5 | minor | 일부 인정 (문서만) | gbrjob 표면처리 HAL vs 주문서 ENIG | ORDER 2장에 불일치와 '주문 화면 값이 정함'을 명시. 적층 설정 수정은 다음 새 배선 때 |
| E-R6 | minor | 인정 (수정) | VBAT_LOGIC_SW에 세라믹 커패시터만 있음 - 켜질 때 울림 가능 | ORDER 7장 4번에 켜질 때 VIN 파형 측정 추가 |
| J1 | major | 인정 (수정) | SMD 대체 부품이 THT 자리 대체품처럼 적혀 있었음 | ORDER 5장을 표 1(같은 풋프린트 / 호환 NOT_VERIFIED)과 표 2(새 풋프린트 필요, 사용 불가)로 나눔 |
| J2 | major | 인정 (수정) | 레일 쪽 가장자리까지 구리 0.3 mm, V-cut 기준 0.4 mm | edge_distance.py에 구리-가장자리 측정 추가 (M 0.3 mm), 메모에 '라우팅 탭 / 마우스바이트, V-cut 아님', 3장 5번에 멈춤 조건 |
| J3 | major | 인정 (수정) | J8-J12, J14, J16은 JLC 쪽 1번 핀 번호가 반대 - 미리보기에서 돌리면 안 됨 | ORDER 3장 4번과 메모에 경고 (목록은 회전 표에서 읽음) |
| J4 | minor | 인정 (수정) | 미실장 Q3 / R47에 솔더 페이스트 | paste_variant.py가 두 묶음 모두 fit none 자리의 페이스트를 뺌; 메모에 빈 자리 전체 목록 |
| J5 | minor | 일부 인정 (문서만) | E-R5와 같음 | E-R5와 같음 |
| J6 | minor | 인정 (수정) | with_mcu 메모의 베이킹 목록에 C6510 없음 | with_mcu 메모 전체를 따로 적음 (C6510 포함) |
| J7 | minor | 인정 (수정) | '가장자리 2.5 mm 안 32개'에 JLC가 안 붙이는 자리 6개 포함 | JLC 실장 26 / with_mcu 27 / 미실장 6으로 나눠 표기 (edge_distance.json, M) |
| J8 | minor | 인정 (수정) | DFM·위치 확인이 결제 전인지 후인지 모름 | ORDER 3장 7번: 결제 뒤 확인과 보류 / 취소 절차, NOT_VERIFIED |
| J9 | minor | 인정 (수정) | U8 재고가 빠르게 줄어듦 | 지켜볼 줄 규칙에 '계획 때 재고의 절반 미만' 추가 |
| MR-1 | major | 인정 (수정) | '가정 치수로 충돌 0건'이 보수적 경우(U6 -0.5 mm, C3 -0.05 mm)를 숨김 | README 1·5장, ORDER 7장 7번에 두 경우 모두 (fit_checks.json에서 생성) |
| MR-2 | major | 인정 (수정) | 계획 17장의 위험 항목 일부가 사용자 문서에 없음 | README 5장에 추가; docs_ko.py가 17장 모든 항목의 대응을 검사 (빠지면 오류) |
| MR-3 | minor | 인정 (수정) | E-R4와 같음 | E-R4와 같음 |
| MR-4 | minor | 인정 (수정) | 띠 안 프로그래밍 헤더가 케이블타이 장착과 충돌한다는 경고 없음 | ORDER 7장 10번, 8장에 '장착 전에 띠를 비움' |
| MR-5 | minor | 일부 인정 | 낡은 문구 (보드 주석층, ORDER J3 줄, 슬라이드 e1) | ORDER J3 줄은 사양 값(ESP-12F) 사용, 슬라이드 e1 문구 수정. 보드 주석층 글자는 다음 새 배선 때 |
| MR-6 | minor | 인정 (수정) | TQFP-32 영역 검사가 몸체 상자만 봐서 U1 패드를 놓침 | fit_checks.py가 패드 범위도 셈 (패드만 겹치면 부품 전체 높이로 보수적 계산) |

## English detail

Evidence files are in `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/reports/` unless stated.

### E-R1 (upheld, not fixed)
Primary source: `routing_rules.json` check `hot_loop` (new, `scripts/routing_checks.py`, M). Copper edge gaps: C57.1-U8.6 0.925 mm,
C57.2-U8.4 3.195 mm, C43.1-U8.6 2.426 mm, C43.2-U8.4 5.375 mm; F.Cu GND pieces (zone islands joined by F.Cu GND tracks and pads):
U8.4, C57.2 and C43.2 are on three different pieces -> `output_cap_gnd_joined_to_U8.4_on_F.Cu` is empty. This reproduces the reviewer's
islands #4 / #8 / #3. Criterion = plan line 448 ("both capacitors within 2 mm of the IC, GND vias to In1 directly at the capacitor
pads"). Not fixed: the +x/-y corner has no free 1206 spot next to U8 (placement step searched it); a fix means re-placing the corner
(U6, C45, R63/R64, C58, C43, C57) or adding a smaller capacitor (BOM change), then a new live route. That is a design iteration
whose outcome is uncertain and it overlaps the open battery_path decision, so it is left to the user (ORDER_ko.md section 0 B).

### E-R2 (upheld, not fixed)
`routing_rules.json` check `input_cap` (new): C41.1-U8.3 pad edge gap 5.601 mm, copper path 23.36 mm with 2 vias (track model, E;
reviewer 22.94 mm with his own model), C41.1-L5.1 26.48 mm, C42 140+ mm. Criterion = plan line 449 + 2.0 mm figure of line 448.
The '5.4 mm' figure exists only in the placement-step hand-off (no repository file carries it); the documents now quote the measured
path. Same reason for not fixing as E-R1.

While adding this check a defect of the existing track model was found and fixed: a track end that lands on another segment
(e.g. B.Cu end x 129.3005 on the 14.3 mm VBAT_LOGIC_SW track at y 125.48) was not tied to the cut node when the rebuilt cut point
rounded to a different 1 um key, so U8.3 looked disconnected. The fix always ties the end to its cut point and ties track ends on
a via annulus to the via. battery_path numbers are unchanged (0.03231 Ohm at 20 C, 0.03739 Ohm at 60 C, 16.1 mm narrower than 2.0 mm).

### E-R3 (upheld, not fixed)
`hot_loop.information_boost_fb_under_output_cap`: 5 BOOST_FB F.Cu segments (0.15 / 0.2 mm) inside the C57 courtyard box. Not a plan
criterion, kept as information; to be fixed together with E-R1.

### E-R4 / MR-3 (upheld, fixed)
`scripts/docs_ko.py` counts on the final board file: 2610 segments, 0 arcs, 444 vias (438 x 0.6/0.3 mm, 6 x 0.8/0.4 mm), same as the
reviewers' grep. The old numbers came from `kicad/work/route_reuse_log.json` (import stage).

### E-R5 / J5 (partly upheld: document only)
`out/gerber/vtol_carrier-job.gbrjob` GeneralSpecs.Finish = "HAL lead-free" (M); board stackup from `scripts/build_pcb.py:208,223`.
Changing it changes `kicad/work/vtol_carrier_unrouted.kicad_pcb` and the replay refuses the recorded sessions (route.py:971), i.e. a
new live route. ORDER_ko.md section 2 now states the mismatch and that the order-page field (ENIG) decides; whether JLC reads the job
file is NOT_VERIFIED. Change build_pcb.py together with the next live route.

### E-R6 (upheld as a bring-up item)
No design change asked by the reviewer for the default numbers; ORDER_ko.md 7.4 adds the VIN / VBAT_LOGIC_SW scope capture at switch-on.

### J1 (upheld, fixed)
Classification is read from the plan's own alternate notes (`spec/rev3_plan.json alternates[].note`): 'new footprint' / 'footprint
not chosen' -> table 2 (not usable with these files); 'same series and land pattern' / 'same MPN' -> same package; everything else ->
footprint compatibility NOT_VERIFIED. A gate group without a plan note raises an error.

### J2 (upheld, fixed in the order text)
`edge_distance.json copper_to_edge_mm` (new, M): minimum copper (zone fills refilled in memory, tracks, vias, pads, all 4 copper layers)
to the straight outline lines is 0.3 mm on every side (the 0.3 mm edge rule of the unchanged rule file). JLC numbers (V-cut >= 0.4 mm,
routed / mouse-bite >= 0.2 mm) are F from the JLC capabilities page as read by the JLC reviewer on 2026-11-11; not re-read in this step.
The remark asks for routed tabs / mouse bites, not V-cut, and a V-cut-only answer is a stop condition. Pulling copper back to 0.4 mm
would be a design change (user).

### J3 (upheld, fixed)
Refs read from `out/cpl_rotation_table.csv` notes containing 'mirrored': J8, J9, J10, J11, J12, J14, J16.

### J4 (upheld, fixed)
`scripts/paste_variant.py` now removes F.Paste from fit 'none' parts in both sets (default set additionally U1..U4);
`scripts/checks_exports.py` builds both zips' F.Paste from these variants and still proves every other layer equals the design board.
The remark lists every intentionally empty footprint from the spec (fit none).

### J6, J7, J8, J9 (upheld, fixed)
J6: separate with_mcu remark with C6510. J7: `edge_distance.json bodies_under_2p5_by_fit` = jlc 26, jlc + jlc_optional 27, not fitted
by JLC 6 (U4, J1, J19, JP1, Q3, R47). J8: ORDER 3.7. J9: watch rule adds lines whose gate stock is below half of the plan snapshot
(`spec/rev3_plan.json bom_lines[].stock`).

### MR-1 (upheld, fixed)
Text generated from `out/fit_checks.json module_keepouts` (nominal collisions, below 0.5 mm, conservative collisions, conservative below
0.5 mm). See README_ko.md section 1 table and section 5, ORDER_ko.md 7.7.

### MR-2 (upheld, fixed)
`docs_ko.py` reads every bullet of REV3_PLAN.md section 17, maps it to a README / ORDER line (table S17) and stops with an error if a
bullet is unmapped or a number of a new sentence is not in the bullet. New README lines: valley current limit, hysteresis margin,
loaded-voltage cut-off, switch-on threshold, start-up into 226.9 uF, +3V3 headroom, drain after cut-off, DC-bias derating, U8 generic
symbol, ESP-12F RF assumption, programming access, AVCC choke.

### MR-4 (upheld, fixed) / MR-5 (partly) / MR-6 (upheld, fixed)
MR-4: ORDER 7.10 and section 8. MR-5: ORDER section 10 now prints the spec value as built (J3 'ESP-12F ...'), slide e1 caption says
the GY-521 is soldered; the Dwgs.User comment texts ('module ~1.5 mm', 'ESP-12E') and the B.SilkS U8 / U9 texts stay on the board
until the next live route (they come from build_pcb.py / place.py and would break the replay). MR-6: `scripts/fit_checks.py`
`under()` counts a footprint whose pad extents overlap a zone (full part height charged, label 'pads only').
