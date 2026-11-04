# 임시 출력 계획 v0.9 (CadQuery 트윈 STL, 부품 등급별 설정)

**임시본입니다.** 공식 출력 파일은 Fusion 360 실행 뒤 `tools/make_print_files.py`로 만듭니다. 질량은 Bambu Studio가 이 STL을 등급 설정으로 슬라이스한 G-code의 부품 압출량(서포트·브림 제외)입니다.

프린터 `Bambu Lab P2S 0.4 nozzle`, 공정 `0.20mm Standard @BBL P2S`, 필라멘트 `Bambu PLA Matte @BBL P2S` (밀도 1.31 g/cm³).

| 부품 | 등급 | 설정 | 수량 | 출력 방향 | 서포트 | 1개 부품 g | 1개 필라멘트 g | 1개 시간 |
|---|---|---|---|---|---|---|---|---|
| propulsion_module | A_load_path | 4 walls, 4 top / 4 bottom layers, 15 % gyroid | 1 | upright, exhaust exit on the bed, turned 45 deg on the plate (v0.9: the fins on the axes span 246.8 mm; axis-aligned the P2S plate refuses it - its wrap-detection zone - so the fins print diagonally, 176.2 mm); no support: the fin-root fairings' keels are 45 deg, each servo pocket's ceiling bridges its width onto the service-side lintel and the lintel bridges the opening length (40 mm); the lead pipes rise at 33 deg from vertical | 없음 | 301.2 | 301.2 | 8 h 57 min |
| intake_module | A_load_path | 4 walls, 4 top / 4 bottom layers, 15 % gyroid | 1 | printed flipped (SPEC 5.1 / 12): the spigot stands on the bed; the 45 deg floor wedges keep supports out of the closed lead channels only in this orientation (upright, the channel roofs would need support inside); v0.6.2: every window ceiling is a 45 deg truncated gable whose flat is bridged, and the part is a Bambu project with a support blocker over each window opening (snug supports, shrunk 0.5 mm), so no support inside the windows - checked on the sliced G-code; supports stay under the bulkhead inside the spigot, under the shelf ledge, the spigot step and the lead bulges | 일반 (밀착형) (`normal(auto)`, `snug`) | 122.6 | 201.9 | 4 h 41 min |
| avionics_module | B_shell | 3 walls, 4 top / 4 bottom layers, 15 % gyroid | 1 | upright | 없음 | 57.7 | 238.9 | 16 h 28 min |
| └ avionics_logo (주황 상감, 같은 출력물) | B_shell | 바깥 벽 1줄 | 1 | 모듈과 함께 | - | 0.90 | 75.6 | 모듈 시간에 포함 |
| avionics_tray | C_flat_plate | 3 walls, 4 top / 4 bottom layers, 15 % gyroid | 1 | flat | 없음 | 14.2 | 14.3 | 0 h 27 min |
| top_cap | A_load_path | 4 walls, 4 top / 4 bottom layers, 15 % gyroid | 1 | flat: plate top face on the bed, skirt / ribs / hub / rim ring up | 없음 | 35.2 | 35.2 | 0 h 43 min |
| vane | D_small_keep | 5 walls, 5 top / 5 bottom layers, 15 % gyroid | 4 | flat; the D5 shaft stands 1 mm proud of the 3 mm plate -> supports on build plate | 트리 (`tree(auto)`) | 3.6 | 4.1 | 0 h 09 min |
| vane_coupler | D_small_keep | 5 walls, 5 top / 5 bottom layers, 15 % gyroid | 4 | upright, flange on the bed, D-bore vertical | 없음 | 0.9 | 0.9 | 0 h 06 min |
| servo_cover | C_flat_plate | 3 walls, 4 top / 4 bottom layers, 15 % gyroid | 4 | flat | 없음 | 3.4 | 3.4 | 0 h 05 min |
| wire_cover | B_shell | 3 walls, 4 top / 4 bottom layers, 15 % gyroid | 1 | flat | 없음 | 2.2 | 2.2 | 0 h 04 min |

**합계 (수량 반영)**: 32 h 41 min, 필라멘트 827 g, 부품 566 g (발사대 제외 기체 부품 566 g)

## 색별 필라멘트 (슬라이서 M, 퍼지·프라임 타워·서포트 포함)

| 색 (Bambu PLA Matte) | 코드 | 색값 | 슬라이스 g | 예비 10 % 포함 g | 1 kg 롤 | 부품 |
|---|---|---|---|---|---|---|
| Ivory White | 11100 | #FFFFFF | 752 | 827 | 1 | propulsion_module, intake_module, avionics_module, avionics_tray, top_cap, vane, vane_coupler, servo_cover, wire_cover |
| Mandarin Orange | 11300 | #F99963 | 76 | 83 | 1 | avionics_module |

## 2색 부품: avionics_module + avionics_logo (한 출력물, 필라멘트 2개)

* AMS 슬롯: 1번 Ivory White (#FFFFFF, 코드 11100), 2번 Mandarin Orange (#F99963, 코드 11300). 출력 파일 `avionics_module.3mf` (색 지정이 들어 있는 Bambu 프로젝트).
* 퍼지량(플러시): 흰→주황 150 mm³, 주황→흰 210 mm³ (**A, 가정값**: Bambu Studio 측정표에서 가장 가까운 색 쌍 연회색 #D1D3D5 ↔ 주황 #FF6A13; 흰 #FFFFFF / Mandarin #F99963 쌍은 표에 없음). 프라임 타워 켬. 얇은 벽 감지(detect_thin_wall) 켬 - 주황 한 줄이 한 번만 지나가게 합니다.
* 슬라이스(M, 위 퍼지 A 기준): 시간 16 h 28 min (한 색일 때 2 h 02 min, **+14 h 26 min**), 색 바꿈 **576번** (출력 높이 3.4-124.2 mm).
* 무게(M, 2색 슬라이스, 퍼지 A 기준): 흰 부품 57.55 g, 주황 부품 0.90 g, 퍼지 135.8 g, 프라임 타워 44.6 g; 필라멘트 흰 163.2 g, 주황 75.6 g (한 색일 때 58.4 g).
* 비교(M): 표 없이 CLI 기본 퍼지 280 mm³이면 +15 h 34 min, 퍼지 211.3 g, 타워 44.6 g (위쪽 기준값).
* 주황이 들어간 출력 경로(M, 필라멘트 2): Outer wall 0.90 g, Prime tower 18.13 g. 부품에서는 바깥 벽(Outer wall)만 주황입니다.
* 주황 한 번 지나가기 검사(M, G-code): **PASS** - 주황 경로 8147 mm, 한 번 지나갈 때 8108 mm (비 1.005, 기준 ≤ 1.15); 주황 0.902 g, 고체 부피 × 밀도 0.892 g (+1.1 %, 기준 ±10 %). 정보: 주황이 찍히는 층 574개, 주황 없이 색만 바꾸는 층 2개 (출력 높이 14.1, 124.1 mm), 0.4 mm보다 짧게 찍히는 층 0개.
* 질량 표(`slicer_part_masses.json`, 리뷰 F3): 다른 부품과 같은 기준으로 맞췄습니다. 로고를 채운 원래 모듈을 파이프라인 설정으로 슬라이스한 값 58.60 g + 2색 슬라이스 안에서 잰 로고 차이 +0.044 g = 모듈 합계 58.64 g. 주황 0.902 g, 흰 몸통 57.74 g (모두 M).
* 시간 합계: 위 표의 모듈 시간은 2색 프로젝트(P2S 프로필을 상속까지 푼 설정)의 시간이고, 다른 부품은 기존 파이프라인 설정의 시간입니다. 그래서 출력 계획 합계의 차이는 위의 '한 색일 때 대비 +' 값과 같지 않습니다(두 설정의 한 색 시간이 다름).
* 이 2색 프로젝트는 Bambu Studio의 P2S 시스템 프로필을 풀어서(상속 포함) 썼습니다 - 사용자가 여는 그대로입니다. 3MF는 매번 바이트까지 같지는 않습니다(부품 uuid, 예상 시간 몇 초). 비교는 해시가 아니라 G-code 숫자(색 바꿈, 색별 g)로 합니다.
