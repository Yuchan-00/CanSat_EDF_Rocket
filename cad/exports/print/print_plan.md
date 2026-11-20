# 출력 계획 (Bambu Studio CLI로 슬라이스한 실제 값)

원본: Fusion 360 `CanSatEDFRocket.py 0.6.3` 내보내기, 프리셋 `BUDGET_90mm_6S`, params.json sha256 `5ec0ed291081…` (`python tools/make_print_files.py --check`로 최신 여부 확인).

프린터 `Bambu Lab P2S 0.4 nozzle`, 공정 `0.20mm Standard @BBL P2S` 기반 + wall_loops 5, top/bottom 5 layers, 15 % gyroid, tree(auto) supports where listed (except launch_pad: normal(auto), intake_module: normal(auto) / snug / support_expansion -0.5 mm), auto brim; intake_module: Bambu project with a support blocker over every window opening, 필라멘트 `Bambu PLA Matte @BBL P2S` (밀도 1.31 g/cm³).

| 부품 | 출력 파일 | 수량 | 출력 방향 | 서포트 | 크기 (mm) | 1개 출력 시간 | 1개 필라멘트 |
|---|---|---|---|---|---|---|---|
| propulsion_module | `propulsion_module.stl` | 1 | upright, exhaust exit on the bed; pod undersides are 45 deg chamfers | 없음 | 176.2 x 176.2 x 190.0 | 7 h 54 min | 370.0 g |
| intake_module | `intake_module.3mf` | 1 | printed flipped (SPEC 5.1 / 12): the spigot stands on the bed; the 45 deg floor wedges keep supports out of the closed lead channels only in this orientation (upright, the channel roofs would need support inside); v0.6.2: every window ceiling is a 45 deg truncated gable whose flat is bridged, and the part is a Bambu project with a support blocker over each window opening (snug supports, shrunk 0.5 mm), so no support inside the windows - checked on the sliced G-code; supports stay under the bulkhead inside the spigot, under the shelf ledge, the spigot step and the lead bulges | 일반 (밀착형) (`normal(auto)`, `snug`) | 127.9 x 120.0 x 172.9 | 4 h 46 min | 214.6 g |
| avionics_module | `avionics_module.stl` | 1 | upright | 없음 | 120.0 x 120.0 x 200.0 | 2 h 54 min | 125.6 g |
| payload_module | `payload_module.stl` | 1 | upright; the bulkhead 15 mm above the bed and (v0.6) the four COM-board bosses under it need short supports inside the skirt | 트리 (`tree(auto)`) | 138.3 x 120.0 x 198.4 | 5 h 39 min | 199.5 g |
| nose_cone | `nose_cone.stl` | 1 | base down; the anchor bar spans the bore 4 mm above the skirt and needs supports | 트리 (`tree(auto)`) | 126.0 x 120.0 x 155.0 | 1 h 32 min | 70.9 g |
| avionics_tray | `avionics_tray.stl` | 1 | flat | 없음 | 160.0 x 100.0 x 2.0 | 0 h 45 min | 41.4 g |
| piston | `piston.stl` | 1 | flat, spring recess facing up | 없음 | 116.6 x 116.6 x 2.4 | 0 h 34 min | 29.3 g |
| vane | `vane.stl` | 4 | flat; the D5 shaft stands 1 mm proud of the 3 mm plate -> supports on build plate | 트리 (`tree(auto)`) | 46.0 x 32.0 x 5.0 | 0 h 09 min | 4.0 g |
| vane_horn | `vane_horn.stl` | 4 | flat on its 8 x 17 face; the D-bore prints vertical (round) | 없음 | 17.0 x 8.0 x 3.0 | 0 h 02 min | 0.4 g |
| latch_arm | `latch_arm.stl` | 1 | flat | 없음 | 10.2 x 13.3 x 3.0 | 0 h 01 min | 0.3 g |
| launch_pad | `launch_pad.stl` | 1 | flipped: stands on the fin-locator ribs, legs up; grid supports fill the 4 mm gap under the plate | 일반 (그리드) (`normal(auto)`) | 190.0 x 190.0 x 114.0 | 5 h 30 min | 202.0 g |

서포트 열은 Bambu Studio가 G-code 머리말에 기록한 `enable_support` / `support_type` / `support_style` 값입니다 (슬라이스하지 않은 부품은 설정값).

**intake_module 창 안 서포트 검사 (v0.6.2, 결정 A)**: **PASS**.
출력 파일 `intake_module.3mf`은 Bambu Studio 프로젝트입니다. 창마다 서포트 차단(support blocker) 볼륨이 들어 있으니 STL 대신 이 파일을 여세요. 슬라이스한 G-code의 서포트 압출 경로를 설계 좌표로 되돌려 창 개구부(window sectors up to the strut faces, r 57.00..60.00, Z 209.90..326.90 (design frame), grown by 0.2 mm (half the support line width) across the faces and 0.1 mm in Z) 안을 셌습니다: 45° 0 mm, 135° 0 mm, 225° 0 mm, 315° 0 mm. 차단 볼륨 없이 슬라이스하면 FAIL, 창 안 서포트 58229.4 mm.

**합계 (수량 반영)**: 30 h 21 min, PLA 1272 g (발사대 제외 기체만: 24 h 50 min, 1070 g)
