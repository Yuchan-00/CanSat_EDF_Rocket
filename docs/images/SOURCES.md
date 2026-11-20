# README 그림 출처

README에 넣은 그림은 모두 아래 원본을 자르거나 줄인 복사본입니다. 예외는 `pcb_schematic.jpg` 하나로, 저장소의 회로도 파일을 kicad-cli로 다시 내보낸 그림입니다(아래 표).
렌더는 CadQuery 형상을 PyVista(VTK)로 그린 화면용 그림이고(색은 화면용), 영상은 시뮬레이션이며 실제 비행 영상이 아닙니다.
만든 스크립트: 작업 폴더의 `build.py`(저장소 밖; 2026-11-17 PCB 그림은 `build_images.py`와 `sch_render.py`, 역시 저장소 밖). sha256은 전체 64자리입니다.
`CanSat_v0.9/`로 시작하는 원본은 저장소 밖의 결과물 폴더에 있고, 나머지 경로는 저장소 기준 상대 경로입니다.

| 파일 | 크기 | 만든 방법 | 원본 | 원본 sha256 | 파일 sha256 |
|---|---|---|---|---|---|
| `hero_rocket.jpg` | 25 KB | 로켓 대표 렌더, 오른쪽 기체 부분만 자름 | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/v09_10_대표_로켓.png` | `5cb559b694dbbb71ae655d58caa144dd358b12ab0464d75e6c12bbd88fe6eb2f` | `684e1b4349dbb15d53314785f1a3e10b3ab03e97e8d019e20cdca3683270b167` |
| `rocket_layout.jpg` | 76 KB | 로켓 구성 (외관 + 1/4 잘라냄), 1200 px로 줄임 | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/v09_02_로켓_구성.png` | `07f2d11ef5a97b6c60a39ff2a14a00399d2fea29f965ae49b31e6c1b51282dd3` | `a96d04cf779e109e88ae3a6740ed49e5ac25e4635cfa44e5ba2f0fc929fb9dae` |
| `vane_section.jpg` | 90 KB | 직접 구동 베인 단면, 1200 px로 줄임 | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/v09_04_직접구동_베인_단면.png` | `2da606fb02c5681cc27a32c2541c9c6e5ff1da8373ef002d04bb5b08092bffc9` | `e20f4287477811e206874a2edeb3ce05f60d667a7efcf841dc06d7ad6103ca68` |
| `vane_exploded.jpg` | 89 KB | 직접 구동 베인 분해도, 1200 px로 줄임 | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/v09_05_직접구동_베인_분해도.png` | `a77b89942d50fa58648241182c4c2892eba1cb20143f0c22c2083006e16a67dc` | `2fad203a34b79913c60d5db0fb8002e6cad53716f20edd6f374427eed032709c` |
| `lead_section.jpg` | 129 KB | 모터 전원선 통로 단면 (시계 장착), 1200 px로 줄임 | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/v09_06_모터전원선_통로_단면.png` | `1860569f25d5dbd964c2e62cfff770d43e21e38cd8dbe0ced25efd208a2425de` | `cc643507b65e085b6119ef4623367bd7792bdd746c6c7f272b2f9c7119110be0` |
| `lead_range.jpg` | 116 KB | 모터선 통로 여유 범위, 1200 px로 줄임 | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/v09_06b_모터선_통로_여유범위.png` | `30309d0814d11a98815ed3f2620bd425250bf47c9b464bad1220eda0188f025f` | `0aae460b8778a59188a46eecd50b2e4213724339acb4c2a4c7649ebcdcc72902` |
| `side_rocket.jpg` | 73 KB | 로켓 옆모습 (로고 양쪽), 1200 px로 줄임 | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/v09_09a_로켓_옆모습_로고.png` | `0609f1df31baa8a547e3a2a3ba58f9f08b933340173e54faa5d9376d10845698` | `ce58529237530b420fe725a333ca5982efbf5473a48816769ec21e8247d89589` |
| `logo_closeup_rocket.jpg` | 92 KB | 로고 확대 (로켓), 1200 px로 줄임 | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/v09_08a_로고_확대_로켓.png` | `292f1c65620305b5fbe743434ea36c4c76ed5c87ae5f0cdf00b6310fc44fac45` | `ad32eb7ed804f1d85c08a81c54592319ff55c94d5976099823407af50a012d1a` |
| `logo_refined.jpg` | 23 KB | 고른 launch 세로판을 다듬은 최종 평면도, 1200 px로 줄임 | `(작업 폴더 scratchpad)/livery/refine/flat.png` | `56d97f6a93f118f3634fa9ba7ac6941db6592c5160f2f2de5ccf07d12b986b6f` | `6c1659d295d1a5143299d2324b47e7413f6fd834fcbf200bafc13bb8fbe0c1a3` |
| `process_black_rocket.jpg` | 70 KB | 흰색 도색 직전(2026-11-04 오전, 검정 PLA) 로켓 구성 렌더, 1200 px로 줄임 | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/수정전_보관_흰색도색/v09_02_로켓_구성.png` | `7e5df896b892ac5935b114f5a49a24de599f895419fc25d625e0a863be52353e` | `33981554c1bf659c65ff6e8ab2a1ca03faf5be8e3c86435135159e0b041e0df3` |
| `lead_evolution.jpg` | 117 KB | 모터선 통로 단면 5판(10-28 처음, 10-28 수정, 10-29 실물 사진 기준, 11-04 시계 장착 검정, 11-04 흰색)의 '전체 경로' 칸만 잘라 나란히 놓음 | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/수정전_보관/v09_06_모터전원선_통로_단면.png` | `436ee099922ae8a849b42104caecaa1fc94ceaade94406f2d0429ee77295e2b0` | `519b9462ef5ac2ab0b9ad88659560d06f6a94a50844506bb69975c39e51ea1a0` |
| ↳ | | | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/수정전_보관_모터선출구/v09_06_모터전원선_통로_단면.png` | `d81e108e170a2e5eba012c5f6da72a8e2b256dca7f8a46b401af9c4eef8dacea` | |
| ↳ | | | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/수정전_보관_모터선출구_2/v09_06_모터전원선_통로_단면.png` | `3d080ac48ceb8443875e76aea36390a8c68817eb38752ff63b586ad6b6c9bdef` | |
| ↳ | | | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/수정전_보관_흰색도색/v09_06_모터전원선_통로_단면.png` | `367ff84cd06cac54f34dfb0c4d71fb6bb37a35f3cecafc7ea831bee801fc1351` | |
| ↳ | | | `CanSat_v0.9/06_3D_CAD/CAD_v0.9/렌더/발표용_1920x1080/v09_06_모터전원선_통로_단면.png` | `1860569f25d5dbd964c2e62cfff770d43e21e38cd8dbe0ced25efd208a2425de` | |
| `fusion_v063.jpg` | 94 KB | Fusion 360 스크립트 v0.6.3 실행(2026-10-20) 미리보기, 900 px로 줄임 | `cad/exports/fusion/preview.png` | `f1a76cf92dad5fb11d0679d5e9f4f8053f786714e1e677b2c8a00c6277381eca` | `4aa58b78d043565335768d8f26c054f5c286b3ad204829a9d7f3209408c7c9e9` |
| `gns_board.jpg` | 83 KB | v0.8.1 GNS 보드 배치 그림, 1200 px로 줄임 | `cad/exports/gns_board_layout.png` | `923db0ded89b7c9ca28e4f1288f2f47229b370652de38e22fe338cf9082af751` | `de7a6d805581301899c3089fc83ccf5044f964a47a8449130542ed271039d8f0` |
| `edf_photo.jpg` | 91 KB | 사용자가 찍은 실물 EDF 사진, EDF 주변만 자름, EXIF(촬영 정보) 없이 새로 저장 | `docs/실물사진/EDF_모터선_출구_2026-10-29_2.jpg` | `16458c23acb60e1a921d06f0a5047d2898e566911c709779290833b1e933161a` | `831506d8f692d7425b1742446585fc6bf669e443d6cff7218850ba287cc7695d` |
| `pcb_carrier.jpg` | 155 KB | 캐리어 PCB rev 3.1 윗면 3/4 렌더 (kicad-cli, 모든 부품 3D 모델), 1200 px로 줄임 | `pcb/vtol_carrier/out/presentation/pcb_iso_front_2400.png` | `0b348f218c655127c1f34b07d55dd86d1d5219023f0662337755bb3e2e0875c0` | `0b8598407a1e8de4de14544e6b82f34c58d83cc92eb9da43cb148050563aa2ca` |
| `pcb_top.jpg` | 181 KB | 같은 보드 윗면 렌더 (kicad-cli), 1200 px로 줄임 | `pcb/vtol_carrier/out/presentation/pcb_top_2400.png` | `e740f0c72e2302d3984139fe44825fc86c261a1da3930c7ce3ed30638a6e8027` | `db4b311128f36b2e317dfe5d28709fdabd88ec6849a9c24bedcf07ddf5e1fba1` |
| `pcb_bottom.jpg` | 131 KB | 같은 보드 아랫면 렌더 (kicad-cli), 1200 px로 줄임. 아랫면 실크의 연락처(이메일) 줄을 둘레 솔더마스크 색 사각형으로 가림 (원본 픽셀 x 1842–1896, y 385–872) | `pcb/vtol_carrier/out/presentation/pcb_bottom_2400.png` | `db67f281aca5d2e38f63039dd3032a18d42c096d589077013086ae698aac8eb3` | `d43ce08a0c2e1b45ee622abc19f430545ec38025e15865638f662fe8c93eab70` |
| `pcb_mounted.jpg` | 103 KB | 직접 장착한 뒤 앞쪽 비스듬히 (MCU, ESP-12F, 스루홀 커넥터, GY-521, BMP280, OpenLog, GPS 플러그; 모듈은 가정 치수), 1200 px로 줄임 | `pcb/vtol_carrier/out/render/b2_completed_front.png` | `14f2202687451bfca1af643d4610b62b3dfec24e8aa49bf73443d308fdd88d9f` | `0868d7d27c8c5df9ee1559d6d41bc083f1304ce20fc3d77f4d98ae213dd6f633` |
| `pcb_delivered_vs_completed.jpg` | 275 KB | 설명 슬라이드 e1: JLC에서 받은 상태 vs 직접 장착한 뒤 + 누가 무엇을 하는지, 1600 px로 줄임 | `pcb/vtol_carrier/out/render/e1_delivered_vs_completed.png` | `336912dd2468f6dfd46347a17a63d7831daf31b4b483a47430e7bc1452042cf6` | `7015869da21da812c360f25566b44ee870e27c6bae61f57bbd0c2102f6beaeba` |
| `pcb_schematic.jpg` | 66 KB | 회로도 8장 모음, 1600 px로 줄임. rev 3.1 회로도를 kicad-cli `sch export svg`로 내보내고 Edge(headless)로 3000 px PNG를 만든 뒤 4 × 2로 모음 (저장소의 `out/presentation/sch_*.png` 중 FCU · IO 시트가 rev 3.1 이전 그림이라 다시 그림) | `pcb/vtol_carrier/kicad/vtol_carrier.kicad_sch` | `b31b5902acdd68c700415401dd1135cd26b991a769d7f34f9abfd5b463d714e1` | `96a27a9ed78c94e6f5954e6b7fbdc7481ff1b61096d065f6efbed06cf109a752` |
| ↳ | | 중간 PNG (저장소 밖) | `vtol_carrier.png` | `19bb50070710cd0a825d3ca9224f6a697b233c7c84d1fa7b1f0e1c64c8f52455` | |
| ↳ | | 중간 PNG (저장소 밖) | `vtol_carrier-Power.png` | `926ca8cf0431224f38529af0f38724f0c737e48aac5ff9b66073ca62732d35c3` | |
| ↳ | | 중간 PNG (저장소 밖) | `vtol_carrier-FCU.png` | `cb46b2c8c8b7734899013538a94cc6fca6bcc3a09a2b48111c03903eeb404b89` | |
| ↳ | | 중간 PNG (저장소 밖) | `vtol_carrier-NAV.png` | `c77a938028891d7e3144d2ee5128a38dae1cbca14a5e3bf98a21699ef9aa05f8` | |
| ↳ | | 중간 PNG (저장소 밖) | `vtol_carrier-COM.png` | `771206213fd3179704908c44c99df01dfc95f543f441f0d7fb0cce40e09bda04` | |
| ↳ | | 중간 PNG (저장소 밖) | `vtol_carrier-GNS.png` | `771c57d6f0e040ca39efc6afc21fe64269c197ee4f4feb274f3da2fc56e940f2` | |
| ↳ | | 중간 PNG (저장소 밖) | `vtol_carrier-Level_shift.png` | `da74ca6bb7d8d0941587e8a1b01873d85081c79fc6fa22c1f9b0f4e0bcdca791` | |
| ↳ | | 중간 PNG (저장소 밖) | `vtol_carrier-IO_connectors.png` | `e9be71bc6ee050e0ade94fa698766f4b4d60079b591105ba833a095e89b0496a` | |

* 실물 사진(`edf_photo.jpg`)은 EDF 주변만 잘라 EXIF 없이 다시 저장했습니다. 원본(`docs/실물사진/`)은 손·책상이 보여서 공개 저장소에 올리지 않습니다(.gitignore, 로컬에만 있음).
* `(작업 폴더 scratchpad)`는 로고 작업 때 쓴 임시 폴더입니다. 같은 로고의 최종 데이터는 `cad/livery/`에 있습니다.
* PCB 그림(2026-11-17)의 기판: `pcb_mounted.jpg`, `pcb_delivered_vs_completed.jpg`의 원본은 `pcb/vtol_carrier/out/render/render_report.json`의 `source_board_sha256` = `0986418f8f24d9933acf771afaad9528e17b084a3799a530b40c08e068021c1f` (지금 저장소의 `vtol_carrier.kicad_pcb`와 같음, M). `pcb_carrier.jpg`, `pcb_top.jpg`, `pcb_bottom.jpg`의 원본(`out/presentation/`)은 같은 전체 빌드(2026-11-13 21:14–21:18)에서 만들어져 같은 기판으로 봅니다(E; `presentation_log.json`에는 기판 해시가 없음, 그림에 rev 3.1 실크가 보임).
