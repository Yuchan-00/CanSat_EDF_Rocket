# 임시 출력 파일 (v0.8 GPS 포드 + v0.8.1 GNS 보드) - CadQuery 트윈에서 만든 것

**주의: 임시본입니다.** 공식 출력 파일(`cad/exports/print`)은 Fusion 360에서 스크립트를 실행한 결과로만 만듭니다. v0.8.1 Fusion 실행 전까지 GPS 포드·GNS 보드 때문에 바뀐 세 부품(페이로드 모듈: GPS 포드, 벽·바닥 1.2 mm; 항공전자 모듈: GPS 선 구멍 + GNS 보스 4개; GPS 포드 뚜껑 1.2 mm)만 CadQuery 트윈 STL에서 같은 방식(방향, 슬라이서 설정)으로 만들었습니다. Fusion 스크립트의 피처 호출을 OCC로 재현한 형상이 이 형상과 같다는 것은 확인했습니다 (`cad/fusion360/replay_x3.py`, 불리언 차이 0 mm³). Fusion 실행 뒤 `python tools/make_print_files.py`가 만든 공식 파일로 바꾸고 이 폴더는 지웁니다. 나머지 부품은 v0.6.3 출력 파일을 그대로 씁니다 (형상 변화 없음).

프린터 `Bambu Lab P2S 0.4 nozzle`, 공정 `0.20mm Standard @BBL P2S` 기반 + 벽 5겹, 15 % 자이로이드, 필라멘트 `Bambu PLA Matte @BBL P2S` (Bambu PLA Matte 검정, 밀도 1.31 g/cm³).

| 부품 | 출력 파일 | 수량 | 출력 방향 | 서포트 | 크기 (mm) | 1개 출력 시간 | 1개 필라멘트 |
|---|---|---|---|---|---|---|---|
| payload_module | `payload_module.3mf` | 1 | upright; the bulkhead 15 mm above the bed and (v0.6) the four COM-board bosses under it need short supports inside the skirt; v0.8: the GPS pod's floor rises at 45 deg and its raceway is a vertical tunnel (no support there; 1.2 m of support in the connector pocket's ceiling), its corner bosses bridge 5 mm; normal(auto) supports (the CLI refuses tree(auto) on this part); v0.8.1: pod walls / floor 1.2 mm | 일반 (그리드) (`normal(auto)`) | 193.8 x 120.0 x 198.4 | 6 h 17 min | 277.4 g |
| avionics_module | `avionics_module.3mf` | 1 | upright; v0.8: + the GPS lead hole; v0.8.1: + four 45-deg diamond GNS bosses on the -x wall | 없음 | 120.0 x 120.0 x 200.0 | 2 h 54 min | 124.8 g |
| gps_pod_lid | `gps_pod_lid.3mf` | 1 | flat: lid top face on the bed, the three ground-plane posts up | 없음 | 68.6 x 64.7 x 13.0 | 0 h 13 min | 5.6 g |

**합계**: 9 h 23 min, PLA 408 g
