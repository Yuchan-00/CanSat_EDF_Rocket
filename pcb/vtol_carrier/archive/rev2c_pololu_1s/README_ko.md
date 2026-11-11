# rev 2c 보관본 (ESP-12E + Pololu 5 V 모듈 + 1S 로직 배터리)

이 폴더는 캐리어 기판 **rev 2c** 를 rev 3 작업으로 다시 만들기 전에 그대로 복사해 둔 것입니다.
이 폴더의 파일은 고치지 않습니다.

| 항목 | 내용 |
|---|---|
| 보관한 날 | 2026-11-10 |
| 기준 커밋 | `4dbce6d5db93e67c60fa1267f991fe5647e0ba34` (브랜치 pcb-rev3 의 출발점, "Docs: match the current hardware ...") |
| 보관 위치 | `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/archive/rev2c_pololu_1s` |
| 파일 수 / 크기 | 249개 / 44,268,745 바이트 (M: 복사 스크립트가 센 값) |
| 복사 확인 | 249개 모두 커밋의 git blob id 와 같고, 복사본이 원본과 바이트 단위로 같음 (M) |

## 왜 보관했는가

rev 3 에서는 사양(`spec`), 회로도와 기판(`kicad`), 출력물(`out`)을 스크립트로 다시 만듭니다.
다시 만들면 rev 2c 의 파일이 덮어써지므로, rev 2c 의 근거(배치, 배선 기록, 검사 보고서, BOM)를 먼저 남겼습니다.
rev 3 의 검사 가운데 몇 개는 rev 2c 와 비교합니다(예: MCU 256핀의 넷, 설계 규칙 파일 해시, 크리스털 배선 길이).

rev 2c 에서 rev 3 로 바뀌는 큰 내용 (자세한 것은 `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/REV3_PLAN.md`):

- Pololu S9V11F5S6CMA 5 V 모듈(J6)을 빼고 기판 위 승압 회로(U8 TPS61023, L5, U9 TLV3012B 등 13개)로 바꿈
- ESP-12E 를 JLCPCB 부품 라이브러리의 ESP-12F(C82891)로 바꿈 (같은 풋프린트)
- 손납땜이던 커넥터를 JLCPCB 가 실장하는 부품으로 바꿈

## 들어 있는 것

| 폴더 / 파일 | 원래 위치 (rev 2c) | 파일 수 |
|---|---|---|
| `spec/` | `pcb/vtol_carrier/spec/` (rev 3 계획 파일 2개는 rev 2c 가 아니므로 넣지 않음) | 6 |
| `kicad/` | `pcb/vtol_carrier/kicad/` (회로도, 기판, IR, 배치, `work/` 의 배선 기록) | 110 |
| `out/` | `pcb/vtol_carrier/out/` (거버, BOM / CPL, 보고서, 렌더, STEP) | 114 |
| `scripts_rev2c/` | `pcb/vtol_carrier/scripts/` (rev 2c 를 만든 스크립트) | 18 |
| `README_rev2c_project_ko.md` | `pcb/vtol_carrier/README_ko.md` (rev 2c 설명서) | 1 |

## 주의

- NOT_VERIFIED: rev 2c 기판은 만들지도, 전원을 넣지도 않았습니다. 이 폴더의 "통과"는 도구 검사 결과일 뿐입니다.
- rev 2c 문서에는 서로 맞지 않는 곳이 있습니다(기판 층수 2 / 4, 크리스털 부하 커패시터 22 pF / 27 pF,
  실장하지 않는 헤더의 populate 값). 보관본은 고치지 않았고, rev 3 사양에서 바로잡습니다.
- `scripts_rev2c/` 의 스크립트는 기록용입니다. 여기서 실행하면 경로가 맞지 않습니다.
