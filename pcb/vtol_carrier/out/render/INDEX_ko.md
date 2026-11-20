# 캐리어 기판 rev 3 - 장착 그림 목록

이 폴더(B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/out/render)의 그림은 모두 최종 배선 기판
B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/kicad/vtol_carrier.kicad_pcb
으로 만들었습니다. scripts/make_all.py 가 그림을 다시 만들 때마다 그때의 기판을 쓰고, 쓴 기판의 sha256을
render_report.json 의 source_board_sha256 에 적습니다 (2026-11-12 납품 시점: 131c4d63d10c5158ca873b255b21a47d089b56b5a63e1a9d775ce273fb8ec556,
승압부 재배치와 아랫면 실크 두 줄이 들어간 최종 기판, M. 같은 날의 변경 뒤: 588fb390b516894c1f9e82f9e9767a2582520e57ff55cd6bf820e169490c6518 -
표면처리 표기 한 줄만 'HAL lead-free' 로 바뀌었고 구리·배치·실크는 131c4d63과 같음, M. 04:26 이후의 지금 기판:
cc750ce9e23322b295687ed662a24f2a7e924d10c8ceb610b7e668cc46b414d2 - 아랫면 실크 이름표 3개(J14, J2, J7)의 핀 글자 순서만
아랫면에서 본 패드 순서로 바뀌었고 구리·배치는 같음, M. 아랫면 그림 a3에 새 글자가 보입니다).

**2026-11-12 재설계 뒤의 지금 기판 (이 줄이 최신; 위의 sha256은 재설계 전 기록):**
bbd15b37c6c3a418045b0ba28f09a82fd97eaf345bd29fedf819e555f2ba383d (M, render_report.json source_board_sha256 와 대조할 것).
기판 -y 변(그림 a1 / b1의 아래쪽 변)에 승압 회로용 탭(KiCad x 117.3~132, y 135~142.2 mm; 외접 76 x 77.2 mm)이 생겼고,
U8, L5, C41, C57, R63, C58, R64가 그 탭 위에, C43이 탭 바로 위 본체에 있습니다. 프로그래밍 구멍 J19, J22, J26은 0.55 mm 아래(+y)로,
C42는 0.20 mm 오른쪽(+x)으로 옮겨졌고 배선은 새로 했습니다. 그림은 모두 이 기판으로 다시 만들었습니다 (make_all.py 의 renders / d2 / slides 단계).
이 문단은 손으로 덧붙인 것이고 숫자의 출처는 out/reports/board_metrics.json board_fit.outline 과 kicad/placement.json 입니다.
2026-11-12 변경: ESP-12F(U7)와 스루홀 커넥터 12개(J2, J5, J8~J12, J14~J18)는 JLC가 아니라 사용자가 사서 납땜합니다 (JLC는 SMT 부품만 실장).
그래서 '받는 상태' 그림(a1~a3)에는 ESP-12F와 스루홀 커넥터가 없고 (빈 패드와 빈 도금 구멍만 보임; SMD 커넥터 J7은 있음), '장착한 뒤' 그림에만 있습니다.
기판 파일은 고치지 않고, 그림용 사본 세 개만 만들어 그렸습니다.

- B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/kicad/render_as_delivered.kicad_pcb : JLC에서 받는 상태 (사용자가 붙이는 부품 U1~U4, U7, J1, J2, J5, J8~J12, J14~J18의 3D 모양을 숨김; 목록은 사양 파일의 fit 값에서 읽고 render_report.json 의 hidden 에 적음)
- B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/kicad/render_completed.kicad_pcb : 직접 장착한 뒤 (MCU 4개, ESP-12F, 스루홀 커넥터가 보이고, GY-521, BMP280, OpenLog, GPS 플러그 모델을 붙임)
- B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/kicad/render_completed_limit.kicad_pcb : 장착한 뒤 + 13.4 mm 높이 한계를 나타내는 빨간 테두리

**중요:** 이 그림들은 설명용입니다. 모듈 치수 대부분은 판매 페이지 값이거나 가정 값(B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/module_dimensions.json 에서 A 표시)이라서,
그림에서 모듈이 들어가 보인다고 해서 실제로 맞는다는 뜻이 아닙니다. 그림 속 '가정'(빨간 글씨)은 이런 값으로 그린 부분입니다.
OpenLog와 BMP280이 실제로 들어가는지는 모듈을 캘리퍼스로 잰 뒤 맞춤 검사로 정합니다.

## 그림 목록

| 파일 | 보여 주는 것 | 가정한 것 |
|---|---|---|
| a1_as_delivered_top.png | JLC에서 받는 상태, 윗면. SMT 부품만 붙어 있음. MCU 자리 U1~U4, ESP-12F 자리 U7, 스루홀 커넥터 자리 12곳(J2, J5, J8~J12, J14~J18), J1 구멍 8개, 프로그래밍 헤더 자리 8곳(J3, J19~J22, J24~J26)이 비어 있음. 주문 안내서 7장의 커넥터 방향 표는 이 그림의 위/아래/왼쪽/오른쪽을 기준으로 함 | 없음 (부품 모양은 KiCad 라이브러리 3D 모델) |
| a2_as_delivered_front.png | 받는 상태, 앞쪽 비스듬히 | a1과 같음 |
| a3_as_delivered_bottom.png | 받는 상태, 아랫면. 부품도 다리도 없음(빈 도금 구멍과 아랫면 인쇄 글자만) | 없음 |
| b1_completed_top.png | 직접 장착한 뒤, 윗면: MCU 4개, ESP-12F(U7), 스루홀 커넥터 12개, GY-521(J1), BMP280(J2), OpenLog(J5), GPS 플러그(J7) | 모듈 외형, 높이, 방향(가정). GPS 리드선 모양과 방향(가정). J14(Molex) 3D 모양: KiCad 라이브러리에 3D 파일이 없어 프로젝트에서 만든 모양(가정). 스루홀 다리 길이는 라이브러리 3D 모양 값 |
| b2_completed_front.png | 장착한 뒤, 앞쪽 비스듬히 | b1과 같음 |
| b3_completed_back.png | 장착한 뒤, 뒤쪽 비스듬히 | b1과 같음 |
| b4_completed_side_limit.png | 장착한 뒤 옆면(정투영). 빨간 선 = 기판 윗면에서 13.4 mm 높이 한계. 가장 높은 것은 GPS 리드선 굽힘 12.75 mm, 그다음 GY-521 핀 끝 8.54 mm | 두 높이 모두 가정 값으로 그린 것 |
| c1_gy521_on_J1.png | GY-521을 J1에 납땜: 부품 면 위, 1번 핀 VCC, 모듈 밑면 2.54 mm(핀 스트립 플라스틱 높이) | 핀 순서(판매 페이지), 모듈 크기, 받침 높이(가정). 계획 문서의 '약 2 mm 받침'과 그림의 2.54 mm가 다름 |
| c2_bmp280_in_J2.png | BMP280을 ㄱ자 헤더로 J2 소켓에 꽂아 눕힌 모습, 모듈 밑면 2.5 mm, 센서 면이 기판 쪽 | 헤더 종류(ㄱ자), 핀 인쇄 순서, 센서 면 방향(가정, 확인 안 됨) |
| c3_openlog_in_J5.png | OpenLog를 microSD 면이 위로 오게 J5에 꽂은 모습 | 맞춤 전체가 가정. 실제로 들어가는지는 실측으로 결정. 모듈 아래 부품 높이, 카드 돌출 길이(가정) |
| c4_esp12f_keepout.png | ESP-12F(사용자가 납땜한 뒤의 모습)와 안테나 아래 구리 금지 영역(기판 파일의 keep-out: x 74.63~98.87 mm, y 65~72.4 mm, 4개 층 모두). 3D 그림에서는 구리가 없는 빈 녹색 영역으로 보이고, e4에는 빨간 점선으로 표시 | 없음 (영역은 기판 파일 값) |
| c5_mcu_pin1.png | U3(COM) ATmega128A, 칩의 1번 핀 점과 기판 실크의 삼각형 표시 | 없음 (칩 모양은 KiCad 라이브러리 3D 모델) |
| d1_gps_plug_J7.png | J7(JST-GH 4핀)에 꽂은 플러그와 리드선 시작 부분. 빨간 선이 J7 1번 핀(GPS_VCC) | 선 색, 굽힘, 리드선 방향(가정). 플러그 높이 7.3 mm는 JST 카탈로그 참고값 |
| d2_neo7m_top_cap.png | 차체 윗뚜껑 위의 NEO-7M과 J7까지 내려오는 리드선(차체 형상은 cad/layout.py) | NEO-7M 외형은 NEO-6M 일반형 35 x 25 mm 보드로 그림(가정), 방향, 리드선 경로(그린 길이 177 mm, 차체 CAD 예산 253.9 mm). 간섭 검사는 하지 않음 |
| e1_delivered_vs_completed.png | 슬라이드: 'JLC에서 받은 상태' 와 '직접 장착한 뒤' 비교 + 누가 무엇을 하는지 표 | 장착한 뒤 그림의 모듈 치수(가정) |
| e2a_gy521_J1.png | 슬라이드: GY-521 → J1. 받은 상태/장착 후 윗면, 확대 그림, J1 핀 표(spec 파일에서 읽음), 누가 납땜하는지 | 모듈 쪽 핀 이름 순서(판매 페이지, 가정), 받침 높이 |
| e2b_bmp280_J2.png | 슬라이드: BMP280 → J2, 같은 구성 | 헤더 종류, 핀 인쇄 순서, 센서 면 방향(가정) |
| e2c_openlog_J5.png | 슬라이드: OpenLog → J5, 같은 구성. 핀 이름 순서는 OpenLog.brd 인쇄 순서(문서 값) | 맞춤(가정, 실측으로 결정) |
| e3_gps_lead_J7.png | 슬라이드: GPS 리드선. d2 그림, J7 핀 표(신호, 선 색, 모듈 핀), d1 확대 | 선 색, NEO-7M 쪽 핀 순서(확인 안 됨), 리드선 경로 |
| e4_overview_callouts.png | 슬라이드: 윗면 전체에 설명 표시(JLC가 실장하는 것과 사용자가 하는 것 - 표시는 사양 파일의 fit 값에서 읽음: 커넥터 J14, J8~J11, J2, J5는 '사용자 납땜', J7과 승압 회로는 'JLC'), 안테나 구리 금지 영역 빨간 점선 | 모듈 치수(가정) |
| render_report.json | 그림 a1~d1의 명령, 사본 기판의 해시, 붙인/숨긴 3D 모델 목록 | - |
| slides_report.json | 슬라이드 e1~e4가 쓴 입력 그림과 결과의 해시 | - |
| d2_neo7m_top_cap.json | d2 그림의 모든 숫자와 출처 표시 | - |
| _spike_rev2c/ | rev 2c 시험용 사본으로 만든 예전 d2 시험 그림. **최종 그림이 아님** (보관용, 결과물 폴더에는 복사하지 않음) | - |

그림 a1~d1의 실제 크기는 1904 x 1064 픽셀입니다(이 PC의 kicad-cli가 -w 1920 -h 1080 요청에 이 크기로 씀, 측정값).
d2와 슬라이드 e1~e4는 1920 x 1080 픽셀입니다.

## 다시 만드는 방법

명령은 B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/scripts 폴더에서, 환경 변수 PYTHONDONTWRITEBYTECODE=1 로 실행합니다.

1. 모듈 3D 모델(모듈 치수 파일이 바뀌었을 때만):
   `"B:/0)AI_Cad/CanSat_EDF_Rocket/.venv/Scripts/python.exe" -B -X utf8 make_module_models.py`
2. 그림 a1~d1 (약 3~5분):
   `"C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/python.exe" -B -X utf8 render_rev3.py --quality high`
3. 그림 d2 (1~4분):
   `"B:/0)AI_Cad/CanSat_EDF_Rocket/.venv/Scripts/python.exe" -B -X utf8 render_d2_scene.py`
4. 슬라이드 e1~e4:
   `"C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/python.exe" -B -X utf8 render_rev3.py --slides-only`

## 확인하지 못한 것

- 모듈 실물 치수(GY-521, BMP280 TYE-BMP29, OpenLog, NEO-7M SZH-NT07): 하나도 재지 않았습니다.
- 모듈과 주변 부품의 간섭: 그림은 맞춤 검사가 아닙니다.
- 차체 안 GPS 리드선 경로와 배터리·트레이와의 간섭.
- J14 Molex 커넥터의 실제 3D 모양(라이브러리 3D 파일 없음).
- JLC의 DFM 결과와 견적, 실제 납땜 결과.
