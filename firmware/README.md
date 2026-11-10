# CanSat EDF Rocket — ATmega128 비행 컴퓨터 펌웨어 (v0.9: NAV · FCU · COM · GNS 네 칩)

> **2026-11-06: 기체 무선이 ESP8266(ESP-NOW 브리지)으로 바뀌었습니다. 0.10절이 무선에 관해 0절의 나머지와 1~15절보다
> 우선합니다(LoRa 설명은 예비 무선의 설명으로 유효).**
> **v0.9 펌웨어 (2026-10-29)는 바로 아래 0절에 있습니다. 0절이 1~15절보다 우선합니다.** 1~15절은 v0.6.3(세 칩, 2026-10-16)
> 때의 설명과 수치입니다. 배선·퓨즈·LoRa 설정·로그 형식처럼 바뀌지 않은 부분은 그대로 유효하고, 칩 개수·시간·크기·검증 수치는
> 0절의 값이 맞습니다. v0.7~v0.8.1(GPS, GNS 칩)은 `B:\Video\펌웨어\v0.8.1_4칩\README_v0.8.1_4칩.md`에 있습니다.

## 0. v0.9 펌웨어 (2026-10-29, 네 칩)

이번 라운드의 결과 전체(한국어, 합격 기준 22개, FRESH3, 실기 측정 계획): `docs/v0.9_펌웨어_결과.md`. 판정 원본:
`fw09/impl/integ/ACCEPTANCE.md`, 마지막 전체 검사 기록 `fw09/impl/fix/runs/final/` (`fw09/` =
`C:\Users\yc012\AppData\Local\Temp\claude\B--Claude\bed4b475-b8b1-46b7-ab64-c3b11e27401e\scratchpad\fw09\`; 복사본
`B:\Video\펌웨어\v0.9\구현\`). 아래 수치는 모두 시뮬레이터 측정(M)입니다. **실제 칩에서 잰 값은 없습니다.**

### 0.1 상태

* 칩 4개(ATmega128A 16 MHz): NAV(센서·자세·고도), FCU(모든 비행 판단·출력·로그), COM(LoRa), GNS(GPS·위치 제어). 칩·배선·무게는
  v0.8.1과 같습니다. 캐리어 기판 rev 2c에는 5번째 칩 자리가 없습니다(사용자가 2026-11-05에 뺌). 그래서 P3(5번째 칩으로
  NAV 일 옮기기)는 기판에 준비된 예비안이 아닙니다. 쓰려면 기판을 다시 설계해야 하고, 사용자가 정합니다(`docs/v0.9_펌웨어_결과.md` 8장).
* 부품과 펌웨어 (2026-11-04 사용자 결정): 기압계는 BMP280 모듈(TYE-BMP29, I2C 0x77, 캐리어 J2)이고, GPS는 NEO-7M(SZH-NT07)입니다.
  2026-11-10 제어 라운드부터 NAV의 기본 기압계 드라이버는 BMP280입니다(`src/hal/bmp280.*`, `src/core/bmp280_math.*`. 선택은
  `src/hal/baro.h`, params `flight_computer.nav.baro_sensor`). 강제 모드, 압력 ×4 / 온도 ×1 오버샘플링, IIR 끔입니다. 데이터시트
  값(A)이고 실제 부품으로는 아직 재지 않았습니다. MS5611 드라이버(`src/hal/ms5611.*`, `src/core/ms5611_math.*`)는 빌드 옵션입니다
  (env `nav_ms5611`). 시뮬레이션의 기압계 모델도 BMP280입니다(`sim/sim6dof.py` `BARO_MODELS`, 잡음 0.44 m는 E). 이 문서 아래쪽의
  MS5611 설명은 그 빌드 옵션의 설명입니다.
  GPS: 펌웨어가 쓰는 UBX 메시지는 NEO-6M과 NEO-7M이 같다고 봅니다(A). MON-VER는 7로 읽습니다(0.10.1절).
* 펌웨어 SPEC(`fw09/spec/SPEC_firmware_v09.md`) 7절의 기준 22개가 모두 통과했습니다(시뮬레이터). 실기 측정(같은 문서 7장) 전에는
  비행하지 않습니다.
* 기준 모델(sim/)에서 바뀐 것은 RP1(통신 규약) 하나입니다. 제어 법칙은 v0.9 시뮬레이션·제어 라운드의 것 그대로입니다.

### 0.2 빌드 구성 (G1: 기체 상수를 구성마다)

VTOL 빌드는 `-D FC_VCFG=<n>`으로 기체 구성을 고릅니다(`platformio.ini`). `params_gen.h`가 그 구성의 CAD 보고서
(`cad/exports/cadquery_report_vtol{,_tether,_hybrid}.json`)에서 만든 상수를 고릅니다. 탄도 빌드는 로켓 상수
(`cadquery_report.json`)를 씁니다. 구성이 없거나(VTOL AVR 빌드) VTOL 이미지에 로켓 상수를 쓰거나, FCU의 `PWR_SRC`가 구성과
다르면 빌드가 `#error`로 멈춥니다.

| 구성 | FC_VCFG | NAV | FCU | GNS | COM | 베인 레버 `MIX_L_V_M` | 호버 스로틀 | `FC_PARAMS_TAG` |
|---|---|---|---|---|---|---|---|---|
| 배터리 (현장 / 시연 / GPS 없음) | 0 | `nav` | `fcu` / `fcu_demo` / `fcu_nogps` | `gns` / `gns_demo` / 없음 | `com` (= `com_lora_uart`), `com_sx127x` | 0.210045 m | 0.730 | 0x09 |
| 유선 | 1 | `nav_wired` | `fcu_wired` | `gns_wired` | `com_wired` | 0.184700 m | 0.673 | 0x67 |
| 하이브리드 | 2 | `nav_wired_buffer` | `fcu_wired_buffer` | `gns_wired_buffer` | `com_wired_buffer` | 0.197319 m | 0.701 | 0x5D |
| 로켓 (탄도) | - | `nav_ballistic` | `fcu_ballistic` | - | `com_ballistic` | 0.351006 m | 0.916 | 0x76 |

(G1 전에는 모든 VTOL 빌드가 로켓 상수 0.351006 m / 0.916 / 0x95로 만들어졌습니다. 출처: `fw09/impl/D/runs/cfg_macro_compare.txt`.)

* 유선·하이브리드 FCU는 GPS 없이 납니다(`ONB_NO_GPS`). `gns_wired*`는 설정 표시를 기체에 맞추려고 만든 이미지입니다.
* **섞인 칩 거부(검토 뒤 수정 F1, `src/main.c`, VTOL만)**: FCU가 NAV 프레임의 불일치 표시(NAV_RC_TAG_MISMATCH 0x20)나 COM IL_STAT의
  다른 `params_tag`를 보면 표시를 걸어 둡니다. 그동안 SAFE·ARMED(모터 꺼짐)에서는 감독 코드의 시동 스위치 입력을 열린 것으로
  넣습니다: ON은 `LP_REJ_ARM_SWITCH`로 거부되고 TELEM의 ARM_SWITCH 표시가 열림으로 보입니다. 비행 중에는 아무것도 바꾸지
  않습니다. 진짜 원인은 로그(COM 글 "MISMATCH", NAV 리셋 원인 0x20)와 NAV LED에 남습니다. 공동 시뮬레이션(유선 FCU + 배터리
  NAV·GNS·COM): 52 s 내내 SAFE, ON 100번 모두 응답·거부.
* **GNS는 못 잡습니다**: glink 프레임에 설정 표시가 없습니다(넣으려면 `sim/glink_proto.py` 규약 변경). GNS를 구운 뒤 해시를
  `B:\Video\펌웨어\v0.9\구현\FINAL_HASHES.txt`와 비교하세요.

### 0.3 nlink 타이밍 (P1, NAV → FCU)

* 전송(72 B, 2 MHz SPI)은 이제 NAV의 Timer3 compare-B 인터럽트가 한 바이트씩 보냅니다(I3). 바이트 간격 140 클럭(`NL_PACE` 107 +
  33), 전송마다 TCNT3 = 0, 72번의 인터럽트 ≈ 4.0k 클럭(250 µs)의 CPU. 예전의 바쁜 대기(~10k 클럭)가 없어졌습니다
  (`src/hal/nlink_spi.c` 머리 주석).
* SS 낮은 시간: 단일 칩 벤치 618.44~620.56 µs(`fw09/impl/fix/runs/final/all_benches_nav_0.log`), 칩 계산 ~632 µs, 4칩
  공동 시뮬레이션 636~641 µs(`params.json` `flight_computer.nlink.exchange_watchdog_note`).
* 밀어내기 하한 `NAV_PUSH_MIN_US_VTOL` 1310 → **890 µs**(`src/nav/nav.h`), FCU 벤치의 가짜 NAV `SB_NAV_PUSH_US_VTOL` 1560 → 1140 µs.
  NAV 벤치가 하한을 다시 계산해 `nav.h`와 같은지 검사합니다(890 = 890). 전송 앞 NAV 코드를 바꾸면 하한도 다시 정해야 합니다.
* 틱 사이 프레임 시각 최대 변화 211.2 µs(FCU 프레임 시간초과 4.6 ms의 여유 600 µs 안). 프레임 시간초과 4.6 ms는 그대로입니다.
* CRC-16/CCITT-FALSE는 256칸 표로 계산합니다(I5, 값은 같음). 쿼터니언 적분은 Q30 / Q32 고정소수점(I1, `src/core/qfix.h`
  `fx_mulhi`)이고, `sim/quat.py integrate`와 같은 급수·뉴턴 창·실수 대체 경로를 씁니다.
* 결과: NAV 필수 작업(틱 → FCU 응답, MS5611 버스 포함) 최대 **2782.2 µs (69.6 %)** / 평균 2499.9 µs; 시간 중요 구간(틱 → 프레임
  전달) 1759.2 µs (44.0 %); 스택 여유 2678 B(`fw09/impl/fix/runs/final/all_benches_nav_0.log`). v0.8.1은 4099.2 µs(102.5 %)였습니다
  (`history/v0.9_pre_fw/README.md`).

### 0.4 W1 전송 감시 타이머 (NAV, `src/hal/nlink_spi.c`)

* NAV가 `nlink_spi_exchange_wait`에서 기다리는 동안 TCNT3(원자적으로)와 TOV3를 봅니다. TCNT3 ≥ `NLINK_WD_COUNTS`(16000 = 1000 µs,
  `params.json` `flight_computer.nlink.exchange_watchdog_us`) 이거나 TOV3가 서 있으면, 인터럽트를 막고 아직 열려 있을 때만:
  compare-B 인터럽트 끔, SS 올림, Timer1 시각 기록, 버퍼 0번 바이트 지움(섞인 FCU 프레임을 동기 바이트에서 거부), busy = 0,
  작동 1번 셈(`nlink_spi_stats()->resyncs`). 가장 늦은 끊김은 max(시작 + 1000 µs, 루프 7단계의 기다림)입니다.
* 생성기 검사: 1000 µs > 1.5 × 전송(945 µs) 그리고 < 틱 − 밀어내기 하한(3110 µs). `nlink_spi.c` / `nav_main.c`의 `#error`도 같은 것을 봅니다.
* 부팅 검사: 모든 드라이버 초기화 뒤 Timer3가 nlink 것인지(`TCCR3A == 0`, `TCCR3B == CS30`, Timer3 인터럽트 없음) 봅니다. 아니면
  NAV는 전송을 시작하지 않고(FCU에게는 NAV가 없는 것) LED가 약 16 Hz로 깜박입니다. Timer3를 쓰는 다른 드라이버(`hal/pwm.c`,
  `hal/gps_uart.c`, glink 벤치 배선)는 NAV 비행 빌드에서 컴파일 단계에서 막힙니다.
* 단일 칩 벤치 빌드(`NLINK_HW 0`)에는 감시 타이머가 없습니다(Timer3를 벤치 배선과 나눠 씀, 멈춤 원인이 없음).
* FCU는 끊긴 전송을 재동기(짧은 전송) 또는 프레임 없음으로 봅니다: 프레임 하나를 잃고 원래의 시간초과 틱으로 넘어갑니다.
* 공동 시뮬레이션 고장 `nl_lose_cmp`(전송 30번째 바이트에서 compare 인터럽트를 지움): 작동 1번, 시간초과 1 + 재동기 1, 비상정지
  없음. 감시 타이머 없는 나무(now1)에서는 같은 고장이 I9 ABORT(설계대로의 대조). 정상 실행 작동 0번. 비용 0.0 / +0.9 µs.
  (`fw09/impl/fix/runs/final/rows_summary.txt`)
* 작동 횟수는 nlink 프레임에 없습니다(규약 변경 필요, 결정 대기). 실기에서는 FCU의 재동기·시간초과 수로 간접 확인합니다.

### 0.5 RP1 전원 텔레메트리 (COM / FCU / 지상 브리지 + `sim/link_proto.py`)

* 유선·하이브리드 빌드: TELEM v4 = TYPE 0x23, 41 B(TELEM v2 + 2 B), TELEM v5 = TYPE 0x24, 47 B(TELEM v3 + 2 B). 덧붙인 2 B =
  `v_aux`(전선 또는 버퍼 전압), `pwr_flags`(전원 상태 비트). `LP_LEN_MAX` 40 → 42, 최대 프레임 47 B(`src/core/link_proto.h`).
* ilink: `IL_SNAP2` 55 → 57 B, `IL_SNAP3` 62 → 65 B(`pw_vaux`, `pw_flags`, `spare3`; `src/core/ilink_proto.h`). 탄도 이미지는 RP1을
  넣지 않습니다(`LP_RP1 0`, v0.8.1 길이).
* 이유 코드 R_W_LOST 0x30, R_W_SAG 0x31, R_W_BUF 0x32, R_W_D2 0x33, REJ_POWER 0x58이 `sim/link_proto.py`에 들어갔습니다.
* 지상 브리지(`firmware/ground_bridge`): `LINK_LEN_MAX` 42, 47 B 프레임(`bridge_config_gen.h` e6f18bab).
* 응답률(링크 모델, E): lora_uart TELEM5 2.014 Hz(요청 2 Hz, 여유 얇음), sx127x 4.439 Hz
  (`fw09/impl/fix/runs/final/rtt_numbers.txt`). TUNE 7400 짝 비교: 배터리 비행 130번 기록 동일, 유선·하이브리드 결과 바뀜 0
  (`fw09/impl/C/tune_pairs.md`).
* 변경 이력: `docs/제어_변경_이력.md`의 "v0.9 펌웨어 라운드" 절, `B:\Video\제어변경_이력\v0.9_제어_변경이력.md` 10절. 비교 영상
  29(유선 전압 저하 표시), 30(하이브리드 버퍼 전압 표시), 31(NAV 시간 넘침 해결): `B:\Video\영상\제어변경_비교\v0.9\04_펌웨어라운드_RP1\`.

### 0.6 ADC (검토 뒤 수정 F2, 유선·하이브리드 FCU)

* ADC 인터럽트(`__vector_21`)를 다시 써서 79 / 85 → **45 / 56 클럭**(fcu_wired / fcu_wired_buffer, 한계 58: 전송 중에 돌 수 있는
  인터럽트의 한계, 6.3). `run_simbench.py`가 이것을 전송 인터럽트 검사와 "시간 중요 구간 + 모든 인터럽트"에 넣습니다.
* 새 시험 `atmega128_fc/tools/adc_chain_test/`: 비행 코드의 ADC 체인을 simavr에서 돌립니다(통과, 옛 / 새 인터럽트 결과 같음).
  simavr는 변환이 16 µs(칩 104 µs)입니다.

### 0.7 측정값 요약 (시뮬레이터, M; `fw09/impl/fix/runs/final/`)

| 항목 | 한계 | 값 |
|---|---|---|
| NAV 필수 작업 (벤치) | 3200 µs | 최대 2782.2 µs (69.6 %) |
| FCU 시간 중요 구간 + 모든 인터럽트 (벤치) | 3200 µs | fcu 3089.4, fcu_demo 3082.4, fcu_nogps 2744.6, fcu_wired 2761.4, fcu_wired_buffer 2790.1, fcu_ballistic 2167.0 µs |
| IMU → 베인 (4칩 공동 시뮬레이션, 닫힌 고리) | 4710 µs | 최악 4076.1 µs (수다 고장), 정상 ≤ 3872.8 µs |
| NAV 틱 → 베인 (같음) | 4000 µs | 최악 3578.4 µs, 정상 ≤ 3375.0 µs |
| 비행 중 MS5611 계산 | 50 Hz | 50 Hz (정상 5개 실행 모두) |
| 위치 유지(HOLD) 중 명령 응답 | 미응답 0 | HOLD 13.1~20.0 s, 미응답 0 |
| 스택 여유 | ≥ 1024 B | NAV 2678, FCU 1215 / 1215 / 1370 / 1355 / 1361 (탄도 1205), GNS 1266, COM 2659 / 2649 B |
| GNS 20 ms 주기 여유 | > 10000 µs | 15001.4 µs (시연 14899.4) |
| glink 여유 (L = 3) | ≥ 2000 µs | 최소 2548.4 µs |
| 크기 (flash / 정적 RAM) | - | nav 35212 / 987, fcu 50894 / 2454, fcu_wired 47148 / 2294, fcu_wired_buffer 47876 / 2294, gns 37436 / 2265, com 11270 / 1117 B |

출처: `all_benches_nav_0.log`, `all_benches_fcu_0.log`, `all_benches_fcu_1.log`, `all_benches_fcu_2.log`, `all_benches_gns_0.log`,
`all_benches_com_0.log`, `rows_summary.txt`(모두 `fw09/impl/fix/runs/final/`), 크기는 `fw09/impl/accept/FINAL_HASHES.txt`.

### 0.8 검증 명령 (`firmware/atmega128_fc`에서, PY = `..\..\.venv\Scripts\python.exe`, `PYTHONIOENCODING=utf-8`, `PLATFORMIO_RUN_JOBS=2`)

1. 헤더: `PY ../../tools/gen_firmware_params.py --check`, `PY ../../tools/gen_gains_header.py --check`,
   `PY ../ground_bridge/gen_bridge_config.py --check`(모두 "up to date").
2. 벤치 + 4칩 공동 시뮬레이션 + 탄도 해시: `PY tools/run_all_benches.py --cosim-jobs 2 --pio-jobs 2` (순서 GNS → NAV → FCU → COM →
   공동 시뮬레이션 → FCU 재판정 → 탄도 해시; 약 25분). FCU 단일 칩 벤치는 벤치 자신의 일 때문에 일부 타이밍 검사가 "실패"로
   찍히고, 그 검사는 공동 시뮬레이션이 판정합니다(재판정 단계가 판정). 그래서 요약 JSON의 `"pass"`는 false로 남습니다(설계대로).
3. 공동 시뮬레이션만: `PY tools/run_cosim4.py --jobs 2` (설명: `atmega128_fc/tools/cosim/README.md`).
4. ADC 체인: `PY tools/adc_chain_test/run_adc_chain_test.py`.
5. 호스트 시험과 고정소수점 시험(프로젝트 루트에서): `PY tools/build_host_test.py --configs --qfix` (8 빌드 × 29699 검사).
6. 교차 검사(프로젝트 루트에서): `PY tools/crosscheck_controller.py` (약 7분, 구성별 기체 상수 검사 포함).

### 0.9 HEX

* 최종 이미지 78개(PlatformIO 환경 75개 + 지상 브리지 3개)의 해시: `B:\Video\펌웨어\v0.9\구현\FINAL_HASHES.txt`.
* 칩에 올리는 HEX: `B:\Video\펌웨어\v0.9\구현\HEX_비행이미지\` (안내 `B:\Video\펌웨어\v0.9\README.md`).
* 퓨즈, ISP 순서, 배선은 5절, 7절, 4절 그대로입니다. GNS를 구울 때는 glink 커넥터를 뽑습니다(PE0/PE1 = ISP 핀).
* 2026-11-06 라운드의 새 해시는 0.10절과 `docs/v0.9_펌웨어_결과.md` 11장에 있습니다(위 목록은 그 전 것).

## 0.10 2026-11-06 펌웨어 라운드: ESP8266 무선(ESP-NOW 브리지), 헤더 재생성, NEO-7M

표시: **M** 측정(시뮬레이터·호스트), **E** 추정, **A** 가정. 결과 전체와 합격 기준 재판정: `docs/v0.9_펌웨어_결과.md` 11장.
**이번 라운드는 제어를 바꾸지 않았습니다.** 시뮬레이션의 링크 모델도 LoRa(lora_uart) 그대로입니다(비관적인 쪽, E).
(2026-11-10 제어 라운드: 링크 모델이 `link.vehicle_radio`를 따라 ESP-NOW 모델 `esp_bridge`가 되었습니다. 아래 문단은 그 전 상태입니다.)
다만 지상국은 링크 지연 `t_link`를 실제로 잰 왕복 시간으로 씁니다. ESP에서는 약 0.06 s(E)라서 assist RELEASE 앞당김 규칙이
시뮬레이션하지 않은 값(시뮬레이션은 약 0.50 s)에서 돕니다. ESP-NOW 링크 모델(왕복 약 60 ms)로 assist·링크 손실 시나리오를 다시
돌리는 일은 제어 관련이라 다음 라운드입니다(검토 LB-6).

### 0.10.1 무엇이 바뀌었나

* 기체 무선이 **ESP8266(ESP-12E) + ESP-NOW**입니다 (사용자 결정 2026-11-05: Wi-Fi 채택, 로켓도 ESP). COM 백엔드는
  `params.json` `flight_computer.link.vehicle_radio`(`esp_bridge` 기본 / `lora_uart` / `sx127x`)가 정합니다.
  `flight_computer.link.radio`는 이제 **링크 모델**(시뮬레이션·지상국 응답 대기)이고 `lora_uart` 그대로입니다.
* 새 백엔드 `src/hal/esp_bridge.c`: USART1 **38400 8N1**(U2X 0, UBRR 25, 오차 +0.16 %), ESP 리셋 PC6(COM 찬 부팅 때만:
  부팅하자마자 L, 2 ms 뒤 해제), 해제 뒤 **400 ms 동안 받은 바이트 버림**(ESP8266 ROM의 74880 baud 부팅 글자), 바쁨 신호(AUX)
  없음 → 응답을 바로 UART로 보냄. 검토 반영(2026-11-06): PC6은 **오픈 드레인**(L로 몰거나 입력 Hi-Z; H로 몰지 않음 - 손으로
  RST-GND를 붙여도 핀 단락 없음, F1). COM만 리셋될 때(워치독·저전압·외부 리셋)는 ESP를 리셋하지 않고 창도 열지 않습니다
  (비행 중 COM 재시작의 링크 손실이 LoRa 때와 같음, F2). 프레임 규약(CRC, 논스, TELEM, 명령)은 그대로입니다. ESP는 바이트를 그대로 옮기는 브리지
  (`firmware/esp_bridge`)만 돌립니다.
* LoRa는 예비로 남습니다: `com_lora_uart`, `com_sx127x`(+ `_ballistic`)는 늘 빌드됩니다. 되돌리기: `vehicle_radio`를
  `lora_uart`(또는 `sx127x`)로 고치고 헤더 생성 → 빌드.
* 헤더 3개(`params_gen.h`, `gains.h`, `bridge_config_gen.h`)를 지금 `params.json`으로 다시 만들었습니다(그 전에 params 해시가
  낡은 CAD 보고서 5개·검사 보고서 5개를 다시 만듦: 값은 같고 해시·시간만 다름, M). `gains.h`는 내용이 같았습니다(M). 바뀐 것은 해시·설정 표시(`FC_PARAMS_TAG`)와 위 ESP 매크로, 로직 배터리 핀(`LOGIC_BAT_*`, 아직 읽지 않음),
  핀 설명 글입니다. 기체 상수(믹서 레버, 호버 스로틀 등)는 바뀌지 않았습니다(M, 헤더 비교).
* GPS: MON-VER의 `00070000`(u-blox 7, NEO-7M)을 UNKNOWN 대신 7로 읽습니다 (`core/gps_ubx.c`, `sim/gps_nav.py`; 보고만,
  동작에 쓰지 않음).

### 0.10.2 빌드 환경

| 환경 | 무선 | 비고 |
|---|---|---|
| `com`, `com_wired`, `com_wired_buffer`, `com_ballistic` | `vehicle_radio` (= esp_bridge) | 비행 이미지 |
| `com_esp_bridge`, `com_esp_bridge_ballistic` | esp_bridge 고정 | `com` / `com_ballistic`과 같은 이미지(M, 해시) |
| `com_lora_uart`, `com_sx127x` (+ `_ballistic`) | LoRa 고정 | 예비 |
| `com_simbench_esp`, `com_simbench_esp_ballistic` | esp_bridge | 단일 칩 벤치(`run_com_simbench.py`) |
| `com_cosim*` | lora_uart 고정 | 4칩 공동 시뮬레이션은 LoRa 타이밍 그대로 |

### 0.10.3 VTOL 캐리어 배선 (rev 2c, `pcb/vtol_carrier/spec/carrier_spec.json`, 읽기만 함)

| ESP-12E 핀 | 넷 | COM | 경로 |
|---|---|---|---|
| 21 RXD0 (GPIO3) | ESP_RXD0 | PD3 TXD1 | Q4 (BSS138) → 1 kΩ (3.3 V 쪽) → RXD0 |
| 22 TXD0 (GPIO1) | ESP_TXD0 | PD2 RXD1 | Q11 (BSS138) |
| 1 RST | ESP_RST | PC6 | Q5 (BSS138, 비반전). LV 쪽 10 kΩ 풀업 = RST 풀업. PC6을 L로 몰면 리셋, 입력(Hi-Z)이면 동작 (오픈 드레인) |
| 3 EN / 18 GPIO0 / 17 GPIO2 / 16 GPIO15 | | | 10 kΩ 풀업 + 100 nF / 10 kΩ 풀업 / 10 kΩ 풀업 / 10 kΩ 풀다운 |
| 8 VCC | +3V3 | | U6 AMS1117-3.3, 100 µF 탄탈 + 10 µF + 100 nF |

* PC3/PC4/PC5(M0/M1/AUX)는 이 기판에서 연결되지 않습니다. esp_bridge 빌드는 이 핀을 건드리지 않습니다.
* ESP 굽기: J26(COM 리셋 고정) + J3 1-2(IO0-GND) → 3.3 V USB-UART로 굽기 → 둘 다 빼고 J3 5-6으로 ESP 리셋(J26을 빼서 생긴
  COM 외부 리셋은 ESP를 리셋하지 않음) 또는 로직 전원을 껐다 켬 (`firmware/esp_bridge/README.md`).

### 0.10.4 로켓 COM 보드 배선 (손배선, 통신·로거 상자)

SPEC 10.3 규칙(MCU → 3.3 V 부품 신호는 BSS138 채널로)과 캐리어 rev 2c의 결정(ESP TX → COM RX 방향도 채널로, 직결은 V_IH 여유가
0.3 V뿐 - rev-1 검토 E2)을 따릅니다. 이 두 번째 것은 SPEC 10.3의 "모듈 → MCU 직결" 규칙의 예외이고 SPEC 10.7에 적었습니다
(LoRa 예비 결선은 10.3 그대로). 레벨시프터 2(상자 안 BSS138 4채널, HV 5 V / LV = AMS1117 #2의 3.3 V)를 이렇게 씁니다.

| 신호 | COM | 레벨시프터 2 | ESP-12E | 비고 |
|---|---|---|---|---|
| COM → ESP 데이터 | PD3 TXD1 | ch1 (HV ← PD3, LV → 1 kΩ →) | 21 RXD0 | 1 kΩ: 굽는 USB-UART가 이김 |
| ESP 리셋 | PC6 | ch2 (HV ← PC6, LV →) | 1 RST | 시프터의 LV 10 kΩ 풀업이 RST 풀업(A: 모듈의 LV 풀업 값 확인; 없으면 RST에 10 kΩ → 3.3 V를 따로 답니다). 비반전, PC6 오픈 드레인 |
| ESP → COM 데이터 | PD2 RXD1 | ch3 (LV ← TXD0, HV →) | 22 TXD0 | |
| OpenLog 로그선 | (FCU PE1) | ch4 | — | 4.10의 "어느 채널?" 문제: ESP 빌드에서는 ch4 |
| 전원 | — | — | 8 VCC ← AMS1117 #2 3.3 V | 100 µF + 10 µF + 100 nF를 핀 가까이. **5 V 금지** |
| 부팅 핀 | — | — | 3 EN 10 kΩ↑ + 100 nF, 18 GPIO0 10 kΩ↑, 17 GPIO2 10 kΩ↑, 16 GPIO15 10 kΩ↓ | 플래시에서 부팅 |
| 굽기 헤더 | — | — | IO0, GND, TXD0, RXD0, RST, GND (3.3 V 전용) | 굽는 동안 COM을 리셋 상태로(COM ISP 헤더의 RESET-GND 점퍼) |

* M0/M1/AUX(PC3/PC4/PC5)는 연결하지 않습니다. LoRa로 돌아가려면 4.2절 결선과 `vehicle_radio`를 되돌립니다.
* AMS1117 #2(1 A)는 ESP 송신 피크(약 400 mA, A)를 감당합니다(E). 5 V 로직 UBEC의 여유는 계산하지 않았습니다(4.1, 미검증).
* **사용자 결정이 필요한 것 (로켓)**: (1) 거리: 로켓은 정점 140-150 m + 낙하산 표류라 Wi-Fi(PCB 안테나 100-300 m, A)의 끝이나
  밖일 수 있습니다(E, 캐리어 spec `esp12e.range.rocket_need`). ESP-NOW 1 Mbps + 지상 고이득 안테나 계획(1 km에서 약 −84 dBm,
  E)으로 1 km 지상 시험을 통과해야 합니다. (2) 상자 맞춤: ESP-12E(24 × 16 × 3 mm, A)가 30 × 20 × 15 mm 상자에 OpenLog·
  시프터·AMS1117과 함께 들어가는지 CAD로 확인하지 않았습니다. (3) 안테나 방향: PCB 안테나 끝을 동체 벽 쪽, ESC·모터선·배터리에서
  멀리(4.3절). (4) 무게: params의 상자 무게(20 g, A)는 고치지 않았습니다(비행 질량에 들어가는 값). 이 결정 전까지 로켓도 LoRa를
  고를 수 있습니다(`vehicle_radio`).

### 0.10.5 검증 (M, 시뮬레이터·호스트; 실기 없음)

* 0.8절의 명령 그대로 + 새 시험: `PY tools/esp_bridge_test/run_esp_bridge_test.py` (COM의 ESP 백엔드를 simavr에서, 두 경우.
  찬 부팅: 리셋 유지·해제, PC6을 H로 몰지 않음, 부팅 창 바이트 16개 버림, 창 끝에 걸친 묶음은 앞만 버리고 뒤는 그대로, 창 뒤
  프레임 그대로, USART1 비트 26.00 µs = UBRR 25, 응답 즉시 송신. 따뜻한 부팅(워치독 리셋): PC6 건드리지 않음, 창 없음 - 23개 검사
  통과), 프로젝트 루트에서 `PY -m pytest tools/test_esp_bridge.py` (ESP 브리지 휴대형 부분 호스트 시험 2,264,897 검사 실패 0,
  설정 생성기).
* COM 벤치 `com_simbench_esp`: 통과. 메인 루프 최대 945 µs, 상향 → IL_CMD ≤ 238 µs, 스택 여유 2625 B(비행 `com_esp_bridge`),
  flash 11318 B / RAM 1151 B. 벤치의 가짜 지상국 바이트는 부팅 창을 거치지 않습니다(창은 위 simavr 시험이 핀에서 잼,
  `docs/v0.9_펌웨어_결과.md` 11.9절 F2). 응답 창 80 ms는 lora_uart 모델의 대기(590.6 ms) 기준으로 여유 27.8 ms(도착) / 106.4 ms(다음 요청 전).
* 합격 기준 22개: 20개 통과, 19번 이번에 판정 안 함, 21번(탄도 해시) 사용자 재선언 대기 - `docs/v0.9_펌웨어_결과.md` 11.3절.
* HEX 해시: `docs/v0.9_펌웨어_결과.md` 11.4절, 전체 값 `firmware/atmega128_fc/HEX_HASHES_2026-11-06.txt`.
* 독립 검토 반영(PC6 오픈 드레인, COM만 리셋될 때 ESP 그대로, 시험 추가, 문서): `docs/v0.9_펌웨어_결과.md` 11.9절.

`firmware/atmega128_fc/` 는 ATmega128A(16 MHz, 5 V) **세 개**에 올라가는 베어메탈 C99 펌웨어입니다 (avr-libc, Arduino 미사용,
PlatformIO `atmelavr` + `ATmega128` 보드). 소스 트리는 하나이고, 빌드 환경이 칩의 역할(`-D FC_ROLE_NAV / FC_ROLE_FCU /
FC_ROLE_COM`)을 정합니다 (`platformio.ini`). 시뮬레이션의 참조 구현(`sim/controller.py`, `sim/estimator.py`, `sim/onboard.py`,
`sim/link_proto.py`, `sim/ilink_proto.py`, `sim/nlink_proto.py`, `sim/edf.py`, `sim/quat.py`, `sim/state_machine.py`의
`AltitudeFilter`)을 식 단위로 옮긴 휴대형 코어(`src/core`)를 세 칩이 나눠 씁니다.

| 칩 | 하는 일 | 소스 |
|---|---|---|
| **NAV** (항법) | MPU-6050·기압계(I2C; 기본 드라이버 BMP280 - 2026-11-10, MS5611은 빌드 옵션 `nav_ms5611`), 패드 정렬 / 자이로 적분 자세 추정, 기울기, 고도 필터, 센서 상태. 매 틱 끝에 항법 프레임을 SPI(nlink)로 FCU에 보냄. **비행 판단 없음** | `src/nav/` + `src/core/` |
| **FCU** (비행 제어) | 감시기(지상 명령 + 인터록 I1–I10), 자세제어, PWM 6채널, OpenLog 로그. 틱은 NAV 프레임이 도착하면 시작. **모든 비행 판단은 여기서** | `src/main.c` + `src/core/` |
| **COM** (통신) | 무선 드라이버(기본 ESP-12E 브리지, 예비 LoRa), 상향 CMD 해독 → FCU로 전달(ilink), FCU 스냅샷으로 TELEM 작성. **비행 판단 없음** | `src/com/` + `src/core/` |

**경위.** v0.4(2026-10-13): 기체 안의 컴퓨터는 **자세제어만** 합니다. 발사·스로틀·모터 차단·노즈(낙하산) 사출은 모두
**지상국 명령**이며 **LoRa 모듈**로 전달됩니다(SPEC §8.8, §10.3). 기체에 남은 자율 로직은 명령 실행기와 어떤 명령으로도
끌 수 없는 안전 인터록(I1–I10)뿐입니다. v0.6.0(2026-10-15, 사용자 규칙: 비행 계산은 모두 ATmega128에서, 칩 개수는 제한
없음): 무선을 COM 칩으로 나눴습니다(ilink, SPEC §10.4). v0.6.3: 센서와 추정기를 NAV 칩으로 나눴습니다(nlink, SPEC §10.5).
이유: v0.6의 FCU 하나가 센서·추정·감시·제어·출력을 한 틱에 차례로 하면 시간임계 부분이 인터럽트 포함 3,309 µs(4 ms 틱의
82.7 %, 한계 80 %)였습니다(`simbench_report.json` `v06_reference`). 나눈 뒤 FCU는 1,944 µs(48.6 %, 인터럽트 포함 53.9 %)입니다.

> **수치의 출처.** 2026-10-16 에 실행한 보고서에서 옮겼습니다: `firmware/atmega128_fc/host_test/build/`의
> `simbench_report.json`(FCU, 10-16 01:27), `nav_simbench_report.json`(NAV, 10-16 01:28), `com_simbench_report.json`(COM,
> 10-16 01:28), `cosim_report.json`(세 칩 co-simulation, 10-16 01:51), `crosscheck_report.json`(10-16 01:42; `--only ilink`·`nlink`·`log`
> 절 보고서는 01:52). 이 보고서들은 `params.json` 해시를 기록하지 않습니다. 지금 `params.json`(sha256 `0a5c261b…`)으로 만든
> `params_gen.h`(`FC_PARAMS_TAG` 0x6E)로 빌드한 뒤에 실행한 순서로 묶입니다(co-simulation의 COM 리셋 실행에 FCU 로그
> "params tag 0x6E match"가 남습니다). 호스트 단위시험은 2026-10-16 이 문서를 고치면서 다시 실행했습니다.
> **실기(하드웨어)와 실제 무선 구간에서 측정한 값은 하나도 없습니다** — 칩, LoRa
> 모듈, 센서 없이 호스트 시험과 simavr 시뮬레이터로만 확인했습니다. 실기 확인이 필요한 항목은 "미검증", 근거가 되는 측정이나
> 보고서가 없는 값은 "가정값"으로 표시했습니다.

> **예산 설계(기본값)**: 6S 1개, Hobbywing SkyWalker 100A V2, ESC BEC로 서보 구동, 로직은 별도 5 V UBEC,
> 분압 47k/4.7k. 원래 12S 설계는 `params.json`의 `KAIST_90mm_12S` 프리셋. **SkyWalker 기본 스로틀 범위는
> 1100–1940 µs**이므로 처음 한 번 ESC 스로틀 보정(최대 → 최소)으로 1000–2000 µs에 맞추십시오.

## 1. 요약

| 항목 | NAV | FCU | COM |
|---|---|---|---|
| 비행 빌드 (PlatformIO 환경) | `nav` | `fcu` | `com` (= `com_lora_uart`), `com_sx127x` |
| 플래시 (131,072 B 중) | 17,332 B (13.2 %) | 27,956 B (21.3 %) | 10,280 B (7.8 %) / 10,756 B (8.2 %) |
| 정적 SRAM (4,096 B 중) | 771 B | 2,417 B | 928 B / 929 B |
| 스택 → 비행 빌드 여유 (≥ 1,024 B 조건) | 깊이 222 B + 시뮬레이터가 안 돈 인터럽트 22 B → **3,081 B** | 깊이 365 B → **1,314 B** (단독 측정 370 B → 1,309 B) | 깊이 162 / 161 B → **3,006 B** (단독 측정 297 / 302 B → 2,871 / 2,865 B) |
| 시간임계 부분 (4 ms 틱, 한계 80 % = 3,200 µs) | 틱 → 프레임 전달 최대 **2,240 µs (56.0 %)**, MPU-6050 버스트까지 더한 상한 2,435 µs (60.9 %) | 프레임 → 틱 끝, 모든 인터럽트 포함: 정상 비행 최대 **2,118 µs (53.0 %, lora_uart) / 2,165 µs (54.1 %, sx127x)**; 고장 실행까지 최대 2,578 µs (64.4 %); 관측이 아닌 최악 합성 2,802 µs (70.0 %) | 고정 마감 없음. 메인 루프 1회 최대 **848 / 842 µs** (조건: ilink 수신 링이 차는 2,560 µs 미만) |
| 근거 | `cosim_report.json`, `nav_simbench_report.json` | `cosim_report.json`, `simbench_report.json` | `cosim_report.json`, `com_simbench_report.json` |

(COM 칸의 "A / B" 는 lora_uart / sx127x 백엔드입니다.)

| 항목 | 값 | 근거 |
|---|---|---|
| 샘플 → PWM | NAV의 IMU 샘플(MPU-6050 버스트 시작) → FCU의 마지막 베인 비교 레지스터: p50 4,184 µs (비행 틱만 4,242 µs), p99 4,528 µs, **최대 4,658 µs (lora_uart) / 4,655 µs (sx127x)**. 6자유도 시뮬레이션은 4.71 ms로 비행 (`sim/fc_timing.py` `FC_COMPUTE_DELAY_S_MEASURED`). co-simulation은 이 4.71 ms를 판정 한계로 씀(11.4) | co-simulation, 무선마다 149,915 틱 |
| 응답 지연 (기체 안) | 상향 프레임이 COM 무선 드라이버에 도착 → TELEM이 무선에 넘어감: p50 10.7 / 9.4 ms, **최대 12.5 / 13.2 ms**. 링크 모델은 14 ms (`sim/link_model.py` `FC_REPLY_DELAY_S`) | co-simulation, 무선마다 1,080 건 |
| 명령 지연 | 모든 IL_CMD가 마지막 바이트 뒤 첫 FCU 틱의 감시기에 도착 (0 틱, 1,240 + 1,200 건) | co-simulation |
| 호스트 단위시험 | **20,173 검사, 실패 0** (`sim/out/link_vectors.json`, `ilink_vectors.json`, `nlink_vectors.json` 벡터 바이트 단위 일치 포함) | `tools/build_host_test.py --run` (2026-10-16) |
| 교차검증 (C vs Python) | 제어기 13,624 벡터 max \|Δδ\| = 2.2e-6 rad (한계 1e-4); 감시기 506,232 틱 **불일치 0**; LoRa 프로토콜 3,000 스트림 382,765 B 불일치 0; ilink 3,000 스트림 750,317 B 불일치 0; nlink 인코딩 10,000 + 해독 20,000 불일치 0; 추정기·고도필터·로그 왕복 PASS | `tools/crosscheck_controller.py` |
| 세 칩 co-simulation | 102 실행(정상 80 = 무선 2 × 시작 지연 40, 진단 2, 고장 20), 판정 43개 모두 PASS | `tools/run_cosim.py` |

## 2. 부품 선정 — ATmega128 호환성

| 칩 | 기능 | 부품 | ATmega128 쪽 연결 | 전압/레벨 | 메모 |
|---|---|---|---|---|---|
| 공통 | MCU | ATmega128A 최소 시스템 보드 ×3, 16 MHz 크리스털 | — | 5 V | 크리스털이 16 MHz인지 확인(예산안: 8 MHz로 온 사례). 세 칩 모두 같은 퓨즈(5절). ISP는 PE0/PE1/PB1 |
| NAV | IMU | GY-521 (MPU-6050) | TWI PD0/PD1, 400 kHz, 0x68 | 모듈 VCC 5 V, I2C 3.3 V | ±2000 °/s, ±16 g, DLPF 98 Hz, 1 kHz |
| NAV | 기압계 | BMP280 모듈 (TYE-BMP29, 디바이스마트 15442588; 사용자 결정 2026-11-04, GY-63 MS5611 대신) | TWI, 0x77 (CSB·SDO를 3.3 V에) | 모듈 VCC 3.3 V, I2C 3.3 V | 기본 드라이버 `hal/bmp280.c`(2026-11-10): 보정값 타당성 검사, 강제 모드 ×4 / ×1, IIR 끔 (데이터시트 값 A, 실제 부품 미측정). MS5611 드라이버(PROM CRC4, 2차 온도보상)는 빌드 옵션 `nav_ms5611` |
| NAV | 레벨시프터 1 | BSS138 4채널 | HV = 5 V, LV = 3.3 V | | ch1 SCL, ch2 SDA, ch4 MPU INT(선택, 펌웨어 미사용), ch3 예비 |
| NAV | 3.3 V #1 | AMS1117-3.3 모듈 | 5 V → 3.3 V | | 레벨시프터 1의 LV 기준 |
| FCU | 베인 서보 ×4 | MG90S | OC1A PB5, OC1B PB6, OC1C PB7, OC3A PE3 | 전원 5 V (ESC BEC), 신호 5 V | 50 Hz, 1500 µs 중심, 11.1 µs/° |
| FCU | 사출 서보 | MG90S | OC3C PE5 | 5 V | 래치 1000.5 µs ↔ 해제 1999.5 µs (`fc_config.json`) |
| FCU | ESC | Hobbywing SkyWalker 100A V2 (6S, BEC 5 V/7 A) | OC3B PE4 | 신호 5 V | 1000 µs = 정지. POWERED 외 모든 상태에서 1000 µs |
| FCU | 로거 | OpenLog 호환 + microSD | USART0 TXD0 = PE1 → 레벨시프터 한 채널 → RXI (4.10) | VCC 5 V, RXI 3.3 V | 57600 baud. PE1은 ISP 핀 겸용 → 프로그래밍할 때 분리 |
| FCU | 배터리 전압 | 47 kΩ / 4.7 kΩ + 100 nF | ADC0 PF0, 내부 2.56 V 기준 | 6S 만충 25.2 V → 2.29 V | AREF에는 100 nF만 |
| FCU | 표시/스위치 | LED + 1 kΩ, 5 V 능동 부저 + NPN, 아밍 플러그 | PC0, PC1, PC2 | 5 V | 아밍 스위치 GND 단락 = 아밍, 내부 풀업 |
| COM | **레벨시프터 2 (무선용)** | BSS138 4채널 | HV = 5 V, LV = 3.3 V (AMS1117 #2) | | MCU → 모듈 신호만 통과 (4.2) |
| COM | 3.3 V #2 | AMS1117-3.3 모듈 | 5 V → 3.3 V | | LoRa 모듈(송신 약 120 mA)과 레벨시프터 2 |
| COM | **LoRa (형식 A)** | Ebyte E22 / E220 / E32 계열 UART 투명 모드 모듈 | USART1 PD2/PD3, M0 PC3, M1 PC4, AUX PC5 | 신호 3.3 V, 모듈 VCC는 설명서 확인 | `link.uart_baud` 9600 8N1. 채널·공중속도·주소는 미리 설정 |
| COM | **LoRa (형식 B)** | SX1276 계열 900 MHz SPI 모듈 (RFM95W, Ra-01H). Ra-01/Ra-02는 433 MHz라 이 설정에 맞지 않음 | SPI PB0–PB3, DIO0 PE7, RESET PC6 | **3.3 V 전용** (5 V 금지) | 레지스터는 `params.json` `link.sx127x`에서 생성 |
| 공통 | 칩 사이 링크 | ilink: 1 kΩ 직렬 저항 1개 + 선 3가닥, nlink: 선 5가닥 | 4.8, 4.9 | 5 V 끼리 직결 | 레벨시프터 없음 (세 칩 모두 5 V) |
| 공통 | 전원 | ESC BEC → 서보 5개, 별도 5 V 3 A UBEC → 세 보드 + 통신·로거 상자 | — | 로직과 서보 전원 분리 | 서보 전류로 로직이 떨어지면 브라운아웃(4.0 V) 리셋 |
| 공통 | 프로그래머 | USBasp | 칩마다 6핀 ISP | 5 V | MOSI→PE0(PDI), MISO←PE1(PDO), SCK→PB1. 칩마다 뺄 것은 7.3 |

**모듈 형식 선택**: COM 펌웨어는 세 백엔드를 지원하며 `params.json` `flight_computer.link.vehicle_radio`(`esp_bridge` 기본 /
`lora_uart` / `sx127x`)로 컴파일 때 고릅니다(2026-11-06; `flight_computer.link.radio`는 이제 시뮬레이션의 링크 모델이고 COM
빌드를 바꾸지 않습니다, 0.10절). LoRa 모듈이면 형식 A 또는 B입니다. 어느 쪽인지 모르면: 핀이 `M0 M1 RXD TXD AUX VCC GND` 이면 형식 A(Ebyte UART),
`NSS SCK MOSI MISO DIO0 RESET 3.3V` 이면 형식 B(SX127x SPI)입니다. 두 백엔드는 항상 함께 컴파일·시험됩니다
(`com_lora_uart`, `com_sx127x` 환경). NAV와 FCU는 무선 형식과 무관합니다.

## 3. 주파수와 법규 (미검증 — 비행 전 반드시 확인)

* 한국에서는 **KC 인증을 받은 920 MHz 대역(920.9–923.3 MHz) 모듈**을 쓰십시오(SPEC §10.3). 900 MHz 계열 Ebyte
  모듈(E22-900T…, E220-900T…)이나 RFM95(SX1276)가 여기에 해당할 수 있습니다 — **모듈 자체의 KC 인증 여부를 확인**하십시오.
* **433 MHz 모듈(E32-433T…, Ra-02 등)을 한국에서 써도 되는지는 확인하지 못했습니다.** 쓰기 전에 전파누리
  (rra.go.kr)에서 확인하십시오. 생성기(`tools/gen_firmware_params.py`)는 `link.sx127x.freq_hz`가
  `link.frequency_note`에 적힌 대역(현재 920.9–923.3 MHz) 밖이면 헤더를 만들지 않습니다.
* 송신 출력: `link.sx127x.tx_power_dbm` = 14 dBm (PA_BOOST). Ebyte 모듈은 설정 도구로 출력을 정합니다(E22-900T22D는
  최대 22 dBm). 한국 920 MHz 대역의 허용 출력·안테나 이득 조건은 **확인하지 못했으니** 법규 범위 안의 최소 출력으로
  설정하십시오.

## 4. 배선표

핀 배정은 `params.json`(FCU `flight_computer.pins`, NAV `flight_computer.nav.pins`, COM `flight_computer.com.pins`)에서 생성기가
읽어 칩마다 따로 ATmega128 대체기능 표와 대조합니다. 같은 칩에서 핀을 두 번 쓰거나, ilink가 그 칩의 USART가 아니거나,
nlink가 SPI 핀(양쪽 PB0 = SS)이 아니면 헤더를 만들지 않습니다. **다른 칩의 같은 이름 핀은 다른 핀입니다** (예: FCU PC0 = LED,
COM PC0 = LED, NAV PC1 = LED). 핀 표 전체는 `src/board.h` 머리말에 있습니다.

### 4.1 전압 영역

| 영역 | 공급원 | 부하 |
|---|---|---|
| 6S (22.2 V, 최대 25.2 V) | 6S 2200 mAh ×1 | ESC 전원, 로직 UBEC 입력, 전압 분압기 상단 (FCU PF0) |
| 5 V 서보 | ESC BEC 5 V/7 A | MG90S ×5 V+ (베인 4 + 사출 1) |
| 5 V 로직 | 5 V 3 A UBEC (하나) | **NAV·FCU·COM 세 보드** VCC/AVCC(LC), **통신·로거 상자**(OpenLog VCC, 레벨시프터 2 HV, AMS1117 #2 입력), NAV의 GY-521 VCC·레벨시프터 1 HV·AMS1117 #1 입력, FCU의 부저·LED (BMP280은 3.3 V 모듈이라 AMS1117 #1에서) |
| 3.3 V 센서 (NAV) | **AMS1117 #1** (NAV 보드) | 레벨시프터 1 LV 기준 (I2C 센서 쪽), BMP280 모듈 VCC·CSB·SDO |
| 3.3 V 무선 (통신·로거 상자) | **AMS1117 #2** | 레벨시프터 2 LV 기준, LoRa 모듈 VCC (형식 B는 3.3 V 전용; 형식 A는 모듈 설명서의 허용 전압 확인) |
| GND | 모든 모듈 공통 | ilink·nlink 하네스마다 GND 선(`board.h`). v0.5까지는 비행 컴퓨터 한 점 성형 접지였고, 세 보드 구성의 접지점은 정해지지 않았습니다 |

5 V 로직의 합계 전류는 계산하지 않았습니다(미검증). 세 칩이 한 UBEC에서 같이 켜지는 것은 펌웨어가 전제로 합니다
(`src/com/com_link.h` R3: 같은 퓨즈, 같은 리셋 지연).

### 4.2 LoRa 모듈 결선 (COM ↔ 통신·로거 상자, 레벨시프터 2)

원칙(SPEC §10.3): **MCU → 모듈 신호는 모두 BSS138 채널을 거쳐 3.3 V로**, **모듈 → MCU 신호는 직결**
(3.3 V ≥ ATmega128의 V_IH 3.0 V @ 5 V). 모듈 쪽에 5 V를 직접 넣지 마십시오. 아래 핀은 모두 **COM** 칩의 핀입니다.

**형식 A — Ebyte E22/E220/E32 (UART 투명 모드), `radio = lora_uart`**

| 모듈 핀 | COM | 경로 | 비고 |
|---|---|---|---|
| RXD | PD3 / TXD1 | BSS138-2 **ch1** (HV ← PD3, LV → RXD) | 9600 8N1 |
| TXD | PD2 / RXD1 | 직결 | |
| M0 | PC3 | BSS138-2 **ch2** | 펌웨어가 부팅 직후 LOW(일반 = 투명 모드) |
| M1 | PC4 | BSS138-2 **ch3** | 〃 |
| AUX | PC5 | 직결 | 입력 + 내부 풀업: 연결하지 않으면 "준비됨"으로 읽음. 응답은 AUX = H일 때(또는 50 ms 대기 후) 송신 |
| VCC / GND | 3.3 V (AMS1117 #2) / GND | — | 모듈 설명서의 전압 범위 확인 |
| (ch4) | — | 예비 | 4.10 참고 |

**형식 B — SX1276/77/78 SPI (RFM95/96, Ra-01H), `radio = sx127x`**

| 모듈 핀 | COM | 경로 | 비고 |
|---|---|---|---|
| NSS | PB0 | BSS138-2 **ch1** | 시프터 풀업 덕분에 COM 리셋(ISP) 중에도 H 유지 → 칩 비선택 |
| SCK | PB1 / SCK | BSS138-2 **ch2** | COM ISP SCK와 같은 핀. SPI 500 kHz (`fc_config` `sx127x_spi_hz_max`) |
| MOSI | PB2 / MOSI | BSS138-2 **ch3** | (ATmega128 ISP는 PB2/PB3를 쓰지 않음) |
| RESET | PC6 | BSS138-2 **ch4** | L = 리셋, H = 해제(모듈 풀업이 유지) |
| MISO | PB3 / MISO | 직결 | |
| DIO0 | PE7 | 직결 | RxDone / TxDone, 메인 루프에서 폴링 |
| 3.3V / GND | AMS1117 #2 / GND | — | **5 V 금지** |

두 형식의 핀은 겹치지 않으므로 결선은 사용하는 모듈 것만 하면 됩니다. COM 보드와 상자는 페이로드 격벽 아래에 5.0 mm
떨어져 나란히 매달립니다(4.4).

### 4.3 안테나 배치 (PLA 동체 내부)

* LoRa 모듈은 페이로드 격벽 아래의 통신·로거 상자(θ −90°, 축에서 40 mm)에 있습니다. 안테나는 **동체 밖으로 내지 않고**
  에비오닉스 벽을 따라 **안쪽에 수직으로** 늘어뜨립니다(`electronics.radio_log` 설명; 지상국 안테나도 수직).
* PLA는 전파를 거의 막지 않습니다. **ESC(트레이 +x 면), 모터 3상선(0°), 배터리 대전류선, EDF 덕트에서 최대한 멀리**(가능하면
  10 cm 이상) — 스위칭 잡음과 금속 차폐. 안테나 선이 이 배선과 나란히 달리지 않게 하십시오.
* 1/4 파장 와이어 안테나: 922 MHz ≈ 81 mm, 433 MHz ≈ 173 mm. 스프링/PCB 안테나는 모듈 그대로. 동축 피그테일은 짧게.
* 금속(배터리 팩, 알루미늄 부품) 뒤에 숨기지 말 것. 배치 후 반드시 6.8절의 지상 거리 시험을 하십시오.

### 4.4 보드 위치 (`python cad/layout.py`, CAD 검사 C22 / C24 / C25)

| 보드 | 위치 (배기구 끝 Z = 0, mm) | 고정 |
|---|---|---|
| NAV (40 g 가정, 50 × 50 × 15) | 트레이 +x 면 위쪽 (v0.5 비행 컴퓨터 자리), x 26–41, z 457.9–507.9 | 44 mm 정사각 M3 구멍 |
| FCU (20 g 가정, 50 × 50 × 15) | **흡입 모듈 격벽 윗면에 눕혀** (베이 바닥, 트레이 아래), z 335.9–350.9, 축 중심 | 3 mm 양면 폼 테이프 + 격벽 통풍구 두 개(45/225° 또는 135/315°)를 지나는 케이블 타이. 서보선·모터선 구멍은 막지 않음 (C25) |
| COM (20 g 가정, 50 × 50 × 15) | **페이로드 격벽 아래면**, z 514.9–529.9, 축 중심 | 보스 4개(Ø7 × 3, ±22 mm), M3×5 자가 탭핑, 막힌 구멍 깊이 3.8 mm (C22). 부품이 아래로 매달리므로 매달린 면에는 일자 핀헤더 대신 직접 납땜이나 ㄱ자 헤더 |
| 통신·로거 상자 (20 g 가정, 30 × 20 × 15) | 페이로드 격벽 아래면, x −15…15, y −50…−30, z 517.9–532.9 | 양면 폼 테이프 + 케이블 타이 (C24). 안에 LoRa 모듈, OpenLog + microSD, 레벨시프터 2, AMS1117 #2 |

보드 사이 틈(같은 출력): FCU–NAV 107 mm, FCU–COM 164 mm, COM–NAV 7.07 mm, COM–상자 5.0 mm. 질량과 크기는 모두
**가정값**입니다 — 달고 재서 `params.json`에 넣으십시오. 하네스 길이는 정해지지 않았습니다(예산안의 ilink 선 30 cm는 가정값).
서보 연장선 외의 배선 경로(사출 서보선, ESC 신호선, 아밍 스위치·부저·LED, 전원선, 보드 사이 하네스)는 CAD에 없습니다(미정).

### 4.5 FCU 결선

| 커넥터 | 핀 | FCU | 상대편 | 비고 |
|---|---|---|---|---|
| 로직 전원 | 5V, GND | VCC, AVCC(LC), GND | 5 V UBEC | VCC/GND 쌍마다 100 nF |
| ISP (6핀) | MISO, VCC, SCK, MOSI, RST, GND | PE1(PDO), VCC, PB1, PE0(PDI), RESET, GND | USBasp | **PB2/PB3가 아니라 PE0/PE1**. 7.3 |
| nlink | SS, SCK, MOSI, MISO, GND | PB0, PB1, PB2, PB3 | NAV PB0–PB3 (4.9) | FCU는 SPI 슬레이브. PB0 입력 + 풀업 |
| ilink | TX, RX, GND | PD3 (TXD1), PD2 (RXD1) | COM PE0, PE1 (4.8) | TX 쪽에 1 kΩ 직렬 |
| 로그 | TX, GND | PE1 (TXD0) | → 통신·로거 상자 OpenLog RXI (4.10) | 프로그래밍 중 분리 |
| 베인 서보 1–4 | SIG, V+, GND | PB5, PB6, PB7, PE3 | 서보 연장선 (4.11) | V+ = 서보 5 V 줄(ESC BEC). FCU VCC와 잇지 않음 (아래) |
| ESC | SIG, GND | PE4 (OC3B) | ESC 신호선 | 10 kΩ 풀다운. ESC 빨간선(BEC +5 V)은 FCU 로직 5 V(VCC)에 잇지 않음 (아래) |
| 사출 서보 | SIG, V+, GND | PE5 (OC3C) | MG90S (페이로드 모듈) | V+ = 서보 5 V 줄. 선 경로 미정 |
| 배터리 전압 | VBAT | PF0 (ADC0) | 팩+ → 47 kΩ → PF0 → 4.7 kΩ → GND | PF0에 100 nF |
| 아밍 스위치 | SW | PC2 | 플러그 → GND | 열림 = 안전. **비행 중 열리면 KILL** |
| LED / 부저 | LED, BUZ | PC0, PC1 | | |

**서보 전원 (4.1).** ESC BEC(+5 V)는 서보 5 V 줄에만 갑니다: FCU 서보 헤더의 V+ 줄(베인 4개)과 사출 서보 V+. 이 줄은 FCU의
VCC(로직 UBEC 5 V)와 연결하지 않습니다. 위 표의 "잇지 않음"은 FCU 로직 5 V에 대한 말입니다. GND는 모두 공통입니다. 산 ESC에서
BEC 출력이 어느 선으로 나오는지는 확인하지 않았습니다(ESC 설명서로 확인, 미검증).

v0.6.3의 FCU에는 I2C도 센서도 없습니다. PD0/PD1/PE6은 비어 있습니다.

### 4.6 NAV 결선

| 커넥터 | 핀 | NAV | 상대편 | 비고 |
|---|---|---|---|---|
| 로직 전원 | 5V, GND | VCC, AVCC(LC), GND | 5 V UBEC | AMS1117 #1 입력도 여기서 |
| ISP (6핀) | | PE1, VCC, PB1, PE0, RESET, GND | USBasp | 뺄 것 없음 (7.3) |
| I2C 센서 | SCL, SDA | PD0, PD1 | BSS138-1 ch1/ch2 → GY-521, BMP280 (GY-63 대신, 2026-11-04) | 400 kHz |
| MPU INT (선택) | INT | PE6 (INT6) | BSS138-1 ch4 | 펌웨어 미사용 (틱에서 폴링) |
| nlink | SS, SCK, MOSI, MISO, GND | PB0, PB1, PB2, PB3 | FCU PB0–PB3 (4.9) | NAV는 SPI 마스터. PB3 입력 + 풀업 |
| LED | LED | PC1 | + 1 kΩ | 8.6 |

NAV에는 로그 포트가 없습니다. NAV가 아는 것은 모두 항법 프레임에 실려 FCU가 기록합니다(12절 `N`, `T`).

### 4.7 COM 결선 (무선 제외)

| 커넥터 | 핀 | COM | 상대편 | 비고 |
|---|---|---|---|---|
| 로직 전원 | 5V, GND | VCC, AVCC(LC), GND | 5 V UBEC | |
| ISP (6핀) | | PE1, VCC, PB1, PE0, RESET, GND | USBasp | **ilink 커넥터를 뺀 뒤** (7.3) |
| ilink | RX, TX, GND | PE0 (RXD0), PE1 (TXD0) | FCU PD3, PD2 (4.8) | PE0/PE1 = COM의 ISP 핀 |
| 무선 | | 4.2 | 통신·로거 상자 | |
| LED | LED | PC0 | + 1 kΩ | 8.6 |

### 4.8 ilink 하네스 (FCU ↔ COM, SPEC §10.4)

| FCU | 경로 | COM | 비고 |
|---|---|---|---|
| PD3 TXD1 | **1 kΩ 직렬 저항** → | PE0 RXD0 (COM ISP PDI) | 프로그래밍 중 실수로 꽂혀 있어도 FCU 대 프로그래머 전류가 5 mA로 제한됨 |
| PD2 RXD1 | ← 직결 | PE1 TXD0 (COM ISP PDO) | |
| GND | — | GND | |

250,000 baud 8N1, U2X = 0 (UBRR 3, 오차 0 %): 바이트당 40 µs, 틱당 100 B. 두 칩 모두 5 V이므로 레벨시프터가 없습니다. 두 쪽 모두
RX 핀 풀업을 켜므로 상대가 꺼져 있거나 리셋 중이거나 빠져 있으면 선은 잡음 없이 H로 쉽니다. ilink는 위 표의 **3선(TX, RX, GND)**
입니다(`board.h`). COM 보드의 5 V는 ilink로 보내지 않고 4.1대로 로직 UBEC에서 받습니다. 예산안은 이 하네스를 4핀
커넥터(TX, RX, GND, 5 V)로 잡았습니다(가정값). 그 커넥터를 쓰면 4번째 핀은 비워 둡니다. 그래서 ilink 커넥터를 빼면 데이터만
끊기고 COM은 켜진 채로 있습니다.

### 4.9 nlink 하네스 (NAV → FCU, SPEC §10.5)

| NAV | 방향 | FCU | 비고 |
|---|---|---|---|
| PB0 (SS 출력) | → | PB0 SS | FCU 풀업: 평소 H. 꺼지거나 리셋 중인 NAV는 SS를 H로 둠 → FCU 슬레이브는 SCK를 무시 |
| PB1 SCK | → | PB1 SCK | **FCU의 ISP SCK이기도 함** — FCU를 프로그래밍할 때 NAV를 리셋으로 잡거나 이 커넥터를 뺌 |
| PB2 MOSI | → | PB2 MOSI | 항법 프레임 |
| PB3 MISO (입력 + 풀업) | ← | PB3 MISO | FCU 상태 바이트. FCU가 없으면 0xFF |
| GND | — | GND | |

SPI 모드 0, MSB 먼저, 2 MHz(F_CPU/8), 바이트 사이 4 µs 간격. 틱마다 72 B 교환 한 번, SS LOW 약 630 µs(co-simulation
634–642 µs). 레벨시프터 없음. 2 MHz 신호를 보드 사이 선으로 보낸 실기 파형은 확인하지 않았습니다(미검증, 선은 짧게).

### 4.10 로그선 (FCU → 통신·로거 상자의 OpenLog)

| FCU | 경로 | OpenLog | 비고 |
|---|---|---|---|
| PE1 TXD0 | 레벨시프터 한 채널 (HV ← PE1, LV → RXI) | RXI | 57600 8N1. PE1 = FCU ISP PDO → 프로그래밍 중 분리 |
| — | — | VCC 5 V, GND | 로직 UBEC |

**어느 시프터 채널을 쓸지는 소스에 정해져 있지 않습니다(미검증).** 형식 A(lora_uart) 결선이면 레벨시프터 2의 ch4가 비어
있습니다. 형식 B(sx127x)이면 레벨시프터 2의 네 채널을 무선이 모두 쓰므로 채널이 하나 모자랍니다. 레벨시프터 1의 빈 ch3는
NAV 보드(트레이) 위에 있어 FCU(베이 바닥) → 상자(격벽 아래) 경로에서 벗어납니다. 결선 전에 정하십시오(15절).
2026-11-06: ESP 빌드(기본)에서는 레벨시프터 2의 ch4입니다(0.10.4). 형식 B(sx127x)로 돌아갈 때만 미정입니다.

로그선은 FCU를 프로그래밍할 때 뽑아야 합니다(`board.h`: "unplug the OpenLog while programming the FCU", 7.3). 그러므로 로그선에는
**적어도 FCU 쪽 끝에 빼고 꽂는 커넥터가 있어야 합니다.** OpenLog의 5 V는 이 선이 아니라 로직 UBEC에서 받습니다(4.1).

### 4.11 서보 연장선 (베인 서보 4개 → FCU, SPEC §3.8)

서보선은 케이스에서 60 mm만 남기고 26 AWG 3심 연장선을 이어, 포드 창 → 핀 옆 → 원뿔 세로 슬롯 → 데크 홈 → 스피곳 턱 →
흡입 모듈 180° 통로 → 격벽 180° 서보선 구멍을 지나 **베이 바닥의 FCU 보드 윗면에서 끝납니다**(v0.6.3; v0.6에서는 트레이의
비행 컴퓨터로 올라갔음). 연장선 길이(여유 50 mm 포함, `cad/layout.py`):

| 포드 (베인) | 각도 | 경로 전체 | 연장선 | FCU 신호 핀 |
|---|---|---|---|---|
| 1 | 24° | 534 mm | **524 mm** | PB5 (OC1A) |
| 2 | 114° | 444 mm | **434 mm** | PB6 (OC1B) |
| 3 | 204° | 460 mm | **450 mm** | PB7 (OC1C) |
| 4 | 294° | 549 mm | **539 mm** | PE3 (OC3A) |

빨강 → 서보 5 V 줄(ESC BEC, FCU VCC와 분리, 4.5), 갈색 → GND. 격벽 위 마지막 구간(구멍 → FCU 윗면 → 보드 가운데)은 보드 위 서보 헤더 위치를 몰라 추정한
길이입니다. 단면 3.6 × 1.3 mm, 10 g/m는 가정값(`servo_leads.*_ASSUMED`)이니 산 전선을 재고 다십시오. 세 가닥을 가볍게 꼬고
ESC와 모터 전선에서 떨어뜨려 묶습니다.

## 5. 퓨즈

세 칩(NAV, FCU, COM) 모두 같은 값입니다(`platformio.ini` `[env]`). 같은 퓨즈라 리셋 지연(16K CK + 65 ms)도 같고, COM의 부팅
STAT 대기(100 ms)는 이것을 전제로 합니다.

| 퓨즈 | 값 | 비트 |
|---|---|---|
| lfuse | **0x3F** | BODLEVEL=0(4.0 V), BODEN=0(BOD 켬), SUT=11, CKSEL=1111 |
| hfuse | **0xCF** | OCDEN=1, JTAGEN=1(JTAG 끔), SPIEN=0, CKOPT=0, EESAVE=1, BOOTSZ=11, BOOTRST=1 |
| efuse | **0xFF** | M103C=1(ATmega103 호환모드 **해제**), WDTON=1 |

```
.venv/Scripts/python.exe -m platformio run -d firmware/atmega128_fc -e nav -t fuses   # USBasp를 NAV ISP에
.venv/Scripts/python.exe -m platformio run -d firmware/atmega128_fc -e fcu -t fuses   # FCU ISP에 (7.3의 준비 먼저)
.venv/Scripts/python.exe -m platformio run -d firmware/atmega128_fc -e com -t fuses   # COM ISP에 (7.3의 준비 먼저)
# 같은 일을 avrdude로:  avrdude -c usbasp -p m128 -B 8 -U lfuse:w:0x3F:m -U hfuse:w:0xCF:m -U efuse:w:0xFF:m
```
새 칩은 M103C가 켜져 있습니다. 해제하지 않으면 USART1·Timer3가 없는 ATmega103으로 동작해 ilink(FCU USART1), 무선(COM
USART1), 서보(FCU Timer3)가 동작하지 않습니다. 세 칩 모두 해제하십시오.

## 6. 링크 — LoRa(지상), ilink(FCU ↔ COM), nlink(NAV → FCU)

### 6.1 LoRa 동작 (SPEC §10.3)

* **지상국이 마스터**입니다. 지상국(`tools/ground_station.py`, 기준 로직 `sim/ground_station_logic.py`)이 주기적으로
  CMD 프레임을 보내고, 기체는 **유효한 상향 프레임에 대한 응답으로만** TELEM 프레임을 보냅니다. 반이중 무선에서
  상향·하향이 부딪히지 않습니다. 기체는 스스로 송신하지 않습니다.
* 프레임: `0xA5 0x5A | LEN | VEH | TYPE | SEQ | PAYLOAD | CRC16`(CCITT-FALSE, 하위 바이트 먼저). CMD 13 B, TELEM 36 B.
  VEH = `link.vehicle_id` (90). CRC·길이·기체 ID가 틀린 프레임은 COM이 조용히 버리고 카운터만 올립니다(IL_STAT → FCU `S`·`C` 기록).
* 명령은 SEQ가 직전 처리분보다 새로울 때만 한 번 실행되고(반복 전송은 응답만), LAUNCH와 ARMED에서의 THROTTLE은
  **최근 텔레메트리의 nonce**를 담아야 합니다. nonce는 FCU가 부팅 때 배터리 ADC의 LSB 잡음으로 씨앗을 정하고(16비트
  LFSR) LAUNCH마다 바뀝니다 — 보안 기능이 아니라, 예전 세션에서 녹음된 LAUNCH가 다시 실행되지 않게 하는 장치입니다.
* 처리 경로(v0.6.3):
  1. **COM**: 바이트 수신(형식 A: USART1 수신 인터럽트 → 링, 형식 B: DIO0 RxDone 폴링 → FIFO) → 메인 루프 1회에 최대 8 B를
     스트리밍 디코더에 → 받아들인 CMD 하나마다 IL_CMD(seq, cmd, arg, nonce, RSSI, SNR)를 ilink 송신 링에. 상향 완료 → IL_CMD
     큐까지 최대 0.30 ms (lora_uart) / 0.52 ms (sx127x). COM은 명령을 해석하지 않습니다.
  2. **FCU**: 틱이 시작될 때 수신 링에 이미 있던 바이트 중 IL_CMD를 끝낼 수 있는 것만 시간임계 부분에서 해독(최대 16 B) → **같은
     틱의 감시기**가 명령과 인터록을 처리 → 응답 요청이면 그 틱의 값으로 스냅샷 → 백그라운드에서 IL_SNAP(55 B, 선로 2.2 ms)을 보냄.
  3. **COM**: SNAP이 응답 게이트(6.2)를 통과하면 1단계(q에서 roll/pitch/yaw, float32), 2단계(양자화 + TELEM 인코딩 + CRC) →
     무선이 비면 송신(형식 A: AUX = H 확인 후 USART1 송신 인터럽트, 형식 B: 대기 모드 → FIFO 적재 → TX → TxDone 또는 시간초과
     52 COM 틱(208 ms) → 연속 수신 복귀).
* **지상국 보조(assist)** 는 v0.6.3부터 응답 시간초과도 판단 시점으로 씁니다: 마지막 텔레메트리를 도착 후 흐른 시간 a만큼
  외삽해 CUT(P + dP/dt·a)과 예측형 RELEASE(v_z − g·a)를 판단합니다. `sim/ground_station_logic.py`의 연습 모드(무선마다 100 시드,
  5 % 손실)에서 I4가 먼저 모터를 끈 비행이 15 → 10(lora_uart), 19 → 6(sx127x)으로 줄었습니다. 남은 것은 대부분 CUT 교환
  자체가 손실된 경우입니다.

### 6.2 ilink (FCU ↔ COM, SPEC §10.4, `src/core/ilink_proto.h`, `sim/ilink_proto.py`)

프레임: `0xC5 0x5C | LEN | TYPE | PAYLOAD | CRC16`(CCITT-FALSE, LEN·TYPE·PAYLOAD 위, 하위 바이트 먼저). 길이는 TYPE마다 고정.

| TYPE | 방향 | 페이로드 / 프레임 | 내용 |
|---|---|---|---|
| IL_CMD 0x01 | COM → FCU | 8 B / 14 B | COM 디코더가 받아들인 상향 CMD 하나: seq, cmd, arg, nonce, rssi_dbm(없으면 127), snr_q4 |
| IL_SNAP 0x02 | FCU → COM | 49 B / 55 B | 감시기가 응답한 틱의 스냅샷: reply_seq, state, flags, reason, ack_seq, nonce, throttle, tick, h, v_z, tilt, V_cell, loop_max, q[4] |
| IL_STAT 0x03 | COM → FCU | 36 B / 42 B | COM 틱, 리셋 원인, 플래그(radio ok 0x01, boot 0x02, sx127x 0x04), RegVersion, RSSI/SNR, params tag, 해독·무선·응답 카운터, ilink 오류, 루프 최대 |

* **디코더**(양쪽 같은 코드): 바이트마다 일정한 일만 하고 되감지 않습니다(FCU의 `il_decoder_push`는 simavr에서 바이트당 ≤ 91 사이클,
  방향 필터 +4). 거부된 후보 안에 숨은 프레임은 잃습니다 — 잃은 IL_CMD는 지상국의 재전송이 메웁니다. **방향 마스크**(v0.6.3):
  FCU는 CMD·STAT만, COM은 SNAP만 받아들이고, 반대 방향의 LEN/TYPE은 헤더에서 거부합니다.
* **STAT 간격(D4)**: IL_CMD 바로 뒤에, 직전 STAT 후 1 s(1 / `ilink.stat_hz`)가 지났을 때만. 상향이 1.5 s 없으면 자체 분주로. IL_CMD
  앞에는 끼지 않습니다(42 B STAT 뒤의 IL_CMD는 한 틱 늦게 감시기에 감). 예외는 부팅 STAT 하나(COM 리셋마다 많아야 한 번). 간격
  1–2.5 s(co-simulation 1.32–2.38 s).
* **부팅 STAT(R3)**: COM 리셋 후 100 ms 이상 지나서 보냅니다(같은 UBEC으로 같이 켜진 FCU가 받을 준비가 되도록). com_simbench
  측정 100.3 ms.
* **응답 게이트(D5)**: SNAP은 reply_seq가 **80 ms(20 COM 틱) 안에** COM이 보낸 IL_CMD의 seq와 맞을 때만 TELEM이 되고, 그 창이
  열려 있는 동안만 무선에 넘어갑니다(열린 슬롯 4개). 엉뚱한 SNAP, 중복, 늦은 SNAP은 거부됩니다(com_simbench가 일부러 보낸 3개 모두
  거부). 80 ms는 지상국 응답 대기 한도가 이미 기체에 주는 68 ms보다 크고, 정상 응답 경로(약 10–13 ms)의 몇 배입니다.
* **FCU 쪽 예산**: 시간임계 부분에서 최대 16 B, 백그라운드에서 다른 일보다 먼저 틱당 최대 16 B, 나머지는 다음 프레임을 기다리는
  동안. 수신 링(63 B)이 넘치면(COM 폭주) 다음 틱의 시간임계·백그라운드는 해독하지 않습니다 — IL_CMD가 못 들어오면 링크 손실입니다.
* **고장 의미**: COM이 죽거나 리셋 중이거나 선이 끊기면 IL_CMD가 안 옵니다 = 무선 링크 손실(I1p / I1). FCU가 죽으면 SNAP이
  없으므로 COM은 아무것도 보내지 않고, 지상국은 응답 시간초과를 봅니다.

### 6.3 nlink (NAV → FCU, SPEC §10.5, `src/core/nlink_proto.h`, `src/hal/nlink_spi.h`, `sim/nlink_proto.py`)

> v0.9: 전송은 인터럽트로 한 바이트씩(바이트 간격 140 클럭), 밀어내기 하한 890 µs, 전송 감시 타이머 W1 1000 µs - 0.3절과 0.4절.
> 아래 프레임 형식은 v0.6.3 것입니다. VTOL 빌드는 v0.8.1부터 NL_TYPE_NAV4 (0x04, 72 B 교환 그대로) / NL_TYPE_FCU4 (0x05, 13 B)를
> 씁니다(`src/core/nlink_proto.h`). FCU 규칙, 시간초과 4.6 ms, 58 클럭 인터럽트 한계는 그대로입니다.

NAV는 매 틱 항법 상태를 다 계산하자마자 SS를 내리고 **정확히 72 B**를 주고받은 뒤 SS를 올립니다. FCU의 틱은 그 마지막 바이트가
들어오면 시작합니다.

| 방향 | 바이트 | 내용 |
|---|---|---|
| NAV → FCU (MOSI) | 72 B: `0xD5 0x5D \| 0x01 \| SEQ \| 66 B \| CRC16` | SEQ = NAV 틱 하위 바이트. q[4], rate[3](바이어스 보정), accel[3], h, v_z, h_baro_rel, tilt (f32); imu_fail, baro_fails, twi_errors, loop_us_max (u16); flags, reset_cause (u8) |
| FCU → NAV (MISO, 같은 교환) | 앞 10 B: `0xD5 0x5D \| 0x02 \| SEQ \| state \| fflags \| params_tag \| 0 \| CRC16`, 나머지 0x00 | SEQ = FCU가 마지막으로 받아들인 NAV SEQ, state = FCU 최신 틱 뒤 감시기 상태, fflags: BENCH 0x01(`fcu_bench_vanes`), RECOVERY 0x02(I10 부팅) |

| NAV flags | 뜻 |
|---|---|
| IMU_OK 0x01 | 이번 틱 MPU-6050 버스트 읽기 성공 |
| BARO_OK 0x02 | MS5611 샘플이 예정대로 (v0.5 규칙) |
| ALIGNED 0x04 | 패드 정렬 완료 (패드를 떠나면 고정) |
| GYRO_SAT 0x08 | 자이로가 int16 한계 |
| EST_VALID 0x10 | 이 NAV 부팅 뒤 끝난 패드 정렬에서 나온 자세 |
| BARO_NEW 0x20 | 이번 틱 고도 필터가 새 기압 샘플을 씀 |
| BOOT 0x40 | 리셋 뒤 아직 유효한 FCU 프레임을 못 받음 |

* **FCU의 규칙**(감시기 자체는 그대로): `imu_ok` = 이번 틱 유효 프레임 && IMU_OK && (EST_VALID || 비행 상태 아님), `baro_ok` =
  유효 프레임 && BARO_OK, `aligned` = ALIGNED, `gyro_saturated` = GYRO_SAT. h, v_z, tilt, 각속도, q는 프레임에서(없으면 마지막
  유효값 유지). 유효 프레임이 없으면(시간초과, CRC, 길이, 동기) 그 틱은 imu_ok = baro_ok = false — 죽은 NAV나 끊긴 선은
  IMU + 기압 고장이고, 25 틱(0.1 s) 뒤 I9입니다. **비행 중 재부팅한 NAV**는 EST_VALID가 없으므로(패드 정렬을 다시 해야 생김)
  imu_ok가 false → I9 → ABORT입니다.
* **NAV의 규칙**: 마지막 유효 FCU 프레임의 상태로 패드/비행을 고릅니다(첫 유효 FCU 프레임 전에는 SAFE로 봄). SAFE·ARMED이면
  고도 필터를 0에 붙잡고 지면 기준 기압을 저역 통과, 추정기는 패드 정렬(BENCH면 자이로만). FCU 상태 바이트는 FCU가 이번 틱
  프레임을 기다리는 동안 나가므로 NAV가 쓰는 상태는 한 FCU 틱 묵은 것이고, **NAV의 패드 → 비행 전환은 감시기보다 2 틱 늦습니다**
  (v0.5: 1 틱). LAUNCH 뒤 스로틀은 0에서 0.5 s 동안 오르므로 추정기가 패드 정렬을 떠날 때 기체는 아직 움직이지 않았습니다.
* **프레임 시간초과 4.6 ms** (`flight_computer.nlink.frame_timeout_ms`): 이전 틱 시작부터 4.6 ms 안에 프레임이 없으면 FCU는 자기
  Timer2로 틱을 돌리고(항법 데이터 무효), 프레임이 다시 올 때까지 4 ms마다 돕니다. 4.6인 이유(params 설명): 패드 → 비행 전환 때
  NAV 프레임 시각이 최대 0.45 ms 움직이고, 4.6 ms보다 늦게 시작한 대체 틱은 FCU 백그라운드 시간을 없애 비행 기록을 떨어뜨립니다
  (A/B: 4.6 ms 0개, 4.7 ms 1개, 5.0 ms 2–4개) → 4.6 ms는 양쪽에 0.15 ms 여유. co-simulation의 틱 간 프레임 시각 최대 증가는
  436 / 450 µs로 여유 600 µs 안입니다.
* **SPI 슬레이브 재동기**: SS가 H인데 교환이 덜 끝났으면(바이트 손실, 여분 SCK, SS 글리치, 교환 중 NAV 리셋) 그 부분 교환을 버리고
  응답 0번 바이트를 다시 싣습니다. 오류 하나는 그 교환과 많아야 하나를 더 잃습니다.
* **늦은 응답 바이트**: FCU 슬레이브 인터럽트가 바이트 간격보다 늦으면 이전 MOSI 바이트가 시프트 레지스터에 남습니다. 1–9번
  바이트면 NAV가 그 FCU 프레임의 CRC를 거부하고 마지막 유효 상태를 유지합니다. 10–71번은 검사하지 않으므로 무해합니다. 그래서
  교환 중에 돌 수 있는 FCU 인터럽트는 모두 짧게 만들었습니다(11.6). 한계는 두 가지입니다: **58 사이클**은 측정한 한계(경쟁
  인터럽트 58 사이클까지 모든 바이트가 제때, simbench 검사 `exchange_isrs_le_58_cycles`), **약 50 사이클**은 `src/hal/nlink_spi.h`의
  설계 목표입니다(ilink RX 51 / 최장 57, 로그 UDRE 46: 경계에 있음).

### 6.4 지연 예산 (`sim/link_model.py`; `*_ASSUMED` 값 포함 — 실측 아님)

| 무선 | 상향 | 기체 안 (상향 도착 → TELEM 무선) | 하향 | 왕복 | 응답 대기 한도 | 최대 요청률 | 프레임 1개 손실 시 상향 공백 |
|---|---|---|---|---|---|---|---|
| `lora_uart` (공중속도 2.4 kbps 가정) | 149.2 ms | 14 ms (co-sim 최대 12.5) | 273.8 ms | **437.0 ms** | 531.0 ms | 2.29 Hz | 1.031 s (< I1p 2.0 s) |
| `sx127x` (SF7, 125 kHz, 4/5) | 64.9 ms | 14 ms (co-sim 최대 13.2) | 119.6 ms | **198.4 ms** | 292.4 ms | 5.04 Hz | 1.000 s |

* `poll_hz` 2 Hz는 두 무선 모두 가능합니다. 패드에서 연속 2개 손실까지 견디고 3개째에 SAFE(I1p), 비행 중에는 연속 4개까지 견딥니다(I1).
* 기체 안 지연: `FC_REPLY_DELAY_S` = 14 ms(co-simulation 최대를 올림), 상향 → IL_CMD `COM_FORWARD_S` = 0.6 ms. v0.5 한 칩은
  16 틱(64 ms)이었으므로 lora_uart 왕복이 491 → 437 ms로 줄었습니다.
* 응답 대기 한도는 14 ms가 아니라 `FC_REPLY_ALLOWANCE_S` = 68 ms + 여유 40 ms로 만듭니다(v0.6.2와 같은 531 / 292 ms). 기체가
  아직 정당하게 늦게 응답할 수 있는 시간(COM 응답 창 80 ms가 여기서 나옴) 동안 지상국이 다음 요청을 보내 부딪히지 않게 하기
  위해서입니다. COM 응답 창 검사(`run_com_simbench.py`): 지상국 시간초과 전 도착 한계 108 ms 대비 여유 27.8 / 23.1 ms, 지상국의
  다음 요청 전 송신 종료 한계 대비 여유 38.9 / 83.1 ms(lora_uart는 AUX가 붙은 경우 56 ms 포함).
* 연속 상향(버스트)이 COM에 쌓이면 기체 안 지연이 길어집니다(co-simulation 최대 38.7 ms, lora_uart). 모델은 이것을 넣지 않고,
  응답 대기 한도의 40 ms 여유가 덮습니다.

### 6.5 Ebyte 모듈 설정 (형식 A) — 지상국 모듈과 기체 모듈을 **똑같이**

모듈을 USB-TTL(3.3 V) 어댑터에 연결하고 M0 = M1 = 1(설정 모드)로 제조사 설정 도구(RF Setting)에서:

| 항목 | 값 | 이유 |
|---|---|---|
| UART | 9600, 8N1 | `link.uart_baud` |
| 공중 속도 (Air data rate) | **2.4 kbps** | `link.lora_uart_air_rate_bps` 가정값. 바꾸면 params와 `sim/link_model.py` 예산도 갱신 |
| 채널 | 920.9–923.3 MHz 안의 채널 | 예: E22/E220-900 계열은 *850.125 MHz + CH × 1 MHz* → 922.125 MHz = CH 72 (**모듈 설명서로 확인 — 미검증**) |
| 주소 (ADDH/ADDL) | 양쪽 같은 값(예: 0x0000) | 프레임 안의 VEH로 기체를 구분 |
| 전송 방식 | **투명(transparent)** 전송 (고정점 아님) | COM은 바이트만 옮김 |
| 서브패킷 크기 | 64 B 이상 | TELEM 36 B가 한 패킷에 |
| RSSI 바이트 추가 | 끔 | 수신 데이터 = 프레임 그대로 |
| WOR / LBT / 암호키 | 끔 / 끔 / 양쪽 같게(0) | |
| 송신 출력 | 법규 범위 최소값 | 3절 |

설정 후 M0 = M1 = 0으로 돌아가면 일반(투명) 모드입니다. 기체에서는 COM 펌웨어가 PC3/PC4를 부팅 즉시 LOW로 둡니다.
900 MHz 모듈은 KR920 대역 채널을 쓰고, 433 MHz 모듈(E32-433)은 3절 확인 전에는 쓰지 마십시오.

### 6.6 SX127x 설정 (형식 B)

설정할 것이 없습니다 — `params.json` `link.sx127x`에서 생성된 레지스터(`params_gen.h`)를 COM이 부팅 때 씁니다:
FRF 0xE68666 (922.099976 MHz), BW 125 kHz / CR 4/5 / 명시적 헤더(0x72), SF7 / 페이로드 CRC(0x74), AGC(0x04),
PA_BOOST 14 dBm(0x8C), 프리앰블 8, 동기워드 0x12, LNA 0x23. RegVersion이 0x12가 아니면 COM LED가 빠르게 깜빡이고,
IL_STAT의 radio ok 플래그가 꺼져 FCU 로그에 `T` "COM boot: radio sx127x FAIL, version 0x.." 가 남습니다. 상향이 없으므로
아밍되지 않습니다. RFM95는 868/915(= 920 대역 포함) 칩(SX1276), **Ra-02는 433 MHz(SX1278)** 이므로 주파수와 모듈을
맞추십시오(생성기는 대역 밖 주파수를 거부).

### 6.7 링크·칩 고장 때 동작 (어떤 명령으로도 끌 수 없음)

| 상황 | 동작 |
|---|---|
| ARMED, 유효 상향 2.0 s 없음 (I1p) | SAFE (모터 원래 꺼짐) |
| POWERED/UNPOWERED, 유효 상향 3.0 s 없음 (I1) | ABORT: 모터 즉시 정지, 0.5 s 뒤 사출 → RELEASED |
| COM 고장·리셋, ilink 선 끊김 | IL_CMD가 없음 = 위의 링크 손실. co-sim: COM 0.5 s 리셋 → 상향 공백 0.78 s, 상태 변화 없음. ilink 4 s 절단 → 마지막 IL_CMD 3.0 s 뒤 ABORT, 0.5 s 뒤 RELEASED |
| COM 폭주 (ilink에 쓰레기) | FCU 시간임계 부분은 한계 안(co-sim 최대 2,280 µs, 57.0 %, sx127x `babble_cmdhdr`). 폭주 중에는 IL_CMD가 못 들어올 수 있음 → 링크 손실 규칙 |
| NAV 고장, nlink 선 끊김 | 프레임 없음 → FCU는 4.6 ms 뒤 자기 Timer2로 틱, imu_ok = baro_ok = false → 0.1 s(25 틱) 뒤 I9: ARMED → SAFE, 비행 중 → ABORT |
| NAV가 비행 중 재부팅 | EST_VALID 없음 → I9 → ABORT (co-sim: 마지막 유효 프레임 25 틱 뒤 ABORT, 125 틱 뒤 RELEASED) |
| 링크와 무관하게 항상 | I4 원지점 제한(h + 1.20 v²/2g ≥ 150 m → 모터 차단), I5 지오펜스(h > 160 m → ABORT), I6 낙하 감지(v_z ≤ −8 m/s 0.3 s → 사출), I7 모터 시간 25 s → 차단, I2 기울기 45°, I3 회전율, I8 셀전압, I9 센서 고장 |
| 아밍 스위치 열림 | KILL (ARMED → SAFE, 비행 중 → ABORT) |
| FCU 비행 중 워치독/브라운아웃 리셋 (I10) | SAFE로 부팅하되 사출 서보 열림, 전원 재투입 전 아밍 금지 |

### 6.8 지상 거리 시험 (첫 비행 전 필수)

1. 기체를 비행 형태로 조립(안테나 위치 그대로), **모터선 분리 또는 로터 제거**, 사출 링크 분리.
2. 지상국을 켜고 텔레메트리 수신·`link_ok` 플래그·(형식 B) RSSI를 확인. ARM → DISARM이 되는지.
3. 기체를 세운 채 지상국을 들고 **예상 최대 거리의 1.5배 이상**(고도 150 m + 바람 표류를 생각해 최소 300 m) 걸어가며
   텔레메트리 수신율, 왕복 시간, 오류 카운터(`S` 기록의 `link_bad`, `radio_errors`)를 기록. 기체를 눕혀서도 한 번.
4. 그 거리에서 ARM → 지상국 전원 끄기 → **2.0 s 후 SAFE**가 되는지(LED, 로그 `E` 기록 `I1p`).
5. 모터 없이 ARM → LAUNCH → (POWERED, 베인 동작) → 지상국 끄기 → **3.0 s 후 ABORT, 0.5 s 후 사출 서보 해제** 확인.
6. 수신율이 나쁘면 안테나 위치·출력·공중속도(낮출수록 거리↑, 지연↑)를 바꾸고 params/시뮬레이션을 갱신한 뒤 반복.

## 7. 빌드와 프로그래밍

### 7.1 PlatformIO 환경

| 환경 | 칩 | 용도 |
|---|---|---|
| `nav` | NAV | 비행 |
| `fcu` | FCU | 비행 |
| `fcu_bench_tether` | FCU | 로드셀 계류 추력시험의 0–50 % 구간: ESC 스로틀 50 % 제한 (`FC_THROTTLE_LIMIT=0.5f`). 100 % 추력은 이 빌드로 잴 수 없음 → `fcu` (10절 9단계) |
| `fcu_bench_hover` | FCU | 계류 호버: ESC 스로틀을 호버 스로틀(`FC_HOVER_THROTTLE` 0.915, CAD 질량 2.586 kg) + 5 %로 제한 |
| `fcu_bench_vanes` | FCU | 베인 방향·트림 시험(9.3): SAFE + 아밍 플러그 제거 + NAV 정렬 완료 시 자세제어가 베인을 구동, ESC는 항상 1000 µs. **비행 금지** |
| `com` | COM | 비행, 무선 = `params.json` `flight_computer.link.vehicle_radio` (현재 `esp_bridge`; `link.radio`는 시뮬레이션 링크 모델) |
| `com_lora_uart`, `com_sx127x` | COM | 같은 비행 펌웨어, 백엔드 강제 (두 백엔드가 항상 컴파일되게; 가진 모듈에 맞는 쪽을 올려도 됨) |
| `nav_simbench`, `fcu_simbench`, `fcu_simbench_micro`, `com_simbench`, `com_simbench_sx127x` | 각 칩 | simavr 단독 측정 빌드(가짜 센서·가짜 상대 칩·가짜 지상국). **기체에 올리지 말 것** |
| `nav_cosim`, `fcu_cosim`, `com_cosim`, `com_cosim_sx127x` | 각 칩 | 세 칩 co-simulation 빌드(진짜 SPI·USART 경로). **기체에 올리지 말 것** |

`pio run`의 기본 환경은 `nav, fcu, fcu_bench_tether, fcu_bench_hover, fcu_bench_vanes, com, com_lora_uart, com_sx127x`입니다.
측정·co-simulation 빌드는 13절의 도구가 직접 빌드합니다. 벤치 빌드의 크기(`avr-size`, 2026-10-16 01:25 ELF): `fcu_bench_tether`
27,954 B / 2,417 B, `fcu_bench_hover` 27,960 B / 2,417 B, `fcu_bench_vanes` 28,492 B / 2,417 B(플래시 / 정적 SRAM). 벤치 빌드의 스택은
측정하지 않았습니다.

### 7.2 순서

```
# 1) 생성 헤더 (params.json / CAD / gains.json / fc_config.json 이 바뀔 때마다)
.venv/Scripts/python.exe tools/gen_firmware_params.py
.venv/Scripts/python.exe tools/gen_gains_header.py
# 2) 빌드 (기본 환경 전부)
.venv/Scripts/python.exe -m platformio run -d firmware/atmega128_fc
# 3) 퓨즈: 새 칩마다 한 번 (5절)
# 4) 업로드: NAV → FCU → COM. USBasp를 그 칩의 ISP에 꽂고, 7.3의 준비를 먼저
.venv/Scripts/python.exe -m platformio run -d firmware/atmega128_fc -e nav -t upload
.venv/Scripts/python.exe -m platformio run -d firmware/atmega128_fc -e fcu -t upload
.venv/Scripts/python.exe -m platformio run -d firmware/atmega128_fc -e com -t upload
```

* 순서는 `platformio.ini` 머리말을 따랐습니다. 각 단계에서 신경 쓸 것은 프로그래밍하는 칩의 ISP 핀뿐입니다(7.3).
* **세 칩은 같은 `params_gen.h`로 빌드해야 합니다.** 빌드마다 params tag(`FC_PARAMS_TAG` = `params.json` sha256 앞부분의 하위
  바이트, 지금 0x6E)가 들어가고, 다르면 이렇게 드러납니다: COM의 tag는 IL_STAT으로 와서 FCU 로그 `T` "params tag 0x.. MISMATCH,
  FCU 0x.."와 `S` 기록 `com_flags` 0x80, FCU의 tag는 nlink 응답으로 가서 NAV reset_cause 비트 0x20(`N` 기록)과 NAV LED 두 번 깜빡임.
  비행에는 영향이 없지만(보고만), **생성기를 다시 돌렸으면 세 칩을 모두 다시 올리십시오.**
* **생성 헤더는 지금 `params.json`(sha256 `0a5c261b…`)과 맞습니다(2026-10-16 확인).** `gen_firmware_params.py --check`,
  `gen_gains_header.py --check`, `firmware/ground_bridge/gen_bridge_config.py --check` 모두 "up to date"이고 경고가 없습니다.
  CadQuery 보고서와 `sim/out/gains.json`이 같은 해시로 다시 만들어졌으므로 `gains.h`의 `GAINS_PARAMS_STALE`은 0입니다.
  Fusion 360 재실행은 아직 남아 있지만(출력 파일용), 헤더의 원본(`params_gen.h` 머리말의 목록)에는 Fusion 보고서가 없습니다.
* 생성 헤더(`src/params_gen.h`, `src/gains.h`)는 손으로 고치지 않습니다. 기체별 보정값과 무선 드라이버 한계값은
  `firmware/atmega128_fc/fc_config.json`에 있습니다(`link` 절: AUX 대기 50 ms, SPI 500 kHz, PA_BOOST, TX 시간초과,
  nonce 씨앗 변환 횟수).

### 7.3 ISP: 칩마다 뺄 것 (`src/board.h`, `platformio.ini`)

| 칩 | 프로그래밍 전에 | 이유 |
|---|---|---|
| NAV | 뺄 것 없음 | NAV의 PE0/PE1은 비행 빌드에서 쓰지 않음. NAV PB1(= ISP SCK)은 FCU PB1로 가지만, NAV가 리셋된 동안 PB0–PB2는 떠 있고 FCU의 SS 풀업이 FCU SPI를 쉬게 둠. 그동안 FCU는 프레임 없이 자기 시계로 돎(SAFE에서 무해, 로그에 시간초과로 남음) |
| FCU | ① OpenLog 로그선(FCU PE1) 분리 ② **NAV를 리셋으로 잡거나**(NAV RESET을 GND로) **nlink 커넥터 분리** | FCU PE0/PE1 = ISP PDI/PDO, 그중 PE1 = 로그 TX. FCU PB1 = ISP SCK인데 동작 중인 NAV가 SCK를 구동함 |
| COM | **ilink 커넥터 분리** | COM PE0/PE1 = ISP PDI/PDO = ilink RX/TX. 꽂힌 채이면 1 kΩ가 FCU 대 프로그래머 전류를 5 mA로 제한. (sx127x) COM PB1 = 무선 SCK, 레벨시프터 풀업이 NSS를 H로 지켜 모듈은 반응하지 않음 |

USBasp가 5 V를 공급할지, 기체 UBEC로 켤지는 정해지지 않았습니다. 둘을 동시에 연결하지 마십시오(일반 주의, 미검증).

**조립된 기체의 FCU.** FCU는 흡입 모듈 격벽 위에 있고 윗면이 트레이 아래 끝과 2.0 mm 떨어져 있어 ISP 헤더에 손이 닿지 않습니다
(`python cad/layout.py`). 벤치 빌드를 오갈 때(9.3, 10절)와 params가 바뀔 때마다 FCU를 다시 올려야 하므로, 조립된 기체에서 FCU에
닿는 순서를 `docs/제작가이드.md` 3장 10단계에 적었습니다(에이비오닉스 모듈을 흡입 모듈에서 들어 올림). FCU ISP 연장선을
밖으로 빼는 방법은 설계되지 않았습니다(경로가 CAD에 없음, 미검증).

## 8. 펌웨어 구조와 동작

```
src/core/   휴대형 C99 (AVR 헤더 없음, 호스트에서도 컴파일) - 세 칩이 나눠 씀
  onboard.c       sim/onboard.py OnboardSupervisor (명령 실행 + 인터록 I1-I10, 틱 단위로 동일)        FCU (COM: TELEM 양자화)
  link_proto.c    sim/link_proto.py (CRC16, 프레임, 재동기 스트리밍 디코더, 정수 양자화)              COM
  ilink_proto.c   sim/ilink_proto.py (v0.6, FCU <-> COM)                                              FCU, COM
  nlink_proto.c   sim/nlink_proto.py (v0.6.3, NAV -> FCU)                                             NAV, FCU
  quat.c, pid.c, mixer.c, attitude_ctrl.c   sim/quat.py, sim/controller.py                            FCU (NAV: quat)
  att_estimator.c sim/estimator.py PadAlignGyroEstimator                                              NAV
  alt_filter.c    sim/state_machine.py AltitudeFilter                                                 NAV
  thrust_model.c  sim/edf.py                                                                          FCU (기록만)
  ms5611_math.c, log_record.c, fc_config.c
src/hal/    FCU: pwm, timer(인터럽트 없음), adc(+nonce 씨앗), gpio, uart_log, ilink_uart(USART1), nlink_spi(슬레이브), watchdog(I10), stack
            NAV: twi, mpu6050, ms5611, nlink_spi(마스터), timer(Timer2 틱 + Timer0 IMU 리드), watchdog, stack (uart_log = 측정 빌드 콘솔)
            COM: ilink_uart(USART0), radio.h + lora_uart.c / sx127x.c (컴파일 때 하나만 실제 코드), watchdog, stack
src/main.c  FCU 250 Hz 루프 (프레임 시작)       src/nav/  NAV 루프        src/com/  COM 루프
src/board.h 세 칩의 핀 표
```

### 8.1 FCU 루프 — 프레임 시작 + 대체 시계 (`src/main.c`)

**틱 시작**: NAV의 72 B 교환이 끝나면. 기다리는 동안 폴링하며(슬립 없음, FCU에는 타이머 인터럽트가 없음) 들어오는 ilink 바이트를
해독합니다. 이전 틱 시작부터 4.6 ms 안에 프레임이 없으면 자기 Timer2로 틱을 시작하고(항법 데이터 무효), 프레임이 돌아올 때까지
4 ms마다 그렇게 합니다.

시간임계 부분 (`sim/sim6dof.py` FlightSim.run 순서):
1. 교환 꺼내기 + `nl_decode_nav`(동기, TYPE, CRC); 아밍 스위치
2. ilink 수신: 틱 시작 때 링에 있던 바이트 중 IL_CMD를 끝낼 수 있는 것만(최대 16 B) — 선로에서 틱 전에 끝난 IL_CMD는 이 틱의 감시기로
3. 온보드 감시기: 직전 호출 뒤 해독된 IL_CMD, 프레임에서 센서 불리언(6.3), 인터록
4. 자세제어 + 믹서(제어 활성일 때, 프레임의 q·각속도), **베인 펄스 바로 씀** — 프레임 → 베인 레지스터가 FCU 몫의 샘플 → PWM 경로
5. ESC·사출 펄스 (베인 뒤 약 45 µs)
6. 앵커·위상, 배터리 ADC(2 Hz 저역 통과, 감시기는 이전 틱까지 거른 값), 이벤트·링크 기록 포장, 응답 스냅샷(IL_SNAP용), 다음 교환이
   NAV에 돌려줄 FCU 프레임(`nlink_spi_set_reply`), LED·부저, 루프 시간 기록 — 시간임계 부분 끝
7. 기록용 EDF 추력 추정, 5 틱마다 비행 기록 스냅샷

ilink 송신 인터럽트는 시간임계 부분 동안 막아 둡니다(수신은 막을 수 없음). **백그라운드**: 다음 교환이 시작될 수 있는 시각(예상 프레임
− 350 µs − 630 µs)보다 280 µs 앞에 끝나는 작업만 하나씩 시작합니다 — SNAP 송신 → ilink 해독(틱당 16 B까지 다른 일보다 먼저) → 이벤트·링크
기록 → 비행 기록 포장 → IL_STAT → `C` 기록 → `N` 기록 → COM/NAV 텍스트 기록 → 스택 검사 → 1 Hz 상태 기록. 교환 중에는 SPI 슬레이브
인터럽트가 약 4.5k 사이클 CPU를 쓰므로 그때 끝나지 않은 작업이 없어야 합니다.

안전 불변식: ESC는 리셋 뒤 첫 PWM 프레임부터 POWERED 외에는 정지 펄스; 아밍은 SPEC §8.8의 모든 조건(아밍 스위치, 센서, 정렬,
기울기, 배터리, 살아 있는 링크)을 갖춘 ARM 명령으로만; 아밍 스위치 열림 = KILL; 무선과 센서는 다른 칩에 있으므로 무선·I2C 고장이 이
루프를 멈출 수 없음; 링크는 링과 통째 교환에서 읽고, 기다리지 않는 통째 쓰기로 씀; 멈춘 루프는 워치독(60 ms)이 SAFE 부팅으로
되돌리고, 비행 중이었다면 사출을 엶(I10).

### 8.2 NAV 루프 (`src/nav/nav_main.c`)

Timer2 250 Hz가 틱입니다. Timer0 리드 인터럽트가 틱 496 µs 전(31 카운트 × 256 / 16 MHz)에 MPU-6050 버스트 읽기를 시작하므로
틱 때 샘플이 RAM에 있습니다(v0.5 타이밍).
1. (래치할 출력 없음)
2. 센서: IMU 샘플 변환, 실패 계수(실패가 이어지면 26 틱 주기로 `twi_recover`가 버스를 풂), 이번 위상의 MS5611 단계(버스 스크립트는
   3단계 동안 돎)
3. 자세 추정: 패드 모드면 패드 정렬, 아니면 자이로만; 바이어스 보정 각속도, 기울기, 패드 정지 계수
4. 고도 필터: 패드 모드면 0 유지, 아니면 예측 + 기압 보정; 지면 기준(패드에서 저역 통과, 이후 고정); baro_ok(5 틱마다 샘플 + 1 틱 여유);
   aligned(패드를 떠나면 고정); EST_VALID(첫 aligned에서 고정)
5. 항법 프레임 작성 + 교환(SS LOW, 72 B, SS HIGH) — **FCU의 틱이 이때 시작**. 시간임계 부분 끝(틱 → 프레임 전달)
6. FCU 응답 해독: 유효하면 상태·fflags가 **다음 틱**의 모드를 정함, 아니면 마지막 유효값 유지. LED

백그라운드(다음 틱 450 µs 전까지): MS5611 계산(int64 보상, powf 고도, 한 패스에 한 단계), 스택 검사. 그 뒤 CPU는 잡니다.
부팅: SS를 SPI를 켜기 **전에** 출력 H로(FCU 슬레이브가 반쯤 된 교환을 보지 않게), 센서 초기화(MPU-6050 WHO_AM_I 3회, MS5611
리셋·PROM·CRC·첫 D2) 결과는 reset_cause 비트 7/6과 LED로. 모든 I2C 대기는 끝이 있고, 멈춘 루프는 워치독이 리셋합니다.

### 8.3 COM 루프 (`src/com/com_main.c`, `com_link.h`)

고정 마감이 없고 단계마다 일의 양에 한계가 있습니다. 메인 루프 1회 = 워치독 → (COM 틱마다) LED → `radio_service` 한 단계 → 상향
바이트 최대 8개 해독(IL_CMD 즉시 큐) → ilink 바이트 최대 32개 해독 → 비싼 작업 **하나**(응답 1단계, 응답 2단계, IL_STAT 중) → 무선이
비었으면 대기 TELEM 송신 → 스택 검사 64 B. 1회 시간은 IL_STAT의 `loop_us_max`로 FCU에 보고됩니다. 수신 링 64 B(250 kbaud로
2.56 ms)가 1회 시간의 한계입니다.

부팅: 무선 핀 안전(Ebyte M0/M1 LOW = 일반 모드; SX127x 비선택 + 리셋 유지), LED 끔, 워치독, Timer2 틱 + Timer1 자유 계수,
ilink USART0, 인터럽트 켬, `radio_init`(SX127x는 워치독을 차며 약 12 ms), 디코더. COM 틱이 25(100 ms)가 되면 첫 비싼 작업으로
부팅 STAT. COM은 SNAP에서 만든 TELEM 말고는 무선으로 아무것도 보내지 않습니다(`radio_send` 호출 지점은 응답 게이트 뒤 하나뿐).

### 8.4 상태와 명령 (SPEC §8.8, `sim/onboard.py`와 동일 — FCU)

| 상태 | 모터 | 베인 | 사출 |
|---|---|---|---|
| SAFE (부팅) | 1000 µs | 중립 | 래치 |
| ARMED | 1000 µs | 중립 (추정기 정렬 중) | 래치 |
| POWERED | 목표 스로틀까지 0.5 s 램프 | 자세제어 | 래치 |
| UNPOWERED | 1000 µs | 0.5 s 동안 자세제어(스월 감쇠) 후 중립 | 래치 (RELEASE 대기 가능) |
| ABORT | 즉시 1000 µs | 중립 | 0.5 s 뒤 해제 → RELEASED |
| RELEASED | 1000 µs (전원 재투입까지) | 중립 | 해제 |

| 명령 | 받는 상태 | 효과 |
|---|---|---|
| ARM | SAFE | 아밍 스위치 닫힘, 센서 정상, 정렬 완료(추정 창 500 샘플 + 2 s 정지 + 지상 기압 기준), 기울기 < 5°, 셀 ≥ 3.3 V, 직전 2 s 안에 다른 프레임 → ARMED |
| DISARM | ARMED | SAFE |
| LAUNCH(nonce) | ARMED | 텔레메트리 nonce 일치 → POWERED, 목표 = climb_throttle |
| THROTTLE(u, nonce) | ARMED(nonce), POWERED, UNPOWERED | u = 0..climb; u > 0 → POWERED, u = 0 → UNPOWERED (패드에서 u = 0은 거부) |
| CUT | POWERED | UNPOWERED |
| RELEASE | ARMED, POWERED, UNPOWERED | 스로틀 0, 0.2 s 뒤 사출 → RELEASED (대기 중엔 KILL만 받음) |
| KILL | SAFE 외 | ARMED → SAFE, 비행 중 → ABORT |

해당되지 않는 명령은 무시되고 텔레메트리의 REJECTED 플래그와 reason 코드로 보고됩니다. 인터록은 같은 틱의 명령 뒤에
평가되므로 항상 마지막 결정권을 가집니다.

### 8.5 센서 입력의 펌웨어 정의 (FCU가 NAV 프레임에서 만드는 값)

* `imu_ok`: 이번 틱 유효 항법 프레임 && IMU_OK && (EST_VALID || 비행 상태 아님). 25 틱(0.1 s) 연속 false면 I9.
* `baro_ok`: 유효 프레임 && BARO_OK(NAV: 5 틱마다 샘플 + 1 틱 여유). 125 틱(0.5 s) 연속 false면 I9.
* `aligned`: ALIGNED — 패드에서 추정 창(500 샘플) 완료 + 지상 기압 기준 + 2 s 정지. 패드를 떠나면 그 값을 유지(보고용).
* `gyro_saturated`: GYRO_SAT(MPU-6050의 int16 한계값). I3의 회전율은 바이어스 보정 자이로.
* h, v_z, tilt, 각속도, q: 마지막 유효 프레임 값(프레임이 없는 틱에는 그대로 유지).

### 8.6 LED와 부저

| 보드 | 표시 | 뜻 |
|---|---|---|
| FCU | LED 짧게 깜빡(약 1 s마다) | SAFE, 준비됨(정렬·센서·링크 정상) |
| FCU | LED 빠르게(약 4 Hz) | SAFE 미준비, 또는 ABORT |
| FCU | LED 켜짐 + 부저 짧게(1 s마다) | ARMED |
| FCU | LED 켜짐 | POWERED / UNPOWERED |
| FCU | LED 느리게(약 1 Hz) + 위치 비프 | RELEASED |
| FCU | LED 느리게 | `fcu_bench_vanes` 벤치 모드 |
| FCU | LED·부저 빠르게 | I10 복구 부팅 |
| FCU | 부저 빠르게 (SAFE) | 셀 전압 < 3.3 V |
| NAV (우선순위 순) | 빠르게(약 4 Hz) / 느리게(약 1 Hz) / 두 번 깜빡 / 한 번 짧게 / 켜짐에 짧은 꺼짐 / 계속 켜짐 | IMU 없음·고장 / 기압계 없음·늦음 / FCU params tag 다름 / 1 s 넘게 유효 FCU 프레임 없음 / 패드에서 아직 정렬 안 됨 / 정렬됨 또는 비행 중 |
| COM | 빠르게 / 업링크마다 100 ms 켜짐 / 1.02 s마다 32 ms | SX127x RegVersion 틀림 / 상향 프레임 전달 / 정상 대기 |

## 9. 보정 (캘리브레이션)

보정값은 모두 `fc_config.json`에 넣고 `tools/gen_firmware_params.py`를 다시 실행한 뒤 **세 칩을 다시 올립니다**(7.2).

### 9.1 IMU 장착 방향 (NAV 보드)
기체축: +x = 베인 1, +y = 베인 2, +z = 위. 로그에서: 똑바로 세움 accel_z ≈ +9.8; 노즈를 +x로 기울이면 accel_x < 0,
기울이는 동안 gyro_y > 0; 노즈를 +y로 기울이면 accel_y < 0, gyro_x < 0; 위에서 보아 반시계 회전 gyro_z > 0.
맞지 않으면 `imu_mounting`을 `+x,-y,…`로 바꿉니다(반사 조합은 생성기가 거부). MPU-6050은 NAV 보드에 있으므로 NAV 보드를
트레이에 붙인 방향 그대로 확인합니다.

### 9.2 자이로 바이어스 (NAV)
자동: NAV가 패드 모드(FCU 상태 SAFE/ARMED)인 동안 평균(처음 500 샘플 산술, 이후 지수, τ = 2 s). 아밍 전 2 s 이상 움직이지 않아야 합니다
(FCU LED 짧게 깜빡임 = 준비됨(정렬·센서·링크 정상), 빠르게 깜빡임 = 미준비; NAV LED 계속 켜짐 = 정렬됨).

### 9.3 서보 방향과 트림 (`fcu_bench_vanes` 빌드)
부호: δᵢ > 0이면 기체에 +ê_θ 방향 힘 (뒷전은 −ê_θ 쪽). 베인1 +y, 베인2 −x, 베인3 −y, 베인4 +x.
1. FCU에 `fcu_bench_vanes`를 올립니다(NAV, COM은 비행 빌드 그대로). NAV를 연결하고 아밍 플러그를 **뺀** 채 2 s 정지 → NAV 정렬 후
   벤치 모드(FCU LED 느린 점멸). FCU가 BENCH 플래그를 보내면 NAV 추정기는 자이로만 쓰고 고도는 0에 붙잡아 둡니다. ESC는 계속 1000 µs.
2. 위에서 보아 반시계로 돌리면 네 베인의 뒷전이 **모두 반시계**로. 노즈를 베인 1(+x) 쪽으로 기울이면 베인 2, 4의
   뒷전이 **−x**로, 노즈를 베인 2(+y) 쪽으로 기울이면 베인 1, 3의 뒷전이 **−y**로.
3. 반대로 움직이는 베인은 `servo_signs`를 −1로. 비행 빌드(SAFE)에서 베인 중립을 각도계로 0°로 맞추는
   `servo_trims_us`. 기계 한계 ±25°는 펌웨어가 클램프. **확인이 끝나면 FCU에 `fcu`를 다시 올리십시오.**

### 9.4 ESC / 9.5 사출 서보 / 9.6 배터리 전압
ESC 범위는 서보테스터로 1000/2000 µs에 맞춤(로터 제거). 사출 래치 위치 `release_servo`(C15 4 mm 덮음 확인),
`vbat_scale` = 멀티미터 실측 / 로그 값(0.8–1.2).

## 10. 벤치 시험 순서

1. **칩마다 퓨즈·업로드** (5절, 7절). 각 보드를 5 V만 주고 LED 확인(8.6): NAV 혼자면 센서가 정상일 때 "FCU 프레임 없음"(1 s마다
   한 번 짧게), COM 혼자면 1.02 s마다 짧게(형식 B에서 빠르게 깜빡이면 모듈 문제).
2. **NAV + FCU (nlink 연결), 동력 없이**(모터선 분리): FCU 로그 `B`(형식 4, role FCU, params·gains 해시), `T` "NAV boot: reset 0x01,
   flags 0x.., loop max 0x.... us, I2C errors 0x0000", `N` 기록에서 유효 프레임이 초당 약 250개 늘고 bad·timeouts·seq_lost가 부팅
   뒤 늘지 않는지, NAV reset_cause 비트 7–5 = 0. `F` 기록 overruns가 멈춰 있는지. NAV 루프 최대를 11.2와 비교.
3. **+ COM (ilink 연결)**: `T` "COM boot: radio lora_uart ok, version 0x00, reset 0x01, params tag 0x6E match", `C` 기록이 1–2.5 s마다,
   `S` 기록의 `com_age_s` ≤ 3, `com_flags`에 0x80 없음. 스택 여유(`S`) ≥ 1024 B, FCU 루프 시간(`F` loop_us_max)을 11절과 비교.
4. **지상국 링크**: 텔레메트리 수신, 상태 SAFE, `link_ok`. 정렬 후 ARM → ARMED, DISARM → SAFE. 지상국을 끄면 2.0 s 후
   SAFE(I1p). 아밍 플러그를 빼면 ARMED → SAFE. 잘못된 nonce의 LAUNCH는 거부되는지(REJECTED, reason 0x41).
5. **베인**: 9.3의 `fcu_bench_vanes`로 방향·트림 확정 → `fcu` 비행 빌드로 복귀.
6. **모터선 분리 상태의 명령 경로**: ARM → LAUNCH → POWERED에서 베인이 움직이고 ESC 펄스가 1000→2000 µs로 0.5 s 램프되는지
   로그로 확인. CUT → UNPOWERED(0.5 s 뒤 베인 중립). KILL → ABORT → 0.5 s 후 사출. 지상국 끄기 → 3.0 s 후 ABORT.
7. **칩·선 고장 주입 (모터선 분리, 사출 링크 분리)** — co-simulation에서만 확인한 동작입니다(미검증):
   ARMED에서 nlink 커넥터 빼기 → 0.1 s 뒤 SAFE(I9). ARMED에서 ilink 커넥터 빼기 → 2.0 s 뒤 SAFE(I1p). POWERED에서 nlink 빼기
   또는 NAV 리셋 버튼 → 0.1 s 뒤 ABORT(I9), 0.5 s 뒤 사출. POWERED에서 COM 리셋을 짧게 → 상태 변화 없이 링크 복귀(공백 < 3 s).
   `E` 기록의 원인과 `N`·`S` 카운터를 봅니다.
8. **지상 거리 시험**: 6.8절.
9. **로드셀 계류 추력시험**: 노즈콘/낙하산 제거 또는 사출 링크 분리. 두 부분으로 나눕니다.
   * 0–50 % (`fcu_bench_tether`, 스로틀 50 % 제한): ARM → LAUNCH → 수 초 → CUT/KILL. 이 빌드는 50 %를 넘지 않으므로 100 % 추력
     (≥ 30 N, ≤ 120 A, `docs/시험_안전_법규.md` 2단계)은 잴 수 없습니다.
   * 50–100 % (비행 빌드 `fcu`, 스로틀 제한 없음): 지상국을 `--allow-throttle-cmd`로 켜고 `throttle <u>`로 단계마다 올립니다(u는
     `climb_throttle` 1.0에서 잘림, ARMED에서 시작할 때 확인을 물음). LAUNCH는 0.5 s 만에 1.0까지 오르므로 곡선이 아니라 100 % 한
     점입니다. POWERED가 25 s(`max_motor_time_s`, I7)를 넘으면 CUT됩니다.
   로그 스로틀과 로드셀 추력으로 `edf.thrust_nominal_N`, `throttle_exponent` 검증 → params → 시뮬레이션 재튜닝 → 생성기 → 세 칩 다시 올림.
   (90 mm EDF 6S는 2.3–3 kW 급 — 흡입구·배기 근처 접근 금지, 배터리 분리 상태에서만 접근.)
10. **계류 호버** (`fcu_bench_hover`): 짧은 줄로 계류, 사출 링크 분리. 지상국 THROTTLE로 호버 근처 유지, 자세제어 확인.
    기울기 45°·회전율·링크 손실·KILL 시 ABORT. 로그로 자세오차, 베인 포화, 루프 시간, 왕복 시간 확인.

## 11. 시간 (simavr 측정, 실기 아님)

> v0.9: 이 절의 수치는 v0.6.3(세 칩) 기록입니다. 네 칩 v0.9의 값은 0.7절, 4칩 공동 시뮬레이션은 `tools/run_cosim4.py`
> (`tools/cosim/README.md`)입니다. 아래 `run_cosim.py`(세 칩)는 탄도 빌드용으로 남아 있습니다.

모두 PlatformIO `tool-simavr`(ATmega128 코어, 16 MHz, 명령어 단위 사이클 정확)에서 실제 펌웨어를 돌린 결과입니다. 단독 측정은 상대
칩·센서·지상국이 가짜이고, co-simulation은 세 칩을 한 프로세스에서 선로(SPI, USART)로 묶어 돌립니다(`tools/cosim/README.md`).

### 11.1 FCU 단독 (`run_simbench.py`, `fcu_simbench`, 12 s, 가짜 NAV·COM)

| 구간 (비행, 최대) | µs |
|---|---|
| NAV 프레임 꺼내기 + `nl_decode_nav`, 아밍 스위치 | 218 |
| ilink 수신 (틱 시작 때 대기 바이트 중 IL_CMD를 끝낼 수 있는 것) | 70 |
| 온보드 감시기 (명령, 인터록, 프레임의 센서 불리언) | 237 |
| 자세제어 + 믹서 | 1,247 |
| 베인 펄스 쓰기 | 98 |
| ESC/사출 쓰기, 앵커, 배터리, 기록, 응답 스냅샷, NAV 응답, LED, 시간 기록 | 218 |
| **시간임계 부분 합계 (측정 최대)** | **1,944 (48.6 %)** |
| + 인터럽트 (ilink RX 56 B × 51 사이클 = 178.5 µs, 로그 UDRE 12 × 46 = 34.5 µs) | 2,157 (53.9 %) |
| 틱 시작 → 마지막 베인 레지스터 | 1,778 |
| 백그라운드 포함 최대 / 백그라운드 한 작업 최대 | 3,037 / 434 (NAV 텍스트 기록) |
| 비교: v0.6 FCU (센서·추정 포함, 2칩 구성) | 2,971, 인터럽트 포함 3,309 (82.7 %) |

같은 실행에서: IL_CMD 16개 모두 0 틱 지연으로 응답, NAV 리셋(틱 2,500) → 25 틱 뒤 ABORT(I9), 대체 틱 간격 4,004–4,027 µs,
ilink 폭주(틱당 100 B, 1.5 s)와 느린 폭주(300 µs마다 IL_CMD 헤더) 동안 비행 기록 누락 0, 시간임계 부분 최대 1,944 µs 그대로.

### 11.2 NAV 단독 (`run_nav_simbench.py`: `nav_simbench` 12 s + `nav_cosim` 15 s 단독 + 비행 빌드 SS 파형)

| 구간 (비행, 최대) | µs |
|---|---|
| 센서 (IMU 변환, 실패 계수, MS5611 단계) | 108 |
| 추정기 + 기울기 + 패드 정지 + 고도 필터 | 1,202 |
| 프레임 작성 + `nl_encode_nav` | 194 |
| nlink 교환: 측정 빌드의 우회 / **실제 선로(`nav_cosim`)** | 69 / **643** |
| FCU 응답 해독 + LED | 30 |
| **시간임계 부분 (틱 → 프레임 전달), 선로 기준 + MS5611 TWI 인터럽트 107 µs** | **2,247 (56.2 %)** (패드: 선로 기준 1,825, TWI 제외) |
| IMU 샘플(리드 496 µs 전) → 프레임 준비 | 2,757 |
| 백그라운드(MS5611 계산, 스택 검사) / 백그라운드 포함 루프 | 590 / 2,641 — 다음 IMU 리드(3,504 µs) 전에 끝남 |
| 틱 간 프레임 준비 시각 최대 증가 | 271 (+TWI) |

비행 빌드(센서 없음)를 simavr에서 6 s 돌린 SS 파형: 교환 1,497회, SS LOW 634.3 µs, 교환 간격 3,997.6–4,002.6 µs. 판정은 FCU의
프레임 시간초과 4.6 ms 기준(틱 위 여유 600 µs)입니다: 틱 간 프레임 준비 시각 최대 증가 271 µs, `nav_cosim` 단독 실행의 SS 간격
3,859–4,139 µs. 세 칩 기준은 11.4를 보십시오.

### 11.3 COM 단독 (`run_com_simbench.py`, 15 s, 가짜 지상국·FCU)

| 구간 (최대) | lora_uart µs | sx127x µs |
|---|---|---|
| 틱 작업(LED) + `radio_service` 한 단계 | 17 | 253 |
| 상향: radio_read → lp 디코더 → IL_CMD (+ STAT) | 128 | 163 |
| ilink: 읽기 → `il_decoder_push` → 응답 게이트 → SNAP | 136 | 149 |
| 비싼 작업 하나 (응답 1/2단계 또는 IL_STAT) | 662 | 734 |
| 대기 TELEM → radio_send, 스택 검사 | 167 | 158 |
| **메인 루프 1회 최대 / 평균** | **821 / 95.1** | **907 / 102.8** |
| 상향 완료 → IL_CMD 큐 | 232 | 375 |
| SNAP 완료 → TELEM 무선 | 1,565 | 1,634 |

판정(모두 PASS): 모든 TELEM이 Python 파이프라인과 같음(29 / 27개, 각도 차 0), 모든 상향이 IL_CMD로 그대로 FCU에, D4 STAT 간격,
D5 일부러 보낸 나쁜 SNAP 3개 거부, R3 부팅 STAT 100.3 ms, R4 반대 방향 프레임 2개를 헤더에서 거부, 스택 여유 ≥ 1024 B.

### 11.4 세 칩 co-simulation (`run_cosim.py`, 102 실행)

정상 비행(15 s): ARM 3 s, LAUNCH 4 s, 동력 비행. COM 시작 지연 0–19.5 ms(0.5 ms 간격)와 NAV 시작 지연 0–1.75 ms를 짝지은 40가지
위상 × 무선 2종 = 80 실행. ilink 버스트가 NAV/FCU 틱과 교환의 모든 위상에 떨어지게 하려는 것입니다.

| 항목 | lora_uart | sx127x |
|---|---|---|
| FCU 시간임계 (프레임 → 끝, 모든 인터럽트 + 항상 바쁜 로그 포트) 최대 | 2,118 µs (53.0 %) | 2,165 µs (54.1 %) |
| 같은 값 p50 / p99 | 1,849 / 1,992 µs | 1,849 / 1,992 µs |
| NAV 시간임계 (틱 → 프레임 전달, + TWI) 최대 | 2,240 µs (56.0 %) | 2,240 µs (56.0 %) |
| **샘플 → PWM** (IMU 샘플 → 마지막 베인 레지스터) p50 / p99 / 최대 | 4,184 / 4,528 / **4,658 µs** | 4,185 / 4,528 / **4,655 µs** |
| 그중 NAV 틱 → 프레임 / FCU 감지 / 프레임 → 베인, 최대 | 2,150 / 51 / 1,922 µs | 2,149 / 48 / 1,930 µs |
| 상향 도착 → TELEM 무선 p50 / 최대 | 10.7 / 12.5 ms | 9.4 / 13.2 ms |
| 상향 → IL_CMD 큐 최대 | 0.30 ms | 0.52 ms |
| IL_CMD → 감시기 | 0 틱 (1,240건) | 0 틱 (1,200건) |
| COM 메인 루프 1회 최대 | 848 µs | 842 µs |
| SS LOW (교환) | 634–642 µs | 634–642 µs |
| FCU 틱 간격 | 3,844–4,333 µs | 3,834–4,352 µs |
| FCU 백그라운드 끝 → 다음 교환 최소 여유 | 233 µs | 227 µs |
| 틱 간 NAV 프레임 시각 최대 증가 (한계 600 µs) | 436 µs | 450 µs |
| 놓친 틱, 잃은 항법 프레임, 거부된 ilink 프레임 | 0 | 0 |

* 샘플 → PWM의 6자유도 모델값은 4.71 ms입니다(`sim/fc_timing.py`: 이전 co-simulation 실행의 최대 4,709 µs를 올림 — 이번 최대보다
  큼). co-simulation의 샘플 → PWM 판정 한계도 한 틱(4 ms)이 아니라 이 4.71 ms입니다(`run_cosim.py`에 이유를 적음: 6자유도
  시뮬레이션·게인 튜닝·몬테카를로가 모두 이 지연으로 비행함). 4 ms를 넘은 틱 수는 참고로만 보고합니다(lora_uart 108,075,
  sx127x 108,210 / 149,915 틱). 게인은 이 지연으로 튜닝했고, 지금 `params.json`(`0a5c261b…`)으로 다시 돌린 튜닝도 같은 게인
  값을 냈습니다(`src/gains.h` `GAINS_PARAMS_STALE` 0).
* 고장 실행까지 합한 FCU 최대는 2,578 µs(64.4 %): NAV 재부팅 직후 첫 프레임 틱(그중 프레임 감지 지연 913 µs). 관측이 아닌 최악
  합성(인터럽트 없는 최악 + ilink 버스트 또는 IL_CMD 해독 + 42 B STAT + 바쁜 로그 포트)은 2,802 µs(70.0 %).
* nlink 데이터 무결성: 모든 실행에서 FCU가 쓴 프레임 377,610개가 NAV가 보낸 것과 같고, FCU·NAV 디코더가 `sim/nlink_proto.py`와
  일치, 손상된 응답을 받아들인 적 0.

### 11.5 고장 시나리오 (co-simulation, 6 s 동력 비행 중부터, 두 무선 모두 PASS; 수치는 lora_uart)

| 시나리오 | 결과 |
|---|---|
| `cut`: ilink 양방향 4 s 절단 | 마지막 IL_CMD 750 틱(3.0 s) 뒤 ABORT(I1), 125 틱(0.5 s) 뒤 RELEASED |
| `reset`: COM 0.5 s 리셋 | 상태 변화 없음, 상향 공백 194 틱(0.78 s) < I1. FCU 로그 `T` "COM boot: radio lora_uart ok, version 0x00, reset 0x02, params tag 0x6E match" |
| `babble` / `babble_cmdhdr`: COM이 1 s 동안 ilink에 쓰레기 / IL_CMD 헤더 반복 | FCU 시간임계 최대 2,226 / 2,279 µs (55.6 / 57.0 %), 항법 프레임 손실 0, 비행 기록 누락 0, 상태 변화 없음, 폭주 감지, 링크 복귀(공백 330 틱) |
| `nav_hold`: NAV를 리셋으로 잡음 | 마지막 프레임 틱 4,615 µs 뒤 첫 대체 틱, 이후 약 4,010 µs마다(Timer2, 간격 4,004–4,041 µs). 마지막 유효 프레임 25 틱 뒤 ABORT(I9), 125 틱 뒤 RELEASED. 판정 한계: 첫 간격은 4,600 ± 60 µs, 그 뒤 간격은 4,000 − 20 … 4,000 + 60 µs(`run_cosim.py`에 이유를 적음: 대체 틱은 기다리던 ilink 바이트 해독과 그때 쌓인 수신 인터럽트를 먼저 끝내므로 수십 µs 늦게 시작할 수 있고, 대체 틱은 틱 수로 세는 I9 ABORT까지만 돎) |
| `nav_pulse`: 교환 중(30번째 바이트) NAV 리셋 1 ms + 리셋 지연 65 ms | 부분 교환 버림. 74.4 ms 뒤 프레임 재개(IMU_OK, BOOT, EST_VALID 없음) → 25 틱 뒤 ABORT(I9), 125 틱 뒤 RELEASED |
| `nl_flip`: 8번째 교환마다 MOSI·MISO 비트 반전 (1 s) | 손상 63회 모두 거부, 유효 프레임 없는 FCU 틱 32, 틱 간격 최대 4,124 µs, 상태 변화 없음 |
| `nl_drop`: 8번째 교환마다 MOSI 바이트 누락 | 32회 모두 거부(재동기 32, 대체 틱 32, 간격 최대 4,620 µs — 시간초과 4,600 µs 뒤 대체 틱이 위와 같은 이유로 조금 늦음. `nl_*` 판정 한계는 시간초과 + 60 µs, sx127x 최대 4,627 µs), 상태 변화 없음 |
| `nl_extra`: 8번째 교환마다 여분 SCK 바이트 | 32회 모두 거부, 간격 최대 4,133 µs, 상태 변화 없음 |
| `nl_noise`: 두 선에 비트 오류 2e-4 | 유효 프레임 없는 FCU 틱 23, NAV가 거부한 FCU 응답 4, 상태 변화 없음 |

### 11.6 인터럽트 사이클 (비행 ELF 역어셈블, 응답 4 + 벡터 JMP 3 포함)

| 칩 | 인터럽트 | 사이클 (보통 / 최장) | 메모 |
|---|---|---|---|
| FCU | nlink SPI 슬레이브 (벡터 17) | 62 / 138 (교환 끝) | 교환 하나 약 4.5k 사이클(280 µs). 다음 응답 바이트는 SPIF 뒤 23 사이클에 씀 |
| FCU | ilink RX (30) | 51 / 57 | 손으로 쓴 ISR |
| FCU | ilink UDRE (31) | 47 | v0.6.3부터 손으로 씀. 시간임계 부분 동안 막음 |
| FCU | 로그 UDRE (19) | 46 | |
| FCU | 타이머·TWI | 없음 | 교환 중 돌 수 있는 FCU 인터럽트는 모두 ≤ 58 사이클(측정 한계, 검사). 설계 목표는 약 50 사이클(`nlink_spi.h`) |
| NAV | Timer2 틱 (9) / Timer0 IMU 리드 (15) / TWI (33) | 65 / 153 / 156 | SPI 인터럽트 없음(마스터). MS5611 스크립트 11회 = 107.2 µs, MPU-6050 버스트 19회 = 194.8 µs |

co-simulation에서 응답 바이트 쓰기의 최소 여유는 **0 사이클**이었습니다: SPIF 때 ilink 인터럽트가 돌고 있으면 쓰기가 SPIF 뒤 최대
76 사이클까지 늦어집니다. 정상 비행에서 무선마다 150,000 교환 중 늦은 응답 바이트는 lora_uart 0개, sx127x 1개(10번째 이후, 무해),
1–9번째 늦은 바이트 0개(15절).

### 11.7 연산 비용 (`fcu_simbench_micro`, 32회 평균, 사이클)

attitude_ctrl_update 7,440, onboard_update(하트비트 1개) 675, (프레임 없음) 399, nl_decode_nav(유효) 2,792, nl_encode_fcu 631,
il_decoder_push: 14 B IL_CMD 1,033 / 42 B IL_STAT 3,321 / 사냥 중 바이트 27, il_encode(SNAP 55 B) 2,069, log_pack_flight 8,255,
atan2f 2,921, asinf 2,582, acosf 2,472, powf 4,850, sinf 1,700.

### 11.8 시뮬레이터가 다루지 않는 것 (분석으로 더함, 실기 확인 필요)

* NAV의 I2C: 센서가 가짜라 TWI 인터럽트가 돌지 않습니다. MS5611 스크립트(11회, 107.2 µs)를 해당 위상의 틱에 더하고, MPU-6050
  버스트(19회, 194.8 µs)는 틱 전 리드 시간에 돈다고 보았습니다(상한 판정에는 더함).
* simavr의 Timer0은 `timer.c`의 TCNT0 미리 넣기를 무시합니다 — IMU 리드는 Timer2 비교 496 µs 전에 놓고 계산했습니다.
* 잠든 코어를 깨우는 인터럽트의 추가 4 사이클(0.25 µs)은 모델에 없습니다.
* 실제 SPI 신호(2 MHz, 보드 사이 선), USART 선로, LoRa 모듈 내부 지연(`*_ASSUMED`), 실제 칩의 리셋 지연(65 ms는 데이터시트의
  대표값)은 측정하지 않았습니다.

## 12. 로그 (형식 4)

FCU가 OpenLog로 57600 baud 이진 프레임을 씁니다: `0xA5 0x5A | type | len | payload | Fletcher-16`. 형식 4는 v0.6에서 정했고,
v0.6.3은 바이트 배치를 그대로 두고 `N` 기록만 **더했습니다**(모르는 디코더는 건너뜀). 센서와 추정기가 NAV로 갔으므로 `F`의
q / accel / h_est / vz_est / h_baro / tilt와 `S`의 twi_errors / baro_fails / imu_fails는 항법 프레임에서 온 NAV의 값입니다.

| type | 주기 | 내용 |
|---|---|---|
| `B` | 부팅 | 형식 4, FCU 리셋 원인, 루프 Hz, 카운트/µs, "fw 3.1.0 role FCU BUDGET_90mm_6S params ⟨sha⟩ gains ⟨sha⟩" |
| `T` | 사건 | 텍스트: "RECOVERY (I10) …", "COM boot: radio … ok/FAIL, version 0x.., reset 0x.., params tag 0x.. match/MISMATCH"(COM의 첫 STAT과 COM 재부팅마다), "NAV boot: reset 0x.., flags 0x.., loop max 0x.... us, I2C errors 0x...."(NAV의 첫 프레임과 재부팅마다) |
| `F` | 50 Hz | 틱, 상태, 플래그(제어/포화/사출/링크/인터록/스위치/센서/벤치), reason, 보조 비트(정렬/복구/응답대기/프레임수신/거부/무선), **FCU 루프 시간(직전/최대, 프레임 → 틱 끝)**, overruns(**유효 항법 프레임 없이 돈 FCU 틱**), q, 필터 자이로, 가속도, 자세오차, 모멘트, 적분기, δ1–4, PWM 6개, h/v_z, 기압 고도, 스로틀, 셀전압, 기울기, 추력 추정 |
| `E` | 전이 때 | 이전/다음 상태, 원인(명령 / 인터록 / 사출 타이머), reason |
| `L` | 상향 CMD마다 | SEQ, 명령, arg, nonce, 결과(실행/거부 코드/하트비트/반복), 처리 후 상태, RSSI·SNR(COM이 IL_CMD에 실어 보냄) |
| `S` | 1 Hz, 40 B | 스택 최소 여유, I2C 오류(NAV), 로그 손실, COM 링크 카운터(최신 IL_STAT: 상향 프레임 수, CRC·길이 오류, 다른 기체·형식, 무선 오류, 응답 수, RSSI), 기압·IMU 실패(NAV), 배터리 mV, FCU 리셋 원인, FCU ilink 카운터(IL_CMD, SNAP, 거부, UART 오류, 버림), COM STAT 나이(초), COM 플래그(\| 0x80 = params tag 불일치) |
| `C` | IL_STAT마다 (1–2.5 s) | FCU 틱 + IL_STAT 36 B 그대로 |
| `N` | 1 Hz, 20 B | 받은 유효 항법 프레임, 검사 실패 교환, 시간초과 틱, 잃은 NAV 틱(SEQ 간격), SPI 재동기, SPI 덮어쓰기, NAV 루프 최대, NAV 플래그, **NAV reset_cause** |

**리셋 원인 비트.** FCU(`B`, `S`)와 COM(IL_STAT → `C`, `T`)은 MCUCSR 그대로입니다: 0x01 전원(PORF), 0x02 외부(EXTRF), 0x04
브라운아웃(BORF), 0x08 워치독(WDRF), 0x10 JTAG(JTRF). NAV의 reset_cause(`N`, `T`)는 비트 4–0이 같은 MCUCSR이고, 비트 7–5가
NAV의 부팅·링크 결과입니다:

| 비트 | 이름 | 뜻 |
|---|---|---|
| 0x80 | NAV_RC_IMU_INIT_FAIL | 부팅 때 MPU-6050 WHO_AM_I를 3번 시도해도 못 알아봄 |
| 0x40 | NAV_RC_BARO_INIT_FAIL | 부팅 때 MS5611 리셋 / PROM 읽기 / PROM CRC / 첫 D2 실패 |
| 0x20 | NAV_RC_TAG_MISMATCH | 마지막 유효 FCU 프레임의 params tag가 NAV 빌드와 다름 (첫 유효 FCU 프레임 전에는 0) |

비트 7/6은 부팅부터 다음 리셋까지 고정, 비트 5는 FCU 프레임을 따라 바뀝니다. 보고용이며 어떤 비행 판단도 읽지 않습니다.

변환: `.venv/Scripts/python.exe firmware/atmega128_fc/tools/decode_log.py LOG00001.TXT` → `LOG00001_flight.csv`,
`_link.csv`, `_com.csv`, `_nav.csv`(NAV reset_cause는 `nav_reset`, `nav_rc`와 비트별 0/1 열로) 및 전이·거부·상태 요약.
형식 3(v0.5 한 칩) 로그도 읽습니다.

## 13. 검증 (호스트, 시뮬레이터)

```
.venv/Scripts/python.exe tools/gen_firmware_params.py                       # 생성 헤더
.venv/Scripts/python.exe tools/gen_gains_header.py
.venv/Scripts/python.exe tools/build_host_test.py --run                    # 단위시험 (zig cc -std=c99 -O1 -ffp-contract=off)
.venv/Scripts/python.exe tools/crosscheck_controller.py                    # 참조 Python vs C 코어 (--only ilink | nlink | log: 한 절만)
.venv/Scripts/python.exe firmware/atmega128_fc/tools/run_simbench.py       # FCU 루프 시간/스택/링크
.venv/Scripts/python.exe firmware/atmega128_fc/tools/run_nav_simbench.py   # NAV 루프 시간/스택/교환 파형
.venv/Scripts/python.exe firmware/atmega128_fc/tools/run_com_simbench.py   # COM 루프/응답 게이트/STAT 간격
.venv/Scripts/python.exe firmware/atmega128_fc/tools/run_cosim.py          # 세 칩 co-simulation (16 스레드에서 약 8분)
.venv/Scripts/python.exe -m pytest firmware/atmega128_fc/tools/test_decode_log.py -q -p no:cacheprovider   # 로그 디코더
.venv/Scripts/python.exe -m platformio run -d firmware/atmega128_fc        # 기본 빌드 전부
```

* **단위시험**(`host_test/test_core.c`, 20,173 검사, 실패 0): `sim/out/link_vectors.json`, `ilink_vectors.json`(디코더 스트림마다
  방향 마스크, 바이트마다 반환 프레임·카운터·`il_decoder_need`/`il_decoder_cmd_need`, SNAP → TELEM), `nlink_vectors.json`(NAV 필드 ↔
  72 B 프레임, FCU 프레임, 두 디코더)의 모든 벡터가 C와 **바이트 단위로 같음**, CRC, 양자화, SPEC §8.8 명령표 전체, 아밍 조건, 램프,
  I1/I1p 정확한 틱, I2–I10, 텔레메트리 필드, 쿼터니언·믹서·PID·추정기·고도필터·EDF·MS5611·로그. 한 바이트 오류 57,600가지에서 다음
  프레임 손실 0, CRC 오수용 0.
* **교차검증**(`crosscheck_report.json`): (1) 제어기 13,624 벡터 44 시퀀스, max |Δδ| = 2.2e-6 rad; (2) **감시기** — 무작위 지상국
  (손실·정전·반복/과거/반바퀴 SEQ·잘못된 nonce·알 수 없는 명령·한 틱에 여러 프레임)과 무작위 고장, 6자유도 기록 4개 비행을 틱 단위로
  재생: 506,232 틱 불일치 0, 모든 인터록·I10·거부 코드 발생; (3) LoRa 프로토콜 3,000 스트림 불일치 0; (4) **ilink** 3,000 스트림(마스크
  ALL / FCU / COM) 750,317 B — 반환 프레임·카운터·need 값 모두 같음, 인코딩·CRC·스냅샷 불일치 0, TELEM 20,000개 중 13개 각도가 0.01°
  한 단위 차(float32 vs float64 삼각함수, 허용); (5) **nlink** NAV·FCU 인코딩 각 5,000, 해독 버퍼 20,000 불일치 0; (6) 추정기 14,400
  단계, 고도필터 45,000 단계; (7) 로그 형식 4 왕복(`N`은 `test_decode_log.py`가 C 패커로 확인).
* float 연산이 개입하는 두 임계값(I3 |ω_xy|, I4 h + k v²)은 float32/float64 반올림 차이가 있을 수 있어, 그 1e-5(상대) 띠 안의
  입력은 양쪽에 넣기 전에 띠 밖으로 옮깁니다(이번 실행 2개).

## 14. CodeVisionAVR에 대하여

평가판(`C:\cvavreval`, V4.07)은 플래시 4 KB 제한이라 이 펌웨어(NAV 약 17 KB, FCU 약 28 KB, COM 약 10–11 KB)는 어느 칩도 들어가지
않습니다. 지원 툴체인은 avr-gcc(PlatformIO 또는 Microchip Studio 7: 역할별 소스 묶음은 `platformio.ini`의 `build_src_filter`,
`-D FC_ROLE_*`, `F_CPU=16000000UL`, `-std=c99 -Os`, `libm`). 정식판으로 옮길 경우: `ISR(...)` → `interrupt [...]`, `<avr/io.h>` →
`<mega128a.h>`, `util/atomic.h`·`sei/cli` → `#asm`, `<avr/wdt.h>`·`PSTR`·`pgm_read_byte`·`<util/delay.h>`·`<avr/sleep.h>` 대응,
`.init1`/`.init3`/`.noinit` 이동, `sqrtf/atan2f/asinf/acosf/powf/lroundf` 이름, `int64_t`(MS5611), `__BYTE_ORDER__` 검사
(log_record.c, link_proto.c), 손으로 쓴 ISR(nlink SPI, ilink RX/UDRE, 로그 UDRE: `ISR_NAKED` + 인라인 어셈블리) — 미검증.

## 15. 알려진 제한과 확인 필요 사항

* **실기·실제 무선 미검증**: 세 칩, LoRa 모듈(형식 A/B 모두), 레벨시프터 경유 SPI 500 kHz, AUX 동작, 안테나 배치, 거리,
  루프 시간·스택은 시뮬레이터와 계산값입니다. 10절 순서대로 확인하십시오.
* **nlink 응답 바이트 여유 0 사이클**: co-simulation에서 SPIF 때 ilink 인터럽트가 돌고 있으면 FCU의 다음 응답 바이트 쓰기 여유가
  0 사이클까지 줄었습니다. 정상 비행에서 무선마다 150,000 교환 중 늦은 바이트는 lora_uart 0개, sx127x 1개였고 10번째 이후라
  무해했습니다. 1–9번째 바이트가 늦으면 NAV는
  그 FCU 프레임을 거부하고 마지막 유효 상태를 유지합니다(패드 → 비행 전환이 그만큼 늦음). 실제 칩의 첫 SCK 에지는 시뮬레이터 기준보다
  약 4 사이클 늦어 여유가 조금 더 있으나 측정하지 않았습니다. FCU 인터럽트를 늘리거나 길게 만들지 마십시오(측정 한계 58 사이클, `nlink_spi.h`의 설계 목표 약 50 사이클).
* **보드 질량·크기는 가정값**: NAV 40 g, FCU 20 g, COM 20 g, 통신·로거 상자 20 g, 보드 50 × 50 × 15 mm. 달고 재서 `params.json`에
  넣고 무게중심·게인을 다시 계산하십시오.
* **OpenLog 레벨시프터 채널 미정**(4.10): 형식 B(sx127x)이면 레벨시프터 2에 빈 채널이 없습니다. 2026-11-06: ESP 빌드(기본)에서는
  ch4로 정해졌습니다(0.10.4). 형식 B로 돌아갈 때만 미정입니다.
* **배선 경로 미정**: 사출 서보선(페이로드 모듈 → FCU PE5), ESC 신호선, 아밍 스위치·부저·LED, 5 V 분배, ilink·nlink·로그선의 길이와
  경로는 CAD에 없습니다. nlink 2 MHz를 보드 사이 선으로 보낸 신호 품질은 확인하지 않았습니다.
* **NAV 재부팅 = 비행 종료**: 비행 중 NAV가 리셋되면(브라운아웃, 워치독) EST_VALID가 없어 I9 → ABORT입니다(설계상 안전 쪽).
  FCU 리셋은 I10(사출). 비행 중 COM 리셋은 상향 공백이 3 s(I1) 안이면 상태 변화가 없습니다(co-sim 0.5 s 리셋 → 공백 0.78 s).
* **부팅 STAT 시각**은 세 칩의 리셋 지연(65 ms, 데이터시트 대표값)이 비슷하다는 가정 위에 있습니다. FCU가 COM보다 약 99.5 ms 넘게
  늦게 리셋을 빠져나오면 부팅 STAT 일부를 잃지만, 다음 STAT이 1–2.5 s 안에 옵니다(비행 영향 없음).
* `lora_uart`는 공중속도 2.4 kbps 가정에서 왕복 437 ms → 최대 2.29 Hz이고 `poll_hz`는 2 Hz입니다. 프레임 1개 손실 후 상향 공백
  1.03 s, I1p 2.0 s. 손실된 명령은 0.5 s 뒤에 다시 나가므로, 사출 명령 하나가 손실되면 노즈가 0.5 s 늦게(하강 속도 약 5 m/s 더)
  열립니다. Ebyte 모듈의 내부 지연은 `sim/link_model.py`의 `*_ASSUMED` 값이므로 실측(지상국 로그의 왕복 시간)으로 바꾸십시오.
* Ebyte 채널 번호 공식, 한국 920 MHz 대역 출력 한도, 433 MHz 사용 가능 여부는 확인하지 못했습니다(3절, 6.5절).
* nonce는 보안 기능이 아닙니다(평문, 16비트). 같은 VEH를 쓰는 다른 송신기를 막지 못하므로 `vehicle_id`를 행사장에서
  겹치지 않게 정하십시오.
* FCU가 I10으로 부팅하면(공중 리셋) 상태가 SAFE이므로 NAV는 고도 필터를 패드 모드로 되돌립니다(h = 0으로 붙잡고 지면 기준을
  다시 거름; fflags와 무관, `nlink_proto.h`). 그 후 텔레메트리의 h는 의미가 없습니다.
* 서보는 50 Hz 프레임에서만 새 값을 받습니다(시뮬레이션이 모델링). 샘플 → PWM 레지스터 최대 4.66 ms, 시뮬레이션 4.71 ms.
* 벤치 빌드(`fcu_bench_*`)는 측정 보고서에 없습니다(크기만 7.1, 스택 미측정).
* 부호 규약: δ 양수 = 기체에 +ê_θ 방향 힘(뒷전 −ê_θ). SPEC §8.2와 같은 기준.
