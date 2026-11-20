# ESP8266 무선 브리지 (UART ↔ ESP-NOW)

기체와 지상국 사이의 무선을 LoRa 대신 Wi-Fi(ESP8266)로 하는 펌웨어입니다 (사용자 결정 2026-11-05, 이번 라운드 2026-11-06).
스케치 하나를 양쪽에 씁니다. 역할은 빌드 플래그로 고릅니다.

| 역할 | 모듈 | 연결 | 빌드 환경 |
|---|---|---|---|
| 기체 (`BRIDGE_ROLE_VEHICLE`) | ESP-12E (VTOL 캐리어 U7에 납땜, 로켓은 COM 보드에 배선) | UART0 ↔ COM USART1 (BSS138 경유), RST ← COM PC6 | `vehicle_esp12e` |
| 지상 (`BRIDGE_ROLE_GROUND`) | ESP-07S 계열 (u.FL, 외부 안테나 8~14 dBi) | UART0 ↔ 3.3 V USB-UART ↔ PC (`tools/ground_station.py`) | `ground_esp07s` |

표시: **M** = 측정값, **E** = 추정값, **A** = 가정값(아직 확인하지 않음).

## 하는 일

* 바이트를 그대로 옮깁니다. COM이 보낸 바이트가 그대로 지상 PC에 가고, 지상국이 보낸 바이트가 그대로 COM에 갑니다.
* 통신 규약(프레임, CRC, 논스, TELEM 형식, 명령)은 바뀌지 않습니다. 검사는 양 끝(COM의 `core/link_proto.c`, 지상국의
  `sim/link_proto.py`)이 합니다. 브리지는 아무것도 고치거나 버리지 않습니다.
* UART → 무선: 완전한 프레임(`A5 5A LEN ... CRC`)은 마지막 바이트가 들어오면 바로 한 패킷으로 보냅니다. 프레임이 아닌
  바이트는 UART가 `idle_flush_bytes`(3바이트 시간, 38400 baud에서 781 µs) 동안 조용하면 보냅니다. 덜 들어온 프레임은
  가장 긴 프레임 시간 + 그 틈(13.0 ms)을 기다립니다. 한 패킷은 250 B(ESP-NOW 한계, A)를 넘지 않습니다.
  규칙은 LoRa 지상 브리지(`firmware/ground_bridge`)와 같습니다. 코드: `src/bridge_core.h`.
* 무선 → UART: 짝(peer) MAC에서 온 패킷만 그대로 UART로 씁니다. 다른 장치의 패킷은 버립니다.
* 시리얼에는 무선 바이트만 나갑니다. 스케치는 아무 글자도 출력하지 않습니다.
* 비행 판단은 하지 않습니다(프로젝트 규칙: 비행 계산은 모두 ATmega128).

## 설정 (모두 `params.json` `flight_computer.link.esp_bridge`에서)

빌드할 때마다 `gen_esp_bridge_config.py`가 `src/esp_bridge_config_gen.h`를 다시 만듭니다. 숫자를 소스에 손으로 쓰지 마십시오.

| 항목 | 값 | 비고 |
|---|---|---|
| UART | 38400 8N1 (양쪽 ESP, COM 모두 같음) | COM UBRR 25, 오차 +0.16 %; ESP 분주 오차 +0.016 % (E, 80 MHz/2083) |
| ESP-NOW 채널 | 1 | 고정 채널. 현장에서 빈 채널로 바꾸십시오 (A) |
| PHY | 802.11b | `WiFi.setPhyMode(WIFI_PHY_MODE_11B)` |
| 비트레이트 | 1 Mbps | ESP-NOW 기본값이 1 Mbps라고 알려져 있음 (A: Espressif ESP-NOW 안내서, 디스크에 없음) |
| 송신 출력 | 기체 20 dBm, 지상 20 dBm | `WiFi.setOutputPower`. 지상 고이득 안테나를 쓰면 EIRP 규정 확인 (A) |
| MAC | 기체 `02:CA:5A:00:00:01`, 지상 `02:CA:5A:00:00:02` | 스케치가 `wifi_set_macaddr`로 직접 정함(로컬 관리 주소). 모듈 MAC을 읽을 필요 없음 |

쓰는 API (ESP8266 Arduino 코어 3.x. **코어 3.1.2로 컴파일했습니다 - 이름과 원형은 M, 실기 동작은 A**, "빌드" 절): `WiFi.mode(WIFI_STA)`,
`WiFi.disconnect()`, `wifi_set_macaddr(STATION_IF, mac)`, `WiFi.setPhyMode`, `WiFi.setOutputPower`,
`WiFi.setSleepMode(WIFI_NONE_SLEEP)`, `wifi_set_channel`, `esp_now_init`, `esp_now_set_self_role(ESP_NOW_ROLE_COMBO)`,
`esp_now_add_peer`, `esp_now_register_recv_cb`, `esp_now_register_send_cb`, `esp_now_send`.
비트레이트 고정 API `wifi_set_user_rate_limit`는 `-D BRIDGE_USE_RATE_LIMIT_API`일 때만 들어갑니다. 코어 3.1.2의 SDK에 있습니다
(이 플래그로 컴파일됨, M). 이 호출이 ESP-NOW 프레임에 정말 적용되는지는 모릅니다(A).
ESP-NOW 프레임에는 `RC_LIMIT_11B`가 아니라 P2P 쪽 제한이 적용될 수도 있습니다(A). 코어 3.1.2의 SDK 헤더(`user_interface.h`)에는
`RC_LIMIT_P2P_11B`라는 이름이 없습니다. P2P는 `RC_LIMIT_P2P_11G`, `RC_LIMIT_P2P_11N`뿐입니다(M, 헤더를 읽음). 그래서 둘을 빌드로
비교할 수는 없고, 11b 1 Mbps로 나가는지는 실기에서 재야 합니다(A).
스케치는 처음에 이 PC에서 **가짜 헤더(stub)로 문법만** 확인했습니다(zig c++, 두 역할, 디버그·속도 플래그 포함, E). 그 뒤
2026-11-06에 실제 코어 3.1.2로 컴파일했습니다(M, "빌드" 절).

## 송신 순서와 진단

* 한 번에 ESP-NOW 패킷 하나만 보냅니다. 앞 패킷의 송신 콜백이 오거나 30 ms가 지나야 다음 패킷을 보냅니다(A: SDK 콜백 시간은
  재지 않음). SDK가 송신을 거절하면 같은 패킷을 다음 반복에서 다시 보냅니다(순서 유지). 50 ms 동안 계속 거절되면 그 패킷을
  버리고 셉니다(무선 손실 한 번과 같음). 링크 규약이 2 Hz 반이중 폴링이라 기다리는 일은 드뭅니다 (검토 LB-3).
* `-D BRIDGE_DEBUG_UART1`로 빌드하면 1초마다 숫자를 GPIO2(UART1, 송신만, 115200 8N1)로 냅니다: 보냄 성공/실패, 거절/버림/
  시간 초과, 짝에게서 받은 패킷, 다른 MAC에서 온 패킷(`foreign`), 하향 링 버림. 거리 시험 때 3.3 V USB-UART의 RX를 GPIO2에
  대고 읽습니다. GPIO2에는 이미 10 kΩ 부팅 풀업이 있고 UART1은 쉴 때 H라서 부팅 조건이 그대로입니다(A). ESP-12E의 모듈 LED가
  GPIO2에 있으면 깜박입니다(A). 기본 빌드에는 없습니다. 링크 UART(UART0)에는 링크 바이트만 나갑니다 (검토 LB-4).
* 첫 실기 시험 확인 목록: 디버그 출력의 `foreign=0`, 받은 패킷의 MAC이 짝의 station MAC인지 (검토 LB-7). 컴파일과 `espnow.h`
  원형 대조는 끝났습니다(코어 3.1.2로 컴파일, M, "빌드" 절).

## 기체 쪽 배선 요약 (자세히: `firmware/README.md` 0.10절)

| ESP-12E 핀 | 연결 | 비고 |
|---|---|---|
| 22 TXD0 (GPIO1) | BSS138 채널 → COM PD2 (RXD1) | 캐리어 Q11 |
| 21 RXD0 (GPIO3) | 1 kΩ ← BSS138 채널 ← COM PD3 (TXD1) | 캐리어 Q4 |
| 1 RST | BSS138 채널 ← COM PC6 (LV 쪽 10 kΩ 풀업 = RST 풀업) | 캐리어 Q5, 비반전: PC6을 L로 몰면 리셋, PC6 입력(Hi-Z) = 동작. PC6은 H로 몰지 않습니다(오픈 드레인) |
| 3 EN | 10 kΩ → 3.3 V, 100 nF → GND | |
| 18 GPIO0 / 17 GPIO2 / 16 GPIO15 | 10 kΩ → 3.3 V / 10 kΩ → 3.3 V / 10 kΩ → GND | 플래시에서 부팅 |
| 8 VCC | 3.3 V (100 µF + 10 µF + 100 nF) | **5 V 금지** |

COM이 **찬 부팅**(전원 투입, 또는 리셋 원인 표시 없음)을 하면 ESP를 리셋 상태로 잡고(PC6 L), 2 ms 뒤 놓습니다(PC6을 입력으로
돌림). 놓은 뒤 400 ms 동안 받은 바이트(ESP8266의 74880 baud 부팅 글자)는 COM이 버립니다(`src/hal/esp_bridge.c`). ESP는 "준비"
바이트를 보내지 않습니다. 준비 전에 온 요청은 응답이 없고, 지상국은 이것을 무선 손실 한 번으로 처리합니다.

COM만 리셋될 때(워치독, 저전압, 외부 리셋)는 ESP를 건드리지 않습니다. ESP는 계속 돌고, 버리는 창도 열지 않습니다. 그래서 비행 중
COM 재시작의 링크 손실은 LoRa 때와 같습니다(검토 F2, 4칩 공동 시뮬레이션 18행). 멈춘 ESP는 COM 리셋으로 살아나지 않습니다.
ESP 자체 워치독(A)이나 전원을 껐다 켜야 합니다.

PC6은 오픈 드레인으로 씁니다. 리셋할 때만 L로 몰고, 놓을 때는 입력(Hi-Z)으로 돌립니다. RST를 H로 올리는 것은 Q5의 풀업입니다.
그래서 COM이 도는 동안 J3 5-6(RST-GND)을 손으로 붙여도 PC6 출력이 GND와 단락되지 않습니다(검토 F1).

## 빌드 (arduino-cli, 2026-11-06)

**ESP8266 펌웨어를 이 PC에서 컴파일했습니다 (M).** 사용자가 설치를 허락했습니다(2026-11-06, "알아서 다 설치하고 진행해줘").
아무 장치에도 굽지 않았습니다. ESP 보드는 연결되어 있지 않습니다.

설치한 것 (M):

* arduino-cli 1.5.1 (`%LOCALAPPDATA%\Programs\arduino-cli\arduino-cli.exe`).
* 코어 `esp8266:esp8266` 3.1.2: 보드 관리자 색인 `https://arduino.esp8266.com/stable/package_esp8266com_index.json`으로
  `arduino-cli core install esp8266:esp8266`. 349.2 MB, `%LOCALAPPDATA%\Arduino15\packages\esp8266`
  (xtensa-lx106-elf-gcc 3.1.0-gcc10.3, mkspiffs, mklittlefs, python3 3.7.2).
* PlatformIO `espressif8266` 플랫폼은 **설치하지 않았습니다.** 그래서 `platformio.ini` 쪽은 아직 빌드하지 않았습니다. 쓰려면
  `.venv\Scripts\python.exe -m platformio pkg install -g -p platformio/espressif8266`(플랫폼 + `toolchain-xtensa` +
  `framework-arduinoespressif8266` + `tool-esptoolpy` 등, 약 250~400 MB, E) 뒤
  `.venv\Scripts\python.exe -m platformio run -d firmware/esp_bridge`. 아래 굽기 순서의 `platformio ... -t upload` 명령도
  이 플랫폼이 있어야 씁니다.

빌드 방법: `.venv\Scripts\python.exe firmware/esp_bridge/build_arduino_cli.py` (변형 이름을 주면 그것만 빌드).

* 먼저 설정 헤더가 최신인지 봅니다(`gen_esp_bridge_config.py --check`). 낡았으면 빌드하지 않습니다.
* `src/`를 `build/sketch/esp_bridge/`로 복사하고 빈 `.ino`를 붙여 컴파일합니다. 코드는 `src/`에만 있습니다. `build/`는 git이
  무시합니다(`.gitignore`).
* 보드: `esp8266:esp8266:generic:eesz=4M2M,baud=115200` (ESP-12E, ESP-07S 모두 4 MB 플래시). 플래그 `-Wall -Wextra`.
* 결과: `build/<변형>/esp_bridge.ino.bin`, 로그 `build/<변형>/compile.log`, 요약 `build/build_results.json`.

빌드 결과 (M, 2026-11-06):

| 변형 | 플래그 | .bin 크기 | sha256 |
|---|---|---|---|
| `vehicle` | `-DBRIDGE_ROLE_VEHICLE` | 275,536 B | `e805a9a12fa29a873dc33463cbe96025744a98c6d128cecec8f2f1e012e79a6c` |
| `ground` | `-DBRIDGE_ROLE_GROUND` | 275,536 B | `fdc8a7bb89b8b37626c6e9f3c3ec07eca1fcfbebd0a736bfa5838a475b508586` |
| `vehicle_debug` | 기체 + `-DBRIDGE_DEBUG_UART1` | 275,824 B | `0bec3f5030803508ae22dab78dc0bbe388c496fc9b2229d32aac33039de7e5f1` |
| `vehicle_ratelimit` | 기체 + `-DBRIDGE_USE_RATE_LIMIT_API` | 275,552 B | `689729263d4c4eb2bc40d37473799675869eaaf9bdf6edd1fce27f4ccca0ae76` |
| `ground_debug_ratelimit` | 지상 + 디버그 + 속도 고정 | 275,840 B | `0b1720929c40f4f372e928ae74e8876f6633167e1e5cc823735f72261e688075` |

* 5개 모두 exit 0입니다. 브리지 소스(`main.cpp`, `bridge_core.*`)의 경고는 0개입니다.
* 두 번 빌드해서 sha256이 같았습니다(결정적).
* 메모리 (`vehicle`): RAM 29,820 / 80,192 B (37 %), IRAM 59,759 / 65,536 B (91 %), 플래시 코드 241,920 / 1,048,576 B (23 %).
  디버그 빌드는 RAM 29,928 B입니다.

컴파일이 보여 주는 것 (M): 스케치가 부르는 ESP8266 API가 코어 3.1.2에 있고 원형이 맞습니다. `WiFi.mode`, `wifi_set_macaddr`,
`WiFi.setPhyMode`, `WiFi.setOutputPower`, `WiFi.setSleepMode`, `wifi_set_channel`, `esp_now_*`(콜백 포함),
`wifi_set_user_rate_limit`.

컴파일이 보여 주지 않는 것 (A): 실기 동작입니다. ESP-NOW가 정말 1 Mbps로 나가는지, 속도 고정 호출이 ESP-NOW 프레임에
적용되는지(`RC_LIMIT_11B` 대 P2P), 송신 출력, 거리. 굽고 재야 합니다.

이 PC에서 확인한 것 (M): 휴대형 부분(`src/bridge_core.*`)의 호스트 시험 `tools/test_esp_bridge.py`
(zig c++, 무작위 흐름 400개 + 몰아넣기 400개 + 링 경계 등 검사 2,264,897개, 실패 0; 검토가 지적한 두 변이를 넣으면 각각
1685개, 211개 실패), 설정 생성기 검사, COM 쪽 백엔드의 simavr 시험
(`firmware/atmega128_fc/tools/esp_bridge_test`).

## 굽기 (기체 ESP-12E, 캐리어)

1. J26을 끼웁니다(COM을 리셋 상태로: PC6, PD3가 Hi-Z → ESP RST는 풀업으로 H, RXD0 선은 H로 쉼).
2. J3 1-2번(IO0-GND)에 점퍼를 끼웁니다(다운로드 모드).
3. 로직 배터리를 켭니다(J15).
4. 3.3 V USB-UART: RX → J3.3, TX → J3.4, GND → J3.2. J3.5-J3.6을 잠깐 붙여 리셋합니다.
5. `... -e vehicle_esp12e -t upload --upload-port COMx`
6. J3 점퍼와 J26을 빼고, J3.5-J3.6을 다시 잠깐 붙여 ESP를 리셋합니다(J26을 빼서 생긴 COM 리셋은 외부 리셋이라 ESP를 리셋하지
   않습니다). 또는 로직 전원을 껐다 켭니다(찬 부팅: COM이 ESP를 리셋함).

## 지상 ESP-07S 배선과 굽기 (검토 D2)

지상 모듈은 캐리어가 없는 맨 모듈입니다. 아래처럼 작은 기판(만능 기판)에 꾸밉니다. 풀업·풀다운 값과 핀 이름은 ESP-12E 캐리어와
같은 값을 썼습니다(A: Espressif ESP8266EX 하드웨어 설계 안내서와 Ai-Thinker ESP-07S 데이터시트는 디스크에 없음, 확인하십시오).

| ESP-07S 핀 | 연결 | 비고 |
|---|---|---|
| VCC (3V3) | 별도 3.3 V 레귤레이터(USB 5 V → AMS1117-3.3 등, 500 mA 이상, A) + 100 µF + 100 nF | USB-UART의 3.3 V 핀은 송신 첨두 전류(수백 mA, A)를 못 댐. **5 V 금지** |
| GND | USB-UART GND, 레귤레이터 GND 공통 | |
| TXD0 (GPIO1) | USB-UART RX | 3.3 V 신호 USB-UART만 (5 V USB-UART 금지) |
| RXD0 (GPIO3) | USB-UART TX | |
| EN (CH_PD) | 10 kΩ → 3.3 V, 100 nF → GND | |
| RST | 10 kΩ → 3.3 V, 누름 버튼 → GND | 리셋 |
| GPIO0 | 10 kΩ → 3.3 V, 점퍼 → GND | 점퍼를 끼우고 리셋 = 굽기 모드 |
| GPIO2 | 10 kΩ → 3.3 V | (디버그 빌드: UART1 송신 출력) |
| GPIO15 | 10 kΩ → GND | |
| 안테나 | u.FL에 2.4 GHz 안테나(8~14 dBi) | **안테나를 먼저 단 뒤 전원** (A) |

굽기와 실행:

1. GPIO0 점퍼를 끼웁니다.
2. 레귤레이터에 전원을 넣고 RST 버튼을 누릅니다(굽기 모드로 부팅).
3. `.venv\Scripts\python.exe -m platformio run -d firmware/esp_bridge -e ground_esp07s -t upload --upload-port COMy`
4. GPIO0 점퍼를 빼고 RST 버튼을 누릅니다(플래시에서 실행).
5. `tools/ground_station.py --port COMy` (기본 `--link-hw esp_bridge`, 38400 baud; `docs/지상국_사용법.md` 0절).

**어떤 장치도 이 라운드에서 굽지 않았습니다.**

## 비행 전 조건

* 지상 거리 시험 1 km 이상(예상 최대 거리의 2배)을 먼저 합니다. 실패하면 LoRa로 돌아갑니다(`flight_computer.link.vehicle_radio`
  = `lora_uart` 또는 `sx127x`, 다시 생성·빌드). 계산 링크 예산(E): 20 dBm − 2 dBi(기체, 동체 안) + 8 dBi(지상) − 자유공간 손실
  − 10 dB 여유 → 1 km에서 약 −84 dBm (감도 약 −98 dBm, A) (`pcb/vtol_carrier/spec/carrier_spec.json` `radio_decision_2026_11_05`).
* 왕복 시간을 실제로 재십시오. 2026-11-10 제어 라운드부터 시뮬레이션과 지상국은 ESP-NOW 링크 모델(`sim/link_model.py`
  `esp_bridge`)을 씁니다: 정상 왕복 51.7 ms(탄도 TELEM v1) ~ 57.0 ms(VTOL TELEM v7), MAC 재전송을 모두 쓰면 146~152 ms,
  응답 제한 240~246 ms (모두 E/A: 38400 baud UART 두 구간 + 브리지 처리 1 ms(A) + 802.11b 1 Mbps 프레임(E)). 실측 왕복이
  모델과 다르면 `ESP_HOP_PROCESS_S_ASSUMED` 등 상수를 고치십시오. 지상국의 `t_link`는 실측 왕복으로 계속 갱신됩니다(규칙 R-ESP1).
* ESP-12E와 지상 모듈의 KC 인증, 고이득 안테나의 EIRP 한도를 확인하십시오 (A).
