# 지상 브리지 (USB ↔ LoRa, SX127x 모듈용)

> 2026-11-06: 기본 무선은 ESP8266(ESP-NOW)입니다(`firmware/esp_bridge/README.md`). 이 브리지는 **LoRa 예비** 장비로 남습니다.
> 쓰려면 기체를 `flight_computer.link.vehicle_radio` = `sx127x`로 빌드하고, 지상국을 `--link-hw sx127x --radio sx127x`로 실행합니다.

지상 LoRa 모듈이 **SX1276 계열(SPI)** 일 때만 필요합니다.
Ebyte E22/E220/E32 같은 UART 투명 모듈이면 USB-UART 어댑터만 쓰고 이 브리지는 필요 없습니다.

브리지는 바이트를 그대로 옮깁니다. 지상국 프로그램이 보낸 바이트가 그대로 전파로 나가고, 받은 패킷이 그대로 PC로 갑니다.
CRC·기체 번호 검사는 양 끝(지상국 프로그램, 기체)이 합니다. 브리지는 아무것도 버리지 않습니다.

## 보드 선택: Arduino Nano (ATmega328P)

* 비행 컴퓨터와 같은 AVR 툴체인(PlatformIO `atmelavr`)이라 이 PC에서 바로 빌드·검증됩니다.
* 싸고 구하기 쉽습니다. USB 시리얼이 내장돼 있습니다.
* 5 V 보드라서 레벨시프터가 필요합니다. 기체와 같은 규칙(SPEC §10.3)입니다.
* ESP32는 3.3 V라 시프터가 필요 없지만, 툴체인이 크고 기체와 생태계가 달라 고르지 않았습니다.
* Arduino Uno도 핀이 같아 그대로 됩니다 (`-e uno`).

## 모듈

* 주파수 `params.json` `flight_computer.link.sx127x.freq_hz` = 922.1 MHz (920 MHz 대역).
* **Ra-01 / Ra-02 / RFM96 은 433 MHz 대역 부품이라 922 MHz에서 쓸 수 없습니다.**
  RFM95W(868/915 MHz) 또는 Ra-01H(803–930 MHz) 같은 900 MHz대 SX1276 모듈을 쓰십시오. 국내는 KC 인증품.
* 안테나를 연결하기 전에는 절대 송신하지 마십시오. 출력단이 망가집니다.

## 배선

| Nano | 거치는 곳 | 모듈 |
|---|---|---|
| D13 (SCK) | BSS138 채널 1 | SCK |
| D11 (MOSI) | BSS138 채널 2 | MOSI |
| D10 | BSS138 채널 3 | NSS |
| D9 | BSS138 채널 4 | RESET |
| D12 (MISO) | 직결 | MISO |
| D2 (INT0) | 직결 | DIO0 |
| 5V | BSS138 HV | — |
| 3.3 V 레귤레이터 출력 | BSS138 LV | VCC |
| GND | 공통 | GND |

* 3.3 V는 별도 레귤레이터(예: AMS1117-3.3, 5 V 입력)로 만듭니다. Nano의 3V3 핀은 송신 전류를 못 댑니다.
* 모듈 → Nano 선(MISO, DIO0)은 직결해도 됩니다. 3.3 V가 ATmega328P의 입력 High 문턱(0.6 × 5 V = 3.0 V)보다 높습니다.
* BSS138 보드는 느려서 SPI를 1 MHz로 돌립니다 (`src/bridge_pins.h` `BRIDGE_SPI_HZ`, 가정값).
  74LVC125 같은 3.3 V 버퍼를 쓰면 더 빨라도 됩니다.

## 빌드와 업로드

```
.venv\Scripts\python.exe -m platformio run -d firmware/ground_bridge
.venv\Scripts\python.exe -m platformio run -d firmware/ground_bridge -e nano -t upload --upload-port COM7
```

* 구형 부트로더 Nano 호환품은 `-e nano_oldboot`, Uno는 `-e uno`.
* 빌드할 때마다 `gen_bridge_config.py`가 `params.json`에서 `src/bridge_config_gen.h`를 다시 만듭니다.
  주파수·SF·대역폭·코딩률·싱크워드·출력·프리앰블·시리얼 속도를 손으로 쓰지 마십시오. 기체와 항상 같아야 합니다.
* 확인만: `.venv\Scripts\python.exe firmware/ground_bridge/gen_bridge_config.py --check`

## 동작 확인

* 모듈이 응답하지 않으면 브리지는 1초마다 `BRIDGE: SX127x not found` 를 시리얼로 보냅니다.
  아두이노 시리얼 모니터(9600 baud)로 보면 됩니다. 지상국 프로그램은 이 글자를 잡음으로 버립니다.
* 지상국 실행: `.venv\Scripts\python.exe tools\ground_station.py --port COM7 --radio sx127x`
* 포트를 열면 Nano가 리셋됩니다. 약 2초 뒤부터 동작합니다.

## 검증 상태 (2026-10-13)

| 항목 | 결과 |
|---|---|
| 빌드 (nano, nano_oldboot, uno) | 성공. RAM 313 B / 2048 B, 플래시 4,326 B. 브리지 소스 `-Wall -Wextra` 경고 0 |
| simavr (ATmega328P, 모듈 없음) | 부팅 후 9600 baud로 `BRIDGE: SX127x not found` 반복 출력 확인 |
| 실제 SX1276 모듈과 무선 송수신 | **미검증** (하드웨어 없음) |

### v0.9 전원 텔레메트리 (fw09 RP1, 2026-10-29)

* 유선·하이브리드 빌드는 TELEM v4 / v5 (TYPE 0x23 / 0x24, 41 / 47 바이트) 로 답합니다. 끝의 2바이트가 전원 정보입니다
  (v_aux 0.125 V 단위, pwr_flags).
* 브리지는 이 프레임도 그대로 옮깁니다. 코드 변경은 없습니다. `LINK_LEN_MAX` 가 40 → 42, 버퍼가 45 → 47 바이트로 바뀌는데,
  이것은 `gen_bridge_config.py` 가 `sim/link_proto.py` 에서 다시 만든 헤더 값입니다.
* 빌드 (nano, nano_oldboot, uno): 성공. RAM 324 B (이전 322 B), 플래시 4,326 B. 실제 모듈 송수신은 여전히 미검증입니다.
