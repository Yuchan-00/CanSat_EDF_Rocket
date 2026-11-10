# 예산안 - VTOL 구성 (v0.9 CAD-2)

로켓 예산(`예산안.md`)의 조사 가격을 그대로 쓰고, VTOL에서 빠지는 줄을 빼고 새 줄을 더했습니다. (ASSUMED) 표시는 조사하지 않은 추정가입니다.

**배터리 VTOL 합계: 408,903원**

기체 질량 (CadQuery 트윈, 출력 무게는 슬라이서 값): battery 1653.7 g (T/W 1.87 @ 미검증 공칭 30.3 N), tether 1405.8 g (T/W 2.20 @ 미검증 공칭 30.3 N), hybrid 1526.8 g (T/W 2.02 @ 미검증 공칭 30.3 N)

| 분류 | 품목 | 가격 | 배송 | 비고 | 판매처 |
|---|---|---|---|---|---|
| 추진 | EDF Hobbyocean 90mm 12엽 6S 1450KV | 83,400 | 0 | 추력표 없음 - Powerfun 90mm 6S 1450KV 표(3.09 kg @ 22.2 V / 102 A)를 대용으로 사용 | [링크](https://ko.aliexpress.com/item/1005007300026404.html) |
| 추진 | ESC Hobbywing SkyWalker 100A V2 (5 V/7 A BEC) | 51,300 | 0 | 100 A 연속 / 120 A 순간. BEC로 서보 5개 구동 | [링크](https://ko.aliexpress.com/item/1005012523818954.html) |
| 추진 | LiPo CNHL G+Plus 6S 2200 mAh 70C XT60 | 40,207 | 9,372 | 349 g, 109 x 35 x 51 mm. 해외 배송 15-20 영업일 | [링크](https://chinahobbyline.com/products/cnhl-gplus-series-2200mah-22-2v-6s-70c-lipo-battery-with-xt60-plug) |
| 구동 | MG90S 서보 5개 세트 | 13,932 | 0 | 베인 4 + 노즈 분리 1. v0.9: 베인 서보는 핀 뿌리 페어링 안에서 베인 축을 직접 돌린다 (동봉 2암 혼을 8.8 mm로 잘라 인쇄 커플러의 홈에 끼움; 푸시로드·클레비스·암 없음) | [링크](https://ko.aliexpress.com/item/1005008490187744.html) |
| 비행 컴퓨터 | MPU-6050 GY-521 | 2,572 | 0 |  | [링크](https://ko.aliexpress.com/item/1005010057794277.html) |
| 비행 컴퓨터 | BMP280 기압계 모듈 [TYE-BMP29] | 2,750 | 0 | I2C 0x77, NAV 기본 기압계 드라이버 (flight_computer.nav.baro_sensor). 디바이스마트 판매가(부가세 포함), 2026-11-05 조회·2026-11-11 다시 확인. 배송비는 M3 볼트 키트 줄에 1회 (같은 주문 가정 - ASSUMED). v0.9까지의 MS5611 GY-63(4,780원 + 배송 1,973원, 2026-10-13 조사)은 펌웨어 빌드 옵션(nav_ms5611)으로만 남고 사지 않음 | [링크](https://www.devicemart.co.kr/goods/view?no=15442588) |
| 비행 컴퓨터 | OpenLog 호환 로거 | 3,980 | 0 | microSD 카드는 보유 가정 | [링크](https://ko.aliexpress.com/item/1005006370090065.html) |
| 항법 | GPS 모듈 NEO-7M [SZH-NT07] (u-blox NEO-7M) | 9,460 | 0 | 디바이스마트 판매가(부가세 포함), 2026-11-05 조회·2026-11-11 다시 확인. 배송비는 M3 볼트 키트 줄에 1회 (같은 주문 가정 - ASSUMED) (부가세 별도로 환산하면 8,600원). v0.8-v0.9 예산은 GY-NEO6MV2(NEO-6M, eleparts 8,000원 부가세 별도)였음. 보드·안테나 치수와 무게는 다시 재지 않음 (ASSUMED: GY-NEO6MV2와 같은 급). v0.8.1: GNS 보드 USART1 (PD2/PD3) 38400 bps, 레벨 변환은 GNS 보드의 4채널 BSS138 시프터 (아래 줄). 업그레이드: NEO-M8N (정품 모듈 약 55,364원 부가세 별도) | [링크](https://www.devicemart.co.kr/goods/view?no=10895400) |
| 통신 | ESP-12E Wi-Fi 모듈 (ESP8266, 기체 무선) | 2,640 | 0 | 디바이스마트 판매가(부가세 포함), 2026-11-05 조회·2026-11-11 다시 확인. 배송비는 M3 볼트 키트 줄에 1회 (같은 주문 가정 - ASSUMED). 기체 무선 기본값 (flight_computer.link.vehicle_radio = esp_bridge, firmware/esp_bridge). 로켓은 통신·로거 상자에 손배선 (firmware/README.md 0.10.4 - 거리 시험과 상자 맞춤은 사용자 결정 대기), VTOL은 캐리어 rev 2c의 U7 자리 (JLC 조립이면 LCSC C89297, 손납땜이면 이 모듈). 부팅 핀 저항·커패시터는 '분압 저항·커패시터' 줄 몫 (ASSUMED). 지상 쪽 ESP는 아래 지상 표 (합계에 미포함) | [링크](https://www.devicemart.co.kr/goods/view?no=1378118) |
| 비행 컴퓨터 | 분압 저항·커패시터 (ASSUMED) | 1,000 | 0 | 추정값 | - |
| 배선 | XT60H 커넥터 + 12 AWG 실리콘 전선 1 m x2 | 7,800 | 3,500 | RCbank, 한 번에 주문 시 배송비 1회 | [링크](https://www.rcbank.co.kr/shop/goods/goods_view.php?goodsno=4825) |
| 배선 | 베인 서보 연장선: 26 AWG 3심 서보선 3 m + JR 3핀 하우징 4세트 + 열수축 (ASSUMED) | 7,000 | 0 | 추정값 (로켓 예산과 같은 단가) | - |
| 배선 | 5.5 mm 총알(불릿) 커넥터 3쌍 + 열수축 튜브 (ASSUMED) | 5,000 | 0 | 추정값 - 가격 조사 안 함. v0.9: 모터 선 3가닥은 총알 커넥터를 단 채 추진 모듈 슬롯 -> 흡입 모듈의 열린 홈(폭 22 mm, 인쇄 덮개) -> 격벽 구멍을 지나 항공전자 칸에서 ESC 선과 잇는다 (커넥터 외경 8 mm ASSUMED가 한 줄로 지나감, C29). 모터 쪽 3개 + ESC 쪽 3개 | - |
| 배선 | 모터 연장선: 12 AWG 실리콘 전선 2 m (ASSUMED 필요) | 6,000 | 0 | 단가는 위 XT60H 줄의 같은 판매처 12 AWG 1 m 가격 (3,000원/m, 같은 주문이면 배송비 없음). cad/layout.py 모터 선 경로 349 mm x 3가닥 -> 2 m. EDF 기본 선이 항공전자 칸까지 닿으면 필요 없음 (기본 선 길이 미확인 - ASSUMED) | [링크](https://www.rcbank.co.kr/shop/goods/goods_view.php?goodsno=4825) |
| 구조 | M2 나사: M2 x 8 셀프태핑 12개 (ASSUMED) | 3,000 | 0 | 추정값 - 가격 조사 안 함. v0.9 직결 베인: 서보 덮개 4개 x 2 + 모터 선 홈 덮개 4개 (셀프태핑) | - |
| 구조 | 베인 고정 부품: M2 x 3 무두볼트(컵 포인트) 4개 + 0.9 mm 육각 렌치, 황동 파이프 6 x 0.45 mm 30 cm (부싱 4개), EVA 폼 3 mm, 케이블 타이 2.5 mm 3개 이상 (ASSUMED) | 6,000 | 0 | 추정값 - 가격 조사 안 함. v0.9 수정: 커플러를 베인 축 D컷에 무두볼트로 고정(축 빠짐·D컷 유격 제거), 힌지 보스에 황동 부싱(유격 0.1 mm), 서보와 덮개 사이 폼(흔들림 제거), 흡입부 지주의 모터 선 고정점에 케이블 타이 | - |
| 구조 | M3 볼트·너트 키트 340개 | 9,680 | 2,700 | 5-20 mm. 모듈 결합 (EDF 탭 고정 나사는 2026-11 키웨이 시계 장착으로 없어짐) | [링크](https://www.devicemart.co.kr/goods/view?no=1385067) |
| 비행 컴퓨터 | VTOL 캐리어 PCB 제작 (rev 2c, 4층, 5장) (ASSUMED) | 15,000 | 0 | 추정값 - 견적 안 받음 (v0.9 CAD-2의 2층 기판 추정값 그대로; 4층은 더 비쌀 수 있음). ATmega128A 4개(NAV·FCU·COM·GNS; 5번째 칩 자리 없음), ESP-12E 납땜 자리, BMP280·GY-521·OpenLog·Pololu 5 V 모듈 헤더, 레벨 시프터·레귤레이터 온보드 (pcb/vtol_carrier). 흡입 격벽 위 FCU 자리에 폼 테이프 + 케이블 타이 | - |
| 비행 컴퓨터 | 캐리어 부품: ATmega128A TQFP-64 x4, 16 MHz 크리스털 x4, AMS1117, BSS138, 수동 소자, 헤더·커넥터 (ASSUMED) | 45,000 | 0 | 추정값 - 가격 조사 안 함 (v0.9 CAD-2 추정값 그대로, rev 2c 부품표로 다시 계산하지 않음). 기존 보드 4개·레벨 시프터 3개·AMS1117 3개·하네스 3개를 대신함. ESP-12E·BMP280·GY-521·OpenLog·GPS는 각자의 줄 | - |
| 비행 컴퓨터 | 로직 전원 1S 리튬폴리머 3.7 V 4200 mAh [DTP105565P] | 18,150 | 0 | 디바이스마트 판매가(부가세 포함), 2026-11-05 조회·2026-11-11 다시 확인. 배송비는 M3 볼트 키트 줄에 1회 (같은 주문 가정 - ASSUMED). 캐리어 J14. 6S 팩과 분리된 로직 전원 (configs.vtol.logic_power). 충전기 1S 설정 필요 | [링크](https://www.devicemart.co.kr/goods/view?no=16021596) |
| 비행 컴퓨터 | Pololu 5 V 승강압 모듈 S9V11F5S6CMA (#2870, 저전압 차단 조정) | 19,360 | 0 | 디바이스마트 판매가(부가세 포함), 2026-11-05 조회·2026-11-11 다시 확인. 배송비는 M3 볼트 키트 줄에 1회 (같은 주문 가정 - ASSUMED). 캐리어 J6. 1S 팩 -> 5 V 로직. 로켓 예산의 5 V UBEC를 대신함 | [링크](https://www.devicemart.co.kr/goods/view?no=15855364) |
| 비행 컴퓨터 | 로직 전원 토글 스위치 MTS-102 (2단 3P, 듀폰 하네스) | 1,100 | 0 | 디바이스마트 판매가(부가세 포함), 2026-11-05 조회·2026-11-11 다시 확인. 배송비는 M3 볼트 키트 줄에 1회 (같은 주문 가정 - ASSUMED). 캐리어 J15 | [링크](https://www.devicemart.co.kr/goods/view?no=13029107) |
| 항법 | GPS 캡 부품: FR4 동박 40 mm 접지판, JST-GH 1쌍, 28 AWG 4심 0.4 m, 폼 테이프 (ASSUMED) | 3,000 | 0 | 추정값. GPS는 탑캡 위에 테이프로 고정 | - |
| 구조 | Bambu PLA Matte 아이보리 화이트(흰색) (Ivory White, 코드 11100) 리필 1 kg x1 (VTOL 기체 출력 부품 + 2색 부품의 퍼지·타워 몫: 슬라이스 781 g + 10 %) | 18,000 | 0 | 가격: 공식 스토어 PLA Matte 리필 18,000원/kg (2026-10-13 조사), 색마다 같은 값으로 가정(ASSUMED). 구매할 때 색 이름·코드를 확인하세요. 7만원 이상 무료배송. 슬라이스 출처: 임시 VTOL 출력 계획 (CadQuery 트윈 STL 슬라이스) | [링크](https://kr.store.bambulab.com/products/pla-matte-filament) |
| 구조 | Bambu PLA Matte 만다린 오렌지(주황) (Mandarin Orange, 코드 11300) 리필 1 kg x1 (VTOL YUCHAN 로고 상감 + 퍼지·프라임 타워 몫: 슬라이스 76 g + 10 %) | 18,000 | 0 | 가격: 공식 스토어 PLA Matte 리필 18,000원/kg (2026-10-13 조사), 색마다 같은 값으로 가정(ASSUMED). 구매할 때 색 이름·코드를 확인하세요. 7만원 이상 무료배송. 슬라이스 출처: 임시 VTOL 출력 계획 (CadQuery 트윈 STL 슬라이스) | [링크](https://kr.store.bambulab.com/products/pla-matte-filament) |

## 로켓 예산에서 빠진 줄

| 분류 | 품목 | 가격 + 배송 |
|---|---|---|
| 비행 컴퓨터 | ATmega128 최소 시스템 보드 x4 (NAV + FCU + COM + GNS) | 42,752 |
| 비행 컴퓨터 | I2C 레벨 시프터 4채널 | 1,761 |
| 비행 컴퓨터 | AMS1117-3.3 모듈 x3 (NAV 센서용 + 무선·로거 상자용 + GNS GPS 레벨 시프터용) | 4,203 |
| 비행 컴퓨터 | 5 V 3 A UBEC (2-6S) 로직 전원 | 4,196 |
| 항법 | GPS 포드 부품: 단면 FR4 동박 기판 50 x 50 x 0.8 mm (접지판), JST-GH 4핀 커넥터 1쌍, 28 AWG 4심 선 0.5 m, M2 나사 5개 + 나일론 M2 스페이서 4개, 고무 그로밋 (ASSUMED) | 4,000 |
| 항법 | I2C 레벨 시프터 4채널 (GNS 보드의 GPS 신호선용 세 번째) | 1,761 |
| 항법 | glink 배선: 4핀 커넥터 1쌍 + 신호선 30 cm (ASSUMED) | 1,500 |
| 통신 | LoRa 모듈 2개 (예비 링크: 기체용, 지상용) | 0 |
| 통신 | I2C 레벨 시프터 4채널 (무선 모듈 신호선용 두 번째) | 1,761 |
| 비행 컴퓨터 | ilink 배선: 1 kΩ 직렬 저항 + 4핀 커넥터 1쌍 + 신호선 30 cm (ASSUMED) | 1,500 |
| 비행 컴퓨터 | nlink 배선: 5핀 커넥터 1쌍 + 신호선 30 cm (ASSUMED) | 1,500 |
| 회수 | 립스톱 나일론 1 x 1.5 m x2 (1.4 m 낙하산 자작) | 6,302 |
| 회수 | 낙하산 줄·쇼크 코드·스위블 (ASSUMED) | 3,000 |
| 회수 | 사출 스프링 OD 40 (ASSUMED) | 3,000 |

## 유선(배터리 없음) / 하이브리드 변형 - 기체 쪽 추가 (배터리 줄은 그대로 둠: 시험용)

| 품목 | 가격 | 출처 |
|---|---|---|
| 기체 쪽 유선 키트: 커패시터·TVS·브레이크 초퍼·하드포인트(아이볼트·보강 와셔·케이블 부트) | 30,000 | 유선전원 검토 costs_krw (ASSUMED) |
| XT90-S 1쌍 (기체-테더) | 5,000 | 유선전원 검토 (ASSUMED, 2쌍 가격의 절반) |
| D1/D2 이상적 다이오드 보드 (LM74700-Q1 x2 + FET 6개 + PCB) | 50,000 | 유선전원 검토 (ASSUMED) |
| 버퍼 배터리 6S 650 mAh 120C (GNB) | 45,000 | 유선전원 검토 (ASSUMED) |

유선 변형 기체 쪽 35,000원, 하이브리드 기체 쪽 130,000원.

## 지상 장비 묶음 (유선전원 검토 costs_krw.kits, 합계에 미포함)

| 묶음 | 가격 | 내용 |
|---|---|---|
| i_thrust_test | 60,207 | watt meter + optional 2nd 6S 2200 |
| ii_battery_less_ground_lipo | 340,207 | 2nd pack, cable, XT90-S, fuse/E-stop, gantry, Dyneema/bungee, vehicle kit |
| iii_hybrid_RSP | 1,027,265 | RSP-1500-24 + ii without the 2nd pack + buffer 650 + D1/D2 board |
| iii_hybrid_SE_estimate | 663,073 | as above with SE-1500-24 (excl. import) |

전원 장치(RSP-1500-24)는 추력 시험 뒤에 삽니다 (유선전원 검토 권고).
