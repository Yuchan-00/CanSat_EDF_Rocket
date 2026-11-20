# 사용자 실측 (시뮬레이션 단계가 참고할 것)

- 2026-11-18 03시대, 사용자: "모터를 최대 출력으로 두었을 떄 1분 30초까지는 안정적으로 출력이 유지되었어. 참고해줘."
  - 해석 (M, 사용자 실측): EDF를 최대 출력으로 90 s 동안 돌려도 출력이 떨어지지 않았다. 추력 값·전압·온도·배터리는 아직 받지 않음.
- 2026-11-18 03시대, 사용자 추가: "추력은 정확하지 않지만 pwm 기준 80% 지점에서 회전 토크 발생, 93% 지점에서 상승 및 호버링 시작이었어. 테스트에 사용한 로켓의 질량은 1700g수준이었고"
  - M (사용자): 시험 기체 약 1,700 g(VTOL 구성으로 보임; 설계 VTOL 1,657.5 g), PWM 93 %에서 이륙·호버 시작 → 추력(93 %) ≈ 무게 = 1.70 × 9.81 ≈ 16.7 N (지면 효과로 조금 과대평가될 수 있음). PWM 80 %에서 기체가 반작용 토크로 돌기 시작.
  - E: 추력 ∝ (스로틀)^2 로 보면 최대 추력 ≈ 16.7 / 0.93^2 ≈ 19.3 N (≈ 1.97 kgf); 선형이면 ≈ 18.0 N. 설계 모델 edf.thrust_nominal_N = 30.30 N (판매처 표 3.09 kgf, 미검증 PROXY) 보다 약 36-40 % 낮음. PWM % 와 ESC 스로틀의 대응(ESC 캘리브레이션 범위)은 모름(A).
  - 영향 (E): 로켓 구성 2,605 g 의 무게 25.6 N > 추정 최대 추력 18-19.3 N → 이 추력이면 로켓은 이륙하지 못함. VTOL은 최대 추력 대 무게 약 1.1-1.2 (설계 가정보다 훨씬 작음) → 제어 여유 부족, 설계 호버 스로틀(약 0.70) 대신 실제 약 0.93.
  - 사용자 추가: "참고로 vane은 추가하지않았고 pcb, 서보모터의 무게에 맞는 무게추를 추가하여 순수 EDF 모터의 추력만 확인한 내용이야." → 날개 없음, PCB·서보 무게는 무게추로 대신함 = EDF 단독 추력 시험. 따라서 16.7 N(93 %)은 날개 손실이 없는 값이고, 날개를 달면 추력은 조금 더 줄어들 수 있음(E). PWM 80 %의 회전은 날개가 없어서 팬 반작용 토크를 막을 것이 없었던 것으로, 제어 결함 증거가 아님.
  - 사용자 추가 (2026-11-18 05시대): "Cansat을 제외하면 실제 무게는 1800g수준이야" → 실제 로켓(CanSat 미탑재) 질량 약 1,800 g (M, 사용자). CAD 보고서의 CanSat 제외 질량(약 2,255 g = 2,605 - CanSat 350)보다 약 450 g 가벼움 → CAD의 가정 질량(ASSUMED)이 과대.
  - **REPORT STAGE INSTRUCTION**: for the two rocket cases use the MEASURED masses as the primary case: (b) without CanSat = 1,800 g (M, user), (a) with CanSat = 1,800 + CanSat mass from params (≈ 350 g → ≈ 2,150 g, E). Keep the CAD masses (2,605 / ≈2,255 g) as a secondary column and say the CAD over-estimates by ≈450 g (list which CAD masses are ASSUMED). CG / inertia for the measured mass are unknown: scale the CAD inertia by the mass ratio and keep the CAD CG (A), and show how the stability margin moves if the missing 450 g sat in the nose or in the tail (bounding cases). Thrust: show both the design 30.3 N and the measured-estimate 18-19.3 N; with 1,800 g the measured-estimate T/W is only ≈1.02-1.09 (E) and with the CanSat ≈0.85-0.92 (cannot lift) - compute it in the tools, don't just quote this.
  - 할 일: 이번 시뮬 결과(설계 추력 30.3 N 기준)와 별도로 실측 기반 추력(18-19.3 N)으로 다시 계산해 비교할 것. 추력대 측정(저울)과 ESC 캘리브레이션 확인을 사용자에게 요청.
  - 의미: 로켓 비행의 모터 가동 시간(약 13-14 s, CUT까지)과 VTOL의 비행 시간 규칙(이륙 뒤 30 s에 착륙 시작, 60 s 하드 컷)은 모두 이 90 s 안에 들어감. 추력 곡선(.eng)에서 시간에 따른 추력 감소(배터리 처짐)를 가정했다면 90 s 안에서는 실측과 맞지 않을 수 있으니 보고서에 비교해 적을 것.
