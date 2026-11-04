# 산정 보고서 (sizing) — preset `KAIST_90mm_12S`

- params.json sha256: `01ffc621423b9051…`
- 구조물 질량 출처: **CAD report: cadquery_report_KAIST_90mm_12S.json**
- EDF: JP Hobby 90mm 12-blade EDF, 12S, 4250-1080KV outrunner (JPH6007-10x family) — 공칭 추력 52.5 N / 피크 59.8 N, 12S 44.4 V
- 전장 941 mm, 핀 포함 외경 238 mm, 동체 OD 120 mm

## 판정 요약

| 검사 | 결과 | 값 |
|---|---|---|
| C12 추중비 (T/W @ thrust_nominal) | **WARN** | 1.215 (PASS ≥ 1.3, WARN ≥ 1.15) |
| C13 정적 안정성 (nose+fins Barrowman) | **WARN** | 정적 여유 0.55 cal, CG−hinge 395 mm (≥ 250) |
| 호버 가능 (u_h ≤ 1) | **OK** | u_h = 0.907 |
| 임무 상승 (1-D) 목표 도달 | **OK** | 정점 138.7 m |
| 낙하산 크기 | **OK** | 요구 D 1.66 m vs params 1.7 m |

## 1. 질량 예산 (mass budget)

| 항목 | 종류 | 그룹 | 질량 g | solid g | CoM x,y,z mm | 출처 | 비고 |
|---|---|---|---|---|---|---|---|
| `avionics_module` | printed | structure | 215.8 | 215.8 | 0.9, 0.0, 407.2 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 215.1 g |
| `avionics_tray` | printed | structure | 67.2 | 67.2 | 25.0, 0.0, 407.9 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 67.2 g |
| `gps_pod_lid` | printed | structure | 5.5 | 5.5 | -82.6, -17.6, 733.4 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 5.5 g |
| `intake_module` | printed | structure | 131.9 | 131.9 | 0.0, -0.0, 263.2 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 126.7 g |
| `latch_arm` | printed | structure | 0.3 | 0.3 | 65.7, 5.3, 742.0 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 0.3 g |
| `nose_cone` | printed | structure | 142.7 | 142.7 | 0.1, -0.0, 813.3 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 142.7 g |
| `payload_module` | printed | structure | 327.3 | 327.3 | -2.1, -0.5, 616.6 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 322.4 g |
| `piston` | printed | structure | 51.8 | 51.8 | -0.0, -0.0, 534.1 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 51.8 g |
| `propulsion_module` | printed | structure | 552.5 | 552.5 | -0.0, 0.0, 100.3 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 478.1 g |
| `vane_1` | printed | structure | 4.3 | 4.3 | 30.3, -0.0, 31.5 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 4.3 g |
| `vane_2` | printed | structure | 4.3 | 4.3 | 0.0, 30.3, 31.5 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 4.3 g |
| `vane_3` | printed | structure | 4.3 | 4.3 | -30.3, 0.0, 31.5 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 4.3 g |
| `vane_4` | printed | structure | 4.3 | 4.3 | -0.0, -30.3, 31.5 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 4.3 g |
| `vane_horn_1` | printed | structure | 0.4 | 0.4 | 57.0, -0.0, 40.9 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 0.4 g |
| `vane_horn_2` | printed | structure | 0.4 | 0.4 | 0.0, 57.0, 40.9 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 0.4 g |
| `vane_horn_3` | printed | structure | 0.4 | 0.4 | -57.0, 0.0, 40.9 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 0.4 g |
| `vane_horn_4` | printed | structure | 0.4 | 0.4 | -0.0, -57.0, 40.9 | CAD:cadquery | CAD report (cadquery_report_KAIST_90mm_12S.json); estimate was 0.4 g |
| `pushrod_1` | placeholder | actuation | 1.0 | - | 61.9, 11.9, 48.0 | CAD:cadquery | SPEC s6: 1 g; length 25.8 mm |
| `pushrod_2` | placeholder | actuation | 1.0 | - | -11.9, 61.9, 48.0 | CAD:cadquery | SPEC s6: 1 g; length 25.8 mm |
| `pushrod_3` | placeholder | actuation | 1.0 | - | -61.9, -11.9, 48.0 | CAD:cadquery | SPEC s6: 1 g; length 25.8 mm |
| `pushrod_4` | placeholder | actuation | 1.0 | - | 11.9, -61.9, 48.0 | CAD:cadquery | SPEC s6: 1 g; length 25.8 mm |
| `servo_arm_1` | placeholder | actuation | 1.0 | - | 60.5, 20.9, 49.0 | CAD:cadquery | SPEC s6: 1 g, negligible |
| `servo_arm_2` | placeholder | actuation | 1.0 | - | -20.9, 60.5, 49.0 | CAD:cadquery | SPEC s6: 1 g, negligible |
| `servo_arm_3` | placeholder | actuation | 1.0 | - | -60.5, -20.9, 49.0 | CAD:cadquery | SPEC s6: 1 g, negligible |
| `servo_arm_4` | placeholder | actuation | 1.0 | - | 20.9, -60.5, 49.0 | CAD:cadquery | SPEC s6: 1 g, negligible |
| `servo_release` | placeholder | actuation | 13.4 | - | 66.3, 14.0, 724.9 | CAD:cadquery | mass = servo.mass_g |
| `servo_vane_1` | placeholder | actuation | 13.4 | - | 51.9, 23.0, 32.4 | CAD:cadquery | mass = servo.mass_g |
| `servo_vane_2` | placeholder | actuation | 13.4 | - | -23.0, 51.9, 32.4 | CAD:cadquery | mass = servo.mass_g |
| `servo_vane_3` | placeholder | actuation | 13.4 | - | -51.9, -23.0, 32.4 | CAD:cadquery | mass = servo.mass_g |
| `servo_vane_4` | placeholder | actuation | 13.4 | - | 23.0, -51.9, 32.4 | CAD:cadquery | mass = servo.mass_g |
| `com` | placeholder | avionics | 20.0 | - | -0.0, 0.0, 504.5 | CAD:cadquery | mass = electronics.com.mass_g (ASSUMED in params; weigh it) |
| `esc` | placeholder | avionics | 150.0 | - | 36.5, -0.0, 380.0 | CAD:cadquery | mass = electronics.esc.mass_g |
| `fcu` | placeholder | avionics | 20.0 | - | -0.0, 0.0, 310.5 | CAD:cadquery | mass = electronics.fcu.mass_g (ASSUMED in params; weigh it) |
| `gns` | placeholder | avionics | 26.6 | - | -42.5, -0.0, 465.5 | CAD:cadquery | mass = electronics.gns.mass_g_ASSUMED (board + shifter + leads) + avionics_module.gns_mount.screws_g_ASSUMED (Layout.gns_mass_g; weigh it) |
| `gps` | placeholder | avionics | 17.0 | - | -84.1, -17.9, 721.2 | CAD:cadquery | mass = electronics.gps.board_mass_g (vendor 17 g with the antenna; weigh it), over the patch and board boxes by volume |
| `gps_ground_plane` | placeholder | avionics | 4.5 | - | -83.5, -17.7, 721.6 | CAD:cadquery | mass = payload_module.gps_pod.ground_plane.mass_g_ASSUMED |
| `gps_lead` | lumped | avionics | 4.7 | - | -71.7, -13.8, 633.0 | params | v0.8: GPS lead along Layout.gps_lead_polyline + slack, connector pair, screws / standoffs / tape (payload_module.gps_pod.lead, ASSUMED) at their mass centroid |
| `misc_electronics` | placeholder | avionics | 100.0 | - | 13.5, 0.0, 466.0 | CAD:cadquery | mass = electronics.misc.mass_g |
| `nav` | placeholder | avionics | 45.0 | - | 34.0, -0.0, 465.0 | CAD:cadquery | mass = electronics.nav.mass_g |
| `radio_log` | placeholder | avionics | 20.0 | - | -0.0, -40.0, 507.5 | CAD:cadquery | mass = electronics.radio_log.mass_g (ASSUMED in params; weigh it) |
| `spring` | placeholder | ejection | 10.0 | - | 0.0, -0.0, 525.0 | CAD:cadquery | mass = 0.5 x hardware_mass_g.spring_piston_hardware |
| `joint_screws_inserts` | lumped | hardware | 40.0 | - | 0.0, 0.0, 441.5 | params | SPEC s6: point mass at the mean of the four joint Z |
| `pushrods_horns_clevises` | lumped | hardware | 10.0 | - | 0.0, 0.0, 37.0 | params | SPEC s6: point at vane_hinge_z |
| `spring_piston_hardware_balance` | lumped | hardware | 10.0 | - | 0.0, 0.0, 534.0 | params | NOT in the SPEC s6 table: the other half of hardware_mass_g.spring_piston_hardware, lumped at the piston |
| `cansat_payload` | placeholder | payload | 1000.0 | - | -0.0, -0.0, 623.5 | CAD:cadquery | mass = payload.mass_g (competition max) |
| `battery` | placeholder | power | 680.0 | - | -1.5, -0.0, 384.0 | CAD:cadquery | mass = battery.mass_g |
| `edf_unit` | placeholder | propulsion | 450.0 | - | 0.0, -0.0, 164.2 | CAD:cadquery | mass = edf.mass_g (vendor) |
| `chute_pack` | placeholder | recovery | 210.0 | - | -0.0, -0.0, 800.0 | CAD:cadquery | chute + lines/swivel + shock cord |

| 합계 | 값 |
|---|---|
| **전체 (full stack)** | **4406.8 g** = 43.22 N |
| 출력 구조물 (as printed / fully solid) | 1514.0 g / 1514.0 g |
| 비출력 품목 | 2892.8 g |
| 그룹 `actuation` | 75.0 g |
| 그룹 `avionics` | 407.8 g |
| 그룹 `ejection` | 10.0 g |
| 그룹 `hardware` | 60.0 g |
| 그룹 `payload` | 1000.0 g |
| 그룹 `power` | 680.0 g |
| 그룹 `propulsion` | 450.0 g |
| 그룹 `recovery` | 210.0 g |
| 그룹 `structure` | 1514.0 g |
| **사출 후 (post-ejection: −cansat, −nose_cone, −chute_pack)** | **3054.1 g** |
| 하강 질량 descent_mass_g (post-ejection + chute_pack) | 3264.1 g |
| 하강 질량 (노즈콘 테더 포함) | 3406.8 g |
| CG full (x, y, z) | 1.3, -0.3, 432.2 mm |
| CG post-ejection (x, y, z) | 1.9, -0.4, 326.5 mm |
| I_xx / I_yy / I_zz (full, about CG) | 0.2292 / 0.2297 / 0.0073 kg·m² |
| I_xx / I_yy / I_zz (post-ejection) | 0.1056 / 0.1061 / 0.0059 kg·m² |
| 발사대 `launch_pad` (지상 장비, 비행 질량 제외) | 196.6 g (solid 356.8 g) |

ESTIMATE 열: 출력 구조물은 해석식(튜브 = 면적 × 벽 × 밀도, 콘/노즈 = 회전 쉘 수치적분, 핀 = 평면형 × t 등)으로 추정. 'as printed'는 벽 5 루프 × 0.42 mm + 15 % 인필 모델, 'solid'는 CAD 질량특성 보고서가 보여줄 완전 충전 값. CAD 보고서가 생기면 자동으로 그 값을 사용한다.

## 2. 추력 / 추중비

| 항목 | 값 |
|---|---|
| 중량 W | 43.22 N (4407 g) |
| T/W @ thrust_nominal | 1.215 → C12 **WARN** |
| T/W @ thrust_peak | 1.384 |
| 호버 스로틀 u_h = (W/T_max)^(1/2) | 0.907 → P 3383 W, I 76.2 A |
| T/W @ climb_throttle 1.0 | 1.215 |
| T/W = 1.30 에 필요한 스로틀 | 1.034 |
| T/W = 1.30 허용 최대 질량 | 4118 g |

## 3. 배기 유속 · 베인 제어권한 (SPEC §8.3–8.4)

| 조건 | T N | A_e m² | V_e m/s | q_e Pa | ṁ kg/s |
|---|---|---|---|---|---|
| at_nominal_thrust | 52.5 | 0.00507 | 91.9 | 5174 | 0.571 |
| at_hover_thrust | 43.2 | 0.00507 | 83.4 | 4259 | 0.518 |
| at_climb_throttle | 52.5 | 0.00507 | 91.9 | 5174 | 0.571 |

| 조건 | F/베인 @ 20° N | M_pitch N·m | M_roll N·m | α_pitch rad/s² | α_roll rad/s² | M_pitch per deg |
|---|---|---|---|---|---|---|
| at_nominal_thrust | 4.16 | 3.289 | 0.466 | 14.35 | 64.09 | 0.1645 |
| at_hover_thrust | 3.43 | 2.707 | 0.384 | 11.81 | 52.76 | 0.1354 |

- 레버암 l_v = z_CG − vane_hinge_z = 395.2 mm; S_v = 960 mm²; C_Lα = 2.4 /rad (stall 25.0°); r_v = 28.0 mm
- 관성 (full, CG 기준): I_xx 0.2292, I_yy 0.2297, I_zz 0.0073 kg·m² (점질량+봉/상자 근사)
- CG 횡방향 오프셋 1.3 mm (배터리 −x / ESC·NAV +x 배치) → 트림 토크 ≈ W × 오프셋 = 0.057 N·m

### 롤 권한 vs 잔류 스월 토크

| 항목 | 값 |
|---|---|
| ω (kv × V, 무부하) | 47952 rpm = 5022 rad/s |
| P/ω @ power_nominal | 0.902 N·m |
| τ_swirl = 0.15 × P/ω (ASSUMED) | 0.135 N·m |
| M_roll,max @ hover / @ nominal | 0.384 / 0.466 N·m |
| 권한/스월 비 @ hover / @ nominal | 2.84 / 3.44 |
| 스월 트림 δ_z | 7.05° = 35 % of δ_cmd,max (mixer roll_priority_scale 0.5) |

swirl ~ u^2 and q_e ~ u^2 -> the trim deflection is roughly throttle-independent; omega from kv x V is the unloaded speed, so tau_swirl is a lower bound.

## 4. 임무 에너지 (1-D 수직 상승)

모델: 1-D vertical, thrust T_max u^n with first-order lag, ramp, drag 0.5 rho v^2 Cd A_ref, cut-off h + k v^2/2g; A_ref = 0.01131 m², Cd 0.6, 램프 0.5 s, 추력 시정수 0.15 s, coast_drag_factor 1.2, 배터리 2200 mAh × 44.4 V = 97.7 Wh, C-rating 75C.

| 시나리오 | T/W@u | 도달 | t_cut s | h_cut m | v_cut m/s | 정점 m | 오버슈트 m | t_apogee s | I_peak A | C-rate | Wh 사용 | 잔량 % | geofence |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| target_140m @ u=1.00 (sensitivity) | 1.21 | yes | 11.4 | 113.6 | 20.8 | 138.7 | -1.3 | 13.6 | 102.0 | 46.4C | 13.81 | 85.9 | ok |
| competition 체험부 175m | 1.21 | yes | 12.7 | 142.8 | 23.0 | 172.8 | -2.2 | 15.2 | 102.0 | 46.4C | 15.49 | 84.1 | ⚠ geofence |
| competition 슬기부 250m | 1.21 | yes | 15.2 | 206.0 | 26.8 | 245.9 | -4.1 | 18.1 | 102.0 | 46.4C | 18.68 | 80.9 | ⚠ geofence |
| competition 창작부 350m | 1.21 | yes | 18.2 | 292.1 | 30.8 | 343.3 | -6.7 | 21.4 | 102.0 | 46.4C | 22.44 | 77.0 | ⚠ geofence |

지오펜스 geofence_altitude_m = 160.0 m: 이를 넘는 시나리오는 §8.8 상태기계에서 ABORT 된다.

## 5. 회수 (낙하산)

| 하강 질량 정의 | 질량 g | 요구 면적 m² @ 5.5 m/s | 요구 D m | params D m | params 낙하산 하강속도 m/s | 착지 에너지 J |
|---|---|---|---|---|---|---|
| post_ejection | 3054 | 2.155 | 1.66 | 1.70 | 5.36 | 43.9 |
| post_ejection_plus_chute_pack (SPEC descent_mass_g) | 3264 | 2.304 | 1.71 | 1.70 | 5.54 | 50.1 |
| post_ejection_plus_nose_and_chute_pack (tethered nose) | 3407 | 2.404 | 1.75 | 1.70 | 5.66 | 54.6 |

Cd = 0.75, ρ = 1.225 kg/m³. 캔위성은 자체 낙하산(별도).

## 6. 안정성 (CG vs Barrowman CP)

| 성분 | C_Nα /rad | x from tip mm | z mm |
|---|---|---|---|
| nose | 2.000 | 95.0 | 846.0 |
| fins | 3.047 | 890.2 | 50.8 |
| boat_tail_cone | -0.666 | 794.1 | 146.9 |

| 항목 | 값 |
|---|---|
| 핀 노출 평면형 (root c / tip c / s / LE sweep) | 89.3 / 40.0 / 70.0 / 49.3 mm, 루트 반경 49.0 mm, 기준 직경 120 mm |
| CP (nose + fins) | z = 365.9 mm, C_Nα 합 5.047 /rad |
| CG full / post-ejection | z = 432.2 / 326.5 mm |
| 정적 여유 full (coast 형상) | **0.55 cal** → C13 **WARN** (PASS ≥ 1.0, WARN ≥ 0.5) |
| 정적 여유 post-ejection (낙하산 하강 중 — 참고용) | -0.33 cal |
| 감도: 보트테일(콘 120→98) 포함 | CP z = 399.2 mm, 여유 0.27 cal |
| CG − vane_hinge_z (= l_v) | 395.2 mm (C13 요구 ≥ 250) |

## 7. 국내 법규 분류 (항공안전법, 질량 기준)

| 항목 | 값 |
|---|---|
| 최대이륙중량 | 4.407 kg |
| 조종자 증명 | **3종 (2 kg 초과 ~ 7 kg 이하: 필기시험)** |
| 기체신고 (> 2 kg) | **필요** |
| 비행승인 (> 150 m) @ 목표 고도 140 m ★ | 불필요 (고도 기준; 관제권/비행금지구역은 별도) |
| 비행승인 @ 예측 정점 138.7 m | 불필요 |
| 경연 사출 고도 (m) | 체험부 175, 슬기부 250, 창작부 350 → 비행승인 필요: 체험부, 슬기부, 창작부 |

> 항공안전법: 최대이륙중량 2 kg 초과 -> 기체신고 및 3종 이상 조종자 증명(2~7 kg); 고도 150 m 초과 또는 관제권/비행금지구역 -> 비행승인 필요. 최종 확인은 드론원스톱(drone.onestop.go.kr).

## 8. 경고 · 가정

- ⚠ CAD report total 4396.8 g differs from the budget sum 4406.8 g (items missing from one side, or the 10 g hardware balance)
- ⚠ C12 WARN: T/W at thrust_nominal = 1.215 (PASS >= 1.3, WARN >= 1.15); mass 4407 g vs 4118 g allowed for 1.30
- ⚠ scenario competition 체험부 175m: altitude exceeds geofence_altitude_m 160 m -> ABORT by the onboard geofence interlock I5 (SPEC s8.8)
- ⚠ scenario competition 슬기부 250m: altitude exceeds geofence_altitude_m 160 m -> ABORT by the onboard geofence interlock I5 (SPEC s8.8)
- ⚠ scenario competition 창작부 350m: altitude exceeds geofence_altitude_m 160 m -> ABORT by the onboard geofence interlock I5 (SPEC s8.8)
- ⚠ C13 WARN: static margin 0.55 cal, CG above hinge 395 mm

- `_ASSUMED` 키 (실측 전 신뢰 금지): design_load_N_ASSUMED, electrical_plant_ASSUMED, esc_options_ASSUMED, gear_backlash_deg_ASSUMED, hardware_g_ASSUMED, hub_d_ASSUMED, hybrid_extra_ASSUMED, lead_exit_ASSUMED, len_lip_top_to_housing_bottom_ASSUMED, lip_t_ASSUMED, mass_g_per_m_ASSUMED, mass_g_per_m_ASSUMED, od_ASSUMED, print_tolerance_ASSUMED, residual_swirl_torque_fraction_ASSUMED, ribbon_t_ASSUMED, ribbon_w_ASSUMED, stock_len_in_edf_mass_ASSUMED, tether_kit_ASSUMED, tether_plant_ASSUMED, vtol_crossflow_cd_ASSUMED
- 푸시로드 길이 (arm tip → horn pin, 베인 1): 25.80 mm — CAD 보고서 `pushrod_len_mm` 와 대조
- 노즈 캐비티 0.90 L (요구 0.7 L), 흡입 면적비 3.29 (최소 개구, v0.6.2 창 아래 거싯 포함; 바깥면 3.41) (요구 ≥ 2.0)
- 추력·전력은 벤더 수치 (SPEC §12: 비행 전 벤치 추력 시험 필수); 배터리 전압 강하·ESC 효율은 모델에 없음.
