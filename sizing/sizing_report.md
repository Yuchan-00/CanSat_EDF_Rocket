# 산정 보고서 (sizing) — preset `BUDGET_90mm_6S`

- params.json sha256: `01ffc621423b9051…`
- 구조물 질량 출처: **CAD report: cadquery_report.json**
- EDF: Hobbyocean 90mm 12-blade EDF, 6S 1450KV (AliExpress 1005007300026404) — 공칭 추력 30.302548499999997 N / 피크 37.1672035 N, 6S 22.2 V
- 전장 856 mm, 핀 포함 외경 247 mm, 동체 OD 120 mm

## 판정 요약

| 검사 | 결과 | 값 |
|---|---|---|
| C12 추중비 (T/W @ thrust_nominal) | **WARN** | 1.184 (PASS ≥ 1.3, WARN ≥ 1.15) |
| C13 정적 안정성 (nose+fins Barrowman) | **WARN** | 정적 여유 0.60 cal, CG−hinge 350 mm (≥ 250) |
| 호버 가능 (u_h ≤ 1) | **OK** | u_h = 0.919 |
| 임무 상승 (1-D) 목표 도달 | **OK** | 정점 138.9 m |
| 낙하산 크기 | **OK** | 요구 D 1.37 m vs params 1.4 m |

## 1. 질량 예산 (mass budget)

| 항목 | 종류 | 그룹 | 질량 g | solid g | CoM x,y,z mm | 출처 | 비고 |
|---|---|---|---|---|---|---|---|
| `avionics_logo` | printed | structure | 0.9 | 0.9 | 23.3, 0.4, 408.3 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 0.9 g |
| `avionics_module` | printed | structure | 123.9 | 123.9 | 1.1, 0.0, 434.9 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 126.6 g |
| `avionics_tray` | printed | structure | 36.4 | 36.4 | 25.0, -0.0, 432.7 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 40.9 g |
| `gps_pod_lid` | printed | structure | 5.6 | 5.6 | -82.6, -17.6, 698.7 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 5.5 g |
| `intake_module` | printed | structure | 130.1 | 130.1 | 2.0, -0.0, 260.8 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 132.6 g |
| `latch_arm` | printed | structure | 0.3 | 0.3 | 65.7, 5.3, 707.3 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 0.3 g |
| `nose_cone` | printed | structure | 67.8 | 67.8 | 0.3, -0.0, 759.6 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 70.4 g |
| `payload_module` | printed | structure | 171.1 | 171.1 | -3.3, -0.7, 611.2 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 184.1 g |
| `piston` | printed | structure | 25.2 | 25.2 | -0.0, -0.0, 550.1 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 31.8 g |
| `propulsion_module` | printed | structure | 346.8 | 346.8 | 1.7, 0.1, 85.3 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 387.8 g |
| `servo_cover_1` | printed | structure | 3.4 | 3.4 | 72.4, 8.8, 45.9 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 3.4 g |
| `servo_cover_2` | printed | structure | 3.4 | 3.4 | -8.7, 72.4, 45.9 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 3.4 g |
| `servo_cover_3` | printed | structure | 3.4 | 3.4 | -72.4, -8.7, 45.9 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 3.4 g |
| `servo_cover_4` | printed | structure | 3.4 | 3.4 | 8.7, -72.4, 45.9 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 3.4 g |
| `vane_1` | printed | structure | 3.6 | 3.6 | 30.3, -0.0, 31.4 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 4.3 g |
| `vane_2` | printed | structure | 3.6 | 3.6 | 0.0, 30.3, 31.4 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 4.3 g |
| `vane_3` | printed | structure | 3.6 | 3.6 | -30.3, 0.0, 31.4 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 4.3 g |
| `vane_4` | printed | structure | 3.6 | 3.6 | -0.0, -30.3, 31.4 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 4.3 g |
| `vane_coupler_1` | printed | structure | 0.9 | 0.9 | 59.3, 0.0, 37.0 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 0.9 g |
| `vane_coupler_2` | printed | structure | 0.9 | 0.9 | -0.0, 59.3, 37.0 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 0.9 g |
| `vane_coupler_3` | printed | structure | 0.9 | 0.9 | -59.3, -0.0, 37.0 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 0.9 g |
| `vane_coupler_4` | printed | structure | 0.9 | 0.9 | 0.0, -59.3, 37.0 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 0.9 g |
| `wire_cover` | printed | structure | 2.2 | 2.2 | 69.7, -0.0, 192.4 | CAD:cadquery | CAD report (cadquery_report.json); estimate was 2.2 g |
| `hinge_bushing_1` | placeholder | actuation | 0.5 | - | 48.5, 0.0, 37.0 | CAD:cadquery | brass tube, Layout.dd_bush_mass_g |
| `hinge_bushing_2` | placeholder | actuation | 0.5 | - | 0.0, 48.5, 37.0 | CAD:cadquery | brass tube, Layout.dd_bush_mass_g |
| `hinge_bushing_3` | placeholder | actuation | 0.5 | - | -48.5, 0.0, 37.0 | CAD:cadquery | brass tube, Layout.dd_bush_mass_g |
| `hinge_bushing_4` | placeholder | actuation | 0.5 | - | -0.0, -48.5, 37.0 | CAD:cadquery | brass tube, Layout.dd_bush_mass_g |
| `servo_horn_1` | placeholder | actuation | 1.0 | - | 65.2, 0.0, 37.0 | CAD:cadquery | vane_drive.horn.mass_g |
| `servo_horn_2` | placeholder | actuation | 1.0 | - | 0.0, 65.2, 37.0 | CAD:cadquery | vane_drive.horn.mass_g |
| `servo_horn_3` | placeholder | actuation | 1.0 | - | -65.2, 0.0, 37.0 | CAD:cadquery | vane_drive.horn.mass_g |
| `servo_horn_4` | placeholder | actuation | 1.0 | - | -0.0, -65.2, 37.0 | CAD:cadquery | vane_drive.horn.mass_g |
| `servo_leads` | lumped | actuation | 26.8 | - | -19.1, -1.4, 209.2 | params | SPEC s3.8: extension wire beyond the splice + slack + connectors at the FCU (servo_leads.mass_g_per_m_ASSUMED) at the route centroid Layout.sl_lead_com |
| `servo_release` | placeholder | actuation | 13.4 | - | 66.3, 14.0, 690.2 | CAD:cadquery | mass = servo.mass_g |
| `servo_vane_1` | placeholder | actuation | 13.4 | - | 80.8, -0.0, 42.5 | CAD:cadquery | mass = servo.mass_g |
| `servo_vane_2` | placeholder | actuation | 13.4 | - | 0.0, 80.8, 42.5 | CAD:cadquery | mass = servo.mass_g |
| `servo_vane_3` | placeholder | actuation | 13.4 | - | -80.8, 0.0, 42.5 | CAD:cadquery | mass = servo.mass_g |
| `servo_vane_4` | placeholder | actuation | 13.4 | - | -0.0, -80.8, 42.5 | CAD:cadquery | mass = servo.mass_g |
| `com` | placeholder | avionics | 20.0 | - | -0.0, 0.0, 522.4 | CAD:cadquery | mass = electronics.com.mass_g (ASSUMED in params; weigh it) |
| `esc` | placeholder | avionics | 92.0 | - | 30.5, -0.0, 405.4 | CAD:cadquery | mass = electronics.esc.mass_g |
| `fcu` | placeholder | avionics | 20.0 | - | -0.0, 0.0, 343.4 | CAD:cadquery | mass = electronics.fcu.mass_g (ASSUMED in params; weigh it) |
| `gns` | placeholder | avionics | 26.6 | - | -42.5, -0.0, 483.4 | CAD:cadquery | mass = electronics.gns.mass_g_ASSUMED (board + shifter + leads) + avionics_module.gns_mount.screws_g_ASSUMED (Layout.gns_mass_g; weigh it) |
| `gps` | placeholder | avionics | 17.0 | - | -84.1, -17.9, 686.5 | CAD:cadquery | mass = electronics.gps.board_mass_g (vendor 17 g with the antenna; weigh it), over the patch and board boxes by volume |
| `gps_ground_plane` | placeholder | avionics | 4.5 | - | -83.5, -17.7, 686.9 | CAD:cadquery | mass = payload_module.gps_pod.ground_plane.mass_g_ASSUMED |
| `gps_lead` | lumped | avionics | 4.4 | - | -72.4, -13.8, 620.2 | params | v0.8: GPS lead along Layout.gps_lead_polyline + slack, connector pair, screws / standoffs / tape (payload_module.gps_pod.lead, ASSUMED) at their mass centroid |
| `misc_electronics` | placeholder | avionics | 40.0 | - | 14.0, 0.0, 377.9 | CAD:cadquery | mass = electronics.misc.mass_g |
| `nav` | placeholder | avionics | 40.0 | - | 33.5, -0.0, 482.9 | CAD:cadquery | mass = electronics.nav.mass_g |
| `radio_log` | placeholder | avionics | 20.0 | - | -0.0, -40.0, 525.4 | CAD:cadquery | mass = electronics.radio_log.mass_g (ASSUMED in params; weigh it) |
| `spring` | placeholder | ejection | 10.0 | - | 0.0, -0.0, 541.9 | CAD:cadquery | mass = 0.5 x hardware_mass_g.spring_piston_hardware |
| `joint_screws_inserts` | lumped | hardware | 40.0 | - | 0.0, 0.0, 439.3 | params | SPEC s6: point mass at the mean of the four joint Z |
| `spring_piston_hardware_balance` | lumped | hardware | 10.0 | - | 0.0, 0.0, 550.1 | params | NOT in the SPEC s6 table: the other half of hardware_mass_g.spring_piston_hardware, lumped at the piston |
| `vane_drive_hardware` | lumped | hardware | 3.0 | - | 0.0, 0.0, 45.9 | params | vane_drive.hardware_g_ASSUMED at the fairings' mid height |
| `cansat_payload` | placeholder | payload | 350.0 | - | 0.0, -0.0, 626.3 | CAD:cadquery | mass = payload.mass_g (competition max) |
| `battery` | placeholder | power | 349.0 | - | -1.5, -0.0, 455.4 | CAD:cadquery | mass = battery.mass_g |
| `motor_leads` | lumped | power | 52.1 | - | 51.6, 0.0, 255.9 | params | v0.9: motor_leads.* (ASSUMED g/m, bullets) along Layout.motor_lead_polyline |
| `edf_unit` | placeholder | propulsion | 369.0 | - | -0.0, -0.0, 156.0 | CAD:cadquery | mass = edf.mass_g (vendor) |
| `chute_pack` | placeholder | recovery | 100.0 | - | -0.0, -0.0, 760.3 | CAD:cadquery | chute + lines/swivel + shock cord |

| 합계 | 값 |
|---|---|
| **전체 (full stack)** | **2609.2 g** = 25.59 N |
| 출력 구조물 (as printed / fully solid) | 941.9 g / 941.9 g |
| 비출력 품목 | 1667.3 g |
| 그룹 `actuation` | 99.6 g |
| 그룹 `avionics` | 284.5 g |
| 그룹 `ejection` | 10.0 g |
| 그룹 `hardware` | 53.0 g |
| 그룹 `payload` | 350.0 g |
| 그룹 `power` | 401.1 g |
| 그룹 `propulsion` | 369.0 g |
| 그룹 `recovery` | 100.0 g |
| 그룹 `structure` | 941.9 g |
| **사출 후 (post-ejection: −cansat, −nose_cone, −chute_pack)** | **2091.4 g** |
| 하강 질량 descent_mass_g (post-ejection + chute_pack) | 2191.4 g |
| 하강 질량 (노즈콘 테더 포함) | 2259.2 g |
| CG full (x, y, z) | 1.9, -0.5, 387.2 mm |
| CG post-ejection (x, y, z) | 2.4, -0.6, 317.2 mm |
| I_xx / I_yy / I_zz (full, about CG) | 0.1334 / 0.1342 / 0.0051 kg·m² |
| I_xx / I_yy / I_zz (post-ejection) | 0.0789 / 0.0796 / 0.0047 kg·m² |
| 발사대 `launch_pad` (지상 장비, 비행 질량 제외) | 196.6 g (solid 356.8 g) |

ESTIMATE 열: 출력 구조물은 해석식(튜브 = 면적 × 벽 × 밀도, 콘/노즈 = 회전 쉘 수치적분, 핀 = 평면형 × t 등)으로 추정. 'as printed'는 벽 5 루프 × 0.42 mm + 15 % 인필 모델, 'solid'는 CAD 질량특성 보고서가 보여줄 완전 충전 값. CAD 보고서가 생기면 자동으로 그 값을 사용한다.

## 2. 추력 / 추중비

| 항목 | 값 |
|---|---|
| 중량 W | 25.59 N (2609 g) |
| T/W @ thrust_nominal | 1.184 → C12 **WARN** |
| T/W @ thrust_peak | 1.453 |
| 호버 스로틀 u_h = (W/T_max)^(1/2) | 0.919 → P 1757 W, I 79.1 A |
| T/W @ climb_throttle 1.0 | 1.184 |
| T/W = 1.30 에 필요한 스로틀 | 1.048 |
| T/W = 1.30 허용 최대 질량 | 2377 g |

## 3. 배기 유속 · 베인 제어권한 (SPEC §8.3–8.4)

| 조건 | T N | A_e m² | V_e m/s | q_e Pa | ṁ kg/s |
|---|---|---|---|---|---|
| at_nominal_thrust | 30.3 | 0.00507 | 69.8 | 2986 | 0.434 |
| at_hover_thrust | 25.6 | 0.00507 | 64.2 | 2522 | 0.399 |
| at_climb_throttle | 30.3 | 0.00507 | 69.8 | 2986 | 0.434 |

| 조건 | F/베인 @ 20° N | M_pitch N·m | M_roll N·m | α_pitch rad/s² | α_roll rad/s² | M_pitch per deg |
|---|---|---|---|---|---|---|
| at_nominal_thrust | 2.40 | 1.682 | 0.269 | 12.60 | 52.96 | 0.0841 |
| at_hover_thrust | 2.03 | 1.420 | 0.227 | 10.64 | 44.72 | 0.0710 |

- 레버암 l_v = z_CG − vane_hinge_z = 350.2 mm; S_v = 960 mm²; C_Lα = 2.4 /rad (stall 25.0°); r_v = 28.0 mm
- 관성 (full, CG 기준): I_xx 0.1334, I_yy 0.1342, I_zz 0.0051 kg·m² (점질량+봉/상자 근사)
- CG 횡방향 오프셋 2.0 mm (배터리 −x / ESC·NAV +x 배치) → 트림 토크 ≈ W × 오프셋 = 0.051 N·m

### 롤 권한 vs 잔류 스월 토크

| 항목 | 값 |
|---|---|
| ω (kv × V, 무부하) | 32190 rpm = 3371 rad/s |
| P/ω @ power_nominal | 0.672 N·m |
| τ_swirl = 0.15 × P/ω (ASSUMED) | 0.101 N·m |
| M_roll,max @ hover / @ nominal | 0.227 / 0.269 N·m |
| 권한/스월 비 @ hover / @ nominal | 2.25 / 2.67 |
| 스월 트림 δ_z | 8.87° = 44 % of δ_cmd,max (mixer roll_priority_scale 0.5) |

swirl ~ u^2 and q_e ~ u^2 -> the trim deflection is roughly throttle-independent; omega from kv x V is the unloaded speed, so tau_swirl is a lower bound.

## 4. 임무 에너지 (1-D 수직 상승)

모델: 1-D vertical, thrust T_max u^n with first-order lag, ramp, drag 0.5 rho v^2 Cd A_ref, cut-off h + k v^2/2g; A_ref = 0.01193 m², Cd 0.6, 램프 0.5 s, 추력 시정수 0.15 s, coast_drag_factor 1.2, 배터리 2200 mAh × 22.2 V = 48.8 Wh, C-rating 70C.

| 시나리오 | T/W@u | 도달 | t_cut s | h_cut m | v_cut m/s | 정점 m | 오버슈트 m | t_apogee s | I_peak A | C-rate | Wh 사용 | 잔량 % | geofence |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| target_140m @ u=1.00 (sensitivity) | 1.18 | yes | 12.6 | 118.4 | 18.8 | 138.9 | -1.1 | 14.7 | 102.0 | 46.4C | 7.71 | 84.2 | ok |
| competition 체험부 175m | 1.18 | yes | 14.2 | 149.1 | 20.6 | 173.2 | -1.8 | 16.4 | 102.0 | 46.4C | 8.69 | 82.2 | ⚠ geofence |
| competition 슬기부 250m | 1.18 | yes | 17.2 | 216.1 | 23.6 | 246.8 | -3.2 | 19.7 | 102.0 | 46.4C | 10.59 | 78.3 | ⚠ geofence |
| competition 창작부 350m | 1.18 | yes | 20.9 | 307.6 | 26.3 | 345.0 | -5.0 | 23.6 | 102.0 | 46.4C | 12.89 | 73.6 | ⚠ geofence |

지오펜스 geofence_altitude_m = 160.0 m: 이를 넘는 시나리오는 §8.8 상태기계에서 ABORT 된다.

## 5. 회수 (낙하산)

| 하강 질량 정의 | 질량 g | 요구 면적 m² @ 5.5 m/s | 요구 D m | params D m | params 낙하산 하강속도 m/s | 착지 에너지 J |
|---|---|---|---|---|---|---|
| post_ejection | 2091 | 1.476 | 1.37 | 1.40 | 5.39 | 30.3 |
| post_ejection_plus_chute_pack (SPEC descent_mass_g) | 2191 | 1.547 | 1.40 | 1.40 | 5.51 | 33.3 |
| post_ejection_plus_nose_and_chute_pack (tethered nose) | 2259 | 1.594 | 1.42 | 1.40 | 5.60 | 35.4 |

Cd = 0.75, ρ = 1.225 kg/m³. 캔위성은 자체 낙하산(별도).

## 6. 안정성 (CG vs Barrowman CP)

| 성분 | C_Nα /rad | x from tip mm | z mm |
|---|---|---|---|
| nose | 2.000 | 70.0 | 786.3 |
| fins | 3.642 | 799.5 | 56.8 |
| boat_tail_cone | -0.752 | 745.0 | 111.3 |

| 항목 | 값 |
|---|---|
| 핀 노출 평면형 (root c / tip c / s / LE sweep) | 89.6 / 60.0 / 76.0 / 29.6 mm, 루트 반경 47.4 mm, 기준 직경 120 mm |
| CP (nose + fins) | z = 315.4 mm, C_Nα 합 5.642 /rad |
| CG full / post-ejection | z = 387.2 / 317.2 mm |
| 정적 여유 full (coast 형상) | **0.60 cal** → C13 **WARN** (PASS ≥ 1.0, WARN ≥ 0.5) |
| 정적 여유 post-ejection (낙하산 하강 중 — 참고용) | 0.01 cal |
| 감도: 보트테일(콘 120→94.8, z 90–131.0; 그 위 원통부는 동체) 포함 | CP z = 346.8 mm, 여유 0.34 cal |
| CG − vane_hinge_z (= l_v) | 350.2 mm (C13 요구 ≥ 250) |

## 7. 국내 법규 분류 (항공안전법, 질량 기준)

| 항목 | 값 |
|---|---|
| 최대이륙중량 | 2.609 kg |
| 조종자 증명 | **3종 (2 kg 초과 ~ 7 kg 이하: 필기시험)** |
| 기체신고 (> 2 kg) | **필요** |
| 비행승인 (> 150 m) @ 목표 고도 140 m ★ | 불필요 (고도 기준; 관제권/비행금지구역은 별도) |
| 비행승인 @ 예측 정점 138.9 m | 불필요 |
| 경연 사출 고도 (m) | 체험부 175, 슬기부 250, 창작부 350 → 비행승인 필요: 체험부, 슬기부, 창작부 |

> 항공안전법: 최대이륙중량 2 kg 초과 -> 기체신고 및 3종 이상 조종자 증명(2~7 kg); 고도 150 m 초과 또는 관제권/비행금지구역 -> 비행승인 필요. 최종 확인은 드론원스톱(drone.onestop.go.kr).

## 8. 경고 · 가정

- ⚠ CAD report total 2599.2 g differs from the budget sum 2609.2 g (items missing from one side, or the 10 g hardware balance)
- ⚠ C12 WARN: T/W at thrust_nominal = 1.184 (PASS >= 1.3, WARN >= 1.15); mass 2609 g vs 2377 g allowed for 1.30
- ⚠ scenario competition 체험부 175m: altitude exceeds geofence_altitude_m 160 m -> ABORT by the onboard geofence interlock I5 (SPEC s8.8)
- ⚠ scenario competition 슬기부 250m: altitude exceeds geofence_altitude_m 160 m -> ABORT by the onboard geofence interlock I5 (SPEC s8.8)
- ⚠ scenario competition 창작부 350m: altitude exceeds geofence_altitude_m 160 m -> ABORT by the onboard geofence interlock I5 (SPEC s8.8)
- ⚠ C13 WARN: static margin 0.60 cal, CG above hinge 350 mm

- `_ASSUMED` 키 (실측 전 신뢰 금지): design_load_N_ASSUMED, electrical_plant_ASSUMED, esc_options_ASSUMED, gear_backlash_deg_ASSUMED, hardware_g_ASSUMED, hub_d_ASSUMED, hybrid_extra_ASSUMED, lead_exit_ASSUMED, len_lip_top_to_housing_bottom_ASSUMED, lip_t_ASSUMED, mass_g_per_m_ASSUMED, mass_g_per_m_ASSUMED, od_ASSUMED, print_tolerance_ASSUMED, residual_swirl_torque_fraction_ASSUMED, ribbon_t_ASSUMED, ribbon_w_ASSUMED, stock_len_in_edf_mass_ASSUMED, tether_kit_ASSUMED, tether_plant_ASSUMED, vtol_crossflow_cd_ASSUMED
- 푸시로드 길이 (arm tip → horn pin, 베인 1): 0.00 mm — CAD 보고서 `pushrod_len_mm` 와 대조
- 노즈 캐비티 0.66 L (요구 0.4 L), 흡입 면적비 5.97 (최소 개구, v0.6.2 창 아래 거싯 포함; 바깥면 6.23) (요구 ≥ 2.0)
- 추력·전력은 벤더 수치 (SPEC §12: 비행 전 벤치 추력 시험 필수); 배터리 전압 강하·ESC 효율은 모델에 없음.
