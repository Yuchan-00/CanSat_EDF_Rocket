"""VTOL scenarios (SPEC 8.9, v0.7 / v0.7.1): the ground operator, the field and demo missions, the safety scenarios,
the pass / fail criteria, the hand-push envelope, the vertical-gain + NAV-estimator tuning and the renderer log export.

The vehicle (``sim.sim6dof.FlightSim`` with ``Scenario.mode = "vtol"``) flies the altitude itself
(``sim.onboard.FcuStack``, v0.7.1 tick order: attitude controller -> vanes -> supervisor -> ``sim.vertical_ctrl`` ->
ESC); the ground only presses ON / UP / DOWN / OFF (and KILL).  ``VtolOperator`` is that person: it acts on the
latest decoded telemetry only and presses buttons of the real ``sim.ground_station_logic.GroundAssist``
(repeat-until-acknowledged, OFF / KILL pre-empt), whose frames cross ``sim.link_model`` (TELEM v2, 39 B).

v0.7.1 (architect 2026-10-20): the NAV runs the gated Mahony estimator (``sim.estimator.GatedMahonyEstimator``,
``sim6dof.VTOL_ESTIMATOR``) and the VTOL altitude filter with the accelerometer z-bias state
(``sim.state_machine.VtolAltitudeFilter``); the vertical controller v2 has the reference-rate feed-forward and the
three touchdown signatures; the supervisor has V8 / V9 (baro-lost time limit) and V10 (LAND floor timeout) and the
LANDED rules; two build profiles: 'field' (mission.vtol as written) and 'demo' (the indoor 1-2 m hover the user will
show, a person pushing the vehicle with a padded pole).  The 6-DOF models the body cross-flow drag and the EDF ram
(momentum) drag (params aero.vtol_*), accelerometer vibration, the baro's static-pressure error with the fan running,
wind whose direction swings / wanders / reverses, and hand pushes (``sim.sim6dof.Push``).

Fix round (2026-10-21, after the independent physics review and the v0.7.1b control fix): the VTOL scenarios fly the
CAD lateral CG (``cad_cg_offset_mm``; ``axis_cg`` what-ifs), the v0.7.1b attitude overrides and thrust learning
(``sim.sim6dof``); new scenarios V_strong_plant / D_strong_plant (thrust x1.25) and V_weak_plant / D_weak_plant (x0.85)
judged on the vertical criteria + LANDED by a touchdown signature, V_wind_swing_slow (a swing at the estimator-drag
loop's own period), the ideal-attitude what-ifs both ways (CAD CG / CG on the axis); the swing and the reversal no
longer inherit the nominal step gusts; the push envelope is run for the residual swirl x1 / x0 / x2 with the I3 cause
(tilt rate / spin) split; the estimator feasibility uses the drag time constant net of the vane counter-force (9.55 s)
and also the params rule 1 / kp >= tau_d (``estimator_feasible``), ``--tune-est`` re-tunes the estimator gains only.

Run ``python sim/vtol_scenarios.py`` (every scenario: plots, renderer logs, ``sim/out/vtol/vtol_metrics.json``, the
20-seed baro study, the push envelope), ``--tune`` (vertical + estimator gains -> ``sim/out/vtol_gains.json``, then
every scenario with them), ``--tune-est`` (estimator gains only), ``--tune-only``, ``--only NAME[,NAME]``,
``--envelope-only``, ``--workers N``, ``--no-export``.

Field nominal mission (``nominal_script``): ON at 1 s -> spool warning -> climb to 5 m -> HOLD; UP x3 -> 8 m; three
gusts while holding 8 m; DOWN -> 7 m; OFF -> landing on the ground beside the pad; LANDED -> SAFE.  OFF comes before
the V2 flight-time LAND (30 s of motor time).  Demo nominal ("시연 정상", ``demo_script`` / ``DEMO_PUSHES``): demo
profile, ON -> 1.5 m hover, five pushes from different directions at different heights above the intake windows,
OFF -> landing.

Criteria (module constants, stated here and never loosened to make a run pass):
``NOMINAL_CRITERIA`` (the task of 2026-10-20): no interlock acted; the landing ends LANDED -> SAFE; the motor is not
stopped in the air (no false touchdown); first touchdown |v_z| <= 1.0 m/s, horizontal speed <= 0.5 m/s RELATIVE TO THE
AIR MASS (v0.7.1: the mean wind - steady + swing / wander / reversal, without turbulence and gusts; the vehicle has no
position sensing and drifts with the air: a physical limit; the ground-relative speed is reported next to it), tilt
<= 5 deg; altitude tracking error in HOLD <= 0.5 m RMS (true altitude against the controller's reference); after each
gust the TRUE tilt is back within 2 deg in <= 2 s (from the gust's end).  ``PUSH_CRITERIA`` (v0.7.1, the demo): after
each push no interlock, the motor still runs and the attitude is back within 2 deg of its pre-push attitude <= 2 s
after the push ends.  Throttle saturation time is reported, not judged (T/W ~1.19, u_hover 0.915).

Renderer log contract (``export_npz``, read by ``tools/sim_video.py``; ``.npz``, one row per FCU tick, 250 Hz)::

  t [N] s            pos [N,3] world m, the exit-plane centre (pos_ref = "exit_plane"; post-ejection CG after a release)
  q [N,4]            body -> world (w, x, y, z), truth
  vel [N,3] m/s      CG velocity, truth            delta [N,4] rad   actual vane angles (servo outputs)
  throttle [N]       the FCU's ESC command u (0..1, vertical controller while the motor is enabled, else 0)
  thrust [N] N       fan thrust (lagged), truth    state [N]  link_proto STATE_* (SAFE 0 ARMED 1 POWERED 2
                                                              UNPOWERED 3 RELEASED 4 ABORT 5)
  phase [N]          link_proto PHASE_* (NONE 0 SPOOL 1 CLIMB 2 HOLD 3 LAND 4 LANDED 5)
  h_target [N] m     supervisor altitude target in SPOOL / CLIMB / HOLD, NaN otherwise (LAND has no target)
  h_ref [N] m        vertical-controller reference, NaN while the controller is idle
  h_est [N] m        NAV altitude filter (what the vehicle believes)     tilt_deg [N]  true tilt
  wind [N,3] m/s     world wind incl. gusts and turbulence               gust_active [N] bool
  wind_mean [N,3]    v0.7.1: the mean wind (steady + swing / wander / reversal + speed modulation; no gusts, turbulence)
  push_force [N,3] N v0.7.1: the force the person applies (world; 0 when not pushing)
  push_point [N,3] m v0.7.1: the contact point on the body surface (world; NaN when not pushing)
  push_active [N]    v0.7.1: bool
  ground_z () m      the ground level away from the pad (pad top = 0; default -0.11 m, the pad's foot level)
  event_t [E] / event_text [E]   supervisor transitions, rejections, faults, touchdowns, ground-station events ("GS ...")
  cmd_t [C] / cmd_name [C]       each ground command at its first transmission (ON, UP, DOWN, OFF, KILL)
  meta ()            JSON: scenario, title, title_ko, seed, mode "vtol", profile, params_sha256, notes / notes_ko (model
                     caveats), pos_ref, z_cg_m, t_max_n, limits {u_max, hover_throttle, tilt_abort_deg, ceiling_m,
                     geofence_m, tilt_rate_abort_dps, spin_rate_abort_dps}, poster_t, source, vgains (+ vgains_source),
                     estimator (+ its gains), criteria (incl. gust_band_deg), verdict, touchdown summary, pushes (per push
                     response), aero (the v0.7.1 drag model numbers, ASSUMED coefficients named), timing
  extra (not read by the renderer): h_true [N] m (CG rise above its pad height), vz_est [N], v_cell [N],
                     vc_sat [N] (+1 throttle at the ceiling, -1 at u_min), surface_z [N] m (surface under the exit
                     plane), tilt_ctrl_deg [N] (the attitude estimate's tilt), tilt_est_err_deg [N], reason [N]
                     (latched TELEM reason code), vc_td_by [N] (0 / still 1 / support 2 / floor 3 / all 4), alt_bias [N],
                     vc_k_thrust [N] (the learnt thrust ratio k_T, v0.7.1b), vc_low_alt [N] (LAND in the flare band)

v0.8 (GPS position hold; architect 2026-10-21, params mission.vtol.position_hold):
* ``base_vtol`` builds the GPS build by default (``GPS_DEFAULT`` = ``sim.sim6dof.GpsSim()``, the NEO-7M class model; the tuned
  position gains of the profile from ``sim/out/vtol_gains.json`` "position"); ``gps=None`` = the v0.7.1 build,
  ``gps_off`` = the v0.8 build without the GPS (indoor fallback / the before of the position hold on the same plant),
  ``v071_build``, ``with_gps`` (receiver fields replaced).
* ``scenarios``: every v0.7.1 scenario re-run with the GPS build (same names; the weak plant at T/W 1.02 now expects the
  authority rule V14), the station-keeping set (``G_calm``, ``G_wind3/5/8``, ``G_wind_limit`` = the operator limit of params
  wind_limits_mps (6 m/s + gusts to field_gust, 7 m/s since the physics review) with turbulence intensity 0.2,
  ``G_gusts8/10``; the swing at the estimator period ``V_wind_swing_slow``,
  ``V_wind_reversal``), the demo set (``GD_*``: calm, 4 m/s, 3 N s pushes in the safe band, along a pole from 1.0 / 0.3 m
  hand height, consecutive 3 s / 1.5 s apart, 5 N s), the GPS faults (outages 3 / 20 s, lead fault, multipath jump,
  degraded, a velocity bias for the plausibility rule V13, sky blocked 34 / 60 %, no answer, cold start), the heading
  set-up error 15 deg (judged: params heading.tolerance_deg) and 30 / 45 / 90 deg (beyond it: explorations), 0.3 m/s
  velocity noise, latency 0.30 s, a mis-set latency 0.45 s (exploration), random multipath, the NEO-M8N, the take-off
  check V15 (with and without an uncalibrated baro fan offset), and the what-ifs (GPS off, ideal attitude, the v0.7.1
  build).
* criteria (``v08_criteria`` = params criteria_v08_ASSUMED + V08_* constants, ``v08_checks``, ``push_v08_checks``,
  never loosened): no interlock; LANDED -> SAFE; no false touchdown; touchdown |v_z| <= 1 m/s, <= 0.5 m/s over the GROUND,
  tilt <= 5 deg, energy-method tip-over margin > 0 (``sim.sim6dof.tip_over_vcrit``, params tipover; physics review), <= 3 m
  from the hold point; altitude hold RMS <= 0.5 m; station keeping (TRUE CG vs the TRUE CG at the hold capture, HOLD
  without the push windows) max <= 3.0 / 2.5 m (field / demo); the controller's NAV-frame RMS <= 0.5 m; tilt setpoint <=
  cap (+ the 0.1 deg series tolerance), true tilt in HOLD <= cap + 5 deg; pushes <= 3 N s back within 1 m of the
  pre-push position <= 6 s.  The TRUE station-keeping RMS is judged as a statistic (params criteria note, architect after
  the physics review): its p50 over >= 20 seeds per scenario family <= 1.0 m (``sk_stats_study``, sk_rms_stats.json),
  p90 reported.  The v0.7.1 checks stay in every record (``checks``).
* ``horizontal_margins``: the linear horizontal loop (the attitude closed loop of ``sim.loop_margins``, the drag time
  constant, the NAV / slot / pipeline delays) - its phase margin is the LINEAR heading-rotation limit only (the flown
  tolerance is set by the NAV gate, physics review); ``tune_position``
  (``--tune-pos``): the position gains under PM >= 45 deg / GM >= 2, stage 1 on ``ptune_cases`` (seeds 300+), stage 2 on
  campaign seed 808; validation on the fresh seed PVALID_MC_SEED (4242).
* renderer contract v5 (``export_npz``, GPS runs; tools/sim_video.py): hold_point [N,3] (TRUE CG at the hold capture, z 0,
  NaN while none), hold_point_est [N,3] (the NAV hold point in the world frame), gps_ok, gps_sats, gps_hacc_m, gps_fix
  (0 / 3), pos_est [N,3] (the NAV CG estimate, z NaN), tilt_sp_deg [N] (+ tilt_sp [N,2] level frame), pos_mode, pos_dist,
  psi_err_deg, gps_err [N,2], gps_resets, pos_requests, pos_limited, pos_guard, vel_est_h, gps_gate_ok; meta.gps_on,
  meta.gps (receiver, module, hold captures, the quote), meta.criteria.station_keeping_max_m {field, demo},
  touchdown_from_hold_point_m, touchdown_ground_speed_mps.

v0.9 (VTOL hardware round, architect 2026-10-28; plant in ``sim.sim6dof`` / ``sim.tether`` / ``sim.edf``):
* ``vehicle(power)`` = the VTOL hardware configuration of params mission.vtol.hardware_config ('vtol') with the power
  source's CAD report (battery default; ``run_scenario`` picks the scenario's ``power_source``); ``base_vtol`` takes the
  power source's CAD lateral CG, the NAV GPS lever arm of the top-cap antenna (``nav_lever_vtol``, a GpsSim.nav override
  until the firmware round puts it into params) and clips push heights above the 475 mm vehicle to the top cap
  (``retarget_pushes``: the v0.8.1 'nose' pushes act at the top cap).  Every v0.8.1 named scenario is kept with its name
  and expectation; the model-ratio cases (x1.25, T/W 1.02, the no-lift T/W 0.93 = ``plant_no_lift_scale``) fly without
  the thrust's voltage term (``MODEL_RATIO_VEXP``).
* criteria: + touchdown ground speed <= 0.3 m/s on the VTOL configuration (``V09_TOUCHDOWN_VH_MAX_MPS``, architect
  decision g; the rigid-foot tip-over between two fins is ~0.64 m/s) and the tip-over margin on the solid-fin footprint
  (plant); the tether expectations of the proposal (``tether_checks``: arrest load <= the 300 N proof load, no ground
  contact > 1 m/s after a power loss, LAND <= 0.2 s after a hybrid tether loss, no supply trip, buffer >= 3.4 V/cell).
* ``scenarios_v09``: the thrust envelope (2.14 / 2.46 / 1.9 kgf), the tether test (battery-less: taut / slack feed,
  indoors / outdoors, 3 m, snag, unplug at 1 m / 2 m, cut, supply trip, the supply limit with a weak fan, a set-point sag),
  the hybrid demo (hover, crossover, cut / unplug on the buffer, buffer low).  The rules W_LOST / W_SAG / W_BUF (tether
  proposal firmware.rules_ASSUMED_thresholds) are expected there - the v0.8.1 supervisor does not have them yet.
* renderer contract v6 (``export_npz``): hardware_config, power_source, tether_attach_body_m; with a rig tether_anchor_m,
  tether_tension_N, tether_connected (the cable at the vehicle); with a supply supply_V / supply_A (+ supply_v_V /
  supply_i_A), supply_state (0 CV, 1 limit, 2 off), tether_power; hybrid buffer_soc / buffer_v_V / buffer_i_A; v_bus.

Invariant: every number judged here comes from a run of ``sim.sim6dof`` in this process (or its worker
processes); the criteria are module constants; nothing about the vehicle is decided here (the operator presses
buttons only; the tuner only chooses the vertical, estimator and position gains it writes to ``sim/out/vtol_gains.json``
- never params.json).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import math
import os
import sys
import time
from concurrent.futures.process import BrokenProcessPool
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(_HERE)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from sim import link_proto as lp  # noqa: E402
from sim.controller import gains_from_control  # noqa: E402
from sim.fc_timing import FCTiming  # noqa: E402
from sim.ground_station_logic import GroundAssist  # noqa: E402
from sim.sim6dof import (G0, VTOL_ACCEL_VIBRATION_RMS_G_ASSUMED, VTOL_ESTIMATOR, FlightSim, GpsSim, Gust,  # noqa: E402
                         Push, Scenario, SensorConfig, SimOptions, SimResult, compute_vtol_metrics, exit_plane_positions,
                         neo_m8n, push_recovered, vtol_aero_geometry)
from sim.position_ctrl import PositionParams  # noqa: E402
from sim.tether import PowerConfig, PowerEvent, TetherConfig  # noqa: E402
from sim.vehicle import Vehicle, load_vehicle  # noqa: E402
from sim.vertical_ctrl import VerticalParams  # noqa: E402

OUT_DIR = os.path.join(PROJECT_ROOT, "sim", "out", "vtol")
GAINS_JSON = os.path.join(PROJECT_ROOT, "sim", "out", "gains.json")          # attitude gains (run_scenarios.py)
VTOL_GAINS_JSON = os.path.join(PROJECT_ROOT, "sim", "out", "vtol_gains.json")  # vertical + estimator gains (--tune)
PARAMS_PATH = os.path.join(PROJECT_ROOT, "params.json")
VGAIN_KEYS = ("alt_kp", "vz_kp", "vz_ki", "vz_i_max")
EGAIN_KEYS = ("est_kp", "est_ki", "est_accel_lpf_hz")      # GatedMahonyEstimator kp, ki, accel_lpf_hz
GAIN_KEYS = VGAIN_KEYS + EGAIN_KEYS
PROFILES = ("field", "demo")

# ---- criteria (task of 2026-10-20; judged on the nominal missions and on every campaign run) ----
NOMINAL_CRITERIA: dict[str, float] = {
    "touchdown_vz_max_mps": 1.0,          # |v_z| at the first touchdown
    "touchdown_vh_max_mps": 0.5,          # horizontal speed at the first touchdown, RELATIVE TO THE MEAN WIND (v0.7.1)
    "touchdown_tilt_max_deg": 5.0,        # true tilt at the first touchdown
    "hold_rms_err_max_m": 0.5,            # RMS of (true altitude - reference) over the HOLD phase
    "gust_band_deg": 2.0,                 # after each gust the TRUE tilt is back within this band ...
    "gust_recovery_max_s": 2.0,           # ... within this time from the gust's end
}
PUSH_CRITERIA: dict[str, float] = {
    "push_band_deg": 2.0,                 # after each push the attitude is back within this of its pre-push attitude ...
    "push_recovery_max_s": 2.0,           # ... within this time from the push's end; no interlock; the motor still runs
}
CRITERIA_TEXT = ("no interlock; LANDED -> SAFE; no false touchdown (motor stopped in the air); touchdown |v_z| <= 1.0 "
                 "m/s, horizontal <= 0.5 m/s relative to the mean wind (the air mass the vehicle drifts with; the "
                 "ground-relative speed is reported), tilt <= 5 deg; hold RMS (true - reference) <= 0.5 m; true tilt back "
                 "within 2 deg <= 2 s after each gust")
CRITERIA_TEXT_KO = ("인터록 작동 없음; 착지 확인(LANDED) 후 SAFE; 공중에서 모터 정지(오착지) 없음; 착지 순간 수직 속도 "
                    "1.0 m/s 이하, 수평 속도 0.5 m/s 이하(평균 바람 기준 - 위치 센서가 없어 공기와 함께 흐르는 것은 물리적 "
                    "한계, 지면 기준 속도는 따로 표시), 기울기 5도 이하; 호버링 고도 오차 RMS 0.5 m 이하; 돌풍이 끝난 뒤 "
                    "2초 안에 실제 기울기 2도 이내로 복귀")
PUSH_CRITERIA_TEXT = ("after each push: no interlock, the motor still runs, and the attitude is back within 2 deg of its "
                      "pre-push attitude <= 2 s after the push ends")
PUSH_CRITERIA_TEXT_KO = "밀기마다: 인터록 작동 없음, 모터 계속 작동, 밀기가 끝난 뒤 2초 안에 밀기 전 자세의 2도 이내로 복귀"
VERTICAL_CHECKS = ("no_interlock", "landed_then_safe", "no_false_touchdown", "touchdown_vz", "hold_rms")

# field nominal timeline [s, absolute]: ON at 1 s; motor at ~3.7 s; HOLD at 5 m at ~8.7 s, 8 m at ~12.7 s
T_ON = 1.0
NOMINAL_GUSTS_T0 = (15.0, 19.0, 23.0)
NOMINAL_GUST_DIRS = (0.0, 120.0, 240.0)
T_DOWN = 26.0
T_OFF = 29.0                              # motor time at OFF ~25 s < 30 s (V2 = max_flight_s - land_reserve_s)
T_END = 90.0
FAULT_T = 16.0                            # faults in HOLD at 8 m (after the first gust)
IDEAL_ATT = "truth_noise"                 # what-if attitude reference: truth + 0.5 deg white noise (BNO085 class)
OLD_EST = "pad_gyro"                      # the v0.6.3 / v0.7.0 NAV estimator (pad alignment, then gyro only)
GUST_SWEEP_MPS = (16.0, 20.0, 25.0)       # single 2 s gusts in HOLD at 8 m
# demo timeline: ON at 1 s, motor at ~3.7 s, HOLD at 1.5 m at ~6.7 s; pushes from 10 s; OFF at 30 s (motor time ~26 s)
DEMO_T_OFF = 30.0
DEMO_T_END = 50.0
DEMO_PUSH_T0 = 10.0                       # single-push demo cases
# "시연 정상": five pushes from alternating directions (a person pushing it back and forth), above the intake windows
# (window top 0.327 m above the exit plane), 5-15 N, 0.2-0.4 s (user 2026-10-20); half-sine, central contact
DEMO_PUSHES = (
    Push(10.0, 0.30, 10.0, azimuth_deg=0.0, z_body_m=0.40),
    Push(14.0, 0.25, 8.0, azimuth_deg=180.0, z_body_m=0.45),
    Push(18.0, 0.35, 12.0, azimuth_deg=90.0, z_body_m=0.38),
    Push(22.0, 0.20, 6.0, azimuth_deg=270.0, z_body_m=0.50),
    Push(26.0, 0.30, 15.0, azimuth_deg=45.0, z_body_m=0.40),
)
NOSE_Z_M = 0.80                           # "at the nose" (nose cone 0.716 .. 0.856 m above the exit plane)
SAME_IMPULSE_NS = 2.0                     # push at the CG height vs at the nose
BARO_OFFSET_DEMO_M = 1.0                  # true baro static-pressure error at hover thrust (task: 1.0 m)
# fix round: plant thrust vs the controllers' model (physics review HIGH: x1.20-1.30 stopped the motor 6-8 m up before
# v0.7.1b; the proxy table gives +23 % on a fresh 25.2 V pack).  x0.85 = T/W 1.02, the weakest plant that still hovers
# (x0.80 cannot lift off, T/W 0.96)
PLANT_STRONG = 1.25
PLANT_WEAK = 0.85                         # v0.7.1 value (T/W 1.02 on the v0.7.1 vehicle); v0.8: ``plant_weak_scale``
PLANT_WEAK_TW = 1.02                      # v0.8: the weak plant = plant T/W 1.02 on the current vehicle (the v0.7.1 meaning:
                                          # the weakest plant that still hovers); the GPS pod made the vehicle heavier
PLANT_NO_LIFT = 0.80                      # thrust x0.80: T/W < 1 (v0.8 vehicle 0.93) - the take-off check V15 case
PLANT_NO_LIFT_TW = 0.93                   # v0.9: the no-lift plant = T/W 0.93 on the current vehicle (``plant_no_lift_scale``;
                                          # x0.80 is T/W 1.50 on the light VTOL vehicle)
MODEL_RATIO_VEXP = 1e-9                   # v0.9: the model-ratio scenarios (x1.25, T/W 1.02, T/W 0.93) fly without the thrust's
                                          # voltage term ((V / 22.2)^1e-9 = 1): the ratio is the plant / model ratio at any
                                          # pack voltage (the v0.9 electrical plant would add x1.2 on a fresh pack)
PLANT_FORBID = "V10|V6|V5|V4|V9"          # no LAND timeout, hard cut-off, fall, geofence or baro-limit rule may act
DETECTOR_SIGNATURES = ("still", "support", "floor", "all")   # sim.vertical_ctrl.TD_NAMES without "none"

PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]  # run_scenarios
INK, INK2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#fcfcfb"
MODEL_NOTES = ("SIMULATION (sim.sim6dof 6-DOF, VTOL build v0.8; the plant = the v0.8 CAD vehicle with the GPS pod). No "
               "ground-effect model (free-air thrust down to the ground). Without the GPS (GPS-off what-ifs, the v0.7.1 build) "
               "nothing on the vehicle senses horizontal position: wind, pushes, the CG offset and tilt-estimate errors move "
               "it sideways and only drag slows it (reported as drift). Horizontal drag = body "
               "cross-flow drag (C_dc 1.2 ASSUMED - finite cylinders of L/D ~7 sit nearer 0.7-0.9, the campaign spreads "
               "+-30 %; A_plan = OD x length) + EDF ram drag mdot V (mdot = sqrt(rho A_exit T), actuator disk) at the "
               "intake below the CG: with the vane counter-force the calm-air drag time constant is ~9.6 s (vtol_aero_geometry). Lateral "
               "CG = the CAD value (v0.8: 0.9 mm off the thrust axis; v0.7.1: 2.57 mm) unless noted. EDF residual swirl torque 15 % ASSUMED. "
               "Ground contact = exit-plane centre on a rigid surface, no bounce, no tip-over. Accelerometer vibration 0.1 g "
               "RMS at hover (ASSUMED). IMU modelled at the CG (no lever arm). Thrust T = T_max u^2 (PROXY table, bench "
               "test pending) without a battery-voltage term unless noted. Not modelled: jet damping, thrust lapse with "
               "axial inflow (both stabilising), thrust-line misalignment. Wind uniform with height. Hand push = a force of "
               "the given profile at a body surface point (v0.8 pole pushes: along the pole from the hand height; the person is not "
               "modelled; an off-centre contact or an angled pole needs friction >= tan(angle)).")
MODEL_NOTES_KO = ("시뮬레이션 결과입니다(6자유도, VTOL v0.8, GPS 포드 포함 CAD 기체). 지면 효과 모델 없음. GPS가 없으면 기체에 위치/수평 속도 센서가 "
                  "없어 바람·밀기·무게중심 치우침·자세 추정 오차로 옆으로 흘러가며 공기 저항으로만 멈춥니다. 수평 저항 = "
                  "동체 횡류 저항(Cd 1.2 가정) + 덕트팬 흡입 운동량 저항. 무게중심은 CAD 값(v0.8: 추력축에서 0.9 mm). 덕트팬 "
                  "잔류 회전 토크 15%(가정). 착지 = 강체 지면, 튕김·전복 모델 없음. 가속도계 진동 0.1 g(가정). 추력표는 "
                  "대체 제품 자료(실측 전). 밀기 = 동체 표면 한 점에 가한 힘(막대·사람은 모델 아님).")
VERTICAL_CRITERIA_TEXT = ("vertical criteria: no interlock; LANDED -> SAFE; no false touchdown; touchdown |v_z| <= 1.0 m/s; "
                          "hold RMS <= 0.5 m; LANDED by a touchdown signature of the detector")
VERTICAL_CRITERIA_TEXT_KO = ("수직 기준: 인터록 작동 없음; 착지 확인 후 SAFE; 공중 모터 정지 없음; 착지 수직 속도 1.0 m/s 이하; "
                             "호버링 고도 오차 RMS 0.5 m 이하; 착지 감지 신호로 착지 확인")

# ---- v0.8: GPS position hold (architect 2026-10-21; params mission.vtol.position_hold) ----
GPS_DEFAULT = GpsSim()                    # u-blox NEO-7M class receiver model with its stated sources (sim.sim6dof.GpsSim)
PGAIN_KEYS = ("kp_pos", "kv", "ki", "kd")  # position-loop gains the tuner chooses (sim.position_ctrl.PositionParams)
# the v0.8 criteria: params position_hold.criteria_v08_ASSUMED (``v08_criteria``) + these, each with its source
V08_SK_RMS_MAX_M = 1.0                    # review v0.8 (9): "quote station keeping as ~1 m RMS / 2.5 m max until a real
                                          # module is measured" - the RMS half of that quote; since the physics review
                                          # (architect 18:10, params criteria_v08_ASSUMED note) the TARGET of the p50 of
                                          # the true RMS over >= 20 seeds per scenario family (``sk_stats_study``), not a
                                          # per-run check: one run's true RMS is one draw of the ASSUMED GPS wander
                                          # (G_calm seeds 70-74: 0.37-1.19 m with NAV-frame RMS 0.15-0.21 m)
V08_SK_STATS_MIN_SEEDS = 20               # params criteria note: ">= 20 seeds per scenario family"
V08_TILT_EXCESS_DEG = 5.0                 # true tilt in HOLD outside the push windows <= the loop's tilt cap + this (= the
                                          # guard's TILT_GUARD_EXCESS_DEG, review v0.8 (4))
V08_TILT_SP_TOL_DEG = 0.1               # the tilt-setpoint cap check allows the documented atan-series error of the reference
                                          # (sim.position_ctrl step 10: atan(x) / x to 0.4 % at 20 deg = 0.08 deg)
V08_PUSH_RULE_NS = 3.0                    # params demo_rules: pushes <= 3 N s; those are judged on the return criterion,
                                          # stronger ones are reported
V09_TOUCHDOWN_VH_MAX_MPS = 0.3            # v0.9 (architect decision g): touchdown horizontal speed over the ground on the VTOL
                                          # configuration (the rigid-foot tip-over between two solid fins is ~0.64 m/s,
                                          # sim.sim6dof.tip_over_vcrit with Vehicle.footprint_radius_m) - judged besides the
                                          # footprint tip-over margin
V09_CLIMB_OVERSHOOT_MAX_M = 0.5           # v0.9 fix round (control review medium 6): the first climb's true-height overshoot
V09_CLIMB_OVERSHOOT_TETHER_MAX_M = 0.3    # over its target; tether / hybrid (pulley clearance, tethered ceiling): 0.3 m - a
                                          # NEW criterion (tightening), judged from this round on
V08_CRITERIA_FIXED: dict[str, float] = {  # kept from v0.7.1 (unchanged)
    "touchdown_vz_max_mps": NOMINAL_CRITERIA["touchdown_vz_max_mps"],
    "touchdown_tilt_max_deg": NOMINAL_CRITERIA["touchdown_tilt_max_deg"],
    "hold_rms_err_max_m": NOMINAL_CRITERIA["hold_rms_err_max_m"],
}
V08_CRITERIA_TEXT = ("v0.8 (position hold, params criteria_v08_ASSUMED): no interlock; LANDED -> SAFE; no false touchdown; "
                     "touchdown |v_z| <= 1.0 m/s, tip-over margin (energy method, lean + ground speed) > 0, tilt <= the "
                     "position loop's wind trim + 5 deg (v0.8.1 keep-trim policy; <= 5 deg in calm air), <= 3 m from the "
                     "hold point (the ground speed is reported); altitude hold RMS <= 0.5 m; station keeping "
                     "in HOLD (true CG vs the hold point, push windows excluded) max <= 3.0 m field / 2.5 m demo; the "
                     "controller's NAV-frame distance RMS <= 0.5 m; tilt setpoint <= the cap, true tilt in HOLD <= cap + "
                     "5 deg; each push <= 3 N s back within 1 m <= 6 s.  The true station-keeping RMS is reported per run "
                     "and judged as the p50 over >= 20 seeds per scenario family (target <= 1.0 m)")
V08_CRITERIA_TEXT_KO = ("v0.8 기준(위치 유지): 인터록 없음; 착지 후 SAFE; 공중 모터 정지 없음; 착지 수직 속도 1.0 m/s 이하, "
                        "넘어짐 여유(에너지법, 기울기+지면 속도) 0 초과, 기울기는 위치 제어기의 바람 보정 기울기 + 5도 이하"
                        "(v0.8.1 트림 유지 착지, 무풍에서는 5도 이하), 유지점에서 3 m 이내(지면 속도는 보고); 고도 "
                        "오차 RMS 0.5 m 이하; 호버링 중 유지점 이탈(밀기 구간 제외) 최대 3.0 m(현장)/2.5 m(시연); 제어기 항법 "
                        "기준 거리 RMS 0.5 m 이하; 기울기 명령 한계 이내; 3 N·s 이하 밀기는 6초 안에 1 m 이내로 복귀. 실제 "
                        "위치 유지 RMS는 비행마다 보고하고, 시나리오 계열별 20개 이상 시드의 중앙값(목표 1.0 m 이하)으로 판정")
MODEL_NOTES_GPS = ("GPS (v0.8): u-blox NEO-7M class model (sim.sim6dof.GpsSim): UBX bytes over the UART incl. the NAV's boot "
                   "configuration; 5 Hz; latency 0.12 s ASSUMED (the NAV compensates 0.22 s); per-axis position wander Gauss-Markov 2.12 m / 200 s "
                   "ASSUMED + 0.2 m white + satellite-set steps; velocity 0.10 m/s white + 0.02 m/s / 10 s. Open sky only - "
                   "no position hold indoors. Station keeping quoted as ~1 m RMS / 2.5 m max until a real module is measured. "
                   "No magnetometer: heading = pad set-up + gyro + GPS-velocity correction.")
MODEL_NOTES_GPS_KO = ("GPS(v0.8): u-blox NEO-7M급 모델 - 5 Hz, 지연 0.12초(가정; NAV 보정값은 0.22초), 위치 오차가 천천히 ~2 m 떠돎(가정). 하늘이 열린 "
                      "실외에서만 위치 유지(실내 불가). 실제 모듈을 측정하기 전까지 위치 유지 성능은 RMS ~1 m / 최대 2.5 m로 "
                      "표기. 지자기 센서 없음: 방향 = 패드 설치 방향 + 자이로 + GPS 속도 보정.")


def v08_criteria(p: dict, profile: str | None) -> dict[str, float]:
    """The v0.8 criteria for a build profile: params mission.vtol.position_hold.criteria_v08_ASSUMED (station keeping max
    by profile, NAV-frame RMS, touchdown over the ground, touchdown from the hold point, the 3 N s push return) + the
    module constants above (never loosened to make a run pass)."""
    c = p["mission"]["vtol"]["position_hold"]["criteria_v08_ASSUMED"]
    prof = profile or str(p["mission"]["vtol"].get("profile", "field"))
    skm = c["station_keeping_max_m"]
    out = {"station_keeping_max_m": float(skm.get(prof, skm.get("field")) if isinstance(skm, dict) else skm),
           "station_keeping_rms_p50_m": V08_SK_RMS_MAX_M,
           "nav_frame_rms_m": float(c.get("controller_nav_frame_rms_m", c["nav_frame_rms_m"])),
           "tip_margin_min_mps": 0.0,
           "touchdown_ground_speed_mps": float(c["touchdown_ground_speed_mps"]),
           "touchdown_from_hold_point_m": float(c["touchdown_from_hold_point_m"]),
           "push_return_within_1m_s": float(c["push_3Ns_return_within_1m_s"]), "push_rule_max_impulse_ns": V08_PUSH_RULE_NS,
           "tilt_excess_over_cap_deg": V08_TILT_EXCESS_DEG}
    out.update(V08_CRITERIA_FIXED)
    if str((p.get("mission") or {}).get("config")) == "vtol":
        out["touchdown_vh_ground_max_mps"] = V09_TOUCHDOWN_VH_MAX_MPS
        out["climb_overshoot_max_m"] = V09_CLIMB_OVERSHOOT_MAX_M
        out["climb_overshoot_tether_max_m"] = V09_CLIMB_OVERSHOOT_TETHER_MAX_M
    return out


# --------------------------------------------------------------------------- #
# the ground operator
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class OpStep:
    """One button press of the VTOL operator script.  The step waits until (a) ``t`` (absolute), (b) the latest
    telemetry shows ``phase`` (if given), and (c) ``after`` seconds have passed since the later of: the previous
    press, the moment the phase condition started to hold.  ON additionally waits for telemetry showing SAFE.
    KILL needs nothing but its time (it is pressed even without telemetry)."""

    cmd: str                        # "on" | "off" | "up" | "down" | "kill"
    t: float = 0.0
    phase: str | None = None        # link_proto.PHASE_NAMES value, e.g. "HOLD"
    after: float = 0.0
    note: str = ""


class VtolOperator:
    """The person at the VTOL ground station: presses the scripted buttons on the latest telemetry only; an ON that
    the vehicle rejected (e.g. not aligned yet) is pressed again after ``RETRY_S`` while the telemetry still shows SAFE
    (at most ``TRIES`` presses).  An ON that was accepted and then ended in SAFE (V7 take-off gate) is not retried."""

    TRIES = 8
    RETRY_S = 0.5                   # ASSUMED operator reaction time before repeating a rejected ON

    def __init__(self, sc: Scenario):
        for s in sc.vtol_script:
            if s.cmd not in ("on", "off", "up", "down", "kill"):
                raise ValueError(f"operator step {s.cmd!r}: not a VTOL button")
            if s.phase is not None and s.phase not in lp.PHASE_NAMES.values():
                raise ValueError(f"operator step phase {s.phase!r} not in {sorted(lp.PHASE_NAMES.values())}")
        self.steps = list(sc.vtol_script)
        self.i = 0
        self.t_cur = 0.0                     # when the current step became current (= the previous press)
        self.t_cond: float | None = None     # since when its phase condition holds
        self.on_rec = None
        self.on_t = 0.0
        self.on_tries = 0
        self.log: list[dict[str, Any]] = []

    def expects_flight(self) -> bool:
        """Another take-off is still to come (the simulation keeps running after a landing)."""
        return any(s.cmd == "on" for s in self.steps[self.i:])

    def _press(self, gs: GroundAssist, cmd: str, now: float, note: str = "") -> Any:
        rec = getattr(gs, cmd)(now)
        self.log.append({"t": now, "cmd": cmd.upper(), "step": self.i, "note": note})
        return rec

    def step(self, gs: GroundAssist, now: float) -> None:
        tl = gs.latest
        r = self.on_rec
        if (r is not None and r.result.startswith("rejected") and tl is not None and tl.state == lp.STATE_SAFE
                and gs.pending is None and now >= self.on_t + self.RETRY_S and self.on_tries < self.TRIES):
            self.on_rec = self._press(gs, "on", now, f"retry after '{r.result}'")
            self.on_t, self.on_tries = now, self.on_tries + 1
            return
        if self.i >= len(self.steps):
            return
        s = self.steps[self.i]
        if now < s.t:
            return
        if s.cmd == "kill":
            if now >= self.t_cur + s.after:
                self._press(gs, "kill", now, s.note)
                self.i, self.t_cur, self.t_cond = self.i + 1, now, None
            return
        if tl is None or not gs.synced:
            return
        t_ref = self.t_cur
        if s.phase is not None:
            if tl.phase_name != s.phase:
                self.t_cond = None
                return
            if self.t_cond is None:
                self.t_cond = now
            t_ref = max(t_ref, self.t_cond)
        if now < t_ref + s.after:
            return
        if s.cmd == "on":
            if tl.state != lp.STATE_SAFE:
                return
            self.on_rec = self._press(gs, "on", now, s.note)
            self.on_t, self.on_tries = now, 1
        else:
            self._press(gs, s.cmd, now, s.note)
        self.i, self.t_cur, self.t_cond = self.i + 1, now, None


# --------------------------------------------------------------------------- #
# gains
# --------------------------------------------------------------------------- #
def attitude_gains(p: dict) -> dict:
    """Attitude gains: params.control <- sim/out/gains.json (the tuned set run_scenarios.py writes), as the renderer's
    development export does."""
    g = gains_from_control(p["control"])
    if os.path.exists(GAINS_JSON):
        with open(GAINS_JSON, "r", encoding="utf-8") as f:
            gj = json.load(f)
        g.update({k: v for k, v in gj.items() if k in g and k not in ("loop_hz", "imu_hz", "baro_hz")})
    return g


def load_vgains(path: str = VTOL_GAINS_JSON) -> tuple[dict | None, str]:
    """Vertical + estimator gains from sim/out/vtol_gains.json (None -> params mission.vtol.gains_ASSUMED and
    nav_estimator *_ASSUMED) and their source.  The dict carries the field gains (VGAIN_KEYS, EGAIN_KEYS) and, under
    "_profiles", per-profile overrides the file gives ("profiles" in the JSON).  A file tuned on another params.json
    still loads; its source string says so."""
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        src = os.path.relpath(path, PROJECT_ROOT).replace("\\", "/")
        sha = (d.get("_meta") or {}).get("params_sha256")
        if sha and sha != params_sha256():
            src += f" (tuned on params sha {sha[:12]}, params.json has changed since)"
        g: dict[str, Any] = {k: float(d[k]) for k in GAIN_KEYS if k in d}
        pfields = set(PositionParams.__dataclass_fields__)
        if isinstance(d.get("position"), dict):                # v0.8: the tuned position-loop gains
            g["_position"] = {k: float(v) for k, v in d["position"].items() if k in pfields}
        prof: dict[str, Any] = {}
        for n, o in (d.get("profiles") or {}).items():
            prof[n] = {k: float(v) for k, v in o.items() if k in GAIN_KEYS}
            if isinstance(o.get("position"), dict):
                prof[n]["_position"] = {k: float(v) for k, v in o["position"].items() if k in pfields}
        if prof:
            g["_profiles"] = prof
        return g, src
    return None, "params.json mission.vtol.gains_ASSUMED / nav_estimator"


def profile_gains(gains: dict | None, profile: str | None) -> dict | None:
    """The gains dict for a build profile: the field gains with the profile's overrides ("_profiles"); v0.8: the
    position gains under "_position" (merged key by key)."""
    if gains is None:
        return None
    g = {k: (dict(v) if isinstance(v, dict) else v) for k, v in gains.items() if k != "_profiles"}
    if profile and profile != "field":
        ov = dict((gains.get("_profiles") or {}).get(profile, {}))
        pos = ov.pop("_position", None)
        g.update(ov)
        if pos:
            g["_position"] = {**(g.get("_position") or {}), **pos}
    return g


def params_sha256() -> str:
    with open(PARAMS_PATH, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def vtol_sensors(estimator: str = VTOL_ESTIMATOR, vibration_g: float = VTOL_ACCEL_VIBRATION_RMS_G_ASSUMED,
                 **kw: Any) -> SensorConfig:
    """The VTOL build's sensor model: the v0.7.1 NAV estimator (gated Mahony) and the ASSUMED accelerometer vibration;
    every other field the SensorConfig default (MPU-6050 / BMP280 models; 2026-11-10)."""
    return SensorConfig(attitude_estimator=estimator, accel_vibration_rms_mps2=vibration_g * G0, **kw)


# --------------------------------------------------------------------------- #
# scenarios
# --------------------------------------------------------------------------- #
def nominal_script(t_on: float = T_ON, n_up: int = 3, up_after: float = 1.0, up_gap: float = 0.5,
                   t_down: float | None = T_DOWN, t_off: float | None = T_OFF) -> tuple[OpStep, ...]:
    """ON; UP x n_up once HOLD shows (first ``up_after`` s after HOLD, then every ``up_gap`` s); DOWN at ``t_down``;
    OFF at ``t_off`` (None: not pressed)."""
    steps = [OpStep("on", t=t_on, note="take off")]
    for i in range(n_up):
        steps.append(OpStep("up", phase="HOLD", after=up_after if i == 0 else up_gap, note=f"UP {i + 1}/{n_up}"))
    if t_down is not None:
        steps.append(OpStep("down", t=t_down, phase="HOLD", note="DOWN"))
    if t_off is not None:
        steps.append(OpStep("off", t=t_off, phase="HOLD", note="land"))
    return tuple(steps)


def demo_script(t_on: float = T_ON, t_off: float | None = DEMO_T_OFF) -> tuple[OpStep, ...]:
    """Demo: ON, hold the demo hover altitude, OFF at ``t_off`` (None: not pressed)."""
    steps = [OpStep("on", t=t_on, note="take off")]
    if t_off is not None:
        steps.append(OpStep("off", t=t_off, phase="HOLD", note="land"))
    return tuple(steps)


def nominal_gusts(p: dict, t0s: tuple[float, ...] = NOMINAL_GUSTS_T0, dirs: tuple[float, ...] = NOMINAL_GUST_DIRS,
                  speed: float | None = None, duration: float | None = None, rise_s: float = 0.0) -> tuple[Gust, ...]:
    """Gusts of params aero.gust_mps_default for aero.gust_duration_s (the ballistic S2 gust), step-shaped."""
    a = p["aero"]
    sp = float(a["gust_mps_default"]) if speed is None else speed
    du = float(a["gust_duration_s"]) if duration is None else duration
    return tuple(Gust(t0, du, sp, d, rise_s) for t0, d in zip(t0s, dirs))


def plant_weak_scale() -> float:
    """Thrust scale of the weak plant: plant T/W = PLANT_WEAK_TW (1.02) on the current vehicle (v0.8: 0.876; v0.7.1 used
    0.85 = T/W 1.02 on the lighter v0.7.1 vehicle)."""
    veh = vehicle()
    return round(PLANT_WEAK_TW * veh.full.mass_kg * G0 / veh.t_max_n, 4)


def plant_no_lift_scale() -> float:
    """v0.9: thrust scale of the take-off-check plant: plant T/W = PLANT_NO_LIFT_TW (0.93, the v0.8 meaning of x0.80) on
    the current vehicle at the params thrust (no voltage term in the controllers' model)."""
    veh = vehicle()
    return round(PLANT_NO_LIFT_TW * veh.full.mass_kg * G0 / veh.t_max_n, 4)


def cad_cg_offset_mm(power: str | None = None) -> tuple[float, float]:
    """The CAD vehicle's lateral CG offset from the thrust axis, body (x, y) [mm] (``sim.vehicle`` full.com_m: 2.57,
    -0.16 mm).  Fix round (physics review): the VTOL scenarios fly it by default - the vanes must hold the thrust moment
    T e with a side force T e / l_v, which drifts the vehicle ~0.28 m/s per mm even with a perfect attitude reference
    (``axis_cg`` gives the what-if with the CG on the axis; the build guide balances it)."""
    c = vehicle(power).full.com_m
    return float(c[0]) * 1e3, float(c[1]) * 1e3


def nav_lever_vtol(power: str | None = None) -> dict[str, Any] | None:
    """v0.9: the NAV's GPS lever-arm constant for the VTOL hardware configuration (the top-cap patch minus the CAD CG of
    the power source) as a ``GpsSim.nav`` override - the firmware round puts it into params (electronics.gps.
    antenna_lever_b_mm per configuration); None on the rocket vehicle (the params / module constant applies)."""
    veh = vehicle(power)
    if getattr(veh, "hardware_config", "rocket") != "vtol" or veh.gps_antenna_mm is None:
        return None
    a, c = veh.gps_antenna_mm, veh.full.com_m * 1e3
    return {"lever_b_m": tuple((float(a[i]) - float(c[i])) * 1e-3 for i in range(3)),
            "lever_source": "v0.9 VTOL top-cap patch - CAD CG (sim plant round; params key in the firmware round)"}


def base_vtol(name: str, title: str, vgains: dict | None, seed: int = 70, **kw: Any) -> Scenario:
    """A VTOL Scenario: the field nominal script, the v0.7.1 sensors (``vtol_sensors``), the gains of its profile, the
    CAD lateral CG offset (``cad_cg_offset_mm``) unless ``cg_offset_mm`` is given.  v0.8: the GPS position-hold build
    (``GPS_DEFAULT``, the position gains of the profile) unless ``gps`` is given (``gps=None``: no GPS - with ``nav_v08``
    unset that is the v0.7.1 build, ``gps_off`` gives the v0.8 build without the GPS)."""
    prof = kw.get("profile")
    vg = profile_gains(vgains, prof)
    pw = kw.get("power_source")
    d: dict[str, Any] = dict(name=name, title=title, mode=lp.MODE_VTOL, seed=seed, t_end=T_END,
                             vgains=vg, vtol_script=nominal_script(), sensors=vtol_sensors(),
                             cg_offset_mm=cad_cg_offset_mm(pw), gps=GPS_DEFAULT, pgains=(vg or {}).get("_position"))
    d.update(kw)
    lev = nav_lever_vtol(pw)
    if d.get("gps") is not None and lev is not None and "lever_b_m" not in (d["gps"].nav or {}):
        d["gps"] = replace(d["gps"], nav={**(d["gps"].nav or {}), **lev})
    if d.get("pushes"):
        d["pushes"] = retarget_pushes(tuple(d["pushes"]), pw)
    return Scenario(**d)


def vtol_push_top_m(power: str | None = None) -> float | None:
    """v0.9: the highest push contact on the VTOL vehicle - the top cap minus 15 mm (ASSUMED pad half-width); None on the
    rocket vehicle (no clipping)."""
    veh = vehicle(power)
    if getattr(veh, "hardware_config", "rocket") != "vtol":
        return None
    return float(veh.layout.z_cap_top) * 1e-3 - 0.015


def retarget_pushes(pushes: tuple, power: str | None = None) -> tuple:
    """v0.9 re-targeting of the v0.8.1 scenarios to the VTOL hardware (475 mm tall): a push above the VTOL vehicle's top
    (the rocket's nose heights 0.50-0.83 m) acts at the top cap (``vtol_push_top_m``); lower pushes are unchanged."""
    top = vtol_push_top_m(power)
    if top is None:
        return pushes
    return tuple(replace(pu, z_body_m=min(float(pu.z_body_m), top)) for pu in pushes)


def gps_off(sc: Scenario, suffix: str = "_gps_off") -> Scenario:
    """What-if: the same scenario flown by the v0.8 build WITHOUT the GPS (the indoor fallback: v0.8 NAV order and
    take-off check, no position hold, no ON gate) - the 'before' of the position hold on the same plant."""
    return replace(sc, name=sc.name + suffix, gps=None, nav_v08=True, takeoff_check=True,
                   title=sc.title + " | WHAT-IF GPS off (the v0.8 build without position hold: drifts with the air)")


def v071_build(sc: Scenario, suffix: str = "_v071") -> Scenario:
    """The same scenario flown by the v0.7.1 build (no GPS, the v0.7.1 NAV order, no take-off check) on the v0.8 plant."""
    return replace(sc, name=sc.name + suffix, gps=None, nav_v08=None, takeoff_check=None,
                   title=sc.title + " | the v0.7.1 build (no GPS) on the v0.8 plant")


def with_gps(sc: Scenario, suffix: str = "", **gkw: Any) -> Scenario:
    """The scenario with the GPS build and receiver fields replaced (``GpsSim`` keyword arguments)."""
    g = replace(sc.gps if sc.gps is not None else GPS_DEFAULT, **gkw)
    return replace(sc, name=sc.name + suffix, gps=g, nav_v08=None, takeoff_check=None)


def scenario_nominal(p: dict, vgains: dict | None, seed: int = 70, **kw: Any) -> Scenario:
    a = p["aero"]
    return base_vtol("V_nominal", f"field nominal: ON -> 5 m, UP x3 -> 8 m, 3 gusts {a['gust_mps_default']:.0f} m/s x "
                                  f"{a['gust_duration_s']:.0f} s in HOLD, DOWN -> 7 m, OFF -> landing; calm air",
                     vgains, seed=seed, gusts=nominal_gusts(p), **kw)


def scenario_demo(p: dict, vgains: dict | None, seed: int = 80, name: str = "D_demo_nominal",
                  pushes: tuple = DEMO_PUSHES, **kw: Any) -> Scenario:
    """The demo profile hover with hand pushes (default: "시연 정상", ``DEMO_PUSHES``), then OFF."""
    d = dict(profile="demo", vtol_script=demo_script(), t_end=DEMO_T_END, pushes=pushes)
    d.update(kw)
    title = d.pop("title", "demo nominal: demo profile, ON -> 1.5 m hover, 5 pushes 6-15 N x 0.2-0.35 s from alternating "
                           "directions above the intake windows, OFF -> landing; no wind")
    return base_vtol(name, title, vgains, seed=seed, **d)


def ideal_att(sc: Scenario, suffix: str = "_ideal_att") -> Scenario:
    """What-if: the same scenario with the attitude estimate replaced by truth + 0.5 deg white noise."""
    return replace(sc, name=sc.name + suffix, sensors=replace(sc.sensors, attitude_estimator=IDEAL_ATT),
                   title=sc.title + " | WHAT-IF attitude reference = truth + 0.5 deg noise (not the NAV estimator)")


def axis_cg(sc: Scenario, suffix: str = "_axis_cg") -> Scenario:
    """What-if: the same scenario with the CG on the thrust axis (a balanced vehicle) instead of the CAD offset."""
    return replace(sc, name=sc.name + suffix, cg_offset_mm=(0.0, 0.0),
                   title=sc.title + " | WHAT-IF CG on the thrust axis (balanced; the v0.8 CAD CG is 0.9 mm off)")


def old_estimator(sc: Scenario, suffix: str = "_pad_gyro") -> Scenario:
    """Comparison: the same scenario with the v0.6.3 / v0.7.0 NAV estimator (pad alignment, then gyro only)."""
    return replace(sc, name=sc.name + suffix, sensors=replace(sc.sensors, attitude_estimator=OLD_EST),
                   title=sc.title + " | COMPARISON: the v0.7.0 NAV estimator (pad alignment + gyro only)")


def low_battery_v_rest(p: dict) -> float:
    """Resting cell voltage of the low-battery scenario: the full-throttle sag (``SimOptions.batt_sag_v_full_throttle``,
    ASSUMED battery model of ``sim.sim6dof``) leaves 0.05 V above battery.min_cell_v, so the take-off climb at u ~1
    passes and I8 (V_cell < min_cell_v) comes in HOLD at 8 m, at an UP climb or when the DoD slope has taken the rest."""
    return round(float(p["battery"]["min_cell_v"]) + SimOptions().batt_sag_v_full_throttle + 0.05, 2)


def scenarios_v071_set(p: dict, vgains: dict | None) -> list[tuple[Scenario, dict[str, Any]]]:
    """The v0.7.1 scenario set with its v0.7.1 expectations (``nominal`` = NOMINAL_CRITERIA, ``push`` = PUSH_CRITERIA);
    ``base_vtol`` gives them the v0.8 GPS build - ``scenarios`` converts the expectations to v0.8.  Expectation keys:
    ``expect`` = the rule that must act ("" = none; "A|B" = either), ``outcome`` = "landed" (LANDED -> SAFE under power)
    or "abort" (motor off by an ABORT rule) or "any", ``info`` = report only (what-if / exploration), ``forbid`` = rules
    that must NOT act, ``title_ko`` = the Korean title (video cards)."""
    a = p["aero"]
    v = p["mission"]["vtol"]
    stuck = float(p["vane"]["max_deflection_mech_deg"])
    gusts = nominal_gusts(p)
    hold_script = nominal_script(t_down=None, t_off=None)        # UP x3 to 8 m, then hold: a fault decides the rest
    out: list[tuple[Scenario, dict[str, Any]]] = []
    nom = scenario_nominal(p, vgains)
    out.append((nom, {"expect": "", "outcome": "landed", "nominal": True,
                      "title_ko": "현장 정상 비행: 5 m 이륙, 8 m까지 상승, 호버링 중 돌풍 3회, 착륙 (무풍)"}))
    out.append((ideal_att(nom), {"expect": "", "outcome": "landed", "nominal": True, "info": True,
                                 "title_ko": "가정 비교: 이상적인 자세 기준(실제+0.5도 잡음)일 때의 현장 정상 비행"}))
    # fix round: the what-ifs both ways - the CAD CG (default) and a balanced CG on the thrust axis
    out.append((axis_cg(ideal_att(nom)), {"expect": "", "outcome": "landed", "nominal": True, "info": True,
                                          "title_ko": "가정 비교: 이상적인 자세 기준 + 무게중심을 추력축에 맞춘 경우"}))
    out.append((axis_cg(nom), {"expect": "", "outcome": "landed", "nominal": True, "info": True,
                               "title_ko": "가정 비교: 무게중심을 추력축에 맞춘 현장 정상 비행 (v0.8 CAD 무게중심은 0.9 mm 벗어남)"}))
    out.append((old_estimator(nom), {"expect": "", "outcome": "landed", "nominal": True, "info": True,
                                     "title_ko": "비교: v0.7.0 자세 추정기(자이로만)로 현장 정상 비행"}))
    wind = replace(nom, name="V_nominal_wind", wind_mps=float(a["wind_mps_default"]), wind_dir_deg=30.0,
                   turbulence_sigma_mps=0.5,
                   title=f"field nominal + steady wind {a['wind_mps_default']:.0f} m/s (params aero.wind_mps_default) + 0.5 "
                         f"m/s turbulence: the drift without position sensing")
    out.append((wind, {"expect": "", "outcome": "landed", "nominal": True,
                       "title_ko": f"현장 정상 비행 + 일정한 바람 {a['wind_mps_default']:.0f} m/s, 난류 0.5 m/s"}))
    out.append((ideal_att(wind), {"expect": "", "outcome": "landed", "nominal": True, "info": True,
                                  "title_ko": "가정 비교: 이상적인 자세 기준 + 바람 3 m/s"}))
    # fix round (physics review, low): the swing and the reversal no longer inherit the nominal step gusts silently
    swing = replace(nom, name="V_wind_swing", wind_mps=4.0, wind_dir_deg=0.0, wind_swing_deg=75.0, wind_swing_period_s=30.0,
                    turbulence_sigma_mps=0.3, gusts=(),
                    title="field mission (no step gusts) in a wind of 4 m/s whose direction swings +-75 deg with a 30 s "
                          "period (+ 0.3 m/s turbulence)")
    out.append((swing, {"expect": "", "outcome": "landed", "nominal": True,
                        "title_ko": "방향이 좌우로 흔들리는 바람: 4 m/s, ±75도, 주기 30초 (돌풍 없음)"}))
    rev = replace(nom, name="V_wind_reversal", wind_mps=4.0, wind_dir_deg=0.0, wind_reversal_t_s=17.0, wind_reversal_s=4.0,
                  turbulence_sigma_mps=0.3, gusts=(),
                  title="field mission (no step gusts) in a 4 m/s wind that turns 180 deg over 4 s at 17 s (in HOLD at 8 m)")
    out.append((rev, {"expect": "", "outcome": "landed", "nominal": True,
                      "title_ko": "바람 방향 반전: 4 m/s 바람이 17초부터 4초 동안 180도 돌아감 (돌풍 없음)"}))
    # a swing at the estimator-drag loop's own period (physics review, medium 2: the campaign's 10-40 s missed it) - the
    # linear loop tau_d s^3 + s^2 + kp s + ki of the flown estimator gains, tau_d net of the vane counter-force
    eg = {k: (vgains or {}).get(k, assumed_gains(p)[k]) for k in ("est_kp", "est_ki")}
    tau_dl = vtol_aero_geometry(vehicle())["drag_time_constant_hover_s"]
    p_loop = estimator_loop(eg, tau_dl)["period_s"] or 60.0
    p_loop = float(round(p_loop))
    slow = replace(swing, name="V_wind_swing_slow", wind_swing_period_s=p_loop,
                   title=f"field mission (no step gusts) in a 4 m/s wind swinging +-75 deg with a {p_loop:.0f} s period - the "
                         f"estimator-drag loop's own period (kp {eg['est_kp']:g}, ki {eg['est_ki']:g}, tau_d {tau_dl:.2f} s)")
    out.append((slow, {"expect": "", "outcome": "landed", "nominal": True, "info": True,
                       "title_ko": f"느리게 흔들리는 바람: 4 m/s, ±75도, 주기 {p_loop:.0f}초 (자세 추정 루프의 고유 주기)"}))
    gsw = replace(nom, name="V_gusty_swing", wind_mps=3.0, wind_dir_deg=60.0, wind_swing_deg=60.0, wind_swing_period_s=20.0,
                  wind_dir_rate_sigma_dps=3.0, wind_speed_mod_frac=0.3, wind_speed_mod_period_s=13.0,
                  turbulence_sigma_mps=0.8, gusts=nominal_gusts(p, speed=6.0, rise_s=0.3),
                  title="field nominal in a gusty swinging wind: 3 m/s +-30 % speed, direction +-60 deg (20 s) + a random "
                        "walk, 0.8 m/s turbulence, three 6 m/s gusts")
    out.append((gsw, {"expect": "", "outcome": "landed", "nominal": True,
                      "title_ko": "돌풍 + 흔들리는 바람: 3 m/s(±30%), 방향 ±60도 + 무작위 변화, 난류 0.8 m/s, 6 m/s 돌풍 3회"}))
    below = replace(nom, name="V_ground_below", ground_z_m=-1.5,
                    title="field nominal with the ground 1.5 m below the pad top (pad on a platform): landing below the pad "
                          "level")
    out.append((below, {"expect": "", "outcome": "landed", "nominal": True,
                        "title_ko": "패드보다 1.5 m 낮은 지면에 착륙 (패드가 단 위에 있는 경우)"}))
    out.append((ideal_att(below), {"expect": "", "outcome": "landed", "nominal": True, "info": True,
                                   "title_ko": "가정 비교: 이상적인 자세 기준 + 패드보다 낮은 지면"}))
    # fix round (physics review HIGH, v0.7.1b control fix): a plant stronger / weaker than the thrust model - the
    # vehicle must land on the ground by the touchdown detector (k_T learnt in flight), never stop the motor in the air
    for nm, ts, ko in (("V_strong_plant", PLANT_STRONG, "추력이 모델보다 25% 강한 기체"),
                       ("V_weak_plant", plant_weak_scale(), "추력이 모델보다 약한 기체 (추력/중량 1.02)")):
        out.append((replace(nom, name=nm, thrust_scale=ts, thrust_voltage_exp=MODEL_RATIO_VEXP,
                            title=f"field nominal with the plant thrust x{ts:.2f} of the model (the controllers keep the "
                                  f"nominal model; v0.7.1b learns k_T in flight): landing by the touchdown detector"),
                    {"expect": "", "outcome": "landed", "vertical": True, "forbid": PLANT_FORBID,
                     "title_ko": f"현장 정상 비행 - {ko}: 착지 감지로 착륙해야 함 (공중 모터 정지 금지)"}))
    out.append((base_vtol("V_link_loss", f"total link loss in HOLD at 8 m from {FAULT_T:.0f} s: V1 LAND after "
                                         f"{v['link_loss_land_s']:.0f} s, autonomous landing", vgains,
                          vtol_script=nominal_script(t_down=None, t_off=T_OFF), gusts=gusts[:1],
                          link_outages=((FAULT_T, float("inf")),)),
                {"expect": "V1", "outcome": "landed", "title_ko": "통신 두절: 2초 뒤 자동 착륙 (V1)"}))
    out.append((base_vtol("V_baro_fail", f"MS5611 silent from {FAULT_T:.0f} s in HOLD at 8 m: I9 baro -> LAND on the "
                                         f"dead-reckoned altitude (v0.7.1: learnt accelerometer bias, time limit V8 / V9)",
                          vgains, vtol_script=hold_script, gusts=gusts[:1], baro_fail_t_s=FAULT_T),
                {"expect": "I9 baro", "outcome": "landed", "forbid": "V5|V6|V9",
                 "title_ko": "기압계 고장: 가속도 적분 고도로 착륙 (시간 제한 V8/V9)"}))
    v_rest = low_battery_v_rest(p)
    out.append((base_vtol("V_low_battery", f"pack at {v_rest:.2f} V/cell at rest (nearly empty; ASSUMED sag / DoD "
                                           f"model): I8 V_cell < {p['battery']['min_cell_v']} V in HOLD -> LAND", vgains,
                          vtol_script=hold_script, options=replace(SimOptions(), batt_v_full_cell=v_rest)),
                {"expect": "I8", "outcome": "landed", "title_ko": "배터리 부족: 셀 전압 저하로 자동 착륙 (I8)"}))
    out.append((base_vtol("V_low_battery_vthrust", f"V_low_battery ({v_rest:.2f} V/cell) with thrust x (V_cell / 3.7 V)^2 "
                                                   f"(what-if: the params thrust model has no voltage term)", vgains,
                          vtol_script=hold_script, options=replace(SimOptions(), batt_v_full_cell=v_rest),
                          thrust_voltage_exp=2.0),
                {"expect": "I8", "outcome": "any", "info": True,
                 "title_ko": "가정: 배터리 부족 + 전압에 따른 추력 감소"}))
    out.append((base_vtol("V_flight_time", f"hold 5 m without OFF: V2 flight time (motor time "
                                           f"{v['max_flight_s'] - v['land_reserve_s']:.0f} s) -> LAND", vgains,
                          vtol_script=(OpStep("on", t=T_ON),)),
                {"expect": "V2", "outcome": "landed", "forbid": "V6",
                 "title_ko": "비행 시간 제한: 모터 30초 후 자동 착륙 (V2)"}))
    n_up_max = int(round((float(v["alt_max_m"]) - float(v["hover_altitude_m"])) / float(v["alt_step_m"])))
    out.append((base_vtol("V_flight_time_alt_max", f"UP x{n_up_max} to alt_max_m = {v['alt_max_m']:.0f} m and hold "
                                                   f"without OFF: V2 LAND at {v['max_flight_s'] - v['land_reserve_s']:.0f} s "
                                                   f"motor time must land before the V6 hard cut-off at "
                                                   f"{v['hard_cutoff_s']:.0f} s", vgains,
                          vtol_script=nominal_script(n_up=n_up_max, t_down=None, t_off=None)),
                {"expect": "V2", "outcome": "landed", "forbid": "V6",
                 "title_ko": f"최고 고도 {v['alt_max_m']:.0f} m에서 비행 시간 제한 착륙 (V6 전에 착지)"}))
    out.append((base_vtol("V_stuck_vane", f"vane 1 servo hard-over to +{stuck:.0f} deg (mechanical stop) at motor start "
                                          f"+ 14 s in HOLD at 8 m: spin / tilt -> ABORT", vgains, vtol_script=hold_script,
                          vane_failure_index=0, vane_failure_t_after_launch_s=14.0, vane_failure_stuck_deg=stuck),
                {"expect": "I3|I2", "outcome": "abort", "title_ko": "베인 서보 고착: 회전/기울기 비상정지"}))
    out.append((base_vtol("V_imu_fail", f"IMU / NAV failure at {FAULT_T + 1:.0f} s in HOLD at 8 m: I9 IMU -> ABORT",
                          vgains, vtol_script=hold_script, imu_fail_t_s=FAULT_T + 1.0),
                {"expect": "I9 IMU", "outcome": "abort", "title_ko": "IMU/항법 컴퓨터 고장: 비상정지 (I9)"}))
    out.append((base_vtol("V_kill", "operator KILL at 18 s in HOLD at 8 m: ABORT, release after "
                                    f"{p['mission']['abort_release_delay_s']} s", vgains,
                          vtol_script=hold_script + (OpStep("kill", t=18.0, note="KILL"),)),
                {"expect": "KILL", "outcome": "abort", "title_ko": "운용자 KILL: 즉시 모터 정지"}))
    for spd in GUST_SWEEP_MPS:
        out.append((base_vtol(f"V_gust_beyond_{spd:.0f}", f"single gust {spd:.0f} m/s for 2 s in HOLD at 8 m "
                                                         f"(exploring the vane authority): what happens", vgains,
                              vtol_script=nominal_script(t_down=None, t_off=T_OFF), gusts=(Gust(16.0, 2.0, spd, 0.0),)),
                    {"expect": "", "outcome": "any", "info": True,
                     "title_ko": f"한계 탐색: {spd:.0f} m/s 돌풍 2초"}))
    # ---- demo profile (v0.7.1): the indoor hover with a person pushing ----
    dem = scenario_demo(p, vgains)
    out.append((dem, {"expect": "", "outcome": "landed", "nominal": True, "push": True,
                      "title_ko": "시연 정상: 1.5 m 호버링, 사람이 막대로 5번 밀기(6~15 N), 착륙"}))
    out.append((ideal_att(dem), {"expect": "", "outcome": "landed", "nominal": True, "push": True, "info": True,
                                 "title_ko": "가정 비교: 이상적인 자세 기준으로 시연 정상"}))
    out.append((axis_cg(ideal_att(dem)), {"expect": "", "outcome": "landed", "nominal": True, "push": True, "info": True,
                                          "title_ko": "가정 비교: 이상적인 자세 기준 + 무게중심을 추력축에 맞춘 시연 정상"}))
    for nm, ts, ko in (("D_strong_plant", PLANT_STRONG, "추력이 모델보다 25% 강한 기체"),
                       ("D_weak_plant", plant_weak_scale(), "추력이 모델보다 약한 기체 (추력/중량 1.02)")):
        out.append((replace(dem, name=nm, thrust_scale=ts, thrust_voltage_exp=MODEL_RATIO_VEXP,
                            title=f"demo nominal (5 pushes) with the plant thrust x{ts:.2f} of the model: landing by the "
                                  f"touchdown detector"),
                    {"expect": "", "outcome": "landed", "vertical": True, "forbid": PLANT_FORBID,
                     "title_ko": f"시연 정상 - {ko}: 착지 감지로 착륙해야 함 (공중 모터 정지 금지)"}))
    z_cg = vehicle().z_cg_m
    for nm, z, ko in (("D_push_cg", z_cg, f"무게중심 높이({z_cg:.2f} m)"), ("D_push_nose", NOSE_Z_M, f"노즈({NOSE_Z_M:.2f} m)")):
        pu = Push(DEMO_PUSH_T0, 0.30, math.pi * SAME_IMPULSE_NS / (2.0 * 0.30), azimuth_deg=0.0, z_body_m=z)
        out.append((scenario_demo(p, vgains, name=nm, pushes=(pu,), vtol_script=demo_script(t_off=18.0),
                                  title=f"demo hover, one push of {SAME_IMPULSE_NS:.1f} N s ({pu.force_n:.1f} N x 0.30 s "
                                        f"half-sine) at z = {z:.3f} m above the exit plane, then OFF"),
                    {"expect": "", "outcome": "any", "info": True, "push": True,
                     "title_ko": f"같은 충격량 {SAME_IMPULSE_NS:.1f} N·s로 {ko}를 밀었을 때"}))
    out.append((scenario_demo(p, vgains, name="D_over_push",
                              pushes=(Push(DEMO_PUSH_T0, 0.40, 25.0, azimuth_deg=0.0, z_body_m=NOSE_Z_M),),
                              vtol_script=demo_script(t_off=None), t_end=20.0,
                              title=f"demo hover, an over-push: 25 N x 0.4 s at the nose (z {NOSE_Z_M:.2f} m) -> the rate / "
                                    f"tilt abort and the fall from the hover"),
                {"expect": "I2|I3", "outcome": "abort", "title_ko": "과도한 밀기: 노즈를 25 N으로 0.4초 → 비상정지, 1.5 m에서 낙하"}))
    out.append((scenario_demo(p, vgains, name="D_over_push_slow",
                              pushes=(Push(DEMO_PUSH_T0, 3.0, 7.0, azimuth_deg=0.0, z_body_m=0.70, profile="trapezoid",
                                           rise_s=0.5),),
                              vtol_script=demo_script(t_off=None), t_end=20.0,
                              title="demo hover, a slow over-push: 7 N held 3 s at z 0.70 m (2.3 N m about the CG): the "
                                    "vehicle leans over -> tilt abort I2 (scratch search: 5 N held at 0.55 m only leans "
                                    "it to ~22 deg)"),
                {"expect": "I2", "outcome": "abort",
                 "title_ko": "과도한 밀기(천천히): 0.7 m 높이를 7 N으로 3초 계속 → 45도 넘게 기울어 비상정지(I2), 1.5 m에서 낙하"}))
    out.append((scenario_demo(p, vgains, name="D_baro_offset_uncal", pushes=(), vtol_script=demo_script(t_off=20.0),
                              baro_thrust_offset_m=BARO_OFFSET_DEMO_M, baro_thrust_comp_m=0.0,
                              title=f"demo hover with the baro reading {BARO_OFFSET_DEMO_M:.1f} m high at hover thrust "
                                    f"(fan pressure on the MS5611), NOT calibrated (comp 0)"),
                {"expect": "", "outcome": "landed", "nominal": True, "info": True,
                 "title_ko": f"기압계 팬 압력 오차 {BARO_OFFSET_DEMO_M:.1f} m, 보정 안 함 → 실제로 1 m 낮게 떠 있음"}))
    out.append((scenario_demo(p, vgains, name="D_baro_offset_cal", pushes=(), vtol_script=demo_script(t_off=20.0),
                              baro_thrust_offset_m=BARO_OFFSET_DEMO_M, baro_thrust_comp_m=BARO_OFFSET_DEMO_M,
                              title=f"demo hover with the baro reading {BARO_OFFSET_DEMO_M:.1f} m high at hover thrust, "
                                    f"calibrated on the tether (comp {BARO_OFFSET_DEMO_M:.1f} m, the reference's "
                                    f"compensation on the current throttle)"),
                {"expect": "", "outcome": "landed", "nominal": True,
                 "title_ko": f"기압계 팬 압력 오차 {BARO_OFFSET_DEMO_M:.1f} m, 테더 시험으로 보정함 (현재 보정식)"}))
    out.append((scenario_demo(p, vgains, name="D_baro_offset_cal_lagged", pushes=(), vtol_script=demo_script(t_off=20.0),
                              baro_thrust_offset_m=BARO_OFFSET_DEMO_M, baro_thrust_comp_m=BARO_OFFSET_DEMO_M,
                              sensors=vtol_sensors(baro_comp_lagged_whatif=True),
                              title="WHAT-IF: D_baro_offset_cal with the compensation on the thrust ratio through the "
                                    "EDF lag, delayed by the baro latency (not the reference's NAV filter)"),
                {"expect": "", "outcome": "landed", "nominal": True, "info": True,
                 "title_ko": "가정 비교: 보정식에 추력 지연과 기압계 지연을 반영했을 때"}))
    return out


DEMO_PUSH_Z_M = 0.38                      # v0.8 demo pushes: in the safe band between the intake-window top (0.327 m) and the
                                          # same distance above the CG (params demo_rules "0.33-0.42 m")
DEMO_PUSH_NS = 3.0                        # params demo_rules: <= 3 N s
POLE_HAND_M = 1.0                         # hand height of a standing person holding the pole (ASSUMED, waist height)
POLE_HAND_LOW_M = 0.3                     # a pole held low (crouching) - a steep upward push (ASSUMED)
GPS_FAULT_T = FAULT_T                     # GPS faults in HOLD at 8 m


def _push_series(t0s: tuple[float, ...], azs: tuple[float, ...], impulse: float = DEMO_PUSH_NS, dur: float = 0.3,
                 z: float = DEMO_PUSH_Z_M, **kw: Any) -> tuple[Push, ...]:
    """Half-sine pushes of the given impulse along the force (horizontal model, or along the pole with hand_height_m)."""
    return tuple(Push(t0, dur, math.pi * impulse / (2.0 * dur), azimuth_deg=a, z_body_m=z, **kw) for t0, a in zip(t0s, azs))


def _v08_exp(exp: dict[str, Any]) -> dict[str, Any]:
    """A v0.7.1 expectation for the v0.8 GPS build: the NOMINAL_CRITERIA judgement becomes the v0.8 criteria (``v08``),
    the attitude push criterion the position push criterion (``push_v08``); the v0.7.1 checks stay in the record."""
    e = dict(exp)
    if e.pop("nominal", False):
        e["v08"] = True
    if e.pop("push", False):
        e["push_v08"] = True
    return e


def scenarios(p: dict, vgains: dict | None) -> list[tuple[Scenario, dict[str, Any]]]:
    """Every exported scenario (v0.8) with its expectation: ``v08`` = judge the v0.8 criteria (``v08_checks``),
    ``push_v08`` = judge the position return of every push <= 3 N s, ``vertical`` = the vertical criteria (plant
    spread), ``expect`` / ``forbid`` / ``outcome`` ("landed", "abort", "safe" = motor stopped on the pad, "no_flight" =
    ON never accepted, "any"), ``reject`` = a rejection reason that must appear (e.g. "GPS"), ``info`` = report only,
    ``title_ko`` = the Korean title.  (1) every v0.7.1 scenario re-run with the GPS build (same names); (2) the v0.8
    station-keeping, push, GPS-fault and take-off-check scenarios; (3) what-ifs: GPS off (the v0.8 build without the GPS
    = the before of the position hold on the same plant), the ideal attitude, the v0.7.1 build."""
    v = p["mission"]["vtol"]
    out: list[tuple[Scenario, dict[str, Any]]] = []
    by: dict[str, Scenario] = {}
    for sc, exp in scenarios_v071_set(p, vgains):
        e = _v08_exp(exp)
        if sc.name in ("V_weak_plant", "D_weak_plant"):
            # v0.8: T/W 1.02 leaves no tilt authority for position hold: the authority rule V14 lands it
            e.pop("vertical", None)
            e.update(expect="V14", outcome="landed", detector=True, forbid=PLANT_FORBID)
            e["title_ko"] = e["title_ko"] + " → 위치 유지 여유 부족(V14)으로 착륙"
        out.append((sc, e))
        by[sc.name] = sc
    nom = by["V_nominal"]
    calm = replace(nom, name="G_calm", gusts=(), title="field mission with GPS position hold, calm air: ON -> 5 m, UP x3 -> "
                                                     "8 m, DOWN -> 7 m, OFF -> landing at the hold point")
    out.append((calm, {"expect": "", "outcome": "landed", "v08": True,
                       "title_ko": "GPS 위치 유지 - 무풍: 5 m 이륙, 8 m까지 상승, 호버링, 유지점에 착륙"}))
    for spd, judged in ((3.0, True), (5.0, True), (8.0, False)):
        sc = replace(calm, name=f"G_wind{spd:.0f}", wind_mps=spd, wind_dir_deg=30.0, turbulence_sigma_mps=0.2 * spd,
                     title=f"field mission with GPS position hold in a steady {spd:.0f} m/s wind + turbulence intensity 0.2"
                           + ("" if judged else " (beyond the field limit 6 m/s mean: exploration)"))
        out.append((sc, {"expect": "", "outcome": "landed", "v08": True, "info": not judged,
                         "title_ko": f"GPS 위치 유지 - 일정한 바람 {spd:.0f} m/s (난류 강도 0.2)"
                                     + ("" if judged else " - 운용 한계 초과 탐색")}))
    lim = v["position_hold"]["wind_limits_mps"]
    wm, wg = float(lim["field_mean"]), float(lim["field_gust"])
    g_lim = tuple(Gust(t0, 3.0, wg - wm, 30.0, rise_s=0.5) for t0 in (14.0, 19.0, 24.0))
    out.append((replace(calm, name="G_wind_limit", wind_mps=wm, wind_dir_deg=30.0, turbulence_sigma_mps=0.2 * wm, gusts=g_lim,
                        title=f"field mission at the operator wind limit: {wm:.0f} m/s mean + turbulence intensity 0.2 + "
                              f"three gusts to {wg:.0f} m/s (params position_hold.wind_limits_mps)"),
                {"expect": "", "outcome": "landed", "v08": True,
                 "title_ko": f"운용 한계 바람: 평균 {wm:.0f} m/s + 난류 + 돌풍 {wg:.0f} m/s 3회"}))
    for spd, judged in ((5.0, True), (7.0, False)):
        gs_ = tuple(Gust(t0, 2.0, spd, d, rise_s=0.3) for t0, d in zip((14.0, 19.0, 24.0), (0.0, 120.0, 240.0)))
        out.append((replace(calm, name=f"G_gusts{spd + 3.0:.0f}", wind_mps=3.0, wind_dir_deg=60.0, turbulence_sigma_mps=0.6,
                            gusts=gs_, title=f"field mission, 3 m/s wind (TI 0.2) + three {spd:.0f} m/s gusts (peaks ~"
                                             f"{spd + 3.0:.0f} m/s) from different directions"),
                    {"expect": "", "outcome": "landed", "v08": True, "info": not judged,
                     "title_ko": f"3 m/s 바람 + 여러 방향 돌풍 {spd:.0f} m/s 3회 (최대 ~{spd + 3.0:.0f} m/s)"
                                 + ("" if judged else " - 한계 탐색")}))
    # ---- demo profile ----
    dem = by["D_demo_nominal"]
    dcalm = replace(dem, name="GD_calm", pushes=(), title="demo profile with GPS position hold, calm: 1.5 m hover, OFF -> "
                                                          "landing at the hold point")
    out.append((dcalm, {"expect": "", "outcome": "landed", "v08": True,
                        "title_ko": "시연 - GPS 위치 유지, 무풍 1.5 m 호버링"}))
    dl = float(lim["demo_mean"])
    out.append((replace(dcalm, name="GD_wind4", wind_mps=dl, wind_dir_deg=200.0, turbulence_sigma_mps=0.2 * dl,
                        title=f"demo profile at the demo wind limit {dl:.0f} m/s mean + turbulence intensity 0.2"),
                {"expect": "", "outcome": "landed", "v08": True,
                 "title_ko": f"시연 운용 한계 바람 {dl:.0f} m/s (난류 강도 0.2)"}))
    band = _push_series((10.0, 14.0, 18.0), (0.0, 120.0, 240.0))
    out.append((replace(dcalm, name="GD_push_band_3Ns", pushes=band,
                        title=f"demo hover, three {DEMO_PUSH_NS:g} N s pushes (pole held level) in the safe band z "
                              f"{DEMO_PUSH_Z_M:.2f} m, 4 s apart from 3 directions (the demo rules)"),
                {"expect": "", "outcome": "landed", "v08": True, "push_v08": True,
                 "title_ko": f"시연 규칙대로 밀기: 수평 막대로 {DEMO_PUSH_NS:g} N·s씩 3번, 4초 간격, 안전 높이"}))
    out.append((replace(dcalm, name="GD_wind4_push", pushes=band, wind_mps=dl, wind_dir_deg=200.0,
                        turbulence_sigma_mps=0.2 * dl,
                        title=f"demo hover at {dl:.0f} m/s wind (TI 0.2) with three {DEMO_PUSH_NS:g} N s pushes"),
                {"expect": "", "outcome": "landed", "v08": True, "push_v08": True,
                 "title_ko": f"바람 {dl:.0f} m/s 속에서 {DEMO_PUSH_NS:g} N·s 밀기 3번"}))
    for nm, hh, ko in (("GD_push_angled", POLE_HAND_M, "서서 든 막대(손 높이 1.0 m)"),
                       ("GD_push_angled_low", POLE_HAND_LOW_M, "낮게 든 막대(손 높이 0.3 m)")):
        out.append((replace(dcalm, name=nm, pushes=_push_series((10.0, 14.0, 18.0), (0.0, 120.0, 240.0),
                                                                hand_height_m=hh, pole_length_m=2.5),
                            title=f"demo hover, three {DEMO_PUSH_NS:g} N s pushes along a 2.5 m pole held at {hh:.1f} m "
                                  f"(the force points up along the pole)"),
                    {"expect": "", "outcome": "landed", "v08": True, "push_v08": True,
                     "title_ko": f"비스듬한 막대로 밀기: {ko}, {DEMO_PUSH_NS:g} N·s씩 3번 (힘이 막대 방향으로 위를 향함)"}))
    out.append((replace(dcalm, name="GD_push_consecutive_rule", pushes=_push_series((10.0, 13.3, 16.6), (90.0, 90.0, 90.0)),
                        title=f"demo hover, three {DEMO_PUSH_NS:g} N s pushes from the same side 3 s apart (the minimum the "
                              f"demo rules allow)"),
                {"expect": "", "outcome": "landed", "v08": True, "push_v08": True,
                 "title_ko": f"같은 방향에서 3초 간격으로 {DEMO_PUSH_NS:g} N·s 연속 밀기 (규칙의 최소 간격)"}))
    out.append((replace(dcalm, name="GD_push_consecutive_fast", pushes=_push_series((10.0, 11.5, 13.0), (90.0, 90.0, 90.0)),
                        title=f"demo hover, three {DEMO_PUSH_NS:g} N s pushes from the same side 1.5 s apart (closer than "
                              f"the rules allow: exploration)"),
                {"expect": "", "outcome": "landed", "v08": True, "push_v08": True, "info": True,
                 "title_ko": "규칙보다 짧게 1.5초 간격 연속 밀기 (탐색)"}))
    out.append((replace(dcalm, name="GD_push_5Ns", pushes=_push_series((10.0, 16.0), (0.0, 180.0), impulse=5.0),
                        title="demo hover, two 5 N s pushes (beyond the 3 N s demo rule: exploration)"),
                {"expect": "", "outcome": "landed", "v08": True, "push_v08": True, "info": True,
                 "title_ko": "규칙 초과 5 N·s 밀기 2번 (탐색)"}))
    # ---- GPS faults ----
    out.append((replace(with_gps(calm, outages=((GPS_FAULT_T, GPS_FAULT_T + 3.0),)), name="G_gps_outage_3s",
                        title=f"field hold, GPS outage (no fix) for 3 s at {GPS_FAULT_T:.0f} s: dead reckoning / attitude "
                              f"only, then the hold resumes (no LAND)"),
                {"expect": "", "forbid": "V11|V13", "outcome": "landed", "v08": True,
                 "title_ko": "GPS 3초 끊김: 관성 추정·자세 유지 후 위치 유지 재개 (착륙하지 않음)"}))
    out.append((replace(with_gps(calm, outages=((GPS_FAULT_T, GPS_FAULT_T + 20.0),)), name="G_gps_outage_20s",
                        wind_mps=3.0, wind_dir_deg=30.0, turbulence_sigma_mps=0.6,
                        title=f"field hold in 3 m/s wind, GPS outage for 20 s from {GPS_FAULT_T:.0f} s: V11 GPS lost -> "
                              f"LAND (the landing drifts with the wind, attitude only)"),
                {"expect": "V11", "outcome": "landed", "info_criteria": True,
                 "title_ko": "GPS 20초 끊김(바람 3 m/s): V11 GPS 상실 → 자동 착륙 (바람에 밀리며 착륙)"}))
    out.append((replace(with_gps(dcalm, outages=((13.0, 33.0),)), name="GD_gps_outage_push",
                        pushes=_push_series((10.0, 14.5), (0.0, 180.0)),
                        title="demo hover, GPS outage from 13 s with pushes before / during it: V11 -> LAND"),
                {"expect": "V11", "outcome": "landed", "info_criteria": True,
                 "title_ko": "시연 중 GPS 끊김 + 밀기: V11 → 자동 착륙"}))
    out.append((replace(with_gps(calm, silent=((GPS_FAULT_T, 1e9),)), name="G_gps_lead_fault",
                        title=f"field hold, the GPS lead fails at {GPS_FAULT_T:.0f} s (no bytes): V11 -> LAND"),
                {"expect": "V11", "outcome": "landed", "info_criteria": True,
                 "title_ko": "GPS 배선 단선(비행 중): V11 → 자동 착륙"}))
    out.append((replace(with_gps(calm, jumps=((GPS_FAULT_T, GPS_FAULT_T + 10.0, 5.0, 0.0),)), name="G_gps_jump5",
                        title=f"field hold, a 5 m multipath position jump for 10 s at {GPS_FAULT_T:.0f} s: rejected, NAV "
                              f"position reset, the vehicle must not follow it"),
                {"expect": "", "forbid": "V11|V13", "outcome": "landed", "v08": True,
                 "title_ko": "GPS 위치 5 m 튐(다중경로) 10초: 기체가 따라가지 않아야 함"}))
    out.append((replace(with_gps(calm, degrade=((GPS_FAULT_T, GPS_FAULT_T + 13.0, 6.0, 6, 2.0),)), name="G_gps_degraded",
                        title=f"field hold, degraded GPS for 13 s at {GPS_FAULT_T:.0f} s: hAcc 6 m, 6 satellites, errors x2"),
                {"expect": "", "forbid": "V11|V13", "outcome": "landed", "v08": True,
                 "title_ko": "GPS 정확도 저하(hAcc 6 m, 위성 6개, 오차 2배) 13초"}))
    out.append((replace(with_gps(calm, vel_bias=((GPS_FAULT_T, 1e9, 1.0, 0.0),)), name="G_gps_vel_bias",
                        title=f"field hold, a 1 m/s GPS velocity bias from {GPS_FAULT_T:.0f} s (the position stays right): "
                              f"the GPS-IMU plausibility rule V13 must stop the runaway -> LAND"),
                {"expect": "V13", "outcome": "landed", "info_criteria": True,
                 "title_ko": "GPS 속도 오차 1 m/s(위치는 정상): 타당성 검사 V13 → 착륙 (폭주 방지)"}))
    out.append((replace(with_gps(calm, sky_blocked=0.34), name="G_sky_blocked_34",
                        title="field mission with a third of the sky blocked (6 of 9 satellites, errors x1.22 ASSUMED)"),
                {"expect": "", "outcome": "landed", "v08": True,
                 "title_ko": "하늘 1/3 가려짐 (위성 6개): 위치 유지"}))
    out.append((replace(with_gps(calm, sky_blocked=0.6), name="G_sky_blocked_60", t_end=12.0,
                        title="field set-up with 60 % of the sky blocked (4 satellites): the ON gate refuses (REJ_GPS)"),
                {"expect": "", "outcome": "no_flight", "reject": "GPS",
                 "title_ko": "하늘 60% 가려짐 (위성 4개): ON 거부 (GPS 준비 안 됨)"}))
    out.append((replace(with_gps(calm, silent=((-1e9, 1e9),)), name="G_gps_silent", t_end=12.0,
                        title="the GPS never answers (lead not connected): the ON gate refuses (REJ_GPS)"),
                {"expect": "", "outcome": "no_flight", "reject": "GPS",
                 "title_ko": "GPS 무응답(연결 안 됨): ON 거부"}))
    cold = replace(with_gps(calm, ttff_s=27.0), name="G_no_fix_at_on", t_end=80.0,
                   vtol_script=(OpStep("on", t=1.0, note="ON before the fix"),) + nominal_script(t_on=40.0, t_down=None,
                                                                                               t_off=None)[:1]
                   + (OpStep("off", phase="HOLD", after=8.0, note="land"),),
                   title="cold start (TTFF 27 s, data sheet): ON at 1 s refused (REJ_GPS, retried), ON at 40 s with the fix "
                         "-> flight")
    out.append((cold, {"expect": "", "outcome": "landed", "reject": "GPS",
                       "title_ko": "냉시동(위성 고정 27초): 1초 ON 거부 → 40초 ON 정상 비행"}))
    # params heading.tolerance_deg (architect 18:10 after the physics review: 45 -> 15 deg): set-up errors within it are
    # judged, larger ones are explorations (the yaw correction is margin, not the primary means)
    tol = float(v["position_hold"]["heading"]["tolerance_deg"])
    for err in (15.0, 30.0, 45.0, 90.0):
        judged = err <= tol
        out.append((replace(with_gps(nom, heading_err_deg=err), name=f"G_heading_{err:.0f}",
                            title=f"field nominal (3 gusts) with the vehicle set up {err:.0f} deg off the pad azimuth (no "
                                  f"magnetometer: the NAV corrects it from GPS velocity changes)"
                                  + ("" if judged else f" - beyond the {tol:g} deg set-up tolerance: exploration")),
                    {"expect": "", "outcome": "landed", "v08": True, "info": not judged,
                     "title_ko": f"패드 방향 설치 오차 {err:.0f}도 (지자기 센서 없음, GPS로 보정)"
                                 + ("" if judged else f" - 허용치 {tol:g}도 초과 탐색")}))
    for err in (15.0, 30.0):
        judged = err <= tol
        out.append((replace(with_gps(by["D_demo_nominal"], heading_err_deg=err), name=f"GD_heading_{err:.0f}",
                            title=f"demo nominal (5 pushes) with a {err:.0f} deg pad heading error"
                                  + ("" if judged else f" - beyond the {tol:g} deg set-up tolerance: exploration")),
                    {"expect": "", "outcome": "landed", "v08": True, "push_v08": True, "info": not judged,
                     "title_ko": f"시연 정상 + 패드 방향 오차 {err:.0f}도" + ("" if judged else f" - 허용치 {tol:g}도 초과 탐색")}))
    out.append((replace(with_gps(nom, vel_white_mps=0.3), name="G_vel_noise_03",
                        title="field nominal with 0.3 m/s GPS velocity noise (EDF vibration on the crystal-only NEO-6M, the "
                              "review's case)"),
                {"expect": "", "outcome": "landed", "v08": True,
                 "title_ko": "GPS 속도 잡음 0.3 m/s (진동): 방향 보정 영향"}))
    out.append((replace(with_gps(calm, latency_s=0.30, latency_jitter_s=0.05), name="G_latency_030",
                        title="field mission, true GPS latency 0.30 s + 0.05 s jitter while the NAV assumes 0.22 s"),
                {"expect": "", "outcome": "landed", "v08": True,
                 "title_ko": "GPS 지연 0.30초(+흔들림) - 항법은 0.22초로 가정"}))
    # physics-review fix (medium 7): the GPS latency is ASSUMED 0.22 s and only the iTOW jitter is removed on board - a
    # receiver 0.23 s slower than params (true 0.45 s) shows how much a mis-set latency costs (exploration: the remedy is
    # the bench measurement of the module actually bought, params latency_s; hardware checklist before free flight)
    for base, nm, ko in ((calm, "G_latency_045", "현장 무풍"), (by["D_demo_nominal"], "GD_push_latency_045", "시연 밀기 5회")):
        out.append((replace(with_gps(base, latency_s=0.45, latency_jitter_s=0.05), name=nm,
                            title=base.title + " | true GPS latency 0.45 s + 0.05 s jitter while the NAV assumes 0.22 s "
                                               "(a mis-set latency: exploration)"),
                    {"expect": "", "outcome": "landed", "v08": True, "push_v08": bool(base.pushes), "info": True,
                     "title_ko": f"GPS 지연 0.45초(항법은 0.22초로 가정, 잘못 설정된 지연 탐색): {ko}"}))
    out.append((replace(with_gps(calm, multipath_rate_hz=1.0 / 15.0), name="G_multipath",
                        title="field mission with random multipath jumps (1-4 m for 2-10 s, every ~15 s)"),
                {"expect": "", "forbid": "V11|V13", "outcome": "landed", "v08": True,
                 "title_ko": "무작위 다중경로 위치 튐 (15초마다 1~4 m)"}))
    m8 = neo_m8n()
    for base, nm, ko in ((calm, "G_m8n_calm", "무풍"), (by.get("V_nominal_wind", calm), "G_m8n_wind3", "바람 3 m/s")):
        out.append((replace(base, name=nm, gps=m8, title=base.title + " | NEO-M8N (upgrade path: 10 Hz, 0.12 s latency)"),
                    {"expect": "", "outcome": "landed", "v08": True,
                     "title_ko": f"NEO-M8N 수신기(업그레이드): {ko}"}))
    # ---- take-off check ----
    nl_ = plant_no_lift_scale()
    out.append((replace(calm, name="G_takeoff_weak", thrust_scale=nl_, t_end=30.0, thrust_voltage_exp=MODEL_RATIO_VEXP,
                        title=f"take-off check: plant thrust x{nl_:.2f} (T/W {PLANT_NO_LIFT_TW} < 1 at every pack voltage) - "
                              f"no climb at full throttle "
                              f"-> V15 motor off, SAFE, no release"),
                {"expect": "V15", "outcome": "safe",
                 "title_ko": "이륙 검사: 추력 부족(추력/중량 < 1) → 모터 정지, SAFE (V15)"}))
    # physics-review fix (medium 4): an UNCALIBRATED thrust-proportional baro offset (true 1.0 m, params
    # baro_thrust_comp_m 0) raised the NAV h on the pad and defeated the check (motor 19.8 s on the pad); h0 is now
    # latched after the ramp + the settle time
    out.append((replace(calm, name="G_takeoff_weak_baro_uncal", thrust_scale=nl_, t_end=30.0,
                        thrust_voltage_exp=MODEL_RATIO_VEXP,
                        baro_thrust_offset_m=BARO_OFFSET_DEMO_M, baro_thrust_comp_m=0.0,
                        title=f"take-off check with an uncalibrated baro fan offset (true {BARO_OFFSET_DEMO_M:g} m, "
                              f"compensation 0): plant thrust x{nl_:.2f} (T/W < 1) -> V15 motor off, SAFE"),
                {"expect": "V15", "outcome": "safe",
                 "title_ko": f"이륙 검사 + 보정 안 된 기압계 팬 오차 {BARO_OFFSET_DEMO_M:g} m: 추력 부족 → 모터 정지, SAFE (V15)"}))
    # ---- what-ifs ----
    for name in ("V_nominal", "G_calm", "G_wind3", "G_wind5", "V_wind_swing", "V_wind_swing_slow", "V_wind_reversal",
                 "D_demo_nominal", "GD_calm", "GD_wind4", "GD_push_band_3Ns", "GD_push_angled"):
        sc = next(s for s, _ in out if s.name == name)
        ko = next(e["title_ko"] for s, e in out if s.name == name)
        out.append((gps_off(sc), {"expect": "", "outcome": "landed", "v08": True, "push_v08": bool(sc.pushes),
                                  "info": True, "title_ko": "가정 비교(GPS 끔): " + ko}))
    for name in ("G_wind5", "GD_push_band_3Ns"):
        sc = next(s for s, _ in out if s.name == name)
        ko = next(e["title_ko"] for s, e in out if s.name == name)
        out.append((ideal_att(sc), {"expect": "", "outcome": "landed", "v08": True, "push_v08": bool(sc.pushes),
                                    "info": True, "title_ko": "가정 비교(이상적인 자세 기준): " + ko}))
    for name in ("V_nominal", "D_demo_nominal"):
        sc = by[name]
        out.append((v071_build(sc), {"expect": "", "outcome": "landed", "nominal": True, "push": bool(sc.pushes),
                                     "info": True, "title_ko": "v0.7.1 빌드(GPS 없음)로 같은 비행: "
                                                               + next(e["title_ko"] for s, e in out if s.name == name)}))
    out += scenarios_v09(p, vgains, by)
    return out


# --------------------------------------------------------------------------- #
# v0.9 (VTOL hardware round): thrust envelope, tether / hybrid power
# --------------------------------------------------------------------------- #
T_ENV_LOW_KGF = 2.14                      # architect decision b: T/W 1.3 of the battery VTOL at 22.2 V (the envelope's low end)
T_ENV_STRESS_KGF = 1.9                    # the stress band's low end (exercises the take-off check)
T_END_OF_PACK_KGF = 2.46                  # T_max at 22.2 V that keeps T/W 1.3 at the end of the pack (sizing loaded_pack)
TETHER_TEST_T_OFF = 24.0                  # tether test: OFF before V2 (30 s motor time) with the demo build profile
TETHER_EVENT_T = 13.0                     # power events in HOLD (the hover is reached at ~6-8 s)
W_PROOF_N = 300.0                         # params power.design_load_N_ASSUMED.proof (the arrest load limit, W3)
W_CONTACT_MAX_MPS = 1.0                   # tether proposal W2 / W3: no ground contact faster than this after a power loss
W_LAND_AFTER_LOSS_S = 0.2                 # W2: LAND <= 0.1 s after V_tether < threshold (+ the 40 ms persistence + a tick)
W_BUFFER_MIN_CELL_V = 3.4                 # W2 / W_BUF: the buffer stays >= 3.4 V / cell under load


def tether_script(t_on: float = T_ON, alt: str = "1.5", t_off: float | None = TETHER_TEST_T_OFF) -> tuple[OpStep, ...]:
    """Tether test (demo build profile, 1.5 m hover): ``alt`` "1.0" = DOWN once (alt_step 0.5 m), "1.5" = hold, "2.0" / "2.5"
    / "3.0" = UP x1 / x2 / x3; OFF at ``t_off`` (None: not pressed)."""
    steps = [OpStep("on", t=t_on, note="take off")]
    n = {"1.0": -1, "1.5": 0, "2.0": 1, "2.5": 2, "3.0": 3}[alt]
    for i in range(abs(n)):
        steps.append(OpStep("down" if n < 0 else "up", phase="HOLD", after=1.0 if i == 0 else 0.5,
                            note=f"{'DOWN' if n < 0 else 'UP'} {i + 1}/{abs(n)}"))
    if t_off is not None:
        steps.append(OpStep("off", t=t_off, phase="HOLD", note="land"))
    return tuple(steps)


def scenario_tether(p: dict, vgains: dict | None, name: str, title: str, source: str = "tether", indoor: bool = True,
                    alt: str = "1.5", t_off: float | None = TETHER_TEST_T_OFF, tether: TetherConfig | None = None,
                    **kw: Any) -> Scenario:
    """A tethered test / demo flight (architect decision f): demo build profile, the overhead rig (``TetherConfig`` from
    params power.tether_plant_ASSUMED), ``power`` 'tether' (battery-less) or 'hybrid'; ``indoor`` = the v0.8 build
    without the GPS (no position hold indoors), else the GPS build outdoors."""
    d: dict[str, Any] = dict(profile="demo", vtol_script=tether_script(alt=alt, t_off=t_off),
                             t_end=(t_off + 20.0) if t_off is not None else 40.0, power_source=source,
                             tether=tether if tether is not None else TetherConfig.from_params(p))
    if indoor:
        d.update(gps=None, nav_v08=True, takeoff_check=True)
    seed = int(kw.pop("seed", 90))
    d.update(kw)
    return base_vtol(name, title, vgains, seed=seed, **d)


def tether_checks(m: dict[str, Any], want: dict[str, Any]) -> dict[str, bool | None]:
    """v0.9 tether / hybrid expectations (tether proposal sim.scenarios W2-W5): ``in_flight`` the power event came while the
    vehicle flew under power more than 0.3 m up (else the scenario did not test it - FAIL); ``arrest`` catch-line max <= W_PROOF_N;
    ``contact`` every ground contact after the power loss <= W_CONTACT_MAX_MPS (none is fine); ``land_after_loss`` LAND
    <= W_LAND_AFTER_LOSS_S after the loss; ``no_trip`` the supply never tripped (W4: the SUPPLY_LIMIT rule lands before
    the latch); ``buffer`` the buffer >= W_BUFFER_MIN_CELL_V per cell under load; ``no_abort`` no ABORT (W5)."""
    out: dict[str, bool | None] = {}
    if want.get("in_flight"):
        # the scenario reached its power event with the vehicle flying under power (else every check below is vacuous)
        out["event_in_flight"] = bool(m.get("powered_at_power_loss")) and (m.get("h_at_power_loss_m") or 0.0) > 0.3
    if want.get("arrest"):
        x = m.get("tether_catch_max_n")
        out["arrest_le_proof"] = None if x is None else x <= W_PROOF_N
    if want.get("contact"):
        x = m.get("contact_speed_max_after_loss_mps")
        out["contact_after_loss_le_1mps"] = True if x is None else x <= W_CONTACT_MAX_MPS
    if want.get("land_after_loss"):
        x = m.get("land_after_power_loss_s")
        out["land_after_loss"] = bool(m.get("landed_before_power_loss")) or (x is not None and x <= W_LAND_AFTER_LOSS_S)
    if want.get("no_trip"):
        out["supply_not_tripped"] = not m.get("supply_tripped")
    if want.get("buffer"):
        x = m.get("buffer_cell_v_min")
        out["buffer_cell_v_ge_3v4"] = True if x is None else x >= W_BUFFER_MIN_CELL_V
    if want.get("no_abort"):
        out["no_abort"] = m.get("t_abort_s") is None
    return out


def scenarios_v09(p: dict, vgains: dict | None, by: dict[str, Scenario]) -> list[tuple[Scenario, dict[str, Any]]]:
    """The v0.9 named scenarios (VTOL hardware configuration): the thrust envelope on the battery vehicle, the tether test
    (battery-less: taut / slack feed, indoors without the GPS and outdoors with it, snag, unplug at 1 m and 2 m, cut,
    supply trip, the supply limit), the hybrid demo (hover, crossover, cut / unplug on the buffer, buffer low).  Rules the
    v0.8.1 supervisor does not have yet (W_LOST, W_SAG, W_BUF; tether proposal firmware.rules_ASSUMED_thresholds) are
    EXPECTED here - the old controller is expected to fail them (the baseline shows what breaks)."""
    out: list[tuple[Scenario, dict[str, Any]]] = []
    nom, dem, calm = by["V_nominal"], by["D_demo_nominal"], None
    # ---- thrust envelope (battery) ----
    for nm, kgf, ko, info in (("V9_thrust_low", T_ENV_LOW_KGF, "추력 하한 2.14 kgf", False),
                              ("V9_thrust_end_of_pack", T_END_OF_PACK_KGF, "팩 말기 T/W 1.3 기준 2.46 kgf", False),
                              ("V9_thrust_stress", T_ENV_STRESS_KGF, "스트레스 1.9 kgf (T/W 1.15)", True)):
        out.append((replace(nom, name=nm, t_max_kgf=kgf,
                            title=f"field nominal on the battery VTOL with the plant T_max {kgf:g} kgf at 22.2 V (thrust "
                                  f"envelope; the controllers keep the 3.09 kgf model)"),
                    {"expect": "", "outcome": "landed", "v08": True, "info": info,
                     "title_ko": f"현장 정상 비행 - {ko}"}))
    out.append((replace(dem, name="D9_thrust_low", t_max_kgf=T_ENV_LOW_KGF,
                        title=f"demo nominal (5 pushes) with the plant T_max {T_ENV_LOW_KGF:g} kgf"),
                {"expect": "", "outcome": "landed", "v08": True, "push_v08": True,
                 "title_ko": f"시연 정상 - 추력 하한 {T_ENV_LOW_KGF:g} kgf"}))
    vr = low_battery_v_rest(p)
    out.append((replace(nom, name="V9_low_pack_thrust_low", t_max_kgf=T_ENV_LOW_KGF, vtol_script=nominal_script(t_down=None, t_off=None),
                        options=replace(SimOptions(), batt_v_full_cell=3.75),
                        title=f"field hold on a pack at 3.75 V/cell rest with T_max {T_ENV_LOW_KGF:g} kgf: the voltage sag "
                              f"cuts the thrust (x(V / 22.2)^1.61) - I8 / authority"),
                {"expect": "", "outcome": "landed", "info": True,
                 "title_ko": f"전압이 낮은 팩(3.75 V/셀) + 추력 하한 {T_ENV_LOW_KGF:g} kgf"}))
    # ---- tether test (battery-less) ----
    tc = TetherConfig.from_params(p)
    out.append((scenario_tether(p, vgains, "W_taut_hover", "tether test (battery-less, indoors, no GPS): constant-tension "
                                "feed, 1.5 m hover, OFF -> landing"),
                {"expect": "", "outcome": "landed", "v08": True, "title_ko": "유선 시험(배터리 없음, 실내): 장력 급전, 1.5 m 호버링"}))
    out.append((scenario_tether(p, vgains, "W_slack_hover", "tether test (battery-less, indoors): slack feed (0.5 m paid "
                                "out, the cable hangs as a catenary), 1.5 m hover", tether=replace(tc, feed="slack")),
                {"expect": "", "outcome": "landed", "v08": True, "title_ko": "유선 시험: 느슨한 급전(케이블 늘어짐), 1.5 m 호버링"}))
    out.append((scenario_tether(p, vgains, "W_taut_hover_1m_gps", "tether test outdoors with the GPS: 1.0 m hover (DOWN "
                                "once), position hold", indoor=False, alt="1.0"),
                {"expect": "", "outcome": "landed", "v08": True,
                 "title_ko": "유선 시험(실외, GPS): 1.0 m 호버링, 위치 유지"}))
    out.append((scenario_tether(p, vgains, "W_alt_max_3m", "tether test: UP x3 to the demo alt_max 3.0 m and back down "
                                "(the catch line follows)", alt="3.0"),
                {"expect": "", "outcome": "landed", "v08": True, "title_ko": "유선 시험: 최고 3.0 m까지 상승 후 착륙"}))
    out.append((scenario_tether(p, vgains, "W_snag", "tether test: a snag / handler jerk 20 N for 0.2 s at 30 deg from the "
                                "vertical in the 1.5 m hover (proposal W5)",
                                tether=replace(tc, snag_t_s=TETHER_EVENT_T, snag_azimuth_deg=45.0)),
                {"expect": "", "outcome": "landed", "forbid": "I2|I3", "tether": {"no_abort": True},
                 "title_ko": "유선 시험: 줄 걸림/당김 20 N 0.2초 (W5)"}))
    for alt, nm in (("1.0", "W_unplug_1m"), ("2.0", "W_unplug_2m")):
        out.append((scenario_tether(p, vgains, nm, f"tether test (battery-less): the XT90-S pulls out at {TETHER_EVENT_T:.0f} s "
                                    f"in a {alt} m hover - no power: the vehicle falls into the catch line + bungee (W3)",
                                    alt=alt, t_off=None, t_end=TETHER_EVENT_T + 8.0,
                                    power_events=(PowerEvent(TETHER_EVENT_T, "unplug"),)),
                    {"expect": "", "outcome": "any", "tether": {"in_flight": True, "arrest": True, "contact": True},
                     "title_ko": f"유선 시험(배터리 없음): {alt} m에서 커넥터 빠짐 → 낙하, 안전줄+번지로 잡힘"}))
    out.append((scenario_tether(p, vgains, "W_cut_2m", "tether test (battery-less): the conductors are cut at 2.0 m (power "
                                "and the cable feed gone, the catch line intact)", alt="2.0", t_off=None,
                                t_end=TETHER_EVENT_T + 8.0, power_events=(PowerEvent(TETHER_EVENT_T, "cut"),)),
                {"expect": "", "outcome": "any", "tether": {"in_flight": True, "arrest": True, "contact": True},
                 "title_ko": "유선 시험(배터리 없음): 2.0 m에서 전선 절단 → 낙하, 안전줄로 잡힘"}))
    out.append((scenario_tether(p, vgains, "W_supply_trip", "tether test (battery-less): the supply trips (E-stop / fault) in "
                                "the 1.5 m hover", t_off=None, t_end=TETHER_EVENT_T + 8.0,
                                power_events=(PowerEvent(TETHER_EVENT_T, "supply_trip"),)),
                {"expect": "", "outcome": "any", "tether": {"in_flight": True, "arrest": True, "contact": True},
                 "title_ko": "유선 시험: 전원 장치 차단(비상정지) → 낙하, 안전줄로 잡힘"}))
    out.append((scenario_tether(p, vgains, "W_sag_limit", "tether test, weak fan (P2) at 2.14 kgf and a long thin tether "
                                "(loop x2.5): the climb draws more than the 60 A limit -> the supply's constant current, "
                                "V_bus sags; the SUPPLY_LIMIT rule (W_SAG) must land before the 5 s latch (proposal W4)",
                                t_max_kgf=T_ENV_LOW_KGF, fan_power_model=1.0, alt="2.0",
                                power_events=(PowerEvent(0.0, "r_loop", 2.5),)),
                {"expect": "W_SAG|SAG|SUPPLY", "outcome": "landed", "tether": {"no_trip": True},
                 "title_ko": "유선 시험: 약한 팬 + 긴 전선 → 전원 전류 제한, 전압 강하 → 차단 전에 착륙해야 함 (W4)"}))
    out.append((scenario_tether(p, vgains, "W_supply_sag", "tether test: the supply set point sags 25.2 -> 21.0 V over 4 s "
                                "in the hover (a failing supply / mains dip): W_SAG must land", t_off=None,
                                t_end=TETHER_EVENT_T + 16.0, power_events=(PowerEvent(TETHER_EVENT_T, "sag", 21.0, 4.0),)),
                {"expect": "W_SAG|SAG|SUPPLY", "outcome": "landed",
                 "title_ko": "유선 시험: 전원 전압이 25.2 → 21.0 V로 떨어짐 → 착륙해야 함"}))
    # ---- hybrid demo (tether + 650 mAh buffer behind ideal diodes) ----
    out.append((scenario_tether(p, vgains, "H_hover", "hybrid demo (tether + 6S 650 mAh buffer, indoors): 1.5 m hover with "
                                "pushes, OFF -> landing; P1 fan: the tether alone carries the hover", source="hybrid",
                                pushes=_push_series((10.0, 14.0, 18.0), (0.0, 120.0, 240.0))),
                {"expect": "", "outcome": "landed", "v08": True, "push_v08": True,
                 "title_ko": "하이브리드 시연(실내): 1.5 m 호버링 + 밀기 3번, 전선만으로 호버"}))
    out.append((scenario_tether(p, vgains, "H_crossover", "hybrid demo, weak fan (P2) at 2.14 kgf: the hover draws more than "
                                "the 40 A crossover - the buffer shares the load in the hover", source="hybrid",
                                t_max_kgf=T_ENV_LOW_KGF, fan_power_model=1.0),
                {"expect": "", "outcome": "landed", "v08": True, "tether": {"buffer": True},
                 "title_ko": "하이브리드: 약한 팬 → 호버 중에도 버퍼 배터리가 같이 방전 (교차 40 A 초과)"}))
    for ev, nm, alt in (("cut", "H_cut_2m", "2.0"), ("unplug", "H_unplug_1m", "1.0")):
        out.append((scenario_tether(p, vgains, nm, f"hybrid demo: the tether is {'cut' if ev == 'cut' else 'unplugged'} at "
                                    f"{TETHER_EVENT_T:.0f} s in a {alt} m hover - the buffer carries the vehicle; W_LOST must "
                                    f"start the LAND <= 0.1 s after V_tether drops (proposal W2)", source="hybrid", alt=alt,
                                    t_off=None, t_end=TETHER_EVENT_T + 25.0, power_events=(PowerEvent(TETHER_EVENT_T, ev),)),
                    {"expect": "W_LOST|TETHER", "outcome": "landed", "tether": {"in_flight": True, "land_after_loss": True, "buffer": True},
                     "title_ko": f"하이브리드: {alt} m에서 전선 {'절단' if ev == 'cut' else '빠짐'} → 버퍼로 즉시 착륙해야 함 (W2)"}))
    out.append((scenario_tether(p, vgains, "H_buffer_low", "hybrid demo with the buffer at 3.55 V/cell rest and the tether "
                                "unplugged in the hover: the buffer falls below 3.4 V/cell under load - W_BUF / I8 lands",
                                source="hybrid", t_off=None, t_end=TETHER_EVENT_T + 25.0,
                                power=PowerConfig.from_params(p, source="hybrid", buf_v_rest_cell=3.55),
                                power_events=(PowerEvent(TETHER_EVENT_T, "unplug"),)),
                {"expect": "W_BUF|W_LOST|I8|TETHER", "outcome": "landed", "tether": {"in_flight": True, "land_after_loss": True},
                 "title_ko": "하이브리드: 버퍼 전압 낮음(3.55 V/셀) + 전선 빠짐 → 착륙해야 함"}))
    return out


def scenario_names(p: dict) -> list[str]:
    return [sc.name for sc, _ in scenarios(p, None)]


# --------------------------------------------------------------------------- #
# verdicts
# --------------------------------------------------------------------------- #
def nominal_checks(m: dict[str, Any], crit: dict[str, float] = NOMINAL_CRITERIA) -> dict[str, bool]:
    """Each NOMINAL_CRITERIA check (False also when the quantity does not exist, e.g. no touchdown).  touchdown_vh is
    judged on the speed relative to the mean wind (``touchdown_vh_air``)."""
    def le(x: Any, lim: float) -> bool:
        return x is not None and abs(x) <= lim
    gusts = [g for g in m.get("gusts", []) if g.get("in_flight")]
    return {
        "no_interlock": not m.get("interlock_codes"),
        "landed_then_safe": m.get("t_landed_s") is not None and m.get("t_safe_after_landed_s") is not None
                            and m.get("t_abort_s") is None,
        "no_false_touchdown": not m.get("false_touchdown", False),
        "touchdown_vz": le(m.get("touchdown_vz"), crit["touchdown_vz_max_mps"]),
        "touchdown_vh": le(m.get("touchdown_vh_air"), crit["touchdown_vh_max_mps"]),
        "touchdown_tilt": le(m.get("touchdown_tilt_deg"), crit["touchdown_tilt_max_deg"]),
        "hold_rms": le(m.get("hold_rms_err_m"), crit["hold_rms_err_max_m"]),
        "gust_recovery": all(g.get("recovery_s") is not None and g["recovery_s"] <= crit["gust_recovery_max_s"]
                             for g in gusts),
    }


def push_checks(m: dict[str, Any], crit: dict[str, float] = PUSH_CRITERIA) -> dict[str, bool]:
    """``push_recovered`` for every push (False for a push that came while the vehicle was not flying under power)."""
    return {f"push_{r['i'] + 1}": push_recovered(r, crit["push_recovery_max_s"]) for r in m.get("pushes", [])}


V081_TD_TRIM_EXCESS_DEG = 5.0             # v0.8.1 touchdown (params position_hold.touchdown_policy, architect 21:00): the
                                          # tilt at the contact <= the position loop's wind trim + this (secondary)
V081_CALM_WIND_MPS = 0.5                  # "calm air" for the v0.7 touchdown tilt <= 5 deg check: no wind sample above this
                                          # in the whole run (wind_max_mps: mean + swing + turbulence + gusts)


def v08_checks(m: dict[str, Any], crit: dict[str, float]) -> dict[str, bool | None]:
    """The v0.8 criteria (``v08_criteria``): True / False, None = not applicable (e.g. no HOLD window to judge the station
    keeping in).  A required quantity that does not exist (no touchdown) is False.

    Touchdown (v0.8.1, params mission.vtol.position_hold.touchdown_policy, architect 2026-10-21 21:00 - the keep-trim
    policy): PRIMARY the energy-method tip-over margin > 0 (lean AND ground speed, the params tipover geometry: the
    physical criterion); SECONDARY the tilt at the contact <= the position loop's wind trim (its integrator as a tilt,
    ``touchdown_trim_deg``; 0 in a build without the position loop) + ``V081_TD_TRIM_EXCESS_DEG``; the v0.7 tilt <= 5 deg
    only in calm air (``wind_max_mps`` <= ``V081_CALM_WIND_MPS``, else None).  The ground speed over the ground (params
    criteria_v08 touchdown_ground_speed_mps 0.5) is REPORTED (the verdict text, ``touchdown_vh_ground``), no longer judged
    on its own: the tip-over margin judges the speed together with the lean (a 0.5 m/s drift leaning away from the motion
    is safe, 0.3 m/s leaning 8 deg toward it is not)."""
    def le(x: Any, lim: float, required: bool = True) -> bool | None:
        if x is None:
            return False if required else None
        return abs(x) <= lim
    cap = m.get("tilt_cap_deg")
    tilt_td = m.get("touchdown_tilt_deg")
    trim_td = m.get("touchdown_trim_deg")
    calm = m.get("wind_max_mps") is not None and m["wind_max_mps"] <= V081_CALM_WIND_MPS
    extra: dict[str, bool | None] = {}
    if "climb_overshoot_max_m" in crit:
        # v0.9 fix round (control review medium 6; physics review HIGH 2 / medium 4): the first climb's overshoot, a
        # landing confirmed while hanging on the catch line, the slack-fed cable loop at the intake windows
        lim = crit["climb_overshoot_tether_max_m" if m.get("power_source") in ("tether", "hybrid") else "climb_overshoot_max_m"]
        x = m.get("climb_overshoot_transient_m")
        extra["climb_overshoot"] = None if x is None else x <= lim
        extra["not_hanging"] = None if m.get("landed_hanging") is None else not m["landed_hanging"]
        extra["cable_clear_of_intake"] = None if m.get("cable_at_intake_s") is None else m["cable_at_intake_s"] <= 0.0
    return extra | {
        "no_interlock": not m.get("interlock_codes"),
        "landed_then_safe": m.get("t_landed_s") is not None and m.get("t_safe_after_landed_s") is not None
                            and m.get("t_abort_s") is None,
        "no_false_touchdown": not m.get("false_touchdown", False),
        "touchdown_vz": le(m.get("touchdown_vz"), crit["touchdown_vz_max_mps"]),
        # v0.9 (VTOL configuration): the ground speed at the contact <= 0.3 m/s (None on the rocket vehicle)
        "touchdown_vh_ground": (le(m.get("touchdown_vh"), crit["touchdown_vh_ground_max_mps"])
                                if "touchdown_vh_ground_max_mps" in crit else None),
        # primary (physics-review fix medium 8 + architect 21:00): the energy-method tip-over margin > 0
        "touchdown_tip_margin": (False if m.get("touchdown_tip_margin_mps") is None
                                 else m["touchdown_tip_margin_mps"] > crit["tip_margin_min_mps"]),
        # secondary: no lean beyond the wind trim + 5 deg at the contact
        "touchdown_tilt_trim": (False if tilt_td is None else
                                abs(tilt_td) <= abs(trim_td or 0.0) + V081_TD_TRIM_EXCESS_DEG),
        # the v0.7 tilt <= 5 deg in calm air only (in wind the vehicle keeps its trim to the contact)
        "touchdown_tilt": le(tilt_td, crit["touchdown_tilt_max_deg"]) if calm else None,
        "touchdown_from_hold": le(m.get("touchdown_from_hold_m"), crit["touchdown_from_hold_point_m"]),
        "hold_rms": le(m.get("hold_rms_err_m"), crit["hold_rms_err_max_m"]),
        "station_keeping_max": le(m.get("sk_max_m"), crit["station_keeping_max_m"], required=False),
        # the true RMS is judged as a statistic (p50 over >= 20 seeds, ``sk_stats_study``), the controller per run on
        # the NAV-frame RMS (params criteria note, architect 18:10)
        "nav_rms": le(m.get("nav_rms_m"), crit["nav_frame_rms_m"], required=False),
        "tilt_sp_within_cap": None if cap is None else le(m.get("tilt_sp_max_deg"), float(cap) + V08_TILT_SP_TOL_DEG,
                                                           required=False),
        "tilt_within_cap": None if cap is None else le(m.get("tilt_max_hold_deg"), float(cap) + crit["tilt_excess_over_cap_deg"],
                                                       required=False),
    }


def push_v08_checks(m: dict[str, Any], crit: dict[str, float]) -> dict[str, bool | None]:
    """v0.8 push criterion (params criteria_v08 push_3Ns_return_within_1m_s): each push <= ``push_rule_max_impulse_ns``
    (the impulse along the force) in flight, no interlock / ABORT in its window, the TRUE CG back within 1 m of its
    pre-push position <= ``push_return_within_1m_s`` after the push ends; stronger pushes: None (reported, not judged); a
    push that came while the vehicle was not flying under power: False."""
    out: dict[str, bool | None] = {}
    for r in m.get("pushes", []):
        key = f"push_{r['i'] + 1}"
        if (r.get("impulse_ns") or 0.0) > crit["push_rule_max_impulse_ns"] + 1e-9:
            out[key] = None
            continue
        ret = r.get("return_1p0m_s")
        out[key] = bool(r.get("in_flight") and not r.get("interlock_codes") and not r.get("abort")
                        and ret is not None and ret <= crit["push_return_within_1m_s"])
    return out


def _rule_hit(m: dict[str, Any], want: str) -> bool:
    names = " ".join(m.get("interlocks", []))
    return any(w in names for w in want.split("|"))


def judge(m: dict[str, Any], exp: dict[str, Any]) -> tuple[str, str]:
    """PASS / FAIL / INFO with the reason, from the scenario expectation (``scenarios``)."""
    if m.get("nan_abort"):
        return "FAIL", "numerical failure"
    parts = []
    ok = True
    if exp.get("nominal"):
        c = nominal_checks(m)
        bad = [k for k, v in c.items() if not v]
        ok = not bad
        parts.append("all nominal criteria met" if ok else "failed: " + ", ".join(bad))
    if exp.get("vertical"):
        # fix round (plant spread scenarios): the vertical criteria (VERTICAL_CHECKS) and LANDED by a touchdown signature
        # of the detector (not a supervisor timeout); the horizontal ones are reported, judged by the nominal scenarios
        c = nominal_checks(m)
        bad = [k for k in VERTICAL_CHECKS if not c[k]]
        det = m.get("landed_by") in DETECTOR_SIGNATURES
        if not det:
            bad.append("landed_by_detector")
        ok = ok and not bad
        parts.append(("vertical criteria met" if not bad else "failed: " + ", ".join(bad))
                     + f"; k_T hold {_f(m.get('kt_hold_mean'), 3)} (plant x{_f(m.get('kt_plant_ratio_nominal'), 2)}), "
                       f"hold RMS {_f(m.get('hold_rms_err_m'), 3)} m")
    if exp.get("detector"):
        # an expected LAND rule (e.g. V14): the landing itself - by a touchdown signature, |v_z| <= 1 m/s, no false
        # touchdown (the rule that acted is judged by expect)
        bad = [k for k, v in (("landed_by_detector", m.get("landed_by") in DETECTOR_SIGNATURES),
                              ("touchdown_vz", m.get("touchdown_vz") is not None
                               and abs(m["touchdown_vz"]) <= NOMINAL_CRITERIA["touchdown_vz_max_mps"]),
                              ("no_false_touchdown", not m.get("false_touchdown", False))) if not v]
        ok = ok and not bad
        parts.append("landing by the detector, |v_z| <= 1 m/s: " + ("met" if not bad else "failed: " + ", ".join(bad)))
    if exp.get("push"):
        c = push_checks(m)
        bad = [k for k, v in c.items() if not v]
        ok = ok and not bad
        parts.append(f"pushes recovered {sum(c.values())}/{len(c)}" + ("" if not bad else " (not: " + ", ".join(bad) + ")"))
    if exp.get("v08") or exp.get("info_criteria") or exp.get("push_v08"):
        crit = v08_criteria(vehicle().params, m.get("profile"))
    if exp.get("v08") or exp.get("info_criteria"):
        c = v08_checks(m, crit)
        bad = [k for k, v in c.items() if v is False]
        na = [k for k, v in c.items() if v is None]
        if exp.get("v08"):
            ok = ok and not bad
            parts.append(("all v0.8 criteria met" if not bad else "v0.8 failed: " + ", ".join(bad))
                         + (f" (n/a: {', '.join(na)})" if na else ""))
        else:
            parts.append("v0.8 criteria reported, not judged (a fault case): " + ("all met" if not bad else
                                                                                 "not met: " + ", ".join(bad)))
    if exp.get("push_v08"):
        c = push_v08_checks(m, crit)
        judged = {k: v for k, v in c.items() if v is not None}
        bad = [k for k, v in judged.items() if not v]
        ok = ok and not bad
        parts.append(f"pushes <= {crit['push_rule_max_impulse_ns']:g} N s back within 1 m <= "
                     f"{crit['push_return_within_1m_s']:g} s: {sum(judged.values())}/{len(judged)}"
                     + ("" if not bad else " (not: " + ", ".join(bad) + ")")
                     + (f"; {len(c) - len(judged)} stronger push(es) reported" if len(c) > len(judged) else ""))
    tw = exp.get("tether")
    if tw:
        c = tether_checks(m, tw)
        bad = [k for k, v in c.items() if v is False]
        ok = ok and not bad
        parts.append("tether: " + (", ".join(f"{k} {v}" for k, v in c.items()) or "-")
                     + f" (catch max {_f(m.get('tether_catch_max_n'), 1)} N, contact after loss "
                       f"{_f(m.get('contact_speed_max_after_loss_mps'))} m/s, LAND after loss {_f(m.get('land_after_power_loss_s'))} s, "
                       f"buffer min {_f(m.get('buffer_cell_v_min'))} V/cell)")
    rej = exp.get("reject")
    if rej:
        hit = any(rej in str(r.get("reason", "")) for r in m.get("rejections", []))
        ok = ok and hit
        parts.append(f"ON rejected '{rej}': {hit} ({len(m.get('rejections', []))} rejection(s))")
    want = exp.get("expect", "")
    if want and want != "KILL":
        hit = _rule_hit(m, want)
        ok = ok and hit
        parts.append(f"rule {want}: {'acted' if hit else 'did NOT act'} ({' '.join(m.get('interlocks', [])) or 'none'})")
    if want == "KILL":
        hit = m.get("abort_reason") is not None and "KILL" in m["abort_reason"]
        ok = ok and hit
        parts.append(f"KILL -> ABORT: {hit}")
    forbid = exp.get("forbid", "")
    if forbid:
        hit = _rule_hit(m, forbid)
        ok = ok and not hit
        parts.append(f"{forbid} did {'ACT' if hit else 'not act'}")
    oc = exp.get("outcome", "any")
    if oc == "landed":
        good = bool(m.get("landed_normally") and m.get("t_safe_after_landed_s") is not None)
        ok = ok and good
        parts.append(f"landed under power -> LANDED -> SAFE: {good}" + (f" (by {m['landed_by']})" if m.get("landed_by") else ""))
        if m.get("false_touchdown"):
            ok = False
            parts.append(f"FALSE touchdown: motor off {_f(m.get('height_at_landed_m'))} m above the surface")
    elif oc == "abort":
        good = m.get("t_abort_s") is not None
        ok = ok and good
        parts.append(f"ABORT (motor off, release): {good}")
    elif oc == "safe":
        # v0.8 take-off check: the motor ran and stopped (SAFE) without LANDED, ABORT or release
        good = (m.get("t_motor_start_s") is not None and m.get("t_abort_s") is None and m.get("t_released_s") is None
                and m.get("final_state") == "SAFE")
        ok = ok and good
        parts.append(f"motor stopped on the pad -> SAFE, no ABORT / release: {good}")
    elif oc == "no_flight":
        good = m.get("t_motor_start_s") is None and m.get("final_state") == "SAFE"
        ok = ok and good
        parts.append(f"never started the motor (ON refused): {good}")
    if m.get("gps_build"):
        parts.append(f"touchdown v_z {_f(m.get('touchdown_vz'))} m/s, {_f(m.get('touchdown_vh'))} m/s over the ground, "
                     f"tilt {_f(m.get('touchdown_tilt_deg'), 1)} deg (lean toward the motion "
                     f"{_f(m.get('touchdown_lean_toward_deg'), 1)} deg, tip-over margin "
                     f"{_f(m.get('touchdown_tip_margin_mps'))} m/s), {_f(m.get('touchdown_from_hold_m'))} m from the hold "
                     f"point; station keeping RMS {_f(m.get('sk_rms_m'))} / max {_f(m.get('sk_max_m'))} m (NAV frame RMS "
                     f"{_f(m.get('nav_rms_m'))} m)")
    else:
        parts.append(f"touchdown v_z {_f(m.get('touchdown_vz'))} m/s, horizontal {_f(m.get('touchdown_vh_air'))} m/s vs the "
                     f"mean wind ({_f(m.get('touchdown_vh'))} m/s vs the ground), tilt {_f(m.get('touchdown_tilt_deg'), 1)} "
                     f"deg, {_f(m.get('touchdown_distance_m'), 1)} m from the pad")
    verdict = "INFO" if exp.get("info") else ("PASS" if ok else "FAIL")
    return verdict, "; ".join(parts)


def _f(x: Any, nd: int = 2) -> str:
    return "-" if x is None else f"{x:.{nd}f}"


# --------------------------------------------------------------------------- #
# renderer log export
# --------------------------------------------------------------------------- #
def export_npz(res: SimResult, path: str, veh: Vehicle, extra_meta: dict | None = None) -> dict[str, Any]:
    """Write the renderer log (contract in the module docstring) and return its meta."""
    L, sc, vr = res.log, res.scenario, res.vtol or {}
    p = veh.params
    n = L["t"].shape[0]
    recs = (res.link or {}).get("records", [])
    cmds = sorted(((r["t_first_tx"] if r.get("t_first_tx") is not None else r["t_queued"]), r["name"]) for r in recs)
    vpd = vr.get("vp", {})
    # poster: the first fault / interlock / push if any, else the middle of HOLD
    m_hold = np.flatnonzero(L["phase"] == lp.PHASE_HOLD)
    poster = float(L["t"][m_hold[len(m_hold) // 2]]) if m_hold.size else float(L["t"][n // 2])
    if sc.pushes:
        poster = min(sc.pushes[0].t0_s + 0.5 * sc.pushes[0].duration_s, float(L["t"][-1]))
    for te, e in res.events:
        if e.startswith("FAULT") or "-> ABORT" in e or "-> LAND:" in e:
            poster = min(float(te) + 0.5, float(L["t"][-1]))
            break
    ground_z = float(vr.get("ground_z_m", -0.11))
    m = p["mission"]
    meta = {"scenario": sc.name, "title": sc.title, "seed": sc.seed, "mode": lp.MODE_VTOL, "profile": vr.get("profile"),
            "params_sha256": params_sha256(), "notes": MODEL_NOTES + (" " + sc.notes if sc.notes else ""),
            "notes_ko": MODEL_NOTES_KO, "pos_ref": "exit_plane", "z_cg_m": float(vr.get("z_cg_full_m", veh.z_cg_m)),
            "t_max_n": float(veh.t_max_n),
            "limits": {"u_max": float(vpd.get("u_max", 1.0)), "hover_throttle": float(vpd.get("u_hover", veh.hover_throttle)),
                       "tilt_abort_deg": float(m["tilt_abort_deg"]), "tilt_rate_abort_dps": float(m["tilt_rate_abort_dps"]),
                       "spin_rate_abort_dps": float(m["spin_rate_abort_dps"]),
                       "ceiling_m": float(res.vtol["sp"]["ceiling_m"]) if res.vtol else None,
                       "geofence_m": float(res.vtol["sp"]["v_geofence_m"]) if res.vtol else None},
            "poster_t": poster, "source": "sim/vtol_scenarios.py (sim.sim6dof.FlightSim, mode vtol, v0.7.1)",
            "estimator": sc.sensors.attitude_estimator, "estimator_params": vr.get("estimator_params"),
            "vgains": {k: vpd.get(k) for k in VGAIN_KEYS}, "sim_runtime_s": round(res.runtime_s, 2),
            "t_liftoff": res.t_liftoff, "t_touchdown": res.t_touchdown, "aero": vr.get("aero"), "timing": vr.get("timing"),
            "plant": vr.get("plant"), "plant_v09": vr.get("plant_v09"),
            "hardware_config": (vr.get("plant_v09") or {}).get("hardware_config"),
            "power_source": (vr.get("plant_v09") or {}).get("power_source"),
            "wind": {"wind_mps": sc.wind_mps, "wind_dir_deg": sc.wind_dir_deg, "swing_deg": sc.wind_swing_deg,
                     "swing_period_s": sc.wind_swing_period_s, "dir_rate_sigma_dps": sc.wind_dir_rate_sigma_dps,
                     "reversal_t_s": sc.wind_reversal_t_s, "reversal_s": sc.wind_reversal_s,
                     "speed_mod_frac": sc.wind_speed_mod_frac, "turbulence_sigma_mps": sc.turbulence_sigma_mps}}
    gps_on = "hold_true" in L
    meta["gps_on"] = bool(gps_on)
    meta["build"] = vr.get("build")
    if gps_on:
        g = vr.get("gps") or {}
        cfgr = g.get("configurator") or {}
        meta["notes"] = meta["notes"] + " " + MODEL_NOTES_GPS
        meta["notes_ko"] = MODEL_NOTES_KO + " " + MODEL_NOTES_GPS_KO
        meta["gps"] = {"receiver": (g.get("receiver") or {}).get("receiver"), "module_hw": cfgr.get("hw"),
                       "cfg_ok": cfgr.get("cfg_ok"), "rate_hz": (g.get("nav_params") or {}).get("rate_hz"),
                       "latency_assumed_s": (g.get("nav_params") or {}).get("latency_s"),
                       "heading_err_deg": g.get("heading_err_deg"), "pad_azimuth_deg": g.get("pad_azimuth_deg"),
                       "hold_captures": g.get("hold_captures"), "tilt_cap_deg": (g.get("pos_params") or {}).get("tilt_cap_deg"),
                       "fence_m": (g.get("pos_params") or {}).get("fence_m"),
                       "quote": "station keeping ~1 m RMS / 2.5 m max until a real module is measured; open sky only"}
        meta["source"] = "sim/vtol_scenarios.py (sim.sim6dof.FlightSim, mode vtol, v0.8 GPS position hold)"
    if extra_meta:
        meta.update(extra_meta)
    crit = dict(meta.get("criteria") or {})
    crit.setdefault("gust_band_deg", NOMINAL_CRITERIA["gust_band_deg"])
    if gps_on:
        cv = p["mission"]["vtol"]["position_hold"]["criteria_v08_ASSUMED"]
        crit.setdefault("station_keeping_max_m", cv["station_keeping_max_m"])
        crit.setdefault("touchdown_from_hold_point_m", cv["touchdown_from_hold_point_m"])
        crit.setdefault("touchdown_ground_speed_mps", cv["touchdown_ground_speed_mps"])
    meta["criteria"] = crit
    data = {
        "t": L["t"], "pos": exit_plane_positions(res), "vel": L["vel"], "q": L["q"], "delta": L["delta"],
        "throttle": L["throttle"], "thrust": L["thrust"], "state": L["state"].astype(np.int8),
        "phase": L["phase"].astype(np.int8), "h_target": L["h_tgt"], "h_ref": L["h_ref"], "h_est": L["h_est"],
        "tilt_deg": L["tilt_deg"], "wind": L["wind"], "gust_active": L["gust_active"], "wind_mean": L["wind_mean"],
        "ground_z": np.array(ground_z),
        "event_t": np.array([t for t, _ in res.events], dtype=float),
        "event_text": np.array([e for _, e in res.events], dtype=str),
        "cmd_t": np.array([t for t, _ in cmds], dtype=float), "cmd_name": np.array([c for _, c in cmds], dtype=str),
        "meta": np.array(json.dumps(_clean(meta), ensure_ascii=False)),
        "h_true": L["h_true"], "vz_est": L["vz_est"], "v_cell": L["v_cell"], "vc_sat": L["vc_sat"],
        "surface_z": L["surface_z"], "tilt_ctrl_deg": L["tilt_ctrl_deg"], "tilt_est_err_deg": L["tilt_est_err_deg"],
        "reason": L["reason"], "vc_td_by": L["vc_td_by"], "alt_bias": L["alt_bias"],
    }
    if "vc_k_thrust" in L:                          # fix round: the v0.7.1b thrust learning
        data.update(vc_k_thrust=L["vc_k_thrust"], vc_low_alt=L["vc_low_alt"])
    # v0.9 renderer contract v6 (every VTOL export; tools/sim_video.py reads them): hardware_config and power_source
    # always; tether_attach_body_m (the eye bolt, vehicle frame from the exit-plane centre) whenever the vehicle has the top
    # cap; with a tether rig: tether_anchor_m (world, the origin of pos), per sample tether_tension_N (catch line + cable
    # at the vehicle) and tether_connected (the power cable plugged in / not cut at the vehicle - False after an unplug or
    # a cut; the catch line stays); with a supply (tether / hybrid): supply_V / supply_A (supply terminal, D1 current; the
    # renderer's names supply_v_V / supply_i_A too), supply_state (0 CV, 1 current limit, 2 off / lost), tether_power (bool);
    # hybrid: buffer_soc, buffer_v_V, buffer_i_A (discharge > 0).  Keys that do not apply are absent (not NaN).
    pv = vr.get("plant_v09") or {}
    data.update(hardware_config=np.array(str(pv.get("hardware_config") or getattr(veh, "hardware_config", "rocket"))),
                power_source=np.array(str(pv.get("power_source") or "battery")))
    att = pv.get("tether_attach_body_m")
    if att is not None:
        data["tether_attach_body_m"] = np.array(att, dtype=float)
    if pv.get("tether_anchor_m") is not None and "tether_tension_n" in L:
        data.update(tether_anchor_m=np.array(pv["tether_anchor_m"], dtype=float), tether_tension_N=L["tether_tension_n"],
                    tether_catch_N=L["tether_catch_n"], tether_cable_N=L["tether_cable_n"])
        data["tether_connected"] = (L["cable_attached"] if "cable_attached" in L
                                    else np.ones(L["t"].shape[0], dtype=bool))
    if "supply_v" in L and pv.get("power_source") in ("tether", "hybrid"):
        st = np.where(L["supply_state"] == 1, 0, np.where(L["supply_state"] == 2, 1, 2)).astype(np.int8)
        data.update(supply_V=L["supply_v"], supply_A=L["supply_a"], supply_v_V=L["supply_v"], supply_i_A=L["supply_a"],
                    supply_state=st, tether_power=L["tether_connected"])
    if "buffer_soc" in L and np.isfinite(L["buffer_soc"]).any():
        data.update(buffer_soc=L["buffer_soc"], buffer_v_V=L["buffer_v"], buffer_i_A=L["buffer_a"])
    if "v_bus" in L:
        data.update(v_bus=L["v_bus"], i_bus=L["i_bus"], thrust_limited=L["thrust_limited"])
    if "delta_servo" in L:
        data["delta_servo"] = L["delta_servo"]
    if sc.pushes:
        data.update(push_force=L["push_force"], push_point=L["push_point"], push_active=L["push_active"])
    if "hold_true" in L:
        # v0.8 renderer contract v5 (tools/sim_video.py): the TRUE CG at the hold capture per sample (z ignored, NaN while
        # none), the NAV's hold point in the world frame, GPS status, the NAV CG estimate, the tilt setpoint
        nn = L["t"].shape[0]

        def xy0(a: np.ndarray) -> np.ndarray:
            o = np.full((nn, 3), np.nan)
            o[:, :2] = a
            o[np.isfinite(a[:, 0]), 2] = 0.0
            return o
        data.update(hold_point=xy0(L["hold_true"]), hold_point_est=xy0(L["hold_nav"]), gps_ok=L["gps_ok"],
                    gps_sats=L["gps_sats"].astype(np.int16), gps_hacc_m=L["gps_hacc_m"],
                    gps_fix=L["gps_fix"].astype(np.int8), pos_est=L["pos_est"], tilt_sp_deg=L["tilt_sp_deg"],
                    tilt_sp=L["tilt_sp"], pos_mode=L["pos_mode"], pos_dist=L["pos_dist"], psi_err_deg=L["psi_err_deg"],
                    gps_err=L["gps_err"], gps_resets=L["gps_resets"], pos_requests=L["pos_requests"],
                    pos_limited=L["pos_limited"], pos_guard=L["pos_guard"], vel_est_h=L["vel_est_h"],
                    gps_gate_ok=L["gps_gate_ok"])
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    # written to a temporary file and moved into place (the renderers read these logs while scenarios are re-run)
    tmp = path + f".tmp{os.getpid()}.npz"
    np.savez_compressed(tmp, **data)
    for k in range(50):
        try:
            os.replace(tmp, path)
            break
        except PermissionError:                     # a reader has it open (Windows): retry
            if k == 49:
                raise
            time.sleep(0.2)
    return meta


def _clean(o: Any) -> Any:
    """JSON-safe copy (inf / nan -> strings, numpy -> python, tuples -> lists)."""
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating, float)):
        f = float(o)
        return f if math.isfinite(f) else str(f)
    if isinstance(o, np.ndarray):
        return _clean(o.tolist())
    return o


# --------------------------------------------------------------------------- #
# plots
# --------------------------------------------------------------------------- #
def plot_vtol(res: SimResult, m: dict[str, Any], verdict: tuple[str, str], path: str) -> None:
    """Six panels: altitude, vertical speed, throttle, tilt, horizontal track, wind - with the phase bands; gusts red,
    pushes purple."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    L, sc = res.log, res.scenario
    t = L["t"]
    fig, axes = plt.subplots(3, 2, figsize=(14, 11.5), facecolor=SURFACE)
    title = f"{sc.name}: {sc.title}"
    fig.suptitle("\n".join([title[:160], f"{verdict[0]} - {verdict[1]}"[:230]]), fontsize=9, color=INK, x=0.01,
                 ha="left")
    phase = L["phase"]
    bands = {lp.PHASE_SPOOL: "#fdf3d6", lp.PHASE_CLIMB: "#e3eefb", lp.PHASE_HOLD: "#e3f6ec", lp.PHASE_LAND: "#fde9dc",
             lp.PHASE_LANDED: "#efefec"}

    def style(ax: Any, ttl: str, unit: str) -> None:
        ax.set_facecolor(SURFACE)
        for s_ in ("top", "right"):
            ax.spines[s_].set_visible(False)
        for s_ in ("left", "bottom"):
            ax.spines[s_].set_color(MUTED)
        ax.tick_params(colors=INK2, labelsize=8)
        ax.grid(True, color=GRID, lw=0.6)
        ax.set_title(ttl, fontsize=9.5, color=INK, loc="left")
        ax.set_ylabel(unit, fontsize=8, color=INK2)

    def phase_bands(ax: Any) -> None:
        k0 = 0
        for k in range(1, len(t) + 1):
            if k == len(t) or phase[k] != phase[k0]:
                c = bands.get(int(phase[k0]))
                if c:
                    ax.axvspan(t[k0], t[k - 1], color=c, lw=0, zorder=0)
                k0 = k
        for g in m.get("gusts", []):
            ax.axvspan(g["t0_s"], g["t1_s"], color=PALETTE[7], alpha=0.12, lw=0, zorder=0)
        for pu in sc.pushes:
            ax.axvspan(pu.t0_s, pu.t1_s, color=PALETTE[6], alpha=0.25, lw=0, zorder=0)

    ax = axes[0, 0]
    style(ax, "Altitude above the pad top (bands: spool, climb, hold, land, landed; red = gust, purple = push)", "m")
    phase_bands(ax)
    ax.plot(t, L["h_true"], color=PALETTE[0], lw=1.8, label="truth (CG rise)")
    ax.plot(t, L["h_est"], color=PALETTE[1], lw=1.0, label="NAV estimate")
    ax.plot(t, L["h_ref"], color=PALETTE[2], lw=1.2, ls="--", label="controller reference")
    ax.plot(t, L["h_tgt"], color=INK2, lw=1.0, ls=":", label="target")
    ax.plot(t, L["surface_z"], color=MUTED, lw=1.0, label="surface below")
    for td in (res.vtol or {}).get("touchdowns", []):
        ax.axvline(td["t"], color=PALETTE[7], lw=0.8, ls="--")
    ax.legend(fontsize=7.5, frameon=False, ncol=3)
    ax = axes[0, 1]
    style(ax, "Vertical speed", "m/s")
    phase_bands(ax)
    ax.plot(t, L["vel"][:, 2], color=PALETTE[0], lw=1.6, label="v_z truth")
    ax.plot(t, L["vz_est"], color=PALETTE[1], lw=1.0, label="v_z estimate")
    ax.plot(t, L["vz_cmd"], color=PALETTE[2], lw=1.0, ls="--", label="v_z command")
    ax.legend(fontsize=7.5, frameon=False, ncol=3)
    ax = axes[1, 0]
    style(ax, "Throttle (ESC command) and its feed-forward; marks = saturated", "-")
    phase_bands(ax)
    ax.plot(t, L["throttle"], color=PALETTE[0], lw=1.2, label="u (FCU command)")
    ax.plot(t, np.where(L["motor_on"], L["u_ff"], np.nan), color=PALETTE[1], lw=1.0, label="u_ff = u_hover cos^-1/2")
    hov = m.get("u_hover_model")
    if hov:
        ax.axhline(hov, color=MUTED, lw=0.8, ls="--")
        ax.text(t[0], hov, f" u_hover {hov:.3f}", fontsize=7, color=INK2, va="bottom")
    sat = L["vc_sat"] != 0
    if sat.any():
        ax.plot(t[sat], L["throttle"][sat], "|", color=PALETTE[7], ms=6, label="saturated")
    ax.set_ylim(-0.02, 1.05)
    ax.legend(fontsize=7.5, frameon=False, ncol=3, loc="lower center")
    ax = axes[1, 1]
    style(ax, "Tilt (truth) and the controller's tilt (estimate)", "deg")
    phase_bands(ax)
    nm = 50                                                  # 0.2 s running mean: the truth_noise error is white
    err = np.convolve(L["tilt_est_err_deg"], np.ones(nm) / nm, mode="same") if len(t) > nm else L["tilt_est_err_deg"]
    ax.plot(t, err, color=PALETTE[3], lw=1.0, zorder=1, label="estimate error (0.2 s mean)")
    ax.plot(t, L["tilt_ctrl_deg"], color=PALETTE[1], lw=1.0, zorder=2, label="estimated tilt")
    ax.plot(t, L["tilt_deg"], color=PALETTE[0], lw=1.4, zorder=3, label="true tilt")
    ax.axhline(NOMINAL_CRITERIA["gust_band_deg"], color=MUTED, lw=0.8, ls="--")
    top = float(np.nanmax(L["tilt_deg"][L["state"] == lp.STATE_POWERED])) if np.any(L["state"] == lp.STATE_POWERED) else 10.0
    ax.set_ylim(0.0, max(5.0, min(top * 1.15, 60.0)))
    ax.legend(fontsize=7.5, frameon=False, ncol=3)
    ax = axes[2, 0]
    style(ax, "Horizontal track of the exit plane (top view; pad at the origin)", "y [m]")
    pe = exit_plane_positions(res)
    ax.plot(pe[:, 0], pe[:, 1], color=PALETTE[6], lw=1.4, label="track")
    ax.plot([0.0], [0.0], "s", color=INK2, ms=6, label="pad")
    for i, td in enumerate((res.vtol or {}).get("touchdowns", [])):
        ax.plot([td["x"]], [td["y"]], "o", color=PALETTE[7], ms=8, label="touchdown" if i == 0 else None)
    for i, pu in enumerate(sc.pushes):
        k = int(np.searchsorted(t, pu.t0_s))
        if k < len(t):
            ax.annotate("", xy=(pe[k, 0] + 0.6 * math.cos(math.radians(pu.azimuth_deg)),
                                pe[k, 1] + 0.6 * math.sin(math.radians(pu.azimuth_deg))), xytext=(pe[k, 0], pe[k, 1]),
                        arrowprops=dict(arrowstyle="->", color=PALETTE[4], lw=1.5))
    ax.set_xlabel("x [m]", fontsize=8, color=INK2)
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=7.5, frameon=False)
    ax = axes[2, 1]
    style(ax, "Wind (world, incl. gusts / turbulence), mean-wind direction and the horizontal speed", "m/s")
    phase_bands(ax)
    ax.plot(t, np.hypot(L["wind"][:, 0], L["wind"][:, 1]), color=PALETTE[0], lw=0.8, label="|wind| horizontal")
    ax.plot(t, np.hypot(L["vel"][:, 0], L["vel"][:, 1]), color=PALETTE[1], lw=1.4, label="vehicle horizontal speed")
    wm = L["wind_mean"]
    if np.any(np.hypot(wm[:, 0], wm[:, 1]) > 0.1):
        ax2 = ax.twinx()
        ax2.plot(t, np.degrees(np.unwrap(np.arctan2(wm[:, 1], wm[:, 0]))), color=PALETTE[5], lw=1.0, ls="--")
        ax2.set_ylabel("mean-wind direction [deg] (dashed)", fontsize=8, color=PALETTE[5])
        ax2.tick_params(colors=PALETTE[5], labelsize=8)
    ax.set_xlabel("t [s]", fontsize=8, color=INK2)
    ax.legend(fontsize=7.5, frameon=False, ncol=2)
    for a_ in (axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1], axes[2, 1]):
        a_.set_xlim(t[0], t[-1])
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=100, facecolor=SURFACE)
    plt.close(fig)


# --------------------------------------------------------------------------- #
# running
# --------------------------------------------------------------------------- #
_VEH: Vehicle | None = None                  # the VTOL hardware configuration with the battery (the default vehicle)
_VEHS: dict[str, Vehicle] = {}              # ... with the other power sources (tether, hybrid)


def hardware_config() -> str:
    """v0.9 (architect decision a): the hardware configuration the VTOL flight mode flies - params
    mission.vtol.hardware_config ('vtol'; mission.config stays the CAD default 'rocket')."""
    with open(PARAMS_PATH, "r", encoding="utf-8") as f:
        p = json.load(f)
    return str(((p.get("mission") or {}).get("vtol") or {}).get("hardware_config") or "rocket")


def vehicle(power: str | None = None) -> Vehicle:
    """The vehicle the VTOL scenarios fly: the hardware configuration of ``hardware_config`` (v0.9: 'vtol', the CAD report
    of the power source - battery 1647 g, tether 1399 g, hybrid 1520 g) - cached per power source."""
    global _VEH
    pw = power or "battery"
    if pw == "battery":
        if _VEH is None:
            hw = hardware_config()
            _VEH = load_vehicle(config=hw, power="battery") if hw == "vtol" else load_vehicle()
        return _VEH
    if pw not in _VEHS:
        _VEHS[pw] = load_vehicle(config="vtol", power=pw)
    return _VEHS[pw]


def run_scenario(sc: Scenario, gains: dict | None = None) -> tuple[SimResult, dict[str, Any]]:
    veh = vehicle(sc.power_source)
    g = gains if gains is not None else attitude_gains(veh.params)
    res = FlightSim(veh, sc, g).run()
    return res, compute_vtol_metrics(res)


def summary_row(m: dict[str, Any], verdict: tuple[str, str]) -> dict[str, Any]:
    """The few numbers a one-line report needs."""
    keys = ("scenario", "estimator", "t_motor_start_s", "t_hold_s", "t_land_s", "land_by", "t_landed_s", "landed_by",
            "t_abort_s", "abort_reason", "interlocks", "motor_time_s", "hold_rms_err_m", "hold_max_abs_err_m",
            "climb_overshoot_m", "sat_high_after_ramp_s", "u_mean_hold", "max_tilt_powered_deg", "max_tilt_est_err_deg",
            "rms_tilt_est_err_deg", "gust_recovery_max_s", "gust_recovery_dev_max_s", "gust_peak_tilt_max_deg",
            "touchdown_vz", "touchdown_vh_air", "touchdown_vh", "touchdown_tilt_deg", "touchdown_distance_m",
            "false_touchdown", "height_at_landed_m", "drift_max_m", "vh_max_airborne_mps", "touchdown_confirm_s",
            "push_peak_tilt_max_deg", "push_recovery_max_s", "push_peak_disp_max_m", "push_alt_drop_max_m",
            "push_min_height_m", "abort_rate_cause", "no_liftoff", "kt_hold_mean", "kt_final", "kt_learn_s",
            "support_margin_ground_g", "yaw_integ_pinned_frac_hold", "yaw_err_hold_mean_deg", "yaw_rate_rms_hold_deg_s",
            # v0.8
            "sk_rms_m", "sk_max_m", "sk_p95_m", "sk_max_all_m", "nav_rms_m", "nav_est_err_rms_m", "touchdown_from_hold_m",
            "tilt_sp_max_deg", "tilt_max_hold_deg", "tilt_sp_mean_hold_deg", "psi_err_max_deg", "gps_avail_frac_powered",
            "pos_mode_ATT_s", "pos_mode_DR_s", "gps_resets", "pos_requests", "gps_rejections", "guard_s",
            "limited_frac_hold", "touchdown_tip_margin_mps", "touchdown_lean_toward_deg", "touchdown_tip_margin_geom_mps")
    row = {k: m.get(k) for k in keys}
    row["push_excursion_max_m"] = max((r.get("excursion_m") or 0.0 for r in m.get("pushes", []) if r.get("in_flight")),
                                      default=None)
    rets = [r.get("return_1p0m_s") for r in m.get("pushes", []) if r.get("in_flight")]
    row["push_return_1m_max_s"] = (None if any(x is None for x in rets) else max(rets)) if rets else None
    row["verdict"], row["why"] = verdict
    return row


def run_named(name: str, vgains: dict | None, vgains_src: str, out_dir: str | None, export: bool = True,
              plot: bool = True) -> dict[str, Any]:
    """Worker: build scenario ``name``, run it, judge it, write its plot and renderer log; return a JSON-safe record."""
    p = vehicle().params
    found = [(sc, exp) for sc, exp in scenarios(p, vgains) if sc.name == name]
    if not found:
        raise KeyError(f"no VTOL scenario {name!r}")
    sc, exp = found[0]
    veh = vehicle(sc.power_source)
    res, m = run_scenario(sc)
    m["estimator"] = sc.sensors.attitude_estimator
    verdict = judge(m, exp)
    crit8 = v08_criteria(p, m.get("profile"))
    rec: dict[str, Any] = {"name": name, "title": sc.title, "title_ko": exp.get("title_ko"), "profile": sc.profile or "field",
                           "expect": exp, "verdict": verdict[0], "why": verdict[1],
                           "checks": nominal_checks(m), "push_checks": push_checks(m), "metrics": m,
                           "checks_v08": v08_checks(m, crit8) if m.get("gps_build") or exp.get("v08") else None,
                           "push_checks_v08": push_v08_checks(m, crit8) if sc.pushes else None,
                           "build": m.get("build"),
                           "events": [(round(te, 3), e) for te, e in res.events if not e.startswith("GS TX")],
                           "operator": (res.vtol or {}).get("operator_log", []), "vgains_source": vgains_src,
                           "vp": (res.vtol or {}).get("vp"), "summary": summary_row(m, verdict)}
    if out_dir and plot:
        plot_vtol(res, m, verdict, os.path.join(out_dir, f"{name}.png"))
        rec["png"] = f"{name}.png"
    if out_dir and export:
        crit = dict(NOMINAL_CRITERIA) if exp.get("nominal") else {"gust_band_deg": NOMINAL_CRITERIA["gust_band_deg"]}
        if exp.get("vertical"):
            crit.update({k: NOMINAL_CRITERIA[k] for k in ("touchdown_vz_max_mps", "hold_rms_err_max_m")})
        if exp.get("push"):
            crit.update(PUSH_CRITERIA)
        ctext = CRITERIA_TEXT if exp.get("nominal") else (VERTICAL_CRITERIA_TEXT if exp.get("vertical") else "")
        ctext_ko = CRITERIA_TEXT_KO if exp.get("nominal") else (VERTICAL_CRITERIA_TEXT_KO if exp.get("vertical") else "")
        if exp.get("v08") or exp.get("push_v08") or exp.get("info_criteria"):
            crit.update(v08_criteria(p, m.get("profile")))
            cv = p["mission"]["vtol"]["position_hold"]["criteria_v08_ASSUMED"]
            crit["station_keeping_max_m"] = cv["station_keeping_max_m"]        # {field, demo} (renderer contract v5)
            ctext, ctext_ko = V08_CRITERIA_TEXT, V08_CRITERIA_TEXT_KO
        export_npz(res, os.path.join(out_dir, f"{name}.npz"), veh,
                   {"vgains_source": vgains_src, "criteria": crit, "title_ko": exp.get("title_ko"),
                    "criteria_text": ctext + (" | " + PUSH_CRITERIA_TEXT if exp.get("push") else ""),
                    "criteria_text_ko": ctext_ko + (" | " + PUSH_CRITERIA_TEXT_KO if exp.get("push") else ""),
                    "v08_checks": rec.get("checks_v08"), "push_v08_checks": rec.get("push_checks_v08"),
                    "station_keeping": {k: m.get(k) for k in ("sk_rms_m", "sk_max_m", "sk_p95_m", "sk_max_all_m",
                                                               "nav_rms_m", "touchdown_from_hold_m", "touchdown_vh_ground",
                                                               "t_hold_capture_s", "sk_window_s")},
                    "expectation": exp, "verdict": verdict[0], "verdict_why": verdict[1],
                    "touchdown": {k: m.get(k) for k in ("touchdown_vz", "touchdown_vh_air", "touchdown_vh",
                                                         "touchdown_tilt_deg", "touchdown_distance_m", "touchdown_t_s",
                                                         "false_touchdown", "landed_by", "height_at_landed_m",
                                                         "touchdown_tip_margin_mps", "touchdown_tip_vcrit_mps",
                                                         "touchdown_lean_toward_deg")},
                    "thrust_learning": {k: m.get(k) for k in ("kt_hold_mean", "kt_final", "kt_learn_s",
                                                               "kt_plant_ratio_nominal")},
                    "abort_rate_cause": m.get("abort_rate_cause"),
                    "pushes": m.get("pushes")})
        rec["npz"] = f"{name}.npz"
    return _clean(rec)


POOL_RETRIES = 3        # a worker process that dies (seen on this PC under load: BrokenProcessPool, no Python traceback)
                        # costs only its unfinished jobs: they are re-run in a fresh pool, the last attempt in-process


POOL_STALL_S = 600.0    # mp.Pool: no job finished for this long (the longest single run is ~30 s of CPU) = a worker died
                        # holding a job; the pool is terminated and the unfinished jobs re-run


def _pool_call(fn, job: tuple) -> Any:
    return fn(*job)


def pool_map(fn, jobs: list, workers: int, log=None) -> list:
    """``[fn(*job) for job in jobs]`` in ``workers`` processes, results in job order.  Robust to a worker process dying:
    the finished results are kept and the rest re-submitted to a new pool (POOL_RETRIES times, then in this process).
    Every job is a pure function of its arguments, so a re-run gives the same result.  Fix round: a spawn-context
    ``multiprocessing.Pool`` instead of ``ProcessPoolExecutor`` - on this PC the executor's workers die at their first
    call-queue read (OSError WinError 6 on the queue semaphore) in the third and later executors of a process once
    sim.sim6dof (scipy) is imported (scratch v071/vtolsim-fix pool_probe5-8: executor 2 of 6 ok, mp.Pool 5 of 5 ok); a
    job lost with a dead worker is caught by the POOL_STALL_S watchdog.  An exception raised by ``fn`` itself propagates."""
    import multiprocessing as mp
    out: list = [None] * len(jobs)
    todo = list(range(len(jobs)))
    for attempt in range(POOL_RETRIES + 1):
        if not todo:
            break
        nw = max(1, workers >> max(0, attempt - 1))   # the same count once more after a break, then fewer
        if nw <= 1 or len(todo) <= 1 or attempt == POOL_RETRIES:
            for i in todo:
                out[i] = fn(*jobs[i])
            todo = []
            break
        left = list(todo)
        try:
            with mp.get_context("spawn").Pool(min(nw, len(todo))) as pool:
                asy = {i: pool.apply_async(_pool_call, (fn, tuple(jobs[i]))) for i in todo}
                pending = set(todo)
                t_last = time.perf_counter()
                while pending:
                    done = [i for i in pending if asy[i].ready()]
                    for i in done:
                        out[i] = asy[i].get()          # re-raises an exception of fn
                        pending.discard(i)
                    if done:
                        t_last = time.perf_counter()
                    elif time.perf_counter() - t_last > POOL_STALL_S:
                        break
                    else:
                        time.sleep(0.05)
                left = sorted(pending)
                if left:
                    pool.terminate()
        except (BrokenProcessPool, OSError, EOFError) as exc:          # pool infrastructure, not fn
            left = [i for i in todo if out[i] is None]
            if log is not None:
                log(f"  worker pool error {exc!r}")
        todo = left
        if todo and log is not None:
            log(f"  worker pool stalled / broke (attempt {attempt + 1}): {len(todo)} jobs re-run")
    return out


def run_all(names: list[str], vgains: dict | None, vgains_src: str, out_dir: str | None, workers: int,
            export: bool = True, plot: bool = True) -> list[dict[str, Any]]:
    """``run_named`` for each name, in ``workers`` processes (1 = in this process); results in ``names`` order."""
    return pool_map(run_named, [(n, vgains, vgains_src, out_dir, export, plot) for n in names], workers, log=print)


# --------------------------------------------------------------------------- #
# seed studies (one fault, many noise / bias realisations)
# --------------------------------------------------------------------------- #
STUDY_SEEDS = tuple(range(100, 120))       # 20 realisations of the sensor noise / biases (incl. the accelerometer bias
                                           # direction: 10 mg ASSUMED residual, random direction per seed)


def _baro_study_one(seed: int, vgains: dict | None) -> dict[str, Any]:
    """V_baro_fail with another seed: where does the dead-reckoned landing end?"""
    p = vehicle().params
    sc = [s for s, _ in scenarios(p, vgains) if s.name == "V_baro_fail"][0]
    res, m = run_scenario(replace(sc, seed=int(seed)))
    L = res.log
    k_f = int(np.searchsorted(L["t"], FAULT_T))
    h_f = float(L["h_true"][k_f])
    after = L["t"] >= FAULT_T
    max_h = float(L["h_true"][after].max())
    if m.get("t_abort_s") is not None:
        outcome = "abort"
    elif m.get("landed_normally"):
        outcome = "landed"
    else:
        outcome = "airborne at t_end"
    hs_abort = None
    if m.get("t_abort_s") is not None:
        ka = min(int(np.searchsorted(L["t"], m["t_abort_s"])), len(L["t"]) - 1)
        hs_abort = float(exit_plane_positions(res)[ka, 2] - L["surface_z"][ka])
    return _clean({"seed": seed, "outcome": outcome, "h_at_fault_m": h_f, "max_h_after_fault_m": max_h,
                   "flew_up_m": max_h - h_f, "touchdown_vz": m.get("touchdown_vz"), "t_landed_s": m.get("t_landed_s"),
                   "landed_by": m.get("landed_by"), "abort_reason": m.get("abort_reason"),
                   "height_at_abort_m": hs_abort, "interlocks": m.get("interlocks"),
                   "false_touchdown": m.get("false_touchdown"), "motor_time_s": m.get("motor_time_s"),
                   "touchdown_vh_air": m.get("touchdown_vh_air"),
                   "h_est_at_land_m": None if m.get("t_landed_s") is None else
                   float(L["h_est"][min(int(np.searchsorted(L["t"], m["t_landed_s"])), len(L["t"]) - 1)])})


def baro_fail_study(vgains: dict | None, workers: int, seeds: tuple[int, ...] = STUDY_SEEDS) -> dict[str, Any]:
    """The baro-lost landing (v0.7.1: the VTOL altitude filter's learnt accelerometer bias, the descending reference
    on the dead-reckoned altitude without the flare, the V8 / V9 time limit) over many sensor realisations: landed /
    ABORT counts, the touchdown speeds, how far the vehicle climbed after the failure."""
    rows = pool_map(_baro_study_one, [(s, vgains) for s in seeds], workers, log=print)
    landed = [r for r in rows if r["outcome"] == "landed"]
    tdz = [abs(r["touchdown_vz"]) for r in landed if r["touchdown_vz"] is not None]
    by: dict[str, int] = {}
    for r in landed:
        by[str(r.get("landed_by"))] = by.get(str(r.get("landed_by")), 0) + 1
    return {"scenario": "V_baro_fail (MS5611 silent at 16 s in HOLD at 8 m) x seeds", "seeds": list(seeds),
            "n": len(rows), "landed": len(landed), "landed_within_1mps": sum(v <= 1.0 for v in tdz),
            "landed_by": by, "abort": sum(r["outcome"] == "abort" for r in rows),
            "abort_heights_m": [r["height_at_abort_m"] for r in rows if r["outcome"] == "abort"],
            "flew_up_more_than_2m": sum(r["flew_up_m"] > 2.0 for r in rows),
            "max_h_after_fault_m": max(r["max_h_after_fault_m"] for r in rows),
            "touchdown_abs_vz_max": max(tdz) if tdz else None,
            "touchdown_abs_vz_median": float(np.median(tdz)) if tdz else None, "runs": rows}


# --------------------------------------------------------------------------- #
# push envelope (task 5): push height along the body x impulse in the demo hover
# --------------------------------------------------------------------------- #
ENVELOPE_HEIGHTS_M = (0.05, 0.12, 0.20, 0.27, 0.33, 0.376, 0.42, 0.47, 0.53, 0.60, 0.67, 0.74, 0.82)
ENVELOPE_IMPULSES_NS = (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0)
ENVELOPE_DURATIONS_S = (0.2, 0.4)
ENVELOPE_AZIMUTHS_DEG = (0.0, 45.0)        # along a vane axis / between two vanes (body yaw ~0 at the push)
ENVELOPE_T_END = 18.0                      # push at DEMO_PUSH_T0 = 10 s, window 5 s after it ends
# fix round (physics review, medium 3): the mid-height limits were set by yaw spin from the ASSUMED residual swirl
# torque (edf.residual_swirl_torque_fraction_ASSUMED 0.15) - the envelope is run with the swirl x1 (the main map), x0
# and x2 (fraction 0 and 0.30)
ENVELOPE_SWIRL_SCALES = (1.0, 0.0, 2.0)


def envelope_case(p: dict, vgains: dict | None, z: float, impulse: float, duration: float, az: float,
                  seed: int = 90, swirl_scale: float = 1.0) -> Scenario:
    pu = Push(DEMO_PUSH_T0, duration, math.pi * impulse / (2.0 * duration), azimuth_deg=az, z_body_m=z)
    sw = "" if swirl_scale == 1.0 else f"_sw{swirl_scale:g}"
    return scenario_demo(p, vgains, seed=seed, name=f"E_z{z:.3f}_I{impulse:g}_T{duration:g}_a{az:g}{sw}", pushes=(pu,),
                         vtol_script=demo_script(t_off=None), t_end=ENVELOPE_T_END, swirl_scale=swirl_scale,
                         title=f"push envelope: {impulse:g} N s half-sine over {duration:g} s at z {z:.3f} m, toward {az:g} deg"
                               + ("" if swirl_scale == 1.0 else f", residual swirl x{swirl_scale:g}"))


def _envelope_one(args: tuple) -> dict[str, Any]:
    z, imp, du, az, vgains = args[:5]
    sw = float(args[5]) if len(args) > 5 else 1.0
    p = vehicle().params
    res, m = run_scenario(envelope_case(p, vgains, z, imp, du, az, swirl_scale=sw))
    r = (m.get("pushes") or [{}])[0]
    gps = bool(m.get("gps_build"))
    if gps:
        # v0.8 GPS build: recovered = back within 1 m of the pre-push position within the v0.8 push return time (the
        # position loop tilts on purpose to return - the v0.7.1 attitude rule is reported as ``attitude_recovered``)
        crit = v08_criteria(p, "demo")
        ret = r.get("return_1p0m_s")
        rec = bool(r.get("in_flight") and not r.get("interlock_codes") and not r.get("abort") and ret is not None
                   and ret <= crit["push_return_within_1m_s"])
    else:
        rec = push_recovered(r, PUSH_CRITERIA["push_recovery_max_s"])
    if not r.get("in_flight"):
        outcome = "not_in_flight"
    elif r.get("abort"):
        outcome = "abort"
    elif rec:
        outcome = "recovered"
    elif r.get("interlock_codes"):
        outcome = "interlock"
    else:
        outcome = "slow_recovery"
    return _clean({"z_body_m": z, "impulse_ns": imp, "duration_s": du, "azimuth_deg": az, "swirl_scale": sw,
                   "force_n": r.get("force_n"),
                   "outcome": outcome, "interlocks": r.get("interlocks"), "abort_cause": r.get("abort_cause"),
                   "peak_tilt_deg": r.get("peak_tilt_deg"),
                   "peak_dev_deg": r.get("peak_dev_deg"), "recovery_s": r.get("recovery_s"),
                   "max_rate_deg_s": r.get("max_rate_deg_s"), "max_yaw_rate_deg_s": r.get("max_yaw_rate_deg_s"),
                   "max_rate_xy_deg_s": r.get("max_rate_xy_deg_s"), "max_rate_z_deg_s": r.get("max_rate_z_deg_s"),
                   "peak_horiz_disp_m": r.get("peak_horiz_disp_m"), "alt_drop_m": r.get("alt_drop_m"),
                   "min_height_m": r.get("min_height_m"), "abort_reason": m.get("abort_reason"),
                   # v0.8
                   "gps": gps, "excursion_m": r.get("excursion_m"), "return_1p0m_s": r.get("return_1p0m_s"),
                   "attitude_recovered": push_recovered(r, PUSH_CRITERIA["push_recovery_max_s"]) if r else None})


def envelope_by_height(rows: list[dict[str, Any]], win: tuple[float, float]) -> dict[str, Any]:
    """Per push height: the largest impulse recovered (with every smaller one) in every duration and direction, the
    first ABORT impulse, the ABORT rules and causes (I3 split into tilt rate / spin, fix round), the peak tilt at 1 N s."""
    by_z: dict[str, Any] = {}
    for z in ENVELOPE_HEIGHTS_M:
        rz = [r for r in rows if r["z_body_m"] == z]
        if not rz:
            continue
        rec_all = [i for i in ENVELOPE_IMPULSES_NS
                   if all(r["outcome"] == "recovered" for r in rz if r["impulse_ns"] == i)]
        # largest impulse such that it and every smaller impulse recovered in every duration and direction
        lim = 0.0
        for i in ENVELOPE_IMPULSES_NS:
            if i in rec_all:
                lim = i
            else:
                break
        first_abort = min((r["impulse_ns"] for r in rz if r["outcome"] == "abort"), default=None)
        fa = [r for r in rz if r["outcome"] == "abort" and r["impulse_ns"] == first_abort]
        by_z[f"{z:.3f}"] = {"z_body_m": z, "max_recovered_impulse_ns": lim, "first_abort_impulse_ns": first_abort,
                            "in_window_keepout": bool(win[0] <= z <= win[1]),
                            "abort_rules": sorted({x for r in rz if r["outcome"] == "abort" for x in (r["interlocks"] or [])}),
                            "abort_causes": sorted({str(r.get("abort_cause")) for r in rz if r["outcome"] == "abort"}),
                            "first_abort_causes": sorted({str(r.get("abort_cause")) for r in fa}),
                            "max_rate_xy_at_limit_deg_s": max((r.get("max_rate_xy_deg_s") or 0.0) for r in rz
                                                              if r["impulse_ns"] == lim) if lim else None,
                            "max_rate_z_at_limit_deg_s": max((r.get("max_rate_z_deg_s") or 0.0) for r in rz
                                                             if r["impulse_ns"] == lim) if lim else None,
                            "peak_tilt_at_1ns_deg": max((r["peak_tilt_deg"] or 0.0) for r in rz if r["impulse_ns"] == 1.0)}
    return by_z


def push_envelope(vgains: dict | None, workers: int, out_dir: str | None = OUT_DIR, log=print,
                  swirl_scales: tuple[float, ...] = ENVELOPE_SWIRL_SCALES) -> dict[str, Any]:
    """The recovered / ABORT boundary of a single half-sine push in the demo hover over the push height and impulse
    (both durations, both directions), the peak tilt, and the recommendation: the height band with the largest
    recovered impulse outside the intake-window keep-out, the largest push recovered there (every duration and
    direction), plot + JSON.  Fix round: the whole map for each residual-swirl scale in ``swirl_scales`` (the first is
    the main map and sets the recommendation; ``by_height_swirl`` and ``recommendation_all_swirl`` = the smallest limit
    over the scales), I3 ABORTs split into tilt rate / spin, max |w_xy| and |w_z| per run."""
    veh = vehicle()
    L = veh.layout
    jobs = [(z, i, d, a, vgains, sw) for sw in swirl_scales for z in ENVELOPE_HEIGHTS_M for i in ENVELOPE_IMPULSES_NS
            for d in ENVELOPE_DURATIONS_S for a in ENVELOPE_AZIMUTHS_DEG]
    t0 = time.perf_counter()
    rows = pool_map(_envelope_one, [(j,) for j in jobs], workers, log=log)
    win = (float(L.z_window_bottom) * 1e-3, float(L.z_window_top) * 1e-3)
    sw0 = swirl_scales[0]
    by_sw = {f"{sw:g}": envelope_by_height([r for r in rows if r["swirl_scale"] == sw], win) for sw in swirl_scales}
    by_z = by_sw[f"{sw0:g}"]
    ok_z = [v for v in by_z.values() if not v["in_window_keepout"]]
    best = max(ok_z, key=lambda v: (v["max_recovered_impulse_ns"], -abs(v["z_body_m"] - veh.z_cg_m))) if ok_z else None
    rec = None
    if best is not None:
        zb = best["z_body_m"]
        band = [v["z_body_m"] for v in ok_z if v["max_recovered_impulse_ns"] >= best["max_recovered_impulse_ns"]]
        fmax = max((r["force_n"] for r in rows if r["z_body_m"] == zb and r["impulse_ns"] <= best["max_recovered_impulse_ns"]
                    and r["outcome"] == "recovered" and r["swirl_scale"] == sw0), default=None)
        rec = {"height_m": zb, "heights_with_the_same_limit_m": band,
               "max_recovered_impulse_ns": best["max_recovered_impulse_ns"], "largest_force_recovered_n": fmax,
               "cg_height_m": veh.z_cg_m, "window_keepout_m": list(win),
               "cg_clear_of_window_top_m": veh.z_cg_m - win[1], "swirl_scale": sw0,
               "note": ("recommended pushing height = the height outside the intake-window keep-out with the largest impulse "
                        "recovered in both durations and both directions (ties: nearest the CG); the limit is that impulse "
                        "(every smaller one recovered too); the demo limit should keep a margin below it")}
    # the smallest limit per height over every swirl scale (what holds whatever the unmeasured swirl torque)
    zmin: dict[str, Any] = {}
    for kz in by_z:
        lims = {s: by_sw[s][kz]["max_recovered_impulse_ns"] for s in by_sw if kz in by_sw[s]}
        zmin[kz] = {"z_body_m": by_z[kz]["z_body_m"], "in_window_keepout": by_z[kz]["in_window_keepout"],
                    "max_recovered_impulse_ns_by_swirl": lims, "min_over_swirl_ns": min(lims.values())}
    ok_m = [v for v in zmin.values() if not v["in_window_keepout"]]
    bm = max(ok_m, key=lambda v: (v["min_over_swirl_ns"], -abs(v["z_body_m"] - veh.z_cg_m))) if ok_m else None
    rec_all = None if bm is None else {
        "height_m": bm["z_body_m"], "max_recovered_impulse_ns": bm["min_over_swirl_ns"],
        "heights_with_the_same_limit_m": [v["z_body_m"] for v in ok_m if v["min_over_swirl_ns"] >= bm["min_over_swirl_ns"]],
        "swirl_scales": list(swirl_scales),
        "note": "the same rule on the smallest limit over the residual-swirl scales (swirl fraction 0.15 x scale)"}
    out = {"_meta": {"written_by": "sim/vtol_scenarios.py push_envelope (v0.8)",
                     "date": _dt.datetime.now().isoformat(timespec="seconds"),
                     "params_sha256": params_sha256(), "profile": "demo", "push_t0_s": DEMO_PUSH_T0, "profile_shape": "half_sine",
                     "heights_m": list(ENVELOPE_HEIGHTS_M), "impulses_ns": list(ENVELOPE_IMPULSES_NS),
                     "durations_s": list(ENVELOPE_DURATIONS_S), "azimuths_deg": list(ENVELOPE_AZIMUTHS_DEG),
                     "swirl_scales": list(swirl_scales), "main_swirl_scale": sw0,
                     "swirl_fraction_ASSUMED": float(veh.params["edf"]["residual_swirl_torque_fraction_ASSUMED"]),
                     "cg_offset_mm": list(cad_cg_offset_mm()),
                     "criteria": PUSH_CRITERIA, "criteria_text": PUSH_CRITERIA_TEXT,
                     "gps_build": bool(rows and rows[0].get("gps")),
                     "criteria_v08": {k: v08_criteria(veh.params, "demo")[k] for k in ("push_return_within_1m_s",
                                                                                      "push_rule_max_impulse_ns")},
                     "outcomes": {"recovered": "v0.8 GPS build: no interlock / ABORT, back within 1 m of the pre-push "
                                               "position <= push_return_within_1m_s after the push (the attitude rule of "
                                               "v0.7.1 is in each run's attitude_recovered); without the GPS: push_recovered (no "
                                               "interlock, motor on, attitude back within 2 deg of the pre-push attitude <= 2 s "
                                               "after the push)",
                                  "abort": "an ABORT rule acted (I2 tilt / I3 rate or spin): motor off, the fall from the "
                                           "hover, release", "slow_recovery": "no interlock but not back within 2 s",
                                  "interlock": "another rule acted (e.g. a LAND)"},
                     "abort_causes": {"tilt": "I2 tilt > tilt_abort_deg", "tilt_rate": "I3: |w_xy| > tilt_rate_abort_dps",
                                      "spin": "I3: |w_z| > spin_rate_abort_dps (the swirl torque spins the vehicle while a "
                                              "saturated pitch pair makes the mixer drop the yaw command)",
                                      "tilt_rate+spin": "I3: both", "gyro_saturation": "I3: gyro at full scale"},
                     "runs": len(rows), "wall_s": round(time.perf_counter() - t0, 1), "model_notes": MODEL_NOTES},
           "by_height": by_z, "recommendation": rec, "by_height_swirl": by_sw, "by_height_min_over_swirl": zmin,
           "recommendation_all_swirl": rec_all, "runs": rows}
    out = _clean(out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "push_envelope.json"), "w", encoding="utf-8", newline="\n") as f:
            json.dump(out, f, indent=1, ensure_ascii=False)
        plot_envelope(out, os.path.join(out_dir, "push_envelope.png"))
        if len(swirl_scales) > 1:
            plot_envelope_swirl(out, os.path.join(out_dir, "push_envelope_swirl.png"))
    log(f"push envelope: {len(rows)} runs in {out['_meta']['wall_s']:.0f} s; recommendation {rec}; over the swirl scales "
        f"{rec_all}")
    return out


def plot_envelope_swirl(env: dict[str, Any], path: str) -> None:
    """The recovered-impulse limit per push height for each residual-swirl scale (one line each), the window keep-out
    and the CG height."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8.5, 6.0), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    frac = float(env["_meta"].get("swirl_fraction_ASSUMED", 0.15))
    for i, (sw, byz) in enumerate(env["by_height_swirl"].items()):
        zz = [v["z_body_m"] for v in byz.values()]
        ll = [v["max_recovered_impulse_ns"] for v in byz.values()]
        ax.plot(ll, zz, "o-", color=PALETTE[i % len(PALETTE)], lw=1.6,
                label=f"residual swirl x{float(sw):g} (fraction {frac * float(sw):.2f})")
    rec = env.get("recommendation") or {}
    win = rec.get("window_keepout_m")
    if win:
        ax.axhspan(win[0], win[1], color="#000000", alpha=0.10, lw=0)
        ax.text(0.1, win[1], " intake windows (keep out)", va="bottom", fontsize=8, color=INK)
    ax.axhline(vehicle().z_cg_m, color=INK, lw=0.8, ls="--")
    ax.text(0.1, vehicle().z_cg_m, " CG", va="bottom", fontsize=8, color=INK)
    for s_ in ("top", "right"):
        ax.spines[s_].set_visible(False)
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_xlabel("largest impulse recovered (every duration / direction, every smaller impulse too) [N s]", fontsize=9,
                  color=INK2)
    ax.set_ylabel("push height above the exit plane [m]", fontsize=9, color=INK2)
    ax.set_title("Demo hover: push limit vs the ASSUMED EDF residual swirl torque", fontsize=10, color=INK, loc="left")
    ax.legend(fontsize=8.5, frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=100, facecolor=SURFACE)
    plt.close(fig)


def plot_envelope(env: dict[str, Any], path: str) -> None:
    """Height x impulse map per duration (worst of the two directions; the main swirl scale): colour = outcome,
    number = peak tilt."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    meta = env["_meta"]
    zs, imps, durs = meta["heights_m"], meta["impulses_ns"], meta["durations_s"]
    sw0 = meta.get("main_swirl_scale", 1.0)
    env = dict(env, runs=[r for r in env["runs"] if r.get("swirl_scale", 1.0) == sw0])
    rank = {"recovered": 0, "slow_recovery": 1, "interlock": 2, "abort": 3, "not_in_flight": 4}
    cols = {0: "#1baf7a", 1: "#eda100", 2: "#8a63d2", 3: "#e34948", 4: "#bbbbbb"}
    fig, axes = plt.subplots(1, len(durs), figsize=(7.2 * len(durs), 7.0), facecolor=SURFACE)
    axes = np.atleast_1d(axes)
    rec = env.get("recommendation") or {}
    win = rec.get("window_keepout_m")
    for ax, du in zip(axes, durs):
        ax.set_facecolor(SURFACE)
        for iz, z in enumerate(zs):
            for ii, imp in enumerate(imps):
                rr = [r for r in env["runs"] if r["z_body_m"] == z and r["impulse_ns"] == imp and r["duration_s"] == du]
                if not rr:
                    continue
                worst = max(rr, key=lambda r: rank.get(r["outcome"], 5))
                ax.add_patch(plt.Rectangle((ii - 0.5, iz - 0.5), 1, 1, color=cols[rank.get(worst["outcome"], 4)], lw=0))
                pt = max((r["peak_tilt_deg"] or 0.0) for r in rr)
                ax.text(ii, iz, f"{pt:.0f}" if pt < 100 else ">", ha="center", va="center", fontsize=7, color="white")
        ax.set_xticks(range(len(imps)))
        ax.set_xticklabels([f"{i:g}" for i in imps], fontsize=8)
        ax.set_yticks(range(len(zs)))
        ax.set_yticklabels([f"{z:.2f}" for z in zs], fontsize=8)
        ax.set_xlim(-0.5, len(imps) - 0.5)
        ax.set_ylim(-0.5, len(zs) - 0.5)
        ax.set_xlabel("push impulse [N s] (half-sine)", fontsize=9, color=INK2)
        ax.set_ylabel("push height above the exit plane [m]", fontsize=9, color=INK2)
        ax.set_title(f"duration {du:g} s (worst of 0 / 45 deg; number = peak tilt deg)", fontsize=9.5, color=INK, loc="left")
        if win:
            lo = np.interp(win[0], zs, range(len(zs)))
            hi = np.interp(win[1], zs, range(len(zs)))
            ax.axhspan(lo, hi, color="#000000", alpha=0.10, lw=0)
            ax.text(len(imps) - 0.6, hi, "intake windows (keep out)", ha="right", va="bottom", fontsize=8, color=INK)
        zc = np.interp(vehicle().z_cg_m, zs, range(len(zs)))
        ax.axhline(zc, color=INK, lw=0.8, ls="--")
        ax.text(-0.4, zc, " CG", va="bottom", fontsize=8, color=INK)
    handles = [plt.Rectangle((0, 0), 1, 1, color=cols[v]) for v in (0, 1, 2, 3)]
    slow = ("not back within 1 m in time" if env["_meta"].get("gps_build") else "slow recovery (> 2 s)")
    fig.legend(handles, ["recovered", slow, "other rule", "ABORT (I2 / I3)"], loc="lower center", ncol=4,
               frameon=False, fontsize=9)
    fig.suptitle("Demo hover (1.5 m): single hand push - recovered / ABORT boundary" +
                 (f"; recommended height {rec['height_m']:.2f} m, limit {rec['max_recovered_impulse_ns']:g} N s" if rec else ""),
                 fontsize=10.5, color=INK, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0.05, 1, 0.95))
    fig.savefig(path, dpi=100, facecolor=SURFACE)
    plt.close(fig)


# --------------------------------------------------------------------------- #
# gain tuning: vertical controller (VGAIN_KEYS) and the NAV estimator (EGAIN_KEYS)
# --------------------------------------------------------------------------- #
TUNE_GRID: dict[str, tuple[float, ...]] = {
    "alt_kp": (0.8, 1.0, 1.25, 1.6, 2.0, 2.5),
    "vz_kp": (0.08, 0.1, 0.125, 0.16, 0.2, 0.25),
    "vz_ki": (0.02, 0.04, 0.08, 0.15),
    "vz_i_max": (0.08, 0.15),
}
EST_GRID: dict[str, tuple[float, ...]] = {
    # fix round: with the corrected drag time constant (9.55 s) the params rule 1 / kp >= tau_d allows kp <= 0.105 and the
    # Routh margin ki < kp / (2 tau_d); 0.15 stays in the grid to record that it is infeasible now (the v0.7.1 grid was
    # kp 0.08-0.6, ki 0.002-0.04, accel LPF 1-4 Hz; its winner kp 0.15 / ki 0.01 fails both rules)
    "est_kp": (0.03, 0.05, 0.07, 0.09, 0.1, 0.15),
    "est_ki": (0.0005, 0.001, 0.002, 0.003, 0.005),
    "est_accel_lpf_hz": (0.5, 1.0, 2.0),
}
TUNE_REFINE_FACTORS = (0.8, 1.0, 1.25)
# stage 1 (vertical): the field nominal with the v0.7.1 estimator on two seeds, a heavy / weak and a light / strong
# plant (the controller keeps the nominal model), the demo nominal with its pushes
TUNE_CASES: tuple[dict[str, Any], ...] = (
    {"seed": 70},
    {"seed": 71},
    {"seed": 72, "mass_scale": 1.05, "thrust_scale": 0.97},
    {"seed": 73, "mass_scale": 0.95, "thrust_scale": 1.03},
    {"seed": 80, "demo": True},
)
# stage 1 (estimator): calm field nominals on three seeds, the 3 m/s windy nominal, the demo nominal on two seeds
EST_CASES: tuple[dict[str, Any], ...] = (
    {"seed": 70},
    {"seed": 71},
    {"seed": 74},
    {"seed": 75, "wind_mps": 3.0, "turbulence_sigma_mps": 0.5},
    {"seed": 80, "demo": True},
    {"seed": 81, "demo": True},
)
TUNE_MC_SEED = 7
TUNE_MC_N = 60                       # stage 2 field campaign runs per finalist
TUNE_MC_N_DEMO = 40                  # stage 2 demo campaign runs per finalist
TUNE_STAGE2_PER_VZ_KP = 1            # vertical finalists: the stage-1 best of each vz_kp value (+ the overall best, + ASSUMED)
EST_STAGE2_N = 4                     # estimator finalists: the stage-1 best EST_STAGE2_N (+ ASSUMED)
PM_MIN_DEG = 45.0                    # linear phase-margin constraint on both vertical loops
EST_ROUTH_MARGIN = 2.0               # estimator gains must satisfy kp > margin x tau_d x ki (Routh, tau_d = drag time const.)
EST_KP_TAU_MAX = 1.0                 # ... and kp tau_d <= this (params nav_estimator: "1 / kp must be longer than that drag
                                     # time constant"; with ki = 0 the loop tau_d s^2 + s + kp has zeta = 1 / (2 sqrt(kp tau_d))
                                     # >= 0.5) - fix round: the v0.7.1 tuner checked the Routh rule only
PROFILE_SIGNIFICANCE_SE = 2.0        # demo-profile gains only if better than the field gains by > 2 paired standard errors
COST_DOC = ("vertical, per run: c = hold_rms_err_m / 0.5 + 0.5 max(climb_overshoot_m, 0) / 0.5 + 0.5 |touchdown_vz| / 1.0 + "
            "0.25 touchdown_confirm_s / 2 + 0.5 u_jitter_hold / 0.01 + 0.25 sat_high_after_ramp_s / 2 + "
            "10 [false touchdown] + 10 [interlock] + 10 [no LANDED -> SAFE] + 5 [hold RMS > 0.5 m] + "
            "5 [|touchdown v_z| > 1.0 m/s]; estimator, per run: c_e = rms_tilt_est_err_deg / 0.5 + touchdown_vh_air / 0.5 + "
            "2 [touchdown_vh_air > 0.5 m/s] + 2 [a push not recovered, each] + 0.5 drift_max_m / 20 + the vertical "
            "penalties (interlock, no LANDED, false touchdown: 10 each); stage 1 = mean over the cases on the grid (vertical "
            "+ local refinement x TUNE_REFINE_FACTORS); stage 2 = mean of c + c_e over TUNE_MC_N field + TUNE_MC_N_DEMO demo "
            "campaign runs (common random numbers for every finalist); a finalist with a false touchdown in stage 2 is "
            "disqualified.  Vertical gain sets whose linear phase margin is below PM_MIN_DEG and estimator sets that violate "
            "kp > EST_ROUTH_MARGIN tau_d ki are not evaluated.  Penalties also carry gain-independent failures (equal for "
            "every finalist on the same samples)")


def vertical_margins(vp: VerticalParams, t_max_n: float, mass_kg: float, tau_s: float, delay_s: float
                     ) -> dict[str, float | None]:
    """Linear margins of the two vertical loops about level hover (small-signal, thrust model T = T_max u^n):
    plant v_z(s) / u(s) = b e^(-s d) / (s (tau s + 1)), b = n T_max u_h^(n-1) / m; inner loop C_v = vz_kp + vz_ki / s;
    outer loop alt_kp / s around the closed inner loop (the altitude estimate taken as ideal; the reference-rate
    feed-forward does not change the loop).  Returns the crossover frequencies, phase margins and the inner gain margin."""
    n = vp.throttle_exp
    b = n * t_max_n * vp.u_hover ** (n - 1.0) / mass_kg
    w = np.logspace(-2, 3, 20000)
    s = 1j * w
    P = b * np.exp(-s * delay_s) / (s * (tau_s * s + 1.0))
    Lv = (vp.vz_kp + vp.vz_ki / s) * P
    Tv = Lv / (1.0 + Lv)
    Lh = vp.alt_kp / s * Tv

    def margins(Lw: np.ndarray) -> tuple[float | None, float | None, float | None]:
        mag = np.abs(Lw)
        ph = np.unwrap(np.angle(Lw))
        ic = np.flatnonzero((mag[:-1] >= 1.0) & (mag[1:] < 1.0))
        wc = pm = gm = None
        if ic.size:
            i = ic[0]
            wc = float(w[i])
            pm = float(180.0 + np.degrees(ph[i]))
        ig = np.flatnonzero((ph[:-1] > -np.pi) & (ph[1:] <= -np.pi))
        if ig.size:
            gm = float(-20.0 * np.log10(mag[ig[0]]))
        return wc, pm, gm

    wcv, pmv, gmv = margins(Lv)
    wch, pmh, _ = margins(Lh)
    return {"b_mps2_per_u": b, "delay_s": delay_s, "tau_s": tau_s, "vz_crossover_rad_s": wcv, "vz_pm_deg": pmv,
            "vz_gm_db": gmv, "alt_crossover_rad_s": wch, "alt_pm_deg": pmh}


def actuator_delay_s(p: dict) -> float:
    """Effective ESC command delay for the margins (v0.7.1 VTOL order): the ESC-path compute delay
    (``FCTiming.esc_delay_s``: the measured vane path + the ASSUMED supervisor / vertical-controller time) + half a PWM
    frame (zero-order hold) + the 1.5 ms pulse decode."""
    fc_hz = float(p["flight_computer"]["servo_frame_hz"])
    return FCTiming().esc_delay_s(1.0 / float(p["control"]["loop_hz"])) + 0.5 / fc_hz + 1.5e-3


def estimator_feasible(eg: dict[str, float], tau_d: float) -> bool:
    """The gated Mahony loop seen through the drag lag (the accelerometer reveals a tilt error only through the drag
    the resulting drift builds, time constant tau_d = m / (net drag derivative) - fix round: net of the vane
    counter-force, ``vtol_aero_geometry`` drag_time_constant_hover_s = 9.55 s): tau_d s^3 + s^2 + kp s + ki = 0 is stable
    iff kp > tau_d ki; EST_ROUTH_MARGIN keeps a margin; and the params nav_estimator rule kp tau_d <= EST_KP_TAU_MAX."""
    return (eg["est_kp"] > EST_ROUTH_MARGIN * tau_d * eg["est_ki"]) and (eg["est_kp"] * tau_d <= EST_KP_TAU_MAX)


def estimator_loop(eg: dict[str, float], tau_d: float) -> dict[str, float | None]:
    """The linear estimator-drag loop tau_d s^3 + s^2 + kp s + ki: the oscillatory pole pair's period and damping (the
    physics review measured ~50 s / zeta ~0.11 for kp 0.15, ki 0.01 at tau_d 9.55 s)."""
    r = np.roots([tau_d, 1.0, float(eg["est_kp"]), float(eg["est_ki"])])
    osc = [z for z in r if abs(z.imag) > 1e-9]
    if not osc:
        return {"period_s": None, "zeta": None, "poles": [[float(z.real), float(z.imag)] for z in r], "stable": bool(np.all(r.real < 0))}
    z = max(osc, key=lambda c: c.real)
    wn = abs(z)
    return {"period_s": float(2.0 * math.pi / abs(z.imag)), "zeta": float(-z.real / wn),
            "poles": [[float(x.real), float(x.imag)] for x in r], "stable": bool(np.all(r.real < 0))}


# --------------------------------------------------------------------------- #
# v0.8: linear margins of the horizontal (position) loop
# --------------------------------------------------------------------------- #
H_LOOP_MEAS_DELAY_TICKS = 2.5        # NAV step on the mean body force of its 5 ticks: the estimate refers ~2.5 ticks back
H_LOOP_SLOT_TICKS = 5                # the 5-part nlink slot set completes 5 ticks after the NAV latch (sim.nlink_proto)
H_LOOP_SP_TICKS = 1                  # the setpoint builder's tilt reaches the attitude controller in the next tick
PM_MIN_H_DEG = 45.0                  # horizontal loop phase-margin constraint (the 2-D loop rotated by a heading error e
                                     # loses e of LINEAR phase margin; the flown heading tolerance is set by the NAV gate,
                                     # see horizontal_margins - params heading.tolerance_deg 15 deg)


def attitude_closed_loop(w: np.ndarray, p: dict, g: dict, veh: Vehicle) -> np.ndarray:
    """theta / theta_sp of the pitch / roll attitude loop at hover (``sim.loop_margins`` loop broken at the moment, the
    VTOL attitude gains, the CAD I_xx, servo lag, MPU DLPF, compute delay + frame wait + pulse decode + frame ZOH):
    T = (kp + ki dt / (1 - z^-1)) att_kp A / (I s^2) / (1 + L)."""
    from sim.fc_timing import mean_wait_after_available
    from sim.loop_margins import loop_response
    dt = 1.0 / float(p["control"]["loop_hz"])
    tim = FCTiming()
    fr = float(p["flight_computer"]["servo_frame_hz"])
    comp = tim.delay_s(dt)
    delay = comp + mean_wait_after_available(dt, fr, 0.5 * dt, comp) + float(p["servo"]["pwm_center_us"]) * 1e-6
    tau_s = float(p["servo"]["time_constant_s"])
    inertia = float(np.asarray(veh.full.inertia_kg_m2)[0, 0])
    L = loop_response(w, g, "xy", dt, inertia, 1.0, tau_s, SensorConfig().imu_dlpf_tau_s, delay, fr)
    s = 1j * w
    zi = np.exp(-s * dt)
    integ = dt / (1.0 - zi)
    zoh = (1.0 - np.exp(-s / fr)) / (s / fr)
    act = np.exp(-s * delay) * zoh / (tau_s * s + 1.0)
    fwd = (g["rate_kp_xy"] + g["rate_ki_xy"] * integ) * g["att_kp_xy"] * act / (inertia * s * s)
    return fwd / (1.0 + L)


def _pm_gm(w: np.ndarray, L: np.ndarray) -> dict[str, float | None]:
    """Phase margin at the highest |L| = 1 crossing, the gain margin at the first -180 deg crossing above it."""
    mag = np.abs(L)
    idx = np.flatnonzero((mag[:-1] >= 1.0) & (mag[1:] < 1.0))
    if idx.size == 0:
        return {"crossover_rad_s": None, "phase_margin_deg": None, "gain_margin_x": None}
    i = int(idx[-1])
    wc = float(w[i])
    pm = float(180.0 + math.degrees(math.atan2(L[i].imag, L[i].real)))
    pm = (pm + 180.0) % 360.0 - 180.0
    re, im = L.real, L.imag
    cr = [c for c in np.flatnonzero((np.sign(im[:-1]) != np.sign(im[1:])) & (re[:-1] < 0.0)) if w[c] > wc]
    gm = float(1.0 / mag[cr[0]]) if cr else None
    return {"crossover_rad_s": wc, "phase_margin_deg": pm, "gain_margin_x": gm}


def horizontal_margins(pg: dict[str, float] | None = None, veh: Vehicle | None = None,
                       gps_direct: bool = False) -> dict[str, Any]:
    """Linear margins of one axis of the v0.8 position hold about the hover (small tilt, calm air - the drag time
    constant at its longest): plant a_cmd -> p = T_att(s) / (s (s + 1 / tau_d)) (theta_sp = a_cmd / g, a = g theta - v /
    tau_d); controller (sim.position_ctrl, unlimited) C(s) = (kv + ki / s)(kp_pos + s) + kd s^2 / (1 + s / (2 pi
    kd_lpf_hz)); delays: the NAV estimate's age (H_LOOP_MEAS_DELAY_TICKS) + the slot set (H_LOOP_SLOT_TICKS) + the job
    pipeline (PIPELINE_TICKS) + the setpoint tick (H_LOOP_SP_TICKS) + the 50 Hz hold (half a step).  The flown loop uses
    the NAV's IMU-propagated p, v: the GPS latency is compensated by the delayed-state buffer and is NOT in the loop.
    ``gps_direct`` = a what-if loop that used the GPS velocity / position directly: + latency_s_ASSUMED + the 5 Hz hold
    (what the latency would cost without the IMU propagation).  Heading error: a heading error e rotates the 2-D loop by
    e (eigenvalues e^{+-je} of the rotation), so in the LINEAR loop e eats e of phase margin - but that is NOT the flown
    tolerance (v0.8 physics review, medium 5): without the NAV yaw correction the 6-DOF held 20 deg, was marginal at 30
    and ended in V13 at 45 deg with these gains AND with a 49.5 deg-phase-margin set alike - the limit is the NAV velocity
    gate / velocity-reset spiral feeding plausibility rule (c), not the margin.  ``linear_heading_rotation_limit_deg``
    reports the linear number for what it is; params heading.tolerance_deg is 15 deg since the review."""
    from sim.position_ctrl import PIPELINE_TICKS
    veh = veh or vehicle()
    p = veh.params
    pp = PositionParams.from_params(p, veh.full.mass_kg, veh.t_max_n, **(pg or {}))
    tau_d = vtol_aero_geometry(veh)["drag_time_constant_hover_s"]
    from sim.controller import gains_for_mode
    g = gains_for_mode(p["control"], attitude_gains(p), lp.MODE_VTOL)
    dt = 1.0 / float(p["control"]["loop_hz"])
    T = pp.T
    w = np.logspace(-3, math.log10(0.999 * math.pi / dt), 8000)
    s = 1j * w
    T_att = attitude_closed_loop(w, p, g, veh)
    d = (H_LOOP_MEAS_DELAY_TICKS + H_LOOP_SLOT_TICKS + PIPELINE_TICKS + H_LOOP_SP_TICKS) * dt + 0.5 * T
    if gps_direct:
        rate = float(p["flight_computer"]["nav"]["gps"]["rate_hz"])
        d += float(p["flight_computer"]["nav"]["gps"]["latency_s_ASSUMED"]) + 0.5 / rate
    C = (pp.kv + pp.ki / s) * (pp.kp_pos + s) + pp.kd * s * s / (1.0 + s / (2.0 * math.pi * pp.kd_lpf_hz))
    L = C * np.exp(-s * d) * T_att / (s * (s + 1.0 / tau_d))
    mg = _pm_gm(w, L)
    # the inner velocity loop alone (position loop open): C_v = kv + ki / s + kd s LPF on v
    Cv = pp.kv + pp.ki / s + pp.kd * s / (1.0 + s / (2.0 * math.pi * pp.kd_lpf_hz))
    Lv = Cv * np.exp(-s * d) * T_att / (s + 1.0 / tau_d)
    mv = _pm_gm(w, Lv)
    ia = int(np.argmin(np.abs(np.abs(T_att) - 1.0 / math.sqrt(2.0))))
    return {"gains": {k: getattr(pp, k) for k in ("kp_pos", "kv", "ki", "kd", "kd_lpf_hz")}, "tau_d_s": tau_d,
            "delay_s": d, "gps_direct": gps_direct, "attitude_bandwidth_rad_s": float(w[ia]),
            "crossover_rad_s": mg["crossover_rad_s"], "phase_margin_deg": mg["phase_margin_deg"],
            "gain_margin_x": mg["gain_margin_x"], "velocity_loop_crossover_rad_s": mv["crossover_rad_s"],
            "velocity_loop_phase_margin_deg": mv["phase_margin_deg"], "velocity_loop_gain_margin_x": mv["gain_margin_x"],
            "linear_heading_rotation_limit_deg": mg["phase_margin_deg"],
            "note": ("heading error e rotates the linear 2-D loop by e (stable while e < the phase margin, linear, "
                     "unsaturated); NOT the flown heading tolerance - the 6-DOF limit without the NAV yaw correction is "
                     "~20-30 deg, set by the NAV velocity gate (physics review medium 5)")}


def tune_case(sc_base: Scenario, case: dict[str, Any], vg: dict[str, float]) -> Scenario:
    sc = replace(sc_base, seed=int(case["seed"]), vgains=dict(vg),
                 mass_scale=float(case.get("mass_scale", 1.0)), thrust_scale=float(case.get("thrust_scale", 1.0)),
                 wind_mps=float(case.get("wind_mps", sc_base.wind_mps)),
                 turbulence_sigma_mps=float(case.get("turbulence_sigma_mps", sc_base.turbulence_sigma_mps)))
    if case.get("estimator"):
        sc = replace(sc, sensors=replace(sc.sensors, attitude_estimator=case["estimator"]))
    return sc


def tune_cost_one(m: dict[str, Any]) -> tuple[float, dict[str, Any]]:
    """Vertical cost of one run (COST_DOC)."""
    c = nominal_checks(m)
    hr = m.get("hold_rms_err_m")
    hr = 10.0 if hr is None else hr
    tdz = m.get("touchdown_vz")
    tdz = 10.0 if tdz is None else abs(tdz)
    conf = m.get("touchdown_confirm_s")
    conf = 10.0 if conf is None or conf < 0 else conf
    jit = m.get("u_jitter_hold") or 0.0
    parts = {
        "hold": hr / 0.5, "overshoot": 0.5 * max(m.get("climb_overshoot_m") or 0.0, 0.0) / 0.5,
        "td_vz": 0.5 * tdz / 1.0, "td_confirm": 0.25 * conf / 2.0, "jitter": 0.5 * jit / 0.01,
        "sat": 0.25 * (m.get("sat_high_after_ramp_s") or 0.0) / 2.0,
        "pen": (10.0 * (not c["no_false_touchdown"]) + 10.0 * (not c["no_interlock"]) + 10.0 * (not c["landed_then_safe"])
                + 5.0 * (not c["hold_rms"]) + 5.0 * (not c["touchdown_vz"])),
    }
    return float(sum(parts.values())), parts


def est_cost_one(m: dict[str, Any]) -> tuple[float, dict[str, Any]]:
    """Estimator cost of one run (COST_DOC)."""
    c = nominal_checks(m)
    er = m.get("rms_tilt_est_err_deg")
    er = 10.0 if er is None else er
    vh = m.get("touchdown_vh_air")
    vh = 5.0 if vh is None else vh
    pushes = push_checks(m)
    parts = {"est_err": er / 0.5, "vh_air": vh / 0.5, "vh_fail": 2.0 * (vh > NOMINAL_CRITERIA["touchdown_vh_max_mps"]),
             "push": 2.0 * sum(not v for v in pushes.values()), "drift": 0.5 * (m.get("drift_max_m") or 0.0) / 20.0,
             "pen": (10.0 * (not c["no_false_touchdown"]) + 10.0 * (not c["no_interlock"])
                     + 10.0 * (not c["landed_then_safe"]))}
    return float(sum(parts.values())), parts


TUNE_KEEP = ("hold_rms_err_m", "hold_max_abs_err_m", "climb_overshoot_m", "touchdown_vz", "touchdown_vh",
             "touchdown_vh_air", "touchdown_tilt_deg", "touchdown_confirm_s", "false_touchdown", "height_at_landed_m",
             "interlocks", "u_jitter_hold", "sat_high_after_ramp_s", "u_mean_hold", "t_climb_s", "drift_max_m",
             "rms_tilt_est_err_deg", "max_tilt_est_err_deg", "push_recovery_max_s", "push_peak_tilt_max_deg", "landed_by")


def _case_scenario(case: dict[str, Any], g: dict[str, float]) -> Scenario:
    veh = vehicle()
    base = scenario_demo(veh.params, None) if case.get("demo") else scenario_nominal(veh.params, None)
    return tune_case(base, case, g)


def _tune_eval(g: dict[str, float], case_idx: int, which: str = "v") -> dict[str, Any]:
    cases = TUNE_CASES if which == "v" else EST_CASES
    res, m = run_scenario(_case_scenario(cases[case_idx], g))
    cost, parts = tune_cost_one(m)
    cost_e, parts_e = est_cost_one(m)
    return _clean({"g": g, "case": case_idx, "cost": cost if which == "v" else cost_e, "cost_v": cost, "cost_e": cost_e,
                   "parts": parts, "parts_e": parts_e, "m": {k: m.get(k) for k in TUNE_KEEP},
                   "checks": nominal_checks(m), "push_checks": push_checks(m)})


def _tune_eval_v(g: dict[str, float], case_idx: int) -> dict[str, Any]:
    return _tune_eval(g, case_idx, "v")


def _tune_eval_e(g: dict[str, float], case_idx: int) -> dict[str, Any]:
    return _tune_eval(g, case_idx, "e")


def _tune_mc_eval(g: dict[str, float], j: int, profile: str = "field") -> dict[str, Any]:
    """Stage 2: campaign run j of the tuning seed with gains ``g`` (field or demo campaign sampler)."""
    from sim import vtol_campaign as vc          # lazy: vtol_campaign imports this module
    veh = vehicle()
    s = vc.sample(TUNE_MC_SEED, j, profile=profile)
    res, m = run_scenario(vc.build_scenario(veh.params, s, g))
    cost, parts = tune_cost_one(m)
    cost_e, parts_e = est_cost_one(m)
    return _clean({"g": g, "j": j, "profile": profile, "cost": cost + cost_e, "cost_v": cost, "cost_e": cost_e,
                   "m": {k: m.get(k) for k in TUNE_KEEP}, "checks": nominal_checks(m), "push_checks": push_checks(m)})


def _tune_mc_eval_field(g: dict[str, float], j: int) -> dict[str, Any]:
    return _tune_mc_eval(g, j, "field")


def _tune_mc_eval_demo(g: dict[str, float], j: int) -> dict[str, Any]:
    return _tune_mc_eval(g, j, "demo")


def _grid(axes: dict[str, tuple[float, ...]]) -> list[dict[str, float]]:
    out = [{}]
    for k, vals in axes.items():
        out = [dict(d, **{k: float(v)}) for d in out for v in vals]
    return out


def _gkey(g: dict[str, float], keys: tuple[str, ...] = GAIN_KEYS) -> tuple:
    return tuple(round(float(g[k]), 6) for k in keys if k in g)


def assumed_gains(p: dict) -> dict[str, float]:
    """params.json mission.vtol.gains_ASSUMED + nav_estimator *_ASSUMED as a gains dict."""
    v = p["mission"]["vtol"]
    ne = v["nav_estimator"]
    g = {k: float(v["gains_ASSUMED"][k]) for k in VGAIN_KEYS}
    g.update(est_kp=float(ne["kp_ASSUMED"]), est_ki=float(ne["ki_ASSUMED"]), est_accel_lpf_hz=float(ne["accel_lpf_hz_ASSUMED"]))
    return g


def tune(workers: int, out_path: str = VTOL_GAINS_JSON, log=print) -> dict[str, Any]:
    """Stage 1V: grid (TUNE_GRID + local refinement) of the four vertical gains on TUNE_CASES with the ASSUMED estimator
    gains.  Stage 1E: grid (EST_GRID) of the three estimator gains on EST_CASES with the stage-1V best vertical gains.
    Stage 2: the finalists (vertical: overall best, best of each vz_kp, ASSUMED; estimator: the best EST_STAGE2_N +
    ASSUMED; each vertical finalist with the best estimator set, each estimator finalist with the best vertical set) on
    TUNE_MC_N field + TUNE_MC_N_DEMO demo campaign runs; the J2 minimum per profile is written to ``out_path`` (the demo
    set only under "profiles" when it differs from the field set)."""
    veh = vehicle()
    p = veh.params
    tau = float(p["edf"]["thrust_time_constant_s"])
    d_s = actuator_delay_s(p)
    tau_d = vtol_aero_geometry(veh)["drag_time_constant_hover_s"]
    t0 = time.perf_counter()
    base = assumed_gains(p)
    v_eval: dict[tuple, dict[str, Any]] = {}
    e_eval: dict[tuple, dict[str, Any]] = {}

    def vp_of(vg: dict[str, float]) -> VerticalParams:
        return VerticalParams.from_params(p, veh.full.mass_kg, veh.t_max_n, **{k: vg[k] for k in VGAIN_KEYS})

    def pmap(fn, a: list, b: list) -> list:
        return pool_map(fn, list(zip(a, b)), workers, log=log)

    def evaluate_v(cands: list[dict[str, float]], eg: dict[str, float]) -> None:
        todo = []
        for vg in cands:
            key = _gkey(vg, VGAIN_KEYS)
            if key in v_eval:
                continue
            mg = vertical_margins(vp_of(vg), veh.t_max_n, veh.full.mass_kg, tau, d_s)
            ok = (mg["vz_pm_deg"] or 0.0) >= PM_MIN_DEG and (mg["alt_pm_deg"] or 0.0) >= PM_MIN_DEG
            v_eval[key] = {"g": dict(vg), "margins": mg, "feasible": ok, "runs": [], "cost": None}
            if ok:
                todo.extend((key, dict(vg, **eg), i) for i in range(len(TUNE_CASES)))
        if not todo:
            return
        results = pmap(_tune_eval_v, [g for _, g, _ in todo], [i for _, _, i in todo])
        for (key, _, _), r in zip(todo, results):
            v_eval[key]["runs"].append(r)
        for e in v_eval.values():
            if e["feasible"] and e["cost"] is None and len(e["runs"]) == len(TUNE_CASES):
                e["cost"] = float(np.mean([r["cost"] for r in e["runs"]]))

    def evaluate_e(cands: list[dict[str, float]], vg: dict[str, float]) -> None:
        todo = []
        for eg in cands:
            key = _gkey(eg, EGAIN_KEYS)
            if key in e_eval:
                continue
            ok = estimator_feasible(eg, tau_d)
            e_eval[key] = {"g": dict(eg), "feasible": ok, "runs": [], "cost": None}
            if ok:
                todo.extend((key, dict(vg, **eg), i) for i in range(len(EST_CASES)))
        if not todo:
            return
        results = pmap(_tune_eval_e, [g for _, g, _ in todo], [i for _, _, i in todo])
        for (key, _, _), r in zip(todo, results):
            e_eval[key]["runs"].append(r)
        for e in e_eval.values():
            if e["feasible"] and e["cost"] is None and len(e["runs"]) == len(EST_CASES):
                e["cost"] = float(np.mean([r["cost"] for r in e["runs"]]))

    def best(d: dict[tuple, dict[str, Any]]) -> dict[str, Any]:
        return min((e for e in d.values() if e["cost"] is not None), key=lambda e: e["cost"])

    eg0 = {k: base[k] for k in EGAIN_KEYS}
    vg0 = {k: base[k] for k in VGAIN_KEYS}
    # ---- stage 1V ----
    grid = _grid(TUNE_GRID)
    log(f"tuning stage 1V: {len(grid)} vertical grid points x {len(TUNE_CASES)} cases, {workers} workers")
    evaluate_v(grid + [vg0], eg0)
    b = best(v_eval)
    log(f"  grid best J1 = {b['cost']:.3f} at {b['g']} ({time.perf_counter() - t0:.0f} s)")
    for it in range(2):
        n0 = len(v_eval)
        evaluate_v(_grid({k: tuple(b["g"][k] * f for f in TUNE_REFINE_FACTORS) for k in VGAIN_KEYS}), eg0)
        nb = best(v_eval)
        log(f"  refine {it + 1}: {len(v_eval) - n0} new points, best J1 = {nb['cost']:.3f} at {nb['g']}")
        if nb is b:
            break
        b = nb
    vbest = dict(b["g"])
    # ---- stage 1E ----
    egrid = _grid(EST_GRID)
    log(f"tuning stage 1E: {len(egrid)} estimator grid points x {len(EST_CASES)} cases with {vbest} "
        f"(tau_d {tau_d:.2f} s; {sum(not estimator_feasible(e, tau_d) for e in egrid)} infeasible)")
    evaluate_e(egrid + [eg0], vbest)
    be = best(e_eval)
    log(f"  estimator grid best J1e = {be['cost']:.3f} at {be['g']} ({time.perf_counter() - t0:.0f} s)")
    ebest = dict(be["g"])
    # ---- stage 2 ----
    vfeas = sorted([e for e in v_eval.values() if e["cost"] is not None], key=lambda e: e["cost"])
    efeas = sorted([e for e in e_eval.values() if e["cost"] is not None], key=lambda e: e["cost"])
    fin: dict[tuple, dict[str, float]] = {}

    def add(vg: dict[str, float], eg: dict[str, float]) -> None:
        g = {**{k: vg[k] for k in VGAIN_KEYS}, **{k: eg[k] for k in EGAIN_KEYS}}
        fin[_gkey(g)] = g
    add(vbest, ebest)
    add(vg0, eg0)
    add(vbest, eg0)
    add(vg0, ebest)
    for kp in sorted({round(e["g"]["vz_kp"], 6) for e in vfeas}):
        same = [e for e in vfeas if round(e["g"]["vz_kp"], 6) == kp][:TUNE_STAGE2_PER_VZ_KP]
        for e in same:
            add(e["g"], ebest)
    for e in efeas[:EST_STAGE2_N]:
        add(vbest, e["g"])
    fins = list(fin.values())
    log(f"tuning stage 2: {len(fins)} finalists x ({TUNE_MC_N} field + {TUNE_MC_N_DEMO} demo) campaign runs "
        f"(seed {TUNE_MC_SEED})")
    jobs_f = [(g, j) for g in fins for j in range(TUNE_MC_N)]
    jobs_d = [(g, j) for g in fins for j in range(TUNE_MC_N_DEMO)]
    res_f = pmap(_tune_mc_eval_field, [a for a, _ in jobs_f], [j for _, j in jobs_f])
    res_d = pmap(_tune_mc_eval_demo, [a for a, _ in jobs_d], [j for _, j in jobs_d])
    stage2 = []
    for g in fins:
        row: dict[str, Any] = {"g": g}
        for prof, rr in (("field", res_f), ("demo", res_d)):
            rs = [r for r in rr if _gkey(r["g"]) == _gkey(g)]
            row[f"J2_{prof}"] = float(np.mean([r["cost"] for r in rs]))
            row[f"J2v_{prof}"] = float(np.mean([r["cost_v"] for r in rs]))
            row[f"J2e_{prof}"] = float(np.mean([r["cost_e"] for r in rs]))
            row[f"n_{prof}"] = len(rs)
            for c in ("no_false_touchdown", "touchdown_vz", "touchdown_vh", "hold_rms", "no_interlock", "landed_then_safe"):
                row[f"fail_{c}_{prof}"] = int(sum(not r["checks"][c] for r in rs))
            row[f"push_fail_{prof}"] = int(sum(sum(not v for v in r["push_checks"].values()) for r in rs))
            row[f"pass_all_{prof}"] = int(sum(all(r["checks"].values()) and all(r["push_checks"].values()) for r in rs))
            for k in ("hold_rms_err_m", "rms_tilt_est_err_deg", "touchdown_vh_air", "sat_high_after_ramp_s",
                      "touchdown_confirm_s", "drift_max_m"):
                vals = [r["m"][k] for r in rs if r["m"].get(k) is not None]
                row[f"mean_{k}_{prof}"] = float(np.mean(vals)) if vals else None
        row["J2"] = row["J2_field"] + row["J2_demo"]
        stage2.append(row)
        log(f"  {g}: J2 field {row['J2_field']:.3f} demo {row['J2_demo']:.3f}; false td {row['fail_no_false_touchdown_field']}"
            f"/{row['fail_no_false_touchdown_demo']}, vh_air fails {row['fail_touchdown_vh_field']}/"
            f"{row['fail_touchdown_vh_demo']}, push fails {row['push_fail_demo']}, est err "
            f"{row['mean_rms_tilt_est_err_deg_field']:.2f} deg")

    def disq(r: dict[str, Any], prof: str) -> bool:
        return r[f"fail_no_false_touchdown_{prof}"] > 0

    win_f = min(stage2, key=lambda r: (disq(r, "field"), r["J2_field"]))
    win_d = min(stage2, key=lambda r: (disq(r, "demo"), r["J2_demo"]))
    base_row = next(r for r in stage2 if _gkey(r["g"]) == _gkey({**vg0, **eg0}))
    out: dict[str, Any] = {k: round(float(win_f["g"][k]), 6) for k in GAIN_KEYS}
    # the demo profile gets its own gains only when they beat the field gains on the SAME demo samples by more than
    # PROFILE_SIGNIFICANCE_SE standard errors of the paired difference (else the field gains serve both profiles)
    cd = {r["j"]: r["cost"] for r in res_d if _gkey(r["g"]) == _gkey(win_d["g"])}
    cf = {r["j"]: r["cost"] for r in res_d if _gkey(r["g"]) == _gkey(win_f["g"])}
    dd = np.array([cd[j] - cf[j] for j in sorted(set(cd) & set(cf))])
    se = float(dd.std(ddof=1) / math.sqrt(dd.size)) if dd.size > 1 else float("inf")
    profiles_check = {"demo_winner": win_d["g"], "paired_mean_diff_demo_minus_field_gains": float(dd.mean()) if dd.size else 0.0,
                      "paired_se": se, "n": int(dd.size), "rule_se": PROFILE_SIGNIFICANCE_SE}
    demo_diff = bool(dd.size > 1 and float(dd.mean()) < -PROFILE_SIGNIFICANCE_SE * se)
    if demo_diff:
        out["profiles"] = {"demo": {k: round(float(win_d["g"][k]), 6) for k in GAIN_KEYS}}
    vw = v_eval.get(_gkey(win_f["g"], VGAIN_KEYS))
    out["_meta"] = {
        "written_by": "sim/vtol_scenarios.py --tune (v0.7.1)", "date": _dt.datetime.now().isoformat(timespec="seconds"),
        "params_sha256": params_sha256(), "mass_kg": veh.full.mass_kg, "t_max_n": veh.t_max_n,
        "u_hover": vp_of(win_f["g"]).u_hover, "mass_source": veh.mass_source, "drag_time_constant_s": tau_d,
        "esc_delay_for_margins_s": d_s,
        "profiles_note": ("the demo profile uses different gains (under 'profiles.demo')" if demo_diff else
                          "the same gains serve both profiles: the demo winner is not better than the field gains on the "
                          "same demo samples by more than PROFILE_SIGNIFICANCE_SE paired standard errors"),
        "profiles_check": profiles_check,
        "demo_winner": win_d["g"], "field_winner": win_f["g"],
        "tuning_scenarios": ("stage 1V: sim.vtol_scenarios.scenario_nominal (field) x TUNE_CASES + scenario_demo; stage 1E: "
                             "EST_CASES; stage 2: sim.vtol_campaign sampling (field and demo); FCU timing vane "
                             f"{FCTiming().delay_s(1.0 / float(p['control']['loop_hz'])) * 1e3:.2f} ms / ESC "
                             f"{FCTiming().esc_delay_s(1.0 / float(p['control']['loop_hz'])) * 1e3:.2f} ms + "
                             f"{p['flight_computer']['servo_frame_hz']} Hz PWM latch; MPU-6050 / MS5611 noise, vibration "
                             f"{VTOL_ACCEL_VIBRATION_RMS_G_ASSUMED} g (ASSUMED), cross-flow + ram drag"),
        "cost": COST_DOC, "pm_min_deg": PM_MIN_DEG, "est_routh_margin": EST_ROUTH_MARGIN,
        "stage1_cases": list(TUNE_CASES), "est_cases": list(EST_CASES),
        "grid": {k: list(v) for k, v in TUNE_GRID.items()}, "est_grid": {k: list(v) for k, v in EST_GRID.items()},
        "refine_factors": list(TUNE_REFINE_FACTORS),
        "stage2_campaign_seed": TUNE_MC_SEED, "stage2_runs_per_finalist": {"field": TUNE_MC_N, "demo": TUNE_MC_N_DEMO},
        "evaluations": int(sum(len(e["runs"]) for e in v_eval.values()) + sum(len(e["runs"]) for e in e_eval.values())
                           + len(res_f) + len(res_d)),
        "vertical_sets_evaluated": len(vfeas), "vertical_sets_infeasible": int(sum(1 for e in v_eval.values() if not e["feasible"])),
        "estimator_sets_evaluated": len(efeas), "estimator_sets_infeasible": int(sum(1 for e in e_eval.values() if not e["feasible"])),
        "stage1v_best": {"g": vbest, "J1": b["cost"]}, "stage1e_best": {"g": ebest, "J1e": be["cost"]},
        "stage2": stage2, "J2_field": win_f["J2_field"], "J2_demo": win_d["J2_demo"],
        "margins": vw["margins"] if vw else vertical_margins(vp_of(win_f["g"]), veh.t_max_n, veh.full.mass_kg, tau, d_s),
        "baseline_ASSUMED": {**vg0, **eg0}, "baseline_J2_field": base_row["J2_field"], "baseline_J2_demo": base_row["J2_demo"],
        "stage1v_top10": [{"g": e["g"], "J1": e["cost"], "margins": e["margins"]} for e in vfeas[:10]],
        "stage1e_top10": [{"g": e["g"], "J1e": e["cost"]} for e in efeas[:10]],
        "stage1v_table_columns": ["alt_kp", "vz_kp", "vz_ki", "vz_i_max", "J1 (None: infeasible)", "vz_pm_deg", "alt_pm_deg"],
        "stage1v_table": [[*(round(e["g"][k], 6) for k in VGAIN_KEYS), e["cost"], e["margins"]["vz_pm_deg"],
                           e["margins"]["alt_pm_deg"]] for e in v_eval.values()],
        "stage1e_table_columns": ["est_kp", "est_ki", "est_accel_lpf_hz", "J1e (None: infeasible)"],
        "stage1e_table": [[*(round(e["g"][k], 6) for k in EGAIN_KEYS), e["cost"]] for e in e_eval.values()],
        "runtime_s": round(time.perf_counter() - t0, 1),
        "note": ("vertical-controller gains for sim.vertical_ctrl (VerticalParams overrides) and the NAV estimator gains "
                 "for sim.estimator.GatedMahonyEstimator (est_kp, est_ki, est_accel_lpf_hz; the gate stays params "
                 "accel_gate); params.json keeps gains_ASSUMED / nav_estimator *_ASSUMED as the documented starting "
                 "point - the firmware generator decides which set it emits"),
    }
    out = _clean(out)
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, indent=1)
    log(f"wrote {out_path}: {{{', '.join(f'{k}: {out[k]}' for k in GAIN_KEYS)}}} J2 field {win_f['J2_field']:.3f} "
        f"(ASSUMED {base_row['J2_field']:.3f}), demo winner {win_d['g']} J2 demo {win_d['J2_demo']:.3f} "
        f"(ASSUMED {base_row['J2_demo']:.3f}); {out['_meta']['evaluations']} runs in {out['_meta']['runtime_s']:.0f} s")
    return out


def tune_estimator(workers: int, out_path: str = VTOL_GAINS_JSON, log=print) -> dict[str, Any]:
    """Fix round (2026-10-21): re-tune the NAV estimator gains only, the vertical gains kept from ``out_path``.  Why:
    the corrected drag time constant (``vtol_aero_geometry``, 9.55 s with the vane counter-force) makes the v0.7.1
    winner kp 0.15 / ki 0.01 infeasible (Routh 1.57 < EST_ROUTH_MARGIN, kp tau_d 1.43 > EST_KP_TAU_MAX); the vertical
    gains still meet their constraint (linear margins unchanged: the vertical plant did not change) - they are re-checked
    on the scenarios and campaigns, not re-tuned.  Stage 1E: EST_GRID on EST_CASES with the kept vertical gains.  Stage
    2: the best EST_STAGE2_N feasible sets (+ the previous set and the params *_ASSUMED set as references, which cannot
    win when infeasible) on TUNE_MC_N field + TUNE_MC_N_DEMO demo campaign runs (common random numbers, the fix-round
    sampler); the feasible J2 field minimum is written; the demo profile gets its own set only under the
    PROFILE_SIGNIFICANCE_SE paired rule."""
    veh = vehicle()
    p = veh.params
    ag = vtol_aero_geometry(veh)
    tau_d = ag["drag_time_constant_hover_s"]
    t0 = time.perf_counter()
    with open(out_path, "r", encoding="utf-8") as f:
        prev = json.load(f)
    vg = {k: float(prev[k]) for k in VGAIN_KEYS}
    e_prev = {k: float(prev[k]) for k in EGAIN_KEYS}
    base = assumed_gains(p)
    e_ass = {k: base[k] for k in EGAIN_KEYS}
    egrid = _grid(EST_GRID)
    feas = [e for e in egrid if estimator_feasible(e, tau_d)]
    log(f"estimator re-tune: tau_d {tau_d:.3f} s (ram only {ag['drag_time_constant_ram_only_s']:.3f} s); {len(egrid)} grid "
        f"points, {len(feas)} feasible x {len(EST_CASES)} cases; vertical gains kept {vg}; previous estimator {e_prev} "
        f"feasible {estimator_feasible(e_prev, tau_d)}; params ASSUMED {e_ass} feasible {estimator_feasible(e_ass, tau_d)}")
    jobs = [(dict(vg, **e), i) for e in feas for i in range(len(EST_CASES))]
    res1 = pool_map(_tune_eval_e, jobs, workers, log=log)
    table = []
    for e in egrid:
        rs = [r for r in res1 if _gkey(r["g"], EGAIN_KEYS) == _gkey(e, EGAIN_KEYS)]
        cost = float(np.mean([r["cost"] for r in rs])) if rs else None
        table.append({"g": e, "feasible": estimator_feasible(e, tau_d), "J1e": cost, "loop": estimator_loop(e, tau_d),
                      "mean_rms_tilt_est_err_deg": float(np.mean([r["m"]["rms_tilt_est_err_deg"] for r in rs
                                                                  if r["m"].get("rms_tilt_est_err_deg") is not None]))
                      if rs else None})
    ranked = sorted([r for r in table if r["J1e"] is not None], key=lambda r: r["J1e"])
    log(f"  stage 1E best J1e {ranked[0]['J1e']:.3f} at {ranked[0]['g']} ({time.perf_counter() - t0:.0f} s); top: "
        + "; ".join(f"{r['g']} {r['J1e']:.3f}" for r in ranked[:EST_STAGE2_N]))
    fins = [r["g"] for r in ranked[:EST_STAGE2_N]]
    refs = [e for e in (e_prev, e_ass) if _gkey(e, EGAIN_KEYS) not in {_gkey(f_, EGAIN_KEYS) for f_ in fins}]
    allg = [dict(vg, **e) for e in fins + refs]
    jobs_f = [(g, j) for g in allg for j in range(TUNE_MC_N)]
    jobs_d = [(g, j) for g in allg for j in range(TUNE_MC_N_DEMO)]
    log(f"  stage 2: {len(allg)} sets ({len(fins)} finalists + {len(refs)} references) x ({TUNE_MC_N} field + "
        f"{TUNE_MC_N_DEMO} demo) campaign runs (seed {TUNE_MC_SEED})")
    res_f = pool_map(_tune_mc_eval_field, jobs_f, workers, log=log)
    res_d = pool_map(_tune_mc_eval_demo, jobs_d, workers, log=log)
    stage2 = []
    for g in allg:
        e = {k: g[k] for k in EGAIN_KEYS}
        row: dict[str, Any] = {"g": g, "feasible": estimator_feasible(e, tau_d), "loop": estimator_loop(e, tau_d),
                               "reference": _gkey(e, EGAIN_KEYS) in {_gkey(x, EGAIN_KEYS) for x in refs}}
        for prof, rr in (("field", res_f), ("demo", res_d)):
            rs = [r for r in rr if _gkey(r["g"]) == _gkey(g)]
            row[f"J2_{prof}"] = float(np.mean([r["cost"] for r in rs]))
            row[f"J2v_{prof}"] = float(np.mean([r["cost_v"] for r in rs]))
            row[f"J2e_{prof}"] = float(np.mean([r["cost_e"] for r in rs]))
            row[f"n_{prof}"] = len(rs)
            for c in ("no_false_touchdown", "touchdown_vz", "touchdown_vh", "hold_rms", "no_interlock", "landed_then_safe"):
                row[f"fail_{c}_{prof}"] = int(sum(not r["checks"][c] for r in rs))
            row[f"push_fail_{prof}"] = int(sum(sum(not v for v in r["push_checks"].values()) for r in rs))
            row[f"pass_all_{prof}"] = int(sum(all(r["checks"].values()) and all(r["push_checks"].values()) for r in rs))
            for k in ("hold_rms_err_m", "rms_tilt_est_err_deg", "touchdown_vh_air", "drift_max_m"):
                vals = [r["m"][k] for r in rs if r["m"].get(k) is not None]
                row[f"mean_{k}_{prof}"] = float(np.mean(vals)) if vals else None
                row[f"median_{k}_{prof}"] = float(np.median(vals)) if vals else None
        row["J2"] = row["J2_field"] + row["J2_demo"]
        stage2.append(row)
        log(f"  {e}{' (reference)' if row['reference'] else ''}{'' if row['feasible'] else ' INFEASIBLE'}: J2 field "
            f"{row['J2_field']:.3f} demo {row['J2_demo']:.3f}; vh_air median {row['median_touchdown_vh_air_field']} / "
            f"{row['median_touchdown_vh_air_demo']}; est err {row['mean_rms_tilt_est_err_deg_field']:.2f} deg; false td "
            f"{row['fail_no_false_touchdown_field']}/{row['fail_no_false_touchdown_demo']}")
    cand = [r for r in stage2 if r["feasible"]]

    def disq(r: dict[str, Any], prof: str) -> bool:
        return r[f"fail_no_false_touchdown_{prof}"] > 0
    win_f = min(cand, key=lambda r: (disq(r, "field"), r["J2_field"]))
    win_d = min(cand, key=lambda r: (disq(r, "demo"), r["J2_demo"]))
    cd = {r["j"]: r["cost"] for r in res_d if _gkey(r["g"]) == _gkey(win_d["g"])}
    cf = {r["j"]: r["cost"] for r in res_d if _gkey(r["g"]) == _gkey(win_f["g"])}
    dd = np.array([cd[j] - cf[j] for j in sorted(set(cd) & set(cf))])
    se = float(dd.std(ddof=1) / math.sqrt(dd.size)) if dd.size > 1 else float("inf")
    demo_diff = bool(dd.size > 1 and float(dd.mean()) < -PROFILE_SIGNIFICANCE_SE * se)
    out: dict[str, Any] = {k: round(float(win_f["g"][k]), 6) for k in GAIN_KEYS}
    if demo_diff:
        out["profiles"] = {"demo": {k: round(float(win_d["g"][k]), 6) for k in GAIN_KEYS}}
    old_meta = dict(prev.get("_meta") or {})
    vt = {k: old_meta.get(k) for k in ("written_by", "date", "params_sha256", "stage1v_best", "margins", "J2_field", "J2_demo",
                                         "baseline_J2_field", "baseline_J2_demo", "cost", "pm_min_deg", "stage1_cases",
                                         "grid", "refine_factors", "evaluations", "runtime_s", "stage1v_top10",
                                         "drag_time_constant_s", "esc_delay_for_margins_s", "field_winner", "demo_winner")}
    tau = float(p["edf"]["thrust_time_constant_s"])
    vpw = VerticalParams.from_params(p, veh.full.mass_kg, veh.t_max_n, **vg)
    out["_meta"] = {
        "written_by": "sim/vtol_scenarios.py --tune-est (v0.7.1 fix round: estimator re-tune, vertical gains kept)",
        "date": _dt.datetime.now().isoformat(timespec="seconds"), "params_sha256": params_sha256(),
        "mass_kg": veh.full.mass_kg, "t_max_n": veh.t_max_n, "u_hover": vpw.u_hover, "mass_source": veh.mass_source,
        "drag_time_constant_s": tau_d, "drag_time_constant_ram_only_s": ag["drag_time_constant_ram_only_s"],
        "vane_counterforce_factor_ram": ag["vane_counterforce_factor_ram"],
        "esc_delay_for_margins_s": actuator_delay_s(p),
        "why": ("physics review (medium 2): the drag time constant is 9.55 s with the vane counter-force, not 6.51 s; the "
                "v0.7.1 estimator gains kp 0.15 / ki 0.01 then fail the tuner's Routh margin (kp / (tau_d ki) = 1.57 < 2) "
                "and the params rule 1 / kp >= tau_d (kp tau_d = 1.43) - re-tuned within both; the vertical gains keep "
                "their linear margins (the vertical plant is unchanged) and are re-checked on the scenarios / campaigns"),
        "vertical_gains_from": "the v0.7.1 tuning (sim/vtol_scenarios.py --tune, 2026-10-21T00:30), kept",
        "vertical_tuning_v071": vt,
        "margins": vertical_margins(vpw, veh.t_max_n, veh.full.mass_kg, tau, actuator_delay_s(p)),
        "est_routh_margin": EST_ROUTH_MARGIN, "est_kp_tau_max": EST_KP_TAU_MAX, "est_grid": {k: list(v) for k, v in EST_GRID.items()},
        "est_cases": list(EST_CASES), "cost": COST_DOC,
        "stage1e_table_columns": ["est_kp", "est_ki", "est_accel_lpf_hz", "feasible", "J1e (None: infeasible)",
                                  "loop period s", "loop zeta", "mean RMS tilt-estimate error deg"],
        "stage1e_table": [[*(round(r["g"][k], 6) for k in EGAIN_KEYS), r["feasible"], r["J1e"], r["loop"]["period_s"],
                           r["loop"]["zeta"], r["mean_rms_tilt_est_err_deg"]] for r in table],
        "stage1e_best": {"g": ranked[0]["g"], "J1e": ranked[0]["J1e"]},
        "stage2_campaign_seed": TUNE_MC_SEED, "stage2_runs_per_finalist": {"field": TUNE_MC_N, "demo": TUNE_MC_N_DEMO},
        "stage2_sampler": "sim.vtol_campaign.sample (fix round: thrust x0.80-1.25 + voltage exponent 0-2, CAD CG + 0-3 mm, "
                          "swirl x0-2, wind swing period 10-80 s)",
        "stage2": stage2, "J2_field": win_f["J2_field"], "J2_demo": win_d["J2_demo"], "field_winner": win_f["g"],
        "demo_winner": win_d["g"],
        "previous_estimator": e_prev, "previous_estimator_feasible": estimator_feasible(e_prev, tau_d),
        "previous_estimator_loop": estimator_loop(e_prev, tau_d), "winner_loop": estimator_loop(win_f["g"], tau_d),
        "profiles_note": ("the demo profile uses different gains (under 'profiles.demo')" if demo_diff else
                          "the same gains serve both profiles: the demo winner is not better than the field gains on the "
                          "same demo samples by more than PROFILE_SIGNIFICANCE_SE paired standard errors"),
        "profiles_check": {"demo_winner": win_d["g"], "paired_mean_diff_demo_minus_field_gains": float(dd.mean()) if dd.size else 0.0,
                           "paired_se": se, "n": int(dd.size), "rule_se": PROFILE_SIGNIFICANCE_SE},
        "evaluations": len(res1) + len(res_f) + len(res_d), "runtime_s": round(time.perf_counter() - t0, 1),
        "note": ("vertical-controller gains for sim.vertical_ctrl (VerticalParams overrides) and the NAV estimator gains for "
                 "sim.estimator.GatedMahonyEstimator (est_kp, est_ki, est_accel_lpf_hz; the gate stays params accel_gate); "
                 "params.json keeps gains_ASSUMED / nav_estimator *_ASSUMED as the documented starting point - the firmware "
                 "generator decides which set it emits"),
    }
    out = _clean(out)
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, indent=1)
    log(f"wrote {out_path}: {{{', '.join(f'{k}: {out[k]}' for k in GAIN_KEYS)}}} J2 field {win_f['J2_field']:.3f}, "
        f"demo winner {win_d['g']} (paired diff {float(dd.mean()) if dd.size else 0.0:+.3f} +- {se:.3f}); "
        f"{out['_meta']['evaluations']} runs in {out['_meta']['runtime_s']:.0f} s")
    return out


# --------------------------------------------------------------------------- #
# v0.8: position-loop gain tuning (PGAIN_KEYS) - tuning seeds only; validation on fresh seeds (review v0.8 (4))
# --------------------------------------------------------------------------- #
PGRID: dict[str, tuple[float, ...]] = {
    "kp_pos": (0.5, 0.65, 0.8, 0.9),
    "kv": (1.4, 1.8, 2.2, 2.6),
    "ki": (0.15, 0.25, 0.35),
    "kd": (0.2, 0.35, 0.5),
}
GM_MIN_H_X = 2.0                     # horizontal loop gain margin >= 2 (6 dB), with PM_MIN_H_DEG
PTUNE_SEED0 = 300                    # stage-1 noise / receiver seeds 300.. (the scenario set flies 70 / 80, the campaigns
                                     # their own seeds; validation: PVALID_MC_SEED, never seen by the tuner)
PTUNE_MC_SEED = 808                  # stage-2 campaign seed (tuning)
PVALID_MC_SEED = 4242                # validation campaign seed (fresh: no tuning iteration used it)
PTUNE_MC_N = 30                      # stage-2 runs per profile per finalist
PTUNE_FINALISTS = 5
ADVERSE_PLANT = {"thrust_scale": 0.96, "mass_scale": 1.014, "servo_tau_scale": 1.45, "swirl_scale": 1.6}
                                     # a weak / slow / high-swirl plant (inside the campaign spread; the smoke campaign's
                                     # GPS-only I3 spin abort had this plant) - stage-1 robustness case
PCOST_DOC = ("position tuning, per run (GPS build): c = sk_rms / 1.0 + 0.5 sk_max / sk_max_crit + nav_rms / 0.5 + "
             "touchdown_vh_ground / 0.5 + 0.25 touchdown_tilt / 5 + 0.25 touchdown_from_hold / 3 + 2 limited_frac_hold + "
             "per push: excursion / 1.5 + return_1m / 6 (not back: 2); penalties 20 [ABORT] + 10 [false touchdown] + 10 [no "
             "LANDED -> SAFE] + 5 [V13 GPS implausible] + 5 [V12 fence] + 3 [touchdown over the ground > 0.5 m/s] + 3 [a push "
             "<= 3 N s not back within 1 m in 6 s, each]; stage 1 = mean over PTUNE_CASES on the feasible grid (PM >= "
             "PM_MIN_H_DEG, GM >= GM_MIN_H_X of the linear horizontal loop); stage 2 = mean over PTUNE_MC_N field + demo "
             "campaign runs (seed PTUNE_MC_SEED, GPS on; demo pushes 'light' = the live demo as planned) of the PTUNE_FINALISTS best + the prototype gains; the winner = "
             "the lowest stage-2 mean with the fewest ABORTs among the sets that meet the margin constraint "
             "(select_position_winner: the prototype reference cannot win when its phase margin is below it)")


def ptune_cases() -> tuple[dict[str, Any], ...]:
    """Stage-1 cases: scenario name in the v0.8 set + overrides (own seeds PTUNE_SEED0 + index)."""
    return ({"name": "G_calm"}, {"name": "G_wind5"}, {"name": "G_wind_limit"}, {"name": "V_wind_swing"},
            {"name": "G_heading_30"}, {"name": "GD_push_band_3Ns"}, {"name": "GD_push_angled_low"},
            {"name": "GD_wind4_push"}, {"name": "G_calm", "plant": ADVERSE_PLANT},
            {"name": "GD_push_band_3Ns", "plant": ADVERSE_PLANT})


def pos_cost_one(m: dict[str, Any], crit: dict[str, float]) -> tuple[float, dict[str, float]]:
    """Position-tuning cost of one run (PCOST_DOC)."""
    def val(x: Any, default: float) -> float:
        return default if x is None else float(x)
    pushes = [r for r in m.get("pushes", []) if r.get("in_flight")]
    pc = 0.0
    pf = 0
    for r in pushes:
        ret = r.get("return_1p0m_s")
        pc += val(r.get("excursion_m"), 3.0) / 1.5 + (2.0 if ret is None else ret / crit["push_return_within_1m_s"])
        if (r.get("impulse_ns") or 0.0) <= crit["push_rule_max_impulse_ns"] + 1e-9 and (
                ret is None or ret > crit["push_return_within_1m_s"]):
            pf += 1
    il = " ".join(m.get("interlocks") or [])
    tdv = m.get("touchdown_vh")
    parts = {"sk_rms": val(m.get("sk_rms_m"), 3.0) / crit["station_keeping_rms_p50_m"],
             "sk_max": 0.5 * val(m.get("sk_max_m"), 6.0) / crit["station_keeping_max_m"],
             "nav_rms": val(m.get("nav_rms_m"), 1.5) / crit["nav_frame_rms_m"],
             "td_vh": val(tdv, 2.0) / crit["touchdown_ground_speed_mps"],
             "td_tilt": 0.25 * val(m.get("touchdown_tilt_deg"), 20.0) / crit["touchdown_tilt_max_deg"],
             "td_hold": 0.25 * val(m.get("touchdown_from_hold_m"), 10.0) / crit["touchdown_from_hold_point_m"],
             "limited": 2.0 * val(m.get("limited_frac_hold"), 1.0), "push": pc,
             "pen": (20.0 * (m.get("t_abort_s") is not None) + 10.0 * bool(m.get("false_touchdown"))
                     + 10.0 * (not (m.get("t_landed_s") is not None and m.get("t_safe_after_landed_s") is not None))
                     + 5.0 * ("V13" in il) + 5.0 * ("V12" in il)
                     + 3.0 * (tdv is None or tdv > crit["touchdown_ground_speed_mps"]) + 3.0 * pf)}
    return float(sum(parts.values())), parts


PTUNE_KEEP = ("sk_rms_m", "sk_max_m", "nav_rms_m", "touchdown_vh", "touchdown_tilt_deg", "touchdown_from_hold_m",
              "limited_frac_hold", "t_abort_s", "abort_reason", "interlocks", "psi_err_max_deg", "tilt_max_hold_deg",
              "false_touchdown")


def _ptune_eval(g: dict[str, float], case_idx: int) -> dict[str, Any]:
    veh = vehicle()
    p = veh.params
    case = ptune_cases()[case_idx]
    vg, _ = load_vgains()
    sc = next(s for s, _ in scenarios(p, vg) if s.name == case["name"])
    sc = replace(sc, seed=PTUNE_SEED0 + case_idx, pgains={**(sc.pgains or {}), **g}, **(case.get("plant") or {}))
    res, m = run_scenario(sc)
    crit = v08_criteria(p, m.get("profile"))
    cost, parts = pos_cost_one(m, crit)
    pr = [{k: r.get(k) for k in ("excursion_m", "return_1p0m_s", "impulse_ns")} for r in m.get("pushes", [])]
    return _clean({"g": g, "case": case_idx, "cost": cost, "parts": parts, "m": {k: m.get(k) for k in PTUNE_KEEP},
                   "pushes": pr})


def _ptune_mc(g: dict[str, float], j: int, profile: str, seed: int = PTUNE_MC_SEED) -> dict[str, Any]:
    from sim import vtol_campaign as vc          # lazy: vtol_campaign imports this module
    veh = vehicle()
    vg, _ = load_vgains()
    s = vc.sample(seed, j, profile=profile, pushes="light" if profile == "demo" else "any")   # demo: as planned
    sc = vc.build_scenario(veh.params, s, vg, "on")
    sc = replace(sc, pgains={**(sc.pgains or {}), **g})
    res, m = run_scenario(sc)
    crit = v08_criteria(veh.params, m.get("profile"))
    cost, parts = pos_cost_one(m, crit)
    return _clean({"g": g, "j": j, "profile": profile, "cost": cost, "parts": parts,
                   "m": {k: m.get(k) for k in PTUNE_KEEP}})


def _ptune_mc_field(g: dict[str, float], j: int) -> dict[str, Any]:
    return _ptune_mc(g, j, "field")


def _ptune_mc_demo(g: dict[str, float], j: int) -> dict[str, Any]:
    return _ptune_mc(g, j, "demo")


def prototype_pgains() -> dict[str, float]:
    """The prototype's tuned gains (sim.position_ctrl module constants / params gains_ASSUMED when recorded)."""
    veh = vehicle()
    pp = PositionParams.from_params(veh.params, veh.full.mass_kg, veh.t_max_n)
    return {k: float(getattr(pp, k)) for k in PGAIN_KEYS}


def select_position_winner(stage2: list[dict[str, Any]]) -> dict[str, Any]:
    """The stage-2 winner: among the sets that meet the linear margin constraint (PM >= PM_MIN_H_DEG, GM >= GM_MIN_H_X;
    the prototype reference cannot win when it does not), the fewest ABORTs, then the lowest J2 = field + demo mean."""
    def feasible(r: dict[str, Any]) -> bool:
        mg = r.get("margins") or {}
        return (mg.get("phase_margin_deg") or -1e9) >= PM_MIN_H_DEG and (mg.get("gain_margin_x") or 0.0) >= GM_MIN_H_X
    cand = [r for r in stage2 if feasible(r)] or list(stage2)
    return min(cand, key=lambda r: (r["aborts"], r["J2"]))


def tune_position(workers: int, out_path: str = VTOL_GAINS_JSON, log=print) -> dict[str, Any]:
    """Stage 0: the PGRID sets with the linear horizontal margins (``horizontal_margins``: PM >= PM_MIN_H_DEG, GM >=
    GM_MIN_H_X); stage 1: every feasible set + the prototype gains on ``ptune_cases`` (tuning seeds); stage 2: the
    PTUNE_FINALISTS best + the prototype on PTUNE_MC_N field + PTUNE_MC_N demo campaign runs (GPS on, seed
    PTUNE_MC_SEED, common random numbers); the winner (select_position_winner) is written to out_path under
    "position" (the vertical / estimator gains and their meta are kept).  It is a CANDIDATE: validate_position (the
    fresh seed) decides whether it replaces the prototype gains (v0.8: it did not - _meta_position.validation_gate)."""
    t0 = time.perf_counter()
    base = prototype_pgains()
    grid = _grid(PGRID)
    feas, table = [], []
    for g in grid + [base]:
        mg = horizontal_margins(g)
        ok = (mg["phase_margin_deg"] or -1e9) >= PM_MIN_H_DEG and (mg["gain_margin_x"] or 0.0) >= GM_MIN_H_X
        table.append({"g": g, "pm_deg": mg["phase_margin_deg"], "gm_x": mg["gain_margin_x"],
                      "wc_rad_s": mg["crossover_rad_s"], "feasible": ok})
        if ok and _gkey(g, PGAIN_KEYS) not in {_gkey(f, PGAIN_KEYS) for f in feas}:
            feas.append(g)
    cands = feas + ([base] if _gkey(base, PGAIN_KEYS) not in {_gkey(f, PGAIN_KEYS) for f in feas} else [])
    nc = len(ptune_cases())
    log(f"position tuning stage 1: {len(grid)} grid sets, {len(feas)} feasible (PM >= {PM_MIN_H_DEG} deg, GM >= "
        f"{GM_MIN_H_X}) + the prototype {base} x {nc} cases, {workers} workers")
    jobs = [(g, i) for g in cands for i in range(nc)]
    res1 = pool_map(_ptune_eval, jobs, workers, log=log)
    s1 = []
    for g in cands:
        rs = [r for r in res1 if _gkey(r["g"], PGAIN_KEYS) == _gkey(g, PGAIN_KEYS)]
        s1.append({"g": g, "J1": float(np.mean([r["cost"] for r in rs])),
                   "aborts": int(sum(r["m"].get("t_abort_s") is not None for r in rs)),
                   "margins": next(x for x in table if _gkey(x["g"], PGAIN_KEYS) == _gkey(g, PGAIN_KEYS)),
                   "runs": rs})
    s1.sort(key=lambda e: (e["aborts"], e["J1"]))
    log("  stage 1 best: " + "; ".join(f"{e['g']} J1 {e['J1']:.3f} aborts {e['aborts']}" for e in s1[:PTUNE_FINALISTS])
        + f" ({time.perf_counter() - t0:.0f} s)")
    fins = [e["g"] for e in s1[:PTUNE_FINALISTS]]
    if _gkey(base, PGAIN_KEYS) not in {_gkey(f, PGAIN_KEYS) for f in fins}:
        fins.append(base)
    jf = [(g, j) for g in fins for j in range(PTUNE_MC_N)]
    rf = pool_map(_ptune_mc_field, jf, workers, log=log)
    rd = pool_map(_ptune_mc_demo, jf, workers, log=log)
    s2 = []
    for g in fins:
        row: dict[str, Any] = {"g": g, "prototype": _gkey(g, PGAIN_KEYS) == _gkey(base, PGAIN_KEYS)}
        for prof, rr in (("field", rf), ("demo", rd)):
            rs = [r for r in rr if _gkey(r["g"], PGAIN_KEYS) == _gkey(g, PGAIN_KEYS)]
            row[f"J2_{prof}"] = float(np.mean([r["cost"] for r in rs]))
            row[f"aborts_{prof}"] = int(sum(r["m"].get("t_abort_s") is not None for r in rs))
            row[f"v13_{prof}"] = int(sum("V13" in " ".join(r["m"].get("interlocks") or []) for r in rs))
            for k in ("sk_rms_m", "sk_max_m", "nav_rms_m", "touchdown_vh", "touchdown_from_hold_m"):
                vals = [r["m"][k] for r in rs if r["m"].get(k) is not None]
                row[f"median_{k}_{prof}"] = float(np.median(vals)) if vals else None
        row["J2"] = row["J2_field"] + row["J2_demo"]
        row["aborts"] = row["aborts_field"] + row["aborts_demo"]
        row["margins"] = horizontal_margins(g)
        s2.append(row)
        log(f"  stage 2 {g}{' (prototype)' if row['prototype'] else ''}: J2 field {row['J2_field']:.3f} demo "
            f"{row['J2_demo']:.3f}, aborts {row['aborts_field']}/{row['aborts_demo']}, V13 {row['v13_field']}/"
            f"{row['v13_demo']}, PM {row['margins']['phase_margin_deg']:.1f} deg")
    win = select_position_winner(s2)
    d: dict[str, Any] = {}
    if os.path.exists(out_path):
        with open(out_path, "r", encoding="utf-8") as f:
            d = json.load(f)
    d["position"] = {k: round(float(win["g"][k]), 6) for k in PGAIN_KEYS}
    d["_meta_position"] = {
        "written_by": "sim/vtol_scenarios.py --tune-pos (v0.8)", "date": _dt.datetime.now().isoformat(timespec="seconds"),
        "params_sha256": params_sha256(), "cost": PCOST_DOC, "grid": {k: list(v) for k, v in PGRID.items()},
        "pm_min_deg": PM_MIN_H_DEG, "gm_min_x": GM_MIN_H_X, "stage1_cases": list(ptune_cases()),
        "stage1_seed0": PTUNE_SEED0, "stage2_campaign_seed": PTUNE_MC_SEED, "stage2_runs_per_profile": PTUNE_MC_N,
        "validation_campaign_seed": PVALID_MC_SEED, "prototype_gains": base, "winner": win["g"],
        "winner_margins": horizontal_margins(win["g"]), "winner_margins_gps_direct_whatif": horizontal_margins(
            win["g"], gps_direct=True), "prototype_margins": horizontal_margins(base),
        "stage0_table": table, "stage1": [{k: v for k, v in e.items() if k != "runs"} for e in s1],
        "stage2": s2, "evaluations": len(res1) + len(rf) + len(rd), "runtime_s": round(time.perf_counter() - t0, 1),
        "note": ("position-loop gains for sim.position_ctrl (PositionParams overrides; params mission.vtol.position_hold."
                 "gains_ASSUMED keeps the prototype's starting point - the architect / firmware generator decides; the "
                 "tilt fade, guards and limits are the reference's / params' (not tuned here)")}
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(_clean(d), f, indent=1)
    log(f"wrote {out_path}: position {d['position']} (prototype {base}); {d['_meta_position']['evaluations']} runs in "
        f"{d['_meta_position']['runtime_s']:.0f} s")
    return d


PVALID_N = 40                        # validation runs per profile


def validate_position(workers: int, n: int = PVALID_N, log=print) -> dict[str, Any]:
    """The validation gate of a tuned position-gain candidate (``tune_position``): on the FRESH campaign seed
    PVALID_MC_SEED (no tuning iteration used it), n field + n demo (pushes as planned) runs, paired: the candidate in
    sim/out/vtol_gains.json "_meta_position.candidate" (else "position"), the prototype gains, and the v0.8 build without
    the GPS.  Writes sim/out/vtol/validation_v08/ (campaigns + validation_summary.json).  The candidate is adopted only if
    it is better by > 2 paired standard errors of the cost with no more ABORTs - the decision is recorded by hand in the
    gains file (``_meta_position.validation_gate``), not here."""
    from sim import vtol_campaign as vc          # lazy: vtol_campaign imports this module
    vg, src = load_vgains()
    with open(VTOL_GAINS_JSON, "r", encoding="utf-8") as f:
        mp = (json.load(f).get("_meta_position") or {})
    cand = mp.get("candidate") or (vg or {}).get("_position") or prototype_pgains()
    proto = prototype_pgains()
    base = os.path.join(OUT_DIR, "validation_v08")
    out: dict[str, Any] = {"_meta": {"seed": PVALID_MC_SEED, "n": n, "candidate_position": cand, "prototype_position": proto,
                                     "vgains_source": src, "demo_pushes": "light (the live demo as planned)",
                                     "note": "fresh seed: no tuning iteration used it (tuning: scenario seeds 300+, campaign "
                                             "seed 808)"}}
    p = vehicle().params
    for prof in ("field", "demo"):
        pushes = "light" if prof == "demo" else "any"
        runs = {}
        for key, pos, mode, sub in (("candidate", cand, "on", f"{prof}_gps_on_candidate"),
                                    ("prototype", proto, "on", f"{prof}_gps_on_prototype"),
                                    ("gps_off", proto, "off", f"{prof}_gps_off")):
            r = vc.run_campaign(n, PVALID_MC_SEED, workers, "gated_mahony", prof, dict(vg or {}, _position=pos), src,
                                os.path.join(base, sub), lambda s: None, pushes, mode)
            runs[key] = r["rows"]
        crit = v08_criteria(p, prof)
        summ: dict[str, Any] = {}
        for key, rr in runs.items():
            costs = [pos_cost_one(r["metrics"] | {"pushes": r["pushes"]}, crit)[0] for r in rr]
            summ[key] = {"pass": sum(r["verdict"] == "PASS" for r in rr),
                         "aborts": [r["i"] for r in rr if r["metrics"].get("t_abort_s") is not None],
                         "v13": [r["i"] for r in rr if "V13" in " ".join(r["metrics"].get("interlocks") or [])],
                         "touchdown_vh_ground_le_0p5": sum(1 for r in rr if (r["metrics"].get("touchdown_vh") or 9) <= 0.5),
                         "mean_cost": float(np.mean(costs)), "check_pass_rate": vc.summarize(rr)["check_pass_rate"]}
        cc = np.array([pos_cost_one(r["metrics"] | {"pushes": r["pushes"]}, crit)[0] for r in runs["candidate"]])
        cp = np.array([pos_cost_one(r["metrics"] | {"pushes": r["pushes"]}, crit)[0] for r in runs["prototype"]])
        dd = cc - cp
        summ["paired"] = {"cost_candidate_minus_prototype": {"mean": float(dd.mean()),
                                                              "se": float(dd.std(ddof=1) / math.sqrt(dd.size))},
                          "aborts_only_with_gps_candidate": sorted(set(summ["candidate"]["aborts"]) - set(summ["gps_off"]["aborts"])),
                          "aborts_only_with_gps_prototype": sorted(set(summ["prototype"]["aborts"]) - set(summ["gps_off"]["aborts"]))}
        out[prof] = summ
        log(f"validation {prof}: " + json.dumps({k: {kk: v[kk] for kk in ("pass", "aborts", "mean_cost")}
                                                 for k, v in summ.items() if k != "paired"}) + f" {summ['paired']}")
    os.makedirs(base, exist_ok=True)
    with open(os.path.join(base, "validation_summary_rerun.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(_clean(out), f, indent=1, ensure_ascii=False)
    return out


# --------------------------------------------------------------------------- #
# v0.8 physics-review fix: the true station-keeping RMS as a statistic per scenario family
# --------------------------------------------------------------------------- #
SK_STATS_FAMILIES = ("G_calm", "G_wind3", "G_wind5", "G_wind_limit", "V_wind_swing", "GD_calm", "GD_wind4",
                     "GD_push_band_3Ns")
SK_STATS_SEED0 = 500                 # seeds 500.. (not the scenario seeds 70 / 80, the tuning 300.. / 808, the validation
                                     # 4242 or the campaigns' run seeds)


def _sk_stats_one(name: str, seed: int, vgains: dict | None) -> dict[str, Any]:
    """Worker: scenario ``name`` flown with seed ``seed`` (GPS error, sensor and turbulence draws), judged per run."""
    p = vehicle().params
    sc, _ = next((s, e) for s, e in scenarios(p, vgains) if s.name == name)
    sc = replace(sc, seed=int(seed))
    res, m = run_scenario(sc)
    crit = v08_criteria(p, m.get("profile"))
    c = v08_checks(m, crit)
    pc = push_v08_checks(m, crit) if sc.pushes else {}
    return _clean({"name": name, "seed": int(seed), "failed": [k for k, v in {**c, **pc}.items() if v is False],
                   "abort": m.get("t_abort_s") is not None, "interlocks": m.get("interlocks"),
                   **{k: m.get(k) for k in ("sk_rms_m", "sk_max_m", "nav_rms_m", "touchdown_vh_ground",
                                            "touchdown_tilt_deg", "touchdown_tip_margin_mps", "touchdown_from_hold_m",
                                            "psi_err_end_deg")}})


def sk_stats_study(vgains: dict | None, workers: int, n: int = V08_SK_STATS_MIN_SEEDS, out_dir: str | None = OUT_DIR,
                   families: tuple[str, ...] = SK_STATS_FAMILIES, log=print) -> dict[str, Any]:
    """params criteria_v08_ASSUMED note (architect 18:10 after the v0.8 physics review): the TRUE station-keeping RMS
    is dominated by the ASSUMED GPS wander draw (one run = one draw), so it is judged as the p50 over >= 20 seeds per
    scenario family against V08_SK_RMS_MAX_M (1.0 m) and reported with its p90 - not loosened, moved to the statistic it
    measures; every other v0.8 criterion is still judged per run (the counts of runs failing each are listed)."""
    def pct(v: list, q: float) -> float | None:
        v = [x for x in v if x is not None]
        return float(np.percentile(v, q)) if v else None
    jobs = [(nm, SK_STATS_SEED0 + i, vgains) for nm in families for i in range(n)]
    rows = pool_map(_sk_stats_one, jobs, workers, log=log)
    fam: dict[str, Any] = {}
    for nm in families:
        rr = [r for r in rows if r["name"] == nm]
        sk = [r["sk_rms_m"] for r in rr]
        fails: dict[str, int] = {}
        for r in rr:
            for k in r["failed"]:
                fails[k] = fails.get(k, 0) + 1
        p50 = pct(sk, 50)
        fam[nm] = {"n": len(rr), "seeds": [r["seed"] for r in rr], "sk_rms_p50_m": p50, "sk_rms_p90_m": pct(sk, 90),
                   "sk_rms_max_m": pct(sk, 100), "sk_max_p50_m": pct([r["sk_max_m"] for r in rr], 50),
                   "sk_max_max_m": pct([r["sk_max_m"] for r in rr], 100),
                   "nav_rms_p50_m": pct([r["nav_rms_m"] for r in rr], 50),
                   "nav_rms_max_m": pct([r["nav_rms_m"] for r in rr], 100),
                   "touchdown_vh_p50_mps": pct([r["touchdown_vh_ground"] for r in rr], 50),
                   "touchdown_vh_p90_mps": pct([r["touchdown_vh_ground"] for r in rr], 90),
                   "tip_margin_min_mps": min((r["touchdown_tip_margin_mps"] for r in rr
                                              if r["touchdown_tip_margin_mps"] is not None), default=None),
                   "runs_all_per_run_checks_met": sum(1 for r in rr if not r["failed"]),
                   "per_run_check_failures": fails, "aborts": sum(1 for r in rr if r["abort"]),
                   "target_p50_m": V08_SK_RMS_MAX_M,
                   "target_met": None if p50 is None else bool(p50 <= V08_SK_RMS_MAX_M)}
    out = {"_meta": {"written_by": "sim/vtol_scenarios.py sk_stats_study", "date": _dt.datetime.now().isoformat(
        timespec="seconds"), "params_sha256": params_sha256(), "seeds": f"{SK_STATS_SEED0}..{SK_STATS_SEED0 + n - 1}",
        "criterion": "true station-keeping RMS p50 over >= 20 seeds per scenario family <= 1.0 m (params "
                     "criteria_v08_ASSUMED note); p90 reported; the GPS error model is ASSUMED (Gauss-Markov 2.12 m / "
                     "200 s) - re-derive after a real module is measured"},
           "families": fam, "runs": rows}
    if out_dir:
        with open(os.path.join(out_dir, "sk_rms_stats.json"), "w", encoding="utf-8", newline="\n") as f:
            json.dump(_clean(out), f, indent=1, ensure_ascii=False)
    return out


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--only", default="", help="comma-separated scenario names (default: all)")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--tune", action="store_true", help="tune the vertical + estimator gains first (sim/out/vtol_gains.json)")
    ap.add_argument("--tune-est", action="store_true",
                    help="re-tune the estimator gains only, the vertical gains kept (fix round; sim/out/vtol_gains.json)")
    ap.add_argument("--tune-pos", action="store_true",
                    help="v0.8: tune the position-loop gains (sim/out/vtol_gains.json position)")
    ap.add_argument("--validate-pos", action="store_true",
                    help="v0.8: the validation gate of the position-gain candidate on the fresh seed (validation_v08)")
    ap.add_argument("--tune-only", action="store_true", help="stop after the tuning")
    ap.add_argument("--assumed", action="store_true", help="fly params gains_ASSUMED / nav_estimator, not vtol_gains.json")
    ap.add_argument("--no-export", action="store_true", help="no renderer logs / plots")
    ap.add_argument("--no-envelope", action="store_true", help="skip the push envelope")
    ap.add_argument("--envelope-only", action="store_true", help="only the push envelope")
    ap.add_argument("--no-sk-stats", action="store_true", help="skip the station-keeping RMS statistic (sk_rms_stats.json)")
    ap.add_argument("--sk-stats-only", action="store_true", help="only the station-keeping RMS statistic")
    ap.add_argument("--out", default=OUT_DIR)
    args = ap.parse_args(argv)
    t0 = time.perf_counter()
    os.makedirs(args.out, exist_ok=True)
    if args.tune:
        tune(args.workers)
    if args.tune_est:
        tune_estimator(args.workers)
    if args.tune_pos:
        tune_position(args.workers)
    if args.validate_pos:
        validate_position(args.workers)
    if args.tune_only:
        print(f"done in {time.perf_counter() - t0:.0f} s")
        return 0
    if args.assumed:
        vg, src = None, "params.json mission.vtol.gains_ASSUMED / nav_estimator"
    else:
        vg, src = load_vgains()
    veh = vehicle()
    p = veh.params
    ag = vtol_aero_geometry(veh)
    print("VTOL drag model (v0.7.1, params aero.vtol_*): " + ", ".join(
        f"{k} {v:.4g}" if isinstance(v, float) else f"{k} {v}" for k, v in ag.items()))
    if args.sk_stats_only:
        st = sk_stats_study(vg, args.workers, out_dir=args.out)
        for nm, f in st["families"].items():
            print(f"sk stats {nm}: n {f['n']}, true RMS p50 {_f(f['sk_rms_p50_m'])} / p90 {_f(f['sk_rms_p90_m'])} m "
                  f"(target p50 <= {f['target_p50_m']:g}: {f['target_met']}), per-run checks met {f['runs_all_per_run_checks_met']}"
                  f"/{f['n']} {f['per_run_check_failures']}")
        print(f"done in {time.perf_counter() - t0:.0f} s")
        return 0
    if args.envelope_only:
        env = push_envelope(vg, args.workers, args.out)
        mpath = os.path.join(args.out, "vtol_metrics.json")
        if os.path.exists(mpath):                     # keep the scenario metrics file's study section current
            with open(mpath, "r", encoding="utf-8") as f:
                mj = json.load(f)
            mj.setdefault("studies", {})["push_envelope"] = {"recommendation": env["recommendation"],
                                                             "by_height": env["by_height"],
                                                             "recommendation_all_swirl": env["recommendation_all_swirl"],
                                                             "by_height_min_over_swirl": env["by_height_min_over_swirl"],
                                                             "file": "push_envelope.json", "date": env["_meta"]["date"]}
            with open(mpath, "w", encoding="utf-8", newline="\n") as f:
                json.dump(_clean(mj), f, indent=1, ensure_ascii=False)
        print(f"done in {time.perf_counter() - t0:.0f} s")
        return 0
    names = scenario_names(p)
    if args.only:
        want = [s.strip() for s in args.only.split(",") if s.strip()]
        bad = [w for w in want if w not in names]
        if bad:
            raise SystemExit(f"unknown scenario(s) {bad}; known: {names}")
        names = want
    print(f"VTOL scenarios ({len(names)}), gains from {src}: {vg}")
    recs = run_all(names, vg, src, args.out, args.workers, export=not args.no_export, plot=not args.no_export)
    lines = []
    for r in recs:
        line = f"{r['name']}: {r['verdict']} - {r['why']}"
        lines.append(line)
        print(line)
    studies: dict[str, Any] = {}
    if not args.only:
        bs = baro_fail_study(vg, args.workers)
        studies["baro_fail_seeds"] = bs
        line = (f"study baro failure x {bs['n']} seeds: landed {bs['landed']} (by {bs['landed_by']}; |v_z| <= 1 m/s: "
                f"{bs['landed_within_1mps']}, max {_f(bs['touchdown_abs_vz_max'])} m/s), ABORT {bs['abort']} (heights "
                f"{[round(h, 2) for h in bs['abort_heights_m'] if h is not None]} m), climbed > 2 m after the failure "
                f"{bs['flew_up_more_than_2m']} (highest {bs['max_h_after_fault_m']:.1f} m)")
        lines.append(line)
        print(line)
        mpath = os.path.join(args.out, "vtol_metrics.json")
        if args.no_envelope and os.path.exists(mpath):      # keep the last push envelope (sim/out/vtol/push_envelope.json)
            with open(mpath, "r", encoding="utf-8") as f:
                prev = (json.load(f).get("studies") or {}).get("push_envelope")
            if prev:
                studies["push_envelope"] = dict(prev, note="carried over from the previous run (--no-envelope)")
        if not args.no_envelope:
            env = push_envelope(vg, args.workers, args.out, log=lambda s: None)
            studies["push_envelope"] = {"recommendation": env["recommendation"], "by_height": env["by_height"],
                                        "recommendation_all_swirl": env["recommendation_all_swirl"],
                                        "by_height_min_over_swirl": env["by_height_min_over_swirl"],
                                        "file": "push_envelope.json"}
            line = f"push envelope: {env['_meta']['runs']} runs; recommendation {env['recommendation']}"
            lines.append(line)
            print(line)
        if not args.no_sk_stats:
            st = sk_stats_study(vg, args.workers, out_dir=args.out, log=lambda s: None)
            studies["sk_rms_stats"] = {"families": st["families"], "file": "sk_rms_stats.json", "_meta": st["_meta"]}
            for nm, f in st["families"].items():
                line = (f"station keeping statistic {nm} (n {f['n']}, seeds {st['_meta']['seeds']}): true RMS p50 "
                        f"{_f(f['sk_rms_p50_m'])} / p90 {_f(f['sk_rms_p90_m'])} m - target p50 <= {f['target_p50_m']:g} m: "
                        f"{'met' if f['target_met'] else 'NOT met'}; NAV RMS p50 {_f(f['nav_rms_p50_m'])} m; per-run "
                        f"v0.8 checks met {f['runs_all_per_run_checks_met']}/{f['n']} {f['per_run_check_failures']}")
                lines.append(line)
                print(line)
    out = {"_meta": {"written_by": "sim/vtol_scenarios.py (v0.8)", "date": _dt.datetime.now().isoformat(timespec="seconds"),
                     "params_sha256": params_sha256(), "vgains": vg, "vgains_source": src,
                     "criteria": NOMINAL_CRITERIA, "criteria_text": CRITERIA_TEXT, "push_criteria": PUSH_CRITERIA,
                     "push_criteria_text": PUSH_CRITERIA_TEXT, "model_notes": MODEL_NOTES, "aero": ag,
                     "vibration_g_ASSUMED": VTOL_ACCEL_VIBRATION_RMS_G_ASSUMED,
                     "timing": {"vane_delay_s": FCTiming().delay_s(1.0 / float(p["control"]["loop_hz"])),
                                "esc_delay_s": FCTiming().esc_delay_s(1.0 / float(p["control"]["loop_hz"]))},
                     "runtime_s": round(time.perf_counter() - t0, 1)},
           "scenarios": {r["name"]: r for r in recs}, "studies": studies}
    if not args.only:
        with open(os.path.join(args.out, "vtol_metrics.json"), "w", encoding="utf-8", newline="\n") as f:
            json.dump(_clean(out), f, indent=1, ensure_ascii=False)
        with open(os.path.join(args.out, "run_log.txt"), "w", encoding="utf-8", newline="\n") as f:
            f.write(f"gains: {vg} ({src})\n" + "\n".join(lines) + "\n")
    print(f"done in {time.perf_counter() - t0:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
