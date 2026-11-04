"""VTOL Monte-Carlo campaign (SPEC 8.9, v0.7 / v0.7.1): many missions under sampled environments, plants, sensors,
links, operator timings and (demo profile) hand pushes, each judged by ``sim.vtol_scenarios.NOMINAL_CRITERIA`` (and
``PUSH_CRITERIA`` for the pushes).

Field profile (``--profile field``, default): the field nominal mission of ``sim.vtol_scenarios`` (ON -> 5 m, UP x3
-> 8 m, hold with gusts, DOWN, OFF -> landing).  Demo profile (``--profile demo``): the indoor demonstration - ON ->
1.5 m hover (optionally UP once), 1-6 random hand pushes, OFF -> landing.  Every run is flown by
``sim.sim6dof.FlightSim`` (VTOL build v0.7.1: attitude controller -> vanes -> supervisor -> vertical controller ->
ESC, FCU timing, MPU-6050 / MS5611 models, gated Mahony NAV estimator unless ``--estimator`` says otherwise, the VTOL
altitude filter) with the operator pressing the buttons of the real ``GroundAssist`` through ``sim.link_model``.  Run i
draws its variables from ``numpy.random.default_rng([campaign_seed, i])`` only (the demo profile from
``[campaign_seed, i, 0xDE]``), so a run is reproducible from (campaign_seed, i, profile) alone, independent of N, of the
worker count and of the order the pool finishes in.

Sampled (``SAMPLING`` / ``SAMPLING_DEMO``; every range is ASSUMED - a spread to look for failure modes, not a measured
distribution; the nominal value of each is the default of ``sim.sim6dof``):
  environment   field: wind speed / direction, turbulence sigma; a swinging direction (probability, amplitude,
                period, phase), a random-walk direction (rate sigma), a 180 deg reversal (time, duration), a speed
                modulation; 0-4 gusts (speed, duration, direction, start time in HOLD, 1 - cos rise, vertical component).
                demo: indoor drafts only (light wind and turbulence, no gusts)
  pushes        demo: count 1-6, peak force 2-20 N, duration 0.1-0.6 s, height over the reachable body (0.05-0.83 m
                above the exit plane), any direction, a small off-centre contact angle and vertical component,
                half-sine or trapezoid, non-overlapping times in HOLD
  link          packet loss, radio (lora_uart / sx127x)
  sensors       gyro noise density x, gyro-bias wander sigma x, accelerometer bias, accelerometer vibration, baro noise,
                the baro's static-pressure error with the fan running (true value) and its calibration residual (the
                firmware compensation = true - residual)
  plant         mass x (inertia with it), CG offset (the CAD lateral CG + a 0-3 mm residual), thrust x (0.80-1.25: the
                thrust is a PROXY table - fix round, physics review HIGH), a battery-voltage exponent k (0-2; the proxy
                table's 22.2 -> 25.2 V ratio gives 1.61), the EDF residual swirl torque x (0-2), servo time constant x,
                vane zero errors, battery state (resting cell voltage), cross-flow drag x (+-30 %), ram drag x (+-20 %)
  pushes set    demo ``--pushes any`` (default) as above; ``--pushes light`` = the live demo as planned (5-15 N x
                0.2-0.4 s between the intake-window top and the same distance above the CG)
  pad           initial tilt (pad not level) and heading
  operator      ON time, UP timing, DOWN time, OFF delay
The controllers always keep the nominal model (mass, T_max, gains); only the plant / environment changes.

Outcome classes (``outcome_of``; the renderer's tracks contract): pass (landed, every criterion met), landed_fail
(landed - LANDED by the detector or a supervisor rule - with a criterion failed), abort (ABORT / release), not_landed
(no LANDED confirmation), other (incl. a plant that never lifted off: T/W < 1 at the weak end of the thrust spread -
the motor runs on the pad until OFF / V2, LANDED -> SAFE on the pad; ``no_liftoff_runs``).

Outputs (``--out``, default ``sim/out/vtol/campaign_<profile>_n<N>_s<seed>[_<estimator>]``): ``runs.csv`` (one flat row
per run), ``runs.json`` (the sample, metrics, checks, push responses and verdict of every run), ``summary.json`` (pass
rates per criterion, percentiles, drift by wind-speed bin and by wind-swing amplitude, push recovery by force and by
height, interlock / failure-mode counts with run ids, false / unconfirmed touchdowns, rank correlations of the sampled
inputs with the outcomes, the worst runs), ``summary.txt``, ``summary.png`` and ``tracks.npz`` (the compact per-run
tracks for ``tools/campaign_video.py``, 10 Hz).

Usage:  python sim/vtol_campaign.py --n 20 [--profile field|demo] [--pushes any|light] [--seed 2026] [--workers 15]
        [--estimator gated_mahony|pad_gyro|truth_noise] [--assumed] [--out DIR]
        (default output dir campaign_<profile>[_light]_n<N>_s<seed>)

v0.8 (GPS position hold): ``--gps on`` (default: the GPS build), ``off`` (the v0.8 build without the GPS), ``both``
(``run_paired``: every sample flown on and off - common random numbers - plus ``paired.json``: runs that pass / ABORT only
with or only without the GPS), ``v071`` (the v0.7.1 build in the v0.7.1 environment).  The v0.8 variables come from
their own stream [seed, i, V08_STREAM] (``SAMPLING_GPS``, ASSUMED ranges): the receiver (true latency 0.15-0.30 s +
jitter, wander sigma 1.5-3 m and tau 60-300 s, white position / velocity noise, satellite-set steps, random multipath in
30 % of the runs, sky blocked 0-30 %, a 2-8 s outage in HOLD in 10 %), the pad heading set-up error (normal 10 deg,
clipped +-30), the outdoor envelope (turbulence intensity 0.2; field gusts clipped to the operator limit field_gust; the
demo OUTDOORS in 0-4 m/s) and the pole (half the demo pushes along a 2.5-3.5 m pole held 0.3-1.2 m above the ground).
GPS runs are judged on ``sim.vtol_scenarios.v08_checks`` + ``push_v08_checks`` (pushes beyond the 3 N s demo rule
reported, not judged); tracks.npz carries hold_xy, sk_rms_m, sk_max_m, touchdown_from_hold_m, gps_mode.

v0.9 (VTOL hardware round, architect decisions a-f): every run flies the VTOL hardware configuration of its power source
(``sim.vtol_scenarios.vehicle``); the v0.9 plant variables come from their own stream [seed, i, V09_STREAM (, profile tag)]
(``SAMPLING_V09``, ASSUMED ranges): T_max at 22.2 V U(2.14, 3.09) kgf (replaces the thrust_scale 0.80-1.25 spread: the
sample keeps thrust_scale 1.0; ``opts["t_max_kgf_range"]`` narrows it - the harness's thrust_low / thrust_stress groups), the
fan power model P1 .. P2, the rotor inertia x, the vane play per vane (lost motion + gear backlash), the thrust's voltage
exponent U(1, 2) (the voltage dependence is kept; 1.61 = proxy), the build tolerance (mass x U(0.95, 1.05) as before, the
lateral CG residual 0 .. 1.5 mm around the CAD value of the power source).  Profiles: field / demo (battery), tether_test
(battery-less over the tether: demo build profile, hover 1.0-1.5 m, sometimes UP to 2-3 m, indoors without the GPS or
outdoors with it, no pushes), hybrid_demo (tether + buffer: the demo with the light pushes, indoors or outdoors); the
tether rig (anchor height / offset, constant-tension or slack feed, tension, catch-line slack), the supply's current limit
and the loop resistance x, the buffer's rest voltage are sampled for the tether profiles.

Invariant: nothing here changes the vehicle or a criterion - every number in the outputs comes from a run of
``sim.sim6dof`` with the sampled inputs written next to it.
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import json
import math
import os
import sys
import time
from dataclasses import replace
from typing import Any

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(_HERE)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from sim import vtol_scenarios as vs  # noqa: E402
from sim.sim6dof import G0, GpsSim, Gust, Push, SensorConfig, SimOptions, exit_plane_positions  # noqa: E402
from sim.tether import PowerConfig, TetherConfig  # noqa: E402

DEFAULT_SEED = 2026
RADIOS = ("lora_uart", "sx127x")
ESTIMATOR_CHOICES = ("gated_mahony", "pad_gyro", "truth_noise")
TRACK_HZ = 10.0
OUTCOME_NAMES = ("pass", "landed_fail", "abort", "not_landed", "other")
OUTCOME_LABELS_KO = ("착지 - 모든 기준 통과", "착지 - 기준 미달", "비상정지", "착지 확인 안 됨", "이륙 불가·기타")
# "other" (index 4) also holds the runs whose plant cannot fly (fix round: thrust x0.80 and up to the weak end of the
# spread give T/W <= ~1 - no lift-off, or hops that never leave the flare band; the motor runs on the pad, the mission is
# not possible; ``no_liftoff_runs`` / ``cannot_fly_runs`` in the summary)
PUSH_SETS = ("any", "light")
FIX_STREAM = 0xF1C          # random stream of the variables the fix round added (thrust_voltage_exp, swirl_scale)
# ASSUMED sampling ranges (uniform unless noted); documented in the module docstring
SAMPLING: dict[str, Any] = {
    "wind_mps": (0.0, 6.0),                 # steady wind near the ground (params aero.wind_mps_default = 3)
    "wind_dir_deg": (0.0, 360.0),
    "turbulence_sigma_mps": (0.0, 1.0),     # horizontal; vertical = 0.5 x (sim6dof ASSUMED ratio)
    "p_swing": 0.5,                         # probability of a swinging wind direction ...
    "wind_swing_deg": (10.0, 90.0),         # ... amplitude ...
    "wind_swing_period_s": (10.0, 80.0),    # ... period (phase uniform 0..360); fix round: to 80 s, past the ~54 s period
                                            # of the estimator-drag loop (tau_d 9.55 s; the v0.7.1 range 10-40 s missed it)
    "p_walk": 0.3,                          # probability of a random-walk direction ...
    "wind_dir_rate_sigma_dps": (0.5, 5.0),  # ... rate sigma
    "p_reversal": 0.15,                     # probability of a 180 deg reversal ...
    "wind_reversal_t_s": (12.0, 26.0),      # ... start (in HOLD) ...
    "wind_reversal_s": (2.0, 10.0),         # ... duration
    "p_speed_mod": 0.3,                     # probability of a speed modulation ...
    "wind_speed_mod_frac": (0.1, 0.4),      # ... amplitude (fraction of the mean) ...
    "wind_speed_mod_period_s": (8.0, 30.0), # ... period
    "n_gusts": (0, 4),                      # integer, inclusive
    "gust_mps": (2.0, 10.0),
    "gust_duration_s": (0.3, 2.0),
    "gust_t0_s": (13.0, 26.0),              # absolute; the vehicle holds 8 m from ~13 s to the OFF
    "gust_rise_s": (0.0, 0.5),
    "gust_vertical_mps": (-1.0, 1.0),
    "packet_loss": (0.0, 0.3),
    "gyro_noise_scale": (0.5, 2.0),         # x the MPU-6050 datasheet density
    "gyro_bias_sigma_scale": (0.25, 1.5),   # x the ASSUMED 0.25 deg/s Gauss-Markov bias wander
    "accel_bias_g": (0.0, 0.02),
    "vibration_g": (0.0, 0.3),              # accelerometer vibration RMS at hover (0.1 g = the scenario value)
    "baro_noise_m": (0.1, 0.5),
    "baro_thrust_offset_m": (0.0, 1.5),     # true static-pressure error at hover thrust (params note: uncalibrated 0..1.5)
    "baro_comp_residual_sigma_m": 0.1,      # tether-calibration residual, normal, clipped +-0.3 m: comp = true - residual
    "mass_scale": (0.95, 1.05),
    "cg_offset_mm": (0.0, 1.5),             # radial, random direction, AROUND the CAD lateral CG (fix round; v0.7.1: around
                                            # the axis) - build tolerance (v0.9 architect decision e: +-1.5 mm; was 0-3 mm)
    # fix round (physics review HIGH): the thrust is a PROXY table (bench test pending; it gives +23 % on a fresh 25.2 V
    # pack); x0.80 cannot lift off (T/W 0.96), x1.25 stopped the motor in the air before v0.7.1b
    "thrust_scale": (0.80, 1.25),
    "thrust_voltage_exp": (1.0, 2.0),       # T x (V_cell / 3.7 V)^k (v0.9: the voltage dependence is kept, was 0-2); the proxy table's 22.2 -> 25.2 V
                                            # ratio gives k = 1.61 (``proxy_voltage_exponent``); 2 = T ~ omega^2 ~ V^2
    "swirl_scale": (0.0, 2.0),              # EDF residual swirl torque x (fraction 0 .. 0.30; params 0.15 ASSUMED)
    "servo_tau_scale": (0.7, 1.5),
    "crossflow_cd_scale": (0.7, 1.3),       # +-30 % on aero.vtol_crossflow_cd_ASSUMED
    "ram_drag_scale": (0.8, 1.2),           # +-20 % on the EDF ram drag
    "vane_trim_sigma_deg": 1.0,             # normal, clipped to +-3 deg, per vane
    "batt_v_rest_cell": (3.85, 4.20),
    "initial_tilt_deg": (0.0, 3.0),         # below arm_tilt_max_deg (ON is refused above it)
    "t_on_s": (0.5, 3.0),
    "up_after_s": (0.5, 3.0),
    "up_gap_s": (0.3, 1.5),
    "t_down_s": (24.0, 28.0),
    "off_after_down_s": (1.5, 4.0),
}
# demo profile (indoor, no wind available; a person pushes the hovering vehicle) - ASSUMED, only the keys that differ
SAMPLING_DEMO: dict[str, Any] = {
    "wind_mps": (0.0, 0.5),                 # indoor drafts
    "turbulence_sigma_mps": (0.0, 0.2),
    "n_pushes": (1, 6),                     # integer, inclusive
    "push_force_n": (2.0, 20.0),
    "push_duration_s": (0.1, 0.6),
    "push_z_body_m": (0.05, 0.83),          # anywhere a pole can reach on the body (exit plane .. below the nose tip)
    "push_offset_sigma_deg": 10.0,          # off-centre contact angle, normal, clipped +-30 deg (needs friction tan(off))
    "push_vertical_sigma_n": 1.0,           # normal, clipped +-3 N
    "p_trapezoid": 0.3,                     # else half-sine
    "push_rise_s": (0.05, 0.15),
    "push_gap_s": 1.0,                      # pushes start >= this after the previous one ended
    "t_first_push_after_on_s": (7.0, 9.0),  # ON + spool 2 s + climb 3 s ~ HOLD at ON + 5.7 s
    # v0.9 campaign round (bug fix): the v0.9 fix-round take-off (two-slope ramp, thrust search, reference held until the
    # lift-off, approach slow-down) reaches HOLD at sample t_on + 8.3 s (p50; p95 8.7-9.0 s in the fix round's EVAL /
    # FRESH2 demo groups), so ON + 7-9 s put the first push into the CLIMB in 38-44 of 60 demo runs, against the
    # documented "pushes in HOLD"; the same U draw is mapped into ON + 10-12 s (>= 1 s after the p95 HOLD) on the VTOL
    # configuration (``push_timing`` "v09"; "v081" keeps the old window - ``sample(..., push_timing=...)``)
    "t_first_push_after_on_s_v09": (10.0, 12.0),
    "hold_s": (20.0, 25.0),                 # ON -> OFF; motor time at OFF <= hold - 2.7 s < V2's 30 s
    "p_up": 0.3,                            # probability of one UP (1.5 -> 2.0 m) at ON + 6..8 s
}
# fix round: the live demo as planned (user 2026-10-20: "a person pushes it lightly", 5-15 N for 0.2-0.4 s) - pushes
# between the intake-window top and the same distance above the CG (z_window_top .. 2 z_cg - z_window_top = 0.327 ..
# 0.425 m: clear of the fan windows, symmetric about the CG); ``--pushes light`` (ASSUMED ranges, the user's words)
SAMPLING_DEMO_LIGHT: dict[str, Any] = {
    "push_force_n": (5.0, 15.0),
    "push_duration_s": (0.2, 0.4),
}
# v0.8 (GPS position hold): the receiver, the set-up, the outdoor envelope and the pole - own stream [seed, i, V08_STREAM
# (, 0xDE)], so every v0.7.1 / fix-round draw of run (seed, i, profile) is kept (ASSUMED ranges; the receiver's nominal
# values are sim.sim6dof.GpsSim's)
V08_STREAM = 0x0E8
GPS_MODES = ("on", "off", "both", "v071")   # GPS build / the v0.8 build without the GPS / paired on + off / the v0.7.1 build
SAMPLING_GPS: dict[str, Any] = {
    "latency_s": (0.15, 0.30),              # the true latency (the NAV assumes params 0.22 s; AP u-blox 6 0.22, M8 0.12)
    "latency_jitter_s": (0.0, 0.05),
    "pos_gm_sigma_m": (1.5, 3.0),           # around DS 2.5 m CEP = 2.12 m per axis ("GPS quality")
    "pos_gm_tau_s": (60.0, 300.0),          # no data sheet value (researcher: 100-300 s)
    "pos_white_m": (0.1, 0.4),
    "vel_white_mps": (0.05, 0.30),          # DS 0.1 m/s CEP .. the review's 0.3 m/s (EDF vibration on a crystal receiver)
    "vel_gm_sigma_mps": (0.0, 0.05),
    "step_rate_hz": (0.0, 1.0 / 60.0),      # satellite-set changes
    "p_multipath": 0.3,                     # probability of random multipath jumps in a run ...
    "multipath_rate_hz": (1.0 / 60.0, 1.0 / 15.0),   # ... their rate (1-4 m, 2-10 s each: GpsSim defaults)
    "sky_blocked": (0.0, 0.3),              # the field: open sky up to a third blocked (a tree line, buildings)
    "p_outage": 0.1,                        # probability of a GPS outage (no fix) in HOLD ...
    "outage_s": (2.0, 8.0),                 # ... its length (start uniform in the hold)
    "heading_err_sigma_deg": 10.0,          # pad set-up error, normal, clipped +-30 deg (the review's design budget; phone
                                            # compass +-5 deg + a set-up mistake)
    "turbulence_intensity": 0.2,            # review v0.8 (5): near-ground turbulence sigma = 0.2 x the mean wind
    "demo_wind_mps": (0.0, 4.0),            # v0.8 demo: OUTDOOR (GPS needs open sky), demo wind limit 4 m/s (params)
    "p_pole": 0.5,                          # demo pushes along a pole held at hand height ...
    "hand_height_m": (0.3, 1.2),            # ... (crouching .. chest)
    "pole_length_m": (2.5, 3.5),            # params demo_rules: >= 2.5 m
}
# ---- v0.9 plant (own stream; ASSUMED ranges, architect decisions b-f) ----
V09_STREAM = 0x909
CAMPAIGN_PROFILES = ("field", "demo", "tether_test", "hybrid_demo")
PROFILE_TAG = {"field": [], "demo": [0xDE], "tether_test": [0x7E7], "hybrid_demo": [0x4B1]}
SAMPLING_V09: dict[str, Any] = {
    "t_max_kgf": (2.14, 3.09),              # plant T_max at 22.2 V (design envelope; opts t_max_kgf_range narrows it)
    "fan_power_model": (0.0, 1.0),          # 0 = P1 (proxy efficiency) .. 1 = P2 (pessimistic)
    "rotor_inertia_scale": (0.7, 1.3),      # x edf.thrust_v09.rotor_inertia_kg_m2_ASSUMED (4.5e-5)
    "lost_motion_deg": (0.3, 1.0),          # per vane (params vane.direct_drive_play.sampled_lost_motion_deg)
    "gear_backlash_deg": (0.5, 2.0),        # per vane (params vane.direct_drive_play.sampled_gear_backlash_deg)
    # tether profiles
    "p_indoor": 0.5,                        # tether_test / hybrid_demo indoors (no GPS) vs outdoors (GPS build)
    "anchor_height_m": (3.5, 4.0),          # pulley above the ground (proposal)
    "anchor_offset_m": (0.0, 0.3),          # pulley set-up offset from the pad centre (ASSUMED)
    "p_slack_feed": 0.3,                    # slack feed (catenary) instead of the constant-tension feed
    "tension_n": (0.5, 2.0),                # proposal
    # fix round (physics review medium 4): the slack loop hangs (L - h) / 2 below the eye bolt; above 0.36 m of slack it
    # reaches the intake windows (181 mm below the eye) - capped at 0.25 m (loop >= 55 mm above the window top; the
    # procedure's slack-feed limit).  The same U draw as before, mapped into the new range (was (0.2, 1.0))
    "cable_slack_m": (0.1, 0.25),
    "belay_slack_m": (0.0, 0.3),            # proposal (the 'autolock' device's slack; the SRL keeps the line taut)
    "supply_limit_a": (60.0, 75.0),         # CC limit (60 A set; datasheet CC onset up to 105-135 % of 63 A)
    "r_loop_scale": (1.0, 1.5),             # route / connector / temperature spread of the loop resistance
    "buf_v_rest_cell": (3.70, 3.90),        # hybrid buffer at the flight (storage 3.80)
    "tether_hold_s": (14.0, 20.0),          # ON -> OFF (motor time stays under V2's 30 s)
    "p_down_1m": 0.4,                       # tether_test: DOWN once to 1.0 m
    "p_up": 0.3,                            # tether_test: UP 1-3 times (2.0-3.0 m)
}
# ---- v0.9 fix round (independent reviews: physics HIGH 1 / medium 3 / 5 / 6, control HIGH 2 / 3) - own stream, so
# every earlier draw of run (seed, i, profile) is kept.  ASSUMED ranges ----
FIX9_STREAM = 0x9F1
SAMPLING_FIX9: dict[str, Any] = {
    "vane_force_scale": (0.72, 1.05),       # hub / motor-wake dead inner span (x0.76 / x0.85) .. the params model + 5 %
    "vane_yaw_lever_x_at_min": 1.13,        # yaw lever x, linear in the force scale: 1.13 at 0.72 .. 1.00 at 1.05
    "p_swirl_negative": 0.5,                # the residual-swirl sign relative to the rotor is unknown (stator under- or
                                            # over-turning): the magnitude draw of FIX_STREAM keeps, the sign is drawn here
    "ge_thrust_a": (0.2, 0.8),              # ground effect T x (1 + a_T e), e = exp(-4 h / D) (ASSUMED shape, no bench data)
    "ge_vane_a": (0.4, 1.0),                # vane q x (1 - a_q e)
    "adc_gain_err": (-0.005, 0.005),        # per channel, after the two-point calibration the procedure requires
    "regen_eff": (0.0, 0.8),                # ESC active-freewheel recovery (0 = coasts; bench test pending)
    "clamp_r_ohm": (1.0, 2.0),              # brake chopper resistor (proposal 1-2 Ohm)
    "ovp_v": (27.6, 32.4),                  # RSP-1500-24 OVP (datasheet range)
    "p_srl": 0.5,                           # catch-line device: self-retracting lifeline, else auto-locking belay + handler
    "srl_retract_n": (0.3, 1.0),
    "lock_speed_mps": (1.2, 1.8),
    "handler_tau_s": (0.3, 0.8),            # autolock: the handler's take-in lag (pay-out is free)
}
CSV_KEYS = ("i", "seed", "profile", "outcome", "verdict", "failed", "no_interlock", "landed_then_safe", "no_false_touchdown",
            "touchdown_vz_ok", "touchdown_vh_ok", "touchdown_tilt_ok", "hold_rms_ok", "gust_recovery_ok",
            "pushes_recovered", "n_pushes", "interlocks", "abort_reason", "land_by", "landed_by", "touchdown_vz",
            "touchdown_vh_air", "touchdown_vh", "touchdown_tilt_deg", "touchdown_distance_m", "height_at_landed_m",
            "hold_rms_err_m", "hold_max_abs_err_m", "climb_overshoot_m", "sat_high_after_ramp_s", "u_mean_hold",
            "u_max_used", "max_tilt_powered_deg", "max_tilt_est_err_deg", "rms_tilt_est_err_deg", "gust_recovery_max_s",
            "gust_recovery_dev_max_s", "gust_peak_tilt_max_deg", "push_peak_tilt_max_deg", "push_recovery_max_s",
            "push_peak_disp_max_m", "drift_max_m", "vh_max_airborne_mps", "motor_time_s", "v_cell_min",
            "touchdown_confirm_s", "n_touchdowns", "rtt_mean_s", "timeouts", "no_liftoff", "cannot_fly", "abort_rate_cause",
            "kt_hold_mean", "kt_hold_err_vs_thrust_scale", "plant_thrust_to_weight",
            # sample
            "estimator", "wind_mps", "wind_dir_deg", "wind_swing_deg", "wind_swing_period_s", "wind_dir_rate_sigma_dps",
            "wind_reversal_t_s", "wind_speed_mod_frac", "turbulence_sigma_mps", "n_gusts", "gust_max_mps", "packet_loss",
            "radio", "gyro_noise_scale", "gyro_bias_sigma_scale", "accel_bias_g", "vibration_g", "baro_noise_m",
            "baro_thrust_offset_m", "baro_thrust_comp_m", "mass_scale", "cg_offset_mm", "thrust_scale",
            "thrust_voltage_exp", "swirl_scale", "servo_tau_scale",
            "crossflow_cd_scale", "ram_drag_scale", "vane_trim_max_abs_deg", "batt_v_rest_cell", "initial_tilt_deg",
            "t_on_s", "t_off_s", "push_force_max_n", "push_impulse_max_ns", "runtime_s",
            # v0.8 (GPS position hold; the checks are the v0.8 ones for gps_mode on / off: "" = not applicable)
            "gps_mode", "touchdown_vh_ground_ok", "touchdown_from_hold_ok", "station_keeping_max_ok",
            "touchdown_tip_margin_ok", "nav_rms_ok", "tilt_sp_within_cap_ok", "tilt_within_cap_ok", "sk_rms_m", "sk_max_m",
            "sk_max_all_m", "nav_rms_m", "nav_est_err_rms_m", "touchdown_from_hold_m", "touchdown_vh_ground",
            "tilt_sp_max_deg", "tilt_max_hold_deg", "limited_frac_hold", "psi_err_max_deg", "gps_avail_frac_powered",
            "pos_mode_ATT_s", "gps_resets", "pos_requests", "push_excursion_max_m", "push_return_1m_max_s",
            "gps_latency_s", "gps_vel_white_mps", "gps_pos_gm_sigma_m", "gps_pos_gm_tau_s", "gps_sky_blocked",
            "gps_multipath_rate_hz", "gps_outage_s", "heading_err_deg", "n_pole_pushes", "touchdown_tip_margin_mps",
            "touchdown_lean_toward_deg",
            # v0.8.1 (keep-trim touchdown criterion, four-chip GNS chain; campaign round: the FLOWN wind)
            "touchdown_tilt_trim_ok", "touchdown_trim_deg", "touchdown_tilt_sp_deg", "gps_chain", "wind_max_mps",
            # v0.9 plant
            "power_source", "t_max_kgf", "fan_power_model", "touchdown_vh_ground_ok", "max_wz_powered_deg_s",
            "v_bus_min_powered", "i_bus_max", "supply_cc_s", "thrust_limited_s", "tether_catch_max_n", "buffer_soc_min")


def flown_env(r: dict[str, Any]) -> dict[str, Any]:
    """The environment run ``r`` actually flew (``run_one`` records it from the Scenario): for the v0.8 modes the
    outdoor envelope (``v08_env``: the demo's outdoor wind, turbulence intensity 0.2, the field gusts clipped to the
    operator limit) replaces the sample's v0.7.1 values; older records fall back to the sample."""
    s = r["sample"]
    f = r.get("flown") or {}
    return {"wind_mps": f.get("wind_mps", s["wind_mps"]),
            "turbulence_sigma_mps": f.get("turbulence_sigma_mps", s["turbulence_sigma_mps"]),
            "gust_max_mps": f.get("gust_max_mps", max((g["speed_mps"] for g in s["gusts"]), default=0.0)),
            "n_gusts": f.get("n_gusts", len(s["gusts"]))}


def _u(rng: np.random.Generator, key: str, table: dict[str, Any] = SAMPLING) -> float:
    lo, hi = table[key]
    return float(rng.uniform(lo, hi))


def light_push_band_m() -> tuple[float, float]:
    """The ``--pushes light`` height band: from the intake-window top to the same distance above the CG [m].  v0.9: on the
    VTOL vehicle the window top (0.319 m) is above the CG (0.247 m) - the band is from the window top + 10 mm to the top cap
    - 15 mm (the avionics section, clear of the fan windows)."""
    veh = vs.vehicle()
    zt = float(veh.layout.z_window_top) * 1e-3
    hi = 2.0 * veh.z_cg_m - zt
    top = vs.vtol_push_top_m()
    if top is not None and hi <= zt + 0.02:
        return zt + 0.01, top
    return zt, hi


def sample(campaign_seed: int, i: int, estimator: str = "gated_mahony", profile: str = "field",
           pushes: str = "any", push_timing: str | None = None) -> dict[str, Any]:
    """The variables of run i (a pure function of (campaign_seed, i, profile, pushes, push_timing)); JSON-safe.
    ``pushes`` (demo): "any" = SAMPLING_DEMO (anywhere a pole reaches, 2-20 N x 0.1-0.6 s: many ABORTs by design),
    "light" = the live demo as planned (SAMPLING_DEMO_LIGHT, the ``light_push_band_m`` heights).  ``push_timing`` (demo
    profiles): "v09" = the first push / the GPS-outage window from ON + 10 s (the v0.9 take-off reaches HOLD later),
    "v081" = from ON + 7 s; default "v09" on the VTOL hardware configuration, else "v081" (v0.9 campaign round)."""
    if profile not in CAMPAIGN_PROFILES:
        raise ValueError(f"profile {profile!r} not in {CAMPAIGN_PROFILES}")
    if pushes not in PUSH_SETS:
        raise ValueError(f"pushes {pushes!r} not in {PUSH_SETS}")
    tprof = profile
    if profile in ("tether_test", "hybrid_demo"):
        # v0.9: the tether profiles start from a demo sample of their own stream (tag), then the v0.9 block below
        pushes = "light" if profile == "hybrid_demo" else pushes
    demo = profile != "field"
    tag = PROFILE_TAG[profile]
    rng = np.random.default_rng([int(campaign_seed), int(i)] + tag)
    s: dict[str, Any] = {"i": int(i), "campaign_seed": int(campaign_seed), "estimator": estimator, "profile": profile,
                         "seed": int(rng.integers(1, 2 ** 31 - 1))}
    if demo and pushes != "any":
        s["push_set"] = pushes
    S = SAMPLING
    D = SAMPLING_DEMO
    if demo:
        s["wind_mps"] = _u(rng, "wind_mps", D)
        s["wind_dir_deg"] = _u(rng, "wind_dir_deg")
        s["turbulence_sigma_mps"] = _u(rng, "turbulence_sigma_mps", D)
        s.update(wind_swing_deg=0.0, wind_swing_period_s=30.0, wind_swing_phase_deg=0.0, wind_dir_rate_sigma_dps=0.0,
                 wind_reversal_t_s=None, wind_reversal_s=5.0, wind_speed_mod_frac=0.0, wind_speed_mod_period_s=20.0)
        s["gusts"] = []
    else:
        for k in ("wind_mps", "wind_dir_deg", "turbulence_sigma_mps"):
            s[k] = _u(rng, k)
        swing = rng.uniform() < S["p_swing"]
        s["wind_swing_deg"] = _u(rng, "wind_swing_deg") if swing else 0.0
        s["wind_swing_period_s"] = _u(rng, "wind_swing_period_s")
        s["wind_swing_phase_deg"] = float(rng.uniform(0.0, 360.0))
        walk = rng.uniform() < S["p_walk"]
        s["wind_dir_rate_sigma_dps"] = _u(rng, "wind_dir_rate_sigma_dps") if walk else 0.0
        rev = rng.uniform() < S["p_reversal"]
        s["wind_reversal_t_s"] = _u(rng, "wind_reversal_t_s") if rev else None
        s["wind_reversal_s"] = _u(rng, "wind_reversal_s")
        mod = rng.uniform() < S["p_speed_mod"]
        s["wind_speed_mod_frac"] = _u(rng, "wind_speed_mod_frac") if mod else 0.0
        s["wind_speed_mod_period_s"] = _u(rng, "wind_speed_mod_period_s")
        lo, hi = S["n_gusts"]
        n_g = int(rng.integers(lo, hi + 1))
        s["gusts"] = [{"t0_s": _u(rng, "gust_t0_s"), "duration_s": _u(rng, "gust_duration_s"),
                       "speed_mps": _u(rng, "gust_mps"), "dir_deg": float(rng.uniform(0.0, 360.0)),
                       "rise_s": _u(rng, "gust_rise_s"), "vertical_mps": _u(rng, "gust_vertical_mps")} for _ in range(n_g)]
    s["packet_loss"] = _u(rng, "packet_loss")
    s["radio"] = RADIOS[int(rng.integers(0, len(RADIOS)))]
    for k in ("gyro_noise_scale", "gyro_bias_sigma_scale", "accel_bias_g", "vibration_g", "baro_noise_m", "mass_scale",
              "thrust_scale", "servo_tau_scale", "crossflow_cd_scale", "ram_drag_scale", "batt_v_rest_cell",
              "baro_thrust_offset_m"):
        s[k] = _u(rng, k)
    res = float(np.clip(rng.normal(0.0, S["baro_comp_residual_sigma_m"]), -0.3, 0.3))
    s["baro_thrust_comp_m"] = s["baro_thrust_offset_m"] - res
    r_cg, a_cg = _u(rng, "cg_offset_mm"), float(rng.uniform(0.0, 2.0 * math.pi))
    cad = vs.cad_cg_offset_mm()
    s["cg_residual_mm"] = [r_cg * math.cos(a_cg), r_cg * math.sin(a_cg)]
    s["cg_offset_mm"] = [cad[0] + s["cg_residual_mm"][0], cad[1] + s["cg_residual_mm"][1]]
    s["vane_trim_err_deg"] = [float(v) for v in np.clip(rng.normal(0.0, S["vane_trim_sigma_deg"], 4), -3.0, 3.0)]
    s["initial_tilt_deg"] = _u(rng, "initial_tilt_deg")
    s["initial_tilt_axis_deg"] = float(rng.uniform(0.0, 360.0))
    s["initial_yaw_deg"] = float(rng.uniform(0.0, 360.0))
    s["t_on_s"] = _u(rng, "t_on_s")
    if demo:
        s["t_off_s"] = s["t_on_s"] + _u(rng, "hold_s", D)
        s["t_up_s"] = s["t_on_s"] + float(rng.uniform(6.0, 8.0)) if rng.uniform() < D["p_up"] else None
        n_p = int(rng.integers(D["n_pushes"][0], D["n_pushes"][1] + 1))
        pt = push_timing or ("v09" if vs.hardware_config() == "vtol" else "v081")
        if pt not in ("v09", "v081"):
            raise ValueError(f"push_timing {pt!r} not in ('v09', 'v081')")
        s["push_timing"] = pt
        t = s["t_on_s"] + _u(rng, "t_first_push_after_on_s_v09" if pt == "v09" else "t_first_push_after_on_s", D)
        DP = {**D, **SAMPLING_DEMO_LIGHT, "push_z_body_m": light_push_band_m()} if pushes == "light" else D
        plist = []
        for _ in range(n_p):
            du = _u(rng, "push_duration_s", DP)
            if t + du > s["t_off_s"] - 1.0:
                break
            trap = rng.uniform() < D["p_trapezoid"]
            plist.append({"t0_s": t, "duration_s": du, "force_n": _u(rng, "push_force_n", DP),
                          "azimuth_deg": float(rng.uniform(0.0, 360.0)), "z_body_m": _u(rng, "push_z_body_m", DP),
                          "offset_deg": float(np.clip(rng.normal(0.0, D["push_offset_sigma_deg"]), -30.0, 30.0)),
                          "profile": "trapezoid" if trap else "half_sine", "rise_s": _u(rng, "push_rise_s", D),
                          "vertical_n": float(np.clip(rng.normal(0.0, D["push_vertical_sigma_n"]), -3.0, 3.0))})
            # the next push: after a gap, spread over the hold
            t = t + du + D["push_gap_s"] + float(rng.uniform(0.0, 3.0))
        s["pushes"] = plist
    else:
        for k in ("up_after_s", "up_gap_s", "t_down_s"):
            s[k] = _u(rng, k)
        s["t_off_s"] = s["t_down_s"] + _u(rng, "off_after_down_s")
        s["pushes"] = []
    # fix round: the new variables come from their own stream [seed, i, FIX_STREAM (, 0xDE)], so each run (seed, i,
    # profile) keeps every v0.7.1 random number (the widened ranges - thrust_scale, wind_swing_period_s - map the same
    # draw to a new value; the CG is shifted by the CAD offset) and the demo push sets share them
    rf = np.random.default_rng([int(campaign_seed), int(i), FIX_STREAM] + tag)
    for k in ("thrust_voltage_exp", "swirl_scale"):
        s[k] = _u(rf, k)
    # v0.8: the GPS receiver, the pad set-up, the outdoor envelope and the pole (own stream; the pole draws come last so
    # the GPS variables are the same for every push set)
    rv = np.random.default_rng([int(campaign_seed), int(i), V08_STREAM] + tag)
    G = SAMPLING_GPS
    g: dict[str, Any] = {k: _u(rv, k, G) for k in ("latency_s", "latency_jitter_s", "pos_gm_sigma_m", "pos_gm_tau_s",
                                                    "pos_white_m", "vel_white_mps", "vel_gm_sigma_mps", "step_rate_hz")}
    mp = rv.uniform() < G["p_multipath"]
    g["multipath_rate_hz"] = _u(rv, "multipath_rate_hz", G) if mp else 0.0
    g["sky_blocked"] = _u(rv, "sky_blocked", G)
    out_on = rv.uniform() < G["p_outage"]
    dur = _u(rv, "outage_s", G)
    if demo:
        # v0.9 campaign round (bug fix): the outage starts in HOLD - ON + 10 s with the v0.9 take-off (``push_timing``)
        h0, h1 = s["t_on_s"] + (10.0 if s.get("push_timing") == "v09" else 7.0), s["t_off_s"] - 1.0
    else:
        h0, h1 = 13.0, s["t_down_s"]
    t_o = float(rv.uniform(h0, max(h0, h1 - dur)))
    g["outages"] = [[t_o, t_o + dur]] if out_on else []
    g["heading_err_deg"] = float(np.clip(rv.normal(0.0, G["heading_err_sigma_deg"]), -30.0, 30.0))
    s["gps"] = g
    ti = float(G["turbulence_intensity"])
    if demo:
        wd = _u(rv, "demo_wind_mps", G)
        s["v08_env"] = {"wind_mps": wd, "turbulence_sigma_mps": ti * wd, "gust_clip_mps": None}
    else:
        lim = vs.vehicle().params["mission"]["vtol"]["position_hold"]["wind_limits_mps"]
        s["v08_env"] = {"wind_mps": s["wind_mps"], "turbulence_sigma_mps": ti * s["wind_mps"],
                        "gust_clip_mps": max(0.0, float(lim["field_gust"]) - s["wind_mps"])}
    for q in s.get("pushes", []):
        pole = rv.uniform() < G["p_pole"]
        hh, pl = _u(rv, "hand_height_m", G), _u(rv, "pole_length_m", G)
        if pole:
            q["hand_height_m"], q["pole_length_m"] = hh, pl
    sample_v09(s, campaign_seed, i, tprof, tag)
    return s


def sample_v09(s: dict[str, Any], campaign_seed: int, i: int, profile: str, tag: list) -> None:
    """The v0.9 plant variables of run i (own stream [seed, i, V09_STREAM] + the profile tag; ``SAMPLING_V09``), in place:
    T_max, fan power model, rotor inertia x, the vane play per vane, the power source of the profile and (tether
    profiles) the rig, supply and buffer, the indoor / outdoor choice, the hover script."""
    r9 = np.random.default_rng([int(campaign_seed), int(i), V09_STREAM] + tag)
    V = SAMPLING_V09
    s["thrust_scale"] = 1.0                                  # v0.9: the envelope replaces the thrust_scale spread
    s["t_max_kgf"] = _u(r9, "t_max_kgf", V)
    s["fan_power_model"] = _u(r9, "fan_power_model", V)
    s["rotor_inertia_scale"] = _u(r9, "rotor_inertia_scale", V)
    lm = [_u(r9, "lost_motion_deg", V) for _ in range(4)]
    gb = [_u(r9, "gear_backlash_deg", V) for _ in range(4)]
    s["vane_play_deg"] = [a + b for a, b in zip(lm, gb)]
    s["power_source"] = {"tether_test": "tether", "hybrid_demo": "hybrid"}.get(profile, "battery")
    s["campaign_profile"] = profile
    # tether rig / supply / buffer (drawn for every profile so the stream is the same; used by the tether profiles)
    tr = {"anchor_height_m": _u(r9, "anchor_height_m", V), "anchor_offset_m": _u(r9, "anchor_offset_m", V),
          "anchor_dir_deg": float(r9.uniform(0.0, 360.0)), "slack_feed": bool(r9.uniform() < V["p_slack_feed"]),
          "tension_n": _u(r9, "tension_n", V), "cable_slack_m": _u(r9, "cable_slack_m", V),
          "belay_slack_m": _u(r9, "belay_slack_m", V)}
    pw = {"supply_limit_a": _u(r9, "supply_limit_a", V), "r_loop_scale": _u(r9, "r_loop_scale", V),
          "buf_v_rest_cell": _u(r9, "buf_v_rest_cell", V)}
    indoor = bool(r9.uniform() < V["p_indoor"])
    hold = _u(r9, "tether_hold_s", V)
    down = bool(r9.uniform() < V["p_down_1m"])
    n_up = int(r9.integers(1, 4)) if r9.uniform() < V["p_up"] else 0
    # fix round: vane effectiveness, swirl sign, ground effect, ADC gain, regeneration, catch-line device (own stream)
    rx = np.random.default_rng([int(campaign_seed), int(i), FIX9_STREAM] + tag)
    F = SAMPLING_FIX9
    vf = _u(rx, "vane_force_scale", F)
    lo_f, hi_f = F["vane_force_scale"]
    neg = bool(rx.uniform() < F["p_swirl_negative"])
    f9 = {"vane_force_scale": vf,
          "vane_yaw_lever_scale": 1.0 + (F["vane_yaw_lever_x_at_min"] - 1.0) * (hi_f - vf) / (hi_f - lo_f),
          "swirl_sign": -1.0 if neg else 1.0,
          "ge_thrust_a": _u(rx, "ge_thrust_a", F), "ge_vane_a": _u(rx, "ge_vane_a", F),
          "adc_gain_err": [_u(rx, "adc_gain_err", F) for _ in range(3)],
          "regen_eff": _u(rx, "regen_eff", F), "clamp_r_ohm": _u(rx, "clamp_r_ohm", F), "ovp_v": _u(rx, "ovp_v", F),
          "belay_device": "srl" if rx.uniform() < F["p_srl"] else "autolock",
          "srl_retract_n": _u(rx, "srl_retract_n", F), "lock_speed_mps": _u(rx, "lock_speed_mps", F),
          "handler_tau_s": _u(rx, "handler_tau_s", F)}
    s["fix9"] = f9
    s["swirl_scale"] = abs(float(s.get("swirl_scale", 1.0))) * f9["swirl_sign"]
    if profile in ("tether_test", "hybrid_demo"):
        # the cable's lateral pull / the catch line need the pad under the pulley; the (sampled) demo pushes for the
        # hybrid demo only; the hover script: OFF after hold (motor time < 30 s), tether_test DOWN to 1.0 m or UP x1-3
        s["tether"], s["power"], s["indoor"] = tr, pw, indoor
        s["t_off_s"] = s["t_on_s"] + hold
        s["t_up_s"] = None
        if profile == "tether_test":
            s["pushes"] = []
            s["tether_alt_steps"] = -1 if down else n_up
        else:
            s["pushes"] = [q for q in s.get("pushes", []) if q["t0_s"] + q["duration_s"] < s["t_off_s"] - 1.0]
            s["tether_alt_steps"] = 0
        if indoor:
            s["v08_env"] = {"wind_mps": s["wind_mps"], "turbulence_sigma_mps": s["turbulence_sigma_mps"],
                            "gust_clip_mps": None}       # indoors: the demo drafts (0-0.5 m/s), not the outdoor envelope


def opts_tag(opts: dict | None) -> str:
    """The output-folder suffix of the campaign-wide overrides (``apply_opts``)."""
    if not opts:
        return ""
    t = "".join(f"_{k}{v:g}" for k, v in sorted((opts.get("gps") or {}).items()))
    return t + ("_windlimit" if opts.get("wind_limit") else "")


def gps_sim_of(s: dict[str, Any]) -> GpsSim:
    """The receiver model of sample ``s`` (``SAMPLING_GPS``; the other GpsSim fields keep their stated defaults)."""
    g = s.get("gps") or {}
    kw = {k: g[k] for k in ("latency_s", "latency_jitter_s", "pos_gm_sigma_m", "pos_gm_tau_s", "pos_white_m",
                            "vel_white_mps", "vel_gm_sigma_mps", "step_rate_hz", "multipath_rate_hz", "sky_blocked",
                            "heading_err_deg") if k in g}
    kw["outages"] = tuple(tuple(w) for w in g.get("outages", []))
    return GpsSim(**kw)


def build_scenario(p: dict, s: dict[str, Any], vgains: dict | None, gps_mode: str = "v071"):
    """The Scenario for sample ``s`` (field nominal mission or demo hover with pushes).  ``gps_mode`` (v0.8): "on" = the
    GPS build with the sample's receiver and set-up in the v0.8 outdoor envelope (``v08_env``: turbulence intensity 0.2,
    field gusts clipped to the operator limit, the demo outdoors up to 4 m/s), "off" = the same sample flown by the v0.8
    build without the GPS (the paired before), "v071" = the v0.7.1 build in the v0.7.1 environment (the default, so a
    v0.7.1 sample builds the v0.7.1 run)."""
    if gps_mode not in ("on", "off", "v071"):
        raise ValueError(f"gps_mode {gps_mode!r} not in ('on', 'off', 'v071')")
    v8 = gps_mode in ("on", "off") and "v08_env" in s
    if v8:
        env = s["v08_env"]
        s = dict(s, wind_mps=env["wind_mps"], turbulence_sigma_mps=env["turbulence_sigma_mps"])
        clip = env.get("gust_clip_mps")
        if clip is not None:
            s["gusts"] = [dict(g, speed_mps=min(g["speed_mps"], clip)) for g in s["gusts"]
                          if min(g["speed_mps"], clip) > 0.0]
        s["initial_yaw_deg"] = 0.0                  # set up toward the pad azimuth (+ the sampled heading error)
    sens0 = SensorConfig()
    sens = replace(sens0, attitude_estimator=s["estimator"],
                   gyro_noise_density_rad_s_sqrthz=sens0.gyro_noise_density_rad_s_sqrthz * s["gyro_noise_scale"],
                   gyro_bias_sigma_rad_s=sens0.gyro_bias_sigma_rad_s * s["gyro_bias_sigma_scale"],
                   accel_bias_mps2=s["accel_bias_g"] * G0, baro_noise_m=s["baro_noise_m"],
                   accel_vibration_rms_mps2=s.get("vibration_g", 0.0) * G0)
    gusts = tuple(Gust(g["t0_s"], g["duration_s"], g["speed_mps"], g["dir_deg"], g["rise_s"], g["vertical_mps"])
                  for g in sorted(s["gusts"], key=lambda g: g["t0_s"]))
    demo = s.get("profile", "field") != "field"
    cprof = s.get("campaign_profile", s.get("profile", "field"))
    tprof = cprof in ("tether_test", "hybrid_demo")
    if tprof and s.get("indoor"):
        gps_mode = "off"                            # v0.9: an indoor tether flight has no GPS (the v0.8 build without it)
    if gps_mode == "on":
        gkw: dict[str, Any] = {"gps": gps_sim_of(s)}
    elif gps_mode == "off":
        gkw = {"gps": None, "nav_v08": True, "takeoff_check": True}
    else:
        gkw = {"gps": None}
    if tprof:
        script = vs.tether_script(t_on=s["t_on_s"], alt={-1: "1.0", 0: "1.5", 1: "2.0", 2: "2.5", 3: "3.0"}[
            int(s.get("tether_alt_steps", 0))], t_off=s["t_off_s"])
        t_end = s["t_off_s"] + 20.0
    elif demo:
        script = [vs.OpStep("on", t=s["t_on_s"], note="take off")]
        if s.get("t_up_s") is not None:
            script.append(vs.OpStep("up", t=s["t_up_s"], phase="HOLD", note="UP"))
        script.append(vs.OpStep("off", t=s["t_off_s"], phase="HOLD", note="land"))
        script = tuple(script)
        t_end = s["t_off_s"] + 20.0
    else:
        script = vs.nominal_script(t_on=s["t_on_s"], up_after=s["up_after_s"], up_gap=s["up_gap_s"],
                                   t_down=s["t_down_s"], t_off=s["t_off_s"])
        t_end = vs.T_END
    pushes = tuple(Push(q["t0_s"], q["duration_s"], q["force_n"], q["azimuth_deg"], q["z_body_m"], q["offset_deg"],
                        q["profile"], q["rise_s"], q["vertical_n"],
                        hand_height_m=q.get("hand_height_m") if v8 else None,
                        pole_length_m=q.get("pole_length_m", 2.5) if v8 else 2.5) for q in s.get("pushes", []))
    kw9 = v09_kw(p, s)
    kw9.setdefault("cg_offset_mm", tuple(s["cg_offset_mm"]))
    return vs.base_vtol(f"C{s['campaign_seed']}_{s.get('profile', 'field')[0].upper()}{s['i']:04d}"
                        + ("" if gps_mode == "v071" else f"_{gps_mode}"),
                        f"VTOL campaign run ({s.get('profile', 'field')} profile, sampled inputs)",
                        vgains, seed=s["seed"], vtol_script=script, gusts=gusts, sensors=sens, t_end=t_end,
                        profile="demo" if demo else None, pushes=pushes,
                        wind_mps=s["wind_mps"], wind_dir_deg=s["wind_dir_deg"],
                        wind_swing_deg=s.get("wind_swing_deg", 0.0), wind_swing_period_s=s.get("wind_swing_period_s", 30.0),
                        wind_swing_phase_deg=s.get("wind_swing_phase_deg", 0.0),
                        wind_dir_rate_sigma_dps=s.get("wind_dir_rate_sigma_dps", 0.0),
                        wind_reversal_t_s=s.get("wind_reversal_t_s"), wind_reversal_s=s.get("wind_reversal_s", 5.0),
                        wind_speed_mod_frac=s.get("wind_speed_mod_frac", 0.0),
                        wind_speed_mod_period_s=s.get("wind_speed_mod_period_s", 20.0),
                        turbulence_sigma_mps=s["turbulence_sigma_mps"], packet_loss=s["packet_loss"], radio=s["radio"],
                        mass_scale=s["mass_scale"], thrust_scale=s["thrust_scale"],
                        thrust_voltage_exp=float(s.get("thrust_voltage_exp", 0.0)),
                        swirl_scale=float(s.get("swirl_scale", 1.0)),
                        servo_tau_scale=s["servo_tau_scale"], vane_trim_err_deg=tuple(s["vane_trim_err_deg"]),
                        crossflow_cd_scale=s.get("crossflow_cd_scale", 1.0), ram_drag_scale=s.get("ram_drag_scale", 1.0),
                        baro_thrust_offset_m=s.get("baro_thrust_offset_m", 0.0),
                        baro_thrust_comp_m=s.get("baro_thrust_comp_m"),
                        initial_tilt_deg=s["initial_tilt_deg"], initial_tilt_axis_deg=s["initial_tilt_axis_deg"],
                        initial_yaw_deg=s["initial_yaw_deg"],
                        options=replace(SimOptions(), batt_v_full_cell=s["batt_v_rest_cell"]), **gkw, **kw9)


def v09_kw(p: dict, s: dict[str, Any]) -> dict[str, Any]:
    """The v0.9 plant fields of sample ``s`` (``sample_v09``) for ``vs.base_vtol``; empty for a pre-v0.9 sample."""
    if "t_max_kgf" not in s:
        return {}
    i_rot = float((p["edf"].get("thrust_v09") or {}).get("rotor_inertia_kg_m2_ASSUMED", 0.0))
    kw: dict[str, Any] = {"t_max_kgf": float(s["t_max_kgf"]), "fan_power_model": float(s["fan_power_model"]),
                          "rotor_inertia_kg_m2": i_rot * float(s["rotor_inertia_scale"]),
                          "vane_play_deg": tuple(float(v) for v in s["vane_play_deg"]),
                          "power_source": s.get("power_source", "battery")}
    src = kw["power_source"]
    f9 = s.get("fix9")
    if f9:
        kw.update(vane_force_scale=float(f9["vane_force_scale"]), vane_yaw_lever_scale=float(f9["vane_yaw_lever_scale"]),
                  ge_thrust_a=float(f9["ge_thrust_a"]), ge_vane_a=float(f9["ge_vane_a"]))
    if src in ("tether", "hybrid"):
        tr, pw = s["tether"], s["power"]
        a = math.radians(tr["anchor_dir_deg"])
        dev = {}
        if f9:
            dev = {"belay_device": f9["belay_device"], "srl_retract_n": float(f9["srl_retract_n"]),
                   "lock_speed_mps": float(f9["lock_speed_mps"]), "handler_tau_s": float(f9["handler_tau_s"])}
        kw["tether"] = TetherConfig.from_params(p, anchor_xy_m=(tr["anchor_offset_m"] * math.cos(a),
                                                                tr["anchor_offset_m"] * math.sin(a)),
                                                anchor_height_m=tr["anchor_height_m"],
                                                feed="slack" if tr["slack_feed"] else "tension", tension_n=tr["tension_n"],
                                                cable_slack_m=tr["cable_slack_m"], belay_slack_m=tr["belay_slack_m"], **dev)
        base = PowerConfig.from_params(p, source=src)
        ekw = {}
        if f9:
            ekw = {"adc_gain_err": tuple(float(v) for v in f9["adc_gain_err"]), "regen_eff": float(f9["regen_eff"]),
                   "clamp_r_ohm": float(f9["clamp_r_ohm"]), "ovp_v": float(f9["ovp_v"])}
        kw["power"] = PowerConfig.from_params(p, source=src, i_limit_a=pw["supply_limit_a"],
                                              r_loop_ohm=base.r_loop_ohm * pw["r_loop_scale"],
                                              buf_v_rest_cell=pw["buf_v_rest_cell"], **ekw)
        # the CAD lateral CG of this power source + the sampled residual (sample() drew it around the battery CG)
        cad = vs.cad_cg_offset_mm(src)
        kw["cg_offset_mm"] = (cad[0] + s["cg_residual_mm"][0], cad[1] + s["cg_residual_mm"][1])
    return kw


def outcome_of(m: dict[str, Any], verdict: str) -> int:
    """Index into OUTCOME_NAMES: pass / landed_fail / abort / not_landed / other (incl. a plant that never lifted off)."""
    if m.get("t_abort_s") is not None or m.get("t_released_s") is not None:
        return 2
    if m.get("no_liftoff") or m.get("cannot_fly"):
        return 4
    if m.get("t_landed_s") is not None:
        return 0 if verdict == "PASS" else 1
    if m.get("t_motor_start_s") is not None:
        return 3
    return 4


def track_of(res: Any) -> dict[str, Any]:
    """The run's compact track for tracks.npz: 10 Hz samples of t, the exit-plane position, phase, state and the mean
    wind (the full log stays in the worker)."""
    L = res.log
    t = L["t"]
    step = max(1, int(round(1.0 / (TRACK_HZ * float(t[1] - t[0]))))) if len(t) > 1 else 1
    idx = np.arange(0, len(t), step)
    if idx[-1] != len(t) - 1:
        idx = np.append(idx, len(t) - 1)
    pe = exit_plane_positions(res)
    return {"t": t[idx].astype(float), "pos": pe[idx].astype(np.float32), "phase": L["phase"][idx].astype(np.uint8),
            "state": L["state"][idx].astype(np.uint8), "wind_xy": L["wind_mean"][idx, :2].astype(np.float32)}


def apply_opts(s: dict[str, Any], opts: dict[str, Any] | None) -> dict[str, Any]:
    """Physics-review fix round: campaign-wide overrides of a sample (recorded in the sample as ``opts``):
    ``gps`` = {GpsSim field: value} replacing the sampled receiver values (e.g. latency_s 0.45: the mis-set latency case,
    physics review medium 7); ``wind_limit`` (field) = the operator wind limit for every run (params position_hold.
    wind_limits_mps: field_mean wind, turbulence intensity 0.2, the sampled gusts clipped to field_gust - the plant /
    sensor / receiver spread kept: physics review medium 9)."""
    if not opts:
        return s
    s = dict(s, opts=dict(opts))
    if opts.get("gps"):
        s["gps"] = dict(s.get("gps") or {}, **opts["gps"])
    if opts.get("wind_limit") and s.get("profile", "field") == "field":
        lim = vs.vehicle().params["mission"]["vtol"]["position_hold"]["wind_limits_mps"]
        wm, wg = float(lim["field_mean"]), float(lim["field_gust"])
        s["v08_env"] = {"wind_mps": wm, "turbulence_sigma_mps": float(SAMPLING_GPS["turbulence_intensity"]) * wm,
                        "gust_clip_mps": max(0.0, wg - wm)}
    rng_t = opts.get("t_max_kgf_range")
    if rng_t and "t_max_kgf" in s:
        # v0.9: the same U(2.14, 3.09) draw mapped linearly into the group's range (thrust_low / thrust_stress)
        lo0, hi0 = SAMPLING_V09["t_max_kgf"]
        q = (float(s["t_max_kgf"]) - lo0) / (hi0 - lo0)
        s["t_max_kgf"] = float(rng_t[0]) + q * (float(rng_t[1]) - float(rng_t[0]))
    return s


def run_one(args: tuple) -> dict[str, Any]:
    """Worker: sample, build, fly, judge run i.  ``args`` = (campaign_seed, i, estimator, vgains[, profile[, pushes[,
    gps_mode[, opts]]]]); gps_mode "v071" (default: the v0.7.1 build, NOMINAL_CRITERIA / PUSH_CRITERIA), "on" / "off" (the
    v0.8 GPS build / the v0.8 build without it, both judged on the v0.8 criteria: ``vs.v08_checks`` +
    ``vs.push_v08_checks`` - pushes stronger than the 3 N s demo rule are reported, not judged); opts: ``apply_opts``."""
    campaign_seed, i, estimator, vgains = args[:4]
    profile = args[4] if len(args) > 4 else "field"
    pushes = args[5] if len(args) > 5 else "any"
    gps_mode = args[6] if len(args) > 6 else "v071"
    opts = args[7] if len(args) > 7 else None
    p = vs.vehicle().params
    pt = (opts or {}).get("push_timing")            # v0.9 campaign round: opts {"push_timing": "v081"} = the old window
    s0 = (sample(campaign_seed, i, estimator, profile, pushes) if pt is None
          else sample(campaign_seed, i, estimator, profile, pushes, push_timing=pt))
    s = apply_opts(s0, opts)
    sc = build_scenario(p, s, vgains, gps_mode)
    t0 = time.perf_counter()
    res, m = vs.run_scenario(sc)
    if gps_mode == "v071":
        checks = vs.nominal_checks(m)
        pchecks = vs.push_checks(m)
    else:
        crit = vs.v08_criteria(p, m.get("profile"))
        checks = vs.v08_checks(m, crit)
        pchecks = vs.push_v08_checks(m, crit)
    failed = [k for k, v in checks.items() if v is False] + [k for k, v in pchecks.items() if v is False]
    verdict = "PASS" if not failed else "FAIL"
    keep = ("interlocks", "interlock_codes", "abort_reason", "land_by", "landed_by", "t_on_s", "t_motor_start_s", "t_hold_s",
            "t_land_s", "t_landed_s", "t_safe_after_landed_s", "t_abort_s", "t_released_s", "touchdown_vz", "touchdown_vh", "touchdown_vh_air",
            "touchdown_vh_air_local", "touchdown_tilt_deg", "touchdown_distance_m", "touchdown_t_s", "false_touchdown",
            "height_at_landed_m", "hold_rms_err_m", "hold_max_abs_err_m", "hold_rms_err_settled_m", "climb_overshoot_m",
            "sat_high_s", "sat_high_after_ramp_s", "sat_low_s", "u_mean_hold", "u_max_used", "max_tilt_powered_deg",
            "max_tilt_est_err_deg", "rms_tilt_est_err_deg", "est_gate_fraction_powered", "gust_recovery_max_s",
            "gust_recovery_dev_max_s", "gust_peak_tilt_max_deg", "gust_peak_tilt_dev_max_deg", "n_gusts_in_flight",
            "push_peak_tilt_max_deg", "push_peak_dev_max_deg", "push_recovery_max_s", "push_peak_disp_max_m",
            "push_alt_drop_max_m", "push_min_height_m", "n_pushes_in_flight", "drift_max_m", "vh_max_airborne_mps",
            "motor_time_s", "v_cell_min", "touchdown_confirm_s", "n_touchdowns", "n_liftoffs", "final_state", "rejections",
            "timeouts", "rtt", "landed_normally", "nan_abort", "wind_max_mps",
            # fix round
            "no_liftoff", "cannot_fly", "lifted_off", "abort_rate_cause", "kt_hold_mean", "kt_final", "kt_learn_s", "kt_min", "kt_max",
            "kt_hold_err_vs_thrust_scale", "plant_thrust_to_weight", "max_rate_powered_deg_s", "plant_thrust_ratio_hold",
            "plant_thrust_to_weight_hold", "support_margin_ground_g", "f_excess_ground_max_g", "yaw_integ_pinned_frac_hold",
            "yaw_err_hold_mean_deg", "yaw_rate_rms_hold_deg_s",
            # v0.8
            "gps_build", "sk_rms_m", "sk_max_m", "sk_p95_m", "sk_max_all_m", "sk_window_s", "nav_rms_m", "nav_max_m",
            "nav_est_err_rms_m", "touchdown_from_hold_m", "touchdown_vh_ground", "tilt_cap_deg", "tilt_sp_max_deg",
            "tilt_sp_mean_hold_deg", "tilt_max_hold_deg", "limited_frac_hold", "guard_s", "gps_avail_frac_powered",
            "pos_mode_VEL_s", "pos_mode_POS_s", "pos_mode_DR_s", "pos_mode_ATT_s", "psi_err_max_deg", "psi_err_end_deg",
            "psi_err_start_deg", "yaw_windows_used", "gps_resets", "gps_pos_resets", "gps_vel_resets", "pos_requests",
            "gps_rejections", "t_hold_capture_s", "profile",
            # physics-review fix: the energy-method tip-over margin at the touchdown
            "touchdown_tip_margin_mps", "touchdown_tip_vcrit_mps", "touchdown_lean_toward_deg", "touchdown_tip_margin_geom_mps",
            # v0.8.1 (campaign round): the keep-trim touchdown criterion's inputs, the chain flown, more GPS bookkeeping
            "touchdown_trim_deg", "touchdown_trim_cmd_deg", "touchdown_tilt_sp_deg", "touchdown_pos_mode", "gps_chain",
            "glink_latency_ticks", "n_hold_captures", "nav_est_err_max_m", "psi_err_max_powered_deg", "yaw_corr_end_deg",
            "vc_sat_high_hold_s", "rx_fix_frac", "gps_cfg_ok", "pos_mode_VEL_s", "drift_final_m",
            # v0.9 plant (VTOL hardware round): configuration, bus, supply, tether, buffer, power loss, yaw
            "hardware_config", "power_source", "t_max_kgf_plant", "fan_power_model", "vane_play_deg", "rotor_inertia_kg_m2",
            "max_wz_powered_deg_s", "v_bus_min_powered", "i_bus_max", "supply_cc_s", "thrust_limited_s", "supply_a_max",
            "buffer_soc_min", "buffer_cell_v_min", "buffer_a_max", "batt_soc_final", "power_events", "t_power_loss_s",
            "supply_tripped", "t_supply_trip_s", "land_after_power_loss_s", "contacts_after_loss",
            "contact_speed_max_after_loss_mps", "tether_catch_max_n", "tether_cable_max_n", "tether_tension_max_n",
            "tether_catch_taut_s", "tether_catch_taut_powered_s", "touchdown_tip_footprint_r_m",
            # v0.9 fix round (independent reviews): catch-line device, hanging, the cable loop, regeneration, take-off
            "tether_catch_loaded_powered_s", "catch_at_landed_n", "landed_hanging", "tether_locks",
            "tether_locked_powered_s", "tether_loop_depth_max_m", "cable_at_intake_s", "regen_j", "clamp_j",
            "p_regen_max_w", "regen_s", "v_bus_max", "supply_ovp_trip", "h_at_power_loss_m", "powered_at_power_loss",
            "climb_overshoot_transient_m")
    pushes = [{k: r.get(k) for k in ("i", "t0_s", "force_n", "impulse_ns", "duration_s", "z_body_m", "azimuth_deg",
                                      "offset_deg", "in_flight", "peak_tilt_deg", "peak_dev_deg", "recovery_s",
                                      "peak_horiz_disp_m", "alt_drop_m", "min_height_m", "interlocks", "abort",
                                      "abort_cause", "max_rate_xy_deg_s", "max_rate_z_deg_s",
                                      "pole", "pole_elev_deg", "impulse_h_ns", "excursion_m", "return_1p0m_s",
                                      "return_0p5m_s", "nav_excursion_m")}
              for r in m.get("pushes", [])]
    # campaign round (bug fix): the environment actually flown - for "on" / "off" the v0.8 envelope replaced the sample's
    # wind, turbulence and (field) gust speeds; the CSV, tracks and summary bins read it (``flown_env``)
    fl = {"wind_mps": float(sc.wind_mps), "turbulence_sigma_mps": float(sc.turbulence_sigma_mps),
          "gust_max_mps": max((float(g.speed_mps) for g in sc.gusts), default=0.0), "n_gusts": len(sc.gusts)}
    # v0.9 campaign round (bug fix): an indoor tether / hybrid run flies the build without the GPS (``build_scenario``
    # switches "on" to "off") - record the build flown, not the one requested
    gps_flown = ("off" if gps_mode == "on" and profile in ("tether_test", "hybrid_demo") and s.get("indoor")
                 else gps_mode)
    out = vs._clean({"i": i, "profile": profile, "gps_mode": gps_flown, "gps_mode_requested": gps_mode,
                     "sample": s, "flown": fl,
                     "metrics": {k: m.get(k) for k in keep}, "checks": checks,
                     "push_checks": pchecks, "pushes": pushes, "failed": failed, "verdict": verdict,
                     "outcome": outcome_of(m, verdict), "runtime_s": round(time.perf_counter() - t0, 2)})
    tr = track_of(res)
    if "hold_true" in res.log:
        caps = ((res.vtol or {}).get("gps") or {}).get("hold_captures") or []
        tr["hold_xy"] = np.array(caps[0]["true_xy"] if caps else [np.nan, np.nan], dtype=float)
    out["_track"] = tr
    return out


def flat_row(r: dict[str, Any]) -> dict[str, Any]:
    """One CSV row."""
    s, m, c = r["sample"], r["metrics"], r["checks"]
    row: dict[str, Any] = {"i": r["i"], "seed": s["seed"], "profile": r.get("profile", "field"),
                           "outcome": OUTCOME_NAMES[r.get("outcome", 4)], "verdict": r["verdict"],
                           "failed": ";".join(r["failed"])}
    def ok(x: Any) -> Any:
        return "" if x is None else int(bool(x))
    for k in ("no_interlock", "landed_then_safe", "no_false_touchdown"):
        row[k] = int(bool(c.get(k)))
    for k in ("touchdown_vz", "touchdown_vh", "hold_rms", "gust_recovery"):
        row[f"{k}_ok"] = int(bool(c.get(k))) if k in c else ""
    # campaign round (bug fix): the v0.8.1 touchdown_tilt check is None outside calm air (not applicable) - written as
    # "" instead of 0 (a fail); touchdown_tilt_trim is the v0.8.1 secondary touchdown criterion
    for k in ("touchdown_tilt", "touchdown_vh_ground", "touchdown_tip_margin", "touchdown_tilt_trim", "touchdown_from_hold",
              "station_keeping_max", "nav_rms", "tilt_sp_within_cap", "tilt_within_cap"):
        row[f"{k}_ok"] = ok(c.get(k)) if k in c else ""
    row["gps_mode"] = r.get("gps_mode", "v071")
    g = s.get("gps") or {}
    for k in ("latency_s", "vel_white_mps", "pos_gm_sigma_m", "pos_gm_tau_s", "sky_blocked", "multipath_rate_hz"):
        row[f"gps_{k}"] = g.get(k)
    row["gps_outage_s"] = sum(w[1] - w[0] for w in g.get("outages", [])) if g else None
    row["heading_err_deg"] = g.get("heading_err_deg")
    row["n_pole_pushes"] = sum(1 for q in r.get("pushes", []) if q.get("pole"))
    row["push_excursion_max_m"] = max((q.get("excursion_m") or 0.0 for q in r.get("pushes", []) if q.get("in_flight")),
                                      default=None)
    rets = [q.get("return_1p0m_s") for q in r.get("pushes", []) if q.get("in_flight")]
    row["push_return_1m_max_s"] = (None if any(x is None for x in rets) else max(rets)) if rets else None
    pc = r.get("push_checks") or {}
    row["pushes_recovered"] = sum(v is True for v in pc.values())
    row["n_pushes"] = len(pc)
    row["interlocks"] = ";".join(m.get("interlocks") or [])
    for k in CSV_KEYS:
        if k in row:
            continue
        if k in m:
            v = m[k]
            row[k] = ";".join(v) if isinstance(v, list) else v
        elif k in s:
            v = s[k]
            row[k] = "|".join(f"{x:.3f}" for x in v) if isinstance(v, list) else v
    rtt = m.get("rtt") or {}
    row["rtt_mean_s"] = rtt.get("mean")
    fe = flown_env(r)                   # campaign round (bug fix): the flown wind / turbulence / gusts, not the sample's
    row["wind_mps"], row["turbulence_sigma_mps"] = fe["wind_mps"], fe["turbulence_sigma_mps"]
    row["n_gusts"] = fe["n_gusts"]
    row["gust_max_mps"] = fe["gust_max_mps"]
    row["cg_offset_mm"] = math.hypot(*s["cg_offset_mm"])
    row["vane_trim_max_abs_deg"] = max(abs(v) for v in s["vane_trim_err_deg"])
    row["push_force_max_n"] = max((q["force_n"] for q in s.get("pushes", [])), default=0.0)
    row["push_impulse_max_ns"] = max((q.get("impulse_ns") or 0.0 for q in r.get("pushes", [])), default=0.0)
    row["runtime_s"] = r["runtime_s"]
    return {k: row.get(k) for k in CSV_KEYS}


def _fmt(x: Any, nd: int = 2) -> str:
    return "-" if x is None else f"{x:.{nd}f}"


def _pct(vals: list[float]) -> dict[str, float | int | None]:
    a = np.asarray([v for v in vals if v is not None and isinstance(v, (int, float)) and math.isfinite(v)], dtype=float)
    if a.size == 0:
        return {"n": 0}
    return {"n": int(a.size), "min": float(a.min()), "p5": float(np.percentile(a, 5)), "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max()), "mean": float(a.mean())}


def _bins(rows: list[dict[str, Any]], x_of, edges: list[float], y_of) -> list[dict[str, Any]]:
    """Per bin [lo, hi) of x: n and the percentiles of y (None values skipped)."""
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        rr = [r for r in rows if x_of(r) is not None and lo <= x_of(r) < hi]
        out.append({"lo": lo, "hi": hi, "n": len(rr), **{k: v for k, v in _pct([y_of(r) for r in rr]).items() if k != "n"}})
    return out


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    n = len(rows)
    checks = list(rows[0]["checks"]) if rows else []
    out: dict[str, Any] = {"n": n, "pass": sum(r["verdict"] == "PASS" for r in rows)}
    out["pass_rate"] = out["pass"] / n if n else None
    def rate(vals: list) -> float | None:
        vv = [v for v in vals if v is not None]           # v0.8 checks: None = not applicable (not counted)
        return (sum(bool(v) for v in vv) / len(vv)) if vv else None
    out["check_pass_rate"] = {c: rate([r["checks"].get(c) for r in rows]) for c in checks} if n else {}
    out["check_n_applicable"] = {c: sum(r["checks"].get(c) is not None for r in rows) for c in checks} if n else {}
    pushes = [q for r in rows for q in r.get("pushes", [])]
    pchk = [v for r in rows for v in (r.get("push_checks") or {}).values() if v is not None]
    out["push_recovered_rate"] = (sum(bool(v) for v in pchk) / len(pchk)) if pchk else None
    out["n_pushes_judged"] = len(pchk)
    out["n_pushes"] = len(pushes)
    vert = vs.VERTICAL_CHECKS
    out["vertical_pass_rate"] = (sum(all(r["checks"][c] for c in vert) for r in rows) / n) if n else None
    out["vertical_checks"] = list(vert)
    out["outcomes"] = {OUTCOME_NAMES[k]: [r["i"] for r in rows if r.get("outcome") == k] for k in range(len(OUTCOME_NAMES))}
    out["outcome_counts"] = {k: len(v) for k, v in out["outcomes"].items()}
    il: dict[str, list[int]] = {}
    for r in rows:
        for x in r["metrics"].get("interlocks") or []:
            il.setdefault(x, []).append(r["i"])
    out["interlock_counts"] = {k: len(v) for k, v in il.items()}
    out["interlock_runs"] = il
    lb: dict[str, list[int]] = {}
    for r in rows:
        if r["metrics"].get("landed_by"):
            lb.setdefault(str(r["metrics"]["landed_by"]), []).append(r["i"])
    out["landed_by"] = {k: len(v) for k, v in lb.items()}
    out["landed_by_runs"] = lb
    out["abort_runs"] = [r["i"] for r in rows if r["metrics"].get("t_abort_s") is not None]
    # fix round: plants that cannot lift off (T/W < 1), the cause of the I3 ABORTs, the thrust learning
    out["no_liftoff_runs"] = [r["i"] for r in rows if r["metrics"].get("no_liftoff")]
    out["cannot_fly_runs"] = [r["i"] for r in rows if r["metrics"].get("cannot_fly")]
    ac: dict[str, list[int]] = {}
    for r in rows:
        if r["metrics"].get("abort_rate_cause"):
            ac.setdefault(str(r["metrics"]["abort_rate_cause"]), []).append(r["i"])
    out["abort_rate_cause_runs"] = ac
    out["abort_rate_cause_counts"] = {k: len(v) for k, v in ac.items()}
    pac: dict[str, int] = {}
    for r in rows:
        for q in r.get("pushes", []):
            if q.get("abort") and q.get("abort_cause"):
                pac[str(q["abort_cause"])] = pac.get(str(q["abort_cause"]), 0) + 1
    out["push_abort_cause_counts"] = pac
    flown = [r for r in rows if not r["metrics"].get("cannot_fly")]
    out["n_flown"] = len(flown)
    out["pass_flown"] = sum(r["verdict"] == "PASS" for r in flown)
    out["vertical_pass_rate_flown"] = (sum(all(r["checks"][c] for c in vs.VERTICAL_CHECKS) for r in flown) / len(flown)
                                       if flown else None)
    out["false_touchdown_runs"] = [r["i"] for r in rows if r["metrics"].get("false_touchdown")]
    out["not_landed_runs"] = [r["i"] for r in rows if not r["checks"].get("landed_then_safe")]
    unconf = [r for r in rows if (r["metrics"].get("n_touchdowns") or 0) > 0 and r["metrics"].get("t_landed_s") is None
              and r["metrics"].get("touchdown_t_s") is not None
              and (r["metrics"].get("t_abort_s") is None or r["metrics"]["t_abort_s"] > r["metrics"]["touchdown_t_s"])]
    out["touchdown_not_confirmed_runs"] = [r["i"] for r in unconf]
    modes: dict[str, list[int]] = {}
    for r in rows:
        modes.setdefault(",".join(r["failed"]) or "PASS", []).append(r["i"])
    out["failure_modes"] = {k: len(v) for k, v in sorted(modes.items(), key=lambda kv: -len(kv[1]))}
    out["failure_mode_runs"] = {k: v for k, v in sorted(modes.items(), key=lambda kv: -len(kv[1]))}
    fc: dict[str, list[int]] = {}
    for r in rows:
        for f in r["failed"]:
            fc.setdefault(f, []).append(r["i"])
    out["failed_criterion_runs"] = fc
    metrics = ("touchdown_vz", "touchdown_vh_air", "touchdown_vh", "touchdown_tilt_deg", "touchdown_distance_m",
               "hold_rms_err_m", "hold_max_abs_err_m", "climb_overshoot_m", "sat_high_after_ramp_s", "u_mean_hold",
               "max_tilt_powered_deg", "max_tilt_est_err_deg", "rms_tilt_est_err_deg", "gust_recovery_max_s",
               "gust_recovery_dev_max_s", "gust_peak_tilt_dev_max_deg", "push_peak_tilt_max_deg", "push_recovery_max_s",
               "push_peak_disp_max_m", "push_alt_drop_max_m", "drift_max_m", "vh_max_airborne_mps", "motor_time_s",
               "v_cell_min", "touchdown_confirm_s", "kt_hold_mean", "kt_hold_err_vs_thrust_scale", "plant_thrust_to_weight",
               "height_at_landed_m", "support_margin_ground_g", "yaw_integ_pinned_frac_hold", "yaw_err_hold_mean_deg",
               "yaw_rate_rms_hold_deg_s",
               # v0.8
               "sk_rms_m", "sk_max_m", "sk_p95_m", "sk_max_all_m", "nav_rms_m", "nav_max_m", "nav_est_err_rms_m",
               "touchdown_from_hold_m", "touchdown_vh_ground", "tilt_sp_max_deg", "tilt_max_hold_deg",
               "limited_frac_hold", "psi_err_max_deg", "psi_err_end_deg", "gps_avail_frac_powered", "pos_mode_ATT_s",
               "pos_mode_DR_s", "gps_resets",
               # v0.8.1 touchdown (keep-trim policy: tip-over margin primary, tilt <= trim + 5 deg secondary)
               "touchdown_tip_margin_mps", "touchdown_lean_toward_deg", "touchdown_trim_deg", "touchdown_tilt_sp_deg",
               "wind_max_mps")
    out["percentiles"] = {k: _pct([r["metrics"].get(k) for r in rows]) for k in metrics}
    exc = [q.get("excursion_m") for r in rows for q in r.get("pushes", []) if q.get("in_flight")]
    out["percentiles"]["push_excursion_m"] = _pct(exc)
    out["percentiles"]["push_return_1m_s"] = _pct([q.get("return_1p0m_s") for r in rows for q in r.get("pushes", [])
                                                   if q.get("in_flight")])
    out["push_not_returned_1m"] = sum(1 for r in rows for q in r.get("pushes", [])
                                      if q.get("in_flight") and q.get("return_1p0m_s") is None)
    out["gps_mode"] = rows[0].get("gps_mode", "v071") if rows else None
    lr: dict[str, list[int]] = {}
    for r in rows:
        for rq in r["metrics"].get("pos_requests") or []:
            lr.setdefault(str(rq), []).append(r["i"])
    out["pos_request_runs"] = lr
    out["sk_max_by_wind_speed"] = _bins(rows, lambda r: flown_env(r)["wind_mps"],
                                        [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.01], lambda r: r["metrics"].get("sk_max_m"))
    out["touchdown_vh_ground_by_wind_speed_v08"] = _bins(
        rows, lambda r: flown_env(r)["wind_mps"],
        [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.01], lambda r: r["metrics"].get("touchdown_vh"))
    out["tip_margin_by_wind_speed"] = _bins(rows, lambda r: flown_env(r)["wind_mps"], [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.01],
                                            lambda r: r["metrics"].get("touchdown_tip_margin_mps"))
    out["percentiles"]["touchdown_abs_vz"] = _pct([abs(r["metrics"]["touchdown_vz"]) for r in rows
                                                   if r["metrics"].get("touchdown_vz") is not None])
    # drift / touchdown speeds by wind-speed bin and by wind-swing amplitude (no position sensing: the air moves it);
    # campaign round (bug fix): binned on the FLOWN wind (the v0.8 demo flies outdoors in 0-4 m/s, not the sample's
    # indoor draft)
    out["drift_by_wind_speed"] = _bins(rows, lambda r: flown_env(r)["wind_mps"], [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.01],
                                       lambda r: r["metrics"].get("drift_max_m"))
    out["touchdown_vh_air_by_wind_speed"] = _bins(rows, lambda r: flown_env(r)["wind_mps"], [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.01],
                                                  lambda r: r["metrics"].get("touchdown_vh_air"))
    out["touchdown_vh_ground_by_wind_speed"] = _bins(rows, lambda r: flown_env(r)["wind_mps"],
                                                     [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.01],
                                                     lambda r: r["metrics"].get("touchdown_vh"))
    out["drift_by_wind_swing"] = _bins(rows, lambda r: r["sample"].get("wind_swing_deg", 0.0), [0.0, 1e-9, 30.0, 60.0, 90.01],
                                       lambda r: r["metrics"].get("drift_max_m"))
    out["touchdown_vh_air_by_wind_swing"] = _bins(rows, lambda r: r["sample"].get("wind_swing_deg", 0.0),
                                                  [0.0, 1e-9, 30.0, 60.0, 90.01], lambda r: r["metrics"].get("touchdown_vh_air"))
    # push recovery by force and by height (demo)
    if pushes:
        def rec_rate(qq: list[dict[str, Any]]) -> dict[str, Any]:
            ok = [q for q in qq if q.get("in_flight") and not q.get("interlocks") and not q.get("abort")
                  and q.get("recovery_s") is not None and q["recovery_s"] <= vs.PUSH_CRITERIA["push_recovery_max_s"]]
            return {"n": len(qq), "recovered": len(ok), "rate": (len(ok) / len(qq)) if qq else None,
                    "aborts": sum(bool(q.get("abort")) for q in qq),
                    "peak_tilt_p50": _pct([q.get("peak_tilt_deg") for q in qq]).get("p50"),
                    "peak_tilt_max": _pct([q.get("peak_tilt_deg") for q in qq]).get("max"),
                    "disp_p50_m": _pct([q.get("peak_horiz_disp_m") for q in qq]).get("p50")}
        fe = [2.0, 5.0, 10.0, 15.0, 20.01]
        out["push_recovery_by_force"] = [{"lo": lo, "hi": hi, **rec_rate([q for q in pushes if lo <= q["force_n"] < hi])}
                                         for lo, hi in zip(fe[:-1], fe[1:])]
        he = [0.0, 0.2099, 0.3269, 0.45, 0.6, 0.9]
        out["push_recovery_by_height"] = [{"lo": lo, "hi": hi, **rec_rate([q for q in pushes if lo <= q["z_body_m"] < hi])}
                                          for lo, hi in zip(he[:-1], he[1:])]
        ie = [0.0, 1.0, 2.0, 3.0, 5.0, 8.0, 20.0]
        out["push_recovery_by_impulse"] = [{"lo": lo, "hi": hi, **rec_rate([q for q in pushes
                                                                             if lo <= (q.get("impulse_ns") or 0.0) < hi])}
                                           for lo, hi in zip(ie[:-1], ie[1:])]
    worst = sorted(rows, key=lambda r: -(abs(r["metrics"].get("touchdown_vz") or 0.0)))[:5]
    out["worst_touchdown_vz"] = [{"i": r["i"], "touchdown_vz": r["metrics"].get("touchdown_vz"), "failed": r["failed"],
                                  "interlocks": r["metrics"].get("interlocks")} for r in worst]
    # what the failures correlate with (Spearman rank correlation of each sampled scalar with each metric)
    scal = ("wind_mps", "turbulence_sigma_mps", "wind_swing_deg", "packet_loss", "gyro_bias_sigma_scale", "accel_bias_g",
            "vibration_g", "baro_noise_m", "baro_thrust_offset_m", "mass_scale", "thrust_scale", "servo_tau_scale",
            "crossflow_cd_scale", "ram_drag_scale", "batt_v_rest_cell", "initial_tilt_deg", "t_off_s",
            "thrust_voltage_exp", "swirl_scale", "wind_swing_period_s")
    corr: dict[str, dict[str, float]] = {}
    if n >= 5:
        for met in ("touchdown_vh_air", "drift_max_m", "hold_rms_err_m", "touchdown_vz", "rms_tilt_est_err_deg"):
            ys = [r["metrics"].get(met) for r in rows]
            if any(y is None for y in ys):
                continue
            yr = np.argsort(np.argsort([abs(y) for y in ys]))
            corr[met] = {}
            for k in scal:
                xs = [flown_env(r)[k] if k in ("wind_mps", "turbulence_sigma_mps") else r["sample"].get(k) for r in rows]
                if any(x is None for x in xs) or len(set(xs)) < 2:
                    continue
                xr = np.argsort(np.argsort(xs))
                cc = np.corrcoef(xr, yr)[0, 1]
                corr[met][k] = float(cc) if math.isfinite(cc) else 0.0
    out["rank_correlation"] = corr
    return out


def write_tracks(path: str, rows: list[dict[str, Any]], meta: dict[str, Any], hz: float = TRACK_HZ) -> str:
    """``tracks.npz`` per the renderer's TRACKS CONTRACT (tools/campaign_video.py): a common grid at ``hz``; pos linear,
    phase / state held, alive = within the run's log, after the end the last value; scalar per-run columns."""
    tr = [r["_track"] for r in rows]
    t0 = min(float(x["t"][0]) for x in tr)
    t1 = max(float(x["t"][-1]) for x in tr)
    grid = np.round(t0 + np.arange(int(math.floor((t1 - t0) * hz + 1e-9)) + 1) / hz, 9)
    R, T = len(tr), grid.size
    pos = np.zeros((R, T, 3), dtype=np.float32)
    alive = np.zeros((R, T), dtype=bool)
    phase = np.zeros((R, T), dtype=np.uint8)
    state = np.zeros((R, T), dtype=np.uint8)
    wind = np.zeros((R, T, 2), dtype=np.float32)
    for r, x in enumerate(tr):
        tl = np.asarray(x["t"], dtype=float)
        tq = np.minimum(grid, tl[-1])
        hi = np.clip(np.searchsorted(tl, tq, side="right") - 1, 0, tl.size - 1)
        for j in range(3):
            pos[r, :, j] = np.interp(tq, tl, np.asarray(x["pos"])[:, j])
        for j in range(2):
            wind[r, :, j] = np.interp(tq, tl, np.asarray(x["wind_xy"])[:, j])
        phase[r] = np.asarray(x["phase"])[hi]
        state[r] = np.asarray(x["state"])[hi]
        alive[r] = (grid <= tl[-1] + 1e-9) & (grid >= tl[0] - 1e-9)

    def col(get) -> np.ndarray:
        return np.array([np.nan if get(rw) is None else float(get(rw)) for rw in rows])
    data: dict[str, Any] = {
        "t": grid, "pos": pos, "alive": alive, "phase": phase, "state": state,
        "outcome": np.array([int(rw.get("outcome", 4)) for rw in rows], dtype=np.uint8),
        "outcome_names": np.array(list(OUTCOME_NAMES)), "outcome_labels_ko": np.array(list(OUTCOME_LABELS_KO)),
        "meta": np.array(json.dumps(vs._clean(meta), ensure_ascii=False)), "wind_xy": wind,
        "failed": np.array([";".join(rw["failed"]) for rw in rows]),
        "wind_mps": col(lambda rw: flown_env(rw)["wind_mps"]), "wind_dir_deg": col(lambda rw: rw["sample"]["wind_dir_deg"]),
        "wind_swing_deg": col(lambda rw: rw["sample"].get("wind_swing_deg", 0.0)),
        "wind_swing_period_s": col(lambda rw: rw["sample"].get("wind_swing_period_s")),
        "touchdown_t": col(lambda rw: rw["metrics"].get("touchdown_t_s")),
        "touchdown_vz": col(lambda rw: rw["metrics"].get("touchdown_vz")),
        "touchdown_vh": col(lambda rw: rw["metrics"].get("touchdown_vh_air")),
        "touchdown_vh_ground": col(lambda rw: rw["metrics"].get("touchdown_vh")),
        "touchdown_tilt_deg": col(lambda rw: rw["metrics"].get("touchdown_tilt_deg")),
        "touchdown_distance_m": col(lambda rw: rw["metrics"].get("touchdown_distance_m")),
        "drift_max_m": col(lambda rw: rw["metrics"].get("drift_max_m")),
        "seed": np.array([int(rw["sample"]["seed"]) for rw in rows], dtype=np.int64),
        "run_index": np.array([int(rw["i"]) for rw in rows], dtype=np.int32),
    }
    if any("hold_xy" in x for x in tr):
        # v0.8 (renderer contract, tools/campaign_video.py): the TRUE CG at each run's hold capture, station keeping and
        # the touchdown against it (NaN when a run never set one)
        data["hold_xy"] = np.array([np.asarray(x.get("hold_xy", [np.nan, np.nan]), dtype=float) for x in tr])
        data["sk_rms_m"] = col(lambda rw: rw["metrics"].get("sk_rms_m"))
        data["sk_max_m"] = col(lambda rw: rw["metrics"].get("sk_max_m"))
        data["touchdown_from_hold_m"] = col(lambda rw: rw["metrics"].get("touchdown_from_hold_m"))
        data["touchdown_tip_margin_mps"] = col(lambda rw: rw["metrics"].get("touchdown_tip_margin_mps"))
        data["gps_mode"] = np.array([str(rw.get("gps_mode", "v071")) for rw in rows])
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez_compressed(path, **data)
    return path


def plot_summary(rows: list[dict[str, Any]], path: str, title: str) -> None:
    """Four panels: touchdown |v_z|, touchdown horizontal speed (vs the mean wind) vs drift, hold RMS (field) or push peak
    tilt vs push height (demo), attitude-estimate error vs the sampled gyro-bias wander - criterion lines dashed, failed
    runs in red."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pal = vs.PALETTE
    fig, ax = plt.subplots(2, 2, figsize=(12, 8.5), facecolor=vs.SURFACE)
    fig.suptitle(title, fontsize=10, color=vs.INK, x=0.01, ha="left")
    ok = [r["verdict"] == "PASS" for r in rows]
    col = [pal[2] if o else pal[7] for o in ok]
    crit = vs.NOMINAL_CRITERIA

    def style(a: Any, ttl: str, xl: str, yl: str) -> None:
        a.set_facecolor(vs.SURFACE)
        for s_ in ("top", "right"):
            a.spines[s_].set_visible(False)
        a.tick_params(colors=vs.INK2, labelsize=8)
        a.grid(True, color=vs.GRID, lw=0.6)
        a.set_title(ttl, fontsize=9.5, color=vs.INK, loc="left")
        a.set_xlabel(xl, fontsize=8, color=vs.INK2)
        a.set_ylabel(yl, fontsize=8, color=vs.INK2)

    tdz = [abs(r["metrics"]["touchdown_vz"]) if r["metrics"].get("touchdown_vz") is not None else np.nan for r in rows]
    a = ax[0, 0]
    style(a, "Touchdown |v_z| per run", "run", "m/s")
    a.scatter([r["i"] for r in rows], tdz, c=col, s=14)
    a.axhline(crit["touchdown_vz_max_mps"], color=vs.MUTED, ls="--", lw=0.9)
    a = ax[0, 1]
    if rows and rows[0].get("gps_mode", "v071") != "v071":
        # v0.8.1 (campaign round): the judged touchdown criterion is the energy-method tip-over margin (> 0); the speed
        # over the ground is reported (dotted 0.5 m/s), no longer judged on its own (params touchdown_policy)
        style(a, "v0.8.1: touchdown tip-over margin vs speed over the ground (dotted 0.5 m/s: reported only)",
              "touchdown speed over the ground [m/s]", "tip-over margin [m/s]")
        a.scatter([r["metrics"].get("touchdown_vh") if r["metrics"].get("touchdown_vh") is not None else np.nan
                   for r in rows],
                  [r["metrics"].get("touchdown_tip_margin_mps") if r["metrics"].get("touchdown_tip_margin_mps") is not None
                   else np.nan for r in rows], c=col, s=14)
        c8 = vs.v08_criteria(vs.vehicle().params, rows[0].get("profile"))
        a.axhline(c8["tip_margin_min_mps"], color=vs.MUTED, ls="--", lw=0.9)
        a.axvline(c8["touchdown_ground_speed_mps"], color=vs.MUTED, ls=":", lw=0.9)
    else:
        style(a, "Touchdown horizontal speed vs the mean wind, vs the largest drift", "drift [m]", "m/s")
        a.scatter([r["metrics"].get("drift_max_m") or np.nan for r in rows],
                  [r["metrics"].get("touchdown_vh_air") if r["metrics"].get("touchdown_vh_air") is not None else np.nan
                   for r in rows], c=col, s=14)
        a.axhline(crit["touchdown_vh_max_mps"], color=vs.MUTED, ls="--", lw=0.9)
    a = ax[1, 0]
    pushes = [(q, r["verdict"] == "PASS") for r in rows for q in r.get("pushes", []) if q.get("in_flight")]
    if pushes:
        style(a, "Push peak tilt vs push height (size ~ impulse; red = abort / not recovered)", "z on the body [m]", "deg")
        a.scatter([q["z_body_m"] for q, _ in pushes], [q["peak_tilt_deg"] for q, _ in pushes],
                  s=[8 + 6 * (q.get("impulse_ns") or 0.0) for q, _ in pushes],
                  c=[pal[7] if (q.get("abort") or q.get("recovery_s") is None or q["recovery_s"] > 2.0) else pal[2]
                     for q, _ in pushes])
        a.axvspan(0.2099, 0.3269, color="#000000", alpha=0.08)
        a.set_yscale("log")
    else:
        style(a, "Hold RMS (true - reference) per run", "run", "m")
        a.scatter([r["i"] for r in rows], [r["metrics"].get("hold_rms_err_m") or np.nan for r in rows], c=col, s=14)
        a.axhline(crit["hold_rms_err_max_m"], color=vs.MUTED, ls="--", lw=0.9)
    a = ax[1, 1]
    style(a, "RMS attitude-estimate error vs sampled gyro-bias wander", "gyro_bias_sigma_scale [-]", "RMS tilt error [deg]")
    a.scatter([r["sample"]["gyro_bias_sigma_scale"] for r in rows],
              [r["metrics"].get("rms_tilt_est_err_deg") or np.nan for r in rows], c=col, s=14)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=100, facecolor=vs.SURFACE)
    plt.close(fig)


def run_campaign(n: int, seed: int = DEFAULT_SEED, workers: int = 1, estimator: str = "gated_mahony",
                 profile: str = "field", vgains: dict | None = None, src: str = "", out_dir: str | None = None,
                 log=print, pushes: str = "any", gps_mode: str = "on", opts: dict | None = None) -> dict[str, Any]:
    """Run N campaign runs and write every output; returns the summary.  ``pushes`` (demo): "any" | "light";
    ``gps_mode`` (v0.8): "on" (default: the GPS build), "off" (the v0.8 build without it), "v071" (the v0.7.1 build in the
    v0.7.1 environment); "both" = ``run_paired``; ``opts`` = campaign-wide overrides (``apply_opts``)."""
    if pushes not in PUSH_SETS:
        raise ValueError(f"pushes {pushes!r} not in {PUSH_SETS}")
    if gps_mode == "both":
        return run_paired(n, seed, workers, estimator, profile, vgains, src, out_dir, log, pushes, opts)
    if gps_mode not in GPS_MODES:
        raise ValueError(f"gps_mode {gps_mode!r} not in {GPS_MODES}")
    if profile == "hybrid_demo":
        pushes = "light"            # v0.9 campaign round (bug fix): ``sample`` always flies the hybrid demo's light pushes
    elif profile != "demo":
        pushes = "any"
    tag = (profile + ("" if pushes == "any" else f"_{pushes}") + ("" if gps_mode == "v071" else f"_gps{gps_mode}")
           + opts_tag(opts))
    out_dir = out_dir or os.path.join(vs.OUT_DIR, f"campaign_{tag}_n{n}_s{seed}"
                                      + ("" if estimator == "gated_mahony" else f"_{estimator}"))
    os.makedirs(out_dir, exist_ok=True)
    t0 = time.perf_counter()
    jobs = [(seed, i, estimator, vgains, profile, pushes, gps_mode, opts) for i in range(n)]
    log(f"VTOL campaign: profile {profile}, pushes {pushes}, GPS {gps_mode}, N {n}, seed {seed}, estimator {estimator}, "
        f"{workers} workers, gains {vgains} ({src}) -> {out_dir}")
    rows: list[dict[str, Any]] = list(vs.pool_map(run_one, [(j,) for j in jobs], workers, log=log))
    rows.sort(key=lambda r: r["i"])
    summ = summarize(rows)
    samp = SAMPLING if profile == "field" else {**SAMPLING, **SAMPLING_DEMO}
    if pushes == "light":
        samp = {**samp, **SAMPLING_DEMO_LIGHT, "push_z_body_m": light_push_band_m()}
    # v0.9 campaign round (bug fix): the light-push text from the band actually flown (on the VTOL vehicle 0.33-0.46 m,
    # ABOVE the intake windows and the CG - not "near the CG") and Korean titles for the tether profiles (they were
    # titled as the hard-push demo); a fixed T_max range (opts) is named in the title
    zb = light_push_band_m()
    lf, ld = SAMPLING_DEMO_LIGHT["push_force_n"], SAMPLING_DEMO_LIGHT["push_duration_s"]
    light_ko = (f"계획대로 가볍게 밀기({lf[0]:g}~{lf[1]:g} N × {ld[0]:g}~{ld[1]:g} s, 몸통 높이 {zb[0]:.2f}~{zb[1]:.2f} m)")
    if profile == "field":
        title_ko = f"현장 프로파일 무작위 비행 {n}회"
    elif profile == "tether_test":
        title_ko = f"유선 시험 프로파일(배터리 없는 유선 전원, 실내 GPS 끔 / 실외 GPS 켬, 밀기 없음) 무작위 비행 {n}회"
    elif profile == "hybrid_demo":
        title_ko = f"하이브리드 시연 프로파일(유선 + 650 mAh 버퍼, 실내 / 실외) - {light_ko} 무작위 비행 {n}회"
    elif pushes == "light":
        title_ko = f"시연 프로파일 - {light_ko} 무작위 비행 {n}회"
    else:
        title_ko = f"시연 프로파일(동체 어디든 세게 밀기 포함) 무작위 비행 {n}회"
    rng_t = (opts or {}).get("t_max_kgf_range")
    if rng_t:
        title_ko += (f" - 최대 추력 {float(rng_t[0]):g} kgf 고정" if float(rng_t[0]) == float(rng_t[1])
                     else f" - 최대 추력 {float(rng_t[0]):g}~{float(rng_t[1]):g} kgf")
    tether_prof = profile in ("tether_test", "hybrid_demo")
    v8 = gps_mode != "v071"
    if v8:
        # campaign round (bug fix): the v0.8 envelope REPLACES the sample's wind / turbulence (``build_scenario``: the demo
        # flies outdoors in demo_wind_mps, turbulence = intensity x wind, field gusts clipped to the operator limit) -
        # the flown ranges and the receiver ranges go first (the renderer's title card lists the first entries)
        G = SAMPLING_GPS
        ti = float(G["turbulence_intensity"])
        # v0.9 campaign round (bug fix): every profile but the field flies the demo's outdoor wind (the tether profiles
        # indoors the demo drafts) - the tether profiles were listed with the field's 0-6 m/s and gusts
        w_lo, w_hi = G["demo_wind_mps"] if profile != "field" else SAMPLING["wind_mps"]
        lim = vs.vehicle().params["mission"]["vtol"]["position_hold"]["wind_limits_mps"]
        first = {"wind_mps": (w_lo, w_hi), "turbulence_sigma_mps": (ti * w_lo, ti * w_hi)}
        if tether_prof:
            first["indoor_wind_mps"] = SAMPLING_DEMO["wind_mps"]
            first["p_indoor"] = SAMPLING_V09["p_indoor"]
        if profile == "field":
            first["gust_mps"] = f"{SAMPLING['gust_mps'][0]:g}~{SAMPLING['gust_mps'][1]:g}, <= {float(lim['field_gust']):g} - wind"
        # v0.9 campaign round (bug fix): the v0.9 plant ranges (T_max envelope, rotor, vane play, vane effectiveness, the
        # swirl sign) were missing from the meta - listed right after the wind (the renderer shows the first entries)
        if rng_t:
            first["t_max_kgf"] = (float(rng_t[0]), float(rng_t[1]))
        else:
            first["t_max_kgf"] = SAMPLING_V09["t_max_kgf"]
        first["swirl_scale"] = SAMPLING["swirl_scale"]
        first["p_swirl_negative"] = SAMPLING_FIX9["p_swirl_negative"]
        first["vane_force_scale"] = SAMPLING_FIX9["vane_force_scale"]
        for k in ("fan_power_model", "rotor_inertia_scale", "lost_motion_deg", "gear_backlash_deg"):
            first[k] = SAMPLING_V09[k]
        first.update({f"gps_{k}": G[k] for k in ("latency_s", "pos_gm_sigma_m", "pos_gm_tau_s", "vel_white_mps",
                                                  "sky_blocked", "outage_s")})
        first["gps_heading_err_sigma_deg"] = G["heading_err_sigma_deg"]
        samp = {**first, **{k: v for k, v in samp.items() if k not in first}, "v0.8": SAMPLING_GPS,
                "v0.9": SAMPLING_V09, "v0.9_fix": SAMPLING_FIX9}
        title_ko = ("" if tether_prof else {"on": "GPS 위치 유지 - ", "off": "GPS 끔(비교) - "}[gps_mode]) + title_ko
    meta = {"written_by": "sim/vtol_campaign.py (v0.8)", "date": _dt.datetime.now().isoformat(timespec="seconds"),
            "params_sha256": vs.params_sha256(), "campaign_seed": seed, "n": n, "estimator": estimator, "profile": profile,
            "pushes": pushes, "gps_mode": gps_mode, "gps_on": gps_mode == "on", "opts": opts,
            # v0.9 campaign round: the vehicle flown (renderer contract v0.9: tools/campaign_video.py names it on the
            # title card through sim_video.hardware_of); the tether profiles fly indoors without the GPS (p_indoor)
            "hardware_config": vs.hardware_config(),
            "power_source": {"tether_test": "tether", "hybrid_demo": "hybrid"}.get(profile, "battery"),
            "gps_note": ("indoor runs fly the build without the GPS (runs' gps_mode 'off')" if tether_prof else None),
            "title": f"VTOL campaign, {profile} profile{'' if pushes == 'any' else ', ' + pushes + ' pushes'}, GPS "
                     f"{gps_mode}, N {n}, seed {seed}, estimator {estimator}", "title_ko": title_ko,
            "outcome_note": "other (index 4) = the plant never lifted off (T/W < 1) or another end; see no_liftoff_runs",
            "vgains": vgains, "vgains_source": src, "sampling_ASSUMED": samp,
            "criteria": dict(vs.v08_criteria(vs.vehicle().params, profile)) if v8 else vs.NOMINAL_CRITERIA,
            "push_criteria": vs.PUSH_CRITERIA,
            "criteria_text": vs.V08_CRITERIA_TEXT if v8 else vs.CRITERIA_TEXT + " | " + vs.PUSH_CRITERIA_TEXT,
            "model_notes": vs.MODEL_NOTES + (" " + vs.MODEL_NOTES_GPS if gps_mode == "on" else ""),
            "notes_ko": vs.MODEL_NOTES_KO + (" " + vs.MODEL_NOTES_GPS_KO if gps_mode == "on" else ""),
            "tracks_note": "touchdown_vh = relative to the mean wind (v0.7.1 criterion); touchdown_vh_ground = ground-relative "
                           "(the v0.8 criterion); hold_xy = the TRUE CG at the hold capture (v0.8 GPS runs)",
            "wall_s": round(time.perf_counter() - t0, 1), "workers": workers}
    if v8:
        cv = vs.vehicle().params["mission"]["vtol"]["position_hold"]["criteria_v08_ASSUMED"]
        meta["criteria"]["station_keeping_max_m"] = cv["station_keeping_max_m"]
    write_tracks(os.path.join(out_dir, "tracks.npz"), rows, meta)
    for r in rows:
        r.pop("_track", None)
    with open(os.path.join(out_dir, "runs.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(vs._clean({"_meta": meta, "runs": rows}), f, indent=1, ensure_ascii=False)
    with open(os.path.join(out_dir, "runs.csv"), "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(CSV_KEYS), lineterminator="\n")
        w.writeheader()
        for r in rows:
            w.writerow(flat_row(r))
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(vs._clean({"_meta": meta, "summary": summ}), f, indent=1, ensure_ascii=False)
    lines = [f"VTOL campaign {tag} N {n} seed {seed} estimator {estimator}: PASS {summ['pass']}/{n} (all criteria), "
             f"vertical criteria {summ['vertical_pass_rate']:.0%}; outcomes {summ['outcome_counts']}",
             f"  no lift-off (T/W < 1): {summ['no_liftoff_runs']}; cannot fly (no lift-off or never above the flare "
             f"band): {summ['cannot_fly_runs']}; of the {summ['n_flown']} that flew: PASS "
             f"{summ['pass_flown']}, vertical criteria "
             + ("-" if summ['vertical_pass_rate_flown'] is None else f"{summ['vertical_pass_rate_flown']:.0%}"),
             f"  I3 abort causes: {summ['abort_rate_cause_counts']}; push abort causes: {summ['push_abort_cause_counts']}"]
    lines += [f"  {c}: " + ("n/a" if v is None else f"{v:.0%}") + f" (n {summ['check_n_applicable'][c]})"
              for c, v in summ["check_pass_rate"].items()]
    if summ.get("push_recovered_rate") is not None:
        lines.append(f"  pushes recovered: {summ['push_recovered_rate']:.0%} of {summ['n_pushes']}")
    lines.append(f"  interlocks: {summ['interlock_counts']}; landed by: {summ['landed_by']}")
    lines.append(f"  false touchdowns (motor off in the air): {summ['false_touchdown_runs']}; touchdown not confirmed: "
                 f"{summ['touchdown_not_confirmed_runs']}; aborts: {summ['abort_runs']}")
    lines.append(f"  failure modes: {summ['failure_modes']}")
    if v8:
        lines.append(f"  position-hold requests: { {k: len(v) for k, v in summ['pos_request_runs'].items()} }; pushes not back "
                     f"within 1 m in the window: {summ['push_not_returned_1m']}")
    for k in ("touchdown_abs_vz", "touchdown_vh_air", "touchdown_vh", "touchdown_tilt_deg", "drift_max_m", "hold_rms_err_m",
              "rms_tilt_est_err_deg", "max_tilt_est_err_deg", "sat_high_after_ramp_s", "push_peak_tilt_max_deg",
              "kt_hold_err_vs_thrust_scale", "plant_thrust_to_weight", "sk_rms_m", "sk_max_m", "nav_rms_m",
              "touchdown_from_hold_m", "psi_err_max_deg", "push_excursion_m", "push_return_1m_s", "tilt_max_hold_deg",
              "touchdown_tip_margin_mps", "touchdown_trim_deg"):
        pc = summ["percentiles"].get(k) or {}
        if pc.get("n"):
            lines.append(f"  {k}: p50 {pc['p50']:.3g}, p95 {pc['p95']:.3g}, max {pc['max']:.3g} (n {pc['n']})")
    lines.append(f"  wall {meta['wall_s']:.0f} s")
    with open(os.path.join(out_dir, "summary.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    plot_summary(rows, os.path.join(out_dir, "summary.png"),
                 f"VTOL campaign {tag} N {n}, seed {seed}, estimator {estimator} (green PASS, red FAIL; dashed = criterion)")
    log("\n".join(lines))
    return {"out_dir": out_dir, "summary": summ, "meta": meta, "rows": rows}


PAIRED_KEYS = ("sk_rms_m", "sk_max_m", "touchdown_vh", "touchdown_from_hold_m", "drift_max_m", "touchdown_tilt_deg",
               "touchdown_vz", "touchdown_tip_margin_mps", "touchdown_distance_m")


def run_paired(n: int, seed: int = DEFAULT_SEED, workers: int = 1, estimator: str = "gated_mahony", profile: str = "field",
               vgains: dict | None = None, src: str = "", out_dir: str | None = None, log=print,
               pushes: str = "any", opts: dict | None = None) -> dict[str, Any]:
    """v0.8 paired campaign: every sample flown with the GPS build ("on") and without the GPS ("off", the v0.8 build) -
    common random numbers; writes both campaigns and ``paired.json`` (per run: both verdicts, failed checks, the key
    metrics; counts of runs that pass / abort only with or only without the GPS)."""
    if profile != "demo":
        pushes = "any"
    tag = profile + ("" if pushes == "any" else f"_{pushes}") + opts_tag(opts)
    base = out_dir or os.path.join(vs.OUT_DIR, f"campaign_{tag}_n{n}_s{seed}_paired")
    on = run_campaign(n, seed, workers, estimator, profile, vgains, src, os.path.join(base, "gps_on"), log, pushes, "on",
                      opts)
    off = run_campaign(n, seed, workers, estimator, profile, vgains, src, os.path.join(base, "gps_off"), log, pushes, "off",
                       opts)
    ron = {r["i"]: r for r in on["rows"]}
    roff = {r["i"]: r for r in off["rows"]}
    runs = []
    for i in sorted(ron):
        a, b = ron[i], roff[i]
        runs.append({"i": i, "verdict_on": a["verdict"], "verdict_off": b["verdict"], "failed_on": a["failed"],
                     "failed_off": b["failed"], "abort_on": a["metrics"].get("t_abort_s") is not None,
                     "abort_off": b["metrics"].get("t_abort_s") is not None,
                     **{f"{k}_on": a["metrics"].get(k) for k in PAIRED_KEYS},
                     **{f"{k}_off": b["metrics"].get(k) for k in PAIRED_KEYS}})

    def le(x: Any, lim: float) -> bool:
        return x is not None and x <= lim
    summ = {"n": len(runs), "pass_on": sum(r["verdict_on"] == "PASS" for r in runs),
            "pass_off": sum(r["verdict_off"] == "PASS" for r in runs),
            "abort_on": sum(r["abort_on"] for r in runs), "abort_off": sum(r["abort_off"] for r in runs),
            "abort_only_with_gps": [r["i"] for r in runs if r["abort_on"] and not r["abort_off"]],
            "abort_only_without_gps": [r["i"] for r in runs if r["abort_off"] and not r["abort_on"]],
            "pass_only_with_gps": [r["i"] for r in runs if r["verdict_on"] == "PASS" and r["verdict_off"] != "PASS"],
            "pass_only_without_gps": [r["i"] for r in runs if r["verdict_off"] == "PASS" and r["verdict_on"] != "PASS"],
            "touchdown_vh_ground_le_0p5": {"on": sum(le(r["touchdown_vh_on"], 0.5) for r in runs),
                                           "off": sum(le(r["touchdown_vh_off"], 0.5) for r in runs)},
            "touchdown_tip_margin_gt_0": {"on": sum((r["touchdown_tip_margin_mps_on"] or -1.0) > 0.0 for r in runs),
                                          "off": sum((r["touchdown_tip_margin_mps_off"] or -1.0) > 0.0 for r in runs)},
            "touchdown_distance_m": {"on": _pct([r["touchdown_distance_m_on"] for r in runs]),
                                     "off": _pct([r["touchdown_distance_m_off"] for r in runs])},
            "drift_max_m": {"on": _pct([r["drift_max_m_on"] for r in runs]), "off": _pct([r["drift_max_m_off"] for r in runs])},
            "touchdown_vh": {"on": _pct([r["touchdown_vh_on"] for r in runs]),
                             "off": _pct([r["touchdown_vh_off"] for r in runs])}}
    os.makedirs(base, exist_ok=True)
    with open(os.path.join(base, "paired.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(vs._clean({"_meta": {"written_by": "sim/vtol_campaign.py run_paired (v0.8)", "campaign_seed": seed,
                                       "n": n, "profile": profile, "pushes": pushes, "vgains_source": src,
                                       "date": _dt.datetime.now().isoformat(timespec="seconds")},
                             "summary": summ, "runs": runs}), f, indent=1, ensure_ascii=False)
    log(f"paired {tag} N {n} seed {seed}: PASS on {summ['pass_on']} / off {summ['pass_off']}; aborts on {summ['abort_on']} "
        f"/ off {summ['abort_off']} (only with GPS {summ['abort_only_with_gps']}); touchdown <= 0.5 m/s over the ground "
        f"on {summ['touchdown_vh_ground_le_0p5']['on']} / off {summ['touchdown_vh_ground_le_0p5']['off']}")
    return {"out_dir": base, "summary": summ, "on": on, "off": off}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--n", type=int, default=20, help="number of runs (10 ... 1000)")
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED, help="campaign seed")
    ap.add_argument("--profile", default="field", choices=vs.PROFILES)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--estimator", default="gated_mahony", choices=ESTIMATOR_CHOICES,
                    help="NAV attitude estimator: gated_mahony (v0.7.1), pad_gyro (v0.7.0), truth_noise (what-if)")
    ap.add_argument("--assumed", action="store_true", help="params gains_ASSUMED / nav_estimator, not vtol_gains.json")
    ap.add_argument("--pushes", default="any", choices=PUSH_SETS,
                    help="demo pushes: any (anywhere, 2-20 N) | light (the live demo as planned: 5-15 N x 0.2-0.4 s near the CG)")
    ap.add_argument("--gps", default="on", choices=GPS_MODES,
                    help="v0.8: on (the GPS position-hold build), off (the v0.8 build without the GPS), both (paired on + "
                         "off on the same samples), v071 (the v0.7.1 build in the v0.7.1 environment)")
    ap.add_argument("--gps-set", nargs="*", default=[], metavar="FIELD=VALUE",
                    help="fixed receiver values for every run, e.g. latency_s=0.45 (physics-review fix: the mis-set GPS "
                         "latency case)")
    ap.add_argument("--wind-limit", action="store_true",
                    help="field: every run at the operator wind limit (params wind_limits_mps, TI 0.2), the spread kept")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    if not 1 <= args.n <= 100000:
        raise SystemExit("--n must be >= 1")
    opts: dict[str, Any] = {}
    if args.gps_set:
        opts["gps"] = {kv.split("=", 1)[0]: float(kv.split("=", 1)[1]) for kv in args.gps_set}
    if args.wind_limit:
        opts["wind_limit"] = True
    vg, src = (None, "params.json mission.vtol.gains_ASSUMED / nav_estimator") if args.assumed else vs.load_vgains()
    run_campaign(args.n, args.seed, args.workers, args.estimator, args.profile, vg, src, args.out, pushes=args.pushes,
                 gps_mode=args.gps, opts=opts or None)
    return 0


if __name__ == "__main__":
    sys.exit(main())
