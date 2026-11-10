"""Scenario runner, gain tuner, timing / estimator studies, plots and report
(SPEC §9) for the ATmega128 flight computer.

Plant model (``sim.sim6dof``): 250 Hz control with the measured v0.6.3
sample -> output-register delay (``sim.fc_timing.FC_COMPUTE_DELAY_S_MEASURED``,
4.71 ms, sub-tick), servo + ESC commands latched at the 50 Hz PWM frames (fixed
phase to the control tick) and decoded at the pulse falling edge, then servo lag
+ rate limit + saturation; MPU-6050 gyro integrated by the firmware estimator
(pad alignment, gyro-only after launch; the NAV's pad -> flight switch 2 ticks
behind the supervisor); barometer (BMP280 model since 2026-11-10, sim6dof.BARO_MODELS); the measured on-vehicle link delays of
``sim.link_model``.

Scenarios (v0.4: every flight runs the ground station, the LoRa link model and
the onboard supervisor of SPEC §8.8): S1 hover-stabilise from 10 deg (T = W);
S2 nominal ground-assisted flight to ``mission.target_altitude_m`` with wind +
gust and ``link.sim_packet_loss``; S3 competition flight to 250 m
(informational: apogee limit and geofence raised in the scenario only); S4 one
vane hard-over at launch + 2 s (expect I3 ABORT -> chute); S5 gain sensitivity
+-50 %; S6 total link loss 4 s after launch (expect I1 ABORT -> chute); S7
assist off and nobody cuts or releases (expect I4 cut, I6 release); S8 30 %
packet loss (no unsafe outcome; ABORT allowed) plus a link-seed Monte Carlo.
Informational: S2t (truth + 0.5 deg estimator), S2b (worst-case gyro-bias
drift), S2s (S2 on the sx127x backend), S4x (S4 with I3 off: why I3 exists), and
the what-if runs of the proposed fixes (coast_drag_factor recalibrated from S7,
delay-compensated assist release) - they never change a contract verdict.

Gains: the plant did not change in v0.4, so the existing ``gains.json`` is
reused when it still meets the margin targets on this run (linear PM >= 30 deg,
GM >= 2 on both axes, S1 PASS); otherwise (or with ``--retune``) the tuner
(coarse grids, then Nelder-Mead on log-gains, deterministic) minimises the
unchanged S1 cost ``COST_DOC``.  ``gains.json`` is rewritten either way with the
current params sha256.  After the scenarios: timing sensitivity, linear loop
margins (``sim.loop_margins``), 6-DOF gain-margin search and an S2 Monte Carlo
(estimator + link; its seeds stop a few seconds after the release, so their S2
verdict is reported without the touchdown term and labelled so).

Outputs: ``sim/out/<scenario>.png``, ``timing.png``, ``estimator.png``,
``S5.png``, ``tuning.png`` (when tuned), ``sim_report.md``, ``gains.json``,
``gains.h``, ``metrics.json``.

Invariant: everything here reads the vehicle through ``sim.vehicle`` and the
gains through ``params.control`` / ``gains.json``; verdict thresholds are the
ones written in the report next to each number.

Usage: ``python sim/run_scenarios.py [--no-tune] [--retune] [--quick] [--mass-source auto|cad|sizing|estimate]
                                      [--estimator pad_gyro|truth_noise|mahony] [--out DIR]``
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
from dataclasses import replace
from typing import Any

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import numpy as np  # noqa: E402
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from scipy.optimize import minimize  # noqa: E402

from sim import link_model  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim.controller import GAIN_KEYS, gains_from_control  # noqa: E402
from sim.estimator import Mpu6050  # noqa: E402
from sim.fc_timing import FCTiming, frame_hz_from_timer, mean_wait_after_available, timer_of_channel  # noqa: E402
from sim.loop_margins import axis_margins  # noqa: E402
from sim.onboard import State, SupervisorParams  # noqa: E402
from sim.sim6dof import (ESTIMATORS, FlightSim, Scenario, ScriptedOperator, SensorConfig, SimResult,  # noqa: E402
                         command_latencies, compute_metrics)
from sim.vehicle import PROJECT_ROOT, Vehicle, load_vehicle  # noqa: E402

OUT_DIR = os.path.join(PROJECT_ROOT, "sim", "out")

# validated categorical palette (dataviz skill reference instance, light surface; 4-slot set re-validated:
# CVD worst adjacent dE 9.1, contrast WARN for slots 3-4 -> every plot carries a legend, the report the tables)
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
INK, INK2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#fcfcfb"
TUNED_KEYS_XY = ("att_kp_xy", "rate_kp_xy", "rate_ki_xy", "rate_kd_xy")
TUNED_KEYS_Z = ("att_kp_z", "rate_kp_z", "rate_ki_z")
TUNED_KEYS = TUNED_KEYS_XY + TUNED_KEYS_Z
S5_KEYS = TUNED_KEYS_XY + ("all",)
S1_TILT_DEG = 10.0
S1_T_END = 5.0
TUNE_T_END = 4.0
# 6-DOF stability criterion of the gain-margin search: a limit cycle shows up as vane-command
# fluctuation about its 1 s mean; with noise only it stays below 0.1 deg (measured, see report)
JITTER_UNSTABLE_DEG = 1.0
MARGIN_F_MAX = 16.0
# servo verdict thresholds (classical robustness targets)
PM_MIN_DEG = 30.0
GM_MIN_X = 2.0
MC_SEEDS_FULL, MC_SEEDS_QUICK = 20, 5
MC_EXTRA_S = 2.0                   # S2 Monte Carlo seeds run to the nominal S2 release + 1 s + this (before touchdown)
S8_SEEDS_FULL, S8_SEEDS_QUICK = 10, 3
# flight verdict thresholds (task brief of the v0.4 ground-link change)
APOGEE_MIN_FRACTION = 0.8          # apogee >= 0.8 x target
RELEASE_VZ_MAX_MPS = 5.0           # release (servo open) while |v_z| < 5 m/s
TOUCHDOWN_MAX_MPS = 7.0            # touchdown speed < 7 m/s
FAULT_TO_ABORT_MAX_S = 5.0         # S4: ABORT within 5 s of the fault
S6_OUTAGE_AFTER_LAUNCH_S = 4.0
S8_PACKET_LOSS = 0.30
COAST_FACTOR_STEP = 0.05           # what-if: recalibrated coast_drag_factor rounded up to this step ...
COAST_FACTOR_MARGIN = 1.03         # ... after this margin on the measured requirement


# --------------------------------------------------------------------------- #
# scenarios
# --------------------------------------------------------------------------- #
# v0.9 plant round: the plant of the ballistic runs - {} = the v0.9 plant (direct-drive vane play, rotor gyro / spool
# reaction, sim.sim6dof defaults); --plant v081 = both off (the v0.8.1 plant, for the before / after comparison)
PLANT_OVERRIDES: dict = {}
PLANT_V081 = {"vane_play_deg": 0.0, "rotor_inertia_kg_m2": 0.0}


def _fs(veh: Vehicle, sc: Scenario, gains: dict | None = None) -> FlightSim:
    """FlightSim with the plant overrides of this run (PLANT_OVERRIDES)."""
    return FlightSim(veh, replace(sc, **PLANT_OVERRIDES) if PLANT_OVERRIDES else sc, gains)


def scenario_s1(sensors: SensorConfig, gains: dict | None = None, t_end: float = S1_T_END, seed: int = 11,
                name: str = "S1", title: str = "", timing: FCTiming | None = None) -> Scenario:
    return Scenario(name=name, title=title or f"hover-stabilise from {S1_TILT_DEG:.0f} deg initial tilt, fixed thrust = weight",
                    t_end=t_end, seed=seed, bypass_state_machine=True, fixed_throttle=None,
                    initial_tilt_deg=S1_TILT_DEG, initial_tilt_axis_deg=0.0, initial_altitude_m=20.0,
                    gains=gains, sensors=sensors, timing=timing or FCTiming())


def _link_desc(veh: Vehicle, loss: float | None = None, radio: str | None = None) -> str:
    lk = veh.params["flight_computer"]["link"]
    # 2026-11-10: the scenario's link model (sim.link_model.model_radio: link.vehicle_radio when modelled)
    return (f"{radio or link_model.model_radio(veh.params)} link, "
            f"{100.0 * (lk['sim_packet_loss'] if loss is None else loss):.0f} % packet loss")


def scenario_s2(veh: Vehicle, sensors: SensorConfig, timing: FCTiming | None = None) -> Scenario:
    a = veh.params["aero"]
    return Scenario(name="S2", title=f"nominal ground-assisted flight to {veh.params['mission']['target_altitude_m']:.0f} m "
                                     f"({_link_desc(veh)}), wind {a['wind_mps_default']:.0f} m/s + gust "
                                     f"{a['gust_mps_default']:.0f} m/s for {a['gust_duration_s']:.0f} s at launch + 5 s",
                    t_end=150.0, seed=22, wind_mps=float(a["wind_mps_default"]), wind_dir_deg=0.0,
                    gust_mps=float(a["gust_mps_default"]), gust_t0_after_launch_s=5.0,
                    gust_duration_s=float(a["gust_duration_s"]), sensors=sensors, timing=timing or FCTiming())


def scenario_s3(veh: Vehicle, sensors: SensorConfig) -> tuple[Scenario, str | None]:
    """250 m competition flight (informational).  The onboard apogee limiter (I4) and geofence (I5) sit below
    the target, so this scenario alone raises them by their baseline margins above the target."""
    m = veh.params["mission"]
    target = 250.0
    limit = target + (float(m["apogee_limit_m"]) - float(m["target_altitude_m"]))
    geofence = target + (float(m["geofence_altitude_m"]) - float(m["target_altitude_m"]))
    conflict = (f"S3 (informational): the 250 m competition target exceeds mission.apogee_limit_m = {m['apogee_limit_m']:.0f} m "
                f"and mission.geofence_altitude_m = {m['geofence_altitude_m']:.0f} m; S3 alone runs with I4 at {limit:.0f} m and "
                f"I5 at {geofence:.0f} m (target + the baseline margins). Flying 250 m needs those params changed and the "
                f"legal approval for flights above 150 m.")
    a = veh.params["aero"]
    sc = Scenario(name="S3", title=f"competition flight to {target:.0f} m (scenario-only I4 {limit:.0f} m, I5 {geofence:.0f} m), "
                                   f"{_link_desc(veh)}, wind {a['wind_mps_default']:.0f} m/s",
                  t_end=180.0, seed=33, wind_mps=float(a["wind_mps_default"]), target_altitude_m=target,
                  geofence_altitude_m=geofence, apogee_limit_m=limit, sensors=sensors, notes=conflict)
    return sc, conflict


def scenario_s6(veh: Vehicle, sensors: SensorConfig) -> Scenario:
    return replace(scenario_s2(veh, sensors), name="S6", seed=66, t_end=90.0,
                   link_outage_after_launch_s=S6_OUTAGE_AFTER_LAUNCH_S,
                   title=f"total link loss {S6_OUTAGE_AFTER_LAUNCH_S:.0f} s after launch ({_link_desc(veh)} before it); "
                         f"expect I1 ABORT -> chute")


def scenario_s7(veh: Vehicle, sensors: SensorConfig) -> Scenario:
    return replace(scenario_s2(veh, sensors), name="S7", assist=False,
                   title=f"assist off, the operator never cuts or releases ({_link_desc(veh)}); expect I4 cut at the "
                         f"limiter and I6 release")


def scenario_s8(veh: Vehicle, sensors: SensorConfig, seed: int = 88) -> Scenario:
    return replace(scenario_s2(veh, sensors), name="S8" if seed == 88 else f"S8_{seed}", seed=seed,
                   packet_loss=S8_PACKET_LOSS,
                   title=f"ground-assisted flight with {S8_PACKET_LOSS * 100:.0f} % packet loss; no unsafe outcome "
                         f"(ABORT allowed)")


def scenario_s2_sx127x(veh: Vehicle, sensors: SensorConfig) -> Scenario:
    return replace(scenario_s2(veh, sensors), name="S2s", radio="sx127x",
                   title=f"S2 on the sx127x backend ({_link_desc(veh, radio='sx127x')}) - informational")


def scenario_s2_truth(veh: Vehicle, sensors: SensorConfig) -> Scenario:
    """Informational: S2 on the truth + 0.5 deg estimator (the previous BNO085-class idealisation)."""
    sens = replace(sensors, attitude_estimator="truth_noise")
    return replace(scenario_s2(veh, sens), name="S2t",
                   title=f"S2 on the truth + {math.degrees(sens.att_noise_rad):.1f} deg estimator (reference, not the firmware)")


def scenario_s2_bias_ramp(veh: Vehicle, sensors: SensorConfig, t_launch_to_apogee_s: float) -> Scenario:
    """Informational worst case: the gyro bias is perfect on the pad and then drifts linearly on x and y so
    that it reaches the full ``gyro_bias_max`` at the nominal apogee time (no random component)."""
    r = sensors.gyro_bias_max_rad_s / t_launch_to_apogee_s
    sens = replace(sensors, gyro_bias_sigma_rad_s=0.0, gyro_bias_ramp_rad_s2=(r, r, 0.0))
    return replace(scenario_s2(veh, sens), name="S2b",
                   title=f"S2 with worst-case gyro-bias drift: +{math.degrees(sensors.gyro_bias_max_rad_s):.1f} deg/s on x and y "
                         f"between launch and apogee ({math.degrees(r):.3f} deg/s per s)")


def scenario_s4(veh: Vehicle, sensors: SensorConfig) -> Scenario:
    stuck = float(veh.params["vane"]["max_deflection_mech_deg"])
    return Scenario(name="S4", title=f"vane 1 servo failure (hard-over to +{stuck:.0f} deg mechanical stop) at launch + 2 s; "
                                     f"expect I3 ABORT -> chute",
                    t_end=60.0, seed=44, vane_failure_index=0, vane_failure_t_after_launch_s=2.0,
                    vane_failure_stuck_deg=stuck, sensors=sensors)


def scenario_s4_no_i3(veh: Vehicle, sensors: SensorConfig) -> Scenario:
    """Informational: S4 with interlock I3 disabled - what the vehicle does without the spin / saturation supervisor."""
    sc = scenario_s4(veh, sensors)
    return replace(sc, name="S4x", options=replace(sc.options, supervisor_enabled=False), t_end=30.0,
                   title=sc.title.split(";")[0] + " with interlock I3 disabled (study: why I3 exists)")


def s5_variants(sensors: SensorConfig, tuned: dict) -> list[tuple[str, str, float, Scenario]]:
    """+-50 % on each tuned xy gain separately, and on all tuned gains together."""
    out = []
    for key in S5_KEYS:
        for f in (0.5, 1.5):
            g = dict(tuned)
            for k in (TUNED_KEYS if key == "all" else (key,)):
                g[k] = tuned[k] * f
            out.append((key, f"{key} x{f:.1f}", f, scenario_s1(sensors, gains=g, seed=55, name=f"S5_{key}_x{f:.1f}",
                                                                title=f"S1 with {key} x {f:.1f}")))
    return out


def timing_variants(veh: Vehicle) -> list[tuple[str, str, FCTiming]]:
    """Baseline + the requested sensitivity rows (PWM phase, 2x frame rate, +-1 loop of delay, the v0.6.2
    one-tick delay) + references."""
    p = veh.params
    dt = 1.0 / float(p["control"]["loop_hz"])
    f0 = float(p["flight_computer"]["servo_frame_hz"])
    base = FCTiming()
    d0 = base.delay_s(dt)
    v = [("nominal", f"baseline: {f0:.0f} Hz frames, {d0 * 1e3:.2f} ms compute delay (v0.6.3 measured max), "
                     f"phase {0.5:.2f} dt", base)]
    for fr in (0.0, 0.1, 0.25, 0.75):
        v.append((f"phase_{fr:.2f}", f"{f0:.0f} Hz, frame edge {fr:.2f} dt after the tick", replace(base, phase_s=fr * dt)))
    v.append(("timer3_half_frame", f"{f0:.0f} Hz, Timer3 (vane 4 + ESC) half a frame after Timer1",
              replace(base, timer3_phase_s=0.5 / f0)))
    v.append(("frame_2x", f"{2 * f0:.0f} Hz frames (servo frame-rate sensitivity)", replace(base, frame_hz=2.0 * f0)))
    v.append(("delay_one_tick", f"{f0:.0f} Hz, compute delay exactly 1 loop ({dt * 1e3:.2f} ms, the v0.6.2 model)",
              replace(base, compute_delay_ticks=1)))
    v.append(("delay_minus_1", f"{f0:.0f} Hz, compute delay - 1 loop ({(d0 - dt) * 1e3:.2f} ms)",
              replace(base, compute_delay_s=d0 - dt)))
    v.append(("delay_plus_1", f"{f0:.0f} Hz, compute delay + 1 loop ({(d0 + dt) * 1e3:.2f} ms)",
              replace(base, compute_delay_s=d0 + dt)))
    v.append(("continuous", f"no frame latch, {d0 * 1e3:.2f} ms compute delay (ideal interface reference)",
              replace(base, frame_hz=0.0)))
    return v


MARGIN_VARIANTS = ("nominal", "frame_2x", "delay_minus_1", "delay_plus_1", "continuous")


# --------------------------------------------------------------------------- #
# tuning
# --------------------------------------------------------------------------- #
COST_DOC = ("settling time [s] + 2 x saturation time [s] + 0.15 x overshoot [deg] + 0.5 x rms tilt (last 1 s) [deg] "
            "+ 0.3 x vane-command jitter (std about the 1 s mean) [deg] + 0.1 x max yaw error [deg]; "
            "+50 if the run diverged (tilt > 30 deg or NaN)")


CONSTRAINT_DOC = (f"margin constraint (added for the 250 Hz / 50 Hz-frame plant): + 2 + 0.5 x (30 - PM) per axis whose linear "
                  f"phase margin PM < {PM_MIN_DEG:.0f} deg, + 10 x (2 - GM) per axis whose linear gain margin GM < {GM_MIN_X:.0f} "
                  f"(sim/loop_margins.py at the tuning timing)")


def s1_cost(m: dict[str, Any], t_end: float) -> float:
    """Scalar tuning cost from S1 metrics - see COST_DOC (unchanged from the 400 Hz tuning)."""
    if m.get("nan_abort") or m["max_tilt_deg"] > 30.0:
        return 50.0 + m["max_tilt_deg"]
    settle = m["settling_time_s"] if m["settling_time_s"] is not None else t_end + 2.0
    return (settle + 2.0 * m["saturation_time_s"] + 0.15 * m["overshoot_deg"] + 0.5 * m["rms_tilt_last_1s_deg"]
            + 0.3 * m["delta_cmd_jitter_last_1s_deg"] + 0.1 * (m["max_yaw_error_deg"] or 0.0))


def margin_penalty(lin: dict[str, dict[str, Any]]) -> float:
    """CONSTRAINT_DOC: soft-but-dominant penalty for linear margins below the targets."""
    pen = 0.0
    for ax_ in ("xy", "z"):
        pm, gm = lin[ax_]["phase_margin_deg"], lin[ax_]["gain_margin_upper_x"]
        if pm is None:
            pen += 20.0
        elif pm < PM_MIN_DEG:
            pen += 2.0 + 0.5 * (PM_MIN_DEG - pm)
        if gm is not None and gm < GM_MIN_X:
            pen += 10.0 * (GM_MIN_X - gm)
    return pen


def tune_gains(veh: Vehicle, base: dict, sensors: SensorConfig, quick: bool = False,
               t_end: float = TUNE_T_END, timing: FCTiming | None = None,
               margin_constraint: bool = True) -> tuple[dict, list[dict[str, Any]]]:
    history: list[dict[str, Any]] = []
    cache: dict[tuple[float, ...], float] = {}
    timing = timing or FCTiming()

    def evaluate(vals: np.ndarray) -> float:
        key = tuple(np.round(vals, 6))
        if key in cache:
            return cache[key]
        g = dict(base)
        for k, v in zip(TUNED_KEYS, vals):
            g[k] = float(v)
        res = _fs(veh, scenario_s1(sensors, gains=g, t_end=t_end, seed=11, timing=timing)).run()
        m = compute_metrics(res)
        lin = linear_margins(veh, g, timing, sensors)
        pen = margin_penalty(lin) if margin_constraint else 0.0
        c = s1_cost(m, t_end) + pen
        cache[key] = c
        history.append({"gains": {k: float(v) for k, v in zip(TUNED_KEYS, vals)}, "cost": c, "penalty": pen,
                        "settling_time_s": m["settling_time_s"], "overshoot_deg": m["overshoot_deg"],
                        "saturation_time_s": m["saturation_time_s"], "max_tilt_deg": m["max_tilt_deg"],
                        "jitter_deg": m["delta_cmd_jitter_last_1s_deg"],
                        "pm_xy": lin["xy"]["phase_margin_deg"], "pm_z": lin["z"]["phase_margin_deg"]})
        return c

    x0 = np.array([base[k] for k in TUNED_KEYS], dtype=float)
    n = len(TUNED_KEYS)
    # stage 1a: coarse grid on the two dominant pitch/yaw gains
    best_x, best_c = x0.copy(), evaluate(x0)
    kp_mults = (0.5, 1.0, 2.0, 3.0) if not quick else (1.0, 2.0)
    att_mults = (0.5, 1.0, 1.5) if not quick else (1.0,)
    for fk in kp_mults:
        for fa in att_mults:
            x = x0.copy()
            x[TUNED_KEYS.index("rate_kp_xy")] *= fk
            x[TUNED_KEYS.index("att_kp_xy")] *= fa
            c = evaluate(x)
            if c < best_c:
                best_x, best_c = x, c
    # stage 1b: coarse grid on the roll (z) loop, which trims the swirl torque
    for fk in ((0.5, 1.0, 2.0, 4.0) if not quick else (1.0, 2.0)):
        for fi in ((1.0, 2.0) if not quick else (1.0,)):
            x = best_x.copy()
            x[TUNED_KEYS.index("rate_kp_z")] *= fk
            x[TUNED_KEYS.index("rate_ki_z")] *= fi
            c = evaluate(x)
            if c < best_c:
                best_x, best_c = x, c
    # stage 2: Nelder-Mead in log space, bounded to [0.2x, 5x] of the start point
    lo, hi = np.log(0.2 * x0), np.log(5.0 * x0)

    def f_log(z: np.ndarray) -> float:
        zc = np.clip(z, lo, hi)
        penalty = 10.0 * float(np.sum((z - zc) ** 2))
        return evaluate(np.exp(zc)) + penalty

    res = minimize(f_log, np.log(best_x), method="Nelder-Mead",
                   options={"maxfev": 25 if quick else 120, "xatol": 0.02, "fatol": 0.01,
                            "initial_simplex": np.log(best_x) + np.vstack([np.zeros(n), 0.35 * np.eye(n)])})
    zbest = np.clip(res.x, lo, hi)
    xb = np.exp(zbest)
    cb = evaluate(xb)
    if cb > best_c:  # never return something worse than the grid optimum
        xb, cb = best_x, best_c
    tuned = dict(base)
    for k, v in zip(TUNED_KEYS, xb):
        tuned[k] = float(v)
    return tuned, history


# --------------------------------------------------------------------------- #
# stability studies
# --------------------------------------------------------------------------- #
def axis_jitter_deg(res: SimResult, last_s: float = 1.0) -> tuple[float, float]:
    """Std of the pitch/yaw (mean of d_x, d_y) and roll (d_z) vane commands about
    their mean over the last ``last_s`` seconds [deg]."""
    L = res.log
    t = L["t"]
    d = L["delta_cmd"][t >= t[-1] - last_s]
    dx, dy, dz = 0.5 * (d[:, 0] - d[:, 2]), 0.5 * (d[:, 1] - d[:, 3]), d.mean(axis=1)
    return float(np.degrees(0.5 * (dx.std() + dy.std()))), float(np.degrees(dz.std()))


def sim_stable(veh: Vehicle, gains: dict, timing: FCTiming, sensors: SensorConfig, axis: str) -> bool:
    res = _fs(veh, scenario_s1(sensors, gains=gains, t_end=TUNE_T_END, seed=11, timing=timing)).run()
    m = compute_metrics(res)
    if res.nan_abort or m["max_tilt_deg"] > 30.0 or m["settling_time_s"] is None:
        return False
    jxy, jz = axis_jitter_deg(res)
    return (jxy if axis == "xy" else jz) < JITTER_UNSTABLE_DEG


def sim_gain_margin(veh: Vehicle, tuned: dict, timing: FCTiming, sensors: SensorConfig, axis: str,
                    iters: int = 6) -> tuple[float | None, str]:
    """Largest factor on the rate gains (kp, ki, kd of the axis) that the 6-DOF
    S1 run survives (geometric bisection in [1, MARGIN_F_MAX])."""
    sfx = "xy" if axis == "xy" else "z"

    def scaled(f: float) -> dict:
        g = dict(tuned)
        for k in (f"rate_kp_{sfx}", f"rate_ki_{sfx}", f"rate_kd_{sfx}"):
            g[k] = tuned[k] * f
        return g

    if not sim_stable(veh, scaled(1.0), timing, sensors, axis):
        return None, "unstable at x1"
    if sim_stable(veh, scaled(MARGIN_F_MAX), timing, sensors, axis):
        return MARGIN_F_MAX, f">= {MARGIN_F_MAX:.0f}"
    lo, hi = 1.0, MARGIN_F_MAX
    for _ in range(iters):
        mid = math.sqrt(lo * hi)
        if sim_stable(veh, scaled(mid), timing, sensors, axis):
            lo = mid
        else:
            hi = mid
    return lo, f"{lo:.2f} (unstable at {hi:.2f})"


def timing_delay_budget(veh: Vehicle, timing: FCTiming) -> dict[str, float]:
    """Equivalent dead time of the actuator path for the linear analysis and the report."""
    p = veh.params
    dt = 1.0 / float(p["control"]["loop_hz"])
    fr = float(p["flight_computer"]["servo_frame_hz"]) if timing.frame_hz is None else float(timing.frame_hz)
    phase = 0.5 * dt if timing.phase_s is None else float(timing.phase_s)
    zoh = fr > 0.0
    comp = timing.delay_s(dt)
    wait = mean_wait_after_available(dt, fr, phase, comp) if zoh else 0.0
    pulse = float(p["servo"]["pwm_center_us"]) * 1e-6 if (zoh and timing.pulse_decode) else 0.0
    hold = 0.5 / fr if zoh else 0.5 * dt
    return {"frame_hz": fr, "compute_s": comp, "wait_s": wait, "pulse_s": pulse, "hold_s": hold,
            "dead_time_s": comp + wait + pulse, "total_equiv_s": comp + wait + pulse + hold}


def linear_margins(veh: Vehicle, g: dict, timing: FCTiming, sensors: SensorConfig,
                   servo_tau_s: float | None = None) -> dict[str, dict[str, Any]]:
    p = veh.params
    dt = 1.0 / float(p["control"]["loop_hz"])
    b = timing_delay_budget(veh, timing)
    I = veh.full.inertia_kg_m2
    tau_s = float(p["servo"]["time_constant_s"]) if servo_tau_s is None else float(servo_tau_s)
    out = {}
    for axis, inertia, km in (("xy", float(I[0, 0]), 1.0), ("z", float(I[2, 2]), float(g["roll_priority_scale"]))):
        out[axis] = axis_margins(g, axis, dt, inertia, km, tau_s, sensors.imu_dlpf_tau_s, b["dead_time_s"], b["frame_hz"])
    return out


# --------------------------------------------------------------------------- #
# plots
# --------------------------------------------------------------------------- #
def _style_axes(ax: plt.Axes, title: str, ylabel: str) -> None:
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=10, color=INK, fontweight="bold")
    ax.set_ylabel(ylabel, color=INK2, fontsize=9)
    ax.grid(True, color=GRID, linewidth=0.6)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(MUTED)
    ax.tick_params(colors=INK2, labelsize=8)


def _shade_states(ax: plt.Axes, t: np.ndarray, active: np.ndarray) -> None:
    """Light bands where attitude control is active (POWERED + the coast hold)."""
    active = np.asarray(active, dtype=bool)
    if not active.any():
        return
    edges = np.flatnonzero(np.diff(active.astype(int)) != 0) + 1
    starts = [0] + edges.tolist() if active[0] else edges.tolist()
    for s in starts[::1]:
        if not active[min(s, len(active) - 1)]:
            continue
        e_candidates = [e for e in edges if e > s]
        e = e_candidates[0] if e_candidates else len(t) - 1
        ax.axvspan(t[s], t[e], color=PALETTE[0], alpha=0.06, lw=0)


def plot_scenario(res: SimResult, m: dict[str, Any], veh: Vehicle, path: str) -> None:
    L = res.log
    t = L["t"]
    sc = res.scenario
    fig, axes = plt.subplots(4, 2, figsize=(13, 13), facecolor=SURFACE)
    fig.suptitle(f"{sc.name}: {sc.title}\nmass source: {res.mass_source}; estimator {sc.sensors.attitude_estimator}",
                 fontsize=11, color=INK, x=0.01, ha="left")
    ax = axes[0, 0]
    _style_axes(ax, "Attitude", "deg")
    ax.plot(t, L["euler_deg"][:, 0], color=PALETTE[0], lw=1.6, label="roll (about x_b)")
    ax.plot(t, L["euler_deg"][:, 1], color=PALETTE[1], lw=1.6, label="pitch (about y_b)")
    ax.plot(t, L["tilt_est_deg"], color=PALETTE[2], lw=0.8, alpha=0.8, label="tilt estimate")
    ax.plot(t, L["tilt_est_err_deg"], color=PALETTE[3], lw=1.2, label="tilt-estimate error")
    ax.plot(t, L["tilt_deg"], color=INK, lw=1.2, ls="--", label="tilt (z_b vs up)")
    ax.legend(fontsize=8, frameon=False, ncol=2)
    ax = axes[0, 1]
    _style_axes(ax, "Body rates (truth)", "deg/s")
    for i, lab in enumerate(("p (x_b)", "q (y_b)", "r (z_b)")):
        ax.plot(t, np.degrees(L["omega"][:, i]), color=PALETTE[i], lw=1.4, label=lab)
    ax.legend(fontsize=8, frameon=False, ncol=3)
    ax = axes[1, 0]
    _style_axes(ax, "Vane deflections (actual, solid) and controller commands (dotted)", "deg")
    dmax = float(veh.params["vane"]["max_deflection_cmd_deg"])
    for i in range(4):
        ax.plot(t, np.degrees(L["delta"][:, i]), color=PALETTE[i], lw=1.4, label=f"vane {i + 1}")
        ax.plot(t, np.degrees(L["delta_cmd"][:, i]), color=PALETTE[i], lw=0.7, ls=":", alpha=0.7)
    ax.axhline(dmax, color=MUTED, lw=0.8, ls="--")
    ax.axhline(-dmax, color=MUTED, lw=0.8, ls="--")
    ax.legend(fontsize=8, frameon=False, ncol=4)
    ax = axes[1, 1]
    _style_axes(ax, "Altitude (CG above pad)", "m")
    h = L["pos"][:, 2] - L["pos"][0, 2]
    ax.plot(t, h, color=PALETTE[0], lw=1.8, label="truth")
    ax.plot(t, L["h_est"], color=PALETTE[1], lw=1.0, label="estimate (baro + accel)")
    hb = L["h_baro"]
    ok = np.isfinite(hb)
    ax.plot(t[ok], hb[ok], ".", color=PALETTE[2], ms=2.5, label=f"baro samples ({sc.sensors.baro_latency_s:.2f} s late)")
    tgt = sc.target_altitude_m or veh.params["mission"]["target_altitude_m"]
    if not sc.bypass_state_machine:
        lim = sc.apogee_limit_m or float(veh.params["mission"]["apogee_limit_m"])
        ax.axhline(tgt, color=MUTED, lw=0.8, ls="--")
        ax.text(t[-1], tgt, f" target {tgt:.0f} m", fontsize=7, color=INK2, va="top", ha="right")
        ax.axhline(lim, color=PALETTE[7], lw=0.8, ls=":")
        ax.text(t[-1], lim, f" apogee limit (I4) {lim:.0f} m", fontsize=7, color=INK2, va="bottom", ha="right")
        for key, lab, col in (("t_cutoff_s", "cut", PALETTE[3]), ("t_deploy_s", "release", PALETTE[4])):
            if m.get(key) is not None:
                ax.axvline(m[key], color=col, lw=0.9, ls="--", label=f"{lab} ({m.get('cut_by' if lab == 'cut' else 'release_by') or 'ABORT'})")
    ax.legend(fontsize=8, frameon=False)
    ax = axes[2, 0]
    _style_axes(ax, "Velocity", "m/s")
    ax.plot(t, L["vel"][:, 2], color=PALETTE[0], lw=1.6, label="v_z truth")
    ax.plot(t, L["vz_est"], color=PALETTE[1], lw=1.0, label="v_z estimate")
    ax.plot(t, np.hypot(L["vel"][:, 0], L["vel"][:, 1]), color=PALETTE[2], lw=1.2, label="horizontal speed")
    ax.legend(fontsize=8, frameon=False, ncol=3)
    ax = axes[2, 1]
    _style_axes(ax, "Throttle and thrust (fraction of T_max)", "-")
    ax.plot(t, L["throttle"], color=PALETTE[0], lw=1.6, label="throttle u (commanded)")
    ax.plot(t, L["thrust"] / veh.t_max_n, color=PALETTE[1], lw=1.4, label="thrust / T_max (lagged)")
    ax.axhline(veh.weight_n / veh.t_max_n, color=MUTED, lw=0.8, ls="--")
    ax.text(t[0], veh.weight_n / veh.t_max_n, " weight / T_max", fontsize=7, color=INK2, va="bottom")
    ax.set_ylim(-0.02, 1.05)
    ax.legend(fontsize=8, frameon=False, ncol=2)
    ax = axes[3, 0]
    _style_axes(ax, "Horizontal position (drift from the pad)", "m")
    ax.plot(t, L["pos"][:, 0] - L["pos"][0, 0], color=PALETTE[0], lw=1.6, label="x (toward vane 1)")
    ax.plot(t, L["pos"][:, 1] - L["pos"][0, 1], color=PALETTE[1], lw=1.6, label="y")
    if sc.wind_mps:
        ax.plot(t, L["wind"][:, 0], color=PALETTE[2], lw=0.9, ls="--", label="wind x [m/s]")
    ax.legend(fontsize=8, frameon=False, ncol=3)
    ax.set_xlabel("t [s]", color=INK2, fontsize=9)
    ax = axes[3, 1]
    _style_axes(ax, "Supervisor state (SPEC 8.8), ground frames, saturation", "")
    ax.step(t, L["state"], where="post", color=PALETTE[0], lw=1.6, label="state")
    n_top = max(int(s) for s in State)
    rx = np.flatnonzero(L["frames_rx"] > 0)
    if rx.size:
        ax.plot(t[rx], np.full(rx.size, n_top + 0.3), "|", color=PALETTE[2], ms=6, label="ground frame received")
    ax.set_yticks([int(s) for s in State])
    ax.set_yticklabels([s.name for s in State], fontsize=7)
    sat = L["saturated"]
    if sat.any():
        ax.fill_between(t, -0.5, n_top + 0.5, where=sat, color=PALETTE[7], alpha=0.25, lw=0, label="vane saturated")
    ax.legend(fontsize=8, frameon=False, loc="center right")
    ax.set_ylim(-0.5, n_top + 0.6)
    ax.set_xlabel("t [s]", color=INK2, fontsize=9)
    for ax in axes.flat:
        _shade_states(ax, t, L["control_active"])
        if m.get("fault_t_s") is not None:
            ax.axvline(m["fault_t_s"], color=PALETTE[7], lw=0.9, ls="--")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_s5(results: list[tuple[str, str, float, SimResult, dict]], tuned_res: SimResult, path: str) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(16, 8), facecolor=SURFACE)
    fig.suptitle("S5: gain sensitivity +-50 % around the tuned set (S1 case, tilt vs time)", fontsize=11, color=INK,
                 x=0.01, ha="left")
    axes.flat[-1].set_visible(False)
    for ax, key in zip(axes.flat, S5_KEYS):
        _style_axes(ax, key if key != "all" else "all tuned gains together", "tilt [deg]")
        ax.plot(tuned_res.log["t"], tuned_res.log["tilt_deg"], color=INK, lw=1.6, label="x1.0 (tuned)")
        for (k, label, f, res, m) in results:
            if k != key:
                continue
            col = PALETTE[0] if f < 1 else PALETTE[1]
            ax.plot(res.log["t"], res.log["tilt_deg"], color=col, lw=1.4,
                    label=f"x{f:.1f}: settle {m['settling_time_s'] if m['settling_time_s'] is not None else float('nan'):.2f} s, "
                          f"max delta {m['max_delta_deg']:.1f} deg")
        ax.axhline(1.0, color=MUTED, lw=0.8, ls="--")
        ax.set_xlabel("t [s]", color=INK2, fontsize=9)
        ax.legend(fontsize=8, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_tuning(history: list[dict[str, Any]], path: str) -> None:
    fig, ax = plt.subplots(figsize=(9, 4), facecolor=SURFACE)
    _style_axes(ax, "Gain tuning: cost per evaluation (grid, then Nelder-Mead)", "cost [s-equivalent]")
    c = np.array([h["cost"] for h in history])
    ax.plot(np.arange(len(c)), c, color=PALETTE[0], lw=1.2, marker="o", ms=3, label="evaluation")
    ax.plot(np.arange(len(c)), np.minimum.accumulate(c), color=PALETTE[1], lw=1.6, label="best so far")
    ax.set_xlabel("evaluation", color=INK2, fontsize=9)
    ax.set_yscale("log")
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_timing(trows: dict[str, dict[str, Any]], path: str, f0: float) -> None:
    """(a, b) vane-1 command chain at the start of S1 for the baseline and the 2x frame rate,
    (c) S1 tilt for the timing variants, (d) linear phase margins."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), facecolor=SURFACE)
    fig.suptitle("Flight-computer timing: 250 Hz control, PWM-frame zero-order hold, compute delay",
                 fontsize=11, color=INK, x=0.01, ha="left")
    for ax, key in ((axes[0, 0], "nominal"), (axes[0, 1], "frame_2x")):
        res = trows[key]["s1_result"]
        L = res.log
        t = L["t"]
        sel = t <= 0.30
        _style_axes(ax, f"Vane 1 command chain, S1 start - {trows[key]['label']}", "deg")
        ax.step(t[sel], np.degrees(L["delta_cmd"][sel, 0]), where="post", color=PALETTE[0], lw=1.2,
                label="controller output (250 Hz)")
        ax.step(t[sel], np.degrees(L["delta_srv"][sel, 0]), where="post", color=PALETTE[1], lw=1.6,
                label="servo-side command (frame latch + pulse decode)")
        ax.plot(t[sel], np.degrees(L["delta"][sel, 0]), color=PALETTE[2], lw=1.8, label="vane deflection")
        ax.set_xlabel("t [s]", color=INK2, fontsize=9)
        ax.legend(fontsize=8, frameon=False, loc="lower right")
    ax = axes[1, 0]
    _style_axes(ax, "S1 tilt for the timing variants", "tilt [deg]")
    for i, key in enumerate(MARGIN_VARIANTS):
        if key not in trows:
            continue
        L = trows[key]["s1_result"].log
        ax.plot(L["t"], L["tilt_deg"], color=PALETTE[i], lw=1.5, label=trows[key]["label"])
    ax.axhline(1.0, color=MUTED, lw=0.8, ls="--")
    ax.set_xlabel("t [s]", color=INK2, fontsize=9)
    ax.legend(fontsize=8, frameon=False)
    ax = axes[1, 1]
    _style_axes(ax, "Linear phase margin (hover, small signal)", "phase margin [deg]")
    keys = [k for k in MARGIN_VARIANTS if k in trows]
    xs = np.arange(len(keys))
    pm_xy = [trows[k]["lin"]["xy"]["phase_margin_deg"] or 0.0 for k in keys]
    pm_z = [trows[k]["lin"]["z"]["phase_margin_deg"] or 0.0 for k in keys]
    ax.bar(xs - 0.2, pm_xy, width=0.38, color=PALETTE[0], label="pitch/yaw (x, y)", edgecolor=SURFACE, linewidth=2)
    ax.bar(xs + 0.2, pm_z, width=0.38, color=PALETTE[1], label="roll (z)", edgecolor=SURFACE, linewidth=2)
    for x_, a, b in zip(xs, pm_xy, pm_z):
        ax.text(x_ - 0.2, a, f"{a:.0f}", ha="center", va="bottom", fontsize=8, color=INK2)
        ax.text(x_ + 0.2, b, f"{b:.0f}", ha="center", va="bottom", fontsize=8, color=INK2)
    ax.axhline(PM_MIN_DEG, color=MUTED, lw=0.8, ls="--", label=f"{PM_MIN_DEG:.0f} deg target")
    ax.set_xticks(xs)
    ax.set_xticklabels([k.replace("_", " ") for k in keys], fontsize=8)
    ax.legend(fontsize=8, frameon=False, loc="upper left")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_estimator(mc: list[tuple[int, SimResult, dict]], s2: SimResult, s2b: SimResult | None, path: str) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8), facecolor=SURFACE, gridspec_kw={"width_ratios": [2.2, 1]})
    fig.suptitle("Gyro-only attitude estimate after launch: tilt-estimate error (MPU-6050 bias drift)",
                 fontsize=11, color=INK, x=0.01, ha="left")
    ax = axes[0]
    _style_axes(ax, "Tilt-estimate error vs time since launch", "deg")
    for i, (seed, res, m) in enumerate(mc):
        L = res.log
        tl = L["t"] - res.scenario.launch_time_s
        ax.plot(tl, L["tilt_est_err_deg"], color=PALETTE[0], lw=0.8, alpha=0.35,
                label=f"S2 Monte Carlo ({len(mc)} seeds)" if i == 0 else None)
    L = s2.log
    ax.plot(L["t"] - s2.scenario.launch_time_s, L["tilt_est_err_deg"], color=INK, lw=1.4, label="S2 (seed 22)")
    if s2b is not None:
        L = s2b.log
        ax.plot(L["t"] - s2b.scenario.launch_time_s, L["tilt_est_err_deg"], color=PALETTE[1], lw=1.8,
                label="S2b worst-case bias drift")
    t_ap = [m["t_apogee_s"] - res.scenario.launch_time_s for (_, res, m) in mc]
    x_hi = max(t_ap) + 3.0
    y_hi = 0.0                                            # y range from the plotted window only (not the descent)
    for r_ in [res for (_, res, _) in mc] + [s2] + ([s2b] if s2b is not None else []):
        tl = r_.log["t"] - r_.scenario.launch_time_s
        sel = (tl >= -1.0) & (tl <= x_hi)
        y_hi = max(y_hi, float(r_.log["tilt_est_err_deg"][sel].max()))
    ax.set_xlim(-1.0, x_hi)
    ax.set_ylim(0.0, 1.15 * y_hi)
    ax.axvline(float(np.median(t_ap)), color=MUTED, lw=0.8, ls="--")
    ax.text(float(np.median(t_ap)), 1.15 * y_hi, " apogee (median)", fontsize=7, color=INK2, va="top")
    ax.set_xlabel("t - t_launch [s]", color=INK2, fontsize=9)
    ax.legend(fontsize=8, frameon=False, loc="upper left")
    ax = axes[1]
    _style_axes(ax, "Error at apogee per seed", "deg")
    vals = np.array([m["tilt_est_err_at_apogee_deg"] for (_, _, m) in mc])
    jitter = np.linspace(-0.15, 0.15, len(vals))
    ax.plot(jitter, vals, "o", color=PALETTE[0], ms=8, mec=SURFACE, mew=2, label="Monte Carlo seed")
    p95 = float(np.percentile(vals, 95))
    ax.axhline(p95, color=MUTED, lw=0.8, ls="--")
    ax.text(0.3, p95, f"p95 {p95:.2f}", fontsize=8, color=INK2, va="bottom", ha="right")
    if s2b is not None:
        mb = compute_metrics(s2b)
        ax.plot([0.0], [mb["tilt_est_err_at_apogee_deg"]], "D", color=PALETTE[1], ms=8, mec=SURFACE, mew=2,
                label="S2b worst case")
    ax.set_xlim(-0.4, 0.4)
    ax.set_xticks([])
    ax.set_ylim(bottom=0.0)
    ax.legend(fontsize=8, frameon=False, loc="upper left")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(path, dpi=110)
    plt.close(fig)


# --------------------------------------------------------------------------- #
# verdicts and report
# --------------------------------------------------------------------------- #
def _fmt(v: Any, nd: int = 2) -> str:
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, (float, np.floating)):
        return f"{float(v):.{nd}f}"
    return str(v)


def verdict_s1(m: dict[str, Any], dmax: float) -> tuple[str, str]:
    ok_settle = m["settling_time_s"] is not None and m["settling_time_s"] < 2.0
    ok_delta = m["max_delta_deg"] < dmax - 1e-6
    ok_os = m["overshoot_deg"] < 5.0
    v = "PASS" if (ok_settle and ok_delta and ok_os) else "FAIL"
    return v, (f"control tilt settles to < 1 deg in {_fmt(m['settling_time_s'])} s (< 2 s: {ok_settle}; true tilt "
               f"{_fmt(m.get('settling_time_true_s'))} s, rms true tilt over the last 1 s {_fmt(m.get('rms_true_tilt_last_1s_deg'))} deg), "
               f"max |delta| {m['max_delta_deg']:.1f} deg (< {dmax:.0f}: {ok_delta}), overshoot {m['overshoot_deg']:.2f} deg (< 5: {ok_os})")


def _ok_touchdown(m: dict[str, Any]) -> tuple[bool, str]:
    td = m.get("touchdown_speed_mps")
    ok = td is not None and td < TOUCHDOWN_MAX_MPS
    return ok, f"touchdown {_fmt(td, 1)} m/s (< {TOUCHDOWN_MAX_MPS:.0f}: {ok})"


def _ok_apogee_limit(m: dict[str, Any], limit: float) -> tuple[bool, str]:
    ok = m["apogee_m"] <= limit
    return ok, f"apogee {m['apogee_m']:.1f} m (<= limit {limit:.0f}: {ok})"


def verdict_s2(m: dict[str, Any], p: dict, target: float | None = None, limit: float | None = None,
               touchdown: bool = True) -> tuple[str, str]:
    """S2 criteria: apogee <= apogee_limit_m and >= 0.8 x target, release (servo open) while |v_z| < 5 m/s,
    no ABORT, touchdown < 7 m/s.  ``touchdown=False`` drops the last term (and says so in the text) for runs that
    stop before touchdown - the S2 Monte Carlo seeds end a few seconds after the release, so with the term they
    could never PASS."""
    ms = p["mission"]
    target = float(ms["target_altitude_m"]) if target is None else target
    limit = float(ms["apogee_limit_m"]) if limit is None else limit
    ok_hi, s_hi = _ok_apogee_limit(m, limit)
    ok_lo = m["apogee_m"] >= APOGEE_MIN_FRACTION * target
    vz = m.get("vz_release_mps")
    ok_rel = vz is not None and abs(vz) < RELEASE_VZ_MAX_MPS
    ok_ab = m["t_abort_s"] is None
    ok_td, s_td = _ok_touchdown(m) if touchdown else (True, "touchdown not judged (run ends before touchdown)")
    v = "PASS" if (ok_hi and ok_lo and ok_rel and ok_ab and ok_td) else "FAIL"
    return v, (f"{s_hi}, >= {APOGEE_MIN_FRACTION:.1f} x target = {APOGEE_MIN_FRACTION * target:.0f} m: {ok_lo}; cut by "
               f"{m.get('cut_by')} at h {_fmt(m.get('h_cutoff_m'), 1)} m; release by {m.get('release_by') or '-'} at "
               f"{_fmt(m.get('t_deploy_s'))} s ({_fmt(m.get('release_minus_apogee_s'))} s after apogee) with v_z "
               f"{_fmt(vz, 1)} m/s (|v_z| < {RELEASE_VZ_MAX_MPS:.0f}: {ok_rel}); no ABORT: {ok_ab}"
               f"{'' if ok_ab else ' (' + str(m['abort_reason']) + ')'}; {s_td}")


def attitude_ok(m: dict[str, Any], tilt_abort: float) -> bool:
    """The attitude-control part of a flight (used for the timing rows): tilt held and no I2 / I3 abort."""
    reason = m.get("abort_reason") or ""
    return m["max_tilt_deg"] < tilt_abort and not reason.startswith(("I2", "I3"))


def verdict_s4(m: dict[str, Any]) -> tuple[str, str]:
    ok_abort = (m["t_abort_s"] is not None and m["fault_t_s"] is not None
                and m["t_abort_s"] - m["fault_t_s"] < FAULT_TO_ABORT_MAX_S)
    ok_i3 = (m.get("abort_reason") or "").startswith("I3")
    ok_dep = m["t_deploy_s"] is not None
    ok_td, s_td = _ok_touchdown(m)
    v = "PASS" if (ok_abort and ok_dep) else "FAIL"
    return v, (f"fault at {_fmt(m['fault_t_s'])} s, ABORT at {_fmt(m['t_abort_s'])} s "
               f"({_fmt((m['t_abort_s'] - m['fault_t_s']) if ok_abort else None)} s after the fault, < {FAULT_TO_ABORT_MAX_S:.0f} s: "
               f"{ok_abort}; by I3: {ok_i3}; reason: {m['abort_reason']}); release at {_fmt(m['t_deploy_s'])} s "
               f"(h = {_fmt(m.get('h_deploy_m'))} m): {ok_dep}; {s_td} (reported)")


def verdict_s6(m: dict[str, Any]) -> tuple[str, str]:
    ok_i1 = (m.get("abort_reason") or "").startswith("I1")
    ok_dep = m["t_deploy_s"] is not None
    ok_td, s_td = _ok_touchdown(m)
    v = "PASS" if (ok_i1 and ok_dep and ok_td) else "FAIL"
    return v, (f"link lost at {_fmt(m.get('fault_t_s'))} s; ABORT at {_fmt(m['t_abort_s'])} s by I1: {ok_i1} "
               f"({m['abort_reason']}); release at {_fmt(m['t_deploy_s'])} s (h {_fmt(m.get('h_deploy_m'), 1)} m, "
               f"v_z {_fmt(m.get('vz_release_mps'), 1)} m/s): {ok_dep}; {s_td}")


def verdict_s7(m: dict[str, Any], p: dict) -> tuple[str, str]:
    ok_hi, s_hi = _ok_apogee_limit(m, float(p["mission"]["apogee_limit_m"]))
    ok_td, s_td = _ok_touchdown(m)
    ok_i4 = (m.get("cut_by") or "").startswith("I4")
    ok_i6 = (m.get("release_by") or "").startswith("I6")
    v = "PASS" if (ok_hi and ok_td and ok_i4 and ok_i6) else "FAIL"
    return v, (f"cut by {m.get('cut_by')} (I4 expected: {ok_i4}) at h {_fmt(m.get('h_cutoff_m'), 1)} m, v_z "
               f"{_fmt(m.get('vz_cutoff_mps'), 1)} m/s; {s_hi}; release by {m.get('release_by')} (I6 expected: {ok_i6}) "
               f"{_fmt(m.get('release_minus_apogee_s'))} s after apogee at v_z {_fmt(m.get('vz_release_mps'), 1)} m/s; {s_td}")


def verdict_s8(m: dict[str, Any], p: dict) -> tuple[str, str]:
    flew = m.get("t_liftoff_s") is not None
    ok_hi, s_hi = _ok_apogee_limit(m, float(p["mission"]["apogee_limit_m"]))
    ok_td, s_td = _ok_touchdown(m)
    v = "PASS" if (flew and ok_hi and ok_td) else "FAIL"
    if not flew:
        return v, "the vehicle never launched (operator retries exhausted) - no flight to judge"
    return v, (f"{s_hi}; {s_td}; ABORT {'none' if m['t_abort_s'] is None else str(m['abort_reason'])} (allowed); "
               f"cut by {m.get('cut_by')}, release {_fmt(m.get('release_minus_apogee_s'))} s after apogee at v_z "
               f"{_fmt(m.get('vz_release_mps'), 1)} m/s by {m.get('release_by') or 'ABORT'}; polls {m.get('polls')}, "
               f"timeouts {m.get('timeouts')}")


def verdict_s5(rows: list[tuple[str, str, float, SimResult, dict]]) -> tuple[str, str]:
    bad = [label for (_, label, _, res, m) in rows
           if m["settling_time_s"] is None or m["settling_time_s"] > 3.0 or m["max_tilt_deg"] > 30.0 or res.nan_abort]
    v = "PASS" if not bad else "FAIL"
    return v, (f"all {len(rows)} variants settle within 3 s and never exceed 30 deg" if not bad else
               "variants failing (settle > 3 s or tilt > 30 deg): " + ", ".join(bad))


METRIC_ROWS: list[tuple[str, str, str, int]] = [
    ("max tilt (true, control active)", "max_tilt_deg", "deg", 2),
    ("settling time to < 1 deg (control tilt)", "settling_time_s", "s", 2),
    ("settling time to < 1 deg (true tilt)", "settling_time_true_s", "s", 2),
    ("overshoot (control tilt)", "overshoot_deg", "deg", 2),
    ("rms true tilt, last 1 s of control", "rms_true_tilt_last_1s_deg", "deg", 2),
    ("apogee (CG rise)", "apogee_m", "m", 1),
    ("time of apogee", "t_apogee_s", "s", 1),
    ("lateral drift at apogee", "lateral_drift_at_apogee_m", "m", 1),
    ("lateral drift max", "lateral_drift_max_m", "m", 1),
    ("max vane deflection (actual)", "max_delta_deg", "deg", 2),
    ("max vane command", "max_delta_cmd_deg", "deg", 2),
    ("time in vane saturation", "saturation_time_s", "s", 2),
    ("vane-command fluctuation, last 1 s of control (flights: includes the post-cut-off hold)",
     "delta_cmd_jitter_last_1s_deg", "deg", 3),
    ("max tilt-estimate error (control active)", "max_tilt_est_err_deg", "deg", 2),
    ("tilt-estimate error at apogee", "tilt_est_err_at_apogee_deg", "deg", 2),
    ("true tilt at apogee", "tilt_at_apogee_deg", "deg", 1),
    ("energy used", "energy_Wh", "Wh", 2),
    ("max body rate", "max_rate_deg_s", "deg/s", 1),
    ("max yaw error", "max_yaw_error_deg", "deg", 2),
    ("max v_z", "max_vz_mps", "m/s", 1),
    ("altitude estimate error (max)", "max_altitude_error_est_m", "m", 2),
    ("LAUNCH executed onboard", "t_launch_s", "s", 2),
    ("liftoff", "t_liftoff_s", "s", 2),
    ("motor cut-off", "t_cutoff_s", "s", 2),
    ("cut by", "cut_by", "", 0),
    ("altitude at cut-off", "h_cutoff_m", "m", 1),
    ("v_z at cut-off", "vz_cutoff_mps", "m/s", 1),
    ("abort", "t_abort_s", "s", 2),
    ("release decided onboard (command / I6)", "t_release_cmd_s", "s", 2),
    ("release by", "release_by", "", 0),
    ("release servo open", "t_deploy_s", "s", 2),
    ("release - apogee", "release_minus_apogee_s", "s", 2),
    ("altitude at release", "h_deploy_m", "m", 1),
    ("v_z at release", "vz_release_mps", "m/s", 1),
    ("touchdown", "t_touchdown_s", "s", 1),
    ("touchdown speed (total)", "touchdown_speed_mps", "m/s", 2),
    ("touchdown v_z", "touchdown_vz_mps", "m/s", 2),
    ("peak yaw rate after cut-off", "max_yaw_rate_after_cutoff_deg_s", "deg/s", 1),
    ("final state", "final_state", "", 0),
    ("wall-clock runtime", "runtime_s", "s", 1),
]


def _st(m: dict[str, Any], key: str, field_: str, scale: float = 1.0, nd: int = 2) -> str:
    s = m.get(key) or {}
    v = s.get(field_)
    return "-" if v is None else f"{v * scale:.{nd}f}"


def link_table(ms: list[dict[str, Any]]) -> str:
    """Ground-link and command-latency metrics of the flight scenarios."""
    rows = ["| scenario | radio / loss | polls | replies | timeouts | up lost / down lost / collisions | RTT mean / max [s] "
            "| commands (sent x, result) | decision -> exec mean / max [s] | CUT decision -> exec [s] | "
            "RELEASE decision -> exec [s] |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for m in ms:
        if "link_radio" not in m:
            continue
        ls = m["link_stats"]
        cmds = ", ".join(f"{c['cmd']} {c['n_tx']}x {c['result']}"
                         for c in m["commands"])
        rows.append(f"| {m['scenario']} | {m['link_radio']} / {m['link_loss'] * 100:.0f} % | {m['polls']} | {m['replies']} | "
                    f"{m['timeouts']} | {ls['up_lost']} / {ls['down_lost']} / {ls['collision']} | "
                    f"{_st(m, 'rtt', 'mean', nd=3)} / {_st(m, 'rtt', 'max', nd=3)} | {cmds} | "
                    f"{_st(m, 'cmd_decision_to_exec', 'mean', nd=3)} / {_st(m, 'cmd_decision_to_exec', 'max', nd=3)} | "
                    f"{_fmt(m.get('cut_decision_to_exec_s'), 3)} | {_fmt(m.get('release_decision_to_exec_s'), 3)} |")
    return "\n".join(rows)


def metric_table(ms: list[dict[str, Any]]) -> str:
    head = "| metric | unit | " + " | ".join(m["scenario"] for m in ms) + " |"
    sep = "|---|---|" + "---|" * len(ms)
    rows = [head, sep]
    for label, key, unit, nd in METRIC_ROWS:
        rows.append(f"| {label} | {unit} | " + " | ".join(_fmt(m.get(key), nd) for m in ms) + " |")
    return "\n".join(rows)


def servo_assessment(trows: dict[str, dict[str, Any]], runs: dict[str, Any], retuned_2x: dict[str, Any] | None,
                     whatif: dict[str, Any] | None) -> tuple[str, list[str]]:
    """Adequate when, at the 50 Hz baseline with the tuned gains, both axes have a linear phase margin
    >= PM_MIN_DEG and a 6-DOF gain margin >= GM_MIN_X, and S1-S3 pass.  The 2x-frame rows (same gains,
    and gains retuned for 2x frames) quantify what a faster servo would buy."""
    nom, fast = trows["nominal"], trows.get("frame_2x")
    f0 = nom["budget"]["frame_hz"]
    lines = []
    ok = True
    for ax_ in ("xy", "z"):
        pm = nom["lin"][ax_]["phase_margin_deg"]
        gm = nom["gm_sim"].get(ax_, (None, ""))[0]
        ok_ax = pm is not None and pm >= PM_MIN_DEG and gm is not None and gm >= GM_MIN_X
        ok = ok and ok_ax
        line = (f"{'pitch/yaw' if ax_ == 'xy' else 'roll'} loop at {f0:.0f} Hz: crossover "
                f"{_fmt(nom['lin'][ax_]['crossover_hz'])} Hz, phase margin {_fmt(pm, 1)} deg (>= {PM_MIN_DEG:.0f}: "
                f"{pm is not None and pm >= PM_MIN_DEG}), 6-DOF gain margin {nom['gm_sim'].get(ax_, (None, '-'))[1]} "
                f"(>= {GM_MIN_X:.0f}: {gm is not None and gm >= GM_MIN_X})")
        if fast is not None:
            line += (f"; same gains at {fast['budget']['frame_hz']:.0f} Hz: phase margin {_fmt(fast['lin'][ax_]['phase_margin_deg'], 1)} deg, "
                     f"6-DOF gain margin {fast['gm_sim'].get(ax_, (None, '-'))[1]}")
        lines.append(line)
    nominal_pass = (runs["S1"]["verdict"][0] == "PASS"
                    and all(runs[k].get("attitude_ok", False) for k in ("S2", "S3") if k in runs))
    ok = ok and nominal_pass
    lines.append(f"S1 PASS and attitude held in S2 / S3 (max tilt < tilt_abort_deg, no I2 / I3 abort) at the baseline timing: "
                 f"{nominal_pass} (the S2 mission verdict depends on the link and the cut / release rules, not on the servo; "
                 f"S4 depends on the gyro range)")
    if fast is not None:
        a, b = nom["s1"], fast["s1"]
        c, d = nom["s2"], fast["s2"]
        lines.append(f"same gains, {f0:.0f} -> {fast['budget']['frame_hz']:.0f} Hz frames: S1 settling "
                     f"{_fmt(a['settling_time_s'])} -> {_fmt(b['settling_time_s'])} s, overshoot {_fmt(a['overshoot_deg'])} -> "
                     f"{_fmt(b['overshoot_deg'])} deg, jitter {_fmt(a['delta_cmd_jitter_last_1s_deg'], 3)} -> "
                     f"{_fmt(b['delta_cmd_jitter_last_1s_deg'], 3)} deg; S2 max tilt {_fmt(c['max_tilt_deg'])} -> "
                     f"{_fmt(d['max_tilt_deg'])} deg, drift at apogee {_fmt(c['lateral_drift_at_apogee_m'], 1)} -> "
                     f"{_fmt(d['lateral_drift_at_apogee_m'], 1)} m")
        lines.append(f"equivalent actuator dead time + hold: {nom['budget']['total_equiv_s'] * 1e3:.1f} ms at {f0:.0f} Hz vs "
                     f"{fast['budget']['total_equiv_s'] * 1e3:.1f} ms at {fast['budget']['frame_hz']:.0f} Hz, against a servo "
                     f"time constant of {nom['servo_tau_s'] * 1e3:.0f} ms")
    if retuned_2x is not None and "s2" in retuned_2x:
        r1, r2 = retuned_2x["s1"], retuned_2x["s2"]
        a, c = nom["s1"], nom["s2"]
        lines.append(f"gains retuned for {retuned_2x['timing'].frame_hz:.0f} Hz frames (same cost + constraint): S1 settling "
                     f"{_fmt(a['settling_time_s'])} -> {_fmt(r1['settling_time_s'])} s, overshoot {_fmt(a['overshoot_deg'])} -> "
                     f"{_fmt(r1['overshoot_deg'])} deg, max yaw error {_fmt(a['max_yaw_error_deg'])} -> {_fmt(r1['max_yaw_error_deg'])} deg; "
                     f"S2 max tilt {_fmt(c['max_tilt_deg'])} -> {_fmt(r2['max_tilt_deg'])} deg, drift at apogee "
                     f"{_fmt(c['lateral_drift_at_apogee_m'], 1)} -> {_fmt(r2['lateral_drift_at_apogee_m'], 1)} m; phase margin "
                     f"pitch {_fmt(retuned_2x['lin']['xy']['phase_margin_deg'], 1)} / roll {_fmt(retuned_2x['lin']['z']['phase_margin_deg'], 1)} deg, "
                     f"6-DOF gain margin pitch {retuned_2x['gm_sim']['xy'][1]} / roll {retuned_2x['gm_sim']['z'][1]}")
    if whatif is not None:
        lines.append(f"what-if (linear only, ASSUMED digital servo with half the MG90S time constant, "
                     f"{whatif['tau_s'] * 1e3:.0f} ms, at {whatif['frame_hz']:.0f} Hz, baseline gains): phase margin pitch "
                     f"{_fmt(whatif['lin']['xy']['phase_margin_deg'], 1)} / roll {_fmt(whatif['lin']['z']['phase_margin_deg'], 1)} deg")
        if fast is not None:
            d_frame = [fast["lin"][a_]["phase_margin_deg"] - nom["lin"][a_]["phase_margin_deg"] for a_ in ("xy", "z")]
            d_both = [whatif["lin"][a_]["phase_margin_deg"] - nom["lin"][a_]["phase_margin_deg"] for a_ in ("xy", "z")]
            lines.append(f"the frame rate is not the limiting lag: {f0:.0f} -> {fast['budget']['frame_hz']:.0f} Hz adds "
                         f"{d_frame[0]:+.1f} / {d_frame[1]:+.1f} deg of phase margin (pitch / roll), while also halving the servo "
                         f"time constant would add {d_both[0]:+.1f} / {d_both[1]:+.1f} deg - if the attitude/heading loops need "
                         f"more bandwidth later, buy a faster servo (lower time constant, measured), not just a higher frame rate")
    verdict = (f"ADEQUATE: {f0:.0f} Hz analog servos (MG90S) meet the margins with the retuned gains; a faster digital servo is optional"
               if ok else f"NOT ADEQUATE at {f0:.0f} Hz: a faster (digital, >= {2 * f0:.0f} Hz frame) servo is recommended")
    return verdict, lines


ORDER_MAIN = ["S1", "S2", "S3", "S4", "S6", "S7", "S8"]
ORDER_INFO = ["S2s", "S2t", "S2b", "S4x"]
ORDER_WHATIF = ["S2w", "S7w", "S8w"]


def loss_tolerance(rt: dict[str, float], poll_hz: float, threshold_s: float) -> int:
    """Consecutive lost uplink frames the vehicle survives before a link-loss timer of ``threshold_s`` expires:
    gap(n) = (poll interval after a good round) + n x (poll interval after a failed round)."""
    ok_iv = max(1.0 / poll_hz, rt["rtt_worst_s"])
    fail_iv = max(1.0 / poll_hz, rt["reply_timeout_s"])
    n = 0
    while ok_iv + (n + 1) * fail_iv < threshold_s and n < 1000:
        n += 1
    return n


def write_link_sections(A, ctx: dict[str, Any]) -> None:
    veh: Vehicle = ctx["veh"]
    p = veh.params
    lk, ms = p["flight_computer"]["link"], p["mission"]
    A("\n## Ground link (SPEC §10.3, `sim/link_model.py`)\n")
    A(f"Frames: uplink CMD {lp.CMD_FRAME_BYTES} bytes, downlink TELEM {lp.TELEM_FRAME_BYTES} bytes "
      f"(`sim/link_proto.py`). The ground polls at `link.poll_hz` = {lk['poll_hz']} Hz but never while a reply is outstanding "
      f"(half-duplex), so the request rate is min(poll_hz, 1 / round trip). Vehicle id {lk['vehicle_id']}, link model "
      f"`{link_model.model_radio(p)}` (link.vehicle_radio; the LoRa fallback model is link.radio `{lk['radio']}`).\n")
    A("| quantity | " + " | ".join(link_model.RADIOS) + " |\n|" + "---|" * (len(link_model.RADIOS) + 1))
    bud = {r: link_model.budget(link_model.LinkConfig.from_params(p, radio=r)) for r in link_model.RADIOS}
    rows = [("uplink one-way (CMD) [ms]", lambda b: f"{b['uplink_s'] * 1e3:.1f}"),
            ("  of which on air [ms]", lambda b: f"{b['uplink']['air'] * 1e3:.1f}"),
            ("downlink one-way (TELEM) [ms]", lambda b: f"{b['downlink_s'] * 1e3:.1f}"),
            ("  of which on air [ms]", lambda b: f"{b['downlink']['air'] * 1e3:.1f}"),
            ("on-vehicle: uplink at the COM -> TELEM handed to the radio (measured worst, rounded up) [ms]",
             lambda b: f"{b['fc_reply_delay_s'] * 1e3:.1f}"),
            ("round trip = uplink + on-vehicle + downlink [ms]", lambda b: f"{b['rtt_worst_s'] * 1e3:.1f}"),
            ("ground reply timeout [ms]", lambda b: f"{b['reply_timeout_s'] * 1e3:.1f}"),
            ("max request rate [Hz]", lambda b: f"{b['max_poll_hz']:.2f}"),
            (f"link.poll_hz = {lk['poll_hz']} achievable", lambda b: "yes" if b["poll_hz_achievable"] else "**no**")]
    rows.append((f"consecutive lost uplink frames tolerated before I1p ({ms['link_loss_disarm_s']} s) / "
                 f"I1 ({ms['link_loss_abort_s']} s)",
                 lambda b: f"{loss_tolerance(b, lk['poll_hz'], ms['link_loss_disarm_s'])} / "
                           f"{loss_tolerance(b, lk['poll_hz'], ms['link_loss_abort_s'])}"))
    for lab, fn in rows:
        A(f"| {lab} | " + " | ".join(fn(bud[r]) for r in link_model.RADIOS) + " |")
    b0 = bud["lora_uart"]
    A(f"\nlora_uart on air = (8 x bytes + {link_model.EBYTE_AIR_OVERHEAD_BITS_ASSUMED:.0f} bits ASSUMED overhead) / "
      f"`lora_uart_air_rate_bps` ({lk['lora_uart_air_rate_bps']} bps); UART {lk['uart_baud']} baud 8N1 on both ends of each hop. "
      f"sx127x on air = Semtech AN1200.13 time-on-air with SF{lk['sx127x']['spreading_factor']}, "
      f"BW {lk['sx127x']['bandwidth_hz'] / 1e3:.0f} kHz, CR 4/{lk['sx127x']['coding_rate_denom']}, "
      f"{lk['sx127x']['preamble_symbols']}-symbol preamble, explicit header, CRC on: CMD "
      f"{b0['sx127x']['toa_cmd_s'] * 1e3:.3f} ms, TELEM {b0['sx127x']['toa_telem_s'] * 1e3:.3f} ms. ASSUMED constants (replace "
      f"with bench measurements of the real modules): "
      + ", ".join(f"`{k}` = {v}" for k, v in b0["assumed"].items()) + ". Measured on-vehicle delays (v0.6.3 three-chip "
      f"co-simulation, `sim/link_model.py`): `FC_REPLY_DELAY_S` = {link_model.FC_REPLY_DELAY_S} s (uplink frame at the "
      f"COM's radio driver -> TELEM handed to the radio, normal flight max 12.5 ms lora_uart / 13.2 ms sx127x, p50 10.7 / "
      f"9.4 ms; one constant, so the mean and the worst round trip are equal), `COM_FORWARD_S` = "
      f"{link_model.COM_FORWARD_S} s (uplink -> IL_CMD at the FCU; the supervisor takes it at the first FCU tick after "
      f"that). The ground's reply timeout is built on `FC_REPLY_ALLOWANCE_S` = {link_model.FC_REPLY_ALLOWANCE_S} s "
      f"instead (+ the {link_model.REPLY_TIMEOUT_MARGIN_S_ASSUMED * 1e3:.0f} ms margin): the latest reply the COM's reply "
      f"gate still lets out (firmware `COM_REPLY_WINDOW_MS` = 80 ms was derived from it, h + start <= 108 ms) - so bursts "
      f"of queued uplinks (up to 38.7-43.5 ms measured) and a stuck module AUX never answer into the ground's next request; the timeout "
      f"is unchanged from v0.6.2.")
    if not b0["poll_hz_achievable"]:
        A(f"\n**Finding:** with `lora_uart` at {lk['lora_uart_air_rate_bps']} bps the round trip is "
          f"{b0['rtt_worst_s'] * 1e3:.0f} ms, so the ground gets {b0['max_poll_hz']:.2f} telemetry frames per second, not "
          f"{lk['poll_hz']}; a single lost uplink frame leaves the vehicle "
          f"{(max(1 / lk['poll_hz'], b0['rtt_worst_s']) + max(1 / lk['poll_hz'], b0['reply_timeout_s'])):.2f} s without a "
          f"frame (I1p at {ms['link_loss_disarm_s']} s). A higher air rate (Ebyte 4.8k / 9.6k) or the sx127x backend "
          f"({bud['sx127x']['rtt_worst_s'] * 1e3:.0f} ms round trip) restores the margin.")
    sp = ctx["sp"]
    A("\n## Onboard supervisor (SPEC §8.8, `sim/onboard.py`)\n")
    A(f"Integer control ticks at {sp.loop_hz} Hz; durations n = smallest n with n / loop_hz >= seconds:\n")
    A("| timer | ticks | seconds |\n|---|---|---|")
    for lab, key in (("throttle ramp 0 -> 1", "ramp_ticks"), ("coast control hold", "coast_hold_ticks"),
                     ("I1p link loss on the pad", "link_disarm_ticks"), ("I1 link loss in flight", "link_abort_ticks"),
                     ("I3 rate / saturation persistence", "rate_persist_ticks"), ("I6 fall confirm", "fall_confirm_ticks"),
                     ("I7 max motor time", "max_motor_ticks"), ("I9 IMU failure", "imu_fail_ticks"),
                     ("I9 baro failure", "baro_fail_ticks"), ("release after cut / RELEASE", "release_after_cut_ticks"),
                     ("release after ABORT", "abort_release_ticks")):
        n = getattr(sp, key)
        A(f"| {lab} | {n} | {n / sp.loop_hz:g} |")
    A(f"\nI9 thresholds from {sp.i9_source}. Readings of the contract made by the reference (each the safe side; also "
      f"listed in the module docstring):\n")
    for r in SUPERVISOR_READINGS:
        A(f"* {r}")


SUPERVISOR_READINGS = (
    "a rejected command consumes its SEQ (ack_seq confirms it and the REJECTED flag says it was not executed), so a repeated "
    "frame can never execute it later;",
    "ARM needs a valid ground frame within link_loss_disarm_s before the ARM frame (\"link alive\"), so the first frame after "
    "a silence cannot arm;",
    "THROTTLE(u = 0) in ARMED is rejected (it would enter UNPOWERED on the pad, where I1 could abort and fire the release);",
    "while a release is pending only KILL is accepted; KILL or an abort keeps the earlier of the pending release time and "
    "abort_release_delay_s; on the pad, KILL / I1p / I9 / the arm switch cancel a pending release (-> SAFE);",
    "throttle increases are slewed at 1 / throttle_ramp_s, decreases are immediate;",
    "I7 counts the cumulative POWERED time since boot;",
    "KILL in ABORT or RELEASED is accepted and changes nothing (motor already off, release open or pending); the arm switch "
    "acts as KILL in ARMED / POWERED / UNPOWERED;",
    "ARM is rejected while the release servo is open (after an I10 boot); the downlink header SEQ echoes the request SEQ.",
)


def write_whatif_section(A, ctx: dict[str, Any]) -> None:
    wi = ctx.get("whatif")
    if not wi:
        return
    runs = ctx["runs"]
    p = ctx["veh"].params
    if wi.get("adopted"):
        A("\n## Coast factor check and S2 seeds (SPEC §8.8 v0.4 contract)\n")
        A(f"`mission.coast_drag_factor` = {p['mission']['coast_drag_factor']} and the predictive assist release "
          f"(`mission.assist_release_rule` = {p['mission'].get('assist_release_rule')}) are in params.json. Re-measured on S7 "
          f"(I4 is the only cut, so the coast after it is undisturbed):\n")
        A("| iteration | coast_drag_factor flown | h / v_z at the I4 cut (estimate) | apogee | factor that would have "
          "predicted it (estimate) | (truth) |\n|---|---|---|---|---|---|")
        for i, r in enumerate(wi["k_rows"]):
            A(f"| {i} | {r['k_flown']:.3f} | {r['h_est']:.1f} m / {r['vz_est']:.2f} m/s | {r['apogee']:.1f} m | "
              f"{r['k_req_est']:.3f} | {r['k_req_truth']:.3f} |")
        A(f"\nRecommended factor (measured requirement x {COAST_FACTOR_MARGIN}, rounded up to {COAST_FACTOR_STEP}): "
          f"**{wi['k_rec']:.2f}** - the value in params.json.")
        if wi.get("mc"):
            rows = wi["mc"]
            A(f"\nS2 over {len(rows)} more seeds (301-{300 + len(rows)}, contract params): PASS "
              f"{sum(1 for _, _, v in rows if v[0] == 'PASS')} / {len(rows)}; apogee "
              f"{min(m['apogee_m'] for _, m, _ in rows):.1f}-{max(m['apogee_m'] for _, m, _ in rows):.1f} m; release - apogee "
              + ", ".join(_fmt(m.get('release_minus_apogee_s'), 2) for _, m, _ in rows) + " s; v_z at release "
              + ", ".join(_fmt(m.get('vz_release_mps'), 1) for _, m, _ in rows) + " m/s; cut by "
              + ", ".join(str(m.get('cut_by') or '-').split(' ')[0] for _, m, _ in rows) + ".")
        return
    A("\n## Proposed fixes (what-if runs, NOT the SPEC §8.8 contract)\n")
    A(f"The contract runs above use `mission.coast_drag_factor` = {p['mission']['coast_drag_factor']} in both the onboard "
      f"apogee limiter I4 and the ground auto-CUT, and the literal auto-RELEASE rule. Measured on S7 (I4 is the only cut, so "
      f"the coast after it is undisturbed):\n")
    A("| iteration | coast_drag_factor flown | h / v_z at the I4 cut (estimate) | apogee | factor that would have "
      "predicted it (estimate) | (truth) |\n|---|---|---|---|---|---|")
    for i, r in enumerate(wi["k_rows"]):
        A(f"| {i} | {r['k_flown']:.3f} | {r['h_est']:.1f} m / {r['vz_est']:.2f} m/s | {r['apogee']:.1f} m | "
          f"{r['k_req_est']:.3f} | {r['k_req_truth']:.3f} |")
    A(f"\nThe factor is above 1 because the fan keeps pushing after the cut (thrust lag tau = "
      f"{p['edf']['thrust_time_constant_s']} s: impulse ~ T tau) and the body drag removes less coast height than the "
      f"assumed {(1.0 - float(p['mission']['coast_drag_factor'])) * 100:.0f} %. Proposed `mission.coast_drag_factor` = **{wi['k_rec']:.2f}** (the measured requirement x "
      f"{COAST_FACTOR_MARGIN}, rounded up to {COAST_FACTOR_STEP}). Proposed assist release: RELEASE when the downlinked v_z "
      f"predicted to the servo-opening time, v_z - g (t_link + release_after_cut_s + T_tel / 2) with T_tel = max(1 / poll_hz, "
      f"t_link), is <= apogee_detect_vz_mps (`GroundParams.release_compensated`); the literal rule (v_z <= 0 on telemetry "
      f"spanning apogee_confirm_s) cannot open the nose before ~ apogee + t_link + one telemetry interval + "
      f"release_after_cut_s.\n")
    A(metric_table([runs[k]["metrics"] for k in ORDER_WHATIF if k in runs]))
    A("")
    A(link_table([runs[k]["metrics"] for k in ORDER_WHATIF if k in runs]))
    A("")
    for k in ORDER_WHATIF:
        if k in runs:
            A(f"* **{k} - {runs[k]['verdict'][0]}**: {runs[k]['scenario'].title}. {runs[k]['verdict'][1]}")
    if wi.get("mc"):
        rows = wi["mc"]
        A(f"\nS2w over {len(rows)} seeds: PASS {sum(1 for _, _, v in rows if v[0] == 'PASS')} / {len(rows)}; apogee "
          f"{min(m['apogee_m'] for _, m, _ in rows):.1f}-{max(m['apogee_m'] for _, m, _ in rows):.1f} m; v_z at release "
          + ", ".join(_fmt(m.get('vz_release_mps'), 1) for _, m, _ in rows) + " m/s.")


def write_report(path: str, ctx: dict[str, Any]) -> None:
    veh: Vehicle = ctx["veh"]
    s = veh.summary()
    p = veh.params
    sensors: SensorConfig = ctx["sensors"]
    base, tuned, prev = ctx["base"], ctx["tuned"], ctx["prev_gains"]
    history, runs, s5rows = ctx["history"], ctx["runs"], ctx["s5rows"]
    trows, mc = ctx["trows"], ctx["mc"]
    fc = p["flight_computer"]
    dmax = float(p["vane"]["max_deflection_cmd_deg"])
    dt = 1.0 / float(p["control"]["loop_hz"])
    lines: list[str] = []
    A = lines.append
    A("# CanSat EDF Rocket - 6-DOF simulation report (SPEC §9), ATmega128 flight computer\n")
    A(f"Generated {_dt.datetime.now().isoformat(timespec='seconds')} by `sim/run_scenarios.py`; "
      f"total wall-clock {ctx['total_runtime']:.0f} s. All numbers below are produced by the run that wrote this file.\n")
    A("## Vehicle (mass source: **" + s["mass_source"] + "**)\n")
    src = os.path.relpath(veh.source_path, PROJECT_ROOT).replace("\\", "/") if veh.source_path else "internal estimate"
    A(f"Mass properties read from `{src}` (params sha256 recorded in it: `{ctx['cad_params_sha'][:16]}`, current params.json: "
      f"`{ctx['params_sha'][:16]}` - {'consistent' if ctx['cad_params_sha'] == ctx['params_sha'] else 'STALE: regenerate the CAD report'}).\n")
    A("| quantity | value |\n|---|---|")
    A(f"| mass, full stack | {s['mass_kg'] * 1e3:.0f} g |")
    A(f"| mass, post-ejection / descent | {s['post_ejection_mass_kg'] * 1e3:.0f} g / {s['descent_mass_kg'] * 1e3:.0f} g |")
    A(f"| CG height above exit plane | {s['z_cg_mm']:.1f} mm (post-ejection {s['z_cg_post_ejection_mm']:.1f} mm) |")
    I = s["inertia_diag_kg_m2"]
    A(f"| inertia about CG (Ixx, Iyy, Izz) | {I[0]:.4f}, {I[1]:.4f}, {I[2]:.4f} kg m2 |")
    A(f"| vane lever arm l_v | {s['l_v_mm']:.1f} mm |")
    A(f"| T/W at nominal thrust, hover throttle | {s['thrust_to_weight']:.3f}, u = {s['hover_throttle']:.3f} |")
    A(f"| climb throttle / thrust at climb throttle | {p['mission']['climb_throttle']:.2f} / "
      f"{veh.t_max_n * p['mission']['climb_throttle'] ** p['edf']['throttle_exponent']:.1f} N vs weight {veh.weight_n:.1f} N |")
    A(f"| Barrowman CP (nose+fins / incl. body lift) | {s['z_cp_nose_fins_mm']:.0f} / {s['z_cp_mm']:.0f} mm above exit; "
      f"static margin {s['static_margin_cal_nose_fins']:.2f} cal |")
    A(f"| pitch / roll authority at T_max, full {dmax:.0f} deg | {s['pitch_authority_Nm_at_Tmax_full_delta']:.2f} / "
      f"{s['roll_authority_Nm_at_Tmax_full_delta']:.3f} N m ({s['pitch_accel_rad_s2_at_Tmax_full_delta']:.1f} rad/s2) |")
    if veh.warnings:
        A("\nMass-source warnings: " + "; ".join(veh.warnings))

    # ---- timing ---------------------------------------------------------------------------------------------
    nb = trows["nominal"]["budget"]
    A("\n## Flight-computer timing model (`sim/fc_timing.py`)\n")
    A(f"* MCU {fc['mcu']} at {fc['f_cpu_hz'] / 1e6:.0f} MHz. Control loop `control.loop_hz` = {p['control']['loop_hz']} Hz "
      f"({fc['control_timer']}); physics RK4 at {ctx['physics_hz']:.0f} Hz (`control.imu_hz`), every step additionally broken "
      f"at each actuator-command change.")
    A(f"* Compute delay: the sample of tick k reaches the output registers {nb['compute_s'] * 1e3:.2f} ms later "
      f"(`sim/fc_timing.py` FC_COMPUTE_DELAY_S_MEASURED: the v0.6.3 three-chip co-simulation's maximum from the NAV's "
      f"MPU-6050 burst start to the FCU's last vane register, 4.709 ms; p99 4.53 ms, p50 4.24 ms in flight), sub-tick "
      f"resolution (v0.6.2 and earlier: exactly one control tick, {dt * 1e3:.1f} ms, by construction).")
    A(f"* Pad / flight switch: the NAV selects pad mode (estimator alignment, altitude filter held at 0) from the FCU "
      f"state of the last nlink exchange, {ctx['sensors'].nav_state_lag_ticks} control ticks behind the supervisor "
      f"(v0.5: 1); the altitude hold follows that state, not the true liftoff.")
    A(f"* PWM frames: `flight_computer.servo_frame_hz` = {fc['servo_frame_hz']} Hz (timer check: f_cpu / (prescaler (TOP+1)) = "
      f"{frame_hz_from_timer(fc):.2f} Hz). Channel -> timer from `flight_computer.pins`: vanes 1-4 = Timer "
      f"{', '.join(str(x) for x in ctx['channel_timer'][:4])}, ESC = Timer {ctx['channel_timer'][4]}. Each frame edge latches the "
      f"latest available command (OCR double buffering, zero-order hold for {1e3 / fc['servo_frame_hz']:.0f} ms); baseline phase: "
      f"first edge {0.5:.2f} dt = {0.5 * dt * 1e3:.1f} ms after the control tick, both timers in phase.")
    A(f"* Pulse decode: a servo acts on its pulse at the falling edge, i.e. the pulse width (`servo.pwm_center_us` "
      f"{p['servo']['pwm_center_us']} us +- `pwm_us_per_deg` x delta) after the frame edge. Then servo first-order lag "
      f"tau = {p['servo']['time_constant_s']} s, rate limit {p['servo']['rate_limit_deg_s']} deg/s, saturation at the "
      f"{p['vane']['max_deflection_mech_deg']} deg stop. ESC: same frame latch, then the {p['edf']['thrust_time_constant_s']} s thrust lag.")
    A(f"* Equivalent actuator delay at the baseline: compute {nb['compute_s'] * 1e3:.1f} ms + wait to the frame edge "
      f"{nb['wait_s'] * 1e3:.1f} ms + pulse {nb['pulse_s'] * 1e3:.1f} ms + half-frame hold {nb['hold_s'] * 1e3:.1f} ms = "
      f"**{nb['total_equiv_s'] * 1e3:.1f} ms** (the previous 400 Hz model without a frame latch had {0.5e3 / 400:.2f} ms); "
      f"servo lag tau = {p['servo']['time_constant_s'] * 1e3:.0f} ms on top.")

    # ---- sensors ----------------------------------------------------------------------------------------------
    A("\n## Sensor and estimator model (`sim/estimator.py`)\n")
    sig_g = math.degrees(sensors.gyro_noise_density_rad_s_sqrthz)
    A(f"* MPU-6050 gyro: {sig_g:.3f} deg/s/sqrt(Hz), +-{sensors.gyro_range_dps:.0f} deg/s range, {sensors.gyro_lsb_per_dps} LSB/(deg/s) "
      f"int16 quantisation ({1 / sensors.gyro_lsb_per_dps:.3f} deg/s step), on-chip DLPF first-order tau {sensors.imu_dlpf_tau_s * 1e3:.1f} ms "
      f"(ASSUMED DLPF_CFG = 2), sampled at the control tick. Bias: Gauss-Markov per axis, sigma "
      f"{math.degrees(sensors.gyro_bias_sigma_rad_s):.2f} deg/s, tau {sensors.gyro_bias_tau_s:.0f} s (ASSUMED), clipped to "
      f"+-{math.degrees(sensors.gyro_bias_max_rad_s):.1f} deg/s (brief: slowly varying, up to 0.5 deg/s).")
    A(f"* MPU-6050 accelerometer: {sensors.accel_noise_density_mps2_sqrthz / 9.80665 * 1e6:.0f} ug/sqrt(Hz), "
      f"+-{sensors.accel_range_g:.0f} g / {sensors.accel_lsb_per_g:.0f} LSB/g, bias {sensors.accel_bias_mps2 / 9.80665 * 1e3:.0f} mg "
      f"(ASSUMED residual after an offset calibration; the datasheet's uncalibrated +-50 mg would level the pad attitude wrong by up to "
      f"{math.degrees(math.atan(0.05)):.1f} deg).")
    A(f"* BMP280 barometer (forced mode, pressure x4, IIR off): {sensors.baro_noise_m:.2f} m RMS (ESTIMATE: 2.5 x the "
      f"datasheet noise, the margin of the MS5611 model it replaces), {sensors.baro_quant_m * 100.0:.1f} cm steps, at "
      f"`control.baro_hz` = {p['control']['baro_hz']} Hz, "
      f"{sensors.baro_latency_s:.2f} s latency (compensated in the altitude filter).")
    A(f"* Attitude estimate (default for S1-S5, `pad_gyro`): on the pad (IDLE/ARMED) gyro bias = running mean then EMA with "
      f"tau = {sensors.pad_avg_tau_s:.1f} s (ASSUMED, exported to gains.json as `est_pad_avg_tau_s`), roll/pitch levelled from the "
      f"averaged accelerometer, yaw = 0; after launch q <- q (x) exp((gyro - b_hat) dt), bias frozen, accelerometer unused. "
      f"The rate PID receives gyro - b_hat. The FC is assumed to have been aligning for {sensors.pad_prealign_s:.0f} s before t = 0. "
      f"The truth + {math.degrees(sensors.att_noise_rad):.1f} deg option is kept (`--estimator truth_noise`, scenario S2t).")

    # ---- gains -------------------------------------------------------------------------------------------------
    A("\n## Gains\n")
    reuse = ctx.get("gains_reuse")
    if history:
        A(f"Retuned on S1 for this plant ({p['control']['loop_hz']} Hz, {nb['compute_s'] * 1e3:.2f} ms compute delay, "
          f"{fc['servo_frame_hz']} Hz frame latch, "
          f"`{sensors.attitude_estimator}` estimator; coarse grids, then Nelder-Mead on log-gains bounded to [0.2x, 5x] of "
          f"params.control; {len(history)} evaluations, {TUNE_T_END:.0f} s runs, seed 11). Cost unchanged: {COST_DOC}; the tilt "
          f"terms use the control tilt (body z vs the estimated vertical), so the slowly drifting estimate error does not enter the "
          f"controller tuning. Added: {CONSTRAINT_DOC}.\n")
    elif reuse:
        A(f"**Not retuned** (v0.4 changed the mission logic and the radio link, not the plant): the tuned set of "
          f"`gains.json` from {reuse['prev_date']} (params sha `{reuse['prev_sha'][:16]}`, {reuse['prev_evaluations']} tuner "
          f"evaluations) was re-verified on this run before reuse: linear phase margin pitch "
          f"{_fmt(reuse['lin']['xy']['phase_margin_deg'], 1)} / roll {_fmt(reuse['lin']['z']['phase_margin_deg'], 1)} deg "
          f"(>= {PM_MIN_DEG:.0f}), linear gain margin {_fmt(reuse['lin']['xy']['gain_margin_upper_x'])} / "
          f"{_fmt(reuse['lin']['z']['gain_margin_upper_x'])} (>= {GM_MIN_X:.0f}), S1 {reuse['s1_verdict']}; the 6-DOF gain "
          f"margins are recomputed in the timing section below. Tuning method of that set: {COST_DOC}; {CONSTRAINT_DOC}. "
          f"`gains.json` was rewritten with the current params sha256 (`--retune` forces a new tuning).\n")
    else:
        A("Tuning skipped (--no-tune).\n")
    alt = ctx.get("alt_tunes") or {}
    if "unconstrained" in alt:
        u = alt["unconstrained"]
        A(f"Why the constraint: the unchanged cost alone (same search, {u['evaluations']} evaluations) converges to "
          + ", ".join(f"{k} {u['gains'][k]:.3g}" for k in TUNED_KEYS)
          + f" with S1 settling {_fmt(u['s1']['settling_time_s'])} s but a linear phase margin of only "
          f"{_fmt(u['lin']['xy']['phase_margin_deg'], 1)} deg (pitch) / {_fmt(u['lin']['z']['phase_margin_deg'], 1)} deg (roll): the "
          f"roll loop (I_zz = {veh.full.inertia_kg_m2[2, 2]:.4f} kg m2) is fast, and the S1 transient barely excites it, so the "
          f"cost cannot see the missing margin.\n")
    prev_label = (f"previous gains.json ({prev.get('loop_hz')} Hz)" if prev else "previous gains.json")
    A(f"| gain | params.json | {prev_label} | tuned (gains.json) |\n|---|---|---|---|")
    for k in GAIN_KEYS:
        if k in ("loop_hz", "imu_hz", "baro_hz", "yaw_hold"):
            continue
        mark = " *" if k in TUNED_KEYS else ""
        pv = prev.get(k) if prev else None
        A(f"| {k}{mark} | {base[k]} | {_fmt(pv, 4) if isinstance(pv, float) else _fmt(pv)} | {tuned[k]:.4g} |")
    if history:
        best = min(history, key=lambda h: h["cost"])
        A(f"\ninitial cost {history[0]['cost']:.3f} -> best {best['cost']:.3f} (settling {_fmt(history[0]['settling_time_s'])} s -> "
          f"{_fmt(best['settling_time_s'])} s, jitter {_fmt(history[0]['jitter_deg'], 3)} -> {_fmt(best['jitter_deg'], 3)} deg, "
          f"saturation {_fmt(history[0]['saturation_time_s'])} -> {_fmt(best['saturation_time_s'])} s).")
    if ctx.get("prev_on_new_plant"):
        mp_ = ctx["prev_on_new_plant"]
        A(f"\nFor reference, the previous (400 Hz) gains on this plant: S1 settling {_fmt(mp_['settling_time_s'])} s, saturation "
          f"{_fmt(mp_['saturation_time_s'])} s, vane-command jitter {_fmt(mp_['delta_cmd_jitter_last_1s_deg'], 2)} deg "
          f"(roll-loop limit cycle; linear roll phase margin {_fmt(ctx['prev_lin']['z']['phase_margin_deg'], 1)} deg) - they must not be flown on the ATmega128.")

    # ---- scenarios ---------------------------------------------------------------------------------------------
    lm = ctx["trows"]["nominal"]["lin"]
    A(f"\nLinear margins of the tuned set at the baseline timing: pitch/yaw PM {_fmt(lm['xy']['phase_margin_deg'], 1)} deg at "
      f"{_fmt(lm['xy']['crossover_hz'])} Hz, GM x{_fmt(lm['xy']['gain_margin_upper_x'])}; roll PM {_fmt(lm['z']['phase_margin_deg'], 1)} deg "
      f"at {_fmt(lm['z']['crossover_hz'])} Hz, GM x{_fmt(lm['z']['gain_margin_upper_x'])}.")
    # ---- ground link + supervisor --------------------------------------------------------------------------------
    write_link_sections(A, ctx)

    A("\n## Scenario metrics (SPEC §9 table)\n")
    order = [k for k in ORDER_MAIN + ORDER_INFO if k in runs]
    A(metric_table([runs[k]["metrics"] for k in order]))
    A(f"\n{', '.join(k for k in ORDER_INFO if k in runs)} are informational studies, not pass/fail scenarios. Tilt-estimate "
      "error = angle between the true and the estimated up direction in the body frame. Control tilt = angle between the "
      "body z axis and the *estimated* vertical (what the loop regulates); with the gyro-only estimator the true tilt = "
      "control tilt + estimate error. Apogee = CG rise above its pad height (truth); release = the tick the release servo "
      "opens (ejection modelled at that tick).")
    A("\n### Ground link and command latency\n")
    A("decision = the ground queued the command (operator or assist); exec = the onboard tick that processed its SEQ; "
      "a command sent more than once lost a frame on the way up or its acknowledgement on the way down.\n")
    A(link_table([runs[k]["metrics"] for k in order]))
    A("\n## Verdicts\n")
    for k in order:
        v, why = runs[k]["verdict"]
        A(f"* **{k} - {v}**: {runs[k]['scenario'].title}. {why}")
    if s5rows:
        v5, why5 = verdict_s5(s5rows)
        A(f"* **S5 - {v5}**: gain sensitivity +-50 %. {why5}")
    s8mc = ctx.get("s8_mc") or []
    if s8mc:
        A(f"\n### S8 link-seed Monte Carlo ({len(s8mc)} seeds, {S8_PACKET_LOSS * 100:.0f} % loss)\n")
        A("| seed | verdict | apogee [m] | cut by | abort | release by | v_z at release [m/s] | touchdown [m/s] | polls | timeouts |\n"
          "|---|---|---|---|---|---|---|---|---|---|")
        for seed, mm, vv in s8mc:
            A(f"| {seed} | {vv[0]} | {mm['apogee_m']:.1f} | {mm.get('cut_by')} | {mm.get('abort_reason') or '-'} | "
              f"{mm.get('release_by') or ('ABORT' if mm.get('t_abort_s') is not None else '-')} | {_fmt(mm.get('vz_release_mps'), 1)} | "
              f"{_fmt(mm.get('touchdown_speed_mps'), 1)} | {mm.get('polls')} | {mm.get('timeouts')} |")
        n_pass = sum(1 for _, _, vv in s8mc if vv[0] == "PASS")
        A(f"\n{n_pass} / {len(s8mc)} seeds PASS the S8 criteria.")
    write_whatif_section(A, ctx)
    if "S4x" in runs:
        L4 = runs["S4x"]["result"].log
        gmax = math.degrees(ctx["gyro_max_rad_s"])
        satk = np.flatnonzero(np.abs(np.degrees(L4["gyro"])).max(axis=1) >= gmax - 0.5 / sensors.gyro_lsb_per_dps)
        A("\n### S4 and the gyro range (why I3 exists)\n")
        m4 = runs["S4"]["metrics"]
        A(f"S4 (I3 on): ABORT {_fmt(m4['t_abort_s'] - m4['fault_t_s'] if m4['t_abort_s'] is not None else None)} s after the "
          f"fault ({m4['abort_reason']}), release at h {_fmt(m4.get('h_deploy_m'), 1)} m, touchdown "
          f"{_fmt(m4['touchdown_speed_mps'], 1)} m/s.")
        if satk.size:
            k0 = int(satk[0])
            k45 = np.flatnonzero(L4["tilt_deg"] > float(p["mission"]["tilt_abort_deg"]))
            A(f"S4x (I3 off): with vane 1 stuck at the stop, the roll torque spins the vehicle up (I_zz = "
              f"{veh.full.inertia_kg_m2[2, 2]:.4f} kg m2): the body rate reaches {np.degrees(np.abs(L4['omega']).max()):.0f} deg/s "
              f"and the MPU-6050 clips at {gmax:.0f} deg/s at t = {L4['t'][k0]:.2f} s (true tilt {L4['tilt_deg'][k0]:.1f} deg). "
              f"The gyro-only estimate then loses the rotation about z_b and with it the tilt direction: the true tilt crosses "
              f"{p['mission']['tilt_abort_deg']:.0f} deg at t = {(L4['t'][k45[0]] if k45.size else float('nan')):.2f} s while the "
              f"estimated tilt stays at {(L4['tilt_est_deg'][k45[0]] if k45.size else float('nan')):.1f} deg (estimate error up "
              f"to {L4['tilt_est_err_deg'].max():.0f} deg), so I2 cannot fire; outcome without I3: "
              f"{runs['S4x']['verdict'][1]}")
        else:
            A("S4x (I3 off): the gyro never reached full scale.")
    if s5rows:
        A("\n### S5 detail\n")
        A("| variant | settling [s] | overshoot [deg] | max tilt [deg] | max delta [deg] | saturation [s] | jitter [deg] |\n|---|---|---|---|---|---|---|")
        for (_, label, _, res, m) in s5rows:
            A(f"| {label} | {_fmt(m['settling_time_s'])} | {_fmt(m['overshoot_deg'])} | {_fmt(m['max_tilt_deg'])} | "
              f"{_fmt(m['max_delta_deg'])} | {_fmt(m['saturation_time_s'])} | {_fmt(m['delta_cmd_jitter_last_1s_deg'], 3)} |")

    # ---- timing sensitivity ------------------------------------------------------------------------------------
    A("\n## Timing sensitivity (tuned gains)\n")
    A("S1 and S2 rerun for each row; S1 verdict as above; S2 attitude = max tilt under control < tilt_abort_deg and no I2 / I3 "
      "abort (the S2 mission criteria depend on the link, not on the actuator timing). Equivalent delay = compute + wait to "
      "the frame edge + pulse decode + half the hold period.\n")
    A("| variant | equiv. delay [ms] | S1 settle [s] | S1 overshoot [deg] | S1 max delta [deg] | S1 sat [s] | S1 jitter [deg] | "
      "S1 max yaw err [deg] | S2 max tilt [deg] | S2 drift @ apogee [m] | S2 apogee [m] | S2 max delta [deg] | S2 sat [s] | "
      "S2 max yaw err [deg] | S1 / S2 attitude |\n"
      "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for key, row in trows.items():
        a, b = row["s1"], row["s2"]
        A(f"| {row['label']} | {row['budget']['total_equiv_s'] * 1e3:.1f} | {_fmt(a['settling_time_s'])} | {_fmt(a['overshoot_deg'])} | "
          f"{_fmt(a['max_delta_deg'])} | {_fmt(a['saturation_time_s'])} | {_fmt(a['delta_cmd_jitter_last_1s_deg'], 3)} | "
          f"{_fmt(a['max_yaw_error_deg'], 1)} | {_fmt(b['max_tilt_deg'])} | {_fmt(b['lateral_drift_at_apogee_m'], 1)} | "
          f"{_fmt(b['apogee_m'], 1)} | {_fmt(b['max_delta_deg'])} | {_fmt(b['saturation_time_s'])} | "
          f"{_fmt(b['max_yaw_error_deg'], 1)} | {row['v1']} / {row['v2']} |")
    A("\n### Stability margins\n")
    A(f"Linear: `sim/loop_margins.py` (hover, small signal, discrete controller + MPU DLPF + dead time + frame ZOH + servo lag, "
      f"no rate limit). 6-DOF: largest factor on the axis rate gains (kp, ki, kd) for which S1 ({TUNE_T_END:.0f} s) still settles "
      f"with a vane-command jitter < {JITTER_UNSTABLE_DEG} deg on that axis (geometric bisection up to x{MARGIN_F_MAX:.0f}; includes "
      f"saturation and the rate limit).\n")
    A("| variant | axis | crossover [Hz] | phase margin [deg] | linear gain margin | 6-DOF gain margin |\n|---|---|---|---|---|---|")
    for key in MARGIN_VARIANTS:
        if key not in trows:
            continue
        row = trows[key]
        for ax_ in ("xy", "z"):
            lm = row["lin"][ax_]
            A(f"| {row['label']} | {'pitch/yaw' if ax_ == 'xy' else 'roll'} | {_fmt(lm['crossover_hz'])} | {_fmt(lm['phase_margin_deg'], 1)} | "
              f"{_fmt(lm['gain_margin_upper_x'])} | {row['gm_sim'].get(ax_, (None, '-'))[1]} |")
    ph_keys = [k for k in trows if k.startswith("phase_") or k == "nominal"]
    if len(ph_keys) > 1:
        rng_ = {a_: [trows[k]["lin"][a_]["phase_margin_deg"] for k in ph_keys] for a_ in ("xy", "z")}
        def spread(sk: str, key: str) -> float:
            vals = [trows[k][sk][key] for k in ph_keys if trows[k][sk][key] is not None]
            return (max(vals) - min(vals)) if vals else float("nan")
        A(f"\nPWM phase sweep ({len(ph_keys)} offsets of the frame edge within the control period): linear phase margin "
          f"pitch {min(rng_['xy']):.1f}-{max(rng_['xy']):.1f} deg, roll {min(rng_['z']):.1f}-{max(rng_['z']):.1f} deg; spread of "
          f"S1 settling {spread('s1', 'settling_time_s'):.3f} s, S1 overshoot {spread('s1', 'overshoot_deg'):.2f} deg, S2 max tilt "
          f"{spread('s2', 'max_tilt_deg'):.2f} deg, S2 drift at apogee {spread('s2', 'lateral_drift_at_apogee_m'):.1f} m.")
    A("\n### Are 50 Hz analog servos adequate?\n")
    sv, slines = ctx["servo"]
    A(f"**{sv}.**\n")
    for ln in slines:
        A(f"* {ln}")

    # ---- estimator Monte Carlo ---------------------------------------------------------------------------------
    if mc:
        A("\n## S2 Monte Carlo: gyro-only estimator and ground link\n")
        t_rel, t_end_mc = ctx.get("mc_t_release_s"), ctx.get("mc_t_end_s")
        A(f"{len(mc)} seeds of S2 (sensor noise, gyro-bias realisation, accelerometer bias direction, packet-loss draws, "
          f"nonce seed), each run to t = {_fmt(t_end_mc, 1)} s (the nominal S2 release at {_fmt(t_rel, 1)} s + "
          f"{_fmt(None if t_rel is None or t_end_mc is None else t_end_mc - t_rel, 1)} s) - before touchdown, so the "
          f"touchdown term of the S2 verdict is not judged here.\n")
        keys = [("tilt-estimate error at apogee", "tilt_est_err_at_apogee_deg", "deg"),
                ("max tilt-estimate error (control active)", "max_tilt_est_err_deg", "deg"),
                ("max tilt (control active)", "max_tilt_deg", "deg"),
                ("lateral drift at apogee", "lateral_drift_at_apogee_m", "m"),
                ("apogee", "apogee_m", "m"),
                ("altitude at cut-off", "h_cutoff_m", "m"),
                ("release - apogee", "release_minus_apogee_s", "s"),
                ("v_z at release", "vz_release_mps", "m/s")]
        A("| metric | unit | n | min | median | p95 | max |\n|---|---|---|---|---|---|---|")
        for label, k, u in keys:
            v = np.array([m[k] for (_, _, m) in mc if m.get(k) is not None], dtype=float)
            if v.size:
                A(f"| {label} | {u} | {v.size} | {v.min():.2f} | {np.median(v):.2f} | {np.percentile(v, 95):.2f} | {v.max():.2f} |")
        cuts = [m.get("cut_by") for (_, _, m) in mc]
        rels = [m.get("release_by") or ("ABORT" if m.get("t_abort_s") is not None else "none") for (_, _, m) in mc]
        lim = float(p["mission"]["apogee_limit_m"])
        A(f"\nCut by: {', '.join(f'{c} x{cuts.count(c)}' for c in sorted(set(map(str, cuts))))}; release by: "
          f"{', '.join(f'{r} x{rels.count(r)}' for r in sorted(set(map(str, rels))))}; apogee above the {lim:.0f} m limit in "
          f"{sum(1 for (_, _, m) in mc if m['apogee_m'] > lim)} / {len(mc)} seeds; S2 verdict without the touchdown "
          f"term (apogee window, release |v_z|, no ABORT; the runs end before touchdown) PASS in "
          f"{sum(1 for (_, _, m) in mc if verdict_s2(m, p, touchdown=False)[0] == 'PASS')} / {len(mc)} seeds.")
        if "S2b" in runs:
            mb = runs["S2b"]["metrics"]
            A(f"\nWorst case S2b (bias drifts by the full {math.degrees(sensors.gyro_bias_max_rad_s):.1f} deg/s on x and y between "
              f"launch and apogee): tilt-estimate error at apogee {_fmt(mb['tilt_est_err_at_apogee_deg'])} deg, max under control "
              f"{_fmt(mb['max_tilt_est_err_deg'])} deg, true max tilt {_fmt(mb['max_tilt_deg'])} deg, drift at apogee "
              f"{_fmt(mb['lateral_drift_at_apogee_m'], 1)} m.")
        drift = math.degrees(sensors.gyro_bias_sigma_rad_s) * math.sqrt(2.0 / sensors.gyro_bias_tau_s)
        A(f"\nThe controller holds the *estimated* attitude upright, so the tilt-estimate error becomes a real tilt of the thrust "
          f"axis and a lateral drift; it is not observable in flight without an absolute reference. Contributors: the pad levelling "
          f"error atan(accel bias / g) = {math.degrees(math.atan(sensors.accel_bias_mps2 / 9.80665)):.2f} deg (fixed, ASSUMED "
          f"{sensors.accel_bias_mps2 / 9.80665 * 1e3:.0f} mg) and the integral of the bias change since launch; with the ASSUMED "
          f"Gauss-Markov model the bias wanders at sigma sqrt(2/tau) = {drift:.3f} deg/s per sqrt(s), so the error at apogee scales "
          f"with that number (measure the MPU-6050 bias drift over a 20 s window with the board at flight temperature to replace "
          f"the assumption; temperature-compensating the bias with the MPU-6050 die temperature is the cheapest improvement).")

    # ---- what the firmware must match ------------------------------------------------------------------------
    A("\n## What the firmware must match for these results to hold\n")
    A(f"* Gains: `sim/out/gains.json` (C99 header `sim/out/gains.h`, `#define GAINS_<KEY>`), including "
      f"`est_pad_avg_tau_s` = {sensors.pad_avg_tau_s} s for the estimator.")
    A("* Estimator: `sim/estimator.py` `PadAlignGyroEstimator` equations (running mean then EMA of gyro and accelerometer while "
      "SAFE/ARMED, levelling with yaw = 0, gyro-only quaternion integration after LAUNCH with the bias frozen) and the rate PID fed "
      "with gyro - b_hat.")
    A(f"* MPU-6050 configuration: FS_SEL = 3 (+-{sensors.gyro_range_dps:.0f} deg/s), AFS_SEL = 3, DLPF_CFG = 2 (the sim's "
      f"{sensors.imu_dlpf_tau_s * 1e3:.1f} ms lag), accelerometer offsets calibrated to <= {sensors.accel_bias_mps2 / 9.80665 * 1e3:.0f} mg.")
    A(f"* Timing: NAV IMU sample (MPU-6050 burst start) -> the FCU's last vane OCR write <= "
      f"{nb['compute_s'] * 1e3:.2f} ms (the co-simulation's maximum, which the sim applies to every sample; the "
      f"delay rows show what one tick more or less costs); Timer1 and Timer3 started together (the Timer3-offset row "
      f"shows the cost if not); {fc['servo_frame_hz']} Hz frames; the NAV's pad -> flight switch "
      f"{ctx['sensors'].nav_state_lag_ticks} ticks behind the supervisor; uplink -> TELEM handed to the radio "
      f"<= {link_model.FC_REPLY_DELAY_S * 1e3:.0f} ms in normal flight (`sim/link_model.py`).")
    A("* Onboard supervisor: `sim/onboard.py` (states, command table, interlock order, integer tick timers, per-mille throttle "
      "slew, 16-bit LFSR nonce) - every decision, tick for tick; I3 is what turns S4 into an ABORT instead of a powered crash (S4x).")
    A("* Link: `sim/link_proto.py` frame format, CRC, scaling and resynchronising decoder; byte vectors in "
      "`sim/out/link_vectors.json` (written by `sim/test_sim.py`); the flight computer replies once per received valid frame, "
      "within one control tick, echoing the request SEQ in the downlink header.")

    # ---- events / conflicts / plots / assumptions -----------------------------------------------------------
    A("\n## Event logs\n")
    A("Onboard events carry the supervisor's text; `GS` lines are the ground station (queue = decision, TX = first "
      "transmission, ACK = the telemetry confirming the SEQ).\n")
    for k in order + [k for k in ORDER_WHATIF if k in runs]:
        A(f"**{k}**")
        for t, e in runs[k]["result"].events:
            A(f"- t = {t:7.3f} s: {e}")
        A("")
    if ctx["conflicts"]:
        A("## Spec conflicts / findings\n")
        for c in ctx["conflicts"]:
            A(f"* {c}")
    A("\n## Plots\n")
    for k in order + [k for k in ORDER_WHATIF if k in runs]:
        A(f"* `sim/out/{k}.png` - {runs[k]['scenario'].title}")
    A("* `sim/out/S5.png` - gain sensitivity" + ("; `sim/out/tuning.png` - tuner convergence" if history else ""))
    A("* `sim/out/timing.png` - command chain through the frame latch, timing variants, phase margins")
    A("* `sim/out/estimator.png` - tilt-estimate error of the gyro-only estimator (Monte Carlo + worst case)")
    A("\n## Modelling assumptions not in params.json\n")
    A("* Fan speed omega = omega_max u with omega_max = KV x V_nom (no-load), so tau_swirl = f (P_nom/omega_max) T/T_max; "
      "sign +z for the CW-from-intake rotation.")
    A("* Body normal force 1/2 rho V^2 A_ref C_Na sin(alpha) at the Barrowman CP (nose 2.0, fins TIR-33 N=4, body allowance); no aerodynamic damping.")
    A("* Thrust acts along the body axis through the centreline; the CAD report's lateral CG offset is not modelled.")
    A("* Servo: first-order lag + rate limit from `servo.*`, saturated at the mechanical stop; analog-servo dead band and "
      "per-pulse motor drive are not modelled (bench step test of the MG90S at 50 Hz recommended).")
    A("* Parachute: drag at the payload-module top (tether), canopy inflation tau 1.0 s, line damping 0.8 N m s; descent mass = post-ejection + chute.")
    A("* Release servo: ejection at the tick the supervisor opens it (after release_after_cut_s / abort_release_delay_s); its "
      "frame latch and 90 deg travel are not modelled.")
    A("* Gyro scale-factor and cross-axis errors of the MPU-6050 are not modelled (only noise, bias drift, range, quantisation).")
    A(f"* Altitude complementary filter {SensorConfig().altitude_crossover_hz:g} Hz, zeta {SensorConfig().altitude_zeta:g}, "
      "baro latency compensated (feeds the telemetry and the interlocks only).")
    A("* Battery: 4.15 V/cell fresh, -0.55 V over the capacity, -0.25 V/cell IR sag at full throttle.")
    A("* Ground link: every `*_ASSUMED` constant of `sim/link_model.py` (listed in the Ground link section); latency is "
      "deterministic per frame size (no module jitter), losses are independent per frame; the ground station runs at the "
      "control-tick resolution (4 ms) and has no USB-serial buffering beyond the ASSUMED latency.")
    sc0 = runs["S2"]["scenario"]
    A(f"* Operator (`sim/sim6dof.py` ScriptedOperator): ARM from t = {sc0.arm_time_s} s while the telemetry shows SAFE, LAUNCH "
      f"from t = {sc0.launch_time_s} s while it shows ARMED, retry after {ScriptedOperator.RETRY_S} s (ASSUMED reaction time), "
      f"at most {ScriptedOperator.TRIES} commands; the IMU and baro never fail and the arm switch stays closed in the "
      f"scenarios (those interlocks are exercised by `sim/test_sim.py`).")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


def write_gains_header(path: str, gains: dict[str, Any]) -> None:
    """C99 (and C++) mirror of gains.json for the avr-gcc firmware (SPEC §8.6:
    the firmware never types a gain).  One ``#define GAINS_<KEY>`` per scalar."""
    meta = gains.get("_meta", {})
    lines = ["/* generated by sim/run_scenarios.py from sim/out/gains.json - do not edit */",
             f"/* {meta.get('date', '')}  mass source: {meta.get('mass_source', '')}  params sha256: {meta.get('params_sha256', '')[:16]} */",
             "#ifndef GAINS_H", "#define GAINS_H", ""]
    for k, v in gains.items():
        if k.startswith("_"):
            continue
        name = "GAINS_" + k.upper()
        if isinstance(v, bool):
            lines.append(f"#define {name} {1 if v else 0}")
        elif isinstance(v, int):
            lines.append(f"#define {name} {v}")
        else:
            s = f"{float(v):.6g}"
            if not any(ch in s for ch in ".en"):     # C needs a '.' or an exponent before the f suffix: 40 -> 40.0f
                s += ".0"
            lines.append(f"#define {name} {s}f")
    lines += ["", "#endif /* GAINS_H */"]
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def _sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--no-tune", action="store_true", help="use params.control (or an existing gains.json) without tuning")
    ap.add_argument("--retune", action="store_true",
                    help="run the tuner even when the existing gains.json still meets the margin targets")
    ap.add_argument("--quick", action="store_true", help="fewer tuner evaluations, Monte Carlo seeds and margin steps")
    ap.add_argument("--mass-source", choices=("auto", "cad", "sizing", "estimate"), default="auto")
    ap.add_argument("--estimator", choices=ESTIMATORS, default="pad_gyro")
    ap.add_argument("--out", default=OUT_DIR)
    ap.add_argument("--use-gains-json", action="store_true", help="with --no-tune: start from sim/out/gains.json if present")
    ap.add_argument("--plant", choices=("v09", "v081"), default="v09",
                    help="v09 (default): the v0.9 plant (vane play, rotor terms); v081: both off (before / after)")
    args = ap.parse_args(argv)
    PLANT_OVERRIDES.clear()
    if args.plant == "v081":
        PLANT_OVERRIDES.update(PLANT_V081)
    t_start = time.perf_counter()
    os.makedirs(args.out, exist_ok=True)

    veh = load_vehicle(prefer=None if args.mass_source == "auto" else args.mass_source)
    s = veh.summary()
    p = veh.params
    print(f"vehicle: mass {s['mass_kg'] * 1e3:.0f} g, CG z {s['z_cg_mm']:.1f} mm, Ixx {s['inertia_diag_kg_m2'][0]:.4f} kg m2, "
          f"T/W {s['thrust_to_weight']:.3f}, hover u {s['hover_throttle']:.3f}  [source: {s['mass_source']}]")
    for w in veh.warnings:
        print("  warning:", w)
    sensors = SensorConfig(attitude_estimator=args.estimator)
    base = gains_from_control(p["control"])
    conflicts: list[str] = []
    params_sha = _sha256(os.path.join(PROJECT_ROOT, "params.json"))
    cad_params_sha = ""
    if veh.source_path and veh.source_path.endswith(".json"):
        try:
            with open(veh.source_path, "r", encoding="utf-8") as f:
                cad_params_sha = json.load(f).get("params_sha256", "") or ""
        except (OSError, json.JSONDecodeError):
            cad_params_sha = ""
    if cad_params_sha and cad_params_sha != params_sha:
        conflicts.append("the mass-property report was generated from a different params.json (sha256 mismatch); regenerate it.")
    t_climb = veh.t_max_n * p["mission"]["climb_throttle"] ** p["edf"]["throttle_exponent"]
    if t_climb / veh.weight_n < 1.15:
        conflicts.append(f"climb throttle {p['mission']['climb_throttle']:.2f} gives {t_climb:.1f} N against a weight of "
                         f"{veh.weight_n:.1f} N (T/W {t_climb / veh.weight_n:.3f}); the climb acceleration is only "
                         f"{(t_climb - veh.weight_n) / veh.full.mass_kg:.2f} m/s2 before drag.")
    if veh.aero.static_margin_cal < 1.0:
        conflicts.append(f"coast static margin {veh.aero.static_margin_cal:.2f} calibers (nose+fins CP {veh.aero.z_cp_nose_fins_m * 1e3:.0f} mm, "
                         f"CG {veh.z_cg_m * 1e3:.0f} mm) is below the 1.0 cal PASS level of C13 (>= 0.5 -> WARN).")
    edf = p["edf"]
    omega_max = edf["kv"] * edf["voltage_nominal_V"] * 2 * math.pi / 60
    tau_swirl = edf["residual_swirl_torque_fraction_ASSUMED"] * edf["power_nominal_W"] / omega_max
    spin = tau_swirl * (p["mission"]["climb_throttle"] ** 2) * edf["thrust_time_constant_s"] / float(veh.full.inertia_kg_m2[2, 2])
    hold = float(p["mission"].get("coast_control_hold_s", 0.0))
    if hold < 2 * edf["thrust_time_constant_s"]:
        conflicts.append(f"finding: attitude control stops {hold:.2f} s after cut-off while the swirl torque "
                         f"{tau_swirl * p['mission']['climb_throttle'] ** 2:.3f} N m decays with tau = {edf['thrust_time_constant_s']:.2f} s; "
                         f"with Izz = {float(veh.full.inertia_kg_m2[2, 2]):.4f} kg m2 the body can spin up to ~{math.degrees(spin):.0f} deg/s "
                         f"in yaw. Set mission.coast_control_hold_s >= {2 * edf['thrust_time_constant_s']:.2f} s.")
    fc = p["flight_computer"]
    if abs(frame_hz_from_timer(fc) - float(fc["servo_frame_hz"])) > 1e-6:
        conflicts.append(f"flight_computer.servo_frame_hz = {fc['servo_frame_hz']} but f_cpu/(prescaler (pwm_top+1)) = "
                         f"{frame_hz_from_timer(fc):.3f} Hz; the sim uses servo_frame_hz.")
    if float(p["servo"].get("pwm_hz", fc["servo_frame_hz"])) != float(fc["servo_frame_hz"]):
        conflicts.append(f"servo.pwm_hz = {p['servo']['pwm_hz']} differs from flight_computer.servo_frame_hz = {fc['servo_frame_hz']}.")
    if veh.source_path:
        with open(veh.source_path, "r", encoding="utf-8") as f:
            com = json.load(f).get("totals", {}).get("com_mm")
        if com and math.hypot(com[0], com[1]) > 0.5:
            r_off = math.hypot(com[0], com[1]) * 1e-3
            m_trim = veh.weight_n * r_off                                   # thrust = weight at hover
            d_trim = math.degrees(m_trim / veh.vane_moment_gains(veh.weight_n)[0])
            conflicts.append(f"finding (not modelled in the 6-DOF): the CAD CG is {r_off * 1e3:.1f} mm off the thrust axis "
                             f"(x {com[0]:.2f}, y {com[1]:.2f} mm): a {m_trim:.3f} N m pitch/yaw moment at hover that the rate "
                             f"integrator must trim ({d_trim:.2f} deg of d_x/d_y; integrator limit rate_i_max_xy = "
                             f"{p['control']['rate_i_max_xy']} N m).")

    # ---- previous gains ----------------------------------------------------------------------------------------
    gains_path = os.path.join(args.out, "gains.json")
    prev_gains: dict | None = None
    prev_meta: dict = {}
    if os.path.exists(gains_path):
        with open(gains_path, "r", encoding="utf-8") as f:
            prev_full = json.load(f)
        prev_gains = {k: v for k, v in prev_full.items() if k in GAIN_KEYS}
        prev_meta = prev_full.get("_meta", {}) or {}

    # ---- tuning (only when the existing gains no longer meet the margin targets, or --retune) ------------------
    history: list[dict[str, Any]] = []
    alt_tunes: dict[str, dict[str, Any]] = {}
    gains_reuse: dict[str, Any] | None = None
    dmax_cmd = float(p["vane"]["max_deflection_cmd_deg"])
    if args.no_tune:
        tuned = dict(base)
        if args.use_gains_json and prev_gains:
            tuned.update({k: v for k, v in prev_gains.items() if k not in ("loop_hz", "imu_hz", "baro_hz")})
            print(f"gains from {gains_path}")
    else:
        if not args.retune and prev_gains and prev_meta.get("tuned_keys") and prev_gains.get("loop_hz") == p["control"]["loop_hz"]:
            g_prev = dict(base)
            g_prev.update({k: v for k, v in prev_gains.items() if k not in ("loop_hz", "imu_hz", "baro_hz")})
            lin_prev = linear_margins(veh, g_prev, FCTiming(), sensors)
            v1_prev = verdict_s1(compute_metrics(_fs(veh, scenario_s1(sensors, gains=g_prev), g_prev).run()), dmax_cmd)[0]
            margins_ok = all(lin_prev[a]["phase_margin_deg"] is not None and lin_prev[a]["phase_margin_deg"] >= PM_MIN_DEG
                             and (lin_prev[a]["gain_margin_upper_x"] is None or lin_prev[a]["gain_margin_upper_x"] >= GM_MIN_X)
                             for a in ("xy", "z"))
            if margins_ok and v1_prev == "PASS":
                tuned = g_prev
                gains_reuse = {"prev_date": prev_meta.get("tuned_on") or prev_meta.get("date", "?"),
                               "prev_sha": prev_meta.get("tuned_params_sha256") or prev_meta.get("params_sha256", ""),
                               "prev_evaluations": prev_meta.get("evaluations"), "lin": lin_prev, "s1_verdict": v1_prev,
                               "prev_meta": prev_meta}
                print(f"gains: reusing {gains_path} (PM xy {lin_prev['xy']['phase_margin_deg']:.1f} z "
                      f"{lin_prev['z']['phase_margin_deg']:.1f} deg, S1 {v1_prev}) - no retune")
            else:
                print(f"gains: the existing gains fail the margin check (PM xy {_fmt(lin_prev['xy']['phase_margin_deg'], 1)}, "
                      f"z {_fmt(lin_prev['z']['phase_margin_deg'], 1)}, S1 {v1_prev}) -> retune")
    if not args.no_tune and gains_reuse is None:
        t0 = time.perf_counter()
        tuned, history = tune_gains(veh, base, sensors, quick=args.quick)
        print(f"tuning (cost + margin constraint): {len(history)} evaluations in {time.perf_counter() - t0:.0f} s; "
              + ", ".join(f"{k} {base[k]:.3g} -> {tuned[k]:.3g}" for k in TUNED_KEYS))
        plot_tuning(history, os.path.join(args.out, "tuning.png"))
        # documentation: the unchanged cost alone, and the constrained tuning for 2x frame rate (servo question)
        f2 = 2.0 * float(fc["servo_frame_hz"])
        for key, tim, constrained in (("unconstrained", FCTiming(), False), ("frame_2x", FCTiming(frame_hz=f2), True)):
            t0 = time.perf_counter()
            g_alt, h_alt = tune_gains(veh, base, sensors, quick=args.quick, timing=tim, margin_constraint=constrained)
            m_alt = compute_metrics(_fs(veh, scenario_s1(sensors, gains=g_alt, timing=tim), g_alt).run())
            alt_tunes[key] = {"gains": g_alt, "evaluations": len(h_alt), "best_cost": min(h["cost"] for h in h_alt),
                              "lin": linear_margins(veh, g_alt, tim, sensors), "s1": m_alt, "timing": tim}
            print(f"tuning {key}: {len(h_alt)} evaluations in {time.perf_counter() - t0:.0f} s; "
                  + ", ".join(f"{k} {g_alt[k]:.3g}" for k in TUNED_KEYS)
                  + f"; PM xy {_fmt(alt_tunes[key]['lin']['xy']['phase_margin_deg'], 1)} z {_fmt(alt_tunes[key]['lin']['z']['phase_margin_deg'], 1)}")
    gains_out: dict[str, Any] = {k: tuned[k] for k in GAIN_KEYS}
    gains_out["est_pad_avg_tau_s"] = sensors.pad_avg_tau_s
    gains_out["_meta"] = {
        "written_by": "sim/run_scenarios.py", "date": _dt.datetime.now().isoformat(timespec="seconds"),
        "mass_source": veh.mass_source, "mass_kg": veh.full.mass_kg, "z_cg_mm": veh.z_cg_m * 1e3,
        "params_sha256": params_sha, "tuned_keys": list(TUNED_KEYS) if history else [],
        "tuning_scenario": (f"S1 hover-stabilise from {S1_TILT_DEG:.0f} deg, {sensors.attitude_estimator} estimator, "
                            f"{p['control']['loop_hz']} Hz control with {FCTiming().delay_s(1.0 / p['control']['loop_hz']) * 1e3:.2f} ms "
                            f"compute delay (v0.6.3 measured max), {fc['servo_frame_hz']} Hz PWM frame latch, NAV pad -> flight "
                            f"switch {sensors.nav_state_lag_ticks} ticks behind the supervisor"),
        "cost": COST_DOC, "constraint": CONSTRAINT_DOC,
        "evaluations": len(history), "best_cost": min((h["cost"] for h in history), default=None),
        "est_pad_avg_tau_s": "firmware estimator: pad gyro-bias / accelerometer averaging time constant (running mean, then EMA)",
        "note": "keys without a leading underscore mirror params.control (+ est_*); firmware generators skip '_meta'",
        "tuned_on": _dt.datetime.now().isoformat(timespec="seconds") if history else None,
        "tuned_params_sha256": params_sha if history else None,
    }
    if gains_reuse is not None:
        pm = gains_reuse["prev_meta"]
        gains_out["_meta"].update({
            "tuned_keys": pm.get("tuned_keys", []), "evaluations": pm.get("evaluations"), "best_cost": pm.get("best_cost"),
            "tuning_scenario": pm.get("tuning_scenario", gains_out["_meta"]["tuning_scenario"]),
            "tuned_on": gains_reuse["prev_date"], "tuned_params_sha256": gains_reuse["prev_sha"],
            "gains_source": (f"reused from the tuning of {gains_reuse['prev_date']} (params sha {gains_reuse['prev_sha'][:16]}); "
                             f"plant unchanged; re-verified on {gains_out['_meta']['date']}: linear PM xy "
                             f"{gains_reuse['lin']['xy']['phase_margin_deg']:.1f} / z {gains_reuse['lin']['z']['phase_margin_deg']:.1f} deg, "
                             f"S1 {gains_reuse['s1_verdict']}"),
        })
    with open(gains_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(gains_out, f, indent=2)
    write_gains_header(os.path.join(args.out, "gains.h"), gains_out)
    print(f"wrote {gains_path} (+ gains.h, C99, for the firmware)")
    prev_on_new_plant = None
    prev_lin = None
    if prev_gains and prev_gains.get("loop_hz") != p["control"]["loop_hz"]:
        g_prev = dict(base)
        g_prev.update({k: v for k, v in prev_gains.items() if k not in ("loop_hz", "imu_hz", "baro_hz")})
        prev_on_new_plant = compute_metrics(_fs(veh, scenario_s1(sensors, gains=g_prev), g_prev).run())
        prev_lin = linear_margins(veh, g_prev, FCTiming(), sensors)
        print(f"previous gains on the new plant: S1 settle {_fmt(prev_on_new_plant['settling_time_s'])} s, jitter "
              f"{prev_on_new_plant['delta_cmd_jitter_last_1s_deg']:.2f} deg, roll PM {_fmt(prev_lin['z']['phase_margin_deg'], 1)} deg")

    # ---- scenarios ------------------------------------------------------------------------------------------
    runs: dict[str, Any] = {}
    s3, c3 = scenario_s3(veh, sensors)
    if c3:
        conflicts.append(c3)
    scen = [scenario_s1(sensors), scenario_s2(veh, sensors), s3, scenario_s4(veh, sensors), scenario_s6(veh, sensors),
            scenario_s7(veh, sensors), scenario_s8(veh, sensors), scenario_s2_sx127x(veh, sensors),
            scenario_s2_truth(veh, sensors), scenario_s4_no_i3(veh, sensors)]
    if args.quick:
        scen = [replace(sc, t_end=min(sc.t_end, 110.0 if sc.name == "S3" else 90.0)) for sc in scen]
    tilt_abort = float(p["mission"]["tilt_abort_deg"])

    def judge(sc: Scenario, m: dict[str, Any]) -> tuple[str, str]:
        base_name = sc.name.split("_")[0]
        if base_name == "S1":
            return verdict_s1(m, dmax_cmd)
        if base_name in ("S4", "S4x"):
            v = verdict_s4(m)
            return v if base_name == "S4" else ("INFO", f"would be {v[0]} by the S4 criteria: {v[1]}")
        if base_name == "S6":
            return verdict_s6(m)
        if base_name in ("S7", "S7w"):
            v = verdict_s7(m, p)
            return v if base_name == "S7" else ("INFO", f"would be {v[0]} by the S7 criteria: {v[1]}")
        if base_name in ("S8", "S8w"):
            v = verdict_s8(m, p)
            return v if base_name == "S8" else ("INFO", f"would be {v[0]} by the S8 criteria: {v[1]}")
        v = verdict_s2(m, p, target=sc.target_altitude_m, limit=sc.apogee_limit_m)
        if base_name == "S2":
            return v
        extra = (f"; tilt-estimate error at apogee {_fmt(m['tilt_est_err_at_apogee_deg'])} deg, max under control "
                 f"{_fmt(m['max_tilt_est_err_deg'])} deg" if base_name in ("S2t", "S2b") else "")
        crit = "S2 criteria with the scenario's target / limit" if base_name == "S3" else "S2 criteria"
        return "INFO", f"would be {v[0]} by the {crit}: {v[1]}{extra}"

    def run_one(sc: Scenario, plot: bool = True, quiet: bool = False) -> tuple[SimResult, dict[str, Any], tuple[str, str]]:
        res = _fs(veh, sc, tuned).run()
        m = compute_metrics(res)
        v = judge(sc, m)
        if plot:
            runs[sc.name] = {"scenario": sc, "result": res, "metrics": m, "verdict": v,
                             "attitude_ok": attitude_ok(m, tilt_abort) if not sc.bypass_state_machine else True}
            plot_scenario(res, m, veh, os.path.join(args.out, f"{sc.name}.png"))
        print(f"{sc.name}: {v[0]} - {v[1]}  [{res.runtime_s:.1f} s wall]")
        if not quiet:
            for t, e in res.events:
                print(f"    t={t:7.3f}  {e}")
        return res, m, v

    for sc in scen:
        run_one(sc)
    # S8 link-seed Monte Carlo (the packet-loss pattern decides the outcome)
    s8_mc = []
    for i in range(S8_SEEDS_QUICK if args.quick else S8_SEEDS_FULL):
        sc8 = scenario_s8(veh, sensors, seed=201 + i)
        if args.quick:
            sc8 = replace(sc8, t_end=90.0)
        _, m8, v8 = run_one(sc8, plot=False, quiet=True)
        s8_mc.append((201 + i, m8, v8))

    # ---- what-if: recalibrated coast_drag_factor (from S7) and the delay-compensated assist release ------------
    whatif_ctx: dict[str, Any] = {"k_rows": []}
    g0 = SupervisorParams.from_params(p).g

    def k_row(name: str, k_flown: float) -> dict[str, float] | None:
        r = runs[name]
        m_, L_ = r["metrics"], r["result"].log
        if not (m_.get("cut_by") or "").startswith("I4") or m_.get("t_cutoff_s") is None:
            return None
        kc = min(int(np.searchsorted(L_["t"], m_["t_cutoff_s"])), len(L_["t"]) - 1)
        h_e, v_e = float(L_["h_est"][kc]), float(L_["vz_est"][kc])
        h_t, v_t = float(L_["pos"][kc, 2] - L_["pos"][0, 2]), float(L_["vel"][kc, 2])
        return {"k_flown": k_flown, "h_est": h_e, "vz_est": v_e, "apogee": m_["apogee_m"],
                "k_req_est": (m_["apogee_m"] - h_e) / (v_e * v_e / (2.0 * g0)),
                "k_req_truth": (m_["apogee_m"] - h_t) / (v_t * v_t / (2.0 * g0))}

    row0 = k_row("S7", float(p["mission"]["coast_drag_factor"]))
    if row0 is not None:
        whatif_ctx["k_rows"].append(row0)
        k_rec = math.ceil(row0["k_req_est"] * COAST_FACTOR_MARGIN / COAST_FACTOR_STEP - 1e-9) * COAST_FACTOR_STEP
        for _ in range(3):                         # the requirement shifts a little with the cut speed: iterate
            sc7w = replace(scenario_s7(veh, sensors), name="S7w", coast_drag_factor=k_rec,
                           title=f"S7 with coast_drag_factor = {k_rec:.2f} (what-if)")
            run_one(sc7w)
            rw = k_row("S7w", k_rec)
            if rw is None:
                break
            whatif_ctx["k_rows"].append(rw)
            k_new = math.ceil(max(rw["k_req_est"], row0["k_req_est"]) * COAST_FACTOR_MARGIN / COAST_FACTOR_STEP - 1e-9) \
                * COAST_FACTOR_STEP
            if k_new <= k_rec + 1e-9:
                break
            k_rec = k_new
        whatif_ctx["k_rec"] = k_rec
        k_par = float(p["mission"]["coast_drag_factor"])
        predictive = str(p["mission"].get("assist_release_rule", "predictive")) == "predictive"
        # v0.4: the recommendation is the contract once params.json carries it; then the S2 seeds run the contract
        adopted = abs(k_rec - k_par) < 1e-9 and predictive
        whatif_ctx["adopted"] = adopted
        wi_kw = {} if adopted else dict(coast_drag_factor=k_rec, release_compensated=True)
        if not adopted:
            run_one(replace(scenario_s2(veh, sensors), name="S2w", **wi_kw,
                            title=f"S2 with coast_drag_factor = {k_rec:.2f} and the delay-compensated assist release (what-if)"))
            run_one(replace(scenario_s8(veh, sensors), name="S8w", **wi_kw,
                            title=f"S8 with coast_drag_factor = {k_rec:.2f} and the delay-compensated assist release (what-if)"))
        wmc = []
        for i in range(3 if args.quick else 10):
            scw = replace(scenario_s2(veh, sensors), name=f"S2w_{301 + i}", seed=301 + i, t_end=60.0, **wi_kw)
            _, mw, _ = run_one(scw, plot=False, quiet=True)
            wmc.append((301 + i, mw, verdict_s2(mw, p)))
        whatif_ctx["mc"] = wmc
    else:
        whatif_ctx = {}
    # ---- findings of the v0.4 link / supervisor runs (contract params) -------------------------------------
    lim_ = float(p["mission"]["apogee_limit_m"])
    m7 = runs["S7"]["metrics"]
    if m7["apogee_m"] > lim_ and whatif_ctx.get("k_rows"):
        r0 = whatif_ctx["k_rows"][0]
        conflicts.append(
            f"finding (contract cannot hold the legal ceiling): I4 cuts when h + coast_drag_factor v_z^2/(2g) >= "
            f"{lim_:.0f} m with mission.coast_drag_factor = {p['mission']['coast_drag_factor']}, but the coast after the cut "
            f"reaches {m7['apogee_m']:.1f} m (S7; S2 {runs['S2']['metrics']['apogee_m']:.1f} m): the measured factor is "
            f"{r0['k_req_est']:.3f} (estimate) / {r0['k_req_truth']:.3f} (truth) because the fan thrust decays with tau = "
            f"{p['edf']['thrust_time_constant_s']} s after the cut and the drag loss is smaller than {(1.0 - float(p['mission']['coast_drag_factor'])) * 100:.0f} %. With "
            f"coast_drag_factor = {whatif_ctx['k_rec']:.2f} the same flight peaks at {runs['S7w']['metrics']['apogee_m']:.1f} m "
            f"(S7w).")
    m2_ = runs["S2"]["metrics"]
    predictive_ = str(p["mission"].get("assist_release_rule", "predictive")) == "predictive"
    rel_rows = [c for c in command_latencies(runs["S2"]["result"]) if c["cmd"] == "RELEASE"]
    if (predictive_ and m2_.get("vz_release_mps") is not None and abs(m2_["vz_release_mps"]) >= RELEASE_VZ_MAX_MPS
            and rel_rows):
        rr = rel_rows[0]
        mc_ = whatif_ctx.get("mc") or []
        n_ok = sum(1 for _, _, v in mc_ if v[0] == "PASS")
        conflicts.append(
            f"finding: S2 opens the nose {m2_['release_minus_apogee_s']:.2f} s after apogee at v_z {m2_['vz_release_mps']:.1f} m/s "
            f"(by {m2_.get('release_by')}). The predictive assist RELEASE was decided in time, but it was transmitted "
            f"{rr['n_tx']}x (first transmission -> execution {_fmt(rr['tx_to_exec_s'], 2)} s): "
            + ("a lost uplink frame is repeated only after the reply timeout, one poll period (1 / poll_hz = "
               f"{1.0 / float(p['flight_computer']['link']['poll_hz']):.1f} s) later, i.e. ~5 m/s more descent speed. "
               if rr["n_tx"] > 1 else "")
            + (f"Over {len(mc_)} seeds with the same {float(p['flight_computer']['link']['sim_packet_loss']) * 100:.0f} % loss "
               f"{n_ok} meet every S2 criterion (seeds 301-{300 + len(mc_)}). " if mc_ else "")
            + "A faster air rate (lower round trip, faster polling) or the sx127x backend shortens the repeat; "
              f"the late release still comes before the I6 fall-safe (v_z <= {p['mission']['fall_deploy_vz_mps']} m/s).")
    elif m2_.get("vz_release_mps") is not None and abs(m2_["vz_release_mps"]) >= RELEASE_VZ_MAX_MPS:
        conflicts.append(
            f"finding: the literal assist auto-RELEASE (downlinked v_z <= {p['mission']['apogee_detect_vz_mps']} m/s over "
            f"{p['mission']['apogee_confirm_s']} s of telemetry) opens the nose {m2_['release_minus_apogee_s']:.2f} s after "
            f"apogee at v_z {m2_['vz_release_mps']:.1f} m/s in S2 (by {m2_.get('release_by')}): telemetry age + one telemetry "
            f"interval + uplink + release_after_cut_s add up to ~1 s on this link, so |v_z| < {RELEASE_VZ_MAX_MPS:.0f} m/s at "
            f"release is not reachable with that rule; the delay-compensated rule of the what-if runs releases within "
            f"{abs(runs['S2w']['metrics'].get('release_minus_apogee_s') or float('nan')):.2f} s of apogee (S2w)."
            if "S2w" in runs else
            f"finding: the literal assist auto-RELEASE opens the nose {m2_['release_minus_apogee_s']:.2f} s after apogee at "
            f"v_z {m2_['vz_release_mps']:.1f} m/s in S2.")
    bl = link_model.budget(link_model.LinkConfig.from_params(p))
    if not bl["poll_hz_achievable"]:
        conflicts.append(
            f"finding: link.poll_hz = {bl['poll_hz']} Hz is not achievable on `{bl['radio']}` at "
            f"{p['flight_computer']['link']['lora_uart_air_rate_bps']} bps (round trip {bl['rtt_worst_s'] * 1e3:.0f} ms -> "
            f"{bl['max_poll_hz']:.2f} requests/s with ASSUMED Ebyte overheads); one lost uplink frame already uses "
            f"{(max(1 / bl['poll_hz'], bl['rtt_worst_s']) + max(1 / bl['poll_hz'], bl['reply_timeout_s'])) / float(p['mission']['link_loss_disarm_s']) * 100:.0f} % "
            f"of link_loss_disarm_s. Measure the module round trip; a 4.8k+ air rate or the sx127x backend restores margin.")
    m2 = runs["S2"]["metrics"]
    t_asc = m2["t_apogee_s"] - runs["S2"]["scenario"].launch_time_s
    lin_nom = linear_margins(veh, tuned, FCTiming(), sensors)
    conflicts.append(
        f"finding: heading (rotation about z_b) excursion. The residual swirl torque "
        f"({tau_swirl * p['mission']['climb_throttle'] ** 2:.3f} N m at climb throttle, ASSUMED fraction) arrives at liftoff with the "
        f"roll integrator empty (the pad holds the vehicle), and the roll loop is margin-limited at {fc['servo_frame_hz']} Hz frames "
        f"(crossover {_fmt(lin_nom['z']['crossover_hz'])} Hz, PM {_fmt(lin_nom['z']['phase_margin_deg'], 1)} deg): S2 max yaw error "
        f"{_fmt(m2['max_yaw_error_deg'], 0)} deg, peak yaw rate after cut-off {_fmt(m2.get('max_yaw_rate_after_cutoff_deg_s'), 0)} deg/s. "
        f"Heading is not a mission requirement, but a swirl feed-forward (roll trim -tau_swirl(u) added to M_z, or the roll "
        f"integrator preloaded at LAUNCH) would remove most of it; that is a controller change (sim/controller.py) and was not made.")
    s2b = scenario_s2_bias_ramp(veh, sensors, t_asc)
    if args.quick:
        s2b = replace(s2b, t_end=90.0)
    run_one(s2b)

    # S5
    s5rows = []
    for key, label, f_, sc in s5_variants(sensors, tuned):
        res = _fs(veh, sc, tuned).run()
        m = compute_metrics(res)
        s5rows.append((key, label, f_, res, m))
        print(f"S5 {label}: settle {_fmt(m['settling_time_s'])} s, overshoot {m['overshoot_deg']:.2f} deg, max tilt {m['max_tilt_deg']:.1f}, "
              f"max delta {m['max_delta_deg']:.1f} deg, sat {m['saturation_time_s']:.2f} s")
    plot_s5(s5rows, runs["S1"]["result"], os.path.join(args.out, "S5.png"))
    v5 = verdict_s5(s5rows)
    print(f"S5: {v5[0]} - {v5[1]}")

    # ---- timing sensitivity + margins ------------------------------------------------------------------------
    trows: dict[str, dict[str, Any]] = {}
    t_s2_end = (m2["t_deploy_s"] or m2["t_apogee_s"]) + 1.0
    for key, label, tim in timing_variants(veh):
        r1 = _fs(veh, scenario_s1(sensors, timing=tim), tuned).run()
        # same t_end as S2 in a full run: the pre-generated noise realisation depends on the run length
        r2 = _fs(veh, replace(scenario_s2(veh, sensors, timing=tim), t_end=150.0 if not args.quick else t_s2_end), tuned).run()
        a, b = compute_metrics(r1), compute_metrics(r2)
        v1 = verdict_s1(a, dmax_cmd)[0]
        v2 = "PASS" if attitude_ok(b, tilt_abort) else "FAIL"
        row = {"label": label, "timing": tim, "s1": a, "s2": b, "s1_result": r1, "v1": v1, "v2": v2,
               "budget": timing_delay_budget(veh, tim), "lin": linear_margins(veh, tuned, tim, sensors), "gm_sim": {},
               "servo_tau_s": float(p["servo"]["time_constant_s"])}
        if key in MARGIN_VARIANTS:
            for ax_ in ("xy", "z"):
                row["gm_sim"][ax_] = sim_gain_margin(veh, tuned, tim, sensors, ax_, iters=3 if args.quick else 6)
        trows[key] = row
        print(f"timing {key}: delay {row['budget']['total_equiv_s'] * 1e3:.1f} ms; S1 settle {_fmt(a['settling_time_s'])} s, "
              f"sat {a['saturation_time_s']:.2f} s, jitter {a['delta_cmd_jitter_last_1s_deg']:.3f}; S2 max tilt {b['max_tilt_deg']:.2f}, "
              f"drift {b['lateral_drift_at_apogee_m']:.1f} m; PM xy {_fmt(row['lin']['xy']['phase_margin_deg'], 1)} "
              f"z {_fmt(row['lin']['z']['phase_margin_deg'], 1)}; GM sim {row['gm_sim']}")
    plot_timing(trows, os.path.join(args.out, "timing.png"), float(fc["servo_frame_hz"]))
    retuned_2x = alt_tunes.get("frame_2x")
    if retuned_2x is not None:
        g2 = retuned_2x["gains"]
        retuned_2x["s2"] = compute_metrics(_fs(veh, scenario_s2(veh, sensors, timing=retuned_2x["timing"]), g2).run())
        retuned_2x["gm_sim"] = {ax_: sim_gain_margin(veh, g2, retuned_2x["timing"], sensors, ax_, iters=3 if args.quick else 6)
                                for ax_ in ("xy", "z")}
    tau_half = 0.5 * float(p["servo"]["time_constant_s"])
    whatif = {"tau_s": tau_half, "frame_hz": 2.0 * float(fc["servo_frame_hz"]),
              "lin": linear_margins(veh, tuned, FCTiming(frame_hz=2.0 * float(fc["servo_frame_hz"])), sensors, servo_tau_s=tau_half)}
    servo = servo_assessment(trows, runs, retuned_2x, whatif)
    print("servo:", servo[0])
    for ln in servo[1]:
        print("   ", ln)

    # ---- estimator Monte Carlo ------------------------------------------------------------------------------
    mc: list[tuple[int, SimResult, dict]] = []
    if args.estimator == "pad_gyro":
        n_mc = MC_SEEDS_QUICK if args.quick else MC_SEEDS_FULL
        for i in range(n_mc):
            seed = 101 + i
            sc = replace(scenario_s2(veh, sensors), name=f"MC{seed}", seed=seed, t_end=t_s2_end + MC_EXTRA_S)
            res = _fs(veh, sc, tuned).run()
            mc.append((seed, res, compute_metrics(res)))
        errs = np.array([m["tilt_est_err_at_apogee_deg"] for (_, _, m) in mc])
        print(f"estimator MC ({n_mc} seeds): tilt-estimate error at apogee median {np.median(errs):.2f}, "
              f"p95 {np.percentile(errs, 95):.2f}, max {errs.max():.2f} deg")
        plot_estimator(mc, runs["S2"]["result"], runs["S2b"]["result"], os.path.join(args.out, "estimator.png"))

    # ---- finding: what a lost CUT / RELEASE exchange still costs (ground assist, v0.6.3) --------------------
    pool = ([("S2", runs["S2"]["metrics"])] + [(f"S2w_{seed}", mm) for seed, mm, _ in (whatif_ctx.get("mc") or [])]
            + [(f"MC{seed}", mm) for (seed, _, mm) in mc])

    def _ntx(mm: dict[str, Any], name: str) -> Any:
        return next((c["n_tx"] for c in mm.get("commands", []) if c["cmd"] == name), "-")

    i4_pool = [(k, mm) for k, mm in pool if str(mm.get("cut_by") or "").startswith("I4")]
    late_pool = [(k, mm) for k, mm in pool if mm.get("vz_release_mps") is not None
                 and abs(mm["vz_release_mps"]) >= RELEASE_VZ_MAX_MPS]
    lt = link_model.round_trip(link_model.LinkConfig.from_params(p))
    poll_iv = 1.0 / float(p["flight_computer"]["link"]["poll_hz"])
    t_rep = max(poll_iv, lt["reply_timeout_s"])                 # a lost CUT / RELEASE goes out again this much later
    ac = runs["S2"]["metrics"].get("assist_cut") or {}
    rate = float(ac.get("rate_mps") or 0.0)
    t_tel = max(poll_iv, lt["rtt_worst_s"])
    lim_m = float(p["mission"]["apogee_limit_m"])
    conflicts.append(
        f"finding (link-loss residual, {len(pool)} S2-type flights at "
        f"{100 * float(p['flight_computer']['link']['sim_packet_loss']):.0f} % loss: S2, its extra seeds and the Monte Carlo "
        f"seeds): the onboard I4 limiter cut first in {len(i4_pool)} ("
        + (", ".join(f"{k}: apogee {mm['apogee_m']:.1f} m, CUT sent {_ntx(mm, 'CUT')}x" for k, mm in i4_pool) or "none")
        + f"); the release missed |v_z| < {RELEASE_VZ_MAX_MPS:.0f} m/s in {len(late_pool)} ("
        + (", ".join(f"{k}: v_z {mm['vz_release_mps']:.2f} m/s, RELEASE sent {_ntx(mm, 'RELEASE')}x" for k, mm in late_pool)
           or "none")
        + f"). Since v0.6.3 the ground assist also decides at a reply timeout on the extrapolated telemetry, so a lost "
          f"telemetry frame no longer postpones the CUT / RELEASE; what remains is a lost CUT / RELEASE exchange itself, "
          f"repeated {t_rep:.3f} s later (max(reply timeout, poll interval)): dP/dt x that = {rate * t_rep:.1f} m of "
          f"predicted apogee (S2 dP/dt {rate:.1f} m/s) against the {lim_m - float(p['mission']['target_altitude_m']):.0f} m "
          f"between mission.target_altitude_m and apogee_limit_m, and g x that = {9.80665 * t_rep:.1f} m/s of descent "
          f"against the release window. I4 caps the apogee in those flights (no unsafe outcome). Removing it needs a "
          f"faster link (shorter timeout and poll interval) or target_altitude_m <= "
          f"{lim_m - rate * (0.5 * t_tel + t_rep):.0f} m (limit - dP/dt (T_tel / 2 + repeat)).")

    total = time.perf_counter() - t_start
    ctx = {"veh": veh, "sensors": sensors, "base": base, "tuned": tuned, "prev_gains": prev_gains, "history": history,
           "runs": runs, "s5rows": s5rows, "trows": trows, "mc": mc, "servo": servo, "conflicts": conflicts,
           "mc_t_end_s": t_s2_end + MC_EXTRA_S, "mc_t_release_s": m2["t_deploy_s"] or m2["t_apogee_s"],
           "total_runtime": total, "params_sha": params_sha, "cad_params_sha": cad_params_sha,
           "channel_timer": timer_of_channel(fc), "physics_hz": float(p["control"]["imu_hz"]),
           "prev_on_new_plant": prev_on_new_plant, "prev_lin": prev_lin, "alt_tunes": alt_tunes,
           "gyro_max_rad_s": Mpu6050(sensors.gyro_lsb_per_dps, sensors.gyro_range_dps, sensors.accel_lsb_per_g,
                                     sensors.accel_range_g, 9.80665).gyro_max,
           "gains_reuse": gains_reuse, "sp": SupervisorParams.from_params(p), "s8_mc": s8_mc, "whatif": whatif_ctx}
    write_report(os.path.join(args.out, "sim_report.md"), ctx)
    metrics_out: dict[str, Any] = {k: runs[k]["metrics"] | {"verdict": runs[k]["verdict"][0]} for k in runs}
    metrics_out["S8_mc"] = {str(seed): mm | {"verdict": vv[0]} for seed, mm, vv in s8_mc}
    metrics_out["whatif"] = {k: v for k, v in whatif_ctx.items() if k != "mc"} | (
        {"S2w_mc": {str(seed): mm | {"verdict": vv[0]} for seed, mm, vv in whatif_ctx["mc"]}} if whatif_ctx.get("mc") else {})
    metrics_out["link_budget"] = {r: link_model.budget(link_model.LinkConfig.from_params(p, radio=r)) for r in link_model.RADIOS}
    metrics_out["S5"] = {"verdict": v5[0], "variants": {label: m for (_, label, _, _, m) in s5rows}}
    metrics_out["timing"] = {k: {"label": r["label"], "budget": r["budget"], "S1": r["s1"], "S2": r["s2"], "verdict_S1": r["v1"],
                                 "verdict_S2": r["v2"], "linear_margins": r["lin"],
                                 "sim_gain_margin": {a_: {"factor": g_[0], "text": g_[1]} for a_, g_ in r["gm_sim"].items()}}
                         for k, r in trows.items()}
    metrics_out["servo_assessment"] = {"verdict": servo[0], "lines": servo[1]}
    metrics_out["alternative_tunings"] = {k: {"gains": {kk: a_["gains"][kk] for kk in TUNED_KEYS}, "evaluations": a_["evaluations"],
                                              "linear_margins": a_["lin"], "S1": a_["s1"], "S2": a_.get("s2"),
                                              "sim_gain_margin": a_.get("gm_sim")}
                                          for k, a_ in alt_tunes.items()}
    if mc:
        metrics_out["estimator_mc"] = {str(seed): m for (seed, _, m) in mc}
    metrics_out["_meta"] = gains_out["_meta"] | {"conflicts": conflicts, "total_runtime_s": total}
    with open(os.path.join(args.out, "metrics.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(metrics_out, f, indent=2, default=str)
    print(f"report: {os.path.join(args.out, 'sim_report.md')}  (total {total:.0f} s)")
    for c in conflicts:
        print("CONFLICT:", c)
    return 0


if __name__ == "__main__":
    sys.exit(main())
