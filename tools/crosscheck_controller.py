#!/usr/bin/env python3
"""Cross-check the ATmega128 firmware core (C, float32) against the Python reference (float64).

Runs, in order:

1. ``tools/gen_firmware_params.py`` and ``tools/gen_gains_header.py`` (headers up to date with the
   current params.json / CAD report / gains.json), ``tools/build_host_test.py`` (host binary).
2. **Plant check** - the generator's l_v, r_v, S_v, A_e, T_max equal ``sim.vehicle.load_vehicle()``
   (the simulation's own mass-property path) to 1e-12.  fw09 X1 (SPEC_firmware_v09 3.2 G1 / WP-X): **per
   configuration** - the rocket and the VTOL battery / tether / hybrid vehicles (``load_vehicle(config='vtol',
   power=...)``) against the generator's per-configuration sets, and the host core compiled for each VTOL configuration
   and build profile (``-D FC_VCFG``, ``-D MISSION_PROFILE_DEMO``; ``test_core --consts``) against the reference's own
   parameter objects of that configuration (plant, attitude gains, supervisor incl. the tethered limits and the power
   table, vertical controller, position hold, NAV estimator and altitude filter, VEH_* / FC_HOVER_THROTTLE): a build
   that flies another vehicle's constants fails (until G1 every VTOL image carried the rocket's l_v 351 mm and hover
   throttle 0.916 and this check saw only the rocket).  The 6-DOF replays of 4d / 7b record the parameters each run flew
   with (and its configuration's nominal ones, record format ``ccrec/1`` at SIXDOF_HOOKS) and compare them with the build
   of the run's configuration BEFORE replaying it through that build's host core; ``--only g1`` runs just these checks;
   ``--campaign-set <set> --campaign-dir <harness out> --campaign-stride N`` replays the runs the eval harness recorded
   (``eval_v09g.py --record``; SPEC 8.4) the same way.
3. **Attitude controller** - >= 2000 input vectors in sequences with state carried across steps
   (integrators, filters, saturation latch): random but physically plausible attitude/rate/throttle
   histories plus the controller inputs recorded from 6-DOF runs of the simulation.  Inputs are
   rounded to float32 and fed identically to ``sim.controller.AttitudeController`` (float64) and
   ``test_core --vectors`` (float32).  Assertion: max |delta_C - delta_py| < DELTA_BOUND_RAD.
4. **Onboard supervisor** (SPEC 8.8) - ``sim.onboard.OnboardSupervisor`` vs ``src/core/onboard.c``
   (``test_core --onb-vectors``) on (a) random command / sensor sequences from a scripted ground
   (polls with loss, outages, repeated / old / wrapped SEQs, wrong nonces, unknown codes, several
   frames in one tick) and scripted faults (arm switch, IMU / baro failures, tilt and rate spikes,
   low battery, climbs past the apogee limit and the geofence, falls), with default and shortened
   timers, random nonce seeds and I10 boots; (b) the supervisor inputs recorded tick by tick from
   closed-loop 6-DOF flights (ground station + link model + supervisor).  Inputs are float32 and the
   reference gets the firmware's float32 thresholds.  Assertion: on every tick identical state,
   target and slewed throttle, control/release flags, reason, telemetry flags, ack SEQ, nonce, reply
   SEQ, the 28 TELEM payload bytes, the executed/rejected commands and the state transitions
   (0 mismatching ticks); every interlock (I1-I9, arm switch), I10 and every rejection code must
   occur.  The only float arithmetic the supervisor does on sensor values is |w_xy| (I3) and
   h + k v_z^2 (I4); inputs within 1e-5 (relative) of those two thresholds are moved out of the
   band before both sides see them (their count is reported).
5. **Link protocol** (SPEC 10.3) - ``sim.link_proto`` vs ``src/core/link_proto.c``
   (``test_core --proto-vectors``): random byte streams of valid, foreign, mistyped, corrupted
   (bit flips, cut, LEN changed, bytes inserted), back-to-back and garbage-wrapped frames through
   both streaming decoders (frames and every counter equal), random CMD / TELEM encodings (bytes
   equal) and float32 quantisation (equal integers, incl. ties, NaN, inf, denormals).
6. **ilink** (SPEC 10.4, v0.6 FCU <-> COM) - ``sim.ilink_proto`` vs ``src/core/ilink_proto.c``
   (``test_core --ilink-vectors``): random streams through both streaming decoders (valid CMD / SNAP / STAT
   frames, protocol-rich garbage, fake headers swallowing real frames, sync spam, valid-CRC frames with a wrong
   LEN / TYPE, corrupted frames incl. dropped bytes; max_out 0..8; the decoder's accept mask ALL, FCU = CMD | STAT
   or COM = SNAP, v0.6.3) - every returned frame, every counter, the open candidate and il_decoder_need /
   il_decoder_cmd_need after every byte equal, and the zero-copy il_decoder_push (a twin decoder in C) returns the
   same frames;
   re-encodings of random payloads incl. every special float pattern (bytes equal) and refused (TYPE, LEN)
   pairs; il_crc16 == lp_crc16; the FCU snapshot (il_snap_from_tlm_src) byte-exact; the COM TELEM path
   (il_unpack_snap, il_snap_to_tlm_src, onboard_telemetry, lp_encode_telem) vs ``telem_frame_from_snap``:
   every byte equal except roll / pitch / yaw, which may differ by one 0.01 deg quantum (float32 vs float64
   trig, counted).  ``--only ilink`` runs this section alone without the generators (it builds the host
   test from the headers as they are) and writes ``crosscheck_report_ilink.json`` (``--only log``: section 7's
   log round trip the same way, ``--only nlink``: section 6b).
6b. **nlink** (SPEC 10.5, v0.6.3 NAV <-> FCU) - ``sim.nlink_proto`` vs ``src/core/nlink_proto.c``
   (``test_core --nlink-vectors``): random NAV frames (every float class incl. NaN payloads, signalling NaN,
   +-inf, -0, denormals; random integers) and FCU frames encoded by both (72 bytes equal, the FCU half's 62
   padding zeros included), and random exchange buffers through both decoders (valid frames of either side,
   bit flips, byte substitutions, shifts by a lost / extra byte, swapped TYPEs, echoed bytes, random tails of the
   FCU half, random bytes): validity of nl_decode_nav AND nl_decode_fcu and every decoded field (floats as bit
   patterns) equal.
7. **Estimator** (``sim.estimator.PadAlignGyroEstimator``) and **altitude filter**
   (``AltitudeFilter``) sequences, and a log-format round trip: ``test_core --log-sample`` writes every record
   type of log format 4 (boot 'B' with role FCU, text, 3 'F', 'E', 'L', 'S' with every field set to a distinct
   value and a real RSSI, 'C' = one IL_STAT) with junk between frames; ``decode_log.py`` must return every field
   as written.

v0.7 / v0.7.1 VTOL build (SPEC 8.9; own random seeds, run after the sections above, which keep their v0.6.3 draws - the
ballistic sections use ``mode='ballistic'`` parameters and the v0.6.3 stream generator byte for byte; v0.7.1 only adds
the FCU frame's thr_q byte to the nlink section's decode comparison, 0 in every v0.6.3 frame, and appends its own
thr_q stream after the v0.6.3 draws):

4b. **VTOL supervisor** - ``sim.onboard.OnboardSupervisor`` (VTOL) vs ``onboard.c`` (``--onb-vectors``, ``#S ... mode=1``) on
   (a) random sequences (``VtolRandomOperator``: ON / OFF / UP / DOWN / KILL, the ballistic build's codes, unknown codes,
   wrong nonces, repeated / old / half-space SEQs, several frames a tick, slow polls, outages; arm switch, IMU, baro,
   tilt, rate, spin, gyro saturation, low cell, altitude spikes above the ceiling and the geofence, falls, a tilt / sag
   at the end of the spool, faults while LANDED; altitude and the controller flags from a 1-D plant flown by the Python
   vertical controller, plus injected spurious at_target / touchdown flags; v0.7.1: the ground signature and
   low_in_land_ticks of that controller or injected, scripted baro losses with short time limits; shortened timers, low
   altitude limits, I10 boots) and (b) the supervisor inputs of every stack replay of 4d (with the Python controller's
   flags).  Every tick identical: state, phase, target (dm), motor_on, buzzer, baro_lost, control / release, reason,
   flags, ack, nonce, reply SEQ, the 31 TELEM2 bytes, executed / rejected commands with their phases, and the log of
   state transitions and phase changes; every VTOL interlock (V1-V10, I1p, I2, I3, I8, I9 IMU / baro, arm switch) and
   rejection code must occur, and the v0.7.1 LANDED rules (I9 IMU, V4, V5 -> SAFE) - with no LANDED -> ABORT other than
   by KILL.
4c. **Vertical controller v2** - ``sim.vertical_ctrl`` (float64) vs ``vertical_ctrl.c`` (float32, ``--vc-vectors``) on
   random phase scripts (take-offs, target changes, landings incl. baro-lost ones, restarts) flown by the reference on a
   1-D plant (ground below / at / above the pad, NAV altitude offsets, the plant's body-z specific force or none), with
   noise, h / v_z jumps, tilts to 80 deg as NAV quaternions, NaN / inf inputs and parameter variants (gains, ramp,
   confirm / floor times, support threshold, still lead, the feed-forward off; v0.7.1b: plants x0.9-1.35 of the thrust
   model, the thrust learning off / tau 0.5 s / 2 s, its tilt guard 5 / 15 deg, k_T frozen at a clamp); the C gets every
   parameter of the reference (vc_c_overrides).  Compared: the outputs, the signature flags (v0.7.1b: + low_alt,
   kt_learn), td_by, the five counters (floor run, still / support / floor / all-three), low_in_land, the exact integer
   reference and (v0.7.1b) k_T; touchdowns by each of still / support / floor / all must occur and k_T must be learnt.
   v0.7.1b replay rules (justification in _vc_row and vc_prop_step): each tick the C starts from the reference's k_T
   (k_sync - a replay holds f_z fixed, so a free-running k_T difference would grow wherever u < u_ff), and r_thr /
   d_lpf are compared net of the part their inputs' differences (u within VC_U_TOL, k_T) carry through their lags.
4d. **FCU stack replay** - the supervisor inputs of 1-D closed-loop flights (this file) and of 6-DOF VTOL runs
   (``sim/vtol_scenarios.py`` scenarios incl. the swinging wind and the demo profile's push and baro-offset runs,
   ``FlightSim`` mode 'vtol', recorded at ``FcuStack.step`` in a child process from --sim-root, a snapshot of sim/ when
   given, with the run's own SupervisorParams / VerticalParams) into ``sim.onboard.FcuStack`` and ``onb_stack_step``
   (``--stack-vectors``, v0.7.1 tick order): the supervisor outputs identical (TELEM2 throttle - the previous tick's
   float32 vs float64 controller output - within one count), the controller as in 4c (k_sync, the propagated r_thr /
   d_lpf parts), the attitude controller's inputs of each tick (the previous tick's control_active and ESC throttle)
   within VC_U_TOL.
3b. **Attitude controller, VTOL gains** (v0.7.1b) - the controller section's comparison (DELTA_BOUND_RAD) with the VTOL
   build's gains: ``sim.controller.gains_for_mode(..., 'vtol')`` (params.json control.vtol_overrides on top of the tuned
   gains) vs ``test_core --vectors-vtol`` (core/fc_config.c fc_vtol_gains), on random sequences of its own seed and
   yaw-rate wind-up sequences; rows with the z integrator beyond the ballistic limit must occur.
5b. **Link protocol v2** - TELEM2 (TYPE 0x21, 31 B, LEN_MAX 34) and the codes 8..11 through both decoders with every
   accept mask, TELEM2 encodings.  6b. **ilink SNAP2** - SNAP2 streams with the VTOL COM's mask, encodings, the FCU's
   SNAP2 and the VTOL COM's SNAP2 -> TELEM2 path.
7b. **VTOL NAV estimator and altitude filter** (v0.7.1) - ``sim.estimator.GatedMahonyEstimator`` vs ``att_mahony_*``
   (``--mahony-vectors``) and ``sim.state_machine.VtolAltitudeFilter`` vs ``alt_vtol_*`` (``--altv-vectors``) on synthetic
   sensor streams (pad alignment, drag-balanced hovers with the gate open, vertical accelerations with it closed,
   vibration, gyro-bias wander, re-entry to the pad; accelerometer offsets up to beyond the bias clamp, baro noise,
   thrust-dependent baro errors with and without the compensation, baro outages, a return to the pad) and on the NAV's
   own calls recorded in the 6-DOF runs of 4d (the objects' constants, float32-rounded for both sides); bounds in
   check_mahony / check_altv.

v0.8 (GPS position hold, the VTOL build with gps_hold; a v0.8 params.json): sections 4b-4d run the v0.7.1 VTOL build of
the same params.json on both sides (SupervisorParams gps=False / takeoff_check=False, the vertical controller's take-off
check off: gps_hold=0 takeoff_check=0 vc.takeoff_ticks=0 for the C); 7b also compares the published prediction
(att_mahony_predict vs GatedMahonyEstimator.predict: q_pub within MAHONY_Q_TOL, rate within MAHONY_RATE_TOL) on every
step; 3b adds tilt-setpoint sequences (attitude_ctrl_set_tilt in its two background halves, the second one by
attitude_ctrl_set_tilt_finish or left to the update, vs AttitudeController.tilt_sp) under the same DELTA_BOUND_RAD.  Own
sections (their seeds after the v0.7 ones):

8a. **Position controller** - ``sim.position_ctrl.PositionHold`` (float64, the firmware's float32 constants: PosParams32)
   vs ``core/position_ctrl.c`` through its background pieces P0..P10 (``--pos-vectors``) on random histories (phase
   scripts, altitudes through h_enable / flare / fade, k_T across the thrust-limited / capped regions, tilts and rates
   around the guard, saturation, slot dropouts, FIX_OK loss, NAV resets, drifts to the fence, plausibility runs;
   parameter variants).  Discrete outputs equal every step; floats within the bounds of the POS_* block; the a-limit
   tie |a| == a_lim (by design while the vertical loop is saturated high) may go the other way in float32 - that
   sequence is compared up to it (counted, <= 1 in 10 sequences).  Every mode, every request, limiting, guard and
   implausibility must occur.
8b. **NAV GPS side** - ``sim.gps_nav.NavGps`` (in closed loop with ``UbxReceiverStub``) vs ``gps_nav_tick``
   (``--gpsnav-vectors``) on the same UART bytes and IMU samples: boot configuration from 38400 / the factory 9600, a
   NAK, a silent receiver, a u-blox 8 MON-VER, no configurator; a truth trajectory with lever arm and heading error;
   faults (position / velocity outlier bursts -> rejections and resets, poor and late epochs, outages, corrupted /
   repeated bytes, garbage, wrong-baud bytes).  Integers equal every tick, the float state at every step within the
   GPS_* bounds, the next frame's slot equal (or one count at a rounding boundary of the reference's value).
8c. **v0.8 supervisor rules** - random sequences of 4b's operator with the GPS inputs (ON gate, LAND requests,
   take-off check, TELEM v3 fields), scripted clean flights: every tick identical incl. the 37 TELEM v3 bytes; REJ_GPS
   and every V11-V15 reason must occur.
8d. **FCU stack with position hold** - 3-D point-mass closed loops of the v0.8 ``FcuStack`` (attitude lag, wind and
   pushes, a NAV snapshot model in nlink v3 slots; outage, NAV reset, lost frames, fence, authority) replayed into
   ``onb_stack_step`` (rt part + every background piece): 4d's comparison plus the slot receiver, the published
   position outputs, the tilt builder, the attitude tilt and (on commit ticks) the position state; TELEM v3 dist may
   be one 0.1 m count off at a rounding boundary of the reference's distance (counted); the supervisor inputs the
   Python stack gave its supervisor through both supervisors.
8e. **v0.8 wire formats** - TELEM3 (link decoder streams with every accept mask that has it, encodings), SNAP3 (ilink
   streams with ALL8 / the VTOL + GPS COM / FCU masks, re-encodings), nlink v3 (encodings of random fields as raw float
   patterns incl. NaN payloads, decodes of corrupted / foreign / random buffers) and random slot sequences through
   both slot receivers: bytes, fields and counters equal.

Vertical-controller bounds (4c / 4d), justification (v0.7.1): the altitude reference is an exact int32 in both languages
(v0.7.0's float32 h_ref sum and its knife-edge arrival ticks are gone); h_ref = ref x 1e-6 within 2 ulp (5e-7 m at 5 m).
The float32 port differs from the float64 reference by per-tick rounding of u_ff (sqrtf, a division: <= 2 ulp of 1 =
2.4e-7) and of vz_kp e (|e| <= 2.5 m/s, 1 ulp 2.4e-7), the integrator (<= 1/2 ulp(vz_i_max) = 7.5e-9 per tick, worst
case linear 1.1e-4 over 15000 ticks, typically ~1e-6) and three contractive low-passes (v_lpf, d_lpf, r: their own
VC_*_TOL lines).  Tolerances: h_ref 1e-4 m, v_z command / error 1e-4 m/s x max(1, alt_kp), throttle / u_ff / integrator
/ ESC 2e-4 (the worst-case integrator drift with margin; 2/5 of one ESC PWM count, 0.5 us of the 1000 us throttle
range).  A discrete decision on floats (the still lead, the command's downward saturation, still, support, the
integrator clamp, conditional integration, saturation) whose float64 margin is below its VC_AMB_* band may go the other
way in float32: when C and reference then differ, that tick is a knife-edge decision - counted by kind, the next
VC_RESYNC_TICKS ticks are not compared and the following tick must agree again (a lasting branch, e.g. a counter one
tick apart, ends that sequence's comparison and is counted).  Any difference at a tick without such a margin fails.

Error bound for the controller (DELTA_BOUND_RAD = 1e-4 rad), justification: float32 unit round-off
is u = 2^-24 = 6.0e-8.  The path from inputs to delta is ~60 flops plus atan2f/sqrtf (<= 2 ulp), so a
stateless step contributes <~ 1e-6 relative.  Stateful parts: the LPF states are contractive (error
stays a few ulp); the D term divides a w_d rounding error (ulp(4 rad/s) = 2.4e-7) by dt = 4 ms and
multiplies by Kd ~ 0.03 -> 2e-6 N m; the integrator accumulates at most one rounding of ulp(i_max)
(3.6e-8 N m) per step -> 1.8e-5 N m worst case over 500 steps.  The smallest moment gain is the roll
channel at the thrust floor, 4 r_v k_floor / roll_priority_scale ~ 0.66 N m/rad, so the worst-case
delta error is ~ 3e-5 rad.  1e-4 rad leaves 3x margin over that worst case and is 1/8 of one servo
PWM count (0.5 us at 11.1 us/deg = 7.9e-4 rad): the float32 controller can never move a servo by
more than the quantisation of the timer that drives it.  A sequence in which float32 and float64
legitimately disagree on the discrete saturation flag (|margin| < 1e-5 rad) is reported and
excluded from that step on (the integrator then follows a different branch); any other flag
mismatch fails.

Run from the project root: ``.venv/Scripts/python.exe tools/crosscheck_controller.py``.
Writes ``firmware/atmega128_fc/host_test/build/crosscheck_report.json``.
"""
from __future__ import annotations

import argparse
import dataclasses
import glob
import json
import math
import os
import random
import re
import struct
import subprocess
import sys
import time

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
TOOLS = os.path.join(ROOT, "tools")
for _p in (ROOT, os.path.join(ROOT, "cad"), TOOLS, os.path.join(ROOT, "firmware", "atmega128_fc", "tools")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import build_host_test as bht  # noqa: E402
import decode_log  # noqa: E402
import gen_firmware_params as gfp  # noqa: E402
import gen_gains_header as ggh  # noqa: E402
from sim import glink_proto as gl  # noqa: E402
from sim import gps_nav as gpn  # noqa: E402
from sim import ilink_proto as il  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim import nlink_proto as nl  # noqa: E402
from sim import onboard as onb  # noqa: E402
from sim import position_ctrl as pcm  # noqa: E402
from sim import quat  # noqa: E402
from sim import vertical_ctrl as vcm  # noqa: E402
from sim.controller import AttitudeController, MixerPlant, gains_for_mode, vtol_overrides  # noqa: E402
from sim.sim6dof import FlightSim, Scenario, SensorConfig  # noqa: E402
from sim.estimator import GatedMahonyEstimator  # noqa: E402
from sim.state_machine import AltitudeFilter, VtolAltitudeFilter  # noqa: E402
from sim.vehicle import load_vehicle  # noqa: E402
from sim.vertical_ctrl import VerticalController, VerticalInputs, VerticalParams  # noqa: E402

DELTA_BOUND_RAD = 1e-4
SAT_MARGIN_AMBIGUOUS = 1e-5
EST_Q_BOUND = 1e-4
EST_BIAS_BOUND = 1e-5
ALT_H_BOUND_M = 5e-3
ALT_V_BOUND_MPS = 5e-3
FLOAT_BAND_REL = 1e-5            # I3 / I4 inputs this close (relative) to the threshold are moved out of the band
BUILD = os.path.join(ROOT, "firmware", "atmega128_fc", "host_test", "build")
TRANS_RE = re.compile(r"^(\w+) -> (\w+): ")
STATE_NAMES = {s.name: int(s) for s in onb.State}


def f32(x: float) -> float:
    return float(np.float32(x))


def r32(x: float) -> str:
    """Exact text of the float32 value of x (repr of the double that equals it)."""
    return repr(float(np.float32(x)))


def run_exe(args: list[str]) -> None:
    r = subprocess.run([bht.EXE, *args], cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        raise SystemExit(f"test_core {' '.join(args)} failed ({r.returncode}): {r.stdout} {r.stderr}")


def read_csv(path: str) -> tuple[list[str], np.ndarray]:
    with open(path, "r", encoding="utf-8") as f:
        head = f.readline().strip().split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return head, data


# --------------------------------------------------------------------------- #
# reference objects
# --------------------------------------------------------------------------- #
class Ref:
    def __init__(self) -> None:
        self.d = gfp.derive()
        self.p = self.d["p"]
        self.veh = load_vehicle()
        self.ginfo = ggh.resolve()
        self.gains = self.ginfo["gains"]
        v = self.veh
        self.plant = MixerPlant(
            l_v_m=v.l_v_m, r_v_m=v.r_v_m, s_v_m2=v.s_v_m2, cl_alpha=float(self.p["vane"]["cl_alpha_per_rad"]),
            a_e_m2=v.a_e_m2, t_max_n=v.t_max_n, throttle_exp=float(self.p["edf"]["throttle_exponent"]),
            delta_max_rad=math.radians(self.p["vane"]["max_deflection_cmd_deg"]),
            pwm_center_us=float(self.p["servo"]["pwm_center_us"]), pwm_us_per_deg=float(self.p["servo"]["pwm_us_per_deg"]),
            servo_signs=tuple(self.d["signs"]), servo_trims_us=tuple(self.d["trims"]))
        self.loop_hz = int(self.p["control"]["loop_hz"])
        self.dt = 1.0 / self.loop_hz

    def controller(self) -> AttitudeController:
        return AttitudeController(self.p["control"], self.plant, self.gains)


def check_plant(ref: Ref) -> dict:
    d, v = ref.d, ref.veh
    pairs = {"l_v_m": (d["l_v"], v.l_v_m), "r_v_m": (d["r_v"], v.r_v_m), "s_v_m2": (d["s_v"], v.s_v_m2),
             "a_e_m2": (d["a_e"], v.a_e_m2), "t_max_n": (d["t_max"], v.t_max_n), "z_cg_m": (d["z_cg"], v.z_cg_m),
             "mass_kg": (d["m_full"], v.full.mass_kg)}
    out = {}
    for k, (a, b) in pairs.items():
        rel = abs(a - b) / max(abs(b), 1e-30)
        out[k] = {"generator": a, "sim_vehicle": b, "rel_diff": rel}
        if rel > 1e-12:
            raise SystemExit(f"plant mismatch {k}: generator {a!r} vs sim.vehicle {b!r}")
    out["mass_source"] = v.mass_source
    return out


# --------------------------------------------------------------------------- #
# recorded 6-DOF runs (closed loop: ground station + link + onboard supervisor)
# --------------------------------------------------------------------------- #
def recorded_runs(ref: Ref) -> list[dict]:
    """Nominal flight, vane failure (I3/I2 abort), operator KILL, long pad time; the supervisor's inputs are
    captured tick by tick (OnboardSupervisor.update wrapped on the instance)."""
    a = ref.p["aero"]
    stuck = float(ref.p["vane"]["max_deflection_mech_deg"])
    specs = [
        Scenario(name="rec_nominal", t_end=150.0, seed=22, wind_mps=float(a["wind_mps_default"]),
                 gust_mps=float(a["gust_mps_default"]), gust_t0_after_launch_s=5.0, gust_duration_s=float(a["gust_duration_s"])),
        Scenario(name="rec_vane_fail", t_end=60.0, seed=44, vane_failure_index=0, vane_failure_t_after_launch_s=2.0,
                 vane_failure_stuck_deg=stuck),
        Scenario(name="rec_kill", t_end=60.0, seed=5, wind_mps=float(a["wind_mps_default"]), kill_time_s=4.0),
        Scenario(name="rec_pad_long", t_end=170.0, seed=66, arm_time_s=2.0, launch_time_s=12.0,
                 wind_mps=float(a["wind_mps_default"])),
    ]
    runs = []
    for sc in specs:
        t0 = time.perf_counter()
        sim = FlightSim(ref.veh, sc)
        captured: list = []
        orig = sim.sup.update

        def rec(inp, _orig=orig, _cap=captured):
            out = _orig(inp)
            _cap.append((inp, int(out.state)))
            return out

        sim.sup.update = rec
        res = sim.run()
        lseed = sc.seed if sc.link_seed is None else sc.link_seed
        runs.append(dict(name=sc.name, sc=sc, log=res.log, n=len(res.log["t"]), events=res.events,
                         wall=time.perf_counter() - t0, dt=sim.dt_c, sup_inputs=captured,
                         sup_seed=(lseed * 40503 + 12345) & 0xFFFF))
    return runs


# --------------------------------------------------------------------------- #
# attitude controller vectors
# --------------------------------------------------------------------------- #
def random_controller_sequences(rng: np.random.Generator, n_seq: int, dt: float) -> list[list[dict]]:
    seqs = []
    for s in range(n_seq):
        n = int(rng.integers(60, 160)) if s % 10 else int(rng.integers(400, 600))
        tilt0 = math.radians(rng.uniform(0.0, 35.0 if s % 7 else 70.0))
        ax = rng.uniform(0, 2 * math.pi)
        psi = rng.uniform(-math.pi, math.pi)
        q = quat.multiply(quat.from_yaw(psi), quat.from_axis_angle(np.array([math.cos(ax), math.sin(ax), 0.0]), tilt0))
        # yaw reference: heading at "arming", within +-150 deg of the current heading
        yq = quat.from_yaw(psi + math.radians(rng.uniform(-150.0, 150.0)))
        w = rng.normal(0.0, 1.0, 3) * np.array([1.0, 1.0, 0.5])
        bias = rng.normal(0.0, math.radians(0.5), 3)
        mode = s % 5
        u = rng.uniform(0.0, 1.0)
        rows = []
        for k in range(n):
            # mean-reverting random body rate with a restoring term so tilt stays plausible
            tv = quat.tilt_vector(q)
            w = 0.97 * w + rng.normal(0.0, 0.35, 3) * np.array([1.0, 1.0, 0.5]) - 0.8 * dt * 25.0 * np.array([tv[0], tv[1], 0.0])
            w = np.clip(w, -6.0, 6.0)
            q = quat.integrate(q, w, dt)
            if mode == 0:
                u_k = u                                         # constant
            elif mode == 1:
                u_k = min(1.0, k / max(n * 0.5, 1))             # ramp 0 -> 1 (crosses the thrust floor)
            elif mode == 2:
                u_k = 0.0 if (k // 40) % 2 else 1.0             # steps between floored and full
            elif mode == 3:
                u_k = float(np.clip(0.9 + 0.08 * math.sin(0.05 * k) + rng.normal(0, 0.01), 0.0, 1.0))
            else:
                u_k = rng.uniform(0.0, 1.0)
            qq = q if (k % 17) else -q                          # the other sign of the same attitude
            gyro = w + bias + rng.normal(0.0, 0.01, 3)
            rows.append(dict(reset=1 if k == 0 else 0, set_yaw=1 if k == 0 else 0, yq=yq, q=qq, gyro=gyro, u=u_k))
        seqs.append(rows)
    return seqs


def tilt_controller_sequences(rng: np.random.Generator, n_seq: int, dt: float) -> list[list[dict]]:
    """v0.8: random_controller_sequences with a tilt setpoint on every row (the position hold's builder output: slewing
    at up to 0.36 deg per tick to 0.35 rad, held, back to (0, 0) and on again); the setter's second half run by
    attitude_ctrl_set_tilt_finish or left to the update (random), the yaw reference re-set during a tilt now and then."""
    seqs = random_controller_sequences(rng, n_seq, dt)
    for rows in seqs:
        t = np.zeros(2)
        tgt = rng.uniform(-0.35, 0.35, 2)
        for k, r in enumerate(rows):
            if rng.random() < 0.03:
                tgt = np.zeros(2) if rng.random() < 0.3 else rng.uniform(-0.35, 0.35, 2)
            d = tgt - t
            n = float(np.linalg.norm(d))
            if n > 0.00628:
                d *= 0.00628 / n
            t = t + d
            r["tilt"] = (float(t[0]), float(t[1]))
            r["finish"] = int(rng.random() < 0.6)
            if k > 0 and rng.random() < 0.01:
                r["set_yaw"] = 1
    return seqs


def recorded_controller_sequences(runs: list[dict]) -> list[list[dict]]:
    seqs = []
    for run in runs:
        L = run["log"]
        act, st = L["control_active"], L["state"]
        k_arm = None
        k = 0
        while k < run["n"]:
            if st[k] == onb.State.ARMED and (k == 0 or st[k - 1] != onb.State.ARMED):
                k_arm = k
            if act[k] and (k == 0 or not act[k - 1]):
                rows = []
                yq = L["q_est"][k_arm] if k_arm is not None else L["q_est"][k]
                j = k
                while j < run["n"] and act[j]:
                    rows.append(dict(reset=1 if j == k else 0, set_yaw=1 if j == k else 0, yq=yq, q=L["q_est"][j],
                                     gyro=L["gyro"][j], u=float(L["throttle"][j])))
                    j += 1
                seqs.append(rows)
                k = j
            else:
                k += 1
    return seqs


def yaw_windup_sequences(rng: np.random.Generator, n_seq: int, dt: float) -> list[list[dict]]:
    """v0.7.1b: sequences that hold a body-z rate error long enough to wind the z integrator to its limit (the EDF swirl
    case of the VTOL attitude override control.vtol_overrides.rate_i_max_z): nearly upright, a constant yaw-rate offset
    of either sign with noise, hover throttle, 10 s."""
    seqs = []
    for s in range(n_seq):
        q = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(rng.uniform(0.0, 3.0)))
        wz = (1.0 if s % 2 else -1.0) * rng.uniform(0.2, 0.4)
        u = rng.uniform(0.85, 0.95)
        rows = []
        for k in range(2500):
            gyro = np.array([rng.normal(0.0, 0.01), rng.normal(0.0, 0.01), wz + rng.normal(0.0, 0.01)])
            rows.append(dict(reset=1 if k == 0 else 0, set_yaw=1 if k == 0 else 0, yq=quat.from_yaw(0.0), q=q, gyro=gyro, u=u))
        seqs.append(rows)
    return seqs


def py_controller(ref: Ref, seqs: list[list[dict]], vtol: bool = False) -> list[list[dict]]:
    """The reference controller on float32 inputs: the ballistic build's gains (ref.controller(), v0.6.3) or (vtol,
    v0.7.1b) the VTOL build's - sim.controller.gains_for_mode(..., 'vtol'), core/fc_config.c fc_vtol_gains."""
    ctrl = (AttitudeController(ref.p["control"], ref.plant, gains_for_mode(ref.p["control"], ref.gains, lp.MODE_VTOL))
            if vtol else ref.controller())
    out = []
    for rows in seqs:
        o_rows = []
        for r in rows:
            q = np.array([f32(x) for x in r["q"]])
            yq = np.array([f32(x) for x in r["yq"]])
            g = np.array([f32(x) for x in r["gyro"]])
            u = f32(r["u"])
            if r["reset"]:
                ctrl.reset()
            if r["set_yaw"]:
                ctrl.set_yaw_reference(yq)
            if r.get("tilt") is not None:                     # v0.8: the tilt setpoint (float32, level frame)
                ctrl.tilt_sp = (f32(r["tilt"][0]), f32(r["tilt"][1]))
            co = ctrl.update(q, g, u)
            gx, gy, gz = ctrl.mixer.moment_gains(co.thrust_est)
            dx, dy = co.moments[0] / gx, co.moments[1] / gy
            dz = ctrl.mixer.roll_priority_scale * co.moments[2] / gz
            margin = max(abs(dx), abs(dy)) + abs(dz) - ctrl.mixer.delta_max
            o_rows.append(dict(delta=co.delta_cmd, moments=co.moments, rate_sp=co.rate_sp, err=co.att_error,
                               wf=co.gyro_filtered, sat=bool(co.saturated), that=co.thrust_est, integ=co.integrator,
                               pwm=co.pwm_us, margin=margin, floored=ctrl.mixer.thrust_floored(u),
                               clamped=bool(np.any(np.abs(co.rate_sp) >= ctrl.rate_sp_max - 1e-12))))
        out.append(o_rows)
    return out


def check_controller(ref: Ref, seqs: list[list[dict]], kinds: list[str], vtol: bool = False) -> dict:
    os.makedirs(BUILD, exist_ok=True)
    fin, fout = os.path.join(BUILD, "ctrl_in.csv"), os.path.join(BUILD, "ctrl_out.csv")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("seq,reset,set_yaw,yqw,yqx,yqy,yqz,qw,qx,qy,qz,gx,gy,gz,u\n")
        for s, rows in enumerate(seqs):
            for r in rows:
                vals = [str(s), str(r["reset"]), str(r["set_yaw"])] + [r32(x) for x in r["yq"]] + \
                       [r32(x) for x in r["q"]] + [r32(x) for x in r["gyro"]] + [r32(r["u"])]
                if r.get("tilt") is not None:                 # v0.8: tilt setpoint, finish the setter's 2nd half?
                    vals += [r32(r["tilt"][0]), r32(r["tilt"][1]), str(int(r.get("finish", 1)))]
                f.write(",".join(vals) + "\n")
    run_exe(["--vectors-vtol" if vtol else "--vectors", fin, fout])
    _, c = read_csv(fout)
    py = py_controller(ref, seqs, vtol)
    i_lim_ballistic = float(ref.controller().pid.i_max[2])
    i = 0
    stats = {"vectors": 0, "sequences": len(seqs), "max_abs_d_delta_rad": 0.0, "max_abs_d_pwm_us": 0.0,
             "max_abs_d_integ_nm": 0.0, "max_abs_d_moment_nm": 0.0, "max_abs_d_rate_sp": 0.0, "max_abs_d_att_err": 0.0,
             "sat_flag_mismatch_ambiguous": 0, "sat_flag_mismatch_hard": 0, "excluded_rows": 0,
             "rows_saturated": 0, "rows_floored": 0, "rows_rate_sp_clamped": 0, "by_kind": {},
             "rows_integ_z_beyond_ballistic_limit": 0}
    worst = None
    for s, rows in enumerate(py):
        kind = kinds[s]
        ks = stats["by_kind"].setdefault(kind, {"sequences": 0, "vectors": 0, "max_abs_d_delta_rad": 0.0})
        ks["sequences"] += 1
        diverged = False
        for o in rows:
            cr = c[i]
            i += 1
            if int(cr[0]) != s:
                raise SystemExit(f"row order mismatch at C row {i}")
            if diverged:
                stats["excluded_rows"] += 1
                continue
            c_sat = bool(cr[17])
            if c_sat != o["sat"]:
                if abs(o["margin"]) < SAT_MARGIN_AMBIGUOUS:
                    stats["sat_flag_mismatch_ambiguous"] += 1
                    diverged = True
                    stats["excluded_rows"] += 1
                    continue
                stats["sat_flag_mismatch_hard"] += 1
            stats["vectors"] += 1
            ks["vectors"] += 1
            stats["rows_saturated"] += int(o["sat"])
            stats["rows_floored"] += int(o["floored"])
            stats["rows_rate_sp_clamped"] += int(o["clamped"])
            stats["rows_integ_z_beyond_ballistic_limit"] += int(abs(o["integ"][2]) > i_lim_ballistic + 1e-6)
            dd = float(np.max(np.abs(cr[1:5] - o["delta"])))
            if dd > stats["max_abs_d_delta_rad"]:
                stats["max_abs_d_delta_rad"] = dd
                worst = {"seq": s, "kind": kind, "c": cr[1:5].tolist(), "py": list(map(float, o["delta"]))}
            ks["max_abs_d_delta_rad"] = max(ks["max_abs_d_delta_rad"], dd)
            stats["max_abs_d_moment_nm"] = max(stats["max_abs_d_moment_nm"], float(np.max(np.abs(cr[5:8] - o["moments"]))))
            stats["max_abs_d_rate_sp"] = max(stats["max_abs_d_rate_sp"], float(np.max(np.abs(cr[8:11] - o["rate_sp"]))))
            stats["max_abs_d_att_err"] = max(stats["max_abs_d_att_err"], float(np.max(np.abs(cr[11:14] - o["err"]))))
            stats["max_abs_d_integ_nm"] = max(stats["max_abs_d_integ_nm"], float(np.max(np.abs(cr[19:22] - o["integ"]))))
            stats["max_abs_d_pwm_us"] = max(stats["max_abs_d_pwm_us"], float(np.max(np.abs(cr[22:26] - o["pwm"]))))
    if i != len(c):
        raise SystemExit(f"C produced {len(c)} rows, expected {i}")
    stats["worst"] = worst
    stats["bound_rad"] = DELTA_BOUND_RAD
    stats["pass"] = (stats["max_abs_d_delta_rad"] < DELTA_BOUND_RAD and stats["sat_flag_mismatch_hard"] == 0
                     and stats["vectors"] >= 2000 and (not vtol or stats["rows_integ_z_beyond_ballistic_limit"] > 0))
    return stats


# --------------------------------------------------------------------------- #
# onboard supervisor
# --------------------------------------------------------------------------- #
FLOAT_FIELDS = ("tilt_abort_deg", "arm_tilt_max_deg", "min_cell_v", "geofence_m", "apogee_limit_m", "coast_drag_factor",
                "fall_vz_mps", "tilt_rate_abort_rad_s", "spin_rate_abort_rad_s")
C_OVERRIDES = ("ramp_ticks", "coast_hold_ticks", "fall_confirm_ticks", "link_abort_ticks", "link_disarm_ticks",
               "max_motor_ticks", "release_after_cut_ticks", "abort_release_ticks", "rate_persist_ticks", "imu_fail_ticks",
               "baro_fail_ticks", "climb_milli", "tilt_abort_deg", "arm_tilt_max_deg", "min_cell_v", "geofence_m",
               "apogee_limit_m", "fall_vz_mps", "gyro_saturation_abort")


def sup_params(ref: Ref, overrides: dict | None = None) -> onb.SupervisorParams:
    """The reference's parameters with the firmware's float32 thresholds (params_gen.h stores float32); the BALLISTIC
    build (v0.7: params.json mission.mode only picks the default, this section checks the v0.6.3 mission)."""
    sp = onb.SupervisorParams.from_params(ref.p, fc_config_path=gfp.FC_CONFIG, mode=lp.MODE_BALLISTIC)
    kw = {k: f32(getattr(sp, k)) for k in FLOAT_FIELDS}
    for k, v in (overrides or {}).items():
        if k not in C_OVERRIDES:
            raise ValueError(k)
        kw[k] = f32(v) if k in FLOAT_FIELDS else v
    return dataclasses.replace(sp, **kw)


class Nudger:
    """Moves an input out of the float32-vs-float64 band of the two computed thresholds (I3 |w_xy|, I4 apogee)."""

    def __init__(self, sp: onb.SupervisorParams):
        self.sp = sp
        self.count = 0

    def gyro(self, w: list[float]) -> list[float]:
        thr = self.sp.tilt_rate_abort_rad_s
        for _ in range(4):
            wx, wy = f32(w[0]), f32(w[1])
            if abs(math.hypot(wx, wy) - thr) >= FLOAT_BAND_REL * thr:
                break
            w = [w[0] * 1.001 + 1e-3, w[1] * 1.001, w[2]]
            self.count += 1
        return [f32(w[0]), f32(w[1]), f32(w[2])]

    def h(self, h: float, vz: float) -> float:
        lim = self.sp.apogee_limit_m
        for _ in range(4):
            hh, vv = f32(h), f32(vz)
            v = vv if vv > 0.0 else 0.0
            if abs(hh + self.sp.apogee_k * v * v - lim) >= FLOAT_BAND_REL * max(1.0, abs(lim)):
                break
            h += 0.01
            self.count += 1
        return f32(h)


def _frames_cols(frames: list[onb.UplinkFrame]) -> list[str]:
    cols = []
    for i in range(4):
        if i < len(frames):
            fr = frames[i]
            cols += [str(fr.seq), str(fr.cmd.cmd), str(fr.cmd.arg), str(fr.cmd.nonce)]
        else:
            cols += ["0", "0", "0", "0"]
    return cols


def _inp_row(inp: onb.OnboardInputs) -> str:
    g = inp.gyro_rad_s
    vals = [str(inp.tick), str(len(inp.frames))] + _frames_cols(inp.frames) + [
        r32(inp.h_m), r32(inp.vz_mps), r32(inp.tilt_deg), r32(g[0]), r32(g[1]), r32(g[2]), str(int(inp.gyro_saturated)),
        r32(inp.v_cell), str(int(inp.imu_ok)), str(int(inp.baro_ok)), str(int(inp.aligned)), str(int(inp.arm_switch)),
        r32(inp.roll_deg), r32(inp.pitch_deg), r32(inp.yaw_deg), r32(inp.loop_max_us)]
    return ",".join(vals)


def _py_row(sup: onb.OnboardSupervisor, out: onb.OnboardOutputs, n_exec_before: int, t: int) -> dict:
    ex = [(e["seq"], e["cmd"], e["code"], e["state_before"], e["state_after"]) for e in sup.exec_log[n_exec_before:]]
    trans = []
    for e in out.events:
        m = TRANS_RE.match(e)
        if m:
            trans.append((STATE_NAMES[m.group(1)], STATE_NAMES[m.group(2)]))
    return dict(t=t, state=int(out.state), target=sup.target_milli, out=out.throttle_milli, active=int(out.control_active),
                release=int(out.release), reason=out.reason, flags=out.flags, ack=out.ack_seq, nonce=out.nonce,
                reply=-1 if out.reply_seq is None else out.reply_seq,
                telem=out.telemetry.pack().hex() if out.telemetry is not None else "-", exec=ex, trans=trans)


V063_NONCE_COMMANDS = (lp.CMD_LAUNCH, lp.CMD_THROTTLE)   # v0.6.3 link_proto.NONCE_COMMANDS: v0.7 added ON (= code 8,
                                                        # which this operator sends as an unknown code) - pinned so the
                                                        # ballistic sequences keep their v0.6.3 random draws


class RandomOperator:
    """Random ground + random sensors for one supervisor sequence; drives the Python reference as it goes
    (the operator reacts to the reference's state - both sides then get exactly these inputs).  The
    operator mostly tries to fly (ARM, LAUNCH / THROTTLE, then CUT / RELEASE / KILL) and sometimes sends
    whatever does not apply; faults are more likely in flight so every interlock gets its turn."""

    CMD_WEIGHTS = {  # state -> [(cmd, weight)]
        onb.State.SAFE: [(lp.CMD_ARM, 10), (lp.CMD_NONE, 4), (lp.CMD_KILL, 1), (lp.CMD_LAUNCH, 1), (lp.CMD_DISARM, 1),
                         (lp.CMD_THROTTLE, 1)],
        onb.State.ARMED: [(lp.CMD_LAUNCH, 6), (lp.CMD_THROTTLE, 4), (lp.CMD_NONE, 4), (lp.CMD_DISARM, 1), (lp.CMD_RELEASE, 1),
                          (lp.CMD_KILL, 1), (lp.CMD_ARM, 1), (lp.CMD_CUT, 1)],
        onb.State.POWERED: [(lp.CMD_NONE, 14), (lp.CMD_THROTTLE, 3), (lp.CMD_CUT, 1), (lp.CMD_RELEASE, 1), (lp.CMD_KILL, 1),
                            (lp.CMD_ARM, 1), (lp.CMD_LAUNCH, 1)],
        onb.State.UNPOWERED: [(lp.CMD_NONE, 10), (lp.CMD_THROTTLE, 3), (lp.CMD_RELEASE, 2), (lp.CMD_KILL, 1), (lp.CMD_CUT, 1),
                              (lp.CMD_DISARM, 1)],
        onb.State.RELEASED: [(lp.CMD_NONE, 8), (lp.CMD_KILL, 1), (lp.CMD_ARM, 1), (lp.CMD_THROTTLE, 1)],
        onb.State.ABORT: [(lp.CMD_NONE, 8), (lp.CMD_KILL, 1), (lp.CMD_RELEASE, 1), (lp.CMD_THROTTLE, 1), (lp.CMD_ARM, 1)],
    }
    FAULT_P = {"arm": 0.0002, "imu": 0.0003, "baro": 0.0002, "tilt": 0.0003, "rate": 0.0004, "sat": 0.0002,
               "spin": 0.0003, "vcell": 0.0003}

    def __init__(self, rng: random.Random, sp: onb.SupervisorParams, nudger: Nudger):
        self.r = rng
        self.sp = sp
        self.nd = nudger
        r = rng
        self.poll = max(1, int(sp.link_disarm_ticks * r.choice([0.08, 0.15, 0.3, 0.45, 0.7, 1.3])))
        self.loss = r.choice([0.0, 0.05, 0.15, 0.4])
        self.seq = r.randrange(256)
        self.pending: lp.Command | None = None
        self.next_poll = r.randrange(1, 30)
        self.outage_until = -1
        self.aligned_from = r.choice([0, 0, r.randrange(0, 400)])
        self.h, self.vz = 0.0, 0.0
        self.tilt_max = r.choice([4.0, 4.0, 4.0, 8.0])
        self.tilt = r.uniform(0.0, self.tilt_max)
        self.vcell = r.uniform(3.4, 4.2)
        self.fault = {k: 0 for k in self.FAULT_P}
        self.climb_a = r.uniform(4.0, 25.0)
        self.clean = r.random() < 0.35                              # a clean flight: no faults, the operator waits
        self.flight_fault_gain = 0.0 if self.clean else r.choice([1.0, 5.0, 20.0])
        self.arm_closed_from = r.choice([0, 0, 0, r.randrange(0, 800)])
        self.vcell_low_until = r.choice([0, 0, 0, r.randrange(0, 800)])

    def _cmd(self, state: onb.State, nonce: int) -> lp.Command:
        r = self.r
        if r.random() < 0.01:
            return lp.Command(r.choice([8, 9, 0x7F, 0xFF]), 0, 0)
        choices = self.CMD_WEIGHTS[state]
        if self.clean and state in (onb.State.POWERED, onb.State.UNPOWERED) and r.random() < 0.97:
            choices = [(lp.CMD_NONE, 1)]
        c = r.choices([c for c, _ in choices], [w for _, w in choices])[0]
        arg = 0
        if c == lp.CMD_THROTTLE:
            arg = r.choice([0, 1, r.randrange(0, 1001), 1000, r.randrange(1000, 65536), self.sp.climb_milli])
        good = r.random() < 0.75
        n = nonce if good else (nonce ^ (1 + r.randrange(65535)))
        return lp.Command(c, arg, n if c in V063_NONCE_COMMANDS or r.random() < 0.3 else r.randrange(65536))

    def frames(self, t: int, sup: onb.OnboardSupervisor) -> list[onb.UplinkFrame]:
        r = self.r
        if r.random() < 0.0004:                                    # a total outage
            self.outage_until = t + int(self.sp.link_disarm_ticks * r.choice([0.5, 1.1, 2.5, 6.0]))
        if t < self.next_poll:
            return []
        self.next_poll = t + max(1, self.poll + r.randrange(-3, 4))
        if t < self.outage_until or r.random() < self.loss:
            return []
        out = []
        # ack bookkeeping: the pending command is done once the vehicle processed its SEQ
        if self.pending is not None and sup.has_seq and sup.last_seq == self.seq and r.random() < 0.85:
            self.pending = None
        if self.pending is None and r.random() < 0.5:
            self.seq = (self.seq + 1) & 0xFF
            self.pending = self._cmd(sup.state, sup.nonce)
        cmd = self.pending if self.pending is not None else lp.Command(lp.CMD_NONE)
        if cmd.cmd in V063_NONCE_COMMANDS and cmd.nonce != sup.nonce and r.random() < 0.5:
            cmd = lp.Command(cmd.cmd, cmd.arg, sup.nonce)          # the ground re-sends with the latest nonce
            self.pending = cmd
        seq = self.seq
        x = r.random()
        if x < 0.02:
            seq = (seq - r.randrange(1, 200)) & 0xFF                # an old / far SEQ
        elif x < 0.03:
            seq = (seq + 128) & 0xFF                                # exactly half the space away: not newer
        out.append(onb.UplinkFrame(seq, cmd))
        if r.random() < 0.03:                                        # several frames in one tick
            for _ in range(r.randrange(1, 4)):
                if r.random() < 0.5:
                    out.append(onb.UplinkFrame(seq, cmd))
                else:
                    self.seq = (self.seq + 1) & 0xFF
                    out.append(onb.UplinkFrame(self.seq, self._cmd(sup.state, sup.nonce)))
        return out[:4]

    def sensors(self, t: int, state: onb.State) -> dict:
        r, sp, fl = self.r, self.sp, self.fault
        dt = 1.0 / sp.loop_hz
        gain = self.flight_fault_gain if state in (onb.State.POWERED, onb.State.UNPOWERED) else 1.0
        for k, p in self.FAULT_P.items():
            if fl[k] > 0:
                fl[k] -= 1
            elif r.random() < p * gain:
                fl[k] = {"arm": r.randrange(1, 40), "imu": r.randrange(1, 2 * sp.imu_fail_ticks + 2),
                         "baro": r.randrange(1, 2 * sp.baro_fail_ticks + 2), "tilt": r.randrange(1, 30),
                         "rate": r.randrange(1, 2 * sp.rate_persist_ticks + 2), "sat": r.randrange(1, 2 * sp.rate_persist_ticks + 2),
                         "spin": r.randrange(1, 2 * sp.rate_persist_ticks + 2), "vcell": r.randrange(1, 30)}[k]
        # vertical motion: climbs while POWERED, ballistic otherwise, ground at 0
        if state == onb.State.POWERED:
            a = self.climb_a
        elif self.h > 0.0 or self.vz > 0.0:
            a = -9.8 if (state != onb.State.RELEASED or self.vz > -6.0) else 0.0
        else:
            a = 0.0
        self.vz += a * dt + r.gauss(0.0, 0.05)
        self.h += self.vz * dt
        if self.h < 0.0:
            self.h, self.vz = 0.0, 0.0
        h = self.h + (r.choice([20.0, 60.0, 200.0]) if r.random() < 0.0005 * gain else 0.0)   # baro glitch
        self.tilt = min(max(self.tilt + r.gauss(0.0, 0.2), 0.0), self.tilt_max)
        tilt = self.tilt if not fl["tilt"] else r.uniform(sp.tilt_abort_deg - 3.0, 95.0)
        if r.random() < 0.002:
            tilt = r.choice([sp.arm_tilt_max_deg, sp.tilt_abort_deg,
                             f32(np.nextafter(np.float32(sp.arm_tilt_max_deg), np.float32(0)))])
        thr = sp.tilt_rate_abort_rad_s
        w = [r.gauss(0.0, 0.5), r.gauss(0.0, 0.5), r.gauss(0.0, 1.0)]
        if fl["rate"]:
            ang = r.uniform(0, 2 * math.pi)
            mag = thr * r.uniform(0.95, 1.6)
            w[0], w[1] = mag * math.cos(ang), mag * math.sin(ang)
        if fl["spin"]:
            w[2] = r.choice([-1, 1]) * sp.spin_rate_abort_rad_s * r.uniform(0.97, 1.3)
        w = self.nd.gyro(w)
        self.vcell = min(max(self.vcell + r.gauss(0.0, 0.002), sp.min_cell_v + 0.02), 4.2)
        vcell = self.vcell if not fl["vcell"] else r.choice([sp.min_cell_v, sp.min_cell_v - 0.01, sp.min_cell_v - 0.4, 0.0])
        if t < self.vcell_low_until:
            vcell = sp.min_cell_v - 0.05
        hh = self.nd.h(h, self.vz)
        return dict(h_m=hh, vz_mps=f32(self.vz), tilt_deg=f32(tilt), gyro_rad_s=tuple(w), gyro_saturated=bool(fl["sat"]),
                    v_cell=f32(vcell), imu_ok=not fl["imu"], baro_ok=not fl["baro"], aligned=t >= self.aligned_from,
                    arm_switch=not fl["arm"] and t >= self.arm_closed_from, roll_deg=f32(r.uniform(-180, 180)), pitch_deg=f32(r.uniform(-90, 90)),
                    yaw_deg=f32(r.uniform(-180, 180)), loop_max_us=float(r.choice([0, r.randrange(0, 4000), 65535, 70000.5])))


def onb_random_sequences(ref: Ref, n_seq: int, seed: int) -> list[dict]:
    rng = random.Random(seed)
    seqs = []
    for s in range(n_seq):
        short = s % 3 != 0
        ov: dict = {}
        if short:                                                     # shortened timers: dense events
            r = rng
            ov = dict(ramp_ticks=r.choice([0, 1, 7, 30]), coast_hold_ticks=r.choice([0, 3, 40]),
                      fall_confirm_ticks=r.choice([1, 5, 30]), link_disarm_ticks=r.choice([40, 90, 140]),
                      max_motor_ticks=r.choice([150, 400, 900]), release_after_cut_ticks=r.choice([0, 5, 30]),
                      abort_release_ticks=r.choice([0, 3, 40]), rate_persist_ticks=r.choice([1, 3, 6]),
                      imu_fail_ticks=r.choice([1, 4, 12]), baro_fail_ticks=r.choice([2, 10, 40]),
                      climb_milli=r.choice([1000, 700, 1]))
            ov["link_abort_ticks"] = ov["link_disarm_ticks"] + r.choice([1, 60, 200])
            if r.random() < 0.3:
                ov["gyro_saturation_abort"] = False
            if r.random() < 0.4:
                ov["apogee_limit_m"] = r.choice([30.0, 80.0, 400.0])
                ov["geofence_m"] = r.choice([40.0, 120.0, 170.0])
        sp = sup_params(ref, ov)
        nd = Nudger(sp)
        op = RandomOperator(rng, sp, nd)
        reset = rng.random() < 0.06
        nonce_seed = rng.choice([0, 1, 0xFFFF, rng.randrange(65536)])
        sup = onb.OnboardSupervisor(sp, seed=nonce_seed, reset_in_flight=reset)
        n = rng.choice([1500, 3000, 6000]) if short else rng.choice([3000, 8000])
        rows, py = [], []
        for t in range(n):
            frames = op.frames(t, sup)
            inp = onb.OnboardInputs(tick=t, frames=frames, **op.sensors(t, sup.state))
            n0 = len(sup.exec_log)
            out = sup.update(inp)
            rows.append(_inp_row(inp))
            py.append(_py_row(sup, out, n0, t))
        seqs.append(dict(kind="random", seed=nonce_seed, reset=reset, overrides=ov, rows=rows, py=py, nudged=nd.count))
    return seqs


def onb_recorded_sequences(ref: Ref, runs: list[dict]) -> list[dict]:
    """The 6-DOF runs' supervisor inputs (float32-rounded, nudged) replayed into a fresh reference."""
    seqs = []
    for run in runs:
        sp = sup_params(ref)
        nd = Nudger(sp)
        sup = onb.OnboardSupervisor(sp, seed=run["sup_seed"])
        rows, py, same = [], [], 0
        for inp0, st_sim in run["sup_inputs"]:
            g = nd.gyro(list(inp0.gyro_rad_s))
            inp = onb.OnboardInputs(
                tick=inp0.tick, frames=list(inp0.frames)[:4], h_m=nd.h(inp0.h_m, inp0.vz_mps), vz_mps=f32(inp0.vz_mps),
                tilt_deg=f32(inp0.tilt_deg), gyro_rad_s=tuple(g), gyro_saturated=bool(inp0.gyro_saturated),
                v_cell=f32(inp0.v_cell), imu_ok=inp0.imu_ok, baro_ok=inp0.baro_ok, aligned=inp0.aligned,
                arm_switch=inp0.arm_switch, roll_deg=f32(inp0.roll_deg), pitch_deg=f32(inp0.pitch_deg),
                yaw_deg=f32(inp0.yaw_deg), loop_max_us=f32(inp0.loop_max_us))
            n0 = len(sup.exec_log)
            out = sup.update(inp)
            same += int(int(out.state) == st_sim)
            rows.append(_inp_row(inp))
            py.append(_py_row(sup, out, n0, inp.tick))
        seqs.append(dict(kind="recorded:" + run["name"], seed=run["sup_seed"], reset=False, overrides={}, rows=rows, py=py,
                         nudged=nd.count, replay_state_agreement=same / max(1, len(rows))))
    return seqs


def check_onboard(ref: Ref, seqs: list[dict]) -> dict:
    fin, fout = os.path.join(BUILD, "onb_in.txt"), os.path.join(BUILD, "onb_out.csv")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        for i, sq in enumerate(seqs):
            ov = " ".join(f"{k}={(r32(v) if isinstance(v, float) else int(v))}" for k, v in sq["overrides"].items())
            f.write(f"#S {i} {sq['seed']} {int(sq['reset'])} {ov}\n")
            f.write("\n".join(sq["rows"]) + "\n")
    run_exe(["--onb-vectors", fin, fout])
    st = {"ticks": 0, "sequences": len(seqs), "mismatch_ticks": 0, "first_mismatches": [], "groups": {},
          "interlocks_seen": {}, "rejections_seen": {}, "transitions_seen": {}, "commands_executed": {}, "i10_boots": 0,
          "telemetry_frames": 0, "frames": 0, "nudged_inputs": 0, "recorded_replay_state_agreement": {}}
    names = {v: k for k, v in STATE_NAMES.items()}
    with open(fout, "r", encoding="utf-8") as f:
        f.readline()
        c_lines = f.read().splitlines()
    i = 0
    for si, sq in enumerate(seqs):
        g = st["groups"].setdefault(sq["kind"].split(":")[0], {"sequences": 0, "ticks": 0, "mismatch_ticks": 0})
        g["sequences"] += 1
        st["nudged_inputs"] += sq["nudged"]
        st["i10_boots"] += int(sq["reset"])
        if "replay_state_agreement" in sq:
            st["recorded_replay_state_agreement"][sq["kind"]] = round(sq["replay_state_agreement"], 6)
        for o in sq["py"]:
            parts = c_lines[i].split(",")
            i += 1
            if int(parts[0]) != si or int(parts[1]) != o["t"]:
                raise SystemExit(f"onb row order mismatch at {i}: {parts[:2]} vs seq {si} tick {o['t']}")
            c_exec = [] if parts[14] == "-" else [tuple(int(x) for x in e.split(":")) for e in parts[14].split("|")]
            c_trans = [] if parts[15] == "-" else [tuple(int(x) for x in e.split(":")) for e in parts[15].split("|")]
            c = dict(state=int(parts[2]), target=int(parts[3]), out=int(parts[4]), active=int(parts[5]),
                     release=int(parts[6]), reason=int(parts[7]), flags=int(parts[8]), ack=int(parts[9]), nonce=int(parts[10]),
                     reply=int(parts[11]), telem=parts[13],
                     exec=[e[:5] for e in c_exec if e[2] < onb_const("ONB_EXEC_NONE")],    # v0.7 rows add the phases
                     trans=[(e[0], e[1]) for e in c_trans])
            interlock = int(parts[12])
            st["ticks"] += 1
            g["ticks"] += 1
            bad = [k for k in c if c[k] != o[k]]
            if interlock and o["reason"] != interlock:
                bad.append("interlock")
            if bad:
                st["mismatch_ticks"] += 1
                g["mismatch_ticks"] += 1
                if len(st["first_mismatches"]) < 12:
                    st["first_mismatches"].append({"seq": si, "kind": sq["kind"], "tick": o["t"], "fields": bad,
                                                   "c": {k: c[k] for k in bad if k in c}, "py": {k: o[k] for k in bad if k in o}})
            if interlock:
                nm = lp.REASON_NAMES.get(interlock, str(interlock))
                st["interlocks_seen"][nm] = st["interlocks_seen"].get(nm, 0) + 1
            for e in c_exec:
                if e[2] >= 0x40 and e[2] < 0xFE:
                    nm = lp.REASON_NAMES.get(e[2], hex(e[2]))
                    st["rejections_seen"][nm] = st["rejections_seen"].get(nm, 0) + 1
                elif e[2] == 0:
                    nm = lp.CMD_NAMES.get(e[1], str(e[1]))
                    st["commands_executed"][nm] = st["commands_executed"].get(nm, 0) + 1
            for e in c_trans:
                key = f"{names[e[0]]}->{names[e[1]]}"
                st["transitions_seen"][key] = st["transitions_seen"].get(key, 0) + 1
            st["telemetry_frames"] += int(o["telem"] != "-")
            st["frames"] += len(c_exec)
            if int(parts[16]) or any(e[3] or e[4] for e in c_trans) or any(e[5] or e[6] for e in c_exec):
                raise SystemExit(f"ballistic supervisor row with a VTOL phase: {c_lines[i - 1][:200]}")
    if i != len(c_lines):
        raise SystemExit(f"C produced {len(c_lines)} supervisor rows, expected {i}")
    need_il = [lp.REASON_NAMES[k] for k in lp.INTERLOCK_REASONS if k != lp.R_I10_RESET]
    need_rej = [lp.REASON_NAMES[k] for k in lp.REJECT_REASONS]
    st["missing_interlocks"] = [k for k in need_il if k not in st["interlocks_seen"]]
    st["missing_rejections"] = [k for k in need_rej if k not in st["rejections_seen"]]
    st["pass"] = (st["mismatch_ticks"] == 0 and not st["missing_interlocks"] and not st["missing_rejections"]
                  and st["i10_boots"] > 0)
    if st["pass"]:                                   # ~150 MB of vectors: kept only when something failed
        for path in (fin, fout):
            os.remove(path)
    return st


_ONB_CONST: dict[str, int] = {}


def onb_const(name: str) -> int:
    if not _ONB_CONST:
        text = open(os.path.join(ROOT, "firmware", "atmega128_fc", "src", "core", "onboard.h"), encoding="utf-8").read()
        for m in re.finditer(r"#define\s+(ONB_\w+)\s+(0x[0-9A-Fa-f]+|\d+)u?", text):
            _ONB_CONST[m.group(1)] = int(m.group(2), 0)
    return _ONB_CONST[name]


# --------------------------------------------------------------------------- #
# link protocol
# --------------------------------------------------------------------------- #
V063_MAX_PAYLOAD_LEN = 28       # link_proto v0.6.3 (TELEM): the ballistic section's streams keep their v0.6.3 bytes
V063_LEN_MAX = 31               # (v0.7: 31 / 34 with TELEM2; the decoders on both sides use the v0.7 LEN_MAX)


def proto_streams(rng: random.Random, vid: int, n: int) -> list[tuple[int | None, tuple[int, ...], bytes]]:
    """The v0.6.3 stream generator, byte for byte (its constants pinned); TELEM2 streams: check_protocol_v2."""
    out = []
    for s in range(n):
        dec_vid = rng.choice([vid, vid, None, (vid + 1) & 0xFF])
        acc = rng.choice([(lp.TYPE_CMD,), (lp.TYPE_TELEM,), (lp.TYPE_CMD, lp.TYPE_TELEM)])
        parts = []
        for _ in range(rng.randrange(1, 12)):
            k = rng.random()
            veh = vid if rng.random() < 0.8 else rng.randrange(256)
            if k < 0.30:
                b = lp.encode_command(veh, rng.randrange(256), lp.Command(rng.randrange(256), rng.randrange(65536),
                                                                          rng.randrange(65536)))
            elif k < 0.50:
                b = lp.encode_telemetry(veh, rng.randrange(256), lp.Telemetry(
                    rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(65536),
                    rng.randrange(1 << 32), *[rng.randrange(-32768, 32768) for _ in range(5)],
                    *[rng.randrange(65536) for _ in range(4)]))
            elif k < 0.58:                                       # other TYPE / payload length, valid CRC
                pl = bytes(rng.randrange(256) for _ in range(rng.randrange(0, V063_MAX_PAYLOAD_LEN + 1)))
                b = lp.encode_frame(veh, rng.choice([lp.TYPE_CMD, lp.TYPE_TELEM, 0x30, 0x00, 0xFF]), rng.randrange(256), pl)
            elif k < 0.72:                                       # garbage, rich in sync bytes
                b = bytes(rng.choice([0xA5, 0x5A, rng.randrange(256), V063_LEN_MAX, 8, 3]) for _ in range(rng.randrange(1, 40)))
            else:                                                # a corrupted valid frame
                base = bytearray(lp.encode_command(veh, rng.randrange(256), lp.Command(rng.randrange(8), rng.randrange(65536),
                                                                                         rng.randrange(65536)))
                                 if rng.random() < 0.6 else lp.encode_telemetry(veh, rng.randrange(256), lp.Telemetry(
                                     0, 0, 0, 0, 0, rng.randrange(1 << 32), 0, 0, 0, 0, 0, 0, 0, 0, 0)))
                m = rng.random()
                if m < 0.35:
                    j = rng.randrange(len(base))
                    base[j] ^= 1 << rng.randrange(8)
                elif m < 0.5:
                    base = base[:rng.randrange(1, len(base))]
                elif m < 0.65:
                    base[2] = rng.choice([0, 1, 2, V063_LEN_MAX + 1, 0xFF, rng.randrange(lp.LEN_MIN, V063_LEN_MAX + 1)])
                elif m < 0.8:
                    j = rng.randrange(len(base) + 1)
                    base[j:j] = bytes([rng.choice([0xA5, 0x5A, rng.randrange(256)])])
                else:
                    base = bytearray([0xA5]) + base
                b = bytes(base)
            parts.append(b)
        out.append((dec_vid, acc, b"".join(parts)))
    return out


def check_protocol(ref: Ref, seed: int, n_streams: int, n_encode: int, n_quant: int) -> dict:
    rng = random.Random(seed)
    vid = int(ref.p["flight_computer"]["link"]["vehicle_id"])
    streams = proto_streams(rng, vid, n_streams)
    lines, expect = [], []
    for dec_vid, acc, data in streams:
        mask = (1 if lp.TYPE_CMD in acc else 0) | (2 if lp.TYPE_TELEM in acc else 0)
        dec = lp.FrameDecoder(dec_vid, acc)
        frames = dec.feed(data)
        lines.append(f"D {-1 if dec_vid is None else dec_vid} {mask} {data.hex()}")
        exp = "D" + "".join(f" {fr.veh}:{fr.type}:{fr.seq}:{fr.payload.hex()}" for fr in frames)
        exp += " S " + " ".join(str(dec.stats[k]) for k in lp.STAT_KEYS) + " 0"
        expect.append(exp)
    for _ in range(n_encode):
        c = lp.Command(rng.randrange(256), rng.choice([0, 65535, rng.randrange(65536)]), rng.choice([0, 65535, rng.randrange(65536)]))
        veh, seq = rng.randrange(256), rng.randrange(256)
        lines.append(f"C {veh} {seq} {c.cmd} {c.arg} {c.nonce}")
        expect.append("C " + lp.encode_command(veh, seq, c).hex())
        t = lp.Telemetry(rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(65536),
                         rng.choice([0, (1 << 32) - 1, rng.randrange(1 << 32)]), *[rng.randrange(-32768, 32768) for _ in range(5)],
                         *[rng.randrange(65536) for _ in range(4)])
        lines.append(f"T {veh} {seq} " + " ".join(str(getattr(t, k)) for k in bht.TELEM_ORDER))
        expect.append("T " + lp.encode_telemetry(veh, seq, t).hex())
    nq_special = 0
    specials = [0.0, -0.0, float("nan"), float("inf"), float("-inf"), 1e-45, -1e-45, 1e-38, 3.4e38, -3.4e38]
    for k in range(n_quant):
        scale = rng.choice([1, 10, 100, 1000])
        lim = rng.choice([lp.I16, lp.U16])
        m = rng.random()
        if k < len(specials):
            x = specials[k]
            nq_special += 1
        elif m < 0.3:                                             # exact ties n + 1/2 over the scale
            x = (rng.randrange(-70000, 70000) + 0.5) / scale
        elif m < 0.5:                                             # just around a tie
            x = float(np.nextafter(np.float32((rng.randrange(-70000, 70000) + 0.5) / scale),
                                   np.float32(rng.choice([-1e9, 1e9]))))
        else:
            x = rng.uniform(-1, 1) * 10 ** rng.uniform(-6, 6)
        x = f32(x)
        lines.append(f"Q {x!r} {scale} {lim[0]} {lim[1]}")
        expect.append(f"Q {lp.quantize(x, scale, lim)}")
    fin, fout = os.path.join(BUILD, "proto_in.txt"), os.path.join(BUILD, "proto_out.txt")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    run_exe(["--proto-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        got = f.read().splitlines()
    st = {"streams": n_streams, "stream_bytes": sum(len(d) for _, _, d in streams), "encodings": 2 * n_encode,
          "quantisations": n_quant, "mismatches": 0, "first_mismatches": [],
          "py_frames": 0, "py_counters_total": {k: 0 for k in lp.STAT_KEYS}}
    if len(got) != len(expect):
        raise SystemExit(f"proto-vectors: {len(got)} lines, expected {len(expect)}")
    for k, (a, b) in enumerate(zip(got, expect)):
        if a != b:
            st["mismatches"] += 1
            if len(st["first_mismatches"]) < 8:
                st["first_mismatches"].append({"line": k, "input": lines[k][:200], "c": a[:300], "py": b[:300]})
    for dec_vid, acc, data in streams:
        dec = lp.FrameDecoder(dec_vid, acc)
        st["py_frames"] += len(dec.feed(data))
        for key in lp.STAT_KEYS:
            st["py_counters_total"][key] += dec.stats[key]
    st["pass"] = st["mismatches"] == 0 and all(v > 0 for v in st["py_counters_total"].values())
    return st


# --------------------------------------------------------------------------- #
# FCU <-> COM ilink (SPEC 10.4, v0.6)
# --------------------------------------------------------------------------- #
_IL_SPECIAL_BITS = (0x7FC00000, 0xFFC00000, 0x7FC0BEEF, 0x7F800001, 0x7F800000, 0xFF800000, 0x80000000, 0x00000001,
                    0x007FFFFF, 0x80000001, 0x7F7FFFFF, 0x00800000)


def _il_rand_float_bits(rng: random.Random) -> int:
    k = rng.random()
    if k < 0.15:
        return rng.choice(_IL_SPECIAL_BITS)
    if k < 0.3:
        return rng.randrange(1 << 32)                            # any pattern, NaN payloads included
    return il.f32_bits(rng.uniform(-1, 1) * 10 ** rng.uniform(-6, 6))


def _il_rand_payload(rng: random.Random, typ: int) -> bytes:
    if typ == il.TYPE_CMD:
        return il.IlCmd(rng.randrange(256), rng.randrange(256), rng.randrange(65536), rng.randrange(65536),
                        rng.randrange(-128, 128), rng.randrange(-128, 128)).pack()
    if typ == il.TYPE_STAT:
        return il.Stat(rng.randrange(1 << 32), rng.randrange(256), rng.randrange(256), rng.randrange(256),
                       rng.randrange(-128, 128), rng.randrange(-128, 128), rng.randrange(256),
                       *[rng.randrange(65536) for _ in range(13)]).pack()
    ints = (rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(256),
            rng.randrange(65536), rng.randrange(65536), rng.randrange(1 << 32))
    return struct.pack("<BBBBBHHI9I", *ints, *(_il_rand_float_bits(rng) for _ in range(9)))


V063_IL_ACCEPT_ALL = il.ACCEPT_CMD | il.ACCEPT_SNAP | il.ACCEPT_STAT    # v0.6.3 ACCEPT_ALL (7; v0.7: 15 with SNAP2) -
                                                                      # the v0.6.3 section keeps its streams and masks


def ilink_streams(rng: random.Random, n: int) -> list[tuple[int, int, bytes]]:
    """Random byte streams for the ilink decoders: valid frames of every type, garbage rich in sync / LEN / TYPE
    bytes, fake headers that swallow real frames, sync spam (C5 5C LEN TYPE repeated), valid-CRC frames with a wrong
    LEN / TYPE, and corrupted frames (bit flips, cuts, LEN changed, bytes inserted or dropped, doubled sync); max_out
    per stream in 0..8, accept mask ALL / FCU / COM (v0.6.3 direction filter)."""
    out = []
    types = (il.TYPE_CMD, il.TYPE_SNAP, il.TYPE_STAT)
    for _ in range(n):
        max_out = rng.choice([0, 1, 2, 2, 2, 3, 8])
        accept = rng.choice([V063_IL_ACCEPT_ALL, il.ACCEPT_FCU, il.ACCEPT_FCU, il.ACCEPT_COM])
        parts = []
        for _ in range(rng.randrange(1, 14)):
            k = rng.random()
            if k < 0.42:
                typ = rng.choice(types)
                b = il.encode(typ, _il_rand_payload(rng, typ))
            elif k < 0.54:                                       # garbage, rich in protocol bytes
                b = bytes(rng.choice([il.SYNC0, il.SYNC1, il.CMD_LEN, il.SNAP_LEN, il.STAT_LEN, il.TYPE_CMD,
                                      il.TYPE_SNAP, il.TYPE_STAT, rng.randrange(256)]) for _ in range(rng.randrange(1, 60)))
            elif k < 0.62:                                       # a fake header whose body swallows real frames
                typ = rng.choice(types)
                inner = b"".join(il.encode(t, _il_rand_payload(rng, t))
                                 for t in rng.choices([il.TYPE_CMD, il.TYPE_CMD, il.TYPE_STAT], k=rng.randrange(1, 4)))
                b = bytes([il.SYNC0, il.SYNC1, il.PAYLOAD_LEN[typ], typ]) + inner[:rng.randrange(1, len(inner) + 1)]
            elif k < 0.66:                                       # sync spam: valid-looking headers back to back
                typ = rng.choice(types)
                b = bytes([il.SYNC0, il.SYNC1, il.PAYLOAD_LEN[typ], typ]) * rng.randrange(1, 30)
            elif k < 0.70:                                       # valid CRC, LEN / TYPE wrong
                typ = rng.choice([0x00, 0x04, 0xFF, il.TYPE_CMD, il.TYPE_SNAP, il.TYPE_STAT])
                ln = rng.choice([il.CMD_LEN, il.SNAP_LEN, il.STAT_LEN, rng.randrange(0, 60)])
                body = bytes([ln, typ]) + bytes(rng.randrange(256) for _ in range(ln))
                crc = il.crc16(body)
                b = bytes([il.SYNC0, il.SYNC1]) + body + bytes([crc & 0xFF, crc >> 8])
            else:                                                # a corrupted valid frame
                typ = rng.choice(types)
                base = bytearray(il.encode(typ, _il_rand_payload(rng, typ)))
                m = rng.random()
                if m < 0.35:
                    for _ in range(rng.choice([1, 1, 1, 2, 3])):
                        base[rng.randrange(len(base))] ^= 1 << rng.randrange(8)
                elif m < 0.5:
                    base = base[:rng.randrange(1, len(base))]
                elif m < 0.65:
                    base[rng.choice([2, 3])] = rng.choice([0, 1, 7, 9, 0xC5, 0xFF, il.CMD_LEN, il.SNAP_LEN, il.STAT_LEN,
                                                           rng.randrange(256)])
                elif m < 0.75:
                    j = rng.randrange(len(base) + 1)
                    base[j:j] = bytes([rng.choice([il.SYNC0, il.SYNC1, rng.randrange(256)])])
                elif m < 0.8:                                    # 1-3 bytes dropped (UART overrun)
                    for _ in range(rng.choice([1, 1, 2, 3])):
                        del base[rng.randrange(len(base))]
                else:
                    base = bytearray(rng.choice([[il.SYNC0], [il.SYNC0, il.SYNC1], [il.SYNC0, il.SYNC1, il.SYNC0]])) + base
                b = bytes(base)
            parts.append(b)
        out.append((max_out, accept, b"".join(parts)))
    return out


def _il_rand_attitude(rng: random.Random, max_tilt_deg: float = 85.0) -> tuple[float, float, float, float]:
    """Unit quaternion with tilt < max_tilt_deg (away from the ZYX singularity) and any heading, sometimes scaled
    by 1 +- 5 % (an estimator q between normalisations)."""
    tl, az, yw = math.radians(rng.uniform(0, max_tilt_deg)), rng.uniform(-math.pi, math.pi), rng.uniform(-math.pi, math.pi)
    tw, tx, ty = math.cos(tl / 2), math.cos(az) * math.sin(tl / 2), math.sin(az) * math.sin(tl / 2)
    yc, ys = math.cos(yw / 2), math.sin(yw / 2)
    q = (yc * tw, yc * tx - ys * ty, yc * ty + ys * tx, ys * tw)
    s = rng.uniform(0.95, 1.05) if rng.random() < 0.2 else 1.0
    return tuple(il.f32(c * s) for c in q)


def check_ilink(vid: int, loop_hz: int, seed: int, n_streams: int, n_snaps: int) -> dict:
    """``sim.ilink_proto`` vs ``src/core/ilink_proto.c`` (``test_core --ilink-vectors``): random streams through both
    decoders (every returned frame, every counter, the open candidate), random CMD / SNAP / STAT re-encodings
    (C: unpack + pack + il_encode, bytes equal) and invalid (TYPE, LEN) pairs, il_crc16 / lp_crc16 on random data,
    the FCU snapshot (il_snap_from_tlm_src, bytes equal) and the COM TELEM path (il_unpack_snap, il_snap_to_tlm_src,
    onboard_telemetry, lp_encode_telem vs ``telem_frame_from_snap``): every byte equal except roll / pitch / yaw,
    which may differ by one 0.01 deg quantum (float32 vs float64 trig; counted)."""
    rng = random.Random(seed)
    streams = ilink_streams(rng, n_streams)
    lines, expect, kinds = [], [], []
    tot = {k: 0 for k in il.DEC_STAT_KEYS}
    n_frames = n_need = n_corner = 0
    names = {V063_IL_ACCEPT_ALL: "all", il.ACCEPT_FCU: "fcu", il.ACCEPT_COM: "com"}
    by_accept = {names[a]: 0 for a in (V063_IL_ACCEPT_ALL, il.ACCEPT_FCU, il.ACCEPT_COM)}
    for mo, acc, data in streams:
        dec, frames, need, cneed = il.decode_trace(data, mo, acc)
        by_accept[names[acc]] += 1
        n_frames += len(frames)
        n_need += len(need)
        n_corner += sum(1 for a, c in zip(need, cneed) if a <= 2 and c == 15)    # only CRC bytes left, the next one 0xC5
        for k in il.DEC_STAT_KEYS:
            tot[k] += dec.stats[k]
        lines.append(f"D {mo} {acc} {data.hex()}")
        expect.append("D" + "".join(f" {fr.type}:{fr.len}:{fr.payload.hex()}" for fr in frames) + " S "
                      + " ".join(str(dec.stats[k]) for k in il.DEC_STAT_KEYS) + f" P {dec.pending}"
                      + f" N {need.hex()} C {cneed.hex()}")     # C appends " PUSHDIFF" if il_decoder_push disagrees
        kinds.append("decode")
    n_enc = 0
    for i in range(3000):
        if i % 4 == 3:                                           # invalid (TYPE, LEN) pairs and wrong-length payloads
            typ = rng.choice([0, 4, 0x10, 0xFF, il.TYPE_CMD, il.TYPE_SNAP, il.TYPE_STAT])
            pl = bytes(rng.randrange(256) for _ in range(rng.choice([0, 7, 9, 35, 37, 48, 50, il.CMD_LEN, il.SNAP_LEN])))
        else:
            typ = rng.choice([il.TYPE_CMD, il.TYPE_SNAP, il.TYPE_STAT])
            pl = _il_rand_payload(rng, typ)
        lines.append(f"E {typ} {pl.hex()}")
        try:
            expect.append("E " + il.encode(typ, pl).hex())
            n_enc += 1
        except ValueError:
            expect.append("E -")
        kinds.append("encode")
    for i in range(3000):
        init = il.CRC_INIT if i % 2 else rng.randrange(65536)
        data = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 256)))
        c = il.crc16(data, init)
        lines.append(f"R {init} {data.hex()}")
        expect.append(f"R {c:04x} {c:04x}")
        kinds.append("crc")
    for i in range(2000):                                        # FCU: tlm source + q -> SNAP payload
        def fv() -> float:
            k = rng.random()
            if k < 0.05:
                return rng.choice([math.nan, math.inf, -math.inf, -0.0, 0.0])
            return il.f32(rng.uniform(-1, 1) * 10 ** rng.uniform(-3, 5))
        src = il.TlmSource(state=rng.randrange(256), flags=rng.randrange(256), reason=rng.randrange(256),
                           ack_seq=rng.randrange(256), nonce=rng.randrange(65536), throttle_milli=rng.randrange(65536),
                           tick=rng.randrange(1 << 32), h_m=fv(), vz_mps=fv(), roll_deg=fv(), pitch_deg=fv(), yaw_deg=fv(),
                           tilt_deg=fv(), v_cell=fv(), loop_max_us=fv())
        q = _il_rand_attitude(rng) if rng.random() < 0.8 else tuple(fv() for _ in range(4))
        seq = rng.randrange(256)
        nums = [seq, src.state, src.flags, src.reason, src.ack_seq, src.nonce, src.throttle_milli, src.tick,
                *(il.f32_bits(getattr(src, n)) for n in ("h_m", "vz_mps", "roll_deg", "pitch_deg", "yaw_deg", "tilt_deg",
                                                         "v_cell", "loop_max_us")), *(il.f32_bits(c) for c in q)]
        lines.append("F " + " ".join(str(x) for x in nums))
        expect.append("F " + il.snap_from_tlm_src(seq, src, q).pack().hex())
        kinds.append("snap")
    telem_rows = []
    for i in range(n_snaps):                                     # COM: SNAP -> TELEM frame
        bits = [il.f32_bits(rng.uniform(-200, 4000)), il.f32_bits(rng.uniform(-450, 450)), il.f32_bits(rng.uniform(0, 95)),
                il.f32_bits(rng.uniform(2.5, 70)), il.f32_bits(rng.uniform(0, 70000))]
        for j in range(5):
            if rng.random() < 0.03:
                bits[j] = rng.choice(_IL_SPECIAL_BITS)
        if rng.random() < 0.01:
            qb = [rng.choice(_IL_SPECIAL_BITS[:7]) for _ in range(4)]   # NaN / inf / -0 attitude: angles NaN or saturated
        else:
            qb = [il.f32_bits(c) for c in _il_rand_attitude(rng)]
        payload = struct.pack("<BBBBBHHI9I", rng.randrange(256), rng.randrange(6), rng.randrange(256), rng.randrange(256),
                              rng.randrange(256), rng.randrange(65536), rng.randrange(1001), rng.randrange(1 << 32), *bits, *qb)
        lines.append(f"P {vid} {loop_hz} {payload.hex()}")
        expect.append("P " + il.telem_frame_from_snap(il.Snap.unpack(payload), vid, loop_hz).hex())
        kinds.append("telem")
        telem_rows.append(len(lines) - 1)
    fin, fout = os.path.join(BUILD, "ilink_in.txt"), os.path.join(BUILD, "ilink_out.txt")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    run_exe(["--ilink-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        got = f.read().splitlines()
    if len(got) != len(expect):
        raise SystemExit(f"ilink-vectors: {len(got)} lines, expected {len(expect)}")
    st = {"streams": n_streams, "streams_by_accept": by_accept, "stream_bytes": sum(len(d) for _, _, d in streams),
          "py_frames": n_frames,
          "py_counters_total": tot, "need_cmd_need_pairs": n_need, "cmd_need_15_corner": n_corner,
          "encodings": n_enc, "encodings_refused": 3000 - n_enc, "crc_cases": 3000,
          "snapshots": 2000, "telem_frames": n_snaps, "mismatches": {"decode": 0, "encode": 0, "crc": 0, "snap": 0, "telem": 0},
          "telem_angle_fields_one_quantum_off": 0, "telem_frames_with_angle_diff": 0, "telem_max_angle_diff_cdeg": 0,
          "first_mismatches": []}
    for k, (a, b) in enumerate(zip(got, expect)):
        kind = kinds[k]
        if kind == "telem" and a != b:
            fc, fp = bytes.fromhex(a[2:]), bytes.fromhex(b[2:])
            ok = len(fc) == len(fp) == lp.TELEM_FRAME_BYTES and lp.crc16_ccitt(fc[2:-2]) == (fc[-2] | fc[-1] << 8)
            other = [j for j in range(min(len(fc), len(fp)) - 2) if fc[j] != fp[j] and not 20 <= j < 26]
            diffs = [abs(int.from_bytes(fc[o:o + 2], "little", signed=True) - int.from_bytes(fp[o:o + 2], "little", signed=True))
                     for o in il.TELEM_ANGLE_OFFSETS] if ok else [99]
            st["telem_angle_fields_one_quantum_off"] += sum(1 for d in diffs if d == 1)
            st["telem_frames_with_angle_diff"] += 1
            st["telem_max_angle_diff_cdeg"] = max(st["telem_max_angle_diff_cdeg"], max(diffs))
            if ok and not other and max(diffs) <= 1:
                continue
        elif a == b:
            continue
        st["mismatches"][kind] += 1
        if len(st["first_mismatches"]) < 8:
            st["first_mismatches"].append({"line": k, "kind": kind, "input": lines[k][:200], "c": a[:300], "py": b[:300]})
    st["pass"] = (sum(st["mismatches"].values()) == 0 and st["telem_max_angle_diff_cdeg"] <= 1
                  and all(v > 0 for v in tot.values()))
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


def print_ilink(st: dict) -> None:
    print(f"== ilink (FCU <-> COM): {st['streams']} streams (accept masks {st['streams_by_accept']}; "
          f"{st['stream_bytes']} bytes, {st['py_frames']} returned frames, "
          f"counters {st['py_counters_total']}, need / cmd_need after every byte incl. {st['cmd_need_15_corner']} "
          f"cmd_need-15 corners, il_decoder_push on a twin decoder), {st['encodings']} encodings + {st['encodings_refused']} refused, "
          f"{st['crc_cases']} CRCs, {st['snapshots']} FCU snapshots, {st['telem_frames']} COM TELEM frames; "
          f"mismatches {st['mismatches']} -> {'PASS' if st['pass'] else 'FAIL'}")
    print(f"   TELEM roll/pitch/yaw (float32 C vs float64 reference): {st['telem_angle_fields_one_quantum_off']} fields one "
          f"0.01 deg quantum off in {st['telem_frames_with_angle_diff']} of {st['telem_frames']} frames, max "
          f"{st['telem_max_angle_diff_cdeg']} quantum; every other byte equal")
    for m in st["first_mismatches"][:4]:
        print(f"   mismatch: {m}")


# --------------------------------------------------------------------------- #
# NAV <-> FCU nlink (SPEC 10.5, v0.6.3)
# --------------------------------------------------------------------------- #
def _nl_rand_nav(rng: random.Random) -> tuple[int, list[int], list[int]]:
    """(seq, 14 float bit patterns, 6 integers): plausible values, special floats and raw patterns."""
    k = rng.random()
    if k < 0.5:
        fl = [*(_il_rand_attitude(rng) if rng.random() < 0.9 else tuple(rng.uniform(-1, 1) for _ in range(4))),
              *(rng.uniform(-10, 10) for _ in range(3)), *(rng.uniform(-160, 160) for _ in range(3)),
              rng.uniform(-50, 3500), rng.uniform(-400, 400), rng.uniform(-50, 3500), rng.uniform(0, 180)]
        bits = [nl.f32_bits(x) for x in fl]
    else:
        bits = [_il_rand_float_bits(rng) for _ in range(14)]
    ints = [rng.choice([0, 1, 0xFFFF, rng.randrange(65536)]) for _ in range(4)] + [rng.randrange(256), rng.randrange(256)]
    return rng.randrange(256), bits, ints


def _nl_expect_decode(buf: bytes) -> str:
    r = nl.decode_nav_raw(buf)
    if r is None:
        s = "D 0"
    else:
        fld = nl.nav_fields(*r)
        s = (f"D 1 {fld['seq']} " + " ".join(f"{b:08x}" for b in fld["float_bits"]) + " "
             + " ".join(str(fld["ints"][k]) for k in nl.NAV_INT_FIELDS))
    f = nl.decode_fcu(buf)
    return s + (" | 0" if f is None else f" | 1 {f.seq} {f.state} {f.fflags} {f.params_tag} {f.thr_q}")


def check_nlink(seed: int, n_frames: int, n_decodes: int) -> dict:
    """``sim.nlink_proto`` vs ``src/core/nlink_proto.c`` (``test_core --nlink-vectors``): NAV / FCU encodings (72 bytes
    equal) and both decoders on random exchange buffers (validity and every field equal, floats as bit patterns)."""
    rng = random.Random(seed)
    lines, expect, kinds = [], [], []
    navs: list[bytes] = []
    fcus: list[bytes] = []
    for _ in range(n_frames):
        seq, bits, ints = _nl_rand_nav(rng)
        fr = nl.encode_nav_payload(seq, nl.nav_payload_from_bits(bits, ints))
        navs.append(fr)
        lines.append("N " + " ".join(str(x) for x in [seq, *bits, *ints]))
        expect.append("N " + fr.hex())
        kinds.append("encode_nav")
        f = nl.Fcu(rng.randrange(256), rng.choice([rng.randrange(6), rng.randrange(256)]), rng.randrange(256), rng.randrange(256))
        ff = nl.encode_fcu(f)
        fcus.append(ff)
        lines.append(f"F {f.seq} {f.state} {f.fflags} {f.params_tag}")
        expect.append("F " + ff.hex())
        kinds.append("encode_fcu")
    reasons = {"nav": {"valid": 0, "sync": 0, "type": 0, "crc": 0}, "fcu": {"valid": 0, "sync": 0, "type": 0, "crc": 0}}
    for _ in range(n_decodes):
        k = rng.random()
        base = bytearray(rng.choice(navs) if rng.random() < 0.6 else rng.choice(fcus))
        if k < 0.25:
            pass                                                   # valid
        elif k < 0.45:
            for _ in range(rng.choice([1, 1, 2, 3])):
                base[rng.randrange(nl.XFER_BYTES)] ^= 1 << rng.randrange(8)
        elif k < 0.55:
            base[rng.randrange(nl.XFER_BYTES)] = rng.randrange(256)
        elif k < 0.62:
            base = base[1:] + bytes([rng.randrange(256)])          # a lost first byte
        elif k < 0.69:
            base = bytes([rng.randrange(256)]) + base[:-1]         # an extra byte before the frame
        elif k < 0.75:
            base[2] = rng.choice([0, nl.TYPE_NAV, nl.TYPE_FCU, 3, 0xFF])
        elif k < 0.82:                                             # a valid FCU frame with a random tail
            base = bytearray(rng.choice(fcus)[:nl.FCU_FRAME_BYTES] + bytes(rng.randrange(256) for _ in range(62)))
        elif k < 0.88:                                             # echoed bytes: MISO = previous MOSI from byte j on
            j = rng.randrange(1, nl.XFER_BYTES)
            mosi = rng.choice(navs)
            base = bytearray(rng.choice(fcus)[:j] + mosi[j - 1:nl.XFER_BYTES - 1])
        elif k < 0.93:                                             # a frame with a CRC that matches a wrong TYPE
            t = rng.choice([0, 3, 0xFF, nl.TYPE_FCU, nl.TYPE_NAV])
            body = bytes([t]) + bytes(base[3:70])
            crc = nl.crc16(body)
            base = bytearray(base[:2] + body + bytes([crc & 0xFF, crc >> 8]))
        else:
            base = bytearray(rng.randrange(256) for _ in range(nl.XFER_BYTES))
            if rng.random() < 0.5:
                base[0], base[1] = nl.SYNC0, nl.SYNC1
        buf = bytes(base)
        assert len(buf) == nl.XFER_BYTES
        reasons["nav"][nl.nav_reject_reason(buf) or "valid"] += 1
        reasons["fcu"][nl.fcu_reject_reason(buf) or "valid"] += 1
        lines.append("D " + buf.hex())
        expect.append(_nl_expect_decode(buf))
        kinds.append("decode")
    # v0.7.1: the FCU frame's thr_q byte (own random stream after the v0.6.3 draws): encodings with every thr_q, the
    # FCU side's quantisation u -> thr_q, and decodes of those frames, valid and corrupted
    rng2 = random.Random(seed + 1000)
    thr_frames: list[bytes] = []
    for _ in range(max(1, n_frames // 5)):
        u = rng2.choice([rng2.random(), 0.0, 1.0, rng2.uniform(-0.5, 1.5), rng2.randrange(251) / 250 + 0.002 * rng2.random()])
        tq = nl.fcu_throttle_q(u) if rng2.random() < 0.8 else rng2.randrange(256)
        f = nl.Fcu(rng2.randrange(256), rng2.randrange(6), rng2.randrange(4), rng2.randrange(256), tq)
        ff = nl.encode_fcu(f)
        thr_frames.append(ff)
        lines.append(f"F {f.seq} {f.state} {f.fflags} {f.params_tag} {f.thr_q}")
        expect.append("F " + ff.hex())
        kinds.append("encode_fcu")
    for _ in range(max(1, n_decodes // 10)):
        b = bytearray(rng2.choice(thr_frames))
        if rng2.random() < 0.4:
            b[rng2.randrange(nl.FCU_FRAME_BYTES)] ^= 1 << rng2.randrange(8)
        elif rng2.random() < 0.3:
            b[nl.FCU_FRAME_BYTES + rng2.randrange(62)] = rng2.randrange(256)
        buf = bytes(b)
        reasons["fcu"][nl.fcu_reject_reason(buf) or "valid"] += 1
        reasons["nav"][nl.nav_reject_reason(buf) or "valid"] += 1
        lines.append("D " + buf.hex())
        expect.append(_nl_expect_decode(buf))
        kinds.append("decode")
    fin, fout = os.path.join(BUILD, "nlink_in.txt"), os.path.join(BUILD, "nlink_out.txt")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    run_exe(["--nlink-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        got = f.read().splitlines()
    if len(got) != len(expect):
        raise SystemExit(f"nlink-vectors: {len(got)} lines, expected {len(expect)}")
    st = {"nav_encodings": n_frames, "fcu_encodings": n_frames + len(thr_frames), "decode_buffers": n_decodes + max(1, n_decodes // 10),
          "decode_outcomes": reasons, "thr_q_frames": len(thr_frames),
          "mismatches": {"encode_nav": 0, "encode_fcu": 0, "decode": 0}, "first_mismatches": []}
    for k, (a, b) in enumerate(zip(got, expect)):
        if a == b:
            continue
        st["mismatches"][kinds[k]] += 1
        if len(st["first_mismatches"]) < 8:
            st["first_mismatches"].append({"line": k, "kind": kinds[k], "input": lines[k][:200], "c": a[:300], "py": b[:300]})
    st["pass"] = (sum(st["mismatches"].values()) == 0
                  and all(v > 0 for side in reasons.values() for v in side.values()))
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


def print_nlink(st: dict) -> None:
    print(f"== nlink (NAV <-> FCU): {st['nav_encodings']} NAV + {st['fcu_encodings']} FCU encodings, {st['decode_buffers']} "
          f"exchange buffers through both decoders (outcomes {st['decode_outcomes']}); mismatches {st['mismatches']} "
          f"-> {'PASS' if st['pass'] else 'FAIL'}")
    for m in st["first_mismatches"][:4]:
        print(f"   mismatch: {m}")


# --------------------------------------------------------------------------- #
# estimator and altitude filter
# --------------------------------------------------------------------------- #
def check_estimator(ref: Ref, rng: np.random.Generator, n_seq: int) -> dict:
    """``sim.estimator.PadAlignGyroEstimator`` vs ``att_est_*``: pad alignment (running mean -> EMA,
    accelerometer levelling) for longer than the averaging window, then gyro-only flight."""
    from sim.estimator import PadAlignGyroEstimator
    dt = ref.dt
    seqs = []
    n_avg = ref.gains["est_pad_n_avg"]
    n_pad = int(n_avg * 1.6)
    for s in range(n_seq):
        tilt = math.radians(rng.uniform(0, 6))
        ax = rng.uniform(0, 2 * math.pi)
        q_true = quat.multiply(quat.from_yaw(rng.uniform(-3, 3)),
                               quat.from_axis_angle(np.array([math.cos(ax), math.sin(ax), 0.0]), tilt))
        bias = rng.normal(0, math.radians(0.5), 3)
        rows = []
        w = np.zeros(3)
        for k in range(n_pad + 1000):
            pad = k < n_pad
            if pad:
                acc = quat.rotate_inverse(q_true, np.array([0, 0, ref.d["g0"]])) + rng.normal(0, 0.05, 3)
                gyro = bias + rng.normal(0, 0.003, 3)
            else:
                w = np.clip(0.97 * w + rng.normal(0, 0.3, 3), -5, 5)
                acc = np.array([0.0, 0.0, 15.0]) + rng.normal(0, 0.5, 3)
                gyro = w + bias + rng.normal(0, 0.003, 3)
            rows.append(dict(init=int(k == 0), gyro=gyro, acc=acc, pad=int(pad)))
        seqs.append(rows)
    fin, fout = os.path.join(BUILD, "est_in.csv"), os.path.join(BUILD, "est_out.csv")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("seq,init,gx,gy,gz,ax,ay,az,on_pad\n")
        for s, rows in enumerate(seqs):
            for r in rows:
                f.write(",".join([str(s), str(r["init"])] + [r32(x) for x in r["gyro"]] + [r32(x) for x in r["acc"]]
                                 + [str(r["pad"])]) + "\n")
    run_exe(["--est-vectors", fin, fout])
    _, c = read_csv(fout)
    i = 0
    st = {"rows": 0, "sequences": n_seq, "pad_ticks_per_seq": n_pad, "n_avg": n_avg, "max_abs_d_q": 0.0,
          "max_abs_d_bias": 0.0, "max_abs_d_rate": 0.0}
    for rows in seqs:
        est = PadAlignGyroEstimator(dt, ref.gains["est_pad_avg_tau_s"])
        if est.n_avg != n_avg:
            raise SystemExit(f"N_avg mismatch: reference {est.n_avg}, gains.h {n_avg}")
        for r in rows:
            g = np.array([f32(x) for x in r["gyro"]])
            a = np.array([f32(x) for x in r["acc"]])
            q = est.update(g, a, bool(r["pad"]))
            rate = est.rate(g)
            cr = c[i]
            i += 1
            st["rows"] += 1
            cq = cr[1:5]
            dq = min(float(np.max(np.abs(cq - q))), float(np.max(np.abs(cq + q))))
            st["max_abs_d_q"] = max(st["max_abs_d_q"], dq)
            st["max_abs_d_bias"] = max(st["max_abs_d_bias"], float(np.max(np.abs(cr[5:8] - est.bias))))
            st["max_abs_d_rate"] = max(st["max_abs_d_rate"], float(np.max(np.abs(cr[8:11] - rate))))
    st["pass"] = st["max_abs_d_q"] < EST_Q_BOUND and st["max_abs_d_bias"] < EST_BIAS_BOUND
    st["bounds"] = {"q": EST_Q_BOUND, "bias_rad_s": EST_BIAS_BOUND}
    return st


def check_alt_filter(ref: Ref, rng: np.random.Generator, n_seq: int, baro_div: int) -> dict:
    sens = SensorConfig()
    dt = ref.dt
    seqs = []
    for s in range(n_seq):
        lat = rng.uniform(0.0, 0.1)
        h_true, v_true = 0.0, 0.0
        rows = []
        for k in range(int(30.0 / dt)):
            t = k * dt
            a = (rng.uniform(2, 5) if t < 10 else -9.0) if s % 2 == 0 else 3.0 * math.sin(0.5 * t)
            h_true += v_true * dt
            v_true += a * dt
            has = int(k % baro_div == 0)
            rows.append(dict(t=k, lat=lat, az=a + rng.normal(0, 0.3), has=has, hb=h_true + rng.normal(0, 0.3)))
        seqs.append(rows)
    fin, fout = os.path.join(BUILD, "alt_in.csv"), os.path.join(BUILD, "alt_out.csv")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("seq,t,latency,az,has_baro,h_baro\n")
        for s, rows in enumerate(seqs):
            for r in rows:
                f.write(",".join([str(s), str(r["t"]), r32(r["lat"]), r32(r["az"]), str(r["has"]), r32(r["hb"])]) + "\n")
    run_exe(["--alt-vectors", fin, fout])
    _, c = read_csv(fout)
    i = 0
    st = {"rows": 0, "sequences": n_seq, "max_abs_d_h_m": 0.0, "max_abs_d_v_mps": 0.0, "max_h_m": 0.0}
    dtf = f32(dt)
    for rows in seqs:
        af = AltitudeFilter(f32(sens.altitude_crossover_hz), f32(sens.altitude_zeta), f32(rows[0]["lat"]))
        af.reset(0.0)
        for r in rows:
            af.predict(f32(r["az"]), dtf)
            if r["has"]:
                af.correct(f32(r["hb"]), r["t"] * dtf)
            cr = c[i]
            i += 1
            st["rows"] += 1
            st["max_abs_d_h_m"] = max(st["max_abs_d_h_m"], abs(cr[2] - af.h))
            st["max_abs_d_v_mps"] = max(st["max_abs_d_v_mps"], abs(cr[3] - af.v))
            st["max_h_m"] = max(st["max_h_m"], abs(af.h))
    st["pass"] = st["max_abs_d_h_m"] < ALT_H_BOUND_M and st["max_abs_d_v_mps"] < ALT_V_BOUND_MPS
    st["bounds"] = {"h_m": ALT_H_BOUND_M, "v_mps": ALT_V_BOUND_MPS}
    return st


LOG_SAMPLE_INFO = "fw 3.0.0 role FCU BUDGET_90mm_6S params 0123456789abcdef"     # test_core.c write_log_sample
# the IL_STAT test_core.c write_log_sample logs as its 'C' record (FCU tick 1245), built here with the reference
LOG_SAMPLE_STAT = il.Stat(com_ticks=1240, reset_cause=0x08, flags=il.STAT_RADIO_OK | il.STAT_BOOT | il.STAT_SX127X,
                          radio_version=0x12, rssi_dbm=-101, snr_q4=-10, params_tag=0x48, dec_frames=77, dec_bad_crc=3,
                          cmd_forwarded=77, replies=76, il_tx_dropped=1, loop_us_max=812)


def check_log_roundtrip() -> dict:
    """``test_core --log-sample`` (every record type of log format 4, junk between frames) through ``decode_log.decode``:
    every field as written by the C packers, per record type (flight 'F', event 'E', link 'L', boot 'B', status 'S'
    with the v0.6 ilink / COM fields, COM 'C' = one IL_STAT byte for byte, text 'T')."""
    path = os.path.join(BUILD, "log_sample.bin")
    run_exe(["--log-sample", path])
    with open(path, "rb") as f:
        d = decode_log.decode(f.read())
    loop_hz, counts_per_us = decode_log.defaults_from_params()
    fl = d["flight"]
    lsb = {k: 1.0 / v for k, v in decode_log.SCALE.items()}      # fixed point truncates: error < 1 LSB
    ev, lk, st, bt, cm = d["events"], d["link"], d["status"], d["boot"], d["com"]
    checks = {
        "framing": d["bad"] == 0 and d["skipped"] == 3 and d["text"] == ["boot sample"],   # no format warning either
        "flight": (len(fl) == 3 and fl[2]["reason"] == "I4 apogee limiter" and fl[0]["link_ok"] == 1
                   and fl[0]["radio_ok"] == 1 and fl[0]["aligned"] == 1
                   and abs(fl[1]["gyro_f_x"] - 0.125) < lsb["rate"] and abs(fl[2]["h_est_m"] - 12.345) < lsb["h"]
                   and abs(fl[0]["vz_est_mps"] + 1.5) < lsb["vz"] and abs(fl[0]["v_cell"] - 3.95) < lsb["vcell"]
                   and abs(fl[0]["delta_2"] + 0.05) < lsb["angle"] and abs(fl[0]["qw"] - 1.0) <= lsb["q"]
                   and fl[0]["loop_us_max"] == 2345 and fl[2]["tick"] == 1010),
        "event": (len(ev) == 1 and ev[0]["frm"] == "ARMED" and ev[0]["to"] == "POWERED" and ev[0]["cause"] == "cmd LAUNCH"
                  and ev[0]["tick"] == 1010),
        "link": (len(lk) == 1 and lk[0]["seq"] == 42 and lk[0]["cmd"] == "LAUNCH" and lk[0]["nonce"] == 0xBEEF
                 and lk[0]["result"] == "executed" and lk[0]["state_after"] == "POWERED" and lk[0]["rssi_dbm"] == -97
                 and lk[0]["snr_db"] == -1.5),
        "boot": (len(bt) == 1 and bt[0]["format"] == decode_log.FORMAT_VERSION == 4 and bt[0]["role"] == "FCU"
                 and bt[0]["reset"] == ["power_on"] and bt[0]["loop_hz"] == round(loop_hz)
                 and bt[0]["counts_per_us"] == round(counts_per_us) and bt[0]["info"] == LOG_SAMPLE_INFO),
        "status_v4": (len(st) == 1 and st[0]["format"] == 4 and st[0]["tick"] == 1250 and st[0]["stack_free"] == 1500
                      and (st[0]["twi_errors"], st[0]["uart_dropped"], st[0]["link_other"], st[0]["radio_errors"],
                           st[0]["baro_fails"], st[0]["imu_fails"]) == (11, 12, 13, 14, 15, 16)
                      and st[0]["link_frames"] == 77 and st[0]["link_bad"] == 3 and st[0]["replies"] == 76
                      and st[0]["vbat_v"] == 23.9 and st[0]["reset"] == ["power_on"] and st[0]["rssi_dbm"] == -98
                      and (st[0]["il_cmds"], st[0]["il_snaps"], st[0]["il_bad"], st[0]["il_uart_errors"],
                           st[0]["il_dropped"]) == (78, 76, 2, 1, 4)
                      and st[0]["com_age_s"] == 1 and st[0]["com_flags"] == (il.STAT_RADIO_OK | decode_log.COM_TAG_MISMATCH)
                      and st[0]["com_params_mismatch"] == 1 and st[0]["radio_ok"] == 1 and st[0]["com_boot"] == 0
                      and st[0]["backend"] == "lora_uart"),
        "com": (len(cm) == 1 and cm[0]["tick"] == 1245 and abs(cm[0]["t_s"] - 1245 / loop_hz) < 1e-12
                and all(cm[0][k] == getattr(LOG_SAMPLE_STAT, k) for k in il.STAT_FIELDS)
                and cm[0]["reset"] == ["watchdog"] and cm[0]["radio_ok"] == 1 and cm[0]["com_boot"] == 1
                and cm[0]["backend"] == "sx127x" and cm[0]["snr_db"] == -2.5),
    }
    return {"records": len(fl), "events": len(ev), "link_records": len(lk), "status": len(st), "boot": len(bt),
            "com": len(cm), "skipped_bytes": d["skipped"], "bad": d["bad"], "checks": checks, "pass": all(checks.values())}


# --------------------------------------------------------------------------- #
# v0.7 VTOL build (SPEC 8.9): supervisor, vertical controller, FCU stack
# --------------------------------------------------------------------------- #
VTOL_FLOAT_FIELDS = FLOAT_FIELDS + ("ceiling_m", "v_geofence_m", "v_fall_vz_mps")
VTOL_OVERRIDES = ("spool_ticks", "link_land_ticks", "link_disarm_ticks", "endurance_ticks", "hard_cutoff_ticks",
                  "v_fall_confirm_ticks", "safe_after_landed_ticks", "abort_release_ticks", "imu_fail_ticks",
                  "baro_fail_ticks", "rate_persist_ticks", "hover_dm", "alt_step_dm", "alt_min_dm", "alt_max_dm",
                  "ceiling_m", "v_geofence_m", "v_fall_vz_mps", "gyro_saturation_abort",
                  "land_timeout_ticks", "baro_limit_base_ticks", "baro_limit_mt_per_dm", "v_geofence_dm")   # v0.7.1
# VerticalParams fields rounded to float32 for both sides (the C gets the derived constants of vc_c_overrides)
VC_FLOAT_FIELDS = ("climb_rate_mps", "descent_rate_mps", "touchdown_rate_mps", "flare_altitude_m", "alt_kp", "vz_kp", "vz_ki",
                   "vz_i_max", "vz_max_up_mps", "vz_max_down_mps", "u_min", "u_max", "u_hover", "throttle_exp",
                   "cos_tilt_min", "td_vz_still_mps", "support_g", "thrust_tau_s", "still_tau_s", "still_lead",
                   "support_tau_s", "g", "kt_tau_s", "kt_min", "kt_max", "kt_max_tilt_deg",   # v0.7.1b: kt_*
                   "takeoff_climb_m")                                                         # v0.8
VC_INT_FIELDS = ("ramp_ticks", "landed_confirm_ticks", "floor_ticks", "takeoff_ticks",          # v0.8: + takeoff_ticks
                 "takeoff_settle_ticks")                                          # physics-review fix
VC_BOOL_FIELDS = ("vz_ff",)
# float32 C vs float64 reference, vertical controller v2 (justification: module docstring, section 4c)
VC_H_TOL_M = 1e-4               # h_ref (= the exact integer reference x 1e-6: <= 2 ulp)
VC_VZ_TOL_MPS = 1e-4            # vz_cmd, vz_err, dh_ref (x max(1, alt_kp))
VC_U_TOL = 2e-4                 # u, u_ff, integ, ESC, attitude-schedule u (throttle units; < 1/2 ESC PWM count)
VC_COS_TOL = 1e-6               # cos(tilt) from the float32 quaternion: a few ulp of 1
VC_VLPF_TOL_MPS = 1e-4          # v_lpf: ulp(|v_z| <= 4 m/s) / a_V(0.004) = 1.2e-4 worst-case accumulation; measured <= 1e-6
VC_DLPF_TOL_MPS2 = 5e-4         # d_lpf: ulp(|f_z| <= 40 m/s^2) / a_S(0.0198) = 1.9e-4 worst case
VC_RTHR_TOL = 2e-5              # r: ulp(r <= 1.2) / a_T(0.026) = 5e-6 worst case
VC_KT_TOL = 1e-5                # v0.7.1b k_T after ONE learning step from the reference's k_T (k_sync, see _vc_row): a_K x the
                                # sample's float32 error (r within VC_RTHR_TOL, r >= 0.25: 1.1e-4 relative at k_max 1.4)
                                # = 9e-7 at a_K 0.008, + 1/2 ulp(1.4) = 6e-8 -> 1e-6; x10
VC_AMB_H_M = 1e-5               # a decision whose float64 margin is this close may legitimately go the other way in
VC_AMB_VZ_MPS = 1e-4            # float32 (x max(1, alt_kp) for v_z; v_lpf errors are up to ~1e-4 by the bound above)
VC_AMB_U = 1e-5                 # throttle-unit decisions (saturation, conditional integration)
VC_AMB_I = 2e-5                 # the integrator clamp (pinned): the integrator's float32 drift (measured <= 4e-6) x 5
VC_AMB_A = 5e-4                 # the support threshold on d_lpf (VC_DLPF_TOL_MPS2)
VC_RESYNC_TICKS = 3             # these ticks are not compared; the next one must agree again (else the sequence is cut)
VC_TD_TOL = 2e-6                # v0.9 td_bias: <= 150 float32 steps of 0.0008 (a few ulp of 0.12 each), measured below
PHASE_OF_NAME = {v: k for k, v in lp.PHASE_NAMES.items()}
VC_SIG_BITS = (("low", 1), ("still", 2), ("support", 4), ("floor", 8), ("ground_signature", 16),   # vertical_ctrl.h
               ("low_alt", 32), ("kt_learn", 64),                                                    # VC_SIG_* (v0.7.1b: +2)
               ("takeoff_fail", 128))                                                                # v0.8
# v0.7.1 VTOL NAV estimator / altitude filter (section 7b; justification in check_mahony / check_altv)
MAHONY_Q_TOL = 1e-4             # quaternion components: the ballistic section's EST_Q_BOUND (justification: check_mahony)
MAHONY_BIAS_TOL = 1e-5          # rad/s: the ballistic section's EST_BIAS_BOUND (pad average); in flight b integrates
                                # ki x the e difference (<= MAHONY_E_TOL): 0.005 x 60 s x 2e-5 = 6e-6 rad/s
MAHONY_RATE_TOL = 1e-5          # rad/s: the bias bound + 1/2 ulp of the gyro full scale (34.9 rad/s: 1.9e-6)
MAHONY_E_TOL = 1e-4             # rad (correction error = the tilt part of the q difference)
MAHONY_ALPF_TOL = 1e-4          # m/s^2 (low-passed specific force: ulp(20) / alpha_a(0.049) = 4e-5 worst case)
MAHONY_AMB_GATE = 1e-4          # m/s^2: gate decision | |a_lpf| - g | vs accel_gate g within this -> knife-edge
ALTV_H_TOL_M = 2e-3             # h
ALTV_V_TOL_MPS = 2e-3           # v
ALTV_B_TOL_MPS2 = 1e-4          # bias state
ALTV_COMP_TOL_M = 1e-6          # the compensation of the latest sample
ALTV_REL_TOL = 1e-4             # h, v, b: + this x the magnitude (float32 rounding scales with it; see check_altv)


def sup_params_vtol(ref: Ref, overrides: dict | None = None, v08: bool = False) -> onb.SupervisorParams:
    """The VTOL reference parameters with the firmware's float32 thresholds; overrides = shortened timers etc.  A
    v_geofence_m override without v_geofence_dm also moves the baro-lost clamp (v0.7.1).  v0.8: ``v08`` False = the
    v0.7.1 VTOL build of this params.json (no GPS rules, no take-off check: sections 4b-4d), True = the v0.8 build."""
    sp = onb.SupervisorParams.from_params(ref.p, fc_config_path=gfp.FC_CONFIG, mode=lp.MODE_VTOL,
                                          gps=None if v08 else False, takeoff_check=None if v08 else False)
    kw = {k: f32(getattr(sp, k)) for k in VTOL_FLOAT_FIELDS}
    ov = dict(overrides or {})
    if "v_geofence_m" in ov and "v_geofence_dm" not in ov:
        ov["v_geofence_dm"] = onb.metres_to_dm(ov["v_geofence_m"])
    for k, v in ov.items():
        if k not in VTOL_OVERRIDES:
            raise ValueError(k)
        kw[k] = f32(v) if k in VTOL_FLOAT_FIELDS else v
    sp = dataclasses.replace(sp, **kw)
    sp.check_vtol()
    return sp


def vc_params32(ref: Ref, overrides: dict | None = None, v08: bool = False) -> VerticalParams:
    """The firmware's vertical-controller parameters (gen_firmware_params: the gains params_gen.h carries, the default
    build profile) as float32 values; overrides by field name.  v0.8: ``v08`` False = the v0.7.1 controller (the
    take-off check off: takeoff_ticks 0)."""
    vp = ref.d["vt"]["vp"]
    kw = {k: f32(getattr(vp, k)) for k in VC_FLOAT_FIELDS}
    if not v08:
        kw.update(takeoff_ticks=0, takeoff_climb_m=0.0, takeoff_settle_ticks=0)
    for k, v in (overrides or {}).items():
        if k not in VC_FLOAT_FIELDS + VC_INT_FIELDS + VC_BOOL_FIELDS:
            raise ValueError(k)
        kw[k] = f32(v) if k in VC_FLOAT_FIELDS else (bool(v) if k in VC_BOOL_FIELDS else int(v))
    vp = dataclasses.replace(vp, **kw)
    vp.check()
    return vp


def vc_c_overrides(vp: VerticalParams) -> dict:
    """Every vc_params_t field of core/vertical_ctrl.h from the reference parameters (the derived constants rounded to
    float32 once, as gen_firmware_params writes them), so the C controller runs exactly the reference's numbers."""
    n = vp.throttle_exp
    return {"up_um": vp.up_um, "dn_um": vp.dn_um, "td_um": vp.td_um, "flare_um": vp.flare_um,
            "um_tick_to_mps": f32(vp.loop_hz / 1e6), "flare_altitude_m": f32(vp.flare_altitude_m), "alt_kp": f32(vp.alt_kp),
            "vz_kp": f32(vp.vz_kp), "vz_ki_dt": f32(vp.vz_ki * vp.dt), "vz_i_max": f32(vp.vz_i_max),
            "vz_max_up_mps": f32(vp.vz_max_up_mps), "vz_max_down_mps": f32(vp.vz_max_down_mps), "u_min": f32(vp.u_min),
            "u_max": f32(vp.u_max), "u_hover": f32(vp.u_hover), "inv_u_hover": f32(1.0 / vp.u_hover),
            "throttle_exp": f32(n), "throttle_exp_int": 2 if n == 2.0 else 0,
            "cos_tilt_min": f32(vp.cos_tilt_min), "ramp_ticks": vp.ramp_ticks,
            "landed_confirm_ticks": vp.landed_confirm_ticks, "floor_ticks": vp.floor_ticks,
            "td_vz_still_mps": f32(vp.td_vz_still_mps), "lead_mps": f32(vp.still_lead * vp.touchdown_rate_mps),
            "support_mps2": f32(vp.support_g * vp.g), "g": f32(vp.g), "thrust_alpha": f32(vp.thrust_alpha),
            "still_alpha": f32(vp.still_alpha), "support_alpha": f32(vp.support_alpha), "vz_ff": int(bool(vp.vz_ff)),
            "kt_alpha": f32(vp.kt_alpha), "kt_min": f32(vp.kt_min), "kt_max": f32(vp.kt_max),     # v0.7.1b
            "kt_cos_min": f32(vp.kt_cos_min),
            "takeoff_climb_m": f32(vp.takeoff_climb_m), "takeoff_ticks": int(vp.takeoff_ticks),   # v0.8
            "takeoff_settle_ticks": int(vp.takeoff_settle_ticks),                                 # physics-review fix
            **vc_v09_c_overrides(vp)}                                                             # v0.9


VC_V9_FLAG_BITS = (("two", 0x01), ("boost", 0x02), ("liftoff", 0x04), ("ref_hold", 0x08), ("land_rebase", 0x10),
                   ("td_rel", 0x20))                                    # core/vertical_ctrl.h VC_V9_*


def vc_v09_c_overrides(vp: VerticalParams) -> dict:
    """v0.9 vc_params_t fields (core/vertical_ctrl.h; tools/gen_firmware_params.py vertical_v09 - the same derivation)."""
    v9 = gfp.vertical_v09(vp, "crosscheck")
    return {"ramp_total_ticks": v9["rt"], "v9": sum(b for k, b in VC_V9_FLAG_BITS if v9[k]),
            "approach_shift": v9["approach_shift"], "approach_min_um": v9["approach_min_um"],
            "to_u_start": f32(v9["u_start"]), "to_s1": f32(v9["s1"]), "to_s2": f32(v9["s2"]),
            "to_search_f": f32(v9["search_f"]), "to_search_vz": f32(v9["search_vz"]), "kt_alpha_to": f32(v9["kt_alpha_to"]),
            "to_airborne_mps2": f32(v9["airborne_mps2"]), "td_step": f32(v9["td_step"]), "td_max": f32(v9["td_max"]),
            "td_ok": f32(v9["td_ok"])}


SUP_C_U32 = ("ramp_ticks", "coast_hold_ticks", "fall_confirm_ticks", "link_abort_ticks", "link_disarm_ticks",
             "max_motor_ticks", "release_after_cut_ticks", "abort_release_ticks", "rate_persist_ticks", "imu_fail_ticks",
             "baro_fail_ticks", "spool_ticks", "link_land_ticks", "endurance_ticks", "hard_cutoff_ticks",
             "v_fall_confirm_ticks", "safe_after_landed_ticks", "land_timeout_ticks", "baro_limit_base_ticks",
             "baro_limit_mt_per_dm", "hover_dm", "alt_step_dm", "alt_min_dm", "alt_max_dm", "v_geofence_dm", "climb_milli")
SUP_C_F32 = ("tilt_abort_deg", "arm_tilt_max_deg", "min_cell_v", "geofence_m", "apogee_limit_m", "fall_vz_mps", "ceiling_m",
             "v_geofence_m", "v_fall_vz_mps")


def sup_c_overrides(sp: onb.SupervisorParams) -> dict:
    """The supervisor fields of a #K / #S line (test_core.c onb_override) from reference parameters: a 6-DOF run of the
    demo profile or with other thresholds runs the C with exactly its numbers."""
    d = {k: int(getattr(sp, k)) for k in SUP_C_U32}
    d.update({k: f32(getattr(sp, k)) for k in SUP_C_F32})
    d["gyro_saturation_abort"] = int(bool(sp.gyro_saturation_abort))
    d["gps_hold"] = int(bool(sp.gps_hold))                 # v0.8 (the v0.7.1 sections: 0)
    d["takeoff_check"] = int(bool(sp.takeoff_check))
    return d


def q_from_tilt(tilt_deg: float, axis_rad: float) -> tuple:
    """A float32 unit quaternion tilted by tilt_deg about the horizontal axis at axis_rad (yaw 0); non-finite tilt ->
    a non-finite q (the controller then takes c = 1)."""
    if not math.isfinite(tilt_deg):
        return (f32(math.nan),) * 4 if tilt_deg != tilt_deg else (1.0, f32(math.inf), 0.0, 0.0)
    h = math.radians(tilt_deg) / 2.0
    return (f32(math.cos(h)), f32(math.sin(h) * math.cos(axis_rad)), f32(math.sin(h) * math.sin(axis_rad)), 0.0)


def _q_text(q) -> str:
    return ",".join(r32(x) for x in q)


def _f_text(x) -> str:
    return "nan" if x is None else r32(x)


def vc_sig(o) -> int:
    """VerticalOutputs flags as core/vertical_ctrl.h VC_SIG_* bits."""
    return sum(b for k, b in VC_SIG_BITS if getattr(o, k))


def vc_state(vc: VerticalController) -> tuple:
    """The reference controller's counters after an update (compared with the C's): floor run, the four confirmation
    counters (v0.7.1b: + all three above the flare band), low_in_land."""
    return (vc.floor_run, *vc.cnt, vc.low_in_land, vc.airborne, vc.to_passed, vc.ramp_k, vc.fz_lpf, vc.td_bias)


def _ov_text(ov: dict, prefix: str = "") -> str:
    return " ".join(f"{prefix}{k}={(r32(v) if isinstance(v, float) else int(v))}" for k, v in ov.items())


class TracingSupervisor(onb.OnboardSupervisor):
    """The reference supervisor, unchanged, with every state transition and phase change recorded as
    (state_from, state_to, phase_from, phase_to) in call order (the C onb_trans_t log)."""

    def __init__(self, *a, **kw):
        self.trace: list[list[int]] = []
        super().__init__(*a, **kw)

    def update(self, inp):
        self.last_inp = inp                       # v0.8: the supervisor's own inputs (FcuStack fills the GPS ones)
        return super().update(inp)

    def _enter(self, new, t, why):
        rec = [int(self.state), int(new), int(self.phase), 0]
        self.trace.append(rec)
        super()._enter(new, t, why)
        rec[3] = int(self.phase)

    def _set_phase(self, new, t, why):
        self.trace.append([int(self.state), int(self.state), int(self.phase), int(new)])
        super()._set_phase(new, t, why)


def _vtol_inp_row(inp: onb.OnboardInputs) -> str:
    """A VTOL supervisor / stack row (test_core.c run_onb_vectors_mode): the ballistic columns, the v0.7 controller flags
    and ESC throttle, the v0.7.1 ground signature, low_in_land_ticks, NAV quaternion and body-z specific force."""
    q = inp.q_nav if inp.q_nav is not None else (1.0, 0.0, 0.0, 0.0)
    return (_inp_row(inp) + f",{int(inp.vc_touchdown)},{int(inp.vc_at_target)},{r32(inp.throttle_prev)},"
            f"{int(inp.vc_ground_signature)},{int(inp.vc_low_in_land_ticks)},{_q_text(q)},{_f_text(inp.accel_z_mps2)}")


def _py_row_vtol(sup: TracingSupervisor, out: onb.OnboardOutputs, n_exec_before: int, n_trace_before: int, t: int) -> dict:
    ex = [(e["seq"], e["cmd"], e["code"], e["state_before"], e["state_after"], e["phase_before"], e["phase_after"])
          for e in sup.exec_log[n_exec_before:]]
    return dict(t=t, state=int(out.state), target=sup.target_milli, out=out.throttle_milli, active=int(out.control_active),
                release=int(out.release), reason=out.reason, flags=out.flags, ack=out.ack_seq, nonce=out.nonce,
                reply=-1 if out.reply_seq is None else out.reply_seq,
                telem=out.telemetry.pack().hex() if out.telemetry is not None else "-", exec=ex,
                trans=[tuple(x) for x in sup.trace[n_trace_before:]], phase=out.phase,
                h_tgt_dm=onb.metres_to_dm(out.h_tgt_m), motor_on=int(out.motor_on), buzzer=int(out.buzzer),
                baro_lost=int(out.baro_lost))


def _c_onb_row(parts: list[str], vtol: bool) -> dict:
    """One C --onb-vectors / --stack-vectors row -> the comparable fields."""
    c_exec = [] if parts[14] == "-" else [tuple(int(x) for x in e.split(":")) for e in parts[14].split("|")]
    c_trans = [] if parts[15] == "-" else [tuple(int(x) for x in e.split(":")) for e in parts[15].split("|")]
    d = dict(state=int(parts[2]), target=int(parts[3]), out=int(parts[4]), active=int(parts[5]), release=int(parts[6]),
             reason=int(parts[7]), flags=int(parts[8]), ack=int(parts[9]), nonce=int(parts[10]), reply=int(parts[11]),
             telem=parts[13], exec=[e for e in c_exec if e[2] < onb_const("ONB_EXEC_NONE")],
             trans=[(a, b, pf, pt) for a, b, _, pf, pt in c_trans])
    if vtol:
        d.update(phase=int(parts[16]), h_tgt_dm=int(parts[17]), motor_on=int(parts[18]), buzzer=int(parts[19]),
                 baro_lost=int(parts[20]))
    return d, int(parts[12]), c_exec, c_trans


class Plant1D:
    """Vertical point mass for the sequence generators (thrust T_max u^n through the EDF lag, gravity, ground at
    z_ground; no attitude, drag or ground effect - it only has to give the supervisor and the controller plausible
    altitude histories).  v0.7.1: the body-z specific force ``fz`` (thrust / m in the air, g resting on the ground) for
    the controller's support signature."""

    def __init__(self, ref: Ref, z_ground: float = 0.0, mass_scale: float = 1.0, thrust_scale: float = 1.0):
        self.m = ref.veh.full.mass_kg * mass_scale
        self.t_max = ref.veh.t_max_n * thrust_scale            # v0.7.1b: plants off the thrust model (k_T)
        self.n = float(ref.p["edf"]["throttle_exponent"])
        self.tau = float(ref.p["edf"]["thrust_time_constant_s"])
        self.zg = z_ground
        self.z, self.v, self.T, self.ground = z_ground, 0.0, 0.0, True

    def step(self, u: float, dt: float) -> None:
        self.T += (self.t_max * max(u, 0.0) ** self.n - self.T) * (1.0 - math.exp(-dt / self.tau))
        a = self.T / self.m - onb.G0
        if self.ground and a <= 0.0:
            self.v = 0.0
            return
        self.ground = False
        self.v += a * dt
        self.z += self.v * dt
        if self.z <= self.zg:
            self.z, self.v, self.ground = self.zg, 0.0, True

    @property
    def fz(self) -> float:
        return onb.G0 if self.ground else self.T / self.m


class VtolRandomOperator:
    """Random ground + sensors + vertical-controller flags for the VTOL supervisor.  The ground mostly flies (ON, UP /
    DOWN, OFF) and sometimes sends what does not apply (the ballistic build's codes, unknown codes, wrong nonces,
    repeated / old / half-space SEQs, several frames per tick); polls slow enough for I1p / V1 / REJ_LINK, outages;
    faults (arm switch, IMU, baro, tilt, rate, spin, gyro saturation, low cell, baro altitude spikes above the ceiling
    and the geofence, falls, a tilt / sag at the end of the spool for V7) more likely in flight.  Altitude and the
    controller flags come from a 1-D plant flown by the Python vertical controller on the supervisor's outputs (the
    FcuStack order), with spurious at_target / touchdown flags injected; v0.7.1: the ground signature and
    low_in_land_ticks either from that controller or injected (the template sim/test_vtol_ref.vtol_random_sequence:
    a random ground signature in LAND, a LAND-tick counter), so V8 / V9 / V10 occur."""

    S = onb.State
    CMD_WEIGHTS = {
        S.SAFE: [(lp.CMD_ON, 12), (lp.CMD_NONE, 4), (lp.CMD_OFF, 1), (lp.CMD_UP, 1), (lp.CMD_DOWN, 0.5), (lp.CMD_KILL, 1),
                 (lp.CMD_ARM, 1), (lp.CMD_LAUNCH, 0.5), (lp.CMD_THROTTLE, 0.5), (lp.CMD_DISARM, 0.3)],
        S.ARMED: [(lp.CMD_NONE, 10), (lp.CMD_OFF, 1.5), (lp.CMD_KILL, 1), (lp.CMD_ON, 1), (lp.CMD_UP, 1), (lp.CMD_CUT, 0.5)],
        S.POWERED: [(lp.CMD_NONE, 14), (lp.CMD_UP, 4), (lp.CMD_DOWN, 4), (lp.CMD_OFF, 1.2), (lp.CMD_KILL, 0.3),
                    (lp.CMD_ON, 0.5), (lp.CMD_THROTTLE, 0.5), (lp.CMD_RELEASE, 0.3)],
        S.UNPOWERED: [(lp.CMD_NONE, 8), (lp.CMD_OFF, 1), (lp.CMD_ON, 1), (lp.CMD_UP, 1), (lp.CMD_KILL, 0.3)],
        S.RELEASED: [(lp.CMD_NONE, 8), (lp.CMD_ON, 1), (lp.CMD_KILL, 1), (lp.CMD_RELEASE, 0.5)],
        S.ABORT: [(lp.CMD_NONE, 8), (lp.CMD_ON, 1), (lp.CMD_KILL, 1), (lp.CMD_OFF, 1), (lp.CMD_DOWN, 1), (lp.CMD_ARM, 0.5)],
    }
    FAULT_P = {"arm": 0.0001, "imu": 0.0002, "baro": 0.0003, "tilt": 0.0002, "rate": 0.0003, "sat": 0.0001, "spin": 0.0002,
               "vcell": 0.0003, "high": 0.0004, "fall": 0.0003, "gate": 0.001}

    def __init__(self, rng: random.Random, ref: Ref, sp: onb.SupervisorParams, vp: VerticalParams, nudger: Nudger):
        r = self.r = rng
        self.sp, self.vp, self.nd = sp, vp, nudger
        self.poll = max(1, int(min(sp.link_disarm_ticks, sp.link_land_ticks) * r.choice([0.05, 0.1, 0.2, 0.4, 0.7, 1.3])))
        self.loss = r.choice([0.0, 0.05, 0.15, 0.4])
        self.seq = r.randrange(256)
        self.pending: lp.Command | None = None
        self.next_poll = r.randrange(1, 30)
        self.outage_until = -1
        self.aligned_from = r.choice([0, 0, r.randrange(0, 400)])
        self.arm_closed_from = r.choice([0, 0, 0, r.randrange(0, 600)])
        self.clean = r.random() < 0.3
        self.gain = 0.0 if self.clean else r.choice([1.0, 4.0, 15.0])
        self.fault = {k: 0 for k in self.FAULT_P}
        self.tilt_max = r.choice([3.0, 4.0, 4.0, 8.0])
        self.tilt = r.uniform(0.0, self.tilt_max)
        self.vcell = r.uniform(3.4, 4.2)
        self.vcell_low_until = r.choice([0, 0, 0, r.randrange(0, 800)])
        self.plant = Plant1D(ref, z_ground=r.choice([0.0, 0.0, -0.8, 0.6]), mass_scale=r.choice([1.0, 1.0, 0.9, 1.08]))
        self.vc = VerticalController(vp)
        self.vo = None
        self.u = 0.0
        self.h_off = 0.0
        self.vz_bias = r.choice([0.0, 0.0, 0.05, -0.2])
        self.inject = r.choice([0.0, 0.001, 0.004])
        # v0.7.1
        self.gsig_p = r.choice([0.0, 0.0, 0.0, 0.002, 0.5, 1.0])     # injected ground signature in LAND (per tick)
        self.lil_inject = r.random() < 0.4                          # low_in_land_ticks = a LAND-tick counter
        self.lil_cnt = 0
        self.q_axis = r.uniform(0.0, 2.0 * math.pi)
        self.accel_none = r.random() < 0.15                         # no accelerometer value (NaN in C, None in Python)
        self.landed_faults = r.random() < 0.3                       # IMU / altitude / fall faults while LANDED
        self.landed_kind = None                                     # set by the generator: one fault kind, every LANDED
        self.baro_script = False                                    # set by the generator: a baro loss in CLIMB / HOLD
        self.baro_done = False

    def _cmd(self, state: onb.State, nonce: int) -> lp.Command:
        r = self.r
        if r.random() < 0.01:
            return lp.Command(r.choice([12, 0x7F, 0xFF]), 0, 0)
        w = self.CMD_WEIGHTS[state]
        if self.clean and state == onb.State.POWERED:
            w = [(lp.CMD_NONE, 30), (lp.CMD_UP, 2), (lp.CMD_DOWN, 2), (lp.CMD_OFF, 0.2)]
        c = r.choices([c for c, _ in w], [x for _, x in w])[0]
        good = r.random() < 0.8
        n = nonce if good else (nonce ^ (1 + r.randrange(65535)))
        return lp.Command(c, r.choice([0, 500]), n if c in lp.NONCE_COMMANDS or r.random() < 0.3 else r.randrange(65536))

    def frames(self, t: int, sup: onb.OnboardSupervisor) -> list[onb.UplinkFrame]:
        r = self.r
        if r.random() < 0.0004:
            self.outage_until = t + int(self.sp.link_land_ticks * r.choice([0.5, 1.1, 2.5, 6.0]))
        if t < self.next_poll:
            return []
        self.next_poll = t + max(1, self.poll + r.randrange(-3, 4))
        if t < self.outage_until or r.random() < self.loss:
            return []
        if self.pending is not None and sup.has_seq and sup.last_seq == self.seq and r.random() < 0.85:
            self.pending = None
        if self.pending is None and r.random() < 0.5:
            self.seq = (self.seq + 1) & 0xFF
            self.pending = self._cmd(sup.state, sup.nonce)
        cmd = self.pending if self.pending is not None else lp.Command(lp.CMD_NONE)
        if cmd.cmd in lp.NONCE_COMMANDS and cmd.nonce != sup.nonce and r.random() < 0.5:
            cmd = lp.Command(cmd.cmd, cmd.arg, sup.nonce)
            self.pending = cmd
        seq = self.seq
        x = r.random()
        if x < 0.02:
            seq = (seq - r.randrange(1, 200)) & 0xFF
        elif x < 0.03:
            seq = (seq + 128) & 0xFF
        out = [onb.UplinkFrame(seq, cmd)]
        if r.random() < 0.03:
            for _ in range(r.randrange(1, 4)):
                if r.random() < 0.5:
                    out.append(onb.UplinkFrame(seq, cmd))
                else:
                    self.seq = (self.seq + 1) & 0xFF
                    out.append(onb.UplinkFrame(self.seq, self._cmd(sup.state, sup.nonce)))
        return out[:4]

    def sensors(self, t: int, sup: onb.OnboardSupervisor) -> dict:
        r, sp, fl = self.r, self.sp, self.fault
        st = sup.state
        flying = st in (onb.State.POWERED, onb.State.UNPOWERED)
        for k, p in self.FAULT_P.items():
            if fl[k] > 0:
                fl[k] -= 1
            elif (not self.clean or k == "gate") and r.random() < (0.02 if (k == "gate" and st == onb.State.ARMED)
                                                                    else p * (self.gain if flying else 1.0)):
                fl[k] = {"arm": r.randrange(1, 40), "imu": r.randrange(1, 2 * sp.imu_fail_ticks + 2),
                         "baro": r.randrange(1, 2 * sp.baro_fail_ticks + 2), "tilt": r.randrange(1, 30),
                         "rate": r.randrange(1, 2 * sp.rate_persist_ticks + 2),
                         "sat": r.randrange(1, 2 * sp.rate_persist_ticks + 2),
                         "spin": r.randrange(1, 2 * sp.rate_persist_ticks + 2), "vcell": r.randrange(1, 60),
                         "high": r.randrange(1, 20), "fall": r.randrange(1, 2 * sp.v_fall_confirm_ticks + 3),
                         "gate": r.randrange(1, 40)}[k]
        if (self.baro_script and not self.baro_done and st == onb.State.POWERED and sup.phase in onb.FLY_PHASES
                and t % 97 == 0 and r.random() < 0.3):
            fl["baro"] = max(fl["baro"], sp.baro_fail_ticks + r.randrange(2, 400))
            self.baro_done = True
        if self.landed_kind is not None and st == onb.State.UNPOWERED and t - sup.t_entry == 5:
            k = self.landed_kind
            fl[k] = max(fl[k], {"imu": sp.imu_fail_ticks + 2, "high": 3, "fall": sp.v_fall_confirm_ticks + 2}[k])
        elif self.landed_faults and st == onb.State.UNPOWERED and r.random() < 0.02:
            k = r.choice(["imu", "imu", "high", "fall"])
            fl[k] = max(fl[k], {"imu": sp.imu_fail_ticks + 2, "high": 3, "fall": sp.v_fall_confirm_ticks + 2}[k])
        pl = self.plant
        pl.step(self.u, 1.0 / sp.loop_hz)
        if st in (onb.State.SAFE, onb.State.ARMED):              # NAV pad mode: h zeroed at the pad level
            self.h_off = pl.z
        h = pl.z - self.h_off + r.gauss(0.0, 0.02)
        vz = pl.v + self.vz_bias + r.gauss(0.0, 0.02)
        if fl["high"]:
            h = r.choice([sp.ceiling_m + r.uniform(0.0, 2.0), sp.v_geofence_m + r.uniform(0.0, 5.0), sp.ceiling_m,
                          sp.v_geofence_m])
        if fl["fall"]:
            vz = r.choice([sp.v_fall_vz_mps, sp.v_fall_vz_mps - r.uniform(0.0, 3.0)])
        self.tilt = min(max(self.tilt + r.gauss(0.0, 0.2), 0.0), self.tilt_max)
        tilt = self.tilt
        if fl["tilt"]:
            tilt = r.uniform(sp.tilt_abort_deg - 3.0, 95.0)
        elif fl["gate"]:
            tilt = r.choice([sp.arm_tilt_max_deg, sp.arm_tilt_max_deg + 2.0, 20.0])
        if r.random() < 0.002:
            tilt = r.choice([sp.arm_tilt_max_deg, sp.tilt_abort_deg, f32(np.nextafter(np.float32(sp.arm_tilt_max_deg),
                                                                                     np.float32(0)))])
        thr = sp.tilt_rate_abort_rad_s
        w = [r.gauss(0.0, 0.5), r.gauss(0.0, 0.5), r.gauss(0.0, 1.0)]
        if fl["rate"]:
            ang = r.uniform(0, 2 * math.pi)
            mag = thr * r.uniform(0.95, 1.6)
            w[0], w[1] = mag * math.cos(ang), mag * math.sin(ang)
        if fl["spin"]:
            w[2] = r.choice([-1, 1]) * sp.spin_rate_abort_rad_s * r.uniform(0.97, 1.3)
        w = self.nd.gyro(w)
        self.vcell = min(max(self.vcell + r.gauss(0.0, 0.002), sp.min_cell_v + 0.02), 4.2)
        vcell = self.vcell
        if fl["vcell"] or (fl["gate"] and r.random() < 0.5):
            vcell = r.choice([sp.min_cell_v, sp.min_cell_v - 0.01, sp.min_cell_v - 0.4, 0.0])
        if t < self.vcell_low_until:
            vcell = sp.min_cell_v - 0.05
        vo = self.vo
        at = bool(vo is not None and vo.at_target)
        td = bool(vo is not None and vo.touchdown)
        if r.random() < self.inject:                              # spurious / missing flags: any phase
            at = not at if r.random() < 0.5 else at
            td = not td if r.random() < 0.5 else td
        tp = self.u if r.random() > 0.01 else r.choice([0.0, 1.0, r.random(), 1.0005])
        # v0.7.1: ground signature and low_in_land_ticks of the controller, or injected
        land = sup.phase == lp.PHASE_LAND
        gs = bool(vo is not None and vo.ground_signature)
        if self.gsig_p > 0.0:
            gs = land and r.random() < self.gsig_p
        elif self.gsig_p < 0.0:                                   # generator: never a ground signature (V9)
            gs = False
        lil = int(vo.low_in_land_ticks) if vo is not None else 0
        if self.lil_inject:
            self.lil_cnt = (self.lil_cnt + 1 if r.random() < 0.998 else 0) if (land and not sup.baro_lost) else 0
            lil = self.lil_cnt
        lil = min(lil, 0xFFFF)
        q = q_from_tilt(tilt, self.q_axis)
        az = None if self.accel_none else f32(pl.fz + r.gauss(0.0, 0.3))
        return dict(h_m=f32(h), vz_mps=f32(vz), tilt_deg=f32(tilt), gyro_rad_s=tuple(w), gyro_saturated=bool(fl["sat"]),
                    v_cell=f32(vcell), imu_ok=not fl["imu"], baro_ok=not fl["baro"], aligned=t >= self.aligned_from,
                    arm_switch=not fl["arm"] and t >= self.arm_closed_from, roll_deg=f32(r.uniform(-180, 180)),
                    pitch_deg=f32(r.uniform(-90, 90)), yaw_deg=f32(r.uniform(-180, 180)),
                    loop_max_us=float(r.choice([0, r.randrange(0, 4000), 65535, 70000.5])),
                    vc_touchdown=td, vc_at_target=at, throttle_prev=f32(tp), vc_ground_signature=gs,
                    vc_low_in_land_ticks=lil, q_nav=q, accel_z_mps2=az)

    def after(self, inp: onb.OnboardInputs, out: onb.OnboardOutputs) -> None:
        """The FcuStack step after the supervisor: the vertical controller on this tick's phase, the ESC command."""
        self.vo = self.vc.update(VerticalInputs(phase=out.phase, h_tgt_m=out.h_tgt_m, baro_lost=out.baro_lost, h_m=inp.h_m,
                                                vz_mps=inp.vz_mps, cos_tilt=vcm.cos_tilt_from_q(inp.q_nav),
                                                f_z_mps2=inp.accel_z_mps2))
        self.u = f32(self.vo.u) if out.motor_on else 0.0


def onb_vtol_random_sequences(ref: Ref, n_seq: int, seed: int) -> list[dict]:
    rng = random.Random(seed)
    vp = vc_params32(ref)
    seqs = []
    n_landed_kind = n_baro_script = 0
    for s in range(n_seq):
        short = s % 3 != 0
        ov: dict = {}
        if short:
            r = rng
            ov = dict(spool_ticks=r.choice([0, 1, 30, 200]), link_disarm_ticks=r.choice([40, 90, 140]),
                      link_land_ticks=r.choice([40, 120, 300]), endurance_ticks=r.choice([300, 800, 1500]),
                      v_fall_confirm_ticks=r.choice([1, 5, 30]), safe_after_landed_ticks=r.choice([0, 1, 20, 100]),
                      abort_release_ticks=r.choice([0, 3, 40]), imu_fail_ticks=r.choice([1, 4, 12]),
                      baro_fail_ticks=r.choice([2, 10, 40]), rate_persist_ticks=r.choice([1, 3, 6]),
                      land_timeout_ticks=r.choice([0, 40, 250, 1500]), baro_limit_base_ticks=r.choice([0, 30, 200]),
                      baro_limit_mt_per_dm=r.choice([0, 1000, 62500]))
            ov["hard_cutoff_ticks"] = ov["endurance_ticks"] + r.choice([1, 100, 400])
            if r.random() < 0.3:
                ov["gyro_saturation_abort"] = False
            if r.random() < 0.4:                                          # low limits: the plant itself reaches V3 / V4
                ov.update(hover_dm=r.choice([20, 30]), alt_min_dm=10, alt_step_dm=r.choice([5, 10]),
                          alt_max_dm=r.choice([40, 60]), ceiling_m=r.choice([7.0, 9.0]), v_geofence_m=r.choice([10.0, 12.0]))
                ov["v_geofence_dm"] = onb.metres_to_dm(ov["v_geofence_m"])
        if short and s % 8 == 5:                                          # a baro loss with a short time limit: V8 / V9
            ov.update(baro_limit_base_ticks=rng.choice([0, 30]), baro_limit_mt_per_dm=rng.choice([0, 1000]))
        sp = sup_params_vtol(ref, ov)
        nd = Nudger(sp)
        op = VtolRandomOperator(rng, ref, sp, vp, nd)
        if short and s % 8 == 5:
            op.baro_script, op.inject, op.gsig_p, op.clean = True, 0.0, (-1.0, 1.0)[n_baro_script % 2], True  # V9 / V8
            n_baro_script += 1
        if short and s % 8 == 3:                                          # the LANDED rules (I9 IMU, V4, V5 -> SAFE)
            op.landed_kind, op.clean, op.inject = ("imu", "high", "fall")[n_landed_kind % 3], True, 0.0
            n_landed_kind += 1
        reset = rng.random() < 0.06
        nonce_seed = rng.choice([0, 1, 0xFFFF, rng.randrange(65536)])
        sup = TracingSupervisor(sp, seed=nonce_seed, reset_in_flight=reset)
        n = rng.choice([2000, 4000, 8000]) if short else rng.choice([4000, 12000])
        rows, py = [], []
        for t in range(n):
            frames = op.frames(t, sup)
            inp = onb.OnboardInputs(tick=t, frames=frames, **op.sensors(t, sup))
            n0, n1 = len(sup.exec_log), len(sup.trace)
            out = sup.update(inp)
            op.after(inp, out)
            rows.append(_vtol_inp_row(inp))
            py.append(_py_row_vtol(sup, out, n0, n1, t))
        ov2 = dict(ov, mode=1, gps_hold=0, takeoff_check=0)              # v0.8 params.json: the v0.7.1 VTOL build
        seqs.append(dict(kind="vtol_random", seed=nonce_seed, reset=reset, overrides=ov2, rows=rows, py=py, nudged=nd.count))
    return seqs


def check_onboard_vtol(ref: Ref, seqs: list[dict]) -> dict:
    """The VTOL supervisor: every tick identical state, phase, target (dm), motor_on, buzzer, baro_lost, control /
    release flags, reason, telemetry flags, ack, nonce, reply SEQ, the 31 TELEM2 payload bytes, executed / rejected
    commands with their phases, and the transition + phase-change log; every VTOL interlock / rejection code must occur."""
    fin, fout = os.path.join(BUILD, "onbv_in.txt"), os.path.join(BUILD, "onbv_out.csv")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        for i, sq in enumerate(seqs):
            f.write(f"#S {i} {sq['seed']} {int(sq['reset'])} {_ov_text(sq['overrides'])}\n")
            f.write("\n".join(sq["rows"]) + "\n")
    run_exe(["--onb-vectors", fin, fout])
    st = {"ticks": 0, "sequences": len(seqs), "mismatch_ticks": 0, "first_mismatches": [], "groups": {},
          "interlocks_seen": {}, "rejections_seen": {}, "transitions_seen": {}, "phase_changes_seen": {},
          "commands_executed": {}, "i10_boots": 0, "telemetry_frames": 0, "frames": 0, "nudged_inputs": 0,
          "ticks_by_phase": {}}
    names = {v: k for k, v in STATE_NAMES.items()}
    with open(fout, "r", encoding="utf-8") as f:
        f.readline()
        c_lines = f.read().splitlines()
    i = 0
    for si, sq in enumerate(seqs):
        g = st["groups"].setdefault(sq["kind"].split(":")[0], {"sequences": 0, "ticks": 0, "mismatch_ticks": 0})
        g["sequences"] += 1
        st["nudged_inputs"] += sq["nudged"]
        st["i10_boots"] += int(sq["reset"])
        for o in sq["py"]:
            parts = c_lines[i].split(",")
            i += 1
            if int(parts[0]) != si or int(parts[1]) != o["t"]:
                raise SystemExit(f"onb-vtol row order mismatch at {i}: {parts[:2]} vs seq {si} tick {o['t']}")
            c, interlock, c_exec, c_trans = _c_onb_row(parts, True)
            st["ticks"] += 1
            g["ticks"] += 1
            bad = [k for k in c if c[k] != o[k]]
            if interlock and o["reason"] != interlock:
                bad.append("interlock")
            if bad:
                st["mismatch_ticks"] += 1
                g["mismatch_ticks"] += 1
                if len(st["first_mismatches"]) < 12:
                    st["first_mismatches"].append({"seq": si, "kind": sq["kind"], "tick": o["t"], "fields": bad,
                                                   "c": {k: c[k] for k in bad if k in c}, "py": {k: o[k] for k in bad if k in o}})
            if interlock:
                nm = lp.REASON_NAMES.get(interlock, str(interlock))
                st["interlocks_seen"][nm] = st["interlocks_seen"].get(nm, 0) + 1
            for e in c_exec:
                if 0x40 <= e[2] < 0xFE:
                    nm = lp.REASON_NAMES.get(e[2], hex(e[2]))
                    st["rejections_seen"][nm] = st["rejections_seen"].get(nm, 0) + 1
                elif e[2] == 0:
                    nm = lp.CMD_NAMES.get(e[1], str(e[1]))
                    st["commands_executed"][nm] = st["commands_executed"].get(nm, 0) + 1
            for a, b, _, pf, pt in c_trans:
                if a == b:
                    key = f"{lp.PHASE_NAMES[pf]}->{lp.PHASE_NAMES[pt]}"
                    st["phase_changes_seen"][key] = st["phase_changes_seen"].get(key, 0) + 1
                else:
                    key = f"{names[a]}->{names[b]}"
                    st["transitions_seen"][key] = st["transitions_seen"].get(key, 0) + 1
            pn = lp.PHASE_NAMES.get(c.get("phase", 0), "?")
            st["ticks_by_phase"][pn] = st["ticks_by_phase"].get(pn, 0) + 1
            st["telemetry_frames"] += int(o["telem"] != "-")
            st["frames"] += len(c_exec)
    if i != len(c_lines):
        raise SystemExit(f"C produced {len(c_lines)} VTOL supervisor rows, expected {i}")
    need_il = [lp.REASON_NAMES[k] for k in lp.VTOL_INTERLOCK_REASONS if k != lp.R_I10_RESET]
    need_rej = [lp.REASON_NAMES[k] for k in lp.VTOL_REJECT_REASONS]
    st["missing_interlocks"] = [k for k in need_il if k not in st["interlocks_seen"]]
    st["missing_rejections"] = [k for k in need_rej if k not in st["rejections_seen"]]
    need_ph = ("SPOOL->CLIMB", "CLIMB->HOLD", "CLIMB->LAND", "HOLD->LAND", "LAND->LANDED")
    seen_ph = set(st["phase_changes_seen"]) | {f"{lp.PHASE_NAMES[pf]}->{lp.PHASE_NAMES[pt]}" for sq in seqs for o in sq["py"]
                                              for a, b, pf, pt in o["trans"] if a != b}
    st["missing_phase_changes"] = [k for k in need_ph if k not in seen_ph]
    # v0.7.1: the LANDED rules (I9 IMU, V4, V5 -> SAFE, never ABORT) and V8 / V10 -> LANDED
    landed = {lp.R_I9_IMU: "I9 IMU", lp.R_V_GEOFENCE: "V4", lp.R_V_FALL: "V5", lp.R_ARM_SWITCH_OPEN: "arm switch"}
    st["landed_rules_seen"] = {}
    for sq in seqs:
        for o in sq["py"]:
            for a, b, pf, pt in o["trans"]:
                if a == int(onb.State.UNPOWERED) and b == int(onb.State.SAFE) and o["reason"] in landed:
                    nm = landed[o["reason"]]
                    st["landed_rules_seen"][nm] = st["landed_rules_seen"].get(nm, 0) + 1
                if a == int(onb.State.UNPOWERED) and b == int(onb.State.ABORT) and not any(
                        e[1] == lp.CMD_KILL and e[2] == 0 for e in o["exec"]):
                    st["landed_rules_seen"]["LANDED->ABORT without KILL"] = (
                        st["landed_rules_seen"].get("LANDED->ABORT without KILL", 0) + 1)
    st["missing_landed_rules"] = [v for v in ("I9 IMU", "V4", "V5") if v not in st["landed_rules_seen"]]
    if "LANDED->ABORT without KILL" in st["landed_rules_seen"]:
        st["missing_landed_rules"].append("a LANDED -> ABORT without KILL happened")
    st["pass"] = (st["mismatch_ticks"] == 0 and not st["missing_interlocks"] and not st["missing_rejections"]
                  and not st["missing_phase_changes"] and st["i10_boots"] > 0 and not st["missing_landed_rules"])
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


# ---- vertical controller ----------------------------------------------------------------------------------------- #
def vc_before(vc: VerticalController) -> tuple:
    """The reference controller's state before an update that vc_margins needs (v0.9: + td_bias, the take-off check,
    the lift-off latch and k_T of the search decision)."""
    return (vc.active, vc.ref_um, vc.integ, vc.ramp_k, vc.td_bias, vc.to_passed, vc.airborne, vc.k_t)


def vc_margins(vp: VerticalParams, before: tuple, inp: VerticalInputs, o, after: tuple | None = None) -> tuple[bool, dict]:
    """The float decisions of this tick (v0.7.1: the integer reference has none) from the reference's own outputs and
    the integrator before the update, and whether float32 rounding could flip one: ambiguous = some decision's float64
    margin is within the float32 error bound (VC_AMB_*).  Decisions: the still lead (h_err vs -still_lead
    touchdown_rate), the command's downward saturation ('low', 'pinned'), still (|v_lpf| vs td_vz_still), support (d_lpf
    vs support_g g), the integrator clamp at -vz_i_max ('pinned'), the output saturation and the conditional
    integration.  The recomputed output must equal the reference's (a consistency check of this function)."""
    active, _ref0, integ0, ramp_k = before[:4]
    td0 = before[4] if len(before) > 4 else 0.0          # v0.9 td_bias before this tick (0 outside LAND)
    mode = vcm.mode_of_phase(inp.phase)
    if mode == vcm.MODE_IDLE:
        return False, {}
    if not active:
        integ0, ramp_k, td0 = 0.0, 0, 0.0
    h = inp.h_m
    eps_v = VC_AMB_VZ_MPS * max(1.0, vp.alt_kp)
    m: dict[str, float] = {}
    amb = False
    h_err = vp.alt_kp * (o.h_ref - h)
    raw = (o.dh_ref_mps if vp.vz_ff else 0.0) + h_err
    if math.isfinite(h_err):
        m["lead"] = h_err + vp.still_lead * vp.touchdown_rate_mps
        m["cmd_down"] = raw + vp.vz_max_down_mps
        amb = abs(m["lead"]) < eps_v or abs(m["cmd_down"]) < eps_v
    m["still"] = abs(o.vz_lpf) - vp.td_vz_still_mps
    amb = amb or abs(m["still"]) < VC_AMB_VZ_MPS
    # v0.7.1b thrust learning: its tilt guard (c of the float32 q, float32 vs float64 within VC_COS_TOL) and its r guard
    m["kt_cos"] = o.cos_tilt - vp.kt_cos_min
    m["kt_r"] = o.thrust_ratio - vcm.KT_R_MIN_ASSUMED
    amb = amb or abs(m["kt_cos"]) < VC_COS_TOL or abs(m["kt_r"]) < VC_RTHR_TOL
    if inp.f_z_mps2 is not None and math.isfinite(inp.f_z_mps2):
        m["support"] = o.f_excess_mps2 - vp.support_g * vp.g
        amb = amb or abs(m["support"]) < VC_AMB_A
    e, dt = o.vz_err, vp.dt
    cap = vp.cap_at(ramp_k)                                # v0.9 two-slope ceiling (cap_at), the W_SAG ceiling
    if getattr(inp, "u_cap", None) is not None and inp.u_cap < cap:
        cap = inp.u_cap
    i_raw = integ0 + vp.vz_ki * e * dt
    i_try = vcm.clampf(i_raw, -vp.vz_i_max, vp.vz_i_max)
    u_try = o.u_ff + vp.vz_kp * e + i_try
    u_raw = o.u_ff + vp.vz_kp * e + o.integ + td0           # td_bias of the previous tick (a LAND -> FLY tick too)
    if len(before) > 5 and mode == vcm.MODE_FLY:          # v0.9 take-off thrust search: e > 0 (its sign in float32)
        rt = vp.ramp_total_ticks
        if (vp.to_boost_per_s > 0.0 and vp.takeoff_ticks > 0 and not before[5] and (rt <= 0 or ramp_k + 1 >= rt)
                and before[7] > vp.kt_min):
            amb = amb or abs(e) < eps_v
    if after is not None and len(after) > 9 and mode == vcm.MODE_FLY and len(before) > 6 and not before[6]:
        lift = vp.kt_alpha_to > 0.0 or vp.to_cap_margin > 0.0 or vp.to_ref_hold
        if lift and inp.f_z_mps2 is not None and math.isfinite(inp.f_z_mps2):   # the lift-off latch
            m["airborne"] = after[9] - vp.g * (1.0 + vp.to_airborne_fz)
            amb = amb or abs(m["airborne"]) < VC_AMB_A
    m.update(i_lo=i_raw + vp.vz_i_max, i_hi=i_raw - vp.vz_i_max, aw_cap=u_try - cap, aw_min=u_try - vp.u_min,
             sat_cap=u_raw - cap, sat_min=u_raw - vp.u_min, e=e)
    amb = amb or abs(m["i_lo"]) < VC_AMB_I or abs(m["i_hi"]) < VC_AMB_I
    amb = amb or abs(u_raw - cap) < VC_AMB_U or abs(u_raw - vp.u_min) < VC_AMB_U
    amb = amb or (e > 0.0 and abs(u_try - cap) < VC_AMB_U) or (e < 0.0 and abs(u_try - vp.u_min) < VC_AMB_U)
    amb = amb or (abs(e) < eps_v and (u_try > cap - VC_AMB_U or u_try < vp.u_min + VC_AMB_U))
    if not (min(max(u_raw, vp.u_min), cap) == o.u):
        raise SystemExit(f"vc_margins disagrees with sim/vertical_ctrl.py: u {o.u!r} vs {min(max(u_raw, vp.u_min), cap)!r}")
    return amb, m


def _vc_row(inp: VerticalInputs, q: tuple, k_sync: float) -> str:
    """A --vc-vectors row; v0.7.1b: k_sync = the reference's k_T before this tick's update, which the C takes as its own
    k_T before the step.  Why: a replay holds f_z fixed (the reference's closed loop recorded it), so a free-running C
    k_T would learn from its own throttle while the recorded f_z does not answer it - a float32 difference dk moves u_ff
    by -u_ff dk / (2 k), r by -(u_ff / u) r dk / k and the next sample by +s (u_ff / u) dk / k: per learning tick
    dk' = dk (1 + a_K ((s / k)(u_ff / u) - 1)), neutral in a free-flight hover, GROWING wherever the throttle sits below
    the feed-forward (descents, a pinned loop, ground above the flare band).  Measured without k_sync: differences grew
    from 1e-7 to 1.8e-4 (6.5e-4 after a mismatch) over 1500-9000 ticks and pulled u / r / d_lpf past their tolerances.
    With k_sync every learning step, the clamps and the k_T that u_ff and d use are compared within one-step float32
    rounding (VC_KT_TOL); the reset at take-off and the clamps are also unit-tested in test_core."""
    return (f"{inp.phase},{r32(inp.h_tgt_m)},{int(inp.baro_lost)},{r32(inp.h_m)},{r32(inp.vz_mps)},{_q_text(q)},"
            f"{_f_text(inp.f_z_mps2)},{r32(k_sync)},{_f_text(inp.u_cap)}")


def vc_random_sequences(ref: Ref, rng: random.Random, n_seq: int) -> list[dict]:
    """Phase scripts (IDLE / take-off / CLIMB / HOLD with target changes / LAND, baro-lost landings, restarts) on a 1-D
    plant (ground below / at / above the pad) flown by the reference controller itself (closed loop on the reference,
    the same inputs then go to both), with noise, h / v_z jumps, tilts up to 80 deg as NAV quaternions, the body-z
    specific force of the plant (absent in some sequences: the floor signature's case), rare NaN / inf inputs and
    parameter variants (gains, ramp 0 / 1, confirmation and floor times, support threshold, still lead, the
    feed-forward off; v0.7.1b: plants x0.9 - 1.35 of the thrust model, the thrust learning off / tau 0.5 s / 2 s, its
    tilt guard 5 / 15 deg, k_T frozen at 0.7 or 1.4)."""
    seqs = []
    for s in range(n_seq):
        ov: dict = {}
        if s % 3:
            r = rng
            ov = dict(alt_kp=r.choice([0.5, 1.0, 2.5]), vz_kp=r.choice([0.08, 0.15, 0.4]), vz_ki=r.choice([0.0, 0.08, 0.3]),
                      vz_i_max=r.choice([0.05, 0.15, 0.3]), ramp_ticks=r.choice([0, 1, 125]),
                      landed_confirm_ticks=r.choice([1, 20, 125]), floor_ticks=r.choice([1, 50, 250]),
                      support_g=r.choice([0.1, 0.25, 0.5]), still_lead=r.choice([1.0, 2.0]), vz_ff=r.random() < 0.8)
            # v0.7.1b thrust learning: off / fast / the params' 2 s, a tighter tilt guard, sometimes k_T frozen at a clamp
            ov.update(kt_tau_s=r.choice([0.0, 0.5, 2.0, 2.0]), kt_max_tilt_deg=r.choice([15.0, 15.0, 5.0]))
            if r.random() < 0.2:
                kt = r.choice([0.7, 1.4])
                ov.update(kt_min=kt, kt_max=kt)
        vp = vc_params32(ref, ov)
        vc = VerticalController(vp)
        pl = Plant1D(ref, z_ground=rng.choice([0.0, -1.0, 0.8]), mass_scale=rng.choice([1.0, 0.85, 1.1]),
                     thrust_scale=rng.choice([1.0, 1.0, 0.9, 1.25, 1.35]))
        dt = vp.dt
        n = rng.choice([1500, 4000, 9000])
        no_accel = rng.random() < 0.3
        axis = rng.uniform(0.0, 2.0 * math.pi)
        h_bias = rng.choice([0.0, 0.0, 0.0, 1.5, 3.0])          # the NAV altitude reads high on the ground (ground above
        vz_bias = rng.choice([0.0, 0.0, 0.0, 0.2])               # the pad / baro offset): still is gated out, floor or
        rows, qs, py, befores, states, ksync = [], [], [], [], [], []  # support confirm
        phase, tgt, baro_lost, h_off = lp.PHASE_NONE, 0.0, False, 0.0
        t_next = rng.randrange(10, 200)
        u = 0.0
        tilt = 0.0
        for k in range(n):
            if k >= t_next:                                      # the next scripted event
                t_next = k + rng.randrange(50, 1500)
                x = rng.random()
                if phase == lp.PHASE_NONE:
                    phase, tgt, baro_lost = lp.PHASE_CLIMB, rng.choice([2.0, 5.0, 8.3, 12.0]), False
                elif phase in (lp.PHASE_CLIMB, lp.PHASE_HOLD):
                    if x < 0.5:
                        tgt = round(max(1.0, tgt + rng.choice([-1.0, 1.0, 0.5, -3.0, 4.0])), 1)
                        phase = rng.choice([phase, lp.PHASE_HOLD])
                    elif x < 0.8:
                        phase, baro_lost = lp.PHASE_LAND, rng.random() < 0.2
                        t_next = k + rng.randrange(1500, 6000)     # long enough to reach the ground and confirm
                    else:
                        phase = rng.choice([lp.PHASE_NONE, lp.PHASE_HOLD])
                else:
                    phase = rng.choice([lp.PHASE_NONE, lp.PHASE_LANDED, lp.PHASE_LAND, lp.PHASE_CLIMB])
                    if phase == lp.PHASE_LAND:
                        baro_lost = not baro_lost if rng.random() < 0.3 else baro_lost
            pl.step(u, dt)
            if phase in (lp.PHASE_NONE, lp.PHASE_SPOOL):
                h_off = pl.z
            tilt = min(max(tilt + rng.gauss(0.0, 0.3), 0.0), 20.0)   # v0.7.1b: was 15, the learning's tilt guard - a walk
            #   clamped exactly on it made a persistent knife-edge; now it crosses the guard (5 / 15 deg) and moves on
            h = pl.z - h_off + h_bias + rng.gauss(0.0, 0.03)
            vz = pl.v + vz_bias + rng.gauss(0.0, 0.03)
            fz = None if no_accel else pl.fz + rng.gauss(0.0, 0.3)
            ti = tilt
            y = rng.random()
            if y < 0.002:
                h += rng.choice([-5.0, 5.0, 30.0])
            elif y < 0.004:
                vz += rng.choice([-3.0, 3.0])
            elif y < 0.006:
                ti = rng.uniform(40.0, 80.0)
            elif y < 0.0065:
                h = rng.choice([math.nan, math.inf])
            elif y < 0.007:
                vz = rng.choice([math.nan, -math.inf])
            elif y < 0.0075:
                ti = rng.choice([math.nan, math.inf])
            elif y < 0.008 and fz is not None:
                fz = rng.choice([math.nan, math.inf, 3.0 * onb.G0])
            q = q_from_tilt(ti, axis)
            inp = VerticalInputs(phase=phase, h_tgt_m=f32(tgt), baro_lost=baro_lost, h_m=f32(h), vz_mps=f32(vz),
                                 cos_tilt=vcm.cos_tilt_from_q(q), f_z_mps2=None if fz is None else f32(fz))
            befores.append(vc_before(vc))
            ksync.append(vc.k_t)
            o = vc.update(inp)
            states.append(vc_state(vc))
            u = o.u if phase in (lp.PHASE_CLIMB, lp.PHASE_HOLD, lp.PHASE_LAND) else 0.0
            rows.append(inp)
            qs.append(q)
            py.append(o)
        seqs.append(dict(kind="random", overrides=ov, vp=vp, inputs=rows, q=qs, py=py, before=befores, states=states,
                         ksync=ksync))
    return seqs


VC_CONT = (("u", "u"), ("h_ref", "h_ref"), ("vz_cmd", "vz_cmd"), ("vz_err", "vz_err"), ("u_ff", "u_ff"), ("integ", "integ"),
           ("dh_ref", "dh_ref_mps"), ("cos_tilt", "cos_tilt"), ("v_lpf", "vz_lpf"), ("d_lpf", "f_excess_mps2"),
           ("r_thr", "thrust_ratio"), ("k_t", "k_thrust"))
VC_EXACT = (("mode", "mode"), ("sat", "sat"), ("at_target", "at_target"), ("touchdown", "touchdown"), ("td_count", "td_count"),
            ("td_by", "td_by"), ("low_in_land", "low_in_land_ticks"), ("ref_um", "ref_um"))


def vc_tolerances(vp: VerticalParams) -> dict:
    v = VC_VZ_TOL_MPS * max(1.0, vp.alt_kp)
    return {"u": VC_U_TOL, "u_ff": VC_U_TOL, "integ": VC_U_TOL, "h_ref": VC_H_TOL_M, "vz_cmd": v, "vz_err": v,
            "dh_ref": VC_VZ_TOL_MPS, "cos_tilt": VC_COS_TOL, "v_lpf": VC_VLPF_TOL_MPS, "d_lpf": VC_DLPF_TOL_MPS2,
            "r_thr": VC_RTHR_TOL, "k_t": VC_KT_TOL}


def vc_parse_c(parts: list[str], b: int) -> dict:
    """C put_vc_out columns from index b: VC_OUT_HEAD then floor_run, cnt0..3, k_t (test_core.c; v0.7.1b: cnt3, k_t)."""
    names = ("mode", "u", "h_ref", "vz_cmd", "vz_err", "u_ff", "integ", "sat", "at_target", "touchdown", "td_count", "sig",
             "td_by", "low_in_land", "dh_ref", "cos_tilt", "ref_um", "v_lpf", "d_lpf", "r_thr", "floor_run", "cnt0", "cnt1",
             "cnt2", "cnt3", "k_t")
    out = {}
    for j, k in enumerate(names):
        x = parts[b + j]
        out[k] = float(x) if k in ("u", "h_ref", "vz_cmd", "vz_err", "u_ff", "integ", "dh_ref", "cos_tilt", "v_lpf", "d_lpf",
                                   "r_thr", "k_t") else int(x)
    j = b + len(names)
    if len(parts) >= j + 6 and parts[j] == "V9":         # v0.9 (--vc-vectors only): airborne, to_passed, ramp_k,
        out.update(airborne=int(parts[j + 1]), to_passed=int(parts[j + 2]), ramp_k=int(parts[j + 3]),   # fz_lpf, td_bias
                   fz_lpf=float(parts[j + 4]), td_bias=float(parts[j + 5]))
    return out


def vc_prop_init() -> dict:
    return {"u_c": 0.0, "u_py": 0.0, "r_thr": 0.0, "d_lpf": 0.0}


def vc_prop_step(prop: dict, vp: VerticalParams, c: dict, o, f_z) -> None:
    """v0.7.1b: the part of the C-vs-reference difference of r_thr and d_lpf that their INPUTS carry, propagated through
    the same lags (call on every tick, compared or not): r <- (1 - a_T) r + a_T (u_last / u_hover)^2, so the applied
    throttles' difference (u, compared within VC_U_TOL - the integrator's float32 drift alone reaches 1.4e-5 over 6500
    ticks, i.e. 3e-5 of r at u ~ 1, beyond the old VC_RTHR_TOL that assumed u equal to 1e-6) moves r by
    a_T ((u_C / u_h)^2 - (u_py / u_h)^2) per tick; d <- (1 - a_S) d + a_S (f_z - k_T g r), so r's and k_T's differences
    move d by a_S (-k_T g dr - g r dk).  vc_compare then holds r_thr and d_lpf to their rounding tolerances AFTER
    removing these propagated parts, i.e. it checks each lag's own update exactly while the upstream differences stay
    within their own tolerances.  IDLE resets both sides (0)."""
    if o.mode == vcm.MODE_IDLE:
        prop.update(vc_prop_init())
        return
    xc, xp = prop["u_c"] * vp.inv_u_hover, prop["u_py"] * vp.inv_u_hover
    prop["r_thr"] = (1.0 - vp.thrust_alpha) * prop["r_thr"] + vp.thrust_alpha * (xc * xc - xp * xp)
    if f_z is not None and math.isfinite(f_z):
        dr, dk = c["r_thr"] - o.thrust_ratio, c["k_t"] - o.k_thrust
        prop["d_lpf"] = (1.0 - vp.support_alpha) * prop["d_lpf"] + vp.support_alpha * (
            -(o.k_thrust * vp.g) * dr - (vp.g * o.thrust_ratio) * dk)
    prop["u_c"], prop["u_py"] = c["u"], o.u


def vc_compare(c: dict, o, state: tuple, tol: dict, prop: dict | None = None) -> tuple[list[str], dict]:
    """The C outputs vs the reference's outputs and counters (state = vc_state after the update); returns (bad fields,
    |differences| of the continuous ones).  v0.7.1b: r_thr / d_lpf minus the propagated input differences (prop,
    vc_prop_step); dd also reports their raw differences as r_thr_raw / d_lpf_raw."""
    bad, dd = [], {}
    for ck, pk in VC_CONT:
        a, b = c[ck], float(getattr(o, pk))
        if math.isnan(a) and math.isnan(b):
            continue
        d = abs(a - b) if not (math.isinf(a) or math.isinf(b)) else (0.0 if a == b else math.inf)
        if prop is not None and ck in ("r_thr", "d_lpf") and math.isfinite(d):
            dd[ck + "_raw"] = d
            d = abs((a - b) - prop[ck])
        dd[ck] = d
        if not d <= tol[ck]:
            bad.append(ck)
    for ck, pk in VC_EXACT:
        if c[ck] != int(getattr(o, pk)):
            bad.append(ck)
    if c["sig"] != vc_sig(o):
        bad.append("sig")
    if (c["floor_run"], c["cnt0"], c["cnt1"], c["cnt2"], c["cnt3"]) != tuple(min(x, 0xFFFF) for x in state[:5]):
        bad.append("counters")
    if "airborne" in c and len(state) > 10:                # v0.9: the latches exact, fz_lpf (FLY / IDLE) or td_bias (LAND)
        if (c["airborne"], c["to_passed"], c["ramp_k"]) != (int(state[6]), int(state[7]), int(state[8])):
            bad.append("v09_latches")
        for key, want, tl in (("fz_lpf", state[9], VC_DLPF_TOL_MPS2), ("td_bias", state[10], VC_TD_TOL)):
            d = abs(c[key] - want)
            dd[key] = d
            if not d <= tl:
                bad.append(key)
    return bad, dd


def check_vertical(seqs: list[dict]) -> dict:
    """float32 C (core/vertical_ctrl.c) vs float64 reference (sim/vertical_ctrl.py) on the same input sequences.
    Continuous outputs within the VC_* tolerances; mode, sat, at_target, touchdown, the signature flags, td_by, every
    counter, low_in_land and the integer reference exact.  A tick whose float64 decision margins lie within the float32
    error bound (vc_margins) is ambiguous: when it differs, VC_RESYNC_TICKS ticks are skipped and the next must agree
    again, else the sequence is compared up to it only (its state may legitimately branch) and counted; any other
    difference fails."""
    fin, fout = os.path.join(BUILD, "vc_in.txt"), os.path.join(BUILD, "vc_out.csv")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        for i, sq in enumerate(seqs):
            f.write(f"#V {i} {_ov_text(vc_c_overrides(sq['vp']), 'vc.')}\n")
            f.write("\n".join(_vc_row(x, q, ks) for x, q, ks in zip(sq["inputs"], sq["q"], sq["ksync"])) + "\n")
    run_exe(["--vc-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        f.readline()
        c_lines = f.read().splitlines()
    st = {"sequences": len(seqs), "ticks": 0, "ticks_compared": 0, "mismatch_ticks": 0, "ambiguous_sequences": 0,
          "ambiguous_ticks": 0, "ambiguous_cut": [], "knife_edges": {}, "knife_edges_resynced": 0,
          "first_mismatches": [], "max_abs": {**{ck: 0.0 for ck, _ in VC_CONT}, "r_thr_raw": 0.0, "d_lpf_raw": 0.0,
                                              "fz_lpf": 0.0, "td_bias": 0.0},
          "ticks_by_mode": {}, "sat_ticks": {"+1": 0, "-1": 0}, "at_target_ticks": 0, "touchdowns": 0,
          "touchdowns_by": {"still": 0, "support": 0, "floor": 0, "all": 0}, "baro_lost_land_ticks": 0, "nonfinite_inputs": 0,
          "kt_learn_ticks": 0, "kt_range": [math.inf, -math.inf],
          "tolerances": {"h_ref_m": VC_H_TOL_M, "vz_mps": VC_VZ_TOL_MPS, "u": VC_U_TOL, "cos_tilt": VC_COS_TOL,
                         "v_lpf_mps": VC_VLPF_TOL_MPS, "d_lpf_mps2": VC_DLPF_TOL_MPS2, "r_thr": VC_RTHR_TOL,
                         "k_t": VC_KT_TOL}}
    i = 0
    for si, sq in enumerate(seqs):
        vp = sq["vp"]
        tol = vc_tolerances(vp)
        stop, skip_until, check_resync = False, -1, False
        prop = vc_prop_init()
        for k, (inp, o, bef, sta) in enumerate(zip(sq["inputs"], sq["py"], sq["before"], sq["states"])):
            parts = c_lines[i].split(",")
            i += 1
            if int(parts[0]) != si or int(parts[1]) != k:
                raise SystemExit(f"vc-vectors row order mismatch at {i}")
            st["ticks"] += 1
            c = vc_parse_c(parts, 2)
            vc_prop_step(prop, vp, c, o, inp.f_z_mps2)
            if stop or k <= skip_until:
                continue
            amb, _ = vc_margins(vp, bef, inp, o, sta)
            st["ambiguous_ticks"] += int(amb)
            bad, dd = vc_compare(c, o, sta, tol, prop)
            if check_resync:                                     # the first tick after a resync window
                check_resync = False
                if bad:                                          # a lasting branch (e.g. one integrator step): cut
                    st["ambiguous_sequences"] += 1
                    st["ambiguous_cut"].append({"seq": si, "tick": k, "fields": bad})
                    stop = True
                    continue
                st["knife_edges_resynced"] += 1
            if bad and amb:                                      # float32 took the other side of a knife-edge decision
                st["knife_edges"][",".join(bad)] = st["knife_edges"].get(",".join(bad), 0) + 1
                skip_until, check_resync = k + VC_RESYNC_TICKS, True
                continue
            for key, d in dd.items():
                st["max_abs"][key] = max(st["max_abs"][key], d)
            st["ticks_compared"] += 1
            mn = vcm.MODE_NAMES[o.mode]
            st["ticks_by_mode"][mn] = st["ticks_by_mode"].get(mn, 0) + 1
            if o.sat:
                st["sat_ticks"]["+1" if o.sat > 0 else "-1"] += 1
            st["at_target_ticks"] += int(o.at_target)
            if o.touchdown and o.td_count == vp.landed_confirm_ticks:
                st["touchdowns"] += 1
                st["touchdowns_by"][vcm.TD_NAMES[o.td_by]] += 1
            st["baro_lost_land_ticks"] += int(o.mode == vcm.MODE_LAND and inp.baro_lost)
            st["kt_learn_ticks"] += int(o.kt_learn)
            st["kt_range"] = [min(st["kt_range"][0], o.k_thrust), max(st["kt_range"][1], o.k_thrust)]
            st["nonfinite_inputs"] += int(not all(map(math.isfinite, (inp.h_m, inp.vz_mps, inp.cos_tilt))))
            if bad:
                st["mismatch_ticks"] += 1
                if len(st["first_mismatches"]) < 10:
                    st["first_mismatches"].append({"seq": si, "tick": k, "fields": bad,
                                                   "input": _vc_row(inp, sq["q"][k], sq["ksync"][k]),
                                                   "c": {x: c.get(x) for x in bad},
                                                   "py": {x: getattr(o, dict(VC_CONT + VC_EXACT).get(x, x), None)
                                                          for x in bad}})
    if i != len(c_lines):
        raise SystemExit(f"C produced {len(c_lines)} vc rows, expected {i}")
    st["pass"] = (st["mismatch_ticks"] == 0 and st["ticks_compared"] > 0.95 * st["ticks"] and st["touchdowns"] > 0
                  and all(v > 0 for v in st["touchdowns_by"].values()) and st["kt_learn_ticks"] > 0
                  and st["sat_ticks"]["+1"] > 0 and st["sat_ticks"]["-1"] > 0 and st["at_target_ticks"] > 0)
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


# ---- FCU stack replays (1-D closed loops in this file, 6-DOF VTOL runs of sim/vtol_scenarios.py) ----------------- #
def stack_1d_flights(ref: Ref, rng: random.Random, n: int) -> list[dict]:
    """Closed-loop flights of the Python FcuStack on Plant1D with a scripted ground (ON, UP / DOWN, OFF, sometimes a
    link outage, a baro failure, a KILL, a second hop): the supervisor's sensor and frame inputs per tick are recorded
    (float32) for the replay into both stacks.  v0.7.1: the NAV quaternion (from the tilt) and the plant's body-z
    specific force ride along (the controller's cos(tilt) and support signature)."""
    sp = sup_params_vtol(ref)
    vp = vc_params32(ref)
    runs = []
    for s in range(n):
        seed = rng.randrange(65536)
        fcu = onb.FcuStack(sp, vp, seed=seed)
        pl = Plant1D(ref, z_ground=rng.choice([0.0, -1.5, 0.5]), mass_scale=rng.choice([1.0, 1.0, 0.92, 1.05]),
                     thrust_scale=rng.choice([1.0, 1.0, 1.25, 0.9]))      # v0.7.1b: plants off the thrust model
        seq, h_off, recs = 10, 0.0, []
        t_on = rng.randrange(20, 300)
        script = {t_on: lp.CMD_ON}
        t = t_on + sp.spool_ticks + rng.randrange(800, 2500)
        for _ in range(rng.randrange(0, 5)):
            script[t] = rng.choice([lp.CMD_UP, lp.CMD_UP, lp.CMD_DOWN])
            t += rng.randrange(150, 900)
        fault = rng.choice(["", "", "outage", "baro", "kill", "rehop", "none_off"])
        if fault != "none_off":
            script[t + rng.randrange(200, 1500)] = lp.CMD_KILL if fault == "kill" else lp.CMD_OFF
        t_fault = t_on + sp.spool_ticks + rng.randrange(1000, 3000)
        n_ticks = rng.choice([9000, 14000]) if fault != "none_off" else 16000
        noise = rng.choice([0.0, 0.02, 0.05])
        no_accel = rng.random() < 0.25
        axis = rng.uniform(0.0, 2.0 * math.pi)
        for k in range(n_ticks):
            frames = []
            cmd = script.get(k)
            if fault == "rehop" and k > t_on + 1000 and fcu.sup.state == onb.State.SAFE and k % 400 == 0:
                cmd = lp.CMD_ON
            link = not (fault == "outage" and t_fault <= k < t_fault + 3 * sp.link_land_ticks)
            if link and (cmd is not None or k % 50 == 0):
                if cmd is not None:
                    seq = (seq + 1) & 0xFF
                frames = [onb.UplinkFrame(seq, lp.Command(lp.CMD_NONE if cmd is None else cmd, 0, fcu.sup.nonce))]
            if fcu.sup.state in (onb.State.SAFE, onb.State.ARMED):
                h_off = pl.z
            tilt = f32(abs(rng.gauss(0.5, 0.3)))
            inp = onb.OnboardInputs(
                tick=k, frames=frames, h_m=f32(pl.z - h_off + rng.gauss(0.0, noise)), vz_mps=f32(pl.v + rng.gauss(0.0, noise)),
                tilt_deg=tilt, gyro_rad_s=(f32(rng.gauss(0, 0.05)), f32(rng.gauss(0, 0.05)), f32(rng.gauss(0, 0.05))),
                v_cell=f32(4.0 - 0.3 * k / 16000), baro_ok=not (fault == "baro" and k >= t_fault), roll_deg=0.0,
                pitch_deg=0.0, yaw_deg=f32(rng.uniform(-180, 180)), loop_max_us=f32(2900.0), q_nav=q_from_tilt(tilt, axis),
                accel_z_mps2=None if no_accel else f32(pl.fz + rng.gauss(0.0, 0.2 + 5 * noise)))
            recs.append(inp)
            fo = fcu.step(inp)
            pl.step(fo.throttle, 1.0 / sp.loop_hz)
        runs.append(dict(name=f"1d_{s}_{fault or 'nominal'}{'_noaccel' if no_accel else ''}", seed=seed, inputs=recs,
                         reset=False))
    return runs


# ---- 6-DOF recordings (sections 4d and 7b; fw09 X1: with the run's own parameters) ------------------------------- #
# The 6-DOF VTOL runs of sim/vtol_scenarios.py are recorded in a CHILD process whose sys.path starts at --sim-root
# (default: this project; another directory = a read-only snapshot copy of sim/ + params.json + the files sim reads, so
# a sim/ being edited by someone else is never read half-way).
# fw09 X1 (SPEC_firmware_v09 3.2 / WP-X): every run is flown the way the reference flies it (sim.vtol_scenarios
# run_scenario: the vehicle of the scenario's power source, load_vehicle(config='vtol', power=...), and the attitude gains
# of that vehicle's params - until v0.9 this recorder flew the VTOL scenarios on load_vehicle(), the ROCKET airframe of
# mission.config), and the record carries, next to the inputs, the parameter objects the run flew with and the NOMINAL
# objects of its configuration (the same constructors on a scenario that has only the run's profile, power source and
# build switches).  Before any replay, check_sixdof_params compares the nominal objects with the constants of the host
# core compiled for that configuration (test_core --consts, -D FC_VCFG / -D MISSION_PROFILE_*): a firmware build that
# does not carry its configuration's constants - the G1 defect, every VTOL image flying the rocket's l_v 351 mm and hover
# throttle 0.916 - fails there.  The same hooks (SIXDOF_HOOKS, read from this file's source) record the eval harness's
# runs (eval_v09g.py --record) for --campaign-set.
#
# Record format "ccrec/1": one numpy .npz per run (np.savez_compressed, no pickles):
#   meta    str (JSON)  {format, run: {kind 'named' | 'run', name, set, seed, group, i, k, source}, params, nominal,
#                        nominal_error, stack: {seed, reset, chain, cols, slot_len, ticks}, events, sim_root}
#           params / nominal = {veh: {power_source, hardware_config, mass_kg, z_cg_m, ixx, iyy, izz, l_v_m, r_v_m, s_v_m2,
#                        a_e_m2, t_max_n}, plant: MixerPlant fields (no servo signs / trims), gains: the attitude gains
#                        dict, sp / vp / pp: dataclasses.asdict of SupervisorParams / VerticalParams / PositionParams (pp
#                        null without the position hold), est: {dt, n_avg, kp, ki, accel_gate, g, alpha_a, bias_limit},
#                        alt: {k1, k2, k3, latency, pad_bias_tau_s, bias_limit, comp_m, inv_u_hover, throttle_exp},
#                        switches: {profile, gps_on, v08, tko, chain, power_source, estimator}}
#   stack   float64 (ticks x len(STACK_COLS)): every FcuStack.step input of the run's FlightSim (None -> NaN with a has_*
#           column); frames int64 (n x 5: stack row, seq, cmd, arg, nonce); slots int32 (n x (1 + slot_len): row, bytes)
#   est     float64 (n x 7): GatedMahonyEstimator.update(gyro xyz, accel xyz, on_pad)
#   alt     float64 (n x 7): VtolAltitudeFilter.step(a, dt, h_baro (NaN = None), has_baro, t, pad, u_prev)
# Only the first FlightSim.run of a job is recorded (a named scenario that flies more than one is recorded by its first).
SIXDOF_HOOKS = r'''
import dataclasses as _dc
import json as _json
import math as _math
import numpy as _np
from sim import estimator as _E, onboard as _O, state_machine as _SM, vtol_scenarios as _vts
from sim import sim6dof as _S6
CCREC_FORMAT = "ccrec/1"
STACK_COLS = ("tick", "h_m", "vz_mps", "tilt_deg", "gyro0", "gyro1", "gyro2", "gsat", "v_cell", "imu_ok", "baro_ok",
              "aligned", "arm", "roll", "pitch", "yaw", "loop_max_us", "q0", "q1", "q2", "q3", "has_q", "az", "has_az",
              "cos", "has_cos", "v_in", "has_v_in", "v_tether", "has_v_tether", "v_buffer", "has_v_buffer", "buf_i_pos",
              "has_buf_i_pos")
_REC = {"on": False, "cap": False}


def _jd(o):
    if hasattr(o, "tolist"):
        return o.tolist()
    if isinstance(o, (set, frozenset)):
        return sorted(o)
    return str(o)


def _jsafe(d):
    return _json.loads(_json.dumps(d, default=_jd))


def ccrec_params(fs, est, valt) -> dict:
    """The parameter objects a FlightSim flies (the flight computer's side), JSON-safe."""
    v, pl = fs.veh, fs.ctrl.mixer.p
    inertia = _np.asarray(v.full.inertia_kg_m2, dtype=float)
    diag = _np.diag(inertia) if inertia.ndim == 2 else inertia
    sc = fs.sc
    out = {"veh": {"power_source": getattr(v, "power_source", None), "hardware_config": getattr(v, "hardware_config", None),
                   "mass_kg": float(v.full.mass_kg), "z_cg_m": float(v.z_cg_m), "ixx": float(diag[0]), "iyy": float(diag[1]),
                   "izz": float(diag[2]), "l_v_m": float(v.l_v_m), "r_v_m": float(v.r_v_m), "s_v_m2": float(v.s_v_m2),
                   "a_e_m2": float(v.a_e_m2), "t_max_n": float(v.t_max_n)},
           "plant": {f.name: float(getattr(pl, f.name)) for f in _dc.fields(pl)
                     if f.name not in ("servo_signs", "servo_trims_us")},
           "gains": {k: x for k, x in fs.gains.items() if isinstance(x, (int, float, bool)) and not k.startswith("_")},
           "sp": _jsafe(_dc.asdict(fs.sp)),
           "vp": None if fs.vp is None else _jsafe(_dc.asdict(fs.vp)),
           "pp": None if getattr(fs, "pp", None) is None else _jsafe(_dc.asdict(fs.pp)),
           "est": None if est is None else {"dt": float(est.dt), "n_avg": int(est.n_avg), "kp": float(est.kp),
                                            "ki": float(est.ki), "accel_gate": float(est.accel_gate), "g": float(est.g),
                                            "alpha_a": float(est.alpha_a), "bias_limit": float(est.bias_limit)},
           "alt": None if valt is None else {"k1": float(valt.k1), "k2": float(valt.k2), "k3": float(valt.k3),
                                             "latency": float(valt.latency), "pad_bias_tau_s": float(valt.pad_bias_tau_s),
                                             "bias_limit": float(valt.bias_limit), "comp_m": float(valt.comp_m),
                                             "inv_u_hover": float(valt.inv_u_hover), "throttle_exp": float(valt.throttle_exp)},
           "switches": {"profile": sc.profile, "gps_on": bool(fs.gps_on), "v08": bool(fs.v08), "tko": bool(fs.tko),
                        "chain": str(fs.chain), "power_source": str(getattr(sc, "power_source", None) or "battery"),
                        "estimator": str(sc.sensors.attitude_estimator)}}
    return out


def ccrec_nominal(fs) -> dict:
    """The NOMINAL parameter objects of the run's configuration: the reference's constructors (FlightSim.__init__ and the
    estimator line of FlightSim.run) on a scenario that keeps only the run's profile, power source and build switches
    (GPS build or not, the v0.8 NAV order, the take-off check) with the tuned gains of sim/out/vtol_gains.json - what the
    firmware build of that configuration must carry."""
    sc = fs.sc
    vg, _src = _vts.load_vgains()
    sc_n = _vts.base_vtol("ccrec_nominal", "nominal parameters", vg, profile=sc.profile,
                          power_source=getattr(sc, "power_source", None),
                          gps=(_vts.GPS_DEFAULT if fs.gps_on else None), nav_v08=bool(fs.v08), takeoff_check=bool(fs.tko))
    fs_n = _S6.FlightSim(fs.veh, sc_n, _vts.attitude_gains(fs.veh.params))
    sens, vga = sc_n.sensors, (sc_n.vgains or {})

    def _eg(sv, key):
        return sv if sv is not None else (None if vga.get(key) is None else float(vga[key]))
    est = _E.GatedMahonyEstimator.from_params(fs_n.p, fs_n.dt_c, sens.pad_avg_tau_s, g=_S6.G0, kp=_eg(sens.gm_kp, "est_kp"),
                                              ki=_eg(sens.gm_ki, "est_ki"),
                                              accel_lpf_hz=_eg(sens.gm_accel_lpf_hz, "est_accel_lpf_hz"),
                                              accel_gate=sens.gm_accel_gate)
    return ccrec_params(fs_n, est, fs_n.valt)


_u0, _s0 = _E.GatedMahonyEstimator.update, _SM.VtolAltitudeFilter.step
_f0, _fi0, _r0 = _O.FcuStack.step, _O.FcuStack.__init__, _S6.FlightSim.run


def _upd(self, gyro, accel, on_pad):
    if _REC["cap"]:
        _REC.setdefault("est_obj", self)
        _REC.setdefault("est", []).append([*map(float, gyro), *map(float, accel), float(bool(on_pad))])
    return _u0(self, gyro, accel, on_pad)


def _stp(self, a, dt, h_baro, t, pad, u_prev=0.0):
    if _REC["cap"]:
        _REC.setdefault("alt", []).append([float(a), float(dt), _math.nan if h_baro is None else float(h_baro),
                                           float(h_baro is not None), float(t), float(bool(pad)), float(u_prev)])
    return _s0(self, a, dt, h_baro, t, pad, u_prev)


def _finit(self, sp, vp=None, seed=0, reset_in_flight=False, pp=None, chain="fcu"):
    self._ccrec = (int(seed), bool(reset_in_flight), str(chain))
    return _fi0(self, sp, vp, seed=seed, reset_in_flight=reset_in_flight, pp=pp, chain=chain)


def _o(x):
    return (_math.nan, 0.0) if x is None else (float(x), 1.0)


def _fstep(self, inp):
    if _REC["cap"]:
        st = _REC.setdefault("stack", [])
        k = len(st)
        q = getattr(inp, "q_nav", None)
        bip = getattr(inp, "buf_i_pos", None)
        row = [float(inp.tick), float(inp.h_m), float(inp.vz_mps), float(inp.tilt_deg), *map(float, inp.gyro_rad_s),
               float(bool(inp.gyro_saturated)), float(inp.v_cell), float(bool(inp.imu_ok)), float(bool(inp.baro_ok)),
               float(bool(inp.aligned)), float(bool(inp.arm_switch)), float(inp.roll_deg), float(inp.pitch_deg),
               float(inp.yaw_deg), float(inp.loop_max_us)]
        row += [*map(float, q), 1.0] if q is not None else [_math.nan] * 4 + [0.0]
        for nm in ("accel_z_mps2", "cos_tilt", "v_in", "v_tether", "v_buffer"):
            row += _o(getattr(inp, nm, None))
        row += [_math.nan, 0.0] if bip is None else [float(bool(bip)), 1.0]
        st.append(row)
        for fr in inp.frames:
            _REC.setdefault("frames", []).append([k, int(fr.seq), int(fr.cmd.cmd), int(fr.cmd.arg), int(fr.cmd.nonce)])
        sl = getattr(inp, "gps_slot", None)
        if sl is not None:
            _REC.setdefault("slots", []).append([k, *bytes(sl)])
    return _f0(self, inp)


def _run(self, *a, **kw):
    if _REC["on"] and "fs" not in _REC:
        _REC["fs"], _REC["cap"] = self, True
        try:
            return _r0(self, *a, **kw)
        finally:
            _REC["cap"] = False
    return _r0(self, *a, **kw)


def ccrec_install():
    _E.GatedMahonyEstimator.update, _SM.VtolAltitudeFilter.step = _upd, _stp
    _O.FcuStack.__init__, _O.FcuStack.step, _S6.FlightSim.run = _finit, _fstep, _run


def ccrec_begin():
    _REC.clear()
    _REC.update(on=True, cap=False)


def ccrec_abort():
    _REC.clear()
    _REC.update(on=False, cap=False)


def ccrec_end(path, run_info, res=None, sim_root=None) -> str:
    """Write the record of the job's first FlightSim run to ``path``; returns a one-line summary."""
    _REC["on"] = False
    fs = _REC.get("fs")
    if fs is None or not getattr(fs, "vtol", False) or not _REC.get("stack"):
        _REC.clear()
        _REC.update(on=False, cap=False)
        return "no VTOL FlightSim run recorded"
    params = ccrec_params(fs, _REC.get("est_obj"), fs.valt)
    try:
        nominal, nerr = ccrec_nominal(fs), None
    except Exception as exc:                       # recorded: the parent's check fails on a run without its nominal
        nominal, nerr = None, repr(exc)
    seed, reset, chain = getattr(fs.fcu, "_ccrec", (0, False, "fcu"))
    slots = _REC.get("slots") or []
    slen = max((len(s) - 1 for s in slots), default=0)
    meta = {"format": CCREC_FORMAT, "run": run_info, "params": params, "nominal": nominal, "nominal_error": nerr,
            "stack": {"seed": seed, "reset": reset, "chain": chain, "cols": list(STACK_COLS), "slot_len": slen,
                      "ticks": len(_REC["stack"])},
            "events": None if res is None else len(res.events), "sim_root": sim_root}
    _np.savez_compressed(
        path, meta=_np.array(_json.dumps(meta, default=_jd)), stack=_np.array(_REC["stack"], dtype=float),
        frames=_np.array(_REC.get("frames") or _np.zeros((0, 5)), dtype=_np.int64).reshape(-1, 5),
        slots=_np.array([s + [0] * (slen + 1 - len(s)) for s in slots] or _np.zeros((0, slen + 1)),
                        dtype=_np.int32).reshape(-1, slen + 1),
        est=_np.array(_REC.get("est") or _np.zeros((0, 7)), dtype=float).reshape(-1, 7),
        alt=_np.array(_REC.get("alt") or _np.zeros((0, 7)), dtype=float).reshape(-1, 7))
    msg = (f"{len(_REC['stack'])} stack ticks, {len(_REC.get('est') or [])} estimator / {len(_REC.get('alt') or [])} "
           f"filter steps, {params['veh']['hardware_config']} {params['veh']['power_source']} vehicle, profile "
           f"{params['switches']['profile'] or 'field'}" + ("" if nerr is None else f", NOMINAL FAILED {nerr}"))
    _REC.clear()
    _REC.update(on=False, cap=False)
    return msg
'''
SIXDOF_RECORDER = r'''
import os, sys
root, out_dir, names = sys.argv[1], sys.argv[2], sys.argv[3:]
sys.path.insert(0, root)
''' + SIXDOF_HOOKS + r'''
ccrec_install()
_vg, _src = _vts.load_vgains()
# the scenarios as sim/vtol_scenarios.py run_named builds them (vehicle().params, the tuned vertical / estimator gains of
# sim/out/vtol_gains.json); flown as run_scenario flies them (the scenario's power source, that vehicle's attitude gains)
_scs = {sc.name: sc for sc, _ in _vts.scenarios(_vts.vehicle().params, _vg)}
for nm in names:
    if nm not in _scs:
        print(f"REC6 {nm}: no such scenario", flush=True)
        continue
    sc = _scs[nm]
    veh = _vts.vehicle(sc.power_source)
    ccrec_begin()
    try:
        res = _S6.FlightSim(veh, sc, _vts.attitude_gains(veh.params)).run()
    except Exception as exc:
        ccrec_abort()
        print(f"REC6 {nm}: {exc!r}", flush=True)
        continue
    print(f"REC6 {nm}: " + ccrec_end(os.path.join(out_dir, f"rec_{nm}.npz"), {"kind": "named", "name": nm,
                                                                           "source": "crosscheck"}, res, root), flush=True)
'''
# the nominal parameter objects of each configuration's flight builds (section 2, per configuration): the reference's
# constructors as above, without a run
SIXDOF_NOMINALS = r'''
import json, os, sys
root, out = sys.argv[1], sys.argv[2]
sys.path.insert(0, root)
''' + SIXDOF_HOOKS + r'''
_vg, _src = _vts.load_vgains()
res = {}
# (power source, profile, the build's switches: the battery builds fly the GPS position hold (four-chip chain), the
# tether / hybrid FCU images are ONB_NO_GPS (the v0.8 build without the GPS: v0.8 NAV order, take-off check)
for pw in ("battery", "tether", "hybrid"):
    for prof in (None, "demo"):
        gps = pw == "battery"
        sc = _vts.base_vtol("ccrec_nominal", "nominal parameters", _vg, profile=prof, power_source=pw,
                            gps=(_vts.GPS_DEFAULT if gps else None), nav_v08=True, takeoff_check=True)
        try:
            fs = _S6.FlightSim(_vts.vehicle(pw), sc, _vts.attitude_gains(_vts.vehicle(pw).params))
            res[f"{pw}/{prof or 'field'}"] = {"nominal": ccrec_nominal(fs), "error": None}
        except Exception as exc:
            res[f"{pw}/{prof or 'field'}"] = {"nominal": None, "error": repr(exc)}
with open(out, "w", encoding="utf-8") as f:
    json.dump(res, f, default=_jd)
print("NOMINALS " + ", ".join(f"{k}: {'ok' if v['error'] is None else v['error']}" for k, v in res.items()), flush=True)
'''
VTOL_6DOF_RUNS = ("V_nominal_wind", "V_wind_swing", "V_ground_below", "V_link_loss", "V_baro_fail",
                  "V_flight_time_alt_max", "V_stuck_vane", "D_demo_nominal", "D_push_cg", "D_baro_offset_cal")
NAV_6DOF_RUNS = VTOL_6DOF_RUNS


def _run_child(src: str, cwd: str, args: list[str]) -> tuple[int, str]:
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    r = subprocess.run([sys.executable, "-c", src, *args], cwd=cwd, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", env=env)
    return r.returncode, r.stdout + r.stderr


def record_6dof(sim_root: str, out_dir: str, names: tuple[str, ...]) -> str:
    """Run SIXDOF_RECORDER on the sim at ``sim_root`` (one rec_<name>.npz per scenario); returns its log."""
    os.makedirs(out_dir, exist_ok=True)
    for old in (glob.glob(os.path.join(out_dir, "rec_*.npz")) + glob.glob(os.path.join(out_dir, "stack_*.json")) +
                glob.glob(os.path.join(out_dir, "nav_*.npz"))):
        os.remove(old)
    rc, txt = _run_child(SIXDOF_RECORDER, sim_root, [sim_root, out_dir, *names])
    lines = [x for x in txt.splitlines() if x.startswith("REC6") or "Error" in x]
    return "; ".join(lines)[-4000:] + ("" if rc == 0 else f" (recorder exit {rc})")


def load_ccrec(path: str) -> dict:
    """A ccrec/1 record -> {name, meta, params, nominal, run (check_stack's run dict), nav (est / alt arrays + meta)}."""
    with np.load(path, allow_pickle=False) as z:
        meta = json.loads(str(z["meta"]))
        st, fr, sl = np.array(z["stack"]), np.array(z["frames"]), np.array(z["slots"])
        est, alt = np.array(z["est"]), np.array(z["alt"])
    if meta.get("format") != "ccrec/1":
        raise SystemExit(f"{path}: not a ccrec/1 record ({meta.get('format')!r})")
    cols = {c: i for i, c in enumerate(meta["stack"]["cols"])}
    frames: dict[int, list] = {}
    for row, s_, c_, a_, n_ in fr.tolist():
        frames.setdefault(int(row), []).append(onb.UplinkFrame(int(s_), lp.Command(int(c_), int(a_), int(n_))))
    slots = {int(r[0]): bytes(int(x) for x in r[1:]) for r in sl.tolist()}

    def opt(r, nm):
        return None if r[cols["has_" + nm]] == 0.0 else float(r[cols[nm]])
    inputs = []
    for k, r in enumerate(st.tolist()):
        q = None if r[cols["has_q"]] == 0.0 else tuple(r[cols["q0"]:cols["q0"] + 4])
        inputs.append(onb.OnboardInputs(
            tick=int(r[cols["tick"]]), frames=frames.get(k, []), h_m=r[cols["h_m"]], vz_mps=r[cols["vz_mps"]],
            tilt_deg=r[cols["tilt_deg"]], gyro_rad_s=tuple(r[cols["gyro0"]:cols["gyro0"] + 3]),
            gyro_saturated=bool(r[cols["gsat"]]), v_cell=r[cols["v_cell"]], imu_ok=bool(r[cols["imu_ok"]]),
            baro_ok=bool(r[cols["baro_ok"]]), aligned=bool(r[cols["aligned"]]), arm_switch=bool(r[cols["arm"]]),
            roll_deg=r[cols["roll"]], pitch_deg=r[cols["pitch"]], yaw_deg=r[cols["yaw"]],
            loop_max_us=r[cols["loop_max_us"]], q_nav=q, accel_z_mps2=opt(r, "az"), cos_tilt=opt(r, "cos"),
            gps_slot=slots.get(k), v_in=opt(r, "v_in"), v_tether=opt(r, "v_tether"), v_buffer=opt(r, "v_buffer"),
            buf_i_pos=None if r[cols["has_buf_i_pos"]] == 0.0 else bool(r[cols["buf_i_pos"]])))
    run_info = meta.get("run") or {}
    name = str(run_info.get("name") or os.path.splitext(os.path.basename(path))[0])
    prm = meta["params"]
    run = dict(name="6dof_" + name, seed=int(meta["stack"]["seed"]), inputs=inputs, reset=bool(meta["stack"]["reset"]),
               events=meta.get("events"), record=os.path.basename(path))
    spr = onb.SupervisorParams(**prm["sp"])
    run["sp"] = dataclasses.replace(spr, **{k: f32(getattr(spr, k)) for k in VTOL_FLOAT_FIELDS})
    vpr = VerticalParams(**prm["vp"])
    run["vp"] = dataclasses.replace(vpr, **{k: f32(getattr(vpr, k)) for k in VC_FLOAT_FIELDS})
    run["profile"] = prm["sp"].get("profile", "") or ""
    if prm.get("pp"):                                    # v0.8: a run with position hold (both sides: float32 constants)
        run["pp"] = pos_params32(pcm.PositionParams(**prm["pp"]))
        run["chain"] = str(meta["stack"].get("chain", "fcu"))
    run["vcfg"], run["host_profile"] = run_config(prm)
    run["power_build"] = POWER_BUILD.get(prm["switches"]["power_source"], 0) if run["vcfg"] in (1, 2) else 0
    nav = None
    if prm.get("est") and prm.get("alt") and len(est) and len(alt):
        nav = dict(name="6dof_" + name, meta={"name": name, "est": prm["est"], "alt": prm["alt"]}, est=est, alt=alt)
    return dict(name=name, path=path, meta=meta, params=prm, nominal=meta.get("nominal"),
                nominal_error=meta.get("nominal_error"), run=run, nav=nav)


def sixdof_records(a) -> tuple[str, str]:
    """The directory with the 6-DOF recordings (ccrec/1 rec_<name>.npz) for this run and a note on their origin:
    --replays DIR as given, else recorded now from --sim-root (default: this project) into the build directory."""
    if a.replays:
        return a.replays, f"recordings from {a.replays}"
    out = os.path.join(BUILD, "sixdof_records")
    names = tuple(dict.fromkeys(list(a.vtol_runs) + list(a.nav_runs)))
    root = a.sim_root or ROOT
    log = record_6dof(root, out, names)
    return out, f"recorded now from {root}: {log}"


def load_sixdof_records(rec_dir: str, names) -> list[dict]:
    out = []
    for nm in names:
        path = os.path.join(rec_dir, f"rec_{nm}.npz")
        if os.path.exists(path):
            out.append(load_ccrec(path))
    return out


# ---- fw09 X1: the firmware build's constants per configuration (test_core --consts) ------------------------------ #
POWER_BUILD = {"battery": 0, "tether": 1, "hybrid": 2}      # sim.power_link builds = core/power_mon.h PWR_SRC_*
VCFG_POWER = {0: "battery", 1: "tether", 2: "hybrid", 3: "rocket"}
_CONSTS: dict = {}
_BUILT: set = set()


def run_config(prm: dict) -> tuple[int, str | None]:
    """(FC_VCFG, host build profile) of a recorded run: the vehicle it flew (hardware_config / power_source) and the
    build profile of its supervisor (None = the header's default 'field')."""
    veh = prm["veh"]
    vcfg = 3 if veh.get("hardware_config") != "vtol" else bht.VCFG_OF_POWER.get(str(veh.get("power_source")), -1)
    if vcfg < 0:
        raise SystemExit(f"recorded run: unknown power source {veh.get('power_source')!r}")
    prof = prm["sp"].get("profile") or prm["switches"].get("profile") or "field"
    if not re.fullmatch(r"[a-z][a-z0-9_]*", str(prof)):
        raise SystemExit(f"recorded run: bad profile {prof!r}")
    return vcfg, (None if prof == "field" else str(prof))


def host_exe(profile: str | None, vcfg: int | None) -> str:
    """The host core of a configuration (built once per crosscheck run): -D FC_VCFG, the build profile, and for the
    tether / hybrid vehicles -D PWR_SRC = FC_VCFG as their FCU envs (fcu_wired / fcu_wired_buffer)."""
    pwr = vcfg if vcfg in (1, 2) else None
    key = (profile, vcfg, pwr)
    if key not in _BUILT:
        bht.build(False, profile, vcfg, pwr)
        _BUILT.add(key)
    return bht.exe_path(profile, vcfg, pwr)


def c_consts(profile: str | None, vcfg: int | None) -> dict:
    """test_core --consts of that build -> {key: int | float | str}."""
    key = (profile, vcfg)
    if key not in _CONSTS:
        exe = host_exe(profile, vcfg)
        out = os.path.join(BUILD, f"consts_{profile or 'field'}_{'none' if vcfg is None else vcfg}.txt")
        r = subprocess.run([exe, "--consts", out], cwd=ROOT, capture_output=True, text=True, encoding="utf-8",
                           errors="replace")
        if r.returncode != 0:
            raise SystemExit(f"test_core --consts ({profile}, FC_VCFG {vcfg}) failed: {r.stdout} {r.stderr}")
        d: dict = {}
        with open(out, "r", encoding="utf-8") as f:
            for line in f:
                k, _, v = line.rstrip("\n").partition(" ")
                try:
                    d[k] = int(v)
                except ValueError:
                    try:
                        d[k] = float(v)
                    except ValueError:
                        d[k] = v
        _CONSTS[key] = d
    return _CONSTS[key]


C_GAIN_KEYS = ("att_kp_xy", "att_kp_z", "rate_sp_max_xy_rad_s", "rate_sp_max_z_rad_s", "rate_kp_xy", "rate_ki_xy",
               "rate_kd_xy", "rate_i_max_xy", "rate_kp_z", "rate_ki_z", "rate_kd_z", "rate_i_max_z", "d_term_lpf_hz",
               "gyro_lpf_hz", "thrust_estimate_floor_fraction", "roll_priority_scale", "yaw_hold", "loop_hz")
C_PLANT_KEYS = ("l_v_m", "r_v_m", "s_v_m2", "cl_alpha", "a_e_m2", "t_max_n", "throttle_exp", "delta_max_rad",
                "pwm_center_us", "pwm_us_per_deg")
# keys the configuration check does not judge, each with its reason (listed in the report with both values):
PARAM_EXEMPT = {
    # the build variant of the run (the 6-DOF flies GPS / no-GPS / v0.7.1 builds of the same configuration; the host
    # core holds every variant and the replay hands it the run's switches - the vehicle constants are what G1 is about)
    "sup.gps_hold": "build variant (GPS position hold on / off)",
    "sup.takeoff_check": "build variant (v0.8 take-off check on / off)",
    "vc.takeoff_ticks": "build variant (the take-off check's time: 0 without it)",
    "vc.takeoff_climb_m": "build variant (the take-off check's climb: 0 without it)",
    "vc.takeoff_settle_ticks": "build variant (the take-off check's settle time: 0 without it)",
    # the 6-DOF's simulated barometer is delayed by SensorConfig.baro_latency_s (0.1 s, SPEC 9) and its filter compensates
    # that delay; the NAV compensates the real measurement schedule (hal/baro.h BARO_LATENCY_S: BMP280 forced x4
    # 16.75 ms since 2026-11-10, MS5611 15.9 ms) - a sensor model pairing, not a vehicle constant (the replays hand both
    # sides the recorded latency)
    "altv.latency_s": "sensor model (6-DOF baro delay 0.1 s vs the chip's barometer schedule: BMP280 16.75 ms / MS5611 15.9 ms)",
}
PARAM_REL_TOL = 1e-6        # float32 constants: the C rounds once (params_gen.h %.9g) or derives a few in float32 at init
                            # (alpha = 1 - expf(..), x dt); 1e-6 relative = ~8 ulp - G1 differences are 0.2-0.6 relative


def _pv_same(c, p) -> bool:
    if c is None or p is None:
        return c is None and p is None
    if isinstance(c, str) or isinstance(p, str):
        return str(c) == str(p)
    if isinstance(p, bool):
        p = int(p)
    if isinstance(c, int) and isinstance(p, int):
        return c == p
    fc, fp = f32(c), f32(p)
    if fc == fp or (fc != fc and fp != fp):
        return True
    return abs(fc - fp) <= PARAM_REL_TOL * max(abs(fc), abs(fp))


def param_view(prm: dict, power_build: int = 0) -> dict:
    """The C-key view (test_core --consts keys) of a recorded / nominal parameter set: what the firmware build of that
    configuration must hold.  The supervisor's limits are the tethered ones in a power build (onb.tether_limits, as the
    reference resolves them and core/fc_config.c fills them when PWR_SRC != 0)."""
    v: dict = {}
    for k in C_PLANT_KEYS:
        v["plant." + k] = prm["plant"][k]
    veh = prm["veh"]
    v.update({"macro.VEH_MASS_KG": veh["mass_kg"], "macro.VEH_Z_CG_M": veh["z_cg_m"], "macro.VEH_IXX_KG_M2": veh["ixx"],
              "macro.VEH_IYY_KG_M2": veh["iyy"], "macro.VEH_IZZ_KG_M2": veh["izz"]})
    for k in C_GAIN_KEYS:
        if k in prm["gains"]:
            v["gains." + k] = prm["gains"][k]
    sp = onb.SupervisorParams(**prm["sp"])
    if power_build and getattr(sp, "power_rules", False):
        lim = onb.tether_limits(sp, tethered=True)
        sp = dataclasses.replace(sp, hover_dm=lim.hover_dm, alt_min_dm=lim.alt_min_dm, alt_max_dm=lim.alt_max_dm,
                                 ceiling_m=lim.ceiling_m, v_geofence_m=lim.geofence_m, endurance_ticks=lim.endurance_ticks,
                                 hard_cutoff_ticks=lim.hard_cutoff_ticks)
    v.update({"sup." + k: x for k, x in sup_c_overrides(sp).items()})
    if getattr(sp, "power_rules", False):
        v.update({"sup." + k: x for k, x in pm_c_overrides(sp, power_build).items()})
    if prm.get("vp"):
        vp = VerticalParams(**prm["vp"])
        v.update({"vc." + k: x for k, x in vc_c_overrides(vp).items()})
        v["macro.FC_HOVER_THROTTLE"] = vp.u_hover
    if prm.get("pp"):
        v.update({"pp." + k: x for k, x in pos_c_overrides(pcm.PositionParams(**prm["pp"])).items()})
    e = prm.get("est")
    if e:
        v.update({"mah.dt": e["dt"], "mah.n_avg": e["n_avg"], "mah.kp": e["kp"], "mah.ki_dt": e["ki"] * e["dt"],
                  "mah.gate_mps2": e["accel_gate"] * e["g"], "mah.g": e["g"], "mah.alpha_a": e["alpha_a"],
                  "mah.bias_limit": e["bias_limit"]})
    al = prm.get("alt")
    if al:
        dt = e["dt"] if e else 1.0 / 250.0
        v.update({"altv.k1": al["k1"], "altv.k2": al["k2"], "altv.k3": al["k3"], "altv.latency_s": al["latency"],
                  "altv.pad_n_avg": max(1, int(round(al["pad_bias_tau_s"] / dt))), "altv.bias_limit_mps2": al["bias_limit"],
                  "altv.comp_m": al["comp_m"], "altv.inv_u_hover": al["inv_u_hover"], "altv.throttle_exp": al["throttle_exp"],
                  "altv.throttle_exp_int": 2 if al["throttle_exp"] == 2.0 else 0})
    return v


def compare_params(c: dict, nominal: dict | None, recorded: dict | None) -> dict:
    """Per key of the nominal view: the build constant (c) vs the configuration's nominal value -> 'build_mismatch'
    (a FAIL unless the key is in PARAM_EXEMPT: then 'exempt'); the run's recorded value vs nominal -> 'variant' (the
    scenario's own choice: the replay hands it to both sides).  Keys the C does not dump are 'not_in_build'."""
    out = {"build_mismatch": [], "exempt": [], "variant": [], "not_in_build": [], "compared": 0, "max_rel_equal": 0.0}
    keys = sorted(nominal) if nominal is not None else sorted(recorded or {})
    for k in keys:
        nv = None if nominal is None else nominal.get(k)
        rv = None if recorded is None else recorded.get(k)
        if k not in c:
            out["not_in_build"].append(k)
            continue
        cv = c[k]
        out["compared"] += 1
        if nominal is not None:
            if _pv_same(cv, nv):
                if not isinstance(cv, str) and nv is not None and not isinstance(nv, bool):
                    fc, fn = f32(cv), f32(nv)
                    if fc == fc and fn == fn and max(abs(fc), abs(fn)) > 0:
                        out["max_rel_equal"] = max(out["max_rel_equal"], abs(fc - fn) / max(abs(fc), abs(fn)))
            else:
                (out["exempt"] if k in PARAM_EXEMPT else out["build_mismatch"]).append(
                    {"key": k, "build": cv, "nominal": nv, **({"why": PARAM_EXEMPT[k]} if k in PARAM_EXEMPT else {})})
        if recorded is not None and nominal is not None and not _pv_same(rv, nv):
            out["variant"].append({"key": k, "run": rv, "nominal": nv, "build": cv})
    return out


def check_sixdof_params(recs: list[dict]) -> dict:
    """fw09 X1: before the 6-DOF replays - for every recorded run, the constants of the host core of its configuration
    and build profile (test_core --consts) against the nominal parameter objects of that configuration (recorded with the
    run, made by the reference's own constructors).  FAIL on any build mismatch outside PARAM_EXEMPT, on a run whose
    nominal could not be made, and on a host core that does not compile the run's configuration (meta.vehicle)."""
    st = {"runs": [], "build_mismatch_runs": 0, "pass": True, "exempt_keys": dict(PARAM_EXEMPT),
          "tolerance_rel": PARAM_REL_TOL}
    for rec in recs:
        run = rec["run"]
        c = c_consts(run["host_profile"], run["vcfg"])
        r = {"name": rec["name"], "vcfg": run["vcfg"], "profile": run["host_profile"] or "field",
             "build_vehicle": c.get("meta.vehicle"), "build_vehicle_name": c.get("meta.vehicle_name"),
             "switches": rec["params"]["switches"]}
        if rec["nominal"] is None:
            r.update(pass_=False, error=f"no nominal parameters: {rec.get('nominal_error')}")
        else:
            nom = param_view(rec["nominal"], run["power_build"])
            got = param_view(rec["params"], run["power_build"])
            cmp_ = compare_params(c, nom, got)
            r.update(cmp_)
            r["pass_"] = not cmp_["build_mismatch"] and c.get("meta.vehicle") == run["vcfg"]
            if c.get("meta.vehicle") != run["vcfg"]:
                r["error"] = (f"the host core built with -D FC_VCFG={run['vcfg']} compiles vehicle "
                              f"{c.get('meta.vehicle')} ({c.get('meta.vehicle_name')}): params_gen.h has no "
                              f"per-configuration constants for this run's vehicle")
        st["build_mismatch_runs"] += int(not r["pass_"])
        st["pass"] = st["pass"] and r["pass_"]
        st["runs"].append(r)
    st["pass"] = st["pass"] and len(recs) > 0
    return st


def print_sixdof_params(st: dict, title: str) -> None:
    print(f"== {title}: {len(st['runs'])} runs, {st['build_mismatch_runs']} with build constants that differ from the "
          f"configuration the 6-DOF flew -> {'PASS' if st['pass'] else 'FAIL'}")
    for r in st["runs"]:
        bm = r.get("build_mismatch", [])
        line = (f"   {r['name']}: FC_VCFG {r['vcfg']} ({VCFG_POWER.get(r['vcfg'])}) profile {r['profile']}, build vehicle "
                f"{r['build_vehicle']} ({r['build_vehicle_name']}); {r.get('compared', 0)} constants compared, build "
                f"mismatches {len(bm)}, exempt {len(r.get('exempt', []))}, scenario variants "
                f"{[x['key'] for x in r.get('variant', [])]}")
        print(line)
        if r.get("error"):
            print(f"      ERROR {r['error']}")
        for x in bm[:8]:
            print(f"      MISMATCH {x['key']}: build {x['build']!r} vs the 6-DOF's {x['nominal']!r}")
        if len(bm) > 8:
            print(f"      ... {len(bm) - 8} more: {[x['key'] for x in bm[8:]]}")


def check_plant_configs(ref: Ref, a) -> dict:
    """Section 2 per configuration (fw09 X1): (a) the generator's constants of the rocket and of each VTOL configuration
    (gen_firmware_params derive()['vcfg'][name]) equal sim.vehicle.load_vehicle (the simulation's own mass-property path)
    to 1e-12; (b) the host core compiled for each VTOL configuration and build profile (test_core --consts, -D FC_VCFG,
    -D MISSION_PROFILE_DEMO) holds the reference's nominal parameter objects of that configuration (the flight build's
    switches: battery with the GPS position hold, tether / hybrid ONB_NO_GPS) - plant, gains, supervisor, vertical
    controller, position hold, NAV estimator / altitude filter, the VEH_* / FC_HOVER_THROTTLE macros; (c) the rocket
    host core (no FC_VCFG) holds the rocket's plant."""
    out = {"configs": {}, "builds": {}, "pass": True}
    names = ("rocket", "battery", "tether", "hybrid")
    for nm in names:
        veh = load_vehicle() if nm == "rocket" else load_vehicle(config="vtol", power=nm)
        g = ref.d if nm == "rocket" else (ref.d.get("vcfg") or {}).get(nm)
        r = {"mass_source": getattr(veh, "mass_source", None)}
        if g is None:
            why = (ref.d.get("vcfg_errors") or {}).get(nm) if "vcfg" in ref.d else \
                "gen_firmware_params.derive() has no per-configuration constants (pre-G1 generator)"
            r.update(pass_=False, error=f"generator: no constants for the {nm} configuration: {why}")
        else:
            pairs = {"l_v_m": (g["l_v"], veh.l_v_m), "r_v_m": (g["r_v"], veh.r_v_m), "s_v_m2": (g["s_v"], veh.s_v_m2),
                     "a_e_m2": (g["a_e"], veh.a_e_m2), "t_max_n": (g["t_max"], veh.t_max_n),
                     "z_cg_m": (g["z_cg"], veh.z_cg_m), "mass_kg": (g["m_full"], veh.full.mass_kg)}
            bad = []
            for k, (x, y) in pairs.items():
                rel = abs(x - y) / max(abs(y), 1e-30)
                r[k] = {"generator": x, "sim_vehicle": y, "rel_diff": rel}
                if not rel <= 1e-12:
                    bad.append(k)
            r["pass_"] = not bad
            if bad:
                r["error"] = f"generator != sim.vehicle: {bad}"
        out["configs"][nm] = r
        out["pass"] = out["pass"] and r["pass_"]
    # (b) the host builds of the VTOL configurations vs the nominal objects (child process on --sim-root)
    path = os.path.join(BUILD, "nominal_params.json")
    root = a.sim_root or ROOT
    rc, txt = _run_child(SIXDOF_NOMINALS, root, [root, path])
    if rc != 0 or not os.path.exists(path):
        out["builds"]["error"] = f"nominal parameter child failed ({rc}): {txt[-2000:]}"
        out["pass"] = False
        return out
    with open(path, "r", encoding="utf-8") as f:
        noms = json.load(f)
    for key, nv in noms.items():
        pw, prof = key.split("/")
        vcfg = bht.VCFG_OF_POWER[pw]
        hprof = None if prof == "field" else prof
        c = c_consts(hprof, vcfg)
        b = {"vcfg": vcfg, "profile": prof, "build_vehicle": c.get("meta.vehicle"),
             "build_vehicle_name": c.get("meta.vehicle_name")}
        if nv["nominal"] is None:
            b.update(pass_=False, error=nv["error"])
        else:
            cmp_ = compare_params(c, param_view(nv["nominal"], POWER_BUILD[pw]), None)
            b.update(cmp_)
            b["l_v_m"] = {"build": c.get("plant.l_v_m"), "reference": nv["nominal"]["plant"]["l_v_m"]}
            b["u_hover"] = {"build": c.get("vc.u_hover"), "reference": (nv["nominal"]["vp"] or {}).get("u_hover")}
            b["pass_"] = not cmp_["build_mismatch"] and c.get("meta.vehicle") == vcfg
            if c.get("meta.vehicle") != vcfg:
                b["error"] = (f"the host core built with -D FC_VCFG={vcfg} compiles vehicle {c.get('meta.vehicle')} "
                              f"({c.get('meta.vehicle_name')})")
        out["builds"][key] = b
        out["pass"] = out["pass"] and b["pass_"]
    # (c) the rocket host core (unit tests, ballistic constants) holds the rocket's plant
    c = c_consts(None, None)
    rv = load_vehicle()
    rb = {k: {"build": c.get("plant." + k), "sim_vehicle": getattr(rv, k)} for k in ("l_v_m", "r_v_m", "s_v_m2", "a_e_m2",
                                                                                     "t_max_n")}
    rb["pass_"] = all(_pv_same(x["build"], x["sim_vehicle"]) for x in rb.values() if isinstance(x, dict))
    out["builds"]["rocket/default"] = rb
    out["pass"] = out["pass"] and rb["pass_"]
    return out


def print_plant_configs(pc: dict) -> None:
    print(f"== plant per configuration (generator == sim.vehicle; host core per FC_VCFG == the reference's own objects) "
          f"-> {'PASS' if pc['pass'] else 'FAIL'}")
    for nm, r in pc["configs"].items():
        lv = r.get("l_v_m", {})
        print(f"   {nm:8s} generator vs sim.vehicle: l_v {lv.get('sim_vehicle', float('nan')) * 1e3:.3f} mm, "
              f"max rel diff {max((x['rel_diff'] for x in r.values() if isinstance(x, dict) and 'rel_diff' in x), default=float('nan')):.1e}"
              f" -> {'ok' if r['pass_'] else 'FAIL'}{'' if not r.get('error') else ' (' + r['error'] + ')'}")
    for key, b in pc["builds"].items():
        if key == "error":
            print(f"   ERROR {b}")
            continue
        if key == "rocket/default":
            print(f"   rocket host core (no FC_VCFG): plant {[(k, v['build']) for k, v in b.items() if isinstance(v, dict)]} "
                  f"-> {'ok' if b['pass_'] else 'FAIL'}")
            continue
        bm = b.get("build_mismatch", [])
        print(f"   host core {key} (FC_VCFG {b['vcfg']}, build vehicle {b['build_vehicle']} {b['build_vehicle_name']}): "
              f"{b.get('compared', 0)} constants, l_v build {b.get('l_v_m', {}).get('build')} vs reference "
              f"{b.get('l_v_m', {}).get('reference')}, u_hover {b.get('u_hover', {}).get('build')} vs "
              f"{b.get('u_hover', {}).get('reference')}; mismatches {len(bm)}, exempt {[x['key'] for x in b.get('exempt', [])]}"
              f", max rel diff of the equal ones {b.get('max_rel_equal', 0.0):.1e} -> {'ok' if b['pass_'] else 'FAIL'}")
        if b.get("error"):
            print(f"      ERROR {b['error']}")
        for x in bm[:10]:
            print(f"      MISMATCH {x['key']}: build {x['build']!r} vs reference {x['nominal']!r}")
        if len(bm) > 10:
            print(f"      ... {len(bm) - 10} more: {[x['key'] for x in bm[10:]]}")


def _f32_inp(inp0: onb.OnboardInputs, nd: Nudger) -> onb.OnboardInputs:
    """float32 supervisor / stack inputs; a run without a NAV quaternion (an older recording) gets one from its tilt
    (about x), so both sides take cos(tilt) from q (the FCU path)."""
    g = nd.gyro(list(inp0.gyro_rad_s))
    q = (tuple(f32(x) for x in inp0.q_nav) if inp0.q_nav is not None else q_from_tilt(f32(inp0.tilt_deg), 0.0))
    az = None if inp0.accel_z_mps2 is None else f32(inp0.accel_z_mps2)
    return onb.OnboardInputs(
        tick=inp0.tick, frames=list(inp0.frames)[:4], h_m=f32(inp0.h_m), vz_mps=f32(inp0.vz_mps), tilt_deg=f32(inp0.tilt_deg),
        gyro_rad_s=tuple(g), gyro_saturated=bool(inp0.gyro_saturated), v_cell=f32(inp0.v_cell), imu_ok=inp0.imu_ok,
        baro_ok=inp0.baro_ok, aligned=inp0.aligned, arm_switch=inp0.arm_switch, roll_deg=f32(inp0.roll_deg),
        pitch_deg=f32(inp0.pitch_deg), yaw_deg=f32(inp0.yaw_deg), loop_max_us=f32(inp0.loop_max_us), q_nav=q,
        accel_z_mps2=az, cos_tilt=None if inp0.cos_tilt is None else f32(inp0.cos_tilt),     # v0.8 (nlink v3)
        gps_slot=inp0.gps_slot,
        # fw09 X1: the v0.9 power signals as recorded (volts; the C gets the codes PowerMonitor._code makes of them)
        v_in=getattr(inp0, "v_in", None), v_tether=getattr(inp0, "v_tether", None),
        v_buffer=getattr(inp0, "v_buffer", None), buf_i_pos=getattr(inp0, "buf_i_pos", None))


def _pw_code(v: float | None, vpc: float) -> int:
    """sim.onboard.PowerMonitor._code (None -> -1: no sample this tick)."""
    if v is None:
        return -1
    c = lp.round_half_away(float(v) / vpc) if v == v else 0
    return min(max(c, 0), 1023)


def _pw_line(inp: onb.OnboardInputs, vpc: float) -> str:
    """The 'P' line of test_core --stack-vectors (v0.9 power inputs of the next row; the stack computes the controller
    flags itself)."""
    return (f"P {_pw_code(inp.v_in, vpc)} {_pw_code(inp.v_tether, vpc)} {_pw_code(inp.v_buffer, vpc)} "
            f"{int(bool(inp.buf_i_pos))} 0 0 0\n")


def check_stack(ref: Ref, runs: list[dict]) -> tuple[dict, list[dict]]:
    """Replay recorded supervisor inputs into the Python FcuStack (float32 inputs and thresholds; v0.7.1 tick order)
    and the C stack (onb_stack_step, --stack-vectors): the supervisor outputs must be identical tick for tick (TELEM2
    bytes too, except the throttle field, which carries the previous tick's float32 vs float64 controller output: within
    one count), the controller outputs within the VC_* tolerances, its flags and counters exact, the attitude
    controller's inputs (v0.7.1: the previous tick's control_active exact and ESC throttle within VC_U_TOL).  A tick on
    which the controller's float64 decision lies within the float32 error bound (vc_margins) is ambiguous: the run is
    compared up to it.  Also returns the supervisor-only sequences (the Python stack's supervisor inputs incl. its flags)
    for check_onboard_vtol."""
    sp0, vp0 = sup_params_vtol(ref), vc_params32(ref)
    exp, sup_seqs = [], []
    blocks: dict[str, list[str]] = {}                       # fw09 X1: the host core of each run's configuration
    if True:
        for ri, run in enumerate(runs):
            sp, vp, pp = run.get("sp", sp0), run.get("vp", vp0), run.get("pp")
            nd = Nudger(sp)
            gns = run.get("chain") == "gns"                 # v0.8.1: the four-chip FCU (ONB_CHAIN_GNS)
            fcu = onb.FcuStack(sp, vp, seed=run["seed"], reset_in_flight=run["reset"], pp=pp,
                               chain="gns" if gns else "fcu")
            tsup = TracingSupervisor(sp, seed=run["seed"], reset_in_flight=run["reset"])
            fcu.sup = tsup
            # fw09 X1: a 6-DOF run replays through the host core compiled for ITS configuration and build profile
            # (-D FC_VCFG, -D MISSION_PROFILE_*); a tether / hybrid run carries its power signals ('P' lines, the
            # build's pw.* table and the tethered limits the reference resolves in SAFE - check_onboard_power's mapping)
            exe = host_exe(run.get("host_profile"), run["vcfg"]) if "vcfg" in run else bht.EXE
            pwb = int(run.get("power_build", 0))
            sp_c = sp
            pw_ov = ""
            if pwb and getattr(sp, "power_rules", False):
                lim = onb.tether_limits(sp, tethered=True)
                sp_c = dataclasses.replace(sp, hover_dm=lim.hover_dm, alt_min_dm=lim.alt_min_dm, alt_max_dm=lim.alt_max_dm,
                                           ceiling_m=f32(lim.ceiling_m), v_geofence_m=f32(lim.geofence_m),
                                           endurance_ticks=lim.endurance_ticks, hard_cutoff_ticks=lim.hard_cutoff_ticks)
                pw_ov = " " + _ov_text(pm_c_overrides(sp, pwb))
            elif getattr(sp, "power_rules", False):
                pw_ov = " " + _ov_text({"pw.build": 0})
            head = (f"#K {ri} {run['seed']} {int(run['reset'])} mode=1 {_ov_text(sup_c_overrides(sp_c))}{pw_ov} "
                    f"{_ov_text(vc_c_overrides(vp), 'vc.')}"
                    f"{(' ' + _ov_text(pos_c_overrides(pp), 'pp.')) if pp is not None else ''}"
                    f"{' chain=1' if gns else ''}\n")
            run["_vp"] = vp
            run["_exe"] = exe
            rows, py, sinps = [], [], []
            for inp0 in run["inputs"]:
                inp = _f32_inp(inp0, nd)
                n0, n1 = len(tsup.exec_log), len(tsup.trace)
                bef = vc_before(fcu.vc)
                prev, prev_thr = fcu.vc_out, fcu.throttle
                k_before = fcu.vc.k_t
                fo = fcu.step(inp)
                if pp is not None:                          # v0.8: the supervisor's own inputs (GPS ones included,
                    li = tsup.last_inp                      # dist as the float32 the C carries)
                    tg = li.tlm_gps
                    sinps.append(dataclasses.replace(li, throttle_prev=f32(prev_thr),
                                                     tlm_gps=None if tg is None else (*tg[:4], f32(tg[4]))))
                else:
                    sinps.append(dataclasses.replace(inp, vc_touchdown=bool(prev is not None and prev.touchdown),
                                                     vc_at_target=bool(prev is not None and prev.at_target),
                                                     vc_ground_signature=bool(prev is not None and prev.ground_signature),
                                                     vc_low_in_land_ticks=int(prev.low_in_land_ticks) if prev is not None
                                                     else 0, throttle_prev=f32(prev_thr)))
                vin = VerticalInputs(phase=fo.sup.phase, h_tgt_m=fo.sup.h_tgt_m, baro_lost=fo.sup.baro_lost, h_m=inp.h_m,
                                     vz_mps=inp.vz_mps, cos_tilt=inp.cos_tilt if inp.cos_tilt is not None else
                                     vcm.cos_tilt_from_q(inp.q_nav), f_z_mps2=inp.accel_z_mps2,
                                     u_cap=fo.sup.u_cap)    # fw09 final verifier: the W_SAG ceiling FcuStack.step passes
                amb, _ = vc_margins(vp, bef, vin, fo.vc)
                r = _py_row_vtol(tsup, fo.sup, n0, n1, inp.tick)
                r.update(vc=fo.vc, vstate=vc_state(fcu.vc), esc=fo.throttle, amb=amb, att_active=int(fo.att_active),
                         att_u=fo.att_u, fz=inp.accel_z_mps2,
                         g=None if pp is None else (stack_n_expected(fcu, fo) if gns else stack_g_expected(fcu, fo)),
                         gns=gns)
                rows.append((_pw_line(inp, sp.pw_v_per_count) if pwb else "") +
                            _vtol_inp_row(inp) + f",{r32(k_before)}" +      # v0.7.1b k_sync (see _vc_row)
                            (_gps_cols(inp) if pp is not None else ""))
                py.append(r)
            blocks.setdefault(exe, []).append(head + "\n".join(rows) + "\n")
            exp.append(py)
            # the supervisor alone on the Python stack's inputs (its controller flags, float32 ESC throttle): exact
            sup2 = TracingSupervisor(sp, seed=run["seed"], reset_in_flight=run["reset"])
            srows, spy = [], []
            for sinp in sinps:
                n0, n1 = len(sup2.exec_log), len(sup2.trace)
                so = sup2.update(sinp)
                srows.append(_v08_inp_row(sinp) if pp is not None else _vtol_inp_row(sinp))
                spy.append(_py_row_vtol(sup2, so, n0, n1, sinp.tick))
            if not pwb:                                   # a power run's supervisor is compared inside the stack replay
                sup_seqs.append(dict(kind="vtol_replay:" + run["name"], seed=run["seed"], reset=run["reset"],
                                     overrides=dict(sup_c_overrides(sp), mode=1), rows=srows, py=spy, nudged=nd.count))
    by_run: dict[int, list[str]] = {}
    tmp_files = []
    for gi, (exe, blk) in enumerate(blocks.items()):
        fin, fout = os.path.join(BUILD, f"stack_in_{gi}.txt"), os.path.join(BUILD, f"stack_out_{gi}.csv")
        tmp_files += [fin, fout]
        with open(fin, "w", encoding="utf-8", newline="\n") as f:
            f.write("".join(blk))
        r = subprocess.run([exe, "--stack-vectors", fin, fout], cwd=ROOT, capture_output=True, text=True, encoding="utf-8",
                           errors="replace")
        if r.returncode != 0:
            raise SystemExit(f"{os.path.basename(exe)} --stack-vectors failed ({r.returncode}): {r.stdout} {r.stderr}")
        with open(fout, "r", encoding="utf-8") as f:
            f.readline()
            for line in f.read().splitlines():
                by_run.setdefault(int(line.split(",", 1)[0]), []).append(line)
    c_lines = [x for ri in range(len(runs)) for x in by_run.get(ri, [])]
    st = {"runs": [], "ticks": 0, "ticks_compared": 0, "mismatch_ticks": 0, "ambiguous_runs": 0, "ambiguous_ticks": 0,
          "ambiguous_cut": [], "knife_edges": {}, "knife_edges_resynced": 0, "first_mismatches": [],
          "telem_throttle_one_count_off": 0, "max_abs": {k: 0.0 for k in ("u", "h_ref", "integ", "esc", "att_u", "v_lpf",
                                                                           "d_lpf", "k_t")},
          "touchdowns_by": {}, "max_abs_g": {}, "pos_knife_edges": []}
    i = 0
    for ri, (run, py) in enumerate(zip(runs, exp)):
        vp = run["_vp"]
        tol = vc_tolerances(vp)
        stop_at, skip_until, check_resync, n_knife = None, -1, False, 0
        prev_g = None
        mm = 0
        prop = vc_prop_init()
        for k, o in enumerate(py):
            parts = c_lines[i].split(",")
            i += 1
            if int(parts[0]) != ri or int(parts[1]) != o["t"]:
                raise SystemExit(f"stack-vectors row order mismatch at {i}")
            st["ticks"] += 1
            cv = vc_parse_c(parts, 21)
            vc_prop_step(prop, vp, cv, o["vc"], o["fz"])
            if stop_at is not None or k <= skip_until:
                continue
            c, interlock, _, _ = _c_onb_row(parts, True)
            bad = []
            if o.get("gns") and o.get("g") is not None:   # v0.8.1: this tick's TELEM distance is the GNS set's
                prev_g = {"dist": o["g"]["tlm"][4]}
            for key in c:
                if c[key] == o[key]:
                    continue
                if key == "telem" and c[key] != "-" and o[key] != "-" and len(c[key]) == len(o[key]):
                    bc, bp = bytes.fromhex(c[key]), bytes.fromhex(o[key])
                    tc, tp = (int.from_bytes(b[22:24], "little") for b in (bc, bp))      # throttle_milli (u16 at 22)
                    if len(bc) == lp.TELEM3_PAYLOAD_LEN and prev_g is not None:          # v0.8: dist_dm (u16 at 35)
                        dc, dp = (int.from_bytes(b[35:37], "little") for b in (bc, bp))
                        dref = prev_g["dist"]
                        if (dc != dp and abs(dc - dp) == 1 and dref == dref and
                                abs(abs(dref * 10.0 - math.floor(dref * 10.0)) - 0.5) <= POS_X_TOL * 10.0):
                            st["telem_dist_one_count_off"] = st.get("telem_dist_one_count_off", 0) + 1
                            bc, bp = bc[:35], bp[:35]
                    if bc[:22] == bp[:22] and bc[24:] == bp[24:] and abs(tc - tp) <= 1:
                        if tc != tp:
                            st["telem_throttle_one_count_off"] += 1
                        continue
                bad.append(key)
            vo = o["vc"]
            vbad, ds = vc_compare(cv, vo, o["vstate"], tol, prop)
            if not (vo.mode == vcm.MODE_IDLE and cv["mode"] == vcm.MODE_IDLE):
                bad += ["vc." + b for b in vbad]
            esc, att_active, att_u = float(parts[47]), int(parts[48]), float(parts[49])     # v0.7.1b: +cnt3, k_t
            ds = {key: ds[key] for key in ("u", "h_ref", "integ", "v_lpf", "d_lpf", "k_t") if key in ds}
            ds["esc"] = abs(esc - o["esc"])
            ds["att_u"] = abs(att_u - o["att_u"])
            if not ds["esc"] <= VC_U_TOL:
                bad.append("esc")
            if not ds["att_u"] <= VC_U_TOL:
                bad.append("att_u")
            if o.get("g") is not None and o.get("gns"):   # v0.8.1: the GNS chain's tail of the row
                gbad = stack_n_compare(parts, o["g"], st["max_abs_g"], st)
                if gbad and o["amb"] and set(gbad) <= {"pflags", "kt_q"}:
                    bad += ["vc." + b for b in gbad]            # the controller's flags at a knife edge: as vc.*
                else:
                    bad += ["g." + b for b in gbad]
            elif o.get("g") is not None:                  # v0.8: the position hold's tail of the row
                gbad = stack_g_compare(parts, o["g"], st["max_abs_g"])
                prev_g = o["g"]
                if gbad and set(gbad) <= {"lim_run", "integ", "tilt_l", "tilt_next", "att_tilt"} and "lim_run" in gbad:
                    # the a-limit tie of check_pos (|a| == a_lim by design while the vertical loop is saturated high):
                    # this run is compared up to here
                    st["pos_knife_edges"].append({"run": run["name"], "tick": o["t"], "fields": gbad})
                    stop_at = k
                    continue
                bad += ["g." + b for b in gbad]
            if att_active != o["att_active"]:
                bad.append("att_active")
            if interlock and o["reason"] != interlock:
                bad.append("interlock")
            st["ambiguous_ticks"] += int(o["amb"])
            if check_resync:                              # the first tick after a resync window: everything agrees again?
                check_resync = False
                if bad:                                   # a lasting branch: the run is compared up to here
                    stop_at = k
                    st["ambiguous_runs"] += 1
                    st["ambiguous_cut"].append({"run": run["name"], "tick": o["t"], "fields": bad})
                    continue
                st["knife_edges_resynced"] += 1
            if o["amb"] and bad and all(b.startswith("vc.") or b == "esc" for b in bad):
                # the controller took the other side of a knife-edge decision in float32 (this tick's supervisor, which
                # ran before it, agrees): its flags act next tick - skip VC_RESYNC_TICKS, then everything must agree
                key = ",".join(bad)
                st["knife_edges"][key] = st["knife_edges"].get(key, 0) + 1
                skip_until, check_resync = k + VC_RESYNC_TICKS, True
                n_knife += 1
                st["ticks_compared"] += 1
                continue
            for key, d in ds.items():
                st["max_abs"][key] = max(st["max_abs"][key], d)
            st["ticks_compared"] += 1
            if vo.touchdown and vo.td_count == vp.landed_confirm_ticks:
                nm = vcm.TD_NAMES[vo.td_by]
                st["touchdowns_by"][nm] = st["touchdowns_by"].get(nm, 0) + 1
            if bad:
                st["mismatch_ticks"] += 1
                mm += 1
                if len(st["first_mismatches"]) < 10:
                    st["first_mismatches"].append({"run": run["name"], "tick": o["t"], "fields": bad})
        phases = sorted({lp.PHASE_NAMES[o["phase"]] for o in py})
        st["runs"].append({"name": run["name"], "profile": run.get("profile", "field (the generator's)"), "ticks": len(py),
                           "host_core": os.path.basename(run["_exe"]), "power_build": int(run.get("power_build", 0)),
                           "mismatch_ticks": mm, "knife_edges": n_knife,
                           "compared_until": len(py) if stop_at is None else stop_at, "phases": phases,
                           "reasons": sorted({lp.REASON_NAMES.get(o["reason"], o["reason"]) for o in py if o["reason"]}),
                           "touchdown_by": sorted({vcm.TD_NAMES[o["vc"].td_by] for o in py if o["vc"].touchdown}),
                           **({"wall_s": round(run["wall"], 1)} if "wall" in run else {})})
    if i != len(c_lines):
        raise SystemExit(f"C produced {len(c_lines)} stack rows, expected {i}")
    st["pass"] = (st["mismatch_ticks"] == 0 and st["ticks_compared"] > 0 and
                  len(st["pos_knife_edges"]) <= max(1, len(runs) // 10))
    if st["pass"]:
        for path in tmp_files:
            os.remove(path)
    return st, sup_seqs


# ---- protocol v2 (TELEM2) and ilink SNAP2 ----------------------------------------------------------------------- #
_LP_MASK = {lp.TYPE_CMD: 1, lp.TYPE_TELEM: 2, lp.TYPE_TELEM2: 4}      # link_proto.h LP_ACCEPT_*


def _rand_telem2(rng: random.Random) -> lp.Telemetry:
    return lp.Telemetry(rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(65536),
                        rng.choice([0, (1 << 32) - 1, rng.randrange(1 << 32)]), *[rng.randrange(-32768, 32768) for _ in range(5)],
                        *[rng.randrange(65536) for _ in range(4)], rng.randrange(256), rng.randrange(-32768, 32768))


def check_protocol_v2(ref: Ref, seed: int, n_streams: int, n_encode: int) -> dict:
    """v0.7 TELEM2 (TYPE 0x21, 31 bytes, LEN_MAX 34) and the VTOL command codes: random streams mixing CMD (codes
    0..11 and junk), TELEM and TELEM2 frames, LEN 32..34 frames of other TYPEs, corrupted TELEM2 (bit flips, cuts, LEN
    changed, bytes inserted) and garbage through both decoders with every accept mask that includes TELEM2 (frames and
    every counter equal); random TELEM2 encodings (bytes equal)."""
    rng = random.Random(seed)
    vid = int(ref.p["flight_computer"]["link"]["vehicle_id"])
    lines, expect, n_bytes = [], [], 0
    for _ in range(n_streams):
        dec_vid = rng.choice([vid, vid, None, (vid + 1) & 0xFF])
        acc = rng.choice([(lp.TYPE_TELEM, lp.TYPE_TELEM2), (lp.TYPE_TELEM2,), (lp.TYPE_CMD, lp.TYPE_TELEM, lp.TYPE_TELEM2),
                          (lp.TYPE_CMD,), (lp.TYPE_TELEM,)])
        parts = []
        for _ in range(rng.randrange(1, 12)):
            k = rng.random()
            veh = vid if rng.random() < 0.8 else rng.randrange(256)
            if k < 0.2:
                parts.append(lp.encode_command(veh, rng.randrange(256), lp.Command(rng.choice([8, 9, 10, 11, rng.randrange(256)]),
                                                                                    rng.randrange(65536), rng.randrange(65536))))
            elif k < 0.45:
                parts.append(lp.encode_telemetry(veh, rng.randrange(256), _rand_telem2(rng)))
            elif k < 0.55:
                t = _rand_telem2(rng)
                parts.append(lp.encode_telemetry(veh, rng.randrange(256), dataclasses.replace(t, phase=None, h_tgt_dm=None)))
            elif k < 0.63:                                        # other TYPE / payload length up to the new LEN_MAX
                pl = bytes(rng.randrange(256) for _ in range(rng.choice([28, 29, 30, 31, rng.randrange(0, 32)])))
                parts.append(lp.encode_frame(veh, rng.choice([lp.TYPE_CMD, lp.TYPE_TELEM, lp.TYPE_TELEM2, 0x22, 0xFF]),
                                             rng.randrange(256), pl))
            elif k < 0.75:
                parts.append(bytes(rng.choice([0xA5, 0x5A, rng.randrange(256), lp.LEN_MAX, lp.LEN_MAX + 1, 33, 31, 0x21])
                                   for _ in range(rng.randrange(1, 45))))
            else:
                base = bytearray(lp.encode_telemetry(veh, rng.randrange(256), _rand_telem2(rng)))
                m = rng.random()
                if m < 0.35:
                    base[rng.randrange(len(base))] ^= 1 << rng.randrange(8)
                elif m < 0.5:
                    base = base[:rng.randrange(1, len(base))]
                elif m < 0.7:
                    base[2] = rng.choice([31, 32, 33, 35, 0xFF, rng.randrange(lp.LEN_MIN, lp.LEN_MAX + 1)])
                elif m < 0.85:
                    j = rng.randrange(len(base) + 1)
                    base[j:j] = bytes([rng.choice([0xA5, 0x5A, rng.randrange(256)])])
                else:
                    base = bytearray([0xA5]) + base
                parts.append(bytes(base))
        data = b"".join(parts)
        n_bytes += len(data)
        mask = 0
        for t in acc:
            mask |= _LP_MASK[t]
        dec = lp.FrameDecoder(dec_vid, acc)
        frames = dec.feed(data)
        lines.append(f"D {-1 if dec_vid is None else dec_vid} {mask} {data.hex()}")
        expect.append("D" + "".join(f" {fr.veh}:{fr.type}:{fr.seq}:{fr.payload.hex()}" for fr in frames) + " S "
                      + " ".join(str(dec.stats[k]) for k in lp.STAT_KEYS) + " 0")
    n_tlm2 = 0
    for _ in range(n_encode):
        t = _rand_telem2(rng)
        veh, seq = rng.randrange(256), rng.randrange(256)
        lines.append(f"U {veh} {seq} " + " ".join(str(getattr(t, k)) for k in bht.TELEM2_ORDER))
        expect.append("U " + lp.encode_telemetry(veh, seq, t).hex())
        n_tlm2 += 1
    fin, fout = os.path.join(BUILD, "proto2_in.txt"), os.path.join(BUILD, "proto2_out.txt")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    run_exe(["--proto-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        got = f.read().splitlines()
    if len(got) != len(expect):
        raise SystemExit(f"proto-vectors v2: {len(got)} lines, expected {len(expect)}")
    st = {"streams": n_streams, "stream_bytes": n_bytes, "telem2_encodings": n_tlm2,
          "mismatches": 0, "first_mismatches": [], "py_frames_by_type": {}}
    for k, (a, b) in enumerate(zip(got, expect)):
        if a != b:
            st["mismatches"] += 1
            if len(st["first_mismatches"]) < 8:
                st["first_mismatches"].append({"line": k, "input": lines[k][:200], "c": a[:300], "py": b[:300]})
    for x in expect[:n_streams]:
        for tok in x.split(" S ")[0].split()[1:]:
            ty = int(tok.split(":")[1])
            st["py_frames_by_type"][hex(ty)] = st["py_frames_by_type"].get(hex(ty), 0) + 1
    st["pass"] = st["mismatches"] == 0 and st["py_frames_by_type"].get(hex(lp.TYPE_TELEM2), 0) > 0
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


def check_ilink_v2(vid: int, loop_hz: int, seed: int, n_streams: int, n_snaps: int) -> dict:
    """SNAP2 (TYPE 4, 55 bytes) on the ilink: random streams of SNAP2 / SNAP / CMD / STAT frames, garbage and corrupted
    SNAP2 through both decoders with the VTOL COM mask (SNAP2 only), ALL, FCU and the ballistic COM mask (every frame,
    counter, candidate, need / cmd_need equal); SNAP2 re-encodings; the FCU's SNAP2 (il_snap2_from_tlm_src: phase,
    spare 0, h_tgt_m) byte-exact; the VTOL COM path SNAP2 -> TELEM2 vs telem_frame_from_snap (angles within one
    0.01 deg quantum, every other byte equal)."""
    rng = random.Random(seed)
    lines, expect, kinds = [], [], []
    types = (il.TYPE_CMD, il.TYPE_SNAP, il.TYPE_STAT, il.TYPE_SNAP2, il.TYPE_SNAP2)

    def payload(typ: int) -> bytes:
        if typ != il.TYPE_SNAP2:
            return _il_rand_payload(rng, typ)
        return _il_rand_payload(rng, il.TYPE_SNAP) + bytes([rng.randrange(256), rng.choice([0, 0, rng.randrange(256)])]) + \
            struct.pack("<I", _il_rand_float_bits(rng)) + bytes([rng.randrange(256), rng.randrange(256)])   # v0.9 pw tail

    n_frames = 0
    for _ in range(n_streams):
        max_out = rng.choice([0, 1, 1, 2, 8])
        accept = rng.choice([il.ACCEPT_COM_VTOL, il.ACCEPT_COM_VTOL, il.ACCEPT_ALL, il.ACCEPT_FCU, il.ACCEPT_COM])
        parts = []
        for _ in range(rng.randrange(1, 12)):
            k = rng.random()
            typ = rng.choice(types)
            if k < 0.45:
                parts.append(il.encode(typ, payload(typ)))
            elif k < 0.55:
                parts.append(bytes(rng.choice([il.SYNC0, il.SYNC1, il.SNAP2_LEN, il.SNAP_LEN, il.TYPE_SNAP2, il.TYPE_SNAP,
                                               rng.randrange(256)]) for _ in range(rng.randrange(1, 70))))
            elif k < 0.62:
                inner = b"".join(il.encode(t, payload(t)) for t in rng.choices([il.TYPE_CMD, il.TYPE_SNAP2], k=rng.randrange(1, 3)))
                parts.append(bytes([il.SYNC0, il.SYNC1, il.SNAP2_LEN, il.TYPE_SNAP2]) + inner[:rng.randrange(1, len(inner) + 1)])
            elif k < 0.68:
                ln = rng.choice([il.SNAP2_LEN, il.SNAP_LEN, rng.randrange(0, 62)])
                body = bytes([ln, rng.choice([il.TYPE_SNAP2, il.TYPE_SNAP, 5])]) + bytes(rng.randrange(256) for _ in range(ln))
                crc = il.crc16(body)
                parts.append(bytes([il.SYNC0, il.SYNC1]) + body + bytes([crc & 0xFF, crc >> 8]))
            else:
                base = bytearray(il.encode(il.TYPE_SNAP2, payload(il.TYPE_SNAP2)))
                m = rng.random()
                if m < 0.4:
                    base[rng.randrange(len(base))] ^= 1 << rng.randrange(8)
                elif m < 0.55:
                    base = base[:rng.randrange(1, len(base))]
                elif m < 0.7:
                    base[rng.choice([2, 3])] = rng.choice([il.SNAP_LEN, il.TYPE_SNAP, il.CMD_LEN, 0xC5, rng.randrange(256)])
                elif m < 0.85:
                    j = rng.randrange(len(base) + 1)
                    base[j:j] = bytes([rng.choice([il.SYNC0, il.SYNC1, rng.randrange(256)])])
                else:
                    del base[rng.randrange(len(base))]
                parts.append(bytes(base))
        data = b"".join(parts)
        dec, frames, need, cneed = il.decode_trace(data, max_out, accept)
        n_frames += len(frames)
        lines.append(f"D {max_out} {accept} {data.hex()}")
        expect.append("D" + "".join(f" {fr.type}:{fr.len}:{fr.payload.hex()}" for fr in frames) + " S "
                      + " ".join(str(dec.stats[k]) for k in il.DEC_STAT_KEYS) + f" P {dec.pending}"
                      + f" N {need.hex()} C {cneed.hex()}")
        kinds.append("decode")
    for _ in range(1000):
        pl = payload(il.TYPE_SNAP2)
        lines.append(f"E {il.TYPE_SNAP2} {pl.hex()}")
        expect.append("E " + il.encode(il.TYPE_SNAP2, pl).hex())
        kinds.append("encode")
    for _ in range(1000):                                         # FCU: tlm source (v2) + q -> SNAP2 payload
        def fv() -> float:
            return rng.choice([math.nan, math.inf, -math.inf, -0.0]) if rng.random() < 0.05 else \
                il.f32(rng.uniform(-1, 1) * 10 ** rng.uniform(-3, 5))
        tgt = rng.choice([0.0, 5.0, 12.0, il.f32(rng.uniform(-50, 50)), math.nan, math.inf])
        src = il.TlmSource(state=rng.randrange(256), flags=rng.randrange(256), reason=rng.randrange(256),
                           ack_seq=rng.randrange(256), nonce=rng.randrange(65536), throttle_milli=rng.randrange(65536),
                           tick=rng.randrange(1 << 32), h_m=fv(), vz_mps=fv(), roll_deg=fv(), pitch_deg=fv(), yaw_deg=fv(),
                           tilt_deg=fv(), v_cell=fv(), loop_max_us=fv(), phase=rng.randrange(256), h_tgt_m=tgt)
        q = _il_rand_attitude(rng)
        seq = rng.randrange(256)
        nums = [seq, src.state, src.flags, src.reason, src.ack_seq, src.nonce, src.throttle_milli, src.tick,
                *(il.f32_bits(getattr(src, n)) for n in ("h_m", "vz_mps", "roll_deg", "pitch_deg", "yaw_deg", "tilt_deg",
                                                         "v_cell", "loop_max_us")), *(il.f32_bits(c) for c in q),
                src.phase, il.f32_bits(tgt)]
        lines.append("F " + " ".join(str(x) for x in nums))
        expect.append("F " + il.snap_from_tlm_src(seq, src, q).pack().hex())
        kinds.append("snap")
    for _ in range(n_snaps):                                      # VTOL COM: SNAP2 -> TELEM2 frame
        bits = [il.f32_bits(rng.uniform(-5, 40)), il.f32_bits(rng.uniform(-5, 5)), il.f32_bits(rng.uniform(0, 60)),
                il.f32_bits(rng.uniform(2.5, 4.3)), il.f32_bits(rng.uniform(0, 70000))]
        for j in range(5):
            if rng.random() < 0.03:
                bits[j] = rng.choice(_IL_SPECIAL_BITS)
        qb = [il.f32_bits(c) for c in _il_rand_attitude(rng)]
        tb = rng.choice([il.f32_bits(rng.choice([0.0, 2.0, 5.0, 12.0, 3276.75, -3276.85])),
                         il.f32_bits(rng.uniform(-40, 40)), rng.choice(_IL_SPECIAL_BITS)])
        # v0.9 RP1: + the power tail (pw_vaux, pw_flags; the build bits 0..3 - a power build's snapshot becomes TELEM4)
        pl2 = struct.pack("<BBBBBHHI9IBBIBB", rng.randrange(256), rng.randrange(6), rng.randrange(256), rng.randrange(256),
                          rng.randrange(256), rng.randrange(65536), rng.randrange(1001), rng.randrange(1 << 32), *bits, *qb,
                          rng.randrange(6), rng.choice([0, 0, 0xFF]), tb, rng.randrange(256),
                          rng.choice([0, 0, rng.randrange(64), rng.randrange(256)]))
        if tb == 0x7F800001:                                      # signalling NaN: Python quietens it (vectors note)
            tb = 0x7FC00001
            pl2 = pl2[:51] + struct.pack("<I", tb) + pl2[55:]
        lines.append(f"P {vid} {loop_hz} {pl2.hex()}")
        expect.append("P " + il.telem_frame_from_snap(il.Snap.unpack(pl2), vid, loop_hz).hex())
        kinds.append("telem")
    fin, fout = os.path.join(BUILD, "ilink2_in.txt"), os.path.join(BUILD, "ilink2_out.txt")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    run_exe(["--ilink-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        got = f.read().splitlines()
    if len(got) != len(expect):
        raise SystemExit(f"ilink-vectors v2: {len(got)} lines, expected {len(expect)}")
    st = {"streams": n_streams, "py_frames": n_frames, "encodings": 1000, "snapshots": 1000, "telem2_frames": n_snaps,
          "mismatches": {"decode": 0, "encode": 0, "snap": 0, "telem": 0}, "telem_angle_fields_one_quantum_off": 0,
          "first_mismatches": []}
    for k, (a, b) in enumerate(zip(got, expect)):
        kind = kinds[k]
        if a == b:
            continue
        if kind == "telem":
            fc, fp = bytes.fromhex(a[2:]), bytes.fromhex(b[2:])
            ok = (len(fc) == len(fp) and len(fc) in (lp.TELEM2_FRAME_BYTES, lp.TELEM4_FRAME_BYTES,   # v0.9 RP1: TELEM4
                                                     lp.TELEM6_FRAME_BYTES)   # 2026-11-10: a non-zero spare -> TELEM6
                  and lp.crc16_ccitt(fc[2:-2]) == (fc[-2] | fc[-1] << 8))
            other = [j for j in range(min(len(fc), len(fp)) - 2) if fc[j] != fp[j] and not 20 <= j < 26]
            diffs = [abs(int.from_bytes(fc[o:o + 2], "little", signed=True) - int.from_bytes(fp[o:o + 2], "little", signed=True))
                     for o in il.TELEM_ANGLE_OFFSETS] if ok else [99]
            if ok and not other and max(diffs) <= 1:
                st["telem_angle_fields_one_quantum_off"] += sum(1 for d in diffs if d == 1)
                continue
        st["mismatches"][kind] += 1
        if len(st["first_mismatches"]) < 8:
            st["first_mismatches"].append({"line": k, "kind": kind, "input": lines[k][:200], "c": a[:300], "py": b[:300]})
    st["pass"] = sum(st["mismatches"].values()) == 0 and n_frames > 0
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


# ---- v0.7.1 VTOL NAV estimator and altitude filter (section 7b) ------------------------------------------------- #
def load_nav_replays(paths: list[str]) -> list[dict]:
    out = []
    for path in paths:
        with np.load(path) as z:
            out.append(dict(name="6dof_" + json.loads(str(z["meta"]))["name"], meta=json.loads(str(z["meta"])),
                            est=np.array(z["est"]), alt=np.array(z["alt"])))
    return out


def mahony_consts(ref: Ref) -> dict:
    """The firmware's VTOL estimator constants (gen_firmware_params, default profile) as the replay uses them."""
    e = ref.d["vt"]["profiles"][ref.d["vt"]["default"]]["est"]
    return dict(dt=ref.dt, n_avg=int(ref.gains["est_pad_n_avg"]), kp=e["kp"], ki=e["ki"], accel_gate=e["accel_gate"],
                g=e["g"], alpha_a=e["alpha_a"], bias_limit=e["bias_limit_rad_s"])


def mahony_synthetic(ref: Ref, rng: random.Random, n_seq: int) -> list[dict]:
    """Synthetic NAV streams for the VTOL estimator: pad alignment (gyro bias, noise, a few deg of pad tilt) for 1.2-2
    averaging windows, then flight - attitude random walks, a hover whose specific force is drag-balanced (|f| ~ g:
    the gate opens), vertical accelerations of the take-off / climbs (|f| up to 1.3 g: gate closed), vibration noise,
    a slowly wandering gyro bias; sometimes a landing back into pad mode (re-entry) and a second flight."""
    c = mahony_consts(ref)
    seqs = []
    g0 = onb.G0
    for s in range(n_seq):
        n_avg = c["n_avg"]
        bias = [rng.gauss(0.0, math.radians(0.5)) for _ in range(3)]
        q = quat.from_euler(math.radians(rng.uniform(-3, 3)), math.radians(rng.uniform(-3, 3)), rng.uniform(-3, 3))
        w = np.zeros(3)
        vib = rng.choice([0.05, 0.3, 1.5])
        rows = []
        segs = [("pad", int(n_avg * rng.uniform(1.2, 2.0))), ("flight", rng.randrange(2000, 9000))]
        if rng.random() < 0.4:
            segs += [("pad", rng.randrange(100, 900)), ("flight", rng.randrange(1000, 4000))]
        for kind, n in segs:
            for k in range(n):
                if kind == "pad":
                    w = np.zeros(3)
                    acc_w = np.array([0.0, 0.0, g0])
                else:
                    w = np.clip(0.995 * w + np.array([rng.gauss(0, 0.02), rng.gauss(0, 0.02), rng.gauss(0, 0.01)]), -1.5, 1.5)
                    q = quat.integrate(q, w, ref.dt)
                    phase = (k // 600) % 4
                    az = {0: 0.0, 1: rng.gauss(0.25 * g0, 0.02), 2: 0.0, 3: rng.gauss(-0.1 * g0, 0.02)}[phase]
                    acc_w = np.array([rng.gauss(0, 0.3), rng.gauss(0, 0.3), g0 + az])
                bias = [b + rng.gauss(0.0, 2e-6) for b in bias]
                f_b = quat.rotate_inverse(q, acc_w) + np.array([rng.gauss(0, vib) for _ in range(3)])
                gyro = w + np.array(bias) + np.array([rng.gauss(0, 0.003) for _ in range(3)])
                rows.append([f32(x) for x in gyro] + [f32(x) for x in f_b] + [1.0 if kind == "pad" else 0.0])
        seqs.append(dict(name=f"synthetic_{s}", consts=c, est=np.array(rows)))
    return seqs


def check_mahony(seqs: list[dict]) -> dict:
    """``sim.estimator.GatedMahonyEstimator`` (float64) vs ``att_mahony_*`` (float32, ``--mahony-vectors``) on the same
    float32 sensor streams and float32 constants: q, the bias, the bias-corrected rate, the gate, the correction error e,
    the low-passed specific force and the pad bias.

    Bounds, justification: on the pad both run the PadAlignGyroEstimator branch (the ballistic section's EST_* bounds
    hold; the C pad branch is the ballistic one bit for bit).  In flight the integration (quat_integrate, renormalised
    every tick) adds <= ~2 ulp of 1 (2.4e-7) per tick to q.  While the gate is closed (vertical accelerations, vibration
    beyond accel_gate) the estimator is gyro-only and that error random-walks as in the ballistic section (sqrt(N) x
    2.4e-7: 3e-5 after 15000 ticks, 3.6e-3 if every rounding added up); while it is open the tilt part is contracted with
    the time constant 1 / kp (6.7 s at kp 0.15); the yaw part always integrates.  MAHONY_Q_TOL = the ballistic section's
    EST_Q_BOUND 1e-4 (1/8 of one servo PWM count, 7.9e-4 rad); e is the tilt part of that difference (MAHONY_E_TOL 1e-4
    rad).  The bias: the pad average as in the ballistic section (EST_BIAS_BOUND 1e-5 rad/s), then
    in flight it integrates ki x the e difference (<= MAHONY_E_TOL): <= 6e-6 rad/s over 60 s - MAHONY_BIAS_TOL 1e-5
    rad/s; the rate g - b adds 1/2 ulp of the gyro range (1.9e-6): MAHONY_RATE_TOL 1e-5 rad/s.  The low-pass
    of the specific force is contractive (alpha 0.049): ulp(20 m/s^2) / alpha = 4e-5 -> MAHONY_ALPF_TOL 1e-4.
    The gate is a threshold on | |a_lpf| - g |: when the float64 margin is below MAHONY_AMB_GATE (> the a_lpf error) the
    float32 estimator may decide the other way; such a tick ends that sequence's comparison (counted, like the
    controller's knife-edge cuts) - a gate step changes q by kp e dt for good."""
    fin, fout = os.path.join(BUILD, "mahony_in.txt"), os.path.join(BUILD, "mahony_out.csv")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        for si, sq in enumerate(seqs):
            c = sq["consts"]
            f.write(f"#M {c['n_avg']} {r32(c['kp'])} {r32(c['ki'] * c['dt'])} {r32(c['accel_gate'] * c['g'])} {r32(c['g'])} "
                    f"{r32(c['alpha_a'])} {r32(c['bias_limit'])}\n")
            f.write("\n".join(f"{si}," + ",".join(r32(x) for x in r[:6]) + f",{int(r[6])}" for r in sq["est"]) + "\n")
    run_exe(["--mahony-vectors", fin, fout])
    _, cdat = read_csv(fout)
    cdat = np.atleast_2d(cdat)
    st = {"sequences": len(seqs), "steps": 0, "steps_compared": 0, "flight_steps": 0, "gate_open_steps": 0,
          "max_abs": {"q": 0.0, "q_tilt": 0.0, "bias": 0.0, "rate": 0.0, "e": 0.0, "a_lpf": 0.0, "q_pub": 0.0,
                      "rate_pub": 0.0},
          "gate_mismatch": 0, "knife_edge_cuts": [], "mismatches": 0, "first_mismatches": [],
          "bounds": {"q": MAHONY_Q_TOL, "bias_rad_s": MAHONY_BIAS_TOL, "rate_rad_s": MAHONY_RATE_TOL, "e_rad": MAHONY_E_TOL,
                     "a_lpf_mps2": MAHONY_ALPF_TOL,
                     "gate_knife_edge_mps2": MAHONY_AMB_GATE}, "by_sequence": []}
    i = 0
    for si, sq in enumerate(seqs):
        c = sq["consts"]
        est = GatedMahonyEstimator(c["dt"], c["n_avg"] * c["dt"], kp=f32(c["kp"]), ki=f32(c["ki"]),
                                   accel_gate=f32(c["accel_gate"]), accel_lpf_hz=1.0, bias_limit_rad_s=f32(c["bias_limit"]),
                                   g=f32(c["g"]))
        est.alpha_a = f32(c["alpha_a"])                       # the constant the C gets (its float32 rounding)
        est.n_avg = int(c["n_avg"])
        stop = False
        mx = {"q": 0.0, "bias": 0.0, "steps": 0}
        for r in sq["est"]:
            cr = cdat[i]
            i += 1
            st["steps"] += 1
            if stop:
                continue
            g, a, pad = np.array(r[:3]), np.array(r[3:6]), bool(r[6])
            q_pub, r_pub = est.predict(g, pad)                # v0.8: the published prediction, before the correction
            q = est.update(g, a, pad)
            rate = est.rate(g)
            if int(cr[0]) != si:
                raise SystemExit(f"mahony-vectors row order mismatch at {i}")
            cg = int(cr[11])
            if not pad:
                st["flight_steps"] += 1
                st["gate_open_steps"] += int(est.gate)
            if cg != int(est.gate):
                na = float(np.sqrt(est.a_lpf @ est.a_lpf))
                margin = abs(na - est.g) - est.accel_gate * est.g
                if abs(margin) < MAHONY_AMB_GATE:
                    st["knife_edge_cuts"].append({"seq": sq["name"], "step": int(mx["steps"]), "margin": margin})
                    stop = True
                    continue
                st["gate_mismatch"] += 1
            dq = min(float(np.max(np.abs(cr[1:5] - q))), float(np.max(np.abs(cr[1:5] + q))))
            up_c = quat.rotate_inverse(np.array(cr[1:5], dtype=float) / np.linalg.norm(cr[1:5]), np.array([0, 0, 1.0]))
            up_p = quat.rotate_inverse(q, np.array([0, 0, 1.0]))
            d = {"q": dq, "q_tilt": float(np.linalg.norm(up_c - up_p)), "bias": float(np.max(np.abs(cr[5:8] - est.bias))),
                 "rate": float(np.max(np.abs(cr[8:11] - rate))), "e": float(np.max(np.abs(cr[12:15] - est.e))),
                 "a_lpf": float(np.max(np.abs(cr[15:18] - est.a_lpf))),
                 "q_pub": min(float(np.max(np.abs(cr[21:25] - q_pub))), float(np.max(np.abs(cr[21:25] + q_pub)))),
                 "rate_pub": float(np.max(np.abs(cr[25:28] - r_pub)))}
            for k, v in d.items():
                st["max_abs"][k] = max(st["max_abs"][k], v)
            mx["q"], mx["bias"] = max(mx["q"], dq), max(mx["bias"], d["bias"])
            mx["steps"] += 1
            st["steps_compared"] += 1
            bad = [k for k, tol in (("q", MAHONY_Q_TOL), ("bias", MAHONY_BIAS_TOL), ("rate", MAHONY_RATE_TOL),
                                    ("e", MAHONY_E_TOL), ("a_lpf", MAHONY_ALPF_TOL), ("q_pub", MAHONY_Q_TOL),
                                    ("rate_pub", MAHONY_RATE_TOL)) if not d[k] <= tol]
            if cg != int(est.gate):
                bad.append("gate")
            if bad:
                st["mismatches"] += 1
                if len(st["first_mismatches"]) < 8:
                    st["first_mismatches"].append({"seq": sq["name"], "step": int(mx["steps"]), "fields": bad,
                                                   "d": {k: d[k] for k in bad if k in d}})
        st["by_sequence"].append({"name": sq["name"], "steps": len(sq["est"]), "compared": mx["steps"],
                                  "max_abs_q": mx["q"], "max_abs_bias": mx["bias"]})
    if i != len(cdat):
        raise SystemExit(f"C produced {len(cdat)} mahony rows, expected {i}")
    st["pass"] = (st["mismatches"] == 0 and st["gate_mismatch"] == 0 and st["steps_compared"] > 0.9 * st["steps"]
                  and st["gate_open_steps"] > 0 and st["gate_open_steps"] < st["flight_steps"])
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


def altv_consts(ref: Ref) -> dict:
    a = ref.d["vt"]["profiles"][ref.d["vt"]["default"]]["alt"]
    vp = ref.d["vt"]["vp"]
    return dict(k1=a["k1"], k2=a["k2"], k3=a["k3"], latency=0.1, pad_bias_tau_s=a["pad_tau_s"], bias_limit=a["bias_limit"],
                comp_m=a["comp_m"], inv_u_hover=1.0 / vp.u_hover, throttle_exp=vp.throttle_exp)


def altv_synthetic(ref: Ref, rng: random.Random, n_seq: int) -> list[dict]:
    """Synthetic NAV streams for the VTOL altitude filter: the pad (h = v = 0, accelerometer offset + noise, baro samples
    that only stamp), take-off, climbs, holds and descents of a true altitude history, an accelerometer offset of up to
    ~2 x the MPU-6050 tolerance (beyond the 1 m/s^2 clamp in some), baro noise, a thrust-dependent baro error with and
    without the compensation, baro outages of up to 10 s (the dead reckoning) and a return to the pad.  The envelope is the
    vehicle's (|h| a few tens of metres, |v| a few m/s): the float32 bounds of check_altv are absolute for it."""
    seqs = []
    dt = ref.dt
    div = int(ref.d["baro_div"])
    for s in range(n_seq):
        c = dict(altv_consts(ref))
        c["latency"] = rng.choice([0.0, 0.05, 0.1])
        c["comp_m"] = rng.choice([0.0, 0.0, 0.4, -0.3])
        true_comp = c["comp_m"] if rng.random() < 0.7 else rng.uniform(-0.5, 1.5)
        bias = rng.choice([0.0, 0.05, 0.1, -0.2, 0.7, -0.5, 1.2])   # 1.2: beyond the 1.0 m/s^2 clamp
        rows = []
        h = v = 0.0
        t = 0
        tgt = 0.0
        segs = [("pad", rng.randrange(300, 900)), ("fly", rng.randrange(3000, 12000))]
        if rng.random() < 0.4:
            segs += [("pad", rng.randrange(100, 600)), ("fly", rng.randrange(1000, 5000))]
        outage = None
        for kind, n in segs:
            for k in range(n):
                t += 1
                if kind == "pad":
                    h = v = 0.0
                    a_true = 0.0
                    u = 0.0
                    tgt = rng.uniform(1.0, 12.0)
                else:
                    if k % 1500 == 0:
                        tgt = rng.choice([0.0, rng.uniform(1.0, 12.0)])     # climbs, holds, descents within 0..12 m
                    a_true = min(max(1.5 * (tgt - h) - 2.0 * v, -3.0), 3.0) + rng.gauss(0.0, 0.05)
                    v += a_true * dt
                    h = max(h + v * dt, 0.0)
                    u = min(1.0, max(0.0, 0.91 + 0.05 * a_true + rng.gauss(0.0, 0.01)))
                    if outage is None and rng.random() < 2e-4:
                        outage = (t, t + rng.randrange(500, 2500))          # <= 10 s (the baro-lost limit's order)
                az = a_true + bias + rng.gauss(0.0, 0.3)
                has = (t % div == 0) and not (outage is not None and outage[0] <= t < outage[1])
                hb = h + true_comp * (u * c["inv_u_hover"]) ** 2 + rng.gauss(0.0, 0.25) if has else math.nan
                u_prev = round(u * 250) / 250 if kind == "fly" else 0.0
                rows.append([t, f32(az), float(has), f32(hb) if has else math.nan, 1.0 if kind == "pad" else 0.0, f32(u_prev)])
        seqs.append(dict(name=f"synthetic_{s}", consts=c, alt=np.array(rows)))
    return seqs


def altv_from_replay(rp: dict, dt: float) -> dict:
    """A 6-DOF replay's filter calls as (tick, a, has, h_baro, pad, u_prev) rows (t in s -> ticks)."""
    m = rp["meta"]["alt"]
    rows = []
    for a, _dt, hb, has, t, pad, u in rp["alt"]:
        rows.append([int(round(t / dt)), f32(a), has, f32(hb) if has else math.nan, pad, f32(u)])
    c = dict(k1=m["k1"], k2=m["k2"], k3=m["k3"], latency=m["latency"], pad_bias_tau_s=m["pad_bias_tau_s"],
             bias_limit=m["bias_limit"], comp_m=m["comp_m"], inv_u_hover=m["inv_u_hover"], throttle_exp=m["throttle_exp"])
    return dict(name=rp["name"], consts=c, alt=np.array(rows))


def check_altv(seqs: list[dict], dt: float) -> dict:
    """``sim.state_machine.VtolAltitudeFilter`` (float64) vs ``alt_vtol_*`` (float32, ``--altv-vectors``) on the same
    float32 streams and float32 constants (k1..k3 rounded once, as params_gen.h carries them): h, v, the bias state and
    the compensation of the latest sample.

    Bounds, justification: the filter is the v0.6.3 two-state filter plus a slow bias state (pole 1 / 40 s); its float32
    error comes from the prediction (h += v dt + a dt^2 / 2, v += (a - b) dt: one rounding of ulp(v) per tick, corrected
    by the baro loop with a ~0.5 Hz bandwidth) and the corrections; through a baro outage (dead reckoning) nothing
    corrects it: <= 2 ulp(v) x ticks = 2.4e-7 x 1250 = 3e-4 m/s and its integral in h after 5 s; h itself rounds by
    1/2 ulp(h) per tick (4.8e-7 at 10 m: 1.2e-3 m over a 10 s outage, the worst case).  ALTV_V_TOL / ALTV_H_TOL
    2e-3 (the v0.6.3 filter's own section uses 5e-3), the bias 1e-4 m/s^2 (1/1000 of the 0.1 m/s^2 the state carries),
    each plus ALTV_REL_TOL x the magnitude: the first baro sample after an outage of G seconds corrects with dt_b = G
    (VtolAltitudeFilter.correct, like the v0.6.3 AltitudeFilter), i.e. k2 r G in one step - after the multi-second
    outages of the synthetic streams h and v leave the vehicle's envelope by orders of magnitude (finding reported to the
    reference owner: v -86 m/s after an 8.7 s gap with r = -1 m), and there float32 keeps ~1e-7 x |v| per operation."""
    fin, fout = os.path.join(BUILD, "altv_in.txt"), os.path.join(BUILD, "altv_out.csv")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        for si, sq in enumerate(seqs):
            c = sq["consts"]
            n_avg = max(1, int(round(c["pad_bias_tau_s"] / dt)))
            n2 = c["throttle_exp"] == 2.0
            f.write(f"#A {r32(c['k1'])} {r32(c['k2'])} {r32(c['k3'])} {r32(c['latency'])} {r32(dt)} {n_avg} "
                    f"{r32(c['bias_limit'])} {r32(c['comp_m'])} {r32(c['inv_u_hover'])} {2 if n2 else 0} "
                    f"{r32(c['throttle_exp'])}\n")
            f.write("\n".join(f"{si},{int(r[0])},{r32(r[1])},{int(r[2])},{_f_text(None if not r[2] else r[3])},{int(r[4])},"
                              f"{r32(r[5])}" for r in sq["alt"]) + "\n")
    run_exe(["--altv-vectors", fin, fout])
    _, cdat = read_csv(fout)
    cdat = np.atleast_2d(cdat)
    st = {"sequences": len(seqs), "steps": 0, "baro_samples": 0, "pad_steps": 0, "outage_steps": 0,
          "max_abs": {"h": 0.0, "v": 0.0, "b": 0.0, "comp": 0.0}, "max_h_m": 0.0, "mismatches": 0, "first_mismatches": [],
          "bounds": {"h_m": ALTV_H_TOL_M, "v_mps": ALTV_V_TOL_MPS, "b_mps2": ALTV_B_TOL_MPS2, "comp_m": ALTV_COMP_TOL_M,
                     "relative": ALTV_REL_TOL},
          "by_sequence": []}
    i = 0
    for si, sq in enumerate(seqs):
        c = sq["consts"]
        f = VtolAltitudeFilter(1.0, 1.0, 1.0, f32(c["latency"]), c["pad_bias_tau_s"], f32(c["bias_limit"]), f32(c["comp_m"]),
                               1.0, f32(c["throttle_exp"]))
        f.k1, f.k2, f.k3 = f32(c["k1"]), f32(c["k2"]), f32(c["k3"])       # the C's constants (rounded once)
        f.inv_u_hover = f32(c["inv_u_hover"])
        last_baro = -1
        mx = {"h": 0.0, "v": 0.0, "b": 0.0}
        for r in sq["alt"]:
            cr = cdat[i]
            i += 1
            if int(cr[0]) != si:
                raise SystemExit(f"altv-vectors row order mismatch at {i}")
            tk, a, has, hb, pad, u = int(r[0]), float(r[1]), bool(r[2]), float(r[3]), bool(r[4]), float(r[5])
            f.step(a, dt, hb if has else None, tk * dt, pad, u)
            st["steps"] += 1
            st["baro_samples"] += int(has)
            st["pad_steps"] += int(pad)
            if has:
                last_baro = tk
            elif not pad and last_baro >= 0 and tk - last_baro > 10:
                st["outage_steps"] += 1
            d = {"h": abs(cr[2] - f.h), "v": abs(cr[3] - f.v), "b": abs(cr[4] - f.b), "comp": abs(cr[5] - f.last_comp_m)}
            rel = {"h": abs(f.h), "v": abs(f.v), "b": abs(f.b), "comp": 0.0}
            st["max_abs_v_mps"] = max(st.get("max_abs_v_mps", 0.0), abs(f.v))
            for k, val in d.items():
                st["max_abs"][k] = max(st["max_abs"][k], float(val))
            for k in mx:
                mx[k] = max(mx[k], float(d[k]))
            st["max_h_m"] = max(st["max_h_m"], abs(f.h))
            bad = [k for k, tol in (("h", ALTV_H_TOL_M), ("v", ALTV_V_TOL_MPS), ("b", ALTV_B_TOL_MPS2),
                                    ("comp", ALTV_COMP_TOL_M)) if not d[k] <= tol + ALTV_REL_TOL * rel[k]]
            if bad:
                st["mismatches"] += 1
                if len(st["first_mismatches"]) < 8:
                    st["first_mismatches"].append({"seq": sq["name"], "tick": tk, "fields": bad,
                                                   "d": {k: float(d[k]) for k in bad}})
        st["by_sequence"].append({"name": sq["name"], "steps": len(sq["alt"]), **{f"max_abs_{k}": v for k, v in mx.items()}})
    if i != len(cdat):
        raise SystemExit(f"C produced {len(cdat)} altv rows, expected {i}")
    st["pass"] = st["mismatches"] == 0 and st["steps"] > 0 and st["outage_steps"] > 0 and st["pad_steps"] > 0
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


def run_nav_sections(ref: Ref, a, report: dict, recs: list[dict] | None, note: str) -> list[str]:
    """Section 7b (v0.7.1 VTOL NAV): the estimator and the altitude filter on synthetic streams and on 6-DOF sensor
    replays (the ccrec/1 recordings of sixdof_records of the --nav-runs scenarios; fw09 X1: their constants were checked
    against the build of the run's configuration by check_sixdof_params before this replay)."""
    t0 = time.perf_counter()
    rng = random.Random(a.seed + 21)
    replays = [] if recs is None else [r["nav"] for r in recs if r["name"] in a.nav_runs and r["nav"] is not None]
    got = {rp["meta"]["name"] for rp in replays}
    missing = [] if recs is None else [nm for nm in a.nav_runs if nm not in got]
    mseq = mahony_synthetic(ref, rng, a.nav_seqs)
    for rp in replays:
        m = rp["meta"]["est"]
        mseq.append(dict(name=rp["name"], consts=dict(dt=m["dt"], n_avg=m["n_avg"], kp=m["kp"], ki=m["ki"],
                                                       accel_gate=m["accel_gate"], g=m["g"], alpha_a=m["alpha_a"],
                                                       bias_limit=m["bias_limit"]),
                         est=np.array([[f32(x) for x in r[:6]] + [r[6]] for r in rp["est"]])))
    ms = check_mahony(mseq)
    aseq = altv_synthetic(ref, rng, a.nav_seqs) + [altv_from_replay(rp, ref.dt) for rp in replays]
    al = check_altv(aseq, ref.dt)
    ms["replays"] = al["replays"] = [rp["name"] for rp in replays]
    ms["replay_note"] = al["replay_note"] = note
    ms["replays_missing"] = al["replays_missing"] = missing         # v0.8.1: a requested replay that did not load FAILS
    if missing:
        ms["pass"] = al["pass"] = False
        print(f"   6-DOF NAV recordings missing: {missing} -> both NAV sections FAIL")
    ms["wall_s"] = al["wall_s"] = round(time.perf_counter() - t0, 1)
    report["nav_estimator_vtol"], report["nav_alt_filter_vtol"] = ms, al
    print(f"== VTOL NAV estimator (GatedMahonyEstimator): {ms['steps']} steps in {ms['sequences']} sequences "
          f"({len(replays)} 6-DOF replays: {', '.join(ms['replays']) or 'none'}), {ms['steps_compared']} compared, gate "
          f"open {ms['gate_open_steps']} of {ms['flight_steps']} flight steps; max |d| "
          f"{ {k: float(f'{v:.3g}') for k, v in ms['max_abs'].items()} } (bounds {ms['bounds']}); gate knife-edge cuts "
          f"{len(ms['knife_edge_cuts'])}, gate mismatches {ms['gate_mismatch']}, mismatching steps {ms['mismatches']} "
          f"-> {'PASS' if ms['pass'] else 'FAIL'}")
    for m in ms["first_mismatches"][:4]:
        print(f"   mismatch: {m}")
    print(f"== VTOL NAV altitude filter (VtolAltitudeFilter): {al['steps']} steps in {al['sequences']} sequences "
          f"({al['baro_samples']} baro samples, {al['pad_steps']} pad steps, {al['outage_steps']} steps in baro outages, "
          f"|h| up to {al['max_h_m']:.1f} m); max |d| { {k: float(f'{v:.3g}') for k, v in al['max_abs'].items()} } "
          f"(bounds {al['bounds']}); mismatching steps {al['mismatches']} -> {'PASS' if al['pass'] else 'FAIL'}")
    for m in al["first_mismatches"][:4]:
        print(f"   mismatch: {m}")
    print(f"   6-DOF replays: {note}")
    return ["nav_estimator_vtol", "nav_alt_filter_vtol"]


def run_vtol_sections(ref: Ref, a, report: dict) -> list[str]:
    """Sections 4b-4d and 5b/6b (v0.7 VTOL); returns the report keys whose 'pass' counts."""
    keys = []
    t0 = time.perf_counter()
    vs = onb_vtol_random_sequences(ref, a.vtol_seqs, a.seed + 11)
    rng = random.Random(a.seed + 12)
    runs1 = stack_1d_flights(ref, rng, a.stack_1d)
    rec_dir, note6 = (None, "skipped (--no-vtol-6dof)") if a.no_vtol_6dof else sixdof_records(a)
    recs = [] if rec_dir is None else load_sixdof_records(rec_dir, tuple(dict.fromkeys(list(a.vtol_runs) +
                                                                                       list(a.nav_runs))))
    if rec_dir is not None:
        # fw09 X1: the parameters each 6-DOF run flew with vs the constants of the firmware build of its configuration -
        # BEFORE the replays (which hand both sides the run's own numbers): a G1-type build fails here
        prm = check_sixdof_params(recs)
        report["sixdof_params"] = prm
        keys.append("sixdof_params")
        print_sixdof_params(prm, "6-DOF run parameters vs the firmware build of each run's configuration (X1)")
    runs6 = [r["run"] for r in recs if r["name"] in a.vtol_runs]
    sk, sup_replays = check_stack(ref, runs1 + runs6)
    sk["six_dof_note"] = note6 or "ok"
    if rec_dir is not None:                         # v0.8.1: a requested recording that did not load is a FAIL, not
        got = {r["name"][len("6dof_"):] for r in runs6}                     # a silent pass on fewer runs
        sk["six_dof_missing"] = [nm for nm in a.vtol_runs if nm not in got]
        if sk["six_dof_missing"]:
            sk["pass"] = False
            print(f"   6-DOF stack recordings missing: {sk['six_dof_missing']} -> the stack replay FAILS")
    report["sixdof_records"] = {"dir": rec_dir, "note": note6}
    sk["wall_s"] = round(time.perf_counter() - t0, 1)
    ov = check_onboard_vtol(ref, vs + sup_replays)
    report["onboard_vtol"] = ov
    report["stack"] = sk
    keys += ["onboard_vtol", "stack"]
    print(f"== VTOL supervisor: {ov['ticks']} ticks in {ov['sequences']} sequences "
          f"({', '.join(f'{k}: {v['sequences']} seq / {v['ticks']} ticks' for k, v in ov['groups'].items())}), "
          f"{ov['frames']} uplink frames, {ov['telemetry_frames']} TELEM2 replies; mismatching ticks {ov['mismatch_ticks']}; "
          f"I10 boots {ov['i10_boots']}; inputs moved out of the float32 band {ov['nudged_inputs']} "
          f"-> {'PASS' if ov['pass'] else 'FAIL'}")
    print(f"   interlocks: {dict(sorted(ov['interlocks_seen'].items()))}")
    print(f"   rejections: {dict(sorted(ov['rejections_seen'].items()))}")
    print(f"   executed:   {dict(sorted(ov['commands_executed'].items()))}")
    print(f"   transitions: {dict(sorted(ov['transitions_seen'].items()))}")
    print(f"   phase changes: {dict(sorted(ov['phase_changes_seen'].items()))}; ticks by phase {ov['ticks_by_phase']}")
    print(f"   LANDED rules (v0.7.1, -> SAFE): {ov['landed_rules_seen']}")
    if ov["missing_interlocks"] or ov["missing_rejections"] or ov["missing_phase_changes"] or ov["missing_landed_rules"]:
        print(f"   MISSING coverage: interlocks {ov['missing_interlocks']}, rejections {ov['missing_rejections']}, phase "
              f"changes {ov['missing_phase_changes']}, LANDED rules {ov['missing_landed_rules']}")
    for m in ov["first_mismatches"][:5]:
        print(f"   mismatch: {m}")
    print(f"== FCU stack replay (supervisor -> vertical controller -> ESC): {len(sk['runs'])} runs "
          f"({len(runs1)} 1-D closed loops, {len(runs6)} 6-DOF VTOL runs; 6-DOF: {sk['six_dof_note']}), {sk['ticks']} ticks, "
          f"{sk['ticks_compared']} compared; mismatching ticks {sk['mismatch_ticks']}; knife-edge controller decisions "
          f"{sk['knife_edges']} ({sk['knife_edges_resynced']} agreed again after {VC_RESYNC_TICKS} ticks, "
          f"{sk['ambiguous_runs']} runs cut there); TELEM2 throttle one count off {sk['telem_throttle_one_count_off']}; max |d| "
          f"{ {k: float(f'{v:.3g}') for k, v in sk['max_abs'].items()} } -> {'PASS' if sk['pass'] else 'FAIL'}")
    for r in sk["runs"]:
        if r["name"].startswith("6dof") or r["mismatch_ticks"] or r["compared_until"] < r["ticks"] or r["knife_edges"]:
            print(f"   {r}")
    for m in sk["first_mismatches"][:5]:
        print(f"   mismatch: {m}")
    vcs = check_vertical(vc_random_sequences(ref, random.Random(a.seed + 13), a.vc_seqs))
    report["vertical"] = vcs
    keys.append("vertical")
    print(f"== vertical controller: {vcs['ticks']} ticks in {vcs['sequences']} sequences, {vcs['ticks_compared']} compared "
          f"; knife-edge decisions {vcs['knife_edges']} ({vcs['knife_edges_resynced']} agreed again after {VC_RESYNC_TICKS} "
          f"ticks, {vcs['ambiguous_sequences']} sequences cut there); k_T learnt on {vcs['kt_learn_ticks']} ticks, range "
          f"{[round(x, 3) for x in vcs['kt_range']]}; modes {vcs['ticks_by_mode']}, sat "
          f"{vcs['sat_ticks']}, at_target {vcs['at_target_ticks']}, touchdowns {vcs['touchdowns']}, baro-lost landing "
          f"ticks {vcs['baro_lost_land_ticks']}, non-finite inputs {vcs['nonfinite_inputs']}; max |d| "
          f"{ {k: float(f'{v:.3g}') for k, v in vcs['max_abs'].items()} } (tolerances {vcs['tolerances']}); mismatching "
          f"ticks {vcs['mismatch_ticks']} -> {'PASS' if vcs['pass'] else 'FAIL'}")
    for m in vcs["first_mismatches"][:5]:
        print(f"   mismatch: {m}")
    # v0.7.1b: the attitude controller with the VTOL build's gains (control.vtol_overrides; core/fc_config.c fc_vtol_gains)
    crng = np.random.default_rng(a.seed + 16)
    cseqs = random_controller_sequences(crng, max(a.random_seqs // 2, 10), ref.dt) + yaw_windup_sequences(crng, 6, ref.dt)
    ntilt = 12                                          # v0.8: tilt-setpoint sequences (attitude_ctrl_set_tilt / _finish)
    cseqs += tilt_controller_sequences(crng, ntilt, ref.dt)
    cv = check_controller(ref, cseqs, ["random"] * (len(cseqs) - 6 - ntilt) + ["yaw_windup"] * 6 + ["tilt_v08"] * ntilt,
                          vtol=True)
    report["controller_vtol"] = cv
    keys.append("controller_vtol")
    cv["overrides"] = vtol_overrides(ref.p["control"])
    print(f"== attitude controller, VTOL gains (control.vtol_overrides {cv['overrides']}): {cv['vectors']} vectors in {cv['sequences']} sequences; max |d delta| = {cv['max_abs_d_delta_rad']:.3e} rad "
          f"(bound {DELTA_BOUND_RAD:.0e}), max |d I| = {cv['max_abs_d_integ_nm']:.3e} N m; rows with |I_z| beyond the "
          f"ballistic limit {cv['rows_integ_z_beyond_ballistic_limit']}; sat-flag mismatches ambiguous "
          f"{cv['sat_flag_mismatch_ambiguous']}, hard {cv['sat_flag_mismatch_hard']} -> {'PASS' if cv['pass'] else 'FAIL'}")
    p2 = check_protocol_v2(ref, a.seed + 14, a.proto_streams // 2, 2000)
    report["protocol_v2"] = p2
    keys.append("protocol_v2")
    print(f"== link protocol v2 (TELEM2, LEN_MAX {lp.LEN_MAX}, codes 8..11): {p2['streams']} streams, frames by TYPE "
          f"{p2['py_frames_by_type']}, {p2['telem2_encodings']} TELEM2 encodings; mismatches {p2['mismatches']} "
          f"-> {'PASS' if p2['pass'] else 'FAIL'}")
    for m in p2["first_mismatches"][:3]:
        print(f"   mismatch: {m}")
    i2 = check_ilink_v2(int(ref.p["flight_computer"]["link"]["vehicle_id"]), ref.loop_hz, a.seed + 15,
                        a.ilink_streams // 2, a.ilink_snaps // 4)
    report["ilink_v2"] = i2
    keys.append("ilink_v2")
    print(f"== ilink SNAP2: {i2['streams']} streams ({i2['py_frames']} frames), {i2['encodings']} encodings, "
          f"{i2['snapshots']} FCU SNAP2, {i2['telem2_frames']} SNAP2 -> TELEM2 ({i2['telem_angle_fields_one_quantum_off']} "
          f"angle fields one quantum off); mismatches {i2['mismatches']} -> {'PASS' if i2['pass'] else 'FAIL'}")
    for m in i2["first_mismatches"][:3]:
        print(f"   mismatch: {m}")
    keys += run_nav_sections(ref, a, report, None if rec_dir is None else recs, note6)
    return keys




# --------------------------------------------------------------------------- #
# v0.8 GPS position hold (sections 8a-8f)
# --------------------------------------------------------------------------- #

# 8a position controller: float32 C vs float64 reference, both on float32-representable parameters and inputs
# (justification): the step is ~120 flops plus <= 4 sqrt; a stateless output carries a few ulp (a_cmd ~ 4 m/s^2:
# ulp 4.8e-7).  State: the integrator gains at most 1/2 ulp(i_max 3) = 1.2e-7 m/s^2 per step (linear worst case 1.2e-4
# over 1000 steps of saturation, typically a random walk ~4e-6); dv low-passes (vx - v_prev) / T: the cm -> m conversion
# rounds v to 1/2 ulp(3 m/s) = 1.2e-7, x 1/T = 50 -> 6e-6 m/s^2 per step into a contractive filter (alpha 0.47): <= 1.3e-5;
# the C's 1/T, ki T and the series' reciprocals add <= 1 ulp each.  a_cmd = kv ev + I - kd dv: <= 2e-4 worst case.
# Bounds: a_cmd / integ / dv 2e-4 m/s^2 (= 2e-5 rad of tilt, 1/5 of the attitude controller's DELTA_BOUND), tilt 2e-5 rad,
# v_sp 1e-5 m/s, hold / dist 1e-5 m (cm integers x 0.01 at <= 20 m: 1 ulp = 1.9e-6), cos_lim 1e-6, a_lim^2 1e-5 relative
# + POS_A2_ABS_TOL.  Discrete outputs (mode, requests, limited, guard, engaged, has_hold, lost, implausible, lim_run, vexc_run,
# guard_left) must be equal on every step.
POS_A_TOL = 2e-4
POS_TILT_TOL = 2e-5
POS_V_TOL = 1e-5
POS_X_TOL = 1e-5
POS_COS_TOL = 1e-6
POS_A2_REL_TOL = 1e-5
POS_AMB_REL = 1e-5              # |a|^2 vs a_lim^2 within this (relative): a knife-edge (check_pos)
POS_A2_ABS_TOL = 5e-5           # a_lim^2 = g^2 (1 - c^2) / c^2 is ill-conditioned as cos_lim -> 1: c = (1 + m) / x carries
                                # <= 3/2 ulp(1) = 1.8e-7 (two roundings in x, m, one in the division), d(a_lim^2) = 2 g^2
                                # dc / c^3 <= 3.5e-5 (m/s^2)^2 at c ~ 1; 5e-5 is d(a_lim) <= 2.5e-4 m/s^2 (2.5e-5 rad) for
                                # a_lim >= 0.1 m/s^2 (measured 1.2e-5 at cos_lim 0.9945)
POS_C_FLOAT = ("T", "g", "tw", "kp_pos", "kv", "ki", "kd", "alpha_d", "i_max", "v_max", "v_max_land", "flare_speed",
               "flare_altitude_m", "cos_cap", "cos_flare_cap", "cos_excess", "sin_excess", "margin_g", "margin_frac",
               "h_enable_m", "fence_m", "spin_z_rad", "spin_xy_rad2", "authority_min_ktw", "tilt_fade_m", "slew_per_tick")
POS_C_INT = ("step_ticks", "guard_steps", "timeout_steps", "land_steps", "plaus_limited_steps", "plaus_vexc_steps",
             "plaus_reset_window_steps", "plaus_resets")
POS_BASE_FLOAT = ("g", "tw", "kp_pos", "kv", "ki", "kd", "i_max", "v_max", "v_max_land", "flare_speed", "flare_altitude_m",
                  "margin_g", "margin_frac", "h_enable_m", "fence_m", "plaus_v_excess_mps", "authority_min_ktw",
                  "tilt_fade_m")


class PosParams32(pcm.PositionParams):
    """sim.position_ctrl.PositionParams whose derived constants are the float32 values the firmware holds (the base
    floats are float32 already, see pos_params32): the reference then compares its float32 inputs with exactly the
    C's thresholds."""

    @property
    def T(self) -> float:
        return f32(1.0 / self.loop_hz)

    @property
    def alpha_d(self) -> float:
        return f32(1.0 - math.exp(-2.0 * math.pi * self.kd_lpf_hz * (1.0 / self.loop_hz)))   # the parent's, T = 1 / hz

    @property
    def cos_cap(self) -> float:
        return f32(math.cos(math.radians(self.tilt_cap_deg)))

    @property
    def cos_flare_cap(self) -> float:
        return f32(math.cos(math.radians(min(self.tilt_cap_deg, self.flare_tilt_cap_deg))))

    @property
    def cos_excess(self) -> float:
        return f32(math.cos(math.radians(self.tilt_guard_excess_deg)))

    @property
    def sin_excess(self) -> float:
        return f32(math.sin(math.radians(self.tilt_guard_excess_deg)))

    @property
    def spin_z_rad(self) -> float:
        return f32(math.radians(self.spin_z_dps))

    @property
    def spin_xy_rad2(self) -> float:
        return f32(math.radians(self.spin_xy_dps) ** 2)

    @property
    def slew_per_tick(self) -> float:
        return f32(math.radians(self.tilt_rate_dps) / self.fcu_hz)


def pos_params32(pp: pcm.PositionParams, **overrides) -> PosParams32:
    """The reference parameters with every float the step reads float32-representable (overrides by field name)."""
    kw = dataclasses.asdict(pp)
    kw.update(overrides)
    for k in POS_BASE_FLOAT:
        kw[k] = f32(kw[k])
    p32 = PosParams32(**kw)
    if f32(p32.v_max + p32.plaus_v_excess_mps) != p32.v_max + p32.plaus_v_excess_mps:
        raise ValueError("v_max + plaus_v_excess_mps must be float32-representable (the C's plaus_vt)")
    p32.check()
    return p32


def pos_c_overrides(pp: pcm.PositionParams) -> dict:
    """Every pos_params_t field of core/position_ctrl.h for a #P / #K / #N line (test_core.c pos_override): the
    generator's mapping (tools/gen_firmware_params.py pos_c_fields; v0.8.1: keep_trim of the touchdown policy)."""
    return gfp.pos_c_fields(pp)


def ref_pos_params(ref: Ref) -> pcm.PositionParams:
    """The generator's position-hold parameters of the default build profile (params_gen.h POS_*)."""
    vt = ref.d["vt"]
    pp = vt["profiles"][vt["default"]]["pp"]
    if pp is None:
        raise SystemExit("params.json has no position hold (mission.vtol.position_hold.enabled)")
    return pp


def gps_data_of(d: dict) -> pcm.GpsData:
    """The slot integers of a row -> the reference's GpsData (sim.nlink_proto.GpsSlotRx dequantisation)."""
    return pcm.GpsData(flags=d["flags"], num_sv=9, hacc_dm=25, reset_count=d["rc"], off=(d["off"][0] / 100.0,
                       d["off"][1] / 100.0), p=(d["p"][0] / 100.0, d["p"][1] / 100.0), v=(d["v"][0] / 100.0,
                       d["v"][1] / 100.0), cos_psi=d["cs"][0] / 32767.0, sin_psi=d["cs"][1] / 32767.0)


def pos_random_sequences(ref: Ref, rng: random.Random, n_seq: int) -> list[dict]:
    """Random position-controller histories: phase scripts (CLIMB / HOLD / LAND with the motor on, ground phases, motor
    off), altitudes through h_enable, the flare and the fade band, k_T across the thrust-limited / capped / no-margin
    regions, tilts and rates around the guard thresholds, vertical-loop saturation, slot sets with dropouts, FIX_OK
    loss, NAV resets (offset jumps), a random-walk position and velocity (limits, fence, plausibility (a) / (b) run
    long enough), heading cos / sin; parameter variants (short windows, few resets, small fence / v_max / i_max)."""
    base = ref_pos_params(ref)
    seqs = []
    for s in range(n_seq):
        ov = {}
        if s % 4 == 1:
            ov = dict(plaus_resets=rng.choice([1, 2, 3, 4]), plaus_reset_window_s=rng.choice([0.5, 2.0, 10.0]),
                      fence_m=rng.choice([3.0, 6.0]), gps_timeout_s=rng.choice([0.2, 0.5]), gps_land_after_s=1.0)
        elif s % 4 == 2:
            ov = dict(v_max=0.75, plaus_v_excess_mps=0.5, i_max=0.5, plaus_limited_s=0.4, plaus_v_excess_s=0.3,
                      guard_hold_s=0.1, tilt_cap_deg=12.0)
        elif s % 4 == 3:
            ov = dict(kd=0.0, ki=1.0, h_enable_m=1.0, tilt_fade_m=0.6, flare_altitude_m=1.5, margin_g=0.3,
                      margin_frac=1.0)
        if s % 8 >= 6:                                                    # v0.8.1: the v0.8 fade (the comparison build);
            ov = dict(ov, touchdown_policy="fade")                        # the others fly params' keep_trim
        pp = pos_params32(base, **ov)
        n = rng.randrange(300, 1200)
        rows = []
        px, py, vx, vy = rng.uniform(-3, 3), rng.uniform(-3, 3), 0.0, 0.0
        offx = offy = 0
        rc = rng.randrange(256)
        psi = rng.uniform(-math.pi, math.pi)
        kt = f32(rng.uniform(0.9, 1.2))
        phase, motor = lp.PHASE_CLIMB, True
        h = 0.0
        drift_left, drift = 0, (0.0, 0.0)
        script = sorted(rng.sample(range(5, n), k=min(n - 6, rng.randrange(3, 9))))
        for t in range(n):
            if t in script:
                phase = rng.choice([lp.PHASE_CLIMB, lp.PHASE_HOLD, lp.PHASE_HOLD, lp.PHASE_HOLD, lp.PHASE_LAND,
                                    lp.PHASE_LANDED, lp.PHASE_SPOOL])
                motor = phase not in (lp.PHASE_LANDED,) and rng.random() < 0.95
            if phase == lp.PHASE_CLIMB:
                h = min(h + 0.02, 8.0)
            elif phase == lp.PHASE_LAND:
                h = max(h - rng.choice([0.002, 0.01, 0.03]), -0.05 if s % 2 == 0 else -1.2)   # -1.2: lower ground
            elif phase in (lp.PHASE_LANDED, lp.PHASE_SPOOL):
                h = rng.uniform(-0.05, 0.1)
            else:
                h = h + rng.uniform(-0.02, 0.02)
            ax, ay = rng.gauss(0, 0.6), rng.gauss(0, 0.6)
            if rng.random() < 0.01:
                ax, ay = rng.choice([-6, 6]), rng.choice([-6, 6])          # a gust: a large velocity error
            if drift_left > 0:                                             # a drift away (fence, v_sp at its limit)
                drift_left -= 1
                ax, ay = 3.0 * drift[0] - vx, 3.0 * drift[1] - vy
            elif rng.random() < 0.003:
                drift_left, drift = rng.randrange(100, 400), (rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5))
            vx = max(-4.0, min(4.0, 0.98 * vx + ax * 0.02))
            vy = max(-4.0, min(4.0, 0.98 * vy + ay * 0.02))
            px += vx * 0.02
            py += vy * 0.02
            if rng.random() < (0.004 if s % 3 == 0 else 0.0008):           # a NAV position reset: offset + counter
                dx, dy = rng.randrange(-800, 800), rng.randrange(-800, 800)
                offx = max(-32000, min(32000, offx + dx))
                offy = max(-32000, min(32000, offy + dy))
                px += dx / 100.0
                py += dy / 100.0
                rc = (rc + rng.choice([1, 1, 2, 5])) & 0xFF
            if rng.random() < (0.002 if s % 3 == 0 else 0.0004):
                rc = (rc + 1) & 0xFF                                       # a velocity reset: counter only
            if rng.random() < 0.02:
                kt = f32(min(1.4, max(0.5, kt + rng.gauss(0, 0.05))))
            psi += rng.gauss(0, 0.01)
            has = rng.random() > (0.4 if (t // 100) % 7 == 3 else 0.05)    # dropouts, a long one now and then
            fix = rng.random() > (0.5 if (t // 150) % 5 == 2 else 0.01)
            tilt = abs(rng.gauss(0, 0.12)) if rng.random() > 0.02 else rng.uniform(0.3, 0.6)
            # physics-review fix: the vertical controller's flare band / touchdown signatures / take-off check and the
            # NAV's GPS-IMU inconsistency flag (stretches of it, so plausibility (a) runs long enough to latch)
            low_alt = phase == lp.PHASE_LAND and h <= pp.flare_altitude_m and rng.random() > 0.02
            ground = low_alt and (h < 0.05 or rng.random() < 0.03)
            lifted = t > 40 or rng.random() < 0.3
            incons = 0x80 if (t // 200) % 3 == 1 else 0
            rows.append(dict(
                seq=t, phase=phase, motor=int(motor), h=f32(h), cos=f32(math.cos(tilt)),
                low_alt=int(low_alt), ground=int(ground), lifted=int(lifted),
                r=[f32(rng.gauss(0, 0.5)), f32(rng.gauss(0, 0.5)),
                   f32(rng.gauss(0, 0.6) if rng.random() > 0.01 else rng.choice([-4.0, 4.0]))],
                kt=kt, sat=int(rng.random() < 0.06), has=int(has), flags=(0x2F if fix else 0x2E) | incons, rc=rc,
                off=[offx, offy], p=[int(max(-32000, min(32000, round(px * 100)))),
                                     int(max(-32000, min(32000, round(py * 100))))],
                v=[int(round(vx * 100)), int(round(vy * 100))],
                cs=[int(round(math.cos(psi) * 32767)), int(round(math.sin(psi) * 32767))]))
        seqs.append(dict(pp=pp, ov=ov, rows=rows))
    return seqs


def check_pos(ref: Ref, seqs: list[dict]) -> dict:
    """8a: sim.position_ctrl.PositionHold (float64, PosParams32) vs core/position_ctrl.c through its background pieces
    (test_core --pos-vectors: pos_step = pos_latch + P0..P10) on the same float32 inputs; bounds above."""
    fin, fout = os.path.join(BUILD, "pos_in.txt"), os.path.join(BUILD, "pos_out.csv")
    exp = []
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        for i, sq in enumerate(seqs):
            pp = sq["pp"]
            f.write(f"#P {i} {_ov_text(pos_c_overrides(pp), 'pp.')}\n")
            ph = pcm.PositionHold(pp)
            out = []
            for d in sq["rows"]:
                data = gps_data_of(d) if d["has"] else None
                o = ph.step(pcm.PosInputs(phase=d["phase"], motor_on=bool(d["motor"]), h_m=d["h"], cos_tilt=d["cos"],
                                          rate=tuple(d["r"]), k_t=d["kt"], vc_sat_high=bool(d["sat"]), data=data,
                                          vc_low_alt=bool(d["low_alt"]), vc_ground=bool(d["ground"]),
                                          lifted=bool(d["lifted"])))
                out.append(dict(o=o, implausible=int(ph.implausible), lim_run=ph.lim_run, vexc_run=ph.vexc_run,
                                guard_left=ph.guard_left, dv=tuple(ph.dv)))
                f.write(",".join(str(x) for x in (d["seq"], d["phase"], d["motor"])) + "," +
                        ",".join(r32(x) for x in (d["h"], d["cos"], *d["r"], d["kt"])) + "," +
                        ",".join(str(x) for x in (d["sat"], d["has"], d["flags"], d["rc"], *d["off"], *d["p"], *d["v"],
                                                  *d["cs"], d["low_alt"], d["ground"], d["lifted"])) + "\n")
            exp.append(out)
    run_exe(["--pos-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        f.readline()
        lines = f.read().splitlines()
    st = {"sequences": len(seqs), "steps": 0, "mismatch_steps": 0, "first_mismatches": [],
          "max_abs": {k: 0.0 for k in ("a_cmd", "integ", "dv", "tilt", "v_sp", "hold", "dist", "cos_lim", "a_lim2_rel",
                                       "trim")},
          "modes": {}, "requests_seen": 0, "limited_steps": 0, "guard_steps": 0, "implausible_steps": 0,
          "hold_steps": 0, "variants": sorted({",".join(sorted(sq["ov"])) or "default" for sq in seqs})}
    st["knife_edge_cuts"] = []
    i = 0
    for si, (sq, out) in enumerate(zip(seqs, exp)):
        cut = False
        for d, r in zip(sq["rows"], out):
            c = lines[i].split(",")
            i += 1
            if cut:
                continue
            o = r["o"]
            st["steps"] += 1
            bad = []
            disc = dict(mode=o.mode, requests=o.requests, limited=int(o.limited), guard=int(o.guard),
                        engaged=int(o.engaged), has_hold=int(o.hold is not None), lost=o.lost_steps,
                        implausible=r["implausible"], lim_run=r["lim_run"], vexc_run=r["vexc_run"],
                        guard_left=r["guard_left"])
            cd = dict(mode=int(c[1]), requests=int(c[2]), limited=int(c[3]), guard=int(c[4]), engaged=int(c[5]),
                      has_hold=int(c[6]), lost=int(c[7]), implausible=int(c[23]), lim_run=int(c[24]),
                      vexc_run=int(c[25]), guard_left=int(c[26]))
            bad += [k for k in disc if disc[k] != cd[k]]
            cf = [float(x) for x in c[8:23]] + [float(c[27]), float(c[28])]
            hold = o.hold if o.hold is not None else (0.0, 0.0)
            pairs = {"tilt": (cf[0:4], [*o.tilt_l, *o.tilt_w], POS_TILT_TOL), "a_cmd": (cf[4:6], list(o.a_cmd), POS_A_TOL),
                     "v_sp": (cf[6:8], list(o.v_sp), POS_V_TOL), "hold": (cf[8:10], list(hold), POS_X_TOL),
                     "integ": (cf[13:15], list(o.integ), POS_A_TOL), "dv": (cf[15:17], list(r["dv"]), POS_A_TOL),
                     "cos_lim": ([cf[11]], [o.cos_lim], POS_COS_TOL),
                     "trim": ([float(c[29]), float(c[30])], list(o.trim_l), POS_TILT_TOL)}     # v0.8.1
            for k, (cv, pv, tol) in pairs.items():
                dm = max(abs(a - b) for a, b in zip(cv, pv))
                st["max_abs"][k] = max(st["max_abs"][k], dm)
                if not dm <= tol:
                    bad.append(k)
            if math.isnan(o.dist_m) != math.isnan(cf[10]):
                bad.append("dist")
            elif not math.isnan(o.dist_m):
                dm = abs(cf[10] - o.dist_m)
                st["max_abs"]["dist"] = max(st["max_abs"]["dist"], dm)
                if not dm <= POS_X_TOL:
                    bad.append("dist")
            da = abs(cf[12] - o.a_lim2)
            st["max_abs"]["a_lim2_rel"] = max(st["max_abs"]["a_lim2_rel"], da / max(o.a_lim2, 1e-12) if o.a_lim2 > 0 else da)
            if not da <= POS_A2_REL_TOL * o.a_lim2 + POS_A2_ABS_TOL:
                bad.append("a_lim2")
            nm = pcm.MODE_NAMES[o.mode]
            st["modes"][nm] = st["modes"].get(nm, 0) + 1
            st["requests_seen"] |= o.requests
            st["limited_steps"] += int(o.limited)
            st["guard_steps"] += int(o.guard)
            st["implausible_steps"] += r["implausible"]
            st["hold_steps"] += int(o.hold is not None)
            if bad and set(bad) <= {"limited", "lim_run", "a_cmd", "tilt", "trim"} and "limited" in bad:
                # the a limit |a|^2 > a_lim^2 decided by rounding: |a| == a_lim in exact arithmetic happens by design when
                # the vertical loop is saturated high (a_lim^2 = min(a_lim^2, |a_prev|^2) with the integrator frozen).  The
                # unlimited side's |a|^2 gives the float64 margin; within POS_AMB_REL the two sides may differ - the
                # sequence is compared up to here (its limited_prev / lim_run differ from now on)
                na2 = (o.a_cmd[0] ** 2 + o.a_cmd[1] ** 2) if not o.limited else (cf[4] ** 2 + cf[5] ** 2)
                if abs(na2 - o.a_lim2) <= POS_AMB_REL * o.a_lim2 + POS_A2_ABS_TOL:
                    st["knife_edge_cuts"].append({"seq": si, "step": d["seq"], "fields": bad, "sat_high": d["sat"],
                                                  "rel_margin": abs(na2 - o.a_lim2) / max(o.a_lim2, 1e-12)})
                    cut = True
                    continue
            if bad:
                st["mismatch_steps"] += 1
                if len(st["first_mismatches"]) < 10:
                    st["first_mismatches"].append({"seq": si, "step": d["seq"], "fields": bad,
                                                   "py": {k: disc[k] for k in bad if k in disc},
                                                   "c": {k: cd[k] for k in bad if k in cd}})
    if i != len(lines):
        raise SystemExit(f"pos-vectors: C wrote {len(lines)} rows, expected {i}")
    req_all = pcm.REQ_GPS_LOST | pcm.REQ_FENCE | pcm.REQ_IMPLAUSIBLE | pcm.REQ_AUTHORITY
    st["coverage"] = {"every_mode": len(st["modes"]) == 5, "every_request": st["requests_seen"] == req_all,
                      "limited": st["limited_steps"] > 0, "guard": st["guard_steps"] > 0,
                      "implausible": st["implausible_steps"] > 0}
    st["bounds"] = {"a_cmd/integ/dv": POS_A_TOL, "tilt": POS_TILT_TOL, "v_sp": POS_V_TOL, "hold/dist": POS_X_TOL,
                    "cos_lim": POS_COS_TOL, "a_lim2_rel": POS_A2_REL_TOL, "a_lim2_abs": POS_A2_ABS_TOL}
    st["pass"] = (st["mismatch_steps"] == 0 and st["steps"] > 0 and all(st["coverage"].values()) and
                  len(st["knife_edge_cuts"]) <= max(1, len(seqs) // 10))
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


# 8b NAV GPS side: float32 C vs float64 reference on the same UART bytes and IMU samples.  Integer paths (parser, epoch
# assembler, iTOW clock, configurator, the quality / freshness / gate bookkeeping, the counters, the slot header) must be
# equal on every tick.  Floats (justification): the state is compared at the step (k mod 5 == 4), where both have fused the
# cycle's epoch (the C over j = 1..3, the reference at j = 0) and integrated the step.  Per step the float32 integration
# rounds p by 1/2 ulp(|p| <= 64 m) = 3.8e-6 m and v by 1/2 ulp(4 m/s) = 2.4e-7; the fixed-gain fusion contracts the error
# every epoch (K_p ~ 0.1-0.5), so it stays near ulp-size: bound p 1e-3 m (1/10 of a slot count), v 1e-4 m/s, b 1e-4
# m/s^2, the reset offset 1e-4 m (a float32 sum of a few innovations); the heading's cos / sin 1e-5 (atan2f, cosf / sinf
# <= 2 ulp; the yaw Kalman gain from P float32), P_psi 1e-4 relative, the yaw correction 1e-5 rad.  Slot payloads: equal,
# or one count (cm, cm/s, Q15) where the float32 value lies within the bound of a rounding boundary (counted).
GPS_P_TOL = 1e-3
GPS_V_TOL = 1e-4
GPS_B_TOL = 1e-4
GPS_OFF_TOL = 1e-4
GPS_CS_TOL = 1e-5
GPS_PPSI_REL = 1e-4
GPS_YAW_TOL = 1e-5


def _quat_of(R) -> tuple:
    """Unit quaternion (w, x, y, z) of a rotation matrix (body -> level)."""
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        q = (0.25 * s, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s)
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        q = ((R[2][1] - R[1][2]) / s, 0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s)
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        q = ((R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s)
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        q = ((R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s)
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def gps_flights(ref: Ref, rng: random.Random, n: int) -> list[dict]:
    """NAV GPS histories (sim.gps_nav.NavGps in closed loop with sim.gps_nav.UbxReceiverStub): boot configuration from a
    receiver at 38400 or the factory 9600, one that NAKs CFG-NAV5, one silent at first, a u-blox 8 MON-VER, or no
    configurator (pre-configured); a truth trajectory (pad, then horizontal accelerations up to 3 m/s^2 and yaw motion,
    the vehicle tilted to its acceleration) seen by the IMU (body specific force, attitude, rate) and by the receiver at
    the antenna (lever arm, the heading error the yaw updates must find) with noise, 5 Hz epochs delivered after a
    latency at the UART's byte rate; faults: position and velocity outlier bursts (rejections, resets), poor epochs
    (unusable), outages, a latency beyond the delayed-state buffer, corrupted / duplicated bytes, garbage, bytes at the
    wrong baud.  Records each tick's inputs (float32) and the reference's outputs."""
    gp = ref.d["vt"]["gps"]["gp"]
    lev = tuple(float(x) for x in gp.lever_b_m)
    psi0 = -math.radians(gp.pad_azimuth_deg)
    g0 = gpn.G0 if hasattr(gpn, "G0") else 9.80665
    flights = []
    for fi in range(n):
        var = fi % 7
        stub = gpn.UbxReceiverStub(baud=gpn.GPS_BAUD_DEFAULT if var in (1, 4) else gpn.GPS_BAUD,
                                   hw=gpn.HW_UBLOX8 if var == 5 else gpn.HW_UBLOX6,
                                   nak=(gpn.ID_CFG_NAV5,) if var == 2 else ())
        silent_until = rng.randrange(150, 700) if var == 3 else 0
        configure = var != 6
        nav = gpn.NavGps(gp, configure=configure)
        slot_tx = nl.GpsSlotTx(gp.step_ticks)
        frame = gpn.LocalFrame.at(rng.randrange(-800000000, 800000000), rng.randrange(-1700000000, 1700000000))
        itow0 = rng.randrange(0, 600000000)
        nav_baud = gpn.GPS_BAUD
        fifo: list[tuple[int, int]] = []           # (byte, baud it was sent at)
        sched: dict[int, bytes] = {}               # tick -> epoch bytes that start then
        n_ticks = rng.randrange(3500, 8000)
        t_off = rng.randrange(1300, 2600)
        err = math.radians(rng.uniform(-10.0, 10.0))
        dpsi = psi0 + err                          # the true level -> W rotation (the filter starts at psi0)
        pw, vw, aw = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]
        psib, rpsi = rng.uniform(-math.pi, math.pi), 0.0
        lat_ticks = rng.choice([40, 50, 55, 60, 95])          # the clock learns the delivery delay (leaky minimum)
        wander = [0.0, 0.0]
        bad_p = bad_v = poor = outage = 0
        rows, py = [], []
        for k in range(n_ticks):
            pad = k < t_off
            # truth
            if not pad:
                for i in range(2):
                    aw[i] += (-aw[i] / 2.0) * 0.004 + rng.gauss(0, 1.0) * math.sqrt(0.004)
                    aw[i] = max(-3.0, min(3.0, aw[i]))
                rpsi += -rpsi * 0.004 + rng.gauss(0, 0.3) * math.sqrt(0.004)
                psib += rpsi * 0.004
                for i in range(2):
                    vw[i] += aw[i] * 0.004
                    pw[i] += vw[i] * 0.004
            c, s = math.cos(dpsi), math.sin(dpsi)
            al = (c * aw[0] + s * aw[1], -s * aw[0] + c * aw[1]) if not pad else (0.0, 0.0)   # W -> level
            fz = (al[0], al[1], g0)
            nz = math.sqrt(fz[0] ** 2 + fz[1] ** 2 + fz[2] ** 2)
            zb = (fz[0] / nz, fz[1] / nz, fz[2] / nz)
            xp = (math.cos(psib), math.sin(psib), 0.0)
            d = xp[0] * zb[0] + xp[1] * zb[1]
            xb = [xp[0] - d * zb[0], xp[1] - d * zb[1], -d * zb[2]]
            nx = math.sqrt(sum(u * u for u in xb))
            xb = [u / nx for u in xb]
            yb = (zb[1] * xb[2] - zb[2] * xb[1], zb[2] * xb[0] - zb[0] * xb[2], zb[0] * xb[1] - zb[1] * xb[0])
            R = [[xb[0], yb[0], zb[0]], [xb[1], yb[1], zb[1]], [xb[2], yb[2], zb[2]]]
            q = _quat_of(R)
            fb = [rng.gauss(0, 0.05), rng.gauss(0, 0.05), nz + rng.gauss(0, 0.05)]
            wl = (0.0, 0.0, rpsi)
            wb = [R[0][j] * wl[0] + R[1][j] * wl[1] + R[2][j] * wl[2] for j in range(3)]
            # the receiver: an epoch measured every 50 ticks (5 Hz), its bytes after the latency
            if k % 50 == 0 and k > 0:
                if outage == 0 and rng.random() < 0.01:
                    outage = rng.randrange(10, 25)                      # 2..5 s without epochs
                if bad_p == 0 and not pad and rng.random() < 0.02:
                    bad_p = rng.choice([2, 4, 12])                      # short (rejected) or persistent (reset)
                    jump = (rng.choice([-1, 1]) * rng.uniform(8, 30), rng.choice([-1, 1]) * rng.uniform(8, 30))
                if bad_v == 0 and not pad and rng.random() < 0.02:
                    bad_v = rng.choice([2, 8, 15])
                    vjump = (rng.choice([-1, 1]) * rng.uniform(2, 5), rng.choice([-1, 1]) * rng.uniform(2, 5))
                if poor == 0 and rng.random() < 0.01:
                    poor = rng.randrange(2, 30)
                lw = [sum(R[i][j] * lev[j] for j in range(3)) for i in range(3)]
                wxl = (wb[1] * lev[2] - wb[2] * lev[1], wb[2] * lev[0] - wb[0] * lev[2], wb[0] * lev[1] - wb[1] * lev[0])
                lvw = [sum(R[i][j] * wxl[j] for j in range(3)) for i in range(3)]
                ant = [pw[0] + c * lw[0] - s * lw[1], pw[1] + s * lw[0] + c * lw[1]]
                antv = [vw[0] + c * lvw[0] - s * lvw[1], vw[1] + s * lvw[0] + c * lvw[1]]
                for i in range(2):
                    wander[i] = 0.995 * wander[i] + rng.gauss(0, 0.05)
                mx = ant[0] + wander[0] + rng.gauss(0, 0.3)
                my = ant[1] + wander[1] + rng.gauss(0, 0.3)
                mvx, mvy = antv[0] + rng.gauss(0, 0.05), antv[1] + rng.gauss(0, 0.05)
                if bad_p:
                    bad_p -= 1
                    mx, my = mx + jump[0], my + jump[1]
                if bad_v:
                    bad_v -= 1
                    mvx, mvy = mvx + vjump[0], mvy + vjump[1]
                if outage:
                    outage -= 1
                else:
                    lat, lon = frame.to_llh(mx, my)
                    good = poor == 0
                    if poor:
                        poor -= 1
                    ep = gpn.GpsEpoch(itow_ms=(itow0 + 4 * k) & 0xFFFFFFFF, lat_e7=lat, lon_e7=lon, hmsl_mm=45000,
                                      hacc_mm=rng.randrange(1200, 3000) if good else rng.randrange(6000, 30000),
                                      vacc_mm=3900, vel_n_cms=lp.round_half_away(mvx * 100),
                                      vel_e_cms=lp.round_half_away(-mvy * 100), vel_d_cms=0,
                                      sacc_cms=rng.randrange(20, 60), gps_fix=3 if good or rng.random() < 0.5 else 2,
                                      fix_ok=good or rng.random() < 0.3,
                                      num_sv=rng.randrange(8, 13) if good else rng.randrange(3, 7), pacc_cm=350)
                    by = gpn.epoch_frames(ep, order=rng.choice(["psv", "psv", "vsp", "spv"]))
                    if rng.random() < 0.03:                             # a corrupted byte (bad checksum)
                        b = bytearray(by)
                        b[rng.randrange(len(b))] ^= 1 << rng.randrange(8)
                        by = bytes(b)
                    if rng.random() < 0.02:                             # NMEA text / garbage before it
                        by = b"$GPGGA,000000.00,,,,,0,00,99.99,,,,,,*48\r\n" + by
                    if rng.random() < 0.02:                             # a repeated message
                        by = by + by[:44]
                    late = rng.randrange(70, 120) if rng.random() < (0.08 if var == 4 else 0.01) else 0
                    sched[k + lat_ticks + late + rng.randrange(0, 4)] = by   # late: older than the 16-step buffer
            if k in sched and stub.baud == gpn.GPS_BAUD and (not configure or gpn.ID_CFG_RATE in stub.cfg) and \
                    k >= silent_until:
                fifo += [(b, stub.baud) for b in sched.pop(k)]
            else:
                sched.pop(k, None)
            bpt = max(1, stub.baud // 10 // 250)
            rx = bytearray()
            for b, baud in fifo[:bpt]:
                rx.append(b if baud == nav_baud else rng.randrange(256))   # a byte at the wrong baud: garbage
            del fifo[:bpt]
            a32 = [f32(x) for x in fb]
            q32 = [f32(x) for x in q]
            w32 = [f32(x) for x in wb]
            acts, snap = nav.tick(k, bytes(rx), a32, q32, w32, pad)
            tx_hex = []
            act_baud = 0
            for act in acts:
                if act[0] == "baud":
                    nav_baud = act[1]
                    act_baud = act[1]
                else:
                    tx_hex.append(act[1].hex())
                    if k >= silent_until:
                        ans = stub.receive(act[1], nav_baud)
                        fifo += [(b, nav_baud) for b in ans]          # the stub answers only at the NAV's baud
            if k % gp.step_ticks == gp.step_ticks - 1:
                slot_tx.latch(snap, nav.filter.cycle)
            fl = nav.filter
            cfg = nav.cfg
            py.append(dict(
                k=k, baud=act_baud, tx="".join(tx_hex) or "-",
                parser=":".join(str(nav.parser.stats[x]) for x in gpn.PARSER_STAT_KEYS),
                asm=":".join(str(nav.asm.stats[x]) for x in gpn.ASSEMBLER_STAT_KEYS),
                clock=(0, 0, nav.clock.relearned) if nav.clock.offset is None else (1, nav.clock.offset, nav.clock.relearned),
                cfg=None if cfg is None else (cfg.state, int(cfg.cfg_ok), int(cfg.ver_ok),
                                              0 if cfg.ver is None else cfg.ver.module, cfg.naks, cfg.timeouts),
                init=int(fl.init), pad=int(fl.pad), acq_left=fl.acq_left, reset_count=fl.reset_count & 0xFF,
                cycle=fl.cycle & 0xFFFF, yaw_used=fl.yaw_used,
                stats=":".join(str(fl.stats[x]) for x in ("fused", "rej_p", "rej_v", "pos_resets", "vel_resets",
                                                         "dropped_old", "dropped_stale", "n_epoch_dropped", "unusable")),
                p=tuple(fl.p), v=tuple(fl.v), b=tuple(fl.b), off=tuple(fl.off), c_psi=fl.c_psi, s_psi=fl.s_psi,
                p_psi=fl.p_psi, yaw_corr=fl.yaw_corr, slot=slot_tx.slot(k + 1).hex()))
            rows.append(f"{k}," + ",".join(r32(x) for x in (*a32, *q32, *w32)) + f",{int(pad)}," + (rx.hex() or "-"))
        flights.append(dict(var=var, configure=configure, rows=rows, py=py, lat_ticks=lat_ticks, err_deg=math.degrees(err)))
    return flights


def _slot_close(cs: str, ps: str, latched: dict | None) -> str:
    """'eq' / 'one' / 'diff': 'one' = the same header, payload integers one count apart, and each differing integer's
    reference value (the float the reference latched at the step) within its float bound of a rounding boundary
    (lp.quantize rounds half away from zero: |x| scale = n + 1/2)."""
    if cs == ps:
        return "eq"
    bc, bp = bytes.fromhex(cs), bytes.fromhex(ps)
    j = bc[0] & 7
    if bc[0] != bp[0] or j == 0 or j > 4 or latched is None:     # header (cycle, part) and STATUS: exact
        return "diff"
    ic, ip = struct.unpack("<hh", bc[1:]), struct.unpack("<hh", bp[1:])
    vals, scale, tol = {1: (latched["off"], 100.0, GPS_OFF_TOL), 2: (latched["p"], 100.0, GPS_P_TOL),
                        3: (latched["v"], 100.0, GPS_V_TOL), 4: (latched["cs"], 32767.0, GPS_CS_TOL)}[j]
    for a, b_, x in zip(ic, ip, vals):
        if a == b_:
            continue
        if abs(a - b_) != 1 or abs(abs(abs(x) * scale - math.floor(abs(x) * scale)) - 0.5) > tol * scale:
            return "diff"
    return "one"


def check_gpsnav(ref: Ref, flights: list[dict]) -> dict:
    """8b: sim.gps_nav.NavGps vs core/gps_nav.c gps_nav_tick (test_core --gpsnav-vectors) on the flights of gps_flights:
    every integer output each tick, the float state at each step, the slot of each next frame (bounds above)."""
    fin, fout = os.path.join(BUILD, "gpsnav_in.txt"), os.path.join(BUILD, "gpsnav_out.csv")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        for i, fl in enumerate(flights):
            f.write(f"#G {i} {int(fl['configure'])}\n")
            f.write("\n".join(fl["rows"]) + "\n")
    run_exe(["--gpsnav-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        f.readline()
        lines = f.read().splitlines()
    st = {"flights": len(flights), "ticks": 0, "steps_compared": 0, "mismatch_ticks": 0, "first_mismatches": [],
          "slot_one_count": 0, "max_abs": {k: 0.0 for k in ("p", "v", "b", "off", "cs", "p_psi_rel", "yaw_corr")},
          "totals": {}, "cfg_end": [], "variants": sorted({fl["var"] for fl in flights})}
    i = 0
    tot = [0] * 9
    for fi, fl in enumerate(flights):
        bad_flight = 0
        latched = None
        for o in fl["py"]:
            c = lines[i].split(",")
            i += 1
            st["ticks"] += 1
            bad = []
            if int(c[0]) != o["k"]:
                raise SystemExit(f"gpsnav-vectors: row order at {i}")
            if int(c[2]) != o["baud"]:
                bad.append("baud")
            if c[3] != o["tx"]:
                bad.append("tx")
            if c[5] != o["parser"]:
                bad.append("parser")
            if c[6] != o["asm"]:
                bad.append("asm")
            ch, coff, crl = (int(x) for x in c[7].split(":"))
            if (ch, coff if ch else 0, crl) != (o["clock"][0], gpn._i32(o["clock"][1]) if o["clock"][0] else 0,
                                                o["clock"][2]):
                bad.append("clock")
            if o["cfg"] is not None and tuple(int(x) for x in c[8].split(":")) != o["cfg"]:
                bad.append("cfg")
            j = o["k"] % 5
            if j == 4:
                st["steps_compared"] += 1
                for key, col in (("init", 9), ("pad", 10), ("acq_left", 11), ("reset_count", 12), ("cycle", 13),
                                 ("yaw_used", 14)):
                    if int(c[col]) != o[key]:
                        bad.append(key)
                if c[15] != o["stats"]:
                    bad.append("stats")
                cf = [float(x) for x in c[16:28]]
                for key, vals, pv, tol in (("p", cf[0:2], o["p"], GPS_P_TOL), ("v", cf[2:4], o["v"], GPS_V_TOL),
                                           ("b", cf[4:6], o["b"], GPS_B_TOL), ("off", cf[6:8], o["off"], GPS_OFF_TOL),
                                           ("cs", cf[8:10], (o["c_psi"], o["s_psi"]), GPS_CS_TOL),
                                           ("yaw_corr", cf[11:12], (o["yaw_corr"],), GPS_YAW_TOL)):
                    dm = max(abs(a - b) for a, b in zip(vals, pv))
                    st["max_abs"][key] = max(st["max_abs"][key], dm)
                    if not dm <= tol:
                        bad.append(key)
                dr = abs(cf[10] - o["p_psi"]) / max(o["p_psi"], 1e-30)
                st["max_abs"]["p_psi_rel"] = max(st["max_abs"]["p_psi_rel"], dr)
                if not dr <= GPS_PPSI_REL:
                    bad.append("p_psi")
            if j == 4:                                           # the reference latched these at this step
                latched = dict(off=o["off"], p=o["p"], v=o["v"], cs=(o["c_psi"], o["s_psi"]))
            sc = _slot_close(c[28], o["slot"], latched)
            if sc == "one":
                st["slot_one_count"] += 1
            elif sc == "diff":
                bad.append("slot")
            if bad:
                st["mismatch_ticks"] += 1
                bad_flight += 1
                if len(st["first_mismatches"]) < 12:
                    st["first_mismatches"].append({"flight": fi, "k": o["k"], "fields": bad,
                                                   "c": {k: c[n] for k, n in (("tx", 3), ("parser", 5), ("asm", 6),
                                                                              ("clock", 7), ("cfg", 8), ("stats", 15),
                                                                              ("slot", 28)) if k in bad},
                                                   "py": {k: o[k] for k in ("tx", "parser", "asm", "clock", "cfg", "stats",
                                                                            "slot") if k in bad}})
        last = fl["py"][-1]
        for n_, x in enumerate(last["stats"].split(":")):
            tot[n_] += int(x)
        st["cfg_end"].append({"var": fl["var"], "cfg": last["cfg"], "yaw_used": last["yaw_used"],
                              "err_deg": round(fl["err_deg"], 2), "yaw_corr_deg": round(math.degrees(last["yaw_corr"]), 2),
                              "mismatch_ticks": bad_flight})
    if i != len(lines):
        raise SystemExit(f"gpsnav-vectors: C wrote {len(lines)} rows, expected {i}")
    st["totals"] = dict(zip(("fused", "rej_p", "rej_v", "pos_resets", "vel_resets", "dropped_old", "dropped_stale",
                             "n_epoch_dropped", "unusable"), tot))
    st["coverage"] = {k: st["totals"][k] > 0 for k in ("fused", "rej_p", "rej_v", "pos_resets", "vel_resets",
                                                       "dropped_old", "unusable")}
    st["coverage"]["yaw_updates"] = any(x["yaw_used"] > 0 for x in st["cfg_end"])
    st["coverage"]["cfg_ok"] = any(x["cfg"] is not None and x["cfg"][1] for x in st["cfg_end"])
    st["coverage"]["naks"] = any(x["cfg"] is not None and x["cfg"][4] > 0 for x in st["cfg_end"])
    st["bounds"] = {"p": GPS_P_TOL, "v": GPS_V_TOL, "b": GPS_B_TOL, "off": GPS_OFF_TOL, "cos/sin psi": GPS_CS_TOL,
                    "p_psi_rel": GPS_PPSI_REL, "yaw_corr": GPS_YAW_TOL}
    st["pass"] = st["mismatch_ticks"] == 0 and st["ticks"] > 0 and all(st["coverage"].values())
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


# 8c the v0.8 supervisor rules: REJ_GPS (the ON gate after "aligned"), V11-V14 (the position controller's LAND requests in
# CLIMB / HOLD), V15 (the take-off check -> SAFE) and TELEM v3, with the VTOL random operator of 4b plus random GPS inputs
V08_REASONS = {"V11 gps lost": lp.R_V_GPS_LOST, "V12 fence": lp.R_V_FENCE, "V13 gps implausible": lp.R_V_GPS_IMPLAUSIBLE,
               "V14 authority": lp.R_V_AUTHORITY, "V15 take-off check": lp.R_V_TAKEOFF_CHECK}


def _v08_inp_row(inp: onb.OnboardInputs) -> str:
    """A v0.8 supervisor row (test_core.c run_onb_vectors_mode): the v0.7.1 columns, k_sync (unused: nan), a reserved
    column, cos_tilt, gps_gate_ok, pc_requests, vc_takeoff_fail, the five TELEM v3 inputs, no slot (-1 x 5)."""
    tg = inp.tlm_gps if inp.tlm_gps is not None else (0, 0, lp.HACC_NONE, 0, math.nan)
    return (_vtol_inp_row(inp) + f",nan,0,{_f_text(inp.cos_tilt)},{int(inp.gps_gate_ok)},{int(inp.pc_requests)},"
            f"{int(inp.vc_takeoff_fail)},{int(tg[0])},{int(tg[1])},{int(tg[2])},{int(tg[3])},{_f_text(tg[4])},-1,-1,-1,-1,-1")


class V08RandomOperator(VtolRandomOperator):
    """VtolRandomOperator plus the v0.8 inputs: the ON gate (GATE_OK) on and off, the position controller's requests
    (each bit alone, several, flickering), the take-off check flag, random TELEM v3 fields (dist NaN / huge / small)."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        r = self.r
        self.gate_p = r.choice([0.0, 0.3, 0.9, 1.0])
        self.req_p = r.choice([0.0, 0.0005, 0.003])
        self.req_hold = 0
        self.req = 0
        self.tof_p = r.choice([0.0, 0.0, 0.0005, 0.003])
        self.req_bits = [pcm.REQ_GPS_LOST, pcm.REQ_FENCE, pcm.REQ_IMPLAUSIBLE, pcm.REQ_AUTHORITY,
                         pcm.REQ_FENCE | pcm.REQ_AUTHORITY, 0x0F, 0x10 | pcm.REQ_FENCE]

    script: dict | None = None
    sseq = 10

    def frames(self, t: int, sup: onb.OnboardSupervisor) -> list[onb.UplinkFrame]:
        """A scripted ground (ON, then polls every 50 ticks) when ``script`` is set, else the random one."""
        if self.script is None:
            return super().frames(t, sup)
        cmd = self.script.get(t)
        if cmd is None and t % 50:
            return []
        if cmd is not None:
            self.sseq = (self.sseq + 1) & 0xFF
        return [onb.UplinkFrame(self.sseq, lp.Command(lp.CMD_NONE if cmd is None else cmd, 0, sup.nonce))]

    def sensors(self, t: int, sup: onb.OnboardSupervisor) -> dict:
        d = super().sensors(t, sup)
        r = self.r
        fly = sup.state == onb.State.POWERED and sup.phase in (lp.PHASE_CLIMB, lp.PHASE_HOLD)
        if self.req_hold > 0:
            self.req_hold -= 1
        elif r.random() < (self.req_p * (20.0 if fly else 1.0)):
            self.req = r.choice(self.req_bits)
            self.req_hold = r.randrange(1, 60)
        elif self.req_hold == 0:
            self.req = 0
        dist = r.choice([math.nan, f32(r.uniform(0.0, 40.0)), f32(7000.0), f32(0.04), f32(0.05)])
        d.update(gps_gate_ok=r.random() < self.gate_p, pc_requests=self.req, vc_takeoff_fail=r.random() < self.tof_p,
                 tlm_gps=(r.randrange(256), r.randrange(30), r.choice([0, 25, 254, 255]), r.randrange(5), dist),
                 cos_tilt=None)
        return d


def check_onboard_v08(ref: Ref, n_seq: int, seed: int = 20261022) -> dict:
    """8c: random v0.8 VTOL supervisor sequences (gps_hold, takeoff_check) through check_onboard_vtol's comparison (every
    tick identical incl. the 37 TELEM v3 bytes); REJ_GPS, every V11-V15 reason and an ON with the gate open must occur."""
    rng = random.Random(seed)
    vp = vc_params32(ref, v08=True)
    seqs = []
    for s in range(n_seq):
        ov: dict = {}
        if s % 2:
            ov = dict(spool_ticks=rng.choice([1, 30]), link_disarm_ticks=rng.choice([90, 140]),
                      link_land_ticks=rng.choice([120, 300]), endurance_ticks=rng.choice([800, 1500]),
                      safe_after_landed_ticks=rng.choice([1, 20]))
            ov["hard_cutoff_ticks"] = ov["endurance_ticks"] + 100
        sp = sup_params_vtol(ref, ov, v08=True)
        nd = Nudger(sp)
        op = V08RandomOperator(rng, ref, sp, vp, nd)
        if s % 2 == 0:                                                    # scripted clean flights: the gate open,
            op.clean, op.inject, op.gate_p, op.tof_p = True, 0.0, 1.0, 0.0  # one request kind each (V11 .. V14 in turn)
            op.req_p = 0.0005
            op.script = {rng.randrange(50, 300): lp.CMD_ON}
            op.loss = 0.0
            op.aligned_from = op.arm_closed_from = op.vcell_low_until = 0
            op.tilt_max, op.tilt = 1.0, 0.5                               # the V7 take-off gate passes
            op.req_bits = [(pcm.REQ_GPS_LOST, pcm.REQ_FENCE, pcm.REQ_IMPLAUSIBLE, pcm.REQ_AUTHORITY)[(s // 2) % 4]]
        elif s % 4 == 1:                                                  # the take-off check fires (V15)
            op.clean, op.inject, op.gate_p, op.tof_p, op.req_p = True, 0.0, 1.0, 0.004, 0.0
            op.script = {rng.randrange(50, 300): lp.CMD_ON}
            op.loss = 0.0
            op.aligned_from = op.arm_closed_from = op.vcell_low_until = 0
            op.tilt_max, op.tilt = 1.0, 0.5                               # the V7 take-off gate passes
        reset = rng.random() < 0.05
        nonce_seed = rng.randrange(65536)
        sup = TracingSupervisor(sp, seed=nonce_seed, reset_in_flight=reset)
        rows, py = [], []
        for t in range(rng.choice([3000, 6000, 10000])):
            frames = op.frames(t, sup)
            inp = onb.OnboardInputs(tick=t, frames=frames, **op.sensors(t, sup))
            n0, n1 = len(sup.exec_log), len(sup.trace)
            out = sup.update(inp)
            op.after(inp, out)
            rows.append(_v08_inp_row(inp))
            py.append(_py_row_vtol(sup, out, n0, n1, t))
        seqs.append(dict(kind="v08_random", seed=nonce_seed, reset=reset, overrides=dict(sup_c_overrides(sp), mode=1),
                         rows=rows, py=py, nudged=nd.count))
    st = check_onboard_vtol(ref, seqs)
    st["v08_reasons_seen"] = {k: st["interlocks_seen"].get(lp.REASON_NAMES[v], 0) for k, v in V08_REASONS.items()}
    st["rej_gps_seen"] = st["rejections_seen"].get(lp.REASON_NAMES[lp.REJ_GPS], 0)
    st["telem3_frames"] = sum(1 for sq in seqs for o in sq["py"] if o["telem"] != "-" and
                              len(o["telem"]) == 2 * lp.TELEM3_PAYLOAD_LEN)
    # the v0.7.1 coverage is section 4b's; this section's own: no mismatch, every v0.8 reason, REJ_GPS, TELEM v3 replies
    st["pass"] = (st["mismatch_ticks"] == 0 and all(v > 0 for v in st["v08_reasons_seen"].values())
                  and st["rej_gps_seen"] > 0 and st["telem3_frames"] > 0 and st["ticks"] > 0)
    return st


# 8d the FCU stack with position hold: 3-D point-mass closed loops of sim.onboard.FcuStack (v0.8 build) replayed into the C
# stack (check_stack with the run's PositionParams: the slots in, the published outputs, the tilt builder, the attitude
# tilt and the position state out)
STACK_G_FIELDS = ("mode", "requests", "engaged")


def stack_3d_flights(ref: Ref, rng: random.Random, n: int) -> list[dict]:
    """Closed-loop flights of the Python v0.8 FcuStack: the vertical plant of the 1-D flights flown by the ESC command, a
    horizontal point mass that follows the stack's tilt setpoint through a 0.15 s attitude lag (a_W = g (t_y, -t_x) in
    W, the slots' heading psi between level and W), wind gusts and a steady drift; a NAV model that latches a snapshot
    (p, v of the truth + 0.1 m / 0.05 m/s noise, cm / cm/s, FIX_OK | GATE_OK | HEAD_OK | FIX3D | CFG_OK | INIT | VER_OK)
    at every 5th tick into nlink v3 slots; faults: GPS outages (no FIX_OK, then no slots), a position jump with a NAV
    reset (offset + counter), lost NAV frames (no slot), a push beyond the fence, k_T low (no authority).  The ground
    sends ON, sometimes UP / DOWN, then OFF (LAND)."""
    sp = sup_params_vtol(ref, v08=True)
    vp = vc_params32(ref, v08=True)
    pp = pos_params32(ref_pos_params(ref))
    runs = []
    gfl = gpn.GF_FIX_OK | gpn.GF_GATE_OK | gpn.GF_HEAD_OK | gpn.GF_FIX3D | gpn.GF_CFG_OK | gpn.GF_INIT | gpn.GF_VER_OK
    for s in range(n):
        seed = rng.randrange(65536)
        fault = ("", "outage", "jump", "frames", "fence", "authority", "")[s % 7]
        ppr = pos_params32(ref_pos_params(ref), fence_m=6.0) if fault == "fence" else pp
        fcu = onb.FcuStack(sp, vp, seed=seed, pp=ppr)
        pl = Plant1D(ref, thrust_scale=0.9 if fault == "authority" else 1.0)
        tx = nl.GpsSlotTx(5)
        psi = rng.uniform(-math.pi, math.pi)
        cps, sps = math.cos(psi), math.sin(psi)
        cq, sq = lp.quantize(cps, 32767.0, (-32767, 32767)), lp.quantize(sps, 32767.0, (-32767, 32767))
        p, v, tw = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]
        off, rc = [0, 0], rng.randrange(256)
        wind = (rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3))
        seq, h_off, recs = 10, 0.0, []
        t_on = rng.randrange(100, 400)
        script = {t_on: lp.CMD_ON}
        t = t_on + sp.spool_ticks + rng.randrange(1500, 3000)
        for _ in range(rng.randrange(0, 3)):
            script[t] = rng.choice([lp.CMD_UP, lp.CMD_DOWN])
            t += rng.randrange(400, 1200)
        script[t + rng.randrange(1500, 3000)] = lp.CMD_OFF
        t_fault = t_on + sp.spool_ticks + rng.randrange(1500, 3500)
        n_ticks = t + 6000
        a_tl = 1.0 - math.exp(-0.004 / 0.15)
        gust = [0.0, 0.0]
        for k in range(n_ticks):
            frames = []
            cmd = script.get(k)
            if cmd is not None or k % 50 == 0:
                if cmd is not None:
                    seq = (seq + 1) & 0xFF
                frames = [onb.UplinkFrame(seq, lp.Command(lp.CMD_NONE if cmd is None else cmd, 0, fcu.sup.nonce))]
            if fcu.sup.state in (onb.State.SAFE, onb.State.ARMED):
                h_off = pl.z
            # horizontal plant: the tilt the attitude controller holds (lag), in W; on the ground nothing moves
            tl = fcu.tilt
            tgt = (cps * tl[0] - sps * tl[1], sps * tl[0] + cps * tl[1])      # level -> W
            for i in range(2):
                tw[i] += a_tl * (tgt[i] - tw[i])
            if fault == "fence" and t_fault <= k < t_fault + 700:
                gust = [6.0, -4.0]                                          # beyond the tilt authority: pushed out
            elif rng.random() < 0.002:
                gust = [rng.gauss(0, 1.0), rng.gauss(0, 1.0)]
            else:
                gust = [0.98 * gust[0], 0.98 * gust[1]]
            if not pl.ground:
                aw = (onb.G0 * tw[1] + wind[0] + gust[0], -onb.G0 * tw[0] + wind[1] + gust[1])
                for i in range(2):
                    v[i] += (aw[i] - 0.3 * v[i]) * 0.004
                    p[i] += v[i] * 0.004
            else:
                v = [0.0, 0.0]
            # the NAV's snapshot at its step (tick mod 5 == 4), sent in the next five frames
            slot = tx.slot(k)
            lost_frame = fault == "frames" and t_fault <= k < t_fault + 1500 and rng.random() < 0.2
            outage = fault == "outage" and t_fault <= k < t_fault + 1800
            if k % 5 == 4:
                if fault == "jump" and k == t_fault - (t_fault % 5) + 4:
                    off = [off[0] + 450, off[1] - 300]
                    rc = (rc + 1) & 0xFF
                fl = gfl & ~(gpn.GF_FIX_OK | gpn.GF_GATE_OK) if outage else gfl
                if fault == "fence" and t_fault <= k < t_fault + 700:
                    fl |= gpn.GF_INCONS                                     # physics-review fix: rule (a)'s NAV flag
                px = p[0] + off[0] / 100.0 + rng.gauss(0, 0.1)
                py_ = p[1] + off[1] / 100.0 + rng.gauss(0, 0.1)
                snap = gpn.GpsSnapshot(flags=fl, num_sv=9, hacc_dm=22, reset_count=rc, off_x_cm=off[0], off_y_cm=off[1],
                                       px_cm=gpn.quant_cm(px), py_cm=gpn.quant_cm(py_),
                                       vx_cms=gpn.quant_cm(v[0] + rng.gauss(0, 0.05)),
                                       vy_cms=gpn.quant_cm(v[1] + rng.gauss(0, 0.05)), cos_q15=cq, sin_q15=sq)
                tx.latch(snap, k // 5)
            tilt = math.sqrt(tw[0] ** 2 + tw[1] ** 2) + abs(rng.gauss(0.0, 0.003))
            tilt_deg = f32(math.degrees(tilt))
            cos_t = f32(math.cos(tilt))
            inp = onb.OnboardInputs(
                tick=k, frames=frames, h_m=f32(pl.z - h_off + rng.gauss(0.0, 0.02)), vz_mps=f32(pl.v + rng.gauss(0.0, 0.02)),
                tilt_deg=tilt_deg, gyro_rad_s=(f32(rng.gauss(0, 0.05)), f32(rng.gauss(0, 0.05)), f32(rng.gauss(0, 0.05))),
                v_cell=f32(4.0 - 0.3 * k / 20000), roll_deg=0.0, pitch_deg=0.0, yaw_deg=f32(math.degrees(psi)),
                loop_max_us=f32(2900.0), q_nav=q_from_tilt(tilt_deg, 0.0), cos_tilt=cos_t,
                accel_z_mps2=f32(pl.fz + rng.gauss(0.0, 0.2)),
                gps_slot=None if (lost_frame or (outage and k > t_fault + 600)) else slot)
            recs.append(inp)
            fo = fcu.step(inp)
            pl.step(fo.throttle, 1.0 / sp.loop_hz)
        runs.append(dict(name=f"3d_{s}_{fault or 'nominal'}", seed=seed, inputs=recs, reset=False, sp=sp, vp=vp, pp=ppr,
                         profile="field (the generator's) + GPS"))
    return runs


def _gps_cols(inp: onb.OnboardInputs) -> str:
    """The v0.8 stack-row columns (test_core.c run_onb_vectors_mode): reserved, cos_tilt, the supervisor's GPS inputs
    (ignored by a stack row: the stack fills them), the slot (-1 x 5 = none)."""
    sl = ",".join(str(b) for b in inp.gps_slot) if inp.gps_slot is not None else "-1,-1,-1,-1,-1"
    return f",0,{_f_text(inp.cos_tilt)},0,0,0,0,0,255,0,nan,{sl}"


def stack_g_expected(fcu, fo) -> dict:
    """The reference's values of the C stack row's v0.8 tail (after ',G,')."""
    po = fcu.pos_out()
    ph = fcu.pos
    hold = ph.hold_rf if ph.hold_rf is not None else (0.0, 0.0)
    return dict(att_tilt=tuple(fo.att_tilt), mode=po.mode, requests=po.requests, engaged=int(po.engaged),
                tilt_l=tuple(po.tilt_l), dist=po.dist_m, tilt_next=tuple(fcu.tilt), gate_ok=int(fcu.rx.gate_ok),
                fresh=int(fcu.rx.fresh()), n=ph.n & 0xFFFF, p_engaged=int(ph.engaged), has_hold=int(ph.hold_rf is not None),
                hold_rf=tuple(hold), integ=tuple(ph.integ), dv=tuple(ph.dv), lost=ph.lost, guard_left=ph.guard_left,
                implausible=int(ph.implausible), p_requests=ph.requests, lim_run=ph.lim_run, vexc_run=ph.vexc_run,
                to_run=fo.vc.takeoff_run, to_passed=int(fo.vc.takeoff_passed))


def stack_g_compare(parts: list[str], e: dict, ds: dict) -> list[str]:
    """Compare the C row's v0.8 tail with stack_g_expected; returns the mismatching keys, updates ds (max |d|)."""
    bad = []
    if int(parts[50]) != e["to_run"] or int(parts[51]) != e["to_passed"]:
        bad.append("takeoff")
    g = 54
    if parts[g] != "G":
        return ["G tail missing"]
    c = parts[g + 1:]
    for key, j in (("mode", 0), ("requests", 1), ("engaged", 2), ("gate_ok", 8), ("fresh", 9)):
        if int(c[j]) != e[key]:
            bad.append(key)
    # the position state: the reference updates it at the due tick, the C over its pieces until the commit - equal when
    # no step is in the C's pieces (job 0: the commit tick and after)
    state = int(c[10]) == 0
    if state:
        for key, j in (("p_engaged", 12), ("has_hold", 13), ("lost", 20), ("guard_left", 21), ("implausible", 22),
                       ("p_requests", 23), ("lim_run", 24), ("vexc_run", 25)):
            if int(c[j]) != e[key]:
                bad.append(key)
        if int(c[11]) != e["n"]:
            bad.append("n")
    if int(c[27]) != 0:
        bad.append("overruns")
    fl = lambda j: float(c[j])  # noqa: E731
    for key, vals, pv, tol in (("att_tilt", (float(parts[52]), float(parts[53])), e["att_tilt"], POS_TILT_TOL),
                               ("tilt_l", (fl(3), fl(4)), e["tilt_l"], POS_TILT_TOL),
                               ("tilt_next", (fl(6), fl(7)), e["tilt_next"], POS_TILT_TOL),
                               *((("hold_rf", (fl(14), fl(15)), e["hold_rf"], POS_X_TOL),
                                  ("integ", (fl(16), fl(17)), e["integ"], POS_A_TOL),
                                  ("dv", (fl(18), fl(19)), e["dv"], POS_A_TOL)) if state else ())):
        dm = max(abs(a - b) for a, b in zip(vals, pv))
        ds[key] = max(ds.get(key, 0.0), dm)
        if not dm <= tol:
            bad.append(key)
    if math.isnan(e["dist"]) != math.isnan(fl(5)) or (not math.isnan(e["dist"]) and not abs(fl(5) - e["dist"]) <= POS_X_TOL):
        bad.append("dist")
    return bad


def check_stack_v08(ref: Ref, rng: random.Random, n: int) -> dict:
    """8d: the 3-D GPS flights into both stacks (check_stack, v0.8 tail) and their supervisor inputs into both
    supervisors (check_onboard_vtol: the GPS inputs the Python stack gave its supervisor)."""
    t0 = time.perf_counter()
    runs = stack_3d_flights(ref, rng, n)
    sk, sup = check_stack(ref, runs)
    ov = check_onboard_vtol(ref, sup)
    sk["supervisor_replay"] = {k: ov[k] for k in ("ticks", "mismatch_ticks", "first_mismatches", "interlocks_seen",
                                                   "rejections_seen", "transitions_seen", "phase_changes_seen")}
    sk["supervisor_replay"]["pass"] = ov["mismatch_ticks"] == 0 and ov["ticks"] > 0
    sk["wall_s"] = round(time.perf_counter() - t0, 1)
    sk["pass"] = sk["pass"] and sk["supervisor_replay"]["pass"]
    return sk


# 8e the v0.8 wire formats: TELEM v3 (link), SNAP3 (ilink), nlink v3 and its GPS slot receiver - random encodings, decoder
# streams and slot sequences through both implementations (bytes / fields / counters equal)
_LP_MASK[lp.TYPE_TELEM3] = 8                                       # link_proto.h LP_ACCEPT_TELEM3


def _rand_telem3(rng: random.Random) -> lp.Telemetry:
    t = _rand_telem2(rng)
    return dataclasses.replace(t, gps_flags=rng.randrange(256), num_sv=rng.randrange(256), hacc_dm=rng.randrange(256),
                               pos_mode=rng.randrange(256), dist_dm=rng.choice([lp.DIST_NONE, 0, rng.randrange(65536)]))


def _rand_f32_bits(rng: random.Random) -> int:
    if rng.random() < 0.1:
        return rng.choice([0x7FC00000, 0x7FA00001, 0xFFC12345, 0x7F800000, 0xFF800000, 0x80000000, 0x00000001, 0x007FFFFF])
    return struct.unpack("<I", struct.pack("<f", rng.uniform(-1, 1) * 10 ** rng.uniform(-6, 6)))[0]


def check_proto_v3(ref: Ref, seed: int, n: int) -> dict:
    """8e: link (TELEM3 in streams with every accept mask that has it, TELEM3 encodings), ilink (SNAP3 in streams with
    the ALL8 / VTOL+GPS COM / FCU masks, SNAP3 re-encodings), nlink v3 (frames of random fields incl. NaN payloads, inf,
    -0, denormals and random slots encoded by both; corrupted / foreign / random exchange buffers decoded by both) and
    random slot sequences (lost frames, out-of-order and foreign cycles, empty slots) through both slot receivers."""
    rng = random.Random(seed)
    vid = int(ref.p["flight_computer"]["link"]["vehicle_id"])
    st = {"link_streams": 0, "telem3_frames": 0, "telem3_encodings": 0, "ilink_streams": 0, "snap3_frames": 0,
          "snap3_encodings": 0, "nav3_encodings": 0, "nav3_decodes": 0, "nav3_valid": 0, "slot_sequences": 0,
          "slot_sets": 0, "slot_broken": 0, "mismatches": 0, "first_mismatches": []}

    def run(mode: str, lines: list[str], expect: list[str], tag: str) -> None:
        fin, fout = os.path.join(BUILD, f"p3_{tag}_in.txt"), os.path.join(BUILD, f"p3_{tag}_out.txt")
        with open(fin, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(lines) + "\n")
        run_exe([mode, fin, fout])
        with open(fout, "r", encoding="utf-8") as f:
            got = f.read().splitlines()
        if len(got) != len(expect):
            raise SystemExit(f"{mode} ({tag}): {len(got)} lines, expected {len(expect)}")
        bad = 0
        for k, (a, b) in enumerate(zip(got, expect)):
            if a != b:
                bad += 1
                if len(st["first_mismatches"]) < 8:
                    st["first_mismatches"].append({"tag": tag, "line": k, "input": lines[k][:160], "c": a[:240],
                                                   "py": b[:240]})
        st["mismatches"] += bad
        if bad == 0:
            for path in (fin, fout):
                os.remove(path)

    # link: TELEM3 streams and encodings
    lines, expect = [], []
    for _ in range(n):
        dec_vid = rng.choice([vid, vid, None, (vid + 1) & 0xFF])
        acc = rng.choice([(lp.TYPE_TELEM3,), (lp.TYPE_TELEM2, lp.TYPE_TELEM3), (lp.TYPE_CMD, lp.TYPE_TELEM3),
                          (lp.TYPE_CMD, lp.TYPE_TELEM, lp.TYPE_TELEM2, lp.TYPE_TELEM3), (lp.TYPE_TELEM2,)])
        parts = []
        for _ in range(rng.randrange(1, 10)):
            k = rng.random()
            veh = vid if rng.random() < 0.8 else rng.randrange(256)
            if k < 0.4:
                parts.append(lp.encode_telemetry(veh, rng.randrange(256), _rand_telem3(rng)))
            elif k < 0.5:
                parts.append(lp.encode_telemetry(veh, rng.randrange(256), _rand_telem2(rng)))
            elif k < 0.6:
                parts.append(lp.encode_command(veh, rng.randrange(256), lp.Command(rng.randrange(256), rng.randrange(65536),
                                                                                    rng.randrange(65536))))
            elif k < 0.7:
                pl = bytes(rng.randrange(256) for _ in range(rng.choice([36, 37, 31, rng.randrange(0, 38)])))
                parts.append(lp.encode_frame(veh, rng.choice([lp.TYPE_TELEM3, lp.TYPE_TELEM2, 0x23]), rng.randrange(256), pl))
            elif k < 0.8:
                parts.append(bytes(rng.choice([0xA5, 0x5A, 37, 40, 41, 0x22, rng.randrange(256)])
                                   for _ in range(rng.randrange(1, 50))))
            else:
                base = bytearray(lp.encode_telemetry(veh, rng.randrange(256), _rand_telem3(rng)))
                m = rng.random()
                if m < 0.4:
                    base[rng.randrange(len(base))] ^= 1 << rng.randrange(8)
                elif m < 0.6:
                    base = base[:rng.randrange(1, len(base))]
                elif m < 0.8:
                    base[2] = rng.choice([36, 38, 39, 40, 41, 0xFF])
                else:
                    j = rng.randrange(len(base) + 1)
                    base[j:j] = bytes([rng.choice([0xA5, 0x5A, rng.randrange(256)])])
                parts.append(bytes(base))
        data = b"".join(parts)
        mask = 0
        for t in acc:
            mask |= _LP_MASK[t]
        dec = lp.FrameDecoder(dec_vid, acc)
        frames = dec.feed(data)
        st["telem3_frames"] += sum(1 for fr in frames if fr.type == lp.TYPE_TELEM3)
        lines.append(f"D {-1 if dec_vid is None else dec_vid} {mask} {data.hex()}")
        expect.append("D" + "".join(f" {fr.veh}:{fr.type}:{fr.seq}:{fr.payload.hex()}" for fr in frames) + " S "
                      + " ".join(str(dec.stats[k]) for k in lp.STAT_KEYS) + " 0")
    st["link_streams"] = n
    for _ in range(2 * n):
        t = _rand_telem3(rng)
        veh, seq = rng.randrange(256), rng.randrange(256)
        lines.append(f"V {veh} {seq} " + " ".join(str(getattr(t, k)) for k in bht.TELEM3_ORDER))
        expect.append("V " + lp.encode_telemetry(veh, seq, t).hex())
        st["telem3_encodings"] += 1
    run("--proto-vectors", lines, expect, "link")

    # ilink: SNAP3 streams and re-encodings
    def snap3_payload() -> bytes:                                 # v0.9 RP1: + pw_vaux, pw_flags, spare3
        return (_il_rand_payload(rng, il.TYPE_SNAP) + bytes([rng.randrange(256), rng.randrange(256)]) +
                struct.pack("<I", _il_rand_float_bits(rng)) + bytes([rng.randrange(256) for _ in range(3)]) +
                struct.pack("<I", _il_rand_float_bits(rng)) + bytes([rng.randrange(256) for _ in range(3)]))

    lines, expect = [], []
    for _ in range(n):
        max_out = rng.choice([0, 1, 2, 8])
        accept = rng.choice([il.ACCEPT_ALL8, il.ACCEPT_COM_GPS, il.ACCEPT_COM_GPS, il.ACCEPT_FCU, il.ACCEPT_COM_VTOL])
        parts = []
        for _ in range(rng.randrange(1, 10)):
            k = rng.random()
            if k < 0.45:
                parts.append(il.encode(il.TYPE_SNAP3, snap3_payload()))
            elif k < 0.55:
                typ = rng.choice([il.TYPE_CMD, il.TYPE_SNAP, il.TYPE_STAT, il.TYPE_SNAP2])
                parts.append(il.encode(typ, _il_rand_payload(rng, typ) if typ != il.TYPE_SNAP2 else
                                       _il_rand_payload(rng, il.TYPE_SNAP) + bytes(2) + bytes(4) + bytes(2)))
            elif k < 0.65:
                parts.append(bytes(rng.choice([il.SYNC0, il.SYNC1, il.SNAP3_LEN, il.TYPE_SNAP3, rng.randrange(256)])
                                   for _ in range(rng.randrange(1, 80))))
            else:
                base = bytearray(il.encode(il.TYPE_SNAP3, snap3_payload()))
                m = rng.random()
                if m < 0.4:
                    base[rng.randrange(len(base))] ^= 1 << rng.randrange(8)
                elif m < 0.6:
                    base = base[:rng.randrange(1, len(base))]
                elif m < 0.8:
                    base[rng.choice([2, 3])] = rng.choice([il.SNAP2_LEN, il.TYPE_SNAP2, 0xC5, rng.randrange(256)])
                else:
                    del base[rng.randrange(len(base))]
                parts.append(bytes(base))
        data = b"".join(parts)
        dec, frames, need, cneed = il.decode_trace(data, max_out, accept)
        st["snap3_frames"] += sum(1 for fr in frames if fr.type == il.TYPE_SNAP3)
        lines.append(f"D {max_out} {accept} {data.hex()}")
        expect.append("D" + "".join(f" {fr.type}:{fr.len}:{fr.payload.hex()}" for fr in frames) + " S "
                      + " ".join(str(dec.stats[k]) for k in il.DEC_STAT_KEYS) + f" P {dec.pending}"
                      + f" N {need.hex()} C {cneed.hex()}")
    st["ilink_streams"] = n
    for _ in range(n):
        pl = snap3_payload()
        lines.append(f"E {il.TYPE_SNAP3} {pl.hex()}")
        expect.append("E " + il.encode(il.TYPE_SNAP3, pl).hex())
        st["snap3_encodings"] += 1
    run("--ilink-vectors", lines, expect, "ilink")

    # nlink v3: encodings, decodes, slot sequences
    def nav3_fields() -> tuple:
        fb = [_rand_f32_bits(rng) for _ in range(12)]
        iv = [rng.randrange(-32768, 32768), rng.randrange(-32768, 32768), rng.randrange(-32768, 32768),
              rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(65536), rng.randrange(256),
              rng.randrange(256)]
        sl = bytes(rng.randrange(256) for _ in range(5)) if rng.random() < 0.7 else nl.SLOT_EMPTY
        return rng.randrange(256), fb, iv, sl

    def nav3_frame(seq: int, fb: list, iv: list, sl: bytes) -> bytes:
        """The reference's v3 frame (sim.nlink_proto._frame, TYPE_NAV3) of a payload packed from raw float bit patterns
        (Nav3.pack would pass NaN payloads through a double)."""
        return nl._frame(nl.TYPE_NAV3, seq, struct.pack("<12IhhhBBBHBB5s", *fb, *iv, sl))

    lines, expect = [], []
    for _ in range(n):
        seq, fb, iv, sl = nav3_fields()
        lines.append(f"M {seq} " + " ".join(str(x) for x in fb) + " " + " ".join(str(x) for x in iv) + " " +
                     " ".join(str(b) for b in sl))
        buf = nav3_frame(seq, fb, iv, sl)
        expect.append("M " + bytes(buf).hex())
        st["nav3_encodings"] += 1
    for _ in range(2 * n):
        seq, fb, iv, sl = nav3_fields()
        buf = bytearray(nav3_frame(seq, fb, iv, sl))
        m = rng.random()
        if m < 0.3:
            buf[rng.randrange(len(buf))] ^= 1 << rng.randrange(8)
        elif m < 0.4:
            buf[2] = rng.choice([nl.TYPE_NAV, nl.TYPE_FCU, 0x04])
        elif m < 0.5:
            buf = bytearray(rng.randrange(256) for _ in range(len(buf)))
        elif m < 0.6:
            buf = buf[1:] + bytes([rng.randrange(256)])
        lines.append("X " + bytes(buf).hex())
        if nl.decode_nav3(bytes(buf)) is None:
            expect.append("X 0")
        else:                                                 # the fields as raw patterns (nlink_proto._nav3_fields)
            d = nl._nav3_fields(buf[3], bytes(buf[nl.HEADER_BYTES:nl.HEADER_BYTES + nl.NAV3_LEN]))
            expect.append(f"X 1 {d['seq']} " + " ".join(f"{x:08x}" for x in d["float_bits"]) +
                          f" {d['accel_x_q']} {d['accel_y_q']} {d['h_baro_rel_cm']} {d['imu_fail']} {d['baro_fails']} "
                          f"{d['twi_errors']} {d['loop_us_max']} {d['flags']} {d['reset_cause']} {d['slot_hex']}")
            st["nav3_valid"] += 1
        st["nav3_decodes"] += 1
    for _ in range(n):                                        # slot sequences
        rx = nl.GpsSlotRx()
        tx = nl.GpsSlotTx(5)
        toks, out = [], []
        cyc = rng.randrange(32)
        p_mis, p_bad = rng.choice([(0.0, 0.0), (0.0, 0.02), (0.02, 0.05), (0.15, 0.1)])
        for k in range(rng.randrange(20, 200)):
            sl = tx.slot(k + (rng.choice([1, 2]) if rng.random() < p_mis else 0))   # NAV / FCU tick misalignment
            # the NAV latches after building its frame (tick mod 5 == 4)
            if k % 5 == 4 and rng.random() < 0.95:
                snap = gpn.GpsSnapshot(flags=rng.choice([0x2F, 0x2D, 0x7F, rng.randrange(256)]), num_sv=rng.randrange(256),
                                       hacc_dm=rng.randrange(256), reset_count=rng.randrange(256),
                                       off_x_cm=rng.randrange(-32768, 32768), off_y_cm=rng.randrange(-32768, 32768),
                                       px_cm=rng.randrange(-32768, 32768), py_cm=rng.randrange(-32768, 32768),
                                       vx_cms=rng.randrange(-32768, 32768), vy_cms=rng.randrange(-32768, 32768),
                                       cos_q15=rng.randrange(-32767, 32768), sin_q15=rng.randrange(-32767, 32768))
                cyc = (cyc + rng.choice([1, 1, 1, 2, 31])) & 0xFFFF
                tx.latch(snap, cyc)
            x = rng.random() / max(p_bad, 1e-9)
            if x < 0.6:
                sl = None                                            # no valid frame this tick
            elif x < 0.85:
                sl = bytes(rng.randrange(256) for _ in range(5))     # a foreign slot
            elif x < 1.0:
                sl = nl.SLOT_EMPTY
            toks.append("-" if sl is None else bytes(sl).hex())
            g = rx.feed(sl)
            tok = f"{int(g is not None)}:{int(rx.gate_ok)}:{int(rx.fresh())}"
            if g is not None:
                tok += (f":{g.flags}:{g.num_sv}:{g.hacc_dm}:{g.reset_count}:{round(g.off[0] * 100)}:{round(g.off[1] * 100)}:"
                        f"{round(g.p[0] * 100)}:{round(g.p[1] * 100)}:{round(g.v[0] * 100)}:{round(g.v[1] * 100)}:"
                        f"{round(g.cos_psi * 32767)}:{round(g.sin_psi * 32767)}:{g.cycle}")
            out.append(tok)
        lines.append("W " + " ".join(toks))
        expect.append("W " + " ".join(out) + f" S {rx.sets} {rx.broken}")
        st["slot_sequences"] += 1
        st["slot_sets"] += rx.sets
        st["slot_broken"] += rx.broken
    run("--nlink-vectors", lines, expect, "nlink")
    st["pass"] = (st["mismatches"] == 0 and st["telem3_frames"] > 0 and st["snap3_frames"] > 0 and st["nav3_valid"] > 0
                  and st["slot_sets"] > 0 and st["slot_broken"] > 0)
    return st


def run_v08_sections(ref: Ref, a, report: dict) -> list[str]:
    """Sections 8a-8e (v0.8 GPS position hold; own random seeds); returns the report keys whose 'pass' counts."""
    keys = []
    t0 = time.perf_counter()
    ps = check_pos(ref, pos_random_sequences(ref, random.Random(a.seed + 31), a.pos_seqs))
    ps["wall_s"] = round(time.perf_counter() - t0, 1)
    report["position_ctrl"] = ps
    keys.append("position_ctrl")
    print(f"== position controller (PositionHold vs the background pieces P0..P10): {ps['steps']} steps in "
          f"{ps['sequences']} sequences (parameter variants {len(ps['variants'])}), modes {ps['modes']}, limited "
          f"{ps['limited_steps']}, guard {ps['guard_steps']}, implausible {ps['implausible_steps']}, requests seen "
          f"0x{ps['requests_seen']:02X}; max |d| { {k: float(f'{v:.3g}') for k, v in ps['max_abs'].items()} } (bounds "
          f"{ps['bounds']}); a-limit ties (cut there) {len(ps['knife_edge_cuts'])}; mismatching steps "
          f"{ps['mismatch_steps']} -> {'PASS' if ps['pass'] else 'FAIL'}")
    for m in ps["first_mismatches"][:4]:
        print(f"   mismatch: {m}")
    t0 = time.perf_counter()
    gs = check_gpsnav(ref, gps_flights(ref, random.Random(a.seed + 32), a.gps_flights))
    gs["wall_s"] = round(time.perf_counter() - t0, 1)
    report["gps_nav"] = gs
    keys.append("gps_nav")
    print(f"== NAV GPS side (NavGps: UBX parser, configurator, epoch assembler, iTOW clock, filter): {gs['ticks']} ticks in "
          f"{gs['flights']} flights (variants {gs['variants']}), {gs['steps_compared']} steps compared; totals "
          f"{gs['totals']}; slot payloads one count off at a rounding boundary {gs['slot_one_count']}; max |d| "
          f"{ {k: float(f'{v:.3g}') for k, v in gs['max_abs'].items()} } (bounds {gs['bounds']}); mismatching ticks "
          f"{gs['mismatch_ticks']} -> {'PASS' if gs['pass'] else 'FAIL'}")
    print(f"   per flight (configurator end state, yaw updates, injected heading error vs the estimate): {gs['cfg_end']}")
    for m in gs["first_mismatches"][:4]:
        print(f"   mismatch: {m}")
    t0 = time.perf_counter()
    so = check_onboard_v08(ref, a.v08_sup_seqs, a.seed + 33)
    so["wall_s"] = round(time.perf_counter() - t0, 1)
    report["onboard_v08"] = so
    keys.append("onboard_v08")
    print(f"== v0.8 supervisor rules: {so['ticks']} ticks in {so['sequences']} sequences, {so['telem3_frames']} TELEM v3 "
          f"replies; REJ_GPS {so['rej_gps_seen']}, {so['v08_reasons_seen']}; mismatching ticks {so['mismatch_ticks']} "
          f"-> {'PASS' if so['pass'] else 'FAIL'}")
    for m in so["first_mismatches"][:4]:
        print(f"   mismatch: {m}")
    sk = check_stack_v08(ref, random.Random(a.seed + 34), a.stack_3d)
    report["stack_gps"] = sk
    keys.append("stack_gps")
    print(f"== FCU stack with position hold (3-D point-mass closed loops): {len(sk['runs'])} runs, {sk['ticks']} ticks, "
          f"{sk['ticks_compared']} compared; mismatching ticks {sk['mismatch_ticks']}; controller knife-edges "
          f"{sk['knife_edges']}, a-limit ties {len(sk['pos_knife_edges'])}; TELEM v3 dist one count off at a 0.1 m "
          f"boundary {sk.get('telem_dist_one_count_off', 0)}; max |d| (position tail) "
          f"{ {k: float(f'{v:.3g}') for k, v in sk['max_abs_g'].items()} }; supervisor replay "
          f"{sk['supervisor_replay']['ticks']} ticks, {sk['supervisor_replay']['mismatch_ticks']} mismatching, interlocks "
          f"{sk['supervisor_replay']['interlocks_seen']} -> {'PASS' if sk['pass'] else 'FAIL'}")
    for r in sk["runs"]:
        print(f"   {r['name']}: {r['ticks']} ticks, compared until {r['compared_until']}, mismatching {r['mismatch_ticks']}, "
              f"phases {r['phases']}, reasons {r['reasons']}")
    for m in sk["first_mismatches"][:4]:
        print(f"   mismatch: {m}")
    t0 = time.perf_counter()
    p3 = check_proto_v3(ref, a.seed + 35, a.proto_v3)
    p3["wall_s"] = round(time.perf_counter() - t0, 1)
    report["protocol_v3"] = p3
    keys.append("protocol_v3")
    print(f"== v0.8 wire formats: link {p3['link_streams']} streams ({p3['telem3_frames']} TELEM3 frames), "
          f"{p3['telem3_encodings']} TELEM3 encodings; ilink {p3['ilink_streams']} streams ({p3['snap3_frames']} SNAP3), "
          f"{p3['snap3_encodings']} SNAP3 encodings; nlink v3 {p3['nav3_encodings']} encodings, {p3['nav3_decodes']} decodes "
          f"({p3['nav3_valid']} valid); {p3['slot_sequences']} slot sequences ({p3['slot_sets']} sets, {p3['slot_broken']} "
          f"broken); mismatches {p3['mismatches']} -> {'PASS' if p3['pass'] else 'FAIL'}")
    for m in p3["first_mismatches"][:4]:
        print(f"   mismatch: {m}")
    return keys


# --------------------------------------------------------------------------- #
# v0.8.1 four-chip build (sections 9a-9c): glink + nlink v4 wire formats, the GNS node, the FCU's GNS chain
# --------------------------------------------------------------------------- #
GLINK_LATENCY_DEFAULT = gl.LATENCY_TICKS_DEFAULT


def glink_latency(ref: Ref) -> int:
    g = ref.d["vt"].get("glink") if ref.d.get("vt") else None
    return int(g["latency"]) if g else GLINK_LATENCY_DEFAULT


def gps_c_overrides(gp) -> dict:
    """Every gps_nav_params_t field for a '#N' line (test_core.c gp_override; the generator's mapping)."""
    return gfp.gps_c_fields(gp)


def _ov_tokens(prefix: str, d: dict) -> str:
    return " ".join(f"{prefix}.{k}={r32(v) if isinstance(v, float) else int(v)}" for k, v in d.items())


# 9a the v0.8.1 wire formats: glink STATE / RESULT encodings, whole-buffer decodes, decoder streams with both accept
# masks, the NAV's slot phase (GnsSlotTx) with random latencies / late / missing / foreign results, the FCU's slot
# receiver (GnsSlotRx) on random slot streams, the nlink v4 FCU4 / NAV4 frames, the NAV side GlinkNav on random inputs
def _rand_state_payload(rng: random.Random) -> bytes:
    fb = [_rand_f32_bits(rng) for _ in range(11)]
    return struct.pack("<I3IB4I3IIBBBH", rng.randrange(1 << 32), *fb[0:3], rng.randrange(256), *fb[3:7], *fb[7:10], fb[10],
                       rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(65536))


def _rand_result(rng: random.Random) -> gl.Result:
    eng = rng.random() < 0.8
    i16 = lambda: rng.choice([rng.randrange(-32767, 32768), rng.randrange(-300, 300)])  # noqa: E731
    return gl.Result(tilt_x=i16() if eng else nl.TILT_NONE, tilt_y=i16(), flags=rng.randrange(256),
                     num_sv=rng.randrange(256), hacc_dm=rng.randrange(256), pc=rng.randrange(256), pos_x_cm=i16(),
                     pos_y_cm=i16(), vel_x_cms=i16(), vel_y_cms=i16(), trim_x=i16(), trim_y=i16())


def check_glink_wire(ref: Ref, seed: int, n: int) -> dict:
    """9a: every glink / nlink v4 producer and consumer of sim/glink_proto.py and sim/nlink_proto.py against the C
    (test_core --glink-vectors); the outputs equal (targets / distances within float32: the reference divides in
    float64)."""
    rng = random.Random(seed)
    lines: list[str] = []
    expect: list = []
    st = {"encodings": 0, "decodes": 0, "decodes_valid": 0, "streams": 0, "stream_frames": 0, "slot_tx_seqs": 0,
          "slot_tx_latched": 0, "slot_tx_missed": 0, "slot_rx_seqs": 0, "slot_rx_sets": 0, "slot_rx_broken": 0,
          "fcu4": 0, "v4_decodes": 0, "nav_ticks": 0, "nav_states": 0, "mismatches": 0, "first_mismatches": []}
    reasons = {None: 0, "length": 1, "sync": 2, "type": 3, "crc": 4}
    for _ in range(n):                                          # encodings
        if rng.random() < 0.5:
            pl, typ = _rand_state_payload(rng), gl.TYPE_STATE
        else:
            pl, typ = _rand_result(rng).pack(), gl.TYPE_RESULT
        seq = rng.randrange(256)
        lines.append(f"E {typ} {seq} {pl.hex()}")
        expect.append(("E", "E " + gl._frame(typ, seq, pl).hex()))
        st["encodings"] += 1
    for _ in range(2 * n):                                      # whole-buffer decodes
        base = bytearray(gl._frame(gl.TYPE_STATE, rng.randrange(256), _rand_state_payload(rng)) if rng.random() < 0.5 else
                         gl.encode_result(rng.randrange(256), _rand_result(rng)))
        m = rng.random()
        if m < 0.3:
            base[rng.randrange(len(base))] ^= 1 << rng.randrange(8)
        elif m < 0.4:
            base[2] = rng.choice([0, 3, 0xFF, gl.TYPE_STATE, gl.TYPE_RESULT])
        elif m < 0.5:
            base = base[:rng.randrange(0, len(base))]
        elif m < 0.55:
            base = bytearray(rng.randrange(256) for _ in range(rng.randrange(0, 70)))
        elif m < 0.6:
            base += bytes([rng.randrange(256)])
        b = bytes(base)
        rr = gl.reject_reason(b)
        d = gl.decode_frame(b)
        lines.append(f"X {b.hex() or '-'}")
        expect.append(("X", f"X {reasons[rr]} {int(d is not None)} {d[0] if d else 0} {d[1] if d else 0}"))
        st["decodes"] += 1
        st["decodes_valid"] += int(d is not None)
    for _ in range(n):                                          # decoder streams
        acc = rng.choice([(gl.TYPE_STATE,), (gl.TYPE_RESULT,), (gl.TYPE_STATE, gl.TYPE_RESULT)])
        parts = []
        for _ in range(rng.randrange(1, 8)):
            k = rng.random()
            if k < 0.35:
                parts.append(gl._frame(gl.TYPE_STATE, rng.randrange(256), _rand_state_payload(rng)))
            elif k < 0.6:
                parts.append(gl.encode_result(rng.randrange(256), _rand_result(rng)))
            elif k < 0.75:
                parts.append(bytes(rng.choice([0xA5, 0x5A, 1, 2, rng.randrange(256)]) for _ in range(rng.randrange(1, 40))))
            else:
                bb = bytearray(gl.encode_result(rng.randrange(256), _rand_result(rng)) if rng.random() < 0.5 else
                               gl._frame(gl.TYPE_STATE, rng.randrange(256), _rand_state_payload(rng)))
                mm = rng.random()
                if mm < 0.4:
                    bb[rng.randrange(len(bb))] ^= 1 << rng.randrange(8)
                elif mm < 0.7:
                    del bb[rng.randrange(len(bb))]
                else:
                    bb = bb[:rng.randrange(1, len(bb))]
                parts.append(bytes(bb))
        data = b"".join(parts)
        dec = gl.GlinkDecoder(acc)
        fr = dec.feed(data)
        mask = (1 if gl.TYPE_STATE in acc else 0) | (2 if gl.TYPE_RESULT in acc else 0)
        lines.append(f"D {mask} {data.hex()}")
        expect.append(("D", "D" + "".join(f" {t}:{s}:{p.hex()}" for t, s, p in fr) + " S " +
                       " ".join(str(dec.stats[k]) for k in gl.DEC_STAT_KEYS)))
        st["streams"] += 1
        st["stream_frames"] += len(fr)
    for _ in range(n // 2):                                     # the NAV's slot phase
        L = rng.randrange(2, 17)
        tx = nl.GnsSlotTx(5, L)
        ticks = rng.randrange(20, 160)
        toks, out = [], []
        p_late, p_miss, p_wrong = rng.choice([(0.0, 0.0, 0.0), (0.1, 0.1, 0.0), (0.3, 0.2, 0.1)])
        pend: dict[int, list] = {}
        for m in range(ticks // 5 + 2):                         # cycle m's RESULT arrives some ticks after its sample
            if rng.random() < p_miss:
                continue
            k_s = 5 * m + 4
            k_rx = k_s + rng.choice([1, 1, 2, 2, 3]) + (rng.randrange(1, 20) if rng.random() < p_late else 0)
            sq = (m & 0xFF) if rng.random() >= p_wrong else rng.randrange(256)
            pend.setdefault(k_rx, []).append((sq, _rand_result(rng).pack()))
        for k in range(ticks):
            ev = pend.get(k, [])
            for sq, pl in ev:
                tx.result(sq, pl, k)
            toks.append(";".join(f"{sq}:{pl.hex()}" for sq, pl in ev) or "-")
            out.append(tx.slot(k).hex())
        lines.append(f"T {L} " + " ".join(toks))
        expect.append(("T", "T " + " ".join(out) + f" S {tx.latched} {tx.missed}"))
        st["slot_tx_seqs"] += 1
        st["slot_tx_latched"] += tx.latched
        st["slot_tx_missed"] += tx.missed
    for _ in range(n // 2):                                     # the FCU's slot receiver
        L = rng.randrange(2, 8)
        tx = nl.GnsSlotTx(5, L)
        rx = nl.GnsSlotRx(5)
        land = rng.choice([2, 4, 20])
        ticks = rng.randrange(30, 300)
        p_lost, p_bad, p_gap = rng.choice([(0.0, 0.0, 0.0), (0.05, 0.02, 0.0), (0.2, 0.05, 0.02), (0.0, 0.0, 0.05)])
        gap = 0
        toks, out = [], []
        for k in range(ticks):
            if k % 5 == 0 and k >= 5:
                if gap == 0 and rng.random() < p_gap:
                    gap = rng.randrange(10, 80)                 # the GNS silent for a while (fallback, GPS_LOST)
                if gap == 0:
                    tx.result(((k - 1) // 5) & 0xFF, _rand_result(rng).pack(), k)
            if gap:
                gap -= 1
            sl = tx.slot(k)
            x = rng.random()
            if x < p_lost:
                sl = None
            elif x < p_lost + p_bad:
                sl = bytes(rng.randrange(256) for _ in range(5))
            g = rx.feed(sl)
            eng, tgt = rx.target()
            tok = (int(g is not None), int(eng), tgt[0], tgt[1], int(rx.fallback()), rx.requests(land), int(rx.gate_ok),
                   int(rx.fresh()))
            if g is not None:
                pcb = g.mode | (nl.PC_HOLD if g.hold else 0) | (g.requests << nl.PC_REQ_SHIFT)
                tok += (g.flags, g.num_sv, g.hacc_dm, pcb, round(g.pos[0] * 100), round(g.pos[1] * 100),
                        round(g.vel[0] * 100), round(g.vel[1] * 100), round(g.trim[0] * 10000), round(g.trim[1] * 10000),
                        g.cycle, g.dist_m)
            toks.append("-" if sl is None else sl.hex())
            out.append(tok)
        lines.append(f"W {land} " + " ".join(toks))
        expect.append(("W", out, rx.sets, rx.broken))
        st["slot_rx_seqs"] += 1
        st["slot_rx_sets"] += rx.sets
        st["slot_rx_broken"] += rx.broken
    for _ in range(n // 2):                                     # FCU4 encodings, v4 / v3 / v0.7.1 decodes
        f = nl.Fcu4(rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(256),
                    rng.randrange(256), rng.randrange(65536))
        lines.append(f"F {f.seq} {f.state} {f.fflags} {f.params_tag} {f.thr_q} {f.pflags} {f.kt_q}")
        expect.append(("F", "F " + nl.encode_fcu4(f).hex()))
        st["fcu4"] += 1
        kind = rng.random()
        if kind < 0.3:
            buf = bytearray(nl.encode_fcu4(f))
        elif kind < 0.6:
            buf = bytearray(nl._frame(nl.TYPE_NAV4, rng.randrange(256),
                                      struct.pack("<12IhhhBBBHBB5s", *[_rand_f32_bits(rng) for _ in range(12)],
                                                  *[rng.randrange(-32768, 32768) for _ in range(3)],
                                                  *[rng.randrange(256) for _ in range(3)], rng.randrange(65536),
                                                  rng.randrange(256), rng.randrange(256),
                                                  bytes(rng.randrange(256) for _ in range(5)))))
        elif kind < 0.8:
            buf = bytearray(nl.encode_fcu(nl.Fcu(rng.randrange(256), rng.randrange(256), rng.randrange(256),
                                                 rng.randrange(256), rng.randrange(256))))
        else:
            buf = bytearray(rng.randrange(256) for _ in range(nl.XFER_BYTES))
        if rng.random() < 0.3:
            buf[rng.randrange(nl.XFER_BYTES)] ^= 1 << rng.randrange(8)
        b = bytes(buf)
        n4, f4, n3, f1 = nl.decode_nav4(b), nl.decode_fcu4(b), nl.decode_nav3(b), nl.decode_fcu(b)
        reenc = int(n4 is not None and nl._frame(nl.TYPE_NAV4, b[3], b[4:4 + nl.NAV3_LEN]) == b[:4 + nl.NAV3_LEN + 2])
        y = f"Y {int(n4 is not None)} {int(f4 is not None)} {int(n3 is not None)} {int(f1 is not None)} {reenc}"
        if f4 is not None:
            y += f" {f4.seq} {f4.state} {f4.fflags} {f4.params_tag} {f4.thr_q} {f4.pflags} {f4.kt_q}"
        lines.append(f"Y {b.hex()}")
        expect.append(("Y", y))
        st["v4_decodes"] += 1
    for _ in range(max(2, n // 100)):                           # the NAV side (GlinkNav) on random inputs
        L = rng.randrange(2, 17)
        lines.append(f"#N {L}")
        nav = gl.GlinkNav(5, L)
        k = rng.randrange(0, 1 << 20)
        fcu = None
        for _ in range(rng.randrange(50, 400)):
            k = (k + (1 if rng.random() < 0.97 else rng.randrange(2, 9))) & 0xFFFFFFFF
            a = [_rand_f32_bits(rng) if rng.random() < 0.05 else struct.unpack("<I", struct.pack("<f", rng.gauss(0, 3)))[0]
                 for _ in range(3)]
            q = [struct.unpack("<I", struct.pack("<f", rng.uniform(-1, 1)))[0] for _ in range(4)]
            w = [struct.unpack("<I", struct.pack("<f", rng.gauss(0, 0.3)))[0] for _ in range(3)]
            hb = struct.unpack("<I", struct.pack("<f", rng.uniform(-1, 9)))[0]
            pad = rng.random() < 0.2
            valid = rng.random() < 0.9
            if valid and (fcu is None or rng.random() < 0.3):
                fcu = nl.Fcu4(0, rng.randrange(6), 0, 0, 0, rng.randrange(256), rng.randrange(65536))
            imu, est = rng.random() < 0.95, rng.random() < 0.95
            fv = [struct.unpack("<f", struct.pack("<I", x))[0] for x in a + q + w + [hb]]
            fr = nav.tick(k, fv[0:3], fv[3:7], fv[7:10], fv[10], pad, fcu, valid and fcu is not None, imu, est)
            lines.append(f"n {k} " + " ".join(str(x) for x in a + q + w + [hb]) +
                         f" {int(pad)} {int(fcu is not None)} {int(valid and fcu is not None)} "
                         f"{fcu.state if fcu else 0} {fcu.pflags if fcu else 0} {fcu.kt_q if fcu else 0} {int(imu)} {int(est)}")
            expect.append(("n", "n " + (fr.hex() if fr is not None else "-")))
            st["nav_ticks"] += 1
            st["nav_states"] += int(fr is not None)
    fin, fout = os.path.join(BUILD, "glink_in.txt"), os.path.join(BUILD, "glink_out.txt")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    run_exe(["--glink-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        got = f.read().splitlines()
    if len(got) != len(expect):
        raise SystemExit(f"glink-vectors: {len(got)} lines, expected {len(expect)}")
    st["target_max_abs"] = 0.0
    st["dist_max_abs"] = 0.0
    for li, (c, e) in enumerate(zip(got, expect)):
        ok = True
        wdet = None
        if e[0] == "W":
            toks = c.split()
            if toks[0] != "W" or toks[-3] != "S" or int(toks[-2]) != e[2] or int(toks[-1]) != e[3] or \
                    len(toks) - 4 != len(e[1]):
                ok = False
            else:
                for ct, pt in zip(toks[1:-3], e[1]):
                    cv = ct.split(":")
                    ints = [int(cv[0]), int(cv[1]), int(cv[4]), int(cv[5]), int(cv[6]), int(cv[7])]
                    if ints != [pt[0], pt[1], pt[4], pt[5], pt[6], pt[7]]:
                        ok = False
                        wdet = (ct, pt)
                        break
                    # the target: the C's q x 1e-4f vs the reference's q / 1e4 (float64): <= 1 ulp of float32 (|q| up to 3.3 rad
                    # in these random sets)
                    dt0 = max(abs(float(cv[2]) - pt[2]), abs(float(cv[3]) - pt[3]))
                    st["target_max_abs"] = max(st["target_max_abs"], dt0)
                    if not dt0 <= 2.5e-7 * max(1.0, abs(pt[2]), abs(pt[3])):
                        ok = False
                        wdet = ("target", ct, pt[:4])
                        break
                    if pt[0]:
                        if [int(x) for x in cv[8:19]] != list(pt[8:19]):
                            ok = False
                            wdet = ("set", ct, pt)
                            break
                        cd, pd = float(cv[19]), pt[19]          # float32 hypot of cm x 0.01: 1 ulp of up to 463 m
                        if math.isnan(cd) != math.isnan(pd) or (not math.isnan(pd) and
                                                                not abs(cd - pd) <= POS_X_TOL + 2.4e-7 * pd):
                            ok = False
                            wdet = ("dist", ct, pd)
                            break
                        if not math.isnan(pd):
                            st["dist_max_abs"] = max(st["dist_max_abs"], abs(cd - pd))
        else:
            ok = c == e[1]
        if not ok:
            st["mismatches"] += 1
            if len(st["first_mismatches"]) < 8:
                st["first_mismatches"].append({"line": li, "kind": e[0], "input": lines[li][:160], "c": c[:240],
                                               "py": str(e[1])[:240], "detail": str(wdet)[:300]})
    st["pass"] = (st["mismatches"] == 0 and st["decodes_valid"] > 0 and st["stream_frames"] > 0 and st["slot_tx_latched"] > 0
                  and st["slot_tx_missed"] > 0 and st["slot_rx_sets"] > 0 and st["slot_rx_broken"] > 0 and st["nav_states"] > 0)
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


# 9b the GNS node: float32 C vs float64 reference (sim.glink_proto.GnsNode) on the same UBX bytes and glink STATE frames.
# Integer paths (the UBX parser / configurator / assembler / iTOW clock as in 8b, the gap / restart / IMU / clock-slip
# rules, the decoder counters, the position controller's discrete state) equal; floats within the 8a / 8b bounds; the
# RESULT bytes equal or one count (tilt / trim 1e-4 rad, pos / vel cm) where the float32 value lies at a rounding
# boundary within its bound.
GNS_TILT_TOL = POS_TILT_TOL


def _result_close(cs: str, ps: str, po, fp: tuple, fv: tuple) -> str:
    """'eq' / 'one' / 'diff' for two RESULT frames (hex): 'one' = every integer equal except tilt / trim / pos / vel
    components one count apart whose reference value lies within its float bound of a rounding boundary."""
    if cs == ps:
        return "eq"
    bc, bp = bytes.fromhex(cs), bytes.fromhex(ps)
    if len(bc) != len(bp) or bc[:4] != bp[:4]:
        return "diff"
    rc, rp = gl.Result.unpack(bc[4:-2]), gl.Result.unpack(bp[4:-2])
    if (rc.flags, rc.num_sv, rc.hacc_dm, rc.pc) != (rp.flags, rp.num_sv, rp.hacc_dm, rp.pc) or \
            (rc.tilt_x == nl.TILT_NONE) != (rp.tilt_x == nl.TILT_NONE):
        return "diff"
    hold = po.hold
    ref = {"tilt_x": (po.tilt_l[0], 1e4, GNS_TILT_TOL), "tilt_y": (po.tilt_l[1], 1e4, GNS_TILT_TOL),
           "trim_x": (po.trim_l[0], 1e4, GNS_TILT_TOL), "trim_y": (po.trim_l[1], 1e4, GNS_TILT_TOL),
           "pos_x_cm": (fp[0] - (hold[0] if hold else 0.0), 100.0, GPS_P_TOL),
           "pos_y_cm": (fp[1] - (hold[1] if hold else 0.0), 100.0, GPS_P_TOL),
           "vel_x_cms": (fv[0], 100.0, GPS_V_TOL), "vel_y_cms": (fv[1], 100.0, GPS_V_TOL)}
    for k, (x, sc, tol) in ref.items():
        a, b = getattr(rc, k), getattr(rp, k)
        if a == b:
            continue
        if abs(a - b) != 1 or abs(abs(abs(x) * sc - math.floor(abs(x) * sc)) - 0.5) > tol * sc:
            return "diff"
    return "one"


def gns_flights(ref: Ref, rng: random.Random, n: int) -> list[dict]:
    """GNS histories (sim.glink_proto.GnsNode with sim.gps_nav.UbxReceiverStub as in 8b): boot configuration variants;
    a NAV (sim.glink_proto.GlinkNav) summing a truth trajectory's body force (pad, a climb to ~3 m, a hover with
    horizontal accelerations and yaw motion, a landing through the flare band and the touchdown signature) and sending
    STATE frames with the FCU fields of a phase script (pflags SAT_HI / LOW_ALT / GROUND / LIFTED, k_T); 5 Hz UBX epochs
    after a latency at the UART's byte rate, with position / velocity outliers, poor epochs and outages; glink faults:
    lost STATE frames (gaps), corrupted frames, garbage, IMU-bad frames, a NAV reset (its tick clock restarts: the
    timebase restart), a cut longer than 10 s, the GNS's own 4 ms clock jumping (clock slips).  Records each line's
    reference outputs."""
    gp = ref.d["vt"]["gps"]["gp"]
    lev = tuple(float(x) for x in gp.lever_b_m)
    psi0 = -math.radians(gp.pad_azimuth_deg)
    g0 = onb.G0
    flights = []
    for fi in range(n):
        var = fi % 8
        ppv = {}
        if var == 3:
            ppv = dict(touchdown_policy="fade")
        elif var == 5:
            ppv = dict(gps_timeout_s=0.2, gps_land_after_s=0.4, fence_m=4.0)
        pp = pos_params32(ref_pos_params(ref), **ppv)
        configure = var not in (4, 6)
        stub = gpn.UbxReceiverStub(baud=gpn.GPS_BAUD_DEFAULT if var in (1,) else gpn.GPS_BAUD,
                                   hw=gpn.HW_UBLOX8 if var == 2 else gpn.HW_UBLOX6,
                                   nak=(gpn.ID_CFG_NAV5,) if var == 2 else ())
        node = gl.GnsNode(gp, pp, configure=configure)
        pre = {}                                                   # the filter's p / v as the RESULT carried them

        def _fuse_hook(k_, _f=node.filter, _orig=node.filter.gns_fuse, _pre=pre):
            _pre["p"], _pre["v"] = tuple(_f.p), tuple(_f.v)
            return _orig(k_)

        node.filter.gns_fuse = _fuse_hook
        nav = gl.GlinkNav(5, glink_latency(ref))
        frame = gpn.LocalFrame.at(rng.randrange(-800000000, 800000000), rng.randrange(-1700000000, 1700000000))
        itow0 = rng.randrange(0, 600000000)
        gns_baud = gpn.GPS_BAUD
        fifo: list[tuple[int, int]] = []
        sched: dict[int, bytes] = {}
        n_ticks = rng.randrange(4000, 9000)
        t_off = rng.randrange(1300, 2200)                          # lift-off
        t_land = n_ticks - rng.randrange(900, 1400)
        err = math.radians(rng.uniform(-10.0, 10.0))
        dpsi = psi0 + err
        pw, vw, aw = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]
        psib, rpsi = rng.uniform(-math.pi, math.pi), 0.0
        lat_ticks = rng.choice([40, 50, 55, 60])
        wander = [0.0, 0.0]
        bad_p = bad_v = poor = outage = 0
        k_nav = rng.randrange(0, 3000)                            # the NAV's tick counter (its own clock)
        cut_left = 0
        reset_at = rng.randrange(t_off + 500, t_land - 500) if var == 6 else -1
        long_cut_at = rng.randrange(t_off + 300, t_land - 3000) if var == 6 and t_land - t_off > 4000 else -1
        h = 0.0
        kt = 1.0
        lines, py = [], []
        st_prev: bytes | None = None
        for k in range(n_ticks):
            pad = k < t_off
            land = k >= t_land
            # truth (horizontal) - in flight only
            if not pad:
                for i in range(2):
                    aw[i] += (-aw[i] / 2.0) * 0.004 + rng.gauss(0, 1.0) * math.sqrt(0.004)
                    aw[i] = max(-3.0, min(3.0, aw[i]))
                rpsi += -rpsi * 0.004 + rng.gauss(0, 0.3) * math.sqrt(0.004)
                psib += rpsi * 0.004
                for i in range(2):
                    vw[i] += aw[i] * 0.004
                    pw[i] += vw[i] * 0.004
            # vertical: climb 1.5 m/s to 3 m, hold, land at 0.5 m/s
            if pad:
                h = 0.0
            elif land:
                h = max(h - 0.002, 0.0)
            else:
                h = min(h + 0.006, 3.0)
            c, s = math.cos(dpsi), math.sin(dpsi)
            al = (c * aw[0] + s * aw[1], -s * aw[0] + c * aw[1]) if not pad else (0.0, 0.0)
            fz = (al[0], al[1], g0)
            nz = math.sqrt(fz[0] ** 2 + fz[1] ** 2 + fz[2] ** 2)
            zb = (fz[0] / nz, fz[1] / nz, fz[2] / nz)
            xp = (math.cos(psib), math.sin(psib), 0.0)
            d = xp[0] * zb[0] + xp[1] * zb[1]
            xb = [xp[0] - d * zb[0], xp[1] - d * zb[1], -d * zb[2]]
            nx = math.sqrt(sum(u * u for u in xb))
            xb = [u / nx for u in xb]
            yb = (zb[1] * xb[2] - zb[2] * xb[1], zb[2] * xb[0] - zb[0] * xb[2], zb[0] * xb[1] - zb[1] * xb[0])
            R = [[xb[0], yb[0], zb[0]], [xb[1], yb[1], zb[1]], [xb[2], yb[2], zb[2]]]
            q = _quat_of(R)
            fb = [rng.gauss(0, 0.05), rng.gauss(0, 0.05), nz + rng.gauss(0, 0.05)]
            wl = (0.0, 0.0, rpsi)
            wb = [R[0][j] * wl[0] + R[1][j] * wl[1] + R[2][j] * wl[2] for j in range(3)]
            # the receiver: 5 Hz epochs after the latency (8b's model)
            if k % 50 == 0 and k > 0:
                if outage == 0 and rng.random() < 0.01:
                    outage = rng.randrange(10, 25)
                if bad_p == 0 and not pad and rng.random() < 0.02:
                    bad_p = rng.choice([2, 4, 12])
                    jump = (rng.choice([-1, 1]) * rng.uniform(8, 30), rng.choice([-1, 1]) * rng.uniform(8, 30))
                if bad_v == 0 and not pad and rng.random() < 0.02:
                    bad_v = rng.choice([2, 8, 15])
                    vjump = (rng.choice([-1, 1]) * rng.uniform(2, 5), rng.choice([-1, 1]) * rng.uniform(2, 5))
                if poor == 0 and rng.random() < 0.01:
                    poor = rng.randrange(2, 30)
                lw = [sum(R[i][j] * lev[j] for j in range(3)) for i in range(3)]
                wxl = (wb[1] * lev[2] - wb[2] * lev[1], wb[2] * lev[0] - wb[0] * lev[2], wb[0] * lev[1] - wb[1] * lev[0])
                lvw = [sum(R[i][j] * wxl[j] for j in range(3)) for i in range(3)]
                ant = [pw[0] + c * lw[0] - s * lw[1], pw[1] + s * lw[0] + c * lw[1]]
                antv = [vw[0] + c * lvw[0] - s * lvw[1], vw[1] + s * lvw[0] + c * lvw[1]]
                for i in range(2):
                    wander[i] = 0.995 * wander[i] + rng.gauss(0, 0.05)
                mx = ant[0] + wander[0] + rng.gauss(0, 0.3)
                my = ant[1] + wander[1] + rng.gauss(0, 0.3)
                mvx, mvy = antv[0] + rng.gauss(0, 0.05), antv[1] + rng.gauss(0, 0.05)
                if bad_p:
                    bad_p -= 1
                    mx, my = mx + jump[0], my + jump[1]
                if bad_v:
                    bad_v -= 1
                    mvx, mvy = mvx + vjump[0], mvy + vjump[1]
                if outage:
                    outage -= 1
                else:
                    lat, lon = frame.to_llh(mx, my)
                    good = poor == 0
                    if poor:
                        poor -= 1
                    ep = gpn.GpsEpoch(itow_ms=(itow0 + 4 * k) & 0xFFFFFFFF, lat_e7=lat, lon_e7=lon, hmsl_mm=45000,
                                      hacc_mm=rng.randrange(1200, 3000) if good else rng.randrange(6000, 30000),
                                      vacc_mm=3900, vel_n_cms=lp.round_half_away(mvx * 100),
                                      vel_e_cms=lp.round_half_away(-mvy * 100), vel_d_cms=0,
                                      sacc_cms=rng.randrange(20, 60), gps_fix=3 if good or rng.random() < 0.5 else 2,
                                      fix_ok=good or rng.random() < 0.3,
                                      num_sv=rng.randrange(8, 13) if good else rng.randrange(3, 7), pacc_cm=350)
                    by = gpn.epoch_frames(ep, order=rng.choice(["psv", "psv", "vsp", "spv"]))
                    if rng.random() < 0.03:
                        b2 = bytearray(by)
                        b2[rng.randrange(len(b2))] ^= 1 << rng.randrange(8)
                        by = bytes(b2)
                    if rng.random() < 0.02:
                        by = b"$GPGGA,000000.00,,,,,0,00,99.99,,,,,,*48\r\n" + by
                    sched[k + lat_ticks + rng.randrange(0, 4)] = by
            if k in sched and stub.baud == gpn.GPS_BAUD and (not configure or gpn.ID_CFG_RATE in stub.cfg):
                fifo += [(b, stub.baud) for b in sched.pop(k)]
            else:
                sched.pop(k, None)
            bpt = max(1, stub.baud // 10 // 250)
            rx = bytearray()
            for b, baud in fifo[:bpt]:
                rx.append(b if baud == gns_baud else rng.randrange(256))
            del fifo[:bpt]
            # the GNS: its 4 ms poll (sometimes its clock jumps: a slip at the next STATE), then the STATE of the NAV's
            # previous tick
            n_poll = 1 + (3 if (var == 7 and rng.random() < 0.002) else 0)
            for _ in range(n_poll):
                kp = 0 if node.nt is None else node.nt + 1
                acts = node.poll(kp, bytes(rx))
                tx_hex, act_baud = [], 0
                for act in acts:
                    if act[0] == "baud":
                        gns_baud = act[1]
                        act_baud = act[1]
                    else:
                        tx_hex.append(act[1].hex())
                        ans = stub.receive(act[1], gns_baud)
                        fifo += [(b, gns_baud) for b in ans]
                lines.append(f"P {kp} {rx.hex() or '-'}")
                py.append(("P", f"P {act_baud} {''.join(tx_hex) or '-'}"))
                rx = bytearray()
            if st_prev is not None:
                data = st_prev
                if var in (5, 7) and rng.random() < 0.02:
                    b2 = bytearray(data)
                    b2[rng.randrange(len(b2))] ^= 1 << rng.randrange(8)   # a bit error: CRC
                    data = bytes(b2)
                if var == 7 and rng.random() < 0.01:
                    data = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 30))) + data
                res = node.rx_bytes(data)
                lines.append(f"S {data.hex()}")
                if res:
                    po, flt = node.pos.out, node.filter
                    ph = node.pos
                    hold = ph.hold_rf if ph.hold_rf is not None else (0.0, 0.0)
                    py.append(("S", dict(
                        res=res[0].hex(), po=po, flt_p=pre["p"], flt_v=pre["v"],
                        ints=[int(flt.init), int(flt.pad), flt.acq_left, flt.reset_count & 0xFF, flt.cycle & 0xFFFF,
                              flt.yaw_used & 0xFFFF] + [flt.stats[x] & 0xFFFF for x in (
                                  "fused", "rej_p", "rej_v", "pos_resets", "vel_resets", "dropped_old", "dropped_stale",
                                  "n_epoch_dropped", "unusable")] + [flt.stats.get("timebase_restarts", 0) & 0xFFFF],
                        floats=[*flt.p, *flt.v, *flt.b, *flt.off, flt.c_psi, flt.s_psi, flt.p_psi, flt.yaw_corr],
                        clock=(0, 0, node.clock.relearned) if node.clock.offset is None else
                        (1, gpn._i32(node.clock.offset), node.clock.relearned),
                        pos_ints=[ph.n & 0xFFFF, int(ph.engaged), int(ph.hold_rf is not None), po.mode, po.requests,
                                  min(ph.lost, 0xFFFF), min(ph.guard_left, 255), int(ph.implausible), min(ph.lim_run, 0xFFFF),
                                  min(ph.vexc_run, 0xFFFF)],
                        pos_floats=[*po.tilt_l, *po.trim_l, *ph.integ, *hold],
                        node=[node.stats[x] & 0xFFFF for x in ("states", "hold_steps", "gaps", "restarts", "imu_bad",
                                                               "clock_slips", "misaligned")] + [node.cycles & 0xFFFF,
                                                                                                node.nt & 0xFFFFFFFF],
                        dec=[node.dec.stats[x] for x in gl.DEC_STAT_KEYS], a_lim2=po.a_lim2,
                        na2=po.a_cmd[0] ** 2 + po.a_cmd[1] ** 2, limited=po.limited)))
                else:
                    py.append(("S", dict(res=None, dec=[node.dec.stats[x] for x in gl.DEC_STAT_KEYS])))
                st_prev = None
            # the NAV's tick (after the GNS: its STATE reaches the GNS in the next tick)
            if k == reset_at:                                   # a NAV reset: its tick clock restarts
                nav = gl.GlinkNav(5, glink_latency(ref))
                k_nav = rng.randrange(0, 50)
            phase = (lp.PHASE_NONE if pad else lp.PHASE_LAND if land else
                     lp.PHASE_CLIMB if h < 3.0 else lp.PHASE_HOLD)
            low_alt = land and h <= 1.0
            ground = low_alt and h <= 0.0 and rng.random() < 0.8
            if rng.random() < 0.01:
                kt = f32(min(1.4, max(0.7, kt + rng.gauss(0, 0.05))))
            pf = nl.fcu4_pflags(phase, not pad, (not pad) and rng.random() < 0.04, low_alt, ground,
                                (not pad) and h > 0.4)
            f4 = nl.Fcu4(0, lp.STATE_SAFE if pad else lp.STATE_POWERED, 0, 0, 0, pf, nl.kt_quant(kt))
            imu_ok = not (var == 7 and rng.random() < 0.01)
            a32 = [f32(x) for x in fb]
            q32 = [f32(x) for x in q]
            w32 = [f32(x) for x in wb]
            fr = nav.tick(k_nav, a32, q32, w32, f32(h), pad, f4, rng.random() < 0.97, imu_ok, True)
            k_nav = (k_nav + 1) & 0xFFFFFFFF
            if fr is not None:
                lost = (var in (5, 7) and rng.random() < 0.05) or cut_left > 0
                if k == long_cut_at:
                    cut_left = 2600                             # > 10 s without a STATE: the timebase restarts
                if not lost:
                    st_prev = fr
            if cut_left:
                cut_left -= 1
        flights.append(dict(var=var, configure=configure, pp=pp, lines=lines, py=py))
    return flights


def check_gns(ref: Ref, flights: list[dict]) -> dict:
    """9b: sim.glink_proto.GnsNode vs core/gns_node.c (test_core --gns-vectors) on the flights of gns_flights."""
    gp = ref.d["vt"]["gps"]["gp"]
    fin, fout = os.path.join(BUILD, "gns_in.txt"), os.path.join(BUILD, "gns_out.txt")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        for i, fl in enumerate(flights):
            f.write(f"#N {i} {int(fl['configure'])} {_ov_tokens('gp', gps_c_overrides(gp))} "
                    f"{_ov_tokens('pp', pos_c_overrides(fl['pp']))}\n")
            f.write("\n".join(fl["lines"]) + "\n")
    run_exe(["--gns-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()
    st = {"flights": len(flights), "lines": 0, "cycles": 0, "mismatch_lines": 0, "first_mismatches": [],
          "result_one_count": 0, "a_limit_ties_cut": [], "max_abs": {k: 0.0 for k in ("p", "v", "b", "off", "cs", "p_psi_rel",
                                                                                      "yaw_corr", "tilt", "trim", "integ",
                                                                                      "hold")},
          "node_totals": {}, "variants": sorted({fl["var"] for fl in flights}), "modes": {}, "requests_seen": 0}
    i = 0
    tot = {k: 0 for k in ("states", "hold_steps", "gaps", "restarts", "imu_bad", "clock_slips", "misaligned")}
    for fi, fl in enumerate(flights):
        cut = False
        last_node = None
        for kind, e in fl["py"]:
            c = lines[i]
            i += 1
            st["lines"] += 1
            if cut:
                continue
            bad = []
            if kind == "P":
                if c != e:
                    bad.append("poll")
            else:
                tk = c.split()
                if e["res"] is None:
                    if tk[1] != "-" or [int(x) for x in tk[3:8]] != e["dec"]:
                        bad.append("no-result/dec")
                else:
                    st["cycles"] += 1
                    if tk[1] == "-" or "|" in tk:
                        bad.append("result count")
                    else:
                        po = e["po"]
                        rc = _result_close(tk[1], e["res"], po, e["flt_p"], e["flt_v"])
                        if rc == "one":
                            st["result_one_count"] += 1
                        elif rc == "diff":
                            bad.append("result")
                        ints = [int(x) for x in tk[2:18]]
                        if ints != e["ints"]:
                            bad.append("filter ints")
                        cf = [float(x) for x in tk[18:30]]
                        pe = e["floats"]
                        for key, cv, pv, tol in (("p", cf[0:2], pe[0:2], GPS_P_TOL), ("v", cf[2:4], pe[2:4], GPS_V_TOL),
                                                 ("b", cf[4:6], pe[4:6], GPS_B_TOL), ("off", cf[6:8], pe[6:8], GPS_OFF_TOL),
                                                 ("cs", cf[8:10], pe[8:10], GPS_CS_TOL),
                                                 ("yaw_corr", cf[11:12], pe[11:12], GPS_YAW_TOL)):
                            dm = max(abs(a - b) for a, b in zip(cv, pv))
                            st["max_abs"][key] = max(st["max_abs"][key], dm)
                            if not dm <= tol:
                                bad.append(key)
                        dr = abs(cf[10] - pe[10]) / max(pe[10], 1e-30)
                        st["max_abs"]["p_psi_rel"] = max(st["max_abs"]["p_psi_rel"], dr)
                        if not dr <= GPS_PPSI_REL:
                            bad.append("p_psi")
                        ch, coff, crl = int(tk[30]), int(tk[31]), int(tk[32])
                        if (ch, coff if ch else 0, crl) != e["clock"]:
                            bad.append("clock")
                        pints = [int(x) for x in tk[33:43]]
                        if pints != e["pos_ints"]:
                            bad.append("pos ints")
                        pf_ = [float(x) for x in tk[43:51]]
                        for key, cv, pv, tol in (("tilt", pf_[0:2], e["pos_floats"][0:2], GNS_TILT_TOL),
                                                 ("trim", pf_[2:4], e["pos_floats"][2:4], GNS_TILT_TOL),
                                                 ("integ", pf_[4:6], e["pos_floats"][4:6], POS_A_TOL),
                                                 ("hold", pf_[6:8], e["pos_floats"][6:8], GPS_P_TOL)):
                            dm = max(abs(a - b) for a, b in zip(cv, pv))
                            st["max_abs"][key] = max(st["max_abs"][key], dm)
                            if not dm <= tol:
                                bad.append(key)
                        node = [int(x) for x in tk[51:60]]
                        if node != e["node"]:
                            bad.append("node")
                        last_node = node
                        if [int(x) for x in tk[61:66]] != e["dec"]:
                            bad.append("dec")
                        nm = pcm.MODE_NAMES[po.mode]
                        st["modes"][nm] = st["modes"].get(nm, 0) + 1
                        st["requests_seen"] |= po.requests
                        if bad and set(bad) <= {"pos ints", "tilt", "trim", "integ", "result"} and \
                                abs(e["na2"] - e["a_lim2"]) <= POS_AMB_REL * e["a_lim2"] + POS_A2_ABS_TOL:
                            # the a-limit tie of 8a (|a| == a_lim by design while the vertical loop is saturated high)
                            st["a_limit_ties_cut"].append({"flight": fi, "line": i, "fields": bad})
                            cut = True
                            continue
            if bad:
                st["mismatch_lines"] += 1
                if len(st["first_mismatches"]) < 10:
                    st["first_mismatches"].append({"flight": fi, "var": fl["var"], "line": i, "fields": bad, "c": c[:300],
                                                   "py": (e if kind == "P" else e.get("res"))})
        if last_node is not None:
            for n_, key in enumerate(tot):
                tot[key] += last_node[n_]
    if i != len(lines):
        raise SystemExit(f"gns-vectors: C wrote {len(lines)} lines, expected {i}")
    st["node_totals"] = tot
    st["coverage"] = {"gaps": tot["gaps"] > 0, "restarts": tot["restarts"] > 0, "imu_bad": tot["imu_bad"] > 0,
                      "clock_slips": tot["clock_slips"] > 0, "every_mode": len(st["modes"]) >= 4}
    st["bounds"] = {"p": GPS_P_TOL, "v": GPS_V_TOL, "b": GPS_B_TOL, "off": GPS_OFF_TOL, "cos/sin psi": GPS_CS_TOL,
                    "p_psi_rel": GPS_PPSI_REL, "yaw_corr": GPS_YAW_TOL, "tilt / trim": GNS_TILT_TOL, "integ": POS_A_TOL}
    st["pass"] = (st["mismatch_lines"] == 0 and st["cycles"] > 0 and all(st["coverage"].values()) and
                  len(st["a_limit_ties_cut"]) <= max(1, len(flights) // 8))
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


# 9c the FCU's GNS chain (onb_stack ONB_CHAIN_GNS vs sim.onboard.FcuStack(chain="gns")): 3-D point-mass closed loops of the
# Python four-chip chain (GlinkNav -> GnsNode -> nlink v4 slots -> FcuStack) replayed into the C stack (the slot of each
# NAV frame in, the supervisor / vertical controller outputs, the GNS receiver's state, the FCU4 fields and the builder
# out) and their supervisor inputs into both supervisors
def stack_n_expected(fcu, fo) -> dict:
    rx = fcu.rx
    eng, tgt = rx.target()
    tl = rx.tlm(bool(fcu.sup_out is not None and fcu.sup_out.motor_on))
    return dict(att_tilt=tuple(fo.att_tilt), gate_ok=int(rx.gate_ok), fresh=int(rx.fresh()), fallback=int(rx.fallback()),
                engaged=int(eng), target=tuple(tgt), tilt_next=tuple(fcu.tilt), requests=rx.requests(fcu.pp.land_steps),
                pflags=fo.fcu4[0], kt_q=fo.fcu4[1], tlm=tl, sets=rx.sets, broken=rx.broken, k_t=fo.vc.k_thrust)


def stack_n_compare(parts: list[str], e: dict, ds: dict, st: dict) -> list[str]:
    """The C row's v0.8.1 tail (after ',N,') vs stack_n_expected; kt_q may be one count off where the reference's k_T x
    16384 lies at a rounding boundary within the controller's k_T bound (counted)."""
    bad = []
    g = 54
    if parts[g] != "N":
        return ["N tail missing"]
    c = parts[g + 1:]
    for key, j in (("gate_ok", 0), ("fresh", 1), ("fallback", 2), ("engaged", 3), ("requests", 8), ("pflags", 9)):
        if int(c[j]) != e[key]:
            bad.append(key)
    if int(c[10]) != e["kt_q"]:
        kq = e["k_t"] * nl.KT_SCALE
        if abs(int(c[10]) - e["kt_q"]) == 1 and abs(abs(kq - math.floor(kq)) - 0.5) <= VC_KT_TOL * nl.KT_SCALE:
            st["kt_q_one_count_off"] = st.get("kt_q_one_count_off", 0) + 1
        else:
            bad.append("kt_q")
    tl = e["tlm"]
    if [int(c[11]), int(c[12]), int(c[13]), int(c[14])] != [tl[0], tl[1], tl[2], tl[3]]:
        bad.append("tlm")
    cd = float(c[15])
    if math.isnan(cd) != math.isnan(tl[4]) or (not math.isnan(tl[4]) and not abs(cd - tl[4]) <= POS_X_TOL):
        bad.append("tlm.dist")
    if int(c[16]) != e["sets"] & 0xFFFF or int(c[17]) != e["broken"] & 0xFFFF:
        bad.append("sets/broken")
    for key, vals, pv, tol in (("att_tilt", (float(parts[52]), float(parts[53])), e["att_tilt"], POS_TILT_TOL),
                               ("target", (float(c[4]), float(c[5])), e["target"], 2e-7),
                               ("tilt_next", (float(c[6]), float(c[7])), e["tilt_next"], POS_TILT_TOL)):
        dm = max(abs(a - b) for a, b in zip(vals, pv))
        ds[key] = max(ds.get(key, 0.0), dm)
        if not dm <= tol:
            bad.append(key)
    return bad


def _q_of_tilt(t) -> tuple:
    """Unit quaternion of a horizontal rotation vector t (level frame)."""
    th = math.sqrt(t[0] * t[0] + t[1] * t[1])
    if th < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    k = math.sin(th / 2.0) / th
    return (math.cos(th / 2.0), k * t[0], k * t[1], 0.0)


def _rot_q(q, v) -> tuple:
    """R(q) v (body -> level)."""
    w, x, y, z = q
    r = ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
         (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
         (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
    return tuple(r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2] for i in range(3))


def stack_gns_flights(ref: Ref, rng: random.Random, n: int) -> list[dict]:
    """Closed loops of the Python four-chip chain: the vertical plant of the 1-D flights, a horizontal point mass that
    follows the FCU's tilt setpoint through a 0.15 s attitude lag (a_W = g (t_y, -t_x), the NAV level frame = W turned by
    the pad heading), wind and gusts; the NAV (GlinkNav: the body force, the attitude, h, the FCU4 frame of the FCU's
    previous tick) -> the GNS (GnsNode, configure off; 5 Hz UBX epochs of the truth + 0.1 m / 0.05 m/s noise after
    0.22 s) -> the RESULT back in the next tick -> the NAV4 slot of tick k + L -> FcuStack(chain='gns').  Faults: a glink
    cut (the FCU's own GNS-loss request, V11), a GPS outage (the GNS's request, V11), lost NAV frames (broken sets), a
    push beyond a 6 m fence (V12), k_T low (V14), a steady wind through the landing (the keep-trim touchdown).  The
    ground sends ON, sometimes UP / DOWN, then OFF (LAND)."""
    sp = sup_params_vtol(ref, v08=True)
    vp = vc_params32(ref, v08=True)
    gp = ref.d["vt"]["gps"]["gp"]
    lev = tuple(float(x) for x in gp.lever_b_m) if gp.lever_comp else (0.0, 0.0, 0.0)
    L = glink_latency(ref)
    runs = []
    for s in range(n):
        seed = rng.randrange(65536)
        fault = ("", "glink_cut", "outage", "frames", "fence", "authority", "wind_land", "")[s % 8]
        ppr = pos_params32(ref_pos_params(ref), fence_m=6.0) if fault == "fence" else pos_params32(ref_pos_params(ref))
        fcu = onb.FcuStack(sp, vp, seed=seed, pp=ppr, chain="gns")
        node = gl.GnsNode(gp, ppr, configure=False)
        gnav = gl.GlinkNav(5, L)
        pl = Plant1D(ref, thrust_scale=0.9 if fault == "authority" else 1.0)
        frame = gpn.LocalFrame.at(375665432, 1269780123)
        itow0 = rng.randrange(0, 600000000)
        psi_t = -math.radians(gp.pad_azimuth_deg) + math.radians(rng.uniform(-3.0, 3.0))   # level -> W (truth)
        cps, sps = math.cos(psi_t), math.sin(psi_t)
        p, v, tw = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]
        wind = ((rng.uniform(0.8, 1.2), rng.uniform(-0.4, 0.4)) if fault == "wind_land" else
                (rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3)))
        seq, h_off, recs = 10, 0.0, []
        t_on = rng.randrange(1500, 1900)                        # after the GNS's ON gate (5 s of good epochs)
        script = {t_on: lp.CMD_ON}
        t = t_on + sp.spool_ticks + rng.randrange(1500, 3000)
        for _ in range(rng.randrange(0, 3)):
            script[t] = rng.choice([lp.CMD_UP, lp.CMD_DOWN])
            t += rng.randrange(400, 1200)
        script[t + rng.randrange(1500, 3000)] = lp.CMD_OFF
        t_fault = t_on + sp.spool_ticks + rng.randrange(1500, 3500)
        n_ticks = t + 6000
        a_tl = 1.0 - math.exp(-0.004 / 0.15)
        gust = [0.0, 0.0]
        pend: dict[int, bytes] = {}
        st_prev = None
        f4_last = None
        q_att = (1.0, 0.0, 0.0, 0.0)
        for k in range(n_ticks):
            # the GNS (its poll, the epochs; then the STATE of the NAV's previous tick -> the RESULT to the NAV now)
            cut = fault == "glink_cut" and t_fault <= k < t_fault + 2500
            outage = fault == "outage" and t_fault <= k < t_fault + 2000
            if k % 50 == 0 and k > 0 and not outage:
                rl = _rot_q(q_att, lev)                         # the antenna: CG + R l (level), turned into W
                ant = (p[0] + cps * rl[0] - sps * rl[1], p[1] + sps * rl[0] + cps * rl[1])
                lat, lon = frame.to_llh(ant[0] + rng.gauss(0, 0.1), ant[1] + rng.gauss(0, 0.1))
                ep = gpn.GpsEpoch(itow_ms=(itow0 + 4 * k) & 0xFFFFFFFF, lat_e7=lat, lon_e7=lon, hmsl_mm=45000,
                                  hacc_mm=1800, vacc_mm=3900, vel_n_cms=lp.round_half_away((v[0] + rng.gauss(0, 0.05)) * 100),
                                  vel_e_cms=lp.round_half_away(-(v[1] + rng.gauss(0, 0.05)) * 100), vel_d_cms=0,
                                  sacc_cms=30, gps_fix=3, fix_ok=True, num_sv=10, pacc_cm=350)
                pend[k + 55] = gpn.epoch_frames(ep)
            node.poll(k, pend.pop(k, b""))
            if st_prev is not None and not cut:
                for fr_ in node.rx_bytes(st_prev):
                    gnav.rx_bytes(fr_, k)
            st_prev = None
            # the NAV frame of this tick (its GNS slot) -> the FCU
            frames = []
            cmd = script.get(k)
            if cmd is not None or k % 50 == 0:
                if cmd is not None:
                    seq = (seq + 1) & 0xFF
                frames = [onb.UplinkFrame(seq, lp.Command(lp.CMD_NONE if cmd is None else cmd, 0, fcu.sup.nonce))]
            if fcu.sup.state in (onb.State.SAFE, onb.State.ARMED):
                h_off = pl.z
            slot = gnav.slot(k)
            lost_frame = fault == "frames" and t_fault <= k < t_fault + 1500 and rng.random() < 0.2
            tilt = math.sqrt(tw[0] ** 2 + tw[1] ** 2)
            tilt_deg = f32(math.degrees(tilt))
            cos_t = f32(math.cos(tilt))
            inp = onb.OnboardInputs(
                tick=k, frames=frames, h_m=f32(pl.z - h_off + rng.gauss(0.0, 0.02)), vz_mps=f32(pl.v + rng.gauss(0.0, 0.02)),
                tilt_deg=tilt_deg, gyro_rad_s=(f32(rng.gauss(0, 0.05)), f32(rng.gauss(0, 0.05)), f32(rng.gauss(0, 0.05))),
                v_cell=f32(4.0 - 0.3 * k / 20000), roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, loop_max_us=f32(2900.0),
                q_nav=q_from_tilt(tilt_deg, 0.0), cos_tilt=cos_t, accel_z_mps2=f32(pl.fz + rng.gauss(0.0, 0.2)),
                gps_slot=None if lost_frame else slot)
            recs.append(inp)
            fo = fcu.step(inp)
            pl.step(fo.throttle, 1.0 / sp.loop_hz)
            # the plant: the attitude follows the builder's tilt (the level frame), the point mass in W
            tl = fcu.tilt
            for i in range(2):
                tw[i] += a_tl * (tl[i] - tw[i])
            if fault == "fence" and t_fault <= k < t_fault + 700:
                gust = [6.0, -4.0]
            elif rng.random() < 0.002:
                gust = [rng.gauss(0, 1.0), rng.gauss(0, 1.0)]
            else:
                gust = [0.98 * gust[0], 0.98 * gust[1]]
            q_att = _q_of_tilt(tw)                              # body -> level: the tilt vector (no yaw)
            ez = _rot_q(q_att, (0.0, 0.0, 1.0))
            al = (pl.T / pl.m * ez[0], pl.T / pl.m * ez[1])     # the thrust's horizontal acceleration, level frame
            aw = (cps * al[0] - sps * al[1], sps * al[0] + cps * al[1])
            if not pl.ground:
                for i in range(2):
                    v[i] += (aw[i] + wind[i] + gust[i] - 0.3 * v[i]) * 0.004
                    p[i] += v[i] * 0.004
            else:
                v = [0.0, 0.0]
            # the NAV's correction part: body force (level-frame acceleration rotated into the tilted body), q, h
            q = q_att
            a_body = (0.0, 0.0, onb.G0 if pl.ground else pl.T / pl.m)
            on_pad = f4_last is None or f4_last.state in (lp.STATE_SAFE, lp.STATE_ARMED)
            fr = gnav.tick(k, [f32(x + rng.gauss(0, 0.05)) for x in a_body], [f32(x) for x in q], [f32(0.0)] * 3,
                           f32(pl.z - h_off), on_pad, f4_last, f4_last is not None)
            if fr is not None and not cut:
                st_prev = fr
            f4_last = nl.Fcu4(0, int(fo.sup.state), 0, 0, nl.fcu_throttle_q(fo.throttle), fo.fcu4[0], fo.fcu4[1])
        runs.append(dict(name=f"gns_{s}_{fault or 'nominal'}", seed=seed, inputs=recs, reset=False, sp=sp, vp=vp, pp=ppr,
                         chain="gns", profile="field (the generator's) + GPS, four chips"))
    return runs


def check_stack_gns(ref: Ref, rng: random.Random, n: int) -> dict:
    """9c: the GNS-chain flights into both stacks (check_stack, v0.8.1 tail) and their supervisor inputs into both
    supervisors."""
    t0 = time.perf_counter()
    runs = stack_gns_flights(ref, rng, n)
    sk, sup = check_stack(ref, runs)
    ov = check_onboard_vtol(ref, sup)
    sk["supervisor_replay"] = {k: ov[k] for k in ("ticks", "mismatch_ticks", "first_mismatches", "interlocks_seen",
                                                   "rejections_seen", "transitions_seen", "phase_changes_seen")}
    sk["supervisor_replay"]["pass"] = ov["mismatch_ticks"] == 0 and ov["ticks"] > 0
    reasons = set()
    for r in sk["runs"]:
        reasons |= set(r["reasons"])
    sk["reasons_seen"] = sorted(reasons)
    want = {lp.REASON_NAMES[lp.R_V_GPS_LOST], lp.REASON_NAMES[lp.R_V_FENCE], lp.REASON_NAMES[lp.R_V_AUTHORITY]}
    sk["coverage"] = {"v11_v12_v14": want <= reasons}
    sk["wall_s"] = round(time.perf_counter() - t0, 1)
    sk["pass"] = sk["pass"] and sk["supervisor_replay"]["pass"] and all(sk["coverage"].values())
    return sk


def run_v081_sections(ref: Ref, a, report: dict) -> list[str]:
    """Sections 9a-9c (v0.8.1 four-chip build; own random seeds); returns the report keys whose 'pass' counts."""
    keys = []
    t0 = time.perf_counter()
    gw = check_glink_wire(ref, a.seed + 41, a.glink_wire)
    gw["wall_s"] = round(time.perf_counter() - t0, 1)
    report["glink_wire"] = gw
    keys.append("glink_wire")
    print(f"== v0.8.1 wire formats: {gw['encodings']} glink encodings, {gw['decodes']} whole-buffer decodes "
          f"({gw['decodes_valid']} valid), {gw['streams']} decoder streams ({gw['stream_frames']} frames), "
          f"{gw['slot_tx_seqs']} NAV slot-phase sequences (latched {gw['slot_tx_latched']}, missed {gw['slot_tx_missed']}), "
          f"{gw['slot_rx_seqs']} FCU slot-receiver sequences ({gw['slot_rx_sets']} sets, {gw['slot_rx_broken']} broken), "
          f"{gw['fcu4']} FCU4 encodings + {gw['v4_decodes']} v4 / v3 / v0.7.1 decodes, {gw['nav_ticks']} GlinkNav ticks "
          f"({gw['nav_states']} STATE frames); max |d target| {gw['target_max_abs']:.2g} rad, |d dist| "
          f"{gw['dist_max_abs']:.2g} m; mismatches {gw['mismatches']} -> {'PASS' if gw['pass'] else 'FAIL'}")
    for m in gw["first_mismatches"][:4]:
        print(f"   mismatch: {m}")
    t0 = time.perf_counter()
    gs = check_gns(ref, gns_flights(ref, random.Random(a.seed + 42), a.gns_flights))
    gs["wall_s"] = round(time.perf_counter() - t0, 1)
    report["gns_node"] = gs
    keys.append("gns_node")
    print(f"== GNS node (UBX, configurator, iTOW clock, the filter's gns_step / gns_fuse, the position controller, the gap / "
          f"restart / IMU / clock-slip rules): {gs['lines']} lines in {gs['flights']} flights (variants {gs['variants']}), "
          f"{gs['cycles']} cycles; node totals {gs['node_totals']}; modes {gs['modes']}, requests seen "
          f"0x{gs['requests_seen']:02X}; RESULT one count off at a rounding boundary {gs['result_one_count']}; a-limit ties "
          f"(cut there) {len(gs['a_limit_ties_cut'])}; max |d| { {k: float(f'{v:.3g}') for k, v in gs['max_abs'].items()} } "
          f"(bounds {gs['bounds']}); mismatching lines {gs['mismatch_lines']} -> {'PASS' if gs['pass'] else 'FAIL'}")
    for m in gs["first_mismatches"][:4]:
        print(f"   mismatch: {m}")
    sk = check_stack_gns(ref, random.Random(a.seed + 43), a.stack_gns)
    report["stack_gns"] = sk
    keys.append("stack_gns")
    print(f"== FCU stack, GNS chain (Python four-chip closed loops replayed): {len(sk['runs'])} runs, {sk['ticks']} ticks, "
          f"{sk['ticks_compared']} compared; mismatching ticks {sk['mismatch_ticks']}; controller knife-edges "
          f"{sk['knife_edges']}; kt_q one count off {sk.get('kt_q_one_count_off', 0)}; TELEM v3 dist one count off "
          f"{sk.get('telem_dist_one_count_off', 0)}; max |d| (GNS tail) "
          f"{ {k: float(f'{v:.3g}') for k, v in sk['max_abs_g'].items()} }; reasons {sk['reasons_seen']}; supervisor replay "
          f"{sk['supervisor_replay']['ticks']} ticks, {sk['supervisor_replay']['mismatch_ticks']} mismatching -> "
          f"{'PASS' if sk['pass'] else 'FAIL'}")
    for r in sk["runs"]:
        print(f"   {r['name']}: {r['ticks']} ticks, compared until {r['compared_until']}, mismatching {r['mismatch_ticks']}, "
              f"phases {r['phases']}, reasons {r['reasons']}")
    for m in sk["first_mismatches"][:4]:
        print(f"   mismatch: {m}")
    return keys


# --------------------------------------------------------------------------- #
# ---- v0.9 (control / power / fix rounds): section 10 ------------------------------------------------------------- #
# 10a vertical controller v0.9 (two-slope ceiling, thrust search, take-off learning + lift-off latch, reference hold,
#     approach, LAND rebase, touchdown release, the W_SAG ceiling) - the --vc-vectors comparison of section 4c on take-off
#     / landing scripts that exercise them, with the v0.9 state columns (airborne, to_passed, ramp_k, fz_lpf / td_bias);
# 10b power monitor (--pm-vectors) exact against sim.onboard.PowerMonitor, both builds, every condition and the tail;
# 10c power rules in the supervisor (--onb-vectors with the power columns): W_ARM, V7, UP refused, W_SAG / W_LOST /
#     W_BUF / W_D2 / I8 on the buffer, LAND vs SAFE on the pad (fix round F1), W_LOST during LAND, the TELEM tail.
def vc_v09_sequences(ref: Ref, rng: random.Random, n_seq: int) -> list[dict]:
    """Take-off from the pad (the plant on its ground: f_z = g until the thrust lifts it), climb, hold with target
    changes, LAND to the ground, IDLE; plants x0.7 - x1.45 of the thrust model (weak: the search to kt_min and V15's
    ceiling; strong: an early lift-off on the pad segment, the take-off learning); variants switch single v0.9 options
    off; some sequences land 0.5-1.5 s after the lift-off (the rebase) and some carry a W_SAG ceiling."""
    seqs = []
    for s in range(n_seq):
        ov: dict = {}
        if s % 4 == 1:
            ov.update(vz_kp=rng.choice([0.05, 0.08]), alt_kp=rng.choice([1.0, 1.25]))
        vp = vc_params32(ref, ov, v08=True)                     # the take-off check on (the search needs it)
        var = s % 6
        if var == 1:
            vp = dataclasses.replace(vp, to_ref_hold=False)
        elif var == 2:
            vp = dataclasses.replace(vp, land_ref_rebase=False, td_release_per_s=0.0)
        elif var == 3:
            vp = dataclasses.replace(vp, to_learn_tau_s=0.0)
        elif var == 4:
            vp = dataclasses.replace(vp, ref_approach_shift=0)
        vc = VerticalController(vp)
        pl = Plant1D(ref, z_ground=0.0, mass_scale=rng.choice([1.0, 1.0, 0.9]),
                     thrust_scale=rng.choice([0.7, 0.85, 1.0, 1.1, 1.25, 1.45]))
        dt = vp.dt
        n = rng.choice([3000, 5000, 8000])
        cap_seq = s % 3 == 2
        early_land = rng.random() < 0.3
        rows, qs, py, befores, states, ksync = [], [], [], [], [], []
        phase, tgt, baro_lost = lp.PHASE_NONE, 0.0, False
        t_next = rng.randrange(5, 60)
        u, lift_k, u_cap, cap_until = 0.0, None, None, -1
        for k in range(n):
            if k >= t_next:
                x = rng.random()
                if phase == lp.PHASE_NONE:
                    phase, tgt = lp.PHASE_CLIMB, rng.choice([1.5, 2.0, 3.0, 5.0])
                    t_next = k + rng.randrange(1500, 3000)
                elif phase in (lp.PHASE_CLIMB, lp.PHASE_HOLD):
                    if x < 0.4:
                        tgt = round(max(1.0, tgt + rng.choice([-1.0, 1.0, 0.5, -0.5, 2.0])), 1)
                        phase = lp.PHASE_HOLD
                        t_next = k + rng.randrange(400, 1500)
                    else:
                        phase = lp.PHASE_LAND
                        t_next = k + rng.randrange(2500, 5000)
                elif phase == lp.PHASE_LAND:
                    phase = lp.PHASE_NONE
                    t_next = k + rng.randrange(20, 200)
            if early_land and phase == lp.PHASE_CLIMB and lift_k is not None and k == lift_k + rng.randrange(1, 4) * 125:
                phase = lp.PHASE_LAND                                  # a power rule acting in the climb (F9 rebase)
                t_next = k + 4000
            if cap_seq and phase in (lp.PHASE_CLIMB, lp.PHASE_HOLD) and k > cap_until and rng.random() < 0.004:
                u_cap, cap_until = max(u, 0.0), k + rng.randrange(100, 600)   # W_SAG: frozen at the throttle then
            if k > cap_until:
                u_cap = None
            pl.step(u, dt)
            h = pl.z + rng.gauss(0.0, 0.01)
            vz = pl.v + rng.gauss(0.0, 0.02)
            fz = pl.fz + rng.gauss(0.0, 0.05)
            q = q_from_tilt(min(abs(rng.gauss(0.0, 2.0)), 10.0), rng.uniform(0.0, 2.0 * math.pi))
            inp = VerticalInputs(phase=phase, h_tgt_m=f32(tgt), baro_lost=baro_lost, h_m=f32(h), vz_mps=f32(vz),
                                 cos_tilt=vcm.cos_tilt_from_q(q), f_z_mps2=f32(fz),
                                 u_cap=None if u_cap is None else f32(u_cap))
            befores.append(vc_before(vc))
            ksync.append(vc.k_t)
            was_air = vc.airborne
            o = vc.update(inp)
            if vc.airborne and not was_air:
                lift_k = k
            states.append(vc_state(vc))
            u = o.u if phase in (lp.PHASE_CLIMB, lp.PHASE_HOLD, lp.PHASE_LAND) else 0.0
            if o.mode == vcm.MODE_IDLE:
                lift_k = None
            rows.append(inp)
            qs.append(q)
            py.append(o)
        seqs.append(dict(kind="v09", overrides=ov, vp=vp, inputs=rows, q=qs, py=py, before=befores, states=states,
                         ksync=ksync))
    return seqs


def vc_v09_coverage(seqs: list[dict]) -> dict:
    """What the v0.9 sequences exercised, counted on the reference's own states."""
    cov = {"search_ticks": 0, "lift_offs": 0, "ref_hold_ticks": 0, "approach_limited_ticks": 0, "land_rebases": 0,
           "td_release_ticks": 0, "td_release_full_ticks": 0, "u_cap_ticks": 0, "two_slope_pad_ticks": 0,
           "learn_to_ticks": 0, "takeoff_fails": 0, "touchdowns": 0}
    for sq in seqs:
        vp = sq["vp"]
        prev_ref = None
        for inp, o, bef, sta in zip(sq["inputs"], sq["py"], sq["before"], sq["states"]):
            active, ref0, _, ramp_k, _, to_passed0, air0, k0 = bef
            if o.mode == vcm.MODE_FLY:
                if o.k_thrust < k0 and not o.kt_learn:
                    cov["search_ticks"] += 1
                if active and ref0 == o.ref_um and o.ref_um != vcm.target_um(inp.h_tgt_m) and not air0 and \
                        vp.to_ref_hold and ramp_k + 1 < vp.ramp_total_ticks:
                    cov["ref_hold_ticks"] += 1
                dl = o.ref_um - ref0 if active else 0
                if active and 0 < abs(dl) < min(vp.up_um, vp.dn_um) and o.ref_um != vcm.target_um(inp.h_tgt_m):
                    cov["approach_limited_ticks"] += 1
                if ramp_k + 1 <= vp.ramp_ticks and vp.to_u_start > 0.0:
                    cov["two_slope_pad_ticks"] += 1
                if o.kt_learn and air0 and not to_passed0:
                    cov["learn_to_ticks"] += 1
                if inp.u_cap is not None and o.u == inp.u_cap:
                    cov["u_cap_ticks"] += 1
            if sta[6] and not air0:
                cov["lift_offs"] += 1
            if o.mode == vcm.MODE_LAND:
                if (prev_ref is not None and prev_ref[0] != vcm.MODE_LAND and active and vp.land_ref_rebase
                        and math.isfinite(inp.h_m) and vcm.m_to_um(inp.h_m) > ref0):
                    cov["land_rebases"] += 1
                if sta[10] < 0.0:
                    cov["td_release_ticks"] += 1
                if sta[10] <= -vp.td_release_max + 1e-9:
                    cov["td_release_full_ticks"] += 1
            cov["takeoff_fails"] += int(o.takeoff_fail)
            cov["touchdowns"] += int(o.touchdown and o.td_count == vp.landed_confirm_ticks)
            prev_ref = (o.mode, o.ref_um)
    return cov


def check_vertical_v09(ref: Ref, a) -> dict:
    seqs = vc_v09_sequences(ref, random.Random(a.seed + 101), a.vc09_seqs)
    st = check_vertical(seqs)
    st["coverage"] = vc_v09_coverage(seqs)
    cov = st["coverage"]
    need = ("search_ticks", "lift_offs", "ref_hold_ticks", "approach_limited_ticks", "td_release_full_ticks",
            "u_cap_ticks", "two_slope_pad_ticks", "learn_to_ticks", "touchdowns")
    st["missing_coverage"] = [k for k in need if cov[k] <= 0]
    st["pass"] = (st["mismatch_ticks"] == 0 and st["ticks_compared"] > 0.9 * st["ticks"] and not st["missing_coverage"])
    return st


PM_KEYS = ("pw_shift", "pw_arm_lo_q", "pw_arm_hi_q", "pw_buf_arm_lo_q", "pw_buf_arm_hi_q", "pw_lost_q", "pw_lost_margin_q",
           "pw_sag_q", "pw_sag_clear_q", "pw_buf_low_q", "pw_buf_res_q", "pw_d2_q", "pw_i8buf_q", "pw_sag_warn_ticks",
           "pw_sag_land_ticks", "pw_buf_res_ticks", "pw_lost_ticks", "pw_buf_ticks", "pw_d2_ticks", "pw_sag_release_ticks")


def pm_c_overrides(sp: onb.SupervisorParams, build: int) -> dict:
    d = {"pw." + k[3:]: int(getattr(sp, k)) for k in PM_KEYS}
    d["pw.build"] = build
    return d


def pm_sup_params(ref: Ref, power: str, short: bool, rng: random.Random) -> onb.SupervisorParams:
    sp = onb.SupervisorParams.from_params(ref.p, fc_config_path=gfp.FC_CONFIG, mode=lp.MODE_VTOL, power=power)
    if short:                                                   # short persistence: every counter reaches its limit
        sp = dataclasses.replace(sp, pw_lost_ticks=rng.choice([1, 3, 10]), pw_sag_warn_ticks=rng.choice([1, 5, 20]),
                                 pw_buf_ticks=rng.choice([2, 8]), pw_buf_res_ticks=rng.choice([5, 40]),
                                 pw_d2_ticks=rng.choice([3, 12]), pw_sag_release_ticks=rng.choice([1, 6, 30]),
                                 pw_shift=rng.choice([0, 1, 3, 8]))
        sp = dataclasses.replace(sp, pw_sag_land_ticks=sp.pw_sag_warn_ticks + rng.choice([0, 3, 50]))
    return sp


def _code_v(code: int | None, vpc: float) -> float | None:
    """A 10-bit code as the voltage the reference's PowerMonitor turns back into exactly that code."""
    if code is None:
        return None
    v = code * vpc
    assert min(max(lp.round_half_away(v / vpc), 0), 1023) == code
    return v


def pm_voltage_script(rng: random.Random, n: int, wired_buffer: bool, vpc: float) -> list[tuple]:
    """Per tick (c_in, c_tet, c_buf, buf_i_pos) codes: the tether near its set point with sags, dips, a loss, the buffer
    resting / sharing / low / charging (D2), noise, rare missing channels."""
    out = []
    vt, vb = 25.0, 22.8
    ev, ev_end = None, -1
    for k in range(n):
        if k > ev_end and rng.random() < 0.01:
            ev, ev_end = rng.choice(["sag", "dip", "lost", "buf_low", "buf_res", "d2", "arm_lo", "arm_hi", "i8"]),                 k + rng.randrange(5, 400)
        if k > ev_end:
            ev = None
        t_target = {"sag": rng.uniform(18.5, 20.6), "dip": 20.2, "lost": rng.choice([0.0, 12.0]), "arm_lo": 24.3,
                    "arm_hi": 25.9}.get(ev, 25.0)
        b_target = {"buf_low": rng.uniform(19.5, 20.6), "buf_res": rng.uniform(20.8, 21.7), "d2": 24.2,
                    "i8": 18.8}.get(ev, 22.8)
        vt += 0.3 * (t_target - vt) + rng.gauss(0.0, 0.05)
        vb += 0.2 * (b_target - vb) + rng.gauss(0.0, 0.02)
        if ev == "lost" and wired_buffer:
            vin = vb - 0.3                                       # the bus follows the buffer through D2
        else:
            vin = vt - 0.2
        code = lambda x: min(max(int(round(x / vpc)), 0), 1023)  # noqa: E731
        ci, ct, cb = code(vin), code(vt), code(vb)
        if rng.random() < 0.003:
            ci = None
        if rng.random() < 0.002:
            ct = None
        if rng.random() < 0.002:
            cb = None
        out.append((ci, ct if wired_buffer else None, cb if wired_buffer else None,
                    wired_buffer and vb > vt - 0.6 and rng.random() < 0.9))
    return out


def check_pm(ref: Ref, a) -> dict:
    """C pm_sample (core/power_mon.c) vs sim.onboard.PowerMonitor.sample, tick by tick, exact."""
    from sim import power_link as pl
    rng = random.Random(a.seed + 102)
    fin, fout = os.path.join(BUILD, "pm_in.txt"), os.path.join(BUILD, "pm_out.csv")
    seqs = []
    for s in range(a.pm_seqs):
        power = ("wired", "wired_buffer")[s % 2]
        build = pl.build_of_source(power)
        sp = pm_sup_params(ref, power, s % 3 != 0, rng)
        vpc = sp.pw_v_per_count
        pm = onb.PowerMonitor(sp)
        n = rng.choice([800, 2000, 4000])
        script = pm_voltage_script(rng, n, build == pl.PWR_WIRED_BUFFER, vpc)
        powered, rows, py = False, [], []
        for k, (ci, ct, cb, bip) in enumerate(script):
            if rng.random() < 0.004:
                powered = not powered
            thr = f32(rng.uniform(0.0, 1.0))
            inp = onb.OnboardInputs(tick=k, v_in=_code_v(ci, vpc), v_tether=_code_v(ct, vpc), v_buffer=_code_v(cb, vpc),
                                    buf_i_pos=bip, throttle_prev=thr)
            pm.sample(inp, build, powered)
            note = 0 if pm.note is None else (1 if "cleared" not in pm.note else 2)
            va = pm.volts(pm.y_buf if build == pl.PWR_WIRED_BUFFER else pm.y_in)
            py.append(dict(note=note, y_in=-1 if pm.y_in is None else pm.y_in, y_tet=-1 if pm.y_tet is None else pm.y_tet,
                           y_buf=-1 if pm.y_buf is None else pm.y_buf, sag_cnt=pm.sag_cnt, rel_cnt=pm.rel_cnt,
                           sag_warn=int(pm.sag_warn), u_cap=pm.u_cap if pm.sag_warn else None, lost_cnt=pm.lost_cnt,
                           lost=int(pm.lost), buf_cnt=pm.buf_cnt, res_cnt=pm.res_cnt, d2_cnt=pm.d2_cnt,
                           sag_land=int(pm.sag_land()), lost_cond=int(pm.lost_cond()), buf_cond=int(pm.buf_cond()),
                           d2_cond=int(pm.d2_cond()), i8=int(pm.i8_buf()), arm=int(pm.arm_ok(build)),
                           flags=(pm.flags(build) & 0x3F) | (build << 6), vaux=pl.quant_v(va)))
            rows.append(f"{int(powered)} {r32(thr)} {-1 if ci is None else ci} {-1 if ct is None else ct} "
                        f"{-1 if cb is None else cb} {int(bool(bip))}")
        seqs.append(dict(build=build, ov=pm_c_overrides(sp, build), rows=rows, py=py))
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        for i, sq in enumerate(seqs):
            f.write(f"#P {i} {_ov_text(sq['ov'])}\n" + "\n".join(sq["rows"]) + "\n")
    run_exe(["--pm-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        f.readline()
        c_lines = f.read().splitlines()
    st = {"sequences": len(seqs), "ticks": 0, "mismatch_ticks": 0, "first_mismatches": [], "seen": {}}
    i = 0
    for si, sq in enumerate(seqs):
        for k, o in enumerate(sq["py"]):
            p = c_lines[i].split(",")
            i += 1
            if int(p[0]) != si or int(p[1]) != k:
                raise SystemExit(f"pm-vectors row order mismatch at {i}")
            fl = int(p[6])
            c = dict(note=int(p[2]), y_in=int(p[3]), y_tet=int(p[4]), y_buf=int(p[5]), sag_cnt=int(p[7]), rel_cnt=int(p[8]),
                     sag_warn=fl & 1, u_cap=None if p[9] == "nan" else float(p[9]), lost_cnt=int(p[10]),
                     lost=(fl >> 1) & 1, buf_cnt=int(p[11]), res_cnt=int(p[12]), d2_cnt=int(p[13]), sag_land=int(p[14]),
                     lost_cond=int(p[15]), buf_cond=int(p[16]), d2_cond=int(p[17]), i8=int(p[18]), arm=int(p[19]),
                     flags=int(p[20]), vaux=int(p[21]))
            if sq["build"] == 1:                                # the wired build keeps no buffer state, and vice versa
                for key in ("y_tet", "y_buf", "lost_cnt", "lost", "buf_cnt", "res_cnt", "d2_cnt"):
                    c[key] = o[key]
            else:
                for key in ("y_in", "sag_cnt", "rel_cnt", "sag_warn", "u_cap", "note"):
                    c[key] = o[key]
            bad = [key for key in o if not (c[key] == o[key] or (key == "u_cap" and o[key] is not None and c[key] is not None
                                                                   and f32(c[key]) == f32(o[key])))]
            st["ticks"] += 1
            for key in ("sag_land", "lost_cond", "buf_cond", "d2_cond", "i8", "arm"):
                st["seen"][key] = st["seen"].get(key, 0) + o[key]
            st["seen"]["sag_warn_on"] = st["seen"].get("sag_warn_on", 0) + int(o["note"] == 1)
            st["seen"]["sag_warn_cleared"] = st["seen"].get("sag_warn_cleared", 0) + int(o["note"] == 2)
            if bad:
                st["mismatch_ticks"] += 1
                if len(st["first_mismatches"]) < 10:
                    st["first_mismatches"].append({"seq": si, "tick": k, "fields": bad, "c": {x: c[x] for x in bad},
                                                   "py": {x: o[x] for x in bad}})
    if i != len(c_lines):
        raise SystemExit(f"C produced {len(c_lines)} pm rows, expected {i}")
    st["missing"] = [k for k in ("sag_land", "lost_cond", "buf_cond", "d2_cond", "i8", "arm", "sag_warn_on",
                                 "sag_warn_cleared") if st["seen"].get(k, 0) == 0]
    st["pass"] = st["mismatch_ticks"] == 0 and not st["missing"]
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


def check_onboard_power(ref: Ref, a) -> dict:
    """The VTOL supervisor of the wired / wired_buffer builds (sim.onboard.OnboardSupervisor with power='wired' /
    'wired_buffer'; C: the same table + pw.build and the tethered limits) on random operator sequences with power
    scripts and random controller flags (take-off passed, support, lift-off); every supervisor output, the TELEM2 bytes
    and the monitor state compared; every power reason / rejection must occur, LAND and SAFE outcomes both."""
    from sim import power_link as pl
    rng = random.Random(a.seed + 103)
    vp = vc_params32(ref)
    seqs = []
    for s in range(a.pw_sup_seqs):
        power = ("wired", "wired_buffer")[s % 2]
        build = pl.build_of_source(power)
        base = pm_sup_params(ref, power, True, rng)
        lim = onb.tether_limits(base, tethered=True)
        ov = dict(spool_ticks=rng.choice([0, 30]), link_disarm_ticks=140, link_land_ticks=300, endurance_ticks=lim.endurance_ticks,
                  hard_cutoff_ticks=lim.hard_cutoff_ticks, hover_dm=lim.hover_dm, alt_min_dm=lim.alt_min_dm,
                  alt_max_dm=lim.alt_max_dm, ceiling_m=lim.ceiling_m, v_geofence_m=lim.geofence_m)
        sp = sup_params_vtol(ref, ov)
        sp = dataclasses.replace(sp, power=power, power_rules=True,
                                 **{k: getattr(base, k) for k in PM_KEYS + ("pw_v_per_count",)},
                                 w_hover_dm=base.w_hover_dm, w_alt_min_dm=base.w_alt_min_dm, w_alt_max_dm=base.w_alt_max_dm,
                                 w_ceiling_m=f32(base.w_ceiling_m), w_geofence_m=f32(base.w_geofence_m),
                                 w_endurance_ticks=base.w_endurance_ticks, w_hard_cutoff_ticks=base.w_hard_cutoff_ticks)
        vpc = sp.pw_v_per_count
        nd = Nudger(sp)
        op = VtolRandomOperator(rng, ref, sp, vp, nd)
        s0 = rng.randrange(65536)
        sup = TracingSupervisor(sp, seed=s0, reset_in_flight=False)
        n = rng.choice([3000, 6000])
        script = pm_voltage_script(rng, n, build == pl.PWR_WIRED_BUFFER, vpc)
        rows, py = [], []
        tp = sup_f = air = False
        pad_left, forced, forced_left = 0, None, 0
        code = lambda x: min(max(int(round(x / vpc)), 0), 1023)  # noqa: E731
        for t in range(n):
            frames = op.frames(t, sup)
            sens = op.sensors(t, sup)
            ci, ct, cb, bip = script[t]
            if sup.state == onb.State.POWERED:                   # the controller's latches (previous tick): a pad phase
                if pad_left > 0:                                 # (support, nothing passed), then the lift-off and the
                    pad_left -= 1                                # take-off check, then random
                    tp, sup_f, air = False, True, False
                    if pad_left == 0:
                        air, sup_f = True, False
                else:
                    if rng.random() < 0.004:
                        tp = True
                    if rng.random() < 0.01:
                        sup_f = not sup_f
                if forced is None and rng.random() < 0.004:      # a power event while POWERED (often on the pad)
                    forced = rng.choice(["sag", "lost", "buf_low", "d2", "i8"])
                    forced_left = rng.randrange(20, 600)
            else:
                tp = sup_f = air = False
                pad_left = rng.randrange(20, 300)
            if forced is not None:
                forced_left -= 1
                if forced_left <= 0:
                    forced = None
                elif build == pl.PWR_WIRED:
                    ci = code(19.0) if forced in ("sag", "lost") else ci
                elif forced == "lost":
                    ci, ct = code(22.5), code(0.0)
                elif forced == "buf_low":
                    cb = code(19.9)
                elif forced == "d2":
                    cb = code(24.4)
                elif forced == "i8":
                    cb = code(19.2)
            inp = onb.OnboardInputs(tick=t, frames=frames, **sens, v_in=_code_v(ci, vpc), v_tether=_code_v(ct, vpc),
                                    v_buffer=_code_v(cb, vpc), buf_i_pos=bip, vc_takeoff_passed=tp, vc_support=sup_f,
                                    vc_airborne=air)
            n0, n1 = len(sup.exec_log), len(sup.trace)
            out = sup.update(inp)
            op.after(inp, out)
            rows.append(f"P {-1 if ci is None else ci} {-1 if ct is None else ct} {-1 if cb is None else cb} {int(bool(bip))} "
                        f"{int(tp)} {int(sup_f)} {int(air)}\n" + _vtol_inp_row(inp))
            r = _py_row_vtol(sup, out, n0, n1, t)
            pm = sup.pm
            r["pw"] = (pm.y_in if pm.y_in is not None else -1, pm.y_tet if pm.y_tet is not None else -1,
                       pm.y_buf if pm.y_buf is not None else -1, int(out.u_cap is not None),
                       None if out.u_cap is None else f32(out.u_cap),
                       (out.pwr.v_aux_q, out.pwr.flags) if (out.pwr is not None and out.reply_seq is not None) else (0, 0))
            py.append(r)
        ov2 = dict(ov, mode=1, gps_hold=0, takeoff_check=0, v_geofence_dm=sp.v_geofence_dm, **pm_c_overrides(sp, build))
        seqs.append(dict(kind=power, seed=s0, rows=rows, py=py, ov=ov2, sp=sp))
    fin, fout = os.path.join(BUILD, "onbp_in.txt"), os.path.join(BUILD, "onbp_out.csv")
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        for i, sq in enumerate(seqs):
            f.write(f"#S {i} {sq['seed']} 0 {_ov_text(sq['ov'])}\n" + "\n".join(sq["rows"]) + "\n")
    run_exe(["--onb-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        f.readline()
        c_lines = f.read().splitlines()
    st = {"sequences": len(seqs), "ticks": 0, "mismatch_ticks": 0, "first_mismatches": [], "reasons_seen": {},
          "rejections_seen": {}, "safe_on_pad": 0, "land": 0}
    i = 0
    for si, sq in enumerate(seqs):
        for o in sq["py"]:
            parts = c_lines[i].split(",")
            i += 1
            c, interlock, c_exec, _ = _c_onb_row(parts, True)
            st["ticks"] += 1
            bad = [k for k in c if c[k] != o[k]]
            if interlock and o["reason"] != interlock:
                bad.append("interlock")
            j = parts.index("W") if "W" in parts else -1
            if j < 0:
                bad.append("W tail missing")
            else:
                cw = (int(parts[j + 1]), int(parts[j + 2]), int(parts[j + 3]), int(parts[j + 5]),
                      None if parts[j + 6] == "nan" else float(parts[j + 6]), (int(parts[j + 7]), int(parts[j + 8])))
                pw = o["pw"]
                keep = (sq["kind"] == "wired", sq["kind"] == "wired_buffer", sq["kind"] == "wired_buffer", True, True, True)
                for q, (x, y) in enumerate(zip(cw, pw)):
                    if q == 4 and x is not None and y is not None:
                        x, y = f32(x), f32(y)                      # the C prints float32 with 9 digits
                    if keep[q] and x != y:
                        bad.append(("y_in", "y_tet", "y_buf", "u_capped", "u_cap", "tail")[q])
            if interlock:
                nm = lp.REASON_NAMES.get(interlock, str(interlock))
                st["reasons_seen"][nm] = st["reasons_seen"].get(nm, 0) + 1
                if interlock in (0x30, 0x31, 0x32, 0x33, lp.R_I8_VCELL):
                    if o["state"] == int(onb.State.SAFE):
                        st["safe_on_pad"] += 1
                    elif o["phase"] == lp.PHASE_LAND:
                        st["land"] += 1
            for e in c_exec:
                if 0x40 <= e[2] < 0xFE:
                    nm = lp.REASON_NAMES.get(e[2], hex(e[2]))
                    st["rejections_seen"][nm] = st["rejections_seen"].get(nm, 0) + 1
            if bad:
                st["mismatch_ticks"] += 1
                if len(st["first_mismatches"]) < 10:
                    st["first_mismatches"].append({"seq": si, "kind": sq["kind"], "tick": o["t"], "fields": bad,
                                                   "c": {k: c[k] for k in bad if k in c},
                                                   "py": {k: o[k] for k in bad if k in o}})
    if i != len(c_lines):
        raise SystemExit(f"C produced {len(c_lines)} power-supervisor rows, expected {i}")
    need = [pl.POWER_REASON_NAMES[k] for k in pl.POWER_INTERLOCK_REASONS]
    st["missing_reasons"] = [k for k in need if k not in st["reasons_seen"]]
    st["missing_rejections"] = [] if pl.POWER_REASON_NAMES[pl.REJ_POWER] in st["rejections_seen"] else ["REJ_POWER"]
    st["pass"] = (st["mismatch_ticks"] == 0 and not st["missing_reasons"] and not st["missing_rejections"]
                  and st["safe_on_pad"] > 0 and st["land"] > 0)
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


def check_telem4(a) -> dict:
    """The TELEM v4 / v5 prototype (core/link_proto_v09.c) vs the reference's bytes: the TELEM2 / TELEM3 payload of a
    random Telemetry + sim.power_link.PowerTelem.encode(), framed with sim.link_proto's encode_frame (its LEN limit lifted
    to the proposal's 42 for the comparison only)."""
    from sim import power_link as pl
    rng = random.Random(a.seed + 104)
    fin, fout = os.path.join(BUILD, "t4_in.txt"), os.path.join(BUILD, "t4_out.txt")
    rows, want = [], []
    old = lp.MAX_PAYLOAD_LEN
    try:
        lp.MAX_PAYLOAD_LEN = pl.TELEM3_PAYLOAD_LEN_REF + pl.TAIL_BYTES
        for k in range(a.telem4_n):
            v5 = k % 2 == 1
            tl = _rand_telem3(rng) if v5 else _rand_telem2(rng)
            pay = tl.pack()
            assert len(pay) == (lp.TELEM3_PAYLOAD_LEN if v5 else lp.TELEM2_PAYLOAD_LEN)
            tail = pl.PowerTelem.from_physical(rng.uniform(0.0, 32.0), rng.randrange(64), rng.randrange(3))
            veh, seq = rng.randrange(256), rng.randrange(256)
            rows.append(f"{5 if v5 else 4} {veh} {seq} {tail.v_aux_q} {tail.flags} {pay.hex()}")
            want.append(lp.encode_frame(veh, 0x24 if v5 else 0x23, seq, pay + tail.encode()).hex())
    finally:
        lp.MAX_PAYLOAD_LEN = old
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(rows) + "\n")
    run_exe(["--telem4-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        got = f.read().split()
    bad = sum(1 for g, w in zip(got, want) if g != w) + abs(len(got) - len(want))
    lens = sorted({len(w) // 2 for w in want})
    st = {"frames": len(want), "mismatches": bad, "frame_bytes": lens, "pass": bad == 0 and lens == [41, 47]}
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


def check_telem67(a) -> dict:
    """2026-11-10 control round: TELEM v6 / v7 (core/link_proto_v09.c lp_encode_telem6 / 7) vs the reference's bytes: the
    TELEM2 / TELEM3 payload of a random Telemetry + the lbat byte (sim.link_proto with_logic_tail -> encode_telemetry)."""
    rng = random.Random(a.seed + 106)
    fin, fout = os.path.join(BUILD, "t67_in.txt"), os.path.join(BUILD, "t67_out.txt")
    rows, want = [], []
    for k in range(a.telem4_n):
        v7 = k % 2 == 1
        tl = _rand_telem3(rng) if v7 else _rand_telem2(rng)
        lb = rng.randrange(1, 256)
        veh, seq = rng.randrange(256), rng.randrange(256)
        rows.append(f"{7 if v7 else 6} {veh} {seq} {lb} 0 {tl.pack().hex()}")
        want.append(lp.encode_telemetry(veh, seq, lp.with_logic_tail(tl, lb)).hex())
    with open(fin, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(rows) + "\n")
    run_exe(["--telem4-vectors", fin, fout])
    with open(fout, "r", encoding="utf-8") as f:
        got = f.read().split()
    bad = sum(1 for g, w in zip(got, want) if g != w) + abs(len(got) - len(want))
    lens = sorted({len(w) // 2 for w in want})
    st = {"frames": len(want), "mismatches": bad, "frame_bytes": lens, "pass": bad == 0 and lens == [40, 46]}
    if st["pass"]:
        for path in (fin, fout):
            os.remove(path)
    return st


def telem4_link_budget(ref: Ref) -> dict:
    """sim.power_link.telem4_budget for both radio backends (the link_model formulas, params flight_computer.link)."""
    from sim import link_model as lmm
    from sim import power_link as pl
    out = {}
    for radio in getattr(lmm, "RADIOS", ("lora_uart", "sx127x")):         # 2026-11-10: + esp_bridge
        try:
            cfg = lmm.LinkConfig.from_params(ref.p, radio=radio)
        except TypeError:
            cfg = lmm.LinkConfig.from_params(ref.p)
        b = pl.telem4_budget(cfg)
        out[radio] = {k: (v if not isinstance(v, dict) else {kk: (round(vv, 4) if isinstance(vv, float) else vv)
                                                               for kk, vv in v.items()}) for k, v in b.items()}
    return out


def run_v09_sections(ref: Ref, a, report: dict) -> list[str]:
    """Section 10 (v0.9); returns the report keys whose 'pass' counts."""
    t0 = time.perf_counter()
    v9 = check_vertical_v09(ref, a)
    report["vertical_v09"] = v9
    print(f"== v0.9 vertical controller: {v9['ticks']} ticks in {v9['sequences']} sequences, {v9['ticks_compared']} compared, "
          f"mismatches {v9['mismatch_ticks']}, knife edges {sum(v9['knife_edges'].values())} (resynced "
          f"{v9['knife_edges_resynced']}), cut {v9['ambiguous_sequences']}; coverage {v9['coverage']}; max |d| u "
          f"{v9['max_abs']['u']:.2e}, k_T {v9['max_abs']['k_t']:.2e}, fz_lpf {v9['max_abs']['fz_lpf']:.2e}, td_bias "
          f"{v9['max_abs']['td_bias']:.2e}; missing {v9['missing_coverage']} -> {'PASS' if v9['pass'] else 'FAIL'}")
    if v9["first_mismatches"]:
        print("   first mismatches:", json.dumps(v9["first_mismatches"][:3], default=str)[:1500])
    pm = check_pm(ref, a)
    report["power_monitor"] = pm
    print(f"== v0.9 power monitor (pm_sample vs PowerMonitor.sample): {pm['ticks']} ticks in {pm['sequences']} sequences, "
          f"mismatches {pm['mismatch_ticks']}; seen {pm['seen']}; missing {pm['missing']} -> {'PASS' if pm['pass'] else 'FAIL'}")
    if pm["first_mismatches"]:
        print("   first mismatches:", json.dumps(pm["first_mismatches"][:3], default=str)[:1500])
    ps = check_onboard_power(ref, a)
    report["onboard_power"] = ps
    print(f"== v0.9 power rules in the supervisor: {ps['ticks']} ticks in {ps['sequences']} sequences, mismatches "
          f"{ps['mismatch_ticks']}; reasons {ps['reasons_seen']}; rejections {ps['rejections_seen']}; LAND {ps['land']} / "
          f"SAFE on the pad {ps['safe_on_pad']}; missing {ps['missing_reasons'] + ps['missing_rejections']} -> "
          f"{'PASS' if ps['pass'] else 'FAIL'}")
    if ps["first_mismatches"]:
        print("   first mismatches:", json.dumps(ps["first_mismatches"][:3], default=str)[:1500])
    t4 = check_telem4(a)
    report["telem4"] = t4
    print(f"== v0.9 TELEM v4 / v5 prototype: {t4['frames']} frames ({t4['frame_bytes']} B), mismatches {t4['mismatches']} -> "
          f"{'PASS' if t4['pass'] else 'FAIL'}")
    t67 = check_telem67(a)
    report["telem67"] = t67
    print(f"== 2026-11-10 TELEM v6 / v7 (logic-battery tail): {t67['frames']} frames ({t67['frame_bytes']} B), mismatches "
          f"{t67['mismatches']} -> {'PASS' if t67['pass'] else 'FAIL'}")
    try:
        report["telem4_budget"] = telem4_link_budget(ref)
        for radio, b in report["telem4_budget"].items():
            print(f"   {radio}: " + ", ".join(f"{k} {v['frame_bytes']} B rtt {v['rtt_s']:.4f} s ({v['max_poll_hz']:.2f} Hz, "
                                              f"poll ok {v['poll_ok']})" for k, v in b.items() if isinstance(v, dict)))
    except Exception as exc:                                     # the budget is a report, not a check
        report["telem4_budget"] = {"error": repr(exc)}
        print(f"   link budget: {exc!r}")
    report["v09_wall_s"] = round(time.perf_counter() - t0, 1)
    return ["vertical_v09", "power_monitor", "onboard_power", "telem4", "telem67"]


# --------------------------------------------------------------------------- #
# fw09 X1: the campaign replay (SPEC_firmware_v09 8.4, --campaign-set)
# --------------------------------------------------------------------------- #
def campaign_selection(manifest: dict, stride: int) -> list[tuple[str, str]]:
    """(group, record key) of every run the replay needs: every named run, and per group every stride-th campaign run
    (k = i - the group's first index, k % stride == 0)."""
    sel = [("named", str(nm)) for nm in manifest.get("named", [])]
    for g, gd in manifest["groups"].items():
        first = int(gd["first"])
        planned = gd.get("planned")                     # the harness's planned jobs (--limit / --groups launches)
        for i in (sorted(int(x) for x in planned) if planned is not None else range(first, first + int(gd["n"]))):
            if (i - first) % stride == 0:
                sel.append((g, f"{i:04d}"))
    return sel


def _merge_counts(acc: dict | None, st: dict, sums: tuple, lists: tuple, maxes: tuple, dicts_sum: tuple = ()) -> dict:
    if acc is None:
        return json.loads(json.dumps(st, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    for k in sums:
        acc[k] = acc.get(k, 0) + st.get(k, 0)
    for k in lists:
        acc[k] = list(acc.get(k, [])) + list(st.get(k, []))
    for k in maxes:
        for kk, v in st.get(k, {}).items():
            acc.setdefault(k, {})[kk] = max(acc[k].get(kk, 0.0), v)
    for k in dicts_sum:
        for kk, v in st.get(k, {}).items():
            acc.setdefault(k, {})[kk] = acc[k].get(kk, 0) + v
    acc["first_mismatches"] = (list(acc.get("first_mismatches", [])) + list(st.get("first_mismatches", [])))[:10]
    acc["pass"] = bool(acc.get("pass", True)) and bool(st.get("pass", False))
    return acc


def run_campaign(ref: Ref, a) -> dict:
    """Replay the recorded eval-harness runs of --campaign-set (records/<group>/<i>.npz, records/named/<name>.npz,
    records/manifest.json - eval_v09g.py --record): per run the parameter check against the host core of its configuration
    and build profile (check_sixdof_params), then the FCU stack replay through that core (check_stack: supervisor,
    vertical controller, ESC, the GNS-chain slot receiver of the four-chip runs, the power signals of tether / hybrid runs)
    and the NAV estimator / altitude-filter replay (check_mahony / check_altv on the run's own constants).  PASS: every
    selected record present and of this set, 0 build mismatches, 0 mismatching stack ticks, the NAV q within MAHONY_Q_TOL
    and the filter within the ALTV bounds on every compared step."""
    root = os.path.abspath(a.campaign_dir)
    rec_root = os.path.join(root, "records")
    rep: dict = {"set": a.campaign_set, "dir": root, "stride": a.campaign_stride, "errors": [], "pass": False}
    mpath = os.path.join(rec_root, "manifest.json")
    if not os.path.exists(mpath):
        rep["errors"].append(f"{mpath} missing (run the harness with --record)")
        print(f"== campaign replay: {rep['errors'][0]} -> FAIL")
        return rep
    with open(mpath, "r", encoding="utf-8") as f:
        man = json.load(f)
    rep["manifest"] = {k: man.get(k) for k in ("set", "seed", "record_stride", "harness", "harness_sha256",
                                               "hooks_sha256", "tree", "groups", "named")}
    if man.get("set") != a.campaign_set:
        rep["errors"].append(f"the records are of set {man.get('set')!r}, not {a.campaign_set!r}")
    rs = int(man.get("record_stride", 1))
    if a.campaign_stride % rs != 0:
        rep["errors"].append(f"--campaign-stride {a.campaign_stride} is not a multiple of the harness's record stride {rs}")
    sel = campaign_selection(man, a.campaign_stride)
    paths, missing = [], []
    for g, key in sel:
        p = os.path.join(rec_root, g, key + ".npz")
        (paths.append((g, key, p)) if os.path.exists(p) else missing.append(f"{g}/{key}"))
    rep["selected"], rep["missing"] = len(sel), missing
    print(f"== campaign replay {a.campaign_set} (seed {man.get('seed')}): {len(sel)} runs selected (named "
          f"{len(man.get('named', []))}, stride {a.campaign_stride} per group), {len(paths)} records found, "
          f"{len(missing)} missing")
    prm_acc = {"runs": [], "build_mismatch_runs": 0, "pass": True}
    stk = mah = alv = None
    by_group: dict = {}
    no_nav = []
    for b0 in range(0, len(paths), max(1, a.campaign_batch)):
        tb = time.perf_counter()
        batch = paths[b0:b0 + a.campaign_batch]
        recs = []
        for g, key, p in batch:
            r = load_ccrec(p)
            ri = r["meta"].get("run") or {}
            if ri.get("set") != a.campaign_set or int(ri.get("seed", -1)) != int(man.get("seed", -2)):
                rep["errors"].append(f"{g}/{key}: a record of set {ri.get('set')!r} seed {ri.get('seed')}")
            r["group"], r["name"] = g, f"{g}/{key}"
            r["run"]["name"] = f"6dof_{g}_{key}"
            recs.append(r)
        pr = check_sixdof_params(recs)
        prm_acc["runs"] += pr["runs"]
        prm_acc["build_mismatch_runs"] += pr["build_mismatch_runs"]
        prm_acc["pass"] = prm_acc["pass"] and pr["pass"]
        st, _ = check_stack(ref, [r["run"] for r in recs])
        stk = _merge_counts(stk, st, ("ticks", "ticks_compared", "mismatch_ticks", "ambiguous_runs", "ambiguous_ticks",
                                      "knife_edges_resynced", "telem_throttle_one_count_off", "telem_dist_one_count_off"),
                            ("runs", "ambiguous_cut", "pos_knife_edges"), ("max_abs", "max_abs_g"),
                            ("knife_edges", "touchdowns_by"))
        navs = [r["nav"] for r in recs if r["nav"] is not None]
        no_nav += [r["name"] for r in recs if r["nav"] is None]
        if navs:
            mseq = [dict(name=rp["name"], consts=dict(rp["meta"]["est"]),
                         est=np.array([[f32(x) for x in rr[:6]] + [rr[6]] for rr in rp["est"]])) for rp in navs]
            ms = check_mahony(mseq)
            mah = _merge_counts(mah, ms, ("steps", "steps_compared", "flight_steps", "gate_open_steps", "gate_mismatch",
                                          "mismatches", "sequences"), ("knife_edge_cuts", "by_sequence"), ("max_abs",))
            al = check_altv([altv_from_replay(rp, ref.dt) for rp in navs], ref.dt)
            alv = _merge_counts(alv, al, ("steps", "baro_samples", "pad_steps", "outage_steps", "mismatches", "sequences"),
                                ("by_sequence",), ("max_abs",))
            alv["max_h_m"] = max(alv.get("max_h_m", 0.0), al["max_h_m"])
        for r, sr, pp_ in zip(recs, st["runs"], pr["runs"]):
            gq = by_group.setdefault(r["group"], {"runs": 0, "ticks": 0, "mismatch_ticks": 0, "build_mismatch_runs": 0,
                                                  "vcfg": set(), "profiles": set(), "power_builds": set(), "cut": 0})
            gq["runs"] += 1
            gq["ticks"] += sr["ticks"]
            gq["mismatch_ticks"] += sr["mismatch_ticks"]
            gq["build_mismatch_runs"] += int(not pp_["pass_"])
            gq["vcfg"].add(r["run"]["vcfg"])
            gq["profiles"].add(r["run"]["host_profile"] or "field")
            gq["power_builds"].add(r["run"].get("power_build", 0))
            gq["cut"] += int(sr["compared_until"] < sr["ticks"])
        print(f"   batch {b0 // max(1, a.campaign_batch) + 1}: {[r['name'] for r in recs]} - params "
              f"{'ok' if pr['pass'] else 'FAIL'}, stack mismatching ticks {st['mismatch_ticks']} of {st['ticks']}, NAV "
              f"{'-' if not navs else ms['mismatches']} / filter {'-' if not navs else al['mismatches']} mismatching steps "
              f"({time.perf_counter() - tb:.0f} s)", flush=True)
        del recs
    rep["params"] = prm_acc
    rep["stack"], rep["nav_estimator"], rep["nav_alt_filter"] = stk, mah, alv
    rep["by_group"] = {g: {k: (sorted(v) if isinstance(v, set) else v) for k, v in d.items()} for g, d in by_group.items()}
    rep["no_nav_recording"] = no_nav
    n_runs = len(paths)
    ok_stack = (stk is not None and stk["mismatch_ticks"] == 0 and stk["ticks_compared"] > 0
                and len(stk["pos_knife_edges"]) <= max(1, n_runs // 10))
    ok_mah = (mah is not None and mah["mismatches"] == 0 and mah["gate_mismatch"] == 0
              and mah["steps_compared"] >= 0.9 * mah["steps"])
    ok_alv = alv is not None and alv["mismatches"] == 0
    rep["checks"] = {"records_complete": not missing, "records_of_this_set": not rep["errors"],
                     "params": prm_acc["pass"] and n_runs > 0, "stack": ok_stack, "nav_estimator": ok_mah,
                     "nav_alt_filter": ok_alv, "nav_recorded": not no_nav}
    rep["pass"] = all(rep["checks"].values())
    print_sixdof_params(dict(prm_acc, pass_=prm_acc["pass"]), f"campaign {a.campaign_set}: run parameters vs the firmware "
                        "build of each run's configuration (X1)")
    if stk is not None:
        print(f"== campaign {a.campaign_set} FCU stack replay: {len(stk['runs'])} runs, {stk['ticks']} ticks, "
              f"{stk['ticks_compared']} compared, mismatching ticks {stk['mismatch_ticks']}; knife edges "
              f"{stk['knife_edges']} (resynced {stk['knife_edges_resynced']}, runs cut {stk['ambiguous_runs']}); position "
              f"knife edges {len(stk['pos_knife_edges'])}; max |d| {stk['max_abs']} -> {'PASS' if ok_stack else 'FAIL'}")
        for m in stk["first_mismatches"][:5]:
            print(f"   mismatch: {m}")
    if mah is not None:
        print(f"== campaign {a.campaign_set} NAV estimator replay: {mah['steps']} steps ({mah['steps_compared']} compared), "
              f"max |d| q {mah['max_abs']['q']:.3g} (bound {MAHONY_Q_TOL}), gate mismatches {mah['gate_mismatch']}, "
              f"knife-edge cuts {len(mah['knife_edge_cuts'])}, mismatching steps {mah['mismatches']} -> "
              f"{'PASS' if ok_mah else 'FAIL'}")
    if alv is not None:
        print(f"== campaign {a.campaign_set} NAV altitude filter replay: {alv['steps']} steps, max |d| "
              f"{ {k: float(f'{v:.3g}') for k, v in alv['max_abs'].items()} }, mismatching steps {alv['mismatches']} -> "
              f"{'PASS' if ok_alv else 'FAIL'}")
    for g, d in rep["by_group"].items():
        print(f"   {g:16s} runs {d['runs']:3d}  FC_VCFG {d['vcfg']}  profiles {d['profiles']}  power builds "
              f"{d['power_builds']}  ticks {d['ticks']}  mismatching {d['mismatch_ticks']}  build mismatches "
              f"{d['build_mismatch_runs']}  runs cut at a knife edge {d['cut']}")
    if rep["errors"] or missing or no_nav:
        print(f"   errors {rep['errors'][:10]}; missing records {missing[:20]}; without NAV recording {no_nav[:10]}")
    print(f"== campaign {a.campaign_set} checks {rep['checks']} -> {'PASS' if rep['pass'] else 'FAIL'}")
    return rep


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seed", type=int, default=20261012)
    ap.add_argument("--random-seqs", type=int, default=40)
    ap.add_argument("--onb-seqs", type=int, default=120, help="random supervisor sequences")
    ap.add_argument("--proto-streams", type=int, default=3000)
    ap.add_argument("--ilink-streams", type=int, default=3000)
    ap.add_argument("--ilink-snaps", type=int, default=20000, help="random SNAPs through the COM TELEM path")
    ap.add_argument("--nlink-frames", type=int, default=5000, help="random NAV and FCU frames encoded by both")
    ap.add_argument("--nlink-decodes", type=int, default=20000, help="random exchange buffers through both nlink decoders")
    ap.add_argument("--no-recorded", action="store_true", help="skip the 6-DOF recorded runs")
    ap.add_argument("--vtol-seqs", type=int, default=120, help="random VTOL supervisor sequences")
    ap.add_argument("--vc-seqs", type=int, default=150, help="random vertical-controller sequences")
    ap.add_argument("--stack-1d", type=int, default=16, help="1-D closed-loop FcuStack flights replayed into both stacks")
    ap.add_argument("--vtol-runs", nargs="*", default=list(VTOL_6DOF_RUNS),
                    help="sim/vtol_scenarios.py scenarios whose 6-DOF FcuStack inputs are replayed")
    ap.add_argument("--no-vtol-6dof", action="store_true", help="skip the 6-DOF VTOL replays")
    ap.add_argument("--nav-seqs", type=int, default=12, help="synthetic VTOL NAV estimator / altitude-filter sequences")
    ap.add_argument("--nav-runs", nargs="*", default=list(NAV_6DOF_RUNS),
                    help="sim/vtol_scenarios.py scenarios whose NAV estimator / filter calls are recorded and replayed")
    ap.add_argument("--sim-root", default=None,
                    help="record the 6-DOF runs (stack replays 4d, NAV replays 7b) from the sim/ + params.json under this "
                         "directory (a read-only snapshot copy; default: this project)")
    ap.add_argument("--replays", default=None,
                    help="use the stack_*.json / nav_*.npz recordings of this directory instead of recording")
    ap.add_argument("--pos-seqs", type=int, default=60, help="v0.8 random position-controller sequences (8a)")
    ap.add_argument("--gps-flights", type=int, default=14, help="v0.8 NAV GPS closed-loop flights (8b)")
    ap.add_argument("--v08-sup-seqs", type=int, default=32, help="v0.8 random supervisor sequences (8c)")
    ap.add_argument("--stack-3d", type=int, default=7, help="v0.8 3-D closed-loop stack flights with position hold (8d)")
    ap.add_argument("--proto-v3", type=int, default=1500, help="v0.8 streams / encodings per wire format (8e)")
    ap.add_argument("--glink-wire", type=int, default=1500, help="v0.8.1 glink / nlink v4 encodings, decodes, streams (9a)")
    ap.add_argument("--gns-flights", type=int, default=12, help="v0.8.1 GNS node flights (9b)")
    ap.add_argument("--stack-gns", type=int, default=8, help="v0.8.1 four-chip closed loops replayed into the GNS-chain "
                    "FCU stack (9c)")
    ap.add_argument("--vc09-seqs", type=int, default=36, help="v0.9 vertical-controller take-off / landing sequences (10a)")
    ap.add_argument("--pm-seqs", type=int, default=40, help="v0.9 power-monitor sequences (10b)")
    ap.add_argument("--telem4-n", type=int, default=2000, help="v0.9 TELEM v4 / v5 prototype frames (10d)")
    ap.add_argument("--pw-sup-seqs", type=int, default=60, help="v0.9 power-rule supervisor sequences (10c)")
    ap.add_argument("--only", choices=["ilink", "nlink", "log", "vtol", "v08", "v081", "v09", "g1"], help="run one section on its own: build the "
                    "host test from the headers as they are (no generators), check it, write crosscheck_report_<section>"
                    ".json (vtol: the v0.7 sections 4b-4d, 5b, 6b, after gen_firmware_params; v08: sections 8a-8e, after "
                    "gen_firmware_params; v081: sections 9a-9c, after gen_firmware_params; g1 (fw09 X1): the plant check "
                    "per configuration and the 6-DOF runs' parameters vs the build of their configuration, after "
                    "gen_firmware_params, no replay)")
    ap.add_argument("--campaign-set", default=None,
                    help="fw09 X1 (SPEC_firmware_v09 8.4): replay the runs the eval harness recorded (eval_v09g.py "
                         "--record) for this set - every named run and every N-th campaign run per group - each through "
                         "the host core of its configuration (battery -> FC_VCFG 0, tether -> 1, hybrid -> 2; its build "
                         "profile), after checking its parameters against that build; needs --campaign-dir")
    ap.add_argument("--campaign-dir", default=None,
                    help="the harness's --out directory of that set (records in <dir>/records, manifest.json there)")
    ap.add_argument("--campaign-stride", type=int, default=4, help="every N-th recorded campaign run per group (k % N == 0)")
    ap.add_argument("--campaign-batch", type=int, default=6, help="records replayed per batch (memory)")
    a = ap.parse_args(argv)
    t_start = time.perf_counter()
    if a.campaign_set:
        if not a.campaign_dir:
            ap.error("--campaign-set needs --campaign-dir")
        print("== params_gen.h + build (--campaign-set: the configurations' constants come from params_gen.h)")
        if gfp.main([]) != 0:
            return 2
        bht.build()
        ref = Ref()
        rep = run_campaign(ref, a)
        rep["wall_s"] = round(time.perf_counter() - t_start, 1)
        path = os.path.join(BUILD, f"crosscheck_report_campaign_{a.campaign_set}.json")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(rep, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        print(f"crosscheck --campaign-set {a.campaign_set} --campaign-stride {a.campaign_stride}: "
              f"{'PASS' if rep['pass'] else 'FAIL'} ({rep['wall_s']} s); report {os.path.relpath(path, ROOT)}")
        return 0 if rep["pass"] else 1
    if a.only == "g1":
        print("== params_gen.h + build (--only g1: the vehicle constants per configuration)")
        if gfp.main([]) != 0:
            return 2
        bht.build()
        ref = Ref()
        rep = {"seed": a.seed}
        pc = check_plant_configs(ref, a)
        rep["plant_configs"] = pc
        print_plant_configs(pc)
        rec_dir, note6 = sixdof_records(a)
        names = tuple(dict.fromkeys(list(a.vtol_runs) + list(a.nav_runs)))
        recs = load_sixdof_records(rec_dir, names)
        prm = check_sixdof_params(recs)
        prm["records"] = {"dir": rec_dir, "note": note6, "missing": [n for n in names if n not in {r["name"] for r in recs}]}
        if prm["records"]["missing"]:
            prm["pass"] = False
        rep["sixdof_params"] = prm
        print_sixdof_params(prm, "6-DOF run parameters vs the firmware build of each run's configuration (X1)")
        print(f"   6-DOF records: {note6[:600]}")
        rep["pass"] = pc["pass"] and prm["pass"]
        rep["wall_s"] = round(time.perf_counter() - t_start, 1)
        path = os.path.join(BUILD, "crosscheck_report_g1.json")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(rep, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        print(f"crosscheck --only g1: {'PASS' if rep['pass'] else 'FAIL'} ({rep['wall_s']} s); report "
              f"{os.path.relpath(path, ROOT)}")
        return 0 if rep["pass"] else 1
    if a.only == "vtol":
        print("== params_gen.h + build (--only vtol: the VTOL numbers come from params_gen.h, so it is regenerated first)")
        if gfp.main([]) != 0:
            return 2
        bht.build()
        ref = Ref()
        rep: dict = {"seed": a.seed}
        keys = run_vtol_sections(ref, a, rep)
        rep["pass"] = all(rep[k]["pass"] for k in keys)
        rep["wall_s"] = round(time.perf_counter() - t_start, 1)
        path = os.path.join(BUILD, "crosscheck_report_vtol.json")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(rep, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        print(f"crosscheck --only vtol: {'PASS' if rep['pass'] else 'FAIL'} ({rep['wall_s']} s); report "
              f"{os.path.relpath(path, ROOT)}")
        return 0 if rep["pass"] else 1
    if a.only == "v08":
        print("== params_gen.h + build (--only v08: the position-hold / GPS numbers come from params_gen.h)")
        if gfp.main([]) != 0:
            return 2
        bht.build()
        ref = Ref()
        rep = {"seed": a.seed}
        keys = run_v08_sections(ref, a, rep)
        rep["pass"] = all(rep[k]["pass"] for k in keys)
        rep["wall_s"] = round(time.perf_counter() - t_start, 1)
        path = os.path.join(BUILD, "crosscheck_report_v08.json")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(rep, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        print(f"crosscheck --only v08: {'PASS' if rep['pass'] else 'FAIL'} ({rep['wall_s']} s); report "
              f"{os.path.relpath(path, ROOT)}")
        return 0 if rep["pass"] else 1
    if a.only == "v081":
        print("== params_gen.h + build (--only v081: the GNS / glink numbers come from params_gen.h)")
        if gfp.main([]) != 0:
            return 2
        bht.build()
        ref = Ref()
        rep = {"seed": a.seed}
        keys = run_v081_sections(ref, a, rep)
        rep["pass"] = all(rep[k]["pass"] for k in keys)
        rep["wall_s"] = round(time.perf_counter() - t_start, 1)
        path = os.path.join(BUILD, "crosscheck_report_v081.json")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(rep, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        print(f"crosscheck --only v081: {'PASS' if rep['pass'] else 'FAIL'} ({rep['wall_s']} s); report "
              f"{os.path.relpath(path, ROOT)}")
        return 0 if rep["pass"] else 1
    if a.only == "v09":
        print("== params_gen.h + build (--only v09: the v0.9 vertical / power numbers come from params_gen.h)")
        if gfp.main([]) != 0:
            return 2
        bht.build()
        ref = Ref()
        rep = {"seed": a.seed}
        keys = run_v09_sections(ref, a, rep)
        rep["pass"] = all(rep[k]["pass"] for k in keys)
        rep["wall_s"] = round(time.perf_counter() - t_start, 1)
        path = os.path.join(BUILD, "crosscheck_report_v09.json")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(rep, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        print(f"crosscheck --only v09: {'PASS' if rep['pass'] else 'FAIL'} ({rep['wall_s']} s); report "
              f"{os.path.relpath(path, ROOT)}")
        return 0 if rep["pass"] else 1
    if a.only == "ilink":
        print("== build (--only ilink: generators not run, headers used as they are)")
        bht.build()
        vid, hz = il.load_link_params()
        st = check_ilink(vid, hz, a.seed + 3, a.ilink_streams, a.ilink_snaps)
        print_ilink(st)
        st["wall_s"] = round(time.perf_counter() - t_start, 1)
        path = os.path.join(BUILD, "crosscheck_report_ilink.json")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump({"seed": a.seed, "vehicle_id": vid, "loop_hz": hz, "ilink": st, "pass": st["pass"]}, f, indent=1)
        print(f"crosscheck --only ilink: {'PASS' if st['pass'] else 'FAIL'} ({st['wall_s']} s); report "
              f"{os.path.relpath(path, ROOT)}")
        return 0 if st["pass"] else 1
    if a.only == "nlink":
        print("== build (--only nlink: generators not run, headers used as they are)")
        bht.build()
        st = check_nlink(a.seed + 4, a.nlink_frames, a.nlink_decodes)
        print_nlink(st)
        st["wall_s"] = round(time.perf_counter() - t_start, 1)
        path = os.path.join(BUILD, "crosscheck_report_nlink.json")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump({"seed": a.seed, "nlink": st, "pass": st["pass"]}, f, indent=1)
        print(f"crosscheck --only nlink: {'PASS' if st['pass'] else 'FAIL'} ({st['wall_s']} s); report "
              f"{os.path.relpath(path, ROOT)}")
        return 0 if st["pass"] else 1
    if a.only == "log":
        print("== build (--only log: generators not run, headers used as they are)")
        bht.build()
        lg = check_log_roundtrip()
        print(f"== log format round trip: per record type {lg['checks']} -> {'PASS' if lg['pass'] else 'FAIL'}")
        path = os.path.join(BUILD, "crosscheck_report_log.json")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump({"log_roundtrip": lg, "pass": lg["pass"]}, f, indent=1)
        return 0 if lg["pass"] else 1
    rng = np.random.default_rng(a.seed)
    print("== generators / build")
    if gfp.main([]) != 0 or ggh.main([]) != 0:
        return 2
    bht.build()
    ref = Ref()
    report = {"seed": a.seed, "gains_source": ref.ginfo["source"], "gains_warnings": ref.ginfo["warnings"],
              "plant": check_plant(ref)}
    print(f"== plant: generator == sim.vehicle (l_v {ref.d['l_v'] * 1e3:.3f} mm, source {ref.veh.mass_source})")
    pc = check_plant_configs(ref, a)                     # fw09 X1: section 2 per configuration
    report["plant_configs"] = pc
    print_plant_configs(pc)
    runs = []
    if not a.no_recorded:
        runs = recorded_runs(ref)
        report["recorded_runs"] = [{"name": r["name"], "ticks": r["n"], "wall_s": round(r["wall"], 2),
                                    "events": [f"{t:.3f} {e}" for t, e in r["events"]]} for r in runs]
        print("== recorded 6-DOF runs: " + ", ".join(f"{r['name']} ({r['n']} ticks)" for r in runs))

    # controller
    rnd = random_controller_sequences(rng, a.random_seqs, ref.dt)
    rec = recorded_controller_sequences(runs)
    seqs = rnd + rec
    kinds = ["random"] * len(rnd) + ["recorded"] * len(rec)
    cs = check_controller(ref, seqs, kinds)
    report["controller"] = cs
    print(f"== controller: {cs['vectors']} vectors in {cs['sequences']} sequences "
          f"({cs['by_kind'].get('random', {}).get('vectors', 0)} random, {cs['by_kind'].get('recorded', {}).get('vectors', 0)} recorded); "
          f"max |d delta| = {cs['max_abs_d_delta_rad']:.3e} rad (bound {DELTA_BOUND_RAD:.0e}), "
          f"max |d pwm| = {cs['max_abs_d_pwm_us']:.3e} us, max |d I| = {cs['max_abs_d_integ_nm']:.3e} N m, "
          f"max |d M| = {cs['max_abs_d_moment_nm']:.3e} N m")
    print(f"   coverage: {cs['rows_saturated']} saturated, {cs['rows_floored']} thrust-floored, "
          f"{cs['rows_rate_sp_clamped']} rate-setpoint-clamped rows; sat-flag mismatches ambiguous "
          f"{cs['sat_flag_mismatch_ambiguous']}, hard {cs['sat_flag_mismatch_hard']} -> {'PASS' if cs['pass'] else 'FAIL'}")

    # onboard supervisor
    t0 = time.perf_counter()
    oseqs = onb_random_sequences(ref, a.onb_seqs, a.seed + 1)
    if runs:
        oseqs += onb_recorded_sequences(ref, runs)
    os_ = check_onboard(ref, oseqs)
    os_["wall_s"] = round(time.perf_counter() - t0, 1)
    report["onboard"] = os_
    print(f"== onboard supervisor: {os_['ticks']} ticks in {os_['sequences']} sequences "
          f"({', '.join(f'{k}: {v['sequences']} seq / {v['ticks']} ticks' for k, v in os_['groups'].items())}), "
          f"{os_['frames']} uplink frames, {os_['telemetry_frames']} TELEM replies; mismatching ticks {os_['mismatch_ticks']}; "
          f"I10 boots {os_['i10_boots']}; inputs moved out of the float32 band {os_['nudged_inputs']} "
          f"-> {'PASS' if os_['pass'] else 'FAIL'}")
    print(f"   interlocks: {dict(sorted(os_['interlocks_seen'].items()))}")
    print(f"   rejections: {dict(sorted(os_['rejections_seen'].items()))}")
    print(f"   executed:   {dict(sorted(os_['commands_executed'].items()))}")
    print(f"   transitions: {dict(sorted(os_['transitions_seen'].items()))}")
    if os_["recorded_replay_state_agreement"]:
        print(f"   recorded replay (float32 inputs) reproduces the 6-DOF run's own states: {os_['recorded_replay_state_agreement']}")
    if os_["missing_interlocks"] or os_["missing_rejections"]:
        print(f"   MISSING coverage: interlocks {os_['missing_interlocks']}, rejections {os_['missing_rejections']}")
    for m in os_["first_mismatches"][:5]:
        print(f"   mismatch: {m}")

    # protocol
    ps = check_protocol(ref, a.seed + 2, a.proto_streams, 2000, 200000)
    report["protocol"] = ps
    print(f"== link protocol: {ps['streams']} streams ({ps['stream_bytes']} bytes, {ps['py_frames']} valid frames, counters "
          f"{ps['py_counters_total']}), {ps['encodings']} encodings, {ps['quantisations']} quantisations; mismatches "
          f"{ps['mismatches']} -> {'PASS' if ps['pass'] else 'FAIL'}")
    for m in ps["first_mismatches"][:3]:
        print(f"   mismatch: {m}")
    ils = check_ilink(int(ref.p["flight_computer"]["link"]["vehicle_id"]), ref.loop_hz, a.seed + 3, a.ilink_streams,
                      a.ilink_snaps)
    report["ilink"] = ils
    print_ilink(ils)
    nls = check_nlink(a.seed + 4, a.nlink_frames, a.nlink_decodes)
    report["nlink"] = nls
    print_nlink(nls)

    es = check_estimator(ref, rng, 8)
    report["estimator"] = es
    print(f"== estimator (PadAlignGyroEstimator, N_avg {es['n_avg']}): {es['rows']} steps, max |d q| {es['max_abs_d_q']:.2e}, "
          f"max |d bias| {es['max_abs_d_bias']:.2e} rad/s, max |d rate| {es['max_abs_d_rate']:.2e} rad/s "
          f"-> {'PASS' if es['pass'] else 'FAIL'}")
    al = check_alt_filter(ref, rng, 6, ref.d["baro_div"])
    report["alt_filter"] = al
    print(f"== altitude filter: {al['rows']} steps (|h| up to {al['max_h_m']:.0f} m), max |d h| {al['max_abs_d_h_m']:.2e} m, "
          f"max |d v| {al['max_abs_d_v_mps']:.2e} m/s -> {'PASS' if al['pass'] else 'FAIL'}")
    lg = check_log_roundtrip()
    report["log_roundtrip"] = lg
    print(f"== log format round trip (format {decode_log.FORMAT_VERSION}): {lg['records']} flight, {lg['events']} event, "
          f"{lg['link_records']} link, {lg['boot']} boot, {lg['status']} status (v4), {lg['com']} COM ('C') records, "
          f"{lg['skipped_bytes']} junk bytes skipped; per type {lg['checks']} -> {'PASS' if lg['pass'] else 'FAIL'}")

    # v0.7 VTOL build (own random seeds: the sections above keep their v0.6.3 draws)
    vkeys = run_vtol_sections(ref, a, report)
    # v0.8 GPS position hold (own random seeds)
    vkeys += run_v08_sections(ref, a, report)
    # v0.8.1 four-chip build: glink, the GNS node, the FCU's GNS chain (own random seeds)
    vkeys += run_v081_sections(ref, a, report)
    # v0.9: the take-off / reference / touchdown controller, the power monitor and rules (own random seeds)
    vkeys += run_v09_sections(ref, a, report)

    ok = all(report[k]["pass"] for k in ("plant_configs", "controller", "onboard", "protocol", "ilink", "nlink", "estimator",
                                         "alt_filter", "log_roundtrip", *vkeys))
    report["pass"] = ok
    report["wall_s"] = round(time.perf_counter() - t_start, 1)
    with open(os.path.join(BUILD, "crosscheck_report.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(report, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    print(f"crosscheck: {'PASS' if ok else 'FAIL'} ({report['wall_s']} s); report "
          f"{os.path.relpath(os.path.join(BUILD, 'crosscheck_report.json'), ROOT)}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
