"""6-DOF rigid-body flight simulation (SPEC §9), ATmega128 flight computer.

Physics (RK4; ``control.loop_hz`` x ``physics_substeps`` = ``control.imu_hz``
= 1 kHz, steps additionally broken at every actuator-command change):
state x = [r_w(3), v_w(3), q(4), omega_b(3), T, delta(4)]

* gravity -m g z_w
* thrust T along +z_b with the first-order lag of §8.7 (``sim.edf``)
* vane side forces q_e S_v C_L(delta_i) along e_theta_i at (r_v, z_hinge) and
  their moments about the CG (exact cross products; §8.3-8.4)
* residual swirl torque about z_b (§8.4)
* body drag 1/2 rho V^2 C_d A_ref along -V_rel and the normal force
  1/2 rho V^2 A_ref C_Na sin(alpha) at the Barrowman CP (``sim.vehicle``)
* wind: steady + gust (world frame)
* servo: first-order lag + rate limit + mechanical saturation (``servo.*``)
* parachute after DEPLOY (drag at the tether point, inflation lag, line damping)
* ground: the vehicle cannot sink below the pad; it leaves when the net
  vertical acceleration is positive and sticks on touchdown

Flight-computer timing (``sim.fc_timing``): the controller runs at
``control.loop_hz`` (250 Hz); the command computed from the sample of tick k
reaches the output registers ``FCTiming.delay_s`` later (default: the v0.6.3
co-simulation's measured maximum, 4.71 ms from the NAV's IMU sample to the
FCU's last vane register); servo and ESC commands are latched only at the
``flight_computer.servo_frame_hz`` PWM frames (zero-order hold, fixed phase
offset to the control tick) and act at the falling edge of their pulse; then
servo lag + rate limit + saturation.

Pad / flight switch (v0.6.3 NAV, ``firmware/.../nav/nav_main.c``): the NAV
selects pad mode (estimator alignment, altitude filter held at 0) from the FCU
state of the last nlink exchange, ``SensorConfig.nav_state_lag_ticks`` = 2
control ticks behind the supervisor (v0.5 single chip: 1).  The altitude hold
follows that state, not the true liftoff.

Sensors: MPU-6050 gyro/accelerometer (``sim.estimator.Mpu6050``: noise,
slowly varying gyro bias, on-chip DLPF, +-2000 deg/s range, int16
quantisation), BMP280 barometer (``control.baro_hz``; ``BARO_MODELS``: 0.44 m
RMS, 0.055 m steps, 0.1 s latency; the MS5611 of v0.5 .. v0.9 stays selectable).
Attitude estimate (``SensorConfig.attitude_estimator``):
``pad_gyro`` = the firmware estimator (``sim.estimator.PadAlignGyroEstimator``:
gyro bias + accelerometer levelling on the pad, gyro-only after launch;
default), ``truth_noise`` = truth + 0.5 deg noise (BNO085-class reference) or
``mahony`` (legacy study).  The controller (``sim.controller``) and the
onboard supervisor (``sim.onboard``) run on the sensed quantities only.

Ground link (SPEC §8.8 / §10.3, v0.4): the vehicle has no mission sequencing.
Every flight scenario runs the ground station (``sim.ground_station_logic``,
assist mode by default, a scripted operator for ARM / LAUNCH / KILL) and the
onboard supervisor (``sim.onboard``) connected through ``sim.link_model``: the
ground polls, an uplink CMD frame reaches the flight computer after the
uplink latency (lost with ``link.sim_packet_loss``) and is handled at the next
control tick after the COM's forwarding (``link_model.COM_FORWARD_S``); the
TELEM reply leaves ``link_model.FC_REPLY_DELAY_S`` after the uplink frame
arrived (measured v0.6.3 on-vehicle reply path) and reaches the ground after
the downlink latency.  Frames are real bytes (``sim.link_proto``) and go
through the same streaming decoders as on the hardware.  S1-type runs
(``bypass_state_machine``) fly the controller alone at a fixed throttle.

VTOL build (v0.7, SPEC 8.9; ``Scenario.mode = "vtol"``): the FCU tick is
``sim.onboard.FcuStack`` - the supervisor (VTOL rules, phases) first, then the
vertical controller (``sim.vertical_ctrl``, altitude / vertical-speed loop on the
NAV's altitude filter), the ESC command = its throttle while the supervisor
enables the motor, then the attitude controller with that throttle.  The whole
tick output (vanes and ESC) goes through the same compute delay + PWM-frame latch
as the ballistic build (``FCTiming``: 4.71 ms measured).  The ground is a
``sim.vtol_scenarios.VtolOperator`` pressing ON / UP / DOWN / OFF / KILL through
the real ``GroundAssist`` (assist off) and the link model; the downlink carries
TELEM v2 frames (39 B, ``link_proto.TYPE_TELEM2``) and the ground's reply timeout
is built on that frame size (``vtol_round_trip``).  Added for the VTOL runs (each a
no-op at its default, so a ballistic ``Scenario`` executes exactly the v0.6.3
statements - every VTOL statement sits behind ``if vt:``):

* landing surface: the pad's top plate (z = 0, radius ``launch_pad.plate_od / 2``
  around the origin) and the ground everywhere else at ``Scenario.ground_z_m``
  (default: the level the pad's feet stand on, -(plate_t + leg_h + foot_t) =
  -0.11 m; may be set lower, e.g. a pad on a platform).  Contact is the exit-plane
  centre against the surface below it (no rim / fin geometry): the first contact
  with v_z < 0 is a touchdown (velocities and body rates zeroed, the attitude
  kept - no tip-over model), the vehicle then rests there until the net vertical
  force lifts it again (a hop is a lift-off + another touchdown, all recorded);
* NO ground-effect model (thrust near the ground is the free-air thrust), no
  position or velocity sensing (nothing on the vehicle measures horizontal
  motion: wind and tilt errors drift it, reported as drift metrics);
* wind = steady + gust schedule (``Gust``: absolute times, step or 1 - cos ramps)
  + turbulence (first-order Gauss-Markov per axis, ASSUMED correlation time and
  vertical ratio) from its own random stream (the v0.6.3 draws stay untouched);
* faults: baro silent from ``baro_fail_t_s`` (the NAV filter then integrates the
  accelerometer only), IMU / NAV failure from ``imu_fail_t_s`` (NAV outputs frozen,
  ``imu_ok`` false), the ballistic vane hard-over / freeze, link outage windows;
* plant spread for the campaign (``mass_scale``, ``cg_offset_mm``,
  ``thrust_scale``, ``thrust_voltage_exp``, ``servo_tau_scale``,
  ``vane_trim_err_deg``): only the plant changes, the controllers keep the nominal
  model.  The thrust model has no voltage term unless ``thrust_voltage_exp`` > 0
  (T x (V_cell / V_nom_cell)^k, a what-if: params.json has no measured curve).

VTOL v0.7.1 (architect 2026-10-20; every item behind ``if vt`` / a VTOL-only Scenario field):

* FCU tick order (mission.vtol.fcu_tick_order_note): ``FcuStack.step`` returns the
  attitude controller's inputs of THIS tick = the previous tick's control_active and
  ESC throttle (``att_active`` / ``att_u``); the vanes keep the measured 4.71 ms path,
  the ESC gets its own latch with ``FCTiming.esc_delay_s`` (vane path +
  ``fc_timing.FC_VTOL_ESC_EXTRA_S_ASSUMED`` 0.80 ms for the supervisor + vertical
  controller, ASSUMED from the v0.7 measurements until the co-simulation measures it);
* NAV: ``sim.estimator.GatedMahonyEstimator`` (``SensorConfig.attitude_estimator =
  "gated_mahony"``, gains params nav_estimator <- Scenario.vgains est_* <- SensorConfig
  gm_*; the pad alignment runs during the pre-alignment) and
  ``sim.state_machine.VtolAltitudeFilter`` (pad bias calibration also during the
  pre-alignment; the thrust compensation on the FCU's last ESC throttle the NAV has,
  ``nlink`` thr_q with the same 2-tick lag as the state); the FCU gets the NAV
  quaternion and the body-z specific force (``OnboardInputs.q_nav`` /
  ``accel_z_mps2``; frozen with the IMU);
* aerodynamics (params aero.vtol_*, ``vtol_aero_geometry``): axial drag on the axial
  speed, body cross-flow drag 1/2 rho C_dc A_plan |V_perp| V_perp at the side-silhouette
  centroid, EDF ram drag sqrt(rho A_exit T) V_perp at the intake windows (station
  velocities include omega x r) - replacing the slender-body normal force;
  ``crossflow_cd_scale`` / ``ram_drag_scale`` spread them;
* wind: the mean wind's direction swings (sinusoid), wanders (random-walk rate,
  ``WIND_DIR_RATE_TAU_S_ASSUMED``), reverses (180 deg, 1 - cos) and its speed may be
  modulated (``wind_mean_vtol``; logged as ``wind_mean``), plus the gusts and turbulence;
* hand pushes (``Push``): a world force (half-sine / trapezoid) at the body-surface point
  facing the person at the push height (re-evaluated from the attitude every physics
  step: the pole tip slides on the round body; an optional tangential offset -> yaw
  torque), plant only; logged as push_force / push_point / push_active;
* sensors: the barometer's static-pressure error with the fan running
  (``baro_thrust_offset_m`` x T / (m g), sensor side) and the firmware's compensation
  (``baro_thrust_comp_m``; ``SensorConfig.baro_comp_lagged_whatif`` = a WHAT-IF filter
  with the compensation through the thrust lag and the baro latency); accelerometer
  vibration (``SensorConfig.accel_vibration_rms_mps2``, ASSUMED, own random stream);
* metrics: touchdown horizontal speed relative to the mean wind (the criterion; the
  vehicle drifts with the air - no position sensing) and to the local wind, the push
  response (``push_response``), which touchdown signature / rule confirmed LANDED.

VTOL v0.7.1 fix round (2026-10-21, after the independent physics review and the v0.7.1b control fix; every item VTOL
only, the ballistic statements are untouched):

* attitude gains: ``sim.controller.gains_for_mode`` - params.control <- the gains file <- (VTOL) control.vtol_overrides
  (v0.7.1b: rate_i_max_z 0.25, the review found the body-axis integrator pinned at 0.15 by the residual swirl); the
  ballistic dict is ``gains_from_control`` bit for bit;
* the vertical controller's thrust learning is logged (``vc_k_thrust``, ``vc_kt_learn``, ``vc_low_alt``); the
  touchdown signature names come from ``sim.vertical_ctrl.TD_NAMES`` (v0.7.1b adds 4 = "all");
* drag time constant (``vtol_aero_geometry``): the EDF ram drag acts at the intake 0.108 m below the CG, so holding
  the attitude needs a vane side force (l_in / l_v) D_ram ALONG the motion - the effective linear damping is
  mdot (1 + l_intake_above_cg / l_v) = 0.682 mdot and the time constant m / that (9.55 s, not m / mdot = 6.51 s);
* plant: ``Scenario.swirl_scale`` (the ASSUMED residual swirl torque x this: the envelope / campaign sensitivity);
  the push contact point is taken from the CG (the body axis is offset by ``cg_offset_mm``); the VTOL scenarios
  default to the CAD lateral CG (``sim.vtol_scenarios.cad_cg_offset_mm``);
* metrics: the cause of an I3 ABORT from the true rates over the persistence window (tilt rate |w_xy| vs spin |w_z|),
  max |w_xy| and |w_z| per push, lift-off / k_T summaries.

VTOL v0.8 (GPS position hold; architect 2026-10-21, params mission.vtol.position_hold, electronics.gps,
flight_computer.nav.gps; every item behind ``Scenario.gps`` / ``Scenario.nav_v08`` - with both at their defaults a VTOL
scenario runs the v0.7.1 statements and the v0.7.1 supervisor build, ``gps=False`` / ``takeoff_check=False``):

* GPS receiver (plant, ``GpsSim`` / ``GpsReceiverPlant``): u-blox NEO-7M class (2026-11-10: the part on the vehicle;
  ``neo_6m`` = the NEO-6M model of v0.8 .. v0.9, ``neo_m8n`` = the upgrade path) at the CAD antenna phase centre (``Layout.gps_antenna_xyz``, option-B pod; the true lever arm is that point
  minus the plant CG, the NAV subtracts its params / CAD constant); UART-level: the receiver answers the NAV's boot
  configuration (CFG-PRT at 9600 or 38400, CFG-RATE, CFG-MSG, CFG-NAV5 with ACK / NAK, MON-VER) and, once configured,
  sends NAV-POSLLH, NAV-SOL and NAV-VELNED of every epoch (the GPS-time grid of the configured rate) as UBX bytes at its
  baud rate, the last byte ``latency_s`` (+ jitter) after the measurement; per-axis error = Gauss-Markov wander + white
  noise + satellite-set steps (+ multipath jumps / ramps, scripted or random) on the position, white + Gauss-Markov on the
  velocity, quantised by the protocol (1e-7 deg, cm/s); fix acquisition (TTFF), outages (no fix, messages still flow),
  silent windows (no bytes), degraded windows, a sky-blocked fraction (satellites x (1 - f), errors x sqrt(n_open / n),
  ASSUMED geometry scaling); every number a ``GpsSim`` field with its source;
* NAV (v0.8 order, ``sim.estimator`` docstring): IMU read -> ``predict`` -> publish (q_pub, rate, accel, the previous
  tick's h / v_z, cos_tilt / tilt from q_pub, the GPS slot of this tick) -> Mahony correction -> altitude filter on the
  corrected attitude -> ``sim.gps_nav.NavGps`` on the UART bytes of this tick (parser, configurator, iTOW epoch assembly,
  fixed-gain filter, latched snapshot at k mod 5 == 4 -> ``sim.nlink_proto.GpsSlotTx``);
* FCU: ``sim.onboard.FcuStack`` v0.8 (slot receiver, ON gate, V11-V15, the 50 Hz position step after the ESC write with
  its 4-tick job pipeline, the 250 Hz tilt-setpoint builder); the attitude controller of each tick flies the tilt the
  builder made at the end of the previous tick (``FcuOutputs.att_tilt`` -> ``AttitudeController.tilt_sp``);
* v0.8.1 four-chip chain (``GpsSim.chain`` "gns", the default when params has flight_computer.glink; "fcu" = the v0.8
  chain above): the receiver feeds the GNS board (``sim.glink_proto.GnsNode``: UBX, the same filter, the position
  controller), the NAV sends the glink STATE frame at k mod 5 == 4 (``GlinkNav``), the GNS answers in the next tick, the
  NAV puts the RESULT into the nlink v4 slot of tick k + L (``GpsSim.glink_latency_ticks``, else params
  glink.latency_ticks_ASSUMED), the FCU stack runs without the position controller (``FcuStack(chain="gns")``: the slot's
  TILT, the TRIM fallback, its own V11) and returns pflags / k_T in the nlink v4 FCU frame with the nlink lag; real bytes
  on both links; ``GpsSim.glink_cut`` windows cut the glink wire (fault study);
* pole pushes (``Push.hand_height_m``): the force acts along the pole from the person's hand to the contact point (fixed
  at the push start; the old horizontal model when ``hand_height_m`` is None);
* the plant's mass, CG and inertia are the CAD report's (``sim.vehicle``; v0.8: the GPS pod included).

VTOL v0.9 plant (VTOL hardware round, architect 2026-10-28; each element a Scenario field, off at 0 -> the v0.8.1
statements bit for bit, which the regression pins check):

* hardware configuration: the VTOL scenarios fly ``Vehicle.hardware_config`` 'vtol' (``sim.vehicle.load_vehicle(config=
  'vtol', power=...)``: the CAD report of the power source - battery 1647.2 g, tether 1399.3 g, hybrid 1520.3 g; no chute:
  a release opens nothing; the GPS antenna on the top cap); the ballistic mission flies the rocket;
* direct-drive vane play (``vane_play_deg``, params vane.direct_drive_play 0.65 + 1.0 deg ASSUMED): the vane angle is the
  play operator of the servo state (``sim.edf.vane_play``, memory ``vane_mem`` updated after every step); logged
  ``delta`` = the vanes, ``delta_servo`` = the servo outputs;
* EDF rotor (``rotor_inertia_kg_m2``, params edf.thrust_v09 4.5e-5 ASSUMED): omega(T) = omega_ref sqrt(T / T_fan), the
  gyroscopic moment -w x h and the spool-up / spool-down reaction +s I domega/dt in the derivative (``sim.edf.RotorModel``;
  with the 0.15 s thrust lag a motor cut gives ~0.3 N m of yaw torque for ~0.3 s - the stators turning the decaying
  swirl);
* thrust envelope (``t_max_kgf``: the plant T_max at 22.2 V; the controllers keep params thrust_nominal_N) with the voltage
  term T x (V_bus / 22.2)^k (k = params voltage_exponent_proxy 1.61 unless ``thrust_voltage_exp`` > 0);
* VTOL electrical plant (``power_source`` / ``power`` / ``power_events``; ``sim.tether.PowerSystem``): battery / tether /
  hybrid bus solved once per tick on the lagged thrust -> v_cell = V_bus / cells (what the FCU's divider reads), the thrust
  limit when the sources cannot carry the fan power, the measured signals (``OnboardInputs`` fields the supervisor declares:
  v_in, v_tether, v_buffer, buf_i_pos); a dead bus stops the ESC and holds the servos;
* tether mechanics (``tether``: ``sim.tether.TetherConfig`` / ``TetherRig``): overhead anchor, cable feed (constant
  tension or slack catenary + half the span's drag) held per step, the catch line (bungee spring-damper when taut) in the
  derivative at the eye bolt, the snag jerk; logs tether_catch_n / tether_cable_n / tether_tension_n / tether_l0 / tether_d;
* touchdown (VTOL configuration): the tip-over margin uses the solid-fin footprint (``Vehicle.footprint_radius_m`` along the
  motion, less the CG offset, CG height z_cg); ``power_tether_metrics`` adds the bus / supply / tether / buffer numbers.

Plant limits of v0.9: the pad holds the vehicle until lift-off (the spool-up reaction is absorbed by the pad even when the
thrust nearly equals the weight and the real friction would not hold it); the rotor speed is algebraic in the lagged
thrust (no separate motor / ESC dynamics; the no-load speed is used at T_max - an upper bound); the FCU / COM logic keeps
running on a dead bus (its outputs have no effect); one bus solve per tick (no LC / capacitor dynamics, no regeneration).

GPS physical limits stated: open sky only (no position hold indoors - the gps=None build is the indoor fallback); the
hold is to within the GPS wander (ASSUMED Gauss-Markov 2.12 m / 200 s: quote ~1 m RMS / 2.5 m max until a real module
is measured); no body blocking / ESC / LoRa interference model beyond the sky-blocked fraction, outages and degraded
windows; no ground-effect model.

Not modelled (stated physical / model limits): ground effect; tip-over and contact bounce (the touchdown tip-over
margin is EVALUATED by the energy method, ``tip_over_vcrit``, not simulated); position / velocity sensing;
jet damping and the internal-flow Coriolis moment (~0.05 N m s/rad of pitch damping - stabilising, so the omission is
conservative); the thrust lapse with axial inflow (~mdot v_z, ~0.4 N per m/s - also conservative); the IMU lever arm
(the NAV board sits 0.03-0.14 m above the CG: alpha x r, omega x (omega x r) are not in the accelerometer);
thrust-line misalignment (0.5 deg ~ 0.22 N of side force, the same effect as a CG offset); the person and the pole;
angular vibration of the gyro; the cross-flow drag is lumped at one station (no distributed pitch damping); C_dc 1.2
(params, ASSUMED) is high for a finite cylinder of L/D ~7 (0.7-0.9 - the campaign spreads +-30 %); after an ABORT
release the VTOL drag geometry stays the full vehicle's (the descent is parachute-dominated); an off-centre push needs
contact friction >= tan(offset) (not enforced; the campaign clips offsets to +-30 deg).

Invariant: the truth state never reaches the controller, the supervisor or the
ground; every number that is not in ``params.json`` is an explicit
``SensorConfig`` / ``SimOptions`` field (or a ``sim.link_model`` constant) marked
ASSUMED in its comment.
"""
from __future__ import annotations

import math
import struct
import time
from collections import deque
from dataclasses import dataclass, field, asdict, replace
from typing import Any

import numpy as np

try:
    from sim import glink_proto as gl
    from sim import gps_nav as gn
    from sim import link_proto as lp
    from sim import power_link as pl
    from sim import nlink_proto as nl
    from sim import quat
    from sim.position_ctrl import PositionParams
    from sim.controller import AttitudeController, MixerPlant, gains_for_mode
    from sim.edf import EDFModel, RotorModel, VaneModel, vane_play, vane_play_deg_from_params
    from sim import tether as tt
    from sim.estimator import (GatedMahonyEstimator, Mpu6050, PadAlignGyroEstimator, gyro_bias_process, up_error_rad,
                               white_noise_sigma)
    from sim.fc_timing import EPS_T, FCTiming, OutputLatch, frame_hz_from_timer, glink_latency_ticks, timer_of_channel
    from sim.ground_station_logic import GroundAssist, GroundParams
    from sim.link_model import COM_FORWARD_S, FC_REPLY_DELAY_S, LinkConfig, LinkModel, budget as link_budget, round_trip
    from sim.onboard import (TARGET_PHASES, FcuStack, OnboardInputs, OnboardSupervisor, State, SupervisorParams,
                             UplinkFrame)
    from sim.state_machine import VTOL_ALT_BIAS_TAU_S_ASSUMED, AltitudeFilter, VtolAltitudeFilter
    from sim.vehicle import G0, Vehicle
    from sim.vertical_ctrl import MODE_IDLE, TD_NAMES, VerticalParams
except ImportError:  # pragma: no cover - flat import when sim/ itself is on sys.path
    import glink_proto as gl  # type: ignore
    import gps_nav as gn  # type: ignore
    import link_proto as lp  # type: ignore
    import power_link as pl  # type: ignore
    import nlink_proto as nl  # type: ignore
    import quat  # type: ignore
    from position_ctrl import PositionParams  # type: ignore
    from controller import AttitudeController, MixerPlant, gains_for_mode  # type: ignore
    from edf import EDFModel, RotorModel, VaneModel, vane_play, vane_play_deg_from_params  # type: ignore
    import tether as tt  # type: ignore
    from estimator import (GatedMahonyEstimator, Mpu6050, PadAlignGyroEstimator, gyro_bias_process,  # type: ignore
                           up_error_rad, white_noise_sigma)
    from fc_timing import (EPS_T, FCTiming, OutputLatch, frame_hz_from_timer, glink_latency_ticks,  # type: ignore
                           timer_of_channel)
    from ground_station_logic import GroundAssist, GroundParams  # type: ignore
    from link_model import COM_FORWARD_S, FC_REPLY_DELAY_S, LinkConfig, LinkModel, budget as link_budget, round_trip  # type: ignore
    from onboard import (TARGET_PHASES, FcuStack, OnboardInputs, OnboardSupervisor, State,  # type: ignore
                         SupervisorParams, UplinkFrame)
    from state_machine import VTOL_ALT_BIAS_TAU_S_ASSUMED, AltitudeFilter, VtolAltitudeFilter  # type: ignore
    from vehicle import G0, Vehicle  # type: ignore
    from vertical_ctrl import MODE_IDLE, TD_NAMES, VerticalParams  # type: ignore

ESTIMATORS = ("pad_gyro", "truth_noise", "mahony", "gated_mahony")
VTOL_ESTIMATOR = "gated_mahony"             # the v0.7.1 VTOL NAV estimator (params mission.vtol.nav_estimator)
PAD_STATES = (State.SAFE, State.ARMED)
VTOL_LOG_KEYS = ("phase", "h_tgt", "h_ref", "vz_cmd", "u_ff", "vc_integ", "vc_sat", "vc_mode", "td_count", "motor_on",
                 "baro_lost", "reason", "h_true", "gust_active", "imu_ok", "baro_ok", "surface_z")
# ---- v0.7.1 VTOL plant / sensor constants that params.json does not give (ASSUMED, each stated) ----
VTOL_ACCEL_VIBRATION_RMS_G_ASSUMED = 0.10   # accelerometer vibration at hover thrust, per axis, white at the NAV sample
                                            # rate after the MPU-6050 DLPF (DLPF_CFG 2: 94 Hz) - a stand-in for the fan's
                                            # ~450 Hz shaft tone aliased below the sample rate; scaled with T / (m g)
                                            # (imbalance force ~ rotor speed^2 ~ thrust).  Nothing measured: the campaign
                                            # spreads it 0..0.3 g (0.3 g = the reference study's stress value)
WIND_DIR_RATE_TAU_S_ASSUMED = 5.0           # random-walk wind direction: correlation time of its rate (first-order
                                            # Gauss-Markov), so the direction wanders smoothly instead of jumping


def vtol_round_trip(cfg: LinkConfig, gps: bool = False, power: bool = False, lbat: bool = False) -> dict[str, float]:
    """The VTOL build's ground round trip: ``sim.link_model.round_trip(cfg, mode='vtol')`` (TELEM v2, 39 B; v0.7.1 -
    the link model now sizes the reply itself, this name is kept for the callers); v0.8 ``gps``: TELEM v3, 45 B; v0.9
    RP1 ``power`` (the tether / hybrid builds): TELEM v4 41 B / with ``gps`` TELEM v5 47 B; 2026-11-10 ``lbat`` (the
    battery build with the logic-battery tail): TELEM v6 40 B / with ``gps`` TELEM v7 46 B."""
    if power:
        return round_trip(cfg, mode=lp.MODE_VTOL, gps=bool(gps), power=True)
    if lbat:
        return round_trip(cfg, mode=lp.MODE_VTOL, gps=bool(gps), lbat=True)
    if gps:
        return round_trip(cfg, mode=lp.MODE_VTOL, gps=True)
    return round_trip(cfg, mode=lp.MODE_VTOL)


def vtol_link_budget(cfg: LinkConfig, gps: bool = False, power: bool = False, lbat: bool = False) -> dict:
    """``sim.link_model.budget(cfg, mode='vtol')`` (TELEM v2 downlink frame; v0.8 ``gps``: TELEM v3; v0.9 ``power``:
    TELEM v4 / v5; 2026-11-10 ``lbat``: TELEM v6 / v7)."""
    if power:
        return link_budget(cfg, mode=lp.MODE_VTOL, gps=bool(gps), power=True)
    if lbat:
        return link_budget(cfg, mode=lp.MODE_VTOL, gps=bool(gps), lbat=True)
    if gps:
        return link_budget(cfg, mode=lp.MODE_VTOL, gps=True)
    return link_budget(cfg, mode=lp.MODE_VTOL)


# 2026-11-10: the logic battery the 6-DOF reports in the TELEM v6 / v7 tail (the plant has no logic pack; only the frame
# length matters to the run - the value is a mid-discharge 1S pack under the logic load, A)
SIM_LOGIC_BAT_V_ASSUMED = 3.85


def power_tail_link(sp: Any, src: str | None) -> bool:
    """v0.9 RP1: True when this run's supervisor replies with the power tail (TELEM v4 / v5): a VTOL supervisor with the
    power rules whose build is wired / wired_buffer - fixed (``sp.power``), or 'auto' resolved from the plant source
    ``src`` the way ``OnboardSupervisor._resolve_power`` resolves it from the plant's signals (tether -> wired, hybrid ->
    wired_buffer).  The ground's round trip / reply timeout is sized for that frame; the frame itself is chosen per reply
    by ``link_proto.with_power_tail`` from the reply's own power tail (its build bits)."""
    if not getattr(sp, "power_rules", False):
        return False
    if sp.power == "auto":
        return src is not None and pl.build_of_source(src) != pl.PWR_BATTERY
    return pl.build_of_source("battery" if sp.power == "battery" else sp.power) != pl.PWR_BATTERY


# --------------------------------------------------------------------------- #
# v0.7.1 VTOL aerodynamics (params aero.vtol_*): body cross-flow drag + EDF ram drag
# --------------------------------------------------------------------------- #
def body_radius_mm(L: Any, z_mm: float) -> float:
    """Outer radius of the body of revolution at height z above the exit plane (``sim.vehicle`` Layout): the vane
    section (r = vane_section_r) up to the propulsion cone, the cone from cone_r_bottom to cone_r_top, the body tube
    (body_r) to the nose base, then the nose profile; the fins are not part of it (``vtol_aero_geometry``)."""
    if z_mm <= L.cone_z0:
        return float(L.vane_section_r)
    if z_mm <= L.cone_z1:
        f = (z_mm - L.cone_z0) / (L.cone_z1 - L.cone_z0)
        return float(L.cone_r_bottom + f * (L.cone_r_top - L.cone_r_bottom))
    if getattr(L, "z_nose_base", None) is None:
        # v0.9 VTOL configuration: the body tube up to the flat top cap
        return float(L.body_r) if z_mm <= float(L.z_cap_top) else 0.0
    if z_mm <= L.z_nose_base:
        return float(L.body_r)
    if z_mm >= L.z_nose_tip:
        return 0.0
    return float(L.nose_outer_radius(z_mm - L.z_nose_base))      # cad.layout: x from the nose base


def vtol_aero_geometry(vehicle: Vehicle) -> dict[str, float]:
    """The numbers of the v0.7.1 VTOL drag model (params aero.vtol_aero_note), derived from params / ``sim.vehicle``:

    * A_plan = body.od x vehicle length (the note's definition); cross-checked against the side silhouette of the body
      of revolution (integral of 2 r(z) dz) + two fin planforms (0.5 (root + tip chord) x span each; seen from the
      side the 4 fins show 2 to 2.83 planforms - 2 is used for the check);
    * z_cf = centroid of that side silhouette (body + 2 fins) - where the cross-flow drag acts;
    * z_intake = mid-height of the intake windows (z_window_bottom .. z_window_top): the ingested air turns there, the
      EDF momentum (ram) drag mdot x V_perp acts there;
    * A_exit = the exhaust annulus (``Vehicle.a_e_m2``), mdot = sqrt(rho A_exit T) (actuator disk: T = mdot V_e,
      mdot = rho A_exit V_e), at hover T = m g;
    * the linear drag derivative at hover in calm air, NET of the vane counter-force (fix round 2026-10-21, the physics
      review): the ram drag acts at the intake l_in = z_intake - z_cg (0.108 m BELOW the CG); its moment l_in x D must
      be held by a vane side force D (-l_in) / l_v (vanes at l_v below the CG) that points ALONG the motion, so the net
      horizontal force is D (1 + l_in / l_v): dD/dv = mdot_hover (1 + l_in / l_v) = 0.682 mdot_hover.  The cross-flow
      drag is quadratic (derivative 0 in calm air, 2 kc v at v; its own counter-force factor 1 + l_cf / l_v ~ 0.99) - it
      only shortens the time constant in wind, so the calm-air value is the worst case the estimator must respect;
    * the drag time constant m / dD/dv (``drag_time_constant_hover_s``, 9.55 s - what the NAV estimator's 1 / kp must
      exceed and its Routh rule uses; ``drag_time_constant_ram_only_s`` = m / mdot_hover = 6.51 s is the v0.7.1 number
      without the counter-force)."""
    p = vehicle.params
    L = vehicle.layout
    a = p["aero"]
    rho = float(a["rho_kg_m3"])
    length_mm = float((getattr(L, "z_nose_tip", None) or L.z_cap_top) - L.z_exit)     # v0.9 VTOL: to the top cap
    od_mm = float(p["body"]["od"])
    a_plan = od_mm * length_mm * 1e-6
    n = 2000
    zs = L.z_exit + (np.arange(n) + 0.5) / n * length_mm
    rs = np.array([body_radius_mm(L, float(z)) for z in zs])
    dz = length_mm / n
    a_body = float((2.0 * rs).sum() * dz) * 1e-6
    m_body = float((2.0 * rs * zs).sum() * dz) * 1e-9          # first moment [m^3]
    span = float(L.fin_tip_r - L.fin_root_r)
    cr, ct = float(L.fin_root_chord), float(L.fin_tip_chord)
    a_fin = 0.5 * (cr + ct) * span * 1e-6
    # fin planform centroid height (trapezoid, root from fin_te_z to fin_root_le_z, tip from fin_te_z to fin_tip_le_z)
    z_fin = (float(L.fin_te_z) + (cr * cr + cr * ct + ct * ct) / (3.0 * (cr + ct))) * 1e-3
    a_sil = a_body + 2.0 * a_fin
    z_cf = (m_body + 2.0 * a_fin * z_fin) / a_sil
    z_in = 0.5 * (float(L.z_window_bottom) + float(L.z_window_top)) * 1e-3
    a_exit = float(vehicle.a_e_m2)
    w = vehicle.full.mass_kg * G0
    mdot_h = math.sqrt(rho * a_exit * w)
    cdc = float(a["vtol_crossflow_cd_ASSUMED"])
    l_v = float(vehicle.l_v_m)
    l_in = z_in - vehicle.z_cg_m
    l_cf = z_cf - vehicle.z_cg_m
    f_in = 1.0 + l_in / l_v                     # net / ram drag after the vane counter-force (0.682)
    f_cf = 1.0 + l_cf / l_v
    kc = 0.5 * rho * cdc * a_plan
    ram = bool(a.get("vtol_ram_drag", True))
    d_eff = mdot_h * f_in if ram else 0.0
    m = vehicle.full.mass_kg
    return {"rho_kg_m3": rho, "length_m": length_mm * 1e-3, "od_m": od_mm * 1e-3, "a_plan_m2": a_plan,
            "a_silhouette_body_m2": a_body, "a_fin_planform_m2": a_fin, "a_silhouette_body_2fins_m2": a_sil,
            "z_crossflow_m": z_cf, "z_crossflow_body_only_m": m_body / a_body, "z_fin_centroid_m": z_fin,
            "z_intake_m": z_in, "z_window_bottom_m": float(L.z_window_bottom) * 1e-3,
            "z_window_top_m": float(L.z_window_top) * 1e-3, "z_cg_m": vehicle.z_cg_m, "a_exit_m2": a_exit,
            "crossflow_cd_ASSUMED": cdc, "ram_drag": ram,
            "mdot_hover_kg_s": mdot_h, "exit_speed_hover_mps": w / mdot_h,
            "crossflow_coeff_N_per_mps2": kc, "l_vane_m": l_v,
            "vane_counterforce_factor_ram": f_in, "vane_counterforce_factor_crossflow": f_cf,
            "ram_drag_derivative_hover_N_per_mps": mdot_h,
            "drag_derivative_hover_N_per_mps": d_eff,
            "drag_time_constant_hover_s": m / d_eff if d_eff > 0.0 else float("inf"),
            "drag_time_constant_ram_only_s": m / mdot_h,
            "crossflow_derivative_at_1mps_N_per_mps": 2.0 * kc * 1.0 * f_cf,
            "drag_time_constant_at_1mps_s": m / (d_eff + 2.0 * kc * f_cf),
            "l_crossflow_above_cg_m": l_cf, "l_intake_above_cg_m": l_in}


# --------------------------------------------------------------------------- #
# v0.7.1 hand pushes (the live demo: a person pushes the hovering vehicle with a padded pole)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Push:
    """One push on the vehicle (plant only; nothing on the vehicle knows about it).  The force is horizontal in the
    world (+ an optional vertical component), fixed in direction for the push, with a half-sine or trapezoid profile;
    it acts at a point of the body surface: the side facing the person (opposite to the force direction) at height
    ``z_body_m`` above the exit plane, turned ``offset_deg`` around the body axis (a tangential offset: the force then
    has a lever arm about the axis - a yaw torque, which needs contact friction >= tan(offset_deg)).  The contact point
    is re-evaluated from the current attitude (the padded pole tip slides on the round body: it stays on the side facing
    the person; a point fixed on the body would add an unphysical yaw lever arm that grows with the body's own spin)."""

    t0_s: float                     # start, absolute sim time
    duration_s: float
    force_n: float                  # peak horizontal force
    azimuth_deg: float = 0.0        # direction the force pushes toward (world, from +x toward +y)
    z_body_m: float = 0.45          # contact height on the body above the exit plane
    offset_deg: float = 0.0         # contact point turned around the body axis (yaw torque)
    profile: str = "half_sine"      # "half_sine" | "trapezoid"
    rise_s: float = 0.05            # trapezoid: ramp time at both ends
    vertical_n: float = 0.0         # peak vertical component (up +)
    # v0.8 pole model (task: pole pushes act along the pole): None = the horizontal model above.  Otherwise the person
    # holds the pole at this height above the ground the vehicle would land on (``FlightSim.ground_z``), the pole of
    # ``pole_length_m`` runs from the hand to the contact point (azimuth: toward ``azimuth_deg``), and ``force_n`` acts
    # ALONG the pole (fixed at the push start: up when the hand is below the contact point); ``vertical_n`` is then
    # ignored.  An angled pole needs contact friction >= tan(elevation) on the round body (not enforced, stated).
    hand_height_m: float | None = None
    pole_length_m: float = 2.5      # params demo_rules: a padded pole >= 2.5 m

    def __post_init__(self) -> None:
        if self.profile not in ("half_sine", "trapezoid"):
            raise ValueError(f"push profile {self.profile!r} not in ('half_sine', 'trapezoid')")
        if not (self.duration_s > 0.0 and self.force_n >= 0.0):
            raise ValueError("push: need duration_s > 0 and force_n >= 0")
        if self.hand_height_m is not None and not self.pole_length_m > 0.0:
            raise ValueError("push: the pole model needs pole_length_m > 0")

    @property
    def pole(self) -> bool:
        return self.hand_height_m is not None

    @property
    def t1_s(self) -> float:
        return self.t0_s + self.duration_s

    def factor(self, t: float) -> float:
        """Profile 0..1 at time t (0 outside [t0, t1))."""
        if not (self.t0_s <= t < self.t0_s + self.duration_s):
            return 0.0
        s = t - self.t0_s
        if self.profile == "half_sine":
            return math.sin(math.pi * s / self.duration_s)
        r = min(self.rise_s, 0.5 * self.duration_s)
        if r <= 0.0:
            return 1.0
        e = min(s, self.duration_s - s)
        return 1.0 if e >= r else e / r

    @property
    def impulse_ns(self) -> float:
        """Horizontal impulse = integral of the force over the push [N s]."""
        if self.profile == "half_sine":
            return self.force_n * self.duration_s * 2.0 / math.pi
        r = min(self.rise_s, 0.5 * self.duration_s)
        return self.force_n * (self.duration_s - r)

    def force_world(self, t: float) -> tuple[float, float, float]:
        f = self.factor(t)
        if f <= 0.0:
            return 0.0, 0.0, 0.0
        a = math.radians(self.azimuth_deg)
        return f * self.force_n * math.cos(a), f * self.force_n * math.sin(a), f * self.vertical_n


class LaggedBaroCompFilter(VtolAltitudeFilter):
    """WHAT-IF study class (not the firmware reference): ``VtolAltitudeFilter`` whose baro thrust compensation uses the
    thrust ratio (u / u_hover)^n passed through the EDF's first-order lag (edf.thrust_time_constant_s) and delayed by the
    baro latency - the thrust that set the static-pressure error of the sample being corrected - instead of the FCU's
    current throttle.  Scratch study v071/vtolsim/baro_comp_study.py: with comp = offset = 1.0 m the reference's
    compensation sat ~0.7 m low with 8 s of throttle saturation in the demo hover, this one tracked like no offset."""

    def setup_lag(self, dt: float, tau_s: float, latency_s: float) -> None:
        self.a_lag = 1.0 - math.exp(-dt / tau_s) if tau_s > 0.0 else 1.0
        self.r_lag = 0.0
        n = max(1, int(round(latency_s / dt)))
        self.r_buf: deque[float] = deque([0.0] * n, maxlen=n)

    def step(self, a_z_world: float, dt: float, h_baro: float | None, t: float, pad: bool, u_prev: float = 0.0) -> None:
        r_cmd = 0.0 if not (u_prev == u_prev) or u_prev <= 0.0 else (min(u_prev, 1.0) * self.inv_u_hover) ** self.throttle_exp
        self.r_lag += self.a_lag * (r_cmd - self.r_lag)
        self.r_buf.append(self.r_lag)
        super().step(a_z_world, dt, h_baro, t, pad, u_prev)

    def baro_compensation(self, u_prev: float) -> float:
        return self.comp_m * self.r_buf[0] if self.comp_m != 0.0 else 0.0


def push_with_impulse(impulse_ns: float, duration_s: float, **kw: Any) -> Push:
    """A half-sine push of the given horizontal impulse and duration (force = pi I / (2 T))."""
    return Push(duration_s=duration_s, force_n=math.pi * impulse_ns / (2.0 * duration_s), **kw)


# --------------------------------------------------------------------------- #
# v0.8 GPS receiver (plant side).  2026-11-10: u-blox NEO-7M class by default (the part on the vehicle); the NEO-6M
# model of v0.8 .. v0.9 = neo_6m(), the NEO-M8N upgrade path = neo_m8n()
# --------------------------------------------------------------------------- #
GPS_SITE_LAT_E7_ASSUMED = 365000000       # the field's latitude (36.5 N, central Korea): only the metres per 1e-7 deg
GPS_SITE_LON_E7_ASSUMED = 1275000000      # (127.5 E)
GPS_SITE_HMSL_MM_ASSUMED = 50000          # 50 m above sea level (only in the messages; the NAV does not use height)
GPS_WEEK_ASSUMED = 2440                   # GPS week in NAV-SOL (not used by the NAV)
GPS_TOW0_MS_ASSUMED = 259_200_000         # GPS time of week at sim t = 0 before the per-seed offset (0 .. 600 s)
GPS_CFG_RESPONSE_S_ASSUMED = 0.03         # the receiver's ACK / MON-VER answer starts this long after the request's last
                                          # byte (u-blox: ~10-100 ms; the NAV waits CFG_ACK_TIMEOUT_S 0.3 s)
GPS_NO_FIX_HACC_MM = 4_294_967            # hAcc / vAcc reported without a fix (u-blox reports kilometres) ASSUMED
GPS_NMEA_BYTES_PER_EPOCH_ASSUMED = 420    # factory output (GGA, GLL, GSA, 2 GSV, RMC, VTG) ~420 B per 1 Hz epoch
GPS_UART_BITS_PER_BYTE = 10               # 8N1
GPS_HW = {"NEO-6M": (gn.HW_UBLOX6, "7.03 (45969)"), "NEO-M8N": (gn.HW_UBLOX8, "ROM CORE 3.01 (107888)"),
          # u-blox 7: hwVersion 00070000, firmware 1.00 (59842) - ASSUMED (the usual u-blox 7 ROM string; the NAV only
          # reports the module generation, sim.gps_nav.MonVer.module)
          "NEO-7M": (gn.HW_UBLOX7, "1.00 (59842)")}
GPS_NAV_MSGS = ((gn.CLS_NAV, gn.ID_NAV_POSLLH, "p"), (gn.CLS_NAV, gn.ID_NAV_SOL, "s"), (gn.CLS_NAV, gn.ID_NAV_VELNED, "v"))


@dataclass(frozen=True)
class GpsSim:
    """The receiver model of one run (plant side) and the set-up the scenario flies with.  Sources: DS = data sheet
    (u-blox NEO-6 GPS.G6-HW-09005-E; NEO-7 GPS.G7-HW-11004; NEO-M8 UBX-15031086), AP = ArduPilot AP_GPS_UBLOX,
    ASSUMED = no source (the review's quote applies: station keeping ~1 m RMS / 2.5 m max until a real module is
    measured on the pod).  Windows are absolute sim times [t0, t1).

    2026-11-10 (control round, plant change): the default is the NEO-7M class the vehicle carries (was the NEO-6M:
    ``neo_6m()``).  What changes against the NEO-6M model: the MON-VER answer (u-blox 7), the rate limit (10 Hz; the NAV
    still asks for params rate_hz = 5 Hz) and the latency (0.12 s instead of 0.22 s).  The data sheets are not in the
    repository and no NEO-7M has been measured: every u-blox 7 number here is ASSUMED.  The accuracy figures are the
    same in both data sheets (2.5 m CEP, 0.1 m/s), so the error model is unchanged.  The NAV keeps compensating
    params ``flight_computer.nav.gps.latency_s_ASSUMED`` (0.22 s) until the user decides otherwise: the 0.10 s
    difference is inside the campaign's latency spread (``sim.vtol_campaign.SAMPLING_GPS``)."""

    receiver: str = "NEO-7M"                # "NEO-7M" | "NEO-6M" | "NEO-M8N" (MON-VER hwVersion, the rate limit)
    rate_hz_max: float = 10.0               # DS NEO-7 max navigation rate 10 Hz ASSUMED (NEO-6M: 5 Hz; M8N: 10 Hz single
                                            # GNSS); CFG-RATE above it: NAK
    latency_s: float = 0.12                 # measurement -> last byte of the epoch at the NAV: AP lag of the u-blox 7 / M8
                                            # generations (u-blox 6: 0.22) - ASSUMED (no data sheet value, not measured);
                                            # the NAV assumes params latency_s_ASSUMED
    latency_jitter_s: float = 0.0           # extra delivery delay U(0, j) per epoch ASSUMED (review stress 0.1 s)
    pos_gm_sigma_m: float = 2.12            # DS 2.5 m CEP (24 h static) -> 2.12 m 1-sigma per axis; the same at minute scale
                                            # ASSUMED
    pos_gm_tau_s: float = 200.0             # position wander correlation time ASSUMED (no u-blox figure; 100-300 s)
    pos_white_m: float = 0.2                # position white noise per axis ASSUMED
    step_rate_hz: float = 1.0 / 120.0       # satellite-set changes (Poisson) ASSUMED ...
    step_m: tuple = (0.5, 2.0)              # ... their size (uniform, random direction; decay with the wander's tau)
    vel_white_mps: float = 0.10             # DS 0.1 m/s CEP (0.085 per axis) inflated for the hover's dynamics ASSUMED
    vel_gm_sigma_mps: float = 0.02          # slowly correlated velocity error ASSUMED ...
    vel_gm_tau_s: float = 10.0              # ... its correlation time ASSUMED
    hacc_m: float = 2.5                     # hAcc reported in a good fix ASSUMED (u-blox reports ~ the CEP)
    sacc_mps: float = 0.30                  # sAcc reported ASSUMED (conservative, as u-blox)
    num_sv: int = 9                         # satellites used with an open sky ASSUMED
    sky_blocked: float = 0.0                # fraction of the sky blocked: satellites x (1 - f), the satellite-geometry
                                            # errors (wander, white, steps, velocity) x sqrt(n_open / n) - ASSUMED DOP-like
                                            # scaling; fewer than 4 satellites: no fix
    ttff_s: float = 1.0                     # DS hot start 1 s (backup cell, powered within ~4 h); cold / warm 27 s
    backup_config: bool = False             # the backup cell kept the configuration (38400, rate, UBX on): the NAV's 38400
                                            # probe is acknowledged; False: factory 9600 baud, 1 Hz NMEA (the NEO-6M has no
                                            # flash - params electronics.gps)
    nak_ids: tuple = ()                     # CFG ids the receiver answers with ACK-NAK (fault study)
    outages: tuple = ()                     # (t0, t1): no fix (sky blocked, jamming) - the messages still flow, gpsFix 0
    silent: tuple = ()                      # (t0, t1): no bytes at all (a lead / connector fault)
    jumps: tuple = ()                       # (t0, t1, dx_m, dy_m): multipath position offset (velocity clean)
    ramps: tuple = ()                       # (t0, t1, vx_mps, vy_mps): an offset growing at this rate from t0 (velocity clean)
    degrade: tuple = ()                     # (t0, t1, hacc_m, num_sv, error_scale): reported quality + errors x scale
    vel_bias: tuple = ()                    # (t0, t1, bvx_mps, bvy_mps): a Doppler velocity bias (position clean) - the
                                            # GPS-IMU disagreement the plausibility rule V13 must catch (fault study)
    multipath_rate_hz: float = 0.0          # random multipath position jumps (Poisson, velocity clean) ASSUMED ...
    multipath_m: tuple = (1.0, 4.0)         # ... size (uniform, random direction) ...
    multipath_s: tuple = (2.0, 10.0)        # ... duration
    heading_err_deg: float = 0.0            # pad set-up error: the vehicle's +X (vane 1) points this far (counter-clockwise
                                            # from above) from the pad azimuth; true initial yaw = -azimuth + this
    pad_azimuth_deg: float | None = None    # None: params position_hold.heading.pad_azimuth_deg (NAV and set-up target)
    nav: dict | None = None                 # ``sim.gps_nav.NavGpsParams`` overrides (what-ifs; the M8N build)
    seed_salt: int = 0x6F5                  # own random streams [seed, salt(, k)] - the v0.7.1 draws stay untouched
    chain: str | None = None                # v0.8.1 flight-computer chain: "gns" = four chips (the GNS board runs the
                                            # filter + position controller, sim.glink_proto) | "fcu" = the v0.8 three chips;
                                            # None: params (flight_computer.glink present -> "gns")
    glink_latency_ticks: int | None = None  # "gns": NAV ticks from the STATE sample to the frame carrying the TILT part;
                                            # None: params glink.latency_ticks_ASSUMED (3)
    glink_cut: tuple = ()                   # "gns": (t0, t1) windows with the glink wire cut (no byte either way): the
                                            # GNS's gap rules and the FCU's TRIM fallback / own V11 (fault study)

    def __post_init__(self) -> None:
        if self.receiver not in GPS_HW:
            raise ValueError(f"GPS receiver {self.receiver!r} not in {sorted(GPS_HW)}")
        if self.chain not in (None, "fcu", "gns"):
            raise ValueError(f"GpsSim.chain {self.chain!r} not in (None, 'fcu', 'gns')")
        if not (0.0 <= self.sky_blocked < 1.0 and self.latency_s >= 0.0 and self.latency_jitter_s >= 0.0):
            raise ValueError("GpsSim: need 0 <= sky_blocked < 1, latency_s >= 0, latency_jitter_s >= 0")


def neo_6m(**kw: Any) -> GpsSim:
    """The NEO-6M model of v0.8 .. v0.9 (GY-NEO6MV2; the default until 2026-11-10): DS 5 Hz, AP u-blox 6 lag 0.22 s
    (= the latency the NAV compensates, params latency_s_ASSUMED).  Kept for the before / after of that change."""
    d: dict[str, Any] = dict(receiver="NEO-6M", rate_hz_max=5.0, latency_s=0.22)
    d.update(kw)
    return GpsSim(**d)


def neo_m8n(**kw: Any) -> GpsSim:
    """The upgrade path (u-blox NEO-M8N): DS 10 Hz single GNSS, 2.5 m CEP, 0.05 m/s velocity (0.042 per axis); AP lag
    0.12 s.  The NAV is built for it (CFG-RATE 10 Hz, latency 0.12 s): the ``nav`` overrides."""
    d: dict[str, Any] = dict(receiver="NEO-M8N", rate_hz_max=10.0, latency_s=0.12, vel_white_mps=0.05,
                             nav={"rate_hz": 10.0, "latency_s": 0.12})
    d.update(kw)
    return GpsSim(**d)


def _in_window(t: float, wins: tuple) -> Any:
    for w in wins:
        if float(w[0]) <= t < float(w[1]):
            return w
    return None


class GpsReceiverPlant:
    """UART-level u-blox receiver (``GpsSim``).  The NAV side calls, once per NAV tick at time t: ``step(t, truth)``
    (epochs measured up to t: the TRUE antenna position / velocity of this tick; the bytes of their messages are queued
    with their UART times), ``deliver(t, nav_baud)`` (the bytes that have arrived by t; bytes sent at a baud rate the
    NAV's UART is not set to are lost as framing errors) and ``nav_tx(t, data, nav_baud)`` (the NAV's configuration
    bytes; answered after ``GPS_CFG_RESPONSE_S_ASSUMED``).  Configuration state: baud, outProto (NMEA on / off),
    measurement period, the enabled NAV messages, dynamic model.  Nothing here reaches the vehicle except bytes."""

    def __init__(self, cfg: GpsSim, seed: int, t_power: float):
        c = self.c = cfg
        self.rng = np.random.default_rng([int(seed), int(cfg.seed_salt)])
        self.rng_mp = np.random.default_rng([int(seed), int(cfg.seed_salt), 0x3A])
        self.rng_j = np.random.default_rng([int(seed), int(cfg.seed_salt), 77])
        self.frame = gn.LocalFrame.at(GPS_SITE_LAT_E7_ASSUMED, GPS_SITE_LON_E7_ASSUMED)
        self.t_power = float(t_power)
        self.tow0 = GPS_TOW0_MS_ASSUMED + int(self.rng.integers(0, 600_000))
        self.hw, self.sw = GPS_HW[c.receiver]
        if c.backup_config:
            self.baud, self.meas_ms, self.nmea = gn.GPS_BAUD, 200, False
            self.enabled = {(a, b) for a, b, _ in GPS_NAV_MSGS}
            self.dyn = gn.DYN_AIRBORNE_1G
        else:
            self.baud, self.meas_ms, self.nmea = gn.GPS_BAUD_DEFAULT, 1000, True
            self.enabled: set[tuple[int, int]] = set()
            self.dyn = 0
        self.parser = gn.UbxParser()
        self.tx_free = self.t_power
        self.chunks: deque[list] = deque()          # [t_start, baud, bytes, n_delivered]
        self.next_tow = self._next_tow(self.t_power)
        self.t_last_epoch: float | None = None
        n0 = max(int(c.num_sv), 0)
        self.n_open = n0
        self.n_sky = int(round(n0 * (1.0 - float(c.sky_blocked))))
        self.sky_scale = math.sqrt(n0 / max(self.n_sky, 4)) if n0 > 0 else 1.0
        self.gm_p = self.rng.normal(0.0, c.pos_gm_sigma_m, 2) if c.pos_gm_sigma_m > 0 else np.zeros(2)
        self.step_off = np.zeros(2)
        self.gm_v = self.rng.normal(0.0, c.vel_gm_sigma_mps, 2) if c.vel_gm_sigma_mps > 0 else np.zeros(2)
        self.mp_events: list[tuple[float, float, float, float]] = []     # random multipath (t0, t1, dx, dy)
        self.t_mp_next = (self.t_power + float(self.rng_mp.exponential(1.0 / c.multipath_rate_hz))
                          if c.multipath_rate_hz > 0 else math.inf)
        self.last: dict[str, Any] = {"fix": False, "num_sv": 0, "hacc_m": math.nan, "err_p": (math.nan, math.nan),
                                     "err_v": (math.nan, math.nan), "err_slow": (math.nan, math.nan), "t_e": math.nan}
        self.stats = {"epochs": 0, "epochs_fix": 0, "bytes_tx": 0, "bytes_lost_baud": 0, "bytes_lost_silent": 0,
                      "cfg_rx": 0, "cfg_naks": 0, "cfg_ignored_baud": 0, "mon_ver": 0, "multipath_events": 0}
        self.log: list[str] = []

    # ---- GPS time ----
    def tow_ms(self, t: float) -> float:
        return self.tow0 + 1000.0 * t

    def t_of_tow(self, tow: float) -> float:
        return (tow - self.tow0) / 1000.0

    def _next_tow(self, t: float) -> int:
        """First measurement epoch (a multiple of the measurement period in GPS time) at or after t."""
        tw = self.tow_ms(t)
        return int(math.ceil(tw / self.meas_ms - 1e-9)) * self.meas_ms

    # ---- UART ----
    def _send(self, t: float, data: bytes) -> None:
        """Queue bytes on the receiver's TX line at its current baud, starting no earlier than t (serial)."""
        if not data:
            return
        t0 = max(float(t), self.tx_free)
        self.chunks.append([t0, self.baud, bytes(data), 0])
        self.tx_free = t0 + len(data) * GPS_UART_BITS_PER_BYTE / self.baud
        self.stats["bytes_tx"] += len(data)

    def deliver(self, t: float, nav_baud: int) -> bytes:
        """Bytes whose last bit arrived by t; a byte sent inside a ``silent`` window or at another baud is lost."""
        out = bytearray()
        while self.chunks:
            ch = self.chunks[0]
            t0, baud, data, n = ch
            dt_b = GPS_UART_BITS_PER_BYTE / baud
            m = min(len(data), int(math.floor((t - t0) / dt_b + 1e-9)))
            if m > n:
                for i in range(n, m):
                    tb = t0 + (i + 1) * dt_b
                    if self.c.silent and _in_window(tb, self.c.silent) is not None:
                        self.stats["bytes_lost_silent"] += 1
                    elif baud != nav_baud:
                        self.stats["bytes_lost_baud"] += 1
                    else:
                        out.append(data[i])
                ch[3] = m
            if m >= len(data):
                self.chunks.popleft()
            else:
                break
        return bytes(out)

    def nav_tx(self, t: float, data: bytes, nav_baud: int) -> None:
        """The NAV sent ``data`` at ``nav_baud`` starting at t: parsed when its last byte has arrived; CFG frames are
        acknowledged (ACK-NAK for ``nak_ids`` and for a rate above ``rate_hz_max``) and applied after the ACK (CFG-PRT's
        new baud after the ACK left), MON-VER polls answered."""
        t_done = float(t) + len(data) * GPS_UART_BITS_PER_BYTE / nav_baud
        if self.c.silent and _in_window(t_done, self.c.silent) is not None:
            return
        if nav_baud != self.baud:
            self.stats["cfg_ignored_baud"] += 1           # framing errors at the receiver: nothing parsed
            return
        t_ans = t_done + GPS_CFG_RESPONSE_S_ASSUMED
        for m in self.parser.feed(data):
            if m.cls == gn.CLS_CFG:
                self.stats["cfg_rx"] += 1
                ok = m.id not in self.c.nak_ids
                new_baud = None
                if ok and m.id == gn.ID_CFG_RATE and len(m.payload) >= 2:
                    meas = struct.unpack("<H", m.payload[:2])[0]
                    ok = meas > 0 and 1000.0 / meas <= self.c.rate_hz_max + 1e-9
                    if ok:
                        self.meas_ms = int(meas)
                        self.next_tow = self._next_tow(t_ans)
                if ok and m.id == gn.ID_CFG_PRT and len(m.payload) == gn.LEN_CFG_PRT:
                    f = struct.unpack(gn.CFG_PRT_FMT, m.payload)
                    new_baud = int(f[4])
                    self.nmea = bool(f[6] & gn.PROTO_NMEA)
                if ok and m.id == gn.ID_CFG_MSG and len(m.payload) == 3:
                    cls_, id_, rate = m.payload[0], m.payload[1], m.payload[2]
                    if rate:
                        self.enabled.add((cls_, id_))
                    else:
                        self.enabled.discard((cls_, id_))
                if ok and m.id == gn.ID_CFG_NAV5 and len(m.payload) == gn.LEN_CFG_NAV5:
                    self.dyn = int(struct.unpack(gn.CFG_NAV5_FMT, m.payload)[1])
                if not ok:
                    self.stats["cfg_naks"] += 1
                self._send(t_ans, gn.ubx_frame(gn.CLS_ACK, gn.ID_ACK_ACK if ok else gn.ID_ACK_NAK, bytes([m.cls, m.id])))
                if new_baud is not None:
                    self.baud = new_baud                     # after its own ACK (queued at the old baud above)
                    self.tx_free = max(self.tx_free, t_ans)
            elif (m.cls, m.id) == (gn.CLS_MON, gn.ID_MON_VER) and m.length == 0:
                self.stats["mon_ver"] += 1
                self._send(t_ans, gn.ubx_frame(gn.CLS_MON, gn.ID_MON_VER, gn.mon_ver_payload(self.sw, self.hw)))

    # ---- measurements ----
    def step(self, t: float, p_ant: tuple, v_ant: tuple) -> None:
        """Every measurement epoch up to t (once per NAV tick): the TRUE antenna position (x, y, z) / velocity of this
        tick plus the error model; its messages queued so that the last byte arrives ``latency_s`` (+ jitter) after the
        measurement (or as soon as the line is free)."""
        c = self.c
        while self.t_of_tow(self.next_tow) <= t + 1e-12:
            tow = self.next_tow
            self.next_tow = tow + self.meas_ms
            te = self.t_of_tow(tow)
            dt = te - self.t_last_epoch if self.t_last_epoch is not None else self.meas_ms / 1000.0
            self.t_last_epoch = te
            self.stats["epochs"] += 1
            # error processes (also without a fix)
            if c.pos_gm_tau_s > 0 and c.pos_gm_sigma_m > 0:
                a = math.exp(-dt / c.pos_gm_tau_s)
                self.gm_p = a * self.gm_p + math.sqrt(1.0 - a * a) * c.pos_gm_sigma_m * self.rng.normal(size=2)
                self.step_off = a * self.step_off
            if c.step_rate_hz > 0 and self.rng.uniform() < c.step_rate_hz * dt:
                mag = float(self.rng.uniform(*c.step_m))
                ang = float(self.rng.uniform(0.0, 2.0 * math.pi))
                self.step_off = self.step_off + mag * np.array([math.cos(ang), math.sin(ang)])
            if c.vel_gm_tau_s > 0 and c.vel_gm_sigma_mps > 0:
                a = math.exp(-dt / c.vel_gm_tau_s)
                self.gm_v = a * self.gm_v + math.sqrt(1.0 - a * a) * c.vel_gm_sigma_mps * self.rng.normal(size=2)
            wp = self.rng.normal(0.0, c.pos_white_m, 2) if c.pos_white_m > 0 else np.zeros(2)
            wv = self.rng.normal(0.0, c.vel_white_mps, 2) if c.vel_white_mps > 0 else np.zeros(2)
            while te >= self.t_mp_next:                     # random multipath events
                t0 = self.t_mp_next
                mag = float(self.rng_mp.uniform(*c.multipath_m))
                ang = float(self.rng_mp.uniform(0.0, 2.0 * math.pi))
                self.mp_events.append((t0, t0 + float(self.rng_mp.uniform(*c.multipath_s)), mag * math.cos(ang),
                                       mag * math.sin(ang)))
                self.stats["multipath_events"] += 1
                self.t_mp_next = t0 + float(self.rng_mp.exponential(1.0 / c.multipath_rate_hz))
            jit = float(self.rng_j.uniform(0.0, c.latency_jitter_s)) if c.latency_jitter_s > 0 else 0.0
            # quality of this epoch
            nsv, scale, hacc = self.n_sky, self.sky_scale, c.hacc_m * self.sky_scale
            d = _in_window(te, c.degrade)
            if d is not None:
                hacc, nsv = float(d[2]), int(d[3])
                scale *= float(d[4]) if len(d) > 4 else 1.0
            fix = (te >= self.t_power + c.ttff_s and _in_window(te, c.outages) is None and nsv >= 4)
            ep = scale * (self.gm_p + self.step_off + wp)
            for w in tuple(c.jumps) + tuple(self.mp_events):
                if float(w[0]) <= te < float(w[1]):
                    ep = ep + np.array([float(w[2]), float(w[3])])
            for w in c.ramps:
                if float(w[0]) <= te < float(w[1]):
                    ep = ep + (te - float(w[0])) * np.array([float(w[2]), float(w[3])])
            ev = scale * (self.gm_v + wv)
            for w in c.vel_bias:
                if float(w[0]) <= te < float(w[1]):
                    ev = ev + np.array([float(w[2]), float(w[3])])
            if fix:
                self.stats["epochs_fix"] += 1
                lat, lon = self.frame.to_llh(float(p_ant[0] + ep[0]), float(p_ant[1] + ep[1]))
                vn = lp.round_half_away(100.0 * float(v_ant[0] + ev[0]))
                ve = lp.round_half_away(-100.0 * float(v_ant[1] + ev[1]))
                vd = lp.round_half_away(-100.0 * float(v_ant[2]))
                hm = int(round(1000.0 * hacc * (1.0 + 0.05 * float(self.rng.normal()))))
                ep_ = gn.GpsEpoch(itow_ms=int(tow) % 604_800_000, lat_e7=lat, lon_e7=lon,
                                  hmsl_mm=GPS_SITE_HMSL_MM_ASSUMED + lp.round_half_away(1000.0 * float(p_ant[2])),
                                  hacc_mm=max(hm, 100), vacc_mm=max(int(1.5 * hm), 150), vel_n_cms=vn, vel_e_cms=ve,
                                  vel_d_cms=vd, sacc_cms=lp.round_half_away(100.0 * c.sacc_mps * scale), gps_fix=gn.FIX_3D,
                                  fix_ok=True, num_sv=nsv, pacc_cm=max(hm // 10 * 2, 10))
            else:
                ep_ = gn.GpsEpoch(itow_ms=int(tow) % 604_800_000, lat_e7=0, lon_e7=0, hmsl_mm=0, hacc_mm=GPS_NO_FIX_HACC_MM,
                                  vacc_mm=GPS_NO_FIX_HACC_MM, vel_n_cms=0, vel_e_cms=0, vel_d_cms=0, sacc_cms=2000,
                                  gps_fix=0, fix_ok=False, num_sv=min(nsv, 3), pacc_cm=GPS_NO_FIX_HACC_MM // 10)
            self.last = {"fix": fix, "num_sv": ep_.num_sv, "hacc_m": ep_.hacc_mm / 1000.0, "t_e": te,
                         "gps_fix": ep_.gps_fix,
                         "err_p": (float(ep[0]), float(ep[1])) if fix else (math.nan, math.nan),
                         "err_v": (float(ev[0]), float(ev[1])) if fix else (math.nan, math.nan),
                         "err_slow": (float(scale * (self.gm_p[0] + self.step_off[0])),
                                      float(scale * (self.gm_p[1] + self.step_off[1]))) if fix else (math.nan, math.nan)}
            order = "".join(ch for a, b, ch in GPS_NAV_MSGS if (a, b) in self.enabled)
            data = gn.epoch_frames(ep_, GPS_WEEK_ASSUMED, order) if order else b""
            if self.nmea and self.meas_ms >= 1000:
                data = self._nmea(tow) + data
            if data:
                t_tx = len(data) * GPS_UART_BITS_PER_BYTE / self.baud
                self._send(max(t, te + c.latency_s + jit - t_tx), data)

    def _nmea(self, tow: float) -> bytes:
        """Factory NMEA output (text the UBX parser skips; length GPS_NMEA_BYTES_PER_EPOCH_ASSUMED)."""
        s = f"$GPTXT,01,01,02,tow {int(tow)} factory NMEA output (simulated filler)*00\r\n".encode("ascii")
        n = GPS_NMEA_BYTES_PER_EPOCH_ASSUMED
        return (s * (n // len(s) + 1))[:n]


# --------------------------------------------------------------------------- #
# configuration
# --------------------------------------------------------------------------- #
# Barometer models (SensorConfig fields).  2026-11-10 control round: the NAV's barometer is the Bosch BMP280 in forced
# mode, pressure oversampling x4, IIR filter off (firmware hal/bmp280.h); before: the MS5611 (GY-63).
#   BMP280   noise: datasheet (BST-BMP280-DS001) noise table, "standard resolution" (x4), IIR off: 2.1 Pa RMS - ASSUMED
#            (the datasheet is not in the repository) = 0.175 m at sea level.  The MS5611 model's 0.25 m (task brief)
#            is 2.5 x its datasheet figure (0.012 mbar at OSR 4096 = 0.10 m, ASSUMED); the same margin is kept:
#            0.175 m x 2.5 = 0.44 m (ESTIMATE; the field value needs a bench measurement of the module).
#            steps: x4 oversampling = 18 bit = 0.66 Pa per LSB (datasheet resolution table, ASSUMED) = 0.055 m; the
#            firmware's Q24.8 Pa result and its float altitude are finer.
#            lag:   forced mode + IIR off: no filter state in the sensor.  The driver publishes a sample 16.75 ms after
#            the middle of its pressure sampling (hal/bmp280.h BMP280_LATENCY_S; MS5611: 15.9 ms); the model keeps the
#            0.1 s of SPEC section 9 for both (conservative, unchanged).
#   MS5611   the v0.5 .. v0.9 model: ``replace(sens, **BARO_MODELS["MS5611"])``; bit-identical to those versions
#            together with the altitude-filter crossover they flew, ``**LEGACY_SENSOR_SET`` (below).
# Altitude-filter retune that goes with the BMP280 model (2026-11-10, SensorConfig.altitude_crossover_hz 0.5 ->
# 0.4 Hz; the one value feeds the ballistic AltitudeFilter and the VTOL filter, NAV firmware ALT_CROSSOVER_HZ /
# ALTV_K1..3).  Why it was needed: with 0.44 m of barometer noise the 0.5 Hz filter carries enough noise in v_z and
# the accelerometer-bias state that the baro-lost landing (dead reckoning from the state at the loss) no longer
# reached the ground in 2 of 8 seeds (sim/test_vtol_scenarios.py test_baro_lost_landing_reaches_the_ground, seeds
# 70 .. 77: 6 / 8 at 0.5 Hz, 8 / 8 at 0.4 Hz - MEASURED).  Chosen on TUNE 7400 and the deterministic scenarios only
# (PASS of 156 field samples: MS5611 model 20, BMP280 + NEO-7M at 0.5 Hz 14, at 0.4 Hz 18, at 0.35 Hz 16 - MEASURED;
# differences of +-2 are threshold jitter).  0.4 Hz keeps the take-off settle rule without an FCU change:
# baro latency 0.1 s + 1 / (2 pi 0.4 Hz) = 0.498 s <= sim/vertical_ctrl.py TAKEOFF_SETTLE_S_ASSUMED 0.5 s; a lower
# crossover would not (0.35 Hz: 0.555 s).
BARO_PA_PER_M_SEA_LEVEL = 1.225 * 9.80665          # dp / dh at sea level, ISA [Pa / m]
BARO_MODELS: dict[str, dict[str, float]] = {
    "BMP280": {"baro_noise_m": 0.44, "baro_quant_m": 0.055},
    "MS5611": {"baro_noise_m": 0.25, "baro_quant_m": 0.0},
}
# the sensor / filter set the v0.5 .. v0.9 snapshots flew (regression pins: SensorConfig(**LEGACY_SENSOR_SET))
LEGACY_SENSOR_SET: dict[str, float] = {**BARO_MODELS["MS5611"], "altitude_crossover_hz": 0.5}


@dataclass
class SensorConfig:
    """Sensor + estimator model: GY-521 MPU-6050 and the BMP280 barometer
    (2026-11-10; ``BARO_MODELS``: the GY-63 MS5611 of v0.5 .. v0.9 stays
    selectable).  Each field says where its number comes from: datasheet, the
    task brief of the ATmega128 retune, SPEC §9, or ASSUMED."""

    # MPU-6050 gyro, FS_SEL = 3
    gyro_noise_density_rad_s_sqrthz: float = math.radians(0.005)   # MPU-6050 datasheet (task brief)
    gyro_range_dps: float = 2000.0                                  # FS_SEL = 3 (task brief)
    gyro_lsb_per_dps: float = 16.4                                  # FS_SEL = 3 sensitivity (task brief)
    gyro_bias_max_rad_s: float = math.radians(0.5)                  # task brief: slowly varying bias up to 0.5 deg/s
    gyro_bias_sigma_rad_s: float = math.radians(0.25)               # ASSUMED: Gauss-Markov stationary std (= bound / 2)
    gyro_bias_tau_s: float = 60.0                                   # ASSUMED: Gauss-Markov correlation time ("slowly")
    gyro_bias_ramp_rad_s2: tuple[float, float, float] | None = None # worst-case study: deterministic drift from launch
    imu_dlpf_tau_s: float = 0.0028                                  # ASSUMED DLPF_CFG = 2 (gyro 98 Hz, datasheet delay 2.8 ms)
    # MPU-6050 accelerometer, AFS_SEL = 3
    accel_noise_density_mps2_sqrthz: float = 400e-6 * G0            # MPU-6050 datasheet, 400 ug/sqrt(Hz)
    accel_bias_mps2: float = 0.01 * G0                              # ASSUMED 10 mg residual after offset calibration
    accel_range_g: float = 16.0                                     # ASSUMED AFS_SEL = 3
    accel_lsb_per_g: float = 2048.0                                 # AFS_SEL = 3 sensitivity (datasheet)
    # BMP280 barometer (rate = control.baro_hz; BARO_MODELS["BMP280"], the numbers' sources there)
    baro_noise_m: float = BARO_MODELS["BMP280"]["baro_noise_m"]     # white, per sample [m RMS]
    baro_quant_m: float = BARO_MODELS["BMP280"]["baro_quant_m"]     # output step [m]; 0 = none (the MS5611 model)
    baro_latency_s: float = 0.1                                     # SPEC §9 (kept: covers the driver's 16.75 ms)
    # attitude estimators
    attitude_estimator: str = "pad_gyro"                            # "pad_gyro" (firmware) | "truth_noise" | "mahony"
    pad_avg_tau_s: float = 2.0                                      # ASSUMED firmware pad-averaging time constant
    pad_prealign_s: float = 10.0                                    # ASSUMED: FC aligning on the pad this long before t = 0
    nav_state_lag_ticks: int = 2                                    # v0.6.3 co-simulation: the NAV's pad -> flight switch
                                                                    # (estimator + altitude hold) 2 ticks behind the supervisor
    att_noise_rad: float = math.radians(0.5)                        # truth_noise: SPEC §9 "truth + 0.5 deg"
    mahony_kp: float = 1.0                                          # ASSUMED (legacy mahony mode)
    mahony_ki: float = 0.05                                         # ASSUMED (gyro-bias learning)
    mahony_accel_gate: float = 0.08                                 # ASSUMED: use accel only when ||a|-g|/g < gate
    altitude_crossover_hz: float = 0.4                              # complementary-filter crossover; 0.5 until
                                                                    # 2026-11-10 (retuned for the BMP280 noise on
                                                                    # TUNE 7400, the note above BARO_MODELS)
    altitude_zeta: float = 0.7                                      # ASSUMED
    # ---- v0.7.1 VTOL build only (a ballistic run never reads them) ----
    accel_vibration_rms_mps2: float = 0.0                           # accelerometer vibration at hover thrust (ASSUMED,
                                                                    # VTOL_ACCEL_VIBRATION_RMS_G_ASSUMED in VTOL runs)
    gm_kp: float | None = None                                      # gated_mahony overrides (tuning); None -> params
    gm_ki: float | None = None                                      # mission.vtol.nav_estimator (kp_ASSUMED, ki_ASSUMED,
    gm_accel_lpf_hz: float | None = None                            # accel_lpf_hz_ASSUMED, accel_gate)
    gm_accel_gate: float | None = None
    vtol_alt_bias_tau_s: float = VTOL_ALT_BIAS_TAU_S_ASSUMED        # VTOL altitude filter bias pole (reference, ASSUMED)
    baro_comp_lagged_whatif: bool = False                           # WHAT-IF (not the reference): the baro thrust
                                                                    # compensation on the thrust ratio through the EDF lag,
                                                                    # delayed by the baro latency (LaggedBaroCompFilter)


@dataclass
class SimOptions:
    """Environment / model constants not in params.json (all ASSUMED)."""

    chute_inflate_tau_s: float = 1.0        # canopy inflation time constant
    chute_rot_damping_nms: float = 0.8      # suspension-line damping on body rates under canopy
    batt_v_full_cell: float = 4.15          # V per cell, fresh pack
    batt_v_dod_slope: float = 0.55          # V per cell lost over the full capacity
    batt_sag_v_full_throttle: float = 0.25  # V per cell IR sag at u = 1 (~100 A x 2.5 mOhm)
    stop_after_touchdown_s: float = 2.0     # sim continues this long after touchdown (no landing detector in v0.4)
    descent_substeps_after_release_s: float = 1.0   # canopy inflating this long at full rate, then descent_substeps
    supervisor_enabled: bool = True         # False = interlock I3 (spin / gyro saturation) off - "why I3" study only


@dataclass(frozen=True)
class Gust:
    """One gust of the VTOL schedule (world frame, on top of the steady wind and the turbulence)."""

    t0_s: float                 # start, absolute sim time
    duration_s: float
    speed_mps: float            # horizontal peak speed
    dir_deg: float = 0.0        # direction the air moves toward, from +x toward +y
    rise_s: float = 0.0         # 0: step (rectangular, as the ballistic gust); > 0: 1 - cos ramps of this length at both ends
    vertical_mps: float = 0.0   # peak vertical component, up +

    @property
    def t1_s(self) -> float:
        return self.t0_s + self.duration_s

    def factor(self, t: float) -> float:
        """Envelope 0..1 at time t (1 on the plateau, 0 outside [t0, t1))."""
        if not (self.t0_s <= t < self.t0_s + self.duration_s):
            return 0.0
        r = min(self.rise_s, 0.5 * self.duration_s)
        if r <= 0.0:
            return 1.0
        e = min(t - self.t0_s, self.t0_s + self.duration_s - t)
        return 1.0 if e >= r else 0.5 * (1.0 - math.cos(math.pi * e / r))


@dataclass
class Scenario:
    """One simulation case.  Times are absolute unless noted."""

    name: str
    title: str = ""
    t_end: float = 60.0
    seed: int = 1
    # environment
    wind_mps: float = 0.0
    wind_dir_deg: float = 0.0               # direction the air moves toward, from +x toward +y
    gust_mps: float = 0.0
    gust_t0_after_launch_s: float = 5.0
    gust_duration_s: float = 1.0
    gust_dir_deg: float | None = None       # None -> same as the wind
    # initial condition
    initial_tilt_deg: float = 0.0
    initial_tilt_axis_deg: float = 0.0      # rotation axis in the body xy plane, from +x
    initial_altitude_m: float = 0.0         # of the exit plane above the pad
    initial_yaw_deg: float = 0.0
    # mission (v0.4: operator + ground station; the vehicle only executes commands and interlocks)
    bypass_state_machine: bool = False      # S1: controller always active, fixed throttle, no supervisor / link
    fixed_throttle: float | None = None     # with bypass; None -> hover throttle
    arm_time_s: float = 0.5                 # operator queues ARM (once the ground has telemetry showing SAFE)
    launch_time_s: float = 1.0              # operator queues LAUNCH (once the telemetry shows ARMED)
    kill_time_s: float | None = None        # operator queues KILL
    target_altitude_m: float | None = None  # ground assist target (None -> mission.target_altitude_m)
    geofence_altitude_m: float | None = None    # onboard I5 override (S3 only)
    apogee_limit_m: float | None = None         # onboard I4 override (S3 only)
    # ground link
    assist: bool = True                     # ground assist mode (auto-CUT / auto-RELEASE)
    radio: str | None = None                # None -> sim.link_model.model_radio (2026-11-10: link.vehicle_radio when modelled,
                                            # the ESP bridge; else link.radio, the LoRa fallback)
    packet_loss: float | None = None        # None -> flight_computer.link.sim_packet_loss
    link_outage_after_launch_s: float | None = None   # total link loss from this long after LAUNCH executes (S6)
    link_seed: int | None = None            # None -> derived from seed
    arm_switch_open_t_s: float | None = None    # fault: the arm switch opens at this time
    # what-if studies only (not the SPEC 8.8 contract): proposed fixes reported next to the contract runs
    coast_drag_factor: float | None = None      # replaces mission.coast_drag_factor in I4 and in the assist CUT
    release_compensated: bool | None = None     # None: mission.assist_release_rule; True/False forces predictive/literal
    # faults
    vane_failure_index: int | None = None   # 0-based vane
    vane_failure_t_after_launch_s: float = 2.0
    vane_failure_stuck_deg: float | None = None   # None -> frozen at its current angle, else hardover to this
    # numerics
    physics_substeps: int | None = None     # None -> ceil(control.imu_hz / control.loop_hz) (250 Hz x 4 = 1 kHz)
    descent_substeps: int = 1               # under canopy nothing is fast; control-rate RK4 is plenty
    # overrides
    gains: dict | None = None
    sensors: SensorConfig = field(default_factory=SensorConfig)
    options: SimOptions = field(default_factory=SimOptions)
    timing: FCTiming = field(default_factory=FCTiming)
    notes: str = ""
    # ---- v0.7 VTOL build (SPEC 8.9).  Every field below is a no-op at its default: a Scenario that leaves them alone
    # runs the v0.6.3 ballistic statements bit-identically (scratch regression + sim/test_vtol_scenarios.py) ----
    mode: str = "ballistic"                 # "ballistic" (v0.4-v0.6.3 mission, ScriptedOperator) | "vtol" (FcuStack +
                                            # sim.vtol_scenarios.VtolOperator).  Deliberately NOT params.json
                                            # mission.mode: the fields above describe the ballistic mission, a VTOL
                                            # run is built by sim.vtol_scenarios
    vtol_script: tuple = ()                 # VTOL ground operator steps (sim.vtol_scenarios.OpStep)
    vgains: dict | None = None              # vertical-controller overrides (VerticalParams field -> value)
    gusts: tuple = ()                       # VTOL gust schedule (Gust, absolute times)
    turbulence_sigma_mps: float = 0.0       # horizontal turbulence std (first-order Gauss-Markov per axis), 0 = off
    turbulence_tau_s: float = 1.5           # ASSUMED correlation time (~ L / V, L ~ 5 m at a few m/s near the ground)
    turbulence_vertical_ratio: float = 0.5  # ASSUMED sigma_w / sigma_u near the ground
    ground_z_m: float | None = None         # VTOL landing surface away from the pad [m, exit-plane height]; None -> the
                                            # level the pad's feet stand on, -(plate_t + leg_h + foot_t)
    link_outages: tuple = ()                # absolute [t0, t1) windows of total link loss
    # 2026-11-10 study hook: ((t_from_launch_s, radio), ...) - frames sent from launch + t on that link model (e.g. the
    # ESP link's latency stepping to the LoRa model's); the ground keeps the timeout / t_link default of ``radio``
    link_radio_steps: tuple = ()
    # 2026-11-10: the logic-battery tail (TELEM v6 / v7) of the VTOL battery build on the carrier; None -> params
    # flight_computer.logic_battery_monitor.telemetry_builds (sim.link_proto.lbat_build); a ballistic run never has it
    lbat_telemetry: bool | None = None
    baro_fail_t_s: float | None = None      # fault: the barometer delivers nothing from this time (VTOL)
    imu_fail_t_s: float | None = None       # fault: IMU / NAV failure from this time - NAV outputs frozen (VTOL)
    mass_scale: float = 1.0                 # plant mass and inertia x this (the controllers keep the nominal model)
    cg_offset_mm: tuple = (0.0, 0.0)        # plant CG offset from the thrust axis, body (x, y) [mm]
    thrust_scale: float = 1.0               # plant thrust x this (fan / proxy-table spread; controllers keep T_max)
    thrust_voltage_exp: float = 0.0         # 0: no voltage term (the v0.6.3 model); k > 0: T x (V_cell / V_nom_cell)^k
    servo_tau_scale: float = 1.0            # plant servo time constant x this
    vane_trim_err_deg: tuple = (0.0, 0.0, 0.0, 0.0)   # plant vane angle offsets (mechanical zero error) [deg]
    # ---- v0.7.1 VTOL build (each a no-op at its default; read only when mode == "vtol") ----
    profile: str | None = None              # build profile 'field' | 'demo' (params mission.vtol.profiles); None ->
                                            # params mission.vtol.profile
    wind_swing_deg: float = 0.0             # mean-wind direction: sinusoidal swing amplitude [deg] ...
    wind_swing_period_s: float = 30.0       # ... period ...
    wind_swing_phase_deg: float = 0.0       # ... phase at t = 0
    wind_dir_rate_sigma_dps: float = 0.0    # random-walk direction: std of the direction rate [deg/s] (first-order
                                            # Gauss-Markov rate, WIND_DIR_RATE_TAU_S_ASSUMED; own random stream)
    wind_reversal_t_s: float | None = None  # the mean-wind direction turns 180 deg from this time ...
    wind_reversal_s: float = 5.0            # ... over this long (1 - cos blend, speed unchanged) ...
    wind_reversal_sign: float = 1.0         # ... counter-clockwise from above (+1: from +x toward +y) or clockwise (-1)
    wind_speed_mod_frac: float = 0.0        # mean-wind speed x (1 + frac sin(2 pi t / period)) ...
    wind_speed_mod_period_s: float = 20.0   # ... period
    pushes: tuple = ()                      # hand pushes (Push, absolute times) - plant only
    baro_thrust_offset_m: float = 0.0       # plant: true barometer static-pressure error with the fan running, as an altitude
                                            # at the nominal hover thrust (m_nominal g) [m, + = reads high], x T / (m g)
    baro_thrust_comp_m: float | None = None # firmware compensation (VtolAltitudeFilter); None -> params
                                            # mission.vtol.baro_thrust_comp_m (0.0 = not calibrated)
    crossflow_cd_scale: float = 1.0         # plant: aero.vtol_crossflow_cd_ASSUMED x this (campaign spread)
    ram_drag_scale: float = 1.0             # plant: EDF ram drag x this (campaign spread)
    # ---- v0.7.1 fix round (VTOL only) ----
    swirl_scale: float = 1.0                # plant: the EDF residual swirl torque (edf.residual_swirl_torque_fraction_
                                            # ASSUMED 0.15) x this - envelope / campaign sensitivity (0 .. 2 = 0 .. 0.30)
    # ---- v0.8 (VTOL only; each a no-op at its default: the v0.7.1 statements and the v0.7.1 supervisor build) ----
    gps: GpsSim | None = None               # GPS position hold build: the receiver model + the pad set-up; None = no GPS
                                            # (the supervisor built with gps=False: no ON gate, no V11-V14, TELEM v2)
    nav_v08: bool | None = None             # the v0.8 NAV tick order (publish after the gyro prediction); None -> on iff
                                            # ``gps`` is set (the GPS-off what-if of the v0.8 build: nav_v08=True)
    takeoff_check: bool | None = None       # supervisor V15 + the vertical controller's take-off check; None -> on iff the
                                            # v0.8 NAV order is on (the v0.8 build)
    pgains: dict | None = None              # sim.position_ctrl.PositionParams overrides (tuning; vtol_gains.json position)
    # ---- v0.9 plant (VTOL hardware round).  None -> the params / vehicle value; 0 (or None for the switches) -> the
    # element is off and the v0.8.1 plant statements run bit for bit (the regression pins set them so) ----
    vane_play_deg: Any = None               # direct-drive vane play, total width [deg], one value or 4; None -> params
                                            # vane.direct_drive_play (lost motion 0.65 + gear backlash 1 ASSUMED; 0 for the
                                            # pushrod linkage), 0 -> no play
    rotor_inertia_kg_m2: float | None = None    # EDF rotor inertia (gyroscopic moment + spool-up reaction); None -> params
                                                # edf.thrust_v09.rotor_inertia_kg_m2_ASSUMED (4.5e-5), 0 -> off
    t_max_kgf: float | None = None          # VTOL plant T_max at 22.2 V [kgf] (the v0.9 envelope 2.14 .. 3.09, stress 1.9 ..);
                                            # None -> thrust_scale x params thrust_nominal_N.  Controllers keep the model
    fan_power_model: float = 0.0            # VTOL electrical plant: 0 = P1 (proxy efficiency) .. 1 = P2 (pessimistic)
    power_source: str | None = None         # VTOL electrical plant 'battery' | 'tether' | 'hybrid'; None -> the vehicle's on the
                                            # v0.9 VTOL vehicle, the v0.8.1 battery model on a rocket vehicle
    power: Any = None                       # sim.tether.PowerConfig (None -> PowerConfig.from_params with the source)
    power_events: tuple = ()                # sim.tether.PowerEvent (absolute times): unplug, cut, supply_trip, sag, r_loop
    tether: Any = None                      # sim.tether.TetherConfig (overhead anchor, feed, catch line); None -> no tether
    # ---- v0.9 fix round (physics review; defaults = off, bit-identical) ----
    vane_force_scale: float = 1.0           # PLANT vane side force x (hub / motor wake: live span x0.76-0.85); the controllers'
                                            # mixer model keeps the params C_La S_v (the campaign samples 0.72-1.05)
    vane_yaw_lever_scale: float = 1.0       # PLANT vane yaw lever x (the force centroid moves out with a dead inner span)
    ge_thrust_a: float = 0.0                # ground effect (ASSUMED shape): T x (1 + a_T e), vane q x (1 - a_q e),
    ge_vane_a: float = 0.0                  # e = exp(-4 h / D_duct), h = exit plane above the surface the jet meets


@dataclass
class SimResult:
    scenario: Scenario
    log: dict[str, np.ndarray]
    events: list[tuple[float, str]]
    gains: dict
    mass_source: str
    runtime_s: float
    t_liftoff: float | None
    t_touchdown: float | None
    touchdown_speed_mps: float | None
    final_state: State
    nan_abort: bool = False
    touchdown_vz_mps: float | None = None
    link: dict | None = None                # ground link records (flight scenarios): see FlightSim.run
    vtol: dict | None = None                # VTOL runs: touchdowns, lift-offs, controller parameters, operator log


class ScriptedOperator:
    """The person at the ground station in a simulated flight, acting on the latest telemetry only:
    from ``arm_time_s`` ARM whenever it shows SAFE, from ``launch_time_s`` LAUNCH whenever it shows ARMED
    (so a rejected command or an I1p disarm on the pad is simply retried, at most ``TRIES`` commands),
    nothing more once it has shown a flight state; KILL at ``kill_time_s``.  CUT / RELEASE come from the
    ground assist mode (or from nobody, S7)."""

    TRIES = 8
    RETRY_S = 0.5           # ASSUMED operator reaction time before repeating a command that did not take

    def __init__(self, sc: Scenario):
        self.sc = sc
        self.tries = 0
        self.t_next = sc.arm_time_s
        self.flown = False
        self.kill_rec = None

    def step(self, gs: GroundAssist, now: float) -> None:
        tl = gs.latest
        sc = self.sc
        if sc.kill_time_s is not None and self.kill_rec is None and now >= sc.kill_time_s:
            self.kill_rec = gs.kill(now)
        if tl is None or not gs.synced:
            return
        if tl.state in (lp.STATE_POWERED, lp.STATE_UNPOWERED, lp.STATE_RELEASED, lp.STATE_ABORT):
            self.flown = True
        if self.flown or gs.pending is not None or now < self.t_next or self.tries >= self.TRIES:
            return
        if tl.state == lp.STATE_SAFE:
            gs.arm(now)
        elif tl.state == lp.STATE_ARMED and now >= sc.launch_time_s:
            gs.launch(now)
        else:
            return
        self.tries += 1
        self.t_next = now + self.RETRY_S


class _MassCfg:
    """Pre-flattened constants for one mass configuration (floats and lists only)."""

    def __init__(self, m: float, inertia: np.ndarray, z_cg_m: float, vane: VaneModel, z_cp_m: float,
                 z_attach_m: float):
        self.m = float(m)
        self.inv_m = 1.0 / self.m
        self.z_cg = float(z_cg_m)
        I = np.asarray(inertia, dtype=float)
        self.I = I.tolist()
        self.invI = np.linalg.inv(I).tolist()
        B = vane.moment_matrix(z_cg_m)
        self.b0, self.b1, self.b2 = B[0].tolist(), B[1].tolist(), B[2].tolist()
        self.l_cp = float(z_cp_m - z_cg_m)
        self.l_att = float(z_attach_m - z_cg_m)


# --------------------------------------------------------------------------- #
# the simulator
# --------------------------------------------------------------------------- #
class FlightSim:
    N_STATE = 18

    def __init__(self, vehicle: Vehicle, scenario: Scenario, gains: dict | None = None, operator: Any = None):
        """``operator``: the VTOL ground operator (an object with ``step(gs, now)`` and ``expects_flight()``);
        None -> ``sim.vtol_scenarios.VtolOperator(scenario)`` for a VTOL scenario, ``ScriptedOperator`` otherwise."""
        self.veh = vehicle
        self.sc = scenario
        self.mode = lp.check_mode(scenario.mode)
        self.vtol = self.mode == lp.MODE_VTOL
        p = vehicle.params
        L = vehicle.layout
        self.p = p
        self.g = G0
        self.rho = float(p["aero"]["rho_kg_m3"])
        self.edf = EDFModel.from_params(p, L.duct_id)
        self.vane = VaneModel.from_params(p, L)
        self.ctrl_hz = float(p["control"]["loop_hz"])
        self.dt_c = 1.0 / self.ctrl_hz
        imu_hz = float(p["control"]["imu_hz"])
        self.n_sub = int(scenario.physics_substeps) if scenario.physics_substeps else max(1, math.ceil(imu_hz / self.ctrl_hz - 1e-9))
        self.dt_p = self.dt_c / self.n_sub
        if scenario.sensors.attitude_estimator not in ESTIMATORS:
            raise ValueError(f"attitude_estimator {scenario.sensors.attitude_estimator!r} not in {ESTIMATORS}")
        # flight-computer output timing (params.flight_computer)
        fc = p["flight_computer"]
        tim = scenario.timing
        self.frame_hz = float(fc["servo_frame_hz"]) if tim.frame_hz is None else float(tim.frame_hz)
        self.frame_hz_timer = frame_hz_from_timer(fc)
        self.channel_timer = timer_of_channel(fc)          # [vane1..4, ESC]
        self.phase_s = 0.5 * self.dt_c if tim.phase_s is None else float(tim.phase_s)
        t1 = self.channel_timer[0]
        self.channel_phase_s = [self.phase_s + (0.0 if tmr == t1 else tim.timer3_phase_s) for tmr in self.channel_timer]

        # gains: params.control <- gains file/dict (<- VTOL: control.vtol_overrides, v0.7.1b) <- scenario overrides;
        # ballistic: gains_for_mode == gains_from_control, bit for bit
        g = gains_for_mode(p["control"], gains, self.mode)
        if scenario.gains:
            g.update({k: v for k, v in scenario.gains.items() if not k.startswith("_")})
        self.gains = g
        s = p["servo"]
        plant = MixerPlant(
            l_v_m=vehicle.l_v_m, r_v_m=vehicle.r_v_m, s_v_m2=vehicle.s_v_m2,
            cl_alpha=float(p["vane"]["cl_alpha_per_rad"]), a_e_m2=vehicle.a_e_m2,
            t_max_n=vehicle.t_max_n, throttle_exp=float(p["edf"]["throttle_exponent"]),
            delta_max_rad=math.radians(p["vane"]["max_deflection_cmd_deg"]),
            pwm_center_us=float(s["pwm_center_us"]), pwm_us_per_deg=float(s["pwm_us_per_deg"]),
        )
        self.ctrl = AttitudeController(p["control"], plant, g)
        # v0.8: the GPS position-hold build, the v0.8 NAV order, the take-off check (VTOL only)
        self.gps_on = self.vtol and scenario.gps is not None
        self.v08 = self.vtol and bool(scenario.nav_v08 if scenario.nav_v08 is not None else self.gps_on)
        self.tko = self.vtol and bool(scenario.takeoff_check if scenario.takeoff_check is not None else self.v08)
        self.nav_thr_lagged = self.vtol and bool((p["mission"].get("vtol") or {}).get("nav_throttle_lagged", False))
        if self.gps_on and not self.v08:
            raise ValueError("the GPS build flies the v0.8 NAV order (nav_v08 must not be False with gps set)")
        # onboard supervisor + ground station + link (SPEC 8.8 / 10.3)
        sp_kw: dict[str, Any] = ({"profile": scenario.profile, "gps": self.gps_on, "takeoff_check": self.tko}
                                 if self.vtol else {})
        self.sp = SupervisorParams.from_params(p, mode=self.mode, geofence_altitude_m=scenario.geofence_altitude_m,
                                               apogee_limit_m=scenario.apogee_limit_m,
                                               coast_drag_factor=scenario.coast_drag_factor,
                                               rate_abort_enabled=scenario.options.supervisor_enabled, **sp_kw)
        lseed = scenario.seed if scenario.link_seed is None else scenario.link_seed
        self.vp: VerticalParams | None = None
        self.fcu: FcuStack | None = None
        self.chain = "fcu"                          # v0.8.1: set to the GPS build's chain below
        self.glink_L: int | None = None
        if self.vtol:
            # the controller's model: nominal mass and T_max (plant spread below never reaches it)
            vfields = set(VerticalParams.__dataclass_fields__)
            vg = {k: v for k, v in (scenario.vgains or {}).items() if k in vfields}
            if not self.tko and "takeoff_ticks" in vfields:
                vg["takeoff_ticks"] = 0                 # v0.8 take-off check off: the v0.7.1 vertical controller
            self.vp = VerticalParams.from_params(p, vehicle.full.mass_kg, vehicle.t_max_n, profile=scenario.profile, **vg)
            self.pp: PositionParams | None = None
            if self.gps_on:
                # v0.8: the position hold's model T/W from the same model mass (the CAD vehicle, GPS pod included) and
                # T_max; Scenario.pgains = the tuned gains (sim/out/vtol_gains.json "position") over params / module values
                pfields = set(PositionParams.__dataclass_fields__)
                pg = {k: v for k, v in (scenario.pgains or {}).items() if k in pfields}
                self.pp = PositionParams.from_params(p, vehicle.full.mass_kg, vehicle.t_max_n, g=self.sp.g,
                                                     profile=scenario.profile, **pg)
                # v0.8.1: the four-chip chain (GNS board, sim.glink_proto) unless the scenario asks for the v0.8 one
                self.chain = scenario.gps.chain or ("gns" if "glink" in p["flight_computer"] else "fcu")
                self.glink_L = (int(scenario.gps.glink_latency_ticks) if scenario.gps.glink_latency_ticks is not None
                                else glink_latency_ticks(p))
            self.fcu = FcuStack(self.sp, self.vp, seed=(lseed * 40503 + 12345) & 0xFFFF, pp=self.pp,
                                chain=self.chain if self.gps_on else "fcu")
            self.sup = self.fcu.sup
        else:
            self.sup = OnboardSupervisor(self.sp, seed=(lseed * 40503 + 12345) & 0xFFFF)   # firmware: ADC-noise seed
        self.gp_nav = None
        self.ant_b_mm: tuple | None = None
        if self.gps_on:
            gs_ = scenario.gps
            nav_kw = dict(gs_.nav or {})
            if gs_.pad_azimuth_deg is not None:
                nav_kw["pad_azimuth_deg"] = float(gs_.pad_azimuth_deg)
            self.gp_nav = gn.NavGpsParams.from_params(p, **nav_kw)
            ant = getattr(vehicle, "gps_antenna_mm", None) or getattr(L, "gps_antenna_xyz", None)
            if ant is None:
                raise ValueError("the GPS build needs the antenna (Layout.gps_antenna_xyz or the VTOL top-cap patch)")
            self.ant_b_mm = tuple(float(v) for v in ant)     # phase centre, vehicle frame [mm] (CAD; v0.9 VTOL: top cap)
        self.link_cfg = LinkConfig.from_params(p, radio=scenario.radio, loss=scenario.packet_loss)
        # v0.9 RP1: the tether / hybrid builds reply with TELEM v4 / v5 (the power tail): the ground's round trip and
        # reply timeout are sized for that frame (the plant source as the supervisor will resolve its build from it)
        self.pwr_link = self.vtol and power_tail_link(self.sp, scenario.power_source or (
            getattr(vehicle, "power_source", None) if str(getattr(vehicle, "hardware_config", "rocket")) == "vtol"
            else None))
        # 2026-11-10: the carrier's battery build replies with the logic-battery tail (TELEM v6 / v7)
        self.lbat_link = bool(self.vtol and not self.pwr_link and (
            lp.lbat_build(p, self.mode) if scenario.lbat_telemetry is None else scenario.lbat_telemetry))
        self.lbat_q = lp.lbat_quant(SIM_LOGIC_BAT_V_ASSUMED) if self.lbat_link else lp.LBAT_NONE
        self.link_rt = (vtol_round_trip(self.link_cfg, gps=self.gps_on, power=self.pwr_link, lbat=self.lbat_link)
                        if self.vtol else round_trip(self.link_cfg))
        self.gp = GroundParams.from_params(p, reply_timeout_s=self.link_rt["reply_timeout_s"],
                                           t_link_default_s=self.link_rt["rtt_mean_s"],
                                           target_altitude_m=scenario.target_altitude_m,
                                           coast_drag_factor=scenario.coast_drag_factor,
                                           release_compensated=scenario.release_compensated, mode=self.mode)
        self.ground = GroundAssist(self.gp, assist=scenario.assist)
        self.link = LinkModel(self.link_cfg, seed=lseed + 7919)
        if scenario.link_outages:
            self.link.outages.extend((float(a), float(b)) for a, b in scenario.link_outages)
        self.link_steps = tuple((float(a), str(r)) for a, r in scenario.link_radio_steps)   # armed at launch (run loop)
        self.vehicle_id = int(p["flight_computer"]["link"]["vehicle_id"])
        self.uplink_decoder = lp.FrameDecoder(self.vehicle_id, (lp.TYPE_CMD,))
        if operator is not None:
            self.operator = operator
        elif self.vtol:
            try:
                from sim.vtol_scenarios import VtolOperator
            except ImportError:  # pragma: no cover
                from vtol_scenarios import VtolOperator  # type: ignore
            self.operator = VtolOperator(scenario)
        else:
            self.operator = ScriptedOperator(scenario)
        sens = scenario.sensors
        self.alt = AltitudeFilter(sens.altitude_crossover_hz, sens.altitude_zeta, sens.baro_latency_s)
        self.valt: VtolAltitudeFilter | None = None
        if self.vtol:
            # v0.7.1 VTOL NAV altitude filter (h, v, accelerometer z-bias; pad calibration; baro thrust compensation)
            fcls = LaggedBaroCompFilter if sens.baro_comp_lagged_whatif else VtolAltitudeFilter
            self.valt = fcls.from_params(p, u_hover=self.vp.u_hover, crossover_hz=sens.altitude_crossover_hz,
                                         zeta=sens.altitude_zeta, baro_latency_s=sens.baro_latency_s,
                                         bias_tau_s=sens.vtol_alt_bias_tau_s, pad_bias_tau_s=sens.pad_avg_tau_s)
            if sens.baro_comp_lagged_whatif:
                self.valt.setup_lag(self.dt_c, float(p["edf"]["thrust_time_constant_s"]), sens.baro_latency_s)
            if scenario.baro_thrust_comp_m is not None:
                self.valt.comp_m = float(scenario.baro_thrust_comp_m)

        # servo / vane constants
        self.servo_tau = float(s["time_constant_s"])
        self.servo_rate = math.radians(s["rate_limit_deg_s"])
        self.delta_mech = self.vane.delta_mech_max_rad
        self.stall = self.vane.stall_rad
        e = self.vane.e_theta()
        self.ex, self.ey = e[:, 0].tolist(), e[:, 1].tolist()
        self.sv_cla = self.vane.s_v_m2 * self.vane.cl_alpha
        self.inv_2ae = 1.0 / (2.0 * self.edf.a_e_m2)
        self.swirl_per_N = self.edf.swirl_sign * self.edf.swirl_fraction * self.edf.p_nom_w / self.edf.omega_max_rad_s / self.edf.t_max_n
        if self.vtol and float(scenario.swirl_scale) != 1.0:
            self.swirl_per_N *= float(scenario.swirl_scale)       # VTOL plant: swirl sensitivity (fix round)
        self.inv_tau_T = 1.0 / self.edf.tau_s
        self.inv_tau_s = 1.0 / self.servo_tau
        a = vehicle.aero
        self.a_ref, self.cd, self.cna = a.a_ref_m2, a.cd, a.cn_alpha_total
        rec = p["recovery"]
        self.chute_cda_full = float(rec["vehicle_chute_cd"]) * math.pi * (0.5 * float(rec["vehicle_chute_d_m"])) ** 2
        if not getattr(vehicle, "has_chute", True):
            self.chute_cda_full = 0.0       # v0.9 VTOL configuration: no parachute (a release opens nothing)
        # shock cord anchored at the top of the payload module after nose release (VTOL configuration: the top cap)
        z_attach = float(getattr(L, "z_payload_top", None) or getattr(L, "z_cap_top", 0.0)) * 1e-3
        self.cfg_full = _MassCfg(vehicle.full.mass_kg, vehicle.full.inertia_kg_m2, vehicle.z_cg_m, self.vane,
                                 a.z_cp_m, z_attach)
        self.cfg_post = _MassCfg(vehicle.descent_mass_kg, vehicle.post_ejection.inertia_kg_m2,
                                 float(vehicle.post_ejection.com_m[2]), self.vane, a.z_cp_m, z_attach)
        b = p["battery"]
        self.e_cap_j = float(b["cells"]) * 3.7 * float(b["capacity_mAh"]) / 1000.0 * 3600.0
        self.rng = np.random.default_rng(scenario.seed)
        sens = scenario.sensors
        self.imu = Mpu6050(sens.gyro_lsb_per_dps, sens.gyro_range_dps, sens.accel_lsb_per_g, sens.accel_range_g, G0)
        self.pwm_center_s = float(s["pwm_center_us"]) * 1e-6
        # ---- plant spread (VTOL campaign) - applied only when set, so the defaults keep every v0.6.3 number ----
        ms = float(scenario.mass_scale)
        if ms != 1.0:
            self.cfg_full = _MassCfg(vehicle.full.mass_kg * ms, np.asarray(vehicle.full.inertia_kg_m2) * ms,
                                     vehicle.z_cg_m, self.vane, a.z_cp_m, z_attach)
            self.cfg_post = _MassCfg(vehicle.descent_mass_kg * ms, np.asarray(vehicle.post_ejection.inertia_kg_m2) * ms,
                                     float(vehicle.post_ejection.com_m[2]), self.vane, a.z_cp_m, z_attach)
        cgo = tuple(float(v) for v in scenario.cg_offset_mm)
        self.cg_off = (cgo[0] * 1e-3, cgo[1] * 1e-3) if (cgo[0] != 0.0 or cgo[1] != 0.0) else None
        self.thrust_scale = float(scenario.thrust_scale)
        self.thrust_vexp = float(scenario.thrust_voltage_exp)
        self.v_nom_cell = float(p["edf"]["voltage_nominal_V"]) / float(p["edf"]["cells"])
        if scenario.servo_tau_scale != 1.0:
            self.servo_tau *= float(scenario.servo_tau_scale)
            self.inv_tau_s = 1.0 / self.servo_tau
        trims = tuple(float(v) for v in scenario.vane_trim_err_deg)
        self.vane_trim = [math.radians(v) for v in trims] if any(v != 0.0 for v in trims) else None
        # VTOL landing surface: the pad's top plate (z = 0) around the origin, the ground elsewhere
        lpd = p["launch_pad"]
        self.pad_radius_m = 0.5 * float(lpd["plate_od"]) * 1e-3
        self.pad_foot_z_m = -(float(lpd["plate_t"]) + float(lpd["leg_h"]) + float(lpd["foot_t"])) * 1e-3
        self.ground_z = self.pad_foot_z_m if scenario.ground_z_m is None else float(scenario.ground_z_m)
        # v0.8 physics-review fix (medium 8): the touchdown tip-over margin (``tip_over_vcrit``).  Footprint radius =
        # the fin-tip circle of the layout (footprint_d / 2); the CG height of the JUDGED margin = r / tan(params
        # position_hold.tipover.static_angle_deg 14.2 deg) = 0.49 m (the params note's "CG 0.487 m above the ground" -
        # the CG above the GROUND when standing on the 0.11 m pad, conservative); the geometric one (the sim's own
        # contact plane, the exit plane / fin trailing edges: CG z_cg above it, static angle ~18 deg) is reported too
        self.tip_r_m = 0.5 * float(L.footprint_d) * 1e-3
        tov = (p["mission"].get("vtol", {}).get("position_hold") or {}).get("tipover") or {}
        sa = tov.get("static_angle_deg")
        self.tip_h_params_m = (self.tip_r_m / math.tan(math.radians(float(sa)))) if sa else None
        # v0.7.1 VTOL aerodynamics (params aero.vtol_*; the ballistic model keeps the slender-body normal force)
        self.vt_aero: dict[str, float] | None = None
        if self.vtol:
            ag = vtol_aero_geometry(vehicle)
            self.vt_aero = ag
            self.vt_kc = 0.5 * self.rho * ag["crossflow_cd_ASSUMED"] * ag["a_plan_m2"] * float(scenario.crossflow_cd_scale)
            self.vt_z_cf = ag["z_crossflow_m"]
            self.vt_z_in = ag["z_intake_m"]
            self.vt_ram_k = (self.rho * ag["a_exit_m2"]) if ag["ram_drag"] else 0.0   # mdot^2 / T
            self.vt_ram_scale = float(scenario.ram_drag_scale)
            self.vt_axial_k = 0.5 * self.rho * self.cd * self.a_ref
        self.body_layout = L
        # ---- v0.9 plant (VTOL hardware round; every element off at 0 -> the v0.8.1 statements) ----
        self.hw = str(getattr(vehicle, "hardware_config", "rocket"))
        pl = scenario.vane_play_deg
        if pl is None:
            pl = vane_play_deg_from_params(p)
        pls = tuple(float(v) for v in pl) if isinstance(pl, (tuple, list)) else (float(pl),) * 4
        if len(pls) != 4 or min(pls) < 0.0:
            raise ValueError("vane_play_deg: one value or four, >= 0")
        self.play_deg = pls
        self.play_h = [math.radians(v) * 0.5 for v in pls]
        self.play_on = any(h > 0.0 for h in self.play_h)
        self.vane_mem = [0.0, 0.0, 0.0, 0.0]            # actual vane angles (the play operator's state)
        if self.vtol and scenario.t_max_kgf is not None:
            self.thrust_scale = float(scenario.t_max_kgf) * G0 / self.edf.t_max_n * float(scenario.thrust_scale)
        self.t_fan_n = self.edf.t_max_n * self.thrust_scale        # the plant fan's T_max at 22.2 V
        self.rotor = RotorModel.from_params(p, self.t_fan_n, scenario.rotor_inertia_kg_m2)
        self.rot_on = self.rotor.i_rot_kg_m2 > 0.0
        self.rot_s_i = self.rotor.sign * self.rotor.i_rot_kg_m2
        # VTOL electrical plant (sim.tether): battery / tether / hybrid bus -> v_cell = V_bus / cells, thrust limit
        src = scenario.power_source or (getattr(vehicle, "power_source", None) if self.hw == "vtol" else None)
        self.pw: Any = None
        self.t_lim = math.inf
        self.meas: dict[str, Any] = {}
        if self.vtol and src is not None:
            cfgp = (scenario.power if scenario.power is not None else
                    tt.PowerConfig.from_params(p, source=src, v_rest_cell=float(scenario.options.batt_v_full_cell)))
            if cfgp.source != src:
                cfgp = replace(cfgp, source=src)
            if scenario.power_events:
                cfgp = replace(cfgp, events=tuple(cfgp.events) + tuple(scenario.power_events))
            self.pw = tt.PowerSystem(cfgp, tt.fan_power_from_params(p, self.t_fan_n, scenario.fan_power_model),
                                     seed=scenario.seed)
            if self.thrust_vexp <= 0.0:
                self.thrust_vexp = float((p["edf"].get("thrust_v09") or {}).get("voltage_exponent_proxy", 1.61))
        self.pw_cells = float(self.pw.cfg.cells) if self.pw is not None else float(p["battery"]["cells"])
        # tether mechanics (overhead anchor, cable feed, catch line) at the top-cap hardpoint
        self.rig: Any = None
        self.att_rb: tuple | None = None
        if self.vtol and scenario.tether is not None:
            att = getattr(vehicle, "tether_attach_body_m", None)
            if att is None:
                raise ValueError("a tether needs the VTOL configuration's top-cap hardpoint (Vehicle.tether_attach_body_m)")
            dx, dy = self.cg_off if self.cg_off is not None else (0.0, 0.0)
            self.att_rb = (float(att[0]) - dx, float(att[1]) - dy, float(att[2]) - self.cfg_full.z_cg)
            self.rig = tt.TetherRig(scenario.tether, self.ground_z, self.cfg_full.m, seed=scenario.seed)
            # slack-fed cable loop at the intake windows (fix round): the loop's lowest point more than this below the
            # eye bolt (window top + 50 mm clearance) and within 0.1 m of the axis counts as 'cable at the intake'
            self.loop_limit_m = float(att[2]) - float(L.z_window_top) * 1e-3 - 0.05
        # ---- v0.9 fix round (physics review): vane effectiveness, ground effect, regeneration inputs ----
        vfs, vys = float(scenario.vane_force_scale), float(scenario.vane_yaw_lever_scale)
        if vfs != 1.0:
            self.sv_cla *= vfs
        if vys != 1.0:
            for cfg_ in (self.cfg_full, self.cfg_post):
                cfg_.b2 = [v * vys for v in cfg_.b2]
        self.ge_a_t, self.ge_a_q = float(scenario.ge_thrust_a), float(scenario.ge_vane_a)
        self.ge_on = self.vtol and (self.ge_a_t != 0.0 or self.ge_a_q != 0.0)
        self.ge_k = 4.0 / (float(p["propulsion_module"]["duct_id"]) * 1e-3)       # e = exp(-ge_k h)
        if self.pw is not None and self.pw.cfg.regen_eff > 0.0:
            self.pw.set_rotor(0.5 * self.rotor.i_rot_kg_m2 * self.rotor.omega_ref_rad_s ** 2, self.rotor.t_fan_n,
                              self.edf.tau_s)

    def plant_thrust(self, u_esc: float, v_cell: float) -> float:
        """Steady thrust the plant produces for the decoded ESC command, with the plant spread (thrust_scale and the
        optional voltage term T x (V_cell / V_nom_cell)^k)."""
        tn = self.edf.thrust_steady(u_esc) * self.thrust_scale
        if self.thrust_vexp > 0.0:
            tn *= (max(v_cell, 0.0) / self.v_nom_cell) ** self.thrust_vexp
        if self.pw is not None and tn > self.t_lim:
            tn = self.t_lim                 # v0.9: the bus cannot carry the fan power (supply limit / collapse / ESC off)
        return tn

    def plant_vanes(self, srv: list[float]) -> list[float]:
        """Decoded vane commands + the mechanical zero error of each vane (campaign)."""
        if self.vane_trim is None:
            return srv
        return [srv[i] + self.vane_trim[i] for i in range(4)]

    def _att_world(self, x: list[float]) -> np.ndarray:
        """v0.9: the tether hardpoint in the world frame for state x (CG + R r_b)."""
        R = quat.to_matrix(np.array(x[6:10]))
        return np.array(x[0:3]) + R @ np.array(self.att_rb)

    def _att_vel_world(self, x: list[float]) -> np.ndarray:
        """v0.9: the hardpoint's world velocity v + R (w x r_b)."""
        R = quat.to_matrix(np.array(x[6:10]))
        return np.array(x[3:6]) + R @ np.cross(np.array(x[10:13]), np.array(self.att_rb))

    def surface_z(self, xe: float, ye: float) -> float:
        """VTOL landing surface height under the exit-plane centre (xe, ye): pad top 0 on the plate, else ground_z."""
        return 0.0 if xe * xe + ye * ye <= self.pad_radius_m * self.pad_radius_m else self.ground_z

    def pulse_width_s(self, channel: int, value: float) -> float:
        """Servo pulse width for a vane command (``mixer.pwm``); the ESC pulse is
        taken as the servo centre width (its 0.15 s thrust lag dwarfs +-0.5 ms)."""
        if channel < 4:
            d = math.degrees(value)
            mp = self.ctrl.mixer.p
            return (mp.pwm_center_us + mp.servo_signs[channel] * d * mp.pwm_us_per_deg + mp.servo_trims_us[channel]) * 1e-6
        return self.pwm_center_s

    def make_latch(self, init: list[float]) -> OutputLatch:
        tim = self.sc.timing
        return OutputLatch(self.dt_c, tim.delay_s(self.dt_c), self.frame_hz, self.channel_phase_s, init,
                           self.pulse_width_s if tim.pulse_decode else None)

    # ------------------------------------------------------------------ #
    # environment
    # ------------------------------------------------------------------ #
    def wind(self, t: float, t_launch: float | None) -> tuple[float, float, float]:
        sc = self.sc
        th = math.radians(sc.wind_dir_deg)
        wx, wy = sc.wind_mps * math.cos(th), sc.wind_mps * math.sin(th)
        t0 = 0.0 if t_launch is None else t_launch
        if sc.gust_mps and (t_launch is not None or sc.bypass_state_machine):
            tg = t0 + sc.gust_t0_after_launch_s
            if tg <= t < tg + sc.gust_duration_s:
                thg = math.radians(sc.gust_dir_deg if sc.gust_dir_deg is not None else sc.wind_dir_deg)
                wx += sc.gust_mps * math.cos(thg)
                wy += sc.gust_mps * math.sin(thg)
        return wx, wy, 0.0

    def battery_v_cell(self, energy_j: float, u: float) -> float:
        o = self.sc.options
        return o.batt_v_full_cell - o.batt_v_dod_slope * energy_j / self.e_cap_j - o.batt_sag_v_full_throttle * u * u

    def wind_mean_vtol(self, k: int, t: float, walk: np.ndarray | None) -> tuple[float, float]:
        """VTOL mean wind (world, horizontal) at tick k / time t: speed wind_mps x (1 + frac sin(2 pi t / P_s)) toward
        the direction wind_dir_deg + swing A sin(2 pi t / P + phase) + the random-walk offset ``walk[k]`` + the reversal
        (0 -> 180 deg with a 1 - cos blend over wind_reversal_s from wind_reversal_t_s).  No turbulence, no gusts."""
        sc = self.sc
        deg = sc.wind_dir_deg
        if sc.wind_swing_deg:
            deg += sc.wind_swing_deg * math.sin(2.0 * math.pi * t / sc.wind_swing_period_s
                                                + math.radians(sc.wind_swing_phase_deg))
        if walk is not None:
            deg += float(walk[k])
        if sc.wind_reversal_t_s is not None and t > sc.wind_reversal_t_s:
            s = min((t - sc.wind_reversal_t_s) / max(sc.wind_reversal_s, 1e-9), 1.0)
            deg += (1.0 if sc.wind_reversal_sign >= 0.0 else -1.0) * 180.0 * 0.5 * (1.0 - math.cos(math.pi * s))
        spd = sc.wind_mps
        if sc.wind_speed_mod_frac:
            spd *= 1.0 + sc.wind_speed_mod_frac * math.sin(2.0 * math.pi * t / sc.wind_speed_mod_period_s)
        th = math.radians(deg)
        return spd * math.cos(th), spd * math.sin(th)

    def wind_direction_walk(self, n: int) -> np.ndarray | None:
        """Random-walk direction offset [deg] per control tick: the rate is a first-order Gauss-Markov process (std
        wind_dir_rate_sigma_dps, correlation WIND_DIR_RATE_TAU_S_ASSUMED, stationary start), integrated; its own random
        stream (the v0.6.3 draws and the turbulence stream are unchanged)."""
        sc = self.sc
        if not sc.wind_dir_rate_sigma_dps > 0.0:
            return None
        rng = np.random.default_rng([int(sc.seed), 0x3D1E])
        a = math.exp(-self.dt_c / WIND_DIR_RATE_TAU_S_ASSUMED)
        b = math.sqrt(1.0 - a * a) * sc.wind_dir_rate_sigma_dps
        w = rng.normal(size=n)
        out = np.empty(n)
        r = sc.wind_dir_rate_sigma_dps * w[0]
        d = 0.0
        for i in range(n):
            out[i] = d
            d += r * self.dt_c
            r = a * r + b * w[i]
        return out

    def push_state(self, t: float) -> list[tuple[int, Push, float]]:
        """Pushes active at time t: (index, push, profile factor > 0)."""
        out = []
        for i, pu in enumerate(self.sc.pushes):
            f = pu.factor(t)
            if f > 0.0:
                out.append((i, pu, f))
        return out

    def push_contact_body(self, pu: Push, q: np.ndarray) -> tuple[float, float, float]:
        """Contact point of push ``pu`` in the body frame relative to the CG [m] for the attitude q at the push start:
        on the body surface (radius ``body_radius_mm`` at z_body around the BODY AXIS) facing the person (opposite the
        force direction), turned by offset_deg about the world vertical, projected onto the body's cross-section; minus
        the plant's CG offset from that axis (``cg_offset_mm``, fix round: a central push then has the lever arm of the
        offset about the long axis, as the physics review asked)."""
        a = math.radians(pu.azimuth_deg + 180.0 + pu.offset_deg)
        n_w = np.array([math.cos(a), math.sin(a), 0.0])
        n_b = quat.to_matrix(q).T @ n_w
        nh = math.hypot(float(n_b[0]), float(n_b[1]))
        nx, ny = (float(n_b[0]) / nh, float(n_b[1]) / nh) if nh > 1e-9 else (1.0, 0.0)
        r = body_radius_mm(self.body_layout, pu.z_body_m * 1e3) * 1e-3
        dx, dy = self.cg_off if self.cg_off is not None else (0.0, 0.0)
        return r * nx - dx, r * ny - dy, pu.z_body_m - self.cfg_full.z_cg

    POLE_MAX_ELEV_DEG = 80.0        # a pole steeper than this is not a push (clipped; the scenario says where the hand is)

    def pole_direction(self, pu: Push, contact_w: np.ndarray) -> tuple[tuple[float, float, float], float]:
        """v0.8 pole push: the unit force direction (world) along the pole from the hand to the contact point and its
        elevation [deg] (+ = pushing upward).  The hand is ``hand_height_m`` above the ground the person stands on
        (``ground_z``), on the person's side (opposite ``azimuth_deg``) at the horizontal distance the pole length leaves:
        sin(elevation) = (z_contact - z_hand) / pole_length (clipped to +-POLE_MAX_ELEV_DEG)."""
        z_hand = self.ground_z + float(pu.hand_height_m)
        s = (float(contact_w[2]) - z_hand) / float(pu.pole_length_m)
        lim = math.sin(math.radians(self.POLE_MAX_ELEV_DEG))
        el = math.asin(min(max(s, -lim), lim))
        a = math.radians(pu.azimuth_deg)
        ce = math.cos(el)
        return (ce * math.cos(a), ce * math.sin(a), math.sin(el)), math.degrees(el)

    def push_force(self, ip: int, pu: Push, t: float, push_dir: dict) -> tuple[float, float, float]:
        """World force of push ``ip`` at time t: the horizontal model (``Push.force_world``, bit-identical to v0.7.1) or,
        for a pole push, ``force_n`` x profile along the direction fixed at the push start (``pole_direction``)."""
        if not pu.pole:
            return pu.force_world(t)
        f = pu.factor(t)
        u = push_dir.get(ip)
        if f <= 0.0 or u is None:
            return 0.0, 0.0, 0.0
        fm = f * pu.force_n
        return fm * u[0], fm * u[1], fm * u[2]

    def wind_vtol(self, k: int, t: float, t_launch: float | None, turb: np.ndarray | None,
                  walk: np.ndarray | None = None) -> tuple[tuple[float, float, float], bool, tuple[float, float]]:
        """VTOL wind (world): the mean wind (``wind_mean_vtol``: steady + direction swing / walk / reversal + speed
        modulation) + the scenario's single gust (relative to the motor start, as the ballistic ``wind``) + the ``Gust``
        schedule (absolute) + turbulence sample k.  Returns (wind, a gust is active, the mean wind (x, y))."""
        sc = self.sc
        mx, my = self.wind_mean_vtol(k, t, walk)
        wx, wy, wz = mx, my, 0.0
        active = False
        if sc.gust_mps and t_launch is not None:
            tg = t_launch + sc.gust_t0_after_launch_s
            if tg <= t < tg + sc.gust_duration_s:
                thg = math.radians(sc.gust_dir_deg if sc.gust_dir_deg is not None else sc.wind_dir_deg)
                wx += sc.gust_mps * math.cos(thg)
                wy += sc.gust_mps * math.sin(thg)
                active = True
        for g in sc.gusts:
            f = g.factor(t)
            if f > 0.0:
                thg = math.radians(g.dir_deg)
                wx += f * g.speed_mps * math.cos(thg)
                wy += f * g.speed_mps * math.sin(thg)
                wz += f * g.vertical_mps
                active = True
        if turb is not None:
            wx += float(turb[k, 0])
            wy += float(turb[k, 1])
            wz += float(turb[k, 2])
        return (wx, wy, wz), active, (mx, my)

    def turbulence(self, n: int) -> np.ndarray | None:
        """First-order Gauss-Markov turbulence per axis at the control rate (own random stream, so the v0.6.3 draws of
        ``self.rng`` are unchanged): x_{k+1} = a x_k + sqrt(1 - a^2) sigma n_k, a = exp(-dt / tau), stationary start."""
        sc = self.sc
        if not sc.turbulence_sigma_mps > 0.0:
            return None
        rng = np.random.default_rng([int(sc.seed), 0x7E1B])
        sig = np.array([1.0, 1.0, sc.turbulence_vertical_ratio]) * sc.turbulence_sigma_mps
        a = math.exp(-self.dt_c / max(sc.turbulence_tau_s, 1e-6))
        w = rng.normal(size=(n, 3))
        out = np.empty((n, 3))
        out[0] = sig * w[0]
        b = math.sqrt(1.0 - a * a)
        for i in range(1, n):
            out[i] = a * out[i - 1] + b * sig * w[i]
        return out

    # ------------------------------------------------------------------ #
    # dynamics
    # ------------------------------------------------------------------ #
    def deriv(self, x: list[float], cfg: _MassCfg, t_cmd: float, dcmd: list[float], wind: tuple[float, float, float],
              chute_cda: float, frozen: int, ext: tuple | None = None, teth: tuple | None = None) -> list[float]:
        """Time derivative of the 18-state vector (pure-float inner loop).  ``ext`` (VTOL push): (F world x, y, z [N],
        contact point body x, y, z relative to the CG [m]); VTOL runs replace the body aerodynamics by the v0.7.1
        cross-flow + ram-drag model (``vtol_aero_geometry``)."""
        (px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz, T, d1, d2, d3, d4) = x
        ww, xx, yy, zz = qw * qw, qx * qx, qy * qy, qz * qz
        r00 = ww + xx - yy - zz
        r01 = 2.0 * (qx * qy - qw * qz)
        r02 = 2.0 * (qx * qz + qw * qy)
        r10 = 2.0 * (qx * qy + qw * qz)
        r11 = ww - xx + yy - zz
        r12 = 2.0 * (qy * qz - qw * qx)
        r20 = 2.0 * (qx * qz - qw * qy)
        r21 = 2.0 * (qy * qz + qw * qx)
        r22 = ww - xx - yy + zz
        # air-relative velocity (world), then body
        ux, uy, uz = vx - wind[0], vy - wind[1], vz - wind[2]
        vbx = r00 * ux + r10 * uy + r20 * uz
        vby = r01 * ux + r11 * uy + r21 * uz
        vbz = r02 * ux + r12 * uy + r22 * uz
        V = math.sqrt(vbx * vbx + vby * vby + vbz * vbz)
        # thrust, swirl, vanes
        Tp = T if T > 0.0 else 0.0
        q_e = Tp * self.inv_2ae
        Fx = 0.0
        Fy = 0.0
        Fz = Tp
        if self.ge_on:
            # fix round (physics review medium 3, ASSUMED shape): ground effect of the exit plane over the surface the
            # jet meets - the ground, or over the pad the floor under its 102 mm hole (the pad foot level)
            xe_, ye_ = px - cfg.z_cg * r02, py - cfg.z_cg * r12
            pr_ = self.pad_radius_m
            surf_ = self.pad_foot_z_m if xe_ * xe_ + ye_ * ye_ <= pr_ * pr_ else self.ground_z
            hge = pz - cfg.z_cg * r22 - surf_
            ge = math.exp(-self.ge_k * hge) if hge > 0.0 else 1.0
            Fz = Tp * (1.0 + self.ge_a_t * ge)
            q_e *= max(1.0 - self.ge_a_q * ge, 0.0)
        Mx = 0.0
        My = 0.0
        Mz = self.swirl_per_N * Tp
        if self.cg_off is not None:
            # VTOL campaign: CG (dx, dy) off the duct axis; the thrust acts on the axis, r = (-dx, -dy, .) from the CG
            Mx -= self.cg_off[1] * Tp
            My += self.cg_off[0] * Tp
        ds = (d1, d2, d3, d4)
        stall = self.stall
        kf = q_e * self.sv_cla
        ex, ey, b0, b1, b2 = self.ex, self.ey, cfg.b0, cfg.b1, cfg.b2
        play = self.play_on
        mem, ph = self.vane_mem, self.play_h
        for i in range(4):
            d = ds[i]
            if play:
                # v0.9 direct-drive play: the vane angle = the play operator of the servo output (state x[14 + i])
                lo, hi = d - ph[i], d + ph[i]
                d = lo if mem[i] < lo else (hi if mem[i] > hi else mem[i])
            if d > stall:
                d = stall
            elif d < -stall:
                d = -stall
            Fi = kf * d
            Fx += Fi * ex[i]
            Fy += Fi * ey[i]
            Mx += Fi * b0[i]
            My += Fi * b1[i]
            Mz += Fi * b2[i]
        # body aerodynamics
        if self.vtol:
            # v0.7.1 VTOL (params aero.vtol_aero_note): axial drag on the axial speed; cross-flow drag
            # 1/2 rho C_dc A_plan |V_perp| V_perp at the side-silhouette centroid; EDF ram drag mdot V_perp at the intake
            # station, mdot = sqrt(rho A_exit T); V_perp = the body-xy air-relative velocity of each station (CG velocity
            # + omega x r)
            Fz -= self.vt_axial_k * abs(vbz) * vbz
            lc = self.vt_z_cf - cfg.z_cg
            cx = vbx + wy * lc
            cy = vby - wx * lc
            kc = self.vt_kc * math.sqrt(cx * cx + cy * cy)
            fcx, fcy = -kc * cx, -kc * cy
            Fx += fcx
            Fy += fcy
            Mx -= lc * fcy
            My += lc * fcx
            if self.vt_ram_k > 0.0 and Tp > 0.0:
                li = self.vt_z_in - cfg.z_cg
                md = math.sqrt(self.vt_ram_k * Tp) * self.vt_ram_scale
                frx, fry = -md * (vbx + wy * li), -md * (vby - wx * li)
                Fx += frx
                Fy += fry
                Mx -= li * fry
                My += li * frx
        elif V > 1e-9:
            c = 0.5 * self.rho * V * self.a_ref
            cd, cn = c * self.cd, c * self.cna
            fnx, fny = -cn * vbx, -cn * vby
            Fx += -cd * vbx + fnx
            Fy += -cd * vby + fny
            Fz += -cd * vbz
            Mx -= cfg.l_cp * fny
            My += cfg.l_cp * fnx
        if self.rot_on:
            # v0.9 rotor: gyroscopic moment -w x h (h = -s I omega(T) z_b) and the spool-up reaction +s I domega/dt
            rt = self.rotor
            om = rt.omega_ref_rad_s * math.sqrt(Tp / rt.t_fan_n)
            hz = -self.rot_s_i * om
            Mx -= wy * hz
            My += wx * hz
            tf = Tp if Tp > rt.t_floor_frac * rt.t_fan_n else rt.t_floor_frac * rt.t_fan_n
            Mz += self.rot_s_i * rt.omega_ref_rad_s * ((t_cmd - T) * self.inv_tau_T) / (2.0 * math.sqrt(rt.t_fan_n * tf))
        if teth is not None:
            # v0.9 tether: catch line (spring-damper when taut) + the cable / snag force (world, held over the step) at
            # the hardpoint rb (body, relative to the CG)
            ax_, ay_, az_, rbx, rby, rbz, l0, kb, cb, fcx, fcy, fcz, l0d = teth
            pxa = px + r00 * rbx + r01 * rby + r02 * rbz
            pya = py + r10 * rbx + r11 * rby + r12 * rbz
            pza = pz + r20 * rbx + r21 * rby + r22 * rbz
            cwx, cwy, cwz = wy * rbz - wz * rby, wz * rbx - wx * rbz, wx * rby - wy * rbx
            vxa = vx + r00 * cwx + r01 * cwy + r02 * cwz
            vya = vy + r10 * cwx + r11 * cwy + r12 * cwz
            vza = vz + r20 * cwx + r21 * cwy + r22 * cwz
            fkx, fky, fkz, _ = tt.catch_line_force((ax_ - pxa, ay_ - pya, az_ - pza), (vxa, vya, vza), l0, kb, cb, l0d)
            twx, twy, twz = fkx + fcx, fky + fcy, fkz + fcz
            tbx = r00 * twx + r10 * twy + r20 * twz
            tby = r01 * twx + r11 * twy + r21 * twz
            tbz = r02 * twx + r12 * twy + r22 * twz
            Fx += tbx
            Fy += tby
            Fz += tbz
            Mx += rby * tbz - rbz * tby
            My += rbz * tbx - rbx * tbz
            Mz += rbx * tby - rby * tbx
        if ext is not None:
            # hand push: world force -> body, applied at the contact point (body, relative to the CG)
            ewx, ewy, ewz, rx, ry, rz = ext
            pfx = r00 * ewx + r10 * ewy + r20 * ewz
            pfy = r01 * ewx + r11 * ewy + r21 * ewz
            pfz = r02 * ewx + r12 * ewy + r22 * ewz
            Fx += pfx
            Fy += pfy
            Fz += pfz
            Mx += ry * pfz - rz * pfy
            My += rz * pfx - rx * pfz
            Mz += rx * pfy - ry * pfx
        # parachute (world drag along -V_rel at the tether point) + line damping
        if chute_cda > 0.0:
            if V > 1e-9:
                cc = 0.5 * self.rho * chute_cda * V
                fwx, fwy, fwz = -cc * ux, -cc * uy, -cc * uz
                fbx = r00 * fwx + r10 * fwy + r20 * fwz
                fby = r01 * fwx + r11 * fwy + r21 * fwz
                fbz = r02 * fwx + r12 * fwy + r22 * fwz
                Fx += fbx
                Fy += fby
                Fz += fbz
                Mx -= cfg.l_att * fby
                My += cfg.l_att * fbx
            cr = self.sc.options.chute_rot_damping_nms
            Mx -= cr * wx
            My -= cr * wy
            Mz -= cr * wz
        # translation
        im = cfg.inv_m
        ax = (r00 * Fx + r01 * Fy + r02 * Fz) * im
        ay = (r10 * Fx + r11 * Fy + r12 * Fz) * im
        az = (r20 * Fx + r21 * Fy + r22 * Fz) * im - self.g
        # rotation: I wdot = M - w x (I w)
        I = cfg.I
        Lx = I[0][0] * wx + I[0][1] * wy + I[0][2] * wz
        Ly = I[1][0] * wx + I[1][1] * wy + I[1][2] * wz
        Lz = I[2][0] * wx + I[2][1] * wy + I[2][2] * wz
        gx = Mx - (wy * Lz - wz * Ly)
        gy = My - (wz * Lx - wx * Lz)
        gz = Mz - (wx * Ly - wy * Lx)
        J = cfg.invI
        wdx = J[0][0] * gx + J[0][1] * gy + J[0][2] * gz
        wdy = J[1][0] * gx + J[1][1] * gy + J[1][2] * gz
        wdz = J[2][0] * gx + J[2][1] * gy + J[2][2] * gz
        # quaternion kinematics dq/dt = 1/2 q (x) (0, w)
        qdw = -0.5 * (qx * wx + qy * wy + qz * wz)
        qdx = 0.5 * (qw * wx + qy * wz - qz * wy)
        qdy = 0.5 * (qw * wy - qx * wz + qz * wx)
        qdz = 0.5 * (qw * wz + qx * wy - qy * wx)
        # thrust lag
        tdot = (t_cmd - T) * self.inv_tau_T
        # servos: lag + rate limit (saturation applied after the step)
        rate, its = self.servo_rate, self.inv_tau_s
        dd = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            if frozen >= 0 and i == frozen:
                continue
            r = (dcmd[i] - ds[i]) * its
            if r > rate:
                r = rate
            elif r < -rate:
                r = -rate
            dd[i] = r
        return [vx, vy, vz, ax, ay, az, qdw, qdx, qdy, qdz, wdx, wdy, wdz, tdot, dd[0], dd[1], dd[2], dd[3]]

    def rk4(self, x: list[float], h: float, args: tuple) -> list[float]:
        f = self.deriv
        k1 = f(x, *args)
        x2 = [xi + 0.5 * h * ki for xi, ki in zip(x, k1)]
        k2 = f(x2, *args)
        x3 = [xi + 0.5 * h * ki for xi, ki in zip(x, k2)]
        k3 = f(x3, *args)
        x4 = [xi + h * ki for xi, ki in zip(x, k3)]
        k4 = f(x4, *args)
        xn = [xi + h / 6.0 * (a + 2.0 * b + 2.0 * c + d) for xi, a, b, c, d in zip(x, k1, k2, k3, k4)]
        # renormalise q, clamp servo travel, no negative thrust
        n = math.sqrt(xn[6] ** 2 + xn[7] ** 2 + xn[8] ** 2 + xn[9] ** 2)
        xn[6] /= n
        xn[7] /= n
        xn[8] /= n
        xn[9] /= n
        if xn[13] < 0.0:
            xn[13] = 0.0
        dm = self.delta_mech
        for i in range(14, 18):
            if xn[i] > dm:
                xn[i] = dm
            elif xn[i] < -dm:
                xn[i] = -dm
        return xn

    # ------------------------------------------------------------------ #
    # run
    # ------------------------------------------------------------------ #
    def run(self) -> SimResult:
        t_wall = time.perf_counter()
        sc, sens, opt = self.sc, self.sc.sensors, self.sc.options
        dt_c = self.dt_c
        n_max = int(round(sc.t_end / dt_c)) + 1
        rng = self.rng
        imu_hz = float(self.p["control"]["imu_hz"])
        baro_hz = float(self.p["control"]["baro_hz"])
        n_baro = max(1, int(round(self.ctrl_hz / baro_hz)))
        n_lat = int(round(sens.baro_latency_s * baro_hz))
        mode = sens.attitude_estimator
        n_pre = int(round(sens.pad_prealign_s / dt_c)) if mode != "truth_noise" else 0
        n_tot = n_pre + n_max

        # ---- pre-generated noise and bias (vectorised; index j = n_pre + k) -----
        sig_g = white_noise_sigma(sens.gyro_noise_density_rad_s_sqrthz, sens.imu_dlpf_tau_s, imu_hz)
        sig_a = white_noise_sigma(sens.accel_noise_density_mps2_sqrthz, sens.imu_dlpf_tau_s, imu_hz)
        gyro_noise = rng.normal(0.0, sig_g, size=(n_tot, 3))
        accel_noise = rng.normal(0.0, sig_a, size=(n_tot, 3))
        att_noise = rng.normal(0.0, sens.att_noise_rad, size=(n_max, 3))
        baro_noise = rng.normal(0.0, sens.baro_noise_m, size=n_max // n_baro + 2)
        baro_q = float(sens.baro_quant_m)
        ramp_start = n_pre + (0 if sc.bypass_state_machine else int(round(sc.launch_time_s / dt_c)))
        gyro_bias = gyro_bias_process(rng, n_tot, dt_c, sens.gyro_bias_sigma_rad_s, sens.gyro_bias_tau_s,
                                      sens.gyro_bias_max_rad_s,
                                      None if sens.gyro_bias_ramp_rad_s2 is None else np.asarray(sens.gyro_bias_ramp_rad_s2),
                                      ramp_start)
        ab = rng.normal(size=3)
        accel_bias = ab / np.linalg.norm(ab) * sens.accel_bias_mps2
        imu = self.imu

        # ---- initial state -----------------------------------------------------
        cfg = self.cfg_full
        axis = np.array([math.cos(math.radians(sc.initial_tilt_axis_deg)), math.sin(math.radians(sc.initial_tilt_axis_deg)), 0.0])
        yaw0_deg = sc.initial_yaw_deg
        if self.gps_on:
            # v0.8: the vehicle is set up on the pad with its +X toward the pad azimuth (compass bearing, W: x north, y
            # west -> heading -azimuth) plus the set-up error; initial_yaw_deg adds to it
            yaw0_deg = sc.initial_yaw_deg + float(sc.gps.heading_err_deg) - float(self.gp_nav.pad_azimuth_deg)
        q0 = quat.multiply(quat.from_yaw(math.radians(yaw0_deg)),
                           quat.from_axis_angle(axis, math.radians(sc.initial_tilt_deg)))
        R0 = quat.to_matrix(q0)
        pos0 = R0 @ np.array([0.0, 0.0, cfg.z_cg]) + np.array([0.0, 0.0, sc.initial_altitude_m])
        x = [float(pos0[0]), float(pos0[1]), float(pos0[2]), 0.0, 0.0, 0.0,
             float(q0[0]), float(q0[1]), float(q0[2]), float(q0[3]), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        hover_u = self.veh.hover_throttle
        u_fixed = 0.0
        if sc.bypass_state_machine:
            u_fixed = hover_u if sc.fixed_throttle is None else float(sc.fixed_throttle)
            x[13] = self.edf.thrust_steady(u_fixed)  # start with the lag settled
        on_ground = sc.initial_altitude_m <= 0.0 and not sc.bypass_state_machine
        pad_z_bottom = 0.0
        q_rest = q0.copy()        # attitude the ground holds (pad before liftoff, landing attitude after)
        z_cg_ref = x[2]  # altitude h is measured as CG rise above its pad height

        # ---- estimators: pad alignment before t = 0 (vehicle static at q0) -----
        vt = self.vtol
        if mode == "gated_mahony":
            # v0.7.1 VTOL NAV estimator (params mission.vtol.nav_estimator <- the tuned est_* keys of Scenario.vgains
            # (sim/out/vtol_gains.json) <- SensorConfig gm_* overrides)
            vga = sc.vgains or {}

            def _eg(sv: float | None, key: str) -> float | None:
                return sv if sv is not None else (None if vga.get(key) is None else float(vga[key]))
            est = GatedMahonyEstimator.from_params(self.p, dt_c, sens.pad_avg_tau_s, g=G0, kp=_eg(sens.gm_kp, "est_kp"),
                                                   ki=_eg(sens.gm_ki, "est_ki"),
                                                   accel_lpf_hz=_eg(sens.gm_accel_lpf_hz, "est_accel_lpf_hz"),
                                                   accel_gate=sens.gm_accel_gate)
        else:
            est = PadAlignGyroEstimator(dt_c, sens.pad_avg_tau_s)
        pad_est = mode in ("pad_gyro", "gated_mahony")        # estimators with the firmware's pad alignment
        q_est = q0.copy() if mode == "truth_noise" else quat.IDENTITY.copy()
        mahony_bias = np.zeros(3)
        f_static = R0.T @ np.array([0.0, 0.0, self.g])
        zero3 = np.zeros(3)
        valt = self.valt
        # ---- v0.8 GPS: receiver (plant), the NAV's GPS part, the nlink slot transmitter (None without the GPS) ----
        gps_rx: GpsReceiverPlant | None = None
        nav_gps = slot_tx = None
        n_boot = int(round(sens.pad_prealign_s / dt_c))       # NAV / GPS power-on = the start of the pad pre-alignment
        r_ant_b = np.zeros(3)
        nav_baud = [gn.GPS_BAUD]                               # the NAV's USART1 baud (the configurator switches it)
        gns_chain = self.gps_on and self.chain == "gns"
        gl_sent: list = [None]                                 # v0.8.1: the STATE frame the NAV sent this tick (glink)
        f4_view: list = [None]                                 # v0.8.1: the NAV's last valid nlink v4 FCU frame (lagged)
        if self.gps_on:
            gps_rx = GpsReceiverPlant(sc.gps, sc.seed, -n_boot * dt_c)
            if gns_chain:
                # v0.8.1 four chips: the receiver is on the GNS board (its USART1); the NAV's glink part sends the STATE
                # frames and fills the nlink v4 slots from the GNS's RESULT frames (sim.glink_proto)
                nav_gps = gl.GnsNode(self.gp_nav, self.pp, configure=True)
                slot_tx = gl.GlinkNav(self.gp_nav.step_ticks, self.glink_L)
            else:
                nav_gps = gn.NavGps(self.gp_nav, configure=True)
                slot_tx = nl.GpsSlotTx(self.gp_nav.step_ticks)
            cxy0 = self.cg_off if self.cg_off is not None else (0.0, 0.0)
            # the TRUE lever arm: the CAD phase centre minus the plant CG (the NAV subtracts its own params / CAD value)
            r_ant_b = np.array([self.ant_b_mm[0] * 1e-3 - cxy0[0], self.ant_b_mm[1] * 1e-3 - cxy0[1],
                                self.ant_b_mm[2] * 1e-3 - cfg.z_cg])
        st_n = self.gp_nav.step_ticks if self.gp_nav is not None else 5
        pos_src = nav_gps if gns_chain else self.fcu          # where the position controller runs (logs, bookkeeping)

        def gps_nav_tick(jn: int, t_: float, p_ant: Any, v_ant: Any, accel_: Any, q_corr_: Any, rate_: Any,
                         on_pad_: bool, h_: float = 0.0) -> None:
            """The receiver up to t_ and the NAV's GPS part of NAV tick jn (v0.8 order step 6): UART bytes -> parser ->
            configurator (its UART actions back to the receiver) / epochs -> filter; snapshot latched for the slots.
            v0.8.1 (gns chain): the GNS polls the receiver, then takes the STATE frame the NAV sent in tick jn - 1 (its
            RESULT back to the NAV's glink receiver in this tick), then the NAV's glink part of tick jn (force sum, the
            STATE frame at jn mod 5 == 4)."""
            gps_rx.step(t_, p_ant, v_ant)
            if gns_chain:
                acts = nav_gps.poll(jn, gps_rx.deliver(t_, nav_baud[0]))
                if gl_sent[0] is not None and _in_window(t_, sc.gps.glink_cut) is None:
                    for fr_ in nav_gps.rx_bytes(gl_sent[0]):
                        slot_tx.rx_bytes(fr_, jn)
                f4 = f4_view[0]
                gl_sent[0] = slot_tx.tick(jn, accel_, q_corr_, rate_, h_, on_pad_, f4, f4 is not None)
            else:
                acts, snap = nav_gps.tick(jn, gps_rx.deliver(t_, nav_baud[0]), accel_, q_corr_, rate_, on_pad_)
            for a_ in acts:
                if a_[0] == "baud":
                    nav_baud[0] = int(a_[1])
                elif a_[0] == "tx":
                    gps_rx.nav_tx(t_, a_[1], nav_baud[0])
            if not gns_chain and jn % st_n == st_n - 1:
                slot_tx.latch(snap, nav_gps.filter.cycle)

        if self.gps_on and n_pre < n_boot:
            # an estimator without the pad pre-alignment (truth_noise what-if): the GPS still powered up with the NAV
            ant0 = np.asarray(pos0) + R0 @ r_ant_b
            for jb in range(n_boot - n_pre):
                gps_nav_tick(jb, (jb - n_boot) * dt_c, ant0, zero3, f_static + accel_bias, q0, zero3, True)
        for j in range(n_pre):
            gyr = imu.gyro(zero3, gyro_bias[j], gyro_noise[j])
            acc = imu.accel(f_static, accel_bias, accel_noise[j])
            if pad_est:
                q_est = est.update(gyr, acc, on_pad=True)
            else:
                q_est, mahony_bias = self._mahony(q_est, mahony_bias, gyr, acc, dt_c)
            if vt:
                # VTOL NAV altitude filter: the pad calibration of the accelerometer z-bias runs with the alignment
                valt.pad(float((quat.to_matrix(q_est) @ acc)[2]) - self.g, dt_c)
            if gps_rx is not None:
                jn = n_boot - n_pre + j
                gps_nav_tick(jn, (jn - n_boot) * dt_c, np.asarray(pos0) + R0 @ r_ant_b, zero3, acc, q_est,
                             gyr - est.bias if pad_est else gyr, True)
        w_dlpf = np.zeros(3)      # truth body rate through the MPU-6050 on-chip DLPF
        # ---- v0.9 plant logs / state ----
        self.vane_mem = [0.0, 0.0, 0.0, 0.0]
        self.t_lim = math.inf
        self.meas = {}
        L9: dict[str, np.ndarray] = {}
        if self.play_on:
            L9["delta_servo"] = np.zeros((n_max, 4))  # servo outputs (the vanes follow them through the play)
        if self.rot_on:
            L9["rotor_omega"] = np.zeros(n_max)
        if self.pw is not None:
            for k_ in ("v_bus", "i_bus", "supply_v", "supply_a", "buffer_a", "buffer_v", "batt_soc", "v_in_meas"):
                L9[k_] = np.zeros(n_max)
            L9["buffer_soc"] = np.full(n_max, np.nan)
            L9["supply_state"] = np.zeros(n_max, dtype=np.int8)
            L9["tether_connected"] = np.zeros(n_max, dtype=bool)      # tether power present at the vehicle
            L9["cable_attached"] = np.zeros(n_max, dtype=bool)        # the power cable still plugged / not cut
            L9["thrust_limited"] = np.zeros(n_max, dtype=bool)
            if self.pw.cfg.regen_eff > 0.0:
                L9["p_regen_w"] = np.zeros(n_max)                       # fix round: regeneration into the bus
        if self.rig is not None:
            for k_ in ("tether_catch_n", "tether_cable_n", "tether_tension_n", "tether_l0", "tether_d"):
                L9[k_] = np.zeros(n_max)
            L9["tether_locked"] = np.zeros(n_max, dtype=bool)              # fix round: the catch-line device locked
            if self.rig.cfg.feed == "slack":
                L9["tether_loop_depth"] = np.zeros(n_max)                  # fix round: the cable loop below the eye bolt
                L9["tether_loop_x"] = np.zeros(n_max)
            self.rig.begin(self._att_world(x))
        tau_dlpf = sens.imu_dlpf_tau_s
        self.alt.reset(0.0)
        alt = valt if vt else self.alt                         # the NAV altitude filter of this build
        baro_queue: deque[float] = deque()
        v_prev = np.zeros(3)

        # ---- actuator interface: compute delay + PWM-frame ZOH + pulse decode ----
        latch_esc = None
        if vt:
            # v0.7.1 VTOL tick order: the vanes keep the measured delay, the ESC register comes after the supervisor and
            # the vertical controller (FCTiming.esc_delay_s); one latch per path, the same frame edges and pulse decode
            tim = sc.timing
            pw = self.pulse_width_s if tim.pulse_decode else None
            latch = OutputLatch(dt_c, tim.delay_s(dt_c), self.frame_hz, self.channel_phase_s[:4], [0.0] * 4, pw)
            latch_esc = OutputLatch(dt_c, tim.esc_delay_s(dt_c), self.frame_hz, self.channel_phase_s[4:5], [u_fixed],
                                    (lambda c, v: self.pulse_width_s(4, v)) if tim.pulse_decode else None)
            self.latch_esc = latch_esc
        else:
            latch = self.make_latch([0.0, 0.0, 0.0, 0.0, u_fixed])
        self.latch = latch
        srv = [0.0, 0.0, 0.0, 0.0]          # decoded servo commands (plant side)
        u_esc = u_fixed                     # decoded ESC command
        pending: list[tuple[float, int, float]] = []

        # ---- logs ----------------------------------------------------------------
        L: dict[str, np.ndarray] = {
            "t": np.zeros(n_max), "pos": np.zeros((n_max, 3)), "vel": np.zeros((n_max, 3)),
            "q": np.zeros((n_max, 4)), "omega": np.zeros((n_max, 3)), "thrust": np.zeros(n_max),
            "throttle": np.zeros(n_max), "throttle_esc": np.zeros(n_max), "delta": np.zeros((n_max, 4)),
            "delta_cmd": np.zeros((n_max, 4)), "delta_srv": np.zeros((n_max, 4)),
            "moments": np.zeros((n_max, 3)), "rate_sp": np.zeros((n_max, 3)), "att_err": np.zeros((n_max, 3)),
            "state": np.zeros(n_max, dtype=np.int8), "h_est": np.zeros(n_max), "vz_est": np.zeros(n_max),
            "h_baro": np.full(n_max, np.nan), "energy_j": np.zeros(n_max), "saturated": np.zeros(n_max, dtype=bool),
            "tilt_deg": np.zeros(n_max), "tilt_vec": np.zeros((n_max, 3)), "euler_deg": np.zeros((n_max, 3)),
            "gyro": np.zeros((n_max, 3)), "accel": np.zeros((n_max, 3)), "q_est": np.zeros((n_max, 4)),
            "tilt_est_deg": np.zeros(n_max), "tilt_est_err_deg": np.zeros(n_max), "gyro_bias": np.zeros((n_max, 3)),
            "tilt_ctrl_deg": np.zeros(n_max), "tilt_ctrl_vec": np.zeros((n_max, 3)),
            "gyro_bias_est": np.zeros((n_max, 3)), "wind": np.zeros((n_max, 3)), "v_cell": np.zeros(n_max),
            "chute_cda": np.zeros(n_max), "control_active": np.zeros(n_max, dtype=bool),
            "frames_rx": np.zeros(n_max, dtype=np.int8), "link_ok": np.zeros(n_max, dtype=bool),
            "pwm_us": np.zeros((n_max, 4)), "integrator": np.zeros((n_max, 3)),
        }
        L.update(L9)
        events: list[tuple[float, str]] = []
        energy = 0.0
        t_launch: float | None = 0.0 if sc.bypass_state_machine else None
        t_liftoff: float | None = None
        t_touchdown: float | None = None
        touchdown_speed: float | None = None
        touchdown_vz: float | None = None
        t_deploy: float | None = None
        ejected = False
        chute_cda = 0.0
        frozen = -1
        failed = False
        stuck_rad = None if sc.vane_failure_stuck_deg is None else math.radians(sc.vane_failure_stuck_deg)
        prev_active = False
        prev_state = self.sup.state
        u = u_fixed
        v_cell = self.battery_v_cell(0.0, 0.0)
        nan_abort = False
        n_log = 0
        yaw_ref_set = False
        state = State.POWERED if sc.bypass_state_machine else self.sup.state
        gs, link, sup = self.ground, self.link, self.sup
        # the NAV's view of the supervisor state: the state after tick k - lag (v0.6.3: lag 2, the last nlink exchange)
        lag = int(sens.nav_state_lag_ticks)
        if lag < 1:
            raise ValueError("nav_state_lag_ticks must be >= 1 (the state of this tick is not known before the estimate)")
        nav_states: deque[State] = deque([sup.state] * lag, maxlen=lag)
        # ---- VTOL build (every VTOL statement below sits behind ``if vt``) ----
        if vt and sc.bypass_state_machine:
            raise ValueError("a VTOL scenario flies through the supervisor and the vertical controller (no bypass)")
        turb = self.turbulence(n_max) if vt else None
        walk = self.wind_direction_walk(n_max) if vt else None
        touchdowns: list[dict[str, Any]] = []
        liftoffs: list[float] = []
        rest_surface = 0.0                  # surface height the resting vehicle stands on (pad top at the start)
        t_state = 0.0                      # time of the last supervisor state change (VTOL stop rule)
        gust_on = False
        fo = None
        u_att = 0.0                         # VTOL: the attitude controller's gain-schedule throttle (previous tick's ESC u)
        acc_nav_z = self.g                  # VTOL: the NAV frame's body-z specific force (frozen with the IMU)
        wind_mean = (0.0, 0.0)
        push_rb: dict[int, tuple[float, float, float]] = {}   # VTOL push contact points (body, rel. CG), fixed at start
        if vt:
            # the NAV's view of the FCU's ESC throttle (nlink FCU byte 7, thr_q) with the same lag as the state
            nav_thr_q: deque[int] = deque([0] * lag, maxlen=lag)
            # v0.9 fix round (control review medium 6): the FCU may send its thrust-lagged throttle in thr_q (params
            # mission.vtol.nav_throttle_lagged) - the NAV's baro compensation then follows the thrust that set the
            # static-pressure error (the reference compensated the commanded throttle: the climb overshoot); FCU: the
            # rotor feed-forward's u_lag (no new state, no NAV change)
            nav_lag_a = (dt_c / (self.edf.tau_s + dt_c)) if self.nav_thr_lagged else 0.0
            nav_u_lag = 0.0
            # v0.8.1: the NAV's view of the nlink v4 FCU frame (pflags, k_T for the GNS) with the same lag
            nav_f4: deque = deque([None] * lag, maxlen=lag)
            vib_rms = float(sens.accel_vibration_rms_mps2)
            vib = (np.random.default_rng([int(sc.seed), 0x71B]).normal(size=(n_max, 3)) if vib_rms > 0.0 else None)
            w_hover_nom = self.veh.full.mass_kg * self.g        # nominal hover thrust (baro offset, vibration scale)
            L.update({
                "phase": np.zeros(n_max, dtype=np.int8), "h_tgt": np.full(n_max, np.nan), "h_ref": np.full(n_max, np.nan),
                "vz_cmd": np.full(n_max, np.nan), "u_ff": np.zeros(n_max), "vc_integ": np.zeros(n_max),
                "vc_sat": np.zeros(n_max, dtype=np.int8), "vc_mode": np.zeros(n_max, dtype=np.int8),
                "td_count": np.zeros(n_max, dtype=np.int16), "motor_on": np.zeros(n_max, dtype=bool),
                "baro_lost": np.zeros(n_max, dtype=bool), "reason": np.zeros(n_max, dtype=np.uint8),
                "h_true": np.zeros(n_max), "gust_active": np.zeros(n_max, dtype=bool),
                "imu_ok": np.ones(n_max, dtype=bool), "baro_ok": np.ones(n_max, dtype=bool),
                "surface_z": np.zeros(n_max),
                # v0.7.1
                "wind_mean": np.zeros((n_max, 3)), "push_force": np.zeros((n_max, 3)),
                "push_point": np.full((n_max, 3), np.nan), "push_active": np.zeros(n_max, dtype=bool),
                "vc_td_by": np.zeros(n_max, dtype=np.int8), "vc_low": np.zeros(n_max, dtype=bool),
                "vc_still": np.zeros(n_max, dtype=bool), "vc_support": np.zeros(n_max, dtype=bool),
                "vc_floor": np.zeros(n_max, dtype=bool), "vc_low_in_land": np.zeros(n_max, dtype=np.int16),
                "vc_vz_lpf": np.zeros(n_max), "vc_f_excess": np.zeros(n_max), "vc_thrust_ratio": np.zeros(n_max),
                "alt_bias": np.zeros(n_max), "baro_comp_m": np.zeros(n_max), "est_gate": np.zeros(n_max, dtype=bool),
                "att_u": np.zeros(n_max), "accel_nav_z": np.zeros(n_max),
                # fix round: the v0.7.1b thrust learning and the flare-band flag
                "vc_k_thrust": np.ones(n_max), "vc_kt_learn": np.zeros(n_max, dtype=bool),
                "vc_low_alt": np.zeros(n_max, dtype=bool),
            })
        v08 = self.v08
        h_pub = v_pub = 0.0                 # v0.8 NAV frame: the altitude filter of the PREVIOUS tick
        c_pub, tilt_pub = 1.0, 0.0          # v0.8 NAV frame: cos_tilt / tilt of the published q (nlink v3)
        slot_pub: bytes | None = None       # v0.8 NAV frame: the GPS slot of this tick
        q_corr = q_est                      # v0.8: the NAV's corrected attitude (altitude filter, GPS part)
        push_dir: dict[int, tuple[float, float, float]] = {}  # v0.8 pole pushes: unit force direction (world), fixed at start
        push_elev: dict[int, float] = {}
        hold_true: tuple | None = None      # v0.8: TRUE CG (x, y) at the position loop's latest hold capture
        hold_caps: list[dict[str, Any]] = []
        last_hold_rf: tuple | None = None
        nav_origin_w: tuple | None = None   # v0.8: the NAV frame's origin in the sim world (receiver's site frame)
        if v08:
            L.update({"cos_tilt_pub": np.ones(n_max), "h_pub": np.zeros(n_max)})
        if self.gps_on:
            L.update({
                "gps_ok": np.zeros(n_max, dtype=bool), "gps_gate_ok": np.zeros(n_max, dtype=bool),
                "gps_flags": np.zeros(n_max, dtype=np.uint8), "gps_sats": np.zeros(n_max, dtype=np.int16),
                "gps_hacc_m": np.full(n_max, np.nan), "gps_fix": np.zeros(n_max, dtype=np.int8),
                "gps_rx_fix": np.zeros(n_max, dtype=bool), "gps_err": np.full((n_max, 2), np.nan),
                "gps_err_slow": np.full((n_max, 2), np.nan), "gps_err_v": np.full((n_max, 2), np.nan),
                "pos_est": np.full((n_max, 3), np.nan), "vel_est_h": np.full((n_max, 2), np.nan),
                "nav_b": np.zeros((n_max, 2)), "gps_resets": np.zeros(n_max, dtype=np.int16),
                "psi_err_deg": np.zeros(n_max), "yaw_corr_deg": np.zeros(n_max),
                "pos_mode": np.zeros(n_max, dtype=np.int8), "pos_dist": np.full(n_max, np.nan),
                "pos_limited": np.zeros(n_max, dtype=bool), "pos_guard": np.zeros(n_max, dtype=bool),
                "pos_engaged": np.zeros(n_max, dtype=bool), "pos_requests": np.zeros(n_max, dtype=np.uint8),
                "pos_lost": np.zeros(n_max, dtype=np.int16), "pos_integ": np.zeros((n_max, 2)),
                "pos_a_cmd": np.zeros((n_max, 2)), "pos_v_sp": np.zeros((n_max, 2)), "pos_cos_lim": np.ones(n_max),
                "tilt_sp": np.zeros((n_max, 2)), "tilt_sp_deg": np.zeros(n_max),
                "hold_true": np.full((n_max, 2), np.nan), "hold_nav": np.full((n_max, 2), np.nan),
            })

        for k in range(n_max):
            t = k * dt_c
            j = n_pre + k
            n_log = k + 1
            # ---------------- truth -> sensors (sample at tick k) ----------------
            q = np.array(x[6:10])
            w = np.array(x[10:13])
            v = np.array(x[3:6])
            R = quat.to_matrix(q)
            acc_w = (v - v_prev) / dt_c if k > 0 else np.zeros(3)
            v_prev = v
            gyro = imu.gyro(w_dlpf, gyro_bias[j], gyro_noise[j])
            if vt and vib is not None:
                # VTOL: accelerometer vibration (ASSUMED RMS at hover thrust, x T / (m g)) on top of the sensor noise
                accel = imu.accel(R.T @ (acc_w + np.array([0.0, 0.0, self.g])), accel_bias,
                                  accel_noise[j] + vib[k] * (vib_rms * max(x[13], 0.0) / w_hover_nom))
            else:
                accel = imu.accel(R.T @ (acc_w + np.array([0.0, 0.0, self.g])), accel_bias, accel_noise[j])
            h_true = x[2] - z_cg_ref
            h_baro: float | None = None
            if k % n_baro == 0:
                if vt and sc.baro_thrust_offset_m:
                    # VTOL plant: the barometer's static-pressure error with the fan running, x T / (m g) (sensor side)
                    hb = h_true + baro_noise[k // n_baro] + sc.baro_thrust_offset_m * max(x[13], 0.0) / w_hover_nom
                else:
                    hb = h_true + baro_noise[k // n_baro]
                if baro_q > 0.0:
                    hb = math.floor(hb / baro_q + 0.5) * baro_q          # the sensor's output step (no random draw)
                baro_queue.append(hb)
                if len(baro_queue) > n_lat:
                    h_baro = baro_queue.popleft()
            # VTOL faults: baro silent (the NAV filter gets no correction), IMU / NAV dead (NAV outputs frozen)
            imu_dead = vt and sc.imu_fail_t_s is not None and t >= sc.imu_fail_t_s
            baro_dead = vt and sc.baro_fail_t_s is not None and t >= sc.baro_fail_t_s
            if baro_dead:
                h_baro = None
            # ---------------- attitude estimate (pad / flight from the NAV's lagged view of the state) ----------------
            pad_mode = (not sc.bypass_state_machine) and nav_states[0] in PAD_STATES
            if gns_chain:
                f4_view[0] = nav_f4[0]
            if imu_dead:
                pass                                # q_est / rate_meas: the last values the NAV delivered
            elif mode == "truth_noise":
                q_est = quat.multiply(q, quat.from_rotvec(att_noise[k]))
                rate_meas = gyro
            elif pad_est and v08:
                # v0.8 NAV order (sim.estimator docstring): 2. predict from the previous tick's corrected q and bias ->
                # 3. publish (q_pub, rate) -> 4. the Mahony correction, used from the next tick
                if hasattr(est, "predict"):
                    q_pub_, rate_pub_ = est.predict(gyro, pad_mode)
                else:                                               # (pad_gyro comparison estimator: same rule)
                    rate_pub_ = np.asarray(gyro, dtype=float) - est.bias
                    q_pub_ = est.q.copy() if pad_mode else quat.integrate(est.q, rate_pub_, dt_c)
                est.update(gyro, accel, pad_mode)
                q_est, rate_meas = q_pub_, rate_pub_
                q_corr = est.q.copy()
            elif pad_est:
                q_est = est.update(gyro, accel, pad_mode)
                rate_meas = est.rate(gyro)
            else:
                q_est, mahony_bias = self._mahony(q_est, mahony_bias, gyro, accel, dt_c)
                rate_meas = gyro
            if v08 and not imu_dead:
                if not pad_est:
                    q_corr = q_est
                # the frame of this tick (published before the correction part): the altitude filter's h / v_z of the
                # PREVIOUS tick, cos_tilt / tilt of the published q (float32, nlink v3), the GPS slot of NAV tick n_boot + k
                h_pub, v_pub = alt.h, alt.v
                c_pub, tilt_pub = nl.nav3_tilt(q_est)
                slot_pub = slot_tx.slot(n_boot + k) if slot_tx is not None else None
            elif v08:
                slot_pub = None                     # IMU / NAV failed: no valid NAV frame content beyond the frozen values
            tilt_est = quat.tilt(q_est)
            # ---------------- altitude filter ----------------
            if vt:
                if not imu_dead:
                    # v0.7.1 VTOL NAV filter (sim.state_machine.VtolAltitudeFilter): pad hold + bias calibration or the
                    # prediction with the learnt bias, then the baro correction with the thrust compensation on the
                    # FCU's last ESC throttle the NAV has (nlink thr_q, same lag as the state); v0.8: on the corrected q
                    a_z_w = float((quat.to_matrix(q_corr if v08 else q_est) @ accel)[2]) - self.g
                    valt.step(a_z_w, dt_c, h_baro, t, pad_mode, nl.fcu_throttle(nav_thr_q[0]))
                    acc_nav_z = float(accel[2])
                    if gps_rx is not None:
                        # v0.8 NAV order step 6: the receiver (TRUE antenna position / velocity of this tick) and the
                        # NAV's GPS part on the corrected attitude and the bias-corrected rate
                        r_w = R @ r_ant_b
                        gps_nav_tick(n_boot + k, t, np.array(x[0:3]) + r_w, v + np.cross(R @ w, r_w), accel, q_corr,
                                     (gyro - est.bias) if pad_est else gyro, pad_mode, alt.h)
            elif not imu_dead:
                a_z_w = float((quat.to_matrix(q_est) @ accel)[2]) - self.g
                if pad_mode:
                    self.alt.h, self.alt.v = 0.0, 0.0   # pad mode (NAV: state SAFE / ARMED): known reference
                else:
                    self.alt.predict(a_z_w, dt_c)
                if h_baro is not None:
                    self.alt.correct(h_baro, t)
            # ---------------- mission: ground station -> link -> onboard supervisor ----------------
            if sc.bypass_state_machine:
                active, release = True, False
                u = u_fixed
                state = State.POWERED
                if not yaw_ref_set:
                    self.ctrl.set_yaw_reference(q_est)
                    yaw_ref_set = True
            else:
                # ground side (its clock = sim time): telemetry that has arrived, operator, next request
                for tr in link.deliver("down", t):
                    gs.receive(tr.data, tr.t_arrival)
                self.operator.step(gs, t)
                up = gs.poll(t)
                if up is not None:
                    link.send("up", up, t)
                # vehicle side: frames the COM received and forwarded over the ilink (COM_FORWARD_S) since the
                # previous FCU tick
                rx_tr = link.deliver("up", t - COM_FORWARD_S)
                frames = ([UplinkFrame.from_frame(f) for f in self.uplink_decoder.feed(b"".join(tr.data for tr in rx_tr))]
                          if rx_tr else [])
                L["frames_rx"][k] = len(frames)
                eul = quat.to_euler(q_est)
                inp = OnboardInputs(
                    tick=k, frames=frames, h_m=alt.h, vz_mps=alt.v, tilt_deg=math.degrees(tilt_est),
                    gyro_rad_s=(float(rate_meas[0]), float(rate_meas[1]), float(rate_meas[2])),
                    gyro_saturated=imu.gyro_saturated(gyro), v_cell=v_cell,
                    aligned=(est.n >= est.n_avg) if pad_est else True,
                    arm_switch=not (sc.arm_switch_open_t_s is not None and t >= sc.arm_switch_open_t_s),
                    roll_deg=math.degrees(eul[0]), pitch_deg=math.degrees(eul[1]), yaw_deg=math.degrees(eul[2]))
                if vt:
                    # FCU tick (sim.onboard.FcuStack, v0.7.1 order): attitude controller on the previous tick's
                    # supervisor outputs and ESC throttle (fo.att_active / att_u, run below) -> vanes -> supervisor ->
                    # vertical controller (NAV quaternion cos(tilt), body-z specific force) -> ESC
                    inp.imu_ok = not imu_dead
                    inp.baro_ok = not baro_dead
                    inp.q_nav = (float(q_est[0]), float(q_est[1]), float(q_est[2]), float(q_est[3]))
                    inp.accel_z_mps2 = acc_nav_z
                    for k_m, v_m in self.meas.items():
                        # v0.9 power signals (sim.tether.PowerSystem.measure, previous tick): only the fields the
                        # supervisor declares (the firmware round adds them: v_in, v_tether, v_buffer, buf_i_pos)
                        if hasattr(inp, k_m):
                            setattr(inp, k_m, v_m)
                    if v08:
                        # v0.8 FCU inputs from the nlink v3 frame: h / v_z one tick old, cos_tilt / tilt from the
                        # published q (the vertical controller's c), the GPS slot
                        inp.h_m, inp.vz_mps = h_pub, v_pub
                        inp.cos_tilt, inp.tilt_deg = c_pub, tilt_pub
                        inp.gps_slot = slot_pub
                    fo = self.fcu.step(inp)
                    out = fo.sup
                    if self.nav_thr_lagged:
                        nav_u_lag += nav_lag_a * (float(fo.throttle) - nav_u_lag)
                        nav_thr_q.append(nl.fcu_throttle_q(nav_u_lag))
                    else:
                        nav_thr_q.append(nl.fcu_throttle_q(fo.throttle))
                    if gns_chain:
                        nav_f4.append(nl.Fcu4(0, int(out.state), 0, 0, nl.fcu_throttle_q(fo.throttle), fo.fcu4[0],
                                              fo.fcu4[1]))
                    if self.gps_on:
                        # truth bookkeeping (never reaches the vehicle): the TRUE CG where the position loop captured
                        # its hold point (at the step whose inputs captured it; re-captures after a GPS loss in LAND;
                        # v0.8.1: the GNS's step, run in this tick by the glink part)
                        hr = pos_src.pos.hold_rf
                        if hr is not None and (last_hold_rf is None or ((fo.pos is not None or gns_chain)
                                                                        and hr != last_hold_rf)):
                            hold_true = (float(x[0]), float(x[1]))
                            hold_caps.append({"t": t, "true_xy": hold_true, "nav_rf": list(hr),
                                              "phase": lp.PHASE_NAMES.get(int(out.phase), str(out.phase))})
                        last_hold_rf = hr
                else:
                    out = sup.update(inp)
                state = out.state
                nav_states.append(state)
                L["link_ok"][k] = bool(out.flags & lp.FLAG_LINK_OK)
                for e in out.events:
                    events.append((t, e))
                if out.reply_seq is not None:
                    # TELEM handed to the vehicle radio FC_REPLY_DELAY_S after the uplink frame arrived at the COM
                    t_reply = max(t, max(tr.t_arrival for tr in rx_tr) + FC_REPLY_DELAY_S) if rx_tr else t
                    tlm_tx = out.telemetry
                    pw_tx = getattr(out, "pwr", None)
                    if pw_tx is not None:          # v0.9 RP1: the power builds' TELEM v4 / v5 (battery build: unchanged)
                        tlm_tx = lp.with_power_tail(tlm_tx, pw_tx.v_aux_q, pw_tx.flags)
                    if self.lbat_link:             # 2026-11-10: the carrier battery build's TELEM v6 / v7
                        tlm_tx = lp.with_logic_tail(tlm_tx, self.lbat_q)
                    link.send("down", lp.encode_telemetry(self.vehicle_id, out.reply_seq, tlm_tx), t_reply)
                if state == State.ARMED and prev_state != State.ARMED:
                    self.ctrl.set_yaw_reference(q_est)
                if state == State.POWERED and t_launch is None:
                    t_launch = t
                    for dt_step, r_step in self.link_steps:     # 2026-11-10 study hook (no-op by default)
                        link.cfg_schedule.append((t + dt_step, LinkConfig.from_params(self.p, radio=r_step,
                                                                                      loss=self.link_cfg.loss)))
                        events.append((t + dt_step, f"FAULT link model {self.link_cfg.radio} -> {r_step} from launch + "
                                                    f"{dt_step:.2f} s (ground keeps its {self.link_cfg.radio} timeout)"))
                    if sc.link_outage_after_launch_s is not None:
                        t_out = t + sc.link_outage_after_launch_s
                        link.outages.append((t_out, float("inf")))
                        events.append((t_out, f"FAULT total link loss from launch + {sc.link_outage_after_launch_s:.1f} s"))
                if out.release and not ejected:
                    ejected = True
                    t_deploy = t
                    cfg = self.cfg_post
                    dz = self.cfg_full.z_cg - self.cfg_post.z_cg
                    shift = R @ np.array([0.0, 0.0, -dz])
                    x[0] += float(shift[0])
                    x[1] += float(shift[1])
                    x[2] += float(shift[2])
                    z_cg_ref -= dz
                    events.append((t, f"ejection: mass {self.cfg_full.m:.3f} -> {cfg.m:.3f} kg, chute inflating"))
                active = out.control_active
                u = out.throttle
                if vt:
                    u = fo.throttle                 # the vertical controller's u while motor_on, else 0 (ESC path)
                    active = fo.att_active          # v0.7.1: the attitude controller of this tick runs on the previous
                    u_att = fo.att_u                # tick's control_active and ESC throttle
                    if state != prev_state:
                        t_state = t
                    vo = fo.vc
                    L["phase"][k] = out.phase
                    if out.phase in TARGET_PHASES:
                        L["h_tgt"][k] = out.h_tgt_m
                    if vo.mode != MODE_IDLE:
                        L["h_ref"][k] = vo.h_ref
                        L["vz_cmd"][k] = vo.vz_cmd
                    L["u_ff"][k] = vo.u_ff
                    L["vc_integ"][k] = vo.integ
                    L["vc_sat"][k] = vo.sat
                    L["vc_mode"][k] = vo.mode
                    L["td_count"][k] = vo.td_count
                    L["motor_on"][k] = out.motor_on
                    L["baro_lost"][k] = out.baro_lost
                    L["reason"][k] = out.reason
                    L["imu_ok"][k] = not imu_dead
                    L["baro_ok"][k] = not baro_dead
                    L["vc_td_by"][k] = vo.td_by
                    L["vc_low"][k] = vo.low
                    L["vc_still"][k] = vo.still
                    L["vc_support"][k] = vo.support
                    L["vc_floor"][k] = vo.floor
                    L["vc_low_in_land"][k] = min(vo.low_in_land_ticks, 32767)
                    L["vc_vz_lpf"][k] = vo.vz_lpf
                    L["vc_f_excess"][k] = vo.f_excess_mps2
                    L["vc_thrust_ratio"][k] = vo.thrust_ratio
                    L["alt_bias"][k] = valt.b
                    L["baro_comp_m"][k] = valt.last_comp_m
                    L["est_gate"][k] = bool(getattr(est, "gate", False))
                    L["att_u"][k] = u_att
                    L["accel_nav_z"][k] = acc_nav_z
                    L["vc_k_thrust"][k] = vo.k_thrust
                    L["vc_kt_learn"][k] = vo.kt_learn
                    L["vc_low_alt"][k] = vo.low_alt
                prev_state = state
            if ejected and t_deploy is not None:
                chute_cda = self.chute_cda_full * (1.0 - math.exp(-(t - t_deploy) / opt.chute_inflate_tau_s))
            # ---------------- controller ----------------
            if active and not prev_active:
                self.ctrl.reset()
            prev_active = active
            if self.gps_on:
                # v0.8: this tick's attitude controller flies the tilt the setpoint builder made at the end of the previous
                # FCU tick (level-frame rotation vector, FcuOutputs.att_tilt)
                self.ctrl.tilt_sp = fo.att_tilt if fo is not None else (0.0, 0.0)
            if active:
                co = self.ctrl.update(q_est, rate_meas, u_att if vt else u)
                dcmd = co.delta_cmd
                L["moments"][k] = co.moments
                L["rate_sp"][k] = co.rate_sp
                L["att_err"][k] = co.att_error
                L["saturated"][k] = co.saturated
                L["pwm_us"][k] = co.pwm_us
                L["integrator"][k] = co.integrator
            else:
                dcmd = np.zeros(4)
                L["pwm_us"][k] = self.ctrl.mixer.pwm(dcmd)
            # ---------------- actuator interface ----------------
            if vt:
                pending.extend(latch.tick(k, [float(dcmd[0]), float(dcmd[1]), float(dcmd[2]), float(dcmd[3])]))
                pending.extend((te, 4, val) for te, _, val in latch_esc.tick(k, [float(u)]))
            else:
                pending.extend(latch.tick(k, [float(dcmd[0]), float(dcmd[1]), float(dcmd[2]), float(dcmd[3]), float(u)]))
            pending.sort(key=lambda e: e[0])
            # ---------------- vane failure (servo-side) ----------------
            if (sc.vane_failure_index is not None and not failed and t_launch is not None
                    and t >= t_launch + sc.vane_failure_t_after_launch_s):
                failed = True
                i = sc.vane_failure_index
                if stuck_rad is None:
                    frozen = i
                    events.append((t, f"FAULT vane {i + 1} frozen at {math.degrees(x[14 + i]):.1f} deg"))
                else:
                    events.append((t, f"FAULT vane {i + 1} hardover to {math.degrees(stuck_rad):.1f} deg"))
            # commands that change exactly at the tick
            while pending and pending[0][0] <= t + EPS_T:
                _, c, val = pending.pop(0)
                if c < 4:
                    srv[c] = val
                else:
                    u_esc = val
            if failed and stuck_rad is not None:
                srv[sc.vane_failure_index] = stuck_rad
            # ---------------- energy / battery ----------------
            if self.pw is not None:
                # v0.9 VTOL electrical plant: the bus of this tick on the lagged thrust (battery / tether / hybrid);
                # v_cell = V_bus / cells (the FCU's PF0 divider), the thrust limit when the sources cannot carry the fan
                bus = self.pw.step(t, dt_c, x[13], (self.plant_thrust(u_esc, v_cell) if self.pw.cfg.regen_eff > 0.0
                                                   and u_esc > 0.0 else None))
                energy += bus.v_bus * bus.i_load * dt_c
                if "p_regen_w" in L:
                    L["p_regen_w"][k] = self.pw.p_regen_w
                v_cell = bus.v_bus / self.pw_cells
                self.t_lim = self.pw.thrust_limit_n(bus)
                self.meas = self.pw.measure(bus)
                if self.rig is not None and self.pw.cable_cut and self.rig.cable_present:
                    self.rig.cut_cable()
                if not bus.avionics_ok:
                    srv = [x[14], x[15], x[16], x[17]]      # servos unpowered: they hold where they are (no jet anyway)
                L["v_bus"][k] = bus.v_bus
                L["i_bus"][k] = bus.i_load
                L["supply_v"][k] = bus.v_supply
                L["supply_a"][k] = bus.i_tether
                L["supply_state"][k] = bus.supply_state
                L["buffer_soc"][k] = bus.soc_buffer
                L["buffer_a"][k] = bus.i_buffer
                L["buffer_v"][k] = bus.v_buffer
                L["batt_soc"][k] = bus.soc
                L["tether_connected"][k] = bus.connected
                L["cable_attached"][k] = (not self.pw.cable_cut) and self.pw.plugged
                L["thrust_limited"][k] = bus.limited or not bus.esc_ok
                L["v_in_meas"][k] = self.meas.get("v_in", np.nan)
            else:
                energy += self.edf.power(u_esc) * dt_c
                v_cell = self.battery_v_cell(energy, u_esc)
            # ---------------- log (before stepping) ----------------
            L["t"][k] = t
            L["pos"][k] = x[0:3]
            L["vel"][k] = x[3:6]
            L["q"][k] = q
            L["omega"][k] = w
            L["thrust"][k] = x[13]
            L["throttle"][k] = u
            L["throttle_esc"][k] = u_esc
            if self.play_on:
                L["delta"][k] = self.vane_mem
                L["delta_servo"][k] = x[14:18]
            else:
                L["delta"][k] = x[14:18]
            if self.rot_on:
                L["rotor_omega"][k] = self.rotor.omega(x[13])
            if self.rig is not None:
                L["tether_catch_n"][k] = self.rig.st.tension_catch
                L["tether_cable_n"][k] = self.rig.st.tension_cable
                L["tether_tension_n"][k] = self.rig.st.tension_catch + self.rig.st.tension_cable
                L["tether_l0"][k] = self.rig.st.l0
                L["tether_d"][k] = self.rig.distance(self._att_world(x))
                L["tether_locked"][k] = self.rig.st.locked
                if "tether_loop_depth" in L:
                    L["tether_loop_depth"][k] = self.rig.st.loop_depth_m if self.rig.cable_present else 0.0
                    L["tether_loop_x"][k] = self.rig.st.loop_x_m
            L["delta_cmd"][k] = dcmd
            L["delta_srv"][k] = srv
            L["state"][k] = int(state)
            L["h_est"][k] = alt.h
            L["vz_est"][k] = alt.v
            if h_baro is not None:
                L["h_baro"][k] = h_baro
            L["energy_j"][k] = energy
            L["tilt_deg"][k] = math.degrees(quat.tilt(q))
            L["tilt_vec"][k] = quat.tilt_vector(q)
            L["euler_deg"][k] = np.degrees(quat.to_euler(q))
            L["gyro"][k] = gyro
            L["accel"][k] = accel
            L["q_est"][k] = q_est
            L["tilt_est_deg"][k] = math.degrees(tilt_est)
            L["tilt_est_err_deg"][k] = math.degrees(up_error_rad(q, q_est))
            # tilt the controller acts on: body z vs the *estimated* vertical for the integrating estimators
            # (their error is a slow offset the loop cannot see); the truth for truth_noise (zero-mean white error)
            if mode == "truth_noise":
                L["tilt_ctrl_deg"][k] = L["tilt_deg"][k]
                L["tilt_ctrl_vec"][k] = L["tilt_vec"][k]
            else:
                L["tilt_ctrl_deg"][k] = L["tilt_est_deg"][k]
                L["tilt_ctrl_vec"][k] = quat.tilt_vector(q_est)
            L["gyro_bias"][k] = gyro_bias[j]
            L["gyro_bias_est"][k] = est.bias if pad_est else (mahony_bias if mode == "mahony" else 0.0)
            if vt:
                wind, gust_on, wind_mean = self.wind_vtol(k, t, t_launch, turb, walk)
                L["gust_active"][k] = gust_on
                L["h_true"][k] = h_true
                L["surface_z"][k] = self.surface_z(x[0] - cfg.z_cg * R[0, 2], x[1] - cfg.z_cg * R[1, 2])
                L["wind_mean"][k, 0] = wind_mean[0]
                L["wind_mean"][k, 1] = wind_mean[1]
                # hand pushes: the contact point of the current attitude (recorded at each push's start too); log force /
                # point at the tick
                fxw = fyw = fzw = 0.0
                pts = []
                for ip, pu in enumerate(sc.pushes):
                    if ip not in push_rb and t + dt_c > pu.t0_s:
                        push_rb[ip] = self.push_contact_body(pu, q)
                        if pu.pole:
                            push_dir[ip], push_elev[ip] = self.pole_direction(pu, np.array(x[0:3]) + R @ np.array(push_rb[ip]))
                    f = pu.factor(t)
                    if f > 0.0:
                        a_, b_, c_ = self.push_force(ip, pu, t, push_dir)
                        fxw, fyw, fzw = fxw + a_, fyw + b_, fzw + c_
                        pts.append(self.push_contact_body(pu, q))
                if pts:
                    L["push_active"][k] = True
                    L["push_force"][k] = (fxw, fyw, fzw)
                    rb = pts[0]
                    L["push_point"][k] = np.array(x[0:3]) + R @ np.array(rb)
            else:
                wind = self.wind(t, t_launch)
            L["wind"][k] = wind
            L["v_cell"][k] = v_cell
            L["chute_cda"][k] = chute_cda
            L["control_active"][k] = active
            if v08:
                L["cos_tilt_pub"][k] = c_pub
                L["h_pub"][k] = h_pub
            if self.gps_on:
                fl_ = nav_gps.filter
                rx_ = self.fcu.rx
                g_ = rx_.latest
                L["gps_ok"][k] = bool(g_ is not None and rx_.fresh() and g_.fix_ok)
                L["gps_gate_ok"][k] = rx_.gate_ok
                L["gps_flags"][k] = fl_.snap.flags
                L["gps_sats"][k] = fl_.last_sv
                if fl_.last_hacc_mm != 0xFFFFFFFF:
                    L["gps_hacc_m"][k] = fl_.last_hacc_mm / 1000.0
                L["gps_fix"][k] = gn.FIX_3D if fl_.last_fix3d else 0
                lr_ = gps_rx.last
                L["gps_rx_fix"][k] = lr_["fix"]
                L["gps_err"][k] = lr_["err_p"]
                L["gps_err_slow"][k] = lr_["err_slow"]
                L["gps_err_v"][k] = lr_["err_v"]
                if fl_.init and fl_.frame is not None:
                    if nav_origin_w is None:
                        nav_origin_w = gps_rx.frame.to_w(fl_.frame.lat0_e7, fl_.frame.lon0_e7)
                    L["pos_est"][k, 0] = nav_origin_w[0] + fl_.p[0]
                    L["pos_est"][k, 1] = nav_origin_w[1] + fl_.p[1]
                    L["vel_est_h"][k] = fl_.v
                L["nav_b"][k] = fl_.b
                L["gps_resets"][k] = fl_.reset_count
                dpsi = fl_.psi + quat.yaw(q_est) - quat.yaw(q)
                L["psi_err_deg"][k] = math.degrees(math.atan2(math.sin(dpsi), math.cos(dpsi)))
                L["yaw_corr_deg"][k] = math.degrees(fl_.yaw_corr)
                po_ = pos_src.pos_out()
                L["pos_mode"][k] = po_.mode
                L["pos_dist"][k] = po_.dist_m
                L["pos_limited"][k] = po_.limited
                L["pos_guard"][k] = po_.guard
                L["pos_engaged"][k] = po_.engaged
                L["pos_requests"][k] = po_.requests
                L["pos_lost"][k] = min(po_.lost_steps, 32767)
                L["pos_integ"][k] = po_.integ
                L["pos_a_cmd"][k] = po_.a_cmd
                L["pos_v_sp"][k] = po_.v_sp
                L["pos_cos_lim"][k] = po_.cos_lim
                if po_.hold is not None and nav_origin_w is not None:
                    L["hold_nav"][k, 0] = nav_origin_w[0] + po_.hold[0]
                    L["hold_nav"][k, 1] = nav_origin_w[1] + po_.hold[1]
                if pos_src.pos.hold_rf is not None and hold_true is not None:
                    L["hold_true"][k] = hold_true
                ts_ = fo.att_tilt if fo is not None else (0.0, 0.0)
                L["tilt_sp"][k] = ts_
                L["tilt_sp_deg"][k] = math.degrees(math.hypot(ts_[0], ts_[1]))
            spread = self.thrust_scale != 1.0 or self.thrust_vexp > 0.0 or self.vane_trim is not None or self.pw is not None
            # ---------------- physics: RK4 broken at every command change ----------------
            n_sub = (sc.descent_substeps if (ejected and t_deploy is not None
                                              and t - t_deploy >= opt.descent_substeps_after_release_s) else self.n_sub)
            h_nom = dt_c / n_sub
            t_next = t + dt_c
            stops = [t + h_nom * (i + 1) for i in range(n_sub - 1)]
            stops += [e[0] for e in pending if e[0] < t_next - EPS_T]
            stops.append(t_next)
            stops.sort()
            a_t = t
            for b_t in stops:
                hstep = b_t - a_t
                if hstep > 1e-10:
                    if spread:
                        args = (cfg, self.plant_thrust(u_esc, v_cell), self.plant_vanes(srv), wind, chute_cda, frozen)
                    else:
                        args = (cfg, self.edf.thrust_steady(u_esc), srv, wind, chute_cda, frozen)
                    if vt and sc.pushes:
                        # hand push over this step: the force at the step's midpoint (midpoint rule), at the contact point
                        # of the attitude at the step's start (the pole tip slides on the round body: the contact stays
                        # on the side facing the person - a body-fixed point would give an unphysical yaw lever arm)
                        tm = 0.5 * (a_t + b_t)
                        ext = None
                        for ip, pu in enumerate(sc.pushes):
                            f = pu.factor(tm)
                            if f > 0.0 and ip in push_rb:
                                a_, b_, c_ = self.push_force(ip, pu, tm, push_dir)
                                rb = self.push_contact_body(pu, np.array(x[6:10]))
                                ext = (a_, b_, c_, rb[0], rb[1], rb[2] + self.cfg_full.z_cg - cfg.z_cg) if ext is None else (
                                    ext[0] + a_, ext[1] + b_, ext[2] + c_, ext[3], ext[4], ext[5])
                        if ext is not None:
                            args = args + (ext,)
                    if self.rig is not None:
                        # v0.9 tether: the handler / feed / catenary / drag once per step (held), the catch line in the
                        # derivative; the snag jerk at the step's midpoint
                        pa_, va_ = self._att_world(x), self._att_vel_world(x)
                        self.rig.update(a_t, hstep, pa_, va_, wind, self.rho)
                        sfx, sfy, sfz = self.rig.cfg.snag_force(0.5 * (a_t + b_t))
                        fc = self.rig.st.cable_force_w
                        rfx, rfy, rfz = self.rig.catch_free_w(pa_)       # SRL retract force (fix round; 0 otherwise)
                        if len(args) == 6:
                            args = args + (None,)
                        args = args + ((self.rig.A[0], self.rig.A[1], self.rig.A[2], self.att_rb[0], self.att_rb[1],
                                        self.att_rb[2], self.rig.st.l0, self.rig.k, self.rig.c, fc[0] + sfx + rfx,
                                        fc[1] + sfy + rfy, fc[2] + sfz + rfz, self.rig.st.l0_dot),)
                    x = self.rk4(x, hstep, args)
                    if self.play_on:
                        # v0.9 vane play: the actual vane angles follow the new servo outputs through the play
                        dmx = self.delta_mech
                        for i_ in range(4):
                            v_ = vane_play(self.vane_mem[i_], x[14 + i_], self.play_h[i_])
                            self.vane_mem[i_] = dmx if v_ > dmx else (-dmx if v_ < -dmx else v_)
                    if self.rig is not None:
                        pa_, va_ = self._att_world(x), self._att_vel_world(x)
                        A_ = self.rig.A
                        self.rig.st.tension_catch = tt.catch_line_force(
                            (A_[0] - pa_[0], A_[1] - pa_[1], A_[2] - pa_[2]), tuple(va_), self.rig.st.l0, self.rig.k,
                            self.rig.c, self.rig.st.l0_dot)[3] + self.rig.st.retract_n
                    r22 = 1.0 - 2.0 * (x[7] * x[7] + x[8] * x[8])
                    z_bottom = x[2] - cfg.z_cg * r22
                    if vt:
                        # VTOL ground contact: rest on the surface below, touchdown on any surface (pad or ground)
                        if on_ground:
                            if x[5] <= 0.0 or z_bottom <= rest_surface:
                                x[0], x[1], x[2] = float(pos0[0]), float(pos0[1]), float(pos0[2])
                                x[3] = x[4] = x[5] = 0.0
                                x[6], x[7], x[8], x[9] = float(q_rest[0]), float(q_rest[1]), float(q_rest[2]), float(q_rest[3])
                                x[10] = x[11] = x[12] = 0.0
                            else:
                                on_ground = False
                                liftoffs.append(b_t)
                                if t_liftoff is None:
                                    t_liftoff = b_t
                                    events.append((b_t, f"liftoff (T = {x[13]:.1f} N)"))
                                else:
                                    events.append((b_t, f"lift-off again from z = {rest_surface:+.2f} m (T = {x[13]:.1f} N)"))
                        else:
                            r02 = 2.0 * (x[7] * x[9] + x[6] * x[8])
                            r12 = 2.0 * (x[8] * x[9] - x[6] * x[7])
                            xe, ye = x[0] - cfg.z_cg * r02, x[1] - cfg.z_cg * r12
                            surf = self.surface_z(xe, ye)
                            if z_bottom <= surf and x[5] < 0.0:
                                td = {"t": b_t, "vz": x[5], "vh": math.hypot(x[3], x[4]),
                                      "vh_air_mean": math.hypot(x[3] - wind_mean[0], x[4] - wind_mean[1]),
                                      "vh_air_local": math.hypot(x[3] - wind[0], x[4] - wind[1]),
                                      "wind_x": wind[0], "wind_y": wind[1], "wind_mean_x": wind_mean[0],
                                      "wind_mean_y": wind_mean[1],
                                      "speed": math.hypot(x[3], math.hypot(x[4], x[5])),
                                      "tilt_deg": math.degrees(quat.tilt(np.array(x[6:10]))),
                                      "x": xe, "y": ye, "surface_z": surf, "on_pad": surf == 0.0,
                                      "state": int(state), "phase": int(out.phase), "u_esc": u_esc, "thrust_n": x[13],
                                      "rate_deg_s": math.degrees(math.sqrt(x[10] ** 2 + x[11] ** 2 + x[12] ** 2)),
                                      "cg_x": x[0], "cg_y": x[1]}
                                # v0.8 physics-review fix (medium 8): energy-method tip-over margin at the contact
                                vh_ = td["vh"]
                                along = (r02 * x[3] + r12 * x[4]) / vh_ if vh_ > 1e-6 else 0.0
                                lean_t = math.degrees(math.asin(max(-1.0, min(1.0, along))))
                                i_cg = min(cfg.I[0][0], cfg.I[1][1])
                                td["lean_toward_deg"] = lean_t
                                if self.tip_h_params_m is not None:
                                    vc_ = tip_over_vcrit(cfg.m, i_cg, self.tip_r_m, self.tip_h_params_m, lean_t)
                                    td["tip_vcrit_mps"] = vc_
                                    td["tip_margin_mps"] = vc_ - vh_
                                vcg = tip_over_vcrit(cfg.m, i_cg, self.tip_r_m, cfg.z_cg, lean_t)
                                td["tip_vcrit_geom_mps"] = vcg
                                td["tip_margin_geom_mps"] = vcg - vh_
                                if self.hw == "vtol":
                                    # v0.9 VTOL configuration: the solid-fin footprint (the square through the four fin
                                    # tips) along the motion in the body frame, less the plant CG offset along it; CG
                                    # height = z_cg above the contact plane (fin trailing edges = exit plane, geometric)
                                    Rt = quat.to_matrix(np.array(x[6:10]))
                                    if vh_ > 1e-6:
                                        db = Rt.T @ np.array([x[3] / vh_, x[4] / vh_, 0.0])
                                        ang = math.atan2(float(db[1]), float(db[0]))
                                    else:
                                        db = np.array([math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0])
                                        ang = math.pi / 4
                                    nb = math.hypot(float(db[0]), float(db[1])) or 1.0
                                    co = self.cg_off if self.cg_off is not None else (0.0, 0.0)
                                    r_fp = self.veh.footprint_radius_m(ang) - (co[0] * db[0] + co[1] * db[1]) / nb
                                    vcp = tip_over_vcrit(cfg.m, i_cg, r_fp, cfg.z_cg, lean_t)
                                    td["tip_footprint_r_m"] = r_fp
                                    td["tip_vcrit_mps"] = vcp
                                    td["tip_margin_mps"] = vcp - vh_
                                if self.gps_on:
                                    # v0.8: the touchdown against the hold point (TRUE CG at its latest capture)
                                    td["hold_true"] = hold_true
                                    td["from_hold_m"] = (None if hold_true is None else
                                                         math.hypot(x[0] - hold_true[0], x[1] - hold_true[1]))
                                    po_td = pos_src.pos_out()
                                    td["pos_mode"] = int(po_td.mode)
                                    # v0.8.1 touchdown criterion (vtol_scenarios.v08_checks): the position loop's wind
                                    # trim (its integrator as a tilt) and the trim / setpoint it commanded at the contact
                                    td["trim_deg"] = math.degrees(math.atan(math.hypot(po_td.integ[0], po_td.integ[1])
                                                                            / self.g))
                                    td["trim_cmd_deg"] = math.degrees(math.hypot(po_td.trim_l[0], po_td.trim_l[1]))
                                    ts_td = fo.att_tilt if fo is not None else (0.0, 0.0)
                                    td["tilt_sp_deg"] = math.degrees(math.hypot(ts_td[0], ts_td[1]))
                                touchdowns.append(td)
                                if t_touchdown is None:
                                    t_touchdown, touchdown_speed, touchdown_vz = b_t, td["speed"], td["vz"]
                                events.append((b_t, f"touchdown #{len(touchdowns)} on the {'pad' if td['on_pad'] else 'ground'} "
                                                    f"(z {surf:+.2f} m) at v_z {td['vz']:.2f} m/s, horizontal "
                                                    f"{td['vh']:.2f} m/s, tilt {td['tilt_deg']:.1f} deg, "
                                                    f"{lp.STATE_NAMES[int(state)]}/{lp.PHASE_NAMES[int(out.phase)]}"))
                                x[2] -= (z_bottom - surf)
                                x[3] = x[4] = x[5] = 0.0
                                x[10] = x[11] = x[12] = 0.0
                                on_ground = True
                                rest_surface = surf
                                pos0 = np.array(x[0:3])
                                q_rest = np.array(x[6:10])
                    elif on_ground:
                        if x[5] <= 0.0 or z_bottom <= pad_z_bottom:
                            # pad (or ground after touchdown) holds it: restore pose, kill velocities
                            x[0], x[1], x[2] = float(pos0[0]), float(pos0[1]), float(pos0[2])
                            x[3] = x[4] = x[5] = 0.0
                            x[6], x[7], x[8], x[9] = float(q_rest[0]), float(q_rest[1]), float(q_rest[2]), float(q_rest[3])
                            x[10] = x[11] = x[12] = 0.0
                        else:
                            on_ground = False
                            if t_liftoff is None:
                                t_liftoff = b_t
                                events.append((b_t, f"liftoff (T = {x[13]:.1f} N)"))
                    elif z_bottom <= pad_z_bottom and x[5] < 0.0 and t_liftoff is not None:
                        touchdown_speed = math.hypot(x[3], math.hypot(x[4], x[5]))
                        touchdown_vz = x[5]
                        t_touchdown = b_t
                        events.append((b_t, f"touchdown at {touchdown_speed:.1f} m/s (v_z {x[5]:.1f} m/s), "
                                            f"tilt {math.degrees(quat.tilt(np.array(x[6:10]))):.0f} deg"))
                        x[2] -= (z_bottom - pad_z_bottom)
                        x[3] = x[4] = x[5] = 0.0
                        x[10] = x[11] = x[12] = 0.0
                        on_ground = True
                        pos0 = np.array(x[0:3])
                        q_rest = np.array(x[6:10])
                    # MPU-6050 on-chip DLPF (exact first-order update over the step)
                    a_f = 1.0 - math.exp(-hstep / tau_dlpf) if tau_dlpf > 0.0 else 1.0
                    w_dlpf = w_dlpf + a_f * (np.array(x[10:13]) - w_dlpf)
                    a_t = b_t
                # apply the command changes that fall at the end of this step
                while pending and pending[0][0] <= a_t + EPS_T and pending[0][0] < t_next - EPS_T:
                    _, c, val = pending.pop(0)
                    if c < 4:
                        srv[c] = val
                    else:
                        u_esc = val
                if failed and stuck_rad is not None:
                    srv[sc.vane_failure_index] = stuck_rad
            if not all(math.isfinite(v_) for v_ in x):
                nan_abort = True
                events.append((t, "numerical failure (NaN) - run aborted"))
                break
            # ---------------- termination ----------------
            if vt:
                # VTOL: on the ground after the motor ran (a flight, or a take-off that never left the pad), motor off
                # for good (SAFE after LANDED, or RELEASED), no further take-off in the operator's script:
                # stop_after_touchdown_s after the later of the last touchdown and the last state change
                if (on_ground and t_launch is not None and state in (State.SAFE, State.RELEASED)
                        and not self.operator.expects_flight()
                        and t - max(touchdowns[-1]["t"] if touchdowns else 0.0, t_state) >= opt.stop_after_touchdown_s):
                    break
            elif not sc.bypass_state_machine:
                if t_touchdown is not None and t - t_touchdown >= opt.stop_after_touchdown_s:
                    break

        if self.pw is not None:
            events.extend(self.pw.log)
            events.sort(key=lambda te: te[0])
        log = {key: arr[:n_log] for key, arr in L.items()}
        final_state = self.sup.state if not sc.bypass_state_machine else State.POWERED
        link_rec = None
        if not sc.bypass_state_machine:
            for t_g, e in gs.events:
                events.append((t_g, "GS " + e))
            events.sort(key=lambda te: te[0])
            link_rec = {
                "radio": self.link_cfg.radio, "loss": self.link_cfg.loss,
                "budget": (vtol_link_budget(self.link_cfg, gps=self.gps_on, power=self.pwr_link, lbat=self.lbat_link)
                           if vt else link_budget(self.link_cfg)),
                "lbat_telemetry": self.lbat_link, "radio_steps": list(self.link_steps),
                "link_stats": dict(link.stats), "ground_stats": dict(gs.stats),
                "uplink_decoder": dict(self.uplink_decoder.stats), "downlink_decoder": dict(gs.decoder.stats),
                "rtt": [r for _, r in gs.rtt_all], "records": [vars(r).copy() | {"name": r.name} for r in gs.records],
                "exec_log": list(sup.exec_log), "assist_log": list(gs.assist_log), "dt": dt_c,
                "outages": list(link.outages), "t_launch": t_launch,
            }
        vtol_rec = None
        if vt:
            vtol_rec = {
                "touchdowns": touchdowns, "liftoffs": liftoffs, "vp": asdict(self.vp),
                "u_hover_model": self.vp.u_hover, "hover_margin": self.vp.hover_margin,
                "sp": {k: getattr(self.sp, k) for k in ("spool_ticks", "hover_dm", "alt_step_dm", "alt_min_dm", "alt_max_dm",
                                                       "link_land_ticks", "endurance_ticks", "hard_cutoff_ticks", "ceiling_m",
                                                       "v_geofence_m", "v_fall_vz_mps", "v_fall_confirm_ticks",
                                                       "safe_after_landed_ticks", "tilt_abort_deg", "min_cell_v")},
                "t_max_n_model": float(self.veh.t_max_n), "weight_n_model": float(self.veh.full.mass_kg * self.g),
                "powered_ticks": self.sup.powered_ticks, "ground_z_m": self.ground_z, "pad_radius_m": self.pad_radius_m,
                "pad_foot_z_m": self.pad_foot_z_m, "z_cg_full_m": self.cfg_full.z_cg, "z_cg_post_m": self.cfg_post.z_cg,
                "t_deploy": t_deploy, "round_trip": dict(self.link_rt),
                "operator_log": list(getattr(self.operator, "log", [])),
                "plant": {"mass_kg": self.cfg_full.m, "mass_scale": float(sc.mass_scale), "cg_offset_mm": list(sc.cg_offset_mm),
                          "thrust_scale": self.thrust_scale, "thrust_voltage_exp": self.thrust_vexp,
                          "servo_tau_s": self.servo_tau, "vane_trim_err_deg": list(sc.vane_trim_err_deg),
                          "crossflow_cd_scale": float(sc.crossflow_cd_scale), "ram_drag_scale": float(sc.ram_drag_scale),
                          "baro_thrust_offset_m": float(sc.baro_thrust_offset_m),
                          "accel_vibration_rms_mps2": float(sens.accel_vibration_rms_mps2),
                          "swirl_scale": float(sc.swirl_scale), "swirl_torque_per_N": self.swirl_per_N,
                          "v_nom_cell": self.v_nom_cell},
                "rate_abort": {"tilt_rate_abort_rad_s": self.sp.tilt_rate_abort_rad_s,
                               "spin_rate_abort_rad_s": self.sp.spin_rate_abort_rad_s,
                               "rate_persist_ticks": self.sp.rate_persist_ticks},
                "attitude_gains": {k: self.gains.get(k) for k in ("rate_i_max_z", "rate_i_max_xy", "roll_priority_scale")
                                   if k in self.gains},
                # v0.7.1
                "profile": self.sp.profile, "aero": dict(self.vt_aero or {}),
                "timing": {"vane_delay_s": sc.timing.delay_s(dt_c), "esc_delay_s": sc.timing.esc_delay_s(dt_c)},
                "estimator": mode,
                "estimator_params": ({"kp": est.kp, "ki": est.ki, "accel_gate": est.accel_gate,
                                      "accel_lpf_alpha": est.alpha_a, "bias_limit_rad_s": est.bias_limit,
                                      "gated_fraction": est.gated_ticks / max(1, est.flight_ticks)}
                                     if mode == "gated_mahony" else None),
                "alt_filter": {"k1": valt.k1, "k2": valt.k2, "k3": valt.k3, "comp_m": valt.comp_m,
                               "bias_final_mps2": valt.b, "bias_limit_mps2": valt.bias_limit},
                "pushes": [{"i": i, "t0_s": pu.t0_s, "duration_s": pu.duration_s, "force_n": pu.force_n,
                            "azimuth_deg": pu.azimuth_deg, "z_body_m": pu.z_body_m, "offset_deg": pu.offset_deg,
                            "profile": pu.profile, "vertical_n": pu.vertical_n, "impulse_ns": pu.impulse_ns,
                            "contact_body_rel_cg_m": list(push_rb.get(i, (math.nan,) * 3)),
                            "pole": pu.pole, "hand_height_m": pu.hand_height_m, "pole_length_m": pu.pole_length_m,
                            "pole_elev_deg": push_elev.get(i),
                            "impulse_h_ns": (pu.impulse_ns * math.cos(math.radians(push_elev[i])) if i in push_elev
                                             else pu.impulse_ns)}
                           for i, pu in enumerate(sc.pushes)],
                # v0.8
                "plant_v09": {"hardware_config": self.hw, "power_source": (self.pw.cfg.source if self.pw is not None else None),
                              "vane_play_deg": list(self.play_deg), "rotor_inertia_kg_m2": self.rotor.i_rot_kg_m2,
                              "rotor_omega_ref_rad_s": self.rotor.omega_ref_rad_s, "t_fan_n": self.t_fan_n,
                              "t_max_kgf_22v2": self.t_fan_n / G0, "thrust_voltage_exp": self.thrust_vexp,
                              "fan_power_model": float(sc.fan_power_model),
                              "fan_power_t_ref_n": (self.pw.fan.t_ref_n if self.pw is not None else None),
                              "power": (asdict(self.pw.cfg) if self.pw is not None else None),
                              "supply_tripped": (self.pw.tripped if self.pw is not None else None),
                              "tether": (asdict(self.rig.cfg) if self.rig is not None else None),
                              "tether_anchor_m": (list(self.rig.A) if self.rig is not None else None),
                              "tether_attach_body_m": (None if getattr(self.veh, "tether_attach_body_m", None) is None
                                                       else [float(v) for v in self.veh.tether_attach_body_m]),
                              "tether_attach_rel_cg_m": (list(self.att_rb) if self.att_rb is not None else None),
                              "has_chute": bool(self.chute_cda_full > 0.0),
                              # fix round (physics review)
                              "vane_force_scale": float(sc.vane_force_scale),
                              "vane_yaw_lever_scale": float(sc.vane_yaw_lever_scale),
                              "ge_thrust_a": self.ge_a_t, "ge_vane_a": self.ge_a_q,
                              "tether_loop_limit_m": (self.loop_limit_m if self.rig is not None else None),
                              "regen_j": (self.pw.regen_j if self.pw is not None else None),
                              "clamp_j": (self.pw.clamp_j if self.pw is not None else None)},
                "build": {"gps": self.gps_on, "nav_v08": self.v08, "takeoff_check": self.tko,
                          "sup_gps_hold": self.sp.gps_hold, "sup_takeoff_check": self.sp.takeoff_check,
                          "vc_takeoff_ticks": self.vp.takeoff_ticks},
            }
            if self.gps_on:
                fl_ = nav_gps.filter
                cf_ = nav_gps.cfg
                vtol_rec["gps"] = {
                    "receiver": asdict(sc.gps), "receiver_stats": dict(gps_rx.stats),
                    "receiver_state": {"baud": gps_rx.baud, "meas_ms": gps_rx.meas_ms, "nmea": gps_rx.nmea,
                                       "enabled": sorted([list(e) for e in gps_rx.enabled]), "dyn_model": gps_rx.dyn,
                                       "n_sky": gps_rx.n_sky, "sky_scale": gps_rx.sky_scale},
                    "nav_params": asdict(self.gp_nav), "pos_params": asdict(self.pp),
                    "pos_params_derived": {"tilt_limit_kt1": list(self.pp.tilt_limit(1.0)),
                                           "tilt_limit_kt1_deg": math.degrees(math.acos(self.pp.tilt_limit(1.0)[0])),
                                           "timeout_steps": self.pp.timeout_steps, "land_steps": self.pp.land_steps},
                    "nav_stats": dict(fl_.stats), "parser_stats": dict(nav_gps.parser.stats),
                    "assembler_stats": dict(nav_gps.asm.stats), "itow_offset_ms": nav_gps.clock.offset,
                    "itow_relearned": nav_gps.clock.relearned,
                    "configurator": {"done": cf_.done, "cfg_ok": cf_.cfg_ok, "ver_ok": cf_.ver_ok,
                                     "hw": None if cf_.ver is None else cf_.ver.hw,
                                     "sw": None if cf_.ver is None else cf_.ver.sw, "naks": cf_.naks,
                                     "timeouts": cf_.timeouts, "log": list(cf_.log)},
                    "reset_count": fl_.reset_count, "offset_m": list(fl_.off), "yaw_corr_deg": math.degrees(fl_.yaw_corr),
                    "yaw_windows_used": fl_.yaw_used, "head_ok": fl_.head_ok(), "psi0_deg": math.degrees(fl_.psi0),
                    "lever_nav_b_m": list(self.gp_nav.lever_b_m), "lever_true_b_m": [float(v) for v in r_ant_b],
                    "antenna_b_mm": list(self.ant_b_mm), "nav_origin_w": None if nav_origin_w is None else list(nav_origin_w),
                    "hold_captures": hold_caps, "hold_true_last": None if hold_true is None else list(hold_true),
                    "slot_sets": self.fcu.rx.sets, "slot_broken": self.fcu.rx.broken,
                    # v0.8.1: the flight-computer chain and its glink / GNS counters
                    "chain": self.chain, "glink_latency_ticks": self.glink_L if gns_chain else None,
                    "glink": ({"states_sent": slot_tx.states, "results_latched": slot_tx.tx.latched,
                               "results_missed": slot_tx.tx.missed, "nav_rx": dict(slot_tx.rx.stats),
                               "gns_rx": dict(nav_gps.dec.stats), "gns": dict(nav_gps.stats),
                               "fcu_tilts": self.fcu.rx.tilts} if gns_chain else None),
                    "initial_yaw_deg": yaw0_deg, "heading_err_deg": float(sc.gps.heading_err_deg),
                    "pad_azimuth_deg": float(self.gp_nav.pad_azimuth_deg)}
        return SimResult(sc, log, events, self.gains, self.veh.mass_source, time.perf_counter() - t_wall,
                         t_liftoff, t_touchdown, touchdown_speed, final_state, nan_abort, touchdown_vz, link_rec, vtol_rec)

    # ------------------------------------------------------------------ #
    def _mahony(self, q_est: np.ndarray, bias: np.ndarray, gyro: np.ndarray, accel: np.ndarray, dt: float
                ) -> tuple[np.ndarray, np.ndarray]:
        """Mahony complementary filter.  Accelerometer correction only when the
        specific force magnitude is close to g (in powered flight the
        accelerometer measures thrust direction, not gravity - see report)."""
        s = self.sc.sensors
        a_n = float(np.linalg.norm(accel))
        omega = gyro - bias
        if a_n > 1e-6 and abs(a_n - self.g) / self.g < s.mahony_accel_gate:
            a_hat = accel / a_n
            w_, x_, y_, z_ = q_est
            v_hat = np.array([2.0 * (x_ * z_ - w_ * y_), 2.0 * (y_ * z_ + w_ * x_), 1.0 - 2.0 * (x_ * x_ + y_ * y_)])
            e = np.cross(a_hat, v_hat)
            bias = bias - s.mahony_ki * e * dt
            omega = omega + s.mahony_kp * e
        return quat.integrate(q_est, omega, dt), bias


# --------------------------------------------------------------------------- #
# metrics (SPEC §9 table)
# --------------------------------------------------------------------------- #
def _event_time(events: list[tuple[float, str]], needle: str) -> float | None:
    for t, e in events:
        if needle in e:
            return t
    return None


def _settling(tw: np.ndarray, tilt_deg: np.ndarray, band: float) -> float | None:
    """Time from the window start until the tilt stays inside ``band`` (None: never)."""
    above = np.flatnonzero(tilt_deg > band)
    if above.size == 0:
        return 0.0
    if above[-1] + 1 < tilt_deg.size:
        return float(tw[above[-1] + 1] - tw[0])
    return None


def _overshoot(tilt_vec: np.ndarray) -> float:
    """Largest excursion past upright along the initial tilt direction [deg]."""
    tv0 = tilt_vec[0][:2]
    n0 = float(np.linalg.norm(tv0))
    if n0 <= 1e-4:
        return 0.0
    proj = tilt_vec[:, :2] @ (tv0 / n0)
    return float(max(0.0, -np.degrees(proj.min())))


def compute_metrics(res: SimResult, settle_band_deg: float = 1.0) -> dict[str, Any]:
    """Max tilt, settling time, apogee, drift, vane use, energy, saturation ...  (a VTOL run: ``compute_vtol_metrics``)"""
    if res.scenario.mode == lp.MODE_VTOL:
        return compute_vtol_metrics(res)
    L = res.log
    t = L["t"]
    dt = float(t[1] - t[0]) if len(t) > 1 else 0.0
    active = L["control_active"]
    tilt = L["tilt_deg"]
    m: dict[str, Any] = {"scenario": res.scenario.name, "title": res.scenario.title, "mass_source": res.mass_source,
                         "runtime_s": res.runtime_s, "sim_duration_s": float(t[-1]), "nan_abort": res.nan_abort}
    # controlled window
    idx = np.flatnonzero(active)
    if idx.size:
        i0, i1 = idx[0], idx[-1]
        tw = t[i0:i1 + 1]
        tiltw = tilt[i0:i1 + 1]
        m["t_control_start_s"] = float(tw[0])
        m["t_control_end_s"] = float(tw[-1])
        m["max_tilt_deg"] = float(tiltw.max())
        m["max_tilt_overall_deg"] = float(tilt.max())
        # controller-performance metrics use the control tilt (body z vs the estimated vertical, see the log);
        # the true-tilt versions carry the attitude-estimate error on top and are reported alongside
        ctrlw = L["tilt_ctrl_deg"][i0:i1 + 1]
        m["settling_time_s"] = _settling(tw, ctrlw, settle_band_deg)
        m["settling_time_true_s"] = _settling(tw, tiltw, settle_band_deg)
        m["overshoot_deg"] = _overshoot(L["tilt_ctrl_vec"][i0:i1 + 1])
        m["overshoot_true_deg"] = _overshoot(L["tilt_vec"][i0:i1 + 1])
        err = L["att_err"][i0:i1 + 1]
        m["max_yaw_error_deg"] = float(np.degrees(np.abs(err[:, 2]).max()))
        last = tw >= tw[-1] - 1.0
        m["rms_tilt_last_1s_deg"] = float(np.sqrt(np.mean(ctrlw[last] ** 2)))
        m["rms_true_tilt_last_1s_deg"] = float(np.sqrt(np.mean(tiltw[last] ** 2)))
        dcl = L["delta_cmd"][i0:i1 + 1][last]
        m["mean_abs_delta_cmd_last_1s_deg"] = float(np.degrees(np.abs(dcl).mean()))
        # jitter = fluctuation about the 1 s mean (the trim itself, e.g. swirl compensation, is not jitter)
        m["delta_cmd_jitter_last_1s_deg"] = float(np.degrees(dcl.std(axis=0).mean())) if dcl.shape[0] > 1 else 0.0
        m["saturation_time_s"] = float(L["saturated"][i0:i1 + 1].sum() * dt)
        m["max_rate_deg_s"] = float(np.degrees(np.abs(L["omega"][i0:i1 + 1]).max()))
        m["max_delta_deg"] = float(np.degrees(np.abs(L["delta"][i0:i1 + 1]).max()))
        m["max_delta_cmd_deg"] = float(np.degrees(np.abs(L["delta_cmd"][i0:i1 + 1]).max()))
        m["max_tilt_est_err_deg"] = float(L["tilt_est_err_deg"][i0:i1 + 1].max())
    else:
        m.update(max_tilt_deg=float(tilt.max()), settling_time_s=None, overshoot_deg=None, saturation_time_s=0.0,
                 max_delta_deg=float(np.degrees(np.abs(L["delta"]).max())), max_delta_cmd_deg=0.0,
                 max_rate_deg_s=float(np.degrees(np.abs(L["omega"]).max())), max_yaw_error_deg=None,
                 max_tilt_est_err_deg=float(L["tilt_est_err_deg"].max()))
    # trajectory
    h = L["pos"][:, 2] - L["pos"][0, 2]
    k_ap = int(np.argmax(h))
    m["apogee_m"] = float(h[k_ap])
    m["t_apogee_s"] = float(t[k_ap])
    # how far the gyro-only attitude estimate has walked off by apogee (flight scenarios only)
    m["tilt_est_err_at_apogee_deg"] = None if res.scenario.bypass_state_machine else float(L["tilt_est_err_deg"][k_ap])
    m["tilt_at_apogee_deg"] = None if res.scenario.bypass_state_machine else float(tilt[k_ap])
    lateral = np.hypot(L["pos"][:, 0] - L["pos"][0, 0], L["pos"][:, 1] - L["pos"][0, 1])
    m["lateral_drift_at_apogee_m"] = float(lateral[k_ap])
    m["lateral_drift_max_m"] = float(lateral.max())
    m["lateral_drift_final_m"] = float(lateral[-1])
    m["max_speed_mps"] = float(np.linalg.norm(L["vel"], axis=1).max())
    m["max_vz_mps"] = float(L["vel"][:, 2].max())
    m["energy_Wh"] = float(L["energy_j"][-1] / 3600.0)
    m["final_v_cell"] = float(L["v_cell"][-1])
    m["max_thrust_N"] = float(L["thrust"].max())
    m["max_altitude_error_est_m"] = float(np.abs(L["h_est"] - h).max())
    # events (onboard supervisor transitions, SPEC 8.8)
    ev = [(te, e) for te, e in res.events if not e.startswith("GS ")]
    m["t_liftoff_s"] = res.t_liftoff
    m["t_launch_s"] = _event_time(ev, "ARMED -> POWERED")
    m["t_cutoff_s"] = _event_time(ev, "POWERED -> UNPOWERED")
    m["cut_by"] = next((e.split(": ", 1)[1].split(" (")[0].split(":")[0] for _, e in ev if "POWERED -> UNPOWERED" in e
                        and not e.startswith("UN")), None)
    if m["t_cutoff_s"] is not None:
        kc = int(np.searchsorted(t, m["t_cutoff_s"]))
        kc = min(kc, len(h) - 1)
        m["h_cutoff_m"] = float(h[kc])
        m["vz_cutoff_mps"] = float(L["vel"][kc, 2])
    m["t_abort_s"] = _event_time(ev, "-> ABORT")
    m["abort_reason"] = next((e.split(": ", 1)[1] for _, e in ev if "-> ABORT" in e), None)
    m["t_release_cmd_s"] = next((te for te, e in ev if "release in" in e), None)
    m["release_by"] = next((e.split(": ", 1)[1].split(" (")[0].split(":")[0] for _, e in ev if "release in" in e), None)
    m["t_deploy_s"] = _event_time(ev, "-> RELEASED")
    if m["t_deploy_s"] is not None:
        kd = min(int(np.searchsorted(t, m["t_deploy_s"])), len(h) - 1)
        m["h_deploy_m"] = float(h[kd])
        m["vz_release_mps"] = float(L["vel"][kd, 2])
        m["release_minus_apogee_s"] = float(t[kd] - t[k_ap])
    m["interlocks"] = sorted({e.split(": ", 1)[1].split(" ")[0] for _, e in ev
                              if ": I" in e and " -> " in e or "I6 fall-safe" in e})
    m["t_touchdown_s"] = res.t_touchdown
    m["touchdown_speed_mps"] = res.touchdown_speed_mps
    m["touchdown_vz_mps"] = res.touchdown_vz_mps
    # peak yaw rate after motor cut-off (swirl torque decays slower than the centring vanes)
    if m["t_cutoff_s"] is not None:
        kc = min(int(np.searchsorted(t, m["t_cutoff_s"])), len(t) - 1)
        m["max_yaw_rate_after_cutoff_deg_s"] = float(np.degrees(np.abs(L["omega"][kc:, 2]).max()))
    m["final_state"] = res.final_state.name
    m["fault_t_s"] = _event_time(ev, "FAULT")
    if res.link is not None:
        m.update(link_metrics(res))
    return m


def _stats(v: list[float]) -> dict[str, float | int | None]:
    if not v:
        return {"n": 0, "mean": None, "p95": None, "max": None, "min": None}
    a = np.asarray(v, dtype=float)
    return {"n": int(a.size), "mean": float(a.mean()), "p95": float(np.percentile(a, 95)), "max": float(a.max()),
            "min": float(a.min())}


def command_latencies(res: SimResult) -> list[dict[str, Any]]:
    """Each ground command matched to its onboard execution (same SEQ, first processing after the first
    transmission): decision -> execution, first transmission -> execution, decision -> acknowledgement."""
    lk = res.link or {}
    dt = lk.get("dt", 0.0)
    rows = []
    for r in lk.get("records", []):
        row = {"cmd": r["name"], "source": r["source"], "t_queued": r["t_queued"], "seq": r["seq"], "n_tx": r["n_tx"],
               "result": r["result"] or ("pending" if r["n_tx"] else "never sent"), "t_exec": None, "exec_code": None,
               "decision_to_exec_s": None, "tx_to_exec_s": None, "decision_to_ack_s": None}
        if r["t_first_tx"] is not None:
            ex = next((e for e in lk.get("exec_log", []) if e["seq"] == r["seq"] and e["tick"] * dt >= r["t_first_tx"] - 1e-9),
                      None)
            if ex is not None:
                row["t_exec"] = ex["tick"] * dt
                row["exec_code"] = ex["code"]
                row["decision_to_exec_s"] = row["t_exec"] - r["t_queued"]
                row["tx_to_exec_s"] = row["t_exec"] - r["t_first_tx"]
        if r["t_ack"] is not None:
            row["decision_to_ack_s"] = r["t_ack"] - r["t_queued"]
        rows.append(row)
    return rows


def link_metrics(res: SimResult) -> dict[str, Any]:
    lk = res.link
    rows = command_latencies(res)
    out: dict[str, Any] = {"link_radio": lk["radio"], "link_loss": lk["loss"], "rtt": _stats(lk["rtt"]),
                           "polls": lk["ground_stats"]["polls"], "replies": lk["ground_stats"]["replies"],
                           "timeouts": lk["ground_stats"]["timeouts"], "link_stats": lk["link_stats"],
                           "commands": rows,
                           "cmd_decision_to_exec": _stats([r["decision_to_exec_s"] for r in rows if r["decision_to_exec_s"] is not None]),
                           "cmd_tx_to_exec": _stats([r["tx_to_exec_s"] for r in rows if r["tx_to_exec_s"] is not None]),
                           "cmd_decision_to_ack": _stats([r["decision_to_ack_s"] for r in rows if r["decision_to_ack_s"] is not None]),
                           "cmd_tx_count": _stats([float(r["n_tx"]) for r in rows if r["n_tx"]])}
    for name in ("CUT", "RELEASE", "LAUNCH", "ARM"):
        r = next((r for r in rows if r["cmd"] == name and r["t_exec"] is not None), None)
        out[f"{name.lower()}_decision_to_exec_s"] = None if r is None else r["decision_to_exec_s"]
    a = next((x for x in lk["assist_log"] if x["cmd"] == "CUT"), None)
    out["assist_cut"] = a
    out["assist_release"] = next((x for x in lk["assist_log"] if x["cmd"] == "RELEASE"), None)
    return out


def scenario_to_dict(sc: Scenario) -> dict[str, Any]:
    d = asdict(sc)
    return d


# --------------------------------------------------------------------------- #
# VTOL metrics (SPEC 8.9; the pass / fail criteria live in sim.vtol_scenarios)
# --------------------------------------------------------------------------- #
GUST_RECOVERY_WINDOW_S = 5.0        # gust recovery is looked for this long after each gust (or until the next one)
FALSE_TOUCHDOWN_HEIGHT_M = 0.05     # LANDED confirmed with the exit plane more than this above the surface = in the air


def tip_over_vcrit(m_kg: float, i_cg: float, r_m: float, h_cg_m: float, lean_toward_deg: float,
                   g: float = G0) -> float:
    """Energy-method tip-over threshold at touchdown (params mission.vtol.position_hold.tipover: "tip-over margin from
    lean + ground speed (energy method) > 0"; v0.8 physics review medium 8, scratch v08/physreview/calc_tip.py): the
    vehicle moving at v_h catches the downstream footprint edge (radius r); angular momentum about the edge m v_h h_e
    (h_e = d cos a0, d = hypot(r, h_cg)) is kept through the impact and the rotation tips the vehicle over when its
    kinetic energy L^2 / (2 I_edge) exceeds the CG rise m g d (1 - cos a0); a0 = atan(r / h_cg) - lean_toward (lean
    toward the motion brings the CG nearer the edge), I_edge = I_cg + m d^2.  Returns v_crit [m/s] (0 if a0 <= 0: the
    CG already past the edge).  The vertical speed is ignored (its impulse acts through the edge - conservative)."""
    d = math.hypot(r_m, h_cg_m)
    a0 = math.atan2(r_m, h_cg_m) - math.radians(lean_toward_deg)
    if a0 <= 0.0:
        return 0.0
    i_edge = i_cg + m_kg * d * d
    return math.sqrt(2.0 * m_kg * g * d * (1.0 - math.cos(a0)) * i_edge) / (m_kg * d * math.cos(a0))


def exit_plane_positions(res: SimResult) -> np.ndarray:
    """World position of the exit-plane centre (the renderer's ``pos``, the pad contact point) per sample: the logged
    CG minus R(q) (0, 0, z_cg), with the post-ejection CG after a release."""
    L = res.log
    vr = res.vtol or {}
    z_full = float(vr.get("z_cg_full_m", 0.0))
    z_post = float(vr.get("z_cg_post_m", z_full))
    t_dep = vr.get("t_deploy")
    zc = np.where(L["t"] >= (t_dep if t_dep is not None else np.inf), z_post, z_full)
    q = L["q"]
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r02 = 2.0 * (x * z + w * y)
    r12 = 2.0 * (y * z - w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return L["pos"] - np.stack([r02, r12, r22], axis=1) * zc[:, None]


def _rms(a: np.ndarray) -> float | None:
    return float(np.sqrt(np.mean(np.square(a)))) if a.size else None


def body_z_world(q: np.ndarray) -> np.ndarray:
    """Body z axis in the world frame (third column of R(q)) for an (N, 4) quaternion array."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)], axis=1)


def gust_recovery(res: SimResult, band_deg: float = 2.0, window_s: float = GUST_RECOVERY_WINDOW_S) -> list[dict[str, Any]]:
    """For each gust of the schedule (and the single scenario gust): peak true tilt, baseline tilt over the second before
    it, and the recovery time = time after the gust's END until the true tilt is back below ``band_deg`` for the rest of
    the window (gust end + ``window_s``, cut at the next gust and at the end of POWERED); 0 if it never left the band
    after the gust ended, None if it is still outside at the window end.  Reported alongside (diagnostics, not the
    criterion): ``_ctrl`` = the same on the control tilt (the attitude estimate; its gyro-only drift is an offset the
    loop cannot see) and ``_dev`` = the true attitude's deviation from its mean attitude over the second before the gust
    (angle between the body axes; separates the gust response from the slow estimate-drift tilt)."""
    L, sc = res.log, res.scenario
    t = L["t"]
    bz = body_z_world(L["q"])
    gusts = list(sc.gusts)
    t_l = (res.link or {}).get("t_launch")
    if sc.gust_mps and t_l is not None:
        gusts.append(Gust(t_l + sc.gust_t0_after_launch_s, sc.gust_duration_s, sc.gust_mps,
                          sc.gust_dir_deg if sc.gust_dir_deg is not None else sc.wind_dir_deg))
    gusts.sort(key=lambda g: g.t0_s)
    powered = L["state"] == lp.STATE_POWERED
    pw = np.flatnonzero(powered)
    t_pw_end = float(t[pw[-1]]) if pw.size else float(t[-1])
    rows = []
    for i, g in enumerate(gusts):
        w_end = min(g.t1_s + window_s, gusts[i + 1].t0_s if i + 1 < len(gusts) else np.inf, t_pw_end, float(t[-1]))
        row: dict[str, Any] = {"t0_s": g.t0_s, "t1_s": g.t1_s, "speed_mps": g.speed_mps, "dir_deg": g.dir_deg,
                               "rise_s": g.rise_s, "window_end_s": w_end}
        inwin = (t >= g.t0_s) & (t <= w_end)
        before = (t >= g.t0_s - 1.0) & (t < g.t0_s)
        i_g = int(np.searchsorted(t, g.t0_s))
        if not np.any(inwin) or i_g >= t.size or not powered[i_g]:
            row.update(in_flight=False)            # the gust came while the vehicle was not flying under power
            rows.append(row)
            continue
        row["in_flight"] = True
        b0 = bz[before].mean(axis=0) if np.any(before) else bz[i_g]
        b0 = b0 / max(float(np.linalg.norm(b0)), 1e-12)
        dev = np.degrees(np.arccos(np.clip(bz @ b0, -1.0, 1.0)))
        for key, arr in (("", L["tilt_deg"]), ("_ctrl", L["tilt_ctrl_deg"]), ("_dev", dev)):
            row[f"peak_tilt{key}_deg"] = float(arr[inwin].max())
            row[f"baseline_tilt{key}_deg"] = float(arr[before].mean()) if np.any(before) else None
            after = (t >= g.t1_s) & (t <= w_end)
            ta, aa = t[after], arr[after]
            out_idx = np.flatnonzero(aa > band_deg)
            if ta.size == 0:
                rec = None
            elif out_idx.size == 0:
                rec = 0.0
            elif out_idx[-1] + 1 < ta.size:
                rec = float(ta[out_idx[-1] + 1] - g.t1_s)
            else:
                rec = None
            row[f"recovery{key}_s"] = rec
        rows.append(row)
    return rows


PUSH_WINDOW_S = 5.0                 # push response is looked at this long after each push (or until the next one)


def push_response(res: SimResult, band_deg: float = 2.0, window_s: float = PUSH_WINDOW_S) -> list[dict[str, Any]]:
    """Per hand push (``Scenario.pushes``): the window runs from the push start to push end + ``window_s`` (cut at the
    next push, at the end of POWERED and at the log end).  Reported: the push (force, impulse, height, direction),
    peak true tilt, peak deviation from the mean attitude over the second before the push (``_dev``: the angle between
    body axes), recovery = time after the push END until that deviation stays below ``band_deg`` for the rest of the
    window (0 if it never left the band after the push ended, None if still outside at the window end; ``recovery_true_s``
    the same on the true tilt - the renderer's rule), the peak horizontal displacement of the exit plane from its
    position at the push start, the altitude excursion (max |h - h(t0)|) and drop (h(t0) - min h), the minimum height of
    the exit plane above the surface below it, and the interlock reason codes that appeared in the window."""
    L, sc = res.log, res.scenario
    t = L["t"]
    pushes = sorted(sc.pushes, key=lambda pu: pu.t0_s)
    if not pushes:
        return []
    bz = body_z_world(L["q"])
    pe = exit_plane_positions(res)
    h = L["h_true"]
    hs = pe[:, 2] - L["surface_z"]
    powered = L["state"] == lp.STATE_POWERED
    pw = np.flatnonzero(powered)
    t_pw_end = float(t[pw[-1]]) if pw.size else float(t[-1])
    rows = []
    for i, pu in enumerate(pushes):
        w_end = min(pu.t1_s + window_s, pushes[i + 1].t0_s if i + 1 < len(pushes) else np.inf, float(t[-1]))
        i0 = int(np.searchsorted(t, pu.t0_s))
        row: dict[str, Any] = {"i": i, "t0_s": pu.t0_s, "t1_s": pu.t1_s, "force_n": pu.force_n, "impulse_ns": pu.impulse_ns,
                               "duration_s": pu.duration_s, "azimuth_deg": pu.azimuth_deg, "z_body_m": pu.z_body_m,
                               "offset_deg": pu.offset_deg, "profile": pu.profile, "vertical_n": pu.vertical_n,
                               "window_end_s": w_end}
        if i0 >= t.size or not powered[i0]:
            row["in_flight"] = False
            rows.append(row)
            continue
        row["in_flight"] = True
        w_pw = min(w_end, t_pw_end) if t_pw_end >= pu.t0_s else w_end
        inwin = (t >= pu.t0_s) & (t <= w_end)
        before = (t >= pu.t0_s - 1.0) & (t < pu.t0_s)
        b0 = bz[before].mean(axis=0) if np.any(before) else bz[i0]
        b0 = b0 / max(float(np.linalg.norm(b0)), 1e-12)
        dev = np.degrees(np.arccos(np.clip(bz @ b0, -1.0, 1.0)))
        for key, arr in (("_true", L["tilt_deg"]), ("", dev)):
            after = (t >= pu.t1_s) & (t <= w_pw)
            ta, aa = t[after], arr[after]
            out_idx = np.flatnonzero(aa > band_deg)
            if ta.size == 0:
                rec = None
            elif out_idx.size == 0:
                rec = 0.0
            elif out_idx[-1] + 1 < ta.size:
                rec = float(ta[out_idx[-1] + 1] - pu.t1_s)
            else:
                rec = None
            row[f"recovery{key}_s"] = rec
        row["peak_tilt_deg"] = float(L["tilt_deg"][inwin].max())
        row["baseline_tilt_deg"] = float(L["tilt_deg"][before].mean()) if np.any(before) else None
        row["peak_dev_deg"] = float(dev[inwin].max())
        row["peak_horiz_disp_m"] = float(np.hypot(pe[inwin, 0] - pe[i0, 0], pe[inwin, 1] - pe[i0, 1]).max())
        row["alt_excursion_m"] = float(np.abs(h[inwin] - h[i0]).max())
        row["alt_drop_m"] = float(h[i0] - h[inwin].min())
        row["min_height_m"] = float(hs[inwin].min())
        row["max_rate_deg_s"] = float(np.degrees(np.abs(L["omega"][inwin]).max()))
        row["max_yaw_rate_deg_s"] = float(np.degrees(np.abs(L["omega"][inwin, 2]).max()))
        # fix round: the tilt rate |w_xy| (the I3 tilt-rate test) separately from the spin |w_z| (the I3 spin test)
        row["max_rate_xy_deg_s"] = float(np.degrees(np.hypot(L["omega"][inwin, 0], L["omega"][inwin, 1]).max()))
        row["max_rate_z_deg_s"] = row["max_yaw_rate_deg_s"]
        row["sat_high_s"] = float(((L["vc_sat"] == 1) & inwin).sum() * (t[1] - t[0]))
        codes0 = set(np.unique(L["reason"][:i0]).tolist()) if i0 > 0 else set()
        row["interlock_codes"] = sorted(int(c) for c in set(np.unique(L["reason"][inwin]).tolist()) - codes0
                                        if 0 < int(c) < 0x40)
        row["interlocks"] = [lp.REASON_NAMES.get(c, str(c)) for c in row["interlock_codes"]]
        row["abort"] = bool(np.any(inwin & (L["state"] == lp.STATE_ABORT)))
        row["abort_cause"] = None
        if row["abort"] and lp.R_I3_RATE in row["interlock_codes"]:
            ka = int(np.flatnonzero(inwin & (L["state"] == lp.STATE_ABORT))[0])
            ac = rate_abort_cause(res, float(t[ka]))
            row["abort_cause"] = None if ac is None else ac["cause"]
        elif row["abort"] and lp.R_I2_TILT in row["interlock_codes"]:
            row["abort_cause"] = "tilt"
        row["still_powered_at_window_end"] = bool(powered[min(int(np.searchsorted(t, w_end)), t.size - 1)])
        row.update(push_position_response(res, i, pu, pushes))
        rows.append(row)
    return rows


POS_RETURN_WINDOW_S = 8.0           # v0.8: the position return after a push is looked for this long after its end (cut at
                                    # the next push and at the end of POWERED)
POS_RETURN_BANDS_M = (1.0, 0.5)     # v0.8: "back within" these distances of the pre-push position (params criteria_v08:
                                    # 1 m; the prototype's 0.5 m reported)


def push_position_response(res: SimResult, i: int, pu: Push, pushes: list[Push]) -> dict[str, Any]:
    """v0.8 position response of push i (TRUE CG, horizontal): the pre-push position = the mean CG xy over the second
    before the push; ``excursion_m`` = the largest distance from it from the push start to the window end
    (POS_RETURN_WINDOW_S after the push end, cut at the next push / the end of POWERED / the log); ``return_<b>m_s`` =
    time from the push END until the CG is back within b metres after the excursion peak (0 if it never left the band,
    None if not back before the window end); the pole elevation and the horizontal impulse of a pole push.  With GPS the
    NAV's own view (``pos_dist``, distance from its hold point) is reported too."""
    L = res.log
    t = L["t"]
    out: dict[str, Any] = {}
    vr = res.vtol or {}
    pr = next((r for r in vr.get("pushes", []) if r.get("t0_s") == pu.t0_s), {})
    out["pole"] = bool(pr.get("pole", False))
    out["pole_elev_deg"] = pr.get("pole_elev_deg")
    out["impulse_h_ns"] = pr.get("impulse_h_ns", pu.impulse_ns)
    before = (t >= pu.t0_s - 1.0) & (t < pu.t0_s)
    i0 = int(np.searchsorted(t, pu.t0_s))
    if i0 >= t.size:
        return out
    cg = L["pos"][:, :2]
    pre = cg[before].mean(axis=0) if np.any(before) else cg[i0]
    powered = L["state"] == lp.STATE_POWERED
    pw = np.flatnonzero(powered & (t >= pu.t0_s))
    t_pw_end = float(t[pw[-1]]) if pw.size else float(t[i0])
    nxt = [q.t0_s for q in pushes if q.t0_s > pu.t0_s]
    w_end = min(pu.t1_s + POS_RETURN_WINDOW_S, min(nxt) if nxt else np.inf, t_pw_end, float(t[-1]))
    win = (t >= pu.t0_s) & (t <= w_end)
    d = np.hypot(cg[:, 0] - pre[0], cg[:, 1] - pre[1])
    out["pre_xy"] = [float(pre[0]), float(pre[1])]
    out["pos_window_end_s"] = float(w_end)
    if not np.any(win):
        return out
    iw = np.flatnonzero(win)
    k_pk = int(iw[np.argmax(d[iw])])
    out["excursion_m"] = float(d[k_pk])
    out["t_excursion_peak_s"] = float(t[k_pk])
    for b in POS_RETURN_BANDS_M:
        key = f"return_{str(b).replace('.', 'p')}m_s"
        if d[k_pk] <= b:
            out[key] = 0.0
            continue
        after = np.flatnonzero(win & (t >= t[k_pk]) & (d <= b))
        out[key] = float(max(t[after[0]] - pu.t1_s, 0.0)) if after.size else None
    if "pos_dist" in L:
        pdw = L["pos_dist"][iw]
        out["nav_excursion_m"] = float(np.nanmax(pdw)) if np.any(np.isfinite(pdw)) else None
    return out


def rate_abort_cause(res: SimResult, t_abort: float | None) -> dict[str, Any] | None:
    """Cause of an I3 ABORT (fix round; the reason code is the same for both): over the persistence window before the
    ABORT (``rate_persist_ticks`` ticks), how many ticks had the TRUE tilt rate |w_xy| above tilt_rate_abort and how many
    the spin |w_z| above spin_rate_abort (the supervisor tests the bias-corrected gyro, within the 0.5 deg/s bias of
    these); ``cause`` = "tilt_rate", "spin" or "tilt_rate+spin" (both in every tick of the window), "gyro_saturation"
    when neither, None without an ABORT."""
    if t_abort is None or not res.vtol or "rate_abort" not in res.vtol:
        return None
    ra = res.vtol["rate_abort"]
    t = res.log["t"]
    ka = int(np.searchsorted(t, t_abort - 1e-9))
    n = int(ra["rate_persist_ticks"])
    w = res.log["omega"][max(0, ka - n):ka + 1]
    if w.shape[0] == 0:
        return None
    wxy = np.hypot(w[:, 0], w[:, 1])
    wz = np.abs(w[:, 2])
    n_xy = int(np.sum(wxy > ra["tilt_rate_abort_rad_s"]))
    n_z = int(np.sum(wz > ra["spin_rate_abort_rad_s"]))
    m = w.shape[0]
    if n_xy >= m - 1 and n_z >= m - 1:
        cause = "tilt_rate+spin"
    elif n_xy == 0 and n_z == 0:
        cause = "gyro_saturation"
    else:
        cause = "spin" if n_z >= n_xy else "tilt_rate"
    return {"cause": cause, "window_ticks": m, "ticks_tilt_rate": n_xy, "ticks_spin": n_z,
            "max_w_xy_deg_s": float(np.degrees(wxy.max())), "max_w_z_deg_s": float(np.degrees(wz.max()))}


def push_recovered(row: dict[str, Any], recovery_max_s: float) -> bool:
    """A push is recovered: in flight, no interlock acted in its window, the motor still runs, and the attitude is back
    within the band of its pre-push attitude within ``recovery_max_s`` after the push ended."""
    return bool(row.get("in_flight") and not row.get("interlock_codes") and not row.get("abort")
                and row.get("recovery_s") is not None and row["recovery_s"] <= recovery_max_s)


def compute_vtol_metrics(res: SimResult) -> dict[str, Any]:
    """Everything the VTOL verdicts and the campaign use (``sim.vtol_scenarios.NOMINAL_CRITERIA`` judges them)."""
    L, sc = res.log, res.scenario
    vr = res.vtol or {}
    t = L["t"]
    dt = float(t[1] - t[0]) if len(t) > 1 else 0.0
    state, phase = L["state"], L["phase"]
    P = state == lp.STATE_POWERED
    hold = phase == lp.PHASE_HOLD
    m: dict[str, Any] = {"scenario": sc.name, "title": sc.title, "mode": lp.MODE_VTOL, "seed": sc.seed,
                         "profile": vr.get("profile"), "build": vr.get("build"),
                         "mass_source": res.mass_source, "runtime_s": res.runtime_s, "sim_duration_s": float(t[-1]),
                         "nan_abort": res.nan_abort, "final_state": res.final_state.name,
                         "final_phase": lp.PHASE_NAMES.get(int(phase[-1]), str(int(phase[-1])))}
    ev = [(te, e) for te, e in res.events if not e.startswith("GS ")]

    def first(mask: np.ndarray) -> float | None:
        idx = np.flatnonzero(mask)
        return float(t[idx[0]]) if idx.size else None

    m["t_on_s"] = _event_time(ev, "SAFE -> ARMED")
    m["t_motor_start_s"] = _event_time(ev, "ARMED -> POWERED")
    m["t_liftoff_s"] = res.t_liftoff
    m["t_hold_s"] = first(hold)
    m["t_land_s"] = first(phase == lp.PHASE_LAND)
    land_ev = next((e for _, e in ev if "-> LAND: " in e), None)
    m["land_by"] = None if land_ev is None else land_ev.split("-> LAND: ", 1)[1].split(" ")[0]
    m["t_landed_s"] = _event_time(ev, "POWERED -> UNPOWERED")
    m["t_safe_after_landed_s"] = _event_time(ev, "UNPOWERED -> SAFE")
    m["t_abort_s"] = _event_time(ev, "-> ABORT")
    m["abort_reason"] = next((e.split(": ", 1)[1] for _, e in ev if "-> ABORT" in e), None)
    m["t_released_s"] = _event_time(ev, "-> RELEASED")
    m["t_fault_s"] = _event_time(ev, "FAULT")
    codes = sorted({int(c) for c in np.unique(L["reason"]) if 0 < int(c) < 0x40})
    m["interlock_codes"] = codes
    m["interlocks"] = [lp.REASON_NAMES.get(c, str(c)) for c in codes]
    rej = [e for e in (res.link or {}).get("exec_log", []) if e["code"]]
    m["rejections"] = [{"t": e["tick"] * dt, "cmd": lp.CMD_NAMES.get(e["cmd"], str(e["cmd"])),
                        "reason": lp.REASON_NAMES.get(e["code"], str(e["code"]))} for e in rej]
    m["motor_time_s"] = float(vr.get("powered_ticks", int(P.sum())) * dt)
    m["landed_normally"] = m["t_landed_s"] is not None and m["t_abort_s"] is None
    # ---- altitude ----
    h = L["h_true"]
    href, htgt = L["h_ref"], L["h_tgt"]
    m["max_h_m"] = float(h.max())
    m["hold_s"] = float(hold.sum() * dt)
    if hold.any():
        e = h[hold] - href[hold]
        m["hold_rms_err_m"] = _rms(e)
        m["hold_max_abs_err_m"] = float(np.abs(e).max())
        m["hold_mean_err_m"] = float(e.mean())
        settled = hold & (href == htgt)
        m["hold_rms_err_settled_m"] = _rms(h[settled] - htgt[settled])
        m["hold_rms_est_err_m"] = _rms(L["h_est"][hold] - h[hold])
    else:
        m.update(hold_rms_err_m=None, hold_max_abs_err_m=None, hold_mean_err_m=None, hold_rms_err_settled_m=None,
                 hold_rms_est_err_m=None)
    tg = htgt[P & np.isfinite(htgt)]
    if tg.size:
        tgt0 = float(tg[0])
        ip = np.flatnonzero(P)
        chg = np.flatnonzero(P & np.isfinite(htgt) & (htgt != tgt0))
        i_end = chg[0] if chg.size else ip[-1] + 1
        m["first_target_m"] = tgt0
        m["climb_overshoot_m"] = float(h[ip[0]:i_end].max() - tgt0)
        # fix round (control review medium 6): the take-off transient alone - from the motor start to
        # CLIMB_OVERSHOOT_WINDOW_S after the reference reached the first target (HOLD) - the judged quantity (the whole
        # first-target window above also holds the hold-phase baro excursions)
        th = m.get("t_hold_s")
        i_w = i_end if th is None else min(i_end, int(np.searchsorted(L["t"], th + CLIMB_OVERSHOOT_WINDOW_S)))
        m["climb_overshoot_transient_m"] = float(h[ip[0]:max(i_w, ip[0] + 1)].max() - tgt0)
        m["t_climb_s"] = None if m["t_hold_s"] is None or m["t_motor_start_s"] is None else m["t_hold_s"] - m["t_motor_start_s"]
    m["targets_m"] = sorted({float(v) for v in htgt[np.isfinite(htgt)]})
    # ---- throttle / saturation ----
    u = L["throttle"]
    m["u_hover_model"] = vr.get("u_hover_model")
    m["hover_margin"] = vr.get("hover_margin")
    if P.any():
        sat = L["vc_sat"]
        ramp_ticks = int((vr.get("vp") or {}).get("ramp_ticks", 0))
        ip = np.flatnonzero(P)
        in_ramp = np.zeros_like(P)
        # the take-off ramp ceiling (designed saturation) covers the first ramp_ticks ticks of each POWERED stretch
        starts = ip[np.r_[True, np.diff(ip) > 1]]
        for s0 in starts:
            in_ramp[s0:s0 + ramp_ticks] = True
        m["sat_high_s"] = float(((sat == 1) & P).sum() * dt)
        m["sat_high_after_ramp_s"] = float(((sat == 1) & P & ~in_ramp).sum() * dt)
        m["sat_low_s"] = float(((sat == -1) & P).sum() * dt)
        m["sat_fraction_powered"] = float(((sat != 0) & P & ~in_ramp).sum() / max(1, int((P & ~in_ramp).sum())))
        m["u_max_used"] = float(u[P].max())
        m["u_mean_powered"] = float(u[P].mean())
        uh = u[hold]
        m["u_mean_hold"] = float(uh.mean()) if uh.size else None
        m["u_p99_hold"] = float(np.percentile(uh, 99)) if uh.size else None
        m["u_jitter_hold"] = float(np.diff(uh).std()) if uh.size > 2 else None
        m["vc_integ_final"] = float(L["vc_integ"][ip[-1]])
        m["vc_integ_max_abs"] = float(np.abs(L["vc_integ"][P]).max())
        # ---- attitude ----
        tilt = L["tilt_deg"]
        m["max_tilt_powered_deg"] = float(tilt[P].max())
        m["max_tilt_ctrl_powered_deg"] = float(L["tilt_ctrl_deg"][P].max())
        m["rms_tilt_hold_deg"] = _rms(tilt[hold]) if hold.any() else None
        m["max_tilt_est_err_deg"] = float(L["tilt_est_err_deg"][P].max())
        m["rms_tilt_est_err_deg"] = _rms(L["tilt_est_err_deg"][P])
        m["tilt_est_err_end_powered_deg"] = float(L["tilt_est_err_deg"][ip[-1]])
        if "est_gate" in L:
            m["est_gate_fraction_powered"] = float(L["est_gate"][P].mean())
        m["max_rate_powered_deg_s"] = float(np.degrees(np.abs(L["omega"][P]).max()))
        m["max_delta_cmd_deg"] = float(np.degrees(np.abs(L["delta_cmd"][P]).max()))
        m["vane_saturation_s"] = float((L["saturated"] & P).sum() * dt)
    m["gusts"] = gust_recovery(res)
    gin = [g for g in m["gusts"] if g.get("in_flight")]
    m["n_gusts_in_flight"] = len(gin)
    for key in ("", "_ctrl", "_dev"):
        rec = [g.get(f"recovery{key}_s") for g in gin]
        m[f"gust_recovery{key}_max_s"] = (None if any(r is None for r in rec) else max(rec)) if rec else 0.0
        m[f"gust_peak_tilt{key}_max_deg"] = max((g[f"peak_tilt{key}_deg"] for g in gin), default=None)
    # ---- touchdown / landing ----
    tds = vr.get("touchdowns", [])
    m["n_touchdowns"] = len(tds)
    m["n_liftoffs"] = len(vr.get("liftoffs", []))
    td = landing_touchdown(tds)
    for k_ in ("vz", "vh", "tilt_deg", "surface_z", "on_pad", "u_esc", "x", "y", "rate_deg_s"):
        m[f"touchdown_{k_}"] = None if td is None else td[k_]
    # v0.7.1: horizontal touchdown speed relative to the air mass the vehicle drifts with (the mean wind: steady + swing
    # / walk / reversal, no turbulence, no gusts - the controller criterion) and to the instantaneous local wind
    m["touchdown_vh_air"] = None if td is None else td.get("vh_air_mean")
    m["touchdown_vh_air_local"] = None if td is None else td.get("vh_air_local")
    m["touchdown_wind_mean_mps"] = None if td is None else math.hypot(td.get("wind_mean_x", 0.0), td.get("wind_mean_y", 0.0))
    m["touchdown_t_s"] = None if td is None else td["t"]
    m["touchdown_phase"] = None if td is None else lp.PHASE_NAMES.get(td["phase"], str(td["phase"]))
    m["touchdown_state"] = None if td is None else lp.STATE_NAMES.get(td["state"], str(td["state"]))
    # v0.8 physics-review fix (medium 8): the energy-method tip-over margin at the first contact (params tipover)
    for k_ in ("lean_toward_deg", "tip_vcrit_mps", "tip_margin_mps", "tip_vcrit_geom_mps", "tip_margin_geom_mps",
               "tip_footprint_r_m"):
        m[f"touchdown_{k_}"] = None if td is None else td.get(k_)
    m["touchdown_max_abs_vz"] = max((abs(d["vz"]) for d in tds), default=None)
    m["touchdown_max_vh"] = max((d["vh"] for d in tds), default=None)
    m["touchdown_distance_m"] = None if td is None else math.hypot(td["x"], td["y"])
    m["touchdown_confirm_s"] = (None if td is None or m["t_landed_s"] is None else m["t_landed_s"] - td["t"])
    pe = exit_plane_positions(res)
    # LANDED confirmed while still airborne (the motor stops in the air: the vehicle falls the rest of the way); a vehicle
    # that never left the surface (e.g. too little thrust) is not a false touchdown
    if m["t_landed_s"] is not None:
        kl = min(int(np.searchsorted(t, m["t_landed_s"] - 1e-9)), len(t) - 1)
        m["height_at_landed_m"] = float(pe[kl, 2] - L["surface_z"][kl])     # exit plane above the surface below it
    else:
        m["height_at_landed_m"] = None
    m["false_touchdown"] = bool(m["height_at_landed_m"] is not None and m["height_at_landed_m"] > FALSE_TOUCHDOWN_HEIGHT_M)
    # v0.7.1: which touchdown signature confirmed LANDED (still / support / floor), or a supervisor rule (V8 / V10)
    m["landed_by"] = None
    if m["t_landed_s"] is not None and "vc_td_by" in L:
        kl = int(np.searchsorted(t, m["t_landed_s"] - 1e-9))
        by = int(L["vc_td_by"][max(kl - 1, 0)])
        if by:
            m["landed_by"] = TD_NAMES.get(by, str(by))            # v0.7.1b: 4 = "all" (above the flare band)
        else:
            rc = int(L["reason"][min(kl, len(t) - 1)])
            m["landed_by"] = lp.REASON_NAMES.get(rc, f"reason {rc}") if rc else "unknown"
    # fix round: the cause of an I3 ABORT (tilt rate vs spin), lift-off, the v0.7.1b thrust learning
    ac = rate_abort_cause(res, m["t_abort_s"]) if (m["t_abort_s"] is not None and lp.R_I3_RATE in codes) else None
    m["abort_rate_cause"] = None if ac is None else ac["cause"]
    m["abort_rate_detail"] = ac
    m["lifted_off"] = res.t_liftoff is not None
    m["no_liftoff"] = bool(res.t_liftoff is None and m["t_motor_start_s"] is not None)
    # cannot fly: no lift-off, or never above the flare band (a plant at T/W ~1 hops on the pad; k_T never learns)
    fl = (vr.get("vp") or {}).get("flare_altitude_m")
    m["cannot_fly"] = bool(m["no_liftoff"] or (m["t_motor_start_s"] is not None and fl is not None
                                               and m["max_h_m"] < float(fl)))
    if "vc_k_thrust" in L and P.any() and not m["no_liftoff"]:          # (no lift-off: k_T never learns)
        kt = L["vc_k_thrust"]
        lrn = L["vc_kt_learn"] & P
        m["kt_learn_s"] = float(lrn.sum() * dt)
        m["kt_final"] = float(kt[np.flatnonzero(P)[-1]])
        m["kt_hold_mean"] = float(kt[hold].mean()) if hold.any() else None
        m["kt_min"] = float(kt[P].min())
        m["kt_max"] = float(kt[P].max())
        pl = vr.get("plant") or {}
        tsc = float(pl.get("thrust_scale", 1.0))
        msc = float(pl.get("mass_scale", 1.0))
        kv = float(pl.get("thrust_voltage_exp", 0.0))
        vn = float(pl.get("v_nom_cell", 0.0))
        m["kt_plant_ratio_nominal"] = tsc               # plant thrust / model thrust before any voltage term
        # the plant's true thrust ratio over HOLD (thrust_scale x (V_cell / V_nom)^k, the voltage term when set)
        if hold.any():
            vr_ = (np.clip(L["v_cell"][hold], 0.0, None) / vn) ** kv if (kv > 0.0 and vn > 0.0) else np.ones(int(hold.sum()))
            m["plant_thrust_ratio_hold"] = float(tsc * vr_.mean())
        else:
            m["plant_thrust_ratio_hold"] = None
        m["kt_hold_err_vs_plant"] = (None if m["kt_hold_mean"] is None or m["plant_thrust_ratio_hold"] is None
                                     or m["kt_learn_s"] <= 0.0 else m["kt_hold_mean"] - m["plant_thrust_ratio_hold"])
        m["kt_hold_err_vs_thrust_scale"] = m["kt_hold_err_vs_plant"]      # (name kept for the campaign columns)
        m["plant_thrust_to_weight"] = (tsc * float(vr.get("t_max_n_model", 0.0)) / (msc * float(vr.get("weight_n_model", 1.0)))
                                       if vr.get("t_max_n_model") else None)
        m["plant_thrust_to_weight_hold"] = (None if m["plant_thrust_ratio_hold"] is None or not m["plant_thrust_to_weight"]
                                            else m["plant_thrust_to_weight"] * m["plant_thrust_ratio_hold"] / tsc)
    # fix round (physics review, medium 6): the support signature's margin on the ground - the largest excess specific
    # force d / g between the first touchdown and LANDED (in LAND, on the surface) against the support threshold
    if tds and m["t_landed_s"] is not None and "vc_f_excess" in L:
        gnd = (t >= tds[0]["t"]) & (t < m["t_landed_s"]) & (phase == lp.PHASE_LAND)
        sg = (vr.get("vp") or {}).get("support_g")
        if gnd.any():
            m["f_excess_ground_max_g"] = float(L["vc_f_excess"][gnd].max() / G0)
            m["support_g"] = None if sg is None else float(sg)
            m["support_margin_ground_g"] = None if sg is None else m["f_excess_ground_max_g"] - float(sg)
    # fix round (physics review, medium 5): the body-axis (yaw) rate integrator against its limit in HOLD, the yaw error
    ag_ = vr.get("attitude_gains") or {}
    if hold.any() and "integrator" in L and ag_.get("rate_i_max_z"):
        iz = L["integrator"][hold, 2]
        lim = float(ag_["rate_i_max_z"])
        m["yaw_integ_limit"] = lim
        m["yaw_integ_hold_mean"] = float(iz.mean())
        m["yaw_integ_pinned_frac_hold"] = float(np.mean(np.abs(iz) >= 0.999 * lim))
        m["yaw_err_hold_mean_deg"] = float(np.degrees(L["att_err"][hold, 2].mean()))
        m["yaw_rate_rms_hold_deg_s"] = float(np.degrees(np.sqrt(np.mean(L["omega"][hold, 2] ** 2))))
    m["pushes"] = push_response(res) if sc.pushes else []
    pin = [r for r in m["pushes"] if r.get("in_flight")]
    m["n_pushes_in_flight"] = len(pin)
    m["push_peak_tilt_max_deg"] = max((r["peak_tilt_deg"] for r in pin), default=None)
    m["push_peak_dev_max_deg"] = max((r["peak_dev_deg"] for r in pin), default=None)
    rec = [r.get("recovery_s") for r in pin]
    m["push_recovery_max_s"] = (None if any(r is None for r in rec) else max(rec)) if rec else None
    m["push_peak_disp_max_m"] = max((r["peak_horiz_disp_m"] for r in pin), default=None)
    m["push_alt_drop_max_m"] = max((r["alt_drop_m"] for r in pin), default=None)
    m["push_min_height_m"] = min((r["min_height_m"] for r in pin), default=None)
    # ---- drift (no horizontal sensing: reported, not controlled) ----
    horiz = np.hypot(pe[:, 0] - pe[0, 0], pe[:, 1] - pe[0, 1])
    airborne = np.zeros(len(t), dtype=bool)
    if res.t_liftoff is not None:
        t_down = tds[-1]["t"] if tds else float(t[-1])
        airborne = (t >= res.t_liftoff) & (t <= t_down)
    vh = np.hypot(L["vel"][:, 0], L["vel"][:, 1])
    m["drift_max_m"] = float(horiz.max())
    m["drift_final_m"] = float(horiz[-1])
    m["vh_max_airborne_mps"] = float(vh[airborne].max()) if airborne.any() else 0.0
    m["vh_mean_hold_mps"] = float(vh[hold].mean()) if hold.any() else None
    m["wind_max_mps"] = float(np.hypot(L["wind"][:, 0], L["wind"][:, 1]).max())
    # ---- v0.8 GPS position hold ----
    if "hold_true" in L:
        m.update(gps_hold_metrics(res))
        m["gps_rejections"] = sum(1 for r in m["rejections"] if r["reason"] == lp.REASON_NAMES.get(lp.REJ_GPS))
    # ---- battery ----
    m["energy_Wh"] = float(L["energy_j"][-1] / 3600.0)
    m["v_cell_min"] = float(L["v_cell"].min())
    m["v_cell_final"] = float(L["v_cell"][-1])
    m["max_altitude_error_est_m"] = float(np.abs(L["h_est"][P] - h[P]).max()) if P.any() else None
    if res.link is not None:
        m.update(vtol_link_metrics(res))
    m.update(power_tether_metrics(res, m))
    return m


def landing_touchdown(tds: list) -> dict | None:
    """v0.9 fix round: the touchdown the landing criteria judge - the first one that is not a take-off re-contact (a
    POWERED contact in CLIMB: a plant at T ~ W re-touching the pad before it climbs away); the first one if all are."""
    if not tds:
        return None
    return next((d for d in tds if not (d.get("state") == int(lp.STATE_POWERED) and d.get("phase") == int(lp.PHASE_CLIMB))),
                tds[0])


CLIMB_OVERSHOOT_WINDOW_S = 3.0      # fix round: the take-off overshoot window after HOLD starts (climb_overshoot_transient_m)
POWER_LOSS_EVENTS = ("POWER tether unplugged", "POWER tether cut", "POWER supply tripped", "POWER supply latched off")


def power_tether_metrics(res: SimResult, m: dict[str, Any]) -> dict[str, Any]:
    """v0.9 (VTOL hardware round): the hardware configuration, the power source, the bus (V min, I max, the time in the
    supply's constant-current region, limited thrust), the first power loss (unplug / cut / supply trip or latch-off) and
    what followed (LAND after it, ground contacts after it and their speed), the tether tensions (catch line max = the
    arrest load), the buffer's state of charge and lowest cell voltage, the rotor gyro / play settings.  None where not
    applicable."""
    L = res.log
    vr = res.vtol or {}
    pv = vr.get("plant_v09") or {}
    t = L["t"]
    dt = float(t[1] - t[0]) if len(t) > 1 else 0.0
    P = L["state"] == lp.STATE_POWERED
    out: dict[str, Any] = {"hardware_config": pv.get("hardware_config"), "power_source": pv.get("power_source"),
                           "t_max_kgf_plant": pv.get("t_max_kgf_22v2"), "fan_power_model": pv.get("fan_power_model"),
                           "vane_play_deg": pv.get("vane_play_deg"), "rotor_inertia_kg_m2": pv.get("rotor_inertia_kg_m2"),
                           "max_wz_powered_deg_s": (float(np.degrees(np.abs(L["omega"][P, 2])).max()) if P.any() else None)}
    if "v_bus" in L:
        out["v_bus_min_powered"] = float(L["v_bus"][P].min()) if P.any() else None
        out["i_bus_max"] = float(L["i_bus"].max())
        cc = L["supply_state"] == 2
        out["supply_cc_s"] = float(cc.sum() * dt)
        out["thrust_limited_s"] = float((L["thrust_limited"] & P).sum() * dt)
        out["supply_a_max"] = float(L["supply_a"].max())
        bs = L["buffer_soc"]
        out["buffer_soc_min"] = float(np.nanmin(bs)) if np.isfinite(bs).any() else None
        pw = (pv.get("power") or {})
        out["buffer_cell_v_min"] = (None if out["buffer_soc_min"] is None else
                                    float((L["v_bus"][np.isfinite(bs) & (L["buffer_a"] > 0.5)]).min() / float(pw.get("buf_cells", 6)))
                                    if np.any(np.isfinite(bs) & (L["buffer_a"] > 0.5)) else None)
        out["buffer_a_max"] = float(L["buffer_a"].max())
        out["batt_soc_final"] = float(L["batt_soc"][-1]) if pv.get("power_source") == "battery" else None
    ev = [(te, e) for te, e in res.events if e.startswith("POWER")]
    out["power_events"] = [(round(float(te), 3), e) for te, e in ev]
    tl = next((float(te) for te, e in ev if any(e.startswith(k) for k in POWER_LOSS_EVENTS)), None)
    out["t_power_loss_s"] = tl
    out["supply_tripped"] = pv.get("supply_tripped")
    out["t_supply_trip_s"] = next((float(te) for te, e in ev if "latched off" in e or "tripped" in e), None)
    tland = m.get("t_land_s")
    out["land_after_power_loss_s"] = (None if tl is None or tland is None or tland < tl else tland - tl)
    out["landed_before_power_loss"] = bool(tl is not None and tland is not None and tland < tl)
    tds = vr.get("touchdowns", [])
    after = [d for d in tds if tl is not None and d["t"] >= tl]
    out["contacts_after_loss"] = len(after)
    out["contact_speed_max_after_loss_mps"] = max((d["speed"] for d in after), default=None)
    if tl is not None:
        k0 = int(np.searchsorted(t, tl))
        hh = L["h_true"][k0:] if "h_true" in L else np.zeros(0)
        out["h_at_power_loss_m"] = float(L["h_true"][min(k0, len(t) - 1)]) if "h_true" in L else None
        out["h_min_after_loss_m"] = float(hh.min()) if hh.size else None
        out["powered_at_power_loss"] = bool(L["state"][min(max(k0 - 1, 0), len(t) - 1)] == lp.STATE_POWERED)
    if "tether_catch_n" in L:
        out["tether_catch_max_n"] = float(L["tether_catch_n"].max())
        out["tether_cable_max_n"] = float(L["tether_cable_n"].max())
        out["tether_tension_max_n"] = float(L["tether_tension_n"].max())
        out["tether_catch_taut_s"] = float((L["tether_catch_n"] > 0.0).sum() * dt)
        out["tether_catch_taut_powered_s"] = float(((L["tether_catch_n"] > 0.0) & P).sum() * dt)
        # fix round (physics review HIGH 2): 'loaded' = above the SRL retract force; the line carrying the vehicle when
        # the supervisor confirmed LANDED (> 0.5 m g: hanging on the line, not standing on the ground) is judged apart
        ld = L["tether_catch_n"] > tt.CATCH_LOADED_N
        out["tether_catch_loaded_powered_s"] = float((ld & P).sum() * dt)
        tla = m.get("t_landed_s")
        if tla is not None:
            kl = min(int(np.searchsorted(t, tla)), len(t) - 1)
            cat = float(L["tether_catch_n"][max(kl - 1, 0)])
            mg = float((vr.get("plant") or {}).get("mass_kg") or 0.0) * G0
            out["catch_at_landed_n"] = cat
            out["landed_hanging"] = bool(mg > 0.0 and cat > 0.5 * mg)
        else:
            out["catch_at_landed_n"], out["landed_hanging"] = None, None
        if "tether_locked" in L:
            lk = L["tether_locked"].astype(bool)
            out["tether_locks"] = int(np.count_nonzero(lk[1:] & ~lk[:-1]) + (1 if lk.size and lk[0] else 0))
            out["tether_locked_powered_s"] = float((lk & P).sum() * dt)
        if "tether_loop_depth" in L:
            lim = pv.get("tether_loop_limit_m")
            out["tether_loop_depth_max_m"] = float(L["tether_loop_depth"][P].max()) if P.any() else 0.0
            if lim is not None:
                at = (L["tether_loop_depth"] > float(lim)) & (L["tether_loop_x"] < 0.1) & P
                out["cable_at_intake_s"] = float(at.sum() * dt)
    if "p_regen_w" in L:
        out["regen_j"], out["clamp_j"] = pv.get("regen_j"), pv.get("clamp_j")
        out["p_regen_max_w"] = float(L["p_regen_w"].max())
        out["regen_s"] = float((L["p_regen_w"] > 0.0).sum() * dt)
        out["v_bus_max"] = float(L["v_bus"].max())
    out["supply_ovp_trip"] = any("OVP" in e for _, e in ev)
    return out


SK_PUSH_EXCLUDE_S = 6.0             # v0.8 station keeping leaves out each push from its start to this long after its end
                                    # (the push response is judged on its own: excursion, return within 1 m)


def gps_hold_metrics(res: SimResult) -> dict[str, Any]:
    """v0.8 position hold (TRUE CG, horizontal; the truth hold point = the CG where the position loop captured its hold
    point): station keeping RMS / max / p95 over HOLD without the push windows (``sk_*``; ``sk_max_all_m`` with them),
    the NAV's own distance from its hold point (``nav_*``: the control error, the GPS wander cancels in it), the NAV
    position-estimate error against the truth, the touchdown against the hold point, the tilt setpoint against its cap,
    GPS availability and the position-loop modes, the heading error, resets and the position controller's requests."""
    L, sc = res.log, res.scenario
    vr = res.vtol or {}
    g = vr.get("gps") or {}
    t = L["t"]
    dt = float(t[1] - t[0]) if len(t) > 1 else 0.0
    P = L["state"] == lp.STATE_POWERED
    hold = L["phase"] == lp.PHASE_HOLD
    ht = L["hold_true"]
    have = np.isfinite(ht[:, 0])
    cg = L["pos"][:, :2]
    d = np.hypot(cg[:, 0] - ht[:, 0], cg[:, 1] - ht[:, 1])
    excl = np.zeros(len(t), dtype=bool)
    for pu in sc.pushes:
        excl |= (t >= pu.t0_s) & (t <= pu.t1_s + SK_PUSH_EXCLUDE_S)
    w = hold & have & ~excl
    wa = hold & have
    out: dict[str, Any] = {"gps_build": True, "gps_receiver": (g.get("receiver") or {}).get("receiver")}
    caps = g.get("hold_captures") or []
    out["n_hold_captures"] = len(caps)
    out["t_hold_capture_s"] = caps[0]["t"] if caps else None
    out["sk_window_s"] = float(w.sum() * dt)
    out["sk_rms_m"] = _rms(d[w]) if w.any() else None
    out["sk_max_m"] = float(d[w].max()) if w.any() else None
    out["sk_p95_m"] = float(np.percentile(d[w], 95)) if w.any() else None
    out["sk_max_all_m"] = float(d[wa].max()) if wa.any() else None
    pdist = L["pos_dist"]
    wn = w & np.isfinite(pdist)
    out["nav_rms_m"] = _rms(pdist[wn]) if wn.any() else None
    out["nav_max_m"] = float(pdist[wn].max()) if wn.any() else None
    pe = L["pos_est"][:, :2]
    we = P & np.isfinite(pe[:, 0])
    if we.any():
        en = np.hypot(pe[we, 0] - cg[we, 0], pe[we, 1] - cg[we, 1])
        out["nav_est_err_rms_m"] = _rms(en)
        out["nav_est_err_max_m"] = float(en.max())
    tds = vr.get("touchdowns") or []
    td = landing_touchdown(tds)
    out["touchdown_from_hold_m"] = None if td is None else td.get("from_hold_m")
    out["touchdown_vh_ground"] = None if td is None else td.get("vh")
    out["touchdown_pos_mode"] = None if td is None else td.get("pos_mode")
    # v0.8.1 touchdown criterion: the position loop's wind trim at the contact (integrator as a tilt), the trim / the
    # setpoint it commanded; the chain flown (v0.8 "fcu" / v0.8.1 "gns")
    out["touchdown_trim_deg"] = None if td is None else td.get("trim_deg")
    out["touchdown_trim_cmd_deg"] = None if td is None else td.get("trim_cmd_deg")
    out["touchdown_tilt_sp_deg"] = None if td is None else td.get("tilt_sp_deg")
    out["gps_chain"] = g.get("chain")
    out["glink_latency_ticks"] = g.get("glink_latency_ticks")
    pp = g.get("pos_params") or {}
    cap = pp.get("tilt_cap_deg")
    out["tilt_cap_deg"] = cap
    out["tilt_sp_max_deg"] = float(L["tilt_sp_deg"][P].max()) if P.any() else 0.0
    out["tilt_sp_mean_hold_deg"] = float(L["tilt_sp_deg"][w].mean()) if w.any() else None
    wt = hold & ~excl
    out["tilt_max_hold_deg"] = float(L["tilt_deg"][wt].max()) if wt.any() else None
    out["limited_frac_hold"] = float(L["pos_limited"][w].mean()) if w.any() else None
    out["guard_s"] = float((L["pos_guard"] & P).sum() * dt)
    out["gps_avail_frac_powered"] = float(L["gps_ok"][P].mean()) if P.any() else None
    for mode, name in ((1, "VEL"), (2, "POS"), (3, "DR"), (4, "ATT")):
        out[f"pos_mode_{name}_s"] = float(((L["pos_mode"] == mode) & P).sum() * dt)
    # heading error over the airborne part (lift-off to the first touchdown: the touchdown impact feeds bogus yaw windows
    # to the NAV until it is back in pad mode - reported separately as psi_err_max_powered_deg)
    air = np.zeros(len(t), dtype=bool)
    if res.t_liftoff is not None:
        air = P & (t >= res.t_liftoff) & (t <= (tds[0]["t"] if tds else float(t[-1])))
    pa_ = np.abs(L["psi_err_deg"][air])
    out["psi_err_max_deg"] = float(pa_.max()) if pa_.size else None
    pe_ = np.abs(L["psi_err_deg"][P]) if P.any() else np.zeros(0)
    out["psi_err_max_powered_deg"] = float(pe_.max()) if pe_.size else None
    out["psi_err_end_deg"] = float(L["psi_err_deg"][np.flatnonzero(P)[-1]]) if P.any() else None
    out["psi_err_start_deg"] = float(L["psi_err_deg"][np.flatnonzero(P)[0]]) if P.any() else None
    out["yaw_corr_end_deg"] = g.get("yaw_corr_deg")
    out["yaw_windows_used"] = g.get("yaw_windows_used")
    out["gps_resets"] = int(L["gps_resets"][-1]) if len(t) else 0
    ns = g.get("nav_stats") or {}
    out["gps_pos_resets"] = ns.get("pos_resets")
    out["gps_vel_resets"] = ns.get("vel_resets")
    rq = int(np.bitwise_or.reduce(L["pos_requests"].astype(int))) if len(t) else 0
    out["pos_requests"] = [n for b, n in ((1, "gps_lost"), (2, "fence"), (4, "gps_implausible"), (8, "authority")) if rq & b]
    out["vc_sat_high_hold_s"] = float(((L["vc_sat"] == 1) & hold).sum() * dt)
    out["rx_fix_frac"] = float(L["gps_rx_fix"].mean()) if len(t) else None
    cfgr = g.get("configurator") or {}
    out["gps_cfg_ok"] = cfgr.get("cfg_ok")
    out["gps_module_hw"] = cfgr.get("hw")
    return out


def vtol_link_metrics(res: SimResult) -> dict[str, Any]:
    """Ground link of a VTOL run: round trips, timeouts, every command with its latencies."""
    lk = res.link
    rows = command_latencies(res)
    out: dict[str, Any] = {"link_radio": lk["radio"], "link_loss": lk["loss"], "rtt": _stats(lk["rtt"]),
                           "polls": lk["ground_stats"]["polls"], "replies": lk["ground_stats"]["replies"],
                           "timeouts": lk["ground_stats"]["timeouts"], "link_stats": lk["link_stats"],
                           "commands": rows,
                           "cmd_decision_to_exec": _stats([r["decision_to_exec_s"] for r in rows if r["decision_to_exec_s"] is not None]),
                           "cmd_tx_count": _stats([float(r["n_tx"]) for r in rows if r["n_tx"]]),
                           "telem_frame_bytes": lk["budget"]["telem_frame_bytes"],
                           "reply_timeout_s": lk["budget"]["reply_timeout_s"]}
    for name in ("ON", "OFF", "UP", "DOWN", "KILL"):
        r = next((r for r in rows if r["cmd"] == name and r["t_exec"] is not None), None)
        out[f"{name.lower()}_decision_to_exec_s"] = None if r is None else r["decision_to_exec_s"]
    return out
