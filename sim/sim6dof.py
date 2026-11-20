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

Plant options written for the removed VTOL program and kept for the rocket (each a Scenario field, off at 0 -> the
v0.8.1 statements bit for bit): direct-drive vane play (``vane_play_deg``, params vane.direct_drive_play: the vane angle
is the play operator of the servo state, ``sim.edf.vane_play``; logged ``delta`` = the vanes, ``delta_servo`` = the servo
outputs), the EDF rotor (``rotor_inertia_kg_m2``: gyroscopic moment and spool-up reaction, ``sim.edf.RotorModel``), the
plant spread (mass, CG offset, thrust / voltage term, servo lag, vane trim, vane force / yaw lever).  The VTOL build
itself (FcuStack, vertical / position control, GPS, tether / power plant, pushes, VTOL aerodynamics and metrics) was
removed on 2026-11-18 (rocket only).

Invariant: the truth state never reaches the controller, the supervisor or the
ground; every number that is not in ``params.json`` is an explicit
``SensorConfig`` / ``SimOptions`` field (or a ``sim.link_model`` constant) marked
ASSUMED in its comment.
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from typing import Any

import numpy as np

try:
    from sim import link_proto as lp
    from sim import quat
    from sim.controller import AttitudeController, MixerPlant, gains_from_control
    from sim.edf import EDFModel, RotorModel, VaneModel, vane_play, vane_play_deg_from_params
    from sim.estimator import Mpu6050, PadAlignGyroEstimator, gyro_bias_process, up_error_rad, white_noise_sigma
    from sim.fc_timing import EPS_T, FCTiming, OutputLatch, frame_hz_from_timer, timer_of_channel
    from sim.ground_station_logic import GroundAssist, GroundParams
    from sim.link_model import COM_FORWARD_S, FC_REPLY_DELAY_S, LinkConfig, LinkModel, budget as link_budget, round_trip
    from sim.onboard import OnboardInputs, OnboardSupervisor, State, SupervisorParams, UplinkFrame
    from sim.state_machine import AltitudeFilter
    from sim.vehicle import G0, Vehicle
except ImportError:  # pragma: no cover - flat import when sim/ itself is on sys.path
    import link_proto as lp  # type: ignore
    import quat  # type: ignore
    from controller import AttitudeController, MixerPlant, gains_from_control  # type: ignore
    from edf import EDFModel, RotorModel, VaneModel, vane_play, vane_play_deg_from_params  # type: ignore
    from estimator import Mpu6050, PadAlignGyroEstimator, gyro_bias_process, up_error_rad, white_noise_sigma  # type: ignore
    from fc_timing import EPS_T, FCTiming, OutputLatch, frame_hz_from_timer, timer_of_channel  # type: ignore
    from ground_station_logic import GroundAssist, GroundParams  # type: ignore
    from link_model import COM_FORWARD_S, FC_REPLY_DELAY_S, LinkConfig, LinkModel, budget as link_budget, round_trip  # type: ignore
    from onboard import OnboardInputs, OnboardSupervisor, State, SupervisorParams, UplinkFrame  # type: ignore
    from state_machine import AltitudeFilter  # type: ignore
    from vehicle import G0, Vehicle  # type: ignore

ESTIMATORS = ("pad_gyro", "truth_noise", "mahony")
PAD_STATES = (State.SAFE, State.ARMED)


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
# 0.4 Hz; it feeds the ballistic AltitudeFilter, NAV firmware ALT_CROSSOVER_HZ).  History - the evidence comes from
# the removed VTOL program (2026-11-18), the rocket keeps the value.  Why it was needed: with 0.44 m of barometer noise the 0.5 Hz filter carries enough noise in v_z and
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
    # ---- campaign / study options.  Every field below is a no-op at its default: a Scenario that leaves them alone
    # runs the v0.6.3 ballistic statements bit-identically ----
    mode: str = "ballistic"                 # the mission program: "ballistic" only (the VTOL program was removed)
    link_outages: tuple = ()                # absolute [t0, t1) windows of total link loss
    # 2026-11-10 study hook: ((t_from_launch_s, radio), ...) - frames sent from launch + t on that link model (e.g. the
    # ESP link's latency stepping to the LoRa model's); the ground keeps the timeout / t_link default of ``radio``
    link_radio_steps: tuple = ()
    mass_scale: float = 1.0                 # plant mass and inertia x this (the controllers keep the nominal model)
    cg_offset_mm: tuple = (0.0, 0.0)        # plant CG offset from the thrust axis, body (x, y) [mm]
    thrust_scale: float = 1.0               # plant thrust x this (fan / proxy-table spread; controllers keep T_max)
    thrust_voltage_exp: float = 0.0         # 0: no voltage term (the v0.6.3 model); k > 0: T x (V_cell / V_nom_cell)^k
    servo_tau_scale: float = 1.0            # plant servo time constant x this
    vane_trim_err_deg: tuple = (0.0, 0.0, 0.0, 0.0)   # plant vane angle offsets (mechanical zero error) [deg]
    # ---- v0.9 plant options (written in the VTOL hardware round).  None -> the params / vehicle value; 0 -> the
    # element is off and the v0.8.1 plant statements run bit for bit (the regression pins set them so) ----
    vane_play_deg: Any = None               # direct-drive vane play, total width [deg], one value or 4; None -> params
                                            # vane.direct_drive_play (lost motion 0.65 + gear backlash 1 ASSUMED; 0 for the
                                            # pushrod linkage), 0 -> no play
    rotor_inertia_kg_m2: float | None = None    # EDF rotor inertia (gyroscopic moment + spool-up reaction); None -> params
                                                # edf.thrust_v09.rotor_inertia_kg_m2_ASSUMED (4.5e-5), 0 -> off
    # ---- v0.9 fix round (physics review; defaults = off, bit-identical) ----
    vane_force_scale: float = 1.0           # PLANT vane side force x (hub / motor wake: live span x0.76-0.85); the controllers'
                                            # mixer model keeps the params C_La S_v (the campaign samples 0.72-1.05)
    vane_yaw_lever_scale: float = 1.0       # PLANT vane yaw lever x (the force centroid moves out with a dead inner span)


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
        """``operator``: the ground operator (an object with ``step(gs, now)``); None -> ``ScriptedOperator``."""
        self.veh = vehicle
        self.sc = scenario
        self.mode = lp.check_mode(scenario.mode)
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

        # gains: params.control <- gains file/dict <- scenario overrides
        g = gains_from_control(p["control"], gains)
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
        # onboard supervisor + ground station + link (SPEC 8.8 / 10.3)
        self.sp = SupervisorParams.from_params(p, mode=self.mode, geofence_altitude_m=scenario.geofence_altitude_m,
                                               apogee_limit_m=scenario.apogee_limit_m,
                                               coast_drag_factor=scenario.coast_drag_factor,
                                               rate_abort_enabled=scenario.options.supervisor_enabled)
        lseed = scenario.seed if scenario.link_seed is None else scenario.link_seed
        self.sup = OnboardSupervisor(self.sp, seed=(lseed * 40503 + 12345) & 0xFFFF)   # firmware: ADC-noise seed
        self.link_cfg = LinkConfig.from_params(p, radio=scenario.radio, loss=scenario.packet_loss)
        self.link_rt = (round_trip(self.link_cfg))
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
        else:
            self.operator = ScriptedOperator(scenario)
        sens = scenario.sensors
        self.alt = AltitudeFilter(sens.altitude_crossover_hz, sens.altitude_zeta, sens.baro_latency_s)

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
        self.inv_tau_T = 1.0 / self.edf.tau_s
        self.inv_tau_s = 1.0 / self.servo_tau
        a = vehicle.aero
        self.a_ref, self.cd, self.cna = a.a_ref_m2, a.cd, a.cn_alpha_total
        rec = p["recovery"]
        self.chute_cda_full = float(rec["vehicle_chute_cd"]) * math.pi * (0.5 * float(rec["vehicle_chute_d_m"])) ** 2
        if not getattr(vehicle, "has_chute", True):
            self.chute_cda_full = 0.0       # a vehicle without a chute part: a release opens nothing
        # shock cord anchored at the top of the payload module after nose release
        z_attach = float(L.z_payload_top) * 1e-3
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
        # ---- plant spread (campaign options) - applied only when set, so the defaults keep every v0.6.3 number ----
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
        # ---- v0.9 plant options (written in the VTOL hardware round; every element off at 0 -> the v0.8.1 statements) ----
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
        self.t_fan_n = self.edf.t_max_n * self.thrust_scale        # the plant fan's T_max at 22.2 V
        self.rotor = RotorModel.from_params(p, self.t_fan_n, scenario.rotor_inertia_kg_m2)
        self.rot_on = self.rotor.i_rot_kg_m2 > 0.0
        self.rot_s_i = self.rotor.sign * self.rotor.i_rot_kg_m2
        # ---- v0.9 fix round (physics review): vane effectiveness ----
        vfs, vys = float(scenario.vane_force_scale), float(scenario.vane_yaw_lever_scale)
        if vfs != 1.0:
            self.sv_cla *= vfs
        if vys != 1.0:
            for cfg_ in (self.cfg_full, self.cfg_post):
                cfg_.b2 = [v * vys for v in cfg_.b2]

    def plant_thrust(self, u_esc: float, v_cell: float) -> float:
        """Steady thrust the plant produces for the decoded ESC command, with the plant spread (thrust_scale and the
        optional voltage term T x (V_cell / V_nom_cell)^k)."""
        tn = self.edf.thrust_steady(u_esc) * self.thrust_scale
        if self.thrust_vexp > 0.0:
            tn *= (max(v_cell, 0.0) / self.v_nom_cell) ** self.thrust_vexp
        return tn

    def plant_vanes(self, srv: list[float]) -> list[float]:
        """Decoded vane commands + the mechanical zero error of each vane (campaign)."""
        if self.vane_trim is None:
            return srv
        return [srv[i] + self.vane_trim[i] for i in range(4)]

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

    # ------------------------------------------------------------------ #
    # dynamics
    # ------------------------------------------------------------------ #
    def deriv(self, x: list[float], cfg: _MassCfg, t_cmd: float, dcmd: list[float], wind: tuple[float, float, float],
              chute_cda: float, frozen: int) -> list[float]:
        """Time derivative of the 18-state vector (pure-float inner loop)."""
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
        Mx = 0.0
        My = 0.0
        Mz = self.swirl_per_N * Tp
        if self.cg_off is not None:
            # campaign option: CG (dx, dy) off the duct axis; the thrust acts on the axis, r = (-dx, -dy, .) from the CG
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
        if V > 1e-9:
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
        est = PadAlignGyroEstimator(dt_c, sens.pad_avg_tau_s)
        pad_est = mode == "pad_gyro"                          # the estimator with the firmware's pad alignment
        q_est = q0.copy() if mode == "truth_noise" else quat.IDENTITY.copy()
        mahony_bias = np.zeros(3)
        f_static = R0.T @ np.array([0.0, 0.0, self.g])
        zero3 = np.zeros(3)
        for j in range(n_pre):
            gyr = imu.gyro(zero3, gyro_bias[j], gyro_noise[j])
            acc = imu.accel(f_static, accel_bias, accel_noise[j])
            if pad_est:
                q_est = est.update(gyr, acc, on_pad=True)
            else:
                q_est, mahony_bias = self._mahony(q_est, mahony_bias, gyr, acc, dt_c)
        w_dlpf = np.zeros(3)      # truth body rate through the MPU-6050 on-chip DLPF
        # ---- v0.9 plant logs / state ----
        self.vane_mem = [0.0, 0.0, 0.0, 0.0]
        L9: dict[str, np.ndarray] = {}
        if self.play_on:
            L9["delta_servo"] = np.zeros((n_max, 4))  # servo outputs (the vanes follow them through the play)
        if self.rot_on:
            L9["rotor_omega"] = np.zeros(n_max)
        tau_dlpf = sens.imu_dlpf_tau_s
        self.alt.reset(0.0)
        alt = self.alt                         # the NAV altitude filter of this build
        baro_queue: deque[float] = deque()
        v_prev = np.zeros(3)

        # ---- actuator interface: compute delay + PWM-frame ZOH + pulse decode ----
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
            accel = imu.accel(R.T @ (acc_w + np.array([0.0, 0.0, self.g])), accel_bias, accel_noise[j])
            h_true = x[2] - z_cg_ref
            h_baro: float | None = None
            if k % n_baro == 0:
                hb = h_true + baro_noise[k // n_baro]
                if baro_q > 0.0:
                    hb = math.floor(hb / baro_q + 0.5) * baro_q          # the sensor's output step (no random draw)
                baro_queue.append(hb)
                if len(baro_queue) > n_lat:
                    h_baro = baro_queue.popleft()
            # ---------------- attitude estimate (pad / flight from the NAV's lagged view of the state) ----------------
            pad_mode = (not sc.bypass_state_machine) and nav_states[0] in PAD_STATES
            if mode == "truth_noise":
                q_est = quat.multiply(q, quat.from_rotvec(att_noise[k]))
                rate_meas = gyro
            elif pad_est:
                q_est = est.update(gyro, accel, pad_mode)
                rate_meas = est.rate(gyro)
            else:
                q_est, mahony_bias = self._mahony(q_est, mahony_bias, gyro, accel, dt_c)
                rate_meas = gyro
            tilt_est = quat.tilt(q_est)
            # ---------------- altitude filter ----------------
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
                out = sup.update(inp)
                state = out.state
                nav_states.append(state)
                L["link_ok"][k] = bool(out.flags & lp.FLAG_LINK_OK)
                for e in out.events:
                    events.append((t, e))
                if out.reply_seq is not None:
                    # TELEM handed to the vehicle radio FC_REPLY_DELAY_S after the uplink frame arrived at the COM
                    t_reply = max(t, max(tr.t_arrival for tr in rx_tr) + FC_REPLY_DELAY_S) if rx_tr else t
                    link.send("down", lp.encode_telemetry(self.vehicle_id, out.reply_seq, out.telemetry), t_reply)
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
                prev_state = state
            if ejected and t_deploy is not None:
                chute_cda = self.chute_cda_full * (1.0 - math.exp(-(t - t_deploy) / opt.chute_inflate_tau_s))
            # ---------------- controller ----------------
            if active and not prev_active:
                self.ctrl.reset()
            prev_active = active
            if active:
                co = self.ctrl.update(q_est, rate_meas, u)
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
            wind = self.wind(t, t_launch)
            L["wind"][k] = wind
            L["v_cell"][k] = v_cell
            L["chute_cda"][k] = chute_cda
            L["control_active"][k] = active
            spread = self.thrust_scale != 1.0 or self.thrust_vexp > 0.0 or self.vane_trim is not None
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
                    x = self.rk4(x, hstep, args)
                    if self.play_on:
                        # v0.9 vane play: the actual vane angles follow the new servo outputs through the play
                        dmx = self.delta_mech
                        for i_ in range(4):
                            v_ = vane_play(self.vane_mem[i_], x[14 + i_], self.play_h[i_])
                            self.vane_mem[i_] = dmx if v_ > dmx else (-dmx if v_ < -dmx else v_)
                    r22 = 1.0 - 2.0 * (x[7] * x[7] + x[8] * x[8])
                    z_bottom = x[2] - cfg.z_cg * r22
                    if on_ground:
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
            if not sc.bypass_state_machine:
                if t_touchdown is not None and t - t_touchdown >= opt.stop_after_touchdown_s:
                    break

        log = {key: arr[:n_log] for key, arr in L.items()}
        final_state = self.sup.state if not sc.bypass_state_machine else State.POWERED
        link_rec = None
        if not sc.bypass_state_machine:
            for t_g, e in gs.events:
                events.append((t_g, "GS " + e))
            events.sort(key=lambda te: te[0])
            link_rec = {
                "radio": self.link_cfg.radio, "loss": self.link_cfg.loss,
                "budget": (link_budget(self.link_cfg)),
                "radio_steps": list(self.link_steps),
                "link_stats": dict(link.stats), "ground_stats": dict(gs.stats),
                "uplink_decoder": dict(self.uplink_decoder.stats), "downlink_decoder": dict(gs.decoder.stats),
                "rtt": [r for _, r in gs.rtt_all], "records": [vars(r).copy() | {"name": r.name} for r in gs.records],
                "exec_log": list(sup.exec_log), "assist_log": list(gs.assist_log), "dt": dt_c,
                "outages": list(link.outages), "t_launch": t_launch,
            }
        return SimResult(sc, log, events, self.gains, self.veh.mass_source, time.perf_counter() - t_wall,
                         t_liftoff, t_touchdown, touchdown_speed, final_state, nan_abort, touchdown_vz, link_rec)

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
    """Max tilt, settling time, apogee, drift, vane use, energy, saturation ..."""
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
