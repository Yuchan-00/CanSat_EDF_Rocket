"""Tests for the simulation package (SPEC §8-9, ATmega128 flight computer).

Covers: quaternion utilities, mixer round trip and saturation priority,
controller stabilising a linearised plant, the (superseded) v0.3 state machine,
thrust / vane formulas against hand calculations, the three mass-source
fallbacks, the hand-unrolled 6-DOF derivative against the numpy model, the
flight-computer output timing (compute delay, PWM-frame zero-order hold,
pulse decode, phase offsets, non-integer frame/tick ratios), the MPU-6050
model (quantisation, range, noise, bias process), the pad-alignment /
gyro-only estimator, the linear loop margins, and (v0.4) the LoRa link: frame
format, CRC, scaling and resynchronising decoder against independent code and
fixed byte vectors (writes ``sim/out/link_vectors.json``), the onboard supervisor
(every command in every state, every interlock, pad and flight variants), the
ground assist rules, the link-model latency against hand-computed time-on-air,
and short closed-loop scenario runs (link loss, assist off, nominal launch).
v0.5: the internal mass estimate's tab-mount solids (exact sector / cylinder
helpers, thin liner, per-screw lugs, holes as material removed) and its
agreement with the independent sizing estimator; per-preset report lookup
(``_<preset>`` files, other-preset reports skipped, stale params flagged).
v0.6.3: the measured sub-tick compute delay (4.71 ms; frame latch, no-latch path, mean wait), the NAV's
pad -> flight switch 2 ticks behind the supervisor (estimator and altitude hold, lag 1 vs 2), the on-vehicle
link timing (COM forwarding, reply 14 ms after the uplink, reply timeout on the COM-window allowance), the
assist CUT / RELEASE rules at a reply timeout, and tools/gen_gains_header.py writing to another drive.
v0.7: the link protocol's ON / OFF / UP / DOWN codes and TELEM v2 (TYPE 0x21, 31 bytes) against the independent encoder;
the supervisor and ground sections here test the BALLISTIC build (``mode='ballistic'``: params.json now defaults to
'vtol'); the VTOL build, the vertical controller and the ballistic bit-identity digest are in ``sim/test_vtol_ref.py``.

Run: ``.venv/Scripts/python.exe -m pytest sim -q -p no:cacheprovider`` from the project root.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import sys

import numpy as np
import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from dataclasses import replace  # noqa: E402

from sim import quat  # noqa: E402
from sim.controller import AttitudeController, LowPass, MixerPlant, VaneMixer, gains_from_control  # noqa: E402
from sim.edf import EDFModel, VaneModel  # noqa: E402
from sim.estimator import (Mpu6050, PadAlignGyroEstimator, gyro_bias_process, level_from_accel,  # noqa: E402
                           quantize_clip, up_error_rad, white_noise_sigma)
from sim.fc_timing import (EPS_T, FCTiming, OutputLatch, frame_hz_from_timer, mean_wait_after_available,  # noqa: E402
                           timer_of_channel)
from sim.loop_margins import axis_margins  # noqa: E402
from sim.sim6dof import FlightSim, Scenario, SensorConfig, SimOptions, compute_metrics  # noqa: E402
from sim.state_machine import (AltitudeFilter, MissionInputs, MissionParams, MissionStateMachine, State as MState,  # noqa: E402
                               ABORT_TO_DEPLOY_S, DEPLOY_TO_DESCENT_S, LANDING_STATIC_S, G0)
from sim.vehicle import PROJECT_ROOT, Vehicle, compute_aero, estimate_report, load_vehicle  # noqa: E402
from sim import link_model as lm  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim.ground_station_logic import GroundAssist, GroundParams  # noqa: E402
from sim.onboard import (OnboardInputs, OnboardSupervisor, State, SupervisorParams, UplinkFrame,  # noqa: E402
                         lfsr16_next, ticks_ge)

RNG = np.random.default_rng(1234)


@pytest.fixture(scope="module")
def veh() -> Vehicle:
    return load_vehicle(prefer="estimate")


def _plant(v: Vehicle) -> MixerPlant:
    p = v.params
    return MixerPlant(l_v_m=v.l_v_m, r_v_m=v.r_v_m, s_v_m2=v.s_v_m2, cl_alpha=p["vane"]["cl_alpha_per_rad"],
                      a_e_m2=v.a_e_m2, t_max_n=v.t_max_n, throttle_exp=p["edf"]["throttle_exponent"],
                      delta_max_rad=math.radians(p["vane"]["max_deflection_cmd_deg"]),
                      pwm_center_us=p["servo"]["pwm_center_us"], pwm_us_per_deg=p["servo"]["pwm_us_per_deg"])


# --------------------------------------------------------------------------- #
# quaternions
# --------------------------------------------------------------------------- #
def test_quat_multiply_matches_matrix_product():
    for _ in range(20):
        a, b = quat.random_unit(RNG), quat.random_unit(RNG)
        np.testing.assert_allclose(quat.to_matrix(quat.multiply(a, b)), quat.to_matrix(a) @ quat.to_matrix(b), atol=1e-12)


def test_quat_rotate_and_inverse_roundtrip():
    for _ in range(20):
        q, v = quat.random_unit(RNG), RNG.normal(size=3)
        np.testing.assert_allclose(quat.rotate_inverse(q, quat.rotate(q, v)), v, atol=1e-12)
        np.testing.assert_allclose(quat.multiply(q, quat.conjugate(q)), quat.IDENTITY, atol=1e-12)


def test_rotvec_roundtrip_and_small_angle():
    for _ in range(20):
        rv = RNG.normal(size=3)
        rv *= RNG.uniform(0.01, 3.0) / np.linalg.norm(rv)      # |rv| < pi so the shortest path is unique
        np.testing.assert_allclose(quat.to_rotvec(quat.from_rotvec(rv)), rv, atol=1e-10)
    tiny = np.array([1e-4, -2e-4, 3e-5])
    q = quat.from_rotvec(tiny)
    np.testing.assert_allclose(2.0 * q[1:], tiny, rtol=1e-6)   # e = 2 v for small angles
    # shortest path: negated quaternion gives the same rotation vector
    q = quat.from_rotvec(np.array([0.3, 0.2, -0.1]))
    np.testing.assert_allclose(quat.to_rotvec(-q), quat.to_rotvec(q), atol=1e-12)


def test_quat_integration_matches_derivative():
    q = quat.random_unit(RNG)
    w = np.array([0.3, -0.2, 0.5])
    dt = 1e-4
    q_euler = quat.normalize(q + quat.derivative(q, w) * dt)
    np.testing.assert_allclose(quat.integrate(q, w, dt), q_euler, atol=1e-9)
    # integrating a body-x rate rotates the body z axis toward -y (right-hand rule)
    q1 = quat.integrate(quat.IDENTITY, np.array([1.0, 0.0, 0.0]), 0.1)
    z_w = quat.rotate(q1, np.array([0.0, 0.0, 1.0]))
    assert z_w[1] < 0 and abs(z_w[1]) == pytest.approx(math.sin(0.1), abs=1e-9)


def test_euler_yaw_tilt_and_error():
    q = quat.from_euler(math.radians(5.0), math.radians(-7.0), math.radians(40.0))
    r, p, y = quat.to_euler(q)
    assert (math.degrees(r), math.degrees(p), math.degrees(y)) == pytest.approx((5.0, -7.0, 40.0), abs=1e-9)
    assert math.degrees(quat.yaw(q)) == pytest.approx(40.0, abs=1e-9)
    # tilt about x by 10 deg -> tilt() = 10 deg and tilt_vector = (-10 deg, 0, 0): the rotation back to upright
    q = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(10.0))
    assert math.degrees(quat.tilt(q)) == pytest.approx(10.0, abs=1e-9)
    np.testing.assert_allclose(np.degrees(quat.tilt_vector(q)), [-10.0, 0.0, 0.0], atol=1e-9)
    # error quaternion: rotation vector of error(q, q_sp) brings q onto q_sp
    q_sp = quat.from_yaw(math.radians(40.0))
    e = quat.to_rotvec(quat.error(q, q_sp))
    q_corr = quat.multiply(q, quat.from_rotvec(e))
    np.testing.assert_allclose(quat.to_matrix(q_corr), quat.to_matrix(q_sp), atol=1e-9)


# --------------------------------------------------------------------------- #
# thrust and vane formulas vs hand calculation (KAIST preset numbers)
# --------------------------------------------------------------------------- #
def test_edf_formulas_hand_calc(veh):
    edf = EDFModel.from_params(veh.params, veh.layout.duct_id)
    e = veh.params["edf"]
    d = veh.layout.duct_id * 1e-3
    a_e = math.pi / 4 * (d * d - (e["exit_hub_fraction"] * d) ** 2)
    assert edf.a_e_m2 == pytest.approx(a_e, rel=1e-12)
    T = e["thrust_nominal_N"]
    v_e = math.sqrt(T / (1.225 * a_e))
    assert edf.exhaust_velocity(T) == pytest.approx(v_e, rel=1e-9)
    assert edf.dynamic_pressure(T) == pytest.approx(0.5 * 1.225 * v_e ** 2, rel=1e-9)
    assert edf.dynamic_pressure(T) == pytest.approx(T / (2 * a_e), rel=1e-12)
    if (veh.layout.duct_id, e["exit_hub_fraction"], T) == (90.0, 0.45, 52.5):   # the SPEC v0.1 numbers
        assert edf.exhaust_velocity(T) == pytest.approx(91.9, abs=0.1)
        assert edf.dynamic_pressure(T) == pytest.approx(5174.0, abs=1.0)
    # thrust law and lag: 63.2 % of the step after one time constant
    assert edf.thrust_steady(0.5) == pytest.approx(T * 0.5 ** e["throttle_exponent"])
    assert edf.step(0.0, 1.0, edf.tau_s) == pytest.approx(T * (1 - math.exp(-1)), rel=1e-9)
    Tn = 0.0
    for _ in range(1000):
        Tn = edf.step(Tn, 1.0, edf.tau_s / 1000)
    assert Tn == pytest.approx(T * (1 - math.exp(-1)), rel=1e-6)
    # swirl: f P_nom / omega_max at u = 1, quadratic in u, sign from the rotation text
    omega_max = e["kv"] * e["voltage_nominal_V"] * 2 * math.pi / 60
    tau1 = e["residual_swirl_torque_fraction_ASSUMED"] * e["power_nominal_W"] / omega_max
    assert abs(edf.swirl_torque_from_throttle(1.0)) == pytest.approx(tau1, rel=1e-9)
    assert abs(edf.swirl_torque_from_throttle(0.5)) == pytest.approx(0.25 * tau1, rel=1e-9)
    assert edf.swirl_torque(edf.thrust_steady(0.7)) == pytest.approx(edf.swirl_torque_from_throttle(0.7), rel=1e-9)
    assert edf.swirl_sign == (1.0 if "CCW" not in e["rotation"].upper() else -1.0)
    assert edf.power(0.9) == pytest.approx(e["power_nominal_W"] * 0.729)


def test_vane_force_hand_calc(veh):
    edf = EDFModel.from_params(veh.params, veh.layout.duct_id)
    vane = VaneModel.from_params(veh.params, veh.layout)
    v = veh.params["vane"]
    q_e = edf.dynamic_pressure(veh.t_max_n)
    # hand calculation from the params: S_v = (r_out - r_in) * chord, F = q_e S_v C_La delta
    s_v = (v["span_r_out"] - v["span_r_in"]) * v["chord"] * 1e-6
    assert vane.s_v_m2 == pytest.approx(s_v, rel=1e-12)
    d20 = math.radians(20.0)
    F = vane.forces(q_e, np.array([d20]))[0]
    assert F == pytest.approx(q_e * s_v * v["cl_alpha_per_rad"] * d20, rel=1e-12)
    if (veh.layout.duct_id, veh.t_max_n, v["cl_alpha_per_rad"]) == (90.0, 52.5, 2.4):
        # q_e = 5174 Pa -> F = 5174 * S_v * 2.4 * 0.349 (= 5.06 N for the SPEC v0.1 36.5 x 32 mm vane)
        assert F == pytest.approx(5174.0 * s_v * 2.4 * 0.34907, rel=2e-3)
    # stall: flat beyond stall_deg, sign kept
    stall = math.radians(v["stall_deg"])
    assert vane.cl(stall + 0.3) == pytest.approx(vane.cl(stall))
    assert vane.cl(-(stall + 0.3)) == pytest.approx(-vane.cl(stall))
    # SPEC §8.4 derivation: vane 1 (+x) with +delta gives (l_v F, 0, r_v F); vane 3 gives (-l_v F, 0, r_v F)
    z_cg = veh.z_cg_m
    l_v = z_cg - vane.z_hinge_m
    Fb, Mb = vane.forces_moments(q_e, np.array([d20, 0.0, 0.0, 0.0]), z_cg)
    np.testing.assert_allclose(Fb, [0.0, F, 0.0], atol=1e-12)
    np.testing.assert_allclose(Mb, [l_v * F, 0.0, vane.r_v_m * F], atol=1e-12)
    _, M3 = vane.forces_moments(q_e, np.array([0.0, 0.0, d20, 0.0]), z_cg)
    np.testing.assert_allclose(M3, [-l_v * F, 0.0, vane.r_v_m * F], atol=1e-12)
    # pure roll: all four at +delta -> (0, 0, 4 r_v F) and zero net force
    Fb, Mb = vane.forces_moments(q_e, np.full(4, d20), z_cg)
    np.testing.assert_allclose(Fb, 0.0, atol=1e-12)
    np.testing.assert_allclose(Mb, [0.0, 0.0, 4 * vane.r_v_m * F], atol=1e-12)


# --------------------------------------------------------------------------- #
# mixer
# --------------------------------------------------------------------------- #
def test_mixer_roundtrip_and_physical_consistency(veh):
    plant = _plant(veh)
    mixer = VaneMixer(plant, veh.params["control"]["thrust_estimate_floor_fraction"], 1.0)
    edf = EDFModel.from_params(veh.params, veh.layout.duct_id)
    vane = VaneModel.from_params(veh.params, veh.layout)
    for u in (0.6, 0.85, 1.0):
        t_hat = mixer.thrust_estimate(u)
        gx, gy, gz = mixer.moment_gains(t_hat)
        for _ in range(50):
            M = np.array([RNG.uniform(-0.6, 0.6) * gx * plant.delta_max_rad, RNG.uniform(-0.6, 0.6) * gy * plant.delta_max_rad,
                          RNG.uniform(-0.3, 0.3) * gz * plant.delta_max_rad])
            delta, sat = mixer.deflections(M, t_hat)
            assert not sat
            np.testing.assert_allclose(mixer.moments(delta, t_hat), M, atol=1e-10)
            # the linear model equals the exact cross-product physics in the unstalled regime
            _, M_phys = vane.forces_moments(edf.dynamic_pressure(t_hat), delta, veh.z_cg_m)
            np.testing.assert_allclose(M_phys, M, atol=1e-9)
    # thrust floor
    assert mixer.thrust_estimate(0.0) == pytest.approx(plant.t_max_n * mixer.floor_fraction)
    assert mixer.thrust_floored(0.1) and not mixer.thrust_floored(0.9)


def test_mixer_saturation_priority(veh):
    plant = _plant(veh)
    mixer = VaneMixer(plant, 0.25, 1.0)
    t_hat = plant.t_max_n
    gx, gy, gz = mixer.moment_gains(t_hat)
    dmax = plant.delta_max_rad
    # 1) pitch within limits + excessive roll -> roll shrinks, pitch preserved
    M = np.array([0.6 * dmax * gx, 0.0, 5.0 * dmax * gz])
    delta, sat = mixer.deflections(M, t_hat)
    assert sat
    dx, dy, dz = mixer.split(delta)
    assert dx == pytest.approx(0.6 * dmax) and dy == pytest.approx(0.0)
    assert dz == pytest.approx(0.4 * dmax)
    assert np.abs(delta).max() == pytest.approx(dmax)
    # 2) excessive pitch -> roll dropped, (dx, dy) scaled uniformly so max|delta_i| = dmax
    M = np.array([3.0 * dmax * gx, -1.5 * dmax * gy, 0.5 * dmax * gz])
    delta, sat = mixer.deflections(M, t_hat)
    assert sat
    dx, dy, dz = mixer.split(delta)
    assert dz == pytest.approx(0.0)
    assert dx / dy == pytest.approx(3.0 / -1.5)
    assert np.abs(delta).max() == pytest.approx(dmax)
    # roll priority scale shapes the roll command
    mixer2 = VaneMixer(plant, 0.25, 0.5)
    d2, _ = mixer2.deflections(np.array([0.0, 0.0, 0.2 * dmax * gz]), t_hat)
    assert mixer2.split(d2)[2] == pytest.approx(0.1 * dmax)
    # PWM mapping
    pwm = mixer.pwm(np.array([math.radians(10.0), 0.0, -math.radians(10.0), 0.0]))
    c, k = plant.pwm_center_us, plant.pwm_us_per_deg
    np.testing.assert_allclose(pwm, [c + 10 * k, c, c - 10 * k, c])


def test_lowpass_alpha():
    dt = 1 / 400
    lp = LowPass(40.0, dt, n=1)
    assert lp.alpha == pytest.approx(dt / (1 / (2 * math.pi * 40.0) + dt))
    lp.update(np.array([1.0]))             # primes to the first sample
    assert lp.update(np.array([1.0]))[0] == pytest.approx(1.0)
    assert LowPass(0.0, dt).alpha == 1.0  # disabled


# --------------------------------------------------------------------------- #
# controller on a linearised plant
# --------------------------------------------------------------------------- #
def _run_linear_plant(veh: Vehicle, gains: dict, tilt0_deg: float = 10.0, t_end: float = 3.0) -> np.ndarray:
    """Rigid body with the linear vane model and a first-order servo, no noise."""
    ctrl = AttitudeController(veh.params["control"], _plant(veh), gains)
    dt = ctrl.dt
    u = veh.hover_throttle
    T = ctrl.mixer.thrust_model(u)
    I = np.diag(np.diag(veh.full.inertia_kg_m2))
    inv_I = np.linalg.inv(I)
    q = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(tilt0_deg))
    w = np.zeros(3)
    delta = np.zeros(4)
    tau_s = veh.params["servo"]["time_constant_s"]
    ctrl.set_yaw_reference(q)
    tilts = []
    for _ in range(int(t_end / dt)):
        out = ctrl.update(q, w, u)
        delta = delta + (out.delta_cmd - delta) * dt / tau_s
        M = ctrl.mixer.moments(delta, T)
        w = w + inv_I @ M * dt
        q = quat.integrate(q, w, dt)
        tilts.append(math.degrees(quat.tilt(q)))
    return np.array(tilts)


def test_controller_stabilises_linear_plant(veh):
    g = gains_from_control(veh.params["control"])
    tilts = _run_linear_plant(veh, g)
    assert tilts[-1] < 0.5, f"tilt after 3 s = {tilts[-1]:.2f} deg"
    assert tilts.max() < 10.5
    assert np.all(tilts[-200:] < 1.0)           # stays settled over the last 0.5 s
    # tuned gains (if the tuner has run) must also stabilise the plant
    gp = os.path.join(PROJECT_ROOT, "sim", "out", "gains.json")
    if os.path.exists(gp):
        with open(gp, "r", encoding="utf-8") as f:
            tuned = {k: v for k, v in json.load(f).items() if not k.startswith("_")}
        assert _run_linear_plant(veh, tuned)[-1] < 0.5


def test_controller_yaw_hold_and_integrator_freeze(veh):
    ctrl = AttitudeController(veh.params["control"], _plant(veh))
    q = quat.from_yaw(math.radians(30.0))
    ctrl.set_yaw_reference(q)
    out = ctrl.update(q, np.zeros(3), 0.9)
    np.testing.assert_allclose(out.att_error, 0.0, atol=1e-12)
    np.testing.assert_allclose(out.delta_cmd, 0.0, atol=1e-12)
    # yaw error of +10 deg -> positive z rate setpoint, clamped at rate_sp_max_z
    q2 = quat.from_yaw(math.radians(20.0))
    out = ctrl.update(q2, np.zeros(3), 0.9)
    assert out.rate_sp[2] > 0
    assert out.rate_sp[2] == pytest.approx(min(ctrl.g["att_kp_z"] * math.radians(10.0), ctrl.g["rate_sp_max_z_rad_s"]))
    # integrator frozen while the thrust estimate is floored (u = 0)
    ctrl.reset()
    for _ in range(10):
        out = ctrl.update(q2, np.zeros(3), 0.0)
    np.testing.assert_allclose(out.integrator, 0.0)
    for _ in range(10):
        out = ctrl.update(q2, np.zeros(3), 0.9)
    assert out.integrator[2] > 0


# --------------------------------------------------------------------------- #
# v0.3 mission state machine - SUPERSEDED by sim.onboard (SPEC 8.8 v0.4).  The class and these tests stay
# only while tools/crosscheck_controller.py and tools/gen_firmware_params.py still import it.
# --------------------------------------------------------------------------- #
def _mp(veh: Vehicle, **kw) -> MissionParams:
    return MissionParams.from_params(veh.params, **kw)


def _inp(t: float, **kw) -> MissionInputs:
    base = dict(t=t, h=0.0, v_z=0.0, tilt_deg=0.0, accel_norm=G0, v_cell=4.1)
    base.update(kw)
    return MissionInputs(**base)


def test_state_machine_nominal_sequence(veh):
    mp = _mp(veh)
    sm = MissionStateMachine(mp)
    assert sm.update(_inp(0.0)).state == MState.IDLE
    assert sm.update(_inp(0.1, arm_cmd=True, tilt_deg=mp.arm_tilt_max_deg + 1)).state == MState.IDLE   # too tilted
    assert sm.update(_inp(0.2, arm_cmd=True, v_cell=mp.min_cell_v - 0.1)).state == MState.IDLE          # low cell
    assert sm.update(_inp(0.3, arm_cmd=True)).state == MState.ARMED
    out = sm.update(_inp(1.0, launch_cmd=True))
    assert out.state == MState.LAUNCH and out.control_active and not out.vanes_centre
    out = sm.update(_inp(1.0 + 0.5 * mp.throttle_ramp_s))
    assert out.state == MState.LAUNCH and out.throttle == pytest.approx(0.5 * mp.climb_throttle)
    out = sm.update(_inp(1.0 + mp.throttle_ramp_s))
    assert out.state == MState.ASCENT and out.throttle == pytest.approx(mp.climb_throttle)
    # cut-off rule: h + f v^2/(2g) >= target, exactly at the boundary
    v = 15.0
    h_cut = mp.target_altitude_m - mp.coast_drag_factor * v * v / (2 * G0)
    assert sm.update(_inp(5.0, h=h_cut - 0.01, v_z=v)).state == MState.ASCENT
    out = sm.update(_inp(5.1, h=h_cut + 0.01, v_z=v))
    assert out.state == MState.COAST and out.throttle == 0.0
    # control stays on for coast_control_hold_s after cut-off (swirl decay), then the vanes centre
    assert out.control_active == (mp.coast_control_hold_s > 0.0) and out.vanes_centre != out.control_active
    out = sm.update(_inp(5.1 + mp.coast_control_hold_s + 0.01, h=h_cut + 0.5, v_z=v - 0.5))
    assert out.state == MState.COAST and out.vanes_centre and not out.control_active
    # apogee detection needs v_z <= threshold for apogee_confirm_s continuously
    t = 5.1
    for _ in range(5):
        t += 0.1
        assert sm.update(_inp(t, h=150.0, v_z=1.0)).state == MState.COAST
    t0 = t
    while sm.state == MState.COAST:
        t += 0.02
        sm.update(_inp(t, h=150.0, v_z=-0.1))
    assert t - t0 == pytest.approx(mp.apogee_confirm_s, abs=0.03)
    out = sm.update(_inp(t, h=150.0))
    assert out.state == MState.DEPLOY and out.release
    assert sm.update(_inp(t + DEPLOY_TO_DESCENT_S - 0.01, h=149.0)).state == MState.DEPLOY
    assert sm.update(_inp(t + DEPLOY_TO_DESCENT_S + 0.01, h=148.0)).state == MState.DESCENT
    # descent: impact spike then 3 s static baro -> LANDED
    t += 20.0
    sm.update(_inp(t, h=0.3, accel_norm=5 * G0))
    for i in range(int(LANDING_STATIC_S / 0.05) + 2):
        out = sm.update(_inp(t + 0.05 * (i + 1), h=0.3 + 0.1 * math.sin(i)))
    assert out.state == MState.LANDED and not out.control_active
    names = [e for _, e in sm.events]
    assert [n.split(":")[0] for n in names] == [
        "IDLE -> ARMED", "ARMED -> LAUNCH", "LAUNCH -> ASCENT", "ASCENT -> COAST", "COAST -> DEPLOY",
        "DEPLOY -> DESCENT", "DESCENT -> LANDED"]


def test_state_machine_coast_timeout(veh):
    mp = _mp(veh)
    sm = MissionStateMachine(mp)
    sm.update(_inp(0.0, arm_cmd=True))
    sm.update(_inp(1.0, launch_cmd=True))
    sm.update(_inp(1.0 + mp.throttle_ramp_s))
    sm.update(_inp(2.0, h=mp.target_altitude_m, v_z=0.0))
    assert sm.state == MState.COAST
    t = 2.0
    while sm.state == MState.COAST:
        t += 0.05
        sm.update(_inp(t, h=200.0, v_z=5.0))     # still climbing: apogee never detected
    assert t - 2.0 == pytest.approx(mp.coast_timeout_s, abs=0.06)
    assert sm.state == MState.DEPLOY


@pytest.mark.parametrize("kw, needle", [
    (dict(tilt_deg=50.0), "tilt"),
    (dict(h=400.0), "geofence"),
    (dict(v_cell=3.0), "V_cell"),
    (dict(kill_cmd=True), "RC kill"),
])
def test_state_machine_abort_paths(veh, kw, needle):
    mp = _mp(veh)
    sm = MissionStateMachine(mp)
    sm.update(_inp(0.0, arm_cmd=True))
    sm.update(_inp(1.0, launch_cmd=True))
    out = sm.update(_inp(1.2, **kw))
    assert out.state == MState.ABORT and out.throttle == 0.0 and not out.control_active
    assert needle in out.abort_reason
    assert sm.update(_inp(1.2 + ABORT_TO_DEPLOY_S - 0.01)).state == MState.ABORT
    out = sm.update(_inp(1.2 + ABORT_TO_DEPLOY_S + 0.01))
    assert out.state == MState.DEPLOY and out.release


def test_state_machine_no_abort_outside_powered_states(veh):
    mp = _mp(veh)
    sm = MissionStateMachine(mp)
    assert sm.update(_inp(0.0, arm_cmd=True, kill_cmd=True)).state == MState.ARMED   # ARMED is unpowered
    sm.update(_inp(0.5, disarm_cmd=True))
    assert sm.state == MState.IDLE


def test_altitude_filter_tracks_climb_with_latency():
    dt, baro_dt, lat = 1 / 400, 1 / 50, 0.1
    af = AltitudeFilter(0.5, 0.7, baro_latency_s=lat)
    af.reset(0.0)
    v, a = 12.0, 0.4
    t_last_b = -1.0
    for k in range(int(6.0 / dt)):
        t = k * dt
        h_true = v * t + 0.5 * a * t * t
        af.predict(a, dt)
        if t - t_last_b >= baro_dt - 1e-9 and t >= lat:
            t_last_b = t
            tl = t - lat
            af.correct(v * tl + 0.5 * a * tl * tl, t)
    h_true = v * 6.0 + 0.5 * a * 36.0
    assert af.h == pytest.approx(h_true, abs=0.6)
    assert af.v == pytest.approx(v + a * 6.0, abs=0.3)


# --------------------------------------------------------------------------- #
# mass sources
# --------------------------------------------------------------------------- #
def test_internal_estimate_sanity(veh):
    rep = estimate_report(veh.layout, veh.params)
    tot = rep["totals"]
    p = veh.params
    L = veh.layout
    if getattr(L, "direct_drive", False):
        # v0.9 direct drive: 4 trimmed stock horns + the cover / coupler screws lump (no arms, pushrods, clevises)
        linkage = 4 * p["vane_drive"]["horn"]["mass_g"] + p["vane_drive"]["hardware_g_ASSUMED"]
        if getattr(L, "dd_bushing", False):       # v0.9 fix round: + 4 brass hinge bushings
            linkage += L.vane_count * L.dd_bush_mass_g
    else:
        linkage = 8.0 + p["hardware_mass_g"]["pushrods_horns_clevises"]       # 4 arms + 4 pushrods at 1 g + the lump
    non_printed = (p["edf"]["mass_g"] + p["battery"]["mass_g"] + p["electronics"]["esc"]["mass_g"] + p["electronics"]["nav"]["mass_g"]
                   + p["electronics"]["misc"]["mass_g"] + 5 * p["servo"]["mass_g"] + linkage + p["payload"]["mass_g"]
                   + 0.5 * p["hardware_mass_g"]["spring_piston_hardware"] + p["recovery"]["chute_mass_g"]
                   + p["recovery"]["lines_swivel_mass_g"] + p["recovery"]["shock_cord_mass_g"]
                   + p["hardware_mass_g"]["joint_screws_inserts"]
                   + (L.ml_mass_g if getattr(L, "has_motor_leads", False) else 0.0)               # v0.9 motor leads
                   + ((L.gps_board_mass_g + L.gps_gp_mass_g + L.gps_lead_mass_g)                 # v0.8 GPS pod contents
                      if getattr(L, "has_gps_pod", False) else 0.0)
                   + (L.gns_mass_g if getattr(L, "has_gns", False) else 0.0)                     # v0.8.1 GNS board
                   + p["electronics"].get("com", {}).get("mass_g", 0.0)                          # v0.6 COM board
                   + p["electronics"].get("radio_log", {}).get("mass_g", 0.0)                    # v0.6.2 radio/log box
                   + p["electronics"].get("fcu", {}).get("mass_g", 0.0)                          # v0.6.3 FCU board
                   + (veh.layout.sl_lead_mass_g if getattr(veh.layout, "servo_leads", False) else 0.0))   # v0.6 servo leads
    assert tot["mass_g"] == pytest.approx(non_printed + tot["printed_mass_g"], rel=1e-9)
    assert 800 < tot["printed_mass_g"] < 2500
    assert 300 < tot["com_mm"][2] < 600
    I = np.array(tot["inertia_about_com_kg_m2"])
    np.testing.assert_allclose(I, I.T, atol=1e-12)
    assert np.all(np.linalg.eigvalsh(I) > 0)
    assert I[0, 0] == pytest.approx(I[1, 1], rel=0.05)
    assert I[2, 2] < 0.1 * I[0, 0]
    ejected = sum(rep["parts"][n]["mass_g"] for n in ("cansat_payload", "nose_cone", "chute_pack"))
    assert tot["post_ejection"]["mass_g"] == pytest.approx(tot["mass_g"] - ejected, rel=1e-9)
    assert tot["descent_mass_g"] == pytest.approx(tot["post_ejection"]["mass_g"] + rep["parts"]["chute_pack"]["mass_g"])
    # placeholders report density = mass / volume
    bat = rep["parts"]["battery"]
    assert bat["density_g_cm3"] == pytest.approx(bat["mass_g"] / (bat["volume_mm3"] / 1000.0))
    for n in ("edf_unit", "battery", "esc", "nav", "fcu", "cansat_payload"):
        assert rep["parts"][n]["kind"] == "placeholder"


def _sizing_estimator():
    """sizing/structure_estimate.py: an independent analytic estimate to cross-check the vehicle model."""
    import importlib.util
    path = os.path.join(_ROOT, "sizing", "structure_estimate.py")
    spec = importlib.util.spec_from_file_location("structure_estimate_xcheck", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_vehicle_solid_helpers_exact():
    from sim import vehicle as vm
    rho = 1240.0
    tube = vm._tube(50.0, 40.0, 0.0, 10.0, rho)
    full = vm._sector(40.0, 50.0, 0.0, 360.0, (0.0, 10.0), rho)
    assert full.m == pytest.approx(tube.m, rel=1e-12)
    np.testing.assert_allclose(full.c, tube.c, atol=1e-12)
    np.testing.assert_allclose(full.I, tube.I, rtol=1e-9, atol=1e-15)
    q = vm._sector(40.0, 50.0, 30.0, 120.0, (0.0, 10.0), rho)     # rotated quarter: centroid on the 75 deg bisector
    c = 4.0 / (3.0 * math.pi) * (50.0**3 - 40.0**3) / (50.0**2 - 40.0**2) * math.sqrt(2.0) * 1e-3
    np.testing.assert_allclose(q.c[:2], c * np.array([math.cos(math.radians(75)), math.sin(math.radians(75))]), rtol=1e-9)
    assert np.allclose(q.I, q.I.T) and np.all(np.linalg.eigvalsh(q.I) > 0)
    cyl = vm._cylinder_between(20.0, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 10.0]), rho)
    np.testing.assert_allclose(cyl.I, vm._tube(10.0, 0.0, 0.0, 10.0, rho).I, rtol=1e-12)
    area, _ = vm._box_annulus_area(0.0, 30.0, 30.0, 0.0, 30.0, n=4000)
    assert area == pytest.approx(math.pi * 900.0 / 2, rel=1e-4)


def _solid_mass_model(p: dict) -> dict:
    """A copy of params with printer.mass_model 'solid': the geometry tests compare solids at printer.density_g_cm3 (the
    v0.9 slicer / class effective densities are checked by test_internal_estimate_print_mass_model)."""
    q = json.loads(json.dumps(p))
    q["printer"]["mass_model"] = {"mode": "solid"}
    return q


def test_internal_estimate_print_mass_model():
    """v0.9 (printer.mass_model 'slicer_part_g'): every printed part of the sim estimate is its solid estimate scaled to
    the effective density of its slicer-table entry (part_g / solid_volume_mm3) or, without one, its print class's
    effective_density_fallback_g_cm3; volumes, centroids and the inertia ratio are kept; mode 'solid' changes nothing."""
    from sim import vehicle as vm
    p = vm.layout_mod.load_params(os.path.join(PROJECT_ROOT, "params.json"))
    mm = p["printer"].get("mass_model") or {}
    if mm.get("mode") != "slicer_part_g":
        pytest.skip("default preset keeps the solid mass model")
    L = vm.Layout(p)
    parts, solid = vm.estimate_parts(L, p), vm.estimate_parts(L, _solid_mass_model(p))
    with open(os.path.join(PROJECT_ROOT, mm["table"]), encoding="utf-8") as f:
        table = json.load(f)["parts"]
    checked = 0
    for name, pt in parts.items():
        if pt.kind != "printed":
            assert pt.mass_g == solid[name].mass_g
            continue
        base = vm._printed_base(name)
        ent = table.get(base)
        rho = (ent["part_g"] * 1000.0 / ent["solid_volume_mm3"] if ent
               else p["printer"]["print_classes"][p["printer"]["part_class"][base]]["effective_density_fallback_g_cm3"])
        assert pt.mass_g == pytest.approx(solid[name].mass_g * rho / p["printer"]["density_g_cm3"], rel=1e-12), name
        assert pt.volume_mm3 == solid[name].volume_mm3 and np.allclose(pt.com_mm, solid[name].com_mm)
        np.testing.assert_allclose(np.asarray(pt.inertia_about_com_kg_m2) * solid[name].mass_g,
                                   np.asarray(solid[name].inertia_about_com_kg_m2) * pt.mass_g, rtol=1e-9, atol=1e-15)
        checked += 1
    assert checked >= 10


@pytest.mark.parametrize("preset", [None, "KAIST_90mm_12S"])
def test_internal_estimate_matches_sizing_estimator(preset):
    """Two independent analytic models of the printed modules (incl. the v0.5 tab mount) and the edf_unit agree."""
    from sim import vehicle as vm
    se = _sizing_estimator()
    p = _solid_mass_model(vm.layout_mod.load_params(os.path.join(PROJECT_ROOT, "params.json"), preset))
    L = vm.Layout(p)
    rep = estimate_report(L, p)["parts"]
    sest = se.StructureEstimator(L, p)
    est = sest.estimate_all()
    for name, solids in (("propulsion_module", sest.propulsion_module()), ("intake_module", sest.intake_module())):
        assert rep[name]["mass_g"] == pytest.approx(est[name].mass_solid_g, rel=0.015), name
        z_solid = sum(s.volume_mm3 * s.com_mm[2] for s in solids) / sum(s.volume_mm3 for s in solids)   # fill = 1
        assert rep[name]["com_mm"][2] == pytest.approx(z_solid, abs=1.0), name
    edf_v, edf_s = rep["edf_unit"], se.placeholder_parts(L, p)["edf_unit"]
    assert edf_v["volume_mm3"] == pytest.approx(edf_s.volume_mm3, rel=1e-9)
    np.testing.assert_allclose(edf_v["com_mm"], edf_s.com_mm, atol=1e-9)
    np.testing.assert_allclose(np.diag(edf_v["inertia_about_com_kg_m2"]), np.diag(edf_s.inertia_about_com_kg_m2), rtol=1e-9)


def test_internal_estimate_tab_mount_geometry(veh):
    """v0.5 tabs: edf_unit gets motor + tab plates; the propulsion module gets the cylinder, liner and bosses.  wire09c: this
    checks the SCREW tab mount (tabs at 90 / 270, lugs); the default params use the clock mount (the EDF drawn at a computed
    keyway seat; modelled by the estimate since the simfix round 2026-11-04, see
    test_internal_estimate_propulsion_volume_follows_the_solid), so this runs on the same params with
    propulsion_module.tab_mount 'screws'."""
    import copy
    from sim import vehicle as vm
    p = copy.deepcopy(veh.params)
    if "propulsion_module" in p:
        p["propulsion_module"]["tab_mount"] = "screws"
    L = vm.Layout(p)
    if not L.tabs:
        pytest.skip("default preset is not a tab mount")
    parts = vm.estimate_parts(L, p)
    e = p["edf"]
    bb = parts["edf_unit"].bbox_mm
    assert bb[0, 2] == pytest.approx(L.z_edf_housing_bottom - e["motor_protrusion_rear"])
    assert bb[1, 2] == pytest.approx(L.z_edf_lip_top)
    # v0.5d: tabs at 90 / 270 deg -> they set the y extent; along x the bell lip is the widest part
    assert L.tab_thetas == [90.0, 270.0]
    assert max(abs(bb[0, 1]), bb[1, 1]) == pytest.approx(max(L.tab_r_out, e["lip_od"] / 2))
    assert max(abs(bb[0, 0]), bb[1, 0]) == pytest.approx(max(L.tab_t / 2, e["lip_od"] / 2))
    p_lc = json.loads(json.dumps(p))
    p_lc["edf"]["mount"] = "lip_clamp"
    lc = vm.estimate_parts(vm.Layout(p_lc), p_lc)
    rho = p["printer"]["density_g_cm3"] * 1e3
    r_skin = L.body_r - L.cone_wall
    lugs = sum(vm._sector(L.block_r_in, r_skin, a0, a1, zr, rho).m for a0, a1 in vm._lug_sectors(L) for zr in L.boss_z_ranges)
    assert parts["propulsion_module"].mass_g > lc["propulsion_module"].mass_g + 1e3 * lugs   # cylinder + liner + lugs - cuts
    assert parts["propulsion_module"].bbox_mm[1, 2] == pytest.approx(L.z_prop_top)
    assert parts["edf_unit"].mass_g == lc["edf_unit"].mass_g == e["mass_g"]
    rep = estimate_report(L, p)
    assert any("tab mount" in w for w in rep["warnings"])
    assert not any("screw holes" in w and "ignored" in w and "tab-screw holes are modelled" not in w for w in rep["warnings"])
    rep_lc = estimate_report(vm.Layout(p_lc), p_lc)        # lip_clamp keeps the v0.4 text verbatim
    assert rep_lc["warnings"] == ["simplified-solid estimate: pods include the 45 deg support wedge; "
                                  "screw holes, fillets and the vane D-flat are ignored"]


def test_internal_estimate_propulsion_volume_follows_the_solid():
    """simfix 2026-11-04: the estimate's printed mass is its own volume x the slicer-table density (part grams / volume of
    the CAD solid that was sliced), so it must model the mount the solid has.  With the wire09c clock mount modelled
    (keyways round the circle, floor pads, column guide walls, blister + keel, lead passage; no lugs / tab screws / 22 mm
    slot) the propulsion module's signed volume is +2.3 % over the CadQuery solid (M 2026-11-04: 327,653 vs 320,359 mm^3;
    the deck and the servo-lead cutters are not modelled - the same +2.3 % as the screw-mount rounds pre_wire and
    wire_round2).  The screw-mount estimate on the clock solid was +8.1 % (+17.7 g propulsion module, E), which cost
    test_sim_assist_off_limiter_cuts its I4 cut.  The bound sits between the two."""
    from sim import vehicle as vm
    path = os.path.join(PROJECT_ROOT, "cad", "exports", "cadquery_report.json")
    if not os.path.exists(path):
        pytest.skip("no CadQuery report")
    with open(path, encoding="utf-8") as f:
        cad = json.load(f)["parts"]["propulsion_module"]
    p = vm.layout_mod.load_params(os.path.join(PROJECT_ROOT, "params.json"))
    L = vm.Layout(p)
    pt = vm.estimate_parts(L, p)["propulsion_module"]
    v_est = pt.mass_g / pt.density_g_cm3 * 1e3                       # signed volume (cuts subtracted), mm^3
    assert abs(v_est / cad["volume_mm3"] - 1.0) <= 0.04, (v_est, cad["volume_mm3"])
    if getattr(L, "clock_mount", False):
        rho = p["printer"]["density_g_cm3"] * 1e3
        sest = _sizing_estimator().StructureEstimator(L, p)
        assert sum(s.m for s in vm._clock_mount_solids(L, rho)) / rho * 1e9 == pytest.approx(
            sum(s.volume_mm3 for s in sest.clock_mount_solids()), rel=1e-9)


def test_internal_estimate_tab_mount_contract_v05b(veh):
    """SPEC §3.7 / §5.1 (v0.5 review fixes): thin liner, per-screw lugs, holes as material removed (counterbore
    to t_cbore_end, nothing past the skin), two-box wire slot, channel + shelf notch; hand volumes."""
    from sim import vehicle as vm
    L, p = veh.layout, veh.params
    if not L.tabs:
        pytest.skip("default preset is not a tab mount")
    rho = p["printer"]["density_g_cm3"] * 1e3
    vol = lambda sols: sum(s.m for s in sols) / rho * 1e9   # noqa: E731  signed mm^3
    # liner = tube + ring outside the tube / inside the sleeve bore
    liner = [vm._tube(b, a, za, zb, rho) for a, b, za, zb in vm._liner_layers(L)]
    assert vol(liner) == pytest.approx(math.pi * ((L.liner_r_out**2 - L.duct_r**2) * (L.z_liner_top - L.z_edf_shoulder)
                                                  + (L.edf_bore_r**2 - L.liner_r_out**2) * (L.z_liner_top - L.z_liner_ring_bottom)))
    # holes: clearance / tap are full cylinders, the counterbore stops at the outer skin
    holes = vm._tab_screw_hole_cuts(L, rho)
    n = L.tab_count * len(L.tab_screw_zs)
    R, rs, rc = L.body_r, L.tab_screw_r, L.tab_screw_d_cbore / 2
    f, f2 = math.sqrt(R * R - rs * rs), -R * R / (R * R - rs * rs) ** 1.5
    expect = n * (math.pi / 4 * (L.tab_screw_d_clear**2 * (L.tab_screw_t_head - L.tab_screw_t_keyway)
                                 + L.tab_screw_d_tap**2 * (-L.tab_screw_t_keyway - L.tab_screw_t_tap_end))
                  + math.pi * rc * rc * (f + f2 * rc * rc / 8 - L.tab_screw_t_head))
    assert -vol(holes) == pytest.approx(expect, rel=1e-3)
    for s in holes:                                   # each cut sits on its tab's side, between bore and skin
        r = math.hypot(s.c[0], s.c[1]) * 1e3
        assert L.edf_bore_r < r < L.body_r and min(abs(s.c[2] * 1e3 - z) for z in L.tab_screw_zs) < 1e-9
    # the sim and sizing estimators cut the same material for the slot, the channel and the notch
    se = _sizing_estimator()
    sest = se.StructureEstimator(L, p)
    tab_v = vol(vm._tab_mount_solids(L, rho))
    assert tab_v == pytest.approx(sum(s.volume_mm3 for s in sest.tab_mount_solids()), rel=1e-6)
    wire_v = vol(vm._wire_channel_solids(L, rho))
    assert wire_v == pytest.approx(sum(s.volume_mm3 for s in sest.wire_channel_solids()), rel=1e-6)
    notch = vm._radial_cut([(L.shelf_id / 2, L.body_r, L.z_lip_pocket_top, L.z_shelf_top)], L.wire_notch_r0,
                           L.wire_ch_r0, L.wire_ch_w / 2, *vm._wire_notch_z(L), L.wire_theta, rho)
    assert -vol(notch) > 0.9 * L.wire_ch_w * (L.wire_ch_r0 - L.shelf_id / 2) * (L.z_shelf_top - L.wire_notch_z0)


@pytest.mark.parametrize("preset", [None, "KAIST_90mm_12S"])
def test_internal_estimate_v05d_features(preset):
    """SPEC v0.5d in the sim estimate: the floor wedge (exact triangular prism of its part inside the channel void,
    tabs only) and the tray stop (both presets) match hand volumes and the sizing estimator."""
    from sim import vehicle as vm
    p = _solid_mass_model(vm.layout_mod.load_params(os.path.join(PROJECT_ROOT, "params.json"), preset))
    L = vm.Layout(p)
    rho = p["printer"]["density_g_cm3"] * 1e3
    se = _sizing_estimator()
    sest = se.StructureEstimator(L, p)
    stops = vm._tray_stop_solids(L, rho)
    h = L.z_tray_bottom - L.z_tray_stop_bottom
    assert len(stops) == 2 and h == pytest.approx(L.tray_stop_h)
    for s, b in zip(stops, sest.tray_stop_blocks()):
        assert s.m / rho * 1e9 == pytest.approx(L.rail_gap * L.rail_depth * h, rel=1e-12)
        np.testing.assert_allclose(s.c * 1e3, b.com_mm, atol=1e-9)
    p0 = json.loads(json.dumps(p))
    del p0["avionics_module"]["tray_stop_h"]
    dm = vm.estimate_parts(L, p)["avionics_module"].mass_g - vm.estimate_parts(vm.Layout(p0), p0)["avionics_module"].mass_g
    assert dm == pytest.approx(2 * L.rail_gap * L.rail_depth * h * rho * 1e-6, rel=1e-9)
    if not L.tabs or getattr(L, "wire_ch_open", False):     # v0.9: the open groove has no floor wedge
        assert not L.tabs or L.wire_ch_floor_rz is None
        return
    poly = vm._wire_floor_wedge_rz(L)
    leg = L.wire_ch_r1 - L.body_id / 2
    wedge = vm._plate_rz(poly, L.wire_ch_w, L.wire_theta, rho)
    assert wedge.m / rho * 1e9 == pytest.approx(L.wire_ch_w * leg * leg / 2, rel=1e-12)
    assert wedge.c[2] * 1e3 == pytest.approx(L.wire_ch_z0 + leg / 3)
    assert wedge.c[0] * 1e3 == pytest.approx(L.body_id / 2 + 2 * leg / 3) and abs(wedge.c[1]) < 1e-12   # wire_theta 0
    assert wedge.m / rho * 1e9 == pytest.approx(sest.wire_floor_wedge().volume_mm3, rel=1e-12)


@pytest.mark.parametrize("preset", [None, "KAIST_90mm_12S"])
def test_internal_estimate_v062_gussets_and_radio_log(preset):
    """v0.6.2 in the sim estimate: the intake gains exactly rho x Layout's gusset volume (one solid per window at the
    gusset centroid, on the axis in sum) against the same params without intake_module.window_bottom_end, and the
    radio_log placeholder is the Layout box with the params mass (misc_electronics keeps its box, params mass)."""
    from sim import vehicle as vm
    p = _solid_mass_model(vm.layout_mod.load_params(os.path.join(PROJECT_ROOT, "params.json"), preset))
    L = vm.Layout(p)
    rho = p["printer"]["density_g_cm3"] * 1e3
    assert L.has_window_gable and L.has_radio_log
    g = vm._window_gussets(L, rho)
    assert len(g) == len(L.window_thetas)
    assert sum(s.m for s in g) / rho * 1e9 == pytest.approx(L.gusset_volume_mm3, rel=1e-12)
    np.testing.assert_allclose(sum(s.m * s.c for s in g)[:2], [0.0, 0.0], atol=1e-12)     # equally spaced windows
    p0 = json.loads(json.dumps(p))
    del p0["intake_module"]["window_bottom_end"]
    if getattr(L, "wire_ch_open", False):
        # v0.9: the open motor-lead groove's roof wedge rises above the plain window bottom; without the gable the
        # window cut would bite into it - Layout refuses that combination (sizing/test_sizing.py says the same)
        with pytest.raises(ValueError, match="reaches a window opening"):
            vm.Layout(p0)
        parts = vm.estimate_parts(L, p)
    else:
        parts, parts0 = vm.estimate_parts(L, p), vm.estimate_parts(vm.Layout(p0), p0)
    if not getattr(L, "wire_ch_open", False):
        dm = parts["intake_module"].mass_g - parts0["intake_module"].mass_g
        assert dm == pytest.approx(L.gusset_volume_mm3 * rho * 1e-6, rel=1e-9)
    rl = parts["radio_log"]
    assert rl.kind == "placeholder" and rl.mass_g == pytest.approx(p["electronics"]["radio_log"]["mass_g"])
    np.testing.assert_allclose(rl.com_mm, L.radio_log_center, atol=1e-9)
    assert rl.com_mm[2] == pytest.approx(L.z_payload_bulkhead_bottom - p["electronics"]["radio_log"]["dims_lwh"][2] / 2)
    assert parts["misc_electronics"].mass_g == pytest.approx(p["electronics"]["misc"]["mass_g"])


@pytest.mark.parametrize("preset", [None, "KAIST_90mm_12S"])
def test_internal_estimate_v063_nav_and_fcu(preset):
    """v0.6.3 in the sim estimate: ``nav`` is the v0.6.2 ``fc`` box (tray +x face) with electronics.nav.mass_g, the ``fcu``
    placeholder is the Layout box on the intake bulkhead (foam_t above its top) with electronics.fcu.mass_g, and both
    agree with the sizing estimator; the servo-lead lump (tab mount) is Layout's, its bay legs ending on the FCU."""
    from sim import vehicle as vm
    p = vm.layout_mod.load_params(os.path.join(PROJECT_ROOT, "params.json"), preset)
    L = vm.Layout(p)
    assert L.has_fcu
    parts = vm.estimate_parts(L, p)
    assert "fc" not in parts
    nav, fcu = parts["nav"], parts["fcu"]
    assert nav.kind == fcu.kind == "placeholder"
    assert nav.mass_g == pytest.approx(p["electronics"]["nav"]["mass_g"]) and fcu.mass_g == pytest.approx(p["electronics"]["fcu"]["mass_g"])
    np.testing.assert_allclose(nav.com_mm, [0.5 * (L.nav_x[0] + L.nav_x[1]), 0.0, L.nav_center_z], atol=1e-9)
    np.testing.assert_allclose(fcu.com_mm, L.fcu_center, atol=1e-9)
    assert fcu.com_mm[2] == pytest.approx(L.z_intake_bulkhead_top + p["electronics"]["fcu"]["foam_t"]
                                          + p["electronics"]["fcu"]["dims_lwh"][2] / 2)
    sest = _sizing_estimator().placeholder_parts(L, p)
    for n in ("nav", "fcu"):
        np.testing.assert_allclose(parts[n].com_mm, sest[n].com_mm, atol=1e-9)
        assert parts[n].mass_g == pytest.approx(sest[n].mass_g)
    if L.servo_leads:
        sl = parts["servo_leads"]
        assert sl.mass_g == pytest.approx(L.sl_lead_mass_g) and sl.com_mm[2] == pytest.approx(L.sl_lead_com[2])
        assert L.servo_lead_polyline(0)[-1] == pytest.approx((L.fcu_center[0], L.fcu_center[1], L.fcu_z[1]))


def test_s2_verdict_without_touchdown_term(veh):
    """The S2 Monte Carlo seeds stop a few seconds after the release, so they have no touchdown speed: the full
    verdict can never PASS there (the v0.5c report said 'PASS in 0 / 20'); the summary judges them without the
    touchdown term and says so."""
    from sim.run_scenarios import MC_EXTRA_S, verdict_s2
    p = veh.params
    m = {"apogee_m": float(p["mission"]["target_altitude_m"]), "vz_release_mps": -1.0, "t_abort_s": None,
         "touchdown_speed_mps": None, "cut_by": "CUT", "h_cutoff_m": 100.0, "release_by": "RELEASE", "t_deploy_s": 12.0,
         "release_minus_apogee_s": 0.5}
    assert verdict_s2(m, p)[0] == "FAIL" and "touchdown -" in verdict_s2(m, p)[1]
    v, why = verdict_s2(m, p, touchdown=False)
    assert v == "PASS" and "touchdown not judged" in why
    assert verdict_s2(m | {"touchdown_speed_mps": 5.0}, p)[0] == "PASS"
    assert verdict_s2(m | {"t_abort_s": 3.0, "abort_reason": "I2"}, p, touchdown=False)[0] == "FAIL"   # other terms stay
    assert verdict_s2(m | {"vz_release_mps": -9.0}, p, touchdown=False)[0] == "FAIL"
    assert MC_EXTRA_S > 0


def test_mass_source_fallbacks(tmp_path, veh):
    """cadquery -> fusion360 -> sizing -> internal estimate, each labelled."""
    root = tmp_path / "proj"
    (root / "cad" / "exports").mkdir(parents=True)
    (root / "sizing").mkdir()
    shutil.copy(os.path.join(PROJECT_ROOT, "params.json"), root / "params.json")
    base = estimate_report(veh.layout, veh.params)

    def fake(gen: str, scale: float) -> dict:
        rep = json.loads(json.dumps(base))
        rep["generator"] = gen
        for pt in rep["parts"].values():
            pt["mass_g"] *= scale
            pt["inertia_about_com_kg_m2"] = (np.array(pt["inertia_about_com_kg_m2"]) * scale).tolist()
        rep["parts"]["launch_pad"] = {"kind": "printed", "mass_g": 500.0, "com_mm": [0, 0, -50], "volume_mm3": 1.0,
                                      "density_g_cm3": 1.31, "bbox_mm": [[0, 0, 0], [0, 0, 0]],
                                      "inertia_about_com_kg_m2": np.zeros((3, 3)).tolist()}
        return rep

    with open(root / "cad" / "exports" / "cadquery_report.json", "w", encoding="utf-8") as f:
        json.dump(fake("cadquery", 1.10), f)
    with open(root / "cad" / "exports" / "fusion360_report.json", "w", encoding="utf-8") as f:
        json.dump(fake("fusion360", 1.20), f)
    with open(root / "sizing" / "sizing_report.json", "w", encoding="utf-8") as f:
        json.dump({"generator": "sizing", "total_mass_g": base["totals"]["mass_g"] * 1.30,
                   "cg_z_mm": base["totals"]["com_mm"][2] + 20.0}, f)

    m0 = base["totals"]["mass_g"] * 1e-3
    v1 = load_vehicle(root=str(root))
    assert "cadquery" in v1.mass_source and v1.full.mass_kg == pytest.approx(1.10 * m0, rel=1e-6)   # launch pad excluded
    os.remove(root / "cad" / "exports" / "cadquery_report.json")
    v2 = load_vehicle(root=str(root))
    assert "fusion360" in v2.mass_source and v2.full.mass_kg == pytest.approx(1.20 * m0, rel=1e-6)
    assert v2.post_ejection.mass_kg == pytest.approx(1.20 * base["totals"]["post_ejection"]["mass_g"] * 1e-3, rel=1e-6)
    os.remove(root / "cad" / "exports" / "fusion360_report.json")
    v3 = load_vehicle(root=str(root))
    assert v3.mass_source.startswith("sizing_report.json") and "mass" in v3.mass_source
    assert v3.full.mass_kg == pytest.approx(1.30 * m0, rel=1e-6)
    assert v3.full.com_m[2] * 1e3 == pytest.approx(base["totals"]["com_mm"][2] + 20.0)
    assert v3.full.inertia_kg_m2[0, 0] == pytest.approx(1.30 * base["totals"]["inertia_about_com_kg_m2"][0][0], rel=1e-6)
    assert v3.warnings  # inertia was scaled, not read
    os.remove(root / "sizing" / "sizing_report.json")
    v4 = load_vehicle(root=str(root))
    assert v4.mass_source.startswith("internal estimate") and v4.full.mass_kg == pytest.approx(m0, rel=1e-9)
    # a CAD-schema sizing report (what sizing.py writes) is read as totals
    with open(root / "sizing" / "sizing_report.json", "w", encoding="utf-8") as f:
        json.dump({"generator": "sizing", "totals": base["totals"]}, f)
    v5 = load_vehicle(root=str(root))
    assert "CAD-schema totals" in v5.mass_source and v5.full.mass_kg == pytest.approx(m0, rel=1e-9)
    # explicit preference for the estimate ignores files
    v6 = load_vehicle(root=str(root), prefer="estimate")
    assert v6.mass_source.startswith("internal estimate")
    # a CAD report that lists failed components (partial geometry) is never used: no evidence is not a PASS
    bad = fake("cadquery", 1.10)
    bad["failed_components"] = ["propulsion_module"]
    with open(root / "cad" / "exports" / "cadquery_report.json", "w", encoding="utf-8") as f:
        json.dump(bad, f)
    v7 = load_vehicle(root=str(root))
    assert "cadquery" not in v7.mass_source and any("failed components" in w for w in v7.warnings)


@pytest.mark.parametrize("preset", [None, "KAIST_90mm_12S"])
def test_cad_report_missing_components_is_not_used(tmp_path, preset):
    """A CAD report that silently dropped a part is no evidence: load_vehicle skips it (next source, warning),
    load_cad_report refuses it; launch_pad is optional; the expected set is the internal estimate's (= the sizing
    estimator's expected_report_parts, the SPEC §6 list)."""
    from sim import vehicle as vm
    root = tmp_path / "proj"
    exp = root / "cad" / "exports"
    exp.mkdir(parents=True)
    shutil.copy(os.path.join(PROJECT_ROOT, "params.json"), root / "params.json")
    pp = str(root / "params.json")
    p = vm.layout_mod.load_params(pp, preset)
    L = vm.Layout(p)
    expected = vm.expected_report_parts(L, p)
    se = _sizing_estimator()
    assert sorted(expected) == sorted(se.expected_report_parts(L, p))
    assert "launch_pad" not in expected and len(expected) == len(set(expected))
    base = estimate_report(L, p, pp)
    default = vm.layout_mod.load_params(pp)["meta"]["preset"]
    name = "cadquery_report.json" if preset in (None, default) else f"cadquery_report_{preset}.json"

    def write(parts: dict) -> None:
        rep = json.loads(json.dumps(base))
        rep["generator"], rep["parts"] = "cadquery", parts
        with open(exp / name, "w", encoding="utf-8") as f:
            json.dump(rep, f)

    full = base["parts"]
    write(full)
    v = load_vehicle(root=str(root), preset=preset)
    assert "cadquery" in v.mass_source and v.full.mass_kg == pytest.approx(base["totals"]["mass_g"] * 1e-3, rel=1e-9)
    linkage = "vane_drive_hardware" if getattr(L, "direct_drive", False) else "pushrods_horns_clevises"   # v0.9 mode
    for dropped in ("piston", "edf_unit", linkage, "vane_3"):
        write({n: d for n, d in full.items() if n != dropped})
        v = load_vehicle(root=str(root), preset=preset)
        assert v.mass_source.startswith("internal estimate"), v.mass_source
        assert any(name in w and "missing components" in w and dropped in w and "ignored" in w for w in v.warnings)
        with pytest.raises(ValueError, match="missing components"):
            vm.load_cad_report(str(exp / name), expected)
    # only vane_1..n under the base name (v0.9: vane_coupler_k / vane_drive_hardware stay in)
    vanes = {n: d for n, d in full.items() if not (n.startswith("vane_") and n[len("vane_"):].isdigit())}
    vanes["vane"] = full["vane_1"]                       # one entry for four vanes would undercount: refused
    write(vanes)
    assert load_vehicle(root=str(root), preset=preset).mass_source.startswith("internal estimate")
    rep = json.loads(json.dumps(base))
    del rep["parts"]                                     # totals only: cannot show that nothing was dropped
    with open(exp / name, "w", encoding="utf-8") as f:
        json.dump(rep, f)
    v = load_vehicle(root=str(root), preset=preset)
    assert v.mass_source.startswith("internal estimate") and any("missing components" in w for w in v.warnings)
    with pytest.raises(ValueError):
        vm.load_cad_report(str(exp / name), expected)


def test_mass_source_per_preset(tmp_path, veh):
    """An alt preset reads cadquery_report_<preset>.json / sizing_report_<preset>.json; a report built for another
    preset is skipped (warning), one built from another params.json is used but flagged (as sizing.find_cad_report)."""
    from sim import vehicle as vm
    root = tmp_path / "proj"
    exp = root / "cad" / "exports"
    exp.mkdir(parents=True)
    (root / "sizing").mkdir()
    shutil.copy(os.path.join(PROJECT_ROOT, "params.json"), root / "params.json")
    pp = str(root / "params.json")
    alt = next(iter(vm.layout_mod.load_params(pp)["alt_presets"]))
    p_alt = vm.layout_mod.load_params(pp, alt)
    base_def, base_alt = estimate_report(veh.layout, veh.params, pp), estimate_report(vm.Layout(p_alt), p_alt, pp)
    assert base_alt["preset"] == alt != base_def["preset"] and base_def["params_sha256"]

    def write(path, rep: dict, scale: float, **over) -> None:
        rep = json.loads(json.dumps(rep))
        rep.update(over)
        for pt in rep["parts"].values():
            pt["mass_g"] *= scale
        with open(path, "w", encoding="utf-8") as f:
            json.dump(rep, f)

    m_def, m_alt = base_def["totals"]["mass_g"] * 1e-3, base_alt["totals"]["mass_g"] * 1e-3
    write(exp / "cadquery_report.json", base_def, 1.10)
    v = load_vehicle(root=str(root), preset=alt)                      # only the default preset's report exists
    assert v.mass_source.startswith("internal estimate") and v.full.mass_kg == pytest.approx(m_alt, rel=1e-9)
    assert any("cadquery_report.json" in w and "ignored" in w for w in v.warnings)
    v = load_vehicle(root=str(root))
    assert "cadquery_report.json" in v.mass_source and v.full.mass_kg == pytest.approx(1.10 * m_def, rel=1e-6)
    assert not any("ignored" in w or "different params.json" in w for w in v.warnings)
    write(exp / f"cadquery_report_{alt}.json", base_alt, 1.20)
    v = load_vehicle(root=str(root), preset=alt)
    assert f"cadquery_report_{alt}.json" in v.mass_source and v.full.mass_kg == pytest.approx(1.20 * m_alt, rel=1e-6)
    assert not any("ignored" in w or "different params.json" in w for w in v.warnings)
    write(exp / f"cadquery_report_{alt}.json", base_alt, 1.20, params_sha256="0" * 64)   # stale: used, flagged
    v = load_vehicle(root=str(root), preset=alt)
    assert f"cadquery_report_{alt}.json" in v.mass_source
    assert any("different params.json" in w for w in v.warnings)
    write(exp / f"cadquery_report_{alt}.json", base_def, 1.30)        # mislabelled: preset is not the alt one
    v = load_vehicle(root=str(root), preset=alt, prefer="cad")
    assert v.mass_source.startswith("internal estimate")
    assert any(f"cadquery_report_{alt}.json" in w and "ignored" in w for w in v.warnings)
    # sizing reports: the plain one belongs to the default preset; the suffixed one is read when its preset matches
    os.remove(exp / f"cadquery_report_{alt}.json")
    with open(root / "sizing" / "sizing_report.json", "w", encoding="utf-8") as f:
        json.dump({"preset": base_def["preset"], "totals": base_def["totals"]}, f)
    with open(root / "sizing" / f"sizing_report_{alt}.json", "w", encoding="utf-8") as f:
        json.dump({"preset": base_def["preset"], "totals": base_def["totals"]}, f)
    v = load_vehicle(root=str(root), preset=alt, prefer="sizing")
    assert v.mass_source.startswith("internal estimate") and any("ignored" in w for w in v.warnings)
    with open(root / "sizing" / f"sizing_report_{alt}.json", "w", encoding="utf-8") as f:
        json.dump({"preset": alt, "params_sha256": base_alt["params_sha256"], "totals": base_alt["totals"]}, f)
    v = load_vehicle(root=str(root), preset=alt)
    assert "CAD-schema totals" in v.mass_source and v.source_path.endswith(f"sizing_report_{alt}.json")
    assert v.full.mass_kg == pytest.approx(m_alt, rel=1e-9)


def test_barrowman_hand_calc(veh):
    L = veh.layout
    a = compute_aero(L, veh.params, veh.z_cg_m)
    # LD-Haack (C = 0) encloses exactly half the cylinder volume -> X_cp = L/2 from the tip
    assert a.z_cp_nose_m * 1e3 == pytest.approx(L.z_nose_tip - 0.5 * L.nose_len, rel=0.01)
    fin = veh.params["fin"]
    spec_v01 = (fin["root_chord"], fin["span"], fin["tip_chord"], fin["t"], L.body_od, L.vane_section_od,
                L.nose_len, L.z_nose_tip) == (90.0, 70.0, 40.0, 3.0, 120.0, 98.0, 190.0, 931.0)
    if spec_v01:
        # TIR-33 form, hand numbers: s = 70, d = 120, Cr = 90, Ct = 40, l = sqrt(71^2 + 25^2), K = 1 + 49/119
        s, d, cr, ct = 70.0, 120.0, 90.0, 40.0
        l = math.hypot(71.0, 25.0)
        cn = (1 + 49 / 119) * 16 * (s / d) ** 2 / (1 + math.sqrt(1 + (2 * l / (cr + ct)) ** 2))
        assert a.cn_alpha_fins == pytest.approx(cn, rel=1e-9)
        assert a.cn_alpha_fins == pytest.approx(3.03, abs=0.02)
        xf = 841 + 50 / 3 * (90 + 80) / 130 + (130 - 3600 / 130) / 6
        assert L.z_nose_tip - a.z_cp_fins_m * 1e3 == pytest.approx(xf, abs=1e-6)
    assert a.cn_alpha_total == pytest.approx(2.0 + a.cn_alpha_fins + veh.params["aero"]["body_cn_alpha_per_rad"])
    assert 1.0 < a.cn_alpha_fins < 8.0
    assert a.z_cp_m < veh.z_cg_m   # CP below CG: statically stable in coast


# --------------------------------------------------------------------------- #
# 6-DOF simulation
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("plant", ["v09", "v081"])
def test_deriv_matches_numpy_model(veh, plant):
    """The hand-unrolled derivative equals an independent numpy evaluation.  v0.9: with the direct-drive vane play (the
    vanes = the play operator of the servo states, memory 0) and the rotor's gyroscopic moment -w x h and spool-up
    reaction s I domega/dt (``sim.edf.RotorModel``); 'v081' = both off (the v0.8.1 derivative)."""
    from sim.edf import RotorModel, vane_play
    sc = Scenario(name="t", t_end=0.01) if plant == "v09" else Scenario(name="t", t_end=0.01, vane_play_deg=0.0,
                                                                         rotor_inertia_kg_m2=0.0)
    sim = FlightSim(veh, sc)
    assert sim.play_on == sim.rot_on == (plant == "v09")
    edf, vane = sim.edf, sim.vane
    cfg = sim.cfg_full
    rot = RotorModel.from_params(veh.params, sim.t_fan_n, sc.rotor_inertia_kg_m2)
    for _ in range(5):
        q = quat.random_unit(RNG)
        x = [*RNG.normal(size=3), *RNG.normal(scale=5, size=3), *q, *RNG.normal(scale=0.5, size=3), 30.0,
             *RNG.uniform(-0.3, 0.3, size=4)]
        wind = (1.0, -2.0, 0.0)
        dcmd = list(RNG.uniform(-0.2, 0.2, size=4))
        dx = np.array(sim.deriv(x, cfg, 40.0, dcmd, wind, 0.0, -1))
        # independent model
        R = quat.to_matrix(q)
        v_rel = np.array(x[3:6]) - np.array(wind)
        vb = R.T @ v_rel
        V = np.linalg.norm(vb)
        T = x[13]
        vanes = np.array([vane_play(0.0, x[14 + i], sim.play_h[i]) for i in range(4)])
        Fb, Mb = vane.forces_moments(edf.dynamic_pressure(T), vanes, cfg.z_cg)
        Fb = Fb + np.array([0, 0, T])
        Mb = Mb + np.array([0, 0, edf.swirl_torque(T)])
        if plant == "v09":
            Mb = Mb + np.array(rot.gyro_moment(tuple(x[10:13]), T)) + np.array([0, 0, rot.reaction_moment_z(
                T, (40.0 - T) / edf.tau_s)])
        c = 0.5 * sim.rho * V * sim.a_ref
        Fb = Fb - c * sim.cd * vb - c * sim.cna * np.array([vb[0], vb[1], 0.0])
        FN = -c * sim.cna * np.array([vb[0], vb[1], 0.0])
        Mb = Mb + np.cross(np.array([0, 0, cfg.l_cp]), FN)
        acc = R @ Fb / cfg.m + np.array([0, 0, -G0])
        w = np.array(x[10:13])
        I = np.array(cfg.I)
        wdot = np.linalg.solve(I, Mb - np.cross(w, I @ w))
        qdot = quat.derivative(q, w)
        tdot = (40.0 - T) / edf.tau_s
        ddot = np.clip((np.array(dcmd) - np.array(x[14:18])) / sim.servo_tau, -sim.servo_rate, sim.servo_rate)
        ref = np.concatenate([x[3:6], acc, qdot, wdot, [tdot], ddot])
        np.testing.assert_allclose(dx, ref, rtol=1e-9, atol=1e-9)


def test_sim_holds_on_pad_and_lifts_off(veh):
    sc = Scenario(name="pad", t_end=2.5, arm_time_s=0.2, launch_time_s=0.5, seed=3)   # ARM / LAUNCH go over the link
    res = FlightSim(veh, sc).run()
    L = res.log
    k_launch = int(np.searchsorted(L["t"], 0.5))
    np.testing.assert_allclose(L["pos"][:k_launch] - L["pos"][0], 0.0, atol=1e-12)   # nothing moves before launch
    np.testing.assert_allclose(L["omega"][:k_launch], 0.0, atol=1e-12)
    assert res.t_liftoff is not None and res.t_liftoff > 0.5
    assert L["thrust"][-1] > veh.weight_n and L["vel"][-1, 2] > 0
    assert np.all(np.isfinite(L["pos"]))
    states = set(L["state"].tolist())
    assert {int(State.SAFE), int(State.ARMED), int(State.POWERED)} <= states     # via the ground link (SPEC 8.8)
    assert L["frames_rx"].sum() > 0 and res.link["ground_stats"]["telemetry"] > 0


@pytest.mark.parametrize("lag", [1, 2])
def test_sim_nav_pad_switch_lag(veh, lag):
    """v0.6.3: the NAV's pad -> flight switch (estimator alignment, altitude hold) follows the supervisor state
    ``nav_state_lag_ticks`` ticks late (2; v0.5 single chip: 1) - the state, not the true liftoff."""
    sc = Scenario(name="pad", t_end=2.0, arm_time_s=0.2, launch_time_s=0.5, seed=3,
                  sensors=SensorConfig(nav_state_lag_ticks=lag))
    res = FlightSim(veh, sc).run()
    L = res.log
    k_p = int(np.flatnonzero(L["state"] == int(State.POWERED))[0])      # first tick that ends POWERED
    k_sw = k_p + lag                                                     # first tick in flight mode
    b = L["gyro_bias_est"]
    assert np.any(b[k_sw - 1] != b[k_sw - 2])                            # pad mode: the bias average still moves
    np.testing.assert_array_equal(b[k_sw:], np.broadcast_to(b[k_sw - 1], b[k_sw:].shape))   # then frozen
    no_baro = np.isnan(L["h_baro"])
    held = np.flatnonzero(no_baro[:k_sw])
    assert held.size and np.all(L["h_est"][held] == 0.0) and np.all(L["vz_est"][held] == 0.0)
    k_free = k_sw + int(np.flatnonzero(no_baro[k_sw:])[0])
    assert L["vz_est"][k_free] != 0.0                                    # predicting (still on the pad: tiny)
    assert res.t_liftoff is not None and res.t_liftoff > L["t"][k_sw]    # the switch comes before the liftoff


def test_sim_on_vehicle_link_timing(veh):
    """Uplink frames reach the supervisor at the first tick >= arrival + COM_FORWARD_S; every TELEM is handed to
    the vehicle radio FC_REPLY_DELAY_S after the uplink frame it answers arrived (sim.link_model, v0.6.3)."""
    sc = Scenario(name="link", t_end=3.0, arm_time_s=0.2, launch_time_s=0.5, seed=3, packet_loss=0.0)
    sim = FlightSim(veh, sc)
    res = sim.run()
    dt = sim.dt_c
    t_last = float(res.log["t"][-1])
    ups = [tr for tr in sim.link.log if tr.direction == "up" and not tr.lost
           and tr.t_arrival + lm.COM_FORWARD_S <= t_last + 1e-12]                 # received before the run ended
    downs = [tr for tr in sim.link.log if tr.direction == "down"]
    assert len(ups) > 5 and len(downs) == len(ups)
    for u, d in zip(ups, downs):
        assert d.t_tx == pytest.approx(u.t_arrival + lm.FC_REPLY_DELAY_S, abs=1e-9)
    want = np.zeros(len(res.log["t"]), dtype=int)
    for u in ups:
        k = math.ceil((u.t_arrival + lm.COM_FORWARD_S) / dt - 1e-9)
        if k < want.size:
            want[k] += 1
    np.testing.assert_array_equal(res.log["frames_rx"], want)


def test_sim_hover_scenario_settles_and_metrics(veh):
    sc = Scenario(name="hover", t_end=3.0, bypass_state_machine=True, initial_tilt_deg=10.0, initial_altitude_m=20.0, seed=5)
    res = FlightSim(veh, sc).run()
    m = compute_metrics(res)
    assert not res.nan_abort
    assert m["max_tilt_deg"] == pytest.approx(10.0, abs=1.0)
    assert m["settling_time_s"] is not None and m["settling_time_s"] < 3.0
    assert m["max_delta_deg"] <= veh.params["vane"]["max_deflection_mech_deg"] + 1e-9
    assert m["energy_Wh"] > 0


def test_mahony_converges_at_rest(veh):
    sc = Scenario(name="m", t_end=0.01, sensors=SensorConfig(attitude_estimator="mahony"))
    sim = FlightSim(veh, sc)
    q_true = quat.from_euler(math.radians(6.0), math.radians(-4.0), 0.0)
    g_b = quat.rotate_inverse(q_true, np.array([0.0, 0.0, G0]))
    q_est, bias = quat.IDENTITY.copy(), np.zeros(3)
    for _ in range(16000):   # 40 s: the bias integrator (kp/ki = 20 s) must settle too
        q_est, bias = sim._mahony(q_est, bias, np.zeros(3), g_b, 1 / 400)
    # roll/pitch converge (yaw is unobservable from gravity alone)
    r, p, _ = quat.to_euler(q_est)
    assert math.degrees(r) == pytest.approx(6.0, abs=0.2)
    assert math.degrees(p) == pytest.approx(-4.0, abs=0.2)
    np.testing.assert_allclose(bias, 0.0, atol=math.radians(0.05))


# --------------------------------------------------------------------------- #
# flight-computer output timing (sim.fc_timing)
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("frame_hz, phase_frac, d", [
    (50.0, 0.5, 1), (50.0, 0.0, 1), (50.0, 0.9, 2), (100.0, 0.3, 1), (100.0, 0.0, 0), (60.0, 0.2, 1)])
def test_output_latch_zero_order_hold(frame_hz, phase_frac, d):
    """Values change only at frame edges and carry the newest command available there:
    computed at tick floor(t_e / dt) - d (a command available exactly at the edge is taken)."""
    dt = 1.0 / 250.0
    phase = phase_frac * dt
    latch = OutputLatch(dt, d * dt, frame_hz, [phase], [-1.0])
    n = 500
    ev = []
    for k in range(n):
        out = latch.tick(k, [float(k)])          # command value = index of the tick it was computed at
        assert all(k * dt - EPS_T <= e[0] < (k + 1) * dt - EPS_T for e in out)
        ev += out
    t_f = 1.0 / frame_hz
    edges = [phase + i * t_f for i in range(int(n * dt / t_f) + 2) if phase + i * t_f < n * dt - EPS_T]
    expected = []
    for te in edges:
        j = math.floor((te + EPS_T) / dt) - d
        if j >= 0:
            expected.append((te, float(j)))
    assert len(ev) == len(expected)              # one change per edge (all commands differ), none in between
    for (t_e, c, v), (te, vj) in zip(ev, expected):
        assert c == 0
        assert t_e == pytest.approx(te, abs=1e-9)
        assert v == vj


def test_output_latch_pulse_decode_and_channels():
    dt, f = 1.0 / 250.0, 50.0
    pw = lambda c, v: 1.5e-3 + 1e-4 * c + 1e-5 * v   # noqa: E731
    latch = OutputLatch(dt, dt, f, [0.5 * dt, 0.5 * dt + 0.01], [0.0, 0.0], pw)
    ev = []
    for k in range(60):
        ev += latch.tick(k, [float(k + 1), float(k + 1)])
    for t, c, v in ev:
        te = t - pw(c, v)
        phase = 0.5 * dt + (0.01 if c == 1 else 0.0)
        n = (te - phase) * f
        assert n == pytest.approx(round(n), abs=1e-7)             # applied pulse-width after a frame edge
        assert v == float(math.floor((te + EPS_T) / dt))          # = 1 + index of the tick before the edge tick (d = 1)
    assert {c for _, c, _ in ev} == {0, 1}


@pytest.mark.parametrize("d", [0, 1, 2])
def test_output_latch_continuous_is_pure_delay(d):
    dt = 1.0 / 250.0
    latch = OutputLatch(dt, d * dt, 0.0, [0.0], [0.0])
    for k in range(20):
        ev = latch.tick(k, [float(k + 1)])
        expect = float(k + 1 - d) if k >= d else None
        if expect is None:
            assert ev == []
        else:
            assert ev == [(k * dt, 0, expect)]


def test_mean_wait_and_timer_mapping(veh):
    dt = 1.0 / 250.0
    assert mean_wait_after_available(dt, 50.0, 0.5 * dt) == pytest.approx(0.5 * dt)
    assert mean_wait_after_available(dt, 50.0, 0.0) == pytest.approx(0.0, abs=1e-12)
    assert mean_wait_after_available(dt, 100.0, 0.0) == pytest.approx(0.25 * dt)   # edges alternate 0 and dt/2 after a tick
    fc = veh.params["flight_computer"]
    assert timer_of_channel(fc) == [1, 1, 1, 3, 3]          # PB5/6/7 = OC1A/B/C, PE3/PE4 = OC3A/B
    assert frame_hz_from_timer(fc) == pytest.approx(float(fc["servo_frame_hz"]))


def _short_s1(veh, timing: FCTiming, t_end: float = 0.6, **kw) -> FlightSim:
    sc = Scenario(name="t", t_end=t_end, bypass_state_machine=True, initial_tilt_deg=10.0, initial_altitude_m=20.0,
                  seed=7, timing=timing, **kw)
    return FlightSim(veh, sc)


def test_sim_vane_commands_change_only_at_frame_edges(veh):
    sim = _short_s1(veh, FCTiming())
    res = sim.run()
    L = res.log
    dt, t_f, ph = sim.dt_c, 1.0 / sim.frame_hz, sim.channel_phase_s[0]
    d_s = FCTiming().delay_s(dt)
    assert d_s == pytest.approx(4.71e-3)                               # v0.6.3 co-simulation max (sub-tick)
    assert sim.frame_hz == pytest.approx(float(veh.params["flight_computer"]["servo_frame_hz"]))
    srv, cmd = L["delta_srv"][:, 0], L["delta_cmd"][:, 0]
    changes = np.flatnonzero(np.diff(srv) != 0.0) + 1
    assert changes.size >= 5        # the command must vary (it may sit at its limit for some frames)
    assert changes.size <= int(0.6 * sim.frame_hz) + 1                # at most one change per frame
    for k in changes:
        te = ph + math.floor((k * dt - ph) / t_f) * t_f                # last frame edge before the logged tick
        j = math.floor((te - d_s + EPS_T) / dt)                        # newest command available at the edge
        assert srv[k] == cmd[j]
        t_apply = te + sim.pulse_width_s(0, srv[k])                    # acts at the falling edge of the pulse
        assert (k - 1) * dt < t_apply <= k * dt + 1e-9


@pytest.mark.parametrize("d", [0, 2])
def test_sim_compute_delay_without_frame_latch(veh, d):
    sim = _short_s1(veh, FCTiming(frame_hz=0.0, compute_delay_ticks=d), t_end=0.2)
    L = sim.run().log
    np.testing.assert_array_equal(L["delta_srv"][d:, :], L["delta_cmd"][:len(L["t"]) - d, :])


def test_fc_timing_delay_seconds_default_is_the_measured_max():
    from sim.fc_timing import FC_COMPUTE_DELAY_S_MEASURED
    dt = 1.0 / 250.0
    assert FC_COMPUTE_DELAY_S_MEASURED == pytest.approx(4.71e-3)       # co-simulation max 4709.0 us, rounded up
    assert FCTiming().delay_s(dt) == FC_COMPUTE_DELAY_S_MEASURED
    assert FCTiming(compute_delay_ticks=1).delay_s(dt) == pytest.approx(dt)          # the v0.6.2 one-tick model
    assert FCTiming(compute_delay_s=1.5e-3).delay_s(dt) == pytest.approx(1.5e-3)
    with pytest.raises(ValueError):
        FCTiming(compute_delay_s=1e-3, compute_delay_ticks=1).delay_s(dt)
    with pytest.raises(ValueError):
        FCTiming(compute_delay_s=-1e-3).delay_s(dt)


@pytest.mark.parametrize("frame_hz, phase_frac", [(50.0, 0.5), (50.0, 0.0), (50.0, 0.1), (50.0, 0.2), (100.0, 0.3),
                                                  (60.0, 0.9)])
def test_output_latch_sub_tick_delay(frame_hz, phase_frac):
    """Sub-tick compute delay d = 4.71 ms: an edge at t_e latches the command of the newest tick j with
    j dt + d <= t_e (ties taken), i.e. j = floor((t_e - d) / dt) - an edge less than 0.71 ms after a tick misses
    the command computed one tick earlier and keeps the one before."""
    dt, d = 1.0 / 250.0, 4.71e-3
    phase = phase_frac * dt
    latch = OutputLatch(dt, d, frame_hz, [phase], [-1.0])
    ev = []
    n = 500
    for k in range(n):
        out = latch.tick(k, [float(k)])
        assert all(k * dt - EPS_T <= e[0] < (k + 1) * dt - EPS_T for e in out)
        ev += out
    t_f = 1.0 / frame_hz
    expected = []
    for i in range(int(n * dt / t_f) + 2):
        te = phase + i * t_f
        if te >= n * dt - EPS_T:
            break
        j = math.floor((te - d + EPS_T) / dt)
        if j >= 0:
            expected.append((te, float(j)))
    assert [(round(t, 9), v) for t, _, v in ev] == [(round(t, 9), v) for t, v in expected]
    if phase_frac in (0.0, 0.1):        # edge within 0.71 ms after the tick: two ticks old, not one
        assert all(math.floor((te + EPS_T) / dt) - j == 2 for te, j in expected)
    if phase_frac == 0.5:                # same commands as the one-tick model at the baseline phase
        assert all(math.floor((te + EPS_T) / dt) - j == 1 for te, j in expected)


def test_output_latch_sub_tick_delay_without_frames_changes_inside_the_tick():
    dt, d = 1.0 / 250.0, 4.71e-3
    latch = OutputLatch(dt, d, 0.0, [0.0], [0.0])
    for k in range(10):
        ev = latch.tick(k, [float(k + 1)])
        if k == 0:
            assert ev == []
        else:                             # command of tick k - 1, applied 0.71 ms after tick k
            assert len(ev) == 1 and ev[0][1:] == (0, float(k))
            assert ev[0][0] == pytest.approx((k - 1) * dt + d, abs=1e-12)


def test_mean_wait_with_sub_tick_delay():
    dt, d = 1.0 / 250.0, 4.71e-3
    for ph in (0.0, 0.3 * dt, 0.5 * dt):     # a whole number of ticks of delay changes nothing
        assert mean_wait_after_available(dt, 50.0, ph, 2 * dt) == pytest.approx(mean_wait_after_available(dt, 50.0, ph))
    assert mean_wait_after_available(dt, 50.0, 0.5 * dt, d) == pytest.approx(0.5 * dt - (d - dt))   # 1.29 ms
    assert mean_wait_after_available(dt, 50.0, 0.0, d) == pytest.approx(dt - (d - dt))              # 3.29 ms


def test_sim_sub_tick_compute_delay_without_frame_latch(veh):
    """4.71 ms: the command of tick k acts 0.71 ms after tick k + 1, so the log (taken at the tick) shows it at k + 2."""
    sim = _short_s1(veh, FCTiming(frame_hz=0.0), t_end=0.2)
    L = sim.run().log
    np.testing.assert_array_equal(L["delta_srv"][2:, :], L["delta_cmd"][:len(L["t"]) - 2, :])


# --------------------------------------------------------------------------- #
# MPU-6050 model and the firmware estimator (sim.estimator)
# --------------------------------------------------------------------------- #
def test_mpu6050_quantisation_range_and_noise():
    imu = Mpu6050(16.4, 2000.0, 2048.0, 16.0, G0)
    g = imu.gyro(np.radians([1.0, -0.02, 0.04]), np.zeros(3), np.zeros(3))
    np.testing.assert_allclose(np.degrees(g), [16 / 16.4, 0.0, 1 / 16.4], atol=1e-12)   # int16 counts / 16.4
    x = RNG.normal(scale=0.5, size=(50, 3))
    lsb = math.radians(1.0 / 16.4)
    q = quantize_clip(x, lsb, math.radians(2000.0))
    np.testing.assert_allclose(q / lsb, np.round(q / lsb), atol=1e-9)
    g = imu.gyro(np.radians([3000.0, -3000.0, 10.0]), np.zeros(3), np.zeros(3))
    assert math.degrees(g[0]) == pytest.approx(32767 / 16.4)
    assert math.degrees(g[1]) == pytest.approx(-32768 / 16.4)
    assert imu.gyro_saturated(g) and not imu.gyro_saturated(np.radians([1990.0, 0.0, 0.0]))
    a = imu.accel(np.array([0.0, 0.0, G0]), np.zeros(3), np.zeros(3))
    assert a[2] / G0 * 2048 == pytest.approx(2048.0)
    # white-noise sigma: density x sqrt(ENBW of the first-order DLPF), or of the Nyquist band without it
    assert white_noise_sigma(0.005, 0.0028, 1000.0) == pytest.approx(0.005 * math.sqrt(1 / (4 * 0.0028)))
    assert white_noise_sigma(0.005, 0.0, 1000.0) == pytest.approx(0.005 * math.sqrt(500.0))


def test_gyro_bias_process_bounded_slow_and_ramp():
    dt, sig, tau, bound = 1 / 250, math.radians(0.25), 60.0, math.radians(0.5)
    b = gyro_bias_process(np.random.default_rng(5), 30000, dt, sig, tau, bound)
    assert np.abs(b).max() <= bound + 1e-15
    step_std = np.diff(b, axis=0).std()
    assert step_std == pytest.approx(sig * math.sqrt(1 - math.exp(-2 * dt / tau)), rel=0.05)
    r = math.radians(0.04)
    b = gyro_bias_process(np.random.default_rng(5), 1000, dt, 0.0, tau, bound, ramp=np.array([r, -r, 0.0]),
                          ramp_start_index=100)
    np.testing.assert_allclose(b[:101], 0.0, atol=1e-15)
    np.testing.assert_allclose(b[100 + 250], [r * 1.0, -r * 1.0, 0.0], rtol=1e-9)


def test_estimator_pad_alignment_levels_and_learns_bias():
    dt = 1 / 250
    rng = np.random.default_rng(3)
    q_true = quat.from_euler(math.radians(3.0), math.radians(-2.0), math.radians(40.0))
    f_b = quat.rotate_inverse(q_true, np.array([0.0, 0.0, G0]))
    bias = np.radians([0.3, -0.2, 0.4])
    est = PadAlignGyroEstimator(dt, 2.0)
    for _ in range(2000):
        est.update(bias + rng.normal(0, math.radians(0.05), 3), f_b + rng.normal(0, 0.01, 3), on_pad=True)
    np.testing.assert_allclose(est.bias, bias, atol=math.radians(0.01))
    assert math.degrees(up_error_rad(q_true, est.q)) < 0.05
    assert abs(quat.yaw(est.q)) < 1e-9                                  # pad heading is the yaw reference
    r, p, _ = quat.to_euler(level_from_accel(f_b))
    assert (math.degrees(r), math.degrees(p)) == pytest.approx((3.0, -2.0), abs=1e-6)


def test_estimator_is_gyro_only_after_launch():
    dt = 1 / 250
    q_true = quat.from_euler(math.radians(2.0), math.radians(1.0), 0.0)
    f_b = quat.rotate_inverse(q_true, np.array([0.0, 0.0, G0]))
    bias = np.radians([0.2, 0.1, -0.3])
    est = PadAlignGyroEstimator(dt, 1.0)
    for _ in range(1000):
        est.update(bias, f_b, on_pad=True)
    q_t = q_true.copy()
    w = np.array([0.5, -0.2, 0.8])
    for _ in range(250):                                 # 1 s of rotation, accelerometer reads garbage (thrust)
        q_t = quat.integrate(q_t, w, dt)
        est.update(w + bias, np.array([100.0, -50.0, 3.0]), on_pad=False)
    assert math.degrees(up_error_rad(q_t, est.q)) < 1e-6
    np.testing.assert_allclose(est.rate(w + bias), w, atol=1e-12)
    # a bias change after launch integrates straight into the error: 0.1 deg/s on x_b for 5 s = 0.5 deg about x_b,
    # which tilts the estimated up direction by 0.5 deg x |x_b x up_b| (the vehicle is no longer upright here)
    db = math.radians(0.1)
    for _ in range(int(5.0 / dt)):
        est.update(bias + np.array([db, 0.0, 0.0]), f_b, on_pad=False)
    up_b = quat.rotate_inverse(q_t, np.array([0.0, 0.0, 1.0]))
    expect = 0.5 * float(np.linalg.norm(np.cross([1.0, 0.0, 0.0], up_b)))
    assert math.degrees(up_error_rad(q_t, est.q)) == pytest.approx(expect, rel=0.01)


def test_sim_gyro_only_estimate_tracks_truth_without_bias_drift(veh):
    """Firmware estimator in a short powered flight with no bias drift and no accelerometer bias:
    the tilt-estimate error stays at the levelling/noise floor."""
    sens = SensorConfig(gyro_bias_sigma_rad_s=0.0, accel_bias_mps2=0.0)
    sc = Scenario(name="t", t_end=4.0, arm_time_s=0.2, launch_time_s=0.5, seed=9, sensors=sens)
    L = FlightSim(veh, sc).run().log
    assert L["state"][-1] == int(State.POWERED)
    assert L["tilt_est_err_deg"].max() < 0.2


def test_truth_noise_option_still_available(veh):
    sens = SensorConfig(attitude_estimator="truth_noise")
    L = _short_s1(veh, FCTiming(), t_end=0.4, sensors=sens).run().log
    err = L["tilt_est_err_deg"]
    assert 0.2 < np.sqrt(np.mean(err ** 2)) < 1.5                      # ~0.5 deg per axis of white noise
    np.testing.assert_array_equal(L["tilt_ctrl_deg"], L["tilt_deg"])
    with pytest.raises(ValueError):
        FlightSim(veh, Scenario(name="t", sensors=SensorConfig(attitude_estimator="nope")))


def test_spin_supervisor_triggers_abort(veh):
    """Vane hard-over: interlock I3 (default ON, params.mission thresholds) turns the spin-up into ABORT before the
    MPU-6050 saturates; with I3 disabled the tilt abort I2 never fires (the v0.2 finding) and there is no ABORT."""
    stuck = float(veh.params["vane"]["max_deflection_mech_deg"])
    base = Scenario(name="t", t_end=6.0, seed=44, vane_failure_index=0, vane_failure_t_after_launch_s=2.0,
                    vane_failure_stuck_deg=stuck)
    res = FlightSim(veh, base).run()
    ab = [(t, e) for t, e in res.events if "POWERED -> ABORT: I3" in e]
    assert ab
    t_fault = next(t for t, e in res.events if e.startswith("FAULT"))
    assert 0.0 < ab[0][0] - t_fault < 1.0
    assert "full scale" not in ab[0][1]          # the rate limit fires before the gyro clips
    assert any("ABORT -> RELEASED" in e for _, e in res.events)
    off = replace(base, options=SimOptions(supervisor_enabled=False))
    res_off = FlightSim(veh, off).run()
    assert not any("-> ABORT" in e for _, e in res_off.events)


# --------------------------------------------------------------------------- #
# linear loop margins (sim.loop_margins)
# --------------------------------------------------------------------------- #
def test_loop_margins_integrator_plus_delay():
    """L = Kp e^{-s tau} / (I s): crossover Kp/I, PM = 90 deg - w_c tau, phase crossover at pi/(2 tau)."""
    g = {"rate_kp_xy": 1.0, "rate_ki_xy": 0.0, "rate_kd_xy": 0.0, "att_kp_xy": 0.0, "gyro_lpf_hz": 0.0, "d_term_lpf_hz": 0.0}
    dt, inertia, tau = 1e-5, 0.1, 0.02
    m = axis_margins(g, "xy", dt, inertia, 1.0, 0.0, 0.0, tau, 0.0, n=40000)
    wc = 1.0 / inertia
    assert m["crossover_rad_s"] == pytest.approx(wc, rel=1e-3)
    assert m["phase_margin_deg"] == pytest.approx(90.0 - math.degrees(wc * tau), abs=0.05)
    assert m["gain_margin_upper_x"] == pytest.approx((math.pi / (2 * tau)) / wc, rel=2e-3)


def test_written_gains_meet_margin_targets(veh):
    """The gains.json written by the tuner keeps PM >= 30 deg on both axes at the baseline timing."""
    gp = os.path.join(PROJECT_ROOT, "sim", "out", "gains.json")
    if not os.path.exists(gp):
        pytest.skip("sim/out/gains.json not generated yet")
    with open(gp, "r", encoding="utf-8") as f:
        gj = json.load(f)
    if gj.get("loop_hz") != veh.params["control"]["loop_hz"]:
        pytest.skip("gains.json was tuned for another loop rate")
    from sim.run_scenarios import GM_MIN_X, PM_MIN_DEG, linear_margins
    g = gains_from_control(veh.params["control"], gj)
    lin = linear_margins(veh, g, FCTiming(), SensorConfig())
    for ax in ("xy", "z"):
        assert lin[ax]["phase_margin_deg"] >= PM_MIN_DEG - 0.5
        assert lin[ax]["gain_margin_upper_x"] >= GM_MIN_X


def test_gains_header_is_valid_c99_literals(tmp_path):
    """gains.h is consumed by the avr-gcc C99 firmware: every float literal needs a '.' or an exponent."""
    import re
    from sim.run_scenarios import write_gains_header
    path = tmp_path / "gains.h"
    write_gains_header(str(path), {"loop_hz": 250, "rate_kd_z": 0.0, "d_term_lpf_hz": 40.0, "att_kp_xy": 3.5396,
                                   "tiny": 1e-7, "yaw_hold": True, "_meta": {"date": "x"}})
    text = path.read_text(encoding="utf-8")
    defs = dict(re.findall(r"#define (GAINS_\w+) (\S+)", text))
    assert defs == {"GAINS_LOOP_HZ": "250", "GAINS_RATE_KD_Z": "0.0f", "GAINS_D_TERM_LPF_HZ": "40.0f",
                    "GAINS_ATT_KP_XY": "3.5396f", "GAINS_TINY": "1e-07f", "GAINS_YAW_HOLD": "1"}
    assert "#ifndef GAINS_H" in text and "namespace" not in text and "_META" not in text


def test_gen_gains_header_writes_and_checks_outside_the_project_drive(tmp_path):
    """tools/gen_gains_header.py with --out under %TEMP% (another drive than the project on this PC): relpath across
    drives raised ValueError; the header is written, --check agrees, and the rendering is deterministic."""
    tools = os.path.join(PROJECT_ROOT, "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import contextlib
    import io
    import gen_gains_header as ggh
    out = tmp_path / "gains.h"
    with contextlib.redirect_stdout(io.StringIO()):
        assert ggh.main(["--out", str(out)]) == 0
        assert ggh.main(["--out", str(out), "--check"]) == 0
    text = out.read_text(encoding="utf-8")
    assert text == ggh.render(ggh.resolve())                     # no timestamps: same input, same bytes
    assert "#define GAINS_PARAMS_STALE" in text and "#define GAIN_RATE_KP_XY" in text
    r = ggh.rel(str(out))
    assert "\\" not in r and r.endswith("/gains.h")


# =========================================================================== #
# v0.4 ground link: protocol (sim.link_proto)
# =========================================================================== #
import binascii  # noqa: E402
import random  # noqa: E402
import struct  # noqa: E402


def _ref_frame(veh: int, typ: int, seq: int, payload: bytes) -> bytes:
    """Independent encoder: binascii.crc_hqx is CRC-16/CCITT (poly 0x1021, MSB first) with the given init."""
    body = bytes([3 + len(payload), veh, typ, seq]) + payload
    return bytes([0xA5, 0x5A]) + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


def test_crc16_ccitt_check_value_and_independent_crc():
    assert lp.crc16_ccitt(b"123456789") == 0x29B1                        # CRC-16/CCITT-FALSE check value
    assert lp.crc16_ccitt(b"") == 0xFFFF
    rng = random.Random(7)
    for _ in range(200):
        d = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 40)))
        assert lp.crc16_ccitt(d) == binascii.crc_hqx(d, 0xFFFF)


def test_frame_fixed_byte_vectors():
    """Hand-checked frames (hex computed with binascii.crc_hqx, see _ref_frame) - the firmware must emit these bytes."""
    assert lp.encode_command(90, 1, lp.Command(lp.CMD_ARM)).hex() == "a55a085a100101000000001fb3"
    assert lp.encode_command(90, 0x2A, lp.Command(lp.CMD_THROTTLE, 750, 0xBEEF)).hex() == "a55a085a102a04ee02efbeab68"
    tlm = lp.Telemetry(2, 0x2F, 0, 7, 0x1234, 12136, 1193, 2271, -125, 50, -17999, 134, 1000, 3900, 2875)
    assert lp.encode_telemetry(90, 7, tlm).hex() == ("a55a1f5a2007022f00073412682f0000a904df0883ff3200b1b98600e8033c0f3b0b"
                                                     "f3e4")
    assert len(lp.encode_command(90, 0, lp.Command(0))) == lp.CMD_FRAME_BYTES == 13
    assert len(lp.encode_telemetry(90, 0, tlm)) == lp.TELEM_FRAME_BYTES == 36
    assert lp.CMD_PAYLOAD_LEN == 5 and lp.TELEM_PAYLOAD_LEN == 28
    # v0.7 VTOL: ON / UP and TELEM v2 (TYPE 0x21: the v1 payload + phase u8 + h_tgt i16 in 0.1 m) - 31 bytes, LEN_MAX 34
    assert lp.encode_command(90, 0x2B, lp.Command(lp.CMD_ON, 0, 0xBEEF)).hex() == "a55a085a102b080000efbef0ee"
    assert lp.encode_command(90, 0x2C, lp.Command(lp.CMD_UP)).hex() == "a55a085a102c0a00000000ab24"
    t2 = lp.Telemetry(2, 0x2F, 0, 7, 0x1234, 12136, 49, -4, -125, 50, -17999, 168, 918, 3860, 2875, lp.PHASE_HOLD, 50)
    b2 = lp.encode_telemetry(90, 7, t2)
    assert b2.hex() == "a55a225a2107022f00073412682f00003100fcff83ff3200b1b9a8009603140f3b0b0332000adf"
    assert b2 == _ref_frame(90, 0x21, 7, struct.pack("<BBBBHIhhhhhHHHHBh", 2, 0x2F, 0, 7, 0x1234, 12136, 49, -4, -125, 50,
                                                      -17999, 168, 918, 3860, 2875, 3, 50))
    assert len(b2) == lp.TELEM2_FRAME_BYTES == 39 and lp.TELEM2_PAYLOAD_LEN == 31
    assert (lp.TYPE_TELEM2, lp.CMD_ON, lp.CMD_OFF, lp.CMD_UP, lp.CMD_DOWN) == (0x21, 8, 9, 10, 11)
    assert t2.h_tgt_m == 5.0 and t2.phase_name == "HOLD" and t2.is_v2 and not tlm.is_v2 and tlm.h_tgt_m is None
    assert lp.decode_telemetry(lp.FrameDecoder(90, lp.TELEM_TYPES).feed(b2)[0]) == t2
    assert b2[:-5][6:] == lp.encode_telemetry(90, 7, replace(t2, phase=None, h_tgt_dm=None))[6:-2]   # v2 = v1 + 3 bytes
    # v0.8 VTOL + GPS: TELEM v3 (TYPE 0x22) = the v2 payload + gps_flags u8, num_sv u8, hacc u8 (0.1 m), pos_mode u8,
    # dist u16 (0.1 m, 0xFFFF none) - 37 bytes, LEN_MAX 40
    t3 = replace(t2, gps_flags=0x2F, num_sv=9, hacc_dm=25, pos_mode=2, dist_dm=7)
    b3 = lp.encode_telemetry(90, 7, t3)
    assert b3 == _ref_frame(90, 0x22, 7, struct.pack("<BBBBHIhhhhhHHHHBhBBBBH", 2, 0x2F, 0, 7, 0x1234, 12136, 49, -4, -125,
                                                      50, -17999, 168, 918, 3860, 2875, 3, 50, 0x2F, 9, 25, 2, 7))
    assert len(b3) == lp.TELEM3_FRAME_BYTES == 45 and lp.TELEM3_PAYLOAD_LEN == 37
    assert lp.LEN_MAX == 3 + 39 == 3 + lp.TELEM5_PAYLOAD_LEN       # v0.9 RP1: TELEM v5 is the longest (v0.8: 3 + 37)
    assert b3[6:6 + 31] == b2[6:6 + 31] and t3.is_v3 and not t2.is_v3 and t3.dist_m == pytest.approx(0.7)
    assert lp.decode_telemetry(lp.FrameDecoder(90, lp.TELEM_TYPES).feed(b3)[0]) == t3
    t3n = lp.Telemetry.from_physical(state=0, flags=0, reason=lp.REJ_GPS, ack_seq=0, nonce=0, t_ms=0, h_m=0.0, vz_mps=0.0,
                                     roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, tilt_deg=0.0, throttle=0.0, vcell_v=4.0,
                                     loop_max_us=0.0, phase=0, h_tgt_m=0.0, gps=(0x20, 4, 300, 0, float('nan')))
    assert (t3n.dist_dm, t3n.hacc_dm, t3n.dist_m, t3n.wire_type) == (lp.DIST_NONE, 255, None, lp.TYPE_TELEM3)
    far = lp.Telemetry.from_physical(state=0, flags=0, reason=0, ack_seq=0, nonce=0, t_ms=0, h_m=0.0, vz_mps=0.0,
                                     roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, tilt_deg=0.0, throttle=0.0, vcell_v=4.0,
                                     loop_max_us=0.0, phase=0, h_tgt_m=0.0, gps=(0, 0, 0, 0, 1e9))
    assert far.dist_dm == lp.DIST_NONE - 1                                # far away saturates below 'none'


def test_protocol_round_trip_random():
    rng = random.Random(11)
    dec_up, dec_down = lp.FrameDecoder(90, (lp.TYPE_CMD,)), lp.FrameDecoder(90, (lp.TYPE_TELEM,))
    for _ in range(300):
        c = lp.Command(rng.randrange(8), rng.randrange(0x10000), rng.randrange(0x10000))
        seq = rng.randrange(256)
        b = lp.encode_command(90, seq, c)
        assert b == _ref_frame(90, lp.TYPE_CMD, seq, struct.pack("<BHH", c.cmd, c.arg, c.nonce))
        (fr,) = dec_up.feed(b)
        assert (fr.seq, lp.decode_command(fr)) == (seq, c) and fr.encode() == b
        t = lp.Telemetry(rng.randrange(6), rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(0x10000),
                         rng.randrange(0x100000000), *[rng.randrange(-0x8000, 0x8000) for _ in range(5)],
                         *[rng.randrange(0x10000) for _ in range(4)])
        b = lp.encode_telemetry(90, seq, t)
        (fr,) = dec_down.feed(b)
        assert lp.decode_telemetry(fr) == t
    assert dec_up.stats["frames"] == dec_down.stats["frames"] == 300
    with pytest.raises(ValueError):
        lp.Command(1, 70000, 0)
    with pytest.raises(ValueError):
        lp.Telemetry(0, 0, 0, 0, 0, 0, 40000, 0, 0, 0, 0, 0, 0, 0, 0)
    # v0.7: VTOL command codes and TELEM v2 through the same decoder rules
    dec2 = lp.FrameDecoder(90, lp.TELEM_TYPES)
    for _ in range(300):
        c = lp.Command(rng.choice(lp.VTOL_ONLY_CMDS), rng.randrange(0x10000), rng.randrange(0x10000))
        seq = rng.randrange(256)
        (fr,) = dec_up.feed(lp.encode_command(90, seq, c))
        assert lp.decode_command(fr) == c
        t = lp.Telemetry(rng.randrange(6), rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(0x10000),
                         rng.randrange(0x100000000), *[rng.randrange(-0x8000, 0x8000) for _ in range(5)],
                         *[rng.randrange(0x10000) for _ in range(4)], rng.randrange(256), rng.randrange(-0x8000, 0x8000))
        b = lp.encode_telemetry(90, seq, t)
        assert b == _ref_frame(90, lp.TYPE_TELEM2, seq, struct.pack(lp.TELEM2_FMT, *t.wire_values()))
        (fr,) = dec2.feed(b)
        assert fr.type == lp.TYPE_TELEM2 and lp.decode_telemetry(fr) == t and fr.encode() == b
    assert dec_down.feed(lp.encode_telemetry(90, 1, t)) == [] and dec_down.stats["wrong_type"] == 1   # v1-only decoder
    for bad in (dict(phase=1), dict(h_tgt_dm=5), dict(phase=256, h_tgt_dm=0), dict(phase=0, h_tgt_dm=40000)):
        with pytest.raises(ValueError):
            lp.Telemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, **bad)
    with pytest.raises(ValueError):
        lp.Telemetry.unpack(bytes(30))
    with pytest.raises(ValueError):                                       # a TELEM2 frame must carry 31 bytes
        lp.decode_telemetry(lp.Frame(90, lp.TYPE_TELEM2, 0, bytes(28)))


def test_telemetry_scaling_rounding_and_saturation():
    t = lp.Telemetry.from_physical(state=1, flags=0, reason=0, ack_seq=0, nonce=0, t_ms=4, h_m=12.25, vz_mps=-0.125,
                                   roll_deg=-0.005, pitch_deg=0.015, yaw_deg=-180.0, tilt_deg=-1.0, throttle=0.0625,
                                   vcell_v=3.3, loop_max_us=1e6)
    assert (t.h_dm, t.vz_cms, t.roll_cdeg, t.pitch_cdeg, t.yaw_cdeg) == (123, -13, -1, 2, -18000)   # half away from zero
    assert (t.tilt_cdeg, t.throttle_milli, t.vcell_mv, t.loop_max_us) == (0, 63, 3300, 65535)      # clamp / saturate
    t = lp.Telemetry.from_physical(state=1, flags=0, reason=0, ack_seq=0, nonce=0, t_ms=4, h_m=5000.0, vz_mps=-400.0,
                                   roll_deg=float("nan"), pitch_deg=float("inf"), yaw_deg=0.0, tilt_deg=700.0, throttle=2.0,
                                   vcell_v=4.2, loop_max_us=0.0)
    assert (t.h_dm, t.vz_cms, t.roll_cdeg, t.pitch_cdeg, t.tilt_cdeg, t.throttle_milli) == (32767, -32768, 0, 32767, 65535, 2000)
    assert t.h_m == pytest.approx(3276.7) and t.vz_mps == pytest.approx(-327.68)
    assert lp.round_half_away(2.5) == 3 and lp.round_half_away(-2.5) == -3 and lp.round_half_away(0.49) == 0
    assert lp.Command.throttle(0.6005).arg == 601
    # TELEM v2 target: the h scaling (0.1 m, half away from zero, saturated)
    kw = dict(state=2, flags=0, reason=0, ack_seq=0, nonce=0, t_ms=0, h_m=0.0, vz_mps=0.0, roll_deg=0.0, pitch_deg=0.0,
              yaw_deg=0.0, tilt_deg=0.0, throttle=0.0, vcell_v=4.0, loop_max_us=0.0, phase=lp.PHASE_HOLD)
    for h_t, want in ((5.0, 50), (12.25, 123), (-0.05, -1), (0.04, 0), (5000.0, 32767), (-1e9, -32768), (float("nan"), 0)):
        assert lp.Telemetry.from_physical(h_tgt_m=h_t, **kw).h_tgt_dm == want, h_t


def test_seq_newer_serial_arithmetic():
    cases = {(1, 0): True, (0, 0): False, (127, 0): True, (128, 0): False, (0, 255): True, (3, 250): True,
             (250, 3): False, (200, 100): True, (100, 200): False}
    for (a, b), want in cases.items():
        assert lp.seq_newer(a, b) is want, (a, b)


def test_decoder_resync_and_drop_rules():
    good = [lp.encode_command(90, s, lp.Command(lp.CMD_KILL if s % 2 else lp.CMD_ARM, s, s)) for s in range(1, 6)]
    bad_crc = bytearray(good[0])
    bad_crc[7] ^= 0x10
    other_veh = lp.encode_command(91, 9, lp.Command(lp.CMD_ARM))
    telem = lp.encode_telemetry(90, 3, lp.Telemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    short_cmd = lp.encode_frame(90, lp.TYPE_CMD, 2, b"\x01\x02")
    stream = (b"\x00\xa5\x13" + bytes(bad_crc) + good[1] + b"\xa5\xa5" + good[2] + other_veh + telem + short_cmd
              + bytes([0xA5, 0x5A, 0x01]) + good[3] + bytes([0xA5, 0x5A, lp.LEN_MAX]) + good[4] + bytes(40))
    dec = lp.FrameDecoder(90, (lp.TYPE_CMD,))
    got = dec.feed(stream)
    assert [f.seq for f in got] == [2, 3, 4, 5]                  # every good frame found again, nothing else
    s = dec.stats
    assert s["bad_crc"] >= 2 and s["bad_len"] == 1 and s["wrong_vehicle"] == 1 and s["wrong_type"] == 1
    assert s["bad_payload_len"] == 1 and s["frames"] == 4
    one = lp.FrameDecoder(90, (lp.TYPE_CMD,))                    # byte-at-a-time = block
    assert [f for b in stream for f in one.feed(bytes([b]))] == got and one.stats == dec.stats
    any_veh = lp.FrameDecoder(None, (lp.TYPE_CMD, lp.TYPE_TELEM))
    assert len(any_veh.feed(other_veh + telem)) == 2


def test_link_vectors_independent_check_and_write():
    """>= 50 frames incl. corrupted ones, verified against the independent encoder; written for the firmware."""
    vid = int(load_vehicle(prefer="estimate").params["flight_computer"]["link"]["vehicle_id"])
    v = lp.make_test_vectors(vid)
    assert len(v["frames"]) >= 50 and v["crc16_ccitt_false_check"]["crc"] == 0x29B1
    n_valid = n_bad = 0
    for vec in v["frames"] + v["streams"]:
        data = bytes.fromhex(vec["hex"])
        for ef in vec["expected_frames"]:                      # every expected frame is a correct frame inside the data
            ref = _ref_frame(ef["veh"], ef["type"], ef["seq"], bytes.fromhex(ef["payload_hex"]))
            assert ref.hex() in data.hex()
            assert ef["veh"] == vid and ef["type"] in vec["decoder"]["accept_types"]
            if "cmd" in ef:
                assert struct.pack("<BHH", ef["cmd"]["cmd"], ef["cmd"]["arg"], ef["cmd"]["nonce"]).hex() == ef["payload_hex"]
        if vec in v["frames"]:
            if vec["expected_frames"]:
                n_valid += 1
                assert len(vec["expected_frames"]) == 1 and vec["hex"] == _ref_frame(
                    vid, vec["expected_frames"][0]["type"], vec["expected_frames"][0]["seq"],
                    bytes.fromhex(vec["expected_frames"][0]["payload_hex"])).hex()
            else:
                n_bad += 1
    assert n_valid >= 30 and n_bad >= 15
    names = {vec["name"] for vec in v["frames"]}
    assert {"wrong_vehicle", "bad_payload_len", "truncated", "sync_broken", "corrupt_len_255"} <= names
    # v0.7: VTOL commands and TELEM v2 (the independent encoder above checked their bytes too)
    by = {vec["name"]: vec for vec in v["frames"] + v["streams"]}
    t2 = [vec for vec in v["frames"] if vec["name"].startswith("telem2_") and vec["expected_frames"]
          and vec["expected_frames"][0]["type"] == lp.TYPE_TELEM2]
    assert len(t2) >= 10 and all(len(vec["expected_frames"][0]["telem"]) == 17 for vec in t2)
    assert all(len(ef["telem"]) == 15 for vec in v["frames"] + v["streams"] for ef in vec["expected_frames"]
               if ef["type"] == lp.TYPE_TELEM)                          # v1 frames keep the v0.6.3 field set
    assert {vec["expected_frames"][0]["cmd"]["cmd"] for vec in v["frames"] if vec["name"].startswith("cmd_")
            and vec["expected_frames"]} >= set(lp.VTOL_ONLY_CMDS)
    assert by["telem2_on_v1_decoder"]["expected_stats"]["wrong_type"] == 1
    assert by["telem_v1_len_31"]["expected_stats"]["bad_payload_len"] == 1
    assert by["telem2_len_28"]["expected_stats"]["bad_payload_len"] == 1
    assert by[f"telem2_len_{lp.LEN_MAX + 1}"]["expected_stats"]["bad_len"] == 1
    assert [ef["type"] for ef in by["telem_v1_v2_mixed_with_noise"]["expected_frames"]] == [
        lp.TYPE_TELEM, lp.TYPE_TELEM2, lp.TYPE_TELEM2, lp.TYPE_TELEM, lp.TYPE_TELEM2]
    assert len(by["telem2_inside_bad_candidate"]["expected_frames"]) == 1
    assert v["constants"]["LEN_MAX"] == 42 and v["constants"]["TYPE_TELEM2"] == 0x21   # v0.9 RP1 (v0.8: 40)
    # v0.8: TELEM v3 frames, decoded with all 22 fields by the v0.8 ground decoder, refused by the v0.7 one
    assert v["constants"]["TYPE_TELEM3"] == 0x22 and v["constants"]["TELEM3_FRAME_BYTES"] == 45
    t3 = [vec for vec in v["frames"] if vec["name"].startswith("telem3_") and vec["expected_frames"]
          and vec["expected_frames"][0]["type"] == lp.TYPE_TELEM3]
    assert len(t3) >= 10 and all(len(vec["expected_frames"][0]["telem"]) == 22 for vec in t3)
    assert by["telem3_on_v2_decoder"]["expected_stats"]["wrong_type"] == 1
    assert by["telem3_len_31"]["expected_stats"]["bad_payload_len"] == 1
    assert by["telem3_len_max_plus1"]["expected_stats"]["bad_len"] >= 1
    assert [ef["type"] for ef in by["telem_v1_v2_v3_mixed_with_noise"]["expected_frames"]] == [
        lp.TYPE_TELEM, lp.TYPE_TELEM3, lp.TYPE_TELEM2, lp.TYPE_TELEM3, lp.TYPE_TELEM3]
    assert all(s["expected_frames"] for s in v["streams"])
    assert [sc["int"] for sc in v["scaling"]] == [123, -123, 32767, 13, -32768, 0, 63, 3300]
    assert lp.make_test_vectors(vid) == v                        # deterministic
    out = os.path.join(PROJECT_ROOT, "sim", "out")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "link_vectors.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(v, f, indent=1)


# =========================================================================== #
# v0.4 onboard supervisor (sim.onboard) - SPEC 8.8
# =========================================================================== #
class _Sup:
    """Drives an OnboardSupervisor tick by tick with a scripted ground (default inputs: healthy, upright, on the pad)."""

    def __init__(self, sp: SupervisorParams, seed: int = 1, reset_in_flight: bool = False, **inp):
        self.sp = sp
        self.sup = OnboardSupervisor(sp, seed=seed, reset_in_flight=reset_in_flight)
        self.k = 0
        self.seq = 0
        self.base = dict(h_m=0.0, vz_mps=0.0, tilt_deg=0.5, v_cell=4.1)
        self.base.update(inp)
        self.out = None

    def tick(self, frames=(), **kw):
        d = dict(self.base)
        d.update(kw)
        self.out = self.sup.update(OnboardInputs(tick=self.k, frames=list(frames), **d))
        self.k += 1
        return self.out

    def send(self, cmd: int, arg: int = 0, nonce: int | None = None, seq: int | None = None, **kw):
        if seq is None:
            self.seq = (self.seq + 1) & 0xFF
            seq = self.seq
        nonce = self.sup.nonce if nonce is None else nonce
        return self.tick([UplinkFrame(seq, lp.Command(cmd, arg, nonce))], **kw)

    def hb(self, **kw):
        return self.tick([UplinkFrame(self.seq, lp.Command(lp.CMD_NONE))], **kw)

    def run(self, n: int, hb_every: int | None = None, **kw):
        for i in range(n):
            if hb_every and i % hb_every == 0:
                self.hb(**kw)
            else:
                self.tick(**kw)
        return self.out

    @property
    def last_code(self) -> int:
        return self.sup.exec_log[-1]["code"]


def _sp(veh: Vehicle, **kw) -> SupervisorParams:
    """The ballistic build (v0.7: params.json mission.mode is 'vtol'; this section tests the v0.4 mission)."""
    return SupervisorParams.from_params(veh.params, mode=lp.MODE_BALLISTIC, **kw)


def _to(veh: Vehicle, state: str, **kw) -> _Sup:
    """A harness in SAFE / ARMED / POWERED / UNPOWERED / RELEASED / ABORT, or ARMED+pending / UNPOWERED+pending."""
    h = _Sup(_sp(veh, **kw))
    h.hb()
    h.tick()
    if state == "SAFE":
        return h
    assert h.send(lp.CMD_ARM).state == State.ARMED
    if state.startswith("ARMED"):
        if state.endswith("pending"):
            h.send(lp.CMD_RELEASE)
        return h
    assert h.send(lp.CMD_LAUNCH).state == State.POWERED
    if state == "POWERED":
        return h
    if state.startswith("UNPOWERED"):
        assert h.send(lp.CMD_CUT).state == State.UNPOWERED
        if state.endswith("pending"):
            h.send(lp.CMD_RELEASE)
        return h
    if state == "ABORT":
        assert h.send(lp.CMD_KILL).state == State.ABORT
        return h
    assert state == "RELEASED"
    h.send(lp.CMD_RELEASE)
    h.run(h.sp.release_after_cut_ticks + 1, hb_every=20)
    assert h.sup.state == State.RELEASED
    return h


S_, A_, P_, U_, R_, AB_ = State.SAFE, State.ARMED, State.POWERED, State.UNPOWERED, State.RELEASED, State.ABORT
CMD_VARIANTS = {"ARM": (lp.CMD_ARM, 0, True), "DISARM": (lp.CMD_DISARM, 0, True), "LAUNCH": (lp.CMD_LAUNCH, 0, True),
                "LAUNCH_badnonce": (lp.CMD_LAUNCH, 0, False), "THROTTLE_500": (lp.CMD_THROTTLE, 500, True),
                "THROTTLE_500_badnonce": (lp.CMD_THROTTLE, 500, False), "THROTTLE_0": (lp.CMD_THROTTLE, 0, True),
                "CUT": (lp.CMD_CUT, 0, True), "RELEASE": (lp.CMD_RELEASE, 0, True), "KILL": (lp.CMD_KILL, 0, True),
                "UNKNOWN_9": (9, 0, True)}
_RS, _RN, _RP, _RU = lp.REJ_STATE, lp.REJ_NONCE, lp.REJ_RELEASE_PENDING, lp.REJ_UNKNOWN_CMD
# start state -> command -> (state after, code); code 0 = executed
TRANSITIONS = {
    "SAFE": {"ARM": (A_, 0), "DISARM": (S_, _RS), "LAUNCH": (S_, _RS), "LAUNCH_badnonce": (S_, _RS), "THROTTLE_500": (S_, _RS),
             "THROTTLE_500_badnonce": (S_, _RS), "THROTTLE_0": (S_, _RS), "CUT": (S_, _RS), "RELEASE": (S_, _RS),
             "KILL": (S_, _RS), "UNKNOWN_9": (S_, _RU)},
    "ARMED": {"ARM": (A_, _RS), "DISARM": (S_, 0), "LAUNCH": (P_, 0), "LAUNCH_badnonce": (A_, _RN), "THROTTLE_500": (P_, 0),
              "THROTTLE_500_badnonce": (A_, _RN), "THROTTLE_0": (A_, lp.REJ_THROTTLE_ZERO), "CUT": (A_, _RS),
              "RELEASE": (A_, 0), "KILL": (S_, 0), "UNKNOWN_9": (A_, _RU)},
    "POWERED": {"ARM": (P_, _RS), "DISARM": (P_, _RS), "LAUNCH": (P_, _RS), "LAUNCH_badnonce": (P_, _RS),
                "THROTTLE_500": (P_, 0), "THROTTLE_500_badnonce": (P_, 0), "THROTTLE_0": (U_, 0), "CUT": (U_, 0),
                "RELEASE": (U_, 0), "KILL": (AB_, 0), "UNKNOWN_9": (P_, _RU)},
    "UNPOWERED": {"ARM": (U_, _RS), "DISARM": (U_, _RS), "LAUNCH": (U_, _RS), "LAUNCH_badnonce": (U_, _RS),
                  "THROTTLE_500": (P_, 0), "THROTTLE_500_badnonce": (P_, 0), "THROTTLE_0": (U_, 0), "CUT": (U_, _RS),
                  "RELEASE": (U_, 0), "KILL": (AB_, 0), "UNKNOWN_9": (U_, _RU)},
    "RELEASED": {**{c: (R_, _RS) for c in CMD_VARIANTS}, "KILL": (R_, 0), "UNKNOWN_9": (R_, _RU)},
    "ABORT": {**{c: (AB_, _RP) for c in CMD_VARIANTS}, "KILL": (AB_, 0), "UNKNOWN_9": (AB_, _RU)},
    "ARMED+pending": {**{c: (A_, _RP) for c in CMD_VARIANTS}, "KILL": (S_, 0), "UNKNOWN_9": (A_, _RU)},
    "UNPOWERED+pending": {**{c: (U_, _RP) for c in CMD_VARIANTS}, "KILL": (AB_, 0), "UNKNOWN_9": (U_, _RU)},
}


@pytest.mark.parametrize("start", list(TRANSITIONS))
def test_supervisor_every_command_in_every_state(veh, start):
    for name, (code_cmd, arg, good_nonce) in CMD_VARIANTS.items():
        h = _to(veh, start)
        nonce0 = h.sup.nonce
        pend0 = h.sup.release_deadline
        out = h.send(code_cmd, arg, nonce=nonce0 if good_nonce else nonce0 ^ 0x5A5A)
        want_state, want_code = TRANSITIONS[start][name]
        assert (out.state, h.last_code) == (want_state, want_code), (start, name)
        tl = out.telemetry
        assert out.reply_seq == h.seq and tl.ack_seq == h.seq                      # every processed SEQ is acknowledged
        assert tl.flag(lp.FLAG_REJECTED) == (want_code != 0)
        if want_code:
            assert tl.reason == want_code
        # the nonce moves only on an executed LAUNCH / THROTTLE-from-ARMED
        moved = start == "ARMED" and want_code == 0 and code_cmd in (lp.CMD_LAUNCH, lp.CMD_THROTTLE)
        assert h.sup.nonce == (lfsr16_next(nonce0) if moved else nonce0), (start, name)
        if start.endswith("pending") and name == "KILL" and want_state == State.ABORT:
            assert h.sup.release_deadline == pend0                                # the earlier release time is kept
        if name == "THROTTLE_500" and want_state == State.POWERED:
            assert h.sup.target_milli == 500


def test_supervisor_none_frames_and_sequence_rules(veh):
    h = _Sup(_sp(veh))
    out = h.tick([UplinkFrame(200, lp.Command(lp.CMD_ARM))])        # first frame after boot: new whatever its SEQ ...
    assert out.state == State.SAFE and h.last_code == lp.REJ_LINK     # ... but no earlier frame -> "link alive" fails
    assert out.telemetry.ack_seq == 200 and out.telemetry.flag(lp.FLAG_REJECTED)
    h.seq = 200
    out = h.hb()                                                       # duplicate SEQ: reply only
    assert out.reply_seq == 200 and len(h.sup.exec_log) == 1 and out.telemetry.flag(lp.FLAG_REJECTED)
    out = h.tick([UplinkFrame(201, lp.Command(lp.CMD_NONE))])          # newer NONE: adopts the SEQ, clears REJECTED
    assert out.telemetry.ack_seq == 201 and not out.telemetry.flag(lp.FLAG_REJECTED) and len(h.sup.exec_log) == 1
    h.seq = 201
    assert h.send(lp.CMD_ARM).state == State.ARMED                     # seq 202
    assert h.send(lp.CMD_DISARM, seq=202).state == State.ARMED         # repeat of 202: not executed again
    assert h.send(lp.CMD_DISARM, seq=150).state == State.ARMED         # older SEQ: ignored
    assert h.sup.last_seq == 202
    h.sup.last_seq = 250
    assert h.send(lp.CMD_DISARM, seq=3).state == State.SAFE            # wrap-around: 3 is newer than 250
    out = h.tick()
    assert out.reply_seq is None and out.telemetry is None             # no frame -> no reply (the ground is the master)


@pytest.mark.parametrize("kw, code", [
    (dict(arm_switch=False), lp.REJ_ARM_SWITCH), (dict(imu_ok=False), lp.REJ_SENSORS), (dict(baro_ok=False), lp.REJ_SENSORS),
    (dict(aligned=False), lp.REJ_NOT_ALIGNED), (dict(tilt_deg="max"), lp.REJ_TILT), (dict(v_cell="min-"), lp.REJ_VCELL),
    (dict(v_cell="min"), 0), (dict(tilt_deg="max-"), 0)])
def test_supervisor_arm_conditions(veh, kw, code):
    sp = _sp(veh)
    kw = dict(kw)
    if kw.get("tilt_deg") == "max":
        kw["tilt_deg"] = sp.arm_tilt_max_deg                   # tilt < arm_tilt_max_deg is required
    elif kw.get("tilt_deg") == "max-":
        kw["tilt_deg"] = sp.arm_tilt_max_deg - 1e-6
    if kw.get("v_cell") == "min-":
        kw["v_cell"] = sp.min_cell_v - 1e-6
    elif kw.get("v_cell") == "min":
        kw["v_cell"] = sp.min_cell_v                           # V_cell >= min_cell_v is accepted
    h = _Sup(sp)
    h.hb()
    out = h.send(lp.CMD_ARM, **kw)
    assert h.last_code == code and out.state == (State.ARMED if code == 0 else State.SAFE)


def test_supervisor_arm_needs_recent_link_and_no_open_release(veh):
    sp = _sp(veh)
    h = _Sup(sp)
    h.hb()
    h.run(sp.link_disarm_ticks)                                # silence >= link_loss_disarm_s
    h.send(lp.CMD_ARM)
    assert h.last_code == lp.REJ_LINK
    h.send(lp.CMD_ARM)
    assert h.last_code == 0 and h.sup.state == State.ARMED      # the previous frame is now recent
    h = _Sup(sp, reset_in_flight=True)                         # I10
    out = h.hb()
    assert out.state == State.SAFE and out.release and out.reason == lp.R_I10_RESET
    assert out.flags & lp.FLAG_INTERLOCK and out.flags & lp.FLAG_RELEASED
    h.send(lp.CMD_ARM)
    assert h.last_code == lp.REJ_RELEASED


def test_supervisor_nonce_lfsr(veh):
    s, seen = 1, set()
    for _ in range(65535):
        s = lfsr16_next(s)
        assert s != 0
        seen.add(s)
    assert s == 1 and len(seen) == 65535                       # maximal length
    assert lfsr16_next(0) == lfsr16_next(0xACE1)
    sp = _sp(veh)
    a, b = OnboardSupervisor(sp, seed=1234), OnboardSupervisor(sp, seed=1234)
    assert a.nonce == b.nonce == lfsr16_next(1234) and OnboardSupervisor(sp, seed=99).nonce != a.nonce
    h = _to(veh, "ARMED")
    n0 = h.sup.nonce
    h.send(lp.CMD_THROTTLE, 300)                               # THROTTLE from ARMED: nonce required and advanced
    assert h.sup.state == State.POWERED and h.sup.nonce == lfsr16_next(n0)
    h.send(lp.CMD_THROTTLE, 900, nonce=0)                       # in POWERED no nonce is needed, none consumed
    assert h.sup.target_milli == 900 and h.sup.nonce == lfsr16_next(n0)


def test_supervisor_throttle_slew_and_clamp(veh):
    sp = _sp(veh)
    h = _to(veh, "ARMED")
    outs = [h.send(lp.CMD_LAUNCH).throttle_milli] + [h.tick().throttle_milli for _ in range(sp.ramp_ticks + 3)]
    want = [min(sp.climb_milli, (i + 1) * 1000 // sp.ramp_ticks) for i in range(sp.ramp_ticks + 4)]
    assert outs == want and outs[sp.ramp_ticks - 1] == sp.climb_milli       # 0 -> climb in throttle_ramp_s
    assert h.send(lp.CMD_THROTTLE, 400).throttle_milli == 400              # decreases are immediate
    lvl = math.ceil(400 * sp.ramp_ticks / 1000)
    assert h.send(lp.CMD_THROTTLE, 1000).throttle_milli == (lvl + 1) * 1000 // sp.ramp_ticks   # increase slews from 400
    assert h.send(lp.CMD_CUT).throttle_milli == 0
    h2 = _to(veh, "ARMED", climb_milli=700)
    h2.send(lp.CMD_THROTTLE, 1500)
    assert h2.sup.target_milli == 700                                      # clamp(u, 0, climb_throttle)
    h2.run(sp.ramp_ticks + 2)
    assert h2.out.throttle_milli == 700 and h2.out.throttle == pytest.approx(0.7)


def test_supervisor_control_active_and_coast_hold(veh):
    sp = _sp(veh)
    h = _to(veh, "POWERED")
    assert h.out.control_active and not h.out.vanes_centre
    out = h.send(lp.CMD_CUT)
    assert out.state == State.UNPOWERED and out.control_active == (sp.coast_hold_ticks > 0)
    for i in range(1, sp.coast_hold_ticks + 2):
        out = h.tick() if i % 50 else h.hb()
        assert out.control_active == (i < sp.coast_hold_ticks), i
    assert out.vanes_centre
    for st in ("SAFE", "ARMED", "ABORT", "RELEASED"):
        assert not _to(veh, st).out.control_active


def test_supervisor_link_loss_interlocks_pad_and_flight(veh):
    sp = _sp(veh)
    for st, n, want, code in (("ARMED", sp.link_disarm_ticks, State.SAFE, lp.R_I1P_LINK_LOSS_PAD),
                              ("POWERED", sp.link_abort_ticks, State.ABORT, lp.R_I1_LINK_LOSS),
                              ("UNPOWERED", sp.link_abort_ticks, State.ABORT, lp.R_I1_LINK_LOSS)):
        h = _to(veh, st)
        last = h.k - 1                                          # the tick of the last frame
        h.run(n - 1 - (h.k - 1 - last))
        assert h.sup.state.name == st
        out = h.tick()
        assert h.k - 1 - last == n and out.state == want and out.reason == code and out.flags & lp.FLAG_INTERLOCK
        if want == State.ABORT:
            assert out.throttle == 0.0
            h.run(sp.abort_release_ticks - 1)
            assert h.sup.state == State.ABORT and not h.out.release
            out = h.tick()
            assert out.state == State.RELEASED and out.release
    h = _to(veh, "SAFE")
    assert h.run(3 * sp.link_abort_ticks).state == State.SAFE    # no link interlock in SAFE
    h = _to(veh, "RELEASED")
    assert h.run(3 * sp.link_abort_ticks).state == State.RELEASED


def test_supervisor_interlock_reason_outranks_rejections(veh):
    """SPEC 10.3: once an interlock has fired, a rejected command sets REJECTED but keeps the interlock code."""
    sp = _sp(veh)
    h = _to(veh, "ARMED")
    h.run(sp.link_disarm_ticks)                                  # I1p: ARMED -> SAFE
    assert h.sup.state == State.SAFE and h.sup.reason == lp.R_I1P_LINK_LOSS_PAD
    out = h.send(lp.CMD_LAUNCH, nonce=h.sup.nonce)                # the operator's LAUNCH arrives after the disarm
    assert h.last_code == lp.REJ_STATE
    tl = out.telemetry
    assert tl.flag(lp.FLAG_REJECTED) and tl.flag(lp.FLAG_INTERLOCK) and tl.reason == lp.R_I1P_LINK_LOSS_PAD
    # with no interlock latched the rejection code is reported
    h = _to(veh, "SAFE")
    out = h.send(lp.CMD_LAUNCH, nonce=h.sup.nonce)
    assert out.telemetry.reason == lp.REJ_STATE and not out.telemetry.flag(lp.FLAG_INTERLOCK)


def test_supervisor_tilt_rate_vcell_geofence_interlocks(veh):
    sp = _sp(veh)
    # I2: POWERED only, strictly above the threshold
    h = _to(veh, "POWERED")
    assert h.tick(tilt_deg=sp.tilt_abort_deg).state == State.POWERED
    out = h.tick(tilt_deg=sp.tilt_abort_deg + 0.01)
    assert out.state == State.ABORT and out.reason == lp.R_I2_TILT
    assert _to(veh, "UNPOWERED").tick(tilt_deg=80.0).state == State.UNPOWERED
    # I3: persistence counted in ticks, reset by a good tick; spin and saturation paths; disabled study flag
    for kw in (dict(gyro_rad_s=(sp.tilt_rate_abort_rad_s * 1.01, 0.0, 0.0)),
               dict(gyro_rad_s=(0.0, 0.0, -sp.spin_rate_abort_rad_s * 1.01)), dict(gyro_saturated=True)):
        h = _to(veh, "POWERED")
        h.run(sp.rate_persist_ticks - 1, **kw)
        h.tick()
        h.run(sp.rate_persist_ticks - 1, **kw)
        assert h.sup.state == State.POWERED
        out = h.tick(**kw)
        assert out.state == State.ABORT and out.reason == lp.R_I3_RATE, kw
    h = _to(veh, "POWERED", gyro_saturation_abort=False)
    assert h.run(3 * sp.rate_persist_ticks, gyro_saturated=True).state == State.POWERED
    h = _to(veh, "POWERED", rate_abort_enabled=False)
    assert h.run(3 * sp.rate_persist_ticks, gyro_rad_s=(10.0, 0.0, 30.0)).state == State.POWERED
    assert _to(veh, "UNPOWERED").run(3 * sp.rate_persist_ticks, gyro_rad_s=(10.0, 0.0, 30.0)).state == State.UNPOWERED
    # I8: POWERED only
    out = _to(veh, "POWERED").tick(v_cell=sp.min_cell_v - 0.01)
    assert out.state == State.ABORT and out.reason == lp.R_I8_VCELL
    assert _to(veh, "UNPOWERED").tick(v_cell=sp.min_cell_v - 0.5).state == State.UNPOWERED
    # I5: POWERED and UNPOWERED, strictly above
    for st in ("POWERED", "UNPOWERED"):
        h = _to(veh, st, apogee_limit_m=10 * sp.geofence_m)     # keep I4 out of the way
        assert h.tick(h_m=sp.geofence_m, vz_mps=-1.0).state.name == st
        out = h.tick(h_m=sp.geofence_m + 0.01, vz_mps=-1.0)
        assert out.state == State.ABORT and out.reason == lp.R_I5_GEOFENCE


def test_supervisor_apogee_limiter_and_motor_time(veh):
    sp = _sp(veh)
    v = 20.0
    h_lim = sp.apogee_limit_m - sp.apogee_k * v * v
    h = _to(veh, "POWERED")
    assert h.tick(h_m=h_lim - 0.01, vz_mps=v).state == State.POWERED
    out = h.tick(h_m=h_lim + 0.01, vz_mps=v)
    assert out.state == State.UNPOWERED and out.reason == lp.R_I4_APOGEE_LIMIT and not out.release   # CUT only
    assert out.control_active                                                   # the coast hold still applies
    assert _to(veh, "POWERED").tick(h_m=h_lim + 1.0, vz_mps=-v).state == State.POWERED   # falling: v_z^2 term off
    # I7: cumulative POWERED time
    sp7 = _sp(veh, max_motor_ticks=200)
    h = _to(veh, "POWERED", max_motor_ticks=200)
    h.run(99, hb_every=20)
    h.send(lp.CMD_CUT)
    h.run(50, hb_every=20)
    h.send(lp.CMD_THROTTLE, 800)                               # re-light in UNPOWERED
    assert h.sup.state == State.POWERED and h.sup.powered_ticks == 100 + 1
    n = 0
    while h.sup.state == State.POWERED:
        h.tick() if n % 20 else h.hb()
        n += 1
    assert h.sup.state == State.UNPOWERED and h.out.reason == lp.R_I7_MOTOR_TIME
    assert h.sup.powered_ticks == sp7.max_motor_ticks


def test_supervisor_fall_safe_release(veh):
    sp = _sp(veh)
    for st in ("POWERED", "UNPOWERED"):
        h = _to(veh, st)
        h.run(sp.fall_confirm_ticks - 1, vz_mps=sp.fall_vz_mps)
        h.tick(vz_mps=sp.fall_vz_mps + 0.1)                    # one slower tick restarts the count
        h.run(sp.fall_confirm_ticks - 1, vz_mps=sp.fall_vz_mps)
        assert h.sup.release_deadline is None
        out = h.tick(vz_mps=sp.fall_vz_mps)                     # v_z <= fall_deploy_vz_mps (inclusive)
        assert out.state == State.UNPOWERED and out.reason == lp.R_I6_FALL and out.throttle == 0.0
        t_rel = h.sup.release_deadline
        assert t_rel == h.k - 1 + sp.release_after_cut_ticks
        while h.sup.state != State.RELEASED:
            h.tick(vz_mps=-10.0)
        assert h.k - 1 == t_rel and h.out.release
    h = _to(veh, "ARMED")
    assert h.run(2 * sp.fall_confirm_ticks, hb_every=20, vz_mps=-20.0).state == State.ARMED   # pad: no I6


def test_supervisor_sensor_failure_and_arm_switch_pad_and_flight(veh):
    sp = _sp(veh)
    for key, n, code in (("imu_ok", sp.imu_fail_ticks, lp.R_I9_IMU), ("baro_ok", sp.baro_fail_ticks, lp.R_I9_BARO)):
        for st, want in (("ARMED", State.SAFE), ("POWERED", State.ABORT), ("UNPOWERED", State.ABORT)):
            h = _to(veh, st)
            h.run(n - 1, hb_every=20, **{key: False})
            h.hb()                                             # one good read resets the count
            h.run(n - 1, hb_every=20, **{key: False})
            assert h.sup.state.name == st
            out = h.hb(**{key: False})
            assert out.state == want and out.reason == code, (key, st)
            assert not out.flags & lp.FLAG_SENSORS_OK
    for st, want in (("ARMED", State.SAFE), ("POWERED", State.ABORT), ("UNPOWERED", State.ABORT)):
        out = _to(veh, st).hb(arm_switch=False)
        assert out.state == want and out.reason == lp.R_ARM_SWITCH_OPEN and not out.flags & lp.FLAG_ARM_SWITCH
    assert _to(veh, "SAFE").hb(arm_switch=False).state == State.SAFE


def test_supervisor_release_paths_and_interlock_last_word(veh):
    sp = _sp(veh)
    # RELEASE on the pad opens after release_after_cut_s; KILL before that cancels it
    h = _to(veh, "ARMED+pending")
    t0 = h.k - 1
    h.run(sp.release_after_cut_ticks, hb_every=20)
    assert h.sup.state == State.RELEASED and h.k - 1 == t0 + sp.release_after_cut_ticks
    h = _to(veh, "ARMED+pending")
    h.send(lp.CMD_KILL)
    assert h.run(2 * sp.release_after_cut_ticks, hb_every=20).state == State.SAFE and not h.out.release
    # ABORT keeps an earlier pending release time
    h = _to(veh, "UNPOWERED+pending")
    t_rel = h.sup.release_deadline
    h.run(10, hb_every=5)
    h.send(lp.CMD_KILL)
    assert h.sup.state == State.ABORT and h.sup.release_deadline == t_rel
    while not h.out.release:
        h.hb()
    assert h.k - 1 == t_rel
    # an interlock acts after the commands of the same tick
    h = _to(veh, "POWERED")
    out = h.send(lp.CMD_THROTTLE, 600, tilt_deg=sp.tilt_abort_deg + 5.0)
    assert h.last_code == 0 and out.state == State.ABORT and out.reason == lp.R_I2_TILT


def test_supervisor_telemetry_fields(veh):
    sp = _sp(veh)
    h = _to(veh, "POWERED")
    out = h.hb(h_m=12.34, vz_mps=-1.5, roll_deg=1.0, pitch_deg=-2.0, yaw_deg=170.0, tilt_deg=2.25, v_cell=3.95,
               loop_max_us=2500.0)
    t = out.telemetry
    assert t.state == lp.STATE_POWERED and t.t_ms == (h.k - 1) * 1000 // sp.loop_hz and t.nonce == h.sup.nonce
    assert (t.h_dm, t.vz_cms, t.roll_cdeg, t.pitch_cdeg, t.yaw_cdeg, t.tilt_cdeg) == (123, -150, 100, -200, 17000, 225)
    assert (t.vcell_mv, t.loop_max_us, t.throttle_milli) == (3950, 2500, out.throttle_milli)
    want = lp.FLAG_ARM_SWITCH | lp.FLAG_SENSORS_OK | lp.FLAG_ALIGNED | lp.FLAG_LINK_OK | lp.FLAG_CONTROL_ACTIVE
    assert t.flags == want and t.reason == lp.REASON_NONE
    b = lp.encode_telemetry(90, out.reply_seq, t)
    assert lp.decode_telemetry(lp.FrameDecoder(90).feed(b)[0]) == t


def test_supervisor_ticks_from_params(veh):
    sp = _sp(veh)
    m, hz = veh.params["mission"], int(veh.params["control"]["loop_hz"])
    for key, sec in (("link_abort_ticks", m["link_loss_abort_s"]), ("link_disarm_ticks", m["link_loss_disarm_s"]),
                     ("ramp_ticks", m["throttle_ramp_s"]), ("fall_confirm_ticks", m["fall_deploy_confirm_s"]),
                     ("max_motor_ticks", m["max_motor_time_s"]), ("abort_release_ticks", m["abort_release_delay_s"]),
                     ("release_after_cut_ticks", m["release_after_cut_s"]), ("coast_hold_ticks", m["coast_control_hold_s"])):
        n = getattr(sp, key)
        assert n / hz >= sec - 1e-12 and (n - 1) / hz < sec, key
    assert ticks_ge(0.3, 250) == 75 and ticks_ge(0.02, 250) == 5 and ticks_ge(0.0041, 250) == 2
    assert sp.apogee_limit_m == m["apogee_limit_m"] and sp.geofence_m == m["geofence_altitude_m"]
    assert sp.climb_milli == round(1000 * m["climb_throttle"])
    assert _sp(veh, geofence_altitude_m=270.0, apogee_limit_m=260.0).geofence_m == 270.0


# =========================================================================== #
# v0.4 ground station logic (sim.ground_station_logic)
# =========================================================================== #
def _gp(**kw) -> GroundParams:
    d = dict(vehicle_id=90, poll_hz=4.0, reply_timeout_s=0.5, t_link_default_s=0.4, target_altitude_m=140.0,
             coast_drag_factor=0.85, apogee_detect_vz_mps=0.0, apogee_confirm_s=0.3, coast_timeout_s=6.0,
             release_after_cut_s=0.2, release_compensated=False)
    d.update(kw)
    return GroundParams(**d)


def _tlm_bytes(seq: int, state: int = lp.STATE_SAFE, ack: int = 0, t_ms: int = 0, h: float = 0.0, vz: float = 0.0,
               flags: int = 0, reason: int = 0, nonce: int = 0x1111) -> bytes:
    t = lp.Telemetry.from_physical(state=state, flags=flags, reason=reason, ack_seq=ack, nonce=nonce, t_ms=t_ms, h_m=h,
                                   vz_mps=vz, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, tilt_deg=0.0, throttle=0.0,
                                   vcell_v=4.0, loop_max_us=0.0)
    return lp.encode_telemetry(90, seq, t)


def test_telem_v6_v7_logic_battery_tail(veh):
    """2026-11-10 control round: TELEM v6 (0x25) / v7 (0x26) = TELEM v2 / v3 + lbat u8 (0.02 V); the tail rule
    (non-zero snapshot byte, no power tail), encode / decode round trips, the old frames unchanged byte for byte,
    LEN_MAX unchanged, and the ground's display / warning (warning only)."""
    assert (lp.TYPE_TELEM6, lp.TYPE_TELEM7) == (0x25, 0x26)
    assert (lp.TELEM6_PAYLOAD_LEN, lp.TELEM7_PAYLOAD_LEN, lp.TELEM6_FRAME_BYTES, lp.TELEM7_FRAME_BYTES) == (32, 38, 40, 46)
    assert lp.LEN_MAX == 42 and lp.MAX_PAYLOAD_LEN == lp.TELEM5_PAYLOAD_LEN       # TELEM v5 stays the longest
    assert len(set(lp.PAYLOAD_LEN.values())) == len(lp.PAYLOAD_LEN)               # one length per TYPE (decoder rule)
    assert lp.lbat_quant(4.2) == 210 and lp.lbat_quant(3.0) == 150 and lp.lbat_quant(0.0) == 1 and lp.lbat_quant(9.9) == 255
    assert lp.lbat_quant(None) == 1 and lp.lbat_quant(float("nan")) == 1 and lp.lbat_quant(3.61) == 181
    assert lp.logic_tail_due(181) and not lp.logic_tail_due(lp.LBAT_NONE) and not lp.logic_tail_due(181, 0x40)
    kw = dict(state=lp.STATE_POWERED, flags=0x2F, reason=0, ack_seq=4, nonce=0x1234, t_ms=5000, h_m=1.5, vz_mps=0.1,
              roll_deg=1.0, pitch_deg=-1.0, yaw_deg=3.0, tilt_deg=1.4, throttle=0.6, vcell_v=3.9, loop_max_us=2800.0)
    t2 = lp.Telemetry.from_physical(**kw, phase=lp.PHASE_HOLD, h_tgt_m=1.5)
    t3 = lp.Telemetry.from_physical(**kw, phase=lp.PHASE_HOLD, h_tgt_m=1.5, gps=(0x2F, 9, 12, 2, 0.4))
    t6 = lp.Telemetry.from_physical(**kw, phase=lp.PHASE_HOLD, h_tgt_m=1.5, lbat_q=193)
    t7 = lp.with_logic_tail(t3, 193)
    assert t6 == lp.with_logic_tail(t2, 193) and t6.lbat_v == pytest.approx(3.86) and t6.has_lbat and not t2.has_lbat
    assert (t6.wire_type, t7.wire_type) == (lp.TYPE_TELEM6, lp.TYPE_TELEM7)
    assert t6.pack() == t2.pack() + bytes([193]) and t7.pack() == t3.pack() + bytes([193])
    assert lp.with_logic_tail(t2, lp.LBAT_NONE) is t2                          # no monitor: TELEM v2 unchanged
    t1 = lp.Telemetry.from_physical(**kw)
    assert lp.with_logic_tail(t1, 193) is t1                                   # the ballistic TELEM v1 never carries it
    t5 = lp.with_power_tail(t3, 190, 0x80 | 0x01)
    assert lp.with_logic_tail(t5, 193) is t5                                   # the power builds keep TELEM v5
    with pytest.raises(ValueError):
        lp.replace(t5, lbat_q=193)
    with pytest.raises(ValueError):
        lp.replace(t2, lbat_q=0)                                               # LBAT_NONE never travels
    dec = lp.FrameDecoder(90, lp.TELEM_TYPES)
    frames = dec.feed(lp.encode_telemetry(90, 7, t6) + lp.encode_telemetry(90, 8, t7) + lp.encode_telemetry(90, 9, t3))
    assert [lp.decode_telemetry(f) for f in frames] == [t6, t7, t3] and [f.type for f in frames] == [0x25, 0x26, 0x22]
    old = lp.FrameDecoder(90, lp.TELEM_TYPES_V09)                              # a v0.9 ground: wrong_type, nothing else
    assert old.feed(lp.encode_telemetry(90, 7, t6)) == [] and old.stats["wrong_type"] == 1
    # the ground: show it, warn below the params threshold (A) once, clear with hysteresis; nothing is sent for it
    gp = GroundParams.from_params(veh.params, reply_timeout_s=0.25, t_link_default_s=0.06, mode=lp.MODE_VTOL)
    w = float(veh.params["flight_computer"]["logic_battery_monitor"]["ground_warn_below_v_ASSUMED"])
    assert gp.lbat_warn_v == w == 3.6
    gs = GroundAssist(gp)
    for i, v in enumerate((3.90, 3.58, 3.56, 3.62, 3.70, 3.50)):
        tl = lp.Telemetry.from_physical(**{**kw, "t_ms": 1000 * (i + 1)}, phase=lp.PHASE_HOLD, h_tgt_m=1.5,
                                        lbat_q=lp.lbat_quant(v))
        gs.receive(lp.encode_telemetry(90, 0, tl), float(i))
        assert gs.lbat_v == pytest.approx(v)
    ev = [e for _, e in gs.events if "LOGIC BATTERY LOW" in e or "logic battery" in e]
    assert len([e for e in ev if "LOW" in e]) == 2 and any("back above 3.65" in e for e in ev)
    assert gs.lbat_low and gs.vtol_status()["lbat_low"] and "로직 배터리 3.50 V" in gs.vtol_status()["lbat_ko"]
    assert gs.queue == type(gs.queue)() and gs.pending is None and not gs.records       # a warning only


def _up(frame: bytes) -> tuple[int, lp.Command]:
    (fr,) = lp.FrameDecoder(90, (lp.TYPE_CMD,)).feed(frame)
    return fr.seq, lp.decode_command(fr)


def test_ground_sync_repeat_until_ack_and_timeouts():
    gs = GroundAssist(_gp())
    gs.arm(0.0)
    seq, c = _up(gs.poll(0.0))
    assert (seq, c.cmd) == (0, lp.CMD_NONE)                  # nothing but heartbeats before the first telemetry
    assert gs.poll(0.3) is None                               # reply outstanding, deadline not reached
    gs.receive(_tlm_bytes(0, ack=41, t_ms=100), 0.42)
    assert gs.synced and gs.rtt_all[-1][1] == pytest.approx(0.42)
    seq, c = _up(gs.poll(0.42))                              # fresh command goes out at once
    assert (seq, c.cmd) == (42, lp.CMD_ARM)
    assert gs.poll(0.9) is None and gs.poll(0.92) is not None    # timeout 0.5 s -> repeated, same SEQ
    assert gs.stats["timeouts"] == 1 and gs.pending.n_tx == 2 and gs.pending.seq == 42
    gs.receive(_tlm_bytes(42, state=lp.STATE_SAFE, ack=42, t_ms=600, flags=lp.FLAG_REJECTED, reason=lp.REJ_TILT), 1.3)
    rec = gs.records[0]
    assert rec.result == "rejected: tilt" and rec.t_ack == 1.3 and gs.pending is None
    seq, c = _up(gs.poll(1.3 + 0.25))                        # heartbeat repeats the last SEQ
    assert (seq, c.cmd) == (42, lp.CMD_NONE)
    assert gs.poll(1.3 + 0.25 + 0.01) is None


def test_ground_queue_priority_renumber_and_nonce():
    gs = GroundAssist(_gp())
    gs.poll(0.0)
    gs.receive(_tlm_bytes(0, state=lp.STATE_ARMED, ack=7, t_ms=10, nonce=0xABCD), 0.4)
    gs.launch(0.4)
    gs.throttle(0.5, 0.4)
    seq, c = _up(gs.poll(0.4))
    assert (seq, c.cmd, c.nonce) == (8, lp.CMD_LAUNCH, 0xABCD)      # LAUNCH carries the latest telemetry nonce
    gs.cut(0.5)                                                      # pre-empts: LAUNCH and THROTTLE superseded
    assert [r.result for r in gs.records] == ["superseded", "superseded", ""] and gs.pending.name == "CUT"
    gs.receive(_tlm_bytes(8, state=lp.STATE_POWERED, ack=8, t_ms=500), 0.8)   # ack for the LAUNCH SEQ: not the CUT
    assert gs.pending.name == "CUT" and gs.pending.seq is None
    seq, c = _up(gs.poll(0.8))
    assert (seq, c.cmd) == (9, lp.CMD_CUT)
    gs.receive(_tlm_bytes(9, state=lp.STATE_POWERED, ack=20, t_ms=900), 1.2)  # vehicle already past SEQ 9 (e.g. reboot)
    assert gs.pending.seq is None                                    # renumbered: the vehicle could never run SEQ 9
    seq, c = _up(gs.poll(1.45))
    assert (seq, c.cmd) == (21, lp.CMD_CUT)


def test_ground_rtt_estimate_and_stale_reply():
    gs = GroundAssist(_gp(rtt_window=2))
    assert gs.t_link == 0.4                                          # default before any measurement
    gs.poll(0.0)
    gs.receive(_tlm_bytes(0, ack=0, t_ms=10), 0.3)
    gs.poll(0.3)
    gs.receive(_tlm_bytes(0, ack=0, t_ms=20), 0.8)
    # 2026-11-10 R-ESP1: max(mean of 0.3 and 0.5 = 0.4, latest 0.5) - a rise counts at once (v0.4..v0.9: the mean 0.4)
    assert gs.t_link == pytest.approx(0.5)
    gs.poll(1.0)
    gs.receive(_tlm_bytes(5, ack=0, t_ms=30), 1.2)                   # header SEQ != request SEQ: no RTT sample
    assert gs.stats["stale_replies"] == 1 and len(gs.rtt_all) == 2
    gs.receive(_tlm_bytes(0, ack=0, t_ms=25), 1.3)                   # older vehicle time: data ignored
    assert gs.latest.t_ms == 30
    assert gs.rtt_all[-1][1] == pytest.approx(0.3)                   # (its SEQ answered the 1.0 s request: RTT 0.3)
    assert gs.t_link == pytest.approx(0.4)                           # a fall: the mean of 0.5 and 0.3 (> latest 0.3)


def test_ground_late_reply_is_a_round_trip_sample():
    """2026-11-10 R-ESP1 (a): a reply after its request's timeout but before the next request is a round-trip sample
    ("late"), so t_link follows a link whose delay rose above the reply timeout (an ESP-timed ground on a slower link);
    a reply that could belong to an older unanswered request with the same SEQ is not measured ("stale")."""
    gs = GroundAssist(_gp(rtt_window=8, reply_timeout_s=0.25, t_link_default_s=0.06, poll_hz=2.0))
    assert gs.poll(0.0) is not None
    gs.receive(_tlm_bytes(0, ack=0, t_ms=10), 0.06)                  # on time: RTT 0.06
    assert gs.t_link == pytest.approx(0.06)
    assert gs.poll(0.5) is not None
    assert gs.poll(0.75) is None and gs.stats["timeouts"] == 1      # no reply within 0.25 s: a timeout
    gs.receive(_tlm_bytes(0, ack=0, t_ms=20), 0.99)                  # ... the reply at 0.49 s: late, measured
    assert gs.stats["late_replies"] == 1 and gs.stats["stale_replies"] == 0
    assert gs.t_link == pytest.approx(0.49)                          # max(mean 0.275, latest 0.49): at once
    assert any("late reply" in e for _, e in gs.events)
    assert gs.reply_timeout() == pytest.approx(0.49 + 0.04)          # (c) the next request waits for such a reply
    gs.receive(_tlm_bytes(0, ack=0, t_ms=21), 1.0)                   # a second copy: already answered -> stale
    assert gs.stats["stale_replies"] == 1 and gs.stats["late_replies"] == 1
    # an unanswered request, then the next heartbeat with the same SEQ: its late reply is ambiguous (not measured)
    g3 = GroundAssist(_gp(rtt_window=8, reply_timeout_s=0.25, t_link_default_s=0.06, poll_hz=2.0))
    assert g3.poll(0.0) is not None and g3.poll(0.25) is None        # no reply: a timeout
    assert g3.poll(0.5) is not None and g3.poll(0.75) is None        # the next heartbeat (same SEQ), no reply in time
    g3.receive(_tlm_bytes(0, ack=0, t_ms=30), 0.9)                   # could be the 0.0 s request's: not measured
    assert not g3.rtt_all and g3.stats["stale_replies"] == 1 and g3.stats["late_replies"] == 0
    assert g3.t_link == 0.06 and g3.reply_timeout() == 0.25
    # the adaptive timeout is limited, and never below the configured one
    g2 = GroundAssist(_gp(reply_timeout_s=0.25, poll_hz=2.0))
    g2.rtts.append(3.0)
    assert g2.reply_timeout() == pytest.approx(1.0)
    g2.rtts.clear()
    g2.rtts.append(0.05)
    assert g2.reply_timeout() == 0.25


def test_ground_assist_auto_cut_rule():
    """SPEC 8.8 v0.4: CUT when P + max(dP/dt, 0) T_tel / 2 >= target, P = h + v_z t_link + k max(v_z, 0)^2 / (2 g)."""
    gp = _gp()
    gs = GroundAssist(gp)
    gs.poll(0.0)
    vz, dh = 20.0, 10.0                                        # 10 m of height per telemetry second -> dP/dt = 10 m/s
    gs.receive(_tlm_bytes(0, state=lp.STATE_ARMED, ack=0, t_ms=10), 0.4)      # sync + one round-trip sample
    tl, t_tel = gs.t_link, gs.t_tel()
    assert t_tel == max(1.0 / gp.poll_hz, tl)
    lead = 0.5 * dh * t_tel
    p0 = gs.predicted_apogee(0.0, vz, tl)
    assert lead > 1.5
    h1 = round(gp.target_altitude_m - p0 - 2 * dh - 1.1, 1)     # P3 = target - 1.1 m: only P3 + lead reaches the target
    gs.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=1000, h=h1, vz=vz), 0.5)
    gs.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=2000, h=round(h1 + dh, 1), vz=vz), 0.6)
    assert not gs.cut_queued
    gs.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=3000, h=round(h1 + 2 * dh, 1), vz=vz), 0.7)
    assert gs.cut_queued and gs.pending.name == "CUT" and gs.pending.source == "assist"
    c = gs.assist_log[0]
    assert c["rate_mps"] == pytest.approx(dh, abs=1e-6) and c["lead_m"] == pytest.approx(lead, abs=1e-6)
    assert c["predicted_m"] < gp.target_altitude_m <= c["predicted_m"] + c["lead_m"]     # the lead made the decision
    gs.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=4000, h=200.0, vz=vz), 0.8)
    assert sum(1 for r in gs.records if r.name == "CUT") == 1       # once per flight
    # no rate yet (first POWERED telemetry): no lead
    g2 = GroundAssist(gp)
    g2.poll(0.0)
    hc = round(gp.target_altitude_m - g2.predicted_apogee(0.0, vz, g2.t_link) + 0.05, 1)
    g2.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=10, h=hc, vz=vz), 0.4)
    assert g2.cut_queued and g2.assist_log[0]["lead_m"] == 0.0
    off = GroundAssist(gp, assist=False)
    off.poll(0.0)
    off.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=10, h=300.0, vz=vz), 0.4)
    assert not off.records


def test_ground_assist_auto_release_rules():
    gp = _gp()
    gs = GroundAssist(gp)
    gs.poll(0.0)
    t = 0.0

    def feed(state, t_ms, vz):
        nonlocal t
        t += 0.1
        gs.receive(_tlm_bytes(0, state=state, ack=0, t_ms=t_ms, h=100.0, vz=vz), t)

    feed(lp.STATE_POWERED, 1000, 20.0)
    gs.cut_queued = True                                       # isolate the release rule
    feed(lp.STATE_UNPOWERED, 2000, 5.0)
    feed(lp.STATE_UNPOWERED, 3000, -0.1)                       # v_z <= 0 starts the confirmation window
    feed(lp.STATE_UNPOWERED, 3200, 0.2)                        # a positive sample resets it
    feed(lp.STATE_UNPOWERED, 3400, -0.1)
    feed(lp.STATE_UNPOWERED, 3699, -1.0)
    assert not gs.release_queued                               # 299 ms < apogee_confirm_s
    feed(lp.STATE_UNPOWERED, 3700, -1.5)
    assert gs.release_queued and gs.pending.name == "RELEASE" and gs.assist_log[-1]["why"] == "apogee"
    # coast timeout after the first UNPOWERED telemetry
    gs2 = GroundAssist(gp)
    gs2.poll(0.0)
    gs2.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=10, h=50.0, vz=10.0), 0.1)
    gs2.cut_queued = True
    gs2.receive(_tlm_bytes(0, state=lp.STATE_UNPOWERED, ack=0, t_ms=1000, h=60.0, vz=8.0), 0.2)
    gs2.receive(_tlm_bytes(0, state=lp.STATE_UNPOWERED, ack=0, t_ms=6999, h=60.0, vz=8.0), 0.3)
    assert not gs2.release_queued
    gs2.receive(_tlm_bytes(0, state=lp.STATE_UNPOWERED, ack=0, t_ms=7000, h=60.0, vz=8.0), 0.4)
    assert gs2.release_queued and gs2.assist_log[-1]["why"] == "coast_timeout"
    # SPEC 8.8 v0.4 default: delay-compensated release on the predicted v_z
    gs3 = GroundAssist(_gp(release_compensated=True))
    gs3.poll(0.0)
    gs3.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=10, h=50.0, vz=10.0), 0.1)
    gs3.cut_queued = True
    lead = gs3.t_link + 0.2 + 0.5 * max(0.25, gs3.t_link)
    gs3.receive(_tlm_bytes(0, state=lp.STATE_UNPOWERED, ack=0, t_ms=500, h=60.0, vz=round(gs3.gp.g * lead + 0.05, 2)), 0.2)
    assert not gs3.release_queued
    gs3.receive(_tlm_bytes(0, state=lp.STATE_UNPOWERED, ack=0, t_ms=600, h=60.0, vz=round(gs3.gp.g * lead - 0.05, 2)), 0.3)
    assert gs3.release_queued


def test_ground_assist_rules_at_a_reply_timeout():
    """v0.6.3: a lost exchange is a decision point - at the reply timeout the CUT rule runs on P + dP/dt x age and
    the predictive RELEASE rule on v_z - g x age (age = now - arrival of the latest telemetry); the CUT goes out at
    once.  Nothing is extrapolated without a measured dP/dt, after the CUT, or with the literal release rule."""
    gp = _gp(poll_hz=2.0, reply_timeout_s=0.3, release_compensated=True)
    gs = GroundAssist(gp)
    gs.poll(0.0)
    vz, dh = 20.0, 10.0
    gs.receive(_tlm_bytes(0, state=lp.STATE_ARMED, ack=0, t_ms=10), 0.2)       # sync, t_link = 0.2 s
    tl, t_tel = gs.t_link, gs.t_tel()
    p0 = gs.predicted_apogee(0.0, vz, tl)
    h1 = round(gp.target_altitude_m - p0 - dh - 8.0, 1)                       # P2 = target - 8 m: no CUT on telemetry
    gs.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=1000, h=h1, vz=vz), 0.5)
    gs.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=2000, h=round(h1 + dh, 1), vz=vz), 1.0)
    assert not gs.cut_queued and gs.cut_rate == pytest.approx(dh, abs=1e-6)
    lead = 0.5 * dh * t_tel
    p2 = gs.cut_prev[1]
    age_cut = (gp.target_altitude_m - lead - p2) / dh                          # extrapolated P + lead reaches the target
    assert gs.poll(1.0) is not None                                            # the next request (lost)
    assert gs.poll(1.0 + 0.299) is None and not gs.cut_queued                  # reply outstanding
    up = gs.poll(1.0 + 0.3)                                                    # timeout, age 0.3 s < age_cut
    assert gs.stats["timeouts"] == 1 and not gs.cut_queued and up is None      # (next poll slot not reached)
    assert 0.3 < age_cut < 1.3
    gs.poll(1.5)                                                               # heartbeat at the slot, lost again
    seq, c = _up(gs.poll(1.5 + 0.3))                                           # second timeout: age 0.8 s
    assert 0.8 >= age_cut and gs.cut_queued and c.cmd == lp.CMD_CUT          # CUT sent at the timeout itself
    a = gs.assist_log[-1]
    assert a["extrapolated"] and a["age_s"] == pytest.approx(0.8)
    assert a["predicted_m"] == pytest.approx(p2 + dh * 0.8) and a["lead_m"] == pytest.approx(lead)
    assert a["predicted_at_telemetry_m"] == pytest.approx(p2)
    # no extrapolation without a measured rate
    g2 = GroundAssist(gp)
    g2.poll(0.0)
    g2.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=10, h=100.0, vz=vz), 0.2)
    assert not g2.cut_queued and g2.cut_prev[1] < gp.target_altitude_m
    g2.poll(0.5)
    g2.poll(0.5 + 0.3 + 5.0)                                                   # very old, but no dP/dt
    assert not g2.cut_queued
    # predictive RELEASE at a timeout: v_z - g (age + lead) <= apogee_detect_vz
    g3 = GroundAssist(gp)
    g3.poll(0.0)
    g3.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=10, h=50.0, vz=10.0), 0.2)
    g3.cut_queued = True
    lead_r = g3.t_link + gp.release_after_cut_s + 0.5 * g3.t_tel()
    v1 = round(gp.g * (lead_r + 0.45), 2)                                      # crosses after 0.45 s more
    g3.poll(0.2)
    g3.receive(_tlm_bytes(0, state=lp.STATE_UNPOWERED, ack=0, t_ms=500, h=60.0, vz=v1), 0.3)
    assert not g3.release_queued
    g3.poll(0.7)                                                               # lost
    up = g3.poll(0.7 + 0.3)                                                    # timeout: age 0.7 s >= 0.45 s
    assert g3.release_queued and _up(up)[1].cmd == lp.CMD_RELEASE
    r = g3.assist_log[-1]
    assert r["extrapolated"] and r["why"] == "apogee" and r["vz_mps"] - gp.g * (r["age_s"] + lead_r) <= 0.0
    # the literal rule never extrapolates
    g4 = GroundAssist(_gp(poll_hz=2.0, reply_timeout_s=0.3, release_compensated=False))
    g4.poll(0.0)
    g4.receive(_tlm_bytes(0, state=lp.STATE_POWERED, ack=0, t_ms=10, h=50.0, vz=10.0), 0.2)
    g4.cut_queued = True
    g4.poll(0.2)
    g4.receive(_tlm_bytes(0, state=lp.STATE_UNPOWERED, ack=0, t_ms=500, h=60.0, vz=1.0), 0.3)
    g4.poll(0.7)
    g4.poll(0.7 + 0.3 + 3.0)
    assert not g4.release_queued


def test_ground_release_rule_from_params(veh):
    """mission.assist_release_rule selects the assist RELEASE rule; the shipped params use the predictive one."""
    import copy
    p = veh.params
    assert p["mission"]["assist_release_rule"] == "predictive"
    assert GroundParams.from_params(p, reply_timeout_s=0.5, t_link_default_s=0.4).release_compensated is True
    q = copy.deepcopy(p)
    q["mission"]["assist_release_rule"] = "literal"
    assert GroundParams.from_params(q, reply_timeout_s=0.5, t_link_default_s=0.4).release_compensated is False
    q["mission"]["assist_release_rule"] = "early"
    with pytest.raises(ValueError):
        GroundParams.from_params(q, reply_timeout_s=0.5, t_link_default_s=0.4)


def test_ground_and_supervisor_closed_loop_without_physics(veh):
    """Ground + link model + supervisor: ARM, then LAUNCH with the telemetry nonce, then CUT and RELEASE."""
    p = veh.params
    cfg = lm.LinkConfig.from_params(p, loss=0.0)
    rt = lm.round_trip(cfg)
    gs = GroundAssist(GroundParams.from_params(p, rt["reply_timeout_s"], rt["rtt_mean_s"], mode=lp.MODE_BALLISTIC),
                      assist=False)
    sp = SupervisorParams.from_params(p, mode=lp.MODE_BALLISTIC)
    sup = OnboardSupervisor(sp, seed=77)
    link = lm.LinkModel(cfg, seed=1)
    dec = lp.FrameDecoder(int(p["flight_computer"]["link"]["vehicle_id"]), (lp.TYPE_CMD,))
    dt = 1.0 / sp.loop_hz
    plan = {0.5: "ARM", 1.5: "LAUNCH", 3.0: "CUT", 4.0: "RELEASE"}
    done = set()
    for k in range(int(6.0 / dt)):
        t = k * dt
        for tr in link.deliver("down", t):
            gs.receive(tr.data, tr.t_arrival)
        for t_c, name in plan.items():
            if t >= t_c and name not in done:
                done.add(name)
                getattr(gs, name.lower())(t)
        up = gs.poll(t)
        if up:
            link.send("up", up, t)
        rx = b"".join(tr.data for tr in link.deliver("up", t))
        frames = [UplinkFrame.from_frame(f) for f in dec.feed(rx)] if rx else []
        out = sup.update(OnboardInputs(tick=k, frames=frames, v_cell=4.1))
        if out.reply_seq is not None:
            link.send("down", lp.encode_telemetry(90, out.reply_seq, out.telemetry), t + dt)
    assert [r.result for r in gs.records] == ["accepted"] * 4
    assert [e["code"] for e in sup.exec_log] == [0, 0, 0, 0] and sup.state == State.RELEASED
    assert link.stats["collision"] == 0 and gs.stats["timeouts"] == 0
    for r in gs.records:                                       # decision -> ack = at most two round trips
        assert r.t_ack - r.t_queued <= 2 * rt["rtt_worst_s"] + 2 * dt


# =========================================================================== #
# v0.4 link model (sim.link_model)
# =========================================================================== #
@pytest.mark.parametrize("pl, sf, bw, cr, pre, ih, crc, n_sym, toa_ms", [
    (13, 7, 125e3, 5, 8, False, True, 33, 46.336),       # (8+4.25) 1.024 + 33 x 1.024 ms
    (36, 7, 125e3, 5, 8, False, True, 63, 77.056),
    (13, 12, 125e3, 5, 8, False, True, 23, 1155.072),    # T_sym 32.768 ms > 16 ms -> LDRO on: 8 + ceil(100/40) 5
    (36, 9, 250e3, 8, 8, True, False, 72, 172.544),      # implicit header, no CRC: 8 + ceil(260/36) 8; T_sym 2.048 ms
    (0, 7, 125e3, 5, 6, False, False, 8, 18.688),        # empty payload: 8 + max(ceil(0/28), 0); (6+4.25+8) x 1.024
])
def test_lora_time_on_air_hand_values(pl, sf, bw, cr, pre, ih, crc, n_sym, toa_ms):
    t_sym = 2 ** sf / bw
    ldro = t_sym > 0.016
    assert lm.lora_symbols(pl, sf, cr, explicit_header=not ih, crc=crc, ldro=ldro) == n_sym
    assert lm.lora_time_on_air(pl, sf, bw, cr, pre, explicit_header=not ih, crc=crc) * 1e3 == pytest.approx(toa_ms, abs=1e-9)


def test_link_hop_timing_hand_values(veh):
    p = veh.params
    lk = p["flight_computer"]["link"]
    cfg = lm.LinkConfig.from_params(p, radio="lora_uart")
    baud, air = float(lk["uart_baud"]), float(lk["lora_uart_air_rate_bps"])
    up = (lm.GROUND_SERIAL_LATENCY_S_ASSUMED + 13 * 10 / baud + 3 * 10 / baud + lm.EBYTE_MODULE_PROCESS_S_ASSUMED
          + (13 * 8 + lm.EBYTE_AIR_OVERHEAD_BITS_ASSUMED) / air + 13 * 10 / baud)
    down = (36 * 10 / baud + 3 * 10 / baud + lm.EBYTE_MODULE_PROCESS_S_ASSUMED + (36 * 8 + lm.EBYTE_AIR_OVERHEAD_BITS_ASSUMED) / air
            + 36 * 10 / baud + lm.GROUND_SERIAL_LATENCY_S_ASSUMED)
    assert lm.hop_timing(cfg, "up", 13).arrival == pytest.approx(up, rel=1e-12)
    assert lm.hop_timing(cfg, "down", 36).arrival == pytest.approx(down, rel=1e-12)
    rt = lm.round_trip(cfg)
    # v0.6.3 three-chip co-simulation: uplink at the COM -> TELEM handed to the radio, normal max 13.135 ms (sx127x),
    # 12.649 ms (lora_uart), rounded up to the next ms; the COM -> FCU forwarding max 0.517 ms
    assert lm.FC_REPLY_DELAY_S == 0.014 and lm.COM_FORWARD_S == 0.0006
    assert rt["rtt_worst_s"] == pytest.approx(up + lm.FC_REPLY_DELAY_S + down)
    assert rt["rtt_mean_s"] == rt["rtt_worst_s"]                    # one conservative constant: no tick-phase term
    # the timeout waits for the latest reply the COM may send (its 80 ms reply window was derived from this allowance:
    # h + start <= allowance + margin = 108 ms), not for the typical one
    assert lm.FC_REPLY_ALLOWANCE_S == 0.068
    assert rt["reply_timeout_s"] == pytest.approx(up + lm.FC_REPLY_ALLOWANCE_S + down + lm.REPLY_TIMEOUT_MARGIN_S_ASSUMED)
    assert rt["reply_timeout_s"] - (up + down) >= 0.080 + 0.0044 + 0.02          # COM window + sx127x FIFO load + 20 ms
    assert rt["reply_timeout_s"] - rt["rtt_worst_s"] >= 0.0435 - lm.FC_REPLY_DELAY_S   # a 43.5 ms burst reply is not a timeout
    b = lm.budget(cfg)
    assert b["fc_reply_delay_s"] == lm.FC_REPLY_DELAY_S and b["measured"]["COM_FORWARD_S"] == lm.COM_FORWARD_S
    assert b["fc_reply_allowance_s"] == lm.FC_REPLY_ALLOWANCE_S
    sx = lm.LinkConfig.from_params(p, radio="sx127x")
    h = lm.hop_timing(sx, "up", 13)
    assert h.air_end - h.air_start == pytest.approx(lm.lora_time_on_air(13, sx.sf, sx.bw_hz, sx.cr_denom, sx.preamble))
    assert lm.budget(sx)["poll_hz_achievable"] and lm.budget(cfg)["poll_hz_achievable"]   # v0.4 poll_hz = 2
    from dataclasses import replace as _dc_replace
    assert not lm.budget(_dc_replace(cfg, poll_hz=4.0))["poll_hz_achievable"]            # the v0.4 draft's 4 Hz
    with pytest.raises(ValueError):
        lm.LinkConfig.from_params(p, radio="wifi")


def test_link_loss_rate_seeded_outage_and_collision(veh):
    p = veh.params
    cfg = lm.LinkConfig.from_params(p, loss=0.3)
    a, b = lm.LinkModel(cfg, seed=5), lm.LinkModel(cfg, seed=5)
    n = 20000
    for i in range(n):
        a.send("up", b"\x00" * 13, i * 1.0)
        b.send("up", b"\x00" * 13, i * 1.0)
    lost_a = [tr.lost for tr in a.log]
    assert lost_a == [tr.lost for tr in b.log]                         # same seed, same pattern
    frac = sum(1 for x in lost_a if x) / n
    assert frac == pytest.approx(0.3, abs=4 * math.sqrt(0.3 * 0.7 / n))
    c = lm.LinkModel(cfg, seed=6)
    for i in range(200):
        c.send("up", b"\x00" * 13, i * 1.0)
    assert [tr.lost for tr in c.log] != lost_a[:200]
    m = lm.LinkModel(lm.LinkConfig.from_params(p, loss=0.0), seed=1, outages=[(10.0, float("inf"))])
    ok = m.send("up", b"\x00" * 13, 0.0)
    gone = m.send("up", b"\x00" * 13, 10.0)
    assert not ok.lost and gone.lost == "outage"
    assert [tr.data for tr in m.deliver("up", 100.0)] == [ok.data] and m.stats["outage"] == 1
    m = lm.LinkModel(lm.LinkConfig.from_params(p, radio="lora_uart", loss=0.0), seed=1)   # the LoRa half-duplex model
    u = m.send("up", b"\x00" * 13, 0.0)
    d = m.send("down", b"\x00" * 36, u.t_air_start)                    # both radios on air at once
    assert u.lost == d.lost == "collision"


def test_link_esp_bridge_model(veh):
    """2026-11-10: the ESP-NOW bridge model - selected by link.vehicle_radio, hand values of one hop (two 38400-baud UART
    legs, the bridge processing, 802.11b DCF access and the frame at 1 Mbps), the MAC-retry bound behind the reply
    timeout, the retry draws on their own stream (the loss draws keep the one-draw-per-frame sequence), no collisions
    (CSMA/CA), and the study hook that steps the link model mid-run."""
    p = veh.params
    lk = p["flight_computer"]["link"]
    assert lk["vehicle_radio"] == "esp_bridge" and lm.model_radio(p) == "esp_bridge" and lk["radio"] == "lora_uart"
    cfg = lm.LinkConfig.from_params(p)
    assert cfg.radio == "esp_bridge" and cfg.esp_uart_baud == lk["esp_bridge"]["uart_baud"] == 38400
    assert cfg.esp_air_rate_bps == lk["esp_bridge"]["air_rate_bps"] == 1e6
    old = json.loads(json.dumps(p))
    old["flight_computer"]["link"]["vehicle_radio"] = "sx127x"
    assert lm.model_radio(old) == "sx127x"
    del old["flight_computer"]["link"]["vehicle_radio"]
    assert lm.model_radio(old) == "lora_uart" == lm.LinkConfig.from_params(old).radio      # v0.9 files: link.radio
    baud, rate = 38400.0, 1e6
    access = lm.DSSS_DIFS_S + 0.5 * lm.DSSS_CW_MIN * lm.DSSS_SLOT_S
    assert access == pytest.approx(360e-6)
    air13 = 192e-6 + 8 * (lm.ESPNOW_FRAME_OVERHEAD_BYTES_ASSUMED + 13) / rate
    up = lm.GROUND_SERIAL_LATENCY_S_ASSUMED + 2 * 130 / baud + lm.ESP_HOP_PROCESS_S_ASSUMED + access + air13
    air36 = 192e-6 + 8 * (lm.ESPNOW_FRAME_OVERHEAD_BYTES_ASSUMED + 36) / rate
    down = 2 * 360 / baud + lm.ESP_HOP_PROCESS_S_ASSUMED + access + air36 + lm.GROUND_SERIAL_LATENCY_S_ASSUMED
    assert lm.hop_timing(cfg, "up", 13).arrival == pytest.approx(up, rel=1e-12)
    assert lm.hop_timing(cfg, "down", 36).arrival == pytest.approx(down, rel=1e-12)
    rt = lm.round_trip(cfg)
    assert rt["rtt_mean_s"] == pytest.approx(up + lm.FC_REPLY_DELAY_S + down)
    assert 0.045 < rt["rtt_mean_s"] < 0.065                                 # the README's ~0.06 s (E)
    # one retry: ACK timeout + DIFS + CW 63 / 2 slots + the frame again; the worst = every attempt on both hops
    assert lm.esp_hop_extra_s(13, rate, 1) == pytest.approx(lm.DSSS_ACK_TIMEOUT_S + lm.DSSS_DIFS_S + 31.5 * 20e-6 + air13)
    r = lm.ESP_MAC_ATTEMPTS_ASSUMED - 1
    assert rt["rtt_worst_s"] == pytest.approx(rt["rtt_mean_s"] + lm.esp_hop_extra_s(13, rate, r)
                                              + lm.esp_hop_extra_s(36, rate, r))
    assert rt["reply_timeout_s"] == pytest.approx(rt["uplink_worst_s"] + lm.FC_REPLY_ALLOWANCE_S + rt["downlink_worst_s"]
                                                  + lm.REPLY_TIMEOUT_MARGIN_S_ASSUMED)
    # the half-duplex discipline still holds against the COM's 80 ms reply window, and one lost frame stays far from
    # the pad link-loss interlock (the generator's check: poll interval + timeout < link_loss_disarm_s)
    assert rt["reply_timeout_s"] - (rt["uplink_worst_s"] + rt["downlink_worst_s"]) >= 0.080 + 0.02
    gap = max(1.0 / cfg.poll_hz, rt["rtt_worst_s"]) + max(1.0 / cfg.poll_hz, rt["reply_timeout_s"])
    assert gap < float(p["mission"]["link_loss_disarm_s"])
    for kw, nb in (({}, 36), ({"mode": "vtol"}, 39), ({"mode": "vtol", "gps": True}, 45),
                   ({"mode": "vtol", "lbat": True}, 40), ({"mode": "vtol", "gps": True, "lbat": True}, 46)):
        b = lm.budget(cfg, **kw)
        assert b["telem_frame_bytes"] == nb and b["poll_hz_achievable"]
        assert b["downlink"]["uart_to_esp"] == pytest.approx(nb * 10 / baud)
    # retry distribution: the truncated geometric of an independent-attempt channel with loss after the last attempt
    pmf = lm.esp_retry_pmf(0.05)
    assert len(pmf) == lm.ESP_MAC_ATTEMPTS_ASSUMED and sum(pmf) == pytest.approx(1.0) and pmf == sorted(pmf, reverse=True)
    assert lm.esp_attempt_fail(0.05) ** lm.ESP_MAC_ATTEMPTS_ASSUMED == pytest.approx(0.05)
    assert lm.esp_retry_pmf(0.0)[0] == 1.0
    a, b2 = lm.LinkModel(cfg, seed=5), lm.LinkModel(lm.LinkConfig.from_params(p, radio="lora_uart"), seed=5)
    n = 4000
    for i in range(n):
        a.send("up", b"\x00" * 13, i * 1.0)
        b2.send("up", b"\x00" * 13, i * 1.0)
    assert [bool(t.lost) for t in a.log] == [bool(t.lost) for t in b2.log]        # the same loss draws as LoRa
    ks = [t.retries for t in a.log if not t.lost]
    assert max(ks) <= r and min(ks) == 0 and a.stats["retries"] == sum(t.retries for t in a.log)
    for k in range(3):
        assert ks.count(k) / len(ks) == pytest.approx(pmf[k], abs=4 * math.sqrt(pmf[k] * (1 - pmf[k]) / len(ks)))
    t_k = [t for t in a.log if not t.lost and t.retries == 2][0]
    assert t_k.t_arrival - t_k.t_tx == pytest.approx(lm.hop_timing(cfg, "up", 13, retries=2).arrival)
    with pytest.raises(ValueError):
        lm.hop_timing(lm.LinkConfig.from_params(p, radio="lora_uart"), "up", 13, retries=1)
    m = lm.LinkModel(lm.LinkConfig.from_params(p, loss=0.0), seed=1)
    u = m.send("up", b"\x00" * 13, 0.0)
    d = m.send("down", b"\x00" * 36, u.t_air_start)                    # CSMA/CA: no collision
    assert u.lost == d.lost == ""
    s = lm.LinkModel(lm.LinkConfig.from_params(p, loss=0.0), seed=1,
                     cfg_schedule=[(5.0, lm.LinkConfig.from_params(p, radio="lora_uart", loss=0.0))])
    e1, e2 = s.send("up", b"\x00" * 13, 4.0), s.send("up", b"\x00" * 13, 5.0)
    assert (e1.radio, e2.radio) == ("esp_bridge", "lora_uart")
    assert e2.t_arrival - 5.0 == pytest.approx(lm.hop_timing(lm.LinkConfig.from_params(p, radio="lora_uart"), "up", 13).arrival)


# =========================================================================== #
# v0.4 closed-loop scenario smoke tests (sim.sim6dof with ground + link + supervisor)
# =========================================================================== #
def test_sim_total_link_loss_aborts_and_releases(veh):
    sc = Scenario(name="t", t_end=9.0, seed=3, link_outage_after_launch_s=1.0, packet_loss=0.0)
    res = FlightSim(veh, sc).run()
    m = compute_metrics(res)
    sp = SupervisorParams.from_params(veh.params)
    assert m["abort_reason"].startswith("I1") and m["t_deploy_s"] is not None
    t_out = m["fault_t_s"]
    # the last frame reached the vehicle before the outage began: ABORT within link_loss_abort_s (+ the last poll gap)
    assert sp.link_abort_ticks / sp.loop_hz - 1.0 < m["t_abort_s"] - t_out <= sp.link_abort_ticks / sp.loop_hz + 0.05
    assert m["t_deploy_s"] - m["t_abort_s"] == pytest.approx(sp.abort_release_ticks / sp.loop_hz)


def test_sim_assist_off_limiter_cuts(veh):
    sc = Scenario(name="t", t_end=14.0, seed=4, assist=False, packet_loss=0.0)
    res = FlightSim(veh, sc).run()
    m = compute_metrics(res)
    assert m["cut_by"].startswith("I4") and m["t_abort_s"] is None
    assert not any(r["name"] in ("CUT", "RELEASE") for r in res.link["records"])
    L = res.log
    kc = int(np.searchsorted(L["t"], m["t_cutoff_s"]))
    sp = SupervisorParams.from_params(veh.params)
    pred = L["h_est"][kc] + sp.apogee_k * L["vz_est"][kc] ** 2
    assert pred >= sp.apogee_limit_m and L["h_est"][kc - 1] + sp.apogee_k * L["vz_est"][kc - 1] ** 2 < sp.apogee_limit_m


@pytest.mark.xfail(strict=True, reason=(
    "reviewfix F2 (independent review 2026-11-04, M): test_sim_assist_off_limiter_cuts passes on the INTERNAL ESTIMATE "
    "mass only (2,585.71 g, E; 13.5 g under the CAD report).  At the CAD mass (2,599.21 g, M) the same scenario (seed 4, "
    "14 s, assist off) reaches a predicted apogee of at most 148.71 m < the 150 m I4 limit inside the window, so there "
    "is no I4 cut before 14 s.  Nothing is loosened: whether the scenario should change (e.g. a longer t_end) is a "
    "user decision.  strict: if this starts to pass (a mass or model change), pytest reports it so the note is revisited.  "
    "2026-11-10: pinned to the lora_uart link model the note was measured on - on the ESP-NOW model (the default since the "
    "2026-11-10 control round) the operator's ARM / LAUNCH exchanges are ~0.4 s shorter, the launch comes earlier and the "
    "I4 cut falls inside the 14 s window (it passes there); the window question stays the user's."))
def test_sim_assist_off_limiter_cuts_at_cad_mass():
    """The same scenario and assertions as test_sim_assist_off_limiter_cuts, on the CAD-report vehicle."""
    v = load_vehicle(prefer="cad")
    if "internal estimate" in v.mass_source:
        pytest.skip("no usable CAD report")
    sc = Scenario(name="t", t_end=14.0, seed=4, assist=False, packet_loss=0.0, radio="lora_uart")
    m = compute_metrics(FlightSim(v, sc).run())
    assert m["cut_by"] is not None and m["cut_by"].startswith("I4") and m["t_abort_s"] is None


def test_internal_estimate_total_mass_follows_the_cad_report():
    """reviewfix F2 (2026-11-04): the sim fixture flies the internal estimate (load_vehicle(prefer='estimate')), and
    test_sim_assist_off_limiter_cuts is decided by ~10 g of vehicle mass, so the estimate's whole-vehicle mass is bound
    to the CadQuery report both ways.  M 2026-11-04 (reviewfix): estimate 2,585.71 g vs CAD 2,599.21 g (-0.52 %).  The bound
    (+-1.0 %, ~26 g) flags a lost or doubled module or a mount model drifting far from the solid; the finer guard of the
    propulsion module is test_internal_estimate_propulsion_volume_follows_the_solid (+-4 % of its volume)."""
    path = os.path.join(PROJECT_ROOT, "cad", "exports", "cadquery_report.json")
    if not os.path.exists(path):
        pytest.skip("no CadQuery report")
    with open(path, encoding="utf-8") as f:
        cad_g = json.load(f)["totals"]["mass_g"]
    v = load_vehicle(prefer="estimate")
    est_g = v.full.mass_kg * 1e3
    assert abs(est_g / cad_g - 1.0) <= 0.010, (est_g, cad_g)


def test_sim_operator_kill_through_the_link_and_determinism(veh):
    sc = Scenario(name="t", t_end=5.0, seed=8, kill_time_s=3.0)
    r1, r2 = FlightSim(veh, sc).run(), FlightSim(veh, sc).run()
    assert r1.events == r2.events                                       # seeded loss + nonce: reproducible
    ab = [e for _, e in r1.events if "POWERED -> ABORT" in e]
    assert ab and "KILL" in ab[0]
    rows = compute_metrics(r1)["commands"]
    assert [r["cmd"] for r in rows] == ["ARM", "LAUNCH", "KILL"] and all(r["decision_to_exec_s"] > 0 for r in rows)

