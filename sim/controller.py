"""Reference attitude controller - SPEC §8.5 (mixer) and §8.6 (cascade).

This file *is* the controller specification for the firmware
(``firmware/teensy_tvc/src/core/{pid,mixer,attitude_controller}.h``):
``tools/crosscheck_controller.py`` feeds identical inputs to both.  Keep it
numpy-only, every state explicit, every equation written out.

Loop (period dt = 1 / ``control.loop_hz``), inputs: attitude estimate q,
gyro omega [rad/s, body], throttle u [0..1]:

  1. setpoint  q_sp = upright with heading psi_ref (yaw hold) or current heading
  2. error     q_e = q^-1 (x) q_sp, w >= 0, e = 2 atan2(|v|, w) v/|v|  (rotation vector, body frame)
  3. rate sp   w_sp,xy = clamp(att_kp_xy e_xy, +-rate_sp_max_xy); w_sp,z = clamp(att_kp_z e_z, +-rate_sp_max_z)
  4. gyro LPF  w_f  = LPF(gyro, gyro_lpf_hz)
  5. rate PID  M = Kp (w_sp - w_f) + I - Kd d/dt LPF(w_f, d_term_lpf_hz),
               I += Ki (w_sp - w_f) dt, |I| <= i_max, frozen while the previous
               cycle saturated a vane or while the thrust estimate is floored
  6. mixer     T_hat = max(T_max u^n, floor T_max); k = T_hat S_v C_La / (2 A_e)
               d_x = M_x/(2 l_v k), d_y = M_y/(2 l_v k), d_z = roll_scale M_z/(4 r_v k)
               d1 = d_x + d_z, d2 = d_y + d_z, d3 = -d_x + d_z, d4 = -d_y + d_z
               saturation: max|d_i| = max(|d_x|,|d_y|) + |d_z|; first shrink |d_z|
               to d_max - max(|d_x|,|d_y|) (>= 0), then scale (d_x, d_y) so the
               largest |d_i| = d_max
  7. servo     pwm_i = pwm_center + sign_i deg(d_i) pwm_us_per_deg + trim_i

Invariant: no number lives here - gains come from the ``control`` dict
(``params.json`` or ``sim/out/gains.json``), plant constants from the vehicle.
The VTOL build's options (the tilt set point, the reduced attitude error, the per-axis anti-windup, the rotor
feed-forwards, the yaw reserve and the thrust-normalised yaw integrator, params control.vtol_overrides) were removed
with the VTOL program (2026-11-18): the ballistic controller below is the v0.8.1 one, bit for bit.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import quat
except ImportError:  # pragma: no cover
    import quat  # type: ignore

GAIN_KEYS = (
    "loop_hz", "imu_hz", "baro_hz",
    "att_kp_xy", "att_kp_z", "rate_sp_max_xy_rad_s", "rate_sp_max_z_rad_s",
    "rate_kp_xy", "rate_ki_xy", "rate_kd_xy", "rate_i_max_xy",
    "rate_kp_z", "rate_ki_z", "rate_kd_z", "rate_i_max_z",
    "d_term_lpf_hz", "gyro_lpf_hz", "thrust_estimate_floor_fraction", "roll_priority_scale", "yaw_hold",
)



# --------------------------------------------------------------------------- #
# plant constants the controller needs (filled from Vehicle by sim6dof)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MixerPlant:
    """Constants of SPEC §8.3-8.5 (SI, rad)."""

    l_v_m: float            # z_cg - z_hinge
    r_v_m: float            # vane force centroid radius
    s_v_m2: float           # vane area
    cl_alpha: float         # per rad
    a_e_m2: float           # exhaust annulus
    t_max_n: float          # edf.thrust_nominal_N
    throttle_exp: float     # edf.throttle_exponent
    delta_max_rad: float    # vane.max_deflection_cmd_deg
    pwm_center_us: float
    pwm_us_per_deg: float
    servo_signs: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    servo_trims_us: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# filters
# --------------------------------------------------------------------------- #
class LowPass:
    """First-order low-pass on a vector: y += a (x - y), a = dt / (tau + dt),
    tau = 1 / (2 pi f_c).  ``f_c <= 0`` disables the filter (pass-through)."""

    def __init__(self, f_cut_hz: float, dt: float, n: int = 3):
        self.alpha = 1.0 if f_cut_hz <= 0.0 else dt / (1.0 / (2.0 * math.pi * f_cut_hz) + dt)
        self.y = np.zeros(n)
        self.primed = False

    def reset(self) -> None:
        self.y[:] = 0.0
        self.primed = False

    def update(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if not self.primed:          # start on the first sample (no start-up transient)
            self.y = x.copy()
            self.primed = True
        else:
            self.y = self.y + self.alpha * (x - self.y)
        return self.y.copy()


# --------------------------------------------------------------------------- #
# rate PID
# --------------------------------------------------------------------------- #
class RatePID:
    """Three-axis rate PID of SPEC §8.6 with explicit state."""

    def __init__(self, g: dict, dt: float):
        self.dt = dt
        self.kp = np.array([g["rate_kp_xy"], g["rate_kp_xy"], g["rate_kp_z"]], dtype=float)
        self.ki = np.array([g["rate_ki_xy"], g["rate_ki_xy"], g["rate_ki_z"]], dtype=float)
        self.kd = np.array([g["rate_kd_xy"], g["rate_kd_xy"], g["rate_kd_z"]], dtype=float)
        self.i_max = np.array([g["rate_i_max_xy"], g["rate_i_max_xy"], g["rate_i_max_z"]], dtype=float)
        self.gyro_lpf = LowPass(float(g["gyro_lpf_hz"]), dt)
        self.d_lpf = LowPass(float(g["d_term_lpf_hz"]), dt)
        self.integ = np.zeros(3)
        self.d_prev = np.zeros(3)
        self.d_primed = False
        self.last_p = np.zeros(3)
        self.last_d = np.zeros(3)
        self.last_i = np.zeros(3)

    def reset(self) -> None:
        self.gyro_lpf.reset()
        self.d_lpf.reset()
        self.integ[:] = 0.0
        self.d_prev[:] = 0.0
        self.d_primed = False

    def update(self, rate_sp: np.ndarray, gyro: np.ndarray, freeze_integrator: bool) -> tuple[np.ndarray, np.ndarray]:
        """Returns (moment command [N m], filtered gyro).  ``freeze_integrator``: all three integrators hold."""
        w_f = self.gyro_lpf.update(gyro)
        err = np.asarray(rate_sp, dtype=float) - w_f
        if not freeze_integrator:
            self.integ = np.clip(self.integ + self.ki * err * self.dt, -self.i_max, self.i_max)
        i_out = self.integ
        w_d = self.d_lpf.update(w_f)
        if self.d_primed:
            d_rate = (w_d - self.d_prev) / self.dt
        else:
            d_rate = np.zeros(3)
            self.d_primed = True
        self.d_prev = w_d
        self.last_p = self.kp * err
        self.last_d = -self.kd * d_rate              # D on the measurement: opposes gyro change
        self.last_i = i_out
        return self.last_p + i_out + self.last_d, w_f


# --------------------------------------------------------------------------- #
# mixer
# --------------------------------------------------------------------------- #
class VaneMixer:
    """Moments -> vane deflections (SPEC §8.5) and the forward linear model."""

    def __init__(self, plant: MixerPlant, floor_fraction: float, roll_priority_scale: float):
        self.p = plant
        self.floor_fraction = float(floor_fraction)
        self.roll_priority_scale = float(roll_priority_scale)
        self.delta_max = float(plant.delta_max_rad)

    # thrust estimate
    def thrust_model(self, u: float) -> float:
        u = min(max(float(u), 0.0), 1.0)
        return self.p.t_max_n * u ** self.p.throttle_exp

    def thrust_estimate(self, u: float) -> float:
        """T_hat = max(T_model(u), floor T_max)."""
        return max(self.thrust_model(u), self.floor_fraction * self.p.t_max_n)

    def thrust_floored(self, u: float) -> bool:
        return self.thrust_model(u) < self.floor_fraction * self.p.t_max_n

    # authority
    def force_slope(self, t_hat: float) -> float:
        """k = q_e S_v C_La with q_e = T_hat / (2 A_e)  [N/rad per vane]."""
        return t_hat / (2.0 * self.p.a_e_m2) * self.p.s_v_m2 * self.p.cl_alpha

    def moment_gains(self, t_hat: float) -> tuple[float, float, float]:
        """(2 l_v k, 2 l_v k, 4 r_v k): N m per rad of (d_x, d_y, d_z)."""
        k = self.force_slope(t_hat)
        return 2.0 * self.p.l_v_m * k, 2.0 * self.p.l_v_m * k, 4.0 * self.p.r_v_m * k

    @staticmethod
    def combine(dx: float, dy: float, dz: float) -> np.ndarray:
        return np.array([dx + dz, dy + dz, -dx + dz, -dy + dz])

    @staticmethod
    def split(delta: np.ndarray) -> tuple[float, float, float]:
        """d_x = (d1 - d3)/2, d_y = (d2 - d4)/2, d_z = (d1+d2+d3+d4)/4."""
        d = np.asarray(delta, dtype=float)
        return 0.5 * (d[0] - d[2]), 0.5 * (d[1] - d[3]), 0.25 * (d[0] + d[1] + d[2] + d[3])

    def deflections(self, moments: np.ndarray, t_hat: float) -> tuple[np.ndarray, bool]:
        """Inverse mixer with saturation priority.  Returns (delta[4] rad, saturated)."""
        gx, gy, gz = self.moment_gains(t_hat)
        dx = float(moments[0]) / gx
        dy = float(moments[1]) / gy
        dz = self.roll_priority_scale * float(moments[2]) / gz
        m_xy = max(abs(dx), abs(dy))
        saturated = False
        if m_xy + abs(dz) > self.delta_max + 1e-12:
            saturated = True
            if m_xy < self.delta_max:
                dz = math.copysign(self.delta_max - m_xy, dz)        # 1) shrink roll
            else:
                dz = math.copysign(0.0, dz)                          # 2) drop roll, scale pitch / yaw into d_max
                s = self.delta_max / m_xy
                dx *= s
                dy *= s
        return self.combine(dx, dy, dz), saturated

    def moments(self, delta: np.ndarray, t_hat: float) -> np.ndarray:
        """Forward linear model of SPEC §8.4 (roll_priority_scale is a command
        shaping term, so the physical roll moment is 4 r_v k d_z)."""
        gx, gy, gz = self.moment_gains(t_hat)
        dx, dy, dz = self.split(delta)
        return np.array([gx * dx, gy * dy, gz * dz])

    def pwm(self, delta: np.ndarray) -> np.ndarray:
        d = np.degrees(np.asarray(delta, dtype=float))
        return (self.p.pwm_center_us + np.asarray(self.p.servo_signs) * d * self.p.pwm_us_per_deg
                + np.asarray(self.p.servo_trims_us))


# --------------------------------------------------------------------------- #
# attitude controller
# --------------------------------------------------------------------------- #
@dataclass
class ControllerOutput:
    delta_cmd: np.ndarray          # (4,) rad
    moments: np.ndarray            # (3,) N m demanded
    rate_sp: np.ndarray            # (3,) rad/s
    att_error: np.ndarray          # (3,) rad rotation vector (body)
    gyro_filtered: np.ndarray      # (3,) rad/s
    saturated: bool
    thrust_est: float
    pwm_us: np.ndarray             # (4,)
    integrator: np.ndarray = field(default_factory=lambda: np.zeros(3))


class AttitudeController:
    """Cascaded attitude -> rate controller feeding the vane mixer."""

    def __init__(self, control: dict, plant: MixerPlant, gains: dict | None = None):
        g = dict(control)
        if gains:
            g.update({k: v for k, v in gains.items() if not k.startswith("_")})
        self.g = g
        self.dt = 1.0 / float(g["loop_hz"])
        self.att_kp = np.array([g["att_kp_xy"], g["att_kp_xy"], g["att_kp_z"]], dtype=float)
        self.rate_sp_max = np.array([g["rate_sp_max_xy_rad_s"], g["rate_sp_max_xy_rad_s"], g["rate_sp_max_z_rad_s"]],
                                    dtype=float)
        self.yaw_hold = bool(g["yaw_hold"])
        self.pid = RatePID(g, self.dt)
        self.mixer = VaneMixer(plant, g["thrust_estimate_floor_fraction"], g["roll_priority_scale"])
        self.yaw_ref = 0.0
        self.last_saturated = False

    def reset(self) -> None:
        self.pid.reset()
        self.last_saturated = False

    def set_yaw_reference(self, q_est: np.ndarray) -> None:
        """Capture the arming heading for yaw hold (SPEC §8.6)."""
        self.yaw_ref = quat.yaw(q_est)

    def setpoint(self, q_est: np.ndarray) -> np.ndarray:
        psi = self.yaw_ref if self.yaw_hold else quat.yaw(q_est)
        return quat.from_yaw(psi)

    def update(self, q_est: np.ndarray, gyro: np.ndarray, throttle: float) -> ControllerOutput:
        q_sp = self.setpoint(q_est)
        e = quat.to_rotvec(quat.error(q_est, q_sp))
        rate_sp = np.clip(self.att_kp * e, -self.rate_sp_max, self.rate_sp_max)
        t_hat = self.mixer.thrust_estimate(throttle)
        floored = self.mixer.thrust_floored(throttle)
        gyro = np.asarray(gyro, dtype=float)
        moments, w_f = self.pid.update(rate_sp, gyro, self.last_saturated or floored)
        delta, sat = self.mixer.deflections(moments, t_hat)
        self.last_saturated = sat
        return ControllerOutput(delta, moments, rate_sp, e, w_f, sat, t_hat, self.mixer.pwm(delta),
                                self.pid.integ.copy())


def gains_from_control(control: dict, overrides: dict | None = None) -> dict:
    """The ``params.control`` dict (optionally overridden) restricted to GAIN_KEYS."""
    g = {k: control[k] for k in GAIN_KEYS if k in control}
    if overrides:
        g.update({k: v for k, v in overrides.items() if k in GAIN_KEYS})
    return g
