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
v0.7.1b: the VTOL build flies ``gains_for_mode(control, gains, "vtol")`` = those gains with
``params.control.vtol_overrides`` on top (rate_i_max_z); the ballistic build's gains are unchanged.

v0.8 (VTOL with GPS position hold): step 1's setpoint can be tilted.  ``tilt_sp`` = (t_x, t_y), the rotation vector
of the tilt in the NAV level frame (rad) that the FCU's position hold built at the end of the previous tick
(``sim.position_ctrl.TiltSetpoint``); then q_sp = q(t) (x) q_yaw with q(t) by series (no trig: th^2 = t.t, w = 1 -
th^2/8 + th^4/384, k = 1/2 - th^2/48 + th^4/3840, q(t) = (w, k t_x, k t_y, 0)) and q_yaw = (c, 0, 0, s) the yaw-hold
quaternion, expanded: q_sp = (w c, k t_x c + k t_y s, k t_y c - k t_x s, w s) (== ``sim.position_ctrl.setpoint_quat``).
``tilt_sp`` None or (0, 0): the v0.7.1 setpoint exactly (the ballistic build never sets it).

v0.9 (VTOL hardware round: the light VTOL vehicle - I_xx 0.029-0.035 instead of 0.132 kg m2 -, the direct-drive vanes
with ~1.65 deg of play, the EDF rotor's gyroscopic and spool-up torques).  Four VTOL-only options, each read from the
gains dict (``VTOL_EXTRA_KEYS``, set through params.control.vtol_overrides; absent = off, so the ballistic build and every
earlier call stays bit-identical):

* ``att_err_mode`` (step 2): 0 = the rotation vector (v0.8.1); 1 = the reduced attitude error ``quat.reduced_error``
  while the tilt set point is non-zero (the v0.8.2 study's candidate D, the C branch c->tilt_on); 2 = the reduced error
  in every tick of the VTOL build (also at the (0, 0) / None tilt of the take-off and of the GPS-off build).  The
  reduced error does not turn the tilt correction by phi / 2 for a heading error phi (the rotation vector does, and
  scales it by (phi / 2) / sin(phi / 2)): e_xy = 2 (w x - y z, w y + x z), e_z = 2 z.  Same first-order error, so the
  linear loop is unchanged.  No trig, square root or division (the rotation vector needs sqrtf + atan2f + a division
  above 11.4 deg).  Flown: 1 (TUNE seed 7400: without it 13 more aborts of 143; mode 2 cost 2 aborts in the tether
  group - not flown).
* ``aw_per_axis`` (step 5): 0 = the v0.8.1 anti-windup (every integrator frozen when the previous tick saturated a
  vane or the thrust estimate is floored); 1 = per-axis conditional integration: an axis' integrator is frozen only
  when the mixer limited THAT axis in the previous tick (pitch / roll: (d_x, d_y) scaled; yaw: d_z shrunk or dropped)
  AND this tick's rate error has the sign of that axis' previous moment demand (integrating would push further into
  the limit); an error that opposes the demand always integrates (it unwinds).  The floored-thrust freeze stays for all
  three axes.  Why (v0.8.2 diagnosis): the global freeze kept the YAW integrator below its need whenever pitch / roll
  saturated a vane - exactly when the swirl was turning the vehicle - and 72 % of the later yaw runaways had a yaw
  integrator that never reached its limit.
* ``ff_rotor_h_nms`` (N m s, signed: s I_rot omega_ref, the rotor's angular momentum magnitude at u = 1 with the swirl
  sign s) with ``ff_rotor_tau_s`` (the EDF thrust lag): the rotor speed estimate omega_hat = omega_ref u_lag, u_lag the
  first-order lag of the ESC throttle u (u_lag += a (u - u_lag), a = dt / (tau + dt)), so domega_hat / dt = omega_ref
  (u - u_lag) / tau.
  - ``ff_rotor_react`` = 1: spool-up reaction feed-forward.  The motor torque that accelerates the rotor turns the
    stator the other way: M_react = +s I_rot domega/dt about body z (the swirl's sign).  The yaw moment command gets
    -h_ref (u - u_lag) / tau / roll_priority_scale (the mixer applies roll_priority_scale to every yaw command, so the
    physical torque is the cancelling one).  Measured need (v0.9 plant round): 0.1-0.3 N m at lift-off against
    0.14 N m of yaw authority at hover; hover throttle jitter of +-0.04 at 1-2 Hz gives +-0.03-0.05 N m at the yaw
    crossover.
  - ``ff_rotor_gyro`` = 1: gyroscopic decoupling.  The rotor's angular momentum h = -s I_rot omega z_b gives the body
    the moment -omega_b x h = (-w_y h_z, w_x h_z, 0); the command gets +(w_y h_z, -w_x h_z) with h_z = -h_ref u_lag and
    the filtered gyro (0.12 N m s at hover: 1 rad/s of pitch rate couples 0.12 N m into roll, 22 % of the hover
    authority).
  The rotor numbers are ASSUMED (edf.thrust_v09.rotor_inertia_kg_m2_ASSUMED, omega_ref = KV x 22.2 V no-load, the
  rotation direction edf.rotation): a wrong rotation sign doubles instead of cancels - verify the fan's rotation before
  flight (it also sets the swirl sign).
* ``mix_yaw_reserve_deg`` (step 6): the mixer's saturation rule keeps r = min(|d_z|, reserve) of the yaw command when
  pitch / roll need the whole deflection (v0.8.1: d_z = 0, i.e. r = 0): d_z = sign(d_z) max(d_max - m_xy, r) and (d_x,
  d_y) scaled into d_max - |d_z|.  Why: on the light vehicle (I_zz 0.0032 kg m2) the swirl alone (0.045-0.054 N m at x1)
  accelerates the dropped yaw at 14-17 rad/s^2 - 0.25 s of pitch / roll saturation (a tether jerk, a push) left
  200-400 deg/s and a heading error that the coupling then turned into the I3 spin (v0.9 TUNE tether runs).  NOT
  flown (0 in params): 8 deg on TUNE cost 2 tether / 1 demo aborts (less pitch / roll authority under the disturbance).
* ``yaw_i_thrust_norm`` = 1 (the add-on "B", in its continuous form): the yaw integrator is a thrust-proportional trim.
  Its state I_z is held at the thrust T_max: the moment it adds is I_z T_hat / T_max, and it integrates rate_ki_z e_z dt
  T_max / T_hat (the integral loop gain is unchanged at any thrust).  The residual swirl torque is proportional to the
  thrust (SPEC 8.4; the vane deflection that cancels it is thrust-independent, 7.49 deg per swirl unit - v0.8.2
  diagnosis), so a trim learnt at one thrust is right at every other thrust (take-off, climbs, pushes, gusts) instead of
  lagging by the integral time constant.  rate_i_max_z then bounds I_z (at T_max).  NOT flown (0 in params): TUNE +1
  field pass, +1 tether abort (the add-on rule: adopted only with no regression).
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
# v0.9: VTOL-only options (module docstring); only params.control.vtol_overrides may set them (never params.control /
# gains.json: the ballistic tuner writes exactly GAIN_KEYS); absent = off
VTOL_EXTRA_KEYS = ("att_err_mode", "aw_per_axis", "ff_rotor_h_nms", "ff_rotor_tau_s", "ff_rotor_react", "ff_rotor_gyro",
                   "yaw_i_thrust_norm", "mix_yaw_reserve_deg", "mix_yaw_trim_reserve")


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

    def update(self, rate_sp: np.ndarray, gyro: np.ndarray, freeze_integrator, i_scale: np.ndarray | None = None,
               freeze_fn=None) -> tuple[np.ndarray, np.ndarray]:
        """Returns (moment command [N m], filtered gyro).  ``freeze_integrator``: a bool (all three axes, v0.8.1) or a
        per-axis sequence of 3 bools; ``freeze_fn(err)`` (v0.9 per-axis conditional anti-windup) - when given, the mask
        is computed from this tick's rate error.  ``i_scale`` (v0.9 yaw_i_thrust_norm): per-axis factor s on the
        integrator's contribution, the state integrating ki e dt / s (the moment it adds is s x state)."""
        w_f = self.gyro_lpf.update(gyro)
        err = np.asarray(rate_sp, dtype=float) - w_f
        if freeze_fn is not None:
            freeze_integrator = freeze_fn(err)
        if i_scale is None:
            if isinstance(freeze_integrator, (bool, np.bool_)):
                if not freeze_integrator:
                    self.integ = np.clip(self.integ + self.ki * err * self.dt, -self.i_max, self.i_max)
            else:
                fr = np.asarray(freeze_integrator, dtype=bool)
                new = np.clip(self.integ + self.ki * err * self.dt, -self.i_max, self.i_max)
                self.integ = np.where(fr, self.integ, new)
            i_out = self.integ
        else:
            fr = (np.full(3, bool(freeze_integrator)) if isinstance(freeze_integrator, (bool, np.bool_))
                  else np.asarray(freeze_integrator, dtype=bool))
            new = np.clip(self.integ + self.ki * err * self.dt / i_scale, -self.i_max, self.i_max)
            self.integ = np.where(fr, self.integ, new)
            i_out = self.integ * i_scale
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
        self.last_lim = (False, False)
        self.yaw_reserve = 0.0          # v0.9 mix_yaw_reserve_deg (rad; AttitudeController sets it): 0 = the v0.8.1 mixer
        self.yaw_trim = 0.0             # v0.9 fix round mix_yaw_trim_reserve: the yaw integrator's deflection this tick (rad,
                                        # signed; AttitudeController sets it; 0 = off)

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
        lim_xy = False
        if m_xy + abs(dz) > self.delta_max + 1e-12:
            saturated = True
            r = min(abs(dz), self.yaw_reserve)                       # v0.9: yaw share kept under pitch / roll demand
            if self.yaw_trim * dz > 0.0:
                # fix round (control review HIGH 3): the trim the yaw integrator holds against the residual swirl is
                # kept (only the transient yaw share is dropped) - dropping it let the swirl spin the body
                r = max(r, min(abs(dz), abs(self.yaw_trim), self.delta_max))
            if m_xy < self.delta_max - r:
                dz = math.copysign(self.delta_max - m_xy, dz)        # 1) shrink roll
            else:
                dz = math.copysign(r, dz)                            # 2) roll to its reserve (v0.8.1: 0), scale
                s = (self.delta_max - r) / m_xy                      #    pitch / yaw into the rest
                dx *= s
                dy *= s
                lim_xy = True
        self.last_lim = (lim_xy, saturated)          # v0.9: (pitch / roll limited, yaw limited) - per-axis anti-windup
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
        self.mixer.yaw_reserve = math.radians(float(g.get("mix_yaw_reserve_deg", 0.0)))   # v0.9 (module docstring)
        self.yaw_trim_reserve = bool(g.get("mix_yaw_trim_reserve", 0))                     # v0.9 fix round
        self.yaw_ref = 0.0
        self.last_saturated = False
        self.tilt_sp: tuple | None = None    # v0.8 VTOL: level-frame tilt rotation vector (module docstring)
        # v0.9 VTOL options (module docstring; absent from the gains dict = off: the v0.8.1 controller)
        self.err_mode = int(g.get("att_err_mode", 0))
        self.aw_per_axis = bool(g.get("aw_per_axis", 0))
        self.yaw_i_norm = bool(g.get("yaw_i_thrust_norm", 0))
        self.rotor_h = float(g.get("ff_rotor_h_nms", 0.0))
        tau = float(g.get("ff_rotor_tau_s", 0.15))
        self.rotor_a = self.dt / (tau + self.dt)
        self.rotor_react = float(g.get("ff_rotor_react", 0))      # fix round: a gain (1 = the control round's on)
        self.rotor_gyro = bool(g.get("ff_rotor_gyro", 0))
        # -h_ref du_lag/dt / roll_priority_scale with du_lag/dt = (u - u_lag) / tau: one constant on (u - u_lag)
        rps = float(g["roll_priority_scale"])
        self.rotor_react_k = self.rotor_h / tau / rps if rps > 0.0 else 0.0
        self.u_lag = 0.0
        self.u_lag_primed = False
        self.last_moments = np.zeros(3)
        if self.err_mode not in (0, 1, 2):
            raise ValueError(f"att_err_mode {self.err_mode} not in (0, 1, 2)")

    def reset(self) -> None:
        self.pid.reset()
        self.last_saturated = False
        self.last_moments = np.zeros(3)
        self.u_lag_primed = False

    def set_yaw_reference(self, q_est: np.ndarray) -> None:
        """Capture the arming heading for yaw hold (SPEC §8.6)."""
        self.yaw_ref = quat.yaw(q_est)

    def setpoint(self, q_est: np.ndarray) -> np.ndarray:
        psi = self.yaw_ref if self.yaw_hold else quat.yaw(q_est)
        q_y = quat.from_yaw(psi)
        t = self.tilt_sp
        if t is None or (t[0] == 0.0 and t[1] == 0.0):
            return q_y
        tx, ty = float(t[0]), float(t[1])
        th2 = tx * tx + ty * ty
        w = 1.0 - th2 / 8.0 + th2 * th2 / 384.0
        k = 0.5 - th2 / 48.0 + th2 * th2 / 3840.0
        c, s = float(q_y[0]), float(q_y[3])
        return np.array([w * c, k * tx * c + k * ty * s, k * ty * c - k * tx * s, w * s])

    def reduced_error_on(self) -> bool:
        """v0.9 ``att_err_mode``: fly the reduced attitude error this tick (module docstring)."""
        if self.err_mode == 2:
            return True
        if self.err_mode == 1:
            t = self.tilt_sp
            return t is not None and not (t[0] == 0.0 and t[1] == 0.0)
        return False

    def _freeze_mask(self, floored: bool):
        """v0.9 per-axis conditional anti-windup (module docstring): a function of this tick's rate error."""
        lim_xy, lim_z = self.mixer.last_lim if self.last_saturated else (False, False)
        m = self.last_moments

        def fn(err: np.ndarray) -> list[bool]:
            return [floored or (lim_xy and float(err[0]) * float(m[0]) > 0.0),
                    floored or (lim_xy and float(err[1]) * float(m[1]) > 0.0),
                    floored or (lim_z and float(err[2]) * float(m[2]) > 0.0)]
        return fn

    def update(self, q_est: np.ndarray, gyro: np.ndarray, throttle: float) -> ControllerOutput:
        q_sp = self.setpoint(q_est)
        if self.reduced_error_on():
            e = quat.reduced_error(quat.error(q_est, q_sp))
        else:
            e = quat.to_rotvec(quat.error(q_est, q_sp))
        rate_sp = np.clip(self.att_kp * e, -self.rate_sp_max, self.rate_sp_max)
        t_hat = self.mixer.thrust_estimate(throttle)
        floored = self.mixer.thrust_floored(throttle)
        gyro = np.asarray(gyro, dtype=float)
        if self.aw_per_axis:
            i_scale = (np.array([1.0, 1.0, t_hat / self.mixer.p.t_max_n]) if self.yaw_i_norm else None)
            moments, w_f = self.pid.update(rate_sp, gyro, False, i_scale=i_scale, freeze_fn=self._freeze_mask(floored))
        elif self.yaw_i_norm:
            moments, w_f = self.pid.update(rate_sp, gyro, self.last_saturated or floored,
                                           i_scale=np.array([1.0, 1.0, t_hat / self.mixer.p.t_max_n]))
        else:
            moments, w_f = self.pid.update(rate_sp, gyro, self.last_saturated or floored)
        if self.rotor_h != 0.0:
            # v0.9 rotor feed-forwards (module docstring): omega_hat = omega_ref u_lag, domega_hat/dt = omega_ref (u -
            # u_lag) / tau; h_ref = s I_rot omega_ref
            u = min(max(float(throttle), 0.0), 1.0)
            if not self.u_lag_primed:
                self.u_lag = u
                self.u_lag_primed = True
            du = u - self.u_lag
            self.u_lag += self.rotor_a * du
            if self.rotor_react:
                moments = moments + np.array([0.0, 0.0, -self.rotor_react * self.rotor_react_k * du])
            if self.rotor_gyro:
                hz = -self.rotor_h * self.u_lag
                moments = moments + np.array([float(w_f[1]) * hz, -float(w_f[0]) * hz, 0.0])
        if self.yaw_trim_reserve:
            # fix round: the yaw integrator's share of d_z (the mixer's d_z scaling: roll_priority_scale M_z / g_z)
            gz = self.mixer.moment_gains(t_hat)[2]
            self.mixer.yaw_trim = self.mixer.roll_priority_scale * float(self.pid.integ[2]) / gz if gz > 0.0 else 0.0
        delta, sat = self.mixer.deflections(moments, t_hat)
        self.last_saturated = sat
        self.last_moments = moments
        return ControllerOutput(delta, moments, rate_sp, e, w_f, sat, t_hat, self.mixer.pwm(delta),
                                self.pid.integ.copy())


def gains_from_control(control: dict, overrides: dict | None = None) -> dict:
    """The ``params.control`` dict (optionally overridden) restricted to GAIN_KEYS."""
    g = {k: control[k] for k in GAIN_KEYS if k in control}
    if overrides:
        g.update({k: v for k, v in overrides.items() if k in GAIN_KEYS})
    return g


# --------------------------------------------------------------------------- #
# v0.7.1b: attitude gains of the VTOL build (params.control.vtol_overrides)
# --------------------------------------------------------------------------- #
VTOL_MODE = "vtol"                  # sim.link_proto.MODE_VTOL (not imported: this module stays numpy + quat only)


def vtol_overrides(control: dict) -> dict:
    """``params.control.vtol_overrides`` restricted to the attitude-gain keys (architect v0.7.1b: rate_i_max_z 0.25 -
    the physics review found the body-axis rate integrator pinned at 0.15 in every hover, the EDF residual swirl needs
    ~0.17 at hover / 0.20 at T_max after roll_priority_scale).  ``note`` and keys starting with '_' are comments; any
    other key that is not in GAIN_KEYS (or the loop rates, which the build fixes) is an error, never ignored."""
    out = {}
    for k, v in (control.get("vtol_overrides") or {}).items():
        if k == "note" or k.startswith("_") or k.endswith("_note"):
            continue
        if (k not in GAIN_KEYS and k not in VTOL_EXTRA_KEYS) or k in ("loop_hz", "imu_hz", "baro_hz"):
            raise ValueError(f"control.vtol_overrides.{k}: not an attitude gain the VTOL build may override")
        out[k] = v
    return out


def gains_for_mode(control: dict, gains: dict | None, mode: str) -> dict:
    """The attitude gains a build flies: ``gains_from_control(control, gains)`` (params.control <- the tuned gains file),
    then, in the VTOL build only, ``vtol_overrides(control)`` on top (they win over sim/out/gains.json, which was tuned
    for the ballistic flight).  Ballistic: exactly ``gains_from_control(control, gains)`` - the v0.6.3 dict, bit for bit.
    The firmware's twin is core/fc_config.c fc_vtol_gains (params_gen.h VTOL_OV_*)."""
    g = gains_from_control(control, gains)
    if mode == VTOL_MODE:
        g.update(vtol_overrides(control))
    return g
