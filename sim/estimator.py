"""ATmega128 flight-computer attitude estimator and the MPU-6050 IMU model.

Executable specification of what the firmware does with the GY-521
MPU-6050 (v0.6.3: on the NAV board, ``params.electronics.nav``; the estimator
runs on the NAV ATmega128A, ``firmware/atmega128_fc/src/nav/nav_main.c``); keep
it numpy-only, every state explicit, so ``firmware/`` can mirror it line by line.

Estimator (``PadAlignGyroEstimator``), called once per control tick
(dt = 1 / ``control.loop_hz``) with the bias-uncorrected gyro g_k [rad/s] and
specific force a_k [m/s^2], both in the body frame:

  on the pad (mission state IDLE or ARMED; the vehicle is static):
      n      = min(n + 1, N_avg),        N_avg = round(pad_avg_tau_s / dt)
      b_hat += (g_k - b_hat) / n          gyro bias: running mean of the first
      f_hat += (a_k - f_hat) / n          N_avg samples, then an EMA with alpha = 1/N_avg
      roll   = atan2(f_y, f_z)
      pitch  = atan2(-f_x, sqrt(f_y^2 + f_z^2))
      q_hat  = q_zyx(roll, pitch, yaw = 0)          (pad heading is the yaw reference)
  after launch (every other state): gyro only, bias frozen, accelerometer unused
      q_hat  = q_hat (x) exp((g_k - b_hat) dt)       (``quat.integrate``)
  rate fed to the rate PID: g_k - b_hat

Why gyro-only after launch: in powered flight the accelerometer measures the
specific force, which points along the thrust axis whatever the attitude, so a
gravity-referenced correction would drag the estimate toward "upright" and
hide a real tilt (see the S2m study of the previous report).

MPU-6050 model (``Mpu6050``): truth -> + bias + white noise -> range clip ->
int16 quantisation.  Gyro: FS_SEL = 3 (+-2000 deg/s, 16.4 LSB/(deg/s)),
noise density 0.005 deg/s/sqrt(Hz) (datasheet), slowly varying bias up to
0.5 deg/s per axis (first-order Gauss-Markov, ``gyro_bias_process``).  The
on-chip DLPF is a first-order lag whose time constant is the datasheet delay
of the configured DLPF_CFG; white noise is scaled by the equivalent noise
bandwidth 1 / (4 tau) of that lag (or imu_hz / 2 with the DLPF off).

VTOL NAV estimator (v0.7.1, ``GatedMahonyEstimator``, params.json
``mission.vtol.nav_estimator``; the VTOL build only - the ballistic NAV keeps
``PadAlignGyroEstimator`` bit for bit).  Why: in the 6-DOF the gyro-only
estimate drifted 7.6 deg over a 40 s hover with the ASSUMED MPU-6050 bias
wander; the vehicle then held a wrong vertical and, with nothing that measures
horizontal motion, landed 7.7 m/s sideways 120 m away.  Same call as the
ballistic estimator, once per control tick (dt = 1 / ``control.loop_hz``):

  pad (``on_pad`` = ``sim.nlink_proto.nav_pad_mode``: FCU state SAFE / ARMED):
      exactly the PadAlignGyroEstimator pad branch (n, b_hat, f_hat, levelling),
      plus  a_lpf = f_hat,  b_pad = b_hat,  e = 0, gate closed.
      Re-entry (flight -> pad, i.e. LANDED -> SAFE before another hop):
      f_hat := a_lpf first - the levelling starts from the specific force just
      measured at the landing spot, not from the previous pad (no attitude jump;
      the EMA then refines it with the time constant N_avg dt); b_hat continues
      from the in-flight value; n stays at N_avg.
  flight (every other state):
      a_lpf += alpha_a (a_k - a_lpf),   alpha_a = 1 - exp(-2 pi f_a dt), f_a = accel_lpf_hz
      gate   = | |a_lpf| - g | < accel_gate g                    (strict)
      v_hat  = R(q_hat)^T e_z = (2(x z - w y), 2(y z + w x), 1 - 2(x^2 + y^2))  estimated up, body frame
      e      = (a_lpf / |a_lpf|) x v_hat   if gate, else (0, 0, 0)        [rad]
      b_hat  = b_pad + clamp(b_hat - b_pad - ki e dt, -b_lim, +b_lim)  per axis   (in-flight learning)
      q_hat  = q_hat (x) exp((g_k - b_hat + kp e) dt)                         (``quat.integrate``)
  rate fed to the rate PID: g_k - b_hat (the correction kp e is not a body rate).

Units: g_k [rad/s], a_k [m/s^2] body frame; kp [1/s], ki [1/s^2], b_lim [rad/s].
The order inside a flight tick is the order above (the bias update uses this
tick's e; the integration and ``rate`` use the updated bias).

Reasons for each rule:
* Why an accelerometer correction can work in a hover although the
  accelerometer mostly sees thrust: the specific force is (thrust + aero
  force) / m.  An un-accelerated vehicle carries its weight with exactly that,
  so the averaged specific force points along world up and reveals the tilt.
  A tilt error first accelerates the vehicle sideways (then f is still along
  the thrust axis and e ~ 0); the EDF ram drag and the body cross-flow drag
  build up until they balance the tilted thrust, and only then does f point up
  again.  With the attitude loop holding the estimate upright, true tilt x and
  drift speed v obey dx/dt = -kp v / (tau_d g), dv/dt = g x - v / tau_d
  (tau_d = m / (dD/dv), the drag time constant): damping zeta = 1 / (2 sqrt(kp
  tau_d)), so 1 / kp must stay longer than tau_d (the architect's rule) - the
  6-DOF models both drags (aero.vtol_*) to judge it honestly.
* Low-pass at accel_lpf_hz: the correction must follow the drag-balanced mean,
  not vibration or the short accelerations of the attitude loop.
* Gate on |a_lpf| (the vector that is used, not the raw sample): while the
  vehicle accelerates vertically (take-off ramp, T/W 1.19 climb start,
  saturated recoveries) |f| != g and f is not "up" - no correction, no bias
  learning, the estimate is gyro-only exactly as in v0.6.3.
* The in-flight bias change is bounded relative to the last pad value (b_pad),
  not the total: the pad average measures the static offset with the vehicle
  still (not limited, as in v0.6.3); the bound stops a misleading accelerometer
  (a long horizontal acceleration that passes the gate) from walking the bias
  beyond the drift the sensor can have (bias_limit_dps 0.6 > the 0.5 deg/s
  wander bound of the task brief).
* Yaw is not observable (no magnetometer): e is horizontal in the world, so it
  never turns the heading; the yaw estimate drifts with the residual z bias.

v0.8 NAV tick order (architect, params.json flight_computer.nav.schedule_note; the VTOL build): the v0.7.1
integration measured IMU sample -> vane 5.2 ms (> the 6-DOF's 4.71 ms) and NAV tick -> vane 4.7 ms (> 4.0 ms)
because the whole Mahony update, the altitude filter and the baro math ran before the frame left the NAV.  From v0.8
the NAV publishes the cheap gyro prediction and corrects afterwards; per NAV tick k, in this order (normative):

  1. IMU burst read: gyro g_k, specific force a_k (and the baro state machine's bus step, as before).
  2. PREDICT (``GatedMahonyEstimator.predict``): rate_k = g_k - b_{k-1};
         flight:  q_pub = q_{k-1} (x) exp(rate_k dt)          (``quat.integrate``; the corrected q of tick k-1)
         pad:     q_pub = q_{k-1}                             (the levelled pad attitude, no integration)
     cos_tilt / tilt from q_pub (``sim.nlink_proto.nav3_tilt``).
  3. PUBLISH the nlink v3 frame: q_pub, rate_k, a_k, the altitude filter's h / v_z of tick k-1, cos_tilt, tilt, the
     flags (IMU_OK of this read; ALIGNED / EST_VALID / EST_GATE / BARO_NEW as left by tick k-1) and the GPS slot
     (part k mod 5 of the snapshot latched at the last 50 Hz step).
  4. CORRECT (``GatedMahonyEstimator.update`` - the v0.7.1 equations unchanged, from q_{k-1}, b_{k-1}): q_k, b_k.
     The prediction of tick k+1 starts from q_k, so the correction reaches the FCU one tick (4 ms) late - immaterial
     for a correction whose time constant is 1 / kp (>= 6 s).
  5. Altitude filter (``sim.state_machine.VtolAltitudeFilter``) with a_z,world from q_k (corrected) -> h_k, v_z,k for the
     frame of tick k+1.
  6. GPS (``sim.gps_nav``): parse the UART bytes, configurator, epochs; ``NavGpsFilter.tick`` with a_k, q_k and
     g_k - b_k (fusion at k mod 5 == 0, the 50 Hz step + snapshot at k mod 5 == 4).
  7. Background (barometer math: BMP280, or the MS5611 build option).
Consequences, stated: the attitude the FCU flies on is the integration of this tick's gyro from last tick's corrected
attitude (identical to v0.7.1 between corrections); h / v_z are one tick (4 ms) older than in v0.7.1.  The ballistic
NAV keeps the v0.6.3 order and ``PadAlignGyroEstimator`` bit for bit.

Invariant: no number lives here - every constant arrives as an argument from
``sim.sim6dof.SensorConfig`` (where each one is labelled datasheet / task
brief / ASSUMED) or from ``params.json``.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.signal import lfilter

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import quat
except ImportError:  # pragma: no cover
    import quat  # type: ignore

DEG = math.pi / 180.0


# --------------------------------------------------------------------------- #
# IMU measurement model
# --------------------------------------------------------------------------- #
def quantize_clip(x: np.ndarray, lsb: float, full_scale: float) -> np.ndarray:
    """int16 ADC: counts = clip(round(x / lsb), -32768, 32767) and also within
    +-full_scale; returns counts * lsb (same unit as ``x``)."""
    c_max = min(32767.0, math.floor(full_scale / lsb))
    c_min = max(-32768.0, -math.floor(full_scale / lsb))
    # scalar loop: ~10x faster than numpy for the 3-vectors of the inner loop
    out = []
    for xi in np.asarray(x, dtype=float).ravel().tolist():
        c = float(round(xi / lsb))
        out.append((c_max if c > c_max else (c_min if c < c_min else c)) * lsb)
    return np.array(out).reshape(np.shape(x))


def white_noise_sigma(density: float, dlpf_tau_s: float, sample_hz: float) -> float:
    """Per-sample standard deviation of white noise of the given density
    [unit/sqrt(Hz)] after a first-order lag with time constant ``dlpf_tau_s``
    (equivalent noise bandwidth 1/(4 tau)); without a lag the Nyquist band."""
    bw = 1.0 / (4.0 * dlpf_tau_s) if dlpf_tau_s > 0.0 else sample_hz / 2.0
    return density * math.sqrt(bw)


def gyro_bias_process(rng: np.random.Generator, n: int, dt: float, sigma: float, tau_s: float,
                      bound: float, ramp: np.ndarray | None = None, ramp_start_index: int = 0) -> np.ndarray:
    """(n, 3) slowly varying gyro bias [rad/s] sampled every ``dt``.

    Per axis a first-order Gauss-Markov process b_{k+1} = phi b_k + sigma
    sqrt(1 - phi^2) w_k (phi = exp(-dt/tau)) started from its stationary
    distribution, plus an optional deterministic ramp [rad/s^2] from
    ``ramp_start_index`` on (worst-case study), clipped to +-``bound``."""
    b = np.zeros((n, 3))
    if sigma > 0.0 and tau_s > 0.0:
        phi = math.exp(-dt / tau_s)
        w = rng.normal(0.0, sigma * math.sqrt(1.0 - phi * phi), size=(n, 3))
        b0 = rng.normal(0.0, sigma, size=3)
        for i in range(3):
            b[:, i] = lfilter([1.0], [1.0, -phi], w[:, i], zi=[phi * b0[i]])[0]
    if ramp is not None:
        t_rel = np.maximum(np.arange(n) - ramp_start_index, 0) * dt
        b += t_rel[:, None] * np.asarray(ramp, dtype=float)[None, :]
    return np.clip(b, -bound, bound)


class Mpu6050:
    """Gyro + accelerometer sample model (white noise, bias, clip, quantise)."""

    def __init__(self, gyro_lsb_per_dps: float, gyro_range_dps: float, accel_lsb_per_g: float,
                 accel_range_g: float, g0: float):
        self.g_lsb = DEG / gyro_lsb_per_dps                 # rad/s per count
        self.g_fs = gyro_range_dps * DEG
        self.a_lsb = g0 / accel_lsb_per_g                   # m/s^2 per count
        self.a_fs = accel_range_g * g0
        # largest reading the int16 gyro can return (clip level)
        self.gyro_max = min(32767.0, math.floor(self.g_fs / self.g_lsb)) * self.g_lsb

    def gyro_saturated(self, gyro: np.ndarray) -> bool:
        return bool(np.max(np.abs(gyro)) >= self.gyro_max - 0.5 * self.g_lsb)

    def gyro(self, omega_filtered: np.ndarray, bias: np.ndarray, noise: np.ndarray) -> np.ndarray:
        return quantize_clip(np.asarray(omega_filtered) + bias + noise, self.g_lsb, self.g_fs)

    def accel(self, specific_force_b: np.ndarray, bias: np.ndarray, noise: np.ndarray) -> np.ndarray:
        return quantize_clip(np.asarray(specific_force_b) + bias + noise, self.a_lsb, self.a_fs)


# --------------------------------------------------------------------------- #
# the firmware estimator
# --------------------------------------------------------------------------- #
def level_from_accel(f_b: np.ndarray) -> np.ndarray:
    """Attitude with yaw 0 whose gravity reaction matches the body specific force
    ``f_b`` (vehicle at rest): roll = atan2(f_y, f_z), pitch = atan2(-f_x, |f_yz|)."""
    fx, fy, fz = float(f_b[0]), float(f_b[1]), float(f_b[2])
    roll = math.atan2(fy, fz)
    pitch = math.atan2(-fx, math.hypot(fy, fz))
    return quat.from_euler(roll, pitch, 0.0)


class PadAlignGyroEstimator:
    """Pad alignment (gyro bias + accelerometer levelling) then gyro-only
    quaternion integration - see the module docstring for the equations."""

    def __init__(self, dt: float, pad_avg_tau_s: float):
        self.dt = float(dt)
        self.n_avg = max(1, int(round(pad_avg_tau_s / self.dt)))
        self.n = 0
        self.bias = np.zeros(3)
        self.f_mean = np.zeros(3)
        self.q = quat.IDENTITY.copy()
        self.aligned = False

    def update(self, gyro: np.ndarray, accel: np.ndarray, on_pad: bool) -> np.ndarray:
        g = np.asarray(gyro, dtype=float)
        if on_pad:
            self.n = min(self.n + 1, self.n_avg)
            self.bias = self.bias + (g - self.bias) / self.n
            self.f_mean = self.f_mean + (np.asarray(accel, dtype=float) - self.f_mean) / self.n
            self.q = level_from_accel(self.f_mean)
            self.aligned = True
        else:
            self.q = quat.integrate(self.q, g - self.bias, self.dt)
        return self.q.copy()

    def rate(self, gyro: np.ndarray) -> np.ndarray:
        """Bias-corrected body rate for the rate PID."""
        return np.asarray(gyro, dtype=float) - self.bias


def up_body(q: np.ndarray) -> np.ndarray:
    """World up expressed in the body frame, R(q)^T e_z = the third row of R(q) (unit for a unit q)."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return np.array([2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)])


class GatedMahonyEstimator:
    """VTOL NAV attitude estimator (v0.7.1): pad alignment as ``PadAlignGyroEstimator``, then a gated Mahony correction
    with in-flight gyro-bias learning - see the module docstring for every equation and the reason for each rule.
    Same interface as ``PadAlignGyroEstimator`` (``update``, ``rate``, ``bias``, ``q``, ``n``, ``n_avg``, ``aligned``)."""

    def __init__(self, dt: float, pad_avg_tau_s: float, kp: float, ki: float, accel_gate: float, accel_lpf_hz: float,
                 bias_limit_rad_s: float, g: float = 9.80665):
        if not (dt > 0 and kp >= 0 and ki >= 0 and accel_gate > 0 and accel_lpf_hz > 0 and bias_limit_rad_s >= 0 and g > 0):
            raise ValueError("GatedMahonyEstimator: need dt > 0, kp, ki >= 0, accel_gate > 0, accel_lpf_hz > 0, "
                             "bias_limit >= 0, g > 0")
        self.dt = float(dt)
        self.n_avg = max(1, int(round(pad_avg_tau_s / self.dt)))
        self.kp, self.ki = float(kp), float(ki)
        self.accel_gate = float(accel_gate)
        self.g = float(g)
        self.alpha_a = 1.0 - math.exp(-2.0 * math.pi * float(accel_lpf_hz) * self.dt)
        self.bias_limit = float(bias_limit_rad_s)
        self.n = 0
        self.bias = np.zeros(3)
        self.b_pad = np.zeros(3)
        self.f_mean = np.zeros(3)
        self.a_lpf = np.zeros(3)
        self.q = quat.IDENTITY.copy()
        self.aligned = False
        self.gate = False                       # last flight tick: accelerometer used
        self.e = np.zeros(3)                    # last flight tick: correction error [rad]
        self.was_pad = False                    # previous tick in pad mode (re-entry detection)
        self.flight_ticks = 0
        self.gated_ticks = 0                    # flight ticks with the gate open (diagnostics)

    @classmethod
    def from_params(cls, p: dict, dt: float, pad_avg_tau_s: float, g: float = 9.80665, **overrides) -> "GatedMahonyEstimator":
        """params.json ``mission.vtol.nav_estimator`` (kp_ASSUMED, ki_ASSUMED, accel_gate, accel_lpf_hz_ASSUMED,
        bias_limit_dps); ``pad_avg_tau_s`` is the firmware pad-averaging constant (sim: SensorConfig.pad_avg_tau_s).
        Keyword overrides replace kp / ki / accel_gate / accel_lpf_hz / bias_limit_rad_s (studies)."""
        ne = p["mission"]["vtol"]["nav_estimator"]
        if str(ne.get("type", "gated_mahony")) != "gated_mahony":
            raise ValueError(f"mission.vtol.nav_estimator.type {ne.get('type')!r} is not 'gated_mahony'")
        kw = dict(kp=float(ne["kp_ASSUMED"]), ki=float(ne["ki_ASSUMED"]), accel_gate=float(ne["accel_gate"]),
                  accel_lpf_hz=float(ne["accel_lpf_hz_ASSUMED"]), bias_limit_rad_s=math.radians(float(ne["bias_limit_dps"])))
        kw.update({k: float(v) for k, v in overrides.items() if v is not None})
        return cls(dt, pad_avg_tau_s, g=g, **kw)

    def predict(self, gyro: np.ndarray, on_pad: bool) -> tuple[np.ndarray, np.ndarray]:
        """v0.8 NAV order step 2 (module docstring): (q_pub, rate) from the state of the previous tick, no state change.
        Flight: q (x) exp((g - b) dt); pad: the levelled q as it is.  rate = g - b in both."""
        rate = np.asarray(gyro, dtype=float) - self.bias
        if on_pad:
            return self.q.copy(), rate
        return quat.integrate(self.q, rate, self.dt), rate

    def update(self, gyro: np.ndarray, accel: np.ndarray, on_pad: bool) -> np.ndarray:
        g = np.asarray(gyro, dtype=float)
        a = np.asarray(accel, dtype=float)
        if on_pad:
            if not self.was_pad and self.flight_ticks > 0:
                self.f_mean = self.a_lpf.copy()          # re-entry: level from the landing spot, not the previous pad
            self.n = min(self.n + 1, self.n_avg)
            self.bias = self.bias + (g - self.bias) / self.n
            self.f_mean = self.f_mean + (a - self.f_mean) / self.n
            self.q = level_from_accel(self.f_mean)
            self.aligned = True
            self.a_lpf = self.f_mean.copy()
            self.b_pad = self.bias.copy()
            self.e = np.zeros(3)
            self.gate = False
            self.was_pad = True
            return self.q.copy()
        self.was_pad = False
        self.flight_ticks += 1
        self.a_lpf = self.a_lpf + self.alpha_a * (a - self.a_lpf)
        na = math.sqrt(float(self.a_lpf @ self.a_lpf))
        self.gate = na > 0.0 and abs(na - self.g) < self.accel_gate * self.g
        if self.gate:
            self.gated_ticks += 1
            ah = self.a_lpf / na
            v = up_body(self.q)
            self.e = np.array([ah[1] * v[2] - ah[2] * v[1], ah[2] * v[0] - ah[0] * v[2], ah[0] * v[1] - ah[1] * v[0]])
            d = self.bias - self.b_pad - self.ki * self.e * self.dt
            lim = self.bias_limit
            self.bias = self.b_pad + np.array([lim if x > lim else (-lim if x < -lim else x) for x in d.tolist()])
        else:
            self.e = np.zeros(3)
        self.q = quat.integrate(self.q, g - self.bias + self.kp * self.e, self.dt)
        return self.q.copy()

    def rate(self, gyro: np.ndarray) -> np.ndarray:
        """Bias-corrected body rate for the rate PID: g_k - b_hat."""
        return np.asarray(gyro, dtype=float) - self.bias


def up_error_rad(q_true: np.ndarray, q_est: np.ndarray) -> float:
    """Tilt-estimate error: angle between the true and the estimated world-up
    direction expressed in the body frame (independent of the yaw reference)."""
    def up_b(q: np.ndarray) -> tuple[float, float, float]:      # R(q)^T e_z = third row of R(q)
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return 2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)

    a, b = up_b(q_true), up_b(q_est)
    cx = a[1] * b[2] - a[2] * b[1]
    cy = a[2] * b[0] - a[0] * b[2]
    cz = a[0] * b[1] - a[1] * b[0]
    return math.atan2(math.sqrt(cx * cx + cy * cy + cz * cz), a[0] * b[0] + a[1] * b[1] + a[2] * b[2])
