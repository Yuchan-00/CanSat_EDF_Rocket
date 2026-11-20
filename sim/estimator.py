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
