"""Linear stability margins of the attitude loop with the flight-computer timing
(cross-check of the 6-DOF sensitivity runs; small-signal, hover, unsaturated).

Loop broken at the moment command, one axis (x/y pitch or z roll):

  L(jw) = C(z) * F_imu(jw) * A(jw) * K_m / (I jw),        z = exp(jw dt)

  C(z)   = (Kp + Ki dt/(1 - z^-1)) (att_kp dt/(1 - z^-1) + H_g(z))
           + Kd (1 - z^-1)/dt H_d(z) H_g(z)                         (sim.controller, discrete)
           H(z) = alpha / (1 - (1 - alpha) z^-1),  alpha = dt / (1/(2 pi f_c) + dt)
           the attitude estimate is the gyro integral dt/(1 - z^-1)  (sim.estimator, gyro-only)
  F_imu  = 1 / (tau_dlpf s + 1)                                      MPU-6050 on-chip DLPF
  A(jw)  = exp(-s (d dt + wait + pulse)) ZOH_Tf(s) / (tau_servo s + 1)
           ZOH_T(s) = (1 - exp(-s T)) / (s T); without frame latch T = dt and wait = pulse = 0
  K_m    = 1 (pitch: the mixer inverts the plant exactly at the hover thrust),
           roll_priority_scale (roll)
  I      = I_xx or I_zz about the CG

Returned: gain-crossover frequency, phase margin, the upper gain margin (first
-180 deg crossing above crossover) and the lower one (gain-reduction margin
of this conditionally stable loop: the double integrator at low frequency).
The servo rate limit and vane saturation are not in this model; the 6-DOF
gain-margin search in ``run_scenarios`` includes them.

Invariant: every input comes from the gains dict, the Vehicle and the timing
summary; nothing is typed here.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np


def _alpha(f_c: float, dt: float) -> float:
    return 1.0 if f_c <= 0.0 else dt / (1.0 / (2.0 * math.pi * f_c) + dt)


def loop_response(w: np.ndarray, g: dict, axis: str, dt: float, inertia: float, k_m: float, tau_servo: float,
                  tau_dlpf: float, delay_s: float, frame_hz: float) -> np.ndarray:
    """Complex L(jw) on the grid ``w`` [rad/s].  ``delay_s`` = compute delay +
    wait to the frame edge + pulse decode; the hold is added here."""
    sfx = "xy" if axis == "xy" else "z"
    kp, ki, kd = g[f"rate_kp_{sfx}"], g[f"rate_ki_{sfx}"], g[f"rate_kd_{sfx}"]
    att = g[f"att_kp_{sfx}"]
    s = 1j * w
    zi = np.exp(-s * dt)                                      # z^-1
    ag, ad = _alpha(g["gyro_lpf_hz"], dt), _alpha(g["d_term_lpf_hz"], dt)
    h_g = ag / (1.0 - (1.0 - ag) * zi)
    h_d = ad / (1.0 - (1.0 - ad) * zi)
    integ = dt / (1.0 - zi)
    c = (kp + ki * integ) * (att * integ + h_g) + kd * (1.0 - zi) / dt * h_d * h_g
    f_imu = 1.0 / (tau_dlpf * s + 1.0)
    t_hold = 1.0 / frame_hz if frame_hz > 0.0 else dt
    zoh = (1.0 - np.exp(-s * t_hold)) / (s * t_hold)
    act = np.exp(-s * delay_s) * zoh / (tau_servo * s + 1.0)
    return c * f_imu * act * k_m / (inertia * s)


def margins(w: np.ndarray, L: np.ndarray) -> dict[str, Any]:
    mag = np.abs(L)
    ph = np.degrees(np.unwrap(np.angle(L)))
    ph -= 360.0 * np.round((ph[0] + 270.0) / 360.0)          # low-frequency phase of the 1/s^3 loop = -270 deg
    above = mag >= 1.0
    idx = np.flatnonzero(above[:-1] & ~above[1:])            # |L| falls through 1
    out: dict[str, Any] = {"crossover_rad_s": None, "crossover_hz": None, "phase_margin_deg": None,
                           "gain_margin_upper_db": None, "gain_margin_upper_x": None,
                           "phase_crossover_upper_hz": None, "gain_margin_lower_x": None}
    if idx.size == 0:
        return out
    i = int(idx[-1])
    f = (0.0 - math.log(mag[i])) / (math.log(mag[i + 1]) - math.log(mag[i]))
    wc = w[i] * (w[i + 1] / w[i]) ** f
    pc = ph[i] + f * (ph[i + 1] - ph[i])
    out["crossover_rad_s"] = float(wc)
    out["crossover_hz"] = float(wc / (2.0 * math.pi))
    out["phase_margin_deg"] = float(((pc + 180.0) + 180.0) % 360.0 - 180.0)
    # -180 deg crossings: L on the negative real axis
    re, im = L.real, L.imag
    cross = np.flatnonzero((np.sign(im[:-1]) != np.sign(im[1:])) & (re[:-1] < 0.0))
    up = [c for c in cross if w[c] > wc]
    lo = [c for c in cross if w[c] < wc]
    if up:
        c = up[0]
        gm = 1.0 / mag[c]
        out["gain_margin_upper_x"] = float(gm)
        out["gain_margin_upper_db"] = float(20.0 * math.log10(gm))
        out["phase_crossover_upper_hz"] = float(w[c] / (2.0 * math.pi))
    if lo:
        out["gain_margin_lower_x"] = float(1.0 / mag[lo[-1]])
    return out


def axis_margins(g: dict, axis: str, dt: float, inertia: float, k_m: float, tau_servo: float, tau_dlpf: float,
                 delay_s: float, frame_hz: float, n: int = 6000) -> dict[str, Any]:
    w = np.logspace(math.log10(0.05), math.log10(0.999 * math.pi / dt), n)
    L = loop_response(w, g, axis, dt, inertia, k_m, tau_servo, tau_dlpf, delay_s, frame_hz)
    m = margins(w, L)
    m["delay_s"] = float(delay_s)
    m["hold_s"] = float(1.0 / frame_hz if frame_hz > 0.0 else dt)
    return m
