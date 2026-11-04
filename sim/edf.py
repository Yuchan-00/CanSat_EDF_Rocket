"""EDF thrust model (SPEC §8.7), residual swirl torque (§8.4) and vane
aerodynamics (§8.3).

Equations implemented here (all SI):

* thrust command      T_cmd(u) = T_max u^n,  n = ``edf.throttle_exponent``
* thrust lag          dT/dt = (T_cmd - T) / tau,  tau = ``edf.thrust_time_constant_s``
* electrical power    P(u) = P_nom u^3   (energy accounting only)
* exhaust velocity    V_e = sqrt(T / (rho A_e)),  A_e = pi/4 (D^2 - (f_hub D)^2)
* dynamic pressure    q_e = 1/2 rho V_e^2 = T / (2 A_e)   (rho cancels)
* vane lift           C_L(d) = C_La d for |d| <= stall, C_L(+-stall) beyond
* vane side force     F_i = q_e S_v C_L(d_i) along e_theta(theta_i) = (-sin, cos, 0)
                      applied at (r_v cos, r_v sin, z_hinge)
* swirl torque        tau_swirl = sign * f_swirl * P/omega with omega = omega_max u
                      and omega_max = KV * V_nom * 2 pi / 60, so that
                      P/omega = (P_nom/omega_max) u^2 = (P_nom/omega_max) T/T_max

v0.9 plant additions (VTOL hardware round; each one off at 0, so the v0.8.1 plant is reproduced bit for bit):

* rotor speed from the thrust model   omega(T) = omega_ref sqrt(T / T_fan), T ~ omega^2 (the fan law), omega_ref =
  KV V_nom 2 pi / 60 (the no-load speed at 22.2 V - ASSUMED at the fan's T_max at 22.2 V, an upper bound of the loaded
  speed, so the gyroscopic terms are on the large side); ``RotorModel``
* rotor angular momentum              h = -s I_rot omega z_b (s = +1 for a fan turning CW seen from the intake, the swirl
  sign: its angular velocity points along -z_b); gyroscopic moment on the body -omega_b x h; spool-up reaction -dh/dt =
  +s I_rot domega/dt z_b (the motor torque that accelerates the rotor pushes the stator the other way - the same sign
  as the residual swirl torque); I_rot = edf.thrust_v09.rotor_inertia_kg_m2_ASSUMED (4.5e-5)
* direct-drive vane play              the vane follows the servo output through a play (backlash) operator of total
  width b = lost motion (socket fit, 0.65 deg) + gear backlash (1 deg ASSUMED): the vane angle stays until the servo
  output is more than b / 2 away, then it is dragged along (``vane_play``); 1:1 kinematics (no linkage)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def swirl_sign_from_rotation(rotation_text: str) -> float:
    """+1 for a fan turning CW viewed from the intake (rotor angular momentum
    along -z leaves with the exhaust, reaction on the vehicle is +z), -1 for
    CCW.  ``edf.rotation`` is free text; it must contain 'CW' or 'CCW'."""
    t = rotation_text.upper()
    if "CCW" in t:
        return -1.0
    if "CW" in t:
        return 1.0
    raise ValueError(f"edf.rotation {rotation_text!r} must say CW or CCW")


@dataclass(frozen=True)
class EDFModel:
    t_max_n: float
    throttle_exp: float
    tau_s: float
    p_nom_w: float
    omega_max_rad_s: float
    swirl_fraction: float
    swirl_sign: float
    a_e_m2: float
    rho: float

    @classmethod
    def from_params(cls, p: dict, duct_id_mm: float) -> "EDFModel":
        e = p["edf"]
        d = duct_id_mm * 1e-3
        hub = e["exit_hub_fraction"] * d
        return cls(
            t_max_n=float(e["thrust_nominal_N"]),
            throttle_exp=float(e["throttle_exponent"]),
            tau_s=float(e["thrust_time_constant_s"]),
            p_nom_w=float(e["power_nominal_W"]),
            omega_max_rad_s=float(e["kv"]) * float(e["voltage_nominal_V"]) * 2.0 * math.pi / 60.0,
            swirl_fraction=float(e["residual_swirl_torque_fraction_ASSUMED"]),
            swirl_sign=swirl_sign_from_rotation(e["rotation"]),
            a_e_m2=math.pi / 4.0 * (d * d - hub * hub),
            rho=float(p["aero"]["rho_kg_m3"]),
        )

    # ---- thrust ------------------------------------------------------------
    def thrust_steady(self, u: float) -> float:
        """T_cmd = T_max u^n (u clipped to [0, 1])."""
        u = min(max(float(u), 0.0), 1.0)
        return self.t_max_n * u ** self.throttle_exp

    def thrust_rate(self, thrust_n: float, u: float) -> float:
        """dT/dt of the first-order lag."""
        return (self.thrust_steady(u) - thrust_n) / self.tau_s

    def step(self, thrust_n: float, u: float, dt: float) -> float:
        """Exact discrete update of the lag over ``dt`` with ``u`` held."""
        return thrust_n + (self.thrust_steady(u) - thrust_n) * (1.0 - math.exp(-dt / self.tau_s))

    def power(self, u: float) -> float:
        u = min(max(float(u), 0.0), 1.0)
        return self.p_nom_w * u ** 3

    def throttle_for_thrust(self, thrust_n: float) -> float:
        """Inverse of ``thrust_steady``."""
        return (max(thrust_n, 0.0) / self.t_max_n) ** (1.0 / self.throttle_exp)

    # ---- exhaust -----------------------------------------------------------
    def exhaust_velocity(self, thrust_n: float) -> float:
        return math.sqrt(max(thrust_n, 0.0) / (self.rho * self.a_e_m2))

    def dynamic_pressure(self, thrust_n: float) -> float:
        """q_e = 1/2 rho V_e^2 = T / (2 A_e)."""
        return max(thrust_n, 0.0) / (2.0 * self.a_e_m2)

    # ---- swirl -------------------------------------------------------------
    def fan_speed(self, u: float) -> float:
        return self.omega_max_rad_s * min(max(float(u), 0.0), 1.0)

    def swirl_torque_from_throttle(self, u: float) -> float:
        """sign f P(u)/omega(u) = sign f (P_nom/omega_max) u^2."""
        u = min(max(float(u), 0.0), 1.0)
        return self.swirl_sign * self.swirl_fraction * self.p_nom_w / self.omega_max_rad_s * u * u

    def swirl_torque(self, thrust_n: float) -> float:
        """Same disturbance expressed through the (lagged) thrust: u^2 = T/T_max."""
        return self.swirl_sign * self.swirl_fraction * self.p_nom_w / self.omega_max_rad_s * max(thrust_n, 0.0) / self.t_max_n


@dataclass(frozen=True)
class RotorModel:
    """The EDF rotor's speed and angular momentum from the (lagged) thrust (v0.9, see the module docstring)."""

    i_rot_kg_m2: float
    omega_ref_rad_s: float          # speed at t_fan_n
    t_fan_n: float                  # the plant fan's T_max at 22.2 V
    sign: float                     # swirl sign (+1: CW seen from the intake)
    t_floor_frac: float = 0.02      # domega/dT is evaluated with T >= this x t_fan_n (the start from rest is on the pad)

    @classmethod
    def from_params(cls, p: dict, t_fan_n: float, i_rot_kg_m2: float | None = None) -> "RotorModel":
        e = p["edf"]
        tv = e.get("thrust_v09") or {}
        i_rot = float(tv.get("rotor_inertia_kg_m2_ASSUMED", 0.0)) if i_rot_kg_m2 is None else float(i_rot_kg_m2)
        return cls(i_rot, float(e["kv"]) * float(e["voltage_nominal_V"]) * 2.0 * math.pi / 60.0, float(t_fan_n),
                   swirl_sign_from_rotation(e["rotation"]))

    def omega(self, thrust_n: float) -> float:
        return self.omega_ref_rad_s * math.sqrt(max(thrust_n, 0.0) / self.t_fan_n)

    def omega_rate(self, thrust_n: float, thrust_rate: float) -> float:
        """domega/dt = omega_ref / (2 sqrt(T_fan T)) dT/dt, T floored at t_floor_frac T_fan."""
        t = max(thrust_n, self.t_floor_frac * self.t_fan_n)
        return self.omega_ref_rad_s * thrust_rate / (2.0 * math.sqrt(self.t_fan_n * t))

    def h_z(self, thrust_n: float) -> float:
        """Body-z angular momentum of the rotor [N m s] (-s I omega)."""
        return -self.sign * self.i_rot_kg_m2 * self.omega(thrust_n)

    def gyro_moment(self, omega_b: tuple[float, float, float], thrust_n: float) -> tuple[float, float, float]:
        """-omega_b x h with h = (0, 0, h_z)."""
        hz = self.h_z(thrust_n)
        return -omega_b[1] * hz, omega_b[0] * hz, 0.0

    def reaction_moment_z(self, thrust_n: float, thrust_rate: float) -> float:
        """-dh_z/dt = +s I domega/dt (spool-up reaction on the body)."""
        return self.sign * self.i_rot_kg_m2 * self.omega_rate(thrust_n, thrust_rate)


def vane_play(vane_rad: float, servo_rad: float, half_width_rad: float) -> float:
    """Play (backlash) operator: the vane angle stays inside [servo - h, servo + h] (h = half the total play) and moves
    only when the servo output drags it.  h = 0 returns the servo angle exactly."""
    lo, hi = servo_rad - half_width_rad, servo_rad + half_width_rad
    return lo if vane_rad < lo else (hi if vane_rad > hi else vane_rad)


def vane_play_deg_from_params(p: dict) -> float:
    """Total play [deg] of the direct-drive vane (params vane.direct_drive_play: lost motion + gear backlash); 0 for the
    pushrod linkage (the v0.8 build, whose play was never modelled) or when params carry no play."""
    if (p.get("vane_drive") or {}).get("mode") != "direct":
        return 0.0
    dp = (p.get("vane") or {}).get("direct_drive_play") or {}
    return float(dp.get("lost_motion_deg_ASSUMED", 0.0)) + float(dp.get("gear_backlash_deg_ASSUMED", 0.0))


@dataclass(frozen=True)
class VaneModel:
    s_v_m2: float
    cl_alpha: float
    stall_rad: float
    r_v_m: float
    z_hinge_m: float
    thetas_rad: tuple[float, ...]
    delta_cmd_max_rad: float
    delta_mech_max_rad: float

    @classmethod
    def from_params(cls, p: dict, L) -> "VaneModel":
        v = p["vane"]
        return cls(
            s_v_m2=L.vane_area_mm2 * 1e-6,
            cl_alpha=float(v["cl_alpha_per_rad"]),
            stall_rad=math.radians(v["stall_deg"]),
            r_v_m=float(v["force_centroid_r"]) * 1e-3,
            z_hinge_m=L.vane_hinge_z * 1e-3,
            thetas_rad=tuple(math.radians(t) for t in L.vane_thetas),
            delta_cmd_max_rad=math.radians(v["max_deflection_cmd_deg"]),
            delta_mech_max_rad=math.radians(v["max_deflection_mech_deg"]),
        )

    @property
    def count(self) -> int:
        return len(self.thetas_rad)

    def cl(self, delta_rad: np.ndarray | float) -> np.ndarray | float:
        """C_L(delta): linear to stall, flat beyond (sign kept)."""
        return self.cl_alpha * np.clip(delta_rad, -self.stall_rad, self.stall_rad)

    def e_theta(self) -> np.ndarray:
        """(N,3) unit force directions e_theta(theta_i) = (-sin, cos, 0)."""
        th = np.asarray(self.thetas_rad)
        return np.stack([-np.sin(th), np.cos(th), np.zeros_like(th)], axis=1)

    def positions(self) -> np.ndarray:
        """(N,3) force application points (r_v cos, r_v sin, z_hinge) [m]."""
        th = np.asarray(self.thetas_rad)
        return np.stack([self.r_v_m * np.cos(th), self.r_v_m * np.sin(th), np.full_like(th, self.z_hinge_m)], axis=1)

    def force_slope(self, q_e: float) -> float:
        """dF/d delta = q_e S_v C_La [N/rad] in the linear regime."""
        return q_e * self.s_v_m2 * self.cl_alpha

    def forces(self, q_e: float, deltas_rad: np.ndarray) -> np.ndarray:
        """Per-vane side-force magnitudes F_i = q_e S_v C_L(delta_i)."""
        return q_e * self.s_v_m2 * self.cl(np.asarray(deltas_rad, dtype=float))

    def moment_matrix(self, z_cg_m: float) -> np.ndarray:
        """(3,N) matrix B with M_b = B @ F (cross products r_i x e_theta_i about
        the CG).  Row/column meaning: column i = (l_v cos, l_v sin, r_v) for
        vane i where l_v = z_cg - z_hinge - this reproduces SPEC §8.4."""
        r = self.positions() - np.array([0.0, 0.0, z_cg_m])
        return np.cross(r, self.e_theta()).T

    def forces_moments(self, q_e: float, deltas_rad: np.ndarray, z_cg_m: float) -> tuple[np.ndarray, np.ndarray]:
        """Total body force (3,) and moment about the CG (3,) from the vanes."""
        F = self.forces(q_e, deltas_rad)
        return self.e_theta().T @ F, self.moment_matrix(z_cg_m) @ F
