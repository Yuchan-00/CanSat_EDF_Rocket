"""Altitude complementary filters (current) and the v0.3 mission state machine
(SUPERSEDED).

``AltitudeFilter`` is current: the v0.4 flight computer runs it for the
telemetry (h, v_z) and the interlocks of ``sim.onboard``; the firmware mirrors it
(``firmware/atmega128_fc/src/core/alt_filter.c``).  It stays the ballistic
build's filter, bit for bit.

``VtolAltitudeFilter`` (v0.7.1, the VTOL NAV) adds an accelerometer z-bias state
learnt while the baro is healthy and the baro thrust compensation; its equations,
gains and the reason for each rule are in its own docstring.

``MissionStateMachine`` / ``MissionParams`` / ``MissionInputs`` / ``State`` below
are SUPERSEDED by the v0.4 onboard supervisor ``sim.onboard`` (SPEC §8.8: the
vehicle does attitude control only; launch / cut / release are ground commands,
``sim.ground_station_logic``).  Nothing in ``sim/`` flies them any more; they are
kept, unchanged, only because ``tools/crosscheck_controller.py`` and
``tools/gen_firmware_params.py`` still import them for the v0.3 firmware core
(``src/core/state_machine.c``).  Delete them (and their tests in
``sim/test_sim.py``) once those tools are ported to ``sim.onboard``.

v0.3 states and transitions (historical):

  IDLE    -- arm cmd & tilt < arm_tilt_max & V_cell >= min_cell_v --> ARMED
  ARMED   -- launch cmd --> LAUNCH      (u ramps 0 -> climb_throttle over throttle_ramp_s)
  LAUNCH  -- t - t_launch >= throttle_ramp_s --> ASCENT   (u = climb_throttle)
  ASCENT  -- h + coast_drag_factor v_z^2 / (2 g) >= target_altitude --> COAST (u = 0)
  COAST   -- v_z <= apogee_detect_vz for apogee_confirm_s OR t_coast >= coast_timeout_s --> DEPLOY
  DEPLOY  -- release servo released; after 1.0 s --> DESCENT
  DESCENT -- |a| spike seen & baro static for 3 s --> LANDED (disarm)
  LAUNCH/ASCENT -- tilt > tilt_abort_deg OR h > geofence OR V_cell < min OR RC kill --> ABORT
  ABORT   -- u = 0; after 0.5 s --> DEPLOY

Attitude control is active in LAUNCH/ASCENT only; in every other state the
vanes centre.  Numbers that §8.8 leaves open (the landing detector thresholds)
are module constants marked ASSUMED; everything else is ``params.mission`` /
``params.battery``.

Invariant: at most one transition per ``update`` call; every transition is
recorded in ``events`` with its time so the simulation report and the firmware
log can be compared line by line.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

G0 = 9.80665

# §8.8 leaves these unspecified - ASSUMED, tune against drop tests:
LANDING_ACCEL_SPIKE_MPS2 = 2.0 * G0     # | |a| - g | above this marks a touchdown impact
LANDING_BARO_STATIC_M = 0.5             # altitude must stay within this band ...
LANDING_STATIC_S = 3.0                  # ... for this long (spec: "baro static 3 s")
DEPLOY_TO_DESCENT_S = 1.0               # spec: "after 1.0 s"
ABORT_TO_DEPLOY_S = 0.5                 # spec: "then DEPLOY after 0.5 s"

# v0.7.1 VtolAltitudeFilter defaults (the caller may pass others; sim: SensorConfig, firmware: params_gen.h):
# bias pole 1 / 40 s - scratch study v071/pyref/alt_filter_study.py (synthetic 250 Hz NAV, baro 50 Hz 0.25 m RMS 0.1 s
# latency, 10 mg bias): in-flight bias-estimate std 0.0067 / 0.0034 / 0.0018 m/s^2 for 20 / 40 / 80 s, transient after
# a 1.5 m baro step at take-off (uncalibrated thrust error) 0.12 / 0.062 / 0.031 m/s^2; 40 s keeps the bias noise below
# the filter's own v noise over a 20 s dead-reckoned landing (0.0034 x 20 = 0.07 m/s vs ~0.1 m/s) and still learns
# half of a bias change within one ~30 s flight.
VTOL_ALT_BIAS_TAU_S_ASSUMED = 40.0
VTOL_ALT_PAD_BIAS_TAU_S_ASSUMED = 2.0      # = the estimator's pad averaging time constant (SensorConfig.pad_avg_tau_s)
VTOL_ALT_BIAS_LIMIT_MPS2_ASSUMED = 1.0     # ~100 mg: above the MPU-6050 zero-g initial tolerance (X/Y 50 mg, Z 80 mg,
                                           # product specification) - a larger "bias" is a baro fault, not the sensor


class State(IntEnum):
    IDLE = 0
    ARMED = 1
    LAUNCH = 2
    ASCENT = 3
    COAST = 4
    DEPLOY = 5
    DESCENT = 6
    LANDED = 7
    ABORT = 8


POWERED_STATES = (State.LAUNCH, State.ASCENT)
CONTROL_ACTIVE_STATES = (State.LAUNCH, State.ASCENT)
RELEASED_STATES = (State.DEPLOY, State.DESCENT, State.LANDED)


@dataclass
class MissionInputs:
    t: float                       # s
    h: float                       # m, estimated altitude above the pad
    v_z: float                     # m/s, estimated vertical speed
    tilt_deg: float                # from the attitude estimate
    accel_norm: float              # |specific force| m/s^2
    v_cell: float                  # V per cell
    arm_cmd: bool = False
    launch_cmd: bool = False
    disarm_cmd: bool = False
    kill_cmd: bool = False


@dataclass
class MissionOutputs:
    state: State
    throttle: float
    control_active: bool
    vanes_centre: bool
    release: bool
    abort_reason: str | None
    event: str | None = None


@dataclass
class MissionParams:
    target_altitude_m: float
    geofence_altitude_m: float
    climb_throttle: float
    throttle_ramp_s: float
    coast_drag_factor: float
    apogee_detect_vz_mps: float
    apogee_confirm_s: float
    coast_timeout_s: float
    tilt_abort_deg: float
    arm_tilt_max_deg: float
    min_cell_v: float
    coast_control_hold_s: float = 0.0   # attitude control stays on this long after cut-off (swirl decay)

    @classmethod
    def from_params(cls, p: dict, **overrides: float) -> "MissionParams":
        m = p["mission"]
        kw = dict(
            target_altitude_m=float(m["target_altitude_m"]),
            geofence_altitude_m=float(m["geofence_altitude_m"]),
            climb_throttle=float(m["climb_throttle"]),
            throttle_ramp_s=float(m["throttle_ramp_s"]),
            coast_drag_factor=float(m["coast_drag_factor"]),
            apogee_detect_vz_mps=float(m["apogee_detect_vz_mps"]),
            apogee_confirm_s=float(m["apogee_confirm_s"]),
            coast_timeout_s=float(m["coast_timeout_s"]),
            tilt_abort_deg=float(m["tilt_abort_deg"]),
            arm_tilt_max_deg=float(m["arm_tilt_max_deg"]),
            min_cell_v=float(p["battery"]["min_cell_v"]),
            coast_control_hold_s=float(m.get("coast_control_hold_s", 0.0)),
        )
        kw.update({k: float(v) for k, v in overrides.items() if v is not None})
        return cls(**kw)


class MissionStateMachine:
    """SPEC §8.8 with explicit timers.  Call ``update`` once per control cycle."""

    def __init__(self, mp: MissionParams, g: float = G0):
        self.mp = mp
        self.g = g
        self.state = State.IDLE
        self.t_state_entry = 0.0
        self.t_launch: float | None = None
        self.t_coast: float | None = None
        self.t_deploy: float | None = None
        self.t_abort: float | None = None
        self.t_landed: float | None = None
        self.apogee_timer = 0.0
        self.abort_reason: str | None = None
        self.impact_seen = False
        self.h_static_ref = 0.0
        self.t_static_start = 0.0
        self.events: list[tuple[float, str]] = []
        self._last_t: float | None = None

    # ---- helpers -----------------------------------------------------------
    def cutoff_altitude(self, h: float, v_z: float) -> float:
        """Predicted apogee h + coast_drag_factor v_z^2 / (2 g) (rising only)."""
        vz = max(v_z, 0.0)
        return h + self.mp.coast_drag_factor * vz * vz / (2.0 * self.g)

    def _enter(self, new: State, t: float, why: str) -> str:
        self.events.append((t, f"{self.state.name} -> {new.name}: {why}"))
        self.state = new
        self.t_state_entry = t
        return self.events[-1][1]

    def _abort_condition(self, inp: MissionInputs) -> str | None:
        if inp.tilt_deg > self.mp.tilt_abort_deg:
            return f"tilt {inp.tilt_deg:.1f} deg > {self.mp.tilt_abort_deg:.0f}"
        if inp.h > self.mp.geofence_altitude_m:
            return f"geofence h {inp.h:.1f} m > {self.mp.geofence_altitude_m:.0f}"
        if inp.v_cell < self.mp.min_cell_v:
            return f"V_cell {inp.v_cell:.2f} < {self.mp.min_cell_v:.2f}"
        if inp.kill_cmd:
            return "RC kill"
        return None

    def throttle(self, t: float) -> float:
        if self.state == State.LAUNCH and self.t_launch is not None:
            frac = min(max((t - self.t_launch) / self.mp.throttle_ramp_s, 0.0), 1.0) if self.mp.throttle_ramp_s > 0 else 1.0
            return self.mp.climb_throttle * frac
        if self.state == State.ASCENT:
            return self.mp.climb_throttle
        return 0.0

    # ---- main --------------------------------------------------------------
    def update(self, inp: MissionInputs) -> MissionOutputs:
        t = inp.t
        dt = 0.0 if self._last_t is None else max(t - self._last_t, 0.0)
        self._last_t = t
        s = self.state
        event: str | None = None

        if s == State.IDLE:
            if inp.arm_cmd and inp.tilt_deg < self.mp.arm_tilt_max_deg and inp.v_cell >= self.mp.min_cell_v:
                event = self._enter(State.ARMED, t, f"arm (tilt {inp.tilt_deg:.1f} deg, {inp.v_cell:.2f} V/cell)")
        elif s == State.ARMED:
            if inp.disarm_cmd:
                event = self._enter(State.IDLE, t, "disarm")
            elif inp.launch_cmd:
                self.t_launch = t
                event = self._enter(State.LAUNCH, t, "launch cmd")
        elif s in POWERED_STATES:
            why = self._abort_condition(inp)
            if why is not None:
                self.abort_reason = why
                self.t_abort = t
                event = self._enter(State.ABORT, t, why)
            elif s == State.LAUNCH and t - (self.t_launch or t) >= self.mp.throttle_ramp_s:
                event = self._enter(State.ASCENT, t, "ramp complete")
            elif s == State.ASCENT and self.cutoff_altitude(inp.h, inp.v_z) >= self.mp.target_altitude_m:
                self.t_coast = t
                self.apogee_timer = 0.0
                event = self._enter(State.COAST, t, f"cut-off at h {inp.h:.1f} m, v_z {inp.v_z:.1f} m/s "
                                                   f"(predicted {self.cutoff_altitude(inp.h, inp.v_z):.1f} m)")
        elif s == State.COAST:
            if inp.v_z <= self.mp.apogee_detect_vz_mps:
                self.apogee_timer += dt
            else:
                self.apogee_timer = 0.0
            if self.apogee_timer >= self.mp.apogee_confirm_s:
                self.t_deploy = t
                event = self._enter(State.DEPLOY, t, f"apogee confirmed (v_z {inp.v_z:.2f} m/s for "
                                                    f"{self.mp.apogee_confirm_s:.1f} s) at h {inp.h:.1f} m")
            elif t - (self.t_coast or t) >= self.mp.coast_timeout_s:
                self.t_deploy = t
                event = self._enter(State.DEPLOY, t, f"coast timeout {self.mp.coast_timeout_s:.1f} s at h {inp.h:.1f} m")
        elif s == State.ABORT:
            if t - self.t_state_entry >= ABORT_TO_DEPLOY_S:
                self.t_deploy = t
                event = self._enter(State.DEPLOY, t, "abort -> deploy")
        elif s == State.DEPLOY:
            if t - self.t_state_entry >= DEPLOY_TO_DESCENT_S:
                self.h_static_ref = inp.h
                self.t_static_start = t
                event = self._enter(State.DESCENT, t, "release complete")
        elif s == State.DESCENT:
            if abs(inp.accel_norm - self.g) > LANDING_ACCEL_SPIKE_MPS2:
                self.impact_seen = True
            if abs(inp.h - self.h_static_ref) > LANDING_BARO_STATIC_M:
                self.h_static_ref = inp.h
                self.t_static_start = t
            elif self.impact_seen and t - self.t_static_start >= LANDING_STATIC_S:
                self.t_landed = t
                event = self._enter(State.LANDED, t, f"impact seen, baro static {LANDING_STATIC_S:.0f} s (disarm)")
        # LANDED: terminal

        s = self.state
        # After cut-off the fan spins down with tau ~0.15 s and its swirl torque decays with it; keep the
        # attitude loop (and its roll trim) active for coast_control_hold_s so the body does not spin up.
        active = s in CONTROL_ACTIVE_STATES or (
            s == State.COAST and self.t_coast is not None and t - self.t_coast < self.mp.coast_control_hold_s)
        return MissionOutputs(
            state=s,
            throttle=self.throttle(t),
            control_active=active,
            vanes_centre=not active,
            release=s in RELEASED_STATES,
            abort_reason=self.abort_reason,
            event=event,
        )


class AltitudeFilter:
    """Baro + accelerometer complementary filter for h and v_z (SPEC §8.8).

    Continuous form:  dh/dt = v + k1 r,  dv/dt = a_z + k2 r,  r = h_baro* - h,
    k1 = 2 zeta w_n, k2 = w_n^2, w_n = 2 pi f_c.  The correction is applied
    discretely when a baro sample arrives (interval dt_b) and the sensor's
    known latency is compensated with h_baro* = h_baro + v latency.
    ``a_z`` is the world-frame vertical acceleration minus g, obtained by
    rotating the measured specific force with the attitude estimate.
    """

    def __init__(self, crossover_hz: float = 0.5, zeta: float = 0.7, baro_latency_s: float = 0.0):
        w_n = 2.0 * math.pi * crossover_hz
        self.k1 = 2.0 * zeta * w_n
        self.k2 = w_n * w_n
        self.latency = baro_latency_s
        self.h = 0.0
        self.v = 0.0
        self._t_last_baro: float | None = None
        self.initialised = False

    def reset(self, h0: float = 0.0) -> None:
        self.h, self.v = h0, 0.0
        self._t_last_baro = None
        self.initialised = True

    def predict(self, a_z_world: float, dt: float) -> None:
        self.h += self.v * dt + 0.5 * a_z_world * dt * dt
        self.v += a_z_world * dt

    def correct(self, h_baro: float, t: float) -> None:
        if not self.initialised:
            self.h, self.v, self.initialised = h_baro, 0.0, True
        dt_b = 0.0 if self._t_last_baro is None else max(t - self._t_last_baro, 0.0)
        self._t_last_baro = t
        if dt_b <= 0.0:
            return
        r = (h_baro + self.v * self.latency) - self.h
        self.h += self.k1 * r * dt_b
        self.v += self.k2 * r * dt_b


def thrust_ratio(u: float, inv_u_hover: float, throttle_exp: float) -> float:
    """T(u) / (m g) of the controllers' thrust model T = T_max u^n with u_hover = (m g / T_max)^(1/n):
    (u / u_hover)^n; n = 2 (edf.throttle_exponent) -> (u inv_u_hover)^2 with no pow call (the C port precomputes
    inv_u_hover).  u outside [0, 1] is clamped, a non-finite u counts as 0 (motor off)."""
    if not (u == u) or u <= 0.0:          # NaN or <= 0
        return 0.0
    x = (u if u < 1.0 else 1.0) * inv_u_hover
    return x * x if throttle_exp == 2.0 else x ** throttle_exp


class VtolAltitudeFilter:
    """VTOL NAV altitude filter (v0.7.1): baro + accelerometer, states h [m], v [m/s] and the accelerometer z-bias
    b [m/s^2]; ballistic build: ``AltitudeFilter`` unchanged.

    Why the bias state: v0.7.0 landed on the integrated accelerometer after a baro failure and the ASSUMED 10 mg
    residual bias flew 7 of 20 seeds away (up to 147 m, ended only by the V6 hard cutoff).  Learnt while the baro is
    healthy, the bias is removed before the integration, so a baro-lost landing dead-reckons on (a - b).

    Continuous model (a = world-frame vertical specific force minus g from the NAV attitude, the old filter's input):
        dh/dt = v + k1 r,   dv/dt = (a - b) + k2 r,   db/dt = -k3 r,   r = h_b* - h
    Error dynamics (e_h = h_true - h, ...): s^3 + k1 s^2 + k2 s + k3 = 0.  Pole placement: the old filter's pair at
    w = 2 pi f_c with damping zeta, plus a real bias pole at p = 1 / bias_tau_s:
        (s^2 + 2 zeta w s + w^2)(s + p)  ->  k1 = 2 zeta w + p,  k2 = w^2 + 2 zeta w p,  k3 = w^2 p
    (p -> 0 gives the old filter exactly).  The bias pole is slow on purpose: it averages the baro noise (0.25 m RMS
    at 50 Hz) over ~bias_tau_s, and the pad calibration below already gives the bias to a few mm/s^2 at take-off, so
    the flight learning only has to follow slow changes (temperature, vibration rectification).

    Per control tick (dt), the order of the NAV (nav_main.c steps 4):
      pad (FCU state SAFE / ARMED, ``sim.nlink_proto.nav_alt_pad_mode``): h = v = 0 (known: on the pad) and the pad
          calibration n = min(n + 1, N), b += (a - b) / n, N = round(pad_bias_tau_s / dt) - the vehicle is still, so a IS
          the bias; a running mean of the first N pad ticks, then an EMA with 1 / N (the estimator's pad scheme, n never
          reset: after LANDED -> SAFE it continues as the EMA).  Why not a plain EMA: ON needs ALIGNED (>= N ticks on the
          pad) and the spool warning adds 2 s, so the vehicle can take off ~2.5 s after power-up; an EMA with a 2 s time
          constant has then learnt only ~70 % of the bias, and the rest (0.01-0.02 m/s^2 in the scratch study
          v071/pyref/td_study2.py) left a 0.2-0.5 m/s dead-reckoning error after 20 s of a baro-lost landing.  A baro
          sample on the pad only stamps its time (no correction: h, v are known, b is calibrated directly).
      flight: predict  h += v dt + (a - b) dt^2 / 2;  v += (a - b) dt.
      new baro sample h_b at time t (flight): thrust compensation, then the correction of the old filter:
          h_c  = h_b - baro_thrust_comp_m T(u_prev) / (m g)      (``thrust_ratio``; u_prev = the FCU's last ESC
                 throttle, ``sim.nlink_proto.nav_throttle``; positive comp = the baro reads high with the fan running)
          dt_b = t - t_last (0 on the first sample -> stamp only);  r = (h_c + v L) - h  (L = baro latency)
          h += k1 r dt_b;  v += k2 r dt_b;  b = clamp(b - k3 r dt_b, -b_lim, +b_lim)
      baro lost: no samples -> predict only, with the learnt b (nothing else changes: the filter needs no mode).
    b_lim (``bias_limit_mps2``) bounds the state against a baro fault that passes the health check (a bias beyond the
    sensor's zero-g tolerance is not an accelerometer bias).

    v0.8 NAV order (``sim.estimator`` module docstring, step 5): the filter runs AFTER the frame push, with a from the
    CORRECTED attitude q_k of the same tick; its h / v reach the FCU in the frame of the NEXT tick (4 ms later).  The
    equations are unchanged; only the call moves (the 6-DOF publishes the previous tick's h, v).

    Units: h, h_b [m] above the pad; v [m/s] up; a, b [m/s^2]; t [s] (the C port uses tick counts x tick_dt)."""

    def __init__(self, crossover_hz: float = 0.5, zeta: float = 0.7, bias_tau_s: float = VTOL_ALT_BIAS_TAU_S_ASSUMED,
                 baro_latency_s: float = 0.0, pad_bias_tau_s: float = VTOL_ALT_PAD_BIAS_TAU_S_ASSUMED,
                 bias_limit_mps2: float = VTOL_ALT_BIAS_LIMIT_MPS2_ASSUMED, baro_thrust_comp_m: float = 0.0,
                 u_hover: float = 1.0, throttle_exp: float = 2.0):
        if not (crossover_hz > 0 and zeta > 0 and bias_tau_s > 0 and pad_bias_tau_s > 0 and bias_limit_mps2 >= 0
                and u_hover > 0 and throttle_exp > 0):
            raise ValueError("VtolAltitudeFilter: need crossover_hz, zeta, bias_tau_s, pad_bias_tau_s, u_hover, "
                             "throttle_exp > 0 and bias_limit >= 0")
        w = 2.0 * math.pi * crossover_hz
        p = 1.0 / bias_tau_s
        self.k1 = 2.0 * zeta * w + p
        self.k2 = w * w + 2.0 * zeta * w * p
        self.k3 = w * w * p
        self.latency = baro_latency_s
        self.pad_bias_tau_s = float(pad_bias_tau_s)
        self.bias_limit = float(bias_limit_mps2)
        self.comp_m = float(baro_thrust_comp_m)
        self.inv_u_hover = 1.0 / float(u_hover)
        self.throttle_exp = float(throttle_exp)
        self.h = 0.0
        self.v = 0.0
        self.b = 0.0
        self.n_pad = 0                             # pad calibration samples (saturates at the n_avg of the first pad tick)
        self._t_last_baro: float | None = None
        self.last_comp_m = 0.0                     # the compensation applied to the latest sample (diagnostics)

    @classmethod
    def from_params(cls, p: dict, u_hover: float, crossover_hz: float, zeta: float, baro_latency_s: float,
                    bias_tau_s: float = VTOL_ALT_BIAS_TAU_S_ASSUMED,
                    pad_bias_tau_s: float = VTOL_ALT_PAD_BIAS_TAU_S_ASSUMED,
                    bias_limit_mps2: float = VTOL_ALT_BIAS_LIMIT_MPS2_ASSUMED) -> "VtolAltitudeFilter":
        """baro_thrust_comp_m from params.json mission.vtol, the thrust exponent from edf.throttle_exponent; u_hover =
        the vertical controller's (``VerticalParams.u_hover``); the filter constants from the caller (sim: SensorConfig,
        each labelled there; firmware: params_gen.h)."""
        return cls(crossover_hz, zeta, bias_tau_s, baro_latency_s, pad_bias_tau_s, bias_limit_mps2,
                   float(p["mission"]["vtol"].get("baro_thrust_comp_m", 0.0)), u_hover, float(p["edf"]["throttle_exponent"]))

    @property
    def poles(self) -> tuple[complex, ...]:
        """Roots of s^3 + k1 s^2 + k2 s + k3 (continuous error dynamics)."""
        import numpy as _np
        return tuple(complex(x) for x in _np.roots([1.0, self.k1, self.k2, self.k3]))

    def baro_compensation(self, u_prev: float) -> float:
        """baro_thrust_comp_m x T(u_prev) / (m g) [m], subtracted from each flight baro sample."""
        return self.comp_m * thrust_ratio(u_prev, self.inv_u_hover, self.throttle_exp) if self.comp_m != 0.0 else 0.0

    def pad(self, a_z_world: float, dt: float) -> None:
        """Pad tick: h = v = 0, bias calibration from the still vehicle's specific force (running mean of the first
        N = round(pad_bias_tau_s / dt) ticks, then EMA 1 / N)."""
        self.h, self.v = 0.0, 0.0
        n_avg = max(1, int(round(self.pad_bias_tau_s / dt)))
        self.n_pad = min(self.n_pad + 1, n_avg)
        self.b += (a_z_world - self.b) / self.n_pad
        self.b = min(max(self.b, -self.bias_limit), self.bias_limit)

    def predict(self, a_z_world: float, dt: float) -> None:
        a = a_z_world - self.b
        self.h += self.v * dt + 0.5 * a * dt * dt
        self.v += a * dt

    def stamp(self, t: float) -> None:
        """A baro sample on the pad: its time only (the first flight correction then spans one baro period)."""
        self._t_last_baro = t

    def correct(self, h_baro: float, t: float, u_prev: float = 0.0) -> None:
        dt_b = 0.0 if self._t_last_baro is None else max(t - self._t_last_baro, 0.0)
        self._t_last_baro = t
        self.last_comp_m = self.baro_compensation(u_prev)
        if dt_b <= 0.0:
            return
        r = (h_baro - self.last_comp_m + self.v * self.latency) - self.h
        self.h += self.k1 * r * dt_b
        self.v += self.k2 * r * dt_b
        self.b = min(max(self.b - self.k3 * r * dt_b, -self.bias_limit), self.bias_limit)

    def step(self, a_z_world: float, dt: float, h_baro: float | None, t: float, pad: bool, u_prev: float = 0.0) -> None:
        """One NAV tick in the normative order: pad hold + calibration or predict; then the baro sample, if any."""
        if pad:
            self.pad(a_z_world, dt)
        else:
            self.predict(a_z_world, dt)
        if h_baro is not None:
            if pad:
                self.stamp(t)
            else:
                self.correct(h_baro, t, u_prev)
