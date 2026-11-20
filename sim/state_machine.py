"""Altitude complementary filters (current) and the v0.3 mission state machine
(SUPERSEDED).

``AltitudeFilter`` is current: the v0.4 flight computer runs it for the
telemetry (h, v_z) and the interlocks of ``sim.onboard``; the firmware mirrors it
(``firmware/atmega128_fc/src/core/alt_filter.c``).  It stays the ballistic
build's filter, bit for bit.

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


