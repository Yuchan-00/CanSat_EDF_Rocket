"""Onboard supervisor (SPEC §8.8, the rocket's ballistic mission, v0.4) - the only autonomous logic on the vehicle.

Executable specification for the firmware's command executor + safety
interlocks.  Pure Python (no numpy), integer control ticks (``control.loop_hz``)
for every timer, so a C port makes identical decisions.  The VTOL program (SPEC 8.9, v0.7-v0.9) was removed on
2026-11-18 (rocket only); this module is the v0.6.3 ballistic supervisor, bit-identical.

Ballistic build (v0.4)
======================

The vehicle flies attitude control only; launch, throttle, motor
cut and nose release are ground commands received over the LoRa link
(``sim.link_proto``).

States::

    SAFE      motor off (1000 us), vanes centred, release latched      (boot state)
    ARMED     on the pad, motor off, vanes centred, estimator aligning
    POWERED   throttle target > 0, attitude control on, throttle slewed 0 -> 1 in throttle_ramp_s
    UNPOWERED throttle 0 after POWERED; attitude control on for coast_control_hold_s, then vanes centred
    RELEASED  release servo open; motor off until power cycle; vanes centred
    ABORT     motor off now; release after abort_release_delay_s, then RELEASED

Commands (valid CMD frame for this vehicle; executed only when its SEQ is newer
than the last processed SEQ - ``link_proto.seq_newer``; the first frame after
boot is always new; a NONE frame with a newer SEQ only adopts the SEQ):

    ARM       SAFE                      -> ARMED if arm switch closed, sensors healthy, aligned,
                                           tilt < arm_tilt_max_deg, V_cell >= min_cell_v, link alive
    DISARM    ARMED                     -> SAFE
    LAUNCH    ARMED                     nonce == telemetry nonce; target = climb_throttle -> POWERED
    THROTTLE  ARMED (nonce), POWERED, UNPOWERED
                                        target = clamp(u, 0, climb_throttle); u > 0 -> POWERED, u = 0 -> UNPOWERED
    CUT       POWERED                   target 0 -> UNPOWERED
    RELEASE   ARMED, POWERED, UNPOWERED throttle 0 now (POWERED -> UNPOWERED), release servo opens after
                                        release_after_cut_s -> RELEASED
    KILL      any except SAFE           ARMED -> SAFE; POWERED/UNPOWERED -> ABORT; ABORT/RELEASED: no change
    codes 8..11 (the removed VTOL program's ON / OFF / UP / DOWN): REJ_UNKNOWN_CMD, exactly what v0.6.3 answered

Readings of the contract made here (all the safe side, see ``issues`` in the run report):

* "processed" SEQ: a rejected command consumes its SEQ (ack_seq confirms it, the REJECTED
  flag says it was not executed), so a repeated frame can never execute it later.
* "link alive" for ARM: a valid ground frame within ``link_loss_disarm_s`` *before* the ARM
  frame (the first frame after a silence cannot arm).
* THROTTLE(u = 0) in ARMED is rejected (it would enter UNPOWERED on the pad, where I1 can abort
  and fire the release).
* While a release is pending only KILL is accepted; KILL / an interlock abort keeps the earlier of
  the pending release time and abort_release_delay_s.  On the pad (ARMED) KILL, I1p, I9 and the
  arm switch cancel a pending release (-> SAFE).
* Throttle increases are slewed at 1 / throttle_ramp_s; decreases (THROTTLE down, CUT, RELEASE,
  ABORT, any exit from POWERED) are immediate.
* I7 counts the cumulative POWERED time since boot (re-lights add up).
* ARM is rejected while the release servo is open (after an I10 boot).

Interlocks, evaluated every tick after the received commands, in this order; the
first one that applies acts and the rest wait for the next tick:

    arm switch open    ARMED -> SAFE; POWERED/UNPOWERED -> ABORT                (acts as KILL)
    I9  IMU fail >= imu_fail_kill_s / baro fail >= baro_fail_kill_s: ARMED -> SAFE, POWERED/UNPOWERED -> ABORT
    I1p ARMED, no valid ground frame for link_loss_disarm_s                  -> SAFE
    I1  POWERED/UNPOWERED, no valid ground frame for link_loss_abort_s       -> ABORT
    I2  POWERED, tilt > tilt_abort_deg                                       -> ABORT
    I3  POWERED, gyro at full scale (gyro_saturation_abort) or |w_xy| > tilt_rate_abort_dps or
        |w_z| > spin_rate_abort_dps for rate_abort_persist_s                   -> ABORT
    I8  POWERED, V_cell < min_cell_v                                         -> ABORT
    I5  POWERED/UNPOWERED, h > geofence_altitude_m                           -> ABORT
    I4  POWERED, h + coast_drag_factor max(v_z, 0)^2 / (2 g) >= apogee_limit_m -> CUT
    I7  POWERED, cumulative POWERED time >= max_motor_time_s                  -> CUT
    I6  POWERED/UNPOWERED, no release open/pending, v_z <= fall_deploy_vz_mps for
        fall_deploy_confirm_s                                                 -> RELEASE
    I10 boot after a watchdog / brown-out reset in flight: SAFE with the release open

Then the release timer (pending release or ABORT) opens the servo -> RELEASED, the
throttle is slewed, and the POWERED-time counter advances.  Durations are tick
counts n = ticks_ge(seconds) = the smallest n with n / loop_hz >= seconds (the rule of
``tools/gen_firmware_params.py``); a condition "for T" fires on the tick its
counter reaches n (counters start at 1 on the first tick the condition holds).
Throttle is integer per mille: out = min(target, level x 1000 // ramp_ticks), level
+1 per tick while out < target.

Nonce: 16-bit Galois LFSR (taps 0xB400, maximal length, never 0).  At boot
state = seed (0 -> 0xACE1) and nonce = next(state); every executed LAUNCH,
THROTTLE-from-ARMED advances it once.  The firmware seeds it from ADC noise.

Invariant: commands are executed before the interlocks of the same tick, so an
interlock always has the last word; no command can disable an interlock.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from enum import IntEnum
from fractions import Fraction

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import link_proto as lp
    from sim.state_machine import G0
except ImportError:  # pragma: no cover
    import link_proto as lp  # type: ignore
    from state_machine import G0  # type: ignore

_HERE = os.path.dirname(os.path.abspath(__file__))
FC_CONFIG_PATH = os.path.normpath(os.path.join(_HERE, "..", "firmware", "atmega128_fc", "fc_config.json"))
# SPEC §8.8 I9 row ("IMU failure > 0.1 s or baro failure > 0.5 s"); used only when the firmware's
# fc_config.json (imu_fail_kill_s / baro_fail_kill_s, the calibration file the firmware is built from) is absent.
I9_IMU_FAIL_S_SPEC = 0.1
I9_BARO_FAIL_S_SPEC = 0.5
LFSR_TAPS = 0xB400
LFSR_ZERO_SEED = 0xACE1


class State(IntEnum):
    SAFE = lp.STATE_SAFE
    ARMED = lp.STATE_ARMED
    POWERED = lp.STATE_POWERED
    UNPOWERED = lp.STATE_UNPOWERED
    RELEASED = lp.STATE_RELEASED
    ABORT = lp.STATE_ABORT


FLIGHT_STATES = (State.POWERED, State.UNPOWERED)
MOTOR_ENABLED_STATES = (State.ARMED, State.POWERED, State.UNPOWERED)


def ticks_ge(seconds: float, loop_hz: int) -> int:
    """Smallest tick count n with n / loop_hz >= seconds (exact on the double value; values within
    1e-9 tick of an integer snap to it) - identical to tools/gen_firmware_params.py."""
    x = Fraction(float(seconds)) * int(loop_hz)
    n = math.ceil(x)
    if n - x > 1 - Fraction(1, 10 ** 9):
        n -= 1
    return max(int(n), 0)


def lfsr16_next(s: int) -> int:
    """One step of the 16-bit Galois LFSR x^16 + x^14 + x^13 + x^11 + 1 (period 65535, never 0)."""
    s &= 0xFFFF
    if s == 0:
        s = LFSR_ZERO_SEED
    lsb = s & 1
    s >>= 1
    if lsb:
        s ^= LFSR_TAPS
    return s


def _ceil_div(a: int, b: int) -> int:
    return -(-a // b)


def metres_to_dm(x: float) -> int:
    """Altitude [m] -> integer decimetres (round half away from zero, ``link_proto.quantize``)."""
    return lp.quantize(float(x), lp.H_SCALE, lp.I16)


# --------------------------------------------------------------------------- #
# parameters
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SupervisorParams:
    """Every number the supervisor uses; times as control ticks.  ``from_params`` is the only
    constructor the sim uses (params.json + the firmware calibration file for I9)."""

    loop_hz: int
    climb_milli: int
    ramp_ticks: int
    coast_hold_ticks: int
    tilt_abort_deg: float
    arm_tilt_max_deg: float
    min_cell_v: float
    geofence_m: float
    apogee_limit_m: float
    coast_drag_factor: float
    fall_vz_mps: float
    fall_confirm_ticks: int
    link_abort_ticks: int
    link_disarm_ticks: int
    max_motor_ticks: int
    release_after_cut_ticks: int
    abort_release_ticks: int
    tilt_rate_abort_rad_s: float
    spin_rate_abort_rad_s: float
    rate_persist_ticks: int
    gyro_saturation_abort: bool
    imu_fail_ticks: int
    baro_fail_ticks: int
    g: float = G0
    rate_abort_enabled: bool = True          # False only in the "why I3 exists" study
    i9_source: str = ""
    mode: str = lp.MODE_BALLISTIC            # the mission program (params.json mission.mode): ballistic only

    @property
    def apogee_k(self) -> float:
        """coast_drag_factor / (2 g)."""
        return self.coast_drag_factor / (2.0 * self.g)

    @classmethod
    def from_params(cls, p: dict, fc_config_path: str | None = FC_CONFIG_PATH, **overrides) -> "SupervisorParams":
        """``mode`` follows params.json ``mission.mode`` (absent: ballistic; anything else is refused) unless ``mode=``
        is passed; any other keyword overrides the field of that name (``geofence_altitude_m`` is accepted for
        ``geofence_m``)."""
        m = p["mission"]
        hz = int(p["control"]["loop_hz"])
        mode = overrides.pop("mode", None)
        mode = lp.mission_mode(p) if mode is None else lp.check_mode(mode)
        imu_s, baro_s, src = I9_IMU_FAIL_S_SPEC, I9_BARO_FAIL_S_SPEC, "SPEC 8.8 I9 row (fc_config.json not found)"
        if fc_config_path and os.path.exists(fc_config_path):
            with open(fc_config_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            if "imu_fail_kill_s" in cfg and "baro_fail_kill_s" in cfg:
                imu_s, baro_s = float(cfg["imu_fail_kill_s"]), float(cfg["baro_fail_kill_s"])
                src = os.path.relpath(fc_config_path, os.path.join(_HERE, "..")).replace("\\", "/")
        kw = dict(
            loop_hz=hz,
            climb_milli=lp.quantize(float(m["climb_throttle"]), lp.THROTTLE_SCALE, (0, lp.THROTTLE_SCALE)),
            ramp_ticks=ticks_ge(float(m["throttle_ramp_s"]), hz),
            coast_hold_ticks=ticks_ge(float(m.get("coast_control_hold_s", 0.0)), hz),
            tilt_abort_deg=float(m["tilt_abort_deg"]),
            arm_tilt_max_deg=float(m["arm_tilt_max_deg"]),
            min_cell_v=float(p["battery"]["min_cell_v"]),
            geofence_m=float(m["geofence_altitude_m"]),
            apogee_limit_m=float(m["apogee_limit_m"]),
            coast_drag_factor=float(m["coast_drag_factor"]),
            fall_vz_mps=float(m["fall_deploy_vz_mps"]),
            fall_confirm_ticks=max(1, ticks_ge(float(m["fall_deploy_confirm_s"]), hz)),
            link_abort_ticks=max(1, ticks_ge(float(m["link_loss_abort_s"]), hz)),
            link_disarm_ticks=max(1, ticks_ge(float(m["link_loss_disarm_s"]), hz)),
            max_motor_ticks=max(1, ticks_ge(float(m["max_motor_time_s"]), hz)),
            release_after_cut_ticks=ticks_ge(float(m["release_after_cut_s"]), hz),
            abort_release_ticks=ticks_ge(float(m["abort_release_delay_s"]), hz),
            tilt_rate_abort_rad_s=math.radians(float(m["tilt_rate_abort_dps"])),
            spin_rate_abort_rad_s=math.radians(float(m["spin_rate_abort_dps"])),
            rate_persist_ticks=max(1, ticks_ge(float(m["rate_abort_persist_s"]), hz)),
            gyro_saturation_abort=bool(m.get("gyro_saturation_abort", True)),
            imu_fail_ticks=max(1, ticks_ge(imu_s, hz)),
            baro_fail_ticks=max(1, ticks_ge(baro_s, hz)),
            i9_source=src,
            mode=mode,
        )
        alias = {"geofence_altitude_m": "geofence_m"}          # params.json name -> field name
        for k, v in overrides.items():
            if v is None:
                continue
            kw[alias.get(k, k)] = v
        return cls(**kw)


# --------------------------------------------------------------------------- #
# inputs / outputs
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class UplinkFrame:
    """A valid CMD frame for this vehicle (CRC, LEN, VEH and TYPE already checked by the decoder)."""

    seq: int
    cmd: lp.Command

    @classmethod
    def from_frame(cls, fr: lp.Frame) -> "UplinkFrame":
        return cls(fr.seq, lp.decode_command(fr))


@dataclass
class OnboardInputs:
    tick: int                                     # control tick since boot
    frames: list[UplinkFrame] = field(default_factory=list)   # received since the previous tick, oldest first
    h_m: float = 0.0                              # altitude filter (above the pad)
    vz_mps: float = 0.0
    tilt_deg: float = 0.0                         # attitude estimate
    gyro_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)  # bias-corrected body rates
    gyro_saturated: bool = False                  # any gyro axis at full scale this tick
    v_cell: float = 4.2                           # filtered battery voltage per cell
    imu_ok: bool = True                           # this tick's IMU read succeeded
    baro_ok: bool = True                          # the baro is delivering samples on schedule
    aligned: bool = True                          # pad alignment complete
    arm_switch: bool = True                       # debounced, True = closed
    roll_deg: float = 0.0                         # telemetry only
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    loop_max_us: float = 0.0


@dataclass
class OnboardOutputs:
    state: State
    throttle_target: float
    throttle: float                               # slewed command to the ESC / thrust estimate
    throttle_milli: int
    control_active: bool
    vanes_centre: bool
    release: bool                                 # release servo open
    reason: int
    flags: int
    ack_seq: int
    nonce: int
    events: list[str]                             # transitions / rejections of this tick
    reply_seq: int | None                         # SEQ of the uplink frame to answer this tick (None: no reply)
    telemetry: lp.Telemetry | None                # the reply's payload


# --------------------------------------------------------------------------- #
# the supervisor
# --------------------------------------------------------------------------- #
class OnboardSupervisor:
    """SPEC §8.8 reference.  Call ``update`` exactly once per control tick with increasing ticks."""

    def __init__(self, sp: SupervisorParams, seed: int = 0, reset_in_flight: bool = False):
        self.sp = sp
        self.state = State.SAFE
        self.t_entry = 0
        self.nonce = lfsr16_next(seed & 0xFFFF)
        self.has_seq = False
        self.last_seq = 0
        self.ack_rejected = False
        self.last_frame_tick: int | None = None
        self.reason = lp.REASON_NONE
        self.interlock_latched = False
        self.target_milli = 0
        self.level = 0
        self.out_milli = 0
        self.powered_ticks = 0
        self.t_unpowered: int | None = None
        self.release_open = False
        self.release_deadline: int | None = None
        self.rate_count = 0
        self.fall_count = 0
        self.imu_fail = 0
        self.baro_fail = 0
        self.events: list[tuple[int, str]] = []
        self.exec_log: list[dict] = []           # every processed (non-NONE) command
        if reset_in_flight:
            self.reason = lp.R_I10_RESET
            self.interlock_latched = True
            self.release_open = True
            self.events.append((0, "boot: I10 reset in flight -> SAFE with the release open"))

    # ---- helpers -------------------------------------------------------------------------------
    def predicted_apogee(self, h: float, vz: float) -> float:
        v = vz if vz > 0.0 else 0.0
        return h + self.sp.apogee_k * v * v

    def ticks_since_frame(self, t: int) -> int:
        return t - self.last_frame_tick if self.last_frame_tick is not None else 1 << 30

    def link_ok(self, t: int) -> bool:
        return self.ticks_since_frame(t) < self.sp.link_disarm_ticks

    def _enter(self, new: State, t: int, why: str) -> None:
        self.events.append((t, f"{self.state.name} -> {new.name}: {why}"))
        self.state = new
        self.t_entry = t
        if new != State.POWERED:
            self.target_milli = 0
        if new == State.UNPOWERED:
            self.t_unpowered = t
        if new == State.SAFE:
            self.release_deadline = None          # a release pending on the pad is cancelled

    def _interlock(self, code: int) -> None:
        self.reason = code
        self.interlock_latched = True

    def _abort(self, t: int, why: str) -> None:
        d = t + self.sp.abort_release_ticks
        self.release_deadline = d if self.release_deadline is None else min(self.release_deadline, d)
        self._enter(State.ABORT, t, why)

    def _start_release(self, t: int, why: str) -> None:
        self.release_deadline = t + self.sp.release_after_cut_ticks
        self.target_milli = 0
        if self.state == State.POWERED:
            self._enter(State.UNPOWERED, t, why)
        else:
            self.events.append((t, f"{self.state.name}: {why}"))

    def _kill(self, t: int, why: str) -> None:
        if self.state == State.ARMED:
            self._enter(State.SAFE, t, why)
        elif self.state in FLIGHT_STATES:
            self._abort(t, why)

    # ---- commands --------------------------------------------------------------------------------
    def _execute(self, fr: UplinkFrame, t: int, inp: OnboardInputs, link_ok_before: bool) -> int:
        """Execute one new command; returns 0 or the REJ_* code."""
        c, s, sp = fr.cmd.cmd, self.state, self.sp
        tag = f"{fr.cmd.name} (seq {fr.seq})"
        if c not in lp.CMD_NAMES or c == lp.CMD_NONE:
            return lp.REJ_UNKNOWN_CMD                  # also 8..11 (the removed VTOL codes), as in v0.6.3
        if c == lp.CMD_KILL:
            if s == State.SAFE:
                return lp.REJ_STATE
            if s in (State.ARMED,) + FLIGHT_STATES:
                self._kill(t, tag)
            return 0                                   # ABORT / RELEASED: already safe
        if self.release_deadline is not None:
            return lp.REJ_RELEASE_PENDING
        if c == lp.CMD_ARM:
            if s != State.SAFE:
                return lp.REJ_STATE
            code = self._arm_gates(inp, link_ok_before)
            if code:
                return code
            self._enter(State.ARMED, t, f"{tag}, tilt {inp.tilt_deg:.1f} deg, {inp.v_cell:.2f} V/cell")
            return 0
        if c == lp.CMD_DISARM:
            if s != State.ARMED:
                return lp.REJ_STATE
            self._enter(State.SAFE, t, tag)
            return 0
        if c == lp.CMD_LAUNCH:
            if s != State.ARMED:
                return lp.REJ_STATE
            if fr.cmd.nonce != self.nonce:
                return lp.REJ_NONCE
            self.nonce = lfsr16_next(self.nonce)
            self._enter(State.POWERED, t, tag)
            self.target_milli = sp.climb_milli
            return 0
        if c == lp.CMD_THROTTLE:
            u = min(fr.cmd.arg, sp.climb_milli)
            if s == State.ARMED:
                if fr.cmd.nonce != self.nonce:
                    return lp.REJ_NONCE
                if u == 0:
                    return lp.REJ_THROTTLE_ZERO
                self.nonce = lfsr16_next(self.nonce)
                self._enter(State.POWERED, t, f"{tag} u = {u / 1000:.3f}")
                self.target_milli = u
                return 0
            if s == State.POWERED:
                if u == 0:
                    self._enter(State.UNPOWERED, t, f"{tag} u = 0")
                else:
                    self.target_milli = u
                    self.events.append((t, f"POWERED: {tag} u = {u / 1000:.3f}"))
                return 0
            if s == State.UNPOWERED:
                if u > 0:
                    self._enter(State.POWERED, t, f"{tag} u = {u / 1000:.3f}")
                    self.target_milli = u
                return 0
            return lp.REJ_STATE
        if c == lp.CMD_CUT:
            if s != State.POWERED:
                return lp.REJ_STATE
            self._enter(State.UNPOWERED, t, tag)
            return 0
        if c == lp.CMD_RELEASE:
            if s not in (State.ARMED,) + FLIGHT_STATES:
                return lp.REJ_STATE
            self._start_release(t, f"{tag}: throttle 0, release in {sp.release_after_cut_ticks} ticks")
            return 0
        return lp.REJ_UNKNOWN_CMD  # pragma: no cover

    def _arm_gates(self, inp: OnboardInputs, link_ok_before: bool) -> int:
        """The ARM gates in their order: 0 or the REJ_* code of the first that fails."""
        sp = self.sp
        if self.release_open:
            return lp.REJ_RELEASED
        if not inp.arm_switch:
            return lp.REJ_ARM_SWITCH
        if not (inp.imu_ok and inp.baro_ok and self.imu_fail == 0 and self.baro_fail == 0):
            return lp.REJ_SENSORS
        if not inp.aligned:
            return lp.REJ_NOT_ALIGNED
        if not inp.tilt_deg < sp.arm_tilt_max_deg:
            return lp.REJ_TILT
        if not inp.v_cell >= sp.min_cell_v:
            return lp.REJ_VCELL
        if not link_ok_before:
            return lp.REJ_LINK
        return 0

    # ---- interlocks --------------------------------------------------------------------------------
    def _interlocks(self, t: int, inp: OnboardInputs) -> None:
        sp, s = self.sp, self.state
        # persistence counters on this tick's inputs (state after the commands)
        if s == State.POWERED and sp.rate_abort_enabled:
            wx, wy, wz = inp.gyro_rad_s
            bad = ((sp.gyro_saturation_abort and inp.gyro_saturated) or math.hypot(wx, wy) > sp.tilt_rate_abort_rad_s
                   or abs(wz) > sp.spin_rate_abort_rad_s)
            self.rate_count = self.rate_count + 1 if bad else 0
        else:
            self.rate_count = 0
        if s in FLIGHT_STATES and inp.vz_mps <= sp.fall_vz_mps:
            self.fall_count += 1
        else:
            self.fall_count = 0
        if s not in MOTOR_ENABLED_STATES:
            return
        on_pad = s == State.ARMED
        if not inp.arm_switch:
            self._interlock(lp.R_ARM_SWITCH_OPEN)
            self._kill(t, "arm switch opened (KILL)")
            return
        if self.imu_fail >= sp.imu_fail_ticks or self.baro_fail >= sp.baro_fail_ticks:
            code = lp.R_I9_IMU if self.imu_fail >= sp.imu_fail_ticks else lp.R_I9_BARO
            self._interlock(code)
            why = f"{lp.REASON_NAMES[code]} ({max(self.imu_fail, self.baro_fail)} ticks)"
            if on_pad:
                self._enter(State.SAFE, t, why)
            else:
                self._abort(t, why)
            return
        if on_pad:
            if self.ticks_since_frame(t) >= sp.link_disarm_ticks:
                self._interlock(lp.R_I1P_LINK_LOSS_PAD)
                self._enter(State.SAFE, t, f"I1p no ground frame for {self.ticks_since_frame(t)} ticks")
            return
        # POWERED / UNPOWERED
        if self.ticks_since_frame(t) >= sp.link_abort_ticks:
            self._interlock(lp.R_I1_LINK_LOSS)
            self._abort(t, f"I1 no ground frame for {self.ticks_since_frame(t)} ticks")
            return
        if s == State.POWERED:
            if inp.tilt_deg > sp.tilt_abort_deg:
                self._interlock(lp.R_I2_TILT)
                self._abort(t, f"I2 tilt {inp.tilt_deg:.1f} deg > {sp.tilt_abort_deg:.0f}")
                return
            if self.rate_count >= sp.rate_persist_ticks:
                wx, wy, wz = inp.gyro_rad_s
                self._interlock(lp.R_I3_RATE)
                self._abort(t, f"I3 |w_xy| {math.degrees(math.hypot(wx, wy)):.0f} deg/s, |w_z| {math.degrees(abs(wz)):.0f} "
                               f"deg/s{' (gyro at full scale)' if inp.gyro_saturated else ''} for {self.rate_count} ticks")
                return
            if inp.v_cell < sp.min_cell_v:
                self._interlock(lp.R_I8_VCELL)
                self._abort(t, f"I8 V_cell {inp.v_cell:.2f} < {sp.min_cell_v:.2f}")
                return
        if inp.h_m > sp.geofence_m:
            self._interlock(lp.R_I5_GEOFENCE)
            self._abort(t, f"I5 geofence h {inp.h_m:.1f} m > {sp.geofence_m:.0f}")
            return
        if s == State.POWERED:
            pred = self.predicted_apogee(inp.h_m, inp.vz_mps)
            if pred >= sp.apogee_limit_m:
                self._interlock(lp.R_I4_APOGEE_LIMIT)
                self._enter(State.UNPOWERED, t, f"I4 apogee limiter: h {inp.h_m:.1f} m, v_z {inp.vz_mps:.1f} m/s, "
                                                f"predicted {pred:.1f} m >= {sp.apogee_limit_m:.0f}")
                return
            if self.powered_ticks >= sp.max_motor_ticks:
                self._interlock(lp.R_I7_MOTOR_TIME)
                self._enter(State.UNPOWERED, t, f"I7 motor time {self.powered_ticks} ticks")
                return
        if not self.release_open and self.release_deadline is None and self.fall_count >= sp.fall_confirm_ticks:
            self._interlock(lp.R_I6_FALL)
            self._start_release(t, f"I6 fall-safe: v_z {inp.vz_mps:.1f} m/s <= {sp.fall_vz_mps:.0f} for "
                                   f"{self.fall_count} ticks, release in {sp.release_after_cut_ticks} ticks")

    # ---- throttle ------------------------------------------------------------------------------------
    def _slew(self) -> None:
        tgt = self.target_milli if self.state == State.POWERED else 0
        r = self.sp.ramp_ticks
        if r <= 0:
            self.out_milli = tgt
            return
        if tgt < self.out_milli:                        # decreases are immediate
            self.out_milli = tgt
            self.level = _ceil_div(tgt * r, lp.THROTTLE_SCALE)
        elif tgt > self.out_milli:
            self.level = min(self.level + 1, r)
            self.out_milli = min(tgt, self.level * lp.THROTTLE_SCALE // r)

    # ---- main ----------------------------------------------------------------------------------------
    def control_active(self, t: int) -> bool:
        if self.state == State.POWERED:
            return True
        return (self.state == State.UNPOWERED and self.t_unpowered is not None
                and t - self.t_unpowered < self.sp.coast_hold_ticks)

    def flags(self, t: int, inp: OnboardInputs) -> int:
        f = 0
        if inp.arm_switch:
            f |= lp.FLAG_ARM_SWITCH
        if inp.imu_ok and inp.baro_ok and self.imu_fail == 0 and self.baro_fail == 0:
            f |= lp.FLAG_SENSORS_OK
        if inp.aligned:
            f |= lp.FLAG_ALIGNED
        if self.link_ok(t):
            f |= lp.FLAG_LINK_OK
        if self.release_open:
            f |= lp.FLAG_RELEASED
        if self.control_active(t):
            f |= lp.FLAG_CONTROL_ACTIVE
        if self.ack_rejected:
            f |= lp.FLAG_REJECTED
        if self.interlock_latched:
            f |= lp.FLAG_INTERLOCK
        return f

    def telemetry(self, t: int, inp: OnboardInputs) -> lp.Telemetry:
        """The reply's payload (TELEM v1)."""
        thr = self.out_milli / lp.THROTTLE_SCALE
        return lp.Telemetry.from_physical(
            state=int(self.state), flags=self.flags(t, inp), reason=self.reason,
            ack_seq=self.last_seq if self.has_seq else 0, nonce=self.nonce,
            t_ms=(t * 1000) // self.sp.loop_hz, h_m=inp.h_m, vz_mps=inp.vz_mps, roll_deg=inp.roll_deg,
            pitch_deg=inp.pitch_deg, yaw_deg=inp.yaw_deg, tilt_deg=inp.tilt_deg,
            throttle=thr, vcell_v=inp.v_cell, loop_max_us=inp.loop_max_us)

    def update(self, inp: OnboardInputs) -> OnboardOutputs:
        t = inp.tick
        n_ev = len(self.events)
        self.imu_fail = 0 if inp.imu_ok else self.imu_fail + 1
        self.baro_fail = 0 if inp.baro_ok else self.baro_fail + 1
        # 1. received commands
        reply_seq: int | None = None
        for fr in inp.frames:
            link_ok_before = self.link_ok(t)
            self.last_frame_tick = t
            reply_seq = fr.seq
            if self.has_seq and not lp.seq_newer(fr.seq, self.last_seq):
                continue                                   # repeat of a processed SEQ (or older): reply only
            self.has_seq = True
            self.last_seq = fr.seq
            if fr.cmd.cmd == lp.CMD_NONE:
                self.ack_rejected = False
                continue
            state_before = self.state
            code = self._execute(fr, t, inp, link_ok_before)
            self.ack_rejected = code != 0
            if code:
                if not self.interlock_latched:         # the interlock code outranks rejections (ground reads flags)
                    self.reason = code
                self.events.append((t, f"REJECT {fr.cmd.name} (seq {fr.seq}) in {state_before.name}: "
                                       f"{lp.REASON_NAMES.get(code, code)}"))
            rec = {"tick": t, "seq": fr.seq, "cmd": fr.cmd.cmd, "code": code,
                   "state_before": int(state_before), "state_after": int(self.state)}
            self.exec_log.append(rec)
        # 2. interlocks
        self._interlocks(t, inp)
        # 3. release timer (pending release or ABORT)
        if self.release_deadline is not None and t >= self.release_deadline:
            self.release_deadline = None
            self.release_open = True
            self._enter(State.RELEASED, t, "release servo open")
        # 4. throttle slew
        self._slew()
        # the I7 clock: POWERED ticks since boot
        if self.state == State.POWERED:
            self.powered_ticks += 1
        active = self.control_active(t)
        return OnboardOutputs(
            state=self.state, throttle_target=self.target_milli / lp.THROTTLE_SCALE,
            throttle=self.out_milli / lp.THROTTLE_SCALE, throttle_milli=self.out_milli, control_active=active,
            vanes_centre=not active, release=self.release_open, reason=self.reason, flags=self.flags(t, inp),
            ack_seq=self.last_seq if self.has_seq else 0, nonce=self.nonce,
            events=[e for _, e in self.events[n_ev:]], reply_seq=reply_seq,
            telemetry=self.telemetry(t, inp) if reply_seq is not None else None)
