"""LoRa ground-link frame format (SPEC §10.3) - the byte-exact reference.

Pure Python (no numpy): the ground station (``tools/ground_station.py``) and
the firmware cross-check import it.  Nothing here knows about time, radios or
the vehicle state; ``sim.onboard`` and ``sim.ground_station_logic`` use it.

Frame (both directions, every multi-byte field little-endian)::

    0xA5 0x5A | LEN | VEH | TYPE | SEQ | PAYLOAD | CRC16
                ^-------- LEN counts VEH..PAYLOAD (= 3 + payload bytes)
                ^-------------- CRC16 over LEN..PAYLOAD -------------^

CRC16-CCITT: poly 0x1021, init 0xFFFF, MSB first, no reflection, no final XOR
(the "CRC-16/CCITT-FALSE" variant; check value of b"123456789" = 0x29B1),
transmitted low byte first.  VEH = ``flight_computer.link.vehicle_id``.

* Uplink ``TYPE_CMD`` (0x10), 5 payload bytes ``<BHH``: cmd u8, arg u16
  (throttle x1000), nonce u16.
* Downlink ``TYPE_TELEM`` (0x20, TELEM v1, the ballistic build), 28 payload bytes
  ``<BBBBHIhhhhhHHHH``: state u8, flags u8, reason u8, ack_seq u8, nonce u16, t_ms u32,
  h i16 (0.1 m), v_z i16 (cm/s), roll / pitch / yaw i16 (0.01 deg), tilt u16 (0.01 deg),
  throttle u16 (x1000), V_cell u16 (mV), loop_max u16 (us).
* Downlink ``TYPE_TELEM2`` (0x21, TELEM v2, v0.7, the VTOL build), 31 payload bytes
  ``<BBBBHIhhhhhHHHHBh``: the 28 TELEM v1 bytes, then phase u8 (``PHASE_*``) and the
  altitude target h_tgt i16 (0.1 m).  A new TYPE rather than a longer TYPE 0x20: every
  TYPE keeps one fixed payload length (the decoder rule below), and the ballistic build
  keeps transmitting the v0.6.3 TELEM bytes (bit-identical frames and time on air).
  The ground decodes both.
* Downlink ``TYPE_TELEM3`` (0x22, TELEM v3, v0.8, the VTOL build with GPS position hold), 37 payload bytes
  ``TELEM2_FMT + "BBBBH"``: the 31 TELEM v2 bytes, then gps_flags u8 (``sim.gps_nav`` GF_*: fix ok, ON gate ok,
  heading confidence, 3-D fix, configuration acknowledged, initialised, MON-VER answered, pad), num_sv u8, hacc u8
  (0.1 m, saturating at 255 = 25.5 m), pos_mode u8 (``sim.position_ctrl`` MODE_*: 0 off, 1 velocity hold, 2 position
  hold, 3 dead reckoning, 4 attitude only) and dist u16 (distance from the hold point, 0.1 m; ``DIST_NONE`` 0xFFFF =
  no hold point).  Again a new TYPE (one fixed length per TYPE); LEN_MAX grows to 40 (v0.7: 34).  The VTOL build without
  position hold (``SupervisorParams.gps_hold`` False) keeps TELEM v2; the ballistic build keeps TELEM v1.
* Downlink ``TYPE_TELEM4`` (0x23, TELEM v4, v0.9 firmware round RP1, the tether / hybrid power builds without the GPS),
  33 payload bytes ``TELEM2_FMT + "BB"``, and ``TYPE_TELEM5`` (0x24, TELEM v5, the power builds with the GPS), 39 payload
  bytes ``TELEM3_FMT + "BB"``: the TELEM v2 / v3 bytes, then the power tail of ``sim.power_link`` - v_aux u8 (0.125 V:
  the buffer in wired_buffer, V_tether = V_in in wired) and pwr_flags u8 (``power_link.PF_*``, bits 6-7 = the power
  build).  A frame carries the tail exactly when its power build (pwr_flags bits 6-7) is not the battery build
  (``PWR_TAIL_BUILD_SHIFT``): the battery VTOL build keeps TELEM v2 / v3 byte for byte.  Two new TYPEs (one fixed length
  per TYPE); LEN_MAX grows to 42 (v0.8: 40) in every build - the decoders take the longest frame of any build.  The
  power reason codes R_W_LOST 0x30 .. R_W_D2 0x33 and REJ_POWER 0x58 (v0.9 power rules, ``sim.onboard``) are defined
  here since RP1 (``sim.power_link`` re-exports them).
* Downlink ``TYPE_TELEM6`` (0x25, TELEM v6, 2026-11-10 control round, the VTOL battery build on the carrier rev 2c
  without the GPS), 32 payload bytes ``TELEM2_FMT + "B"``, and ``TYPE_TELEM7`` (0x26, TELEM v7, the same build with the
  GPS), 38 payload bytes ``TELEM3_FMT + "B"``: the TELEM v2 / v3 bytes, then the logic-battery tail - lbat u8, the
  FCU's PF3 reading of the 1S logic pack in 0.02 V steps (``LBAT_SCALE``; 4.20 V = 210), 1..255 (``LBAT_NONE`` 0 never
  travels: in the snapshot it means "this build has no monitor").  A frame carries the tail exactly when the FCU's
  snapshot has a non-zero lbat byte and no power tail (``logic_tail_due``): the tether / hybrid builds keep TELEM v4 /
  v5, the rocket and every build without the monitor keep TELEM v1 - v3 byte for byte.  LEN_MAX stays 42 (TELEM5).
  The ground shows it and warns below ``flight_computer.logic_battery_monitor.ground_warn_below_v_ASSUMED``; the
  vehicle takes no action on it (a user decision).
* Downlink SEQ = the SEQ of the uplink frame the telemetry answers (so the ground
  can tell a late reply to an older request from the current one).

Mission modes (v0.7, ``mission.mode``, a build-time choice): ``"vtol"`` (default; the
ground sends ON / OFF / UP / DOWN and KILL, the vehicle flies the altitude itself, SPEC
8.9) and ``"ballistic"`` (the v0.4-v0.6.3 ARM / LAUNCH / THROTTLE / CUT / RELEASE mission,
kept bit-identical).  ``MODE_COMMANDS`` lists the commands each build executes.  The
command codes are one table for both builds; the vehicle rejects the other build's
commands (VTOL: ``REJ_MODE``; ballistic: ``REJ_UNKNOWN_CMD`` - exactly what v0.6.3 answered
for the codes 8..11, so the ballistic build stays bit-identical).

Integer scaling (``quantize``): value x scale, rounded half away from zero,
saturated to the field's range; NaN -> 0.  C: ``q = x*scale; q = (q >= 0) ?
floor(q + 0.5) : -floor(-q + 0.5)`` then clamp.  The byte vectors in
``sim/out/link_vectors.json`` are built from integer fields, so they are exact
in any language.

Streaming decoder (``FrameDecoder``) - the algorithm the firmware mirrors:
a byte-wise parser HUNT0 (wait 0xA5) -> HUNT1 (0x5A; another 0xA5 restarts the
candidate) -> LEN (``LEN_MIN <= LEN <= LEN_MAX``) -> BODY (LEN + 2 bytes).  When a
candidate fails (LEN out of range or CRC mismatch) its first byte (the 0xA5) is
discarded and the remaining candidate bytes are parsed again from HUNT0, so a
real frame that starts inside garbage or inside a corrupted frame is still
found.  A frame whose CRC is correct is consumed whole even when it is then
dropped for a wrong vehicle id, an unexpected TYPE or a payload length that
does not match its TYPE ("dropped silently": counted in ``stats`` only).

Invariant: encode(decode(x)) == x for every valid frame, and the decoder never
emits a frame whose CRC, vehicle id, TYPE or length is wrong.
"""
from __future__ import annotations

import math
import struct
from collections import deque
from dataclasses import dataclass, field, replace
from typing import Iterable

# --------------------------------------------------------------------------- #
# frame constants
# --------------------------------------------------------------------------- #
SYNC0 = 0xA5
SYNC1 = 0x5A
TYPE_CMD = 0x10
TYPE_TELEM = 0x20
TYPE_TELEM2 = 0x21                     # v0.7 TELEM v2 (VTOL build)
TYPE_TELEM3 = 0x22                     # v0.8 TELEM v3 (VTOL build with GPS position hold)
TYPE_TELEM4 = 0x23                     # v0.9 TELEM v4 (RP1): TELEM v2 + the power tail (tether / hybrid builds)
TYPE_TELEM5 = 0x24                     # v0.9 TELEM v5 (RP1): TELEM v3 + the power tail (tether / hybrid builds with GPS)
TYPE_TELEM6 = 0x25                     # 2026-11-10 TELEM v6: TELEM v2 + the logic-battery tail (carrier battery build)
TYPE_TELEM7 = 0x26                     # 2026-11-10 TELEM v7: TELEM v3 + the logic-battery tail (carrier battery build + GPS)
HEADER_BYTES = 3                       # VEH, TYPE, SEQ
CMD_FMT = "<BHH"
TELEM_FMT = "<BBBBHIhhhhhHHHH"
TELEM2_FMT = TELEM_FMT + "Bh"          # + phase u8, h_tgt i16 (0.1 m)
TELEM3_FMT = TELEM2_FMT + "BBBBH"      # + gps_flags u8, num_sv u8, hacc u8 (0.1 m), pos_mode u8, dist u16 (0.1 m)
PWR_TAIL_FMT = "BB"                    # v0.9 power tail: v_aux u8 (0.125 V), pwr_flags u8 (sim.power_link PF_* | build << 6)
TELEM4_FMT = TELEM2_FMT + PWR_TAIL_FMT
TELEM5_FMT = TELEM3_FMT + PWR_TAIL_FMT
LBAT_TAIL_FMT = "B"                    # 2026-11-10 logic-battery tail: lbat u8 (LBAT_SCALE)
TELEM6_FMT = TELEM2_FMT + LBAT_TAIL_FMT
TELEM7_FMT = TELEM3_FMT + LBAT_TAIL_FMT
LBAT_SCALE = 50.0                      # lbat u8 = round(V x 50): 0.02 V steps, 0.02 .. 5.10 V
LBAT_NONE = 0                          # the snapshot's "this build has no logic-battery monitor" (never sent)
PWR_TAIL_BUILD_SHIFT = 6               # pwr_flags bits 6-7 = the power build (sim.power_link PF_BUILD_SHIFT); 0 = battery
CMD_PAYLOAD_LEN = struct.calcsize(CMD_FMT)        # 5
TELEM_PAYLOAD_LEN = struct.calcsize(TELEM_FMT)    # 28
TELEM2_PAYLOAD_LEN = struct.calcsize(TELEM2_FMT)  # 31
TELEM3_PAYLOAD_LEN = struct.calcsize(TELEM3_FMT)  # 37
TELEM4_PAYLOAD_LEN = struct.calcsize(TELEM4_FMT)  # 33
TELEM5_PAYLOAD_LEN = struct.calcsize(TELEM5_FMT)  # 39
TELEM6_PAYLOAD_LEN = struct.calcsize(TELEM6_FMT)  # 32
TELEM7_PAYLOAD_LEN = struct.calcsize(TELEM7_FMT)  # 38
PAYLOAD_LEN = {TYPE_CMD: CMD_PAYLOAD_LEN, TYPE_TELEM: TELEM_PAYLOAD_LEN, TYPE_TELEM2: TELEM2_PAYLOAD_LEN,
               TYPE_TELEM3: TELEM3_PAYLOAD_LEN, TYPE_TELEM4: TELEM4_PAYLOAD_LEN, TYPE_TELEM5: TELEM5_PAYLOAD_LEN,
               TYPE_TELEM6: TELEM6_PAYLOAD_LEN, TYPE_TELEM7: TELEM7_PAYLOAD_LEN}
TELEM_TYPES = (TYPE_TELEM, TYPE_TELEM2, TYPE_TELEM3, TYPE_TELEM4, TYPE_TELEM5, TYPE_TELEM6, TYPE_TELEM7)
TELEM_TYPES_V09 = (TYPE_TELEM, TYPE_TELEM2, TYPE_TELEM3, TYPE_TELEM4, TYPE_TELEM5)   # the v0.9 ground decoder
TELEM_TYPES_V08 = (TYPE_TELEM, TYPE_TELEM2, TYPE_TELEM3)    # the v0.8 ground decoder (and its test vectors)
MAX_PAYLOAD_LEN = max(PAYLOAD_LEN.values())
LEN_MIN = HEADER_BYTES                 # an empty payload is a well-formed frame (dropped later: no such TYPE)
LEN_MAX = HEADER_BYTES + MAX_PAYLOAD_LEN   # 42 (v0.6.3: 31, v0.7: 34, v0.8: 40)
FRAME_OVERHEAD = 2 + 1 + 2             # sync, LEN, CRC
CMD_FRAME_BYTES = FRAME_OVERHEAD + HEADER_BYTES + CMD_PAYLOAD_LEN       # 13
TELEM_FRAME_BYTES = FRAME_OVERHEAD + HEADER_BYTES + TELEM_PAYLOAD_LEN   # 36
TELEM2_FRAME_BYTES = FRAME_OVERHEAD + HEADER_BYTES + TELEM2_PAYLOAD_LEN  # 39
TELEM3_FRAME_BYTES = FRAME_OVERHEAD + HEADER_BYTES + TELEM3_PAYLOAD_LEN  # 45
TELEM4_FRAME_BYTES = FRAME_OVERHEAD + HEADER_BYTES + TELEM4_PAYLOAD_LEN  # 41
TELEM5_FRAME_BYTES = FRAME_OVERHEAD + HEADER_BYTES + TELEM5_PAYLOAD_LEN  # 47
TELEM6_FRAME_BYTES = FRAME_OVERHEAD + HEADER_BYTES + TELEM6_PAYLOAD_LEN  # 40
TELEM7_FRAME_BYTES = FRAME_OVERHEAD + HEADER_BYTES + TELEM7_PAYLOAD_LEN  # 46
DIST_NONE = 0xFFFF                     # TELEM v3 dist: no hold point
DIST_SCALE = 10.0                      # 0.1 m
HACC_NONE = 255
CRC_INIT = 0xFFFF
CRC_POLY = 0x1021

# --------------------------------------------------------------------------- #
# command codes (uplink cmd u8)
# --------------------------------------------------------------------------- #
CMD_NONE = 0          # heartbeat / poll
CMD_ARM = 1
CMD_DISARM = 2
CMD_LAUNCH = 3
CMD_THROTTLE = 4
CMD_CUT = 5
CMD_RELEASE = 6
CMD_KILL = 7
CMD_ON = 8            # v0.7 VTOL: take off (nonce), SPEC 8.9
CMD_OFF = 9           # v0.7 VTOL: land where it is / cancel the spool-up
CMD_UP = 10           # v0.7 VTOL: altitude target + alt_step_m
CMD_DOWN = 11         # v0.7 VTOL: altitude target - alt_step_m
CMD_NAMES = {CMD_NONE: "NONE", CMD_ARM: "ARM", CMD_DISARM: "DISARM", CMD_LAUNCH: "LAUNCH",
             CMD_THROTTLE: "THROTTLE", CMD_CUT: "CUT", CMD_RELEASE: "RELEASE", CMD_KILL: "KILL",
             CMD_ON: "ON", CMD_OFF: "OFF", CMD_UP: "UP", CMD_DOWN: "DOWN"}
THROTTLE_SCALE = 1000                  # arg = u x 1000
NONCE_COMMANDS = (CMD_LAUNCH, CMD_THROTTLE, CMD_ON)   # carry the nonce of the latest telemetry

# --------------------------------------------------------------------------- #
# mission modes (params.json mission.mode, build-time) and the commands each build executes
# --------------------------------------------------------------------------- #
MODE_BALLISTIC = "ballistic"
MODE_VTOL = "vtol"
MISSION_MODES = {MODE_BALLISTIC: 0, MODE_VTOL: 1}     # C: MISSION_MODE_BALLISTIC 0 / MISSION_MODE_VTOL 1
BALLISTIC_ONLY_CMDS = (CMD_ARM, CMD_DISARM, CMD_LAUNCH, CMD_THROTTLE, CMD_CUT, CMD_RELEASE)
VTOL_ONLY_CMDS = (CMD_ON, CMD_OFF, CMD_UP, CMD_DOWN)
MODE_COMMANDS = {MODE_BALLISTIC: BALLISTIC_ONLY_CMDS + (CMD_KILL,), MODE_VTOL: VTOL_ONLY_CMDS + (CMD_KILL,)}


def check_mode(mode: str) -> str:
    """``mode`` if it is a mission mode, else ValueError."""
    if mode not in MISSION_MODES:
        raise ValueError(f"mission.mode must be one of {sorted(MISSION_MODES)}, not {mode!r}")
    return mode


def mission_mode(p: dict) -> str:
    """params.json ``mission.mode``; a params file without the key (v0.6.3 and earlier) is the ballistic mission."""
    return check_mode(str(p.get("mission", {}).get("mode", MODE_BALLISTIC)))


# --------------------------------------------------------------------------- #
# VTOL phases (TELEM v2 phase u8) - sim.onboard: SAFE = NONE, ARMED = SPOOL, POWERED = CLIMB / HOLD / LAND,
# UNPOWERED = LANDED, ABORT / RELEASED = NONE; the ballistic build has no phase (TELEM v1)
# --------------------------------------------------------------------------- #
PHASE_NONE = 0
PHASE_SPOOL = 1       # ARMED: buzzer warning, motor off, spool_warning_s
PHASE_CLIMB = 2       # POWERED: take-off climb until the reference first reaches the target
PHASE_HOLD = 3        # POWERED: holding the altitude target (UP / DOWN move it)
PHASE_LAND = 4        # POWERED: descending to touchdown
PHASE_LANDED = 5      # UNPOWERED: touchdown confirmed, motor off, vanes neutral; SAFE after safe_after_landed_s
PHASE_NAMES = {PHASE_NONE: "NONE", PHASE_SPOOL: "SPOOL", PHASE_CLIMB: "CLIMB", PHASE_HOLD: "HOLD", PHASE_LAND: "LAND",
               PHASE_LANDED: "LANDED"}

# --------------------------------------------------------------------------- #
# vehicle states (telemetry state u8) - sim.onboard.State uses these values
# --------------------------------------------------------------------------- #
STATE_SAFE = 0
STATE_ARMED = 1
STATE_POWERED = 2
STATE_UNPOWERED = 3
STATE_RELEASED = 4
STATE_ABORT = 5
STATE_NAMES = {STATE_SAFE: "SAFE", STATE_ARMED: "ARMED", STATE_POWERED: "POWERED", STATE_UNPOWERED: "UNPOWERED",
               STATE_RELEASED: "RELEASED", STATE_ABORT: "ABORT"}

# --------------------------------------------------------------------------- #
# telemetry flag bits (SPEC §10.3 order)
# --------------------------------------------------------------------------- #
FLAG_ARM_SWITCH = 1 << 0       # arm switch closed (debounced)
FLAG_SENSORS_OK = 1 << 1       # IMU and baro healthy now (no failure timer running)
FLAG_ALIGNED = 1 << 2          # pad alignment complete
FLAG_LINK_OK = 1 << 3          # a valid ground frame within link_loss_disarm_s
FLAG_RELEASED = 1 << 4         # release servo open
FLAG_CONTROL_ACTIVE = 1 << 5   # attitude control running
FLAG_REJECTED = 1 << 6         # the command acknowledged by ack_seq was rejected (reason = its code)
FLAG_INTERLOCK = 1 << 7        # an interlock has acted since boot (latched until power cycle)
FLAG_NAMES = {FLAG_ARM_SWITCH: "arm_switch", FLAG_SENSORS_OK: "sensors_ok", FLAG_ALIGNED: "aligned",
              FLAG_LINK_OK: "link_ok", FLAG_RELEASED: "released", FLAG_CONTROL_ACTIVE: "control_active",
              FLAG_REJECTED: "rejected", FLAG_INTERLOCK: "interlock"}

# --------------------------------------------------------------------------- #
# reason codes (telemetry reason u8): the last interlock that acted, or the last
# command rejection - interlocks 1..63, rejections 0x40..0x7F.  A code names the RULE
# (its condition and parameters); the action may differ between the builds (the state
# and phase in the same telemetry show it): I8 and I9-baro land in the VTOL build and
# abort in the ballistic one, I10 boots ABORT (VTOL) or SAFE with the release open
# (ballistic).  VTOL rules with their own parameters have their own codes (V1..V7; v0.7.1: V8..V10).  In LANDED
# (v0.7.1) I9-IMU, V4 and V5 go to SAFE instead of ABORT: same code, the state in the telemetry shows the action.
# --------------------------------------------------------------------------- #
REASON_NONE = 0
R_I1_LINK_LOSS = 1          # ballistic POWERED/UNPOWERED: no valid ground frame for link_loss_abort_s -> ABORT
R_I1P_LINK_LOSS_PAD = 2     # ARMED (both builds): no valid ground frame for link_loss_disarm_s -> SAFE
R_I2_TILT = 3               # POWERED (both): tilt > tilt_abort_deg -> ABORT
R_I3_RATE = 4               # POWERED (both): spin / gyro saturation persisted -> ABORT
R_I4_APOGEE_LIMIT = 5       # ballistic POWERED: predicted apogee >= apogee_limit_m -> CUT
R_I5_GEOFENCE = 6           # ballistic POWERED/UNPOWERED: h > geofence_altitude_m -> ABORT
R_I6_FALL = 7               # ballistic POWERED/UNPOWERED: v_z <= fall_deploy_vz for fall_deploy_confirm_s -> RELEASE
R_I7_MOTOR_TIME = 8         # ballistic POWERED: cumulative powered time >= max_motor_time_s -> CUT
R_I8_VCELL = 9              # V_cell < min_cell_v: ballistic POWERED -> ABORT; VTOL CLIMB/HOLD -> LAND
R_I9_IMU = 10               # IMU failure persisted (both): ARMED -> SAFE, flight -> ABORT
R_I9_BARO = 11              # baro failure persisted: ARMED -> SAFE; ballistic flight -> ABORT; VTOL flight -> LAND
                            # on the NAV's dead reckoning (accelerometer with the learnt bias, v0.7.1)
R_I10_RESET = 12            # watchdog / brown-out reset in flight: ballistic boots SAFE with the release open, VTOL
                            # boots ABORT (release after abort_release_delay_s)
R_ARM_SWITCH_OPEN = 13      # arm switch opened while ARMED/POWERED/UNPOWERED: acts as KILL (both)
R_V_LINK_LAND = 14          # VTOL CLIMB/HOLD: no valid ground frame for link_loss_land_s -> LAND
R_V_ENDURANCE = 15          # VTOL CLIMB/HOLD: cumulative motor time >= max_flight_s - land_reserve_s -> LAND
R_V_CEILING = 16            # VTOL CLIMB/HOLD: h > ceiling_m -> LAND
R_V_GEOFENCE = 17           # VTOL POWERED/UNPOWERED: h > vtol.geofence_m -> ABORT
R_V_FALL = 18               # VTOL POWERED/UNPOWERED: v_z <= vtol.fall_vz_mps for vtol.fall_confirm_s -> ABORT
R_V_HARD_CUTOFF = 19        # VTOL POWERED: cumulative motor time >= hard_cutoff_s -> ABORT
R_V_TAKEOFF_GATE = 20       # VTOL end of the spool warning: tilt >= arm_tilt_max_deg or V_cell < min_cell_v -> SAFE
R_V_BARO_LIMIT_LANDED = 21  # v0.7.1 VTOL LAND on the dead reckoning (baro lost): the time limit T_lim from the altitude at
                            # the loss (mission.vtol.baro_lost) is reached with a ground signature (support / floor) ->
                            # LANDED (motor off, no release)
R_V_BARO_LIMIT_ABORT = 22   # v0.7.1 the same limit without a ground signature -> ABORT (the altitude is unknown, the
                            # vehicle may be high: motor off, release after abort_release_delay_s)
R_V_LAND_TIMEOUT = 23       # v0.7.1 VTOL LAND (baro healthy): low (h <= flare_altitude_m or the altitude loop saturated
                            # downward) for touchdown.land_floor_timeout_s without a confirmed touchdown -> LANDED (motor
                            # off, NO release)
R_V_GPS_LOST = 24           # v0.8 VTOL CLIMB/HOLD: no usable GPS for gps_loss.timeout_s (attitude-only hold) and then
                            # land_after_s -> LAND (the landing drifts with the wind, like v0.7.1)
R_V_FENCE = 25              # v0.8 VTOL CLIMB/HOLD: horizontal distance from the hold point > fence_m (field 15 / demo
                            # 5 m) -> LAND (never ABORT near people)
R_V_GPS_IMPLAUSIBLE = 26    # v0.8 VTOL CLIMB/HOLD: GPS and IMU disagree (tilt at its limit 3 s while |v| does not
                            # fall, |v| > v_max + 1 m/s for 2 s, or 3 NAV resets in 10 s) -> GPS unusable, attitude
                            # only + LAND
R_V_AUTHORITY = 27          # v0.8 VTOL CLIMB/HOLD: learnt k_T x T/W < authority_check min (1.08): no tilt authority for
                            # position hold -> LAND
R_V_TAKEOFF_CHECK = 28      # v0.8 VTOL CLIMB: the vertical controller at its ceiling for takeoff_check.within_s without
                            # climbing takeoff_check.climb_m -> motor off, SAFE (a T/W < 1 vehicle must not sit at u_max)
REJ_STATE = 0x40           # command not accepted in this state
REJ_NONCE = 0x41            # LAUNCH / THROTTLE-from-ARMED / ON nonce != telemetry nonce
REJ_ARM_SWITCH = 0x42       # ARM / ON: arm switch open
REJ_SENSORS = 0x43          # ARM / ON: IMU or baro not healthy
REJ_NOT_ALIGNED = 0x44      # ARM / ON: pad alignment incomplete
REJ_TILT = 0x45             # ARM / ON: tilt >= arm_tilt_max_deg
REJ_VCELL = 0x46            # ARM / ON: V_cell < min_cell_v
REJ_LINK = 0x47             # ARM / ON: no valid ground frame within link_loss_disarm_s before this one
REJ_UNKNOWN_CMD = 0x48      # cmd code not in the table (ballistic build: also ON / OFF / UP / DOWN, as in v0.6.3)
REJ_THROTTLE_ZERO = 0x49    # THROTTLE(u = 0) in ARMED (would enter UNPOWERED on the pad)
REJ_RELEASE_PENDING = 0x4A  # a release is pending: only KILL is accepted
REJ_RELEASED = 0x4B         # ARM / ON after the release has opened (I10 boot, ABORT): power-cycle first
REJ_MODE = 0x4C             # VTOL build: ARM / DISARM / LAUNCH / THROTTLE / CUT / RELEASE are not used
REJ_ALT_LIMIT = 0x4D        # UP / DOWN: the target would leave [alt_min_m, alt_max_m]
REJ_LANDING = 0x4E          # UP / DOWN during LAND: a landing is never cancelled
REJ_ENDURANCE = 0x4F        # ON: cumulative motor time >= max_flight_s - land_reserve_s (power-cycle, charged pack)
REJ_GPS = 0x50              # v0.8 ON: the GPS is not ready (3-D fix, satellites, hAcc, sAcc for stable_s - the NAV's
                            # GATE_OK; GPS needs open sky: no position hold indoors)
# v0.9 power rules (sim.onboard "v0.9 power rules", the tether / hybrid builds; a block of their own from 0x30 so codes
# still to be assigned to other rules cannot collide) - adopted from sim.power_link in the v0.9 firmware round (RP1)
R_W_LOST = 0x30             # wired_buffer: the tether is lost (V_tether low or below the buffer) -> LAND on the buffer
R_W_SAG = 0x31              # wired: the supply sags / is in its current limit (V_in low) -> throttle ceiling frozen, LAND
R_W_BUF = 0x32              # wired_buffer: the buffer below buf_low_cell_v under load -> LAND
R_W_D2 = 0x33               # wired_buffer: the buffer is being charged while tethered (D2 shorted) -> LAND
REJ_POWER = 0x58            # ON / UP refused: supply / tether / buffer voltage outside the ON window, or UP while sagging
REASON_NAMES = {
    REASON_NONE: "none", R_I1_LINK_LOSS: "I1 link loss", R_I1P_LINK_LOSS_PAD: "I1p link loss on the pad",
    R_I2_TILT: "I2 tilt", R_I3_RATE: "I3 spin/gyro saturation", R_I4_APOGEE_LIMIT: "I4 apogee limiter",
    R_I5_GEOFENCE: "I5 geofence", R_I6_FALL: "I6 fall-safe", R_I7_MOTOR_TIME: "I7 motor time",
    R_I8_VCELL: "I8 V_cell", R_I9_IMU: "I9 IMU failure", R_I9_BARO: "I9 baro failure", R_I10_RESET: "I10 reset in flight",
    R_ARM_SWITCH_OPEN: "arm switch opened",
    R_V_LINK_LAND: "V1 link loss: land", R_V_ENDURANCE: "V2 flight time: land", R_V_CEILING: "V3 ceiling: land",
    R_V_GEOFENCE: "V4 geofence: abort", R_V_FALL: "V5 fall: abort", R_V_HARD_CUTOFF: "V6 hard cutoff: abort",
    R_V_TAKEOFF_GATE: "V7 take-off gate: safe",
    R_V_BARO_LIMIT_LANDED: "V8 baro-lost time limit: landed (ground felt)",
    R_V_BARO_LIMIT_ABORT: "V9 baro-lost time limit: abort (no ground felt)",
    R_V_LAND_TIMEOUT: "V10 LAND floor timeout: landed (motor off, no release)",
    R_V_GPS_LOST: "V11 GPS lost: attitude-only hold, land", R_V_FENCE: "V12 horizontal fence: land",
    R_V_GPS_IMPLAUSIBLE: "V13 GPS implausible: attitude only, land", R_V_AUTHORITY: "V14 no tilt authority: land",
    R_V_TAKEOFF_CHECK: "V15 take-off check: no climb at full throttle, motor off (safe)",
    REJ_STATE: "rejected: state", REJ_NONCE: "rejected: nonce",
    REJ_ARM_SWITCH: "rejected: arm switch open", REJ_SENSORS: "rejected: sensors", REJ_NOT_ALIGNED: "rejected: not aligned",
    REJ_TILT: "rejected: tilt", REJ_VCELL: "rejected: V_cell", REJ_LINK: "rejected: link not established",
    REJ_UNKNOWN_CMD: "rejected: unknown command", REJ_THROTTLE_ZERO: "rejected: THROTTLE 0 on the pad",
    REJ_RELEASE_PENDING: "rejected: release pending", REJ_RELEASED: "rejected: release already open",
    REJ_MODE: "rejected: not a command of this mission mode", REJ_ALT_LIMIT: "rejected: altitude limit",
    REJ_LANDING: "rejected: landing", REJ_ENDURANCE: "rejected: flight time used (power-cycle with a charged pack)",
    REJ_GPS: "rejected: GPS not ready",
    # v0.9 power rules (the order and the texts of sim.power_link's v0.9 registration, so every log line is unchanged)
    R_W_LOST: "W_LOST tether lost: land on the buffer",
    R_W_SAG: "W_SAG supply sag / current limit: land",
    R_W_BUF: "W_BUF buffer low: land",
    R_W_D2: "W_D2 buffer charging while tethered (D2 short): land",
    REJ_POWER: "rejected: power (supply / tether / buffer voltage outside the ON window, or sagging)",
}
# the power builds' codes (sim.power_link re-exports them): kept out of the ALL_* / VTOL_* sets below, which name the
# codes of the battery builds exactly as before RP1 (the power codes were registered after those sets were built)
POWER_INTERLOCK_REASONS = (R_W_LOST, R_W_SAG, R_W_BUF, R_W_D2)
POWER_REJECT_REASONS = (REJ_POWER,)
_POWER_REASONS = POWER_INTERLOCK_REASONS + POWER_REJECT_REASONS
ALL_INTERLOCK_REASONS = tuple(k for k in REASON_NAMES if 0 < k < 0x40 and k not in _POWER_REASONS)
ALL_REJECT_REASONS = tuple(k for k in REASON_NAMES if k >= 0x40 and k not in _POWER_REASONS)
# the codes each build can produce (v0.6.3 names = the ballistic sets, so tools that check "every code occurred" on a
# ballistic run keep their meaning)
BALLISTIC_INTERLOCK_REASONS = tuple(range(R_I1_LINK_LOSS, R_ARM_SWITCH_OPEN + 1))
BALLISTIC_REJECT_REASONS = tuple(range(REJ_STATE, REJ_RELEASED + 1))
VTOL_INTERLOCK_REASONS = (R_I1P_LINK_LOSS_PAD, R_I2_TILT, R_I3_RATE, R_I8_VCELL, R_I9_IMU, R_I9_BARO, R_I10_RESET,
                          R_ARM_SWITCH_OPEN) + tuple(range(R_V_LINK_LAND, R_V_LAND_TIMEOUT + 1))   # v0.7.1: + V8 V9 V10
V08_INTERLOCK_REASONS = (R_V_GPS_LOST, R_V_FENCE, R_V_GPS_IMPLAUSIBLE, R_V_AUTHORITY, R_V_TAKEOFF_CHECK)
V08_REJECT_REASONS = (REJ_GPS,)
# VTOL: no THROTTLE; REJ_RELEASED needs SAFE with the release open, which only the ballistic I10 boot produces (the VTOL
# I10 boots ABORT -> RELEASED and RELEASED never leaves), so the VTOL ON gate keeps the check but can never fail it.
# The v0.8 codes belong to the VTOL build with GPS position hold (VTOL_GPS_*); the v0.7.1 VTOL sets stay as they were.
VTOL_REJECT_REASONS = tuple(k for k in ALL_REJECT_REASONS if k not in (REJ_THROTTLE_ZERO, REJ_RELEASED)
                            and k not in V08_REJECT_REASONS)
VTOL_GPS_INTERLOCK_REASONS = VTOL_INTERLOCK_REASONS + V08_INTERLOCK_REASONS
VTOL_GPS_REJECT_REASONS = VTOL_REJECT_REASONS + V08_REJECT_REASONS
INTERLOCK_REASONS = BALLISTIC_INTERLOCK_REASONS
REJECT_REASONS = BALLISTIC_REJECT_REASONS
MODE_REASONS = {MODE_BALLISTIC: (BALLISTIC_INTERLOCK_REASONS, BALLISTIC_REJECT_REASONS),
                MODE_VTOL: (VTOL_INTERLOCK_REASONS, VTOL_REJECT_REASONS)}
MODE_REASONS_GPS = (VTOL_GPS_INTERLOCK_REASONS, VTOL_GPS_REJECT_REASONS)   # the VTOL build with position hold

# --------------------------------------------------------------------------- #
# telemetry scaling (wire unit per physical unit)
# --------------------------------------------------------------------------- #
H_SCALE = 10.0          # i16, 0.1 m
VZ_SCALE = 100.0        # i16, cm/s
ANGLE_SCALE = 100.0     # i16 roll/pitch/yaw, u16 tilt: 0.01 deg
VCELL_SCALE = 1000.0    # u16, mV
U8, U16, U32 = (0, 0xFF), (0, 0xFFFF), (0, 0xFFFFFFFF)
I16 = (-0x8000, 0x7FFF)


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def crc16_ccitt(data: Iterable[int], crc: int = CRC_INIT) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout)."""
    for b in data:
        crc ^= (b & 0xFF) << 8
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def round_half_away(x: float) -> int:
    """Round half away from zero (C: ``x >= 0 ? floor(x + 0.5) : -floor(-x + 0.5)``); NaN -> 0."""
    if x != x:  # NaN
        return 0
    if x >= 0.0:
        return int(math.floor(x + 0.5))
    return -int(math.floor(-x + 0.5))


def quantize(x: float, scale: float, lim: tuple[int, int]) -> int:
    """Physical value -> saturated integer field (round half away from zero; +-inf saturate, NaN -> 0)."""
    if x != x:
        return 0
    v = x * scale
    if v >= lim[1]:
        return lim[1]
    if v <= lim[0]:
        return lim[0]
    return min(max(round_half_away(v), lim[0]), lim[1])


def seq_newer(a: int, b: int) -> bool:
    """True when 8-bit sequence number ``a`` is newer than ``b`` (serial-number arithmetic,
    RFC 1982 style): 1 <= (a - b) mod 256 <= 127."""
    d = (a - b) & 0xFF
    return 1 <= d <= 127


def _check_u(name: str, v: int, lim: tuple[int, int]) -> int:
    if not isinstance(v, int) or isinstance(v, bool) or not (lim[0] <= v <= lim[1]):
        raise ValueError(f"{name} = {v!r} outside [{lim[0]}, {lim[1]}]")
    return v


# --------------------------------------------------------------------------- #
# payloads
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Command:
    """Uplink CMD payload (wire integers)."""

    cmd: int
    arg: int = 0          # THROTTLE: u x 1000
    nonce: int = 0        # LAUNCH / THROTTLE-from-ARMED: the nonce of the latest telemetry

    def __post_init__(self) -> None:
        _check_u("cmd", self.cmd, U8)
        _check_u("arg", self.arg, U16)
        _check_u("nonce", self.nonce, U16)

    @classmethod
    def throttle(cls, u: float, nonce: int = 0) -> "Command":
        return cls(CMD_THROTTLE, quantize(u, THROTTLE_SCALE, U16), nonce)

    @property
    def name(self) -> str:
        return CMD_NAMES.get(self.cmd, f"CMD{self.cmd}")

    def pack(self) -> bytes:
        return struct.pack(CMD_FMT, self.cmd, self.arg, self.nonce)

    @classmethod
    def unpack(cls, payload: bytes) -> "Command":
        if len(payload) != CMD_PAYLOAD_LEN:
            raise ValueError(f"CMD payload is {CMD_PAYLOAD_LEN} bytes, got {len(payload)}")
        return cls(*struct.unpack(CMD_FMT, payload))


@dataclass(frozen=True)
class Telemetry:
    """Downlink TELEM payload, stored as the wire integers; physical values are properties.

    TELEM v1 (TYPE_TELEM, ballistic build): ``phase`` and ``h_tgt_dm`` are None.  TELEM v2 (TYPE_TELEM2, VTOL
    build): both are integers.  TELEM v3: + the GPS fields.  v0.9 (RP1): ``v_aux_q`` / ``pwr_flags`` set (the power
    tail, both or neither, on a v2 / v3 telemetry) -> TELEM v4 / v5.  2026-11-10: ``lbat_q`` set (the logic-battery
    tail, on a v2 / v3 telemetry without the power tail) -> TELEM v6 / v7.  ``pack`` / ``wire_type`` follow from which
    one it is."""

    state: int
    flags: int
    reason: int
    ack_seq: int
    nonce: int
    t_ms: int
    h_dm: int             # 0.1 m
    vz_cms: int           # cm/s
    roll_cdeg: int        # 0.01 deg
    pitch_cdeg: int
    yaw_cdeg: int
    tilt_cdeg: int        # 0.01 deg, unsigned
    throttle_milli: int   # x1000
    vcell_mv: int         # mV
    loop_max_us: int      # us
    phase: int | None = None      # TELEM v2: PHASE_*
    h_tgt_dm: int | None = None   # TELEM v2: altitude target, 0.1 m
    gps_flags: int | None = None  # TELEM v3: sim.gps_nav GF_*
    num_sv: int | None = None     # TELEM v3
    hacc_dm: int | None = None    # TELEM v3: 0.1 m, 255 = >= 25.5 m / none
    pos_mode: int | None = None   # TELEM v3: sim.position_ctrl MODE_*
    dist_dm: int | None = None    # TELEM v3: 0.1 m from the hold point, DIST_NONE = no hold point
    v_aux_q: int | None = None    # TELEM v4 / v5 power tail: v_aux, 0.125 V (sim.power_link.quant_v)
    pwr_flags: int | None = None  # TELEM v4 / v5 power tail: sim.power_link PF_* | build << 6
    lbat_q: int | None = None     # TELEM v6 / v7 logic-battery tail: 0.02 V (LBAT_SCALE), 1..255

    V3_FIELDS = ("gps_flags", "num_sv", "hacc_dm", "pos_mode", "dist_dm")
    PWR_FIELDS = ("v_aux_q", "pwr_flags")

    def __post_init__(self) -> None:
        for n in ("state", "flags", "reason", "ack_seq"):
            _check_u(n, getattr(self, n), U8)
        for n in ("nonce", "tilt_cdeg", "throttle_milli", "vcell_mv", "loop_max_us"):
            _check_u(n, getattr(self, n), U16)
        _check_u("t_ms", self.t_ms, U32)
        for n in ("h_dm", "vz_cms", "roll_cdeg", "pitch_cdeg", "yaw_cdeg"):
            _check_u(n, getattr(self, n), I16)
        if (self.phase is None) != (self.h_tgt_dm is None):
            raise ValueError("TELEM v2 needs both phase and h_tgt_dm (v1: neither)")
        if self.phase is not None:
            _check_u("phase", self.phase, U8)
            _check_u("h_tgt_dm", self.h_tgt_dm, I16)
        v3 = [getattr(self, n) is None for n in self.V3_FIELDS]
        if any(v3) != all(v3):
            raise ValueError("TELEM v3 needs every GPS field (v1 / v2: none)")
        if not v3[0]:
            if self.phase is None:
                raise ValueError("TELEM v3 extends TELEM v2: phase and h_tgt_dm are needed")
            for n in self.V3_FIELDS[:4]:
                _check_u(n, getattr(self, n), U8)
            _check_u("dist_dm", self.dist_dm, U16)
        if (self.v_aux_q is None) != (self.pwr_flags is None):
            raise ValueError("the TELEM v4 / v5 power tail needs both v_aux_q and pwr_flags (v1 - v3: neither)")
        if self.v_aux_q is not None:
            if self.phase is None:
                raise ValueError("the power tail extends TELEM v2 / v3 (TELEM v4 / v5): phase and h_tgt_dm are needed")
            _check_u("v_aux_q", self.v_aux_q, U8)
            _check_u("pwr_flags", self.pwr_flags, U8)
        if self.lbat_q is not None:
            if self.phase is None:
                raise ValueError("the logic-battery tail extends TELEM v2 / v3 (TELEM v6 / v7): phase and h_tgt_dm are needed")
            if self.v_aux_q is not None:
                raise ValueError("a TELEM carries the power tail (v4 / v5) or the logic-battery tail (v6 / v7), not both")
            _check_u("lbat_q", self.lbat_q, (1, 0xFF))

    @classmethod
    def from_physical(cls, *, state: int, flags: int, reason: int, ack_seq: int, nonce: int, t_ms: int, h_m: float,
                      vz_mps: float, roll_deg: float, pitch_deg: float, yaw_deg: float, tilt_deg: float,
                      throttle: float, vcell_v: float, loop_max_us: float, phase: int | None = None,
                      h_tgt_m: float | None = None, gps: tuple | None = None,
                      pwr: tuple | None = None, lbat_q: int | None = None) -> "Telemetry":
        """``phase`` / ``h_tgt_m`` given (VTOL build) -> TELEM v2; h_tgt is quantised like h (0.1 m).  ``gps`` =
        (gps_flags, num_sv, hacc_dm, pos_mode, dist_m) given as well -> TELEM v3: dist quantised to 0.1 m in
        [0, 0xFFFE], NaN (no hold point) -> DIST_NONE.  ``pwr`` = (v_aux_q, pwr_flags), the wire integers of the
        ``sim.power_link`` tail: appended (TELEM v4 / v5) only when its build bits (pwr_flags >> 6) are not the battery
        build - the rule ``power_tail_due`` (the COM's and the 6-DOF's).  ``lbat_q`` (the snapshot's logic-battery byte):
        appended (TELEM v6 / v7) when ``logic_tail_due`` (non-zero, no power tail)."""
        g: dict = {}
        if gps is not None:
            gf, sv, ha, pm, dist = gps
            dd = DIST_NONE if dist != dist else quantize(float(dist), DIST_SCALE, (0, DIST_NONE - 1))
            g = dict(gps_flags=int(gf) & 0xFF, num_sv=min(int(sv), 255), hacc_dm=min(int(ha), 255),
                     pos_mode=int(pm) & 0xFF, dist_dm=dd)
        if pwr is not None and phase is not None and power_tail_due(int(pwr[1])):
            g.update(v_aux_q=int(pwr[0]), pwr_flags=int(pwr[1]))
        if lbat_q is not None and phase is not None and logic_tail_due(int(lbat_q), int(g.get("pwr_flags") or 0)):
            g.update(lbat_q=int(lbat_q))
        return cls(state=state, flags=flags, reason=reason, ack_seq=ack_seq, nonce=nonce, t_ms=t_ms & 0xFFFFFFFF,
                   h_dm=quantize(h_m, H_SCALE, I16), vz_cms=quantize(vz_mps, VZ_SCALE, I16),
                   roll_cdeg=quantize(roll_deg, ANGLE_SCALE, I16), pitch_cdeg=quantize(pitch_deg, ANGLE_SCALE, I16),
                   yaw_cdeg=quantize(yaw_deg, ANGLE_SCALE, I16), tilt_cdeg=quantize(tilt_deg, ANGLE_SCALE, U16),
                   throttle_milli=quantize(throttle, THROTTLE_SCALE, U16), vcell_mv=quantize(vcell_v, VCELL_SCALE, U16),
                   loop_max_us=quantize(loop_max_us, 1.0, U16), phase=phase,
                   h_tgt_dm=None if h_tgt_m is None else quantize(h_tgt_m, H_SCALE, I16), **g)

    # wire format
    @property
    def is_v2(self) -> bool:
        """VTOL telemetry (v2 or v3: phase and target present)."""
        return self.phase is not None

    @property
    def is_v3(self) -> bool:
        return self.gps_flags is not None

    @property
    def has_pwr(self) -> bool:
        """v0.9: the power tail is present (TELEM v4 = v2 + tail, TELEM v5 = v3 + tail)."""
        return self.pwr_flags is not None

    @property
    def has_lbat(self) -> bool:
        """2026-11-10: the logic-battery tail is present (TELEM v6 = v2 + tail, TELEM v7 = v3 + tail)."""
        return self.lbat_q is not None

    @property
    def lbat_v(self) -> float | None:
        """The logic battery [V] of a TELEM v6 / v7, else None."""
        return None if self.lbat_q is None else self.lbat_q / LBAT_SCALE

    @property
    def power_tail(self) -> tuple[int, int] | None:
        """(v_aux_q, pwr_flags) of a TELEM v4 / v5, else None (``sim.power_link.PowerTelem(*power_tail)``)."""
        return None if self.pwr_flags is None else (self.v_aux_q, self.pwr_flags)

    @property
    def wire_type(self) -> int:
        if self.is_v3:
            return TYPE_TELEM5 if self.has_pwr else (TYPE_TELEM7 if self.has_lbat else TYPE_TELEM3)
        if self.is_v2:
            return TYPE_TELEM4 if self.has_pwr else (TYPE_TELEM6 if self.has_lbat else TYPE_TELEM2)
        return TYPE_TELEM

    @property
    def dist_m(self) -> float | None:
        """Distance from the hold point [m] (v3; None without a hold point or before v3)."""
        if self.dist_dm is None or self.dist_dm == DIST_NONE:
            return None
        return self.dist_dm / DIST_SCALE

    @property
    def hacc_m(self) -> float | None:
        return None if self.hacc_dm is None else self.hacc_dm / 10.0

    def gps_flag(self, bit: int) -> bool:
        return bool(self.gps_flags is not None and self.gps_flags & bit)

    @property
    def h_tgt_m(self) -> float | None:
        return None if self.h_tgt_dm is None else self.h_tgt_dm / H_SCALE

    @property
    def phase_name(self) -> str:
        if self.phase is None:
            return "-"
        return PHASE_NAMES.get(self.phase, f"PHASE{self.phase}")

    # physical views
    @property
    def h_m(self) -> float:
        return self.h_dm / H_SCALE

    @property
    def vz_mps(self) -> float:
        return self.vz_cms / VZ_SCALE

    @property
    def roll_deg(self) -> float:
        return self.roll_cdeg / ANGLE_SCALE

    @property
    def pitch_deg(self) -> float:
        return self.pitch_cdeg / ANGLE_SCALE

    @property
    def yaw_deg(self) -> float:
        return self.yaw_cdeg / ANGLE_SCALE

    @property
    def tilt_deg(self) -> float:
        return self.tilt_cdeg / ANGLE_SCALE

    @property
    def throttle(self) -> float:
        return self.throttle_milli / THROTTLE_SCALE

    @property
    def vcell_v(self) -> float:
        return self.vcell_mv / VCELL_SCALE

    @property
    def t_s(self) -> float:
        return self.t_ms / 1000.0

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, f"STATE{self.state}")

    @property
    def reason_name(self) -> str:
        return REASON_NAMES.get(self.reason, f"reason{self.reason}")

    def flag(self, bit: int) -> bool:
        return bool(self.flags & bit)

    def flag_names(self) -> list[str]:
        return [n for b, n in FLAG_NAMES.items() if self.flags & b]

    def wire_values(self) -> tuple:
        """The payload's integers in wire order (v1: 15, v2: 17, v3: 22, v4: 19, v5: 24, v6: 18, v7: 23)."""
        v = (self.state, self.flags, self.reason, self.ack_seq, self.nonce, self.t_ms, self.h_dm, self.vz_cms,
             self.roll_cdeg, self.pitch_cdeg, self.yaw_cdeg, self.tilt_cdeg, self.throttle_milli, self.vcell_mv,
             self.loop_max_us)
        if self.is_v2:
            v += (self.phase, self.h_tgt_dm)
        if self.is_v3:
            v += tuple(getattr(self, n) for n in self.V3_FIELDS)
        if self.has_pwr:
            v += (self.v_aux_q, self.pwr_flags)
        if self.has_lbat:
            v += (self.lbat_q,)
        return v

    def pack(self) -> bytes:
        """28-byte TELEM v1, 31-byte TELEM v2 (v1 + phase u8 + h_tgt i16), 37-byte TELEM v3 (v2 + the GPS fields),
        33-byte TELEM v4 (v2 + v_aux u8 + pwr_flags u8) or 39-byte TELEM v5 (v3 + the same two bytes), 32-byte TELEM v6
        (v2 + lbat u8) or 38-byte TELEM v7 (v3 + lbat u8)."""
        if self.is_v3:
            fmt = TELEM5_FMT if self.has_pwr else (TELEM7_FMT if self.has_lbat else TELEM3_FMT)
        elif self.is_v2:
            fmt = TELEM4_FMT if self.has_pwr else (TELEM6_FMT if self.has_lbat else TELEM2_FMT)
        else:
            fmt = TELEM_FMT
        return struct.pack(fmt, *self.wire_values())

    @classmethod
    def unpack(cls, payload: bytes) -> "Telemetry":
        """A 28-byte payload is TELEM v1, a 31-byte one TELEM v2, a 37-byte one TELEM v3, a 33-byte one TELEM v4, a
        39-byte one TELEM v5; else a ValueError."""
        if len(payload) == TELEM_PAYLOAD_LEN:
            return cls(*struct.unpack(TELEM_FMT, payload))
        if len(payload) == TELEM2_PAYLOAD_LEN:
            return cls(*struct.unpack(TELEM2_FMT, payload))
        if len(payload) == TELEM3_PAYLOAD_LEN:
            return cls(*struct.unpack(TELEM3_FMT, payload))
        if len(payload) == TELEM4_PAYLOAD_LEN:
            v = struct.unpack(TELEM4_FMT, payload)
            return cls(*v[:17], v_aux_q=v[17], pwr_flags=v[18])
        if len(payload) == TELEM5_PAYLOAD_LEN:
            return cls(*struct.unpack(TELEM5_FMT, payload))
        if len(payload) == TELEM6_PAYLOAD_LEN:
            v = struct.unpack(TELEM6_FMT, payload)
            return cls(*v[:17], lbat_q=v[17])
        if len(payload) == TELEM7_PAYLOAD_LEN:
            v = struct.unpack(TELEM7_FMT, payload)
            return cls(*v[:22], lbat_q=v[22])
        raise ValueError(f"TELEM payload is {TELEM_PAYLOAD_LEN} (v1), {TELEM2_PAYLOAD_LEN} (v2), {TELEM3_PAYLOAD_LEN} "
                         f"(v3), {TELEM4_PAYLOAD_LEN} (v4), {TELEM5_PAYLOAD_LEN} (v5), {TELEM6_PAYLOAD_LEN} (v6) or "
                         f"{TELEM7_PAYLOAD_LEN} (v7) bytes, got {len(payload)}")


def power_tail_due(pwr_flags: int) -> bool:
    """RP1 rule (the COM firmware's and the 6-DOF's): a reply carries the power tail (TELEM v4 / v5) exactly when the
    tail's build bits (pwr_flags >> PWR_TAIL_BUILD_SHIFT) are not the battery build (0)."""
    return ((int(pwr_flags) >> PWR_TAIL_BUILD_SHIFT) & 0x03) != 0


def lbat_quant(v: float | None) -> int:
    """The FCU's logic-battery byte for ``v`` volts: round(v x LBAT_SCALE) clamped to 1..255 (C: LP_LBAT_QUANT in
    core/link_proto_v09.h); None / NaN (no reading) -> 1 (the lowest level: the ground warns)."""
    if v is None or v != v:
        return 1
    return min(max(quantize(float(v), LBAT_SCALE, (0, 0xFF)), 1), 0xFF)


def lbat_q_from_counts(counts: int, q_x1024: int) -> int:
    """The FCU's lbat byte from its 10-bit PF3 code, in integers (main.c, params_gen.h LOGIC_BAT_Q_X1024 = round(V per
    count x LBAT_SCALE x 1024)): (counts x q_x1024 + 512) >> 10, clamped to 1..255."""
    return min(max((int(counts) * int(q_x1024) + 512) >> 10, 1), 0xFF)


def logic_tail_due(lbat_q: int, pwr_flags: int = 0) -> bool:
    """2026-11-10 rule (the COM firmware's and the 6-DOF's): a reply carries the logic-battery tail (TELEM v6 / v7)
    exactly when the snapshot's lbat byte is not ``LBAT_NONE`` and it carries no power tail (``power_tail_due``)."""
    return int(lbat_q) != LBAT_NONE and not power_tail_due(int(pwr_flags))


def lbat_build(p: dict, mode: str | None = None) -> bool:
    """True when the VTOL battery build of this params file reads the logic battery and sends TELEM v6 / v7: mode (None:
    ``mission_mode(p)``) is 'vtol' and ``flight_computer.logic_battery_monitor.telemetry_builds`` is 'vtol_battery' (the
    carrier rev 2c).  The power builds (tether / hybrid) never do - their callers exclude them (power tail)."""
    m = mission_mode(p) if mode is None else check_mode(mode)
    lbm = (p.get("flight_computer") or {}).get("logic_battery_monitor") or {}
    return m == MODE_VTOL and lbm.get("telemetry_builds") == "vtol_battery"


def with_logic_tail(tlm: Telemetry, lbat_q: int) -> Telemetry:
    """``tlm`` (TELEM v2 / v3) with the logic-battery tail if ``logic_tail_due`` (-> TELEM v6 / v7), else unchanged."""
    if tlm.phase is None or not logic_tail_due(lbat_q, tlm.pwr_flags or 0):
        return tlm
    return replace(tlm, lbat_q=int(lbat_q))


def with_power_tail(tlm: Telemetry, v_aux_q: int, pwr_flags: int) -> Telemetry:
    """``tlm`` (TELEM v2 / v3) with the power tail if ``power_tail_due(pwr_flags)`` (-> TELEM v4 / v5), else ``tlm``
    unchanged (the battery build's reply)."""
    if not power_tail_due(pwr_flags) or tlm.phase is None:
        return tlm
    return replace(tlm, v_aux_q=int(v_aux_q), pwr_flags=int(pwr_flags))


# --------------------------------------------------------------------------- #
# frames
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Frame:
    veh: int
    type: int
    seq: int
    payload: bytes

    def encode(self) -> bytes:
        return encode_frame(self.veh, self.type, self.seq, self.payload)


def encode_frame(veh: int, typ: int, seq: int, payload: bytes) -> bytes:
    _check_u("veh", veh, U8)
    _check_u("type", typ, U8)
    _check_u("seq", seq, U8)
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(f"payload {len(payload)} bytes > {MAX_PAYLOAD_LEN}")
    body = bytes([HEADER_BYTES + len(payload), veh, typ, seq]) + bytes(payload)
    crc = crc16_ccitt(body)
    return bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8])


def encode_command(veh: int, seq: int, cmd: Command) -> bytes:
    return encode_frame(veh, TYPE_CMD, seq, cmd.pack())


def encode_telemetry(veh: int, seq: int, tlm: Telemetry) -> bytes:
    """TYPE_TELEM frame for a v1 telemetry, TYPE_TELEM2 for v2, TYPE_TELEM3 for v3 (``Telemetry.wire_type``)."""
    return encode_frame(veh, tlm.wire_type, seq, tlm.pack())


def decode_command(frame: Frame) -> Command:
    if frame.type != TYPE_CMD:
        raise ValueError(f"frame type 0x{frame.type:02X} is not CMD")
    return Command.unpack(frame.payload)


def decode_telemetry(frame: Frame) -> Telemetry:
    """TELEM (v1) .. TELEM7 (v7) frame -> Telemetry; the payload length must be its TYPE's."""
    if frame.type not in TELEM_TYPES:
        raise ValueError(f"frame type 0x{frame.type:02X} is not TELEM / TELEM2 .. TELEM7")
    if len(frame.payload) != PAYLOAD_LEN[frame.type]:
        raise ValueError(f"TYPE 0x{frame.type:02X} payload is {PAYLOAD_LEN[frame.type]} bytes, got {len(frame.payload)}")
    return Telemetry.unpack(frame.payload)


_HUNT0, _HUNT1, _LEN, _BODY = 0, 1, 2, 3
STAT_KEYS = ("frames", "bad_len", "bad_crc", "wrong_vehicle", "wrong_type", "bad_payload_len", "skipped_bytes")


@dataclass
class FrameDecoder:
    """Streaming frame parser with resynchronisation (algorithm in the module docstring).

    ``vehicle_id`` None accepts any vehicle; ``accept_types`` lists the TYPEs this end
    consumes (the vehicle: CMD, the ground: TELEM and TELEM2)."""

    vehicle_id: int | None = None
    accept_types: tuple[int, ...] = (TYPE_CMD, TYPE_TELEM, TYPE_TELEM2)
    stats: dict[str, int] = field(default_factory=lambda: {k: 0 for k in STAT_KEYS})

    def __post_init__(self) -> None:
        self._state = _HUNT0
        self._buf = bytearray()
        self._need = 0
        self._queue: deque[int] = deque()

    def reset(self) -> None:
        self._state = _HUNT0
        self._buf.clear()
        self._queue.clear()

    def feed(self, data: bytes | bytearray | Iterable[int]) -> list[Frame]:
        out: list[Frame] = []
        self._queue.extend(data)
        q = self._queue
        while q:
            self._byte(q.popleft(), out)
        return out

    def _fail(self) -> None:
        """Discard the candidate's first byte (0xA5) and re-parse the rest from HUNT0."""
        rest = self._buf[1:]
        self._buf = bytearray()
        self._state = _HUNT0
        self.stats["skipped_bytes"] += 1
        self._queue.extendleft(reversed(rest))

    def _byte(self, b: int, out: list[Frame]) -> None:
        st = self._state
        if st == _HUNT0:
            if b == SYNC0:
                self._buf = bytearray([b])
                self._state = _HUNT1
            else:
                self.stats["skipped_bytes"] += 1
        elif st == _HUNT1:
            if b == SYNC1:
                self._buf.append(b)
                self._state = _LEN
            elif b == SYNC0:                       # A5 A5 5A ...: the second A5 starts the candidate
                self.stats["skipped_bytes"] += 1
                self._buf = bytearray([b])
            else:
                self.stats["skipped_bytes"] += 2
                self._buf = bytearray()
                self._state = _HUNT0
        elif st == _LEN:
            self._buf.append(b)
            if LEN_MIN <= b <= LEN_MAX:
                self._need = b + 2                 # VEH..PAYLOAD + CRC
                self._state = _BODY
            else:
                self.stats["bad_len"] += 1
                self._fail()
        else:  # _BODY
            self._buf.append(b)
            self._need -= 1
            if self._need > 0:
                return
            buf = self._buf
            ln = buf[2]
            crc_rx = buf[3 + ln] | (buf[4 + ln] << 8)
            if crc16_ccitt(buf[2:3 + ln]) != crc_rx:
                self.stats["bad_crc"] += 1
                self._fail()
                return
            veh, typ, seq = buf[3], buf[4], buf[5]
            payload = bytes(buf[6:3 + ln])
            self._buf = bytearray()
            self._state = _HUNT0
            if self.vehicle_id is not None and veh != self.vehicle_id:
                self.stats["wrong_vehicle"] += 1
            elif typ not in self.accept_types or typ not in PAYLOAD_LEN:
                self.stats["wrong_type"] += 1
            elif len(payload) != PAYLOAD_LEN[typ]:
                self.stats["bad_payload_len"] += 1
            else:
                self.stats["frames"] += 1
                out.append(Frame(veh, typ, seq, payload))


# --------------------------------------------------------------------------- #
# cross-check vectors for the firmware port (sim/out/link_vectors.json)
# --------------------------------------------------------------------------- #
def _fields(fr: Frame) -> dict:
    d = {"veh": fr.veh, "type": fr.type, "seq": fr.seq, "payload_hex": fr.payload.hex()}
    if fr.type == TYPE_CMD and len(fr.payload) == CMD_PAYLOAD_LEN:
        c = Command.unpack(fr.payload)
        d["cmd"] = {"cmd": c.cmd, "arg": c.arg, "nonce": c.nonce}
    elif fr.type in TELEM_TYPES and len(fr.payload) == PAYLOAD_LEN[fr.type]:
        # v1: the 15 v0.6.3 fields; v2: + phase, h_tgt_dm
        d["telem"] = {k: v for k, v in vars(Telemetry.unpack(fr.payload)).items() if v is not None}
    return d


def _decode_vector(name: str, data: bytes, vehicle_id: int, accept: tuple[int, ...], note: str = "") -> dict:
    dec = FrameDecoder(vehicle_id, accept)
    frames = dec.feed(data)
    one = FrameDecoder(vehicle_id, accept)            # byte-at-a-time must give the same result
    frames_1 = [f for b in data for f in one.feed(bytes([b]))]
    if frames_1 != frames or one.stats != dec.stats:  # pragma: no cover - self-check of the reference
        raise AssertionError(f"vector {name}: byte-wise decode differs from block decode")
    return {"name": name, "note": note, "hex": data.hex(), "decoder": {"vehicle_id": vehicle_id, "accept_types": list(accept)},
            "expected_frames": [_fields(f) for f in frames], "expected_stats": dict(dec.stats)}


def make_test_vectors(vehicle_id: int, seed: int = 20261013) -> dict:
    """Deterministic byte vectors: valid CMD / TELEM frames, corrupted frames and multi-frame streams with
    garbage, each with the frames the reference decoder emits and its counters.  ``expected_frames`` is
    normative for the firmware decoder; of the counters, ``frames``, ``bad_len``, ``bad_crc``,
    ``wrong_vehicle``, ``wrong_type`` and ``bad_payload_len`` are normative, ``skipped_bytes`` is informative."""
    import random
    rng = random.Random(seed)
    up, down = (TYPE_CMD,), (TYPE_TELEM,)
    vid = vehicle_id
    frames: list[dict] = []
    valid_up: list[bytes] = []
    valid_down: list[bytes] = []

    def add(name: str, data: bytes, accept: tuple[int, ...], note: str = "") -> None:
        frames.append(_decode_vector(name, data, vid, accept, note))

    # every v0.4 command code, edge arguments (the v0.7 VTOL codes: the block at the end)
    for code, nm in CMD_NAMES.items():
        if code in VTOL_ONLY_CMDS:
            continue
        b = encode_command(vid, (code * 37) & 0xFF, Command(code, 0, 0))
        valid_up.append(b)
        add(f"cmd_{nm.lower()}", b, up)
    for arg, nonce, seq in ((0, 0, 0), (1000, 0xBEEF, 127), (500, 1, 128), (0xFFFF, 0xFFFF, 255)):
        b = encode_command(vid, seq, Command(CMD_THROTTLE, arg, nonce))
        valid_up.append(b)
        add(f"cmd_throttle_arg{arg}_seq{seq}", b, up)
    b = encode_command(vid, 9, Command(CMD_LAUNCH, 0, 0x1234))
    valid_up.append(b)
    add("cmd_launch_nonce_0x1234", b, up)
    for i in range(10):
        c = Command(rng.randrange(0, 8), rng.randrange(0, 0x10000), rng.randrange(0, 0x10000))
        b = encode_command(vid, rng.randrange(0, 256), c)
        valid_up.append(b)
        add(f"cmd_random_{i}", b, up)
    # telemetry: hand-picked extremes and random
    t_cases = [
        Telemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        Telemetry(5, 0xFF, 0x4B, 255, 0xFFFF, 0xFFFFFFFF, 32767, -32768, -32768, 32767, -1, 65535, 65535, 65535, 65535),
        Telemetry.from_physical(state=STATE_POWERED, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED | FLAG_LINK_OK
                                | FLAG_CONTROL_ACTIVE, reason=REASON_NONE, ack_seq=2, nonce=0x5A5A, t_ms=12136, h_m=119.3,
                                vz_mps=22.71, roll_deg=-1.25, pitch_deg=0.5, yaw_deg=-179.99, tilt_deg=1.34, throttle=1.0,
                                vcell_v=3.9, loop_max_us=2875.0),
    ]
    for i in range(10):
        t_cases.append(Telemetry(rng.randrange(0, 6), rng.randrange(0, 256), rng.randrange(0, 256), rng.randrange(0, 256),
                                 rng.randrange(0, 0x10000), rng.randrange(0, 0x100000000),
                                 *[rng.randrange(-0x8000, 0x8000) for _ in range(5)],
                                 *[rng.randrange(0, 0x10000) for _ in range(4)]))
    for i, tlm in enumerate(t_cases):
        b = encode_telemetry(vid, (i * 29) & 0xFF, tlm)
        valid_down.append(b)
        add(f"telem_{i}", b, down)
    # corrupted single frames (each must produce no frame)
    for i, b in enumerate(valid_up[:6] + valid_down[:4]):
        accept = up if b[4] == TYPE_CMD else down
        j = rng.randrange(6, len(b) - 2)                       # a payload / header byte after LEN
        bad = bytearray(b)
        bad[j] ^= 1 << rng.randrange(0, 8)
        add(f"corrupt_bitflip_{i}", bytes(bad), accept, f"bit flip in byte {j} -> CRC mismatch")
    for i, b in enumerate(valid_up[6:9]):
        bad = bytearray(b)
        bad[-1 - (i % 2)] ^= 0x80
        add(f"corrupt_crc_{i}", bytes(bad), up, "CRC byte flipped")
    for ln in (0, 2, LEN_MAX + 1, 0xFF):
        bad = bytearray(valid_up[1])
        bad[2] = ln
        add(f"corrupt_len_{ln}", bytes(bad), up, f"LEN = {ln} outside {LEN_MIN}..{LEN_MAX}")
    bad = bytearray(valid_up[1])
    bad[2] = CMD_PAYLOAD_LEN + HEADER_BYTES + 1                 # plausible LEN, frame too short -> CRC fails / incomplete
    add("corrupt_len_plus1", bytes(bad), up, "LEN one too large: the decoder waits for one more byte, no frame")
    add("wrong_vehicle", encode_command((vid + 1) & 0xFF, 3, Command(CMD_ARM)), up, "valid CRC, other vehicle id")
    add("wrong_type_telem_on_uplink", valid_down[2], up, "valid TELEM frame fed to the vehicle's CMD decoder")
    add("wrong_type_0x30", encode_frame(vid, 0x30, 1, b"\x01\x02\x03\x04\x05"), up, "unknown TYPE")
    add("bad_payload_len", encode_frame(vid, TYPE_CMD, 4, b"\x01\x02\x03"), up, "CMD with a 3-byte payload")
    add("truncated", valid_up[2][:-3], up, "frame cut 3 bytes short")
    add("sync_broken", bytes([SYNC0, 0x00]) + valid_up[3][2:], up, "second sync byte wrong")
    # streams: garbage, back-to-back frames, a real frame starting inside a corrupted one
    streams: list[dict] = []

    def add_stream(name: str, data: bytes, accept: tuple[int, ...], note: str) -> None:
        streams.append(_decode_vector(name, data, vid, accept, note))

    add_stream("garbage_then_two_cmds", bytes([0x00, 0xFF, SYNC1, SYNC0, 0x13]) + valid_up[1] + valid_up[4], up,
               "garbage including a lone A5, then two frames back to back")
    add_stream("double_sync", bytes([SYNC0]) + valid_up[5], up, "A5 A5 5A ...: the second A5 starts the frame")
    cut = bytearray(valid_up[7])
    cut[-1] ^= 0x01
    add_stream("bad_crc_then_good", bytes(cut) + valid_up[8], up, "a corrupted frame followed by a good one")
    inner = valid_up[2]
    prefix = bytes([SYNC0, SYNC1, LEN_MAX])                     # fake header whose body swallows the real frame
    add_stream("frame_inside_bad_candidate", prefix + inner + bytes(rng.randrange(0, 256) for _ in range(LEN_MAX)), up,
               "a fake header (LEN_MAX) whose body contains a real frame: found again after the CRC failure")
    noise = bytes(rng.randrange(0, 256) for _ in range(40))
    add_stream("telem_stream_with_noise", noise[:17] + valid_down[3] + noise[17:] + valid_down[4] + valid_down[5], down,
               "random noise between telemetry frames")
    add_stream("mixed_vehicles", encode_command((vid + 7) & 0xFF, 1, Command(CMD_KILL)) + valid_up[6], up,
               "another vehicle's frame, then ours")

    # ---- v0.7 VTOL: ON / OFF / UP / DOWN, TELEM v2 (TYPE_TELEM2, 31 B); own RNG so the blocks above keep their bytes
    r2 = random.Random(seed + 7)
    down2 = (TYPE_TELEM, TYPE_TELEM2)                           # the v0.7 ground decoder
    vtol_up: list[bytes] = []
    for code in VTOL_ONLY_CMDS:
        nm = CMD_NAMES[code]
        for nonce, seq in ((0, (code * 37) & 0xFF), (0xBEEF, 127), (0xFFFF, 255)):
            b = encode_command(vid, seq, Command(code, 0, nonce))
            vtol_up.append(b)
            add(f"cmd_{nm.lower()}_nonce{nonce:04x}_seq{seq}", b, up)
    t2_cases = [
        Telemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PHASE_NONE, 0),
        Telemetry(5, 0xFF, 0x4F, 255, 0xFFFF, 0xFFFFFFFF, 32767, -32768, -32768, 32767, -1, 65535, 65535, 65535, 65535,
                  255, -32768),
        Telemetry(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PHASE_SPOOL, 32767),
        Telemetry.from_physical(state=STATE_POWERED, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED | FLAG_LINK_OK
                                | FLAG_CONTROL_ACTIVE, reason=REASON_NONE, ack_seq=12, nonce=0x5A5A, t_ms=21384, h_m=4.97,
                                vz_mps=-0.04, roll_deg=0.75, pitch_deg=-1.5, yaw_deg=12.0, tilt_deg=1.68, throttle=0.918,
                                vcell_v=3.86, loop_max_us=2875.0, phase=PHASE_HOLD, h_tgt_m=5.0),
        Telemetry.from_physical(state=STATE_UNPOWERED, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED | FLAG_LINK_OK,
                                reason=R_V_LINK_LAND, ack_seq=40, nonce=0x1357, t_ms=48000, h_m=-0.3, vz_mps=0.0,
                                roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, tilt_deg=2.0, throttle=0.0, vcell_v=3.71,
                                loop_max_us=2900.0, phase=PHASE_LANDED, h_tgt_m=0.0),
    ]
    for i in range(6):
        t2_cases.append(Telemetry(r2.randrange(0, 6), r2.randrange(0, 256), r2.randrange(0, 256), r2.randrange(0, 256),
                                  r2.randrange(0, 0x10000), r2.randrange(0, 0x100000000),
                                  *[r2.randrange(-0x8000, 0x8000) for _ in range(5)],
                                  *[r2.randrange(0, 0x10000) for _ in range(4)], r2.randrange(0, 6),
                                  r2.randrange(-0x8000, 0x8000)))
    valid_down2: list[bytes] = []
    for i, tlm in enumerate(t2_cases):
        b = encode_telemetry(vid, (i * 31 + 5) & 0xFF, tlm)
        valid_down2.append(b)
        add(f"telem2_{i}", b, down2)
    for i, b in enumerate(valid_down2[:4]):
        j = r2.randrange(6, len(b) - 2)
        bad = bytearray(b)
        bad[j] ^= 1 << r2.randrange(0, 8)
        add(f"corrupt_telem2_bitflip_{i}", bytes(bad), down2, f"bit flip in byte {j} -> CRC mismatch")
    bad = bytearray(valid_down2[3])
    bad[-2] ^= 0x01
    add("corrupt_telem2_crc", bytes(bad), down2, "CRC byte flipped")
    add("telem2_on_v1_decoder", valid_down2[3], down, "a TELEM2 frame fed to a TELEM-only (v0.6.3) decoder: wrong_type")
    add("telem2_on_uplink", valid_down2[3], up, "a TELEM2 frame fed to the vehicle's CMD decoder: wrong_type")
    add("telem_v1_len_31", encode_frame(vid, TYPE_TELEM, 3, valid_down2[3][6:-2]), down2,
        "TYPE TELEM (v1) with the 31-byte v2 payload: bad_payload_len")
    add("telem2_len_28", encode_frame(vid, TYPE_TELEM2, 3, valid_down[2][6:-2]), down2,
        "TYPE TELEM2 with a 28-byte v1 payload: bad_payload_len")
    for ln in (LEN_MAX, LEN_MAX + 1):
        bad = bytearray(valid_down2[3])
        bad[2] = ln if ln != bad[2] else ln - 1
        add(f"telem2_len_{bad[2]}", bytes(bad), down2, f"LEN changed to {bad[2]} (LEN_MAX {LEN_MAX})")
    add_stream("telem_v1_v2_mixed_with_noise", valid_down[0] + bytes(r2.randrange(256) for _ in range(9)) + valid_down2[0]
               + valid_down2[3] + valid_down[5] + bytes(r2.randrange(256) for _ in range(5)) + valid_down2[4], down2,
               "TELEM v1 and v2 frames back to back and between noise, one v0.7 ground decoder")
    inner2 = valid_down2[3]
    add_stream("telem2_inside_bad_candidate", bytes([SYNC0, SYNC1, LEN_MAX]) + inner2 +
               bytes(r2.randrange(256) for _ in range(LEN_MAX)), down2,
               "a fake header (LEN_MAX) whose body contains a TELEM2 frame: found again after the CRC failure")
    add_stream("vtol_cmds_back_to_back", b"".join(vtol_up[::3]) + encode_command(vid, 200, Command(CMD_KILL)), up,
               "ON, OFF, UP, DOWN, KILL back to back")

    # ---- v0.8 VTOL + GPS: TELEM v3 (TYPE_TELEM3, 37 B); own RNG so the blocks above keep their bytes
    r3 = random.Random(seed + 8)
    down3 = TELEM_TYPES_V08                                     # the v0.8 ground decoder: v1, v2, v3
    t3_cases = [
        Telemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PHASE_NONE, 0, 0, 0, 0, 0, 0),
        Telemetry(5, 0xFF, REJ_GPS, 255, 0xFFFF, 0xFFFFFFFF, 32767, -32768, -32768, 32767, -1, 65535, 65535, 65535, 65535,
                  255, -32768, 255, 255, 255, 255, 0xFFFF),
        Telemetry.from_physical(state=STATE_POWERED, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED | FLAG_LINK_OK
                                | FLAG_CONTROL_ACTIVE, reason=REASON_NONE, ack_seq=12, nonce=0x5A5A, t_ms=21384, h_m=1.52,
                                vz_mps=-0.04, roll_deg=0.75, pitch_deg=-1.5, yaw_deg=12.0, tilt_deg=3.68, throttle=0.918,
                                vcell_v=3.86, loop_max_us=2875.0, phase=PHASE_HOLD, h_tgt_m=1.5,
                                gps=(0x2F, 9, 25, 2, 0.734)),
        Telemetry.from_physical(state=STATE_SAFE, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED | FLAG_LINK_OK
                                | FLAG_REJECTED, reason=REJ_GPS, ack_seq=3, nonce=0x1357, t_ms=9000, h_m=0.0, vz_mps=0.0,
                                roll_deg=0.1, pitch_deg=0.2, yaw_deg=0.0, tilt_deg=0.3, throttle=0.0, vcell_v=4.1,
                                loop_max_us=2100.0, phase=PHASE_NONE, h_tgt_m=0.0, gps=(0x20, 4, 87, 0, math.nan)),
        Telemetry.from_physical(state=STATE_POWERED, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED | FLAG_LINK_OK
                                | FLAG_CONTROL_ACTIVE | FLAG_INTERLOCK, reason=R_V_FENCE, ack_seq=40, nonce=0x2468,
                                t_ms=48000, h_m=1.1, vz_mps=-0.4, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, tilt_deg=9.5,
                                throttle=0.93, vcell_v=3.71, loop_max_us=2900.0, phase=PHASE_LAND, h_tgt_m=0.0,
                                gps=(0x2B, 8, 31, 2, 5.06)),
    ]
    for i in range(6):
        t3_cases.append(Telemetry(r3.randrange(0, 6), r3.randrange(0, 256), r3.randrange(0, 256), r3.randrange(0, 256),
                                  r3.randrange(0, 0x10000), r3.randrange(0, 0x100000000),
                                  *[r3.randrange(-0x8000, 0x8000) for _ in range(5)],
                                  *[r3.randrange(0, 0x10000) for _ in range(4)], r3.randrange(0, 6),
                                  r3.randrange(-0x8000, 0x8000), r3.randrange(256), r3.randrange(256), r3.randrange(256),
                                  r3.randrange(5), r3.randrange(0x10000)))
    valid_down3: list[bytes] = []
    for i, tlm in enumerate(t3_cases):
        b = encode_telemetry(vid, (i * 37 + 11) & 0xFF, tlm)
        valid_down3.append(b)
        add(f"telem3_{i}", b, down3)
    for i, b in enumerate(valid_down3[:4]):
        j = r3.randrange(6, len(b) - 2)
        bad = bytearray(b)
        bad[j] ^= 1 << r3.randrange(0, 8)
        add(f"corrupt_telem3_bitflip_{i}", bytes(bad), down3, f"bit flip in byte {j} -> CRC mismatch")
    add("telem3_on_v2_decoder", valid_down3[2], down2, "a TELEM3 frame fed to a v0.7 ground decoder: wrong_type")
    add("telem3_len_31", encode_frame(vid, TYPE_TELEM3, 3, valid_down2[3][6:-2]), down3,
        "TYPE TELEM3 with a 31-byte v2 payload: bad_payload_len")
    add("telem3_len_max_plus1", bytes(valid_down3[2][:2]) + bytes([LEN_MAX + 1]) + valid_down3[2][3:], down3,
        f"LEN {LEN_MAX + 1} > LEN_MAX: bad_len")
    add_stream("telem_v1_v2_v3_mixed_with_noise", valid_down[1] + bytes(r3.randrange(256) for _ in range(7))
               + valid_down3[2] + valid_down2[3] + valid_down3[4] + bytes(r3.randrange(256) for _ in range(5))
               + valid_down3[0], down3, "TELEM v1 / v2 / v3 frames between noise, one v0.8 ground decoder")

    # ---- v0.9 RP1 power builds: TELEM v4 / v5 (TYPE_TELEM4 33 B, TYPE_TELEM5 39 B); own RNG so the blocks above keep their
    # draws (their bytes changed only where LEN_MAX enters: corrupt_len_43, the fake LEN_MAX headers)
    r4 = random.Random(seed + 9)
    down5 = TELEM_TYPES                                         # the v0.9 ground decoder: v1 .. v5

    def rnd_v2() -> list:
        return [r4.randrange(0, 6), r4.randrange(0, 256), r4.randrange(0, 256), r4.randrange(0, 256),
                r4.randrange(0, 0x10000), r4.randrange(0, 0x100000000), *[r4.randrange(-0x8000, 0x8000) for _ in range(5)],
                *[r4.randrange(0, 0x10000) for _ in range(4)], r4.randrange(0, 6), r4.randrange(-0x8000, 0x8000)]

    t4_cases = [
        Telemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PHASE_NONE, 0, v_aux_q=0, pwr_flags=0x40),
        Telemetry(5, 0xFF, R_W_D2, 255, 0xFFFF, 0xFFFFFFFF, 32767, -32768, -32768, 32767, -1, 65535, 65535, 65535, 65535,
                  255, -32768, v_aux_q=255, pwr_flags=0xFF),
        Telemetry.from_physical(state=STATE_POWERED, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED | FLAG_LINK_OK
                                | FLAG_CONTROL_ACTIVE | FLAG_INTERLOCK, reason=R_W_SAG, ack_seq=21, nonce=0x2B2B,
                                t_ms=15200, h_m=1.21, vz_mps=-0.2, roll_deg=0.4, pitch_deg=-0.9, yaw_deg=3.0,
                                tilt_deg=1.0, throttle=0.705, vcell_v=3.95, loop_max_us=2810.0, phase=PHASE_LAND,
                                h_tgt_m=0.0, pwr=(190, 0x43)),
        Telemetry.from_physical(state=STATE_SAFE, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED | FLAG_LINK_OK
                                | FLAG_REJECTED, reason=REJ_POWER, ack_seq=2, nonce=0x0102, t_ms=4000, h_m=0.0, vz_mps=0.0,
                                roll_deg=0.0, pitch_deg=0.1, yaw_deg=0.0, tilt_deg=0.2, throttle=0.0, vcell_v=3.6,
                                loop_max_us=2000.0, phase=PHASE_NONE, h_tgt_m=0.0, pwr=(170, 0x80 | 0x04)),
    ]
    for i in range(6):
        t4_cases.append(Telemetry(*rnd_v2(), v_aux_q=r4.randrange(256), pwr_flags=r4.randrange(0x40, 256)))
    t5_cases = [
        Telemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PHASE_NONE, 0, 0, 0, 0, 0, 0, v_aux_q=0, pwr_flags=0x40),
        Telemetry(5, 0xFF, R_W_LOST, 255, 0xFFFF, 0xFFFFFFFF, 32767, -32768, -32768, 32767, -1, 65535, 65535, 65535, 65535,
                  255, -32768, 255, 255, 255, 255, 0xFFFF, v_aux_q=255, pwr_flags=0xFF),
        Telemetry.from_physical(state=STATE_POWERED, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED | FLAG_LINK_OK
                                | FLAG_CONTROL_ACTIVE, reason=REASON_NONE, ack_seq=12, nonce=0x5A5A, t_ms=21384, h_m=1.52,
                                vz_mps=-0.04, roll_deg=0.75, pitch_deg=-1.5, yaw_deg=12.0, tilt_deg=3.68, throttle=0.918,
                                vcell_v=4.12, loop_max_us=2875.0, phase=PHASE_HOLD, h_tgt_m=1.5,
                                gps=(0x2F, 9, 25, 2, 0.734), pwr=(198, 0x80 | 0x01 | 0x08)),
    ]
    for i in range(6):
        t5_cases.append(Telemetry(*rnd_v2(), r4.randrange(256), r4.randrange(256), r4.randrange(256), r4.randrange(5),
                                  r4.randrange(0x10000), v_aux_q=r4.randrange(256), pwr_flags=r4.randrange(0x40, 256)))
    valid_down4: list[bytes] = []
    for i, tlm in enumerate(t4_cases):
        b = encode_telemetry(vid, (i * 41 + 3) & 0xFF, tlm)
        valid_down4.append(b)
        add(f"telem4_{i}", b, down5)
    valid_down5: list[bytes] = []
    for i, tlm in enumerate(t5_cases):
        b = encode_telemetry(vid, (i * 43 + 7) & 0xFF, tlm)
        valid_down5.append(b)
        add(f"telem5_{i}", b, down5)
    for i, b in enumerate(valid_down4[:2] + valid_down5[:2]):
        j = r4.randrange(6, len(b) - 2)
        bad = bytearray(b)
        bad[j] ^= 1 << r4.randrange(0, 8)
        add(f"corrupt_telem45_bitflip_{i}", bytes(bad), down5, f"bit flip in byte {j} -> CRC mismatch")
    add("telem4_on_v3_decoder", valid_down4[2], down3, "a TELEM4 frame fed to a v0.8 ground decoder: wrong_type")
    add("telem5_on_v3_decoder", valid_down5[2], down3, "a TELEM5 frame fed to a v0.8 ground decoder: wrong_type")
    add("telem5_on_uplink", valid_down5[2], up, "a TELEM5 frame fed to the vehicle's CMD decoder: wrong_type")
    add("telem4_len_31", encode_frame(vid, TYPE_TELEM4, 3, valid_down2[3][6:-2]), down5,
        "TYPE TELEM4 with a 31-byte v2 payload (no power tail): bad_payload_len")
    add("telem2_len_33", encode_frame(vid, TYPE_TELEM2, 3, valid_down4[2][6:-2]), down5,
        "TYPE TELEM2 with the 33-byte v4 payload: bad_payload_len")
    add("telem5_len_37", encode_frame(vid, TYPE_TELEM5, 3, valid_down3[2][6:-2]), down5,
        "TYPE TELEM5 with a 37-byte v3 payload (no power tail): bad_payload_len")
    add("telem5_len_max", valid_down5[2], down5, f"TELEM5: LEN {valid_down5[2][2]} = LEN_MAX {LEN_MAX} (the longest frame)")
    add("telem5_len_max_plus1", bytes(valid_down5[2][:2]) + bytes([LEN_MAX + 1]) + valid_down5[2][3:], down5,
        f"LEN {LEN_MAX + 1} > LEN_MAX: bad_len")
    add_stream("telem_v1_to_v5_mixed_with_noise", valid_down[1] + bytes(r4.randrange(256) for _ in range(6))
               + valid_down4[2] + valid_down3[2] + valid_down5[2] + bytes(r4.randrange(256) for _ in range(4))
               + valid_down2[3] + valid_down5[0] + valid_down4[0], down5,
               "TELEM v1 .. v5 frames between noise, one v0.9 ground decoder")
    add_stream("telem5_inside_bad_candidate", bytes([SYNC0, SYNC1, LEN_MAX]) + valid_down5[3] +
               bytes(r4.randrange(256) for _ in range(LEN_MAX)), down5,
               "a fake header (LEN_MAX) whose body contains a TELEM5 frame: found again after the CRC failure")

    # ---- 2026-11-10 control round: TELEM v6 / v7 (the logic-battery tail, TYPE_TELEM6 32 B / TYPE_TELEM7 38 B); own RNG
    # so every vector above keeps its bytes
    r5 = random.Random(seed + 11)
    down7 = TELEM_TYPES                                         # the 2026-11-10 ground decoder: v1 .. v7

    def rnd_v2b() -> list:
        return [r5.randrange(0, 6), r5.randrange(0, 256), r5.randrange(0, 256), r5.randrange(0, 256),
                r5.randrange(0, 0x10000), r5.randrange(0, 0x100000000), *[r5.randrange(-0x8000, 0x8000) for _ in range(5)],
                *[r5.randrange(0, 0x10000) for _ in range(4)], r5.randrange(0, 6), r5.randrange(-0x8000, 0x8000)]

    t6_cases = [Telemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PHASE_NONE, 0, lbat_q=1),
                Telemetry(5, 0xFF, 0xFF, 255, 0xFFFF, 0xFFFFFFFF, 32767, -32768, -32768, 32767, -1, 65535, 65535, 65535,
                          65535, 255, -32768, lbat_q=255),
                Telemetry.from_physical(state=STATE_POWERED, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED
                                        | FLAG_LINK_OK | FLAG_CONTROL_ACTIVE, reason=REASON_NONE, ack_seq=9, nonce=0x4242,
                                        t_ms=12000, h_m=1.5, vz_mps=0.02, roll_deg=0.3, pitch_deg=-0.6, yaw_deg=2.0,
                                        tilt_deg=0.7, throttle=0.61, vcell_v=3.98, loop_max_us=2790.0, phase=PHASE_HOLD,
                                        h_tgt_m=1.5, lbat_q=lbat_quant(3.86))]
    for i in range(5):
        t6_cases.append(Telemetry(*rnd_v2b(), lbat_q=r5.randrange(1, 256)))
    t7_cases = [Telemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PHASE_NONE, 0, 0, 0, 0, 0, 0, lbat_q=1),
                Telemetry.from_physical(state=STATE_POWERED, flags=FLAG_ARM_SWITCH | FLAG_SENSORS_OK | FLAG_ALIGNED
                                        | FLAG_LINK_OK | FLAG_CONTROL_ACTIVE, reason=REASON_NONE, ack_seq=12, nonce=0x5A5A,
                                        t_ms=21384, h_m=1.52, vz_mps=-0.04, roll_deg=0.75, pitch_deg=-1.5, yaw_deg=12.0,
                                        tilt_deg=3.68, throttle=0.918, vcell_v=4.12, loop_max_us=2875.0, phase=PHASE_HOLD,
                                        h_tgt_m=1.5, gps=(0x2F, 9, 25, 2, 0.734), lbat_q=lbat_quant(3.58))]
    for i in range(5):
        t7_cases.append(Telemetry(*rnd_v2b(), r5.randrange(256), r5.randrange(256), r5.randrange(256), r5.randrange(5),
                                  r5.randrange(0x10000), lbat_q=r5.randrange(1, 256)))
    valid_down6: list[bytes] = []
    for i, tlm in enumerate(t6_cases):
        b = encode_telemetry(vid, (i * 47 + 5) & 0xFF, tlm)
        valid_down6.append(b)
        add(f"telem6_{i}", b, down7)
    valid_down7: list[bytes] = []
    for i, tlm in enumerate(t7_cases):
        b = encode_telemetry(vid, (i * 53 + 9) & 0xFF, tlm)
        valid_down7.append(b)
        add(f"telem7_{i}", b, down7)
    for i, b in enumerate(valid_down6[:2] + valid_down7[:2]):
        j = r5.randrange(6, len(b) - 2)
        bad = bytearray(b)
        bad[j] ^= 1 << r5.randrange(0, 8)
        add(f"corrupt_telem67_bitflip_{i}", bytes(bad), down7, f"bit flip in byte {j} -> CRC mismatch")
    add("telem6_on_v09_decoder", valid_down6[2], down5, "a TELEM6 frame fed to a v0.9 ground decoder: wrong_type")
    add("telem7_on_v09_decoder", valid_down7[1], down5, "a TELEM7 frame fed to a v0.9 ground decoder: wrong_type")
    add("telem7_on_uplink", valid_down7[1], up, "a TELEM7 frame fed to the vehicle's CMD decoder: wrong_type")
    add("telem6_len_31", encode_frame(vid, TYPE_TELEM6, 3, valid_down2[3][6:-2]), down7,
        "TYPE TELEM6 with a 31-byte v2 payload (no tail): bad_payload_len")
    add("telem7_len_39", encode_frame(vid, TYPE_TELEM7, 3, valid_down5[2][6:-2]), down7,
        "TYPE TELEM7 with the 39-byte v5 payload: bad_payload_len")
    add("telem3_len_38", encode_frame(vid, TYPE_TELEM3, 3, valid_down7[1][6:-2]), down7,
        "TYPE TELEM3 with the 38-byte v7 payload: bad_payload_len")
    add_stream("telem_v1_to_v7_mixed_with_noise", valid_down[1] + bytes(r5.randrange(256) for _ in range(5))
               + valid_down6[2] + valid_down3[2] + valid_down7[1] + bytes(r5.randrange(256) for _ in range(4))
               + valid_down5[2] + valid_down6[0] + valid_down4[0] + valid_down7[0], down7,
               "TELEM v1 .. v7 frames between noise, one 2026-11-10 ground decoder")
    return {
        "_doc": ("LoRa link byte vectors (SPEC 10.3), generated by sim/link_proto.py make_test_vectors(); each vector is fed to a "
                 "fresh FrameDecoder(vehicle_id, accept_types); expected_frames are normative, see make_test_vectors() for the "
                 "counters. Telemetry fields are the wire integers (h 0.1 m, v_z cm/s, angles 0.01 deg, throttle x1000, mV, us; "
                 "TELEM2 (v0.7 VTOL, TYPE 0x21) adds phase and h_tgt_dm (0.1 m); TELEM3 (v0.8 VTOL + GPS, TYPE 0x22) adds "
                 "gps_flags, num_sv, hacc_dm (0.1 m), pos_mode, dist_dm (0.1 m, 0xFFFF none); TELEM4 / TELEM5 (v0.9 power "
                 "builds, TYPE 0x23 / 0x24) add v_aux_q (0.125 V) and pwr_flags to TELEM2 / TELEM3; TELEM6 / TELEM7 "
                 "(2026-11-10, the carrier battery build, TYPE 0x25 / 0x26) add lbat_q (0.02 V) to TELEM2 / TELEM3; "
                 "LEN_MAX 42)."),
        "seed": seed, "vehicle_id": vid,
        "crc16_ccitt_false_check": {"ascii": "123456789", "crc": crc16_ccitt(b"123456789")},
        "constants": {"SYNC": [SYNC0, SYNC1], "TYPE_CMD": TYPE_CMD, "TYPE_TELEM": TYPE_TELEM, "TYPE_TELEM2": TYPE_TELEM2,
                      "TYPE_TELEM3": TYPE_TELEM3, "LEN_MIN": LEN_MIN, "LEN_MAX": LEN_MAX, "CMD_FRAME_BYTES": CMD_FRAME_BYTES,
                      "TELEM_FRAME_BYTES": TELEM_FRAME_BYTES, "TELEM2_FRAME_BYTES": TELEM2_FRAME_BYTES,
                      "TELEM3_FRAME_BYTES": TELEM3_FRAME_BYTES, "DIST_NONE": DIST_NONE,
                      "CMD_ON": CMD_ON, "CMD_OFF": CMD_OFF, "CMD_UP": CMD_UP, "CMD_DOWN": CMD_DOWN,
                      "TYPE_TELEM4": TYPE_TELEM4, "TYPE_TELEM5": TYPE_TELEM5, "TELEM4_FRAME_BYTES": TELEM4_FRAME_BYTES,
                      "TELEM5_FRAME_BYTES": TELEM5_FRAME_BYTES, "PWR_TAIL_BUILD_SHIFT": PWR_TAIL_BUILD_SHIFT,
                      "R_W_LOST": R_W_LOST, "R_W_SAG": R_W_SAG, "R_W_BUF": R_W_BUF, "R_W_D2": R_W_D2,
                      "REJ_POWER": REJ_POWER, "TYPE_TELEM6": TYPE_TELEM6, "TYPE_TELEM7": TYPE_TELEM7,
                      "TELEM6_FRAME_BYTES": TELEM6_FRAME_BYTES, "TELEM7_FRAME_BYTES": TELEM7_FRAME_BYTES,
                      "LBAT_SCALE": LBAT_SCALE},
        "scaling": [
            {"field": "h_dm", "physical": 12.25, "int": quantize(12.25, H_SCALE, I16)},
            {"field": "h_dm", "physical": -12.25, "int": quantize(-12.25, H_SCALE, I16)},
            {"field": "h_dm", "physical": 5000.0, "int": quantize(5000.0, H_SCALE, I16)},
            {"field": "vz_cms", "physical": 0.125, "int": quantize(0.125, VZ_SCALE, I16)},
            {"field": "vz_cms", "physical": -400.0, "int": quantize(-400.0, VZ_SCALE, I16)},
            {"field": "tilt_cdeg", "physical": -0.5, "int": quantize(-0.5, ANGLE_SCALE, U16)},
            {"field": "throttle_milli", "physical": 0.0625, "int": quantize(0.0625, THROTTLE_SCALE, U16)},
            {"field": "vcell_mv", "physical": 3.3, "int": quantize(3.3, VCELL_SCALE, U16)},
        ],
        "frames": frames,
        "streams": streams,
    }
