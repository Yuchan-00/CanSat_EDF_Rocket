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
* Downlink SEQ = the SEQ of the uplink frame the telemetry answers (so the ground
  can tell a late reply to an older request from the current one).

Mission mode: the rocket's ballistic program only (``mission.mode`` 'ballistic', SPEC 8.8).  The command codes 8..11
(the removed VTOL program's ON / OFF / UP / DOWN) are not in the table: the vehicle answers them REJ_UNKNOWN_CMD,
exactly what v0.6.3 answered.  ``LEN_MAX`` stays 40 (the v0.8.1 decoder limit) so the flight images stay byte-identical;
with TELEM v1 as the longest frame, a candidate with LEN 32..40 now fails at its CRC or payload length.

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
from dataclasses import dataclass, field
from typing import Iterable

# --------------------------------------------------------------------------- #
# frame constants
# --------------------------------------------------------------------------- #
SYNC0 = 0xA5
SYNC1 = 0x5A
TYPE_CMD = 0x10
TYPE_TELEM = 0x20
HEADER_BYTES = 3                       # VEH, TYPE, SEQ
CMD_FMT = "<BHH"
TELEM_FMT = "<BBBBHIhhhhhHHHH"
CMD_PAYLOAD_LEN = struct.calcsize(CMD_FMT)        # 5
TELEM_PAYLOAD_LEN = struct.calcsize(TELEM_FMT)    # 28
PAYLOAD_LEN = {TYPE_CMD: CMD_PAYLOAD_LEN, TYPE_TELEM: TELEM_PAYLOAD_LEN}
TELEM_TYPES = (TYPE_TELEM,)
LEN_MIN = HEADER_BYTES                 # an empty payload is a well-formed frame (dropped later: no such TYPE)
LEN_MAX = 40                           # the v0.8.1 decoder limit, kept: the flight images stay byte-identical (module doc)
MAX_PAYLOAD_LEN = LEN_MAX - HEADER_BYTES   # 37: the longest payload a frame can carry (the decoders' buffers)
FRAME_OVERHEAD = 2 + 1 + 2             # sync, LEN, CRC
CMD_FRAME_BYTES = FRAME_OVERHEAD + HEADER_BYTES + CMD_PAYLOAD_LEN       # 13
TELEM_FRAME_BYTES = FRAME_OVERHEAD + HEADER_BYTES + TELEM_PAYLOAD_LEN   # 36
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
CMD_NAMES = {CMD_NONE: "NONE", CMD_ARM: "ARM", CMD_DISARM: "DISARM", CMD_LAUNCH: "LAUNCH",
             CMD_THROTTLE: "THROTTLE", CMD_CUT: "CUT", CMD_RELEASE: "RELEASE", CMD_KILL: "KILL"}
THROTTLE_SCALE = 1000                  # arg = u x 1000
NONCE_COMMANDS = (CMD_LAUNCH, CMD_THROTTLE)   # carry the nonce of the latest telemetry

# --------------------------------------------------------------------------- #
# mission mode (params.json mission.mode): the rocket's ballistic program is the only one
# --------------------------------------------------------------------------- #
MODE_BALLISTIC = "ballistic"
MISSION_MODES = {MODE_BALLISTIC: 0}                  # C: MISSION_MODE_BALLISTIC 0
BALLISTIC_ONLY_CMDS = (CMD_ARM, CMD_DISARM, CMD_LAUNCH, CMD_THROTTLE, CMD_CUT, CMD_RELEASE)
MODE_COMMANDS = {MODE_BALLISTIC: BALLISTIC_ONLY_CMDS + (CMD_KILL,)}


def check_mode(mode: str) -> str:
    """``mode`` if it is a mission mode, else ValueError."""
    if mode not in MISSION_MODES:
        raise ValueError(f"mission.mode must be one of {sorted(MISSION_MODES)}, not {mode!r}")
    return mode


def mission_mode(p: dict) -> str:
    """params.json ``mission.mode``; a params file without the key (v0.6.3 and earlier) is the ballistic mission."""
    return check_mode(str(p.get("mission", {}).get("mode", MODE_BALLISTIC)))


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
# (its condition and parameters).  The codes 14..28, 0x30..0x33 and 0x4C..0x58 belonged to the removed VTOL
# program (2026-11-18) and are not used.
# --------------------------------------------------------------------------- #
REASON_NONE = 0
R_I1_LINK_LOSS = 1          # ballistic POWERED/UNPOWERED: no valid ground frame for link_loss_abort_s -> ABORT
R_I1P_LINK_LOSS_PAD = 2     # ARMED: no valid ground frame for link_loss_disarm_s -> SAFE
R_I2_TILT = 3               # POWERED: tilt > tilt_abort_deg -> ABORT
R_I3_RATE = 4               # POWERED: spin / gyro saturation persisted -> ABORT
R_I4_APOGEE_LIMIT = 5       # ballistic POWERED: predicted apogee >= apogee_limit_m -> CUT
R_I5_GEOFENCE = 6           # ballistic POWERED/UNPOWERED: h > geofence_altitude_m -> ABORT
R_I6_FALL = 7               # ballistic POWERED/UNPOWERED: v_z <= fall_deploy_vz for fall_deploy_confirm_s -> RELEASE
R_I7_MOTOR_TIME = 8         # ballistic POWERED: cumulative powered time >= max_motor_time_s -> CUT
R_I8_VCELL = 9              # POWERED: V_cell < min_cell_v -> ABORT
R_I9_IMU = 10               # IMU failure persisted: ARMED -> SAFE, flight -> ABORT
R_I9_BARO = 11              # baro failure persisted: ARMED -> SAFE; flight -> ABORT
R_I10_RESET = 12            # watchdog / brown-out reset in flight: boots SAFE with the release open
R_ARM_SWITCH_OPEN = 13      # arm switch opened while ARMED/POWERED/UNPOWERED: acts as KILL
REJ_STATE = 0x40           # command not accepted in this state
REJ_NONCE = 0x41            # LAUNCH / THROTTLE-from-ARMED nonce != telemetry nonce
REJ_ARM_SWITCH = 0x42       # ARM: arm switch open
REJ_SENSORS = 0x43          # ARM: IMU or baro not healthy
REJ_NOT_ALIGNED = 0x44      # ARM: pad alignment incomplete
REJ_TILT = 0x45             # ARM: tilt >= arm_tilt_max_deg
REJ_VCELL = 0x46            # ARM: V_cell < min_cell_v
REJ_LINK = 0x47             # ARM: no valid ground frame within link_loss_disarm_s before this one
REJ_UNKNOWN_CMD = 0x48      # cmd code not in the table (also 8..11, as in v0.6.3)
REJ_THROTTLE_ZERO = 0x49    # THROTTLE(u = 0) in ARMED (would enter UNPOWERED on the pad)
REJ_RELEASE_PENDING = 0x4A  # a release is pending: only KILL is accepted
REJ_RELEASED = 0x4B         # ARM after the release has opened (I10 boot, ABORT): power-cycle first
REASON_NAMES = {
    REASON_NONE: "none", R_I1_LINK_LOSS: "I1 link loss", R_I1P_LINK_LOSS_PAD: "I1p link loss on the pad",
    R_I2_TILT: "I2 tilt", R_I3_RATE: "I3 spin/gyro saturation", R_I4_APOGEE_LIMIT: "I4 apogee limiter",
    R_I5_GEOFENCE: "I5 geofence", R_I6_FALL: "I6 fall-safe", R_I7_MOTOR_TIME: "I7 motor time",
    R_I8_VCELL: "I8 V_cell", R_I9_IMU: "I9 IMU failure", R_I9_BARO: "I9 baro failure", R_I10_RESET: "I10 reset in flight",
    R_ARM_SWITCH_OPEN: "arm switch opened",
    REJ_STATE: "rejected: state", REJ_NONCE: "rejected: nonce",
    REJ_ARM_SWITCH: "rejected: arm switch open", REJ_SENSORS: "rejected: sensors", REJ_NOT_ALIGNED: "rejected: not aligned",
    REJ_TILT: "rejected: tilt", REJ_VCELL: "rejected: V_cell", REJ_LINK: "rejected: link not established",
    REJ_UNKNOWN_CMD: "rejected: unknown command", REJ_THROTTLE_ZERO: "rejected: THROTTLE 0 on the pad",
    REJ_RELEASE_PENDING: "rejected: release pending", REJ_RELEASED: "rejected: release already open",
}
ALL_INTERLOCK_REASONS = tuple(k for k in REASON_NAMES if 0 < k < 0x40)
ALL_REJECT_REASONS = tuple(k for k in REASON_NAMES if k >= 0x40)
BALLISTIC_INTERLOCK_REASONS = tuple(range(R_I1_LINK_LOSS, R_ARM_SWITCH_OPEN + 1))
BALLISTIC_REJECT_REASONS = tuple(range(REJ_STATE, REJ_RELEASED + 1))
INTERLOCK_REASONS = BALLISTIC_INTERLOCK_REASONS
REJECT_REASONS = BALLISTIC_REJECT_REASONS
MODE_REASONS = {MODE_BALLISTIC: (BALLISTIC_INTERLOCK_REASONS, BALLISTIC_REJECT_REASONS)}


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
    """Downlink TELEM payload (TELEM v1, 28 bytes), stored as the wire integers; physical values are properties."""

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

    def __post_init__(self) -> None:
        for n in ("state", "flags", "reason", "ack_seq"):
            _check_u(n, getattr(self, n), U8)
        for n in ("nonce", "tilt_cdeg", "throttle_milli", "vcell_mv", "loop_max_us"):
            _check_u(n, getattr(self, n), U16)
        _check_u("t_ms", self.t_ms, U32)
        for n in ("h_dm", "vz_cms", "roll_cdeg", "pitch_cdeg", "yaw_cdeg"):
            _check_u(n, getattr(self, n), I16)

    @classmethod
    def from_physical(cls, *, state: int, flags: int, reason: int, ack_seq: int, nonce: int, t_ms: int, h_m: float,
                      vz_mps: float, roll_deg: float, pitch_deg: float, yaw_deg: float, tilt_deg: float,
                      throttle: float, vcell_v: float, loop_max_us: float) -> "Telemetry":
        return cls(state=state, flags=flags, reason=reason, ack_seq=ack_seq, nonce=nonce, t_ms=t_ms & 0xFFFFFFFF,
                   h_dm=quantize(h_m, H_SCALE, I16), vz_cms=quantize(vz_mps, VZ_SCALE, I16),
                   roll_cdeg=quantize(roll_deg, ANGLE_SCALE, I16), pitch_cdeg=quantize(pitch_deg, ANGLE_SCALE, I16),
                   yaw_cdeg=quantize(yaw_deg, ANGLE_SCALE, I16), tilt_cdeg=quantize(tilt_deg, ANGLE_SCALE, U16),
                   throttle_milli=quantize(throttle, THROTTLE_SCALE, U16), vcell_mv=quantize(vcell_v, VCELL_SCALE, U16),
                   loop_max_us=quantize(loop_max_us, 1.0, U16))

    # wire format
    @property
    def wire_type(self) -> int:
        return TYPE_TELEM

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
        """The payload's 15 integers in wire order."""
        return (self.state, self.flags, self.reason, self.ack_seq, self.nonce, self.t_ms, self.h_dm, self.vz_cms,
                self.roll_cdeg, self.pitch_cdeg, self.yaw_cdeg, self.tilt_cdeg, self.throttle_milli, self.vcell_mv,
                self.loop_max_us)

    def pack(self) -> bytes:
        """The 28-byte TELEM v1 payload."""
        return struct.pack(TELEM_FMT, *self.wire_values())

    @classmethod
    def unpack(cls, payload: bytes) -> "Telemetry":
        """The 28-byte TELEM v1 payload; any other length is a ValueError."""
        if len(payload) == TELEM_PAYLOAD_LEN:
            return cls(*struct.unpack(TELEM_FMT, payload))
        raise ValueError(f"TELEM payload is {TELEM_PAYLOAD_LEN} bytes, got {len(payload)}")


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
    return encode_frame(veh, tlm.wire_type, seq, tlm.pack())


def decode_command(frame: Frame) -> Command:
    if frame.type != TYPE_CMD:
        raise ValueError(f"frame type 0x{frame.type:02X} is not CMD")
    return Command.unpack(frame.payload)


def decode_telemetry(frame: Frame) -> Telemetry:
    """TELEM frame -> Telemetry; the payload length must be its TYPE's."""
    if frame.type not in TELEM_TYPES:
        raise ValueError(f"frame type 0x{frame.type:02X} is not TELEM")
    if len(frame.payload) != PAYLOAD_LEN[frame.type]:
        raise ValueError(f"TYPE 0x{frame.type:02X} payload is {PAYLOAD_LEN[frame.type]} bytes, got {len(frame.payload)}")
    return Telemetry.unpack(frame.payload)


_HUNT0, _HUNT1, _LEN, _BODY = 0, 1, 2, 3
STAT_KEYS = ("frames", "bad_len", "bad_crc", "wrong_vehicle", "wrong_type", "bad_payload_len", "skipped_bytes")


@dataclass
class FrameDecoder:
    """Streaming frame parser with resynchronisation (algorithm in the module docstring).

    ``vehicle_id`` None accepts any vehicle; ``accept_types`` lists the TYPEs this end
    consumes (the vehicle: CMD, the ground: TELEM)."""

    vehicle_id: int | None = None
    accept_types: tuple[int, ...] = (TYPE_CMD, TYPE_TELEM)
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

    # every v0.4 command code, edge arguments
    for code, nm in CMD_NAMES.items():
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

    return {
        "_doc": ("LoRa link byte vectors (SPEC 10.3), generated by sim/link_proto.py make_test_vectors(); each vector is fed to a "
                 "fresh FrameDecoder(vehicle_id, accept_types); expected_frames are normative, see make_test_vectors() for the "
                 "counters. Telemetry fields are the wire integers (h 0.1 m, v_z cm/s, angles 0.01 deg, throttle x1000, mV, us; "
                 "LEN_MAX 40)."),
        "seed": seed, "vehicle_id": vid,
        "crc16_ccitt_false_check": {"ascii": "123456789", "crc": crc16_ccitt(b"123456789")},
        "constants": {"SYNC": [SYNC0, SYNC1], "TYPE_CMD": TYPE_CMD, "TYPE_TELEM": TYPE_TELEM, "LEN_MIN": LEN_MIN,
                      "LEN_MAX": LEN_MAX, "CMD_FRAME_BYTES": CMD_FRAME_BYTES, "TELEM_FRAME_BYTES": TELEM_FRAME_BYTES},
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
