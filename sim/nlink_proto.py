"""NAV <-> FCU navigation link (SPEC 10.5, v0.6.3) - the byte-exact reference.

v0.6.3 runs the flight computer on three ATmega128A (params.json ``flight_computer.architecture``):
the NAV reads the MPU-6050 and the MS5611 and runs the pad-alignment / gyro-only attitude estimator,
the tilt and the altitude filter; the FCU runs the onboard supervisor, the attitude controller, the
outputs and the log; the COM runs the radio (``sim.ilink_proto``).  This module is the normative
definition of the wire format between the NAV and the FCU; the firmware port is
``firmware/atmega128_fc/src/core/nlink_proto.c`` (``nlink_proto.h``, whose header comment is the
contract) and the SPI driver is ``src/hal/nlink_spi.c``.  Pure Python (no numpy), like
``sim.ilink_proto``.

Wire: SPI, NAV = master, FCU = slave, mode 0, MSB first, ``flight_computer.nlink.spi_clock_hz``
(2 MHz) with ``byte_gap_us`` between bytes.  Once per NAV tick the NAV pulls SS low and exchanges
exactly ``XFER_BYTES`` = 72 bytes full duplex:

    NAV -> FCU (MOSI):  0xD5 0x5D | TYPE_NAV | SEQ | PAYLOAD[66] | CRC16 lo | CRC16 hi
    FCU -> NAV (MISO):  0xD5 0x5D | TYPE_FCU | SEQ | state | fflags | params_tag | thr_q | CRC16 lo | hi
                        then 62 x 0x00 (only the first FCU_FRAME_BYTES = 10 bytes matter)

thr_q (v0.7.1, the byte that was "reserved, always 0" in v0.6.3 - the frame size is unchanged): the VTOL FCU's ESC
throttle command of its latest tick, quantised u8 = round_half_away(u x FCU_THR_SCALE), FCU_THR_SCALE = 250 (0.004 per
step, 0 when the motor is off), so the NAV can apply the baro thrust compensation (mission.vtol.baro_thrust_comp_m,
``sim.state_machine.VtolAltitudeFilter``) to each baro sample BEFORE its altitude filter.  Why on the NAV and in this
byte: the static-pressure error of the MS5611 follows the fan's thrust; corrected before the filter, it never reaches
the filter's states (h, v and the accelerometer-bias state would otherwise absorb every throttle change as a fake
climb or a fake bias), and the FCU would need a copy of the filter to undo it afterwards.  The NAV knows u only
through this byte (the last valid FCU frame: 2 control ticks old, the same lag as the state byte) - immaterial against
the EDF's 0.15 s thrust lag.  Ballistic build: the FCU writes 0 (bit-identical v0.6.3 exchange bytes), the ballistic
NAV ignores it.  ``fcu_throttle(thr_q)`` = min(thr_q, 250) / 250; quantisation error <= 0.002 in u, i.e. <= 0.5 % of the
hover thrust ratio.

CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF; ``sim.link_proto.crc16_ccitt``, the CRC of both other
links) over TYPE, SEQ and the payload.  Multi-byte fields little-endian, floats IEEE-754 binary32
little-endian (struct ``'<f'``).  NAV payload (``NAV_FMT``, 66 bytes): q[4], rate[3], accel[3],
h_m, vz_mps, h_baro_rel_m, tilt_deg (float32), imu_fail, baro_fails, twi_errors, loop_us_max
(uint16), flags, reset_cause (uint8).  reset_cause: bits 4..0 = the NAV's MCUCSR at boot
(``NAV_RC_MCUCSR_MASK``; PORF 0x01, EXTRF 0x02, BORF 0x04, WDRF 0x08, JTRF 0x10), bits 7..5 = the NAV's
boot / link results ``NAV_RC_IMU_INIT_FAIL`` 0x80 (MPU-6050 WHO_AM_I not recognised after 3 tries),
``NAV_RC_BARO_INIT_FAIL`` 0x40 (MS5611 reset / PROM / first D2 failed) and ``NAV_RC_TAG_MISMATCH`` 0x20
(the last valid FCU frame's params_tag differs from the NAV build's FC_PARAMS_TAG) - report only, the
FCU logs them and decides nothing on them (``nav_reset_cause_names``).

Decoding (``decode_nav`` / ``decode_fcu``, == ``nl_decode_nav`` / ``nl_decode_fcu``): the argument is
the whole 72-byte exchange buffer; a frame is valid iff bytes 0, 1 are the sync pair, byte 2 is the
expected TYPE and the CRC bytes equal the CRC over bytes 2 .. end of payload.  Nothing else is looked
at: bytes 10..71 of the FCU's half of the exchange are ignored, so a late slave interrupt that leaves
an echoed byte in bytes 10..71 costs nothing.  The thr_q byte (CRC-covered) is returned as decoded,
whatever its value (a value > 250 reads as u = 1, ``fcu_throttle``).
There is no streaming decoder: the SPI driver delivers whole exchanges, and the slave side keeps the
byte alignment with the rules of ``SpiSlaveModel`` (the normative model of ``hal/nlink_spi.c``'s FCU
slave).

Semantics (SPEC 10.5, ``nlink_proto.h``): ``fcu_nav_inputs`` is what the FCU hands the (unchanged)
supervisor - imu_ok = a valid frame && F_IMU_OK && (F_EST_VALID || not flying), baro_ok = a valid
frame && F_BARO_OK, aligned = F_ALIGNED, gyro_saturated = F_GYRO_SAT; no valid frame -> all false.
The NAV has two pad / flight switches, both from the state of the last valid FCU frame (before the
first valid FCU frame after a NAV boot: SAFE): ``nav_alt_pad_mode`` - the altitude filter held at the
pad and the ground reference low-pass filtered while that state is SAFE or ARMED, whatever the fflags;
``nav_pad_mode`` - the attitude estimator's pad alignment (and the pad-still counter behind ALIGNED /
EST_VALID) while that state is SAFE or ARMED and FF_BENCH is clear.  FF_BENCH (the FCU's bench_vanes
build) only switches the estimator to gyro-only on the pad; the altitude stays held.  v0.7.1 VTOL NAV:
``nav_throttle`` (the last valid FCU frame's thr_q, 0 before the first) is the u of the baro thrust
compensation; the pad / flight switches are unchanged (after LANDED -> SAFE the NAV is in pad mode
again and re-aligns before the next ON).

nlink v3 (v0.8, the VTOL build with GPS position hold; architect + review v0.8 (3): the 72-byte exchange is KEPT -
no frame growth, the FCU's 0.15 ms frame-timeout margin stays): the NAV sends TYPE_NAV3 (0x03) instead of TYPE_NAV
with the 66-byte payload ``NAV3_FMT`` (little-endian, no padding):

    q[4] f32       the PUBLISHED attitude = the gyro prediction of this tick (sim.estimator: predict -> publish ->
                   correct; the correction lands in the next tick's prediction)
    rate[3] f32    gyro - bias (the bias of the previous tick's correction), rad/s
    accel_z f32    body-z specific force of this tick (the vertical controller's support signature, k_T learning)
    h_m, vz_mps f32  the altitude filter's output of the PREVIOUS tick (it runs after the push)
    cos_tilt f32   1 - 2 (qx^2 + qy^2) of the published q (float32 on the NAV) - the vertical controller's c and the
                   position guard's tilt, so the FCU stack takes 4 bytes instead of the quaternion (its stack input
                   shrinks; FCU stack free was 966 B < 1024 B)
    tilt_deg f32   acos(clamp(cos_tilt, -1, 1)) in degrees (the supervisor's I2 / ON / V7 gates, TELEM, log)
    accel_x, accel_y i16   body x / y specific force x 100 (0.01 m/s^2, saturating; log only - v0.6.3 sent f32)
    h_baro_rel_cm i16      baro altitude above the ground reference x 100 (saturating; log only - v0.6.3 f32)
    imu_fail, baro_fails, twi_errors u8   the v0.6.3 counters saturated at 255 (log only; v0.6.3 u16)
    loop_us_max u16, flags u8, reset_cause u8   unchanged meaning
    gps_slot[5]    the rotating GPS slot (below)

GPS slot: byte 0 = header (cycle & 0x1F) << 3 | j, bytes 1..4 = part j of the snapshot the NAV latched at its last
50 Hz step (``sim.gps_nav``: the step runs at NAV tick k with k mod 5 == 4, after the push), sent in the frames of the
next five ticks, j = (frame tick) mod 5:

    j = 0 STATUS   flags u8 (sim.gps_nav GF_*: FIX_OK, GATE_OK, HEAD_OK, FIX3D, CFG_OK, INIT, VER_OK, INCONS - bit 7
                   was PAD until the v0.8 physics-review fix: the GPS-IMU inconsistency flag now), numSV u8,
                   hAcc u8 (0.1 m, saturating 255), reset_count u8 (wraps; position + velocity resets)
    j = 1 OFFSET   cumulative reset offset x / y i16 cm (W frame: x north, y west)
    j = 2 POS      CG position x / y i16 cm
    j = 3 VEL      CG velocity x / y i16 cm/s
    j = 4 HEAD     cos / sin of the NAV's heading psi, i16 Q15 (x 32767)
    j = 5..7       no snapshot yet (before the NAV's first step after boot: header 0x07, payload 0)

``cycle`` = the NAV's step counter mod 32: the FCU (``GpsSlotRx``) takes a set only when slots 0..4 of ONE cycle
arrived (consecutive valid frames), completing at slot 4 - a lost frame costs that 20 ms cycle, never mixes two
snapshots.  The FCU frame is unchanged (thr_q etc.).  The ballistic build keeps TYPE_NAV and the v0.6.3 payload bit for
bit; a VTOL v0.8 FCU refuses a TYPE_NAV frame (``nav3_reject_reason``: 'type') and vice versa - a board flashed with
the other build is an I9 on the pad.

nlink v4 (v0.8.1 four-chip build, architect 2026-10-21 20:30, params flight_computer.architecture_v081: the GNS board runs
the GPS receiver, the GPS filter and the position controller; ``sim.glink_proto`` is the NAV <-> GNS link and holds the
normative per-tick timing).  The 72-byte exchange is kept again; both frame TYPEs are new, so a v0.8 (three-chip) board
and a v0.8.1 board refuse each other ('type' -> I9 on the pad):

    NAV -> FCU  TYPE_NAV4 (0x04): the v3 payload layout (``Nav3``, 66 bytes) unchanged except the slot: a GNS slot.
    FCU -> NAV  TYPE_FCU4 (0x05): 0xD5 0x5D | 0x05 | SEQ | state | fflags | params_tag | thr_q | pflags | kt_q lo hi |
                CRC lo hi (13 bytes, then 59 x 0x00).  pflags = phase (bits 2..0, sim.link_proto PHASE_*) | MOTOR_ON 0x08 |
                SAT_HI 0x10 (the vertical controller's sat == +1 this tick) | LOW_ALT 0x20 | GROUND 0x40 (low_alt and a
                touchdown signature this tick) | LIFTED 0x80 (the take-off check passed, or off); kt_q = k_T x 16384 (u16,
                Q14) - every FCU-side input of the position controller, of the FCU's latest tick (the NAV sees it 2
                control ticks old, like the state byte).  ``SpiSlaveModel(reply_bytes=FCU4_FRAME_BYTES)`` is the slave
                driver's model with the 13-byte reply.

GNS slot (``GnsSlotTx`` NAV side, ``GnsSlotRx`` FCU side): header (cycle & 0x1F) << 3 | i as in v3 (7 = empty); the five
parts of the GNS result of one 50 Hz cycle - the bytes of the glink RESULT payload, copied, not re-quantised:

    i = 0 TILT    tilt setpoint x / y i16 (rotation vector in the attitude controller's level frame, x 1e4 rad;
                  TILT_NONE = -32768 in x: the position loop is not engaged -> the FCU's builder inactive, target 0)
    i = 1 STATUS  flags u8 (sim.gps_nav GF_*), numSV u8, hAcc u8 (0.1 m), pc u8 = mode (bits 2..0) | HOLD 0x08 | requests
                  << 4 (sim.position_ctrl REQ_GPS_LOST / FENCE / IMPLAUSIBLE / AUTHORITY, latched on the GNS)
    i = 2 POS     x / y i16 cm (W): p - hold point when HOLD is set (the FCU's TELEM distance), else p
    i = 3 VEL     x / y i16 cm/s (W)
    i = 4 TRIM    the integrator's trim tilt x / y i16 x 1e4 rad (level frame) - the FCU's fallback target

Phase: the parts of the result of cycle m (STATE sampled at NAV tick 5 m + 4) ride in the frames of NAV ticks 5 m + 4 +
L + i, L = params flight_computer.glink.latency_ticks_ASSUMED (3): TILT first, so the FCU applies a tilt setpoint the
tick it arrives (no wait for the set; the frame CRC protects the part), the STATUS-dependent rules use the set completed
at part 4.  FCU staleness (``GnsSlotRx``): < 2 missed cycles hold the last TILT; >= 2 -> the newest set's TRIM (attitude
only with the wind trim); >= land_steps cycles (params gps_loss timeout + land_after) -> REQ_GPS_LOST from the FCU itself
-> V11 LAND.  The supervisor's V11-V14 act on the newest set's requests | that FCU-side GNS-loss request (``sim.onboard``
FcuStack gns mode); V15 stays the FCU's own take-off check.

Invariant: ``encode_nav`` / ``encode_fcu`` are the only way bytes are produced; ``Nav.unpack(pack())``
reproduces every float bit for bit (quiet NaN payloads included; a signalling NaN is quietened by
Python's float conversion - the C port copies the 4 bytes and keeps it, which is why the vectors
carry float bit patterns); the decoders never return a frame whose sync, TYPE or CRC is wrong.
"""
from __future__ import annotations

import json
import math
import os
import random
import struct
import sys
from dataclasses import dataclass, field
from typing import Iterable, Sequence

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import link_proto as lp
except ImportError:  # pragma: no cover
    import link_proto as lp  # type: ignore

_HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.normpath(os.path.join(_HERE, "..", "params.json"))
VECTORS_PATH = os.path.join(_HERE, "out", "nlink_vectors.json")

# --------------------------------------------------------------------------- #
# constants (== firmware/atmega128_fc/src/core/nlink_proto.h, checked by sim/test_nlink_proto.py)
# --------------------------------------------------------------------------- #
SYNC0 = 0xD5
SYNC1 = 0x5D
TYPE_NAV = 0x01
TYPE_FCU = 0x02
NAV_FMT = "<4f3f3fffffHHHHBB"
NAV_LEN = struct.calcsize(NAV_FMT)                    # 66
HEADER_BYTES = 4                                      # sync x2, TYPE, SEQ
XFER_BYTES = HEADER_BYTES + NAV_LEN + 2               # 72
FCU_FMT = "<BBBB"                                     # state, fflags, params_tag, thr_q (v0.6.3: reserved 0)
FCU_LEN = struct.calcsize(FCU_FMT)                    # 4
FCU_THR_SCALE = 250                                   # thr_q = round_half_away(u x 250), 0..250 (v0.7.1 VTOL)
FCU_FRAME_BYTES = HEADER_BYTES + FCU_LEN + 2          # 10
CRC_INIT = lp.CRC_INIT

F_IMU_OK = 0x01           # this tick's MPU-6050 burst read succeeded
F_BARO_OK = 0x02          # MS5611 samples on schedule
F_ALIGNED = 0x04          # pad alignment complete
F_GYRO_SAT = 0x08         # MPU-6050 gyro at its int16 limit this tick
F_EST_VALID = 0x10        # the attitude came from a pad alignment completed since this NAV boot
F_BARO_NEW = 0x20         # a new baro sample was used by the altitude filter this tick
F_BOOT = 0x40             # set until the NAV has received a valid FCU frame since its reset
FF_BENCH = 0x01           # bench_vanes mode: the estimator stays gyro-only on the pad (the altitude is still held)
FF_RECOVERY = 0x02        # the FCU booted by I10 (in-flight reset)
# Nav.reset_cause (== firmware/atmega128_fc/src/nav/nav.h NAV_RC_*, checked by sim/test_nlink_proto.py)
NAV_RC_MCUCSR_MASK = 0x1F     # bits 4..0: the NAV's MCUCSR at boot (RESET_* of hal/watchdog.h)
NAV_RC_TAG_MISMATCH = 0x20    # the last valid FCU frame carried a params_tag != the NAV build's FC_PARAMS_TAG
NAV_RC_BARO_INIT_FAIL = 0x40  # MS5611 reset / PROM read / PROM CRC / first D2 failed at boot
NAV_RC_IMU_INIT_FAIL = 0x80   # MPU-6050 WHO_AM_I not recognised after 3 tries at boot
MCUCSR_RESET_NAMES = {0x01: "power_on", 0x02: "external", 0x04: "brown_out", 0x08: "watchdog", 0x10: "jtag"}
NAV_RC_NAMES = {NAV_RC_TAG_MISMATCH: "tag_mismatch", NAV_RC_BARO_INIT_FAIL: "baro_init_fail",
                NAV_RC_IMU_INIT_FAIL: "imu_init_fail"}

TYPE_NAV3 = 0x03          # v0.8 VTOL + GPS: nlink v3 payload (module docstring)
NAV3_FMT = "<12fhhhBBBHBB5s"
NAV3_LEN = struct.calcsize(NAV3_FMT)                  # 66 = NAV_LEN: the exchange stays 72 bytes
SLOT_LEN = 5
SLOT_PARTS = 5            # STATUS, OFFSET, POS, VEL, HEAD
SLOT_STATUS, SLOT_OFFSET, SLOT_POS, SLOT_VEL, SLOT_HEAD = range(5)
SLOT_EMPTY = bytes([0x07, 0, 0, 0, 0])                # no snapshot yet
ACCEL_XY_SCALE = 100.0    # 0.01 m/s^2
H_BARO_SCALE = 100.0      # cm
NAV_FLOAT_FIELDS = ("q", "rate", "accel", "h_m", "vz_mps", "h_baro_rel_m", "tilt_deg")   # 4 + 3 + 3 + 1 x 4 = 14 floats
NAV_INT_FIELDS = ("imu_fail", "baro_fails", "twi_errors", "loop_us_max", "flags", "reset_cause")
NAV_N_FLOATS = 14
FCU_FIELDS = ("seq", "state", "fflags", "params_tag", "thr_q")

U8, U16 = lp.U8, lp.U16


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def crc16(data: Iterable[int], crc: int = CRC_INIT) -> int:
    """CRC-16/CCITT-FALSE - ``sim.link_proto.crc16_ccitt`` (one CRC for all three links)."""
    return lp.crc16_ccitt(data, crc)


def f32(x: float) -> float:
    """The float32 value C's ``(float)x`` gives (round to nearest; beyond FLT_MAX -> +-inf)."""
    try:
        return struct.unpack("<f", struct.pack("<f", x))[0]
    except OverflowError:
        return math.copysign(math.inf, x)


def f32_bits(x: float) -> int:
    return struct.unpack("<I", struct.pack("<f", f32(x)))[0]


def f32_from_bits(u: int) -> float:
    return struct.unpack("<f", struct.pack("<I", u & 0xFFFFFFFF))[0]


def _check(name: str, v: int, lim: tuple[int, int]) -> int:
    if not isinstance(v, int) or isinstance(v, bool) or not (lim[0] <= v <= lim[1]):
        raise ValueError(f"{name} = {v!r} outside [{lim[0]}, {lim[1]}]")
    return v


# --------------------------------------------------------------------------- #
# payloads
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Nav:
    """nl_nav_t: one NAV tick's navigation state.  Float fields hold the float32 values the wire carries
    (``pack`` rounds to float32; compare by ``pack()`` - NaN != NaN)."""

    seq: int
    q: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    rate: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    h_m: float = 0.0
    vz_mps: float = 0.0
    h_baro_rel_m: float = 0.0
    tilt_deg: float = 0.0
    imu_fail: int = 0
    baro_fails: int = 0
    twi_errors: int = 0
    loop_us_max: int = 0
    flags: int = 0
    reset_cause: int = 0

    def __post_init__(self) -> None:
        _check("seq", self.seq, U8)
        for n in ("imu_fail", "baro_fails", "twi_errors", "loop_us_max"):
            _check(n, getattr(self, n), U16)
        _check("flags", self.flags, U8)
        _check("reset_cause", self.reset_cause, U8)
        if len(self.q) != 4 or len(self.rate) != 3 or len(self.accel) != 3:
            raise ValueError("q has 4 components, rate and accel 3")

    def floats(self) -> list[float]:
        """The 14 float fields in wire order."""
        return [*self.q, *self.rate, *self.accel, self.h_m, self.vz_mps, self.h_baro_rel_m, self.tilt_deg]

    def ints(self) -> list[int]:
        return [getattr(self, n) for n in NAV_INT_FIELDS]

    def pack(self) -> bytes:
        """The 66-byte payload (SEQ travels in the header)."""
        return struct.pack(NAV_FMT, *(f32(x) for x in self.floats()), *self.ints())

    @classmethod
    def unpack(cls, seq: int, payload: bytes) -> "Nav":
        if len(payload) != NAV_LEN:
            raise ValueError(f"NAV payload is {NAV_LEN} bytes, got {len(payload)}")
        v = struct.unpack(NAV_FMT, payload)
        return cls(seq, tuple(v[0:4]), tuple(v[4:7]), tuple(v[7:10]), v[10], v[11], v[12], v[13], *v[14:20])


def nav_payload_from_bits(float_bits: Sequence[int], ints: Sequence[int]) -> bytes:
    """NAV payload from 14 raw float bit patterns (wire order) and the 6 integer fields - every float class,
    signalling NaN included, exactly as the C struct would hold it."""
    if len(float_bits) != NAV_N_FLOATS or len(ints) != len(NAV_INT_FIELDS):
        raise ValueError("14 float bit patterns and 6 integers")
    return struct.pack("<14I4HBB", *(b & 0xFFFFFFFF for b in float_bits), *ints)


def nav_fields(seq: int, payload: bytes) -> dict:
    """Payload fields in struct order; floats as uint32 bit patterns (exact for every pattern, sNaN included)."""
    u = struct.unpack("<14I4HBB", payload)
    return {"seq": seq, "float_bits": list(u[:14]), "ints": dict(zip(NAV_INT_FIELDS, u[14:]))}


@dataclass(frozen=True)
class Fcu:
    """nl_fcu_t: the FCU's state bytes returned in the same exchange."""

    seq: int                       # SEQ of the last NAV frame the FCU accepted
    state: int                     # supervisor state after the FCU's latest tick (LP_STATE_*)
    fflags: int = 0                # FF_*
    params_tag: int = 0            # FC_PARAMS_TAG of the FCU build
    thr_q: int = 0                 # v0.7.1 VTOL: ESC throttle of the FCU's latest tick x FCU_THR_SCALE (ballistic: 0)

    def __post_init__(self) -> None:
        for n in FCU_FIELDS:
            _check(n, getattr(self, n), U8)

    @property
    def throttle(self) -> float:
        """The throttle the NAV uses for the baro thrust compensation (``fcu_throttle``)."""
        return fcu_throttle(self.thr_q)


def fcu_throttle_q(u: float) -> int:
    """FCU side: ESC throttle u (0..1) -> thr_q = round_half_away(u x 250), saturated to 0..250, NaN -> 0
    (``sim.link_proto.quantize``; C: ``lp_quantize(u, 250, 0, 250)``)."""
    return lp.quantize(float(u), FCU_THR_SCALE, (0, FCU_THR_SCALE))


def fcu_throttle(thr_q: int) -> float:
    """NAV side: thr_q -> u = min(thr_q, 250) / 250 (a value above 250 cannot come from fcu_throttle_q; it reads as full
    throttle, the larger baro correction - the CRC has already accepted the byte)."""
    return min(int(thr_q), FCU_THR_SCALE) / FCU_THR_SCALE


# --------------------------------------------------------------------------- #
# frames
# --------------------------------------------------------------------------- #
def _frame(typ: int, seq: int, payload: bytes) -> bytes:
    body = bytes([typ, seq]) + bytes(payload)
    crc = crc16(body)
    return bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8])


def encode_nav_payload(seq: int, payload: bytes) -> bytes:
    """The 72-byte NAV exchange buffer from SEQ and a raw 66-byte payload (nl_encode_nav)."""
    _check("seq", seq, U8)
    if len(payload) != NAV_LEN:
        raise ValueError(f"NAV payload is {NAV_LEN} bytes, got {len(payload)}")
    return _frame(TYPE_NAV, seq, payload)


def encode_nav(n: Nav) -> bytes:
    return encode_nav_payload(n.seq, n.pack())


def encode_fcu(f: Fcu, reserved: int | None = None) -> bytes:
    """The FCU's 72-byte half of the exchange (nl_encode_fcu): the 10-byte frame, then 62 x 0x00.  Byte 7 is ``f.thr_q``
    (v0.7.1; the ballistic FCU writes 0); ``reserved`` (v0.6.3 test-frame argument) overrides it when given."""
    b7 = f.thr_q if reserved is None else _check("reserved", reserved, U8)
    fr = _frame(TYPE_FCU, f.seq, bytes([f.state, f.fflags, f.params_tag, b7]))
    return fr + bytes(XFER_BYTES - FCU_FRAME_BYTES)


def _check_len(buf: bytes | bytearray) -> bytes:
    b = bytes(buf)
    if len(b) != XFER_BYTES:
        raise ValueError(f"an nlink exchange is {XFER_BYTES} bytes, got {len(b)}")
    return b


def nav_reject_reason(buf: bytes | bytearray) -> str | None:
    """Why ``decode_nav`` refuses a buffer ('sync', 'type', 'crc'), None if it is valid."""
    b = _check_len(buf)
    if b[0] != SYNC0 or b[1] != SYNC1:
        return "sync"
    if b[2] != TYPE_NAV:
        return "type"
    end = HEADER_BYTES + NAV_LEN
    crc = crc16(b[2:end])
    return None if (b[end] | (b[end + 1] << 8)) == crc else "crc"


def fcu_reject_reason(buf: bytes | bytearray) -> str | None:
    """Why ``decode_fcu`` refuses a buffer ('sync', 'type', 'crc'), None if it is valid (bytes 10..71 ignored)."""
    b = _check_len(buf)
    if b[0] != SYNC0 or b[1] != SYNC1:
        return "sync"
    if b[2] != TYPE_FCU:
        return "type"
    end = HEADER_BYTES + FCU_LEN
    crc = crc16(b[2:end])
    return None if (b[end] | (b[end + 1] << 8)) == crc else "crc"


def decode_nav_raw(buf: bytes | bytearray) -> tuple[int, bytes] | None:
    """(SEQ, 66-byte payload) of a valid NAV exchange buffer, None otherwise."""
    if nav_reject_reason(buf) is not None:
        return None
    b = bytes(buf)
    return b[3], b[HEADER_BYTES:HEADER_BYTES + NAV_LEN]


def decode_nav(buf: bytes | bytearray) -> Nav | None:
    """nl_decode_nav: the navigation frame of a valid exchange buffer, None otherwise."""
    r = decode_nav_raw(buf)
    return None if r is None else Nav.unpack(*r)


def decode_fcu(buf: bytes | bytearray) -> Fcu | None:
    """nl_decode_fcu: the FCU frame of a valid exchange buffer (bytes 10..71 ignored; byte 7 = thr_q)."""
    if fcu_reject_reason(buf) is not None:
        return None
    b = bytes(buf)
    return Fcu(b[3], b[4], b[5], b[6], b[7])


# --------------------------------------------------------------------------- #
# semantics (SPEC 10.5): what each side does with a frame
# --------------------------------------------------------------------------- #
FLYING_STATES = frozenset((lp.STATE_POWERED, lp.STATE_UNPOWERED, lp.STATE_RELEASED, lp.STATE_ABORT))   # onboard_flying


@dataclass(frozen=True)
class FcuNavInputs:
    """The sensor booleans the FCU hands its supervisor for one tick."""

    imu_ok: bool
    baro_ok: bool
    aligned: bool
    gyro_saturated: bool


def fcu_nav_inputs(nav: Nav | None, state: int) -> FcuNavInputs:
    """FCU side: ``nav`` = the valid NAV frame that started this tick (None: timeout, sync, TYPE or CRC failure);
    ``state`` = the supervisor state before the tick (onboard_flying decides whether EST_VALID is required).
    A NAV that reset in flight has no EST_VALID -> imu_ok false -> I9 -> ABORT."""
    if nav is None:
        return FcuNavInputs(False, False, False, False)
    fl = nav.flags
    flying = state in FLYING_STATES
    return FcuNavInputs(imu_ok=bool(fl & F_IMU_OK) and (bool(fl & F_EST_VALID) or not flying),
                        baro_ok=bool(fl & F_BARO_OK), aligned=bool(fl & F_ALIGNED), gyro_saturated=bool(fl & F_GYRO_SAT))


def nav_alt_pad_mode(last_fcu: Fcu | None) -> bool:
    """NAV side, altitude filter (``pad_alt`` of nav_main.c): held at the pad (h, v_z reset to 0 every tick before that
    tick's baro correction) and the ground reference low-pass filtered while the last valid FCU frame says SAFE or ARMED,
    whatever its fflags (FF_BENCH included); before the first valid FCU frame after a NAV boot: pad (as SAFE)."""
    if last_fcu is None:
        return True
    return last_fcu.state in (lp.STATE_SAFE, lp.STATE_ARMED)


def nav_pad_mode(last_fcu: Fcu | None) -> bool:
    """NAV side, attitude estimator (``on_pad`` of nav_main.c): pad alignment, the pad-still counter and ALIGNED /
    EST_VALID while the last valid FCU frame says SAFE or ARMED without FF_BENCH; gyro-only otherwise; before the first
    valid FCU frame after a NAV boot: pad (as SAFE).  FF_BENCH switches only this one - the altitude filter follows
    ``nav_alt_pad_mode``."""
    return nav_alt_pad_mode(last_fcu) and not (last_fcu is not None and last_fcu.fflags & FF_BENCH)


def nav_throttle(last_fcu: Fcu | None) -> float:
    """NAV side (v0.7.1 VTOL): the ESC throttle for the baro thrust compensation = ``fcu_throttle`` of the last valid FCU
    frame's thr_q; 0 before the first valid FCU frame after a NAV boot (the motor cannot run before the FCU has
    answered: SAFE until then).  An invalid frame keeps the last valid one, like the state byte."""
    return 0.0 if last_fcu is None else fcu_throttle(last_fcu.thr_q)


def nav_reset_cause_names(reset_cause: int) -> tuple[list[str], list[str]]:
    """(MCUCSR reset flags of bits 4..0, NAV_RC_* results of bits 7..5) of a Nav.reset_cause, by name."""
    _check("reset_cause", reset_cause, U8)
    return ([n for b, n in MCUCSR_RESET_NAMES.items() if reset_cause & b],
            [n for b, n in NAV_RC_NAMES.items() if reset_cause & b])


# --------------------------------------------------------------------------- #
# nlink v3 (v0.8 VTOL + GPS): payload, GPS slot
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Nav3:
    """nl_nav3_t (module docstring "nlink v3").  Floats hold float32 values; the integer fields are the wire integers."""

    seq: int
    q: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    rate: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_z: float = 0.0
    h_m: float = 0.0
    vz_mps: float = 0.0
    cos_tilt: float = 1.0
    tilt_deg: float = 0.0
    accel_x_q: int = 0             # 0.01 m/s^2
    accel_y_q: int = 0
    h_baro_rel_cm: int = 0
    imu_fail: int = 0              # saturated at 255
    baro_fails: int = 0
    twi_errors: int = 0
    loop_us_max: int = 0
    flags: int = 0
    reset_cause: int = 0
    slot: bytes = SLOT_EMPTY

    def __post_init__(self) -> None:
        _check("seq", self.seq, U8)
        for n in ("accel_x_q", "accel_y_q", "h_baro_rel_cm"):
            _check(n, getattr(self, n), lp.I16)
        for n in ("imu_fail", "baro_fails", "twi_errors", "flags", "reset_cause"):
            _check(n, getattr(self, n), U8)
        _check("loop_us_max", self.loop_us_max, U16)
        if len(self.q) != 4 or len(self.rate) != 3 or len(self.slot) != SLOT_LEN:
            raise ValueError("q has 4 components, rate 3, slot 5 bytes")

    @classmethod
    def from_physical(cls, seq: int, q, rate, accel, h_m: float, vz_mps: float, h_baro_rel_m: float, imu_fail: int,
                      baro_fails: int, twi_errors: int, loop_us_max: int, flags: int, reset_cause: int,
                      slot: bytes = SLOT_EMPTY) -> "Nav3":
        """The NAV's encoding: cos_tilt and tilt from the published q (``nav3_tilt``), accel x / y and h_baro quantised,
        the counters saturated."""
        c, t = nav3_tilt(q)
        return cls(seq, tuple(f32(x) for x in q), tuple(f32(x) for x in rate), f32(accel[2]), f32(h_m), f32(vz_mps),
                   c, t, lp.quantize(float(accel[0]), ACCEL_XY_SCALE, lp.I16),
                   lp.quantize(float(accel[1]), ACCEL_XY_SCALE, lp.I16), lp.quantize(float(h_baro_rel_m), H_BARO_SCALE,
                                                                                    lp.I16),
                   min(int(imu_fail), 255), min(int(baro_fails), 255), min(int(twi_errors), 255),
                   min(int(loop_us_max), 0xFFFF), int(flags), int(reset_cause), bytes(slot))

    @property
    def accel(self) -> tuple[float, float, float]:
        return self.accel_x_q / ACCEL_XY_SCALE, self.accel_y_q / ACCEL_XY_SCALE, self.accel_z

    def floats(self) -> list[float]:
        return [*self.q, *self.rate, self.accel_z, self.h_m, self.vz_mps, self.cos_tilt, self.tilt_deg]

    def pack(self) -> bytes:
        return struct.pack(NAV3_FMT, *(f32(x) for x in self.floats()), self.accel_x_q, self.accel_y_q,
                           self.h_baro_rel_cm, self.imu_fail, self.baro_fails, self.twi_errors, self.loop_us_max,
                           self.flags, self.reset_cause, bytes(self.slot))

    @classmethod
    def unpack(cls, seq: int, payload: bytes) -> "Nav3":
        if len(payload) != NAV3_LEN:
            raise ValueError(f"NAV3 payload is {NAV3_LEN} bytes, got {len(payload)}")
        v = struct.unpack(NAV3_FMT, payload)
        return cls(seq, tuple(v[0:4]), tuple(v[4:7]), v[7], v[8], v[9], v[10], v[11], *v[12:21], bytes(v[21]))


def nav3_tilt(q) -> tuple[float, float]:
    """(cos_tilt, tilt_deg) the NAV puts in a v3 frame, from the published q in float32 steps: x2 = f32(x x), y2 =
    f32(y y), c = f32(1 - f32(2 f32(x2 + y2))); tilt = f32(degrees(acos(clamp(c, -1, 1)))) (C: acosf x FC_RAD2DEG_F -
    the last bit may differ, the FCU compares tilt_deg with a threshold only)."""
    x, y = f32(q[1]), f32(q[2])
    c = f32(1.0 - f32(2.0 * f32(f32(x * x) + f32(y * y))))
    t = f32(math.degrees(math.acos(min(max(c, -1.0), 1.0))))
    return c, t


def encode_nav3(n: Nav3) -> bytes:
    """The 72-byte v3 NAV exchange buffer (nl_encode_nav3)."""
    return _frame(TYPE_NAV3, n.seq, n.pack())


def nav3_reject_reason(buf: bytes | bytearray) -> str | None:
    """Why ``decode_nav3`` refuses a buffer ('sync', 'type', 'crc'), None if it is valid."""
    b = _check_len(buf)
    if b[0] != SYNC0 or b[1] != SYNC1:
        return "sync"
    if b[2] != TYPE_NAV3:
        return "type"
    end = HEADER_BYTES + NAV3_LEN
    crc = crc16(b[2:end])
    return None if (b[end] | (b[end + 1] << 8)) == crc else "crc"


def decode_nav3(buf: bytes | bytearray) -> Nav3 | None:
    """nl_decode_nav3: the v3 navigation frame of a valid exchange buffer, None otherwise."""
    if nav3_reject_reason(buf) is not None:
        return None
    b = bytes(buf)
    return Nav3.unpack(b[3], b[HEADER_BYTES:HEADER_BYTES + NAV3_LEN])


def fcu_nav3_inputs(nav: Nav3 | None, state: int) -> FcuNavInputs:
    """The v0.6.3 sensor-boolean rule (``fcu_nav_inputs``) on a v3 frame."""
    if nav is None:
        return FcuNavInputs(False, False, False, False)
    fl = nav.flags
    flying = state in FLYING_STATES
    return FcuNavInputs(imu_ok=bool(fl & F_IMU_OK) and (bool(fl & F_EST_VALID) or not flying),
                        baro_ok=bool(fl & F_BARO_OK), aligned=bool(fl & F_ALIGNED), gyro_saturated=bool(fl & F_GYRO_SAT))


def slot_part(snap, j: int) -> bytes:
    """The 4 payload bytes of part j of a ``sim.gps_nav.GpsSnapshot``."""
    if j == SLOT_STATUS:
        return bytes([snap.flags & 0xFF, min(int(snap.num_sv), 255), min(int(snap.hacc_dm), 255),
                      snap.reset_count & 0xFF])
    if j == SLOT_OFFSET:
        return struct.pack("<hh", snap.off_x_cm, snap.off_y_cm)
    if j == SLOT_POS:
        return struct.pack("<hh", snap.px_cm, snap.py_cm)
    if j == SLOT_VEL:
        return struct.pack("<hh", snap.vx_cms, snap.vy_cms)
    if j == SLOT_HEAD:
        return struct.pack("<hh", snap.cos_q15, snap.sin_q15)
    raise ValueError(f"slot part {j}")


def encode_slot(snap, cycle: int, j: int) -> bytes:
    """Slot bytes: header (cycle & 0x1F) << 3 | j, then part j (j 0..4); ``snap`` None -> SLOT_EMPTY."""
    if snap is None:
        return SLOT_EMPTY
    return bytes([((cycle & 0x1F) << 3) | j]) + slot_part(snap, j)


class GpsSlotTx:
    """NAV side: the slot of the frame of NAV tick k = part (k mod 5) of the snapshot latched at the last step.  Call
    ``latch`` right after the filter's step (tick k mod 5 == 4, correction part) and ``slot(k)`` when building the
    frame of tick k (before the correction part)."""

    def __init__(self, step_ticks: int = 5):
        self.n = int(step_ticks)
        self.snap = None
        self.cycle = 0

    def latch(self, snap, cycle: int) -> None:
        self.snap, self.cycle = snap, int(cycle)

    def slot(self, k: int) -> bytes:
        return encode_slot(self.snap, self.cycle, k % self.n)


def decode_slot(slot: bytes) -> tuple[int, int, bytes] | None:
    """(cycle, j, payload) of a slot, None when j > 4 (no snapshot)."""
    if len(slot) != SLOT_LEN:
        raise ValueError("a GPS slot is 5 bytes")
    j = slot[0] & 0x07
    if j >= SLOT_PARTS:
        return None
    return slot[0] >> 3, j, bytes(slot[1:5])


class GpsSlotRx:
    """FCU side (module docstring): assembles the five parts of one cycle; ``feed`` once per FCU tick with the slot of
    this tick's valid NAV frame (None: no valid frame) returns the complete ``sim.position_ctrl.GpsData`` when this
    tick's slot was part 4 and parts 0..4 of the same cycle arrived in the last five ticks, else None.  ``gate_ok`` =
    GATE_OK of the newest complete set, if it completed within ``STALE_TICKS``."""

    STALE_TICKS = 10                                    # two cycles (a single lost frame keeps the gate)

    def __init__(self) -> None:
        self.parts: list[tuple[int, int, bytes] | None] = [None] * SLOT_PARTS   # (tick, cycle, payload)
        self.tick = -1
        self.latest = None
        self.t_latest: int | None = None
        self.got_part4 = False
        self.sets = 0
        self.broken = 0

    def feed(self, slot: bytes | None):
        try:
            from sim.position_ctrl import GpsData
        except ImportError:  # pragma: no cover
            from position_ctrl import GpsData  # type: ignore
        self.tick += 1
        self.got_part4 = False
        if slot is None:
            return None
        d = decode_slot(slot)
        if d is None:
            return None
        cyc, j, pl = d
        self.parts[j] = (self.tick, cyc, pl)
        if j != SLOT_PARTS - 1:
            return None
        self.got_part4 = True
        ps = self.parts
        if not all(x is not None and x[1] == cyc and self.tick - x[0] == SLOT_PARTS - 1 - i for i, x in enumerate(ps)):
            self.broken += 1
            return None
        st = ps[SLOT_STATUS][2]
        ox, oy = struct.unpack("<hh", ps[SLOT_OFFSET][2])
        px, py = struct.unpack("<hh", ps[SLOT_POS][2])
        vx, vy = struct.unpack("<hh", ps[SLOT_VEL][2])
        c, s = struct.unpack("<hh", ps[SLOT_HEAD][2])
        g = GpsData(flags=st[0], num_sv=st[1], hacc_dm=st[2], reset_count=st[3], off=(ox / 100.0, oy / 100.0),
                    p=(px / 100.0, py / 100.0), v=(vx / 100.0, vy / 100.0), cos_psi=c / 32767.0, sin_psi=s / 32767.0,
                    cycle=cyc)
        self.latest, self.t_latest = g, self.tick
        self.sets += 1
        return g

    def fresh(self) -> bool:
        return self.t_latest is not None and self.tick - self.t_latest <= self.STALE_TICKS

    @property
    def gate_ok(self) -> bool:
        return bool(self.fresh() and self.latest.flags & 0x02)


# --------------------------------------------------------------------------- #
# nlink v4 (v0.8.1 four-chip build: the GNS board runs the GPS filter and the position controller)
# --------------------------------------------------------------------------- #
TYPE_NAV4 = 0x04          # v0.8.1: the v3 payload layout (NAV3_FMT, 66 B) with the GNS slot table (module docstring v4)
TYPE_FCU4 = 0x05          # v0.8.1: the FCU frame + pflags + k_T (the GNS position controller's FCU inputs)
FCU4_FMT = "<BBBBBH"      # state, fflags, params_tag, thr_q, pflags, kt_q
FCU4_LEN = struct.calcsize(FCU4_FMT)                  # 7
FCU4_FRAME_BYTES = HEADER_BYTES + FCU4_LEN + 2        # 13 (the FCU's half of the 72-byte exchange: then 59 x 0x00)
FCU4_FIELDS = ("seq", "state", "fflags", "params_tag", "thr_q", "pflags")
PF_PHASE_MASK = 0x07      # pflags bits 2..0: the supervisor phase after the FCU's latest tick (sim.link_proto PHASE_*)
PF_MOTOR_ON = 0x08        # state POWERED (the ESC follows the vertical controller)
PF_SAT_HI = 0x10          # the vertical controller's sat == +1 in the FCU's latest tick (the NAV ORs it over the cycle)
PF_LOW_ALT = 0x20         # the vertical controller's flare band (LAND and h <= flare_altitude_m)
PF_GROUND = 0x40          # low_alt and a touchdown signature (still / support / floor) this tick
PF_LIFTED = 0x80          # the take-off check passed (or the check is off)
KT_SCALE = 16384          # kt_q = round_half_away(k_T x 16384), 0..65535 (Q14; k_T in [kt_min, kt_max] = [0.7, 1.4])
SLOT4_TILT, SLOT4_STATUS, SLOT4_POS, SLOT4_VEL, SLOT4_TRIM = range(5)
SLOT4_NAMES = ("TILT", "STATUS", "POS", "VEL", "TRIM")
TILT_SCALE = 10000.0      # tilt setpoint / trim: rotation-vector components x 1e4 (1e-4 rad = 0.0057 deg per LSB)
TILT_NONE = -0x8000       # TILT x = -32768: the position loop is not engaged (target 0, the builder inactive); the
                          # saturating quantiser never produces it (lp.quantize clamps to -32768 .. 32767 -> see tilt_q)
PC_MODE_MASK = 0x07       # STATUS byte 3: position-controller mode (sim.position_ctrl MODE_*) ...
PC_HOLD = 0x08            # ... the hold point is captured (POS = p - hold, else POS = p) ...
PC_REQ_SHIFT = 4          # ... its latched LAND requests (sim.position_ctrl REQ_* << 4)


@dataclass(frozen=True)
class Fcu4:
    """nl_fcu4_t: the v0.8.1 FCU frame (TYPE_FCU4).  The first five fields are the v0.6.3 / v0.7.1 ones (the NAV's pad /
    flight switches and the baro thrust compensation read them as before); ``pflags`` / ``kt_q`` carry the vertical
    controller's state the GNS position controller needs (module docstring "nlink v4")."""

    seq: int
    state: int
    fflags: int = 0
    params_tag: int = 0
    thr_q: int = 0
    pflags: int = 0
    kt_q: int = KT_SCALE

    def __post_init__(self) -> None:
        for n in FCU4_FIELDS:
            _check(n, getattr(self, n), U8)
        _check("kt_q", self.kt_q, U16)

    @property
    def throttle(self) -> float:
        return fcu_throttle(self.thr_q)

    @property
    def phase(self) -> int:
        return self.pflags & PF_PHASE_MASK

    @property
    def k_t(self) -> float:
        return kt_value(self.kt_q)


def kt_quant(k_t: float) -> int:
    """FCU side: the learnt thrust ratio -> kt_q = round_half_away(k_T x 16384) saturated to 0..65535, NaN -> 0."""
    return lp.quantize(float(k_t), float(KT_SCALE), U16)


def kt_value(kt_q: int) -> float:
    return int(kt_q) / KT_SCALE


def fcu4_pflags(phase: int, motor_on: bool, sat_hi: bool, low_alt: bool, ground: bool, lifted: bool) -> int:
    """FCU side: the pflags byte of the FCU's latest tick (``sim.onboard.FcuStack`` gns mode)."""
    if not 0 <= int(phase) <= PF_PHASE_MASK:
        raise ValueError(f"phase {phase} does not fit 3 bits")
    return (int(phase) | (PF_MOTOR_ON if motor_on else 0) | (PF_SAT_HI if sat_hi else 0) | (PF_LOW_ALT if low_alt else 0)
            | (PF_GROUND if ground else 0) | (PF_LIFTED if lifted else 0))


def encode_fcu4(f: Fcu4) -> bytes:
    """The FCU's 72-byte half of a v4 exchange (nl_encode_fcu4): the 13-byte frame, then 59 x 0x00."""
    fr = _frame(TYPE_FCU4, f.seq, struct.pack(FCU4_FMT, f.state, f.fflags, f.params_tag, f.thr_q, f.pflags, f.kt_q))
    return fr + bytes(XFER_BYTES - FCU4_FRAME_BYTES)


def fcu4_reject_reason(buf: bytes | bytearray) -> str | None:
    """Why ``decode_fcu4`` refuses a buffer ('sync', 'type', 'crc'), None if it is valid (bytes 13..71 ignored)."""
    b = _check_len(buf)
    if b[0] != SYNC0 or b[1] != SYNC1:
        return "sync"
    if b[2] != TYPE_FCU4:
        return "type"
    end = HEADER_BYTES + FCU4_LEN
    crc = crc16(b[2:end])
    return None if (b[end] | (b[end + 1] << 8)) == crc else "crc"


def decode_fcu4(buf: bytes | bytearray) -> Fcu4 | None:
    """nl_decode_fcu4: the v4 FCU frame of a valid exchange buffer, None otherwise."""
    if fcu4_reject_reason(buf) is not None:
        return None
    b = bytes(buf)
    st, ff, tag, thr, pf, kt = struct.unpack(FCU4_FMT, b[HEADER_BYTES:HEADER_BYTES + FCU4_LEN])
    return Fcu4(b[3], st, ff, tag, thr, pf, kt)


def encode_nav4(n: Nav3) -> bytes:
    """The 72-byte v4 NAV exchange buffer (nl_encode_nav4): the v3 payload (``Nav3``) under TYPE_NAV4; its slot is a
    GNS slot (``GnsSlotTx``)."""
    return _frame(TYPE_NAV4, n.seq, n.pack())


def nav4_reject_reason(buf: bytes | bytearray) -> str | None:
    b = _check_len(buf)
    if b[0] != SYNC0 or b[1] != SYNC1:
        return "sync"
    if b[2] != TYPE_NAV4:
        return "type"
    end = HEADER_BYTES + NAV3_LEN
    crc = crc16(b[2:end])
    return None if (b[end] | (b[end + 1] << 8)) == crc else "crc"


def decode_nav4(buf: bytes | bytearray) -> Nav3 | None:
    """nl_decode_nav4: the v4 navigation frame (v3 payload layout) of a valid exchange buffer, None otherwise."""
    if nav4_reject_reason(buf) is not None:
        return None
    b = bytes(buf)
    return Nav3.unpack(b[3], b[HEADER_BYTES:HEADER_BYTES + NAV3_LEN])


def tilt_q(x: float) -> int:
    """GNS side: one tilt component (rad) -> i16 x 1e4, saturating to -32767 .. 32767 (TILT_NONE stays reserved)."""
    return lp.quantize(float(x), TILT_SCALE, (-0x7FFF, 0x7FFF))


def pc_byte(mode: int, hold: bool, requests: int) -> int:
    """STATUS byte 3: mode | PC_HOLD | requests << 4."""
    if not 0 <= int(mode) <= PC_MODE_MASK or not 0 <= int(requests) <= 0x0F:
        raise ValueError("mode 0..7, requests 0..15")
    return int(mode) | (PC_HOLD if hold else 0) | (int(requests) << PC_REQ_SHIFT)


@dataclass(frozen=True)
class GnsData:
    """The FCU's view of one complete v4 slot set (``GnsSlotRx``): STATUS, POS, VEL, TRIM of one GNS cycle (its TILT
    part is applied on arrival, ``GnsSlotRx.target``)."""

    flags: int                     # sim.gps_nav GF_*
    num_sv: int
    hacc_dm: int
    mode: int                      # sim.position_ctrl MODE_*
    hold: bool                     # hold point captured: pos = p - hold point (W), else pos = p
    requests: int                  # sim.position_ctrl REQ_* (latched on the GNS until the motor stops)
    pos: tuple                     # m (W: x north, y west)
    vel: tuple                     # m/s (W)
    trim: tuple                    # the integrator's trim tilt, level frame (rad)
    tilt: tuple = (0.0, 0.0)       # the TILT part of the same cycle (rad; (0, 0) when not engaged)
    engaged: bool = False          # the TILT part was not TILT_NONE
    cycle: int = 0

    @property
    def fix_ok(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def gate_ok(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def dist_m(self) -> float:
        """|hold point - p| (NaN before the capture) - the TELEM v3 distance (one sqrt per set on the FCU)."""
        return math.hypot(self.pos[0], self.pos[1]) if self.hold else math.nan


def gns_slot_parts(payload: bytes) -> list[bytes]:
    """The five 4-byte parts of a glink RESULT payload (sim.glink_proto): TILT, STATUS, POS, VEL, TRIM."""
    if len(payload) != 4 * SLOT_PARTS:
        raise ValueError("a GNS result payload is 20 bytes")
    return [bytes(payload[4 * i:4 * i + 4]) for i in range(SLOT_PARTS)]


class GnsSlotTx:
    """NAV side of the v4 slot (module docstring "nlink v4"): the result of GNS cycle m (the glink STATE frame the NAV
    sampled at its tick k_m = 5 m + 4) goes into the frames of NAV ticks k_m + L + i, part i = 0 .. 4 (TILT first), L =
    ``latency_ticks`` (params flight_computer.glink.latency_ticks_ASSUMED; >= 2: the GNS cannot answer before the next
    NAV tick).  Deterministic phase: the slot of NAV tick k is part i = (k - 4 - L) mod 5 of cycle m = (k - 4 - L - i) / 5.
    At part 0 the NAV latches the result of cycle m if it holds one with SEQ == m & 0xFF that arrived after the cycle's
    sample tick (k_rx > k_m) and by this frame (k_rx <= k); otherwise the whole set of cycle m is SLOT_EMPTY (a late
    result is never sent).  ``result`` stores the decoded RESULT frames in a ring of ``RING`` entries indexed by SEQ mod
    RING (L up to 16 ticks)."""

    RING = 4

    def __init__(self, step_ticks: int = 5, latency_ticks: int = 3):
        if int(latency_ticks) < 2:
            raise ValueError("glink latency_ticks must be >= 2 (the GNS answers in the NAV's next tick at the earliest)")
        if int(latency_ticks) > 16:
            raise ValueError("glink latency_ticks > 16 does not fit the 4-entry result ring")
        self.n = int(step_ticks)
        self.L = int(latency_ticks)
        self.ring: list[tuple[int, bytes, int] | None] = [None] * self.RING
        self.cur: bytes | None = None
        self.cur_m: int | None = None
        self.latched = 0
        self.missed = 0
        self.results = 0

    def result(self, seq: int, payload: bytes, k_rx: int) -> None:
        """A valid glink RESULT frame (SEQ, 20-byte payload) decoded by the NAV in its tick ``k_rx``."""
        if len(payload) != 4 * SLOT_PARTS:
            raise ValueError("a GNS result payload is 20 bytes")
        self.ring[int(seq) % self.RING] = (int(seq) & 0xFF, bytes(payload), int(k_rx))
        self.results += 1

    def phase(self, k: int) -> tuple[int, int]:
        """(cycle m, part i) of the slot in the frame of NAV tick k (m < 0: before the first result can be due)."""
        r = int(k) - (self.n - 1) - self.L
        i = r % self.n
        return (r - i) // self.n, i

    def slot(self, k: int) -> bytes:
        m, i = self.phase(k)
        if m < 0:
            return SLOT_EMPTY
        if i == SLOT4_TILT:
            e = self.ring[m % self.RING]
            km = self.n * m + self.n - 1
            if e is not None and e[0] == (m & 0xFF) and km < e[2] <= k:
                self.cur = e[1]
                self.latched += 1
            else:
                self.cur = None
                self.missed += 1
            self.cur_m = m
        if self.cur is None or self.cur_m != m:
            return SLOT_EMPTY
        return bytes([((m & 0x1F) << 3) | i]) + self.cur[4 * i:4 * i + 4]


class GnsSlotRx:
    """FCU side of the v4 slot (module docstring "nlink v4").  ``feed`` once per FCU tick with the slot of this tick's
    valid NAV frame (None: no valid frame).  A TILT part is applied at once (``target``); the other parts complete a set
    at part 4 when parts 0..4 of ONE cycle arrived in five consecutive ticks (like ``GpsSlotRx``) - the set's STATUS
    (gate, requests, TELEM) is used from this tick.  Staleness (the FCU's own GNS-loss path): the age of the newest TILT
    part; fewer than two missed cycles (age < 2 x step_ticks) keep its target; from two missed cycles the target is the
    TRIM of the newest complete set (attitude-only hold with the wind trim, like the position controller's ATT mode - the
    FCU has no position estimate to dead-reckon on); from ``land_steps`` cycles the FCU raises REQ_GPS_LOST itself
    (``requests``) - the GNS cannot be reached, the V11 timing of a GPS loss."""

    STALE_TICKS = 10                                    # gate / TELEM freshness (two cycles), as GpsSlotRx

    def __init__(self, step_ticks: int = 5):
        self.n = int(step_ticks)
        self.parts: list[tuple[int, int, bytes] | None] = [None] * SLOT_PARTS
        self.tick = -1
        self.latest: GnsData | None = None
        self.t_latest: int | None = None
        self.got_part4 = False
        self.sets = 0
        self.broken = 0
        self.tilt = (0.0, 0.0)
        self.tilt_on = False
        self.t_tilt: int | None = None
        self.tilts = 0

    def feed(self, slot: bytes | None) -> GnsData | None:
        self.tick += 1
        self.got_part4 = False
        if slot is None:
            return None
        d = decode_slot(slot)
        if d is None:
            return None
        cyc, i, pl = d
        if i == SLOT4_TILT:
            tx, ty = struct.unpack("<hh", pl)
            if tx == TILT_NONE:
                self.tilt_on, self.tilt = False, (0.0, 0.0)
            else:
                self.tilt_on, self.tilt = True, (tx / TILT_SCALE, ty / TILT_SCALE)
            self.t_tilt = self.tick
            self.tilts += 1
        self.parts[i] = (self.tick, cyc, pl)
        if i != SLOT_PARTS - 1:
            return None
        self.got_part4 = True
        ps = self.parts
        if not all(x is not None and x[1] == cyc and self.tick - x[0] == SLOT_PARTS - 1 - j for j, x in enumerate(ps)):
            self.broken += 1
            return None
        tx, ty = struct.unpack("<hh", ps[SLOT4_TILT][2])
        fl, sv, ha, pcb = ps[SLOT4_STATUS][2]
        px, py = struct.unpack("<hh", ps[SLOT4_POS][2])
        vx, vy = struct.unpack("<hh", ps[SLOT4_VEL][2])
        rx_, ry_ = struct.unpack("<hh", ps[SLOT4_TRIM][2])
        eng = tx != TILT_NONE
        g = GnsData(flags=fl, num_sv=sv, hacc_dm=ha, mode=pcb & PC_MODE_MASK, hold=bool(pcb & PC_HOLD),
                    requests=pcb >> PC_REQ_SHIFT, pos=(px / 100.0, py / 100.0), vel=(vx / 100.0, vy / 100.0),
                    trim=(rx_ / TILT_SCALE, ry_ / TILT_SCALE),
                    tilt=(tx / TILT_SCALE, ty / TILT_SCALE) if eng else (0.0, 0.0), engaged=eng, cycle=cyc)
        self.latest, self.t_latest = g, self.tick
        self.sets += 1
        return g

    def fresh(self) -> bool:
        return self.t_latest is not None and self.tick - self.t_latest <= self.STALE_TICKS

    @property
    def gate_ok(self) -> bool:
        return bool(self.fresh() and self.latest.flags & 0x02)

    def tilt_age(self) -> int | None:
        """FCU ticks since the newest TILT part (None: none since boot)."""
        return None if self.t_tilt is None else self.tick - self.t_tilt

    def stale_cycles(self) -> int | None:
        a = self.tilt_age()
        return None if a is None else a // self.n

    def fallback(self) -> bool:
        """Two or more cycles without a TILT part: the trim fallback."""
        a = self.tilt_age()
        return a is None or a >= 2 * self.n

    def target(self) -> tuple[bool, tuple]:
        """(engaged, tilt target in the level frame) for the tilt-setpoint builder of this tick."""
        if not self.fallback():
            return self.tilt_on, self.tilt
        return self.tilt_on, (self.latest.trim if self.latest is not None else (0.0, 0.0))

    def requests(self, land_steps: int) -> int:
        """REQ_* for the supervisor (V11-V14): the newest set's latched requests | REQ_GPS_LOST (0x01) once no TILT part
        came for ``land_steps`` cycles (the FCU's own GNS-loss rule; params gps_loss timeout + land_after)."""
        rq = self.latest.requests if self.latest is not None else 0
        a = self.tilt_age()
        if a is None or a >= int(land_steps) * self.n:
            rq |= 0x01
        return rq

    def tlm(self, motor_on: bool) -> tuple:
        """TELEM v3 GPS fields (gps_flags, num_sv, hacc_dm, pos_mode, dist_m): flags / mode / distance of the newest
        fresh set; in the trim fallback (motor on, engaged) the mode reads ATT (4)."""
        g = self.latest
        fresh = self.fresh()
        flags = g.flags if (g is not None and fresh) else 0
        sv = g.num_sv if g is not None else 0
        ha = g.hacc_dm if g is not None else lp.HACC_NONE
        mode = g.mode if (g is not None and fresh) else 0
        if motor_on and self.tilt_on and self.fallback():
            mode = 4
        dist = g.dist_m if (g is not None and fresh) else math.nan
        return flags, sv, ha, mode, dist


# --------------------------------------------------------------------------- #
# the FCU slave's byte alignment (normative model of hal/nlink_spi.c, FCU role)
# --------------------------------------------------------------------------- #
@dataclass
class SpiSlaveModel:
    """The rules ``hal/nlink_spi.c`` (FCU = SPI slave) follows, at the byte level.

    1. Bytes: every byte the SPI hardware completes (SPIF) is stored at index ``idx`` of the write buffer and
       ``idx`` += 1; at ``idx`` == XFER_BYTES the exchange is complete: it becomes the ready frame (an untaken
       ready frame is replaced: ``overruns`` += 1), ``frames`` += 1, ``idx`` = 0.  No content check in the
       driver - the caller's ``nl_decode_nav`` checks sync, TYPE and CRC.
    2. Resync: whenever the main loop asks (``frame_ready`` / ``take``) and finds SS high and no byte waiting
       for its interrupt: if ``idx`` != 0 the partial exchange is discarded (``resyncs`` += 1, ``idx`` = 0); a
       pending reply becomes active; the driver (re)loads byte 0 of the active reply into SPDR.  A lost byte
       (interrupt latency > one byte + gap), an extra byte (SCK glitch) or an SS glitch therefore costs the
       exchange it happened in (it never completes, or completes misaligned and fails the CRC) and the next
       exchange is aligned again, provided the main loop polls once between exchanges with SS high - the FCU
       waits for the next frame in exactly that state every tick.  When SS glitches high the hardware also
       drops a partially shifted byte (the AVR resets its slave logic): the event 'G' marks that.
    3. Reply: byte k of an exchange returns ``reply[k]`` (k < FCU_FRAME_BYTES) or 0x00 from the active reply
       buffer; ``set_reply`` makes a new reply active at once when no exchange is in progress (idx == 0 and SS
       high: the driver also loads its byte 0 into SPDR), otherwise it waits (pending) and becomes active at
       the end of the exchange in progress or at the next rule-2 poll with SS high.
    The hardware timing that can still make one reply byte wrong (a slave interrupt later than the byte gap
    echoes the previous MOSI byte) is outside this model; the NAV's CRC check catches it."""

    idx: int = 0
    frames: int = 0
    resyncs: int = 0
    overruns: int = 0
    ready: bytes | None = None
    reply: bytes = bytes(FCU_FRAME_BYTES)
    pending: bytes | None = None
    _buf: bytearray = field(default_factory=bytearray)
    reply_bytes: int = FCU_FRAME_BYTES             # v0.8.1: FCU4_FRAME_BYTES (13) in the four-chip build

    def byte(self, mosi: int) -> int:
        """One complete byte: returns what the slave shifted out on MISO during it (its reply byte)."""
        miso = self.reply[self.idx] if self.idx < min(self.reply_bytes, len(self.reply)) else 0
        self._buf.append(mosi & 0xFF)
        self.idx += 1
        if self.idx == XFER_BYTES:
            if self.ready is not None:
                self.overruns = (self.overruns + 1) & 0xFFFF
            self.ready = bytes(self._buf)
            self.frames = (self.frames + 1) & 0xFFFF
            self._buf = bytearray()
            self.idx = 0
            if self.pending is not None:
                self.reply, self.pending = self.pending, None
        return miso

    def poll(self, ss_high: bool) -> None:
        """Rule 2 (``frame_ready`` / ``take`` call it first)."""
        if not ss_high:
            return
        if self.idx != 0:
            self.resyncs = (self.resyncs + 1) & 0xFFFF
            self.idx = 0
            self._buf = bytearray()
        if self.pending is not None:
            self.reply, self.pending = self.pending, None

    def frame_ready(self, ss_high: bool) -> bool:
        self.poll(ss_high)
        return self.ready is not None

    def take(self, ss_high: bool) -> bytes | None:
        self.poll(ss_high)
        r, self.ready = self.ready, None
        return r

    def set_reply(self, tx: bytes, ss_high: bool) -> None:
        n = self.reply_bytes
        if len(tx) < n:
            raise ValueError(f"the reply is the {n}-byte FCU frame")
        if self.idx == 0 and ss_high:
            self.reply, self.pending = bytes(tx[:n]), None
        else:
            self.pending = bytes(tx[:n])

    @property
    def stats(self) -> dict:
        return {"frames": self.frames, "resyncs": self.resyncs, "overruns": self.overruns}


# --------------------------------------------------------------------------- #
# timing (informative; the generator checks the same numbers)
# --------------------------------------------------------------------------- #
def exchange_time_s(spi_hz: float, byte_gap_s: float, bytes_: int = XFER_BYTES) -> float:
    """Wire time of one exchange: bytes x (8 bits + gap), without the master's loop overhead."""
    return bytes_ * (8.0 / spi_hz + byte_gap_s)


def load_nlink_params(path: str = PARAMS_PATH) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        p = json.load(f)
    nl = p["flight_computer"]["nlink"]
    return {"spi_clock_hz": float(nl["spi_clock_hz"]), "spi_mode": int(nl["spi_mode"]),
            "byte_gap_us": float(nl["byte_gap_us"]), "frame_timeout_ms": float(nl["frame_timeout_ms"]),
            "loop_hz": int(p["control"]["loop_hz"]), "nav_tick_hz": int(p["flight_computer"]["nav"]["tick_hz"])}


# --------------------------------------------------------------------------- #
# cross-check vectors for the firmware port (sim/out/nlink_vectors.json)
# --------------------------------------------------------------------------- #
QNAN, NQNAN, QNAN_PAYLOAD, SNAN = 0x7FC00000, 0xFFC00000, 0x7FC0BEEF, 0x7F800001
PINF, NINF, NZERO, DENORM_MIN, DENORM_MAX, FLT_MAX_BITS, NFLT_MIN = (0x7F800000, 0xFF800000, 0x80000000, 0x00000001,
                                                                     0x007FFFFF, 0x7F7FFFFF, 0x80800000)
SPECIAL_BITS = (QNAN, NQNAN, QNAN_PAYLOAD, SNAN, PINF, NINF, NZERO, DENORM_MIN, DENORM_MAX, 0x80000001, FLT_MAX_BITS,
                NFLT_MIN, 0x00800000)


def _decode_entry(buf: bytes) -> dict:
    """Expected results of both decoders on one 72-byte buffer."""
    out: dict = {"nav_valid": False, "fcu_valid": False}
    r = decode_nav_raw(buf)
    if r is not None:
        out.update(nav_valid=True, nav=nav_fields(*r))
    f = decode_fcu(buf)
    if f is not None:
        out.update(fcu_valid=True, fcu=[f.seq, f.state, f.fflags, f.params_tag], fcu_thr_q=f.thr_q)
    out["nav_reason"] = nav_reject_reason(buf) or ""
    out["fcu_reason"] = fcu_reject_reason(buf) or ""
    return out


def make_test_vectors(seed: int = 20261015) -> dict:
    """Deterministic vectors: NAV payloads (fields as float bit patterns + ints <-> 66 bytes <-> 72-byte frame) incl.
    every special float class, FCU frames of every state / flag, and decode cases (valid frames, every single-bit
    error of a NAV frame and of the FCU frame's 10 bytes, wrong sync / TYPE, the other side's frame, shifted by a
    lost or an extra byte, the echo of a late slave interrupt, bytes 10..71 of an FCU half non-zero, all-0 / all-FF)
    with the expected result of nl_decode_nav AND nl_decode_fcu on each buffer; plus SpiSlaveModel scenarios."""
    rng = random.Random(seed)
    navs: list[dict] = []

    def add_nav(name: str, seq: int, payload: bytes, py_roundtrip: bool = True, note: str = "") -> None:
        fr = encode_nav_payload(seq, payload)
        navs.append({"name": name, "note": note, "seq": seq, "payload_hex": payload.hex(), "frame_hex": fr.hex(),
                     "fields": nav_fields(seq, payload), "py_roundtrip": py_roundtrip})

    add_nav("nav_zero", 0, Nav(0, (0.0, 0.0, 0.0, 0.0)).pack())
    add_nav("nav_pad", 7, Nav(7, (1.0, 0.0, 0.0, 0.0), (0.001, -0.002, 0.0005), (0.05, -0.1, 9.80665), 0.0, 0.0, 0.31, 0.4,
                              0, 0, 0, 1432, F_IMU_OK | F_BARO_OK | F_ALIGNED | F_EST_VALID | F_BARO_NEW, 0x01).pack())
    add_nav("nav_boot", 0, Nav(0, flags=F_IMU_OK | F_BARO_OK | F_BOOT, reset_cause=0x08).pack(),
            note="first frame after a NAV watchdog reset: BOOT, no ALIGNED / EST_VALID")
    add_nav("nav_powered", 200, Nav(200, (0.99965, -0.0125, 0.0213, 0.0087), (0.12, -0.31, 1.5), (0.4, -0.2, 31.7), 119.3,
                                    22.71, 118.9, 1.34, 0, 2, 3, 1611,
                                    F_IMU_OK | F_BARO_OK | F_ALIGNED | F_EST_VALID, 0x01).pack())
    add_nav("nav_failing", 255, Nav(255, (0.7071, 0.0, 0.7071, 0.0), (34.9, -34.9, 0.0), (-156.9, 156.9, 0.0), -3.5, -45.0,
                                    -2.0, 90.0, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, F_GYRO_SAT | F_ALIGNED, 0xFF).pack(),
            note="IMU failing, gyro saturated, counters at their maximum")
    add_nav("nav_extremes", 128, Nav(128, (-1.0, 1.0, -0.5, 0.5), (1e30, -1e30, 3.4e38), (-3.4e38, 1e-30, -1e-30), 65535.5,
                                     -65535.5, 1e6, -1e6, 1, 2, 3, 4, 0x7F, 0x80).pack())
    ints = [5, 6, 7, 8, F_IMU_OK, 0x04]
    add_nav("nav_nan_inf", 9, nav_payload_from_bits([QNAN, PINF, NINF, NZERO, NQNAN, QNAN, PINF, NINF, NZERO, QNAN, PINF, NINF,
                                                     NZERO, NQNAN], ints),
            note="quiet NaN, +inf, -inf, -0.0, negative quiet NaN in every float slot")
    add_nav("nav_denormals", 10, nav_payload_from_bits([DENORM_MIN, DENORM_MAX, 0x80000001, NFLT_MIN, FLT_MAX_BITS,
                                                        0x3F800000, DENORM_MIN, 0x807FFFFF, 0x00400000, 0x00800000,
                                                        0xFF7FFFFF, 0x80000000, 0x00000000, 0x7F7FFFFF], ints),
            note="denormals (min / max, negative), -FLT_MIN, FLT_MAX, -FLT_MAX")
    add_nav("nav_nan_payload", 11, nav_payload_from_bits([QNAN_PAYLOAD, 0x3F800000] + [0] * 12, ints),
            note="quiet NaN with a payload: kept bit for bit")
    add_nav("nav_signalling_nan", 12, nav_payload_from_bits([SNAN, 0xFF800001] + [0x3F800000] * 12, ints), py_roundtrip=False,
            note="signalling NaN: the C port copies the 4 bytes (kept); Python quietens it")
    sync_f = struct.unpack("<f", bytes([SYNC0, SYNC1, TYPE_NAV, 0x00]))[0]
    add_nav("nav_sync_in_payload", SYNC0, Nav(SYNC0, (sync_f, sync_f, 1.0, 0.0), (sync_f, 0.0, 0.0), (0.0, 0.0, sync_f),
                                              sync_f, 0.0, 0.0, 0.0, 0x5DD5, 0xD55D, 0x01D5, 0x5D01, SYNC0, SYNC1).pack(),
            note="sync pairs and a plausible header inside a valid NAV frame (SEQ = 0xD5)")
    for i in range(6):
        add_nav(f"nav_random_{i}", rng.randrange(256), Nav(
            rng.randrange(256), tuple(rng.uniform(-1, 1) for _ in range(4)), tuple(rng.uniform(-10, 10) for _ in range(3)),
            tuple(rng.uniform(-160, 160) for _ in range(3)), rng.uniform(-50, 3500), rng.uniform(-400, 400),
            rng.uniform(-50, 3500), rng.uniform(0, 180), rng.randrange(65536), rng.randrange(65536), rng.randrange(65536),
            rng.randrange(65536), rng.randrange(256), rng.randrange(256)).pack())
    for i in range(4):
        add_nav(f"nav_random_bits_{i}", rng.randrange(256), nav_payload_from_bits(
            [rng.choice(SPECIAL_BITS) if rng.random() < 0.3 else rng.randrange(1 << 32) for _ in range(14)],
            [rng.randrange(65536) for _ in range(4)] + [rng.randrange(256) for _ in range(2)]),
            py_roundtrip=False, note="random bit patterns (sNaN possible)")

    fcus: list[dict] = []

    def add_fcu(name: str, f: Fcu, note: str = "") -> None:
        fcus.append({"name": name, "note": note, "fields": [f.seq, f.state, f.fflags, f.params_tag],
                     "frame_hex": encode_fcu(f).hex()})

    for st, nm in lp.STATE_NAMES.items():
        add_fcu(f"fcu_{nm.lower()}", Fcu((st * 37) & 0xFF, st, 0, 0x48))
    add_fcu("fcu_bench", Fcu(17, lp.STATE_SAFE, FF_BENCH, 0x48), note="bench_vanes: the NAV stays in flight mode")
    add_fcu("fcu_recovery", Fcu(255, lp.STATE_SAFE, FF_RECOVERY, 0x00), note="I10 boot")
    add_fcu("fcu_max", Fcu(255, 255, 255, 255))
    add_fcu("fcu_sync_values", Fcu(SYNC0, SYNC1, SYNC0, SYNC1))
    for i in range(4):
        add_fcu(f"fcu_random_{i}", Fcu(rng.randrange(256), rng.randrange(6), rng.randrange(4), rng.randrange(256)))

    # ---- decode cases: one 72-byte buffer each, both decoders' expected results
    decodes: list[dict] = []

    def add_dec(name: str, buf: bytes, note: str = "") -> None:
        assert len(buf) == XFER_BYTES
        decodes.append({"name": name, "note": note, "hex": buf.hex(), **_decode_entry(buf)})

    nav_frames = {v["name"]: bytes.fromhex(v["frame_hex"]) for v in navs}
    fcu_frames = {v["name"]: bytes.fromhex(v["frame_hex"]) for v in fcus}
    for nm, fr in nav_frames.items():
        add_dec(f"valid_{nm}", fr)
    for nm, fr in fcu_frames.items():
        add_dec(f"valid_{nm}", fr)
    base = nav_frames["nav_powered"]
    for j in range(XFER_BYTES):                                    # every single-bit error of a NAV frame
        for bit in range(8):
            bad = bytearray(base)
            bad[j] ^= 1 << bit
            add_dec(f"bitflip_nav_b{j}_{bit}", bytes(bad))
    fbase = fcu_frames["fcu_powered"]
    for j in range(FCU_FRAME_BYTES):                               # ... and of the FCU frame's 10 bytes
        for bit in range(8):
            bad = bytearray(fbase)
            bad[j] ^= 1 << bit
            add_dec(f"bitflip_fcu_b{j}_{bit}", bytes(bad))
    for j in (10, 11, 40, 71):                                     # bytes 10..71 of the FCU half: ignored
        bad = bytearray(fbase)
        bad[j] = 0xA5
        add_dec(f"fcu_tail_b{j}_nonzero", bytes(bad), note="bytes 10..71 of the FCU half are not looked at")
    echo = bytearray(fbase)
    echo[10:] = base[9:71]
    add_dec("fcu_tail_echo", bytes(echo), note="a late slave interrupt from byte 10 on echoes the NAV's bytes: still valid")
    bad = bytearray(fbase)
    bad[5] = base[4]
    add_dec("fcu_echo_in_frame", bytes(bad), note="a late slave interrupt at byte 5 echoes MOSI byte 4: CRC fails")
    add_dec("fcu_reserved_nonzero", encode_fcu(Fcu(3, lp.STATE_ARMED, 0, 0x48), reserved=0x7E),
            note="reserved byte != 0 with its CRC: accepted, reserved not part of nl_fcu_t")
    add_dec("lost_first_byte", base[1:] + bytes([SYNC0]), note="the NAV frame shifted by a lost byte")
    add_dec("extra_first_byte", bytes([0x00]) + base[:-1], note="an extra byte before the frame")
    add_dec("nav_type_fcu", base[:2] + bytes([TYPE_FCU]) + base[3:], note="TYPE 2 in a NAV frame: refused (type)")
    for t in (0x00, 0x03, 0xFF):
        body = bytes([t]) + base[3:70]
        crc = crc16(body)
        add_dec(f"type_0x{t:02x}_valid_crc", base[:2] + body + bytes([crc & 0xFF, crc >> 8]), note="unknown TYPE, CRC right")
    body = bytes([TYPE_FCU]) + base[3:70]
    crc = crc16(body)
    add_dec("fcu_type_with_nav_length", base[:2] + body + bytes([crc & 0xFF, crc >> 8]),
            note="TYPE FCU, CRC over 68 bytes: refused by both (FCU CRC at bytes 8, 9)")
    add_dec("sync_swapped", bytes([SYNC1, SYNC0]) + base[2:])
    add_dec("all_zero", bytes(XFER_BYTES))
    add_dec("all_ff", bytes([0xFF]) * XFER_BYTES, note="MISO pulled up, FCU absent")
    for i in range(16):
        b = bytearray(rng.randrange(256) for _ in range(XFER_BYTES))
        if i % 2 == 0:
            b[0], b[1], b[2] = SYNC0, SYNC1, rng.choice([TYPE_NAV, TYPE_FCU])
        add_dec(f"random_{i}", bytes(b))

    # ---- SpiSlaveModel scenarios (normative for hal/nlink_spi.c's FCU slave): events and the expected outcome
    scen = slave_scenarios(rng, nav_frames, fcu_frames)

    # ---- v0.7.1: the FCU frame's thr_q byte (VTOL ESC throttle for the NAV's baro thrust compensation).  Appended after
    # every v0.6.3 block and without the shared RNG, so the earlier vectors keep their bytes.
    fcus_vtol: list[dict] = []
    thr_cases = [("fcu_vtol_motor_off", Fcu(21, lp.STATE_ARMED, 0, 0x48, fcu_throttle_q(0.0)), 0.0),
                 ("fcu_vtol_hover", Fcu(22, lp.STATE_POWERED, 0, 0x48, fcu_throttle_q(0.9148)), 0.9148),
                 ("fcu_vtol_full", Fcu(23, lp.STATE_POWERED, 0, 0x48, fcu_throttle_q(1.0)), 1.0),
                 ("fcu_vtol_half_step", Fcu(24, lp.STATE_POWERED, 0, 0x48, fcu_throttle_q(0.502)), 0.502),
                 ("fcu_vtol_u_min", Fcu(25, lp.STATE_POWERED, 0, 0x48, fcu_throttle_q(0.5)), 0.5),
                 ("fcu_vtol_landed", Fcu(26, lp.STATE_UNPOWERED, 0, 0x48, 0), 0.0),
                 ("fcu_vtol_thr_q_251", Fcu(27, lp.STATE_POWERED, 0, 0x48, 251), None),
                 ("fcu_vtol_thr_q_255", Fcu(28, lp.STATE_POWERED, 0, 0x48, 255), None)]
    for nm, f, u in thr_cases:
        fr = encode_fcu(f)
        fcus_vtol.append({"name": nm, "fields": [f.seq, f.state, f.fflags, f.params_tag], "thr_q": f.thr_q,
                          "u_in": u, "u_nav": fcu_throttle(f.thr_q), "frame_hex": fr.hex(),
                          "note": "thr_q = round_half_away(u x 250)" if u is not None else
                                  "thr_q > 250 (no FCU encodes it): accepted, the NAV reads u = 1"})
        decodes.append({"name": f"valid_{nm}", "note": "v0.7.1 thr_q byte", "hex": fr.hex(), **_decode_entry(fr)})
    quant = [{"u": u, "thr_q": fcu_throttle_q(u)} for u in (0.0, 0.002, 0.0019999, 0.006, 0.5, 0.9148, 0.998, 0.9979999,
                                                             1.0, 1.5, -0.1)]     # NaN -> 0: tested in sim/test_nlink_proto.py

    v3 = make_v3_vectors()
    v4 = make_v4_vectors()

    return {
        "_doc": ("nlink (NAV <-> FCU, SPEC 10.5) vectors generated by sim/nlink_proto.py make_test_vectors(). navs: fields "
                 "(14 float bit patterns in wire order + 6 integers) <-> 66-byte payload <-> 72-byte frame. fcus: fields "
                 "<-> 72-byte FCU half (10-byte frame + 62 zeros). decodes: one 72-byte exchange buffer each with the "
                 "expected results of nl_decode_nav and nl_decode_fcu (validity and fields) - all normative for the C port. "
                 "slave: SpiSlaveModel scenarios for hal/nlink_spi.c (events: B<hex> one byte, S1/S0 SS level, P poll, "
                 "T take, R<hex> set_reply) with the bytes returned and the counters. v0.7.1: byte 7 of the FCU frame is "
                 "thr_q (decodes: fcu_thr_q; fcus_vtol: FCU frames with a throttle; thr_quant: u -> thr_q)."),
        "seed": seed,
        "crc16_check": {"ascii": "123456789", "crc": crc16(b"123456789")},
        "constants": {"SYNC": [SYNC0, SYNC1], "TYPE_NAV": TYPE_NAV, "TYPE_FCU": TYPE_FCU, "NAV_LEN": NAV_LEN,
                      "HEADER_BYTES": HEADER_BYTES, "XFER_BYTES": XFER_BYTES, "FCU_LEN": FCU_LEN,
                      "FCU_FRAME_BYTES": FCU_FRAME_BYTES, "FCU_THR_SCALE": FCU_THR_SCALE,
                      "flags": {"IMU_OK": F_IMU_OK, "BARO_OK": F_BARO_OK, "ALIGNED": F_ALIGNED, "GYRO_SAT": F_GYRO_SAT,
                                "EST_VALID": F_EST_VALID, "BARO_NEW": F_BARO_NEW, "BOOT": F_BOOT},
                      "fflags": {"BENCH": FF_BENCH, "RECOVERY": FF_RECOVERY}},
        "navs": navs,
        "fcus": fcus,
        "decodes": decodes,
        "slave": scen,
        "fcus_vtol": fcus_vtol,
        "thr_quant": quant,
        "v3": v3,
        "v4": v4,
    }


def _nav3_fields(seq: int, payload: bytes) -> dict:
    u = struct.unpack("<12IhhhBBBHBB5s", payload)
    return {"seq": seq, "float_bits": list(u[:12]), "accel_x_q": u[12], "accel_y_q": u[13], "h_baro_rel_cm": u[14],
            "imu_fail": u[15], "baro_fails": u[16], "twi_errors": u[17], "loop_us_max": u[18], "flags": u[19],
            "reset_cause": u[20], "slot_hex": u[21].hex()}


def make_v3_vectors() -> dict:
    """v0.8 nlink v3 vectors (own RNG: the v0.6.3 / v0.7.1 blocks keep their bytes): v3 payloads (fields <-> 66 bytes
    <-> 72-byte frame), decodes (valid, every single-bit error, the v0.6.3 TYPE fed to the v3 decoder and back), GPS
    slots (snapshot -> the five slots) and slot-receiver streams (complete, a lost frame, a cycle change, j > 4)."""
    import random
    try:
        from sim.gps_nav import GF_FIX_OK, GF_GATE_OK, GF_HEAD_OK, GF_FIX3D, GF_INIT, GpsSnapshot
    except ImportError:  # pragma: no cover
        from gps_nav import GF_FIX_OK, GF_GATE_OK, GF_HEAD_OK, GF_FIX3D, GF_INIT, GpsSnapshot  # type: ignore
    rng = random.Random(20261021)
    snap = GpsSnapshot(flags=GF_FIX_OK | GF_GATE_OK | GF_HEAD_OK | GF_FIX3D | GF_INIT, num_sv=9, hacc_dm=25,
                       reset_count=3, off_x_cm=-152, off_y_cm=48, px_cm=123, py_cm=-457, vx_cms=-12, vy_cms=31,
                       cos_q15=32610, sin_q15=-3212)
    navs3 = []

    def add(name: str, n: Nav3, note: str = "") -> None:
        pl = n.pack()
        navs3.append({"name": name, "note": note, "payload_hex": pl.hex(), "frame_hex": encode_nav3(n).hex(),
                      "fields": _nav3_fields(n.seq, pl)})

    add("nav3_pad", Nav3.from_physical(7, (1.0, 0.0, 0.0, 0.0), (0.001, -0.002, 0.0005), (0.05, -0.1, 9.80665), 0.0,
                                       0.0, 0.31, 0, 0, 0, 1432, F_IMU_OK | F_BARO_OK | F_ALIGNED | F_EST_VALID, 1,
                                       encode_slot(snap, 17, 0)))
    add("nav3_hover", Nav3.from_physical(201, (0.99965, -0.0125, 0.0213, 0.0087), (0.12, -0.31, 1.5), (0.4, -0.2, 9.71),
                                         5.03, -0.04, 5.2, 0, 2, 3, 2290, F_IMU_OK | F_BARO_OK | F_ALIGNED | F_EST_VALID
                                         | 0x80, 1, encode_slot(snap, 18, 3)))
    add("nav3_saturating", Nav3.from_physical(255, (0.7071, 0.0, 0.7071, 0.0), (34.9, -34.9, 0.0), (-400.0, 400.0, 0.0),
                                              -3.5, -45.0, 500.0, 70000, 300, 256, 0xFFFF, F_GYRO_SAT | F_ALIGNED, 0xFF,
                                              encode_slot(snap, 31, 4)),
        note="accel x / y and h_baro saturate at the i16 limits, counters at 255")
    add("nav3_empty_slot", Nav3.from_physical(0, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 9.8), 0.0, 0.0, 0.0,
                                              0, 0, 0, 0, F_IMU_OK | F_BARO_OK | F_BOOT, 0x08, SLOT_EMPTY),
        note="before the NAV's first step: header 0x07")
    for i in range(4):
        q = [rng.uniform(-1, 1) for _ in range(4)]
        nq = math.sqrt(sum(x * x for x in q))
        add(f"nav3_random_{i}", Nav3.from_physical(rng.randrange(256), tuple(x / nq for x in q),
                                                    tuple(rng.uniform(-10, 10) for _ in range(3)),
                                                    tuple(rng.uniform(-160, 160) for _ in range(3)),
                                                    rng.uniform(-5, 40), rng.uniform(-10, 10), rng.uniform(-5, 40),
                                                    rng.randrange(400), rng.randrange(400), rng.randrange(400),
                                                    rng.randrange(65536), rng.randrange(256), rng.randrange(256),
                                                    bytes(rng.randrange(256) for _ in range(5))))
    decodes = []

    def dec(name: str, buf: bytes, note: str = "") -> None:
        n3 = decode_nav3(buf)
        decodes.append({"name": name, "note": note, "hex": buf.hex(), "nav3_valid": n3 is not None,
                        "nav3_reason": nav3_reject_reason(buf) or "", "nav_valid": decode_nav(buf) is not None,
                        "fcu_valid": decode_fcu(buf) is not None,
                        **({"nav3": _nav3_fields(n3.seq, n3.pack())} if n3 is not None else {})})

    base = bytes.fromhex(navs3[1]["frame_hex"])
    for v in navs3:
        dec(f"valid_{v['name']}", bytes.fromhex(v["frame_hex"]))
    for j in range(XFER_BYTES):
        for bit in range(8):
            b = bytearray(base)
            b[j] ^= 1 << bit
            dec(f"bitflip_nav3_b{j}_{bit}", bytes(b))
    v063 = encode_nav(Nav(9, (1.0, 0.0, 0.0, 0.0)))
    dec("v063_frame_to_v3_decoder", v063, "TYPE_NAV (v0.6.3 / ballistic) refused by the v3 decoder: type")
    dec("v3_frame_type_nav", base[:2] + bytes([TYPE_NAV]) + base[3:], "TYPE byte changed to 1: both refuse (CRC)")
    body = bytes([TYPE_NAV]) + base[3:70]
    crc = crc16(body)
    dec("v3_payload_as_type_nav_valid_crc", base[:2] + body + bytes([crc & 0xFF, crc >> 8]),
        "a v3 payload under TYPE_NAV with its CRC: the v0.6.3 decoder accepts it (no content check), v3 refuses")
    # slots: one snapshot -> five slots
    slots = [{"cycle": 18, "j": j, "hex": encode_slot(snap, 18, j).hex()} for j in range(SLOT_PARTS)]
    rx_cases = []

    def run_rx(name: str, seq: list, note: str = "") -> None:
        rx = GpsSlotRx()
        out = []
        for s in seq:
            g = rx.feed(None if s is None else bytes.fromhex(s))
            out.append(None if g is None else {"flags": g.flags, "num_sv": g.num_sv, "hacc_dm": g.hacc_dm,
                                               "reset_count": g.reset_count, "off": list(g.off), "p": list(g.p),
                                               "v": list(g.v), "cos_psi": g.cos_psi, "sin_psi": g.sin_psi,
                                               "cycle": g.cycle})
        rx_cases.append({"name": name, "note": note, "slots": seq, "out": out, "gate_ok_after": rx.gate_ok,
                         "sets": rx.sets, "broken": rx.broken})

    s18 = [encode_slot(snap, 18, j).hex() for j in range(5)]
    s19 = [encode_slot(snap, 19, j).hex() for j in range(5)]
    run_rx("complete", s18, "five consecutive parts of cycle 18: complete at part 4")
    run_rx("two_cycles", s18 + s19)
    run_rx("lost_frame", s18[:2] + [None] + s18[3:] + s19, "frame of part 2 lost: cycle 18 broken, 19 complete")
    run_rx("mixed_cycles", s18[:3] + s19[3:] + s19, "parts of two cycles: broken, then complete")
    run_rx("empty_slots", [SLOT_EMPTY.hex()] * 6 + s18, "no snapshot yet, then cycle 18")
    run_rx("late_start", s18[2:] + s19, "starts mid-cycle: the first set is broken")
    return {"_doc": ("nlink v3 (v0.8 VTOL + GPS, TYPE 0x03, the 72-byte exchange kept): navs3 = fields (12 float bit "
                     "patterns: q4 rate3 accel_z h vz cos_tilt tilt; then the integers) <-> 66-byte payload <-> frame; "
                     "decodes = the expected result of nl_decode_nav3 (and of the v0.6.3 nl_decode_nav / nl_decode_fcu) "
                     "per buffer; slots = one snapshot's five slot byte strings; rx = GpsSlotRx sequences (one slot "
                     "per FCU tick, null = no valid frame) with the sets returned (floats: the exact divisions cm / "
                     "100, Q15 / 32767 - compare within float32)."),
            "constants": {"TYPE_NAV3": TYPE_NAV3, "NAV3_LEN": NAV3_LEN, "SLOT_LEN": SLOT_LEN, "SLOT_EMPTY": SLOT_EMPTY.hex(),
                          "STALE_TICKS": GpsSlotRx.STALE_TICKS, "ACCEL_XY_SCALE": ACCEL_XY_SCALE,
                          "H_BARO_SCALE": H_BARO_SCALE},
            "snapshot": dict(vars(snap)), "navs3": navs3, "decodes": decodes, "slots": slots, "rx": rx_cases}


def make_v4_vectors() -> dict:
    """v0.8.1 nlink v4 vectors (own RNG; the earlier blocks keep their bytes): NAV4 frames (the v3 payload under TYPE 4
    with GNS slots), FCU4 halves, decodes (valid, every single-bit error of an FCU4 frame's 13 bytes, the v3 / v0.6.3
    TYPEs fed to the v4 decoders and back) and a slave scenario with the 13-byte reply.  The GNS slot's NAV phase and FCU
    receiver sequences are in sim/out/glink_vectors.json (slot_tx / slot_rx)."""
    import random
    rng = random.Random(20261022)
    parts = [bytes([0x2c, 0x01, 0x7c, 0xfc]), bytes([0x43, 9, 25, 0x0A]), bytes([0xdb, 0xff, 0x34, 0x00]),
             bytes([0xf8, 0xff, 0x0b, 0x00]), bytes([0xcd, 0x00, 0x9d, 0xfd])]
    navs4 = []
    for name, seq, cyc, i in (("nav4_tilt", 7, 18, 0), ("nav4_status", 8, 18, 1), ("nav4_trim", 255, 31, 4)):
        n = Nav3.from_physical(seq, (0.99965, -0.0125, 0.0213, 0.0087), (0.12, -0.31, 1.5), (0.4, -0.2, 9.71), 5.03,
                               -0.04, 5.2, 0, 2, 3, 2290, F_IMU_OK | F_BARO_OK | F_ALIGNED | F_EST_VALID, 1,
                               bytes([((cyc & 0x1F) << 3) | i]) + parts[i])
        navs4.append({"name": name, "payload_hex": n.pack().hex(), "frame_hex": encode_nav4(n).hex(),
                      "fields": _nav3_fields(n.seq, n.pack())})
    navs4.append({"name": "nav4_empty", "payload_hex": Nav3(0).pack().hex(), "frame_hex": encode_nav4(Nav3(0)).hex(),
                  "fields": _nav3_fields(0, Nav3(0).pack())})
    fcus4 = []
    for name, f in (("pad", Fcu4(3, lp.STATE_SAFE, 0, 0x5A, 0, 0, KT_SCALE)),
                    ("climb_sat", Fcu4(40, lp.STATE_POWERED, 0, 0x5A, 220,
                                       fcu4_pflags(lp.PHASE_CLIMB, True, True, False, False, False), kt_quant(1.0))),
                    ("hold", Fcu4(201, lp.STATE_POWERED, 0, 0x5A, 172,
                                  fcu4_pflags(lp.PHASE_HOLD, True, False, False, False, True), kt_quant(0.9731))),
                    ("land_ground", Fcu4(77, lp.STATE_POWERED, 0, 0x5A, 60,
                                         fcu4_pflags(lp.PHASE_LAND, True, False, True, True, True), kt_quant(1.4))),
                    ("random", Fcu4(rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(256),
                                    rng.randrange(256), rng.randrange(256), rng.randrange(65536)))):
        buf = encode_fcu4(f)
        fcus4.append({"name": name, "fields": dict(vars(f)), "hex": buf.hex()})
    decodes = []

    def dec(name: str, buf: bytes, note: str = "") -> None:
        n4, f4 = decode_nav4(buf), decode_fcu4(buf)
        decodes.append({"name": name, "note": note, "hex": buf.hex(), "nav4_valid": n4 is not None,
                        "nav4_reason": nav4_reject_reason(buf) or "", "fcu4_valid": f4 is not None,
                        "fcu4_reason": fcu4_reject_reason(buf) or "", "nav3_valid": decode_nav3(buf) is not None,
                        "fcu_valid": decode_fcu(buf) is not None,
                        **({"fcu4": dict(vars(f4))} if f4 is not None else {})})

    for v in navs4:
        dec(f"valid_{v['name']}", bytes.fromhex(v["frame_hex"]))
    for v in fcus4:
        dec(f"valid_fcu4_{v['name']}", bytes.fromhex(v["hex"]))
    base = bytes.fromhex(fcus4[2]["hex"])
    for j in range(FCU4_FRAME_BYTES):
        for bit in range(8):
            b = bytearray(base)
            b[j] ^= 1 << bit
            dec(f"bitflip_fcu4_b{j}_{bit}", bytes(b))
    tail = bytearray(base)
    tail[40] = 0x5A
    dec("fcu4_tail_ignored", bytes(tail), "bytes 13..71 of the FCU half are not looked at")
    n3 = encode_nav3(Nav3(5))
    dec("v3_frame_to_v4", n3, "a v0.8 (three-chip) NAV frame: the v4 decoder refuses it (type)")
    dec("v4_frame_to_v3", bytes.fromhex(navs4[0]["frame_hex"]), "and the v3 decoder refuses a v4 frame")
    dec("v071_fcu_to_fcu4", encode_fcu(Fcu(9, lp.STATE_POWERED, 0, 0x5A, 172)), "a v0.7.1 / v0.8 FCU frame: type")
    slave = []
    rep = [bytes.fromhex(v["hex"])[:FCU4_FRAME_BYTES] for v in fcus4]
    m = SpiSlaveModel(reply_bytes=FCU4_FRAME_BYTES)
    ev = ["R" + rep[0].hex()] + _exchange(bytes.fromhex(navs4[0]["frame_hex"])) + ["P", "T", "R" + rep[2].hex()] \
        + _exchange(bytes.fromhex(navs4[1]["frame_hex"])) + ["T"]
    ss_high = True
    miso = bytearray()
    taken = []
    for e in ev:
        if e[0] == "S":
            ss_high = e[1] == "1"
        elif e[0] == "B":
            miso.append(m.byte(int(e[1:], 16)))
        elif e[0] == "P":
            m.poll(ss_high)
        elif e[0] == "T":
            r = m.take(ss_high)
            taken.append("-" if r is None else r.hex())
        elif e[0] == "R":
            m.set_reply(bytes.fromhex(e[1:]), ss_high)
    slave.append({"name": "v4_two_exchanges", "reply_bytes": FCU4_FRAME_BYTES, "events": ev, "miso_hex": miso.hex(),
                  "taken": taken, "stats": m.stats})
    return {"_doc": ("nlink v4 (v0.8.1 four-chip build): navs4 = v3-layout payloads under TYPE_NAV4 with GNS slot parts; "
                     "fcus4 = the 13-byte FCU4 frame + 59 zeros (fields: state, fflags, params_tag, thr_q, pflags, kt_q); "
                     "decodes = the expected nl_decode_nav4 / nl_decode_fcu4 (and the v3 / v0.7.1 decoders) per buffer; "
                     "slave = SpiSlaveModel with a 13-byte reply.  GNS slot phase / receiver: glink_vectors.json."),
            "constants": {"TYPE_NAV4": TYPE_NAV4, "TYPE_FCU4": TYPE_FCU4, "FCU4_LEN": FCU4_LEN,
                          "FCU4_FRAME_BYTES": FCU4_FRAME_BYTES, "KT_SCALE": KT_SCALE, "TILT_SCALE": TILT_SCALE,
                          "TILT_NONE": TILT_NONE, "PF_PHASE_MASK": PF_PHASE_MASK, "PF_MOTOR_ON": PF_MOTOR_ON,
                          "PF_SAT_HI": PF_SAT_HI, "PF_LOW_ALT": PF_LOW_ALT, "PF_GROUND": PF_GROUND,
                          "PF_LIFTED": PF_LIFTED, "PC_MODE_MASK": PC_MODE_MASK, "PC_HOLD": PC_HOLD,
                          "PC_REQ_SHIFT": PC_REQ_SHIFT, "SLOT4": list(SLOT4_NAMES),
                          "STALE_TICKS": GnsSlotRx.STALE_TICKS, "GNS_SLOT_RING": GnsSlotTx.RING},
            "navs4": navs4, "fcus4": fcus4, "decodes": decodes, "slave": slave}


def run_slave_events(events: Sequence[str]) -> dict:
    """Run SpiSlaveModel over an event list (the vector / simavr-harness format): 'S1' / 'S0' SS high / low,
    'B<hex>' one complete byte from the master (MISO byte recorded), 'G' an SS glitch inside a byte (the hardware
    drops the partial byte: nothing stored), 'P' main-loop poll, 'T' take (frame hex recorded, '-' none),
    'R<hex>' set_reply (10 bytes).  Returns the MISO bytes, the taken frames and the counters."""
    m = SpiSlaveModel()
    ss_high = True
    miso = bytearray()
    taken: list[str] = []
    for ev in events:
        k = ev[0]
        if k == "S":
            ss_high = ev[1] == "1"
        elif k == "B":
            miso.append(m.byte(int(ev[1:], 16)))
        elif k == "G":
            pass
        elif k == "P":
            m.poll(ss_high)
        elif k == "T":
            r = m.take(ss_high)
            taken.append("-" if r is None else r.hex())
        elif k == "R":
            m.set_reply(bytes.fromhex(ev[1:]), ss_high)
        else:  # pragma: no cover
            raise ValueError(f"unknown event {ev!r}")
    return {"miso_hex": miso.hex(), "taken": taken, "stats": m.stats, "idx": m.idx}


def _exchange(frame: bytes, drop: Sequence[int] = (), extra_at: int | None = None, glitch_at: int | None = None) -> list[str]:
    """Events of one SS-low exchange of ``frame``; bytes in ``drop`` never reach the slave's ISR (lost: interrupt
    too late), ``extra_at`` inserts a spurious 0x00 byte (SCK glitch), ``glitch_at``: SS pulses high while that byte
    is shifting (the slave hardware resets and drops it; the master goes on with the next byte)."""
    ev = ["S0"]
    for j, b in enumerate(frame):
        if glitch_at == j:
            ev += ["S1", "G", "S0"]
            continue
        if extra_at == j:
            ev.append("B00")
        if j not in drop:
            ev.append(f"B{b:02x}")
    ev.append("S1")
    return ev


def slave_scenarios(rng: random.Random, nav_frames: dict, fcu_frames: dict) -> list[dict]:
    navs = list(nav_frames.values())
    fcu = list(fcu_frames.values())
    rep = [f[:FCU_FRAME_BYTES].hex() for f in fcu]
    out: list[dict] = []

    def add(name: str, events: list[str], note: str = "") -> None:
        out.append({"name": name, "note": note, "events": events, **run_slave_events(events)})

    a, b, c = navs[1], navs[3], navs[4]
    add("clean_three", ["R" + rep[0]] + _exchange(a) + ["P", "T", "R" + rep[1]] + _exchange(b) + ["T", "R" + rep[2]]
        + _exchange(c) + ["T"], note="three exchanges, each taken, a new reply set between them")
    add("lost_byte_resync", ["R" + rep[0]] + _exchange(a, drop=[30]) + ["P", "T"] + _exchange(b) + ["T"],
        note="byte 30 lost: 71 bytes, the poll with SS high discards them (resync); the next exchange is aligned")
    add("lost_byte_no_poll", ["R" + rep[0]] + _exchange(a, drop=[30]) + _exchange(b) + ["T", "P"] + _exchange(c) + ["T"],
        note="no poll between: the next exchange's first byte completes a misaligned frame (CRC fails at the caller), "
             "its remaining 71 bytes are discarded by the next poll, the third is aligned")
    add("extra_byte", ["R" + rep[0]] + _exchange(a, extra_at=10) + ["P", "T"] + _exchange(b) + ["T"],
        note="an SCK glitch adds a byte: the frame completes one byte early (misaligned), 1 byte left -> resync")
    add("ss_glitch", ["R" + rep[0]] + _exchange(a, glitch_at=20) + ["P", "T"] + _exchange(b) + ["T"],
        note="SS pulses high mid-frame (the hardware drops the partial byte): 71 bytes, resync")
    add("ss_glitch_polled", ["R" + rep[0]] + _exchange(a)[:22] + ["S1", "P", "S0"] + _exchange(a)[22:] + ["P", "T"]
        + _exchange(b) + ["T"], note="the main loop polls while SS is (glitched) high mid-frame: resync there, the rest "
                                     "of the frame is a partial exchange discarded by the next poll")
    add("overrun", ["R" + rep[0]] + _exchange(a) + _exchange(b) + ["T", "T"],
        note="two exchanges without a take: the newer one is kept, overruns 1")
    add("reply_pending_mid_exchange", ["R" + rep[0]] + _exchange(a)[:30] + ["R" + rep[3]] + _exchange(a)[30:]
        + _exchange(b) + ["T"], note="set_reply during an exchange: pending, active from the next exchange on")
    add("reply_while_idle", ["R" + rep[0]] + _exchange(a) + ["T", "R" + rep[4]] + _exchange(b) + ["T"],
        note="set_reply between exchanges (SS high, idx 0): this very exchange returns it")
    add("reply_after_resync", ["R" + rep[0]] + _exchange(a, drop=[5]) + ["R" + rep[5], "P"] + _exchange(b) + ["T"],
        note="set_reply while a partial exchange is open: pending; the resync makes it active and preloads byte 0")
    add("take_nothing", ["P", "T"] + _exchange(a)[:10] + ["T", "P"] + _exchange(a)[10:] + ["T"],
        note="take with nothing ready; a take mid-exchange (SS low) does not resync")
    add("reply_pending_empty_exchange", ["R" + rep[0], "S0", "R" + rep[1], "S1", "P"] + _exchange(b) + ["T"],
        note="set_reply while SS is low before the first byte: pending; SS rises without a byte, the poll makes it "
             "active")
    for i in range(10):                                            # random mixes
        ev: list[str] = ["R" + rng.choice(rep)]
        for _ in range(rng.randrange(3, 10)):
            fr = rng.choice(navs)
            k = rng.random()
            if k < 0.5:
                ev += _exchange(fr)
            elif k < 0.65:
                ev += _exchange(fr, drop=rng.sample(range(XFER_BYTES), rng.choice([1, 1, 2])))
            elif k < 0.75:
                ev += _exchange(fr, extra_at=rng.randrange(XFER_BYTES))
            elif k < 0.85:
                ev += _exchange(fr, glitch_at=rng.randrange(1, XFER_BYTES))
            else:
                cut = rng.randrange(1, XFER_BYTES)
                ev += _exchange(fr)[:cut + 1] + ["S1"]               # the master stopped mid-frame (NAV reset)
            r = rng.random()
            if r < 0.6:
                ev += ["P", "T"]
            elif r < 0.8:
                ev += ["T"]
            if rng.random() < 0.5:
                ev.append("R" + rng.choice(rep))
        add(f"random_{i}", ev)
    return out


def write_test_vectors(path: str = VECTORS_PATH) -> dict:
    v = make_test_vectors()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(v, f, indent=1)
        f.write("\n")
    return v


if __name__ == "__main__":  # pragma: no cover
    _v = write_test_vectors()
    sys.stdout.write(f"wrote {os.path.relpath(VECTORS_PATH, os.path.join(_HERE, '..'))}: {len(_v['navs'])} NAV payloads, "
                     f"{len(_v['fcus'])} FCU frames, {len(_v['decodes'])} decode cases, {len(_v['slave'])} slave scenarios\n")
