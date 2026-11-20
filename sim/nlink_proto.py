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

thr_q: the v0.6.3 reserved byte - the rocket's FCU always writes 0 and its NAV ignores it.  (The removed VTOL program,
2026-11-18, sent its ESC throttle in it; the name and the byte stay so the exchange and the flight images are
unchanged.)

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
whatever its value.
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
build) only switches the estimator to gyro-only on the pad; the altitude stays held.

TYPE 0x03 .. 0x05 (nlink v3 / v4 of the removed VTOL program, 2026-11-18) are not used: both decoders refuse them
('type').

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
FCU_FMT = "<BBBB"                                     # state, fflags, params_tag, thr_q (reserved, 0)
FCU_LEN = struct.calcsize(FCU_FMT)                    # 4
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
    thr_q: int = 0                 # reserved byte (the rocket's FCU writes 0)

    def __post_init__(self) -> None:
        for n in FCU_FIELDS:
            _check(n, getattr(self, n), U8)


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
    (reserved; the FCU writes 0); ``reserved`` (v0.6.3 test-frame argument) overrides it when given."""
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


def nav_reset_cause_names(reset_cause: int) -> tuple[list[str], list[str]]:
    """(MCUCSR reset flags of bits 4..0, NAV_RC_* results of bits 7..5) of a Nav.reset_cause, by name."""
    _check("reset_cause", reset_cause, U8)
    return ([n for b, n in MCUCSR_RESET_NAMES.items() if reset_cause & b],
            [n for b, n in NAV_RC_NAMES.items() if reset_cause & b])


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
    reply_bytes: int = FCU_FRAME_BYTES

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

    return {
        "_doc": ("nlink (NAV <-> FCU, SPEC 10.5) vectors generated by sim/nlink_proto.py make_test_vectors(). navs: fields "
                 "(14 float bit patterns in wire order + 6 integers) <-> 66-byte payload <-> 72-byte frame. fcus: fields "
                 "<-> 72-byte FCU half (10-byte frame + 62 zeros). decodes: one 72-byte exchange buffer each with the "
                 "expected results of nl_decode_nav and nl_decode_fcu (validity and fields) - all normative for the C port. "
                 "slave: SpiSlaveModel scenarios for hal/nlink_spi.c (events: B<hex> one byte, S1/S0 SS level, P poll, "
                 "T take, R<hex> set_reply) with the bytes returned and the counters. Byte 7 of the FCU frame is the reserved "
                 "thr_q byte (decodes: fcu_thr_q)."),
        "seed": seed,
        "crc16_check": {"ascii": "123456789", "crc": crc16(b"123456789")},
        "constants": {"SYNC": [SYNC0, SYNC1], "TYPE_NAV": TYPE_NAV, "TYPE_FCU": TYPE_FCU, "NAV_LEN": NAV_LEN,
                      "HEADER_BYTES": HEADER_BYTES, "XFER_BYTES": XFER_BYTES, "FCU_LEN": FCU_LEN,
                      "FCU_FRAME_BYTES": FCU_FRAME_BYTES,
                      "flags": {"IMU_OK": F_IMU_OK, "BARO_OK": F_BARO_OK, "ALIGNED": F_ALIGNED, "GYRO_SAT": F_GYRO_SAT,
                                "EST_VALID": F_EST_VALID, "BARO_NEW": F_BARO_NEW, "BOOT": F_BOOT},
                      "fflags": {"BENCH": FF_BENCH, "RECOVERY": FF_RECOVERY}},
        "navs": navs,
        "fcus": fcus,
        "decodes": decodes,
        "slave": scen,
    }


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
