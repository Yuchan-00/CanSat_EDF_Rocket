"""NAV <-> GNS link "glink" (v0.8.1 four-chip build) - the byte-exact reference, the GNS application cycle and the
normative per-tick timing of the GPS position-hold path.

Why (architect 2026-10-21 20:30, params ``flight_computer.architecture_v081``, ``gns``, ``glink``, ``electronics.gns``):
the v0.8 integration measured the THREE-chip GPS build as not flyable (NAV mandatory work 98.7 % mean / 119.5 % max of
the 4 ms tick, FCU time-critical 83.3 % / 89.2 % with ISRs, no ilink command decoded in position hold, FCU stack free
436 B, IMU -> vane up to 16.4 ms) while the no-GPS v0.8 build passed (IMU -> vane 4.23 ms, NAV tick -> vane 3.74 ms).
A FOURTH ATmega128A, the GNS board, now runs the GPS receiver (its USART1), UBX (``sim.gps_nav`` Part 1), the GPS-aided
horizontal filter (``sim.gps_nav.NavGpsFilter``) and the position controller (``sim.position_ctrl``) at 50 Hz; the NAV
keeps the attitude / altitude estimators and exchanges one STATE / RESULT pair per 50 Hz cycle with the GNS over glink;
the NAV puts the RESULT into the nlink v4 slot (``sim.nlink_proto``); the FCU applies the tilt setpoint and keeps the
supervisor rules V11-V15 on the slot data, without the position controller (``sim.onboard.FcuStack`` gns mode).  The
GNS makes no safety decision: its LAND requests are data the FCU's supervisor acts on, and the FCU detects a silent GNS
itself.

Wire: NAV USART1 (PD3 TXD1 -> GNS PE0 RXD0, GNS PE1 TXD0 -> NAV PD2 RXD1), 250 000 baud 8N1 (params glink.baud; UBRR 3
at 16 MHz: 16e6 / (16 x 4) = 250 000 exactly, 0 % error), interrupt driven on both ends (RX ISR -> ring, TX ISR from a
buffer; no blocking).  10 bit times per byte: 40 us per byte.

Frame: 0xA5 0x5A | TYPE | SEQ | PAYLOAD (fixed length per TYPE) | CRC-16/CCITT-FALSE lo hi over TYPE, SEQ and the payload
(``sim.link_proto.crc16_ccitt``, the CRC of ilink / nlink).  Little-endian, floats IEEE-754 binary32 (the NAV copies its
float32 values; nothing is re-quantised on the way to the GNS).

  STATE  (NAV -> GNS, TYPE 0x01, 54 B payload, frame 60 B = 2.40 ms), once per 50 Hz cycle m, SEQ = m & 0xFF:
      tick u32        the NAV tick k = 5 m + 4 of the sample (k since the NAV's boot)
      fsum[3] f32     body specific force summed over the cycle's ticks (float32 additions on the NAV, 3 per tick)
      n u8            ticks in fsum (5)
      q[4] f32        the CORRECTED attitude after tick k's correction part (sim.estimator: predict -> publish -> correct)
      w[3] f32        tick k's bias-corrected body rate (gyro - the corrected bias) - the lever arm and the guard
      h f32           the altitude filter after tick k (the position loop's engage height and touchdown band)
      nflags u8       NF_PAD 0x01 (the NAV estimator's pad mode, sim.nlink_proto.nav_pad_mode: the filter's pad mode) |
                      NF_FCU_OK 0x02 (a valid FCU frame in this cycle) | NF_IMU_OK 0x04 (tick k's IMU read succeeded) |
                      NF_EST_VALID 0x08 (the attitude comes from a pad alignment since the NAV boot)
      fcu_state u8    the supervisor state of the last valid FCU frame (sim.nlink_proto Fcu4)
      pflags u8       its pflags, with PF_SAT_HI = the OR over the valid FCU frames of the cycle (sat == +1 in any tick)
      kt_q u16        its k_T x 16384
  RESULT (GNS -> NAV, TYPE 0x02, 20 B payload, frame 26 B = 1.04 ms), SEQ = the STATE's SEQ (echo):
      the five 4-byte parts of the nlink v4 slot, in order: TILT, STATUS, POS, VEL, TRIM (``sim.nlink_proto`` module
      docstring "nlink v4"; the NAV copies them byte for byte into its frames).

Load: 3 000 B/s NAV -> GNS (12 % of the line), 1 300 B/s back (5.2 %).  Streaming decoder (``GlinkDecoder``, one per
receiver, bounded work per byte like ``sim.ilink_proto``): SYNC1 (0xA5, else skipped) -> SYNC2 (0x5A; another 0xA5 keeps
SYNC2; else SYNC1) -> TYPE (a known TYPE gives the payload length; else ``bad_type`` -> SYNC1) -> SEQ -> payload -> CRC lo
-> CRC hi (wrong: ``bad_crc`` -> SYNC1); a valid frame of the other direction's TYPE is counted ``wrong_dir`` and
dropped (a loop-back or a crossed lead).  No replay: a frame starting inside a rejected candidate is lost (the next 20 ms
cycle replaces it).

Per-tick timing (normative order; the numbers are ESTIMATES from the v0.8 C core's simavr cycle counts, the four-core
co-simulation measures them - params glink.latency_ticks_ASSUMED until then):

  NAV tick k = 5 m + 4 (j = k mod 5 = 4)
    ~0     tick start, IMU read, gyro prediction, nlink frame push: its slot = ``GnsSlotTx.slot(k)``
    ~3.6 ms  after the correction part (v0.8 integration: frame delivered 2.08 ms; Mahony correction 1.22 ms + altitude
           filter 0.28 ms, report_c-core-v08): fsum += this tick's force, STATE m sampled (q, w, h of tick k, the FCU
           fields of tick k's exchange: the FCU state of its tick k - 2), USART1 TX started (ISR) -> GNS RX complete
           +2.40 ms
  GNS on a valid STATE frame (its 20 ms cycle; order normative):
     1. gap rules (below); 2. ``NavGpsFilter.gns_step`` (the v0.8 NAV's j = 4 STEP: 0.536 + 0.690 ms max v0.8 C);
     3. ``PositionHold.step`` on the latched state (0.816 ms max for all pieces, v0.8 C); 4. RESULT encoded, TX started
     (1.04 ms); 5. ``NavGpsFilter.gns_fuse(tick + 1)`` (the v0.8 j = 0 fusion; up to ~2.0 ms) - after the TX, off the
     latency path.  Between frames: the UBX bytes, the boot configurator, the epoch assembler and the iTOW clock every
     4 ms (the epochs stamped with the GNS's NAV-tick clock nt: nt := STATE tick + 1 at each valid frame, +1 per 4 ms
     timer tick between frames; the reference polls on the NAV tick itself).
  RESULT at the NAV: ~ sample + 2.40 + ~2.35 (GNS decode + STEP + position step + encode) + 1.04 = ~ k start + 9.4 ms,
     i.e. ~1.4 ms into tick k + 2.  The NAV copies the slot during its frame build (~1.0 ms into a tick: the pre-push
     work 1.02 ms, v0.8 C) - k + 2 is missed by ~0.4 ms in this estimate, k + 3 has ~3.6 ms margin: L = 3.
  NAV tick k + L + i, i = 0..4: the frame carries part i of the RESULT of cycle m (TILT first; ``GnsSlotTx``; a RESULT
     not received by the frame build of k + L: the five parts of cycle m are SLOT_EMPTY).
  FCU tick k + L (its tick starts on that frame): the slot receiver takes the TILT (step (0)), the tilt-setpoint builder
     slews toward it at the end of the tick (step (7)), the attitude controller of FCU tick k + L + 1 flies it.  The
     set completes at k + L + 4: the supervisor sees its STATUS (gate, requests, TELEM) in that tick.

  So the NAV sample of tick k reaches the attitude controller at tick k + L + 1 = k + 4 (16 ms) with L = 3 - the v0.8
  three-chip path needed k + 10 (the five-frame slot set, the FCU's 4-tick job pipeline, the builder).  The FCU-side
  inputs are older (the FCU's tick k - 2 vs k + 5 in v0.8) and the GPS snapshot is the same (the STEP of tick k, before
  the epoch fusion of tick k + 1, as in v0.8).  ``sim.fc_timing.gns_chain_ticks`` gives both chains; the 6-DOF
  (``sim.sim6dof``, Scenario.gps.chain / glink_latency_ticks) flies this chain with L from params.

Sequence and staleness rules:
  NAV  takes a RESULT for cycle m only with SEQ == m & 0xFF, received after the cycle's sample tick and by the frame build
       of tick 5 m + 4 + L (``GnsSlotTx``: a 4-entry ring by SEQ); otherwise the five slots of cycle m are SLOT_EMPTY.
  GNS  dk = STATE tick - previous STATE tick: dk = 5 normal; dk = 5 (1 + j) with j > 0 lost frames: j hold steps of the
       position controller (outputs kept, loss counter and guard timer advance - the v0.8 "hold step"), then the filter
       STEP over dk ticks with this frame's mean force (the lost cycles' buffer entries stay stale: an epoch measured in
       the gap is dropped; a large innovation after it ends in the filter's position reset, whose offset keeps the hold
       point); dk <= 0 (a NAV reset: its tick clock restarted) or dk > ``GAP_RESTART_TICKS`` (10 s - the FCU has landed on
       its own V11 long before): ``NavGpsFilter.restart_timebase`` (re-initialised at the next usable epoch; the position
       loop flies DR / ATT on its loss counter meanwhile) and the iTOW clock relearns.  (A first version restarted beyond
       the 0.32 s buffer: a 0.5 s glink cut in a calm field hover then re-initialised p at the GPS's current wander and
       the vehicle moved to the shifted hold point - station keeping max 2.92 m; propagating keeps the reference.)
       A STATE without NF_IMU_OK is treated as lost (no propagation on frozen IMU data).  No STATE: no cycle, no RESULT.
  FCU  (``sim.nlink_proto.GnsSlotRx``): < 2 missed cycles: the last TILT; >= 2: the TRIM of the newest complete set
       (attitude only with the wind trim - the GPS-loss path's ATT, without its DR: the FCU has no position estimate);
       >= land_steps cycles (params gps_loss timeout_s + land_after_s; demo land_after 0): REQ_GPS_LOST raised by the FCU
       itself -> V11 LAND (the existing GPS-loss path; a GNS that still runs raises it itself on its own loss counter).

Physical limits stated: GPS needs open sky (no position hold indoors); the hold is to within the GPS wander (ASSUMED
Gauss-Markov ~2 m: ~1 m RMS / 2.5 m max until a real module is measured); no ground-effect model.

Invariant: ``encode_state`` / ``encode_result`` are the only producers of glink bytes; ``State.unpack(pack())`` returns
every float bit for bit; the decoders never return a frame whose sync, TYPE, length or CRC is wrong; the GNS cycle is a
pure function of the STATE frames and the UBX bytes it was given.
"""
from __future__ import annotations

import json
import math
import os
import struct
import sys
from dataclasses import asdict, dataclass, replace
from typing import Iterable, Sequence

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import gps_nav as gn
    from sim import link_proto as lp
    from sim import nlink_proto as nl
    from sim import position_ctrl as pc
except ImportError:  # pragma: no cover
    import gps_nav as gn  # type: ignore
    import link_proto as lp  # type: ignore
    import nlink_proto as nl  # type: ignore
    import position_ctrl as pc  # type: ignore

_HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.normpath(os.path.join(_HERE, "..", "params.json"))
VECTORS_PATH = os.path.join(_HERE, "out", "glink_vectors.json")

# --------------------------------------------------------------------------- #
# constants (== firmware/atmega128_fc/src/core/glink_proto.h, to be ported)
# --------------------------------------------------------------------------- #
SYNC0 = 0xA5
SYNC1 = 0x5A
TYPE_STATE = 0x01
TYPE_RESULT = 0x02
STATE_FMT = "<I3fB4f3ffBBBH"
STATE_LEN = struct.calcsize(STATE_FMT)                 # 54
RESULT_FMT = "<hhBBBBhhhhhh"
RESULT_LEN = struct.calcsize(RESULT_FMT)               # 20 = the five 4-byte slot parts
HEADER_BYTES = 4                                       # sync x2, TYPE, SEQ
STATE_FRAME_BYTES = HEADER_BYTES + STATE_LEN + 2       # 60
RESULT_FRAME_BYTES = HEADER_BYTES + RESULT_LEN + 2     # 26
PAYLOAD_LEN = {TYPE_STATE: STATE_LEN, TYPE_RESULT: RESULT_LEN}
BAUD = 250000                                          # params flight_computer.glink.baud
BITS_PER_BYTE = 10                                     # 8N1
NF_PAD = 0x01
NF_FCU_OK = 0x02
NF_IMU_OK = 0x04
NF_EST_VALID = 0x08
NF_NAMES = {NF_PAD: "pad", NF_FCU_OK: "fcu_ok", NF_IMU_OK: "imu_ok", NF_EST_VALID: "est_valid"}
STEP_TICKS = 5                                         # NAV ticks per 50 Hz cycle (control.loop_hz / position_hold.loop_hz)
GAP_RESTART_TICKS = 2500                               # 10 s: a longer STATE gap restarts the GNS filter's timebase (module
                                                       # docstring; shorter ones propagate with the frame's mean force)
CLOCK_SLIP_TICKS = 2                                   # GNS NAV-tick clock off by more than this at a STATE: iTOW relearns
LATENCY_TICKS_DEFAULT = 3                              # params glink.latency_ticks_ASSUMED (the value the code falls back to)

# timing ESTIMATES (module docstring; report_c-core-v08 simavr cycle counts at 16 MHz, report_integration-v08) - the
# four-core co-simulation replaces them
NAV_SAMPLE_MS_EST = 3.6           # NAV tick start -> STATE sampled (frame push 2.08 ms + Mahony 1.22 ms + altitude 0.28 ms)
NAV_FRAME_BUILD_MS_EST = 1.0      # NAV tick start -> the slot copied into the nlink frame (pre-push work 1.02 ms)
GNS_WORK_MS_EST = 2.35            # GNS: decode ~0.2 + STEP 1.23 (STEP-A 0.536 + STEP-B 0.690) + position 0.816 + encode
U8, U16, U32 = lp.U8, lp.U16, lp.U32


def crc16(data: Iterable[int], crc: int = lp.CRC_INIT) -> int:
    return lp.crc16_ccitt(data, crc)


def frame_time_s(nbytes: int, baud: int = BAUD) -> float:
    """Line time of ``nbytes`` at ``baud`` 8N1."""
    return nbytes * BITS_PER_BYTE / float(baud)


def _check(name: str, v: int, lim: tuple[int, int]) -> int:
    if not isinstance(v, int) or not lim[0] <= v <= lim[1]:
        raise ValueError(f"{name}={v!r} outside {lim}")
    return v


# --------------------------------------------------------------------------- #
# payloads
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class State:
    """gl_state_t (module docstring).  Floats hold float32 values (``pack`` rounds, ``unpack`` returns them exactly)."""

    tick: int
    fsum: tuple = (0.0, 0.0, 0.0)
    n: int = STEP_TICKS
    q: tuple = (1.0, 0.0, 0.0, 0.0)
    w: tuple = (0.0, 0.0, 0.0)
    h: float = 0.0
    nflags: int = 0
    fcu_state: int = 0
    pflags: int = 0
    kt_q: int = nl.KT_SCALE

    def __post_init__(self) -> None:
        _check("tick", self.tick, U32)
        for k in ("n", "nflags", "fcu_state", "pflags"):
            _check(k, getattr(self, k), U8)
        _check("kt_q", self.kt_q, U16)
        if len(self.fsum) != 3 or len(self.q) != 4 or len(self.w) != 3:
            raise ValueError("fsum 3, q 4, w 3 components")

    def pack(self) -> bytes:
        f = nl.f32
        return struct.pack(STATE_FMT, self.tick, *(f(x) for x in self.fsum), self.n, *(f(x) for x in self.q),
                           *(f(x) for x in self.w), f(self.h), self.nflags, self.fcu_state, self.pflags, self.kt_q)

    @classmethod
    def unpack(cls, payload: bytes) -> "State":
        if len(payload) != STATE_LEN:
            raise ValueError(f"STATE payload is {STATE_LEN} bytes, got {len(payload)}")
        v = struct.unpack(STATE_FMT, payload)
        return cls(v[0], tuple(v[1:4]), v[4], tuple(v[5:9]), tuple(v[9:12]), v[12], v[13], v[14], v[15], v[16])

    @property
    def phase(self) -> int:
        return self.pflags & nl.PF_PHASE_MASK

    @property
    def k_t(self) -> float:
        return nl.kt_value(self.kt_q)

    @property
    def cos_tilt(self) -> float:
        """1 - 2 (qx^2 + qy^2) of the corrected attitude (the guard's tilt on the GNS)."""
        return 1.0 - 2.0 * (self.q[1] * self.q[1] + self.q[2] * self.q[2])


@dataclass(frozen=True)
class Result:
    """gl_result_t: the five slot parts as wire integers (module docstring; ``sim.nlink_proto`` "nlink v4")."""

    tilt_x: int = nl.TILT_NONE
    tilt_y: int = 0
    flags: int = 0
    num_sv: int = 0
    hacc_dm: int = lp.HACC_NONE
    pc: int = 0
    pos_x_cm: int = 0
    pos_y_cm: int = 0
    vel_x_cms: int = 0
    vel_y_cms: int = 0
    trim_x: int = 0
    trim_y: int = 0

    def __post_init__(self) -> None:
        for k in ("tilt_x", "tilt_y", "pos_x_cm", "pos_y_cm", "vel_x_cms", "vel_y_cms", "trim_x", "trim_y"):
            _check(k, getattr(self, k), lp.I16)
        for k in ("flags", "num_sv", "hacc_dm", "pc"):
            _check(k, getattr(self, k), U8)

    def pack(self) -> bytes:
        return struct.pack(RESULT_FMT, self.tilt_x, self.tilt_y, self.flags, self.num_sv, self.hacc_dm, self.pc,
                           self.pos_x_cm, self.pos_y_cm, self.vel_x_cms, self.vel_y_cms, self.trim_x, self.trim_y)

    @classmethod
    def unpack(cls, payload: bytes) -> "Result":
        if len(payload) != RESULT_LEN:
            raise ValueError(f"RESULT payload is {RESULT_LEN} bytes, got {len(payload)}")
        return cls(*struct.unpack(RESULT_FMT, payload))

    def parts(self) -> list[bytes]:
        return nl.gns_slot_parts(self.pack())

    @property
    def engaged(self) -> bool:
        return self.tilt_x != nl.TILT_NONE


def _frame(typ: int, seq: int, payload: bytes) -> bytes:
    _check("seq", seq, U8)
    body = bytes([typ, seq]) + bytes(payload)
    c = crc16(body)
    return bytes([SYNC0, SYNC1]) + body + bytes([c & 0xFF, c >> 8])


def encode_state(seq: int, st: State) -> bytes:
    """gl_encode_state: the 60-byte STATE frame."""
    return _frame(TYPE_STATE, seq, st.pack())


def encode_result(seq: int, r: Result) -> bytes:
    """gl_encode_result: the 26-byte RESULT frame (SEQ = the STATE's SEQ)."""
    return _frame(TYPE_RESULT, seq, r.pack())


def reject_reason(buf: bytes | bytearray) -> str | None:
    """Why a whole buffer is not one valid frame ('length', 'sync', 'type', 'crc'), None if it is."""
    b = bytes(buf)
    if len(b) < HEADER_BYTES + 2:
        return "length"
    if b[0] != SYNC0 or b[1] != SYNC1:
        return "sync"
    if b[2] not in PAYLOAD_LEN:
        return "type"
    if len(b) != HEADER_BYTES + PAYLOAD_LEN[b[2]] + 2:
        return "length"
    end = len(b) - 2
    return None if (b[end] | (b[end + 1] << 8)) == crc16(b[2:end]) else "crc"


def decode_frame(buf: bytes | bytearray) -> tuple[int, int, bytes] | None:
    """(TYPE, SEQ, payload) of one whole valid frame, None otherwise."""
    if reject_reason(buf) is not None:
        return None
    b = bytes(buf)
    return b[2], b[3], b[HEADER_BYTES:-2]


# --------------------------------------------------------------------------- #
# streaming decoder (UART RX ring -> background, one per receiver)
# --------------------------------------------------------------------------- #
DEC_STAT_KEYS = ("frames", "bad_crc", "bad_type", "wrong_dir", "skipped")
_ST_SYNC1, _ST_SYNC2, _ST_TYPE, _ST_SEQ, _ST_PAY, _ST_CK1, _ST_CK2 = range(7)


class GlinkDecoder:
    """Byte-at-a-time decoder (module docstring).  ``accept`` = the TYPEs this end consumes (the NAV: RESULT, the GNS:
    STATE); ``feed`` returns the accepted frames as (TYPE, SEQ, payload)."""

    def __init__(self, accept: Sequence[int] = (TYPE_STATE, TYPE_RESULT)):
        self.accept = tuple(int(t) for t in accept)
        self.stats = {k: 0 for k in DEC_STAT_KEYS}
        self.st = _ST_SYNC1
        self.typ = 0
        self.seq = 0
        self.need = 0
        self.buf = bytearray()
        self.ck_lo = 0

    def _count(self, k: str) -> None:
        self.stats[k] = (self.stats[k] + 1) & 0xFFFF

    def feed_byte(self, b: int) -> tuple[int, int, bytes] | None:
        b &= 0xFF
        st = self.st
        if st == _ST_SYNC1:
            if b == SYNC0:
                self.st = _ST_SYNC2
            else:
                self._count("skipped")
        elif st == _ST_SYNC2:
            if b == SYNC1:
                self.st = _ST_TYPE
            elif b != SYNC0:
                self._count("skipped")
                self.st = _ST_SYNC1
        elif st == _ST_TYPE:
            if b in PAYLOAD_LEN:
                self.typ, self.need = b, PAYLOAD_LEN[b]
                self.st = _ST_SEQ
            else:
                self._count("bad_type")
                self.st = _ST_SYNC1
        elif st == _ST_SEQ:
            self.seq = b
            self.buf = bytearray()
            self.st = _ST_PAY
        elif st == _ST_PAY:
            self.buf.append(b)
            if len(self.buf) == self.need:
                self.st = _ST_CK1
        elif st == _ST_CK1:
            self.ck_lo = b
            self.st = _ST_CK2
        else:
            self.st = _ST_SYNC1
            if (self.ck_lo | (b << 8)) != crc16(bytes([self.typ, self.seq]) + bytes(self.buf)):
                self._count("bad_crc")
                return None
            if self.typ not in self.accept:
                self._count("wrong_dir")
                return None
            self._count("frames")
            return self.typ, self.seq, bytes(self.buf)
        return None

    def feed(self, data: Iterable[int]) -> list[tuple[int, int, bytes]]:
        out = []
        for b in data:
            r = self.feed_byte(b)
            if r is not None:
                out.append(r)
        return out


# --------------------------------------------------------------------------- #
# NAV side
# --------------------------------------------------------------------------- #
class GlinkNav:
    """The NAV's glink part (module docstring): per tick ``tick`` (correction part, after the altitude filter) sums the
    force and ORs the FCU frames' SAT_HI; at k mod 5 == 4 it returns the STATE frame for USART1.  ``rx_bytes`` feeds the
    bytes its USART1 received (RESULT frames -> ``nl.GnsSlotTx``); ``slot(k)`` is the slot of the nlink frame of tick k
    (called at the frame build, BEFORE that tick's ``tick``)."""

    def __init__(self, step_ticks: int = STEP_TICKS, latency_ticks: int = LATENCY_TICKS_DEFAULT):
        self.n = int(step_ticks)
        self.tx = nl.GnsSlotTx(self.n, latency_ticks)
        self.rx = GlinkDecoder(accept=(TYPE_RESULT,))
        self.fsum = [0.0, 0.0, 0.0]
        self.nsum = 0
        self.sat_or = False
        self.fcu_seen = False
        self.states = 0

    def slot(self, k: int) -> bytes:
        return self.tx.slot(k)

    def rx_bytes(self, data: bytes, k: int) -> int:
        """Bytes received on USART1 by the NAV's tick k (background decode); returns the RESULT frames taken."""
        n = 0
        for typ, seq, pl in self.rx.feed(data):
            self.tx.result(seq, pl, k)
            n += 1
        return n

    def tick(self, k: int, accel_b: Sequence[float], q_corr: Sequence[float], rate_b: Sequence[float], h: float,
             on_pad: bool, fcu: "nl.Fcu4 | None", fcu_valid: bool, imu_ok: bool = True,
             est_valid: bool = True) -> bytes | None:
        """NAV tick k: ``fcu`` = the last valid FCU frame (None before the first), ``fcu_valid`` = this tick's exchange
        returned a valid one.  Returns the STATE frame at k mod 5 == 4, else None."""
        f = nl.f32
        for i in range(3):
            self.fsum[i] = f(self.fsum[i] + f(float(accel_b[i])))
        self.nsum += 1
        if fcu_valid and fcu is not None:
            self.fcu_seen = True
            self.sat_or = self.sat_or or bool(fcu.pflags & nl.PF_SAT_HI)
        if k % self.n != self.n - 1:
            return None
        pf = 0 if fcu is None else (fcu.pflags & ~nl.PF_SAT_HI)
        if self.sat_or:
            pf |= nl.PF_SAT_HI
        nf = ((NF_PAD if on_pad else 0) | (NF_FCU_OK if self.fcu_seen else 0) | (NF_IMU_OK if imu_ok else 0)
              | (NF_EST_VALID if est_valid else 0))
        st = State(tick=int(k) & 0xFFFFFFFF, fsum=tuple(self.fsum), n=min(self.nsum, 255),
                   q=tuple(float(x) for x in q_corr), w=tuple(float(x) for x in rate_b), h=float(h), nflags=nf,
                   fcu_state=0 if fcu is None else fcu.state, pflags=pf, kt_q=nl.KT_SCALE if fcu is None else fcu.kt_q)
        self.fsum = [0.0, 0.0, 0.0]
        self.nsum = 0
        self.sat_or = False
        self.fcu_seen = False
        self.states += 1
        return encode_state((k // self.n) & 0xFF, st)


# --------------------------------------------------------------------------- #
# GNS side: the application cycle
# --------------------------------------------------------------------------- #
def result_from(po: "pc.PosOutputs", flt: "gn.NavGpsFilter") -> Result:
    """The RESULT of one GNS cycle from the position controller's outputs and the filter after its STEP (module
    docstring; quantised once here, copied by the NAV)."""
    snap = flt.snap
    if po.engaged:
        tx, ty = nl.tilt_q(po.tilt_l[0]), nl.tilt_q(po.tilt_l[1])
        rx_, ry_ = nl.tilt_q(po.trim_l[0]), nl.tilt_q(po.trim_l[1])
    else:
        tx, ty, rx_, ry_ = nl.TILT_NONE, 0, 0, 0
    hold = po.hold is not None
    px, py = flt.p[0], flt.p[1]
    if hold:
        px, py = px - po.hold[0], py - po.hold[1]
    return Result(tilt_x=tx, tilt_y=ty, flags=snap.flags & 0xFF, num_sv=min(int(snap.num_sv), 255),
                  hacc_dm=min(int(snap.hacc_dm), 255), pc=nl.pc_byte(po.mode, hold, po.requests & 0x0F),
                  pos_x_cm=gn.quant_cm(px), pos_y_cm=gn.quant_cm(py), vel_x_cms=gn.quant_cm(flt.v[0]),
                  vel_y_cms=gn.quant_cm(flt.v[1]), trim_x=rx_, trim_y=ry_)


def gps_data_from(flt: "gn.NavGpsFilter") -> "pc.GpsData":
    """The position controller's GPS data on the GNS: the filter's float state at its latch (no slot quantisation)."""
    s = flt.snap
    return pc.GpsData(flags=s.flags, num_sv=s.num_sv, hacc_dm=s.hacc_dm, reset_count=flt.reset_count & 0xFF,
                      off=(flt.off[0], flt.off[1]), p=(flt.p[0], flt.p[1]), v=(flt.v[0], flt.v[1]),
                      cos_psi=flt.c_psi, sin_psi=flt.s_psi, cycle=flt.cycle)


def pos_inputs_from(st: State, data: "pc.GpsData | None") -> "pc.PosInputs":
    """The position controller's inputs from a STATE frame (the FCU fields as the NAV last received them)."""
    pf = st.pflags
    return pc.PosInputs(phase=pf & nl.PF_PHASE_MASK, motor_on=bool(pf & nl.PF_MOTOR_ON), h_m=float(st.h),
                        cos_tilt=st.cos_tilt, rate=tuple(st.w), k_t=st.k_t, vc_sat_high=bool(pf & nl.PF_SAT_HI),
                        data=data, vc_low_alt=bool(pf & nl.PF_LOW_ALT), vc_ground=bool(pf & nl.PF_GROUND),
                        lifted=bool(pf & nl.PF_LIFTED))


class GnsNode:
    """The GNS board (module docstring): ``poll`` every 4 ms (= NAV tick k in the reference) with the bytes its USART1
    (GPS) received - returns the configurator's UART actions; ``rx_bytes`` with the bytes its USART0 (glink) received -
    returns the RESULT frames to send.  ``on_state`` is one 50 Hz cycle.  ``filter`` / ``pos`` / ``parser`` / ``asm`` /
    ``clock`` / ``cfg`` are the v0.8 NAV objects (``sim.gps_nav.NavGps``) and the position controller."""

    def __init__(self, gp: "gn.NavGpsParams", pp: "pc.PositionParams", configure: bool = True):
        if gp.step_ticks != pp.step_ticks:
            raise ValueError("the GPS fusion rate and the position loop rate must be the same 50 Hz cycle")
        self.gp, self.pp = gp, pp
        self.n = gp.step_ticks
        self.parser = gn.UbxParser()
        self.asm = gn.EpochAssembler()
        self.clock = gn.ItowClock(gp.ms_per_tick, gp.latency_ms)
        self.filter = gn.NavGpsFilter(gp)
        self.cfg = gn.GpsConfigurator(gp.loop_hz, gp.rate_hz) if configure else None
        if not configure:
            self.filter.cfg_ok = True
        self.pos = pc.PositionHold(pp)
        self.dec = GlinkDecoder(accept=(TYPE_STATE,))
        self.nt: int | None = None                 # the GNS's NAV-tick clock
        self.k_state: int | None = None            # tick of the last STATE frame used
        self.last: Result | None = None
        self.last_state: State | None = None
        self.cycles = 0
        self.stats = {"states": 0, "hold_steps": 0, "gaps": 0, "restarts": 0, "imu_bad": 0, "clock_slips": 0,
                      "misaligned": 0}

    # ---- every 4 ms ----
    def poll(self, k: int, ubx: bytes) -> list:
        self.nt = int(k)
        msgs = []
        for m in self.parser.feed(ubx):
            d = gn.decode_msg(m)
            if d is None:
                continue
            msgs.append(d)
            ep = self.asm.add(d, k)
            if ep is not None:
                self.filter.pending.append((ep, self.clock.update(ep.itow_ms, k)))
        acts = []
        if self.cfg is not None:
            acts = self.cfg.tick(msgs)
            self.filter.cfg_ok = self.cfg.cfg_ok
            self.filter.ver_ok = self.cfg.ver_ok
        return acts

    def rx_bytes(self, data: bytes) -> list[bytes]:
        out = []
        for typ, seq, pl in self.dec.feed(data):
            out.append(self.on_state(seq, State.unpack(pl)))
        return out

    def pos_out(self) -> "pc.PosOutputs":
        return self.pos.out

    # ---- one 50 Hz cycle ----
    def on_state(self, seq: int, st: State) -> bytes:
        n = self.n
        k = int(st.tick)
        self.stats["states"] += 1
        # the GNS's NAV-tick clock: the frame arrives in the NAV's tick k + 1 (C port: nt := k + 1 at each frame)
        if self.nt is not None and abs(self.nt - (k + 1)) > CLOCK_SLIP_TICKS:
            self.stats["clock_slips"] += 1
            self.clock.offset = None                 # the iTOW offset was learnt on the old clock: relearn
        self.nt = k + 1
        pin = pos_inputs_from(st, None)
        imu_ok = bool(st.nflags & NF_IMU_OK)
        if self.k_state is None:
            dk = n
        else:
            dk = k - self.k_state
            if dk <= 0 or dk > GAP_RESTART_TICKS:
                self.stats["restarts"] += 1
                self.filter.restart_timebase()
                self.clock.offset = None
                dk = n
            elif dk % n:
                self.stats["misaligned"] += 1
        if not imu_ok:
            # frozen IMU data: like a lost frame (a hold step; the next good frame propagates over the gap)
            self.stats["imu_bad"] += 1
            self.stats["hold_steps"] += 1
            self.pos.step(pin)
            self.last_state = st
            return encode_result(seq, self.last if self.last is not None else Result())
        missed = max(dk // n - 1, 0)
        if missed:
            self.stats["gaps"] += 1
        for _ in range(missed):
            self.stats["hold_steps"] += 1
            self.pos.step(pin)
        self.k_state = k
        nf = st.n if st.n > 0 else n
        scale = dk / nf
        acc = [x * scale for x in st.fsum] if scale != 1.0 else list(st.fsum)
        self.filter.gns_step(k, acc, dk, st.q, st.w, bool(st.nflags & NF_PAD))
        po = self.pos.step(replace(pin, data=gps_data_from(self.filter)))
        r = result_from(po, self.filter)
        self.last = r
        self.last_state = st
        self.cycles += 1
        frame = encode_result(seq, r)
        self.filter.gns_fuse(k + 1)
        return frame


# --------------------------------------------------------------------------- #
# params
# --------------------------------------------------------------------------- #
def glink_params(p: dict) -> dict:
    """params flight_computer.glink (baud, ubrr, latency_ticks_ASSUMED) and the derived frame times."""
    g = p["flight_computer"]["glink"]
    baud = int(g["baud"])
    f_cpu = int(p["flight_computer"].get("gns", {}).get("f_cpu_hz", p["flight_computer"]["f_cpu_hz"]))
    ubrr = int(g["ubrr"])
    return {"baud": baud, "ubrr": ubrr, "baud_actual": f_cpu / (16.0 * (ubrr + 1)),
            "latency_ticks": int(g.get("latency_ticks_ASSUMED", LATENCY_TICKS_DEFAULT)),
            "state_frame_s": frame_time_s(STATE_FRAME_BYTES, baud), "result_frame_s": frame_time_s(RESULT_FRAME_BYTES, baud)}


def timing_estimate(p: dict) -> dict:
    """The module docstring's arrival estimate (ms after the start of the sample tick k) and the earliest feasible L."""
    gp = glink_params(p)
    tick_ms = 1000.0 / int(p["control"]["loop_hz"])
    arrive = NAV_SAMPLE_MS_EST + 1e3 * gp["state_frame_s"] + GNS_WORK_MS_EST + 1e3 * gp["result_frame_s"]
    L_min = 1
    while L_min * tick_ms + NAV_FRAME_BUILD_MS_EST <= arrive:
        L_min += 1
    return {"tick_ms": tick_ms, "sample_ms": NAV_SAMPLE_MS_EST, "state_frame_ms": 1e3 * gp["state_frame_s"],
            "gns_work_ms": GNS_WORK_MS_EST, "result_frame_ms": 1e3 * gp["result_frame_s"], "result_at_nav_ms": arrive,
            "frame_build_ms": NAV_FRAME_BUILD_MS_EST, "latency_ticks_min_est": L_min,
            "latency_ticks_params": gp["latency_ticks"],
            "margin_ms_at_params_L": gp["latency_ticks"] * tick_ms + NAV_FRAME_BUILD_MS_EST - arrive,
            "note": "ESTIMATE from the v0.8 C core cycle counts (report_c-core-v08) and the v0.8 integration's frame time; "
                    "the four-core co-simulation measures it"}


# --------------------------------------------------------------------------- #
# test vectors for the C port (sim/out/glink_vectors.json)
# --------------------------------------------------------------------------- #
def _state_fields(st: State) -> dict:
    pl = st.pack()
    u = struct.unpack("<I3IB4I3IIBBBH", pl)
    return {"tick": u[0], "fsum_bits": list(u[1:4]), "n": u[4], "q_bits": list(u[5:9]), "w_bits": list(u[9:12]),
            "h_bits": u[12], "nflags": u[13], "fcu_state": u[14], "pflags": u[15], "kt_q": u[16]}


def _result_fields(r: Result) -> dict:
    return dict(asdict(r))


def make_test_vectors() -> dict:
    """Deterministic vectors (own RNG): STATE / RESULT payloads and frames, decodes (valid, every single-bit error of each
    frame, wrong TYPE / length), streaming-decoder byte streams, the NAV's slot phase (GnsSlotTx) and the FCU's slot
    receiver (GnsSlotRx) on hand-made sequences, the FCU4 frame, and a GNS cycle replay (synthetic IMU + UBX epochs ->
    RESULT frames) on the module-constant parameters listed in the vector."""
    import random
    rng = random.Random(20261021)
    states, results = [], []

    def add_state(name: str, seq: int, st: State, note: str = "") -> None:
        states.append({"name": name, "note": note, "seq": seq, "fields": _state_fields(st),
                       "payload_hex": st.pack().hex(), "frame_hex": encode_state(seq, st).hex()})

    def add_result(name: str, seq: int, r: Result, note: str = "") -> None:
        results.append({"name": name, "note": note, "seq": seq, "fields": _result_fields(r),
                        "payload_hex": r.pack().hex(), "frame_hex": encode_result(seq, r).hex(),
                        "parts_hex": [x.hex() for x in r.parts()]})

    pf_hold = nl.fcu4_pflags(lp.PHASE_HOLD, True, False, False, False, True)
    add_state("pad", 0, State(4, (0.11, -0.24, 49.03), 5, (1.0, 0.0, 0.0, 0.0), (0.001, -0.002, 0.0003), 0.0,
                              NF_PAD | NF_FCU_OK | NF_IMU_OK | NF_EST_VALID, lp.STATE_SAFE, 0, nl.KT_SCALE))
    add_state("hover", 201, State(1009, (1.21, -0.66, 48.71), 5, (0.99965, -0.0125, 0.0213, 0.0087), (0.12, -0.31, 1.5),
                                  5.03, NF_FCU_OK | NF_IMU_OK | NF_EST_VALID, lp.STATE_POWERED,
                                  pf_hold | nl.PF_SAT_HI, nl.kt_quant(0.9731)))
    add_state("wrap", 255, State(0xFFFFFFFF, (-1e-30, 3.4e38, -0.0), 5, (0.7071, 0.0, 0.7071, 0.0), (34.9, -34.9, 0.0),
                                 -3.5, 0xFF, 0xFF, 0xFF, 0xFFFF), note="extremes: tick u32 max, every flag, big floats")
    for i in range(3):
        q = [rng.uniform(-1, 1) for _ in range(4)]
        nq = math.sqrt(sum(x * x for x in q))
        add_state(f"random_{i}", rng.randrange(256),
                  State(rng.randrange(1 << 32), tuple(rng.uniform(-200, 200) for _ in range(3)), rng.randrange(1, 9),
                        tuple(x / nq for x in q), tuple(rng.uniform(-10, 10) for _ in range(3)), rng.uniform(-5, 40),
                        rng.randrange(16), rng.randrange(8), rng.randrange(256), rng.randrange(65536)))
    add_result("not_engaged", 0, Result(flags=gn.GF_CFG_OK, num_sv=4, hacc_dm=255), "TILT x = -32768 (TILT_NONE)")
    add_result("hold", 201, Result(tilt_x=nl.tilt_q(0.0312), tilt_y=nl.tilt_q(-0.0871),
                                   flags=gn.GF_FIX_OK | gn.GF_GATE_OK | gn.GF_HEAD_OK | gn.GF_FIX3D | gn.GF_CFG_OK
                                   | gn.GF_INIT | gn.GF_VER_OK, num_sv=9, hacc_dm=25,
                                   pc=nl.pc_byte(pc.MODE_POS, True, 0), pos_x_cm=-37, pos_y_cm=52, vel_x_cms=-8,
                                   vel_y_cms=11, trim_x=nl.tilt_q(0.0205), trim_y=nl.tilt_q(-0.0611)))
    add_result("att_requests", 17, Result(tilt_x=nl.tilt_q(0.0205), tilt_y=nl.tilt_q(-0.0611), flags=gn.GF_INIT,
                                          num_sv=3, hacc_dm=200,
                                          pc=nl.pc_byte(pc.MODE_ATT, True, pc.REQ_GPS_LOST | pc.REQ_IMPLAUSIBLE),
                                          pos_x_cm=32767, pos_y_cm=-32767, vel_x_cms=-32768, vel_y_cms=0,
                                          trim_x=nl.tilt_q(0.0205), trim_y=nl.tilt_q(-0.0611)),
               "ATT mode (TILT = TRIM), requests GPS_LOST | IMPLAUSIBLE, saturated POS / VEL")
    decodes = []

    def dec(name: str, buf: bytes, note: str = "") -> None:
        r = decode_frame(buf)
        decodes.append({"name": name, "note": note, "hex": buf.hex(), "valid": r is not None,
                        "reason": reject_reason(buf) or "",
                        **({"type": r[0], "seq": r[1], "payload_hex": r[2].hex()} if r is not None else {})})

    s0 = bytes.fromhex(states[1]["frame_hex"])
    r0 = bytes.fromhex(results[1]["frame_hex"])
    dec("valid_state", s0)
    dec("valid_result", r0)
    for name, base in (("state", s0), ("result", r0)):
        for j in range(len(base)):
            for bit in range(8):
                b = bytearray(base)
                b[j] ^= 1 << bit
                dec(f"bitflip_{name}_b{j}_{bit}", bytes(b))
    dec("unknown_type", s0[:2] + bytes([0x03]) + s0[3:], "TYPE 3 is not a glink TYPE")
    dec("truncated_state", s0[:-1], "one byte short")
    dec("state_type_with_result_length", s0[:2] + bytes([TYPE_RESULT]) + s0[3:], "TYPE 2 with 54 payload bytes: length")
    streams = []

    def stream(name: str, data: bytes, accept: tuple, note: str = "") -> None:
        d = GlinkDecoder(accept)
        fr = d.feed(data)
        streams.append({"name": name, "note": note, "accept": list(accept), "hex": data.hex(),
                        "frames": [{"type": t, "seq": s, "payload_hex": p.hex()} for t, s, p in fr],
                        "stats": dict(d.stats)})

    s1 = bytes.fromhex(states[0]["frame_hex"])
    bad = bytearray(s0)
    bad[20] ^= 0x10
    stream("two_states", s1 + s0, (TYPE_STATE,))
    stream("garbage_then_state", bytes([0x00, 0xA5, 0x00, 0x5A, 0xFF, 0xA5]) + s0, (TYPE_STATE,),
           "skipped bytes; a lone 0xA5 then 0x00 resyncs")
    stream("a5_a5_5a", bytes([0xA5]) + s0, (TYPE_STATE,), "an extra 0xA5 before the sync keeps SYNC2")
    stream("bad_crc_then_good", bytes(bad) + s1, (TYPE_STATE,), "CRC error: the next frame is taken")
    stream("result_at_the_gns", r0 + s0, (TYPE_STATE,), "a RESULT reaching the GNS (crossed lead): wrong_dir")
    stream("results_at_the_nav", r0 + bytes.fromhex(results[2]["frame_hex"]), (TYPE_RESULT,))
    stream("split_frame", s0[:17] + s0[17:], (TYPE_STATE,), "frame split across two reads (same bytes)")
    stream("bad_type", bytes([0xA5, 0x5A, 0x07, 0x00]) + s0, (TYPE_STATE,), "unknown TYPE: resync, the frame after it")
    stream("lost_byte", s0[:30] + s0[31:] + s1, (TYPE_STATE,),
           "byte 30 lost: the decoder runs into the next frame (no replay): that frame is lost too")
    # NAV slot phase (GnsSlotTx, L = 3): results for cycles 0..3 arrive at k + 1, cycle 2's is missing
    tx_cases = []

    def run_tx(name: str, L: int, events: list, ticks: int, note: str = "") -> None:
        t = nl.GnsSlotTx(STEP_TICKS, L)
        ev = {int(e["k"]): e for e in events}
        out = []
        for k in range(ticks):
            if k in ev:
                e = ev[k]
                t.result(e["seq"], bytes.fromhex(e["payload_hex"]), k)
            out.append(t.slot(k).hex())
        tx_cases.append({"name": name, "note": note, "L": L, "events": events, "slots": out,
                         "latched": t.latched, "missed": t.missed})

    pl_hold = results[1]["payload_hex"]
    pl_att = results[2]["payload_hex"]
    ev_norm = [{"k": 5 * m + 5, "seq": m & 0xFF, "payload_hex": pl_hold if m % 2 == 0 else pl_att} for m in range(5)
               if m != 2]
    run_tx("L3_cycle2_missing", 3, ev_norm, 40, "results at k + 1 of each sample; cycle 2's never came: SLOT_EMPTY x 5")
    run_tx("L2", 2, ev_norm, 40, "the earliest phase")
    ev_late = [{"k": 5 * m + 5 + (3 if m == 1 else 0), "seq": m & 0xFF, "payload_hex": pl_hold} for m in range(4)]
    run_tx("L3_late_result", 3, ev_late, 36, "cycle 1's result arrives at k + 4, after its TILT frame (k + 3): not sent")
    run_tx("L12_ring", 12, ev_norm, 60, "a long latency: the 4-entry ring keeps the results until their frames")
    ev_wrong = [{"k": 10, "seq": 7, "payload_hex": pl_hold}]
    run_tx("wrong_seq", 3, ev_wrong, 20, "a result with a SEQ of no due cycle is never sent")
    # FCU slot receiver (GnsSlotRx)
    rx_cases = []
    parts_hold = [bytes.fromhex(x) for x in results[1]["parts_hex"]]
    parts_none = [bytes.fromhex(x) for x in results[0]["parts_hex"]]

    def sl(m: int, i: int, parts: list) -> str:
        return (bytes([((m & 0x1F) << 3) | i]) + parts[i]).hex()

    def run_rx(name: str, seq: list, land_steps: int = 4, note: str = "") -> None:
        r = nl.GnsSlotRx(STEP_TICKS)
        out = []
        for s in seq:
            g = r.feed(None if s is None else bytes.fromhex(s))
            eng, tgt = r.target()
            out.append({"set": None if g is None else {"flags": g.flags, "num_sv": g.num_sv, "hacc_dm": g.hacc_dm,
                                                        "mode": g.mode, "hold": g.hold, "requests": g.requests,
                                                        "pos": list(g.pos), "vel": list(g.vel), "trim": list(g.trim),
                                                        "tilt": list(g.tilt), "engaged": g.engaged, "cycle": g.cycle},
                        "engaged": eng, "target": list(tgt), "fallback": r.fallback(),
                        "requests": r.requests(land_steps), "gate_ok": r.gate_ok})
        rx_cases.append({"name": name, "note": note, "land_steps": land_steps, "slots": seq, "out": out,
                         "sets": r.sets, "broken": r.broken})

    c5 = [sl(5, i, parts_hold) for i in range(5)]
    c6 = [sl(6, i, parts_hold) for i in range(5)]
    run_rx("complete", c5 + c6)
    run_rx("not_engaged", [sl(1, i, parts_none) for i in range(5)])
    run_rx("lost_tilt_frame", [None] + c5[1:] + c6, note="cycle 5's TILT lost: set broken, the old target held")
    run_rx("gns_silent", c5 + [nl.SLOT_EMPTY.hex()] * 30, note="the GNS stops: hold 1 cycle, then TRIM; requests from "
                                                                 "land_steps (4 here) cycles")
    run_rx("empty_then_cycle", [nl.SLOT_EMPTY.hex()] * 3 + c5)
    # FCU4 frames
    fcu4 = []
    for name, f in (("pad", nl.Fcu4(3, lp.STATE_SAFE, 0, 0x5A, 0, 0, nl.KT_SCALE)),
                    ("hold", nl.Fcu4(201, lp.STATE_POWERED, 0, 0x5A, 172, pf_hold, nl.kt_quant(1.0731))),
                    ("land_ground", nl.Fcu4(77, lp.STATE_POWERED, 0, 0x5A, 60,
                                            nl.fcu4_pflags(lp.PHASE_LAND, True, False, True, True, True),
                                            nl.kt_quant(1.4))),
                    ("all", nl.Fcu4(255, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFFFF))):
        buf = nl.encode_fcu4(f)
        fcu4.append({"name": name, "fields": dict(vars(f)), "hex": buf.hex(), "valid": nl.decode_fcu4(buf) is not None})
    return {"_doc": ("glink (NAV <-> GNS, v0.8.1) vectors generated by sim/glink_proto.py make_test_vectors(). states / "
                     "results: fields (floats as float32 bit patterns) <-> payload <-> frame (+ the five slot parts of a "
                     "RESULT); decodes: one whole buffer each with the expected gl_decode result; streams: GlinkDecoder byte "
                     "streams with the accepted frames and the counters; slot_tx: GnsSlotTx (NAV) slot bytes per NAV tick "
                     "for result events; slot_rx: GnsSlotRx (FCU) outputs per FCU tick; fcu4: nlink v4 FCU halves; "
                     "gns_replay: GnsNode cycles (configure off) on a synthetic stream - RESULT frames to match."),
            "constants": {"SYNC": [SYNC0, SYNC1], "TYPE_STATE": TYPE_STATE, "TYPE_RESULT": TYPE_RESULT,
                          "STATE_LEN": STATE_LEN, "RESULT_LEN": RESULT_LEN, "STATE_FRAME_BYTES": STATE_FRAME_BYTES,
                          "RESULT_FRAME_BYTES": RESULT_FRAME_BYTES, "BAUD": BAUD, "NF": {v: k for k, v in NF_NAMES.items()},
                          "GAP_RESTART_TICKS": GAP_RESTART_TICKS, "CLOCK_SLIP_TICKS": CLOCK_SLIP_TICKS,
                          "TYPE_NAV4": nl.TYPE_NAV4, "TYPE_FCU4": nl.TYPE_FCU4, "FCU4_FRAME_BYTES": nl.FCU4_FRAME_BYTES,
                          "PF": {"PHASE_MASK": nl.PF_PHASE_MASK, "MOTOR_ON": nl.PF_MOTOR_ON, "SAT_HI": nl.PF_SAT_HI,
                                 "LOW_ALT": nl.PF_LOW_ALT, "GROUND": nl.PF_GROUND, "LIFTED": nl.PF_LIFTED},
                          "KT_SCALE": nl.KT_SCALE, "TILT_SCALE": nl.TILT_SCALE, "TILT_NONE": nl.TILT_NONE,
                          "PC": {"MODE_MASK": nl.PC_MODE_MASK, "HOLD": nl.PC_HOLD, "REQ_SHIFT": nl.PC_REQ_SHIFT},
                          "SLOT_EMPTY": nl.SLOT_EMPTY.hex(), "GNS_SLOT_RING": nl.GnsSlotTx.RING},
            "crc16_check": {"ascii": "123456789", "crc": crc16(b"123456789")},
            "states": states, "results": results, "decodes": decodes, "streams": streams, "slot_tx": tx_cases,
            "slot_rx": rx_cases, "fcu4": fcu4, "gns_replay": gns_replay_vector()}


REPLAY_T_FLY_TICKS = 500           # gns_replay_vector: pad (SAFE) before, POWERED CLIMB after
REPLAY_T_HOLD_TICKS = 700          # ... HOLD from here
REPLAY_VEL_ERR_TICK = 875          # ... a 1 m/s north GPS velocity error from here (rejections -> a velocity reset)


def gns_replay_vector(n_ticks: int = 1500) -> dict:
    """A GNS replay for the C crosscheck in the normative order: module-constant NavGpsParams / PositionParams (listed),
    configure off; per NAV tick k: ``poll(k, UBX bytes of tick k)``, then - if the NAV sent a STATE in tick k - 1 - that
    frame (``rx_bytes``), whose RESULT is recorded.  Synthetic plant: a straight-line drift (truth x = 0.2 t, y = -0.1 t,
    no acceleration, level, no rotation) with exact UBX epochs every 200 ms whose last byte arrives 0.22 s after the
    measurement; pad (SAFE) to tick 500, then POWERED CLIMB at h 2 m, HOLD from tick 700, a 1 m/s north velocity error
    in the GPS from tick 875.  Floats in the internals are float64 here: compare within float32."""
    gp = gn.NavGpsParams()
    pp = pc.PositionParams()
    node = GnsNode(gp, pp, configure=False)
    lat0, lon0 = 375000000, 1270000000
    lf = gn.LocalFrame.at(lat0, lon0)
    n = node.n
    ticks_per_epoch = int(round(gp.loop_hz / gp.rate_hz))
    lat_ticks = int(round(gp.latency_s * gp.loop_hz))
    pend: dict[int, bytes] = {}
    ubx_by_tick: dict[str, str] = {}
    states: list[dict] = []
    sent: bytes | None = None
    for k in range(n_ticks):
        t = k / gp.loop_hz
        if k % ticks_per_epoch == 0:                                # a measurement at t, complete lat_ticks later
            vx_err = 1.0 if k >= REPLAY_VEL_ERR_TICK else 0.0
            lat, lon = lf.to_llh(0.2 * t, -0.1 * t)
            ep = gn.GpsEpoch(itow_ms=300000000 + int(round(t * 1000)), lat_e7=lat, lon_e7=lon, hmsl_mm=10000,
                             hacc_mm=2000, vacc_mm=3000, vel_n_cms=int(round((0.2 + vx_err) * 100)), vel_e_cms=10,
                             vel_d_cms=0, sacc_cms=20, gps_fix=3, fix_ok=True, num_sv=9, pacc_cm=250)
            pend[k + lat_ticks] = gn.epoch_frames(ep)
        ubx = pend.pop(k, b"")
        if ubx:
            ubx_by_tick[str(k)] = ubx.hex()
        node.poll(k, ubx)
        if sent is not None:
            rf = node.rx_bytes(sent)[0]
            po, f = node.pos_out(), node.filter
            states[-1].update(result_hex=rf.hex(), p=list(f.p), v=list(f.v), b=list(f.b), mode=po.mode,
                              tilt_l=list(po.tilt_l), trim_l=list(po.trim_l), requests=po.requests,
                              reset_count=f.reset_count)
            sent = None
        if k % n == n - 1:
            flying = k >= REPLAY_T_FLY_TICKS
            phase = lp.PHASE_HOLD if k >= REPLAY_T_HOLD_TICKS else (lp.PHASE_CLIMB if flying else lp.PHASE_NONE)
            st = State(tick=k, fsum=(0.0, 0.0, n * 9.80665), n=n, q=(1.0, 0.0, 0.0, 0.0), w=(0.0, 0.0, 0.0),
                       h=2.0 if flying else 0.0, nflags=(0 if flying else NF_PAD) | NF_FCU_OK | NF_IMU_OK | NF_EST_VALID,
                       fcu_state=lp.STATE_POWERED if flying else lp.STATE_SAFE,
                       pflags=nl.fcu4_pflags(phase, flying, False, False, False, True), kt_q=nl.KT_SCALE)
            sent = encode_state((k // n) & 0xFF, st)
            states.append({"tick": k, "state_hex": sent.hex()})
    return {"nav_params": {k_: (list(v_) if isinstance(v_, tuple) else v_) for k_, v_ in asdict(node.gp).items()},
            "pos_params": asdict(node.pp), "local_frame_e7": [lat0, lon0], "configure": False,
            "order": "per tick k: poll(k, ubx_by_tick[k] or nothing); then the STATE sent in tick k - 1, if any",
            "ubx_by_tick": ubx_by_tick, "states": [s for s in states if "result_hex" in s],
            "stats": dict(node.stats), "filter_stats": dict(node.filter.stats)}


def write_test_vectors(path: str = VECTORS_PATH) -> dict:
    v = make_test_vectors()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(v, f, indent=1)
        f.write("\n")
    return v


if __name__ == "__main__":  # pragma: no cover
    _v = write_test_vectors()
    sys.stdout.write(f"wrote {os.path.relpath(VECTORS_PATH, os.path.join(_HERE, '..'))}: {len(_v['states'])} STATE, "
                     f"{len(_v['results'])} RESULT, {len(_v['decodes'])} decodes, {len(_v['streams'])} streams, "
                     f"{len(_v['slot_tx'])} slot_tx, {len(_v['slot_rx'])} slot_rx, {len(_v['gns_replay']['states'])} "
                     f"replay cycles\n")
