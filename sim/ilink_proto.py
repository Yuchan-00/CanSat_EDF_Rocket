"""FCU <-> COM inter-processor link (SPEC 10.4, v0.6) - the byte-exact reference.

v0.6 splits the flight computer over two ATmega128A (params.json
``flight_computer.architecture``): the FCU flies (sensors, estimator, onboard
supervisor, attitude control, PWM, log) and the COM runs the LoRa link (radio
driver, uplink decoding with ``sim.link_proto``, TELEM building).  This module is
the normative definition of the wire format between them; the firmware port is
``firmware/atmega128_fc/src/core/ilink_proto.c`` (``ilink_proto.h``).  Pure Python
(no numpy), like ``sim.link_proto``.

Wire: FCU USART1 <-> COM USART0, ``flight_computer.ilink.baud`` (250 000) 8N1.

Frame::

    0xC5 0x5C | LEN | TYPE | PAYLOAD[LEN] | CRC16 lo | CRC16 hi
                ^------- CRC16 over LEN, TYPE, PAYLOAD -------^

CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR) - the
same CRC as the radio link (``sim.link_proto.crc16_ccitt``).  Every TYPE has one
fixed payload length; multi-byte fields are little-endian, floats IEEE-754 binary32
little-endian (struct ``'<f'``, the AVR's ``float``).

* ``TYPE_CMD``  (0x01, COM -> FCU, 8 B ``<BBHHbb``): seq, cmd, arg, nonce, rssi_dbm
  (``RSSI_NONE`` = 127 when the radio reports none), snr_q4 - one uplink CMD frame the
  COM's ``link_proto`` decoder accepted, in arrival order.  The FCU hands it to the
  supervisor exactly as the v0.5 single-chip firmware did with a frame it decoded.
* ``TYPE_SNAP`` (0x02, FCU -> COM, 49 B ``<BBBBBHHIfffff4f``): reply_seq, state,
  flags, reason, ack_seq, nonce, throttle_milli, tick, h_m, vz_mps, tilt_deg, v_cell,
  loop_max_us, q[4] (w, x, y, z) - the supervisor's reply snapshot of one tick.  The
  COM turns it into TELEM: roll / pitch / yaw from q (``snap_to_tlm_src``), the
  onboard quantisation (``tlm_src_telemetry``, the same call ``sim.onboard`` makes) and
  a ``link_proto`` TELEM frame with SEQ = reply_seq (``telem_frame_from_snap``).
* ``TYPE_SNAP2`` (0x04, FCU -> COM, v0.7 VTOL build, 55 B ``<BBBBBHHIfffff4fBBf``): the 49 SNAP bytes,
  then phase u8 (``link_proto.PHASE_*``), spare u8 (0 - see below) and the altitude target h_tgt_m (float32).  The
  COM turns it into a TELEM2 frame (``link_proto.TYPE_TELEM2``: the v1 fields + phase + h_tgt quantised to 0.1 m).
  A new TYPE, used only by the VTOL build: the ballistic build keeps sending the v0.6.3 SNAP bytes (and its COM
  keeps the ``ACCEPT_COM`` mask), so it stays bit-identical on this wire too.  The spare byte makes LEN 55, three or
  more bits away from the lengths the FCU accepts (CMD 8: 6 bits, STAT 36: 3 bits), the rule the other lengths
  follow; 54 would be two bits from 36.  55 is two bits from SNAP's 49, but no flight decoder accepts both SNAP
  types (ballistic COM: ``ACCEPT_COM`` = SNAP, VTOL COM: ``ACCEPT_COM_VTOL`` = SNAP2; only the ``ACCEPT_ALL`` tap does).
* ``TYPE_SNAP3`` (0x05, FCU -> COM, v0.8 VTOL build with GPS position hold, 62 B ``SNAP2_FMT`` layout with the spare
  byte carrying gps_flags, then num_sv u8, hacc_dm u8 (0.1 m), pos_mode u8 and dist_m f32 (distance from the hold
  point, NaN = none)): the COM turns it into a TELEM3 frame (``link_proto.TYPE_TELEM3``).  62 is three or more bits from
  the lengths the FCU accepts (CMD 8: 4 bits, STAT 36: 3 bits); the VTOL + GPS COM accepts only SNAP3
  (``ACCEPT_COM_GPS``).  ``ACCEPT_ALL`` keeps its v0.7 meaning (15: CMD, SNAP, STAT, SNAP2 - the vectors of v0.7 keep
  their bytes); ``ACCEPT_ALL8`` (31) adds SNAP3.
* v0.9 firmware round (RP1, the power tail of ``sim.power_link``): SNAP2 and SNAP3 carry two more bytes at their end,
  pw_vaux u8 (0.125 V) and pw_flags u8 (PF_* | build << 6; the FCU sends 0, 0 in the battery build) - SNAP2 57 B
  (``SNAP2_FMT`` + "BB"), SNAP3 65 B (``SNAP3_FMT`` + "BB" + a reserved byte ``spare3``, 0, ignored by the COM, kept for a
  bit-exact repack).  The reserved byte keeps the length rule above: 65 is 3 bits from CMD 8 and 4 bits from STAT 36
  (64 would be 2 bits from 8), and the VTOL COM's two lengths 57 / 65 are 4 bits apart (55 / 62 were 2); 57 is 3 bits from
  8 and 4 from 36 (it is 1 bit from SNAP's 49, which no flight decoder accepts together with SNAP2, as before).  The COM
  turns a snapshot whose build bits are not the battery build into TELEM v4 / v5 (``link_proto.power_tail_due``; one COM
  image serves every power build), else into TELEM v2 / v3 as before.
* ``TYPE_STAT`` (0x03, COM -> FCU, 36 B ``<IBBBbbB13H``): ``Stat``, the COM's counters.  Paced by the COM
  (``src/com/com_link.h`` D4): at least 1 / ``flight_computer.ilink.stat_hz`` apart, never queued in front of an
  IL_CMD (so up to ~2.5 / stat_hz apart when the uplink stops), plus one boot STAT (``STAT_BOOT``) sent no earlier
  than 100 ms after the COM's reset, so an FCU powered up at the same moment is listening.

Direction filter (v0.6.3, R4): each side's decoder is built with the set of TYPEs it accepts
(``ACCEPT_*`` bits, ``IlinkDecoder(accept)`` / ``il_decoder_init(d, accept)``): the FCU accepts
CMD and STAT (``ACCEPT_FCU``), the COM only SNAP (``ACCEPT_COM``; v0.7 VTOL build: only SNAP2,
``ACCEPT_COM_VTOL``); ``ACCEPT_ALL`` (all four TYPEs since v0.7) is for taps and tests.  A LEN
or TYPE of a frame that cannot travel in this direction is rejected in the header (rule 3), so
e.g. a SNAP header in garbage on the FCU's RX line never swallows the 55 bytes behind it.

roll / pitch / yaw: this reference computes them in float64 from the float32 q
(``math.atan2`` / ``math.asin`` / ``math.degrees``, the ``sim.quat.to_euler``
formulas); the C COM computes the same formulas in float32 (``atan2f`` / ``asinf`` x
``FC_RAD2DEG_F``, the v0.5 FCU code).  The quantised TELEM roll / pitch / yaw fields
(0.01 deg) may therefore differ by ONE quantum (a value within ~1e-5 deg of a
rounding boundary); every other TELEM byte is exactly equal (the quantisation itself
is exact in both, see ``link_proto.quantize`` / ``lp_quantize``).  Near the ZYX
singularity (pitch +-90 deg) the float32 angles are ill-conditioned and may differ by
more; the vehicle never flies there (tilt_abort_deg).

Streaming decoder (``IlinkDecoder``) - the NORMATIVE rules (v0.6 phase 3, no replay);
``il_decoder_feed`` must agree with them on every returned frame, every counter and
``need`` / ``cmd_need`` after every byte, for every byte stream:

1. One byte at a time.  ``feed_byte(b, max_out)`` examines b against the open candidate
   (rule 3) and, only if b rejects it, examines b ONCE more while hunting (rule 2) -
   bounded work per input byte, and at most ONE frame completes per input byte.
   ``feed(data, max_out)`` is ``feed_byte`` for each byte in order.
2. Hunting (no open candidate): SYNC0 0xC5 opens a candidate (it is its first byte);
   any other byte is discarded (``skipped_bytes`` += 1).
3. Open candidate of n bytes; b is checked at position n and appended if it passes:
   n = 1  b must be SYNC1 0x5C (else reject, no reason counter - a 0xC5 here therefore
          restarts the candidate through rule 4);
   n = 2  LEN: b must be the fixed payload length of an ACCEPTED type (CMD 8, SNAP 49,
          STAT 36, SNAP2 57, SNAP3 65 (v0.9); else ``bad_len``, reject);
   n = 3  TYPE: b must be an accepted type (an unknown TYPE or a known one this decoder
          does not accept: ``bad_type``, reject) whose length is LEN (else ``bad_len``,
          reject);
   4 <= n < LEN + 4   payload byte: always appended (a sync pair inside is never looked at);
   n = LEN + 4   CRC low byte: must equal the low byte of the CRC over LEN, TYPE and the
          payload (else ``bad_crc``, reject - checked as soon as it arrives);
   n = LEN + 5   CRC high byte: must equal the high byte (else ``bad_crc``, reject); if it
          does, the frame is complete (rule 5).
4. Reject: the reason counter (if any) += 1; the candidate's n bytes are discarded
   (``skipped_bytes`` += n) and the decoder hunts; then the rejecting byte b itself is
   examined once by rule 2 (0xC5 opens a new candidate, anything else is discarded and
   counted).  NOTHING ELSE IS REPLAYED: a frame that starts inside a rejected candidate
   (after its first byte and before its rejecting byte) is lost.
5. Complete frame: ``frames`` += 1; it is returned if ``max_out`` >= 1, otherwise it is
   dropped and ``overflow`` += 1 (only a caller passing max_out = 0 sees overflow).  The
   decoder hunts again; the frame's bytes are consumed whole.
6. Counters are uint16 and wrap modulo 65536 (as in C).
7. ``skipped_bytes`` is exactly the number of input bytes that were discarded: every byte
   hunted and not 0xC5, plus every byte of every rejected candidate (the rejecting byte
   is not part of the candidate; it is counted only if its own hunt discards it).
   Invariant (mod 2^16): bytes fed = ``skipped_bytes`` + sum(LEN + 6) over all complete
   frames (returned or overflowed) + ``pending`` (bytes of the open candidate).
8. ``need``: bytes still needed before the open candidate can complete - 0 while hunting,
   LEN + 6 - n once LEN is accepted (n >= 3), and while LEN is not known yet (n = 1, 2)
   the rest of the shortest ACCEPTED frame (FCU and ALL: an IL_CMD, 14 - n; COM: a SNAP,
   55 - n; VTOL COM: a SNAP2, 63 - n (v0.9); ``NEED_NEVER`` = 255 if the mask accepts nothing).  ``cmd_need`` (``NEED_NEVER``
   for a decoder that does not accept CMD, i.e. the COM's): the fewest further
   input bytes after which the decoder can return an IL_CMD (exact: some continuation of
   that length returns one on its last byte, none shorter does) - the minimum of the
   candidate's own ``need`` if it can still be an IL_CMD (n = 1, 2, or LEN = 8) and
   ``k0 + 13``, where k0 is the first byte that can reject the candidate and open a new
   one as its 0xC5: k0 = 1 while hunting or in the header (n <= 3); for n >= 4 the CRC low
   byte (k0 = need - 1) while payload bytes are still to come (they can make that CRC byte
   differ from 0xC5); with only CRC bytes left, k0 = 1 if the next expected CRC byte is
   not 0xC5, else the answer is 15 (that 0xC5 would be accepted, a CMD can open only one
   byte later).

Why no replay (D1 of the phase-2 review): the phase-2 decoder re-fed a rejected
candidate's bytes so a frame hidden inside it was still found; on the FCU that cost up to
6316 cycles for ONE input byte (a 54-byte replay) and 1821 cycles per byte averaged over a
``C5 5C 31 02`` sync-spam stream, against 640 cycles of wire time per byte at 250 kbaud - a
16-byte time-critical decode could take 1.9 ms and a babbling COM could starve the FCU's
background (baro math, log).  The FCU's real-time budget needs a per-byte cost bounded by a
constant, so the decoder examines each byte at most twice.  The price is the lost hidden
frame, which on this wire needs (a) a valid-looking header in garbage right before a frame
(C5 5C LEN TYPE with a matching pair - noise on a 10 cm board-to-board wire), or (b) a
frame's LEN turned into ANOTHER valid LEN (8, 36, 49 are >= 3 bits apart, so never by one
bit error) or (c) a frame's own bytes holding a complete valid header (~7e-10 per position
of random data).  A frame directly behind a corrupted one survives any single substituted,
dropped or inserted byte in the corrupted one, and two bytes dropped from its payload / CRC
unless the shortened candidate's CRC low byte happens to read 0xC5 (1 in 256): the CRC low
byte is checked when it arrives and the rejecting byte is re-examined
(``sim/test_ilink_proto.py`` and the host test sweep every single-byte case).  A lost IL_CMD is covered by the ground
station's retry (``sim.ground_station_logic``: after the reply timeout the next poll repeats
the pending command with the same SEQ until a TELEM acknowledges it), a lost SNAP is one
reply timeout, a lost STAT one missing status update.

The C decoder keeps the candidate's payload in the frame it returns and computes the CRC
incrementally (``il_crc16_update``); this reference keeps the candidate as a byte list and
computes the CRC of the whole candidate when its CRC bytes arrive - two independent
implementations of the same rules (``tools/build_host_test.py`` vectors and
``tools/crosscheck_controller.py --only ilink``).

Invariant: ``encode(t, p)`` is the only way bytes are produced, ``unpack(pack(x))``
reproduces x bit for bit (quiet NaN payloads included; a signalling NaN is quietened by
Python's float conversion - the C port copies the 4 bytes and keeps it), and the
decoder never returns a frame whose CRC, TYPE or LEN is wrong.
"""
from __future__ import annotations

import json
import math
import os
import random
import struct
import sys
from dataclasses import dataclass
from typing import Iterable, Sequence

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import link_proto as lp
except ImportError:  # pragma: no cover
    import link_proto as lp  # type: ignore

_HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.normpath(os.path.join(_HERE, "..", "params.json"))
VECTORS_PATH = os.path.join(_HERE, "out", "ilink_vectors.json")

# --------------------------------------------------------------------------- #
# constants (== firmware/atmega128_fc/src/core/ilink_proto.h, checked by sim/test_ilink_proto.py)
# --------------------------------------------------------------------------- #
SYNC0 = 0xC5
SYNC1 = 0x5C
TYPE_CMD = 0x01
TYPE_SNAP = 0x02
TYPE_STAT = 0x03
TYPE_SNAP2 = 0x04                        # v0.7 VTOL build: SNAP + phase, spare, h_tgt_m
TYPE_SNAP3 = 0x05                        # v0.8 VTOL + GPS: SNAP2 (spare = gps_flags) + num_sv, hacc_dm, pos_mode, dist_m
TYPE_NAMES = {TYPE_CMD: "CMD", TYPE_SNAP: "SNAP", TYPE_STAT: "STAT", TYPE_SNAP2: "SNAP2", TYPE_SNAP3: "SNAP3"}
CMD_FMT = "<BBHHbb"
SNAP_FMT = "<BBBBBHHIfffffffff"
SNAP2_FMT = SNAP_FMT + "BBf" + "BB"      # + phase u8, spare u8 (0), h_tgt_m f32; v0.9 RP1: + pw_vaux u8, pw_flags u8
SNAP3_FMT = SNAP_FMT + "BBfBBBf" + "BBB"  # SNAP2 layout with spare = gps_flags; + num_sv u8, hacc_dm u8, pos_mode u8,
                                          # dist_m f32; v0.9 RP1: + pw_vaux u8, pw_flags u8, spare3 u8 (0)
STAT_FMT = "<IBBBbbB" + "H" * 13
CMD_LEN = struct.calcsize(CMD_FMT)       # 8
SNAP_LEN = struct.calcsize(SNAP_FMT)     # 49
STAT_LEN = struct.calcsize(STAT_FMT)     # 36
SNAP2_LEN = struct.calcsize(SNAP2_FMT)   # 57 (v0.7 - v0.8: 55)
SNAP3_LEN = struct.calcsize(SNAP3_FMT)   # 65 (v0.8: 62)
PAYLOAD_LEN = {TYPE_CMD: CMD_LEN, TYPE_SNAP: SNAP_LEN, TYPE_STAT: STAT_LEN, TYPE_SNAP2: SNAP2_LEN, TYPE_SNAP3: SNAP3_LEN}
SNAP_TYPES = (TYPE_SNAP, TYPE_SNAP2, TYPE_SNAP3)
VALID_LENS = frozenset(PAYLOAD_LEN.values())
MAX_PAYLOAD = max(PAYLOAD_LEN.values())
FRAME_OVERHEAD = 6                       # sync x2, LEN, TYPE, CRC x2
CMD_FRAME_BYTES = FRAME_OVERHEAD + CMD_LEN      # 14
SNAP_FRAME_BYTES = FRAME_OVERHEAD + SNAP_LEN    # 55
STAT_FRAME_BYTES = FRAME_OVERHEAD + STAT_LEN    # 42
SNAP2_FRAME_BYTES = FRAME_OVERHEAD + SNAP2_LEN  # 63 (v0.7: 61)
SNAP3_FRAME_BYTES = FRAME_OVERHEAD + SNAP3_LEN  # 71 (v0.8: 68)
FRAME_MAX_BYTES = SNAP3_FRAME_BYTES              # v0.6.3: 55, v0.7: 61, v0.8: 68
DEC_MAX_OUT = 1                          # IL_DEC_MAX_OUT: at most one frame completes per input byte (rule 1)
CRC_INIT = lp.CRC_INIT

ACCEPT_CMD = 1 << (TYPE_CMD - 1)         # IL_ACCEPT_*: il_decoder_init's accept mask, one bit per TYPE
ACCEPT_SNAP = 1 << (TYPE_SNAP - 1)
ACCEPT_STAT = 1 << (TYPE_STAT - 1)
ACCEPT_SNAP2 = 1 << (TYPE_SNAP2 - 1)     # 8
ACCEPT_SNAP3 = 1 << (TYPE_SNAP3 - 1)     # 16
ACCEPT_ALL = ACCEPT_CMD | ACCEPT_SNAP | ACCEPT_STAT | ACCEPT_SNAP2   # 15 (v0.6.3: 7) - the v0.7 taps / vectors
ACCEPT_ALL8 = ACCEPT_ALL | ACCEPT_SNAP3  # 31: every TYPE of v0.8
ACCEPT_FCU = ACCEPT_CMD | ACCEPT_STAT     # the FCU receives CMD and STAT (every build)
ACCEPT_COM = ACCEPT_SNAP                 # the ballistic COM receives SNAP
ACCEPT_COM_VTOL = ACCEPT_SNAP2           # the VTOL COM receives SNAP2
ACCEPT_COM_GPS = ACCEPT_SNAP3            # the v0.8 VTOL + GPS COM receives SNAP3
ACCEPT_NAMES = {ACCEPT_ALL: "all", ACCEPT_FCU: "fcu", ACCEPT_COM: "com", ACCEPT_COM_VTOL: "comv", ACCEPT_ALL8: "all8",
                ACCEPT_COM_GPS: "comg"}
ACCEPT_COM_BY_MODE = {lp.MODE_BALLISTIC: ACCEPT_COM, lp.MODE_VTOL: ACCEPT_COM_VTOL}
SNAP_TYPE_BY_MODE = {lp.MODE_BALLISTIC: TYPE_SNAP, lp.MODE_VTOL: TYPE_SNAP2}
SNAP3_EXTRA_FIELDS = ("gps_flags", "num_sv", "hacc_dm", "pos_mode", "dist_m")
NEED_NEVER = 0xFF                        # IL_NEED_NEVER: need / cmd_need of a frame this decoder can never return

STAT_RADIO_OK = 0x01                     # sx127x: RegVersion 0x12 read back; lora_uart: always
STAT_BOOT = 0x02                         # first STAT after a COM reset
STAT_SX127X = 0x04                       # backend: set = sx127x, clear = lora_uart
RSSI_NONE = 127                          # RADIO_RSSI_NONE (hal/radio.h)

CMD_FIELDS = ("seq", "cmd", "arg", "nonce", "rssi_dbm", "snr_q4")
SNAP_INT_FIELDS = ("reply_seq", "state", "flags", "reason", "ack_seq", "nonce", "throttle_milli", "tick")
SNAP_FLOAT_FIELDS = ("h_m", "vz_mps", "tilt_deg", "v_cell", "loop_max_us")
SNAP_FIELDS = SNAP_INT_FIELDS + SNAP_FLOAT_FIELDS + ("q",)
SNAP2_EXTRA_FIELDS = ("phase", "spare", "h_tgt_m")        # after q, SNAP2 only
SNAP_PWR_FIELDS = ("pw_vaux", "pw_flags")                  # v0.9 RP1: the last bytes of SNAP2 (SNAP3: + spare3)
STAT_FIELDS = ("com_ticks", "reset_cause", "flags", "radio_version", "rssi_dbm", "snr_q4", "params_tag", "dec_frames",
               "dec_bad_crc", "dec_bad_len", "dec_other", "cmd_forwarded", "radio_rx_errors", "radio_rx_overflow",
               "radio_tx_timeouts", "replies", "replies_dropped", "il_bad", "il_tx_dropped", "loop_us_max")
DEC_STAT_KEYS = ("frames", "bad_crc", "bad_len", "bad_type", "skipped_bytes", "overflow")   # il_dec_stats_t order

U8, U16, U32 = lp.U8, lp.U16, lp.U32
I8 = (-0x80, 0x7F)
# TELEM frame byte offsets of the i16 roll / pitch / yaw fields (link_proto: 6 header bytes + payload offsets 14, 16, 18)
TELEM_ANGLE_OFFSETS = (20, 22, 24)


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def crc16(data: Iterable[int], crc: int = CRC_INIT) -> int:
    """CRC-16/CCITT-FALSE - ``sim.link_proto.crc16_ccitt`` (one CRC for both links)."""
    return lp.crc16_ccitt(data, crc)


def crc16_update(crc: int, b: int) -> int:
    """One byte of the CRC in the byte-wise form ``il_crc16_update`` uses (== lp_crc16's branch-free update):
    x = (crc >> 8) ^ b; x ^= x >> 4; crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x, written on the two register
    bytes: hi' = lo ^ (x << 4) ^ (x >> 3), lo' = (x << 5) ^ x (8-bit shifts).  Checked against ``crc16``."""
    x = ((crc >> 8) ^ b) & 0xFF
    x ^= x >> 4
    hi = ((crc & 0xFF) ^ ((x << 4) & 0xFF) ^ (x >> 3)) & 0xFF
    lo = (((x << 5) & 0xFF) ^ x) & 0xFF
    return (hi << 8) | lo


def f32(x: float) -> float:
    """The float32 value C's ``(float)x`` gives (round to nearest; beyond FLT_MAX -> +-inf)."""
    try:
        return struct.unpack("<f", struct.pack("<f", x))[0]
    except OverflowError:
        return math.copysign(math.inf, x)


def f32_bits(x: float) -> int:
    """IEEE binary32 bit pattern of ``f32(x)``."""
    return struct.unpack("<I", struct.pack("<f", f32(x)))[0]


def f32_from_bits(u: int) -> float:
    return struct.unpack("<f", struct.pack("<I", u & 0xFFFFFFFF))[0]


def clampf(x: float, lo: float, hi: float) -> float:
    """``fc_clampf`` (fcmath.h): ``x < lo ? lo : (x > hi ? hi : x)`` - NaN passes through, unlike min/max."""
    return lo if x < lo else (hi if x > hi else x)


def _check(name: str, v: int, lim: tuple[int, int]) -> int:
    if not isinstance(v, int) or isinstance(v, bool) or not (lim[0] <= v <= lim[1]):
        raise ValueError(f"{name} = {v!r} outside [{lim[0]}, {lim[1]}]")
    return v


def load_link_params(path: str = PARAMS_PATH) -> tuple[int, int]:
    """(flight_computer.link.vehicle_id, control.loop_hz) from params.json."""
    with open(path, "r", encoding="utf-8") as f:
        p = json.load(f)
    return int(p["flight_computer"]["link"]["vehicle_id"]), int(p["control"]["loop_hz"])


# --------------------------------------------------------------------------- #
# payloads
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class IlCmd:
    """IL_TYPE_CMD payload (il_cmd_t): one uplink CMD the COM's link_proto decoder accepted."""

    seq: int
    cmd: int
    arg: int = 0
    nonce: int = 0
    rssi_dbm: int = RSSI_NONE
    snr_q4: int = 0

    def __post_init__(self) -> None:
        for n, lim in (("seq", U8), ("cmd", U8), ("arg", U16), ("nonce", U16), ("rssi_dbm", I8), ("snr_q4", I8)):
            _check(n, getattr(self, n), lim)

    @classmethod
    def from_uplink(cls, fr: lp.Frame, rssi_dbm: int = RSSI_NONE, snr_q4: int = 0) -> "IlCmd":
        c = lp.decode_command(fr)
        return cls(fr.seq, c.cmd, c.arg, c.nonce, rssi_dbm, snr_q4)

    @property
    def command(self) -> lp.Command:
        return lp.Command(self.cmd, self.arg, self.nonce)

    def pack(self) -> bytes:
        return struct.pack(CMD_FMT, self.seq, self.cmd, self.arg, self.nonce, self.rssi_dbm, self.snr_q4)

    @classmethod
    def unpack(cls, payload: bytes) -> "IlCmd":
        if len(payload) != CMD_LEN:
            raise ValueError(f"CMD payload is {CMD_LEN} bytes, got {len(payload)}")
        return cls(*struct.unpack(CMD_FMT, payload))


@dataclass(frozen=True)
class Snap:
    """IL_TYPE_SNAP / IL_TYPE_SNAP2 payload (il_snap_t): the supervisor's reply snapshot.  Float fields hold the
    float32 values the wire carries (``pack`` rounds to float32; compare snapshots by ``pack()`` - NaN != NaN).

    SNAP (ballistic build): ``phase`` and ``h_tgt_m`` are None.  SNAP2 (VTOL build): both set; ``spare`` is the
    reserved byte (the FCU sends 0, the COM ignores it; kept here so any received payload re-packs bit for bit).
    v0.9 RP1: SNAP2 / SNAP3 end with the power tail ``pw_vaux`` / ``pw_flags`` (0 / 0 in the battery build; SNAP: both
    0, not on the wire) and SNAP3 with the reserved byte ``spare3`` (0)."""

    reply_seq: int
    state: int
    flags: int
    reason: int
    ack_seq: int
    nonce: int
    throttle_milli: int
    tick: int
    h_m: float
    vz_mps: float
    tilt_deg: float
    v_cell: float
    loop_max_us: float
    q: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    phase: int | None = None          # SNAP2: link_proto.PHASE_*
    h_tgt_m: float | None = None      # SNAP2: altitude target [m]
    spare: int = 0                    # SNAP2: reserved byte
    gps_flags: int | None = None      # SNAP3 (in the SNAP2 spare byte's position): sim.gps_nav GF_*
    num_sv: int | None = None         # SNAP3
    hacc_dm: int | None = None        # SNAP3: 0.1 m
    pos_mode: int | None = None       # SNAP3: sim.position_ctrl MODE_*
    dist_m: float | None = None       # SNAP3: distance from the hold point [m], NaN = none
    pw_vaux: int = 0                  # v0.9 SNAP2 / SNAP3: the power tail's v_aux (0.125 V)
    pw_flags: int = 0                 # v0.9 SNAP2 / SNAP3: PF_* | build << 6 (0: the battery build, no TELEM tail)
    spare3: int = 0                   # v0.9 SNAP3: reserved byte (the FCU sends 0, the COM ignores it)

    def __post_init__(self) -> None:
        for n in ("reply_seq", "state", "flags", "reason", "ack_seq"):
            _check(n, getattr(self, n), U8)
        _check("nonce", self.nonce, U16)
        _check("throttle_milli", self.throttle_milli, U16)
        _check("tick", self.tick, U32)
        if len(self.q) != 4:
            raise ValueError("q has 4 components (w, x, y, z)")
        if (self.phase is None) != (self.h_tgt_m is None):
            raise ValueError("SNAP2 needs both phase and h_tgt_m (SNAP: neither)")
        if self.phase is not None:
            _check("phase", self.phase, U8)
        _check("spare", self.spare, U8)
        if self.phase is None and self.spare:
            raise ValueError("spare exists in SNAP2 only")
        v3 = [getattr(self, n) is None for n in SNAP3_EXTRA_FIELDS]
        if any(v3) != all(v3):
            raise ValueError("SNAP3 needs every GPS field (SNAP / SNAP2: none)")
        if not v3[0]:
            if self.phase is None or self.spare:
                raise ValueError("SNAP3 extends SNAP2 (phase, h_tgt_m) and has no spare byte")
            for n in SNAP3_EXTRA_FIELDS[:4]:
                _check(n, getattr(self, n), U8)
        for n in SNAP_PWR_FIELDS + ("spare3",):
            _check(n, getattr(self, n), U8)
        if self.phase is None and (self.pw_vaux or self.pw_flags):
            raise ValueError("the power tail exists in SNAP2 / SNAP3 only")
        if self.gps_flags is None and self.spare3:
            raise ValueError("spare3 exists in SNAP3 only")

    @property
    def pwr(self) -> tuple[int, int] | None:
        """(pw_vaux, pw_flags) of a SNAP2 / SNAP3 (v0.9 RP1), None for a SNAP."""
        return None if self.phase is None else (self.pw_vaux, self.pw_flags)

    @property
    def is_v2(self) -> bool:
        return self.phase is not None

    @property
    def is_v3(self) -> bool:
        return self.gps_flags is not None

    @property
    def wire_type(self) -> int:
        if self.is_v3:
            return TYPE_SNAP3
        return TYPE_SNAP2 if self.is_v2 else TYPE_SNAP

    @property
    def gps(self) -> tuple | None:
        """(gps_flags, num_sv, hacc_dm, pos_mode, dist_m) of a SNAP3, else None."""
        return None if not self.is_v3 else tuple(getattr(self, n) for n in SNAP3_EXTRA_FIELDS)

    def pack(self) -> bytes:
        v1 = (self.reply_seq, self.state, self.flags, self.reason, self.ack_seq, self.nonce, self.throttle_milli, self.tick,
              *(f32(getattr(self, n)) for n in SNAP_FLOAT_FIELDS), *(f32(c) for c in self.q))
        if self.is_v3:
            return struct.pack(SNAP3_FMT, *v1, self.phase, self.gps_flags, f32(self.h_tgt_m), self.num_sv, self.hacc_dm,
                               self.pos_mode, f32(self.dist_m), self.pw_vaux, self.pw_flags, self.spare3)
        if self.is_v2:
            return struct.pack(SNAP2_FMT, *v1, self.phase, self.spare, f32(self.h_tgt_m), self.pw_vaux, self.pw_flags)
        return struct.pack(SNAP_FMT, *v1)

    @classmethod
    def unpack(cls, payload: bytes) -> "Snap":
        """49 bytes: SNAP; 57 bytes: SNAP2; 65 bytes: SNAP3 (v0.9 RP1 lengths); any other length is a ValueError."""
        if len(payload) == SNAP_LEN:
            v = struct.unpack(SNAP_FMT, payload)
            return cls(*v[:13], q=tuple(v[13:17]))
        if len(payload) == SNAP2_LEN:
            v = struct.unpack(SNAP2_FMT, payload)
            return cls(*v[:13], q=tuple(v[13:17]), phase=v[17], spare=v[18], h_tgt_m=v[19], pw_vaux=v[20],
                       pw_flags=v[21])
        if len(payload) == SNAP3_LEN:
            v = struct.unpack(SNAP3_FMT, payload)
            return cls(*v[:13], q=tuple(v[13:17]), phase=v[17], gps_flags=v[18], h_tgt_m=v[19], num_sv=v[20],
                       hacc_dm=v[21], pos_mode=v[22], dist_m=v[23], pw_vaux=v[24], pw_flags=v[25], spare3=v[26])
        raise ValueError(f"SNAP payload is {SNAP_LEN} (SNAP), {SNAP2_LEN} (SNAP2) or {SNAP3_LEN} (SNAP3) bytes, got "
                         f"{len(payload)}")


@dataclass(frozen=True)
class Stat:
    """IL_TYPE_STAT payload (il_stat_t): the COM's counters, paced by the COM (module docstring; ilink_proto.h has
    each field)."""

    com_ticks: int = 0
    reset_cause: int = 0
    flags: int = 0
    radio_version: int = 0
    rssi_dbm: int = RSSI_NONE
    snr_q4: int = 0
    params_tag: int = 0
    dec_frames: int = 0
    dec_bad_crc: int = 0
    dec_bad_len: int = 0
    dec_other: int = 0
    cmd_forwarded: int = 0
    radio_rx_errors: int = 0
    radio_rx_overflow: int = 0
    radio_tx_timeouts: int = 0
    replies: int = 0
    replies_dropped: int = 0
    il_bad: int = 0
    il_tx_dropped: int = 0
    loop_us_max: int = 0

    def __post_init__(self) -> None:
        _check("com_ticks", self.com_ticks, U32)
        for n in ("reset_cause", "flags", "radio_version", "params_tag"):
            _check(n, getattr(self, n), U8)
        for n in ("rssi_dbm", "snr_q4"):
            _check(n, getattr(self, n), I8)
        for n in STAT_FIELDS[7:]:
            _check(n, getattr(self, n), U16)

    def pack(self) -> bytes:
        return struct.pack(STAT_FMT, *(getattr(self, n) for n in STAT_FIELDS))

    @classmethod
    def unpack(cls, payload: bytes) -> "Stat":
        if len(payload) != STAT_LEN:
            raise ValueError(f"STAT payload is {STAT_LEN} bytes, got {len(payload)}")
        return cls(*struct.unpack(STAT_FMT, payload))


# --------------------------------------------------------------------------- #
# frames
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Frame:
    type: int
    payload: bytes

    @property
    def len(self) -> int:
        return len(self.payload)

    def encode(self) -> bytes:
        return encode(self.type, self.payload)


def payload_len(typ: int) -> int:
    """Fixed LEN of a TYPE, 0 for an unknown type (il_payload_len)."""
    return PAYLOAD_LEN.get(typ, 0)


def accepted_len(typ: int, accept: int) -> int:
    """LEN of TYPE if the accept mask takes it, else 0 (unknown or not accepted)."""
    return PAYLOAD_LEN[typ] if typ in PAYLOAD_LEN and accept & (1 << (typ - 1)) else 0


def accepted_lens(accept: int) -> frozenset:
    return frozenset(PAYLOAD_LEN[t] for t in PAYLOAD_LEN if accept & (1 << (t - 1)))


def encode(typ: int, payload: bytes) -> bytes:
    """Complete frame (il_encode); ValueError if TYPE is unknown or the payload is not its fixed length."""
    n = payload_len(typ)
    if n == 0:
        raise ValueError(f"unknown ilink TYPE 0x{typ:02X}")
    if len(payload) != n:
        raise ValueError(f"ilink {TYPE_NAMES[typ]} payload is {n} bytes, got {len(payload)}")
    body = bytes([n, typ]) + bytes(payload)
    crc = crc16(body)
    return bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8])


def encode_cmd(c: IlCmd) -> bytes:
    return encode(TYPE_CMD, c.pack())


def encode_snap(s: Snap) -> bytes:
    """IL_TYPE_SNAP frame (ballistic) or IL_TYPE_SNAP2 frame (VTOL), by ``Snap.wire_type``."""
    return encode(s.wire_type, s.pack())


def encode_stat(s: Stat) -> bytes:
    return encode(TYPE_STAT, s.pack())


class IlinkDecoder:
    """Streaming decoder, no replay: the rules are the module docstring's (normative for il_decoder_feed,
    il_decoder_need and il_decoder_cmd_need).  ``accept``: ACCEPT_* mask of the TYPEs this side receives
    (FCU: ACCEPT_FCU, COM: ACCEPT_COM; the default ACCEPT_ALL is for taps and tests)."""

    def __init__(self, accept: int = ACCEPT_ALL) -> None:
        if not isinstance(accept, int) or isinstance(accept, bool) or accept & ~ACCEPT_ALL8 or accept < 0:
            raise ValueError(f"accept mask {accept!r}: ACCEPT_* bits only (0x{ACCEPT_ALL8:02X})")
        self.accept = accept
        self._lens = accepted_lens(accept)
        self._short = min(self._lens) + FRAME_OVERHEAD if self._lens else None
        self.stats: dict[str, int] = {k: 0 for k in DEC_STAT_KEYS}
        self.frame_bytes = 0          # diagnostic (no C counterpart): bytes of all complete frames, returned or overflowed
        self._cand = bytearray()

    @property
    def pending(self) -> int:
        """Bytes of the open candidate (0..FRAME_MAX_BYTES - 1 = 60; il_decoder_t.n)."""
        return len(self._cand)

    @property
    def need(self) -> int:
        """Rule 8: bytes still needed before the open candidate can complete (0 while hunting)."""
        n = len(self._cand)
        if n == 0:
            return 0
        if n < 3:                                   # LEN not known yet: the rest of the shortest accepted frame
            return NEED_NEVER if self._short is None else self._short - n
        return self._cand[2] + FRAME_OVERHEAD - n

    @property
    def cmd_need(self) -> int:
        """Rule 8: the fewest further input bytes after which an IL_CMD can be returned (exact)."""
        c, n = self._cand, len(self._cand)
        if not self.accept & ACCEPT_CMD:
            return NEED_NEVER
        options = []
        if 0 < n and (n < 3 or c[2] == CMD_LEN):     # the candidate itself can still be an IL_CMD
            options.append(CMD_FRAME_BYTES - n)
        if n <= 3:                                  # hunting, or a 0xC5 as the next header byte rejects and opens
            options.append(1 + CMD_FRAME_BYTES - 1)
        else:                                       # payload bytes never reject: the first CRC byte that can
            ln = c[2]
            free = ln + 4 - n                       # payload bytes still to come (chosen so a CRC byte is not 0xC5)
            if free > 0:
                options.append(free + 1 + CMD_FRAME_BYTES - 1)
            else:
                crc = crc16(c[2:4 + ln])
                nxt = crc & 0xFF if n == ln + 4 else crc >> 8
                # a 0xC5 that the CRC byte would accept cannot reject there: then the first byte can only close the
                # candidate (reject + discard, or complete it) and a CMD opens one byte later
                options.append(CMD_FRAME_BYTES + (1 if nxt == SYNC0 else 0))
        return min(options)

    def feed(self, data: bytes | bytearray | Iterable[int], max_out: int = DEC_MAX_OUT) -> list[Frame]:
        out: list[Frame] = []
        for b in data:
            out.extend(self.feed_byte(b, max_out))
        return out

    def _count(self, key: str, k: int = 1) -> None:
        self.stats[key] = (self.stats[key] + k) & 0xFFFF

    def _hunt(self, b: int) -> None:
        """Rule 2 (no candidate open)."""
        if b == SYNC0:
            self._cand = bytearray([b])
        else:
            self._count("skipped_bytes")

    def _check(self, b: int) -> str | None:
        """Rule 3: None if b passes at the candidate's next position, else the rejection reason ('' = no counter)."""
        c = self._cand
        n = len(c)
        if n == 1:
            return None if b == SYNC1 else ""
        if n == 2:
            return None if b in self._lens else "bad_len"
        if n == 3:
            t = accepted_len(b, self.accept)
            return "bad_type" if t == 0 else (None if t == c[2] else "bad_len")
        ln = c[2]
        if n < ln + 4:
            return None
        crc = crc16(c[2:4 + ln])                    # the CRC of the whole candidate (C: incremental)
        want = crc & 0xFF if n == ln + 4 else crc >> 8
        return None if b == want else "bad_crc"

    def feed_byte(self, b: int, max_out: int = DEC_MAX_OUT) -> list[Frame]:
        b &= 0xFF
        if not self._cand:
            self._hunt(b)
            return []
        reason = self._check(b)
        if reason is not None:                      # rule 4: reject, then b is hunted once
            if reason:
                self._count(reason)
            self._count("skipped_bytes", len(self._cand))
            self._cand = bytearray()
            self._hunt(b)
            return []
        c = self._cand
        c.append(b)
        ln = c[2] if len(c) > 2 else -1
        if len(c) < ln + FRAME_OVERHEAD:
            return []
        fr = Frame(c[3], bytes(c[4:4 + ln]))       # rule 5
        self._cand = bytearray()
        self.frame_bytes += ln + FRAME_OVERHEAD
        self._count("frames")
        if max_out >= 1:
            return [fr]
        self._count("overflow")
        return []


# --------------------------------------------------------------------------- #
# FCU snapshot <-> COM telemetry source
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class TlmSource:
    """onb_tlm_src_t: everything a TELEM reply needs, captured in the supervisor's tick."""

    state: int
    flags: int
    reason: int
    ack_seq: int
    nonce: int
    throttle_milli: int
    tick: int
    h_m: float
    vz_mps: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    tilt_deg: float
    v_cell: float
    loop_max_us: float
    phase: int | None = None          # VTOL build (TELEM v2 / SNAP2): link_proto.PHASE_*
    h_tgt_m: float | None = None      # VTOL build: altitude target [m]
    gps: tuple | None = None          # v0.8 VTOL + GPS (TELEM v3 / SNAP3): (gps_flags, num_sv, hacc_dm, pos_mode, dist_m)
    pwr: tuple | None = None          # v0.9 RP1 (SNAP2 / SNAP3): (pw_vaux, pw_flags) - TELEM v4 / v5 when the build bits
                                      # are not the battery build (link_proto.power_tail_due)


def euler_deg(q: Sequence[float]) -> tuple[float, float, float]:
    """ZYX roll, pitch, yaw (deg) of q = (w, x, y, z) in float64: the ``sim.quat.to_euler`` formulas with
    ``fc_clampf`` semantics (NaN passes the clamp), as the v0.5 FCU / v0.6 COM compute them in float32."""
    w, x, y, z = (float(c) for c in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(clampf(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def snap_from_tlm_src(reply_seq: int, src: TlmSource, q: Sequence[float]) -> Snap:
    """FCU side (il_snap_from_tlm_src): the snapshot of one supervisor reply; src's roll / pitch / yaw are
    not sent (the COM recomputes them from q).  Floats are rounded to float32 as the C struct holds them.
    A VTOL source (phase / h_tgt_m set) gives a SNAP2 with spare = 0; with ``gps`` set a SNAP3; v0.9: the power tail
    from ``src.pwr`` (0, 0 without one)."""
    g = {}
    if src.phase is not None and src.pwr is not None:
        g.update(pw_vaux=int(src.pwr[0]), pw_flags=int(src.pwr[1]))
    if src.gps is not None:
        gf, sv, ha, pm, dist = src.gps
        g = dict(gps_flags=int(gf), num_sv=min(int(sv), 255), hacc_dm=min(int(ha), 255), pos_mode=int(pm),
                 dist_m=f32(dist))
    return Snap(reply_seq=reply_seq, state=src.state, flags=src.flags, reason=src.reason, ack_seq=src.ack_seq,
                nonce=src.nonce, throttle_milli=src.throttle_milli, tick=src.tick, h_m=f32(src.h_m),
                vz_mps=f32(src.vz_mps), tilt_deg=f32(src.tilt_deg), v_cell=f32(src.v_cell),
                loop_max_us=f32(src.loop_max_us), q=tuple(f32(c) for c in q), phase=src.phase,
                h_tgt_m=None if src.h_tgt_m is None else f32(src.h_tgt_m), **g)


def snap_to_tlm_src(s: Snap) -> TlmSource:
    """COM side (il_snap_to_tlm_src): back to the TELEM source, roll / pitch / yaw from q (float64 here,
    float32 in C - see the module docstring); SNAP2's phase / h_tgt_m pass through (spare is ignored)."""
    r, p, y = euler_deg(s.q)
    return TlmSource(state=s.state, flags=s.flags, reason=s.reason, ack_seq=s.ack_seq, nonce=s.nonce,
                     throttle_milli=s.throttle_milli, tick=s.tick, h_m=s.h_m, vz_mps=s.vz_mps, roll_deg=r,
                     pitch_deg=p, yaw_deg=y, tilt_deg=s.tilt_deg, v_cell=s.v_cell, loop_max_us=s.loop_max_us,
                     phase=s.phase, h_tgt_m=s.h_tgt_m, gps=s.gps, pwr=s.pwr)


def tlm_src_telemetry(src: TlmSource, loop_hz: int) -> lp.Telemetry:
    """onboard_telemetry: the quantisation ``sim.onboard.OnboardSupervisor.telemetry`` applies (same
    ``Telemetry.from_physical`` call: t_ms = tick 1000 // loop_hz, throttle = throttle_milli / 1000; a VTOL source
    gives TELEM v2 with h_tgt quantised to 0.1 m; v0.9: + the power tail -> TELEM v4 / v5 when its build bits are not
    the battery build)."""
    return lp.Telemetry.from_physical(
        state=src.state, flags=src.flags, reason=src.reason, ack_seq=src.ack_seq, nonce=src.nonce,
        t_ms=(src.tick * 1000) // int(loop_hz), h_m=src.h_m, vz_mps=src.vz_mps, roll_deg=src.roll_deg,
        pitch_deg=src.pitch_deg, yaw_deg=src.yaw_deg, tilt_deg=src.tilt_deg,
        throttle=src.throttle_milli / lp.THROTTLE_SCALE, vcell_v=src.v_cell, loop_max_us=src.loop_max_us,
        phase=src.phase, h_tgt_m=src.h_tgt_m, gps=src.gps, pwr=src.pwr)


def telem_frame_from_snap(s: Snap, vehicle_id: int | None = None, loop_hz: int | None = None) -> bytes:
    """The COM pipeline: SNAP -> TELEM source -> onboard quantisation -> link_proto TELEM frame with
    SEQ = reply_seq (vehicle id / loop rate from params.json unless given); SNAP2 -> TELEM2 (v0.9: TELEM4 with a power
    build's tail), SNAP3 -> TELEM3 (TELEM5)."""
    if vehicle_id is None or loop_hz is None:
        vid, hz = load_link_params()
        vehicle_id = vid if vehicle_id is None else vehicle_id
        loop_hz = hz if loop_hz is None else loop_hz
    return lp.encode_telemetry(int(vehicle_id), s.reply_seq, tlm_src_telemetry(snap_to_tlm_src(s), int(loop_hz)))


# --------------------------------------------------------------------------- #
# cross-check vectors for the firmware port (sim/out/ilink_vectors.json)
# --------------------------------------------------------------------------- #
QNAN, NQNAN, QNAN_PAYLOAD, SNAN = 0x7FC00000, 0xFFC00000, 0x7FC0BEEF, 0x7F800001
PINF, NINF, NZERO, DENORM_MIN, DENORM_MAX, FLT_MAX_BITS, NFLT_MIN = (0x7F800000, 0xFF800000, 0x80000000, 0x00000001,
                                                                     0x007FFFFF, 0x7F7FFFFF, 0x80800000)


def _fields(typ: int, payload: bytes) -> dict:
    """Payload fields in struct order; floats as uint32 bit patterns (exact for every pattern, sNaN included)."""
    if typ == TYPE_CMD:
        return dict(zip(CMD_FIELDS, struct.unpack(CMD_FMT, payload)))
    if typ in SNAP_TYPES:
        ext = {TYPE_SNAP2: "BBIBB", TYPE_SNAP3: "BBIBBBIBBB"}.get(typ, "")
        u = struct.unpack("<BBBBBHHI9I" + ext, payload)
        d = {n: u[i] for i, n in enumerate(SNAP_INT_FIELDS)}
        for i, n in enumerate(SNAP_FLOAT_FIELDS):
            d[n + "_bits"] = u[8 + i]
        d["q_bits"] = list(u[13:17])
        if typ == TYPE_SNAP2:                                   # 22 values in all (v0.9: + pw_vaux, pw_flags)
            d["phase"], d["spare"], d["h_tgt_m_bits"] = u[17], u[18], u[19]
            d["pw_vaux"], d["pw_flags"] = u[20], u[21]
        if typ == TYPE_SNAP3:                                   # 27 values (v0.9: + pw_vaux, pw_flags, spare3)
            d["phase"], d["gps_flags"], d["h_tgt_m_bits"] = u[17], u[18], u[19]
            d["num_sv"], d["hacc_dm"], d["pos_mode"], d["dist_m_bits"] = u[20], u[21], u[22], u[23]
            d["pw_vaux"], d["pw_flags"], d["spare3"] = u[24], u[25], u[26]
        return d
    return dict(zip(STAT_FIELDS, struct.unpack(STAT_FMT, payload)))


def _snap_raw(ints: Sequence[int], float_bits: Sequence[int], v2: tuple | None = None) -> bytes:
    """SNAP payload from integer fields and 9 raw float bit patterns (h, vz, tilt, vcell, loop, q0..q3); with ``v2`` =
    (phase, spare, h_tgt_m bits[, pw_vaux, pw_flags]) a SNAP2 payload (v0.9: the power tail, 0 / 0 if not given)."""
    if v2 is None:
        return struct.pack("<BBBBBHHI9I", *ints, *float_bits)
    v2 = tuple(v2) + (0, 0)[:max(0, 5 - len(v2))]
    return struct.pack("<BBBBBHHI9IBBIBB", *ints, *float_bits, *v2)


def decode_trace(data: bytes, max_out: int = DEC_MAX_OUT,
                 accept: int = ACCEPT_ALL) -> tuple[IlinkDecoder, list[Frame], bytes, bytes]:
    """A fresh decoder (accept mask) fed byte by byte: (decoder, frames, need / cmd_need after each byte)."""
    dec = IlinkDecoder(accept)
    frames: list[Frame] = []
    need, cneed = bytearray(), bytearray()
    for b in data:
        frames += dec.feed_byte(b, max_out)
        need.append(dec.need)
        cneed.append(dec.cmd_need)
    return dec, frames, bytes(need), bytes(cneed)


def _decode_vector(name: str, data: bytes, max_out: int = DEC_MAX_OUT, note: str = "", accept: int = ACCEPT_ALL) -> dict:
    dec, frames, need, cneed = decode_trace(data, max_out, accept)
    blk = IlinkDecoder(accept)                    # self-check: feed() == byte-wise feed_byte()
    if blk.feed(data, max_out) != frames or blk.stats != dec.stats:   # pragma: no cover
        raise AssertionError(f"vector {name}: feed() differs from feed_byte()")
    if (len(data) - dec.stats["skipped_bytes"] - dec.frame_bytes - dec.pending) & 0xFFFF:  # pragma: no cover
        raise AssertionError(f"vector {name}: byte accounting (rule 7) broken")
    return {"name": name, "note": note, "hex": data.hex(), "max_out": max_out, "accept": accept,
            "expected_frames": [{"type": f.type, "len": f.len, "payload_hex": f.payload.hex()} for f in frames],
            "expected_stats": dict(dec.stats), "pending": dec.pending, "need_hex": need.hex(), "cmd_need_hex": cneed.hex()}


def make_test_vectors(vehicle_id: int, loop_hz: int, seed: int = 20261015) -> dict:
    """Deterministic vectors: payloads of each type (fields <-> bytes <-> frame), decoder streams (valid,
    back-to-back, noise, single-bit corruptions, sync pairs in payloads, frames hidden inside a rejected
    candidate - lost since phase 3 -, rejecting bytes re-examined as SYNC0, sync spam, truncated, wrong
    LEN / TYPE, max_out 0) with every returned frame, every counter and need / cmd_need after every byte,
    and SNAP -> TELEM frames of the COM pipeline.  All expectations are normative for the C port except the
    TELEM roll / pitch / yaw fields, which may differ by one quantum (float32 vs float64 trig)."""
    rng = random.Random(seed)
    payloads: list[dict] = []
    frames_by_type: dict[int, list[bytes]] = {TYPE_CMD: [], TYPE_SNAP: [], TYPE_STAT: [], TYPE_SNAP2: []}

    def add_payload(name: str, typ: int, payload: bytes, py_roundtrip: bool = True, note: str = "") -> None:
        fr = encode(typ, payload)
        frames_by_type[typ].append(fr)
        payloads.append({"name": name, "note": note, "type": typ, "payload_hex": payload.hex(), "frame_hex": fr.hex(),
                         "fields": _fields(typ, payload), "py_roundtrip": py_roundtrip})

    # ---- CMD: every v0.4 link command code, edge fields, RSSI none / extremes (v0.7 codes: the VTOL block below)
    for code, nm in lp.CMD_NAMES.items():
        if code in lp.VTOL_ONLY_CMDS:
            continue
        add_payload(f"cmd_{nm.lower()}", TYPE_CMD, IlCmd((code * 41) & 0xFF, code, 0, 0).pack())
    for seq, arg, nonce, rssi, snr in ((0, 0, 0, -128, -128), (255, 0xFFFF, 0xFFFF, 127, 127), (128, 1000, 0xBEEF, -97, -6),
                                       (127, 0x5CC5, 0xC55C, 0x5C, -0x3B)):
        add_payload(f"cmd_thr_seq{seq}_rssi{rssi}", TYPE_CMD, IlCmd(seq, lp.CMD_THROTTLE, arg, nonce, rssi, snr).pack(),
                    note="0x5CC5 / 0xC55C put sync pairs inside the payload" if arg == 0x5CC5 else "")
    for i in range(6):
        add_payload(f"cmd_random_{i}", TYPE_CMD, IlCmd(rng.randrange(256), rng.randrange(256), rng.randrange(65536),
                                                       rng.randrange(65536), rng.randrange(-128, 128),
                                                       rng.randrange(-128, 128)).pack())
    # ---- SNAP: typical, extreme, every special float class
    q_typ = (0.99965, -0.0125, 0.0213, 0.0087)
    add_payload("snap_zero", TYPE_SNAP, Snap(0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0, 0.0)).pack())
    add_payload("snap_pad", TYPE_SNAP, Snap(3, lp.STATE_ARMED, 0x0F, 0, 3, 0xACE1, 0, 2500, 0.0, 0.0, 1.25, 4.05, 2875.0,
                                            (1.0, 0.0, 0.0, 0.0)).pack())
    add_payload("snap_powered", TYPE_SNAP, Snap(17, lp.STATE_POWERED, 0x2F, 0, 17, 0x5A5A, 1000, 12136, 119.3, 22.71, 1.34,
                                                3.9, 3046.5, q_typ).pack())
    add_payload("snap_extremes", TYPE_SNAP, Snap(255, 255, 255, 255, 255, 0xFFFF, 0xFFFF, 0xFFFFFFFF, -1e30, 3.4e38,
                                                 -3.4e38, 1e-30, 65535.5, (-1.0, 1.0, -0.5, 0.5)).pack())
    ints = (9, lp.STATE_ABORT, 0x80, lp.R_I2_TILT, 9, 0x1234, 0, 777)
    add_payload("snap_nan_inf", TYPE_SNAP, _snap_raw(ints, (QNAN, PINF, NINF, NZERO, NQNAN, QNAN, PINF, NINF, NZERO)),
                note="quiet NaN, +inf, -inf, -0.0, negative quiet NaN in floats and q")
    add_payload("snap_denormals", TYPE_SNAP, _snap_raw(ints, (DENORM_MIN, DENORM_MAX, 0x80000001, NFLT_MIN, FLT_MAX_BITS,
                                                              0x3F800000, DENORM_MIN, 0x807FFFFF, 0x00400000)),
                note="denormals (min / max, negative), -FLT_MIN, FLT_MAX")
    add_payload("snap_nan_payload", TYPE_SNAP, _snap_raw(ints, (QNAN_PAYLOAD, 0x3F800000, 0, 0, 0, 0x3F800000, 0, 0, 0)),
                note="quiet NaN with a payload: kept bit for bit")
    add_payload("snap_signalling_nan", TYPE_SNAP, _snap_raw(ints, (SNAN, 0, 0, 0, 0, 0x3F800000, 0, 0, 0)),
                py_roundtrip=False, note="signalling NaN: the C port copies the 4 bytes (kept); Python quietens it")
    q_sync = struct.unpack("<f", bytes([0xC5, 0x5C, 0x08, 0x01]))[0]
    add_payload("snap_sync_in_payload", TYPE_SNAP, Snap(0xC5, 0x5C, 0xC5, 0x5C, 0x08, 0x5CC5, 0xC55C, 0x015C08C5, q_sync,
                                                        0.0, 0.0, 0.0, 0.0, (q_sync, q_sync, 1.0, 0.0)).pack(),
                note="sync pairs and plausible headers inside a valid SNAP: consumed whole")
    for i in range(4):
        add_payload(f"snap_random_{i}", TYPE_SNAP, Snap(
            rng.randrange(256), rng.randrange(6), rng.randrange(256), rng.randrange(256), rng.randrange(256),
            rng.randrange(65536), rng.randrange(1001), rng.randrange(1 << 32), rng.uniform(-50, 600), rng.uniform(-60, 60),
            rng.uniform(0, 90), rng.uniform(3.0, 4.3), rng.uniform(500, 4000),
            tuple(rng.uniform(-1, 1) for _ in range(4))).pack())
    # ---- STAT
    add_payload("stat_zero", TYPE_STAT, Stat().pack())
    add_payload("stat_boot", TYPE_STAT, Stat(com_ticks=1, reset_cause=0x01, flags=STAT_RADIO_OK | STAT_BOOT,
                                             params_tag=0x5C).pack())
    add_payload("stat_max", TYPE_STAT, Stat(0xFFFFFFFF, 255, 255, 255, 127, 127, 255, *([0xFFFF] * 13)).pack())
    add_payload("stat_min", TYPE_STAT, Stat(0, 0, 0, 0, -128, -128, 0, *([0] * 13)).pack())
    add_payload("stat_sx127x", TYPE_STAT, Stat(250 * 37, 0x08, STAT_RADIO_OK | STAT_SX127X, 0x12, -101, -22, 0xA7,
                                               74, 3, 1, 2, 74, 0, 0, 1, 73, 1, 0, 0, 1234).pack())
    for i in range(3):
        add_payload(f"stat_random_{i}", TYPE_STAT, Stat(rng.randrange(1 << 32), rng.randrange(256), rng.randrange(8),
                                                        rng.randrange(256), rng.randrange(-128, 128),
                                                        rng.randrange(-128, 128), rng.randrange(256),
                                                        *[rng.randrange(65536) for _ in range(13)]).pack())

    # ---- decoder streams
    streams: list[dict] = []

    def add(name: str, data: bytes, max_out: int = DEC_MAX_OUT, note: str = "", accept: int = ACCEPT_ALL) -> None:
        streams.append(_decode_vector(name, data, max_out, note, accept))

    cmd_f, snap_f, stat_f = frames_by_type[TYPE_CMD], frames_by_type[TYPE_SNAP], frames_by_type[TYPE_STAT]
    for v in payloads:                                         # every payload's frame alone
        add(f"single_{v['name']}", bytes.fromhex(v["frame_hex"]))
    add("back_to_back_all", b"".join(bytes.fromhex(v["frame_hex"]) for v in payloads), note="every frame above, no gaps")
    for i in range(4):
        noise = bytes(rng.choice([SYNC0, SYNC1, CMD_LEN, SNAP_LEN, STAT_LEN, TYPE_CMD, TYPE_SNAP, TYPE_STAT,
                                  rng.randrange(256)]) for _ in range(rng.randrange(60, 200)))
        add(f"noise_{i}", noise, note="random bytes rich in sync / LEN / TYPE values")
    noise = bytes(rng.randrange(256) for _ in range(90))
    add("noise_between_frames", noise[:13] + cmd_f[3] + noise[13:50] + snap_f[2] + noise[50:] + stat_f[1] + cmd_f[4],
        note="random noise between valid frames")
    # every single-bit corruption of one CMD frame (followed by a good frame); random ones in SNAP / STAT
    base = cmd_f[8]
    for j in range(len(base)):
        for bit in range(8):
            bad = bytearray(base)
            bad[j] ^= 1 << bit
            add(f"bitflip_cmd_b{j}_{bit}", bytes(bad) + cmd_f[1], note=f"CMD byte {j} bit {bit} flipped, then a good CMD")
    for fr, nm in ((snap_f[2], "snap"), (stat_f[4], "stat")):
        for k in range(24):
            j = rng.randrange(len(fr))
            bit = rng.randrange(8)
            bad = bytearray(fr)
            bad[j] ^= 1 << bit
            add(f"bitflip_{nm}_{k}_b{j}_{bit}", bytes(bad) + cmd_f[2], note=f"{nm.upper()} byte {j} bit {bit} flipped")
    # sync pairs inside payloads
    sync_cmd = next(bytes.fromhex(v["frame_hex"]) for v in payloads if v["name"].startswith("cmd_thr_seq127"))
    add("sync_pair_in_cmd_payload", sync_cmd + cmd_f[0], note="valid CMD whose arg / nonce bytes are C5 5C / 5C C5")
    bad = bytearray(sync_cmd)
    bad[-1] ^= 0xFF
    add("sync_pair_in_bad_cmd", bytes(bad) + cmd_f[0], note="the same with its CRC broken: rejected at the CRC high byte, "
        "the inner pairs are NOT re-hunted (no replay), the CMD behind it is found")
    sync_snap = next(bytes.fromhex(v["frame_hex"]) for v in payloads if v["name"] == "snap_sync_in_payload")
    add("sync_pair_in_snap_payload", sync_snap, note="valid SNAP full of sync pairs: one frame, nothing skipped")
    bad = bytearray(sync_snap)
    bad[-2] ^= 0x01
    add("sync_pair_in_bad_snap", bytes(bad) + stat_f[0], note="the same SNAP with a CRC low byte error, then a STAT: found")
    # frames hidden inside a rejected candidate: LOST since phase 3 (no replay; module docstring, rule 4)
    inner3 = cmd_f[0] + cmd_f[1] + cmd_f[2]
    fake = bytes([SYNC0, SYNC1, SNAP_LEN, TYPE_SNAP]) + inner3 + bytes(SNAP_LEN - len(inner3)) + b"\x00\x00"
    for mo in (1, 2, 3):
        add(f"three_cmds_in_bad_snap_max{mo}", fake, mo,
            note="a fake SNAP header whose body holds 3 CMD frames; its CRC fails at the low byte: the 3 CMDs are lost "
                 "(phase 2 replayed them and returned all 3 on the last byte), max_out > 1 changes nothing")
    hidden = bytes([SYNC0, SYNC1, STAT_LEN, TYPE_STAT]) + snap_f[1][:20] + cmd_f[5] + bytes(rng.randrange(256) for _ in range(40))
    add("cmd_in_bad_stat", hidden, note="fake STAT header, part of a SNAP, a real CMD, noise: the CMD is swallowed and lost")
    add("snap_in_bad_snap", bytes([SYNC0, SYNC1, SNAP_LEN, TYPE_SNAP]) + snap_f[3] + bytes(10),
        note="a real SNAP starts 4 bytes into a fake one: swallowed and lost")
    # the rejecting byte is examined once more as a possible SYNC0: a frame starting AT the rejecting byte is found
    a = cmd_f[3]
    for k, nm in ((1, "sync1"), (2, "len"), (3, "type"), (12, "crc_lo"), (13, "crc_hi")):
        add(f"rejecting_byte_is_sync0_at_{nm}", a[:k] + cmd_f[4],
            note=f"a CMD cut after {k} bytes, the next CMD's 0xC5 at its {nm.upper()} position: rejected there and "
                 "re-examined - the next CMD is found")
    for k in (1, 2):
        cut = bytearray(snap_f[0])
        del cut[20:20 + k]
        add(f"dropped_{k}_bytes_then_cmd", bytes(cut) + cmd_f[2],
            note=f"a SNAP that lost {k} payload byte(s), then a CMD: rejected at a CRC byte at or before the CMD's "
                 "0xC5 - the CMD is found")
    cut = bytearray(snap_f[0])
    del cut[20:23]
    add("dropped_3_bytes_then_cmd", bytes(cut) + cmd_f[2],
        note="3 payload bytes lost: the CMD's 0xC5 is taken as the SNAP's last payload byte - the CMD is lost "
             "(trade-off of rule 4)")
    # sync spam: valid-looking headers back to back (phase 2: 1821 cycles per byte on the AVR; now O(1))
    for hdr in ((SYNC0, SYNC1, SNAP_LEN, TYPE_SNAP), (SYNC0, SYNC1, STAT_LEN, TYPE_STAT), (SYNC0, SYNC1, CMD_LEN, TYPE_CMD)):
        add(f"sync_spam_{TYPE_NAMES[hdr[3]].lower()}", bytes(hdr) * 40 + cmd_f[1],
            note=f"{bytes(hdr).hex(' ')} x 40, then a CMD: each candidate swallows the next headers and fails at a CRC byte")
    # max_out 0: frames counted, not returned
    add("max_out_0", cmd_f[0] + stat_f[0] + snap_f[0], 0, note="every complete frame counts as overflow")
    # cmd_need corner (rule 8): only CRC bytes left and the expected CRC byte is itself 0xC5 -> 15, not 14
    for want_lo, nm in ((True, "lo"), (False, "hi")):
        k = 0
        while True:
            fr = encode_stat(Stat(com_ticks=k, params_tag=0x5C))
            if (fr[-2] if want_lo else fr[-1]) == SYNC0:
                break
            k += 1
        add(f"stat_crc_{nm}_is_sync0", fr + cmd_f[1], note=f"a valid STAT whose CRC {nm} byte is 0xC5 (com_ticks {k}), "
            "then a CMD: cmd_need is 15 just before that byte")
    # truncated frames, each followed by a good frame
    for fr, nm in ((cmd_f[6], "cmd"), (snap_f[4], "snap"), (stat_f[2], "stat")):
        for cut in (1, 2, 3, 4, len(fr) // 2, len(fr) - 2, len(fr) - 1):
            add(f"truncated_{nm}_{cut}", fr[:cut] + cmd_f[7], note=f"{nm.upper()} cut after {cut} bytes, then a good CMD")
        add(f"truncated_{nm}_at_end", fr[:-1], note="stream ends inside a frame: no frame, the candidate stays pending")
    # wrong LEN / TYPE
    for ln in (0, 1, 7, 9, 35, 37, 48, 50, 0xC5, 0xFF):
        bad = bytearray(cmd_f[3])
        bad[2] = ln
        add(f"bad_len_{ln}", bytes(bad) + cmd_f[0], note=f"LEN {ln} is no type's length")
    for typ, ln in ((TYPE_CMD, SNAP_LEN), (TYPE_SNAP, CMD_LEN), (TYPE_STAT, CMD_LEN), (TYPE_CMD, STAT_LEN)):
        body = bytes([ln, typ]) + bytes(rng.randrange(256) for _ in range(ln))
        crc = crc16(body)
        add(f"len_of_other_type_{TYPE_NAMES[typ].lower()}_{ln}", bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8])
            + cmd_f[0], note="valid CRC, LEN of another type: bad_len")
    for typ in (0x00, 0x04, 0x10, 0x20, 0xC5, 0xFF):
        body = bytes([CMD_LEN, typ]) + bytes(range(CMD_LEN))
        crc = crc16(body)
        add(f"bad_type_0x{typ:02x}", bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8]) + cmd_f[0],
            note="valid CRC, unknown TYPE: bad_type")
    # sync edge cases
    add("double_sync0", bytes([SYNC0]) + cmd_f[2], note="C5 C5 5C ...: the second C5 starts the frame")
    add("double_sync_pair", bytes([SYNC0, SYNC1]) + cmd_f[2], note="C5 5C C5 5C ...: LEN = C5 fails (bad_len), that C5 is "
        "re-examined and starts the frame")
    add("sync1_missing", bytes([SYNC0, 0x00, SYNC0, 0x5D]) + cmd_f[2], note="broken sync pairs: skipped only")
    add("lone_sync_at_end", cmd_f[2] + bytes([SYNC0]), note="a lone C5 at the end stays pending")
    add("link_proto_frame", lp.encode_command(vehicle_id, 1, lp.Command(lp.CMD_ARM)) + cmd_f[0],
        note="a radio-link frame on the ilink is noise")
    # long mixed streams
    pool = cmd_f + snap_f + stat_f
    for i in range(6):
        parts = []
        for _ in range(rng.randrange(8, 20)):
            k = rng.random()
            fr = bytearray(rng.choice(pool))
            if k < 0.45:
                pass
            elif k < 0.6:
                fr[rng.randrange(len(fr))] ^= 1 << rng.randrange(8)
            elif k < 0.7:
                fr = fr[:rng.randrange(1, len(fr))]
            elif k < 0.8:
                fr[2] = rng.choice([0, 7, 9, 0xFF, CMD_LEN, SNAP_LEN, STAT_LEN])
            elif k < 0.9:
                j = rng.randrange(len(fr) + 1)
                fr[j:j] = bytes([rng.choice([SYNC0, SYNC1, rng.randrange(256)])])
            else:
                fr = bytearray(rng.choice([SYNC0, SYNC1, rng.randrange(256)]) for _ in range(rng.randrange(1, 30)))
            parts.append(bytes(fr))
        add(f"mixed_{i}", b"".join(parts), rng.choice([1, 2, 2, 3]), note="random mix of good, corrupted, cut frames")

    # ---- direction filter (R4): the streams above through the FCU's (CMD | STAT) and the COM's (SNAP) decoder
    for sv in list(streams):
        nm = sv["name"]
        if nm.startswith("bitflip_cmd_b") and not nm.startswith(("bitflip_cmd_b2_", "bitflip_cmd_b3_")):
            continue                                           # LEN / TYPE flips kept, the other 96 add nothing here
        for acc in (ACCEPT_FCU, ACCEPT_COM):
            add(f"{nm}@{ACCEPT_NAMES[acc]}", bytes.fromhex(sv["hex"]), sv["max_out"], sv["note"], acc)
    add("snap_on_fcu", snap_f[0] + cmd_f[1], accept=ACCEPT_FCU,
        note="a valid SNAP on the FCU's line: rejected at LEN 49 (bad_len), its bytes hunted, the CMD behind it found")
    add("cmd_stat_on_com", cmd_f[0] + stat_f[0] + snap_f[1] + cmd_f[2], accept=ACCEPT_COM,
        note="CMD / STAT on the COM's line: rejected at LEN, the SNAP between them found")
    hdr = bytes([SYNC0, SYNC1, SNAP_LEN, TYPE_SNAP])
    for acc in (ACCEPT_ALL, ACCEPT_FCU):
        add(f"snap_header_then_cmd@{ACCEPT_NAMES[acc]}", hdr + cmd_f[3] + bytes(30) + cmd_f[4], accept=acc,
            note="a SNAP header in garbage right before a CMD: ALL swallows the CMD (lost), the FCU's decoder rejects the "
                 "header at LEN and finds it")
    for typ, ln in ((TYPE_SNAP, CMD_LEN), (TYPE_SNAP, STAT_LEN), (TYPE_CMD, SNAP_LEN), (TYPE_STAT, SNAP_LEN)):
        body = bytes([ln, typ]) + bytes(rng.randrange(256) for _ in range(ln))
        crc = crc16(body)
        fr = bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8])
        for acc in (ACCEPT_FCU, ACCEPT_COM):
            add(f"type_{TYPE_NAMES[typ].lower()}_len_{ln}@{ACCEPT_NAMES[acc]}", fr + cmd_f[0] + snap_f[0], accept=acc,
                note="valid CRC, a TYPE / LEN pair the direction filter refuses")
    for acc in (ACCEPT_FCU, ACCEPT_COM):
        for k in range(3):
            data = bytes(rng.choice([SYNC0, SYNC1, CMD_LEN, SNAP_LEN, STAT_LEN, TYPE_CMD, TYPE_SNAP, TYPE_STAT,
                                     rng.randrange(256)]) for _ in range(rng.randrange(60, 160)))
            data += rng.choice(cmd_f) + rng.choice(snap_f) + rng.choice(stat_f)
            add(f"noise_dir_{k}@{ACCEPT_NAMES[acc]}", data, accept=acc, note="protocol-rich noise, then one frame of each type")

    # ---- v0.7 VTOL build: ON / OFF / UP / DOWN in IL_CMD, SNAP2 (TYPE 4, 55 B), the VTOL COM's mask (SNAP2 only).
    # Own RNG and appended after everything above, so the v0.6.3 vectors keep their bytes (only ACCEPT_ALL changed).
    r2 = random.Random(seed + 7)
    n_pl = len(payloads)
    for code in lp.VTOL_ONLY_CMDS:
        add_payload(f"cmd_{lp.CMD_NAMES[code].lower()}", TYPE_CMD, IlCmd((code * 41) & 0xFF, code, 0,
                                                                        0xACE1 if code == lp.CMD_ON else 0).pack())
    q_hov = (0.99991, 0.0087, -0.0102, 0.0021)
    add_payload("snap2_zero", TYPE_SNAP2, Snap(0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0, 0.0),
                                               phase=lp.PHASE_NONE, h_tgt_m=0.0).pack())
    add_payload("snap2_spool", TYPE_SNAP2, Snap(4, lp.STATE_ARMED, 0x0F, 0, 4, 0xACE1, 0, 2600, 0.0, 0.0, 1.25, 4.05, 2875.0,
                                                (1.0, 0.0, 0.0, 0.0), phase=lp.PHASE_SPOOL, h_tgt_m=5.0).pack())
    add_payload("snap2_hold", TYPE_SNAP2, Snap(21, lp.STATE_POWERED, 0x2F, 0, 21, 0x5A5A, 918, 5346, 4.97, -0.04, 1.68,
                                               3.86, 2950.5, q_hov, phase=lp.PHASE_HOLD, h_tgt_m=6.0).pack())
    add_payload("snap2_land", TYPE_SNAP2, Snap(30, lp.STATE_POWERED, 0x2F, lp.R_V_LINK_LAND, 29, 0x1357, 870, 9000, 0.42,
                                               -0.31, 2.5, 3.7, 3001.0, q_hov, phase=lp.PHASE_LAND, h_tgt_m=0.0).pack())
    add_payload("snap2_landed", TYPE_SNAP2, Snap(31, lp.STATE_UNPOWERED, 0x0F, 0, 31, 0x1357, 0, 9300, -0.25, 0.0, 3.0, 3.75,
                                                 2800.0, (1.0, 0.0, 0.0, 0.0), phase=lp.PHASE_LANDED, h_tgt_m=0.0).pack())
    add_payload("snap2_extremes", TYPE_SNAP2, Snap(255, 255, 255, 255, 255, 0xFFFF, 0xFFFF, 0xFFFFFFFF, -1e30, 3.4e38,
                                                   -3.4e38, 1e-30, 65535.5, (-1.0, 1.0, -0.5, 0.5), phase=255, spare=255,
                                                   h_tgt_m=-3.4e38).pack(), note="spare byte 255: carried, ignored by the COM")
    for h_t in (3276.7, 3276.75, 1e6, -0.05, -3276.85):
        add_payload(f"snap2_target_{h_t:g}", TYPE_SNAP2, Snap(5, lp.STATE_POWERED, 0x2F, 0, 5, 1, 900, 1000, 2.0, 0.1, 1.0,
                                                               3.9, 2900.0, q_hov, phase=lp.PHASE_HOLD, h_tgt_m=h_t).pack(),
                    note="h_tgt quantisation: rounding half away from zero, saturation at the int16 limits")
    ints2 = (9, lp.STATE_ABORT, 0x80, lp.R_V_FALL, 9, 0x1234, 0, 777)
    add_payload("snap2_nan_inf", TYPE_SNAP2, _snap_raw(ints2, (QNAN, PINF, NINF, NZERO, NQNAN, QNAN, PINF, NINF, NZERO),
                                                       (lp.PHASE_NONE, 0, QNAN)), note="NaN / inf in every float, h_tgt NaN")
    add_payload("snap2_h_tgt_inf", TYPE_SNAP2, _snap_raw(ints2, (0, 0, 0, 0, 0, 0x3F800000, 0, 0, 0), (lp.PHASE_LAND, 0, NINF)),
                note="h_tgt -inf saturates to -32768")
    add_payload("snap2_signalling_nan", TYPE_SNAP2, _snap_raw(ints2, (0, 0, 0, 0, 0, 0x3F800000, 0, 0, 0),
                                                              (lp.PHASE_HOLD, 0, SNAN)),
                py_roundtrip=False, note="signalling NaN in h_tgt: the C port copies the 4 bytes; Python quietens it")
    q_sync2 = struct.unpack("<f", bytes([0xC5, 0x5C, SNAP2_LEN, TYPE_SNAP2]))[0]
    add_payload("snap2_sync_in_payload", TYPE_SNAP2, Snap(0xC5, 0x5C, 0xC5, 0x5C, SNAP2_LEN, 0x5CC5, 0xC55C, 0x045C37C5,
                                                          q_sync2, 0.0, 0.0, 0.0, 0.0, (q_sync2, 1.0, 0.0, 0.0), phase=0xC5,
                                                          spare=0x5C, h_tgt_m=q_sync2).pack(),
                note="SNAP2 headers inside a valid SNAP2: consumed whole")
    for i in range(4):
        add_payload(f"snap2_random_{i}", TYPE_SNAP2, Snap(
            r2.randrange(256), r2.randrange(6), r2.randrange(256), r2.randrange(256), r2.randrange(256),
            r2.randrange(65536), r2.randrange(1001), r2.randrange(1 << 32), r2.uniform(-5, 30), r2.uniform(-4, 4),
            r2.uniform(0, 60), r2.uniform(3.0, 4.3), r2.uniform(500, 4000), tuple(r2.uniform(-1, 1) for _ in range(4)),
            phase=r2.randrange(6), spare=0, h_tgt_m=r2.choice([2.0, 5.0, 20.0, r2.uniform(-10, 40)])).pack())
    snap2_f = frames_by_type[TYPE_SNAP2]
    vcmd_f = frames_by_type[TYPE_CMD][-len(lp.VTOL_ONLY_CMDS):]
    for v in payloads[n_pl:]:                                  # each new frame alone, through ALL and its own decoder
        fr = bytes.fromhex(v["frame_hex"])
        add(f"single_{v['name']}", fr)
        add(f"single_{v['name']}@{'comv' if v['type'] == TYPE_SNAP2 else 'fcu'}", fr,
            accept=ACCEPT_COM_VTOL if v["type"] == TYPE_SNAP2 else ACCEPT_FCU)
    add("vtol_back_to_back", b"".join(bytes.fromhex(v["frame_hex"]) for v in payloads[n_pl:]),
        note="every v0.7 frame above, no gaps (ALL)")
    add("vtol_back_to_back@comv", b"".join(bytes.fromhex(v["frame_hex"]) for v in payloads[n_pl:]), accept=ACCEPT_COM_VTOL,
        note="the same on the VTOL COM: only the SNAP2 frames, the CMDs rejected at LEN")
    add("snap_v1_on_vtol_com", snap_f[1] + snap2_f[2] + snap_f[2] + snap2_f[3], accept=ACCEPT_COM_VTOL,
        note="a ballistic SNAP (LEN 49) on the VTOL COM's line: bad_len, the SNAP2 frames found")
    add("snap2_on_ballistic_com", snap2_f[2] + snap_f[1] + snap2_f[3] + snap_f[2], accept=ACCEPT_COM,
        note="SNAP2 (LEN 55) on the ballistic COM's line: bad_len, the SNAP frames found")
    add("snap2_on_fcu", snap2_f[2] + vcmd_f[0] + stat_f[0], accept=ACCEPT_FCU,
        note="a SNAP2 on the FCU's line: bad_len, the CMD (ON) and the STAT behind it found")
    add("cmd_stat_on_vtol_com", vcmd_f[1] + stat_f[1] + snap2_f[4] + cmd_f[2], accept=ACCEPT_COM_VTOL,
        note="CMD / STAT on the VTOL COM's line: rejected at LEN, the SNAP2 between them found")
    hdr2 = bytes([SYNC0, SYNC1, SNAP2_LEN, TYPE_SNAP2])
    for acc in (ACCEPT_ALL, ACCEPT_FCU, ACCEPT_COM_VTOL):
        add(f"snap2_header_then_cmd@{ACCEPT_NAMES[acc]}", hdr2 + vcmd_f[2] + bytes(30) + vcmd_f[3] + snap2_f[0], accept=acc,
            note="a SNAP2 header in garbage before a CMD: ALL / VTOL COM swallow the CMD (lost), the FCU finds both CMDs")
    for typ, ln in ((TYPE_SNAP2, SNAP_LEN), (TYPE_SNAP, SNAP2_LEN), (TYPE_SNAP2, CMD_LEN), (TYPE_CMD, SNAP2_LEN)):
        body = bytes([ln, typ]) + bytes(r2.randrange(256) for _ in range(ln))
        crc = crc16(body)
        fr = bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8])
        for acc in (ACCEPT_ALL, ACCEPT_COM_VTOL):
            add(f"type_{TYPE_NAMES[typ].lower()}_len_{ln}@{ACCEPT_NAMES[acc]}", fr + snap2_f[1] + cmd_f[0], accept=acc,
                note="valid CRC, a TYPE / LEN pair that does not match: bad_len")
    base2 = snap2_f[2]
    for k in range(32):
        j = r2.randrange(len(base2))
        bit = r2.randrange(8)
        bad = bytearray(base2)
        bad[j] ^= 1 << bit
        for acc in (ACCEPT_ALL, ACCEPT_COM_VTOL):
            add(f"bitflip_snap2_{k}_b{j}_{bit}@{ACCEPT_NAMES[acc]}", bytes(bad) + snap2_f[3] + vcmd_f[0], accept=acc,
                note=f"SNAP2 byte {j} bit {bit} flipped, then a SNAP2 and an ON")
    for cut in (1, 2, 3, 4, 30, len(base2) - 2, len(base2) - 1):
        for acc in (ACCEPT_ALL, ACCEPT_COM_VTOL):
            add(f"truncated_snap2_{cut}@{ACCEPT_NAMES[acc]}", base2[:cut] + snap2_f[4], accept=acc,
                note=f"SNAP2 cut after {cut} bytes, then a good SNAP2")
    for acc in (ACCEPT_ALL, ACCEPT_COM_VTOL):
        add(f"sync_spam_snap2@{ACCEPT_NAMES[acc]}", hdr2 * 40 + snap2_f[1], accept=acc,
            note="C5 5C 37 04 x 40, then a SNAP2")
        add(f"max_out_0_snap2@{ACCEPT_NAMES[acc]}", snap2_f[0] + snap2_f[1], 0, accept=acc,
            note="complete SNAP2 frames count as overflow")
    for k in range(4):
        data = bytes(r2.choice([SYNC0, SYNC1, CMD_LEN, SNAP_LEN, STAT_LEN, SNAP2_LEN, TYPE_CMD, TYPE_SNAP, TYPE_STAT,
                                TYPE_SNAP2, r2.randrange(256)]) for _ in range(r2.randrange(60, 200)))
        data += r2.choice(snap2_f) + r2.choice(cmd_f + vcmd_f) + r2.choice(stat_f) + r2.choice(snap_f)
        for acc in (ACCEPT_ALL, ACCEPT_FCU, ACCEPT_COM, ACCEPT_COM_VTOL):
            add(f"noise_v07_{k}@{ACCEPT_NAMES[acc]}", data, accept=acc,
                note="noise rich in sync / LEN / TYPE values incl. SNAP2's, then one frame of each type")
    for sv in [s for s in streams if s["name"].startswith(("noise_", "mixed_", "back_to_back_all", "sync_spam_"))
               and "@" not in s["name"]]:
        add(f"{sv['name']}@comv", bytes.fromhex(sv["hex"]), sv["max_out"], sv["note"], ACCEPT_COM_VTOL)
    pool2 = cmd_f + vcmd_f + snap_f + stat_f + snap2_f
    for i in range(4):
        parts = []
        for _ in range(r2.randrange(8, 20)):
            k = r2.random()
            fr = bytearray(r2.choice(pool2))
            if k < 0.5:
                pass
            elif k < 0.65:
                fr[r2.randrange(len(fr))] ^= 1 << r2.randrange(8)
            elif k < 0.8:
                fr = fr[:r2.randrange(1, len(fr))]
            elif k < 0.9:
                fr[2] = r2.choice([CMD_LEN, SNAP_LEN, STAT_LEN, SNAP2_LEN, 0, 0xFF])
            else:
                j = r2.randrange(len(fr) + 1)
                fr[j:j] = bytes([r2.choice([SYNC0, SYNC1, r2.randrange(256)])])
            parts.append(bytes(fr))
        data = b"".join(parts)
        mo = r2.choice([1, 2, 3])
        for acc in (ACCEPT_ALL, ACCEPT_FCU, ACCEPT_COM_VTOL):
            add(f"mixed_v07_{i}@{ACCEPT_NAMES[acc]}", data, mo, note="random mix incl. SNAP2: good, corrupted, cut frames",
                accept=acc)

    # ---- COM pipeline: SNAP -> TELEM frame (SNAP2 -> TELEM2)
    telem: list[dict] = []

    def add_telem(name: str, payload: bytes, note: str = "") -> None:
        s = Snap.unpack(payload)
        fr = telem_frame_from_snap(s, vehicle_id, loop_hz)
        r, p, y = euler_deg(s.q)
        telem.append({"name": name, "note": note, "snap_hex": payload.hex(), "telem_frame_hex": fr.hex(),
                      "angles_deg": [r, p, y] if all(map(math.isfinite, (r, p, y))) else None})

    for v in payloads:
        if v["type"] in SNAP_TYPES:
            add_telem(v["name"], bytes.fromhex(v["payload_hex"]), v["note"])
    for i in range(16):                                        # plausible attitudes (tilt < 80 deg), any heading
        tilt, az, yaw = math.radians(rng.uniform(0, 80)), rng.uniform(-math.pi, math.pi), rng.uniform(-math.pi, math.pi)
        ax, ay = math.cos(az), math.sin(az)
        qt = (math.cos(tilt / 2), ax * math.sin(tilt / 2), ay * math.sin(tilt / 2), 0.0)
        qy = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        w = qy[0] * qt[0] - qy[3] * qt[3]
        x = qy[0] * qt[1] - qy[3] * qt[2]
        y = qy[0] * qt[2] + qy[3] * qt[1]
        z = qy[0] * qt[3] + qy[3] * qt[0]
        s = Snap(rng.randrange(256), rng.randrange(6), rng.randrange(256), rng.randrange(256), rng.randrange(256),
                 rng.randrange(65536), rng.randrange(1001), rng.randrange(1 << 32), rng.uniform(-100, 3500),
                 rng.uniform(-400, 400), math.degrees(tilt), rng.uniform(0, 70), rng.uniform(0, 70000), (w, x, y, z))
        add_telem(f"telem_attitude_{i}", s.pack())

    v08 = _make_v08_vectors(vehicle_id, loop_hz, seed)
    v09 = _make_v09_vectors(vehicle_id, loop_hz, seed)

    return {
        "_doc": ("ilink (FCU <-> COM, SPEC 10.4) vectors generated by sim/ilink_proto.py make_test_vectors(). payloads: "
                 "fields (floats as uint32 bit patterns) <-> payload bytes <-> encoded frame. streams: fed byte by byte to a "
                 "fresh IlinkDecoder(accept) with max_out per input byte (accept: ACCEPT_* mask, 15 = all, 5 = FCU, 2 = COM "
                 "(ballistic), 8 = COM (VTOL, SNAP2 only)); expected_frames, every counter, the pending candidate "
                 "length and need / cmd_need after every byte (need_hex / cmd_need_hex, one byte each) are normative. "
                 "telem: the COM pipeline SNAP -> TELEM frame and SNAP2 -> TELEM2 frame (SEQ = reply_seq); roll / pitch / yaw "
                 "fields may differ by one 0.01 deg quantum (float32 C vs float64 reference), all other bytes are exact."),
        "seed": seed, "vehicle_id": vehicle_id, "loop_hz": loop_hz,
        "crc16_check": {"ascii": "123456789", "crc": crc16(b"123456789")},
        "constants": {"SYNC": [SYNC0, SYNC1], "TYPE_CMD": TYPE_CMD, "TYPE_SNAP": TYPE_SNAP, "TYPE_STAT": TYPE_STAT,
                      "TYPE_SNAP2": TYPE_SNAP2, "CMD_LEN": CMD_LEN, "SNAP_LEN": SNAP_LEN, "STAT_LEN": STAT_LEN,
                      "SNAP2_LEN": SNAP2_LEN, "FRAME_OVERHEAD": FRAME_OVERHEAD, "FRAME_MAX_BYTES": FRAME_MAX_BYTES,
                      "DEC_MAX_OUT": DEC_MAX_OUT, "ACCEPT_CMD": ACCEPT_CMD, "ACCEPT_SNAP": ACCEPT_SNAP,
                      "ACCEPT_STAT": ACCEPT_STAT, "ACCEPT_SNAP2": ACCEPT_SNAP2, "ACCEPT_ALL": ACCEPT_ALL,
                      "ACCEPT_FCU": ACCEPT_FCU, "ACCEPT_COM": ACCEPT_COM, "ACCEPT_COM_VTOL": ACCEPT_COM_VTOL,
                      "NEED_NEVER": NEED_NEVER},
        "stat_keys": list(DEC_STAT_KEYS),
        "payloads": payloads,
        "streams": streams,
        "telem": telem,
        "v08": v08,
        "v09": v09,
    }


def _make_v08_vectors(vehicle_id: int, loop_hz: int, seed: int) -> dict:
    """v0.8 SNAP3 vectors (own RNG; the v0.6.3 / v0.7 blocks keep their bytes): payloads, decoder streams on the VTOL +
    GPS COM (ACCEPT_COM_GPS), on ACCEPT_ALL8 and on the FCU, and the COM pipeline SNAP3 -> TELEM3."""
    rng = random.Random(seed + 8)
    payloads: list[dict] = []
    frames: list[bytes] = []

    def add_payload(name: str, s: Snap, note: str = "") -> None:
        pl = s.pack()
        fr = encode(TYPE_SNAP3, pl)
        frames.append(fr)
        payloads.append({"name": name, "note": note, "type": TYPE_SNAP3, "payload_hex": pl.hex(), "frame_hex": fr.hex(),
                         "fields": _fields(TYPE_SNAP3, pl)})

    add_payload("snap3_zero", Snap(0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0, 0.0), phase=0,
                                   h_tgt_m=0.0, gps_flags=0, num_sv=0, hacc_dm=0, pos_mode=0, dist_m=0.0))
    add_payload("snap3_pad_no_gps", Snap(3, lp.STATE_SAFE, 0x0F, lp.REJ_GPS, 3, 0xACE1, 0, 2600, 0.0, 0.0, 1.25, 4.05,
                                         2875.0, (1.0, 0.0, 0.0, 0.0), phase=lp.PHASE_NONE, h_tgt_m=0.0, gps_flags=0x30,
                                         num_sv=4, hacc_dm=87, pos_mode=0, dist_m=math.nan),
                note="ON rejected: GPS not ready; no hold point (NaN -> TELEM3 dist 0xFFFF)")
    add_payload("snap3_hold", Snap(21, lp.STATE_POWERED, 0x2F, 0, 21, 0x5A5A, 918, 5346, 1.52, -0.04, 3.68, 3.86,
                                   2875.0, (0.9995, 0.012, -0.028, 0.0087), phase=lp.PHASE_HOLD, h_tgt_m=1.5,
                                   gps_flags=0x7F, num_sv=9, hacc_dm=25, pos_mode=2, dist_m=0.734))
    add_payload("snap3_fence_land", Snap(30, lp.STATE_POWERED, 0xAF, lp.R_V_FENCE, 29, 0x1357, 930, 9000, 1.1, -0.4,
                                         9.5, 3.71, 2900.0, (0.996, 0.06, -0.06, 0.0), phase=lp.PHASE_LAND, h_tgt_m=0.0,
                                         gps_flags=0x2B, num_sv=8, hacc_dm=31, pos_mode=2, dist_m=5.06))
    add_payload("snap3_extremes", Snap(255, 255, 255, 255, 255, 0xFFFF, 0xFFFF, 0xFFFFFFFF, -1e30, 3.4e38, 1e-30, -0.0,
                                       65535.0, (-1.0, 1.0, -0.5, 0.5), phase=255, h_tgt_m=-3.4e38, gps_flags=255,
                                       num_sv=255, hacc_dm=255, pos_mode=255, dist_m=1e6))
    for i in range(4):
        add_payload(f"snap3_random_{i}", Snap(
            rng.randrange(256), rng.randrange(6), rng.randrange(256), rng.randrange(256), rng.randrange(256),
            rng.randrange(65536), rng.randrange(1001), rng.randrange(1 << 32), rng.uniform(-5, 40), rng.uniform(-5, 5),
            rng.uniform(0, 45), rng.uniform(3.3, 4.2), rng.uniform(0, 4000), tuple(rng.uniform(-1, 1) for _ in range(4)),
            phase=rng.randrange(6), h_tgt_m=rng.uniform(0, 10), gps_flags=rng.randrange(256), num_sv=rng.randrange(16),
            hacc_dm=rng.randrange(256), pos_mode=rng.randrange(5), dist_m=rng.uniform(0, 20)))
    streams: list[dict] = []

    def add(name: str, data: bytes, accept: int, note: str = "", max_out: int = DEC_MAX_OUT) -> None:
        streams.append(_decode_vector(name, data, max_out, note, accept))

    snap2 = encode(TYPE_SNAP2, Snap(4, 1, 0x0F, 0, 4, 1, 0, 2600, 0.0, 0.0, 1.0, 4.0, 2000.0, (1.0, 0.0, 0.0, 0.0),
                                    phase=1, h_tgt_m=1.5).pack())
    cmd_on = encode_cmd(IlCmd(12, lp.CMD_ON, 0, 0xACE1, -71, 23))
    for acc in (ACCEPT_COM_GPS, ACCEPT_ALL8, ACCEPT_FCU, ACCEPT_COM_VTOL):
        tag = ACCEPT_NAMES[acc]
        add(f"snap3_back_to_back@{tag}", b"".join(frames), acc, "every SNAP3 frame back to back")
        add(f"snap2_snap3_cmd@{tag}", snap2 + frames[2] + cmd_on + frames[3], acc,
            "a SNAP2, a SNAP3, an ON command, a SNAP3: each decoder takes only its own TYPEs")
    for j in (2, 3, 10, 40, 66, 67):
        bad = bytearray(frames[2])
        bad[j] ^= 0x04
        for acc in (ACCEPT_COM_GPS, ACCEPT_ALL8):
            add(f"snap3_bitflip_b{j}@{ACCEPT_NAMES[acc]}", bytes(bad) + frames[3], acc,
                f"SNAP3 byte {j} flipped, then a good SNAP3")
    noise = bytes(rng.choice([SYNC0, SYNC1, SNAP3_LEN, TYPE_SNAP3, CMD_LEN, STAT_LEN, rng.randrange(256)])
                  for _ in range(120))
    for acc in (ACCEPT_COM_GPS, ACCEPT_ALL8, ACCEPT_FCU):
        add(f"noise_then_snap3@{ACCEPT_NAMES[acc]}", noise + frames[1] + cmd_on, acc,
            "noise rich in SNAP3 sync / LEN / TYPE values, then a SNAP3 and an ON")
    telem: list[dict] = []
    for v in payloads:
        s = Snap.unpack(bytes.fromhex(v["payload_hex"]))
        if not all(math.isfinite(c) for c in s.q):
            continue
        fr = telem_frame_from_snap(s, vehicle_id, loop_hz)
        telem.append({"name": v["name"], "snap_hex": v["payload_hex"], "telem_frame_hex": fr.hex()})
    return {"_doc": ("v0.8 SNAP3 (TYPE 5, 62 B: SNAP2 with the spare byte = gps_flags, then num_sv, hacc_dm, pos_mode, "
                     "dist_m f32): payloads, decoder streams (accept 16 = the VTOL + GPS COM, 31 = every v0.8 TYPE, "
                     "5 = FCU, 8 = the v0.7 VTOL COM) and the COM pipeline SNAP3 -> TELEM3 frame (dist NaN -> 0xFFFF; "
                     "roll / pitch / yaw within one quantum as for v0.7)."),
            "constants": {"TYPE_SNAP3": TYPE_SNAP3, "SNAP3_LEN": SNAP3_LEN, "SNAP3_FRAME_BYTES": SNAP3_FRAME_BYTES,
                          "ACCEPT_SNAP3": ACCEPT_SNAP3, "ACCEPT_ALL8": ACCEPT_ALL8, "ACCEPT_COM_GPS": ACCEPT_COM_GPS,
                          "FRAME_MAX_BYTES": FRAME_MAX_BYTES},
            "payloads": payloads, "streams": streams, "telem": telem}


def _make_v09_vectors(vehicle_id: int, loop_hz: int, seed: int) -> dict:
    """v0.9 RP1 vectors (own RNG; the blocks above keep their draws): SNAP2 / SNAP3 payloads with the power tail of each
    build (battery 0 / 0, wired, wired_buffer, the flag extremes), decoder streams on the VTOL COM (SNAP2 | SNAP3), ALL8
    and the FCU, and the COM pipeline SNAP2 -> TELEM2 / TELEM4, SNAP3 -> TELEM3 / TELEM5 (link_proto.power_tail_due)."""
    rng = random.Random(seed + 9)
    payloads: list[dict] = []
    frames: list[bytes] = []

    def add_payload(name: str, s: Snap, note: str = "") -> None:
        pl = s.pack()
        fr = encode(s.wire_type, pl)
        frames.append(fr)
        payloads.append({"name": name, "note": note, "type": s.wire_type, "payload_hex": pl.hex(), "frame_hex": fr.hex(),
                         "fields": _fields(s.wire_type, pl)})

    q_h = (0.9995, 0.012, -0.028, 0.0087)
    base2 = dict(reply_seq=21, state=lp.STATE_POWERED, flags=0x2F, reason=0, ack_seq=21, nonce=0x5A5A, throttle_milli=705,
                 tick=5346, h_m=1.21, vz_mps=-0.04, tilt_deg=1.68, v_cell=4.12, loop_max_us=2875.0, q=q_h,
                 phase=lp.PHASE_HOLD, h_tgt_m=1.2)
    g3 = dict(gps_flags=0x7B, num_sv=9, hacc_dm=25, pos_mode=2, dist_m=0.734)
    for nm, va, fl in (("battery", 0, 0x00), ("wired", 202, 0x41), ("wired_sag", 160, 0x43),
                       ("wired_buffer", 168, 0x89), ("wired_buffer_lost", 166, 0x9C), ("build3_extremes", 255, 0xFF),
                       ("battery_flags_no_build", 99, 0x3F)):
        add_payload(f"snap2_pwr_{nm}", Snap(**base2, pw_vaux=va, pw_flags=fl),
                    "build bits 0: TELEM2 (no tail) whatever the other bits" if not (fl >> 6) else "")
        add_payload(f"snap3_pwr_{nm}", Snap(**base2, **g3, pw_vaux=va, pw_flags=fl, spare3=0))
    add_payload("snap3_pwr_spare3", Snap(**base2, **g3, pw_vaux=1, pw_flags=0x41, spare3=0xA5),
                "spare3 0xA5: carried, ignored by the COM")
    for i in range(4):
        b = dict(base2, reply_seq=rng.randrange(256), state=rng.randrange(6), flags=rng.randrange(256),
                 reason=rng.choice([0, lp.R_W_LOST, lp.R_W_SAG, lp.R_W_BUF, lp.R_W_D2, lp.REJ_POWER]),
                 tick=rng.randrange(1 << 32), h_m=rng.uniform(-1, 3), q=tuple(rng.uniform(-1, 1) for _ in range(4)),
                 phase=rng.randrange(6), h_tgt_m=rng.uniform(0, 3))
        add_payload(f"snap2_pwr_random_{i}", Snap(**b, pw_vaux=rng.randrange(256), pw_flags=rng.randrange(256)))
        add_payload(f"snap3_pwr_random_{i}", Snap(**b, gps_flags=rng.randrange(256), num_sv=rng.randrange(16),
                                                  hacc_dm=rng.randrange(256), pos_mode=rng.randrange(5),
                                                  dist_m=rng.uniform(0, 20), pw_vaux=rng.randrange(256),
                                                  pw_flags=rng.randrange(256), spare3=0))
    streams: list[dict] = []

    def add(name: str, data: bytes, accept: int, note: str = "", max_out: int = DEC_MAX_OUT) -> None:
        streams.append(_decode_vector(name, data, max_out, note, accept))

    cmd_on = encode_cmd(IlCmd(12, lp.CMD_ON, 0, 0xACE1, -71, 23))
    com = ACCEPT_COM_VTOL | ACCEPT_COM_GPS
    for acc, tag in ((com, "comvg"), (ACCEPT_ALL8, "all8"), (ACCEPT_FCU, "fcu"), (ACCEPT_COM, "com")):
        add(f"pwr_back_to_back@{tag}", b"".join(frames), acc, "every v0.9 SNAP2 / SNAP3 frame back to back")
        add(f"pwr_snap_cmd_mix@{tag}", frames[1] + cmd_on + frames[2] + frames[5] + cmd_on, acc,
            "SNAP3, ON, SNAP2, SNAP3, ON: each decoder takes only its own TYPEs")
    for j in (2, 3, 10, 55, 56, 62, 64, 69):
        bad = bytearray(frames[7])                             # snap3_pwr_wired_buffer (71 bytes)
        bad[j] ^= 0x08
        add(f"pwr_snap3_bitflip_b{j}@comvg", bytes(bad) + frames[6], com, f"SNAP3 byte {j} flipped, then a SNAP2")
    for ln in (55, 62, 64):                                    # the v0.8 lengths and 64 on the v0.9 COM: bad_len
        body = bytes([ln, TYPE_SNAP3 if ln != 55 else TYPE_SNAP2]) + bytes(rng.randrange(256) for _ in range(ln))
        crc = crc16(body)
        add(f"pwr_old_len_{ln}@comvg", bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8]) + frames[3], com,
            f"a valid-CRC frame with LEN {ln} (v0.8 lengths / 64): bad_len, the SNAP behind it found")
    telem: list[dict] = []
    for v in payloads:
        sn = Snap.unpack(bytes.fromhex(v["payload_hex"]))
        if not all(math.isfinite(c) for c in sn.q):
            continue
        fr = telem_frame_from_snap(sn, vehicle_id, loop_hz)
        telem.append({"name": v["name"], "snap_hex": v["payload_hex"], "telem_frame_hex": fr.hex()})
    return {"_doc": ("v0.9 RP1: SNAP2 (57 B: + pw_vaux, pw_flags) / SNAP3 (65 B: + pw_vaux, pw_flags, spare3) payloads of "
                     "every power build, decoder streams (accept 24 = the VTOL COM, 31 = ALL8, 5 = FCU, 2 = ballistic COM) "
                     "and the COM pipeline: a snapshot whose build bits (pw_flags >> 6) are not 0 -> TELEM4 (0x23, 41 B) / "
                     "TELEM5 (0x24, 47 B) with the two tail bytes, else TELEM2 / TELEM3."),
            "constants": {"SNAP2_LEN": SNAP2_LEN, "SNAP3_LEN": SNAP3_LEN, "TYPE_TELEM4": lp.TYPE_TELEM4,
                          "TYPE_TELEM5": lp.TYPE_TELEM5, "TELEM4_FRAME_BYTES": lp.TELEM4_FRAME_BYTES,
                          "TELEM5_FRAME_BYTES": lp.TELEM5_FRAME_BYTES},
            "payloads": payloads, "streams": streams, "telem": telem}


def write_test_vectors(path: str = VECTORS_PATH) -> dict:
    vid, hz = load_link_params()
    v = make_test_vectors(vid, hz)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(v, f, indent=1)
        f.write("\n")
    return v


if __name__ == "__main__":  # pragma: no cover
    _v = write_test_vectors()
    sys.stdout.write(f"wrote {os.path.relpath(VECTORS_PATH, os.path.join(_HERE, '..'))}: {len(_v['payloads'])} payloads, "
                     f"{len(_v['streams'])} streams, {len(_v['telem'])} telem\n")
