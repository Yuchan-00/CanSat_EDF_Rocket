"""Tests of the FCU <-> COM ilink reference (``sim/ilink_proto.py``, SPEC 10.4, v0.6).

Independent checks, not the code checking itself: constants and struct layouts against the C header
(parsed here), the byte-wise CRC update against the bitwise definition, payload layouts against
hand-placed bytes, the decoder rules (module docstring, normative for ``il_decoder_feed`` /
``il_decoder_need`` / ``il_decoder_cmd_need``) against counts worked out by hand, sweeps of every
single-byte error and of two dropped bytes, ``cmd_need`` attained and never beaten (with the ALL and the FCU accept
mask), the v0.6.3 direction filter (FCU = CMD | STAT, COM = SNAP) worked out by hand, the Euler formulas against
``sim.quat.to_euler``, and the COM TELEM pipeline against the
telemetry ``sim.onboard.OnboardSupervisor`` itself builds for the same tick.  v0.7: SNAP2 (TYPE 4, 55 B, the VTOL
build: + phase, spare, h_tgt_m) by hand, the VTOL COM's mask (SNAP2 only), SNAP2 -> TELEM2 against the VTOL supervisor's
own TELEM2, and its C header constants (``test_snap2_constants_match_header`` fails until the C port adds them).  Writes
``sim/out/ilink_vectors.json`` (the firmware host test's vectors, ``tools/build_host_test.py``).

Run: ``.venv/Scripts/python.exe -m pytest sim/test_ilink_proto.py -q -p no:cacheprovider``.
"""
from __future__ import annotations

import copy
import json
import math
import os
import random
import re
import struct
import sys

import numpy as np
import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sim import ilink_proto as il  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim import quat  # noqa: E402
from sim.onboard import OnboardInputs, OnboardSupervisor, SupervisorParams, UplinkFrame  # noqa: E402

CORE = os.path.join(_ROOT, "firmware", "atmega128_fc", "src", "core")
HEADER = open(os.path.join(CORE, "ilink_proto.h"), encoding="utf-8").read()
CSRC = open(os.path.join(CORE, "ilink_proto.c"), encoding="utf-8").read()
VID, HZ = il.load_link_params()


def _hdr_const(name: str) -> int:
    m = re.search(rf"#define\s+{name}\s+(0x[0-9A-Fa-f]+|\d+)u?\b", HEADER)
    assert m, f"{name} not in ilink_proto.h"
    return int(m.group(1), 0)


def _struct_layout(name: str) -> tuple[str, list[str]]:
    """struct-module format and member names of ``typedef struct {...} name;`` in ilink_proto.h."""
    m = re.search(r"typedef struct \{([^{}]*)\}\s*" + name + ";", HEADER)
    assert m, name
    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)
    codes = {"uint8_t": "B", "int8_t": "b", "uint16_t": "H", "int16_t": "h", "uint32_t": "I", "float": "f"}
    fmt, names = "<", []
    for decl in body.split(";"):
        decl = decl.strip()
        if not decl:
            continue
        typ, rest = decl.split(None, 1)
        for var in (v.strip() for v in rest.split(",")):
            arr = re.match(r"(\w+)\[(\d+)\]", var)
            if typ == "lp_cmd_t":                                # lp_cmd_t: cmd u8, arg u16, nonce u16
                fmt += "BHH"
                names += ["cmd", "arg", "nonce"]
            elif arr:
                fmt += codes[typ] * int(arr.group(2))
                names += [arr.group(1)] * int(arr.group(2))
            else:
                fmt += codes[typ]
                names.append(var)
    return fmt, names


def _f32(x: float) -> float:
    return il.f32(x)


# --------------------------------------------------------------------------- #
# constants and layouts == the C header
# --------------------------------------------------------------------------- #
def test_constants_match_header():
    assert (_hdr_const("IL_SYNC0"), _hdr_const("IL_SYNC1")) == (il.SYNC0, il.SYNC1) == (0xC5, 0x5C)
    assert (_hdr_const("IL_TYPE_CMD"), _hdr_const("IL_TYPE_SNAP"), _hdr_const("IL_TYPE_STAT")) == (1, 2, 3)
    assert (_hdr_const("IL_CMD_LEN"), _hdr_const("IL_SNAP_LEN"), _hdr_const("IL_STAT_LEN")) == (il.CMD_LEN, il.SNAP_LEN,
                                                                                                  il.STAT_LEN) == (8, 49, 36)
    assert _hdr_const("IL_FRAME_OVERHEAD") == il.FRAME_OVERHEAD == 6
    assert _hdr_const("IL_DEC_MAX_OUT") == il.DEC_MAX_OUT
    assert (_hdr_const("IL_ACCEPT_CMD"), _hdr_const("IL_ACCEPT_SNAP"), _hdr_const("IL_ACCEPT_STAT")) == (
        il.ACCEPT_CMD, il.ACCEPT_SNAP, il.ACCEPT_STAT) == (1, 2, 4)
    assert il.ACCEPT_FCU == il.ACCEPT_CMD | il.ACCEPT_STAT and il.ACCEPT_COM == il.ACCEPT_SNAP
    assert il.ACCEPT_ALL == 15 and il.ACCEPT_SNAP2 == 8 and il.ACCEPT_COM_VTOL == il.ACCEPT_SNAP2   # v0.7: + SNAP2
    assert re.search(r"#define\s+IL_ACCEPT_FCU\s+\(IL_ACCEPT_CMD \| IL_ACCEPT_STAT\)", HEADER)
    assert re.search(r"#define\s+IL_ACCEPT_COM\s+IL_ACCEPT_SNAP\b", HEADER)
    assert _hdr_const("IL_NEED_NEVER") == il.NEED_NEVER == 0xFF
    assert "flen" not in HEADER and "flen" not in CSRC                     # R5: the deprecated field is gone
    assert "il_decoder_init(il_decoder_t *d, uint8_t accept)" in HEADER
    assert (_hdr_const("IL_STAT_RADIO_OK"), _hdr_const("IL_STAT_BOOT"), _hdr_const("IL_STAT_SX127X")) == (
        il.STAT_RADIO_OK, il.STAT_BOOT, il.STAT_SX127X)
    assert il.SNAP_FRAME_BYTES == 55 and il.CMD_FRAME_BYTES == 14 and il.STAT_FRAME_BYTES == 42
    assert il.SNAP2_FRAME_BYTES == 63 and il.SNAP2_LEN == 57 and il.TYPE_SNAP2 == 4      # v0.9 RP1: + the power tail
    # v0.8: SNAP3 (TYPE 5) - the longest frame; ACCEPT_ALL keeps its v0.7 value, ACCEPT_ALL8 adds SNAP3; v0.9 RP1: 65 B
    assert il.FRAME_MAX_BYTES == il.SNAP3_FRAME_BYTES == 71 and il.SNAP3_LEN == 65 and il.TYPE_SNAP3 == 5
    assert il.ACCEPT_SNAP3 == 16 and il.ACCEPT_ALL8 == 31 and il.ACCEPT_COM_GPS == il.ACCEPT_SNAP3


def test_struct_layouts_match_header():
    fmt, names = _struct_layout("il_cmd_t")
    assert fmt == il.CMD_FMT and names == ["seq", "cmd", "arg", "nonce", "rssi_dbm", "snr_q4"]
    assert list(il.CMD_FIELDS) == names
    fmt, names = _struct_layout("il_snap_t")
    assert fmt == il.SNAP_FMT and names == list(il.SNAP_INT_FIELDS + il.SNAP_FLOAT_FIELDS) + ["q"] * 4
    fmt, names = _struct_layout("il_stat_t")
    assert fmt == il.STAT_FMT and names == list(il.STAT_FIELDS)
    assert struct.calcsize(il.STAT_FMT) == 36 and struct.calcsize(il.SNAP_FMT) == 49


# --------------------------------------------------------------------------- #
# CRC
# --------------------------------------------------------------------------- #
def _crc_bitwise(crc: int, b: int) -> int:
    """CRC-16/CCITT-FALSE, one byte, MSB first, by its definition."""
    crc ^= b << 8
    for _ in range(8):
        crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def test_crc16_update_is_the_bitwise_crc_and_the_link_crc():
    """D3: the byte-wise update (il_crc16_update) == the bitwise definition == lp_crc16; the C port has no nibble table
    any more and its header carries the same expression (the host test checks C on all 65536 x 256 inputs)."""
    rng = random.Random(5)
    for crc in list(range(0, 65536, 1021)) + [rng.randrange(65536) for _ in range(100)]:
        for b in range(256):
            assert il.crc16_update(crc, b) == _crc_bitwise(crc, b), (crc, b)
    for _ in range(300):
        data = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 80)))
        init = rng.choice([il.CRC_INIT, rng.randrange(65536)])
        c = init
        for b in data:
            c = il.crc16_update(c, b)
        assert c == il.crc16(data, init) == lp.crc16_ccitt(data, init)
    assert il.crc16(b"123456789") == 0x29B1
    assert "k_crc_nib" not in CSRC
    body = HEADER[HEADER.index("il_crc16_update(uint16_t crc, uint8_t b)"):]
    body = re.sub(r"\s+", " ", body[:body.index("}")])
    for frag in ("x ^= (uint8_t)(x >> 4)", "(uint8_t)crc ^ (uint8_t)(x << 4) ^ (uint8_t)(x >> 3)", "(uint8_t)(x << 5) ^ x"):
        assert frag in body, frag


# --------------------------------------------------------------------------- #
# payloads and frames
# --------------------------------------------------------------------------- #
def test_payload_layouts_hand_placed():
    assert il.IlCmd(0x7F, 4, 0xBEEF, 0x1234, -97, -6).pack() == bytes([0x7F, 4, 0xEF, 0xBE, 0x34, 0x12, 0x9F, 0xFA])
    s = il.Snap(1, 2, 3, 4, 5, 0x0706, 0x0908, 0x0D0C0B0A, 1.0, -2.0, 0.5, 4.0, 2500.0, (1.0, 0.0, -0.0, 0.25))
    b = s.pack()
    assert b[:13] == bytes(range(1, 14))
    assert b[13:17] == struct.pack("<f", 1.0) and b[17:21] == struct.pack("<f", -2.0) and b[29:33] == struct.pack("<f", 2500.0)
    assert b[33:37] == bytes([0, 0, 0x80, 0x3F]) and b[41:45] == bytes([0, 0, 0, 0x80]) and b[45:49] == struct.pack("<f", 0.25)
    st = il.Stat(com_ticks=0x04030201, reset_cause=5, flags=6, radio_version=7, rssi_dbm=-1, snr_q4=-2, params_tag=0xF8,
                 loop_us_max=0xBEEF)
    b = st.pack()
    assert b[:10] == bytes([1, 2, 3, 4, 5, 6, 7, 0xFF, 0xFE, 0xF8]) and b[34:36] == bytes([0xEF, 0xBE]) and len(b) == 36


def test_encode_layout_and_refusals():
    pl = il.IlCmd(9, lp.CMD_LAUNCH, 0, 0xACE1).pack()
    fr = il.encode(il.TYPE_CMD, pl)
    crc = lp.crc16_ccitt(bytes([8, 1]) + pl)
    assert fr == bytes([0xC5, 0x5C, 8, 1]) + pl + bytes([crc & 0xFF, crc >> 8])
    for typ, n in ((0, 8), (4, 8), (0xFF, 49), (il.TYPE_CMD, 7), (il.TYPE_SNAP, 36), (il.TYPE_STAT, 37)):
        with pytest.raises(ValueError):
            il.encode(typ, bytes(n))
    with pytest.raises(ValueError):
        il.IlCmd(256, 0)
    with pytest.raises(ValueError):
        il.IlCmd(0, 0, rssi_dbm=128)
    with pytest.raises(ValueError):
        il.Stat(rssi_dbm=-129)


def test_pack_unpack_roundtrips_bit_exact():
    rng = random.Random(11)
    for _ in range(300):
        c = il.IlCmd(rng.randrange(256), rng.randrange(256), rng.randrange(65536), rng.randrange(65536),
                     rng.randrange(-128, 128), rng.randrange(-128, 128))
        assert il.IlCmd.unpack(c.pack()) == c
        st = il.Stat(rng.randrange(1 << 32), rng.randrange(256), rng.randrange(256), rng.randrange(256),
                     rng.randrange(-128, 128), rng.randrange(-128, 128), rng.randrange(256),
                     *[rng.randrange(65536) for _ in range(13)])
        assert il.Stat.unpack(st.pack()) == st
    # every float class except signalling NaN (Python's float conversion quietens it; the C port copies bytes)
    patterns = [0x7FC00000, 0xFFC00000, 0x7FC0BEEF, 0xFFFFFFFF, 0x7F800000, 0xFF800000, 0x80000000, 0x00000000,
                0x00000001, 0x007FFFFF, 0x80000001, 0x7F7FFFFF, 0x00800000, 0x3F800000]
    for _ in range(300):
        ints = (rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(256), rng.randrange(256),
                rng.randrange(65536), rng.randrange(65536), rng.randrange(1 << 32))
        bits = [rng.choice(patterns) if rng.random() < 0.5 else il.f32_bits(rng.uniform(-1e6, 1e6)) for _ in range(9)]
        raw = struct.pack("<BBBBBHHI9I", *ints, *bits)
        assert il.Snap.unpack(raw).pack() == raw
        assert il.encode(il.TYPE_SNAP, raw) == il.encode_snap(il.Snap.unpack(raw))


def test_f32_rounding_and_overflow_like_c_cast():
    assert il.f32(0.1) == float(np.float32(0.1))
    assert il.f32(1e39) == math.inf and il.f32(-1e39) == -math.inf
    assert il.f32(3.4028235e38) == float(np.finfo(np.float32).max)
    assert math.isnan(il.f32(math.nan)) and il.f32_bits(math.nan) == 0x7FC00000
    assert il.f32_bits(-0.0) == 0x80000000 and il.f32_from_bits(0x00000001) == 2.0 ** -149


# --------------------------------------------------------------------------- #
# decoder rules (module docstring) - counts worked out by hand
# --------------------------------------------------------------------------- #
CMD_A = il.encode_cmd(il.IlCmd(1, lp.CMD_ARM, 0, 0, -90, 20))            # no 0xC5 inside
CMD_B = il.encode_cmd(il.IlCmd(2, lp.CMD_THROTTLE, 500, 0x1234, -91, 21))
STAT_A = il.encode_stat(il.Stat(com_ticks=250, flags=il.STAT_RADIO_OK | il.STAT_BOOT, params_tag=0x11))
SNAP_A = il.encode_snap(il.Snap(3, lp.STATE_POWERED, 0x2F, 0, 3, 0x5A5A, 1000, 12136, 119.25, 22.75, 1.5, 3.875, 2875.0,
                                (1.0, 0.0, 0.0, 0.0)))
CMD_B_FRAME = il.Frame(il.TYPE_CMD, CMD_B[4:-2])


def _dec(data: bytes, max_out: int = il.DEC_MAX_OUT, accept: int = il.ACCEPT_ALL) -> tuple[list[il.Frame], dict, int]:
    d = il.IlinkDecoder(accept)
    fr = d.feed(data, max_out)
    return fr, dict(d.stats), d.pending


def _stats(**kw) -> dict:
    s = {k: 0 for k in il.DEC_STAT_KEYS}
    s.update(kw)
    return s


def _inner_header(fr: bytes) -> bool:
    """A complete valid header (C5 5C LEN TYPE with a matching pair) anywhere after the frame's first byte."""
    return any(fr[i] == il.SYNC0 and fr[i + 1] == il.SYNC1 and fr[i + 2] in il.VALID_LENS and
               il.payload_len(fr[i + 3]) == fr[i + 2] for i in range(1, len(fr) - 3))


def test_no_sync_byte_or_header_inside_the_hand_frames():
    for fr in (CMD_A, CMD_B, STAT_A):
        assert il.SYNC0 not in fr[1:], fr.hex()
    for fr in (CMD_A, CMD_B, STAT_A, SNAP_A):
        assert not _inner_header(fr)


def test_decoder_valid_frames_of_each_type():
    for fr, typ in ((CMD_A, il.TYPE_CMD), (SNAP_A, il.TYPE_SNAP), (STAT_A, il.TYPE_STAT)):
        frames, st, pend = _dec(fr)
        assert frames == [il.Frame(typ, fr[4:-2])] and st == _stats(frames=1) and pend == 0
    frames, st, pend = _dec(CMD_A + SNAP_A + STAT_A + CMD_B)
    assert [f.type for f in frames] == [1, 2, 3, 1] and st == _stats(frames=4) and pend == 0


def test_decoder_rejection_counts_by_hand():
    # C5 00: SYNC1 missing -> reject (no reason counter, C5 skipped), 00 re-examined and skipped
    assert _dec(bytes([0xC5, 0x00]) + CMD_A)[1] == _stats(frames=1, skipped_bytes=2)
    # C5 C5 5C ...: the second C5 rejects the first candidate and, re-examined, starts the frame
    assert _dec(bytes([0xC5]) + CMD_A)[1] == _stats(frames=1, skipped_bytes=1)
    # LEN 7: bad_len, C5 5C skipped, 07 re-examined and skipped
    assert _dec(bytes([0xC5, 0x5C, 0x07]) + CMD_A)[1] == _stats(frames=1, bad_len=1, skipped_bytes=3)
    # unknown TYPE 5 with LEN 8: bad_type, 4 bytes skipped in all
    assert _dec(bytes([0xC5, 0x5C, 0x08, 0x05]) + CMD_A)[1] == _stats(frames=1, bad_type=1, skipped_bytes=4)
    # v0.7: TYPE 4 is SNAP2 (LEN 55): with LEN 8 it is bad_len for the ALL decoder, bad_type for the FCU's
    assert _dec(bytes([0xC5, 0x5C, 0x08, 0x04]) + CMD_A)[1] == _stats(frames=1, bad_len=1, skipped_bytes=4)
    assert _dec(bytes([0xC5, 0x5C, 0x08, 0x04]) + CMD_A, accept=il.ACCEPT_FCU)[1] == _stats(frames=1, bad_type=1,
                                                                                            skipped_bytes=4)
    # LEN 49 with TYPE CMD: bad_len
    assert _dec(bytes([0xC5, 0x5C, 49, 0x01]) + CMD_A)[1] == _stats(frames=1, bad_len=1, skipped_bytes=4)
    # CRC hi byte flipped: bad_crc at the hi byte; 13 candidate bytes + the hi byte (re-examined, no C5) skipped
    bad = bytearray(CMD_A)
    bad[-1] ^= 0x01
    assert _dec(bytes(bad) + CMD_B)[1] == _stats(frames=1, bad_crc=1, skipped_bytes=14)
    # CRC LOW byte flipped: rejected right there (12 candidate bytes + the lo byte), then the hi byte hunted: 14
    bad = bytearray(CMD_A)
    bad[-2] ^= 0x01
    assert _dec(bytes(bad) + CMD_B)[1] == _stats(frames=1, bad_crc=1, skipped_bytes=14)
    # the rejecting byte is a C5: C5 5C 08 C5 5C 08 01 ...: bad_type, 3 skipped, that C5 opens the frame
    assert _dec(bytes([0xC5, 0x5C, 0x08]) + CMD_A)[1] == _stats(frames=1, bad_type=1, skipped_bytes=3)
    # CMD_A cut before its CRC: CMD_B's C5 is the CRC low byte -> rejected, re-examined, CMD_B found
    assert _dec(CMD_A[:12] + CMD_B)[:2] == ([CMD_B_FRAME], _stats(frames=1, bad_crc=1, skipped_bytes=12))
    # a lone C5 at the end stays pending
    assert _dec(CMD_A + bytes([0xC5]))[1:] == (_stats(frames=1), 1)


def test_decoder_sync_pair_inside_payload_is_consumed_whole():
    c = il.encode_cmd(il.IlCmd(0x5C, 0xC5, 0x5CC5, 0xC55C, 0x5C, -0x3B))
    assert c[4:-2].count(bytes([0xC5, 0x5C])) >= 2
    frames, st, pend = _dec(c + CMD_A)
    assert len(frames) == 2 and st == _stats(frames=2) and pend == 0


def test_decoder_loses_frames_hidden_in_a_rejected_candidate():
    """The no-replay trade-off (rule 4): a frame that starts inside a rejected candidate is lost - the phase-2 decoder
    found these by replaying the candidate."""
    fake = bytes([0xC5, 0x5C, il.STAT_LEN, il.TYPE_STAT]) + bytes(10) + CMD_B + bytes(20)
    frames, st, pend = _dec(fake)
    assert frames == [] and st == _stats(bad_crc=1, skipped_bytes=len(fake)) and pend == 0
    frames, st, _ = _dec(bytes([0xC5, 0x5C, il.SNAP_LEN, il.TYPE_SNAP]) + SNAP_A + bytes(8))
    assert frames == [] and st["bad_crc"] == 1
    inner = CMD_A + CMD_B + CMD_A                                    # three CMDs inside a fake SNAP
    fake = bytes([0xC5, 0x5C, il.SNAP_LEN, il.TYPE_SNAP]) + inner + bytes(il.SNAP_LEN - len(inner)) + b"\x00\x00"
    for mo in (0, 1, 2, 3, 8):
        d = il.IlinkDecoder()
        assert sum(len(d.feed_byte(b, mo)) for b in fake) == 0
        assert d.stats == _stats(bad_crc=1, skipped_bytes=len(fake))
    # ... and what follows the swallowed region is found again
    frames, _, _ = _dec(fake + CMD_B)
    assert frames == [CMD_B_FRAME]


def test_decoder_at_most_one_frame_per_byte_and_max_out_0():
    rng = random.Random(9)
    pool = [CMD_A, CMD_B, STAT_A, SNAP_A]
    for _ in range(200):
        data = b"".join(rng.choice(pool + [bytes([0xC5, 0x5C, 0x08, 0x01]), bytes([0xC5])]) for _ in range(rng.randrange(1, 12)))
        d = il.IlinkDecoder()
        assert all(len(d.feed_byte(b, 8)) <= 1 for b in data)
    d = il.IlinkDecoder()
    assert d.feed(CMD_A + STAT_A + SNAP_A + CMD_B, 0) == [] and d.stats == _stats(frames=4, overflow=4)


def test_decoder_truncated_frame_then_valid():
    for cut in range(1, len(CMD_A)):
        frames, st, pend = _dec(CMD_A[:cut] + CMD_B)
        # CMD_B's C5 at a header or CRC position of the cut candidate is re-examined; in its payload it is swallowed
        assert (frames == [CMD_B_FRAME]) == (cut <= 3 or cut >= 12) and pend == 0, cut
        if cut > 3 and cut < 12:
            assert frames == [] and st["bad_crc"] == 1
    frames, st, pend = _dec(SNAP_A[:-1])
    assert frames == [] and pend == len(SNAP_A) - 1 and st == _stats()


def _variants(a: bytes, values: list[int]):
    """Every single-byte error of frame a: (kind, position, value, bytes) for substitutions and inserts of each value
    and every drop."""
    for j in range(len(a)):
        for v in values:
            if v != a[j]:
                yield "sub", j, v, a[:j] + bytes([v]) + a[j + 1:]
        yield "drop", j, None, a[:j] + a[j + 1:]
    for j in range(len(a) + 1):
        for v in values:
            yield "ins", j, v, a[:j] + bytes([v]) + a[j:]


def test_single_byte_error_never_loses_the_next_frame():
    """Rule 4 + the CRC low byte checked on arrival: a frame directly behind a corrupted one survives any single
    substituted, dropped or inserted byte in it (values here: the protocol bytes and a few others; the host test sweeps
    all 256).  The corrupted frame itself never comes out wrong, except as a CRC-16 false accept."""
    n = 0
    for a in (CMD_A, STAT_A, SNAP_A):
        vals = sorted({0xC5, 0x5C, 0x08, 0x24, 0x31, 0x01, 0x02, 0x03, 0x00, 0xFF})
        for kind, j, v, s in _variants(a, vals):
            frames, _, pend = _dec(s + CMD_B)
            assert frames and frames[-1] == CMD_B_FRAME and pend == 0, (a.hex(), kind, j, v)
            assert all(f == il.Frame(a[3], a[4:-2]) for f in frames[:-1]), (kind, j, v)   # only A itself (an insert before it)
            n += 1
    assert n > 2000


def test_two_dropped_payload_bytes():
    """Two bytes dropped from a frame's payload / CRC: the shortened candidate reaches into the next frame by two
    bytes, so the next frame's 0xC5 lands on the candidate's CRC low byte - re-examined (found) unless the shortened
    candidate's CRC low byte happens to be 0xC5 (accepted there: lost).  Exactly that, for every pair."""
    lost = cases = 0
    for a in (STAT_A, SNAP_A):
        for i in range(4, len(a)):
            for j in range(i + 1, len(a)):
                s = a[:i] + a[i + 1:j] + a[j + 1:]
                frames, _, _ = _dec(s + CMD_B)
                predicted_lost = (il.crc16(s[2:]) & 0xFF) == il.SYNC0
                assert (CMD_B_FRAME not in frames) == predicted_lost, (a.hex(), i, j)
                lost += predicted_lost
                cases += 1
    assert cases == 703 + 1275 and lost < cases * 3 / 256


def test_decoder_bytewise_equals_block_and_byte_accounting():
    rng = random.Random(7)
    pool = [CMD_A, CMD_B, STAT_A, SNAP_A]
    for _ in range(300):
        parts = []
        for _ in range(rng.randrange(1, 10)):
            fr = bytearray(rng.choice(pool))
            k = rng.random()
            if k < 0.3:
                fr[rng.randrange(len(fr))] ^= 1 << rng.randrange(8)
            elif k < 0.45:
                fr = fr[:rng.randrange(1, len(fr))]
            elif k < 0.6:
                fr = bytearray(rng.choice([0xC5, 0x5C, 8, 36, 49, 1, 2, 3, rng.randrange(256)]) for _ in range(rng.randrange(1, 40)))
            parts.append(bytes(fr))
        data = b"".join(parts)
        mo = rng.choice([0, 1, 2, 3])
        a, b = il.IlinkDecoder(), il.IlinkDecoder()
        fa = a.feed(data, mo)
        fb = [f for x in data for f in b.feed_byte(x, mo)]
        assert fa == fb and a.stats == b.stats and a.pending == b.pending
        assert len(data) == a.stats["skipped_bytes"] + a.frame_bytes + a.pending          # rule 7 (no wrap here)
        assert a.stats["frames"] - a.stats["overflow"] == len(fa)


# --------------------------------------------------------------------------- #
# direction filter (v0.6.3, R4)
# --------------------------------------------------------------------------- #
def test_direction_filter_by_hand():
    F, C = il.ACCEPT_FCU, il.ACCEPT_COM
    # the FCU never takes a SNAP: rejected at LEN 49, its 55 bytes hunted (no 0xC5 inside SNAP_A), the CMD behind found
    assert il.SYNC0 not in SNAP_A[1:]
    assert _dec(SNAP_A + CMD_A, accept=F) == ([il.Frame(il.TYPE_CMD, CMD_A[4:-2])], _stats(frames=1, bad_len=1,
                                                                                            skipped_bytes=55), 0)
    # the COM takes only SNAPs: CMD and STAT rejected at LEN
    fr, st, _ = _dec(CMD_A + STAT_A + SNAP_A + CMD_B, accept=C)
    assert fr == [il.Frame(il.TYPE_SNAP, SNAP_A[4:-2])] and st == _stats(frames=1, bad_len=3, skipped_bytes=14 + 42 + 14)
    # an accepted LEN with a TYPE this side does not receive: bad_type (FCU: LEN 8 + TYPE SNAP; COM: LEN 49 + TYPE CMD)
    assert _dec(bytes([0xC5, 0x5C, 8, il.TYPE_SNAP]) + CMD_A, accept=F)[1] == _stats(frames=1, bad_type=1, skipped_bytes=4)
    assert _dec(bytes([0xC5, 0x5C, 49, il.TYPE_CMD]) + SNAP_A, accept=C)[1] == _stats(frames=1, bad_type=1, skipped_bytes=4)
    # ... an accepted TYPE with another accepted TYPE's LEN: bad_len (FCU: LEN 36 + TYPE CMD)
    assert _dec(bytes([0xC5, 0x5C, 36, il.TYPE_CMD]) + CMD_A, accept=F)[1] == _stats(frames=1, bad_len=1, skipped_bytes=4)
    # the reason the filter exists: a SNAP header in garbage swallows the CMD behind it (ALL); the FCU decoder finds it
    junk = bytes([0xC5, 0x5C, il.SNAP_LEN, il.TYPE_SNAP]) + CMD_A + bytes(40) + CMD_B
    assert _dec(junk, accept=il.ACCEPT_ALL)[0] == [CMD_B_FRAME]                    # CMD_A swallowed and lost
    assert _dec(junk, accept=F)[0] == [il.Frame(il.TYPE_CMD, CMD_A[4:-2]), CMD_B_FRAME]
    # need: the shortest ACCEPTED frame before LEN is known; cmd_need: NEED_NEVER without CMD
    d = il.IlinkDecoder(C)
    assert (d.need, d.cmd_need) == (0, il.NEED_NEVER)
    d.feed(bytes([0xC5]))
    assert (d.need, d.cmd_need) == (54, il.NEED_NEVER)
    d.feed(bytes([0x5C]))
    assert d.need == 53
    d = il.IlinkDecoder(il.ACCEPT_STAT)
    d.feed(bytes([0xC5]))
    assert d.need == 41
    d = il.IlinkDecoder(0)                                            # accepts nothing: every LEN is bad_len
    d.feed(bytes([0xC5]))
    assert (d.need, d.cmd_need) == (il.NEED_NEVER, il.NEED_NEVER)
    assert _dec(CMD_A + STAT_A + SNAP_A, accept=0)[0] == []
    for bad in (32, 0xFF, -1, True, 1.0):
        with pytest.raises(ValueError):
            il.IlinkDecoder(bad)


def test_direction_filter_only_removes_frames_of_the_other_direction():
    """On streams without hidden frames (every frame starts at a hunting position for both decoders) the FCU / COM
    decoder returns exactly the ALL decoder's frames of its own types; with garbage in between it can only find MORE of
    its own frames (a refused header no longer swallows them), never one the ALL decoder rejects as corrupted."""
    rng = random.Random(31)
    pool = [CMD_A, CMD_B, STAT_A, SNAP_A]
    for _ in range(300):
        data = b"".join(rng.choice(pool) for _ in range(rng.randrange(1, 12)))
        fa = _dec(data)[0]
        for acc in (il.ACCEPT_FCU, il.ACCEPT_COM):
            want = [f for f in fa if acc & (1 << (f.type - 1))]
            assert _dec(data, accept=acc)[0] == want
    for _ in range(300):
        parts = []
        for _ in range(rng.randrange(1, 10)):
            if rng.random() < 0.6:
                parts.append(rng.choice(pool))
            else:
                parts.append(bytes(rng.choice([0xC5, 0x5C, 8, 36, 49, 1, 2, 3, rng.randrange(256)])
                                   for _ in range(rng.randrange(1, 30))))
        data = b"".join(parts)
        good = {bytes(f) for f in pool}
        for acc in (il.ACCEPT_FCU, il.ACCEPT_COM):
            got = _dec(data, accept=acc)[0]
            assert all(acc & (1 << (f.type - 1)) for f in got)
            assert all(il.encode(f.type, f.payload) in good or il.encode(f.type, f.payload) in data for f in got)


def test_decoder_counters_wrap_at_16_bits():
    d = il.IlinkDecoder()
    d.feed(bytes([0x00, 0x5C]) * 35000)
    assert d.stats["skipped_bytes"] == 70000 & 0xFFFF and d.pending == 0


# --------------------------------------------------------------------------- #
# il_decoder_need / il_decoder_cmd_need (rule 8)
# --------------------------------------------------------------------------- #
def _trace(data: bytes) -> list[tuple[int, int]]:
    d = il.IlinkDecoder()
    out = []
    for b in data:
        d.feed_byte(b)
        out.append((d.need, d.cmd_need))
    return out


def test_need_and_cmd_need_by_hand():
    d = il.IlinkDecoder()
    assert (d.need, d.cmd_need) == (0, 14)
    t = _trace(CMD_A)
    assert t[:4] == [(13, 13), (12, 12), (11, 11), (10, 10)] and t[11:] == [(2, 2), (1, 1), (0, 14)]
    t = _trace(STAT_A)
    assert t[:4] == [(13, 13), (12, 12), (39, 14), (38, 50)]       # LEN 36 known: the TYPE byte can reject (C5)
    assert t[38] == (3, 15) and t[39][0] == 2 and t[40][0] == 1 and t[41] == (0, 14) and len(t) == 42
    assert t[39][1] == (15 if STAT_A[-2] == 0xC5 else 14) and t[40][1] == (15 if STAT_A[-1] == 0xC5 else 14)
    assert _trace(bytes([0x00, 0x5C]))[-1] == (0, 14)


def _cmd_frame_from(cand: bytes) -> bytes:
    """The IL_CMD frame that completes a candidate compatible with one (C5 [5C [08 [01 [payload ...]]]])."""
    pl = bytes(cand[4:4 + il.CMD_LEN])
    pl += bytes(il.CMD_LEN - len(pl))
    fr = il.encode(il.TYPE_CMD, pl)
    assert fr[:len(cand)] == bytes(cand)
    return fr


def _first_cmd_at(dec: il.IlinkDecoder, cont: bytes) -> int | None:
    """1-based index of the first continuation byte that returns an IL_CMD from a copy of dec."""
    d = copy.deepcopy(dec)
    for k, b in enumerate(cont, 1):
        if any(f.type == il.TYPE_CMD for f in d.feed_byte(b)):
            return k
    return None


def _fast_continuations(dec: il.IlinkDecoder) -> list[bytes]:
    """Explicit continuations aiming at the earliest IL_CMD: complete the candidate as an IL_CMD, reject it as early as
    possible with a 0xC5 that opens a fresh IL_CMD (payload fillers chosen so the CRC low byte is not 0xC5), or close it
    with one non-0xC5 byte, or complete it correctly - then an IL_CMD."""
    c = bytes(dec._cand)
    n = len(c)
    fresh = CMD_B
    out = [fresh, bytes([0x00]) + fresh]
    if n == 0:
        return out
    if n < 3 or c[2] == il.CMD_LEN:
        out.append(_cmd_frame_from(c)[n:])
    if n <= 3:
        out.append(bytes([il.SYNC0]) + fresh[1:])
    else:
        ln = c[2]
        free = max(ln + 4 - n, 0)
        fill = bytearray(free)
        if free and il.crc16(c[2:] + bytes(fill)) & 0xFF == il.SYNC0:
            fill[-1] = 1
        out.append(bytes(fill) + bytes([il.SYNC0]) + fresh[1:])
        crc = il.crc16((c + bytes(fill))[2:4 + ln])
        tail = bytes([crc & 0xFF, crc >> 8])[max(n - (ln + 4), 0):]
        out.append(bytes(fill) + tail + fresh)                       # complete it, then a CMD
        out.append(bytes(fill) + tail[:1] + bytes([il.SYNC0]) + fresh[1:])   # CRC lo right, a C5 as CRC hi
    return out


def _states(rng: random.Random, n: int, accept: int = il.ACCEPT_ALL) -> list[il.IlinkDecoder]:
    """Decoders in many states: cut frames of every type (every cut length), corrupted, noise, CRC-0xC5 corners."""
    out = []
    corner = [il.encode_stat(il.Stat(com_ticks=k, params_tag=0x5C)) for k in range(800)]
    corner = [f for f in corner if 0xC5 in (f[-2], f[-1])][:6]
    frames = [CMD_A, CMD_B, STAT_A, SNAP_A] + corner
    for fr in frames:
        for cut in range(0, len(fr)):
            d = il.IlinkDecoder(accept)
            d.feed(fr[:cut])
            out.append(d)
    while len(out) < n:
        d = il.IlinkDecoder(accept)
        d.feed(bytes(rng.choice([0xC5, 0x5C, 0x08, 0x24, 0x31, 0x01, 0x02, 0x03, rng.randrange(256)])
                     for _ in range(rng.randrange(1, 70))))
        out.append(d)
    return out


@pytest.mark.parametrize("accept", [il.ACCEPT_ALL, il.ACCEPT_FCU])
def test_cmd_need_is_exact(accept):
    """cmd_need is attained (an explicit continuation of exactly that length returns an IL_CMD on its last byte) and is
    a lower bound (adversarial random continuations - rich in 0xC5 and in the CRC bytes the candidate expects - never
    return an IL_CMD earlier); for the ALL decoder and the FCU's (CMD | STAT)."""
    rng = random.Random(23)
    states = _states(rng, 700, accept)
    for d in states:
        cn = d.cmd_need
        hits = [k for k in (_first_cmd_at(d, c) for c in _fast_continuations(d)) if k is not None]
        assert min(hits) == cn, (bytes(d._cand).hex(), cn, hits)
        for _ in range(12):                                          # lower bound
            e = copy.deepcopy(d)
            for k in range(1, cn):
                c = bytes(e._cand)
                if len(c) >= 4 and len(c) >= c[2] + 4 and rng.random() < 0.6:
                    crc = il.crc16(c[2:4 + c[2]])
                    b = crc & 0xFF if len(c) == c[2] + 4 else crc >> 8
                else:
                    b = rng.choice([0xC5, 0xC5, 0xC5, 0x5C, 0x08, 0x01, 0x24, 0x31, 0x02, 0x03, rng.randrange(256)])
                assert not any(f.type == il.TYPE_CMD for f in e.feed_byte(b)), (bytes(d._cand).hex(), cn, k)


# --------------------------------------------------------------------------- #
# FCU snapshot <-> COM telemetry
# --------------------------------------------------------------------------- #
def test_euler_equals_sim_quat_to_euler():
    rng = random.Random(3)
    for _ in range(2000):
        q = tuple(_f32(rng.uniform(-1, 1)) for _ in range(4))
        r, p, y = il.euler_deg(q)
        e = quat.to_euler(np.array(q))
        assert (r, p, y) == (math.degrees(e[0]), math.degrees(e[1]), math.degrees(e[2]))
    r, p, y = il.euler_deg((math.nan, 0.0, 0.0, 0.0))             # NaN passes fc_clampf (min/max would give 90 deg)
    assert math.isnan(r) and math.isnan(p) and math.isnan(y)
    assert il.euler_deg((0.92387953, 0.0, 0.38268343, 0.0))[1] == pytest.approx(45.0, abs=1e-5)


def test_snap_from_and_to_tlm_src():
    src = il.TlmSource(state=2, flags=0x2F, reason=5, ack_seq=7, nonce=0xBEEF, throttle_milli=812, tick=123456, h_m=87.1,
                       vz_mps=21.3, roll_deg=99.0, pitch_deg=-99.0, yaw_deg=33.0, tilt_deg=45.0, v_cell=3.95,
                       loop_max_us=2900.0)
    q = (0.92387953, 0.0, 0.38268343, 0.0)
    s = il.snap_from_tlm_src(42, src, q)
    assert s.reply_seq == 42 and s.h_m == _f32(87.1) and s.v_cell == _f32(3.95) and s.q == tuple(_f32(c) for c in q)
    back = il.snap_to_tlm_src(il.Snap.unpack(s.pack()))
    assert (back.state, back.flags, back.reason, back.ack_seq, back.nonce, back.throttle_milli, back.tick) == (
        2, 0x2F, 5, 7, 0xBEEF, 812, 123456)
    assert back.h_m == _f32(87.1) and back.vz_mps == _f32(21.3) and back.tilt_deg == 45.0
    assert back.roll_deg == pytest.approx(0.0, abs=1e-5) and back.pitch_deg == pytest.approx(45.0, abs=1e-5)


def test_telem_frame_from_snap_equals_onboard_supervisor_telemetry():
    """The COM pipeline reproduces, byte for byte, the TELEM frame sim.onboard builds itself for the same tick when its
    inputs are the float32 values the FCU snapshot carries and its roll / pitch / yaw come from the same q."""
    p = json.load(open(il.PARAMS_PATH, encoding="utf-8"))
    sp = SupervisorParams.from_params(p, mode=lp.MODE_BALLISTIC)          # the ballistic build (SNAP -> TELEM v1)
    sup = OnboardSupervisor(sp, seed=0x1D)
    rng = random.Random(17)
    seq, n_replies, states = 0, 0, set()
    script = {2: lp.CMD_NONE, 10: lp.CMD_ARM, 30: lp.CMD_LAUNCH, 200: lp.CMD_THROTTLE, 400: lp.CMD_CUT, 450: lp.CMD_THROTTLE,
              600: lp.CMD_RELEASE, 610: lp.CMD_KILL}
    for t in range(800):
        frames = []
        if t in script or t % 25 == 0:
            cmd = script.get(t, lp.CMD_NONE)
            seq = (seq + 1) & 0xFF
            frames = [UplinkFrame(seq, lp.Command(cmd, 700 if cmd == lp.CMD_THROTTLE else 0, sup.nonce))]
        q = quat.from_euler(math.radians(rng.uniform(-20, 20)), math.radians(rng.uniform(-20, 20)),
                            math.radians(rng.uniform(-180, 180)))
        q32 = tuple(_f32(c) for c in q)
        e = quat.to_euler(np.array(q32))
        inp = OnboardInputs(tick=t, frames=frames, h_m=_f32(t * 0.37), vz_mps=_f32(rng.uniform(-5, 30)),
                            tilt_deg=_f32(rng.uniform(0, 3)), v_cell=_f32(rng.uniform(3.8, 4.1)),
                            roll_deg=math.degrees(e[0]), pitch_deg=math.degrees(e[1]), yaw_deg=math.degrees(e[2]),
                            loop_max_us=_f32(rng.uniform(2000, 3500)))
        out = sup.update(inp)
        states.add(out.state)
        if out.reply_seq is None:
            continue
        src = il.TlmSource(state=int(out.state), flags=out.flags, reason=out.reason, ack_seq=out.ack_seq, nonce=out.nonce,
                           throttle_milli=out.throttle_milli, tick=t, h_m=inp.h_m, vz_mps=inp.vz_mps, roll_deg=0.0,
                           pitch_deg=0.0, yaw_deg=0.0, tilt_deg=inp.tilt_deg, v_cell=inp.v_cell, loop_max_us=inp.loop_max_us)
        snap = il.Snap.unpack(il.encode_snap(il.snap_from_tlm_src(out.reply_seq, src, q32))[4:-2])   # over the wire
        assert il.telem_frame_from_snap(snap, VID, HZ) == lp.encode_telemetry(VID, out.reply_seq, out.telemetry), t
        n_replies += 1
    assert n_replies > 30 and {1, 2, 3} <= {int(s) for s in states}
    assert il.telem_frame_from_snap(snap) == il.telem_frame_from_snap(snap, VID, HZ)          # params defaults


# --------------------------------------------------------------------------- #
# firmware vectors
# --------------------------------------------------------------------------- #
def test_snap3_layout_decoder_masks_and_the_v08_vectors():
    """v0.8 SNAP3 (TYPE 5, 62 B): the SNAP2 layout with the spare byte = gps_flags, then num_sv, hacc_dm, pos_mode, dist_m.
    Only the VTOL + GPS COM (ACCEPT_COM_GPS) and ACCEPT_ALL8 take it; the v0.7 masks keep their behaviour."""
    s = il.Snap(21, lp.STATE_POWERED, 0x2F, 0, 21, 0x5A5A, 918, 5346, 1.5, -0.04, 3.68, 3.86, 2875.0,
                (0.9995, 0.012, -0.028, 0.0087), phase=lp.PHASE_HOLD, h_tgt_m=1.5, gps_flags=0x7F, num_sv=9, hacc_dm=25,
                pos_mode=2, dist_m=0.734)
    pl = s.pack()
    assert len(pl) == il.SNAP3_LEN == 65 and s.wire_type == il.TYPE_SNAP3 and s.is_v3 and s.gps == (0x7F, 9, 25, 2, 0.734)
    assert pl[62:65] == bytes(3)                                 # v0.9 RP1: pw_vaux, pw_flags (battery: 0, 0), spare3
    s2 = il.Snap(*[getattr(s, n) for n in il.SNAP_INT_FIELDS + il.SNAP_FLOAT_FIELDS], q=s.q, phase=s.phase, h_tgt_m=1.5)
    assert pl[:49] == s2.pack()[:49] and pl[49] == s.phase and pl[50] == 0x7F and pl[51:55] == s2.pack()[51:55]
    assert pl[55:59] == bytes([9, 25, 2]) + pl[58:59] and struct.unpack("<f", pl[58:62])[0] == il.f32(0.734)
    assert il.Snap.unpack(pl).pack() == pl and il.Snap.unpack(pl).dist_m == il.f32(0.734)
    fr = il.encode_snap(s)
    assert len(fr) == il.SNAP3_FRAME_BYTES == 71
    assert _dec(fr, accept=il.ACCEPT_COM_GPS)[0] == [il.Frame(il.TYPE_SNAP3, pl)]
    assert _dec(fr, accept=il.ACCEPT_ALL8)[0] == [il.Frame(il.TYPE_SNAP3, pl)]
    for acc in (il.ACCEPT_ALL, il.ACCEPT_FCU, il.ACCEPT_COM, il.ACCEPT_COM_VTOL):
        assert _dec(fr, accept=acc)[0] == []
    with pytest.raises(ValueError):
        il.Snap(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, phase=1, h_tgt_m=0.0, gps_flags=1)   # partial GPS fields
    with pytest.raises(ValueError):
        il.Snap(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gps_flags=1, num_sv=1, hacc_dm=1, pos_mode=1, dist_m=0.0)
    v8 = il.make_test_vectors(VID, HZ)["v08"]
    for pv in v8["payloads"]:
        p8 = bytes.fromhex(pv["payload_hex"])
        assert il.encode(il.TYPE_SNAP3, p8).hex() == pv["frame_hex"]
    for sv in v8["streams"]:
        dec = il.IlinkDecoder(sv["accept"])
        out = dec.feed(bytes.fromhex(sv["hex"]))
        assert [f.payload.hex() for f in out] == [e["payload_hex"] for e in sv["expected_frames"]], sv["name"]
        assert dec.stats == sv["expected_stats"]
    for tv in v8["telem"]:
        snap = il.Snap.unpack(bytes.fromhex(tv["snap_hex"]))
        assert il.telem_frame_from_snap(snap, VID, HZ).hex() == tv["telem_frame_hex"]
        assert bytes.fromhex(tv["telem_frame_hex"])[4] == lp.TYPE_TELEM3
    nan = [t for t in v8["telem"] if t["name"] == "snap3_pad_no_gps"][0]
    t3 = lp.Telemetry.unpack(bytes.fromhex(nan["telem_frame_hex"])[6:-2])
    assert t3.dist_dm == lp.DIST_NONE and t3.reason == lp.REJ_GPS


def test_telem3_frame_from_snap3_equals_the_gps_supervisor_telemetry():
    """v0.8 VTOL + GPS build: SNAP3 -> TELEM3 reproduces the supervisor's own TELEM3 byte for byte (the GPS fields
    as the FCU stack hands them to the supervisor)."""
    p = json.load(open(il.PARAMS_PATH, encoding="utf-8"))
    sp = SupervisorParams.from_params(p, mode=lp.MODE_VTOL)
    assert sp.gps_hold
    sup = OnboardSupervisor(sp, seed=0x2F)
    rng = random.Random(23)
    seq, n_replies = 0, 0
    script = {10: lp.CMD_ON, 300: lp.CMD_ON, 900: lp.CMD_UP, 1300: lp.CMD_OFF}
    for t in range(1800):
        frames = []
        if t in script or t % 25 == 0:
            seq = (seq + 1) & 0xFF
            frames = [UplinkFrame(seq, lp.Command(script.get(t, lp.CMD_NONE), 0, sup.nonce))]
        q32 = tuple(_f32(c) for c in quat.from_euler(math.radians(rng.uniform(-5, 5)), math.radians(rng.uniform(-5, 5)),
                                                    math.radians(rng.uniform(-180, 180))))
        dist = math.nan if t < 700 or rng.random() < 0.1 else _f32(rng.uniform(0, 30))
        g = (rng.randrange(256), rng.randrange(20), rng.randrange(256), rng.randrange(5), dist)
        inp = OnboardInputs(tick=t, frames=frames, h_m=_f32(rng.uniform(-1, 3)), vz_mps=_f32(rng.uniform(-1, 1)),
                            tilt_deg=_f32(rng.uniform(0, 3)), v_cell=_f32(rng.uniform(3.8, 4.1)),
                            loop_max_us=_f32(rng.uniform(2000, 3500)), vc_at_target=t == 800, gps_gate_ok=t >= 250,
                            tlm_gps=g)
        out = sup.update(inp)
        if out.reply_seq is None:
            continue
        src = il.TlmSource(state=int(out.state), flags=out.flags, reason=out.reason, ack_seq=out.ack_seq, nonce=out.nonce,
                           throttle_milli=0, tick=t, h_m=inp.h_m, vz_mps=inp.vz_mps, roll_deg=0.0, pitch_deg=0.0,
                           yaw_deg=0.0, tilt_deg=inp.tilt_deg, v_cell=inp.v_cell, loop_max_us=inp.loop_max_us,
                           phase=out.phase, h_tgt_m=out.h_tgt_m, gps=g)
        snap = il.Snap.unpack(il.encode_snap(il.snap_from_tlm_src(out.reply_seq, src, q32))[4:-2])
        assert snap.is_v3
        mine = lp.encode_telemetry(VID, out.reply_seq, out.telemetry)
        theirs = il.telem_frame_from_snap(snap, VID, HZ)
        # every byte equal except the roll / pitch / yaw quanta (the supervisor got 0 here, the COM computes them)
        assert theirs[:20] == mine[:20] and theirs[26:-2] == mine[26:-2], t
        n_replies += 1
    assert n_replies > 60


def test_vectors_deterministic_consistent_and_write():
    v = il.make_test_vectors(VID, HZ)
    assert il.make_test_vectors(VID, HZ) == v
    assert v["crc16_check"]["crc"] == 0x29B1 and v["vehicle_id"] == VID and v["loop_hz"] == HZ
    for pv in v["payloads"]:
        pl = bytes.fromhex(pv["payload_hex"])
        assert len(pl) == il.PAYLOAD_LEN[pv["type"]] and bytes.fromhex(pv["frame_hex"]) == il.encode(pv["type"], pl)
        if pv["py_roundtrip"]:
            cls = {il.TYPE_CMD: il.IlCmd, il.TYPE_SNAP: il.Snap, il.TYPE_STAT: il.Stat, il.TYPE_SNAP2: il.Snap}[pv["type"]]
            assert cls.unpack(pl).pack() == pl, pv["name"]
    names = {pv["name"] for pv in v["payloads"]}
    assert {"snap_nan_inf", "snap_denormals", "snap_signalling_nan", "snap_nan_payload"} <= names
    tot = {k: 0 for k in il.DEC_STAT_KEYS}
    by_name = {}
    for sv in v["streams"]:
        data = bytes.fromhex(sv["hex"])
        for ef in sv["expected_frames"]:                         # every returned frame is a correct frame in the data
            assert il.encode(ef["type"], bytes.fromhex(ef["payload_hex"])).hex() in data.hex()
        for k in tot:
            tot[k] += sv["expected_stats"][k]
        need, cneed = bytes.fromhex(sv["need_hex"]), bytes.fromhex(sv["cmd_need_hex"])
        assert len(need) == len(cneed) == len(data) and max(need) <= il.FRAME_MAX_BYTES - 1
        if sv["accept"] & il.ACCEPT_CMD:
            assert 1 <= min(cneed) and max(cneed) <= il.SNAP2_LEN + il.CMD_FRAME_BYTES     # a SNAP2 candidate: 69
        else:
            assert set(cneed) == {il.NEED_NEVER}
        for ef in sv["expected_frames"]:                         # only frames of the decoder's own direction
            assert sv["accept"] & (1 << (ef["type"] - 1)), sv["name"]
        by_name[sv["name"]] = sv
    assert all(x > 0 for x in tot.values()), tot                 # every counter exercised
    assert len([s for s in v["streams"] if s["name"].startswith("bitflip_cmd") and s["accept"] == il.ACCEPT_ALL]) == 14 * 8
    # the phase-2 hidden-frame vectors, kept with their phase-3 results: the hidden frames are lost
    for nm in ("three_cmds_in_bad_snap_max1", "three_cmds_in_bad_snap_max2", "three_cmds_in_bad_snap_max3",
               "cmd_in_bad_stat", "snap_in_bad_snap", "dropped_3_bytes_then_cmd"):
        assert by_name[nm]["expected_frames"] == [] and by_name[nm]["expected_stats"]["bad_crc"] == 1, nm
    # a frame starting AT the rejecting byte is found (re-examination), as after 1 or 2 dropped bytes
    for nm in ("rejecting_byte_is_sync0_at_sync1", "rejecting_byte_is_sync0_at_len", "rejecting_byte_is_sync0_at_type",
               "rejecting_byte_is_sync0_at_crc_lo", "rejecting_byte_is_sync0_at_crc_hi", "dropped_1_bytes_then_cmd",
               "dropped_2_bytes_then_cmd", "double_sync0", "double_sync_pair"):
        assert len(by_name[nm]["expected_frames"]) == 1 and by_name[nm]["expected_frames"][0]["type"] == il.TYPE_CMD, nm
    assert by_name["max_out_0"]["expected_stats"]["overflow"] == 3 and by_name["max_out_0"]["expected_frames"] == []
    # direction filter (R4): every stream also through the FCU's and the COM's decoder, plus their own cases
    assert {sv["accept"] for sv in v["streams"]} == {il.ACCEPT_ALL, il.ACCEPT_FCU, il.ACCEPT_COM, il.ACCEPT_COM_VTOL}
    assert [f["type"] for f in by_name["snap_on_fcu"]["expected_frames"]] == [il.TYPE_CMD]
    assert by_name["snap_header_then_cmd@all"]["expected_frames"] == []
    assert [f["type"] for f in by_name["snap_header_then_cmd@fcu"]["expected_frames"]] == [il.TYPE_CMD, il.TYPE_CMD]
    assert [f["type"] for f in by_name["cmd_stat_on_com"]["expected_frames"]] == [il.TYPE_SNAP]
    assert by_name["max_out_0@fcu"]["expected_stats"]["overflow"] == 2                # CMD + STAT, the SNAP refused
    assert by_name["max_out_0@com"]["expected_stats"]["overflow"] == 1
    assert 15 in bytes.fromhex(by_name["stat_crc_lo_is_sync0"]["cmd_need_hex"])      # the cmd_need corner is covered
    assert 15 in bytes.fromhex(by_name["stat_crc_hi_is_sync0"]["cmd_need_hex"])
    for tv in v["telem"]:
        fr = bytes.fromhex(tv["telem_frame_hex"])
        assert fr == il.telem_frame_from_snap(il.Snap.unpack(bytes.fromhex(tv["snap_hex"])), VID, HZ)
    # v0.7 VTOL: SNAP2 payloads, the VTOL COM's mask, SNAP2 -> TELEM2
    assert len([pv for pv in v["payloads"] if pv["type"] == il.TYPE_SNAP2]) >= 15
    assert all(len(_flat(pv["fields"])) <= 22 for pv in v["payloads"])          # v0.9: SNAP2 22 values (ILV_NF 27)
    assert [f["type"] for f in by_name["snap_v1_on_vtol_com"]["expected_frames"]] == [il.TYPE_SNAP2] * 2
    assert [f["type"] for f in by_name["snap2_on_ballistic_com"]["expected_frames"]] == [il.TYPE_SNAP] * 2
    assert [f["type"] for f in by_name["snap2_on_fcu"]["expected_frames"]] == [il.TYPE_CMD, il.TYPE_STAT]
    assert by_name["snap2_header_then_cmd@all"]["expected_frames"][-1]["type"] == il.TYPE_SNAP2
    assert [f["type"] for f in by_name["snap2_header_then_cmd@fcu"]["expected_frames"]] == [il.TYPE_CMD, il.TYPE_CMD]
    assert by_name["max_out_0_snap2@comv"]["expected_stats"]["overflow"] == 2
    t2 = [tv for tv in v["telem"] if tv["name"].startswith("snap2_")]
    assert len(t2) >= 15 and all(bytes.fromhex(tv["telem_frame_hex"])[4] == lp.TYPE_TELEM2 for tv in t2)
    assert all(bytes.fromhex(tv["telem_frame_hex"])[4] == lp.TYPE_TELEM for tv in v["telem"] if tv not in t2)
    written = il.write_test_vectors()
    assert written == v
    with open(il.VECTORS_PATH, encoding="utf-8") as f:
        assert json.load(f) == v


def _flat(fields: dict) -> list[int]:
    out: list[int] = []
    for x in fields.values():
        out.extend(x if isinstance(x, list) else [x])
    return out


# --------------------------------------------------------------------------- #
# v0.7 VTOL build: SNAP2 (TYPE 4, 55 B), the VTOL COM's mask, SNAP2 -> TELEM2
# --------------------------------------------------------------------------- #
SNAP2_A = il.encode_snap(il.Snap(21, lp.STATE_POWERED, 0x2F, 0, 21, 0x5A5A, 918, 5346, 4.97, -0.04, 1.68, 3.86, 2950.5,
                                 (1.0, 0.0, 0.0, 0.0), phase=lp.PHASE_HOLD, h_tgt_m=6.0))


def test_snap2_constants_match_header():
    """Waits for the C port (firmware/atmega128_fc/src/core/ilink_proto.h): the SNAP2 TYPE, LEN and accept bits."""
    assert (_hdr_const("IL_TYPE_SNAP2"), _hdr_const("IL_SNAP2_LEN")) == (il.TYPE_SNAP2, il.SNAP2_LEN) == (4, 57)
    assert (_hdr_const("IL_ACCEPT_SNAP2"), _hdr_const("IL_ACCEPT_COM_VTOL")) == (il.ACCEPT_SNAP2, il.ACCEPT_COM_VTOL) == (8, 8)


def test_snap2_layout_hand_placed_and_roundtrip():
    s = il.Snap(1, 2, 3, 4, 5, 0x0706, 0x0908, 0x0D0C0B0A, 1.0, -2.0, 0.5, 4.0, 2500.0, (1.0, 0.0, -0.0, 0.25),
                phase=lp.PHASE_LAND, h_tgt_m=7.5, spare=0)
    b = s.pack()
    v1 = il.Snap(*(getattr(s, n) for n in il.SNAP_INT_FIELDS + il.SNAP_FLOAT_FIELDS), q=s.q).pack()
    assert len(b) == 57 and b[:49] == v1 and b[49] == lp.PHASE_LAND and b[50] == 0 and b[51:55] == struct.pack("<f", 7.5)
    assert b[55:57] == bytes(2)                                  # v0.9 RP1: pw_vaux, pw_flags (battery: 0, 0)
    assert il.encode_snap(s)[3] == il.TYPE_SNAP2 and il.encode_snap(s)[2] == 57 and len(il.encode_snap(s)) == 63
    assert il.Snap.unpack(b) == s and il.Snap.unpack(b).is_v2 and not il.Snap.unpack(v1).is_v2
    rng = random.Random(4)
    for _ in range(300):
        raw = struct.pack("<BBBBBHHI9IBBIBB", *[rng.randrange(256) for _ in range(5)], rng.randrange(65536),
                          rng.randrange(65536), rng.randrange(1 << 32),
                          *[il.f32_bits(rng.uniform(-1e5, 1e5)) for _ in range(9)], rng.randrange(256), rng.randrange(256),
                          rng.choice([il.f32_bits(rng.uniform(-50, 50)), 0x7FC00000, 0xFF800000]),
                          rng.randrange(256), rng.randrange(256))          # v0.9 RP1: pw_vaux, pw_flags
        assert il.Snap.unpack(raw).pack() == raw and il.encode(il.TYPE_SNAP2, raw) == il.encode_snap(il.Snap.unpack(raw))
    for bad in (dict(phase=1), dict(h_tgt_m=1.0), dict(spare=1), dict(phase=256, h_tgt_m=0.0)):
        with pytest.raises(ValueError):
            il.Snap(0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, **bad)
    with pytest.raises(ValueError):
        il.Snap.unpack(bytes(50))
    with pytest.raises(ValueError):
        il.encode(il.TYPE_SNAP2, bytes(49))


def test_snap2_decoder_and_vtol_com_by_hand():
    CV = il.ACCEPT_COM_VTOL
    assert il.SYNC0 not in SNAP2_A[1:] and not _inner_header(SNAP2_A)
    assert _dec(SNAP2_A + CMD_A) == ([il.Frame(il.TYPE_SNAP2, SNAP2_A[4:-2]), il.Frame(il.TYPE_CMD, CMD_A[4:-2])],
                                     _stats(frames=2), 0)
    # the VTOL COM takes only SNAP2: SNAP (LEN 49), CMD, STAT rejected at LEN, their bytes hunted
    fr, st, _ = _dec(SNAP_A + CMD_A + SNAP2_A + STAT_A, accept=CV)
    assert fr == [il.Frame(il.TYPE_SNAP2, SNAP2_A[4:-2])] and st == _stats(frames=1, bad_len=3, skipped_bytes=55 + 14 + 42)
    # the ballistic COM and the FCU refuse SNAP2 at LEN 57 (v0.9 RP1; v0.7: 55)
    assert _dec(SNAP2_A + SNAP_A, accept=il.ACCEPT_COM)[:2] == ([il.Frame(il.TYPE_SNAP, SNAP_A[4:-2])],
                                                                _stats(frames=1, bad_len=1, skipped_bytes=63))
    assert _dec(SNAP2_A + CMD_A, accept=il.ACCEPT_FCU)[1] == _stats(frames=1, bad_len=1, skipped_bytes=63)
    # LEN 57 with TYPE SNAP, LEN 49 with TYPE SNAP2: bad_len (ALL); LEN 57 + TYPE CMD on the VTOL COM: bad_type
    assert _dec(bytes([0xC5, 0x5C, 57, il.TYPE_SNAP]) + CMD_A)[1] == _stats(frames=1, bad_len=1, skipped_bytes=4)
    assert _dec(bytes([0xC5, 0x5C, 49, il.TYPE_SNAP2]) + CMD_A)[1] == _stats(frames=1, bad_len=1, skipped_bytes=4)
    assert _dec(bytes([0xC5, 0x5C, 57, il.TYPE_CMD]) + SNAP2_A, accept=CV)[1] == _stats(frames=1, bad_type=1, skipped_bytes=4)
    # need on the VTOL COM: the rest of a SNAP2 (63 - n) before LEN, cmd_need never
    d = il.IlinkDecoder(CV)
    d.feed(bytes([0xC5]))
    assert (d.need, d.cmd_need) == (62, il.NEED_NEVER)
    d.feed(bytes([0x5C, 57]))
    assert d.need == 60
    # the ALL decoder: a SNAP2 candidate keeps cmd_need large (its CRC low byte is the first that can reject it)
    d = il.IlinkDecoder()
    d.feed(SNAP2_A[:4])
    assert d.need == 59 and d.cmd_need == 57 + 1 + 13
    # a single byte error in a SNAP2 never loses the frame behind it (ALL and the VTOL COM)
    n = 0
    for kind, j, v, s in _variants(SNAP2_A, [0xC5, 0x5C, 0x39, 0x31, 0x04, 0x02, 0x00]):     # 0x39: SNAP2 LEN 57
        for acc, nxt in ((il.ACCEPT_ALL, CMD_B), (CV, SNAP2_A)):
            frames, _, pend = _dec(s + nxt, accept=acc)
            assert frames and frames[-1] == il.Frame(nxt[3], nxt[4:-2]) and pend == 0, (kind, j, v, acc)
            n += 1
    assert n > 1500


def test_telem2_frame_from_snap2_equals_vtol_supervisor_telemetry():
    """VTOL build: the COM pipeline SNAP2 -> TELEM2 reproduces byte for byte the TELEM2 frame sim.onboard builds itself
    (phase and target included; throttle = the previous tick's ESC throttle, carried as throttle_milli)."""
    p = json.load(open(il.PARAMS_PATH, encoding="utf-8"))
    sp = SupervisorParams.from_params(p, mode=lp.MODE_VTOL, gps=False)     # the v0.7.1 VTOL build (SNAP2 -> TELEM2)
    sup = OnboardSupervisor(sp, seed=0x2E)
    rng = random.Random(19)
    seq, n_replies, phases = 0, 0, set()
    script = {10: lp.CMD_ON, 900: lp.CMD_UP, 1000: lp.CMD_UP, 1100: lp.CMD_DOWN, 1300: lp.CMD_OFF, 1500: lp.CMD_UP}
    for t in range(2600):
        frames = []
        if t in script or t % 25 == 0:
            cmd = script.get(t, lp.CMD_NONE)
            seq = (seq + 1) & 0xFF
            frames = [UplinkFrame(seq, lp.Command(cmd, 0, sup.nonce))]
        q = quat.from_euler(math.radians(rng.uniform(-10, 10)), math.radians(rng.uniform(-10, 10)),
                            math.radians(rng.uniform(-180, 180)))
        q32 = tuple(_f32(c) for c in q)
        e = quat.to_euler(np.array(q32))
        thr_prev = _f32(rng.uniform(0.5, 1.0)) if sup.state == 2 else 0.0
        inp = OnboardInputs(tick=t, frames=frames, h_m=_f32(rng.uniform(-1, 8)), vz_mps=_f32(rng.uniform(-1.5, 1.5)),
                            tilt_deg=_f32(rng.uniform(0, 3)), v_cell=_f32(rng.uniform(3.8, 4.1)),
                            roll_deg=math.degrees(e[0]), pitch_deg=math.degrees(e[1]), yaw_deg=math.degrees(e[2]),
                            loop_max_us=_f32(rng.uniform(2000, 3500)), vc_at_target=t in (800, 1700),
                            vc_touchdown=t == 2000, throttle_prev=thr_prev)
        out = sup.update(inp)
        phases.add(out.phase)
        if out.reply_seq is None:
            continue
        thr_milli = lp.quantize(thr_prev, lp.THROTTLE_SCALE, lp.U16)      # the FCU captures the ESC command in milli
        src = il.TlmSource(state=int(out.state), flags=out.flags, reason=out.reason, ack_seq=out.ack_seq, nonce=out.nonce,
                           throttle_milli=thr_milli, tick=t, h_m=inp.h_m, vz_mps=inp.vz_mps, roll_deg=0.0, pitch_deg=0.0,
                           yaw_deg=0.0, tilt_deg=inp.tilt_deg, v_cell=inp.v_cell, loop_max_us=inp.loop_max_us,
                           phase=out.phase, h_tgt_m=out.h_tgt_m)
        snap = il.Snap.unpack(il.encode_snap(il.snap_from_tlm_src(out.reply_seq, src, q32))[4:-2])   # over the wire
        assert snap.is_v2
        assert il.telem_frame_from_snap(snap, VID, HZ) == lp.encode_telemetry(VID, out.reply_seq, out.telemetry), t
        n_replies += 1
    assert n_replies > 90 and phases >= {lp.PHASE_NONE, lp.PHASE_SPOOL, lp.PHASE_CLIMB, lp.PHASE_HOLD, lp.PHASE_LAND,
                                         lp.PHASE_LANDED}
