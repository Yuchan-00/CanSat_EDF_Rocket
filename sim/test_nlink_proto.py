"""Tests of the NAV <-> FCU nlink reference (``sim/nlink_proto.py``, SPEC 10.5, v0.6.3).

Independent checks, not the code checking itself: constants, flag bits and struct layouts against the C header
(``src/core/nlink_proto.h``, parsed here), payload and frame layouts against hand-placed bytes, the CRC against
``sim.link_proto`` and the bitwise definition, every float class bit for bit, every single-byte error refused, the
FCU half's ignored tail, the FCU / NAV semantics of the header comment as truth tables, the FCU slave's byte
alignment rules (``SpiSlaveModel``, normative for ``src/hal/nlink_spi.c``) on cases worked out by hand, and the
link timing of params.json.  Writes ``sim/out/nlink_vectors.json`` (the firmware host test's vectors,
``tools/build_host_test.py``).

Run: ``.venv/Scripts/python.exe -m pytest sim/test_nlink_proto.py -q -p no:cacheprovider``.
"""
from __future__ import annotations

import json
import math
import os
import random
import re
import struct
import sys

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sim import link_proto as lp  # noqa: E402
from sim import nlink_proto as nl  # noqa: E402

CORE = os.path.join(_ROOT, "firmware", "atmega128_fc", "src", "core")
HEADER = open(os.path.join(CORE, "nlink_proto.h"), encoding="utf-8").read()
CSRC = open(os.path.join(CORE, "nlink_proto.c"), encoding="utf-8").read()


def _hdr_const(name: str) -> int:
    m = re.search(rf"#define\s+{name}\s+(0x[0-9A-Fa-f]+|\d+)u?\b", HEADER)
    assert m, f"{name} not in nlink_proto.h"
    return int(m.group(1), 0)


def _struct_layout(name: str) -> tuple[str, list[str]]:
    """struct-module format and member names of ``typedef struct {...} name;`` in nlink_proto.h."""
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
            if arr:
                fmt += codes[typ] * int(arr.group(2))
                names += [arr.group(1)] * int(arr.group(2))
            else:
                fmt += codes[typ]
                names.append(var)
    return fmt, names


# --------------------------------------------------------------------------- #
# constants and layouts == the C header
# --------------------------------------------------------------------------- #
def test_constants_match_header():
    assert (_hdr_const("NL_SYNC0"), _hdr_const("NL_SYNC1")) == (nl.SYNC0, nl.SYNC1) == (0xD5, 0x5D)
    assert (_hdr_const("NL_TYPE_NAV"), _hdr_const("NL_TYPE_FCU")) == (nl.TYPE_NAV, nl.TYPE_FCU) == (1, 2)
    assert _hdr_const("NL_NAV_LEN") == nl.NAV_LEN == 66 and _hdr_const("NL_HEADER_BYTES") == nl.HEADER_BYTES == 4
    assert _hdr_const("NL_FCU_LEN") == nl.FCU_LEN == 4
    assert re.search(r"#define NL_XFER_BYTES \(NL_HEADER_BYTES \+ NL_NAV_LEN \+ 2u\)", HEADER) and nl.XFER_BYTES == 72
    assert re.search(r"#define NL_FCU_FRAME_BYTES \(NL_HEADER_BYTES \+ NL_FCU_LEN \+ 2u\)", HEADER)
    assert nl.FCU_FRAME_BYTES == 10
    flags = {"NL_F_IMU_OK": nl.F_IMU_OK, "NL_F_BARO_OK": nl.F_BARO_OK, "NL_F_ALIGNED": nl.F_ALIGNED,
             "NL_F_GYRO_SAT": nl.F_GYRO_SAT, "NL_F_EST_VALID": nl.F_EST_VALID, "NL_F_BARO_NEW": nl.F_BARO_NEW,
             "NL_F_BOOT": nl.F_BOOT, "NL_FF_BENCH": nl.FF_BENCH, "NL_FF_RECOVERY": nl.FF_RECOVERY}
    for name, val in flags.items():
        assert _hdr_const(name) == val, name
    assert sorted(v for k, v in flags.items() if k.startswith("NL_F_")) == [1 << i for i in range(7)]


NAV_H = open(os.path.join(_ROOT, "firmware", "atmega128_fc", "src", "nav", "nav.h"), encoding="utf-8").read()
WATCHDOG_H = open(os.path.join(_ROOT, "firmware", "atmega128_fc", "src", "hal", "watchdog.h"), encoding="utf-8").read()


def test_reset_cause_bits_match_nav_h_and_the_contract():
    """reset_cause = MCUCSR (bits 4..0, hal/watchdog.h RESET_*) | NAV_RC_* (bits 7..5, nav/nav.h); the contract text of
    nlink_proto.h names every NAV_RC_* bit with its value; the two parts never overlap."""
    def define(text: str, name: str) -> int:
        m = re.search(rf"#define\s+{name}\s+(0x[0-9A-Fa-f]+)u?\b", text)
        assert m, name
        return int(m.group(1), 16)

    rc = {"NAV_RC_MCUCSR_MASK": nl.NAV_RC_MCUCSR_MASK, "NAV_RC_TAG_MISMATCH": nl.NAV_RC_TAG_MISMATCH,
          "NAV_RC_BARO_INIT_FAIL": nl.NAV_RC_BARO_INIT_FAIL, "NAV_RC_IMU_INIT_FAIL": nl.NAV_RC_IMU_INIT_FAIL}
    for name, val in rc.items():
        assert define(NAV_H, name) == val, name
    assert (nl.NAV_RC_MCUCSR_MASK, nl.NAV_RC_TAG_MISMATCH, nl.NAV_RC_BARO_INIT_FAIL, nl.NAV_RC_IMU_INIT_FAIL) == \
        (0x1F, 0x20, 0x40, 0x80)
    for name in ("NAV_RC_TAG_MISMATCH", "NAV_RC_BARO_INIT_FAIL", "NAV_RC_IMU_INIT_FAIL"):
        assert re.search(rf"0x{rc[name]:02X} {name}\b", HEADER), f"nlink_proto.h does not document {name}"
    mcucsr = {define(WATCHDOG_H, f"RESET_{k}"): v for k, v in (("POWER_ON", "power_on"), ("EXTERNAL", "external"),
                                                               ("BROWN_OUT", "brown_out"), ("WATCHDOG", "watchdog"),
                                                               ("JTAG", "jtag"))}
    assert mcucsr == nl.MCUCSR_RESET_NAMES and sum(mcucsr) == nl.NAV_RC_MCUCSR_MASK
    assert not nl.NAV_RC_MCUCSR_MASK & sum(nl.NAV_RC_NAMES)
    assert nl.nav_reset_cause_names(0x08 | 0x80 | 0x20) == (["watchdog"], ["tag_mismatch", "imu_init_fail"])
    assert nl.nav_reset_cause_names(0x41) == (["power_on"], ["baro_init_fail"])
    assert nl.nav_reset_cause_names(0) == ([], [])


def test_struct_layouts_match_header():
    fmt, names = _struct_layout("nl_nav_t")
    assert names[0] == "seq" and fmt[1] == "B"                     # SEQ travels in the frame header
    assert struct.calcsize("<" + fmt[2:]) == struct.calcsize(nl.NAV_FMT) and "<" + fmt[2:] == "<" + "f" * 14 + "HHHHBB"
    assert nl.NAV_FMT == "<4f3f3fffffHHHHBB"
    assert names[1:] == ["q"] * 4 + ["rate"] * 3 + ["accel"] * 3 + ["h_m", "vz_mps", "h_baro_rel_m", "tilt_deg"] + \
        list(nl.NAV_INT_FIELDS)
    fmt, names = _struct_layout("nl_fcu_t")
    if names == list(nl.FCU_FIELDS[:4]):                            # v0.6.3 header: seq, state, fflags, params_tag
        pytest.xfail("C nl_fcu_t has no thr_q member yet (v0.7.1: the FCU frame's byte 7, sim/nlink_proto.py) - C port")
    assert fmt == "<BBBBB" and names == list(nl.FCU_FIELDS)


def test_c_core_uses_the_link_crc_and_writes_struct_only_when_valid():
    # P1 I5 (fw09): the CRC call is NL_CRC16 - the flash-table nl_crc16 (VTOL, == il_crc16 on every nlink vector) or
    # il_crc16 itself (ballistic images)
    assert ("il_crc16(" in CSRC or ("NL_CRC16(" in CSRC and "#define NL_CRC16 il_crc16" in CSRC)) and "LP_CRC_INIT" in CSRC
    dec = CSRC[CSRC.index("uint8_t nl_decode_nav"):]
    assert dec.index("if (!check(in, NL_TYPE_NAV, NL_NAV_LEN))") < dec.index("n->seq = in[3];")


# --------------------------------------------------------------------------- #
# payloads and frames
# --------------------------------------------------------------------------- #
def _crc_bitwise(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def test_layout_hand_placed():
    n = nl.Nav(0x42, (1.0, -2.0, 0.5, 0.25), (3.0, 4.0, 5.0), (6.0, 7.0, 8.0), 9.0, 10.0, 11.0, 12.0,
               0x0102, 0x0304, 0x0506, 0x0708, 0x09, 0x0A)
    fr = nl.encode_nav(n)
    assert len(fr) == 72 and fr[:4] == bytes([0xD5, 0x5D, 0x01, 0x42])
    f = [1.0, -2.0, 0.5, 0.25, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]
    for i, x in enumerate(f):
        assert fr[4 + 4 * i:8 + 4 * i] == struct.pack("<f", x), i
    assert fr[60:70] == bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07, 0x09, 0x0A])
    crc = _crc_bitwise(fr[2:70])
    assert fr[70:72] == bytes([crc & 0xFF, crc >> 8]) and crc == lp.crc16_ccitt(fr[2:70])
    ff = nl.encode_fcu(nl.Fcu(0x42, lp.STATE_POWERED, nl.FF_RECOVERY, 0x5A))
    crc = _crc_bitwise(bytes([0x02, 0x42, 2, 2, 0x5A, 0]))
    assert ff == bytes([0xD5, 0x5D, 0x02, 0x42, 2, 2, 0x5A, 0, crc & 0xFF, crc >> 8]) + bytes(62)
    assert nl.crc16(b"123456789") == 0x29B1


def test_v3_layout_hand_placed_and_the_exchange_stays_72_bytes():
    """nlink v3 (v0.8 VTOL + GPS): TYPE 3, 66-byte payload - 12 floats, 3 i16, 3 u8, u16, 2 u8, the 5-byte GPS slot."""
    slot = bytes([0x93, 0x11, 0x22, 0x33, 0x44])
    n = nl.Nav3(0x42, (1.0, -2.0, 0.5, 0.25), (3.0, 4.0, 5.0), 6.0, 7.0, 8.0, 0.75, 9.0, -300, 301, -2, 254, 255, 7,
                0x0708, 0x09, 0x0A, slot)
    fr = nl.encode_nav3(n)
    assert len(fr) == nl.XFER_BYTES == 72 and nl.NAV3_LEN == nl.NAV_LEN == 66 and fr[:4] == bytes([0xD5, 0x5D, 0x03, 0x42])
    for i, x in enumerate([1.0, -2.0, 0.5, 0.25, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 0.75, 9.0]):
        assert fr[4 + 4 * i:8 + 4 * i] == struct.pack("<f", x), i
    assert fr[52:58] == struct.pack("<hhh", -300, 301, -2) and fr[58:61] == bytes([254, 255, 7])
    assert fr[61:63] == bytes([0x08, 0x07]) and fr[63:65] == bytes([0x09, 0x0A]) and fr[65:70] == slot
    crc = _crc_bitwise(fr[2:70])
    assert fr[70:72] == bytes([crc & 0xFF, crc >> 8])
    assert nl.decode_nav3(fr) == n and nl.decode_nav(fr) is None and nl.nav_reject_reason(fr) == "type"
    assert nl.decode_nav3(nl.encode_nav(nl.Nav(1))) is None and nl.nav3_reject_reason(nl.encode_nav(nl.Nav(1))) == "type"
    for j in range(72):                                              # every single-byte error refused
        for v in (fr[j] ^ 0x01, fr[j] ^ 0x80, (fr[j] + 77) & 0xFF):
            assert nl.decode_nav3(fr[:j] + bytes([v]) + fr[j + 1:]) is None, j
    # the FCU frame is unchanged by v3
    assert nl.FCU_FRAME_BYTES == 10 and nl.FCU_FMT == "<BBBB"


def test_v3_from_physical_quantisation_tilt_and_saturation():
    from sim.vertical_ctrl import cos_tilt_from_q
    q = (0.99965, -0.0125, 0.0213, 0.0087)
    n = nl.Nav3.from_physical(5, q, (0.1, 0.2, 0.3), (0.405, -0.415, 9.71), 5.0, -0.1, 5.2, 300, 256, 7, 70000,
                              nl.F_IMU_OK, 1, nl.SLOT_EMPTY)
    assert (n.accel_x_q, n.accel_y_q, n.h_baro_rel_cm) == (lp.quantize(0.405, 100.0, lp.I16), -42, 520)
    assert (n.imu_fail, n.baro_fails, n.twi_errors, n.loop_us_max) == (255, 255, 7, 0xFFFF)
    assert n.cos_tilt == pytest.approx(cos_tilt_from_q(q), abs=2e-7) and n.cos_tilt == nl.f32(n.cos_tilt)
    assert n.tilt_deg == pytest.approx(math.degrees(math.acos(cos_tilt_from_q(q))), abs=2e-4)
    assert n.accel == pytest.approx((0.41, -0.42, 9.71), abs=1e-6)
    big = nl.Nav3.from_physical(0, (1, 0, 0, 0), (0, 0, 0), (400.0, -400.0, 0.0), 0, 0, 1000.0, 0, 0, 0, 0, 0, 0)
    assert (big.accel_x_q, big.accel_y_q, big.h_baro_rel_cm) == (32767, -32768, 32767)
    assert nl.nav3_tilt((1.0, 0.0, 0.0, 0.0)) == (1.0, 0.0)
    assert nl.nav3_tilt((math.cos(0.5), math.sin(0.5), 0.0, 0.0))[1] == pytest.approx(math.degrees(1.0), abs=1e-4)
    with pytest.raises(ValueError):
        nl.Nav3(0, slot=bytes(4))
    with pytest.raises(ValueError):
        nl.Nav3(0, imu_fail=256)


def test_v3_slot_layout_hand_placed():
    from sim.gps_nav import GpsSnapshot
    s = GpsSnapshot(flags=0xA5, num_sv=12, hacc_dm=31, reset_count=258 & 0xFF, off_x_cm=-2, off_y_cm=3, px_cm=1000,
                    py_cm=-1000, vx_cms=-1, vy_cms=32767, cos_q15=-32767, sin_q15=12345)
    assert nl.encode_slot(s, 33, 0) == bytes([(1 << 3) | 0, 0xA5, 12, 31, 2])                 # cycle 33 & 31 = 1
    assert nl.encode_slot(s, 1, 1) == bytes([0x09]) + struct.pack("<hh", -2, 3)
    assert nl.encode_slot(s, 1, 2) == bytes([0x0A]) + struct.pack("<hh", 1000, -1000)
    assert nl.encode_slot(s, 1, 3) == bytes([0x0B]) + struct.pack("<hh", -1, 32767)
    assert nl.encode_slot(s, 1, 4) == bytes([0x0C]) + struct.pack("<hh", -32767, 12345)
    assert nl.encode_slot(None, 5, 3) == nl.SLOT_EMPTY and nl.decode_slot(nl.SLOT_EMPTY) is None
    assert nl.decode_slot(bytes([0xFC, 1, 2, 3, 4])) == (31, 4, bytes([1, 2, 3, 4]))
    with pytest.raises(ValueError):
        nl.slot_part(s, 5)


def test_pack_unpack_every_float_class():
    rng = random.Random(4)
    patterns = [0x7FC00000, 0xFFC00000, 0x7FC0BEEF, 0xFFFFFFFF, 0x7F800000, 0xFF800000, 0x80000000, 0x00000000,
                0x00000001, 0x007FFFFF, 0x80000001, 0x7F7FFFFF, 0x00800000, 0x3F800000]
    for _ in range(500):
        bits = [rng.choice(patterns) if rng.random() < 0.5 else nl.f32_bits(rng.uniform(-1e6, 1e6)) for _ in range(14)]
        ints = [rng.randrange(65536) for _ in range(4)] + [rng.randrange(256), rng.randrange(256)]
        pl = nl.nav_payload_from_bits(bits, ints)
        seq = rng.randrange(256)
        n = nl.decode_nav(nl.encode_nav_payload(seq, pl))
        assert n is not None and n.seq == seq and n.pack() == pl
        assert nl.nav_fields(seq, pl) == {"seq": seq, "float_bits": bits, "ints": dict(zip(nl.NAV_INT_FIELDS, ints))}
    with pytest.raises(ValueError):
        nl.Nav(256)
    with pytest.raises(ValueError):
        nl.Nav(0, imu_fail=65536)
    with pytest.raises(ValueError):
        nl.Fcu(0, 300)
    with pytest.raises(ValueError):
        nl.decode_nav(bytes(71))
    assert nl.f32(1e39) == math.inf and nl.f32_bits(-0.0) == 0x80000000


def test_every_single_byte_error_is_refused_and_the_fcu_tail_ignored():
    n = nl.encode_nav(nl.Nav(9, (0.999, 0.01, -0.02, 0.03), flags=nl.F_IMU_OK | nl.F_EST_VALID, loop_us_max=1500))
    f = nl.encode_fcu(nl.Fcu(9, lp.STATE_ARMED, 0, 0x48))
    for j in range(72):
        for v in range(256):
            if v != n[j]:
                assert nl.decode_nav(n[:j] + bytes([v]) + n[j + 1:]) is None, (j, v)
            if v != f[j]:
                g = f[:j] + bytes([v]) + f[j + 1:]
                assert (nl.decode_fcu(g) is None) == (j < 10), (j, v)
    assert nl.decode_nav(f) is None and nl.decode_fcu(n) is None
    assert nl.nav_reject_reason(bytes(72)) == "sync" and nl.nav_reject_reason(f) == "type" and nl.fcu_reject_reason(n) == "type"
    # byte 7 (v0.6.3 "reserved", v0.7.1 thr_q) is CRC-covered and returned as decoded, any value
    assert nl.decode_fcu(nl.encode_fcu(nl.Fcu(1, 2, 3, 4), reserved=0x77)) == nl.Fcu(1, 2, 3, 4, 0x77)
    assert nl.encode_fcu(nl.Fcu(1, 2, 3, 4, 0x77)) == nl.encode_fcu(nl.Fcu(1, 2, 3, 4), reserved=0x77)


def test_fcu_throttle_byte_quantisation_and_ballistic_bytes():
    """v0.7.1 thr_q: u -> round_half_away(u x 250) in 0..250 (NaN -> 0), back u = min(q, 250) / 250; the ballistic FCU
    writes 0, so its exchange bytes are the v0.6.3 ones (the frame below was produced by the v0.6.3 encoder: same CRC)."""
    assert nl.FCU_THR_SCALE == 250 and nl.FCU_LEN == 4 and nl.FCU_FRAME_BYTES == 10   # frame size unchanged
    cases = {0.0: 0, 0.002: 1, 0.0019999: 0, 0.006: 2, 0.5: 125, 0.9148: 229, 0.998: 250, 0.9979999: 249, 1.0: 250,
             1.5: 250, -0.1: 0, float("nan"): 0, float("inf"): 250, float("-inf"): 0}
    for u, q in cases.items():
        assert nl.fcu_throttle_q(u) == q, u
    for q in range(256):
        assert nl.fcu_throttle(q) == min(q, 250) / 250
        assert nl.Fcu(0, 2, 0, 0, q).throttle == nl.fcu_throttle(q)
    for k in range(251):                                            # every step round-trips; error <= half a step
        assert nl.fcu_throttle_q(k / 250) == k
    rng = random.Random(7)
    for _ in range(2000):
        u = rng.random()
        assert abs(nl.fcu_throttle(nl.fcu_throttle_q(u)) - u) <= 0.5 / 250 + 1e-15
    assert nl.nav_throttle(None) == 0.0 and nl.nav_throttle(nl.Fcu(1, 2, 0, 0, 229)) == 229 / 250
    # v0.6.3: encode_fcu(Fcu(0x42, POWERED, RECOVERY, 0x5A)) with the reserved byte 0 (test_layout_hand_placed)
    crc = _crc_bitwise(bytes([0x02, 0x42, 2, 2, 0x5A, 0]))
    assert nl.encode_fcu(nl.Fcu(0x42, lp.STATE_POWERED, nl.FF_RECOVERY, 0x5A)) ==         bytes([0xD5, 0x5D, 0x02, 0x42, 2, 2, 0x5A, 0, crc & 0xFF, crc >> 8]) + bytes(62)
    ff = nl.encode_fcu(nl.Fcu(0x42, lp.STATE_POWERED, 0, 0x5A, 229))
    crc = _crc_bitwise(bytes([0x02, 0x42, 2, 0, 0x5A, 229]))
    assert ff[:10] == bytes([0xD5, 0x5D, 0x02, 0x42, 2, 0, 0x5A, 229, crc & 0xFF, crc >> 8])
    assert nl.decode_fcu(ff) == nl.Fcu(0x42, 2, 0, 0x5A, 229)
    with pytest.raises(ValueError):
        nl.Fcu(0, 2, 0, 0, 256)


# --------------------------------------------------------------------------- #
# semantics (nlink_proto.h header comment)
# --------------------------------------------------------------------------- #
def test_fcu_nav_inputs_truth_table():
    flying = (lp.STATE_POWERED, lp.STATE_UNPOWERED, lp.STATE_RELEASED, lp.STATE_ABORT)
    assert nl.FLYING_STATES == frozenset(flying)
    for state in range(6):
        assert nl.fcu_nav_inputs(None, state) == nl.FcuNavInputs(False, False, False, False)
        for flags in range(128):
            r = nl.fcu_nav_inputs(nl.Nav(1, flags=flags), state)
            imu = bool(flags & nl.F_IMU_OK)
            est = bool(flags & nl.F_EST_VALID)
            assert r.imu_ok == (imu and (est or state not in flying)), (state, flags)
            assert r.baro_ok == bool(flags & nl.F_BARO_OK) and r.aligned == bool(flags & nl.F_ALIGNED)
            assert r.gyro_saturated == bool(flags & nl.F_GYRO_SAT)
    # a NAV that reset in flight: IMU fine, no EST_VALID -> imu_ok false in POWERED (I9), true on the pad
    boot = nl.Nav(0, flags=nl.F_IMU_OK | nl.F_BARO_OK | nl.F_BOOT)
    assert not nl.fcu_nav_inputs(boot, lp.STATE_POWERED).imu_ok and nl.fcu_nav_inputs(boot, lp.STATE_SAFE).imu_ok


def test_nav_pad_mode():
    """Estimator: pad alignment on SAFE / ARMED without FF_BENCH.  Altitude filter: held on SAFE / ARMED whatever the
    fflags - FF_BENCH switches only the estimator (nav_main.c on_pad / pad_alt)."""
    assert nl.nav_pad_mode(None) and nl.nav_alt_pad_mode(None)
    for st in range(6):
        for ff in range(4):
            pad = st in (lp.STATE_SAFE, lp.STATE_ARMED)
            assert nl.nav_pad_mode(nl.Fcu(0, st, ff, 0)) == (pad and not ff & nl.FF_BENCH), (st, ff)
            assert nl.nav_alt_pad_mode(nl.Fcu(0, st, ff, 0)) == pad, (st, ff)
    bench = nl.Fcu(0, lp.STATE_ARMED, nl.FF_BENCH, 0)
    assert nl.nav_alt_pad_mode(bench) and not nl.nav_pad_mode(bench)       # bench_vanes: gyro-only, altitude held


def test_nav_firmware_switches_are_the_reference_ones():
    """nav_main.c computes the two switches exactly as nav_alt_pad_mode / nav_pad_mode: pad_alt from the state alone,
    on_pad = pad_alt && !NL_FF_BENCH; the altitude reset and the ground-reference LPF hang on pad_alt, the estimator's
    pad alignment and ALIGNED on on_pad."""
    src = open(os.path.join(_ROOT, "firmware", "atmega128_fc", "src", "nav", "nav_main.c"), encoding="utf-8").read()
    code = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    assert re.search(r"pad_alt\s*=\s*g_fcu\.state\s*==\s*ONB_SAFE\s*\|\|\s*g_fcu\.state\s*==\s*ONB_ARMED\s*;", code)
    assert re.search(r"bench\s*=\s*\(g_fcu\.fflags\s*&\s*NL_FF_BENCH\)\s*!=\s*0u\s*;", code)
    assert re.search(r"on_pad\s*=\s*pad_alt\s*&&\s*!bench\s*;", code)
    assert re.search(r"att_est_update\(&g_est,[^;]*on_pad\)\s*;", code)
    assert re.search(r"if\s*\(pad_alt\)\s*\{\s*g_alt\.h\s*=\s*0\.0f;\s*g_alt\.v\s*=\s*0\.0f;", code)
    assert re.search(r"if\s*\(pad_alt\)\s*\{\s*h_ground\s*=", code)
    assert re.search(r"if\s*\(on_pad\)\s*\{\s*aligned\s*=", code)
    # the contract text says the same (whitespace and comment stars folded)
    text = " ".join(re.sub(r"^\s*\*", " ", HEADER, flags=re.M).split())
    assert "switches only the estimator to gyro-only on the pad - the altitude stays held" in text
    assert "ground reference low-pass filtered while that state is SAFE or ARMED, whatever the fflags" in text


# --------------------------------------------------------------------------- #
# FCU slave byte alignment (SpiSlaveModel, normative for hal/nlink_spi.c)
# --------------------------------------------------------------------------- #
A = nl.encode_nav(nl.Nav(1, flags=nl.F_IMU_OK))
B = nl.encode_nav(nl.Nav(2, flags=nl.F_IMU_OK))
C = nl.encode_nav(nl.Nav(3, flags=nl.F_IMU_OK))
R1 = nl.encode_fcu(nl.Fcu(1, lp.STATE_SAFE, 0, 0x48))[:10]
R2 = nl.encode_fcu(nl.Fcu(2, lp.STATE_ARMED, 0, 0x48))[:10]


def _xchg(m: nl.SpiSlaveModel, frame: bytes, drop: tuple[int, ...] = ()) -> bytes:
    return bytes(m.byte(b) for j, b in enumerate(frame) if j not in drop)


def test_slave_clean_exchanges_and_reply_timing():
    m = nl.SpiSlaveModel()
    assert _xchg(m, A) == bytes(72)                               # nothing set yet: zeros
    m.set_reply(R1, ss_high=True)                                 # between exchanges: active at once
    assert _xchg(m, B) == R1 + bytes(62)
    assert m.take(True) == B and m.stats == {"frames": 2, "resyncs": 0, "overruns": 1}   # A was never taken
    miso = _xchg(m, A, ())[:30]
    assert miso == (R1 + bytes(62))[:30]


def test_slave_reply_set_during_an_exchange_waits_for_its_end():
    m = nl.SpiSlaveModel()
    m.set_reply(R1, True)
    first = bytes(m.byte(b) for b in A[:20])
    m.set_reply(R2, ss_high=False)                                # SS low, idx 20: pending
    rest = bytes(m.byte(b) for b in A[20:])
    assert first + rest == R1 + bytes(62)                         # this exchange unchanged
    assert _xchg(m, B) == R2 + bytes(62)                          # the next one carries it


def test_slave_lost_byte_resyncs_on_the_next_poll():
    m = nl.SpiSlaveModel()
    _xchg(m, A, drop=(30,))
    assert m.idx == 71 and m.take(ss_high=False) is None and m.idx == 71        # SS low: no resync
    assert m.take(ss_high=True) is None and m.idx == 0 and m.stats["resyncs"] == 1
    _xchg(m, B)
    assert m.take(True) == B


def test_slave_lost_byte_without_poll_costs_two_exchanges():
    m = nl.SpiSlaveModel()
    _xchg(m, A, drop=(30,))
    _xchg(m, B)                                                   # B's first byte completes the misaligned frame
    bad = m.take(ss_high=True)                                    # poll first: B's 71 rest bytes discarded
    assert bad == A[:30] + A[31:] + B[:1] and nl.decode_nav(bad) is None
    assert m.stats == {"frames": 1, "resyncs": 1, "overruns": 0}
    _xchg(m, C)
    assert m.take(True) == C


def test_slave_extra_byte_and_ss_glitch():
    m = nl.SpiSlaveModel()
    for b in A[:10] + bytes([0]) + A[10:]:                        # an SCK glitch adds a byte
        m.byte(b)
    bad = m.take(True)                                            # completed one byte early, 1 byte left -> resync
    assert bad == A[:10] + bytes([0]) + A[10:71] and m.idx == 0 and m.stats["resyncs"] == 1
    m = nl.SpiSlaveModel()
    _xchg(m, A, drop=(5,))                                        # SS glitch: the hardware drops the shifting byte
    m.poll(True)
    _xchg(m, B)
    assert m.take(True) == B and m.stats == {"frames": 1, "resyncs": 1, "overruns": 0}


def test_slave_resync_activates_a_pending_reply():
    m = nl.SpiSlaveModel()
    m.set_reply(R1, True)
    _xchg(m, A, drop=(3,))
    m.set_reply(R2, ss_high=True)                                 # idx 71: pending although SS is high
    assert m.pending == R2
    m.poll(True)                                                  # resync: active now, byte 0 preloaded
    assert _xchg(m, B) == R2 + bytes(62)


def test_slave_vector_scenarios_consistent():
    v = nl.make_test_vectors()
    names = {s["name"] for s in v["slave"]}
    assert {"clean_three", "lost_byte_resync", "lost_byte_no_poll", "extra_byte", "ss_glitch", "overrun",
            "reply_pending_mid_exchange", "reply_while_idle", "reply_after_resync"} <= names
    for s in v["slave"]:
        assert nl.run_slave_events(s["events"]) == {k: s[k] for k in ("miso_hex", "taken", "stats", "idx")}
        n_bytes = sum(1 for e in s["events"] if e[0] == "B")
        assert len(bytes.fromhex(s["miso_hex"])) == n_bytes
    tot = {k: sum(s["stats"][k] for s in v["slave"]) for k in ("frames", "resyncs", "overruns")}
    assert all(x > 0 for x in tot.values()), tot


# --------------------------------------------------------------------------- #
# timing of params.json flight_computer.nlink
# --------------------------------------------------------------------------- #
def test_params_timing():
    t = nl.load_nlink_params()
    assert t["nav_tick_hz"] == t["loop_hz"]
    tick = 1.0 / t["loop_hz"]
    xfer = nl.exchange_time_s(t["spi_clock_hz"], t["byte_gap_us"] * 1e-6)
    assert xfer == pytest.approx(72 * (8 / 2e6 + 4e-6)) and xfer < 0.25 * tick          # 576 us of a 4 ms tick
    assert tick < t["frame_timeout_ms"] * 1e-3 < 2 * tick                               # a lost frame costs one tick
    assert t["spi_mode"] == 0


# --------------------------------------------------------------------------- #
# firmware vectors
# --------------------------------------------------------------------------- #
def test_vectors_deterministic_consistent_and_write():
    v = nl.make_test_vectors()
    assert nl.make_test_vectors() == v
    assert v["crc16_check"]["crc"] == 0x29B1 and v["constants"]["XFER_BYTES"] == 72
    for nv in v["navs"]:
        pl = bytes.fromhex(nv["payload_hex"])
        fr = bytes.fromhex(nv["frame_hex"])
        assert len(pl) == 66 and fr == nl.encode_nav_payload(nv["seq"], pl)
        assert nl.nav_payload_from_bits(nv["fields"]["float_bits"], list(nv["fields"]["ints"].values())) == pl
        if nv["py_roundtrip"]:
            assert nl.decode_nav(fr).pack() == pl, nv["name"]
    names = {nv["name"] for nv in v["navs"]}
    assert {"nav_nan_inf", "nav_denormals", "nav_signalling_nan", "nav_nan_payload", "nav_sync_in_payload"} <= names
    for fv in v["fcus"]:
        fr = bytes.fromhex(fv["frame_hex"])
        assert fr == nl.encode_fcu(nl.Fcu(*fv["fields"])) and fr[10:] == bytes(62)
    for fv in v["fcus_vtol"]:                                       # v0.7.1 thr_q frames
        fr = bytes.fromhex(fv["frame_hex"])
        assert fr == nl.encode_fcu(nl.Fcu(*fv["fields"], fv["thr_q"])) and fr[7] == fv["thr_q"]
        assert fv["u_nav"] == nl.fcu_throttle(fv["thr_q"])
        if fv["u_in"] is not None:
            assert fv["thr_q"] == nl.fcu_throttle_q(fv["u_in"])
    for q in v["thr_quant"]:
        assert q["thr_q"] == nl.fcu_throttle_q(q["u"])
    for d in v["decodes"]:
        if d["fcu_valid"]:
            assert d["fcu_thr_q"] == bytes.fromhex(d["hex"])[7]
    outcomes = set()
    for d in v["decodes"]:
        buf = bytes.fromhex(d["hex"])
        assert len(buf) == 72 and d["nav_valid"] == (nl.decode_nav(buf) is not None)
        assert d["fcu_valid"] == (nl.decode_fcu(buf) is not None)
        outcomes.add((d["nav_reason"], d["fcu_reason"]))
    assert {r for r, _ in outcomes} >= {"", "sync", "type", "crc"} and {r for _, r in outcomes} >= {"", "sync", "type", "crc"}
    assert len([d for d in v["decodes"] if d["name"].startswith("bitflip_nav")]) == 72 * 8
    assert not any(d["nav_valid"] for d in v["decodes"] if d["name"].startswith("bitflip_nav"))
    assert all(d["fcu_valid"] for d in v["decodes"] if d["name"].startswith("fcu_tail"))
    # v0.8 nlink v3 block
    v3 = v["v3"]
    assert v3["constants"]["TYPE_NAV3"] == 3 and v3["constants"]["NAV3_LEN"] == 66
    for nv in v3["navs3"]:
        fr = bytes.fromhex(nv["frame_hex"])
        n3 = nl.decode_nav3(fr)
        assert n3 is not None and n3.pack().hex() == nv["payload_hex"] and nl.encode_nav3(n3) == fr
    for d in v3["decodes"]:
        buf = bytes.fromhex(d["hex"])
        assert d["nav3_valid"] == (nl.decode_nav3(buf) is not None) and d["nav_valid"] == (nl.decode_nav(buf) is not None)
    flips = [d for d in v3["decodes"] if d["name"].startswith("bitflip_nav3")]
    assert len(flips) == 72 * 8 and not any(d["nav3_valid"] for d in flips)
    by3 = {d["name"]: d for d in v3["decodes"]}
    assert by3["v063_frame_to_v3_decoder"]["nav3_reason"] == "type" and by3["v063_frame_to_v3_decoder"]["nav_valid"]
    assert by3["v3_payload_as_type_nav_valid_crc"]["nav_valid"] and not by3["v3_payload_as_type_nav_valid_crc"]["nav3_valid"]
    rx = {c["name"]: c for c in v3["rx"]}
    assert [o is not None for o in rx["complete"]["out"]] == [False] * 4 + [True]
    assert rx["lost_frame"]["broken"] == 1 and rx["lost_frame"]["sets"] == 1 and rx["late_start"]["sets"] == 1
    # v0.8.1 nlink v4 block (sim/glink_proto.py holds the GNS slot sequences)
    v4 = v["v4"]
    assert v4["constants"]["TYPE_NAV4"] == 4 and v4["constants"]["TYPE_FCU4"] == 5 and v4["constants"]["FCU4_FRAME_BYTES"] == 13
    for nv in v4["navs4"]:
        fr = bytes.fromhex(nv["frame_hex"])
        n4 = nl.decode_nav4(fr)
        assert n4 is not None and n4.pack().hex() == nv["payload_hex"] and nl.encode_nav4(n4) == fr
    for fv in v4["fcus4"]:
        buf = bytes.fromhex(fv["hex"])
        assert nl.encode_fcu4(nl.Fcu4(**fv["fields"])) == buf and buf[13:] == bytes(59)
    for d in v4["decodes"]:
        buf = bytes.fromhex(d["hex"])
        assert d["nav4_valid"] == (nl.decode_nav4(buf) is not None) and d["fcu4_valid"] == (nl.decode_fcu4(buf) is not None)
    flips4 = [d for d in v4["decodes"] if d["name"].startswith("bitflip_fcu4")]
    assert len(flips4) == 13 * 8 and not any(d["fcu4_valid"] for d in flips4)
    by4 = {d["name"]: d for d in v4["decodes"]}
    assert by4["v3_frame_to_v4"]["nav4_reason"] == "type" and by4["v3_frame_to_v4"]["nav3_valid"]
    assert by4["v4_frame_to_v3"]["nav4_valid"] and not by4["v4_frame_to_v3"]["nav3_valid"]
    assert by4["v071_fcu_to_fcu4"]["fcu4_reason"] == "type" and by4["v071_fcu_to_fcu4"]["fcu_valid"]
    assert by4["fcu4_tail_ignored"]["fcu4_valid"]
    sl = v4["slave"][0]
    assert sl["miso_hex"][:26] == v4["fcus4"][0]["hex"][:26] and len(sl["taken"]) == 2
    written = nl.write_test_vectors()
    assert written == v
    with open(nl.VECTORS_PATH, encoding="utf-8") as f:
        assert json.load(f) == v
