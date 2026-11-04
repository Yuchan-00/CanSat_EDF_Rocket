"""Tests of the v0.8.1 NAV <-> GNS link reference (``sim/glink_proto.py``) and the four-chip slot path (``sim.nlink_proto``
v4, ``sim.onboard.FcuStack`` gns mode, ``sim.fc_timing`` GNS path).

Independent checks, not the code checking itself: payload / frame layouts against hand-placed bytes, the CRC against a
bitwise CRC-16/CCITT-FALSE written here, every float bit pattern through a round trip, every single-bit error refused,
the streaming decoder's rules on cases worked out by hand, the NAV's slot phase and the FCU's staleness rules at their
exact ticks, the GNS filter equal BIT FOR BIT to the v0.8 NAV filter on float32-exact inputs (the STATE frame only
transports what the v0.8 NAV used), the GNS gap / reset / clock rules, the end-to-end latency NAV sample -> FCU attitude
input (L + 1 ticks), the params link timing, and the vectors file.

Run: ``.venv/Scripts/python.exe -m pytest sim/test_glink_proto.py -q -p no:cacheprovider``.
"""
from __future__ import annotations

import json
import math
import os
import random
import struct
import sys
from dataclasses import replace

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sim import fc_timing as fct  # noqa: E402
from sim import glink_proto as gl  # noqa: E402
from sim import gps_nav as gn  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim import nlink_proto as nl  # noqa: E402
from sim import position_ctrl as pc  # noqa: E402
from sim.onboard import FcuStack, OnboardInputs  # noqa: E402
from sim.state_machine import G0  # noqa: E402

with open(os.path.join(_ROOT, "params.json"), encoding="utf-8") as _f:
    P = json.load(_f)
T_MAX = float(P["edf"]["thrust_nominal_N"])
MASS = T_MAX / (1.177 * G0)


def crc_bitwise(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        for i in range(8):
            bit = ((b >> (7 - i)) & 1) ^ (c >> 15)
            c = ((c << 1) & 0xFFFF) ^ (0x1021 if bit else 0)
    return c


ST = gl.State(tick=0x01020304, fsum=(1.5, -2.25, 49.0), n=5, q=(1.0, -0.5, 0.25, 0.125), w=(0.5, -0.25, 2.0), h=3.75,
              nflags=gl.NF_FCU_OK | gl.NF_IMU_OK, fcu_state=lp.STATE_POWERED, pflags=0xAB, kt_q=0x4321)


# --------------------------------------------------------------------------- #
# layouts, CRC, round trips
# --------------------------------------------------------------------------- #
def test_state_layout_hand_placed():
    pl = ST.pack()
    assert len(pl) == gl.STATE_LEN == 54
    exp = (bytes([0x04, 0x03, 0x02, 0x01]) + b"".join(struct.pack("<f", x) for x in (1.5, -2.25, 49.0)) + bytes([5])
           + b"".join(struct.pack("<f", x) for x in (1.0, -0.5, 0.25, 0.125))
           + b"".join(struct.pack("<f", x) for x in (0.5, -0.25, 2.0)) + struct.pack("<f", 3.75)
           + bytes([0x06, lp.STATE_POWERED, 0xAB, 0x21, 0x43]))
    assert pl == exp
    assert gl.State.unpack(pl) == ST


def test_result_layout_is_the_five_slot_parts():
    r = gl.Result(tilt_x=-1234, tilt_y=567, flags=0x47, num_sv=9, hacc_dm=25,
                  pc=nl.pc_byte(pc.MODE_POS, True, pc.REQ_FENCE), pos_x_cm=-37, pos_y_cm=52, vel_x_cms=-8, vel_y_cms=11,
                  trim_x=205, trim_y=-611)
    pl = r.pack()
    assert len(pl) == gl.RESULT_LEN == 20 == 4 * nl.SLOT_PARTS
    parts = r.parts()
    assert parts[nl.SLOT4_TILT] == struct.pack("<hh", -1234, 567)
    assert parts[nl.SLOT4_STATUS] == bytes([0x47, 9, 25, pc.MODE_POS | nl.PC_HOLD | (pc.REQ_FENCE << 4)])
    assert parts[nl.SLOT4_POS] == struct.pack("<hh", -37, 52)
    assert parts[nl.SLOT4_VEL] == struct.pack("<hh", -8, 11)
    assert parts[nl.SLOT4_TRIM] == struct.pack("<hh", 205, -611)
    assert gl.Result.unpack(pl) == r and r.engaged and not gl.Result().engaged


def test_frame_bytes_and_crc():
    fr = gl.encode_state(0x9C, ST)
    assert len(fr) == gl.STATE_FRAME_BYTES == 60
    assert fr[:4] == bytes([0xA5, 0x5A, gl.TYPE_STATE, 0x9C]) and fr[4:-2] == ST.pack()
    c = crc_bitwise(fr[2:-2])
    assert fr[-2:] == bytes([c & 0xFF, c >> 8]) and c == lp.crc16_ccitt(fr[2:-2])
    assert crc_bitwise(b"123456789") == 0x29B1                      # CRC-16/CCITT-FALSE check value
    rf = gl.encode_result(0x9C, gl.Result())
    assert len(rf) == gl.RESULT_FRAME_BYTES == 26 and rf[2] == gl.TYPE_RESULT
    assert gl.decode_frame(fr) == (gl.TYPE_STATE, 0x9C, ST.pack())
    assert gl.decode_frame(rf) == (gl.TYPE_RESULT, 0x9C, gl.Result().pack())
    with pytest.raises(ValueError):
        gl.encode_state(256, ST)


def test_every_float_bit_pattern_survives():
    pats = (0x7FC00000, 0xFFC00000, 0x7F800000, 0xFF800000, 0x80000000, 0x00000001, 0x007FFFFF, 0x7F7FFFFF, 0x3F800001)
    for u in pats:
        x = struct.unpack("<f", struct.pack("<I", u))[0]
        st = replace(ST, fsum=(x, x, x), q=(x, x, x, x), w=(x, x, x), h=x)
        back = gl.State.unpack(st.pack())
        for v in (*back.fsum, *back.q, *back.w, back.h):
            assert struct.pack("<f", v) == struct.pack("<I", u)


def test_every_single_bit_error_is_refused():
    for fr in (gl.encode_state(7, ST), gl.encode_result(7, gl.Result(tilt_x=3, flags=1))):
        assert gl.reject_reason(fr) is None
        for j in range(len(fr)):
            for bit in range(8):
                b = bytearray(fr)
                b[j] ^= 1 << bit
                assert gl.decode_frame(bytes(b)) is None, (j, bit)


def test_wrong_type_and_length_are_refused():
    fr = gl.encode_state(7, ST)
    assert gl.reject_reason(fr[:-1]) == "length"
    assert gl.reject_reason(fr[:2] + bytes([3]) + fr[3:]) == "type"
    assert gl.reject_reason(fr[:2] + bytes([gl.TYPE_RESULT]) + fr[3:]) == "length"
    assert gl.reject_reason(bytes([0xA5, 0x5B]) + fr[2:]) == "sync"


# --------------------------------------------------------------------------- #
# streaming decoder
# --------------------------------------------------------------------------- #
def test_stream_decoder_rules():
    s = gl.encode_state(1, ST)
    r = gl.encode_result(2, gl.Result())
    d = gl.GlinkDecoder((gl.TYPE_STATE,))
    assert d.feed(bytes([0x00, 0x11]) + s) == [(gl.TYPE_STATE, 1, ST.pack())] and d.stats["skipped"] == 2
    d = gl.GlinkDecoder((gl.TYPE_STATE,))
    assert len(d.feed(bytes([0xA5, 0xA5]) + s)) == 1 and d.stats["skipped"] == 0         # 0xA5 0xA5 keeps SYNC2
    d = gl.GlinkDecoder((gl.TYPE_STATE,))
    assert d.feed(r + s) == [(gl.TYPE_STATE, 1, ST.pack())] and d.stats["wrong_dir"] == 1
    bad = bytearray(s)
    bad[30] ^= 1
    d = gl.GlinkDecoder((gl.TYPE_STATE,))
    assert len(d.feed(bytes(bad) + s)) == 1 and d.stats["bad_crc"] == 1
    d = gl.GlinkDecoder((gl.TYPE_STATE,))
    assert len(d.feed(bytes([0xA5, 0x5A, 0x09]) + s)) == 1 and d.stats["bad_type"] == 1
    d = gl.GlinkDecoder((gl.TYPE_STATE,))
    out = []
    for b in s:                                                       # one byte at a time
        x = d.feed_byte(b)
        if x is not None:
            out.append(x)
    assert out == [(gl.TYPE_STATE, 1, ST.pack())]
    d = gl.GlinkDecoder((gl.TYPE_STATE,))
    assert d.feed(s[:30] + s[31:] + s) == [] and d.stats["bad_crc"] == 1           # a lost byte costs the next frame too


# --------------------------------------------------------------------------- #
# params timing
# --------------------------------------------------------------------------- #
def test_params_link_and_latency_estimate():
    g = gl.glink_params(P)
    assert g["baud"] == gl.BAUD and g["baud_actual"] == pytest.approx(g["baud"])      # UBRR 3 at 16 MHz: 0 % error
    assert g["state_frame_s"] == pytest.approx(60 * 10 / 250000.0) and g["result_frame_s"] == pytest.approx(26e-5 * 4)
    assert g["latency_ticks"] == P["flight_computer"]["glink"]["latency_ticks_ASSUMED"] == fct.glink_latency_ticks(P)
    est = gl.timing_estimate(P)
    assert est["latency_ticks_min_est"] == 3 and est["margin_ms_at_params_L"] > 3.0
    assert est["result_at_nav_ms"] == pytest.approx(3.6 + 2.4 + 2.35 + 1.04)
    # bus load: 12 % / 5.2 % of the line
    assert 50 * gl.STATE_FRAME_BYTES / (gl.BAUD / 10) == pytest.approx(0.12)
    assert 50 * gl.RESULT_FRAME_BYTES / (gl.BAUD / 10) == pytest.approx(0.052)


def test_chain_ticks():
    assert fct.gns_chain_ticks("fcu") == {"sample_to_attitude": 10, "sample_to_request": 10,
                                          "fcu_inputs_age_at_attitude": 5}
    assert fct.gns_chain_ticks("gns", 3) == {"sample_to_attitude": 4, "sample_to_request": 7,
                                             "fcu_inputs_age_at_attitude": 6}
    with pytest.raises(ValueError):
        fct.gns_chain_ticks("nav")


# --------------------------------------------------------------------------- #
# nlink v4: the NAV's slot phase, the FCU's receiver
# --------------------------------------------------------------------------- #
R_HOLD = gl.Result(tilt_x=300, tilt_y=-900, flags=gn.GF_FIX_OK | gn.GF_GATE_OK, num_sv=9, hacc_dm=25,
                   pc=nl.pc_byte(pc.MODE_POS, True, 0), pos_x_cm=30, pos_y_cm=-40, vel_x_cms=5, vel_y_cms=0,
                   trim_x=200, trim_y=-600)


def test_slot_tx_phase_tilt_first_and_staleness():
    L = 3
    tx = nl.GnsSlotTx(5, L)
    pl = R_HOLD.pack()
    for k in range(0, 4 + L):
        assert tx.slot(k) == nl.SLOT_EMPTY                             # before the first result can be due
    tx.result(0, pl, 5)                                                # cycle 0 (sampled at tick 4), back at tick 5
    for i in range(5):                                                 # ticks 7..11 carry parts 0..4 of cycle 0
        assert tx.slot(4 + L + i) == bytes([i]) + pl[4 * i:4 * i + 4]
    tx.result(1, pl, 13)                                               # cycle 1 (tick 9) arrives AFTER its TILT tick 12
    for i in range(5):
        assert tx.slot(12 + i) == nl.SLOT_EMPTY
    tx.result(2, pl, 15)
    assert tx.slot(17) == bytes([(2 << 3) | 0]) + pl[:4]
    assert tx.latched == 2 and tx.missed == 1
    # a result older than its cycle's sample (left in the ring 256 cycles ago) is never re-sent
    tx2 = nl.GnsSlotTx(5, L)
    tx2.result(3, pl, 5)                                               # SEQ 3 but arrived at tick 5 < cycle 3's sample 19
    assert tx2.slot(5 * 3 + 4 + L) == nl.SLOT_EMPTY
    with pytest.raises(ValueError):
        nl.GnsSlotTx(5, 1)


def _slots(m: int, r: gl.Result) -> list[bytes]:
    return [bytes([((m & 0x1F) << 3) | i]) + p for i, p in enumerate(r.parts())]


def test_slot_rx_tilt_on_arrival_set_and_fallback():
    rx = nl.GnsSlotRx(5)
    s = _slots(4, R_HOLD)
    assert rx.target() == (False, (0.0, 0.0)) and rx.fallback()
    assert rx.feed(s[0]) is None
    assert rx.target() == (True, (0.03, -0.09))                        # the TILT applies the tick it arrives
    for p in s[1:4]:
        assert rx.feed(p) is None
    g = rx.feed(s[4])
    assert g is not None and g.mode == pc.MODE_POS and g.hold and g.pos == (0.3, -0.4) and g.trim == (0.02, -0.06)
    assert g.dist_m == pytest.approx(0.5) and rx.gate_ok and g.fix_ok
    assert rx.tlm(True) == (g.flags, 9, 25, pc.MODE_POS, pytest.approx(0.5))
    # the GNS goes silent: one missed cycle holds the TILT, the second switches to the TRIM, land_steps -> REQ_GPS_LOST
    for t in range(1, 30):
        rx.feed(None)
        eng, tgt = rx.target()
        age = t + 4                                                    # ticks since the TILT (it came 4 ticks before s[4])
        if age < 10:
            assert tgt == (0.03, -0.09) and not rx.fallback()
        else:
            assert tgt == (0.02, -0.06) and rx.fallback() and rx.tlm(True)[3] == pc.MODE_ATT
        assert eng
        assert rx.requests(5) == (pc.REQ_GPS_LOST if age >= 25 else 0)
    # not engaged: TILT_NONE
    rx2 = nl.GnsSlotRx(5)
    rx2.feed(_slots(1, gl.Result())[0])
    assert rx2.target() == (False, (0.0, 0.0))


def test_slot_rx_lost_frame_breaks_the_set_but_not_the_tilt():
    rx = nl.GnsSlotRx(5)
    s = _slots(6, R_HOLD)
    rx.feed(s[0])
    rx.feed(None)
    for p in s[2:]:
        out = rx.feed(p)
    assert out is None and rx.broken == 1 and rx.target() == (True, (0.03, -0.09))


def test_fcu4_frame_layout_and_semantics():
    pf = nl.fcu4_pflags(lp.PHASE_LAND, True, True, True, False, True)
    assert pf == lp.PHASE_LAND | 0x08 | 0x10 | 0x20 | 0x80
    f = nl.Fcu4(9, lp.STATE_POWERED, 0, 0x5A, 172, pf, nl.kt_quant(1.0731))
    buf = nl.encode_fcu4(f)
    assert len(buf) == nl.XFER_BYTES and buf[nl.FCU4_FRAME_BYTES:] == bytes(nl.XFER_BYTES - 13)
    assert buf[:11] == bytes([0xD5, 0x5D, nl.TYPE_FCU4, 9, lp.STATE_POWERED, 0, 0x5A, 172, pf]) + struct.pack(
        "<H", round(1.0731 * 16384))
    assert nl.decode_fcu4(buf) == f and nl.decode_fcu(buf) is None and nl.fcu4_reject_reason(buf) is None
    assert nl.fcu_reject_reason(buf) == "type"
    assert nl.kt_quant(1.4) == 22938 and nl.kt_quant(float("nan")) == 0 and nl.kt_quant(9.0) == 0xFFFF
    assert f.phase == lp.PHASE_LAND and f.k_t == pytest.approx(1.0731, abs=1 / 32768)
    # the NAV's pad / flight switches read the v4 FCU frame like the v3 one
    assert not nl.nav_pad_mode(f) and nl.nav_pad_mode(replace(f, state=lp.STATE_SAFE))
    # the slave driver model replies with the 13-byte frame
    m = nl.SpiSlaveModel(reply_bytes=nl.FCU4_FRAME_BYTES)
    m.set_reply(buf, True)
    nav = nl.encode_nav4(nl.Nav3(3))
    miso = bytes(m.byte(b) for b in nav)
    assert miso[:13] == buf[:13] and miso[13:] == bytes(59)
    assert nl.decode_nav4(nav) is not None and nl.decode_nav3(nav) is None and nl.nav3_reject_reason(nav) == "type"


# --------------------------------------------------------------------------- #
# the GNS: v0.8 filter equality, gap rules, clock
# --------------------------------------------------------------------------- #
LF0 = (375000000, 1270000000)


def _epoch_bytes(lf, t: float, x: float, y: float, vx: float, vy: float) -> bytes:
    lat, lon = lf.to_llh(x, y)
    ep = gn.GpsEpoch(itow_ms=200000000 + int(round(t * 1000)), lat_e7=lat, lon_e7=lon, hmsl_mm=1000, hacc_mm=2000,
                     vacc_mm=3000, vel_n_cms=int(round(vx * 100)), vel_e_cms=int(round(-vy * 100)), vel_d_cms=0,
                     sacc_cms=20, gps_fix=3, fix_ok=True, num_sv=9, pacc_cm=250)
    return gn.epoch_frames(ep)


def _stream(n_ticks: int, seed: int = 4):
    """Float32-exact IMU samples (dyadic) + UBX bytes per tick: a vehicle accelerating slowly (pad first)."""
    rng = random.Random(seed)
    gp = gn.NavGpsParams()
    lf = gn.LocalFrame.at(*LF0)
    lat = int(round(gp.latency_s * gp.loop_hz))
    pend: dict[int, bytes] = {}
    ticks = []
    for k in range(n_ticks):
        t = k / gp.loop_hz
        if k % 50 == 0:
            pend[k + lat] = _epoch_bytes(lf, t, 0.1 * t * t, -0.05 * t, 0.2 * t, -0.05)
        acc = (rng.randrange(-512, 512) / 256.0, rng.randrange(-512, 512) / 256.0, 9.75)
        w = (rng.randrange(-64, 64) / 1024.0, rng.randrange(-64, 64) / 1024.0, rng.randrange(-64, 64) / 1024.0)
        ticks.append((pend.pop(k, b""), acc, w, k < 1000))
    return gp, ticks


def test_gns_filter_equals_the_v08_nav_filter_bit_for_bit():
    """The STATE frame transports the force sum, q and w of tick 5 m + 4; the GNS runs STEP m at the frame and the fusion
    as of tick + 1 - the v0.8 NAV's j = 4 / j = 0 order.  With float32-exact inputs both filters agree bit for bit."""
    gp, ticks = _stream(2500)
    q = (1.0, 0.0, 0.0, 0.0)
    v08 = gn.NavGps(gp, configure=False)
    pp = pc.PositionParams()
    gns = gl.GnsNode(gp, pp, configure=False)
    nav = gl.GlinkNav(5, 3)
    sent = None
    f4 = nl.Fcu4(0, lp.STATE_SAFE)
    for k, (ubx, acc, w, pad) in enumerate(ticks):
        v08.tick(k, ubx, acc, q, w, pad)
        gns.poll(k, ubx)
        if sent is not None:
            for fr in gns.rx_bytes(sent):
                nav.rx_bytes(fr, k)
        sent = nav.tick(k, acc, q, w, 0.0, pad, f4, True)
        if k % 5 == 0 and k > 0:                                        # both after the fusion of tick k
            a, b = v08.filter, gns.filter
            assert (a.p, a.v, a.b, a.reset_count, a.yaw_corr, a.p_psi) == (b.p, b.v, b.b, b.reset_count, b.yaw_corr,
                                                                            b.p_psi), k
            assert a.snap == b.snap and a.stats == b.stats
    assert v08.filter.stats["fused"] > 30 and gns.stats["states"] == 499


def _state(k: int, **kw) -> gl.State:
    d = dict(tick=k, fsum=(0.0, 0.0, 5 * 9.75), n=5, nflags=gl.NF_IMU_OK | gl.NF_FCU_OK,
             pflags=nl.fcu4_pflags(lp.PHASE_HOLD, True, False, False, False, True), h=3.0)
    d.update(kw)
    return gl.State(**d)


def test_gns_gap_hold_steps_restart_and_imu_bad():
    gp = gn.NavGpsParams()
    node = gl.GnsNode(gp, pc.PositionParams(), configure=False)
    node.poll(5, b"")
    node.rx_bytes(gl.encode_state(0, _state(4)))
    node.poll(10, b"")
    node.rx_bytes(gl.encode_state(1, _state(9)))
    assert node.stats["hold_steps"] == 0 and node.filter.m_now == 1
    n0 = node.pos.n
    node.poll(20, b"")
    node.rx_bytes(gl.encode_state(3, _state(19)))                       # cycle 2's frame lost
    assert node.stats["gaps"] == 1 and node.stats["hold_steps"] == 1 and node.pos.n == n0 + 2
    assert node.filter.m_now == 3 and node.filter.buf[2 % gn.BUF_N].m != 2       # the lost step's entry stays stale
    node.poll(25, b"")
    node.rx_bytes(gl.encode_state(4, _state(24, nflags=gl.NF_FCU_OK)))   # IMU read failed: a hold step, no propagation
    assert node.stats["imu_bad"] == 1 and node.filter.m_now == 3
    node.poll(30, b"")
    node.rx_bytes(gl.encode_state(5, _state(29)))
    assert node.filter.m_now == 5 and node.stats["gaps"] == 2
    node.poll(10, b"")
    node.rx_bytes(gl.encode_state(6, _state(9)))                        # the NAV reset: its tick went backwards
    assert node.stats["restarts"] == 1 and not node.filter.init and node.clock.offset is None
    node.poll(115, b"")
    node.rx_bytes(gl.encode_state(7, _state(114)))                      # a 0.42 s gap (beyond the 0.32 s buffer)
    assert node.stats["restarts"] == 1 and node.filter.m_now == 22 and node.stats["hold_steps"] == 1 + 1 + 1 + 20
    node.poll(5000, b"")
    node.rx_bytes(gl.encode_state(8, _state(114 + gl.GAP_RESTART_TICKS + 5)))   # longer than 10 s: restart
    assert node.stats["restarts"] == 2


def test_gns_clock_slip_relearns_the_itow_offset():
    gp = gn.NavGpsParams()
    node = gl.GnsNode(gp, pc.PositionParams(), configure=False)
    node.clock.offset = 123
    node.poll(5, b"")
    node.rx_bytes(gl.encode_state(0, _state(4)))
    assert node.clock.offset == 123 and node.stats["clock_slips"] == 0
    node.poll(40, b"")                                                  # the GNS clock ran 30 ticks ahead of the NAV
    node.rx_bytes(gl.encode_state(1, _state(9)))
    assert node.stats["clock_slips"] == 1 and node.clock.offset is None and node.nt == 10


def test_glink_nav_sums_in_float32_ors_sat_and_sends_at_j4():
    nav = gl.GlinkNav(5, 3)
    hi = nl.Fcu4(0, lp.STATE_POWERED, pflags=nl.fcu4_pflags(lp.PHASE_HOLD, True, True, False, False, True), kt_q=17000)
    lo = replace(hi, pflags=hi.pflags & ~nl.PF_SAT_HI)
    out = [nav.tick(k, (0.1, 0.2, 9.8), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 2.5, False,
                    hi if k == 1 else lo, k != 3) for k in range(5)]
    assert out[:4] == [None] * 4
    typ, seq, pl = gl.decode_frame(out[4])
    st = gl.State.unpack(pl)
    f = nl.f32
    s = 0.0
    for _ in range(5):
        s = f(s + f(0.1))
    assert typ == gl.TYPE_STATE and seq == 0 and st.tick == 4 and st.n == 5 and st.fsum[0] == s
    assert st.pflags & nl.PF_SAT_HI and st.kt_q == 17000 and st.h == f(2.5)
    assert st.nflags == gl.NF_FCU_OK | gl.NF_IMU_OK | gl.NF_EST_VALID
    out = [nav.tick(k, (0.0, 0.0, 9.8), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, True, lo, True)
           for k in range(5, 10)]
    st = gl.State.unpack(gl.decode_frame(out[4])[2])
    assert not st.pflags & nl.PF_SAT_HI and st.nflags & gl.NF_PAD and gl.decode_frame(out[4])[1] == 1


# --------------------------------------------------------------------------- #
# the whole path NAV -> GNS -> NAV -> FCU, at its exact ticks
# --------------------------------------------------------------------------- #
def test_end_to_end_tilt_reaches_the_fcu_attitude_input_at_k_plus_L_plus_1():
    """A GNS whose position step returns a known tilt: the STATE of NAV tick k = 5 m + 4 is answered at k + 1, the NAV
    puts the TILT into the frame of k + L, the FCU's slot receiver takes it there and its builder slews toward it at the
    end of that tick: the attitude controller's input (``FcuOutputs.att_tilt``) moves first at FCU tick k + L + 1."""
    for L in (2, 3, 7):
        stack = FcuStack.from_params(P, MASS, T_MAX, mode="vtol", gps=True, chain="gns")
        assert stack.gns and stack.pos is None and isinstance(stack.rx, nl.GnsSlotRx)
        nav = gl.GlinkNav(5, L)
        got = {}
        for k in range(60):
            if k % 5 == 0 and k >= 5:                                  # the RESULT of the STATE of tick k - 1
                m = (k - 1) // 5
                nav.rx_bytes(gl.encode_result(m & 0xFF, replace(R_HOLD, tilt_x=100 * (m + 1))), k)
            stack.step(OnboardInputs(tick=k, gps_slot=nav.slot(k), cos_tilt=1.0, accel_z_mps2=9.8))
            got[k] = stack.rx.target()[1][0]                           # the builder's target at the end of tick k
        for m in range(1, 8):
            k_tilt = 5 * m + 4 + L                                     # sampled at 5 m + 4, in the frame of + L
            assert got[k_tilt] == pytest.approx(0.01 * (m + 1)), (L, m)
            assert got[k_tilt - 1] == pytest.approx(0.01 * m), (L, m)
        assert nav.tx.missed == 0 and stack.rx.sets >= 7 and stack.rx.broken == 0
        # the builder of tick k_tilt -> the attitude controller of the NEXT FCU tick: L + 1 ticks after the sample
        assert fct.gns_chain_ticks("gns", L)["sample_to_attitude"] == L + 1


def test_fcu_stack_gns_builder_and_frame_bits():
    """FcuStack gns mode: the builder slews toward the slot TILT while motor_on (the 6-DOF's closed loop exercises the
    full sequence); the FCU frame bits are this tick's phase / motor_on / sat / low_alt / ground / lifted and k_T."""
    stack = FcuStack.from_params(P, MASS, T_MAX, mode="vtol", gps=True, chain="gns")
    s = _slots(3, R_HOLD)
    fo = stack.step(OnboardInputs(tick=0, gps_slot=s[0], cos_tilt=1.0, accel_z_mps2=9.8))
    pf, kt = fo.fcu4
    assert pf & nl.PF_PHASE_MASK == fo.sup.phase and bool(pf & nl.PF_MOTOR_ON) == fo.sup.motor_on
    assert kt == nl.kt_quant(fo.vc.k_thrust) and bool(pf & nl.PF_LIFTED) == bool(stack.vc.vp.takeoff_ticks <= 0
                                                                                 or fo.vc.takeoff_passed)
    assert fo.tilt_next == (0.0, 0.0)                                  # motor off: inactive
    # the builder itself (pp slew) toward the TILT target when active
    tsp = pc.TiltSetpoint(stack.pp)
    t = tsp.tick((0.03, -0.09), True)
    assert math.hypot(*t) == pytest.approx(stack.pp.slew_per_tick)
    with pytest.raises(ValueError):
        FcuStack.from_params(P, MASS, T_MAX, mode="vtol", gps=True, chain="nav")


# --------------------------------------------------------------------------- #
# vectors
# --------------------------------------------------------------------------- #
def test_vectors_deterministic_consistent_and_write(tmp_path):
    a = gl.make_test_vectors()
    b = gl.make_test_vectors()
    assert json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)
    for s in a["states"]:
        st = gl.State.unpack(bytes.fromhex(s["payload_hex"]))
        assert gl.encode_state(s["seq"], st).hex() == s["frame_hex"]
    for r in a["results"]:
        res = gl.Result.unpack(bytes.fromhex(r["payload_hex"]))
        assert gl.encode_result(r["seq"], res).hex() == r["frame_hex"]
    assert sum(1 for d in a["decodes"] if d["valid"]) == 2
    rep = a["gns_replay"]
    assert len(rep["states"]) > 250 and all(gl.decode_frame(bytes.fromhex(s["result_hex"])) is not None
                                            for s in rep["states"])
    assert any(s["mode"] == pc.MODE_POS for s in rep["states"])
    out = gl.write_test_vectors(str(tmp_path / "glink_vectors.json"))
    assert json.loads((tmp_path / "glink_vectors.json").read_text(encoding="utf-8"))["constants"] == out["constants"]
    gl.write_test_vectors()                                            # the project copy (sim/out/glink_vectors.json)
    assert os.path.exists(gl.VECTORS_PATH)
