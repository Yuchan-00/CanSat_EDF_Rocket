"""Tests of the v0.8 references (GPS position hold): ``sim.gps_nav`` (UBX frames, parser, epoch assembly, iTOW clock,
boot configurator, local frame; the NAV GPS-aided horizontal estimator on synthetic streams), ``sim.position_ctrl`` (the
FCU position hold and the tilt-setpoint builder), ``sim.controller`` (the tilt-setpoint input), ``sim.estimator``
(predict / publish / correct), ``sim.vertical_ctrl`` (take-off check), ``sim.onboard`` (the v0.8 rules at their exact
ticks, the v0.8 FcuStack order, TELEM v3), ``sim.nlink_proto`` v3 slots, and a 3-D point-mass closed loop of the whole
chain written here.

The closed loop is NOT the 6-DOF (``sim.sim6dof`` is the physics reference): the attitude loop is a first-order lag,
the drag linear, the GPS white noise - it checks that the reference pieces work together at the normative tick order
and with the right signs, not flight performance.

Run: ``.venv/Scripts/python.exe -m pytest sim/test_gps_ref.py -q -p no:cacheprovider``.
"""
from __future__ import annotations

import copy
from dataclasses import replace
import json
import math
import os
import random
import struct
import sys

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import numpy as np  # noqa: E402

from sim import gps_nav as gn  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim import nlink_proto as nl  # noqa: E402
from sim import onboard as onb  # noqa: E402
from sim import position_ctrl as pc  # noqa: E402
from sim import quat  # noqa: E402
from sim.estimator import GatedMahonyEstimator  # noqa: E402
from sim.ground_station_logic import REASON_TEXT_KO, GroundAssist, GroundParams, gps_status_ko  # noqa: E402
from sim.onboard import FcuStack, OnboardInputs, OnboardSupervisor, State, SupervisorParams, UplinkFrame  # noqa: E402
from sim.state_machine import G0  # noqa: E402
from sim import vertical_ctrl as vtc  # noqa: E402
from sim.vertical_ctrl import VerticalController, VerticalInputs, VerticalParams  # noqa: E402

with open(os.path.join(_ROOT, "params.json"), encoding="utf-8") as _f:
    P = json.load(_f)
HZ = int(P["control"]["loop_hz"])
DT = 1.0 / HZ
T_MAX = float(P["edf"]["thrust_nominal_N"])
MASS = T_MAX / (1.177 * G0)                     # the prototype's model T/W with the GPS pod (1.177)
GP = gn.NavGpsParams.from_params(P)
PP = pc.PositionParams.from_params(P, MASS, T_MAX)
SP8 = SupervisorParams.from_params(P, mode=lp.MODE_VTOL)
VP8 = VerticalParams.from_params(P, MASS, T_MAX)
# v0.9: the hand-worked take-off-check / FcuStack tests below keep the v0.8.1 take-off law (0 -> u_max in ramp_ticks, no
# thrust search); params.json's mission.vtol.takeoff_v09 has its own tests (sim/test_vtol_ref.py test_v09_*)
VP8 = replace(VP8, to_u_start=0.0, to_slew_per_s=0.0, to_boost_per_s=0.0, u_slew_per_s=0.0,
              to_learn_tau_s=0.0, to_ref_hold=False, td_release_per_s=0.0, ref_approach_shift=0,
              land_ref_rebase=False)   # v0.9 fix round: the hand-worked laws keep the v0.8.1 vertical controller
S_, A_, P_, U_, R_, AB_ = State.SAFE, State.ARMED, State.POWERED, State.UNPOWERED, State.RELEASED, State.ABORT
NONE, SPOOL, CLIMB, HOLD, LAND, LANDED = (lp.PHASE_NONE, lp.PHASE_SPOOL, lp.PHASE_CLIMB, lp.PHASE_HOLD, lp.PHASE_LAND,
                                          lp.PHASE_LANDED)
LF = gn.LocalFrame.at(375665432, 1269780123)      # an origin (Seoul, ~37.57 N): any value works


# =========================================================================== #
# Part 1 - UBX
# =========================================================================== #
def test_ubx_frames_match_the_published_u_center_bytes():
    """CFG frames the NAV sends at boot, hand-checked against the bytes u-center prints for the same settings."""
    assert gn.cfg_rate(200).hex() == "b56206080600c800010001" + "00de6a"
    assert gn.cfg_msg(gn.CLS_NAV, gn.ID_NAV_POSLLH).hex() == "b562060103000102010e47"
    assert gn.cfg_msg(gn.CLS_NAV, gn.ID_NAV_VELNED).hex() == "b562060103000112011e67"
    assert gn.cfg_msg(gn.CLS_NAV, gn.ID_NAV_SOL).hex() == "b562060103000106011" + "24f"
    assert gn.mon_ver_poll().hex() == "b5620a0400000e34"
    assert gn.cfg_prt_uart1(38400).hex() == "b56206001400010000" + "00d0080000009600000300010000000000" + "8d64"
    nav5 = gn.cfg_nav5_airborne()
    assert nav5[:10].hex() == "b5620624240001000600" and len(nav5) == 8 + 36 and nav5[10:-2] == bytes(32)
    assert gn.ubx_checksum(nav5[2:-2]) == (nav5[-2], nav5[-1])
    assert [n for n, _ in gn.config_sequence(5.0)] == ["CFG-RATE", "CFG-MSG POSLLH", "CFG-MSG VELNED", "CFG-MSG SOL",
                                                       "CFG-NAV5"]
    # every message length is its data-sheet length
    assert (gn.LEN_POSLLH, gn.LEN_VELNED, gn.LEN_SOL, gn.LEN_CFG_PRT, gn.LEN_CFG_NAV5) == (28, 36, 52, 20, 36)


def _msgs():
    pos = gn.NavPosllh(345600200, 1269780123, 375665432, 45678, 41234, 2500, 3900)
    vel = gn.NavVelned(345600200, 12, -34, 5, 37, 36, 28743210, 30, 4500000)
    sol = gn.NavSol(345600200, -12345, 2400, 3, 0x0D, 312345678, 98765432, 345678901, 350, 12, -34, 5, 30, 150, 0, 9, 0)
    return pos, vel, sol


def test_ubx_messages_pack_unpack_and_decode():
    pos, vel, sol = _msgs()
    for m, cid in ((pos, gn.ID_NAV_POSLLH), (vel, gn.ID_NAV_VELNED), (sol, gn.ID_NAV_SOL)):
        fr = gn.ubx_frame(gn.CLS_NAV, cid, m.pack())
        (um,) = gn.UbxParser().feed(fr)
        assert gn.decode_msg(um) == m and um.length == len(m.pack())
    # field order of POSLLH: lon before lat (u-blox), little-endian
    assert struct.unpack("<IiiiiII", pos.pack())[1:3] == (1269780123, 375665432)
    assert sol.fix_ok and not gn.NavSol(1, flags=0x0C).fix_ok
    # wrong lengths never decode, an ACK decodes, MON-VER decodes from its first 40 bytes
    assert gn.decode_msg(gn.UbxMsg(gn.CLS_NAV, gn.ID_NAV_SOL, 51, bytes(51))) is None
    assert gn.decode_msg(gn.UbxMsg(gn.CLS_ACK, gn.ID_ACK_ACK, 2, bytes([6, 8]))) == ("ACK", 6, 8)
    assert gn.decode_msg(gn.UbxMsg(gn.CLS_ACK, gn.ID_ACK_NAK, 2, bytes([6, 0x24]))) == ("NAK", 6, 0x24)
    mv = gn.decode_msg(gn.UbxParser().feed(gn.ubx_frame(gn.CLS_MON, gn.ID_MON_VER,
                                                        gn.mon_ver_payload("7.03 (45969)", gn.HW_UBLOX6)))[0])
    assert (mv.sw, mv.hw, mv.module) == ("7.03 (45969)", "00040007", 6)
    assert gn.MonVer("x", "00080000").module == 8 and gn.MonVer("x", "12345678").module == 0
    assert gn.MonVer("x", "00070000").module == gn.MODULE_UBLOX7 == 7          # NEO-7M (2026-11-06, report only)
    assert gn.MonVer("x", "00070001").module == 0


def test_ubx_parser_good_bad_checksum_partial_and_resync():
    pos, vel, sol = _msgs()
    f_pos = gn.ubx_frame(gn.CLS_NAV, gn.ID_NAV_POSLLH, pos.pack())
    f_vel = gn.ubx_frame(gn.CLS_NAV, gn.ID_NAV_VELNED, vel.pack())
    f_ack = gn.ubx_frame(gn.CLS_ACK, gn.ID_ACK_ACK, bytes([6, 0]))
    # good, back to back, behind NMEA text
    ps = gn.UbxParser()
    out = ps.feed(b"$GPGGA,0*47\r\n" + f_pos + f_vel + f_ack)
    assert [(m.cls, m.id) for m in out] == [(1, 2), (1, 0x12), (5, 1)] and ps.stats["frames"] == 3
    assert ps.stats["skipped"] == len(b"$GPGGA,0*47\r\n") and ps.stats["bad_ck"] == 0
    # every single bit flip in the payload or checksum of a frame: that frame refused, the next one found
    for j in range(6, len(f_vel)):
        for bit in range(8):
            bad = bytearray(f_vel)
            bad[j] ^= 1 << bit
            ps = gn.UbxParser()
            out = ps.feed(bytes(bad) + f_ack + f_ack)
            assert out[-1].cls == gn.CLS_ACK and ps.stats["bad_ck"] == 1, (j, bit)
            assert not any(m.cls == gn.CLS_NAV for m in out)
    # a flipped length byte: either bad_len (> 256) or a checksum failure later - never a NAV message
    for bit in range(8):
        bad = bytearray(f_vel)
        bad[5] ^= 1 << bit
        out = gn.UbxParser().feed(bytes(bad) + f_ack + f_ack + f_ack)
        assert not any(m.cls == gn.CLS_NAV for m in out) and out and out[-1].cls == gn.CLS_ACK
    # partial frame: nothing, the state is kept, the rest completes it
    ps = gn.UbxParser()
    assert ps.feed(f_pos[:20]) == [] and ps.state == 6
    assert [m.id for m in ps.feed(f_pos[20:])] == [gn.ID_NAV_POSLLH]
    # B5 B5 62: the second B5 starts the frame
    assert len(gn.UbxParser().feed(b"\xb5" + f_ack)) == 1
    # length limit: 0x0210 = 528 -> bad_len, resync to SYNC1, the next frame found
    bad = bytearray(f_pos)
    bad[4], bad[5] = 0x10, 0x02
    ps = gn.UbxParser()
    out = ps.feed(bytes(bad) + f_ack)
    assert ps.stats["bad_len"] == 1 and out[-1].cls == gn.CLS_ACK
    # truncated MON-VER (160 B: sw 30, hw 10, rom 30, 3 x 30 extensions): stored 64, delivered with truncated, decoded
    mv = gn.ubx_frame(gn.CLS_MON, gn.ID_MON_VER, gn.mon_ver_payload("ROM CORE 3.01", gn.HW_UBLOX8, ["A", "B", "C"]))
    ps = gn.UbxParser()
    (m,) = ps.feed(mv)
    assert m.truncated and len(m.payload) == gn.UBX_MAX_PAYLOAD and m.length == 160 and ps.stats["truncated"] == 1
    assert gn.decode_msg(m).module == 8
    # byte-wise == block-wise, counters wrap at 16 bits
    data = (f_pos + b"\x00\xb5\x62\x01" + f_vel) * 3
    a, b = gn.UbxParser(), gn.UbxParser()
    assert a.feed(data) == [x for c in data for x in [b.feed_byte(c)] if x is not None] and a.stats == b.stats
    ps = gn.UbxParser()
    ps.stats["skipped"] = 0xFFFF
    ps.feed(b"\x00")
    assert ps.stats["skipped"] == 0


def test_epoch_assembler_matches_by_itow():
    pos, vel, sol = _msgs()
    a = gn.EpochAssembler()
    assert a.add(pos, 10) is None and a.add(sol, 11) is None
    ep = a.add(vel, 13)
    assert ep is not None and ep.tick == 13 and ep.itow_ms == 345600200 and ep.num_sv == 9 and ep.fix_ok
    assert (ep.lat_e7, ep.lon_e7, ep.vel_n_cms, ep.vel_e_cms, ep.hacc_mm, ep.sacc_cms) == (375665432, 1269780123, 12,
                                                                                         -34, 2500, 30)
    # a missing message: the open epoch is dropped when the next iTOW starts
    a = gn.EpochAssembler()
    from dataclasses import replace
    a.add(pos, 1)
    a.add(vel, 2)
    assert a.add(replace(pos, itow_ms=345600400), 50) is None and a.stats["incomplete"] == 1
    assert a.add(replace(sol, itow_ms=345600400), 51) is None
    assert a.add(replace(vel, itow_ms=345600400), 52).itow_ms == 345600400
    # any order; duplicates counted
    a = gn.EpochAssembler()
    a.add(vel, 1)
    a.add(vel, 2)
    a.add(sol, 3)
    assert a.add(pos, 4) is not None and a.stats == {"epochs": 1, "incomplete": 0, "duplicate": 1}
    assert a.add(("ACK", 6, 0), 5) is None


def test_itow_clock_learns_the_earliest_arrival_with_a_leak():
    c = gn.ItowClock(4, 220)
    # first epoch: offset = its own delay; t_e = arrival - 220 ms
    assert c.update(345600000, 1000) == 4000 - 220 and c.offset == 4000 - 345600000
    # a later (jittered) arrival does not move the offset by more than the 1 ms leak
    o0 = c.offset
    assert c.update(345600200, 1052) == (345600200 + o0 + 1 - 220) & 0xFFFFFFFF and c.offset == o0 + 1
    # an earlier arrival takes over at once
    c.update(345600400, 1099)
    assert c.offset == 4 * 1099 - 345600400 < o0
    # a jump (week roll-over / receiver reset) re-learns
    c.update(1000, 2000)
    assert c.offset == 8000 - 1000 and c.relearned == 1
    # wrap: the NAV ms clock and iTOW differences are int32
    c = gn.ItowClock(4, 220)
    te = c.update(604799800, 5)
    assert te == (604799800 + (20 - 604799800) - 220) & 0xFFFFFFFF


def _run_configurator(stub: gn.UbxReceiverStub, max_ticks: int = 2000) -> tuple[gn.GpsConfigurator, int, list]:
    cfg = gn.GpsConfigurator(HZ, 5.0)
    parser = gn.UbxParser()
    baud = None
    inbox = b""
    trace = []
    for k in range(max_ticks):
        msgs = [d for d in (gn.decode_msg(m) for m in parser.feed(inbox)) if d is not None]
        inbox = b""
        for a in cfg.tick(msgs):
            trace.append((k, a[0], a[1] if a[0] == "baud" else bytes(a[1][2:4]).hex()))
            if a[0] == "baud":
                baud = a[1]
            else:
                inbox += stub.receive(a[1], baud)
        if cfg.done:
            return cfg, k, trace
    return cfg, max_ticks, trace


def test_configurator_factory_receiver_backup_receiver_nak_and_silence():
    # factory default 9600: probe at 38400 times out, the 9600 path, everything acknowledged, MON-VER read
    cfg, k, tr = _run_configurator(gn.UbxReceiverStub(baud=9600))
    assert cfg.done and cfg.cfg_ok and cfg.prt_acked and cfg.ver.module == 6 and cfg.timeouts == 1 and cfg.naks == 0
    assert tr[0] == (cfg.n_wait - 1, "baud", 38400) and ("baud", 9600) in [(t[1], t[2]) for t in tr]
    assert [t[2] for t in tr if t[1] == "tx"] == ["0600", "0600", "0600", "0608", "0601", "0601", "0601", "0624", "0a04"]
    # kept configured by its backup cell (already at 38400): the probe is acknowledged, no 9600 at all
    cfg2, k2, tr2 = _run_configurator(gn.UbxReceiverStub(baud=38400))
    assert cfg2.cfg_ok and cfg2.timeouts == 0 and all(t[2] != 9600 for t in tr2 if t[1] == "baud") and k2 < k
    # CFG-NAV5 refused: three tries, then the rest; cfg_ok false (the flag the NAV reports)
    cfg3, _, tr3 = _run_configurator(gn.UbxReceiverStub(baud=9600, nak=[gn.ID_CFG_NAV5]))
    assert not cfg3.cfg_ok and cfg3.naks == gn.CFG_TRIES and [t[2] for t in tr3 if t[1] == "tx"].count("0624") == 3
    assert cfg3.ver is not None and cfg3.done
    # a silent receiver: finishes (never blocks), nothing acknowledged, no version
    cfg4, k4, _ = _run_configurator(gn.UbxReceiverStub(silent=True))
    assert cfg4.done and not cfg4.cfg_ok and cfg4.ver is None and not cfg4.prt_acked and k4 < 2000


def test_local_frame_north_west_and_inverse():
    k_n, k_e = LF.k_n, LF.k_e
    assert k_n == pytest.approx(0.011092, rel=1e-3)                    # 1e-7 deg of latitude ~ 1.11 cm
    phi = math.radians(37.5665432)
    w = 1.0 - gn.WGS84_E2 * math.sin(phi) ** 2
    assert k_n == pytest.approx(math.radians(1e-7) * gn.WGS84_A * (1 - gn.WGS84_E2) / w ** 1.5, rel=1e-12)
    assert k_e == pytest.approx(math.radians(1e-7) * gn.WGS84_A / math.sqrt(w) * math.cos(phi), rel=1e-12)
    assert k_e / (k_n * math.cos(phi)) == pytest.approx(1.0042, abs=5e-4)            # N / M at 37.6 deg
    assert LF.to_w(LF.lat0_e7 + 1000, LF.lon0_e7) == pytest.approx((1000 * k_n, 0.0))        # north = +x
    assert LF.to_w(LF.lat0_e7, LF.lon0_e7 + 1000) == pytest.approx((0.0, -1000 * k_e))       # east = -y (west +y)
    for x, y in ((0.0, 0.0), (12.3, -4.5), (-150.0, 220.0)):
        la, lo = LF.to_llh(x, y)
        assert LF.to_w(la, lo) == pytest.approx((x, y), abs=0.006)
    # the antimeridian: longitudes differences wrap
    lf = gn.LocalFrame.at(0, 1799999990)
    assert lf.to_w(0, -1799999990)[1] == pytest.approx(-20 * lf.k_e)


def test_gps_vectors_deterministic_consistent_and_written():
    v = gn.make_test_vectors()
    assert gn.make_test_vectors() == v
    for m in v["messages"]:
        (um,) = gn.UbxParser().feed(bytes.fromhex(m["hex"]))
        assert um.payload.hex() == m["payload_hex"]
    for s in v["streams"]:
        ps = gn.UbxParser()
        out = ps.feed(bytes.fromhex(s["hex"]))
        assert [o.payload.hex() for o in out] == [e["payload_hex"] for e in s["expected"]] and ps.stats == s["stats"]
    names = {s["name"] for s in v["streams"]}
    assert {"epoch_psv", "b5_b5_62", "bad_length", "truncated_mon_ver", "partial_end", "random_noise"} <= names
    assert any(s["stats"]["bad_ck"] for s in v["streams"])
    assert [c["epochs"] for c in v["assembler"] if c["name"] == "missing_sol"][0][0]["itow_ms"] == 1200
    assert v["config_frames"][2]["hex"] == gn.cfg_rate(200).hex()
    written = gn.write_test_vectors()
    with open(gn.VECTORS_PATH, encoding="utf-8") as f:
        assert json.load(f) == written == v


# =========================================================================== #
# Part 2 - the NAV GPS filter on synthetic streams
# =========================================================================== #
def test_fixed_gains_steady_state_and_acquisition():
    g = GP.gains()
    kpv = g.steady.k_pv
    # the prototype model: GPS position followed slowly (~20 s), the velocity strongly, b with ~7 s
    assert kpv[0][0] == pytest.approx(0.00998, rel=1e-2) and kpv[1][1] == pytest.approx(0.3597, rel=1e-3)
    assert kpv[2][1] == pytest.approx(-0.0477, rel=1e-2)
    T = 1.0 / GP.rate_hz

    def eig_taus(K):
        F = np.array([[1, T, -T * T / 2], [0, 1, -T], [0, 0, 1.0]])
        H = np.array([[1, 0, 0], [0, 1, 0.0]])
        A = (np.eye(3) - np.array(K) @ H) @ F
        return sorted(-T / np.log(np.abs(np.linalg.eigvals(A))))

    ts, ta = eig_taus(kpv), eig_taus(g.acq.k_pv)
    assert max(abs(x) for x in np.linalg.eigvals((np.eye(3) - np.array(kpv) @ np.array([[1, 0, 0], [0, 1, 0]])) @
                                                np.array([[1, T, -T * T / 2], [0, 1, -T], [0, 0, 1]]))) < 1.0
    assert ts[1] == pytest.approx(7.05, rel=0.05) and ta[1] < 0.4 * ts[1]       # the acquisition set learns b faster
    assert g.s_vv == pytest.approx(0.0351, rel=1e-2)                             # gate 5 sigma = 0.94 m/s
    assert GP.n_corr == 3 and GP.fresh_ticks == 150 and GP.gate_stable_ticks == 1250 and GP.step_ticks == 5
    assert GP.lever_b_m == pytest.approx((-0.0743, -0.0151, 0.3122))


class Synth:
    """A kinematic truth (no controller) feeding ``NavGpsFilter`` exactly as the NAV does: the body specific force of
    each tick (level attitude unless ``tilt_fn``), the corrected attitude, the rate, and epochs (truth at the epoch +
    noise) completed ``latency`` after the epoch, stamped with ``ItowClock``."""

    def __init__(self, gp: gn.NavGpsParams = GP, seed: int = 3, latency: float = 0.22, sp: float = 0.0,
                 sv: float = 0.0, acc_bias=(0.0, 0.0), psi_true: float = 0.0, lever_true=None, tilt_fn=None):
        self.f = gn.NavGpsFilter(gp)
        self.clock = gn.ItowClock(gp.ms_per_tick, gp.latency_ms)
        self.rng = random.Random(seed)
        self.lat_ticks = int(round(latency * HZ))
        self.sp, self.sv = sp, sv
        self.acc_bias = acc_bias
        self.psi = psi_true                     # true heading of the level frame in W
        self.lever = gp.lever_b_m if lever_true is None else lever_true
        self.tilt_fn = tilt_fn
        self.p = [0.0, 0.0]
        self.v = [0.0, 0.0]
        self.k = 0
        self.queue: list = []
        self.outage = None
        self.jump = None
        self.err_v: list = []
        self.err_p: list = []

    def q_at(self, k):
        if self.tilt_fn is None:
            return (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        return self.tilt_fn(k * DT)

    def step(self, a_w=(0.0, 0.0), on_pad=False):
        k = self.k
        q, w = self.q_at(k)
        R = quat.to_matrix(np.array(q))
        c, s = math.cos(self.psi), math.sin(self.psi)
        a_l = (c * a_w[0] + s * a_w[1], -s * a_w[0] + c * a_w[1])
        f_l = np.array([a_l[0] + self.acc_bias[0], a_l[1] + self.acc_bias[1], G0])
        f_b = R.T @ f_l
        if k % 50 == 0:                         # an epoch every 0.2 s (iTOW = 4 k ms)
            r = R @ np.array(self.lever)
            wl = R @ np.array(w)
            lv = np.cross(wl, r)
            rx, ry = c * r[0] - s * r[1], s * r[0] + c * r[1]
            lvx, lvy = c * lv[0] - s * lv[1], s * lv[0] + c * lv[1]
            out = self.outage is not None and self.outage[0] <= k * DT < self.outage[1]
            jx, jy = self.jump[2:] if (self.jump is not None and self.jump[0] <= k * DT < self.jump[1]) else (0, 0)
            x = self.p[0] + rx + jx + self.rng.gauss(0, self.sp)
            y = self.p[1] + ry + jy + self.rng.gauss(0, self.sp)
            la, lo = LF.to_llh(x, y)
            vn = lp.round_half_away((self.v[0] + lvx + self.rng.gauss(0, self.sv)) * 100)
            ve = lp.round_half_away(-(self.v[1] + lvy + self.rng.gauss(0, self.sv)) * 100)
            ep = gn.GpsEpoch(345600000 + 4 * k, la, lo, 0, 2500, 3900, vn, ve, 0, 30, 0 if out else 3, not out,
                             0 if out else 9, 350)
            self.queue.append((k + self.lat_ticks, ep))
        done = [(t, e) for t, e in self.queue if t <= k]
        self.queue = [(t, e) for t, e in self.queue if t > k]
        eps = [(gn.replace(e, tick=k), self.clock.update(e.itow_ms, k)) for _, e in done]
        snap = self.f.tick(k, f_b, q, w, on_pad, eps)
        for i in range(2):                      # truth integration after the NAV tick (the sample was at tick k)
            self.p[i] += self.v[i] * DT + 0.5 * a_w[i] * DT * DT
            self.v[i] += a_w[i] * DT
        if k % 5 == 4 and self.f.init and not on_pad:
            self.err_v.append(math.hypot(self.f.v[0] - self.v[0], self.f.v[1] - self.v[1]))
            self.err_p.append(math.hypot(self.f.p[0] - self.p[0], self.f.p[1] - self.p[1]))
        self.k += 1
        return snap

    def run(self, seconds, a_fn=None, on_pad=False):
        for _ in range(int(round(seconds * HZ))):
            t = self.k * DT
            self.step((0.0, 0.0) if a_fn is None else a_fn(t), on_pad)


def test_filter_pad_init_lever_and_on_gate_exact_timing():
    s = Synth()
    s.run(0.5, on_pad=True)
    f = s.f
    assert f.init and f.frame is not None
    # first fix: the antenna sits at the origin, the CG at minus the lever arm (level attitude)
    assert f.p == pytest.approx([-GP.lever_b_m[0], -GP.lever_b_m[1]], abs=1e-9)
    # GATE_OK exactly gate_stable_ticks after the first good epoch's fusion, evaluated at the latches (k % 5 == 4)
    k_first = f.good_since
    while not s.snap_gate():
        s.step(on_pad=True)
    k_gate = s.k - 1
    assert k_gate - k_first >= GP.gate_stable_ticks and k_gate - k_first < GP.gate_stable_ticks + GP.step_ticks
    assert k_gate % 5 == 4
    # one bad epoch (4 satellites) restarts the stability time
    s.f._fuse(s.k, gn.GpsEpoch(0, LF.lat0_e7, LF.lon0_e7, 0, 2500, 0, 0, 0, 0, 30, 3, True, 4, 350), 0)
    assert s.f.good_since is None and not s.f.gate_ok(s.k)


def _snap_gate(self):
    return bool(self.f.snap.flags & gn.GF_GATE_OK)


Synth.snap_gate = _snap_gate


def test_filter_converges_on_velocity_and_learns_the_acceleration_bias():
    s = Synth(sp=0.3, sv=0.1, acc_bias=(0.12, -0.08), seed=5)
    s.run(1.0, on_pad=True)

    def a_fn(t):                                # manoeuvres: 0.3 m/s^2 sinusoids
        return (0.3 * math.sin(0.7 * t), 0.3 * math.cos(0.45 * t))

    s.run(25.0, a_fn)
    f = s.f
    # b = the measured-acceleration error: the bias in W (heading 0)
    assert f.b == pytest.approx([0.12, -0.08], abs=0.03)
    ev = s.err_v[-500:]
    assert math.sqrt(sum(e * e for e in ev) / len(ev)) < 0.08
    assert f.stats["rej_v"] == 0 and f.stats["pos_resets"] == 0 and f.stats["vel_resets"] == 0
    assert s.f.snap.flags & gn.GF_FIX_OK


def test_latency_compensation_by_the_delayed_state():
    """True latency 0.22 s.  The filter with the delayed-state buffer (latency 0.22) vs one that believes the epochs
    are current (latency 0): velocity error RMS while the vehicle accelerates."""
    def run(lat):
        gp = gn.NavGpsParams.from_params(P, latency_s=lat)
        s = Synth(gp=gp, latency=0.22, sp=0.0, sv=0.0, seed=1)
        s.run(1.0, on_pad=True)
        s.run(12.0, lambda t: (1.0 * math.sin(1.5 * t), 0.0))
        e = s.err_v[-300:]
        return math.sqrt(sum(x * x for x in e) / len(e))

    comp, raw = run(0.22), run(0.0)
    assert comp < 0.02 and raw > 3 * comp


def test_outage_fix_ok_drops_at_the_exact_latch_and_recovers():
    s = Synth(sp=0.2, sv=0.05)
    s.run(1.0, on_pad=True)
    s.run(3.0)
    s.outage = (s.k * DT, s.k * DT + 3.0)
    k_used = None
    while s.f.snap.flags & gn.GF_FIX_OK:
        k_used = s.f.k_last_used
        s.step()
    k_drop = s.k - 1
    # the first latch with k - k_last_used > fresh_ticks
    assert k_drop % 5 == 4 and k_drop - k_used > GP.fresh_ticks and k_drop - k_used <= GP.fresh_ticks + 5
    assert not s.f.snap.flags & gn.GF_GATE_OK
    s.run(3.5)
    assert s.f.snap.flags & gn.GF_FIX_OK
    # dead reckoning on a static vehicle: the position barely moved
    assert math.hypot(s.f.p[0] - s.p[0], s.f.p[1] - s.p[1]) < 0.5


def test_multipath_jump_is_rejected_then_the_position_resets_into_the_offset():
    s = Synth(sp=0.1, sv=0.05)
    s.run(1.0, on_pad=True)
    s.run(5.0)
    t0 = s.k * DT
    s.jump = (t0, t0 + 10.0, 5.0, 0.0)
    p_rf0 = s.f.p[0] - s.f.off[0]
    s.run(1.4)
    assert s.f.stats["rej_p"] >= 5 and s.f.reset_count == 0              # rejected, not yet reset
    s.run(1.0)
    f = s.f
    assert f.reset_count == 1 and f.stats["pos_resets"] == 1
    assert f.off[0] == pytest.approx(5.0, abs=0.3) and abs(f.off[1]) < 0.3
    assert f.p[0] - f.off[0] == pytest.approx(p_rf0, abs=0.3)            # the reset-free position did not jump
    assert f.snap.reset_count == 1 and f.snap.off_x_cm == gn.quant_cm(f.off[0])
    s.run(12.0)                                                         # the jump ends: a second reset back
    assert f.reset_count == 2 and abs(f.off[0]) < 0.4


def test_velocity_reset_after_five_rejected_epochs():
    """A 90 deg heading error with strong accelerations: the velocity innovations are gated, five in a row reset v to
    the GPS (counted, not in the offset)."""
    gp = gn.NavGpsParams.from_params(P, pad_azimuth_deg=-90.0, yaw_correction=False)
    s = Synth(gp=gp, psi_true=0.0)
    s.run(1.0, on_pad=True)
    s.run(8.0, lambda t: (3.0 if (t % 4.0) < 2.0 else -3.0, 0.0))
    assert s.f.stats["vel_resets"] >= 1 and s.f.reset_count >= 1 and s.f.off == [0.0, 0.0]


def test_lever_arm_compensation_on_a_rocking_static_vehicle():
    """CG at rest, the attitude rocking 10 deg at 1 Hz: the antenna moves ~5 cm and 0.35 m/s - compensated, the CG
    velocity estimate stays ~0; without it the filter believes the vehicle moves."""
    def tilt(t):
        a = math.radians(10.0) * math.sin(2 * math.pi * t)
        w = math.radians(10.0) * 2 * math.pi * math.cos(2 * math.pi * t)
        return (math.cos(a / 2), math.sin(a / 2), 0.0, 0.0), (w, 0.0, 0.0)

    def run(comp):
        gp = gn.NavGpsParams.from_params(P, lever_comp=comp)
        s = Synth(gp=gp, tilt_fn=tilt, lever_true=GP.lever_b_m)
        s.run(1.0, on_pad=True)
        s.run(8.0)
        e = s.err_v[-200:]
        return math.sqrt(sum(x * x for x in e) / len(e))

    with_c, without = run(True), run(False)
    assert with_c < 0.02 and without > 4 * with_c


def test_yaw_correction_information_weighted():
    """Pad heading entered 30 deg wrong: manoeuvres make it observable, the 1-state update shrinks the error and the
    confidence flag comes up; in a steady hover nothing changes (not observable)."""
    gp = gn.NavGpsParams.from_params(P, pad_azimuth_deg=-30.0)       # NAV believes psi = +30 deg, truth 0
    s = Synth(gp=gp, sv=0.05, seed=7)
    s.run(1.0, on_pad=True)
    s.run(10.0)                                                         # hover: no information
    assert s.f.yaw_used == 0 and math.degrees(s.f.psi) == pytest.approx(30.0)
    assert not s.f.head_ok()
    s.run(30.0, lambda t: (1.0 * math.sin(1.1 * t), 1.0 * math.cos(0.8 * t)))
    err = math.degrees(math.atan2(math.sin(s.f.psi), math.cos(s.f.psi)))
    assert abs(err) < 5.0 and s.f.yaw_used >= 10 and s.f.head_ok() and s.f.snap.flags & gn.GF_HEAD_OK
    # the information weighting: P shrinks with every accepted window - to the level the random walk Q (1.8 deg^2/s)
    # and the angle-noise floor YAW_SIGMA_REL (0.25 rad) allow with a window per second (~5 deg; physics-review fix -
    # with the old Q 0.04 deg^2/s and no floor it went below 3 deg and HEAD_OK was overconfident in the 6-DOF)
    assert math.radians(3.0) ** 2 < s.f.p_psi < math.radians(gn.HEAD_OK_DEG) ** 2


def test_yaw_variance_grows_on_unused_windows():
    """Physics-review fix (medium 6): every closed yaw window adds Q x its length to P_psi, used or not - a calm hover
    (no information) makes the heading LESS certain with time and HEAD_OK drops; before, P grew only on used windows."""
    gp = gn.NavGpsParams.from_params(P, pad_azimuth_deg=-30.0)
    s = Synth(gp=gp, sv=0.05, seed=7)
    s.run(1.0, on_pad=True)
    s.run(30.0, lambda t: (1.0 * math.sin(1.1 * t), 1.0 * math.cos(0.8 * t)))
    assert s.f.head_ok()
    s.run(2.0)                                                          # the window spanning the last manoeuvre closes
    p0, used0 = s.f.p_psi, s.f.yaw_used
    s.run(60.0)                                                         # steady hover: windows close, none used
    assert s.f.yaw_used == used0
    grown = s.f.p_psi - p0
    assert grown == pytest.approx(math.radians(1.0) ** 2 * gp.yaw_q_deg2_per_s * 60.0, rel=0.05)
    assert not s.f.head_ok() and not s.f.snap.flags & gn.GF_HEAD_OK


def test_gps_imu_inconsistency_flag():
    """Physics-review fix (HIGH 2): GF_INCONS = the GPS velocity rejected in >= 3 of the last 5 flight epochs; a
    consistent GPS (wind saturation keeps the innovations inside the gate) never sets it; cleared on the pad."""
    f = gn.NavGpsFilter(GP)
    assert gn.INCONS_EPOCHS == 5 and gn.INCONS_MIN_REJ == 3
    for hist, want in ((0b00000, False), (0b00011, False), (0b00111, True), (0b10101, True), (0b11000, False),
                       (0b110000, False), (0b11111, True)):
        f.rej_v_hist = hist
        assert f.incons() == want, bin(hist)
    s = Synth(gp=GP, sv=0.05, seed=3)
    s.run(1.0, on_pad=True)
    s.run(20.0, lambda t: (1.0 * math.sin(1.1 * t), 1.0 * math.cos(0.8 * t)))
    assert not s.f.incons() and not s.f.snap.flags & gn.GF_INCONS
    s.f.rej_v_hist = 0b111
    s.run(0.1, on_pad=True)
    assert s.f.rej_v_hist == 0 and not s.f.snap.flags & gn.GF_INCONS


def test_epoch_older_than_the_buffer_is_dropped_and_two_in_a_cycle_keep_the_newest():
    f = gn.NavGpsFilter(GP)
    q, w = (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    ep = gn.GpsEpoch(0, LF.lat0_e7, LF.lon0_e7, 0, 2500, 0, 0, 0, 0, 30, 3, True, 9, 350)
    for k in range(10):
        f.tick(k, (0, 0, G0), q, w, False, [(ep, 4 * k)] if k == 0 else ())
    assert f.init
    for k in range(10, 200):
        f.tick(k, (0, 0, G0), q, w, False, ())
    f.tick(200, (0, 0, G0), q, w, False, [(ep, 4 * 199 - 400)])          # 400 ms old: > BUF_N steps
    assert f.stats["dropped_old"] == 1
    f.tick(205, (0, 0, G0), q, w, False, [(ep, 4 * 200 - 220), (ep, 4 * 204 - 220)])
    assert f.stats["n_epoch_dropped"] == 1


# =========================================================================== #
# nlink v3 slots with the filter's snapshot
# =========================================================================== #
def test_slots_carry_the_latched_snapshot_and_the_fcu_assembles_one_cycle():
    snap = gn.GpsSnapshot(flags=gn.GF_FIX_OK | gn.GF_GATE_OK, num_sv=8, hacc_dm=31, reset_count=2, off_x_cm=-12,
                          off_y_cm=300, px_cm=-32768, py_cm=32767, vx_cms=-5, vy_cms=7, cos_q15=-32767, sin_q15=1)
    tx = nl.GpsSlotTx()
    assert tx.slot(0) == nl.SLOT_EMPTY                                   # nothing latched yet
    tx.latch(snap, 37)
    slots = [tx.slot(k) for k in range(5, 10)]
    assert [s[0] for s in slots] == [(37 & 0x1F) << 3 | j for j in range(5)]
    rx = nl.GpsSlotRx()
    outs = [rx.feed(s) for s in slots]
    assert outs[:4] == [None] * 4 and rx.got_part4
    g = outs[4]
    assert (g.flags, g.num_sv, g.hacc_dm, g.reset_count, g.cycle) == (3, 8, 31, 2, 5)
    assert g.off == (-0.12, 3.0) and g.p == (-327.68, 327.67) and g.v == (-0.05, 0.07)
    assert g.cos_psi == -1.0 and g.sin_psi == pytest.approx(1 / 32767) and rx.gate_ok
    # a lost frame breaks the cycle (never mixes two snapshots); STALE_TICKS later the gate is gone
    rx2 = nl.GpsSlotRx()
    assert [rx2.feed(s) for s in slots[:2] + [None] + slots[3:]] == [None] * 5 and rx2.broken == 1
    rx3 = nl.GpsSlotRx()
    for s in slots:
        rx3.feed(s)
    for _ in range(nl.GpsSlotRx.STALE_TICKS):
        rx3.feed(None)
    assert rx3.gate_ok
    rx3.feed(None)
    assert not rx3.gate_ok


# =========================================================================== #
# Part 3 - position controller
# =========================================================================== #
def _data(p=(0.0, 0.0), v=(0.0, 0.0), off=(0.0, 0.0), flags=gn.GF_FIX_OK, rc=0, psi=0.0):
    return pc.GpsData(flags=flags, num_sv=9, hacc_dm=25, reset_count=rc, off=off, p=p, v=v, cos_psi=math.cos(psi),
                      sin_psi=math.sin(psi))


def _pin(data, phase=HOLD, h=5.0, cos_tilt=1.0, rate=(0.0, 0.0, 0.0), k_t=1.0, sat=False, motor=True, low_alt=None,
         ground=False, lifted=True):
    """PosInputs; ``low_alt`` None = what the vertical controller reports (LAND and h <= flare_altitude_m)."""
    if low_alt is None:
        low_alt = phase == LAND and h == h and h <= PP.flare_altitude_m
    return pc.PosInputs(phase=phase, motor_on=motor, h_m=h, cos_tilt=cos_tilt, rate=rate, k_t=k_t, vc_sat_high=sat,
                        data=data, vc_low_alt=low_alt, vc_ground=ground, lifted=lifted)


def _engaged(pp=PP, hold=(0.0, 0.0)):
    ph = pc.PositionHold(pp)
    o = ph.step(_pin(_data(p=hold)))
    assert o.engaged and o.hold == pytest.approx(hold)
    return ph


def test_tilt_series_and_setpoint_quaternion_accuracy():
    for deg in (0.0, 1.0, 5.0, 10.0, 20.0, 25.0):
        a = G0 * math.tan(math.radians(deg))
        tx, ty = pc.tilt_of_accel(a, 0.0)
        assert tx == 0.0 and ty == pytest.approx(math.radians(deg), rel=4e-3 if deg <= 20 else 1e-2)
        q = pc.tilt_quat(0.0, math.radians(deg))
        qe = quat.from_rotvec(np.array([0.0, math.radians(deg), 0.0]))
        assert np.allclose(q, qe, atol=1e-6)
    # the expanded product == q(t) (x) q_yaw
    for tx, ty, yaw in ((0.1, -0.2, 0.7), (-0.3, 0.05, -2.5), (0.0, 0.0, 1.0)):
        qy = quat.from_yaw(yaw)
        qs = pc.setpoint_quat((tx, ty), qy)
        qe = quat.multiply(quat.from_rotvec(np.array([tx, ty, 0.0])), qy)
        assert np.allclose(qs, qe, atol=2e-6)
    # sign: thrust toward +x needs a rotation about +y
    assert pc.tilt_of_accel(1.0, 0.0)[1] > 0 and pc.tilt_of_accel(0.0, 1.0)[0] < 0


def test_controller_tilt_setpoint_input_and_v071_identity():
    from sim.controller import AttitudeController, MixerPlant
    plant = MixerPlant(0.3, 0.05, 0.002, 3.0, 0.004, T_MAX, 2.0, math.radians(20), 1500.0, 10.0)
    c1 = AttitudeController(P["control"], plant)
    c2 = AttitudeController(P["control"], plant)
    c2.tilt_sp = (0.0, 0.0)
    q = quat.from_euler(0.02, -0.01, 0.3)
    c1.set_yaw_reference(quat.from_yaw(0.3))
    c2.set_yaw_reference(quat.from_yaw(0.3))
    for _ in range(5):
        o1 = c1.update(q, np.array([0.01, -0.02, 0.0]), 0.9)
        o2 = c2.update(q, np.array([0.01, -0.02, 0.0]), 0.9)
        assert np.array_equal(o1.delta_cmd, o2.delta_cmd)               # (0, 0) = the v0.7.1 setpoint bit for bit
    c2.tilt_sp = (0.0, math.radians(5.0))
    qs = c2.setpoint(q)
    assert np.allclose(qs, pc.setpoint_quat(c2.tilt_sp, quat.from_yaw(0.3)), atol=0)
    up = quat.to_matrix(qs) @ np.array([0.0, 0.0, 1.0])
    assert math.degrees(math.acos(up[2])) == pytest.approx(5.0, abs=1e-3) and up[0] > 0


def test_pos_sign_velocity_limit_and_tilt_cap():
    ph = _engaged()
    o = ph.step(_pin(_data(p=(1.0, 0.0))))                               # 1 m north of the hold point
    assert o.v_sp[0] == pytest.approx(-PP.kp_pos) and o.v_sp[1] == 0.0
    assert o.a_cmd[0] < 0 and o.tilt_w[1] < 0 and o.dist_m == pytest.approx(1.0)
    ph = _engaged()
    o = ph.step(_pin(_data(p=(10.0, 0.0), v=(1.0, 0.0))))                # far, moving away: v_sp at v_max, tilt at the cap
    assert math.hypot(*o.v_sp) == pytest.approx(PP.v_max) and o.limited
    th = math.hypot(*o.tilt_w)
    assert th == pytest.approx(math.radians(min(PP.tilt_cap_deg, math.degrees(math.acos(o.cos_lim)))), rel=4e-3)
    # heading: the level-frame target is the W target rotated by -psi
    ph = _engaged()
    o = ph.step(_pin(_data(p=(1.0, 0.0), psi=math.radians(90.0))))
    assert o.tilt_l == pytest.approx((o.tilt_w[1], -o.tilt_w[0]), abs=1e-6)


def test_pos_tilt_limit_margin_rule():
    tw = PP.tw
    c, s, a2 = PP.tilt_limit(1.0)
    x = tw
    m = min(0.1, 0.5 * (x - 1))
    assert c == pytest.approx(max((1 + m) / x, math.cos(math.radians(20.0))))
    assert math.degrees(math.acos(c)) == pytest.approx(min(20.0, math.degrees(math.acos((1 + m) / x))))
    c2, _, _ = PP.tilt_limit(0.93)                                       # weak plant: k_T T/W = 1.095
    x2 = 0.93 * tw
    assert c2 == pytest.approx((1 + 0.5 * (x2 - 1)) / x2) and math.degrees(math.acos(c2)) == pytest.approx(16.9, abs=0.1)
    assert PP.tilt_limit(1.0 / tw)[2] == 0.0 and PP.tilt_limit(0.5)[2] == 0.0
    c3, _, _ = PP.tilt_limit(1.0, low_cap=True)                          # no separate flare cap (physics-review fix)
    assert c3 == c and PP.flare_tilt_cap_deg == PP.tilt_cap_deg
    demo = pc.PositionParams.from_params(P, MASS, T_MAX, profile="demo")
    assert (demo.tilt_cap_deg, demo.fence_m, PP.tilt_cap_deg, PP.fence_m) == (10.0, 5.0, 20.0, 15.0)
    assert demo.flare_tilt_cap_deg == demo.tilt_cap_deg
    assert "flare_tilt_cap_deg_ASSUMED" not in P["mission"]["vtol"]["position_hold"]   # removed by the architect


def test_pos_params_refuse_a_flare_cap_below_the_profile_cap_and_a_zero_fade():
    """Physics-review fixes: HIGH 1 (a flare cap below the profile cap saturates the loop under the trim needed inside
    the wind limits) and low (tilt_fade_m = 0 divided by zero at touchdown; NaN in float32 C bypassed the limit)."""
    with pytest.raises(ValueError, match="flare tilt cap"):
        pc.PositionParams.from_params(P, MASS, T_MAX, flare_tilt_cap_deg=8.0)
    with pytest.raises(ValueError, match="tilt_fade_m"):
        pc.PositionParams.from_params(P, MASS, T_MAX, tilt_fade_m=0.0)
    big = pc.PositionParams.from_params(P, MASS, T_MAX, tilt_cap_deg=25.0)   # an overridden cap carries the flare cap
    assert big.flare_tilt_cap_deg == 25.0


def test_pos_params_gps_loss_land_after_by_profile():
    """params gps_loss.demo_land_after_s (architect 18:10, physics review HIGH 3): the demo LANDs 1 s after the last
    good step; the field keeps land_after_s."""
    gl = P["mission"]["vtol"]["position_hold"]["gps_loss"]
    demo = pc.PositionParams.from_params(P, MASS, T_MAX, profile="demo")
    assert demo.gps_land_after_s == float(gl["demo_land_after_s"]) == 0.0
    assert PP.gps_land_after_s == float(gl["land_after_s"])
    assert demo.land_steps == demo.timeout_steps == pc.ticks_ge(float(gl["timeout_s"]), demo.loop_hz)


def test_pos_integrator_learns_wind_and_does_not_wind_up_at_the_limit():
    ph = _engaged()
    for _ in range(500):                                                 # a steady 0.2 m/s error: the integrator ramps
        o = ph.step(_pin(_data(v=(-0.2, 0.0))))
    assert ph.integ[0] > 0.3 and not o.limited
    ph = _engaged()
    for _ in range(50):                                                  # pinned at the limit: frozen after one step
        o = ph.step(_pin(_data(p=(-20.0, 0.0), v=(-2.0, 0.0))))
    assert o.limited and ph.integ[0] == pytest.approx(PP.ki * (PP.v_max + 2.0) * PP.T)
    for _ in range(10000):                                               # the integrator limit
        ph.integ = [5.0, 5.0]
        o = ph.step(_pin(_data(v=(0.0, 0.0))))
        break
    assert math.hypot(*ph.integ) <= PP.i_max + 1e-12


def test_pos_hold_capture_reset_frame_up_down_and_land():
    ph = pc.PositionHold(PP)
    o = ph.step(_pin(_data(p=(3.0, 1.0)), phase=CLIMB, h=0.2))           # below h_enable: not engaged
    assert not o.engaged and o.tilt_l == (0.0, 0.0)
    o = ph.step(_pin(_data(p=(3.0, 1.0)), phase=CLIMB, h=0.5))           # engaged, climb: velocity hold, no point
    assert o.engaged and o.mode == pc.MODE_VEL and o.hold is None and math.isnan(o.dist_m)
    o = ph.step(_pin(_data(p=(3.2, 1.1)), phase=HOLD))                   # end of the climb: captured
    assert o.mode == pc.MODE_POS and o.hold == pytest.approx((3.2, 1.1))
    o = ph.step(_pin(_data(p=(3.5, 1.1)), phase=HOLD))                   # UP / DOWN keep the phase HOLD: kept
    assert o.hold == pytest.approx((3.2, 1.1))
    o = ph.step(_pin(_data(p=(13.5, 1.1), off=(10.0, 0.0), rc=1)))       # a NAV reset: p jumps 10 m with the offset
    assert o.hold == pytest.approx((13.2, 1.1)) and o.dist_m == pytest.approx(0.3)
    o = ph.step(_pin(_data(p=(13.4, 1.1), off=(10.0, 0.0), rc=1), phase=LAND))   # OFF: LAND keeps it
    assert o.hold == pytest.approx((13.2, 1.1)) and o.mode == pc.MODE_POS
    o = ph.step(_pin(None, motor=False, phase=LANDED))                   # motor off: everything reset
    assert o == pc.OFF_OUT and ph.hold_rf is None and not ph.engaged


def test_pos_gps_loss_dead_reckoning_attitude_only_and_land_request_steps():
    ph = _engaged()
    ph.integ = [0.4, -0.1]
    lost = _data(flags=0, p=(0.5, 0.0))
    modes, reqs = [], []
    for n in range(1, PP.land_steps + 2):
        o = ph.step(_pin(lost))
        modes.append(o.mode)
        reqs.append(o.requests & pc.REQ_GPS_LOST)
    assert PP.timeout_steps == 50 and PP.land_steps == 200
    assert modes[:PP.timeout_steps - 1] == [pc.MODE_DR] * (PP.timeout_steps - 1)
    assert modes[PP.timeout_steps - 1] == pc.MODE_ATT                    # lost = 50: attitude only
    assert not any(reqs[:PP.land_steps - 1]) and reqs[PP.land_steps - 1]  # lost = 200: the LAND request
    assert o.a_cmd == pytest.approx((0.4, -0.1)) and o.v_sp == (0.0, 0.0)  # the trim tilt only
    # the GPS back in LAND after the loss: the hold point is re-captured where the vehicle is
    o = ph.step(_pin(_data(p=(2.0, 1.0)), phase=LAND))
    assert o.hold == pytest.approx((2.0, 1.0)) and o.mode == pc.MODE_POS
    # a single hold step (lost frame) does not re-capture in LAND
    o = ph.step(_pin(None, phase=LAND))
    o = ph.step(_pin(_data(p=(2.3, 1.0)), phase=LAND))
    assert o.hold == pytest.approx((2.0, 1.0))


def test_pos_fence_request_exact_and_not_in_land():
    ph = _engaged()
    f = PP.fence_m
    o = ph.step(_pin(_data(p=(f, 0.0))))
    assert not o.requests & pc.REQ_FENCE                                  # == fence: not beyond
    o = ph.step(_pin(_data(p=(f + 0.01, 0.0))))
    assert o.requests & pc.REQ_FENCE
    ph = _engaged()
    o = ph.step(_pin(_data(p=(f + 5.0, 0.0)), phase=LAND))
    assert not o.requests & pc.REQ_FENCE


def test_pos_plausibility_limited_overspeed_and_resets():
    # (a) limited for 3 s with |v| not falling, the NAV flagging the GPS velocity inconsistent (physics-review fix)
    inc = gn.GF_FIX_OK | gn.GF_INCONS
    ph = _engaged()
    n = 0
    while not ph.out.requests & pc.REQ_IMPLAUSIBLE:
        ph.step(_pin(_data(p=(-20.0, 0.0), v=(-0.5, 0.0), flags=inc)))     # limited from the first step
        n += 1
        assert n < 1000
    assert n == PP.plaus_limited_steps
    o = ph.step(_pin(_data()))
    assert o.mode == pc.MODE_ATT                                          # latched: attitude only
    # ... but not when the speed falls during the run
    ph = _engaged()
    for i in range(400):
        ph.step(_pin(_data(p=(-20.0, 0.0), v=(-(1.4 - 0.003 * i), 0.0), flags=inc)))   # limited, the speed falling
    assert not ph.out.requests & pc.REQ_IMPLAUSIBLE
    # (b) |v| > v_max + 1 m/s for 2 s
    ph = _engaged()
    vt = PP.v_max + PP.plaus_v_excess_mps + 0.01
    for i in range(PP.plaus_vexc_steps):
        o = ph.step(_pin(_data(v=(vt, 0.0))))
        assert bool(o.requests & pc.REQ_IMPLAUSIBLE) == (i == PP.plaus_vexc_steps - 1)
    # (c) three NAV resets within 10 s
    ph = _engaged()
    ph.step(_pin(_data(rc=1)))
    ph.step(_pin(_data(rc=2)))
    o = ph.step(_pin(_data(rc=2)))
    assert not o.requests & pc.REQ_IMPLAUSIBLE
    o = ph.step(_pin(_data(rc=3)))
    assert o.requests & pc.REQ_IMPLAUSIBLE and o.mode == pc.MODE_ATT
    ph = _engaged()
    ph.step(_pin(_data(rc=1)))
    for _ in range(PP.plaus_reset_window_steps):
        ph.step(_pin(_data(rc=1)))
    ph.step(_pin(_data(rc=2)))
    o = ph.step(_pin(_data(rc=3)))
    assert not o.requests & pc.REQ_IMPLAUSIBLE                            # the first one left the window


def test_pos_authority_request():
    ph = _engaged()
    o = ph.step(_pin(_data(), k_t=PP.authority_min_ktw / PP.tw + 1e-6))
    assert not o.requests & pc.REQ_AUTHORITY
    o = ph.step(_pin(_data(), k_t=PP.authority_min_ktw / PP.tw - 1e-6))
    assert o.requests & pc.REQ_AUTHORITY


def test_pos_guard_spin_tilt_excess_and_release_near_the_limit():
    ph = _engaged()
    ph.integ = [0.3, 0.0]
    o = ph.step(_pin(_data(p=(3.0, 0.0)), rate=(0.0, 0.0, math.radians(PP.spin_z_dps) + 0.01)))
    assert o.guard and o.a_cmd == pytest.approx((0.3, 0.0))              # trim only, no new demand
    for i in range(PP.guard_steps - 1):
        o = ph.step(_pin(_data(p=(3.0, 0.0))))
        assert o.guard, i                                                 # held for hold_s after the last trigger
    o = ph.step(_pin(_data(p=(3.0, 0.0))))
    assert not o.guard
    # tilt excess: the estimated tilt more than 5 deg above the limit keeps the guard on (release only near the limit)
    ph = _engaged()
    c_lim = PP.tilt_limit(1.0)[0]
    lim = math.acos(c_lim)
    for k in range(60):
        o = ph.step(_pin(_data(p=(3.0, 0.0)), cos_tilt=math.cos(lim + math.radians(5.5))))
        assert o.guard
    o = ph.step(_pin(_data(p=(3.0, 0.0)), cos_tilt=math.cos(lim + math.radians(4.5))))
    assert o.guard                                                        # within 5 deg: the hold time runs
    for _ in range(PP.guard_steps):
        o = ph.step(_pin(_data(p=(3.0, 0.0)), cos_tilt=math.cos(lim + math.radians(4.5))))
    assert not o.guard
    # xy rate threshold
    ph = _engaged()
    o = ph.step(_pin(_data(), rate=(math.radians(PP.spin_xy_dps) * 0.8, math.radians(PP.spin_xy_dps) * 0.61, 0.0)))
    assert o.guard


def test_pos_no_new_tilt_demand_while_the_vertical_loop_is_saturated_high():
    ph = _engaged()
    o1 = ph.step(_pin(_data(p=(0.3, 0.0))))
    a1 = math.hypot(*o1.a_cmd)
    i1 = list(ph.integ)
    o2 = ph.step(_pin(_data(p=(3.0, 0.0)), sat=True))                    # would ask for much more
    assert math.hypot(*o2.a_cmd) <= a1 + 1e-12 and ph.integ == i1 and o2.limited
    o3 = ph.step(_pin(_data(p=(3.0, 0.0))))
    assert math.hypot(*o3.a_cmd) > a1


def test_touchdown_policy_from_params_is_keep_vel():
    """v0.9 control round (params position_hold.touchdown_policy): keep_vel (v0.8.1: keep_trim, architect 21:00);
    "keep_trim" and "fade" stay as comparison builds; anything else is refused."""
    assert P["mission"]["vtol"]["position_hold"]["touchdown_policy"]["policy"] == "keep_vel"
    assert PP.touchdown_policy == "keep_vel"
    assert pc.PositionParams.from_params(P, MASS, T_MAX, touchdown_policy="fade").touchdown_policy == "fade"
    assert pc.PositionParams.from_params(P, MASS, T_MAX, touchdown_policy="keep_trim").touchdown_policy == "keep_trim"
    with pytest.raises(ValueError):
        pc.PositionParams.from_params(P, MASS, T_MAX, touchdown_policy="upright")


def test_pos_land_keep_trim_limits_only_the_transient_above_the_trim():
    """keep_trim (position_ctrl step 4): in the vertical controller's flare band below TILT_FADE_M the profile limit
    stays, only e = a - i (the transient above the integrator = the wind trim) is limited to f a_lim, f = h / fade;
    at the pad level (f = 0) the command IS the trim; the ground signature still sets upright; trim_l is the
    integrator's tilt (limited to a_lim) in the level frame."""
    pp = replace(PP, touchdown_policy="keep_trim")
    ph = _engaged(pp)
    for _ in range(300):                                                 # a steady wind: the integrator builds a trim
        ph.step(_pin(_data(p=(0.0, 0.0), v=(-0.3, 0.1))))
    i0 = ph.integ[:]
    assert math.hypot(*i0) > 0.5
    o = ph.step(_pin(_data(p=(4.0, -2.0), v=(-0.3, 0.1)), phase=LAND, h=2.0))
    full = o.a_lim2
    for h, f in ((0.15, 0.5), (0.06, 0.2), (-0.05, 0.0), (-0.25, 0.0), (-0.45, 0.5)):
        o = ph.step(_pin(_data(p=(4.0, -2.0), v=(-0.3, 0.1)), phase=LAND, h=h))
        assert o.a_lim2 == pytest.approx(full)                           # no fade of the limit itself
        ex, ey = o.a_cmd[0] - o.integ[0], o.a_cmd[1] - o.integ[1]
        assert math.hypot(ex, ey) <= f * math.sqrt(full) + 1e-9, (h, f)
        assert o.a_cmd[0] ** 2 + o.a_cmd[1] ** 2 <= full * (1 + 1e-12)
        if f == 0.0:
            assert o.a_cmd == pytest.approx(o.integ) and o.tilt_l == pytest.approx(o.trim_l)
    # trim_l = the series tilt of the integrator, rotated into the level frame by the heading
    psi = 0.4
    o = ph.step(_pin(_data(p=(4.0, -2.0), v=(-0.3, 0.1), psi=psi), phase=LAND, h=-0.05))
    twx, twy = pc.tilt_of_accel(*o.integ)
    c, s = math.cos(psi), math.sin(psi)
    assert o.trim_l == pytest.approx((c * twx + s * twy, -s * twx + c * twy), abs=1e-12)
    assert o.tilt_l == pytest.approx(o.trim_l, abs=1e-12)
    # the ground signature: upright (both policies)
    o = ph.step(_pin(_data(p=(4.0, -2.0)), phase=LAND, h=-0.05, ground=True))
    assert o.a_lim2 == 0.0 and math.hypot(*o.tilt_w) < 1e-12 and o.trim_l == (0.0, 0.0)
    # the same descent with the v0.8 fade: the whole command goes to 0 at the pad level
    phf = _engaged(replace(PP, touchdown_policy="fade"))
    for _ in range(300):
        phf.step(_pin(_data(p=(0.0, 0.0), v=(-0.3, 0.1))))
    o = phf.step(_pin(_data(p=(4.0, -2.0), v=(-0.3, 0.1)), phase=LAND, h=-0.05))
    assert o.a_lim2 == 0.0 and math.hypot(*o.a_cmd) == 0.0 and math.hypot(*phf.integ) > 0.5


def test_pos_land_flare_cap_speed_and_touchdown_fade():
    """The v0.8 "fade" policy (kept as the comparison build)."""
    ph = _engaged(replace(PP, touchdown_policy="fade"))
    o = ph.step(_pin(_data(p=(10.0, 0.0)), phase=LAND, h=2.0))
    assert math.hypot(*o.v_sp) == pytest.approx(PP.v_max_land)
    full_hold = o.a_lim2
    o = ph.step(_pin(_data(p=(10.0, 0.0)), phase=LAND, h=0.8))           # below the flare altitude
    assert math.hypot(*o.v_sp) == pytest.approx(PP.flare_speed)
    assert o.cos_lim == pytest.approx(PP.tilt_limit(1.0)[0]) and o.a_lim2 == pytest.approx(full_hold)   # no flare cap
    full = o.a_lim2
    o = ph.step(_pin(_data(p=(10.0, 0.0)), phase=LAND, h=0.15))          # half the fade height: a quarter of a_lim^2
    assert o.a_lim2 == pytest.approx(full * 0.25)
    o = ph.step(_pin(_data(p=(10.0, 0.0)), phase=LAND, h=0.15, low_alt=False))   # the fade needs the vc flare band
    assert o.a_lim2 == pytest.approx(full)
    o = ph.step(_pin(_data(p=(10.0, 0.0)), phase=LAND, h=-0.05))
    assert o.a_lim2 == 0.0 and o.tilt_w == (0.0, 0.0) or math.hypot(*o.tilt_w) < 1e-12
    o = ph.step(_pin(_data(p=(10.0, 0.0)), phase=LAND, h=-0.25))         # the ground beside the pad: still upright
    assert o.a_lim2 == 0.0
    o = ph.step(_pin(_data(p=(10.0, 0.0)), phase=LAND, h=-0.45))         # airborne further below: lower ground
    assert o.a_lim2 == pytest.approx(full * 0.25)                        # f = (0.45 - 0.3) / 0.3 = 0.5
    o = ph.step(_pin(_data(p=(10.0, 0.0)), phase=LAND, h=-1.2))
    assert o.a_lim2 == pytest.approx(full)
    o = ph.step(_pin(_data(p=(10.0, 0.0)), phase=LAND, h=-1.2, ground=True))   # the vertical controller sees the ground
    assert o.a_lim2 == 0.0 and math.hypot(*o.tilt_w) < 1e-12
    o = ph.step(_pin(_data(p=(10.0, 0.0)), phase=LAND, h=2.5, ground=True))    # ground above the pad level
    assert o.a_lim2 == 0.0


def test_pos_engages_only_after_the_lift_off_is_confirmed():
    """Physics-review fix (medium 4): an uncalibrated baro offset puts the NAV h above h_enable on the pad - the loop
    waits for the vertical controller's take-off check (``lifted``)."""
    ph = pc.PositionHold(PP)
    for _ in range(20):
        o = ph.step(_pin(_data(p=(3.0, 0.0)), phase=CLIMB, h=0.98, lifted=False))
        assert not o.engaged and o.tilt_l == (0.0, 0.0)
    o = ph.step(_pin(_data(p=(3.0, 0.0)), phase=CLIMB, h=0.98, lifted=True))
    assert o.engaged


def test_pos_plausibility_a_counts_only_an_inconsistent_gps_at_the_authority_limit():
    """Physics-review fix (HIGH 2, params plausibility_v13_note): rule (a) counts a limited step only in CLIMB / HOLD
    above the flare altitude, with the vertical loop not saturated high and the NAV's GF_INCONS set; a healthy GPS at
    the authority limit (the operator wind limit) is never dropped."""
    n = PP.plaus_limited_steps
    far = dict(p=(-20.0, 0.0), v=(-2.0, 0.0))                             # limited, |v| not falling
    for flags, kw, want in ((gn.GF_FIX_OK, {}, False),                                  # consistent GPS: kept
                            (gn.GF_FIX_OK | gn.GF_INCONS, {}, True),                    # inconsistent: V13
                            (gn.GF_FIX_OK | gn.GF_INCONS, {"sat": True}, False),        # vertical loop saturated
                            (gn.GF_FIX_OK | gn.GF_INCONS, {"h": 0.9}, False)):          # below the flare altitude
        ph = _engaged()
        for _ in range(n + 5):
            o = ph.step(_pin(_data(flags=flags, **far), **kw))
        assert bool(o.requests & pc.REQ_IMPLAUSIBLE) == want, (flags, kw)
    ph = _engaged()                                                       # LAND: never counted
    for _ in range(n + 5):
        o = ph.step(_pin(_data(flags=gn.GF_FIX_OK | gn.GF_INCONS, **far), phase=LAND, h=5.0))
    assert not o.requests & pc.REQ_IMPLAUSIBLE
    ph = _engaged()                                                       # one consistent epoch ends the run
    for k in range(n + 5):
        fl = gn.GF_FIX_OK | (0 if k == n // 2 else gn.GF_INCONS)
        o = ph.step(_pin(_data(flags=fl, **far)))
    assert not o.requests & pc.REQ_IMPLAUSIBLE


def test_pos_no_plausibility_latch_on_the_ground():
    """Physics-review fix: rules (b) / (c) do not latch while the vertical controller sees the ground."""
    ph = _engaged()
    for k in range(PP.plaus_vexc_steps + 5):
        o = ph.step(_pin(_data(v=(PP.v_max + 2.0, 0.0), rc=k // 3), phase=LAND, h=0.0, ground=True))
    assert not o.requests & pc.REQ_IMPLAUSIBLE
    ph = _engaged()
    for k in range(PP.plaus_vexc_steps + 5):
        o = ph.step(_pin(_data(v=(PP.v_max + 2.0, 0.0)), phase=LAND, h=0.0))
    assert o.requests & pc.REQ_IMPLAUSIBLE                                # without the ground flag (b) latches


def test_pos_hold_step_keeps_outputs_and_counts_the_loss():
    ph = _engaged()
    o = ph.step(_pin(_data(p=(1.0, 0.0))))
    o2 = ph.step(_pin(None))
    assert o2.tilt_l == o.tilt_l and o2.a_cmd == o.a_cmd and o2.lost_steps == 1


def test_tilt_setpoint_slew_rate():
    ts = pc.TiltSetpoint(PP)
    dmax = math.radians(PP.tilt_rate_dps) / HZ
    tgt = (0.0, math.radians(10.0))
    n = 0
    while ts.t != tgt:
        prev = ts.t
        t = ts.tick(tgt)
        assert math.hypot(t[0] - prev[0], t[1] - prev[1]) <= dmax + 1e-12
        n += 1
    assert n == math.ceil(math.radians(10.0) / dmax)
    assert ts.tick(tgt, active=False) == (0.0, 0.0)


# =========================================================================== #
# Part 4 - estimator order, take-off check, supervisor, FcuStack, TELEM v3
# =========================================================================== #
def test_estimator_predict_then_correct_order():
    est = GatedMahonyEstimator.from_params(P, DT, 2.0)
    for _ in range(600):
        est.update(np.array([0.001, -0.002, 0.0005]), np.array([0.05, -0.1, G0]), True)
    q_pad, r = est.predict(np.array([0.001, -0.002, 0.0005]), True)
    assert np.array_equal(q_pad, est.q) and np.allclose(r, [0.001, -0.002, 0.0005] - est.bias)
    g = np.array([0.1, -0.05, 0.2])
    q0, b0 = est.q.copy(), est.bias.copy()
    q_pub, r = est.predict(g, False)
    assert np.array_equal(q_pub, quat.integrate(q0, g - b0, DT)) and np.array_equal(est.q, q0)   # no state change
    est.update(g, np.array([0.0, 0.0, G0]), False)
    q_next, _ = est.predict(g, False)
    assert np.array_equal(q_next, quat.integrate(est.q, g - est.bias, DT))  # the correction lands in the next tick


def test_vertical_takeoff_check_exact_ticks():
    vp = VP8
    assert vp.takeoff_ticks == onb.ticks_ge(3.0, HZ) == 750 and vp.takeoff_climb_m == 0.3
    vc = VerticalController(vp)
    vin = VerticalInputs(phase=CLIMB, h_tgt_m=5.0, baro_lost=False, h_m=0.0, vz_mps=-0.5, cos_tilt=1.0, f_z_mps2=G0)
    fails = []
    for k in range(vp.ramp_ticks + vp.takeoff_ticks + 5):
        o = vc.update(vin)
        fails.append(o.takeoff_fail)
    # saturated from the first tick; counted from the first tick with the ramp over (cap = u_max)
    first = fails.index(True)
    assert first == vp.ramp_ticks - 1 + vp.takeoff_ticks - 1 and o.takeoff_run >= vp.takeoff_ticks
    # climbing 0.3 m above the h0 latched after the ramp + the settle time passes (latched): never fails afterwards
    assert vp.takeoff_settle_ticks == onb.ticks_ge(vtc.TAKEOFF_SETTLE_S_ASSUMED, HZ) == 125
    vc = VerticalController(vp)
    for k in range(vp.ramp_ticks - 1 + vp.takeoff_settle_ticks):        # the ramp ends at tick ramp_ticks - 1
        o = vc.update(vin)
    assert not vc.to_h0_ok
    o = vc.update(VerticalInputs(phase=CLIMB, h_tgt_m=5.0, baro_lost=False, h_m=0.31, vz_mps=-0.5, cos_tilt=1.0))
    assert vc.to_h0_ok and vc.to_h0 == 0.31 and not o.takeoff_passed     # h0 latched at this tick (no climb yet)
    o = vc.update(VerticalInputs(phase=CLIMB, h_tgt_m=5.0, baro_lost=False, h_m=0.62, vz_mps=-0.5, cos_tilt=1.0))
    assert o.takeoff_passed and o.takeoff_run == 0
    for k in range(2000):
        o = vc.update(VerticalInputs(phase=CLIMB, h_tgt_m=5.0, baro_lost=False, h_m=0.0, vz_mps=-0.5, cos_tilt=1.0))
    assert not o.takeoff_fail
    # physics-review fix (medium 4): a thrust-proportional baro offset that lifts the NAV h by 1 m DURING the ramp and
    # the settle time (the vehicle on the pad) is inside h0 - the check still fails the vehicle that cannot climb
    vc = VerticalController(vp)
    n_set = vp.ramp_ticks + vp.takeoff_settle_ticks
    fails = []
    for k in range(n_set + vp.takeoff_ticks + 5):
        h = min(1.0, k / n_set)                                             # 0 -> 1 m while spooling and settling
        o = vc.update(VerticalInputs(phase=CLIMB, h_tgt_m=5.0, baro_lost=False, h_m=h, vz_mps=0.0, cos_tilt=1.0,
                                     f_z_mps2=G0))
        fails.append(o.takeoff_fail)
        assert not o.takeoff_passed
    assert any(fails)
    # before the fix (h0 = h at the start) the same offset passed the check
    vc = VerticalController(replace(vp, takeoff_settle_ticks=0))
    passed = False
    for k in range(n_set + 5):
        h = min(1.0, k / n_set)
        passed = passed or vc.update(VerticalInputs(phase=CLIMB, h_tgt_m=5.0, baro_lost=False, h_m=h, vz_mps=0.0,
                                                    cos_tilt=1.0, f_z_mps2=G0)).takeoff_passed
    assert passed
    # off without the params block
    q = copy.deepcopy(P)
    q["mission"]["vtol"]["position_hold"].pop("takeoff_check_ASSUMED")
    assert VerticalParams.from_params(q, MASS, T_MAX).takeoff_ticks == 0


def test_fcustack_from_params_builds_consistent_parts():
    st = FcuStack.from_params(P, MASS, T_MAX, mode=lp.MODE_VTOL, profile="demo", seed=4)
    assert st.sup.sp.gps_hold and st.pp.profile == st.vc.vp.profile == st.sup.sp.profile == "demo"
    assert st.pp.tw == pytest.approx(T_MAX / (MASS * G0)) and st.pp.tilt_cap_deg == 10.0
    st71 = FcuStack.from_params(P, MASS, T_MAX, mode=lp.MODE_VTOL, gps=False, takeoff_check=False)
    assert st71.pos is None and not st71.sup.sp.takeoff_check
    stb = FcuStack.from_params(P, MASS, T_MAX, mode=lp.MODE_BALLISTIC)
    assert stb.vc is None and stb.pos is None
    assert FcuStack.from_params(P, MASS, T_MAX, mode=lp.MODE_VTOL, pgains={"kv": 1.5}).pp.kv == 1.5
    with pytest.raises(ValueError):
        FcuStack(SP8, VP8)                                                  # position hold needs PositionParams


def test_c_header_constants_mirror_the_physics_review_fixes():
    """The physics-review constants that are module constants of the reference (not params.json, so not in params_gen.h)
    are mirrored as C header constants: they must be equal (core/vertical_ctrl.h, gps_nav.h, position_ctrl.h)."""
    import re
    core = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "firmware", "atmega128_fc", "src", "core")

    def define(header: str, name: str) -> str:
        with open(os.path.join(core, header), encoding="utf-8") as f:
            m = re.search(rf"^#define {name}\s+(\S+)", f.read(), re.M)
        assert m, (header, name)
        return m.group(1)
    assert int(define("vertical_ctrl.h", "VC_TAKEOFF_SETTLE_MS").rstrip("uU")) == round(vtc.TAKEOFF_SETTLE_S_ASSUMED * 1000)
    assert VP8.takeoff_settle_ticks == (500 * HZ + 999) // 1000                        # VC_TAKEOFF_SETTLE_TICKS(hz)
    assert int(define("gps_nav.h", "GPS_INCONS_EPOCHS").rstrip("uU")) == gn.INCONS_EPOCHS
    assert int(define("gps_nav.h", "GPS_INCONS_MIN_REJ").rstrip("uU")) == gn.INCONS_MIN_REJ
    assert float(define("gps_nav.h", "GPS_YAW_SIGMA_REL2_ASSUMED").rstrip("fF")) == gn.YAW_SIGMA_REL ** 2
    assert int(define("gps_nav.h", "GPS_F_INCONS").rstrip("uU"), 16) == gn.GF_INCONS == pc.GF_INCONS
    assert int(define("position_ctrl.h", "POS_GF_INCONS").rstrip("uU"), 16) == gn.GF_INCONS


def test_v08_params_codes_and_texts():
    assert SP8.gps_hold and SP8.takeoff_check and SP8.vtol
    s71 = SupervisorParams.from_params(P, mode=lp.MODE_VTOL, gps=False, takeoff_check=False)
    assert not s71.gps_hold and not s71.takeoff_check
    sb = SupervisorParams.from_params(P, mode=lp.MODE_BALLISTIC, gps=True)
    assert not sb.gps_hold and not sb.takeoff_check                       # never in the ballistic build
    assert (lp.R_V_GPS_LOST, lp.R_V_FENCE, lp.R_V_GPS_IMPLAUSIBLE, lp.R_V_AUTHORITY, lp.R_V_TAKEOFF_CHECK,
            lp.REJ_GPS) == (24, 25, 26, 27, 28, 0x50)
    for c in lp.V08_INTERLOCK_REASONS + lp.V08_REJECT_REASONS:
        assert c in REASON_TEXT_KO and any("가" <= ch <= "힣" for ch in REASON_TEXT_KO[c])
        assert c not in lp.VTOL_INTERLOCK_REASONS + lp.VTOL_REJECT_REASONS
        assert c in lp.VTOL_GPS_INTERLOCK_REASONS + lp.VTOL_GPS_REJECT_REASONS
    assert lp.BALLISTIC_REJECT_REASONS == tuple(range(0x40, 0x4C))       # the ballistic sets are v0.6.3's


class _Sup:
    """The supervisor alone, inputs set per tick (exact-tick rule tests)."""

    def __init__(self, sp=SP8):
        self.sup = OnboardSupervisor(sp, seed=0x5EED)
        self.k = 0
        self.seq = 1

    def tick(self, cmd=None, **kw):
        frames = []
        if cmd is not None or self.k % 50 == 0:
            if cmd is not None:
                self.seq = (self.seq + 1) & 0xFF
            frames = [UplinkFrame(self.seq, lp.Command(lp.CMD_NONE if cmd is None else cmd, 0, self.sup.nonce))]
        base = dict(tick=self.k, frames=frames, h_m=1.0, vz_mps=0.0, tilt_deg=0.5, v_cell=4.0, gps_gate_ok=True)
        base.update(kw)
        o = self.sup.update(OnboardInputs(**base))
        self.k += 1
        return o

    def to_hold(self, **kw):
        self.tick()
        o = self.tick(lp.CMD_ON, **kw)
        assert o.state == A_
        while self.sup.state != P_:
            self.tick(**kw)
        o = self.tick(vc_at_target=True, **kw)
        assert o.phase == HOLD
        return o


def test_on_gate_rej_gps_and_its_place_in_the_order():
    h = _Sup()
    h.tick()
    o = h.tick(lp.CMD_ON, gps_gate_ok=False)
    assert o.state == S_ and o.reason == lp.REJ_GPS and o.telemetry.flag(lp.FLAG_REJECTED)
    o = h.tick(lp.CMD_ON, gps_gate_ok=False, aligned=False)              # not aligned outranks the GPS
    assert o.reason == lp.REJ_NOT_ALIGNED
    o = h.tick(lp.CMD_ON, gps_gate_ok=False, tilt_deg=10.0)              # the GPS outranks the tilt
    assert o.reason == lp.REJ_GPS
    o = h.tick(lp.CMD_ON, gps_gate_ok=True)
    assert o.state == A_
    h71 = _Sup(SupervisorParams.from_params(P, mode=lp.MODE_VTOL, gps=False, takeoff_check=False))
    h71.tick()
    assert h71.tick(lp.CMD_ON, gps_gate_ok=False).state == A_             # the v0.7.1 build has no GPS gate


@pytest.mark.parametrize("bit,code", [(pc.REQ_GPS_LOST, lp.R_V_GPS_LOST), (pc.REQ_FENCE, lp.R_V_FENCE),
                                      (pc.REQ_IMPLAUSIBLE, lp.R_V_GPS_IMPLAUSIBLE), (pc.REQ_AUTHORITY, lp.R_V_AUTHORITY)])
def test_v08_land_rules_act_on_the_tick_of_the_request_climb_hold_only(bit, code):
    h = _Sup()
    h.to_hold()
    o = h.tick()
    assert o.phase == HOLD
    o = h.tick(pc_requests=bit)
    assert o.phase == LAND and o.reason == code and o.flags & lp.FLAG_INTERLOCK and o.state == P_
    o = h.tick(pc_requests=bit | pc.REQ_FENCE)                            # in LAND: nothing more
    assert o.phase == LAND and o.reason == code
    # CLIMB acts too
    h = _Sup()
    h.tick()
    h.tick(lp.CMD_ON)
    while h.sup.state != P_:
        h.tick()
    assert h.tick(pc_requests=bit).phase == LAND
    # the v0.7.1 build ignores the requests
    h = _Sup(SupervisorParams.from_params(P, mode=lp.MODE_VTOL, gps=False, takeoff_check=False))
    h.to_hold()
    assert h.tick(pc_requests=bit).phase == HOLD


def test_v08_rule_order_existing_land_rules_first_then_gps_lost_fence_implausible_authority():
    h = _Sup()
    h.to_hold()
    o = h.tick(pc_requests=0x0F, v_cell=3.0)                              # I8 (V_cell) comes before the GPS rules
    assert o.reason == lp.R_I8_VCELL
    h = _Sup()
    h.to_hold()
    assert h.tick(pc_requests=pc.REQ_AUTHORITY | pc.REQ_FENCE).reason == lp.R_V_FENCE
    h = _Sup()
    h.to_hold()
    assert h.tick(pc_requests=0x0F).reason == lp.R_V_GPS_LOST


def test_takeoff_check_rule_goes_safe_without_release():
    h = _Sup()
    h.tick()
    h.tick(lp.CMD_ON)
    while h.sup.state != P_:
        h.tick()
    o = h.tick(vc_takeoff_fail=True)
    assert o.state == S_ and o.reason == lp.R_V_TAKEOFF_CHECK and not o.release and not o.motor_on
    for _ in range(1000):
        o = h.tick()
    assert o.state == S_ and not o.release
    h = _Sup(SupervisorParams.from_params(P, mode=lp.MODE_VTOL, takeoff_check=False))
    h.tick()
    h.tick(lp.CMD_ON)
    while h.sup.state != P_:
        h.tick()
    assert h.tick(vc_takeoff_fail=True).state == P_


def test_telemetry_v3_in_the_gps_build_v2_without():
    h = _Sup()
    o = h.tick(tlm_gps=(0x29, 9, 25, 2, 0.73))
    t = o.telemetry
    assert t.is_v3 and t.wire_type == lp.TYPE_TELEM3 and (t.gps_flags, t.num_sv, t.hacc_dm, t.pos_mode) == (0x29, 9, 25, 2)
    assert t.dist_dm == 7
    o = h.tick(cmd=lp.CMD_NONE)
    assert o.telemetry.is_v3 and o.telemetry.dist_dm == lp.DIST_NONE and o.telemetry.hacc_dm == 255
    h71 = _Sup(SupervisorParams.from_params(P, mode=lp.MODE_VTOL, gps=False))
    t2 = h71.tick(tlm_gps=(0x2B, 9, 25, 2, 0.73)).telemetry
    assert t2.is_v2 and not t2.is_v3 and t2.wire_type == lp.TYPE_TELEM2
    st = GroundAssist(GroundParams.from_params(P, 0.5, 0.2))
    st.receive(lp.encode_telemetry(90, 0, t), 0.0)
    vs = st.vtol_status()
    assert vs["gps_ready"] is False and vs["num_sv"] == 9 and vs["dist_m"] == pytest.approx(0.7)
    assert "위성 9개" in gps_status_ko(t) and "유지점에서 0.7 m" in gps_status_ko(t)


def test_v08_random_sequences_invariants_and_every_new_code_reachable():
    """Random operator, faults and v0.8 inputs (GPS gate, position-controller requests, take-off check) against the
    v0.8 supervisor: the new rules only LAND from CLIMB / HOLD (or go SAFE: V15), never ABORT, never fire the release;
    REJ_GPS only for an ON without the gate; TELEM is v3; every new code occurs."""
    import dataclasses
    rng = random.Random(20261021)
    seen_il, seen_rej, trans = set(), set(), set()
    for i in range(40):
        sp = dataclasses.replace(SP8, spool_ticks=rng.choice([0, 30]), endurance_ticks=rng.choice([600, 3000]),
                                 link_land_ticks=rng.choice([60, 400]), abort_release_ticks=rng.choice([0, 12]))
        sp = dataclasses.replace(sp, hard_cutoff_ticks=sp.endurance_ticks + 300)
        sup = OnboardSupervisor(sp, seed=rng.randrange(65536))
        seq, prev_state, prev_phase = rng.randrange(256), None, None
        gate_p = rng.choice([0.5, 0.95, 1.0])
        for t in range(rng.choice([1500, 3000])):
            frames = []
            if t % 20 == 0 and rng.random() > 0.05:
                seq = (seq + 1) & 0xFF
                c = rng.choices([lp.CMD_ON, lp.CMD_NONE, lp.CMD_UP, lp.CMD_DOWN, lp.CMD_OFF, lp.CMD_KILL],
                                [6, 8, 1, 1, 0.5, 0.1] if sup.state != P_ else [0.3, 12, 2, 2, 0.3, 0.05])[0]
                frames = [UplinkFrame(seq, lp.Command(c, 0, sup.nonce))]
            st = sup.state
            req = 0
            if st == P_ and rng.random() < 0.004:
                req = rng.choice([pc.REQ_GPS_LOST, pc.REQ_FENCE, pc.REQ_IMPLAUSIBLE, pc.REQ_AUTHORITY, 0x0F])
            inp = OnboardInputs(tick=t, frames=frames, h_m=1.0, vz_mps=0.0, tilt_deg=1.0, v_cell=4.0,
                                gps_gate_ok=rng.random() < gate_p, pc_requests=req,
                                vc_takeoff_fail=st == P_ and sup.phase == CLIMB and rng.random() < 0.003,
                                vc_at_target=sup.phase == CLIMB and rng.random() < 0.01,
                                vc_touchdown=sup.phase == LAND and rng.random() < 0.004,
                                tlm_gps=(rng.randrange(256), rng.randrange(13), rng.randrange(256), rng.randrange(5),
                                         rng.uniform(0, 20)))
            out = sup.update(inp)
            ex = [e for e in sup.exec_log if e["tick"] == t]
            for e in ex:
                if e["code"]:
                    seen_rej.add(e["code"])
                    if e["code"] == lp.REJ_GPS:
                        assert e["cmd"] == lp.CMD_ON and not inp.gps_gate_ok
            if out.reason in lp.V08_INTERLOCK_REASONS and any(f": V1{d} " in e for e in out.events for d in "12345"):
                seen_il.add(out.reason)
                assert out.state != AB_ and not out.release
                if out.reason == lp.R_V_TAKEOFF_CHECK:
                    assert out.state == S_ and prev_state == P_ and prev_phase in (CLIMB, HOLD)
                else:
                    assert out.state == P_ and out.phase == LAND and prev_phase in (CLIMB, HOLD)
            for e in out.events:
                a, sep, rest = e.partition(" -> ")
                if sep and a.isupper() and " " not in a:
                    trans.add((a, rest.split(":")[0]))
            if out.telemetry is not None:
                assert out.telemetry.is_v3
            prev_state, prev_phase = out.state, out.phase
    assert set(lp.V08_INTERLOCK_REASONS) <= seen_il, sorted(set(lp.V08_INTERLOCK_REASONS) - seen_il)
    assert lp.REJ_GPS in seen_rej
    assert ("POWERED", "SAFE") in trans                                    # V15


def _slots_for(snap, cycle):
    return [nl.encode_slot(snap, cycle, j) for j in range(5)]


def test_fcustack_slots_gate_and_request_after_the_job_pipeline():
    """FcuStack v0.8 with synthetic slots: the ON gate from GATE_OK, the climb, the hold capture, a fence request
    computed by the position step from the inputs of tick k, committed at k + PIPELINE_TICKS (job J4) and acting in
    the supervisor at k + PIPELINE_TICKS + 1; the tilt follows the same pipeline."""
    st = FcuStack(SP8, VP8, seed=3, pp=PP)
    snap = gn.GpsSnapshot(flags=gn.GF_FIX_OK | gn.GF_GATE_OK | gn.GF_FIX3D | gn.GF_INIT, num_sv=9, hacc_dm=25,
                          cos_q15=32767)
    k = 0
    seq = 1
    cyc = 0
    pos = [0.0, 0.0]
    log = []

    def tick(cmd=None, h=0.0, snap_now=None):
        nonlocal k, seq, cyc
        frames = []
        if cmd is not None or k % 50 == 0:
            if cmd is not None:
                seq += 1
            frames = [UplinkFrame(seq, lp.Command(lp.CMD_NONE if cmd is None else cmd, 0, st.sup.nonce))]
        sn = snap_now or gn.GpsSnapshot(**{**vars(snap), "px_cm": gn.quant_cm(pos[0]), "py_cm": gn.quant_cm(pos[1])})
        slot = nl.encode_slot(sn, cyc, k % 5)
        if k % 5 == 4:
            cyc += 1
        o = st.step(OnboardInputs(tick=k, frames=frames, h_m=h, vz_mps=0.0, tilt_deg=0.2, v_cell=4.0, cos_tilt=1.0,
                                  gps_slot=slot))                  # no f_z: no thrust learning in this script
        log.append(o)
        k += 1
        return o

    o = tick()
    o = tick(lp.CMD_ON, snap_now=gn.GpsSnapshot(flags=gn.GF_FIX_OK))       # no complete set yet: REJ_GPS
    assert o.sup.reason == lp.REJ_GPS and o.sup.state == S_
    for _ in range(10):
        tick()
    o = tick(lp.CMD_ON)
    assert o.sup.state == A_
    while st.sup.state != P_:
        tick()
    h = 0.0
    while st.sup.phase == CLIMB:
        h += 0.004
        tick(h=h)
    assert st.sup.phase == HOLD
    for _ in range(10):
        tick(h=5.0)
    assert st.pos_out().hold == pytest.approx((0.0, 0.0))
    pos[0] = PP.fence_m + 1.0                                              # the vehicle beyond the fence
    k_req, phases = None, {}
    for _ in range(25):
        o = tick(h=5.0)
        phases[k - 1] = (o.sup.phase, o.sup.reason, st.pos_out().requests)
        if k_req is None and o.pos is not None and o.pos.requests & pc.REQ_FENCE:
            k_req = k - 1
    assert k_req is not None
    pt = pc.PIPELINE_TICKS
    for kk in range(k_req, k_req + pt + 1):                                 # still HOLD until the commit is seen
        assert phases[kk][0] == HOLD, kk
    assert not phases[k_req + pt - 1][2] & pc.REQ_FENCE and phases[k_req + pt][2] & pc.REQ_FENCE   # committed at k + 4
    assert phases[k_req + pt + 1][:2] == (LAND, lp.R_V_FENCE)               # the supervisor acts at k + 5
    # the tilt: a step's target moves the builder from its commit tick on (an upright vehicle before the fence step)
    tl = [x.tilt_next for x in log]
    assert tl[k_req + pt - 1] == tl[k_req - 1] or math.hypot(*tl[k_req + pt]) > math.hypot(*tl[k_req + pt - 1])
    # the position steps ran on the slot-4 ticks only (5 ticks apart)
    steps = [i for i, x in enumerate(log) if x.pos is not None]
    assert all(b - a == 5 for a, b in zip(steps, steps[1:])) and all(i % 5 == 4 for i in steps)
    # the tilt the attitude controller uses is last tick's builder output
    assert log[-1].att_tilt == log[-2].tilt_next


# =========================================================================== #
# Part 5 - 3-D point-mass closed loop of the whole chain (NOT the 6-DOF)
# =========================================================================== #
class Loop3D:
    """NAV (estimator stand-in: the true attitude; NavGps on UBX bytes) -> nlink v3 slots -> FcuStack (supervisor,
    vertical controller, position hold, tilt setpoint) -> a point mass with a first-order attitude lag
    (``tau_att``), the thrust lag of the EDF and linear drag toward the wind (``tau_d`` 9.55 s, the v0.7.1 drag time
    constant).  GPS: white noise ``gps_noise`` (position, velocity), 5 Hz, 0.22 s latency."""

    def __init__(self, wind=(0.0, 0.0), seed=1, profile="field", gps_noise=(0.3, 0.05), outage=None, thrust=1.0,
                 tau_att=0.15, tau_d=9.55):
        self.sp = SupervisorParams.from_params(P, mode=lp.MODE_VTOL, profile=profile)
        self.vp = VerticalParams.from_params(P, MASS, T_MAX, profile=profile)
        self.pp = pc.PositionParams.from_params(P, MASS, T_MAX, profile=profile)
        self.fcu = FcuStack(self.sp, self.vp, seed=seed, pp=self.pp)
        self.gp = gn.NavGpsParams.from_params(P)
        self.nav = gn.NavGps(self.gp, configure=False)
        self.tx = nl.GpsSlotTx()
        self.rng = random.Random(seed)
        self.p = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self.T = 0.0
        self.tilt = (0.0, 0.0)
        self.wind = wind
        self.k = 0
        self.seq = 10
        self.noise = gps_noise
        self.inbox: list = []
        self.next_ms = 200
        self.outage = outage
        self.thrust = thrust
        self.tau_att, self.tau_d = tau_att, tau_d
        self.on_ground = True
        self.touch = None

    def q(self):
        return tuple(float(x) for x in quat.from_rotvec(np.array([self.tilt[0], self.tilt[1], 0.0])))

    def tick(self, cmd=None, push=None):
        k = self.k
        frames = []
        if cmd is not None or k % 50 == 0:
            if cmd is not None:
                self.seq = (self.seq + 1) & 0xFF
            frames = [UplinkFrame(self.seq, lp.Command(lp.CMD_NONE if cmd is None else cmd, 0, self.fcu.sup.nonce))]
        st = self.fcu.sup.state
        q = self.q()
        c, tdeg = nl.nav3_tilt(q)
        tilt_cmd = self.fcu.attitude_tilt()
        o = self.fcu.step(OnboardInputs(tick=k, frames=frames, h_m=self.p[2] + self.rng.gauss(0, 0.02),
                                        vz_mps=self.v[2] + self.rng.gauss(0, 0.02), tilt_deg=tdeg, v_cell=4.0,
                                        cos_tilt=c, accel_z_mps2=G0 if self.on_ground else self.T / MASS,
                                        gps_slot=self.tx.slot(k)))
        a = 1.0 - math.exp(-DT / self.tau_att)
        self.tilt = (0.0, 0.0) if not o.sup.motor_on else (self.tilt[0] + a * (tilt_cmd[0] - self.tilt[0]),
                                                           self.tilt[1] + a * (tilt_cmd[1] - self.tilt[1]))
        self.T += (self.thrust * T_MAX * max(o.throttle, 0.0) ** 2 - self.T) * (1.0 - math.exp(-DT / 0.15))
        n = quat.to_matrix(np.array(self.q())) @ np.array([0.0, 0.0, 1.0])
        acc = [self.T / MASS * n[0] + (self.wind[0] - self.v[0]) / self.tau_d,
               self.T / MASS * n[1] + (self.wind[1] - self.v[1]) / self.tau_d, self.T / MASS * n[2] - G0]
        if push is not None:
            acc[0] += push[0] / MASS / DT
            acc[1] += push[1] / MASS / DT
        if self.on_ground and acc[2] <= 0.0:
            acc = [0.0, 0.0, 0.0]
            self.v = [0.0, 0.0, 0.0]
        else:
            self.on_ground = False
        for i in range(3):
            self.v[i] += acc[i] * DT
            self.p[i] += self.v[i] * DT
        if self.p[2] < 0.0 and not self.on_ground:
            if self.touch is None and o.sup.state == P_:
                self.touch = (math.hypot(self.v[0], self.v[1]), -self.v[2], list(self.p))
            self.p[2] = 0.0
            self.v = [0.0, 0.0, 0.0]
            self.on_ground = True
        f_w = np.array([0.0, 0.0, G0]) if self.on_ground else np.array([acc[0], acc[1], acc[2] + G0])
        f_b = quat.to_matrix(np.array(self.q())).T @ f_w
        if 4 * k >= self.next_ms:
            te = self.next_ms
            self.next_ms += 200
            out = self.outage is not None and self.outage[0] <= te / 1000.0 < self.outage[1]
            r = quat.to_matrix(np.array(self.q())) @ np.array(self.gp.lever_b_m)
            sp_, sv_ = self.noise
            la, lo = LF.to_llh(self.p[0] + r[0] + self.rng.gauss(0, sp_), self.p[1] + r[1] + self.rng.gauss(0, sp_))
            vn = lp.round_half_away((self.v[0] + self.rng.gauss(0, sv_)) * 100)
            ve = lp.round_half_away(-(self.v[1] + self.rng.gauss(0, sv_)) * 100)
            ep = gn.GpsEpoch(345600000 + te, la, lo, 0, 2500, 3900, vn, ve, 0, 30, 0 if out else 3, not out,
                             0 if out else 9, 350)
            self.inbox.append((k + 55, gn.epoch_frames(ep)))
        rx = b"".join(b for t_, b in self.inbox if t_ <= k)
        self.inbox = [(t_, b) for t_, b in self.inbox if t_ > k]
        _, snap = self.nav.tick(k, rx, f_b, q, (0.0, 0.0, 0.0), nl.nav_pad_mode(nl.Fcu(0, int(st))))
        if k % 5 == 4:
            self.tx.latch(snap, self.nav.filter.cycle)
        self.k += 1
        return o

    def run(self, s, until=None):
        o = None
        for _ in range(int(round(s * HZ))):
            o = self.tick()
            if until is not None and until(o):
                break
        return o

    def to_hold(self):
        self.run(6.0)
        o = self.tick(lp.CMD_ON)
        assert o.sup.state == A_, lp.REASON_NAMES.get(o.sup.reason)
        self.run(20.0, until=lambda o: o.sup.phase == HOLD)
        assert self.fcu.sup.phase == HOLD
        self.run(0.2)
        return self.fcu.pos.out.hold


@pytest.mark.parametrize("wind", [(0.0, 0.0), (3.0, 0.0), (0.0, -5.0)])
def test_closed_loop_hold_in_wind_push_return_and_land_at_the_hold_point(wind):
    s = Loop3D(wind=wind, seed=2)
    s.run(1.0)
    assert s.tick(lp.CMD_ON).sup.reason == lp.REJ_GPS                       # the gate needs 5 s of good fixes
    hold = s.to_hold()
    p_hold = list(s.p[:2])
    s.run(12.0)          # fix round: 15 -> 12 s - the climb now takes ~3 s longer (reference hold + approach) and the
                         # scripted flight must stay inside the mission's 30 s motor time (else V2 lands it: an interlock)
    d_nav = s.fcu.pos.out.dist_m
    d_true = math.hypot(s.p[0] - p_hold[0], s.p[1] - p_hold[1])
    assert d_nav < 0.3 and d_true < 1.0 and not s.fcu.pos.out.limited
    s.tick(push=(3.0, 0.0))                                                  # a 3 N s push
    dmax, t_back = 0.0, None
    for i in range(int(8 * HZ)):
        s.tick()
        d = s.fcu.pos.out.dist_m
        dmax = max(dmax, d)
        if t_back is None and dmax > 0.3 and d < 1.0 and i > 50:
            t_back = i * DT
    assert 0.2 < dmax < 1.5 and t_back is not None and t_back < 6.0
    s.tick(lp.CMD_OFF)
    o = s.run(30.0, until=lambda o: o.sup.state != P_)
    assert o.sup.state == U_ and not o.sup.flags & lp.FLAG_INTERLOCK         # landed, no safety rule acted
    vh, vz, p_td = s.touch
    assert vh < 0.5 and vz < 0.6 and math.hypot(p_td[0] - p_hold[0], p_td[1] - p_hold[1]) < 1.5


def test_closed_loop_gps_outage_attitude_only_then_land():
    s = Loop3D(wind=(2.0, 0.0), seed=4)
    s.to_hold()
    t0 = s.k * DT
    s.outage = (t0, t0 + 60.0)
    o = s.run(10.0, until=lambda o: o.sup.phase == LAND)
    t_land = s.k * DT - t0
    assert o.sup.phase == LAND and o.sup.reason == lp.R_V_GPS_LOST
    # last good epoch + NAV fresh 0.6 s + slot 0.02 s + FCU timeout 1 s + land_after 3 s (+ latency 0.22 s)
    assert 4.4 < t_land < 5.4
    assert s.fcu.pos.out.mode == pc.MODE_ATT
    o = s.run(30.0, until=lambda o: o.sup.state != P_)
    assert o.sup.state == U_                                                 # landed (drifting, as v0.7.1)


def test_closed_loop_demo_fence_in_wind_beyond_the_authority_lands_never_aborts():
    # the linear drag of this plant needs ~15 deg of trim for 25 m/s - more than the demo cap (10 deg); the 6-DOF's
    # quadratic drag reaches that near 9 m/s (prototype)
    s = Loop3D(wind=(25.0, 0.0), profile="demo", seed=6, gps_noise=(0.1, 0.03))
    # fix round: with the slower climb (reference hold + approach) the fence may already land the vehicle in the CLIMB
    s.run(6.0)
    assert s.tick(lp.CMD_ON).sup.state == A_
    s.run(25.0, until=lambda o: o.sup.phase in (HOLD, LAND))
    o = s.run(30.0, until=lambda o: o.sup.phase == LAND)
    assert o.sup.phase == LAND and o.sup.reason in (lp.R_V_FENCE, lp.R_V_GPS_IMPLAUSIBLE)
    o = s.run(40.0, until=lambda o: o.sup.state != P_)
    assert o.sup.state in (U_, S_) and o.sup.state != AB_ and not o.sup.release


def test_closed_loop_weak_vehicle_take_off_check_motor_off_safe():
    s = Loop3D(thrust=0.80, seed=8)                                         # T/W 0.94: cannot lift itself
    s.run(6.0)
    s.tick(lp.CMD_ON)
    o = s.run(15.0, until=lambda o: o.sup.state == S_ and o.sup.reason == lp.R_V_TAKEOFF_CHECK)
    assert o.sup.state == S_ and o.sup.reason == lp.R_V_TAKEOFF_CHECK and not o.sup.release
    assert s.p[2] < 0.3


def test_v09_pos_land_keep_vel_holds_zero_ground_speed_in_the_flare():
    """v0.9 touchdown_policy keep_vel (position_ctrl step 4 / 6): above the flare band a LAND still returns to the hold
    point; in the vertical controller's flare band (no ground signature) v_sp = 0 and the velocity loop keeps its full
    authority (no transient fade, the profile limit unchanged); the ground signature sets upright as in every policy."""
    pp = replace(PP, touchdown_policy="keep_vel")
    ph = _engaged(pp)
    for _ in range(300):                                                 # a steady wind: the integrator builds a trim
        ph.step(_pin(_data(p=(0.0, 0.0), v=(-0.3, 0.1))))
    o = ph.step(_pin(_data(p=(4.0, -2.0), v=(-0.3, 0.1)), phase=LAND, h=2.0))
    full = o.a_lim2
    assert math.hypot(*o.v_sp) > 0.1                                    # above the flare band: back to the hold point
    for h in (0.8, 0.15, -0.05, -0.45):
        o = ph.step(_pin(_data(p=(4.0, -2.0), v=(-0.3, 0.1)), phase=LAND, h=h))
        assert o.v_sp == (0.0, 0.0) and o.a_lim2 == pytest.approx(full)
        ex, ey = o.a_cmd[0] - o.integ[0], o.a_cmd[1] - o.integ[1]
        assert math.hypot(ex, ey) > 0.3                                  # the velocity loop still acts on the 0.32 m/s
    o = ph.step(_pin(_data(p=(4.0, -2.0), v=(-0.3, 0.1)), phase=LAND, h=-0.05, ground=True))
    assert o.a_lim2 == 0.0 and math.hypot(*o.tilt_w) < 1e-12
