"""Tests for the ground station (``tools/ground_station.py``) and the SX127x bridge header generator.

v0.7: params.json ``mission.mode`` is "vtol", so every ballistic test below names its build (``mode="ballistic"``,
``--mode ballistic``); the VTOL tests at the end cover the ON / OFF / UP / DOWN / KILL station (refusals, confirmations,
the ON nonce, Ctrl+C = OFF, the other build's commands refused), a ``--sim`` VTOL hop through the real
``sim.onboard.FcuStack`` (take-off, hold, UP / DOWN, landing, LANDED -> SAFE), KILL in the hover, the link cut in the
hover (the vehicle lands by itself, V1), the TELEM v2 columns of the log and the VTOL status block.
v0.7.1: the practice vehicle's NAV is the VTOL altitude filter (pad bias calibration, lagged thr_q) and the FCU stack gets
the NAV quaternion and the body-z specific force, so the touchdown detector v2 confirms the landing (no V10 timeout);
``--profile demo`` flies the demo profile (hover 1.5 m, step 0.5 m) in practice and in the station's texts; the Korean
operator text of every VTOL reason code is shown (reason line, interlock message) and the status block stays within the
console width with those wide characters; the VTOL link timing equals ``sim.link_model.round_trip(cfg, mode='vtol')``.
v0.8: the VTOL tests above run the build without GPS (``gps=False``: env fcu_nogps, TELEM v2 - unchanged behaviour); the
GPS build (params ``mission.vtol.position_hold.enabled``, the default) is covered by: a ``--sim`` hop through the NEO-7M class
receiver model, the NAV's GPS part and the FCU's position loop (ON refused here until the TELEM v3 shows the GPS ready,
then take-off, VEL -> POS hold, UP / DOWN, landing; the TELEM v3 log columns decode again from the raw bytes; the GPS
line of the status block; replay); the GPS lost in the hover (``sim gps off``: V11, DR -> ATT, LAND, SAFE); the ON
refusal with the Korean REJ_GPS text on a fake serial; the note when the vehicle's telemetry version differs from
``--gps``; the TELEM v3 link timing; the ``--gps`` option rules.
v0.8.1 (four chips): the practice GPS build runs the GNS chain (``sim.glink_proto`` GnsNode / GlinkNav, the FCU stack's
``gns`` chain: the hop above flies through it - glink counters, every latched RESULT's TILT part at the FCU); the GNS
health on the GPS line from TELEM v3 (OK / NO-CFG / SILENT / STALE, the events after the first OK, the plain line);
``sim gns off`` in the hover (the glink wire cut: the trim fallback ATT, the FCU's own GPS_LOST V11, LAND, SAFE, GNS STALE).

Covers: command repeat with the same SEQ until the telemetry's ack_seq confirms it (timeouts, a reply that
does not acknowledge yet, heartbeats afterwards); the nonce of the latest telemetry on every LAUNCH
transmission (fake serial and the real onboard supervisor); the station's refusals (stale link, state,
confirmation, throttle flag and cap, target range, quit while live); the assist rules through the real
``GroundAssist`` / ``OnboardSupervisor`` / ``LinkModel`` classes (auto-CUT on the predicted apogee - on a telemetry
frame or, v0.6.3, at a reply timeout on the extrapolated one -, auto-RELEASE after apogee, target change, assist off
-> the onboard I4 / I6 interlocks act); the CSV log (one row per
frame / command / ack / timeout, raw bytes that decode again, replay); a ``--sim`` practice run that reaches
apogee and releases without any interlock; the kill path (typed ``kill`` on the pad, Ctrl+C = SIGINT in
flight, double Ctrl+C quits after the KILL is acknowledged); resynchronisation on a serial stream with garbage,
split / corrupted / foreign frames and the bridge's ASCII error line (fake serial object); pyserial's loop://
port through the real ``SerialTransport``; and ``firmware/ground_bridge/src/bridge_config_gen.h`` being
current and equal to ``params.json``.

Run: ``.venv/Scripts/python.exe -m pytest tools/test_ground_station.py -q -p no:cacheprovider``
"""
from __future__ import annotations

import contextlib
import io
import os
import re
import signal
import sys
from collections import deque

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
for _p in (HERE, ROOT, os.path.join(ROOT, "firmware", "ground_bridge")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import ground_station as gsm  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim.onboard import State  # noqa: E402

P = gsm.load_params()
VID = int(P["flight_computer"]["link"]["vehicle_id"])
M = P["mission"]


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def tlm(seq: int, *, state: int = lp.STATE_SAFE, ack: int = 0, t_ms: int = 0, nonce: int = 0x1111, h: float = 0.0,
        vz: float = 0.0, flags: int = lp.FLAG_LINK_OK | lp.FLAG_SENSORS_OK | lp.FLAG_ALIGNED | lp.FLAG_ARM_SWITCH,
        reason: int = 0, veh: int = VID) -> bytes:
    t = lp.Telemetry.from_physical(state=state, flags=flags, reason=reason, ack_seq=ack, nonce=nonce, t_ms=t_ms, h_m=h,
                                   vz_mps=vz, roll_deg=0.1, pitch_deg=-0.2, yaw_deg=3.0, tilt_deg=0.25, throttle=0.0,
                                   vcell_v=4.1, loop_max_us=3000.0)
    return lp.encode_telemetry(veh, seq, t)


def uplink(frame: bytes) -> tuple[int, lp.Command]:
    (fr,) = lp.FrameDecoder(VID, (lp.TYPE_CMD,)).feed(frame)
    return fr.seq, lp.decode_command(fr)


class FakeSerial:
    """pyserial-like object: ``chunks`` are returned one per read; ``on_write(frame)`` may queue a reply."""

    def __init__(self, chunks=(), on_write=None):
        self.chunks: deque[bytes] = deque(chunks)
        self.written: list[bytes] = []
        self.on_write = on_write

    @property
    def in_waiting(self) -> int:
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, n: int) -> bytes:
        c = self.chunks.popleft()
        assert len(c) == n
        return c

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        if self.on_write:
            r = self.on_write(bytes(data))
            if r:
                self.chunks.append(r)
        return len(data)

    def close(self) -> None:
        pass


def make_station(tmp_path, ser: FakeSerial, *, assist: bool = True, allow_throttle: bool = False, radio=None,
                 mode: str = "ballistic", gps: bool | None = None):
    log = gsm.CsvLog(str(tmp_path / "fake.csv"))
    gps = False if mode == "ballistic" else gps
    st = gsm.GroundStation(P, gsm.SerialTransport("fake", 9600, ser=ser), gsm.ground_params(P, radio, mode=mode, gps=gps),
                           assist=assist, log=log, allow_throttle=allow_throttle, gps=gps)
    return st, log


def rows_of(log: gsm.CsvLog, kind: str | None = None) -> list[dict]:
    log.f.flush()
    rows = gsm.read_log(log.path)
    return [r for r in rows if kind is None or r["kind"] == kind]


def fast_clock(step: float = gsm.FAST_STEP_S):
    t = [0.0]

    def clock() -> float:
        t[0] += step
        return t[0]
    return clock


def practice(tmp_path, name: str, *, radio=None, loss=None, seed: int = 1, script: str = "3:arm,6:launch,60:quit",
             assist: bool = True, target=None, on_step=None, max_time: float = 120.0, mode: str = "ballistic",
             profile: str | None = None, gps: bool | None = None):
    p = P
    vv = gsm.VirtualVehicle(radio, loss, seed, mode=mode, profile=profile, gps=gps)
    log = gsm.CsvLog(str(tmp_path / f"{name}.csv"))
    st = gsm.GroundStation(p, gsm.SimTransport(vv), gsm.ground_params(p, radio, target, mode=mode, gps=gps), assist=assist,
                           log=log, profile=profile, gps=gps)
    console = gsm.Console(want_ui=False, quiet=True)
    gsm.run_live(st, console, fast_clock(), fast=True, script=gsm.parse_script(script), max_time_s=max_time,
                 interactive=False, on_step=on_step)
    log.close()
    return st, vv, gsm.read_log(log.path)


@pytest.fixture(scope="module")
def flight(tmp_path_factory):
    """The default practice flight (lora_uart, 5 % loss, seed 1, assist on), shared by several tests."""
    return practice(tmp_path_factory.mktemp("flight"), "nominal")


# --------------------------------------------------------------------------- #
# command repeat until ack, nonce, refusals (fake serial)
# --------------------------------------------------------------------------- #
def test_command_repeated_with_same_seq_until_ack(tmp_path):
    vehicle = {"ack": 10, "state": lp.STATE_SAFE, "t_ms": 0, "drop": False}

    def answer(frame: bytes):
        seq, _ = uplink(frame)
        if vehicle["drop"]:
            return None
        vehicle["t_ms"] += 100
        return tlm(seq, state=vehicle["state"], ack=vehicle["ack"], t_ms=vehicle["t_ms"])

    ser = FakeSerial(on_write=answer)
    st, log = make_station(tmp_path, ser)
    to = max(st.gs.gp.reply_timeout_s, 1.0 / st.gs.gp.poll_hz)  # a repeat goes out at the timeout or the next poll slot
    st.step(0.0)                                         # heartbeat; reply queued
    st.step(0.05)                                        # reply read: synced on ack_seq 10
    assert st.gs.synced and st.gs.seq == 10
    assert st.request(gsm.Action("arm"), 0.05) == "ARM queued"
    vehicle["drop"] = True                               # the first ARM is lost
    st.step(0.06)
    vehicle["drop"] = False                              # the reply to the second one says: not executed yet (ack 10)
    st.step(0.06 + to + 0.01)                            # timeout -> repeated, same SEQ
    st.step(0.06 + to + 0.05)
    assert st.gs.pending is not None and st.gs.pending.name == "ARM"
    t3 = 0.06 + to + 0.01 + 1.0 / st.gs.gp.poll_hz + 0.01
    vehicle.update(ack=11, state=lp.STATE_ARMED)         # third transmission executes
    st.step(t3)
    st.step(t3 + 0.05)
    assert st.gs.pending is None and st.gs.records[0].result == "accepted"
    st.step(t3 + 0.05 + 1.0 / st.gs.gp.poll_hz)          # afterwards: heartbeat repeating the last SEQ
    sent = [uplink(f) for f in ser.written]
    assert [(s, c.name) for s, c in sent] == [(0, "NONE"), (11, "ARM"), (11, "ARM"), (11, "ARM"), (11, "NONE")]
    ups = rows_of(log, "UP")
    assert [r["cmd"] for r in ups] == ["NONE", "ARM", "ARM", "ARM", "NONE"]
    assert len(rows_of(log, "TMO")) == 1 and rows_of(log, "TMO")[0]["seq"] == "11"
    (ack,) = rows_of(log, "ACK")
    assert (ack["cmd"], ack["result"], ack["n_tx"], ack["seq"]) == ("ARM", "accepted", "3", "11")


def test_launch_nonce_is_the_latest_telemetry_nonce_on_every_transmission(tmp_path):
    vehicle = {"nonce": 0x1234, "ack": 4, "state": lp.STATE_ARMED, "t_ms": 0}

    def answer(frame: bytes):
        seq, _ = uplink(frame)
        vehicle["t_ms"] += 100
        return tlm(seq, state=vehicle["state"], ack=vehicle["ack"], t_ms=vehicle["t_ms"], nonce=vehicle["nonce"])

    ser = FakeSerial(on_write=answer)
    st, log = make_station(tmp_path, ser)
    st.step(0.0)
    st.step(0.05)
    assert st.confirmation_prompt(gsm.Action("launch")) is not None
    st.request(gsm.Action("launch"), 0.05)
    assert st.refused == "confirm" and not st.gs.records         # unconfirmed LAUNCH never queued
    st.request(gsm.Action("launch", confirmed=True), 0.05)
    assert st.refused == "" and st.gs.pending.name == "LAUNCH"
    vehicle["nonce"] = 0x2222                                    # the reply to the first LAUNCH carries a new nonce
    st.step(0.06)                                                # LAUNCH #1 with 0x1234
    st.step(0.1)                                                 # reply: ack 4 (not executed), nonce 0x2222
    st.step(0.06 + 1.0 / st.gs.gp.poll_hz + 0.01)                # LAUNCH #2 must carry 0x2222
    launches = [c for _, c in map(uplink, ser.written) if c.cmd == lp.CMD_LAUNCH]
    assert [c.nonce for c in launches] == [0x1234, 0x2222]
    assert [r["nonce"] for r in rows_of(log, "UP") if r["cmd"] == "LAUNCH"] == ["0x1234", "0x2222"]


def test_station_refusals(tmp_path):
    vehicle = {"state": lp.STATE_SAFE, "t_ms": 0, "ack": 0}

    def answer(frame: bytes):
        seq, _ = uplink(frame)
        vehicle["t_ms"] += 100
        return tlm(seq, state=vehicle["state"], ack=vehicle["ack"], t_ms=vehicle["t_ms"])

    ser = FakeSerial(on_write=answer)
    st, log = make_station(tmp_path, ser)
    st.request(gsm.Action("arm"), 0.0)
    assert st.refused == "stale" and "no telemetry yet" in st.messages[-1][1]
    st.step(0.0)
    st.step(0.05)
    st.request(gsm.Action("launch", confirmed=True), 0.05)
    assert st.refused == "state"                                  # SAFE, not ARMED
    st.request(gsm.Action("throttle", 0.3), 0.05)
    assert st.refused == "throttle"                               # no --allow-throttle-cmd
    st.request(gsm.Action("target", float(M["apogee_limit_m"]) + 0.1), 0.05)
    assert st.refused == "range"
    st.request(gsm.Action("target", 0.0), 0.05)
    assert st.refused == "range"
    st.request(gsm.Action("target", 120.0), 0.05)
    assert st.refused == "" and st.gs.gp.target_altitude_m == 120.0
    st.request(gsm.Action("sim", False, sub="link"), 0.05)
    assert st.refused == "mode"
    late = 0.05 + float(M["link_loss_disarm_s"]) + 0.01            # telemetry now too old
    st.request(gsm.Action("arm"), late)
    assert st.refused == "stale" and not st.gs.records
    st.request(gsm.Action("disarm"), late)                        # reduces energy: always queued
    st.request(gsm.Action("kill"), late)
    assert st.refused == "" and [r.name for r in st.gs.records] == ["DISARM", "KILL"]
    assert st.gs.records[0].result == "superseded"                # KILL pre-empts
    # quit while the vehicle is live needs a confirmation
    vehicle["state"] = lp.STATE_ARMED
    st.step(late + 0.01)
    st.step(late + 0.05)
    assert st.gs.latest.state == lp.STATE_ARMED
    st.request(gsm.Action("quit"), late + 0.05)
    assert st.refused == "confirm" and not st.quit_requested
    st.request(gsm.Action("quit", confirmed=True), late + 0.05)
    assert st.quit_requested
    msgs = [r["note"] for r in rows_of(log, "MSG")]
    assert any("ARM refused" in m for m in msgs) and any("target refused" in m for m in msgs)


def test_throttle_flag_cap_and_confirmation(tmp_path):
    def answer(frame: bytes):
        seq, _ = uplink(frame)
        return tlm(seq, state=lp.STATE_ARMED, ack=0, t_ms=100 + seq, nonce=0x0BAD)

    ser = FakeSerial(on_write=answer)
    st, _ = make_station(tmp_path, ser, allow_throttle=True)
    st.step(0.0)
    st.step(0.05)
    climb = float(M["climb_throttle"])
    big = gsm.Action("throttle", climb + 0.5)
    assert st.confirmation_prompt(big) is not None                # motor start from ARMED
    st.request(big, 0.05)
    assert st.refused == "confirm"
    msg = st.request(gsm.Action("throttle", climb + 0.5, confirmed=True), 0.05)
    assert "capped" in msg and st.gs.pending.arg == lp.quantize(climb, lp.THROTTLE_SCALE, lp.U16)
    st.step(0.06)
    (c,) = [c for _, c in map(uplink, ser.written) if c.cmd == lp.CMD_THROTTLE]
    assert c.nonce == 0x0BAD and c.arg == round(climb * lp.THROTTLE_SCALE)


def test_parse_command_and_script():
    A = gsm.Action
    assert gsm.parse_command("  ARM ") == A("arm")
    assert gsm.parse_command("throttle 0.35") == A("throttle", 0.35)
    assert gsm.parse_command("assist off") == A("assist", False)
    assert gsm.parse_command("target 120") == A("target", 120.0)
    assert gsm.parse_command("sim link off") == A("sim", False, sub="link")
    assert gsm.parse_command("sim switch closed") == A("sim", True, sub="switch")
    assert gsm.parse_command("q") == A("quit") and gsm.parse_command("") is None
    for bad in ("throttle", "throttle -1", "throttle nan", "throttle x", "assist maybe", "target", "fly", "arm now",
                "sim link maybe"):
        with pytest.raises(ValueError):
            gsm.parse_command(bad)
    sc = gsm.parse_script("6:launch, 1:arm ,4:target 100")
    assert [(t, a.name) for t, a in sc] == [(1.0, "arm"), (4.0, "target"), (6.0, "launch")]
    assert all(a.confirmed for _, a in sc)


def test_cli_rules():
    with pytest.raises(SystemExit):
        gsm.main(["--port", "COM99", "--script", "1:arm"])          # scripts only in practice
    with pytest.raises(SystemExit):
        gsm.main(["--port", "COM99", "--fast"])
    with pytest.raises(SystemExit):
        gsm.main(["--port", "COM99", "--no-log"])                  # a real link is always logged
    with pytest.raises(SystemExit):
        gsm.main(["--sim", "--target", str(float(M["apogee_limit_m"]) + 1)])
    with pytest.raises(SystemExit):
        gsm.main(["--port", "COM99", "--loss", "0.1"])             # practice-only option
    for opt in (["--target", "100"], ["--allow-throttle-cmd"], ["--no-assist"]):
        with pytest.raises(SystemExit):
            gsm.main(["--sim", "--mode", "vtol", *opt])            # ballistic-only options
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        assert gsm.main([]) == 2                                   # no source given: usage + port list


# --------------------------------------------------------------------------- #
# serial stream: garbage, resync, the real SerialTransport
# --------------------------------------------------------------------------- #
def test_garbage_resync_on_the_serial_stream(tmp_path):
    a = tlm(1, t_ms=100, h=1.0)
    b = tlm(2, t_ms=200, h=2.0)
    c = tlm(3, t_ms=300, h=3.0)
    d = tlm(4, t_ms=400, h=4.0)
    bad = bytearray(tlm(9, t_ms=250))
    bad[12] ^= 0x10                                                # CRC mismatch
    foreign = tlm(5, t_ms=500, veh=(VID + 1) & 0xFF)               # valid CRC, other vehicle
    cmd = lp.encode_command(VID, 7, lp.Command(lp.CMD_ARM))        # valid CRC, wrong TYPE for the ground
    fake_hdr = bytes([lp.SYNC0, lp.SYNC1, lp.LEN_MAX])            # swallows frame b, then fails its CRC
    chunks = [b"\x00\xff\x13\xa5garbage", b"BRIDGE: SX127x not found\r\n", a[:5], a[5:20], a[20:], bytes(bad),
              foreign + cmd, fake_hdr + b + bytes(range(40, 40 + lp.LEN_MAX)), c + b"\x5a\xa5\x00noise" + d]
    ser = FakeSerial(chunks)
    st, log = make_station(tmp_path, ser)
    got = []
    for i in range(len(chunks)):
        for t_arr, data in st.transport.read(0.1 * i):
            got += st._receive(data, t_arr)
    assert [t.h_m for t in got] == [1.0, 2.0, 3.0, 4.0]
    s = st.tap.stats
    assert s["frames"] == 4 and s["wrong_vehicle"] == 1 and s["wrong_type"] == 1 and s["bad_crc"] >= 2
    assert s["skipped_bytes"] > len(b"BRIDGE: SX127x not found\r\n")
    rows = rows_of(log, "TLM")
    assert [r["hex"] for r in rows] == [x.hex() for x in (a, b, c, d)]
    assert [float(r["h_m"]) for r in rows] == [1.0, 2.0, 3.0, 4.0]


def test_serial_transport_on_pyserial_loopback(tmp_path):
    pytest.importorskip("serial")
    tr = gsm.SerialTransport("loop://", 9600)
    log = gsm.CsvLog(str(tmp_path / "loop.csv"))
    st = gsm.GroundStation(P, tr, gsm.ground_params(P, None, mode="ballistic"), log=log)
    st.step(0.0)                                                   # writes a heartbeat, which loops back
    st.step(0.01)
    assert st.tap.stats["wrong_type"] == 1 and st.gs.latest is None   # its own CMD frame: dropped by TYPE
    tr.close()
    log.close()


# --------------------------------------------------------------------------- #
# practice mode: the real supervisor, link model and assist logic
# --------------------------------------------------------------------------- #
def test_sim_practice_run_reaches_apogee_and_releases_without_interlock(flight):
    st, vv, rows = flight
    s = gsm.summarize_log(rows)
    assert s["states"] == ["SAFE", "ARMED", "POWERED", "UNPOWERED", "RELEASED"]
    assert s["interlocks"] == [] and not s["interlock_flag"]
    assert [(c["cmd"], c["source"], c["result"]) for c in s["commands"]] == [
        ("ARM", "operator", "accepted"), ("LAUNCH", "operator", "accepted"), ("CUT", "assist", "accepted"),
        ("RELEASE", "assist", "accepted")]
    tr = vv.truth()
    assert tr["t_liftoff_s"] is not None and tr["t_apogee_s"] is not None
    # SPEC 8.8 v0.4 predictive release: the nose opens centred on apogee (within half a telemetry interval plus
    # one control-loop quantum), before touchdown
    assert abs(tr["t_release_s"] - tr["t_apogee_s"]) <= 0.5 and tr["t_release_s"] < tr["t_touchdown_s"]
    assert vv.sup.state == State.RELEASED and vv.sup.reason == lp.REASON_NONE
    assert not any(e["code"] for e in vv.sup.exec_log)                      # every command executed
    assert tr["h_max_m"] > 0.8 * float(M["target_altitude_m"])


@pytest.mark.parametrize("radio", ["sx127x"])
def test_sim_practice_run_other_radio(tmp_path, radio):
    st, vv, rows = practice(tmp_path, radio, radio=radio)
    s = gsm.summarize_log(rows)
    assert s["states"][-1] == "RELEASED" and s["interlocks"] == [] and not s["interlock_flag"]
    assert s["rtt_mean_s"] < gsm.link_timing(P, "lora_uart")["t_link_default_s"]


def test_csv_log_has_every_frame_command_and_ack(flight, tmp_path):
    st, vv, rows = flight
    k = {}
    for r in rows:
        k[r["kind"]] = k.get(r["kind"], 0) + 1
    gs = st.gs
    assert k["TLM"] == st.tap.stats["frames"]                       # every decoded telemetry frame
    assert k["UP"] == gs.stats["polls"]                             # every uplink frame (heartbeats too)
    assert k.get("TMO", 0) == gs.stats["timeouts"]
    assert k["CMD"] == len(gs.records) and k["ACK"] == sum(1 for r in gs.records if r.result)
    assert set(r["kind"] for r in rows) <= {"TLM", "UP", "TMO", "CMD", "ACK", "EVT", "MSG"}
    dec = lp.FrameDecoder(VID, (lp.TYPE_TELEM,))
    for r in rows:
        if r["kind"] == "TLM":                                        # the raw bytes decode to the logged values
            (fr,) = dec.feed(bytes.fromhex(r["hex"]))
            t = lp.decode_telemetry(fr)
            assert (t.state_name, t.t_ms, t.h_m, t.vz_mps) == (r["state"], int(r["t_ms"]), float(r["h_m"]), float(r["vz_mps"]))
    up = [r for r in rows if r["kind"] == "UP"]
    assert all(lp.FrameDecoder(VID, (lp.TYPE_CMD,)).feed(bytes.fromhex(r["hex"])) for r in up)
    # replay decodes the same bytes again and tells the same story
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        con = gsm.Console(want_ui=False, quiet=True)
        s = gsm.run_replay(st.log.path, P, con, speed=1.0, fast=True)
    assert s == gsm.summarize_log(rows)
    assert "INTERLOCK" not in buf.getvalue() and "RELEASED" in buf.getvalue()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        assert gsm.main(["--replay", st.log.path, "--fast", "--plain", "--quiet"]) == 0
    assert "states: SAFE -> ARMED -> POWERED -> UNPOWERED -> RELEASED" in buf.getvalue()


def test_launch_nonce_through_the_real_supervisor(tmp_path):
    # 30 % loss: the operator repeats ARM once (the vehicle refuses ARM after 2 s without a frame, which seed 4 hits)
    st, vv, rows = practice(tmp_path, "lossy", loss=0.3, seed=4, script="3:arm,4.5:arm,6:launch,25:quit")
    latest_nonce = None
    n_launch = 0
    for r in rows:
        if r["kind"] == "TLM" and r["note"] != "old":
            latest_nonce = r["nonce"]
        if r["kind"] == "UP" and r["cmd"] == "LAUNCH":
            n_launch += 1
            assert r["nonce"] == latest_nonce
    assert n_launch >= 1
    launches = [e for e in vv.sup.exec_log if e["cmd"] == lp.CMD_LAUNCH]
    assert len(launches) == 1 and launches[0]["code"] == 0              # executed exactly once
    tl = [r for r in rows if r["kind"] == "TLM" and r["note"] != "old"]
    before = [r["nonce"] for r in tl if r["state"] == "ARMED"][-1]
    after = [r["nonce"] for r in tl if r["state"] == "POWERED"][0]
    assert before != after                                              # a new nonce after the executed LAUNCH


def test_assist_cut_rule_uses_the_reference_prediction(flight):
    st, vv, rows = flight
    gs = st.gs
    (cut,) = [a for a in gs.assist_log if a["cmd"] == "CUT"]
    p_tel = cut.get("predicted_at_telemetry_m", cut["predicted_m"])           # P of the telemetry the decision used
    assert p_tel == pytest.approx(gs.predicted_apogee(cut["h_m"], cut["vz_mps"], cut["t_link_s"]))
    target = float(M["target_altitude_m"])
    powered = [r for r in rows if r["kind"] == "TLM" and r["note"] != "old" and r["state"] == "POWERED"]
    # the decision on telemetry: P + max(dP/dt, 0) T_tel / 2 >= target on the first telemetry where it holds
    t_tel = gs.t_tel()
    dec = []
    for a, b in zip(powered, powered[1:]):
        rate = max((float(b["pred_apogee_m"]) - float(a["pred_apogee_m"])) / ((int(b["t_ms"]) - int(a["t_ms"])) / 1000.0), 0.0)
        dec.append((int(b["t_ms"]), float(b["pred_apogee_m"]) + 0.5 * rate * t_tel))
    trig = [t for t, d in dec if d >= target - 0.2]                       # 0.2 m: the log rounds P to 0.1 m
    if cut.get("extrapolated"):
        # v0.6.3: the reply after this telemetry was lost - the same rule ran at the reply timeout on P extrapolated
        # by the telemetry's age, before any telemetry met it
        assert not [t for t in trig if t <= cut["t_ms"]]
        assert any(r["kind"] == "TMO" and float(r["t_gs_s"]) == pytest.approx(cut["t"], abs=1e-6) for r in rows)
        assert cut["predicted_m"] == pytest.approx(p_tel + cut["rate_mps"] * cut["age_s"])
        assert cut["predicted_m"] + cut["lead_m"] >= target > p_tel + cut["lead_m"]
    else:
        assert trig and trig[0] == cut["t_ms"] and cut["predicted_m"] + cut["lead_m"] >= target
    assert cut["lead_m"] == pytest.approx(0.5 * cut["rate_mps"] * t_tel)
    # SPEC 8.8 v0.4 predictive RELEASE: v_z predicted to the servo-opening time <= apogee_detect_vz (at a reply
    # timeout: also extrapolated by the telemetry's age)
    (rel,) = [a for a in gs.assist_log if a["cmd"] == "RELEASE"]
    assert rel["why"] == "apogee" and gs.gp.release_compensated
    lead = gs.t_link + float(M["release_after_cut_s"]) + 0.5 * t_tel + rel.get("age_s", 0.0)
    assert rel["vz_mps"] - gs.gp.g * lead <= float(M["apogee_detect_vz_mps"]) + 0.05


def test_target_command_lowers_the_cut(flight, tmp_path):
    _, vv0, _ = flight
    st, vv, rows = practice(tmp_path, "target100", script="3:arm,4:target 100,6:launch,60:quit")
    (cut,) = [a for a in st.gs.assist_log if a["cmd"] == "CUT"]
    assert st.gs.gp.target_altitude_m == 100.0 and 100.0 <= cut["predicted_m"] + cut["lead_m"] < 120.0   # SPEC 8.8 v0.4
    assert [r.result for r in st.gs.records if r.name == "CUT"] == ["accepted"]      # the assist cut, not I4
    assert vv.truth()["h_max_m"] < vv0.truth()["h_max_m"] - 20.0
    # the release: assist RELEASE or, when it loses the race on the slow lora_uart link, the onboard I6 fall-safe
    # (this seed: I6 first - see the run report); either way the vehicle ends RELEASED
    assert vv.sup.state == State.RELEASED and gsm.summarize_log(rows)["states"][-1] == "RELEASED"


def test_assist_off_leaves_it_to_the_onboard_interlocks(tmp_path):
    st, vv, rows = practice(tmp_path, "noassist", assist=False)
    s = gsm.summarize_log(rows)
    assert not st.gs.assist_log and all(r.source == "operator" for r in st.gs.records)
    assert s["interlocks"] == ["I4 apogee limiter", "I6 fall-safe"]   # onboard: CUT at the limiter, then fall-safe
    assert s["states"][-1] == "RELEASED" and vv.sup.state == State.RELEASED


def test_assist_off_mid_flight_then_manual_cut(tmp_path):
    st, vv, rows = practice(tmp_path, "manual", script="3:arm,6:launch,7:assist off,12:cut,60:quit")
    s = gsm.summarize_log(rows)
    cuts = [c for c in s["commands"] if c["cmd"] == "CUT"]
    assert [(c["source"], c["result"]) for c in cuts] == [("operator", "accepted")]
    assert not any(a["cmd"] == "CUT" for a in st.gs.assist_log)


# --------------------------------------------------------------------------- #
# kill path
# --------------------------------------------------------------------------- #
def test_kill_on_the_pad_disarms(tmp_path):
    st, vv, rows = practice(tmp_path, "padkill", script="3:arm,5:kill,8:quit")
    s = gsm.summarize_log(rows)
    assert s["states"] == ["SAFE", "ARMED", "SAFE"]
    assert [(c["cmd"], c["result"]) for c in s["commands"]] == [("ARM", "accepted"), ("KILL", "accepted")]
    assert vv.truth()["t_liftoff_s"] is None


@pytest.mark.skipif(not hasattr(signal, "raise_signal"), reason="needs signal.raise_signal")
def test_ctrl_c_in_flight_kills_then_double_ctrl_c_quits(tmp_path):
    presses = []

    def on_step(now: float) -> None:
        if now >= 10.0 and len(presses) == 0:
            presses.append(now)
            signal.raise_signal(signal.SIGINT)           # Ctrl+C -> KILL
        elif now >= 10.5 and len(presses) == 1:
            presses.append(now)
            signal.raise_signal(signal.SIGINT)           # second Ctrl+C within 2 s -> quit once the KILL is acked
    st, vv, rows = practice(tmp_path, "ctrlc", script="3:arm,6:launch", on_step=on_step, max_time=60.0)
    kills = [r for r in st.gs.records if r.cmd == lp.CMD_KILL]
    assert len(kills) >= 1 and kills[0].source == "operator" and kills[0].t_queued == pytest.approx(10.0, abs=0.01)
    assert any(r.result == "accepted" for r in kills)
    assert st.gs.records[-1].cmd == lp.CMD_KILL
    t_end = float(rows[-1]["t_gs_s"])
    assert t_end < 10.5 + gsm.QUIT_FLUSH_S + 0.01                      # quit, did not run to max_time
    assert vv.sup.exec_log[-1]["cmd"] == lp.CMD_KILL and vv.sup.exec_log[-1]["state_after"] == int(State.ABORT)
    vv.advance(t_end + 2.0)                                             # link gone: the vehicle finishes its ABORT
    assert vv.sup.state == State.RELEASED
    assert signal.getsignal(signal.SIGINT) is signal.default_int_handler   # handler restored


def test_practice_link_off_triggers_onboard_link_loss(tmp_path):
    st, vv, rows = practice(tmp_path, "linkoff", script="3:arm,4:sim link off,9:sim link on,14:quit")
    assert vv.sup.reason == lp.R_I1P_LINK_LOSS_PAD or any(
        "I1p" in r["reason"] for r in rows if r["kind"] == "TLM")
    s = gsm.summarize_log(rows)
    assert s["states"][:2] == ["SAFE", "ARMED"] and s["states"][-1] == "SAFE"


# --------------------------------------------------------------------------- #
# display
# --------------------------------------------------------------------------- #
def test_status_block_fits_and_shows_the_fields(flight):
    st, _, _ = flight
    lines = st.status_lines(100.0, 100)
    assert all(len(ln) <= 99 for ln in lines)
    text = "\n".join(st.status_lines(100.0, 400))
    for word in ("STATE", "RELEASED", "flags", "v_z", "tilt", "roll", "pitch", "yaw", "throttle", "V_cell", "delay",
                 "loss", "reason", "ack"):
        assert word in text
    assert "RELEASED" in st.plain_line(100.0)


def test_status_block_is_redrawn_in_place(flight, monkeypatch):
    st, _, _ = flight
    out = io.StringIO()
    monkeypatch.setattr(gsm, "_enable_vt", lambda: True)
    monkeypatch.setattr(gsm.Console, "_size", staticmethod(lambda: (120, 40)))
    monkeypatch.setattr(sys, "stdout", out)
    con = gsm.Console(want_ui=True)
    assert con.ui
    fresh = gsm.GroundStation(P, gsm.NullTransport(), gsm.ground_params(P, None, mode="ballistic"))
    n = len(st.status_lines(100.0, 120))
    assert len(fresh.status_lines(0.0, 120)) == n                 # same height before the first telemetry
    con.draw(fresh, 0.0)
    con.draw(st, 100.0)
    con.draw(st, 100.25)
    text = out.getvalue()
    assert text.count("\x1b[2J") == 1                             # screen cleared once, at the layout
    assert text.count(f"\x1b[{n + 1};40r") == 1                  # scroll region below the block, set once
    assert text.count("\x1b7") == 3 and text.count("\x1b8") == 3  # cursor saved / restored around each redraw
    for i in range(1, n + 1):
        assert text.count(f"\x1b[{i};1H\x1b[2K") == 3           # every block line rewritten in place
    con.say("KILL queued")
    assert out.getvalue().endswith("\r\x1b[2KKILL queued\n" + gsm.PROMPT)   # message above a fresh prompt
    con.close()
    assert out.getvalue().endswith("\x1b[r\x1b[40;1H\n") and not con.ui   # scroll region reset on exit


def test_interactive_input_thread_with_piped_stdin(tmp_path):
    """The prompt thread on a pipe: commands are read, 'launch' asks its question (answered 'no' -> cancelled),
    'quit' ends the run.  Real time at --speed 5."""
    import subprocess
    log = tmp_path / "piped.csv"
    r = subprocess.run([sys.executable, os.path.join(HERE, "ground_station.py"), "--sim", "--mode", "ballistic", "--plain",
                        "--quiet", "--speed", "5", "--log", str(log)], input="status\nlaunch\nno\nbogus\nquit\n", text=True,
                       encoding="utf-8", errors="replace", capture_output=True, timeout=120)
    assert r.returncode == 0, r.stderr
    assert "launch: cancelled" in r.stdout and "unknown command 'bogus'" in r.stdout
    rows = gsm.read_log(str(log))
    assert [x["note"] for x in rows if x["kind"] == "MSG"][-1] == "quit"
    assert not any(x["kind"] == "CMD" for x in rows)               # nothing was sent but heartbeats
    # stdin from the NUL device (isatty() is True on Windows, every read is EOF): the prompt thread gives up
    r = subprocess.run([sys.executable, os.path.join(HERE, "ground_station.py"), "--sim", "--mode", "ballistic", "--plain",
                        "--quiet", "--no-log", "--speed", "5", "--script", "15:quit"], stdin=subprocess.DEVNULL, text=True,
                       encoding="utf-8", errors="replace", capture_output=True, timeout=120)
    assert r.returncode == 0, r.stderr
    assert r.stdout.count(gsm.PROMPT) <= gsm.EOF_GIVE_UP and "operator input closed" in r.stdout


# --------------------------------------------------------------------------- #
# SX127x bridge header
# --------------------------------------------------------------------------- #
def test_bridge_header_is_current_and_matches_params():
    import gen_bridge_config as gbc
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        assert gbc.main(["--check"]) == 0, buf.getvalue()
    with open(gbc.OUT, encoding="utf-8") as f:
        text = f.read()
    d = {k: v for k, v in re.findall(r"#define\s+(\w+)\s+(\S+)", text)}
    num = lambda s: int(s.rstrip("UL"), 0)  # noqa: E731
    sx, lk = P["flight_computer"]["link"]["sx127x"], P["flight_computer"]["link"]
    assert num(d["BRIDGE_LORA_FREQ_HZ"]) == sx["freq_hz"]
    assert num(d["BRIDGE_LORA_SF"]) == sx["spreading_factor"]
    assert num(d["BRIDGE_LORA_BW_HZ"]) == sx["bandwidth_hz"]
    assert num(d["BRIDGE_LORA_CR_DENOM"]) == sx["coding_rate_denom"]
    assert num(d["BRIDGE_LORA_SYNC_WORD"]) == sx["sync_word"]
    assert num(d["BRIDGE_LORA_TX_POWER_DBM"]) == sx["tx_power_dbm"]
    assert num(d["BRIDGE_LORA_PREAMBLE"]) == sx["preamble_symbols"]
    assert num(d["BRIDGE_SERIAL_BAUD"]) == lk["uart_baud"]
    assert (num(d["LINK_SYNC0"]), num(d["LINK_SYNC1"]), num(d["LINK_LEN_MAX"])) == (lp.SYNC0, lp.SYNC1, lp.LEN_MAX)
    # v0.7: LEN_MAX 34 (TELEM v2); v0.8: LEN_MAX 40 (TELEM3, 45 B); v0.9 RP1: LEN_MAX 42 - the largest frame is a TELEM5
    # (47 B); the bridge's buffer holds it
    assert num(d["LINK_MAX_FRAME_BYTES"]) == lp.FRAME_OVERHEAD + lp.LEN_MAX == lp.TELEM5_FRAME_BYTES
    bad = {**P, "flight_computer": {**P["flight_computer"], "link": {**lk, "sx127x": {**sx, "bandwidth_hz": 100000}}}}
    with pytest.raises(SystemExit):
        gbc.render(bad)                                                 # not an SX127x bandwidth
    with open(os.path.join(ROOT, "firmware", "ground_bridge", "src", "main.cpp"), encoding="utf-8") as f:
        src = f.read()
    code = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    assert not re.search(r"(?<![\w.])\d{3,}(?![\w.])", code)            # no typed numbers in the bridge source


# --------------------------------------------------------------------------- #
# v0.7 VTOL build: ON / OFF / UP / DOWN / KILL
# --------------------------------------------------------------------------- #
V = M["vtol"]


def tlm2(seq: int, *, state: int = lp.STATE_SAFE, phase: int = lp.PHASE_NONE, tgt: float = 0.0, ack: int = 0,
         t_ms: int = 0, nonce: int = 0x1111, h: float = 0.0, reason: int = 0,
         flags: int = lp.FLAG_LINK_OK | lp.FLAG_SENSORS_OK | lp.FLAG_ALIGNED | lp.FLAG_ARM_SWITCH) -> bytes:
    t = lp.Telemetry.from_physical(state=state, flags=flags, reason=reason, ack_seq=ack, nonce=nonce, t_ms=t_ms, h_m=h,
                                   vz_mps=0.0, roll_deg=0.1, pitch_deg=-0.2, yaw_deg=3.0, tilt_deg=0.25, throttle=0.0,
                                   vcell_v=4.1, loop_max_us=3000.0, phase=phase, h_tgt_m=tgt)
    return lp.encode_telemetry(VID, seq, t)


@pytest.fixture(scope="module")
def vtol_hop(tmp_path_factory):
    """The default VTOL practice hop (lora_uart, 5 % loss, seed 1): ON, UP, DOWN, OFF, quit after LANDED -> SAFE."""
    return practice(tmp_path_factory.mktemp("vtol"), "vtol_hop", mode="vtol", script="3:on,15:up,20:down,25:off,55:quit",
                    max_time=120.0, gps=False)


def test_vtol_practice_hop_sends_the_logic_battery(vtol_hop):
    """2026-11-10: the practice vehicle is the carrier battery build: its replies are TELEM v6 (no GPS here) with the
    logic-battery byte (sim.sim6dof SIM_LOGIC_BAT_V_ASSUMED), the station keeps it, shows it and does not warn above
    the params threshold; the reply timeout is the ESP model's for that frame."""
    st, vv, rows = vtol_hop
    from sim.sim6dof import SIM_LOGIC_BAT_V_ASSUMED
    dec = lp.FrameDecoder(VID, lp.TELEM_TYPES)
    frs = [f for r in rows if r["kind"] == "TLM" for f in dec.feed(bytes.fromhex(r["hex"]))]
    assert frs and all(f.type == lp.TYPE_TELEM6 for f in frs)
    assert all(lp.decode_telemetry(f).lbat_q == lp.lbat_quant(SIM_LOGIC_BAT_V_ASSUMED) for f in frs)
    assert st.gs.lbat_v == pytest.approx(lp.lbat_quant(SIM_LOGIC_BAT_V_ASSUMED) / lp.LBAT_SCALE) and not st.gs.lbat_low
    assert any("logic battery 3.86 V" in e for _, e in st.gs.events)
    assert "logic 3.86 V" in " ".join(st.status_lines(60.0, 200))
    lt = gsm.link_timing(P, None, "vtol", False)
    assert lt["radio"] == "esp_bridge" and lt["lbat_tail"] and st.gs.gp.reply_timeout_s == lt["reply_timeout_s"]


def test_vtol_practice_hop_takes_off_holds_and_lands(vtol_hop):
    st, vv, rows = vtol_hop
    s = gsm.summarize_log(rows)
    assert st.vtol and vv.vtol and s["states"] == ["SAFE", "ARMED", "POWERED", "UNPOWERED", "SAFE"]
    assert s["phases"] == ["NONE", "SPOOL", "CLIMB", "HOLD", "LAND", "LANDED", "NONE"]
    assert [(c["cmd"], c["result"]) for c in s["commands"]] == [("ON", "accepted"), ("UP", "accepted"),
                                                                ("DOWN", "accepted"), ("OFF", "accepted")]
    assert s["interlocks"] == [] and not s["interlock_flag"]
    tr = vv.truth()
    # the target went hover -> + step -> back; the plant (no wind, no ground effect) reached about hover + step
    hover, step = float(V["hover_altitude_m"]), float(V["alt_step_m"])
    targets = [float(r["h_tgt_m"]) for r in rows if r["kind"] == "TLM" and r["phase"] == "HOLD"]
    assert set(round(x, 1) for x in targets) == {hover, hover + step}
    assert hover + step - 0.5 < tr["h_max_m"] < hover + step + 1.0
    assert tr["t_liftoff_s"] is not None and 0.0 < tr["touchdown_vz_max_mps"] < 1.0      # a soft touchdown
    assert vv.sup.state.name == "SAFE" and tr["state"] == "SAFE" and tr["motor_s"] > 10.0
    # v0.7.1: the touchdown detector v2 confirmed the landing on the ground (not the V10 floor timeout, not in the air)
    assert vv.profile == "field" and tr["profile"] == "field" and st.profile == "field"
    assert vv.td_confirmed and all(by in ("still", "support", "floor") for _, by, _ in vv.td_confirmed)
    assert all(h_true <= 0.05 for _, _, h_true in vv.td_confirmed)
    # the TELEM v2 columns are logged and the raw bytes decode again to them
    dec = lp.FrameDecoder(VID, lp.TELEM_TYPES)
    for r in [r for r in rows if r["kind"] == "TLM"][::7]:
        (fr,) = dec.feed(bytes.fromhex(r["hex"]))
        t = lp.decode_telemetry(fr)
        assert t.is_v2 and (t.phase_name, round(t.h_tgt_m, 1)) == (r["phase"], round(float(r["h_tgt_m"]), 1))


@pytest.fixture(scope="module")
def vtol_demo_hop(tmp_path_factory):
    """v0.7.1 demo profile practice hop (hover 1.5 m, step 0.5 m): ON, UP, DOWN, OFF, quit after LANDED -> SAFE."""
    return practice(tmp_path_factory.mktemp("vtol_demo"), "vtol_demo_hop", mode="vtol", profile="demo",
                    script="3:on,13:up,18:down,23:off,45:quit", max_time=120.0, gps=False)


def test_vtol_demo_profile_practice_hop(vtol_demo_hop):
    """--profile demo: the stack flies the demo numbers (params mission.vtol.profiles.demo), every rule unchanged"""
    from sim.onboard import vtol_values
    st, vv, rows = vtol_demo_hop
    dv = vtol_values(P, "demo")
    hover, step = float(dv["hover_altitude_m"]), float(dv["alt_step_m"])
    assert hover != float(V["hover_altitude_m"])                     # the demo profile really differs from field
    assert vv.profile == "demo" and st.profile == "demo" and (st.hover_m, st.step_m) == (hover, step)
    assert "(demo profile)" in st.transport.desc
    s = gsm.summarize_log(rows)
    assert s["states"] == ["SAFE", "ARMED", "POWERED", "UNPOWERED", "SAFE"]
    assert s["phases"] == ["NONE", "SPOOL", "CLIMB", "HOLD", "LAND", "LANDED", "NONE"]
    assert [(c["cmd"], c["result"]) for c in s["commands"]] == [("ON", "accepted"), ("UP", "accepted"),
                                                                ("DOWN", "accepted"), ("OFF", "accepted")]
    assert s["interlocks"] == [] and not s["interlock_flag"]
    targets = [float(r["h_tgt_m"]) for r in rows if r["kind"] == "TLM" and r["phase"] == "HOLD"]
    assert set(round(x, 1) for x in targets) == {hover, hover + step}
    tr = vv.truth()
    assert hover + step - 0.3 < tr["h_max_m"] < hover + step + 0.5 and tr["h_max_m"] < float(dv["alt_max_m"])
    assert 0.0 < tr["touchdown_vz_max_mps"] < 1.0 and tr["state"] == "SAFE"
    assert vv.td_confirmed and all(h_true <= 0.05 for _, _, h_true in vv.td_confirmed)
    on = [m for _, m in st.messages if m.startswith("ON queued")]
    assert on and f"climb to {hover:.1f} m" in on[0]


def test_vtol_reason_texts_in_korean(tmp_path):
    """v0.7.1: the reason line and the interlock message carry the Korean operator text (sim.ground_station_logic
    REASON_TEXT_KO) for V8 / V9 / V10 and every other VTOL code; the status block stays inside the console width"""
    from sim.ground_station_logic import REASON_TEXT_KO, reason_text_ko
    assert {lp.R_V_BARO_LIMIT_LANDED, lp.R_V_BARO_LIMIT_ABORT, lp.R_V_LAND_TIMEOUT} <= set(REASON_TEXT_KO)
    ilock = lp.FLAG_LINK_OK | lp.FLAG_SENSORS_OK | lp.FLAG_ALIGNED | lp.FLAG_ARM_SWITCH | lp.FLAG_INTERLOCK
    specs = [dict(state=lp.STATE_POWERED, phase=lp.PHASE_LAND),
             dict(state=lp.STATE_UNPOWERED, phase=lp.PHASE_LANDED, reason=lp.R_V_LAND_TIMEOUT, flags=ilock)]
    n_tx = [0]

    def answer(frame: bytes):
        seq, _ = uplink(frame)
        n_tx[0] += 1
        return tlm2(seq, t_ms=100 * n_tx[0], **specs[min(n_tx[0] - 1, len(specs) - 1)])

    ser = FakeSerial(on_write=answer)
    st, _ = make_station(tmp_path, ser, mode="vtol", gps=False)
    for k in range(40):
        st.step(0.05 * k)
    ko = reason_text_ko(lp.R_V_LAND_TIMEOUT)
    assert any(m == f"INTERLOCK on the vehicle: {lp.REASON_NAMES[lp.R_V_LAND_TIMEOUT]} - {ko}" for _, m in st.messages)
    wide = st.status_lines(2.0, 400)
    assert any(ln.startswith("reason V10") and ln.endswith(ko) for ln in wide)
    for width in (80, 100, 120):
        for ln in st.status_lines(2.0, width):
            assert gsm.clip_cols(ln, width - 1) == ln                 # within width - 1 console columns
    assert gsm.clip_cols("abc", 2) == "ab" and gsm.clip_cols("가나다", 5) == "가나" and gsm.clip_cols("a가", 2) == "a"


def test_vtol_replay_and_status_block(vtol_hop):
    st, _, rows = vtol_hop
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        con = gsm.Console(want_ui=False, quiet=True)
        s = gsm.run_replay(st.log.path, P, con, speed=1.0, fast=True)
    assert s == gsm.summarize_log(rows)
    lines = st.status_lines(60.0, 120)
    text = "\n".join(lines)
    for word in ("STATE", "PHASE", "target", "h - target", "throttle", "V_cell", "VTOL commands"):
        assert word in text
    fresh = gsm.GroundStation(P, gsm.NullTransport(), gsm.ground_params(P, None, mode="vtol", gps=False), gps=False)
    assert len(fresh.status_lines(0.0, 120)) == len(lines)           # same block height before the first telemetry
    assert any(ln.startswith("GPS    none") for ln in lines)          # v0.8: the build without GPS says so
    assert "NONE" in st.plain_line(60.0) and "SAFE" in st.plain_line(60.0)


def test_vtol_station_refusals_and_confirmations(tmp_path):
    vehicle = {"state": lp.STATE_SAFE, "phase": lp.PHASE_NONE, "tgt": 0.0, "t_ms": 0, "ack": 0}

    def answer(frame: bytes):
        seq, _ = uplink(frame)
        vehicle["t_ms"] += 100
        return tlm2(seq, state=vehicle["state"], phase=vehicle["phase"], tgt=vehicle["tgt"], ack=vehicle["ack"],
                    t_ms=vehicle["t_ms"])

    ser = FakeSerial(on_write=answer)
    st, log = make_station(tmp_path, ser, mode="vtol", gps=False)
    assert st.vtol and not st.gs.assist                               # assist is forced off in VTOL
    st.request(gsm.Action("on", confirmed=True), 0.0)
    assert st.refused == "stale" and "no telemetry yet" in st.messages[-1][1]
    st.step(0.0)
    st.step(0.05)
    assert st.confirmation_prompt(gsm.Action("on")) is not None
    st.request(gsm.Action("on"), 0.05)
    assert st.refused == "confirm" and not st.gs.records              # unconfirmed ON never queued
    for name in ("arm", "launch", "cut", "release", "assist", "target", "throttle"):
        st.request(gsm.Action(name, 1.0 if name in ("throttle", "target") else None), 0.05)
        assert st.refused == "mode", name                              # the ballistic build's commands
    assert not st.gs.records
    # KILL needs a typed confirmation in VTOL (it drops the vehicle); OFF / DOWN are always queued
    assert "falls" in st.confirmation_prompt(gsm.Action("kill"))
    st.request(gsm.Action("kill"), 0.05)
    assert st.refused == "confirm" and not st.gs.records
    late = 0.05 + float(M["link_loss_disarm_s"]) + 0.01
    st.request(gsm.Action("up"), late)
    assert st.refused == "stale"                                       # UP adds energy: never sent later by surprise
    st.request(gsm.Action("down"), late)
    st.request(gsm.Action("off"), late)
    assert [r.name for r in st.gs.records] == ["DOWN", "OFF"] and st.gs.records[0].result == "superseded"
    st.request(gsm.Action("kill", confirmed=True), late)
    assert st.gs.records[-1].name == "KILL" and st.gs.records[1].result == "superseded"
    # ON only from SAFE
    vehicle.update(state=lp.STATE_POWERED, phase=lp.PHASE_HOLD, tgt=5.0)
    st.step(late + 1.0)
    st.step(late + 1.05)
    st.request(gsm.Action("on", confirmed=True), late + 1.05)
    assert st.refused == "state"
    assert "lands by itself" in st.confirmation_prompt(gsm.Action("quit"))
    st.request(gsm.Action("quit"), late + 1.05)
    assert st.refused == "confirm" and not st.quit_requested
    msgs = [r["note"] for r in rows_of(log, "MSG")]
    assert any("not a command of the VTOL vehicle" in m for m in msgs)
    # and the ballistic station refuses the VTOL buttons
    st_b, _ = make_station(tmp_path / "b", FakeSerial(), mode="ballistic")
    for name in ("on", "off", "up", "down"):
        st_b.request(gsm.Action(name, confirmed=True), 0.0)
        assert st_b.refused == "mode", name


def test_vtol_on_carries_the_latest_nonce(tmp_path):
    vehicle = {"nonce": 0x1234, "t_ms": 0}

    def answer(frame: bytes):
        seq, _ = uplink(frame)
        vehicle["t_ms"] += 100
        return tlm2(seq, state=lp.STATE_SAFE, ack=0, t_ms=vehicle["t_ms"], nonce=vehicle["nonce"])

    ser = FakeSerial(on_write=answer)
    st, log = make_station(tmp_path, ser, mode="vtol", gps=False)
    st.step(0.0)
    st.step(0.05)
    st.request(gsm.Action("on", confirmed=True), 0.05)
    vehicle["nonce"] = 0x2222                                    # the reply to the first ON (not executed) has a new nonce
    st.step(0.06)
    st.step(0.1)
    st.step(0.06 + 1.0 / st.gs.gp.poll_hz + 0.01)
    ons = [c for _, c in map(uplink, ser.written) if c.cmd == lp.CMD_ON]
    assert [c.nonce for c in ons] == [0x1234, 0x2222]


def test_vtol_link_timing_equals_the_link_model():
    """v0.7.1: the VTOL timing is sim.link_model.round_trip(cfg, mode='vtol') (the 39-byte TELEM2 reply); v0.8: the GPS
    build's is round_trip(cfg, mode='vtol', gps=True) (the 45-byte TELEM3 reply), the default (position_hold.enabled)"""
    from sim.link_model import LinkConfig, round_trip
    lb = lp.lbat_build(P, "vtol")                    # 2026-11-10: the carrier battery build replies with TELEM v6 / v7
    assert lb
    for radio in ("lora_uart", "sx127x", "esp_bridge"):
        for gps in (False, True):
            rt = round_trip(LinkConfig.from_params(P, radio=radio), mode=lp.MODE_VTOL, gps=gps, lbat=lb)
            lt = gsm.link_timing(P, radio, "vtol", gps)
            assert lt["gps"] is gps and lt["lbat_tail"] is lb and (lt["reply_timeout_s"], lt["t_link_default_s"],
                                                                   lt["rtt_worst_s"]) == \
                   (rt["reply_timeout_s"], rt["rtt_mean_s"], rt["rtt_worst_s"])
        assert gsm.link_timing(P, radio, "vtol")["gps"] is bool(V["position_hold"]["enabled"])
        lt_t = gsm.link_timing(P, radio, "vtol", True, "tether")             # the power builds keep TELEM v5 (no tail)
        assert lt_t["power_tail"] and not lt_t["lbat_tail"]
        assert not gsm.link_timing(P, radio, "ballistic")["lbat_tail"]


def test_vtol_profile_option_rules():
    with pytest.raises(SystemExit):
        gsm.main(["--sim", "--mode", "ballistic", "--profile", "demo", "--fast", "--no-log"])
    with pytest.raises(SystemExit):
        gsm.main(["--sim", "--profile", "indoor", "--fast", "--no-log"])
    with pytest.raises(ValueError):
        gsm.VirtualVehicle(None, None, 1, mode="ballistic", profile="demo")
    # v0.8: --gps names the VTOL build (fcu / fcu_demo vs fcu_nogps); not an option of the ballistic build
    with pytest.raises(SystemExit):
        gsm.main(["--sim", "--mode", "ballistic", "--gps", "on", "--fast", "--no-log"])
    with pytest.raises(ValueError):
        gsm.VirtualVehicle(None, None, 1, mode="ballistic", gps=True)
    with pytest.raises(ValueError):
        gsm.gps_build(P, "ballistic", True)
    assert gsm.gps_build(P, "ballistic") is False and gsm.gps_build(P, "vtol", False) is False
    assert gsm.parse_command("sim gps off") == gsm.Action("sim", False, sub="gps")
    with pytest.raises(ValueError):
        gsm.parse_command("sim gps maybe")


def test_vtol_link_timing_uses_the_telem2_frame():
    lt1, lt2 = gsm.link_timing(P, "lora_uart", "ballistic"), gsm.link_timing(P, "lora_uart", "vtol", False)
    assert lt2["mode"] == "vtol" and lt2["reply_timeout_s"] > lt1["reply_timeout_s"]     # 39-byte TELEM2 downlink
    lt3 = gsm.link_timing(P, "lora_uart", "vtol", True)
    assert lt3["reply_timeout_s"] > lt2["reply_timeout_s"] and not lt1["gps"]           # v0.8: 45-byte TELEM3
    assert gsm.ground_params(P, None, mode="vtol").vtol and not gsm.ground_params(P, None, mode="ballistic").vtol


@pytest.mark.skipif(not hasattr(signal, "raise_signal"), reason="needs signal.raise_signal")
def test_vtol_ctrl_c_lands_and_double_ctrl_c_quits(tmp_path):
    presses = []

    def on_step(now: float) -> None:
        if now >= 14.0 and len(presses) == 0:
            presses.append(now)
            signal.raise_signal(signal.SIGINT)           # Ctrl+C -> OFF (land), not KILL
        elif now >= 14.5 and len(presses) == 1:
            presses.append(now)
            signal.raise_signal(signal.SIGINT)           # second Ctrl+C within 2 s -> quit once the OFF is acked
    st, vv, rows = practice(tmp_path, "vctrlc", mode="vtol", script="3:on", on_step=on_step, max_time=60.0, gps=False)
    offs = [r for r in st.gs.records if r.cmd == lp.CMD_OFF]
    assert offs and offs[0].source == "operator" and offs[0].t_queued == pytest.approx(14.0, abs=0.01)
    assert not any(r.cmd == lp.CMD_KILL for r in st.gs.records)
    assert float(rows[-1]["t_gs_s"]) < 14.5 + gsm.QUIT_FLUSH_S + 0.01
    vv.advance(float(rows[-1]["t_gs_s"]) + 30.0)         # link gone: the vehicle lands by itself and goes SAFE
    assert vv.sup.state.name == "SAFE" and vv.truth()["touchdown_vz_max_mps"] < 1.0
    assert signal.getsignal(signal.SIGINT) is signal.default_int_handler


def test_vtol_kill_in_the_hover_aborts(tmp_path):
    st, vv, rows = practice(tmp_path, "vkill", mode="vtol", script="3:on,14:kill,20:quit", max_time=40.0, gps=False)
    s = gsm.summarize_log(rows)
    assert ("KILL", "accepted") in [(c["cmd"], c["result"]) for c in s["commands"]]
    assert s["states"][-1] in ("ABORT", "RELEASED") and vv.sup.state.name in ("ABORT", "RELEASED")
    vv.advance(25.0)
    assert vv.sup.state.name == "RELEASED" and vv.t_release is not None


def test_vtol_link_cut_in_the_hover_lands_by_itself(tmp_path):
    st, vv, rows = practice(tmp_path, "vlink", mode="vtol", script="3:on,13:sim link off,40:sim link on,45:quit",
                            max_time=60.0, gps=False)
    reasons = [e for _, e in vv.events if "V1" in e]
    assert reasons, vv.events                              # V1 link-loss LAND acted on the vehicle
    assert vv.sup.state.name == "SAFE" and vv.truth()["touchdown_vz_max_mps"] < 1.0
    assert gsm.summarize_log(rows)["states"][-1] == "SAFE"


def test_vtol_piped_stdin_confirmations():
    """the prompt thread asks before ON and KILL in VTOL; 'no' cancels, nothing but heartbeats is sent"""
    import subprocess
    r = subprocess.run([sys.executable, os.path.join(HERE, "ground_station.py"), "--sim", "--mode", "vtol", "--plain",
                        "--quiet", "--no-log", "--speed", "5"], input="status\non\nno\nkill\nno\narm\nquit\n",
                       text=True, encoding="utf-8", errors="replace", capture_output=True, timeout=120)
    assert r.returncode == 0, r.stderr
    assert "on: cancelled" in r.stdout and "kill: cancelled" in r.stdout
    assert "ARM refused: not a command of the VTOL vehicle" in r.stdout


# --------------------------------------------------------------------------- #
# v0.8 VTOL + GPS position hold
# --------------------------------------------------------------------------- #
GATE = 0x02                                                  # sim.gps_nav GF_GATE_OK
GF_READY = 0x01 | GATE | 0x08 | 0x10 | 0x20 | 0x40           # fix_ok gate_ok fix3d cfg_ok init ver_ok


def tlm3(seq: int, *, gps_flags: int = GF_READY, num_sv: int = 9, hacc_dm: int = 24, pos_mode: int = 0,
         dist=float("nan"), state: int = lp.STATE_SAFE, phase: int = lp.PHASE_NONE, tgt: float = 0.0, ack: int = 0,
         t_ms: int = 0, nonce: int = 0x1111, reason: int = 0,
         flags: int = lp.FLAG_LINK_OK | lp.FLAG_SENSORS_OK | lp.FLAG_ALIGNED | lp.FLAG_ARM_SWITCH) -> bytes:
    t = lp.Telemetry.from_physical(state=state, flags=flags, reason=reason, ack_seq=ack, nonce=nonce, t_ms=t_ms, h_m=0.0,
                                   vz_mps=0.0, roll_deg=0.1, pitch_deg=-0.2, yaw_deg=3.0, tilt_deg=0.25, throttle=0.0,
                                   vcell_v=4.1, loop_max_us=3000.0, phase=phase, h_tgt_m=tgt,
                                   gps=(gps_flags, num_sv, hacc_dm, pos_mode, dist))
    return lp.encode_telemetry(VID, seq, t)


@pytest.fixture(scope="module")
def vtol_gps_hop(tmp_path_factory):
    """v0.8 GPS build practice hop (lora_uart, 5 % loss, seed 1): ON at 3 s (refused until the GPS is ready, the scripted
    operator retries), UP, DOWN, OFF, quit after LANDED -> SAFE"""
    return practice(tmp_path_factory.mktemp("vtol_gps"), "vtol_gps_hop", mode="vtol", gps=True,
                    script="3:on,22:up,27:down,32:off,65:quit", max_time=120.0)


def test_vtol_gps_practice_hop(vtol_gps_hop):
    from sim.ground_station_logic import REASON_TEXT_KO
    st, vv, rows = vtol_gps_hop
    assert st.gps and vv.gps_on and "GPS" in st.transport.desc
    s = gsm.summarize_log(rows)
    assert s["states"] == ["SAFE", "ARMED", "POWERED", "UNPOWERED", "SAFE"]
    assert s["phases"] == ["NONE", "SPOOL", "CLIMB", "HOLD", "LAND", "LANDED", "NONE"]
    assert [(c["cmd"], c["result"]) for c in s["commands"]] == [("ON", "accepted"), ("UP", "accepted"),
                                                                ("DOWN", "accepted"), ("OFF", "accepted")]
    assert s["interlocks"] == [] and not s["interlock_flag"]
    # ON was refused here (never sent) while the TELEM v3 showed the GPS not ready, then queued once it was
    refusals = [m for _, m in st.messages if m.startswith("ON refused: GPS not ready")]
    assert refusals and all(m.endswith(REASON_TEXT_KO[lp.REJ_GPS]) for m in refusals)
    ups = [r for r in rows if r["kind"] == "UP" and r["cmd"] == "ON"]
    assert ups and s["gps_ready_first_s"] is not None and float(ups[0]["t_gs_s"]) > s["gps_ready_first_s"]
    tr = vv.truth()
    assert tr["gps"] and tr["gps_ready_s"] is not None and tr["gps_ready_s"] <= s["gps_ready_first_s"]
    # the position loop: velocity hold in the climb, position hold at the hold point, off after the landing
    assert s["pos_modes"][:3] == ["OFF", "VEL", "POS"] and s["pos_modes"][-1] == "OFF"
    assert tr["pos_modes"].split()[:3] == ["OFF", "VEL", "POS"]
    # the distance shown is the GPS wander (the practice vehicle has no horizontal motion): within a few metres
    assert s["dist_max_m"] is not None and s["dist_max_m"] < 5.0
    hover, step = float(V["hover_altitude_m"]), float(V["alt_step_m"])
    assert hover + step - 0.5 < tr["h_max_m"] < hover + step + 1.0
    assert 0.0 < tr["touchdown_vz_max_mps"] < 1.0 and tr["state"] == "SAFE"
    # the TELEM v3 columns are logged and the raw bytes decode again to them
    dec = lp.FrameDecoder(VID, lp.TELEM_TYPES)
    tl = [r for r in rows if r["kind"] == "TLM"]
    assert all(r["gps_flags"] for r in tl)
    for r in tl[::5]:
        (fr,) = dec.feed(bytes.fromhex(r["hex"]))
        t = lp.decode_telemetry(fr)
        assert t.is_v3 and gsm.tlm_fields(t)["gps_flags"] == r["gps_flags"] and str(t.num_sv) == r["num_sv"]
        assert r["pos_mode"] == {0: "OFF", 1: "VEL", 2: "POS", 3: "DR", 4: "ATT"}[t.pos_mode]
        assert (r["dist_m"] == "") == (t.dist_m is None) and (r["hacc_m"] == "") == (t.hacc_dm >= lp.HACC_NONE)
    # status block: one GPS line (Korean status), same height as before the first telemetry; plain line; replay
    lines = st.status_lines(30.0, 160)
    gl = [ln for ln in lines if ln.startswith("GPS    ")]
    assert len(gl) == 1 and "READY" in gl[0] and "GPS 준비됨" in gl[0] and "위성" in gl[0]
    fresh = gsm.GroundStation(P, gsm.NullTransport(), gsm.ground_params(P, None, mode="vtol", gps=True), gps=True)
    assert len(fresh.status_lines(0.0, 120)) == len(lines)
    for width in (80, 120):
        for ln in st.status_lines(30.0, width):
            assert gsm.clip_cols(ln, width - 1) == ln
    assert "GPS ok" in st.plain_line(30.0)
    assert any(m.startswith("position hold VEL -> POS: ") for _, m in st.messages)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rs = gsm.run_replay(st.log.path, P, gsm.Console(want_ui=False, quiet=True), speed=1.0, fast=True)
    assert rs == s and rs["gps_frames"] == len(tl)


def test_vtol_gps_lost_in_the_hover_lands(tmp_path):
    """practice 'sim gps off' in the hover: V11 GPS lost -> dead reckoning -> attitude-only -> LAND -> SAFE"""
    from sim.ground_station_logic import reason_text_ko
    st, vv, rows = practice(tmp_path, "vgpslost", mode="vtol", gps=True, script="3:on,22:sim gps off,60:quit",
                            max_time=80.0)
    s = gsm.summarize_log(rows)
    assert lp.REASON_NAMES[lp.R_V_GPS_LOST] in s["reasons"]
    assert "DR" in s["pos_modes"] and "ATT" in s["pos_modes"] and s["phases"][-3:] == ["LAND", "LANDED", "NONE"]
    assert s["states"][-1] == "SAFE" and vv.truth()["touchdown_vz_max_mps"] < 1.0
    ko = reason_text_ko(lp.R_V_GPS_LOST)
    assert any(m.endswith(ko) and m.startswith("INTERLOCK on the vehicle") for _, m in st.messages)
    assert any(m.startswith("practice: GPS sky BLOCKED") for _, m in st.messages)


def test_vtol_gps_on_refused_until_ready_and_build_notes(tmp_path):
    from sim.ground_station_logic import REASON_TEXT_KO
    vehicle = {"gps_flags": 0x10 | 0x40, "num_sv": 3, "hacc_dm": 255, "t_ms": 0}

    def answer(frame: bytes):
        seq, _ = uplink(frame)
        vehicle["t_ms"] += 100
        return tlm3(seq, gps_flags=vehicle["gps_flags"], num_sv=vehicle["num_sv"], hacc_dm=vehicle["hacc_dm"],
                    t_ms=vehicle["t_ms"])

    ser = FakeSerial(on_write=answer)
    st, log = make_station(tmp_path, ser, mode="vtol", gps=True)
    st.step(0.0)
    st.step(0.05)
    st.request(gsm.Action("on", confirmed=True), 0.05)
    assert st.refused == "gps" and not st.gs.records
    assert st.messages[-1][1] == f"ON refused: GPS not ready (3 SV, hAcc -) - {REASON_TEXT_KO[lp.REJ_GPS]}"
    assert "No GPS" not in st.confirmation_prompt(gsm.Action("on")) and "vane 1" in st.confirmation_prompt(gsm.Action("on"))
    gl = [ln for ln in st.status_lines(0.1, 200) if ln.startswith("GPS")]
    assert gl and "NOT READY" in gl[0] and "GPS 고정 없음" in gl[0] and "정확도 -" in gl[0]
    row = rows_of(log, "TLM")[-1]
    assert (row["gps_flags"], row["num_sv"], row["hacc_m"], row["pos_mode"], row["dist_m"]) == ("0x50", "3", "", "OFF", "")
    vehicle.update(gps_flags=GF_READY, num_sv=9, hacc_dm=24)
    t1 = 0.06 + 1.0 / st.gs.gp.poll_hz
    st.step(t1)
    st.step(t1 + 0.05)
    st.request(gsm.Action("on", confirmed=True), t1 + 0.05)
    assert st.refused == "" and [r.name for r in st.gs.records] == ["ON"]
    assert any(m.startswith("GPS ready (9 SV") for _, m in st.messages)
    assert not any(m.startswith("note:") for _, m in st.messages)
    # the vehicle's telemetry version differs from --gps: one note, ON is left to the vehicle
    st2, _ = make_station(tmp_path / "n2", FakeSerial(on_write=answer), mode="vtol", gps=False)
    st2.step(0.0)
    st2.step(0.05)
    notes = [m for _, m in st2.messages if m.startswith("note:")]
    assert len(notes) == 1 and "TELEM v3 (GPS build)" in notes[0] and "--gps on" in notes[0]

    def answer2(frame: bytes):
        seq, _ = uplink(frame)
        vehicle["t_ms"] += 100
        return tlm2(seq, t_ms=vehicle["t_ms"])
    st3, _ = make_station(tmp_path / "n3", FakeSerial(on_write=answer2), mode="vtol", gps=True)
    st3.step(0.0)
    st3.step(0.05)
    assert any("TELEM v2 (build without GPS" in m for _, m in st3.messages)
    assert any(ln.startswith("GPS    none") and "--gps on" in ln for ln in st3.status_lines(0.1, 200))
    st3.request(gsm.Action("on", confirmed=True), 0.05)
    assert st3.refused == ""                                        # TELEM v2: no GPS gate on the ground


# --------------------------------------------------------------------------- #
# v0.8.1: four chips - the GNS board
# --------------------------------------------------------------------------- #
def _t3(**kw) -> lp.Telemetry:
    (fr,) = lp.FrameDecoder(VID, lp.TELEM_TYPES).feed(tlm3(1, **kw))
    return lp.decode_telemetry(fr)


def test_vtol_gps_practice_runs_the_four_chip_chain(vtol_gps_hop):
    """the practice GPS build is the v0.8.1 chain: GNS board (receiver, filter, position controller), glink STATE /
    RESULT, nlink v4 slots, the FCU stack without the position controller"""
    st, vv, rows = vtol_gps_hop
    tr = vv.truth()
    assert vv.chain == "gns" and tr["chain"] == "gns" and vv.fcu.gns and vv.fcu.pos is None
    assert tr["glink_states"] > 1000 and 0 <= tr["glink_states"] - tr["gns_cycles"] <= 1
    assert tr["glink_results_missed"] <= 1 and tr["glink_results_latched"] >= tr["glink_states"] - 2
    assert tr["fcu_tilt_parts"] == tr["glink_results_latched"]
    gl = [ln for ln in st.status_lines(30.0, 200) if ln.startswith("GPS    ")]
    assert "| GNS OK |" in gl[0] and "GNS 결과" not in gl[0]
    assert "GNS OK" in st.plain_line(30.0)
    assert not any(m.startswith("GNS ") for _, m in st.messages)       # boot SILENT -> OK is not an event


def test_gns_health_from_telem_v3(tmp_path):
    assert gsm.gns_health(_t3()) == gsm.GNS_OK
    assert gsm.gns_health(_t3(gps_flags=0x10 | 0x40, num_sv=0, hacc_dm=255)) == gsm.GNS_OK   # configured, no fix yet
    assert gsm.gns_health(_t3(gps_flags=0x08 | 0x40)) == gsm.GNS_NO_CFG
    assert gsm.gns_health(_t3(gps_flags=0, num_sv=0, hacc_dm=255)) == gsm.GNS_SILENT
    assert gsm.gns_health(_t3(gps_flags=0, num_sv=9, hacc_dm=24)) == gsm.GNS_STALE
    assert gsm.gns_health(_t3(gps_flags=0, num_sv=0, hacc_dm=31)) == gsm.GNS_STALE
    (fr,) = lp.FrameDecoder(VID, lp.TELEM_TYPES).feed(tlm2(1))
    assert gsm.gns_health(lp.decode_telemetry(fr)) is None
    # the station: the GPS line and the events (boot SILENT -> OK silent; OK -> STALE -> OK told)
    vehicle = {"gps_flags": 0, "num_sv": 0, "hacc_dm": 255, "t_ms": 0}

    def answer(frame: bytes):
        seq, _ = uplink(frame)
        vehicle["t_ms"] += 100
        return tlm3(seq, gps_flags=vehicle["gps_flags"], num_sv=vehicle["num_sv"], hacc_dm=vehicle["hacc_dm"],
                    t_ms=vehicle["t_ms"])

    st, _ = make_station(tmp_path, FakeSerial(on_write=answer), mode="vtol", gps=True)
    dt = 1.0 / st.gs.gp.poll_hz
    now = [0.0]

    def poll(**kw) -> list[str]:
        vehicle.update(kw)
        st.step(now[0])
        st.step(now[0] + 0.05)
        now[0] += dt + 0.06
        return [ln for ln in st.status_lines(now[0], 400) if ln.startswith("GPS    ")]
    gl = poll()
    assert "GNS SILENT (GNS/GPS 응답 없음" in gl[0] and "glink" in gl[0]
    assert not any(m.startswith("GNS ") for _, m in st.messages)
    gl = poll(gps_flags=GF_READY, num_sv=9, hacc_dm=24)
    assert "| GNS OK |" in gl[0] and not any(m.startswith("GNS ") for _, m in st.messages)
    gl = poll(gps_flags=0)
    assert "GNS STALE (GNS 결과 끊김" in gl[0] and "NOT READY" in gl[0]
    assert [m for _, m in st.messages if m.startswith("GNS ")] == [f"GNS STALE: {gsm.GNS_HEALTH[gsm.GNS_STALE][1]}"]
    gl = poll(gps_flags=0x08 | 0x40)
    assert "GNS NO-CFG (GNS는 응답" in gl[0]
    poll(gps_flags=GF_READY)
    msgs = [m for _, m in st.messages if m.startswith("GNS ")]
    assert len(msgs) == 3 and msgs[1].startswith("GNS NO-CFG: ") and msgs[2] == f"GNS OK: {gsm.GNS_HEALTH[gsm.GNS_OK][1]}"
    assert "GNS OK" in st.plain_line(now[0])


def test_vtol_gns_link_cut_in_the_hover_lands(tmp_path):
    """practice 'sim gns off' in the hover (the glink wire cut): no TILT part reaches the FCU -> the wind-trim fallback
    (TELEM ATT) after two cycles, the FCU's own GPS_LOST request (V11) after land_steps cycles -> LAND -> SAFE; the GPS
    line shows GNS STALE"""
    from sim.ground_station_logic import reason_text_ko
    st, vv, rows = practice(tmp_path, "vgnscut", mode="vtol", gps=True, script="3:on,22:sim gns off,60:quit",
                            max_time=80.0)
    s = gsm.summarize_log(rows)
    assert lp.REASON_NAMES[lp.R_V_GPS_LOST] in s["reasons"]
    assert "ATT" in s["pos_modes"] and s["phases"][-3:] == ["LAND", "LANDED", "NONE"]
    assert s["states"][-1] == "SAFE" and vv.truth()["touchdown_vz_max_mps"] < 1.0
    ko = reason_text_ko(lp.R_V_GPS_LOST)
    assert any(m.endswith(ko) and m.startswith("INTERLOCK on the vehicle") for _, m in st.messages)
    assert any(m.startswith("practice: GNS link CUT") for _, m in st.messages)
    assert any(m.startswith("GNS STALE: ") for _, m in st.messages)
    gl = [ln for ln in st.status_lines(59.0, 400) if ln.startswith("GPS    ")]
    assert "GNS STALE" in gl[0] and "GNS 결과 끊김" in gl[0]
    assert vv.gl_cut and "practice: GNS link already cut" not in [m for _, m in st.messages]
    assert vv.set_gns(False, 61.0) == "GNS link already cut"
    assert gsm.parse_command("sim gns off") == gsm.Action("sim", False, sub="gns")
    nogps = gsm.VirtualVehicle(None, None, 1, mode="vtol", gps=False)
    assert nogps.set_gns(False, 1.0).startswith("practice: this vehicle has no GNS board")


# --------------------------------------------------------------------------- #
# 2026-11-06: the ground ESP-NOW bridge (firmware/esp_bridge) as the default ground hardware
# --------------------------------------------------------------------------- #
def test_link_hw_default_and_baud_follow_the_vehicle_radio():
    lk = P["flight_computer"]["link"]
    ap = gsm.build_arg_parser(P)
    a = ap.parse_args(["--port", "COM9"])
    assert a.link_hw == lk["vehicle_radio"] == "esp_bridge"
    assert gsm.serial_baud(P, a.link_hw, a.baud) == lk["esp_bridge"]["uart_baud"]
    a = ap.parse_args(["--port", "COM9", "--link-hw", "lora_uart"])
    assert gsm.serial_baud(P, a.link_hw, a.baud) == lk["uart_baud"]          # the Ebyte / SX127x bridge baud, unchanged
    a = ap.parse_args(["--port", "COM9", "--baud", "115200"])
    assert gsm.serial_baud(P, a.link_hw, a.baud) == 115200
    # 2026-11-10: the link MODEL follows the hardware (--link-hw) unless --radio is given: the ESP model with the ESP
    assert a.radio is None and gsm.link_model_of(P, a.link_hw, a.radio) == "esp_bridge" == lk["vehicle_radio"]
    assert gsm.link_model_of(P, "lora_uart") == "lora_uart" and gsm.link_model_of(P, "esp_bridge", "sx127x") == "sx127x"
    assert gsm.link_timing(P, "lora_uart")["reply_timeout_s"] > 2 * gsm.link_timing(P, "esp_bridge")["reply_timeout_s"]
    old = {**P, "flight_computer": {**P["flight_computer"], "link": {k: v for k, v in lk.items() if k != "vehicle_radio"}}}
    assert gsm.default_link_hw(old) == lk["radio"]


def test_serial_transport_releases_dtr_rts_for_the_esp(tmp_path):
    pytest.importorskip("serial")
    tr = gsm.SerialTransport("loop://", 38400, release_lines=True)
    assert tr.ser.is_open and tr.ser.dtr is False and tr.ser.rts is False and "DTR / RTS released" in tr.desc
    st = gsm.GroundStation(P, tr, gsm.ground_params(P, None, mode="ballistic"), log=None)
    st.step(0.0)
    st.step(0.01)
    assert st.tap.stats["wrong_type"] == 1                                   # the same byte path as the LoRa ports
    tr.close()
