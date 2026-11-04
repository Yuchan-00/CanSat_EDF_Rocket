"""Tests of decode_log.py against the firmware's own packer (log format 4, v0.6 / v0.6.3 FCU; format 5, v0.7 VTOL;
format 6, v0.7.1 VTOL).

The C side writes the records: ``test_core --log-sample`` (host build of src/core, tools/build_host_test.py)
frames one record of every v0.6 type with log_record.c / ilink_proto.c; this file decodes them with decode_log.py
and checks every field it set, including the v0.6 'C' record (an IL_STAT as received) and the 40-byte 'S'
record.  The v0.6.3 'N' record (NAV link) is packed by log_record.c in a small host program compiled here with the
same Zig toolchain - also with the NAV_RC_* bits 7..5 of nav_reset_cause set, which must be named apart from the
MCUCSR flags of bits 4..0.  A format-3 (v0.5, 28-byte 'S') record built here byte by byte must still decode.
v0.7, log format 5 (the VTOL build): a host program packs the boot record, 'E' with the phases, 'L' with the phase after
the command, the 'V' record and a flight record from the fine split (equal to log_pack_flight byte for byte) with
log_record.c; the decoder's copies of the cause / flag / mode codes are checked against the C headers, and a format-4
log keeps naming the codes 8..11 as v0.6.3 did.
v0.7.1, log format 6: the same host program now writes format 6 - the boot text naming the build profile, the V8 / V9 /
V10 'E' records (reason codes 21 / 22 / 23) and 'V' records with the touchdown detector v2 fields and the NAV's
estimator-correction count; a v0.7.0 format-5 'V' record (26 bytes) must still decode, alone and in one file with
format-6 records; the decoder's copies of VC_SIG_* / VC_TD_* / NAV_F_EST_GATE equal the C headers.
v0.8, log format 7 (the VTOL build with GPS position hold): a host program packs a 'G' record with log_record.c (log_pack_gps
and log_pack_gps_in_place: the same bytes) and frames it; every field decodes, the "none" codes (no hold point, no
distance, no accuracy, no set) decode as None, the decoder's copies of LOG_G_* / LOG_GPS_LEN / the GPS flag, mode and
request names equal the C header and the references (sim/gps_nav.py GF_NAMES, sim/position_ctrl.py).

    .venv/Scripts/python.exe -m pytest firmware/atmega128_fc/tools/test_decode_log.py -q -p no:cacheprovider
"""
from __future__ import annotations

import os
import struct
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, ROOT)

import build_host_test as bht  # noqa: E402
import decode_log  # noqa: E402
from sim import ilink_proto as il  # noqa: E402
from sim import nlink_proto as nl  # noqa: E402


@pytest.fixture(scope="module")
def sample(tmp_path_factory) -> dict:
    exe = bht.build()
    path = str(tmp_path_factory.mktemp("log") / "sample.bin")
    r = subprocess.run([exe, "--log-sample", path], capture_output=True, text=True, encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stdout + r.stderr
    with open(path, "rb") as f:
        return decode_log.decode(f.read())


def test_layouts_match_the_references():
    assert (decode_log.FORMAT_VERSION, decode_log.FORMAT_VERSION_VTOL, decode_log.FORMAT_VERSION_GPS) == (4, 6, 7)
    assert decode_log.FORMATS_DECODED == (3, 4, 5, 6, 7, 8) and decode_log.FORMAT_VTOL_FIRST == 5   # v0.8.1: 8
    assert (decode_log.FORMAT_BALLISTIC, decode_log.FORMAT_VTOL) == (4, 6)
    assert (decode_log.VTOL_LEN_V5, decode_log.VTOL_LEN_V6) == (26, 35)
    assert struct.calcsize(decode_log.STATUS_FMT) == 40          # log_record.h LOG_STATUS_LEN
    assert struct.calcsize(decode_log.STATUS_FMT_V3) == 28       # v0.5
    assert struct.calcsize(decode_log.COM_FMT) == 40             # LOG_COM_LEN = tick + IL_STAT_LEN
    assert decode_log._STAT_FMT == il.STAT_FMT                    # the standalone copy == sim/ilink_proto.py
    assert decode_log._STAT_FIELDS == tuple(il.STAT_FIELDS)
    assert (decode_log.STAT_RADIO_OK, decode_log.STAT_BOOT, decode_log.STAT_SX127X) == \
           (il.STAT_RADIO_OK, il.STAT_BOOT, il.STAT_SX127X)
    assert struct.calcsize(decode_log.NAV_FMT) == 20             # log_record.h LOG_NAV_LEN (v0.6.3)
    assert decode_log._NAV_FLAG_BITS == {nl.F_IMU_OK: "imu_ok", nl.F_BARO_OK: "baro_ok", nl.F_ALIGNED: "aligned",
                                         nl.F_GYRO_SAT: "gyro_sat", nl.F_EST_VALID: "est_valid",
                                         nl.F_BARO_NEW: "baro_new", nl.F_BOOT: "nav_boot"}
    # nav_reset_cause bits 7..5 (the standalone copy == sim/nlink_proto.py == nav.h, the latter checked there)
    assert decode_log._NAV_RC_BITS == nl.NAV_RC_NAMES == {nl.NAV_RC_TAG_MISMATCH: "tag_mismatch",
                                                         nl.NAV_RC_BARO_INIT_FAIL: "baro_init_fail",
                                                         nl.NAV_RC_IMU_INIT_FAIL: "imu_init_fail"}
    assert decode_log._NAV_RC_MCUCSR_MASK == nl.NAV_RC_MCUCSR_MASK == 0x1F
    assert {b: n for b, n in decode_log.RESET_BITS.items()} == nl.MCUCSR_RESET_NAMES


def test_every_record_type(sample):
    d = sample
    assert d["bad"] == 0 and d["skipped"] == 3                   # the three garbage bytes between flight records
    assert d["text"] == ["boot sample"]                           # no format warning: the boot record says 4
    assert len(d["boot"]) == 1 and d["boot"][0]["format"] == 4 and d["boot"][0]["role"] == "FCU"
    assert len(d["flight"]) == 3 and len(d["events"]) == 1 and len(d["link"]) == 1


def test_status_format4(sample):
    (st,) = sample["status"]
    assert st["format"] == 4
    assert (st["link_frames"], st["link_bad"], st["replies"], st["stack_free"]) == (77, 3, 76, 1500)
    assert st["rssi_dbm"] == -98 and abs(st["vbat_v"] - 23.9) < 1e-9
    assert (st["twi_errors"], st["uart_dropped"], st["link_other"], st["radio_errors"], st["baro_fails"],
            st["imu_fails"]) == (11, 12, 13, 14, 15, 16)
    assert (st["il_cmds"], st["il_snaps"], st["il_bad"], st["il_uart_errors"], st["il_dropped"]) == (78, 76, 2, 1, 4)
    assert st["com_age_s"] == 1 and st["radio_ok"] == 1 and st["com_params_mismatch"] == 1


def test_com_record(sample):
    (c,) = sample["com"]
    assert c["tick"] == 1245 and abs(c["t_s"] - 1245 / sample["boot"][0]["loop_hz"]) < 1e-12
    assert (c["com_ticks"], c["radio_version"], c["params_tag"]) == (1240, 0x12, 0x48)
    assert (c["radio_ok"], c["com_boot"], c["backend"]) == (1, 1, "sx127x")
    assert c["rssi_dbm"] == -101 and c["snr_db"] == -2.5 and c["reset"] == ["watchdog"]
    assert (c["dec_frames"], c["dec_bad_crc"], c["cmd_forwarded"], c["replies"], c["il_tx_dropped"], c["loop_us_max"]) == \
           (77, 3, 77, 76, 1, 812)


NAV_SAMPLE_C = r"""
#include <stdio.h>
#include "log_record.h"
int main(int argc, char **argv)
{
    /* tick, frames, bad, timeouts, seq_lost, resyncs, spi_overruns, nav_loop_us_max, nav_flags, nav_reset_cause */
    const log_nav_t n = {1250u, 2785u, 1u, 26u, 7u, 3u, 2u, 1480u, 0x57u, NAV_RESET_CAUSE};
    uint8_t pay[LOG_MAX_PAYLOAD], fr[LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD];
    const uint8_t len = log_pack_nav(&n, pay);
    const uint16_t m = log_frame(LOG_TYPE_NAV, pay, len, fr);
    FILE *f;
    if (argc != 2 || len != LOG_NAV_LEN || (f = fopen(argv[1], "wb")) == NULL)
        return 3;
    fwrite(fr, 1, m, f);
    fclose(f);
    return 0;
}
"""


def _nav_record_from_c(tmp_path, reset_cause: int) -> dict:
    """One 'N' record packed by log_record.c (host build) with this nav_reset_cause, decoded."""
    src = tmp_path / f"nav_sample_{reset_cause:02x}.c"
    src.write_text(NAV_SAMPLE_C.replace("NAV_RESET_CAUSE", f"0x{reset_cause:02X}u"), encoding="utf-8")
    exe = str(tmp_path / (f"nav_sample_{reset_cause:02x}" + (".exe" if os.name == "nt" else "")))
    r = subprocess.run([sys.executable, "-m", "ziglang", "cc", *bht.CFLAGS, "-Werror", "-I", bht.CORE, str(src),
                        os.path.join(bht.CORE, "log_record.c"), "-o", exe], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stdout + r.stderr
    out = tmp_path / f"nav_{reset_cause:02x}.bin"
    r = subprocess.run([exe, str(out)], capture_output=True, text=True, encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stdout + r.stderr
    d = decode_log.decode(out.read_bytes(), loop_hz=250.0)
    assert d["bad"] == 0 and d["skipped"] == 0
    (n,) = d["nav"]
    return n


def test_nav_record_from_the_c_packer(tmp_path):
    n = _nav_record_from_c(tmp_path, 0x08)
    assert (n["tick"], n["frames"], n["bad"], n["timeouts"], n["seq_lost"], n["resyncs"], n["spi_overruns"],
            n["nav_loop_us_max"], n["nav_flags"], n["nav_reset_cause"]) == (1250, 2785, 1, 26, 7, 3, 2, 1480, 0x57, 0x08)
    assert n["t_s"] == 5.0 and n["nav_reset"] == ["watchdog"] and n["nav_rc"] == []
    assert (n["nav_imu_init_fail"], n["nav_baro_init_fail"], n["nav_tag_mismatch"]) == (0, 0, 0)
    assert (n["imu_ok"], n["baro_ok"], n["aligned"], n["gyro_sat"], n["est_valid"], n["baro_new"], n["nav_boot"]) == \
           (1, 1, 1, 0, 1, 0, 1)
    assert n["nav_est_gate"] == 0


def test_nav_record_names_the_estimator_flag():
    """v0.7.1: nav_flags bit 7 (nav.h NAV_F_EST_GATE, the VTOL NAV's estimator correction in the newest frame)"""
    boot = struct.pack("<BBHB", 6, 1, 250, 2) + b"fw 3.3.0 role FCU vtol"
    recs = [struct.pack(decode_log.NAV_FMT, 250 * k, 10, 0, 0, 0, 0, 0, 2300, fl, 0x01) for k, fl in ((1, 0x97), (2, 0x17))]
    n = decode_log.decode(_frame("B", boot) + b"".join(_frame("N", r) for r in recs))["nav"]
    assert [x["nav_est_gate"] for x in n] == [1, 0] and [x["est_valid"] for x in n] == [1, 1]


@pytest.mark.parametrize("rc, reset, named", [
    (0x88, ["watchdog"], ["imu_init_fail"]),                                   # NAV_RC_IMU_INIT_FAIL | WDRF
    (0x41, ["power_on"], ["baro_init_fail"]),                                  # NAV_RC_BARO_INIT_FAIL | PORF
    (0x22, ["external"], ["tag_mismatch"]),                                    # NAV_RC_TAG_MISMATCH | EXTRF
    (0xE5, ["power_on", "brown_out"], ["tag_mismatch", "baro_init_fail", "imu_init_fail"]),
])
def test_nav_reset_cause_bits_7_5_are_named(tmp_path, rc, reset, named):
    """v0.6.3: bits 7..5 of the NAV's reset_cause are NAV_RC_* (nav.h), not reset flags: named in nav_rc and in their own
    0/1 columns; nav_reset keeps only the MCUCSR flags of bits 4..0."""
    n = _nav_record_from_c(tmp_path, rc)
    assert n["nav_reset_cause"] == rc and n["nav_reset"] == reset and n["nav_rc"] == named
    assert [n[f"nav_{x}"] for x in ("tag_mismatch", "baro_init_fail", "imu_init_fail")] == \
        [int(x in named) for x in ("tag_mismatch", "baro_init_fail", "imu_init_fail")]


def _frame(typ: str, payload: bytes) -> bytes:
    body = bytes([ord(typ), len(payload)]) + payload
    return b"\xA5\x5A" + body + struct.pack("<H", decode_log.fletcher16(body))


def test_format3_status_still_decodes():
    boot = struct.pack("<BBHB", 3, 1, 250, 2) + b"fw 2.0.0 BUDGET_90mm_6S radio lora_uart"
    st = struct.pack(decode_log.STATUS_FMT_V3, 500, 1400, 0, 0, 10, 1, 0, 2, 9, 0, 0, 24000, 1, -90)
    d = decode_log.decode(_frame("B", boot) + _frame("S", st))
    assert d["boot"][0]["role"] is None and d["text"] == ["note: log format 3 (v0.5 single chip), decoder formats (3, 4, 5, 6, 7, 8)"]
    (s,) = d["status"]
    assert s["format"] == 3 and s["link_frames"] == 10 and s["replies"] == 9 and s["rssi_dbm"] == -90
    assert "il_cmds" not in s


def test_nav_record_is_additive():
    """the 'N' record sits between the format-4 records without changing how they decode"""
    boot = struct.pack("<BBHB", 4, 1, 250, 2) + b"fw 3.1.0 role FCU"
    nav = struct.pack(decode_log.NAV_FMT, 500, 10, 0, 0, 0, 0, 0, 1300, 0x17, 0x01)
    st = struct.pack(decode_log.STATUS_FMT, 500, 1400, 0, 0, 10, 1, 0, 2, 9, 0, 0, 24000, 1, -90, 1, 1, 0, 0, 0, 0, 0x01)
    d = decode_log.decode(_frame("B", boot) + _frame("N", nav) + _frame("S", st))
    assert d["text"] == [] and len(d["nav"]) == 1 and len(d["status"]) == 1 and d["status"][0]["link_frames"] == 10


def test_main_writes_csvs(sample, tmp_path):
    exe = bht.EXE
    log = tmp_path / "LOG00001.TXT"
    r = subprocess.run([exe, "--log-sample", str(log)], capture_output=True, text=True, encoding="utf-8", errors="replace")
    assert r.returncode == 0
    assert decode_log.main([str(log)]) == 0
    for suffix in ("_flight.csv", "_link.csv", "_com.csv"):
        assert (tmp_path / ("LOG00001" + suffix)).exists(), suffix


def test_main_names_the_nav_reports(tmp_path, capsys):
    """The summary names every NAV_RC_* bit seen in any 'N' record (with how many), and _nav.csv has their columns."""
    boot = struct.pack("<BBHB", 4, 1, 250, 2) + b"fw 3.1.0 role FCU"
    recs = [struct.pack(decode_log.NAV_FMT, 250 * k, 10, 0, 0, 0, 0, 0, 1300, 0x17, rc)
            for k, rc in ((1, 0x81), (2, 0x81), (3, 0xA1))]
    log = tmp_path / "LOG00002.TXT"
    log.write_bytes(_frame("B", boot) + b"".join(_frame("N", r) for r in recs))
    assert decode_log.main([str(log)]) == 0
    out = capsys.readouterr().out
    assert "imu_init_fail (3 of 3 records)" in out and "tag_mismatch (1 of 3 records)" in out
    assert "baro_init_fail" not in out and "reset power_on," in out
    header = (tmp_path / "LOG00002_nav.csv").read_text(encoding="utf-8").splitlines()[0].split(",")
    assert {"nav_reset", "nav_rc", "nav_imu_init_fail", "nav_baro_init_fail", "nav_tag_mismatch"} <= set(header)



# --------------------------------------------------------------------------- #
# v0.7: log format 5 (the VTOL build)
# --------------------------------------------------------------------------- #
VTOL_SAMPLE_C = r"""
#include <stdio.h>
#include <string.h>
#include "log_record.h"
#include "onboard.h"
#include "vertical_ctrl.h"
static FILE *f;
static void put(uint8_t type, const uint8_t *pay, uint8_t len)
{
    uint8_t fr[LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD];
    fwrite(fr, 1, log_frame(type, pay, len, fr), f);
}
/* v0.7.1: main.c packs the captured 'V' record in place (log_pack_vtol_in_place): the same bytes as log_pack_vtol */
static int inplace_ok(const log_vtol_t *v)
{
    log_vtol_t w = *v;
    uint8_t ref[LOG_MAX_PAYLOAD];
    const uint8_t n = log_pack_vtol(v, ref);
    return log_pack_vtol_in_place(&w) == n && n == LOG_VTOL_LEN && memcmp(&w, ref, n) == 0;
}
int main(int argc, char **argv)
{
    static const char info[] = "fw 3.3.0 role FCU vtol BUDGET_90mm_6S params 0123456789abcdef gains fedcba9876543210";
    static const char prof[] = "VTOL profile demo: vertical gains sim/out/vtol_gains.json (tuned)";
    uint8_t pay[LOG_MAX_PAYLOAD], whole[LOG_MAX_PAYLOAD], fine[LOG_MAX_PAYLOAD], n = 0, k, end = 0;
    log_flight_t r;
    log_vtol_t v;
    if (argc != 2 || (f = fopen(argv[1], "wb")) == NULL)
        return 3;
    pay[n++] = LOG_FORMAT_VERSION_VTOL; pay[n++] = 0x01; pay[n++] = 250; pay[n++] = 0; pay[n++] = 2;
    memcpy(pay + n, info, sizeof info - 1);
    put(LOG_TYPE_BOOT, pay, (uint8_t)(n + sizeof info - 1));
    put(LOG_TYPE_TEXT, (const uint8_t *)prof, (uint8_t)(sizeof prof - 1));   /* main.c log_boot: the build profile */
    /* CLIMB -> HOLD inside POWERED (at target), LAND -> LANDED (touchdown), the LANDED timer, a V1 LAND */
    put(LOG_TYPE_EVENT, pay, log_pack_event2(2500u, ONB_POWERED, ONB_POWERED, ONB_CAUSE_AT_TARGET, 0, LP_PHASE_CLIMB,
                                             LP_PHASE_HOLD, pay));
    put(LOG_TYPE_EVENT, pay, log_pack_event2(7600u, ONB_POWERED, ONB_UNPOWERED, ONB_CAUSE_TOUCHDOWN, 0, LP_PHASE_LAND,
                                             LP_PHASE_LANDED, pay));
    put(LOG_TYPE_EVENT, pay, log_pack_event2(8100u, ONB_UNPOWERED, ONB_SAFE, ONB_CAUSE_LANDED_TIMER, 0, LP_PHASE_LANDED,
                                             LP_PHASE_NONE, pay));
    put(LOG_TYPE_EVENT, pay, log_pack_event2(9000u, ONB_POWERED, ONB_POWERED, LP_R_V_LINK_LAND, LP_R_V_LINK_LAND,
                                             LP_PHASE_HOLD, LP_PHASE_LAND, pay));
    /* v0.7.1: V8 (baro lost, ground felt -> LANDED), V9 (no ground -> ABORT), V10 (LAND floor timeout -> LANDED) */
    put(LOG_TYPE_EVENT, pay, log_pack_event2(9500u, ONB_POWERED, ONB_UNPOWERED, LP_R_V_BARO_LIMIT_LANDED,
                                             LP_R_V_BARO_LIMIT_LANDED, LP_PHASE_LAND, LP_PHASE_LANDED, pay));
    put(LOG_TYPE_EVENT, pay, log_pack_event2(9600u, ONB_POWERED, ONB_ABORT, LP_R_V_BARO_LIMIT_ABORT,
                                             LP_R_V_BARO_LIMIT_ABORT, LP_PHASE_LAND, LP_PHASE_NONE, pay));
    put(LOG_TYPE_EVENT, pay, log_pack_event2(9700u, ONB_POWERED, ONB_UNPOWERED, LP_R_V_LAND_TIMEOUT, LP_R_V_LAND_TIMEOUT,
                                             LP_PHASE_LAND, LP_PHASE_LANDED, pay));
    put(LOG_TYPE_LINK, pay, log_pack_link2(750u, 5, LP_CMD_ON, 0, 0xBEEFu, 0, ONB_ARMED, -80, 22, LP_PHASE_SPOOL, pay));
    put(LOG_TYPE_LINK, pay, log_pack_link2(3300u, 9, LP_CMD_UP, 0, 0, LP_REJ_ALT_LIMIT, ONB_POWERED, 127, 0, LP_PHASE_HOLD,
                                           pay));
    v.tick = 3000u; v.phase = LP_PHASE_HOLD; v.vflags = LOG_V_MOTOR_ON | LOG_V_AT_TARGET | LOG_V_SAT_HIGH;
    v.reason = 0; v.vc_mode = 1; v.h_tgt_dm = 60; v.h_ref = 5.9375f; v.vz_cmd = -0.25f; v.u = 0.9375f;
    v.u_ff = 0.9140625f; v.integ = 0.015625f; v.td_count = 0; v.motor_ticks = 1500u;
    v.sig = 0; v.td_by = VC_TD_NONE; v.low_in_land = 0; v.vz_lpf = 0.0625f; v.f_excess = -0.25f; v.est_gate = 25;
    if (log_pack_vtol(&v, pay) != LOG_VTOL_LEN)
        return 6;
    if (!inplace_ok(&v))
        return 7;
    put(LOG_TYPE_VTOL, pay, log_pack_vtol(&v, pay));
    v.tick = 7575u; v.phase = LP_PHASE_LAND; v.vflags = LOG_V_MOTOR_ON | LOG_V_TOUCHDOWN | LOG_V_SAT_LOW | LOG_V_BARO_LOST;
    v.vc_mode = 2; v.h_tgt_dm = 0; v.h_ref = -0.5f; v.vz_cmd = -0.3f; v.u = 0.5f; v.u_ff = 0.915f; v.integ = -0.08f;
    v.td_count = 125; v.motor_ticks = 6610u;
    v.sig = VC_SIG_LOW | VC_SIG_SUPPORT | VC_SIG_GROUND; v.td_by = VC_TD_SUPPORT; v.low_in_land = 1499u;
    v.vz_lpf = -0.046875f; v.f_excess = 3.5f; v.est_gate = 20;
    if (!inplace_ok(&v))
        return 7;
    put(LOG_TYPE_VTOL, pay, log_pack_vtol(&v, pay));
    v.tick = 7600u; v.f_excess = 1.0e6f; v.vz_lpf = -1.0e6f; v.est_gate = 255;   /* saturation */
    if (!inplace_ok(&v))
        return 7;
    put(LOG_TYPE_VTOL, pay, log_pack_vtol(&v, pay));
    /* the fine flight-record split packs exactly the bytes of the 4-part one */
    memset(&r, 0, sizeof r);
    for (k = 0; k < sizeof r; k++) ((uint8_t *)&r)[k] = (uint8_t)(k * 37u + 11u);
    r.q[0] = 0.99f; r.h_est = 4.5f; r.throttle = 0.93f; r.v_cell = 3.9f; r.tilt_deg = 2.5f; r.thrust_est = 26.0f;
    memset(whole, 0, sizeof whole); memset(fine, 0, sizeof fine);
    (void)log_pack_flight(&r, whole);
    for (k = 0; k < LOG_FLIGHT_PARTS_FINE; k++) {
        if (log_flight_part_fine_start(k) != end)
            return 4;                                  /* the parts must tile the record */
        end = log_pack_flight_part_fine(&r, fine, k);
    }
    if (end != LOG_FLIGHT_LEN || memcmp(whole, fine, LOG_FLIGHT_LEN) != 0)
        return 5;
    put(LOG_TYPE_FLIGHT, fine, LOG_FLIGHT_LEN);
    fclose(f);
    return 0;
}
"""


@pytest.fixture(scope="module")
def vtol_sample(tmp_path_factory) -> dict:
    """format-6 records packed by log_record.c (host build): boot (+ the profile text), 'E' with phases (+ V8 / V9 / V10),
    'L' with the phase, 'V' (format 6), and a flight record from the fine split (which must equal log_pack_flight byte
    for byte)"""
    d = tmp_path_factory.mktemp("vtol")
    src = d / "vtol_sample.c"
    src.write_text(VTOL_SAMPLE_C, encoding="utf-8")
    exe = str(d / ("vtol_sample" + (".exe" if os.name == "nt" else "")))
    r = subprocess.run([sys.executable, "-m", "ziglang", "cc", *bht.CFLAGS, "-Werror", "-I", bht.CORE, str(src),
                        os.path.join(bht.CORE, "log_record.c"), "-o", exe], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stdout + r.stderr
    out = d / "LOG00005.TXT"
    r = subprocess.run([exe, str(out)], capture_output=True, text=True, encoding="utf-8", errors="replace")
    assert r.returncode == 0, (f"vtol_sample exit {r.returncode} (4/5: the fine split differs from log_pack_flight; "
                               f"6: log_pack_vtol's length is not LOG_VTOL_LEN; 7: the in-place pack differs)")
    return {"path": out, "d": decode_log.decode(out.read_bytes())}


def test_vtol_boot_and_events_carry_the_phases(vtol_sample):
    d = vtol_sample["d"]
    assert d["bad"] == 0 and d["skipped"] == 0
    assert d["text"] == ["VTOL profile demo: vertical gains sim/out/vtol_gains.json (tuned)"]
    (b,) = d["boot"]
    assert (b["format"], b["role"], b["mode"], b["loop_hz"], b["profile"]) == (6, "FCU", "vtol", 250, "demo")
    ev = [(e["tick"], e["frm"], e["to"], e["phase_from"], e["phase_to"], e["cause"], e["reason"]) for e in d["events"]]
    assert ev == [(2500, "POWERED", "POWERED", "CLIMB", "HOLD", "at target", "none"),
                  (7600, "POWERED", "UNPOWERED", "LAND", "LANDED", "touchdown", "none"),
                  (8100, "UNPOWERED", "SAFE", "LANDED", "NONE", "LANDED timer", "none"),
                  (9000, "POWERED", "POWERED", "HOLD", "LAND", "V1 link loss: land", "V1 link loss: land"),
                  (9500, "POWERED", "UNPOWERED", "LAND", "LANDED", "V8 baro-lost time limit: landed (ground felt)",
                   "V8 baro-lost time limit: landed (ground felt)"),
                  (9600, "POWERED", "ABORT", "LAND", "NONE", "V9 baro-lost time limit: abort (no ground felt)",
                   "V9 baro-lost time limit: abort (no ground felt)"),
                  (9700, "POWERED", "UNPOWERED", "LAND", "LANDED", "V10 LAND floor timeout: landed (motor off, no release)",
                   "V10 LAND floor timeout: landed (motor off, no release)")]


def test_vtol_link_records(vtol_sample):
    on, up = vtol_sample["d"]["link"]
    assert (on["cmd"], on["seq"], on["nonce"], on["result"], on["state_after"], on["phase_after"]) == \
           ("ON", 5, 0xBEEF, "executed", "ARMED", "SPOOL")
    assert on["rssi_dbm"] == -80 and on["snr_db"] == 5.5
    assert (up["cmd"], up["result"], up["phase_after"], up["rssi_dbm"]) == ("UP", "rejected: altitude limit", "HOLD", None)


def test_vtol_record_fields(vtol_sample):
    hold, land, sat = vtol_sample["d"]["vtol"]
    assert (hold["tick"], hold["phase"], hold["vc_mode"], hold["h_tgt_m"], hold["motor_s"]) == (3000, "HOLD", "FLY", 6.0, 6.0)
    assert (hold["motor_on"], hold["at_target"], hold["sat_high"], hold["touchdown"], hold["baro_lost"]) == (1, 1, 1, 0, 0)
    assert (hold["h_ref_m"], hold["vz_cmd_mps"], hold["u"], hold["u_ff"], hold["integ"]) == \
           (5.9375, -0.25, 0.9375, 0.9140625, 0.015625)      # exact binary fractions: no fixed-point rounding
    assert (land["phase"], land["vc_mode"], land["touchdown"], land["sat_low"], land["baro_lost"], land["td_count"]) == \
           ("LAND", "LAND", 1, 1, 1, 125)
    assert land["h_ref_m"] == -0.5 and abs(land["vz_cmd_mps"] + 0.3) <= 2 ** -7 and abs(land["integ"] + 0.08) <= 2 ** -15
    assert abs(land["motor_s"] - 26.44) < 1e-9
    # format 6 (v0.7.1): the touchdown detector v2 and the NAV's estimator correction (exact binary fractions)
    assert (hold["sig"], hold["td_by"], hold["low_in_land"], hold["vz_lpf_mps"], hold["f_excess_mps2"], hold["est_gate"]) == \
           (0, "none", 0, 0.0625, -0.25, 25)
    assert hold["est_gate_frac"] == 1.0                     # no previous record: 25 frames / the nominal 25 ticks
    assert (land["sig_low"], land["sig_still"], land["sig_support"], land["sig_floor"], land["sig_ground"]) == (1, 0, 1, 0, 1)
    assert (land["td_by"], land["low_in_land"], land["vz_lpf_mps"], land["f_excess_mps2"], land["est_gate"]) == \
           ("support", 1499, -0.046875, 3.5, 20)
    assert land["low_in_land_s"] == 1499 / 250.0
    assert land["est_gate_frac"] == 0.8                     # 4575 ticks since the previous record: the nominal 25
    assert (sat["vz_lpf_mps"], sat["f_excess_mps2"], sat["est_gate"]) == (-256.0, 32767 / 128.0, 255)   # saturated int16
    assert sat["est_gate_frac"] == 1.0                      # 255 frames in 25 ticks: clamped to 1


def test_format5_vtol_record_still_decodes(tmp_path):
    """a v0.7.0 log (format 5, 26-byte 'V'): the format-5 fields, no format-6 columns; in one file with a format-6 log
    (a reflashed vehicle) the _vtol.csv holds both (the union of the columns)"""
    v5 = struct.pack(decode_log.VTOL_FMT, 3000, 3, 0x09, 0, 1, 60, int(5.9375 * 1024), -32, 30720, 29952, 512, 0, 1500)
    assert len(v5) == 26
    boot5 = struct.pack("<BBHB", 5, 1, 250, 2) + b"fw 3.2.0 role FCU vtol"
    d = decode_log.decode(_frame("B", boot5) + _frame("V", v5))
    (b,) = d["boot"]
    (r,) = d["vtol"]
    assert (b["format"], b["mode"]) == (5, "vtol") and "profile" not in b and d["text"] == []
    assert (r["phase"], r["h_tgt_m"], r["h_ref_m"], r["vz_cmd_mps"], r["motor_on"], r["at_target"]) == \
           ("HOLD", 6.0, 5.9375, -0.25, 1, 1)
    assert "sig" not in r and "td_by" not in r and "est_gate" not in r
    ext = struct.pack(decode_log.VTOL_FMT_V6_EXT, 0x13, 2, 10, -6, 448, 7)
    boot6 = struct.pack("<BBHB", 6, 1, 250, 2) + b"fw 3.3.0 role FCU vtol"
    log = tmp_path / "LOG00007.TXT"
    log.write_bytes(_frame("B", boot5) + _frame("V", v5) + _frame("B", boot6) + _frame("V", v5 + ext))
    assert decode_log.main([str(log)]) == 0
    rows = (tmp_path / "LOG00007_vtol.csv").read_text(encoding="utf-8").splitlines()
    header = rows[0].split(",")
    assert len(rows) == 3 and {"sig_support", "td_by", "low_in_land_s", "est_gate_frac"} <= set(header)
    d = decode_log.decode(log.read_bytes())
    r5, r6 = d["vtol"]
    assert "sig" not in r5 and (r6["td_by"], r6["sig_low"], r6["sig_still"], r6["sig_ground"], r6["low_in_land"]) == \
           ("support", 1, 1, 1, 10)
    assert (r6["vz_lpf_mps"], r6["f_excess_mps2"], r6["est_gate"]) == (-6 / 128.0, 3.5, 7)


def test_vtol_fine_flight_record_decodes(vtol_sample):
    (f,) = vtol_sample["d"]["flight"]                       # packed part by part with the fine split
    assert abs(f["h_est_m"] - 4.5) < 1e-3 and abs(f["throttle"] - 0.93) < 1e-4 and abs(f["tilt_deg"] - 2.5) < 1e-2


def test_ballistic_log_keeps_the_v063_command_names():
    """a format-4 (ballistic) log names codes 8..11 as v0.6.3 did (unknown commands); a format-5 log as ON .. DOWN"""
    link = struct.pack("<IBBHHBBbb", 100, 3, 8, 0, 0, 0x48, 0, 127, 0)
    for fmt, want in ((4, "CMD8"), (5, "ON")):
        boot = struct.pack("<BBHB", fmt, 1, 250, 2) + b"fw 3.2.0 role FCU"
        (lk,) = decode_log.decode(_frame("B", boot) + _frame("L", link))["link"]
        assert lk["cmd"] == want and "phase_after" not in lk
    ev = struct.pack("<IBBBB", 100, 0, 1, 0x80 | 9, 0)
    boot4 = struct.pack("<BBHB", 4, 1, 250, 2) + b"fw 3.2.0 role FCU ballistic"
    (e,) = decode_log.decode(_frame("B", boot4) + _frame("E", ev))["events"]
    assert e["cause"] == "cmd CMD9" and "phase_from" not in e


def test_vtol_constants_match_the_firmware_headers():
    """the decoder's copies of ONB_CAUSE_* (VTOL timers), LOG_V_*, VC_MODE_* and the 'V' layout equal the C headers"""
    import re
    src = {n: open(os.path.join(bht.CORE, n), encoding="utf-8").read()
           for n in ("onboard.h", "log_record.h", "vertical_ctrl.h")}

    def num(name: str, text: str) -> int:
        m = re.search(rf"#define\s+{name}\s+\(?(0x[0-9A-Fa-f]+|\d+)", text)
        assert m, name
        return int(m.group(1), 0)
    assert decode_log.CAUSE_VTOL == {num("ONB_CAUSE_SPOOL_DONE", src["onboard.h"]): "spool done",
                                     num("ONB_CAUSE_AT_TARGET", src["onboard.h"]): "at target",
                                     num("ONB_CAUSE_TOUCHDOWN", src["onboard.h"]): "touchdown",
                                     num("ONB_CAUSE_LANDED_TIMER", src["onboard.h"]): "LANDED timer"}
    assert decode_log.VTOL_FLAG_BITS == {num(f"LOG_V_{n.upper()}", src["log_record.h"]): n
                                         for n in ("motor_on", "buzzer", "baro_lost", "at_target", "touchdown", "sat_high",
                                                   "sat_low")}
    assert decode_log.VC_MODES == {num(f"VC_MODE_{n}", src["vertical_ctrl.h"]): n for n in ("IDLE", "FLY", "LAND")}
    assert struct.calcsize(decode_log.VTOL_FMT) == num("LOG_VTOL_LEN_V5", src["log_record.h"]) == 26
    assert decode_log.VTOL_LEN_V6 == num("LOG_VTOL_LEN", src["log_record.h"]) == 35          # format 6 (v0.7.1)
    assert decode_log.VC_SIG_BITS == {num(f"VC_SIG_{n.upper()}", src["vertical_ctrl.h"]): f"sig_{n}"
                                      for n in ("low", "still", "support", "floor", "ground")}
    from sim import vertical_ctrl as vc
    assert decode_log.VC_TD == vc.TD_NAMES                  # the reference's td_by names (v0.7.1b adds 4 "all")
    for n in ("none", "still", "support", "floor", "all"):  # every VC_TD_* the C header defines has the same code
        if re.search(rf"#define\s+VC_TD_{n.upper()}\b", src["vertical_ctrl.h"]):
            assert decode_log.VC_TD[num(f"VC_TD_{n.upper()}", src["vertical_ctrl.h"])] == n
        else:
            assert n == "all"                               # only the v0.7.1b code may still be missing in C
    nav_h = open(os.path.join(bht.CORE, "..", "nav", "nav.h"), encoding="utf-8").read()
    assert decode_log.NAV_F_EST_GATE == num("NAV_F_EST_GATE", nav_h) == 0x80
    assert decode_log.NAV_F_EST_GATE not in decode_log.NAV_FLAG_BITS     # bit 7 is free in the nlink contract
    assert decode_log.SHIFT_INTEG == num("LOG_SHIFT_INTEG", src["log_record.h"])
    assert num("LOG_FORMAT_VERSION_VTOL", src["log_record.h"]) == decode_log.FORMAT_VTOL
    assert num("LOG_FORMAT_VERSION", src["log_record.h"]) == decode_log.FORMAT_BALLISTIC == decode_log.FORMAT_VERSION
    assert (num("LOG_EVENT2_LEN", src["log_record.h"]), num("LOG_LINK2_LEN", src["log_record.h"])) == (10, 15)


def test_main_writes_the_vtol_csv(vtol_sample, capsys):
    path = str(vtol_sample["path"])
    assert decode_log.main([path]) == 0
    out = capsys.readouterr().out
    assert "mode vtol, profile demo" in out and "[CLIMB -> HOLD]: at target" in out and "VTOL: 3 'V' records" in out
    assert "confirmed by support" in out and "V10 counter max 6.00 s" in out and "[LAND -> LANDED]: V10 LAND" in out
    base = os.path.splitext(path)[0]
    header = open(base + "_vtol.csv", encoding="utf-8").read().splitlines()[0].split(",")
    assert {"phase", "h_tgt_m", "h_ref_m", "vz_cmd_mps", "u", "u_ff", "integ", "touchdown", "motor_s"} <= set(header)
    assert {"sig", "sig_low", "sig_still", "sig_support", "sig_floor", "sig_ground", "td_by", "low_in_land",
            "low_in_land_s", "vz_lpf_mps", "f_excess_mps2", "est_gate", "est_gate_frac"} <= set(header)


# ---------------------------------------------------------------- v0.8: log format 7, the 'G' record
GPS_SAMPLE_C = r"""
#include <stdio.h>
#include <string.h>
#include "log_record.h"

static void put(FILE *f, uint8_t type, const uint8_t *p, uint8_t n)
{
    uint8_t fr[LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD];
    const uint16_t k = log_frame(type, p, n, fr);
    fwrite(fr, 1, k, f);
}

int main(int argc, char **argv)
{
    static const char info[] = "fw 3.4.0 role FCU vtol BUDGET_90mm_6S params 0123456789abcdef gains fedcba9876543210";
    uint8_t boot[96], pay[LOG_MAX_PAYLOAD], n = 0, k;
    log_gps_t g, g2;
    FILE *f;
    if (argc < 2) return 1;
    f = fopen(argv[1], "wb");
    if (!f) return 1;
    boot[n++] = LOG_FORMAT_VERSION_GPS; boot[n++] = 0x01; boot[n++] = 250; boot[n++] = 0; boot[n++] = 2;
    memcpy(boot + n, info, sizeof info - 1); n = (uint8_t)(n + sizeof info - 1);
    put(f, LOG_TYPE_BOOT, boot, n);
    g.tick = 12500; g.gps_flags = 0x7B; g.num_sv = 9; g.hacc_dm = 18; g.reset_count = 2; g.pos_mode = 2; g.requests = 0;
    g.gflags = LOG_G_ENGAGED | LOG_G_HOLD | LOG_G_FRESH | LOG_G_GATE_OK; g.set_age = 3;
    g.p_cm[0] = 123; g.p_cm[1] = -45; g.v_cms[0] = -12; g.v_cms[1] = 7; g.hold_cm[0] = 100; g.hold_cm[1] = -20;
    g.dist_dm = 3; g.tilt_mrad[0] = -87; g.tilt_mrad[1] = 52; g.head_q15[0] = 32767; g.head_q15[1] = 0;
    g.sets = 2500; g.broken = 1; g.bg_late = 4;
    if (log_pack_gps(&g, pay) != LOG_GPS_LEN) return 2;
    g2 = g;
    if (log_pack_gps_in_place(&g2) != LOG_GPS_LEN || memcmp(pay, &g2, LOG_GPS_LEN) != 0) return 3;
    put(f, LOG_TYPE_GPS, pay, LOG_GPS_LEN);
    /* the "none" codes: no set, no hold point, no distance, no accuracy; a V11 request, mode ATT */
    memset(&g, 0, sizeof g);
    g.tick = 12600; g.hacc_dm = 255; g.pos_mode = 4; g.requests = 0x01; g.gflags = LOG_G_NO_SET; g.set_age = 0xFF;
    g.hold_cm[0] = g.hold_cm[1] = LOG_HOLD_NONE; g.dist_dm = LOG_DIST_NONE;
    k = log_pack_gps(&g, pay);
    put(f, LOG_TYPE_GPS, pay, k);
    fclose(f);
    return 0;
}
"""


@pytest.fixture(scope="module")
def gps_sample(tmp_path_factory) -> dict:
    tmp = tmp_path_factory.mktemp("gps")
    src = tmp / "gps_sample.c"
    src.write_text(GPS_SAMPLE_C, encoding="utf-8")
    exe = str(tmp / ("gps_sample" + (".exe" if os.name == "nt" else "")))
    r = subprocess.run([sys.executable, "-m", "ziglang", "cc", *bht.CFLAGS, "-Werror", "-I", bht.CORE, str(src),
                        os.path.join(bht.CORE, "log_record.c"), "-o", exe], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stdout + r.stderr
    path = tmp / "gps.bin"
    r = subprocess.run([exe, str(path)], capture_output=True, text=True, encoding="utf-8", errors="replace")
    assert r.returncode == 0, f"gps_sample exit {r.returncode} (2/3: log_pack_gps length / in-place bytes)"
    d = decode_log.decode(path.read_bytes())
    d["path"] = path
    return d


def test_gps_record_from_the_c_packer(gps_sample):
    d = gps_sample
    assert d["bad"] == 0 and d["skipped"] == 0 and len(d["gps"]) == 2
    (b,) = d["boot"]
    assert b["format"] == 7 and b["mode"] == "vtol" and b["gps_hold"] is True
    g = d["gps"][0]
    assert (g["tick"], g["t_s"], g["gps_flags"], g["num_sv"], g["hacc_m"], g["nav_resets"]) == (12500, 50.0, 0x7B, 9, 1.8, 2)
    assert (g["gps_fix_ok"], g["gps_gate_ok"], g["gps_head_ok"], g["gps_fix3d"], g["gps_cfg_ok"], g["gps_init"],
            g["gps_ver_ok"]) == (1, 1, 0, 1, 1, 1, 1)
    assert g["pos_mode"] == "POS" and g["requests"] == 0 and not any(g[f"req_{n}"] for n in decode_log.POS_REQS.values())
    assert (g["g_engaged"], g["g_hold"], g["g_fresh"], g["g_gate_ok"], g["g_no_set"], g["set_age_ticks"]) == (1, 1, 1, 1, 0, 3)
    assert (g["p_x_m"], g["p_y_m"], g["v_x_mps"], g["v_y_mps"], g["hold_x_m"], g["hold_y_m"], g["dist_m"]) == \
           (1.23, -0.45, -0.12, 0.07, 1.0, -0.2, 0.3)
    assert (g["tilt_x_rad"], g["tilt_y_rad"]) == (-0.087, 0.052) and abs(g["tilt_sp_deg"] - 5.8077) < 1e-3
    assert abs(g["head_cos"] - 1.0) < 1e-9 and g["head_sin"] == 0.0 and g["heading_deg"] == 0.0
    assert (g["slot_sets"], g["slot_broken"], g["pos_late_ticks"]) == (2500, 1, 4)
    n = d["gps"][1]
    assert n["hacc_m"] is None and n["hold_x_m"] is None and n["hold_y_m"] is None and n["dist_m"] is None
    assert n["set_age_ticks"] is None and n["g_no_set"] == 1 and n["pos_mode"] == "ATT" and n["req_gps_lost"] == 1
    assert n["heading_deg"] is None


def test_gps_constants_match_the_header_and_the_references():
    import re
    from sim import gps_nav as gn
    from sim import position_ctrl as pc
    h = open(os.path.join(bht.CORE, "log_record.h"), encoding="utf-8").read()

    def num(name: str) -> int:
        m = re.search(rf"#define\s+{name}\s+\(?(0x[0-9A-Fa-f]+|\d+)", h)
        assert m, name
        return int(m.group(1), 0)
    assert decode_log.GPS_LEN == num("LOG_GPS_LEN") == 40
    assert decode_log.FORMAT_VERSION_GPS == num("LOG_FORMAT_VERSION_GPS")
    assert decode_log.G_FLAG_BITS == {num(f"LOG_G_{n.upper()}"): f"g_{n}"
                                      for n in ("engaged", "hold", "fresh", "gate_ok", "no_set")}
    assert (decode_log.HOLD_NONE, decode_log.DIST_NONE) == (num("LOG_HOLD_NONE"), num("LOG_DIST_NONE"))
    assert decode_log._GF_NAMES == gn.GF_NAMES and decode_log.GF_NAMES == gn.GF_NAMES
    assert decode_log._POS_MODES == pc.MODE_NAMES and decode_log._POS_REQS == pc.REQ_NAMES


def test_main_writes_the_gps_csv(gps_sample, capsys):
    path = str(gps_sample["path"])
    assert decode_log.main([path]) == 0
    out = capsys.readouterr().out
    assert "GPS / position hold: 2 'G' records" in out and "LAND requests gps_lost" in out
    header = open(os.path.splitext(path)[0] + "_gps.csv", encoding="utf-8").read().splitlines()[0].split(",")
    assert {"pos_mode", "dist_m", "hold_x_m", "p_x_m", "tilt_sp_deg", "gps_fix_ok", "req_fence", "pos_late_ticks"} <= set(header)


# ---------------------------------------------------------------- v0.8.1: log format 8, the 'G' record in its GNS form
GNS_SAMPLE_C = r"""
#include <stdio.h>
#include <string.h>
#include "log_record.h"

static void put(FILE *f, uint8_t type, const uint8_t *p, uint8_t n)
{
    uint8_t fr[LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD];
    const uint16_t k = log_frame(type, p, n, fr);
    fwrite(fr, 1, k, f);
}

int main(int argc, char **argv)
{
    static const char info[] = "fw 3.5.0 role FCU vtol BUDGET_90mm_6S params 0123456789abcdef gains fedcba9876543210";
    uint8_t boot[96], pay[LOG_MAX_PAYLOAD], n = 0, k;
    log_gns_t g, g2;
    FILE *f;
    if (argc < 2) return 1;
    f = fopen(argv[1], "wb");
    if (!f) return 1;
    boot[n++] = LOG_FORMAT_VERSION_GNS; boot[n++] = 0x01; boot[n++] = 250; boot[n++] = 0; boot[n++] = 2;
    memcpy(boot + n, info, sizeof info - 1); n = (uint8_t)(n + sizeof info - 1);
    put(f, LOG_TYPE_BOOT, boot, n);
    /* engaged in POS with the hold point captured, a wind trim, the newest TILT part and the flown setpoint */
    g.tick = 12500; g.gps_flags = 0x7B; g.num_sv = 9; g.hacc_dm = 18; g.pc = 2 | 0x08; g.set_age = 3;
    g.gflags = LOG_G4_ENGAGED | LOG_G4_HOLD | LOG_G4_FRESH | LOG_G4_GATE_OK;
    g.pos_cm[0] = 123; g.pos_cm[1] = -45; g.vel_cms[0] = -12; g.vel_cms[1] = 7; g.trim[0] = 150; g.trim[1] = -80;
    g.tilt_tgt[0] = -870; g.tilt_tgt[1] = 520; g.tilt_mrad[0] = -87; g.tilt_mrad[1] = 52;
    g.dist_dm = 13; g.sets = 2500; g.broken = 1; g.tilts = 2504; g.bg_late = 0;
    if (log_pack_gns(&g, pay) != LOG_GNS_LEN) return 2;
    g2 = g;
    if (log_pack_gns_in_place(&g2) != LOG_GNS_LEN || memcmp(pay, &g2, LOG_GNS_LEN) != 0) return 3;
    put(f, LOG_TYPE_GPS, pay, LOG_GNS_LEN);
    /* the "none" codes: no set, no TILT part since boot, the FCU's own GPS_LOST (V11), mode ATT in the TRIM fallback */
    memset(&g, 0, sizeof g);
    g.tick = 12600; g.hacc_dm = 255; g.pc = 4 | (0x01 << 4); g.set_age = 0xFF;
    g.gflags = LOG_G4_NO_SET | LOG_G4_NO_TILT | LOG_G4_OWN_LOST | LOG_G4_FALLBACK;
    g.tilt_tgt[0] = -32768; g.dist_dm = LOG_DIST_NONE;
    k = log_pack_gns(&g, pay);
    put(f, LOG_TYPE_GPS, pay, k);
    fclose(f);
    return 0;
}
"""


@pytest.fixture(scope="module")
def gns_sample(tmp_path_factory) -> dict:
    tmp = tmp_path_factory.mktemp("gns")
    src = tmp / "gns_sample.c"
    src.write_text(GNS_SAMPLE_C, encoding="utf-8")
    exe = str(tmp / ("gns_sample" + (".exe" if os.name == "nt" else "")))
    r = subprocess.run([sys.executable, "-m", "ziglang", "cc", *bht.CFLAGS, "-Werror", "-I", bht.CORE, str(src),
                        os.path.join(bht.CORE, "log_record.c"), "-o", exe], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stdout + r.stderr
    path = tmp / "gns.bin"
    r = subprocess.run([exe, str(path)], capture_output=True, text=True, encoding="utf-8", errors="replace")
    assert r.returncode == 0, f"gns_sample exit {r.returncode} (2/3: log_pack_gns length / in-place bytes)"
    d = decode_log.decode(path.read_bytes())
    d["path"] = path
    return d


def test_gns_record_from_the_c_packer(gns_sample):
    d = gns_sample
    assert d["bad"] == 0 and d["skipped"] == 0 and len(d["gps"]) == 2
    (b,) = d["boot"]
    assert b["format"] == 8 and b["mode"] == "vtol" and b["gps_hold"] is True
    g = d["gps"][0]
    assert (g["tick"], g["t_s"], g["gps_flags"], g["num_sv"], g["hacc_m"]) == (12500, 50.0, 0x7B, 9, 1.8)
    assert g["pos_mode"] == "POS" and g["hold"] == 1 and g["requests"] == 0
    assert (g["g_engaged"], g["g_hold"], g["g_fresh"], g["g_gate_ok"], g["g_no_set"], g["g_fallback"], g["g_own_lost"],
            g["g_no_tilt"], g["set_age_ticks"]) == (1, 1, 1, 1, 0, 0, 0, 0, 3)
    assert (g["pos_x_m"], g["pos_y_m"], g["v_x_mps"], g["v_y_mps"], g["dist_m"]) == (1.23, -0.45, -0.12, 0.07, 1.3)
    assert (g["trim_x_rad"], g["trim_y_rad"], g["tilt_tgt_x_rad"], g["tilt_tgt_y_rad"]) == (0.015, -0.008, -0.087, 0.052)
    assert (g["tilt_x_rad"], g["tilt_y_rad"]) == (-0.087, 0.052) and abs(g["tilt_sp_deg"] - 5.8077) < 1e-3
    assert (g["slot_sets"], g["slot_broken"], g["slot_tilts"], g["pos_late_ticks"]) == (2500, 1, 2504, 0)
    assert "nav_resets" not in g and "heading_deg" not in g          # the v0.8 fields the GNS slot no longer carries
    n = d["gps"][1]
    assert n["hacc_m"] is None and n["dist_m"] is None and n["set_age_ticks"] is None and n["tilt_tgt_x_rad"] is None
    assert n["pos_mode"] == "ATT" and n["req_gps_lost"] == 1 and n["g_own_lost"] == 1 and n["g_fallback"] == 1
    assert n["g_no_set"] == 1 and n["g_no_tilt"] == 1


def test_gns_constants_match_the_headers():
    import re
    h = open(os.path.join(bht.CORE, "log_record.h"), encoding="utf-8").read()
    nh = open(os.path.join(bht.CORE, "nlink_proto.h"), encoding="utf-8").read()

    def num(src: str, name: str) -> int:
        m = re.search(rf"#define\s+{name}\s+\(?(-?0x[0-9A-Fa-f]+|-?\d+)", src)
        assert m, name
        return int(m.group(1), 0)
    assert decode_log.GNS_LEN == num(h, "LOG_GNS_LEN") == 40
    assert decode_log.FORMAT_VERSION_GNS == num(h, "LOG_FORMAT_VERSION_GNS")
    assert decode_log.G4_FLAG_BITS == {num(h, f"LOG_G4_{n.upper()}"): f"g_{n}"
                                       for n in ("engaged", "hold", "fresh", "gate_ok", "no_set", "fallback", "own_lost",
                                                 "no_tilt")}
    assert decode_log.TILT_NONE == num(nh, "NL_TILT_NONE") and decode_log.TILT_SCALE == num(nh, "NL_TILT_SCALE")
    assert (decode_log.PC_MODE_MASK, decode_log.PC_HOLD, decode_log.PC_REQ_SHIFT) == (
        num(nh, "NL_PC_MODE_MASK"), num(nh, "NL_PC_HOLD"), num(nh, "NL_PC_REQ_SHIFT"))


def test_main_writes_the_gns_csv(gns_sample, capsys):
    path = str(gns_sample["path"])
    assert decode_log.main([path]) == 0
    out = capsys.readouterr().out
    assert "GPS / position hold (GNS board): 2 'G' records" in out and "LAND requests gps_lost" in out
    assert "the FCU's own GPS_LOST in 1" in out
    header = open(os.path.splitext(path)[0] + "_gps.csv", encoding="utf-8").read().splitlines()[0].split(",")
    assert {"pos_mode", "hold", "dist_m", "pos_x_m", "trim_x_rad", "tilt_tgt_x_rad", "tilt_sp_deg", "g_fallback",
            "g_own_lost", "slot_tilts"} <= set(header)
