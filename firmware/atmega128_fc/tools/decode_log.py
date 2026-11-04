#!/usr/bin/env python3
"""Decode an OpenLog file written by the ATmega128 flight control unit (FCU) into CSV.

Frame format (src/core/log_record.h, LOG_FORMAT_VERSION 4 / LOG_FORMAT_VERSION_VTOL 6): 0xA5 0x5A | type | len | payload |
fletcher16 LE over (type, len, payload).  Types: 'F' flight record (50 Hz), 'E' onboard-supervisor
transition, 'L' uplink CMD the COM forwarded (what the supervisor did with it, RSSI/SNR when the
radio reports them), 'C' one IL_STAT of the COM chip (v0.6: FCU tick + the 36-byte payload of
src/core/ilink_proto.h), 'N' NAV link (v0.6.3, 1 Hz: the navigation frames the FCU received from the
NAV chip, the SPI slave's counters, the NAV's own loop time / flags / reset cause - bits 4..0 its MCUCSR,
named as ``nav_reset``, bits 7..5 the NAV_RC_* boot / link results imu_init_fail 0x80, baro_init_fail
0x40, tag_mismatch 0x20, named as ``nav_rc`` and one 0/1 column each), 'T' text, 'B' boot
record (its text names the role, "role FCU"), 'S' status (1 Hz: stack high-water mark, FCU error
counters, the COM's link counters from its newest IL_STAT, the FCU's ilink counters).  v0.6.3 keeps
every format-4 layout and only adds 'N' (a decoder without it skips the record); since the sensors
moved to the NAV, the 'F' attitude / accel / altitude / tilt fields and the 'S' twi_errors / baro_fails
/ imu_fails are the NAV's values from its frames, and 'F' overruns counts the FCU ticks run without a
valid navigation frame.  Format 3 logs (v0.5, one chip, 28-byte 'S') are still decoded.  Bytes
between frames (power-up noise, a torn last frame) are skipped and counted; a frame whose checksum
fails is skipped.

Format 5 (v0.7, the VTOL build; the ballistic build keeps writing format 4): the boot text names the mode
("role FCU vtol"); 'E' carries the VTOL phases before / after (10 bytes; a phase change inside POWERED is an
'E' record with from == to == POWERED, causes "spool done", "at target", "touchdown", "LANDED timer"),
'L' the phase after the command (15 bytes), and the 'V' record (10 Hz) the vertical mission: phase,
supervisor / controller flags, reason, the controller's mode, the altitude target, reference, vertical-speed
command, throttle, feed-forward, integrator, touchdown counter and the cumulative motor time (_vtol.csv).
Command names follow the log's mode: codes 8..11 are ON / OFF / UP / DOWN in a VTOL log and "CMD8".."CMD11"
(unknown commands, as v0.6.3 logged them) in a format-4 log.

Format 6 (v0.7.1, the VTOL build; format 5 logs of v0.7.0 still decode): only the 'V' record grows, from 26 to 35 bytes
(log_record.h log_vtol_t): the touchdown detector v2 - the controller's signature bits of the tick (low / still /
support / floor / ground: sig_* columns), which signature confirmed the touchdown (td_by: still / support / floor), the
V10 LAND-floor counter (low_in_land ticks and seconds), the still signature's filtered v_z (vz_lpf_mps) and the support
signature's filtered excess body-z specific force (f_excess_mps2) - and est_gate, how many navigation frames since the
previous 'V' record carried the NAV's estimator-correction flag (nav.h NAV_F_EST_GATE; est_gate_frac = est_gate / the
ticks between the records).  The 'N' record's nav_flags bit 7 is that flag of the newest frame (nav_est_gate).  The
boot text after the boot record names the build profile ("VTOL profile field: ..." / "demo"), reported as boot
``profile``.  The V8 / V9 / V10 interlocks are 'E' records with reason codes 21 / 22 / 23 (names from
sim/link_proto.py).

Format 7 (v0.8, the VTOL build with GPS position hold; a VTOL build without it - fcu_nogps - keeps writing format 6): every
format-6 record unchanged, plus the 'G' record (log_record.h log_gps_t, 40 bytes, every 4th 'V' slot: 2.5 Hz, 'V' then
7.5 Hz): the newest complete GPS slot set from the NAV (GPS flags - fix_ok, gate_ok, head_ok, fix3d, cfg_ok, init, ver_ok
and incons (bit 7: GPS-IMU inconsistent), satellites, hAcc, the NAV's position-reset counter, position / velocity in the
NAV frame W: x north, y west, heading cos / sin), the position controller's published mode (OFF / VEL / POS / DR / ATT) and LAND requests (V11 gps_lost,
V12 fence, V13 gps_implausible, V14 authority), engaged / hold / fresh / gate flags, the hold point in the NAV frame, the
distance to it, the tilt setpoint of the next attitude step, the slot receiver's sets / broken cycles and the ticks whose
position work was finished late (_gps.csv).  The boot text names the position-hold gains ("GPS position hold on: ...").

Format 8 (v0.8.1, the four-chip VTOL build with GPS position hold - the GNS board runs the GPS filter and the position
controller, the FCU its GNS chain; fcu_nogps keeps format 6): every format-6 record unchanged, the 'G' record in its GNS
form (log_record.h log_gns_t, 40 bytes, the same 2.5 Hz slot): the newest complete GNS slot set (GPS flags, satellites,
hAcc, the GNS's STATUS byte - its position mode, the hold flag, its latched LAND requests -, POS relative to the hold point
when the hold flag is set, VEL, the wind TRIM), the newest TILT part (None while the GNS's position loop is not engaged),
the tilt setpoint the next attitude step flies, the receiver's flags (fresh, gate_ok, no set, the TRIM fallback, the FCU's
own GPS_LOST request, no TILT part since boot) and counters (sets, broken cycles, TILT parts, ticks whose tilt work was
finished late).  The v0.8 columns the GNS slot no longer carries (the NAV's reset counter, the hold point in the NAV
frame, the heading) are absent; _gps.csv holds the format of the log's boot record.
Flag / mode / request names come from sim/gps_nav.py GF_NAMES and sim/position_ctrl.py MODE_NAMES / REQ_NAMES.

    python decode_log.py LOG00001.TXT                 # -> LOG00001_flight.csv (+ _link.csv, _com.csv, _nav.csv,
                                                      #    _vtol.csv, _gps.csv) + summary
    python decode_log.py LOG00001.TXT --out run.csv

Invariant: this file and log_record.c change together; LOG_FORMAT_VERSION is checked against
the boot record.  State, command and reason names come from sim/link_proto.py (the wire codes the
supervisor and the ground station share) when it can be imported; the 'C' layout is
sim/ilink_proto.py STAT_FMT / STAT_FIELDS when it can be imported (a copy otherwise, checked equal
by test_decode_log.py).
"""
from __future__ import annotations

import argparse
import csv
import os
import struct
import sys

# named as in src/core/log_record.h: LOG_FORMAT_VERSION (4) is the ballistic build's format, LOG_FORMAT_VERSION_VTOL
# (6, v0.7.1; 5 in v0.7.0) the VTOL build's ('E' / 'L' carry the phases, 'V' records)
FORMAT_VERSION, FORMAT_VERSION_VTOL, FORMAT_VERSION_GPS, FORMAT_VERSION_GNS = 4, 6, 7, 8
FORMAT_BALLISTIC, FORMAT_VTOL = FORMAT_VERSION, FORMAT_VERSION_VTOL
FORMAT_VTOL_FIRST = 5                                   # v0.7.0's VTOL format: a log at or above it is a VTOL log
FORMATS_DECODED = (3, 4, 5, 6, 7, 8)
# fixed point: value = raw / 2^shift (LOG_SHIFT_* in src/core/log_record.h)
SHIFT = dict(q=15, rate=10, accel=7, angle=13, moment=12, h=10, vz=7, throttle=15, vcell=12, tilt=7, thrust=8)
SCALE = {k: float(1 << v) for k, v in SHIFT.items()}

try:  # wire codes of SPEC 10.3 (project root on sys.path)
    sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..")))
    from sim import link_proto as _lp
    STATES = dict(_lp.STATE_NAMES)
    CMDS = dict(_lp.CMD_NAMES)
    REASONS = dict(_lp.REASON_NAMES)
    PHASES = dict(_lp.PHASE_NAMES)
    CMD_MAX_BALLISTIC = _lp.CMD_KILL
except Exception:  # pragma: no cover - standalone copy of the decoder
    STATES = {0: "SAFE", 1: "ARMED", 2: "POWERED", 3: "UNPOWERED", 4: "RELEASED", 5: "ABORT"}
    CMDS = {0: "NONE", 1: "ARM", 2: "DISARM", 3: "LAUNCH", 4: "THROTTLE", 5: "CUT", 6: "RELEASE", 7: "KILL",
            8: "ON", 9: "OFF", 10: "UP", 11: "DOWN"}
    REASONS = {0: "none"}
    PHASES = {0: "NONE", 1: "SPOOL", 2: "CLIMB", 3: "HOLD", 4: "LAND", 5: "LANDED"}
    CMD_MAX_BALLISTIC = 7
EXEC_NONE, EXEC_REPEAT = 0xFE, 0xFF                     # onboard.h ONB_EXEC_*
CAUSE_CMD, CAUSE_RELEASE_TIMER = 0x80, 0x7E             # onboard.h ONB_CAUSE_*
# v0.7 VTOL timers (onboard.h ONB_CAUSE_SPOOL_DONE .. ONB_CAUSE_LANDED_TIMER; copy, checked by test_decode_log.py)
CAUSE_VTOL = {0x7D: "spool done", 0x7C: "at target", 0x7B: "touchdown", 0x7A: "LANDED timer"}
# 'V' record (format 5, log_record.h log_vtol_t): tick u32, phase, vflags, reason, vc_mode u8, h_tgt_dm i16,
# h_ref i32 (h shift), vz_cmd i16 (vz), u u16, u_ff u16 (throttle), integ i16 (shift 15), td_count u16, motor_ticks u16
VTOL_FMT = "<IBBBBhihHHhHH"
VTOL_FLAG_BITS = {0x01: "motor_on", 0x02: "buzzer", 0x04: "baro_lost", 0x08: "at_target", 0x10: "touchdown",
                  0x20: "sat_high", 0x40: "sat_low"}     # log_record.h LOG_V_*
VC_MODES = {0: "IDLE", 1: "FLY", 2: "LAND"}             # vertical_ctrl.h VC_MODE_*
SHIFT_INTEG = 15                                        # log_record.h LOG_SHIFT_INTEG
# format 6 (v0.7.1) 'V' extension behind the format-5 fields: sig u8, td_by u8, low_in_land u16, vz_lpf i16 (vz shift),
# f_excess i16 (accel shift), est_gate u8
VTOL_FMT_V6_EXT = "<BBHhhB"
VTOL_LEN_V5 = struct.calcsize(VTOL_FMT)                 # log_record.h LOG_VTOL_LEN_V5 (26)
VTOL_LEN_V6 = VTOL_LEN_V5 + struct.calcsize(VTOL_FMT_V6_EXT)   # LOG_VTOL_LEN (35)
VC_SIG_BITS = {0x01: "sig_low", 0x02: "sig_still", 0x04: "sig_support", 0x08: "sig_floor",
               0x10: "sig_ground"}                      # vertical_ctrl.h VC_SIG_* (copy, checked by test_decode_log.py)
# vertical_ctrl.h VC_TD_* = sim/vertical_ctrl.py TD_NAMES (4 "all": the v0.7.1b gating's support + floor (+ still)
# confirmation above the flare band, in the reference; a header without VC_TD_ALL never writes it)
VC_TD = {0: "none", 1: "still", 2: "support", 3: "floor", 4: "all"}
V_RECORD_TICKS_NOMINAL = 25                             # main.c: one 'V' record per 25 ticks (10 Hz at 250 Hz)
NAV_F_EST_GATE = 0x80                                   # nav/nav.h NAV_F_EST_GATE ('N' nav_flags bit 7, VTOL NAV)
RSSI_NONE = 127                                         # log_record.h LOG_RSSI_NONE
RESET_BITS = {0x01: "power_on", 0x02: "external", 0x04: "brown_out", 0x08: "watchdog", 0x10: "jtag"}
FLAG_BITS = {0x01: "control_active", 0x02: "saturated", 0x04: "release", 0x08: "link_ok", 0x10: "interlock",
             0x20: "arm_switch", 0x40: "sensors_ok", 0x80: "bench"}
AUX_BITS = {0x01: "aligned", 0x02: "recovery", 0x04: "reply_pending", 0x08: "frame_rx", 0x10: "ack_rejected",
            0x20: "radio_ok"}                           # radio_ok = an IL_STAT with RADIO_OK within 3 s (v0.6: 2 s)
# 'F' payload: tick u32, state, flags, reason, aux_bits u8, loop_us_last, loop_us_max, overruns u16,
# q 4xi16, gyro_f 3xi16, accel 3xi16, att_err 3xi16, moments 3xi16, integ 3xi16, delta 4xi16,
# pwm 6xu16, h_est i32, vz i16, h_baro i32, throttle u16, v_cell u16, tilt i16, thrust u16
FLIGHT_FMT = "<IBBBBHHH" + "4h" + "3h" * 5 + "4h" + "6H" + "ihiHHhH"
FLIGHT_COLS = (["tick", "t_s", "state", "flags"] + list(FLAG_BITS.values()) + ["reason", "aux_bits"] + list(AUX_BITS.values())
               + ["loop_us_last", "loop_us_max", "overruns"]
               + [f"q{c}" for c in "wxyz"] + [f"gyro_f_{c}" for c in "xyz"] + [f"accel_{c}" for c in "xyz"]
               + [f"att_err_{c}" for c in "xyz"] + [f"moment_{c}" for c in "xyz"] + [f"integ_{c}" for c in "xyz"]
               + [f"delta_{i}" for i in range(1, 5)]
               + ["pwm_vane1_us", "pwm_vane2_us", "pwm_vane3_us", "pwm_vane4_us", "pwm_esc_us", "pwm_release_us"]
               + ["h_est_m", "vz_est_mps", "h_baro_m", "throttle", "v_cell", "tilt_deg", "thrust_est_n"])
STATUS_FMT_V3 = "<I11HBb"                                # format 3 (v0.5): 28 bytes
STATUS_FMT = STATUS_FMT_V3 + "5HBB"                      # format 4 (v0.6): + FCU ilink counters, COM age / flags = 40 bytes
COM_AGE_NONE = 255                                       # log_record.h LOG_COM_AGE_NONE
COM_TAG_MISMATCH = 0x80                                  # log_record.h LOG_COM_TAG_MISMATCH
# 'C' payload: FCU tick u32 + IL_STAT (src/core/ilink_proto.h il_pack_stat)
_STAT_FMT = "<IBBBbbB" + "H" * 13
_STAT_FIELDS = ("com_ticks", "reset_cause", "flags", "radio_version", "rssi_dbm", "snr_q4", "params_tag", "dec_frames",
                "dec_bad_crc", "dec_bad_len", "dec_other", "cmd_forwarded", "radio_rx_errors", "radio_rx_overflow",
                "radio_tx_timeouts", "replies", "replies_dropped", "il_bad", "il_tx_dropped", "loop_us_max")
try:  # the reference layout (project root on sys.path, see above)
    from sim import ilink_proto as _il
    STAT_FMT, STAT_FIELDS = _il.STAT_FMT, tuple(_il.STAT_FIELDS)
    STAT_RADIO_OK, STAT_BOOT, STAT_SX127X = _il.STAT_RADIO_OK, _il.STAT_BOOT, _il.STAT_SX127X
except Exception:  # pragma: no cover - standalone copy of the decoder
    STAT_FMT, STAT_FIELDS = _STAT_FMT, _STAT_FIELDS
    STAT_RADIO_OK, STAT_BOOT, STAT_SX127X = 0x01, 0x02, 0x04
COM_FMT = "<I" + STAT_FMT[1:]
# 'N' payload (v0.6.3, log_record.h log_nav_t): tick u32, frames, bad, timeouts, seq_lost, resyncs, spi_overruns,
# nav_loop_us_max u16, nav_flags, nav_reset_cause u8
NAV_FMT = "<I7HBB"
NAV_FIELDS = ("tick", "frames", "bad", "timeouts", "seq_lost", "resyncs", "spi_overruns", "nav_loop_us_max", "nav_flags",
              "nav_reset_cause")
_NAV_FLAG_BITS = {0x01: "imu_ok", 0x02: "baro_ok", 0x04: "aligned", 0x08: "gyro_sat", 0x10: "est_valid",
                  0x20: "baro_new", 0x40: "nav_boot"}    # src/core/nlink_proto.h NL_F_* (copy, checked by the test)
# nav_reset_cause bits 7..5 (v0.6.3, src/nav/nav.h NAV_RC_*; copy, checked by the test); bits 4..0 are the NAV's MCUCSR
_NAV_RC_BITS = {0x20: "tag_mismatch", 0x40: "baro_init_fail", 0x80: "imu_init_fail"}
_NAV_RC_MCUCSR_MASK = 0x1F
NAV_FLAG_BITS, NAV_RC_BITS, NAV_RC_MCUCSR_MASK = dict(_NAV_FLAG_BITS), dict(_NAV_RC_BITS), _NAV_RC_MCUCSR_MASK
try:  # the reference flag values (project root on sys.path, see above)
    from sim import nlink_proto as _nl
    NAV_FLAG_BITS = {_nl.F_IMU_OK: "imu_ok", _nl.F_BARO_OK: "baro_ok", _nl.F_ALIGNED: "aligned", _nl.F_GYRO_SAT: "gyro_sat",
                     _nl.F_EST_VALID: "est_valid", _nl.F_BARO_NEW: "baro_new", _nl.F_BOOT: "nav_boot"}
    NAV_RC_BITS, NAV_RC_MCUCSR_MASK = dict(_nl.NAV_RC_NAMES), _nl.NAV_RC_MCUCSR_MASK
except Exception:  # pragma: no cover - standalone copy of the decoder
    pass
NAV_RC_COLS = {b: f"nav_{n}" for b, n in NAV_RC_BITS.items()}     # one 0/1 column per NAV_RC_* bit
# ---- format 7 'G' record (v0.8, log_record.h log_gps_t): tick u32, gps_flags, num_sv, hacc_dm, reset_count, pos_mode,
# requests, gflags, set_age u8, p_cm 2xi16, v_cms 2xi16, hold_cm 2xi16, dist_dm u16, tilt_mrad 2xi16, head_q15 2xi16,
# sets, broken, bg_late u16
GPS_FMT = "<I8B" + "2h" * 3 + "H" + "2h" * 2 + "3H"
GPS_LEN = struct.calcsize(GPS_FMT)                      # log_record.h LOG_GPS_LEN (40)
G_FLAG_BITS = {0x01: "g_engaged", 0x02: "g_hold", 0x04: "g_fresh", 0x08: "g_gate_ok", 0x10: "g_no_set"}   # LOG_G_*
HOLD_NONE, DIST_NONE, HACC_NONE = 0x7FFF, 0xFFFF, 255   # LOG_HOLD_NONE, LOG_DIST_NONE, LP_HACC_NONE
_GF_NAMES = {0x01: "fix_ok", 0x02: "gate_ok", 0x04: "head_ok", 0x08: "fix3d", 0x10: "cfg_ok", 0x20: "init",
             0x40: "ver_ok", 0x80: "incons"}             # sim/gps_nav.py GF_NAMES (copy, checked by test_decode_log.py)
_POS_MODES = {0: "OFF", 1: "VEL", 2: "POS", 3: "DR", 4: "ATT"}
_POS_REQS = {0x01: "gps_lost", 0x02: "fence", 0x04: "gps_implausible", 0x08: "authority"}
GF_NAMES, POS_MODES, POS_REQS = dict(_GF_NAMES), dict(_POS_MODES), dict(_POS_REQS)
# ---- format 8 'G' record (v0.8.1, log_record.h log_gns_t): tick u32, gps_flags, num_sv, hacc_dm, pc, gflags, set_age u8,
# pos_cm 2xi16, vel_cms 2xi16, trim 2xi16 (x 1e4 rad), tilt_tgt 2xi16 (x 1e4 rad, NL_TILT_NONE), tilt_mrad 2xi16, dist_dm,
# sets, broken, tilts, bg_late u16
GNS_FMT = "<I6B" + "2h" * 5 + "H" + "4H"
GNS_LEN = struct.calcsize(GNS_FMT)                      # log_record.h LOG_GNS_LEN (40)
G4_FLAG_BITS = {0x01: "g_engaged", 0x02: "g_hold", 0x04: "g_fresh", 0x08: "g_gate_ok", 0x10: "g_no_set",
                0x20: "g_fallback", 0x40: "g_own_lost", 0x80: "g_no_tilt"}   # LOG_G4_*
TILT_NONE, TILT_SCALE = -32768, 10000.0                  # core/nlink_proto.h NL_TILT_NONE, NL_TILT_SCALE
PC_MODE_MASK, PC_HOLD, PC_REQ_SHIFT = 0x07, 0x08, 4     # core/nlink_proto.h NL_PC_*
try:  # the reference names (project root on sys.path, see above)
    from sim import gps_nav as _gn
    from sim import position_ctrl as _pc
    GF_NAMES, POS_MODES, POS_REQS = dict(_gn.GF_NAMES), dict(_pc.MODE_NAMES), dict(_pc.REQ_NAMES)
except Exception:  # pragma: no cover - standalone copy of the decoder
    pass


def fletcher16(data: bytes) -> int:
    s1 = s2 = 0
    for b in data:
        s1 = (s1 + b) % 255
        s2 = (s2 + s1) % 255
    return (s2 << 8) | s1


def frames(blob: bytes):
    """Yield (type, payload) for every valid frame; returns (skipped_bytes, bad_checksums) at the end."""
    i, n, skipped, bad = 0, len(blob), 0, 0
    while i + 6 <= n:
        if blob[i] != 0xA5 or blob[i + 1] != 0x5A:
            i += 1
            skipped += 1
            continue
        typ, ln = blob[i + 2], blob[i + 3]
        end = i + 4 + ln + 2
        if end > n:
            skipped += n - i
            break
        ck = blob[end - 2] | (blob[end - 1] << 8)
        if fletcher16(blob[i + 2:i + 4 + ln]) != ck:
            bad += 1
            i += 1
            continue
        yield chr(typ), blob[i + 4:i + 4 + ln]
        i = end
    return skipped, bad


def state_name(s: int):
    return STATES.get(s, s)


def reason_name(r: int) -> str:
    return REASONS.get(r, f"reason{r}")


def cmd_name(c: int, vtol: bool = True) -> str:
    """A command code; in a ballistic (format <= 4) log, 8..11 are unknown commands, named as v0.6.3 named them."""
    if not vtol and c > CMD_MAX_BALLISTIC:
        return f"CMD{c}"
    return CMDS.get(c, f"CMD{c}")


def phase_name(p: int) -> str:
    return PHASES.get(p, f"phase{p}")


def cause_name(c: int, vtol: bool = True) -> str:
    if c & CAUSE_CMD:
        return f"cmd {cmd_name(c & 0x7F, vtol)}"
    if c == CAUSE_RELEASE_TIMER:                         # fw09 T2: the VTOL vehicle has no recovery device
        return "abort hold over: motor locked until power cycle (VTOL: no recovery device)" if vtol else "release timer"
    if vtol and c in CAUSE_VTOL:
        return CAUSE_VTOL[c]
    return reason_name(c)


def result_name(code: int) -> str:
    if code == 0:
        return "executed"
    if code == EXEC_NONE:
        return "heartbeat"
    if code == EXEC_REPEAT:
        return "repeat (reply only)"
    return reason_name(code)


def decode_flight(p: bytes, loop_hz: float, counts_per_us: float) -> dict:
    v = struct.unpack(FLIGHT_FMT, p[:struct.calcsize(FLIGHT_FMT)])
    tick, state, flags, reason, aux, lus, lmax, ovr = v[:8]
    k = 8
    q = [x / SCALE["q"] for x in v[k:k + 4]]; k += 4
    gyro = [x / SCALE["rate"] for x in v[k:k + 3]]; k += 3
    acc = [x / SCALE["accel"] for x in v[k:k + 3]]; k += 3
    err = [x / SCALE["angle"] for x in v[k:k + 3]]; k += 3
    mom = [x / SCALE["moment"] for x in v[k:k + 3]]; k += 3
    integ = [x / SCALE["moment"] for x in v[k:k + 3]]; k += 3
    delta = [x / SCALE["angle"] for x in v[k:k + 4]]; k += 4
    pwm = list(v[k:k + 6]); k += 6
    h_est, vz, h_baro, thr, vcell, tilt, thrust = v[k:k + 7]
    row = dict(tick=tick, t_s=tick / loop_hz, state=state_name(state), flags=flags, reason=reason_name(reason), aux_bits=aux,
               loop_us_last=lus, loop_us_max=lmax, overruns=ovr)
    row.update({name: int(bool(flags & bit)) for bit, name in FLAG_BITS.items()})
    row.update({name: int(bool(aux & bit)) for bit, name in AUX_BITS.items()})
    for name, vals in (("q", q), ("gyro_f_", gyro), ("accel_", acc), ("att_err_", err), ("moment_", mom),
                       ("integ_", integ)):
        for c, x in zip("wxyz" if name == "q" else "xyz", vals):
            row[f"{name}{c}"] = x
    for i, x in enumerate(delta, 1):
        row[f"delta_{i}"] = x
    us = [c / counts_per_us for c in pwm]
    return row | {"pwm_vane1_us": us[0], "pwm_vane2_us": us[1], "pwm_vane3_us": us[2],
                  "pwm_vane4_us": us[3], "pwm_esc_us": us[4], "pwm_release_us": us[5],
                  "h_est_m": h_est / SCALE["h"], "vz_est_mps": vz / SCALE["vz"], "h_baro_m": h_baro / SCALE["h"],
                  "throttle": thr / SCALE["throttle"], "v_cell": vcell / SCALE["vcell"], "tilt_deg": tilt / SCALE["tilt"],
                  "thrust_est_n": thrust / SCALE["thrust"]}


def decode_vtol(p: bytes, loop_hz: float, prev_tick: int | None = None) -> dict:
    """'V' record: the vertical mission at 10 Hz (format 5: 26 bytes; format 6: 35 bytes, + the touchdown detector v2
    and the NAV's estimator-correction count - prev_tick, the previous 'V' record's tick, gives est_gate_frac)."""
    (tick, phase, vflags, reason, mode, tgt, h_ref, vz_cmd, u, u_ff, integ, td, motor) = struct.unpack(
        VTOL_FMT, p[:VTOL_LEN_V5])
    row = dict(tick=tick, t_s=tick / loop_hz, phase=phase_name(phase), vflags=vflags)
    row.update({name: int(bool(vflags & bit)) for bit, name in VTOL_FLAG_BITS.items()})
    row.update(reason=reason_name(reason), vc_mode=VC_MODES.get(mode, mode), h_tgt_m=tgt / 10.0,
               h_ref_m=h_ref / SCALE["h"], vz_cmd_mps=vz_cmd / SCALE["vz"], u=u / SCALE["throttle"],
               u_ff=u_ff / SCALE["throttle"], integ=integ / float(1 << SHIFT_INTEG), td_count=td,
               motor_s=motor / loop_hz)
    if len(p) >= VTOL_LEN_V6:                           # format 6 (v0.7.1)
        sig, td_by, low, vz_lpf, f_ex, gate = struct.unpack(VTOL_FMT_V6_EXT, p[VTOL_LEN_V5:VTOL_LEN_V6])
        row["sig"] = sig
        row.update({name: int(bool(sig & bit)) for bit, name in VC_SIG_BITS.items()})
        span = tick - prev_tick if prev_tick is not None and 0 < tick - prev_tick <= 0xFF else V_RECORD_TICKS_NOMINAL
        row.update(td_by=VC_TD.get(td_by, f"td{td_by}"), low_in_land=low, low_in_land_s=low / loop_hz,
                   vz_lpf_mps=vz_lpf / SCALE["vz"], f_excess_mps2=f_ex / SCALE["accel"], est_gate=gate,
                   est_gate_frac=min(1.0, gate / span))
    return row


def decode_gps(p: bytes, loop_hz: float) -> dict:
    """'G' record (format 7, v0.8): the GPS slot set the FCU holds and the position controller's published outputs."""
    v = struct.unpack(GPS_FMT, p[:GPS_LEN])
    tick, gfl, sv, hacc, rc, mode, req, gf, age = v[:9]
    px, py, vx, vy, hx, hy, dist, tx, ty, hc, hs, sets, broken, late = v[9:]
    import math
    row = dict(tick=tick, t_s=tick / loop_hz, gps_flags=gfl)
    row.update({f"gps_{n}": int(bool(gfl & b)) for b, n in GF_NAMES.items()})
    row.update(num_sv=sv, hacc_m=None if hacc == HACC_NONE else hacc / 10.0, nav_resets=rc,
               pos_mode=POS_MODES.get(mode, f"mode{mode}"), requests=req)
    row.update({f"req_{n}": int(bool(req & b)) for b, n in POS_REQS.items()})
    row.update({n: int(bool(gf & b)) for b, n in G_FLAG_BITS.items()})
    row.update(set_age_ticks=None if age == 0xFF else age, p_x_m=px / 100.0, p_y_m=py / 100.0, v_x_mps=vx / 100.0,
               v_y_mps=vy / 100.0, hold_x_m=None if hx == HOLD_NONE else hx / 100.0,
               hold_y_m=None if hy == HOLD_NONE else hy / 100.0,
               dist_m=None if dist == DIST_NONE else dist / 10.0, tilt_x_rad=tx / 1000.0, tilt_y_rad=ty / 1000.0,
               tilt_sp_deg=math.degrees(math.hypot(tx, ty) / 1000.0), head_cos=hc / 32767.0, head_sin=hs / 32767.0,
               heading_deg=math.degrees(math.atan2(hs, hc)) if (hc or hs) else None, slot_sets=sets, slot_broken=broken,
               pos_late_ticks=late)
    return row


def decode_gns(p: bytes, loop_hz: float) -> dict:
    """'G' record (format 8, v0.8.1): the GNS slot set the FCU holds, the newest TILT part and the setpoint flown."""
    import math
    v = struct.unpack(GNS_FMT, p[:GNS_LEN])
    tick, gfl, sv, hacc, pc, gf, age = v[:7]
    px, py, vx, vy, trx, try_, ttx, tty, tx, ty, dist, sets, broken, tilts, late = v[7:]
    req = pc >> PC_REQ_SHIFT
    row = dict(tick=tick, t_s=tick / loop_hz, gps_flags=gfl)
    row.update({f"gps_{n}": int(bool(gfl & b)) for b, n in GF_NAMES.items()})
    row.update(num_sv=sv, hacc_m=None if hacc == HACC_NONE else hacc / 10.0,
               pos_mode=POS_MODES.get(pc & PC_MODE_MASK, f"mode{pc & PC_MODE_MASK}"), hold=int(bool(pc & PC_HOLD)),
               requests=req)
    row.update({f"req_{n}": int(bool(req & b)) for b, n in POS_REQS.items()})
    row.update({n: int(bool(gf & b)) for b, n in G4_FLAG_BITS.items()})
    engaged_part = ttx != TILT_NONE
    row.update(set_age_ticks=None if age == 0xFF else age, pos_x_m=px / 100.0, pos_y_m=py / 100.0, v_x_mps=vx / 100.0,
               v_y_mps=vy / 100.0, trim_x_rad=trx / TILT_SCALE, trim_y_rad=try_ / TILT_SCALE,
               tilt_tgt_x_rad=ttx / TILT_SCALE if engaged_part else None,
               tilt_tgt_y_rad=tty / TILT_SCALE if engaged_part else None,
               dist_m=None if dist == DIST_NONE else dist / 10.0, tilt_x_rad=tx / 1000.0, tilt_y_rad=ty / 1000.0,
               tilt_sp_deg=math.degrees(math.hypot(tx, ty) / 1000.0), slot_sets=sets, slot_broken=broken,
               slot_tilts=tilts, pos_late_ticks=late)
    return row


def defaults_from_params() -> tuple[float, float]:
    """(loop_hz, PWM counts per us) from the project's params.json - used until a boot record says otherwise."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "params.json")
    try:
        import json
        with open(path, "r", encoding="utf-8") as f:
            p = json.load(f)
        fc = p["flight_computer"]
        return float(p["control"]["loop_hz"]), float(fc["f_cpu_hz"]) / float(fc["pwm_timer_prescaler"]) / 1e6
    except (OSError, KeyError, ValueError):
        return 0.0, 0.0


def com_flag_names(flags: int) -> dict:
    """IL_STAT flags -> named columns (backend from IL_STAT_SX127X)."""
    return dict(radio_ok=int(bool(flags & STAT_RADIO_OK)), com_boot=int(bool(flags & STAT_BOOT)),
                backend="sx127x" if flags & STAT_SX127X else "lora_uart")


def decode_status(p: bytes, loop_hz: float) -> dict:
    """'S' record, format 4 (40 bytes) or format 3 (28 bytes, v0.5: the link counters were the vehicle's own decoder)."""
    v4 = len(p) >= struct.calcsize(STATUS_FMT)
    fmt = STATUS_FMT if v4 else STATUS_FMT_V3
    v = struct.unpack(fmt, p[:struct.calcsize(fmt)])
    row = dict(tick=v[0], t_s=v[0] / loop_hz, stack_free=v[1], twi_errors=v[2], uart_dropped=v[3],
               link_frames=v[4], link_bad=v[5], link_other=v[6], radio_errors=v[7], replies=v[8],
               baro_fails=v[9], imu_fails=v[10], vbat_v=v[11] / 1000.0,
               reset=[n for b, n in RESET_BITS.items() if v[12] & b],
               rssi_dbm=None if v[13] == RSSI_NONE else v[13], format=4 if v4 else 3)
    if v4:
        age, flags = v[19], v[20]
        row.update(il_cmds=v[14], il_snaps=v[15], il_bad=v[16], il_uart_errors=v[17], il_dropped=v[18],
                   com_age_s=None if age == COM_AGE_NONE else age, com_flags=flags,
                   com_params_mismatch=int(bool(flags & COM_TAG_MISMATCH)))
        row.update(com_flag_names(flags) if age != COM_AGE_NONE else dict(radio_ok=0, com_boot=0, backend=None))
    return row


def decode_com(p: bytes, loop_hz: float) -> dict:
    """'C' record: the FCU tick it was received in + every IL_STAT field."""
    v = struct.unpack(COM_FMT, p[:struct.calcsize(COM_FMT)])
    row = dict(tick=v[0], t_s=v[0] / loop_hz)
    row.update(zip(STAT_FIELDS, v[1:]))
    row["reset"] = [n for b, n in RESET_BITS.items() if row["reset_cause"] & b]
    row.update(com_flag_names(row["flags"]))
    if row["rssi_dbm"] == RSSI_NONE:
        row["rssi_dbm"] = None
        row["snr_db"] = None
    else:
        row["snr_db"] = row["snr_q4"] / 4.0
    return row


def decode_nav(p: bytes, loop_hz: float) -> dict:
    """'N' record (v0.6.3): the FCU's nlink counters (cumulative) and the NAV's figures from its newest frame."""
    row = dict(zip(NAV_FIELDS, struct.unpack(NAV_FMT, p[:struct.calcsize(NAV_FMT)])))
    row["t_s"] = row["tick"] / loop_hz
    row.update({name: int(bool(row["nav_flags"] & bit)) for bit, name in NAV_FLAG_BITS.items()})
    row["nav_est_gate"] = int(bool(row["nav_flags"] & NAV_F_EST_GATE))    # v0.7.1 VTOL NAV (always 0 from a ballistic NAV)
    rc = row["nav_reset_cause"]
    row["nav_reset"] = [n for b, n in RESET_BITS.items() if rc & NAV_RC_MCUCSR_MASK & b]      # bits 4..0: MCUCSR
    row["nav_rc"] = [n for b, n in NAV_RC_BITS.items() if rc & b]                            # bits 7..5: NAV_RC_*
    row.update({col: int(bool(rc & b)) for b, col in NAV_RC_COLS.items()})
    return row


def decode(blob: bytes, loop_hz: float | None = None, counts_per_us: float | None = None) -> dict:
    d_hz, d_cpu = defaults_from_params()
    loop_hz = loop_hz or d_hz or 1.0
    counts_per_us = counts_per_us or d_cpu or 1.0
    out = {"flight": [], "events": [], "link": [], "text": [], "boot": [], "status": [], "com": [], "nav": [],
           "vtol": [], "gps": [], "skipped": 0, "bad": 0}
    vtol = True                                         # command names: until a boot record says format <= 4
    fmt = None                                          # the boot record's format (the 'G' record's layout)
    gen = frames(blob)
    while True:
        try:
            typ, p = next(gen)
        except StopIteration as stop:
            out["skipped"], out["bad"] = stop.value
            break
        if typ == "F":
            out["flight"].append(decode_flight(p, loop_hz, counts_per_us))
        elif typ == "E":
            tick, fr, to, cause, reason = struct.unpack("<IBBBB", p[:8])
            ev = dict(tick=tick, t_s=tick / loop_hz, frm=state_name(fr), to=state_name(to),
                      cause=cause_name(cause, vtol), reason=reason_name(reason))
            if len(p) >= 10:                            # format 5: the phases before / after
                ev.update(phase_from=phase_name(p[8]), phase_to=phase_name(p[9]))
            out["events"].append(ev)
        elif typ == "L":
            tick, seq, cmd, arg, nonce, code, st, rssi, snr = struct.unpack("<IBBHHBBbb", p[:14])
            lk = dict(tick=tick, t_s=tick / loop_hz, seq=seq, cmd=cmd_name(cmd, vtol), arg=arg,
                      nonce=nonce, result=result_name(code), state_after=state_name(st),
                      rssi_dbm=None if rssi == RSSI_NONE else rssi,
                      snr_db=None if rssi == RSSI_NONE else snr / 4.0)
            if len(p) >= 15:                            # format 5: the phase after the command
                lk["phase_after"] = phase_name(p[14])
            out["link"].append(lk)
        elif typ == "V":
            out["vtol"].append(decode_vtol(p, loop_hz, out["vtol"][-1]["tick"] if out["vtol"] else None))
        elif typ == "G" and fmt == FORMAT_VERSION_GNS and len(p) >= GNS_LEN:   # format 8 (v0.8.1): the GNS form
            out["gps"].append(decode_gns(p, loop_hz))
        elif typ == "G" and len(p) >= GPS_LEN:                 # format 7 (v0.8)
            out["gps"].append(decode_gps(p, loop_hz))
        elif typ == "T":
            text = p.decode("ascii", errors="replace")
            out["text"].append(text)
            words = text.split()
            if len(words) >= 3 and words[:2] == ["VTOL", "profile"] and out["boot"] and "profile" not in out["boot"][-1]:
                out["boot"][-1]["profile"] = words[2].rstrip(":")     # main.c log_boot (v0.7.1): the build profile
        elif typ == "S":
            out["status"].append(decode_status(p, loop_hz))
        elif typ == "C":
            out["com"].append(decode_com(p, loop_hz))
        elif typ == "N":
            out["nav"].append(decode_nav(p, loop_hz))
        elif typ == "B":
            # boot payload: format u8, reset cause u8, loop_hz u16, pwm counts per us u8, ASCII info ("... role FCU ...")
            ver, reset, hz, cpu = struct.unpack("<BBHB", p[:5])
            info = p[5:].decode("ascii", errors="replace")
            words = info.split()
            role = words[words.index("role") + 1] if "role" in words[:-1] else None   # format 3: one chip, no role
            mode = "vtol" if ver >= FORMAT_VTOL_FIRST else ("ballistic" if ver >= FORMAT_BALLISTIC else None)
            out["boot"].append(dict(format=ver, reset=[n for b, n in RESET_BITS.items() if reset & b], loop_hz=hz,
                                    counts_per_us=cpu, role=role, mode=mode, info=info,
                                    gps_hold=True if ver >= FORMAT_VERSION_GPS else (False if mode == "vtol" else None)))
            loop_hz = float(hz) or loop_hz
            counts_per_us = float(cpu) or counts_per_us
            vtol = ver >= FORMAT_VTOL_FIRST
            fmt = ver
            if ver not in FORMATS_DECODED:
                out["text"].append(f"WARNING: log format {ver} is not one this decoder reads {FORMATS_DECODED}")
            elif ver < FORMAT_BALLISTIC:
                out["text"].append(f"note: log format {ver} (v0.5 single chip), decoder formats {FORMATS_DECODED}")
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log")
    ap.add_argument("--out", help="flight CSV (default <log>_flight.csv)")
    a = ap.parse_args(argv)
    with open(a.log, "rb") as f:
        blob = f.read()
    d = decode(blob)
    out = a.out or os.path.splitext(a.log)[0] + "_flight.csv"
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FLIGHT_COLS)
        w.writeheader()
        for r in d["flight"]:
            w.writerow(r)
    base = os.path.splitext(out)[0].replace("_flight", "")
    link_csv, com_csv, nav_csv, vtol_csv = base + "_link.csv", base + "_com.csv", base + "_nav.csv", base + "_vtol.csv"
    gps_csv = base + "_gps.csv"
    for rows, path in ((d["link"], link_csv), (d["com"], com_csv), (d["nav"], nav_csv), (d["vtol"], vtol_csv),
                       (d["gps"], gps_csv)):
        if rows:
            # the union of the columns in first-seen order: one file may hold records of two formats (a format-5 'V'
            # record has no format-6 columns; a reflashed vehicle's log continues in the same file)
            fields = list(dict.fromkeys(k for r in rows for k in r))
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=fields)
                w.writeheader()
                for r in rows:
                    w.writerow(r)
    for b in d["boot"]:
        prof = f", profile {b['profile']}" if "profile" in b else ""
        print(f"boot: format {b['format']}, role {b['role'] or '-'}, mode {b['mode'] or '-'}{prof}, reset "
              f"{'+'.join(b['reset']) or '?'}, loop {b['loop_hz']} Hz, {b['info']}")
    for t in d["text"]:
        print(f"text: {t}")
    for e in d["events"]:
        ph = f" [{e['phase_from']} -> {e['phase_to']}]" if "phase_from" in e else ""
        print(f"t={e['t_s']:9.3f}  {e['frm']} -> {e['to']}{ph}: {e['cause']} (reason: {e['reason']})")
    if d["link"]:
        lk = d["link"]
        n_exec = sum(1 for r in lk if r["result"] == "executed")
        rej = [r for r in lk if r["result"] not in ("executed", "heartbeat", "repeat (reply only)")]
        print(f"link: {len(lk)} uplink frames, {n_exec} commands executed, {len(rej)} rejected -> {link_csv}")
        for r in rej:
            print(f"  t={r['t_s']:9.3f}  {r['cmd']} (seq {r['seq']}) {r['result']}")
    if d["status"]:
        st = d["status"]
        print(f"status: {len(st)} records, min free stack {min(r['stack_free'] for r in st)} B, "
              f"I2C errors {st[-1]['twi_errors']}, log drops {st[-1]['uart_dropped']}, "
              f"uplink frames {st[-1]['link_frames']} (bad {st[-1]['link_bad']}, other {st[-1]['link_other']}), "
              f"replies {st[-1]['replies']}, radio errors {st[-1]['radio_errors']}, last RSSI {st[-1]['rssi_dbm']}")
        r = st[-1]
        if r["format"] == 4:
            age = "-" if r["com_age_s"] is None else f"{r['com_age_s']} s"
            print(f"ilink (FCU): IL_CMD {r['il_cmds']}, SNAP {r['il_snaps']}, rejected {r['il_bad']}, "
                  f"UART errors {r['il_uart_errors']}, dropped {r['il_dropped']}; COM status age {age}, radio "
                  f"{'ok' if r['radio_ok'] else 'NOT ok'}{', PARAMS TAG MISMATCH' if r['com_params_mismatch'] else ''}")
    if d["com"]:
        c = d["com"]
        boots = sum(1 for r in c if r["com_boot"])
        print(f"COM: {len(c)} IL_STAT records ({boots} with the boot flag) -> {com_csv}; last: {c[-1]['backend']} radio "
              f"{'ok' if c[-1]['radio_ok'] else 'NOT ok'}, uplink frames {c[-1]['dec_frames']}, forwarded "
              f"{c[-1]['cmd_forwarded']}, replies {c[-1]['replies']}, COM loop max {c[-1]['loop_us_max']} us")
    if d["nav"]:
        n = d["nav"]
        r = n[-1]
        print(f"NAV link: {len(n)} records -> {nav_csv}; last: {r['frames']} valid frames, {r['bad']} bad, {r['timeouts']} "
              f"ticks without a frame (FCU clock), {r['seq_lost']} NAV ticks lost, SPI resyncs {r['resyncs']} / overruns "
              f"{r['spi_overruns']}; NAV loop max {max(x['nav_loop_us_max'] for x in n)} us, reset "
              f"{'+'.join(r['nav_reset']) or '?'}, flags " + " ".join(k for k in NAV_FLAG_BITS.values() if r[k]))
        seen = [name for b, name in NAV_RC_BITS.items() if any(x["nav_reset_cause"] & b for x in n)]
        if seen:   # NAV_RC_*: boot results (IMU / baro init) and a params tag that differs from the FCU's
            print("NAV reports (reset_cause bits 7..5, in any record): " + ", ".join(
                f"{name} ({sum(1 for x in n if x['nav_reset_cause'] & b)} of {len(n)} records)"
                for b, name in NAV_RC_BITS.items() if name in seen))
    if d["vtol"]:
        v = d["vtol"]
        powered = [r for r in v if r["motor_on"]]
        print(f"VTOL: {len(v)} 'V' records -> {vtol_csv}; phases {' '.join(dict.fromkeys(r['phase'] for r in v))}; "
              f"motor time {v[-1]['motor_s']:.1f} s; throttle "
              + (f"{min(r['u'] for r in powered):.3f}..{max(r['u'] for r in powered):.3f}, at its ceiling in "
                 f"{sum(r['sat_high'] for r in powered)} of {len(powered)} powered records" if powered else "-")
              + f"; touchdown confirmed in {sum(r['touchdown'] for r in v)} records")
        v6 = [r for r in v if "sig" in r]
        if v6:   # format 6: which signature confirmed each touchdown, the V10 counter, the NAV's estimator correction
            by = [r["td_by"] for r in v6 if r["touchdown"] and r["td_by"] != "none"]
            pw6 = [r for r in v6 if r["motor_on"]]
            print(f"  touchdown detector v2: confirmed by {' '.join(dict.fromkeys(by)) or '-'}; signatures seen "
                  + (" ".join(n[4:] for n in VC_SIG_BITS.values() if any(r[n] for r in v6)) or "-")
                  + f"; V10 counter max {max(r['low_in_land_s'] for r in v6):.2f} s; NAV estimator correction in "
                  + (f"{sum(r['est_gate'] for r in pw6)} frames of the {len(pw6)} powered records "
                     f"(mean share {sum(r['est_gate_frac'] for r in pw6) / len(pw6):.2f})" if pw6 else "- (never powered)"))
    if d["gps"]:
        g = d["gps"]
        eng = [r for r in g if r["g_engaged"]]
        dists = [r["dist_m"] for r in eng if r["dist_m"] is not None]
        req = sorted({n for r in g for b, n in POS_REQS.items() if r["requests"] & b})
        gns = "slot_tilts" in g[-1]                          # format 8: the GNS form
        print(f"GPS / position hold{' (GNS board)' if gns else ''}: {len(g)} 'G' records -> {gps_csv}; modes "
              f"{' '.join(dict.fromkeys(r['pos_mode'] for r in g))}"
              f"; engaged in {len(eng)}; distance from the hold point max "
              + (f"{max(dists):.1f} m (mean {sum(dists) / len(dists):.2f} m)" if dists else "-")
              + f"; LAND requests {' '.join(req) or '-'}; satellites {min(r['num_sv'] for r in g)}..{max(r['num_sv'] for r in g)}"
              + (f"; TRIM fallback in {sum(r['g_fallback'] for r in g)} records, the FCU's own GPS_LOST in "
                 f"{sum(r['g_own_lost'] for r in g)}; slot sets {g[-1]['slot_sets']} (broken {g[-1]['slot_broken']}), TILT "
                 f"parts {g[-1]['slot_tilts']}; tilt work late in {g[-1]['pos_late_ticks']} ticks" if gns else
                 f"; NAV position resets {g[-1]['nav_resets']}; slot sets {g[-1]['slot_sets']} (broken "
                 f"{g[-1]['slot_broken']}); position work late in {g[-1]['pos_late_ticks']} ticks"))
    if d["flight"]:
        lmax = max(r["loop_us_max"] for r in d["flight"])
        print(f"{len(d['flight'])} flight records -> {out}; max loop time {lmax} us; "
              f"overruns (format 4 of v0.6.3: ticks without a valid navigation frame) {d['flight'][-1]['overruns']}")
    print(f"skipped bytes {d['skipped']}, bad checksums {d['bad']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
