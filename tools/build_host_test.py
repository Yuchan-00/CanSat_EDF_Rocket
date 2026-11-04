#!/usr/bin/env python3
"""Build (and optionally run) the host test of the ATmega128 firmware core.

Compiles ``firmware/atmega128_fc/host_test/test_core.c`` with every ``src/core/*.c`` (and nothing
from ``src/hal`` or ``main.c``) using the Zig toolchain from the venv:

    python -m ziglang cc -std=c99 -O1 -ffp-contract=off -Wall -Wextra ...

``-ffp-contract=off`` keeps the host from fusing a*b+c into one FMA, so every float expression
rounds exactly like avr-gcc's (which has no FMA).  Before compiling, the LoRa protocol byte vectors
of the reference (``sim/out/link_vectors.json``, written by ``sim.link_proto.make_test_vectors``)
are turned into ``host_test/build/link_vectors_gen.h`` so the unit test checks the C decoder against
them byte for byte (frames emitted, every counter, re-encoding; v0.7: TELEM2 frames with their 17 fields, accept
bit 4 = TELEM2, LEN_MAX 34, the ON / OFF / UP / DOWN codes).  The same is done for the FCU <-> COM
ilink (v0.6): ``sim/out/ilink_vectors.json`` (``python -m sim.ilink_proto`` or the pytest
``sim/test_ilink_proto.py``) -> ``host_test/build/ilink_vectors_gen.h`` (payload fields <-> bytes <->
frames, decoder streams - each with its accept mask - with every returned frame, every counter and il_decoder_need /
il_decoder_cmd_need after every byte, SNAP -> TELEM frames; v0.7: SNAP2 payloads with 20 fields, the VTOL COM mask,
SNAP2 -> TELEM2 frames with their lengths), and for the NAV <-> FCU nlink (v0.6.3):
``sim/out/nlink_vectors.json`` (``python -m sim.nlink_proto`` or the pytest ``sim/test_nlink_proto.py``) ->
``host_test/build/nlink_vectors_gen.h`` (NAV fields <-> 72-byte frames, FCU frames, both decoders on every decode
case; v0.7.1: the FCU frame's thr_q byte - the v0.7.1 FCU frames ``fcus_vtol``, ``fcu_thr_q`` of every valid decode and the
``thr_quant`` u -> thr_q table; v0.8: the ``v3`` block - nlink v3 payloads <-> frames, the v3 decoder on every decode case,
the GPS slots of one snapshot and the slot-receiver sequences).  v0.8 also: TELEM3 (TYPE 0x22, 37 B, 22 fields, accept bit
8, LEN_MAX 40) in the link header, SNAP3 (TYPE 5, 62 B, 24 fields, the ``v08`` block) in the ilink header, and the u-blox
UBX vectors ``sim/out/gps_vectors.json`` (``python -m sim.gps_nav``) -> ``host_test/build/gps_vectors_gen.h`` (messages
<-> bytes and fields, the boot configuration frames, parser streams with every message and counter, the epoch assembler,
the iTOW clock, the local frame).  v0.8.1 (the four-chip build): the NAV <-> GNS glink vectors
``sim/out/glink_vectors.json`` (``python -m sim.glink_proto``) -> ``host_test/build/glink_vectors_gen.h`` (STATE /
RESULT payloads <-> bytes <-> frames, the decoder on every decode case and stream with its counters, the NAV's result
ring and rotating-slot parts, the FCU's slot receiver, the FCU4 reply of nlink v4, the nlink v4 NAV4 frames, and one
GNS-node replay - the NAV STATE bytes in, the RESULT frames out - with the ``gp.`` / ``pp.`` overrides of
gen_firmware_params gps_c_fields / pos_c_fields so the C node runs on the reference's float32 parameters).

Invariant: the core is portable C99 - the build fails if any core file includes an AVR header; the
vector headers are pure functions of the JSON files (and name their sha256).  Output:
``firmware/atmega128_fc/host_test/build/test_core(.exe)``.

    .venv/Scripts/python.exe tools/build_host_test.py           # build
    .venv/Scripts/python.exe tools/build_host_test.py --run     # build and run the unit tests
    .venv/Scripts/python.exe tools/build_host_test.py --run --profile demo   # the same with -D MISSION_PROFILE_DEMO
    .venv/Scripts/python.exe tools/build_host_test.py --run --vcfg 1         # fw09: the VTOL tether configuration
    .venv/Scripts/python.exe tools/build_host_test.py --configs --qfix       # fw09: the regular set - every
                                                                             # configuration x profile + the AVR fx_mulhi

fw09 (SPEC_firmware_v09 G1 / WP-X): params_gen.h carries one vehicle-constant set per configuration, selected by
``-D FC_VCFG`` (0 VTOL battery, 1 tether, 2 hybrid, 3 rocket); a host build without it compiles the rocket set.
``--vcfg n`` builds ``test_core[_<profile>]_vcfg<n>`` (``--pwr-src n`` adds ``-D PWR_SRC=n``, as the FCU's power envs:
core/fc_config.c then fills the build's power table and tethered limits as its defaults - the crosscheck's tether /
hybrid cores; the unit tests assume the host's battery default and run without it); ``--configs`` builds and runs the
unit tests of the whole matrix
(field / demo x rocket / battery / tether / hybrid: the unit tests must pass with every configuration's constants);
``--qfix`` runs ``firmware/atmega128_fc/tools/qfix_test`` (the AVR inline-asm ``fx_mulhi`` of core/qfix.h against its
C definition on 200000 operand pairs in simavr, exit 77 = toolchain / simavr missing, reported as NOT RUN).
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import re
import struct
import subprocess
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
FW = os.path.join(ROOT, "firmware", "atmega128_fc")
CORE = os.path.join(FW, "src", "core")
TEST = os.path.join(FW, "host_test", "test_core.c")
BUILD = os.path.join(FW, "host_test", "build")
EXE = os.path.join(BUILD, "test_core.exe" if os.name == "nt" else "test_core")
LINK_VECTORS = os.path.join(ROOT, "sim", "out", "link_vectors.json")
VECTOR_HEADER = os.path.join(BUILD, "link_vectors_gen.h")
ILINK_VECTORS = os.path.join(ROOT, "sim", "out", "ilink_vectors.json")
ILINK_VECTOR_HEADER = os.path.join(BUILD, "ilink_vectors_gen.h")
NLINK_VECTORS = os.path.join(ROOT, "sim", "out", "nlink_vectors.json")
NLINK_VECTOR_HEADER = os.path.join(BUILD, "nlink_vectors_gen.h")
GPS_VECTORS = os.path.join(ROOT, "sim", "out", "gps_vectors.json")
GPS_VECTOR_HEADER = os.path.join(BUILD, "gps_vectors_gen.h")
GLINK_VECTORS = os.path.join(ROOT, "sim", "out", "glink_vectors.json")
GLINK_VECTOR_HEADER = os.path.join(BUILD, "glink_vectors_gen.h")
CFLAGS = ["-std=c99", "-O1", "-ffp-contract=off", "-Wall", "-Wextra", "-Wno-unused-parameter", "-Werror=implicit-function-declaration"]
AVR_INCLUDE = re.compile(r'^\s*#\s*include\s*[<"](avr/|util/)', re.M)
STAT_ORDER = ("frames", "bad_len", "bad_crc", "wrong_vehicle", "wrong_type", "bad_payload_len", "skipped_bytes")
TELEM_ORDER = ("state", "flags", "reason", "ack_seq", "nonce", "t_ms", "h_dm", "vz_cms", "roll_cdeg", "pitch_cdeg",
               "yaw_cdeg", "tilt_cdeg", "throttle_milli", "vcell_mv", "loop_max_us")
TELEM2_ORDER = TELEM_ORDER + ("phase", "h_tgt_dm")     # v0.7 TELEM v2 (TYPE 0x21): 17 fields
TELEM3_ORDER = TELEM2_ORDER + ("gps_flags", "num_sv", "hacc_dm", "pos_mode", "dist_dm")   # v0.8 TELEM v3: 22 fields
PWR_ORDER = ("v_aux_q", "pwr_flags")                    # v0.9 RP1 TELEM v4 / v5 power tail (f[22], f[23])
LV_MAX_PAYLOAD = 39                                     # link_proto.MAX_PAYLOAD_LEN (v0.9 RP1: TELEM5), checked against the JSON
LV_NF = len(TELEM3_ORDER) + len(PWR_ORDER)              # 24 (v0.8: 22)
ILV_NF = 27                                             # SNAP3: 27 values (q expanded; v0.9: + pw_vaux, pw_flags, spare3)


def core_sources() -> list[str]:
    srcs = sorted(glob.glob(os.path.join(CORE, "*.c")))
    for path in srcs + sorted(glob.glob(os.path.join(CORE, "*.h"))):
        with open(path, "r", encoding="utf-8") as f:
            if AVR_INCLUDE.search(f.read()):
                raise SystemExit(f"build_host_test: {os.path.relpath(path, ROOT)} includes an AVR header - the core must stay portable")
    return srcs


def _c_bytes(b: bytes) -> str:
    return ", ".join(f"0x{x:02X}" for x in b) if b else "0"


def vector_header(path: str = LINK_VECTORS) -> str:
    """C header with the reference's link vectors (sim/out/link_vectors.json)."""
    if not os.path.exists(path):
        raise SystemExit(f"build_host_test: {os.path.relpath(path, ROOT)} missing - run the simulation's "
                         "test_link_vectors_independent_check_and_write (sim/test_sim.py) first")
    raw = open(path, "rb").read()
    lv = json.loads(raw.decode("utf-8"))
    sys.path.insert(0, ROOT)
    from sim import link_proto as lp  # the re-encode flag only; the expectations come from the JSON
    c = lv.get("constants", {})
    if lp.MAX_PAYLOAD_LEN != LV_MAX_PAYLOAD or int(c.get("LEN_MAX", lp.LEN_MAX)) != lp.LEN_MAX:
        raise SystemExit(f"build_host_test: link_proto MAX_PAYLOAD_LEN {lp.MAX_PAYLOAD_LEN} / LEN_MAX {c.get('LEN_MAX')} - "
                         f"lv_frame_t is sized for {LV_MAX_PAYLOAD}; update this generator")
    accept_bit = {lp.TYPE_CMD: 1, lp.TYPE_TELEM: 2, lp.TYPE_TELEM2: 4, lp.TYPE_TELEM3: 8,     # link_proto.h LP_ACCEPT_*
                  lp.TYPE_TELEM4: 16, lp.TYPE_TELEM5: 32}                                    # v0.9 RP1
    out = ["/* link_vectors_gen.h - GENERATED by tools/build_host_test.py from sim/out/link_vectors.json",
           f" * (sha256 {hashlib.sha256(raw).hexdigest()}, seed {lv['seed']}). DO NOT EDIT. */",
           "#ifndef LINK_VECTORS_GEN_H", "#define LINK_VECTORS_GEN_H", "#include <stdint.h>", "",
           "/* kind: 0 other, 1 CMD (f = cmd, arg, nonce), 2 TELEM v1 (15 fields), 3 TELEM v2 (17 fields: + phase, h_tgt_dm),",
           " * 4 TELEM v3 (22 fields: + gps_flags, num_sv, hacc_dm, pos_mode, dist_dm), v0.9 RP1: 5 TELEM v4 (the v2 fields,",
           " * zeros for the GPS ones, f[22] v_aux_q, f[23] pwr_flags), 6 TELEM v5 (the v3 fields, f[22], f[23]) */",
           f"typedef struct {{ uint8_t veh, type, seq, len; uint8_t payload[{LV_MAX_PAYLOAD}]; int8_t kind; long long f[{LV_NF}]; }} "
           "lv_frame_t;",
           "typedef struct { const char *name; const uint8_t *data; uint16_t n; int16_t vid; uint8_t accept;",
           "                 uint8_t n_frames; const lv_frame_t *frames; long stats[7]; uint8_t reencode; } lv_vector_t;",
           "typedef struct { const char *field; double physical; long value; } lv_scaling_t;", "",
           f"#define LV_VEHICLE_ID {int(lv['vehicle_id'])}",
           f"#define LV_CRC_CHECK 0x{int(lv['crc16_ccitt_false_check']['crc']):04X}",
           f"#define LV_LEN_MAX {int(c.get('LEN_MAX', lp.LEN_MAX))}",
           f"#define LV_TYPE_TELEM2 0x{int(c.get('TYPE_TELEM2', lp.TYPE_TELEM2)):02X}",
           f"#define LV_TELEM2_FRAME_BYTES {int(c.get('TELEM2_FRAME_BYTES', lp.TELEM2_FRAME_BYTES))}",
           f"#define LV_TYPE_TELEM3 0x{int(c.get('TYPE_TELEM3', lp.TYPE_TELEM3)):02X}",
           f"#define LV_TELEM3_FRAME_BYTES {int(c.get('TELEM3_FRAME_BYTES', lp.TELEM3_FRAME_BYTES))}",
           f"#define LV_DIST_NONE {int(c.get('DIST_NONE', lp.DIST_NONE))}",
           f"#define LV_NF {LV_NF}",
           "#define LV_CMD_ON_OFF_UP_DOWN {" + ", ".join(str(int(c.get(k, d))) for k, d in
                                                      (("CMD_ON", 8), ("CMD_OFF", 9), ("CMD_UP", 10), ("CMD_DOWN", 11))) + "}",
           ""]
    vecs = list(lv["frames"]) + list(lv["streams"])
    rows = []
    for i, v in enumerate(vecs):
        data = bytes.fromhex(v["hex"])
        acc = 0
        for t in v["decoder"]["accept_types"]:
            acc |= accept_bit.get(t, 0)
        vid = v["decoder"]["vehicle_id"]
        out.append(f"static const uint8_t lv_d{i}[] = {{{_c_bytes(data)}}};")
        fr_rows = []
        for fr in v["expected_frames"]:
            pl = bytes.fromhex(fr["payload_hex"])
            if "telem" in fr and "pwr_flags" in fr["telem"] and "gps_flags" in fr["telem"]:
                kind, f = 6, [int(fr["telem"][k]) for k in TELEM3_ORDER + PWR_ORDER]
            elif "telem" in fr and "pwr_flags" in fr["telem"]:
                kind, f = 5, [int(fr["telem"][k]) for k in TELEM2_ORDER] + [0] * 5 + [int(fr["telem"][k]) for k in PWR_ORDER]
            elif "telem" in fr and "gps_flags" in fr["telem"]:
                kind, f = 4, [int(fr["telem"][k]) for k in TELEM3_ORDER] + [0] * 2
            elif "telem" in fr and "phase" in fr["telem"]:
                kind, f = 3, [int(fr["telem"][k]) for k in TELEM2_ORDER] + [0] * 7
            elif "telem" in fr:
                kind, f = 2, [int(fr["telem"][k]) for k in TELEM_ORDER] + [0] * 9
            elif "cmd" in fr:
                kind, f = 1, [int(fr["cmd"][k]) for k in ("cmd", "arg", "nonce")] + [0] * (LV_NF - 3)
            else:
                kind, f = 0, [0] * LV_NF
            fr_rows.append(f"{{{fr['veh']}, {fr['type']}, {fr['seq']}, {len(pl)}, {{{_c_bytes(pl)}}}, {kind}, "
                           f"{{{', '.join(str(x) + 'LL' for x in f)}}}}}")
        if fr_rows:
            out.append(f"static const lv_frame_t lv_f{i}[] = {{{', '.join(fr_rows)}}};")
        reenc = 0
        if len(v["expected_frames"]) == 1:
            e = v["expected_frames"][0]
            reenc = int(lp.encode_frame(e["veh"], e["type"], e["seq"], bytes.fromhex(e["payload_hex"])) == data)
        stats = ", ".join(f"{int(v['expected_stats'][k])}L" for k in STAT_ORDER)
        rows.append(f"    {{\"{v['name']}\", lv_d{i}, {len(data)}, {-1 if vid is None else int(vid)}, {acc}, "
                    f"{len(v['expected_frames'])}, {'lv_f' + str(i) if fr_rows else '0'}, {{{stats}}}, {reenc}}},")
    out.append("static const lv_vector_t LV_VECTORS[] = {")
    out += rows
    out.append("};")
    out.append(f"#define LV_N_VECTORS {len(vecs)}")
    out.append("static const lv_scaling_t LV_SCALING[] = {")
    for s in lv["scaling"]:
        out.append(f"    {{\"{s['field']}\", {float(s['physical'])!r}, {int(s['int'])}L}},")
    out.append("};")
    out.append(f"#define LV_N_SCALING {len(lv['scaling'])}")
    out += ["", "#endif", ""]
    return "\n".join(out)


def _flat_fields(fields: dict) -> list[int]:
    """Payload fields in struct order (q_bits expanded); floats are their uint32 bit patterns."""
    vals: list[int] = []
    for v in fields.values():
        if isinstance(v, list):
            vals.extend(int(x) for x in v)
        else:
            vals.append(int(v))
    return vals


def ilink_vector_header(path: str = ILINK_VECTORS) -> str:
    """C header with the ilink reference vectors (sim/out/ilink_vectors.json, sim/ilink_proto.py)."""
    if not os.path.exists(path):
        raise SystemExit(f"build_host_test: {os.path.relpath(path, ROOT)} missing - run "
                         "`.venv/Scripts/python.exe -m sim.ilink_proto` (or pytest sim/test_ilink_proto.py) first")
    raw = open(path, "rb").read()
    iv = json.loads(raw.decode("utf-8"))
    out = ["/* ilink_vectors_gen.h - GENERATED by tools/build_host_test.py from sim/out/ilink_vectors.json",
           f" * (sha256 {hashlib.sha256(raw).hexdigest()}, seed {iv['seed']}). DO NOT EDIT. */",
           "#ifndef ILINK_VECTORS_GEN_H", "#define ILINK_VECTORS_GEN_H", "#include <stdint.h>", "",
           "typedef struct { const char *name; uint8_t type, n; const uint8_t *payload; uint8_t frame_n; const uint8_t *frame;",
           f"                 uint8_t n_fields; long long f[{ILV_NF}]; uint8_t py_roundtrip; }} ilv_payload_t;",
           "typedef struct { uint8_t type, len; const uint8_t *payload; } ilv_frame_t;",
           "typedef struct { const char *name; const uint8_t *data; uint16_t n; uint8_t max_out; uint8_t accept; uint8_t n_frames;",
           "                 const ilv_frame_t *frames; unsigned stats[6]; uint8_t pending;",
           "                 const uint8_t *need, *cmd_need; } ilv_stream_t;   /* need / cmd_need after each byte */",
           "typedef struct { const char *name; uint8_t snap_n; const uint8_t *snap; uint8_t telem_n; const uint8_t *telem; } "
           "ilv_telem_t;   /* SNAP 49 -> TELEM 36, SNAP2 57 -> TELEM2 39 / TELEM4 41, SNAP3 65 -> TELEM3 45 / TELEM5 47 */", "",
           f"#define ILV_VEHICLE_ID {int(iv['vehicle_id'])}",
           f"#define ILV_LOOP_HZ {int(iv['loop_hz'])}",
           f"#define ILV_CRC_CHECK 0x{int(iv['crc16_check']['crc']):04X}",
           f"#define ILV_DEC_MAX_OUT {int(iv['constants']['DEC_MAX_OUT'])}",
           f"#define ILV_ACCEPT_FCU {int(iv['constants']['ACCEPT_FCU'])}",
           f"#define ILV_ACCEPT_COM {int(iv['constants']['ACCEPT_COM'])}",
           f"#define ILV_ACCEPT_ALL {int(iv['constants']['ACCEPT_ALL'])}",
           f"#define ILV_ACCEPT_SNAP2 {int(iv['constants']['ACCEPT_SNAP2'])}",
           f"#define ILV_ACCEPT_COM_VTOL {int(iv['constants']['ACCEPT_COM_VTOL'])}",
           f"#define ILV_TYPE_SNAP2 {int(iv['constants']['TYPE_SNAP2'])}",
           f"#define ILV_SNAP2_LEN {int(iv['constants']['SNAP2_LEN'])}",
           f"#define ILV_FRAME_MAX_BYTES {int(iv['constants']['FRAME_MAX_BYTES'])}",
           f"#define ILV_NEED_NEVER {int(iv['constants']['NEED_NEVER'])}",
           f"#define ILV_TYPE_SNAP3 {int(iv['v08']['constants']['TYPE_SNAP3'])}",
           f"#define ILV_SNAP3_LEN {int(iv['v08']['constants']['SNAP3_LEN'])}",
           f"#define ILV_ACCEPT_SNAP3 {int(iv['v08']['constants']['ACCEPT_SNAP3'])}",
           f"#define ILV_ACCEPT_ALL8 {int(iv['v08']['constants']['ACCEPT_ALL8'])}",
           f"#define ILV_ACCEPT_COM_GPS {int(iv['v08']['constants']['ACCEPT_COM_GPS'])}",
           f"#define ILV_NF {ILV_NF}",
           "/* stats[] order: " + ", ".join(iv["stat_keys"]) + " */", ""]
    rows = []
    v08 = iv.get("v08") or {}
    v09 = iv.get("v09") or {}                                   # v0.9 RP1: SNAP2 / SNAP3 with the power tail
    for i, v in enumerate(list(iv["payloads"]) + list(v08.get("payloads", [])) + list(v09.get("payloads", []))):
        pl, fr = bytes.fromhex(v["payload_hex"]), bytes.fromhex(v["frame_hex"])
        f = _flat_fields(v["fields"])
        if len(f) > ILV_NF:
            raise SystemExit(f"build_host_test: payload vector {v['name']} has {len(f)} fields (> {ILV_NF})")
        out.append(f"static const uint8_t ilv_pl{i}[] = {{{_c_bytes(pl)}}};")
        out.append(f"static const uint8_t ilv_pf{i}[] = {{{_c_bytes(fr)}}};")
        rows.append(f"    {{\"{v['name']}\", {int(v['type'])}, {len(pl)}, ilv_pl{i}, {len(fr)}, ilv_pf{i}, {len(f)}, "
                    f"{{{', '.join(str(x) + 'LL' for x in f) or '0'}}}, {int(bool(v.get('py_roundtrip', True)))}}},")
    out.append("static const ilv_payload_t ILV_PAYLOADS[] = {")
    out += rows
    out.append("};")
    out.append(f"#define ILV_N_PAYLOADS {len(rows)}")
    rows = []
    for i, v in enumerate(list(iv["streams"]) + list(v08.get("streams", [])) + list(v09.get("streams", []))):
        data = bytes.fromhex(v["hex"])
        out.append(f"static const uint8_t ilv_d{i}[] = {{{_c_bytes(data)}}};")
        fr_rows = []
        for j, e in enumerate(v["expected_frames"]):
            pl = bytes.fromhex(e["payload_hex"])
            out.append(f"static const uint8_t ilv_d{i}_{j}[] = {{{_c_bytes(pl)}}};")
            fr_rows.append(f"{{{int(e['type'])}, {int(e['len'])}, ilv_d{i}_{j}}}")
        if fr_rows:
            out.append(f"static const ilv_frame_t ilv_f{i}[] = {{{', '.join(fr_rows)}}};")
        need, cneed = bytes.fromhex(v["need_hex"]), bytes.fromhex(v["cmd_need_hex"])
        if len(need) != len(data) or len(cneed) != len(data):
            raise SystemExit(f"build_host_test: stream {v['name']}: need / cmd_need not one per byte")
        out.append(f"static const uint8_t ilv_n{i}[] = {{{_c_bytes(need)}}};")
        out.append(f"static const uint8_t ilv_c{i}[] = {{{_c_bytes(cneed)}}};")
        stats = ", ".join(f"{int(v['expected_stats'][k])}u" for k in iv["stat_keys"])
        rows.append(f"    {{\"{v['name']}\", ilv_d{i}, {len(data)}, {int(v['max_out'])}, {int(v['accept'])}, {len(v['expected_frames'])}, "
                    f"{'ilv_f' + str(i) if fr_rows else '0'}, {{{stats}}}, {int(v['pending'])}, ilv_n{i}, ilv_c{i}}},")
    out.append("static const ilv_stream_t ILV_STREAMS[] = {")
    out += rows
    out.append("};")
    out.append(f"#define ILV_N_STREAMS {len(rows)}")
    rows = []
    for i, v in enumerate(list(iv["telem"]) + list(v08.get("telem", [])) + list(v09.get("telem", []))):
        sb, tb = bytes.fromhex(v["snap_hex"]), bytes.fromhex(v["telem_frame_hex"])
        out.append(f"static const uint8_t ilv_ts{i}[] = {{{_c_bytes(sb)}}};")
        out.append(f"static const uint8_t ilv_tt{i}[] = {{{_c_bytes(tb)}}};")
        rows.append(f"    {{\"{v['name']}\", {len(sb)}, ilv_ts{i}, {len(tb)}, ilv_tt{i}}},")
    out.append("static const ilv_telem_t ILV_TELEM[] = {")
    out += rows
    out.append("};")
    out.append(f"#define ILV_N_TELEM {len(rows)}")
    out += ["", "#endif", ""]
    return "\n".join(out)


def nlink_vector_header(path: str = NLINK_VECTORS) -> str:
    """C header with the nlink reference vectors (sim/out/nlink_vectors.json, sim/nlink_proto.py)."""
    if not os.path.exists(path):
        raise SystemExit(f"build_host_test: {os.path.relpath(path, ROOT)} missing - run "
                         "`.venv/Scripts/python.exe -m sim.nlink_proto` (or pytest sim/test_nlink_proto.py) first")
    raw = open(path, "rb").read()
    nv = json.loads(raw.decode("utf-8"))
    c = nv["constants"]
    out = ["/* nlink_vectors_gen.h - GENERATED by tools/build_host_test.py from sim/out/nlink_vectors.json",
           f" * (sha256 {hashlib.sha256(raw).hexdigest()}, seed {nv['seed']}). DO NOT EDIT. */",
           "#ifndef NLINK_VECTORS_GEN_H", "#define NLINK_VECTORS_GEN_H", "#include <stdint.h>", "",
           "/* NAV fields: 14 float bit patterns in wire order (q[4], rate[3], accel[3], h_m, vz_mps, h_baro_rel_m, tilt_deg),",
           " * then imu_fail, baro_fails, twi_errors, loop_us_max, flags, reset_cause */",
           "typedef struct { const char *name; uint8_t seq; const uint8_t *payload; const uint8_t *frame; uint32_t fb[14];",
           "                 long iv[6]; uint8_t py_roundtrip; } nlv_nav_t;",
           "/* FCU fields: seq, state, fflags, params_tag, thr_q (v0.7.1 byte 7; 0 in the v0.6.3 vectors) */",
           "typedef struct { const char *name; uint8_t f[5]; const uint8_t *frame; } nlv_fcu_t;",
           "typedef struct { const char *name; const uint8_t *buf; uint8_t nav_valid, seq; uint32_t fb[14]; long iv[6];",
           "                 uint8_t fcu_valid; uint8_t fcu[5]; } nlv_dec_t;",
           "typedef struct { double u; uint8_t thr_q; } nlv_thr_t;   /* thr_quant: u -> fcu_throttle_q(u) */", "",
           f"#define NLV_SYNC0 0x{int(c['SYNC'][0]):02X}", f"#define NLV_SYNC1 0x{int(c['SYNC'][1]):02X}",
           f"#define NLV_TYPE_NAV {int(c['TYPE_NAV'])}", f"#define NLV_TYPE_FCU {int(c['TYPE_FCU'])}",
           f"#define NLV_NAV_LEN {int(c['NAV_LEN'])}", f"#define NLV_XFER_BYTES {int(c['XFER_BYTES'])}",
           f"#define NLV_FCU_FRAME_BYTES {int(c['FCU_FRAME_BYTES'])}",
           f"#define NLV_FCU_THR_SCALE {int(c['FCU_THR_SCALE'])}",
           f"#define NLV_CRC_CHECK 0x{int(nv['crc16_check']['crc']):04X}",
           "#define NLV_FLAGS {" + ", ".join(str(int(c["flags"][k])) for k in ("IMU_OK", "BARO_OK", "ALIGNED", "GYRO_SAT",
                                                                                "EST_VALID", "BARO_NEW", "BOOT")) + "}",
           f"#define NLV_FF_BENCH {int(c['fflags']['BENCH'])}", f"#define NLV_FF_RECOVERY {int(c['fflags']['RECOVERY'])}", ""]
    ikeys = ("imu_fail", "baro_fails", "twi_errors", "loop_us_max", "flags", "reset_cause")

    def fb_iv(fields: dict) -> str:
        return ("{" + ", ".join(f"0x{int(b):08X}u" for b in fields["float_bits"]) + "}, {"
                + ", ".join(f"{int(fields['ints'][k])}L" for k in ikeys) + "}")

    rows = []
    for i, v in enumerate(nv["navs"]):
        out.append(f"static const uint8_t nlv_np{i}[] = {{{_c_bytes(bytes.fromhex(v['payload_hex']))}}};")
        out.append(f"static const uint8_t nlv_nf{i}[] = {{{_c_bytes(bytes.fromhex(v['frame_hex']))}}};")
        rows.append(f"    {{\"{v['name']}\", {int(v['seq'])}, nlv_np{i}, nlv_nf{i}, {fb_iv(v['fields'])}, "
                    f"{int(bool(v['py_roundtrip']))}}},")
    out += ["static const nlv_nav_t NLV_NAVS[] = {"] + rows + ["};", f"#define NLV_N_NAVS {len(nv['navs'])}"]
    rows = []
    fcus = [dict(v, thr_q=0) for v in nv["fcus"]] + list(nv.get("fcus_vtol", []))     # v0.6.3 frames, then the v0.7.1 ones
    for i, v in enumerate(fcus):
        out.append(f"static const uint8_t nlv_ff{i}[] = {{{_c_bytes(bytes.fromhex(v['frame_hex']))}}};")
        rows.append(f"    {{\"{v['name']}\", {{{', '.join(str(int(x)) for x in list(v['fields']) + [v['thr_q']])}}}, "
                    f"nlv_ff{i}}},")
    out += ["static const nlv_fcu_t NLV_FCUS[] = {"] + rows + ["};", f"#define NLV_N_FCUS {len(fcus)}",
                                                            f"#define NLV_N_FCUS_V063 {len(nv['fcus'])}"]
    rows = [f"    {{{float(q['u'])!r}, {int(q['thr_q'])}}}," for q in nv.get("thr_quant", [])]
    out += ["static const nlv_thr_t NLV_THR[] = {"] + (rows or ["    {0.0, 0},"]) + ["};",
                                                                                  f"#define NLV_N_THR {len(rows)}"]
    rows = []
    zero = {"float_bits": [0] * 14, "ints": {k: 0 for k in ikeys}}
    for i, v in enumerate(nv["decodes"]):
        buf = bytes.fromhex(v["hex"])
        if len(buf) != int(c["XFER_BYTES"]):
            raise SystemExit(f"build_host_test: nlink decode case {v['name']} is {len(buf)} bytes")
        out.append(f"static const uint8_t nlv_db{i}[] = {{{_c_bytes(buf)}}};")
        nav = v.get("nav") or {**zero, "seq": 0}
        fcu = list(v.get("fcu") or [0, 0, 0, 0]) + [int(v.get("fcu_thr_q", 0))]
        rows.append(f"    {{\"{v['name']}\", nlv_db{i}, {int(v['nav_valid'])}, {int(nav['seq'])}, {fb_iv(nav)}, "
                    f"{int(v['fcu_valid'])}, {{{', '.join(str(int(x)) for x in fcu)}}}}},")
    out += ["static const nlv_dec_t NLV_DECODES[] = {"] + rows + ["};", f"#define NLV_N_DECODES {len(nv['decodes'])}"]
    out += nlink_v3_lines(nv.get("v3"))
    out += nlink_v4_lines(nv.get("v4"))
    out += ["", "#endif", ""]
    return "\n".join(out)


NAV3_IKEYS = ("accel_x_q", "accel_y_q", "h_baro_rel_cm", "imu_fail", "baro_fails", "twi_errors", "loop_us_max", "flags",
              "reset_cause")


def nlink_v3_lines(v3: dict | None) -> list[str]:
    """v0.8 nlink v3 block of nlink_vectors_gen.h (sim/nlink_proto.py make_v3_vectors)."""
    if not v3:
        raise SystemExit("build_host_test: nlink_vectors.json has no v3 block - run `python -m sim.nlink_proto`")
    c = v3["constants"]
    out = ["", "/* ---- v0.8 nlink v3: 12 float bit patterns (q4 rate3 accel_z h vz cos_tilt tilt), the 9 integers, the slot ---- */",
           "typedef struct { const char *name; uint8_t seq; const uint8_t *payload; const uint8_t *frame; uint32_t fb[12];",
           "                 long iv[9]; uint8_t slot[5]; } nlv3_nav_t;",
           "typedef struct { const char *name; const uint8_t *buf; uint8_t nav3_valid, nav_valid, fcu_valid, seq; uint32_t fb[12];",
           "                 long iv[9]; uint8_t slot[5]; } nlv3_dec_t;",
           "typedef struct { uint8_t valid; uint8_t flags, num_sv, hacc_dm, reset_count; long off[2], p[2], v[2], cs[2]; "
           "uint8_t cycle; } nlv3_set_t;",
           "typedef struct { const char *name; uint8_t n; const uint8_t (*slots)[6]; const nlv3_set_t *out; uint8_t gate_ok_after;",
           "                 unsigned sets, broken; } nlv3_rx_t;   /* slots[i][0] = 0: no valid frame that tick */",
           f"#define NLV3_TYPE_NAV3 {int(c['TYPE_NAV3'])}", f"#define NLV3_NAV3_LEN {int(c['NAV3_LEN'])}",
           f"#define NLV3_SLOT_LEN {int(c['SLOT_LEN'])}", f"#define NLV3_STALE_TICKS {int(c['STALE_TICKS'])}"]
    sn = v3["snapshot"]
    out.append("#define NLV3_SNAP {" + ", ".join(str(int(sn[k])) for k in (
        "flags", "num_sv", "hacc_dm", "reset_count", "off_x_cm", "off_y_cm", "px_cm", "py_cm", "vx_cms", "vy_cms", "cos_q15",
        "sin_q15")) + "}")

    def fb_iv(f: dict) -> str:
        return ("{" + ", ".join(f"0x{int(b):08X}u" for b in f["float_bits"]) + "}, {" + ", ".join(
            f"{int(f[k])}L" for k in NAV3_IKEYS) + "}, {" + _c_bytes(bytes.fromhex(f["slot_hex"])) + "}")

    rows = []
    for i, v in enumerate(v3["navs3"]):
        out.append(f"static const uint8_t nlv3_np{i}[] = {{{_c_bytes(bytes.fromhex(v['payload_hex']))}}};")
        out.append(f"static const uint8_t nlv3_nf{i}[] = {{{_c_bytes(bytes.fromhex(v['frame_hex']))}}};")
        rows.append(f"    {{\"{v['name']}\", {int(v['fields']['seq'])}, nlv3_np{i}, nlv3_nf{i}, {fb_iv(v['fields'])}}},")
    out += ["static const nlv3_nav_t NLV3_NAVS[] = {"] + rows + ["};", f"#define NLV3_N_NAVS {len(rows)}"]
    rows = []
    zero = {"seq": 0, "float_bits": [0] * 12, **{k: 0 for k in NAV3_IKEYS}, "slot_hex": "0000000000"}
    for i, v in enumerate(v3["decodes"]):
        out.append(f"static const uint8_t nlv3_db{i}[] = {{{_c_bytes(bytes.fromhex(v['hex']))}}};")
        n3 = v.get("nav3") or zero
        rows.append(f"    {{\"{v['name']}\", nlv3_db{i}, {int(v['nav3_valid'])}, {int(v['nav_valid'])}, {int(v['fcu_valid'])}, "
                    f"{int(n3['seq'])}, {fb_iv(n3)}}},")
    out += ["static const nlv3_dec_t NLV3_DECODES[] = {"] + rows + ["};", f"#define NLV3_N_DECODES {len(rows)}"]
    out.append("static const uint8_t NLV3_SLOTS[5][5] = {" + ", ".join(
        "{" + _c_bytes(bytes.fromhex(s["hex"])) + "}" for s in sorted(v3["slots"], key=lambda s: s["j"])) + "};")
    out.append(f"#define NLV3_SLOTS_CYCLE {int(v3['slots'][0]['cycle'])}")
    rows = []
    for i, r in enumerate(v3["rx"]):
        sl = ["{" + ("0, 0, 0, 0, 0, 0" if s is None else "1, " + _c_bytes(bytes.fromhex(s))) + "}" for s in r["slots"]]
        out.append(f"static const uint8_t nlv3_rs{i}[][6] = {{{', '.join(sl)}}};")
        ss = []
        for g in r["out"]:
            if g is None:
                ss.append("{0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, {0, 0}, 0}")
                continue
            cm = lambda x: str(int(round(float(x) * 100.0)))   # noqa: E731 (the reference's cm / 100: exact back)
            q = lambda x: str(int(round(float(x) * 32767.0)))  # noqa: E731
            ss.append(f"{{1, {g['flags']}, {g['num_sv']}, {g['hacc_dm']}, {g['reset_count']}, {{{cm(g['off'][0])}, "
                      f"{cm(g['off'][1])}}}, {{{cm(g['p'][0])}, {cm(g['p'][1])}}}, {{{cm(g['v'][0])}, {cm(g['v'][1])}}}, "
                      f"{{{q(g['cos_psi'])}, {q(g['sin_psi'])}}}, {g['cycle']}}}")
        out.append(f"static const nlv3_set_t nlv3_ro{i}[] = {{{', '.join(ss)}}};")
        rows.append(f"    {{\"{r['name']}\", {len(r['slots'])}, nlv3_rs{i}, nlv3_ro{i}, {int(r['gate_ok_after'])}, "
                    f"{int(r['sets'])}u, {int(r['broken'])}u}},")
    out += ["static const nlv3_rx_t NLV3_RX[] = {"] + rows + ["};", f"#define NLV3_N_RX {len(rows)}"]
    return out


POSLLH_KEYS = ("itow_ms", "lon_e7", "lat_e7", "height_mm", "hmsl_mm", "hacc_mm", "vacc_mm")
VELNED_KEYS = ("itow_ms", "vel_n_cms", "vel_e_cms", "vel_d_cms", "speed_cms", "gspeed_cms", "heading_e5", "sacc_cms",
               "cacc_e5")
SOL_KEYS = ("itow_ms", "ftow_ns", "week", "gps_fix", "flags", "ecef_x_cm", "ecef_y_cm", "ecef_z_cm", "pacc_cm", "ecef_vx_cms",
            "ecef_vy_cms", "ecef_vz_cms", "sacc_cms", "pdop_e2", "reserved1", "num_sv", "reserved2")
EPOCH_KEYS = ("itow_ms", "lat_e7", "lon_e7", "hmsl_mm", "hacc_mm", "vacc_mm", "vel_n_cms", "vel_e_cms", "vel_d_cms",
              "sacc_cms", "gps_fix", "fix_ok", "num_sv", "pacc_cm", "tick")
MSG_KIND = {"NavPosllh": 1, "NavVelned": 2, "NavSol": 3}     # gps_ubx.h UBX_K_*


def _gps_msg_kind(m: dict) -> tuple[int, list[int]]:
    """(UBX_K_* of a decoded vector message, its fields in C order; ACK: [cls, id]; MON-VER: [module])."""
    if "fields" in m:
        f = m["fields"]
        if m["id"] == 0x02:
            return 1, [int(f[k]) for k in POSLLH_KEYS]
        if m["id"] == 0x12:
            return 2, [int(f[k]) for k in VELNED_KEYS]
        return 3, [int(f[k]) for k in SOL_KEYS]
    if "ack" in m:
        return (4 if m["ack"][0] == "ACK" else 5), [int(m["ack"][1]), int(m["ack"][2])]
    if "mon_ver" in m:
        return 6, [int(m["mon_ver"]["module"])]
    return 0, []


def gps_vector_header(path: str = GPS_VECTORS) -> str:
    """C header with the u-blox UBX vectors (sim/out/gps_vectors.json, sim/gps_nav.py make_test_vectors)."""
    if not os.path.exists(path):
        raise SystemExit(f"build_host_test: {os.path.relpath(path, ROOT)} missing - run `.venv/Scripts/python.exe -m sim.gps_nav`")
    raw = open(path, "rb").read()
    gv = json.loads(raw.decode("utf-8"))
    c = gv["constants"]
    out = ["/* gps_vectors_gen.h - GENERATED by tools/build_host_test.py from sim/out/gps_vectors.json",
           f" * (sha256 {hashlib.sha256(raw).hexdigest()}). DO NOT EDIT. */",
           "#ifndef GPS_VECTORS_GEN_H", "#define GPS_VECTORS_GEN_H", "#include <stdint.h>", "",
           "/* kind = UBX_K_* (0 none, 1 POSLLH, 2 VELNED, 3 SOL, 4 ACK, 5 NAK, 6 MON-VER); f = the fields in C struct order",
           " * (ACK / NAK: cls, id; MON-VER: module) */",
           "typedef struct { uint8_t cls, id; uint16_t length; uint8_t truncated, kind, n_f; const uint8_t *payload; uint8_t n_pl;",
           "                 long long f[17]; } gpsv_msg_t;",
           "typedef struct { const char *name; const uint8_t *frame; uint16_t n; gpsv_msg_t m; } gpsv_frame_t;",
           "typedef struct { const char *name; const uint8_t *data; uint16_t n; uint8_t n_msgs; const gpsv_msg_t *msgs;",
           "                 unsigned stats[5]; uint8_t state_after; } gpsv_stream_t;   /* stats: frames, bad_ck, bad_len, truncated, skipped */",
           "typedef struct { uint32_t tick; uint8_t kind; uint32_t itow; } gpsv_asm_in_t;",
           "typedef struct { long long f[15]; } gpsv_epoch_t;   /* EPOCH_KEYS order */",
           "typedef struct { const char *name; uint8_t n_in; const gpsv_asm_in_t *in; uint8_t n_ep; const gpsv_epoch_t *ep;",
           "                 unsigned stats[3]; } gpsv_asm_t;   /* epochs, incomplete, duplicate */",
           "typedef struct { uint32_t itow, tick, t_e; long offset; unsigned relearned; } gpsv_clock_t;",
           "typedef struct { long lat, lon; double x, y; } gpsv_point_t;", "",
           f"#define GPSV_MAX_PAYLOAD {int(c['UBX_MAX_PAYLOAD'])}", f"#define GPSV_LEN_LIMIT {int(c['UBX_LEN_LIMIT'])}",
           f"#define GPSV_BUF_N {int(c['BUF_N'])}",
           "#define GPSV_FLAGS {" + ", ".join(str(int(c["flags"][k])) for k in ("fix_ok", "gate_ok", "head_ok", "fix3d",
                                                                                 "cfg_ok", "init", "ver_ok", "incons")) + "}",
           f"#define GPSV_CK_CHECK {{{', '.join(str(int(x)) for x in gv['checksum_check']['ck'])}}}"]
    counter = [0]

    def msg_struct(m: dict) -> str:
        kind, f = _gps_msg_kind(m)
        pl = bytes.fromhex(m["payload_hex"])
        nm = f"gpsv_pl{counter[0]}"
        counter[0] += 1
        out.append(f"static const uint8_t {nm}[] = {{{_c_bytes(pl)}}};")
        return (f"{{{int(m['cls'])}, {int(m['id'])}, {int(m['length'])}, {int(bool(m['truncated']))}, {kind}, {len(f)}, {nm}, "
                f"{len(pl)}, {{{', '.join(str(x) + 'LL' for x in f) or '0'}}}}}")

    rows = []
    for i, m in enumerate(gv["messages"]):
        out.append(f"static const uint8_t gpsv_mf{i}[] = {{{_c_bytes(bytes.fromhex(m['hex']))}}};")
        rows.append(f"    {{\"{m['name']}\", gpsv_mf{i}, {len(bytes.fromhex(m['hex']))}, {msg_struct(m)}}},")
    out += ["static const gpsv_frame_t GPSV_MSGS[] = {"] + rows + ["};", f"#define GPSV_N_MSGS {len(rows)}"]
    rows = []
    for i, f in enumerate(gv["config_frames"]):
        out.append(f"static const uint8_t gpsv_cf{i}[] = {{{_c_bytes(bytes.fromhex(f['hex']))}}};")
        rows.append(f"    {{\"{f['name']}\", gpsv_cf{i}, {len(bytes.fromhex(f['hex']))}}},")
    out += ["typedef struct { const char *name; const uint8_t *frame; uint16_t n; } gpsv_cfg_t;",
            "static const gpsv_cfg_t GPSV_CFG[] = {"] + rows + ["};", f"#define GPSV_N_CFG {len(rows)}"]
    rows = []
    for i, s in enumerate(gv["streams"]):
        data = bytes.fromhex(s["hex"])
        out.append(f"static const uint8_t gpsv_sd{i}[] = {{{_c_bytes(data)}}};")
        ms = [msg_struct(m) for m in s["expected"]]
        if ms:
            out.append(f"static const gpsv_msg_t gpsv_sm{i}[] = {{{', '.join(ms)}}};")
        st = s["stats"]
        rows.append(f"    {{\"{s['name']}\", gpsv_sd{i}, {len(data)}, {len(ms)}, {f'gpsv_sm{i}' if ms else '0'}, "
                    f"{{{', '.join(str(int(st[k])) + 'u' for k in ('frames', 'bad_ck', 'bad_len', 'truncated', 'skipped'))}}}, "
                    f"{int(s['state_after'])}}},")
    out += ["static const gpsv_stream_t GPSV_STREAMS[] = {"] + rows + ["};", f"#define GPSV_N_STREAMS {len(rows)}"]
    rows = []
    for i, a in enumerate(gv["assembler"]):
        ins = [f"{{{int(x['tick'])}u, {MSG_KIND[x['kind']]}, {int(x['itow_ms'])}u}}" for x in a["input"]]
        out.append(f"static const gpsv_asm_in_t gpsv_ai{i}[] = {{{', '.join(ins)}}};")
        eps = ["{{" + ", ".join(str(int(e[k])) + "LL" for k in EPOCH_KEYS) + "}}" for e in a["epochs"]]
        if eps:
            out.append(f"static const gpsv_epoch_t gpsv_ae{i}[] = {{{', '.join(eps)}}};")
        st = a["stats"]
        rows.append(f"    {{\"{a['name']}\", {len(ins)}, gpsv_ai{i}, {len(eps)}, {f'gpsv_ae{i}' if eps else '0'}, "
                    f"{{{st['epochs']}u, {st['incomplete']}u, {st['duplicate']}u}}}},")
    out += ["static const gpsv_asm_t GPSV_ASM[] = {"] + rows + ["};", f"#define GPSV_N_ASM {len(rows)}"]
    rows = [f"    {{{int(x['itow_ms'])}u, {int(x['tick'])}u, {int(x['t_e_ms'])}u, {int(x['offset'])}L, {int(x['relearned'])}u}},"
            for x in gv["itow_clock"]]
    out += ["static const gpsv_clock_t GPSV_CLOCK[] = {"] + rows + ["};", f"#define GPSV_N_CLOCK {len(rows)}"]
    lf = gv["local_frame"]
    out += [f"#define GPSV_LF_LAT0 {int(lf['lat0_e7'])}L", f"#define GPSV_LF_LON0 {int(lf['lon0_e7'])}L",
            f"#define GPSV_LF_K_N {float(lf['k_n'])!r}", f"#define GPSV_LF_K_E {float(lf['k_e'])!r}",
            "static const gpsv_point_t GPSV_POINTS[] = {"]
    out += [f"    {{{int(q['lat_e7'])}L, {int(q['lon_e7'])}L, {float(q['x_m'])!r}, {float(q['y_m'])!r}}}," for q in lf["points"]]
    out += ["};", f"#define GPSV_N_POINTS {len(lf['points'])}", "", "#endif", ""]
    return "\n".join(out)


def nlink_v4_lines(v4: dict | None) -> list[str]:
    """v0.8.1 nlink v4 block of nlink_vectors_gen.h (sim/nlink_proto.py make_v4_vectors): NAV4 payloads <-> frames (the
    v3 field layout), FCU4 frames, the four decoders on every decode case."""
    if not v4:
        raise SystemExit("build_host_test: nlink_vectors.json has no v4 block - run `python -m sim.nlink_proto`")
    c = v4["constants"]
    out = ["", "/* ---- v0.8.1 nlink v4: NAV4 (the v3 fields), FCU4 (seq state fflags params_tag thr_q pflags kt_q) ---- */",
           "typedef struct { const char *name; const uint8_t *payload; const uint8_t *frame; uint8_t seq; uint32_t fb[12];",
           "                 long iv[9]; uint8_t slot[5]; } nlv4_nav_t;",
           "typedef struct { const char *name; long f[7]; const uint8_t *buf; } nlv4_fcu_t;",
           "typedef struct { const char *name; const uint8_t *buf; uint8_t nav4_valid, fcu4_valid, nav3_valid, fcu_valid;",
           "                 long fcu4[7]; } nlv4_dec_t;",
           f"#define NLV4_TYPE_NAV4 {int(c['TYPE_NAV4'])}", f"#define NLV4_TYPE_FCU4 {int(c['TYPE_FCU4'])}",
           f"#define NLV4_FCU4_FRAME_BYTES {int(c['FCU4_FRAME_BYTES'])}", f"#define NLV4_KT_SCALE {int(c['KT_SCALE'])}",
           f"#define NLV4_TILT_NONE ({int(c['TILT_NONE'])})",
           "#define NLV4_PF {" + ", ".join(str(int(c[k])) for k in ("PF_PHASE_MASK", "PF_MOTOR_ON", "PF_SAT_HI", "PF_LOW_ALT",
                                                                     "PF_GROUND", "PF_LIFTED")) + "}",
           "#define NLV4_PC {" + ", ".join(str(int(c[k])) for k in ("PC_MODE_MASK", "PC_HOLD", "PC_REQ_SHIFT")) + "}",
           f"#define NLV4_STALE_TICKS {int(c['STALE_TICKS'])}", f"#define NLV4_RING {int(c['GNS_SLOT_RING'])}"]
    rows = []
    for i, v in enumerate(v4["navs4"]):
        f = v["fields"]
        out.append(f"static const uint8_t nlv4_np{i}[] = {{{_c_bytes(bytes.fromhex(v['payload_hex']))}}};")
        out.append(f"static const uint8_t nlv4_nf{i}[] = {{{_c_bytes(bytes.fromhex(v['frame_hex']))}}};")
        rows.append(f"    {{\"{v['name']}\", nlv4_np{i}, nlv4_nf{i}, {int(f['seq'])}, "
                    "{" + ", ".join(f"0x{int(b):08X}u" for b in f["float_bits"]) + "}, {"
                    + ", ".join(f"{int(f[k])}L" for k in NAV3_IKEYS) + "}, {" + _c_bytes(bytes.fromhex(f["slot_hex"])) + "}},")
    out += ["static const nlv4_nav_t NLV4_NAVS[] = {"] + rows + ["};", f"#define NLV4_N_NAVS {len(rows)}"]
    fkeys = ("seq", "state", "fflags", "params_tag", "thr_q", "pflags", "kt_q")
    rows = []
    for i, v in enumerate(v4["fcus4"]):
        out.append(f"static const uint8_t nlv4_fb{i}[] = {{{_c_bytes(bytes.fromhex(v['hex']))}}};")
        rows.append(f"    {{\"{v['name']}\", {{{', '.join(str(int(v['fields'][k])) + 'L' for k in fkeys)}}}, nlv4_fb{i}}},")
    out += ["static const nlv4_fcu_t NLV4_FCUS[] = {"] + rows + ["};", f"#define NLV4_N_FCUS {len(rows)}"]
    rows = []
    for i, v in enumerate(v4["decodes"]):
        buf = bytes.fromhex(v["hex"])
        if len(buf) != 72:
            raise SystemExit(f"build_host_test: nlink v4 decode case {v['name']} is {len(buf)} bytes")
        out.append(f"static const uint8_t nlv4_db{i}[] = {{{_c_bytes(buf)}}};")
        f4 = v.get("fcu4") or {k: 0 for k in fkeys}
        rows.append(f"    {{\"{v['name']}\", nlv4_db{i}, {int(v['nav4_valid'])}, {int(v['fcu4_valid'])}, "
                    f"{int(v['nav3_valid'])}, {int(v['fcu_valid'])}, {{{', '.join(str(int(f4[k])) + 'L' for k in fkeys)}}}}},")
    out += ["static const nlv4_dec_t NLV4_DECODES[] = {"] + rows + ["};", f"#define NLV4_N_DECODES {len(rows)}"]
    return out


def _f32_bits(x: float) -> int:
    return struct.unpack("<I", struct.pack("<f", float(x)))[0]


def _ov_tokens(prefix: str, fields: dict) -> list[str]:
    """'prefix.key=value' override tokens of test_core.c (floats as exact float32 text, integers as integers)."""
    out = []
    for k, v in fields.items():
        if isinstance(v, float):
            out.append(f"{prefix}.{k}={struct.unpack('<f', struct.pack('<f', v))[0]:.9g}")
        else:
            out.append(f"{prefix}.{k}={int(v)}")
    return out


def glink_vector_header(path: str = GLINK_VECTORS) -> str:
    """C header with the glink reference vectors (sim/out/glink_vectors.json, sim/glink_proto.py make_test_vectors)."""
    if not os.path.exists(path):
        raise SystemExit(f"build_host_test: {os.path.relpath(path, ROOT)} missing - run `.venv/Scripts/python.exe -m sim.glink_proto`")
    raw = open(path, "rb").read()
    gv = json.loads(raw.decode("utf-8"))
    c = gv["constants"]
    sys.path.insert(0, ROOT)
    tools = os.path.join(ROOT, "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    from sim import gps_nav as gpn  # the replay's parameter objects -> the firmware's C fields (the generator's mapping)
    from sim import position_ctrl as pcm
    import gen_firmware_params as gfp
    out = ["/* glink_vectors_gen.h - GENERATED by tools/build_host_test.py from sim/out/glink_vectors.json",
           f" * (sha256 {hashlib.sha256(raw).hexdigest()}). DO NOT EDIT. */",
           "#ifndef GLINK_VECTORS_GEN_H", "#define GLINK_VECTORS_GEN_H", "#include <stdint.h>", "",
           "/* states: the STATE fields (floats as bit patterns) <-> payload <-> frame; results: the 12 RESULT integers */",
           "typedef struct { const char *name; uint8_t seq; uint32_t tick, fsum[3]; uint8_t n; uint32_t q[4], w[3], h;",
           "                 uint8_t nflags, fcu_state, pflags; uint16_t kt_q; const uint8_t *payload, *frame; } glv_state_t;",
           "typedef struct { const char *name; uint8_t seq; long f[12]; const uint8_t *payload, *frame; } glv_result_t;",
           "typedef struct { const char *name; const uint8_t *buf; uint16_t n; uint8_t valid, reason, type, seq; } glv_dec_t;",
           "typedef struct { uint8_t type, seq; const uint8_t *payload; } glv_frame_t;",
           "typedef struct { const char *name; uint8_t accept; const uint8_t *data; uint16_t n; uint8_t n_frames;",
           "                 const glv_frame_t *frames; unsigned stats[5]; } glv_stream_t;   /* frames bad_crc bad_type wrong_dir skipped */",
           "typedef struct { uint32_t k; uint8_t seq; uint8_t pl[20]; } glv_ev_t;",
           "typedef struct { const char *name; uint8_t L; uint8_t n_ev; const glv_ev_t *ev; uint16_t n_ticks; const uint8_t (*slots)[5];",
           "                 unsigned latched, missed; } glv_tx_t;",
           "/* slot_rx per tick: has_set, the set (flags num_sv hacc_dm mode hold requests pos2 vel2 trim2 tilt2 engaged cycle),",
           " * engaged (target), target x / y bits (the reference's float64 / 1e4 rounded to float32), fallback, requests, gate_ok */",
           "typedef struct { uint8_t has_set; long s[16]; uint8_t engaged; uint32_t tgt[2]; uint8_t fallback, requests, gate_ok; } glv_rxo_t;",
           "typedef struct { const char *name; uint16_t land_steps; uint8_t n; const uint8_t (*slots)[6]; const glv_rxo_t *out;",
           "                 unsigned sets, broken; } glv_rx_t;   /* slots[i][0] = 0: no valid frame that tick */",
           "typedef struct { const char *name; long f[7]; const uint8_t *buf; uint8_t valid; } glv_fcu4_t;",
           "typedef struct { uint32_t tick; const uint8_t *data; uint16_t n; } glv_ubx_t;",
           "typedef struct { const uint8_t *state; const uint8_t *result; double p[2], v[2], b[2]; uint8_t mode;",
           "                 double tilt_l[2], trim_l[2]; uint8_t requests, reset_count; } glv_cycle_t;", "",
           f"#define GLV_STATE_FRAME_BYTES {int(c['STATE_FRAME_BYTES'])}", f"#define GLV_RESULT_FRAME_BYTES {int(c['RESULT_FRAME_BYTES'])}",
           f"#define GLV_STATE_LEN {int(c['STATE_LEN'])}", f"#define GLV_RESULT_LEN {int(c['RESULT_LEN'])}",
           f"#define GLV_GAP_RESTART_TICKS {int(c['GAP_RESTART_TICKS'])}", f"#define GLV_CLOCK_SLIP_TICKS {int(c['CLOCK_SLIP_TICKS'])}",
           f"#define GLV_CRC_CHECK 0x{int(gv['crc16_check']['crc']):04X}",
           "#define GLV_NF {" + ", ".join(str(int(c["NF"][k])) for k in ("pad", "fcu_ok", "imu_ok", "est_valid")) + "}",
           f"#define GLV_RING {int(c['GNS_SLOT_RING'])}"]
    rows = []
    for i, v in enumerate(gv["states"]):
        f = v["fields"]
        out.append(f"static const uint8_t glv_sp{i}[] = {{{_c_bytes(bytes.fromhex(v['payload_hex']))}}};")
        out.append(f"static const uint8_t glv_sf{i}[] = {{{_c_bytes(bytes.fromhex(v['frame_hex']))}}};")
        rows.append(f"    {{\"{v['name']}\", {int(v['seq'])}, {int(f['tick'])}u, {{{', '.join(f'0x{b:08X}u' for b in f['fsum_bits'])}}}, "
                    f"{int(f['n'])}, {{{', '.join(f'0x{b:08X}u' for b in f['q_bits'])}}}, "
                    f"{{{', '.join(f'0x{b:08X}u' for b in f['w_bits'])}}}, 0x{int(f['h_bits']):08X}u, {int(f['nflags'])}, "
                    f"{int(f['fcu_state'])}, {int(f['pflags'])}, {int(f['kt_q'])}, glv_sp{i}, glv_sf{i}}},")
    out += ["static const glv_state_t GLV_STATES[] = {"] + rows + ["};", f"#define GLV_N_STATES {len(rows)}"]
    rkeys = ("tilt_x", "tilt_y", "flags", "num_sv", "hacc_dm", "pc", "pos_x_cm", "pos_y_cm", "vel_x_cms", "vel_y_cms",
             "trim_x", "trim_y")
    rows = []
    for i, v in enumerate(gv["results"]):
        out.append(f"static const uint8_t glv_rp{i}[] = {{{_c_bytes(bytes.fromhex(v['payload_hex']))}}};")
        out.append(f"static const uint8_t glv_rf{i}[] = {{{_c_bytes(bytes.fromhex(v['frame_hex']))}}};")
        if "".join(v["parts_hex"]) != v["payload_hex"]:
            raise SystemExit(f"build_host_test: glink result {v['name']}: the slot parts are not the payload")
        rows.append(f"    {{\"{v['name']}\", {int(v['seq'])}, {{{', '.join(str(int(v['fields'][k])) + 'L' for k in rkeys)}}}, "
                    f"glv_rp{i}, glv_rf{i}}},")
    out += ["static const glv_result_t GLV_RESULTS[] = {"] + rows + ["};", f"#define GLV_N_RESULTS {len(rows)}"]
    reasons = {"": 0, "length": 1, "sync": 2, "type": 3, "crc": 4}
    rows = []
    for i, v in enumerate(gv["decodes"]):
        buf = bytes.fromhex(v["hex"])
        out.append(f"static const uint8_t glv_db{i}[] = {{{_c_bytes(buf)}}};")
        rows.append(f"    {{\"{v['name']}\", glv_db{i}, {len(buf)}, {int(v['valid'])}, {reasons[v['reason']]}, "
                    f"{int(v.get('type', 0))}, {int(v.get('seq', 0))}}},")
    out += ["static const glv_dec_t GLV_DECODES[] = {"] + rows + ["};", f"#define GLV_N_DECODES {len(rows)}"]
    rows = []
    for i, v in enumerate(gv["streams"]):
        data = bytes.fromhex(v["hex"])
        out.append(f"static const uint8_t glv_td{i}[] = {{{_c_bytes(data)}}};")
        fr = []
        for j, e in enumerate(v["frames"]):
            out.append(f"static const uint8_t glv_tp{i}_{j}[] = {{{_c_bytes(bytes.fromhex(e['payload_hex']))}}};")
            fr.append(f"{{{int(e['type'])}, {int(e['seq'])}, glv_tp{i}_{j}}}")
        if fr:
            out.append(f"static const glv_frame_t glv_tf{i}[] = {{{', '.join(fr)}}};")
        acc = 0
        for t in v["accept"]:
            acc |= 1 if int(t) == 1 else 2
        st = ", ".join(f"{int(v['stats'][k])}u" for k in ("frames", "bad_crc", "bad_type", "wrong_dir", "skipped"))
        rows.append(f"    {{\"{v['name']}\", {acc}, glv_td{i}, {len(data)}, {len(fr)}, {f'glv_tf{i}' if fr else '0'}, {{{st}}}}},")
    out += ["static const glv_stream_t GLV_STREAMS[] = {"] + rows + ["};", f"#define GLV_N_STREAMS {len(rows)}"]
    rows = []
    for i, v in enumerate(gv["slot_tx"]):
        ev = [f"{{{int(e['k'])}u, {int(e['seq'])}, {{{_c_bytes(bytes.fromhex(e['payload_hex']))}}}}}" for e in v["events"]]
        out.append(f"static const glv_ev_t glv_xe{i}[] = {{{', '.join(ev)}}};")
        out.append(f"static const uint8_t glv_xs{i}[][5] = {{{', '.join('{' + _c_bytes(bytes.fromhex(s)) + '}' for s in v['slots'])}}};")
        rows.append(f"    {{\"{v['name']}\", {int(v['L'])}, {len(ev)}, glv_xe{i}, {len(v['slots'])}, glv_xs{i}, "
                    f"{int(v['latched'])}u, {int(v['missed'])}u}},")
    out += ["static const glv_tx_t GLV_TX[] = {"] + rows + ["};", f"#define GLV_N_TX {len(rows)}"]
    rows = []
    for i, v in enumerate(gv["slot_rx"]):
        sl = ["{" + ("0, 0, 0, 0, 0, 0" if s is None else "1, " + _c_bytes(bytes.fromhex(s))) + "}" for s in v["slots"]]
        out.append(f"static const uint8_t glv_ys{i}[][6] = {{{', '.join(sl)}}};")
        outs = []
        for o in v["out"]:
            g = o["set"]
            if g is None:
                s16 = "{" + ", ".join(["0L"] * 16) + "}"
                has = 0
            else:
                has = 1
                q = lambda x: str(int(round(float(x) * 100.0))) + "L"   # noqa: E731 (cm / 100: exact back)
                t = lambda x: str(int(round(float(x) * 10000.0))) + "L"  # noqa: E731 (x 1e-4: exact back)
                s16 = (f"{{{g['flags']}L, {g['num_sv']}L, {g['hacc_dm']}L, {g['mode']}L, {int(g['hold'])}L, {g['requests']}L, "
                       f"{q(g['pos'][0])}, {q(g['pos'][1])}, {q(g['vel'][0])}, {q(g['vel'][1])}, {t(g['trim'][0])}, "
                       f"{t(g['trim'][1])}, {t(g['tilt'][0])}, {t(g['tilt'][1])}, {int(g['engaged'])}L, {g['cycle']}L}}")
            tg = ", ".join(f"0x{_f32_bits(x):08X}u" for x in o["target"])
            outs.append(f"{{{has}, {s16}, {int(o['engaged'])}, {{{tg}}}, {int(o['fallback'])}, {int(o['requests'])}, "
                        f"{int(o['gate_ok'])}}}")
        out.append(f"static const glv_rxo_t glv_yo{i}[] = {{{', '.join(outs)}}};")
        rows.append(f"    {{\"{v['name']}\", {int(v['land_steps'])}, {len(v['slots'])}, glv_ys{i}, glv_yo{i}, {int(v['sets'])}u, "
                    f"{int(v['broken'])}u}},")
    out += ["static const glv_rx_t GLV_RX[] = {"] + rows + ["};", f"#define GLV_N_RX {len(rows)}"]
    fkeys = ("seq", "state", "fflags", "params_tag", "thr_q", "pflags", "kt_q")
    rows = []
    for i, v in enumerate(gv["fcu4"]):
        out.append(f"static const uint8_t glv_fb{i}[] = {{{_c_bytes(bytes.fromhex(v['hex']))}}};")
        rows.append(f"    {{\"{v['name']}\", {{{', '.join(str(int(v['fields'][k])) + 'L' for k in fkeys)}}}, glv_fb{i}, "
                    f"{int(v['valid'])}}},")
    out += ["static const glv_fcu4_t GLV_FCU4[] = {"] + rows + ["};", f"#define GLV_N_FCU4 {len(rows)}"]
    # the GNS replay: its parameter objects -> the firmware's fields (the generator's mapping) as override tokens
    rp = gv["gns_replay"]
    npar = {k: (tuple(v) if isinstance(v, list) else v) for k, v in rp["nav_params"].items()}
    gp = gpn.NavGpsParams(**npar)
    pp = pcm.PositionParams(**rp["pos_params"])
    toks = _ov_tokens("gp", gfp.gps_c_fields(gp)) + _ov_tokens("pp", gfp.pos_c_fields(pp))
    out.append("static const char *const GLV_REPLAY_OV[] = {" + ", ".join(f'"{t}"' for t in toks) + "};")
    out.append(f"#define GLV_REPLAY_N_OV {len(toks)}")
    out.append(f"#define GLV_REPLAY_LAT0 {int(rp['local_frame_e7'][0])}L")
    out.append(f"#define GLV_REPLAY_LON0 {int(rp['local_frame_e7'][1])}L")
    ub = sorted(((int(k), bytes.fromhex(h)) for k, h in rp["ubx_by_tick"].items()))
    for i, (k, b) in enumerate(ub):
        out.append(f"static const uint8_t glv_ub{i}[] = {{{_c_bytes(b)}}};")
    out.append("static const glv_ubx_t GLV_REPLAY_UBX[] = {" + ", ".join(f"{{{k}u, glv_ub{i}, {len(b)}}}"
                                                                        for i, (k, b) in enumerate(ub)) + "};")
    out.append(f"#define GLV_REPLAY_N_UBX {len(ub)}")
    rows = []
    for i, s in enumerate(rp["states"]):
        out.append(f"static const uint8_t glv_cs{i}[] = {{{_c_bytes(bytes.fromhex(s['state_hex']))}}};")
        out.append(f"static const uint8_t glv_cr{i}[] = {{{_c_bytes(bytes.fromhex(s['result_hex']))}}};")
        rows.append(f"    {{glv_cs{i}, glv_cr{i}, {{{s['p'][0]!r}, {s['p'][1]!r}}}, {{{s['v'][0]!r}, {s['v'][1]!r}}}, "
                    f"{{{s['b'][0]!r}, {s['b'][1]!r}}}, {int(s['mode'])}, {{{s['tilt_l'][0]!r}, {s['tilt_l'][1]!r}}}, "
                    f"{{{s['trim_l'][0]!r}, {s['trim_l'][1]!r}}}, {int(s['requests'])}, {int(s['reset_count'])}}},")
    out += ["static const glv_cycle_t GLV_REPLAY[] = {"] + rows + ["};", f"#define GLV_REPLAY_N {len(rows)}"]
    st = rp["stats"]
    out.append("#define GLV_REPLAY_STATS {" + ", ".join(str(int(st[k])) for k in ("states", "hold_steps", "gaps", "restarts",
                                                                                   "imu_bad", "clock_slips", "misaligned")) + "}")
    fs = rp["filter_stats"]
    out.append("#define GLV_REPLAY_FSTATS {" + ", ".join(str(int(fs[k])) for k in ("fused", "rej_p", "rej_v", "pos_resets",
                                                                                    "vel_resets", "dropped_old",
                                                                                    "dropped_stale", "n_epoch_dropped",
                                                                                    "unusable")) + "}")
    out += ["", "#endif", ""]
    return "\n".join(out)


def _write_if_changed(path: str, text: str) -> None:
    old = open(path, "r", encoding="utf-8").read() if os.path.exists(path) else None
    if old != text:
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)


# fw09 G1 / X1: the vehicle configurations of params_gen.h (-D FC_VCFG; tools/gen_firmware_params.py VCFG_ID).  A host
# build without FC_VCFG compiles the rocket set (the unit tests' historical numbers); the VTOL configurations get their
# own binaries test_core[_<profile>]_vcfg<n>.
VCFG_OF_POWER = {"battery": 0, "tether": 1, "hybrid": 2}
VCFG_NAMES = {0: "VTOL battery", 1: "VTOL tether", 2: "VTOL hybrid", 3: "rocket"}
# the matrix of --configs: (profile, vcfg) - None = the header's default (field) / no FC_VCFG (rocket)
CONFIG_MATRIX = ((None, None), (None, 0), (None, 1), (None, 2), ("demo", None), ("demo", 0), ("demo", 1), ("demo", 2))


def exe_path(profile: str | None = None, vcfg: int | None = None, pwr_src: int | None = None) -> str:
    """test_core[_<profile>][_vcfg<n>][_pwr<m>](.exe) in the build directory."""
    name = ("test_core" + (f"_{profile}" if profile else "") + ("" if vcfg is None else f"_vcfg{int(vcfg)}") +
            ("" if pwr_src is None else f"_pwr{int(pwr_src)}"))
    return os.path.join(BUILD, name + (".exe" if os.name == "nt" else ""))


def build(verbose: bool = False, profile: str | None = None, vcfg: int | None = None, pwr_src: int | None = None) -> str:
    """Build test_core (params_gen.h's default build profile) or, with ``profile``, test_core_<profile> compiled with
    -D MISSION_PROFILE_<PROFILE> (v0.7.1: the unit tests then check that profile's numbers); fw09: with ``vcfg``
    (0 battery, 1 tether, 2 hybrid, 3 rocket) -D FC_VCFG=<vcfg>, the VTOL configuration's vehicle constants; with
    ``pwr_src`` (1, 2) also -D PWR_SRC, the default supervisor table of the FCU's power envs (the crosscheck's
    tether / hybrid cores; the unit tests are written for the host's battery default and run without it)."""
    os.makedirs(BUILD, exist_ok=True)
    _write_if_changed(VECTOR_HEADER, vector_header())
    _write_if_changed(ILINK_VECTOR_HEADER, ilink_vector_header())
    _write_if_changed(NLINK_VECTOR_HEADER, nlink_vector_header())
    _write_if_changed(GPS_VECTOR_HEADER, gps_vector_header())
    _write_if_changed(GLINK_VECTOR_HEADER, glink_vector_header())
    defs = []
    if profile:
        if not re.fullmatch(r"[a-z][a-z0-9_]*", profile):
            raise SystemExit(f"build_host_test: bad profile name {profile!r}")
        defs = [f"-DMISSION_PROFILE_{profile.upper()}"]
    if vcfg is not None:
        if int(vcfg) not in VCFG_NAMES:
            raise SystemExit(f"build_host_test: FC_VCFG {vcfg!r} not in {sorted(VCFG_NAMES)}")
        defs.append(f"-DFC_VCFG={int(vcfg)}")
    if pwr_src is not None:                              # as the FCU's power envs (fcu_wired / _buffer: PWR_SRC = FC_VCFG):
        if int(pwr_src) not in (1, 2):                   # core/fc_config.c fills the build's power table and tethered
            raise SystemExit(f"build_host_test: PWR_SRC {pwr_src!r} not in (1, 2)")   # limits as the defaults
        defs.append(f"-DPWR_SRC={int(pwr_src)}")
    exe = exe_path(profile, vcfg, pwr_src)
    cmd = [sys.executable, "-m", "ziglang", "cc", *CFLAGS, *defs, "-I", BUILD, "-o", exe, TEST, *core_sources(), "-lm"]
    if verbose:
        print(" ".join(os.path.relpath(c, ROOT) if os.path.isabs(c) and c.startswith(ROOT) else c for c in cmd))
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.stdout.strip():
        print(r.stdout.strip())
    if r.stderr.strip():
        print(r.stderr.strip())
    if r.returncode != 0:
        raise SystemExit(f"build_host_test: compiler failed (exit {r.returncode})")
    return exe


QFIX_RUNNER = os.path.join(FW, "tools", "qfix_test", "run_qfix_test.py")


def run_unit(exe: str, quiet: bool = False) -> tuple[int, str]:
    """Run one test_core binary's unit tests; returns (exit code, its summary line)."""
    r = subprocess.run([exe], cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
    out = r.stdout.rstrip()
    if not quiet:
        print(out)
    else:                                                # the failures and the summary only
        print("\n".join(x for x in out.splitlines() if x.startswith("FAIL") or x.startswith("test_core:")))
    if r.stderr.strip():
        print(r.stderr.rstrip())
    summary = next((x for x in reversed(out.splitlines()) if x.startswith("test_core:")), "test_core: no summary")
    return r.returncode, summary


def run_qfix() -> int:
    """fw09 WP-X: the AVR inline-asm fx_mulhi against its C definition (200000 pairs in simavr,
    firmware/atmega128_fc/tools/qfix_test).  Exit 0 = pass; 77 = the AVR toolchain or simavr is missing (not run)."""
    r = subprocess.run([sys.executable, QFIX_RUNNER], cwd=ROOT, capture_output=True, text=True, encoding="utf-8",
                       errors="replace")
    print(r.stdout.rstrip())
    if r.stderr.strip():
        print(r.stderr.rstrip())
    return r.returncode


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run", action="store_true", help="run the unit tests after building")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--profile", default=None,
                    help="v0.7.1: build test_core_<profile> with -D MISSION_PROFILE_<PROFILE> (e.g. demo; default: the "
                         "header's default profile)")
    ap.add_argument("--vcfg", type=int, default=None, choices=sorted(VCFG_NAMES),
                    help="fw09: build test_core[_<profile>]_vcfg<n> with -D FC_VCFG=<n> (0 VTOL battery, 1 tether, "
                         "2 hybrid, 3 rocket; default: no FC_VCFG = the rocket set)")
    ap.add_argument("--pwr-src", type=int, default=None, choices=(1, 2),
                    help="fw09: also -D PWR_SRC=<n> (the FCU power envs' defaults; test_core..._pwr<n>)")
    ap.add_argument("--configs", action="store_true",
                    help="fw09: build and run the unit tests of every configuration build (field and demo x rocket / "
                         "battery / tether / hybrid) - the regular test set together with --qfix")
    ap.add_argument("--qfix", action="store_true",
                    help="fw09: run the AVR fx_mulhi vs C test in simavr (200000 pairs; needs PlatformIO's "
                         "toolchain-atmelavr and tool-simavr)")
    a = ap.parse_args(argv)
    if a.configs:
        rc_all, rows = 0, []
        build(a.verbose)                                 # the vector headers once, before the matrix
        for prof, vc in CONFIG_MATRIX:
            exe = build(a.verbose, prof, vc)
            print(f"== {os.path.relpath(exe, ROOT)} (profile {prof or 'default'}, "
                  f"{VCFG_NAMES[vc] if vc is not None else 'no FC_VCFG: rocket set'})")
            rc, summary = run_unit(exe, quiet=True)
            rows.append((os.path.basename(exe), summary, rc))
            rc_all |= rc
        if a.qfix:
            rq = run_qfix()
            rows.append(("qfix_test (AVR fx_mulhi vs C, simavr)", "PASS" if rq == 0 else
                         ("NOT RUN (no AVR toolchain / simavr)" if rq == 77 else "FAIL"), rq if rq != 77 else 0))
            rc_all |= rq if rq != 77 else 0
        print("== summary")
        for name, summary, rc in rows:
            print(f"   {name:40s} {summary}{'' if rc == 0 else '  <- FAIL'}")
        return 1 if rc_all else 0
    exe = build(a.verbose, a.profile, a.vcfg, a.pwr_src)
    print(f"built {os.path.relpath(exe, ROOT)}")
    rc = 0
    if a.run:
        rc, _ = run_unit(exe)
    if a.qfix:
        rq = run_qfix()
        rc |= rq if rq != 77 else 0
    return rc


if __name__ == "__main__":
    sys.exit(main())
