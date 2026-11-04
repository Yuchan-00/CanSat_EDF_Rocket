#!/usr/bin/env python3
"""Measure the COM (LoRa link processor, v0.6; v0.7 both mission modes) main loop in the simavr ATmega128 simulator.

v0.7: params.json ``mission.mode`` is a build-time choice (``src/fc_mode.h``).  The VTOL COM (the plain envs, which fly
params.json's mode - vtol) accepts only IL_SNAP2 from the FCU and sends TELEM v2 (39-byte frames: + phase and the
altitude target); its synthetic ground sends ON with the nonce, its synthetic FCU answers with SNAP2.  The ballistic COM
(``*_ballistic`` envs) is the v0.6.3 one.  ``--mode both`` (default) / ``vtol`` / ``ballistic``; each mode writes
``host_test/build/com_simbench_report_<mode>.json`` (``com_simbench_report.json`` = params.json's mode), and the reply-window
check uses that mode's TELEM frame length.

v0.7.1: the VTOL build profiles (params.json mission.vtol.profiles: field / demo, params_gen.h MISSION_PROFILE) change only
the FCU's numbers; the COM must not depend on them, so a demo vehicle flies fcu_demo + the one nav and com image.  The
VTOL run builds com / com_lora_uart / com_sx127x in a temporary copy of the project (src/, platformio.ini,
tools/pio_mission_mode.py - PlatformIO rebuilds a project's whole .pio/build when PLATFORMIO_BUILD_FLAGS changes) without
and with -D MISSION_PROFILE_DEMO and requires byte-identical HEX files (and the copy's plain images equal to this
project's).  The COM code itself is v0.7.0's: the V8 / V9 / V10 reason codes (21 / 22 / 23) travel in the SNAP2 /
TELEM2 reason byte untouched.

Builds the PlatformIO environments ``com_simbench`` (lora_uart backend) and ``com_simbench_sx127x``
(SX127x backend) - the unchanged COM firmware with a synthetic ground station feeding the radio driver and
a synthetic FCU answering on the ilink (``src/com/com_simbench.h``) - runs each ELF in PlatformIO's
``tool-simavr`` (ATmega128, 16 MHz, cycle-accurate for the AVR instruction set) and parses the console:

* ``SB  t=.. pm=.. pa=.. np=.. sf=.. cl=.. sl=.. el=.. bu=.. s=..`` once per simulated second (hex):
  longest / mean main-loop pass (Timer1, 0.5 us), passes, never-touched stack, the window's maxima of
  uplink frame complete at the COM -> IL_CMD queued (cl), SNAP complete at the COM -> TELEM handed to the
  radio (sl), uplink complete -> TELEM handed to the radio (el; includes the synthetic FCU's one-tick reply
  and the 55-byte SNAP transfer), the burst-window flag and the per-section maxima;
* ``SL  t=.. up=.. fw=.. ...`` the link counters and the synthetic peers' checks;
* ``SD  t=.. rf=.. ...`` D4 (IL_STAT pacing, seen by the synthetic FCU on the wire) and D5 (reply gate: the
  synthetic FCU's deliberate stray / duplicate / late SNAPs, the ground's TELEM-age check);
* ``SI  t=.. cf=.. ...`` both ilink decoders' counters (the COM's, built with IL_ACCEPT_COM, and the synthetic FCU's,
  IL_ACCEPT_FCU) and the boot STAT's time (R3: real time from com_timer_init to the moment the COM queued it, and its
  com_ticks field) - checked against ``COM_STAT_BOOT_MS`` (src/com/com_link.h);
* ``SBW f=..`` the two wrong-direction frames (a valid IL_CMD and IL_STAT) the synthetic FCU sends to the COM once
  (R4): the COM's decoder counters must equal ``sim.ilink_proto.IlinkDecoder(ACCEPT_COM)`` fed with these bytes;
* ``SBT s=.. n=<SNAP payload> f=<TELEM frame>`` for every TELEM the COM handed to the radio: checked here
  against ``sim.ilink_proto.telem_frame_from_snap`` (the normative Python pipeline): every byte equal except
  roll / pitch / yaw, which may differ by one 0.01 deg quantum (float32 avr-libc vs float64; then the CRC
  differs too and is checked for validity instead).

The reply window (src/com/com_link.h COM_REPLY_WINDOW_TICKS, read from the firmware's boot line) is checked
against ``sim/link_model.py`` for both radios: a TELEM handed to the radio within the window (plus the measured
uplink -> IL_CMD time and the radio's own start delay) must reach the ground before its reply timeout, and must
be off the air before the ground's next request can be on the air even with a stuck AUX line (lora_uart).

The flight builds ``com``, ``com_lora_uart`` and ``com_sx127x`` are built as well for their flash / RAM; the
stack depth measured in the bench build is carried over to the flight build of the same backend (static RAM
differs: the bench has its console and peers).  The co-simulation builds ``com_cosim``, ``com_cosim_sx127x``
and ``fcu_cosim`` are built too; for the COM ones the ``g_cosim_report`` symbol (avr-nm) must have the size of
``src/com/com_cosim.h`` and the header's layout is compiled on the host (zig cc) to the same offsets.  The window
holding the stress burst (``bu=1``: back-to-back uplinks, the newest-reply-wins path) is reported separately -
its reply latencies include waiting for the radio by design.  Not simulated (see com_simbench.h): the radio
module and the ilink USART interrupts.

    .venv/Scripts/python.exe firmware/atmega128_fc/tools/run_com_simbench.py

Writes firmware/atmega128_fc/host_test/build/com_simbench_report.json; exit status 0 only if both
backends pass, the window and the co-sim layout checks pass and every COM build is free of warnings
(fcu_cosim is FCU code: its warnings, or its build failure, are printed, not judged).
"""
from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys

FW = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ROOT = os.path.normpath(os.path.join(FW, "..", ".."))
BUILD = os.path.join(FW, "host_test", "build")
OUT = os.path.join(BUILD, "com_simbench_report.json")
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from sim import ilink_proto as il  # noqa: E402
from sim import link_model as lm  # noqa: E402
from sim import link_proto as lp  # noqa: E402

# envs per mission mode (platformio.ini): the plain envs fly params.json's mode (vtol), *_ballistic pin the ballistic one
MODES = {
    "vtol": {"bench": {"com_simbench": "com_lora_uart", "com_simbench_sx127x": "com_sx127x"},   # bench -> flight build
             "flight": ("com", "com_lora_uart", "com_sx127x"),
             "cosim": ("com_cosim", "com_cosim_sx127x", "fcu_cosim"), "foreign": ("fcu_cosim",),
             "radio": {"com_simbench": "lora_uart", "com_simbench_sx127x": "sx127x"},
             # v0.8: SNAP3 -> TELEM3 (the GPS FCU; an ONB_NO_GPS FCU's SNAP2 -> TELEM2 is accepted too); v0.9 RP1: the
             # power builds' TELEM5 (47 B) is the longest reply - the window check uses it, the bench sends all four
             "accept": il.ACCEPT_COM_VTOL | il.ACCEPT_COM_GPS, "telem_bytes": lp.TELEM5_FRAME_BYTES,
             "snap_bytes": il.SNAP3_FRAME_BYTES,
             "start": "ON with the nonce read back from TELEM3 -> POWERED"},
    "ballistic": {"bench": {"com_simbench_ballistic": "com_lora_uart_ballistic",
                            "com_simbench_sx127x_ballistic": "com_sx127x_ballistic"},
                  "flight": ("com_ballistic", "com_lora_uart_ballistic", "com_sx127x_ballistic"),
                  "cosim": ("com_cosim_ballistic", "com_cosim_sx127x_ballistic", "fcu_cosim_ballistic"),
                  "foreign": ("fcu_cosim_ballistic",),
                  "radio": {"com_simbench_ballistic": "lora_uart", "com_simbench_sx127x_ballistic": "sx127x"},
                  "accept": il.ACCEPT_COM, "telem_bytes": lp.TELEM_FRAME_BYTES, "snap_bytes": il.SNAP_FRAME_BYTES,
                  "start": "ARM + LAUNCH with the nonce read back from TELEM -> POWERED"},
}
MODE = "vtol"                             # the mode being run (run_mode sets it)
BENCH_ENVS = MODES[MODE]["bench"]
FLIGHT_ENVS = MODES[MODE]["flight"]
COSIM_ENVS = MODES[MODE]["cosim"]
FOREIGN_ENVS = MODES[MODE]["foreign"]     # built for the co-simulation's convenience; its result is the FCU's business
BENCH_RADIO = MODES[MODE]["radio"]
SECTION_NAMES = ["tick work (LED) + radio_service step", "uplink: radio_read -> lp decoder -> IL_CMD (+ STAT due)",
                 "ilink: ilink_uart_read -> il_decoder_push -> reply gate -> SNAP", "one costly job (reply stage 1 / 2 or IL_STAT)",
                 "pending TELEM -> radio_send, stack scan slice"]
STATES = ["SAFE", "ARMED", "POWERED", "UNPOWERED", "RELEASED", "ABORT"]
RAM_BYTES = 4096                          # ATmega128 SRAM
STACK_MIN_FREE = 1024                     # never-touched stack the flight build must keep (as the FCU)
ILINK_RX_RING = 64                        # hal/ilink_uart.h ILINK_UART_RX_BUF
SX127X_TX_STEPS = 2                       # hal/sx127x.c radio_service: S_RX -> S_TX_LOAD, ..., S_TX_START (+ the chunks)
D5_FAULTS = 3                             # com_simbench.c: stray, duplicate and late SNAP


def simavr_exe() -> str:
    home = os.environ.get("PLATFORMIO_CORE_DIR") or os.path.join(os.path.expanduser("~"), ".platformio")
    exe = os.path.join(home, "packages", "tool-simavr", "bin", "simavr" + (".exe" if os.name == "nt" else ""))
    if not os.path.exists(exe):
        raise SystemExit(f"simavr not found at {exe}: python -m platformio pkg install -g --tool platformio/tool-simavr")
    return exe


def avr_nm_exe() -> str:
    home = os.environ.get("PLATFORMIO_CORE_DIR") or os.path.join(os.path.expanduser("~"), ".platformio")
    return os.path.join(home, "packages", "toolchain-atmelavr", "bin", "avr-nm" + (".exe" if os.name == "nt" else ""))


def hex_sha256(env: str, fw: str | None = None) -> str | None:
    import hashlib
    path = os.path.join(fw or FW, ".pio", "build", env, "firmware.hex")
    if not os.path.exists(path):
        return None
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def profile_identity(envs: tuple[str, ...]) -> dict:
    """v0.7.1: the COM images must be byte-identical for the field and the demo build profile (module docstring)."""
    import shutil
    import tempfile
    tmp = tempfile.mkdtemp(prefix="com_profile_")
    out: dict = {"envs": {}}
    try:
        shutil.copytree(os.path.join(FW, "src"), os.path.join(tmp, "src"))
        shutil.copy2(os.path.join(FW, "platformio.ini"), tmp)
        os.makedirs(os.path.join(tmp, "tools"))
        shutil.copy2(os.path.join(FW, "tools", "pio_mission_mode.py"), os.path.join(tmp, "tools"))
        for flags in (None, "-D MISSION_PROFILE_DEMO"):
            envv = dict(os.environ)
            envv.pop("PLATFORMIO_BUILD_FLAGS", None)
            if flags:
                envv["PLATFORMIO_BUILD_FLAGS"] = flags
            args = [sys.executable, "-m", "platformio", "run", "-d", tmp] + [x for e in envs for x in ("-e", e)]
            r = subprocess.run(args, capture_output=True, text=True, encoding="utf-8", errors="replace", env=envv)
            if r.returncode != 0:
                raise SystemExit(f"profile check build ({flags or 'field'}) failed:\n{r.stdout[-3000:]}")
            for e in envs:
                out["envs"].setdefault(e, {})["demo" if flags else "field"] = hex_sha256(e, tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    for e in envs:
        d = out["envs"][e]
        d["project"] = hex_sha256(e)
        d["identical"] = d["field"] is not None and d["field"] == d["demo"]
        d["copy_equals_project"] = d["field"] == d["project"]
    out["pass"] = all(d["identical"] and d["copy_equals_project"] for d in out["envs"].values())
    return out


def build(env: str, required: bool = True) -> dict:
    r = subprocess.run([sys.executable, "-m", "platformio", "run", "-d", FW, "-e", env], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        if not required:                      # another chip's build (fcu_cosim): reported, not this tool's verdict
            errs = sorted(set(re.findall(r"(?:error|undefined reference)[^\n]*", r.stdout + r.stderr)))
            return {"elf": None, "ram_bytes": None, "flash_bytes": None, "warnings": [], "failed": True,
                    "errors": errs[:20]}
        raise SystemExit(f"build {env} failed:\n{r.stdout[-3000:]}\n{r.stderr[-2000:]}")
    ram = re.search(r"RAM:.*used (\d+) bytes from (\d+)", r.stdout)
    flash = re.search(r"Flash:.*used (\d+) bytes from (\d+)", r.stdout)
    warnings = sorted(set(re.findall(r"warning: [^\n]*", r.stdout + r.stderr)))
    return {"elf": os.path.join(FW, ".pio", "build", env, "firmware.elf"),
            "ram_bytes": int(ram.group(1)) if ram else None, "flash_bytes": int(flash.group(1)) if flash else None,
            "warnings": warnings, "hex_sha256": hex_sha256(env)}


def run(elf: str, timeout_s: float = 300.0) -> str:
    r = subprocess.run([simavr_exe(), "-m", "atmega128", "-f", "16000000", elf], capture_output=True, timeout=timeout_s)
    return (r.stdout + r.stderr).decode("latin-1")


def kv(line: str) -> dict[str, int]:
    return {k: int(v, 16) for k, v in re.findall(r"(?<![0-9a-z])([a-z]+)=(0x[0-9a-f]+)", line)}


def parse(text: str) -> dict:
    sb, sl, sd, si, sbt = [], [], [], [], []
    for m in re.finditer(r"SB (t=[ -~]*)", text):
        line = m.group(1)
        row = kv(line)
        secs = re.search(r"(?<![a-z0-9])s=((?:0x[0-9a-f]+ ?)+)", line)
        row["sections_us"] = [int(x, 16) for x in secs.group(1).split()] if secs else []
        row.pop("s", None)
        sb.append(row)
    for m in re.finditer(r"SL (t=[ -~]*)", text):
        sl.append(kv(m.group(1)))
    for m in re.finditer(r"SD (t=[ -~]*)", text):
        sd.append(kv(m.group(1)))
    for m in re.finditer(r"SI (t=[ -~]*)", text):
        si.append(kv(m.group(1)))
    for m in re.finditer(r"SBT s=(0x[0-9a-f]+) n=([0-9a-f]+) f=([0-9a-f]+)", text):
        sbt.append((int(m.group(1), 16), bytes.fromhex(m.group(2)), bytes.fromhex(m.group(3))))
    sbw = [bytes.fromhex(m.group(1)) for m in re.finditer(r"SBW f=([0-9a-f]+)", text)]
    boots = [kv(m.group(1)) | {"radio": m.group(0).split("radio=")[1].split()[0]}
             for m in re.finditer(r"SB BOOT (radio=[ -~]*)", text)]
    return {"sb": sb, "sl": sl, "sd": sd, "si": si, "sbt": sbt, "sbw": sbw, "boots": boots, "end": "SB END" in text}


def wrong_direction_check(sbw: list[bytes], si: dict, sd: dict, fcu_frames: int) -> dict:
    """R4 on the AVR: the synthetic FCU's wrong-direction frames (a valid IL_CMD and IL_STAT) against the COM's decoder
    counters, predicted by the normative Python decoder with the COM's accept mask (every other ilink byte the COM
    received belongs to a valid SNAP, so the COM's cumulative counters must equal the prediction)"""
    data = b"".join(sbw)
    tap = il.IlinkDecoder(il.ACCEPT_ALL)
    frames_all = tap.feed(data)
    com = il.IlinkDecoder(MODES[MODE]["accept"])      # IL_ACCEPT_COM (ballistic) / IL_ACCEPT_COM_VTOL
    frames_com = com.feed(data)
    want = {k: com.stats[k] for k in ("bad_crc", "bad_len", "bad_type", "skipped_bytes")}
    got = {"bad_crc": si.get("cc"), "bad_len": si.get("cl"), "bad_type": si.get("ct"), "skipped_bytes": si.get("ck")}
    r = {"frames_sent": si.get("ws"), "frames_printed": len(sbw), "bytes": len(data),
         "types_with_accept_all": [f.type for f in frames_all],
         "python_accept_com": {"frames": len(frames_com), "pending_after": com.pending, **want},
         "com_decoder": {"frames": si.get("cf"), "wrong_type": si.get("cw"), **got},
         "fcu_decoder": {"frames": si.get("ff"), "bad_crc": si.get("fc"), "bad_len": si.get("fl"),
                         "bad_type": si.get("ft"), "skipped_bytes": si.get("fk"), "il_cmd_plus_stat": fcu_frames},
         "_doc": "the COM decoder is built with IL_ACCEPT_COM: both frames must be rejected in the header (Python "
                 "IlinkDecoder(ACCEPT_COM) gives the exact counters), the decoder must be back in the hunting state "
                 "for the next SNAP, and every frame it returned is a SNAP (accepted or refused by the reply gate); the "
                 "synthetic FCU's decoder (IL_ACCEPT_FCU) rejected nothing"}
    r["pass"] = (len(sbw) == 2 and si.get("ws") == 2 and [f.type for f in frames_all] == [il.TYPE_CMD, il.TYPE_STAT]
                 and not frames_com and com.pending == 0 and want["bad_len"] + want["bad_type"] == 2 and got == want
                 and si.get("cw") == 0
                 and si.get("cf") == (sd.get("sa") or 0) + (sd.get("rf") or 0)
                 and si.get("fc") == si.get("fl") == si.get("ft") == si.get("fk") == 0
                 and si.get("ff") == fcu_frames)
    r["rejected"] = want["bad_crc"] + want["bad_len"] + want["bad_type"]
    return r


def load_params() -> dict:
    with open(os.path.join(ROOT, "params.json"), "r", encoding="utf-8") as f:
        return json.load(f)


def header_define(path: str, name: str) -> int:
    with open(path, "r", encoding="utf-8") as f:
        m = re.search(rf"#define\s+{name}\s+\(?(\d+)[uUlL]*", f.read())
    if not m:
        raise SystemExit(f"{name} not found in {path}")
    return int(m.group(1))


def check_telem(sbt: list[tuple[int, bytes, bytes]], vid: int, loop_hz: int) -> dict:
    """the COM's TELEM (avr-libc float32) vs the Python pipeline (float64): crosscheck_controller's rule"""
    st = {"frames": len(sbt), "exact": 0, "angle_fields_one_quantum_off": 0, "max_angle_diff_cdeg": 0,
          "mismatches": 0, "first_mismatches": [], "by_type": {}}
    for seq, payload, frame in sbt:
        ref = il.telem_frame_from_snap(il.Snap.unpack(payload), vid, loop_hz)
        tn = {lp.TYPE_TELEM: "TELEM", lp.TYPE_TELEM2: "TELEM2", lp.TYPE_TELEM3: "TELEM3", lp.TYPE_TELEM4: "TELEM4",
              lp.TYPE_TELEM5: "TELEM5"}.get(frame[4] if len(frame) > 4 else -1, "?")
        st["by_type"][tn] = st["by_type"].get(tn, 0) + 1          # v0.9 RP1: the TELEM TYPEs the COM built
        if frame == ref:
            st["exact"] += 1
            continue
        ok = (len(frame) == len(ref) and frame[4] == ref[4] and frame[5] == seq
              and lp.crc16_ccitt(frame[2:-2]) == (frame[-2] | frame[-1] << 8))
        other = [j for j in range(min(len(frame), len(ref)) - 2) if frame[j] != ref[j] and not 20 <= j < 26]
        diffs = [abs(int.from_bytes(frame[o:o + 2], "little", signed=True) - int.from_bytes(ref[o:o + 2], "little", signed=True))
                 for o in il.TELEM_ANGLE_OFFSETS] if ok else [99]
        st["angle_fields_one_quantum_off"] += sum(1 for d in diffs if d == 1)
        st["max_angle_diff_cdeg"] = max(st["max_angle_diff_cdeg"], max(diffs))
        if ok and not other and max(diffs) <= 1:
            continue
        st["mismatches"] += 1
        if len(st["first_mismatches"]) < 5:
            st["first_mismatches"].append({"seq": seq, "com": frame.hex(), "py": ref.hex()})
    st["pass"] = st["frames"] > 0 and st["mismatches"] == 0
    return st


def reply_window_check(p: dict, win_ticks: int, tick_hz: int, cl_max_us: dict, pass_max_us: dict) -> dict:
    """COM_REPLY_WINDOW_TICKS against sim/link_model.py, both radios (com_link.h states the derivation).  Times from the
    moment the ground starts writing a request: the uplink reaches the COM at uplink_s, the IL_CMD is queued cl later,
    the TELEM is handed to the radio < window after that and leaves after the radio's own start delay."""
    aux_ticks = header_define(os.path.join(FW, "src", "params_gen.h"), "LINK_AUX_WAIT_TICKS")
    chunk = header_define(os.path.join(FW, "src", "hal", "sx127x.h"), "SX127X_CHUNK")
    w = win_ticks / tick_hz
    out: dict = {"window_ms": round(w * 1e3, 3), "window_ticks": win_ticks, "telem_frame_bytes": MODES[MODE]["telem_bytes"],
                 "radios": {}}
    ok = True
    for radio in lm.RADIOS:
        cfg = lm.LinkConfig.from_params(p, radio=radio)
        # the ground's reply timeout for THIS mode's reply frame (v0.8: the station waits round_trip(cfg, mode='vtol',
        # gps=True) for a TELEM3 - tools/ground_station.py link_timing; v0.7.1 used the TELEM v1 timeout here)
        rt = lm.round_trip(cfg, telem_bytes=MODES[MODE]["telem_bytes"])
        up = lm.hop_timing(cfg, "up", lp.CMD_FRAME_BYTES)
        down = lm.hop_timing(cfg, "down", MODES[MODE]["telem_bytes"])     # this mode's TELEM (v2: 39 B)
        arrive_bound = rt["reply_timeout_s"] - rt["uplink_s"] - down.arrival
        air_bound = rt["reply_timeout_s"] + up.air_start - rt["uplink_s"] - down.air_end
        cl = cl_max_us[radio] * 1e-6
        if radio == "lora_uart":
            start_idle, start_worst = 0.0, (aux_ticks + 1) / tick_hz      # AUX ready / stuck AUX (LINK_AUX_WAIT_TICKS)
            note = f"start = AUX wait: 0 with the module idle, {(aux_ticks + 1) * 1e3 / tick_hz:.0f} ms with a stuck AUX"
        else:
            steps = math.ceil(MODES[MODE]["telem_bytes"] / chunk) + SX127X_TX_STEPS
            start_idle = start_worst = steps * pass_max_us[radio] * 1e-6  # one radio_service step per main-loop pass
            note = f"start = FIFO load: {steps} radio_service steps x the measured pass max"
        r = {"reply_timeout_ms": round(rt["reply_timeout_s"] * 1e3, 1), "uplink_at_com_ms": round(rt["uplink_s"] * 1e3, 1),
             "bound_arrive_before_timeout_ms": round(arrive_bound * 1e3, 1),
             "bound_off_air_before_next_request_ms": round(air_bound * 1e3, 1),
             "uplink_to_il_cmd_ms": round(cl * 1e3, 2), "radio_start_idle_ms": round(start_idle * 1e3, 1),
             "radio_start_worst_ms": round(start_worst * 1e3, 1), "_doc": note}
        r["margin_arrive_ms"] = round((arrive_bound - (w + cl + start_idle)) * 1e3, 1)
        r["margin_off_air_worst_ms"] = round((air_bound - (w + cl + start_worst)) * 1e3, 1)
        r["pass"] = r["margin_arrive_ms"] > 0 and r["margin_off_air_worst_ms"] > 0
        ok = ok and r["pass"]
        out["radios"][radio] = r
    out["pass"] = ok
    return out


def cosim_layout(builds: dict) -> dict:
    """g_cosim_report in the COM co-sim ELFs (avr-nm size) vs com_cosim.h compiled on the host (zig cc)"""
    hdr = os.path.join(FW, "src", "com", "com_cosim.h")
    want = header_define(hdr, "COM_COSIM_REPORT_BYTES")
    res: dict = {"header_bytes": want}
    com_envs = COSIM_ENVS[:2]
    for env in com_envs:
        r = subprocess.run([avr_nm_exe(), "-S", builds[env]["elf"]], capture_output=True, text=True, encoding="utf-8",
                           errors="replace")
        m = re.search(r"^([0-9a-f]+) ([0-9a-f]+) [BbDd] g_cosim_report$", r.stdout, re.M)
        res[env] = {"address": f"0x{int(m.group(1), 16):06x}", "data_index": int(m.group(1), 16) - 0x800000,
                    "bytes": int(m.group(2), 16)} if m else None
    probe_c = os.path.join(BUILD, "com_cosim_probe.c")
    probe_exe = os.path.join(BUILD, "com_cosim_probe" + (".exe" if os.name == "nt" else ""))
    fields = ["magic", "version", "size", "seqlock", "radio", "done", "t_us", "ticks", "passes", "pass_max_us",
              "stack_free_min", "sec_max_us", "pass_max_s_us", "ground_state", "reply_window_ticks", "c", "ul_count",
              "stat_count", "ul", "st"]
    os.makedirs(BUILD, exist_ok=True)
    with open(probe_c, "w", encoding="utf-8", newline="\n") as f:
        f.write('#include <stdio.h>\n#include "com_cosim.h"\nint main(void)\n{\n')
        f.write('    printf("sizeof %u\\n", (unsigned)sizeof(com_cosim_report_t));\n')
        for fl in fields:
            f.write(f'    printf("{fl} %u\\n", (unsigned)offsetof(com_cosim_report_t, {fl}));\n')
        f.write("    return 0;\n}\n")
    r = subprocess.run([sys.executable, "-m", "ziglang", "cc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                        "-I", os.path.dirname(hdr), "-o", probe_exe, probe_c], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        res["host"] = {"error": (r.stdout + r.stderr)[-1500:]}
    else:
        o = subprocess.run([probe_exe], capture_output=True, text=True, encoding="utf-8", errors="replace").stdout
        res["host"] = {k: int(v) for k, v in (ln.split() for ln in o.split("\n") if ln.strip())}
    res["pass"] = (all(res[e] is not None and res[e]["bytes"] == want for e in com_envs)
                   and res["host"].get("sizeof") == want)
    return res


def bench(env: str, builds: dict, text: str, vid: int, loop_hz: int) -> dict:
    b = builds[env]
    d = parse(text)
    if not d["sb"] or not d["sl"] or not d["sd"] or not d["si"] or not d["end"]:
        print(text[-2000:])
        raise SystemExit(f"{env}: no complete report (did the firmware reach its loop?)")
    sb, sl, sd = d["sb"], d["sl"], d["sd"]
    boot = d["boots"][0] if d["boots"] else {}
    normal = [r for r in sb if not r.get("bu")]
    burst = [r for r in sb if r.get("bu")]
    last, lastd, lasti = sl[-1], sd[-1], d["si"][-1]

    def mx(rows: list[dict], key: str) -> int:
        return max((r.get(key, 0) for r in rows), default=0)

    flight = builds[BENCH_ENVS[env]]
    s: dict = {"windows": len(sb), "radio": boot.get("radio"), "boots": len(d["boots"]),
               "constants": {k: boot.get(k) for k in ("win", "quiet", "div", "hz", "poll", "secs", "boot", "wrong")}}
    s["pass_max_us"] = mx(sb, "pm")
    s["pass_mean_us"] = round(sum(r.get("pa", 0) * r.get("np", 0) for r in sb) / max(1, sum(r.get("np", 0) for r in sb)), 1)
    s["passes_per_s"] = round(sum(r.get("np", 0) for r in sb) / len(sb))
    s["sections_max_us"] = dict(zip(SECTION_NAMES, [max((r["sections_us"][i] for r in sb if len(r["sections_us"]) > i),
                                                        default=0) for i in range(len(SECTION_NAMES))]))
    s["latency_us"] = {
        "uplink_complete_to_il_cmd_queued_max": mx(normal, "cl"),
        "snap_complete_to_telem_to_radio_max": mx(normal, "sl"),
        "uplink_complete_to_telem_to_radio_max": mx(normal, "el"),
        "_doc": "maxima over the one-second windows without the stress burst; times of the synthetic bytes are their "
                "scheduled arrival times (lora_uart 1 byte / 10 bit times at the module baud, sx127x the whole packet "
                "at RxDone, SNAP bytes at ILINK_BAUD), so a latency includes waiting for the pass in progress. "
                "uplink -> TELEM includes the synthetic FCU (reply at the next COM tick, its compute time not "
                f"modelled) and the {MODES[MODE]['snap_bytes']}-byte SNAP transfer"}
    s["burst_window"] = {"windows": len(burst), "pass_max_us": mx(burst, "pm"),
                         "uplink_complete_to_il_cmd_queued_max_us": mx(burst, "cl"),
                         "snap_complete_to_telem_to_radio_max_us": mx(burst, "sl"),
                         "uplink_complete_to_telem_to_radio_max_us": mx(burst, "el"),
                         "_doc": "back-to-back uplinks (SB_BURST_N): replies wait for the busy radio, only the newest is kept"}
    s["counters"] = {"uplinks_sent": last.get("up"), "il_cmd_at_fcu": last.get("fw"), "snaps": last.get("sn"),
                     "telem_to_radio": last.get("rp"), "ground_state": STATES[last["gs"]] if last.get("gs", 9) < 6 else last.get("gs"),
                     "ground_seq_acked": last.get("ga"), "stats_at_fcu": last.get("st"), "stat_check_failures": last.get("sx"),
                     "il_cmd_check_failures": last.get("cb"), "nonce_rejections": last.get("nr"),
                     "telem_bad_at_ground": last.get("tb"), "burst_newest_reply_sent": last.get("bl"),
                     "wrong_type_frames_at_fcu": last.get("wf"),
                     "fcu_tx_dropped": last.get("fd"), "stat_replies": last.get("xr"), "stat_replies_dropped": last.get("xd"),
                     "stat_loop_us_max_last": last.get("xl"), "stat_il_bad": last.get("xb"),
                     "stat_il_tx_dropped": last.get("xt"), "ilink_rx_overflow_errors": last.get("xo"),
                     "radio_rx_overflow_errors": last.get("ro"), "radio_tx_timeouts": last.get("rt"),
                     "radio_tx_frames": last.get("tf"),
                     "console_dropped": last.get("td")}
    s["d5_reply_gate"] = {"replies_refused": lastd.get("rf"), "replies_expired": lastd.get("re"),
                          "replies_superseded": lastd.get("rs"), "snaps_accepted": lastd.get("sa"),
                          "deliberate_bad_snaps_sent": lastd.get("bs"), "refusals_matched_to_an_uplink": lastd.get("gr"),
                          "telem_out_of_window_at_ground": lastd.get("ow"), "telem_unmatched_at_ground": lastd.get("um"),
                          "il_cmd_to_telem_max_us": lastd.get("ag"), "burst_newest_expired": lastd.get("be"),
                          "expiry_events": lastd.get("xe"),
                          "_doc": "the synthetic FCU sends a stray SNAP (never-forwarded reply_seq), one reply twice and one "
                                  "reply after the window: all three must be refused; the ground checks every TELEM's age "
                                  "against the window and that no uplink is answered twice"}
    s["d4_stat_pacing"] = {"com_piggyback": lastd.get("sp"), "com_quiet": lastd.get("sq"), "com_sent": lastd.get("ss"),
                           "fcu_boot": lastd.get("fb"), "fcu_piggyback": lastd.get("fp"), "fcu_quiet": lastd.get("fq"),
                           "fcu_unpaced": lastd.get("fu"), "cmd_behind_piggyback_stat": lastd.get("cp"),
                           "cmd_behind_boot_or_quiet_stat": lastd.get("co"), "gap_min_ticks": lastd.get("gm"),
                           "gap_max_piggyback_ticks": lastd.get("gp"), "gap_max_quiet_ticks": lastd.get("gq"),
                           "_doc": "STAT kinds counted by the COM and classified on the wire by the synthetic FCU (boot; "
                                   "right behind an IL_CMD; no IL_CMD for 1.5 / stat_hz); an IL_CMD starting < 1 FCU tick "
                                   "after a STAT ends is 'behind' it; gaps from the STATs' com_ticks"}
    boot_ms = header_define(os.path.join(FW, "src", "com", "com_link.h"), "COM_STAT_BOOT_MS")
    s["r3_boot_stat"] = {"hold_ms": boot_ms, "hold_ticks": boot.get("boot"),
                         "queued_us_after_com_timer_init": lasti.get("bt"), "com_ticks_at_fcu": lasti.get("bk"),
                         "_doc": "the boot STAT (IL_STAT_BOOT) is queued no earlier than COM_STAT_BOOT_MS after the COM's "
                                 "reset: real time (Timer1, never frozen) from com_timer_init to the stat_send hook, and "
                                 "the com_ticks field of the first STAT the synthetic FCU decoded"}
    s["r3_boot_stat"]["pass"] = (boot.get("boot") is not None and boot["boot"] * 1000 >= boot_ms * boot.get("hz", 0) > 0
                                 and (lasti.get("bt") or 0) >= boot_ms * 1000 and lasti.get("bt") != 0xFFFFFFFF
                                 and boot["boot"] <= (lasti.get("bk") or 0) < 0xFFFFFFFF)
    s["r4_wrong_direction"] = wrong_direction_check(d["sbw"], lasti, lastd, (last.get("fw") or 0) + (last.get("st") or 0))
    s["telem_check"] = check_telem(d["sbt"], vid, loop_hz)
    s["stack"] = {"bench_static_ram_bytes": b["ram_bytes"], "bench_free_min_bytes": min(r.get("sf", 0) for r in sb)}
    s["stack"]["used_bytes"] = RAM_BYTES - b["ram_bytes"] - s["stack"]["bench_free_min_bytes"]
    s["stack"]["flight_env"] = BENCH_ENVS[env]
    s["stack"]["flight_static_ram_bytes"] = flight["ram_bytes"]
    s["stack"]["flight_free_bytes"] = RAM_BYTES - flight["ram_bytes"] - s["stack"]["used_bytes"]
    ring_us = ILINK_RX_RING * 10 * 1_000_000 // ilink_baud()
    s["ilink_rx_ring_us"] = ring_us
    c, g, q = s["counters"], s["d5_reply_gate"], s["d4_stat_pacing"]
    burst_check = ("stress burst: an older reply superseded; the newest one reached the radio (sx127x: or expired, D5 - "
                   "simavr has no TxDone, the radio holds every TELEM for its 208 ms TX timeout); nothing else expired")
    div, quiet = boot.get("div", 0), boot.get("quiet", 0)
    checks = {
        "one boot (no watchdog reset), SB END reached": s["boots"] == 1 and d["end"],
        "every TELEM equals the Python pipeline (angles within one 0.01 deg quantum)": s["telem_check"]["pass"],
        "one TELEM line per TELEM handed to the radio": s["telem_check"]["frames"] == c["telem_to_radio"],
        "the radio sent only the TELEM frames built from SNAPs (driver tx_frames vs TELEM handed over)":
            (c["telem_to_radio"] or 0) - 1 <= (c["radio_tx_frames"] or 0) <= (c["telem_to_radio"] or 0),
        "every uplink reached the FCU as IL_CMD, unchanged": (c["il_cmd_at_fcu"] or 0) >= (c["uplinks_sent"] or 0) - 1
                                                            and c["il_cmd_check_failures"] == 0,
        MODES[MODE]["start"]: c["ground_state"] == "POWERED" and c["nonce_rejections"] == 0,
        "every IL_STAT valid (tag, backend, BOOT once, gap >= 1 / stat_hz, cmd_forwarded)":
            (c["stats_at_fcu"] or 0) >= s["windows"] * div // (div + quiet) and c["stat_check_failures"] == 0,
        burst_check: len(burst) == 1 and (g["replies_superseded"] or 0) > 0
                     and ((c["burst_newest_reply_sent"] == 1 and g["burst_newest_expired"] == 0 and g["replies_expired"] == 0)
                          or (s["radio"] == "sx127x" and c["burst_newest_reply_sent"] == 0 and g["burst_newest_expired"] == 1
                              and g["replies_expired"] == 1 and g["expiry_events"] == 1)),
        "no ilink / radio RX overflow or error, no rejected ilink frame but the R4 wrong-direction ones":
            not (c["ilink_rx_overflow_errors"] or c["radio_rx_overflow_errors"] or c["wrong_type_frames_at_fcu"]
                 or c["stat_il_tx_dropped"] or c["telem_bad_at_ground"])
            and c["stat_il_bad"] == s["r4_wrong_direction"]["rejected"],
        "R4: the COM's decoder (IL_ACCEPT_COM) rejected the IL_CMD and IL_STAT sent to it in the header, exactly as "
        "sim/ilink_proto.py predicts, and stayed in step; the FCU-side decoder (IL_ACCEPT_FCU) rejected nothing":
            s["r4_wrong_direction"]["pass"],
        f"R3: the boot STAT queued >= {boot_ms} ms after the COM's reset (real time and com_ticks)": s["r3_boot_stat"]["pass"],
        f"D5: the {D5_FAULTS} deliberate bad SNAPs (stray, duplicate, late) were all refused, no other SNAP":
            g["deliberate_bad_snaps_sent"] == D5_FAULTS and g["replies_refused"] == D5_FAULTS,
        "D5: every TELEM handed to the radio < reply window after its IL_CMD, none answers an uplink twice":
            g["telem_out_of_window_at_ground"] == 0 and g["telem_unmatched_at_ground"] == 0,
        "D5: IL_STAT replies_dropped <= superseded + refused + expired (the last STAT is an earlier snapshot)":
            c["stat_replies_dropped"] is not None and c["stat_replies_dropped"] <= (g["replies_superseded"] or 0)
            + (g["replies_refused"] or 0) + (g["replies_expired"] or 0),
        "D4: every STAT is the boot STAT, right behind an IL_CMD, or quiet (FCU classification = COM's)":
            q["fcu_unpaced"] == 0 and q["fcu_boot"] == 1 and q["fcu_piggyback"] == q["com_piggyback"]
            and q["fcu_quiet"] == q["com_quiet"] and (q["com_piggyback"] or 0) > 0 and (q["com_quiet"] or 0) > 0,
        "D4: no IL_CMD right behind a STAT": q["cmd_behind_piggyback_stat"] == 0 and q["cmd_behind_boot_or_quiet_stat"] == 0,
        "D4: STAT gaps within [1 / stat_hz, 1 / stat_hz + quiet + 1 tick]":
            (q["gap_min_ticks"] or 0) >= div and max(q["gap_max_piggyback_ticks"] or 0, q["gap_max_quiet_ticks"] or 0) <= div + quiet + 1,
        f"pass max < ilink RX ring fill time ({ring_us} us)": s["pass_max_us"] < ring_us,
        f"flight stack free >= {STACK_MIN_FREE} B": s["stack"]["flight_free_bytes"] >= STACK_MIN_FREE,
        "console complete, no hook event lost": c["console_dropped"] == 0 and lastd.get("ev") == 0,
    }
    s["checks"] = checks
    s["pass"] = all(checks.values())
    lat = s["latency_us"]
    print(f"[{MODE}] {env} ({s['radio']}, {len(sb)} one-second windows, {s['passes_per_s']} passes/s):")
    print(f"  main-loop pass: max {s['pass_max_us']} us, mean {s['pass_mean_us']} us "
          f"(ilink RX ring {ILINK_RX_RING} B = {ring_us} us at ILINK_BAUD)")
    for name, us in s["sections_max_us"].items():
        print(f"    {us:5d} us  {name}")
    print(f"  latency: uplink complete -> IL_CMD queued <= {lat['uplink_complete_to_il_cmd_queued_max']} us; "
          f"SNAP complete -> TELEM to radio <= {lat['snap_complete_to_telem_to_radio_max']} us; "
          f"uplink complete -> TELEM to radio <= {lat['uplink_complete_to_telem_to_radio_max']} us")
    bw = s["burst_window"]
    print(f"  burst window: pass max {bw['pass_max_us']} us, SNAP -> TELEM <= {bw['snap_complete_to_telem_to_radio_max_us']} us "
          f"(waits for the radio), superseded {g['replies_superseded']}, expired {g['replies_expired']}")
    print(f"  link: {c['uplinks_sent']} uplinks, {c['il_cmd_at_fcu']} IL_CMD at the FCU, {c['snaps']} SNAPs, "
          f"{c['telem_to_radio']} TELEM to the radio, ground state {c['ground_state']}, {c['stats_at_fcu']} IL_STAT, "
          f"radio TX timeouts {c['radio_tx_timeouts']}")
    print(f"  D5: refused {g['replies_refused']} of {g['deliberate_bad_snaps_sent']} deliberate bad SNAPs, expired "
          f"{g['replies_expired']}, superseded {g['replies_superseded']}; IL_CMD -> TELEM max {g['il_cmd_to_telem_max_us']} us "
          f"(window {boot.get('win', 0) * 1000 // max(1, boot.get('hz', 1))} ms), out of window {g['telem_out_of_window_at_ground']}")
    print(f"  D4: STATs boot {q['fcu_boot']} / behind an IL_CMD {q['fcu_piggyback']} / quiet {q['fcu_quiet']} / unpaced "
          f"{q['fcu_unpaced']}; IL_CMD behind a STAT {q['cmd_behind_piggyback_stat']} + {q['cmd_behind_boot_or_quiet_stat']}; "
          f"gaps {q['gap_min_ticks']}..{max(q['gap_max_piggyback_ticks'] or 0, q['gap_max_quiet_ticks'] or 0)} ticks")
    r3, r4 = s["r3_boot_stat"], s["r4_wrong_direction"]
    print(f"  R3: boot STAT queued {r3['queued_us_after_com_timer_init']} us after com_timer_init, com_ticks "
          f"{r3['com_ticks_at_fcu']} (hold {r3['hold_ms']} ms = {r3['hold_ticks']} ticks)")
    print(f"  R4: {r4['frames_printed']} wrong-direction frames ({r4['bytes']} B, types {r4['types_with_accept_all']}) -> "
          f"COM decoder {r4['com_decoder']} vs Python ACCEPT_COM {r4['python_accept_com']}; FCU-side decoder "
          f"{r4['fcu_decoder']}")
    tc = s["telem_check"]
    print(f"  TELEM TYPEs built: {tc.get('by_type')}")
    print(f"  TELEM vs sim/ilink_proto.py: {tc['frames']} frames, {tc['exact']} byte-exact, "
          f"{tc['angle_fields_one_quantum_off']} angle fields one quantum off, {tc['mismatches']} mismatches")
    st = s["stack"]
    print(f"  stack: {st['used_bytes']} B deep (bench {st['bench_static_ram_bytes']} B static, {st['bench_free_min_bytes']} B "
          f"never touched) -> {st['flight_env']} ({st['flight_static_ram_bytes']} B static) keeps {st['flight_free_bytes']} B free")
    for k, v in checks.items():
        print(f"    [{'ok' if v else 'FAIL'}] {k}")
    print(f"  -> {'PASS' if s['pass'] else 'FAIL'}")
    return s


def ilink_baud() -> int:
    return int(load_params()["flight_computer"]["ilink"]["baud"])


def run_mode(mode: str, p: dict) -> tuple[dict, bool]:
    global MODE, BENCH_ENVS, FLIGHT_ENVS, COSIM_ENVS, FOREIGN_ENVS, BENCH_RADIO
    MODE = mode
    BENCH_ENVS, FLIGHT_ENVS, COSIM_ENVS = MODES[mode]["bench"], MODES[mode]["flight"], MODES[mode]["cosim"]
    FOREIGN_ENVS, BENCH_RADIO = MODES[mode]["foreign"], MODES[mode]["radio"]
    vid, loop_hz = int(p["flight_computer"]["link"]["vehicle_id"]), int(p["control"]["loop_hz"])
    builds = {env: build(env, required=env not in FOREIGN_ENVS) for env in (*FLIGHT_ENVS, *BENCH_ENVS, *COSIM_ENVS)}
    report: dict = {"mode": mode, "envs": {e: {k: v for k, v in b.items() if k != "elf"} for e, b in builds.items()},
                    "stack_min_free_bytes": STACK_MIN_FREE}
    print(f"[{mode}] COM builds (flash / static RAM of 128 KiB / 4 KiB; {FOREIGN_ENVS[0]} is the co-simulation FCU, not "
          "judged here):")
    for e, b in builds.items():
        if b.get("failed"):
            print(f"  {e:<22} BUILD FAILED (FCU code, not judged here):", *b["errors"], sep="\n    ")
            continue
        print(f"  {e:<22} flash {b['flash_bytes']:6d} B   RAM {b['ram_bytes']:5d} B   warnings {len(b['warnings'])}")
    texts = {env: run(builds[env]["elf"]) for env in BENCH_ENVS}
    for env in BENCH_ENVS:
        report[env] = bench(env, builds, texts[env], vid, loop_hz)
    c0 = report[next(iter(BENCH_ENVS))]["constants"]
    rw = reply_window_check(p, c0["win"], c0["hz"],
                            {BENCH_RADIO[e]: report[e]["latency_us"]["uplink_complete_to_il_cmd_queued_max"] for e in BENCH_ENVS},
                            {BENCH_RADIO[e]: report[e]["pass_max_us"] for e in BENCH_ENVS})
    report["reply_window"] = rw
    print(f"reply window {rw['window_ms']:g} ms ({rw['window_ticks']} COM ticks) vs sim/link_model.py:")
    for radio, r in rw["radios"].items():
        print(f"  {radio:<9} timeout {r['reply_timeout_ms']} ms, uplink at the COM {r['uplink_at_com_ms']} ms -> TELEM within "
              f"{r['bound_arrive_before_timeout_ms']} ms reaches the ground in time (margin {r['margin_arrive_ms']} ms), "
              f"within {r['bound_off_air_before_next_request_ms']} ms is off the air before its next request "
              f"(margin {r['margin_off_air_worst_ms']} ms with {r['radio_start_worst_ms']} ms radio start) "
              f"[{'ok' if r['pass'] else 'FAIL'}]")
    cl = cosim_layout(builds)
    report["cosim_layout"] = cl
    print(f"co-sim report struct: header {cl['header_bytes']} B; "
          + "; ".join(f"{e} g_cosim_report {cl[e]['bytes'] if cl[e] else None} B at {cl[e]['address'] if cl[e] else None}"
                      for e in COSIM_ENVS[:2])
          + f"; host (zig cc) sizeof {cl['host'].get('sizeof')} [{'ok' if cl['pass'] else 'FAIL'}]")
    for e in FOREIGN_ENVS:                    # FCU code (src/main.c ...): reported, not this tool's verdict
        if builds[e]["warnings"]:
            print(f"{e} warnings (FCU code, not judged here):", *builds[e]["warnings"], sep="\n  ")
    ok = (all(report[e]["pass"] for e in BENCH_ENVS) and rw["pass"] and cl["pass"]
          and not any(b["warnings"] for e, b in builds.items() if e not in FOREIGN_ENVS))
    if mode == "vtol":                        # v0.7.1: one COM image for both build profiles
        pc = profile_identity(FLIGHT_ENVS)
        report["profile_check"] = pc
        print("profile check (field vs demo): " + "; ".join(
            f"{e} {'identical' if d['identical'] else 'DIFFERENT'} ({(d['field'] or '-')[:16]})" for e, d in pc["envs"].items())
              + f" [{'ok' if pc['pass'] else 'FAIL'}]")
        ok = ok and pc["pass"]
    report["pass"] = ok
    print(f"[{mode}] COM simbench:", "PASS" if ok else "FAIL")
    return report, ok


def main(argv: list[str] | None = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--mode", choices=("both", "vtol", "ballistic"), default="both",
                    help="vtol: the plain envs (params.json mission.mode); ballistic: the *_ballistic envs; both (default)")
    a = ap.parse_args(argv)
    p = load_params()
    params_mode = str(p.get("mission", {}).get("mode", "ballistic"))
    if params_mode != "vtol" and a.mode in ("both", "vtol"):
        raise SystemExit(f"params.json mission.mode is {params_mode!r}: the plain envs are not the VTOL build")
    ok = True
    for mode in (["vtol", "ballistic"] if a.mode == "both" else [a.mode]):
        report, ok_m = run_mode(mode, p)
        ok = ok and ok_m
        os.makedirs(BUILD, exist_ok=True)
        paths = [os.path.join(BUILD, f"com_simbench_report_{mode}.json")] + ([OUT] if mode == params_mode else [])
        for path in paths:
            with open(path, "w", encoding="utf-8", newline="\n") as f:
                json.dump(report, f, indent=1)
            print(f"-> {os.path.relpath(path, ROOT)}")
    print("COM simbench (all modes run):", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
