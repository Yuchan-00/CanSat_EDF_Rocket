#!/usr/bin/env python3
"""Measure the NAV (navigation computer, v0.6.3; v0.7.1 both mission modes) 250 Hz loop in the simavr ATmega128 simulator.

v0.7.1: the NAV firmware depends on the mission mode (``src/fc_mode.h``).  ``--mode vtol`` (the plain envs ``nav`` /
``nav_simbench`` / ``nav_cosim``, params.json's mode) measures the VTOL NAV - the gated Mahony estimator with its
accelerometer gate OPEN (the bench's flight-mode specific force is a hover's, ~1 g) and the VTOL altitude filter with
the baro thrust compensation running (``NAV_SB_COMP_M``, a bench input) on the synthetic FCU's hover throttle (thr_q) -
and checks that the estimator-correction flag (nav.h NAV_F_EST_GATE) appears only in flight-mode frames, that the NAV
image is byte-identical for the field and demo build profiles, and that the FCU bench's synthetic NAV
(``src/simbench.h`` SB_NAV_PUSH_US_VTOL + NLINK_XFER_US) is not earlier than this measured frame-ready time.  The VTOL
NAV never pushes its frame earlier than nav.h ``NAV_PUSH_MIN_US_VTOL`` after the tick (the estimator branches differ by
up to ~0.75 ms: pushed as soon as ready, the frame-ready time jumped by more than the FCU's frame-timeout margin); the
bench reports the compute end before that wait (SB ``ce``) and the ticks past the floor (``lp``), the composed figures
are max(the measured time, compute end + the barometer's bus interrupts + the wire exchange), and the floor must bound the
variation: compute end + bus interrupts - floor <= the NAV variation the FCU's background timing allows (simbench.h
SB_NAV_JITTER_US, inside main.c's NLINK_EARLY_US);
``--mode ballistic`` the ``*_ballistic`` envs (v0.6.3 estimator and filter); ``--mode both`` (default) both.  Reports
``host_test/build/nav_simbench_report_<mode>.json``; ``nav_simbench_report.json`` is params.json's mode.

Builds the PlatformIO environments ``nav`` (flight), ``nav_simbench`` (the NAV loop with the FCU simbench's synthetic
sensors and a synthetic FCU behind nlink_spi's single-chip bypass, ``src/nav/nav_simbench.h``) and ``nav_cosim`` (the
same sensors, the REAL SPI master), runs ``nav_simbench`` and ``nav_cosim`` - the latter alone: simavr's SPI master
clocks every byte at the programmed rate with nothing on MISO, so no FCU frame is valid and the NAV stays in pad mode,
which gives the real exchange on the wire - in PlatformIO's ``tool-simavr`` (ATmega128, 16 MHz, cycle-accurate for the
AVR instruction set) and parses the console (numbers hex, formats in ``nav_simbench.h``):

* ``SB  t=.. st=.. pd=.. cm=.. cp=.. cf=.. fr=.. fp=.. ff=.. fn=.. jx=.. tm=.. ov=.. sf=.. bg=.. ox=.. s=..`` 1 Hz:
  time-critical part = tick -> frame delivered (SS high), pad / flight maxima, frame ready after the Timer2 ISR
  (max, min, largest tick-to-tick increase), loop incl. background, missed ticks, stack, sections;
* ``SN  t=.. fv=.. fb=.. ex=.. lm=.. rc=.. rd=..`` the NAV's nlink counters and frame fields;
* ``SF  t=.. nv=.. nx=.. sg=.. pm=.. ...`` (single-chip) the synthetic FCU's checks of the NAV frames: validity, SEQ
  continuity, the BOOT / EST_VALID / pad-flight rules of ``core/nlink_proto.h``, the reply-refusal and params-tag fault
  injections, late baro samples.

The frame-ready time on the wire in flight mode is composed: the bench's flight figure with its bypass exchange
section replaced by the stand-alone nav_cosim's real exchange section (both measured; the rest of the tick is the same
code).  Interrupts not in the measured figures (no I2C slave in simavr): every TWI interrupt of the barometer's longest
bus script (BARO_TWI_SCRIPTS below: it runs inside the time-critical part of the script's tick - BMP280: the data read
in phase BMP280_READ_PHASE, the shorter forced write in phase BMP280_START_PHASE; MS5611: phases 0 and MS5611_D1_PHASE - of every
FC_BARO_DIV ticks), counted from the
flight ELF's disassembly (4 response + 3 vector JMP + the longest path through ``__vector_33``).  The MPU-6050 burst read
(Timer0 lead ISR, FC_IMU_LEAD_COUNTS before the tick) runs while the NAV is idle; the check is that the loop incl. its
background ends before that lead.  Stack: the canary high-water mark of the simulated loops (bench and cosim) plus the
deepest interrupt frame NOT exercised in simavr (TWI, Timer0 lead -> mpu6050_start_async), from the disassembly.

The SS pin (PORTB bit 0) is traced into a VCD (simavr ``-o`` / ``--add-trace``, host_test/build/nav_vcd/) for nav_cosim and
for the FLIGHT build, which runs in simavr for ``--flight-wall-s`` host seconds (it never stops by itself; no I2C slave,
so both sensors fail their boot init and every IMU read fails): SS low = the exchange on the wire, SS rise to SS rise =
the spacing of the FCU's frame-triggered ticks.  The flight build must boot, tick and exchange once per tick with no gap
beyond the FCU's frame timeout (a watchdog reset or a hang would show as one).

    .venv/Scripts/python.exe firmware/atmega128_fc/tools/run_nav_simbench.py

Writes firmware/atmega128_fc/host_test/build/nav_simbench_report.json.  Pass: the three NAV builds succeed with no
warning other than gains.h's GAINS_PARAMS_STALE; the bench reached POWERED with 0 missed ticks and every synthetic-FCU
check clean; time-critical part on the wire incl. the TWI interrupts <= 80 % of the tick; the loop incl. background ends
before the IMU lead; the largest tick-to-tick increase of the frame-ready time (+ the TWI interrupts) stays inside the
FCU's frame-timeout margin (NLINK_FRAME_TIMEOUT_US - tick), so NAV jitter alone never makes the FCU fall back on its own
timer; the flight build keeps >= 1024 B of never-touched stack; no SPI interrupt vector in the NAV (it is the master);
the flight build in simavr exchanges every tick (SS spacing inside tick +- the timeout margin) with an SS-low time within
5 % of the generator's NLINK_XFER_US.

v0.8 (VTOL build, nav_main.c "v0.8"): predict -> publish -> correct -> altitude -> GPS.  Sections (VTOL): sensors | gyro
prediction + nl_nav3_tilt + quantised fields + slot + nl_encode_nav3 (+ the wait to the push floor) | the exchange |
Mahony correction + pad-still + VTOL altitude filter | GPS (USART1 bytes -> UBX parser -> configurator / epochs -> the
filter's job of the tick + the slot latch) | FCU reply + LED | background.  The time-critical part is still tick ->
frame delivered; the tick's MANDATORY work (everything up to the FCU reply, every tick) is new: ``SW`` line (wk / wp / wf
window / pad / flight max from the loop's own tick start, ws / wn the flight sum / count -> mean, wl flight ticks ending
inside the next tick's IMU-lead window, wt flight ticks ending after the next tick, ls the largest tick-start lateness after
the Timer2 interrupt).  A synthetic u-blox receiver (the NEO-6M model of v0.8; nav_simbench.c: factory 9600 baud with NMEA, ACKs the boot
configuration, answers MON-VER, 5 Hz NAV-POSLLH / SOL / VELNED once configured) drives the real UBX path through a
byte-accurate Timer3 "wire" into the USART1 RX ring (hal/gps_uart.c; the flight build's USART1 RX interrupt is counted from
its disassembly and compared); lines ``SG`` (configurator / parser / assembler), ``SI`` (filter / UART), ``SH`` (the GPS
section's maximum by job k mod 5: all / flight), ``SJ`` (the synthetic receiver; the synthetic FCU's slot receiver: sets,
broken cycles, GPS flags seen / latest, first CFG_OK / FIX_OK / GATE_OK ticks).  The wire composition replaces the bypass
exchange by nav_cosim's real one in every figure that contains it (crit, frame ready, mandatory work).

v0.8.1 (the four-chip set, nav_main.c "v0.8.1"): the GPS side moved to the GNS board (tools/run_gns_simbench.py); USART1 is
the glink.  Sections (VTOL): sensors | prediction + nl_nav3_tilt + quantised fields + the capped pre-build glink decode +
the GNS slot + nl_encode_nav4 (+ the push-floor wait) | the exchange | the glink decode after the push + Mahony correction
+ pad-still + VTOL altitude filter | glink tick (force sum, the STATE frame at k mod 5 == 4) + glink decode | FCU reply
(nlink v4) + LED | background.  The bench's synthetic FCU decodes nlink v4, feeds the GNS slot into the FCU's own slot
receiver and checks every TILT part rides in the frame of NAV tick 5 m + 4 + L; its replies are nlink v4 FCU frames.
nav_simbench's synthetic GNS answers every STATE with a RESULT NAV_SB_GNS_CYCLE_US after the STATE's last byte (the GNS
bench's measured worst cycle: checked against host_test/build/gns_simbench_report.json) on a byte-accurate Timer3 wire
into the USART1 RX ring, and injects four faulted cycles (no RESULT, one tick too late, a broken CRC, another SEQ) that
must be missed - nothing else.  New lines: ``SG`` (the NAV's glink side: STATE frames sent, RESULT frames taken, slot
latches / misses, decoder and UART counters), ``SH`` (glink timing: the STATE's write after the tick's Timer2 stamp, the
frame build after it, the pre-build decode, the glink section, the RESULT completion after its sample tick and its margin
to the frame build of 5 m + 4 + L), ``SJ`` (the synthetic GNS and the synthetic FCU's slot receiver / TILT timing).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys

FW = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
MODE_ENVS = {"vtol": ("nav", "nav_simbench", "nav_cosim"),
             "ballistic": ("nav_ballistic", "nav_simbench_ballistic", "nav_cosim_ballistic")}
NAV_F_EST_GATE = 0x80                     # src/nav/nav.h (checked against the header in run_mode)
RAM_BYTES = 4096                          # ATmega128 SRAM
STACK_MIN_FREE = 1024                     # never-touched stack the flight build must keep (as the FCU / COM)
CRIT_LIMIT = 0.80                         # time-critical part <= 80 % of the tick (the project's limit)
EXPECTED_WARNINGS = ("GAINS_PARAMS_STALE", "tuned against a different params.json")
STATES = ["SAFE", "ARMED", "POWERED", "UNPOWERED", "RELEASED", "ABORT"]
SECTION_NAMES = ["sensors (IMU convert, failure count, barometer step)", "estimator + tilt + pad-still + altitude filter",
                 "frame build + nl_encode_nav", "nlink_spi_exchange", "FCU reply decode + LED",
                 "background (barometer math, stack scan)"]
# v0.8.1 VTOL (nav_main.c): predict -> publish -> correct -> altitude -> glink (v0.8: GPS)
SECTION_NAMES_V08 = ["sensors (IMU convert, failure count, barometer step)",
                     "gyro prediction + nl_nav3_tilt + quantised fields + pre-build glink decode (<= 8 B) + GNS slot + "
                     "nl_encode_nav4 (+ push-floor wait)",
                     "nlink_spi_exchange",
                     "glink decode after the push + Mahony correction + pad-still + VTOL altitude filter (corrected q)",
                     "glink: force sum, the STATE frame at k mod 5 == 4 (USART1 TX ring), glink decode",
                     "FCU reply decode (nlink v4) + LED", "background (glink decode, barometer math, stack scan)"]
EXCHANGE_SECTION = {"vtol": 2, "ballistic": 3}            # index of nlink_spi_exchange in the section list
# v0.8.1 glink lines (nav_simbench.h)
SW_ALIAS = {"wk": "work_max_us", "wp": "work_pad_max_us", "wf": "work_flight_max_us", "ce": "compute_end_max_us",
            "lp": "late_pushes", "ov": "overruns", "sf": "stack_free", "ws": "work_flight_sum_us", "wn": "work_flight_n",
            "wl": "work_into_imu_lead", "wt": "work_past_tick", "ls": "late_start_max_us"}
SG_ALIAS = {"ss": "states_sent", "rt": "results_taken", "la": "slot_latched", "ms": "slot_missed", "rs": "results_ring",
            "df": "dec_frames", "dc": "dec_bad_crc", "dt": "dec_bad_type", "dw": "dec_wrong_dir", "ds": "dec_skipped",
            "ux": "uart_rx_overflow", "ue": "uart_rx_errors", "tr": "uart_tx_refused", "ub": "uart_rx_bytes",
            "tb": "uart_tx_bytes"}
SI_ALIAS: dict = {}
SH_ALIAS = {"sa": "state_write_max_us", "sb": "state_write_min_us", "pb": "frame_build_max_us", "pn": "prebuild_bytes_max",
            "pt": "prebuild_decode_max_us", "gb": "glink_section_max_us", "ra": "result_after_sample_max_us",
            "rm": "result_after_sample_min_us", "mn": "margin_to_build_min_us", "mx": "margin_to_build_max_us",
            "gc": "gns_cycle_us_input", "gl": "latency_ticks"}
SJ_ALIAS = {"gs": "gns_states", "gx": "gns_states_bad", "gr": "gns_results_sent", "gk": "gns_faults", "xs": "slot_sets",
            "xb": "slot_broken", "xt": "slot_tilts", "ti": "tilt_parts", "tk": "tilt_parts_on_time",
            "tb": "tilt_parts_wrong_tick", "tm": "tilt_cycle_gaps"}
SIGNED_SH = {"mn", "mx"}
SB_ALIAS = {"st": "fcu_state", "pd": "pad", "cm": "crit_max_us", "cp": "crit_pad_max_us", "cf": "crit_flight_max_us",
            "fr": "ready_max_us", "fp": "ready_pad_max_us", "ff": "ready_flight_max_us", "fn": "ready_min_us",
            "jx": "ready_jump_max_us", "tm": "total_max_us", "ov": "overruns", "sf": "stack_free", "bg": "bg_step_max_us",
            "ox": "simbench_overhead_max_us", "ce": "compute_end_max_us", "lp": "late_pushes"}
SN_ALIAS = {"st": "fcu_state", "fv": "fcu_valid", "fb": "fcu_bad", "ex": "exchanges", "lm": "frame_loop_us_max",
            "rc": "frame_reset_cause", "rd": "reports_dropped", "rs": "w1_trips"}
SF_ALIAS = {"nv": "nav_frames_valid", "nx": "nav_frames_refused", "sg": "seq_gaps", "pm": "mode_mismatches",
            "pc": "mode_frames_checked", "ef": "flying_frames_without_imu_ok_est_valid", "bc": "boot_frames",
            "tg": "tag_mismatch_frames", "cr": "replies_corrupted", "br": "replies_refusable", "fb": "nav_fcu_bad",
            "bl": "baro_late_frames", "al": "tick_first_aligned", "ev": "tick_first_est_valid", "ar": "tick_armed",
            "pw": "tick_powered", "ss": "fcu_state", "eg": "est_gate_flight_frames", "ep": "est_gate_pad_frames"}
BOOT_ALIAS = {"rc": "reset_cause", "sr": "stack_reserve", "xb": "xfer_bytes", "gc": "byte_gap_cycles", "sh": "spi_khz",
              "bd": "cosim_build"}
SIGNED16 = {"jx"}

# ---- interrupt model (as tools/run_simbench.py) ----
ISR_ENTRY_CYCLES = 7                      # 4 interrupt response + 3 vector-table JMP (ATmega128, 16-bit PC)
VECTORS = {"tick_timer2": 9, "imu_lead_timer0": 15, "spi_stc": 17, "console_udre0": 19, "twi": 33}
# TWI interrupts of one bus script = per transfer: START + SLA + every data byte (the STOP raises none).  Per barometer
# driver (src/hal/baro.h, params_gen.h FC_BARO_SENSOR): the scripts of one FC_BARO_DIV cycle, each in its own tick.
BARO_TWI_SCRIPTS = {
    # hal/ms5611.c: write 1 B (convert), read 3 B (ADC), write 1 B (convert) - the same script in both phases
    "MS5611": {"phase 0 / MS5611_D1_PHASE": (1 + 1 + 1) + (1 + 1 + 3) + (1 + 1 + 1)},
    # hal/bmp280.c: script A write 2 B (ctrl_meas, forced); script B write 1 B (register), read 6 B (0xF7 .. 0xFC)
    "BMP280": {"BMP280_START_PHASE (forced write)": 1 + 1 + 2, "BMP280_READ_PHASE (data read)": (1 + 1 + 1) + (1 + 1 + 6)},
}


def baro_sensor(fw: str | None = None) -> str:
    """the barometer driver the default NAV envs build: params_gen.h FC_BARO_SENSOR (FC_BARO_BMP280 / FC_BARO_MS5611)"""
    with open(os.path.join(fw or FW, "src", "params_gen.h"), "r", encoding="utf-8") as f:
        m = re.search(r"^#define\s+FC_BARO_SENSOR\s+FC_BARO_(\w+)", f.read(), re.M)
    if not m or m.group(1) not in BARO_TWI_SCRIPTS:
        raise SystemExit("src/params_gen.h: #define FC_BARO_SENSOR FC_BARO_<BMP280 | MS5611> not found")
    return m.group(1)


def twi_script_interrupts(fw: str | None = None) -> int:
    """the TWI interrupts of the longest barometer script of one tick (the figure added to a tick's measured time)"""
    return max(BARO_TWI_SCRIPTS[baro_sensor(fw)].values())


TWI_SCRIPT_INTERRUPTS = max(BARO_TWI_SCRIPTS["MS5611"].values())   # v0.9 name; the build's value: twi_script_interrupts()
ICALL_TARGETS = {"__vector_15": ["mpu6050_start_async"]}         # timer_set_lead_hook in nav_main.c (flight build)
CYCLES = {**{m: 1 for m in ("add adc sub subi sbc sbci and andi or ori eor com neg sbr cbr inc dec tst clr ser mov movw "
                             "ldi in out lsl lsr rol ror asr swap bst bld sec clc sen cln sez clz sei cli ses cls sev clv "
                             "set clt seh clh nop cp cpc cpi wdr sleep").split()},
          **{m: 2 for m in "adiw sbiw mul muls mulsu fmul fmuls fmulsu push pop lds sts ld st ldd std sbi cbi".split()},
          "lpm": 3, "elpm": 3}
SKIPS = {"cpse", "sbrc", "sbrs", "sbic", "sbis"}


def tool_exe(package: str, name: str) -> str:
    home = os.environ.get("PLATFORMIO_CORE_DIR") or os.path.join(os.path.expanduser("~"), ".platformio")
    exe = os.path.join(home, "packages", package, "bin", name + (".exe" if os.name == "nt" else ""))
    if not os.path.exists(exe):
        raise SystemExit(f"{name} not found at {exe} (python -m platformio pkg install -g --tool platformio/{package})")
    return exe


def build(env: str, extra_flags: str | None = None, fw: str | None = None) -> dict:
    """clean + build: PlatformIO prints a file's warnings only when it compiles it, so every NAV file is compiled here.
    extra_flags: PLATFORMIO_BUILD_FLAGS (the profile identity check - PlatformIO wipes a project's whole .pio/build when
    they change, so that check builds in a copy of the project, `fw`)"""
    fw = fw or FW
    envv = dict(os.environ)
    if extra_flags:
        envv["PLATFORMIO_BUILD_FLAGS"] = extra_flags
    else:
        envv.pop("PLATFORMIO_BUILD_FLAGS", None)
    subprocess.run([sys.executable, "-m", "platformio", "run", "-d", fw, "-e", env, "-t", "clean"], capture_output=True,
                   text=True, encoding="utf-8", errors="replace", env=envv)
    r = subprocess.run([sys.executable, "-m", "platformio", "run", "-d", fw, "-e", env], capture_output=True, text=True,
                       encoding="utf-8", errors="replace", env=envv)
    if r.returncode != 0:
        raise SystemExit(f"build {env} failed:\n{r.stdout[-3000:]}\n{r.stderr[-2000:]}")
    ram = re.search(r"RAM:.*used (\d+) bytes", r.stdout)
    flash = re.search(r"Flash:.*used (\d+) bytes", r.stdout)
    warnings = sorted({w.strip() for w in re.findall(r"warning: ([^\n]*)", r.stdout + r.stderr)})
    hex_path = os.path.join(fw, ".pio", "build", env, "firmware.hex")
    sha = None
    if os.path.exists(hex_path):
        import hashlib
        with open(hex_path, "rb") as f:
            sha = hashlib.sha256(f.read()).hexdigest()
    return {"elf": os.path.join(fw, ".pio", "build", env, "firmware.elf"), "hex_sha256": sha,
            "static_ram_bytes": int(ram.group(1)) if ram else None, "flash_bytes": int(flash.group(1)) if flash else None,
            "warnings": warnings,
            "unexpected_warnings": [w for w in warnings if not any(e in w for e in EXPECTED_WARNINGS)]}


SS_TRACE = ["-at", "ss=trace@0x38/0x01"]   # CPU writes to PORTB (data address 0x38), bit 0 = the nlink SS (PB0)


def _vcd_args(vcd: str | None) -> tuple[list[str], str | None]:
    """simavr -o keeps the file name in a short fixed buffer (a ~150-character path silently wrote nothing, measured):
    run simavr in the VCD's directory and pass the bare file name"""
    if not vcd:
        return [], None
    if os.path.exists(vcd):
        os.remove(vcd)
    return ["-o", os.path.basename(vcd), *SS_TRACE], os.path.dirname(os.path.abspath(vcd))


def run(elf: str, timeout_s: float = 300.0, vcd: str | None = None) -> str:
    """simavr on the ELF; with `vcd`, the SS pin's writes go to that VCD file (simavr -o / --add-trace)"""
    extra, cwd = _vcd_args(vcd)
    r = subprocess.run([tool_exe("tool-simavr", "simavr"), "-m", "atmega128", "-f", "16000000", *extra,
                        os.path.abspath(elf)], capture_output=True, timeout=timeout_s, stdin=subprocess.DEVNULL, cwd=cwd)
    return (r.stdout + r.stderr).decode("latin-1")


def run_for(elf: str, wall_s: float, vcd: str) -> None:
    """simavr on a firmware that never stops by itself (the flight build): killed after wall_s seconds of host time;
    simavr writes its VCD as it runs, so what was traced until then is in the file"""
    extra, cwd = _vcd_args(vcd)
    try:
        subprocess.run([tool_exe("tool-simavr", "simavr"), "-m", "atmega128", "-f", "16000000", *extra,
                        os.path.abspath(elf)], capture_output=True, timeout=wall_s, stdin=subprocess.DEVNULL, cwd=cwd)
    except subprocess.TimeoutExpired:
        pass


def ss_edges(vcd: str) -> list[tuple[float, int]]:
    """(time in us, level) of every SS write in the VCD (one traced signal, id '!')"""
    out, t = [], 0.0
    scale = 1e-3                                              # $timescale 1ns -> us
    with open(vcd, "r", encoding="latin-1") as f:
        for line in f:
            line = line.strip()
            if line.startswith("$timescale"):
                m = re.search(r"(\d+)\s*(ns|us|ps)", line)
                if m:
                    scale = int(m.group(1)) * {"ps": 1e-6, "ns": 1e-3, "us": 1.0}[m.group(2)]
            elif line.startswith("#"):
                t = int(line[1:]) * scale
            elif line in ("0!", "1!"):
                out.append((t, int(line[0])))
    return out


def ss_figures(edges: list[tuple[float, int]], skip: int = 2) -> dict:
    """exchanges = SS low pulses; width = SS low (the exchange on the wire); spacing = SS rise to SS rise (the FCU's tick
    spacing: its tick starts on the frame's last byte).  The first `skip` pulses (boot) are left out of the spacing."""
    lows, rises = [], []
    t_fall = None
    for t, v in edges:
        if v == 0:
            t_fall = t
        elif t_fall is not None:
            lows.append(t - t_fall)
            rises.append(t)
            t_fall = None
    sp = [b - a for a, b in zip(rises[skip:], rises[skip + 1:])]
    return {"exchanges": len(lows), "simulated_s": round(rises[-1] * 1e-6, 3) if rises else 0.0,
            "ss_low_min_us": round(min(lows), 2) if lows else None, "ss_low_max_us": round(max(lows), 2) if lows else None,
            "frame_spacing_min_us": round(min(sp), 2) if sp else None,
            "frame_spacing_max_us": round(max(sp), 2) if sp else None}


def header_defines(path: str, names: tuple[str, ...]) -> dict[str, float]:
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()
    out = {}
    for n in names:
        m = re.search(rf"#define\s+{n}\s+\(?(-?0[xX][0-9A-Fa-f]+|-?[0-9.]+)[uUlLf]*", src)
        if not m:
            raise SystemExit(f"{os.path.relpath(path, FW)}: #define {n} not found")
        out[n] = float(m.group(1)) if "." in m.group(1) else int(m.group(1), 0)
    return out


def _kv(line: str, alias: dict[str, str]) -> dict:
    row = {}
    for k, v in re.findall(r"(?<![0-9a-z])([a-z]+)=(0x[0-9a-f]+)", line):
        if k in ("s", "j", "jf"):
            continue
        x = int(v, 16)
        if k in SIGNED16 and x >= 0x8000:
            x -= 0x10000
        row[alias.get(k, k)] = x
    return row


def parse(text: str) -> dict:
    sb, sn, sf = [], [], []
    extra = {}
    for tag, alias in (("SW", SW_ALIAS), ("SG", SG_ALIAS), ("SJ", SJ_ALIAS)):
        extra[tag.lower()] = [_kv(m.group(1), alias) for m in re.finditer(rf"\n{tag} (t=[ -~]*)", text)]
    sh = []
    for m in re.finditer(r"\nSH (t=[ -~]*)", text):
        row = _kv(m.group(1), SH_ALIAS)
        for k in ("margin_to_build_min_us", "margin_to_build_max_us"):
            if k in row and row[k] >= 0x8000:
                row[k] -= 0x10000
        sh.append(row)
    extra["sh"] = sh
    for m in re.finditer(r"\nSB (t=[ -~]*)", text):
        line = m.group(1)
        row = _kv(line, SB_ALIAS)
        secs = re.search(r"(?<![a-z0-9])s=((?:0x[0-9a-f]+ ?)+)", line)
        row["sections_us"] = [int(x, 16) for x in secs.group(1).split()] if secs else []
        sb.append(row)
    for m in re.finditer(r"\nSN (t=[ -~]*)", text):
        sn.append(_kv(m.group(1), SN_ALIAS))
    for m in re.finditer(r"\nSF (t=[ -~]*)", text):
        sf.append(_kv(m.group(1), SF_ALIAS))
    boot = re.search(r"SB BOOT ([ -~]*)", text)
    return {"sb": sb, "sn": sn, "sf": sf, "boot": _kv(boot.group(1), BOOT_ALIAS) if boot else {}, "end": "SB END" in text,
            **extra}


# ---------------------------------------------------------------- disassembly: ISR cycles and stack frames
def disassemble(elf: str) -> dict[str, list[tuple[int, int, str, str]]]:
    """function name -> [(address, size in bytes, mnemonic, operands incl. objdump's target comment)]"""
    r = subprocess.run([tool_exe("toolchain-atmelavr", "avr-objdump"), "-d", elf], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    funcs: dict[str, list[tuple[int, int, str, str]]] = {}
    cur = None
    for line in r.stdout.splitlines():
        m = re.match(r"^[0-9a-f]+ <(\S+)>:$", line)
        if m:
            cur = funcs.setdefault(m.group(1), [])
            continue
        m = re.match(r"^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} )+)\s*\t(\S+)\s*(.*)$", line)
        if m and cur is not None:
            cur.append((int(m.group(1), 16), len(m.group(2).split()), m.group(3), m.group(4)))
    return funcs


def _target(ops: str) -> int:
    m = re.search(r";\s*0x([0-9a-f]+)", ops)
    if not m:
        raise ValueError(f"no branch target in {ops!r}")
    return int(m.group(1), 16)


def longest_cycles(funcs: dict, name: str, stack: frozenset = frozenset()) -> int:
    """cycles of the longest path from the entry of `name` to its RET / RETI (4 each); a call adds call + the callee's
    longest path (call 4, rcall / icall 3 cycles on the 16-bit-PC ATmega128; icall targets from ICALL_TARGETS); raises
    on a loop, an indirect jump or an unknown instruction"""
    if name in stack:
        raise ValueError(f"recursion through {name}")
    ins = funcs[name]
    idx = {a: i for i, (a, _, _, _) in enumerate(ins)}
    memo: dict[int, int] = {}

    def step(i: int, active: frozenset) -> int:
        if i in memo:
            return memo[i]
        if i in active:
            raise ValueError(f"loop in {name}")
        a, size, mn, ops = ins[i]
        act = active | {i}
        if mn in ("reti", "ret"):
            c = 4
        elif mn == "rjmp":
            c = 2 + step(idx[_target(ops)], act)
        elif mn.startswith("br"):
            c = max(1 + step(i + 1, act), 2 + step(idx[_target(ops)], act))
        elif mn in SKIPS:
            c = max(1 + step(i + 1, act), (2 if ins[i + 1][1] == 2 else 3) + step(i + 2, act))
        elif mn in ("call", "rcall") and not ops.startswith(".+0"):
            callee = _callee(funcs, ops)
            if callee is None:
                raise ValueError(f"unknown call target in {name}: {ops}")
            c = (4 if mn == "call" else 3) + longest_cycles(funcs, callee, stack | {name}) + step(i + 1, act)
        elif mn == "icall":
            targets = ICALL_TARGETS.get(name)
            if not targets:
                raise ValueError(f"icall in {name} without a known target")
            c = 3 + max(longest_cycles(funcs, t, stack | {name}) for t in targets) + step(i + 1, act)
        elif mn == "rcall":                                   # rcall .+0: pushes 2 bytes of frame, 3 cycles
            c = 3 + step(i + 1, act)
        elif mn in CYCLES:
            c = CYCLES[mn] + step(i + 1, act)
        else:
            raise ValueError(f"unsupported instruction in {name}: {mn} {ops}")
        memo[i] = c
        return c

    return step(0, frozenset())


def isr_cycles(funcs: dict, name: str) -> dict:
    """{'longest': ISR_ENTRY_CYCLES + the longest path to RETI (calls followed)}"""
    return {"longest": ISR_ENTRY_CYCLES + longest_cycles(funcs, name), "instructions": len(funcs[name])}


def _callee(funcs: dict, ops: str) -> str | None:
    m = re.search(r"<([^>+]+)(?:\+0x[0-9a-f]+)?>", ops)
    return m.group(1) if m and m.group(1) in funcs else None


FRAME_RE = re.compile(r"r28,\s*0x([0-9a-fA-F]+)")


def stack_bytes(funcs: dict, name: str, active: frozenset = frozenset()) -> int:
    """bytes a call of `name` puts on the stack beyond its return address: pushes + frame (in r28,SPL / sbiw|subi r28,N
    or rcall .+0) + the deepest callee (2-byte return address each; icall targets from ICALL_TARGETS)"""
    if name in active:
        raise ValueError(f"recursion through {name}")
    ins = funcs[name]
    own = sum(1 for _, _, mn, _ in ins if mn == "push")
    deeper = 0
    for k, (_, _, mn, ops) in enumerate(ins):
        if mn == "rcall" and ops.startswith(".+0"):
            own += 2                                          # rcall .+0: 2 bytes of frame
        elif mn in ("sbiw", "subi") and FRAME_RE.match(ops) and k > 0 and \
                any(i2[2] == "in" and i2[3].startswith("r28, 0x3d") for i2 in ins[max(0, k - 4):k]):
            own += int(FRAME_RE.match(ops).group(1), 16)
        elif mn in ("call", "rcall"):
            c = _callee(funcs, ops)
            if c and c != name:
                deeper = max(deeper, 2 + stack_bytes(funcs, c, active | {name}))
        elif mn in ("icall", "eicall"):
            for c in ICALL_TARGETS.get(name, []):
                deeper = max(deeper, 2 + stack_bytes(funcs, c, active | {name}))
    return own + deeper


def isr_table(elf: str) -> dict:
    funcs = disassemble(elf)
    out = {}
    for name, vec in VECTORS.items():
        fn = f"__vector_{vec}"
        ins = funcs.get(fn)
        if not ins:
            out[name] = {"vector": vec, "present": False}
            continue
        row: dict = {"vector": vec, "present": True}
        try:
            row.update(isr_cycles(funcs, fn))
        except (ValueError, KeyError) as e:
            row["cycles_error"] = str(e)
        try:
            row["stack_bytes"] = 2 + stack_bytes(funcs, fn)       # + the PC the interrupt pushes
        except (ValueError, KeyError) as e:
            row["stack_error"] = str(e)
        out[name] = row
    return out


# ---------------------------------------------------------------- report
def agg(rows: list[dict], key: str, fn=max):
    vals = [r[key] for r in rows if key in r]
    return fn(vals) if vals else None


def section_max(rows: list[dict], i: int) -> int | None:
    vals = [r["sections_us"][i] for r in rows if len(r.get("sections_us", [])) > i]
    return max(vals) if vals else None


def main() -> int:
    global FW
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--fw", default=FW, help="firmware project directory (default: this script's ../)")
    ap.add_argument("--out", default=None, help="report path for a single --mode (default: <fw>/host_test/build/"
                                                "nav_simbench_report_<mode>.json, + nav_simbench_report.json for "
                                                "params.json's mode)")
    ap.add_argument("--mode", choices=("both", "vtol", "ballistic"), default="both",
                    help="vtol: nav / nav_simbench / nav_cosim; ballistic: the *_ballistic envs; both (default)")
    ap.add_argument("--flight-wall-s", type=float, default=6.0,
                    help="host seconds the flight build runs in simavr (it never stops by itself)")
    args = ap.parse_args()
    FW = os.path.abspath(args.fw)
    try:
        with open(os.path.join(FW, "..", "..", "params.json"), "r", encoding="utf-8") as f:
            params_mode = str(json.load(f).get("mission", {}).get("mode", "ballistic"))
    except (OSError, ValueError):
        params_mode = "ballistic"
    modes = ["vtol", "ballistic"] if args.mode == "both" else [args.mode]
    ok = True
    for mode in modes:
        if args.out and len(modes) == 1:
            paths = [args.out]
        else:
            paths = [os.path.join(FW, "host_test", "build", f"nav_simbench_report_{mode}.json")]
            if mode == params_mode:
                paths.append(os.path.join(FW, "host_test", "build", "nav_simbench_report.json"))
        ok = run_mode(mode, args, paths) and ok
    return 0 if ok else 1


def profile_identity(env: str, plain_sha: str | None) -> dict:
    """The NAV image must not depend on the build profile (params_gen.h MISSION_PROFILE: field / demo), so one nav image
    serves both - a demo vehicle is fcu_demo + nav + com.  Built in a temporary copy of the project (src/,
    platformio.ini, tools/pio_mission_mode.py) without and with -D MISSION_PROFILE_DEMO and compared; the copy's plain
    image is also compared with this project's."""
    import shutil
    import tempfile
    tmp = tempfile.mkdtemp(prefix="nav_profile_")
    try:
        shutil.copytree(os.path.join(FW, "src"), os.path.join(tmp, "src"))
        shutil.copy2(os.path.join(FW, "platformio.ini"), tmp)
        os.makedirs(os.path.join(tmp, "tools"))
        shutil.copy2(os.path.join(FW, "tools", "pio_mission_mode.py"), os.path.join(tmp, "tools"))
        field = build(env, None, tmp)["hex_sha256"]
        demo = build(env, "-D MISSION_PROFILE_DEMO", tmp)["hex_sha256"]
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return {"field_hex_sha256": field, "demo_hex_sha256": demo, "project_hex_sha256": plain_sha,
            "identical": field is not None and field == demo, "copy_equals_project": field == plain_sha}


def run_mode(mode: str, args, out_paths: list[str]) -> bool:
    FLIGHT_ENV, BENCH_ENV, COSIM_ENV = MODE_ENVS[mode]
    vtol = mode == "vtol"
    out_path = out_paths[0]
    print(f"==== NAV, mode {mode}: {FLIGHT_ENV} / {BENCH_ENV} / {COSIM_ENV}")
    k = header_defines(os.path.join(FW, "src", "params_gen.h"),
                       ("FC_TICK_US", "FC_LOOP_HZ", "FC_BARO_DIV", "FC_IMU_LEAD_COUNTS", "FC_T2_PRESCALER", "FC_F_CPU_HZ",
                        "NAV_TICK_HZ", "NLINK_SPI_HZ", "NLINK_BYTE_GAP_CYCLES", "NLINK_XFER_US", "NLINK_FRAME_TIMEOUT_US",
                        "ONB_IMU_FAIL_TICKS"))
    nsb = header_defines(os.path.join(FW, "src", "nav", "nav_simbench.h"),
                         ("NAV_SB_ARM_S", "NAV_SB_LAUNCH_S", "NAV_SB_CORRUPT_S", "NAV_SB_CORRUPT_EVERY", "NAV_SB_TAG_S"))
    tick_us = float(k["FC_TICK_US"])
    lead_us = k["FC_IMU_LEAD_COUNTS"] * k["FC_T2_PRESCALER"] * 1e6 / k["FC_F_CPU_HZ"]
    cyc_us = 1e6 / k["FC_F_CPU_HZ"]

    builds = {env: build(env) for env in (FLIGHT_ENV, BENCH_ENV, COSIM_ENV)}
    profile_check = None
    if vtol:
        profile_check = profile_identity(FLIGHT_ENV, builds[FLIGHT_ENV]["hex_sha256"])
        nav_h = header_defines(os.path.join(FW, "src", "nav", "nav.h"), ("NAV_F_EST_GATE", "NAV_PUSH_MIN_US_VTOL"))
        sbh = header_defines(os.path.join(FW, "src", "simbench.h"), ("SB_NAV_PUSH_US_VTOL", "SB_NAV_JITTER_US"))

    # ---- single-chip bench
    bench_text = run(builds[BENCH_ENV]["elf"])
    b = parse(bench_text)
    if not b["sb"] or not b["end"]:
        print(bench_text[-2000:])
        raise SystemExit(f"{BENCH_ENV}: no report / no SB END (did the firmware reach its loop?)")
    sb, sf = b["sb"], b["sf"]
    last_sb, last_sf, last_sn = sb[-1], (sf[-1] if sf else {}), (b["sn"][-1] if b["sn"] else {})
    flight = [r for r in sb if r.get("fcu_state") in (2, 3)]
    section_names = SECTION_NAMES_V08 if vtol else SECTION_NAMES
    ex_i = EXCHANGE_SECTION[mode]
    n_sec = len(section_names)
    bench = {
        "env": BENCH_ENV, "windows": len(sb), "boot": b["boot"],
        "states": [STATES[r["fcu_state"]] if r.get("fcu_state", 99) < 6 else r.get("fcu_state") for r in sb],
        "crit_pad_max_us": last_sb.get("crit_pad_max_us"), "crit_flight_max_us": last_sb.get("crit_flight_max_us"),
        "ready_pad_max_us": last_sb.get("ready_pad_max_us"), "ready_flight_max_us": last_sb.get("ready_flight_max_us"),
        "ready_min_us": last_sb.get("ready_min_us"), "ready_jump_max_us": last_sb.get("ready_jump_max_us"),
        "total_max_us": agg(sb, "total_max_us"), "flight_total_max_us": agg(flight, "total_max_us"),
        "overruns": last_sb.get("overruns"), "stack_free_min_bytes": agg(sb, "stack_free", min),
        "bg_step_max_us": agg(sb, "bg_step_max_us"), "simbench_overhead_max_us": agg(sb, "simbench_overhead_max_us"),
        "compute_end_max_us": last_sb.get("compute_end_max_us"), "late_pushes": last_sb.get("late_pushes"),
        "sections_max_us": dict(zip(section_names, [section_max(sb, i) for i in range(n_sec)])),
        "flight_sections_max_us": dict(zip(section_names, [section_max(flight, i) for i in range(n_sec)])),
        "nav_nlink": {k2: last_sn.get(k2) for k2 in ("fcu_valid", "fcu_bad", "exchanges", "frame_loop_us_max",
                                                     "frame_reset_cause", "reports_dropped")},
        "synthetic_fcu": last_sf,
        "_doc": "exchange section = nlink_spi's single-chip bypass (a copy), NOT the wire: see cosim_standalone",
    }
    stack_used_bench = RAM_BYTES - builds[BENCH_ENV]["static_ram_bytes"] - bench["stack_free_min_bytes"]

    # ---- nav_cosim alone: the real SPI master (+ the SS pin traced)
    vcd_dir = os.path.join(os.path.dirname(out_path), "nav_vcd")
    os.makedirs(vcd_dir, exist_ok=True)
    cosim_vcd = os.path.join(vcd_dir, "nav_cosim_ss.vcd")
    cosim_text = run(builds[COSIM_ENV]["elf"], vcd=cosim_vcd)
    c = parse(cosim_text)
    if not c["sb"] or not c["end"]:
        print(cosim_text[-2000:])
        raise SystemExit(f"{COSIM_ENV}: no report / no SB END")
    csb, csn = c["sb"], c["sn"]
    cl, cn = csb[-1], (csn[-1] if csn else {})
    cosim = {
        "env": COSIM_ENV, "windows": len(csb), "boot": c["boot"],
        "exchange_max_us": section_max(csb, ex_i),
        "exchange_min_window_max_us": min((r["sections_us"][ex_i] for r in csb if len(r["sections_us"]) > ex_i),
                                          default=None),
        "crit_pad_max_us": cl.get("crit_pad_max_us"), "ready_pad_max_us": cl.get("ready_pad_max_us"),
        "ready_min_us": cl.get("ready_min_us"), "ready_jump_max_us": cl.get("ready_jump_max_us"),
        "total_max_us": agg(csb, "total_max_us"), "overruns": cl.get("overruns"),
        "stack_free_min_bytes": agg(csb, "stack_free", min),
        "fcu_valid": cn.get("fcu_valid"), "fcu_bad": cn.get("fcu_bad"), "exchanges": cn.get("exchanges"),
        "sections_max_us": dict(zip(section_names, [section_max(csb, i) for i in range(n_sec)])),
        "_doc": "run alone: MISO reads 0x00, every FCU frame is refused (fcu_bad == exchanges), the NAV stays in pad "
                "mode; the exchange section is the real SPI master on the wire (+ the console's UDRE interrupts)",
    }
    stack_used_cosim = RAM_BYTES - builds[COSIM_ENV]["static_ram_bytes"] - cosim["stack_free_min_bytes"]
    cosim["ss_pin"] = ss_figures(ss_edges(cosim_vcd))

    # ---- the FLIGHT build in simavr: no I2C slave (both sensors fail their boot init, the IMU read fails every tick),
    # no console; only its SS pin is observed - it must boot, tick and exchange once per tick without a watchdog reset
    flight_vcd = os.path.join(vcd_dir, "nav_flight_ss.vcd")
    run_for(builds[FLIGHT_ENV]["elf"], args.flight_wall_s, flight_vcd)
    flight_smoke = ss_figures(ss_edges(flight_vcd))
    flight_smoke["_doc"] = ("flight ELF without sensors (MPU-6050 / barometer init fail, IMU read fails every tick: reset_cause "
                            "0xC0 | MCUCSR, imu_fail climbing) for ~flight_wall_s of host time: SS low = the exchange on the "
                            "wire, SS rise spacing = what the FCU's frame-triggered tick sees (no gap > the frame timeout: no "
                            "watchdog reset, no missed exchange)")

    # ---- interrupts and stack from the flight ELF
    isr = isr_table(builds[FLIGHT_ENV]["elf"])
    twi_cyc = isr["twi"].get("longest")
    baro = baro_sensor()
    twi_n = twi_script_interrupts()
    twi_us = twi_n * twi_cyc * cyc_us if twi_cyc else None
    isr_stack_unexercised = max((isr[n].get("stack_bytes", 0) for n in ("twi", "imu_lead_timer0")), default=0)
    stack_used = max(stack_used_bench, stack_used_cosim)
    flight_static = builds[FLIGHT_ENV]["static_ram_bytes"]
    flight_stack_free = RAM_BYTES - flight_static - stack_used - isr_stack_unexercised

    # ---- composed flight timing on the wire
    bypass_ex = section_max(sb, ex_i)
    wire_ex = cosim["exchange_max_us"]
    # fw09/p1: interrupt-driven exchange - the bench clocks the real SPI (its figures contain the wire and the ISR)
    with open(os.path.join(FW, "src", "nav", "nav_main.c"), encoding="utf-8") as _f:
        async_ex = vtol and "nlink_spi_exchange_start" in _f.read()
    if async_ex:
        bypass_ex = wire_ex = 0
    crit_flight_wire = bench["crit_flight_max_us"] - bypass_ex + wire_ex
    ready_flight_wire = bench["ready_flight_max_us"] - bypass_ex + wire_ex
    crit_wire_isr = crit_flight_wire + (twi_us or 0.0)
    if vtol:
        # v0.8: the compute end measured from the loop's own tick start (the bench's tick ISR stamp also holds the late
        # start of a tick after an overrun): the flight time-critical part minus its bypass exchange; the barometer's bus
        # interrupts run during the compute (they follow the step-2 baro_step) and are absorbed by the wait unless the
        # compute + they pass the floor
        compute_t0 = bench["crit_flight_max_us"] - bypass_ex
        if async_ex:                                   # fw09/p1: crit (tick -> SS rise) minus the exchange on the wire
            compute_t0 = bench["crit_flight_max_us"] - flight_smoke["ss_low_max_us"]
        compute_isr = compute_t0 + (twi_us or 0.0)
        crit_wire_isr = max(crit_flight_wire, compute_isr + (flight_smoke["ss_low_max_us"] if async_ex else wire_ex))
    margin_timeout_us = k["NLINK_FRAME_TIMEOUT_US"] - tick_us
    work = None
    if vtol:
        # v0.8: the tick's mandatory work (prediction, publish, correction, altitude, GPS, reply) on the wire: the bench's
        # figure minus its bypass exchange + the real exchange; + the TWI script (in every tick that starts a barometer script); the
        # GPS RX interrupts are in the measured figure (the bench's Timer3 wire, see gps_uart below)
        sw = b["sw"]
        wl = sw[-1] if sw else {}
        n_fl = wl.get("work_flight_n") or 0
        mean_fl = (wl.get("work_flight_sum_us") or 0) / n_fl if n_fl else None
        work = {
            "flight_max_bench_us": wl.get("work_flight_max_us"), "pad_max_bench_us": wl.get("work_pad_max_us"),
            "flight_mean_bench_us": None if mean_fl is None else round(mean_fl, 1),
            "flight_max_wire_isr_us": None if wl.get("work_flight_max_us") is None else
            round(wl["work_flight_max_us"] - bypass_ex + wire_ex + (twi_us or 0.0), 1),
            "flight_mean_wire_us": None if mean_fl is None else round(mean_fl - bypass_ex + wire_ex, 1),
            "flight_ticks": n_fl, "ticks_into_imu_lead": wl.get("work_into_imu_lead"),
            "ticks_past_tick": wl.get("work_past_tick"), "late_start_max_us": wl.get("late_start_max_us"),
            "bench_stack_free_min": min((r.get("stack_free", 9999) for r in sw), default=None),
            "_doc": "mandatory work = the loop's own tick start -> the FCU reply handled (everything but the background), "
                    "measured with the bypass exchange; *_wire = that - the bypass exchange + nav_cosim's real one (+ the "
                    "barometer script's TWI interrupts for the max).  The bench's counts (into the IMU lead / past the tick) "
                    "are with the bypass exchange, i.e. ~0.6 ms optimistic; late_start = the largest lateness of a tick "
                    "start after its Timer2 interrupt (the previous tick still running, incl. the bench's own end-of-loop "
                    "work)",
        }
        work["fraction_of_tick_wire_isr"] = None if work["flight_max_wire_isr_us"] is None else round(
            work["flight_max_wire_isr_us"] / tick_us, 4)
        work["mean_fraction_of_tick_wire"] = None if work["flight_mean_wire_us"] is None else round(
            work["flight_mean_wire_us"] / tick_us, 4)
    jump = max(bench["ready_jump_max_us"], cosim["ready_jump_max_us"] or 0) + (twi_us or 0.0)
    bg_end_flight = bench["flight_total_max_us"] - bypass_ex + wire_ex
    timing = {
        "tick_us": tick_us,
        "exchange_wire_us": wire_ex, "exchange_bypass_us": bypass_ex, "nlink_xfer_us_generator": k["NLINK_XFER_US"],
        "crit_pad_wire_us": cosim["crit_pad_max_us"], "crit_flight_wire_us": crit_flight_wire,
        "ready_pad_wire_us": cosim["ready_pad_max_us"], "ready_flight_wire_us": ready_flight_wire,
        "baro_sensor": baro, "twi_scripts": BARO_TWI_SCRIPTS[baro],
        "twi_script_interrupts": twi_n, "twi_isr_cycles": twi_cyc,
        "twi_script_isr_us": None if twi_us is None else round(twi_us, 1),
        "crit_flight_wire_with_isr_us": round(crit_wire_isr, 1), "crit_fraction": round(crit_wire_isr / tick_us, 4),
        "ready_flight_wire_with_isr_us": round(max(ready_flight_wire, compute_isr + (flight_smoke["ss_low_max_us"] if async_ex
                                                                                      else wire_ex)) if vtol
                                               else ready_flight_wire + (twi_us or 0.0), 1),
        "imu_sample_to_frame_ready_us": round(lead_us + (max(ready_flight_wire, compute_isr + (flight_smoke["ss_low_max_us"]
                                                                                              if async_ex else wire_ex)) if vtol
                                                         else ready_flight_wire + (twi_us or 0.0)), 1),
        "ready_jump_max_us": round(jump, 1), "fcu_frame_timeout_margin_us": margin_timeout_us,
        "loop_incl_background_flight_wire_us": bg_end_flight, "imu_lead_at_us": round(tick_us - lead_us, 1),
        "_doc": "wire figures = bench flight figure - its bypass exchange section + nav_cosim's real exchange section; "
                "ready = from the Timer2 ISR's Timer1 stamp to SS high (the FCU's tick starts then); imu_sample_to_frame"
                "_ready = FC_IMU_LEAD_COUNTS lead (the burst read starts then) + ready; ready_jump = largest increase of the "
                "frame-ready time from one tick to the next (+ the longest barometer script's TWI interrupts): the FCU's tick spacing "
                "is the tick + that, and its own timer takes over after NLINK_FRAME_TIMEOUT_US",
    }
    stack = {
        "flight_static_ram_bytes": flight_static, "bench_static_ram_bytes": builds[BENCH_ENV]["static_ram_bytes"],
        "cosim_static_ram_bytes": builds[COSIM_ENV]["static_ram_bytes"],
        "stack_used_bench_bytes": stack_used_bench, "stack_used_cosim_bytes": stack_used_cosim,
        "isr_stack_not_exercised_bytes": isr_stack_unexercised, "flight_stack_free_bytes": flight_stack_free,
        "_doc": "used = RAM - static RAM - never-touched canary bytes of the simulated loop; the flight build keeps RAM - "
                "its static RAM - max(used) - the deepest interrupt frame simavr did not exercise (TWI, Timer0 lead -> "
                "mpu6050_start_async; counted from the disassembly)",
    }

    sfx = last_sf
    checks = {
        "builds_without_unexpected_warnings": all(not v["unexpected_warnings"] for v in builds.values()),
        "bench_end_reached": b["end"], "bench_no_missed_ticks": bench["overruns"] == 0,
        "bench_reached_powered": sfx.get("tick_powered", 0) > 0 and sfx.get("fcu_state") == 2,
        "bench_armed_after_aligned": 0 < sfx.get("tick_first_aligned", 0) <= sfx.get("tick_armed", 0)
                                     and sfx.get("tick_armed", 0) >= nsb["NAV_SB_ARM_S"] * k["FC_LOOP_HZ"],
        "est_valid_with_aligned": sfx.get("tick_first_est_valid") == sfx.get("tick_first_aligned"),
        "nav_frames_all_valid": sfx.get("nav_frames_refused") == 0 and sfx.get("nav_frames_valid", 0) > 0,
        "nav_seq_continuous": sfx.get("seq_gaps") == 0,
        "pad_flight_mode_follows_fcu_state": sfx.get("mode_mismatches") == 0 and sfx.get("mode_frames_checked", 0) > 0,
        "flying_frames_imu_ok_est_valid": sfx.get("flying_frames_without_imu_ok_est_valid") == 0,
        "boot_flag_exactly_two_frames": sfx.get("boot_frames") == 2,
        # the NAV refused exactly the replies it had to: the corrupted ones it got back + the first exchange's zeros (the
        # corruption set for the NEXT exchange may already be counted in replies_corrupted)
        "invalid_replies_refused": sfx.get("replies_corrupted", 0) > 0
                                   and sfx.get("nav_fcu_bad") == sfx.get("replies_refusable")
                                   and sfx.get("replies_corrupted") <= sfx.get("replies_refusable", 0) <= sfx.get("replies_corrupted") + 1,
        "tag_mismatch_reported_once": sfx.get("tag_mismatch_frames") == 1,
        "no_late_baro": sfx.get("baro_late_frames") == 0,
        "cosim_end_reached": c["end"], "cosim_no_missed_ticks": cosim["overruns"] == 0,
        "cosim_every_tick_one_exchange": cosim["exchanges"] is not None and cosim["exchanges"] == cosim["fcu_bad"],
        "crit_with_isr_within_limit": crit_wire_isr <= CRIT_LIMIT * tick_us,
        "loop_ends_before_imu_lead": bg_end_flight < tick_us - lead_us and (cosim["total_max_us"] or 0) < tick_us - lead_us,
        "frame_ready_jitter_inside_fcu_timeout_margin": jump < margin_timeout_us,
        "flight_stack_free_ok": flight_stack_free >= STACK_MIN_FREE,
        "no_spi_slave_isr_in_nav": not isr["spi_stc"]["present"],
        "twi_isr_counted": twi_cyc is not None,
        "cosim_ss_pin_one_pulse_per_exchange": (cosim["ss_pin"]["exchanges"] or 0) >= (cosim["exchanges"] or 1)
                                               and cosim["ss_pin"]["frame_spacing_max_us"] is not None
                                               and cosim["ss_pin"]["frame_spacing_max_us"] < k["NLINK_FRAME_TIMEOUT_US"],
        "flight_build_runs_and_exchanges_every_tick": flight_smoke["exchanges"] >= k["FC_LOOP_HZ"]
                                                      and flight_smoke["frame_spacing_max_us"] is not None
                                                      and flight_smoke["frame_spacing_max_us"] < k["NLINK_FRAME_TIMEOUT_US"]
                                                      and flight_smoke["frame_spacing_min_us"] > tick_us - margin_timeout_us,
        "flight_exchange_matches_generator": flight_smoke["ss_low_max_us"] is not None
                                             and abs(flight_smoke["ss_low_max_us"] - k["NLINK_XFER_US"]) <= 0.05 * k["NLINK_XFER_US"],
    }
    glink = None
    if vtol:
        sg = b["sg"][-1] if b["sg"] else {}
        sj = b["sj"][-1] if b["sj"] else {}
        shl = b["sh"][-1] if b["sh"] else {}
        funcs = disassemble(builds[FLIGHT_ENV]["elf"])
        bench_funcs = disassemble(builds[BENCH_ENV]["elf"])

        def _cyc(fs: dict, fn: str):
            try:
                return isr_cycles(fs, fn)["longest"]
            except (KeyError, ValueError):
                return None
        rx_cyc = _cyc(funcs, "__vector_30")                     # USART1 RX = the glink (flight build)
        udre_cyc = _cyc(funcs, "__vector_31")                   # USART1 UDRE
        t3_cyc = _cyc(bench_funcs, "__vector_26")               # TIMER3 COMPA (the bench's wire)
        nsbh = header_defines(os.path.join(FW, "src", "nav", "nav_simbench.h"), ("NAV_SB_GNS_CYCLE_US",))
        gns_rep = os.path.join(FW, "host_test", "build", "gns_simbench_report.json")
        gns_cycle_measured = None
        try:
            with open(gns_rep, "r", encoding="utf-8") as f:
                gns_cycle_measured = json.load(f).get("cycle", {}).get("state_to_result_max_us_with_isr")
        except (OSError, ValueError):
            pass
        state_bytes, result_bytes = 60, 26
        glink = {
            "nav_side": sg, "timing": shl, "synthetic_gns_and_fcu": sj,
            "usart1_rx_isr_cycles_flight": rx_cyc, "usart1_udre_isr_cycles_flight": udre_cyc,
            "timer3_wire_isr_cycles_bench": t3_cyc,
            "isr_load_per_cycle_us": None if rx_cyc is None or udre_cyc is None else
            round((result_bytes * rx_cyc + state_bytes * udre_cyc) * cyc_us, 1),
            "gns_cycle_us_bench_input": nsbh["NAV_SB_GNS_CYCLE_US"], "gns_cycle_us_measured": gns_cycle_measured,
            "_doc": "ISR cycles: the longest path incl. entry (avr-objdump).  "
                    "the NAV's glink side (nav_main.c v0.8.1): STATE frames written at k mod 5 == 4 after the correction part "
                    "(sa / sb: after the tick's Timer2 stamp), RESULT frames decoded in the tick they completed, latched "
                    "by the frame build of 5 m + 4 + L (la / ms); the synthetic GNS answers NAV_SB_GNS_CYCLE_US after the "
                    "STATE's last byte (a bench input = the GNS bench's measured worst cycle): margin_to_build = the frame "
                    "build of 5 m + 4 + L minus the RESULT's completion (us, min over the run).  The interrupt load per 20 ms "
                    "cycle: 26 RX + 60 UDRE interrupts of the flight build",
        }
        timing["fcu_bench_synthetic_nav_frame_us"] = sbh["SB_NAV_PUSH_US_VTOL"] + k["NLINK_XFER_US"]
        timing["nav_push_min_us_vtol"] = nav_h["NAV_PUSH_MIN_US_VTOL"]
        timing["compute_end_max_us"] = bench["compute_end_max_us"]
        timing["compute_end_from_tick_start_us"] = compute_t0
        timing["compute_end_plus_twi_us"] = round(compute_isr, 1)
        timing["push_us_for_fcu_bench"] = int(math.ceil(compute_isr / 10.0) * 10)
        timing["push_floor_rederived_us"] = int(math.ceil((compute_isr - sbh["SB_NAV_JITTER_US"]) / 10.0) * 10)
        timing["ticks_past_push_floor"] = bench["late_pushes"]
        timing["mandatory_work"] = work
        cosim_late = c["sb"][-1].get("late_pushes")
        faults = sj.get("gns_faults", -1)
        checks.update({
            # the floor bounds the frame time's variation: <= the NAV variation the FCU's background timing allows
            # (main.c NLINK_EARLY_US = SB_NAV_JITTER_US + the FCU's own tick-start lateness)
            "push_floor_width_le_fcu_nav_jitter": compute_isr - nav_h["NAV_PUSH_MIN_US_VTOL"] <= sbh["SB_NAV_JITTER_US"],
            "est_gate_flag_value_matches_nav_h": nav_h["NAV_F_EST_GATE"] == NAV_F_EST_GATE,
            "est_gate_open_in_flight_frames": sfx.get("est_gate_flight_frames", 0) > 0,
            "est_gate_never_on_the_pad": sfx.get("est_gate_pad_frames", 1) == 0,
            "nav_image_identical_for_field_and_demo": bool(profile_check and profile_check["identical"]),
            "fcu_bench_synthetic_nav_not_earlier": timing["fcu_bench_synthetic_nav_frame_us"]
                                                   >= timing["ready_flight_wire_with_isr_us"],
            # v0.8: the mandatory work fits the tick (with the real exchange and the TWI script) at 80 %, and on average
            "mandatory_work_le_80pct": (work["fraction_of_tick_wire_isr"] or 9) <= CRIT_LIMIT,
            "mandatory_work_mean_le_80pct": (work["mean_fraction_of_tick_wire"] or 9) <= CRIT_LIMIT,
            "no_tick_starts_late": (work["ticks_past_tick"] or 0) == 0,
            "push_floor_matches_measurement": nav_h["NAV_PUSH_MIN_US_VTOL"] == timing["push_floor_rederived_us"],
            "fcu_bench_push_is_measured": sbh["SB_NAV_PUSH_US_VTOL"] == timing["push_us_for_fcu_bench"],
            # v0.8.1 glink: a STATE every 50 Hz cycle, every RESULT latched except the four faulted cycles, nothing broken
            "glink_states_every_cycle": (sg.get("states_sent") or 0) >= (sg.get("t", 0) - 4) // 5,
            "glink_states_valid_at_gns": (sj.get("gns_states") or 0) > 0 and sj.get("gns_states_bad") == 0,
            "glink_only_faulted_cycles_missed": faults == 4 and sg.get("slot_missed") == faults,
            "glink_crc_fault_refused": sg.get("dec_bad_crc") == 1 and sg.get("dec_bad_type") == 0
                                       and sg.get("dec_wrong_dir") == 0,
            "glink_uart_clean": sg.get("uart_rx_overflow") == 0 and sg.get("uart_rx_errors") == 0
                                and sg.get("uart_tx_refused") == 0,
            "tilt_parts_at_k_plus_l": (sj.get("tilt_parts") or 0) > 0 and sj.get("tilt_parts_wrong_tick") == 0
                                      and sj.get("tilt_parts_on_time") == sj.get("tilt_parts")
                                      and sj.get("tilt_cycle_gaps") == faults,
            "slot_sets_complete_at_fcu": (sj.get("slot_sets") or 0) > 0 and sj.get("slot_broken") == 0,
            "result_margin_to_build_positive": (shl.get("margin_to_build_min_us") or -1) > 0,
            "bench_gns_cycle_not_shorter_than_measured": gns_cycle_measured is not None
                                                          and nsbh["NAV_SB_GNS_CYCLE_US"] >= gns_cycle_measured,
            "bench_wire_isr_not_cheaper_than_usart1_rx": rx_cyc is not None and t3_cyc is not None and t3_cyc >= rx_cyc,
        })
    report = {
        "mode": mode, "profile_check": profile_check,
        "builds": {e: {kk: vv for kk, vv in v.items() if kk != "elf"} for e, v in builds.items()},
        "constants": {**k, "imu_lead_us": lead_us, "nav_simbench": nsb},
        "bench": bench, "cosim_standalone": cosim, "flight_smoke": flight_smoke, "interrupts": isr, "timing": timing,
        "glink": glink,
        "stack": stack, "checks": checks, "pass": all(checks.values()),
    }
    for path in out_paths:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
    print("NAV builds: " + ", ".join(f"{e} flash {v['flash_bytes']} B RAM {v['static_ram_bytes']} B" for e, v in builds.items()))
    print(f"time-critical (tick -> frame delivered): pad {cosim['crit_pad_max_us']} us on the wire, flight "
          f"{crit_flight_wire} us on the wire (+{timing['twi_script_isr_us']} us TWI) = {timing['crit_fraction']:.1%} of the tick")
    print(f"frame ready after the tick: pad {cosim['ready_pad_max_us']} us, flight {ready_flight_wire} us; "
          f"exchange {wire_ex} us (bypass {bypass_ex} us); largest tick-to-tick increase {timing['ready_jump_max_us']} us "
          f"(FCU timeout margin {margin_timeout_us:.0f} us)")
    print(f"stack: flight free {flight_stack_free} B (used {stack_used} B + ISR {isr_stack_unexercised} B)")
    if vtol:
        print(f"VTOL push floor {nav_h['NAV_PUSH_MIN_US_VTOL']} us: compute end <= {bench['compute_end_max_us']} us "
              f"(+ {timing['twi_script_isr_us']} us {timing['baro_sensor']} bus interrupts = {timing['compute_end_plus_twi_us']} us), ticks "
              f"past the floor {bench['late_pushes']} (cosim {cosim_late})")
        w = timing["mandatory_work"]
        print(f"mandatory work (tick start -> FCU reply): flight max {w['flight_max_wire_isr_us']} us on the wire incl. "
              f"TWI = {100 * (w['fraction_of_tick_wire_isr'] or 0):.1f} % of the tick, mean {w['flight_mean_wire_us']} us "
              f"({100 * (w['mean_fraction_of_tick_wire'] or 0):.1f} %); bench: {w['ticks_into_imu_lead']} of {w['flight_ticks']} "
              f"flight ticks end inside the next IMU lead, {w['ticks_past_tick']} after the next tick, late start <= "
              f"{w['late_start_max_us']} us; push floor re-derived {timing['push_floor_rederived_us']} us (nav.h "
              f"{nav_h['NAV_PUSH_MIN_US_VTOL']}), FCU bench push {timing['push_us_for_fcu_bench']} us (simbench.h "
              f"{sbh['SB_NAV_PUSH_US_VTOL']})")
        g = glink
        print(f"glink (v0.8.1): NAV side {g['nav_side']}; timing {g['timing']}; synthetic GNS / FCU {g['synthetic_gns_and_fcu']}; "
              f"USART1 RX ISR {g['usart1_rx_isr_cycles_flight']} / UDRE {g['usart1_udre_isr_cycles_flight']} cycles (flight), "
              f"bench wire ISR {g['timer3_wire_isr_cycles_bench']}; per cycle {g['isr_load_per_cycle_us']} us; GNS cycle "
              f"bench input {g['gns_cycle_us_bench_input']} us vs measured {g['gns_cycle_us_measured']} us")
        print(f"VTOL: estimator-correction flag in {sfx.get('est_gate_flight_frames')} flight-mode frames, "
              f"{sfx.get('est_gate_pad_frames')} pad-mode frames; nav image field == demo: "
              f"{profile_check['identical'] if profile_check else None}; FCU bench synthetic NAV frame at "
              f"{timing['fcu_bench_synthetic_nav_frame_us']} us >= measured {timing['ready_flight_wire_with_isr_us']} us")
    print(f"flight build in simavr (no sensors): {flight_smoke['exchanges']} exchanges in {flight_smoke['simulated_s']} s, "
          f"SS low {flight_smoke['ss_low_min_us']}..{flight_smoke['ss_low_max_us']} us, frame spacing "
          f"{flight_smoke['frame_spacing_min_us']}..{flight_smoke['frame_spacing_max_us']} us")
    for name, ok in checks.items():
        if not ok:
            print(f"FAIL {name}")
    print(f"[{mode}] {'PASS' if report['pass'] else 'FAIL'} -> {', '.join(out_paths)}")
    return report["pass"]


if __name__ == "__main__":
    sys.exit(main())
