#!/usr/bin/env python3
"""Three-chip co-simulation of the v0.6.3 flight computer: the NAV (env ``nav_cosim``), the FCU (env ``fcu_cosim``) and
the COM (env ``com_cosim`` = lora_uart or ``com_cosim_sx127x``) run together in one libsimavr process, wired as on the
board - NAV SPI master -> FCU SPI slave (the nlink), FCU USART1 <-> COM USART0 (the ilink) - by ``tools/cosim/cosim.c``
(its header: the timing model, the SPI wiring and simavr's limitations it compensates, the event records).

What it does:

1. builds the four co-simulation HEXes and the flight builds ``nav`` / ``fcu`` / ``com_lora_uart`` / ``com_sx127x``
   (PlatformIO; the flight builds for their static RAM and their interrupt cycle counts - the stack criteria are about
   them);
2. finds the objects the harness reads (FCU: ``sb`` = this tick's simbench record, ``g_so`` the supervisor's outputs,
   ``g_sup.last_frame_tick``, ``g_nav`` the navigation frame the FCU holds; NAV: its ``sb`` record; COM:
   ``g_cosim_report``) with avr-nm, their field offsets with avr-gcc layout probes compiled from the firmware headers,
   the PWM compare registers from avr-libc's iom128.h and the channel map from params_gen.h (no offset or address is
   typed in here);
3. builds the harness with zig cc against PlatformIO's libsimavr.a and runs two probes: ``isr_cost`` (simavr charges 7
   cycles per bare-RETI interrupt, the chip 11; the harness adds the 4-cycle interrupt response to the core's clock, the
   probe checks the result is 11) and ``spi_probe`` (simavr's SPI as master and slave: byte time, the slave's echo, SS
   ignored - the facts the harness's shift-register model replaces);
4. runs the scenarios, several at a time:

   * ``normal``: the full 15 s run (pad alignment on the NAV, ARM at 3 s, LAUNCH at 4 s, powered flight) with the COM's
     in-firmware synthetic ground (polls every LINK_RTT_WORST_MS, a back-to-back burst at 9 s, silent 11..13 s), for
     each radio backend and each start-delay pair of the sweep (default 40: COM start delays 0.5 ms apart over 5 ticks,
     NAV start delays 0..1.75 ms - the ilink bursts land at every phase of the NAV / FCU tick and of the exchange);
   * ``diag``: one lora_uart run with simavr's own USART timing (11 bit times per byte) and one with simavr's own
     interrupt cost (no response cycles), for comparison only;
   * faults, each its own run, both backends (default from 6 s of powered flight):
     ilink - ``cut`` both directions for 4 s, ``reset`` the COM for 0.5 s, ``babble`` / ``babble_cmdhdr`` 1 s;
     NAV - (a) ``nav_hold`` the NAV held in reset from 6 s to the end, (b) ``nav_pulse`` the NAV reset in the middle of
     an exchange at 7 s (byte 30), held 1 ms + the chip's reset time-out (65 ms + 16K CK, CKSEL 1111 / SUT 11), then it
     reboots;
     nlink (c) - ``nl_flip`` (a MOSI bit flipped every 8th exchange, a MISO bit every 8th), ``nl_drop`` (a MOSI byte
     never reaches the FCU, every 8th), ``nl_extra`` (a phantom SCK burst shifts an extra byte through the FCU, every 8th),
     ``nl_noise`` (bit errors at 2e-4 on both lines), each for 1 s;

5. analyses each run's event file with the normative Python references (``sim/nlink_proto.py`` both nlink directions,
   ``sim/ilink_proto.py`` both ilink directions, ``sim/link_proto.py`` the TELEM frames, ``decode_log.py`` the FCU's log
   frames) and judges the criteria below; writes ``host_test/build/cosim_report.json``.

Measured (cycle-exact; every interrupt that really ran is in the window, with the chip's interrupt response):
  FCU   time-critical part = the frame's last byte at the FCU (or the tick start, without a frame) -> end of the
        time-critical part (tick marker down), split into interrupts by vector (nlink SPI, ilink RX / UDRE, log UDRE)
        and the rest; the sections of the firmware's own SB lines; the ilink bytes inside it.
  NAV   time-critical part = its Timer2 compare (the tick) -> SS high (the frame delivered), interrupts by vector.
  link  sample -> PWM register: the NAV's IMU sample (the MPU-6050 burst read starts FC_IMU_LEAD_COUNTS before its tick)
        -> the FCU's last vane compare-register write, through NAV tick, frame delivered, FCU tick start.
Not exercised by the co-simulation builds, added analytically (and said so in the report):
  - the NAV's I2C traffic (synthetic sensors, no TWI interrupt): the barometer driver's bus scripts (params_gen.h
    FC_BARO_SENSOR; BMP280: the forced write, 4 TWI interrupts, in phase BMP280_START_PHASE and the data read, 11, in
    phase BMP280_READ_PHASE; MS5611: one script of 11 in phases 0 and D1; longest path from the NAV flight ELF) inside
    the time-critical part of the tick that starts them; the MPU-6050 burst
    (19 TWI interrupts + the Timer0 lead ISR with mpu6050_start_async) runs in the FC_IMU_LEAD_COUNTS before the tick,
    outside the time-critical part (reported);
  - the IMU lead itself: simavr's Timer0 ignores the TCNT0 preload timer.c sets (measured: its compare fires 7 cycles
    AFTER Timer2's), so the lead is placed at the chip's FC_IMU_LEAD_COUNTS x FC_T2_PRESCALER before the Timer2 compare,
    and the Timer0 interrupt simavr runs at the tick is taken out of the NAV's figures, as is its simbench-only console
    UDRE;
  - the FCU's flight records on the log port (the simbench build packs them but sends its report lines instead): a
    continuously busy log port's UDRE interrupts during the measured interval (log_udre cycles from the FCU flight ELF).

Criteria (PASS / FAIL in the report; exit status 0 only if all pass):
  normal flight, every run: FCU time-critical part <= 80 % (every interrupt incl. the busy log port); NAV time-critical
  part <= 80 % (+ the barometer script, + the whole MPU-6050 burst as a bound); sample -> PWM register <= the 6-DOF sim's
  compute delay (sim/fc_timing.py FC_COMPUTE_DELAY_S_MEASURED, 4.71 ms; the count over one control tick is reported);
  the NAV's frame-ready increase from one tick to the next (+ the barometer script) inside the FCU's frame-timeout margin;
  0 missed ticks on both chips; no
  navigation frame lost and no FCU reply refused after the first; no wrong data accepted (every frame the FCU used
  equals bit for bit the one the NAV sent, the NAV accepted exactly the valid replies); the FCU's background ends before
  the NAV's next exchange starts; every IL_CMD reached the supervisor of the first tick after its last byte; no SNAP
  refused, no TELEM expired (sx127x: its burst's newest reply), every normal uplink answered within the reply window;
  COM pass < the ilink RX ring time; the COM's boot IL_STAT reaches the FCU whole (R3); POWERED reached, no interlock;
  no ilink frame / USART error.  Stacks: the flight builds keep >= 1024 B (depth = the lowest SP the harness saw in any
  run, interrupt frames included; the NAV + its deepest interrupt frame the cosim did not exercise).
  faults: cut - ABORT by I1 exactly link_abort_ticks after the last IL_CMD, RELEASED abort_release_ticks later;
  reset - no state change, the link resumes; babble - bounded (and judged separately: <= 80 %); nav_hold - fallback
  ticks NLINK_FRAME_TIMEOUT_US then FC_TICK_US apart, ABORT by I9 exactly ONB_IMU_FAIL_TICKS after the last valid frame,
  RELEASED abort_release_ticks later; nav_pulse - the same ABORT, with frames resumed (IMU_OK, no EST_VALID) before it;
  nl_* - every corrupted exchange refused, nothing wrong accepted, one tick without a frame per corrupted exchange, FCU
  tick spacing <= the frame timeout, no state change, time-critical part <= 80 %, the link continues.

    .venv/Scripts/python.exe firmware/atmega128_fc/tools/run_cosim.py [--sweep 40] [--jobs 12] [--only normal|faults]
"""
from __future__ import annotations

import argparse
import bisect
import concurrent.futures
import json
import math
import os
import re
import statistics
import struct
import subprocess
import sys
import time
from dataclasses import dataclass, field

FW = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ROOT = os.path.normpath(os.path.join(FW, "..", ".."))
TOOLS = os.path.join(FW, "tools")
COSIM = os.path.join(TOOLS, "cosim")
HB = os.path.join(COSIM, "build")
RUNS = os.path.join(HB, "runs")
REPORT = os.path.join(FW, "host_test", "build", "cosim_report.json")
for p in (ROOT, TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import decode_log as dl  # noqa: E402


def _sim_compute_delay_us() -> float:
    """The sample -> output delay the 6-DOF simulation uses (sim/fc_timing.py), in us."""
    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)
    from sim.fc_timing import FC_COMPUTE_DELAY_S_MEASURED
    return FC_COMPUTE_DELAY_S_MEASURED * 1e6
import run_nav_simbench as rnv  # noqa: E402  (NAV ISR cycles / stack from the flight ELF's disassembly)
import run_simbench as rsb  # noqa: E402  (FCU ISR cycles from the flight ELF's disassembly)
from sim import ilink_proto as il  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim import nlink_proto as nlp  # noqa: E402

COSIM_ENVS = {"fcu": "fcu_cosim", "nav": "nav_cosim", "lora_uart": "com_cosim", "sx127x": "com_cosim_sx127x"}
FLIGHT_ENVS = {"fcu": "fcu", "nav": "nav", "lora_uart": "com_lora_uart", "sx127x": "com_sx127x"}
BACKENDS = ("lora_uart", "sx127x")
F_CPU = 16_000_000
CYC_US = 16
RAM_BYTES = 4096
RAMEND = 0x10FF
DATA_BASE = 0x800000                       # avr-nm data addresses -> simavr avr->data[] index
CRIT_LIMIT = 0.80
STACK_MIN_FREE = 1024
ILINK_RX_RING = 64                         # hal/ilink_uart.h ILINK_UART_RX_BUF
CHIP_ISR_OVERHEAD = 4 + 3 + 4              # ATmega128: interrupt response, vector-table JMP, RETI (datasheet)
ISR_RESP = 4                               # cycles cosim.c adds per interrupt (the response simavr does not charge)
BURST_GAP_US = 100_000                     # uplinks closer than this belong to the synthetic ground's burst
RUN_END_MARGIN_US = 100_000                # uplinks this close to the FCU's end of run cannot be answered
SWEEP_SPAN_US = 20000.0                    # COM start delays spread over 5 ticks: every sub-tick phase of the ilink
NAV_DELAY_STEP_US = 250.0                  # NAV start delays (k % 8) x this
FAULT_AT_MS = 6000.0
NAV_RESET_TIMEOUT_MS = 65.0 + 16384 / 16000.0   # ATmega128 CKSEL 1111 / SUT 11: 16K CK + 65 ms (typ.) from reset
NAV_PULSE_MS = 1.0                         # the reset pulse itself
MPU_BURST_INTERRUPTS = (1 + 1 + 1) + (1 + 1 + 14)   # hal/mpu6050.c script: write the register (1 B), read 14 B
VANE_COMMIT_CYC = 2                        # the PWM hook sees the high byte; the low byte (the commit) follows (OUT 1 / STS 2)
FAULTS = {                                 # name -> (cosim fault, at ms, length ms, extra args)
    "cut": ("cut", FAULT_AT_MS, 4000.0, {}),
    "reset": ("reset", FAULT_AT_MS, 500.0, {}),
    "babble": ("babble", FAULT_AT_MS, 1000.0, {"babble": "c55c3102"}),
    "babble_cmdhdr": ("babble", FAULT_AT_MS, 1000.0, {"babble": "c55c0801"}),
    "nav_hold": ("nav_hold", FAULT_AT_MS, 0.0, {}),
    "nav_pulse": ("nav_pulse", 7000.0, NAV_PULSE_MS + NAV_RESET_TIMEOUT_MS, {"nav_reset_byte": "30"}),
    "nl_flip": ("nl_flip", FAULT_AT_MS, 1000.0, {"nl_every": "8"}),
    "nl_drop": ("nl_drop", FAULT_AT_MS, 1000.0, {"nl_every": "8"}),
    "nl_extra": ("nl_extra", FAULT_AT_MS, 1000.0, {"nl_every": "8"}),
    "nl_noise": ("nl_noise", FAULT_AT_MS, 1000.0, {"nl_ber": "2e-4", "seed": "20261015"}),
}
NL_FAULT_BITS = {"mosi_flip": 1, "miso_flip": 2, "drop": 4, "extra": 8, "noise": 16, "reset": 32}
MOSI_SIDE = 1 | 4 | 8 | 16 | 32            # the FCU's half of an exchange can be damaged by these
VEC_NAMES_FCU = {"v17": "nlink SPI slave (17)", "v19": "log UDRE (19)", "v30": "ilink RX (30)", "v31": "ilink UDRE (31)",
                 "vo": "other"}
VEC_NAMES_NAV = {"v9": "Timer2 tick (9)", "v15": "Timer0 IMU lead (15)", "v19": "console UDRE (19, simbench only)",
                 "vo": "other"}
FCU_SECTIONS = rsb.SECTION_NAMES

# ---- layouts read through the avr-gcc probes (type codes: struct module; "4I" = an array, read as a tuple) ----
LAYOUT_FCU = {
    "sb_tick_t": [("tick", "I"), ("state", "B"), ("ph_baro", "B"), ("loop_us", "H"), ("total_us", "H"),
                  ("overruns", "H"), ("n_wait", "B"), ("n_crit", "B"), ("n_cmds", "B"), ("seq_first", "B"),
                  ("seq_last", "B"), ("bg_bytes", "B"), ("crit_over", "H"), ("bg_capped", "H"), ("flood_ticks", "H"),
                  ("snap_dropped", "H"), ("flog_dropped", "H"), ("baro_late", "H"), ("il_cmds", "H"), ("il_snaps", "H"),
                  ("il_bad", "H"), ("rx_overflow", "H"), ("rx_errors", "H"), ("pwm_us", "H"), ("wait", "B"),
                  ("nav_ok", "B"), ("reason", "B"), ("nl_frames", "H"), ("nl_bad", "H"), ("nl_timeouts", "H"),
                  ("nl_seq_lost", "H"), ("nl_resyncs", "H"), ("nl_overruns", "H"), ("evq_dropped", "H"),
                  ("bg_end_us", "H")],
    "onb_outputs_t": [("state", "B"), ("throttle_milli", "H"), ("control_active", "B"), ("release", "B"),
                      ("reason", "B"), ("flags", "B"), ("reply", "B"), ("reply_seq", "B"), ("interlock", "B"),
                      ("n_exec", "B"), ("n_trans", "B"), ("trans", None)],
    "onb_trans_t": [("from", "B"), ("to", "B"), ("cause", "B")],
    "onboard_t": [("state", "B"), ("last_frame_tick", "I"), ("has_frame", "B")],
    "nl_nav_t": [("seq", "B"), ("q", "4I"), ("rate", "3I"), ("accel", "3I"), ("h_m", "I"), ("vz_mps", "I"),
                 ("h_baro_rel_m", "I"), ("tilt_deg", "I"), ("imu_fail", "H"), ("baro_fails", "H"), ("twi_errors", "H"),
                 ("loop_us_max", "H"), ("flags", "B"), ("reset_cause", "B")],
}
LAYOUT_NAV = {
    "nav_sb_tick_t": [("tick", "I"), ("fcu_state", "B"), ("pad", "B"), ("crit_us", "H"), ("ready_us", "H"),
                      ("total_us", "H"), ("overruns", "H"), ("fcu_valid", "H"), ("fcu_bad", "H"), ("loop_us_max", "H"),
                      ("reset_cause", "B")],
}
WAIT_SEEN, WAIT_LATE, WAIT_TIMEOUT = 0, 1, 2                          # main.c TICK_*


# ================================================================== tools
def pio_home() -> str:
    return os.environ.get("PLATFORMIO_CORE_DIR") or os.path.join(os.path.expanduser("~"), ".platformio")


def avr_tool(name: str) -> str:
    exe = os.path.join(pio_home(), "packages", "toolchain-atmelavr", "bin", name + (".exe" if os.name == "nt" else ""))
    if not os.path.exists(exe):
        raise SystemExit(f"{exe} missing (PlatformIO toolchain-atmelavr)")
    return exe


def simavr_dir() -> str:
    d = os.environ.get("SIMAVR_DIR") or os.path.join(pio_home(), "packages", "tool-simavr")
    if not os.path.exists(os.path.join(d, "lib", "libsimavr.a")):
        raise SystemExit(f"libsimavr.a not found under {d} (python -m platformio pkg install -g --tool "
                         "platformio/tool-simavr, or SIMAVR_DIR=<prefix with include/simavr and lib/libsimavr.a>)")
    return d


def sh(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", **kw)


def zig_cc(out: str, sources: list[str], extra: list[str] | None = None, opt: str = "-O2") -> list[str]:
    sim = simavr_dir()
    cmd = [sys.executable, "-m", "ziglang", "cc"] + (["-target", "x86_64-windows-gnu"] if os.name == "nt" else [])
    cmd += ["-std=c99", opt, "-Wall", "-Wextra", "-I", os.path.join(sim, "include", "simavr"), *(extra or []), "-o", out,
            *sources, os.path.join(COSIM, "elf_stubs.c"), os.path.join(sim, "lib", "libsimavr.a")]
    cmd += ["-lws2_32"] if os.name == "nt" else ["-lm"]
    r = sh(cmd)
    if r.returncode != 0:
        raise SystemExit(f"build of {os.path.basename(out)} failed:\n{r.stdout}{r.stderr}")
    return sorted(set(re.findall(r"warning: [^\n]*", r.stdout + r.stderr)))


def exe_name(name: str) -> str:
    return os.path.join(HB, name + (".exe" if os.name == "nt" else ""))


def pio_build(envs: list[str]) -> dict:
    r = sh([sys.executable, "-m", "platformio", "run", "-d", FW, *sum((["-e", e] for e in envs), [])])
    if r.returncode != 0:
        raise SystemExit(f"PlatformIO build failed:\n{r.stdout[-4000:]}\n{r.stderr[-2000:]}")
    out = {}
    blocks = re.split(r"^Processing (\S+)", r.stdout, flags=re.M)
    for i in range(1, len(blocks) - 1, 2):
        env, text = blocks[i], blocks[i + 1]
        ram = re.search(r"RAM:.*used (\d+) bytes", text)
        flash = re.search(r"Flash:.*used (\d+) bytes", text)
        out[env] = {"elf": os.path.join(FW, ".pio", "build", env, "firmware.elf"),
                    "hex": os.path.join(FW, ".pio", "build", env, "firmware.hex"),
                    "static_ram_bytes": int(ram.group(1)) if ram else None,
                    "flash_bytes": int(flash.group(1)) if flash else None,
                    "warnings": sorted(set(re.findall(r"warning: [^\n]*", text)))}
    missing = [e for e in envs if e not in out or out[e]["static_ram_bytes"] is None]
    if missing:
        raise SystemExit(f"no RAM figure for {missing} in the PlatformIO output")
    return out


def nm_symbols(elf: str) -> dict[str, tuple[int, int]]:
    r = sh([avr_tool("avr-nm"), "-S", elf])
    syms = {}
    for m in re.finditer(r"^([0-9a-f]+) ([0-9a-f]+) [A-Za-z] (\S+)$", r.stdout, re.M):
        syms[m.group(3)] = (int(m.group(1), 16), int(m.group(2), 16))
    for m in re.finditer(r"^([0-9a-f]+) [A-Za-z] (\S+)$", r.stdout, re.M):
        syms.setdefault(m.group(2), (int(m.group(1), 16), 0))
    return syms


def layout_probe(tag: str, layout: dict, includes: list[str], defines: list[str]) -> dict:
    """field offsets and struct sizes as avr-gcc lays them out (the src headers, the cosim build's defines)"""
    os.makedirs(HB, exist_ok=True)
    src = os.path.join(HB, f"layout_probe_{tag}.c")
    obj = os.path.join(HB, f"layout_probe_{tag}.o")
    lines = ["/* generated by tools/run_cosim.py: offsets as array sizes, read back with avr-nm -S */", "#include <stddef.h>"]
    lines += [f'#include "{h}"' for h in includes]
    for typ, fields in layout.items():
        lines.append(f"char probe_size__{typ}[sizeof({typ})] = {{1}};")
        for name, _ in fields:
            lines.append(f"char probe_off__{typ}__{name}[offsetof({typ}, {name}) + 1] = {{1}};")
    with open(src, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    r = sh([avr_tool("avr-gcc"), "-mmcu=atmega128", "-std=c99", "-Os", "-c", "-I", os.path.join(FW, "src"), *defines,
            "-o", obj, src])
    if r.returncode != 0:
        raise SystemExit(f"layout probe {tag} failed:\n{r.stdout}{r.stderr}")
    sizes = {name: size for name, (_, size) in nm_symbols(obj).items() if name.startswith("probe_")}
    lay: dict = {}
    for typ, fields in layout.items():
        t = {"size": sizes[f"probe_size__{typ}"], "fields": {}}
        for name, code in fields:
            t["fields"][name] = (sizes[f"probe_off__{typ}__{name}"] - 1, code)
        lay[typ] = t
    return lay


def decode_struct(lay: dict, typ: str, raw: bytes) -> dict:
    out = {}
    for name, (off, code) in lay[typ]["fields"].items():
        if code:
            v = struct.unpack_from("<" + code, raw, off)
            out[name] = v if len(v) > 1 else v[0]
    return out


def header_defines(path: str, names: tuple[str, ...], pattern: str = r"\(?(\d+)[uUlL]*(?![0-9A-Za-z_])") -> dict:
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()
    out = {}
    for n in names:
        m = re.search(rf"#define\s+{n}\s+{pattern}", src)
        if not m:
            raise SystemExit(f"{os.path.relpath(path, ROOT)}: #define {n} not found")
        out[n] = int(m.group(1), 0)
    return out


def pwm_registers() -> tuple[list[int], dict]:
    """data addresses of the OCRnxH registers for PWM channels 0..5 (params_gen.h PWM_CH_* order), from avr-libc's
    iom128.h; and the logical outputs -> channel (params_gen.h PWM_OUT_*)"""
    pg = os.path.join(FW, "src", "params_gen.h")
    with open(pg, "r", encoding="utf-8") as f:
        gen = f.read()
    ch = {m.group(1): int(m.group(2)) for m in re.finditer(r"#define\s+PWM_CH_(OC\d[ABC])\s+(\d+)", gen)}
    outs = {m.group(1): ch[m.group(2)] for m in re.finditer(r"#define\s+PWM_OUT_(\w+)\s+PWM_CH_(OC\d[ABC])", gen)}
    hdr = os.path.join(pio_home(), "packages", "toolchain-atmelavr", "avr", "include", "avr", "iom128.h")
    with open(hdr, "r", encoding="utf-8") as f:
        io = f.read()
    addr = [0] * 6
    for name, k in ch.items():
        m = re.search(rf"#define\s+OCR{name[2:]}H\s+_SFR_(IO|MEM)8\((0x[0-9A-Fa-f]+)\)", io)
        if not m:
            raise SystemExit(f"iom128.h: OCR{name[2:]}H not found")
        addr[k] = int(m.group(2), 16) + (0x20 if m.group(1) == "IO" else 0)
    if len(ch) != 6 or not all(addr) or not all(k in outs for k in ("VANE1", "VANE2", "VANE3", "VANE4", "ESC", "RELEASE")):
        raise SystemExit("params_gen.h: PWM_CH_* / PWM_OUT_* incomplete")
    return addr, outs


# ================================================================== probes
def isr_cost_probe() -> dict:
    """cycles simavr charges per interrupt, with and without the harness's response correction (tools/cosim/isr_cost*.c):
    a bare-RETI Timer0 ISR every 100 cycles during a fixed busy loop; the chip needs 4 (response) + 3 (JMP) + 4 (RETI)"""
    gcc, objcopy = avr_tool("avr-gcc"), avr_tool("avr-objcopy")
    exe = exe_name("isr_cost")
    zig_cc(exe, [os.path.join(COSIM, "isr_cost.c")], opt="-O1")
    hexes = {}
    for tag, flags in (("plain", []), ("isr", ["-DWITH_ISR"])):
        elf, hx = os.path.join(HB, f"isr_cost_{tag}.elf"), os.path.join(HB, f"isr_cost_{tag}.hex")
        r = sh([gcc, "-mmcu=atmega128", "-Os", *flags, "-o", elf, os.path.join(COSIM, "isr_cost_avr.c")])
        if r.returncode != 0:
            raise SystemExit(f"isr_cost_avr build failed: {r.stdout}{r.stderr}")
        sh([objcopy, "-O", "ihex", elf, hx])
        hexes[tag] = hx
    out = {"chip_cycles_per_interrupt": CHIP_ISR_OVERHEAD}
    for resp in (0, ISR_RESP):
        win = {}
        for tag, hx in hexes.items():
            o = sh([exe, hx, str(resp)])
            if o.returncode != 0:
                raise SystemExit(f"isr_cost run failed: {o.stdout}{o.stderr}")
            win[tag] = kvs(o.stdout.split())
        n = win["isr"]["isr"]
        charged = (win["isr"]["window"] - win["plain"]["window"]) / n
        out[f"resp{resp}"] = {"window_plain_cycles": win["plain"]["window"], "window_isr_cycles": win["isr"]["window"],
                              "interrupts": n, "cycles_per_interrupt": round(charged, 3),
                              "running_window_cycles": round(win["isr"]["running"] / n, 3)}
    used = out[f"resp{ISR_RESP}"]
    if abs(used["cycles_per_interrupt"] - CHIP_ISR_OVERHEAD) > 0.01 or abs(out["resp0"]["cycles_per_interrupt"] - 7) > 0.01:
        raise SystemExit(f"isr_cost: unexpected interrupt charge {out}")
    out["reti_cycles_outside_running_window"] = round(used["cycles_per_interrupt"] - used["running_window_cycles"])
    out["_doc"] = ("resp0 = simavr as it is (7 cycles: JMP + RETI, no interrupt response); resp4 = what every core of the "
                   "co-simulation charges (cosim.c isr_resp=4 adds the 4-cycle response when an interrupt starts "
                   "running): 11 = the ATmega128's.  The RUNNING window (vector entry -> RETI) then holds response + JMP + "
                   "body; the RETI's 4 cycles are outside it and are added per interrupt to the interrupt-time split")
    return out


def spi_probe() -> dict:
    gcc, objcopy = avr_tool("avr-gcc"), avr_tool("avr-objcopy")
    exe = exe_name("spi_probe")
    zig_cc(exe, [os.path.join(COSIM, "spi_probe.c")], opt="-O1")
    out = {}
    for mode, flags in (("master", []), ("slave", ["-DSLAVE"])):
        elf, hx = os.path.join(HB, f"spi_probe_{mode}.elf"), os.path.join(HB, f"spi_probe_{mode}.hex")
        r = sh([gcc, "-mmcu=atmega128", "-Os", *flags, "-o", elf, os.path.join(COSIM, "spi_probe_avr.c")])
        if r.returncode != 0:
            raise SystemExit(f"spi_probe_avr build failed: {r.stdout}{r.stderr}")
        sh([objcopy, "-O", "ihex", elf, hx])
        o = sh([exe, mode, hx])
        if o.returncode != 0:
            raise SystemExit(f"spi_probe {mode} failed: {o.stdout}{o.stderr}")
        out[mode] = kvs(o.stdout.split())
    m, s = out["master"], out["slave"]
    out["facts"] = {
        "master_byte_cycles_at_f_cpu_8": m.get("write_to_output"),
        "master_output_is_the_byte_written": m.get("output") == 0x5A,
        "master_input_becomes_spdr": m.get("received") == 0xC3,
        "slave_receives_with_ss_high": s.get("ss_pin") == 1 and s.get("received") == 0x3C,
        "slave_output_is_the_byte_received_not_the_byte_loaded": s.get("output") == 0x3C and s.get("loaded") == 0xA5}
    out["_doc"] = ("simavr's SPI, measured: the master's byte completes 8 x the SCK divider cycles after its SPDR write "
                   "and is output then (cosim.c uses that byte time); a slave receives whatever its SS pin says and "
                   "answers with the byte it received - so cosim.c gates the slave on the FCU's SS level and models its "
                   "shift register (the FCU's SPDR writes, write collisions) instead of using simavr's slave output")
    if out["facts"]["master_byte_cycles_at_f_cpu_8"] != 64 or not all(out["facts"].values()):
        out["unexpected"] = True
    return out


def build_harness() -> tuple[str, list[str]]:
    os.makedirs(HB, exist_ok=True)
    exe = exe_name("cosim")
    w = zig_cc(exe, [os.path.join(COSIM, "cosim.c")], ["-I", os.path.join(FW, "src", "com")])
    return exe, w


# ================================================================== scenarios
@dataclass
class Scn:
    name: str
    backend: str
    delay_us: float = 0.0                  # COM start
    nav_delay_us: float = 0.0              # NAV start
    fault: str = "none"
    fault_name: str = "none"
    fault_at_ms: float = FAULT_AT_MS
    fault_len_ms: float = 0.0
    extra: dict = field(default_factory=dict)
    wire_bits: int = 10
    isr_resp: int = ISR_RESP
    kind: str = "normal"                   # normal / fault / diag

    @property
    def path(self) -> str:
        return os.path.join(RUNS, self.name + ".txt")


def scenarios(sweep: int, only: str | None) -> list[Scn]:
    out: list[Scn] = []
    if only in (None, "normal"):
        for b in BACKENDS:
            for k in range(sweep):
                d = round(SWEEP_SPAN_US * k / sweep, 1)
                nd = (k % 8) * NAV_DELAY_STEP_US
                out.append(Scn(f"normal_{b}_d{int(d * 10):06d}_n{int(nd):04d}", b, d, nd))
        out.append(Scn("diag_simavr_11bit_lora_uart", "lora_uart", 0.0, 0.0, wire_bits=11, kind="diag"))
        out.append(Scn("diag_simavr_isr_cost_lora_uart", "lora_uart", 0.0, 0.0, isr_resp=0, kind="diag"))
    if only in (None, "faults"):
        for b in BACKENDS:
            for fname, (fault, at, length, extra) in FAULTS.items():
                out.append(Scn(f"fault_{fname}_{b}", b, 1000.0, 500.0, fault, fname, at, length, dict(extra), kind="fault"))
    return out


def run_scn(exe: str, s: Scn, hexes: dict, reads: dict, report_addr: dict) -> tuple[Scn, float, str]:
    args = [exe, f"fcu={hexes['fcu']}", f"com={hexes[s.backend]}", f"nav={hexes['nav']}", f"out={s.path}",
            f"report={report_addr[s.backend]}", f"rise={reads['rise']}", f"fall={reads['fall']}",
            f"fallchg={reads['fallchg']}", f"navrise={reads['navrise']}", f"pwm={reads['pwm']}",
            f"delay_us={s.delay_us}", f"nav_delay_us={s.nav_delay_us}", f"wire_bits={s.wire_bits}", "step=16",
            "seconds=16", f"isr_resp={s.isr_resp}", f"fault={s.fault}"]
    if s.fault != "none":
        args += [f"fault_at_ms={s.fault_at_ms}", f"fault_len_ms={s.fault_len_ms}"]
    args += [f"{k}={v}" for k, v in s.extra.items()]
    t0 = time.time()
    r = sh(args, timeout=1800)
    if r.returncode != 0:
        raise SystemExit(f"{s.name}: harness failed ({r.returncode}):\n{r.stdout[-2000:]}{r.stderr[-2000:]}")
    return s, time.time() - t0, r.stderr.strip()


# ================================================================== event file
@dataclass
class Ev:
    R: list = field(default_factory=list)
    S: list = field(default_factory=list)
    U: list = field(default_factory=list)
    C: list = field(default_factory=list)
    NC: list = field(default_factory=list)
    F: list = field(default_factory=list)
    T: dict = field(default_factory=dict)
    B: dict = field(default_factory=dict)
    O: list = field(default_factory=list)
    NT: list = field(default_factory=list)
    NB: dict = field(default_factory=dict)
    X: list = field(default_factory=list)
    E: list = field(default_factory=list)
    D: list = field(default_factory=list)
    W: list = field(default_factory=list)
    UL: list = field(default_factory=list)
    ST: list = field(default_factory=list)
    CR: dict = field(default_factory=dict)
    CS: dict = field(default_factory=dict)
    I: list = field(default_factory=list)
    M: dict = field(default_factory=dict)
    L: dict = field(default_factory=dict)


def kvs(tokens: list[str]) -> dict:
    out = {}
    for t in tokens:
        k, _, v = t.partition("=")
        try:
            out[k] = int(v, 0)
        except ValueError:
            out[k] = v
    return out


def _cn(v: str) -> tuple[int, int]:
    c, n = v.split(":")
    return int(c), int(n)


def parse_events(path: str) -> Ev:
    ev = Ev()
    with open(path, "r", encoding="latin-1") as f:
        for line in f:
            tag, _, rest = line.rstrip("\n").partition(" ")
            if tag == "R":
                i, tin, trx, b, src = rest.split()
                ev.R.append((int(i), int(tin), int(trx), int(b), src))
            elif tag == "S":
                t, b = rest.split()
                ev.S.append((int(t), int(b)))
            elif tag == "U":
                t, b = rest.split()
                ev.U.append((int(t), int(b)))
            elif tag in ("C", "NC"):
                t, _, text = rest.partition(" ")
                (ev.C if tag == "C" else ev.NC).append((int(t), text))
            elif tag == "F":
                t0, t1, hx = rest.split()
                ev.F.append((int(t0), int(t1), bytes.fromhex(hx)))
            elif tag == "T":
                tok = rest.split()
                kv = dict(x.split("=", 1) for x in tok[3:])
                rd = kv.get("rd", "")
                ev.T[int(tok[0])] = {"rise": int(tok[1]), "fall": int(tok[2]), "busy": int(kv["busy"]),
                                     "nisr": int(kv["nisr"]), "isr": {v: _cn(kv[v]) for v in VEC_NAMES_FCU},
                                     "pwm": [int(x) for x in kv["pwm"].split(",")],
                                     "reads": [bytes.fromhex(x) for x in rd.split(",")] if rd else []}
            elif tag == "B":
                tok = rest.split()
                ev.B[int(tok[0])] = (int(tok[1]), [bytes.fromhex(x) for x in tok[2].split(",")] if len(tok) > 2 else [])
            elif tag == "O":
                t, hx = rest.split()
                ev.O.append((int(t), bytes.fromhex(hx)))
            elif tag == "NT":
                tok = rest.split()
                kv = dict(x.split("=", 1) for x in tok[6:])
                ev.NT.append({"ep": int(tok[0]), "tick": int(tok[1]), "rise": int(tok[2]), "fall": int(tok[3]),
                              "t2": int(tok[4]), "t0": int(tok[5]), "busy": int(kv["busy"]), "nisr": int(kv["nisr"]),
                              "isr": {v: _cn(kv[v]) for v in VEC_NAMES_NAV}})
            elif tag == "NB":
                tok = rest.split()
                ev.NB[(int(tok[0]), int(tok[1]))] = bytes.fromhex(tok[2]) if len(tok) > 2 else b""
            elif tag == "X":
                tok = rest.split()
                ev.X.append({"n": int(tok[0]), "ep": int(tok[1]), "ss_lo": int(tok[2]), "ss_hi": int(tok[3]),
                             "nb": int(tok[4]), "t_first": int(tok[5]), "t_last": int(tok[6]), "late": int(tok[7]),
                             "late_hdr": int(tok[8]), "wcol": int(tok[9]),
                             "margin_min": int(tok[10].split(":")[0]), "margin_byte": int(tok[10].split(":")[1]),
                             "margin_vec": int(tok[10].split(":")[2]),
                             "isr_lat_max": int(tok[11]), "dly_max": int(tok[12]), "faults": int(tok[13]),
                             "mosi": b"" if tok[14] == "-" else bytes.fromhex(tok[14]),
                             "miso": b"" if tok[15] == "-" else bytes.fromhex(tok[15]),
                             "frx": tok[16], "miso_clean": tok[17]})
            elif tag == "E":
                tok = rest.split()
                ev.E.append((int(tok[0]), tok[1], tok[2:]))
            elif tag == "D":
                d, t, b, why = rest.split()
                ev.D.append((d, int(t), int(b), why))
            elif tag == "W":
                ev.W.append(rest.split())
            elif tag == "UL":
                ev.UL.append(tuple(int(x) for x in rest.split()))
            elif tag == "ST":
                ev.ST.append(tuple(int(x) for x in rest.split()))
            elif tag == "CR":
                tok = rest.split()
                ev.CR[int(tok[0])] = kvs(tok[1:])
            elif tag == "CS":
                tok = rest.split()
                ev.CS[int(tok[0])] = [int(x) for x in tok[1:]]
            elif tag == "I":
                c, v, n, tot, mn, mx = rest.split()
                ev.I.append((c, int(v), int(n), int(tot), int(mn), int(mx)))
            elif tag == "M":
                ev.M = kvs(rest.split())
            elif tag == "L":
                for item in rest.split():
                    v, n, lat, m = (int(x) for x in item.split(":"))
                    ev.L[v] = {"replies": n, "spif_to_write_max_cycles": lat, "margin_min_cycles": m}
    for x in ev.X:
        x["frx_bytes"] = x["mosi"] if x["frx"] == "=" else (b"" if x["frx"] == "-" else bytes.fromhex(x["frx"]))
        x["miso_clean10"] = x["miso"][:10] if x["miso_clean"] == "=" else bytes.fromhex(x["miso_clean"])
    ev.X.sort(key=lambda x: x["ss_lo"])
    ev.NT.sort(key=lambda n: n["rise"])
    return ev


# ================================================================== analysis helpers
def stats(xs: list[float], nd: int = 1) -> dict:
    if not xs:
        return {"n": 0}
    s = sorted(xs)

    def pct(p: float) -> float:
        return round(s[min(len(s) - 1, max(0, math.ceil(p * len(s)) - 1))], nd)
    return {"n": len(s), "min": round(s[0], nd), "p50": pct(0.5), "p90": pct(0.9), "p99": pct(0.99),
            "mean": round(statistics.fmean(s), nd), "max": round(s[-1], nd)}


def merge_by_vec(parts: list[dict]) -> dict:
    out: dict = {}
    for p in parts:
        for v, d in p.items():
            o = out.setdefault(v, {"replies": 0, "spif_to_write_max_cycles": 0, "margin_min_cycles": 10 ** 9})
            o["replies"] += d["replies"]
            o["spif_to_write_max_cycles"] = max(o["spif_to_write_max_cycles"], d["spif_to_write_max_cycles"])
            o["margin_min_cycles"] = min(o["margin_min_cycles"], d["margin_min_cycles"])
    return dict(sorted(out.items(), key=lambda kv: int(kv[0])))


def hist(xs: list[float], width: float) -> dict:
    out: dict = {}
    for x in xs:
        b = int(x // width) * width
        out[b] = out.get(b, 0) + 1
    return {f"{int(k)}..{int(k + width)}": v for k, v in sorted(out.items())}


def decode_ilink(stream: list[tuple], accept: int = il.ACCEPT_ALL) -> list[dict]:
    """frames in a byte stream [(t_in, t_rx, byte, src)...] with the normative decoder: type, payload, byte span"""
    dec = il.IlinkDecoder(accept)
    frames = []
    for i, (t_in, t_rx, b, src) in enumerate(stream):
        for fr in dec.feed(bytes([b])):
            n = len(fr.payload) + il.FRAME_OVERHEAD
            first = i - n + 1
            frames.append({"type": fr.type, "payload": fr.payload, "first": first, "last": i,
                           "t_first": stream[first][0], "t_last_in": t_in, "t_last_rx": t_rx,
                           "src": {stream[k][3] for k in range(first, i + 1)}})
    return frames


def byte_time(ev: Ev, core: str, uart: str, default: int) -> int:
    for w in ev.W:
        if w[0] == core and w[1] == uart:
            default = int(w[3])
    return default


def transitions(ev: Ev, lay: dict) -> list[dict]:
    so, tr = lay["onb_outputs_t"], lay["onb_trans_t"]
    off_tr, tsize = so["fields"]["trans"][0], tr["size"]
    out = []
    for tick, raw in ev.O:
        o = decode_struct(lay, "onb_outputs_t", raw)
        for k in range(o["n_trans"]):
            t = decode_struct(lay, "onb_trans_t", raw[off_tr + k * tsize: off_tr + (k + 1) * tsize])
            cause = t["cause"]
            out.append({"tick": tick, "t_s": None, "from": dl.state_name(t["from"]), "to": dl.state_name(t["to"]),
                        "cause": dl.cause_name(cause), "cause_code": cause, "reason": dl.reason_name(o["reason"]),
                        "interlock": o["interlock"]})
    return out


def nav_frame_fields(mosi: bytes) -> tuple | None:
    """(seq, 14 float bit patterns, 6 ints) of a valid NAV exchange (the bytes as the NAV sent them), None if invalid"""
    if len(mosi) != nlp.XFER_BYTES:
        return None
    r = nlp.decode_nav_raw(mosi)
    if r is None:
        return None
    f = nlp.nav_fields(*r)
    return f["seq"], tuple(f["float_bits"]), tuple(f["ints"][k] for k in nlp.NAV_INT_FIELDS)


def gnav_fields(lay: dict, raw: bytes) -> tuple:
    """the same from the FCU's g_nav (nl_nav_t as avr-gcc lays it out)"""
    g = decode_struct(lay, "nl_nav_t", raw)
    fb = (*g["q"], *g["rate"], *g["accel"], g["h_m"], g["vz_mps"], g["h_baro_rel_m"], g["tilt_deg"])
    return g["seq"], tuple(fb), tuple(g[k] for k in nlp.NAV_INT_FIELDS)


def sb_lines(lines: list, prefix: str) -> list[dict]:
    out = []
    for t, txt in lines:
        if txt.startswith(prefix + " t="):
            d = {k: int(v, 16) for k, v in re.findall(r"(?<![0-9a-z])([a-z0-9]+)=0x([0-9a-f]+)", txt)}
            m = re.search(r" s=((?:0x[0-9a-f]+ ?)+)", txt)
            if m:
                d["s"] = [int(x, 16) for x in m.group(1).split()]
            d["_t"] = t
            out.append(d)
    return out


# ================================================================== one run
def analyse(ev: Ev, s: Scn, lay: dict, layn: dict, k: dict) -> dict:
    tick_cyc = k["tick_us"] * CYC_US
    il_byte = byte_time(ev, "F", "1", 640)
    log_byte = byte_time(ev, "F", "0", 2800)
    reti = k["isr_cost"]["reti_cycles_outside_running_window"] if s.isr_resp else 4
    resp_missing = 0 if s.isr_resp else ISR_RESP                # diagnostic run: simavr's own interrupt charge
    lead_cyc = k["imu_lead_cycles"]
    log_udre = k["isr_fcu"]["log_udre"]["normal"]
    twi_phase = k["twi_phase_cycles"]
    res: dict = {"scenario": s.name, "backend": s.backend, "kind": s.kind, "com_start_delay_us": s.delay_us,
                 "nav_start_delay_us": s.nav_delay_us, "wire_bits": s.wire_bits, "isr_resp": s.isr_resp,
                 "ilink_byte_cycles": il_byte, "log_byte_cycles": log_byte}
    raw: dict = {}                                               # distributions for the summary (not in the report)
    fault_on = next((t for t, n, _ in ev.E if n == "fault_on"), None)
    fault_off = next((t for t, n, _ in ev.E if n == "fault_off"), None)
    nav_reset_t = next((t for t, n, _ in ev.E if n == "nav_reset"), None)

    # ---- the nlink exchanges
    xs = ev.X
    xt_last = [x["t_last"] for x in xs]
    xss_lo = [x["ss_lo"] for x in xs]
    for x in xs:
        x["nav_sent"] = nav_frame_fields(x["mosi"])
        x["fcu_rx_valid"] = len(x["frx_bytes"]) == nlp.XFER_BYTES and nlp.decode_nav_raw(x["frx_bytes"]) is not None
        x["miso_valid"] = len(x["miso"]) == nlp.XFER_BYTES and nlp.decode_fcu(x["miso"]) is not None
        x["miso_damaged"] = x["miso"][:10] != x["miso_clean10"]
        x["mosi_damaged"] = x["frx_bytes"] != x["mosi"]

    # ---- NAV ticks (their exchange, their sb record, barometer phase)
    nts = ev.NT
    for n in nts:
        n["sb"] = decode_struct(layn, "nav_sb_tick_t", ev.NB[(n["ep"], n["tick"])]) if ev.NB.get((n["ep"], n["tick"])) else None
        a = bisect.bisect_left(xss_lo, n["rise"])
        n["x"] = xs[a] if a < len(xs) and xs[a]["ss_lo"] <= n["fall"] and xs[a]["ep"] == n["ep"] else None
        if n["x"] is not None:
            n["x"]["nt"] = n
        n["phase"] = n["tick"] % k["baro_div"]
        sim_only = sum(n["isr"][v][0] + (reti + resp_missing) * n["isr"][v][1] for v in ("v15", "v19"))
        n["sim_only_cyc"] = sim_only
        n["twi_cyc"] = twi_phase.get(n["phase"], 0)
        if n["x"] is not None:
            n["crit_cyc"] = n["x"]["ss_hi"] - n["t2"] + resp_missing * n["nisr"]
            n["crit_equiv_cyc"] = n["crit_cyc"] - sim_only + n["twi_cyc"]
            n["crit_bound_cyc"] = n["crit_equiv_cyc"] + k["mpu_burst_isr_cycles"]   # as if the whole burst ran inside
            n["ready_cyc"] = n["x"]["t_last"] - n["t2"] + resp_missing * n["nisr"]

    # ---- FCU ticks
    rows: dict[int, dict] = {}
    for t, T in ev.T.items():
        r = {"tick": t, "rise": T["rise"], "fall": T["fall"], "isr": T["isr"], "nisr": T["nisr"], "pwm": T["pwm"],
             "isr_cyc": T["busy"] + (reti + resp_missing) * T["nisr"]}
        if T["reads"]:
            r["n_cmds"], r["seq_first"], r["seq_last"] = T["reads"][0][0], T["reads"][0][1], T["reads"][0][2]
            r["last_frame_tick"] = struct.unpack("<I", T["reads"][1])[0]
            r["gnav"] = T["reads"][2]
        if t in ev.B and ev.B[t][1]:
            r["sb"] = decode_struct(lay, "sb_tick_t", ev.B[t][1][0])
            r["nisr_tick"] = ev.B[t][0]
        rows[t] = r
    ticks = sorted(rows)
    rises = [rows[t]["rise"] for t in ticks]
    good = [rows[t] for t in ticks if "sb" in rows[t]]
    first_valid = next((r["tick"] for r in good if r["sb"]["nav_ok"]), None)
    used_x = -1
    prev = None
    for r in good:
        sb = r["sb"]
        j = bisect.bisect_right(xt_last, r["rise"]) - 1
        r["x"] = xs[j] if sb["wait"] != WAIT_TIMEOUT and j >= 0 and j != used_x else None
        if r["x"] is not None:
            used_x = j
        start = r["x"]["t_last"] if r["x"] is not None else r["rise"]
        r["start"] = start
        r["detect_cyc"] = r["rise"] - start
        # a frame that completed while the FCU was still busy AFTER its previous background: the single-chip report
        # lines simbench_end_of_loop writes (fcu_cosim only; the flight build has none) - flight-equivalent detection
        r["simbench_late"] = bool(r["x"] is not None and sb["wait"] == WAIT_LATE and prev is not None
                                  and prev["rise"] + prev["sb"]["total_us"] * CYC_US < r["x"]["t_last"])
        r["crit_tick_cyc"] = r["fall"] - r["rise"] + resp_missing * r["nisr"]
        prev = r
    detect_ref = max((r["detect_cyc"] for r in good if r["x"] is not None and r["sb"]["wait"] == WAIT_SEEN), default=0)
    for r in good:
        r["detect_equiv_cyc"] = detect_ref if r["simbench_late"] else r["detect_cyc"]
        r["crit_raw_cyc"] = r["fall"] - r["start"] + resp_missing * r["nisr"]
        r["crit_cyc"] = r["crit_tick_cyc"] + r["detect_equiv_cyc"]          # frame -> end, flight-equivalent start
        n_log = math.ceil(r["crit_cyc"] / log_byte)
        r["log_add_cyc"] = max(0, n_log - r["isr"]["v19"][1]) * log_udre
        r["crit_all_cyc"] = r["crit_cyc"] + r["log_add_cyc"]
        r["flight"] = r["sb"]["state"] in (2, 3)

    # ---- FCU: no wrong data accepted, decoder agreement; NAV: accepted exactly the valid replies
    wrong, stale, exact, agree_bad = [], 0, 0, []
    for r in good:
        sb = r["sb"]
        if sb["nav_ok"]:
            g = gnav_fields(lay, r["gnav"]) if "gnav" in r else None
            j = bisect.bisect_right(xt_last, r["rise"]) - 1
            cands = [xs[i]["nav_sent"] for i in range(j, max(-1, j - 4), -1) if xs[i]["nav_sent"] is not None]
            if g is not None and cands and g == cands[0]:
                exact += 1
            elif g is not None and g in cands:
                stale += 1
            else:
                wrong.append({"tick": r["tick"], "t_s": round(r["rise"] / F_CPU, 4), "seq": g[0] if g else None})
        x = r["x"]
        if x is not None and sb["wait"] == WAIT_SEEN and r["rise"] >= x["t_last"] and len(x["frx_bytes"]) == nlp.XFER_BYTES:
            if bool(sb["nav_ok"]) != x["fcu_rx_valid"]:
                agree_bad.append({"tick": r["tick"], "nav_ok": sb["nav_ok"], "python_valid": x["fcu_rx_valid"]})
    nav_accept_bad, nav_accepted_damaged = [], 0
    prev_valid: dict[int, int] = {}
    for n in nts:
        if n["sb"] is None or n["x"] is None:
            continue
        ep = n["ep"]
        dv = n["sb"]["fcu_valid"] - prev_valid.get(ep, 0)
        prev_valid[ep] = n["sb"]["fcu_valid"]
        if dv not in (0, 1) or bool(dv) != n["x"]["miso_valid"]:
            nav_accept_bad.append({"ep": ep, "tick": n["tick"], "delta_valid": dv, "python_valid": n["x"]["miso_valid"]})
        if dv == 1 and n["x"]["miso_damaged"]:
            nav_accepted_damaged += 1
    res["nlink_integrity"] = {
        "fcu_frames_used": exact + stale + len(wrong), "fcu_used_newest_exchange": exact, "fcu_used_older_exchange": stale,
        "fcu_wrong_data_accepted": wrong[:10], "fcu_wrong_data_accepted_n": len(wrong),
        "fcu_decoder_vs_python_mismatch": agree_bad[:10], "fcu_decoder_vs_python_mismatch_n": len(agree_bad),
        "nav_accept_vs_python_mismatch": nav_accept_bad[:10], "nav_accept_vs_python_mismatch_n": len(nav_accept_bad),
        "nav_accepted_damaged_replies": nav_accepted_damaged,
        "_doc": "fcu: every tick whose supervisor used a navigation frame (nav_ok): the FCU's g_nav after the tick, field "
                "by field (floats as bit patterns), against the frame the NAV SENT in the newest exchange before the tick "
                "(older: one of the 3 before - a frame noticed late); wrong = none of them.  decoder: nav_ok against "
                "sim/nlink_proto.py on the bytes the FCU received.  nav: per NAV tick, its accepted-reply counter's step "
                "against sim/nlink_proto.py decode_fcu on the bytes the NAV received; damaged = a reply whose first 10 "
                "bytes differ from what the FCU's shift register sent"}

    # ---- nlink accounting
    last = good[-1]["sb"] if good else {}
    sb_first = rows[first_valid]["sb"] if first_valid is not None else {}
    after_first = [r for r in good if first_valid is not None and r["tick"] > first_valid]
    nav_valid_first = {}
    for x in xs:
        if x["miso_valid"] and x["ep"] not in nav_valid_first:
            nav_valid_first[x["ep"]] = x["n"]
    res["nlink"] = {
        "exchanges": len(xs), "exchange_bytes_not_72": sum(1 for x in xs if x["nb"] != nlp.XFER_BYTES),
        "ss_low_us": stats([(x["ss_hi"] - x["ss_lo"]) / CYC_US for x in xs if x["nb"] == nlp.XFER_BYTES], 2),
        "fcu_first_valid_frame_tick": first_valid,
        "fcu_ticks_without_valid_frame_after_the_first": sum(1 for r in after_first if not r["sb"]["nav_ok"]),
        "fcu_timeout_ticks_after_the_first": sum(1 for r in after_first if r["sb"]["wait"] == WAIT_TIMEOUT),
        "fcu_late_ticks": sum(1 for r in good if r["sb"]["wait"] == WAIT_LATE),
        "fcu_counters_end": {key: last.get(key) for key in ("nl_frames", "nl_bad", "nl_timeouts", "nl_seq_lost",
                                                            "nl_resyncs", "nl_overruns")},
        "fcu_counters_since_first_frame": {key: (last.get(key, 0) - sb_first.get(key, 0)) for key in
                                           ("nl_bad", "nl_timeouts", "nl_seq_lost", "nl_resyncs", "nl_overruns")},
        "nav_replies_refused_after_its_first_valid": sum(1 for x in xs if x["ep"] in nav_valid_first
                                                         and x["n"] > nav_valid_first[x["ep"]] and not x["miso_valid"]),
        "nav_replies_refused_total": sum(1 for x in xs if not x["miso_valid"]),
        "reply_bytes_late": sum(x["late"] for x in xs), "reply_bytes_1_9_late": sum(x["late_hdr"] for x in xs),
        "write_collisions": sum(x["wcol"] for x in xs),
        "reply_write_margin_min_cycles": min((x["margin_min"] for x in xs if x["margin_min"] >= 0), default=None),
        "reply_write_margin_min_at": next(({"exchange": x["n"], "byte": x["margin_byte"], "fcu_isr_running_at_spif":
                                            x["margin_vec"]} for x in sorted(xs, key=lambda x: x["margin_min"])
                                           if x["margin_min"] >= 0), None),
        "reply_write_by_interrupt_at_spif": {str(v): d for v, d in sorted(ev.L.items())},
        "reply_write_after_spif_max_cycles": max((x["isr_lat_max"] for x in xs), default=None),
        "delivery_lateness_max_cycles": max((x["dly_max"] for x in xs), default=None),
        "exchanges_damaged": {"mosi": sum(1 for x in xs if x["mosi_damaged"]), "miso": sum(1 for x in xs if x["miso_damaged"])},
        "harness": {key: ev.M.get(key) for key in ("spi_bytes", "spi_late", "spi_late_hdr", "spi_wcol", "spi_dropped",
                                                    "spi_extra", "spi_flips", "spi_aborted", "spi_ss_high_bytes",
                                                    "spi_out_mismatch", "spi_dly_max", "ss_edges")},
        "_doc": "late = a reply byte (1..71) the FCU's SPI interrupt had not loaded when the NAV started that byte (the NAV "
                "then reads the echo of the previous byte); 1..9 late makes the NAV refuse that reply.  margin = master's "
                "SPDR write - the slave's reply write (cycles, smallest); after_spif = the slave's SPIF -> its reply write "
                "(largest, incl. any interrupt in front); delivery lateness = how far past the byte's end the FCU core was "
                "when the harness delivered it (instruction granularity)"}

    # ---- FCU time-critical part
    def tick_view(r: dict) -> dict:
        sb = r.get("sb", {})
        x = r.get("x")
        c = {"tick": r["tick"], "t_s": round(r["rise"] / F_CPU, 4), "state": dl.state_name(sb.get("state", 0)),
             "wait": sb.get("wait"), "nav_ok": sb.get("nav_ok"),
             "crit_from_frame_us": round(r["crit_cyc"] / CYC_US, 1),
             "crit_all_us": round(r["crit_all_cyc"] / CYC_US, 1), "log_udre_added_us": round(r["log_add_cyc"] / CYC_US, 1),
             "detect_us": round(r["detect_cyc"] / CYC_US, 1), "simbench_late": r["simbench_late"],
             "detect_equiv_us": round(r["detect_equiv_cyc"] / CYC_US, 1),
             "tick_start_to_end_us": round(r["crit_tick_cyc"] / CYC_US, 1),
             "isr_us": round(r["isr_cyc"] / CYC_US, 1), "isr_runs": r["nisr"],
             "isr_by_vector_us": {VEC_NAMES_FCU[v]: round((cc + (reti + resp_missing) * nn) / CYC_US, 1)
                                  for v, (cc, nn) in r["isr"].items() if nn},
             "ilink_bytes_in_crit": {k2: v for k2, v in r.get("rx_in_crit", {}).items() if v},
             "firmware_loop_us": sb.get("loop_us"), "total_us": sb.get("total_us")}
        if x is not None and x.get("nt") is not None:
            c["nav_tick"] = x["nt"]["tick"]
        return c

    stream = [(tin, trx, b, src) for _, tin, trx, b, src in ev.R]
    frames = decode_ilink(stream, il.ACCEPT_FCU)
    cls = ["junk"] * len(stream)
    for fr in frames:
        for i in range(fr["first"], fr["last"] + 1):
            cls[i] = il.TYPE_NAMES.get(fr["type"], "?")
    rx_times = [x[1] for x in stream]
    for r in good:
        a = bisect.bisect_left(rx_times, r["start"])
        b = bisect.bisect_right(rx_times, r["fall"])
        cnt = {"CMD": 0, "STAT": 0, "junk": 0}
        for i in range(a, b):
            cnt[cls[i]] = cnt.get(cls[i], 0) + 1
        r["rx_in_crit"] = cnt
    flight = [r for r in good if r["flight"]]
    window = [r for r in good if fault_on is not None and fault_on <= r["rise"] <= (fault_off or fault_on)] \
        if fault_on is not None else []
    worst = max(good, key=lambda r: r["crit_all_cyc"]) if good else None
    res["fcu_crit"] = {
        "ticks": len(good), "flight_ticks": len(flight),
        "max_all_us": round(worst["crit_all_cyc"] / CYC_US, 1) if worst else None,
        "max_all_fraction": round(worst["crit_all_cyc"] / tick_cyc, 4) if worst else None,
        "worst_tick": tick_view(worst) if worst else None,
        "max_from_frame_us": round(max(r["crit_cyc"] for r in good) / CYC_US, 1) if good else None,
        "max_from_frame_raw_us": round(max(r["crit_raw_cyc"] for r in good) / CYC_US, 1) if good else None,
        "simbench_late_ticks": sum(1 for r in good if r["simbench_late"]),
        "simbench_late_detect_raw_max_us": round(max((r["detect_cyc"] for r in good if r["simbench_late"]), default=0) / CYC_US, 1),
        "detect_ref_us": round(detect_ref / CYC_US, 1),
        "max_from_tick_start_us": round(max(r["crit_tick_cyc"] for r in good) / CYC_US, 1) if good else None,
        "flight_max_all_us": round(max(r["crit_all_cyc"] for r in flight) / CYC_US, 1) if flight else None,
        "pad_max_all_us": round(max((r["crit_all_cyc"] for r in good if r["sb"]["state"] in (0, 1)), default=0) / CYC_US, 1),
        "no_isr_max_us": round(max(r["crit_cyc"] - r["isr_cyc"] for r in good) / CYC_US, 1) if good else None,
        "detect_us": stats([r["detect_cyc"] / CYC_US for r in good if r["x"] is not None and r["sb"]["wait"] == WAIT_SEEN
                            and r["tick"] != first_valid]),
        "isr_in_crit_max_us_by_vector": {VEC_NAMES_FCU[v]: round(max(((r["isr"][v][0] + (reti + resp_missing) * r["isr"][v][1])
                                                                       for r in good if first_valid is not None
                                                                       and r["tick"] > first_valid), default=0) / CYC_US, 1)
                                         for v in VEC_NAMES_FCU},
        "isr_in_crit_before_first_frame_max_us_by_vector": {VEC_NAMES_FCU[v]: round(max(((r["isr"][v][0] + (reti + resp_missing) * r["isr"][v][1])
                                                                       for r in good if first_valid is None
                                                                       or r["tick"] <= first_valid), default=0) / CYC_US, 1)
                                                            for v in VEC_NAMES_FCU},
        "most_ilink_bytes_in_one_crit_part": max((sum(r["rx_in_crit"].values()) for r in good), default=0),
        "no_isr_max_step5_idle": None,
    }
    idle = [r for r in flight if r["sb"]["n_crit"] == 0]
    if idle:
        w = max(idle, key=lambda r: r["crit_cyc"] - r["isr_cyc"])
        res["fcu_crit"]["no_isr_max_step5_idle"] = dict(tick_view(w), no_isr_us=round((w["crit_cyc"] - w["isr_cyc"]) / CYC_US, 1))
    cats = {}
    for r in good:
        c = r["rx_in_crit"]
        key = ("CMD+STAT" if c["CMD"] and c["STAT"] else "CMD" if c["CMD"] else "STAT" if c["STAT"]
               else "junk" if c["junk"] else "none") + ("+log" if r["isr"]["v19"][1] else "") + \
              ("+spi" if r["isr"]["v17"][1] else "")
        if key not in cats or r["crit_all_cyc"] > cats[key]["crit_all_cyc"]:
            cats[key] = r
    res["fcu_crit"]["max_by_coincidence"] = {k2: tick_view(r) for k2, r in sorted(cats.items())}
    raw["fcu_crit_all_us"] = [r["crit_all_cyc"] / CYC_US for r in good]

    # FCU sections, from the firmware's own SB lines (Timer1: every interrupt inside, with the response cycles)
    fsb = sb_lines(ev.C, "SB")
    fl = [d["s"] for d in fsb if d.get("st") in (2, 3) and "s" in d]
    res["fcu_sections_flight_max_us"] = {n: max((v[i] for v in fl if len(v) > i), default=None)
                                         for i, n in enumerate(FCU_SECTIONS)}
    res["fcu_stack_free_report_min"] = min((d["sf"] for d in fsb if "sf" in d), default=None)

    # ---- NAV time-critical part
    nv = [n for n in nts if n.get("crit_cyc") is not None and n["sb"] is not None]
    nflight = [n for n in nv if not n["sb"]["pad"]]
    npad = [n for n in nv if n["sb"]["pad"]]

    def nav_view(n: dict) -> dict:
        return {"ep": n["ep"], "tick": n["tick"], "t_s": round(n["t2"] / F_CPU, 4), "pad": bool(n["sb"]["pad"]),
                "fcu_state_seen": dl.state_name(n["sb"]["fcu_state"]), "baro_phase": n["phase"],
                "crit_us": round(n["crit_cyc"] / CYC_US, 1), "crit_equiv_us": round(n["crit_equiv_cyc"] / CYC_US, 1),
                "crit_bound_us": round(n["crit_bound_cyc"] / CYC_US, 1),
                "ready_us": round(n["ready_cyc"] / CYC_US, 1), "twi_added_us": round(n["twi_cyc"] / CYC_US, 1),
                "simbench_only_isr_removed_us": round(n["sim_only_cyc"] / CYC_US, 1),
                "isr_by_vector_us": {VEC_NAMES_NAV[v]: round((cc + (reti + resp_missing) * nn) / CYC_US, 1)
                                     for v, (cc, nn) in n["isr"].items() if nn},
                "firmware_crit_us": n["sb"]["crit_us"], "firmware_total_us": n["sb"]["total_us"]}

    nworst = max(nv, key=lambda n: n["crit_equiv_cyc"]) if nv else None
    jumps = []
    by_ep: dict = {}
    for n in nv:
        by_ep.setdefault(n["ep"], []).append(n)
    for ep, lst in by_ep.items():
        for a, b in zip(lst, lst[1:]):
            if b["tick"] == a["tick"] + 1:
                jumps.append((b["ready_cyc"] + b["twi_cyc"] - a["ready_cyc"]) / CYC_US)
    nav_last = {ep: lst[-1]["sb"] for ep, lst in by_ep.items()}
    # background end against the chip's IMU lead (the MPU-6050 burst read starts then)
    bg_margin = [((n["t2"] + tick_cyc - lead_cyc) - (n["rise"] + n["sb"]["total_us"] * CYC_US)) / CYC_US for n in nv]
    res["nav_crit"] = {
        "ticks": len(nv), "flight_ticks": len(nflight),
        "max_equiv_us": round(nworst["crit_equiv_cyc"] / CYC_US, 1) if nworst else None,
        "max_equiv_fraction": round(nworst["crit_equiv_cyc"] / tick_cyc, 4) if nworst else None,
        "worst_tick": nav_view(nworst) if nworst else None,
        "max_measured_us": round(max(n["crit_cyc"] for n in nv) / CYC_US, 1) if nv else None,
        "max_bound_us": round(max(n["crit_bound_cyc"] for n in nv) / CYC_US, 1) if nv else None,
        "max_bound_fraction": round(max(n["crit_bound_cyc"] for n in nv) / tick_cyc, 4) if nv else None,
        "flight_max_equiv_us": round(max(n["crit_equiv_cyc"] for n in nflight) / CYC_US, 1) if nflight else None,
        "pad_max_equiv_us": round(max(n["crit_equiv_cyc"] for n in npad) / CYC_US, 1) if npad else None,
        "flight_max_by_baro_phase_us": {str(ph): round(max(n["crit_equiv_cyc"] for n in nflight if n["phase"] == ph) / CYC_US, 1)
                                          for ph in sorted({n["phase"] for n in nflight})},
        "frame_ready_after_tick_us": {"pad": stats([n["ready_cyc"] / CYC_US for n in npad]),
                                      "flight": stats([n["ready_cyc"] / CYC_US for n in nflight]),
                                      "flight_with_twi": stats([(n["ready_cyc"] + n["twi_cyc"]) / CYC_US for n in nflight])},
        "frame_ready_largest_increase_tick_to_tick_us": round(max(jumps), 1) if jumps else None,
        "background_end_before_imu_lead_min_us": round(min(bg_margin), 1) if bg_margin else None,
        "overruns_end": {str(ep): sb["overruns"] for ep, sb in nav_last.items()},
        "marker_jumps": ev.M.get("nav_marker_jumps"),
    }
    nsb = sb_lines(ev.NC, "SB")
    res["nav_stack_free_report_min"] = min((d["sf"] for d in nsb if "sf" in d), default=None)
    raw["nav_crit_equiv_us"] = [n["crit_equiv_cyc"] / CYC_US for n in nv]

    # ---- sample -> PWM register (the NAV's IMU sample -> the FCU's last vane register)
    vane_ch, esc_ch = k["vane_channels"], k["esc_channel"]
    lat = []
    for r in good:
        x = r["x"]
        if x is None or not r["sb"]["nav_ok"] or x.get("nt") is None or r["tick"] == first_valid:
            continue                                         # (the first frame meets the FCU's boot clock: boot_sync)
        pv = [r["pwm"][c] for c in vane_ch]
        if not all(pv):
            continue
        n = x["nt"]
        t_vane = max(pv) + VANE_COMMIT_CYC
        t_lead = n["t2"] - lead_cyc
        nav_part = x["t_last"] - n["t2"]
        fcu_part = t_vane - r["rise"]
        log_add = math.ceil(fcu_part / log_byte) * log_udre
        meas = t_vane - t_lead
        equiv = (lead_cyc + nav_part + r["detect_equiv_cyc"] + fcu_part - n["sim_only_cyc"] + n["twi_cyc"] + log_add
                 + resp_missing * (n["nisr"] + r["nisr"]))
        lat.append({"tick": r["tick"], "nav_tick": n["tick"], "flight": r["flight"], "control": r["sb"]["state"] in (2, 3),
                    "lead_us": lead_cyc / CYC_US, "nav_us": nav_part / CYC_US, "detect_us": r["detect_equiv_cyc"] / CYC_US,
                    "simbench_late": r["simbench_late"],
                    "fcu_us": fcu_part / CYC_US, "measured_us": meas / CYC_US, "equiv_us": equiv / CYC_US,
                    "nav_tick_to_vane_equiv_us": (equiv - lead_cyc) / CYC_US,
                    "frame_to_vane_us": (r["detect_equiv_cyc"] + fcu_part) / CYC_US,
                    "esc_after_vane_us": (r["pwm"][esc_ch] - max(pv)) / CYC_US if r["pwm"][esc_ch] else None,
                    "twi_us": n["twi_cyc"] / CYC_US, "log_us": log_add / CYC_US, "sim_only_us": n["sim_only_cyc"] / CYC_US})
    lw = max(lat, key=lambda d: d["equiv_us"]) if lat else None
    res["latency"] = {
        "ticks": len(lat),
        "imu_sample_to_vane_equiv_us": stats([d["equiv_us"] for d in lat]),
        "imu_sample_to_vane_equiv_flight_us": stats([d["equiv_us"] for d in lat if d["flight"]]),
        "imu_sample_to_vane_measured_us": stats([d["measured_us"] for d in lat]),
        "nav_tick_to_vane_equiv_us": stats([d["nav_tick_to_vane_equiv_us"] for d in lat]),
        "frame_to_vane_us": stats([d["frame_to_vane_us"] for d in lat]),
        "parts_us": {key: stats([d[key] for d in lat]) for key in ("nav_us", "detect_us", "fcu_us")},
        "esc_after_vane_us": stats([d["esc_after_vane_us"] for d in lat if d["esc_after_vane_us"] is not None]),
        "worst": {kk: (round(v, 1) if isinstance(v, float) else v) for kk, v in lw.items()} if lw else None,
        "over_one_tick": sum(1 for d in lat if d["equiv_us"] > k["tick_us"]),
    }
    raw["latency_equiv_us"] = [d["equiv_us"] for d in lat]
    raw["latency_equiv_flight_us"] = [d["equiv_us"] for d in lat if d["flight"]]
    raw["latency_parts"] = {key: [d[key] for d in lat] for key in ("nav_us", "detect_us", "fcu_us", "frame_to_vane_us")}

    # ---- FCU tick spacing, missed ticks, background vs the next exchange
    for r in good:
        r["start_equiv"] = r["start"] + r["detect_equiv_cyc"]
    sp = [(b["start_equiv"] - a["start_equiv"]) / CYC_US for a, b in zip(good, good[1:]) if first_valid is not None
          and a["tick"] > first_valid]
    res["fcu_tick_spacing_us"] = stats(sp)
    res["fcu_tick_spacing_raw_us"] = stats([(b["rise"] - a["rise"]) / CYC_US for a, b in zip(good, good[1:])
                                            if first_valid is not None and a["tick"] > first_valid])
    fv = rows.get(first_valid) if first_valid is not None else None
    res["boot_sync"] = None if fv is None else {
        "first_valid_frame_tick": first_valid, "t_s": round(fv["rise"] / F_CPU, 4), "wait": fv["sb"]["wait"],
        "detect_us": round(fv["detect_cyc"] / CYC_US, 1),
        "timeout_ticks_before": sum(1 for r in good if r["tick"] < first_valid),
        "_doc": "the FCU runs on its own clock (frame timeout, then Timer2) until the NAV's first frame; that frame can "
                "complete while such a tick is still busy (detect = its wait).  Left out of the spacing, the interrupt "
                "split and the latency (they describe the frame-triggered loop)"}
    bgm = []
    for r in good:
        if r["sb"]["wait"] != WAIT_SEEN or first_valid is None or r["tick"] < first_valid:
            continue
        a = bisect.bisect_right(xss_lo, r["rise"])
        if a < len(xs):
            bgm.append(((xs[a]["ss_lo"] - (r["rise"] + r["sb"]["total_us"] * CYC_US)) / CYC_US, r["tick"]))
    res["fcu_background_end_before_next_exchange_us"] = {
        "min": round(min(bgm)[0], 1) if bgm else None, "tick": min(bgm)[1] if bgm else None,
        "ticks_past": sum(1 for m, _ in bgm if m < 0), "bg_end_us_after_anchor_max": max((r["sb"]["bg_end_us"] for r in good), default=None),
        "_doc": "frame ticks: the next exchange's SS fall - (tick start + the firmware's loop incl. background, Timer1); "
                "< 0: a background job still ran when the NAV's next exchange began"}
    res["loop"] = {"total_max_us": max((r["sb"]["total_us"] for r in good), default=None),
                   "fcu_timeout_ticks_after_first_frame": res["nlink"]["fcu_timeout_ticks_after_the_first"],
                   "fcu_marker_jumps": ev.M.get("marker_jumps"), "fcu_last_tick": ev.M.get("last_tick"),
                   "nav_overruns": res["nav_crit"]["overruns_end"], "nav_marker_jumps": ev.M.get("nav_marker_jumps")}
    res["fcu_counters"] = {k2: last.get(k2) for k2 in ("il_cmds", "il_snaps", "il_bad", "rx_overflow", "rx_errors",
                                                       "crit_over", "bg_capped", "flood_ticks", "snap_dropped",
                                                       "flog_dropped", "baro_late", "evq_dropped")}
    res["final_state"] = dl.state_name(last.get("state", 0)) if last else None
    res["reached_powered"] = any(r["sb"]["state"] == 2 for r in good)

    # ---- ilink: COM -> FCU stream, IL_CMD -> the supervisor that got it
    cmds = [fr for fr in frames if fr["type"] == il.TYPE_CMD]
    stats_fr = [fr for fr in frames if fr["type"] == il.TYPE_STAT]
    com_tx = sorted([(tin, tin, b, "c") for _, tin, _, b, src in ev.R if src == "c"]
                    + [(t, t, b, why) for d, t, b, why in ev.D if d == "c2f"])
    com_frames = decode_ilink(com_tx, il.ACCEPT_FCU)
    res["com_tx_frames"] = {
        "CMD": sum(1 for f in com_frames if f["type"] == il.TYPE_CMD),
        "STAT": sum(1 for f in com_frames if f["type"] == il.TYPE_STAT),
        "CMD_not_delivered": [{"t_s": round(f["t_first"] / F_CPU, 4), "seq": f["payload"][0], "cmd": f["payload"][1],
                               "why": sorted(f["src"] - {"c"})} for f in com_frames
                              if f["type"] == il.TYPE_CMD and f["src"] != {"c"}],
        "STAT_not_delivered": [{"t_s": round(f["t_first"] / F_CPU, 4), "why": sorted(f["src"] - {"c"})}
                               for f in com_frames if f["type"] == il.TYPE_STAT and f["src"] != {"c"}]}
    seen = {f["t_first"] for f in cmds}
    res["com_tx_frames"]["CMD_delivered_not_decoded"] = [
        {"t_s": round(f["t_first"] / F_CPU, 4), "seq": f["payload"][0], "cmd": f["payload"][1]}
        for f in com_frames if f["type"] == il.TYPE_CMD and f["src"] == {"c"} and f["t_first"] not in seen]
    lat_cmd = []
    for fr in cmds:
        seq = fr["payload"][0]
        j = bisect.bisect_right(rises, fr["t_last_rx"])
        if j >= len(ticks):
            continue
        first = ticks[j]
        prev = next((g for g in reversed(frames) if g["last"] < fr["first"]), None)
        front = prev is not None and prev["type"] == il.TYPE_STAT and fr["t_first"] - prev["t_last_in"] <= 2 * il_byte
        got = None
        for t in ticks[j:j + 12]:
            r = rows[t]
            if r.get("n_cmds") and ((seq - r["seq_first"]) & 0xFF) <= ((r["seq_last"] - r["seq_first"]) & 0xFF):
                got = t
                break
        lat_cmd.append({"seq": seq, "cmd": fr["payload"][1], "t_last_rx_s": fr["t_last_rx"] / F_CPU, "first_tick": first,
                        "rise_after_us": round((rows[first]["rise"] - fr["t_last_rx"]) / CYC_US, 1),
                        "supervisor_tick": got, "latency_ticks": None if got is None else got - first,
                        "stat_in_front": front})
    st_t = [f["t_last_rx"] for f in stats_fr]
    gaps = [(b - a) / F_CPU for a, b in zip(st_t, st_t[1:])]
    res["il_stat_at_fcu"] = {"frames": len(stats_fr), "first_s": round(st_t[0] / F_CPU, 4) if st_t else None,
                             "max_gap_s": round(max(gaps), 3) if gaps else None,
                             "min_gap_s": round(min(gaps), 3) if gaps else None,
                             "boot_flag_first": bool(stats_fr) and bool(il.Stat.unpack(stats_fr[0]["payload"]).flags & il.STAT_BOOT)}
    res["il_cmd"] = {"frames": len(cmds), "il_stat_frames": len(stats_fr),
                     "latency_ticks": stats([x["latency_ticks"] for x in lat_cmd if x["latency_ticks"] is not None], 0),
                     "not_at_supervisor": [x for x in lat_cmd if x["latency_ticks"] is None],
                     "nonzero": [x for x in lat_cmd if x["latency_ticks"] not in (None, 0)],
                     "with_stat_in_front": sum(1 for x in lat_cmd if x["stat_in_front"])}

    # ---- SNAP frames (FCU -> COM), TELEM on the radio UART (lora_uart), the reply path on the wires
    snaps = decode_ilink([(t, t, b, "f") for t, b in ev.S], il.ACCEPT_COM)
    res["snap_frames"] = sum(1 for f in snaps if f["type"] == il.TYPE_SNAP)
    res["fcu_to_com_other_frames"] = sum(1 for f in decode_ilink([(t, t, b, "f") for t, b in ev.S]) if f["type"] != il.TYPE_SNAP)
    tdec = lp.FrameDecoder(vehicle_id=None, accept_types=(lp.TYPE_TELEM,))
    telems = []
    for i, (t, b) in enumerate(ev.U):
        for fr in tdec.feed(bytes([b])):
            tl = lp.decode_telemetry(fr)
            telems.append({"t_first": ev.U[max(0, i - lp.TELEM_FRAME_BYTES + 1)][0], "t_end": t, "seq": fr.seq,
                           "state": tl.state, "ack": tl.ack_seq, "t_ms": tl.t_ms})
    res["telem_on_radio_uart"] = len(telems)
    snapf = [f for f in snaps if f["type"] == il.TYPE_SNAP]
    fcu_part, com_part, both = [], [], []
    for i, fr in enumerate(cmds):
        near = [abs(fr["t_first"] - cmds[j]["t_first"]) for j in (i - 1, i + 1) if 0 <= j < len(cmds)]
        if fr["src"] != {"c"} or (near and min(near) < BURST_GAP_US * CYC_US):
            continue
        sp_ = next((g for g in snapf if g["payload"][0] == fr["payload"][0] and g["t_first"] > fr["t_last_rx"]), None)
        if sp_ is None:
            continue
        sp_end = sp_["t_last_in"] + il_byte
        fcu_part.append((sp_end - fr["t_last_rx"]) / CYC_US)
        tl = next((x for x in telems if x["seq"] == sp_["payload"][0] and x["t_first"] > sp_end), None)
        if tl is not None and tl["t_first"] - sp_end < 200_000 * CYC_US:
            com_part.append((tl["t_first"] - sp_end) / CYC_US)
            both.append((tl["t_first"] - fr["t_first"]) / CYC_US)
    res["wire_reply_path_us"] = {"il_cmd_end_to_snap_end": stats(fcu_part, 0),
                                 "snap_end_to_telem_first_byte_on_radio_uart": stats(com_part, 0),
                                 "il_cmd_first_byte_to_telem_first_byte": stats(both, 0)}
    texts = []
    for t0, _, rawf in ev.F:
        for typ, payload in dl.frames(rawf):
            if typ == "T":
                texts.append({"t_s": round(t0 / F_CPU, 4), "text": payload.decode("latin-1")})
    res["fcu_text_records"] = texts

    # ---- stack (lowest SP the harness saw)
    res["stack_depth"] = {f"{c}_bytes": RAMEND - ev.M[f"{c}_sp_min"] for c in ("fcu", "com", "nav")}
    res["stack_depth"].update({f"{c}_bytes_after_boot": RAMEND - ev.M[f"{c}_sp_min_run"] for c in ("fcu", "com", "nav")})

    # ---- COM
    com = {}
    for ep, cr in sorted(ev.CR.items()):
        if not cr.get("valid"):
            com[ep] = {"valid": False}
            continue
        uls = sorted([u for u in ev.UL if u[0] == ep], key=lambda u: u[1])
        recs = []
        for (_, u, seq, cmd, flags, num, t_up, t_q, t_sn, t_tl) in uls:
            rec = {"u": u, "seq": seq, "cmd": cmd, "flags": flags}
            if flags & 1:
                rec["t_up"] = t_up
            if flags & 1 and flags & 2:
                rec["up_to_q_us"] = t_q - t_up
            if flags & 2 and flags & 4:
                rec["q_to_snap_us"] = t_sn - t_q
            if flags & 4 and flags & 8:
                rec["snap_to_telem_us"] = t_tl - t_sn
            if flags & 1 and flags & 8:
                rec["up_to_telem_us"] = t_tl - t_up
            if flags & 2 and flags & 8:
                rec["q_to_telem_us"] = t_tl - t_q
            recs.append(rec)
        ups = [x.get("t_up") for x in recs]
        for i, rec in enumerate(recs):
            near = [abs(ups[i] - ups[j]) for j in (i - 1, i + 1) if 0 <= j < len(recs) and ups[i] is not None
                    and ups[j] is not None]
            rec["burst"] = bool(near) and min(near) < BURST_GAP_US
        ep_start = (ev.M.get("com_off", 0) if ep == 0 else (fault_off or 0)) / CYC_US
        fcu_end_us = ev.M.get("fcu_cycle", 0) / CYC_US - ep_start
        for rec in recs:
            rec["at_run_end"] = rec.get("t_up") is not None and rec["t_up"] > fcu_end_us - RUN_END_MARGIN_US
        normal = [x for x in recs if not x["burst"] and not x["at_run_end"]]
        com[ep] = {
            "valid": True, "radio": cr.get("radio"), "done": cr.get("done"), "ticks": cr.get("ticks"),
            "pass_max_us": cr.get("pass_max_us"), "pass_max_per_s_us": ev.CS.get(ep),
            "sections_max_us": [cr.get(f"sec{i}") for i in range(1, 6)], "stack_free_min_report": cr.get("stack_free_min"),
            "counters": {k2: cr.get(k2) for k2 in ("uplinks", "cmd_forwarded", "il_tx_dropped", "il_wrong_type",
                                                   "snaps_accepted", "replies", "replies_superseded", "replies_refused",
                                                   "replies_expired", "stats_sent", "stats_piggyback", "stats_quiet",
                                                   "il_rx_bytes", "il_rx_overflow", "il_rx_errors", "il_bad_crc",
                                                   "il_bad_len", "il_bad_type", "il_skipped", "radio_tx_frames",
                                                   "radio_tx_timeouts", "telem_bad", "telem_out_of_window",
                                                   "telem_unmatched", "nonce_sent", "events_lost")},
            "ground_state": dl.state_name(cr["ground_state"]) if cr.get("ground_state", 255) < 6 else None,
            "reset_cause": cr.get("reset_cause"), "reply_window_ticks": cr.get("reply_window_ticks"),
            "uplinks": len(recs), "burst_uplinks": sum(1 for x in recs if x["burst"]),
            "reply_us_normal": {key: stats([x[key] for x in normal if key in x], 0)
                                for key in ("up_to_q_us", "q_to_snap_us", "snap_to_telem_us", "up_to_telem_us",
                                            "q_to_telem_us")},
            "reply_us_burst": {key: stats([x[key] for x in recs if x["burst"] and key in x], 0)
                               for key in ("up_to_telem_us", "q_to_telem_us")},
            "unanswered_normal": [x["u"] for x in normal if "up_to_telem_us" not in x],
            "records": recs,
        }
    res["com"] = com
    raw["up_to_telem_us"] = [x["up_to_telem_us"] for c in com.values() if c.get("valid") for x in c["records"]
                             if not x["burst"] and "up_to_telem_us" in x and not x.get("at_run_end")]
    for core, key in (("C", "com_isr"), ("F", "fcu_isr"), ("N", "nav_isr")):
        res[key] = {str(v): {"runs": n, "min": mn, "max": mx} for c, v, n, tot, mn, mx in ev.I if c == core}
    res["wire"] = {k2: ev.M.get(k2) for k2 in ("c2f_bytes", "f2c_bytes", "c2f_cut", "f2c_cut", "c2f_babbled", "c2f_rxoff",
                                                "babble_bytes", "rx_delivered", "rx_unmatched", "rx_queue_left",
                                                "rx_queue_overflow", "com_u1_bytes", "console_bytes", "nav_console_bytes")}
    res["decoded_c2f_frames"] = {"CMD": len(cmds), "STAT": len(stats_fr), "junk_bytes": sum(1 for c in cls if c == "junk")}
    res["transitions"] = transitions(ev, lay)
    for tr in res["transitions"]:
        tr["t_s"] = round(rows[tr["tick"]]["rise"] / F_CPU, 4) if tr["tick"] in rows else None
    res["harness_events"] = [{"t_s": round(t / F_CPU, 4), "event": n, "args": a} for t, n, a in ev.E]
    if s.kind == "fault":
        res["fault"] = analyse_fault(s, res, rows, ticks, good, lat_cmd, xs, nts, fault_on, fault_off, nav_reset_t, k,
                                     window, first_valid)
    res["_raw"] = raw
    return res


def analyse_fault(s: Scn, res: dict, rows: dict, ticks: list, good: list, lat_cmd: list, xs: list, nts: list, t_on: int,
                  t_off: int | None, nav_reset_t: int | None, k: dict, window: list, first_valid: int | None) -> dict:
    out: dict = {"fault": s.fault_name, "at_s": round(t_on / F_CPU, 4) if t_on else None,
                 "until_s": round(t_off / F_CPU, 4) if t_off else None}
    tick_c = k["tick_us"] * CYC_US
    limit = CRIT_LIMIT * tick_c
    trs = [tr for tr in res["transitions"] if tr["t_s"] is not None and t_on is not None and tr["t_s"] * F_CPU >= t_on]
    out["transitions_after_fault_start"] = trs
    abort_ticks, rel_ticks, imu_ticks = k["link_abort_ticks"], k["abort_release_ticks"], k["imu_fail_ticks"]
    sup = [t for t in ticks if rows[t].get("n_cmds")]
    before = [t for t in sup if rows[t]["rise"] < t_on]
    after = [t for t in sup if rows[t]["rise"] > t_on]
    out["last_il_cmd_tick_before"] = before[-1] if before else None
    out["first_il_cmd_tick_after"] = after[0] if after else None
    out["link_gap_ticks"] = None if not before or not after else after[0] - before[-1]
    ab = next((tr for tr in trs if tr["to"] == "ABORT"), None)
    rl = next((tr for tr in trs if tr["to"] == "RELEASED"), None)
    post = [r for r in good if r["rise"] >= t_on]
    out["crit_max_all_us_from_fault"] = round(max((r["crit_all_cyc"] for r in post), default=0) / CYC_US, 1)
    out["crit_within_80_percent_from_fault"] = all(r["crit_all_cyc"] <= limit for r in post)
    if s.fault == "cut":
        out["abort"], out["released"] = ab, rl
        out["abort_after_last_frame_ticks"] = None if (ab is None or not before) else ab["tick"] - before[-1]
        out["release_after_abort_ticks"] = None if (ab is None or rl is None) else rl["tick"] - ab["tick"]
        out["last_frame_tick_seen_by_supervisor_at_abort"] = rows[ab["tick"]].get("last_frame_tick") if ab else None
        out["expect"] = (f"ABORT (I1) {abort_ticks} ticks = {abort_ticks / k['loop_hz']:.1f} s after the last IL_CMD, "
                         f"RELEASED {rel_ticks} ticks = {rel_ticks / k['loop_hz']:.1f} s later")
        out["pass"] = bool(ab and rl and ab["cause_code"] == lp.R_I1_LINK_LOSS
                           and out["abort_after_last_frame_ticks"] == abort_ticks
                           and out["last_frame_tick_seen_by_supervisor_at_abort"] == before[-1]
                           and out["release_after_abort_ticks"] == rel_ticks
                           and rl["cause_code"] == dl.CAUSE_RELEASE_TIMER
                           and not any(tr["tick"] < ab["tick"] for tr in trs)
                           and res["loop"]["fcu_timeout_ticks_after_first_frame"] == 0)
    elif s.fault == "reset":
        ep1 = res["com"].get(1, {})
        out["com_after_reset"] = {"report_valid": ep1.get("valid"), "reset_cause": ep1.get("reset_cause"),
                                  "uplinks": ep1.get("uplinks"), "counters": ep1.get("counters")}
        out["fcu_text_after_reset"] = [x for x in res["fcu_text_records"] if x["t_s"] * F_CPU >= t_on]
        out["expect"] = f"no state change; the link resumes; gap < I1 ({abort_ticks} ticks)"
        out["pass"] = bool(not trs and out["link_gap_ticks"] is not None and out["link_gap_ticks"] < abort_ticks
                           and res["final_state"] == "POWERED" and res["loop"]["fcu_timeout_ticks_after_first_frame"] == 0)
    elif s.fault == "babble":
        pre = [r for r in good if r["rise"] < t_on]
        postb = [r for r in good if r["rise"] > t_off + k["loop_hz"] // 2 * tick_c]
        c0 = pre[-1]["sb"] if pre else {}
        c1 = postb[0]["sb"] if postb else {}
        delta = {key: (c1.get(key, 0) - c0.get(key, 0)) for key in ("overruns", "baro_late", "flog_dropped", "flood_ticks",
                                                                   "rx_overflow", "rx_errors", "il_bad", "bg_capped",
                                                                   "snap_dropped", "crit_over", "nl_bad", "nl_timeouts")}
        wmax = max(window, key=lambda r: r["crit_all_cyc"]) if window else None
        rx_isr = k["isr_fcu"]["ilink_rx"]["normal"]
        outside = [r for r in good if r["flight"] and not (t_on <= r["rise"] <= t_off + tick_c)]
        ref = max((r["crit_cyc"] - r["isr_cyc"] for r in outside), default=tick_c)
        bound = ref + math.ceil(tick_c / res["ilink_byte_cycles"]) * rx_isr
        out["during_babble"] = {
            "ticks": len(window),
            "crit_max_us": round(wmax["crit_cyc"] / CYC_US, 1) if wmax else None,
            "crit_all_max_us": round(wmax["crit_all_cyc"] / CYC_US, 1) if wmax else None,
            "crit_all_max_fraction": round(wmax["crit_all_cyc"] / tick_c, 4) if wmax else None,
            "crit_worst_tick": {"tick": wmax["tick"], "isr_us": round(wmax["isr_cyc"] / CYC_US, 1),
                                "isr_runs": wmax["nisr"], "bytes_in_crit": wmax["rx_in_crit"]} if wmax else None,
            "total_max_us": max((r["sb"]["total_us"] for r in window), default=None),
            "counter_deltas_over_the_babble": delta, "babble_bytes": res["wire"]["babble_bytes"],
            "com_bytes_replaced": res["wire"]["c2f_babbled"], "reference_no_isr_us": round(ref / CYC_US, 1),
            "no_isr_max_us": round(max((r["crit_cyc"] - r["isr_cyc"] for r in window), default=0) / CYC_US, 1),
            "bound_us": round(min(bound, tick_c) / CYC_US, 1)}
        out["il_cmds_decoded_not_at_supervisor"] = [x for x in lat_cmd if x["latency_ticks"] is None]
        out["il_cmds_replaced_by_the_babble"] = res["com_tx_frames"]["CMD_not_delivered"]
        out["first_il_cmd_after_babble"] = next(({"tick": x["supervisor_tick"], "latency_ticks": x["latency_ticks"],
                                                 "t_s": x["t_last_rx_s"]} for x in lat_cmd
                                                if x["t_last_rx_s"] * F_CPU > t_off and x["supervisor_tick"]), None)
        out["expect"] = ("bounded: time-critical part below the tick and below the normal worst case + the RX interrupts of "
                         "a continuous byte stream, no navigation frame lost, no dropped flight record, no state change, "
                         "flood detected, the link resumes")
        out["pass"] = bool(wmax and wmax["crit_cyc"] <= min(bound, tick_c) and delta["nl_bad"] == 0
                           and delta["nl_timeouts"] == 0 and delta["flog_dropped"] == 0 and delta["flood_ticks"] > 0
                           and not trs and out["first_il_cmd_after_babble"] is not None
                           and res["loop"]["fcu_timeout_ticks_after_first_frame"] == 0)
        out["crit_within_80_percent"] = bool(wmax and wmax["crit_all_cyc"] <= limit)
    elif s.fault in ("nav_hold", "nav_pulse"):
        t_reset = nav_reset_t if nav_reset_t is not None else t_on
        valid_before = [r for r in good if r["sb"]["nav_ok"] and r["rise"] < t_reset]
        last_valid = valid_before[-1]["tick"] if valid_before else None
        out["nav_reset_at_s"] = round(t_reset / F_CPU, 4)
        out["last_valid_frame_tick"] = last_valid
        out["abort"], out["released"] = ab, rl
        out["abort_after_last_valid_frame_ticks"] = None if (ab is None or last_valid is None) else ab["tick"] - last_valid
        out["release_after_abort_ticks"] = None if (ab is None or rl is None) else rl["tick"] - ab["tick"]
        seq = [r for r in good if last_valid is not None and r["tick"] > last_valid]
        sp = [(b["rise"] - a["rise"]) / CYC_US for a, b in zip([rows[last_valid]] + seq, seq)] if last_valid else []
        out["fallback"] = {"ticks_without_valid_frame_until_abort": sum(1 for r in seq if ab and r["tick"] <= ab["tick"]
                                                                        and not r["sb"]["nav_ok"]),
                           "first_spacing_us": round(sp[0], 1) if sp else None,
                           "timeout_ticks_spacing_us": stats([d for r, d in zip(seq, sp) if r["sb"]["wait"] == WAIT_TIMEOUT][1:]),
                           "frame_timeout_us": k["frame_timeout_us"]}
        resp_x = [x for x in xs if x["ep"] >= 1 and x["nav_sent"] is not None]
        out["frames_after_reboot"] = len(resp_x)
        if resp_x:
            fl = resp_x[0]["nav_sent"][2][4]
            out["first_frame_after_reboot"] = {"t_s": round(resp_x[0]["t_last"] / F_CPU, 4), "flags": fl,
                                               "imu_ok": bool(fl & nlp.F_IMU_OK), "est_valid": bool(fl & nlp.F_EST_VALID),
                                               "boot": bool(fl & nlp.F_BOOT),
                                               "silence_ms": round((resp_x[0]["t_last"] - t_reset) / F_CPU * 1000, 2)}
        before_abort = [x for x in resp_x if ab and x["t_last"] < rows[ab["tick"]]["rise"]]
        out["frames_after_reboot_before_abort"] = len(before_abort)
        out["frames_after_reboot_before_abort_with_imu_ok_without_est_valid"] = sum(
            1 for x in before_abort if (x["nav_sent"][2][4] & nlp.F_IMU_OK) and not (x["nav_sent"][2][4] & nlp.F_EST_VALID))
        out["frames_after_reboot_with_est_valid"] = sum(1 for x in resp_x if x["nav_sent"][2][4] & nlp.F_EST_VALID)
        out["resyncs_delta"] = res["nlink"]["fcu_counters_since_first_frame"]["nl_resyncs"]
        ft_ok = (out["fallback"]["first_spacing_us"] is not None
                 and abs(out["fallback"]["first_spacing_us"] - k["frame_timeout_us"]) <= 60)
        common = bool(ab and rl and ab["cause_code"] == lp.R_I9_IMU and out["abort_after_last_valid_frame_ticks"] == imu_ticks
                      and out["release_after_abort_ticks"] == rel_ticks and rl["cause_code"] == dl.CAUSE_RELEASE_TIMER
                      and not any(tr["tick"] < ab["tick"] for tr in trs) and ft_ok
                      and res["nlink_integrity"]["fcu_wrong_data_accepted_n"] == 0)
        if s.fault == "nav_hold":
            tsp = out["fallback"]["timeout_ticks_spacing_us"]
            out["expect"] = (f"fallback ticks {k['frame_timeout_us']} us after the last frame tick then every "
                             f"{k['tick_us']} us (Timer2), ABORT (I9) {imu_ticks} ticks after the last valid frame, "
                             f"RELEASED {rel_ticks} ticks later")
            # Fallback ticks start from Timer2, but the idle wait finishes the ilink byte it is decoding and the RX
            # interrupts queued at that instant first (main.c ilink_rx_idle), so a fallback tick can start a few tens
            # of us late and the spacing to the next one carries that jitter (measured 4027-4041 us).  The same +-60 us
            # as the first-spacing check above: 1.5 % of a tick, and fallback ticks only run until the I9 ABORT, whose
            # timing is counted in ticks, not microseconds.
            out["pass"] = bool(common and tsp.get("n") and k["tick_us"] - 20 <= tsp["min"] and tsp["max"] <= k["tick_us"] + 60)
        else:
            out["expect"] = (f"the NAV reset in an exchange (partial exchange discarded), reboots after the reset time-out "
                             f"and resumes frames with IMU_OK but no EST_VALID; ABORT (I9) {imu_ticks} ticks after the last "
                             f"valid frame, RELEASED {rel_ticks} ticks later")
            out["pass"] = bool(common and out["frames_after_reboot_before_abort_with_imu_ok_without_est_valid"] >= 1
                               and out["frames_after_reboot_with_est_valid"] == 0 and out["resyncs_delta"] >= 1)
    else:                                                  # nl_*
        dmg = [x for x in xs if x["faults"]]
        mosi_dmg = [x for x in dmg if x["mosi_damaged"]]
        miso_dmg = [x for x in dmg if x["miso_damaged"]]
        win_ticks = [r for r in good if t_on <= r["rise"] <= t_off + 2 * tick_c]
        no_frame = [r for r in win_ticks if not r["sb"]["nav_ok"]]
        consec, run_ = 0, 0
        for r in win_ticks:
            run_ = run_ + 1 if not r["sb"]["nav_ok"] else 0
            consec = max(consec, run_)
        sp = [(b["rise"] - a["rise"]) / CYC_US for a, b in zip(win_ticks, win_ticks[1:])]
        mosi_valid_by_python = sum(1 for x in mosi_dmg if x["fcu_rx_valid"])
        miso_valid_by_python = sum(1 for x in miso_dmg if x["miso_valid"] and x["miso"][:10] != x["miso_clean10"])
        c0 = next((r["sb"] for r in reversed(good) if r["rise"] < t_on), {})
        c1 = next((r["sb"] for r in good if r["rise"] > t_off + 4 * tick_c), {})
        out["exchanges_damaged"] = {"total": len(dmg), "mosi": len(mosi_dmg), "miso": len(miso_dmg),
                                    "mosi_still_valid_by_python": mosi_valid_by_python,
                                    "miso_still_valid_by_python": miso_valid_by_python}
        out["fcu_ticks_without_valid_frame"] = len(no_frame)
        out["fcu_longest_run_without_valid_frame"] = consec
        out["fcu_counter_deltas"] = {key: c1.get(key, 0) - c0.get(key, 0) for key in
                                     ("nl_frames", "nl_bad", "nl_timeouts", "nl_seq_lost", "nl_resyncs", "nl_overruns")}
        out["fcu_tick_spacing_us"] = stats(sp)
        out["nav_refused_damaged_replies"] = sum(1 for x in miso_dmg if not x["miso_valid"])
        out["nav_accepted_damaged_replies"] = res["nlink_integrity"]["nav_accepted_damaged_replies"]
        out["resumed"] = bool(c1) and all(r["sb"]["nav_ok"] for r in good if r["rise"] > t_off + 4 * tick_c)
        out["expect"] = ("every damaged exchange refused (nothing wrong accepted, FCU and NAV), one FCU tick without a valid "
                         "frame per damaged exchange, FCU tick spacing <= the frame timeout + 60 us, no state change, time-critical "
                         "part <= 80 %, frames resume")
        exact_count = len(no_frame) == len(mosi_dmg) - mosi_valid_by_python if s.fault != "nl_noise" else \
            len(no_frame) <= len(mosi_dmg)
        out["pass"] = bool(dmg and res["nlink_integrity"]["fcu_wrong_data_accepted_n"] == 0
                           and res["nlink_integrity"]["nav_accepted_damaged_replies"] == 0
                           and res["nlink_integrity"]["nav_accept_vs_python_mismatch_n"] == 0
                           and exact_count and consec < imu_ticks and not trs
                           # a damaged exchange makes the FCU run that tick from Timer2 after the frame timeout; like the
                           # NAV-hold fallback it can start a few tens of us late (the idle wait finishes its ilink byte and
                           # the queued RX interrupts first), hence the same +60 us (measured nl_drop 4620 / 4627 us)
                           and (sp and max(sp) <= k["frame_timeout_us"] + 60)
                           and out["crit_within_80_percent_from_fault"] and out["resumed"]
                           and out["nav_refused_damaged_replies"] == len(miso_dmg) - miso_valid_by_python)
    return out


# ================================================================== main
def ms5611_d1_phase(gen: dict) -> int:
    """hal/ms5611.h MS5611_D1_PHASE = ceil(MS5611_D1_CONV_MAX_US / FC_TICK_US), from the same headers"""
    h = header_defines(os.path.join(FW, "src", "hal", "ms5611.h"), ("MS5611_D1_CONV_MAX_US",))
    return -(-h["MS5611_D1_CONV_MAX_US"] // gen["FC_TICK_US"])


def baro_twi_phases(gen: dict) -> dict:
    """phase (tick % FC_BARO_DIV) -> TWI interrupts of the barometer script the NAV starts in it, for the driver the
    default NAV envs build (run_nav_simbench.py baro_sensor(): params_gen.h FC_BARO_SENSOR), from the driver's header:
    BMP280 (hal/bmp280.h): the forced write in BMP280_START_PHASE, the data read BMP280_MEAS_TICKS =
    ceil(BMP280_MEAS_MAX_US / FC_TICK_US) ticks later; MS5611 (hal/ms5611.h): the same script in phases 0 and
    MS5611_D1_PHASE"""
    sensor = rnv.baro_sensor(FW)
    scripts = rnv.BARO_TWI_SCRIPTS[sensor]
    if sensor == "BMP280":
        h = header_defines(os.path.join(FW, "src", "hal", "bmp280.h"), ("BMP280_MEAS_MAX_US", "BMP280_START_PHASE"))
        start = h["BMP280_START_PHASE"]
        read = (start + -(-h["BMP280_MEAS_MAX_US"] // gen["FC_TICK_US"])) % gen["FC_BARO_DIV"]
        return {start: min(scripts.values()), read: max(scripts.values())}
    n = max(scripts.values())
    return {0: n, ms5611_d1_phase(gen): n}


def consts(builds: dict) -> dict:
    gen = header_defines(os.path.join(FW, "src", "params_gen.h"),
                         ("FC_LOOP_HZ", "FC_TICK_US", "ONB_LINK_ABORT_TICKS", "ONB_ABORT_RELEASE_TICKS", "ILINK_BAUD",
                          "ONB_IMU_FAIL_TICKS", "FC_IMU_LEAD_COUNTS", "FC_T2_PRESCALER", "FC_BARO_DIV",
                          "NLINK_FRAME_TIMEOUT_US", "NLINK_XFER_US"))
    isr_f = rsb.isr_table(builds["fcu"]["elf"])
    isr_n = rnv.isr_table(builds["nav"]["elf"])
    pwm_addr, outs = pwm_registers()
    twi = isr_n["twi"]["longest"]
    twi_ph = baro_twi_phases(gen)
    k = {"loop_hz": gen["FC_LOOP_HZ"], "tick_us": gen["FC_TICK_US"], "link_abort_ticks": gen["ONB_LINK_ABORT_TICKS"],
         "abort_release_ticks": gen["ONB_ABORT_RELEASE_TICKS"], "ilink_baud": gen["ILINK_BAUD"],
         "imu_fail_ticks": gen["ONB_IMU_FAIL_TICKS"], "baro_div": gen["FC_BARO_DIV"], "baro_sensor": rnv.baro_sensor(FW),
         "twi_phase_interrupts": twi_ph, "twi_phase_cycles": {ph: n * twi for ph, n in twi_ph.items()},
         "frame_timeout_us": gen["NLINK_FRAME_TIMEOUT_US"], "nlink_xfer_us": gen["NLINK_XFER_US"],
         "imu_lead_counts": gen["FC_IMU_LEAD_COUNTS"], "imu_lead_cycles": gen["FC_IMU_LEAD_COUNTS"] * gen["FC_T2_PRESCALER"],
         "isr_fcu": isr_f, "isr_nav": isr_n,
         "twi_script_interrupts": max(twi_ph.values()), "twi_isr_cycles": twi,
         "twi_script_cycles": max(twi_ph.values()) * twi,
         "mpu_burst_interrupts": MPU_BURST_INTERRUPTS,
         "mpu_burst_isr_cycles": MPU_BURST_INTERRUPTS * twi + isr_n["imu_lead_timer0"]["longest"],
         "nav_isr_stack_not_exercised_bytes": max(isr_n["twi"].get("stack_bytes", 0),
                                                  isr_n["imu_lead_timer0"].get("stack_bytes", 0)),
         "pwm_ocr_high_addresses": pwm_addr, "pwm_outputs": outs,
         "vane_channels": [outs[f"VANE{i}"] for i in range(1, 5)], "esc_channel": outs["ESC"]}
    return k


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--sweep", type=int, default=40, help="start-delay pairs per backend (default 40)")
    ap.add_argument("--jobs", type=int, default=max(1, min(12, (os.cpu_count() or 2) - 2)))
    ap.add_argument("--only", choices=("normal", "faults"))
    ap.add_argument("--reuse", action="store_true",
                    help="analyse the event files already in tools/cosim/build/runs instead of running those scenarios again")
    a = ap.parse_args(argv)
    t_start = time.time()
    envs = list(COSIM_ENVS.values()) + list(FLIGHT_ENVS.values())
    print("building", ", ".join(envs), "...")
    benv = pio_build(envs)
    builds = {r: benv[e] for r, e in FLIGHT_ENVS.items()}
    cos = {r: benv[e] for r, e in COSIM_ENVS.items()}
    lay = layout_probe("fcu", LAYOUT_FCU, ["core/onboard.h", "core/nlink_proto.h", "simbench.h"],
                       ["-DFC_ROLE_FCU", "-DILINK_USART=1", "-DFC_SIMBENCH", "-DFC_COSIM"])
    layn = layout_probe("nav", LAYOUT_NAV, ["nav/nav_simbench.h"], ["-DFC_ROLE_NAV", "-DFC_SIMBENCH", "-DFC_COSIM"])
    sb = lay["sb_tick_t"]["fields"]
    if not (sb["seq_first"][0] == sb["n_cmds"][0] + 1 and sb["seq_last"][0] == sb["n_cmds"][0] + 2):
        raise SystemExit("sb_tick_t: n_cmds / seq_first / seq_last are not adjacent")
    syms = nm_symbols(cos["fcu"]["elf"])
    sbn = [n for n in syms if re.fullmatch(r"sb\.\d+", n) and syms[n][1] == lay["sb_tick_t"]["size"]]
    if len(sbn) != 1 or any(n not in syms for n in ("g_so", "g_sup", "g_nav")):
        raise SystemExit(f"fcu_cosim: sb / g_so / g_sup / g_nav not found by avr-nm ({sbn})")
    for n, t in (("g_so", "onb_outputs_t"), ("g_sup", "onboard_t"), ("g_nav", "nl_nav_t")):
        if syms[n][1] != lay[t]["size"]:
            raise SystemExit(f"fcu_cosim: {n} is {syms[n][1]} bytes, the layout probe says {lay[t]['size']}")
    nsyms = nm_symbols(cos["nav"]["elf"])
    nsb = [n for n in nsyms if re.fullmatch(r"sb\.\d+", n) and nsyms[n][1] == layn["nav_sb_tick_t"]["size"]]
    if len(nsb) != 1:
        raise SystemExit(f"nav_cosim: sb not found by avr-nm ({nsb})")
    sb_a, so_a, sup_a, nav_a = (syms[n][0] - DATA_BASE for n in (sbn[0], "g_so", "g_sup", "g_nav"))
    k = consts(builds)
    reads = {"rise": f"{sb_a:#x}:{lay['sb_tick_t']['size']}",
             "fall": f"{sb_a + sb['n_cmds'][0]:#x}:3,{sup_a + lay['onboard_t']['fields']['last_frame_tick'][0]:#x}:4,"
                     f"{nav_a:#x}:{lay['nl_nav_t']['size']}",
             "fallchg": f"{so_a:#x}:{lay['onb_outputs_t']['size']}",
             "navrise": f"{nsyms[nsb[0]][0] - DATA_BASE:#x}:{layn['nav_sb_tick_t']['size']}",
             "pwm": ",".join(f"{x:#x}" for x in k["pwm_ocr_high_addresses"])}
    report_addr = {}
    for b in BACKENDS:
        cs = nm_symbols(cos[b]["elf"])
        if "g_cosim_report" not in cs or cs["g_cosim_report"][1] != 908:
            raise SystemExit(f"{COSIM_ENVS[b]}: g_cosim_report missing or not 908 bytes")
        report_addr[b] = f"{cs['g_cosim_report'][0] - DATA_BASE:#x}"
    exe, hw = build_harness()
    k["isr_cost"] = isr_cost_probe()
    k["spi_probe"] = spi_probe()
    if k["spi_probe"].get("unexpected"):
        raise SystemExit(f"spi_probe: simavr's SPI does not behave as cosim.c assumes: {k['spi_probe']}")
    ic = k["isr_cost"]
    print(f"simavr charges {ic['resp0']['cycles_per_interrupt']} cycles per interrupt; with the harness's response "
          f"correction {ic[f'resp{ISR_RESP}']['cycles_per_interrupt']} (chip {CHIP_ISR_OVERHEAD})")
    print(f"simavr SPI: {k['spi_probe']['facts']}")
    hexes = {r: cos[r]["hex"] for r in ("fcu", "nav")} | {b: cos[b]["hex"] for b in BACKENDS}
    scns = scenarios(a.sweep, a.only)
    os.makedirs(RUNS, exist_ok=True)
    print(f"harness {os.path.relpath(exe, ROOT)} ({'warnings: ' + '; '.join(hw) if hw else 'no warnings'}); "
          f"{len(scns)} runs, {a.jobs} at a time")
    results: dict[str, dict] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.jobs) as pool:
        futs = [pool.submit((lambda s: (s, 0.0, "")) if a.reuse and os.path.exists(s.path) else
                            (lambda s: run_scn(exe, s, hexes, reads, report_addr)), s) for s in scns]
        for fu in concurrent.futures.as_completed(futs):
            s, wall, err = fu.result()
            r = analyse(parse_events(s.path), s, lay, layn, k)
            r["wall_s"] = round(wall, 1)
            if err:
                r["harness_stderr"] = err[-1500:]
            results[s.name] = r
            fc, nc, lt = r["fcu_crit"], r["nav_crit"], r["latency"]["imu_sample_to_vane_equiv_us"]
            print(f"  {s.name:<40} FCU {fc['max_all_us']:7.1f} us  NAV {nc['max_equiv_us'] or 0:7.1f} us  "
                  f"sample->vane max {lt.get('max')} us  late hdr {r['nlink']['reply_bytes_1_9_late']}  "
                  f"wrong {r['nlink_integrity']['fcu_wrong_data_accepted_n']}  CMD lat {r['il_cmd']['latency_ticks'].get('max')}"
                  f"  {wall:5.1f} s")
    report = summarize(results, scns, builds, cos, lay, layn, k, reads, report_addr)
    report["wall_s"] = round(time.time() - t_start, 1)
    os.makedirs(os.path.dirname(REPORT), exist_ok=True)
    with open(REPORT, "w", encoding="utf-8", newline="\n") as f:
        json.dump(report, f, indent=1)
    print_summary(report)
    print(f"-> {os.path.relpath(REPORT, ROOT)}")
    ok = all(c["pass"] for c in report["criteria"].values())
    print("co-simulation:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def summarize(results: dict, scns: list[Scn], builds: dict, cos: dict, lay: dict, layn: dict, k: dict, reads: dict,
              report_addr: dict) -> dict:
    tick_us = k["tick_us"]
    limit_us = CRIT_LIMIT * tick_us
    normal = [results[s.name] for s in scns if s.kind == "normal"]
    faults = [results[s.name] for s in scns if s.kind == "fault"]
    diag = [results[s.name] for s in scns if s.kind == "diag"]
    rep: dict = {"tool": "firmware/atmega128_fc/tools/run_cosim.py", "harness": "firmware/atmega128_fc/tools/cosim/cosim.c",
                 "tick_us": tick_us, "crit_limit_fraction": CRIT_LIMIT, "stack_min_free_bytes": STACK_MIN_FREE,
                 "constants": {kk: v for kk, v in k.items() if kk not in ("isr_fcu", "isr_nav", "isr_cost", "spi_probe")},
                 "isr_cycles_fcu_flight_elf": k["isr_fcu"], "isr_cycles_nav_flight_elf": k["isr_nav"],
                 "isr_cost_probe": k["isr_cost"], "spi_probe": k["spi_probe"],
                 "builds": {e: {kk: v for kk, v in b.items() if kk not in ("elf", "hex")}
                            for e, b in {**{FLIGHT_ENVS[x]: builds[x] for x in builds},
                                         **{COSIM_ENVS[x]: cos[x] for x in cos}}.items()},
                 "harness_reads": reads, "com_report_address": report_addr,
                 "layout": {t: {"size": v["size"], "fields": {n: o for n, (o, _) in v["fields"].items()}}
                            for t, v in {**lay, **layn}.items()},
                 "sweep": {"com_delays_us": sorted({s.delay_us for s in scns if s.kind == "normal"}),
                           "nav_delays_us": sorted({s.nav_delay_us for s in scns if s.kind == "normal"}),
                           "runs": len(normal)},
                 "analytic_additions": {
                     "nav_baro_sensor": k["baro_sensor"],
                     "nav_baro_twi_script_us": round(k["twi_script_cycles"] / CYC_US, 1),
                     "nav_baro_twi_us_by_phase": {str(ph): round(c / CYC_US, 1)
                                                  for ph, c in sorted(k["twi_phase_cycles"].items())},
                     "nav_mpu_burst_isr_us": round(k["mpu_burst_isr_cycles"] / CYC_US, 1),
                     "imu_lead_us": k["imu_lead_cycles"] / CYC_US,
                     "fcu_log_udre_cycles": k["isr_fcu"]["log_udre"]["normal"],
                     "_doc": "added to the measured co-simulation figures (not exercised by the cosim builds): the NAV's "
                             "barometer bus scripts (nav_baro_sensor; TWI interrupts x the longest path in the nav flight "
                             "ELF) to the NAV ticks of the phases that start them (nav_baro_twi_us_by_phase; they run during "
                             "the estimator: they delay the frame); a continuously busy "
                             "FCU log port (one UDRE interrupt per byte time) to the FCU's figures; the IMU lead at "
                             "FC_IMU_LEAD_COUNTS x FC_T2_PRESCALER before the NAV's Timer2 compare (simavr's Timer0 ignores "
                             "timer.c's TCNT0 preload; the Timer0 interrupt simavr runs at the tick and the NAV's simbench "
                             "console UDRE are taken out of the NAV's figures).  The MPU-6050 burst (19 TWI interrupts + the "
                             "lead ISR) runs in the lead time before the tick, outside every measured window"}}
    # ---- normal flight
    by_b = {}
    for b in BACKENDS:
        rr = [r for r in normal if r["backend"] == b]
        if not rr:
            continue
        wf = max(rr, key=lambda r: r["fcu_crit"]["max_all_us"])
        wn = max(rr, key=lambda r: r["nav_crit"]["max_equiv_us"] or 0)
        wl = max(rr, key=lambda r: r["latency"]["imu_sample_to_vane_equiv_us"].get("max", 0))
        cats: dict = {}
        for r in rr:
            for key, v in r["fcu_crit"]["max_by_coincidence"].items():
                if key not in cats or v["crit_all_us"] > cats[key]["crit_all_us"]:
                    cats[key] = {**v, "run": r["scenario"]}
        lats = [x for r in rr for x in [r["il_cmd"]["latency_ticks"]] if x.get("n")]
        reply = {}
        for key in ("up_to_q_us", "q_to_snap_us", "snap_to_telem_us", "up_to_telem_us", "q_to_telem_us"):
            reply[key] = stats([rec[key] for r in rr for ep, c in r["com"].items() if c.get("valid")
                                for rec in c["records"] if not rec["burst"] and key in rec], 0)
        burst = {key: stats([rec[key] for r in rr for ep, c in r["com"].items() if c.get("valid")
                             for rec in c["records"] if rec["burst"] and key in rec], 0)
                 for key in ("up_to_telem_us", "q_to_telem_us")}
        up_tl = [x for r in rr for x in r["_raw"]["up_to_telem_us"]]
        com0 = [r["com"][0] for r in rr if r["com"].get(0, {}).get("valid")]
        lat_all = [x for r in rr for x in r["_raw"]["latency_equiv_us"]]
        nl = [r["nlink"] for r in rr]
        by_b[b] = {
            "runs": len(rr),
            "fcu_crit_all_max_us": wf["fcu_crit"]["max_all_us"], "fcu_crit_all_max_fraction": wf["fcu_crit"]["max_all_fraction"],
            "fcu_crit_worst": {"run": wf["scenario"], **wf["fcu_crit"]["worst_tick"]},
            "fcu_crit_from_frame_max_us": max(r["fcu_crit"]["max_from_frame_us"] for r in rr),
            "fcu_crit_from_frame_raw_max_us": max(r["fcu_crit"]["max_from_frame_raw_us"] for r in rr),
            "fcu_simbench_late_ticks": sum(r["fcu_crit"]["simbench_late_ticks"] for r in rr),
            "fcu_simbench_late_detect_raw_max_us": max(r["fcu_crit"]["simbench_late_detect_raw_max_us"] for r in rr),
            "fcu_crit_from_tick_start_max_us": max(r["fcu_crit"]["max_from_tick_start_us"] for r in rr),
            "fcu_crit_no_isr_max_us": max(r["fcu_crit"]["no_isr_max_us"] for r in rr),
            "fcu_crit_all_us_distribution": stats([x for r in rr for x in r["_raw"]["fcu_crit_all_us"]]),
            "fcu_detect_us": {"max": max(r["fcu_crit"]["detect_us"].get("max", 0) for r in rr),
                              "p99_max": max(r["fcu_crit"]["detect_us"].get("p99", 0) for r in rr)},
            "fcu_isr_in_crit_max_us_by_vector": {v: max(r["fcu_crit"]["isr_in_crit_max_us_by_vector"][v] for r in rr)
                                                 for v in rr[0]["fcu_crit"]["isr_in_crit_max_us_by_vector"]},
            "fcu_isr_in_crit_before_first_frame_max_us_by_vector": {
                v: max(r["fcu_crit"]["isr_in_crit_before_first_frame_max_us_by_vector"][v] for r in rr)
                for v in rr[0]["fcu_crit"]["isr_in_crit_before_first_frame_max_us_by_vector"]},
            "fcu_crit_max_by_coincidence": dict(sorted(cats.items())),
            "most_ilink_bytes_in_one_crit_part": max(r["fcu_crit"]["most_ilink_bytes_in_one_crit_part"] for r in rr),
            "fcu_sections_flight_max_us": {n: max((r["fcu_sections_flight_max_us"][n] or 0) for r in rr) for n in FCU_SECTIONS},
            "nav_crit_equiv_max_us": wn["nav_crit"]["max_equiv_us"], "nav_crit_equiv_max_fraction": wn["nav_crit"]["max_equiv_fraction"],
            "nav_crit_worst": {"run": wn["scenario"], **(wn["nav_crit"]["worst_tick"] or {})},
            "nav_crit_measured_max_us": max(r["nav_crit"]["max_measured_us"] or 0 for r in rr),
            "nav_crit_bound_max_us": max(r["nav_crit"]["max_bound_us"] or 0 for r in rr),
            "nav_crit_flight_max_equiv_us": max(r["nav_crit"]["flight_max_equiv_us"] or 0 for r in rr),
            "nav_crit_pad_max_equiv_us": max(r["nav_crit"]["pad_max_equiv_us"] or 0 for r in rr),
            "nav_crit_flight_max_by_baro_phase_us": {ph: max(r["nav_crit"]["flight_max_by_baro_phase_us"].get(ph, 0) for r in rr)
                                                      for ph in sorted({p for r in rr for p in r["nav_crit"]["flight_max_by_baro_phase_us"]})},
            "nav_frame_ready_flight_with_twi_max_us": max(r["nav_crit"]["frame_ready_after_tick_us"]["flight_with_twi"].get("max", 0) for r in rr),
            "nav_frame_ready_largest_increase_us": max(r["nav_crit"]["frame_ready_largest_increase_tick_to_tick_us"] or 0 for r in rr),
            "nav_background_end_before_imu_lead_min_us": min(r["nav_crit"]["background_end_before_imu_lead_min_us"] for r in rr),
            "latency_imu_sample_to_vane_equiv_us": stats(lat_all),
            "latency_imu_sample_to_vane_equiv_hist_us": hist(lat_all, 100),
            "latency_imu_sample_to_vane_equiv_flight_us": stats([x for r in rr for x in r["_raw"]["latency_equiv_flight_us"]]),
            "boot_sync_detect_max_us": max((r["boot_sync"] or {}).get("detect_us", 0) for r in rr),
            "latency_worst": {"run": wl["scenario"], **(wl["latency"]["worst"] or {})},
            "latency_parts_us": {key: stats([x for r in rr for x in r["_raw"]["latency_parts"][key]])
                                 for key in ("nav_us", "detect_us", "fcu_us", "frame_to_vane_us")},
            "latency_nav_tick_to_vane_equiv_max_us": max(r["latency"]["nav_tick_to_vane_equiv_us"].get("max", 0) for r in rr),
            "latency_over_one_tick": sum(r["latency"]["over_one_tick"] for r in rr),
            "fcu_tick_spacing_us": {"min": min(r["fcu_tick_spacing_us"].get("min", 1e9) for r in rr),
                                    "max": max(r["fcu_tick_spacing_us"].get("max", 0) for r in rr)},
            "fcu_background_end_before_next_exchange_min_us": min(r["fcu_background_end_before_next_exchange_us"]["min"] for r in rr),
            "fcu_background_ticks_past_next_exchange": sum(r["fcu_background_end_before_next_exchange_us"]["ticks_past"] for r in rr),
            "nlink": {"exchanges": sum(x["exchanges"] for x in nl),
                      "fcu_ticks_without_valid_frame_after_the_first": sum(x["fcu_ticks_without_valid_frame_after_the_first"] for x in nl),
                      "fcu_counters_since_first_frame": {kk: sum(x["fcu_counters_since_first_frame"][kk] for x in nl)
                                                         for kk in nl[0]["fcu_counters_since_first_frame"]},
                      "nav_replies_refused_after_its_first_valid": sum(x["nav_replies_refused_after_its_first_valid"] for x in nl),
                      "reply_bytes_late": sum(x["reply_bytes_late"] for x in nl),
                      "reply_bytes_1_9_late": sum(x["reply_bytes_1_9_late"] for x in nl),
                      "write_collisions": sum(x["write_collisions"] for x in nl),
                      "reply_write_margin_min_cycles": min(x["reply_write_margin_min_cycles"] for x in nl
                                                           if x["reply_write_margin_min_cycles"] is not None),
                      "reply_write_margin_min_at": min(({"run": r["scenario"], "margin": r["nlink"]["reply_write_margin_min_cycles"],
                                                         **(r["nlink"]["reply_write_margin_min_at"] or {})} for r in rr),
                                                       key=lambda d: d["margin"] if d["margin"] is not None else 1e9),
                      "reply_write_by_interrupt_at_spif": merge_by_vec([x["reply_write_by_interrupt_at_spif"] for x in nl]),
                      "reply_write_after_spif_max_cycles": max(x["reply_write_after_spif_max_cycles"] or 0 for x in nl),
                      "delivery_lateness_max_cycles": max(x["delivery_lateness_max_cycles"] or 0 for x in nl),
                      "ss_low_us": {"min": min(x["ss_low_us"]["min"] for x in nl), "max": max(x["ss_low_us"]["max"] for x in nl)},
                      "fcu_late_ticks": sum(x["fcu_late_ticks"] for x in nl)},
            "nlink_integrity": {kk: sum(r["nlink_integrity"][kk] for r in rr) for kk in
                                ("fcu_frames_used", "fcu_used_newest_exchange", "fcu_used_older_exchange",
                                 "fcu_wrong_data_accepted_n", "fcu_decoder_vs_python_mismatch_n",
                                 "nav_accept_vs_python_mismatch_n", "nav_accepted_damaged_replies")},
            "missed_ticks": {"fcu_timeouts_after_first_frame": sum(r["loop"]["fcu_timeout_ticks_after_first_frame"] for r in rr),
                             "fcu_marker_jumps": sum(r["loop"]["fcu_marker_jumps"] or 0 for r in rr),
                             "nav_overruns": sum(sum(r["loop"]["nav_overruns"].values()) for r in rr),
                             "nav_marker_jumps": sum(r["loop"]["nav_marker_jumps"] or 0 for r in rr)},
            "loop_total_max_us": max(r["loop"]["total_max_us"] for r in rr),
            "wire_reply_path_us": {key: {"max": max(r["wire_reply_path_us"][key].get("max", 0) for r in rr),
                                         "n": sum(r["wire_reply_path_us"][key].get("n", 0) for r in rr)}
                                   for key in rr[0]["wire_reply_path_us"]},
            "il_cmd_frames": sum(r["il_cmd"]["frames"] for r in rr),
            "il_cmd_latency_ticks": {"min": min(x["min"] for x in lats), "max": max(x["max"] for x in lats),
                                     "n": sum(x["n"] for x in lats)} if lats else None,
            "il_cmd_with_stat_in_front": sum(r["il_cmd"]["with_stat_in_front"] for r in rr),
            "il_cmd_not_at_supervisor": sum(len(r["il_cmd"]["not_at_supervisor"]) for r in rr),
            "com_pass_max_us": max(c["pass_max_us"] for c in com0) if com0 else None,
            "com_sections_max_us": [max(c["sections_max_us"][i] or 0 for c in com0) for i in range(5)] if com0 else None,
            "reply_delay_us_normal": reply, "reply_delay_us_burst": burst,
            "reply_uplink_to_telem_us_distribution": stats(up_tl, 0),
            "reply_uplink_to_telem_hist_ms": hist([x / 1000 for x in up_tl], 5),
            "snaps_refused": sum(c["counters"]["replies_refused"] for c in com0),
            "telem_expired": sum(c["counters"]["replies_expired"] for c in com0),
            "replies_superseded": sum(c["counters"]["replies_superseded"] for c in com0),
            "telem_out_of_window_at_ground": sum(c["counters"]["telem_out_of_window"] for c in com0),
            "unanswered_normal_uplinks": sum(len(c["unanswered_normal"]) for c in com0),
            "ground_state_at_end": sorted({c["ground_state"] for c in com0 if c["ground_state"]}),
            "il_stat_frames_at_fcu": sum(r["il_cmd"]["il_stat_frames"] for r in rr),
            "il_stat_gap_at_fcu_s": {"min": min(r["il_stat_at_fcu"]["min_gap_s"] for r in rr),
                                     "max": max(r["il_stat_at_fcu"]["max_gap_s"] for r in rr)},
            "runs_whose_first_il_stat_at_the_fcu_lacks_boot": sum(1 for r in rr if not r["il_stat_at_fcu"]["boot_flag_first"]),
            "boot_stat_bytes_lost_before_fcu_rx_on": {r["scenario"]: r["wire"]["c2f_rxoff"] for r in rr if r["wire"]["c2f_rxoff"]},
            "fcu_errors": {kk: sum(r["fcu_counters"][kk] or 0 for r in rr) for kk in ("il_bad", "rx_overflow", "rx_errors")},
            "com_errors": {kk: sum(c["counters"][kk] or 0 for c in com0) for kk in ("il_rx_overflow", "il_rx_errors",
                                                                                   "il_bad_crc", "il_bad_len",
                                                                                   "il_bad_type", "il_wrong_type",
                                                                                   "il_tx_dropped", "events_lost")},
            "fcu_to_com_non_snap_frames": sum(r["fcu_to_com_other_frames"] for r in rr),
            "reached_powered_all": all(r["reached_powered"] for r in rr),
            "transitions": sorted({f"{t['from']}->{t['to']} ({t['cause']})" for r in rr for t in r["transitions"]}),
            "interlock_transitions": [{"run": r["scenario"], **t} for r in rr for t in r["transitions"]
                                      if not t["cause_code"] & dl.CAUSE_CMD],
            "stack_depth_max": {kk: max(r["stack_depth"][kk] for r in rr) for kk in rr[0]["stack_depth"]},
        }
    rep["normal"] = {"by_backend": by_b, "runs": {r["scenario"]: summary_run(r) for r in normal}}
    rep["faults"] = {r["scenario"]: {**r["fault"], "fcu_crit_all_max_us": r["fcu_crit"]["max_all_us"],
                                     "nav_crit_equiv_max_us": r["nav_crit"]["max_equiv_us"],
                                     "nlink": r["nlink"], "nlink_integrity": r["nlink_integrity"],
                                     "transitions": r["transitions"], "final_state": r["final_state"],
                                     "harness_events": r["harness_events"]} for r in faults}
    rep["diagnostics"] = {r["scenario"]: summary_run(r) for r in diag}

    # ---- worst-case composition from the co-simulation's own data (not an observation)
    nb = [r for r in normal + faults if "babble" not in r["scenario"]]
    bases = [dict(r["fcu_crit"]["no_isr_max_step5_idle"], run=r["scenario"]) for r in nb
             if r["fcu_crit"].get("no_isr_max_step5_idle")]
    if bases:
        base = max(bases, key=lambda x: x["no_isr_us"])
        rx_c = k["isr_fcu"]["ilink_rx"]["normal"]
        step5 = max((r["fcu_sections_flight_max_us"].get(FCU_SECTIONS[1]) or 0) for r in nb)
        burst_us, stat_us = 56 * rx_c / CYC_US, 42 * rx_c / CYC_US
        il_add = max(burst_us, step5 + stat_us)
        detect = max((r["fcu_crit"]["detect_us"].get("max", 0) for r in nb), default=0)
        comp = base["no_isr_us"] + il_add
        log_byte_us = nb[0]["log_byte_cycles"] / CYC_US
        logu = math.ceil(comp / log_byte_us) * k["isr_fcu"]["log_udre"]["normal"] / CYC_US
        rep["worst_case_composition_fcu"] = {
            "base_tick": base, "base_no_isr_us": base["no_isr_us"], "ilink_add_us": round(il_add, 1),
            "ilink_add_choice": "56-byte burst inside" if burst_us >= step5 + stat_us
            else f"step-5 IL_CMD decode ({step5} us, worst section seen) + 42-byte IL_STAT inside",
            "detect_max_us": detect, "log_udre_us": round(logu, 1),
            "every_interrupt_us": round(comp + logu, 1), "every_interrupt_fraction": round((comp + logu) / tick_us, 4),
            "_doc": "not observed: the worst non-interrupt FCU time-critical part (from the frame) of a flight tick whose step "
                    "5 decoded nothing + the worse of a 56-byte ilink burst arriving inside it (56 x the RX ISR) or the "
                    "step-5 IL_CMD decode + a 42-byte IL_STAT inside + a continuously busy log port"}

    # ---- stack: flight builds keep RAM - static - the deepest stack seen (NAV: + its deepest unexercised ISR frame)
    allr = normal + faults
    stack: dict = {}
    for role, key, add in (("fcu", "fcu_bytes", 0), ("nav", "nav_bytes", k["nav_isr_stack_not_exercised_bytes"])):
        d = max(r["stack_depth"][key] for r in allr)
        stack[role] = {"depth_bytes": d, "depth_after_boot_bytes": max(r["stack_depth"][key.replace("_bytes", "_bytes_after_boot")]
                                                                       for r in allr),
                       "isr_frame_not_exercised_bytes": add, "flight_env": FLIGHT_ENVS[role],
                       "flight_static_ram_bytes": builds[role]["static_ram_bytes"],
                       "cosim_static_ram_bytes": cos[role]["static_ram_bytes"],
                       "flight_free_bytes": RAM_BYTES - builds[role]["static_ram_bytes"] - d - add}
    stack["fcu"]["firmware_report_never_touched_min_cosim"] = min((r["fcu_stack_free_report_min"] or 9999) for r in allr)
    stack["nav"]["firmware_report_never_touched_min_cosim"] = min((r["nav_stack_free_report_min"] or 9999) for r in allr)
    for b in BACKENDS:
        rr = [r for r in allr if r["backend"] == b]
        if not rr:
            continue
        d = max(r["stack_depth"]["com_bytes"] for r in rr)
        stack[b] = {"depth_bytes": d, "depth_after_boot_bytes": max(r["stack_depth"]["com_bytes_after_boot"] for r in rr),
                    "flight_env": FLIGHT_ENVS[b], "flight_static_ram_bytes": builds[b]["static_ram_bytes"],
                    "cosim_static_ram_bytes": cos[b]["static_ram_bytes"],
                    "flight_free_bytes": RAM_BYTES - builds[b]["static_ram_bytes"] - d}
    stack["_doc"] = ("depth = RAMEND - the lowest SP the harness saw after any instruction of that chip in any run "
                     "(interrupt frames included); the flight build keeps 4096 - its static RAM - that depth (NAV: - the "
                     "deepest interrupt frame the cosim build never runs: TWI, Timer0 lead -> mpu6050_start_async, from the "
                     "nav flight ELF)")
    rep["stack"] = stack

    # ---- criteria
    cr: dict = {}

    def crit(name: str, ok: bool, value) -> None:
        cr[name] = {"pass": bool(ok), "value": value}

    if by_b:
        nbx = [r for r in normal + faults if "babble" not in r["scenario"]]
        wr = max(nbx, key=lambda r: r["fcu_crit"]["max_all_us"])
        crit(f"FCU time-critical part (frame -> end, every interrupt, busy log port) <= {CRIT_LIMIT:.0%} of the tick (every "
             f"normal, NAV-, nlink-, cut and reset run)", wr["fcu_crit"]["max_all_us"] <= limit_us,
             {"max_us": wr["fcu_crit"]["max_all_us"], "fraction": round(wr["fcu_crit"]["max_all_us"] / tick_us, 4),
              "expected_fraction": "~0.55-0.60 (architect's estimate for v0.6.3)",
              "limit_us": limit_us, "worst": {"run": wr["scenario"], **wr["fcu_crit"]["worst_tick"]},
              "normal_by_backend": {b: v["fcu_crit_all_max_us"] for b, v in by_b.items()}})
        wn = max(normal + faults, key=lambda r: r["nav_crit"]["max_bound_us"] or 0)
        crit(f"NAV time-critical part (tick -> frame delivered, + the {k['baro_sensor']} script in its phases, + the whole MPU-6050 "
             f"burst as an upper bound) <= {CRIT_LIMIT:.0%} of the tick (every run)",
             (wn["nav_crit"]["max_bound_us"] or 0) <= limit_us,
             {"max_bound_us": wn["nav_crit"]["max_bound_us"], "fraction": wn["nav_crit"]["max_bound_fraction"],
              "max_without_mpu_burst_us": max(r["nav_crit"]["max_equiv_us"] or 0 for r in normal + faults),
              "mpu_burst_isr_us": round(k["mpu_burst_isr_cycles"] / CYC_US, 1), "limit_us": limit_us,
              "worst": {"run": wn["scenario"], **(wn["nav_crit"]["worst_tick"] or {})},
              "_doc": "the MPU-6050 burst starts FC_IMU_LEAD_COUNTS before the tick and its bus time ends before it, so "
                      "its interrupts are normally outside this window; the bound adds all of them anyway"})
        # The requirement is the delay the 6-DOF sim flies with (sim/fc_timing.FC_COMPUTE_DELAY_S_MEASURED, 4.71 ms since
        # v0.6.3; every scenario, the tuning and the Monte Carlo use it), not one control tick: v0.5 / v0.6.2 were already
        # >= ~4.5 ms by construction (burst 0.496 ms before the tick, outputs at the next tick start) while the sim assumed
        # 4.0 ms.  ticks_over keeps the count against one tick for information.
        sim_delay_us = _sim_compute_delay_us()
        crit(f"sample -> PWM register <= the 6-DOF sim's compute delay ({sim_delay_us:.0f} us, sim/fc_timing.py "
             "FC_COMPUTE_DELAY_S_MEASURED; NAV IMU sample = MPU-6050 burst start -> FCU's last vane register, "
             "flight-equivalent; every normal run)",
             all(v["latency_imu_sample_to_vane_equiv_us"].get("max", 1e9) <= sim_delay_us for v in by_b.values()),
             {b: {"max_us": v["latency_imu_sample_to_vane_equiv_us"].get("max"), "limit_us": sim_delay_us,
                  "one_tick_us": tick_us,
                  "ticks_over": v["latency_over_one_tick"], "of": v["latency_imu_sample_to_vane_equiv_us"].get("n"),
                  "parts_max_us": {kk: vv.get("max") for kk, vv in v["latency_parts_us"].items()},
                  "nav_tick_to_vane_max_us": v["latency_nav_tick_to_vane_equiv_max_us"],
                  "worst": v["latency_worst"]} for b, v in by_b.items()})
        margin = k["frame_timeout_us"] - tick_us
        crit(f"NAV frame-ready increase from one tick to the next (+ the {k['baro_sensor']} TWI script) < the FCU's frame-timeout "
             f"margin ({margin} us = NLINK_FRAME_TIMEOUT_US - FC_TICK_US; every normal run, the pad -> flight switch "
             f"included)", all(v["nav_frame_ready_largest_increase_us"] < margin for v in by_b.values()),
             {b: {"largest_increase_us": v["nav_frame_ready_largest_increase_us"], "margin_us": margin}
              for b, v in by_b.items()})
        crit("0 missed ticks (normal flight: FCU timeout ticks after the first frame, NAV overruns, marker jumps)",
             all(not any(v["missed_ticks"].values()) for v in by_b.values()), {b: v["missed_ticks"] for b, v in by_b.items()})
        crit("nlink: no navigation frame lost, no FCU reply refused after the first, no reply byte 1..9 late (normal flight)",
             all(v["nlink"]["fcu_ticks_without_valid_frame_after_the_first"] == 0
                 and not any(v["nlink"]["fcu_counters_since_first_frame"].values())
                 and v["nlink"]["nav_replies_refused_after_its_first_valid"] == 0
                 and v["nlink"]["reply_bytes_1_9_late"] == 0 for v in by_b.values()),
             {b: v["nlink"] for b, v in by_b.items()})
        ints = [r["nlink_integrity"] for r in normal + faults + diag]
        crit("nlink: no wrong data accepted (every frame the FCU used = the one the NAV sent; FCU and NAV decoders agree "
             "with sim/nlink_proto.py; no damaged reply accepted) - every run",
             all(x["fcu_wrong_data_accepted_n"] == 0 and x["fcu_decoder_vs_python_mismatch_n"] == 0
                 and x["nav_accept_vs_python_mismatch_n"] == 0 and x["nav_accepted_damaged_replies"] == 0 for x in ints),
             {kk: sum(x[kk] for x in ints) for kk in ("fcu_frames_used", "fcu_wrong_data_accepted_n",
                                                      "fcu_decoder_vs_python_mismatch_n", "nav_accept_vs_python_mismatch_n",
                                                      "nav_accepted_damaged_replies")})
        crit("the FCU's background ends before the NAV's next exchange starts (normal flight, every frame tick)",
             all(v["fcu_background_ticks_past_next_exchange"] == 0 for v in by_b.values()),
             {b: {"min_margin_us": v["fcu_background_end_before_next_exchange_min_us"],
                  "ticks_past": v["fcu_background_ticks_past_next_exchange"]} for b, v in by_b.items()})
        crit("every IL_CMD reached the supervisor of the first tick after its last byte (0 ticks)",
             all(v["il_cmd_latency_ticks"] and v["il_cmd_latency_ticks"]["max"] == 0 and v["il_cmd_not_at_supervisor"] == 0
                 for v in by_b.values()),
             {b: {"latency": v["il_cmd_latency_ticks"], "not_at_supervisor": v["il_cmd_not_at_supervisor"],
                  "with_stat_in_front": v["il_cmd_with_stat_in_front"]} for b, v in by_b.items()})
        crit("no SNAP refused by the COM's reply gate in normal flight; no TELEM expired (sx127x: its burst's newest "
             "reply, by design in simavr)",
             all(v["snaps_refused"] == 0 for v in by_b.values())
             and all(v["telem_expired"] <= (v["runs"] if b == "sx127x" else 0) for b, v in by_b.items()),
             {b: {"refused": v["snaps_refused"], "expired": v["telem_expired"], "superseded": v["replies_superseded"]}
              for b, v in by_b.items()})
        win_us = 80_000
        crit("every normal uplink answered: TELEM handed to the radio < reply window after the IL_CMD was queued",
             all(v["unanswered_normal_uplinks"] == 0 and v["telem_out_of_window_at_ground"] == 0
                 and (v["reply_delay_us_normal"]["q_to_telem_us"].get("max") or 1e9) < win_us for v in by_b.values()),
             {b: {"q_to_telem_us": v["reply_delay_us_normal"]["q_to_telem_us"],
                  "uplink_to_telem_us": v["reply_uplink_to_telem_us_distribution"],
                  "unanswered": v["unanswered_normal_uplinks"], "out_of_window": v["telem_out_of_window_at_ground"]}
              for b, v in by_b.items()})
        ring_us = ILINK_RX_RING * 10 * 1_000_000 // (k["ilink_baud"] or 250000)
        crit(f"COM main-loop pass < the ilink RX ring fill time ({ring_us} us)",
             all((v["com_pass_max_us"] or 1e9) < ring_us for v in by_b.values()), {b: v["com_pass_max_us"] for b, v in by_b.items()})
        crit("the COM's boot IL_STAT reaches the FCU whole, BOOT flag set (R3; normal flight, every run)",
             all(not v["boot_stat_bytes_lost_before_fcu_rx_on"] and v["runs_whose_first_il_stat_at_the_fcu_lacks_boot"] == 0
                 for v in by_b.values()),
             {b: {"bytes_lost": v["boot_stat_bytes_lost_before_fcu_rx_on"],
                  "runs_without_boot_first": v["runs_whose_first_il_stat_at_the_fcu_lacks_boot"]} for b, v in by_b.items()})
        crit("POWERED reached, no interlock in normal flight",
             all(v["reached_powered_all"] and not v["interlock_transitions"] for v in by_b.values()),
             {b: {"transitions": v["transitions"], "not_by_command": v["interlock_transitions"]} for b, v in by_b.items()})
        crit("no ilink frame rejected / USART error on either chip, nothing but SNAPs FCU -> COM (normal flight)",
             all(not any(v["fcu_errors"].values()) and not any(v["com_errors"].values())
                 and v["fcu_to_com_non_snap_frames"] == 0 for v in by_b.values()),
             {b: {"fcu": v["fcu_errors"], "com": v["com_errors"]} for b, v in by_b.items()})
    for role in ("fcu", "nav") + BACKENDS:
        if role in stack:
            crit(f"{stack[role]['flight_env']} flight build stack >= {STACK_MIN_FREE} B free",
                 stack[role]["flight_free_bytes"] >= STACK_MIN_FREE, stack[role])
    for r in faults:
        f = r["fault"]
        crit(f"fault {r['scenario']}: {f['expect']}", f["pass"],
             {kk: f.get(kk) for kk in ("abort_after_last_frame_ticks", "abort_after_last_valid_frame_ticks",
                                       "release_after_abort_ticks", "link_gap_ticks", "during_babble", "fallback",
                                       "first_frame_after_reboot", "frames_after_reboot_before_abort_with_imu_ok_without_est_valid",
                                       "exchanges_damaged", "fcu_ticks_without_valid_frame", "fcu_counter_deltas",
                                       "fcu_tick_spacing_us", "nav_refused_damaged_replies", "crit_max_all_us_from_fault")
              if kk in f})
        if "crit_within_80_percent" in f:
            d = f["during_babble"]
            crit(f"fault {r['scenario']}: FCU time-critical part <= {CRIT_LIMIT:.0%} of the tick during the babble",
                 f["crit_within_80_percent"],
                 {"max_us": d["crit_all_max_us"], "fraction": d["crit_all_max_fraction"], "limit_us": limit_us,
                  "worst_tick": d["crit_worst_tick"]})
    rep["criteria"] = cr
    return rep


def summary_run(r: dict) -> dict:
    c0 = r["com"].get(0, {})
    return {"fcu_crit_all_max_us": r["fcu_crit"]["max_all_us"], "fcu_crit_worst_tick": r["fcu_crit"]["worst_tick"],
            "fcu_crit_from_tick_start_max_us": r["fcu_crit"]["max_from_tick_start_us"],
            "nav_crit_equiv_max_us": r["nav_crit"]["max_equiv_us"], "nav_crit_worst_tick": r["nav_crit"]["worst_tick"],
            "latency": {kk: r["latency"][kk] for kk in ("imu_sample_to_vane_equiv_us", "nav_tick_to_vane_equiv_us",
                                                        "frame_to_vane_us", "over_one_tick")},
            "nlink": {kk: r["nlink"][kk] for kk in ("exchanges", "fcu_ticks_without_valid_frame_after_the_first",
                                                    "nav_replies_refused_after_its_first_valid", "reply_bytes_late",
                                                    "reply_bytes_1_9_late", "write_collisions",
                                                    "reply_write_margin_min_cycles")},
            "nlink_integrity_wrong": r["nlink_integrity"]["fcu_wrong_data_accepted_n"],
            "loop": r["loop"], "fcu_tick_spacing_us": r["fcu_tick_spacing_us"],
            "il_cmd": {"frames": r["il_cmd"]["frames"], "latency_ticks": r["il_cmd"]["latency_ticks"],
                       "nonzero": r["il_cmd"]["nonzero"][:5], "with_stat_in_front": r["il_cmd"]["with_stat_in_front"]},
            "com_pass_max_us": c0.get("pass_max_us"), "com_reply_us_normal": c0.get("reply_us_normal"),
            "com_counters": c0.get("counters"), "stack_depth": r["stack_depth"], "wire": r["wire"],
            "transitions": r["transitions"], "wall_s": r.get("wall_s")}


def print_summary(rep: dict) -> None:
    t = rep["tick_us"]
    for b, v in rep["normal"]["by_backend"].items():
        w, n = v["fcu_crit_worst"], v["nav_crit_worst"]
        print(f"normal flight, {b} ({v['runs']} runs):")
        print(f"  FCU time-critical part (frame -> end, every interrupt + busy log port): max {v['fcu_crit_all_max_us']} us = "
              f"{100 * v['fcu_crit_all_max_us'] / t:.2f} % (run {w['run']}, tick {w['tick']}, {w['state']}, interrupts "
              f"{w['isr_us']} us in {w['isr_runs']}); from the tick start {v['fcu_crit_from_tick_start_max_us']} us; "
              f"without interrupts {v['fcu_crit_no_isr_max_us']} us; detection max {v['fcu_detect_us']['max']} us")
        print(f"  FCU interrupts inside, max per vector (us): {v['fcu_isr_in_crit_max_us_by_vector']}")
        print(f"  FCU sections (flight max, us): {v['fcu_sections_flight_max_us']}")
        print(f"  NAV time-critical part (tick -> frame delivered, + TWI): max {v['nav_crit_equiv_max_us']} us "
              f"(+ the whole MPU burst as a bound: {v['nav_crit_bound_max_us']} us) = "
              f"{100 * (v['nav_crit_equiv_max_us'] or 0) / t:.2f} % (run {n.get('run')}, tick {n.get('tick')}, phase "
              f"{n.get('baro_phase')}, pad {n.get('pad')}); flight {v['nav_crit_flight_max_equiv_us']} / pad "
              f"{v['nav_crit_pad_max_equiv_us']} us; by phase {v['nav_crit_flight_max_by_baro_phase_us']}")
        lt = v["latency_imu_sample_to_vane_equiv_us"]
        print(f"  sample -> vane register: {lt} us; parts max: "
              f"{ {kk: vv.get('max') for kk, vv in v['latency_parts_us'].items()} }; ticks over one tick "
              f"{v['latency_over_one_tick']} of {lt.get('n')}")
        print(f"  nlink: {v['nlink']}")
        print(f"  integrity: {v['nlink_integrity']}; missed ticks {v['missed_ticks']}")
        print(f"  FCU tick spacing {v['fcu_tick_spacing_us']} us; background before the next exchange min "
              f"{v['fcu_background_end_before_next_exchange_min_us']} us")
        print(f"  IL_CMD -> supervisor {v['il_cmd_latency_ticks']} ticks; COM pass max {v['com_pass_max_us']} us; reply "
              f"uplink -> TELEM {v['reply_uplink_to_telem_us_distribution']} us")
    for name, f in rep["faults"].items():
        print(f"{name}: {'PASS' if f['pass'] else 'FAIL'} - {f['expect']}")
        for tr in f["transitions"]:
            print(f"    tick {tr['tick']} ({tr['t_s']} s): {tr['from']} -> {tr['to']} ({tr['cause']}, reason {tr['reason']})")
        for kk in ("fallback", "first_frame_after_reboot", "exchanges_damaged", "fcu_counter_deltas", "fcu_tick_spacing_us",
                   "during_babble"):
            if kk in f:
                print(f"    {kk}: {f[kk]}")
    st = rep["stack"]
    print("stack: " + "; ".join(f"{kk}: depth {v['depth_bytes']} B -> {v['flight_env']} keeps {v['flight_free_bytes']} B"
                                for kk, v in st.items() if isinstance(v, dict)))
    if "worst_case_composition_fcu" in rep:
        c = rep["worst_case_composition_fcu"]
        print(f"FCU worst-case composition (not observed): {c['base_no_isr_us']} + ilink {c['ilink_add_us']} + log "
              f"{c['log_udre_us']} = {c['every_interrupt_us']} us = {100 * c['every_interrupt_fraction']:.2f} %")
    for name, c in rep["criteria"].items():
        print(f"  [{'ok' if c['pass'] else 'FAIL'}] {name}")


if __name__ == "__main__":
    sys.exit(main())
