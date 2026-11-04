#!/usr/bin/env python3
"""Measure the GNS board's main loop (v0.8.1 four-chip set: ATmega128A #4 - GPS receiver, UBX, the GPS filter and the
position controller at 50 Hz; src/gns/gns_main.c, core/gns_node.h) in the simavr ATmega128 simulator.

Builds the PlatformIO environments ``gns`` / ``gns_demo`` (the flight images of the two build profiles: the position
controller's parameters depend on the profile), ``gns_simbench`` (the same loop with a synthetic NAV on the glink - the
STATE frames of a synthetic flight injected by a Timer1 compare interrupt, bytes 0..58 then the last byte 2.36 ms later,
the injection phase sweeping the GNS's 4 ms poll - and a synthetic u-blox NEO-6M on the GPS UART: factory 9600 baud with
NMEA, ACKs the boot configuration, answers MON-VER, 5 Hz NAV-POSLLH / SOL / VELNED once configured, byte-accurate on a
Timer3 wire; src/gns/gns_simbench.h) and ``gns_cosim`` (the two-core harness's GNS: real glink USART0, build only), runs
``gns_simbench`` in PlatformIO's ``tool-simavr`` (ATmega128, 16 MHz, cycle-accurate for the AVR instruction set) and
parses its console (numbers hex):

* ``GC t= cy= cn= cm= cs= ca= fm= fs= bm= pm= pn= wk= wt= sf= ra=`` 1 Hz: cycles (RESULTs queued), the STATE's completion
  (its last byte in the ring) -> RESULT queued (max / sum / count / last), the fusion (gns_finish: max / sum), the
  completion -> fusion done (max), the 4 ms poll (max, its bytes max), the busy time of the window (CPU awake) and the
  longest pass, never-touched stack, RESULTs refused by the TX ring;
* ``GG`` configurator / parser / epochs / filter, ``GU`` the UARTs, ``GN`` the synthetic NAV's checks and the GNS's own
  counters (gns_stats_t), ``GJ`` the synthetic receiver.

Interrupts not in the measured figures: the 60 RX interrupts of a STATE frame arrive BEFORE its completion (the bench
injects the bytes; the flight build's USART0 RX interrupt is counted from the disassembly: the last byte's adds to the
completion -> RESULT figure), the RESULT's 26 UDRE interrupts run while the fusion does (added to the fusion and the busy
figures).  The GPS bytes' interrupts are in the measurement (the bench wire's Timer3 interrupt, 12 cycles longer than the
flight build's USART1 RX interrupt: conservative).

Criteria (the integration's, v0.8.1): every pio env builds without warnings other than the gains staleness; the bench
reaches its end; the GPS receiver is configured through the 9600-baud path (CFG-PRT switch, ACKs, MON-VER of a u-blox 6),
epochs are fused, the parser and the UARTs are clean; every STATE but the faulted one answered with the right SEQ, the
position loop engaged in HOLD; the gap / IMU-bad / CRC faults counted as gns_node.h defines them; the cycle (completion ->
fusion done, the worst case incl. a poll in progress) leaves margin in the 20 ms budget; the flight builds keep >= 1024 B
of never-touched stack (the bench's depth + the glink interrupt frame the bench does not exercise).  The RESULT time
(``state_to_result_max_us_with_isr``, rounded up to 10 us) is the NAV bench's synthetic-GNS input (nav_simbench.h
NAV_SB_GNS_CYCLE_US: tools/run_nav_simbench.py checks it is not shorter).

    .venv/Scripts/python.exe firmware/atmega128_fc/tools/run_gns_simbench.py [--profile field|demo|both]

Writes firmware/atmega128_fc/host_test/build/gns_simbench_report.json (field; _demo for the demo profile).  Numbers are a
measurement of the code in simavr, not of the board.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
import sys

FW = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ROOT = os.path.normpath(os.path.join(FW, "..", ".."))
OUT_DIR = os.path.join(FW, "host_test", "build")
RAM_BYTES = 4096
STACK_MIN_FREE = 1024
CYCLE_BUDGET_US = 20000.0                     # params flight_computer.gns.loop_hz 50
EXPECTED_WARNINGS = ("GAINS_PARAMS_STALE", "tuned against a different params.json")
# ATmega128 vectors
VECTORS = {"glink_rx_usart0": 18, "glink_udre_usart0": 19, "gps_rx_usart1": 30, "gps_udre_usart1": 31,
           "tick_timer2": 9, "lead_timer0": 15, "bench_t1_compa": 12, "bench_t1_compb": 13, "bench_gps_wire_t3": 26}
ISR_ENTRY_CYCLES = 7
CYCLES = {**{m: 1 for m in ("add adc sub subi sbc sbci and andi or ori eor com neg sbr cbr inc dec tst clr ser mov movw "
                             "ldi in out lsl lsr rol ror asr swap bst bld sec clc sen cln sez clz sei cli ses cls sev clv "
                             "set clt seh clh nop cp cpc cpi wdr sleep").split()},
          **{m: 2 for m in "adiw sbiw mul muls mulsu fmul fmuls fmulsu push pop lds sts ld st ldd std sbi cbi".split()},
          "lpm": 3, "elpm": 3}
SKIPS = {"cpse", "sbrc", "sbrs", "sbic", "sbis"}
GC_ALIAS = {"cy": "cycles", "cn": "cycles_timed", "cm": "state_to_result_max_us", "cs": "state_to_result_sum_us",
            "ca": "state_to_result_last_us", "fm": "fusion_max_us", "fs": "fusion_sum_us", "bm": "state_to_done_max_us",
            "pm": "poll_max_us", "pn": "poll_bytes_max", "wk": "busy_window_us", "wt": "busy_pass_max_us",
            "sf": "stack_free", "ra": "results_refused"}
GG_ALIAS = {"cs": "cfg_state", "co": "cfg_ok", "vo": "ver_ok", "md": "module", "nk": "cfg_naks", "to": "cfg_timeouts",
            "pf": "ubx_frames", "pk": "ubx_bad_ck", "pl": "ubx_bad_len", "ae": "epochs", "ai": "epochs_incomplete",
            "fu": "fused", "rp": "rej_p", "rv": "rej_v", "ps": "pos_resets", "sf": "snap_flags"}
GU_ALIAS = {"ub": "gps_rx_bytes", "ux": "gps_rx_overflow", "ue": "gps_rx_errors", "tb": "gps_tx_bytes",
            "tr": "gps_tx_refused", "bc": "gps_baud_changes", "gb": "glink_rx_bytes", "gx": "glink_rx_overflow",
            "ge": "glink_bad_crc", "gt": "glink_tx_bytes", "rd": "reports_dropped"}
GN_ALIAS = {"ns": "states_injected", "ng": "gap_cycles", "nf": "faults", "nr": "results", "nq": "results_seq_bad",
            "ne": "results_engaged", "nh": "hold_results_not_engaged", "gk": "results_gate_ok", "mm": "pos_modes_seen",
            "hs": "hold_steps", "gp": "gaps", "rs": "restarts", "is": "imu_bad", "cl": "clock_slips", "mi": "misaligned",
            "pm": "pos_mode"}
GJ_ALIAS = {"ra": "rcv_acks", "rv": "rcv_monver", "rs": "rcv_switches", "re": "rcv_epochs", "rd": "rcv_dropped",
            "rn": "rcv_noise_bytes", "rh": "rcv_frames_heard", "rw": "rcv_frames_wrong_baud", "rb": "rcv_baud_x100"}
BOOT_ALIAS = {"rc": "reset_cause", "sr": "stack_reserve", "gb": "glink_baud_x100", "gl": "glink_latency_ticks",
              "lp": "loop_hz"}
POS_MODES = ("OFF", "VEL", "POS", "DR", "ATT")


def tool_exe(package: str, name: str) -> str:
    home = os.environ.get("PLATFORMIO_CORE_DIR") or os.path.join(os.path.expanduser("~"), ".platformio")
    exe = os.path.join(home, "packages", package, "bin", name + (".exe" if os.name == "nt" else ""))
    if not os.path.exists(exe):
        raise SystemExit(f"{name} not found at {exe} (python -m platformio pkg install -g --tool platformio/{package})")
    return exe


def build(env: str) -> dict:
    r = subprocess.run([sys.executable, "-m", "platformio", "run", "-d", FW, "-e", env], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        raise SystemExit(f"build {env} failed:\n{r.stdout[-3000:]}\n{r.stderr[-2000:]}")
    out = {"env": env}
    m = re.search(r"RAM:.*used (\d+) bytes", r.stdout)
    out["static_ram_bytes"] = int(m.group(1)) if m else None
    m = re.search(r"Flash:.*used (\d+) bytes", r.stdout)
    out["flash_bytes"] = int(m.group(1)) if m else None
    warns = sorted({w for w in re.findall(r"warning: (.*)", r.stdout + r.stderr)})
    out["unexpected_warnings"] = [w for w in warns if not any(e in w for e in EXPECTED_WARNINGS)]
    out["elf"] = os.path.join(FW, ".pio", "build", env, "firmware.elf")
    hx = os.path.join(FW, ".pio", "build", env, "firmware.hex")
    out["hex_sha256"] = hashlib.sha256(open(hx, "rb").read()).hexdigest() if os.path.exists(hx) else None
    return out


def run(elf: str, timeout_s: float = 600.0) -> str:
    r = subprocess.run([tool_exe("tool-simavr", "simavr"), "-m", "atmega128", "-f", "16000000", elf],
                       capture_output=True, timeout=timeout_s)
    return (r.stdout + r.stderr).decode("latin-1")


def _kv(line: str, alias: dict) -> dict:
    return {alias.get(k, k): int(v, 16) for k, v in re.findall(r"(?<![0-9a-z])([a-z]+)=(0x[0-9a-f]+)", line)}


def parse(text: str) -> dict:
    out = {}
    for tag, alias in (("GC", GC_ALIAS), ("GG", GG_ALIAS), ("GU", GU_ALIAS), ("GN", GN_ALIAS), ("GJ", GJ_ALIAS)):
        out[tag.lower()] = [_kv(m.group(1), alias) for m in re.finditer(rf"\n{tag} (t=[ -~]*)", text)]
    boot = re.search(r"GB BOOT ([ -~]*)", text)
    out["boot"] = _kv(boot.group(1), BOOT_ALIAS) if boot else {}
    out["end"] = "GB END" in text
    return out


# ---------------------------------------------------------------- disassembly (as run_nav_simbench.py)
def disassemble(elf: str) -> dict:
    r = subprocess.run([tool_exe("toolchain-atmelavr", "avr-objdump"), "-d", elf], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    funcs: dict = {}
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


def isr_info(ins: list) -> dict:
    """normal (no branch / skip taken) and longest path cycles to RETI incl. entry; pushes on the normal path (stack)"""
    idx = {a: i for i, (a, _, _, _) in enumerate(ins)}
    memo: dict = {}

    def step(i: int, active: frozenset) -> int:
        if i in memo:
            return memo[i]
        if i in active:
            raise ValueError("loop in ISR")
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
        elif mn in CYCLES:
            c = CYCLES[mn] + step(i + 1, act)
        else:
            raise ValueError(f"unsupported instruction {mn} {ops}")
        memo[i] = c
        return c

    out = {"instructions": len(ins), "pushes": sum(1 for _, _, mn, _ in ins if mn == "push")}
    try:
        out["longest"] = ISR_ENTRY_CYCLES + step(0, frozenset())
    except (ValueError, KeyError) as e:
        out["longest_error"] = str(e)
    i, normal = 0, ISR_ENTRY_CYCLES
    try:
        for _ in range(len(ins) + 1):
            a, size, mn, ops = ins[i]
            if mn in ("reti", "ret"):
                normal += 4
                break
            if mn == "rjmp":
                normal += 2
                i = idx[_target(ops)]
            elif mn.startswith("br") or mn in SKIPS:
                normal += 1
                i += 1
            else:
                normal += CYCLES[mn]
                i += 1
        out["normal"] = normal
    except (ValueError, KeyError) as e:
        out["normal_error"] = str(e)
    return out


def isr_table(elf: str) -> dict:
    funcs = disassemble(elf)
    out = {}
    for name, vec in VECTORS.items():
        ins = funcs.get(f"__vector_{vec}")
        out[name] = {"vector": vec, "present": bool(ins), **(isr_info(ins) if ins else {})}
    return out


def run_profile(profile: str) -> dict:
    sfx = "" if profile == "field" else "_demo"
    flight_env, bench_env = f"gns{sfx}", f"gns_simbench{sfx}"
    print(f"==== GNS, profile {profile}: {flight_env} / {bench_env}")
    builds = {e: build(e) for e in (flight_env, bench_env)}
    if profile == "field":
        builds["gns_cosim"] = build("gns_cosim")
    text = run(builds[bench_env]["elf"])
    b = parse(text)
    if not b["gc"] or not b["end"]:
        print(text[-2000:])
        raise SystemExit(f"{bench_env}: no report / no GB END")
    gc, gg = b["gc"][-1], (b["gg"][-1] if b["gg"] else {})
    gu, gn, gj = (b["gu"][-1] if b["gu"] else {}), (b["gn"][-1] if b["gn"] else {}), (b["gj"][-1] if b["gj"] else {})
    isr = isr_table(builds[flight_env]["elf"])
    isr_bench = isr_table(builds[bench_env]["elf"])
    rx0 = isr["glink_rx_usart0"].get("normal")
    udre0 = isr["glink_udre_usart0"].get("normal")
    cyc_us = 1.0 / 16.0
    # the cycle: completion -> RESULT queued (+ the last STATE byte's RX interrupt, not in the bench: it arrives at the
    # completion); the fusion runs while the 26 RESULT bytes go out (their UDRE interrupts added)
    s2r = gc.get("state_to_result_max_us")
    s2r_isr = None if s2r is None or rx0 is None else round(s2r + rx0 * cyc_us, 1)
    fus = gc.get("fusion_max_us")
    fus_isr = None if fus is None or udre0 is None else round(fus + 26 * udre0 * cyc_us, 1)
    done = gc.get("state_to_done_max_us")
    done_isr = None if done is None or udre0 is None or rx0 is None else round(done + (26 * udre0 + rx0) * cyc_us, 1)
    n_cyc = gc.get("cycles_timed") or 0
    windows = [r for r in b["gc"] if r.get("busy_window_us") is not None]
    busy_max = max((r["busy_window_us"] for r in windows), default=None)
    # the glink RX / UDRE interrupts do not run in the bench (bypass): their frames are added to the stack figure
    glink_isr_stack = max((isr[n].get("pushes", 0) for n in ("glink_rx_usart0", "glink_udre_usart0")), default=0) + 2
    stack_used = RAM_BYTES - builds[bench_env]["static_ram_bytes"] - min(r.get("stack_free", 0) for r in b["gc"])
    flight_free = RAM_BYTES - builds[flight_env]["static_ram_bytes"] - stack_used - glink_isr_stack
    cycle = {
        "cycles": gc.get("cycles"), "cycles_timed": n_cyc,
        "state_to_result_max_us": s2r, "state_to_result_mean_us": None if not n_cyc else
        round(gc.get("state_to_result_sum_us", 0) / n_cyc, 1),
        "state_to_result_max_us_with_isr": s2r_isr,
        "nav_bench_input_us": None if s2r_isr is None else int(math.ceil(s2r_isr / 10.0) * 10),
        "fusion_max_us": fus, "fusion_max_us_with_udre": fus_isr,
        "fusion_mean_us": None if not n_cyc else round(gc.get("fusion_sum_us", 0) / n_cyc, 1),
        "state_to_done_max_us": done, "state_to_done_max_us_with_isr": done_isr,
        "poll_max_us": gc.get("poll_max_us"), "poll_bytes_max": gc.get("poll_bytes_max"),
        "busy_pass_max_us": gc.get("busy_pass_max_us"),
        "cpu_load_max_window": None if busy_max is None else round(busy_max / 1e6, 4),
        "cycle_budget_us": CYCLE_BUDGET_US,
        "cycle_margin_us": None if done_isr is None else round(CYCLE_BUDGET_US - done_isr - (gc.get("poll_max_us") or 0), 1),
        "_doc": "state_to_result: the STATE frame's last byte in the glink ring (the bench's Timer1 COMPB) -> the RESULT in "
                "the TX ring (the latency path: decode of the last byte, gns_on_state = the gap rules, the filter STEP, "
                "the position step, the RESULT; incl. a 4 ms poll that was running when the frame completed); fusion: "
                "gns_finish right after (off the latency path); state_to_done = the two; cycle_margin = 20 ms - "
                "state_to_done (with the interrupts) - one more poll",
    }
    profile_ok = (gn.get("results", 0) > 0 and gn.get("results") == gn.get("states_injected", 0) - 1
                  and gn.get("results_seq_bad") == 0 and gn.get("results_engaged", 0) > 0
                  and gn.get("hold_results_not_engaged") == 0 and (gn.get("pos_modes_seen", 0) & (1 << 2)))
    checks = {
        "builds_without_unexpected_warnings": all(not v["unexpected_warnings"] for v in builds.values()),
        "bench_end_reached": b["end"],
        "gps_configured_and_acked": gg.get("cfg_state") == 7 and gg.get("cfg_ok") == 1 and gg.get("ver_ok") == 1
                                    and gg.get("module") == 6 and gg.get("cfg_naks") == 0,
        "gps_9600_path_switched_once": gj.get("rcv_switches") == 1 and gu.get("gps_baud_changes") == 2,
        "gps_epochs_fused": (gg.get("epochs") or 0) > 0 and (gg.get("fused") or 0) > 0,
        "gps_parser_clean": gg.get("ubx_bad_ck") == 0 and gg.get("ubx_bad_len") == 0,
        "gps_uart_clean": gu.get("gps_rx_overflow") == 0 and gu.get("gps_tx_refused") == 0,
        "gps_receiver_never_dropped": gj.get("rcv_dropped") == 0,
        "glink_rx_clean": gu.get("glink_rx_overflow") == 0 and gu.get("glink_bad_crc") == 1,
        "every_state_answered_seq_ok_engaged": bool(profile_ok),
        "faults_counted": gn.get("hold_steps") == 3 + 1 + 1 + 1 and gn.get("gaps") == 3 and gn.get("imu_bad") == 1
                          and gn.get("restarts") == 0 and (gn.get("clock_slips") or 0) <= 1,
        "no_result_refused": gc.get("results_refused") == 0,
        "cycle_inside_budget": cycle["cycle_margin_us"] is not None and cycle["cycle_margin_us"] > 0,
        "flight_stack_free_ge_1024": flight_free >= STACK_MIN_FREE,
        "bench_gps_wire_isr_not_cheaper": (isr_bench["bench_gps_wire_t3"].get("normal") or 0)
                                          >= (isr["gps_rx_usart1"].get("normal") or 99),
    }
    report = {
        "profile": profile, "builds": {e: {k: v for k, v in d.items() if k != "elf"} for e, d in builds.items()},
        "boot": b["boot"], "cycle": cycle, "gps": {**gg, **{k: gu.get(k) for k in GU_ALIAS.values()}},
        "synthetic_nav_and_gns_counters": gn, "synthetic_receiver": gj, "interrupts_flight": isr,
        "interrupts_bench": {k: v for k, v in isr_bench.items() if k.startswith("bench")},
        "stack": {"bench_static_ram_bytes": builds[bench_env]["static_ram_bytes"], "stack_used_bytes": stack_used,
                  "glink_isr_frame_not_exercised_bytes": glink_isr_stack,
                  "flight_static_ram_bytes": builds[flight_env]["static_ram_bytes"], "flight_stack_free_bytes": flight_free},
        "checks": checks, "pass": all(checks.values()),
    }
    print("GNS builds: " + ", ".join(f"{e} flash {d['flash_bytes']} B RAM {d['static_ram_bytes']} B" for e, d in builds.items()))
    print(f"cycle: STATE complete -> RESULT queued <= {s2r} us ({s2r_isr} us with the last byte's RX interrupt; mean "
          f"{cycle['state_to_result_mean_us']} us), fusion <= {fus} us ({fus_isr} with the RESULT's UDRE interrupts), "
          f"-> done <= {done_isr} us; poll <= {cycle['poll_max_us']} us; longest awake pass {cycle['busy_pass_max_us']} us; "
          f"CPU load <= {100 * (cycle['cpu_load_max_window'] or 0):.1f} % of a 1 s window; margin in the 20 ms cycle "
          f"{cycle['cycle_margin_us']} us; NAV bench input (nav_simbench.h NAV_SB_GNS_CYCLE_US) {cycle['nav_bench_input_us']} us")
    print(f"stack: {stack_used} B deep + {glink_isr_stack} B glink ISR frame -> flight {flight_env} keeps {flight_free} B free")
    print(f"GPS {report['gps']}; synthetic NAV / GNS counters {gn}")
    for name, ok in checks.items():
        if not ok:
            print(f"FAIL {name}")
    print(f"[gns {profile}] {'PASS' if report['pass'] else 'FAIL'}")
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--profile", choices=("field", "demo", "both"), default="both")
    a = ap.parse_args()
    profiles = ["field", "demo"] if a.profile == "both" else [a.profile]
    ok = True
    for pr in profiles:
        rep = run_profile(pr)
        ok = ok and rep["pass"]
        os.makedirs(OUT_DIR, exist_ok=True)
        path = os.path.join(OUT_DIR, "gns_simbench_report.json" if pr == "field" else "gns_simbench_report_demo.json")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(rep, f, indent=1)
        print(f"-> {os.path.relpath(path, ROOT)}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
