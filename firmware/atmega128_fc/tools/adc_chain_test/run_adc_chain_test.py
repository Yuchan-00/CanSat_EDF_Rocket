#!/usr/bin/env python3
"""The power builds' ADC conversion chain in simavr (fw09 review F2).  ``adc_chain_avr.c`` + ``src/hal/adc.c`` built with
PlatformIO's avr-gcc and the -D flags of each power FCU env (``fcu_wired``: one channel, ``fcu_wired_buffer``: three),
run by ``adc_chain_sim.c`` (zig cc + PlatformIO's libsimavr.a, as tools/cosim/cosim4.c) with distinct millivolts on the
chain's channels.  The flight images run this chain (adc_chain_take, the ADC interrupt, adc_chain_vin); every bench and
co-simulation build replaces it by simbench_adc_chain, so this is the test in which the flight path runs.

Invariant: each chain position's code lands in its own slot (V_in -> code[PM_CH_IN], V_tether -> code[PM_CH_TET], V_buffer
-> code[PM_CH_BUF]), a channel the chain did not reach is PM_CODE_NONE (the first take; a take a quarter conversion after the previous
one: none; 1.5 conversions after it: V_in only), the codes equal simavr's conversion of the input (+-2 counts of mV x 1024 / 2560 at
the internal 2.56 V reference), adc_chain_vin returns V_in once it is converted, and the ADC interrupt ran.

    python firmware/atmega128_fc/tools/adc_chain_test/run_adc_chain_test.py    # exit 0 pass, 1 fail, 77 tools missing
    ... [--adc-src PATH]   build another adc.c (comparison of two versions; same output = same behaviour)

2026-11-10 control round: also the logic-battery path of the VTOL battery FCU (``lbat_avr.c`` + ``src/hal/adc.c`` with the
env ``fcu``'s -D flags: adc_read_blocking at boot, adc_poll_mux on main.c's schedule) with distinct millivolts on the V_cell
and PF3 channels: each result carries its channel, the codes match the inputs, the lbat byte is
sim.link_proto.lbat_q_from_counts of the code; and which FCU envs compile FC_LOGIC_BAT (fcu / fcu_demo / fcu_nogps: 1; the
rocket and the tether / hybrid FCUs: 0).

Output: ``firmware/atmega128_fc/host_test/build/adc_chain_test/`` (elf / hex per env, the simulator, <env>.txt consoles).
"""
from __future__ import annotations

import argparse
import hashlib
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, "..", ".."))
ROOT = os.path.normpath(os.path.join(FW, "..", ".."))
SRC = os.path.join(FW, "src")
OUT = os.path.join(FW, "host_test", "build", "adc_chain_test")
ENVS = ("fcu_wired", "fcu_wired_buffer")
MV = (1000, 2000, 500, 0, 0, 0, 0, 0)        # ADC0 .. ADC7 (the chain uses VBAT / PWR_V_TETHER / PWR_V_BUFFER channels)
NONE = 0xFFFF
LINE_RE = re.compile(r"ADC t=(\d+) code=(\d+),(\d+),(\d+) vin=(\d+)")
SIM_RE = re.compile(r"SIM adc_isr=(\d+) cycles=(\d+) state=(-?\d+)")


def sh(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", **kw)


def pio_home() -> str:
    return os.environ.get("PLATFORMIO_CORE_DIR") or os.path.join(os.path.expanduser("~"), ".platformio")


def tool(package: str, name: str) -> str | None:
    exe = os.path.join(pio_home(), "packages", package, "bin", name + (".exe" if os.name == "nt" else ""))
    return exe if os.path.exists(exe) else None


def simavr_dir() -> str | None:
    d = os.environ.get("SIMAVR_DIR") or os.path.join(pio_home(), "packages", "tool-simavr")
    return d if os.path.exists(os.path.join(d, "lib", "libsimavr.a")) else None


def have_zig() -> bool:
    return sh([sys.executable, "-m", "ziglang", "version"]).returncode == 0


def env_defines(env: str) -> list[str]:
    sys.path.insert(0, os.path.join(FW, "tools"))
    import run_cosim4  # noqa: E402  (the PlatformIO env's -D flags, ${sec.key} resolved)
    return run_cosim4.env_defines(FW, env)


def header_defines(defs: list[str], names: tuple[str, ...]) -> dict:
    """the channel numbers / build macros as params_gen.h and core/power_mon.h give them for these -D flags (each name
    expanded by avr-gcc -E, the integer expression evaluated)"""
    gcc = tool("toolchain-atmelavr", "avr-gcc")
    src = os.path.join(OUT, "probe_channels.c")
    with open(src, "w", encoding="utf-8", newline="\n") as f:
        f.write('#include "params_gen.h"\n#include "core/power_mon.h"\n')
        for n in names:
            f.write(f"PROBE_{n} = {n} ;\n")
    r = sh([gcc, "-mmcu=atmega128", "-E", "-P", "-I", SRC, *defs, src])
    out = {}
    for n in names:
        m = re.search(rf"^PROBE_{n} = (.*) ;$", r.stdout, re.M)
        if m and m.group(1).strip() != n:
            expr = re.sub(r"(\d+)[uUlL]+", r"\1", m.group(1)).replace("&&", " and ").replace("||", " or ")
            out[n] = int(eval(expr, {"__builtins__": {}}, {}))   # an integer expression of the header's macros
    return out


def code_of(mv: int) -> int:
    return mv * 1024 // 2560


def near(a: int, mv: int) -> bool:
    return a != NONE and abs(a - code_of(mv)) <= 2


def build_sim() -> str:
    sim = simavr_dir()
    exe = os.path.join(OUT, "adc_chain_sim" + (".exe" if os.name == "nt" else ""))
    cmd = [sys.executable, "-m", "ziglang", "cc"] + (["-target", "x86_64-windows-gnu"] if os.name == "nt" else [])
    cmd += ["-std=c99", "-O2", "-Wall", "-Wextra", "-I", os.path.join(sim, "include", "simavr"), "-o", exe,
            os.path.join(HERE, "adc_chain_sim.c"), os.path.join(FW, "tools", "cosim", "elf_stubs.c"),
            os.path.join(sim, "lib", "libsimavr.a")]
    cmd += ["-lws2_32"] if os.name == "nt" else ["-lm"]
    r = sh(cmd)
    if r.returncode != 0:
        raise RuntimeError("adc_chain_sim build failed: " + (r.stdout + r.stderr)[-3000:])
    return exe


def run_env(env: str, sim_exe: str, adc_src: str) -> dict:
    gcc, objcopy = tool("toolchain-atmelavr", "avr-gcc"), tool("toolchain-atmelavr", "avr-objcopy")
    defs = env_defines(env)
    elf = os.path.join(OUT, f"{env}.elf")
    hexf = os.path.join(OUT, f"{env}.hex")
    r = sh([gcc, "-mmcu=atmega128", "-Os", "-std=gnu99", "-DF_CPU=16000000UL", "-Wall", *defs, "-I", SRC,
            os.path.join(HERE, "adc_chain_avr.c"), adc_src, "-o", elf])
    if r.returncode != 0:
        return {"env": env, "status": "FAIL", "why": "avr-gcc failed: " + (r.stdout + r.stderr)[-2000:]}
    r = sh([objcopy, "-O", "ihex", "-R", ".eeprom", elf, hexf])
    if r.returncode != 0:
        return {"env": env, "status": "FAIL", "why": "avr-objcopy failed: " + r.stderr[-1000:]}
    ch = header_defines(defs, ("VBAT_ADC_CHANNEL", "PWR_V_TETHER_ADC_CHANNEL", "PWR_V_BUFFER_ADC_CHANNEL", "PM_BUF",
                               "PWR_SRC"))
    s = sh([sim_exe, hexf, "mv=" + ",".join(str(x) for x in MV), "max_ms=200"], timeout=600)
    con = s.stdout + s.stderr
    with open(os.path.join(OUT, f"{env}.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write(con)
    rows = {int(m.group(1)): tuple(int(x) for x in m.groups()[1:]) for m in LINE_RE.finditer(con)}
    sm = SIM_RE.search(con)
    buf = env.endswith("_buffer")
    mv_in = MV[ch.get("VBAT_ADC_CHANNEL", 0)]
    mv_tet = MV[ch.get("PWR_V_TETHER_ADC_CHANNEL", 1)]
    mv_buf = MV[ch.get("PWR_V_BUFFER_ADC_CHANNEL", 2)]

    def full(r: tuple) -> bool:
        if buf:
            return near(r[0], mv_in) and near(r[1], mv_tet) and near(r[2], mv_buf)
        return near(r[0], mv_in) and r[1] == NONE and r[2] == NONE

    checks = {
        "every step reported": sorted(rows) == list(range(10)) and "ADC end" in con,
        "first take: nothing converted (NONE x 3)": 0 in rows and rows[0][:3] == (NONE, NONE, NONE),
        "takes 4 ms apart: every channel in its slot": all(s_ in rows and full(rows[s_]) for s_ in (1, 2, 3, 4, 5, 7, 9)),
        "take a quarter conversion after a take: NONE x 3": 6 in rows and rows[6][:3] == (NONE, NONE, NONE),
        "take 1.5 conversions after a take: V_in only": 8 in rows and near(rows[8][0], mv_in) and rows[8][1] == NONE
                                               and rows[8][2] == NONE,
        "adc_chain_vin 2 conversions after a take = V_in": all(s_ in rows and near(rows[s_][3], mv_in) for s_ in range(10)),
        "the ADC interrupt ran": bool(sm) and int(sm.group(1)) >= (29 if buf else 11),
        "simulator ended normally": bool(sm) and s.returncode == 0,
        "build is the power build of the env": ch.get("PM_BUF") == (1 if buf else 0),
    }
    with open(hexf, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    cv = re.search(r"ADC conv_us=(\d+)", con)
    return {"env": env, "status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
            "rows": {k: list(v) for k, v in sorted(rows.items())}, "adc_isr": int(sm.group(1)) if sm else None,
            "simavr_conversion_us": int(cv.group(1)) if cv else None, "channels": ch, "mv": MV, "hex_sha256": sha,
            "adc_src": adc_src}


LBAT_ENVS_ON = ("fcu", "fcu_demo", "fcu_nogps")
LBAT_ENVS_OFF = ("fcu_ballistic", "fcu_wired", "fcu_wired_buffer")
LBAT_MV_VBAT, LBAT_MV_PF3 = 1300, 1930          # V_cell divider input / the 1S pack 3.86 V through 47k / 47k
LB_RE = re.compile(r"LB t=(\d+) ch=(\d+) code=(\d+) q=(\d+)")


def lbat_flag(env: str) -> int | None:
    """FC_LOGIC_BAT as src/hal/adc.h computes it for the env's -D flags (avr-gcc -E)"""
    gcc = tool("toolchain-atmelavr", "avr-gcc")
    src = os.path.join(OUT, "probe_lbat.c")
    with open(src, "w", encoding="utf-8", newline="\n") as f:
        f.write('#include "hal/adc.h"\nPROBE_LBAT = FC_LOGIC_BAT ;\n')
    r = sh([gcc, "-mmcu=atmega128", "-E", "-P", "-I", SRC, *env_defines(env), src])
    m = re.search(r"^PROBE_LBAT = (\d+) ;$", r.stdout, re.M)
    return int(m.group(1)) if m else None


def run_lbat(sim_exe: str, adc_src: str) -> dict:
    sys.path.insert(0, ROOT)
    from sim import link_proto as lp  # noqa: E402  (the reference lbat byte)
    gcc, objcopy = tool("toolchain-atmelavr", "avr-gcc"), tool("toolchain-atmelavr", "avr-objcopy")
    defs = env_defines("fcu")
    elf, hexf = os.path.join(OUT, "fcu_lbat.elf"), os.path.join(OUT, "fcu_lbat.hex")
    r = sh([gcc, "-mmcu=atmega128", "-Os", "-std=gnu99", "-DF_CPU=16000000UL", "-Wall", *defs, "-I", SRC,
            os.path.join(HERE, "lbat_avr.c"), adc_src, "-o", elf])
    if r.returncode != 0:
        return {"env": "fcu_lbat", "status": "FAIL", "why": "avr-gcc failed: " + (r.stdout + r.stderr)[-2000:]}
    r = sh([objcopy, "-O", "ihex", "-R", ".eeprom", elf, hexf])
    if r.returncode != 0:
        return {"env": "fcu_lbat", "status": "FAIL", "why": "avr-objcopy failed: " + r.stderr[-1000:]}
    ch = header_defines(defs, ("VBAT_ADC_CHANNEL", "LOGIC_BAT_ADC_CHANNEL", "LOGIC_BAT_Q_X1024"))
    mv = [0] * 8
    mv[ch["VBAT_ADC_CHANNEL"]] = LBAT_MV_VBAT
    mv[ch["LOGIC_BAT_ADC_CHANNEL"]] = LBAT_MV_PF3
    s = sh([sim_exe, hexf, "mv=" + ",".join(str(x) for x in mv), "max_ms=200"], timeout=600)
    con = s.stdout + s.stderr
    with open(os.path.join(OUT, "fcu_lbat.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write(con)
    boot = re.search(r"LB boot code=(\d+) q=(\d+)", con)
    rows = [tuple(int(x) for x in m.groups()) for m in LB_RE.finditer(con)]
    vb, lb_ch, qx = ch["VBAT_ADC_CHANNEL"], ch["LOGIC_BAT_ADC_CHANNEL"], ch["LOGIC_BAT_Q_X1024"]
    sched = [lb_ch if (k + 1) % 3 == 0 else vb for k in range(12)]      # the conversion each tick started
    want_ch = [vb] + sched[:-1]                                         # the result a tick reads: the previous start
    flags = {e: lbat_flag(e) for e in LBAT_ENVS_ON + LBAT_ENVS_OFF}
    checks = {
        "boot read of PF3 = the input (+-2 counts), lbat byte = lbat_q_from_counts": bool(boot)
            and near(int(boot.group(1)), LBAT_MV_PF3) and int(boot.group(2)) == lp.lbat_q_from_counts(int(boot.group(1)), qx),
        "12 ticks, no busy ADC (4 ms >> one conversion)": len(rows) == 12 and "busy" not in con and "LB end" in con,
        "each result carries the channel it converted (one PF3 slot every 3rd tick, V_cell otherwise)":
            [r[1] for r in rows] == want_ch,
        "codes match their channel's input (+-2 counts)": all(near(r[2], LBAT_MV_PF3 if r[1] == lb_ch else LBAT_MV_VBAT)
                                                             for r in rows),
        "lbat byte of every PF3 result = sim.link_proto.lbat_q_from_counts (3.86 V -> 193 +-1)":
            all(r[3] == lp.lbat_q_from_counts(r[2], qx) and abs(r[3] - 193) <= 1 for r in rows if r[1] == lb_ch),
        "FC_LOGIC_BAT: 1 in fcu / fcu_demo / fcu_nogps, 0 in the rocket and the tether / hybrid FCUs":
            all(flags[e] == 1 for e in LBAT_ENVS_ON) and all(flags[e] == 0 for e in LBAT_ENVS_OFF),
        "simulator ended normally": s.returncode == 0,
    }
    with open(hexf, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    return {"env": "fcu_lbat", "status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
            "rows": {k: list(v[1:]) + [0] for k, v in enumerate(rows)}, "boot": boot.groups() if boot else None,
            "flags": flags, "channels": ch, "adc_isr": None, "simavr_conversion_us": None, "hex_sha256": sha}


def run(adc_src: str | None = None) -> dict:
    gcc, objcopy = tool("toolchain-atmelavr", "avr-gcc"), tool("toolchain-atmelavr", "avr-objcopy")
    if not gcc or not objcopy or not simavr_dir() or not have_zig():
        return {"status": "NOT RUN", "why": f"avr-gcc {gcc or 'missing'}, avr-objcopy {objcopy or 'missing'}, "
                                            f"libsimavr {simavr_dir() or 'missing'}, zig cc {'ok' if have_zig() else 'missing'}"}
    os.makedirs(OUT, exist_ok=True)
    try:
        sim_exe = build_sim()
    except RuntimeError as e:
        return {"status": "FAIL", "why": str(e)}
    res = {e: run_env(e, sim_exe, adc_src or os.path.join(SRC, "hal", "adc.c")) for e in ENVS}
    res["fcu_lbat"] = run_lbat(sim_exe, adc_src or os.path.join(SRC, "hal", "adc.c"))      # 2026-11-10
    return {"status": "PASS" if all(r["status"] == "PASS" for r in res.values()) else "FAIL", "envs": res}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--adc-src", help="an adc.c to build instead of src/hal/adc.c")
    a = ap.parse_args()
    res = run(a.adc_src)
    if res["status"] == "NOT RUN":
        print(f"adc_chain_test: NOT RUN - {res['why']}")
        return 77
    if "envs" not in res:
        print(f"adc_chain_test: FAIL - {res['why']}")
        return 1
    for e, r in res["envs"].items():
        print(f"adc_chain_test {e}: {r['status']}" + (f" - {r.get('why')}" if "why" in r else
                                                       f"; ADC interrupts {r['adc_isr']}; simavr conversion "
                                                       f"{r['simavr_conversion_us']} us (chip 104 us); hex "
                                                       f"{r['hex_sha256'][:16]}"))
        for k, v in (r.get("checks") or {}).items():
            print(f"   {'ok  ' if v else 'FAIL'} {k}")
        for k, v in (r.get("rows") or {}).items():
            print(f"   step {k}: code {v[0]},{v[1]},{v[2]} vin {v[3]}")
    print(f"adc_chain_test: {res['status']}")
    return 0 if res["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
