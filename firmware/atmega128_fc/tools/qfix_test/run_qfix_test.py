#!/usr/bin/env python3
"""The AVR fx_mulhi test (fw09 SPEC_firmware_v09 WP-X): ``qfix_test.c`` built with PlatformIO's avr-gcc and run in
PlatformIO's simavr (ATmega128, 16 MHz).  It multiplies 200000 operand pairs of every magnitude with the inline-asm
``fx_mulhi`` of ``src/core/qfix.h`` and with its C definition ``fx_mulhi_ref`` (the arithmetic the host test and the
crosscheck run) and counts the pairs where they differ and the pairs whose result is neither the exact floor of the
64-bit product nor one less.

Invariant: the AVR image's fixed-point estimator (NAV, core/att_estimator.c) computes bit for bit what the host core
computes - PASS only with 200000 pairs, 0 mismatches and 0 results outside {floor, floor - 1}.

    python firmware/atmega128_fc/tools/qfix_test/run_qfix_test.py        # exit 0 pass, 1 fail, 77 toolchain missing

Output: ``firmware/atmega128_fc/host_test/build/qfix_test/qfix_test.elf`` and ``qfix_test.txt`` (the simavr console).
"""
from __future__ import annotations

import hashlib
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, "..", ".."))
SRC = os.path.join(FW, "src")
OUT = os.path.join(FW, "host_test", "build", "qfix_test")
PAIRS = 200000
LINE_RE = re.compile(r"QFIX pairs=(\d+) asm_vs_ref_mismatch=(\d+) outside_floor_or_floor_minus_1=(\d+)")
CYC_RE = re.compile(r"QFIX cycles[^:]*: fx_mulhi (\d+), fx_mulhi_ref (\d+), fmul (\d+), fadd (\d+)")


def tool(package: str, name: str) -> str | None:
    home = os.environ.get("PLATFORMIO_CORE_DIR") or os.path.join(os.path.expanduser("~"), ".platformio")
    exe = os.path.join(home, "packages", package, "bin", name + (".exe" if os.name == "nt" else ""))
    return exe if os.path.exists(exe) else None


def run() -> dict:
    """Build + run; returns {'status': 'PASS' | 'FAIL' | 'NOT RUN', ...}."""
    gcc, simavr = tool("toolchain-atmelavr", "avr-gcc"), tool("tool-simavr", "simavr")
    if not gcc or not simavr:
        return {"status": "NOT RUN", "why": f"avr-gcc {gcc or 'missing'}, simavr {simavr or 'missing'} "
                                            "(PlatformIO toolchain-atmelavr / tool-simavr)"}
    os.makedirs(OUT, exist_ok=True)
    elf = os.path.join(OUT, "qfix_test.elf")
    r = subprocess.run([gcc, "-mmcu=atmega128", "-Os", "-std=gnu99", "-DF_CPU=16000000UL", "-Wall", "-I", SRC,
                        os.path.join(HERE, "qfix_test.c"), "-o", elf], capture_output=True, text=True, encoding="utf-8",
                       errors="replace")
    if r.returncode != 0:
        return {"status": "FAIL", "why": "avr-gcc failed: " + (r.stdout + r.stderr)[-2000:]}
    t0 = time.perf_counter()
    try:
        s = subprocess.run([simavr, "-m", "atmega128", "-f", "16000000", elf], capture_output=True, timeout=900,
                           stdin=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        return {"status": "FAIL", "why": "simavr did not finish in 900 s"}
    con = (s.stdout + s.stderr).decode("latin-1")
    with open(os.path.join(OUT, "qfix_test.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write(con)
    m = LINE_RE.search(con)
    if not m:
        return {"status": "FAIL", "why": "no QFIX line in the simavr console: " + con[-1500:]}
    pairs, bad, outside = (int(x) for x in m.groups())
    c = CYC_RE.search(con)
    with open(elf, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    ok = pairs == PAIRS and bad == 0 and outside == 0
    return {"status": "PASS" if ok else "FAIL", "pairs": pairs, "asm_vs_ref_mismatch": bad,
            "outside_floor_or_floor_minus_1": outside, "wall_s": round(time.perf_counter() - t0, 1),
            "cycles": None if not c else dict(zip(("fx_mulhi", "fx_mulhi_ref", "fmul", "fadd"), map(int, c.groups()))),
            "elf_sha256": sha, "avr_gcc": gcc, "simavr": simavr}


def main() -> int:
    res = run()
    if res["status"] == "NOT RUN":
        print(f"qfix_test: NOT RUN - {res['why']}")
        return 77
    if res["status"] == "FAIL" and "pairs" not in res:
        print(f"qfix_test: FAIL - {res['why']}")
        return 1
    print(f"qfix_test: {res['status']} - AVR fx_mulhi vs fx_mulhi_ref on {res['pairs']} pairs (need {PAIRS}): "
          f"{res['asm_vs_ref_mismatch']} mismatches, {res['outside_floor_or_floor_minus_1']} outside floor / floor - 1; "
          f"cycles {res['cycles']}; simavr {res['wall_s']} s; elf sha256 {res['elf_sha256'][:16]}")
    return 0 if res["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
