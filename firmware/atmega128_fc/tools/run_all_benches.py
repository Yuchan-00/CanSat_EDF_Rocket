#!/usr/bin/env python3
"""Run every firmware bench in the fixed order and write one summary (SPEC_firmware_v09 WP-B):

  1. GNS   run_gns_simbench.py --profile both          (the NAV bench reads its report: NAV_SB_GNS_CYCLE_US must not be
                                                         shorter than the GNS's measured cycle - on a fresh tree the NAV
                                                         bench fails bench_gns_cycle_not_shorter_than_measured without it)
  2. NAV   run_nav_simbench.py --mode vtol, --mode ballistic
  3. FCU   run_simbench.py --mode vtol --profile all, --profile power, --mode ballistic
  4. COM   run_com_simbench.py --mode both
  5. co-sim run_cosim4.py (the closed-loop four-core co-simulation, every scenario; its report judges the rows of SPEC
           section 7 the FCU bench's self-load checks defer to)
  6. FCU re-judge: run_simbench.py --rejudge (the FCU bench reports of step 3 against the co-simulation report of step 5:
           no simulation runs again)
  7. ballistic hashes: the *_ballistic flight images built and compared with the declared ones (SPEC section 7 row 21)

Each step runs even when an earlier one failed (a failure is reported, never hidden); exit status 0 only if every step
passed.  Writes firmware/atmega128_fc/host_test/build/all_benches_summary.json (per step: command, exit status, wall time,
pass, the report files and their key figures; the ballistic image hashes).  CPU: the runners build with
PLATFORMIO_RUN_JOBS (default 2 here) and run one simulator at a time; the co-simulation runs --cosim-jobs (default 2).

usage: run_all_benches.py [--skip STEP,...] [--cosim-jobs 2] [--pio-jobs 2] [--summary PATH] [--cosim-arg=ARG ...]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(TOOLS, ".."))
ROOT = os.path.normpath(os.path.join(FW, "..", ".."))
OUT = os.path.join(FW, "host_test", "build")
SUMMARY = os.path.join(OUT, "all_benches_summary.json")
# SPEC_firmware_v09 section 7 row 21: the ballistic images (v0.8.1 code + the regenerated header; declared as an intended
# header-only change against the committed v0.8.1 images 8a01fa8f / bd47aabb / cb956219).  A different hash is reported
# as a difference for the integrator to declare or fix - never accepted silently.
# fw09 integration (G1 / W1 params.json, agent D): params.json gained flight_computer.pins / nlink.exchange_watchdog_us,
# so FC_PARAMS_TAG (0x95 -> 0x76) and FC_PARAMS_SHA256_PREFIX changed in the ballistic header; the ballistic CODE is the
# v0.8.1 code (D's control build: the step-0 tree with only those two macros edited gives these 3 images byte for byte).
# Before this round: 06304c96 / 1b593274 / 242efa34 (step 0) and 8a01fa8f / bd47aabb / cb956219 (committed v0.8.1).
BALLISTIC_EXPECTED = {
    "nav_ballistic": "eb91c39665ef4ad6c75e433a80c8c1c2b144c46183ee3ec9d31b8c35033cef89",
    "fcu_ballistic": "ef8f9cedacd5083ce683fab127619c5f5dc7c4e82b0ab5fe719945508ca4cdd3",
    "com_ballistic": "e8043085e044b782079b816fda9727a7380602998dbb50040affa70cd8b22da2"}

STEPS = [
    ("gns", [["run_gns_simbench.py", "--profile", "both"]], ["gns_simbench_report.json", "gns_simbench_report_demo.json"]),
    ("nav", [["run_nav_simbench.py", "--mode", "vtol"], ["run_nav_simbench.py", "--mode", "ballistic"]],
     ["nav_simbench_report_vtol.json", "nav_simbench_report_ballistic.json"]),
    ("fcu", [["run_simbench.py", "--mode", "vtol", "--profile", "all"], ["run_simbench.py", "--mode", "vtol", "--profile",
                                                                          "power"],
             ["run_simbench.py", "--mode", "ballistic"]],
     ["simbench_report_vtol.json", "simbench_report_vtol_demo.json", "simbench_report_vtol_nogps.json",
      "simbench_report_vtol_wired.json", "simbench_report_vtol_wired_buffer.json", "simbench_report_ballistic.json"]),
    ("com", [["run_com_simbench.py", "--mode", "both"]], ["com_simbench_report_vtol.json", "com_simbench_report_ballistic.json"]),
    ("cosim", [["run_cosim4.py"]], ["cosim4_report.json"]),
    ("fcu_rejudge", [["run_simbench.py", "--rejudge"]], ["simbench_report_vtol.json", "simbench_report_vtol_demo.json",
                                                         "simbench_report_vtol_nogps.json", "simbench_report_vtol_wired.json",
                                                         "simbench_report_vtol_wired_buffer.json"]),
]


def key_figures(name: str, rep: dict) -> dict:
    """a few figures per report for the summary (the reports hold everything)"""
    out = {"pass": rep.get("pass")}
    if name.startswith("cosim4"):
        out["rows"] = {k: v.get("verdict") for k, v in rep.get("rows", {}).items()}
        return out
    for k, v in rep.items():
        if isinstance(v, dict) and "checks" in v:
            out["failed_checks"] = sorted(c for c, ok in v["checks"].items() if not ok)
            if "bench_self_load" in v:
                out["deferred_to_cosim"] = sorted(v["bench_self_load"].get("deferred", {}))
    if "checks" in rep and isinstance(rep["checks"], dict):
        out["failed_checks"] = sorted(c for c, ok in rep["checks"].items() if not ok)
    return out


def run_step(name: str, cmds: list, reports: list, env: dict, extra: list) -> dict:
    st: dict = {"step": name, "commands": [], "pass": True}
    for c in cmds:
        argv = [sys.executable, "-B", os.path.join(TOOLS, c[0]), *c[1:], *(extra if c[0] == "run_cosim4.py" else [])]
        t0 = time.time()
        r = subprocess.run(argv, cwd=FW, capture_output=True, text=True, encoding="utf-8", errors="replace", env=env)
        log = os.path.join(OUT, f"all_benches_{name}_{len(st['commands'])}.log")
        os.makedirs(OUT, exist_ok=True)
        with open(log, "w", encoding="utf-8", newline="\n") as f:
            f.write(r.stdout + r.stderr)
        st["commands"].append({"argv": " ".join(argv[2:]), "exit": r.returncode, "wall_s": round(time.time() - t0, 1),
                               "log": os.path.relpath(log, ROOT)})
        st["pass"] = st["pass"] and r.returncode == 0
        print(f"[{name}] {' '.join(c)}: exit {r.returncode} ({time.time() - t0:.0f} s)", flush=True)
    st["reports"] = {}
    for rp in reports:
        p = os.path.join(OUT, rp)
        try:
            with open(p, "r", encoding="utf-8") as f:
                st["reports"][rp] = key_figures(rp, json.load(f))
        except (OSError, ValueError) as exc:
            st["reports"][rp] = {"missing": str(exc)}
    return st


def ballistic_hashes(env: dict) -> dict:
    envs = list(BALLISTIC_EXPECTED)
    r = subprocess.run([sys.executable, "-m", "platformio", "run", "-j", env.get("PLATFORMIO_RUN_JOBS", "2"), "-d", FW,
                        *sum((["-e", e] for e in envs), [])], capture_output=True, text=True, encoding="utf-8",
                       errors="replace", env=env)
    out: dict = {"step": "ballistic_hashes", "build_exit": r.returncode, "images": {}, "pass": r.returncode == 0}
    for e in envs:
        hx = os.path.join(FW, ".pio", "build", e, "firmware.hex")
        try:
            with open(hx, "rb") as f:
                h = hashlib.sha256(f.read()).hexdigest()
        except OSError:
            h = None
        same = h == BALLISTIC_EXPECTED[e]
        out["images"][e] = {"sha256": h, "expected": BALLISTIC_EXPECTED[e], "equal": same}
        out["pass"] = out["pass"] and same
        print(f"[ballistic] {e}: {h[:8] if h else None} {'==' if same else '!='} {BALLISTIC_EXPECTED[e][:8]}", flush=True)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--skip", default="", help="steps to skip: gns,nav,fcu,com,cosim,fcu_rejudge,ballistic")
    ap.add_argument("--cosim-jobs", type=int, default=2)
    ap.add_argument("--pio-jobs", type=int, default=2)
    ap.add_argument("--summary", default=SUMMARY)
    ap.add_argument("--cosim-arg", action="append", default=[], help="passed to run_cosim4.py (e.g. --cosim-arg=--extra-tree="
                    "NAME=DIR, --cosim-arg=--wd-pair=A,B for row 19)")
    a = ap.parse_args(argv)
    skip = set(x for x in a.skip.split(",") if x)
    env = {**os.environ, "PYTHONIOENCODING": "utf-8", "PLATFORMIO_RUN_JOBS": str(a.pio_jobs)}
    t0 = time.time()
    steps = []
    for name, cmds, reports in STEPS:
        if name in skip:
            steps.append({"step": name, "skipped": True})
            continue
        steps.append(run_step(name, cmds, reports, env, ["--jobs", str(a.cosim_jobs), "--pio-jobs", str(a.pio_jobs),
                                                        *a.cosim_arg]))
    if "ballistic" not in skip:
        steps.append(ballistic_hashes(env))
    else:
        steps.append({"step": "ballistic_hashes", "skipped": True})
    ok = all(s.get("pass", True) for s in steps if not s.get("skipped"))
    summary = {"what": "SPEC_firmware_v09 WP-B: every bench in the fixed order (GNS -> NAV -> FCU all + power -> COM -> "
                       "co-sim -> FCU re-judge -> ballistic hashes)", "wall_s": round(time.time() - t0, 1), "pass": ok,
               "skipped": sorted(skip), "steps": steps}
    os.makedirs(os.path.dirname(a.summary), exist_ok=True)
    with open(a.summary, "w", encoding="utf-8", newline="\n") as f:
        json.dump(summary, f, indent=1)
    print(f"-> {a.summary}: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
