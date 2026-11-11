"""Rebuild the whole rev 3 carrier from spec/rev3_plan.json, except the live Freerouting run.

Invariant: this script only CALLS the generators and checks in their fixed order; it decides nothing itself. Every
output comes from the script named in its step, every PASS / FAIL from the report that script writes. The routed
board is the replay of the recorded Freerouting sessions (kicad/work/ses_replay.json, ``route.py --reuse-ses``),
which route.py refuses if the unrouted board differs from the one the sessions were recorded on - a changed
placement needs a new live route (``--reroute``), which is NOT deterministic and is never run by default.
Network access: none. The JLC parts evidence is re-evaluated from the cache (jlc_sources.py without --online, with
the snapshot day of the existing spec/parts_sources.json); the order-day stock gate scripts/jlc_gate.py is online
and stays a separate, explicit command.

Steps (name: interpreter script args) - ``--list`` prints them:
  probe      PY  probe_library.py                 KiCad 10 libraries -> spec/library_probe.json
  sources    PY  jlc_sources.py --today <day>     cached JLC records -> spec/parts_sources.json (no network)
  spec       PY  build_spec.py                    spec/carrier_spec.json, spec/SPEC_ko.md
  libgate    PY  probe_library.py gate            out/reports/library_gate.json
  schematic  PY  build_schematic.py               kicad/*.kicad_sch, .kicad_pro, IR
  sch_checks PY  rev3_checks.py all               ERC (kicad-cli) + netlist checks + spec/schematic determinism
  place      PY  place.py                         kicad/placement.json
  pcb        PY  build_pcb.py                     kicad/work/vtol_carrier_unrouted.kicad_pcb
  route      KPY route.py --reuse-ses             session replay + close_leftovers + rip-up + stitching + silk
                                                  -> kicad/vtol_carrier.kicad_pcb (kicad-cli DRC inside the clean-up)
  routing    KPY routing_checks.py                out/reports/routing_rules.json (plan section 13 rules)
  edge       KPY edge_distance.py                 out/edge_distance.csv, out/reports/edge_distance.json
  rotation   KPY cpl_rotation.py                  out/cpl_rotation_table.csv (cached EasyEDA records, no network)
  fit        KPY fit_checks.py                    out/fit_checks.json
  prog       PY  prog_access.py                   out/reports/programming_access.json (decision D6 search)
  prog_plot  MPY prog_access_plot.py              out/reports/programming_access.png
  exports    PY  checks_exports.py                ERC, DRC, board == IR, board_metrics, BOM / CPL / gerbers / STEP /
                                                  PDF, fees, off-board list, out/manifest.json + release gate
  exp_files  PY  rev3_checks.py export-files      out/reports/rev3_export_files.json
  exp_det    PY  rev3_checks.py export-determinism out/reports/rev3_export_determinism.json
  models     MPY make_module_models.py            kicad/3d/*.step (CadQuery)
  renders    KPY render_rev3.py --quality high    out/render/a1..d1 + render_*.kicad_pcb
  d2         MPY render_d2_scene.py               out/render/d2_neo7m_top_cap.png
  slides     KPY render_rev3.py --slides-only     out/render/e1..e4
  present    KPY make_presentation.py             out/presentation/*.png
  docs       PY  docs_ko.py                       README_ko.md, out/ORDER_ko.md (numbers read from the reports)
  manifest   PY  make_manifest.py                 'build' section of out/manifest.json (= MANIFEST.json of the plan)

Interpreters: PY = this python (the AI_EDA venv B:/Claude/.venv, package ai_eda); KPY = KiCad's python
(C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/python.exe, pcbnew); MPY = $CARRIER_MODELS_PY, else the main
checkout's venv B:/0)AI_Cad/CanSat_EDF_Rocket/.venv (CadQuery / PyVista / matplotlib; only executed - the scripts and
their outputs stay in this tree). Every call gets -B -X utf8 and PYTHONDONTWRITEBYTECODE=1.

Exit code: 0 = every step ran and every check step passed; 2 = every step ran but a check step reported a failure
(the release gate in out/manifest.json then says NOT ORDERABLE and names it); 1 = a build step failed (stopped).
The log of the run (times, exit codes) is kicad/work/make_all_log.json (evidence, not a deterministic artifact).

Usage:  cd <board>/scripts && B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_all.py
            [--list] [--from STEP] [--only STEP,STEP] [--skip STEP,STEP] [--no-renders] [--reroute]
  --no-renders  skips models, renders, d2, slides, present (pictures only; every order file is still built)
  --reroute     live Freerouting (route.py 40 5 with the ECO seed kicad/work/seed_a7 = sessions of attempt a7, as the
                adopted relayout run) instead of
                the replay: not deterministic, writes new sessions; use only after a placement change.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
KPY = Path(r"C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/python.exe")
MPY = Path(os.environ.get("CARRIER_MODELS_PY") or r"B:/0)AI_Cad/CanSat_EDF_Rocket/.venv/Scripts/python.exe")
LOG = ROOT / "kicad" / "work" / "make_all_log.json"
PICTURES = ("models", "renders", "d2", "slides", "present")


def sources_day() -> str:
    """Snapshot day of the existing spec/parts_sources.json: re-evaluating the same cache with the same day gives a
    byte-identical file. A missing file falls back to today (then a stale record is an error, by design)."""
    p = ROOT / "spec" / "parts_sources.json"
    try:
        return str(json.loads(p.read_text(encoding="utf-8"))["today"])
    except (OSError, ValueError, KeyError):
        return time.strftime("%Y-%m-%d")


def steps(reroute: bool) -> list[tuple[str, str, list[str], str]]:
    """(name, interpreter key, arguments, kind); kind 'build' stops the run on failure, 'check' is recorded."""
    route = ["route.py", "40", "5"] if reroute else ["route.py", "--reuse-ses"]
    return [
        ("probe", "PY", ["probe_library.py"], "build"),
        ("sources", "PY", ["jlc_sources.py", "--today", sources_day()], "build"),
        ("spec", "PY", ["build_spec.py"], "build"),
        ("libgate", "PY", ["probe_library.py", "gate"], "check"),
        ("schematic", "PY", ["build_schematic.py"], "build"),
        ("sch_checks", "PY", ["rev3_checks.py", "all"], "check"),
        ("place", "PY", ["place.py"], "build"),
        ("pcb", "PY", ["build_pcb.py"], "build"),
        ("route", "KPY", route, "build"),
        ("routing", "KPY", ["routing_checks.py"], "check"),
        ("edge", "KPY", ["edge_distance.py"], "check"),
        ("rotation", "KPY", ["cpl_rotation.py"], "build"),
        ("fit", "KPY", ["fit_checks.py"], "check"),
        ("prog", "PY", ["prog_access.py"], "check"),
        ("prog_plot", "MPY", ["prog_access_plot.py"], "build"),
        ("exports", "PY", ["checks_exports.py"], "build"),
        ("exp_files", "PY", ["rev3_checks.py", "export-files"], "check"),
        ("exp_det", "PY", ["rev3_checks.py", "export-determinism"], "check"),
        ("models", "MPY", ["make_module_models.py"], "build"),
        ("renders", "KPY", ["render_rev3.py", "--quality", "high"], "build"),
        ("d2", "MPY", ["render_d2_scene.py"], "build"),
        ("slides", "KPY", ["render_rev3.py", "--slides-only"], "build"),
        ("present", "KPY", ["make_presentation.py"], "build"),
        ("docs", "PY", ["docs_ko.py"], "build"),
        ("manifest", "PY", ["make_manifest.py"], "build"),
    ]


def interpreter(key: str) -> list[str]:
    exe = {"PY": Path(sys.executable), "KPY": KPY, "MPY": MPY}[key]
    if not exe.is_file():
        raise SystemExit(f"interpreter {key} not found: {exe}")
    return [str(exe), "-B", "-X", "utf8"]


def arg_list(flag: str) -> list[str]:
    if flag not in sys.argv:
        return []
    i = sys.argv.index(flag)
    if i + 1 >= len(sys.argv):
        raise SystemExit(f"{flag} needs a value")
    return [x.strip() for x in sys.argv[i + 1].split(",") if x.strip()]


def main() -> int:
    try:
        import ai_eda  # noqa: F401
    except ImportError:
        raise SystemExit("run make_all.py with the AI_EDA venv python (B:/Claude/.venv/Scripts/python.exe)")
    reroute = "--reroute" in sys.argv
    plan = steps(reroute)
    names = [s[0] for s in plan]
    if "--list" in sys.argv:
        for name, key, args, kind in plan:
            print(f"{name:<11} {key:<4} {kind:<6} {' '.join(args)}")
        return 0
    chosen = names
    start = arg_list("--from")
    only = arg_list("--only")
    skip = set(arg_list("--skip")) | (set(PICTURES) if "--no-renders" in sys.argv else set())
    for n in [*start, *only, *skip]:
        if n not in names:
            raise SystemExit(f"unknown step {n!r}; steps: {', '.join(names)}")
    if start:
        chosen = names[names.index(start[0]):]
    if only:
        chosen = [n for n in names if n in only]
    chosen = [n for n in chosen if n not in skip]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    if reroute:
        # relayout step: the adopted board = a7 sessions as ECO seed (nets with changed pads or conflicts start
        # unrouted; in the adopted run 97 of 105 nets started with their a7 routes) + the normal stages + the clean-up
        # with rip-up. The clean-up changes further nets (review RR-6, M: ESP_TXD0, COM_LED, I2C_SCL_3V3 and the nets
        # under ripup.nets_changed in kicad/work/route_reuse_log.json) - 'seeded' does not mean 'untouched'
        env.update(CARRIER_SEED_DIR=str(ROOT / "kicad" / "work" / "seed_a7"))
    log = {"tool": "scripts/make_all.py", "argv": sys.argv[1:], "reroute": reroute,
           "interpreters": {"PY": Path(sys.executable).as_posix(), "KPY": KPY.as_posix(), "MPY": MPY.as_posix()},
           "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "steps": []}
    failed_checks: list[str] = []
    rc = 0
    for name, key, args, kind in plan:
        if name not in chosen:
            continue
        cmd = interpreter(key) + args
        print(f">> [{name}] {' '.join(cmd)}", flush=True)
        t0 = time.time()
        r = subprocess.run(cmd, cwd=str(HERE), env=env, text=True, encoding="utf-8", errors="replace",
                           capture_output=True)
        dt = round(time.time() - t0, 1)
        print((r.stdout or "")[-1500:], flush=True)
        if r.stderr and r.returncode != 0:
            print(r.stderr[-1500:], file=sys.stderr, flush=True)
        log["steps"].append({"step": name, "kind": kind, "command": [Path(cmd[0]).name, *cmd[1:]],
                             "exit": r.returncode, "seconds": dt})
        if r.returncode != 0:
            if kind == "build":
                log["stopped_at"] = name
                log["stderr_tail"] = (r.stderr or "")[-3000:]
                log["stdout_tail"] = (r.stdout or "")[-3000:]
                rc = 1
                break
            failed_checks.append(f"{name} (exit {r.returncode})")
        print(f"   [{name}] exit {r.returncode}, {dt} s", flush=True)
    log["failed_checks"] = failed_checks
    log["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    if rc == 0 and failed_checks:
        rc = 2
    log["exit"] = rc
    LOG.parent.mkdir(parents=True, exist_ok=True)
    LOG.write_text(json.dumps(log, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"exit": rc, "failed_checks": failed_checks, "stopped_at": log.get("stopped_at"),
                      "log": LOG.as_posix()}, indent=1))
    return rc


if __name__ == "__main__":
    sys.exit(main())
