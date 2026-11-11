"""Add the 'build' section to out/manifest.json (the plan's out/MANIFEST.json: on Windows the two names are one file).

Invariant: every value written here is read from a file or a tool run in this call (sha256 of a file, kicad-cli's own
version string, a field of a report) - nothing is typed in. A value whose source is missing is written as
NOT_VERIFIED with the reason. checks_exports.py owns the rest of the manifest (order files, release gate) and is not
changed; this step only (re)writes the key 'build', so it can run any number of times.

The 'build' section carries: plan / spec sha256, option A / B, kicad-cli version, Freerouting jar sha256, the rule-file
comparison, the JLC gate snapshot time, the determinism result of scripts/determinism.py (and whether the files on
disk still equal its second build), the sha256 of the deterministic artifacts as they are now, the files written after
checks_exports.py hashed its list (rev3 export reports, documents, renders - the pictures as evidence), and the
make_all.py log.

Run: cd <board>/scripts && B:/Claude/.venv/Scripts/python.exe -X utf8 -B make_manifest.py   (step 'manifest' of make_all)
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import determinism as DET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "out"
REP = OUT / "reports"
MANIFEST = OUT / "manifest.json"
KICAD_CLI = Path(os.environ.get("KICAD_CLI") or r"C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/kicad-cli.exe")
#: the same default as scripts/route.py JAR (environment FREEROUTING_JAR overrides both)
JAR = Path(os.environ.get("FREEROUTING_JAR") or
           r"C:/Users/yc012/AppData/Local/Temp/claude/B--Claude/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/tools_dl/freerouting/freerouting-2.5.0.jar")
#: written after checks_exports.py made its sha list (so not in manifest['sha256'])
LATE = ["out/reports/rev3_export_files.json", "out/reports/rev3_export_determinism.json",
        "out/reports/determinism.json", "README_ko.md", "out/ORDER_ko.md", "spec/SPEC_ko.md",
        "out/REV3_REPORT_ko.md"]


def sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load(p: Path) -> dict | None:
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def kicad_version() -> str:
    try:
        r = subprocess.run([str(KICAD_CLI), "version"], capture_output=True, text=True, encoding="utf-8",
                           errors="replace", timeout=120)
        return r.stdout.strip() or f"NOT_VERIFIED (exit {r.returncode})"
    except OSError as e:
        return f"NOT_VERIFIED ({e})"


def main() -> int:
    man = load(MANIFEST)
    if man is None:
        raise SystemExit("out/manifest.json missing - run checks_exports.py first")
    spec = load(ROOT / "spec" / "carrier_spec.json") or {}
    rr = load(REP / "routing_rules.json") or {}
    gate = load(OUT / "jlc_gate_report.json") or {}
    src = load(ROOT / "spec" / "parts_sources.json") or {}
    det = load(REP / "determinism.json")
    log = load(ROOT / "kicad" / "work" / "make_all_log.json")
    route_log = load(ROOT / "kicad" / "work" / "route_log.json") or {}
    replay = load(ROOT / "kicad" / "work" / "ses_replay.json") or {}
    now = DET.snapshot()

    if det is None:
        det_out: dict = {"result": "NOT_VERIFIED", "why": "out/reports/determinism.json missing - run scripts/determinism.py"}
    elif "sha256_build2" not in det:
        det_out = {"result": det.get("result"), "why": det.get("why"), "finished": det.get("finished")}
    else:
        want = det["sha256_build2"]["deterministic"]
        drift = sorted(k for k in set(want) | set(now["deterministic"]) if want.get(k) != now["deterministic"].get(k))
        want_n = {k: v["normalised"] for k, v in det["sha256_build2"]["normalised"].items()}
        drift += sorted(k for k in want_n if want_n[k] != now["normalised"].get(k, {}).get("normalised"))
        det_out = {"result": det["result"], "result_by_group": det.get("result_by_group"), "why": det.get("why"),
                   "step_canonical_differing": det["build1_vs_build2"].get("step_canonical", {}).get("differing"),
                   "started": det.get("started"), "finished": det.get("finished"),
                   "make_all_args": det.get("make_all_args"),
                   "build_minutes": [b.get("minutes") for b in det.get("builds", [])],
                   "deterministic_files_compared": det["build1_vs_build2"]["deterministic"]["files"],
                   "deterministic_differing": det["build1_vs_build2"]["deterministic"]["differing"],
                   "normalised_compared": det["build1_vs_build2"]["normalised"]["files"],
                   "normalised_differing": det["build1_vs_build2"]["normalised"]["differing"],
                   "evidence_files": det["build1_vs_build2"]["evidence"]["files"],
                   "evidence_differing_count": det["build1_vs_build2"]["evidence"]["differing_count"],
                   "on_disk_before_vs_build1_differing":
                       det["on_disk_before_vs_build1"]["deterministic"]["differing"]
                       + det["on_disk_before_vs_build1"]["normalised"]["differing"],
                   "files_on_disk_now_equal_build2": not [k for k in drift if k not in ("README_ko.md", "out/ORDER_ko.md")],
                   "changed_since_build2": drift,
                   "changed_since_build2_note": "README_ko.md / out/ORDER_ko.md are regenerated by determinism.py after "
                                                "the two builds so that they carry its result; every other listed "
                                                "file was changed after the check",
                   "report": (REP / "determinism.json").as_posix()}

    rf = (rr.get("checks") or {}).get("rule_file") or {}
    jar = {"path": JAR.as_posix(), "exists": JAR.is_file()}
    if JAR.is_file():
        jar.update(sha256=sha256(JAR), bytes=JAR.stat().st_size)
    jar["live_run_jar_sha256"] = route_log.get("freerouting_jar_sha256") \
        or "NOT_VERIFIED (not found in kicad/work/route_log.json)"
    if JAR.is_file():
        jar["equals_live_run_jar"] = jar["sha256"] == jar["live_run_jar_sha256"]
    late = {}
    for rel in LATE:
        p = ROOT / rel
        late[rel] = sha256(p) if p.is_file() else "missing"
    pictures = {p.relative_to(ROOT).as_posix(): sha256(p)
                for d in ("out/render", "out/presentation") for p in sorted((ROOT / d).glob("*.png"))}
    build = {
        "what": "build provenance (scripts/make_manifest.py); out/MANIFEST.json named in the plan = this file "
                "(file names are case-insensitive on Windows, so one file carries both)",
        "plan": {"json": "spec/rev3_plan.json", "json_sha256": sha256(ROOT / "spec" / "rev3_plan.json"),
                 "md": "spec/REV3_PLAN.md", "md_sha256": sha256(ROOT / "spec" / "REV3_PLAN.md")},
        "spec_sha256": sha256(ROOT / "spec" / "carrier_spec.json"),
        "prog_headers_option": (spec.get("prog_headers") or {}).get("option", "NOT_VERIFIED"),
        "prog_headers_switch": "environment VTOL_PROG_HEADERS (default A); value at this run: "
                               + os.environ.get("VTOL_PROG_HEADERS", "(unset = A)"),
        "kicad_cli": {"path": KICAD_CLI.as_posix(), "version": kicad_version()},
        "freerouting_jar": jar,
        "routing": {"mode": "replay of the recorded sessions (route.py --reuse-ses)",
                    "ses_replay_sha256": sha256(ROOT / "kicad" / "work" / "ses_replay.json"),
                    "unrouted_board_sha256_of_the_sessions": replay.get("unrouted_board_sha256"),
                    "live_run_source": replay.get("source")},
        "rule_file": {"result": rf.get("result", "NOT_VERIFIED"),
                      "sections": {k: {"equal": v.get("equal"), "sha256": v.get("sha256_rev3")}
                                   for k, v in (rf.get("sections") or {}).items()},
                      "kicad_pro_sha256": sha256(ROOT / "kicad" / "vtol_carrier.kicad_pro"),
                      "note": "design-rule sections compared with the archived rev 2c project file by "
                              "scripts/routing_checks.py; the raw .kicad_pro bytes differ because pcbnew rewrites the file"},
        "jlc_gate": {"report": "out/jlc_gate_report.json", "status": gate.get("status", "NOT_VERIFIED"),
                     "snapshot_started_at": gate.get("started_at"), "mode": gate.get("mode"),
                     "spec_sha256_matches": gate.get("spec_sha256") == sha256(ROOT / "spec" / "carrier_spec.json"),
                     "thin_lines": gate.get("thin_lines"),
                     "note": "stock is a snapshot: run scripts/jlc_gate.py again on the order day"},
        "parts_sources": {"file": "spec/parts_sources.json", "snapshot_day": src.get("today"),
                          "status": src.get("status")},
        "release_gate": man.get("release_gate", {}).get("status"),
        "release_gate_reasons": man.get("release_gate", {}).get("reasons"),
        "determinism": det_out,
        "deterministic_artifacts_sha256_now": now["deterministic"],
        "normalised_artifacts_sha256_now": now["normalised"],
        "written_after_checks_exports": late,
        "pictures_evidence_sha256": pictures,
        "pictures_note": "kicad-cli PNG renders are not byte-identical between runs (same content): evidence, not "
                         "deterministic artifacts",
        "make_all_log": None if log is None else {
            "exit": log.get("exit"), "failed_checks": log.get("failed_checks"), "stopped_at": log.get("stopped_at"),
            "started": log.get("started"), "finished": log.get("finished"),
            "steps": [(s["step"], s["exit"], s["seconds"]) for s in log.get("steps", [])]},
    }
    man["build"] = build
    MANIFEST.write_text(json.dumps(man, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({k: build[k] for k in ("spec_sha256", "prog_headers_option", "release_gate")}
                     | {"kicad_cli": build["kicad_cli"]["version"], "jar_sha256": jar.get("sha256"),
                        "rule_file": build["rule_file"]["result"], "gate": build["jlc_gate"]["snapshot_started_at"],
                        "determinism": det_out.get("result"),
                        "on_disk_equal_build2": det_out.get("files_on_disk_now_equal_build2")}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
