"""Determinism check of the whole rev 3 chain: two full ``make_all.py`` builds, sha256 of every output compared.

Invariant: a file listed under DETERMINISTIC must come out byte-identical from two builds of the same inputs
(spec/rev3_plan.json, the scripts, the KiCad 10 libraries, the cached JLC / EasyEDA records and the recorded
Freerouting sessions). Nothing is tolerated: one differing byte is a FAIL. Two kinds of files are compared after a
stated normalisation (NORMALISED): the STEP export (its FILE_NAME header carries the export time) and the kicad-cli
netlists (their ``(date ...)`` line). Everything else (EVIDENCE: reports with run times, the schematic PDF, the
kicad-cli PNG renders, the composed slides that are made from those PNGs, the presentation pictures, the manifest)
is evidence of a run, not a deterministic artifact: its hashes are recorded and the number of differing files is
reported, nothing more. For the STEP file an extra content fingerprint is recorded (``step_canonical``: entities
without numbering and order) because kicad-cli 10.0.6 writes the STEP entities in a different order on every export
(M, 2026-11-11); that fingerprint is information only - it does not turn the byte comparison into a PASS. The live Freerouting run is not part of the chain (make_all.py replays the sessions).

The builds run IN PLACE (the files the user orders are the files that were compared). Before the first build the
current files are hashed too ('on_disk_before'): build 1 == on disk shows that the delivered files are what the
chain produces. After the two builds the docs and manifest steps run once more so that README_ko.md, ORDER_ko.md
and out/manifest.json carry this result.

Writes out/reports/determinism.json. Exit 0 = PASS, 1 = FAIL (or a build step failed).

Usage:  cd <board>/scripts && B:/Claude/.venv/Scripts/python.exe -X utf8 -B determinism.py [--no-renders]
        --no-renders is passed through to make_all.py (pictures are evidence anyway; saves about 6 min per build).
A full run takes about two make_all.py runs (about 25-30 min each on this PC): start it in the background.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORT = ROOT / "out" / "reports" / "determinism.json"

#: byte-identical required (globs relative to the board folder)
DETERMINISTIC = [
    "spec/carrier_spec.json", "spec/SPEC_ko.md", "spec/parts_sources.json", "spec/library_probe.json",
    "kicad/*.kicad_sch", "kicad/vtol_carrier.kicad_pro", "kicad/vtol_carrier.kicad_pcb", "kicad/render_*.kicad_pcb",
    "kicad/placement.json", "kicad/schematic_manifest.json", "kicad/ir/*.json",
    "kicad/work/vtol_carrier_unrouted.kicad_pcb", "kicad/work/unrouted_manifest.json",
    "kicad/3d/*.step", "kicad/3d/models_manifest.json",
    "out/jlcpcb_bom.csv", "out/jlcpcb_cpl.csv", "out/jlcpcb_bom_with_mcu.csv", "out/jlcpcb_cpl_with_mcu.csv",
    "out/vtol_carrier_gerber.zip", "out/vtol_carrier_gerber_with_mcu.zip", "out/gerber/*", "out/gerber_with_mcu/*",
    "out/cpl_rotation_table.csv", "out/vtol_carrier_bom.csv", "out/off_board_purchase.csv",
    "out/jlc_fee_estimate.csv", "out/edge_distance.csv",
    "README_ko.md", "out/ORDER_ko.md",
]
#: compared after removing the named time stamp
NORMALISED = {
    "out/vtol_carrier.step": "STEP header FILE_NAME(...) statement removed (it carries the export time)",
    "out/vtol_carrier.net": "netlist (date \"...\") line removed",
    "kicad/work/vtol_carrier.net": "netlist (date \"...\") line removed",
}
#: evidence: hashed and counted, never judged
EVIDENCE = ["out/reports/*.json", "out/reports/*.png", "out/*.json", "out/*.png", "out/schematic/*",
            "out/render/*.png", "out/render/*.json", "out/presentation/*", "kicad/work/*.json"]

_STEP_FILE_NAME = re.compile(rb"FILE_NAME\(.*?\);", re.S)
_STEP_ENTITY = re.compile(rb"#\d+\s*=\s*(.*?);\s*(?=#\d+\s*=|ENDSEC)", re.S)
_NET_DATE = re.compile(rb'\(date "[^"]*"\)')


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def normalised_bytes(rel: str, data: bytes) -> bytes:
    if rel.endswith(".step"):
        head, sep, rest = data.partition(b"ENDSEC;")
        return _STEP_FILE_NAME.sub(b"FILE_NAME(<removed>);", head) + sep + rest
    if rel.endswith(".net"):
        return _NET_DATE.sub(b'(date "<removed>")', data)
    return data


def step_canonical(data: bytes) -> str:
    """Content fingerprint of a STEP file that ignores entity numbering and order: the sorted multiset of the DATA
    entities with every #reference replaced by '#' and white space removed. Weaker than byte identity (it cannot see
    which entity points to which); used only to tell 'same content, other order' from 'other content'."""
    body = data.split(b"DATA;", 1)[1] if b"DATA;" in data else data
    ents = sorted(re.sub(rb"#\d+", b"#", re.sub(rb"\s+", b"", e)) for e in _STEP_ENTITY.findall(body))
    return sha(b"\n".join(ents)) + f" ({len(ents)} entities)"


def expand(globs: list[str]) -> list[str]:
    out: set[str] = set()
    for g in globs:
        for p in ROOT.glob(g):
            if p.is_file():
                out.add(p.relative_to(ROOT).as_posix())
    return sorted(out)


def snapshot() -> dict[str, dict[str, str]]:
    det = {rel: sha((ROOT / rel).read_bytes()) for rel in expand(DETERMINISTIC)}
    norm = {}
    for rel in NORMALISED:
        p = ROOT / rel
        if p.is_file():
            data = p.read_bytes()
            norm[rel] = {"raw": sha(data), "normalised": sha(normalised_bytes(rel, data))}
            if rel.endswith(".step"):
                norm[rel]["canonical"] = step_canonical(data)
    ev = {rel: sha((ROOT / rel).read_bytes()) for rel in expand(EVIDENCE)
          if rel not in det and rel != "out/reports/determinism.json"}
    return {"deterministic": det, "normalised": norm, "evidence": ev}


def build(extra: list[str]) -> dict:
    t0 = time.time()
    cmd = [sys.executable, "-B", "-X", "utf8", str(HERE / "make_all.py"), *extra]
    r = subprocess.run(cmd, cwd=str(HERE), env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"), text=True,
                       encoding="utf-8", errors="replace", capture_output=True)
    log = json.loads((ROOT / "kicad" / "work" / "make_all_log.json").read_text(encoding="utf-8"))
    return {"exit": r.returncode, "minutes": round((time.time() - t0) / 60, 1), "failed_checks": log["failed_checks"],
            "stopped_at": log.get("stopped_at"), "steps": [(s["step"], s["exit"], s["seconds"]) for s in log["steps"]],
            "stdout_tail": (r.stdout or "")[-800:]}


def compare(a: dict[str, str], b: dict[str, str]) -> dict:
    keys = sorted(set(a) | set(b))
    return {"files": len(keys), "differing": [k for k in keys if a.get(k) != b.get(k)]}


def main() -> int:
    extra = ["--no-renders"] if "--no-renders" in sys.argv else []
    started = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    before = snapshot()
    runs = []
    snaps = []
    for _ in range(2):
        runs.append(build(extra))
        if runs[-1]["exit"] not in (0, 2):
            break
        snaps.append(snapshot())
    res: dict = {"tool": "scripts/determinism.py (two full make_all.py builds in place, sha256 compared)",
                 "started": started, "make_all_args": extra, "builds": runs,
                 "rules": {"deterministic": DETERMINISTIC, "normalised": NORMALISED, "evidence": EVIDENCE,
                           "evidence_note": "reports carry run times; kicad-cli PNG renders differ in bytes between "
                                            "runs with the same content (M, renders step); slides are composed from "
                                            "those PNGs; the schematic PDF carries a creation date. Evidence is "
                                            "recorded, not judged."}}
    if len(snaps) < 2:
        res["result"] = "FAIL"
        res["why"] = f"build {len(snaps) + 1} stopped (make_all exit {runs[-1]['exit']}, step {runs[-1]['stopped_at']})"
    else:
        b1, b2 = snaps
        det = compare(b1["deterministic"], b2["deterministic"])
        norm = compare({k: v["normalised"] for k, v in b1["normalised"].items()},
                       {k: v["normalised"] for k, v in b2["normalised"].items()})
        ev = compare(b1["evidence"], b2["evidence"])
        canon = compare({k: v["canonical"] for k, v in b1["normalised"].items() if "canonical" in v},
                        {k: v["canonical"] for k, v in b2["normalised"].items() if "canonical" in v})
        res["build1_vs_build2"] = {"deterministic": det, "normalised": norm, "step_canonical": canon,
                                   "evidence": {"files": ev["files"], "differing_count": len(ev["differing"]),
                                                "differing": ev["differing"]}}
        res["on_disk_before_vs_build1"] = {
            "deterministic": compare(before["deterministic"], b1["deterministic"]),
            "normalised": compare({k: v["normalised"] for k, v in before["normalised"].items()},
                                  {k: v["normalised"] for k, v in b1["normalised"].items()})}
        res["sha256_build2"] = {"deterministic": b2["deterministic"], "normalised": b2["normalised"]}
        res["sha256_on_disk_before"] = {"deterministic": before["deterministic"], "normalised": before["normalised"]}
        res["evidence_sha256_build2"] = b2["evidence"]
        res["result_by_group"] = {
            "deterministic (byte-identical)": "PASS" if not det["differing"] else "FAIL",
            "normalised (STEP without FILE_NAME header, netlists without date)": "PASS" if not norm["differing"] else "FAIL",
            "step_canonical (entities without numbering / order, information)": "PASS" if not canon["differing"] else "FAIL"}
        res["result"] = "PASS" if not det["differing"] and not norm["differing"] else "FAIL"
        if not det["differing"] and norm["differing"]:
            res["why"] = ("byte comparison after the stated normalisation failed for " + ", ".join(norm["differing"])
                          + "; kicad-cli 10.0.6 'pcb export step' writes the STEP entities in a different order on every "
                          "run (M: two exports of the same board differ in about 47000 lines beyond the FILE_NAME header). "
                          "Entity content without numbering / order: "
                          + ("equal" if not canon["differing"] else "DIFFERENT") + ". No rule was relaxed: the plan's "
                          "STEP check is reported as FAIL.")
    res["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(res, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    # put the result into the documents and the manifest (these two steps read out/reports/determinism.json)
    post = subprocess.run([sys.executable, "-B", "-X", "utf8", str(HERE / "make_all.py"), "--only", "docs,manifest"],
                          cwd=str(HERE), env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"), text=True,
                          encoding="utf-8", errors="replace", capture_output=True)
    print(json.dumps({"result": res["result"], "builds": [{k: r[k] for k in ("exit", "minutes", "failed_checks")}
                                                          for r in runs],
                      "build1_vs_build2": {k: v["differing"] if "differing" in v else v.get("differing_count")
                                           for k, v in res.get("build1_vs_build2", {}).items()},
                      "on_disk_before_vs_build1": {k: v["differing"]
                                                   for k, v in res.get("on_disk_before_vs_build1", {}).items()},
                      "post_docs_manifest_exit": post.returncode, "report": REPORT.as_posix()}, indent=1))
    return 0 if res["result"] == "PASS" and post.returncode == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
