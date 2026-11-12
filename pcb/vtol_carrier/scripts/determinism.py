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

Second folder (relayout-fix step, review RR-2): ``--cross-folder <dir>`` copies this board folder (without the
pictures) and the repository files it reads to <dir>, runs ``make_all.py --no-renders`` there and compares every file
of the deterministic set that the copy rebuilt with this folder's second build. Boards, schematic sheets, gerbers,
zips, BOM and CPL must be equal byte for byte. Files that state their folder (spec/carrier_spec.json 'repository',
the Korean documents, the netlists) or carry the sha256 of such a file (placement.json, the manifests) are compared
after the folder name / those sha256 values are replaced by this folder's (cross_folder() docstring). A difference
is a FAIL of the whole check; without the switch the group is NOT_VERIFIED. Until this existed, byte identity was only shown inside one folder
(M: a replay in a scratch folder gave the same copper with other track uuids).

The JLC stock snapshot is bound to the spec's sha256, and the spec states its checkout: the clone's copy of
out/jlc_gate_report.json is given the clone's spec sha256 only after the clone's spec was built and found equal to
this folder's spec apart from the folder name. ``--cross-only`` keeps the two in-place builds of the existing report
and (re)runs only the second-folder part (refused if the files changed since build 2).

Usage:  cd <board>/scripts && B:/Claude/.venv/Scripts/python.exe -X utf8 -B determinism.py [--no-renders] [--cross-folder <dir> [--cross-only]]
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
    "README_ko.md", "out/ORDER_ko.md", "out/따로_살_부품_ko.md",
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


# ------------------------------------------------------------------------------------------- second folder (RR-2)
CLONE_MARK = ".determinism_clone"
#: picture steps are not run in the clone (make_all.py --no-renders): their outputs are copies, never compared
CLONE_SKIP_DIRS = ("out/render", "out/presentation")
#: kicad-cli netlists carry the schematic's full path; rev3_checks.py compares a fresh export with the file on disk,
#: so the copies from this folder are removed in the clone (they are written again by the build)
CLONE_DROP = ("out/vtol_carrier.net", "kicad/work/vtol_carrier.net")
#: the files a fab order is made of, the boards and the schematic: must be equal WITHOUT any renaming
MUST_BE_RAW_IDENTICAL = ("kicad/vtol_carrier.kicad_pcb", "kicad/work/vtol_carrier_unrouted.kicad_pcb",
                         "out/vtol_carrier_gerber.zip", "out/vtol_carrier_gerber_with_mcu.zip", "out/jlcpcb_bom.csv",
                         "out/jlcpcb_cpl.csv", "out/jlcpcb_bom_with_mcu.csv", "out/jlcpcb_cpl_with_mcu.csv",
                         "out/cpl_rotation_table.csv")
MUST_BE_RAW_PREFIX = ("out/gerber/", "out/gerber_with_mcu/")
MUST_BE_RAW_SUFFIX = (".kicad_sch",)


def repo_dir() -> Path:
    """The checkout whose params.json / cad files the chain reads: <repo>/pcb/vtol_carrier as before, or (redesign2,
    scratch copy of the board folder without a checkout around it) the folder named by $CARRIER_PROJ."""
    return Path(os.environ["CARRIER_PROJ"]) if os.environ.get("CARRIER_PROJ") else ROOT.parents[1]


def folder_tokens(dst: Path, croot: Path) -> list[tuple[bytes, bytes]]:
    """Renaming pairs clone -> this folder: the clone's board folder -> this board folder first, then the clone root ->
    the checkout. In the repository layout (ROOT = repo/pcb/vtol_carrier) this is the same as path_tokens(dst, repo)."""
    return path_tokens(croot, ROOT) + path_tokens(dst, repo_dir())


def path_tokens(a: Path, b: Path) -> list[tuple[bytes, bytes]]:
    """Every spelling of folder ``a`` paired with the same spelling of folder ``b``: forward slashes (JSON, Markdown),
    backslashes, doubled backslashes (kicad-cli netlist)."""
    forms = (lambda q: q.as_posix(), lambda q: q.as_posix().replace("/", "\\"), lambda q: q.as_posix().replace("/", "\\\\"))
    return [(f(a).encode("utf-8"), f(b).encode("utf-8")) for f in forms]


def clone(dst: Path) -> tuple[Path, dict]:
    """Copy this board folder (without the pictures) and the few repository files the chain reads outside it
    (params.json, cad/layout.py, cad/exports/*.json, cad/livery/) to ``dst/pcb/vtol_carrier``. ``dst`` must not exist,
    be empty, or be an earlier clone of this tool (marker file) - nothing else is ever deleted.

    The kicad-cli netlists are dropped (CLONE_DROP) and reported."""
    import shutil

    repo = repo_dir()
    if dst.exists() and any(dst.iterdir()):
        if not (dst / CLONE_MARK).is_file():
            raise SystemExit(f"--cross-folder: {dst} is not empty and was not made by determinism.py")
        shutil.rmtree(dst)
    croot = dst / "pcb" / ROOT.name
    skip = {(ROOT / d).resolve() for d in CLONE_SKIP_DIRS}

    def ignore(folder: str, names: list[str]) -> list[str]:
        return [n for n in names if (Path(folder) / n).resolve() in skip or n == "__pycache__"]
    shutil.copytree(ROOT, croot, ignore=ignore)
    (dst / CLONE_MARK).write_text("scratch copy made by scripts/determinism.py --cross-folder\n", encoding="utf-8", newline="\n")
    shutil.copyfile(repo / "params.json", dst / "params.json")
    (dst / "cad" / "exports").mkdir(parents=True)
    shutil.copyfile(repo / "cad" / "layout.py", dst / "cad" / "layout.py")
    for p in sorted((repo / "cad" / "exports").glob("*.json")):
        shutil.copyfile(p, dst / "cad" / "exports" / p.name)
    # cad/layout.py reads the livery polygons named in params.json (cad_envelope.py, step 'exports')
    shutil.copytree(repo / "cad" / "livery", dst / "cad" / "livery", ignore=shutil.ignore_patterns("__pycache__"))
    dropped = []
    for rel in CLONE_DROP:
        if (croot / rel).is_file():
            (croot / rel).unlink()
            dropped.append(rel)
    return croot, {"netlists_dropped": dropped}


def cross_folder(dst: Path, here: dict[str, dict]) -> dict:
    """Build the same inputs once more in another folder and compare the deterministic set with this folder's last
    build (review RR-2: byte identity had only been shown for two builds in the same folder).

    Only files the clone build wrote itself are compared (modification time after the build start); the rest are
    listed. Three classes, nothing else is tolerated:
      identical                    same bytes
      identical_after_renaming     same bytes after (1) every spelling of the clone's folder is replaced by this folder
                                   and (2) the sha256 of another file of the set, where the clone's file differs, is
                                   replaced by this folder's sha256 (spec/carrier_spec.json states its checkout, and
                                   placement.json / the manifests carry the sha256 of the spec)
      differing                    anything else
    PASS = nothing differing AND every board, schematic sheet, gerber, zip, BOM and CPL is in the first class."""
    croot, adapted = clone(dst)
    repo = repo_dir()
    t0 = time.time()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    make = [sys.executable, "-B", "-X", "utf8", str(croot / "scripts" / "make_all.py")]
    # The JLC stock snapshot out/jlc_gate_report.json is bound to the sha256 of spec/carrier_spec.json, and the spec
    # states its checkout (key 'repository'): in another folder the same design has another spec sha256 and the gate
    # would read 'STALE (other spec)' (M). So the clone's spec is built first; ONLY IF it equals this folder's spec
    # once the folder name is put back, the clone's copy of the snapshot gets the clone's spec sha256. Otherwise
    # nothing is touched, the gate reads STALE there and the documents differ - which the comparison reports.
    pre = subprocess.run(make + ["--only", "probe,sources,spec"], cwd=str(croot / "scripts"), env=env, text=True,
                         encoding="utf-8", errors="replace", capture_output=True)
    spec_here = (ROOT / "spec" / "carrier_spec.json").read_bytes()
    spec_there = (croot / "spec" / "carrier_spec.json").read_bytes()
    back = spec_there
    for a, b in folder_tokens(dst, croot):
        back = back.replace(a, b)
    gate = croot / "out" / "jlc_gate_report.json"
    adapted["spec_step_exit"] = pre.returncode
    adapted["clone_spec_equals_this_spec_apart_from_the_folder_name"] = back == spec_here
    adapted["jlc_gate_report_translated"] = False
    if pre.returncode == 0 and back == spec_here and gate.is_file() and sha(spec_here).encode() in gate.read_bytes():
        gate.write_bytes(gate.read_bytes().replace(sha(spec_here).encode(), sha(spec_there).encode()))
        adapted["jlc_gate_report_translated"] = True
        adapted["jlc_gate_report_spec_sha256"] = {"here": sha(spec_here), "clone": sha(spec_there)}
    r = subprocess.run(make + ["--no-renders"], cwd=str(croot / "scripts"), env=env, text=True,
                       encoding="utf-8", errors="replace", capture_output=True)
    try:
        log = json.loads((croot / "kicad" / "work" / "make_all_log.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        log = {"failed_checks": None, "stopped_at": "make_all_log.json missing", "steps": []}
    out: dict = {"folder": croot.as_posix(), "command": "make_all.py --no-renders", "clone": adapted,
                 "build": {"exit": r.returncode, "minutes": round((time.time() - t0) / 60, 1),
                           "failed_checks": log["failed_checks"], "stopped_at": log.get("stopped_at"),
                           "steps": [(s["step"], s["exit"], s["seconds"]) for s in log["steps"]]},
                 "renaming_rule": f"every spelling of '{croot.as_posix()}' -> '{ROOT.as_posix()}', then of '{dst.as_posix()}' -> '{repo.as_posix()}'; sha256 of a differing "
                                  "file of the set -> this folder's sha256 of that file"}
    if r.returncode not in (0, 2):
        out.update(result="FAIL", why=f"the build in the second folder stopped (exit {r.returncode}, step {log.get('stopped_at')})",
                   stdout_tail=(r.stdout or "")[-800:])
        return out
    tokens = folder_tokens(dst, croot)
    raw = {}
    not_rebuilt, missing = [], []
    for rel in sorted(here["deterministic"]):
        p = croot / rel
        if not p.is_file():
            missing.append(rel)
        elif p.stat().st_mtime < t0:
            not_rebuilt.append(rel)
        else:
            raw[rel] = p.read_bytes()
    sha_map = [(sha(d).encode(), here["deterministic"][rel].encode()) for rel, d in raw.items()
               if sha(d) != here["deterministic"][rel]]

    def renamed(data: bytes) -> bytes:
        for a, b in tokens + sha_map:
            data = data.replace(a, b)
        return data
    same, after, differing = [], [], [m + " (missing in the second folder)" for m in missing]
    for rel, d in raw.items():
        want = here["deterministic"][rel]
        if sha(d) == want:
            same.append(rel)
        elif sha(renamed(d)) == want:
            after.append(rel)
        else:
            differing.append(rel)
    norm_same, norm_diff = [], []
    for rel, want in sorted(here["normalised"].items()):
        p = croot / rel
        if p.is_file() and p.stat().st_mtime >= t0 and not rel.endswith(".step"):
            (norm_same if sha(renamed(normalised_bytes(rel, p.read_bytes()))) == want["normalised"] else norm_diff).append(rel)
    must = [rel for rel in here["deterministic"] if rel in MUST_BE_RAW_IDENTICAL or rel.startswith(MUST_BE_RAW_PREFIX)
            or rel.endswith(MUST_BE_RAW_SUFFIX)]
    must_bad = sorted(rel for rel in must if rel not in same)
    gate_there = None
    try:
        gate_there = json.loads((croot / "out" / "manifest.json").read_text(encoding="utf-8"))["release_gate"]["reasons"]
    except (OSError, ValueError, KeyError):
        pass
    out.update(compared=len(raw) + len(missing), identical=len(same), identical_after_renaming=sorted(after),
               differing=sorted(differing), netlists_identical_after_date_and_renaming=norm_same,
               normalised_netlists_differing=norm_diff, not_rebuilt_in_the_second_folder=not_rebuilt,
               order_files_boards_schematic_checked=len(must), order_files_boards_schematic_not_raw_identical=must_bad,
               spec_sha256_in_the_clone=sha(raw["spec/carrier_spec.json"]) if "spec/carrier_spec.json" in raw else None,
               release_gate_reasons_in_the_clone=gate_there,
               sha256_second_folder={rel: sha(raw[rel]) for rel in MUST_BE_RAW_IDENTICAL if rel in raw},
               result="PASS" if not differing and not norm_diff and not must_bad and same else "FAIL")
    return out


CROSS_GROUP = ("second folder (boards / schematic / gerbers / BOM / CPL byte-identical; files that state their folder "
               "identical after renaming)")
#: regenerated by the docs step after the two builds (they print this report): may differ from build 2 on disk
DOCS_AFTER = ("README_ko.md", "out/ORDER_ko.md")


def judge_cross(res: dict) -> None:
    """Put the second-folder result into result_by_group / result (a FAIL there fails the whole check)."""
    x = res["cross_folder"]
    res["result_by_group"][CROSS_GROUP] = x["result"]
    res.pop("why_cross_folder", None)
    if x["result"] == "FAIL":
        res["result"] = "FAIL"
        res["why_cross_folder"] = ("files differing between this folder and the second folder: "
                                   + ", ".join(x.get("differing", []) + x.get("normalised_netlists_differing", [])
                                               + [f"{f} (not raw-identical)" for f in
                                                  x.get("order_files_boards_schematic_not_raw_identical", [])])
                                   if "differing" in x else x.get("why", ""))


def cross_only(xdir: Path | None) -> int:
    """``--cross-folder <dir> --cross-only``: keep the two in-place builds of the existing report and (re)run only the
    second-folder build. Refused unless the deterministic files on disk still equal build 2 of that report (the two
    documents regenerated after the builds excepted)."""
    if xdir is None:
        raise SystemExit("--cross-only needs --cross-folder <dir>")
    try:
        res = json.loads(REPORT.read_text(encoding="utf-8"))
        want = res["sha256_build2"]["deterministic"]
    except (OSError, ValueError, KeyError):
        raise SystemExit("--cross-only: no complete out/reports/determinism.json - run determinism.py without it first")
    now = snapshot()
    drift = sorted(k for k in set(want) | set(now["deterministic"])
                   if want.get(k) != now["deterministic"].get(k) and k not in DOCS_AFTER)
    if drift:
        raise SystemExit("--cross-only: files changed since build 2 of the report: " + ", ".join(drift))
    b = res["build1_vs_build2"]
    res["result"] = "PASS" if not b["deterministic"]["differing"] and not b["normalised"]["differing"] else "FAIL"
    res["cross_folder"] = cross_folder(xdir, now)
    res["cross_folder"]["mode"] = ("--cross-only at " + time.strftime("%Y-%m-%dT%H:%M:%S%z") + ": compared with the files on disk, "
                                   "which equal build 2 of this report except " + " / ".join(DOCS_AFTER)
                                   + " (regenerated after the builds; the clone makes them from the same report)")
    judge_cross(res)
    res["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    REPORT.write_text(json.dumps(res, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    post = subprocess.run([sys.executable, "-B", "-X", "utf8", str(HERE / "make_all.py"), "--only", "docs,manifest"],
                          cwd=str(HERE), env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"), text=True,
                          encoding="utf-8", errors="replace", capture_output=True)
    print(json.dumps({"result": res["result"], "result_by_group": res["result_by_group"],
                      "cross_folder": {k: v for k, v in res["cross_folder"].items()
                                       if k in ("result", "why", "compared", "identical", "identical_after_renaming", "differing",
                                                "normalised_netlists_differing", "order_files_boards_schematic_not_raw_identical",
                                                "clone", "release_gate_reasons_in_the_clone", "folder")},
                      "post_docs_manifest_exit": post.returncode, "report": REPORT.as_posix()}, indent=1))
    return 0 if res["result"] == "PASS" and post.returncode == 0 else 1


def main() -> int:
    extra = ["--no-renders"] if "--no-renders" in sys.argv else []
    xdir = None
    if "--cross-folder" in sys.argv:
        i = sys.argv.index("--cross-folder")
        if i + 1 >= len(sys.argv):
            raise SystemExit("--cross-folder needs a folder (empty, or an earlier clone of this tool)")
        xdir = Path(sys.argv[i + 1]).resolve()
        if ROOT in xdir.parents or xdir == ROOT or xdir in ROOT.parents:
            raise SystemExit("--cross-folder: the second folder must lie outside this board folder")
    if "--cross-only" in sys.argv:
        return cross_only(xdir)
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
        # relayout-fix step (review RR-2): the same inputs built once more in ANOTHER folder
        if xdir is not None:
            res["cross_folder"] = cross_folder(xdir, b2)
        else:
            res["cross_folder"] = {"result": "NOT_VERIFIED", "why": "not run (determinism.py --cross-folder <empty folder>)"}
        judge_cross(res)
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
                      "cross_folder": {k: v for k, v in res.get("cross_folder", {}).items()
                                       if k in ("result", "why", "compared", "identical", "identical_after_renaming",
                                                "differing", "normalised_netlists_differing",
                                                "order_files_boards_schematic_not_raw_identical", "folder")},
                      "post_docs_manifest_exit": post.returncode, "report": REPORT.as_posix()}, indent=1))
    return 0 if res["result"] == "PASS" and post.returncode == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
