"""Tool checks and JLC order files of the routed rev 3 board (run with the AI_EDA venv python).

Every correctness statement about the order files comes from here (or from the scripts it names):
  * kicad-cli sch erc (JSON, all severities)                 -> out/reports/erc.json
  * kicad-cli sch export netlist + scripted comparison       -> out/vtol_carrier.net, out/reports/netlist_vs_spec.json
  * kicad-cli pcb drc --schematic-parity --severity-all --refill-zones -> out/reports/drc.json
  * scripted board-vs-IR pad-net comparison                  -> out/reports/board_vs_ir.json
  * JLC BOM + CPL, filtered TOGETHER by the spec ``fit`` (fitsets.bom_groups; no --smd-only - since the plan
    amendments of 2026-11-12 no THT part has fit jlc, so both files are SMT only):
        out/jlcpcb_bom.csv / out/jlcpcb_cpl.csv                    default pair: U1..U4 NOT fitted by JLC
        out/jlcpcb_bom_with_mcu.csv / out/jlcpcb_cpl_with_mcu.csv  adds U1..U4 = C6510
    BOM header exactly 'Comment,Designator,Footprint,JLCPCB Part #（optional）' and CPL header
    'Designator,Mid X,Mid Y,Layer,Rotation' with 'mm' units, Layer 'T' and integer rotations as in JLC's sample files
    (read 2026-11-11, SCR/impl/export/jlc_sample_files.txt). Rotation and Mid X / Mid Y come only from
    out/cpl_rotation_table.csv (scripts/cpl_rotation.py, KiCad python) - a CPL row without a table row is an error.
  * gerbers + drill for the 4-layer board, two zips that differ ONLY in F.Paste:
        out/vtol_carrier_gerber.zip           no paste apertures on the U1..U4 pads (scripts/paste_variant.py)
        out/vtol_carrier_gerber_with_mcu.zip  paste on U1..U4 (order it together with the *_with_mcu pair)
    Neither set has paste on unpopulated footprints (spec fit 'none', e.g. Q3 / R47; rev 3 review finding J4).
    The creation-date fields KiCad writes into every gerber / drill / job file are set to RELEASE_STAMP so the same
    board gives byte-identical files (recorded in the manifest).
  * STEP (--subst-models --no-dnp), schematic PDF, KiCad full BOM (all parts incl. user-fitted, DNP column)
  * scripts/cad_envelope.py + scripts/board_metrics.py (pcbnew)   -> out/reports/board_metrics.json
  * JLC fee estimate (E) from the final BOM and today's gate records -> out/reports/jlc_fee_estimate.json + csv
  * off-board purchase list                                         -> out/off_board_purchase.csv
  * out/manifest.json: sha256 of every order file and report, the decisions, the release gate
  * release gate: "ORDERABLE (tool checks only; NOT_VERIFIED - never built)" only when ERC errors 0, DRC 0 / 0 / 0,
    board == IR, net clusters closed, no via on a pad, fit-set equality, the JLC gate of TODAY passed for this spec,
    every board-level rule report passes; otherwise "NOT ORDERABLE" with the reasons. Never 'partial'.
Nothing here edits the design (the paste variant is a temporary copy).

Run:  cd <board>/scripts && B:/Claude/.venv/Scripts/python.exe -X utf8 -B checks_exports.py [--files-only]
      --files-only: only BOM / CPL / gerbers (used by the determinism check); CARRIER_EXPORT_DIR redirects out/.
"""

from __future__ import annotations

import collections
import csv
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

from ai_eda.tools.kicad import sexpr
from ai_eda.tools.manufacturing.outputs import check_drill_files, check_gerber_set

import common
import fitsets
import rev3_checks

K = str(common.KICAD_CLI)
KPY = str(common.KICAD_PY)
SCH = common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_sch"
PCB = common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_pcb"
OUT = Path(os.environ["CARRIER_EXPORT_DIR"]) if os.environ.get("CARRIER_EXPORT_DIR") else common.OUT_DIR
REP = OUT / "reports"
HERE = common.ROOT / "scripts"
ROT_TABLE = common.OUT_DIR / "cpl_rotation_table.csv"
ROT_JSON = common.OUT_DIR / "reports" / "cpl_rotation.json"
GATE = common.OUT_DIR / "jlc_gate_report.json"
#: fixed value written into the date fields of the gerber / drill / job files (byte-identical outputs)
RELEASE_STAMP = "2026-11-11T00:00:00+09:00"
BOM_HEADER = ["Comment", "Designator", "Footprint", "JLCPCB Part #\uff08optional\uff09"]
CPL_HEADER = ["Designator", "Mid X", "Mid Y", "Layer", "Rotation"]
CU4 = "F.Cu,In1.Cu,In2.Cu,B.Cu"
FAB_LAYERS = CU4 + ",F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts"


def run(args: list[str], allow_fail: bool = False, cwd: Path | None = None) -> subprocess.CompletedProcess:
    r = subprocess.run(args, capture_output=True, text=True, encoding="utf-8", errors="replace", cwd=cwd,
                       env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
    if r.returncode != 0 and not allow_fail:
        raise RuntimeError(f"{' '.join(args)}\n{r.stdout}\n{r.stderr}")
    return r


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes((json.dumps(data, indent=1, ensure_ascii=False) + "\n").encode("utf-8"))


def write_csv(path: Path, header: list[str], rows: list[list], bom: bool = True) -> None:
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(header)
    w.writerows(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(buf.getvalue().encode("utf-8-sig" if bom else "utf-8"))


# ------------------------------------------------------------------------------------------------- tool checks
def erc() -> dict:
    path = REP / "erc.json"
    run([K, "sch", "erc", "--format", "json", "--severity-all", "--units", "mm", "-o", str(path), str(SCH)], allow_fail=True)
    d = json.loads(path.read_text(encoding="utf-8"))
    c = collections.Counter()
    for sh in d["sheets"]:
        for v in sh["violations"]:
            c[f"{v['severity']}:{v['type']}"] += 1
    return {"report": path.as_posix(), "kicad_version": d.get("kicad_version"), "sheets_checked": len(d["sheets"]),
            "counts": dict(c), "errors": sum(n for k, n in c.items() if k.startswith("error")),
            "warnings": sum(n for k, n in c.items() if k.startswith("warning"))}


def netlist_check() -> dict:
    net = OUT / f"{common.PROJECT_ID}.net"
    run([K, "sch", "export", "netlist", "--format", "kicadsexpr", "-o", str(net), str(SCH)])
    tree = sexpr.parse_file(net)
    got = {}
    for n in sexpr.find_all(sexpr.find(tree, "nets"), "net"):
        got[str(sexpr.get(n, "name"))] = sorted((str(sexpr.get(x, "ref")), str(sexpr.get(x, "pin"))) for x in sexpr.find_all(n, "node"))
    comps = len(sexpr.find_all(sexpr.find(tree, "components"), "comp"))
    ir = common.build_ir()
    want = {n.name: sorted((p.component_ref, p.pin_number) for p in n.pins) for n in ir.nets}
    named = {k: v for k, v in got.items() if not k.startswith("unconnected-")}
    mism = sorted(k for k in want if want[k] != got.get(k))
    extra = sorted(k for k in named if k not in want)
    res = {"tool": "kicad-cli sch export netlist + scripted comparison", "netlist": net.as_posix(),
           "netlist_sha256_without_date_line": rev3_checks.normalised_netlist(net), "components": comps,
           "spec_nets": len(want), "netlist_named_nets": len(named), "mismatched_nets": mism, "extra_nets": extra,
           "result": "PASS" if not mism and not extra and comps == len(ir.components) else "FAIL"}
    write_json(REP / "netlist_vs_spec.json", res)
    return res


def drc() -> dict:
    path = REP / "drc.json"
    run([K, "pcb", "drc", "--schematic-parity", "--severity-all", "--refill-zones", "--format", "json", "--units", "mm",
         "-o", str(path), str(PCB)], allow_fail=True)
    d = json.loads(path.read_text(encoding="utf-8"))
    for key in ("violations", "unconnected_items", "schematic_parity"):
        if key not in d:
            raise RuntimeError(f"DRC report has no {key!r} container")
    c = collections.Counter(f"{v['severity']}:{v['type']}" for v in d["violations"])
    return {"report": path.as_posix(), "kicad_version": d.get("kicad_version"), "violations": dict(c),
            "errors": sum(n for k, n in c.items() if k.startswith("error")),
            "warnings": sum(n for k, n in c.items() if k.startswith("warning")),
            "unconnected": len(d["unconnected_items"]), "parity_count": len(d["schematic_parity"])}


def board_vs_ir() -> dict:
    tree = sexpr.parse_file(PCB)
    ir = common.build_ir()
    want = {(p.component_ref, p.pin_number): n.name for n in ir.nets for p in n.pins}
    bad, seen = [], set()
    for fp in sexpr.find_all(tree, "footprint"):
        ref = next(str(pr[2]) for pr in sexpr.find_all(fp, "property") if str(pr[1]) == "Reference")
        for pad in sexpr.find_all(fp, "pad"):
            num = str(pad[1])
            if not num:
                continue
            net = sexpr.get(pad, "net")
            net = str(net) if net is not None else None
            w = want.get((ref, num))
            if w is not None:
                seen.add((ref, num))
                if net != w:
                    bad.append([ref, num, net, w])
            elif net is not None and not net.startswith("unconnected-("):
                bad.append([ref, num, net, None])
    res = {"tool": "scripted: routed .kicad_pcb pad nets vs IR nets", "ir_pins": len(want), "pins_found_on_board": len(seen),
           "mismatches": bad, "result": "PASS" if not bad and len(seen) == len(want) else "FAIL"}
    write_json(REP / "board_vs_ir.json", res)
    return res


def board_copper_layers() -> int:
    tree = sexpr.parse_file(PCB)
    names = [str(item[1]) for item in sexpr.find(tree, "layers")[1:] if len(item) > 1 and str(item[1]).endswith(".Cu")]
    if len(names) != 4 or common.LAYERS != 4:
        raise RuntimeError(f"board has copper layers {names}, rev 3 needs 4 (F.Cu / In1 GND / In2 +5V / B.Cu)")
    return len(names)


# ----------------------------------------------------------------------------------------------- BOM and CPL
def rotation_table() -> dict[str, dict]:
    spec_sha, board_sha = common.sha256_file(common.SPEC_PATH), common.sha256_file(PCB)
    info = json.loads(ROT_JSON.read_text(encoding="utf-8"))
    if info["spec_sha256"] != spec_sha or info["board"]["sha256"] != board_sha:
        raise RuntimeError("out/cpl_rotation_table.csv was made from another spec / board: run "
                           "KiCad python scripts/cpl_rotation.py first")
    with ROT_TABLE.open(encoding="utf-8") as f:
        return {r["Designator"]: r for r in csv.DictReader(f)}


def bom_cpl() -> dict:
    spec = common.load_spec()
    table = rotation_table()
    res = {"rotation_table": ROT_TABLE.as_posix(), "pairs": {}}
    for with_mcu, suffix in ((False, ""), (True, "_with_mcu")):
        lines = fitsets.bom_groups(spec["parts"], with_mcu)
        codes = collections.Counter(g["jlc_code"] for g in lines)
        dup = [c for c, n in codes.items() if n > 1]
        bom_rows = [[g["comment"], ",".join(g["designators"]), g["footprint_name"], g["jlc_code"]] for g in lines]
        cpl_rows, missing = [], []
        for ref in sorted((r for g in lines for r in g["designators"]), key=fitsets.natural):
            t = table.get(ref)
            if t is None:
                missing.append(ref)
                continue
            rot = float(t["CPL rotation"])
            if abs(rot - round(rot)) > 1e-9:
                raise RuntimeError(f"{ref}: CPL rotation {rot} is not an integer")
            cpl_rows.append([ref, f"{float(t['CPL Mid X (mm)']):.4f}mm", f"{float(t['CPL Mid Y (mm)']):.4f}mm", "T",
                             str(int(round(rot)) % 360)])
        bom_path, cpl_path = OUT / f"jlcpcb_bom{suffix}.csv", OUT / f"jlcpcb_cpl{suffix}.csv"
        write_csv(bom_path, BOM_HEADER, bom_rows)
        write_csv(cpl_path, CPL_HEADER, cpl_rows)
        fit = rev3_checks.check_fit_files(spec, [r for g in lines for r in g["designators"]],
                                          [r[0] for r in cpl_rows], with_mcu)
        nv = sorted((r for r in fit_refs(lines) if table[r]["status"] != "EE_FIT"
                     and table[r]["kind"] != "non-polar 2-pad"), key=fitsets.natural)
        res["pairs"]["with_mcu" if with_mcu else "main"] = {
            "bom": bom_path.as_posix(), "cpl": cpl_path.as_posix(), "bom_lines": len(lines),
            "bom_lines_smt": sum(1 for g in lines if g["assembly"] == "SMT"),
            "bom_lines_tht": sum(1 for g in lines if g["assembly"] == "THT"),
            "parts": sum(g["qty"] for g in lines), "cpl_rows": len(cpl_rows), "duplicate_codes": dup,
            "cpl_without_rotation_row": missing, "fit_set_check": fit,
            "orientation_critical_rotation_not_verified": nv}
    return res


def fit_refs(lines: list[dict]) -> list[str]:
    return [r for g in lines for r in g["designators"]]


# ---------------------------------------------------------------------------------------------------- gerbers
def normalise_dates(folder: Path) -> int:
    """Replace the export time KiCad writes into gerber / drill / job files by RELEASE_STAMP (comment and
    attribute lines only; the image data is untouched). Returns the number of lines changed."""
    # kicad-cli writes CRLF on Windows: every pattern stops before an optional \r (line ends are kept as written)
    pats = [(re.compile(r"^%TF\.CreationDate,[^*\r\n]*\*%(?=\r?$)", re.M), f"%TF.CreationDate,{RELEASE_STAMP}*%"),
            (re.compile(r"^(G04 Created by KiCad \(PCBNEW [^)]*\)) date [^*\r\n]*\*(?=\r?$)", re.M),
             rf"\1 date {RELEASE_STAMP[:10]} {RELEASE_STAMP[11:19]}*"),
            (re.compile(r"^(; DRILL file [^\r\n]*?) date [^\s]+(?=\r?$)", re.M), rf"\1 date {RELEASE_STAMP[:19]}"),
            (re.compile(r"^; #@! TF\.CreationDate,[^\s]+(?=\r?$)", re.M), f"; #@! TF.CreationDate,{RELEASE_STAMP}"),
            (re.compile(r'"CreationDate":\s*"[^"]*"'), f'"CreationDate": "{RELEASE_STAMP}"')]
    n = 0
    for p in sorted(folder.iterdir()):
        if not p.is_file() or p.suffix.lower() == ".pdf":
            continue
        text = p.read_bytes().decode("utf-8")
        new = text
        for rx, rep in pats:
            new, k = rx.subn(rep, new)
            n += k
        if new != text:
            p.write_bytes(new.encode("utf-8"))
    return n


def export_gerbers(board: Path, folder: Path, layers: str) -> None:
    folder.mkdir(parents=True, exist_ok=True)
    run([K, "pcb", "export", "gerbers", "--layers", layers, "--check-zones", "--subtract-soldermask",
         "-o", str(folder) + "/", str(board)])


def zip_folder(folder: Path, zpath: Path) -> list[str]:
    files = sorted(p for p in folder.iterdir() if p.is_file())
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for p in files:
            info = zipfile.ZipInfo(p.name, date_time=(2026, 11, 11, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            z.writestr(info, p.read_bytes())
    return [p.name for p in files]


def flashes(path: Path) -> int:
    return len(re.findall(r"D03\*", path.read_text(encoding="utf-8")))


def fab_outputs() -> dict:
    n_cu = board_copper_layers()
    work = Path(tempfile.mkdtemp(prefix="vtol_fab_"))
    try:
        # every layer except F.Paste: the design board as it is
        g_base = work / "gerber_base"
        export_gerbers(PCB, g_base, FAB_LAYERS)
        run([K, "pcb", "export", "drill", "--format", "excellon", "--excellon-units", "mm", "-o", str(g_base) + "/", str(PCB)])
        paste = f"{common.PROJECT_ID}-F_Paste.gtp"
        # F.Paste from the paste variants (scripts/paste_variant.py): no paste on any footprint JLC does not fit in
        # that order - default set: U1..U4, the user-soldered ESP-12F U7 (2026-11-12) and the unpopulated footprints
        # (fit none, e.g. Q3 / R47); with_mcu set: U7 and the unpopulated ones. The user-soldered THT connectors
        # (2026-11-12) have no paste pads at all
        variants, g_vars = {}, {}
        for name, flag in (("main", []), ("with_mcu", ["--with-mcu"])):
            var_dir = work / f"variant_{name}"
            r = run([KPY, "-B", str(HERE / "paste_variant.py"), *flag, str(var_dir / PCB.name)])
            variants[name] = json.loads(r.stdout.strip().splitlines()[-1])
            g_vars[name] = work / f"gerber_variant_{name}"
            export_gerbers(var_dir / PCB.name, g_vars[name], FAB_LAYERS)
        g_def, g_mcu = work / "gerber", work / "gerber_with_mcu"
        for name, dest_dir in (("main", g_def), ("with_mcu", g_mcu)):
            shutil.copytree(g_base, dest_dir)
            shutil.copyfile(g_vars[name] / paste, dest_dir / paste)
        changed = (normalise_dates(g_mcu) + normalise_dates(g_def) + normalise_dates(g_base)
                   + sum(normalise_dates(g) for g in g_vars.values()))
        # every layer of a variant except F.Paste must equal the design board's (proves the copy changed nothing else)
        other_diff = sorted({p.name for g in g_vars.values() for p in g.iterdir() if p.name != paste and p.suffix != ".gbrjob"
                             and p.read_bytes() != (g_base / p.name).read_bytes()})
        variant = variants
        out = {}
        for name, folder, zname in (("main", g_def, f"{common.PROJECT_ID}_gerber.zip"),
                                    ("with_mcu", g_mcu, f"{common.PROJECT_ID}_gerber_with_mcu.zip")):
            dest = OUT / ("gerber" if name == "main" else "gerber_with_mcu")
            if dest.exists():
                shutil.rmtree(dest)
            shutil.copytree(folder, dest)
            files = zip_folder(dest, OUT / zname)
            gerbers = [p for p in dest.iterdir() if p.suffix.lower() not in (".drl", ".gbrjob")]
            drills = [p for p in dest.iterdir() if p.suffix.lower() == ".drl"]
            gc = check_gerber_set(gerbers + [p for p in dest.iterdir() if p.suffix == ".gbrjob"], layer_count=n_cu,
                                  zone_layers=["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"])
            dc = check_drill_files(drills)
            out[name] = {"zip": (OUT / zname).as_posix(), "zip_sha256": common.sha256_file(OUT / zname),
                         "files": files, "gerber_check": {"status": str(gc.status), "message": gc.message},
                         "drill_check": {"status": str(dc.status), "message": dc.message},
                         "f_paste_flashes": flashes(dest / paste)}
        res = {"copper_layers": n_cu, "layers": FAB_LAYERS, "sets": out, "paste_variant": variant,
               "date_fields_normalised": {"lines": changed, "value": RELEASE_STAMP},
               "variant_layers_other_than_f_paste_differing": other_diff,
               "paste_decision": "default zip: no paste apertures on U1..U4 (the user hand-solders the MCUs on clean "
                                 "pads); both zips: no paste on the user-soldered ESP-12F U7 (user decision "
                                 "2026-11-12) and on unpopulated footprints (fit none); "
                                 "with_mcu zip: paste on U1..U4, upload it ONLY together with the *_with_mcu "
                                 "BOM / CPL. Whether JLC would itself leave unplaced parts without paste is "
                                 "NOT_VERIFIED, so the two zips are kept.",
               "paste_flash_difference": out["with_mcu"]["f_paste_flashes"] - out["main"]["f_paste_flashes"]}
    finally:
        shutil.rmtree(work, ignore_errors=True)
    write_json(REP / "gerber_check.json", res)
    return res


# ------------------------------------------------------------------------------------------- other exports
def step_and_docs() -> dict:
    out = {}
    step = OUT / f"{common.PROJECT_ID}.step"
    r = run([K, "pcb", "export", "step", "--subst-models", "--no-dnp", "--force", "-o", str(step), str(PCB)], allow_fail=True)
    out["step"] = {"path": step.as_posix(), "exit": r.returncode, "exists": step.exists(),
                   "missing_models": sorted(set(re.findall(r"Could not add 3D model for (\w+)", r.stdout + r.stderr)))}
    sdir = OUT / "schematic"
    if sdir.exists():
        shutil.rmtree(sdir)
    sdir.mkdir()
    run([K, "sch", "export", "pdf", "-o", str(sdir / f"{common.PROJECT_ID}_schematic.pdf"), str(SCH)])
    out["schematic_pdf"] = (sdir / f"{common.PROJECT_ID}_schematic.pdf").as_posix()
    full = OUT / f"{common.PROJECT_ID}_bom.csv"
    run([K, "sch", "export", "bom", "--fields", "Reference,Value,Footprint,${QUANTITY},${DNP},Description",
         "--labels", "Refs,Value,Footprint,Qty,DNP,Description", "--group-by", "Value,Footprint,${DNP}",
         "--ref-range-delimiter", "", "-o", str(full), str(SCH)])
    out["kicad_full_bom"] = full.as_posix()
    pos = REP / "kicad_pos_all.csv"
    run([K, "pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both", "-o", str(pos), str(PCB)])
    out["kicad_pos_all"] = pos.as_posix()
    out["pos_vs_rotation_table"] = pos_vs_table(pos)
    return out


def pos_vs_table(pos: Path) -> dict:
    """kicad-cli's own position file against the rotation table (KiCad origin / rotation / side of every JLC part)."""
    table = rotation_table()
    bad, seen = [], 0
    with pos.open(encoding="utf-8") as f:
        for r in csv.DictReader(f):
            t = table.get(r["Ref"])
            if t is None:
                continue
            seen += 1
            if (abs(float(r["PosX"]) - float(t["KiCad origin X (mm)"])) > 1e-3
                    or abs(float(r["PosY"]) - float(t["KiCad origin Y (mm)"])) > 1e-3
                    or (float(r["Rot"]) - float(t["KiCad rotation"])) % 360 > 1e-3 or r["Side"] != "top"):
                bad.append([r["Ref"], r["PosX"], r["PosY"], r["Rot"], r["Side"]])
    return {"rows_compared": seen, "table_rows": len(table), "differences": bad,
            "result": "PASS" if not bad and seen == len(table) else "FAIL"}


def board_metrics() -> dict:
    run([sys.executable, "-X", "utf8", "-B", str(HERE / "cad_envelope.py")], cwd=HERE)
    run([KPY, "-B", str(HERE / "board_metrics.py")], cwd=HERE)
    d = json.loads((common.OUT_DIR / "reports" / "board_metrics.json").read_text(encoding="utf-8"))
    return {"report": (common.OUT_DIR / "reports" / "board_metrics.json").as_posix(),
            "cluster_paths_all_le_3mm": d["cluster_paths_all_le_3mm"],
            "cluster_paths_max_mm": max((x["copper_path_mm"] or 99) for x in d["cluster_paths"]),
            "net_clusters_ok": d["net_clusters"]["ok"], "nets_split": sorted(d["net_clusters"]["nets_split"]),
            "vias_touching_pads": d["vias_touching_pads"]["count"],
            "silk_over_bodies": len(d["silk_over_bodies"]["over_bodies"]), "hole_to_hole": d["hole_to_hole"],
            "esp12f_ok": d["esp12f"]["ok"], "min_track_width_mm": d["min_track_width_mm"],
            "plug_fit_ok": all(x["ok"] for x in d["plug_fit"]),
            "plug_fit_failed": [x["ref"] for x in d["plug_fit"] if not x["ok"]],
            "power_narrow_outside_pad_entry": d["power_narrow_outside_pad_entry"]}


# ----------------------------------------------------------------------------------------------- fee estimate
FEES = {  # F scout/jlc_rules.md:136-151 (JLC price page, last updated Oct 22, 2026)
    "Economic": {"setup": 8.18, "stencil": 1.53, "feeder_per_type": 3.07, "feeder_types": "Extended (not Preferred) SMT"},
    "Standard": {"setup": 25.56, "stencil": 8.21, "feeder_per_type": 1.53, "feeder_types": "every SMT type"},
}
THT_ORDER, THT_JOINT, SMT_JOINT, CPP = 3.58, 0.0164, 0.0016, 0.45


def tht_fee(tht_lines: int, tht_joints: int, boards: int) -> float:
    """Hand-soldering fee of an order. The per-order part THT_ORDER is charged only when the BOM has a through-hole
    line (esp-user-fix step, finding V2: with 0 THT lines the table still added 3.58 USD; the order page showed no
    hand-soldering line for the no-THT quotes, M spec/jlc_order_page_20261112.json)."""
    return round(THT_ORDER + THT_JOINT * tht_joints * boards, 2) if tht_lines else 0.0


STD_ONLY ={"handling_fee_min": 14.93, "two_fixtures_1_to_29_pcs": 16.42, "packing_E": 0.5}


def price_at(prices: list[dict], qty: int) -> float:
    rows = sorted(prices, key=lambda p: p["startNumber"])
    best = rows[0]["productPrice"]
    for p in rows:
        if qty >= p["startNumber"]:
            best = p["productPrice"]
    return float(best)


def fee_estimate() -> dict:
    spec = common.load_spec()
    gate = json.loads(GATE.read_text(encoding="utf-8"))
    rot = json.loads(ROT_JSON.read_text(encoding="utf-8"))["rows"]
    rec = {}
    for x in gate["lines"] + gate["with_mcu_extra_lines"]:
        rec[x["jlc_code"]] = json.loads(Path(x["evidence_file"]).read_text(encoding="utf-8"))["response"]["data"]
    out, rows = {}, []
    for with_mcu in (False, True):
        lines = fitsets.bom_groups(spec["parts"], with_mcu)
        smt = [g for g in lines if g["assembly"] == "SMT"]
        tht = [g for g in lines if g["assembly"] == "THT"]
        smt_joints = sum(len(rot[r]["kicad_pads_board"]) for g in smt for r in g["designators"])
        tht_joints = sum(len(rot[r]["kicad_pads_board"]) for g in tht for r in g["designators"])
        ext = [g for g in smt if g["library_type"] == "Extended"]
        for n in (2, 5):
            comp_simple = comp_attr = 0.0
            for g in lines:
                d = rec[g["jlc_code"]]
                comp_simple += float(d["prices"][0]["productPrice"]) * g["qty"] * n
                buy = max(g["qty"] * n + int(d.get("lossNumber") or 0), int(d.get("leastNumber") or 0))
                comp_attr += price_at(d["prices"], buy) * buy
            for kind, f in FEES.items():
                types = len(ext) if kind == "Economic" else len(smt)
                row = {"pair": "with_mcu" if with_mcu else "main", "order": kind, "boards": n,
                       "setup": f["setup"], "stencil": f["stencil"], "feeder_types": types,
                       "feeder_loading": round(f["feeder_per_type"] * types, 2),
                       "smt_joints_per_board": smt_joints, "smt_joints": round(SMT_JOINT * smt_joints * n, 2),
                       "tht_joints_per_board": tht_joints, "tht_hand_soldering": tht_fee(len(tht), tht_joints, n),
                       "confirm_parts_placement": CPP, "components_tier1": round(comp_simple, 2),
                       "components_with_loss_and_minimum": round(comp_attr, 2)}
                row["sum_E"] = round(row["setup"] + row["stencil"] + row["feeder_loading"] + row["smt_joints"]
                                     + row["tht_hand_soldering"] + CPP + row["components_with_loss_and_minimum"], 2)
                if kind == "Standard":
                    row["standard_only_fees_applicability_NOT_VERIFIED"] = round(sum(STD_ONLY.values()), 2)
                    row["sum_E_with_standard_only_fees"] = round(row["sum_E"] + row["standard_only_fees_applicability_NOT_VERIFIED"], 2)
                rows.append(row)
    want = [("main", "Standard", 2), ("main", "Standard", 5), ("main", "Economic", 2)]
    out = {"label": "E (estimate). Fee table F scout/jlc_rules.md:136-151 (JLC price page read 2026-11-10); component "
                    "prices = today's gate records (out/jlc_gate_report.json); 'components_with_loss_and_minimum' "
                    "buys max(qty x boards + lossNumber, leastNumber) at the matching price tier (meaning of the two "
                    "fields A). NOT included: PCB fabrication, surface-finish surcharge, baking, the 0.50 USD "
                    "packaging line and the nitrogen-reflow line the order page showed, shipping, tax. The quote on "
                    "JLC's site is the only real price; the Economic and Standard quotes read on 2026-11-12 are in "
                    "spec/jlc_order_page_20261112.json (M) - the order guide shows those and uses this table only as a "
                    "cross-check.",
           "requested": [r for r in rows if (r["pair"], r["order"], r["boards"]) in want], "all": rows,
           "standard_only_fees": STD_ONLY}
    write_json(REP / "jlc_fee_estimate.json", out)
    keys = list(rows[0].keys()) + ["standard_only_fees_applicability_NOT_VERIFIED", "sum_E_with_standard_only_fees"]
    keys = list(dict.fromkeys(keys))
    write_csv(OUT / "jlc_fee_estimate.csv", keys, [[r.get(k, "") for k in keys] for r in rows])
    return {"report": (REP / "jlc_fee_estimate.json").as_posix(),
            "requested": {f"{r['order']} {r['boards']}": r.get("sum_E_with_standard_only_fees", r["sum_E"])
                          for r in out["requested"]}}


# ------------------------------------------------------------------------------------------ off-board purchase
def off_board_purchase() -> dict:
    """Everything the user buys elsewhere (REV3_PLAN sections 8 / 9; spec user_parts; plan amendments of 2026-11-12:
    the ESP-12F and the 12 through-hole connectors). The rows come from scripts/purchase_list.py; stock and price
    only from the LCSC snapshot file named there, otherwise NOT_VERIFIED (devicemart was not fetched)."""
    import purchase_list

    spec = common.load_spec()
    plan = json.loads((common.ROOT / "spec" / "rev3_plan.json").read_text(encoding="utf-8"))
    fit = json.loads((common.OUT_DIR / "fit_checks.json").read_text(encoding="utf-8"))
    rows = purchase_list.build(spec, plan, fit)
    path = OUT / "off_board_purchase.csv"
    write_csv(path, purchase_list.CSV_HEADER, purchase_list.csv_rows(rows))
    return {"file": path.as_posix(), "rows": len(rows),
            "rows_per_group": {g: sum(1 for r in rows if r["group"] == g) for g, _en, _ko in purchase_list.GROUPS},
            "former_jlc_bom_lines_now_bought_by_the_user": [r["lcsc"] for r in rows if r["group"] == "a"],
            "lcsc_snapshot": "spec/lcsc_snapshot_20261112.json" if purchase_list.LCSC.is_file() else None,
            "group_a_cost_for_2_boards_usd_E": purchase_list.group_a_cost(rows),
            "note": "JLC does not ship loose parts with a PCBA order; the LCSC codes are separate LCSC orders. "
                    "Servos, ESC and the 6S pack are vehicle parts, not listed here."}


# ---------------------------------------------------------------------------------------------- release gate
def gate_status() -> dict:
    if not GATE.is_file():
        return {"status": "NOT_VERIFIED", "why": "out/jlc_gate_report.json missing - run scripts/jlc_gate.py"}
    g = json.loads(GATE.read_text(encoding="utf-8"))
    same_spec = g.get("spec_sha256") == common.sha256_file(common.SPEC_PATH)
    return {"status": g["status"] if same_spec else "STALE (other spec)", "with_mcu_status": g.get("with_mcu_status"),
            "started_at": g.get("started_at"), "mode": g.get("mode"), "lines_pass": g.get("main_pair_lines_pass"),
            "lines": g.get("main_pair_lines"), "thin_lines": g.get("thin_lines"), "failures": g.get("failures")}


def main() -> int:
    files_only = "--files-only" in sys.argv
    REP.mkdir(parents=True, exist_ok=True)
    if files_only:
        summary = {"bom_cpl": bom_cpl(), "fab": fab_outputs()}
        print(json.dumps({"pairs": {k: v["bom_lines"] for k, v in summary["bom_cpl"]["pairs"].items()},
                          "zips": {k: v["zip_sha256"] for k, v in summary["fab"]["sets"].items()}}, indent=1))
        return 0
    summary = {
        "erc": erc(),
        "netlist_vs_spec": netlist_check(),
        "drc": drc(),
        "board_vs_ir": board_vs_ir(),
        "bom_cpl": bom_cpl(),
        "fab": fab_outputs(),
        "step_docs": step_and_docs(),
        "board_metrics": board_metrics(),
        "fee_estimate": fee_estimate(),
        "off_board_purchase": off_board_purchase(),
        "jlc_gate": gate_status(),
    }
    rr = json.loads((REP / "routing_rules.json").read_text(encoding="utf-8"))
    fc = json.loads((common.OUT_DIR / "fit_checks.json").read_text(encoding="utf-8"))
    summary["routing_rules"] = {"report": (REP / "routing_rules.json").as_posix(), "result": rr["result"],
                                "results": rr["results"], "board_sha_matches": rr["board"]["sha256"] == common.sha256_file(PCB)}
    summary["fit_checks"] = {"report": (common.OUT_DIR / "fit_checks.json").as_posix(), "errors": fc["errors"],
                             "board_sha_matches": fc["board_sha256"] == common.sha256_file(PCB),
                             "tht_over_rule": fc["tht_leads"]["over_rule_nominal"],
                             "reflow_not_verified": fc["reflow_rating"]["not_verified"],
                             "reflow_at_limit": fc["reflow_rating"]["exceeds_or_at_limit"]}
    reasons = []
    s = summary
    if s["erc"]["errors"]:
        reasons.append(f"ERC errors {s['erc']['errors']}")
    if s["drc"]["errors"] or s["drc"]["unconnected"] or s["drc"]["parity_count"]:
        reasons.append(f"DRC errors {s['drc']['errors']} / unconnected {s['drc']['unconnected']} / parity {s['drc']['parity_count']}")
    for key in ("netlist_vs_spec", "board_vs_ir"):
        if s[key]["result"] != "PASS":
            reasons.append(f"{key} {s[key]['result']}")
    for name, p in s["bom_cpl"]["pairs"].items():
        if p["fit_set_check"]["result"] != "PASS" or p["duplicate_codes"] or p["cpl_without_rotation_row"]:
            reasons.append(f"{name} pair: fit-set check {p['fit_set_check']['result']}, duplicate codes {p['duplicate_codes']}")
    for name, st in s["fab"]["sets"].items():
        if st["gerber_check"]["status"] not in ("ValidationStatus.PASS", "PASS") or \
                st["drill_check"]["status"] not in ("ValidationStatus.PASS", "PASS"):
            reasons.append(f"{name} gerber / drill check {st['gerber_check']['status']} / {st['drill_check']['status']}")
    if s["fab"]["variant_layers_other_than_f_paste_differing"] or s["fab"]["paste_flash_difference"] != 256:
        reasons.append("paste variant changed more than the U1..U4 paste apertures")
    if s["step_docs"]["pos_vs_rotation_table"]["result"] != "PASS":
        reasons.append("kicad-cli position file differs from the CPL rotation table")
    bm = s["board_metrics"]
    if not bm["net_clusters_ok"]:
        reasons.append(f"nets split into copper islands: {bm['nets_split']}")
    if bm["vias_touching_pads"]:
        reasons.append(f"vias touching pads {bm['vias_touching_pads']}")
    if s["jlc_gate"]["status"] != "PASS":
        reasons.append(f"JLC source gate {s['jlc_gate']['status']}")
    if s["routing_rules"]["result"] != "PASS" or not s["routing_rules"]["board_sha_matches"]:
        bad = [k for k, v in s["routing_rules"]["results"].items() if v != "PASS"]
        reasons.append(f"routing_rules {s['routing_rules']['result']} ({', '.join(bad)})"
                       + ("" if s["routing_rules"]["board_sha_matches"] else " - made on another board"))
    if s["fit_checks"]["errors"] or not s["fit_checks"]["board_sha_matches"]:
        reasons.append(f"fit_checks errors {s['fit_checks']['errors']}")
    s["release_gate"] = {"status": "ORDERABLE (tool checks only; NOT_VERIFIED - never built)" if not reasons
                         else "NOT ORDERABLE", "reasons": reasons,
                         "user_gates_not_covered": [
                             "JLC upload preview of the current files: every BOM line matched (SMT only since "
                             "2026-11-12; no THT line, no hand soldering by JLC)",
                             "JLC 'Confirm Parts Placement': rotation of the parts listed as NOT_VERIFIED in "
                             "out/cpl_rotation_table.csv and the polarity of C45 / C47 / C52",
                             "JLC DFM result (edge distances; Economic PCBA: where JLC puts its tooling holes, "
                             "or its request for edge rails)",
                             "jlc_gate.py again on the order day, directly before paying"]}
    files = {}
    # relayout-fix step (review RR-8): files that a LATER step rewrites would be listed here with the hash of the
    # previous run (M: reports/determinism.json). They are hashed by make_manifest.py instead
    # (build.written_after_checks_exports = make_manifest.LATE), after they were written.
    # REV3_REPORT_ko.md is the deliver step's hand-written report (not generated): hashed there as well.
    written_later = ("reports/determinism.json", "reports/rev3_export_files.json", "reports/rev3_export_determinism.json",
                     "ORDER_ko.md", "REV3_REPORT_ko.md", "따로_살_부품_ko.md")
    for p in sorted(OUT.rglob("*")):
        rel = p.relative_to(OUT).as_posix()
        if p.is_file() and p.name != "manifest.json" and not rel.startswith(("render/", "presentation/")) \
                and rel not in written_later:
            files[rel] = common.sha256_file(p)
    for p in [SCH, PCB, common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_pro", common.KICAD_DIR / "placement.json",
              common.SPEC_PATH]:
        files["../" + p.relative_to(common.ROOT).as_posix()] = common.sha256_file(p)
    s["order_files"] = {
        "upload_default": ["out/vtol_carrier_gerber.zip", "out/jlcpcb_bom.csv", "out/jlcpcb_cpl.csv"],
        "upload_with_mcu": ["out/vtol_carrier_gerber_with_mcu.zip", "out/jlcpcb_bom_with_mcu.csv",
                            "out/jlcpcb_cpl_with_mcu.csv"],
        "never_mix": "a zip and a BOM / CPL pair of different sets",
        "format_reference": "JLC sample files Sample-BOM_JLCSMT.xlsx (Last-Modified 14 Jul 2021) and "
                            "Sample-CPL_JLCSMT.xlsx (19 Sep 2019), read 2026-11-11 (SCR/impl/export/jlc_sample_files.txt); "
                            "CSV with UTF-8 BOM (Excel). Acceptance of the exact header strings / encoding is "
                            "NOT_VERIFIED until JLC's upload preview shows every line matched.",
        "excluded_from_hashes": "out/render/, out/presentation/ (render step); " + ", ".join("out/" + r for r in written_later)
                                + " (written by later steps: their hashes are under build.written_after_checks_exports)"}
    s["sha256"] = files
    write_json(OUT / "manifest.json", s)
    brief = {k: s[k] for k in ("erc", "netlist_vs_spec", "drc", "jlc_gate", "release_gate")}
    brief["bom_cpl"] = {k: {kk: v[kk] for kk in ("bom_lines", "bom_lines_smt", "bom_lines_tht", "parts", "cpl_rows",
                                                  "orientation_critical_rotation_not_verified")}
                        | {"fit": v["fit_set_check"]["result"]} for k, v in s["bom_cpl"]["pairs"].items()}
    brief["fab"] = {k: {"zip_sha256": v["zip_sha256"], "gerber": v["gerber_check"]["status"],
                        "drill": v["drill_check"]["status"], "f_paste_flashes": v["f_paste_flashes"]}
                    for k, v in s["fab"]["sets"].items()}
    brief["step"] = s["step_docs"]["step"]
    brief["pos_vs_table"] = s["step_docs"]["pos_vs_rotation_table"]["result"]
    brief["board_metrics"] = {k: bm[k] for k in ("net_clusters_ok", "vias_touching_pads", "esp12f_ok", "plug_fit_ok",
                                                  "cluster_paths_all_le_3mm", "cluster_paths_max_mm")}
    brief["fees"] = s["fee_estimate"]["requested"]
    print(json.dumps(brief, indent=1, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
