"""Tool checks and fabrication outputs for the routed board (run with the AI_EDA venv python).

Every correctness statement in README_ko.md comes from here:
  * kicad-cli sch erc (JSON, all severities)            -> out/reports/erc.json
  * kicad-cli sch export netlist + scripted comparison  -> out/vtol_carrier.net, out/reports/netlist_vs_spec.json
  * kicad-cli pcb drc --schematic-parity --severity-all --refill-zones (JSON) -> out/reports/drc.json
  * scripted board-vs-IR pad-net comparison             -> out/reports/board_vs_ir.json
  * gerbers + drill (+ zip), engine format check        -> out/gerber/, out/vtol_carrier_gerber.zip, out/reports/gerber_check.json
  * BOM CSV (kicad-cli), CPL CSV (kicad-cli pcb export pos --smd-only: the THT connectors are hand-soldered),
    STEP, 3D renders, schematic PDF / SVG
  * scripts/cad_envelope.py + scripts/board_metrics.py (pcbnew): cluster copper paths, crystal nets, GND stitching,
    power / servo widths, hole-to-hole, silkscreen, mated-plug fit vs the CAD -> out/reports/board_metrics.json
  * out/manifest.json: sha256 of every output + the summary counts
  * release gate (rev 2c, rev-2b review PF-B1): "ORDERABLE" only with ERC errors 0, DRC errors 0, unconnected 0, parity
    0, every net one copper cluster, no via touching a pad; otherwise "NOT ORDERABLE" with the reasons (never 'partial')
  * JLC CPL rotations = KiCad rotation + the per-footprint offset of spec/jlc_cpl_rotations_db.csv (community table,
    fetched read-only 2026-11-05 from JLC_ROT_URL; A - JLC's placement preview must still be checked; review PF-M5)
Nothing here edits the design.
"""

from __future__ import annotations

import collections
import json
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

from ai_eda.tools.kicad import sexpr
from ai_eda.tools.manufacturing.outputs import check_drill_files, check_gerber_set

import common

K = str(common.KICAD_CLI)
SCH = common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_sch"
PCB = common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_pcb"
OUT = common.OUT_DIR
REP = OUT / "reports"


def run(args: list[str], allow_fail: bool = False) -> subprocess.CompletedProcess:
    r = subprocess.run(args, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0 and not allow_fail:
        raise RuntimeError(f"{' '.join(args)}\n{r.stdout}\n{r.stderr}")
    return r


def erc() -> dict:
    path = REP / "erc.json"
    run([K, "sch", "erc", "--format", "json", "--severity-all", "--units", "mm", "-o", str(path), str(SCH)], allow_fail=True)
    d = json.loads(path.read_text(encoding="utf-8"))
    c = collections.Counter()
    for sh in d["sheets"]:
        for v in sh["violations"]:
            c[f"{v['severity']}:{v['type']}"] += 1
    return {"report": str(path), "kicad_version": d.get("kicad_version"), "sheets_checked": [s["path"] for s in d["sheets"]],
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
    nc_spec = sum(1 for c in ir.components for p in c.pins if p.electrical_type == "no_connect")
    res = {"tool": "kicad-cli sch export netlist + scripted comparison", "components": comps, "spec_nets": len(want),
           "netlist_named_nets": len(named), "mismatched_nets": mism, "extra_nets": extra,
           "single_pin_unconnected_nets": len(got) - len(named), "spec_no_connect_pins": nc_spec,
           "result": "PASS" if not mism and not extra and comps == len(ir.components) else "FAIL"}
    (REP / "netlist_vs_spec.json").write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8", newline="\n")
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
    p = collections.Counter(f"{v['severity']}:{v['type']}" for v in d["schematic_parity"])
    unc = collections.Counter()
    for v in d["unconnected_items"]:
        for it in v["items"][:1]:
            unc[it.get("description", "")[:60]] += 0
    nets = collections.Counter()
    for v in d["unconnected_items"]:
        desc = " ".join(i.get("description", "") for i in v["items"])
        import re

        m = re.search(r"\[([^\]]+)\]", desc)
        nets[m.group(1) if m else "?"] += 1
    return {"report": str(path), "kicad_version": d.get("kicad_version"), "violations": dict(c),
            "errors": sum(n for k, n in c.items() if k.startswith("error")),
            "warnings": sum(n for k, n in c.items() if k.startswith("warning")),
            "unconnected": len(d["unconnected_items"]), "unconnected_by_net": dict(nets.most_common()),
            "schematic_parity": dict(p), "parity_count": len(d["schematic_parity"])}


def board_vs_ir() -> dict:
    tree = sexpr.parse_file(PCB)
    ir = common.build_ir()
    want = {}
    for n in ir.nets:
        for p in n.pins:
            want[(p.component_ref, p.pin_number)] = n.name
    bad = []
    seen: set = set()  # (ref, pad number): J7 has two MP pads and the SOT-223 two pad-2 copies
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
    (REP / "board_vs_ir.json").write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8", newline="\n")
    return res


def board_copper_layers() -> int:
    """Copper layer count read from the routed board file; must equal common.LAYERS (the delivered choice)."""
    tree = sexpr.parse_file(PCB)
    names = [str(item[1]) for item in sexpr.find(tree, "layers")[1:] if len(item) > 1 and str(item[1]).endswith(".Cu")]
    if len(names) != common.LAYERS:
        raise RuntimeError(f"board has copper layers {names}, common.LAYERS = {common.LAYERS}")
    return len(names)


def fab_outputs() -> dict:
    g = OUT / "gerber"
    if g.exists():
        shutil.rmtree(g)
    g.mkdir(parents=True)
    n_cu = board_copper_layers()
    cu = "F.Cu,B.Cu" if n_cu == 2 else "F.Cu,In1.Cu,In2.Cu,B.Cu"
    run([K, "pcb", "export", "gerbers", "--layers", cu + ",F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts",
         "--check-zones", "--subtract-soldermask", "-o", str(g) + "/", str(PCB)])
    run([K, "pcb", "export", "drill", "--format", "excellon", "--excellon-units", "mm", "--generate-map", "--map-format", "pdf",
         "-o", str(g) + "/", str(PCB)])
    files = sorted(p for p in g.iterdir() if p.is_file())
    zpath = OUT / f"{common.PROJECT_ID}_gerber.zip"
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for p in files:
            info = zipfile.ZipInfo(p.name, date_time=(2026, 11, 4, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, p.read_bytes())
    gerbers = [p for p in files if p.suffix.lower() not in (".drl", ".pdf", ".gbrjob")]
    drills = [p for p in files if p.suffix.lower() == ".drl"]
    zl = ["F.Cu", "B.Cu"] if n_cu == 2 else ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
    gc = check_gerber_set(gerbers + [p for p in files if p.suffix == ".gbrjob"], layer_count=n_cu, zone_layers=zl)
    dc = check_drill_files(drills)
    res = {"copper_layers": n_cu, "gerber_check": {"status": str(gc.status), "message": gc.message},
           "drill_check": {"status": str(dc.status), "message": dc.message},
           "files": [p.name for p in files], "zip": str(zpath)}
    (REP / "gerber_check.json").write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8", newline="\n")
    return res


def bom_cpl() -> dict:
    bom = OUT / f"{common.PROJECT_ID}_bom.csv"
    run([K, "sch", "export", "bom", "--fields", "Reference,Value,Footprint,${QUANTITY},${DNP},Description",
         "--labels", "Refs,Value,Footprint,Qty,DNP,Description", "--group-by", "Value,Footprint,${DNP}",
         "--ref-range-delimiter", "", "-o", str(bom), str(SCH)])
    cpl = OUT / f"{common.PROJECT_ID}_cpl.csv"
    run([K, "pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "front", "--exclude-dnp", "--smd-only",
         "-o", str(cpl), str(PCB)])
    rows = cpl.read_text(encoding="utf-8").strip().splitlines()
    res = {"bom": str(bom), "cpl": str(cpl), "cpl_rows": len(rows) - 1}
    res.update(jlc_and_purchase_lists(cpl))
    return res


def _csv(path: Path, header: list[str], rows: list[list]) -> None:
    import csv
    import io

    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(header)
    w.writerows(rows)
    path.write_bytes(buf.getvalue().encode("utf-8-sig"))  # BOM so Excel / the JLC uploader read the Korean names


def jlc_and_purchase_lists(cpl_kicad: Path) -> dict:
    """JLCPCB BOM + CPL in the columns JLC expects (spec bom_sources.jlc_bom_columns: Comment, Designator, Footprint,
    LCSC Part #) and the devicemart / LCSC purchase lists, all from spec/carrier_spec.json bom_sources (verified online
    by scripts/fetch_parts_sources.py, user rule 2026-11-05) joined with what is actually fitted on this board
    (common.populated: the programming-only footprints in the tie band / under the OpenLog are DNP here).
    Cross-checks: every fitted part has exactly one source line; every CPL row has an LCSC number."""
    import csv

    spec = common.load_spec()
    parts = {p["refdes"]: p for p in spec["parts"]}
    bs = spec["bom_sources"]
    detail = bs.get("sources_detail", {})
    line_of = {}
    for ln in bs["board_lines"]:
        for r in ln["refs"] + ln["dnp_refs"]:
            if r in line_of:
                raise RuntimeError(f"{r} in two BOM lines")
            line_of[r] = ln
    fitted = sorted((r for r in parts if common.populated(parts[r])), key=_natkey)
    no_line = [r for r in fitted if r not in line_of and r not in ("NT1", "JP1")]  # copper features, no part
    jlc_rows, tht_rows, dm_rows, all_rows = [], [], [], []
    groups: dict[str, list[str]] = {}
    for r in fitted:
        if r in line_of:
            groups.setdefault(line_of[r]["line"], []).append(r)
    for ln in bs["board_lines"]:
        refs = groups.get(ln["line"], [])
        dnp = sorted((r for r in ln["refs"] + ln["dnp_refs"] if r not in refs), key=_natkey)
        d = detail.get(ln["line"], {})
        all_rows.append([ln["line"], ln["value"], " ".join(refs), len(refs), " ".join(dnp), ln["footprint"], ln["source"],
                         ln["part_number"], ln["mpn"], ln.get("jlc_type") or "", ln["assembly"],
                         " ".join(d.get("urls", [])), d.get("fetched", bs.get("fetched", "")), d.get("stock", "")])
        if not refs:
            continue
        if ln["source"] == "JLCPCB/LCSC":
            fp = ln["footprint"].split(":")[1]
            row = [ln["value"], ",".join(refs), fp, ln["part_number"]]
            (jlc_rows if ln["assembly"].startswith("JLC SMT") else tht_rows).append(row)
        else:
            dm_rows.append([ln["part_number"], ln["mpn"], len(refs), "on board: " + " ".join(refs), ln["value"]])
    for it in bs["devicemart_purchase_list"]:
        if it["line"] in groups:
            continue  # already listed from the board lines
        dm_rows.append([it["part_number"], it["mpn"], it["qty"], it["for"], ""])
    loose = [[it["part_number"], it["mpn"], it["qty"], it["for"], it.get("jlc_type") or ""] for it in bs["lcsc_loose_items"]]
    OUT.mkdir(exist_ok=True)
    _csv(OUT / "jlcpcb_bom.csv", ["Comment", "Designator", "Footprint", "LCSC Part #"], jlc_rows)
    _csv(OUT / "jlcpcb_bom_tht_hand_solder.csv", ["Comment", "Designator", "Footprint", "LCSC Part #"], tht_rows)
    _csv(OUT / "devicemart_purchase_list.csv", ["devicemart no=", "product name", "qty", "for", "board value"], dm_rows)
    _csv(OUT / "lcsc_loose_items.csv", ["LCSC Part #", "MPN", "qty", "for", "JLC type"], loose)
    _csv(OUT / "bom_sources_all_lines.csv",
         ["line", "value", "fitted refs", "qty fitted", "DNP refs", "footprint", "source", "part number", "MPN / name",
          "JLC type", "assembly", "URL fetched", "fetch date", "stock at fetch"], all_rows)
    # CPL in JLC columns from the kicad-cli position file (SMD, top, DNP excluded)
    jlc_refs = {r for row in jlc_rows for r in row[1].split(",")}
    cpl_rows, cpl_missing, rot_rows = [], [], []
    table = jlc_rotation_table()
    with cpl_kicad.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            ref = row["Ref"]
            if ref not in jlc_refs:
                cpl_missing.append(ref)
            jr, pat = jlc_rotation(row["Package"], float(row["Rot"]), table)
            cpl_rows.append([ref, f"{float(row['PosX']):.4f}mm", f"{float(row['PosY']):.4f}mm", "Top", f"{jr:g}"])
            rot_rows.append([ref, row["Package"], f"{float(row['Rot']):g}", pat or "", f"{jr:g}"])
    _csv(OUT / "jlcpcb_cpl.csv", ["Designator", "Mid X", "Mid Y", "Layer", "Rotation"], cpl_rows)
    _csv(REP / "jlcpcb_cpl_rotation_offsets.csv", ["Designator", "Footprint", "KiCad rotation", "offset pattern (A)",
                                                   "JLC rotation"], rot_rows)
    smd_not_in_cpl = sorted(jlc_refs - {r[0] for r in cpl_rows}, key=_natkey)
    return {"jlcpcb_bom_lines": len(jlc_rows), "jlcpcb_parts": len(jlc_refs), "tht_hand_solder_lines": len(tht_rows),
            "devicemart_lines": len(dm_rows), "fitted_parts_without_source": no_line,
            "cpl_rows_without_lcsc": cpl_missing, "jlc_parts_not_in_cpl": smd_not_in_cpl,
            "result": "PASS" if not no_line and not cpl_missing else "FAIL",
            "cpl_rotation_offsets_applied": sorted({r[3] for r in rot_rows if r[3]}),
            "cpl_rows_rotated": sum(1 for r in rot_rows if r[3]),
            "note": f"JLC CPL rotation = KiCad rotation + offset from {JLC_ROT_URL} (community table, fetched 2026-11-05, "
                    "A); polarized parts (C45, C47, C52, D1-D4, D6, Q1-Q11, U1-U4, U6, U7, Y1-Y4, J7) must still be "
                    "checked in JLC's placement preview against the F.Fab pin-1 / polarity marks"}


JLC_ROT_URL = "https://raw.githubusercontent.com/matthewlai/JLCKicadTools/master/jlc_kicad_tools/cpl_rotations_db.csv"
JLC_ROT_DB = common.ROOT / "spec" / "jlc_cpl_rotations_db.csv"


def jlc_rotation_table() -> list[tuple]:
    import csv
    import re

    rows = []
    with JLC_ROT_DB.open(encoding="utf-8") as f:
        rd = csv.reader(f, skipinitialspace=True)
        next(rd)
        for r in rd:
            if r and r[0]:
                rows.append((r[0], re.compile(r[0]), float(r[1])))
    return rows


def jlc_rotation(package: str, rot: float, table: list[tuple]) -> tuple[float, str | None]:
    for pat, rx, off in table:
        if rx.search(package):
            return (rot + off) % 360, pat
    return rot % 360, None


def _natkey(r: str) -> tuple:
    import re

    m = re.match(r"([A-Z]+)(\d+)", r)
    return (m.group(1), int(m.group(2))) if m else (r, 0)


def step_and_renders() -> dict:
    out = {}
    step = OUT / f"{common.PROJECT_ID}.step"
    r = run([K, "pcb", "export", "step", "--subst-models", "--no-dnp", "--force", "-o", str(step), str(PCB)], allow_fail=True)  # --no-dnp: review MF2-6
    out["step"] = {"path": str(step), "exit": r.returncode, "exists": step.exists(), "log_tail": (r.stdout + r.stderr)[-600:]}
    renders = {
        "render_top.png": ["--side", "top"],
        "render_bottom.png": ["--side", "bottom"],
        "render_iso.png": ["--side", "top", "--rotate", "-45,0,30", "--perspective", "--zoom", "0.9"],
    }
    for name, extra in renders.items():
        p = OUT / name
        r = run([K, "pcb", "render", "-w", "1600", "-h", "1200", "--quality", "high", "--background", "opaque", *extra,
                 "-o", str(p), str(PCB)], allow_fail=True)
        out[name] = {"exit": r.returncode, "exists": p.exists()}
    sdir = OUT / "schematic"
    if sdir.exists():
        shutil.rmtree(sdir)
    sdir.mkdir()
    run([K, "sch", "export", "pdf", "-o", str(sdir / f"{common.PROJECT_ID}_schematic.pdf"), str(SCH)])
    run([K, "sch", "export", "svg", "-o", str(sdir) + "/", str(SCH)])
    out["schematic_exports"] = sorted(p.name for p in sdir.iterdir())
    return out


def board_metrics() -> dict:
    here = common.ROOT / "scripts"
    run([sys.executable, "-X", "utf8", str(here / "cad_envelope.py")])
    r = run([str(common.KICAD_PY), str(here / "board_metrics.py")])
    d = json.loads((REP / "board_metrics.json").read_text(encoding="utf-8"))
    return {"report": str(REP / "board_metrics.json"), "cluster_paths_all_le_3mm": d["cluster_paths_all_le_3mm"],
            "net_clusters_ok": d["net_clusters"]["ok"], "nets_split": sorted(d["net_clusters"]["nets_split"]),
            "vias_touching_pads": d["vias_touching_pads"]["count"],
            "silk_over_bodies": len(d["silk_over_bodies"]["over_bodies"]),
            "supply_pads": d["supply_pads"], "power_narrow_outside_pad_entry": d["power_narrow_outside_pad_entry"],
            "cluster_paths_max_mm": max((x["copper_path_mm"] or 99) for x in d["cluster_paths"]),
            "gnd": d["gnd"], "hole_to_hole": d["hole_to_hole"], "silk": d["silk"], "esp12e": d["esp12e"],
            "min_track_width_mm": d["min_track_width_mm"],
            "plug_fit_ok": all(x["ok"] for x in d["plug_fit"]), "plug_fit_failed": [x["ref"] for x in d["plug_fit"] if not x["ok"]],
            "stdout_tail": r.stdout[-300:]}


def main() -> int:
    REP.mkdir(parents=True, exist_ok=True)
    summary = {
        "erc": erc(),
        "netlist_vs_spec": netlist_check(),
        "drc": drc(),
        "board_vs_ir": board_vs_ir(),
        "fab": fab_outputs(),
        "bom_cpl": bom_cpl(),
        "step_renders": step_and_renders(),
        "board_metrics": board_metrics(),
    }
    files = {}
    for p in sorted(OUT.rglob("*")):
        if p.is_file() and p.name != "manifest.json":
            files[str(p.relative_to(OUT)).replace("\\", "/")] = common.sha256_file(p)
    for p in [SCH, PCB, common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_pro", common.KICAD_DIR / "placement.json"]:
        files["../kicad/" + p.name] = common.sha256_file(p)
    reasons = []
    if summary["erc"]["errors"]:
        reasons.append(f"ERC errors {summary['erc']['errors']}")
    if summary["drc"]["errors"]:
        reasons.append(f"DRC errors {summary['drc']['errors']}")
    if summary["drc"]["unconnected"]:
        reasons.append(f"DRC unconnected {summary['drc']['unconnected']}")
    if summary["drc"]["parity_count"]:
        reasons.append(f"schematic parity {summary['drc']['parity_count']}")
    if not summary["board_metrics"]["net_clusters_ok"]:
        reasons.append(f"nets split into copper islands: {summary['board_metrics']['nets_split']}")
    if summary["board_metrics"]["vias_touching_pads"]:
        reasons.append(f"vias touching pads {summary['board_metrics']['vias_touching_pads']}")
    summary["release_gate"] = {"status": "ORDERABLE (tool checks only; NOT_VERIFIED - never built)" if not reasons
                               else "NOT ORDERABLE", "reasons": reasons}
    summary["sha256"] = files
    (OUT / "manifest.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    brief = {k: summary[k] for k in ("erc", "netlist_vs_spec", "board_vs_ir")}
    brief["drc"] = {k: v for k, v in summary["drc"].items() if k != "unconnected_by_net"}
    brief["fab"] = summary["fab"]["gerber_check"], summary["fab"]["drill_check"]
    brief["bom_cpl"] = summary["bom_cpl"]
    brief["step_renders"] = {k: v for k, v in summary["step_renders"].items() if k != "step"}
    brief["step"] = {k: v for k, v in summary["step_renders"]["step"].items() if k != "log_tail"}
    brief["board_metrics"] = {k: v for k, v in summary["board_metrics"].items() if k != "stdout_tail"}
    brief["release_gate"] = summary["release_gate"]
    print(json.dumps(brief, indent=1, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
