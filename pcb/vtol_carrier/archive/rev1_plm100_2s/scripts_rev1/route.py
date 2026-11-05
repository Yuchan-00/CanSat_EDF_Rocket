"""Route the board with Freerouting (run with KiCad's own python: it needs ``pcbnew``).

Stages (recorded in kicad/work/route_log.json):
  1. copy kicad/work/vtol_carrier_unrouted.kicad_pcb -> kicad/vtol_carrier.kicad_pcb (next to the .kicad_pro,
     so pcbnew loads the project's net classes / design rules)
  2. pcbnew.ExportSpecctraDSN; the GND pours are stripped from the DSN (see below)
  3. java -jar freerouting-2.5.0.jar -de <dsn> -do <ses> -mp <passes> -mt <threads> --gui.enabled=false (headless)
  4. pcbnew.ImportSpecctraSES; if connections are still open, export the routed board again and give
     Freerouting a short completion run on it (it keeps / reworks the existing wiring), import that session
  5. vias come back 0.1 mm larger than the board via sizes (the DSN via padstacks are inflated by VIA_INFLATE_MM so
     Freerouting keeps via drill edges >= 0.5 mm apart - the hole-to-hole rule it does not know, review F6); they are
     shrunk back to the class sizes
  6. neck-downs: Freerouting narrows a few segments at pads below the 0.127 mm board minimum; they are set to
     0.127 mm (DRC afterwards decides whether that is clean)
  7. fill all zones, then GND stitching vias wherever the filled GND copper of BOTH layers holds a whole via
     (2.5 mm grid, hole-to-hole >= 0.55 mm; review F4), fill again
  8. silkscreen: a function label next to every connector / jumper / module header and the reference designators
     wherever they fit without touching pads, other silk or the board edge (review F3); save kicad/vtol_carrier.kicad_pcb
  The locked pre-routed cluster copper (build_pcb.py, DSN "type fix") is kept by Freerouting and returned in the session.

Freerouting is not deterministic (multi-threaded); the final session kicad/work/vtol_carrier.ses is kept, and
``--reuse-ses`` re-imports it into a freshly generated unrouted board (stages 1, 4-import, 5, 6) - that path is
deterministic. Usage: python route.py [passes] [threads] | python route.py --reuse-ses
"""

from __future__ import annotations

import hashlib
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
KICAD = ROOT / "kicad"
WORK = KICAD / "work"
JAR = Path(r"C:/Users/yc012/AppData/Local/Temp/claude/B--Claude/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/tools_dl/freerouting/freerouting-2.5.0.jar")
BOARD = KICAD / "vtol_carrier.kicad_pcb"
MIN_W = pcbnew.FromMM(0.127)
VIA_INFLATE_MM = 0.1
STITCH_PITCH_MM = 2.5


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def export_dsn(board, dsn: Path) -> int:
    """DSN without the GND planes: with them Freerouting treats every GND pad as connected through the pour and
    never routes GND, which leaves pour islands after the fill; GND is routed as 0.2 mm tracks instead and the
    pours are filled on top afterwards (scratch experiments, M)."""
    if not pcbnew.ExportSpecctraDSN(board, str(dsn)):
        raise SystemExit("ExportSpecctraDSN failed")
    text = dsn.read_text(encoding="utf-8")
    stripped = re.sub(r"\n\s*\(plane GND \(polygon [FB]\.Cu[^)]*\)\)", "", text)

    def inflate(m: re.Match) -> str:  # via padstacks only: (padstack "Via[0-1]_600:300_um" (shape (circle F.Cu 600)) ...
        block = m.group(0)
        return re.sub(r"\(circle ([FB]\.Cu) (\d+)\)",
                      lambda c: f"(circle {c.group(1)} {int(c.group(2)) + int(round(VIA_INFLATE_MM * 1000))})", block)

    stripped = re.sub(r'\(padstack "Via\[0-1\]_\d+:\d+_um"(?:\s*\(shape \(circle [FB]\.Cu \d+\)\))+', inflate, stripped)
    dsn.write_text(stripped, encoding="utf-8", newline="\n")
    return text.count("(plane GND") - stripped.count("(plane GND")


def freerouting(dsn: Path, ses: Path, passes: int, threads: str, tag: str) -> dict:
    if ses.exists():
        ses.unlink()
    cmd = ["java", "-Dfile.encoding=UTF-8", "-jar", str(JAR), "-de", str(dsn), "-do", str(ses), "-mp", str(passes),
           "-mt", threads, "--gui.enabled=false"]
    t0 = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", cwd=str(WORK))
    (WORK / f"freerouting_{tag}_stdout.log").write_text(r.stdout + "\n--- stderr ---\n" + r.stderr, encoding="utf-8", newline="\n")
    if r.returncode != 0 or not ses.exists():
        print(r.stdout[-3000:], r.stderr[-3000:])
        raise SystemExit(f"freerouting {tag} failed (exit {r.returncode})")
    passes_log = re.findall(r"Auto-routing pass #(\d+) .*?\((\d+) unrouted", r.stdout)
    return {"command": " ".join(f'"{c}"' if " " in c else c for c in cmd), "seconds": round(time.time() - t0, 1),
            "exit_code": r.returncode, "dsn_sha256": sha(dsn), "ses_sha256": sha(ses),
            "passes_unrouted": [[int(a), int(b)] for a, b in passes_log], "stdout_tail": r.stdout[-2500:]}


def unconnected(board) -> int:
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.BuildConnectivity()
    try:
        return board.GetConnectivity().GetUnconnectedCount(False)
    except AttributeError:  # SWIG sometimes hands back an untyped shared_ptr; kicad-cli DRC counts it instead
        return None


def fix_neckdowns(board) -> list[dict]:
    fixed = []
    for t in board.GetTracks():
        if t.GetClass() == "PCB_TRACK" and t.GetWidth() < MIN_W:
            fixed.append({"net": t.GetNetname(), "layer": t.GetLayerName(), "was_mm": round(pcbnew.ToMM(t.GetWidth()), 4),
                          "x": round(pcbnew.ToMM(t.GetStart().x), 3), "y": round(pcbnew.ToMM(t.GetStart().y), 3)})
            t.SetWidth(MIN_W)
    return fixed


def remove_dangling_vias(board) -> list[dict]:
    """A via whose tracks all sit on one layer (and that is not on a pad) is a stub Freerouting left behind."""
    removed = []
    tracks = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
    zone_nets = {z.GetNetCode() for z in board.Zones()}
    for v in [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]:
        if v.GetNetCode() in zone_nets:
            continue  # a GND via may be fed by the pour on the other layer (stitch), not a stub
        p = v.GetPosition()
        layers = {t.GetLayer() for t in tracks if t.GetNetCode() == v.GetNetCode() and (t.GetStart() == p or t.GetEnd() == p)}
        on_pad = any(pad.HitTest(p) for fp in board.GetFootprints() for pad in fp.Pads() if pad.GetNetCode() == v.GetNetCode())
        if len(layers) <= 1 and not on_pad:
            removed.append({"net": v.GetNetname(), "x": round(pcbnew.ToMM(p.x), 3), "y": round(pcbnew.ToMM(p.y), 3)})
            board.Delete(v)
    return removed


def shrink_vias(board) -> int:
    """Undo the DSN via inflation: every via goes back to its class size (0.6 / 0.8 / 1.2 mm)."""
    n = 0
    d = pcbnew.FromMM(VIA_INFLATE_MM)
    for v in board.GetTracks():
        if v.GetClass() == "PCB_VIA":
            w = v.GetWidth(pcbnew.F_Cu)
            nominal = round(pcbnew.ToMM(w - d), 3)
            if nominal in (0.6, 0.8, 1.0, 1.2):
                v.SetWidth(pcbnew.F_Cu, w - d)
                v.SetWidth(pcbnew.B_Cu, w - d)
                n += 1
    return n


def holes(board) -> list[tuple[float, float, float]]:
    out = []
    for v in board.GetTracks():
        if v.GetClass() == "PCB_VIA":
            out.append((pcbnew.ToMM(v.GetPosition().x), pcbnew.ToMM(v.GetPosition().y), pcbnew.ToMM(v.GetDrillValue()) / 2))
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.HasHole():
                ds = pad.GetDrillSize()
                out.append((pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y), pcbnew.ToMM(max(ds.x, ds.y)) / 2))
    return out


def add_stitching_vias(board) -> list[list[float]]:
    """GND vias on a fixed grid wherever the filled GND copper of both layers holds the whole via (+0.05 mm) and
    the drill keeps >= 0.55 mm to every other hole; outside the PLM100 u.FL no-pour area. Deterministic order."""
    import math

    net = board.FindNet("GND")
    fills = []
    keepouts = []
    for z in board.Zones():
        if z.GetIsRuleArea():
            keepouts.append(z.Outline())
        elif z.GetNetname() == "GND":
            for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
                if z.IsOnLayer(lay):
                    fills.append(z.GetFilledPolysList(lay))
    hl = holes(board)
    added = []
    r_test = 0.3 + 0.05
    ring = [(0.0, 0.0)] + [(r_test * math.cos(k * math.pi / 4), r_test * math.sin(k * math.pi / 4)) for k in range(8)]
    y = 65.0 + 1.25
    while y < 135.0 - 1.0:
        x = 62.0 + 1.25
        while x < 138.0 - 1.0:
            pts = [pcbnew.VECTOR2I(pcbnew.FromMM(x + dx), pcbnew.FromMM(y + dy)) for dx, dy in ring]
            ok = all(f.Contains(p) for f in fills for p in pts) and len(fills) == 2
            if ok and any(k.Contains(pts[0]) for k in keepouts):
                ok = False
            if ok:
                for hx, hy, hr in hl:
                    if math.hypot(hx - x, hy - y) - hr - 0.15 < 0.55:
                        ok = False
                        break
            if ok:
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetWidth(pcbnew.F_Cu, pcbnew.FromMM(0.6))
                v.SetWidth(pcbnew.B_Cu, pcbnew.FromMM(0.6))
                v.SetDrill(pcbnew.FromMM(0.3))
                v.SetNet(net)
                board.Add(v)
                hl.append((x, y, 0.15))
                added.append([round(x, 3), round(y, 3)])
            x = round(x + STITCH_PITCH_MM, 4)
        y = round(y + STITCH_PITCH_MM, 4)
    return added


def finish(board) -> dict:
    shrunk = shrink_vias(board)
    fixed = fix_neckdowns(board)
    items = list(board.GetTracks())
    n_vias = sum(1 for t in items if t.GetClass() == "PCB_VIA")
    n_segs = len(items) - n_vias
    dangling = remove_dangling_vias(board)
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    stitch = add_stitching_vias(board)
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    import silk

    silk_log = silk.add_silk(board)
    board.Save(str(BOARD))
    # unconnected items are counted by kicad-cli DRC (checks_exports.py), not here
    return {"vias_shrunk_after_inflation": shrunk, "neckdowns_widened_to_0.127mm": fixed, "dangling_vias_removed": dangling,
            "segments": n_segs, "routed_vias": n_vias - len(dangling), "gnd_stitching_vias": len(stitch),
            "silk": silk_log, "routed_board_sha256": sha(BOARD)}


def main() -> int:
    reuse = "--reuse-ses" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    passes = int(args[0]) if len(args) > 0 else 40
    threads = args[1] if len(args) > 1 else "4"
    src = WORK / "vtol_carrier_unrouted.kicad_pcb"
    shutil.copyfile(src, BOARD)
    board = pcbnew.LoadBoard(str(BOARD))
    final_ses = WORK / "vtol_carrier.ses"
    if reuse:
        if not final_ses.exists():
            raise SystemExit("--reuse-ses: no kicad/work/vtol_carrier.ses")
        if not pcbnew.ImportSpecctraSES(board, str(final_ses)):
            raise SystemExit("ImportSpecctraSES failed")
        res = {"mode": "reuse recorded session", "ses_sha256": sha(final_ses), **finish(board)}
        (WORK / "route_reuse_log.json").write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8", newline="\n")
        print(json.dumps(res, indent=1))
        return 0
    log: dict = {"freerouting_jar_sha256": sha(JAR), "unrouted_board_sha256": sha(src)}
    dsn1, ses1 = WORK / "vtol_carrier.dsn", WORK / "vtol_carrier_stage1.ses"
    log["gnd_planes_removed_from_dsn"] = export_dsn(board, dsn1)
    if "--stage1-from-recorded" in sys.argv:
        # stage 1 already run (kicad/work/vtol_carrier_stage1.ses from an earlier full run on the same placement)
        log["stage1"] = {"recorded_session": ses1.name, "ses_sha256": sha(ses1),
                         "note": "Freerouting stage-1 session from the previous run (same placement), re-imported"}
    else:
        log["stage1"] = freerouting(dsn1, ses1, passes, threads, "stage1")
    if not pcbnew.ImportSpecctraSES(board, str(ses1)):
        raise SystemExit("ImportSpecctraSES failed")
    log["stage1"]["unconnected_after_import"] = unconnected(board)
    used = ses1
    if log["stage1"]["unconnected_after_import"] > 0:
        dsn2, ses2 = WORK / "vtol_carrier_stage2.dsn", WORK / "vtol_carrier_stage2.ses"
        export_dsn(board, dsn2)
        log["stage2"] = freerouting(dsn2, ses2, 20, threads, "stage2")
        if not pcbnew.ImportSpecctraSES(board, str(ses2)):
            raise SystemExit("ImportSpecctraSES (stage 2) failed")
        log["stage2"]["unconnected_after_import"] = unconnected(board)
        used = ses2
    shutil.copyfile(used, final_ses)
    log["final_session"] = used.name
    log.update(finish(board))
    (WORK / "route_log.json").write_text(json.dumps(log, indent=1) + "\n", encoding="utf-8", newline="\n")
    brief = {k: v for k, v in log.items() if k not in ("stage1", "stage2")}
    for k in ("stage1", "stage2"):
        if k in log:
            brief[k] = {kk: vv for kk, vv in log[k].items() if kk != "stdout_tail"}
    print(json.dumps(brief, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
