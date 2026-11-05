"""Determinism check: run the deterministic steps twice and compare the sha256 of what they write.

Steps: build_schematic.py, place.py, build_pcb.py (unrouted board). Freerouting (route.py) is not part of
this check: its result is kept as the artifact and its hash is recorded in kicad/work/route_log.json.
Writes out/reports/determinism.json. Run with the AI_EDA venv python.
"""

from __future__ import annotations

import json
import subprocess
import sys

import common

PY = sys.executable
STEPS = ["build_schematic.py", "place.py", "build_pcb.py"]


def outputs() -> dict[str, str]:
    files = sorted(common.KICAD_DIR.glob("*.kicad_sch")) + [
        common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_pro",
        common.KICAD_DIR / "placement.json",
        common.KICAD_DIR / "ir" / f"{common.PROJECT_ID}.ir.json",
        common.KICAD_DIR / "ir" / f"{common.PROJECT_ID}.ir_with_pcb.json",
        common.WORK_DIR / f"{common.PROJECT_ID}_unrouted.kicad_pcb",
    ]
    return {p.name: common.sha256_file(p) for p in files}


def run_all() -> dict[str, str]:
    here = common.ROOT / "scripts"
    for s in STEPS:
        r = subprocess.run([PY, "-X", "utf8", str(here / s)], capture_output=True, text=True, encoding="utf-8",
                           errors="replace", cwd=str(here))
        if r.returncode != 0:
            raise SystemExit(f"{s} failed:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
    return outputs()


def routed_geometry() -> tuple[str, str]:
    """Re-import the recorded Freerouting session (route.py --reuse-ses); return (file sha256, sha256 without uuids)."""
    import re

    r = subprocess.run([str(common.KICAD_PY), str(common.ROOT / "scripts" / "route.py"), "--reuse-ses"], capture_output=True,
                       text=True, encoding="utf-8", errors="replace", cwd=str(common.ROOT / "scripts"))
    if r.returncode != 0:
        raise SystemExit(f"route.py --reuse-ses failed: {r.stdout[-2000:]} {r.stderr[-2000:]}")
    board = common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_pcb"
    import hashlib

    from ai_eda.tools.kicad import sexpr

    tree = sexpr.parse_file(board)
    items = []
    for node in tree:
        h = sexpr.head(node)
        if h in ("segment", "via", "arc", "footprint", "zone", "gr_line", "gr_arc", "gr_rect", "gr_text"):
            text = sexpr.dumps(node)
            text = re.sub(r'\(uuid "[0-9a-f-]+"\)', "(uuid)", text)
            if h == "zone":  # filled polygons: order of islands is not stable, compare as a sorted set
                polys = sorted(sexpr.dumps(fp) for fp in sexpr.find_all(node, "filled_polygon"))
                text = re.sub(r"\(filled_polygon[\s\S]*", "", text) + "".join(polys)
            items.append(text)
    canon = "\n".join(sorted(items))
    return common.sha256_file(board), hashlib.sha256(canon.encode("utf-8")).hexdigest()


def main() -> int:
    a = run_all()
    ra = routed_geometry()
    b = run_all()
    rb = routed_geometry()
    diff = sorted(k for k in a if a[k] != b.get(k))
    res = {"tool": "scripts/determinism.py (two full runs, sha256 compared)", "files": a, "differences": diff,
           "routed_board_reimport": {"file_sha256": [ra[0], rb[0]], "sha256_without_uuids": [ra[1], rb[1]],
                                     "bytes_identical": ra[0] == rb[0], "geometry_identical": ra[1] == rb[1],
                                     "note": "pcbnew.ImportSpecctraSES gives imported tracks / vias random KIIDs and KiCad saves items in KIID order; geometry is compared as a sorted set of items without uuids"},
           "result": "PASS" if not diff and ra[1] == rb[1] else "FAIL"}
    (common.OUT_DIR / "reports").mkdir(parents=True, exist_ok=True)
    (common.OUT_DIR / "reports" / "determinism.json").write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(res, indent=1))
    return 0 if res["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
