"""Top-view picture of the D6 search -> out/reports/programming_access.png.

    python scripts/prog_access_plot.py      (any python with matplotlib; reads only out/reports/programming_access.json)

Invariant: the picture draws what scripts/prog_access.py measured; it decides nothing. Fixed size, fixed dpi, Agg
backend, no date in the file.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

ROOT = Path(__file__).resolve().parent.parent
FIXED = ("anchor", "cluster", "esp", "power")


def box(ax, r, **kw):
    ax.add_patch(Rectangle((r[0], r[1]), r[2] - r[0], r[3] - r[1], **kw))


def main() -> int:
    rep = json.loads((ROOT / "out" / "reports" / "programming_access.json").read_text(encoding="utf-8"))
    pic = rep["picture_inputs"]
    o = pic["outline_kicad"]
    fig, ax = plt.subplots(figsize=(11.0, 10.4), dpi=150)
    if pic.get("outline_polygon_kicad"):  # redesign2: the real outline (main rectangle + tab)
        ax.add_patch(Polygon(pic["outline_polygon_kicad"], closed=True, fill=False, ec="black", lw=1.6, zorder=5))
        o = [o[0], o[1], o[2], max(p[1] for p in pic["outline_polygon_kicad"])]
    else:
        box(ax, o, fill=False, ec="black", lw=1.6, zorder=5)
    box(ax, rep["tie_band_kicad"], fc="#ffe08a", ec="#c79a00", lw=0.8, alpha=0.55, zorder=1)
    box(ax, rep["antenna_rule_area_kicad"], fc="none", ec="#7a3fb0", lw=1.0, ls="--", zorder=4)
    for z in rep["forbidden_zones"]:
        module = z["why"].startswith("module body") or "PCB" in z["why"]
        box(ax, z["rect_kicad"], fc="#9ecbff" if module else "#ffb3b3", ec="#1f5fa8" if module else "#b03030", lw=0.5, alpha=0.28, zorder=2)
    band = set(rep["bare_holes_option_A"])
    for ref, c in pic["courtyards"].items():
        r = c["rect"]
        if ref in band:
            box(ax, r, fc="#2e9d4d", ec="#14602b", lw=0.9, alpha=0.75, zorder=6)
            ax.text((r[0] + r[2]) / 2, (r[1] + r[3]) / 2, ref, ha="center", va="center", fontsize=5.5, color="white", zorder=7)
        else:
            fixed = c["how"] in FIXED
            box(ax, r, fc="#8a8a8a" if fixed else "#d9d9d9", ec="#404040" if fixed else "#8c8c8c", lw=0.35, alpha=0.9, zorder=3)
            if (r[2] - r[0]) * (r[3] - r[1]) > 20.0:
                ax.text((r[0] + r[2]) / 2, (r[1] + r[3]) / 2, ref, ha="center", va="center", fontsize=5.0, color="black", zorder=4)
    for case, col in (("fixed_only", "#d62728"), ("fixed_without_power_block", "#ff7f0e")):
        for fam, f in rep["cases"][case]["families"].items():
            for h in f["best_greedy_packing"]["headers"]:
                box(ax, h["courtyard_kicad"], fc="none", ec=col, lw=1.1, ls=":" if fam == "smd" else "-", zorder=8)
    v = rep["verdict_per_family"]
    lines = [
        "Decision D6 - programming headers: option " + rep["option"] + " (bare holes in the tie band, as rev 2c)",
        "green = the eight band headers J3, J19..J22 (2x3), J24..J26 (1x2), not fitted; yellow = tie band x 97..103",
        "dark grey = fixed parts (anchors, MCU clusters, ESP, power block); light grey = other parts; blue = module bodies;",
        "red tint = module header strips / insertion paths / card overhang; violet dashes = ESP antenna rule area",
        "legal header spots on the final placement: SMD 2x3 %d, SMD 1x2 %d, THT 2x3 %d, THT 1x2 %d" % (
            v["smd"]["final_2x3_legal_origins"], v["smd"]["final_1x2_legal_origins"], v["tht"]["final_2x3_legal_origins"], v["tht"]["final_1x2_legal_origins"]),
        "every greedy passive removed: at most %d SMD 2x3 / %d THT 2x3 fit at once (5 needed); red outlines = what still fits" % (
            v["smd"]["fixed_only_2x3_upper_bound_courtyard_only"], v["tht"]["fixed_only_2x3_upper_bound_courtyard_only"]),
        "power block also removed (orange outlines, information only): at most %d SMD 2x3 / %d THT 2x3 with the plug body (A)" % (
            v["smd"]["without_power_block_2x3_upper_bound"], v["tht"]["without_power_block_2x3_upper_bound"]),
    ]
    ax.set_title("\n".join(lines), fontsize=7.5, loc="left")
    ax.set_xlim(o[0] - 2, o[2] + 2)
    ax.set_ylim(o[3] + 2, o[1] - 2)  # KiCad y down
    ax.set_aspect("equal")
    ax.set_xlabel("KiCad x [mm]", fontsize=7)
    ax.set_ylabel("KiCad y [mm]", fontsize=7)
    ax.tick_params(labelsize=6)
    fig.tight_layout()
    out = ROOT / "out" / "reports" / "programming_access.png"
    fig.savefig(out, dpi=150, metadata={"Software": None})
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
