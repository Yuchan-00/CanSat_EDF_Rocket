"""Paste-layer variants of the routed board for the two gerber sets (KiCad python: pcbnew).

Invariant: a copy differs from kicad/vtol_carrier.kicad_pcb ONLY in the F.Paste layer of the pads of parts that JLC
does not fit in that order:
  default set (no switch):   parts whose spec fit is ``jlc_optional`` (U1..U4, fitted by the user) or ``none``
                             (unpopulated footprints such as Q3 / R47) - the stencil prints no paste on them
  --with-mcu:                parts whose spec fit is ``none`` only (U1..U4 are fitted by JLC in that order)
The design board is never written; the copy goes to the path given on the command line (checks_exports.py uses a
temporary folder and exports ONLY F.Paste from it; every other layer comes from the design board).

Run:  C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/python.exe -B paste_variant.py [--with-mcu] <out.kicad_pcb>
Prints a JSON line {mode, refs, pads_changed, pads_with_paste_before, pads_with_paste_after}.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fitsets  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
BOARD = ROOT / "kicad" / "vtol_carrier.kicad_pcb"
SPEC = ROOT / "spec" / "carrier_spec.json"


def main() -> int:
    args = sys.argv[1:]
    with_mcu = "--with-mcu" in args
    args = [a for a in args if a != "--with-mcu"]
    out = Path(args[0]).resolve()
    if out == BOARD.resolve():
        raise SystemExit("refusing to overwrite the design board")
    spec = json.loads(SPEC.read_text(encoding="utf-8"))
    fits = ("none",) if with_mcu else ("jlc_optional", "none")
    refs = sorted((p["refdes"] for p in spec["parts"] if fitsets.fit_of(p) in fits), key=fitsets.natural)
    board = pcbnew.LoadBoard(str(BOARD))

    def paste_pads() -> int:
        return sum(1 for fp in board.GetFootprints() for pd in fp.Pads() if pd.IsOnLayer(pcbnew.F_Paste))

    before = paste_pads()
    changed = 0
    per_ref: dict[str, int] = {}
    for fp in board.GetFootprints():
        if fp.GetReference() not in refs:
            continue
        for pd in fp.Pads():
            if pd.IsOnLayer(pcbnew.F_Paste):
                ls = pd.GetLayerSet()
                ls.RemoveLayer(pcbnew.F_Paste)
                pd.SetLayerSet(ls)
                changed += 1
                per_ref[fp.GetReference()] = per_ref.get(fp.GetReference(), 0) + 1
    after = paste_pads()
    out.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(out), board)
    print(json.dumps({"mode": "with_mcu" if with_mcu else "main", "fits_without_paste": list(fits), "refs": refs,
                      "pads_changed": changed, "pads_changed_per_ref": dict(sorted(per_ref.items())),
                      "pads_with_paste_before": before, "pads_with_paste_after": after}))
    return 0 if changed and before - after == changed else 1


if __name__ == "__main__":
    sys.exit(main())
