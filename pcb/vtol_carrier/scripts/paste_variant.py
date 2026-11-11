"""Paste-layer variant of the routed board for the default gerber set (KiCad python: pcbnew).

Invariant: the copy differs from kicad/vtol_carrier.kicad_pcb ONLY in the F.Paste layer of the pads of parts whose
spec fit is ``jlc_optional`` (U1..U4, fitted by the user in the default order): JLC's stencil then prints no paste on
the four TQFP-64 land patterns, so the user solders the MCUs on clean pads. The design board is never written;
the copy goes to the path given on the command line (checks_exports.py uses a temporary folder and exports ONLY
F.Paste from it; every other layer comes from the design board).

Run:  C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/python.exe -B paste_variant.py <out.kicad_pcb>
Prints a JSON line {refs, pads_changed, pads_with_paste_before, pads_with_paste_after}.
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
    out = Path(sys.argv[1]).resolve()
    if out == BOARD.resolve():
        raise SystemExit("refusing to overwrite the design board")
    spec = json.loads(SPEC.read_text(encoding="utf-8"))
    refs = sorted((p["refdes"] for p in spec["parts"] if fitsets.fit_of(p) == "jlc_optional"), key=fitsets.natural)
    board = pcbnew.LoadBoard(str(BOARD))

    def paste_pads() -> int:
        return sum(1 for fp in board.GetFootprints() for pd in fp.Pads() if pd.IsOnLayer(pcbnew.F_Paste))

    before = paste_pads()
    changed = 0
    for fp in board.GetFootprints():
        if fp.GetReference() not in refs:
            continue
        for pd in fp.Pads():
            if pd.IsOnLayer(pcbnew.F_Paste):
                ls = pd.GetLayerSet()
                ls.RemoveLayer(pcbnew.F_Paste)
                pd.SetLayerSet(ls)
                changed += 1
    after = paste_pads()
    out.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(out), board)
    print(json.dumps({"refs": refs, "pads_changed": changed, "pads_with_paste_before": before,
                      "pads_with_paste_after": after}))
    return 0 if changed and before - after == changed else 1


if __name__ == "__main__":
    sys.exit(main())
