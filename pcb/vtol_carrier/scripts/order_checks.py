"""Order-related tables added to out/fit_checks.json by fit_checks.py (KiCad python: pcbnew).

 reflow_rating        REV3_PLAN section 14 (review OM-9): rated reflow peak of each JLC-fitted part from its document
                      against JLC's oven: Standard 240 +-5 C (worst 245 C), Economic 255 +-5 C (worst 260 C)
                      (F scout/jlc_rules.md:125). A part without a rating in a document read here is NOT_VERIFIED.
                      THT lines (assemblyMode manualWeld in today's gate record) are hand-soldered by JLC, not reflowed.
 connector_drawings   REV3_PLAN section 14 (review E-10): J14 / J15 / J17 / J7 / U7 - (1) the board footprint equals
                      the KiCad library footprint pad by pad (so the design-step drawing comparison of the library
                      footprint, SCR/design/conn_work/fp_vs_ds.json, holds for the board); (2) pitch and drill
                      re-measured on the board against the datasheet values of that comparison; (3) pad count, pitch
                      and drill of JLC's own EasyEDA footprint record of the same code. A datasheet mismatch is an
                      error; a JLC-library difference is reported (it is not a manufacturer drawing).

Invariant: the document values below are transcribed with file:line; nothing is measured on hardware.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import pcbnew

SCR = Path("C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e"
           "/scratchpad/pcb_rev3")
DS = SCR / "datasheets"
KICAD_FP = Path("C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/share/kicad/footprints")
OVEN = {"Standard": (240, 5), "Economic": (255, 5)}

#: rated reflow peak per JLC code: (peak C, extra limit text, label, source file:line)
REFLOW = {
    "C2833309": (260, "peak 260 +-0 C for 4..6.3 mm cans; reflow ONE time (twice only with 30 min between)", "F",
                 f"{DS.as_posix()}/C2833309_text/C2833309.txt:408-411 (table) and :369-370 (one reflow)"),
    "C2286": (260, "260 C for 10 s; reflow twice at most", "F", f"{DS.as_posix()}/C2286_text/C2286.txt:74, :241"),
    "C53426685": (260, "max 260 C, 10 s at max, reflow 2x max - series datasheet of FXL0420-1R0-M (C167203); the "
                       "-MA1 variant is assumed to share it (A)", "F/A",
                  f"{DS.as_posix()}/FXL0420_C167203/FXL0420_C167203.txt:1870-1876"),
    "C82891": (None, "the ESP-12F datasheet gives its reflow profile only as a graphic (section 4); no number read",
               "NOT_VERIFIED", f"{DS.as_posix()}/C82891_text/C82891.txt:271"),
    "C161692": (None, "JST GH catalogue text carries no reflow rating (only -40..+105 C operating)", "NOT_VERIFIED",
                f"{DS.as_posix()}/jst_eGH_png/p0.txt:21"),
}
#: refs of the drawing check and the datasheet item names of fp_vs_ds.json that are re-measured here
CONNECTORS = {"J14": "C5364642", "J15": "C131337", "J17": "C131337", "J7": "C161692", "U7": "C82891"}
PITCH_ITEMS = ("pitch", "signal pitch", "side pad pitch")
HOLE_ITEMS = ("hole dia (0.7 +0.1/-0)", "hole dia")


def reflow_rating(spec: dict, gate: dict | None) -> dict:
    import fitsets

    lines = fitsets.bom_groups(spec["parts"], with_mcu=True)
    mode = {x["jlc_code"]: x.get("assembly_mode") for x in (gate or {}).get("lines", [])}
    mode.update({x["jlc_code"]: "smtWeld" for x in (gate or {}).get("with_mcu_extra_lines", [])})
    plan_list = {"C2833309", "C82891", "C161692", "C131337", "C5364642", "C32713261", "C32713262", "C32713263",
                 "C22436147"}
    rows = []
    for g in lines:
        code = g["jlc_code"]
        am = mode.get(code)
        row = {"jlc_code": code, "designators": g["designators"], "comment": g["comment"], "assembly": g["assembly"],
               "assembly_mode_today": am, "in_plan_list": code in plan_list}
        if g["assembly"] == "THT" or am == "manualWeld":
            row.update({"result": "not reflowed",
                        "why": "THT line, assemblyMode manualWeld in the gate record: JLC hand-solders it "
                               "(F scout/jlc_rules.md:27, :155)"})
        elif code in REFLOW:
            peak, text, label, src = REFLOW[code]
            row.update({"rated_peak_C": peak, "limits": text, "label": label, "source": src})
            if peak is None:
                row["result"] = "NOT_VERIFIED"
            else:
                for kind, (t, tol) in OVEN.items():
                    worst = t + tol
                    row[kind] = ("OK" if peak > worst else "AT THE LIMIT (no margin)" if peak == worst
                                 else f"EXCEEDS: oven up to {worst} C > rated {peak} C")
                row["result"] = "OK" if all(row[k] == "OK" for k in OVEN) else "see per order type"
        else:
            row.update({"result": "NOT_VERIFIED", "why": "no rating read for this part in this step"
                        + (" (not in the plan's list)" if code not in plan_list else "")})
        rows.append(row)
    return {"oven_C": {k: f"{t} +-{tol}" for k, (t, tol) in OVEN.items()},
            "oven_source": "F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-"
                           "c3b11e27401e/scratchpad/pcb_rev3/scout/jlc_rules.md:125",
            "rows": rows,
            "not_verified": [r["jlc_code"] for r in rows if r["result"] == "NOT_VERIFIED"],
            "exceeds_or_at_limit": [f"{r['jlc_code']} {k}: {r[k]}" for r in rows for k in OVEN
                                    if isinstance(r.get(k), str) and r[k] != "OK"]}


def _lib_footprint(fpid: str):
    lib, name = fpid.split(":", 1)
    return pcbnew.FootprintLoad(str(KICAD_FP / f"{lib}.pretty"), name)


def _pads_local(fp) -> list[tuple]:
    out = []
    for p in fp.Pads():
        pos = p.GetFPRelativePosition() if fp.GetBoard() is not None else p.GetPosition()
        out.append((p.GetNumber(), round(pcbnew.ToMM(pos.x), 4), round(pcbnew.ToMM(pos.y), 4),
                    round(pcbnew.ToMM(p.GetSize(pcbnew.F_Cu).x), 4), round(pcbnew.ToMM(p.GetSize(pcbnew.F_Cu).y), 4),
                    round(pcbnew.ToMM(p.GetDrillSize().x), 4)))
    return sorted(out)


def _ee_pads(code: str) -> list[dict] | None:
    f = SCR / "jlc_cache" / f"easyeda_{code}.json"
    if not f.is_file():
        return None
    rec = json.loads(f.read_text(encoding="utf-8"))
    out = []
    for sh in rec["response"]["result"]["packageDetail"]["dataStr"]["shape"]:
        t = sh.split("~")
        if t[0] == "PAD":
            hole_r = float(t[9]) if t[9] else 0.0
            out.append({"n": t[8], "x": float(t[2]) * 0.254, "y": float(t[3]) * 0.254, "drill": round(2 * hole_r * 0.254, 3)})
    return out


def _pitch(pads: list[tuple[str, float, float]], a: str, b: str) -> float | None:
    pa = [p for p in pads if p[0] == a]
    pb = [p for p in pads if p[0] == b]
    if not pa or not pb:
        return None
    return round(math.hypot(pa[0][1] - pb[0][1], pa[0][2] - pb[0][2]), 3)


def connector_drawings(board) -> dict:
    fpv = json.loads((SCR / "design" / "conn_work" / "fp_vs_ds.json").read_text(encoding="utf-8"))
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    rows, errors = [], []
    for ref, code in CONNECTORS.items():
        fp = fps[ref]
        fpid = f"{fp.GetFPID().GetLibNickname()}:{fp.GetFPID().GetLibItemName()}"
        lib = _lib_footprint(fpid)
        same = _pads_local(fp) == _pads_local(lib) if lib else False
        if not same:
            errors.append(f"{ref}: board footprint pads differ from the library footprint {fpid}")
        bp = [(p[0], p[1], p[2]) for p in _pads_local(fp)]
        drills = sorted({p[5] for p in _pads_local(fp) if p[5] > 0})
        pitch = _pitch(bp, "1", "2") if ref != "U7" else _pitch(bp, "1", "2")
        ds_rows = [x for x in fpv["footprint_checks"] if x["part"] == code]
        checks = []
        for x in ds_rows:
            item = x["item"]
            if item in PITCH_ITEMS:
                got, kind = pitch, "pitch pad 1 - pad 2 re-measured on the board (M)"
            elif item in HOLE_ITEMS:
                got, kind = (drills[0] if drills else None), "drill re-measured on the board (M)"
            else:
                got, kind = x["kicad"], "library footprint value of the design step (board == library, M)"
            ds = x["datasheet"]
            if ds is None:
                res = "NOT_VERIFIED"
            elif got is None:
                res = "NOT_MEASURED"
            else:
                tol = x.get("tol") or 0.0
                mode = x.get("mode", "eq")
                ok = (abs(got - ds) <= tol + 1e-6 if mode == "eq" else got >= ds - 1e-6 if mode == "min"
                      else got <= ds + 1e-6)
                res = "MATCH" if ok else "MISMATCH"
                if not ok:
                    errors.append(f"{ref} {item}: board {got} vs datasheet {ds} ({x['source']})")
            checks.append({"item": item, "datasheet": ds, "board": got, "how": kind, "result": res,
                           "source": x["source"]})
        ee = _ee_pads(code)
        ee_cmp = None
        if ee:
            ep = [(e["n"], e["x"], e["y"]) for e in ee]
            ee_pitch = _pitch(ep, "1", "2")
            ee_drills = sorted({e["drill"] for e in ee if e["drill"] > 0})
            ee_cmp = {"pads": len(ee), "board_pads": len(bp), "pitch_1_2": ee_pitch, "drills": ee_drills,
                      "pitch_equal": ee_pitch is not None and pitch is not None and abs(ee_pitch - pitch) <= 0.02,
                      "drill_board": drills,
                      "note": "JLC's EasyEDA footprint of the same code (library data, not a manufacturer drawing); "
                              "differences are reported, they are not errors"}
        rows.append({"ref": ref, "jlc_code": code, "footprint": fpid, "board_equals_library": same,
                     "pitch_1_2_board_mm": pitch, "drills_board_mm": drills, "datasheet_checks": checks,
                     "easyeda_record": ee_cmp,
                     "not_verified": [c["item"] for c in checks if c["result"] in ("NOT_VERIFIED", "NOT_MEASURED")]})
    return {"rows": rows, "errors": errors,
            "source_of_datasheet_values": (SCR / "design" / "conn_work" / "fp_vs_ds.json").as_posix(),
            "label": "board geometry M; datasheet values F as cited per row; J14 hole / tail: Molex drawing not read"}
